# MCU 连接方式

`[mcu]` 节用**一个**连接参数说明主机怎么和这颗 MCU 通信——和 Klipper 一样（Klipper 用 `serial` 或 `canbus_uuid`）。同时给出多个是配置错误，会直接报错，而不是按某个优先级挑一个。

| 参数 | 值 | 说明 |
|------|-----|------|
| `serial` | 串口设备路径（如 `/dev/ttyACM0`） | 打开真实 MCU 所在的串口；可配 `baud`（默认 250000） |
| `canbus_uuid` | MCU 芯片的唯一 ID（12 位十六进制） | 通过 CAN 连接时必给，Klipper 同名参数 |
| `canbus_interface` | CAN 网络接口名 | 可选，默认 `can0` |
| `canbus_nodeid` | **MCU 层 id**（1..255；CAN 上即节点号） | **可选**：不写就由 `[mcu_ids]`（历史名 `[canbus_ids]`）的分配器发 `4` 起的最小空位；写了就用写的那个（分配器装载时预扫全配置保留显式号，自动号绕开，写在哪个位置都生效）。只在两个显式号互撞、或 `4..=255` 占满时装载期报错 |
| `host_library` | klipper host 库（`libklipper_host.so`）的路径 | 在主机进程内运行一份 klipper 固件（模拟与自测用） |

```ini
[mcu]
serial: /dev/serial/by-path/platform-3f980000.usb-usb-0:1.3:1.0-port0
baud: 250000

[mcu canbed]
canbus_uuid: 11aa22bb33cc
canbus_interface: can0
canbus_nodeid: 2

[mcu simulated]
host_library: /usr/local/lib/libklipper_host.so
```

节名取自节的 sub（`[mcu zboard]` → `zboard`）。串口按 raw 模式打开：不回显、不做 CR/LF 转换、不启用协议没用到的流控。`restart_method` 见下面的[固件重启方式](#固件重启方式restart_method)。

CAN 这条走的是 klipper 的 **can serial** 链路（就像把串口协议搬到 CAN 上跑），不是 CAN 协议本身。**配置里的键名与 Klipper 保持一致**（`canbus_uuid` / `canbus_interface`），因为它们描述的是机器怎么接线，而且现有 Klipper 配置应当照用；代码里那层实现则叫 `CanSerial*`，把 `Canbus` 这个名字留给将来真正实现 CAN 协议的接口。

节点号到仲裁 ID 的映射也沿用 Klipper：`0x100 + 2 × nodeid`，主机写这个 ID、MCU 用下一个回；节点号由主机通过 admin ID `0x3f0` 的 `CMD_SET_NODEID` 报文指派，所以配置里 `canbus_uuid` 是必需的（节点号本身可由 `[mcu_ids]` 分配、不必写；历史拼写 `[canbus_ids]` 仍兼容）。

节点号在本仓是 **MCU 层 id**：每个 `[mcu]`（不论接口）在装载期都被分配一个唯一 id（所有接口共用同一个序列，`mcu/ids.rs`），`connect` 打开传输层之前必须持有它——CAN 用它做节点寻址（admin 帧），串口用下节的**帧级指派**把 id 交给设备；`host_library` 与 `test:` 暂不指派。显式 `canbus_nodeid` 就是这个 id 的显式覆盖：分配器
在装载时预扫全配置、先把所有显式号保留下来（`reserve_explicit`），自动号取 `4` 起的
**最小空位**并绕开它们——所以显式号写在哪个位置都生效，发号结果与声明顺序无关。

## 帧级 id 指派（identify 之前）

串口没有 CAN 那样的总线旁道，指派就在**帧格式里开一条旧固件会默认抛弃的信道**：与 admin 帧同位（端口一开、identify 之前），完全不走 msg 体系（不依赖字典、不是命令）。

```text
[len][0x20]["K" "X" "I" "D"][十进制 id][crc16 hi][crc16 lo][0x7e]
```

与常规帧唯一不同的是 **seq 字节**：`0x20` 把合法帧必须置的 `MESSAGE_DEST` 位清掉、改用 bit5；载荷与头部绝不含 `0x7e`（CRC 字节是算出来的、可能恰好是——id 81 就会，固件此时分两步丢弃，同样无害）。其余（长度范围、尾部 SYNC、CRC）都是标准帧结构，有能力的固件无需新校验就能验证。

**旧固件的行为**（`command_find_block`，`third_party/klipper/src/command.c`，由 `mcu/assign.rs` 的可执行移植测试钉住）：长度检查通过 → **seq 字节检查（:287）不过 → 错误分支**：不读载荷、不查 CRC、永不 dispatch、`next_sequence` 不动；丢弃恰好吞下本帧（到帧尾 `0x7e` 为止），至多回一枚**无害空帧**（seq 是它自己的 `next_sequence`，主机读作「无进展的 ack」），此后静默（`CF_NEED_VALID` 抑制重复报错）。夹在它后面的 identify 帧照常解析、照常执行。

**有能力解析的固件**按同形反向回执 `[len][0x20]["KXID"][id][crc][0x7e]`——旧固件发不出这种帧（`command_add_frame` 的 seq 恒在 `0x10..=0x1f`）。主机在 `FrameStream`（`frame.rs`）认出它而不是丢弃，接收任务把 id 记到 `Mcu::assignment_confirmed()`——这是将来「有 id 就加帧头」的寻址头会读的门控钩子。

**边界**：发送是 fire-and-forget——不等回执、不增加连接延迟，发送失败只记 warn 不挡连接；`host_library` 与 `test:` 不发（后者脚本帧会拒绝未映射的发送）；CAN 保留自己的 admin 帧。**本轮帧格式对所有设备都是当前格式**：指派只多发一帧，不改任何已有帧；帧头寻址**未落地**。识别在帧重组处发生，有两个边界：回执若前粘一段**不含 `0x7e` 的坏字节**，可能与回执被一并吞掉（该回执丢失、`assignment_confirmed` 不置位；噪声自带 `0x7e` 则无碍）；回执也可能要等**断连才被排空**（接收任务在关闭路径上同样排空，已有测试钉住）——今天无消费方，两者仅作记录。

## 固件重启方式（`restart_method`）

系统重启时，主机需要决定怎么把 MCU 的**固件**复位。`restart_method` 就是这件事，写在 `[mcu]` 节里。它**只对串口 MCU 有意义**：CAN 与 `host_library` 一律按 `command` 处理，配置里写了也会被忽略（并在日志里告警）。

| 值 | 做法 | 说明 |
|----|------|------|
| `command` | 连上之后用固件的 `config_reset` 清掉旧配置，再重新配置 | 非串口的固定做法；串口固件实现了 `config_reset` 时也可用 |
| `arduino` | 以 2400 波特打开串口，翻转 DTR | 串口**缺省**值；Arduino 及克隆板 |
| `cheetah` | 以 2400 波特做一串 RTS/DTR 时序 | Fysetc Cheetah v1.2 等采用状态机 bootloader 电路的板子 |
| `rpi_usb` | 切断再恢复该 USB 端口的供电 | 树莓派等能给 USB 口断电的机器 |

```ini
[mcu]
serial: /dev/ttyACM0
restart_method: arduino
```

- 值拼错会直接报配置错误，不会静默退回默认。
- 不写时串口默认 `arduino`。
- `arduino` / `cheetah` 的时序要在真板子上才能验证，主机执行时会打一条"未在真板上测过"的告警；`rpi_usb` 已在硬件上验证过。

### `usb_power`：`rpi_usb` 用哪种机制切电

`rpi_usb` 有两条路，用 `usb_power` 选（只对 `restart_method: rpi_usb` 有意义，别的重启方式下给了会被忽略并告警）：

| 值 | 做法 | 适合 |
|----|------|------|
| `auto`（缺省） | 能用 sysfs 就用 sysfs，否则用 hub 控制传输 | 大多数情况 |
| `sysfs` | 只写 `/sys/.../<portN>/disable` | 内核 ≥ 6.0、且该属性可写 |
| `libusb` | 只发 hub 的端口供电控制传输（`hub-ctrl` 那条） | 想绕开某个卡住的 hub 的 sysfs 行为 |

```ini
[mcu]
serial: /dev/ttyACM0
restart_method: rpi_usb
usb_power: libusb
```

`auto` 的"能用"是运行期探出来的：sysfs 的 `disable` 文件存在且**当前进程可写**。所以装了仓库那份 udev 规则（只给 `/dev/bus/usb` 节点权限）时，`auto` 通常走控制传输；想让它走 sysfs，还得加上面 udev 小节里那段 `RUN+=` chmod。

**初始化时会探一次权限**：只要配了 `restart_method: rpi_usb`，MCU 上线时就会按 `usb_power` 试一次；如果两条路都不可用，会打一条告警，并直接给出该 hub 对应的 udev 规则（含从 sysfs 读到的 hub 的 `idVendor`/`idProduct`），不必等到第一次 `FIRMWARE_RESTART` 才发现。

### `rpi_usb` 需要的 udev 规则

`rpi_usb` 要打开该 USB hub 的节点、或写它的 sysfs 端口开关来切电，两者默认都只有 root 能做。规则必须针对**你那颗 hub**，所以仓库给了个脚本，按串口设备生成并安装：

```bash
# 不带参数：按固件的 USB id（`1d50:614e` 串口 / `1d50:606f` CAN）找所有 Klipper 设备，
# 再找它们背后的 hub
scripts/klipperx-usb-udev.sh

# 也可以直接给串口设备（固件改了 id、或有别的来源时）
scripts/klipperx-usb-udev.sh /dev/ttyACM0 /dev/ttyACM1

# 安装：脚本用 sudo 写到 /etc/udev/rules.d/52-klipperx-usb.rules 并重载 udev
scripts/klipperx-usb-udev.sh --install
```

那对 id 就是固件给自己报的（`src/Kconfig` 的 `USB_VENDOR_ID`/`USB_DEVICE_ID` 缺省
`0x1d50`/`0x614e`，CAN 固件写死 `1d50:606f`）；自编改了 id 的话给设备路径，或设
`KLIPPERX_USB_IDS=vendor:product …`。

脚本从每个设备的 sysfs 拓扑找到背后的 hub，按 hub 的 `idVendor`/`idProduct` 各出两条规则
（一条管 `/dev/bus/usb` 节点，给控制传输；一条 `RUN+=` chmod，给 sysfs 端口开关），同一颗
hub 只出一份。规则上面还会写一段这颗 hub 的**端口供电能力**（就是上面那个
`wHubCharacteristics`）：`per-port` 是能用的那种，`ganged` 会连带整颗 hub 的口，
`no power switching` 则直接告诉你这套规则在这颗 hub 上达不到目的——只断开、不下电，请改用
`restart_method: command` 或给板子真断电（同一句也会打到 stderr）。读不到描述符时（普通用户
打不开 hub 的 `/dev/bus/usb` 节点，`lsusb` 也读不了）就写"unknown"并给出确认命令：
`sudo lsusb -v -s <bus>:<dev> | grep -i -A2 wHubCharacteristic`。

装完确认该 hub 节点对运行 klipperx 的用户可写：

```bash
ls -l /dev/bus/usb/001/001
```

#### 匹配的是 hub，不是 MCU

规则里的 `idVendor`/`idProduct` 必须是 **USB hub** 的，不是 MCU 的。用 `lsusb` 很容易先看到 MCU 那行——Klipper 固件给它自己的设备 id 是 `1d50:614e`（串口）、`1d50:606f`（CAN）——**把它写进规则没有用**，因为控制传输发给 hub。

多层 hub（扩展坞串扩展坞那种）时取的是**紧邻 MCU 的那一颗**：脚本与 `resolve_tty_port` 都只从设备目录上溯一级（`dirname` / `parent()`）。端口供电开关在父 hub 的端口上，控制传输也发给父 hub，所以只有这一层对；根 hub 与中间层 hub 都不进规则，也不会被授权。反过来，规则只按 `vid:pid` 认 hub，所以串/并挂了几颗**同型号** hub 时一条规则会同时命中它们（授权范围比预期大），也没法只针对其中某一颗。

而 hub 能不能切电，**与 id 无关**：

- 能力写在 hub 描述符的 `wHubCharacteristics` 低两位：`0x0008` = ganged（整组一起）、`0x0009` = per-port（逐口）、`0x000a` = 不支持。用 `lsusb -v` 看 `wHubCharacteristic`，或直接跑 `uhubctl`（它按这个能力筛）。
- 所以**没有"按 id 判断"的规律**——同一个型号不同批次都可能不同（uhubctl 清单里就有 "rev A,C,F 不支持" 这种注记）。
- 唯一稳的规律是**根 hub**：`1d6b:0001`(1.1) / `1d6b:0002`(2.0) / `1d6b:0003`(3.0)，那是内核给根 hub 的 id。板载 hub 常在其中的，但同样要看能力，不能只看 id。

MCU **直接插在根 hub 上**（机器面板上的普通 USB 口）也支持：控制传输照样发给根 hub。但根 hub 大多数是 `lpsm=2`（不支持供电切换——实测 AMD xHCI 报 `wHubCharacteristics=0x000a`），`CLEAR_FEATURE(PORT_POWER)` 只会让这个设备**断开并重新枚举**（`devnum` 变了、`/dev/ttyACM*` 与 sysfs 都在，内核在端口关着时不拆设备），VBUS 其实没断：板子固件一路照跑，它的 4 位协议序号接着上一条会话往下走，重连的主机从 0 开始就对不上（`Seq mismatch` → identify 超时 → `state: shutdown`）。这种机器上 `rpi_usb` 达不到目的，只能真的断电重来（拔 USB，或按板子的复位；板子由自己电源供电时拔 USB 连断电都算不上）。sysfs 那条 `disable` 走的是同一个端口开关，结论一样。

**开机时先问 hub 自己**：`rpi_usb` 一被配置上（每次连 MCU 时）klipperx 就读一次这颗 hub 的
`wHubCharacteristics`（`GET_DESCRIPTOR` 要 hub 自己的类描述符，sysfs 里没有）。低两位说
`no power switching` 的话，这台的端口开关**只是断开、切不掉 VBUS**，于是当场告警并把这个 MCU
的 `restart_method` 在内存里改成 `command`——连那一次白断开都不做。脚本也按 hub 报同一件事
（读不到时给出确认命令，见下），装规则前就能看出来。

**每次 `rpi_usb` 复位之后 klipperx 还会再验一遍**这个前提：重连上来的固件必须是刚开机的
（它自己报的会话序号从头开始，而且还没有配置——配置在 RAM 里，重启就没了）。如果它从
上一条会话接着答，或者还带着上次的配置，那就说明这次只断开了、没复位，于是：

- 打**一条**告警，说清看到的是什么，并提示只有真断电才能复位；
- 把这个 MCU 的 `restart_method` 在**内存里**改成 `command`（重新走固件的 `config_reset`），
  后续的复位不再去切那个不起作用的端口。`printer.cfg` 不动——改文件是人的事，重启进程也
  还会按文件来。

  注意 `command` 是把配置清掉重发，**不是**把 MCU 复位；而它得先连上才能发。板子若还连着、
  还在跑（这次的断电没真断），它自己的协议序号会接着上一条会话走；传输层会**接管**这块
  仍在运行的固件（采纳它的序号、重发未确认的块，`Mcu::took_over_session`），所以 `command` 不必先断电
  也能接上；RTO 定时重传（`Sender::retransmit`）也已就位：**25 ms 只是无样本时的初值与地板**，拿到样本后按实测估计值布防（仍夹 25 ms–5 s），超时翻倍上限 5 s 不变。链路往返同时被持续采样（样本 = 块写出完成→ack，平滑后夹在 25 ms–5 s）：**实测 RTT 首次超过 25 ms 时日志会 `warn`**（每会话首超阈一次、之后翻倍再告一次），提示检查串口桥延迟（如 FTDI `latency_timer`）、波特率与线缆——看到它先查链路，而不是查固件。

`lsusb` 里常见的 hub 厂商 id：

| 厂商 id | 厂商 | 例 |
|---------|------|----|
| `1d6b` | Linux Foundation（根 hub） | `1d6b:0002` |
| `2109` | VIA Labs | `2109:2811` / `2813` / `2817` |
| `05e3` | Genesys Logic | `05e3:0607` / `0608` / `0610` |
| `0424` | Microchip / SMSC | `0424:2744`（BeagleBone 内部 hub 是 `0424:9514`） |
| `04b4` | Cypress | `04b4:6570` |
| `0451` | TI | `0451:2046` |
| `05ac` | Apple（内置 hub） | `05ac:9139` |

完整的实测清单在 [uhubctl 的 README](https://github.com/mvp/uhubctl#compatible-usb-hubs)。最省事的做法是先 `sudo uhubctl`（或 `sudo lsusb -v`）确认哪个 hub 支持 per-port power switching，再照它的 id 写规则。

脚本读的是 hub 在 sysfs 里自报的 id，不用自己填。可能想改的两处：

- **klipperx 作为服务运行**（没有本地会话，`uaccess` 不生效）：把脚本输出里的
  `TAG+="uaccess"` 换成固定用户组，例如 `GROUP="klipper", MODE="0660"`。
- **sysfs 那条 `RUN+=`** 用的是 `chmod 660` 且不改属组；要让某个组可写，加一个 `chgrp <组>`。

规则文件名以 `52-` 开头是有意的：`uaccess` 这个标签由 systemd 的
`/usr/lib/udev/rules.d/73-seat-late.rules` 消费，设置它的规则必须排在它**之前**（数字更小）才生效。

#### 内核 ≥ 6.0 的 sysfs 端口开关

Linux **6.0 起**有了标准的 sysfs 端口开关。它在 hub 的接口目录下，但**目录名随内核变过**，不要硬编码：

```bash
# ABI 文档写的是 port<X>，但 7.2 内核上实际是 <hub>-port<N>：
ls /sys/bus/usb/devices/1-2/1-2:1.0/1-2-port1/disable
```

写 `1` / `0` 即关/开该端口，在支持供电切换的 hub 上会一并断掉 VBUS；`uhubctl` 在新内核上就优先用它，旧内核才回落到 libusb。

**要判断它可不可用，别看内核版本号，直接探这个文件：**

- **路径**：从 hub 的接口目录（`<hub>:<cfg>.<if>`）出发，glob `<hub>-port<N>/disable`（老内核是 `port<N>`，`disable_path` 两种都认）。注意 `/sys/bus/usb/devices/*` 是**符号链接**，`find` 默认不跟进去，要看真实路径 `/sys/devices/...`。
- **存在 ≠ 能写**：文件默认是 `-rw-r--r-- root:root`，普通用户写不了。而且 udev 的 `MODE=`/`GROUP=` 只作用于 `/dev` 设备节点，**改不了 sysfs 属性**——要给权限必须在 `RUN+=` 里 `chown`/`chmod`。脚本与告警生成的就是这两条：

  ```udev
  SUBSYSTEM=="usb", ATTR{idVendor}=="0424", ATTR{idProduct}=="2137", TAG+="uaccess"
  SUBSYSTEM=="usb", DRIVER=="hub|usb", ATTR{idVendor}=="0424", ATTR{idProduct}=="2137", \
    RUN+="/bin/sh -c \"chmod -f 660 $sys$devpath/*/*port*/disable || true\""
  ```

  glob 要比 hub **深一层**：`RUN+=` 只在 hub 的**设备**目录上触发（接口目录没有 `idVendor`，带 id 过滤的规则进不去），而端口目录在它的接口目录下。uhubctl 自己的规则字面上与这条相近，但它**不带 id 过滤**、会落到**每个** USB 设备上，于是在每个设备目录里匹配到的是它自己的 `port` 符号链接（正是该设备所占的端口）；照抄那个 glob 再加上 id 过滤，匹配到的就是 hub **上一层**喂它的那个口，根 hub 目录下更是一个都匹配不到（`chmod -f` 静默失败，sysfs 这条路就永远不可写）。

- **存在 ≠ 真断电**：文件对**所有**端口都存在，是否真断 VBUS 取决于 hub 的能力（`wHubCharacteristics` 低两位，见上）。

klipperx 两条都实现了（`src/core/klippy/interface/usb.rs`），由 `usb_power` 选择：`auto`
（缺省）能用 sysfs 就用、否则回落控制传输；`libusb` 强制控制传输，用于绕开某个 hub 卡住的
sysfs 行为。脚本输出的两条规则正好一条给一条，装上就两条路都通。

#### 树莓派 5 的坑

Pi 5 的 4 个板载 hub **都报告**支持 per-port power switching，但实际 4 个口是 **ganged** 成一组：
只切某一个端口切不掉 VBUS，要同时对所有板载 hub/port 下手（uhubctl 的说法是 `-l 2 -a 0` 加
`-l 4 -a 0`）。这种板子上 `rpi_usb` 可能达不到预期，需要按实际硬件确认。

`authorized` / unbind 也能让设备重新枚举，但那不是真正断电，不能当 `rpi_usb` 用。

---

- [← 配置文件参考](config.md)
