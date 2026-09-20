# 配置文件参考

Klipper 配置文件采用类 INI 格式，用于定义打印机硬件配置、运动学参数以及各种外设选项。

## 文件格式说明

### 基本语法

配置文件由以下元素组成：

| 元素 | 语法 | 示例 |
|------|------|------|
| 节（Section） | `[type]` 或 `[type name]` | `[mcu]`、`[mcu zboard]`、`[stepper_x]` |
| 参数（Parameter） | `key: value` | `serial: /dev/ttyACM0` |
| 注释（Comment） | `# ...` | `# 这是注释` |
| 多行值（Multiline） | 缩进延续 | 见下方示例 |

### 节（Section）

每个节以 `[` 开头，`]` 结尾，定义一个配置块。节由**类型**和可选的**名称**组成：

```ini
[mcu]
serial: /dev/ttyACM0
baud: 250000

[mcu zboard]
serial: /dev/serial/by-path/xxx

[printer]
kinematics: cartesian
max_velocity: 500
```

**命名规则：**
- 格式为 `[type]` 或 `[type name]`
- `type` 是节的类型（如 `mcu`、`printer`、`stepper_x`）
- `name` 是可选的名称，用空格与类型分隔
- 没有名称时为**匿名节**（如 `[mcu]`）
- 有名称时为**命名节**（如 `[mcu zboard]`）
- 同一类型可以出现多次（如多个 `[mcu]` 对应多个 MCU 设备）

### 参数（Parameter）

参数格式为 `key: value`，冒号后跟一个空格和值。

```ini
[mcu]
serial: /dev/serial/by-path/xxx
baud: 250000
restart_method: arduino
```

### 注释（Comment）

以 `#` 开头的行为注释，不会被解析。

```ini
[mcu]
serial: /dev/ttyACM0
# baud: 250000    ; 这行被注释掉
baud: 250000      ; 行内注释也会被去除
```

行内注释（行尾的 `#`）也会被移除，但引号内的 `#` 不受影响。

### 多行值（Multiline Value）

当参数值为空时，后续缩进（空格或制表符）的行将作为多行值的内容。

```ini
[bed_tilt]
points:
    100, 100
    10, 10
    10, 100
```

在此示例中，`points` 参数的值为三行内容的数组。

### 空节（Empty Section）

节可以没有参数，仅用于声明存在。

```ini
[pause_resume]

[exclude_object]
```

## MCU 连接方式

`[mcu]` 节用**一个**连接参数说明主机怎么和这颗 MCU 通信——和 Klipper 一样（Klipper 用 `serial` 或 `canbus_uuid`）。同时给出多个是配置错误，会直接报错，而不是按某个优先级挑一个。

| 参数 | 值 | 说明 |
|------|-----|------|
| `serial` | 串口设备路径（如 `/dev/ttyACM0`） | 打开真实 MCU 所在的串口；可配 `baud`（默认 250000） |
| `canbus_uuid` | MCU 芯片的唯一 ID（12 位十六进制） | 通过 CAN 连接时必给，Klipper 同名参数 |
| `canbus_interface` | CAN 网络接口名 | 可选，默认 `can0` |
| `canbus_nodeid` | CAN 节点号（1..895） | klipperx 需要显式给出（Klipper 由 `[canbus_ids]` 分配）；配 `canbus_uuid` 一起用 |
| `host_library` | klipper host 库（`libklipper_host.so`）的路径 | 在主机进程内运行一份 klipper 固件（模拟与自测用） |
| `test` | 每行 `输入帧 输出帧...`（十六进制） | 仅测试构建可用，供单元测试脚本化一个假设备 |

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

[mcu fake]
test: 06 10 05 00 00 7e
```

节名取自节的 sub（`[mcu zboard]` → `zboard`）。串口按 raw 模式打开：不回显、不做 CR/LF 转换、不启用协议没用到的流控。`restart_method` 见下面的[固件重启方式](#固件重启方式restart_method)。

CAN 这条走的是 klipper 的 **can serial** 链路（就像把串口协议搬到 CAN 上跑），不是 CAN 协议本身。**配置里的键名与 Klipper 保持一致**（`canbus_uuid` / `canbus_interface`），因为它们描述的是机器怎么接线，而且现有 Klipper 配置应当照用；代码里那层实现则叫 `CanSerial*`，把 `Canbus` 这个名字留给将来真正实现 CAN 协议的接口。

节点号到仲裁 ID 的映射也沿用 Klipper：`0x100 + 2 × nodeid`，主机写这个 ID、MCU 用下一个回；节点号由主机通过 admin ID `0x3f0` 的 `CMD_SET_NODEID` 报文指派，所以配置里 `canbus_uuid` 是必需的。

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
- `arduino` / `cheetah` 的时序要在真板子上才能验证，主机执行时会打一条“未在真板上测过”的告警；`rpi_usb` 已在硬件上验证过。

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

`auto` 的“能用”是运行期探出来的：sysfs 的 `disable` 文件存在且**当前进程可写**。所以装了仓库那份 udev 规则（只给 `/dev/bus/usb` 节点权限）时，`auto` 通常走控制传输；想让它走 sysfs，还得加上面 udev 小节里那段 `RUN+=` chmod。

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
hub 只出一份。

装完确认该 hub 节点对运行 klipperx 的用户可写：

```bash
ls -l /dev/bus/usb/001/001
```

#### 匹配的是 hub，不是 MCU

规则里的 `idVendor`/`idProduct` 必须是 **USB hub** 的，不是 MCU 的。用 `lsusb` 很容易先看到 MCU 那行——Klipper 固件给它自己的设备 id 是 `1d50:614e`（串口）、`1d50:606f`（CAN）——**把它写进规则没有用**，因为控制传输发给 hub。

而 hub 能不能切电，**与 id 无关**：

- 能力写在 hub 描述符的 `wHubCharacteristics` 低两位：`0x0008` = ganged（整组一起）、`0x0009` = per-port（逐口）、`0x000a` = 不支持。用 `lsusb -v` 看 `wHubCharacteristic`，或直接跑 `uhubctl`（它按这个能力筛）。
- 所以**没有“按 id 判断”的规律**——同一个型号不同批次都可能不同（uhubctl 清单里就有 “rev A,C,F 不支持” 这种注记）。
- 唯一稳的规律是**根 hub**：`1d6b:0001`(1.1) / `1d6b:0002`(2.0) / `1d6b:0003`(3.0)，那是内核给根 hub 的 id。板载 hub 常在其中的，但同样要看能力，不能只看 id。

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

- **路径**：从 hub 的接口目录（`<hub>:<cfg>.<if>`）出发，glob `*port<N>/disable`（uhubctl 的规则也用 `$sys$devpath/*port*/disable`）。注意 `/sys/bus/usb/devices/*` 是**符号链接**，`find` 默认不跟进去，要看真实路径 `/sys/devices/...`。
- **存在 ≠ 能写**：文件默认是 `-rw-r--r-- root:root`，普通用户写不了。而且 udev 的 `MODE=`/`GROUP=` 只作用于 `/dev` 设备节点，**改不了 sysfs 属性**——要给权限必须在 `RUN+=` 里 `chown`/`chmod`。uhubctl 给 6.0+ 的规则就是这段：

  ```udev
  SUBSYSTEM=="usb", DRIVER=="hub|usb", \
    RUN+="/bin/sh -c \"chown -f root:dialout $sys$devpath/*port*/disable || true\"", \
    RUN+="/bin/sh -c \"chmod -f 660 $sys$devpath/*port*/disable || true\""
  ```

- **存在 ≠ 真断电**：文件对**所有**端口都存在，是否真断 VBUS 取决于 hub 的能力（`wHubCharacteristics` 低两位，见上）。

klipperx 两条都实现了（`src/core/klippy/interface/usb.rs`），由 `usb_power` 选择：`auto`
（缺省）能用 sysfs 就用、否则回落控制传输；`libusb` 强制控制传输，用于绕开某个 hub 卡住的
sysfs 行为。脚本输出的两条规则正好一条给一条，装上就两条路都通。

#### 树莓派 5 的坑

Pi 5 的 4 个板载 hub **都报告**支持 per-port power switching，但实际 4 个口是 **ganged** 成一组：
只切某一个端口切不掉 VBUS，要同时对所有板载 hub/port 下手（uhubctl 的说法是 `-l 2 -a 0` 加
`-l 4 -a 0`）。这种板子上 `rpi_usb` 可能达不到预期，需要按实际硬件确认。

`authorized` / unbind 也能让设备重新枚举，但那不是真正断电，不能当 `rpi_usb` 用。

## 示例配置

以下是一个完整的示例：

```ini
[mcu]
serial: /dev/serial/by-path/platform-3f980000.usb-usb-0:1.2:1.0-port0
baud: 250000

[mcu zboard]
serial: /dev/serial/by-path/platform-3f980000.usb-usb-0:1.3:1.0-port0

[mcu auxboard]
serial: /dev/serial/by-path/platform-3f980000.usb-usb-0:1.4:1.0-port0

[stepper_z]
step_pin: zboard:PL3
dir_pin: zboard:PL1
enable_pin: !zboard:PK0

[extruder]
step_pin: auxboard:PA4
heater_pin: auxboard:PB4
min_temp: 0
max_temp: 250

[printer]
kinematics: cartesian
max_velocity: 500
max_accel: 3000
```

---

- [← 用户手册首页](README.md)
