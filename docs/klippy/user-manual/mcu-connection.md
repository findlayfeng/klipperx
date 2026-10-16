# MCU 连接方式

`[mcu]` 节用**一个**连接参数说明主机怎么和这颗 MCU 通信——和 Klipper 一样（Klipper 用 `serial` 或 `canbus_uuid`；本仓把 CAN 的身份键规范名为 `uuid`，`canbus_uuid` 仍作兼容别名）。同时给出**接口**参数（`serial` / CAN 家族 / `host_library` / `test`）中多个是配置错误，会直接报错，而不是按某个优先级挑一个；`uuid`/`canbus_uuid` 是**身份**不是接口选择器——写在串口节上不构成冲突（见下节「共享串口」）。

| 参数 | 值 | 说明 |
|------|-----|------|
| `serial` | 串口设备路径（如 `/dev/ttyACM0`） | 打开真实 MCU 所在的串口；可配 `baud`（默认 250000）。**同一路径被两颗及以上 `[mcu]` 引用即共享**（见下节） |
| `uuid` | MCU 芯片的唯一 ID（12 位十六进制） | CAN 模式必给（Klipper 旧名 `canbus_uuid` 兼容，同节双写报错）；写在 `serial` 节上则连接期用它指派 id（见下节「共享串口」，独占串口忽略它） |
| `canbus_interface` | CAN 网络接口名 | 可选，默认 `can0` |
| `canbus_nodeid` | **MCU 层 id**（1..=255；CAN 上即节点号） | **可选**：不写就由 `[mcu_ids]`（历史名 `[canbus_ids]`）的分配器发 `4` 起的最小空位；写了就用写的那个（分配器装载时预扫全配置保留显式号，自动号绕开，写在哪个位置都生效）。只在两个显式号互撞、或 `4..=255` 占满时装载期报错 |
| `canbus_protocol` | CAN **传输层**：`cans`（默认，Klipper 的 can-serial 字节流）或 `isotp`（ISO 15765-2 分段，一条 Klipper 帧 = 一条 ISO-TP 消息） | 只在 CAN 节有意义（其余节写它会被判「第二个接口」）。按**设备类型**选路：支持 FD 的板子用 `cans`；标准 CAN 板子用 `isotp` 承载大于一帧的 Klipper 消息。两种模式**不叠加**。不写 = `cans`，与今天逐字节同行为 |
| `canbus_fd` | CAN **链路层**：`true` = CAN-FD（64 B/帧），默认 `false` = 经典 CAN（8 B/帧） | 与 `canbus_protocol` **正交**。已验收：`cans`+`true`、`isotp`+`false`；**`isotp`+`true` 合法但未验证**（解析期 `warn` 一次）。只在 CAN 节有意义 |
| `host_library` | klipper host 库（`libklipper_host.so`）的路径 | 在主机进程内运行一份 klipper 固件（模拟与自测用）；**同路径被多颗 `[mcu]` 引用时库只加载一次**（通讯复用，一个 runtime/reader），各拿一个通讯引用共享它，`shutdown` 只停本引用、最后一个引用析构才卸载；旧固件没有桥/没有 mux 时，同一时刻只能有一路活动引用（设备层在 `interface/devices/host.rs`）。**一进程只允许一份 host 库**：配置里出现**两个不同 `host_library:` 路径**会在装载期**点名报错**，装载期也有按路径的守卫兜底（原因是 klipper 通过**进程级** `SIGUSR1` handler 停主循环，每份库都会覆盖上一份的 handler ⇒ 两份库同进程会得到「跳进已卸载代码（崩溃）」或「join 等一个被别的库吃掉的信号（挂死）」）；要跑两个固件请用**同一份库的多引用（mux）**，或把它们放到**独立进程**（固件侧 `linux` 目标就是一个可独立运行的固件进程）。**可写 `uuid:`**（12 位十六进制，旧名 `canbus_uuid` 兼容，同节双写报错）：它指明这条路径上被寻址的是**哪块设备**。判定为 **mux** 的路径连接时**按 id 寻址**：先发 `SET_KLIPPER_NODEID`（节上写了 `uuid:` 就用它点名**桥下游那台板子**；没写就用**装载期为该路径指派的那条自身份**，它同时是主机在 `init` 前交给库的身份），再走重确认握手，之后收发都带帧头；非 mux 路径照旧裸帧。判定按配置事实（`mcu/host_libraries.rs`）：同路径 **1 节且无 `uuid:`、也不是桥** 走**裸帧**；**1 节有 `uuid:`**、**该节是桥**（含 0 或 ≥1 台下游设备）、或同路径 **≥2 节** 则**按 id 寻址**——≥2 节时**最多一节可省略 `uuid:`**（它认领桥自身的设备），其余必须写 `uuid:`；**≥2 节都省略**、或**两节写同一个 uuid** 都在装载期报错并点名涉及的节 |
| `uart` | 引用 `[uart_device <名>]` 端口节的名字 | 把这颗 MCU 挂到**桥的下游 uart 口**上（端口本身用 `[uart_device]` 节声明，见下节「桥下游端口」）。`uuid` 可选——不写即 legacy。与 `serial`/`host_library`/`test`/`can`/`canbus_interface` 互斥 |
| `can` | 引用 `[can_device <名>]` 端口节的名字 | 挂到桥的下游 CAN 口。**`uuid` 必填**（CAN 寻址靠它）。与 `serial`/`host_library`/`test`/`canbus_interface`/`uart` 互斥。**尚不可挂**：固件的 CAN **端口**后端（`BRIDGE_PORT_CAN`）还没实现，配置期点名拒绝（CAN **上游**的 mux 承载 `MUX_CAN` 已落地，与这条是两件事） |

```ini
[mcu]
serial: /dev/serial/by-path/platform-3f980000.usb-usb-0:1.3:1.0-port0
baud: 250000

[mcu canbed]
uuid: 11aa22bb33cc
canbus_interface: can0
canbus_nodeid: 2

[mcu simulated]
host_library: /usr/local/lib/libklipper_host.so
```

节名取自节的 sub（`[mcu zboard]` → `zboard`）。串口按 raw 模式打开：不回显、不做 CR/LF 转换、不启用协议没用到的流控。`restart_method` 见下面的[固件重启方式](#固件重启方式restart_method)。

CAN 这条走的是 klipper 的 **can serial** 链路（就像把串口协议搬到 CAN 上跑），不是 CAN 协议本身。**CAN 的键名沿用 Klipper**（`canbus_interface` / `canbus_nodeid` 照旧），身份键则规范为 `uuid`——`canbus_uuid` 仍兼容（现有 Klipper 配置照用，同节两个都写会报错），「机器怎么接线」的语义不变。代码里那层实现叫 `CanSerial*`，把 `Canbus` 这个名字留给将来真正实现 CAN 协议的接口。

**两个新键把「链路层」与「传输层」分开**（本仓扩展，上游无对应）：`canbus_fd` 选链路（经典 8 B/帧 / CAN-FD 64 B/帧），`canbus_protocol` 选传输（`cans` 的字节流切片 / `isotp` 的 ISO 15765-2 分段）。两者正交、四种组合都合法，但**已验收的只有两种**，对应两类设备：**支持 FD 的板子**用 `cans` + `canbus_fd: true`（一帧 64 B 直接装下上限 64 B 的 Klipper 消息，传输层保持现状），**标准 CAN 板子**用 `canbus_protocol: isotp`（8 B 帧装不下，靠 ISO-TP 承载）；`isotp` + `canbus_fd: true` 是保留可能性、解析期会 `warn` 一次「未验证」。不写这两个键就是今天的 `cans` + 经典 CAN，逐字节不变。

节点号到仲裁 ID 的映射也沿用 Klipper：`0x100 + 2 × nodeid`，主机写这个 ID、MCU 用下一个回；这个映射在主机侧单一定义于 `assign.rs`（CAN 与共享串口共用，`canserial.rs` 不再自带常量），打开 CAN 链路前会校验节点号落在 `1..=255`，越界即报错而不是拼出一个非法仲裁 ID。节点号由主机通过 admin ID `0x3f0` 的 `CMD_SET_NODEID` 报文指派，所以配置里 `uuid` 是必需的（节点号本身可由 `[mcu_ids]` 分配、不必写；历史拼写 `[canbus_ids]` 仍兼容）。

节点号在本仓是 **MCU 层 id**：每个 `[mcu]`（不论接口）在装载期都被分配一个唯一 id（所有接口共用同一个序列，`mcu/ids.rs`），`connect` 打开传输层之前必须持有它——CAN 用它做节点寻址（admin 帧），**共享串口**用下节的帧头与**分配期握手**把 id 交给设备、并拿它做寻址，**独占串口不使用它**；`host_library` 与 `test:` **同样领到 id、只是暂不上线**（id 是每个 `[mcu]` 的属性——`connect` 没有它就不开传输层；判定为 **mux** 的 host 库路径**连接时就按 id 寻址**：装载期指派身份、`init` 前投递给库、`SET_NODEID` + 重确认握手、之后收发带帧头——只有 `test:` 恒不用这个号）。显式 `canbus_nodeid` 就是这个 id 的显式覆盖（**仅 CAN 节**——写到 `serial`/`host_library`/`test` 节上会被判为"第二个接口"而在装载期报错）：分配器
在装载时预扫全配置、先把所有显式号保留下来（`reserve_explicit`），自动号取 `4` 起的
**最小空位**并绕开它们——所以显式号写在哪个位置都生效，发号结果与声明顺序无关。

## 共享串口（同一 `serial:` 路径被多颗 MCU 引用）

同一个 `serial:` 路径被**两颗及以上** `[mcu]` 引用，就是**共享串口**：主机只把它打开一次，每颗 MCU 用自己的 **MCU 层 id** 给帧加一个地址头，收发按 id 分派（装载期按路径分组的是 `mcu/serial_ports.rs`，端口/reader/分派是 `interface/devices/serial.rs`）。只被一颗引用、且这颗**不是桥**时仍是**独占**，行为与 Klipper 一样：**不发任何指派帧、线上不带任何头**（被引用的是桥时见下面「桥的上游链路」段）。

```ini
[mcu]
serial: /dev/ttyUSB0        # 与下一条同路径 → 共享

[mcu tool]
serial: /dev/ttyUSB0
```

- **baud**：共享路径上以**显式写出的** baud 为准；多颗写了不同的值就 `warn` 并**取最小值**，没写 baud 的节跟着这个值走（不一致不再是配置错）。
- **id**：就是 `[mcu_ids]` 分给该 MCU 的号（**仅 CAN 节**可用 `canbus_nodeid` 显式覆盖），与 CAN 节点号**同一个号段**——寻址用同一套映射 `0x100 + 2*id`（设备应答 `+1`）。
- **握手**：每颗 MCU 在**打开（分配）这个共享端口时**同步做一次 id 指派握手，identify 之前完成；默认 **500 ms × 3 次**，无回执即连接失败。节上写了 `uuid`（或别名 `canbus_uuid`）时，握手**先按它发一枚 SET_NODEID**（`0x3f0` 的 `0x01`，按 uuid 点名 id）再发原有的重确认请求——帧序见[串口复用协议](../developer-manual/serial-mux-protocol.md)；独占串口**忽略** `uuid`（不发任何帧）。
- **复位**：`restart_method` 的线级复位（`arduino` 的 DTR 翻转、`cheetah`、`rpi_usb` 切电）作用在**整条线**上，会把共享端口上的所有 MCU 一起打断——所以共享路径**强制 `restart_method: command`**：**显式写了**别的值（`arduino`/`cheetah`/`rpi_usb`）会在装载期告警并改用 `command`；没写则静默降级（串口默认的 `arduino` 只是默认）；非法拼写仍是配置错。
- **发现**：还没分配 id 的设备用 `klipperx scan` 扫出可粘进配置的 uuid（`--interface <总线>` 扫 CAN、`--port <tty>` 扫串口、`--host-library <path.so>` 扫一份 host 库；单次广播查询 + 截止窗口 + 按 uuid 去重，三种传输输出同形）。扫 host 库时用 `--host-identity <12 位十六进制>`（默认全 `0`）给出库在 `init` 前要采纳的身份，它**只在给了 `--host-library` 时可用**。

**桥的上游链路也是复用**：被 `[uart_device]`/`[can_device]` 的 `*_mcu` 指为桥的那颗 `[mcu]`，其 `serial:`/`host_library:` 路径**按 id 寻址**，与引用颗数无关——这条链路上还跑着桥下游设备的帧（规则 7）。判定式是 `按 id 寻址 = 共享 ∨ 下游设备数 ≥ 1 ∨ 是桥`（`mcu/serial_ports.rs::is_id_addressed`，host 库侧同构 `mcu/host_libraries.rs::path_is_mux`）；**下游设备数**是挂在桥端口节（`uart:`/`can:`）上的 `[mcu]` 个数（`mcu/bridge_ports.rs::bridge_downstream_counts`）。**0 台下游的桥仍按 id 寻址**是刻意口径：桥被 mux 化后输出就带帧头，按裸帧解会错位。

> 设备端**设备半与桥半都已实现，但没有架构默认启用它们**：`src/mux.c` 的设备半（就地解析、答扫描、采纳 node id、id 冲突照 CAN 弃号、剥头交付）与 `src/bridge.c` 的桥半（端口表、`uuid → 端口`/`id → 端口` 路由、扇出、单设备路径剥头加头）都在，host 库还有下游串口端口与 unit 收发接口；**mux 路径的真端到端 attach 已有永久测试**。串口侧已由各 MCU 架构（`chipid_init` 派生身份）与 linux 进程控制台（`-U <12 hex>`）接线、host 库由 `HOST_MUX_SERIAL`/`HOST_BRIDGE_SERIAL` 选上；CAN 侧由 `MUX_CAN` 接线（`depends on CANBUS && HAVE_CHIPID`，同一套单元跑在 CAN 字节流里）。**`linux` 架构现在也能当 can-serial 节点**（`LINUX_CANSERIAL`：SocketCAN 控制台，身份只能由 `-U` 给——linux 没有芯片 id），但 `MUX_CAN` 仍不适用于 linux。**两种载体都刻意不默认开**——单元格式不是上游 klippy 主机说的格式，必须显式选择；误配到没开 mux 的旧固件只会连接失败/握手超时，不会搞乱会话。CAN 上游的 attach 与 `[can_device]` 下游端口（`BRIDGE_PORT_CAN`）仍未接线，配置期点名拒绝。

### 线上的形状（设备端实现规格）

单元 = **帧头 + 原样的 Klipper 帧**，帧体一个字节都不改（长度/序号/载荷/CRC 全保持）：

```text
[2 字节头(0x100 + 2*id)]      [len][seq][载荷…][crc16 hi][crc16 lo][0x7e]   主机 → 设备
[2 字节头(0x100 + 2*id + 1)]  [len][seq][载荷…][crc16 hi][crc16 lo][0x7e]   设备 → 主机
```

- 帧头是**固定 2 字节的带校验码字**（**不再是** Klipper 的 VLQ）：仲裁 id 的 10 个数据位 + 4 个校验位（缩短汉明码，**只检不纠**），再加两个定界位（首字节最高位恒 1、次字节恒 0）。本号段任何 id 都是 2 字节：`id=1 → 84 42`、`id=255 → AF 7D`（应答方向 `DF 7D`）。头带校验是因为**帧体 CRC 不覆盖头**：头被翻位后落到另一个合法 id 上会把完好的帧投给错误的 MCU（同线几颗的会话序号同步增长，很容易被当成自己的帧），4 位校验把任何 1/2 位错都拦下（丢单元 + 重同步，Klipper 帧本身有块级重传兜底）。`0x7e` **仍可**出现在头里（`id ∈ 120..=127` 时第二字节就是它）——上游协议本来就允许帧内容含 sync 字符（`third_party/klipper/docs/Protocol.md:178-183`）。
- 握手请求/回执 = 上面的头 **++ 一个 9 字节伪帧** `[09][0x20]["KXID"][crc16 hi][crc16 lo][0x7e]`：`seq` 字节用 `0x20`（把合法帧必置的 `MESSAGE_DEST` 清掉、改占 bit5），所以主机自己的 `Frame::parse` 必然拒收它，**id 只在帧头里出现一次**。
- **请求对旧固件无害**：旧固件 `command_find_block`（`third_party/klipper/src/command.c`）先按长度字节判——单元出线第一个字节就是头首字节（`0x80..=0xff`，非法长度），于是立刻进错误分支、丢弃到下一个 `0x7e`（`id ∈ 120..=127` 时是头的第二字节，伪帧下一次再被丢）；不读载荷、不查 CRC、永不 dispatch、序号不动，至多回一枚无害空帧后转静默。整单元（含新头）的行为与帧头/伪帧的编解码都在 `interface/devices/serial.rs`，由那里的**可执行忠实移植测试**钉住。设备端要做的只是：认出请求 → 用 `+1` 方向的头 + 同形伪帧回执。
- 完整的线上规格、超时/重试、主机侧的单 reader 分派与引用计数关闭，见开发手册的 [串口复用协议](../developer-manual/serial-mux-protocol.md)。

## 桥下游端口（`uart:` / `can:` 引用端口节）

桥的**下游端口**用 klipper 一贯的设备节形式声明（`[uart_device <名>]` / `[can_device <名>]`，
选项见 [配置手册 → 已支持的配置节](config.md#uart_device-名--can_device-名--桥下游端口)），桥下游的
子设备 `[mcu]` 用 `uart:` / `can:` 按**名字**引用它们——`[bridge]`/`[bridge_port]`/`bridge_admin`
都不需要，桥是谁、口在哪、谁是 legacy 全部从引用推算：

- **桥** = 端口节 `uart_mcu`/`can_mcu` 指向的那颗 `[mcu]`（必须存在，且桥不能自己又是下游子设备）；
- **共享**（≥2 颗 `[mcu]` 引用同一端口）⇒ 每颗必须写**互异** `uuid:`，按 id 寻址；
- **legacy**（恰 1 颗 + uart 口 + 无 `uuid:`）⇒ **原始帧**：不发 SET_NODEID、不等重确认握手——
  这就是「与没有 admin 的设备共存」的形态（旧串口固件答不了扫描也 admin 不了，但作为单设备口照常跑）；
- CAN 口的子设备**必须有 `uuid:`**（CAN 寻址本来就是 uuid）。

分组与全部冲突校验在装载期一次完成（`mcu/port_grouping.rs`，预扫原始文本、不豁免未定义选项检查），
报错点名涉及的节。

```ini
[mcu bridge_mcu]                 # 桥自己：上游连接照旧（host_library / serial / CAN）
host_library: /usr/local/lib/libklipper_host.so

[uart_device uartxx]
uart_mcu: bridge_mcu
uart_oid: uart0

[mcu child_old]
uart: uartxx                    # 无 uuid ⇒ legacy，原始帧
```

**归属告知桥、下游完全不参与**：端口与子设备的归属关系由配置声明，主机经桥自己的 admin 通道用
端口级命令 `SET_PORT_NODEID(index, id, flags)` 直接告诉桥，**下游设备全程不答任何 admin**（线上
帧形与命令族见[串口复用协议](../developer-manual/serial-mux-protocol.md)）。

> **状态**：**已落地**（2026-10-15 改为配置期声明）。**端口在配置期由主机声明**：主机为每个 `[uart_device]` 发 `config_uart`（带该节的 `uart_baud`）与 `config_bridge_port`（把该 UART 对象接给桥），**端口 `index` = 声明顺序**；固件侧没有端口表；端口 tty **由主机在库 `init` 前按端口槽运行期登记**（`[uart_device] uart_path:`，不写则回落库的构建默认），`shutdown`（如 baud 不可设、tty 打不开、端口数超限）会变成**主机的配置阶段显式报错**。连接时：`legacy` 只向桥发 `SET_PORT_NODEID(index, id, 0)` 之后按原始帧收发；命名设备（写了 `uuid:`）先发同一条（`flags` 带「说单元」位）再做 `SET_KLIPPER_NODEID` + 重确认，之后整单元原样转发。**顺序约束**：桥必须**先于**该子节连接（顺序不对报点名错误，不会自愈）；又因为裸 `[mcu]` 同阶段总先于 `[mcu <名>]` 装载，**桥是裸 `[mcu]` 时子节必须写成 `[mcu <名>]`**。**`[can_device]` 端口暂不能接给桥**（属 P5），会点名报错。

## 固件重启方式（`restart_method`）

系统重启时，主机需要决定怎么把 MCU 的**固件**复位。`restart_method` 就是这件事，写在 `[mcu]` 节里。它**只对串口 MCU 有意义**：CAN 与 `host_library` 一律按 `command` 处理，配置里写了也会被忽略（并在日志里告警）。

| 值 | 做法 | 说明 |
|----|------|------|
| `command` | 连上之后用固件的 `config_reset` 清掉旧配置，再重新配置 | 非串口的固定做法；串口固件实现了 `config_reset` 时也可用 |
| `arduino` | 以 2400 波特打开串口，翻转 DTR | 串口**缺省**值；Arduino 及克隆板 |
| `cheetah` | 以 2400 波特做一串 RTS/DTR 时序 | Fysetc Cheetah v1.2 等采用状态机 bootloader 电路的板子 |
| `rpi_usb` | 切断再恢复该 USB 端口的供电 | 树莓派等能给 USB 口断电的机器 |
| `admin` | 经桥转发 `RESTART_DEVICE (0x05)`，让该设备**重启自己的固件**（进引导是刷写工具的 `REQUEST_BOOTLOADER (0x02)`，不属 `restart_method`） | **仅用于桥下游端口上的命名设备**（写了 `uuid:`）；无 admin 通道的 legacy 设备改用端口节的 `uart_restart_method`（桥对该口做线脉冲），旧固件/非桥构建会点名报错。端口级命令都**无回执**，确认靠「能不能重新握手」。**设备重启会重新枚举 USB**：下游是 USB CDC 时，宿主库桥会因打不开该端口而失败——主机会**有界等 `uart_device` 的 `uart_path` 回来**（≤10s，建议写 by-id/by-path），回来就请求一次 plain restart（重走库会话、按配置重新登记端口）从而自动恢复；每个 MCU 只有一次这样的恢复预算（成功连上即归还），所以「路径在但一直打不开」只会重启一次便停在 error |

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
