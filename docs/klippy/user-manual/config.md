# 配置文件参考

Klipper 配置文件采用类 INI 格式，用于定义打印机硬件配置、运动学参数以及各种外设选项。

## 文件格式说明

### 基本语法

配置文件由以下元素组成：

| 元素 | 语法 | 示例 |
|------|------|------|
| 节（Section） | `[type]` 或 `[type name]` | `[mcu]`、`[mcu zboard]`、`[stepper_x]` |
| 参数（Parameter） | `key: value` 或 `key = value` | `serial: /dev/ttyACM0` |
| 注释（Comment） | `# ...` 或 `; ...` | `# 这是注释` |
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

节头之后可以跟注释，注释不属于节名：

```ini
[fan]  # 打印冷却风扇
```

### 参数（Parameter）

参数用 `:` 或 `=` 分隔键与值，两者等价；若一行里两者都出现，**靠前的那个**是分隔符。

```ini
[mcu]
serial: /dev/serial/by-path/xxx
baud = 250000
restart_method: arduino
```

### 注释（Comment）

以 `#` 开头的行为注释，不会被解析。行内注释以 `#` 或 `;` 开始：

```ini
[mcu]
serial: /dev/ttyACM0
# baud: 250000    ; 这行被注释掉
baud: 250000      ; 行内注释也会被去除
```

`#` 在行内任意位置都开始注释；`;` 仅在**行首或前面是空白**时开始注释，因此 `a;b` 不会被截断，
而 `a ; b` 会。

> **与上游 Klipper 的差异**：本实现剥 `#` 时会跳过引号内部，因此 `pin: "PA#0"` 的值是 `"PA#0"`；
> 上游 klippy 在第一个 `#` 处**无条件截断**（不区分引号），会得到 `"PA`。本实现更宽松；为避免依赖
> 这一行为，不要把 `#` 写进引号内的值。

### 多行值（Multiline Value）

参数值可以跨行：后续行**比该参数所在行缩进更深**时，即为该参数值的延续，各续行以换行连接。
首行的值可以为空，也可以非空。

```ini
[bed_tilt]
points: 100, 100
        10, 10
        10, 100
```

在此示例中，`points` 的值为 `100, 100\n10, 10\n10, 100`（各行以换行连接）。

空的续行与整行注释不参与连接，也不会终止多行值。

**序列化与回写（2026-10-03，`3aab53c`）**：写多行值时（`SAVE_CONFIG` 块与 `build_config_string`），
值内的换行落为 **tab 缩进续行**（与上游 `configparser` 逐字同形）；**空的首行被保留**——
`key:`（值为空）后接缩进续行的配置，读回时值以 `\n` 开头（同 configparser；执行侧 gcode 逐行跑、
空行无害）。块内形状示例：

```ini
#*# [temperature_probe probe]
#*# drift_calibration =
#*# \t300, 0, 0
#*# \t200, 0, 0
```

（上例中 `\t` 为实际 tab 字符，紧跟在 `#*# ` 之后。）

写回的值文本里，**浮点按 Python 的 `str()` 写**（与上游一致）：整数取值的浮点带小数点（`1.0`）、
很小或很大的值用指数（`1e-05`、`1e+16`），因此本仓与上游写出的 `printer.cfg` 可以互换；
`[save_variables]` 的 `[Variables]` 文件同理（上游用 `repr`）。

### 空节（Empty Section）

节可以没有参数，仅用于声明存在。

```ini
[pause_resume]

[exclude_object]
```

### `[include]`（引入其它文件）

```ini
[include mainsail.cfg]           # 子字段写法（与上游同形）
[include path: macros/*.cfg]     # path: 参数写法
[include https://example.com/part.cfg]
```

- **路径基准**：相对**包含它的那个文件**的目录，不是进程 cwd；结对路径按原样。被引入的文件里再 include，继续相对于它自己的目录。
- **通配符**：`*` / `?` / `[]`，与上游 `glob.glob` 同口径（**非递归**，`**` 不做特殊处理），并且**不匹配以 `.` 开头的隐藏文件**（Python 的行为）。如果展开出多个文件，**按路径排序**后依次读入——顺序会影响同名节/参数的覆盖，所以它是行为的一部分，不是实现细节。
- **无匹配 vs 没文件**：有通配符但一个都没匹配上——**静默当作没引入**（上游同口径），不报错；没有通配符而文件不存在——报 `Failed to read file '<path>': …`。
- **URL 取源（本仓特性，上游没有）**：`http://` / `https://` 开头时直接把远端正文当配置。边界：总超时 **10 s**（含 DNS/连接/重定向/读体）、连接 **5 s**、正文上限 **4 MiB**（边读边判，超限报错而不是先读进内存）；成功时会打一行 `Fetched config from URL '<url>' (<n> bytes)`。通配符对 URL 不生效（URL 永远只当一个源）。
- **环检测**：同一个来源被引入两次会报 `Circular include detected: <source>`。
- **`SAVE_CONFIG` 块里的 `[include]`** 不当引入处理（与上游一致）。

> **自包含陷阱**：把 `[include *.cfg]` 写在**被它匹配到的目录**里（例如 `printer.cfg` 自己就在当前目录）时，通配符会把 `printer.cfg` 也展开进来，于是环检测报 `Circular include detected`。上游没有环检测，这种写法会无限递归。要引用“同目录的其它文件”时请收窄到子目录（`[include conf.d/*.cfg]`）或写具名文件。

> 远端内容是**真正的配置正文**（宏可以跑 g-code），所以只引入你信任的来源。

## 选项校验

装载配置时，**每个 section 与每个选项都必须被某个模块读到**：

- 没有任何对象对应的 section（且没有模块读它）→ `Section 'xxx' is not a valid config section`；
- 某个 section 里没人读的选项（通常是拼写错误）→ `Option 'xxx' is not valid in section 'yyy'`。

校验**不**在装载时“提前放行”任何选项，所以一个拼错的键不会被默默忽略——这正是它存在的意义。
有默认值的选项（如 `[output_pin]` 的 `value`）算作已读，不需要写出来。

配置出错时打印机进入 `error` 状态（`info` 的 `state` 为 `error`，不是 `shutdown`）：
`RESTART` 会重新从磁盘读取配置文件，所以改好文件再发一次 `RESTART` 即可，不必重启进程；
文件被删或语法错误时同样是 `error`，消息里带失败原因，进程也不退出。这与 G-code 参数错误
（`!!` 回复、不停机）和真正的内部错误（停机，`state` 为 `shutdown`）是三条不同的路。

选项越界报错的措辞与上游逐字一致；限制值的数字写法与上游不同（本仓 `0`，上游 `0.0`），
只影响显示、不影响判定，原因见[开发手册](../developer-manual/README.md) 的
「越界报错里限制值的写法」一节。

`configfile` 是内置的只读对象，`objects/query configfile` 可读到：

| 字段 | 含义 |
|------|------|
| `config` | 配置文件的每个 section / 选项原文 |
| `settings` | 各模块实际读到的选项及其解析后的值 |
| `warnings` | 弃用与运行期警告；同一条只记一次，按首次出现顺序 |

## MCU 连接方式

`[mcu]` 节的连接参数、固件重启方式、udev 规则等详细说明已移至独立文件，[参见 → MCU 连接方式](mcu-connection.md)。

## 示例配置

以下是一个可直接装载的最小 cartesian 示例（每项必需选项都给出；各节的完整参数见下文）：

```ini
[mcu]
serial: /dev/serial/by-path/platform-3f980000.usb-usb-0:1.2:1.0-port0
baud: 250000

[stepper_x]
step_pin: PA0
dir_pin: PA1
enable_pin: !PA2
rotation_distance: 40
microsteps: 16
position_min: 0
position_max: 200
position_endstop: 0
endstop_pin: PA3
homing_speed: 5

[stepper_y]
step_pin: PA4
dir_pin: PA5
enable_pin: !PA2
rotation_distance: 40
microsteps: 16
position_min: 0
position_max: 200
position_endstop: 0
endstop_pin: PA6
homing_speed: 5

[stepper_z]
step_pin: PB0
dir_pin: PB1
enable_pin: !PA2
rotation_distance: 8
microsteps: 16
position_min: -5
position_max: 100
position_endstop: 0
endstop_pin: PB2
homing_speed: 2

[extruder]
step_pin: PB3
dir_pin: PB4
rotation_distance: 4.233
microsteps: 16
nozzle_diameter: 0.400
filament_diameter: 1.750
heater_pin: PB5
sensor_type: EPCOS 100K B57560G104F
sensor_pin: PC0
min_temp: 0
max_temp: 250
control: pid
pid_Kp: 21.5
pid_Ki: 1.54
pid_Kd: 76.5

[printer]
kinematics: cartesian
max_velocity: 300
max_accel: 3000
```

## 已支持的配置节

本页逐节登记的「未实现 / 与上游不同」之处，统一登记在[本项目与上游的偏移](../developer-manual/upstream-deviations.md)（那里按「不做 / 行为偏移 / 暂缓」分类）；本页仍保留各节的具体说明。

以下为 KlipperX 当前已实现的配置节（按装载顺序；`[mcu]` 与 `[printer]`
分别在最早与最晚装载，其余按各节 `section!` 声明的 `order`；全仓装载 id 总数以
[开发手册模块表](../developer-manual/README.md)的实测数字为准）。

### `[mcu]` / `[mcu <name>]` — MCU 连接

定义一台 MCU 设备的连接方式。支持匿名节（`[mcu]`）和命名节（`[mcu <name>]`）。

| 参数 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `host_library` | 字符串 | 二选一 | — | klipper host 库（`libklipper_host.so`）的路径：在主机进程内跑一份 klipper 固件。与 `serial`/CAN 键互斥；同路径可被多颗 `[mcu]` 引用，判定为 mux 时连接按 id 寻址（见 [MCU 连接方式](mcu-connection.md)）；桥的各端口 tty 由主机在库 `init` 前按端口槽登记（`[uart_device] uart_path:`），不是编译期烘死；**一个进程只允许一份 host 库**——两个不同路径会在装载期点名报错（要两个固件请用同一路径的多引用，或用独立进程） |
| `serial` | 字符串 | 二选一 | — | 串口设备路径（如 `/dev/ttyACM0`）。**同一路径被两颗及以上 `[mcu]` 引用即共享**（见 [MCU 连接方式](mcu-connection.md)） |
| `uart` | 字符串 | 二选一 | — | 引用一个 `[uart_device <名>]` 端口节的**名字**（如 `uart: uartxx` 引 `[uart_device uartxx]`），把这颗 MCU 挂到桥的下游 uart 端口上。与 `serial`/`host_library`/`test`/`can`/`canbus_interface` 互斥；`uuid` 可选——**不写就是 legacy**（原始帧，规则见 [MCU 连接方式](mcu-connection.md#桥下游端口uart--can-引用端口节)）；`canbus_nodeid` 不可用（非 CAN 链路）。**端口在配置期由主机声明**：主机为该端口节发 `config_uart`（带本节的 `uart_baud`）与 `config_bridge_port`（把该 UART 对象接给桥），**端口 `index` = 端口节的声明顺序**；`uart_baud` 缺省 250000，写错/端口打不开等由固件 `shutdown` ⇒ **主机在配置阶段显式报错**（不再有「编译期端口表」）。**顺序约束**：桥必须**先于**本子节连接；而裸 `[mcu]` 在同一阶段总是先于 `[mcu <名>]` 装载，所以桥是裸 `[mcu]` 时本子节**必须**写成 `[mcu <名>]`（否则报点名错误） |
| `can` | 字符串 | 二选一 | — | 引用一个 `[can_device <名>]` 端口节的名字，挂到桥的下游 CAN 端口。**`uuid` 必填**（CAN 寻址靠它，装载期点名报错）；与 `serial`/`host_library`/`test`/`canbus_interface`/`canbus_fd`/`uart` 互斥；`canbus_nodeid` 可用（id 覆盖与链路正交）。端口由主机在配置期声明（`config_can` + `config_bridge_port`，`index` = 声明顺序）；**运行期会话已真机跑通**（`identify` + 配置握手 + 周期 `get_clock`，可反复附着），吞吐见 [桥下游链路的吞吐与余量](mcu-connection.md#桥下游链路的吞吐与余量真机实测) |
| `baud` | 整数 | 否 | `250000` | 串口波特率（仅 `serial` 模式）。共享路径以**显式写出**的值为准——多颗写了不同值就 `warn` 并**取最小值**（没写的节跟着它走） |
| `uuid` | 12 位十六进制 | 二选一 | — | MCU 芯片唯一 ID（12 位 hex）：CAN 模式必给；写在 `serial` 节上则共享串口连接期按它先发 SET_NODEID 再握手（独占串口忽略）。历史键名 `canbus_uuid` 仍兼容，同节两个都写会报错；**写在 `host_library` 节上**则指明这条库路径上被寻址的是哪块设备（判定见 [MCU 连接方式](mcu-connection.md)：同路径 1 节有 `uuid`、或 ≥2 节即按 id 寻址；不写 `uuid:` 的节认领桥自身设备，用装载期为该路径指派的身份点名） |
| `canbus_interface` | 字符串 | 否 | `can0` | CAN 网络接口名 |
| `canbus_nodeid` | 整数 (1..=255) | 否 | 由 `[mcu_ids]` 分配 | **MCU 层 id**，**只有 CAN 节能写它**（写到其它节会被判为"第二个接口"报错；配置键名沿用 Klipper；在 CAN 上就是节点号，见下）。**不写就由分配器发**（`4` 起的最小空位，上限 255——CAN admin 报文只给它留了 1 字节）；写了就用写的那个——分配器装载时**预扫全配置**把显式号保留下来、自动号绕开，所以显式号写在哪个位置都生效、也不会被自动号占用。装载期只在两种情况报错：两个 `[mcu]` 写了同一个号，或 `4..=255` 已占满（上游靠固件 `can_id_conflict` 兜底，本仓提前拒）。 |
| `restart_method` | `command` / `arduino` / `cheetah` / `rpi_usb` / `admin` | 否 | `arduino`（串口）/ `command`（非串口/共享串口） | MCU 固件重启方式。**共享串口强制 `command`**（线级复位会打断整条线上的所有 MCU；显式写了别的值会告警，没写则静默降级）。**桥下游端口上的设备另有一套**：命名设备（写了 `uuid:`）可用 **`admin`**——命令经桥转发 `RESTART_DEVICE (0x05)`、由设备自己重启固件（**不是**进 bootloader：进引导是刷写工具的命令 `REQUEST_BOOTLOADER (0x02)`，不进 `restart_method`）；设备重启会重新枚举，宿主库桥对因此打不开端口的情形会**有界等路径回来 + 一次 plain restart**（见 `uart_path` 行）；legacy 设备的复位由**端口节**的 `uart_restart_method` 决定（见下）。端口级命令都**无回执**，主机靠「复位后能否重新握手/identify」确认 |
| `usb_power` | `auto` / `sysfs` / `libusb` | 否 | `auto` | `rpi_usb` 的切电机制 |

> 详细参数说明见 [MCU 连接方式](mcu-connection.md)。

### `[mcu_ids]` — MCU ID 分配器（历史名 `[canbus_ids]` 兼容）

**无参数节。** 它给每个 `[mcu]`（不论接口类型）发一个**唯一 id**：自动号是
`NODEID_FIRST = 4` 起的**最小空位**，上界 `MAX_MCU_ID = 0xff`（255，由承载它的 CAN admin 报文单字节字段决定）；装载时分配器
先**预扫全配置**，把每个 `canbus_nodeid` 显式写下的号保留下来（`reserve_explicit`，读节的
原始参数、不经 accessor，因此不影响未定义选项检查），自动号绕开这些号——所以显式号写在文件
哪个位置都生效，发号结果与 `[mcu]` 声明顺序无关。**所有接口共享同一个 id 空间**——CAN 与
串口的 MCU 不会拿到同一个号。对 CAN MCU，这个 id 就是它在总线上的节点号（寻址用
`0x100 + 2*id`——该映射单一定义在 `assign.rs`、CAN 与共享串口共用，见
[MCU 连接方式](mcu-connection.md)）；对其它接口：**共享串口**（同一 `serial:` 路径被
≥2 颗 `[mcu]` 引用）在分配端口时用同一套 `0x100 + 2*id`（应答 `+1`）做**帧头寻址**、
并同步握手一次，**独占串口不使用这个号**，`host_library` 与 `test:` **同样从这一序列领到 id，只是暂不上线**——id 是每个 `[mcu]` 的属性（`McuObject::connect` 没有它就拒绝打开传输层），**怎么用**才是各传输自己的事（`test:` 恒不用；host 库路径**按 id 寻址的判定**已在配置层落地：同路径 1 节无 `uuid` = 裸帧、1 节有 `uuid` 或 ≥2 节 = 按 id 寻址，而**判定为 mux 的那些节连接时确实按 id 寻址**（装载期指派身份并在 `init` 前投递、`SET_NODEID` + 重确认握手、收发带帧头）；只有 `test:` 恒不上线）。MCU
在 connect 之前必须先持有这个 id（未分配就拒绝打开传输层）。

不需要它时不必写——第一个 `[mcu]`（不论什么接口）会自己把它拉起来。

```ini
[mcu_ids]             # 可写可不写；写了也只是显式声明这个分配器
[canbus_ids]          # 历史拼写（CAN 模式兼容）：注册的是同一个分配器，两种不要同写
```

与上游的两处差异（详见[本项目与上游的偏移](../developer-manual/upstream-deviations.md)）：
① `canbus_nodeid` 是本仓特有的显式覆盖选项（上游只有 CAN 的 uuid 键——本仓规范名 `uuid`、旧名 `canbus_uuid` 兼容），且上游的分配器只服务
CAN MCU、本仓把它泛化成所有接口共用一个序列；
② 装载期只有两种情况报配置错：**两个显式号互撞**（点明双方），或 `4..=255` **号段占满**
（点名 MCU）；自动号永不与显式号撞（先预扫保留、再取最小空位）。不像上游那样留到固件
`can_id_conflict` 才停机。
发现未分配的板子（拿到可粘进配置的 uuid）见 `klipperx scan`——`--interface <总线>` 扫 CAN、`--port <tty>` 扫串口、`--host-library <path.so>` 扫一份 host 库（三者互斥、必须给其一，裸调用打 help；同一套查询/应答协议；扫 host 库时另可用 `--host-identity` 指定它在 `init` 前要采纳的身份）。

### `[uart_device <名>]` / `[can_device <名>]` — 桥下游端口

桥（跑 `CONFIG_BRIDGE` 固件的那颗 `[mcu]`）的**下游端口**按 klipper 一贯的设备节形式声明，
桥下游的子设备 `[mcu]` 用 `uart:` / `can:` 按**名字**引用它们。`[bridge]` / `[bridge_port]` /
`bridge_admin` 都不需要：桥是谁、口在哪、谁是 legacy，全部从引用推算。

| 参数 | 适用节 | 类型 | 必需 | 说明 |
|------|--------|------|------|------|
| `uart_mcu` | `uart_device` | 字符串 | 是 | 这条 uart 口长在哪颗 `[mcu]` 上（裸 `[mcu]` 写 `mcu`，命名节写名字）。必须指向配置里存在的 `[mcu]`，否则装载报错 |
| `uart_oid` | `uart_device` | 字符串 | 是 | 该端口的名字（如 `uart0`）。固件不再上报端口表，`index` 由**声明顺序**给出；这个名字现在只用于人机对照与查端口节（MCU 后端落地后将成为固件 UART 实例名的挂点） |
| `uart_path` | `uart_device` | 字符串 | 否 | **只对「桥经 `host_library:` 连接」的端口有意义**：这条端口该开哪个 tty（例如 `/dev/serial/by-id/…`）。主机在库 `init` **之前**按端口槽把它登记给库（`klipper_host_port(slot, path)`），`slot` 就是端口 `index`（= 声明顺序）。**不写**则回落库构建期的默认路径（`CONFIG_HOST_BRIDGE_SERIAL_PATH`）；桥不是 host 库却写了它 ⇒ **装载期点名报错**；路径打不开 ⇒ **配置期点名报错**（固件 `shutdown`，库 stderr 里有该路径）。这也意味着一份库可以配多条端口路径，不必为每条 tty 重建固件。**建议写 `by-id`/`by-path` 这种稳定路径**：下游设备被点名重启后会**重新枚举**，路径会短暂消失甚至换名；主机对「就是打不开该路径」这一类失败会**有界等路径回来、再请求一次 plain restart**（进程级重启会重启库会话、按配置重新登记端口）——用 by-id/by-path 才能保证重启后仍指向同一台设备；每个 MCU 只享用一次这样的恢复（重启后成功连上即归还） |
| `uart_baud` | `uart_device` | 整数 | 否 | 端口自身的线参数；**配置期由主机下发给固件**（`config_uart baud=…`），缺省 250000；非零校验，固件设置失败会 `shutdown` ⇒ 配置阶段显式报错 |
| `uart_restart_method` | `arduino` / `cheetah` | 否 | — | 声明**该端口的板子怎么被线级复位**（写其它值报错）。声明了它，挂在该端口上的 **legacy 子设备**（无 `uuid:`）在 `FIRMWARE_RESTART` 时由桥对该口做 DTR\|RTS 脉冲（`RESET_PORT 0x04`）；不声明则子设备沿用 `command`。注意：桥的链路按 id 寻址（共享串口或“它是桥”），所以桥自己走 `serial:` + `restart_method: cheetah` 时**不会驱 RTS**（与非桥的独占串口不同，配置层会告警）。⚠️ **只对 USB-串口桥接板有效**（CH340/CP2102/FTDI 这类，DTR\|RTS 接在复位脚上）：**原生 USB CDC 板**（klipper 自带 USB 栈，设备名形如 `usb-Klipper_<board>_<serial>-if00`）**不把 DTR/RTS 当复位**——固件只把控制线状态记进 `line_control_state`，唯一消费是「波特率 1200 且 DTR 为低」时进 bootloader（`src/generic/usb_cdc.c:452-459,485-494`），所以 250 ms 的 DTR\|RTS 脉冲在那类板子上是**静默无效**的（内核照常接受 ioctl）；这类 legacy 设备目前**没有**经桥的硬复位手段，要用命名设备 + `restart_method: admin`（设备自身复位）或断电 |
| `can_mcu` | `can_device` | 字符串 | 是 | 同 `uart_mcu` |
| `can_oid` | `can_device` | 字符串 | 是 | 同 `uart_oid`。主机按声明顺序发 `config_can`（带本节 `can_bitrate`）+ `config_bridge_port`；`index` = 声明顺序。运行期会话已真机跑通（含 admin 通道的 scan/指派/送引导/隧道刷写），见[MCU 连接方式](mcu-connection.md)的 CAN 小节 |

未知选项会按未定义选项报错。**分组与冲突校验**（装载期，`mcu/port_grouping.rs`）：

- 端口被 **≥2 颗 `[mcu]` 引用即共享**：每颗都必须写 `uuid:` 且互异（缺、重复都报错并点名），按 id 寻址；
- **恰 1 颗 + `uart_device` + 无 `uuid:` ⇒ legacy**：走原始帧（不发 SET、不等握手）——这就是「与没有 admin 的设备共存」的形态；
- **CAN 端口的子设备必须写 `uuid:`**（规则 6）；
- 端口的 `uart_mcu`/`can_mcu` 必须指向存在的 `[mcu]`，且**桥不能自己又是下游子设备**。

```ini
[uart_device uartxx]
uart_mcu: bridge_mcu      # 这条 uart 口长在哪颗 MCU（桥）上
uart_oid: uart0           # 端口名（index 按声明顺序）
uart_baud: 250000         # 可选

[can_device canxx]
can_mcu: bridge_mcu
can_oid: can0

[mcu child_old]
uart: uartxx              # 无 uuid ⇒ legacy，原始帧

[mcu child_new]
uart: uartxx
uuid: 11aa22cc33dd        # 同一条口上第二台必须写互异 uuid
```

> `i2c`/`spi` 复用已有节类型（`[i2c <名>]`/`[spi <名>]`），其端口引用写法尚未定案（见笔记 §5.6 未决 a）。

### `[output_pin <name>]` — 输出引脚（数字 / PWM）

定义一个可通过 `SET_PIN` 命令控制的输出引脚。该节**必须**带名称（即 `[output_pin <name>]` 格式），名称即为 `SET_PIN PIN=<name>` 中引用的引脚名。默认是数字输出，给 `pwm: true` 后变成 PWM。

| 参数 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `pin` | 字符串 | 是 | — | 引脚描述（格式：`[!][<mcu_name>:]<pin>`），对应已配置的 MCU |
| `value` | 0.0 ~ 1.0 | 否 | `0` | 启动时的值（数字输出 `>= 0.5` 为高；PWM 为占空比） |
| `shutdown_value` | 0.0 ~ 1.0 | 否 | `0` | 关机时回退的值 |
| `pwm` | 布尔 | 否 | `false` | 使用 PWM 而非数字输出 |
| `cycle_time` | 秒 | 否 | `0.1` | PWM 周期（仅 `pwm: true`） |
| `hardware_pwm` | 布尔 | 否 | `false` | 用硬件的 `config_pwm_out`；默认走软件 PWM（数字引脚翻转） |

**注意：**
- 数字输出与 PWM 都支持 `!` 取反前缀。
- 值变化**按打印时间生效**（有 `[printer]` 即 toolhead 在时）：`SET_PIN` 经 `GCodeRequestQueue` 排进前瞻、flush 时以 `queue_digital_out(clock)`／对齐后的 `set_pwm` 落地，同值重复设置去重不发帧，`G4` 推进的时间线因此真正给引脚换相定时。**两条兑底**：配置里没有 `[printer]`（lookup 不到 toolhead）或资源未连接时，退回“立即”路径（数字输出 `update_digital_out`、PWM `update_pwm`，软 PWM 对齐到软件周期边界）。
- `SET_PIN PIN=<名> TEMPLATE=<display_template 名> [PARAM_*=…]` **已支持**：模板每 0.5 s 求值一次（上游同款 reactor 定时器，**不走**前瞻），结果经 `send_async_request` 按最近的 min_schedule_time 下发；渲染失败只告警并落 `0`。`TEMPLATE=`（空值）取消订阅；`VALUE=` 与 `TEMPLATE=` 必须给且只给一个（否则报 `SET_PIN command must specify VALUE or TEMPLATE`）。模板路径的值**不除 `scale`**、也不做 `0/1` 校验（上游同）。
- `static_value` 上游已删该参数（`Config_Changes` 20240912），本仓只**接受并忽略**它、不再当非法选项报错；启动值用 `value`。
- 软件 PWM 的 `shutdown_value` 只能是 `0.0` 或 `1.0`（固件只能把引脚固定在高或低）。

**示例：**

```ini
[output_pin my_fan]
pin: mcu:PA0
value: 0
shutdown_value: 0

[output_pin my_light]
pin: mcu:PB5
value: 0

# 软件 PWM（固件翻转 GP10）
[output_pin pwm_fan]
pin: mcu:PA1
pwm: true
cycle_time: 0.02
hardware_pwm: false
```

对应 G-Code 命令：

```gcode
SET_PIN PIN=my_fan VALUE=1      ; 打开风扇
SET_PIN PIN=my_fan VALUE=0      ; 关闭风扇
SET_PIN PIN=pwm_fan VALUE=0.25  ; 25% 占空比
```

### `[board_pins]` / `[board_pins <name>]` — 板级引脚别名

把主板丝印上的排针名映射到 MCU 的真实引脚名。节可以带名称也可以不带；`mcu` 选项指定别名应用到哪几台 MCU（默认 `mcu`）。

| 参数 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `mcu` | 字符串列表 | 否 | `mcu` | 逗号分隔的 MCU 名列表 |
| `aliases` | `名=引脚` 列表 | 否 | — | 逗号分隔的别名列表；值写成 `<...>` 时表示**保留**该引脚 |
| `aliases_<name>` | 同上 | 否 | — | 可以拆成多组；`aliases_` 开头的选项都会被读取 |

**示例：**

```ini
[board_pins]
aliases:
    EXP1_1=PA0, EXP1_2=PA1, EXP1_3=PB0,
    EXP1_9=<GND>, EXP1_10=<5V>

[output_pin fan]
pin: EXP1_1        ; 等价于 PA0
```

被 `<...>` 保留的引脚（如 `EXP1_9`）会拒绝后续引用，报 `pin EXP1_9 is reserved for <GND>`。

### `[i2c_device <name>]` — 原始 I2C 设备

定义一个 I2C 设备，把它接到某台 MCU 的硬件 I2C 总线，或软件（bit-bang）I2C 引脚上。该节**必须**带名称，名称用于 `IIC_READ` / `IIC_WRITE` 的 `DEVICE=<name>`，也是 `objects/query` 中的对象名。

这是 I2C（F7）栈的第一个消费者；上游没有通用 `[i2c_device]` 节，各传感器各自通过 `MCU_I2C_from_config` 读取同样的选项。

| 参数 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `i2c_mcu` | 字符串 | 否 | `mcu` | 设备所在的 MCU 名 |
| `i2c_address` | 整数 (0..127) | 是 | — | 7 位设备地址 |
| `i2c_speed` | 整数 (Hz) | 否 | `100000` | 时钟频率，最低 `100000` |
| `i2c_bus` | 字符串 | 否 | 固件中编号 0 的总线 | 硬件总线名（如 `i2c_1`） |
| `i2c_software_scl_pin` | 引脚描述 | 二选一 | — | 软件 I2C 的 SCL 引脚 |
| `i2c_software_sda_pin` | 引脚描述 | 二选一 | — | 软件 I2C 的 SDA 引脚 |

**注意：**
- 软件 I2C 的两个引脚必须都给出，且都在同一台 `i2c_mcu` 上；只给一个会报错。
- `i2c_bus` 与软件引脚二选一；给了软件引脚就忽略 `i2c_bus`。
- `i2c_address` 是十进制整数（写 `0x68` 会报无法解析），`0x68` 应写 `104`。
- 该节只搬运字节，不含任何设备协议；`IIC_WRITE` / `IIC_READ` 是**调试命令**（上游没有），用于在真板上验证总线。命令前缀用 `IIC_` 而不是 `I2C_`：G-Code 扩展命令名不允许第二个字符是数字（`I2C_READ` 会被判为非法名而拒绝注册）。

**示例：**

```ini
# 硬件 I2C（mcu 的总线 i2c_1，地址 0x68）
[i2c_device accel]
i2c_mcu: mcu
i2c_bus: i2c_1
i2c_address: 104
i2c_speed: 400000

# 软件（bit-bang）I2C
[i2c_device eeprom]
i2c_address: 80
i2c_software_scl_pin: mcu:PA9
i2c_software_sda_pin: mcu:PA10
```

对应 G-Code 命令：

```gcode
IIC_WRITE DEVICE=accel DATA=6b              ; 写 1 字节
IIC_READ DEVICE=accel WRITE=75 READ_LEN=1   ; 写寄存器 0x75 后读 1 字节
```

### `[spi_device <name>]` — 原始 SPI 设备

定义一个 SPI 设备，把它接到某台 MCU 的硬件 SPI 总线，或软件（bit-bang）SPI 引脚上。该节**必须**带名称，名称用于 `SPI_TRANSFER` / `SPI_SEND` 的 `DEVICE=<name>`。

这是 SPI（F6）栈的第一个消费者；上游没有通用 `[spi_device]` 节，各设备各自通过 `MCU_SPI_from_config` 读取同样的选项。

| 参数 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `spi_mcu` | 字符串 | 否 | `mcu` | 设备所在的 MCU 名 |
| `cs_pin` | 引脚描述 | 否 | — | 片选引脚；写 `None`（或不写）表示不用固件驱动 CS |
| `cs_active_high` | 布尔 | 否 | `false` | 片选是否高电平有效 |
| `spi_mode` | 整数 (0..3) | 否 | `0` | SPI 模式（CPOL/CPHA） |
| `spi_speed` | 整数 (Hz) | 否 | `100000` | 时钟频率，最低 `100000` |
| `spi_bus` | 字符串 | 否 | 固件中编号 0 的总线 | 硬件总线名（如 `spi1a`） |
| `spi_software_miso_pin` / `spi_software_mosi_pin` / `spi_software_sclk_pin` | 引脚描述 | 三个一起 | — | 软件 SPI 的三个引脚 |

**注意：**
- 软件 SPI 的三个引脚必须都给出，且都在同一台 `spi_mcu` 上；只给一部分会报错。
- `spi_bus` 与软件引脚二选一；给了软件引脚就忽略 `spi_bus`。
- 片选由**固件**驱动：每次传输会拉低 CS、移位、再释放，所以一次 `SPI_TRANSFER` 就是一个完整的片选周期（命令字节和随后的数据要放在同一个 `DATA` 里）。
- `SPI_TRANSFER` / `SPI_SEND` 是**调试命令**（上游没有），用于在真板上验证总线。

**示例：**

```ini
# 硬件 SPI（STM32F103 的 SPI1 重映射到 PB3/PB4/PB5，固件总线名 spi1a），CS 用 PA15
[spi_device flash]
spi_mcu: mcu
spi_bus: spi1a
cs_pin: PA15
spi_mode: 0
spi_speed: 1000000

# 软件（bit-bang）SPI，同一组引脚
[spi_device flash_sw]
cs_pin: PA15
spi_software_miso_pin: PB4
spi_software_mosi_pin: PB5
spi_software_sclk_pin: PB3
```

对应 G-Code 命令：

```gcode
SPI_TRANSFER DEVICE=flash DATA=9f000000    ; W25 flash JEDEC ID → ef 30 13
```

### `[printer]` — 运动与工具头（注册为 `toolhead`）

整机的运动参数与运动学选择。该节在**最后**装载（上游也是最后 load `toolhead`），装载时注册
`G4` / `M400` / `G28` / `SET_KINEMATIC_POSITION`，并拉起 `gcode_move`（坐标系）与
`query_endstops`。

| 参数 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `kinematics` | 字符串 | 是 | — | `none` / `cartesian` / `corexy` / `corexz` / `hybrid_corexy` / `hybrid_corexz` / `polar` / `delta` / `rotary_delta`（批 #40；`rotary_delta`/`deltesian`/`winch`/`generic_cartesian` 待做） |
| `max_velocity` | 浮点 (mm/s) | 是 | — | `> 0` |
| `max_accel` | 浮点 (mm/s²) | 是 | — | `> 0` |
| `minimum_cruise_ratio` | 浮点 (0..1) | 否 | `0.5` | 巡航段占比下限（上游同名选项） |
| `square_corner_velocity` | 浮点 (mm/s) | 否 | `5.0` | `≥ 0`，决定拐角允许的速度 |
| `max_z_velocity` | 浮点 (mm/s) | 否 | `= max_velocity` | `≤ max_velocity` |
| `max_z_accel` | 浮点 (mm/s²) | 否 | `= max_accel` | `≤ max_accel` |

`kinematics: none` 时不需要任何 `[stepper_*]` 节（开发/测试机）；其余运动学需要
`stepper_x` / `stepper_y` / `stepper_z` 三根轴都存在。

### `[stepper_x]` / `[stepper_y]` / `[stepper_z]` — 电机与轴

一根轴上的电机。`enable_pin` **在本节读取**（由 `[stepper_enable]` 对象管理、跨节引用计数
共享）；编号兄弟节 `[stepper_z1]` / `[stepper_z2]` … 没有自己的工厂，由主节
（`[stepper_z]`）的装载**连带读取**，只写电机选项，不写几何选项。

| 参数 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `step_pin` | 引脚 | 是 | — | 步进信号，`!` 取反即上游的 `invert_step` |
| `dir_pin` | 引脚 | 是 | — | 方向信号，必须与 `step_pin` 同 MCU |
| `enable_pin` | 引脚 | 否 | — | 使能脚，`!` 取反；多个 stepper 节写同一脚时共享并引用计数 |
| `rotation_distance` | 浮点 (mm) | 是 | — | 电机每整圈走的毫米数，`> 0` |
| `microsteps` | 整数 | 是 | — | 每整步的细分数，`≥ 1` |
| `full_steps_per_rotation` | 整数 | 否 | `200` | 电机整步数 |
| `gear_ratio` | `g1:g2` 列表 | 否 | 空 | 逗号分隔的多对减速比，乘进步距；每对恰好 2 项 |
| `step_pulse_duration` | 浮点 (s) | 否 | `0.000002` | 步进脉宽，`0 ≤ … ≤ 0.001` |
| `position_min` | 浮点 (mm) | 否 | `0` | 轴行程下限（主节几何） |
| `position_max` | 浮点 (mm) | 是 | — | 轴行程上限，`≥ position_min`（主节几何） |
| `position_endstop` | 浮点 (mm) | 否 | `= position_min` | 限位开关位置，必须落在 `[min, max]` 内 |
| `endstop_pin` | 引脚 | 否 | — | 归零用限位；不写则该轴不能 `G28` |
| `homing_speed` | 浮点 (mm/s) | 否 | `5.0` | 归零速度，`> 0` |
| `second_homing_speed` | 浮点 (mm/s) | 否 | `= homing_speed / 2` | 第二遍速度（当前回零是单程，选项已读入备用） |
| `homing_retract_speed` | 浮点 (mm/s) | 否 | `= homing_speed` | 回缩速度（同上，单程未用） |
| `homing_retract_dist` | 浮点 (mm) | 否 | `5.0` | 触发后的回缩距离（同上，单程未用） |
| `homing_positive_dir` | 布尔 | 否 | 由 `position_endstop` 推断 | endstop 在低端四分位推为 `false`、高端推为 `true`；居中且未给时报错 |

注意：几何选项（`position_*` / `endstop_pin` / `homing_*`）属于**主节**，编号兄弟只当电机用。

### `[stepper_enable]` — 使能跟踪（无选项）

管理全部 stepper 的使能状态与共享使能脚，注册 `M18` / `M84` /
`SET_STEPPER_ENABLE STEPPER=… ENABLE=…`，并在 `gcode:request_restart` 时停电机
（广播 `stepper:motor_off`）。**该节本身不接受任何选项**（`enable_pin` 写在各
`[stepper_*]` 节里）；装载即生效，配置里写一个空节即可：

```ini
[stepper_enable]
```

### 加热器共用选项（`[extruder]` / `[heater_bed]` / `[heater_generic <name>]`）

三个加热节都经 `heaters::setup_heater` 读同一组选项：

| 参数 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `sensor_type` | 字符串 | 是 | — | 温度传感器类型，查 `heaters` 注册表（可选值见 `[temperature_sensor]` 的速查表） |
| `sensor_pin` | 引脚 | 是* | — | 传感器的 ADC 引脚；`DS18B20` / `temperature_mcu` / `temperature_combined` 不用它 |
| `heater_pin` | 引脚 | 是 | — | 加热输出（按 PWM 接）；固件看门狗上限 `MAX_HEAT_TIME = 3.0 s`（主机停刷新则固件把口拉回 `shutdown_value`） |
| `min_temp` | 浮点 (°C) | 是 | — | 报警下限 |
| `max_temp` | 浮点 (°C) | 是 | — | 报警上限，`> min_temp` |
| `min_extrude_temp` | 浮点 (°C) | 否 | `170` | 允许挤出的最低温度（须在 `[min_temp, max_temp]` 内） |
| `max_power` | 浮点 (0..1] | 否 | `1.0` | 占空比上限 |
| `smooth_time` | 浮点 (s) | 否 | `1.0` | 读数平滑时间，`> 0`（也是 PID 的 `min_deriv_time`） |
| `pwm_cycle_time` | 浮点 (s) | 否 | `0.100` | 加热器 PWM 周期，`> 0` |
| `control` | `watermark` / `pid` | 是 | — | 控制算法 |
| `max_delta` | 浮点 (°C) | 否* | `2.0` | `watermark`（bang-bang）的回差，`≥ 0` |
| `pid_Kp` / `pid_Ki` / `pid_Kd` | 浮点 | `pid` 时是 | — | PID 三参数（内部除以 1000，与上游同基准） |

*`sensor_pin` 由所选传感器工厂决定是否必需；`max_delta` 只在 `control: watermark` 下读。

### `[extruder]` / `[extruder1]` … — 挤出机

热端加热器 + 挤出运动（E 轴）。编号兄弟 `[extruder1]` … 没有自己的工厂，由
`[extruder]` 的装载**连带读取**并按节名注册；`gcode_id` 内定为 `T0` / `T1` …。
选项 = 上表的加热器共用选项 +：

| 参数 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `nozzle_diameter` | 浮点 (mm) | 是 | — | `> 0` |
| `filament_diameter` | 浮点 (mm) | 是 | — | `≥ nozzle_diameter` |
| `max_extrude_cross_section` | 浮点 (mm²) | 否 | `4 × nozzle_diameter²` | 单次挤出截面上限 |
| `max_extrude_only_distance` | 浮点 (mm) | 否 | `50` | 纯挤出移动的长度上限 |
| `max_extrude_only_velocity` | 浮点 (mm/s) | 否 | ready 时由 `max_velocity` 推导 | 纯挤出速度上限 |
| `max_extrude_only_accel` | 浮点 (mm/s²) | 否 | ready 时由 `max_accel` 推导 | 纯挤出加速度上限 |
| `instantaneous_corner_velocity` | 浮点 (mm/s) | 否 | `1.0` | 挤出拐角的瞬时速度 |
| `pressure_advance` | 浮点 | 否 | `0.0` | 压力推进，`≥ 0`（运行期用 `SET_PRESSURE_ADVANCE` 调）——**现已真驱动挤出机运动**：求解器按 `pa × 速度` 提前/回抽，再按 `smooth_time` 做三角窗平滑 |
| `pressure_advance_smooth_time` | 浮点 (s) | 否 | `0.040` | PA 平滑窗口，`(0, 0.200]`；`pressure_advance: 0` 时自动归零（上游同） |
| `step_pin` / `dir_pin` / `rotation_distance` / `microsteps` … | 同 `[stepper_*]` | 否 | — | **写了任一个**才建 E 轴 stepper（上游同规则）；不写则纯加热 |

命令：`M104` / `M109`（等待到目标温度）/ `SET_PRESSURE_ADVANCE`（mux `EXTRUDER`）/
`ACTIVATE_EXTRUDER`。只有名字是 `extruder` 的主挤出机额外注册 `EXTRUDER` 默认项。

### `[heater_bed]` — 热床

选项 = 加热器共用选项（`gcode_id` 内定为 `B`），注册 `M140` / `M190`（等待到目标温度）。

```ini
[heater_bed]
heater_pin: PB5
sensor_type: EPCOS 100K B57560G104F
sensor_pin: PC0
control: watermark
max_delta: 2.0
min_temp: 0
max_temp: 110
```

### `[heater_generic <name>]` — 任意命名的加热器

选项 = 加热器共用选项 + `gcode_id`（字符串，可选，用于 `M105` 报告）。经
`SET_HEATER_TEMPERATURE HEATER=<name>` 控制。

### `[fan]` — 风扇（`M106` / `M107`）

| 参数 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `pin` | 引脚 | 是 | — | PWM 输出脚 |
| `max_power` | 浮点 (0..1] | 否 | `1.0` | 占空比上限 |
| `kick_start_time` | 浮点 (s) | 否 | `0.1` | 静止起步时的满速时间，`≥ 0` |
| `off_below` | 浮点 [0..1] | 否 | `0.0` | 低于该请求值直接关 |
| `cycle_time` | 浮点 (s) | 否 | `0.010` | PWM 周期，`> 0` |
| `hardware_pwm` | 布尔 | 否 | `false` | 用固件 PWM 而非软件翻转 |
| `shutdown_speed` | 浮点 [0..1] | 否 | `0` | 主机停机时固件回退的占空比（被 `max_power` 再截一次） |
| `enable_pin` | 引脚 | 否 | — | 驱动使能脚，只在 0 ↔ 非 0 边沿翻转 |
| `tachometer_pin` | — | 可选 | — | 批 #8 接通 `pulse_counter` 频率计数：报 `rpm`（无边沿时 `rpm: 0.0`，不再拒收、也不静默 `rpm: null`） |
| `tachometer_ppr` | 2 | ≥1 | — | 每转脉冲数（需 `tachometer_pin`；`<1` 拒收） |
| `tachometer_poll_interval` | 0.0015 | >0 | — | 测速轮询间隔秒（需 `tachometer_pin`；`≤0` 拒收） |

> 速度变更**按打印时间生效**（同 `SET_PIN`，经 `GCodeRequestQueue` 钉到前瞻时刻、
> flush 时落 PWM，kick-start 尾巴是队列重跑）；**两条兑底**：无 `[printer]`（lookup 不到
> toolhead）或资源无法定时（`min_schedule_time` 不可得）时立即生效——此时 kick 尾巴才用
> `call_later`。

### `[temperature_sensor <name>]` — 可查询的温度传感器

把任意已注册的传感器类型暴露成一个可 `objects/query` 的对象。`get_status` 报
`temperature` / `measured_min_temp` / `measured_max_temp`（保留 2 位；未上报过的读数 0
不计入 min/max）。

| 参数 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `sensor_type` | 字符串 | 是 | — | 传感器类型，查下表 |
| `min_temp` | 浮点 (°C) | 否 | `-273.15` | 报警下限 |
| `max_temp` | 浮点 (°C) | 否 | `999999999.9` | 报警上限，`> min_temp` |
| 其余 | 视类型 | — | — | 所选传感器工厂的选项（如 DS18B20 的 `serial_no`） |

`sensor_type` 可选值（当前已注册的工厂）：

| 类 | 类型名 |
|----|--------|
| ADC 电压型 | `AD595` `AD597` `AD8494` `AD8495` `AD8496` `AD8497` |
| 电阻/热敏（内建表） | `PT1000`、`PT100 INA826`，以及 8 个内建热敏：`ATC Semitec 104GT-2`、`ATC Semitec 104NT-4-R025H42G`、`EPCOS 100K B57560G104F`、`Generic 3950`、`SliceEngineering 450`、`TDK NTCG104LH104JT1`、`Honeywell 100K 135-104LAG-J01`、`NTC 100K MGB18-104F39050L32` |
| SPI 热电偶 / RTD | `MAX6675` `MAX31855` `MAX31856` `MAX31865`（选项含 `tc_averaging_count` / `tc_use_50Hz_filter` / `rtd_nominal_r` / `rtd_num_of_wires` / `rtd_use_50Hz_filter`，以及 SPI 共用的 `spi_bus` / `spi_mcu` / `cs_pin` 等） |
| 1-wire | `DS18B20`（`serial_no` 必需、`sensor_mcu` 必需、`ds18_report_time` 默认 3.0 s ≥ 1.0 s） |
| MCU 内部 | `temperature_mcu`（可带 `sensor_mcu`，默认 `mcu`；它写的是**芯片名**，即 `[mcu 下游名]` 里的 `下游名`，如 `sensor_mcu: downstream` 对应 printer 对象 `mcu downstream`，与上游 `'%s:ADC_TEMPERATURE' % mcu_name` 同口径） |
| 组合 | `temperature_combined`（`sensor_list` 必需、`maximum_deviation` ≥ 0、`combination_method` ∈ `min`/`max`/`mean` 必需） |
| 主机热区 | `temperature_host`（`sensor_path` 可选，默认 `/sys/class/thermal/thermal_zone0/temp`，见下节） |
| 自定义 | 用 `[thermistor <name>]` / `[adc_temperature <name>]` 定义后按 `<name>` 引用（见下节） |

```ini
[temperature_sensor mcu_temp]
sensor_type: temperature_mcu
min_temp: 0
max_temp: 85

[temperature_sensor hotend]
sensor_type: DS18B20
sensor_mcu: mcu
serial_no: 28ff00abcdef
max_temp: 300
```

### `[temperature_host]` — 主机热区温度传感器（工厂注册节）

裸节本身**没有选项**（写任何选项都会被判 `Option '…' is not valid in section
'temperature_host'`），唯一作用是把 `temperature_host` 传感器工厂注册给 `heaters`
（上游经 `temperature_sensors.cfg` 装载该节的同款语义）。真正的传感器在**消费节**里建：

```ini
[temperature_sensor host]
sensor_type: temperature_host
# sensor_path 可选，默认 /sys/class/thermal/thermal_zone0/temp
# min_temp / max_temp 也写在这个节里
```

- 每 1 s 读一次内核的毫制摄氏度（÷1000），`min_temp`/`max_temp` 越界即
  `invoke_shutdown("HOST temperature … below/above …")`（文案逐字）；
- 传感器自身也是对象 `temperature_host <name>`（`<name>` 取消费节名的最后一词），
  `get_status` 只有 `temperature`（round2）；构造时打不开文件报
  `Unable to open temperature file '<path>'`；
- `--debugoutput` 下跳过开文件与定时器（对应回放/调试输出模式）；读失败时
  `temperature` 置 0 且采样定时器退役（上游 `reactor.NEVER` 同款）。

### `[temperature_probe <name>]` — 探针温度传感器（eddy 漂移补偿的温度通道）

内层先按 `sensor_type` 建一个普通传感器（与 `[temperature_sensor]` 同一 `heaters` 工厂机制），
本节再把它包成可 `objects/query` 的对象 `temperature_probe <name>`。核心与标定命令族
均已落地（A `19e7eb4`、B `65866da`——`calibration_position` / `calibration_bed_temp` /
`calibration_extruder_temp` / `extruder_heating_z` / `resting_z` / `horizontal_move_z` / `speed`
由 `TEMPERATURE_PROBE_*` 命令族消费）；漂移补偿 `EddyDriftCompensation` 也已落地
（C `1f0d623`）：需**同名** `[probe_eddy_current <name>]` 段才会注册，`get_status` 的
`compensation_enabled` 是真实开关。

| 参数 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `sensor_type` | 字符串 | 是 | — | 内层传感器类型（查传感器类型表） |
| `gcode_id` | 字符串 | 否 | — | `M105` 报告用 ID（`heaters` 侧读取） |
| `speed` | 浮点 (mm/s) | 否 | — | 校准移动速度（`> 0`）；缺省时取 probe/lift 速度的较大者 |
| `horizontal_move_z` | 浮点 (mm) | 否 | `2.0` | 点间抬升，`> 0` |
| `resting_z` | 浮点 (mm) | 否 | `0.4` | 采样间停驻高度，`> 0` |
| `calibration_position` | 3 浮点 | 否 | — | 校准起始位置 `x y z`（count 措辞同上游） |
| `calibration_bed_temp` | 浮点 (°C) | 否 | — | 校准期热床目标，`> 50` |
| `calibration_extruder_temp` | 浮点 (°C) | 否 | — | 校准期挤出机目标，`> 50` |
| `extruder_heating_z` | 浮点 (mm) | 否 | `50.0` | 加热期间的停靠高度，`> 0` |
| `smooth_time` | 浮点 (s) | 否 | `2.0` | 读数平滑窗口，`> 0` |
| `min_temp` | 浮点 (°C) | 否 | `-273.15` | 报警下限（`minval` 同值） |
| `max_temp` | 浮点 (°C) | 否 | `99999999.9` | 报警上限，`> min_temp` |
| `calibration_temp` | 浮点 (°C) | 否 | `0.0` | 漂移校准温度（`note_z_calibration_*` 经 `SAVE_CONFIG` 写回） |
| `max_validation_temp` | 浮点 (°C) | 否 | `60.0` | 漂移校验允许的最高温度 |
| `drift_calibration_min_temp` | 浮点 (°C) | 否 | `0.0` | 漂移曲线适用下限（`finish` 时随曲线写回） |
| `drift_calibration` | 两层列表 | 否 | — | 9 段二次多项式系数（`fit` 输出，SAVE_CONFIG 写出；坏曲线报 `Invalid polynomial in drift calibration`） |

> 上述四个漂移选项**仅在同名 `[probe_eddy_current]` 段存在时**被读取（与上游一致），
> 否则会被未用选项校验拒绝。另：`drift_calibration` 写为空视为「未配置」
> （上游空值会崩在 `_check_calibration` 拒载，本仓取代码注释声明的取舍）。

`get_status` 六键：`temperature`（平滑值，**不**舍入）、`measured_min_temp` / `measured_max_temp`
（round2）、`in_calibration`、`estimated_expansion`、`compensation_enabled`（C 起为真实开关，无 eddy 段时恒 `false`）。

### `[thermistor <name>]` / `[adc_temperature <name>]` — 自定义传感器定义

定义一个新的传感器类型供 `sensor_type` 引用；节的 `<name>` 就是注册的类型名。定义与引用在本仓**与文件中先后无关**（两族都声明为 `phase = early`，见开发手册的相位通则）。两者的差别
是标定点的写法（与上游一致，`[adc_temperature]` 按是否给 `resistance1` 区分电阻/电压型）：

| 参数 | 类型 | 说明 |
|------|------|------|
| `temperature1..N` | 浮点 (°C) | 标定点温度，从 1 起连续编号，至少 2 组（单点 + `beta` 形式除外） |
| `resistance1..N` | 浮点 (Ω) | 该点电阻（`[thermistor]` 必需，定义电阻型） |
| `voltage1..N` | 浮点 (V) | 该点电压（`[adc_temperature]` 用它定义线性电压型） |
| `beta` | 浮点 | 可选：单点 + Beta 模型（`temperature1`/`resistance1`/`beta`） |
| `pullup_resistor` | 浮点 (Ω) | 可选，默认 `4700`（在**使用该类型的节**里写，如 `[extruder]`） |
| `inline_resistor` | 浮点 (Ω) | 可选，默认 `0`（同上） |
| `adc_voltage` | 浮点 (V) | 可选，默认 `5.0`（电压型，同上） |
| `voltage_offset` | 浮点 (V) | 可选，默认 `0.0`（电压型，同上） |

使用方（如 `[extruder]`）仍要写 `sensor_type: <name>` 与 `sensor_pin`。

```ini
[thermistor my_ntc]
temperature1: 25
resistance1: 100000
temperature2: 150
resistance2: 1641.9
beta: 3950

[adc_temperature my_volts]
temperature1: 0
voltage1: 0.5
temperature2: 100
voltage2: 4.5
```

裸 `[adc_temperature]`（不带名字）是上游“装载本模块默认传感器”的开关，一般不需要手写。

### `[static_digital_output <name>]` — 开机即定的静态输出

把一组引脚在**配置期**一次性拉到固定电平（如跳线选微步），此后不改；无 oid、不占资源。

| 参数 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `pins` | 逗号分隔的引脚列表 | 是 | 每个可带 `!` 取反；**重复写多行 `pins:` 只有最后一行生效**（与上游解析器一致），要多个引脚就写一行逗号分隔 |

对象无 `get_status`，不出现在 `objects/list`。`order = 35` 排在 `[board_pins]` 之后，别名可用。

```ini
[static_digital_output ms_select]
pins: !PD0, PD1, PD2
```

### `[static_pwm_clock <name>]` — 固定频率、50% 占空比的时钟输出

只消费一个引脚（**不是 pin 芯片**，不注册可被 `pin:` 引用的 chip），建硬件 PWM 并按 `frequency` 输出方波（0.5 / 0.5、`max_duration = 0`）；典型用途是给外部传感器送时钟。

| 参数 | 默认 | 说明 |
|------|------|------|
| `pin` | 必需 | PWM 引脚（可带 `!`） |
| `frequency` | `100` | 输出频率（Hz），必须 > 1/0.3 ≈ 3.3333 且 ≤ 520000000 |

周期必须能被 MCU 时钟整除：`1/frequency` 换算出的 tick 数乘回频率须等于 MCU 时钟频率，否则 build 期报 `Frequency output must be without remainder`（上游同一算法/同一文案）。脉宽低于 40% 或高于 60% 只告警。无 `get_status`。`order = 35`，排在 `[board_pins]` 之后，别名可用。

```ini
[static_pwm_clock ldc1612_clk_in]
pin: carto:PB4
frequency: 24000000
```

### `[duplicate_pin_override]` — 允许一个 pin 被多处使用

上游的“逃生门”：把一个 pin 在**这里**声明一次，之后多个节可以各自占用它（仍按节序解析，本仓按固定装载表）。

| 参数 | 必需 | 说明 |
|------|------|------|
| `pins` | 是 | 逗号分隔的引脚描述 |

- 描述里**不能带修饰符**（`!PA1`/`^PA1` 报 `Invalid pin description`）——极性属于真正驱动它的那一侧；
- 别名按**拼写**记账：`pins: my_alias` 只覆盖同样写 `my_alias` 的节，不覆盖它的目标引脚（上游同）；
- 装载相位 `phase = early`、`order = 17`，在 `[mcu]`(10)/`[multi_pin]`(15)/`[sx1509]`(16) 之后、其余泛型节之前——**晚于 17 才注册的芯片（如 `[adc_scaled]` 的虚拟 pin）不能在这里声明**（上游靠文件顺序，本仓靠固定表）；
- 无资源、无命令、无 `get_status`。语料里没有任何配置使用它。

```ini
[duplicate_pin_override]
pins: PA1, my_alias
```

---

- [← 用户手册首页](README.md)

### `[bed_tilt]` — 床面倾斜补偿（`BED_TILT_CALIBRATE`）

| 选项 | 默认 | 说明 |
|------|------|------|
| `x_adjust` / `y_adjust` / `z_adjust` | `0.0` | 补偿平面；也是 `SAVE_CONFIG` 的回写项 |
| `points` | 无 | 校准探测点，每项 `x,y`，**至少 3 点**；**只有写了它才会注册 `BED_TILT_CALIBRATE`** |
| `horizontal_move_z` | `5.0` | 点间抬升高度 |
| `speed` | `50.0` | 点间移动速度 |

补偿方式是把自己装成 `gcode_move` 的移动变换：G-Code 空间的 Z 按平面加/减，喷嘴相对床面的
实际 Z 保持平整；`M114`/`GET_POSITION` 看到的是补偿后的值。`BED_TILT_CALIBRATE` 逐点探测后
用 `coordinate_descent` 拟合平面并立即应用，同时把三项以 `%.6f` 记入 `SAVE_CONFIG` 待写区
（`SAVE_CONFIG` 命令把这些回写项写回文件并重启，见配置装载）。

### `[z_tilt]` — 多 Z 电机调平（`Z_TILT_ADJUST`）

| 选项 | 默认 | 说明 |
|------|------|------|
| `z_positions` | **必需** | 每个 Z 电机在床面坐标系的 `x,y`，**项数必须等于 Z 电机数**（如 2 个电机 2 项） |
| `points` | **必需** | 校准探测点 `x,y`，**至少 2 点** |
| `horizontal_move_z` | `5.0` | 点间抬升 |
| `speed` | `50.0` | 点间移动（`>0`） |
| `retries` | `0` | 点高差超容差时的重试次数（`>=0`） |
| `retry_tolerance` | `0.0` | 判定收敛的点高差范围（`>0`） |

### `[quad_gantry_level]` — 四点龙门调平（`QUAD_GANTRY_LEVEL`）

| 选项 | 默认 | 说明 |
|------|------|------|
| `gantry_corners` | **必需** | 龙门两侧角点 `x,y`，**至少 2 项** |
| `points` | **必需** | 校准探测点，**恰好 4 点** |
| `horizontal_move_z` | `5.0` | 点间抬升（注意：命令行 `HORIZONTAL_MOVE_Z` 不覆盖它，与上游一致） |
| `speed` | `50.0` | 点间移动 |
| `retries` / `retry_tolerance` | `0` / `0.0` | 重试（错误信息附 `Possibly Z motor numbering is wrong`） |
| `max_adjust` | `4.0` | 单电机调整量上限（`>0`），超过即中止 |

### `[bltouch]` — BLTouch 探针（替代 `[probe]` 的物理实现）

| 选项 | 默认 | 说明 |
|------|------|------|
| `control_pin` | **必需** | 传感器控制脚；占空比即单线协议（按上游 `Commands` 表发命令） |
| `sensor_pin` | **必需** | 触发检测脚（endstop） |
| `z_offset` | **必需** | 探针相对喷嘴的 Z 偏移 |
| `x_offset` / `y_offset` | `0.0` | XY 偏移 |
| `stow_on_each_sample` | `true` | 每次采样后收针 |
| `pin_up_reports_not_triggered` | `true` | 抬针到位即视为未触发 |
| `pin_up_touch_mode_reports_triggered` | `true` | 触摸模式语义 |
| `probe_with_touch_mode` | `false` | 用触摸模式探测 |
| `set_output_mode` | 上游默认 | 输出模式（5V/OD 等） |
| `pin_move_time` | 上游默认 | 抬/落针时序 |

探针族共用选项（`speed`/`lift_speed`/`samples`/`sample_retract_dist`/`samples_result`（默认 `average`）/
`samples_tolerance`/`samples_tolerance_retries`/`horizontal_move_z` 等）由 `probe` 模块在本节上读取。
装载时把自己注册为 `probe` 对象与 `probe` 虚拟 chip（`probe:z_virtual_endstop`），命令见
`BLTOUCH_DEBUG`/`BLTOUCH_STORE` 与探针族三条。

### `[smart_effector]` — Smart Effector 探针

| 选项 | 默认 | 说明 |
|------|------|------|
| `pin` | **必需** | 触发脚（endstop） |
| `control_pin` | 可选 | 控制脚（数字输出）；**只有配置它才注册 `RESET_SMART_EFFECTOR`** |
| `z_offset` | **必需** | 触发偏移 |
| `probe_accel` | 上游默认 | 探测时允许的加速度（`SET_SMART_EFFECTER` 可运行期改） |

同 BLTouch：把自己注册为 `probe` 对象与 `probe` 虚拟 chip，探针族共用选项由 `probe` 模块读取。
`SET_SMART_EFFECTER` 可改 `SENSITIVITY`(0..255)/`ACCEL`(≥0)/`RECOVERY_TIME`(≥0)；注意本模块
的 `probe_accel`/`recovery_time` 在探针移动时的往返钩子（上游 `probe_prepare/finish`）尚未接线
（语料不触发，见 developer-manual 的缺口说明）。

### [screws_tilt_adjust]

探针依次测每个调平螺丝上方的床面高度，`SCREWS_TILT_CALCULATE` 据此输出每颗螺丝该拧多少、往哪拧（时钟记法）。

| 选项 | 默认 | 说明 |
|------|------|------|
| `screw1` … `screw99` | — | 每项两个浮点（X Y）；**数到第一个缺失编号即停**，少于 3 颗拒绝装载（`screws_tilt_adjust: Must have at least three screws`，上游文案；`Need at least 3 probe points …` 仅在显式 `points` 少于 3 时由 probe 侧报出） |
| `screwN_name` | `screw at %.3f,%.3f` | 报告行中的螺丝名 |
| `screw_thread` | `CW-M3` | 螺距/旋向选择表：`CW-M3` `CCW-M3` `CW-M4` `CCW-M4` `CW-M5` `CCW-M5` `CW-M6` `CCW-M6`（8 项，表外值报错） |
| `points` | 全部螺丝坐标 | 探测点（`ProbePointsHelper`，缺省即螺丝位置） |
| `horizontal_move_z` / `speed` | `5.` / `50.` | 采样间抬升高度 / 移动速度（同上） |

命令参数（`MAX_DEVIATION`、`DIRECTION`）、输出格式与状态形状见命令参考的 `SCREWS_TILT_CALCULATE` 条目。`get_status` 暴露三键：`error`、`max_deviation`、`results`（按 `screwN` 分键）。

### [gcode_arcs]

| 选项 | 默认 | 说明 |
|------|------|------|
| `resolution` | `1.` | 每弧段毫米数（above 0） |

`G2`/`G3` 弧与 `G17`/`G18`/`G19` 平面选择已注册（2026-10-21）：`G2/G3` 接受 `X/Y/Z/I/J/K/E/F`，按 `planArc` 分段、每段走与手写 `G1` 同一条处理链（坐标变换 / `G92` / 因子均生效）。

三条按上游的约定：

- **`R` 被拒**：`G2/G3 does not support R moves`（不做半径形式）；
- **`P` 不解析**（上游未读取该词，静默忽略）；
- **平面语义**：`G17`（缺省）= `I/J` ↔ X/Y、Z 螺旋；`G18` = `I/K` ↔ X/Z、Y 螺旋；`G19` = `J/K` ↔ Y/Z、X 螺旋；`G91` 相对模式下拒绝弧（`does not support relative move mode`）；无 `I/J`、`I/K` 或 `J/K` 时报 `requires IJ, IK or JK parameters`。

段数无上限（上游同）：`resolution` 极小 + 大半径会产生很多段。

### [bed_screws]

| 选项 | 默认 | 说明 |
|------|------|------|
| `screw1` … `screw99` | — | 每项两浮点（X Y）；**数到第一个缺失编号即停**，少于 3 颗报 `bed_screws: Must have at least three screws` |
| `screwN_name` | `screw at %.3f,%.3f` | 螺丝显示名 |
| `screwN_fine_adjust` | — | 该螺丝精调坐标（记在粗调名下） |
| `speed` / `probe_speed` | `50.` / `5.` | 行进/探入速度（均 above 0） |
| `horizontal_move_z` / `probe_height` | `5.` / `0.` | 抬升/下探 |

命令：`BED_SCREWS_ADJUST`（常驻）；会话开始后额外注册 `ACCEPT` / `ADJUSTED` / `ABORT`，收尾（`ABORT`、走完或 move 被拒）时注销。
流程：抬到 `horizontal_move_z` → 移到该螺丝 → 下探到 `probe_height`，按 `ACCEPT` 逐颗走完粗调再走精调；`ADJUSTED` 把该螺丝的 `-1` 归 `0` 重新数；`ABORT` 结束会话。
该命令族**不用** `probe`/`manual_probe`/`configfile`（上游同样只用 `toolhead.manual_move`）；与 `manual_probe` 的 `ACCEPT`/`ABORT` 同名，两者同时活跃时按「先注册者占坑」（与上游一致）。

### [pwm_cycle_time <name>] / [pwm_tool <name>]

| 选项 | `pwm_cycle_time` | `pwm_tool` |
|------|------|------|
| `pin` | 必填 | 必填 |
| `cycle_time` | `0.1`（above 0） | `0.1`（above 0） |
| `scale` | `1`（>0） | `1`（>0） |
| `value` / `shutdown_value` | `0`（0..=scale，除以 scale） | `0`（同左；设了 `maximum_mcu_duration` 时两者须相等，build 期拒绝） |
| `hardware_pwm` | —（恒软件 PWM） | `false` |
| `maximum_mcu_duration` | — | `0`=永不兕底；写则 ≥0.5 秒 |

`pwm_cycle_time` 节的 `SET_PIN` 可带 `CYCLE_TIME=`（秒）：运行期只更新主机记账，固件周期 build 期固定。

### [temperature_fan <name>] / [controller_fan <name>]

| 选项 | `temperature_fan` | `controller_fan` |
|------|------|------|
| `pin` | 必填 | 必填 |
| `min_temp` / `max_temp` | 必填（> -273.15 / > min） | — |
| `control` | 必填：`watermark` 或 `pid` | — |
| `max_delta` / `pid_Kp·Ki·Kd` / `pid_deriv_time` | watermark：`2.0`；pid：三项必填（除以 255）、导数窗 `2.0` | — |
| `max_speed` / `min_speed` / `target_temp` | `1` / `0.3` / `40`（max_temp<40 时取之） | — |
| `sensor_type` / `sensor_pin` | 由 heaters 传感器工厂读取 | — |
| `stepper` / `heater` | — | 默认全部步进 / 默认 `extruder`（按对象名，H1 无短名注册表） |
| `fan_speed` / `idle_speed` / `idle_timeout` | — | `1` / 默认=fan_speed / `30`（≥0） |

命令：`SET_TEMPERATURE_FAN_TARGET`（mux 键 `TEMPERATURE_FAN=`，见命令参考）；`controller_fan` 的 per-second tick 自 `klippy:ready` 起，stepper/heater 引用在 connect 期解析（上游文案）。

### [gcode_macro <名>]

| 选项 | 默认 | 说明 |
|------|------|------|
| `gcode` | 必填 | 宏体 |
| `description` | `G-Code macro` | 命令 help 文本 |
| `rename_existing` | — | 被改名的命令：装载期只做同型检查，**连接期把该命令的旧处理器位移到这个名下、本宏接管原名**（2026-10-06）；目标名须未被占用，已占用时报 `gcode command … already registered`（与上游 `gcode.py:142-144` 同款） |
| `variable_<名>` | — | 字面量，`get_status` 可见（只读）。**按 Python 字面量解析**（`{'restore': False, 'temp': 0}`、`False`、单/双引号串、list/tuple/dict、尾随逗号；JSON 写法 `true/false/null` 也收），与上游 `ast.literal_eval` 同口径 |

宏即命令（大写注册）；**宏体已渲染执行**（渲染引擎是 `extras/template.rs` 的 **minijinja 2.24 适配层**：单花括号定界符 `{`/`{%`/`{#}`、缺键即报错、装载期编译，渲染后经 gcode 派发）；`SET_GCODE_VARIABLE` 已注册（变量可写，`VALUE=` 同样按 Python 字面量解析）；裸 `[gcode_macro]` 为共享模板持有者（零选项）。`{% set %}` 作用域对齐 Jinja2 3.1.6（顶层/`if` 块外泄、`for` 块不外泄）；`namespace()`、关键字实参、`{% block %}`、`|float(默认)` 等 Jinja2 构型均可解析（`template.rs` 的三个步进宏字面用例即实测：`namespace(phase=0)`、`{% set count.phase %}`、if/elif 链、for+range、`G4 P` 值与 DIR 正反转）。字符串语义也照 Python：`str` 有 `split`/`strip`/`replace`/`startswith`/`endswith`/`join`/`find`/`count`/`lower`/`upper`，`list` 有 `index`/`count`；**标签打印容器时按 Python `str()` 拼写**（`{'call': 'PAUSE'}`）；`"…%s…" % x` 这类 printf 格式化会改写成单一 `|printf(...)` 过滤器（整数码对浮点按 Python 截断）（只收左侧是字符串字面量的形态）。仍缺的在缝上：无（`printer.objects` 的反射式枚举已于 2026-10-21 落地——`printer` 本身按上游语义可枚举出 `is_queryable` 的对象名且不读 status，`printer.objects` 是同名单/顺序/缓存的本仓等价视图，两者都能 `|length`）。与 Jinja2 的已知差异（`%`/`//` 为欧几里得取余、Strict 下缺键在打印/迭代/判真时报错、`|int`/`|float` 不可转换且无默认时报错、`~` 拼接容器不走 formatter、**列表不可变所以 `append`/`extend` 返回 `None` 且收集不到元素**、部分错误 detail 措辞）见 `extras/template.rs` 模块文档。

### [led <name>] / [neopixel <name>] / [dotstar <name>] / [pca9533 <name>] / [pca9632 <name>] / [display_template <name>]

| 选项 | 适用 | 说明 |
|------|------|------|
| `initial_RED`/`initial_GREEN`/`initial_BLUE`/`initial_WHITE` | 各灯段 | 0..=1 初值（驱动无该色时忽略） |
| `cycle_time`/`hardware_pwm`/`red_pin`…`white_pin` | `led` | PWM/引脚组（`led.py:113-115`） |
| `pin`/`chain_count`/`color_order` | `neopixel` | 链长与色序（`neopixel.py:28-29`） |
| `data_pin`/`clock_pin`/`chain_count` | `dotstar` | 双线链（`dotstar.py:24`） |
| `i2c_*`（bus 选项组） | `pca9533`/`pca9632` | I2C 地址/速率等（`bus.py:302`）；`pca9632` 另有 `color_order` |
| `text`、`param_*` | `display_template` | 模板文本与参数（惰性对象，`is_queryable=false`） |

六段已落地（2026-09-24 批 #2）；`SET_LED`/`SET_LED_TEMPLATE` 未注册（H3/H8），模板 `text` 不渲染（渲染引擎已在 `extras/template.rs`，缺的是 `SET_LED_TEMPLATE` 这条缝）。

### [extruder_stepper <name>]

| 选项 | 默认 | 说明 |
|------|------|------|
| 电机组 | — | 经 `PrinterStepper` 读取（步距等同上游） |
| `extruder` | `extruder` | 绑定的挤出机名（connect 期校验，上游文案 `'<名>' is not a valid extruder.`） |
| `pressure_advance` / `pressure_advance_smooth_time` | `0.` / `0.040` | 本步进**自有**的压力推进（不继承 `[extruder]`，依据上游 `kinematics/extruder.py:12-16`）；2026-10-21 起**真驱动自己的求解器**：装载时 `0` 时刻落表（对齐上游 `_handle_connect` 的 `espa(sk, 0., …)`），`SET_PRESSURE_ADVANCE` 与主挤出机共用同一条应用路径（改值→上报立即、落表排 lookahead；`smooth_time` 变更→flush + 从头重算）；`pa = 0` 走逐点名义位置 |

段已落地（批 #2）；`SET_PRESSURE_ADVANCE [EXTRUDER=<段名>]` 按名挂值 ✓；**宿主 step 同步 2026-10-21 落地**：`ToolHead::connect` 把本段电机收到绑定挤出机的 trapq 上（未绑 ⇒ 不挂在任何 trapq、生成器跳过），`SYNC_EXTRUDER_MOTION EXTRUDER=<挤出机> STEPPER=<本段> MOTION_QUEUE=<名|''>` 运行期重指/解绑，`SET_EXTRUDER_ROTATION_DISTANCE [EXTRUDER=<段名>]` 改步距后步数换算跟着变。

### [exclude_object]

零选项段（上游 `exclude_object.py:12-37`）。装载时注册 `EXCLUDE_OBJECT_START`/`_END`/`EXCLUDE_OBJECT`/`EXCLUDE_OBJECT_DEFINE` 四命令并持有对象状态；排除区内移动被 `MoveTarget` 变换丢弃，**离区时按上游扣被丢弃 prime 段的 E**（`offset[3]`/`extruder_adj`，批 #4）。**变换是链式的**（2026-10-19）：它转发给被自己顶替的占用者（`bed_mesh`/`bed_tilt` 先注册时就是它们），注销时交还——因此与网格/床倾斜同用**不会再丢它们的 Z 补偿**（此前会）。

### [virtual_sdcard] / [display_status] / [homing_override] / [sdcard_loop]

| 节 | 选项 | 说明 |
|---|------|------|
| `virtual_sdcard` | `path`（必）、`on_error_gcode`（默认=上游 `DEFAULT_ERROR_GCODE`） | 回放命令族已注册（`extras/virtual_sdcard.rs:449-470`）：`M20`–`M27`、`M28`–`M30`（上游同款 `SD write not supported`，`virtual_sdcard.py:41,143`）、`SDCARD_RESET_FILE`/`SDCARD_PRINT_FILE` |
| `display_status` | 无 | `M73`/`M117`/`SET_DISPLAY_TEXT` 已注册（批 #7）；进度/消息由这两条命令驱动（无 `[display]` 时也按需创建） |
| `homing_override` | `axes`（默认 `XYZ`）、`set_position_x/y/z`（无）、`gcode`（必） | **接管 `G28`**（`order = 70, phase = late`，晚于 `[printer]` 的注册）：按 `axes` 掩码选择性接管、`G28` 的参数进模板的 `params`/`rawparams`、脚本体里再调 `G28` 回落到原 handler；与 `[safe_z_home]` 互斥 |
| `sdcard_loop` | 无 | `SDCARD_LOOP_BEGIN/_END/_DESIST` 未注册；栈/索引语义已单测钉住 |

### [dual_carriage]

| 选项 | 说明 |
|------|------|
| `axis`（X/Y/Z）、`safe_distance`（毫米） | 第二滑架轴与最近距，按上游读取（`idex_modes.py`/`cartesian.py:24-34`） |
| `primary_carriage`（generic 形态，**可选**） | 无它 = 该轴的**主动滑架**（`axis` 必填、无 `safe_distance`）；有它 = 从动滑架（`axis` 可选、仅作交叉校验，`safe_distance` 生效）。同主滑架的两个 dual 会报重（按**主滑架名**判，非按轴）；`[extra_carriage <name>]` 的 `primary_carriage` **仍必填**（批 #27） |
| 电机组 | 经 `PrinterStepper`（同上游 `LookupMultiRail`） |

cartesian 在 late 阶段认领；注册 `dual_carriage` 对象与 `SET_DUAL_CARRIAGE`/`SAVE_…_STATE`/`RESTORE_…_STATE`。**轨间坐标交接已实现**（批 #4，`toggle_active_dc_rail` 语义：切换/恢复携带 gcode 坐标）；步进仍仅主轨（`updateLimits` 未移植，C1）。已转绿（2026-09-24）。**generic 形态**（批 #12）：`[carriage <name>]`/`[dual_carriage <name>]`/`[extra_carriage <name>]`/`[stepper <name>]` 由运动学注册同一对象与三命令（`dual_carriage` status 报 `active_carriage` + `carriages`），`[printer]` 的 `max_z_velocity/max_z_accel` 经 `carriage::build` 进入 generic 归零（原写死 0 为首因）。

### [servo <name>]

| 选项 | 默认 | 约束 |
|------|------|------|
| `pin` | 必填 | PWM 引脚 |
| `minimum_pulse_width` | `0.001` s | >0 且 <0.020 |
| `maximum_pulse_width` | `0.002` s | >minimum 且 <0.020 |
| `maximum_servo_angle` | `180` | 最大脉宽对应角 |
| `initial_angle` | — | 0..=360；缺省用 `initial_pulse_width` |
| `initial_pulse_width` | `0` | 0..=maximum_pulse_width |

`SET_SERVO`（mux 键 `SERVO=`，ANGLE/WIDTH）已注册；无打印时序排程（同 pwm_tool 既有口径）。

### [probe_eddy_current <名>]

| 选项 | 说明 |
|------|------|
| `i2c_mcu` / `i2c_bus` / `i2c_address` | I2C 组（`setup_i2c` 共享读取） |
| `sensor_type` | 传感器类型（语料 `eddy`） |
| `z_offset` | 探针 Z 偏移 |
| `calibrate` | 静态标定开关（`enable`/`enable_with_touch`；**已接**——2026-10-03 `a6004f7` 由 `PROBE_EDDY_CURRENT_CALIBRATE` 生成，`Z_OFFSET_APPLY_PROBE METHOD=tap` 写回 `tap_z_offset`） |
| 采样组（`samples`/`sample_retract_dist`/`samples_tolerance`/`lift_speed`…） | 由 probe/`ProbePointsHelper` 读取 |

段与对象已落地（2026-09-24 批 #3，`eddy.test` 转绿）；**同名 section 按上游 `strict=False` 后写覆盖合并**（本仓自 2026-09-24 起，eddy.cfg 双段即其用例）。静态标定与 `PROBE_EDDY_CURRENT_CALIBRATE`/`Z_OFFSET_APPLY_PROBE` 已实现（2026-10-03，`a6004f7`）。

> 配置解析补充（批 #5）：除同名 section 合并外，`#*# SAVE_CONFIG` 自动保存块按上游 `_find_autosave_data` **读取合并**（header 逐字节识别、`#*# ` 前缀剥离、正文优先/块只补新、损坏行告警）——语料 `delta_calibrate.cfg` 的双段与高度数据即其用例（回写侧 `SAVE_CONFIG` 命令把待写项写回文件并重启）。

### `[input_shaper]` — 输入整形参数（wave-2）

| 选项 | 默认 | 说明 |
|------|------|------|
| `shaper_type` / `shaper_type_<轴>` | `mzv` | 整形器类型（`zv`/`zvd`/`mzv`/`ei`/`2hump_ei`/`3hump_ei`，可带 `(n,t)`、`(v_tol=)` 括号参数） |
| `shaper_freq_<轴>` | 0（禁用） | 整形频率 Hz |
| `damping_ratio_<轴>` | `0.1` | 阻尼比；超上限报 `Too high value of damping_ratio=…` |

段与 `SET_INPUT_SHAPER` 已落地（wave-2，`hybrid_corexy_dual_carriage.test` 即其验收）；系数表与上游 `shaper_defs.rs` 逐位对齐（`pseudo_inverse` 已在 `mathutil`）。**整形已真的作用于运动（2026-10-21）**：`motion/shaper.rs` 按 `kin_shaper.c` 实现整形位置函数，脉冲装到主 trapq 的每个 stepper，生成窗口取活跃轴脉冲极值、`kin_flush_delay` 随之变宽；无脉冲轴零开销。装/换整形前**已 `flush_step_generation`**；滑车切换/恢复时 `idex_modes` 发 `dual_carriage:update_kinematics`，整形层收到后 flush、按当前活跃轴重装脉冲并重扫窗口（对齐上游 `input_shaper.py:153-168`）。**仍欠**：`resonance_tester` 的 `disable_shaping`/`enable_shaping` 未接（其运动路径未落地）。`connect` 期若存在 `[dual_carriage]` 且任一整形器启用，报上游同文 config_error。

### `[adxl345]` / `[adxl345 <name>]` — 加速度计（wave-2）

| 选项 | 默认 | 说明 |
|------|------|------|
| `cs_pin` | —（SPI 组，必填） | 片选 |
| `axes_map` | `x,y,z` | 轴映射（可负，如 `-x,-y,z`） |
| `rate` | `3200` | 采样率；非法值报 `Invalid rate parameter: %d` |

段/命令层/加速度计接口（`start_internal_client`）与 mux 端点 `adxl345/dump_adxl345`（key `sensor`）已落地。**样本通路已通（2026-10-21）**：共享 `FixedFreqReader` 已泛化成按字段表解码（每字段带宽度与符号，`SamplePayload` 在构造期拒绝格式/载荷不匹配），ADXL345 的 5 字节 `"BBBBB"` + `query_adxl345_status`、MPU9250 的 6 字节 `">hhh"` + `query_mpu9250_status` 各自接上寄存器序列与 `_convert_samples` 缩放（含 `axes_map`）；ldc1612/hx71x/ads1220/ads131m0x 行为逐字节不变（默认类型参数 `P = u32`）。

### `[mpu9250]` / `[mpu9250 <name>]` — 加速度计（wave-2）

| 选项 | 默认 | 说明 |
|------|------|------|
| `i2c_address` | `0x68` | I2C 地址 |
| `i2c_speed` | `400000` | I2C 速率 |
| `rate` | `4000` | 采样率 |
| `axes_map` | `x,y,z` | 轴映射 |

段/命令层/接口与 mux 端点 `mpu9250/dump_mpu9250`（key `sensor`）已落地（带名节按 identifier 注册）。**gap**：同 ADXL345 的 bulk 数据通路（需 `">hhh"` 与 `query_mpu9250_status`），`ACCELEROMETER_*` 命令随之延后。

### `[hall_filament_width_sensor]` / `[tsl1401cl_filament_width_sensor]` — 线宽传感器（2026-10-21）

| 选项 | 默认 | 说明 |
|------|------|------|
| `adc_pin` / `adc1_pin`、`adc2_pin` | —（hall 必填其一） | 采样脚；hall 支持双通道（`RawHallFilamentWidthSensor`） |
| `Cal_dia1` / `Cal_dia2`、`Raw_dia1` / `Raw_dia2` | `1.50` / `2.00`、`100` / `200` | 标定点（两点线性换算） |
| `nominal_filament_dia` | `1.75` | 名义线宽（无读数时用它） |
| `measurement_interval` / `measurement_delay` | `10` / `100` | 测量间隔与延迟（mm） |
| `enable` | `False` 等价 | 段里有 `switch_pin`/`enable` 时按上游启用；`M405`/`M406` 运行期开关 |
| `runout_dia_min` / `runout_dia_max` | `0.1` / `5.0` | 超出区间按 runout 处理 |

两个模块按上游逐行实现：hall 的 `round(read*10000)` → 两点线性 `diameter_new` → `(5*d+new)/6` 平滑 →
`update_filament_array` 按 `interval`/`delay` 入队（`Filament width:%.3f` 日志），三个 pending 旗触发 `M221 S100`，
宽度出窗与断料走 `RunoutHelper`；tsl1401cl 用线性 CCD 读数换算 + `_process_measurement`。命令面：
`M404`（设定名义线宽/日志）、`M405`/`M406`（开/关）、`M407`（报告当前直径）、`QUERY_FILAMENT_WIDTH`。

### `[buttons]` — 按钮/编码器注册表（wave-2，2026-10-21 补齐固件查询）

`[buttons]` 无配置节，是被 `[filament_switch_sensor]`/`[filament_motion_sensor]`/`[gcode_button]` 用 `load_object` 拉起的注册表（`register_debounce_button` / `register_buttons`）。**固件查询 2026-10-21 落地**：`config_buttons`/`buttons_add`/`buttons_query`/`buttons_ack` 四条命令 + `buttons_state` 应答，每 **8 个引脚**一个固件对象（`src/buttons.c:76-78`），查询周期 `QUERY_TIME=2 ms`、`RETRANSMIT_COUNT=50`；`pull_up` 由引脚上的 `~` 决定（线上为 `0xff`）、`invert` 取反电平，宿主侧去抖按 `debounce_delay`。回调在 reactor 上触发（同上游 `register_async_callback`），所以真板上的引脚变化会真的驱动 filament/`gcode_button` 回调。**余项**：`analog_range`（模拟按键）仍缺 `query_adc` 模块，`[gcode_button]` 照旧读后拒绝；`register_buttons` 的失败目前只 `error!` 记录、不是上游的配置错。

### `[pause_resume]` — 暂停/恢复（批 #15）

| 选项 | 默认 | 说明 |
|------|------|------|
| `recover_velocity` | `50.0` | `RESUME` 未给 `VELOCITY=` 时的恢复速度（mm/s） |

四条命令 `PAUSE` / `RESUME`（`VELOCITY=`）/ `CLEAR_PAUSE` / `CANCEL_PRINT` 与 `get_status` 的 `is_paused` 已落地（help 与状态文案逐字照上游）。**SD 分支已接线**（2026-10-03，`6cf2cd5`）：回放中 `PAUSE` / `RESUME` / `CANCEL_PRINT` 真驱动 `virtual_sdcard`（这三支**不发** `action:*` 行，同上游；`RESUME` 撞上 `SD busy` 从命令冒出、`is_paused` 保留），空闲时仍走 `respond_info` 发 `action:paused` / `action:resumed` / `action:cancel`；`pause_resume/{pause,resume,cancel}` 三个 webhooks 端点已注册（`922eadd`）。

### `[filament_switch_sensor <name>]` / `[filament_motion_sensor <name>]` — 断料检测（wave-2）

| 选项 | 默认 | 说明 |
|------|------|------|
| `switch_pin` | —（必填） | 检测引脚 |
| `pause_on_runout` | `True` | 断料时暂停（需 `pause_resume` 对象） |
| `runout_gcode` / `insert_gcode` | 空 | 断料/回插时执行的 g-code |
| `pause_delay` | `0.5`（>0） | 暂停延时 |
| `event_delay` | `3.0`（≥0） | 事件确认延时 |
| `extruder` / `detection_length` | `extruder` / `7.0`（>0） | 仅 `[filament_motion_sensor]` |

两条 mux 命令（key `SENSOR`）：`QUERY_FILAMENT_SENSOR` / `SET_FILAMENT_SENSOR`。段与命令均对上游（`extruders.test` 为其验收）。

### `[heater_fan <name>]` — 加热器联动风扇（批 #6）

| 选项 | 默认 | 说明 |
|------|------|------|
| `heater` | `extruder`（列表） | 触发风扇的加热器名 |
| `heater_temp` | `50.0` | 温度阈值 |
| `fan_speed` | `1.0`（0..=1） | 触发时的风扇速度 |

`pin`/`kick_start_time`/`max_power`/`shutdown_speed`/`off_below`/`cycle_time`/`hardware_pwm` 由 `[fan]` 核心读取。语义：每秒 tick；**任一 heater 有 target 或温度 > `heater_temp` 即为 `fan_speed`**；仅速度变化时写 PWM；掉线关机速度默认 **1.0**（与 `[controller_fan]` 的 0.0 不同）。无 G-Code 命令，`get_status` 转发 fan 状态。
`heater` 名经加热器注册表的 `lookup_heater` 解析（`heaters.rs:501-506`，批 #20），未知名报上游逐字文案 `Unknown heater '<name>'`（`heater_fan.rs:119-120`）；语料中 2 个 run 的 `tachometer_pin`（`generic-prusa-buddy`、`printer-prusa-mini-plus-2020`）批 #8 已接通，实测两条 `run_case OK`。

### `[safe_z_home]` — 归零前安全 Z 抬升（批 #6）

| 选项 | 默认 | 说明 |
|------|------|------|
| `home_xy_position` | —（必填，两个数） | Z 归零前先移动到的 XY |
| `z_hop` | `0.0` | 抬升高度（0 表示整段跳过） |
| `z_hop_speed` | `15.0`（>0） | 抬升速度 |
| `speed` | `50.0`（>0） | 移动速度 |
| `move_to_previous` | `false` | 归零后回到原 XY |

Z 端停来源：`[stepper_z]`，否则 `[carriage <名>]` 中 axis 为 z 者；取不到报 `Missing Z endstop config for safe_z_homing`。**接管 G28**：按需先 `X0 Y0` → 安全位 → `Z0`；`G28 Z` 而 X/Y 未归零报 `Must home X and Y axes first`。与 `[homing_override]` **互斥**（`homing_override and safe_z_homing cannot be used simultaneously`）。装载必须为 `order = 70, phase = late`（晚于 toolhead 的 G28 注册）。

### `[manual_stepper <name>]` — 独立步进电机（批 #6）

| 选项 | 默认 | 说明 |
|------|------|------|
| `step_pin`/`dir_pin`/`enable_pin`/`microsteps`/`rotation_distance` | — | 步进电机核心选项 |
| `endstop_pin` | 无 | 有则可做端停运动 |
| `velocity` | `5.0`（>0） | 默认速度 |
| `accel` | `0.0`（≥0） | 默认加速度 |
| `position_min`/`position_max` | 无 | 可选软限位 |

命令见 G-Code 参考的 `MANUAL_STEPPER`。现状：**`MOVE=` 与 `G1 A<..>` 都会真正出步**——这台电机有自己的 trapq（首次需要时 `take_stepper()` 装进运动队列），`process_move` 按上游 append（`start_axis/end_axis` + `axes_r[ea_index]`），`SET_POSITION` 先 flush 再把 solver 摆到 `commanded_pos`。gap（如实登记）：**自带回零仍缺**——有 `endstop_pin` 时 `STOP_ON_ENDSTOP` 报 `Manual stepper homing is not implemented in this host`（本仓没有 `drip_update_time`，且 `drip_move` 只灌主 trapq、不带额外轴）。

### `[display]` — 液晶与编码器（st7920/hd44780/uc1701/ssd1306 已落地，批 #7+#13）

| 选项 | 默认 | 说明 |
|------|------|------|
| `lcd_type` | —（必填） | 已实现 `st7920`/`hd44780`/`hd44780_spi`/`uc1701`/`ssd1306`/`aip31068_spi`；其余（`sh1106`/`emulated_st7920`/…）报 `lcd_type '<x>' is not implemented in this host` |
| `cs_pin` / `sclk_pin` / `sid_pin` | — | st7920 三线（同 MCU，否则 `st7920 all pins must be on same mcu`） |
| `rs_pin`/`e_pin`/`d4_pin`…`d7_pin` | — | hd44780 4-bit 并行六线（同 MCU，否则 `hd44780 all pins must be on same mcu`）；`hd44780_protocol_init`（默认 True）、`line_length`（16\|20，默认 20，非法值 choice 文案） |
| `uc1701`/`ssd1306` 面板选项 | — | `a0_pin`(uc1701 必填)/`dc_pin`、`contrast`/`vcomh`/`invert`(ssd1306)、`rst_pin`/`reset_pin`；SPI 走 `McuSpi::send` 只入队（真传输在假 MCU 不可用，已知约束） |
| `hd44780_spi` 面板选项 | — | `latch_pin`（必填）、`line_length`（16\|20，默认 20）、`hd44780_protocol_init`（默认 True）、`spi_*`；每位按高/低半字节、每半字节三帧（`data`/`data\|E`/`data`，RS=bit1、E=bit3，批 #30） |
| `aip31068_spi` 面板选项 | — | `latch_pin`（必填，SPI cs/latch）、`line_length`（16\|20，默认 20）、`spi_speed`（≥100000，默认 100 kHz）、`spi_bus`/`spi_software_*`；传输是 9-bit 字（`encode`/`encoded_groups`，批 #28） |
| `display_group` | `_default_16x4` | 组来自随模块发布的 `display.cfg`（20 列时默认 `_default_20x4`）；未知组报 `Unknown display_data group '%s'` |
| 菜单/按键选项 | — | `menu_root`/`menu_timeout`/`menu_reverse_navigation`/`encoder_pins`/`encoder_steps_per_detent`/`encoder_fast_rate`/`click_pin`/`back_pin`/`up_pin`/`down_pin`/`kill_pin`/`analog_range_*`/`analog_pullup_resistor` —— 2026-10-21 起**菜单引擎已消费**（`display/menu.rs`） |

现状（2026-10-21 重核）：**渲染通路已落地**——`screen_update_event` 先问菜单、再按 `display_data` 组的 layout 渲染每项模板（`render(name,**kw)` 与 `draw_progress_bar(row,col,width,value)` 作为宿主可调用注入）→ `draw_text`（`~glyph~` 走 `write_glyph`）→ `flush()`，失败项只记日志并保留已画帧；**菜单引擎也已落地**（`menu` 对象、四个 `menu:*` 事件、嵌入 `menu.cfg` 在内置默认之上被 `printer.cfg` 同名覆盖，只在主 `[display]` 挂载；按键选项已消费）。**六个驱动都会画（2026-10-21）**：`hd44780`/`hd44780_spi`/`aip31068_spi` 走字符发生器（CGRAM 图标），`st7920`/`uc1701`/`ssd1306` 用上游 `font8x14.VGA_FONT`（逐字节 vendor，带漂移守卫测试）；菜单行在菜单占用画面时逐行写入（选中/编辑/容器/滚动标记照上游）；`[display_data]`/`[display_glyph]` 有主配置 factory（同名 section 覆盖出货项，组大小不变）。随模块发布的 `display.cfg`/`menu.cfg` 为 **vendored 副本**（各带漂移守卫测试）。

### `[fan_generic <name>]` — 通用风扇（批 #18）

本身不读选项：整节交给 `[fan]` 的核心（`pin`/`max_power`/`kick_start_time`/`off_below`/`cycle_time`/`hardware_pwm`/`shutdown_speed`/`enable_pin`/`tachometer_*`；**`shutdown_speed` 默认 `0.0`**，与 `[heater_fan]` 的 1.0 不同）；节名即 mux 值，注册 `SET_FAN_SPEED FAN=<name> SPEED=<0..1>`。`TEMPLATE=<display_template 名> [PARAM_*=…]` **已支持**：与 `[output_pin]` 共用同一个 `template_evaluator`（0.5 s 求值一次，结果直接 `Fan::set_speed`）；`SPEED=` 与 `TEMPLATE=` 必须给且只给一个。

### `[idle_timeout]` — 空闲超时（批 #21）

**这个节可以不写**（2026-10-18）：上游在 `toolhead.py:604-614` 无条件加载它，本仓 `load.rs` 的 `DEFAULT_MODULES` 照做——没有该节时对象仍存在、`timeout` 取默认 600、`SET_IDLE_TIMEOUT` 仍可用，`printer.configfile.settings.idle_timeout.timeout` 也有值（客户端宏如 Mainsail 的 `RESUME` 会无条件读 `printer.idle_timeout.state`）。

| 选项 | 默认 | 说明 |
|------|------|------|
| `timeout` | `600` | 空闲秒数，`above 0` |
| `gcode` | 内置（`{% if 'heaters' in printer %}` → `TURN_OFF_HEATERS`；`M84`） | 超时后跑的脚本 |

`SET_IDLE_TIMEOUT [TIMEOUT=]` 可改超时；`get_status` 给 `state`（`Idle`/`Ready`/`Printing`）、`printing_time`、`idle_timeout`。起印识别以 `toolhead:sync_print_time` 事件为准（`calc_print_time` 抬高 print time 时发出）；事件未到达时仍回退到“观察 `print_time` 前进”，且收到过一次事件后不再双发。默认脚本的两行现在都真生效（`TURN_OFF_HEATERS` 与 `M84`）；上游的 `update_timer` 用本仓的 `arm`（注销后重挂）顶替。

### `[verify_heater <heater_name>]` — 加热器升温校验（批 #20）

| 选项 | 默认 | 说明 |
|------|------|------|
| `hysteresis` | `5` | 目标附近容差（`minval=0.`） |
| `max_error` | `120` | 累积误差上限（`minval=0.`） |
| `heating_gain` | `2` | 期望升温（`above 0.`） |
| `check_gain_time` | `60`（`heater_bed`）/ `20`（其他） | 升温窗口秒数（`minval=1.`） |

每个 heater 自动获得一个检查器（配置无此节时用默认值）；失败 `invoke_shutdown("Heater <name> not heating at expected rate" + 提示)`。它读到的是 `Heater::get_temp` 的两个保护后的值：目标为 0 输出 0；**读数超 7 s 未更新时报 `0`**（传感器静默当冷热床，上游 `QUELL_STALE_TIME`）；另有一个与它无关的宿主侧保护——主线程超 5 s 未证明自己活着时 PWM 直接归 0（`MAX_MAINTHREAD_TIME`）。已知差异：`[verify_heater <不存在的 heater>]` 报装载器的 `Section '…' is not a valid config section`，而非上游的 `Unknown heater`。

### `[dac084S085 <name>]` — 四通道 SPI DAC（批 #23）

| 选项 | 默认 | 说明 |
|------|------|------|
| `enable_pin` | —（必填） | SPI 片选（mode 1、默认速率 10 MHz） |
| `scale` | `1.0` | 满量程参考，`above=0.` |
| `channel_A` … `channel_D` | — | 各通道目标值（`minval=0.`、`maxval=scale`）；写值 `int(val*255/scale)`（**截断**，与 `[ad5206]` 的 `+0.5` 不同），未给的通道不写 |

写入在 bring-up 时经 MCU post-init 回调发出。⚠️ 节名**大小写敏感**：配置里写 `[dac084S085 stepper_digipot]`，`section!` 的 id 必须逐字 `dac084S085`（写成小写会报 `Section 'dac084s085 …' is not a valid config section`）。

### `[adc_scaled <name>]` — 参考电压缩放 ADC（批 #19）

| 选项 | 默认 | 说明 |
|------|------|------|
| `vref_pin` / `vssa_pin` | —（必填） | 两路参考 ADC（同 MCU，否则 `vref and vssa must be on same mcu`） |
| `smooth_time` | `2.0` | 参考平滑时间常数（秒），`above=0.` |

节名注册为虚拟 pin chip，消费者写作 `sensor_pin: <name>:PA0`，读数按 `(raw - vssa)/(vref - vssa)` 归一。**必须写在 `[extruder]`/`[heater_bed]` 之前**（本仓以 `phase = early` 保证）。gap：`QUERY_ADC` 未实现。

### `[multi_pin <name>]` — 多 pin 扇出（批 #26）

| 选项 | 默认 | 说明 |
|------|------|------|
| `pins` | —（必填） | 逗号分隔的真实 pin 列表；调用（`set_pwm`/`update_pwm`/`setup_*`/数字输出）逐个转发 |

节名注册为虚拟 pin chip：其他节写 `pin: multi_pin:<name>`（或 `heater_pin: multi_pin:heater`）。装载相位为 **`phase = early`**（`[extruder]` 等主节在装载期就要解析它；prefix-only 降 `order` 无效）。别名不得用于步进电机 pin。

### `[axis_twist_compensation]` — 轴扭曲 Z 补偿（批 #36）

| 选项 | 默认 | 说明 |
|------|------|------|
| `horizontal_move_z` / `speed` | 照上游 | 校准与补偿时的抬升高度与速度 |
| `calibrate_start_x` / `calibrate_end_x` / `calibrate_y` / `calibrate_start_y` / `calibrate_end_y` | 照上游 | 校准扫描范围 |
| `z_compensations` / `zy_compensations` | 照上游 | 各采样点的 Z 补偿值（长度必须与采样点数一致） |

在探针测量完成后经 `probe:update_results` **原地修正上报的 Z**（该事件现在带 `ProbeResultsHandle` 载荷），并注册 `AXIS_TWIST_COMPENSATION_CALIBRATE`。

### `[gcode_button <name>]` — 按钮触发宏（批 #37）

| 选项 | 默认 | 说明 |
|------|------|------|
| `pin` | —（必填） | 按钮引脚 | 
| `press_gcode` / `release_gcode` | — / 空 | 按下/释放时渲染并执行的宏体 |
| `debounce_delay` | `0` | 去抖（秒，`minval=0`） |
| `analog_range` | — | **本仓不支持**：读取后明确报错（上游走 `query_adc`，本仓无该基建） |

`QUERY_BUTTON BUTTON=<name>` 查询状态；`get_status` 报 `PRESSED`/`RELEASED`。gap：**固件侧按钮查询未接**（回调已登记，假固件不会触发）。

### `[homing_heaters]` — 归零期间关加热器（批 #31）

| 选项 | 默认 | 说明 |
|------|------|------|
| `heaters` | 全部已注册 heater | 逗号分隔的 heater 名列表；未知名报 `One or more of these heaters are unknown: ['x']` |
| `steppers` | — | 逗号分隔的 stepper 名列表（**资格过滤未实现**：本仓 homing 事件无载荷，所有归零都被视为符合条件） |

### `[firmware_retraction]` — 固件回抽（批 #32）

| 选项 | 默认 | 说明 |
|------|------|------|
| `retract_length` | `0` | 回抽长度（mm，`minval=0`） |
| `retract_speed` | `20` | 回抽速度（mm/s，`minval=1`） |
| `unretract_extra_length` | `0` | 恢复时的额外长度 |
| `unretract_speed` | `10` | 恢复速度（mm/s，`minval=1`） |

`G10`/`G11` 经 `gcode_move` 的 `SAVE/RESTORE_GCODE_STATE` + `G1 E…` 实现；`SET_RETRACTION` 可改四项（`GET_RETRACTION` 回读，`%.5f`）。

### `[respond]` — 主机回显（批 #29）

| 选项 | 默认 | 说明 |
|------|------|------|
| `default_type` | `echo` | `M118`/`RESPOND` 的默认前缀：`echo`→`echo:`、`command`→`//`、`error`→`!!`（choice，大小写敏感） |
| `default_prefix` | 取自 `default_type` | 覆盖默认前缀 |

注册就绪前可用的两条命令：`M118 <原文>`（前缀 + 原文）与 `RESPOND [TYPE=] [PREFIX=] [MSG=]`（`TYPE` 多了 `echo_no_space`：前缀同为 `echo:` 但**不加空格**）。

### `[mcp4018 <name>]` — 单路 I2C 数字电位器（批 #33）

| 选项 | 默认 | 说明 |
|------|------|------|
| `i2c_address` | **`0x2f`** | I2C 地址（与 `[mcp4451]` 的必填不同） |
| `i2c_mcu` / `i2c_bus` / `i2c_speed` / `i2c_software_*_pin` | `mcu` / — / `100000` / — | 总线选项（同 `[i2c_device]`） |
| `scale` | `1.0` | 满量程参考，`above=0.` |
| `wiper` | —（必填） | 上电初值（`minval=0.`、`maxval=scale`），写 `int(v*127/scale+.5)` 单字节 |

`klippy:connect` 时写入 `wiper`；运行时用 `SET_DIGIPOT DIGIPOT=<name> [WIPER=]` 改值。

### `[mcp4451 <name>]` — I2C 数字电位器（批 #25）

| 选项 | 默认 | 说明 |
|------|------|------|
| `i2c_address` | —（必填） | I2C 地址，**仅 44..47**（其它值报 `mcp4451 address must be between 44 and 47`） |
| `i2c_mcu` / `i2c_bus` / `i2c_speed` | `mcu` / — / `100000` | 总线选项（与 `[i2c_device]` 同口径；也可用软件 SCL/SDA pin） |
| `scale` | `1.0` | 满量程参考，`above=0.` |
| `wiper_0` … `wiper_3` | — | 各路目标值（`minval=0.`、`maxval=scale`）；写值 `int(val*255/scale+.5)`，未给的不写 |

装载时先无条件写 `[0x40,0xff]`、`[0xa0,0xff]`，再按寄存器 `[0x00,0x01,0x06,0x07]` 逐路写；字节为 `[(reg<<4)|((value>>8)&3), value]`。写入在 bring-up 时经 post-init 回调发出。

### `[ad5206 <name>]` — 数字电位器（批 #16）

| 选项 | 默认 | 说明 |
|------|------|------|
| `enable_pin` | —（必填） | SPI 片选（mode 0、默认速率 25 MHz） |
| `scale` | `1.0` | 满量程参考，`above=0.` |
| `channel_1` … `channel_6` | — | 各通道目标值（`minval=0.`、`maxval=scale`）；给出即换算 `int(val*256/scale+.5)` 写入寄存器 `n-1`，未给的通道不写 |

写入在 bring-up 时经 MCU post-init 回调发出（装载期 SPI oid 尚未建立）。语料里写作 `[ad5206 stepper_digipot]`。

### `[load_cell]` — 称重传感器（hx711/hx717/ads1220/ads131m02/ads131m04，批 #14）

| 选项 | 默认 | 说明 |
|------|------|------|
| `sensor_type` | —（必填） | `hx711`/`hx717`（按 `dout_pin`/`sclk_pin` 接线）或 `ads1220`/`ads131m02`/`ads131m04`（SPI 芯片，选项按上游同名：`spi_mcu`/`cs_pin`/`data_ready_pin` 等）；其余值报 `sensor_type '<x>' is not implemented in this host` |
| `dout_pin` / `sclk_pin` | — | 数据与时钟（必须同 MCU，否则 `… config error: All pins must be connected to the same MCU`） |
| `sample_rate` / `gain` | 按芯片 | HX711：80/10、A-128/B-32/A-64（默认 80、A-128）；HX717：320/80/20/10、A-128/B-64/A-64/B-8（默认 320、A-128） |
| `reference_tare_counts` / `counts_per_gram` / `sensor_orientation` | — / — / `normal` | 校准三件套（`counts_per_gram` minval 1.0，非法值 `must have minimum of 1.0`） |

端点 `load_cell/dump_force`（mux key `load_cell`，四列 `time, force (g), counts, tare_counts`）。

### `[load_cell_probe]` — 力探针（2026-10-06）

传感与校准选项与 `[load_cell]` **同一节、同一份读取**（装载期用本节建一次 load cell），所以 `sensor_type`、`dout_pin`/`sclk_pin`、`sample_rate`/`gain`、`reference_tare_counts`/`counts_per_gram`/`sensor_orientation` 都写在本节；另加探测参数：

| 选项 | 默认 | 说明 |
|------|------|------|
| `tare_time` | `4/60` s | 归零时间，限 `[0.01, 1.0]` |
| `trigger_force` | `75` g | 触发力，限 `[10, 250]` |
| `force_safety_limit` | `2000` g | 安全上限，限 `[100, 10000]` |
| `speed` / `lift_speed` | `5.0` / = `speed` | 探测与抬升速率（均须 `> 0`） |
| `samples` / `sample_retract_dist` / `samples_result` / `samples_tolerance` / `samples_tolerance_retries` | `1` / `2.0` / `average` / `0.100` / `0` | 与 `[probe]` 同名的取样参数（`samples_result` 取 `average`/`median`） |
| `x_offset` / `y_offset` / `z_offset` | `0.` / `0.` / —（必填） | 探针相对喷嘴的偏移；`z_offset` 同时是 `probe:z_virtual_endstop` 的 `position_endstop` |
| `drift_filter_cutoff_frequency` / `drift_filter_delay` | 关闭 / `2` | 连续归零的漂移滤波（截止频率 `[0.1, 20]`，延迟 `[1, 2]`） |
| `buzz_filter_cutoff_frequency` / `buzz_filter_delay` | 关闭 / `2` | 蜂鸣滤波（截止频率须大于 `80` 且低于采样率上限；延迟 `[1, 2]`） |
| `notch_filter_frequencies` / `notch_filter_quality` | 空 / `2.0` | 陷波滤波（频率表；品质因数 `[0.5, 6.0]`） |

装载行为（2026-10-06 接线，对齐上游）：

- 注册 `probe` 对象（`PROBE`/`PROBE_ACCURACY` 等命令族）与 `probe` pin chip：`endstop_pin: probe:z_virtual_endstop` 可解析，轨道取本节的 `z_offset` 作 `position_endstop`；**同一配置里再有一个 `probe` chip（`[probe]`/`[probe_eddy_current]`）是配置错误**（不静默覆盖）。
- 在 `klippy:mcu_identify`（所有节装载完、任何连接与配置构建之前）把 Z 轨电机注册进本探针 trigger analog 的 trigger dispatch，触发时才会为该电机发 `stepper_stop_on_trigger`（上游 `probe.LookupZSteppers`）。
- 同一时刻把本节的传感器采样以 **init** 命令交给 `[trigger_analog]`（上游 `trigger_analog.py` 的 `setup_trigger_analog`），固件才会持续上报采样活动。

### `[tmc2208 <stepper>]` / `[tmc2209 <stepper>]` — TMC 步进驱动（UART，批 #7）

| 选项 | 默认 | 说明 |
|------|------|------|
| `uart_pin` | —（必填） | UART 单线（`tx_pin` 缺省复用） |
| `select_pins` / `uart_address` | — | 模拟多路复用（同 mcu、同 pin、`(id,addr)` 唯一） |
| `run_current` / `sense_resistor` / `stealthchop_threshold` | — / 0.110 / — | 电流与静音阈值 |
| `interpolate` / `driver_SGTHRS` / `diag_pin` | True / — / — | 仅 tmc2209 |

命令：`INIT_TMC` / `DUMP_TMC` / `SET_TMC_FIELD` / `SET_TMC_CURRENT`（mux 键 `STEPPER`）；`tmc2209_<stepper>:virtual_endstop` 可用作端停。**活线已接通（2026-10-18）**：真板上读写走 `tmcuart_send`/`tmcuart_response`（5 次重试 + IFCNT 回读校验，共享 UART 有总线锁，`select_pins` 会真拉脚），`DUMP_TMC` 读的是芯片真值、`SET_TMC_CURRENT`/`INIT_TMC` 真的生效；`--lib tmc` 102 测覆盖编解码与两条重试环路。fileoutput（语料）下仍短路：读 0、写丢弃、不发帧。

**SPI 族（`[tmc2130]` / `[tmc2240]` / `[tmc2660]` / `[tmc5160]`）也已落地**（2026-10-06 复核：四个
`section!` 均在工厂表，命令与 UART 族同）；本页尚未为它们补逐项选项表——选项集与上游同名节一致，
逐项说明见[模块表](../developer-manual/README.md)与上游 `config/` 参考。

### `[save_variables]` — 变量持久化（2026-10-04）

| 选项 | 默认 | 说明 |
|------|------|------|
| `filename` | —（必填） | 变量文件路径；只展开 `~` 与 `~/`（`~user` 原样保留） |

文件是 `[Variables]` 段的 configparser 格式，值按 Python literal 写。缺文件时自动建空；
已有文件解析失败报 `Unable to parse existing variable file`。命令见
[`SAVE_VARIABLE`](gcode-commands.md)；变量经 `printer.save_variables.variables.<name>` 供宏读取
（`get_status` 形状即 `{variables: {...}}`）。支持的字面量子集：`None` / `True` / `False` /
整数 / 浮点 / 字符串 / 列表 / 字典（字符串键）；不支持 tuple / set / bytes / complex 与
非字符串 dict 键。

### `[force_move]` — 诊断用力移动（2026-10-04）

| 选项 | 默认 | 说明 |
|------|------|------|
| `enable_force_move` | `False` | 为 `True` 才注册 `FORCE_MOVE`；`STEPPER_BUZZ` 始终注册 |

命令见 [`FORCE_MOVE` / `STEPPER_BUZZ`](gcode-commands.md)。该段会在首个电机装载时被创建，
因此它总在 `[stepper_*]` 之前进工厂表。

