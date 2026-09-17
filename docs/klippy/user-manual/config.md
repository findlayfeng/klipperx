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
| `canserial_nodeid` | CAN 节点号（1..895） | 通过 CAN 上的 klipper "can serial" 链路连接；配 `canserial_interface`（默认 `can0`） |
| `host_library` | klipper host 库（`libklipper_host.so`）的路径 | 在主机进程内运行一份 klipper 固件（模拟与自测用） |
| `test` | 每行 `输入帧 输出帧...`（十六进制） | 仅测试构建可用，供单元测试脚本化一个假设备 |

```ini
[mcu]
serial: /dev/serial/by-path/platform-3f980000.usb-usb-0:1.3:1.0-port0
baud: 250000

[mcu canbed]
canserial_nodeid: 2
canserial_interface: can0

[mcu simulated]
host_library: /usr/local/lib/libklipper_host.so

[mcu fake]
test: 06 10 05 00 00 7e
```

节名取自节的 sub（`[mcu zboard]` → `zboard`），`restart_method` 也在这里设置。串口按 raw 模式打开：不回显、不做 CR/LF 转换、不启用协议没用到的流控。

CAN 这条走的是 klipper 的 **can serial** 链路（就像把串口协议搬到 CAN 上跑），不是 CAN 协议本身，所以参数名用 `canserial_*`：`canbus_*` 这样的名字留给将来真正实现 CAN 协议的接口。节点号到仲裁 ID 的映射沿用 klipper：`0x100 + 2 × nodeid`，主机写这个 ID、MCU 用下一个回。

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
