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

`[mcu]` 节的连接参数、固件重启方式、udev 规则等详细说明已移至独立文件，[参见 → MCU 连接方式](mcu-connection.md)。

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

## 已支持的配置节

以下为 KlipperX 当前已实现的配置节。

### `[mcu]` / `[mcu <name>]` — MCU 连接

定义一台 MCU 设备的连接方式。支持匿名节（`[mcu]`）和命名节（`[mcu <name>]`）。

| 参数 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `serial` | 字符串 | 二选一 | — | 串口设备路径（如 `/dev/ttyACM0`） |
| `baud` | 整数 | 否 | `250000` | 串口波特率（仅 `serial` 模式） |
| `canbus_uuid` | 12 位十六进制 | 二选一 | — | MCU 芯片唯一 ID（12 位 hex），CAN 模式必给 |
| `canbus_interface` | 字符串 | 否 | `can0` | CAN 网络接口名 |
| `canbus_nodeid` | 整数 (1..895) | 与 `canbus_uuid` 同用 | — | CAN 节点号，Klipper 由 `[canbus_ids]` 分配，klipperx 需显式给出 |
| `restart_method` | `command` / `arduino` / `cheetah` / `rpi_usb` | 否 | `arduino`（串口）/ `command`（非串口） | MCU 固件重启方式 |
| `usb_power` | `auto` / `sysfs` / `libusb` | 否 | `auto` | `rpi_usb` 的切电机制 |

> 详细参数说明见 [MCU 连接方式](mcu-connection.md)。

### `[output_pin <name>]` — 数字输出引脚

定义一个可通过 `SET_PIN` 命令控制的数字输出引脚。该节**必须**带名称（即 `[output_pin <name>]` 格式），名称即为 `SET_PIN PIN=<name>` 中引用的引脚名。

| 参数 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `pin` | 字符串 | 是 | — | 引脚描述（格式：`<mcu_name>:<pin>`），对应已配置的 MCU |
| `value` | 0.0 ~ 1.0 | 否 | `0` | 启动时引脚电平（`>= 0.5` 为高） |
| `shutdown_value` | 0.0 ~ 1.0 | 否 | `0` | 关机时引脚回退电平 |

**注意：**
- 本端口仅支持**数字输出**，不支持 PWM（`pwm` / `cycle_time` 选项尚未实现）
- 引脚值变化立即生效，不经过工具头调度

**示例：**

```ini
[output_pin my_fan]
pin: mcu:PA0
value: 0
shutdown_value: 0

[output_pin my_light]
pin: mcu:PB5
value: 0
```

对应 G-Code 命令：

```gcode
SET_PIN PIN=my_fan VALUE=1    ; 打开风扇
SET_PIN PIN=my_fan VALUE=0    ; 关闭风扇
SET_PIN PIN=my_light VALUE=1  ; 打开灯
```

---

- [← 用户手册首页](README.md)
