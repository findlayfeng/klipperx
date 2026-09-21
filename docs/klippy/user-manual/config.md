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
- 值变化采用“立即”路径（数字输出 `update_digital_out`、PWM `update_pwm`），**不**经过工具头调度；PWM 的立即变化会对齐到软件 PWM 的周期边界。上游那种随打印时间生效的 `SET_PIN` 要等运动/时钟层（TODO C1）。
- 软件 PWM 的 `shutdown_value` 只能是 `0.0` 或 `1.0`（固件只能把引脚固定在高或低）。
- `scale` / `static_value` / `TEMPLATE` 尚未实现。

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

---

- [← 用户手册首页](README.md)
