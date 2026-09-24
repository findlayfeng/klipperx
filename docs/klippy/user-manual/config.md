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

### 空节（Empty Section）

节可以没有参数，仅用于声明存在。

```ini
[pause_resume]

[exclude_object]
```

## 选项校验

装载配置时，**每个 section 与每个选项都必须被某个模块读到**：

- 没有任何对象对应的 section（且没有模块读它）→ `Section 'xxx' is not a valid config section`；
- 某个 section 里没人读的选项（通常是拼写错误）→ `Option 'xxx' is not valid in section 'yyy'`。

校验**不**在装载时“提前放行”任何选项，所以一个拼错的键不会被默默忽略——这正是它存在的意义。
有默认值的选项（如 `[output_pin]` 的 `value`）算作已读，不需要写出来。

配置出错时打印机进入 `error` 状态（`info` 的 `state` 为 `error`，不是 `shutdown`）：
修正文件后用 `RESTART` 重载即可，不必重启进程。这与 G-code 参数错误（`!!` 回复、不停机）
和真正的内部错误（停机，`state` 为 `shutdown`）是三条不同的路。

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

以下为 KlipperX 当前已实现的配置节（共 18 个装载 id，按装载顺序；`[mcu]` 与 `[printer]`
分别在最早与最晚装载，其余按各节 `section!` 声明的 `order`）。

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

### `[printer]` — 运动与工具头（注册为 `toolhead`）

整机的运动参数与运动学选择。该节在**最后**装载（上游也是最后 load `toolhead`），装载时注册
`G4` / `M400` / `G28` / `SET_KINEMATIC_POSITION`，并拉起 `gcode_move`（坐标系）与
`query_endstops`。

| 参数 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `kinematics` | 字符串 | 是 | — | `none` / `cartesian` / `corexy` / `corexz` / `hybrid_corexy` / `hybrid_corexz`（delta 族与 `generic_cartesian` 待做） |
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
| `heater_pin` | 引脚 | 是 | — | 加热输出（按 PWM 接） |
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
| `pressure_advance` | 浮点 | 否 | `0.0` | 压力推进，`≥ 0`（运行期用 `SET_PRESSURE_ADVANCE` 调） |
| `pressure_advance_smooth_time` | 浮点 (s) | 否 | `0.040` | PA 平滑时间，`≤ 0.200` |
| `step_pin` / `dir_pin` / `rotation_distance` / `microsteps` … | 同 `[stepper_*]` | 否 | — | **写了任一个**才建 E 轴 stepper（上游同规则）；不写则纯加热 |

命令：`M104` / `M109`（当前不等温）/ `SET_PRESSURE_ADVANCE`（mux `EXTRUDER`）/
`ACTIVATE_EXTRUDER`。只有名字是 `extruder` 的主挤出机额外注册 `EXTRUDER` 默认项。

### `[heater_bed]` — 热床

选项 = 加热器共用选项（`gcode_id` 内定为 `B`），注册 `M140` / `M190`（当前不等温）。

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
| `tachometer_pin` | — | 拒收 | — | 需 `pulse_counter`（F9），写了直接报配置错而不是静默 `rpm: null` |

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
| MCU 内部 | `temperature_mcu`（可带 `sensor_mcu`，默认 `mcu`） |
| 组合 | `temperature_combined`（`sensor_list` 必需、`maximum_deviation` ≥ 0、`combination_method` ∈ `min`/`max`/`mean` 必需） |
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

### `[thermistor <name>]` / `[adc_temperature <name>]` — 自定义传感器定义

定义一个新的传感器类型供 `sensor_type` 引用；节的 `<name>` 就是注册的类型名。两者的差别
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
（**落盘仍需 `SAVE_CONFIG`，该回写见 TODO C2，本仓尚未实现**）。

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

