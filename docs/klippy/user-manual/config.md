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
| `kinematics` | 字符串 | 是 | — | `none` / `cartesian` / `corexy` / `corexz` / `hybrid_corexy` / `hybrid_corexz` / `polar` / `delta`（`rotary_delta`/`deltesian`/`winch`/`generic_cartesian` 待做） |
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
| `tachometer_pin` | — | 可选 | — | 批 #8 接通 `pulse_counter` 频率计数：报 `rpm`（无边沿时 `rpm: 0.0`，不再拒收、也不静默 `rpm: null`） |
| `tachometer_ppr` | 2 | ≥1 | — | 每转脉冲数（需 `tachometer_pin`；`<1` 拒收） |
| `tachometer_poll_interval` | 0.0015 | >0 | — | 测速轮询间隔秒（需 `tachometer_pin`；`≤0` 拒收） |

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

定义一个新的传感器类型供 `sensor_type` 引用；节的 `<name>` 就是注册的类型名。定义与引用在本仓**与文件中先后无关**（`[thermistor <name>]` 声明为 `phase = early`，见开发手册的相位通则）；若用 `[adc_temperature <name>]` 定制电压型，**请把定义写在引用它的 `[extruder]`/`[heater_bed]` 之前**（该 prefix 尚未设为 early，属待补项）。两者的差别
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

段已落地（2026-09-24 集成批 #1）；`G2`/`G3` 与 `G17-G19` 平面命令**未注册**（H10），未知命令经静默放行。

### [bed_screws]

| 选项 | 默认 | 说明 |
|------|------|------|
| `screw1` … `screw99` | — | 每项两浮点（X Y）；**数到第一个缺失编号即停**，少于 3 颗报 `bed_screws: Must have at least three screws` |
| `screwN_name` | `screw at %.3f,%.3f` | 螺丝显示名 |
| `screwN_fine_adjust` | — | 该螺丝精调坐标（记在粗调名下） |
| `speed` / `probe_speed` | `50.` / `5.` | 行进/探入速度（均 above 0） |
| `horizontal_move_z` / `probe_height` | `5.` / `0.` | 抬升/下探 |

段已落地；`BED_SCREWS_ADJUST`/`ACCEPT`/`ADJUSTED`/`ABORT` 命令族**未移植**（H9，语料经未知命令放行）。

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
| `rename_existing` | — | 被改名的命令（仅 load 期同型检查，连接期换名未做） |
| `variable_<名>` | — | 字面量，`get_status` 可见（只读） |

宏即命令（大写注册）；**宏体已渲染执行**（批 #4：受控子集引擎，渲染后经 gcode 派发）；`SET_GCODE_VARIABLE` 已注册（变量可写）；裸 `[gcode_macro]` 为共享模板持有者（零选项）。**`{% set %}` 已支持**（批 #9：顶层可见、`if` 块外泄、`for` 块不外泄，作用域对齐 Jinja2 3.1.6）；其余子集外构（`float`/`default(x)` 过滤器等）仍显式报错（缺口见 `extras/template.rs` 模块文档），完整 Jinja 保真仍属 H3。

### [led <name>] / [neopixel <name>] / [dotstar <name>] / [pca9533 <name>] / [pca9632 <name>] / [display_template <name>]

| 选项 | 适用 | 说明 |
|------|------|------|
| `initial_RED`/`initial_GREEN`/`initial_BLUE`/`initial_WHITE` | 各灯段 | 0..=1 初值（驱动无该色时忽略） |
| `cycle_time`/`hardware_pwm`/`red_pin`…`white_pin` | `led` | PWM/引脚组（`led.py:150`） |
| `pin`/`chain_count`/`color_order` | `neopixel` | 链长与色序（`neopixel.py:106`） |
| `data_pin`/`clock_pin`/`chain_count` | `dotstar` | 双线链（`dotstar.py:55`） |
| `i2c_*`（bus 选项组） | `pca9533`/`pca9632` | I2C 地址/速率等（`bus.py:302`）；`pca9632` 另有 `color_order` |
| `text`、`param_*` | `display_template` | 模板文本与参数（惰性对象，`is_queryable=false`） |

六段已落地（2026-09-24 批 #2）；`SET_LED`/`SET_LED_TEMPLATE` 未注册（H3/H8），模板 `text` 不渲染（随 U-A7b）。

### [extruder_stepper <name>]

| 选项 | 默认 | 说明 |
|------|------|------|
| 电机组 | — | 经 `PrinterStepper` 读取（步距等同上游） |
| `extruder` | `extruder` | 绑定的挤出机名（connect 期校验，上游文案 `'<名>' is not a valid extruder.`） |
| `pressure_advance` / `smooth_time` | `0.` / `0.040` | 本步进独立压力推进 |

段已落地（批 #2）；`SET_PRESSURE_ADVANCE [EXTRUDER=<段名>]` 按名挂值 ✓；宿主 step 同步待 toolhead 缝（H10），`SYNC_EXTRUDER_MOTION`/`SET_EXTRUDER_ROTATION_DISTANCE` 未注册。

### [exclude_object]

零选项段（上游 `exclude_object.py:16-46`）。装载时注册 `EXCLUDE_OBJECT_START`/`_END`/`EXCLUDE_OBJECT`/`EXCLUDE_OBJECT_DEFINE` 四命令并持有对象状态；排除区内移动被 `MoveTarget` 变换丢弃，**离区时按上游扣被丢弃 prime 段的 E**（`offset[3]`/`extruder_adj`，批 #4）。已随 U-A7b 宏体渲染转绿（2026-09-24）。

### [virtual_sdcard] / [display_status] / [homing_override] / [sdcard_loop]

| 节 | 选项 | 说明 |
|---|------|------|
| `virtual_sdcard` | `path`（必）、`on_error_gcode`（默认=上游 `DEFAULT_ERROR_GCODE`） | 文件回放命令族（M20-M27、`SDCARD_RESET_FILE` 等）未注册（H4） |
| `display_status` | 无 | `M73`/`M117`/`SET_DISPLAY_TEXT` 已注册（批 #7）；进度/消息由这两条命令驱动（无 `[display]` 时也按需创建） |
| `homing_override` | `axes`（默认 `XYZ`）、`set_position_x/y/z`（无）、`gcode`（必） | G28 包装未装——脚本体是宏模板、暂不渲染（H9 共担） |
| `sdcard_loop` | 无 | `SDCARD_LOOP_BEGIN/_END/_DESIST` 未注册；栈/索引语义已单测钉住 |

### [dual_carriage]

| 选项 | 说明 |
|------|------|
| `axis`（X/Y/Z）、`safe_distance`（毫米） | 第二滑架轴与最近距，按上游读取（`idex_modes.py`/`cartesian.py:24-34`） |
| `primary_carriage`（generic 形态，**可选**） | 无它 = 该轴的**主动滑架**（`axis` 必填、无 `safe_distance`）；有它 = 从动滑架（`axis` 可选、仅作交叉校验，`safe_distance` 生效）。同主滑架的两个 dual 会报重（按**主滑架名**判，非按轴）；`[extra_carriage <name>]` 的 `primary_carriage` **仍必填**（批 #27） |
| 电机组 | 经 `PrinterStepper`（同上游 `LookupMultiRail`） |

cartesian 在 late 阶段认领；注册 `dual_carriage` 对象与 `SET_DUAL_CARRIAGE`/`SAVE_…_STATE`/`RESTORE_…_STATE`。**轨间坐标交接已实现**（批 #4，`toggle_active_dc_rail` 语义：切换/恢复携带 gcode 坐标）；步进仍仅主轨（`updateLimits` 未移植，C1）。已随 U-A7b 转绿（2026-09-24）。**generic 形态**（批 #12）：`[carriage <name>]`/`[dual_carriage <name>]`/`[extra_carriage <name>]`/`[stepper <name>]` 由运动学注册同一对象与三命令（`dual_carriage` status 报 `active_carriage` + `carriages`），`[printer]` 的 `max_z_velocity/max_z_accel` 经 `carriage::build` 进入 generic 归零（原写死 0 为首因）。

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
| `calibrate` | 静态标定开关（`enable`/`enable_with_touch`；**实现未接——调用显式报错**，见模块残差注记） |
| 采样组（`samples`/`sample_retract_dist`/`samples_tolerance`/`lift_speed`…） | 由 probe/`ProbePointsHelper` 读取 |

段与对象已落地（2026-09-24 批 #3，`eddy.test` 转绿）；**同名 section 按上游 `strict=False` 后写覆盖合并**（本仓自 2026-09-24 起，eddy.cfg 双段即其用例）。静态标定与 `PROBE_EDDY_CURRENT_CALIBRATE`/`Z_OFFSET_APPLY_PROBE` 未实现（模块残差注记）。

> 配置解析补充（批 #5）：除同名 section 合并外，`#*# SAVE_CONFIG` 自动保存块按上游 `_find_autosave_data` **读取合并**（header 逐字节识别、`#*# ` 前缀剥离、正文优先/块只补新、损坏行告警）——语料 `delta_calibrate.cfg` 的双段与高度数据即其用例（回写侧 `SAVE_CONFIG` 命令未做）。

### `[input_shaper]` — 输入整形参数（wave-2）

| 选项 | 默认 | 说明 |
|------|------|------|
| `shaper_type` / `shaper_type_<轴>` | `mzv` | 整形器类型（`zv`/`zvd`/`mzv`/`ei`/`2hump_ei`/`3hump_ei`，可带 `(n,t)`、`(v_tol=)` 括号参数） |
| `shaper_freq_<轴>` | 0（禁用） | 整形频率 Hz |
| `damping_ratio_<轴>` | `0.1` | 阻尼比；超上限报 `Too high value of damping_ratio=…` |

段与 `SET_INPUT_SHAPER` 已落地（wave-2，`hybrid_corexy_dual_carriage.test` 即其验收）；系数表与上游 `shaper_defs.py` 逐位对齐（`pseudo_inverse` 已在 `mathutil`）。**gap：系数未接到步进生成**（无 chelper `input_shaper_alloc`/`set_sk`/`set_shaper_params` 层，`recompute_scan_windows()` 为显式记账 no-op）；`connect` 期若存在 `[dual_carriage]` 且任一整形器启用，报上游同文 config_error。

### `[adxl345]` / `[adxl345 <name>]` — 加速度计（wave-2）

| 选项 | 默认 | 说明 |
|------|------|------|
| `cs_pin` | —（SPI 组，必填） | 片选 |
| `axes_map` | `x,y,z` | 轴映射（可负，如 `-x,-y,z`） |
| `rate` | `3200` | 采样率；非法值报 `Invalid rate parameter: %d` |

段/命令层/加速度计接口（`start_internal_client`）与 mux 端点 `adxl345/dump_adxl345`（key `sensor`）已落地。**gap：bulk 数据通路未接**——共享 `FixedFreqReader` 现绑死 LDC1612 的 4 字节样本与 `query_status_ldc1612`，ADXL345 需 `"BBBBB"` 与 `query_adxl345_status`（泛化归 main 的共享基础设施任务）。

### `[mpu9250]` / `[mpu9250 <name>]` — 加速度计（wave-2）

| 选项 | 默认 | 说明 |
|------|------|------|
| `i2c_address` | `0x68` | I2C 地址 |
| `i2c_speed` | `400000` | I2C 速率 |
| `rate` | `4000` | 采样率 |
| `axes_map` | `x,y,z` | 轴映射 |

段/命令层/接口与 mux 端点 `mpu9250/dump_mpu9250`（key `sensor`）已落地（带名节按 identifier 注册）。**gap**：同 ADXL345 的 bulk 数据通路（需 `">hhh"` 与 `query_mpu9250_status`），`ACCELEROMETER_*` 命令随之延后。

### `[buttons]` — filament 传感器的依赖对象（wave-2，最小实现）

`[buttons]`（`register_debounce_button` / `register_buttons`）作为 `[filament_*]` 的 `load_object` 依赖落地；假 MCU 下无按钮事件源。

### `[pause_resume]` — 暂停/恢复（批 #15）

| 选项 | 默认 | 说明 |
|------|------|------|
| `recover_velocity` | `50.0` | `RESUME` 未给 `VELOCITY=` 时的恢复速度（mm/s） |

四条命令 `PAUSE` / `RESUME`（`VELOCITY=`）/ `CLEAR_PAUSE` / `CANCEL_PRINT` 与 `get_status` 的 `is_paused` 已落地（help 与状态文案逐字照上游）。**gap**：`virtual_sdcard` 无 `is_active`/`do_pause`/`do_resume`/`do_cancel`（SD 分支不可达，命令走 `action:paused`/`action:resumed`/`action:cancel`）；`pause_resume/{pause,resume,cancel}` 三个 webhooks 端点未注册。

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
gap：本仓无 heater 注册表（H1），`heater` 名按 printer 对象名解析；语料中 2 个 run 的 `tachometer_pin`（`generic-prusa-buddy`、`printer-prusa-mini-plus-2020`）批 #8 已接通，实测两条 `run_case OK`。

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

命令见 G-Code 参考的 `MANUAL_STEPPER`。gap（如实登记）：本仓只推进时间线与 `commanded_pos`，**不 append trapq、不产生 steps**；`G1 A<..>` 的 extra axis 词被 `gcode_move` 静默忽略；有 endstop 时的 `STOP_ON_ENDSTOP` 报 `Manual stepper homing is not implemented in this host`。

### `[display]` — 液晶与编码器（st7920/hd44780/uc1701/ssd1306 已落地，批 #7+#13）

| 选项 | 默认 | 说明 |
|------|------|------|
| `lcd_type` | —（必填） | 已实现 `st7920`/`hd44780`/`uc1701`/`ssd1306`/`aip31068_spi`；其余（`sh1106`/`hd44780_spi`/`emulated_st7920`/…）报 `lcd_type '<x>' is not implemented in this host` |
| `cs_pin` / `sclk_pin` / `sid_pin` | — | st7920 三线（同 MCU，否则 `st7920 all pins must be on same mcu`） |
| `rs_pin`/`e_pin`/`d4_pin`…`d7_pin` | — | hd44780 4-bit 并行六线（同 MCU，否则 `hd44780 all pins must be on same mcu`）；`hd44780_protocol_init`（默认 True）、`line_length`（16\|20，默认 20，非法值 choice 文案） |
| `uc1701`/`ssd1306` 面板选项 | — | `a0_pin`(uc1701 必填)/`dc_pin`、`contrast`/`vcomh`/`invert`(ssd1306)、`rst_pin`/`reset_pin`；SPI 走 `McuSpi::send` 只入队（真传输在假 MCU 不可用，已知约束） |
| `aip31068_spi` 面板选项 | — | `latch_pin`（必填，SPI cs/latch）、`line_length`（16\|20，默认 20）、`spi_speed`（≥100000，默认 100 kHz）、`spi_bus`/`spi_software_*`；传输是 9-bit 字（`encode`/`encoded_groups`，批 #28） |
| `display_group` | `_default_16x4` | 组来自随模块发布的 `display.cfg`（20 列时默认 `_default_20x4`）；未知组报 `Unknown display_data group '%s'` |
| 菜单/按键选项 | — | `menu_root`/`menu_timeout`/`menu_reverse_navigation`/`encoder_pins`/`encoder_steps_per_detent`/`encoder_fast_rate`/`click_pin`/`back_pin`/`up_pin`/`down_pin`/`kill_pin`/`analog_range_*`/`analog_pullup_resistor` **只被读取**（菜单未实现） |

gap（如实登记）：**屏幕内容不渲染**（`display_template`/`display_data`/`display_glyph` 只解析+存储；刷新只 clear/flush）；**菜单未实现**（无 `menu` 对象、无 `menu:*` 事件、不装载 `menu.cfg`）；随模块发布的 `display.cfg` 为 **vendored 副本**（有漂移守卫测试）。

### `[fan_generic <name>]` — 通用风扇（批 #18）

本身不读选项：整节交给 `[fan]` 的核心（`pin`/`max_power`/`kick_start_time`/`off_below`/`cycle_time`/`hardware_pwm`/`shutdown_speed`/`enable_pin`/`tachometer_*`；**`shutdown_speed` 默认 `0.0`**，与 `[heater_fan]` 的 1.0 不同）；节名即 mux 值，注册 `SET_FAN_SPEED FAN=<name> SPEED=<0..1>`。`TEMPLATE=` 形式**未实现**（模板求值器未落地，调用报明确拒绝）。

### `[idle_timeout]` — 空闲超时（批 #21）

| 选项 | 默认 | 说明 |
|------|------|------|
| `timeout` | `600` | 空闲秒数，`above 0` |
| `gcode` | 内置（`{% if 'heaters' in printer %}` → `TURN_OFF_HEATERS`；`M84`） | 超时后跑的脚本 |

`SET_IDLE_TIMEOUT [TIMEOUT=]` 可改超时；`get_status` 给 `state`（`Idle`/`Ready`/`Printing`）、`printing_time`、`idle_timeout`。gap：`TURN_OFF_HEATERS` 未实现（默认脚本里那行走「未知命令」）；上游的 `update_timer` 抢占唤醒与 `toolhead:sync_print_time` 事件在本仓降级为定时器自续期 + 观察 `print_time` 前进。

### `[verify_heater <heater_name>]` — 加热器升温校验（批 #20）

| 选项 | 默认 | 说明 |
|------|------|------|
| `hysteresis` | `5` | 目标附近容差（`minval=0.`） |
| `max_error` | `120` | 累积误差上限（`minval=0.`） |
| `heating_gain` | `2` | 期望升温（`above 0.`） |
| `check_gain_time` | `60`（`heater_bed`）/ `20`（其他） | 升温窗口秒数（`minval=1.`） |

每个 heater 自动获得一个检查器（配置无此节时用默认值）；失败 `invoke_shutdown("Heater <name> not heating at expected rate" + 提示)`。已知差异：`[verify_heater <不存在的 heater>]` 报装载器的 `Section '…' is not a valid config section`，而非上游的 `Unknown heater`。

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

### `[respond]` — 主机回显（批 #29）

| 选项 | 默认 | 说明 |
|------|------|------|
| `default_type` | `echo` | `M118`/`RESPOND` 的默认前缀：`echo`→`echo:`、`command`→`//`、`error`→`!!`（choice，大小写敏感） |
| `default_prefix` | 取自 `default_type` | 覆盖默认前缀 |

注册就绪前可用的两条命令：`M118 <原文>`（前缀 + 原文）与 `RESPOND [TYPE=] [PREFIX=] [MSG=]`（`TYPE` 多了 `echo_no_space`：前缀同为 `echo:` 但**不加空格**）。

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

### `[load_cell]` — 称重传感器（hx711/hx717 可用，批 #14）

| 选项 | 默认 | 说明 |
|------|------|------|
| `sensor_type` | —（必填） | `hx711`/`hx717` 已实现；`ads1220`/`ads131m0x` 等报 `sensor_type '<x>' is not implemented in this host` |
| `dout_pin` / `sclk_pin` | — | 数据与时钟（必须同 MCU，否则 `… config error: All pins must be connected to the same MCU`） |
| `sample_rate` / `gain` | 按芯片 | HX711：80/10、A-128/B-32/A-64（默认 80、A-128）；HX717：320/80/20/10、A-128/B-64/A-64/B-8（默认 320、A-128） |
| `reference_tare_counts` / `counts_per_gram` / `sensor_orientation` | — / — / `normal` | 校准三件套（`counts_per_gram` minval 1.0，非法值 `must have minimum of 1.0`） |

端点 `load_cell/dump_force`（mux key `load_cell`，四列 `time, force (g), counts, tare_counts`）。gap：`ads1220`/`ads131m0x`（LC-3）、`[load_cell_probe]` 未做、`hx71x_attach_trigger_analog` 未接。

### `[tmc2208 <stepper>]` / `[tmc2209 <stepper>]` — TMC 步进驱动（UART，批 #7）

| 选项 | 默认 | 说明 |
|------|------|------|
| `uart_pin` | —（必填） | UART 单线（`tx_pin` 缺省复用） |
| `select_pins` / `uart_address` | — | 模拟多路复用（同 mcu、同 pin、`(id,addr)` 唯一） |
| `run_current` / `sense_resistor` / `stealthchop_threshold` | — / 0.110 / — | 电流与静音阈值 |
| `interpolate` / `driver_SGTHRS` / `diag_pin` | True / — / — | 仅 tmc2209 |

命令：`INIT_TMC` / `DUMP_TMC` / `SET_TMC_FIELD` / `SET_TMC_CURRENT`（mux 键 `STEPPER`）；`tmc2209_<stepper>:virtual_endstop` 可用作端停。gap：fileoutput 下总线读短路为 0；`tmc2130`/`tmc2660`/`tmc5160`/`tmc2240` 与 SPI 传输未实现。

