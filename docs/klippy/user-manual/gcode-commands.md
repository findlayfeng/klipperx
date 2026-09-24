# G-Code 命令参考

本章节描述 KlipperX 已实现的 G-Code 命令格式与用法。命令表以源码里的注册点为准
（`gcode.rs` 的内置命令，以及各 `extras` 模块在配置装载时注册的部分）；没有列在这里的
命令要么尚未实现，要么走[未注册命令的处理](#未注册命令的处理)。

## 命令格式总览

KlipperX 支持两类 G-Code 命令：**传统命令**（Traditional）和**扩展命令**（Extended）。

| 类型 | 识别规则 | 参数格式 | 示例 |
|------|----------|----------|------|
| 传统命令 | 大写字母 + 数字开头，且后面整体是个数字（`G1`、`M110`） | 字母 + 值（空格分隔） | `G1 X10.5 Y20 F1000` |
| 扩展命令 | 全大写字母/数字/下划线；**首字符不能是数字，第二个字符也不能是数字** | `KEY=VALUE`（等号分隔） | `SET_PIN PIN=fan VALUE=1` |

分类由名字本身决定：`G1X` 这种「字母+数字+字母」既不是传统命令，也会因为第二位是数字而
不被接受为扩展命令名。

### 公共语法

| 语法 | 说明 | 生效范围 |
|------|------|----------|
| `N<digits>` | 行号前缀，被解析器跳过 | 两类命令 |
| `;` | 注释起始符，其后内容被忽略 | 两类命令（整行层面） |
| `#` | 注释起始符 | **仅扩展命令**的参数部分 |
| `'...'` / `"..."` | 引号包裹（支持 `\` 转义），保留空格与特殊字符 | **仅扩展命令**的参数部分 |
| 大小写 | 命令名与参数名不区分大小写（解析时统一大写） | 两类命令 |

参数**值**的大小写按类型不同：扩展命令的值原样保留（`SET_PIN PIN=my_fan` 里的 `my_fan`
不会被改动），传统命令的值来自大写化后的行（`X10` 这类数值不受影响）。

---

## 就绪前可用性（`when_not_ready`）

命令分两批：

| 批次 | 何时可执行 | 命令 |
|------|-----------|------|
| 就绪前可用（`when_not_ready`） | **打印机就绪前也能用**（客户端在启动期也需要急停与状态） | `M110` `M112` `M115` `RESTART` `FIRMWARE_RESTART` `ECHO` `STATUS` `HELP`（构造时注册的内置批），以及由 `gcode_move` 注册的 `M114` |
| 其余 | 只在打印机 **ready** 后可执行 | 运动、温度、引脚、总线等所有由配置装载的模块注册的命令 |

未就绪时执行后一批命令不会命中处理器，而是由[未注册命令的处理](#未注册命令的处理)返回
当前状态消息（`!! <state message>`）。`HELP` 在未就绪时只列出前一批里带描述的命令（即
`RESTART` / `FIRMWARE_RESTART` / `STATUS` / `HELP`），并在首行提示
`Printer is not ready - not all commands available.`。

---

## 内置命令

这些由调度器 `GCodeDispatch` 在构造时注册，不依赖任何配置节。

### M110 — 设置当前行号

```
M110 [S<line_number>]
```

被接受但忽略，不产生任何效果。

### M112 — 紧急停止

```
M112
```

立即触发打印机停机（状态消息 `Shutdown due to M112 command`），所有运动与加热停止。

### M115 — 获取固件版本

```
M115
```

返回主机自己的版本号（启动参数 `software_version`，主机尚未设置时回退到 crate 版本）：

```
// FIRMWARE_NAME:Klipper FIRMWARE_VERSION:<version>
```

需要 `ok` 应答的输入源（文件输入）下这条走 `ok <msg>` 而不是 `// ` 行，与上游一致。

### RESTART / FIRMWARE_RESTART — 重启

| 命令 | 作用 |
|------|------|
| `RESTART` | 重新装载配置并重启主机软件（换一份对象图，进程不退出） |
| `FIRMWARE_RESTART` | 重启固件 + 主机 + 重新装载配置 |

两者都会触发 `gcode:request_restart` 事件（携带当前打印时间）。

### ECHO — 回显命令行

```
ECHO Hello, KlipperX!
```

把**整行原样**回显（不记日志）：

```
// ECHO Hello, KlipperX!
```

### STATUS — 打印机状态

```
STATUS
```

- 就绪时：`// Klipper state: Ready`
- 未就绪时：作为**错误**报告，内容是当前状态消息 + `Klipper state: Not ready`

状态行不是只能靠轮询拿：调度器订阅了打印机生命周期事件，状态一变就向 g-code 输出
通道各推一行（`klippy:ready` → `// Klipper state: Ready`、`klippy:shutdown` →
`// Klipper state: Shutdown`、`klippy:disconnect` → `// Klipper state: Disconnect`）。
其中停机那条只在第一次停机时打印（未就绪后不再重复）。

### HELP — 列出可用命令

```
HELP
```

列出**带描述文本**的已激活命令，按名字排序，每行 `{命令名（至少 10 字符宽）}: {描述}`，
不记日志。未就绪时只列就绪前那批里带描述的命令并加提示行。注册时没给 `desc` 的命令
（`ECHO` `M110` `M112` `M115` `M114` `G0` `G1` `G4` `M400` `M106` `M107` 等）不出现在列表里。
配置装载后的典型输出形如：

```
// Available extended commands:
// ACTIVATE_EXTRUDER       : Change the active extruder
// FIRMWARE_RESTART        : Restart firmware, host, and reload config
// G28                     : Home one or more axes
// HELP                    : Report the list of available extended G-Code commands
// IIC_READ                : Write then read bytes from an I2C device (debug)
// IIC_WRITE               : Write bytes to an I2C device (debug)
// M104                    : Set extruder temperature
// RESTART                 : Reload config file and restart host software
// SET_HEATER_TEMPERATURE  : Set a heater temperature
// SET_PIN                 : Set the value of a pin
// STATUS                  : Report the printer status
```

（实际条目取决于配置里装载了哪些节。API 侧的 `gcode/help` 返回同一张表，形状是
`{名字: 描述}` 的 JSON 对象。）

---

## 运动与坐标系（`gcode_move`）

`G0`/`G1` 这些解释坐标的命令由 `gcode_move`（上游 `klippy/extras/gcode_move.py`）注册。
它**不是配置节**：`[printer]` 的工厂装载它（上游是 toolhead 装完自己再按名字加载默认模块）。

### G0 / G1 — 直线移动

```
G0 [X<x>] [Y<y>] [Z<z>] [E<e>] [F<f>]
G1 [X<x>] [Y<y>] [Z<z>] [E<e>] [F<f>]
```

- 每个轴词按**当前模式**解释：绝对模式下相对锚点（`base_position`），相对模式下相对上一次位置；
- `E` 单独由 `M82`/`M83` 决定，并先乘挤出系数；
- `F` 是 mm/min，必须大于 0，换算成 mm/s（默认系数 1/60）后再乘 `M220` 的速度系数，
  结果**一直保存**——下次不给 `F` 就沿用它；初始速度 `25` mm/s；
- 未给出的轴沿用当前位置（不会拖动旧坐标）；
- `F<=0` 报 `Invalid speed in '<整行>'`；坐标系还没就绪时报 `Printer is not ready`。

`G0` 与 `G1` 是同一个处理器（上游也是）。只改变 `E` 的移动会被解析并记录，但不产生步进
（挤出机不是运动学导轨）。

坐标是**工具头坐标**：`gcode_move` 把 g-code 坐标算好后交给 toolhead，toolhead 从不直接
看 g-code 坐标。因此只要 toolhead 在 `G1` 之外动过（归零、`SET_KINEMATIC_POSITION`、
手动移动、上一条命令半途失败），状态就要重新锚定，否则下一条 `G1` 会拖着旧坐标跑。
重新锚定由这些事件驱动：`klippy:ready`、`homing:home_rails_end`（带 `axes`）、
`toolhead:set_position`、`gcode:command_error`。

### G20 / G21 — 单位

| 命令 | 行为 |
|------|------|
| `G21` | 毫米（唯一单位），无操作 |
| `G20` | 英寸：**拒绝**，报 `Machine does not support G20 (inches) command (<line>)` |

### G90 / G91 — XYZ 模式

`G90`：XYZ 词绝对；`G91`：XYZ 词相对。

### M82 / M83 — 挤出模式

`M82`：`E` 绝对；`M83`：`E` 相对。与 `G90`/`G91` **互不影响**（XYZ 与 E 是两套开关）。

### G92 — 重新锚定

```
G92 [X<x>] [Y<y>] [Z<z>] [E<e>]
```

把锚点移到使当前 g-code 位置读作所给值的位置。不带任何词的 `G92` 让四个轴全部读作 0。

### M114 — 报告 g-code 位置

```
M114
```

```
X:<x> Y:<y> Z:<z> E:<e>
```

保留三位小数，直接输出（无 `// ` 前缀）；**未就绪时也可执行**。

### M220 / M221 — 速度/挤出系数

| 命令 | 参数 | 说明 |
|------|------|------|
| `M220` | `S`（百分比，默认 `100`，必须 > 0） | 速度系数 |
| `M221` | `S`（百分比，默认 `100`，必须 > 0） | 挤出系数（作用在 `E` 上） |

### SET_GCODE_OFFSET — 虚拟偏移

```
SET_GCODE_OFFSET [X<v>] [Y<v>] [Z<v>] [E<v>] [<轴>_ADJUST<v>] [MOVE=0|1] [MOVE_SPEED=<mm/s>]
```

- 直接给 `X=` 设置该轴偏移；给 `X_ADJUST=` 则在当前偏移上增减（`Y`/`Z`/`E` 同）；
- 偏移同时记进 `homing_position`，归零时会被重新施加；
- `MOVE=1` 立即把工具头按偏移量移动，速度用 `MOVE_SPEED`（缺省为当前速度，必须 > 0）。

### SAVE_GCODE_STATE / RESTORE_GCODE_STATE

```
SAVE_GCODE_STATE [NAME=<name>]
RESTORE_GCODE_STATE [NAME=<name>] [MOVE=0|1] [MOVE_SPEED=<mm/s>]
```

按名字保存/恢复整套坐标状态（绝对/相对模式、锚点、位置、速度与两个系数）。
`NAME` 默认 `default`；恢复一个不存在的名字报 `Unknown g-code state: <name>`。
`RESTORE_… MOVE=1` 会带着恢复的位置走回去，挤出轴保持原位（差额记进锚点）。

`gcode_move` 自己也是一个可查询对象（对象名 `gcode_move`，`objects/list` 里能看到）。
`get_status` 报：`speed`（mm/min）、`speed_factor`（倍数，`1.0` 即 100%）、`extrude_factor`、
`absolute_coordinates` / `absolute_extrude`、`homing_origin`、`position`（工具头坐标）、
`gcode_position`（g-code 坐标）、`axis_map`（`X/Y/Z/E` → 0..3）。

---

## 运动控制（`toolhead`，由 `[printer]` 装载）

`[printer]` 是最后一个装载的节（运动学需要各 `[stepper_*]` 先在场）。它注册下面四条；
解释坐标那一组（`G0` `G1` `G90` …）由它顺带装载的 [`gcode_move`](#运动与坐标系gcode_move) 注册。

### G4 — 暂停

```
G4 [P<milliseconds>] [S<seconds>]
```

给了 `S` 就用 `S`（秒），否则用 `P`（毫秒）除以 1000；负值按 0 处理。

```
G4 P2000     ; 停 2 秒
G4 S1.5      ; 停 1.5 秒
```

### M400 — 等待移动完成

排空规划器（把已排队的移动都走完），无参数。

### SET_KINEMATIC_POSITION — 强制设置底层位置

```
SET_KINEMATIC_POSITION [X<v>] [Y<v>] [Z<v>] [SET_HOMED=<axes>] [CLEAR=<axes>] [CLEAR_HOMED=<axes>]
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `X` / `Y` / `Z` | 当前位置 | 新的位置 |
| `SET_HOMED` | `xyz` | 设置为「已归位」的轴 |
| `CLEAR` / `CLEAR_HOMED` | 空 | 清除归位状态的轴（两者等价，`CLEAR_HOMED` 优先） |

会发 `toolhead:set_position` 事件，`gcode_move` 借此重新锚定（否则下一条 `G1` 会拖旧坐标）。

### G28 — 归零

```
G28 [X] [Y] [Z]
```

点名的轴归零；**一个都不给就归零 XYZ 三轴**。归零由固件触发（endstop 命中即停步），
是本主机里少数几个 `async` 处理器之一。

---

## 电机使能（`stepper_enable`）

| 命令 | 参数 | 说明 |
|------|------|------|
| `M18` | 无 | 关闭全部电机 |
| `M84` | 无 | `M18` 的别名 |
| `SET_STEPPER_ENABLE` | `STEPPER=<name>`（必需）、`ENABLE=<0\|1>`（默认 `1`） | 单独开关某个步进电机 |

`M18`/`M84` 会广播 `stepper:motor_off`，让其余部件一起收尾。

---

## 温度与加热器

### SET_HEATER_TEMPERATURE — 设置加热器目标温度

```
SET_HEATER_TEMPERATURE HEATER=<name> [TARGET=<celsius>]
```

- `HEATER` 是**多路键**：值是加热器的短名——`extruder`（`[extruder]`）、`heater_bed`
  （`[heater_bed]`）、`[heater_generic <name>]` 的 `<name>` 等；
- `TARGET` 默认 `0`（关掉）。

### M104 / M109 — 设置挤出机温度

```
M104 [S<temperature>] [T<index>]
M109 [S<temperature>] [T<index>]
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `S` | `0` | 目标温度（摄氏度） |
| `T` | `0` | 挤出机编号：`0` 是 `extruder`，`n` 是 `extruder<n>` |

只有名字是 `extruder` 的主挤出机注册这两条（与上游一致）。**当前 `M109` 不等待升温**，
与 `M104` 行为相同（`_wait` 参数尚未接上）。编号指向的挤出机不存在时报
`Extruder not configured`。

### M140 / M190 — 设置热床温度

```
M140 [S<temperature>]
M190 [S<temperature>]
```

`S` 默认 `0`。**当前 `M190` 不等待升温**，与 `M140` 行为相同。

### M106 / M107 — 风扇（由 `[fan]` 装载）

```
M106 [S<0..255>]
M107
```

- `M106` 的 `S` 默认 `255`、下限 `0`、**没有上限**（真正封顶的是 `max_power`），内部除以 255
  得到占空比；
- `M107` 等价于 `M106 S0`。

---

## 挤出机

### SET_PRESSURE_ADVANCE — 设置压力推进（多路键 `EXTRUDER`）

```
SET_PRESSURE_ADVANCE [EXTRUDER=<name>] [ADVANCE=<mm>] [SMOOTH_TIME=<s>]
```

两个参数都缺省为该挤出机的当前值，回显两行：

```
// pressure_advance: 0.050000
// pressure_advance_smooth_time: 0.040000
```

### ACTIVATE_EXTRUDER — 切换活动挤出机（多路键 `EXTRUDER`）

```
ACTIVATE_EXTRUDER EXTRUDER=<name>
```

把工具头的活动挤出机设为 `<name>`，回显 `// Activating extruder <name>`。

---

## 引脚输出

### SET_PIN — 设置引脚值（多路键 `PIN`）

```
SET_PIN PIN=<name> VALUE=<0..scale>
```

每个 `[output_pin <name>]` 装载时注册一个 `PIN` 值，因此 `SET_PIN` 是**多路命令**。

| 参数 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `PIN` | 字符串 | 是 | 对应 `[output_pin <name>]` 里的 `<name>` |
| `VALUE` | `0.0 ~ scale` | 是 | 数字输出：`>= 0.5` 为高电平；PWM：除以 `scale` 后作为占空比 |

`scale` 是 PWM 路径的选项（默认 `1`，数字输出恒为 1）：`VALUE` 先按 `scale` 归一，
再交给资源。

配置示例：

```ini
[output_pin my_fan]
pin: PA0
value: 0
shutdown_value: 0

[output_pin pwm_fan]
pin: PA1
pwm: true
cycle_time: 0.02
```

```
SET_PIN PIN=my_fan VALUE=1      ; 数字输出：打开
SET_PIN PIN=my_fan VALUE=0      ; 数字输出：关闭
SET_PIN PIN=pwm_fan VALUE=0.25  ; PWM：25% 占空比
```

值不合法时报错（候选排序后给出）：

```
!! The value 'unknown' is not valid for PIN. Options: 'my_fan', 'my_light'
```

---

## 端停查询

### QUERY_ENDSTOPS / M119

```
QUERY_ENDSTOPS
M119
```

查询所有已注册端停，按注册顺序输出（`respond_raw`，不带 `// ` 前缀）：

```
stepper_x:open stepper_y:open stepper_z:TRIGGERED
```

两条名字注册到同一个处理器上。

---

## 调试总线命令（KlipperX 自有）

以下四条是本项目为真机自测加的，上游没有对应 G-Code；每个设备节注册一个多路值。

### IIC_WRITE / IIC_READ — I2C（由 `[i2c_device <name>]` 装载）

```
IIC_WRITE DEVICE=<name> DATA=<hex>
IIC_READ  DEVICE=<name> [WRITE=<hex>] READ_LEN=<n>
```

| 参数 | 必需 | 说明 |
|------|------|------|
| `DEVICE` | 是 | 设备名，对应 `[i2c_device <name>]` 的 `<name>` |
| `DATA` | 写时必需 | 十六进制字节串（`6b`、`0102ff`），空白会被忽略 |
| `WRITE` | 否 | 读之前先写出的字节（寄存器地址） |
| `READ_LEN` | 读时必需 | 读取字节数，`0..=255` |

```
IIC_WRITE DEVICE=accel DATA=6b     // i2c write ok
IIC_READ  DEVICE=accel WRITE=75 READ_LEN=1   // i2c read: 68
```

NACK 只作为结果回显，不停机（这是 bring-up 探测路径）。

### SPI_TRANSFER / SPI_SEND — SPI（由 `[spi_device <name>]` 装载）

```
SPI_TRANSFER DEVICE=<name> DATA=<hex>
SPI_SEND     DEVICE=<name> DATA=<hex>
```

一次调用 = 一个完整片选周期（拉低片选 → 移出 `DATA` → 读回等长字节 → 释放片选）。
`SPI_SEND` 不读回。

```
SPI_TRANSFER DEVICE=flash DATA=9f000000   // spi transfer: ffef3013
SPI_SEND     DEVICE=flash DATA=04          // spi send ok
```

> W25 系列 flash 的 JEDEC ID：发 `9f` + 3 个哑字节，返回的后 3 字节是厂商/类型/容量。

---

### BED_TILT_CALIBRATE — 床面倾斜校准（`[bed_tilt]` 且写了 `points` 时才注册）

逐点探测（或手动测点）拟合床面倾斜平面并应用到当前会话：更新 `x_adjust`/`y_adjust`/`z_adjust`
并重锚 G-Code 坐标，同时把新值记入 `SAVE_CONFIG` 待写区。

| 参数 | 说明 |
|------|------|
| `METHOD` | `automatic`（默认）或 `manual`（配合 `G1` + `ACCEPT` 手动测点） |
| `HORIZONTAL_MOVE_Z` | 覆盖段内的 `horizontal_move_z` |
| `PROBE_SPEED` / `LIFT_SPEED` / `SAMPLES` / `SAMPLE_RETRACT_DIST` / `SAMPLES_TOLERANCE` / `SAMPLES_TOLERANCE_RETRIES` / `SAMPLES_RESULT` | 自动模式下透传给探针 |

与 `Z_TILT_ADJUST`/`QUAD_GANTRY_LEVEL` 不同，它**没有** `RETRIES`/`RETRY_TOLERANCE`，也不调整电机。

### Z_TILT_ADJUST — 多 Z 电机调平（`[z_tilt]`）

探测若干点，按平面拟合算出每个 Z 电机的调整量，逐个脱挂/挂回依次移动电机使床面（相对喷嘴）水平。

| 参数 | 说明 |
|------|------|
| `METHOD` | `automatic`（默认）或 `manual`（`G1` + `ACCEPT` 手动测点） |
| `HORIZONTAL_MOVE_Z` | 覆盖段内 `horizontal_move_z` |
| `RETRIES` / `RETRY_TOLERANCE` | 覆盖段内 `retries` / `retry_tolerance`（范围 0..30 / 0..1） |
| `PROBE_SPEED` / `LIFT_SPEED` / `SAMPLES` / `SAMPLE_RETRACT_DIST` / `SAMPLES_TOLERANCE` / `SAMPLES_TOLERANCE_RETRIES` / `SAMPLES_RESULT` | 自动模式透传给探针 |

`get_status` 报 `applied`（本轮是否调整过；`stepper_enable:motor_off` 会复位）。

### QUAD_GANTRY_LEVEL — 四点龙门调平（`[quad_gantry_level]`）

4 个探测点 + 2 个 `gantry_corners`，用精确两点直线拟合求四角高度，按差值逐电机调整；调整量超过
`max_adjust`（默认 4.0）会中止并报错。参数与 `Z_TILT_ADJUST` 相同（`METHOD`/`HORIZONTAL_MOVE_Z`/
`RETRIES`/`RETRY_TOLERANCE` + 探针透传组）。注意两个与上游一致的细节：段内 `horizontal_move_z`
**不**被命令行 `HORIZONTAL_MOVE_Z` 覆盖；重试用的是**机架相对高度**而非原始探测 Z。

### BLTOUCH_DEBUG / BLTOUCH_STORE — BLTouch 诊断与模式（由 `[bltouch]` 注册）

| 命令 | 参数 | 说明 |
|------|------|------|
| `BLTOUCH_DEBUG` | `COMMAND`（可选） | 按上游语义下发命令并回显探针状态（细节见 `extras/bltouch.rs` 的 `cmd_BLTOUCH_DEBUG`） |
| `BLTOUCH_STORE` | `MODE`（可选） | 按上游语义存储/回读模式（`cmd_BLTOUCH_STORE`） |

### SET_SMART_EFFECTER / RESET_SMART_EFFECTOR — Smart Effector 调参（由 `[smart_effector]` 注册）

| 命令 | 参数 | 说明 |
|------|------|------|
| `SET_SMART_EFFECTER` | `SENSITIVITY`(0..255)、`ACCEL`(≥0)、`RECOVERY_TIME`(≥0) | 缺省取当前值；错误文案含上游的 `accelartion` 拼写 |
| `RESET_SMART_EFFECTOR` | 无 | **仅当配置了 `control_pin` 才注册**；按 1000 bits/s 向控制脚发 `[131,131]` 帧复位 |

### LDC_CALIBRATE_DRIVE_CURRENT — LDC1612 驱动电流标定（由 `ldc1612` 对象注册，mux 键 `CHIP=`）

| 命令 | 参数 | 说明 |
|------|------|------|
| `LDC_CALIBRATE_DRIVE_CURRENT` | `CHIP`（传感器名） | 对目标 LDC1612 做驱动电流标定，回显 `reg_drive_current` 提取值并给出 `SAVE_CONFIG` 提示（细节见 `extras/ldc1612.rs`）；`ldc1612` 对象由 probe_eddy_current 构造，该命令随对象装载注册 |

## 未注册命令的处理

没有命中处理器的命令走默认处理器（对应上游 `klippy/gcode.py` 的 `cmd_default`）：

| 情况 | 行为 |
|------|------|
| 打印机**未就绪** | 返回状态消息作为错误：`!! <state message>` |
| `M105` | 需要应答的输入源回 `ok T:0`（启动期轮询不报错） |
| `M21` | 静默成功（没有 SD 卡模块） |
| `M140` / `M104` 且 `S0`（没有对应加热器） | 静默成功 |
| `M107`，或 `M106 S0`（没有风扇） | 静默成功 |
| 其余 | 回一行信息：`// Unknown command:"<名字>"` |

注意最后一行是 `// ` 信息而不是 `!! ` 错误——上游同样**回答**未知命令而不是失败，
所以切片机发一堆本主机没有的命令不会中断脚本。

---

## 命令注册说明

- **内置命令**（`M110` `M112` `M115` `RESTART` `FIRMWARE_RESTART` `ECHO` `STATUS` `HELP`）
  在调度器构造时注册，标了 `when_not_ready`，就绪前即可用。
- **配置驱动的命令**由各模块在配置装载时注册，通常只在打印机就绪后激活（例外：`M114`，
  见上面的[就绪前可用性](#就绪前可用性when_not_ready)）；没有对应配置节就没有对应的命令
  （或没有多路值）：
  - `[printer]` → `G4` `M400` `SET_KINEMATIC_POSITION` `G28`（坐标系那组由它顺带装载的
    `gcode_move` 注册）；
  - `[stepper_enable]` → `M18` `M84` `SET_STEPPER_ENABLE`；
  - `[extruder]` / `[heater_bed]` / `[heater_generic]` → `M104` `M109` `M140` `M190`
    与多路的 `SET_HEATER_TEMPERATURE`；
  - `[fan]` → `M106` `M107`；`[output_pin …]` → 多路的 `SET_PIN`；
  - `[i2c_device …]` / `[spi_device …]` → 调试总线四条；
  - 端停对象 → `QUERY_ENDSTOPS` / `M119`。
- **多路命令**（`SET_PIN`、`SET_HEATER_TEMPERATURE`、`SET_PRESSURE_ADVANCE`、
  `ACTIVATE_EXTRUDER`、`IIC_*`、`SPI_*`）：第一次注册固定键参数，之后每个值一条处理器；
  同一个键不能注册两个值。没有注册默认值时键参数是**必需**的。
- **运动相关命令**（`G0` `G1` `G4` `M400` `G28` …）不由 dispatcher 自己实现，而是分别来自
  `gcode_move` 与 `toolhead`。`gcode_move` 不是配置节，由 `[printer]` 的工厂按名字加载，
  所以只要配置里有 `[printer]`，坐标系命令就在。

---

## 命令解析细节

### 传统命令

参数按「字母 + 值」配对解析，空格分隔：

| 输入 | 命令 | 参数 |
|------|------|------|
| `G1 X10.5 Y20 F1000` | `G1` | `X=10.5`, `Y=20`, `F=1000` |
| `M115` | `M115` | 无 |
| `G4 P2000` | `G4` | `P=2000` |
| `N5 G1 X10` | `G1` | `X=10` |

### 扩展命令

参数按 `KEY=VALUE` 解析，支持 shell 风格的引号与转义，键统一大写，**值保留原样**；
遇到 `#` 或 `;` 结束：

| 输入 | 命令 | 参数 |
|------|------|------|
| `SET_PIN PIN=fan VALUE=1` | `SET_PIN` | `PIN=fan`, `VALUE=1` |
| `SET_PIN PIN='my pin' VALUE=0` | `SET_PIN` | `PIN=my pin`, `VALUE=0` |
| `SET_PIN PIN=fan VALUE=1 ; turn on` | `SET_PIN` | `PIN=fan`, `VALUE=1` |
| `SET_PIN PIN=fan VALUE=1 # 注释` | `SET_PIN` | `PIN=fan`, `VALUE=1` |

一个 token 里没有 `=`（且不是注释）就是 `Malformed command '<整行>'`。

### 行号

`N<digits>` 作为前缀被跳过，不影响执行：

| 输入 | 实际命令 |
|------|----------|
| `N5 G1 X10` | `G1 X10` |
| `N100 M110 S50` | `M110 S50` |

---

## 命令错误

命令失败时以 `!! ` 前缀回一行（多行错误的第一行用 `!! `，其余行变成 `// ` 行）：

```
!! Error on 'SET_PIN PIN=fan': missing VALUE
!! Unable to parse move 'G1 Xabc'
!! Error on 'SET_PIN PIN=fan VALUE=-1': VALUE must have minimum of 0
!! The value 'unknown' is not valid for PIN. Options: 'my_fan', 'my_light'
!! Machine does not support G20 (inches) command (G20)
!! Printer is not ready
```

几类来源：

| 来源 | 文案 |
|------|------|
| 缺参数 | `Error on '<整行>': missing <参数名>` |
| 解析失败 | `Error on '<整行>': unable to parse <原值>`；移动词另有一套：`Unable to parse move '<整行>'`，`F<=0` 是 `Invalid speed in '<整行>'` |
| 越界 | `Error on '<整行>': <参数名> must have minimum/maximum/above/below of <限制>` |
| 多路键值未注册 | `The value '<值>' is not valid for <键>. Options: 'a', 'b'`（候选里有包含关系时改成 `. Did you mean 'a'?`，**取排序后的第一个**，与上游取字典序最后一个不同——为了消息稳定） |
| 处理器自己的错误 | 各命令自带的文案（如 `Printer is not ready`、`Extruder not configured`） |

命令错误一律**不停机**：它回报客户端并发 `gcode:command_error` 事件，然后按输入源分两种
走向——需要 `ok` 应答的输入源（文件输入）会被 ack 并**继续执行下一行**，不需要应答的
（API 提交的脚本）则**中止整段脚本**，把错误交回给调用方。

会让打印机停机的只有 `M112` 这类显式命令、固件自报的停机，以及处理器 **panic**
（panic 走 `Internal error on command:"<名字>"` 并 `invoke_shutdown`，两者都与命令错误分开）。

---

- [← 用户手册首页](README.md)
