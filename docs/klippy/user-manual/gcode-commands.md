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
| `RESTART` | 从磁盘重新读取配置文件并重启主机软件（换一份对象图，进程不退出） |
| `FIRMWARE_RESTART` | 重启固件 + 主机 + 从磁盘重新读取配置文件 |

两者都会触发 `gcode:request_restart` 事件（携带当前打印时间）。它们都会重新读盘，所以改完
配置文件后发 `RESTART` 就生效；文件读不到或解析失败时打印机进 `error`（带失败原因，进程不
退出），修好后再发一次。API 的 `gcode/restart` / `gcode/firmware_restart` 在 `gcode` 对象尚未
建立（配置没装载成功）时不等 ready，直接请求宿主重启，因此仍是可用的退路。

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
（`ECHO` `M110` `M112` `M115` `M114` `G0` `G1` `G4` `M400` `M204` `M106` `M107` 等）不出现在列表里。
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

`[printer]` 是最后一个装载的节（运动学需要各 `[stepper_*]` 先在场）。它注册下面这一组；
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

### SET_VELOCITY_LIMIT — 运行期改速度上限

```
SET_VELOCITY_LIMIT [VELOCITY=<v>] [ACCEL=<a>] [SQUARE_CORNER_VELOCITY=<v>] [MINIMUM_CRUISE_RATIO=<r>]
```

四个参数都可省，给谁改谁（没给的不动）；改完**同时**进规划器与 `get_status`（不像只改状态
那样会报旧值）。约束与 `[printer]` 的对应选项相同：`VELOCITY` / `ACCEL` 必须**大于** 0，
`SQUARE_CORNER_VELOCITY` 不得小于 0，`MINIMUM_CRUISE_RATIO` 在 `[0, 1)`。

四个都没给时回一段四行当前值（顺序：`max_velocity` / `max_accel` / `minimum_cruise_ratio` /
`square_corner_velocity`）；给了参数则**不**回话（同上游），值另经日志 rollover 行与
`get_status` 可查。

```
SET_VELOCITY_LIMIT VELOCITY=200 ACCEL=3000
SET_VELOCITY_LIMIT          ; 回当前四个值
```

### M204 — 设置加速度

```
M204 [S<accel>] | [P<accel> T<accel>]
```

`S` 优先；没有 `S` 时取 `P`、`T` 里较小者；`P`/`T` 缺任意一个则整条命令报
`Invalid M204 command "<命令行>"` 且**不改任何值**。等价于只带 `ACCEL` 的
`SET_VELOCITY_LIMIT`。注册时没有描述文本，因此不出现在 `HELP` 列表里。

### FORCE_MOVE — 绕过规划器直接移动一个电机

```
FORCE_MOVE STEPPER=<name> DISTANCE=<mm> VELOCITY=<mm/s> [ACCEL=<mm/s²>]
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `STEPPER` | —（必填） | 电机名（mux 键，同 `[stepper_*]` / `[manual_stepper]` 的名字） |
| `DISTANCE` | —（必填） | 有符号距离；负值反向 |
| `VELOCITY` | —（必填） | 必须大于 0 |
| `ACCEL` | `0` | 不得为负；0 表示匀速 |

它把一段梯形直接灌给这个电机自己的时间轴（不走规划器），**会让运动学位置失效**（上游原文
即如此标注），只作诊断用。
**只在 [`[force_move]`](config.md) 的 `enable_force_move: True` 时注册**（默认关）。

### STEPPER_BUZZ — 往复摆动电机以确认身份

```
STEPPER_BUZZ STEPPER=<name>
```

跑 10 个来回：正走 `1 mm`（速度 `4 mm/s`）、停 50 ms、反走 `1 mm`、停 450 ms；角度模式的段
（没有 `rotation_distance` 但有 `gear_ratio`）改走 `1°`（速度 `1°/0.25 s`）。
每个已注册的电机都有一条 `STEPPER_BUZZ STEPPER=<name>`，**不受 `enable_force_move` 影响**。

> **已知缺口（与 `MANUAL_STEPPER MOVE` 同一处）**：`[manual_stepper]` 这类没有接进运动队列
> 的电机，两条命令目前只走时间轴、**不实际出步**——命令会成功返回，但电机会不动。

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

### TEMPERATURE_WAIT — 等待温度越界

```
TEMPERATURE_WAIT SENSOR=<section> [MINIMUM=<celsius>] [MAXIMUM=<celsius>]
```

- `SENSOR` 必填，值是传感器所属**配置节名**（如 `extruder`、`temperature_sensor mcu_temp`、
  `temperature_fan chamber`）——mux 键，每个经 `heaters::register_sensor` 注册的节自动获得条目；
- `MINIMUM` 默认 -∞、`MAXIMUM` 默认 +∞，**至少给一个**（都缺报
  `Error on 'TEMPERATURE_WAIT': missing MINIMUM or MAXIMUM.`）；给了两者时 `MAXIMUM` 必须
  **严格大于** `MINIMUM`（错误文案逐字上游）；
- 等待循环每秒读一次，读数进入 `[MINIMUM, MAXIMUM]` 即返回；期间每轮回一行真报表
 （`M105` 的 gcode-id 表，`<id>:%.1f /%.1f`，空表回 `T:0`；2026-10-03 `14f5055` 接线）；printer shutdown 时中止。

（上游 `G-Codes.md` 同款语义；2026-10-03 `b5da84e`）

### TEMPERATURE_PROBE_CALIBRATE / _NEXT / _COMPLETE / ABORT / ENABLE — 探针温度标定

```
TEMPERATURE_PROBE_CALIBRATE PROBE=<name> [METHOD=tap] [TARGET=<celsius>] [STEP=<celsius>]
TEMPERATURE_PROBE_ENABLE PROBE=<name> [ENABLE=<0|1>]
TEMPERATURE_PROBE_NEXT / TEMPERATURE_PROBE_COMPLETE / ABORT   （标定期间动态出现）
```

- `PROBE` 是 mux 键，值 = `[temperature_probe <name>]` 的 `<name>`；
- **CALIBRATE** 要求（按序）：已注册标定 helper（无则先报 `No calibration helper registered for […]`）
  → 轴已 homed → 已链接探针（`probe` 对象）→ 无手动探针在跑；
  `TARGET` 必须**大于传感器当前平滑读数**（非 `min_temp`）、`STEP ≥ 1.0`，预期样本数 `≥ 3`
  否则报错中止；启动后注册 `TEMPERATURE_PROBE_NEXT`/`_COMPLETE`；裸名 `ABORT` 在**第一轮手动
  探针会话结束后**（finalize 的 `_prepare_next_sample`）注册，覆盖升温等待期；
- 流程：移动到 `calibration_position`（可选加热床/挤出机脚本，经 `TEMPERATURE_WAIT` 等温）
  → 手动探针定零点 → 每轮升温 `STEP` → 升温到位经 kick 自动发 `TEMPERATURE_PROBE_NEXT`
  → 记热膨胀 → 样本满由 `COMPLETE` 收尾（`≥3` 才出结果）、任何时刻 `ABORT` 收尾；
- **ENABLE** 是漂移补偿的真实开关（`ENABLE=<0|1>` 必填，两段拒绝文案逐字；`704ee0d` 接真）；
- 上游 quirk：`TEMPERATURE_PROBE_COMPLETE` 的 help 显示的是 NEXT 的文案（上游注册即如此，
  本仓逐字保留）。

（2026-10-03 `dfaf418` + `704ee0d`；`_check_homed` 成功分支与整条 `cmd_calibrate` 端到端单测不可达——测试机恒 unhomed，按 `TESTING.md` 的真机约定待真机验证）

### M104 / M109 — 设置挤出机温度

```
M104 [S<temperature>] [T<index>]
M109 [S<temperature>] [T<index>]
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `S` | `0` | 目标温度（摄氏度） |
| `T` | `0` | 挤出机编号：`0` 是 `extruder`，`n` 是 `extruder<n>` |

只有名字是 `extruder` 的主挤出机注册这两条（与上游一致）。**`M109` 等待升温**：
`wait=true` 经 `PrinterHeaters::set_temperature(.., wait)` 阻塞到加热器到达目标
（2026-10-03，`6900894`，匹配上游 `kinematics/extruder.py:260-279`）；`M104` 仅设目标即返回。编号指向的挤出机不存在时报
`Extruder not configured`。

### M140 / M190 — 设置热床温度

```
M140 [S<temperature>]
M190 [S<temperature>]
```

`S` 默认 `0`。**`M190` 等待升温**：`wait=true` 经 `PrinterHeaters::set_temperature(.., wait)`
阻塞到热床到达目标（2026-10-03，`6745e83`，匹配上游 `heater_bed.py:17-28`）；`M140` 仅设目标即返回。

### PID_CALIBRATE — PID 自整定

```
PID_CALIBRATE HEATER=<name> TARGET=<celsius> [WRITE_FILE=<0|1>]
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `HEATER` | —（必填） | 加热器短名（同 `SET_HEATER_TEMPERATURE` 的多路键） |
| `TARGET` | —（必填） | 目标温度（摄氏度） |
| `WRITE_FILE` | `0` | 为 `1` 时把采样过程写入 `/tmp/heattest.txt` |

- 命令由**首台加热器装载**时注册（上游 `Heater.__init__` 的 `load_object` 对应物）——
  配置里没有加热器就没有这条命令，也没有 `[pid_calibrate]` 节（手写会被判未知节，与上游差异见模块文档）；
- 校准跑 `ControlAutoTune`：满功率加热/降功率冷却双相摆动（目标周期性下摆 `TUNE_PID_DELTA=5`），
  记满 12 个峰后用 Åström–Hägglund 估出终极周期/增益、按 Ziegler–Nichols 出参；
- 完成后回三行 `PID parameters: pid_Kp=… pid_Ki=… pid_Kd=…` 与
  `The SAVE_CONFIG command will update the printer config file with these parameters and restart the printer.`；
  结果经 `configfile.set` 暂存四条（`control: pid` + 三个 `pid_K*`），**执行 `SAVE_CONFIG` 才写回并重启**；
- 校准未跑满（如停机）时报 `pid_calibrate interrupted`，交还原控制环（错误路径同样交还）；
- 等待循环走 `set_temperature(.., wait)`，每秒回显一行真 `M105` 报表（`<id>:%.1f /%.1f`，
  空表回 `T:0`；2026-10-03 `14f5055` 接线）。

### M106 / M107 — 风扇（由 `[fan]` 装载）

```
M106 [S<0..255>]
M107
```

- `M106` 的 `S` 默认 `255`、下限 `0`、**没有上限**（真正封顶的是 `max_power`），内部除以 255
  得到占空比；
- `M107` 等价于 `M106 S0`。
- 速度变更**按打印时间生效**（有 `[printer]` 即 toolhead 在时）：经 `GCodeRequestQueue` 钉到前瞻时刻，flush 时落 PWM，kick-start 尾巴作为队列请求重跑；无 toolhead 或资源无法定时则立即生效（两条兑底，兑底时 kick 才用 `call_later`）。

---

## 挤出机

### SET_PRESSURE_ADVANCE — 设置压力推进（多路键 `EXTRUDER`）

```
SET_PRESSURE_ADVANCE [EXTRUDER=<name>] [ADVANCE=<mm>] [SMOOTH_TIME=<s>]
```

两个参数都缺省为该挤出机的当前值；`EXTRUDER=<name>` 亦可指向 `[extruder_stepper <名>]` 段注册的同名步进（按名挂值，批 #2）。回显两行：

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

### G10 / G11 / SET_RETRACTION / GET_RETRACTION — 固件回抽（由 `[firmware_retraction]` 注册，批 #32）
```
G10                 # 回抽（写 gcode 状态：G91 + G1 E-<retract_length> F<retract_speed>*60）
G11                 # 恢复（含 unretract_extra_length）
SET_RETRACTION [RETRACT_LENGTH=] [RETRACT_SPEED=] [UNRETRACT_EXTRA_LENGTH=] [UNRETRACT_SPEED=]
GET_RETRACTION      # 回读四个参数（%.5f）
```
重复 `G10` 为 no-op；`retract_length=0` 时 `G10` 不产生移动。

### M118 / RESPOND — 主机回显（由 `[respond]` 注册，就绪前可用，批 #29）
```
M118 <原文>
RESPOND [TYPE=echo|command|error|echo_no_space] [PREFIX=<前缀>] [MSG=<文本>]
```
`M118` 直接回 `<默认前缀> <原文>`；`RESPOND` 的 `MSG` = 缺省为空串，`PREFIX` 覆盖 `[respond]` 的默认前缀；非法 `TYPE` 报 `RESPOND TYPE '<t>' is invalid. Must be one of 'echo', 'command', or 'error'`。

### SET_IDLE_TIMEOUT — 设置空闲超时（由 `[idle_timeout]` 注册，批 #21）
```
SET_IDLE_TIMEOUT [TIMEOUT=<秒>]
```
不带参数时保持当前值；成功回 `idle_timeout: Timeout set to <值> s`；`TIMEOUT` 必须大于 0。

### AXIS_TWIST_COMPENSATION_CALIBRATE — 校准轴扭曲补偿（由 `[axis_twist_compensation]` 注册，批 #36）
```
AXIS_TWIST_COMPENSATION_CALIBRATE [SAMPLE_COUNT=<n>]
```
逐点探测并记录各采样点的 Z 补偿；help 与回复文案照上游。

### SET_DIGIPOT — 设置数字电位器（多路键 `DIGIPOT`，批 #33）
```
SET_DIGIPOT DIGIPOT=<name> [WIPER=<0..scale>]
```
每个 `[mcp4018 <name>]` 注册一个 `DIGIPOT` 值；不给 `WIPER` 时不写也不报错；给值后回 `New value for DIGIPOT = <name>, wiper = <%.2f>`。

### SET_FAN_SPEED — 设置通用风扇速度（多路键 `FAN`，批 #18）
```
SET_FAN_SPEED FAN=<name> SPEED=<0..1>
```
每个 `[fan_generic <name>]` 注册一个 `FAN` 值。`SPEED` 与 `TEMPLATE` 必须恰给一个，否则报 `SET_FAN_SPEED must specify SPEED or TEMPLATE`；`TEMPLATE=` 形式未实现（渲染引擎已在 `extras/template.rs`，这条缝未接）。

### SET_PIN — 设置引脚值（多路键 `PIN`）

```
SET_PIN PIN=<name> VALUE=<0..scale>
```

每个 `[output_pin <name>]` 装载时注册一个 `PIN` 值，因此 `SET_PIN` 是**多路命令**。

| 参数 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `PIN` | 字符串 | 是 | 对应 `[output_pin <name>]` 里的 `<name>` |
| `VALUE` | `0.0 ~ scale` | 是 | 数字输出：`>= 0.5` 为高电平；PWM：除以 `scale` 后作为占空比 |
| `CYCLE_TIME` | 秒（>0） | 否 | 仅 `pwm_cycle_time` 节的引脚可用：改写该脚 PWM 周期（运行期只改主机记账，固件周期 build 期固定） |

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

### SET_TEMPERATURE_FAN_TARGET — 设定温度风扇目标（由 `[temperature_fan]` 注册，mux 键 `TEMPERATURE_FAN=`）

| 命令 | 参数 | 说明 |
|------|------|------|
| `SET_TEMPERATURE_FAN_TARGET` | `TEMPERATURE_FAN`（风扇名）、`TARGET`（目标温度，须在 `min_temp..=max_temp`） | 重设 `[temperature_fan <名>]` 的目标；缺省打印当前目标；错误文案对上游（`temperature_fan.py`） |

### SET_INPUT_SHAPER — 输入整形参数（由 `[input_shaper]` 注册，wave-2）

| 命令 | 参数 | 说明 |
|------|------|------|
| `SET_INPUT_SHAPER` | `SHAPER_TYPE_X/Y/Z`、`SHAPER_FREQ_X/Y/Z`、`DAMPING_RATIO_X/Y/Z`（均可选） | 缺省参数则报告当前值，如 `shaper_type_x:mzv(5,0.6) shaper_freq_x:22.200 damping_ratio_x:0.100000`（顺序 x→y→z）；错误文案对上游：`Unsupported shaper type: %s`、`Too high value of damping_ratio=%.3f for shaper %s on axis %c`。参数只改上报值（系数未接到步进生成，见 `[input_shaper]` 的 gap） |

### SET_EXTRUDER_ROTATION_DISTANCE / SYNC_EXTRUDER_MOTION — 挤出机（mux 键 `EXTRUDER`，wave-2）

| 命令 | 参数 | 说明 |
|------|------|------|
| `SET_EXTRUDER_ROTATION_DISTANCE` | `EXTRUDER`（名）、`DISTANCE`（可省） | 缺省 `DISTANCE` 则报告当前值；`0` 报 `Rotation distance can not be zero`；负值翻转方向 |
| `SYNC_EXTRUDER_MOTION` | `EXTRUDER`、`MOTION_QUEUE`（可空） | 空值解绑；非挤出机名报 `'%s' is not a valid extruder.` |

### QUERY_FILAMENT_SENSOR / SET_FILAMENT_SENSOR — 断料传感器（由 `[filament_*]` 注册，mux 键 `SENSOR`，wave-2）

| 命令 | 参数 | 说明 |
|------|------|------|
| `QUERY_FILAMENT_SENSOR` | `SENSOR`（传感器名） | 报告断料状态 |
| `SET_FILAMENT_SENSOR` | `SENSOR`、`ENABLE` | 开关传感器 |

### MANUAL_STEPPER — 手动驱动单个步进电机（由 `[manual_stepper <名>]` 注册，mux 键 `STEPPER`，批 #6）

| 参数 | 缺省 | 说明 |
|------|------|------|
| `ENABLE` | — | 0/1 |
| `SET_POSITION` | — | 设定当前位置 |
| `SPEED` | 节的 `velocity` | 本次速度 |
| `ACCEL` | 节的 `accel` | 本次加速度 |
| `MOVE` | — | 相对移动量；带 `MOVE` 时 `SYNC` 缺省 1 |
| `SYNC` | 0 | 是否等到运动结束 |
| `STOP_ON_ENDSTOP` | — | `1`/`-1`/`2`/`-2`（home/inverted/try 变体） |
| `GCODE_AXIS` | — | 单字母（**先转大写再校验**：小写可接受；`X/Y/Z/E/F/N`、多字符与数字被拒）；空值注销 |
| `INSTANTANEOUS_CORNER_VELOCITY`/`LIMIT_VELOCITY`/`LIMIT_ACCEL` | 1.0 / 999999.9 / 999999.9 | 与 `GCODE_AXIS` 同用 |

错误文案对上游：`Move out of range`、`Not a valid GCODE_AXIS`、`Must unregister axis first`、`Axis 'A' already registered`、`No endstop for this manual stepper`。

### G28 与 `[safe_z_home]`（批 #6）

有 `[safe_z_home]` 时 `G28` 被接管：按需先执行 `G28 X0 Y0` → 抬到安全位 → `G28 Z0`（`move_to_previous` 为真时最后回原位）；`G28 Z` 而 X/Y 未归零报 `Must home X and Y axes first`。

### PAUSE / RESUME / CLEAR_PAUSE / CANCEL_PRINT — 暂停与恢复（由 `[pause_resume]` 注册，批 #15）

`PAUSE` 置暂停并跑 `SAVE_GCODE_STATE NAME=PAUSE_STATE`；`RESUME [VELOCITY=]`（默认取 `recover_velocity`）跑 `RESTORE_GCODE_STATE NAME=PAUSE_STATE MOVE=1 MOVE_SPEED=<v>`；`CLEAR_PAUSE` 只清状态；`CANCEL_PRINT` 取消打印（无 `virtual_sdcard` 时回 `action:cancel`）。重复 `PAUSE` 回 `Print already paused`，未暂停时 `RESUME` 回 `Print is not paused, resume aborted`。

### LOAD_CELL_CALIBRATE / LOAD_CELL_TARE / LOAD_CELL_READ / LOAD_CELL_DIAGNOSTIC — 称重校准（由 `[load_cell]` 注册，批 #14）

四条命令**已注册并带上游 help，当前调用报 `not implemented`**（需 `LoadCellSampleCollector` 与交互式校准，见 TODO）。

### M117 / M73 / SET_DISPLAY_TEXT — 显示消息与进度（由 `[display_status]` 注册，批 #7）

| 命令 | 参数 | 说明 |
|------|------|------|
| `M73` | `P`（0–100）、`R` | 进度百分比 + 剩余时间 |
| `M117` | 整行原文 | 设置显示消息（无参数则清空） |
| `SET_DISPLAY_TEXT` | `MSG`（可选） | help = `Set or clear the display message` |

### SET_DISPLAY_GROUP — 切换显示组（由 `[display]` 注册，批 #7）

`GROUP=<名>`（主节另支持无参形式）；help = `Set the active display group`；未知组报 `Unknown display_data group '%s'`。

### INIT_TMC / DUMP_TMC / SET_TMC_FIELD / SET_TMC_CURRENT — TMC 驱动（mux 键 `STEPPER`，批 #7）

| 命令 | 参数 | 说明 |
|------|------|------|
| `INIT_TMC` | — | 重写驱动器寄存器 |
| `DUMP_TMC` | `REGISTER`（可选） | 打印寄存器；未知名报 `Unknown register name '%s'` |
| `SET_TMC_FIELD` | `FIELD`、`VALUE` 或 `VELOCITY` | 未知字段报 `Unknown field name '%s'` |
| `SET_TMC_CURRENT` | `CURRENT`、`HOLDCURRENT` | 应答 `Run Current: %.2fA`（带保持电流时两段） |

### G-Code 宏命令（由 `[gcode_macro <名>]` 注册）

每个 `[gcode_macro <名>]` 段在装载时以**大写宏名**注册为一条命令（help = `description`）。 宏命令的**参数名**也随注册进入 `status.gcode.commands[<大写宏名>]["parameters"]`：装载时扫模板体里的 `params.NAME` / `params['NAME']` / `params["NAME"]`（去重、首次出现序），扫不到则退回该节的 `variable_*` 名，两者都无则不带该键——客户端补全 `KEY=` 左边时用这份（**内建兜底表不含配置定义的宏**，宏只能运行时知道）。**宏体已渲染执行**：渲染引擎是 `extras/template.rs` 的 **minijinja 2.24 适配层**（单花括号定界符、缺键即报错、装载期编译），渲染后回 gcode 派发；`SET_GCODE_VARIABLE` 已注册；`{% set %}` 作用域对齐 Jinja2；`namespace()`、关键字实参、`{% block %}`、`|float(默认)` 等构型均可解析（含 `template.rs` 步进宏字面用例的实测）。与 Jinja2 的已知差异与错误 detail 措辞见 `extras/template.rs` 模块文档。

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

### DELTA_CALIBRATE / DELTA_ANALYZE — delta 校准（由 `[delta_calibrate]` 注册，批 #5）

| 命令 | 参数 | 说明 |
|------|------|------|
| `DELTA_CALIBRATE` | 对上游（手动探测循环驱动，复用 `manual_probe`） | 多点测量 + coordinate descent 拟合 tower 几何（arm_length/delta_radius/angle_trim 等），结果以 SAVE_CONFIG 待写行交回写侧 |
| `DELTA_ANALYZE` | 对上游 | 用已有 `height0..` 数据重算；缺 basic 校验报上游原文 `Must run basic calibration with DELTA_CALIBRATE first` |

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

### SCREWS_TILT_CALCULATE — 螺丝调平计算（由 `[screws_tilt_adjust]` 注册）

| 命令 | 参数 | 说明 |
|------|------|------|
| `SCREWS_TILT_CALCULATE` | `MAX_DEVIATION`（可选，毫米）、`DIRECTION`（可选，`CW` / `CCW`） | **须先配置 `[screws_tilt_adjust]`**（节内选项见 config.md）。依次探测各螺丝上方床面，逐颗输出拧账单：`HH:MM` 是时钟记法——整数位 = 整圈，分 = 60 进制小数圈（**一整圈 = 60 分钟 = 螺距**，由 `screw_thread` 表换算；例 `01:20` 即 1 又 1/3 圈）。基准螺丝默认取第 1 颗；`DIRECTION=CW/CCW` 时按螺纹方向取 Z 极值（最高或最低）螺丝作基准。`MAX_DEVIATION` 是本轮的**延迟错误**：探测全部完成后才比较报错；`DIRECTION` 取值非法报 `DIRECTION must be either CW or CCW`。help：*Tool to help adjust bed leveling screws by calculating the number of turns to level it.* |

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
| `LDC_CALIBRATE_DRIVE_CURRENT` | `CHIP`（传感器名） | 对目标 LDC1612 做驱动电流标定，回显 `reg_drive_current` 提取值并给出 `SAVE_CONFIG` 提示（细节见 `extras/ldc1612.rs`）；`ldc1612` 对象由 probe_eddy_current 构造（接线随 M5d 落地生效），该命令随对象装载注册 |

## 变量（`save_variables`）

### SAVE_VARIABLE — 保存一个变量

```
SAVE_VARIABLE VARIABLE=<name> VALUE=<literal>
```

| 参数 | 说明 |
|------|------|
| `VARIABLE` | 变量名，**必须全小写**（含大写报 `VARIABLE must not contain upper case`） |
| `VALUE` | Python 字面量：`None` / `True` / `False` / 整数 / 浮点 / 字符串 / 列表 / 字典（字符串键） |

写进 [`[save_variables]`](config.md) 的 `filename`（合并已有变量后重写整个 `[Variables]` 段），
随后重新加载；宏里用 `printer.save_variables.variables.<name>` 读取。
`VALUE` 解析失败报 `Unable to parse '<v>' as a literal`。

## 虚拟 SD 卡与打印统计（`[virtual_sdcard]` / `print_stats`）

`[virtual_sdcard]` 装载后注册主机侧文件打印命令族（上游 `klippy/extras/virtual_sdcard.py`）；`print_stats` 对象随 `virtual_sdcard` 懒装载，注册 `SET_PRINT_STATS_INFO`。

| 命令 | 参数 | 说明 |
|------|------|------|
| `M20` | — | 列出 SD 目录：`Begin file list` / 每行 `<名> <字节>` / `End file list`（顶层，不滤扩展名） |
| `M21` | — | 回显 `SD card ok` |
| `M23` | `<filename>`（裸参数） | 选中 SD 文件（去前导 `/`，大小写不敏感匹配）；回显 `File opened:<名> Size:<字节>` / `File selected`；回放中报 `SD busy` |
| `M24` | — | 开始/恢复 SD 打印：`do_resume` 注册 reactor 定时器 → spawn 回放 task，逐行 `gcode.run_script` 回放；已在回放时报 `SD busy` |
| `M25` | — | 暂停 SD 打印：置 `must_pause_work`，回放 task 下一次迭代退出并 `note_pause` |
| `M26` | `S`（int，minval 0） | 设文件位置 `file_position`（回放中报 `SD busy`） |
| `M27` | — | 报 SD 打印状态：无文件时 `Not SD printing.`，否则 `SD printing byte <pos>/<size>` |
| `M28` / `M29` / `M30` | — | 上游 SD 写命令，本仓报 `SD write not supported` |
| `SDCARD_RESET_FILE` | — | 清除已装载的 SD 文件（必要时暂停并关闭）；从 SD 回放中执行时报错 |
| `SDCARD_PRINT_FILE` | `FILENAME` | 重置后装载 SD 文件（去前导 `/`，含子目录递归查找）并立即 `do_resume` 开始打印；回放中报 `SD busy` |
| `SET_PRINT_STATS_INFO` | `TOTAL_LAYER`（int，minval 0）/ `CURRENT_LAYER`（int，minval 0） | 传递切片层信息：`TOTAL_LAYER=0` 清空两值；切换 `TOTAL_LAYER` 重置 `CURRENT_LAYER=0`；`CURRENT_LAYER` 截断到 `TOTAL_LAYER` |

**进度与状态**：`virtual_sdcard` 的 `get_status` 报 `file_path`/`progress`/`is_active`/`file_position`/`file_size`；`print_stats` 的 `get_status` 报 `filename`/`total_duration`/`print_duration`/`filament_used`/`state`/`message`/`info`。回放结束按结果 `note_complete`（EOF）/`note_pause`（被暂停）/`note_error`（错误，并渲染运行 `on_error_gcode`）。

**未实现**：`gcode.get_mutex().test()` 让出（本仓无该 API，回放期间外部命令交错与上游不同）、`_handle_analyze_shutdown`/`_handle_debuginput_exit`、`stats`。`path` 不做 `expanduser`/`normpath`，按原样用于目录列举。

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

### PROBE / QUERY_PROBE / PROBE_ACCURACY — 探针命令（由 probe 对象注册）

| 命令 | 参数 | 说明 |
|------|------|------|
| `PROBE` | 采样组参数；`METHOD=`（`scan`/`rapid_scan`/`tap`，eddy 对象分派；其余对上游 `probe.py`） | 触发一次探测并记录位置 |
| `QUERY_PROBE` | 无 | 回显上次触发状态 |
| `PROBE_ACCURACY` | 采样组（对上游） | 采样精度统计 |

### PROBE_EDDY_CURRENT_TAP_CALIBRATE — eddy tap 标定（由 `[probe_eddy_current]` 注册，批 #3）

| 命令 | 参数 | 说明 |
|------|------|------|
| `PROBE_EDDY_CURRENT_TAP_CALIBRATE` | `TAP=` 子模式（信息分支/试打/拒绝文案对上游 `probe_eddy_current.py`） | eddy tap 标定流程；**静态标定 `CALIBRATE=enable` 与 `Z_OFFSET_APPLY_PROBE` 已实现**（2026-10-03，`4caf425`） |
| `PROBE_EDDY_CURRENT_CALIBRATE` | `CHIP=`（mux，段名）+ `PROBE_SPEED`（默认 5，above 0） | 手动探针 + 校准移动采样生成 `calibrate` 表（`EddyCalibrationTool`，2026-10-03） |
| `Z_OFFSET_APPLY_PROBE` | `METHOD=`（`tap` 走 tap_z_offset 写回，否则偏移校准表） | 把回零偏移应用到校准数据（`EddyCalibrationTool`，2026-10-03） |

### SET_SERVO — 舵机控制（由 `[servo <名>]` 注册，mux 键 `SERVO=`）

| 命令 | 参数 | 说明 |
|------|------|------|
| `SET_SERVO` | `SERVO`（名）、`ANGLE=<0..maximum_servo_angle>` 或 `WIDTH=<秒>` | 按角/脉宽驱动舵机；缺参与越界文案对上游（`servo.py`） |

### SET_DUAL_CARRIAGE / SAVE_DUAL_CARRIAGE_STATE / RESTORE_DUAL_CARRIAGE_STATE — IDEX 双滑架（由 `[dual_carriage]` 注册；`kinematics: generic_cartesian` 时由运动学在 `[printer]` 装载时注册，批 #12）

| 命令 | 参数 | 说明 |
|------|------|------|
| `SET_DUAL_CARRIAGE` | `CARRIAGE=<名字或 0..1>`、`MODE=...` | 切换主/副滑架（**先按名字**如 `carriage_u`；`0/1` 仅在恰 2 滑架时回退，多滑架报 `Invalid CARRIAGE=…`，批 #12）；**现状=记账级**（未接 trapq 切换与限位，C1 缺口入档 idex 模块） |
| `SAVE_DUAL_CARRIAGE_STATE` | `NAME=<名>` | 保存当前滑架状态 |
| `RESTORE_DUAL_CARRIAGE_STATE` | `NAME=<名>`、`MOVE=<0..1>` | 恢复（含恢复移动的完整语义未移植） |

错误措辞对上游 `idex_modes.py:240/283/295`；`T0`/`T1` 宏驱动的切换用例已随宏体渲染引擎转绿。

### EXCLUDE_OBJECT 族 — 打印对象排除（由 `[exclude_object]` 注册）

| 命令 | 作用（上游 `exclude_object.py`） |
|------|------|
| `EXCLUDE_OBJECT_START` | 标记当前对象开始（:190-198） |
| `EXCLUDE_OBJECT_END` | 清除当前对象（:200-213） |
| `EXCLUDE_OBJECT` | 按名/当前排除、RESET 或列表（:215-238） |
| `EXCLUDE_OBJECT_DEFINE` | 定义对象（CENTER/POLYGON）或重置文件（:240-267） |

参数与错误文案对上游；`[gcode_macro M486]` 的宏体驱动用例已转绿（2026-09-24，含排除区 E 补偿）。

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

> 越界文案里 `above` / `below` 是「必须严格大于 / 小于」，`minimum` / `maximum` 是「不得小于 /
> 大于」（同上游 `gcode.py:79-89`）。限制值的数字写法与上游不同（本仓 `0`，上游 `0.0`），
> 只影响显示、不影响判定，原因见[开发手册](../developer-manual/README.md) 的「越界报错里限制值的写法」
> 一节；配置项的同类报错是同一处差异。

| 来源 | 文案 |
|------|------|
| 缺参数 | `Error on '<整行>': missing <参数名>` |
| 解析失败 | `Error on '<整行>': unable to parse <原值>`；移动词另有一套：`Unable to parse move '<整行>'`，`F<=0` 是 `Invalid speed in '<整行>'` |
| 越界 | `Error on '<整行>': <参数名> must have minimum of <限制>`（`maximum` 同形）与 `Error on '<整行>': <参数名> must be above <限制>`（`below` 同形） |
| 多路键值未注册 | `The value '<值>' is not valid for <键>. Options: 'a', 'b'`（候选里有包含关系时改成 `. Did you mean 'a'?`，**取排序后的第一个**，与上游取字典序最后一个不同——为了消息稳定） |
| 处理器自己的错误 | 各命令自带的文案（如 `Printer is not ready`、`Extruder not configured`） |

命令错误一律**不停机**：它回报客户端并发 `gcode:command_error` 事件，然后按输入源分两种
走向——需要 `ok` 应答的输入源（文件输入）会被 ack 并**继续执行下一行**，不需要应答的
（API 提交的脚本）则**中止整段脚本**，把错误交回给调用方。

会让打印机停机的只有 `M112` 这类显式命令、固件自报的停机，以及处理器 **panic**
（panic 走 `Internal error on command:"<名字>"` 并 `invoke_shutdown`，两者都与命令错误分开）。

---

- [← 用户手册首页](README.md)
