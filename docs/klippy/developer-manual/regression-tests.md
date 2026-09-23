# 回归测试（`.test` 与数据字典）

上游把主机侧（klippy）的回归测试集中在一个数据驱动的运行器里：`scripts/test_klippy.py`
读取 `test/klippy/*.test`，为每个用例启动一次 `klippy.py`，以进程退出码判定结果。它不依赖
真实硬件，因为 `klippy.py` 支持一种「文件输出 + 数据字典」模式：该模式下固件不存在，数据
字典由命令行直接注入。

本页描述该模式的运行机制、语料结构，以及本仓库复用语料的方式。字典在正常引导中的来路见
[Identify 机制](identify.md)；此处描述的是它的旁路。

## 用例格式

`.test` 是行式指令文件（`scripts/test_klippy.py:32`）：

| 指令 | 含义 |
|------|------|
| `CONFIG <path>` | 用例使用的配置文件，相对 `.test` 文件解析 |
| `DICTIONARY <file> [<mcu>=<file> …]` | 固件数据字典；首项为主 MCU，其余为次级 MCU |
| `GCODE <path>` | 以文件提供 g-code，与内联 g-code 二选一 |
| `SHOULD_FAIL` | 反转**运行期**期望：期望 `klippy.py` 在跑 g-code 时报错（配置装载失败不算，见「忽略列表」） |
| 其他非空行 | 内联 g-code，按出现顺序执行 |

`#` 起始注释。一份 `.test` 定义的是一**串运行**：每个 `CONFIG` 块是一次 `klippy.py` 调用，在
下一个 `CONFIG`（或文件末尾）处发射——所以 `CONFIG` 写在 `DICTIONARY` 之前也成立。`DICTIONARY`
行整体替换当前字典集合并持续到下一行，因此它把随后的运行按 MCU 目标分组；内联 g-code、
`GCODE` 文件与 `SHOULD_FAIL` 则跨运行共享、只增不减。全语料 37 份文件共 239 次运行，
其中 `printers.test` 一份就占 203 次。

## 数据字典的来源

`test/configs/*.config` 是 kconfig 片段，每份对应一类 MCU 目标。CI（`scripts/ci-build.sh`）逐份
`make` 出 `out/klipper.dict`，按目标名收集为 `<name>.dict`；`.test` 的 `DICTIONARY` 引用的正是
这些构建产物，因此它们不在源码树内。

字典是固件的自我描述：命令、响应、输出消息、枚举与常量（`klippy/msgproto.py:415`），内容为
JSON。正常引导中，该 JSON 由固件在 identify 阶段以 zlib 压缩后分块下发；文件输出模式跳过该
交换，把文件本身作为未压缩字典交给消息解析器（`process_identify(..., decompress=False)`）。

## 运行机制

`klippy.py` 的调用形状为：

```
klippy.py <config> -i <gcode> -o <output> -d <dict> [-d <mcu>=<dict> …]
```

`-o` 的取值决定模式。记

```
伪代码：文件输出模式判定

is_fileoutput() := start_args 含 debugoutput        # klippy/mcu.py:1169
```

`MCUConnectHelper._mcu_identify` 据此选择挂载路径（`klippy/mcu.py:872`）：

- `is_fileoutput()` 为真 → `_attach_file()`（`klippy/mcu.py:841`）；
- 否则 → `_attach()`，即真实串口 / CAN 路径。

`_attach_file()` 打开 `debugoutput` 作为输出目标，读取 `.dict`，调用
`serialhdl.connect_file()`（`klippy/serialhdl.py:207`）：

```
伪代码：连接文件输出目标

connect_file(输出文件, 字典):
    串口设备 = 输出文件
    消息解析器.装入字典(字典, 解压 = 否)     # 字典不压缩，直接是 JSON
    串口队列.分配(输出文件.描述符, 模式 = 文件, 名称)
```

其要点：

- 字典在此时已安装完毕，不发生 identify 交换；
- 传输对象仅持有输出文件的 fd，**不创建读取线程**（`background_thread` 保持 `None`），
  因此不会有任何响应到达；
- `clocksync.connect_file()` 登记一个不与固件同步的时钟。

据此，`is_fileoutput()` 在所有需要等待固件响应处短路，使「无响应」不构成错误：

| 位置 | 行为 |
|------|------|
| `MCU.__init__`（`mcu.py:1161`） | `estimated_print_time` 替换为恒返回 0 的实现 |
| `MCUConfigHelper._send_get_config`（`mcu.py:1037`） | 不发送 `get_config`，直接返回 `{is_config: 0, move_count: 500, crc: 0}` |
| `MCU_trsync.stop` / `wait_end`、`MCU_endstop.home_wait` / `query_endstop`（`mcu.py:271`、`325`、`396`、`403`） | 归位与探测判定直接返回，`wait_end` 立即完成 |
| `MCU.check_timeout`（`mcu.py:898`） | 不进入超时判定，不触发「Lost communication」停机 |
| `MCUConnectHelper._analyze_shutdown`（`mcu.py:884`） | 不分析停机原因 |
| `MCUStatsHelper._ready`（`mcu.py:951`） | 不校验固件时钟频率 |
| `MCUConfigHelper._connect` 收尾（`mcu.py:1073`） | 跳过「配置未生效」检查 |

随后 `klippy.py` 以 `-i` 指向的文件作为输入，命令经已安装的字典编码后写入 `-o` 指向的文件。
`test_klippy.py` 只统计退出码：非零为失败；`SHOULD_FAIL` 时相反。

### 覆盖面

该模式验证的是**主机侧**的完整执行路径：配置装载、对象构造、kinematics 与 extras、g-code 分派、
命令编码。它不验证固件行为，也不比对 `-o` 的内容——没有 golden 输出比较，退出码是唯一判据。

## 语料结构

上游语料位于 `third_party/klipper` 下：

| 路径 | 内容 |
|------|------|
| `test/klippy/*.test` | 用例 |
| `test/klippy/*.cfg` | 用例专用配置 |
| `test/klippy/move.gcode`、`test/klippy/sdcard_loop/` | 用例数据文件 |
| `test/configs/*.config` | 数据字典的 kconfig 片段 |
| `klippy/klippy.py --import-test` | 导入全部 `extras/` 与 `kinematics/` 模块的检查 |
| `scripts/check_whitespace.sh`、`scripts/check-software-div.sh` | CI 卫生检查 |

## 语料总览与架构依赖

| 项 | 数量 |
|----|------|
| `.test` 文件 | 37 |
| 运行（`CONFIG` 块） | 239（`printers.test` 占 203） |
| `test/configs/*.config` 目标 | 40 |
| 被 `.test` 引用的目标 | 28 |

按 `CONFIG_MACH_<FAMILY>` 聚合（`目标` 为该架构的 kconfig 片段数，`引用` 为声明该架构字典
的 `.test` 数）：

| 架构 | 目标 | 引用 | 说明 |
|------|------|------|------|
| `avr` | 7 | 34 | 默认字典（几乎所有功能用例） |
| `stm32` | 17 | 3 | `generic_cartesian_iqex/itex.test`、`printers.test` |
| `linux` | 1 | 2 | `linuxtest.test`、`printers.test`；**本地可编，默认启用** |
| `atsam` | 5 | 1 | `printers.test` |
| `atsamd` | 2 | 1 | `printers.test` |
| `hc32f460` | 2 | 1 | `printers.test` |
| `lpc176x` | 1 | 1 | `printers.test` |
| `pru` | 1 | 1 | `printers.test` |
| `rpxxxx` | 2 | 1 | `printers.test` |
| `ar100` | 1 | 0 | 只为编译覆盖 |
| `simu` | 1 | 0 | 只为编译覆盖 |

逐文件的运行数与架构依赖：

| `.test` | 运行 | 架构 |
|---------|------|------|
| `bed_mesh.test` | 1 | `avr` |
| `bed_screws.test` | 1 | `avr` |
| `bltouch.test` | 1 | `avr` |
| `commands.test` | 1 | `avr` |
| `corexyuv.test` | 1 | `avr` |
| `delta.test` | 1 | `avr` |
| `delta_calibrate.test` | 1 | `avr` |
| `dual_carriage.test` | 1 | `avr` |
| `eddy.test` | 1 | `avr` |
| `exclude_object.test` | 1 | `avr` |
| `extruders.test` | 1 | `avr` |
| `gcode_arcs.test` | 1 | `avr` |
| `generic_cartesian.test` | 1 | `avr` |
| `generic_cartesian_iqex.test` | 1 | `stm32` |
| `generic_cartesian_itex.test` | 1 | `stm32` |
| `hybrid_corexy_dual_carriage.test` | 1 | `avr` |
| `input_shaper.test` | 1 | `avr` |
| `led.test` | 1 | `avr` |
| `linuxtest.test` | 1 | `linux` |
| `load_cell.test` | 1 | `avr` |
| `macros.test` | 1 | `avr` |
| `manual_stepper.test` | 1 | `avr` |
| `multi_z.test` | 1 | `avr` |
| `out_of_bounds.test` | 1 | `avr` |
| `polar.test` | 1 | `avr` |
| `pressure_advance.test` | 1 | `avr` |
| `printers.test` | 203 | `atsam`、`atsamd`、`avr`、`hc32f460`、`linux`、`lpc176x`、`pru`、`rpxxxx`、`stm32` |
| `pwm.test` | 1 | `avr` |
| `quad_gantry_level.test` | 1 | `avr` |
| `rotary_delta_calibrate.test` | 1 | `avr` |
| `screws_tilt_adjust.test` | 1 | `avr` |
| `sdcard_loop.test` | 1 | `avr` |
| `smart_effector.test` | 1 | `avr` |
| `temperature.test` | 1 | `avr` |
| `tmc.test` | 1 | `avr` |
| `z_tilt.test` | 1 | `avr` |
| `z_virtual_endstop.test` | 1 | `avr` |

默认列表（`linux` + `avr` + 各 ARM 家族）会构建 37 份字典：28 个被 `.test` 引用的目标里除 `pru` 外
全部具备（另有 9 份没被引用的目标也一并编出，作为编译覆盖）。引用 `pru` 的运行只有 2 条（`printers.test`
里 `DICTIONARY pru.dict host=linuxprocess.dict` 那一组），它们因字典未构建被跳过。

### 当前状态

按默认 `KLIPPERX_ARCHES`（`linux` + `avr` + 各 ARM 家族）与现有忽略列表，239 次运行的判定：

| 判定 | 次数 | 原因 |
|------|------|------|
| 因字典未构建跳过 | 2 | `printers.test` 中引用 `pru` 的两条运行（默认不编 `pru`） |
| 因忽略列表跳过 | 236 | 尚未落地的配置节/运动学（除 `linuxtest.test` 外全部文件） |
| 实际执行 | **1** | `linuxtest.test`（T1），**通过** |

- `linuxtest.test` 是第一个转绿的用例：它只需要 `kinematics: none`、`heaters` 的传感器注册表、
  `temperature_sensor` 与 `ds18b20`，g-code 只是一次 `G4 P1000`。
- `KLIPPERX_UPSTREAM_ALL=1` 只去掉忽略列表这一层：默认构建下它会跑 237 条可用运行，其中 1 条
  （`linuxtest`）通过、**236 条**在配置装载阶段失败，另外 2 条以「字典未构建」
  计入统计，不算失败。
- 按「首次失败原因」的分组（T3–T10 工单）、运动学细分与完整失败日志不在本手册重复维护，
  统一见 [上游回归测试失败原因分析](../../work-log/2026-09-22-upstream-regression-failures.md)
  （2026-09-22 快照，含 T7 完成后的状态与复盘）。
- 要让实际执行数继续上升：从 `IGNORED` 移除已落地节/运动学的文件。按 T3（`extruder`/`heater_bed`/`fan`）
  与 T4（`probe`/`bltouch`/endstop pin chip）推进。

### 推进口径与验收

- 上面按「**首次失败原因**」的分组只用于**定位**，不是工作队列：`load_config` 遇到第一个未知
  section 就停，修好一个缺口只会让运行前进到下一个缺口，总数可能不变（T7 后的 236 就是例子），
  各组收益不可加。
- 进度以**转绿运行数 / `IGNORED` 条目数**衡量（当前 1 / 36）；先产出「**运行 × 缺口**」矩阵
  （列出每条运行的**全部**缺口，而非第一个），据此找「只差一个缺口」的用例与公共前缀。
- **验收标准**：对应 `.test` 从 `IGNORED` 移除后通过。`ignored_cases_still_fail` 是守卫——
  某个忽略文件的全部可跑运行都通过时报失败并提示移除。
- 看全部缺口（而不只是首次失败）：
  `cargo test -p klipperx --lib upstream_gap_report -- --nocapture`。
- `out_of_bounds.test` 是唯一的 `SHOULD_FAIL`，必须等配置能装载后再移出，否则越界检查会被配置
  错误「喂饱」。

## 本仓库的复用

本仓库把上述语料作为**只读 fixture** 使用。harness 与用例位于
`src/core/klippy/upstream.rs`，以 `#[cfg(test)] mod upstream` 编入主机单元测试（lib 测试目标
`core::klippy::upstream::tests::*`），复用按能力分层推进：

| 阶段 | 依赖 | 状态 |
|------|------|------|
| 语料结构完整、引用可解析 | 无 | `upstream_test_cases_are_well_formed`、`upstream_test_inputs_resolve` |
| 每份 `.cfg` 由本仓库解析器读取 | 无 | `every_upstream_printer_config_parses` |
| 全部缺口扫描（**报告**，不失败） | 无 | `upstream_gap_report`（`-- --nocapture` 查看缺口矩阵） |
| `IGNORED` 条目守卫 | 运行声明的全部字典 + 所用配置节 | `ignored_cases_still_fail`（某个忽略文件全跑通了就报失败） |
| 运行内联 g-code 可解析 | 运行所用配置节 | `#[ignore] upstream_inline_gcode_parses` |
| 运行端到端执行 | 运行声明的全部字典 + 所用配置节 | `upstream_test_cases_run`（按运行的可用性过滤 + 忽略列表） |

### 应答机

端到端执行不靠上游那种「无固件」旁路，而是真的有人应答：
`interface/devices/simulator.rs` 的 `SimulatorDevice` 按一份 `.dict` 驱动，

- 用 `identify` 分块下发 zlib 压缩后的字典（最后一块为空，与真固件一致）；
- 在 `finalize_config crc=%u` 记下 CRC，之后把 `get_config` 报为已配置；
- 用单调计数器回答 `get_clock` / `get_uptime`；
- 对每个收到的块回一个同序号的空载荷 ack，推进主机的发送窗口。

harness 把每个 `[mcu]` / `[mcu <name>]` 的传输键换成 `test: dict=<字典路径>`，主机因此走它的
正常路径（identify、配置握手、时钟、消息序号），而不是一条生产不存在的分支。

### 架构闸与字典解析

激活的**架构列表**（`KLIPPERX_ARCHES`，逗号分隔）只在**构建阶段**生效：
`crates/test-support/build.rs` 据此过滤 `test/configs/*.config`，逐个 `make`，产出同名
`<name>.dict`；选定目标构建失败即报错（交叉工具链不在列表里的目标不会被选中）。

默认列表是「工具链好获得」的那一组：`linux`（主机编译器）、`avr`（`avr-gcc`）与**所有 ARM 家族**
（`arm-none-eabi-gcc`）：`stm32`、`atsam`、`atsamd`、`lpc176x`、`rpxxxx`、`hc32f460`。其余不在默认里：
`pru`（PRU 工具链）、`ar100`（or1k 工具链），而 `simu` 没有任何 `.test` 用。
`KLIPPERX_ALL_ARCHES=1` 忽略该列表，构建 `test/configs/` 下的**全部**目标（含 `pru`、`ar100`、`simu`，
需具备全部交叉工具链）。

架构由 `test/configs/<name>.config` 里的全大写 `CONFIG_MACH_<FAMILY>` 判定（`AVR`、`STM32`、
`LINUX`、`ATSAM`、`ATSAMD`、`RPXXXX`、`LPC176X`、`HC32F460`、`PRU`、`AR100`、`SIMU`；板型号
那个键带小写字母，不作为家族）。

运行阶段的启用条件：**一次运行声明的所有 `DICTIONARY` 都有已构建的字典**。缺一个就跳过这条
运行，不拿其他目标的字典顶替。这一条不受任何运行期变量影响，只能靠在构建阶段多启用架构来满足。

### 忽略列表

上游绝大多数配置会用到本主机尚未实现的节（`extruder`、`heater_bed`、`fan`、`gcode_macro`、
`tmc*`…），它们在 `load_config` 阶段就被拒绝，因此先登记在 `IGNORED` 里跳过；随节落地逐条移除。
列表按 `.test` 文件登记，作用域是该文件的**全部运行**。

`KLIPPERX_UPSTREAM_ALL=1` **只作用于这张列表**：它让字典齐备的运行无视忽略判定并报出失败，
**不会**让因字典未构建而跳过的运行跑起来（那是构建阶段的事，见上一节）。

当前 36 条（`linuxtest.test` 已在 T1 修好并转绿，不再是忽略项；T2 `stepper_enable` 与 T7 温度
传感器的相关文件仍因后续缺节留在列表里）。

失败原因的分组、运动学细分与 `KLIPPERX_UPSTREAM_ALL=1` 的完整失败日志，统一记在
[上游回归测试失败原因分析](../../work-log/2026-09-22-upstream-regression-failures.md)；
本手册只保留机制与推进口径，避免两处统计互相漂移（第一次失败修好后会露出下一个，分组随落地进度变化）。

`out_of_bounds.test` 是唯一声明 `SHOULD_FAIL` 的用例，它期望的是**运行期**错误（`G1 Y9999` 越界），
不是配置错误。上游可以把任何非零退出都当成功，是因为它什么都不缺；本仓库因此把结果分两段：

- `load_config` + `bring_up` 失败 = 「这台主机还跑不了这个用例」，报为**失败**，不算满足 `SHOULD_FAIL`；
- 只有 g-code 阶段的错误才反转成成功（`run_phases` 的两段返回值）。

所以把它留在忽略列表里是正确的：等 `example-cartesian.cfg` 所需的节/运动学落地、把它移出忽略列表后，
反转才会真的去验越界检查，而不是被一个配置装载错误“喂饱”。

### 运行

用例在 lib 测试目标里，用 `-p klipperx --lib` 加名字过滤运行：

```bash
cargo test -p klipperx --lib upstream
# 语料相关的全部用例；字典未构建或列入忽略列表的运行跳过，内联 g-code 阶段不执行

cargo test -p klipperx --lib every_upstream_printer_config_parses
# 单条：全部上游 .cfg 能否被本仓库解析

KLIPPERX_ARCHES=linux cargo test -p klipperx --lib upstream_test_cases_run
# 只编 linux 一份，构建最快；默认还会编 avr 与各 ARM 家族

KLIPPERX_ALL_ARCHES=1 cargo test -p klipperx --lib upstream_test_cases_run
# 构建 test/configs 下的全部目标（需要所有交叉工具链）

KLIPPERX_UPSTREAM_ALL=1 cargo test -p klipperx --lib upstream_test_cases_run
# 只去掉忽略列表：跑全部「字典齐备」的运行并列出失败
```

`--workspace` 与 `--lib` 的取舍、真机用例的约定见[测试](testing.md)。

`CONFIG` 与 `GCODE` 相对 `.test` 文件解析；`DICTIONARY` 是构建产物，因此只校验其对应的
`test/configs/<name>.config` 存在。内联 g-code 阶段仍以 `#[ignore]` 保留（需要同一批缺失的节），
补齐后移除属性即可。

`every_upstream_printer_config_parses` 覆盖 259 份 `.cfg`，现已全部通过。这条用例最初暴露了本
仓库解析器与上游 `configparser` 的四处分歧（多行值、`=` 分隔符、节头行内注释、`;` 行内注释），
它们已在 `src/core/klippy/config/mod.rs` 中修复，并各自有解析器单测。

---

- [← 开发手册首页](README.md)
- [测试 ←](testing.md)
