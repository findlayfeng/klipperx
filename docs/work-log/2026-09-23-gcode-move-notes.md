# G4 / H10（`gcode_move`）动工前调查（2026-09-23）

本文为 `TODO.md` 的 **G4/H10 `gcode_move`（g-code 坐标系）** 做的动工前调查。它是 H2-1 与
`is_fileoutput` 之后回归里最靠前的**运行期**门闩——注意 `gcode_move` 不是 config section
（上游由 `toolhead.py:610-613` 的 default modules 列表按名字加载），所以 `upstream_gap_report`
看不见它，它只出现在 `upstream_test_cases_run` 的首次失败里。

- 上游基线：`third_party/klipper/`（commit `02e71b9`），`klippy/extras/gcode_move.py`（**294 行**，
  2026-10-04 复核更正，原记 349 行）
- 本地基线：`cb50abf`
- 相关记录：[H2 动工前调查](2026-09-23-h2-notes.md)

> **本文是 2026-09-23 当时的快照，不是规范。** 正文 §1–§5 记的是那天的盘点与拍板建议，
> 其中多处已被实现推进取代；**余项与现状以 §6 为准**（`TODO.md` 是权威台账）。

---

## 0. 结论摘要

| 项 | 结论 |
|---|---|
| 为什么现在做 | 首位失败 **`Move out of range: 0.000 -2.000 1.000`（49 次运行）**：`move.gcode` 的 `G92 Y-3`/`G91` 是未知命令（只回提示、不报错），于是 `G1 Y-2` 按**绝对**坐标走到工具头 Y=-2，越过 `position_min: 0`。上游靠 g-code 坐标系偏移让它落在 Y=+1 |
| 这一层是什么 | 一份**g-code 坐标系**状态：绝对/相对（`G90`/`G91`、`M82`/`M83`）、`base_position` 偏移（`G92`/`SET_GCODE_OFFSET`）、速度与挤出倍率（`M220`/`M221`）、存取（`SAVE`/`RESTORE_GCODE_STATE`），`G0`/`G1` 在这份状态上算出**工具头坐标**再交给 `toolhead.move` |
| 最难的一块 | **`G0`/`G1` 要搬家**：现在在 `extras/toolhead.rs::move_command` 里，只有绝对坐标、没有偏移/相对/倍率；搬走后 `toolhead` 只剩「排队一次移动」的 API（上游 `toolhead.move`），四个直接构造 `move_command` 的单测跟着改 |
| 最容易漏的一块 | **`last_position` 必须跟着工具头走**：`G1` 只更新参数里出现的轴，没出现的轴沿用 `last_position` 交给 `toolhead`——回零后不重置就是一次跳变。上游靠 4 个事件（`toolhead:set_position`、`toolhead:manual_move`、`homing:home_rails_end`、`gcode:command_error`）重置，我们只有后两个在发，且 `homing:home_rails_end` **没有载荷**（上游拿 `homing_state.get_axes()` 决定给哪几轴重新锚定） |
| 最大的边界 | `Coord` 固定 4 轴（`mathutil`），上游 `last_position`/`axis_map` 会随 `extra_axes` 长；`move_transform`（`bed_mesh` 挂进来的）与 `GET_POSITION`（要 `kin.get_steppers()` + `calc_position` + MCU 位置）都不在这一层的最小面里 |
| 建议拆分 | **G4-1 坐标系核心**（状态 + `G0/G1/G20/G21/G90/G91/G92/M82/M83/M220/M221/SET_GCODE_OFFSET/SAVE/RESTORE/M114` + `get_status` + 重置链 + `G0`/`G1` 搬家）→ G4-2 外围（`GET_POSITION`、extra-axes `axis_map`、`toolhead:manual_move`/`update_extra_axes` 发送方、`move_transform`） |
| 需要拍板 | ① `G0`/`G1` 归属；② 加载点与注册顺序；③ `homing:home_rails_end` 加 `axes` 载荷；④ `toolhead:set_position` 由谁发；⑤ `move.gcode` 之外的默认速度 25 vs 我们现在的 50 |

---

## 1. 回归证据：为什么是现在

| 量 | 数 |
|---|---|
| 首位失败 `Move out of range: 0.000 -2.000 1.000` | **49 次运行**（`is_fileoutput` 之后的新首位）|
| 语料里的坐标系命令（inline + `*.gcode`） | `G1` 212、`G90` 15、`G91` 9、`M83` 3、`M114` 2、`G92` 2 |
| 不是它的 | `G2` 18（`gcode_arcs`）、`M486` 39（`exclude_object`）——未知命令只回提示，不判失败 |

`move.gcode` 的时间线（`test/klippy/move.gcode`）：

```
G28  G90  G1 F6000 … G4/M400 …      ← 回零与绝对模式，本来就通
M106/M107                           ← H2-1
G92 Y-3  G1 Y-2  G91  G1 Y-1        ← 未知：G92 无效 → G1 Y-2 按绝对坐标 → Y=-2 越界
```

上游的账：`G92 Y-3` 之前工具头最后一次 Y 来自 `G1 X0 Y0 E.01`，是 **0**（不是 `G1 Y1.5`），
`G92 Y-3` 让 `base_position[Y] = 0 - (-3) = 3`，于是 `G1 Y-2` → 工具头 Y = `-2 + 3` = **1**，
`G91` 之后的 `G1 Y-1` 再减 1 = 0。全程在范围内。

（2026-10-04 复核更正：原文按 `G1 Y1.5` 起算得 4.5 / 2.5 / 1.5，与同篇 §0 的「落在 Y=+1」自相矛盾。）

---

## 2. 证据：上游 `gcode_move.py`

### 2.1 状态（`__init__`，`gcode_move.py:21-37`）

| 字段 | 含义 |
|---|---|
| `absolute_coord` / `absolute_extrude` | `G90`/`G91`、`M82`/`M83`；**坐标与挤出分开** |
| `base_position[4]` | `G92`/`SET_GCODE_OFFSET` 的锚点：g-code 坐标 = `last_position - base_position` |
| `last_position[4]` | **工具头坐标**（不是 g-code 坐标），由事件重置跟随工具头 |
| `homing_position[4]` | `SET_GCODE_OFFSET` 的记忆值，回零后回填 `base_position` |
| `speed` / `speed_factor` | `F * speed_factor` = mm/s；`speed_factor` 初值 `1/60`，`M220` 改它 |
| `extrude_factor` | `M221`，作用在 E 的目标值与 `G92 E` 上 |
| `saved_states` | `SAVE`/`RESTORE_GCODE_STATE`（宏用） |
| `move_transform` / `move_with_transform` / `position_with_transform` | **ready 时**解析成 `toolhead.move` / `toolhead.get_position`；`bed_mesh` 用 `set_move_transform` 换掉 |

### 2.2 `G1` 算法（`gcode_move.py:117-141`）

```
对 axis_map 里每个出现的轴：E 先乘 extrude_factor
  相对（G91，或 E 且 M83）：last_position[axis] += v
  绝对：                       last_position[axis] = v + base_position[axis]
F → speed = F * speed_factor（F<=0 报错）
move_with_transform(last_position, speed)          ← 工具头坐标
```

### 2.3 事件（`gcode_move.py:44-56`）

| 事件 | 干什么 | 我们现在发吗 |
|---|---|---|
| `toolhead:set_position` | `reset_last_position` | **不发**（上游 `toolhead.py:390` 由 `ToolHead.set_position` 发）|
| `toolhead:manual_move` | 同上 | **不发**（我们没有 `manual_move` 这个 API）|
| `toolhead:update_extra_axes` | 重建 `axis_map` | **不发**（C1b 有 `extra_axes`，不发事件）|
| `homing:home_rails_end` | `reset` + 给**回零过的轴** `base_position = homing_position` | 发，但**每轴一次且无载荷**（拿不到 `get_axes()`）|
| `gcode:command_error` | `reset` | ✅ 发（`gcode.rs:955`）|
| `extruder:activate_extruder` | `reset` + 挤出倍率归位 | 声明了，**没有发送方**（`ACTIVATE_EXTRUDER` 还没做）|

### 2.4 命令全集

`G1`（`G0` 同一处理）、`G20`（报错）/`G21`、`M82`/`M83`、`G90`/`G91`、`G92`、`M220`、`M221`、
`SET_GCODE_OFFSET`、`SAVE_GCODE_STATE`/`RESTORE_GCODE_STATE`、`M114`、`GET_POSITION`，外加
`get_status`（`speed_factor`/`speed`/`extrude_factor`/`absolute_*`/`homing_origin`/`position`/
`gcode_position`/`axis_map`）。

加载点不是 section：`toolhead.py:610-613`
`modules = ["gcode_move", "homing", "idle_timeout", "statistics", "manual_probe", …]`，
在 `printer.add_object('toolhead', …)` **之后**逐个 `load_object`。

---

## 3. 本仓库现状与差距

| 上游部件 | 现状 | 要做的 |
|---|---|---|
| `G0`/`G1` 在 `gcode_move` | **在 `toolhead`**（`extras/toolhead.rs:394` + `move_command` :998）：只认绝对坐标，`F/60` 直传，无偏移/相对/倍率 | 搬家；`toolhead` 只留「排队一次移动」 |
| `toolhead.move` / `get_position` | `ToolHead::move_to(&mut self, …)` / `commanded_pos()` 在 `Connected` 里；`ToolHeadObject` 只有 `print_time()` 这种锁后访问的先例 | 加 `ToolHeadObject::{move_to, position}`（`Connected` 为 `None` 时「Printer is not ready」/ 不重置） |
| `last_position` 重置 | 事件声明齐（`event/decl/toolhead.rs`、`homing.rs`），**发送方缺 2 个**，`homing:home_rails_end` 无载荷 | 发 `toolhead:set_position`；给 `homing:home_rails_end` 加 `axes` |
| ready 时解析 toolhead | — | `klippy:ready` 事件里 `lookup_object("toolhead")`（**不能用 `connect()`**，见拍板 ②） |
| `Coord` | 固定 `[f64; 4]` | `axis_map` 的 extra 轴缓做（拍板 ⑤） |
| `move_transform` | — | 缓（`bed_mesh` 是另一个 gap），先只认 toolhead |
| `GET_POSITION` | 需要 `kin.get_steppers()` + `get_commanded_position` + `kin.calc_position` + `McuStepper` 的 MCU 位置 | 缓（G4-2）；未知命令不判失败 |
| `M114` | `gcmd.respond_raw` 已有 | 做（三行） |
| 默认 `speed = 25.` | toolhead 侧 `DEFAULT_MOVE_SPEED = 50.0` | 按上游 25（拍板 ⑥） |

四个现有单测直接构造 `move_command`（`toolhead.rs:1377/1410/1437/1461`：解析轴与速度、记住上次
速度、拒绝非正 `F`、未回零轴拒绝）——它们是**参数解析**的测试，跟着 `G1` 搬到 `gcode_move`。

---

## 4. 拆分

### G4-1 坐标系核心（一个提交）

1. 新 `extras/gcode_move.rs`：状态 + `PrinterObject`（对象名 `gcode_move`）+ `get_status`。
2. 命令：`G0`/`G1`、`G20`/`G21`、`M82`/`M83`、`G90`/`G91`、`G92`、`M220`、`M221`、
   `SET_GCODE_OFFSET`、`SAVE_GCODE_STATE`/`RESTORE_GCODE_STATE`、`M114`。
3. 加载：`toolhead::load_config` 里 `gcode_move::ensure(printer)`（对上游 default modules 列表）。
4. `toolhead` 减法：删 `G0`/`G1` 注册与 `move_command`，加 `move_to`/`position` 公共方法。
5. 重置链：`klippy:ready`（解析 toolhead + 首次 reset）、`homing:home_rails_end`（带 `axes`）、
   `gcode:command_error`、`toolhead:set_position`（`SET_KINEMATIC_POSITION` 处发）。
6. 单测：坐标系数学（假的 move target）、`G92` 后 `G1` 的工具头坐标、相对/挤出模式、倍率、
   存取状态、`get_status`；toolhead 那四个测试改成走新 API。

### G4-2 外围（后续）

`GET_POSITION`（要运动学的反解与 stepper 位置查询）、extra-axes `axis_map`（`Coord` 加长或另存）、
`toolhead:manual_move` / `toolhead:update_extra_axes` 的发送方（等 `manual_move` 与 extra-axis
gcode id 的 API 落地）、`move_transform`（等 `bed_mesh`）、`extruder:activate_extruder` 发送方
（等 `ACTIVATE_EXTRUDER`）。

---

## 5. 拍板点

1. **`G0`/`G1` 归属**：**搬到 `gcode_move`**（上游形状；`toolhead` 不再认坐标系）。
   备选是留在 `toolhead` 里加偏移——省一个对象，但 `bed_mesh` 的 `set_move_transform` 就没有
   挂载点，且与上游的分工相反。**建议搬。**
2. **加载点与注册顺序**：`gcode_move::ensure` 放在 `toolhead::load_config` 里（上游
   `add_printer_objects` 同位），但本仓库的 loader **在工厂返回后才 `add_object`**，所以
   `gcode_move` 会排在 `toolhead` **之前**进注册表 → `connect()` 跑在 toolhead 之前，解析不到
   toolhead。**解析必须放 `klippy:ready` 事件**（与上游 `_handle_ready` 同形），`connect` 只做
   无依赖的事。
3. **`homing:home_rails_end` 加 `axes` 载荷**：现在无载荷、每轴发一次；上游带
   `homing_state.get_axes()`，`gcode_move` 靠它只给回零过的轴重新锚定 `base_position`。
   给载荷后每轴发一次与上游「一次发全部」对回零锚定的效果一致。现有 handler 数 = 0，
   **无兼容面，建议加**。
4. **`toolhead:set_position` 谁发**：上游由 `ToolHead.set_position`（`toolhead.py:390`）发，
   覆盖 `SET_KINEMATIC_POSITION` 与回零途中把轴放到 `position_endstop`。我们的
   `motion::ToolHead` 没有 printer 句柄。**建议**：回零路径由 `homing:home_rails_end` 覆盖，
   `SET_KINEMATIC_POSITION` 这条命令处发事件（在 `extras/toolhead.rs`，那里有 printer）；
   不给 motion 层塞句柄。备选：给 `motion::ToolHead` 一个 `Weak<Printer>`，与上游完全同形。
5. **`Coord` 4 轴**：extra 轴的 gcode id（`axis_map[4:]`）缓到 G4-2 —— 现在 `Coord` 是
   `[f64; 4]`，加长会动 motion 热路径（`Move`/`trapq` 都按 4 轴切）。单挤出机的回归不受影响。
6. **默认速度**：上游 `self.speed = 25.`（mm/s，`F` 缺省时的 `G1` 速度），我们现在
   `DEFAULT_MOVE_SPEED = 50.0`。**建议按上游 25**——语料里 `G1` 不带 `F` 的不少。

---

## 6. 实施记录（2026-09-25 回填；2026-10-04 复核）

**G4-1 坐标系核心：已落地** —— `extras/gcode_move.rs`（状态、`G90/G91`、`M82/M83`、`G92`、
`SET_GCODE_OFFSET`、`M220/M221`、`SAVE/RESTORE`、`M114`、`get_status`）；`commands.test` 与
`out_of_bounds.test` 随之转绿，语料全绿（批 #41）。

**G4-2 仍未做**（权威口径是 `gcode_move.rs` 头注释的「What is not here」）：

| 余项 | 现状 |
|---|---|
| `GET_POSITION` | **未注册**（未知命令静默放行，因此不扣分）；需 `kin.get_steppers()` + `calc_position` + `McuStepper` 的 MCU 位置 |
| extra 轴的 `axis_map` | 本仓 `Coord` 是 4 轴（`mathutil.rs:22` `AXES = 4`），映射停在 `E` |
| `toolhead:sync_print_time` | 仍无发送点（C1d 的回调已落地；`idle_timeout` 改为观察 `print_time` 前进） |
| `motion_report`（`dump_trapq`/`dump_stepper`） | 未做，跟踪在 `TODO.md` 的 H10 / B4 |
| `extruder:activate_extruder` 的发送方 | `ACTIVATE_EXTRUDER` 已落地（`extruder.rs:338`）但只调 `set_active_extruder`，**不发事件**——处理器仍空转 |

**原表两行已收尾（复核订正）**：

| 原记 | 现状 |
|---|---|
| `toolhead:manual_move` / `toolhead:update_extra_axes`「处理器已备，**无发送点**」 | 两个事件都已有产线发送点：`manual_move` 由 `safe_z_home` 的 `HomeOps::manual_move` 发出（`safe_z_home.rs:215`，`LiveHome` 在 `:419` 构造）；`update_extra_axes` 由 `ToolHeadObject::add_extra_axis` / `remove_extra_axis` 发出（`toolhead.rs:1876` / `:1892`，`manual_stepper.rs:452` / `:462` 调用）。仍缺的只是上游那两个通用 API 的其余调用方 |
| `set_move_transform`「已写并占上游 `bed_mesh` 的槽，但无人换 target」 | 已有人换：`bed_tilt.rs:376`、`exclude_object.rs:212` 都在产线路径上换 target；缺口只剩上游 `bed_mesh` 本身 |

**正文（§1–§5）中已被实现取代的断言**——正文是 2026-09-23 的快照，除 §1 那处就地更正
与开头那处行数更正外不再改写：

| 正文 | 现状 |
|---|---|
| §2.1「状态（`__init__`，`gcode_move.py:21-37`）」、§2.2「`G1` 算法（`:117-141`）」 | 行号已漂移：`__init__` 在 `:9`、状态字段块 `:29-40`、`cmd_G1` 在 `:134` |
| §3「`G0`/`G1` 现在在 `toolhead`（`extras/toolhead.rs:394` + `move_command :998`）」 | 已搬家：`toolhead.rs` 里没有 `move_command`，`G0`/`G1` 注册在 `extras/gcode_move.rs` |
| §3「`last_position` 重置：发送方缺 2 个，`homing:home_rails_end` **没有载荷**」 | 已补齐：`event/decl/homing.rs:6` 带 `axes`；`extras/toolhead.rs:1752` 发 `toolhead:set_position` |
| §3「我们现在 `DEFAULT_MOVE_SPEED = 50.0`」 | 该常量已不存在；`gcode_move.rs:80` `DEFAULT_SPEED = 25.0` |
| §3「四个现有单测直接构造 `move_command`（`toolhead.rs:1377/1410/1437/1461`）」 | 那四行现是 kinematics 构建代码；参数解析测试已搬到 `gcode_move.rs`（`test_g1_parses_axes_and_speed_into_a_move` 等），「未回零轴拒绝」留在 `toolhead.rs` |
| §3「`move_transform` — 缓（先只认 toolhead）」 | 已实现并被产线使用（见上表） |
| §1「语料里的坐标系命令（inline + `*.gcode`）」的计数 | 实测是 `test/klippy/*.test` 的口径（复核 `G1 212`、`G90 15`、`G91 9`、`M83 3`、`M114 2`、`G92 2` 与正文一致；`G2`/`M486` 为 19/40，正文 18/39），并非正文自称的「inline + `*.gcode`」 |
| §5 拍板①–⑥ | ①搬到 `gcode_move` ✅ ②解析放 `klippy:ready` ✅ ③`home_rails_end` 加 `axes` ✅ ④命令处发 `set_position` ✅ ⑤`Coord` 仍 4 轴（未做，见余项）⑥默认速度 25 ✅ |
