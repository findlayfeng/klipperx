# 事件系统

本文描述打印机级事件总线的形状：事件如何被声明、注册与分发，以及与上游
`klippy/klippy.py` 中 `Printer.send_event` / `register_event_handler` 的对应关系。

> **状态**：已实现。事件词汇为 `KlippyEvent`（`event/printer_bus.rs`），由 `build.rs`
> 从 `event/decl/` 下的声明生成；`Printer` 按事件名注册与分发，未知事件记录警告，
> 处理器 panic 相互隔离。原先的封闭枚举 `PrinterEvent` 已删除，调用点全部迁移。

## 1. 上游事件系统

### 1.1 数据结构与接口

上游 `Printer` 用一个字典保存处理器：

```python
# klippy/klippy.py
class Printer:
    def __init__(self, ...):
        self.event_handlers = {}   # dict[str, list[Callable]]

    def register_event_handler(self, name, cb):
        self.event_handlers.setdefault(name, []).append(cb)

    def send_event(self, name, *params):
        for cb in self.event_handlers.get(name, []):
            cb(*params)
```

事件名是普通字符串，没有集中定义；处理器是任意可调用对象，通过 `send_event` 的可变
参数接收载荷。因此新增事件不需要修改任何公共类型，但事件名拼写错误只会在运行时暴露。

### 1.2 事件清单

上游共有 35 个事件，按命名空间分组如下。

| 命名空间 | 数量 | 事件名 | 载荷 |
|----------|------|--------|------|
| `klippy:` | 8 | `mcu_identify`、`connect`、`ready`、`shutdown`、`disconnect`、`firmware_restart` | 无 |
| | | `notify_mcu_error`、`analyze_shutdown` | `msg: str, details: dict` |
| `idle_timeout:` | 3 | `idle`、`printing`、`ready` | 无 |
| `homing:` | 4 | `homing_move_begin`、`homing_move_end`、`home_rails_begin`、`home_rails_end` | 对象引用（`homing_state`） |
| `stepper:` | 2 | `sync_mcu_position`、`set_dir_inverted` | 对象引用（`stepper`） |
| `toolhead:` | 4 | `manual_move`、`set_position`、`sync_print_time`、`update_extra_axes` | 混合 |
| `gcode:` | 3 | `command_error`、`debuginput_exit` | 无 |
| | | `request_restart` | `print_time: float` |
| `probe:` | 1 | `update_results` | 无 |
| `extruder:` | 1 | `activate_extruder` | 无 |
| `stepper_enable:` | 1 | `motor_off` | 无 |
| `virtual_sdcard:` | 1 | `reset_file` | 无 |
| `load_cell:` | 2 | `calibrate`、`tare` | 无 |
| `menu:` | 4 | `populate`、`init`、`begin`、`exit` | 无 |
| `dual_carriage:` | 1 | `update_kinematics` | 无 |

除 `klippy:notify_mcu_error`、`klippy:analyze_shutdown` 与 `gcode:request_restart` 外，
其余事件的载荷均为空或为对象引用。

### 1.3 触发时序

生命周期事件的触发顺序如下（`klippy/klippy.py`）：

```
_connect()
  ├── _read_config()
  ├── send_event("klippy:mcu_identify")     # 配置装载完成，对象尚未 connect
  ├── 逐个 connect 对象
  ├── send_event("klippy:connect")
  ├── set_state("ready")
  ├── send_event("klippy:ready")
  └── run()
        ├── send_event("klippy:firmware_restart")   # 需要重启时
        └── send_event("klippy:disconnect")

invoke_shutdown(msg, details)
  ├── set_state("shutdown")
  ├── send_event("klippy:shutdown")
  └── send_event("klippy:analyze_shutdown", msg, details)
```

要点：`klippy:mcu_identify` 在配置装载之后、对象连接之前触发；`klippy:connect` 在所有
对象连接成功之后触发；`klippy:analyze_shutdown` 在 `klippy:shutdown` 之后触发，并携带
停机原因与细节。

### 1.4 处理器调用约定

上游在分发时对处理器做了两重保护：

- **异常隔离**：每个处理器单独包裹 `try/except`，一个处理器抛出异常不影响其余处理器。
- **禁止阻塞**：`klippy:shutdown` 与 `klippy:ready` 的分发包在 `reactor.assert_no_pause()`
  中。处理器若在其中等待或暂停，会被 reactor 直接判为错误。

上游代码片段：

```python
def invoke_shutdown(self, msg, details={}):
    ...
    with self.reactor.assert_no_pause():
        for cb in self.event_handlers.get("klippy:shutdown", []):
            try:
                cb()
            except Exception:
                logging.exception("Exception during shutdown handler")
        for cb in self.event_handlers.get("klippy:analyze_shutdown", []):
            try:
                cb(msg, details)
            except Exception:
                logging.exception("Exception in analyze_shutdown handler")
```

## 2. 当前实现

### 2.1 已实现

`KlippyEvent` 声明了上游全部 35 个事件名，由 `build.rs` 从 `event/decl/` 下的声明
生成（见 §3）。事件声明已覆盖 `klippy:`、`idle_timeout:`、`homing:`、`stepper:`、
`toolhead:`、`gcode:`、`probe:`、`extruder:`、`stepper_enable:`、`virtual_sdcard:`、
`load_cell:`、`menu:` 与 `dual_carriage:`。

`Printer` 的处理器按事件名索引，注册与分发接口为 `register_event_handler` /
`send_event`，与上游同名。生命周期事件 `klippy:mcu_identify`、`klippy:connect`、
`klippy:ready`、`klippy:shutdown`、`klippy:analyze_shutdown`、`klippy:firmware_restart`
与 `klippy:disconnect` 已在实际时序上触发（见 §4）。

MCU 侧另有独立的 `event` 模块，以 `McuEvent` trait 表达固件主动推送的消息
（`shutdown` / `is_shutdown` / `starting`、`stats`）。该层与打印机级事件总线是两套
机制：前者由固件推送、经 `Mcu::bind_event` 绑定；后者由主机内部触发。

### 2.2 覆盖范围

`klippy:notify_mcu_error` 已声明并在 `Printer::bring_up` 中接入触发点：当 MCU 对象
`connect()` 失败时，在 `invoke_shutdown` 之前发出，携带 `msg` 分类（`"Protocol error"`
或 `"MCU error during connect"`）与 `details`（原始错误信息）。与上游 `_connect` 中的
`send_event("klippy:notify_mcu_error", msg, {"error": str(e)})` 一致。

截至 2026-09-23，生产路径上**实际发出**的事件（按命名空间）：

| 命名空间 | 已发出 | 尚未发出（模块/发送方未就位） |
|----------|--------|------------------------------|
| `klippy:` | 全部 8 个 | — |
| `homing:` | 全部 4 个（`toolhead` 的回零循环发 begin/end） | — |
| `gcode:` | `command_error`、`request_restart` | `debuginput_exit`（依赖 `GCodeIO`，暂缓） |
| `toolhead:` | `set_position` | `manual_move`（已注册处理器，发送方是 G4-2）、`sync_print_time`、`update_extra_axes` |
| `stepper_enable:` | `motor_off` | — |
| `extruder:` | — | `activate_extruder`（已注册处理器，发送方是 G4-2） |
| `stepper:` | — | `sync_mcu_position`、`set_dir_inverted`（依赖 stepper 资源的同步路径） |
| `idle_timeout:` / `probe:` / `virtual_sdcard:` / `menu:` / `dual_carriage:` | — | 对应 extras 模块尚未实现（H3/H4/H6/H8/H9） |

事件名与变体已经就绪，处理器可先注册；上表右列的事件一旦模块落地，发送点直接用现成变体。`load_cell:` 已有发射者（批 #14：`klippy:ready` 时按状态发 `load_cell:calibrate`/`load_cell:tare`）。

## 3. 设计

### 3.1 方案选型

两种可行形状：一是把所有已知事件收进一个大枚举，并留一个兜底变体；二是保留上游的
字符串总线。对比如下。

| 维度 | 大枚举 + 兜底变体 | 字符串总线 |
|------|-------------------|-----------|
| 类型安全 | 编译期可检查 | 运行时才发现拼写错误 |
| 穷尽性 | `match` 可做穷尽检查 | 无法穷尽 |
| 事件名一致性 | 与上游一致 | 与上游一致 |
| 载荷表达 | 变体字段，类型明确 | 可变参数，类型丢失 |
| 新增事件 | 需改声明 | 无需改动 |
| 分发开销 | 按名查表，等价 | 按名查表 |

事件总数为 35，属于封闭且可预期的集合，因此选用大枚举方案：以枚举承载类型信息，
以 `Unknown` 变体承接未在枚举中声明的事件名，避免因上游新增事件而丢失分发。

### 3.2 枚举定义

枚举由声明文件在编译期生成，变体按事件名排序。形状如下：

```rust
/// 打印机级事件。
///
/// 每个变体对应上游 `send_event` 的一个事件名，`name()` 给出该名字。
/// `Unknown` 承接未声明的事件名。
#[derive(Debug, Clone, PartialEq)]
pub enum KlippyEvent {
    // klippy:
    KlippyMcuIdentify,
    KlippyConnect,
    KlippyReady,
    KlippyShutdown,
    KlippyDisconnect,
    KlippyFirmwareRestart,
    KlippyNotifyMcuError { msg: String, details: HashMap<String, Value> },
    KlippyAnalyzeShutdown { msg: String, details: HashMap<String, Value> },

    // idle_timeout:
    IdleTimeoutIdle,
    IdleTimeoutPrinting,
    IdleTimeoutReady,

    // homing:
    HomingHomingMoveBegin,
    HomingHomingMoveEnd,
    HomingHomeRailsBegin,
    HomingHomeRailsEnd { axes: Vec<usize> },

    // stepper:
    StepperSyncMcuPosition,
    StepperSetDirInverted,

    // toolhead:
    ToolheadManualMove,
    ToolheadSetPosition,
    ToolheadSyncPrintTime,
    ToolheadUpdateExtraAxes,

    // gcode:
    GcodeCommandError,
    GcodeDebuginputExit,
    GcodeRequestRestart { print_time: f64 },

    // probe:
    ProbeUpdateResults,

    // extruder:
    ExtruderActivateExtruder,

    // stepper_enable:
    StepperEnableMotorOff,

    // virtual_sdcard:
    VirtualSdcardResetFile,

    // load_cell:
    LoadCellCalibrate,
    LoadCellTare,

    // menu:
    MenuPopulate,
    MenuInit,
    MenuBegin,
    MenuExit,

    // dual_carriage:
    DualCarriageUpdateKinematics,

    // 兜底
    Unknown { name: String, params: HashMap<String, Value> },
}
```

`name()` 给出与上游一致的字符串：

```rust
impl KlippyEvent {
    pub fn name(&self) -> &str {
        match self {
            KlippyEvent::KlippyConnect => "klippy:connect",
            // ...
            KlippyEvent::Unknown { name, .. } => name.as_str(),
        }
    }
}
```

该枚举不实现 `Hash`，也不实现 `Eq`：变体可携带 `HashMap<String, Value>`（都不实现 `Hash`）与
`f64`（`print_time`，不实现 `Eq`），所以生成器只 derive `Debug, Clone, PartialEq`；分发按
事件名进行，不需要枚举自身可哈希或全序相等。

### 3.3 处理器签名与载荷传递

处理器接收事件引用，需要载荷时读取变体字段，不需要时忽略参数：

```rust
type EventHandler = Arc<dyn Fn(&KlippyEvent) + Send + Sync>;
```

这样 `KlippyNotifyMcuError` 与 `KlippyAnalyzeShutdown` 的载荷通过变体字段传递，而其余
无载荷事件不必传参。注册处写为 `|_| { ... }`。

对于上游以对象引用为载荷的事件（`homing:home_rails_begin` 的 `homing_state`、
`stepper:sync_mcu_position` 的 `stepper` 等），载荷不进入枚举：处理器在注册时已能从
闭包捕获所需对象，或经 `Printer::lookup_object` 取得，无需在分发路径上传递引用。这与
上游处理器直接接收对象的写法在效果上一致，同时避免在枚举中保存带生命周期的引用。
少数需要值的载荷例外地进了枚举：`homing:home_rails_end` 的 `axes: Vec<usize>`（
`gcode_move` 按轴清 homed 状态）与 `gcode:request_restart` 的 `print_time: f64`。

`details` 使用 `HashMap<String, Value>`（`serde_json::Value`），与上游的 `dict` 对应。

### 3.4 编译期代码生成

事件声明分散在各子模块中，避免所有事件集中在一个文件里维护。声明文件只写事件，
`build.rs` 扫描这些声明并生成枚举与 `name()`。这一机制与配置段落的工厂表、API
端点安装表共用，见 [声明式表生成（build.rs）](codegen.md)。

#### 目录布局

```
build.rs                                  # 位于 crate 根
src/core/klippy/event/
├── mod.rs                                # 既有 MCU 事件层；声明 decl 与 printer_bus
├── printer_bus.rs                        # 打印机级事件总线，include! 生成文件
└── decl/
    ├── mod.rs                            # 定义空展开的 event! 宏；列出声明模块
    ├── klippy.rs
    ├── idle_timeout.rs
    ├── homing.rs
    ├── stepper.rs
    ├── toolhead.rs
    ├── gcode.rs
    ├── probe.rs
    ├── extruder.rs
    ├── stepper_enable.rs
    ├── virtual_sdcard.rs
    ├── load_cell.rs
    ├── menu.rs
    └── dual_carriage.rs
```

生成文件写入 `OUT_DIR`，不进入源码树：

```rust
// src/core/klippy/event/printer_bus.rs
include!(concat!(env!("OUT_DIR"), "/klippy_events.rs"));
```

声明文件是正常编译的模块，其中 `event!` 是展开为空的声明宏；`build.rs` 读取其源码文本，
解析每个 `event!` 调用。宏定义在 `decl/mod.rs` 中且位于各声明模块之前，因此子模块
无需 `use` 即可使用。

#### 声明语法

```rust
// src/core/klippy/event/decl/klippy.rs
event!("klippy:mcu_identify");
event!("klippy:connect");
event!("klippy:ready");
event!("klippy:shutdown");
event!("klippy:disconnect");
event!("klippy:firmware_restart");
event!("klippy:notify_mcu_error", { msg: String, details: HashMap<String, Value> });
event!("klippy:analyze_shutdown", { msg: String, details: HashMap<String, Value> });

// src/core/klippy/event/decl/homing.rs
event!("homing:homing_move_begin");
event!("homing:homing_move_end");
event!("homing:home_rails_begin");
event!("homing:home_rails_end");
```

`event!` 有两种形式：`event!("name")` 表示无载荷；`event!("name", { 类型: 字段, ... })`
表示带载荷的结构体变体。

#### build.rs 职责

1. 遍历 `src/core/klippy/event/decl/` 下的 `.rs` 文件（`mod.rs` 除外）。
2. 逐行匹配 `event!` 调用，解析事件名与可选载荷。
3. 生成 `$OUT_DIR/klippy_events.rs`，包含 `KlippyEvent` 定义与 `name()` 实现。
4. 对声明目录与每个声明文件发出 `cargo:rerun-if-changed`。

生成器所需的辅助函数：把 `klippy:mcu_identify` 一类的名字转成 `KlippyMcuIdentify`
（按 `:` 分段、每段按 `_` 分词、各词首字母大写），并按声明顺序稳定输出，便于审阅生成结果。

#### 未知与重复声明

- 同一个事件名在两个声明文件中出现时，`build.rs` 以 `panic!` 终止构建，避免静默覆盖。
- 生成文件头写入「自动生成，请勿手工修改」的注释，并注明生成器位置。

### 3.5 注册与分发

处理器按事件名索引，`Unknown` 也走同一张表：

```rust
/// 已注册的事件处理器。
type EventHandler = Arc<dyn Fn(&KlippyEvent) + Send + Sync>;

struct Inner {
    // ...
    handlers: HashMap<String, Vec<EventHandler>>,
}

impl Printer {
    pub fn register_event_handler(
        &self,
        event: KlippyEvent,
        handler: Box<dyn Fn(&KlippyEvent) + Send + Sync>,
    ) {
        self.lock()
            .handlers
            .entry(event.name().to_string())
            .or_default()
            .push(Arc::from(handler));
    }

    pub fn send_event(&self, event: &KlippyEvent) {
        let handlers = match self.lock().handlers.get(event.name()) {
            Some(handlers) => handlers.clone(),
            None => {
                if matches!(event, KlippyEvent::Unknown { .. }) {
                    tracing::warn!(event = event.name(), "unhandled unknown event");
                }
                return;
            }
        };
        for handler in handlers {
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                handler(event);
            }));
            if let Err(panic) = result {
                tracing::error!(event = event.name(), "event handler panicked: {panic:?}");
            }
        }
    }
}
```

与上游的两点对应：

- **异常隔离**：`catch_unwind` 对应上游的 `try/except`，单个处理器 panic 不影响其余。
- **未知事件告警**：没有注册处理器且事件为 `Unknown` 时记录警告，不中断。

`catch_unwind` 需要 `AssertUnwindSafe`，因为闭包捕获的 `&KlippyEvent` 未必满足
`UnwindSafe`；处理器之间通过共享状态 `&self` 互相影响的可能性由调用约定约束，不由类型
系统约束，与既有 `reactor` 回调的约定一致。

### 3.6 与既有枚举的关系

原先的 `PrinterEvent` 只有 5 个无参变体。迁移时把全部调用点（`printer.rs` 的测试、
`gcode.rs`、`extras/output_pin.rs`、`api/endpoints/gcode.rs`）改为 `KlippyEvent`，
并直接删除 `PrinterEvent`：它没有对外使用者，保留一层转换只会多出一个需要同步维护
的类型。`klippy/mod.rs` 的转出改为 `KlippyEvent`。

## 4. 生命周期事件的触发点

- `klippy:mcu_identify`：在 `Printer::bring_up` 中，配置已装载、对象尚未连接时触发。
  上游在 `_read_config()` 之后、对象连接循环之前触发。
- `klippy:connect`：全部对象连接成功后触发，位置不变。
- `klippy:ready`：状态置为 `Ready` 后触发，位置不变。
- `klippy:shutdown`：`Printer::invoke_shutdown` 中状态置为 `Shutdown` 后触发，位置不变。
- `klippy:analyze_shutdown`：在 `invoke_shutdown` 中 `klippy:shutdown` 之后触发，携带
  停机原因与细节；细节经 `invoke_shutdown_with(msg, details)` 传入（`invoke_shutdown(msg)`
  是 details 为空表的便捷入口），`error_mcu` 用前者带上原始错误。
- `klippy:notify_mcu_error`：在 `Printer::bring_up` 中，MCU 对象 `connect()` 失败时、
  `invoke_shutdown` 之前触发，携带 `msg`（`"Protocol error"` 或 `"MCU error during connect"`）
  与 `details`（`{"error": str}`）。上游在 `_connect` 中对应位置触发。
- `klippy:firmware_restart`、`klippy:disconnect`：位置不变。

## 5. 实施顺序

| 顺序 | 事项 | 依赖 | 状态 |
|------|------|------|------|
| 1 | `build.rs` 生成器与 `decl/` 声明目录 | 无 | 已实现 |
| 2 | 生成 `KlippyEvent` 与 `name()` | 1 | 已实现 |
| 3 | `register_event_handler` / `send_event` 改用 `KlippyEvent`，加入异常隔离与未知事件告警 | 2 | 已实现 |
| 4 | `bring_up` 触发 `KlippyMcuIdentify` | 3 | 已实现 |
| 5 | `invoke_shutdown` 触发 `KlippyAnalyzeShutdown` | 3 | 已实现 |
| 6 | 迁移 `gcode.rs`、`extras/output_pin.rs`、`api/endpoints/gcode.rs` 的调用点 | 3 | 已实现 |
| 7 | 更新测试并删除 `PrinterEvent` | 6 | 已实现 |
| 8 | 其余命名空间的事件在各自模块就位后逐步注册处理器 | 3 | 进行中：`homing` / `gcode` / `toolhead:set_position` / `stepper_enable:motor_off` 已发出（见 §2.2 表）；`idle_timeout` / `probe` / `virtual_sdcard` / `load_cell` / `menu` / `dual_carriage` 等模块落地后接入 |

## 6. 与上游的差异

| 项 | 上游 | 本实现 |
|----|------|--------|
| 事件名 | 运行时字符串 | 编译期枚举，`Unknown` 承接未声明者 |
| 载荷 | `*params` | 变体字段；对象引用由处理器自取 |
| 处理器签名 | 可变参数 | `Fn(&KlippyEvent)` |
| 异常隔离 | `try/except` | `catch_unwind` |
| 禁止阻塞 | `assert_no_pause()` 强制 | 沿用 `reactor` 回调约定，不强制校验 |
| 未知事件 | 静默忽略 | 记录警告 |
| 错误事件 | `klippy:notify_mcu_error` 在 MCU 错误路径发出 | 已接入 `bring_up` 中 MCU 连接失败路径 |

禁止阻塞一项不引入强制机制：`reactor` 已对回调约定「不等待、不做重活」，重复引入运行时
校验的收益有限，且 `catch_unwind` 已覆盖处理器崩溃这一主要风险。

---

- [← 开发手册首页](README.md)
- [时钟与定时器（reactor）→](reactor.md)
