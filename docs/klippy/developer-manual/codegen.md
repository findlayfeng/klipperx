# 声明式表生成（build.rs）

有些「封闭词汇」必须以一个类型或一张表的形式集中起来，但条目天然属于各自的模块：

- **事件名**：`KlippyEvent` 枚举必须列举全部事件；
- **配置段落**：`load.rs` 的工厂表必须列举全部 `[section]`；
- **API 端点**：注册表必须列举全部端点。

手写的中心清单有两个问题：新增模块要回到中心文件改一处（容易漏），以及中心文件与模块定义会彼此漂移。本机制把「条目属于谁」和「条目在哪里被聚合」分开：模块在自己文件里写一行声明，`build.rs` 在编译期扫描并生成聚合结果。

## 机制

```
模块内的宏调用（展开为空，仅供扫描）
        │
        ▼
build.rs 扫描源码，解析声明
        │
        ▼
$OUT_DIR/<generated>.rs  （写入，不进源码树）
        │
        ▼
include!(concat!(env!("OUT_DIR"), "/<generated>.rs"))
```

- 声明宏（`event!` / `section!` / `endpoint!`）展开为**空**，只用于被读取；它同时保证声明是合法 Rust，编辑器能解析。
- 生成文件写入 `OUT_DIR`，不污染源码树；使用方用 `include!` 引入。
- 生成的路径是**全限定路径**，因此引用错误会在编译期报「unresolved path」，不会静默漏条目。

## 三种生成物

| 声明 | 声明位置 | 生成文件 | `include!` 于 | 生成的量 |
|------|----------|----------|----------------|----------|
| `event!("klippy:ready")` | `event/decl/*.rs` | `klippy_events.rs` | `event/printer_bus.rs` | `KlippyEvent` 枚举与 `name()` |
| `section!("mcu", order = 10, load = load_config)` | 拥有该段落的模块 | `section_factories.rs` | `load.rs` | `FACTORIES` 表 |
| `endpoint!(install)` | 端点模块 | `endpoint_installers.rs` | `api/mod.rs` | `ENDPOINT_INSTALLERS` 表 |

事件的声明集中在 `event/decl/`：这些事件名目前没有各自拥有的模块（多数发送方尚未实现），集中声明是当前唯一选择。段落与端点的声明则**内联在拥有者模块**里。

## 声明语法

### 事件

`event!("name")` 或 `event!("name", { field: Type, ... })`，一行一个，写在 `event/decl/<namespace>.rs`：

```rust
event!("klippy:ready");
event!("klippy:analyze_shutdown", { msg: String, details: HashMap<String, Value> });
```

载荷按逗号切分时尊重 `<>` / `()` / `[]` / `{}`，所以 `HashMap<String, Value>` 是一个字段。声明必须在一行内，跨行会终止构建。

### 段落

`section!("id", order = N, load = ..., prefix = ..., object = ..., phase = ...)`，写在拥有该段落的模块顶层
（可以跨行：`rustfmt` 会按需拆行，扫描按括号配对读取）：

```rust
// mcu/mod.rs
section!("mcu", order = 10, phase = early, load = load_config, prefix = load_config_prefix);

// extras/output_pin.rs
section!("output_pin", order = 20, prefix = load_config_prefix);

// extras/toolhead.rs
section!("printer", order = 60, phase = late, object = "toolhead", load = load_config);
```

- `load` 对应 `[id]`，`prefix` 对应 `[id <name>]`，至少写一个；
- `order` 必填，决定同一半（main / prefix）内的装载顺序（`load.rs` 的两次遍历按表序进行）；
- `phase = early | generic | late`（默认 `generic`）：`early` 排在普通节之前（上游先 `pins`/`mcu`），
  `late` 排在之后（上游最后才 load `toolhead`）；
- `object = "<name>"`：装载出的对象以别的名字注册（`[printer]` 注册成 `toolhead`）；
- 声明处需要 `use crate::core::klippy::load::section;` 引入宏。

### 端点

`endpoint!(install)`，写在端点模块顶层（同样可以跨行）。模块提供同名函数，签名是
`api::EndpointInstaller`：

```rust
// api/endpoints/objects_list.rs
endpoint!(install);

pub(crate) fn install(api: &mut Api, wiring: &ApiWiring<'_>) -> Result<(), RegistrationError> {
    api.register(ObjectsList::new(Arc::clone(wiring.printer)))
        .map_err(RegistrationError::Endpoint)
}
```

一个模块可以登记多个端点（`gcode.rs` 的 `install` 装五个），因此一个模块通常只需一行声明。宏定义在 `api/endpoints/mod.rs` 顶部、各 `pub mod` 之前，靠文本作用域覆盖子模块，无需 `use`。

## 名字解析

段落与端点的声明里，被引用的项按以下规则解析：

- 不含 `::`：视为声明模块的**同级**项，即 `<文件模块路径>::<名字>`；
- 含 `::`：按**全限定路径**使用，可指向别处。

文件模块路径由 `src/` 下的相对路径推导（`mod.rs` 代表其目录），例如 `src/core/klippy/extras/output_pin.rs` → `crate::core::klippy::extras::output_pin`。因此 `prefix = load_config_prefix` 生成 `crate::core::klippy::extras::output_pin::load_config_prefix`。

派生错误不会静默：生成表引用一个不存在的路径时，`cargo build` 直接失败。

## 校验

`build.rs` 在生成前终止构建的情形：

| 情形 | 报错 |
|------|------|
| 事件名 / 段落 id / 端点安装函数重复 | duplicate |
| 段落缺 `order`，或 `load`/`prefix` 都没有 | 缺项 |
| 段落有未知选项、`order` 不是整数 | 格式错误 |
| `phase` 不是 `early`/`generic`/`late`，或 `object` 不是带引号的名字 | 格式错误 |
| 事件声明跨行、载荷不是 `{ field: Type }` | 格式错误 |

## 新增一个条目

- **事件**：在对应命名空间的 `event/decl/<ns>.rs` 加一行；新命名空间再加一个声明文件与 `event/decl/mod.rs` 的一行 `pub mod`。
- **段落**：在拥有者模块加一行 `section!` 声明（模块本身仍需在 `extras/mod.rs` 等处 `pub mod`）。
- **端点**：在端点模块加 `endpoint!` 声明与 `install` 函数（模块本身仍需 `pub mod`）；无需改 `api::register`。

## 为什么不是别的做法

- **proc-macro**：需要一个独立的 proc-macro crate，且跨模块聚合仍需全局注册表；这里没有引入新 crate。
- **`inventory` / `linkme` 之类的分布式注册 crate**：得到的是运行期集合，丢掉枚举的穷尽检查，并且顺序无保证（`section!` 的 `order` 是契约的一部分）。
- **手写中心表**：正是本机制要消除的维护点。
