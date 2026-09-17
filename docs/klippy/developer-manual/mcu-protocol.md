# MCU 协议与数据字典

`mcu` 层负责与一颗物理 MCU 通信，并在握手后把固件自述的协议变成可用的类型化接口。与它平级的 `cmd` 定义「有哪些命令、它们是什么意思」，`identify` 负责把字典取回来。

## 核心约束

**除 `identify` / `identify_response` 外，主机不定义任何收发命令格式。**

原因：格式串、参数类型、线上 id 都是固件的实现细节。字典里给出什么，主机就用什么；固件升级换了参数顺序或类型，主机不需要同步修改。唯一的例外是 identify 本身——字典就是通过它传过来的，只能硬编码（`IDENTIFY_MESSAGES`）。

## 数据流

```
                     ┌──────────────── cmd ─────────────────┐
                     │ ClockSync / McuClock / …             │
                     │ call_msg::<GetClock, ClockState>()   │
                     └───────────────┬──────────────────────┘
                                     │ McuCommand / McuResponse
                                     ▼
   ┌──────────────────────────── Mcu ────────────────────────────┐
   │ Parser（共享注册表）   Dictionary（枚举/常量/消息表）          │
   │ send()／call()          PendingCalls（请求响应配对）           │
   └──────┬───────────────────────────────────▲──────────────────┘
          │ mpsc<Payload>                     │ Parser::decode
          ▼                                   │
   ┌── 发送任务 ──┐                    ┌── 接收任务 ──┐
   │ 合并批处理    │                    │ seq 校验     │
   └──────┬───────┘                    └──────▲───────┘
          │ Frame                             │ Frame
          ▼                                   │
   ┌──────────────── Interface（Device） ────────────────────────┐
```

## `Mcu` — 一颗 MCU

| 方法 | 说明 |
|------|------|
| `Mcu::new(config) -> Mcu` | 只起传输：起始注册表只含 identify 一对（`identify::new_parser`）、起收发任务，**未识别** |
| `Mcu::connect(config) -> Arc<Mcu>` | 正常入口：`new` + identify 握手 + 安装字典 |
| `identify(timeout) -> Result<usize>` | 单独执行握手，返回新注册的消息条数 |
| `install_dictionary(dict) -> Result<usize>` | 安装字典（注册到 `Parser` 并留存） |
| `dictionary() -> Option<Arc<Dictionary>>` | 已安装的字典 |
| `is_identified() -> bool` | 是否已完成握手 |
| `name() -> &str` | MCU 名称 |
| `send(name, &[ArgValue])` | **裸**单向发送（不做握手门禁） |
| `call(name, args, response_name, timeout)` | **裸**同步请求/响应（不做握手门禁） |
| `send_msg::<C>(&C)` | 类型化单向发送（定义在 `cmd`） |
| `call_msg::<C, R>(&C, timeout)` | 类型化请求/响应（定义在 `cmd`） |

`Mcu` 不是 `Clone`，并且实现了 `Drop`：最后一个句柄被释放时调用 `Interface::shutdown()` 并 abort 接收任务。因此命令层统一持有 `Arc<Mcu>`，不要克隆。

构造与识别是两步：`Mcu::new(config)` 只把传输拉起来，之后 `Mcu::identify(timeout)` 抓取并安装字典；正常入口 `Mcu::connect(config)` 把这步串起来并返回 `Arc<Mcu>`。`connect` 与 `identify` 都定义在 `identify.rs`（与 `mcu` 平级的模块），因此整条引导流程与它的格式定义、分块驱动在同一个文件里。identify 的格式与分块拼装不放在命令层（`identify`，见 [Identify 机制](identify.md)），因为它是唯一在字典存在之前运行的交换；`mcu` 反过来只在构造时向它要起始 `Parser`，那是整个仓库唯一的反向上行边。

### 裸接口与类型化接口

| | `send` / `call` | `send_msg` / `call_msg` |
|---|---|---|
| 参数 | 字符串名 + `&[ArgValue]` | `McuCommand` / `McuResponse` |
| 返回值 | `Payload` / `Vec<ArgValue>` | 自定义类型 |
| 握手门禁 | 无 | 无字典时返回 `McuError::NotIdentified` |
| 名称校验 | 发送时才由 `Parser` 报错 | 发送**前**即报 `UnknownMessage` |
| 用途 | identify 握手、极底层调试 | 命令层 |

## `Dictionary` — 固件自述的协议

### 结构

固件下发四张表（键是格式串，值是 id）：

```json
{
  "commands":  {"get_clock": 5, "identify offset=%u count=%c": 1},
  "responses": {"clock clock=%u": 18},
  "output":    {"mpu9240 fifo_max=%u": 30},
  "enumerations": {"static_string_id": {"Timer too close": 3}, "pin": {"PL0": [0, 13]}},
  "config": {"CLOCK_FREQ": 20000000}
}
```

三者形状并不一致，这一点很容易踩坑：

- `commands` / `responses` 的键以消息名开头，所以能取出 `MessageDef { name, id, format }`。
- `output` 的键是自由文本（固件写作 `_DECL_OUTPUT("mpu9240 fifo_max=%u")`），**没有**前导消息名，所以原样保留为 `OutputDef { format, id }`，并且**不注册进 `Parser`** —— 异步输出属于尚未实现的事件层。
- 枚举值可以是单个整数，也可以是 `[start, count]` 区间。区间在解析期就展开（与 Klipper 的 `fill_enumerations` 一致）：`"PL0": [0, 13]` 展开成 `PL0`→0 … `PL12`→12。

### 安装

```rust
let installed = mcu.install_dictionary(dictionary)?;
```

- 命令与响应注册进 `Parser`。接收任务持有同一个 `Parser`（内部是 `Arc<Mutex<MsgMap>>`），所以**握手后立即生效**，不需要重启任务或重建解析器。
- 已注册的消息会被跳过。这不是可选优化：identify 一对既被主机硬编码，又原样出现在固件字典里，若不跳过会因 id / name 重复而失败。
- `output` 表不注册；`Dictionary::output()` 仍可读取。

### 查询接口

| 方法 | 说明 |
|------|------|
| `messages()` / `message(name)` | 遍历 / 按名查命令与响应 |
| `commands()` / `responses()` / `output()` | 分表访问 |
| `enumeration(name)` / `enumerations()` | 枚举（`Enumeration::value` / `name` / `iter`） |
| `constant(key)` / `constant_f64(key)` / `constants()` | 编译期常量，如 `CLOCK_FREQ` |
| `raw()` | 原始 JSON，未建模字段（`version` 等）不会丢失 |

## 类型化消息：`McuCommand` / `McuResponse` / `Params`

```rust
pub trait McuCommand {
    const NAME: &'static str;       // 对应字典里的命令名
    fn args(&self) -> Vec<ArgValue>;
}

pub trait McuResponse: Sized {
    const NAME: &'static str;       // 对应字典里的响应名
    fn decode(params: &Params<'_>) -> Result<Self, McuError>;
}
```

两个 trait 都**只声明名字与语义，不声明格式串、id 或参数类型**——这些来自字典。

### 按名字读参数

```rust
Ok(ClockState { clock: params.get_u32("clock")? })
```

`Params` 通过 `Msg::params` 把参数名映射到位置。**不要按下标取参数**：参数顺序由固件决定，某个固件版本插入或调整一个参数，按下标解就会静默解错，而按名字解会直接失败。

可用取值方法：`get_u32` / `get_u8` / `get_u16` / `get_i16` / `get_i32` / `get_bytes` / `get_str`，以及底层的 `get(name, ArgType)`。

- 取值会做范围检查的无损转换，类型不符返回 `McuError::Decode`，并同时报告实际类型与期望类型。
- 参数不存在时，错误里会列出该消息声明的全部参数，便于排查。
- `get_str` 对非法 UTF-8 给出专门错误，而不是笼统的类型不匹配。

### 枚举参数

```rust
let name = params.get_enum("static_string_id", "static_string_id")?;
```

第一个参数是要查的枚举名，第二个是参数名。Klipper 由参数名推断枚举名（`name == enum_name` 或 `name.endswith('_' + enum_name)`），所以两者通常相同——这里显式分开，避免隐式约定。固件未命名的取值渲染为 `?<value>`，与 Klipper 一致。

### 调用示例

```rust
let state: ClockState = mcu.call_msg::<GetClock, ClockState>(&GetClock, timeout).await?;
mcu.send_msg(&SetDigitalOut { oid, value })?;
```

`call_msg` 在发送**之前**就解析命令名与响应名，所以固件不实现的消息会立刻失败，而不是白等一个超时。

## 命令层：把消息包装成能力

命令层的词汇本身在 `cmd`：两个方向的 trait（`McuCommand` / `McuResponse`）、按名取参的 `Params`，以及把它们跑起来的 `Mcu::send_msg` / `Mcu::call_msg`（Rust 允许把固有 `impl` 写在其它模块，`Mcu` 的文档页仍会把它们列在一起）。传输层只剩下按名字收发的 `send` / `call`。

```rust
pub trait ClockSync {
    fn get_clock(&self) -> impl Future<Output = Result<ClockState, McuError>> + Send;
}
```

- 能力 trait 用 `impl Future + Send` 而不是 `async fn`：`async fn` in trait 已稳定但不是 dyn-safe，且 `Send` 会变成隐式假设（触发 `async_fn_in_trait` lint）。显式写出后，返回值可直接用于 `tokio::spawn`。
- 命令模块由 `Arc<Mcu>` 构造，多个模块可共用一个 MCU。
- trait 也是一道测试缝：`cmd::clock` 的测试里就有一个不依赖 MCU 的 `FixedClock` 实现。

identify 是唯一的例外：它不在这一层，格式由主机自有、且运行在字典存在之前，所以它单独成一个与 `mcu` 平级的模块（`identify`，入口 `Mcu::connect`）。除此之外**任何**命令都应走上表的模式。

## 新增一条命令的步骤

1. **确认固件协议**：在固件的 `.dict`（或生成的字典）里找到 `commands` / `responses` 中的格式串，记下消息名与参数名。**不要**把它们抄成主机侧的常量。
2. **定义消息类型**：在所属 `cmd` 模块里实现 `McuCommand` / `McuResponse`，只写 `NAME` 与 `args()` / `decode()`。
3. **定义或扩展能力 trait**：在命令层暴露一个语义化方法（如 `get_clock`），并提供一个 `Mcu*` 实现调 `Mcu::call_msg` / `send_msg`。
4. **不需要手工注册**：[`Mcu::connect`](identify.md) 会把字典里所有命令与响应装进 `Parser`。
5. **补测试**：用 `TestDevice` 的 `MappingEntry` 构造期望的收发帧（见下）。

### 测试要点：帧序号

`TestDevice` 逐条比对**完整 `Frame`（含 seq 与 payload）**，而收发两侧各有一个序号计数器：

- 发送任务每发一批（一次同步调用 = 一批）序号 +1，取低 4 位；
- 接收任务要求**收到的**帧序号从 0 开始、按序递增，比较时取 `seq & 0xf`。

因此第 i 次交换的请求帧与响应帧序号都必须是 `i & 0xf`。序号只有 4 位，**超过 16 次交换会回绕**——构造大 payload 的多分块测试时务必 `& 0x0f`，否则第 17 帧起全部被判为序号不匹配而丢弃（表现为超时）。

## 当前未实现

| 项 | 说明 |
|----|------|
| 事件 / 异步 `output` | `output` 表已解析但不注册，`Parser` 也没有异步投递通道 |
| 并发同名响应 | `PendingCalls` 只按响应名匹配，先到先得；两个并发 `get_clock` 会互相抢答，需要 `oid` 之类的区分参数 |
| 枚举参与编解码 | `ArgType` 没有枚举变体，枚举只在 `Params::get_enum` 与 `Dictionary` 里手工解析 |
| 命名参数 | `Param` 类型已定义但未接入 `Parser::encode` |

---

- [← 开发手册首页](README.md)
- [Parser API 参考 ←](parser-api.md) · [Identify 机制 →](identify.md)
