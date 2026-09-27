# MCU 协议与数据字典

`mcu` 层负责与一颗物理 MCU 通信，并在握手后把固件自述的协议变成可用的类型化接口。与它平级的 `cmd` 定义「有哪些命令、它们是什么意思」，`identify` 负责把字典取回来。

## 核心约束

**除 `identify` / `identify_response` 外，主机不定义任何收发命令格式。**

原因：格式串、参数类型、线上 id 都是固件的实现细节。字典里给出什么，主机就用什么；固件升级换了参数顺序或类型，主机不需要同步修改。唯一的例外是 identify 本身——字典就是通过它传过来的，只能硬编码（`IDENTIFY_MESSAGES`）。

## 数据流

```
                     ┌──────────────── cmd ─────────────────┐
                     │ 命令模块（identify、basecmd 命令）      │
                     │ call_msg::<GetConfig, ConfigState>() │
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
| `bind_callback(name, cb)` | **裸**回调绑定（定义在 `mcu`，不做握手门禁） |
| `bind_event::<E>(handler)` | 类型化事件订阅：按名解码为 `E` 后交给 `FnMut(E)`（定义在 `event`） |

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
- `output` 的键是自由文本（固件写作 `_DECL_OUTPUT("mpu9240 fifo_max=%u")`），**没有**前导消息名，所以原样保留为 `OutputDef { format, id }`，并且**不注册进 `Parser`** —— 事件层（`event/`）已实现但只接 `responses` 表的 `sendf` 类消息，`output` 表的异步投递仍是待办（见「当前未实现」）。
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
Ok(Uptime { high: params.get_u32("high")?, clock: params.get_u32("clock")? })
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
let state: ConfigState = mcu.call_msg::<GetConfig, ConfigState>(&GetConfig, timeout).await?;
mcu.send_msg(&AllocateOids { count: oid_count })?;
```

`call_msg` 在发送**之前**就解析命令名与响应名，所以固件不实现的消息会立刻失败，而不是白等一个超时。

## 命令层：把消息包装成能力

命令层的词汇本身在 `cmd`：两个方向的 trait（`McuCommand` / `McuResponse`）、按名取参的 `Params`，以及把它们跑起来的 `Mcu::send_msg` / `Mcu::call_msg`（Rust 允许把固有 `impl` 写在其它模块，`Mcu` 的文档页仍会把它们列在一起）。传输层只剩下按名字收发的 `send` / `call`。

事件是 `cmd` 的同级层（`event`）：它复用这里的 `Params`，用 `Mcu::bind_event` 把回调绑到字典 `responses` 表里的消息上；因为事件没有请求，所以不经过 `call_msg`，只走 `Parser` 的回调路径（底层是 `mcu` 的 `Mcu::bind_callback`）。

```rust
pub trait ClockSync {
    fn get_clock(&self) -> impl Future<Output = Result<ClockState, McuError>> + Send;
}
```

- 能力 trait 用 `impl Future + Send` 而不是 `async fn`：`async fn` in trait 已稳定但不是 dyn-safe，且 `Send` 会变成隐式假设（触发 `async_fn_in_trait` lint）。显式写出后，返回值可直接用于 `tokio::spawn`。
- 命令模块由 `Arc<Mcu>` 构造，多个模块可共用一个 MCU。
- trait 也是一道测试缝：`cmd/clock.rs` 的测试里就有一个不依赖 MCU 的 `FixedClock` 实现。

上面这段 `ClockSync` 是**已编译**的：`cmd/clock.rs` 通过 `pub mod clock;` 进入编译，`ClockSync` / `McuClock` 把时钟同步与 `Mcu` 解耦，测试里有一个不依赖 MCU 的 `FixedClock` 替身。除此之外**任何**命令都应走上表的模式。

`basecmd.c` 的基础命令（`alloc` / `config` / `uptime` / `shutdown` 四个文件）只有类型化视图，没有对应的 trait：它们是连接与配置期的生命周期操作，当前只有一个实现，也不存在需要替换的后端，等出现使用者时再抽 trait 不迟。其中分配与配置期不是一条条即时发出的，而是由 `mcu/config.rs` 的 `ConfigBuilder` 攒起来一次性下发，见 [MCU 配置构建](mcu-config.md)。

identify 是唯一的例外：它不在这一层，格式由主机自有、且运行在字典存在之前，所以它单独成一个与 `mcu` 平级的模块（`identify`，入口 `Mcu::connect`）。

## 新增一条命令的步骤

1. **确认固件协议**：在固件的 `.dict`（或生成的字典）里找到 `commands` / `responses` 中的格式串，记下消息名与参数名。**不要**把它们抄成主机侧的常量。
2. **定义消息类型**：在所属 `cmd` 模块里实现 `McuCommand` / `McuResponse`，只写 `NAME` 与 `args()` / `decode()`。
3. **定义或扩展能力 trait**：在命令层暴露一个语义化方法（如 `get_clock`），并提供一个 `Mcu*` 实现调 `Mcu::call_msg` / `send_msg`。
4. **不需要手工注册**：[`Mcu::connect`](identify.md) 会把字典里所有命令与响应装进 `Parser`。
5. **补测试**：用 `FrameMock` 的 `MappingEntry` 构造期望的收发帧（见下）。

### 测试要点：帧序号

`FrameMock` 逐条比对**完整 `Frame`（含 seq 与 payload）**，而线上的序号是**一条连接一个数**，两端共享：

- 固件只有一个 `next_sequence`（`src/command.c:16`）：发的每一帧都盖它、只收带它的块、收下就 +1（**先 +1 再 dispatch**，`:301-305`），所以响应与紧随的空帧（ack）盖的都是**收下之后**的号；
- 主机这侧由发送任务独占这个号（`Wire::next`，展开成单调计数，只有低 4 位上线）。**写线之前就推进**：固件可能在写还没返回时就答上来，接收侧要拿这个号判断"这帧回答的是我们发过的块"；
- **空帧的号 = 固件此刻在等的号**。正常链路里它等于"刚发那个块 +1"；对不上时就是同步点：号**大于**我们要发的下一个 → 固件在别的会话里（板子没重启）→ **只在会话的首个新号上**成立：采纳它的号并**换号重发**未确认的块（connection-init，对应上游 `serialqueue.c:197-201` 的豁免与采纳）；首个新号**之后**再出现的越号帧丢弃，不采纳、不扰动在途交换——**但丢弃只丢载荷**：帧携带的号仍写入 `Wire::seen`（`place_frame` 纯函数），它是对固件位置的申报，**改号采纳的正是它**（否则观测号永不前进，改号只能拿陈旧值）；**改号会重挂一次 connection-init 豁免**（`serialqueue.c:196-201/261`）：改号后的第一个新号即使越号也采纳、其后恢复丢弃。**空帧的 ack/nak 二义性不在序号层猜**：`settle` 不对空帧单独改号，只有 identify 在「请求后窗口内静默」时才把固件报告的号读作 nak，调 `renumber_to_firmware` 改号重发（机理与常数见 [Identify 机制](identify.md)）；号**不比上次新**（重复 ack）→ 它在等我们发过的块（丢了）→ **原号重发**。
- **发送时机两道闸**：消息可携带 `SendClocks{min_clock, req_clock}`——`min`（不早于）压队到固件 move 槽释放时刻（每板 `MoveSlots` 池记账），`req` 最低者优先、距其 clock 100 ms 内直接放行（上游 `serialqueue.c:556 / 478-486 / 644-646`）；时钟估计不可用时全放行（`:612-618`）。fake（`test:`）传输**整体放行**两道闸（虚拟时钟无执行推进，闸在语料中会退化为墙钟串行——探针实证），其 `mcu_clock_poll` 也降到 150 ms（生产仍 1 s）。**C5 案结论**：两道闸的算式从未改过——病在入参
（print-time 打底停在 connect 快照）与固件步进链锚点（接管已配置固件缺 `reset_step_clock`）；配套埋点四类
 debug：`gate park … held_by=`、`gate open: released after …`、flush、`move in: … min= req= est= pool=`
（验收时用它们看压队与域界）。

所以真固件给第 i 个映射的响应是 `(i + 1) & 0xf`，后面还跟一个**同号的空帧**（ack）。夹具少一个 ack 也收得下（接收侧认两种号），但「块被收下」只有空帧说得清：没有 ack，12 块窗口（`MAX_PENDING_BLOCKS`，对齐上游）填满后就等一个永远不来的空位——小交换无妨，多块传递必须补。序号只有 4 位，**超过 16 块会回绕**，多分块测试里务必 `& 0x0f`，否则第 17 块起对不上（表现为超时）。

## 当前未实现

| 项 | 说明 |
|----|------|
| `output` 表的异步投递 | `sendf` 类事件（`responses` 表）已可经 `Mcu::bind_event` 回调投递；但 `output()` 申报的消息留在 `output` 表里，`Dictionary` 已解析却**不注册**，也没有对应通道 |
| 并发同名响应 | `PendingCalls` 只按响应名匹配，先到先得；两个并发 `get_clock` 会互相抢答，需要 `oid` 之类的区分参数 |
| 枚举参与编解码 | `ArgType` 没有枚举变体，枚举只在 `Params::get_enum` 与 `Dictionary` 里手工解析 |
| 命名参数 | `Param` 类型已定义但未接入 `Parser::encode` |

固件重启本身**已实现**：`McuConfig.restart_method` 由 `mcu/restart.rs` 读取并分派四种物理复位（`command` / `arduino` / `cheetah` / `rpi_usb`），配置握手在停机或 CRC 不一致时优先真重启（见 [MCU 配置构建](mcu-config.md)）。

---

- [← 开发手册首页](README.md)
- [Parser API 参考 ←](parser-api.md) · [MCU 配置构建 →](mcu-config.md)
