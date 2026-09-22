# Parser API 参考

`Parser` 是 `msg` 层的注册表与编解码入口：按名字或 id 索引消息定义，并把「消息名 + 参数值」翻译成字节，或反过来。

`Parser` **不负责 I/O**。帧的收发、批处理、请求/响应配对都在 `mcu` 层（见 [MCU 协议与数据字典](mcu-protocol.md)）。

## 核心类型

```rust
#[derive(Clone)]
pub struct Parser { /* Arc<Mutex<MsgMap>> */ }

pub struct MsgMap {
    by_id: HashMap<i16, Arc<Msg>>,
    by_name: HashMap<String, i16>,   // name → id，再经 by_id 解析
}
```

- 两个索引都是**唯一**的：id 或 name 重复会被拒绝（`duplicate id` / `duplicate name`）。
- `Parser` 是廉价句柄：`clone()` 共享同一份注册表。`Mcu` 正是靠这一点让接收任务在握手后立刻看到新注册的消息。

## 构造与注册

### `Parser::new() -> Parser`

创建一个**空**注册表。identify 消息由 `identify::new_parser` 显式注册（`Mcu::new` 构造时取用该注册表）（见 `IDENTIFY_MESSAGES`），`Parser` 自己不带任何内置格式。

### `Parser::register(&mut self, id: i16, format: &str) -> MsgResult<()>`

注册一条消息。格式串由空格分隔，首 token 为消息名，其余为 `name=type`。

```rust
let mut parser = Parser::new();
parser.register(5, "get_clock")?;                              // 无参数
parser.register(18, "clock clock=%u")?;                        // 一个 uint32
parser.register(15, "shutdown clock=%u static_string_id=%hu")?; // 混合类型
```

可能失败：格式串为空、参数缺少 `=`、类型说明符未知、id 或 name 重复。

### `Parser::register_all(&mut self, msgs: &[(i16, &str)]) -> MsgResult<()>`

批量注册。**首个失败即返回**，此前已成功的注册保留（因此调用方若需要「要么全成要么全不成」，应先自行校验）。用于装载数据字典与 identify 对内建格式。

### `Parser::is_registered(&self, name: &str) -> bool`

名字是否已注册。`Dictionary::install` 用它跳过固件字典中重复出现的 identify 消息。

## 查找

### `Parser::lookup(&self, name: &str) -> Option<Arc<Msg>>`

按名取消息定义。返回 `Arc<Msg>`，其中带有固件字典给出的**参数名与类型**——`Params` 就是靠它把响应参数按名字取出。

> `Parser` 是**纯编解码器**：它不再保存入站回调（`Msg` 已无 `callback` 字段）。回调按 id
> 存在 `mcu` 层的 `McuEvents` 里（见 [MCU 协议与数据字典](mcu-protocol.md)），这样编解码表
> 可以被发/收两侧共享，而不会把回调捕获的资源拖成 `Mcu` 的强引用环（TODO F8b）。

## 编码（出站）

### `Parser::encode(&self, name: &str, values: &[ArgValue]) -> MsgResult<Payload>`

按注册的顺序编码一条命令：先写入 id（有符号 VLQ），再依次写参数。

```rust
let payload = parser.encode("clock", &[ArgValue::UInt32(1234)])?;
```

参数个数必须与声明一致；每个值必须匹配声明类型，或可无损转换为声明类型（见 `ArgValue::try_convert_to`）。

## 解码（入站）

### `Parser::decode(&self, payload: Payload) -> MsgResult<Vec<(Arc<Msg>, Vec<ArgValue>)>>`

按顺序解析一段 payload 中的**所有**消息块，返回每条消息的定义与参数值（顺序与 payload 中一致）。

```rust
let decoded = parser.decode(frame.into())?;
for (msg, params) in decoded {
    println!("{} (id={}) -> {:?}", msg.name, msg.id, params);
}
```

任一 id 未注册、或参数字节不足，都会返回 `MsgError`（`mcu` 层会记录日志并跳过该帧）。

## 回调绑定

回调不在 `Parser` 上，而在 `mcu` 层的 `McuEvents`（`src/core/klippy/mcu/events.rs`）：

```rust
// McuEvents：按消息 id 索引的回调表
pub(crate) fn bind(
    &self,
    parser: &Parser,
    name: &str,
    callback: impl FnMut(&[ArgValue]) + Send + 'static,
) -> MsgResult<()>;
pub(crate) fn callback(&self, id: i16) -> Option<MsgCallback>;
```

`bind` 先用 `Parser::lookup` 把名字解析成 id 再存入表中（因此只能绑到字典里真实存在的
消息），重复绑定替换旧回调，回调接收**按声明顺序**排开的参数值。`Mcu` 把它们包成
`Mcu::bind_event` / `Mcu::bind_callback`（见 [事件系统](event-system.md)）。

**消息如何被投递由 `mcu` 层的接收任务决定**：

1. 若有同步调用（`Mcu::call`）正在等待该响应名，投递给该调用，回调**不会**触发；
2. 否则按消息 id 查 `McuEvents`，命中就调用它；
3. 否则记录 `Unhandled message … discarding` 警告。

`McuObject` 在本身被丢弃（机器拆机）时调用 `Mcu::clear_events()` 清空该表：回调可能
持有会反向引用 `Mcu` 的资源（传输句柄），不清理就形成强引用环，`Mcu::Drop` 不跑、其
阻塞读驻留，runtime 关停会卡住（TODO F8b）。

## 错误类型

```rust
pub struct MsgError { pub msg: String }
pub type MsgResult<T> = Result<T, MsgError>;
```

`MsgError` 是字符串型错误，`Display` 直接打印 `msg`。上层用 `McuError::Msg` 包装它。

---

- [← 开发手册首页](README.md)
- [消息编解码 ←](message-structure.md) · [MCU 协议与数据字典 →](mcu-protocol.md)
