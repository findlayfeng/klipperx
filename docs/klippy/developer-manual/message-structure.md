# 消息编解码（msg）

`msg` 层只做一件事：把「消息名 + 参数值」和「字节」互相转换。它不知道任何具体命令的存在。

## `Msg` — 一条消息的定义

```rust
pub struct Msg {
    pub id: i16,                            // 线上 id（固件字典给出）
    pub name: String,                       // 消息名（格式串首 token）
    pub params: Vec<(String, ArgType)>,     // 参数名与类型，按声明顺序
    pub callback: Option<MsgCallback>,      // 可选入站回调
}
```

- `Msg::parse(id, "clock clock=%u")` 解析格式串；首 token 是名字，其余是 `name=type` 对。
- `Msg::new(id, name, params)` 由组件构造。
- `Msg::format()` 由 name + params 反推格式串（`%*s` 会归一化为 `%s`）。
- `id` 是 `i16`：Klipper 用有符号 VLQ 编码 id，所以部分 id 在字典里表现为负数。

`MsgCallback` 是 `Arc<Mutex<Box<dyn FnMut(&[ArgValue]) + Send>>>`，通过 `Parser::bind` 绑定。

## `ArgType` / `ArgValue` — 参数类型与取值

| 格式说明符 | `ArgType` | `ArgValue` |
|-----------|-----------|------------|
| `%u` | `UInt32` | `UInt32` |
| `%i` | `Int32` | `Int32` |
| `%hu` | `UInt16` | `UInt16` |
| `%hi` | `Int16` | `Int16` |
| `%c` | `UInt8` | `UInt8` |
| `%s` / `%*s` | `Str` | `Str` |
| `%.*s` | `Bytes` | `Bytes` |

`ArgType::parse_format(spec)` / `format_str()` 在两个方向转换。

`ArgValue::try_convert_to(target)` 只允许**无损**转换：

- 整数 → 整数，且必须落在目标类型范围内（既覆盖加宽，也覆盖带检查的收窄；溢出会被拒绝而不是截断）。
- `Str` ↔ `Bytes`：两者线上编码相同；`Bytes → Str` 还要求合法 UTF-8。
- 数字 ↔ 字符串一律拒绝。

## `Payload` — 一段消息块序列

一条 payload 是若干「消息块」的拼接：`[id, param1, param2, …][id, …]…`。

| 方法 | 说明 |
|------|------|
| `Payload::new()` / `from_raw(Vec<u8>)` | 构造 |
| `push(&self, byte)` / `extend(bytes)` | 追加原始字节 |
| `push_u8/u16/u32/i16/i32/bytes` | 追加单个参数值 |
| `push_value(&ArgValue)` / `extend_values(&[ArgValue])` | 按值追加 |
| `try_merge(&Payload)` | 合并另一段 payload，超限则不改动并报错 |
| `as_parser()` | 得到只读游标 `PayloadParser` |
| `into_raw()` | 取出原始字节 |
| `payload()` / `len()` / `is_empty()` | 只读访问 |

`Payload` 的所有写入都受 `MESSAGE_PAYLOAD_MAX`（`frame.rs`，值为 `MESSAGE_MAX - MESSAGE_MIN` = 59 字节）约束，超限返回 `MsgError`。

`PayloadParser` 提供 `pop_u8/u16/u32/i16/i32/bytes/string` 与 `pop_value(ArgType)` / `pop_values(&[ArgType])`，按同一套编码逐字段读取。

## `param.rs` 中的 `Param`

```rust
pub enum Param {
    Positional(ArgValue),
    Named(String, ArgValue),
}
```

这是「位置 / 命名参数」的描述类型，**目前尚未接入 `Parser::encode`**（`encode` 只接受按声明顺序排列的 `&[ArgValue]`），保留为后续支持命名参数的接口。

---

- [← 开发手册首页](README.md)
- [Parser API 参考 →](parser-api.md)
