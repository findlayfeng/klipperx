# 消息结构

消息结构定义在 `src/core/klippy/msg/mod.rs`，包含三个核心类型。

## `MsgBase`

```rust
pub struct MsgBase {
    pub params: Vec<(String, ArgType)>,
}
```

一条已解析的命令格式，仅包含参数列表。`MsgBase::parse()` 解析 Klipper 格式字符串 `"G1 X=%u Y=%u"` 为 `(name, MsgBase)`。

## `MsgHandler`

`MsgBase` 的包装，附加一个可选的 `FnMut` 回调。实现了 `Deref<Target = MsgBase>`。

```rust
pub struct MsgHandler {
    msg: MsgBase,
    callback: Box<dyn FnMut(&[ArgValue]) + Send>,
}
```

## `MsgEntry`

```rust
pub enum MsgEntry {
    Base(MsgBase),        // 可发送，无回调
    Handler(MsgHandler),  // 有回调，不可发送
}
```

- `Base` → 可通过 `send()` 发送出站消息
- `Handler` → 入站时自动路由到 callback_queue，**不可用于 `send()`**

---

- [← 开发手册首页](README.md)
- [Parser API 参考 ←](parser-api.md) · [内部架构 →](architecture.md)
