# Parser API 参考

## 概述

`Parser` 封装了 Klipper 消息协议的消息注册、发送、接收和路由。

## 核心类型

### `Param` — 命令参数

```rust
pub enum Param {
    Positional(ArgValue),          // 位置参数，按命令定义顺序传入
    Named(String, ArgValue),       // 命名参数，可任意顺序
}
```

位置参数必须全部在命名参数之前。支持的类型转换（`ArgValue` → 目标类型）在 lossless 范围内自动进行，并输出 warn 日志。

### `InboundMessage` — 入站消息

```rust
pub struct InboundMessage {
    pub id: u8,                    // 消息命令 id（注册时指定）
    pub params: Vec<ArgValue>,     // 解码后的参数值
}
```

## 快速开始

```rust
use std::sync::Arc;
use klipperx::core::klippy::msg::Parser;
use klipperx::core::klippy::msg::param::Param;
use klipperx::core::klippy::msg::proto::ArgValue;
use klipperx::core::klippy::traits::KlippyInterface;

// 1. 创建 Parser（需要实现 KlippyInterface 的实例）
let interface: Arc<dyn KlippyInterface> = /* ... */;
let mut parser = Parser::new(interface);

// 2. 注册命令格式（id, 格式字符串）
parser.register(3, "G1 X=%u Y=%u").unwrap();

// 3. 发送命令
parser.send("G1", &[
    Param::Positional(ArgValue::UInt32(100)),
    Param::Positional(ArgValue::UInt32(200)),
]).await.unwrap();

// 4. 启动 inbox 接收入站消息
let mut rx = parser.start_inbox().await.unwrap();

// 5. 接收未经回调绑定的消息
while let Some(msg) = rx.recv().await {
    println!("收到 cmd_id={} 参数={:?}", msg.id, msg.params);
}
```

## API 参考

### `Parser::new(interface: Arc<dyn KlippyInterface>) -> Self`

创建一个新的 `Parser`，自动注册内置的 identify 消息格式：
- id=0: `identify_response offset=%u data=%.*s`
- id=1: `identify offset=%c count=%c`

### `Parser::register(&mut self, id: u8, format: &str) -> MsgResult<()>`

注册一个消息格式。格式字符串是空格分隔的 token，首 token 为命令名，后续为 `name=type` 对。

支持的类型说明符：

| 类型 | 含义 |
|------|------|
| `%u` | uint32 |
| `%i` | int32 |
| `%hu` | uint16 |
| `%hi` | int16 |
| `%s` / `%*s` / `%.*s` | 字符串 |
| `%c` | 字节数组 |

**示例**：
```rust
parser.register(5, "M105")?;                         // 无参数
parser.register(3, "G1 X=%u Y=%u")?;                 // 两个 uint32 参数
parser.register(9, "TEST name=%s data=%c")?;          // 混合类型
```

### `Parser::bind(&mut self, cmd_name: &str, callback: impl FnMut(&[ArgValue]) + Send + 'static) -> MsgResult<()>`

为已注册的命令绑定入站回调。绑定后此命令**不能再用于 `send`**。

```rust
// 所有入站 "temp_report" 消息都进入 callback_queue，而不是 inbox channel
parser.bind("temp_report", |values| {
    println!("温度报告: {:?}", values);
})?;
```

### `Parser::send(&self, cmd_name: &str, params: &[Param]) -> MsgResult<()>`

发送一个命令。支持位置参数和命名参数。

**位置参数**（按顺序）：
```rust
parser.send("G1", &[
    Param::Positional(ArgValue::UInt32(100)),  // X
    Param::Positional(ArgValue::UInt32(200)),  // Y
]).await?;
```

**命名参数**（任意顺序）：
```rust
parser.send("G1", &[
    Param::Named("Y".to_string(), ArgValue::UInt32(200)),
    Param::Named("X".to_string(), ArgValue::UInt32(100)),
]).await?;
```

**混合使用**：
```rust
parser.send("G1", &[
    Param::Positional(ArgValue::UInt32(100)),   // X（位置）
    Param::Named("Y".to_string(), ArgValue::UInt32(200)),  // Y（命名）
]).await?;
```

**常见错误**：
- 命令未注册 → `Unknown command`
- 命令已绑定回调 → `Cannot send Handler type`
- 位置参数出现在命名参数之后 → `Positional param after named param`
- 命名参数重复 → `Duplicate named param`
- 位置参数过多 → `Too many positional params`
- 未知的命名参数名 → `Unknown param`
- 同一参数同时提供位置和命名 → `provided both positionally and by name`
- 缺少必填参数 → `Missing required param`
- 类型不匹配且无法转换 → `Param type mismatch`

### `Parser::send_and_wait(&self, cmd_name: &str, params: &[Param], wait_name: &str, timeout: Option<Duration>) -> MsgResult<Vec<ArgValue>>`

发送命令后等待特定入站消息返回。waiter 在发送前注册，确保不会错过快速响应。

```rust
let params = parser
    .send_and_wait("M105", &[], "temperature", Some(Duration::from_secs(1)))
    .await?;
// params 即为 "temperature" 消息解码后的参数值
```

**路由优先级**：`send_and_wait` 的 waiter 优先级高于 `bind` 注册的回调。如果等待的消息恰好也绑定了回调，消息会投递给 waiter 而非 callback_queue。

### `Parser::take_callback_msgs(&self) -> MsgResult<Vec<InboundMessage>>`

同步取出所有已绑定的回调消息（非阻塞，立即返回，可能为空）。

```rust
let msgs = parser.take_callback_msgs()?;
for msg in msgs {
    println!("bound cmd_id={} params={:?}", msg.id, msg.params);
}
```

通常在主循环中定期轮询，或在 inbox 消息处理循环中穿插调用。

### `Parser::start_inbox(&mut self) -> MsgResult<mpsc::Receiver<InboundMessage>>`

启动 inbox 后台任务，持续接收并解析入站消息。返回一个 `Receiver` 用于消费 **未绑定回调** 的消息。

```rust
let mut rx = parser.start_inbox().await?;
while let Some(msg) = rx.recv().await {
    handle_message(msg.id, &msg.params);
}
```

**注意**：绑定回调的消息（通过 `bind`）不会进入此 channel，而是进入 `callback_queue`，通过 `take_callback_msgs()` 取出。

---

- [← 开发手册首页](README.md)
- [消息结构 →](message-structure.md)
