# 内部架构

本文描述 `msg` / `mcu` 两层实际运行的机制：帧如何被合并发出、入站消息如何路由、请求/响应如何配对。

## 总览

```
                      ┌─────────────── feature ───────────────┐
                      │ call_msg / send_msg                   │
                      └───────────────┬───────────────────────┘
                                      ▼
   ┌──────────────────────────── Mcu ────────────────────────────┐
   │ Parser（共享注册表）          Dictionary（消息表/枚举/常量）  │
   │ send() ─► mpsc<Payload>       PendingCalls（请求响应配对）    │
   │ call() ─► 注册 PendingCall                                    │
   └──────┬───────────────────────────────────▲──────────────────┘
          │                                   │
   ┌──────▼───────┐                    ┌──────┴───────┐
   │ 发送任务      │                    │ 接收任务      │
   │ 合并批处理    │                    │ 校验/解码/路由│
   └──────┬───────┘                    └──────▲───────┘
          │ Frame                             │ Frame
          ▼                                   │
   ┌────────────── Interface（Device）────────────────────────────┐
   │ send() / receive() 均在 spawn_blocking 中执行                │
   └──────────────────────────────────────────────────────────────┘
```

一个 `Mcu` 启动两个 `tokio` 任务：发送任务消费 `mpsc<Payload>`，接收任务在 `interface.receive()` 上循环。`Parser` 的 `clone()` 共享同一份注册表，字典因此可以在任务启动之后再装载。

## 发送侧：合并批处理

`Mcu::send` 只做编码与入队（`try_send`，容量 32），实际出站在发送任务里：

1. 阻塞等待至少一个 `Payload`；
2. 若当前 payload 已达到 `MESSAGE_PAYLOAD_MAX * 2 / 3`（约 39 字节），立即发送；
3. 否则打开 **1ms 窗口**：窗口内到达的 payload 用 `Payload::try_merge` 逐个合并；
4. 合并会导致超限时，先 flush 当前批次，再以新 payload 重新开批；
5. `send_batch` 用当前序号发帧，成功后 `seq = (seq + 1) & 0xf`。

**目的**：减少高频小消息（温度报告之类）的 IO 次数，同时不给大消息增加额外延迟——大 payload 走立即发送分支。

序号只占低 4 位（`MESSAGE_SEQ_MASK = 0x0f`），发满 16 批后回绕。

## 接收侧：校验与路由

```
Frame ─► seq 校验 ─► Parser::decode ─► 逐条消息路由
           │不匹配则丢弃并 warn         │
           │                            ├─ 1. PendingCalls 命中 → oneshot 投递，跳过回调
           │                            ├─ 2. 有绑定回调 → 调用回调
           │                            └─ 3. 都没有 → warn 并丢弃
```

- `seq` 与本地计数器（`seq & 0xf`）不一致时告警并丢弃该帧，计数器仍然递增。
- `Parser::decode` 失败只记录日志并跳过该帧，不影响后续帧。
- 路由优先级与 Klipper 一致：同步等待方优先于回调，因此一条消息要么投给 `call`，要么投给回调，不会两者都收到。

## 请求/响应配对：`PendingCalls`

`Mcu::call` 的步骤：

1. 校验命令已注册（未注册返回 `CommandNotFound`；已绑定回调只告警）；
2. `register`：把 `(response_name, oneshot::Sender)` 放进 `PendingCalls`；
3. `send` 失败 → `cancel` 并返回 `SendFailed`；
4. `timeout(rx)` 等待；
5. 成功由接收任务 `resolve` 完成（此时登记项已被消费，无需清理）；接收端意外关闭或超时则 `cancel`。

匹配规则是**按响应名**、**先到先得**，`resolve` 只在命中时才克隆参数，所以普通异步消息（未命中路径）没有任何额外分配。

已知限制：两个并发调用等待同一个响应名时会互相抢答。Klipper 用 `oid` / command queue 区分，本实现尚未引入，需要时应把区分参数加入 `PendingCall` 并参与匹配。

## 数据字典的装载时机

`Mcu::from_parts` 只注册 identify 一对；握手（`feature::identify`）完成后 `install_dictionary` 把命令与响应注册进 `Parser`。之所以不需要重启接收任务：

```rust
let parser_for_task = parser.clone();   // 同一个 Arc<Mutex<MsgMap>>
```

`register` 只要求句柄的 `&mut`，因此 `install_dictionary(&self)` 里用一个 `clone()` 就能在 `&self` 下完成注册，且对接收任务立即可见。

`Dictionary` 本身放在 `std::sync::Mutex<Option<Arc<Dictionary>>>` 里：只在极短的临界区读写，且 `send_msg` 需要在同步上下文里检查它。

## 错误分层

| 错误 | 层次 | 说明 |
|------|------|------|
| `MsgError` | 编解码 | 未知消息名 / id、参数个数或类型不符、payload 超限 |
| `McuCallError` | 单次调用 | `CommandNotFound` / `CommandHasCallback` / `SendFailed` / `Timeout` |
| `McuError` | MCU 操作总括 | 包装上述两者，另加 `Dictionary` / `UnknownMessage` / `Decode` / `NotIdentified` / 三个 identify 变体 |

`McuError::source()` 会继续向下暴露 `MsgError` / `McuCallError`，便于 `anyhow` 之类的链式错误输出。

## 已实现与未实现

已实现：帧收发与校验、合并发送、同步请求/响应、identify 握手与字典安装（`feature::identify`）、类型化消息与按名取参（`mcu::codec`）、`feature::clock`。

未实现（详见 [MCU 协议与数据字典](mcu-protocol.md#当前未实现)）：事件 / 异步 `output` 投递、并发同名响应的区分、枚举参与 `ArgType` 编解码、命名参数。

`msg` 层也没有 `Default for Parser`：`Parser::new()` 是空注册表，identify 消息由 `Mcu` 显式注册，避免编解码层反向依赖具体协议。

## 性能特性

- **出站批处理**：高频小消息合并进一帧，显著减少 IO 次数与帧头开销。
- **入站无字符串比较**：`Parser::decode` 按 id 查表（`HashMap<i16, _>`），不涉及名字比较。
- **未命中路径零分配**：`PendingCalls::resolve` 只在真正配对成功时克隆参数。
- **锁范围最小**：`MsgMap` 的锁不跨 `.await`；`PendingCalls` 的锁只在单次操作内持有。
- **主要开销**：每条入站消息都会分配一个 `Vec<ArgValue>`（以及字符串/字节参数自身的分配）。这是当前最大的内存开销，与 Klipper 主机端相比仍属可接受范围。

---

- [← 开发手册首页](README.md)
- [Identify 机制 ←](identify.md) · [测试 →](testing.md)
