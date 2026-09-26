# 内部架构

本文描述实际运行的机制：帧如何被合并发出、入站消息如何路由、请求/响应如何配对。主角是传输层 `mcu`（帧、`Parser`、字典、请求响应配对）；它上面的 `cmd`（命令模块）与 `identify`（引导交换）是同级模块，只通过 `Mcu` 的公开/ crate 内部接口使用它。

## 总览

```
                     ┌─────────────── cmd ────────────────┐
                     │ call_msg / send_msg                │
                     └───────────────┬────────────────────┘
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
  └───────────────────────────────────────────────────────┘
```

一个 `Mcu` 启动两个 `tokio` 任务：发送任务消费 `mpsc<Payload>`，接收任务在 `interface.receive()` 上循环。`Parser` 的 `clone()` 共享同一份注册表，字典因此可以在任务启动之后再装载。

## 传输层：`Interface` / `Device`

`Mcu` 只认 `send(Frame)` / `receive() -> Frame`（`interface::Device`）；字节怎么走是下面几个实现的事：

| 实现 | 配置键 | 线上是什么 |
|------|--------|-----------|
| `SerialDevice` | `serial:` | tty 上的字节流 |
| `CanSerialDevice` | `canbus_uuid:` + `canbus_interface:`（+ `canbus_nodeid:`） | SocketCAN，**承载的仍是同一份 serial 字节流** |
| `HostDevice` | `host_library:` | `dlopen` 的 klipper host 库，输入/输出都是协议字节 |
| `FrameMock` | `test:`（仅测试构建） | 脚本化应答 |

**CAN 目前不是把 Klipper 协议「放在」CAN 上**，而是 Klipper 的 can-serial：固件把本该写到串口的字节流原样每 8 字节一段塞进经典 CAN 帧的 8 个数据字节，仲裁 id 只负责寻址（`0x100 + 2*nodeid`，回包 +1）。这里的「串口字节流」就是 MCU 的帧格式——长度、序号、载荷、CRC、`0x7e` SYNC（`src/core/klippy/frame.rs`）——CAN 在这里只是一根更慢的串口线，**不参与分帧**：消息块的头尾仍由这套 serial 格式决定，重组用的是同一个 `FrameStream`。

两个直接后果：

- 改 CAN 不需要另一套协议代码，只需要那个 `0x100 + 2*nodeid` 的映射（单测覆盖它，socket 层只经过编译，见 [测试](testing.md)）；
- TRACE 日志里 CAN 打两种行：**分片行**每个 CAN 帧一行（`rx can frame [can0:0x30a]: id 0x30b data 01020304 05060708`，仲裁 id + 8 个数据字节，就是总线上看到的东西），**整块行**按 serial 帧格式打一次（`rx frame [can0:0x30a]: 0a11 | 01020304 05 | 31d87e`）。`rx` 先逐片打分片行、凑齐后再打整块行；`tx` 先打整块行（拆片之前）、再逐片打分片行。**整块只占一个 CAN 帧时两种行也都会打**——总线上的分片与协议里的块不是一回事。

### 开启 TRACE

帧字节（`tx/rx frame`、`rx/tx can frame`）打在 `TRACE` 上，按模块开就行：

```bash
RUST_LOG=klipperx=trace klipperx ~/printer.cfg --tui
```

`--verbose` 单独用只看得到 `debug`；和 `RUST_LOG` 一起用时取更详细的那个，所以上面这条命令照样是 `trace`。各级别与各类行的完整例子见用户手册的[日志与调试](../user-manual/logging.md)。

## 发送侧：合并批处理

`Mcu::send` 只做编码与入队（出站通道 `SEND_QUEUE_CAPACITY` = 512，队列满时**有界退避等待** ≤ `SYNC_SEND_WAIT` 后仍满才报错；`send_payload` 剩余 ≤16 格 `SYNC_SEND_HEADROOM` 时让位等排空，给同步 `send` 常备余量），实际出站在发送任务里：

1. 阻塞等待至少一个 `Payload`；
2. 若当前 payload 已达到 `MESSAGE_PAYLOAD_MAX * 2 / 3`（约 39 字节），立即发送；
3. 否则打开 **1ms 窗口**：窗口内到达的 payload 用 `Payload::try_merge` 逐个合并；
4. 合并会导致超限时，先 flush 当前批次，再以新 payload 重新开批；
5. `send_batch` 用当前序号发帧，成功后 `seq = (seq + 1) & 0xf`。

**目的**：减少高频小消息（温度报告之类）的 IO 次数，同时不给大消息增加额外延迟——大 payload 走立即发送分支。

序号只占低 4 位（`MESSAGE_SEQ_MASK = 0x0f`），发满 16 批后回绕。

## 接收侧：校验与路由

```
Frame ─► seq 校验 ─► 空帧（ack）跳过 ─► Parser::decode ─► 逐条消息路由
           │不匹配则丢弃并 warn                              │
           │                            ├─ 1. PendingCalls 命中 → oneshot 投递，跳过回调
           │                            ├─ 2. 有绑定回调 → 调用回调
           │                            └─ 3. 都没有 → warn 并丢弃
```

- 序号是**按块（block）**而不是按帧校验的。固件处理一个请求块时发出的每一帧都盖同一个序号：一条响应一帧，外加一帧**空载荷的 ack**（它只用来推进序号）。所以接收侧接受两种序号——「正在等的这个块」（同一请求的多条响应、ack、以及固件主动发的 output）与「下一个块」（我们随后发的那一条的响应）；其余（明显落后或超前的）告警并丢弃。Klipper 客户端用发送窗口做同一件事；本实现把已发未确认的块留在 `Sender::in_flight`（上限 `MAX_PENDING_BLOCKS`），收到 nak（重复的 ack 号）或 RTO 定时器到期就把它们原号重发（`mcu/mod.rs`）。
- 空载荷帧就是 ack：直接跳过，不进解码。
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

`Mcu::new`（内部 `from_parts`）拿到的注册表只认识 identify 一对（由 `identify::new_parser` 构造）；`Mcu::connect`/`Mcu::identify`（都在 `identify.rs`）完成后 `install_dictionary` 把命令与响应注册进 `Parser`。之所以不需要重启接收任务：

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

网络层之上还有一层**主机错误词汇**，它决定错误把机器带到哪个状态（`core/klippy/error.rs`）：

| 错误 | 谁造成 | 结果 |
|------|--------|------|
| `gcode::CommandError` | 客户端的 G-code | 拒绝这一行（`!!`），不停机 |
| `ConfigError` | 配置文件 | `error` 状态（可 `RESTART`），不触发 shutdown |
| `KlippyError::{Connection,Protocol,Request,Parse,Config}` | MCU / 链路 / 连接期配置 | `error` 状态（MCU 错误先 `klippy:notify_mcu_error`） |
| `KlippyError::Internal` | klippy 自身 | `invoke_shutdown`（`shutdown` 状态），状态已不可信 |

两条兵兵底线：G-code handler 的 panic 与 API endpoint 的 panic 都被 `catch_unwind` 兜住，
前者报 `Internal error on command:"X"`、后者报 `Internal Error on WebRequest: <method>`，两者都
`invoke_shutdown`（上游 `gcode.py:230-234`、`webhooks.py:271-276`）。

## 已实现与未实现

已实现：帧收发与校验、合并发送、同步请求/响应、identify 握手与字典安装（`identify`）、类型化消息与按名取参（`cmd`）。`cmd/` 下的命令模块已经就位：`allocate_oids`、`get_config` / `finalize_config`、`get_uptime`、`emergency_stop` / `clear_shutdown`（`basecmd.c` 的四个分区）、`clock`（`ClockSync` / `McuClock`，读固件时钟）、`gpio` / `pwm` / `adc`、`stepper` / `endstop` / `trsync`、`spi` / `i2c` / `thermocouple` / `ds18b20`、`debug`，以及引导用的 `identify` 一对（各模块与固件源文件的对应见[开发手册首页](README.md)的模块表）。事件消息也走上回调投递：`Mcu::bind_event` 把处理器绑到某条响应上，接收任务在无待配对调用时调用它（`event/stats.rs` 的 `stats` 是第一个）。

未实现（详见 [MCU 协议与数据字典](mcu-protocol.md#当前未实现)）：`output` 表的异步投递（`Dictionary` 已解析但不注册，回调只对 `responses` 里的消息生效）、并发同名响应的区分、枚举参与 `ArgType` 编解码、命名参数。

`msg` 层也没有 `Default for Parser`：`Parser::new()` 是空注册表，identify 消息由 `identify::new_parser` 显式注册（`Mcu::new` 取用），避免编解码层反向依赖具体协议。

## 性能特性

- **出站批处理**：高频小消息合并进一帧，显著减少 IO 次数与帧头开销。
- **入站无字符串比较**：`Parser::decode` 按 id 查表（`HashMap<i16, _>`），不涉及名字比较。
- **未命中路径零分配**：`PendingCalls::resolve` 只在真正配对成功时克隆参数。
- **锁范围最小**：`MsgMap` 的锁不跨 `.await`；`PendingCalls` 的锁只在单次操作内持有。
- **主要开销**：每条入站消息都会分配一个 `Vec<ArgValue>`（以及字符串/字节参数自身的分配）。这是当前最大的内存开销，与 Klipper 主机端相比仍属可接受范围。

---

- [← 开发手册首页](README.md)
- [Identify 机制 ←](identify.md) · [测试 →](testing.md)
