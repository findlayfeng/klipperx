# 内部架构与性能特性

## 架构概览

`Parser` 是一个消息通信引擎，包含三个核心子系统：

```
                    ┌───────────────────────────┐
                    │       Parser              │
                    │                           │
  send() ──────────►│  Outbound Coalescing      │──► KlippyInterface::send()
                    │  (run_sender task)        │
                    │                           │
                    │  Inbound Routing          │◄── KlippyInterface::receive()
                    │  (run_inbox task)         │
                    │   ├─ waiter (send_and_wait)│
                    │   ├─ callback_queue (bind) │
                    │   └─ inbox channel (default)│
                    │                           │
                    │  Msg Registry             │
                    │  (MultiIndexMsgMap)       │
                    └───────────────────────────┘
```

## 关键设计决策

### 1. Outbound 合并发送（Coalescing）

`run_sender` 是一个后台 `tokio` 任务，通过 `mpsc` channel 接收 `OutItem`。

**策略**：
- 大 payload（≥ 2/3 `MESSAGE_PAYLOAD_MAX`）立即发送，不等待
- 小 payload 打开一个 **1ms 窗口** 收集后续 payload
- 窗口内到达的 payload 通过 `Payload::try_merge` 合并
- 合并后超限时，先 flush 当前批次，再根据新 payload 大小决定策略

**目的**：减少高频小消息（如温度报告）的 IO 次数，同时不增加大消息的延迟。

### 2. Inbound 路由优先级

每个入站 payload 按以下顺序处理：

```
payload → pop cmd_id → 按 cmd_id 查找 Msg 定义
    │
    ├─ 1. waiter 匹配
    │   （send_and_wait 注册的 PendingWaiter）
    │   → 通过 oneshot 直接返回 params
    │
    ├─ 2. 有回调绑定（MsgEntry::Handler）
    │   → push 到 callback_queue
    │
    └─ 3. 默认路由（MsgEntry::Base）
        → 发送到 inbox channel (mpsc)
```

### 3. PendingWaiter 设计

```rust
struct PendingWaiter {
    id: u64,          // 单调递增唯一标识（用于精确移除）
    msg_id: u8,       // 等待的消息命令 id
    tx: oneshot::Sender<Vec<ArgValue>>,
}
```

- `msg_id` 在 `send_and_wait` 调用时通过 `get_by_name` 一次查找确定
- `process_single` 中通过 `w.msg_id == cmd_id` 做整数比较匹配
- 失败或超时时通过 `remove_waiter(id)` 精确移除，避免影响相同 `msg_id` 的其他 waiter

### 4. 字段可见性

| 字段 | 类型 | 用途 |
|------|------|------|
| `msgs` | `MsgRegistry` (`Arc<Mutex<MultiIndexMsgMap>>`) | 消息注册表，按 id 和 name 双索引 |
| `interface` | `Arc<dyn KlippyInterface>` | 底层 IO 抽象 |
| `outbox` | `AsyncMutex<Option<mpsc::Sender<OutItem>>>` | 懒启动的发送通道 |
| `inbound_tx` | `AsyncMutex<Option<mpsc::Sender<InboundMessage>>>` | 懒启动的 inbox 发送端 |
| `callback_queue` | `Arc<Mutex<VecDeque<InboundMessage>>>` | 绑定回调消息队列 |
| `waiters` | `Arc<Mutex<Vec<PendingWaiter>>>` | 等待中的 send_and_wait |

## 性能特性

### 优势

- **inbound 路由无字符串比较**：`InboundMessage` 使用 `id: u8`，waiter 匹配使用 `msg_id == cmd_id` 整数比较，零堆分配
- **outbound 合并发送**：减少高频小消息的 IO 次数
- **锁范围最小化**：`process_single` 和 `send` 中锁的持有时间控制在 `.await` 之前

### 瓶颈

- **`params: Vec<ArgValue>` 堆分配**：每个入站消息的解码参数都需要分配 Vec，这是当前最大的内存开销
- **`param_defs.to_vec()` 克隆**：`process_single` 中需要克隆参数定义列表（`Vec<(String, ArgType)>`），因为锁不能在 `.await` 期间持有

---

- [← 开发手册首页](README.md)
- [消息结构 ←](message-structure.md) · [测试 →](testing.md)
