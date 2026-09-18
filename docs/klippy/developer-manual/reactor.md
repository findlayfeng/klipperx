# 时钟与定时器（reactor）

`reactor` 是机器的时间来源与定时器表：模块问它"现在几点"（`monotonic`），或让它"过一会儿叫我"（`register_timer`）。它在 `Printer` 上（`Printer::reactor()`，对应上游的 `get_reactor()`，全树 121 处调用），是每个 `get_status(eventtime)` 的 `eventtime` 与每次延迟动作的共同来源。

本文说明它为什么长这样，以及与上游 `klippy/reactor.py` 的对应关系。上游参考实现：`klippy/reactor.py`（447 行）与 `klippy/chelper/pollreactor.c`。

## 上游有两个 reactor，我们只有一个

上游的"reactor"是两套东西：

| | Python 主 reactor | C pollreactor |
|---|---|---|
| 位置 | `klippy/reactor.py` | `klippy/chelper/pollreactor.c` |
| 线程 | klippy 主线程 | `serialqueue.c` 的后台线程（`serialqueue.c:776`） |
| 干什么 | 配置装载、事件、greenlet 协程、`pause` / `completion`、订阅定时器 | 串口 fd 收发 + `retransmit` / `command` 重传定时器（`pollreactor.c:129`） |
| 形状 | greenlet 调度器 | 固定容量数组的纯 C 回调 |

两者靠 completion + `async_complete` 跨线程握手：C 线程收完一帧，把结果投递回主 reactor。

**我们只需要主 reactor 里"时间 + 定时器"那半。** C pollreactor 没有对应物 —— 串口那侧已经是 `mcu` 的 tokio 收发任务，重传/超时由 `pending` 的 oneshot 与 `tokio::time::timeout` 表达，不需要第二个事件循环。而主 reactor 里真正与"调度"有关的另一半（`pause` / `completion` / greenlet）在 async/await 世界里由 Future 直接表达，`reactor` 一个字节都不必碰。

## 上游的复杂度在哪里

`SelectReactor` 的主体是：

- `_timers` 定时器表 + `_next_timer` 最小唤醒时间，决定 `poll()` 的超时；
- `_dispatch_loop`（`:326`）与 `run()`（`:358`）主循环；
- `pause(waketime)`（`:227`）：把当前 greenlet 挂到一个内部定时器上，到期再切回来；
- `_g_dispatch` / `_end_greenlet`（`:253`）与 `ReactorGreenlet`：greenlet 的调度与缓存；
- `ReactorCompletion` / `completion()` / `wait()`（`:185`）：一次性事件；
- `ReactorMutex`（`:271`，greenlet 互斥）、`assert_no_pause`（`:265`）、`register_async_callback`（`:191`）、self-pipe、idle / latency 钩子。

**其中只有第一项与"时间"有关，其余都是"怎么在没有 async 的语言里写等待"。** 上游靠 greenlet 把 `completion.wait()` 和 `pause()` 写成同步阻塞的样子；`_check_timers` 里 `t.waketime = t.callback(eventtime)` 这个返回值契约（`:166`）才是定时器本身。

换到 async/await 之后：

| 上游 | 我们 |
|---|---|
| `pause(waketime)` | `sleep_until(...).await` |
| `completion()` / `complete()` / `wait()` | oneshot channel / `Future` |
| `_g_dispatch` / `_end_greenlet` / `ReactorGreenlet` | 不存在（执行器的事） |
| `ReactorMutex` | 不存在（`async` 互斥或直接共用 `&self`） |
| `assert_no_pause` | 不存在；"这里不许 await" 由代码审查与注释守着 |
| `register_async_callback` / `async_complete` | 不存在；跨任务通信就是 channel |
| `register_timer` / `monotonic` | **保留**，见下 |
| `register_callback`（一次性） | `call_later`（`register_timer` 的默认方法） |
| `set_idle_notifier` / `set_latency_notifier` | 未做：等有消费者（gc / idle_timeout）再说 |

## 我们的形状

`src/core/klippy/reactor.rs`。核心是一个对象安全的 trait：

```rust
pub trait Reactor: Send + Sync {
    fn monotonic(&self) -> f64;
    fn register_timer(&self, callback: TimerCallback, waketime: f64) -> TimerHandle;
    fn unregister_timer(&self, handle: TimerHandle) { handle.cancel(); }
    fn call_later(&self, delay: f64, callback: OneShot) -> TimerHandle { … }
}
```

定时器回调沿用上游契约：`FnMut(f64) -> Option<f64>` —— 收到事件时刻，返回**下一次唤醒时间**，`None` 表示注销。上游用 `NEVER = 9999999999999999.` 当哨兵，我们用 `Option`。`NOW = 0.` 对应"唤醒时间已在过去"，实现会尽快跑。

**为什么是 trait**：机器不该拥有 runtime。`Printer` 拿的是 `Arc<dyn Reactor>`，从不提 tokio；谁建打印机，谁决定什么驱动时间。主机传一个建在自己 runtime 上的 `TokioReactor`；测试传一个能手动拨表的 `ManualReactor`，确定性且完全不需要 runtime —— 这也让 `printer.rs` 的单测不必起 tokio。

**为什么没有 `pause` / `completion`**：它们就是 `async`。上游 `register_callback` 返回一个可以 `wait()` 的 completion；我们凡是"等"的地方都写 `future.await`，`call_later` 只留给"不等、只让它在后面跑一下"的回调。

`TimerHandle` 只用于取消。**丢弃 handle 不会取消定时器**：它一直活到回调返回 `None` 或被 `unregister_timer`。句柄是可克隆的，但克隆不是"多一个订阅"，只是多一个取消入口。

### 两个实现

| 实现 | 用途 | 时钟 | 定时器 |
|---|---|---|---|
| `TokioReactor` | 主机 | tokio 的 `Instant`，起点是构造时刻 | 每次注册 spawn 一个任务：睡到唤醒时间、跑回调、按返回值续睡或结束；取消用 `Notify` 唤醒 |
| `ManualReactor` | 测试（也可用于单线程调用方） | 一个手动推进的 `f64` | 一张定时器表；`advance(delta)` 把钟拨到每个唤醒时间、逐个跑 |

`ManualReactor::advance(delta)` 的语义是"让钟走 `delta` 秒"，不是"跳到 `now+delta` 再跑一次"：它会**在途中每个唤醒时间停下**，所以一个 0.25 s 周期的定时器在 `advance(1.0)` 里跑 4 次，回调看到的是 0.25/0.5/0.75/1.0，而不是都看到 1.0。这与上游 reactor 逐步处理定时器的行为一致，也是订阅类测试想要的确定性。`TokioReactor` 在 `#[tokio::test(start_paused = true)]` 下配合 `tokio::time::advance` 使用，钟是 tokio 的暂停钟。

`TokioReactor` 对超远的唤醒时间分片睡眠（`MAX_TIMER_SLEEP` = 1 天），既保证不会被取消时卡住，也避免把 `f64` 秒直接换成 `Instant` 时溢出——上游的 `NEVER` 换算成秒有 3 亿年。

## 与上游的对应

| 上游 | 位置 | 我们 |
|---|---|---|
| `monotonic()` | `reactor.py:111` | `Reactor::monotonic`；`Printer::eventtime` 读它 |
| `register_timer(cb, waketime)` | `reactor.py:145` | `Reactor::register_timer`，回调返回值契约相同 |
| `unregister_timer` | `reactor.py:152` | `Reactor::unregister_timer` |
| `register_callback(cb, waketime)` | `reactor.py:187` | `Reactor::call_later`（一次性） |
| `pause(waketime)` | `reactor.py:227` | `.await` |
| `completion()` / `wait()` | `reactor.py:185` | oneshot / `Future` |
| `_check_timers` 的返回值契约 | `reactor.py:157-172` | `run_timer` 的循环 |
| `_dispatch_loop` / `run` | `reactor.py:326`、`:358` | 执行器 |
| C pollreactor | `pollreactor.c:129` | 无（串口是 tokio 任务） |

## 时钟的起点

`monotonic()` 单调、不受系统墙钟调整影响，**起点是 reactor 的构造时刻**（上游是开机时刻）。客户端只需要能分辨两次报告，近零的起点正合适；每个打印机各自一个 reactor，也各自一个起点。`Printer::eventtime` 与 `Printer::reactor().monotonic()` 是同一个值——机器不再自己记时间。

## 与打印时序的关系

打印的**硬实时在 MCU**：步进脉冲由固件用 `queue_step` 的 interval/count 自己发，主机只负责把 move **提前**送进 MCU 的步进队列。所以主机是**软实时**——只要不把队列喂空，主机的唤醒抖动不会直接变成丢步；MCU 队列的几百毫秒到秒级缓冲就是余量。reactor 的抖动只在导致队列见底时才成问题。

主机侧仍需的是**确定性**：上游 `_check_timers`（`:157-172`）在**一个线程上按唤醒时间、一次一个**跑回调，所以 printer 状态几乎不需要锁。现状的 `TokioReactor` 是「一个定时器一个 tokio 任务」+ 多线程 runtime，两个同时到期的回调可以被调度到两个 worker **并行**执行且不保证顺序。对当前的消费者（订阅、温度、时钟同步）无害；但在把 toolhead / trapq 这类运动状态交给定时回调之前，必须先改成**串行 dispatcher**（一个任务、最小堆、按唤醒时间出队）并补上延迟度量。这件事排在 TODO 的 **A1b**。

## 还没有的

- **周期性任务的"武装/解除"**：上游用 `register_timer(cb, NEVER)` 注册、`update_timer` 择机武装。我们没有 `update_timer`，因为当前没有消费者；`objects/subscribe` 在无订阅时直接 `unregister_timer`（上游 `webhooks.py` 也是这么收尾的），不需要它。
- **串行调度与 idle / latency 钩子**：现状的定时器是并发的 tokio 任务；串行 dispatcher、`set_latency_notifier`（上游 `reactor.py:316`）与 idle 钩子排在 TODO 的 **A1b**，等运动状态要进定时回调时补。
- **fd 事件**：API 层的 socket 由 tokio 管，MCU 的串口由 `mcu` 的收发任务管，机器层不需要 `register_fd`。

---

- [← 开发手册首页](README.md)
- [Identify 机制 ←](identify.md) · [测试 →](testing.md)
