# 延迟与抖动（主机侧）

主机是**软实时**：步进脉冲由 MCU 自己按 `queue_step` 发，主机只负责把 move **提前**送进 MCU 的
步进队列（见 [时钟与定时器](reactor.md) 与 [运行时编排](runtime.md)）。所以这里的目标不是
「零抖动」，而是**别让 MCU 队列见底**：MCU 队列几百毫秒到秒级的缓冲就是余量，主机偶尔晚
几毫秒无所谓，持续晚到队列排空才是问题。

本文列出抖动可能从哪来，以及**要不要绑核**。结论先给：**现在不需要**，先度量再说——理由和
代价见下。

## 先度量

A1b 的延迟度量（`Reactor::set_latency_notifier`）给出每个分发轮次的两个数：

- `busy`：从该轮**最早的唤醒时间**到轮次结束 —— 把「唤醒晚了」和「回调慢」放在同一条线上；
- 每个回调的 `name` / `duration` / `lateness`。

主机在 `src/klippy.rs` 挂了一个 50 ms 阈值的回调，超了就 `warn!`（上游
`extras/garbage_collection.py` 的 `THRESHOLD` 也是 50 ms）。要判断抖动是不是问题、要不要做
下面的优化，先看这条日志：**是哪个回调、晚多少、忙多久**。没有数据就别先绑核。

## 可能的来源

按「离主机多远」排，从近到远。量级只是数量级，不是实测值。

| # | 来源 | 机制 | 典型量级 | 谁能改善 |
|---|---|---|---|---|
| 1 | dispatcher 里的慢回调 | 串行设计的直接代价：一个回调慢，后面全排它后面 | 由代码决定 | 代码（不许阻塞/重活） |
| 2 | 阻塞误入回调 | `std::thread::sleep`、同步文件 I/O、等锁 | 由代码决定 | 代码 |
| 3 | 日志 | `warn!`/`debug!` 打到 stderr，stderr 有锁且可能是同步写；开着 `RUST_LOG=debug` 时更重 | µs – ms | 日志级别 / 通道 |
| 4 | tokio 定时器分辨率 | timer wheel 以毫秒为粒度，`sleep` 不会比这更准 | ~1 ms | 换驱动（见下） |
| 5 | 唤醒路径 | 定时器到期 → 唤醒 → 任务被 worker poll，中间要排队 | µs – ms | 减少同 worker 的活 |
| 6 | 同 worker 的 IO 事件 | socket/串口事件和定时器共用事件循环；`klippy-api` 与 `klippy-mcu` 已分开，机器侧仍有 MCU 串口 | µs – ms | 已分 runtime（A3） |
| 7 | OS 调度 / CPU 竞争 | 机器 worker 被别的进程抢 CPU、被迁移到忙核 | 空闲核数十 µs，满载可达 ms | CPU 隔离 / 绑核 |
| 8 | blocking pool 压力 | 设备 `send`/`receive` 每次一个 `spawn_blocking`；每个 MCU 的 receive 是常驻阻塞读 | 线程数、调度开销 | 已按 MCU 数控制 |
| 9 | cgroup/容器限流 | CPU quota 用尽时整组被冻结到下一个周期 | 可到 ms 级 | cgroup 配置 |
| 10 | 频率 / C-state | 降频或从深睡眠唤醒要时间 | 数十 – 数百 µs | governor / 关深睡 |
| 11 | 中断 | 网卡/定时器 IRQ 落在机器核上 | µs – ms | IRQ affinity |
| 12 | NUMA / cache | 任务在核间迁移，缓存与内存延迟变大 | 间接 | 绑核 / NUMA |
| 13 | 分配器 | 高压力下 malloc 的锁竞争、`munmap` | 通常 µs，压力下更高 | 少分配 / 换分配器 |
| 14 | API runtime 的 CPU 竞争 | A3 后不再同 runtime 排队，但仍在**同一批核**上抢 CPU | 取决于负载 | 绑核 / 隔离 |

注意第 14 条：**A3 的 runtime 分离解决的是「同一 runtime 内的任务排队」，不解决「同一批 CPU 核
的竞争」**。要再往前一步，就是 CPU 层面的事（第 7/11/14 条）。

MCU 通信本身的延迟（串口/CAN 往返、重传）不在主机这里，且 MCU 队列给了余量，通常不是抖动源。

## 要不要绑核

**现在不需要。** 理由：

1. **硬实时在 MCU**，主机晚几毫秒不丢步；MCU 队列的缓冲是设计的一部分。
2. **当前消费者很轻**：周期性的 reactor 回调只有几个——`objects/subscribe` 的 250 ms 刷新、
   次级 MCU 的 `mcu_recalibrate`（1 s）、`temperature_combined` 的阈值守卫，加上 fan 的一次性
   kick-start `call_later`；每个都正常远低于 1 ms。MCU 时钟同步也不要求亚毫秒精度。
3. **绑核的代价不小，且做一半比不做更糟**：只把线程 `sched_setaffinity` 到某几个核、却不隔离
   这些核（别的进程、IRQ、内核线程还在上面跑），会把抖动集中到这几核上，可能更差。
4. **tokio 的任务会在 worker 之间迁移**，所以「给任务绑核」在进程内做不到；能绑的是 worker
   线程，而实际起作用的是 **cpuset**（一组核）。

**什么时候才值得做**：主机接近 CPU 饱和（一台机器跑多个 printer、或与别的负载共机）、MCU
队列很小（极高步频 / 很短的 move）、或度量显示 `busy`/`lateness` 经常越过阈值。到那时，正确
做法不是单点 `taskset`，而是：

- 给机器 runtime 一组**专用（isolated）核**：`cpuset` cgroup 或 `isolcpus`，
- 把这些核上的 **IRQ 挪走**（`/proc/irq/*/smp_affinity`），
- governor 设 `performance`、关掉深 C-state，
- 每核一个 printer，别把两个机器 runtime 塞同一核。

**机制上怎么加**（未实现）：tokio 的 `Builder::on_thread_start` 可以在每个 worker 线程启动时
调 `libc::sched_setaffinity`，把 `klippy-mcu` 的 worker 绑到机器核集合上；或者更省事地把整个
进程放进 cpuset。两者都要先有「哪些核是机器的」这个配置，所以不是现在该做的事。

## 先做便宜的

在考虑绑核之前，按性价比做这些：

1. **别阻塞回调**（第 1/2 条）——这是设计约定，不是优化；`set_latency_notifier` 会指出违规者。
2. **开会话时别常开 `debug` 日志**（第 3 条）——`RUST_LOG=debug` 本身会引入抖动。
3. **看 `busy`/`lateness` 的数据再决定**（第 4–6 条大多是 ms 以下，够用）。
4. **A3 已完成**（第 6 条）：机器与 API 分了 runtime。
5. 只有 1–4 都做过、数据仍差，才值得动 7/11/14 的核。

---

- [← 开发手册首页](README.md)
- [运行时编排 ←](runtime.md) · [时钟与定时器 ←](reactor.md)
