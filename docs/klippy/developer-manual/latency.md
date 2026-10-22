# 延迟与抖动（主机侧）

主机是**软实时**：步进脉冲由 MCU 自己按 `queue_step` 发，主机只负责把 move **提前**送进 MCU 的
步进队列（见 [时钟与定时器](reactor.md) 与 [运行时编排](runtime.md)）。所以这里的目标不是
「零抖动」，而是**别让 MCU 队列见底**：MCU 队列几百毫秒到秒级的缓冲就是余量，主机偶尔晚
几毫秒无所谓，持续晚到队列排空才是问题。

本文列出抖动可能从哪来，以及**要不要绑核**。结论先给：**现在不需要**，先度量再说——理由和
代价见下。

## 先度量

A1b 的延迟度量（`Reactor::set_latency_notifier`）给出每个分发轮次的两个数：

- `busy`：从该轮**最早能跑的时刻**到轮次结束（每个回调取「唤醒时间」与「它被武装的时刻」中较晚的那个）—— 把「唤醒晚了」和「回调慢」放在同一条线上，但**不**把「武装到过去」的定时器算进来（上游 `idle_timeout` 每次归零都这么武装，见 [reactor](reactor.md) 的“延迟度量”）；
- 每个回调的 `name` / `duration` / `lateness`。

主机在 `src/klippy.rs` 挂了一个 50 ms 阈值的回调，超了就 `warn!`（上游
`extras/garbage_collection.py` 的 `THRESHOLD` 也是 50 ms）。要判断抖动是不是问题、要不要做
下面的优化，先看这条日志：**是哪个回调、晚多少、忙多久**。没有数据就别先绑核。

### 不超阈值也要有数：`system_stats.dispatch`

50 ms 的阈值比我们要看的抖动高好几个数量级，所以「没看到告警」不等于「没抖动」。
因此每个分发轮次**不管超没超**都在 `system_stats.dispatch` 里累加（无锁原子量，热路径不加锁）：
`rounds` / `busy_avg_s` / `busy_max_s` / `lateness_avg_s` / `lateness_max_s` / `over_threshold` / `threshold_s`。
字段定义见[对象状态说明](../third-party-dev/api-reference.md#system_stats)。

拿到它就有可比的数：

```bash
klipperx api -a tcp:127.0.0.1:7125 objects/query '{"objects":{"system_stats":["dispatch"]}}'
```

### 两个可重跑的测量脚本

宿主侧与固件侧分开量，两个脚本都自带 `--help`（单位、判据、示例写在里面）：

- `scripts/kx-sched.py` —— **宿主线程的调度等待**：两次读 `/proc/<pid>/task/*/{schedstat,sched}`
  取差值，`mean_wait_us` = 「每次被调度前在运行队列里等了多久」。它量的是 OS 那一半。
- `scripts/stats-jitter.py` —— **固件每轮任务耗时**（从 `klippy.log` 的 `stats count=/sum=/sumsq=`
  行算 avg/sd/awake）。注意它量的**不是**唤醒迟到，别混用。

两个都在仓库根的 venv 里跑（见 [`python-tools`](../../../.agents/skills/python-tools/SKILL.md)）：

```bash
venv/bin/python scripts/kx-sched.py --pid $(pgrep -f klipperx | head -1) --interval 10
venv/bin/python scripts/stats-jitter.py ~/printer_data/logs/klippy.log
```

### 本机实测基线（2026-10-20，供对照）

固件侧（`klippy.log` 1850 份 5 s 报告，7.7 小时）：每轮任务耗时 **6.4 µs**（180 MHz 那侧）/ **15.9 µs**
（72 MHz 那侧），跨 10 分钟分桶不漂。宿主侧（真机会话，8 路 `gcode/script` + 8 路 `info` 压着）：

| 环境 | `klippy-mcu` worker 平均运行队列等待 |
|---|---|
| 32 核宿主 | 0.09 µs |
| 2 核（`taskset -c 0,1`） | 0.08 / 9.28 µs（两个 worker 里只有一个吃亏） |
| ≈RK3328（4 核 + `CPUQuota=25%`） | 0.08 / 5.60 µs |

读法：**卡片机量级的调度抖动是个位数 µs**，与固件自身 6.4 µs 同量级、距步进允许的 1 ms
（`third_party/klipper/src/stepper.c` 的过期判定）还有两个数量级。所以优先级应该给「不阻塞回调」
与「不把阻塞 IO 放在 worker 上」，而不是加线程。

### 应用层的那一半：`dispatch` 实测

上面那张表是**内核调度**那一半。`system_stats.dispatch` 量的是另一半（从定时器到期到回调真开始跑，
含 tokio 唤醒与轮内排队）。无硬件宿主上连续采样 35 s（`rounds` 13 → 32 → 92）：

| 量 | 实测 |
|---|---|
| `busy_avg_s` | **1.16–1.30 ms** |
| `busy_max_s` | 2.18–2.33 ms |
| `lateness_avg_s` | **1.13–1.27 ms**（≈ `busy_avg`：几乎整轮都算「晚开始」） |
| `lateness_max_s` | 2.30 ms |
| `over_threshold` | 0 / 92 轮（阈值 50 ms） |

这比线程调度那半（亚 µs）**大三个数量级**，原因不在调度而在休眠机制：dispatcher 用
`tokio::time::sleep(remaining)` 等到下一个唤醒时刻，而 tokio 的定时器时间轮是 **ms 级粒度**，
所以「到期 → 真醒」天然差 1–2 ms。这是推断，但三个观测同时成立：lateness 与 busy 几乎相等
（一轮里所有回调都晚启动同样多，而不是某个回调慢）、量级落在 1–2 ms、且与负载无关。

**要不要紧？** 不要紧，但要知道边界：主机只需把 move **提前**送进 MCU 的步进队列，固件那一侧
有几百 ms 缓冲（[与打印时序的关系](reactor.md#与打印时序的关系)）。1–2 ms 的唤醒粒度落在缓冲里，
真机上也没出现过由此引起的停机（50 ms 阈值的告警在真机会话里是 0 条）。若以后真要亚 ms 级唤醒，
改的是**休眠方式**（用 `poll`/`clock_nanosleep` 按绕对时刻休眠、绕开时间轮），不是绑核。

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
   次级 MCU 的 `mcu_recalibrate`（1 s）、全部 MCU 的 `mcu_clock_poll`（1 s，喂时钟估计）、发送任务的 `GATE_REPOLL`（10 ms 分片轮询，压队中的时钟门控消息到点放行、容忍 `set_clock_base` 重锚）、`temperature_combined` 的阈值守卫，加上 fan 的一次性
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
   一次真例：2026-10-22 真机每次固件重置后的首次 `G28 X`，那条 `Reactor busy for 0.148s`
   里 `call_later ran 0.000s late, took 0.148s` —— 名字和耗时直接指到 TMC 使能的寄存器突发；把它拓到
   blocking 池（[reactor](reactor.md#回调里不许等待不许做重活)）后同一流程零告警。注意告警里的
   `busy` 不能只看数字：**「武装到过去」的定时器也会算出很大的 busy 而 reactor 一秒没卡**，
   认那个回调的 `took`（真阻塞）还是 `lateness`（被前一轮拖到）。
2. **开会话时别常开 `debug` 日志**（第 3 条）——`RUST_LOG=debug` 本身会引入抖动。
3. **看 `busy`/`lateness` 的数据再决定**（第 4–6 条大多是 ms 以下，够用）。
4. **A3 已完成**（第 6 条）：机器与 API 分了 runtime。
5. 只有 1–4 都做过、数据仍差，才值得动 7/11/14 的核。

---

- [← 开发手册首页](README.md)
- [运行时编排 ←](runtime.md) · [时钟与定时器 ←](reactor.md)
