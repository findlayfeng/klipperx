# 卡片机验收：宿主稳定性与抖动

给一台**卡片机**（RK3328 / RK3566 / CM4 这一级：4×Cortex-A53 + eMMC/SD）做一次有界体检：
进程会不会挂、线程会不会涨、内存会不会漏、API 响应有没有离群、宿主侧调度抖动有多大，
以及日志会不会因为盘慢而丢。

一条命令跑完，产出一份可以直接贴回来的 `report.md`：

```bash
cargo build --release                     # 卡片机上就地构建；慢是正常的
scripts/host-stability.sh                 # 默认：满配，不接真板
```

- `--quick`：空载 20 s、重启 15 次（约 1 分钟，够看清「有没有明显问题」）。
- `--config ~/printer_data/config/printer.cfg`：追加**只读**的真板检查（不归零、不加热、不动电机）。
- `--out DIR`：报告目录，默认 `./host-stability-<时间戳>/`。

**前置**：`target/release/klipperx`（脚本用 release，debug 的数字不能对比）与一个 python3
（优先用仓库的 `venv/bin/python`，没有就退到系统的；脚本只用标准库）。
不需要 `ss`/`lsof`/`jq`，也不占 7125（默认用自己的 7150 端口，跑完即释放）。

**二进制太旧会被当成失败，而不是静默通过。** 如果报告里出现
`system_stats answered without dispatch/dropped_logs`，说明手上的 `klipperx`
早于这个对象（查未知对象会回一个空对象，正好是「看起来没事」的形状）——
重新 `cargo build --release` 就是。脚本对这类缺失一律 FAIL。

## 它做什么

| 阶段 | 时长 | 干什么 | 硬判据（不过就 FAIL） |
|---|---|---|---|
| 0 指纹 | 秒 | CPU 型号、核数（按亲和性）、`cpu.max` 配额、内存、内核、commit、二进制大小 | 二进制不存在 → 直接停 |
| A 空载 | `--soak` 秒（默认 60） | 起一个**没有任何 MCU** 的宿主，只让 reactor 转 | 线程数不变；RSS 增长 ≤ 2 MiB；`dropped_logs` 不增；无超过 50 ms 的分发轮 |
| B API 时延 | ~15 s | 8 路 `gcode/script` + 8 路 `info` 压着跑，然后 100 次串行计时 | 0 次失败/超时；p50 ≤ 20 ms（含客户端进程启动）；p99 ≤ 100 ms 为建议值 |
| C 重启风暴 | `--storm` 次（默认 60） | 连打 `RESTART`，每次看线程数/RSS | 全部被接受；线程数不变；RSS 增长 ≤ 10 MiB；结束 15 s 内回到 `ready`；SIGTERM 5 s 内干净退出 |
| D 调度 | 与 B 重叠 15 s | `scripts/kx-sched.py` 采每线程运行队列等待 | `klippy-mcu` 最差均值 ≤ 100 µs（建议值） |
| F 盘停顿 | `--f-stall` 秒（默认 20） | `scripts/disk-stall.py`：顺序读写 MB/s、4 KiB 随机读 p50/p99/max，**再在持续写+fsync 压力下重测**（这才是廉价 TF 的失效形状：平均值好看，偶发几百 ms～秒级停顿）；然后用回放 4 万行 gcode 看 `file_position` 推进有没有空档 | 4 KiB 随机 ≥ 500 IOPS；受压时无读数 > 200 ms；回放不停顿且 `over_threshold = 0`（建议值） |
| F2 日志余量 | 与 A 同时采样 | 队列容量 1024 条 ÷ A 段实测行速率 = **这块盘能容忍多长的停顿而不丢日志** | 计算的数，不判 PASS/FAIL |
| E 真板 | ~15 s | 只有带 `--config` 才跑：`ready` + 两份 `last_stats` 差分 | 有字节流动（`bytes_read` 在涨）；`srtt` 出现为建议值 |

**为什么是这些判据**：

- 线程数「必须不变」而不是「大致不变」——本项目每个 `[mcu]` 的收发任务、写日志线程、
  blocking 池占用都是确定值；跑几百秒后多出一个线程就是泄漏（历史上出现过「每次重启多一条
  机器线程」的形态）。
- RSS 用「增幅上限 + 看是否递减」，不用绝对值：Rust 的分配器会留 arena，60 次重启涨几百 kB
  是收敛的；单调线性增长才是漏。
- `dropped_logs` 是异步日志队列满了的计数（见[日志](../user-manual/logging.md#日志是异步的)）。
  卡片机上它一旦非零，说明盘跟不上日志量——那会把「日志写给谁看」变成「日志有没有」。
- p50 含客户端进程启动，所以 20 ms 是宽松的；真正该盯的是 **p99 相对 p50 的倍数**，
  离群意味着某个 endpoint 或锁在特定时刻把 api runtime 卡住了。

## 三条卡片机特有的坑（都是实测出来的）

> **F 段为什么看 p99/max 而不用平均吞吐**：廉价 TF 的典型失效不是「慢」，是「偶发停顿」
> （控制器内部 GC / 映射表刷新）。平均 1200 IOPS 的卡完全可能在持续写几十秒后卡 1 s。
> 平均吞吐看不见它，`disk-stall.py` 的「持续写 + fsync 期间测随机读」就是专门把它逗出来。
> 判读：受压时读的 **max** 在 100 ms 级 ⇒ 只是慢，主进程仍然不受影响（日志丢弃计数会说话）；
> 上到秒级 ⇒ 打印回放会看到空档，但机器时间线仍不受影响（读在 blocking 池上）。
>
> **F2 的日志余量是算的，不是压的**：本宿主**不按 gcode 命令打日志**，所以不接真板、没有真打印时，
> 日志速率小到无法把 1024 条队列填满。报告里给的数是「队列容量 ÷ 实测行速率」，换成你自己的日志
> 速率就对（例：某张卡实测 41 B/行、340 B/s ⇒ ≈8 行/s ⇒ 能容忍 128 s 停顿）。要真测丢弃，得在
> 真板打印时把 `RUST_LOG` 开到 `debug`，那条路径的典型量级是 2000 行/s（≈180 KB/s）。

1. **cgroup 配额会把 worker 数压到 1。** `std::thread::available_parallelism()` 在 Linux 上跟着
   `cpu.max` 走：`CPUQuota=25%` 时报 **1**。API runtime 因此有 `max(2, …)` 的下限
   （[运行时编排](runtime.md)）；如果你在容器/`systemd-run --scope` 里跑，先看一下报告 0 段的
   `cgroup cpu.max` 与 A 段的线程数对不对得上。
2. **2 核时只有一个 worker 开始排队。** 限到 2 核（`taskset -c 0,1`）时实测两个 `klippy-mcu`
   worker 的等待是 **0.08 µs vs 9.28 µs**——不是均摊，是「谁抢到谁跑」。所以 D 段看的是**最差**
   那个线程，不是平均。
3. **唤醒粒度是 ms 级，不是 µs 级。** `system_stats.dispatch.lateness_avg_s` 在空闲宿主上是
   **1–2 ms**，来源是 `tokio::time::sleep` 的时间轮粒度，不是卡不卡。
   [延迟与抖动](latency.md#应用层的那一半dispatch-实测) 有这条的实测与为什么不影响打印。

## 本机基线（用来对照，不是判据）

同机（32×Zen4）实测，供判断卡片机的数字是否「离谱」：

| 阶段 | 结果 |
|---|---|
| A 空载 60 s | 线程 38 恒定、RSS +24 kB、`dropped_logs` 0、`busy_avg`/`lateness_avg` 0.83→1.10 ms |
| B API | 100 次 0 失败，p50 3 ms / p99 4 ms；同窗口 8 线程有活动、机器 runtime 占 1% 一个核 |
| C 60 次重启 | 60/60 接受、线程 38 恒定、RSS +624 kB（递减） |
| D 调度（满载） | `klippy-mcu` 最差均值 0.15 µs；`log-writer` 出现在表里（7 次调度 / 0.1 ms） |
| 限 2 核 / `CPUQuota=25%` | 最差 9.28 µs / 5.60 µs（同一台机器，用来模拟卡片机） |

卡片机上预期：A/C 两项应当与上面接近（线程数与实现相关，RSS 增幅会更大），
B 的 p50 会到几十 ms（客户端进程启动慢），D 会到几 µs 到几十 µs。

## 跑完怎么回报

把 `report.md` 贴回来即可；需要深挖时再看同级 `raw/`：

- `raw/klippy.log`：该次会话的宿主日志（`Reactor busy` 会出现在这里）；
- `raw/rtt.txt`：100 次串行请求的毫秒数（可以画分布）；
- `raw/sched.txt`：每线程调度统计；
- `raw/last_stats_first.json` / `_second.json`：E 段两次 `last_stats`（差分算速率）。

真板阶段只读——它不会让机器动起来。要压吞吐/时序上限是另一件事，用
[压力测试（`klipperx stress`）](stress.md)，那条会独占板子并把固件留在 shutdown。
