---
name: stress-real-board
description: 在真机上跑 `klipperx stress` 的实操：release 构建、目标板与任务的选择、输出怎么读（backlog/链路判上限）、饱和后的收尾、以及 `--task step` 的前置条件与风险。当要用压测量吞吐/时序、对照留档数字、或复现「链路饱和/丢答」时使用。
---

# 真机压测（`klipperx stress`）

> 政策（压测一律 release、CAN `txqueuelen`、刷板纪律）在 [`AGENTS.md`](../../../AGENTS.md) 与
> [`TESTING.md`](../../../TESTING.md)；本页只写**怎么做**、**怎么读**、**坑**。

## 三条硬前置

1. **一律 release**：`cargo build --release`，跑 `./target/release/klipperx stress …`。debug 构建会先到自己的瓶颈（同一档差约 20%）。
2. **`txqueuelen 128` 只对宿主直连 CAN 适用**（`ip link set can0 txqueuelen 128`）。经**桥**的 CAN 是桥内部的 CAN，主机上没有 `can0`，这条不适用——别以为环境没配好。
3. **`--task step` 会真动轴**：目标板要有 `[stepper_*]`/`[manual_stepper]`，电机接着、行程够。空载（无电机）量出来的不是结论。step/dir 在桥板上时，目标就是桥那块 `[mcu]`，不是下游。

## 命令形状

```bash
klipperx stress <cfg> [MCU] --task comm|step|motion [--all-mcus]
# 例：单板默认 [mcu]；指定哪块板按节名（[mcu downstream] → downstream）
```

- `comm`：小消息往返（一问一答 `get_clock`，每往返约 22 B）——**最常用的吞吐刻度**。
- `step`：单向批量 `queue_step`，量固件能收多快。
- `motion`：一整段运动链路的冒烟（整机可用性）。
- `--all-mcus`：多板并发，用来看**并发损耗**。

## 输出怎么读

```
[downstream]  1455 req/s: sent 726, answered 725, achieved 1452 req/s, backlog 1
[downstream]  2842 req/s: sent 1416, answered 1124, achieved 2832 req/s, backlog 292
[downstream]  link gave out at 2842 req/s: 292 requests went unanswered
```

- **backlog 恒 0** ⇒ 还有余量；**backlog 单调上升** ⇒ 这一档已经超了；
- `achieved` 贴着目标 ⇒ 没到顶；`achieved` 追不上且 backlog 涨 ⇒ 到顶；
- `link gave out` 是**预期的饱和终局**，不是缺陷；`last rate it carried` 就是这次量到的上限；
- 报数按「`answered / sent`」，别只看 `achieved`——它会把丢答平均掉。

## 对照留档（同拓扑跑出来应落在同一档）

| 拓扑 | 下游 comm | 并发下的桥 | 下游 step |
|------|-----------|------------|-----------|
| UART 250 kbaud | ~930 req/s（线速上限） | — | — |
| UART 500 kbaud | ~1814 req/s | — | 145 455 steps/s |
| CAN 1 Mbit | ~2268 req/s | ~1814（−36%） | 15 625 steps/s |
| F103 直连 USB（参照） | 3552 req/s | — | 339 623 steps/s |

数字与留档差 1–2% 属正常抖动；**差 10% 以上先查构建（release？）、拓扑（哪块板？）、并发（有没有别的会话在跑）**，再怀疑代码。

## 收尾（今天现场学到的）

- 饱和之后**板子不会自愈**：`identify` 可能不再应答、日志出现 `Retransmit failed … Input/output error`。
- 不必断电：先 `klipperx admin reset --bridge <tty> [--bridge-uuid <hex>]`（**不带** `--bootloader`），再 `admin bridge config` 确认回来。只有连 admin 单元都送不进去才需要真拔电。
- 跑完 `pgrep -x klipper.elf` 自查残留进程。

## 坑

- **同一块板不能并行跑两个压测/会话**：一个 tty 只能有一个占用者，第二个会把第一个的 fd 弄死（EIO，表现为「后端莫名失联」）。
- **`--task comm` 量的是往返，不是带宽**：想算利用率要用 `last_stats` 的 `bytes_read/bytes_write` 差分；USB CDC 的 `baud` 是协商值，**不能当分母**。
- **压测前先把固件刷好并确认 identify**：否则你会把「板子没起来」测成「链路到顶」。
