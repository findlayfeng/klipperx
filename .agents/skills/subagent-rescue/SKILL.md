---
name: subagent-rescue
description: 救援卡住、超时或反复零产出的子代理（worker / reviewer / scout），含 provider content_filter 误杀与测试挂死。当子代理长时间无活动、报 content_filter、或连续超时交不出东西时使用。
---

# 子代理救援（stuck / timeout / content_filter）

## 先判状态，再动手（别急着重跑）

1. `subagent({action:"status", id})`：看 `state`、`updated`、`activity`、当前 tool call 与
   `acceptance`（`review-required` 等）。
2. 有在跑的 bash 时看进程，而不是猜：
   `pgrep -af`，再 `ps -o pid,stat,etime,time,wchan:24 -p <pid>`。
   **判据：全部线程 `futex_wait` 且 CPU 时间≈0 ⇒ 在等**（死锁、或等一个永不到来的信号），
   不是死循环、也不是算得慢。子进程树要一起看（`ps --ppid <cargo-pid>` 找测试二进制）。
3. 读它的日志尾部与事件流：`output-0.log`、`events.jsonl`（异步目录里）。

## 处置（按症状）

- **测试挂死**：直接杀掉挂住的**测试二进制**（`kill` → `kill -9`），cargo/`timeout` 会随之退出，
  子代理立刻看到非零退出并继续，**别等它自己的 900s/1800s 预算**。然后 steer 它：
  「给测试整体加 `tokio::time::timeout` 兜底 / 别再等不会发生的事 / 先用 `--test-threads=1`
  复现并报出挂在哪个测试」。
- **provider `content_filter` 误杀**（`finish_reason: content_filter`，通常零改动、无产出）：
  - 会话历史已很大（>60k 输入）且本就两次被过滤 ⇒ **fresh context 重开**（新建 worker，
    不要 resume 那个被污染的会话）；
  - 上下文已就位、只是侦察过度 ⇒ 同会话 `resume` + **强制落笔**。
- **反复超时零产出**：resume，并把策略压成「立即落笔 → 每步增量提交 → 只读错误点名的文件」；
  要求「10 分钟内必须有首个提交」，这样再超时也不丢进度。
- **命令控制**：`subagent({action:"command.status|yield|cancel", …})` 只对**本地原生 pi**
  子进程可用；工作流（nested）子进程没有命令控制接口——只能杀进程 + `steer`。

## 交棒纪律

- 子代理遇超时必须交**三件套**：① 进度（已提交/未提交/已证事实）② 卡点证据（卡在哪步 +
  日志尾部）③ 建议下一步。只回「超时了」不算报告。
- worker 上下文吃紧**禁止压缩**：停下报告，由 main 裁棒、换棒续做（压缩会吞掉任务书里的
  规格与验收数字）。
- 同一个写手别在同一个 worktree 反复失败：换 fresh context，或换 worktree 重开。

## 写给自己的任务书模板（防超时）

> 立即落笔：读模板后立刻写第一版代码，**10 分钟内必须有首个提交**（WIP 也算），此后每完成
> 一步就提交；只读编译/测试错误点名的文件，禁止 grep 探索与读无关源码；命令形式：单条命令、
> 无 `2>&1`/`|`/`>`/`;`、改文件用 `edit`/`write`；测试窄过滤 + `timeout`；第一次构建可能是
> 冷构建，用 `timeout 600` 等完，别在中途反复探查。
