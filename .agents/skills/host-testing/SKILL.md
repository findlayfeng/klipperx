---
name: host-testing
description: 给本仓（Rust 宿主 + 上游语料）写测试与核闸门的手法：断言形态阶梯（字节/hex → 数值 → 行为可见性 → 日志）、负向对照证明钉子有效、确定性纪律（禁裸 await、禁固定 sleep 后断言、有界轮询）、不依赖调度线程的断言选择、vendor 常量漂移守卫、闸门计数核账与 flake 追查。当要新写/改造测试、评审一个「全绿」声明、或闸门出现单次不可复现失败时使用。
---

# 本仓测试写法

> 事故驱动：本批 30 个子代理 lane 里，测试写法的好坏直接决定了「交付物能不能被验证」——
> 用日志捕获断言的用例会随线程调度闪；「跑通了」式验收让一个把上游语义搞反的实现混过一轮；
> 裸 `await` 一个永不到来的 oneshot 让一条 lane 30 分钟超时。下面的口径都是从这些事故反推出来的。

## 1. 断言形态阶梯（从强到弱，尽量往上靠）

1. **帧/线上字节（hex）**——最硬。显示驱动断言 panel 的帧缓冲字节、SPI 面板断言帧序列、
   MCU 命令断言 `input` 整块 hex、上位机命令断言 `encode()` 结果。这类断言连「顺序对不对」都钉住了。
2. **数值与公式**——`"%02d" % 2.7 == "2"`、`(5*d+new)/6` 的具体输出、`round(raw*10000)`、`9806.65/2048`。
   数字要么从上游源码逐字节抄，要么从本仓实际输出抄（**不许估**）。
3. **行为可见性**——回调被调用及参数、`queue_step` 的 oid/步数、`get_status()` 的键值、
   `resolve()` 的返回值、容器里的 item 序列。「消息/事件继续旅行」用**投递**证明，不要用日志证明（见 §5）。
4. **日志**——最后手段。只有「这条日志本身就是被测行为」（例如 `Unhandled message …` 的去重计数、
   `Stats <t>: …` 的汇总行）才用它。

判定口径：一条验收如果只写「跑通了 / 不报错」，**退回重写**。真实事故：某 lane 声称
「上游 `printer` 可枚举已实现」，实际上它把上游语义搞反了——因为它只断言了「渲染成功」。

## 2. 负向对照：证明钉子真能钉住 bug

写完钉子后，**把实现临时改回旧行为（或短路新逻辑）跑一次，确认用例变红**，再恢复。
报告里贴出那次红的结果（`left:` / `right:` 或 panic 文案）。真实例子：

- 二级 MCU 时钟闸门：把 `if !is_seeded()` 短路成 `if false && …` → 用例报
  `left: (4.05, 20000000.0) right: (0.0, 20000000.0)`，正是缺陷台账里的垃圾 offset；
- `pending::cancel` 按 token：把 `cancel` 改回「删第一个同名匹配」→ 幸存那笔在 `unwrap()` 处
  `RecvError(())`，且编译器提示 `field token is never read`。

没有负向对照的「新增测试」不算证据；有对照的才算。

## 3. 确定性纪律（硬要求）

- **禁裸 `await` 一个可能永不到来的信号**（oneshot / 通道 / 未来事件）。事故：一条 lane 的用例
  同名两笔登记后只 `resolve` 一次，第二笔的 `oneshot` 永远收不到 ⇒ 测试二进制 `Threads=2`、
  `CPU time 00:00:00`、全线程 `futex_wait`，挂到 30 分钟超时。**一律** `tokio::time::timeout` 或
  有界轮询包住。
- **禁「固定 sleep 后紧跟依赖它的断言」**——CPU 抖动下必闪。等后台状态用
  `test_util::wait_until(deadline, interval, pred, what)`。
- **测试必须整体有界**：新增/改造测试要有总体超时兜底，禁止能挂住 CI 的用例。
- **单测不得留永久定时器**：定时器类模块提供「单步驱动」入口（如 `render_once(eventtime)`），
  编排层用假时钟/手动推进，别让测试自己等真时间。
- `block_in_command` 一类要求多线程 runtime；current-thread 下会 panic——需要时显式
  `#[tokio::test(flavor = "multi_thread")]` 并写清原因。

## 4. 需要多线程/跨任务时：先问「证据落在哪个线程」

- 接收任务、定时器回调、后台 flush 可能跑在**别的线程**。
- 断言只能依赖「跨线程可见的共享状态」（通道、`Arc<Mutex<_>>`、绑定的回调、返回的错误），
  **不能**依赖线程局部状态（见 §5）。
- 需要外部进程/真板时按 [真机 lane] 的既有做法（`hardware_test` 的 `case`/`async_case` 自管线程 +
  `recv_timeout`，超时打 `HW-TIMEOUT: <用例>: still at <阶段> after <时长>`）。

## 5. 日志捕获是**线程局部**的（本批踩过）

`tracing::subscriber::set_default(...)` 只对**当前线程**生效。接收任务/定时器换一条线程时，
日志会写到别处（或没有），断言 `matches(...).count() == 1` 就会时而 0、时而 1。

- **不要用日志断言跨线程行为**。要让「消息没被吞」可见，就**给该消息绑一个回调**，断言回调被调用。
- 只有「日志本身就是被测行为」时才用捕获，并且：确认该日志在**测试线程**上同步产生（装配期、
  配置错误期），或把用例限制成 current-thread 并写注释说明依赖。

## 6. 语料对齐与 vendor 常量

- 一切对照读 `third_party/klipper/`（不看机器上其它 klipper 检出）；**语料测试是最终仲裁**。
- 逐字节搬来的常量（`font8x14.VGA_FONT`、`display.cfg`、`menu.cfg`）必须配**漂移守卫测试**：
  读上游文件与内置副本 `assert_eq!`，上游更新时立刻红。
- 上游没有的东西（寄存器表、协议字段）**不许自造**：先在 `third_party/klipper` 里 `rg` 出真身，
  找不到就在模块文档写成显式余项。

## 7. 闸门与计数核账

- 窄过滤跑局部（`cargo test -p klipperx --lib <过滤>`），合并前跑 `cargo test --workspace`。
- **核总数**：`总数 = 上次基线 + 本批新增 fn test 数`。对不上就是漏跑/多跑（真实事故：某个
  「All green」的报告里 `changed tracked files: none`，提交根本不存在）。
- **跑闸门一律用 `scripts/test-log.sh`**，不要裸 `cargo test | grep`：它把整次输出落到
  `target/tmp/test-logs/<时间戳>-<标签>.log`（在 `target/` 里 ⇒ 不进仓库、不受重启影响），
  失败时先按名字列出 `^test .* FAILED`、再贴 `failures:` 段，构建失败则贴尾部诊断，最后打印日志路径：
  ```sh
  scripts/test-log.sh -l gate --workspace
  scripts/test-log.sh -l pending -p klipperx --lib mcu::pending -- --test-threads=1
  TEST_LOG_KEEP=50 scripts/test-log.sh -l hunt -p klipperx --lib --no-fail-fast   # flake 追查
  ```
  目录默认只留最近 20 份（`TEST_LOG_DIR`/`TEST_LOG_KEEP` 可改）。只 grep `^test result:` 会丢掉用例名
  ——本批两次不可复现失败就是因为这个没抓到名字。
- 有界命令：`timeout 1200 cargo test … --no-fail-fast`；并发构建导致慢是正常的，别误判成挂死。
- 判「挂死还是慢」：看测试二进制的 `CPU time` 与线程 `wchan`——**CPU≈0 + 全线程 `futex_wait` = 在等**；
  `rustc`/`cargo` 在跑 = 在编译，放过。

## 8. flake 追查流程（单次失败、随后不可复现）

1. 记录**时间点与当时正在做什么**（合入哪个 lane、刚跑过什么）。
2. `for i in 1 2 3 4 5 6; do scripts/test-log.sh -l hunt-$i -p klipperx --lib --no-fail-fast; done`，
   至少 6 轮（每轮的完整输出都留在 `target/tmp/test-logs/`，抓不到也只是没抓到，不会连名字一起丢）。
3. 抓不到就换维度复跑：`RUST_TEST_THREADS=1` 对照（区分时序 vs 共享状态）、单独跑可疑模块、
   或加 `--test-threads=N`。
4. 找到后用 §3/§5 的口径改掉断言形态，并在决策/台账里记一条；**没找到就如实记为未复现**，
   别写成「已修」。

## 9. 反例清单（都真实发生过）

| 反例 | 后果 | 正确做法 |
|---|---|---|
| `#[test] fn …` 插在 `impl` 体内 | 编译错（`#[test]` 只能用于自由函数）+ `cannot find function load` | 放进文件末尾的 `mod tests` |
| 裸 `await` 一个永不到来的 oneshot | 测试进程全线程 `futex_wait`、CPU 0，挂到超时 | `tokio::time::timeout` / `wait_until` |
| 用 `LogCapture` 断言跨线程 warn | 计数随线程调度变 0 | 绑定回调做证据 |
| 只断言「不报错」 | 把上游语义搞反也能过 | 断言帧字节 / 返回值 / 投递 |
| 期望表只覆盖自己 lane | 合并后 `printer.objects()`、工厂表顺序断红 | 合入时由 main 补跨 lane 期望（本批两次） |
| 测试里等真时间 | 47 s 闸门偶发变慢/闪 | 假时钟、单步驱动 |

## 10. 收口前自查

- [ ] 每条新增测试能指出它**钉住的是哪条缺陷/需求**（一句话）。
- [ ] 至少一条负向对照留了证据。
- [ ] 没有裸 await、没有固定 sleep 断言、没有永久定时器。
- [ ] 数字类断言能回指上游 `file:line` 或本仓实际输出。
- [ ] 合并前跑过 `cargo test --workspace`，总数与「基线 + 新增」一致。
- [ ] 需要同步的手册/差异项已列清单（按 [doc-sync](../doc-sync/SKILL.md)）。
