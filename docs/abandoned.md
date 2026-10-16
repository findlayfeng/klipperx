# 放弃与搁置的改动（`abandoned/`）

本文件是 `abandoned/*` 分支集合的台账。任何**被放弃、被替代或来源可疑**、且不进（或不再进）
主线的改动，都保存成 `abandoned/<短名>` 分支，并在此登记：来源、为什么没进主线、还剩下什么
可挖掘的东西、日期。规则原文见 [`AGENTS.md`](../AGENTS.md) 的「规则 8」。

约定：

- **只保存，不合入**：`abandoned/*` 不参与主线——不合并、不在其上继续开发、不作为验收依据；
  它是素材库，不是分支等待区。
- **不丢弃**：被放弃的改动**不得直接删除**；也无论它来自谁（含子代理写错位置、中断、被否的方向）。
- **删或复活要用户点头**：删除某个条目、或从中取料继续做，都由用户决定，执行仍由 main 完成
  （规则 4/5）。
- **取料要按主线规矩来**：从 `abandoned/*` 摘出来的东西，进主线前必须过正常流程——
  新分支、可编译、有测试、回归与手工台账同步，并在提交信息里注明来源分支。

## 清单

| 分支 | 来源 | 为什么没进主线 | 可挖掘点 | 日期 |
|------|------|----------------|----------|------|
| `abandoned/wip-main-leftovers` | 2026-09-23 自主时段的探针族 worker：把文件写进**主检出**（而非分配它的 worktree）、未提交，随后超时中断 | ① 与已合并实现是**同一批 seam 的另一套**（`mod.rs`/`load.rs`/`upstream.rs` 改同样几处，二选一而非叠加）；② `probe.rs` 的命令实际没注册（`fn register_commands(&self, _printer)` 的参数未使用）；③ `pins.rs` **删掉了 chip 存在性检查**、改成用到时再查——会静默接受未知 chip，并改掉语料首因统计依赖的 `Unknown pin chip name '…'` 文案；④ 从未编译、测试或 review | `bed_mesh.rs`（1004 行）**含插值**：`ZMesh`、`lagrange_interpolation`、`bicubic_interpolation` 与 1D 核，另带 `test_zmesh_lagrange_interpolation`、`test_faulty_region_prefix_options`。主线 `bed_mesh` 目前只做装载与标定探测，插值仍缺——可作素材移植 | 2026-09-23 |
| `abandoned/fix-mcu-leak-probe` | `agents/fix-mcu-leak` worker（2026-09-25，被 main 强收） | 仪表可用但**未定位到环**：只加了 `Mcu` 存活计数与逐例 `scratch_printers_subset`，复现方式是「整套不退出」这种慢信号（会挂住 lane）。主线改用**每例独立 runtime + 有界收尾**（上游同构）解掉阻塞，并用 `PrinterObject::release_cycles()` + `GCodeDispatch` 持 `Weak<Printer>` 断环 | ① `Mcu`/`McuObject`/`McuClock` 存活计数的写法（可作为未来断环单元的脚手架）；② 逐例 OS 线程数 + `mcu_live` 二分法（本次已用它证明「每例泄漏 1 个 Mcu」、`[mcu]+[printer]` 不漏而 `+[stepper_x/y/z]` 开始漏）；③ 【已解决·2026-10-06】环的确切持有者已定位并断掉：`VirtualSdCard` 与 `GCodeButton` 持 `Arc<GCodeDispatch>`、又把捕获 `Arc<Self>` 的 handler 注册回命令/mux 表；改成 `Arc::downgrade` 后 `teardown` 能真的析构部件（撤掉修复探针即点名 cfg 转红、在位则 0.50s 转绿） | 2026-09-25 |
| `abandoned/nextest-lane` | `agents/test-host-lib-decouple` 的附带产物（2026-10-16）：`scripts/nextest-host-library.sh`（单次 `cargo nextest run` 跑那批「装载真库」的测试）+ testing.md 里那句「可选（不接闸门）」的推广 | 用户裁定**放弃 A（换 runner 快车道）**：① 实测收益不大（18 条 `host_tests`：单进程 20.6s → 并行 5.1s，但**整库二进制才是墙钟大头**，省下的占比有限）；② 它要改**全仓闸门口径**（`cargo test --workspace` 在 AGENTS.md/README/testing.md 多处是硬约定），并连带 `cargo hwtest` 别名与 `scripts/cargo-locked.sh` 的对齐；③ 侦察证明「一进程一份库」本来就不靠它解——真相是 `SIGUSR1` disposition（可改 C 解，见 §备注），而 `SIGALRM` 是死代码；④ 那把 `HOST_LIBRARY_TEST_LOCK` **另有用途**（进程级 stderr 重定向与 pty 资源），换 runner 也去不掉它 | ① 若将来闸门墙钟成为瓶颈，`cargo-nextest` 的**最简接法**（一条 `cargo nextest run -p klipperx -E …`、profile 与过滤表达式）可直接抄；② 「每测试一进程 ⇒ 跨测试库污染不可能发生」这条隔离收益的实测数据（`nextest` 与单进程的对比）；③ 如果改走 C 层多实例路线（见下一条备注），此快车道可退为「仅生产/CI 的快路」 | 2026-10-16 |

> 备注（2026-10-16 侦察结论，未立项）：让 host 库支持「一进程多份实例」的真正阻断项只有 **`SIGUSR1` 的进程级 disposition**
> （`third_party/klipper/src/host/host.c:48-55` 的 `longjmp` handler、`:114-119` 安装、`:147-148` `pthread_kill`）；`SIGALRM` 在
> host 构建里是**死代码**（`src/linux/timer.c` 属独立固件进程，不在 `.so` 内），C 侧最小改法是「`eventfd` 唤醒 fd + `irq_wait` 里 `ppoll` 它」，
> 合计约 50 行；收益是**进程内多实例夹具**（可替掉 8 处真固件进程夹具），但**不带来测试并行**（锁另有用途）。
