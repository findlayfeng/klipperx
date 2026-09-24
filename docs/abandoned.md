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
