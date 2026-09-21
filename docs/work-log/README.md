# 工作记录

本目录存放一次性的**分析 / 盘点 / 审计记录**，不是规范，也不随实现推进而更新。
规范与执行清单以 `docs/klippy/` 与根目录的 `TODO.md` 为准。

| 日期 | 记录 |
|---|---|
| 2026-09-21 | [上游功能覆盖审计](2026-09-21-upstream-coverage-audit.md) — 逐项盘点上游全部功能点，产出未实现清单（落到 `TODO.md`） |
| 2026-09-21 | [FW1 / FW3 信息收集与修改建议](2026-09-21-fw1-fw3-notes.md) — 配置装载框架与错误词汇的动工前调查、设计选项与建议 TODO 改法 |
| 2026-09-21 | [FW4 动工记录](2026-09-21-fw4-notes.md) — G-Code 框架收尾做完了哪些、哪些留给 C1/D1，以及 GCodeIO 的拍板点 |
| 2026-09-21 | [FW5 动工前调查](2026-09-21-fw5-notes.md) — 运动栈（toolhead/trapq/itersolve/stepcompress/clocksync/kinematics）的层次与契约、差距、FW5a–FW5e 拆分与拍板点 |
| 2026-09-21 | [FW5e-2 动工记录](2026-09-21-fw5e-notes.md) — `[stepper_*]`/`[printer]` 装载、`G1`/`G4`、连接期 `stepper_get_position` 对齐；真板验收按决定后置到 FW5f 之后 |
| 2026-09-21 | [FW5f 动工记录](2026-09-21-fw5f-notes.md) — `stepcompress` 完整压缩（`(interval,count,add)`/`max_error`），与上游 C harness 向量对拍 |
| 2026-09-21 | [FW6 动工前调查](2026-09-21-fw6-notes.md) — endstop/trsync/命令队列三链的层次与契约、差距、FW6a–FW6f 拆分与拍板点（多实例纯 Rust 触发、假 MCU 多实例验证、不引入 host serialqueue、`stepcompress` history 前置） |
| 2026-09-21 | [FW6a 动工记录](2026-09-21-fw6a-notes.md) — 清 FW5 运动层的单 MCU 假设（每 stepper 自算 flush 时钟、`McuClock`/对齐下放到 `McuChip`），顺带修 `McuClock::seed` 未取 `CLOCK_FREQ` 的 bug |
| 2026-09-21 | [FW6b 动工记录](2026-09-21-fw6b-notes.md) — endstop/trsync 命令层 + `MCU_endstop` + 多实例 `TriggerDispatch`/`MCU_trsync`（每 MCU 一个 registry 路由、纯 Rust 取最慢者延长超时） |
| 2026-09-21 | [FW6c 动工记录](2026-09-21-fw6c-notes.md) — `[stepper_*]` 的 `endstop_pin`/`homing_*` → `Rail`、`query_endstops` 对象 + `query_endstops/status` 端点 + `M119`（FW6 框架验收） |
