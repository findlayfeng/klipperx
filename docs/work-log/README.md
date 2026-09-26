# 工作记录

本目录存放一次性的**分析 / 盘点 / 审计记录**，不是规范，也不随实现推进而更新。
规范与执行清单以 `docs/klippy/` 与根目录的 `TODO.md` 为准。

下表按**创建时间**（首次入库时间）从早到晚排列。对应任务**已完结**的记录会从本目录
清理掉，需要时从 git 历史找回（`git log --diff-filter=D -- docs/work-log/`）。

| 创建时间 | 记录 |
|---|---|
| 2026-09-21 17:12 | [FW4 动工记录](2026-09-21-fw4-notes.md) — G-Code 框架收尾做完了哪些、哪些留给 C1/D1，以及 GCodeIO 的拍板点 |
| 2026-09-21 23:37 | [FW5e-2 动工记录](2026-09-21-fw5e-notes.md) — `[stepper_*]`/`[printer]` 装载、`G1`/`G4`、连接期 `stepper_get_position` 对齐；真板验收按决定后置到 FW5f 之后 |
| 2026-09-21 23:47 | [FW5f 动工记录](2026-09-21-fw5f-notes.md) — `stepcompress` 完整压缩（`(interval,count,add)`/`max_error`），与上游 C harness 向量对拍 |
| 2026-09-22 01:12 | [FW6a 动工记录](2026-09-21-fw6a-notes.md) — 清 FW5 运动层的单 MCU 假设（每 stepper 自算 flush 时钟、`McuClock`/对齐下放到 `McuChip`），顺带修 `McuClock::seed` 未取 `CLOCK_FREQ` 的 bug |
| 2026-09-22 09:31 | [FW6d–FW6f 记录与收口](2026-09-21-fw6de-notes.md) — `stepcompress` history/回零句柄、`Kinematics::home`/`HomingState`、`drip_move`+`G28`、F3 结论与 FW6 状态 |
| 2026-09-23 09:10 | [H2（风扇与通用输出）动工前调查](2026-09-23-h2-notes.md) — 上游 fan/heater_fan/controller_fan 等 21 个文件的依赖盘点、344 次「运行×缺口」测算、GCodeRequestQueue/heater 注册表/tachometer 三处差距与 H2-1…H2-7 拆分 |
| 2026-09-23 10:03 | [G4/H10（gcode_move）动工前调查](2026-09-23-gcode-move-notes.md) — 上游 g-code 坐标系的状态/命令/事件模型、`G0`/`G1` 从 toolhead 搬家的差距、`Move out of range` 49 次的算账、G4-1/G4-2 拆分与六个拍板点 |
