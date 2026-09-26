# 工作记录

本目录存放一次性的**分析 / 盘点 / 审计记录**，不是规范，也不随实现推进而更新。
规范与执行清单以 `docs/klippy/` 与根目录的 `TODO.md` 为准。

下表按**创建时间**（首次入库时间）从早到晚排列。对应任务**已完结**的记录会从本目录
清理掉，需要时从 git 历史找回（`git log --diff-filter=D -- docs/work-log/`）。

| 创建时间 | 记录 |
|---|---|
| 2026-09-23 09:10 | [H2（风扇与通用输出）动工前调查](2026-09-23-h2-notes.md) — 上游 fan/heater_fan/controller_fan 等 21 个文件的依赖盘点、344 次「运行×缺口」测算、GCodeRequestQueue/heater 注册表/tachometer 三处差距与 H2-1…H2-7 拆分 |
| 2026-09-23 10:03 | [G4/H10（gcode_move）动工前调查](2026-09-23-gcode-move-notes.md) — 上游 g-code 坐标系的状态/命令/事件模型、`G0`/`G1` 从 toolhead 搬家的差距、`Move out of range` 49 次的算账、G4-1/G4-2 拆分与六个拍板点 |
