# 工作记录

本目录存放一次性的**分析 / 盘点 / 审计记录**，不是规范，也不随实现推进而更新。
规范与执行清单以 `docs/klippy/` 与根目录的 `TODO.md` 为准。

正文记的是写作当天的盘点与判断，**不随后续实现改写**；每篇末尾的「实施记录」（§6）是唯一
随实现推进更新的回填节——余项与现状以它为准，仍留在本目录的记录都还有未收口的余项。

下表按**创建时间**（首次入库时间）从早到晚排列。对应任务**已完结**的记录会从本目录
清理掉，需要时从 git 历史找回（`git log --diff-filter=D -- docs/work-log/`）。

| 创建时间 | 记录 |
|---|---|
| 2026-09-23 09:10 | [H2（风扇与通用输出）动工前调查](2026-09-23-h2-notes.md) — 上游 fan/heater_fan/controller_fan 等 21 个文件的依赖盘点、344 次「运行×缺口」测算、GCodeRequestQueue/heater 注册表/tachometer 三处差距与 H2-1…H2-7 拆分 |
| 2026-09-23 10:03 | [G4/H10（gcode_move）动工前调查](2026-09-23-gcode-move-notes.md) — 上游 g-code 坐标系的状态/命令/事件模型、`G0`/`G1` 从 toolhead 搬家的差距、`Move out of range` 49 次的算账、G4-1/G4-2 拆分与六个拍板点 |
| 2026-10-16 07:10 | [CAN ISO-TP / CAN-FD 动工前调查与实施规划](2026-10-16-can-isotp-notes.md) — **按设备类型选路**（FD 设备＝`cans+FD`；标准 CAN 设备＝`isotp+classical`；`isotp+FD` 保留可能性不做）：ISO-TP 字段级规格（含 FD 单帧转义式 62 B）、**帧数开销对照（classical 下 `queue_step` 档 2×，FD/cans 下 1 帧/消息）**、CAN-FD 落地细节（`struct canbus_msg` 8→64 的逐处清单、三份 `fdcan.c` 寄存器级缺口、哪些架构不能 FD、主机侧 72B `canfd_frame` 落点、六个未验证项）、主机/固件/配置面/linux 四处文件级落点与十四条风险 |
