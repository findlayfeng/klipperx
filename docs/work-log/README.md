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
| 2026-10-16 23:20 | [MCU 架构的桥端口后端（`SERIAL_UART` / `uart_backend_*`）规格](2026-10-16-bridge-port-backend-spec.md) — 只读侦察留档：桥端口后端今天只有 host 库实现（`src/host/bridge.c`），`src/Kconfig:200-207` 的注释即「An arch selects this once it can hand a UART object out as a bridge port」；本报告给出契约面（`bridge_port_ops`/`uart_backend_*`，架构无关、不用改）、MCU 侧要补的东西（3 个后端函数 + 端口 ops + 自加 `DECL_TASK`，因为 host 的 `bridge_poll()` 挂在 `irq_wait()` 上）、stm32 现有 UART 资产能否复用、Kconfig 接线（架构只需 `select BRIDGE_PORT_SERIAL`）、槽号↔UART 与 baud 来源、`reset` 无 modem 线只能 `NULL`、**linux 架构可离机端到端**（桥=linux 固件进程 + pty 端口 + 假 bootloader）与建议单元拆分。**状态：实现待派** |
| 2026-10-16 10:38 | [Katapult 拓扑刷写：动工前调查、决策与任务拆分](2026-10-16-katapult-flash-task.md) — **引入 katapult（子模组已入主线 `d6a66c4`）+ 自研拓扑刷写工具/桥隧道**的决策论证；兼容性六维裁定（帧层三道闸不互混、寻址逐位同值、bootloader 态握手=超时而非误触发、共存安全但送不到、串口/CAN 协议帧同字节非 1:1、上游边界一致）；缺口清单（共享口下行双重封装→unit 形态两步、桥回程刷写隧道模式）与 **T1–T5 任务拆分**、三个拍板点、五条未验证风险；附三路侦察全文（file:line 级）。**阶段 1 已真机通过（六项全绿）；T3、T5a/T5c/T5b′/T5-tool/T5d-1/T5d-2/T5d-3 与 B1/B2/B3 均已验收合入（见 §8）**；余**阶段 2 真机**（`klipperx flash --bridge … --port … --port-nodeid …`，板子停在 katapult 等下游接线） |
