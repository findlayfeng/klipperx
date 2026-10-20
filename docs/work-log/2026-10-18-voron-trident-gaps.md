# 阶段一：主机对接固件、在真机上安全开发调试（2026-10-18 待办清单）

一次性盘点记录，**不是规范**。范围已按「先跑通阶段一，再谈打第一版」收窄：
**API 对接 = 让一个现成的 Moonraker 把这台 klipperx 当 klippy 用**（不实现 Moonraker/L2，
`server.files`/`server.temperature_store`/`machine.*` 全由 Moonraker 自己提供）。
打印相关（圆弧、床网 profile、PA、打印质量）一律划到阶段二，见 §6。

**阶段一的验收定义（全部做到才算过）**：

1. 板子上电 → 主机 `klipperx klippy printer.cfg -a ...` 连上，不改配置能反复 `RESTART` /
   `FIRMWARE_RESTART` 各 ≥3 次；
2. Mainsail（经 Moonraker）能看到温度曲线、位置、`state`，能在控制台敲 G-code 并看到回显；
3. 三个轴（含 z1/z2 三电机同轨）能手工小位移、能 `G28 X/Y/Z`，**全程没有人身/设备危险**；
4. 能加热到 60 °C / 200 °C 并看到曲线，能随时停；`M112` 急停有效；
5. **出错能自己回来**：拔插 USB、宿主 `systemctl restart`、`kill -9` 后重启，都不需要断电重插；
6. 出任何问题都有日志可查。

## 0. 口径

- 源码基线：分支 `work` @ `35c3e8e`，**以源码为准**；`TODO.md`/`TESTING.md` 只做交叉验证，
  冲突记在 §7。
- 证据标注：**✅复核** = main 亲手对过源码；**◻侦察** = 只读侦察产出、未逐行复核。
- 硬件假设：CoreXY、`[stepper_z]`+`[stepper_z1]`+`[stepper_z2]`、电感/欧姆龙探针、热床 + 单挤出机、
  TMC2209（UART，已真机验过）或 TMC5160（SPI，未上过板）、无 LCD。

## 1. 主机侧 P0 —— 挡在门口（5 条，必须本阶段解决）

| # | 项 | 现状 | 证据 | 在本阶段的表现 | 修法 |
|---|---|---|---|---|---|
| P0-1 | **电机自动使能缺失** | 未实现（代码自陈） | ✅复核 `extras/stepper_enable.rs:647-651`；✅复核 全 `extras/`+`motion/` 无 `set_motors_enable` 调用点，只有 `SET_STEPPER_ENABLE`/`force_move`/`manual_stepper` 显式调用 | 上电后第一次 `G28`/`G1` **一动不动**，跑满行程后报 `No trigger on x after full movement`。写 `enable_pin` 的轴靠 EN 脚；无 EN 脚的 TMC 轴靠 `toff`（`tmc.rs` 先把 `toff` 置 0），两条路都不自己开 | 补上游 `stepper.add_active_callback(motor_enable)`（`third_party/klipper/klippy/extras/stepper_enable.py:51`）。**临时绕行**：逐轴 `SET_STEPPER_ENABLE … ENABLE=1` |
| P0-2 | **`TURN_OFF_HEATERS` 不存在** | 未实现 | ✅复核 `rg TURN_OFF_HEATERS src/` 只命中 `idle_timeout.rs:84,113`（脚本文本）与 `virtual_sdcard.rs:73`；`heaters.rs` 无注册 | `[idle_timeout]` 到点时脚本第一句落成未知命令（`gcode.rs:1184` 只回 `// Unknown command` 且返回 Ok），实际只跑 `M84` ⇒ 调试期间忘了关的床/热端**一直保着温** | 在 `heaters.rs` 注册（上游 `heaters.py:255-330`）。**临时绕行**：`[idle_timeout]` 的 `gcode:` 改写成 `M140 S0` + `M104 S0` |
| P0-3 | **MCU 掉线无活性检测** | 未实现 | ✅复核 `rg "Lost communication\|check_timeout\|is_alive" src/core/klippy/mcu src/core/klippy/interface` 零命中；◻侦察 `mcu/object.rs:713-748`（1 s `mcu_clock_poll` 失败只 `debug!`）、`serial.rs:202-228`（读 `EIO` 后收发任务静默退出） | 调试期间线松/USB 瞬断，主机**仍报 `ready`**，命令全落到已关闭通道、加热按最后目标值继续 —— 最像「在跑其实没跑」，会浪费整轮排查 | 在 `mcu_clock_poll` 判超时 → `invoke_shutdown`；读错误上报而非静默返回 `None`（上游 `mcu.py:895-905`） |
| P0-4 | **串口单次 `open`，复位后不自愈** | 未实现 | ✅复核 `interface/devices/serial.rs:23`（注释明写「the port is opened **once**」）、`:86`；上游 `serialhdl.py:185-204` 为 5 s×90 s 重试 | 每次 `FIRMWARE_RESTART`/`systemctl restart`/断电重上电，只要 tty 还没回来就钉在 `error`，**不会自己再试** —— 调试时一天要撞很多次 | 按上游加 5 s×90 s 重试预算 |
| P0-5 | **加热固件看门狗被关掉** | 与上游相反 | ✅复核 `extras/heaters.rs:810` `pwm.setup_max_duration(0.0)`；上游 `heaters.py:62` 用 `MAX_HEAT_TIME=3.0`，而本仓常量还在、标着 `#[allow(dead_code)]`（`heaters.rs:46-49`） | 宿主**挂住而不是死掉**（反应堆被阻塞——真机已有一次 5 s UART 阻塞先例）时，固件没有「多久没收到新 PWM 就断电」⇒ 热床按最后占空比一直加热。`shutdown_value=0` 挡不住这种情况 | 改 `setup_max_duration(MAX_HEAT_TIME)`；补上游 `set_pwm` 的第二重保护（`read_time > verify_mainthread_time` ⇒ 输出 0） |

## 2. 主机侧 P1 —— 调试期间会踩（7 条）

| # | 项 | 证据 | 在本阶段的表现 |
|---|---|---|---|
| P1-1 | **Z 兄弟电机不进 Z 端停的刹车集** | ✅复核 `extras/stepper.rs:493-513`（只有带自己 `endstop_pin` 的 stepper 注册）；✅复核 `extras/toolhead.rs:880-905`（补充注册只覆盖 `endstop_pairs`）；上游 `stepper.py:434-445` 注册轨道全部 stepper | 首次 `G28 Z` 探针触发瞬间只有 `stepper_z` 刹车，z1/z2 还跑完约 50 ms 队列（0.25–0.5 mm），随后 `set_position(home)` 把三只电机都记成端停位置 ⇒ **gantry 隐斜且不入账**，下一次下降可能刮床。**绕行**：Z 回零低速 + 每次 `G28 Z` 后跑 `Z_TILT_ADJUST` |
| P1-2 | **日志默认不落盘、落盘也不轮转** | ◻侦察 `src/klippy.rs:50-57`（`--logfile` 可选、缺省 `null`）、`src/logging.rs`（只有重启时的 rollover，无按大小轮转）；仓内**无 systemd unit 文件** | 不给 `--logfile` 就没有可回溯的东西；给了也会无限增长。调试期「出问题没日志」＝白干一轮。**绕行**：命令行带 `--logfile` 并外部 logrotate |
| P1-3 | **定时器回调 panic 无人兜** | ◻侦察 `reactor.rs:38-53`、`architecture.md:137`（只有 gcode handler 与 API endpoint 有 `catch_unwind`） | dispatcher 一死，`mcu_clock_poll`/`idle_timeout`/订阅推送全部静默 ⇒ 「看着在、什么都不动」，且没有报错线索 |
| P1-4 | **失败 `G28` 后的恢复路径会 shutdown** | ◻侦察（`TODO.md:478-481` 真板实测） | 端停接错/极性反了 → `No trigger …` → 想用 `SET_KINEMATIC_POSITION` 手动摆位再 `G1`，撞 `Internal error in stepcompress: … Invalid sequence` **直接 shutdown**。首次接线几乎必踩，等于没有「软恢复」 |
| P1-5 | **`FORCE_MOVE`/`STEPPER_BUZZ` 真机不出步** | ◻侦察 + `TESTING.md:439`（`[manual_stepper]` 的自有 trapq 已于 2026-10-18 补齐，这两条前提变了，需重新归因） | 这两条正是「认电机/查某轴动没动」的**第一诊断工具**；不可用会显著拉长排障 |
| P1-6 | **`admin` attach 无会话 takeover** | ◻侦察 `TODO.md:480-485`（`flash/bridge_flash.rs` 只重试 3 次；klippy 侧能接管陈旧会话） | 被硬杀过的板子/桥，`admin scan`/`admin flash` 进不去，只能断电重插 ⇒ 迭代变速慢。走 mux/桥拓扑的调试必踩 |
| P1-7 | **并发 `gcode/script` 会交错** | ◻侦察 `gcode.rs:38-41` 自陈（上游靠 reactor mutex 串行化，本仓在调用方任务上跑） | Mainsail 控制台 + 自己的脚本同时发 `G28`/`G1` 会在 await 点交错。**约定**：调试期只用一条通道发运动命令 |

## 3. Moonraker 对接 —— 阶段一只需到「能看、能敲、能停」

前提讲清：Moonraker 是中间层，klipperx 只负责 **L1**（`0x03`-JSON over unix socket）。
Mainsail 的 HTTP/WS、`server.files`、`server.temperature_store`、`machine.*`、历史与鉴权**都归 Moonraker**，
本阶段不在仓内做任何 L2。

| # | 项 | 现状 | 证据 | 本阶段判定 |
|---|---|---|---|---|
| A-1 | **`toolhead` 缺 `axis_minimum`/`axis_maximum`** | 未实现 | ✅复核 `extras/toolhead.rs:1182-1210` 只回 8 个键（`position`/`homed_axes`/`print_time`/`extruder`/四个速度限）；上游 `toolhead.py:502-515` | **P0（UI）**：Mainsail 的床尺寸/软限位/剩余时间直接读这两个键，缺失就是空/0。调试时不致命但会一直误判「床多大」。（**定级已错**：Moonraker 不读这两个字段，实际是 P2——见 §9） |
| A-2 | `info` 握手 | 可用 | ✅复核 `api/endpoints/info.rs:59-63`；`python_path` 仍报不存在的路径（Moonraker 更新面板对 Klipper 一项是空壳，已登记 E2）；`klipper_path` 2026-10-19 起改为真实释放根（见 api-reference） | ✅实测 Moonraker 能起、能连、`_save_path_info` 正常 |
| A-3 | `webhooks` 状态（`ready`/`error`/`shutdown`） | 可用 | ◻侦察 `api/webhooks.rs:256-263` | 阶段一**关键**：停机/掉线要在 UI 上直接可见，否则 P0-3 那种幽灵运行连界面都看不出来 |
| A-4 | `objects/{list,query,subscribe}`（0.25 s） | 可用 | ✅复核 三端点存在；`objects_subscribe.rs:68,75` | 温度曲线 + 位置就靠它 —— 阶段一的主界面 |
| A-5 | `gcode/{script,help,subscribe_output}` | 可用 | ✅复核 形状与文档逐条一致 | 控制台敲 G-code 与看回显 |
| A-6 | `gcode/restart` / `firmware_restart` / `emergency_stop` | 可用 | ✅复核 三端点注册且在配置炸掉时也能用 | Mainsail 的三个按钮＝阶段一的「救命键」，必须实测 |
| A-7 | `virtual_sdcard.path` 与 Moonraker 的 `gcodes` 目录对齐 | 可用 | ✅复核 `extras/virtual_sdcard.rs:143-146`（只读 `path` 下已存在文件） | 阶段一只需目录存在（文件上传属阶段二） |
| A-8 | `bed_mesh/dump_mesh` | 可用（`profiles` 恒 `{}`） | ✅复核 `api/endpoints/bed_mesh_dump.rs:58,95` | 阶段一够用；profile 是阶段二 |
| A-9 | `*/dump_*` 数据流（adxl345 / motion_report / load_cell …） | host 6 条已实现 | ◻侦察；`crates/klippy-client` 不解析 bulk 流 | 阶段一不用（`klippy-client` 有通用 `api` 透传）；与 Mainsail 无关 |

**阶段一要动的只有 A-1 一处**，其余是「实测一遍能不能通」。若 `info` 的假路径让 Moonraker 起不来，
按已登记方案处理（Moonraker 侧 updater no-op，或 `info` 补占位路径）。

## 4. 阶段一必须上板实测（软件已实现、无板上证据）

| 项 | 为什么还没验 | 验收动作 |
|---|---|---|
| CoreXY 双电机联动 + `G28 X/Y` | 真机只验过 F446 单轴 cartesian | 逐轴 1 mm 步进 → `MSCNT`/`queue_step` 对得上；`G28` 重复 5 次位置一致 |
| 三电机同轨（`stepper_z*`）+ `Z_TILT_ADJUST` | 无板上证据 | connect 期校验 `z_positions` 行数 = 3；`Z_TILT_ADJUST` 真跑一轮 |
| 探针触发链路（`G28 Z` / 单次 `PROBE`） | 语料只有假固件 | 能触发、能停、`QUERY_PROBE` 状态对（精度属阶段二） |
| Moonraker + Mainsail 实连 | 仓内无任何 L2，无对接测试 | 连上后能看到曲线/位置/`state`，能敲 G-code、能按三个按钮 |
| `M112` / `emergency_stop` 端到端 | 未真机验 | 急停后 UI 立刻显示 shutdown，加热与电机都停 |

## 5. 本阶段的配置/运维层绕行（不改代码就能上机）

1. 逐轴 `SET_STEPPER_ENABLE STEPPER=…` 后再动（P0-1）；或用带 `enable_pin` 的配置并接受同样的人工使能。
2. `[idle_timeout]` 的 `gcode:` 改显式 `M140 S0` / `M104 S0`（P0-2）。
3. **不要写 `[homing_override]`**：它只有选项解析、不替换 `G28`，宏会静默不执行（`extras/homing_override.rs` 全文件无 `register_command`）。
4. 不用 BLTouch（`G28 Z` 不降针，`bltouch.rs:26-30` 自陈）；用电感/欧姆龙。
5. Z 回零低速，且 `G28 Z` 后必跑 `Z_TILT_ADJUST`（P1-1）。
6. 起进程时必带 `--logfile <path>`，外部 logrotate；systemd 用 `Restart=on-failure` + `StartLimitIntervalSec`（仓内不带 unit 文件）。
7. 运动命令只走一条通道（P1-7）。

## 6. 不在阶段一（阶段二：打第一版）

- `G2/G3` 圆弧（`gcode_arcs.rs` 零注册、未知命令静默且返回 Ok —— 阶段二第一件）；
- bed_mesh profile 命令族（`BED_MESH_PROFILE/OUTPUT/MAP/OFFSET` 未注册，`PRINT_START` 里的 `LOAD=` 静默失效）；
- 二次回零 / `homing_retract_dist`（`stepper.rs:37`、`toolhead.rs:7067`）；
- 探针结果取「移动终点」而非触发时刻（`toolhead.rs:3866-3898`，修法现成：`past_mcu_position(trigger_time)` 如 `toolhead.rs:3706`）；
- `pressure_advance` 对运动零效果（`extruder.rs:92-96`、`itersolve.rs:484-489`）；
- `exclude_object` 跨文件残留 + 与 `bed_mesh` 抢变换单槽；
- 文件回放无背压 / 无「带真运动的整文件打印」验收；
- `bltouch` / `smart_effector` / `probe activate_gcode`；
- TMC5160（SPI）等 SPI 族上板验证（若阶段一就用 5160 板，则提前到阶段一）。

## 7. 与源码冲突的文档（阶段一相关的几条）

| # | 位置 | 文档说 | 源码 |
|---|---|---|---|
| D1 | `docs/klippy/user-manual/config.md:987` | `[virtual_sdcard]` 回放命令族「未注册」 | 已注册（`extras/virtual_sdcard.rs:428-450`），且引用了已完成的工单号 |
| D2 | `TODO.md:599` | `[homing_override]` 已落地 | 只有选项解析，`G28` 未被替换（§5-3） |
| D3 | `TODO.md:674-677` | `home_axis`/`home_unified` 从不判断有没有触发 | 已加 `trigger_time <= 0.0` → `No trigger on <name>`（`toolhead.rs:3692`、`:3577`），应销 |
| D4 | `TODO.md:272` | `tmcuart` 固件资源仍是未完成项 | 真板已用它读通 `IOIN`（`TESTING.md:356`） |
| D5 | `src/.../pid_calibrate.rs:33-38` | 「`M105` g-code-id 尚未接线」 | 已接（`heaters.rs:444-462`、`:577-580`） |
| D6 | `docs/klippy/user-manual/client.md:398-401` | `bed_mesh/dump_mesh` 未实现 | 已实现（`bed_mesh_dump.rs:58`）；只有未装载 `[bed_mesh]` 时才回 `No registered callback` |
| D7 | `docs/klippy/third-party-dev/README.md:27-45` | `list_endpoints` 样例 16 条 | 实际必含 `bed_mesh/dump_mesh` |
| D8 | `docs/klippy/third-party-dev/api-reference.md` §4 | `response_template` 必填 | 可选、缺省 `{}`（`register_remote_method.rs:59-64`） |

**2026-10-19 复核：D1–D8 已全部按源码改写**——D1 改成「回放命令族已注册」并点名
`virtual_sdcard.rs:449-470`；D2 拆成「`homing_heaters` 已落地 + `homing_override` 的 G28 包装未装」
两条；D3 的 Q12 与 D4 的 `tmcuart` 条目已从 `TODO.md` **删除**（完成即删）；D5 改成「等待期每秒
回 `M105`，与上游同形」（`wait_for_temperature` 走同一张 gcode-id 表）；D6/D7/D8 已随实现改写。

## 8. 建议顺序

1. **P0-1 + P0-4**：不动与连不上，其它都无从谈起（P0-1 有临时绕行，可先上机）。
2. **P0-5 + P0-2**：加热相关的两条安全底线，在做第一次加热调试前落地。
3. **P0-3 + P1-3 + P1-2**：可观测性三件套（掉线要能报、dispatcher 死了要有信号、日志要留得下）。
4. **A-1 + Moonraker 实连**：让 UI 能看能用，之后再谈操作。
5. **P1-1 + P1-4**：Z 回零的安全与失败恢复，`G28 Z` 反复调试前必须解决。
6. **P1-5 + P1-6**：把诊断与远程复位手段补齐，长跑调试效率在这里。

## 9. 实施记录（唯一随实现推进回填的一节）

### 2026-10-18 第一批：P0 五条 + API 一处（已合入 `work`）

| 项 | 状态 | 落点 |
|---|---|---|
| P0-1 电机自动使能 | ✅ 落地（`f0075f7` 系列） | `motion/itersolve` 的 `check_active`、`motion/queuing` 的 `ActiveCallbacks`、`extras/stepper_enable` 的按 `print_time` 排期使能；TMC 虚拟使能由同一条链驱动（`toff` 回填有单测） |
| P0-2 `TURN_OFF_HEATERS` | ✅ 落地（`e964a1e`） | `extras/heaters.rs` 注册（ready 表，文案同上游），三种 heater 归零有测试；`config.md`/`gcode-commands.md` 已同步 |
| P0-3 MCU 失联检测 | ✅ 落地（`e58e614`） | 判据按上游实际实现（未答查询 > 4），不是「估计打印时间 vs 单调时钟」——侦察报告里的表述**有误**，已按 `mcu.py:897-905` 修正；挂在既有 1 s poll 上，file-output 不判 |
| P0-4 串口打开重试 | ✅ 落地（`d004901` + `b86b814`） | 5 s × 90 s，只包打开；**重试改在阻塞池执行**（原本同步睡在 async 里会停住 2 worker 运行时） |
| P0-5 加热固件看门狗 | ✅ 落地（`e964a1e`）；**第二半 2026-10-19 补齐** | `setup_max_duration(MAX_HEAT_TIME=3.0)`；`set_pwm` 的 `verify_mainthread_time` 那一半已做（5 s 截止戳 + 1 s 打戳定时器），并同批补了 `get_temp` 的 7 s 陈旧压制——两者都要求读数与 print time 同域，靠 `agents/feat-sensor-clock-domain` 把 ADC 样本时间统一到 print time 才解锁 |
| A-1 轴范围 + 估计时间 | ✅ 落地（`db76573`） | 按 Moonraker 源码复核后**降为 P2（UI）**：它不读这两个字段 |
| L1 契约守卫 | ✅ 落地（`8294b5f`） | `info` 两路径字段、四态 `state`、`print_stats` 首帧 `state` 三条回归守卫 |

**Moonraker 对接结论（只读 `/home/fengcheng/project/klipperx.worktrees/moonraker` 源码）**：
现成 Moonraker 可把本机当 klippy 用，L1 无阻塞项；握手链 `info` → `objects/*` → `gcode/subscribe_output`
→ `list_endpoints` → `register_remote_method` 全部已注册且形状一致。三条硬依赖已加守卫。
部署约定（非代码缺口）：`-a /tmp/klippy_uds`、`-c <data_path>/config/printer.cfg`、
`--logfile <data_path>/logs/klippy.log`、`[virtual_sdcard] path: <data_path>/gcodes`。

**下一步（第二批，未开始）**：P1-1（Z 兄弟电机进端停刹车）、P1-2/P1-3（日志落盘与轮转、定时器 panic 兜底）、
P1-4（失败 `G28` 后的恢复）、P1-5 归因、P1-6（admin takeover）、上板实测（§4 五项）与 Moonraker 端到端冒烟。

### 2026-10-18 第二批：P1 六条（已合入 `work`）

| 项 | 状态 | 落点与要点 |
|---|---|---|
| P1-1 同轨兄弟电机一起刹车 | ✅ | `Rail::lookup` 把不带自己 `endstop_pin` 的兄弟注册进轨道端停 dispatch；`McuTrsync::add_stepper` 改幂等（轨道注册 + 虚拟端停 helper 会重复注册） |
| P1-2 日志轮转 + rollover 配置块 | ✅ | **修正本笔记 §2 的错误修正**：上游**不是**「2 MB × 5」，是 `TimedRotatingFileHandler(when='midnight', backupCount=5)`；按午夜轮转、新文件写 `rollover_info`（`versions` + `config` 块） |
| P1-3 回调 panic 兜底 | ✅ | 定时器 / latency notifier / flush / active 四类回调都 `catch_unwind`；panic 的定时器退场，其余照跑（偏移已登记） |
| P1-4 失败 `G28` 后 shutdown | ✅ | 首因是**生成地平线回退**（drip 已生成到末端、规划器 `print_time` 停在起点）；修法：地平线取 `max(两处)` + 未触发分支补上游收尾三步 |
| P1-6 admin attach 接管陈旧会话 | ✅ | identify 层的 takeover 本来就在，缺的是「握手无应答就走不到 identify」；现在预算耗尽后开链路 → identify 接管 → 同一 handle 补握手。沉默板子仍 `AttachUnanswered`，被别的进程驱动才报 `SessionHeldByAnotherProcess` |
| P1-5 `FORCE_MOVE`/`STEPPER_BUZZ` 不出步 | ✅ 归因 + 收口 | host 侧本来就出步（+800 / ±800）；真机症状归因为 A 没使能（已修）+ B manual_stepper 未进队列（已修）+ C 队列外电机静默空跑（改 `warn!`）。**新发现真缺口**：IDEX 第二滑车压根不进运动队列，整条驱动链不出步（已登记 `TODO.md`，另立单元） |

**方法上的两条教训（记下来避免重犯）**：
1. 我在任务书里两次把「上游常量」写错（`bglogger` 的 2 MB、`force_move` 的 `KINEMATIC=`），都是凭印象而非回源码；
   两次都由 worker 回源码纠正。此后同类断言一律先 `rg` 上游源码再用。
2. 「让缺陷不静默」不等于「让命令报错」：上游语料要求 `STEPPER_BUZZ STEPPER=dual_carriage` **成功**，
   把本仓缺陷做成命令错误会让合法 G-code 被惩罚 —— 正确落点是日志 + 台账。
