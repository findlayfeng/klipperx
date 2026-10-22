# 本项目与上游的偏移

本仓是 Klipper 主机侧（上游 `klippy/`）的 Rust 重写，行为以**上游为准**；但有若干地方与上游
不一致：有些是**有意取舍**（上游有、本仓不做），有些是**实现形态不同**（同一功能两种表现），
还有些是**尚未做到**（暂缓/未实现）。这一章把这些偏移集中登记，一条一行，指向展开细节的页面。

约定：

- **细节只写在展开页**，本章给一行结论 + 链接；展开页在相应位置回链本章。
- 上游行号引用仍写在展开页；核验方式见[测试 → 文档同步](testing.md#文档同步)。
- **未完成的工作**（工单号、待办、下一步）不在这里登记 —— 那是 [`TODO.md`](../../../TODO.md)
  的职责；一次性分析背景见[工作记录](../../work-log/README.md)。
- **计数型内容**（端点条数、事件条数、语料条数、二进制体积）只写在各自页面，本章不复制数字，
  免得两处对不上。

## 1. 上游有，本仓不做（取舍，不是待办）

### 1.1 G-Code 行协议输入（`GCodeIO`）：伪 tty 与文件输入

上游 `klippy/gcode.py` 的 `GCodeIO`（`:390-494`）承担两条输入通道：

1. **默认模式：伪 tty**。`klippy.py` 用 `util.create_pty()` 把 `-I/--input-tty`（默认
   `/tmp/printer`）做成"虚拟串口"，并在其上跑 Marlin 式**行协议**：每条命令回一行 `ok`
   （或 `ok <msg>`），错误回 `!! …` 且**不中断**后续行；没人消费的应答被丢弃。
   OctoPrint 一类"串口主机软件"就是连这个 pty。
2. **文件输入模式**：`-i/--debuginput <file>` 从文件读 gcode；配合 `-o/--debugoutput` 与
   `-d <dict>` 就是官方文档化的 **batch mode** —— 离线把 gcode 翻成 MCU 命令流，用于
   检视低层行为、对比改动前后的命令流、跑 host 性能基准。

**本仓不做这两条**（决定见 [`TODO.md`](../../../TODO.md) 的 G1b / Q8）：

- 输入通道只有 API（`-a` 的 unix socket / TCP，Moonraker 语义）与终端客户端
  （`klipperx console` / `--tui`）；`-I` / `-l` / `-i` / `-o` 这些开关**不存在**
  （见[客户端使用](../user-manual/client.md)）。
- `ok` 应答机制与 `need_ack` 标记**已从 `src/core/klippy/gcode.rs` 删除**：命令错误一律
  "报告（`!! ` 输出行 + 日志）+ 发 `gcode:command_error` + **中止整段脚本**，把错误交回调用方"。
  因此 `gcode/script` 的成功回复是 `{}`，失败是 JSON-RPC error 回复，输出里**没有 `ok` 行**。
- `gcode:debuginput_exit` 事件**连声明都没有**（上游只有行协议输入会发它）：本仓的事件枚举声明 34 个上游事件名，唯一不声明的就是它（见[事件系统 §2 当前实现](event-system.md#21-已实现)）；
  `stats` / `gcodein` 计数不存在（本仓也没有上游 host 的 `Stats …` 日志行）。

**这些场景本仓不能替代上游** —— 需要时请用上游 Klipper：

| 场景 | 上游怎么做 | 本仓 |
|---|---|---|
| OctoPrint 等串口主机软件 | 连 `~/printer_data/comms/klippy.serial`（老部署 `/tmp/printer`；官方 [OctoPrint.md](https://www.klipper3d.org/OctoPrint.html)） | **不能**：没有 pty；`-a` 的 API 是另一套协议，OctoPrint 不能直接连 |
| 离线把 gcode 翻成 MCU 命令流检视/对比 | `klippy.py printer.cfg -i test.gcode -o test.serial -v -d out/klipper.dict`，再 `parsedump.py`（官方 [Debugging.md](https://www.klipper3d.org/Debugging.html)） | **不能**：没有 `-i`/`-o` |
| host 侧性能基准 | `time klippy.py config/example-cartesian.cfg -i complex.gcode -o /dev/null -d out/klipper.dict`（官方 [Benchmarks.md](https://www.klipper3d.org/Benchmarks.html)） | **不能**（同上） |
| AVR 模拟器联调 | `scripts/avrsim.py out/klipper.elf` + `klippy.py config/generic-simulavr.cfg -i test.gcode`（官方 [Debugging.md](https://www.klipper3d.org/Debugging.html)） | **不能**（同上） |

**注意别把两件事混起来**：Moonraker 的 `[octoprint_compat]` 只是"给切片软件上传 gcode"的
OctoPrint API **子集**（官方配置文档），**不是** OctoPrint 本身接入 Klipper 的途径 ——
OctoPrint 只走 pty，与本条取舍无关。

**本仓的替代物（不同形态，不是同一功能）**：离线跑 gcode 的能力由
[virtual_sdcard](README.md)（`M23`/`M24` 回放 gcodes 目录里的文件，逐行走同一个
`run_script` 调度器）与 API 的 `gcode/script` 提供；"不连真实 MCU 也能跑"由
`test: dict=<file>` 的字典应答机 + `start_args.debug_output` 提供（见[回归测试](regression-tests.md)）。
两者都**不产出** `-o` 那样的 MCU 命令流文件、也不接受任意路径的 `-i <file>`。

### 1.2 其它"上游有、本仓不做"

| 项 | 现状 | 详情 |
|---|---|---|
| `Z_OFFSET_APPLY_ENDSTOP` 的 delta 变体 | 上游把 delta 处理函数**重注册到同名命令**上覆盖普通变体（`manual_probe.py:74-79`） | 本仓按 `kinematics: delta` **二选一注册**（`register_command` 对重名报错，与上游 `gcode.py:142-144` 同款）——对外行为等价 | [G-Code 命令参考](../user-manual/gcode-commands.md) |
| `BED_SCREWS_ADJUST` 命令族的失败收尾 | 上游在三条命令注册中途失败时会留下半个命令组与卡住的 `state='adjust'` | 本仓把已注册的那部分收回并复位会话（`manual_probe` 的命令不被碰）——唯一有意偏离，已在模块 doc 记明 | [配置手册 `[bed_screws]`](../user-manual/config.md)、[模块表](README.md) |
| `[display]` 菜单与渲染 | **都已落地（2026-10-21）**：`menu.py`/`menu_keys.py` + 嵌入 `menu.cfg` + 四个 `menu:*` 事件；渲染 step（`render`/`draw_progress_bar` 宿主可调用注入、`draw_text`、`~glyph~`）与 `hd44780` 画字/图标；出货 3 个 layout 有帧缓冲字节断言。残余偏差：`menu:*` 载荷上游是 `self` 对象、本仓无载荷（按对象名 `menu` 查）；六个驱动全会画（`font8x14.VGA_FONT` 已逐字节 vendor）；`[display_data]`/`[display_glyph]` 的主配置 factory 2026-10-21 落地（此前只靠「选项被读过」侥幸可用，schema 检查会判未知节） | [配置手册 `../user-manual/config.md`](../user-manual/config.md) |
| `samd_sercom` / `replicape` / `palette2`（板级/外设专有模块） | **暂不实现（2026-10-22 拍板）**，三项进本表而不是待办：`samd_sercom`（41 行）是 SAMD21/51 的 pad→SERCOM 外设分配器，只对用 SERCOM 引脚的 samd 板有意义；`replicape`（268 行）是 BeagleBone Replicape 扩展板专用（5 步进合并写法、PCA9685 输出、板载温度/风扇特例），代价是回归里唯一剩下的静态缺节；`palette2`（654 行）是 Palette2 拼接器的串口协议客户端（`PALETTE_*` 命令族 + ping/pong + gcode 隧道），无设备就无法验证 | 三者都绑定具体上游硬件/运行时；没有对应设备时写 654 行协议代码属盲写。需要时按「新增单元」重新立项，届时以 `file:line` 为单位移植并只做语料里出现的能力。**真板 lane 也会拦**：严格模式（默认开）下，配置里出现这三项会让真机用例直接失败并点名它们（`hardware_test.rs` 的 `NOT_IMPLEMENTED_SECTIONS`，名单与本节对齐） | [配置手册](../user-manual/config.md)、[模块表](README.md) |
| 上游无、本仓有 | 通用 `[i2c_device]` / `[spi_device]` 节与 `IIC_WRITE` / `IIC_READ` / `SPI_TRANSFER` / `SPI_SEND` 调试命令、`KlipperX` 自有调试总线命令 —— 属**本仓扩展**，上游没有对应物 | [配置手册](../user-manual/config.md)、[G-Code 命令参考](../user-manual/gcode-commands.md) |

## 2. 行为偏移（同功能，两种表现）
| 响应分派与「没人要的应答」 | 分派键是 `(消息名, oid)` 二元组；没有登记的 `(name, oid)` 落到 `handle_default`，打一条 `got {'#name': …, 'oid': …}` 的 **warning**（`klippy/serialhdl.py:59-62`、`:311-312`）；等待方的登记也是按 `(name, oid)` 精确匹配（`register_response(cb, name, oid)`） | 本仓的 `PendingCalls::resolve` 先按**名字**找待答，再按 oid 判是否属于它：**匹配到 ⇒ 消费并投递**；同名但 oid 无人认领 ⇒ **不消费**、返回 `false`，消息照常走「绑定的回调 / `Unhandled` 告警」那条路（等价于上游 `handle_default` 的可见性，只是文案与通道不同：我们进 `Unhandled message … discarding` 的去重计数）。另一处有意的宽松：**无 oid 的调用接受任意 oid** 的应答（上游的精确二元组会落到 `handle_default`），因为本仓的请求/响应消息里 oid 只是区分参数、不是身份 | [MCU 协议](mcu-protocol.md)、[模块表](README.md) |
| 脚本执行与错误策略 | 行协议输入下错误不中断；`run_script` 持 `gcode` mutex 串行化 | 只有"报告 + 中止整段脚本"一种策略（`need_ack` 已删）；脚本在调用任务上跑完，不再有 mutex | [klippy-runtime](klippy-runtime.md)、`src/core/klippy/gcode.rs` 模块文档 |
| `start_args` 字段 | 上游有 `debuginput`（`-i`）与每 MCU 字典路径（`-d`） | 本仓没有 `debug_input` 字段与字典路径（不实现文件输入 / batch mode）；`debug_output` 保留（语料 harness 的文件输出开关） | [klippy-runtime](klippy-runtime.md)、[回归测试](regression-tests.md) |
| mux 命令的"值不合法"提示 | 按 dict 迭代序取最后一个匹配候选 | 对候选排序后取第一个（消息稳定） | [模块表](README.md) |
| 越界报错的限制值写法 | `0.0` 这类浮点写法 | 整数限制值打印成 `0` | [模块表](README.md)、[配置手册](../user-manual/config.md) |
| MCU 配置 CRC | 上游的 CRC 算法 | **算的不是同一样东西**（本层唯一有意偏离） | [MCU 配置构建](mcu-config.md) |
| identify offset 不匹配 | 不追加数据、用同一 offset 重试（可无限循环） | **落后**的块同样不追加、重发同一 offset（只是用每块的超时预算兜底）；**超前**的块直接报错 | [Identify 机制](identify.md)、[klippy-runtime](klippy-runtime.md) |
| 事件处理器里的配置错误 | `send_event` 向上抛（`_handle_mcu_identify` 里的 `add_stepper` 拒绝直接中断 bring-up） | 事件总线隔离处理器，`load_cell_probe` 的 Z 轴注册遇「共享轴跨 MCU」只 `tracing::error!` 记录后继续 bring-up（同一错误在端停路径上仍是配置错） | `src/core/klippy/extras/load_cell_probe.rs`、`event-system.md` |
| 等待与回调模型 | greenlet + `reactor.pause()` / `completion`（含 `register_fd`） | async/await + 定时器表；`set_latency_notifier` 纯诊断 | [reactor](reactor.md)、[klippy-runtime](klippy-runtime.md) |
| 无出口自旋的两条等待 | `virtual_sdcard.do_pause` 的 `while work_timer is not None and not cmd_from_sd: reactor.pause(.001)`（`virtual_sdcard.py:123-127`）与 drift sweep 的 `while move_times: reactor.pause(evttime + .1)`（`temperature_probe.py:605`），都只靠条件成真退出，没上界 | 同条件、同间隔（1 ms / 100 ms），但各加 10 s 上界：`PAUSE_DRAIN_TIMEOUT`（`do_pause` 超时 `warn!` 后返回，签名仍是 `()`）、`DRIFT_SWEEP_WAIT`（超时后交给既有的 `incomplete sensor data` 拒绝）（2026-10-19） | [测试](testing.md)、[klippy-runtime](klippy-runtime.md) |
| 内置 config/docs 资产 | 上游**直接读源码树**：`klipper_path` 就是 klipper 检出，`config/`、`docs/` 随检出在盘上，无释放概念 | 本机没有源码树也得让 Moonraker 注册到「示例配置」「文档」：`build.rs` 把上游 `config/`、本项目用户手册与上游 `docs/`（放 `docs/klipper/` 子路径，索引 README 构建期生成）**嵌进二进制**，启动时按 `$RUNTIME_DIRECTORY` → `$XDG_RUNTIME_DIR/klipperx` → `/tmp/klipperx-<uid>` 释放（**回落只因未配置；配置了不可写=启动报错退出**），`info` 的 `klipper_path` 报释放根；释放根**永不带 `.git`** + `python_path` 恒为假值 ⇒ Moonraker 的 Klipper 更新项恒为 `BaseDeploy`（2026-10-19） | `src/assets.rs`、`api/endpoints/info.rs`、[api-reference](../third-party-dev/api-reference.md) |
| 发送队列的流控与同步暂停 | 同步写 + `send_mutex`，没有“给同步发送方留空位”这一层；暂停都是 greenlet `pause` | 队列留 `SYNC_SEND_HEADROOM`(16) 槽给同步路径，异步生产者超出就等（`queue_awaiting_room`，`mcu/mod.rs:642`）；两条等待各有上界：`SYNC_SEND_WAIT` = `PAYLOAD_ROOM_WAIT` = 1 s，后者到点报 `send queue has no free slot within …` 而不是永久自旋；同步侧的睡眠走 `sleep_off_worker`（多线程 runtime 上 `block_in_place`，不独占 worker）（2026-10-19） | [klippy-runtime](klippy-runtime.md)、[测试](testing.md) |
| section → 对象表 | 运行时按文件导入 | 编译期由 `section!` 声明生成 | [声明式表生成](codegen.md) |
| 事件系统 | 运行时字符串事件、`*params` 可变参数、`try/except` 隔离、`assert_no_pause()` 强制 | 编译期枚举、变体字段、`catch_unwind`、不做运行时阻塞校验 | [事件系统 §6](event-system.md#6-与上游的差异) |
| 模板引擎 | Jinja2 | minijinja 适配层：`%`/`//` 取余、Strict 缺键时机、`|int`/`|float` 无默认时的不可转换值、`|min`/`|max` 空序列、部分 detail 措辞不同；标签输出走**自定义 formatter**（容器按 Python `str()` 拼写，`~` 拼接仍走引擎 Display）；`"…%s…" % x` 由编译期**源码重写**成单一 `|printf(...)` 过滤器（只收左侧为字符串字面量的形式，`%(key)s` 映射形态不支持；整数码对浮点按 Python 截断，而 `.format()` 方法按 `str.format` 拒浮点）；`str`/`list` 的 Python 方法由 unknown-method callback 提供，而 **`append`/`extend`/`pop` 降级**（值语义，返回 `None` 且列表不变，收集写法拿到空列表） | [模块表](README.md)、[配置手册](../user-manual/config.md)、[G-Code 命令参考](../user-manual/gcode-commands.md) |
| 宏变量与 `SET_GCODE_VARIABLE` 的字面量 | Python `ast.literal_eval`（`gcode_macro.py:153-162`） | 同一解析器（`py_literal`）另外接受 JSON 写法 `true`/`false`/`null`（旧行为保留）；dict 的数字/布尔 key 按上游 `json.dumps` 的形态转字符串 | [模块表](README.md)、[配置手册](../user-manual/config.md) |
| 无配置节也加载的模块 | `toolhead.py:604-614` 无条件 load `gcode_move`/`homing`/`idle_timeout`/`statistics`/`manual_probe`/`tuning_tower`/`garbage_collection` | 只补了 `idle_timeout`（`load.rs` 的 `DEFAULT_MODULES`，缺失的节交给同一个工厂读默认值）；其余四个本仓尚无对应实现，属未覆盖而非有意偏移 | [模块表](README.md)、[配置手册](../user-manual/config.md) |
| 运动数学与步进压缩 | `chelper/` 的 C 代码 | Rust 重写；回零/探针位置**按触发时刻的步数反算**（`past_mcu_position(trigger_time)` → `calc_toolhead_pos`，2026-10-19 对齐上游 `note_home_end`）：探针**报告**触发点但工具头停在规划终点、回零把触发后的过冲算进位置；`home_start` 的 `rest_time` 仍硬编码 | [klippy-runtime](klippy-runtime.md)、[模块表](README.md) |
| move-class 命令的识别 | 由固件 handler 自己 `move_alloc()` 决定（`stepper.c:262`、`pwmcmds.c:86`、`gpiocmds.c:148`），主机不需要一份清单，只按固件报的 `move_count` 记数 | 主机侧维护一份 move-class 清单来决定这条命令占不占 move 槽（`Mcu::send_move` / `send_move_msg`，`src/core/klippy/mcu/mod.rs:2532` `:2628`）。`queue_pwm_out` 与软 PWM 的 `queue_digital_out` 曾走普通 send、不占槽，现改走 move-class（`McuPwm::set_pwm` → `send_move_msg`，`src/core/klippy/mcu/resource/pwm.rs:153-165`；命令自己的 clock 同时是「到期时刻」与「槽释放时刻」）。**清单漏项不报错、只让背压少算**，所以改完由 `resource/pwm.rs` 的两条单测钉住（`test_a_queued_hardware_duty_enters_the_move_pool` / `test_a_queued_software_duty_enters_the_move_pool`） | [模块表](README.md)、[架构总览](architecture.md)、[MCU 协议](mcu-protocol.md) |
| 写方向的等待 | `serialhdl` 把串口设成**非阻塞**自己轮询（`chelper/serialqueue.c:731`），写没有「等端口腾地方」这一阶段；CAN 用 `can` 库的阻塞 send，无截止时间 | 串口／共享串口／刷写 tty 的写**都有上界**：`WRITE_STALL_BOUND` = 1 s，`write_whole` 每字节先等 `POLLOUT` 再写（短写算错误、不是更短的帧），设备侧 `Device::send` 与刷写驱动器的 `TtyPort::write` 用的是同一个（`src/core/klippy/interface/devices/serial.rs:109` `:535`，`src/flash/transport.rs:175`）；CAN socket 用 `SO_SNDTIMEO` = `SEND_TIMEOUT` 100 ms（`src/core/klippy/interface/devices/canserial.rs:119` `:1281`，设不上时 `warn!` 一次并声明「此后写可能无限等」）。到期一律化成 `TimedOut` / `SendError`，不会无限挂在驱动里 | [模块表](README.md)、[架构总览](architecture.md)、[串口复用协议](serial-mux-protocol.md) |
| 电机使能的时机 | `set_motors_enable` 先 `toolhead.flush_step_generation()`，禁用前后各 dwell `DISABLE_STALL_TIME` | 自动使能链按首段运动的 `print_time` 排期 `queue_digital_out`（`add_active_callback` + flush 里的 `itersolve_check_active`，两者与上游同）；手动开关的日期取 `toolhead.get_last_move_time()`，**不 dwell**——排期事件与步进命令同在 MCU 时间轴上，先后由固件保证 | [模块表](README.md)、[G-Code 命令参考](../user-manual/gcode-commands.md) |
| MCU 失联的判定点 | `MCUConnHelper.check_timeout` 由 `calibrate_clock` 调（`mcu.py:1231`），判据是 `clocksync.is_active()`（未答查询 > 4） | 判据与文案相同，但挂在本仓的 **1 s `mcu_clock_poll`** 上（本仓没有独立的 clock calibration 轮）；每 MCU 只报一次，file-output 模式不判 | [模块表](README.md) |
| 连接期串口打开重试 | `connect_uart` 里 5 s 间隔、90 s 预算（`serialhdl.py:181-204`） | 同值同语义，但：预算在**测试构建**下缩到 300 ms（单测不能等 90 s）；重试在**阻塞池**上跑（机器运行时只有 2 个 worker），退出请求在睡眠切片内被察觉；只包「打开端口」，握手与配置不重试 | [MCU 连接方式](../user-manual/mcu-connection.md) |
| 时间敏感回调的锁 | 单线程 reactor，回调自然串行 | `GCodeRequestQueue` 的 sink 回调在**锁外**执行（gcode/flush 双线程） | [模块表](README.md) |
| 回调里的 panic | 无隔离：异常直接打死主循环（所有定时器停掉） | 定时器 / latency notifier / flush 回调 / active 回调全部 `catch_unwind`：记 `error!`（带回调名与 panic 文本）后**丢弃那个定时器**，其余照常调度（2026-10-18） | [架构总览](architecture.md)、[模块表](README.md) |
| `force_move` 对「不在运动队列」的电机 | 每个 `MCU_stepper` 建时就 `allocate_syncemitter`（`klippy/stepper.py:50-56`），所以 `FORCE_MOVE`/`STEPPER_BUZZ` 对任何已注册电机都能出步 | 只驱动 `MotionQueuing` 里持有的那些；名字不在其中时**走时间轴 + `warn!`**（不报错），因为上游语料要求 `STEPPER_BUZZ STEPPER=dual_carriage` 成功（`dual_carriage.test:49`、`hybrid_corexy_dual_carriage.test:42`）。根因是本仓 `idex_modes` 的 carriage 从不进队列（见 §3） | [模块表](README.md)、[G-Code 命令参考](../user-manual/gcode-commands.md) |
| `start_reason` | 主循环每轮写回 `start_args` | 只在一处表示差异 | [klippy-runtime](klippy-runtime.md) |
| `webhooks` 实现形态 | 一个文件同时是传输/派发机器与一个 printer object | 拆成 `klippy-api`（传输/派发）与 `core/klippy/api`（端点） | [klippy-runtime](klippy-runtime.md)、[运行时编排](runtime.md) |
| `[verify_heater <不存在的 heater>]` | 报 `Unknown heater` | 报装载器的 `Section '…' is not a valid config section` | [配置手册](../user-manual/config.md) |
| `[virtual_sdcard]` 细节 | `path` 做 `expanduser`/`normpath`；回放让出用 `gcode.get_mutex().test()`；有 `stats` | `path` 原样用于目录列举；让出用 `tokio::task::yield_now()`；无 `stats` | [模块表](README.md)、[G-Code 命令参考](../user-manual/gcode-commands.md) |
| `trsync_state` 的报告处理 | 每次 `MCU_trsync.start()` 重新绑处理器（`mcu.py:250-260`），`can_trigger=0` 且无待完成 completion 的迟到报告静默忽略（`:228-240`）；关闭一个触发组时向**组内每颗** MCU（报告者自己也在内）发 `trsync_trigger`（`chelper/trdispatch.c:78-89`）；固件无 `trsync_stop`，周期报告只由 `trsync_trigger`/停机清 | 绑定提前到**会话 `attach` 时一次装好、整会话不解除**，无主 oid 静默；`on_restart` 列表发 `trsync_start(oid,0,0,0)` 断掉上一轮遗留的周期报告（2026-10-18）。**语义已与上游逐条对齐**：组关闭后不再接受报告（`TriggerGroup::close` 置 `can_trigger=false`、`McuTrsync::handle_state` 见旗即返回，`src/core/klippy/mcu/resource/trsync.rs:245` `:583`），关闭时广播 `trsync_trigger` + `HOST_REQUEST`（`send_trigger`，`:573`）——差别只在**绑定形态**，不靠每次 start 重绑 | [模块表](README.md)、[MCU 连接](../user-manual/mcu-connection.md) |
| 归零等待的兜底 | `homing.py` 等触发带超时（`TRSYNC_TIMEOUT`），超时报 `Timeout during homing` | 同语义（2026-10-18）：5 s 兜底 + 收尾发 `trsync_trigger` + **失败释放 gcode 锁**；此前无界，丢一份报告就把整台机器锁死（连 `M112` 都排不进去）。**5 s 数的是静默**（最近一次 `trsync_state` 之后多久没再上报），不是「从开始等起 5 s」——后者会把任何比 5 s 长的归零行程都误判成通信超时，而固件其实还在正常上报（真机 2026-10-22：`G28 X` 行程 15.5 s、上报每 75 ms 一次） | [模块表](README.md) |
| TMC 使能突发的执行位置 | `_handle_stepper_enable` 在 reactor 回调里**同步**跑完整段寄存器突发（每写一个寄存器还要读 `IFCNT` 校验 + 回读；真机一次使能实测 ~150 ms），靠 greenlet 不受影响的特性 | 保留「推到下一轮 reactor turn」这层（`stepper_enable` 持全局 enable-lines 锁 + 非重入 `Mutex`），但**那一轮只派发**：`Handle::try_current()` 且 `runtime_flavor() == MultiThread` 时 `spawn_blocking` 跑 `do_enable`/`do_disable`，无 runtime / current-thread runtime（手驱动的 reactor 单测）仍同步执行；新增 `enable_transition` 锁保「一次一段、按投递顺序」（原先由 dispatcher 串行给）。**真机 2026-10-22**：修改前每次固件重置后的第一次 `G28` 都会打 `Reactor busy for 0.148s`（`call_later took 0.149s`）并把其后的时钟同步/加热统计/`objects/subscribe` 一起拖后；修改后同一流程零告警，两个原点的 `Pausing toolhead … phase offset` 从相隔 148 ms 变成隔 6 ms | [模块表](README.md)、[reactor](reactor.md) |
| TMC 相位偏移的读取时机 | `_handle_sync_mcu_pos` 用 `stepper.get_mcu_position()`（solver 位置，永远可读） | 同语义，但本仓现取 `ToolHeadObject::stepper_mcu_position`，而 `probing_move`/`flush_step_generation` 把 toolhead 的 `Connected` 整个 `take()` 出去 → 使能回调恰好在其间拿到 `None`（`Unable to obtain tmc … phase`，53 µs 早退）。现改取 solver 位置（2026-10-18） | [模块表](README.md)、[TESTING](../../../TESTING.md) |
| MCU 连接 `restart_method` | 值域 `command`/`arduino`/`cheetah`/`rpi_usb`，都是「复位这条连接」 | 多一个上游没有的 **`admin`**：经桥转发 `RESTART_DEVICE (0x05)`，让**桥下游的命名设备**重启自己的固件（进引导的 `REQUEST_BOOTLOADER (0x02)` 属刷写工具、**不属** `restart_method`）；桥下游 legacy 设备的线级复位改由**端口节**的 `uart_restart_method` 决定；共享串口仍在解析层被强制成 `command` | [MCU 连接方式](../user-manual/mcu-connection.md)、[串口复用协议](serial-mux-protocol.md) |
| 固件构建的源文件去重 | 每个 `src-$(CONFIG_*) +=` 各列各的；各载体在上游天然互斥，从不重复 | 架构 makefile include 之后加一行 `src-y := $(call uniq,$(src-y))`（**保序去重，不是 `sort`**）：本仓的 `MUX_SERIAL` 可与 `USBSERIAL` 同开，多个载体都会列 `<arch>/chipid.c` ⇒ 重复链接（并让 `compile_time_request` 计入两次）。**保序本身是 ABI**：字典的 message id 按 `src-y` 顺序发放（`buildcommands.py`），排序会把每个目标的字典重新编号，而测试、golden 数据与已记录的线上形状都引用那些 id。顶层 `Makefile` 只包装调用它（`OUT`/`KCONFIG_CONFIG` 外移，见 4.3），不改 `src-y` | `third_party/klipper/Makefile` |
| API 若干已知差异 | 上游 `machine`/`calibration`/`get_status` 等更完整 | 缺 `probe_path`/`rapid_path`、部分 `get_status` 字段未返回等 | [API 参考](../third-party-dev/api-reference.md) |
| ADC 样本的时间戳 | `MCU_adc._handle_analog_in_state` 在 `mcu.py:636-646` 内部把报告时钟转成 print time，消费者拿到的一律是 print time | 同语义，但转换在**资源层**一次完成（2026-10-19）：`AdcCallback` 携带 print time，`clock32_to_clock64` 后再按芯片映射换算（修掉此前把未扩展的 32 位时钟当时间戳、绕圈后样本倒退的 bug）。两处有意偏移：① 芯片**尚无时钟映射**的报整批丢弃并 `warn!` 一次（带 oid）——上游的 `clock_est` 恒可换算，本仓刚连上时可能还没有；② 最后一个样本定在 `next_clock`，上游早一个 `report_clock`（常量平移，见 `TODO.md` F5 余项） | [模块表](README.md)、[配置手册](../user-manual/config.md) |
| 主机停机时对各 MCU 发 `emergency_stop` 的顺序 | 每个 MCU 各注册一个 `klippy:shutdown` 处理器（`mcu.py:800`），按注册序（即配置序）执行；上游每颗 MCU 自带链路，顺序无关紧要 | 处理器体改成**统一走一遍对象表、按「载体链深度降序」停**（`stop_all_mcus`/`shutdown_walk`，`src/core/klippy/mcu/object.rs`）：桥下游的设备只经桥的链路才够得着固件，所以必须在桥还在转发时先停。深度按各 `[mcu]` 的 `carrier`（端口节的 `uart_mcu`/`can_mcu`）逐级算，环形引用只走一遍不打转。**真机 A/B**（F446 桥 + CAN 下游，`M112`，2026-10-22）：改前 `[mcu]` 先发、改后 `[downstream]` 先发，两侧固件**都**收到并回报 `shutdown` —— 本条修的是**不变量**（顺序不该依赖「桥固件停机后 mux 仍恰好在转发」这件事，`src/mux.c` 的路由在 `src/command.c:357` 的命令闸门之下），不是补一个漏掉的停机 | [事件系统 §6](event-system.md#6-与上游的差异)、`src/core/klippy/mcu/object.rs` |

## 3. 暂缓/未实现（本仓没做，方向上不排除）

| 项 | 现状 | 详情 |
|---|---|---|
| 上游 host `Stats …` 日志行 / `gcodein` | 无 host 统计采集器 | **`Stats …` 行已落地（2026-10-21，`extras/printer_stats.rs`）**：1 Hz 汇总各部件 `stats()`，文案 `Stats <eventtime>: <key=value …>`，`debugoutput` 下不装定时器；`gcodein` 计数仍无 | 本条 1.1 |
| `query_adc` / `QUERY_ADC` | 未实现（`adc_scaled` 只用 config 输入量程） | [配置手册](../user-manual/config.md)、[模块表](README.md) |
| `canbus_nodeid` 选项 | 上游没有这个选项（CAN 节点号一律由 `canbus_ids` 按声明顺序发）；且上游的分配器只服务 CAN，节点号只存在于它的 CAN 连接路径里（`klippy/mcu.py:858-862` 经 `canbus_ids.get_nodeid` 交给 `serialhdl.connect_canbus`；本仓的 `Transport` 枚举是另一层说法） | 本仓保留它作显式覆盖：写了就用，且装载时**预扫全配置**把所有显式号先保留（`reserve_explicit`，读节的原始参数、不经 accessor），自动号取 `4` 起的**最小空位**并绕开它们——显式号写在哪个位置都生效，结果与声明顺序无关（上游 `len(ids)+4` 的前提是「没有显式号」，一旦有显式号这个游标就不再等价，故本仓不再用它）；装载期只在**两个显式号互撞**（点名双方）或号段 `4..=255` 占满（点名 MCU）时报配置错，而不到固件 `can_id_conflict` 才停机。更进一步：把 **node id 提升为 MCU 层 id**（`McuConfig.id`），节名改为 `[mcu_ids]`（历史拼写 `[canbus_ids]` 兼容）、分配器（`mcu/ids.rs`）泛化成所有接口共用一个编号序列——CAN 用它寻址，**共享串口**用同一套 `0x100+2*id` 做帧头寻址并在分配端口时同步握手（独占串口不使用；见[串口复用协议](serial-mux-protocol.md)）、`host_library`/`test:` 同样领到 id、只是暂不上线，`connect` 打开传输层前必须持有 | [配置手册](../user-manual/config.md)、[模块表](README.md) |
| 帧级 id 指派信道 | 上游无此机制：串口固件没有任何 id 概念，admin 帧（`0x3f0`）是 CAN 总线层的东西，上游也不在 identify 前对串口做任何指派 | 本仓在**串口线内**开一条复用信道（帧头与伪帧编解码/握手在 `devices/serial.rs`，仲裁 id 映射共用 `assign.rs`，见[串口复用协议](serial-mux-protocol.md)）：同一 `serial:` 路径被 ≥2 颗 `[mcu]` 引用时自动进入**共享模式**——帧头 `2 字节带 4 位检错的码字`（10 数据位 = 仲裁 id、应答 `+1`；**与上游的 Klipper VLQ 不同**，是本仓自定的线上格式）、单元 = 头 + 原样帧、端口只开一次/单 reader 按 id 分派/引用计数关闭；每颗 MCU 在**分配端口时**同步握手（默认 500 ms × 3，无回执即失败，identify 之前）。伪帧 `[09][20]["KXID"][crc16][0x7e]` 仍会被旧固件在 seq 检查处无害丢弃（`command.c` 可执行移植钉住），所以误配到旧固件只是超时、不搞乱会话。另**镜像 CAN 的 admin 命令族**：`0x00` 广播查询未分配设备、`0x20` 应答（uuid + application）、`0x01` 按 uuid 点名指派 id——`attach` 在节上带 `uuid` 时先发 SET_NODEID 再发重确认请求，另有 `klipperx admin scan` 工具（`--interface <bus>` 扫 CAN、`--port <tty>` 扫串口、`--bridge <tty>` 扫桥下游各口，传输必须显式给，输出同形）；上游只在 CAN 侧有这套（`canbus_query.py`），串口/桥侧没有（冲突检测留给固件端结合物理接口）。**独占串口不发任何指派帧、线上无头**（本轮之前的「fire-and-forget 指派帧 + `FrameStream` 回执支路 + `Mcu::assignment_confirmed`」已全部移除）；上游协议允许帧内容含 `0x7e`，故帧头也可为 `0x7e`（`id ∈ 120..=127`）——而**帧体 CRC 不覆盖帧头**，所以本仓另加 4 位校验（缩短汉明码、只检不纠：1/2 位错一律丢单元）。**另有四处上游没有的设备层约束（2026-10-21）**：独占打开走 `flock(LOCK_EX\|LOCK_NB)`（上游不拿锁，同一 tty 可被第二个进程改写线路参数）、端口 `stop()` 后握手**立即** `ConnectionLost`（上游要熬满 timeout）、`answers_for` 区分超时与链路断开、监听队列与每 MCU 的 `FrameStream` 有上界且满了丢帧（丢帧不堵链路） | [MCU 连接方式](../user-manual/mcu-connection.md)、[模块表](README.md) |
| 压力提前（ESPA） | `chelper/kin_extruder.c` 的 `extruder_calc_position`/`pa_move_integrate`：`pa_pos = nominal + pa·v`，再按 `smooth_time` 三角窗**解析积分**平滑；参数是 `pa_list` 链表（按 `active_print_time`），写入路为 `extruder_set_pressure_advance` 并把 `gen_steps_pre/post_active` 设为 `hst` | **主体同形**：`motion/pressure_advance.rs` 用同样的解析积分与逐段查表。三点对齐（2026-10-21 后已无偏差）：① `gen_steps_pre_active`/`post_active = hst`——`StepKinematics` 带 `set_generation_window(pre, post)`，挤出机在每次 `generate_steps` 用 PA 快照现算 `hst`（不缓存）；② 改**值**只立即写上报值，落表排队到 `register_lookahead_callback`（队列非空 ⇒ 在该 move 末尾的 print_time 生效，队列空 ⇒ 立即 `get_last_move_time()`），`smooth_time` 变更走 `flush_step_generation` + 从头重算；③ 扫描窗记账已运行期化（`ToolHead::recompute_scan_windows` 取全 stepper pre/post 最大值，`kin_flush_delay` 不再是常量地板） | [配置手册](../user-manual/config.md)、`TODO.md` H6 |
| 端口级 admin 命令族与配置期 UART 对象 | 上游串口线内没有 admin/寻址概念、也没有桥：`0x3f0` 只是 CAN 总线层的扫描/引导对话 | 本仓在**桥自己**的 admin 通道上定义了 `SET_PORT_NODEID (0x03)`、`RESET_PORT (0x04)`、`REQUEST_BOOTLOADER (0x02)`、`RESTART_DEVICE (0x05)`（面向桥自己、**全无回执**、参数不足/越界静默丢弃；`index` 指不到本机在用端口时**穿透**给本机设备半），端口则在**配置期**由主机用 `config_uart oid=%c baud=%u slot=%c`（UART 口）/ `config_can oid=%c bitrate=%u slot=%c`（CAN 口，**速率运行期给**）+ `config_bridge_port oid=%c port=%c` 声明成**端口对象**（`slot` = 端口 `index` = 端口节声明顺序；这一层本仓叫 `port` 而不是 `uart`，因为它装的不只是 UART）；**没有**编译期端口表，也没有 identify 字典端口表，端口 tty 由 `[uart_device] uart_path:` 在库 `init` 前**运行期**登记（不是编译期烘死）——上游的串口固件既没有「端口」也没有「按主机声明的 UART 对象」这一层 | [串口复用协议](serial-mux-protocol.md)、[MCU 连接方式](../user-manual/mcu-connection.md) |
| mux 的固件载体与身份来源 | 上游固件没有任何 mux；控制台的线格式就是裸帧 | 本仓把 mux 接进了五处载体：**通用串口控制台**（`src/generic/serial_irq.c`，`MUX_SERIAL` 的载体条件是 `SERIAL \|\| USBSERIAL`）、**USB CDC 控制台**（`src/generic/usb_cdc.c`）、**CAN 节点字节流**（`src/generic/canserial.c`，`MUX_CAN`）、**linux 进程控制台**（`src/linux/console.c`，身份来自命令行 `-U <12 hex>`）与 **host 库控制台**（`HOST_MUX_SERIAL`，身份来自 `klipper_host_set_uuid`）；身份统一是 chip id 的 6 字节派生（`src/generic/chipid_uuid.h`，与 CAN uuid 同一实现同一 seed）。**都不默认开**：选了它，控制台线格式就不再是上游 klippy 主机说的裸帧，必须是刻意选择 | [串口复用协议](serial-mux-protocol.md) |
| `[mcu] host_library` 的身份与 mux 判定 | 上游没有 host 库这个接口，更没有"这条库路径是否按 id 寻址"的概念 | 本仓允许 `host_library:` 节写 `uuid:`/`canbus_uuid:`（指明该路径上被寻址的设备），并在**配置层**判定该路径的帧是否带 mux 头（`mcu/host_libraries.rs`：1 节无 `uuid` = 裸帧；1 节有 `uuid`、或同路径 ≥2 节 = 按 id 寻址；≥2 节时最多一节可省略 `uuid` 认领桥自身设备；≥2 节都省略、或两节同 uuid 在装载期报错）。固件侧 host 库另有**单元收发接口**（`klipper_host_input_unit`/`klipper_host_output_unit`：一次一整单元，头 + 帧，需帧接口开关 **且**桥模式）、**身份投递**（`klipper_host_set_uuid`，必须在 `init` 前）与**端口 tty 登记**（`klipper_host_port(slot, path)`，同样必须在 `init` 前；槽数 `KLIPPER_HOST_PORT_SLOTS = 4`，未登记的槽回落构建期默认路径）——上游的 host 库 API 只有字节流与整帧两种。另外本仓的 host 库**不再限制一份**（2026-10-16 起：同路径多节仍只加载一次并共享引用，但同进程可跑多份不同文件的拷贝；按路径拒绝第二条的两处守卫已去掉，见[串口复用协议](serial-mux-protocol.md)的设计规则总表）。**判定为 mux 的路径连接时按 id 寻址**（装载期为该路径指派一条**确定性身份**——`FNV-1a(路径)` 并跳过配置里所有已知 uuid，跨重启稳定；`init` 前投递给库；`SET_NODEID` + 重确认握手；收发带帧头），非 mux 路径仍是裸帧 | [MCU 连接方式](../user-manual/mcu-connection.md)、[串口复用协议](serial-mux-protocol.md) |
| `restart_method: admin` 用在**桥自己**的 `[mcu]` 上 | 上游的 `admin` 只对桥端口子节有意义（由桥转发给下游设备）；本仓额外允许串口 `[mcu]` 写 `admin`，语义是**连接前**在对端 admin 通道上发 `[0x05][0xFF]`（固件把超出端口数的 index 当"本机"），因此不需要 identify，卡在陈旧会话的板子也能靠它自恢复 | [MCU 连接方式](../user-manual/mcu-connection.md#固件重启方式restart_method) |
| 身份键 `uuid` 与扫描输出 | 上游身份键叫 `canbus_uuid`，`canbus_query.py` 输出 `Found canbus_uuid=…`；本仓规范名为 `uuid`（`canbus_uuid` 仍兼容、同节双写报错），`klipperx admin scan` 输出 `Found uuid=…`，重复报文文案 `Duplicate uuid`（上游 `Duplicate canbus_uuid`） | 语义与格式完全一致，仅键名/前缀换字：现有 Klipper 配置照用（`canbus_uuid` 读得动）。见[配置手册](../user-manual/config.md)、[MCU 连接方式](../user-manual/mcu-connection.md) |
| `steppers` 的资格过滤 | 未实现（本仓 homing 事件无载荷） | [配置手册](../user-manual/config.md)、[模块表](README.md) |
| TMC 的活线与运行期链路 | **已补齐并真机验过（2026-10-18，F446 + TMC2209 单线 UART）**：`DUMP_TMC` 读到芯片真值（`IOIN.version=0x21`）、`SET_TMC_CURRENT` 写进去、`MSCNT` 随转动变化、`mcu_phase_offset` 有了写入者。行为偏移四条：① **写要等固件自己的 `tmcuart_response`**（写也回一个空载荷应答，那是「这帧已发完」的完成信号；上游靠阻塞传输天然有它，本仓不收集就会读到写前的旧值）；② SPI 的「写 + 回读」仍是 `spi_send` 紧跟一次 `spi_transfer`（两条消息、两个 ack）——**入口已具备但 TMC 未切**：`McuSpi::transfer_with_preface` 已按上游形状实现（`spi_send` + `spi_transfer` 并进一个 block、一个 ack，片选两次，2026-10-21），TMC 的收敛留在后续单元；③ `TmcRegister.receive_time` 取调用返回后的 reactor 时刻再换 print_time，上游取响应的 `#receive_time`，最多差一个往返；④ `tmc/stallguard_dump` 的读路径按芯片能力判定（`DRV_STATUS.sg_result` 或 `SG_RESULT`/`SG4_RESULT`），上游按 `spi_status` 猜出的 `optimized_spi` 判定（本仓无该字段，节拍统一 5 ms）。另：UART 交换超时 **500 ms**（上游无限等）；both-edge（`dedge`）**已按上游 `stepper.py:82-113` 判定链启用（2026-10-21）**——默认固件 `STEPPER_OPTIMIZED_UNSTEP=1`（`Kconfig` 默认 y）走第 4 级否决，仍单沿；缺 `configfile.deprecate_mcu_code` 的 seam，用同文案 `tracing::warn!` | [配置手册](../user-manual/config.md)、[模块表](README.md)、[真机记录](../../TESTING.md) |
| `TEMPLATE=` 形式的菜单/显示选项 | 未实现（菜单不渲染） | [配置手册](../user-manual/config.md) |
| MCU `output` 表的异步投递等 | 见该页「当前未实现」 | [MCU 协议](mcu-protocol.md#当前未实现) |
| IDEX 第二滑车（`idex_modes`）的步进 | **已修（2026-10-21）**：接线照上游——滑车 rail 与 `stepper_x` **同一个 trapq/queue、各自 solver**（`cartesian.py:28-38`），不是 extra axis。真因两层：`idex_modes` 建的 `PrinterStepper` 未注册成对象 ⇒ 它自己的 `connect` 不跑、宿主 `Stepper` 不存在；且无人交给 toolhead。现按 `dual_carriage_stepper <id>` 隐藏对象注册、`toolhead` 在 `claim` 成立时 `add_stepper` 进主 trapq。**仍缺**：模式层 scale/offset（激活/去激活）、hybrid-corexy 族、`G28` 时第二滑车不被自身端停停住 | `src/core/klippy/extras/idex_modes.rs`、[模块表](README.md) |
| 事件名无发送方的其余项 | `stepper:sync_mcu_position`、`stepper:set_dir_inverted`、`menu:populate`/`init`/`begin`/`exit` | [事件系统](event-system.md#22-覆盖范围) |

## 4. 本仓特有（上游没有）

这一节反过来：**上游没有、本仓自己加的**内容。做第三方兼容性判断时（比如「某个客户端会不会
看到多余的东西」）要知道它们存在。

| 项 | 上游 | 本仓 | 详情 |
|---|---|---|---|
| 身份查询命令 `get_mcu_id`（`get_canbus_id` 为别名） | 上游只有 CAN 的 `get_canbus_id`（`klippy/serialhdl.py:158-167` 在 identify 之后比对 uuid），串口/mux 侧**没有任何身份查询** | 本仓把它改名成 `get_mcu_id` 并**两个名字都声明**（CAN 与 mux 载体同一 handler、同一 `canbus_id` 应答），把同一语义扩展到**所有 id 寻址设备**；主机在 identify 拿到字典后选名（有 `get_mcu_id` 就用它，否则回退别名）。旧主机仍能问 `get_canbus_id`。只在分配后读回做一次比对：分配期扫描对「已被命名的设备沉默」这一固件行为无法区分「不在」与「已命名」，会误杀正常启动 | [串口复用协议](serial-mux-protocol.md#身份证据一条命令两个名字所有-id-寻址设备同语义2026-10-22) |



### 4.1 配置节与命令

| 项 | 位置 | 干什么 | 上游对应物 |
|---|---|---|---|
| `[i2c_device <name>]` | `src/core/klippy/extras/i2c_device.rs`、[配置手册](../user-manual/config.md) | 通用原始 I2C 设备节：只搬字节，不含设备协议 | **无**（上游各 I2C 器件各自走 `bus.MCU_I2C_from_config`） |
| `[uart_device <name>]` / `[can_device <name>]` | `src/core/klippy/mcu/bridge_ports.rs`、[配置手册](../user-manual/config.md#uart_device-名--can_device-名--桥下游端口) | 桥下游端口节：声明端口长在哪颗桥 MCU（`*_mcu`）与它的 oid 名（`*_oid`），子设备按名引用 | **无**（上游没有桥/端口模块化概念，MCU 直连一条线） |
| `[mcu]` 的 `uart:` / `can:` 键 | `src/core/klippy/config/mcu.rs`、[MCU 连接方式](../user-manual/mcu-connection.md#桥下游端口uart--can-引用端口节) | 用端口节名声明这颗 MCU 挂在桥的哪条下游口；`can:` 必带 `uuid:`，`uart:` 无 `uuid:` 即 legacy | **无**（上游 `[mcu]` 只有 `serial`/`canbus_uuid` 两类连接键） |
| `[spi_device <name>]` | `src/core/klippy/extras/spi_device.rs`、[配置手册](../user-manual/config.md) | 通用原始 SPI 设备节：一次传输 = 一个片选脉冲 | **无**（上游走 `bus.MCU_SPI_from_config`） |
| `IIC_WRITE` / `IIC_READ` / `SPI_TRANSFER` / `SPI_SEND` | 同上两模块、[G-Code 命令参考](../user-manual/gcode-commands.md) | 真机自测用的总线调试命令 | **无**（上游全树无这些名字） |
| 其余调试总线命令 | [G-Code 命令参考](../user-manual/gcode-commands.md) 的「调试总线命令（KlipperX 自有）」一节 | 真机自测/诊断 | **无** |

### 4.2 二进制、客户端与诊断工具

| 项 | 位置 | 干什么 | 上游对应物 |
|---|---|---|---|
| `klipperx`（多子命令 CLI） | `src/main.rs`、[开发手册首页](README.md#二进制) | 主 CLI：默认跑主机，另有 `api` / `console` / `stress` 子命令与 `--tui`；**板卡直联的四个动作全在 `admin` 下面**——`admin scan` / `admin flash` / `admin reset` / `admin bridge {config,port}`（顶层没有 `scan`/`flash`），`admin` 里的命令都不读 `printer.cfg`、一条命令一件事 | 主机 ≈ 上游 `klippy.py` 脚本（无子命令）；子命令与 TUI 上游没有 |
| `klippy` 二进制 | `src/bin/klippy/main.rs`、[开发手册首页](README.md#二进制) | 只跑主机、不链客户端库 | 上游是脚本，不是二进制 |
| `klippy-client`（`api` / `console`） | `crates/klippy-client/`、[客户端使用](../user-manual/client.md) | 独立 API 客户端 | **无**（上游 `klippy/console.py` 是 MCU 调试台，不是 API 客户端） |
| `--tui` 进程内窗口 | `crates/klippy-client/src/tui.rs`、[客户端使用](../user-manual/client.md) | 通过进程内管道对自身 API 开一个客户端窗口 | **无** |
| `klipperx stress` | `src/stress.rs`、[压力测试](stress.md) | 逐步给一块 MCU 加载，直到它出错 | **无** |
| `--logfile`（澄清项） | [日志与调试](../user-manual/logging.md) | 日志同时写终端与文件 | **上游有** `-l` / `--logfile`（`klippy.py:272`）——此条列出只为消除误解 |
| `restart_method: rpi_usb` 的 udev 脚本（澄清项） | `scripts/klipperx-usb-udev.sh`、[MCU 连接方式](../user-manual/mcu-connection.md) | 生成 hub 供电授权规则 | 机制上游有（`mcu.py`）；**脚本**是本仓加的 |
| `scripts/pyref-audit.py` | 本页 §6、[测试](testing.md) | 校验文档/注释里的上游行号引用是否还对上 | **无** |
| `scripts/kx-sched.py` | [延迟与抖动](latency.md#两个可重跑的测量脚本) | 量**宿主线程**的运行队列等待（两次 `/proc/<pid>/task/*/schedstat` 差分） | **无** |
| `scripts/stats-jitter.py` | 同上 | 把 `klippy.log` 里的 `stats count=/sum=/sumsq=` 换成固件每轮耗时 avg/sd/awake | **无**（上游 `scripts/graphstats.py` 只吃它自己的 `Stats` 行且要 `print_time`） |

### 4.3 测试与构建基建

| 项 | 位置 | 干什么 |
|---|---|---|
| `KLIPPERX_*` 环境变量族 | [`README.md` → 环境变量](../../../README.md#环境变量开发与测试)、[测试](testing.md) | 构建/测试/诊断开关（语料范围、字典集、TRACE、硬件串口…） |
| 顶层固件构建（根 `Makefile` / `Kconfig` / `scripts/firmware-config.py`） | `Makefile`、`Kconfig`、`kconfig/katapult.kconfig`、[构建 MCU 固件](../user-manual/firmware-build.md) | 一次共享配置编 klipper 与 katapult 两份：根 `Makefile` 只是包装调用两个子模组的 Makefile（`OUT`/`KCONFIG_CONFIG` 外移到 `out/firmware/`，子模组只读），配置由根 `Kconfig`（klipper 树 + katapult 专属菜单）解析后投影/派生。上游只有单目标构建 |
| 上游语料复用 harness | `src/core/klippy/upstream.rs`、[回归测试](regression-tests.md) | 把上游 `test/klippy/*.test` 当只读 fixture 跑（上游只有自己的 `test_klippy.py`） |
| `crates/test-support` | [回归测试](regression-tests.md)、[测试](testing.md) | 构建期按 `KLIPPERX_ARCHES` 编 `.dict`、解析 klipper 检出位置；`PrivateCopy`（把构建产物复制到自己临时路径）：host 库注册表按路径单例，所以不捕 stderr 的用例靠它拿到**自己的库实例**从而免持 `HOST_LIBRARY_TEST_LOCK` |
| 字典驱动应答机 `SimulatorDevice` | `src/core/klippy/interface/devices/simulator.rs`、[回归测试](regression-tests.md) | test-only 假 MCU：按字典**应答** identify/配置握手/时钟/序号，走真实主机路径（上游回归是 `-d`/`-o` 短路、不回应，语义不同） |
| `KLIPPERX_TRACE` 的 `SIM-DIAG:` / `MCU DROP` 诊断行 | `src/core/klippy/mod.rs`、[日志与调试](../user-manual/logging.md) | 追模拟器/MCU 拆除时的挂起 |

### 4.4 桥与串口单元的固件侧（上游没有）

这三项都是**固件**侧的东西（`third_party/klipper` 与 `third_party/katapult` 子模组）；上游既没有桥也没有
单元格式，所以它们不存在「上游对应物」一列可写，也不属于 §3 的未实现清单——它们已经落地。
| 桥的三个端口后端与 `BRIDGE_UART_COUNT` | 上游没有桥，也没有「一个 UART 对象该接到哪条流」这一层 | 固件里四个后端各自实现 `port_backend_open`/`port_backend_port_ops`：`src/stm32/bridge_serial.c`（总开关 `CONFIG_STM32_BRIDGE_SERIAL` 下**多选** USART1/2/3、UART4/5，勾几个就有几个口，槽序 = 选中的 USART 按编号升序）、`src/linux/bridge.c`（`KLIPPER_BRIDGE_PORT_0` 一条静态流）、`src/host/bridge.c`（每槽一路流，`KLIPPER_HOST_PORT_SLOTS` = 4）。四个后端（含 `generic/bridge_can.c`：一个 bxCAN 控制器当口，槽排在 USART 之后，`RESERVE_PINS_bridge_can`；端口型 build **不是 CAN 节点**，所以 `CANBUS` 保持 n、`CANBUS_FREQUENCY` 常量不声明，驱动由 `BRIDGE_PORT_CAN` 的 Makefile 行拉入，速率由 `config_can` 经 `canhw_set_rate` 运行期设）各自 `DECL_CONSTANT("BRIDGE_UART_COUNT", …)`（选中个数 + CAN 槽 / 1 / 4），主机据此列 `--port`——固件不再报端口表；stm32 还逐个 USART/CAN 报 `RESERVE_PINS_bridge_serial*` / `RESERVE_PINS_bridge_can`（`RX,TX`；CAN 那对由 `src/stm32/can.c` 在 `CONFIG_BRIDGE_PORT_CAN` 下声明，`generic/bridge_can.c` 不再声明——一个引脚只能有一个 owner），`admin bridge config` 的槽名就来自它。另外两件事上游也没有：**端口表只在桥的 RAM 里**（复位即空，每条命令都得重新声明），以及 `config_uart`/`config_can`/`config_bridge_port` **打开即生效、无 finalize**（`src/port.c`） | [串口复用协议](serial-mux-protocol.md)、[MCU 连接方式](../user-manual/mcu-connection.md) |
| 口形态（UNITS/legacy）与隧道上行 | 上游的串口只有一个线格式 | 口形态是**主机每次命令的选择**（`SET_PORT_NODEID` 的 `flags` bit0 `MUX_PORT_UNITS`），不是编译期事实；隧道模式（`0x06`）与它正交。**UNITS 口也能上行隧道块**：`SET_PORT_NODEID` 无论 `flags` 都记 `node_id`，所以 `bridge_port_tunnel_up` 在两种口上都能给块加应答头（此前只在 legacy 口记 id，UNITS 口的隧道上行静默丢；2026-10-17 修） | [串口复用协议](serial-mux-protocol.md) |
| **katapult fork 的 mux 构建**（`MUX_SERIAL`） | 上游 katapult 的串口控制台只会说裸帧，因此只能直连刷写 | 本仓的 katapult 子模组新增 `src/mux.c` / `src/mux.h` / `src/generic/mux_chipid.c`，把**引导加载程序自身**接进同一套单元：单元头、`KXID` admin 信道（`0x00`/`0x01`/`0x02`/`0x05`）、自己的帧出线时加 `reply_arb_id` 应答头（无 id 时不加）、身份 = `fasthash64(芯片 UID, 0xA16231A7)` 低 6 字节（与 klipper/CAN 同派生）。**默认 n**（`src/Kconfig`，`depends on SERIAL && HAVE_CHIPID`），帮助文本自己声明与上游 flashtool 不兼容——它只服务「经桥寻址的部署」。根构建在串口目标上把 klipper 的 `MUX_SERIAL` 同步给它（`make menuconfig`） | [串口复用协议](serial-mux-protocol.md)、[TESTING.md](../../../TESTING.md) |

| **CAN 控制器当桥的下游口，以及端口上的 CAN admin 代理** | 上游的 CAN 只有两种身份：本机控制台（`CANSERIAL`）或 USB-CAN；「一块板把 CAN 控制器**转给桥**」不存在，把 `0x3f0`/`0x3f1` 的单帧对话**双向翻译**成 mux 单元更不存在 | `generic/bridge_can.c`：`write` 解单元头得 arb 后**剥头**，只把 klipper 帧按 8 B 切片赶上总线（**单元头不上线**：在 CAN 上地址已由 arb 承载，头是主机↔桥那一段的寻址层；legacy/隧道口本来无头，用端口 `node_id`），收侧按 node 分桶重组后交路由器；`SET_PORT_NODEID` 经 `bridge_port_ops.node_id` 钩子装过滤器（`canhw_add_filter`，bit0 掩码一格盯 `id`/`id+1` 一对）；**单帧 admin 代理**：KXID `0x00`/`0x01`（+`0x11`）/`0x02` → `0x3f0` 单帧，翻译即全部下发（不再重发单元），`0x3f1` 的 `[0x20][uuid][app]` → 带端口路径的单元上行；**唯一**整单元下发的 admin 命令是没有 CAN 原生形态的 `0x05 RESTART_DEVICE`（头在那里是通道标记，不是地址）。另加 `canhw_set_rate()`（运行期改 BTR 并重装过滤器）与 `CANBUS_FILTER_MULTI` 能力位。**F1 上 `STM32_BRIDGE_CAN_*` `depends on !(MACH_STM32F1 && USBSERIAL)`**（USB 与 CAN1 RX0 共用向量 20，与上游 `HAVE_STM32_USBCANBUS depends on !MACH_STM32F1` 同因） | [串口复用协议](serial-mux-protocol.md)、[MCU 连接方式](../user-manual/mcu-connection.md) |
| **console 的 ack 预留与 mux 应答缓冲（上游 bug 的修复）** | 上游同样有：`encode_acknak` 只声明 `max_size`（`min_size` 为 0），`console_sendf` 便按 0 预留空间，`transmit_room()` 压缩后可以把 5 字节的 ack 放到缓冲区之外（`transmit_buf[96..100]`）；`mux_need_nodeid_via` 的答案缓冲也比最坏情况少一字节 | 本仓修了两处：`encode_acknak` 声明 `min_size = MESSAGE_MIN`，`command_encodef` 的 ack 短路也遵守 `buf_size`（放不下就丢 ack，不写越界）；`mux_need_nodeid_via` 的缓冲按「标记 + uuid + app + 满路径 + 两个追加字节」取足。**这是真机压测发现的**：250 kbaud 饱和链路上 ack 几乎每次都走压缩分支，越界的最后一个字节（帧尾 sync `0x7e`）正好落在 `mux_identity[0]`，把设备的 mux 身份从 `97abe031988e` 改成 `7eabe031988e`——之后主机拿配置里的旧 uuid 指派不生效，重连永远握不上手，只有重新上电才恢复 | [串口复用协议](serial-mux-protocol.md)、[MCU 连接方式](../user-manual/mcu-connection.md) |

### 4.5 比上游多出来的状态键与日志前缀（接口形状）

三处**只增不减**的差异，写到客户端兼容判断时要考虑：

1. **新对象 `system_stats` 的 `dropped_logs` 与 `dispatch` 两个子键**。对象名与三个键
   （`sysload`/`cputime`/`memavail`）与上游 `extras/statistics.py` 一致，但上游没有
   `dropped_logs`（日志队列丢弃计数）与 `dispatch`（reactor 迟到统计）。不读它们的客户端会直接忽略。
   字段定义见[对象状态说明](../third-party-dev/api-reference.md#system_stats)；代码在
   `src/core/klippy/extras/system_stats.rs` 的 `SystemStats::get_status`。
2. **`stats` 日志行带 `[<mcu>] ` 前缀**（上游是裸的 `stats count=%u …`）。只影响读日志的人与脚本，
   不影响线协议；双 MCU 的日志因此可以归族。
3. **`last_stats` 的键名曾与本仓文档不同**：本仓曾对外报 `mcu_tick_avg`/`mcu_tick_stddev`/
   `mcu_tick_awake`，现已改成上游名 `mcu_task_avg`/`mcu_task_stddev`/`mcu_awake`（数值公式一直与
   上游相同，见 `third_party/klipper/klippy/mcu.py` 的 `MCUStatsHelper.stats`）。同时补上了上游
   也有的 `srtt`/`rttvar`/`rto`（秒，首样本之后才出现）。`bytes_retransmit`/`bytes_invalid`/
   `send_seq`/`receive_seq`/`retransmit_seq` 仍**不上报**：它们属于上游 C 侧 `serialqueue`，本仓的
   传输层没有等价计数器，报 0 比不报更容易误导。

### 4.6 配置 `[include]`：URL 取源与自包含保护

上游 `_resolve_include`（`third_party/klipper/klippy/configfile.py:181-195`）只有「路径 + `glob.glob`」，
本仓在其上加了两件事（代码在 `src/core/klippy/config/mod.rs` 的 `resolve_include_sources`）：

1. **`[include http(s)://…]` 直接取远端正文**。这是新能力，上游没有。边界：总超时 10 s、
   连接 5 s、正文上限 4 MiB（边读边判），成功时 `info!` 留一行（URL + 字节数）。
   配置侧行为见[配置手册](../user-manual/config.md#include引入其它文件)。
2. **自包含保护**。通配符展开会把「包含者自己」也匹配进来（`[include *.cfg]` 写在同目录），
   上游没有环检测 ⇒ 无限递归；本仓报 `Circular include detected: <source>`。

除这两点外，展开语义与上游一致：相对包含者目录、`*`/`?`/`[]` 非递归、**不匹配前导点文件**、
排序后逐个读、有通配但零匹配时静默不报错。

### 4.7 一条负向结论

**API 端点没有本仓特有项**：`src/core/klippy/api/endpoints/` 注册的 13 条 path 全部能对上上游
`klippy/webhooks.py`（以及 `query_endstops` / `pause_resume` / `bed_mesh` 各自的 extras）的注册处，
没有上游没有的端点；`gcode/restart` 与 `gcode/firmware_restart` 是两条 path、同一个实现结构。

## 5. 计数与清单（改了要连带重核的地方）

| 数字 | 位置 |
|---|---|
| 上游语料声明运行数 / 通过数 / 跳过数 | [`README.md` → 当前状态](../../../README.md#当前状态) |
| 端点总数（16 条注册路径 + 内建 `list_endpoints`） | [模块表](README.md) |
| 事件总数与"已发出/无发送方" | [事件系统](event-system.md) |
| `[extras]` 模块数与 `section!` 数 | [`README.md` → 当前状态](../../../README.md#当前状态) |
| 二进制体积 | [模块表](README.md) |

## 6. 上游行号引用的核验

本仓文档与源码注释里大量引上游 `xxx.py:NN-NN`。改动上游相关行为后这些行号会漂移，
核验方式是 `python3 scripts/pyref-audit.py --docs`（以及不加 `--docs` 扫源码），细则与
噪声说明见[测试 → 文档同步](testing.md#文档同步)。

---

- [← 开发手册首页](README.md)
