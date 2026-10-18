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
| `[display]` 菜单 | 选项只被读取，**菜单内容不渲染**（LED/屏输出本身可用） | [配置手册 `[display]`](../user-manual/config.md) |
| 上游无、本仓有 | 通用 `[i2c_device]` / `[spi_device]` 节与 `IIC_WRITE` / `IIC_READ` / `SPI_TRANSFER` / `SPI_SEND` 调试命令、`KlipperX` 自有调试总线命令 —— 属**本仓扩展**，上游没有对应物 | [配置手册](../user-manual/config.md)、[G-Code 命令参考](../user-manual/gcode-commands.md) |

## 2. 行为偏移（同功能，两种表现）

| 项 | 上游 | 本仓 | 详情 |
|---|---|---|---|
| 脚本执行与错误策略 | 行协议输入下错误不中断；`run_script` 持 `gcode` mutex 串行化 | 只有"报告 + 中止整段脚本"一种策略（`need_ack` 已删）；脚本在调用任务上跑完，不再有 mutex | [klippy-runtime](klippy-runtime.md)、`src/core/klippy/gcode.rs` 模块文档 |
| `start_args` 字段 | 上游有 `debuginput`（`-i`）与每 MCU 字典路径（`-d`） | 本仓没有 `debug_input` 字段与字典路径（不实现文件输入 / batch mode）；`debug_output` 保留（语料 harness 的文件输出开关） | [klippy-runtime](klippy-runtime.md)、[回归测试](regression-tests.md) |
| mux 命令的"值不合法"提示 | 按 dict 迭代序取最后一个匹配候选 | 对候选排序后取第一个（消息稳定） | [模块表](README.md) |
| 越界报错的限制值写法 | `0.0` 这类浮点写法 | 整数限制值打印成 `0` | [模块表](README.md)、[配置手册](../user-manual/config.md) |
| MCU 配置 CRC | 上游的 CRC 算法 | **算的不是同一样东西**（本层唯一有意偏离） | [MCU 配置构建](mcu-config.md) |
| identify offset 不匹配 | 不追加数据、用同一 offset 重试（可无限循环） | **落后**的块同样不追加、重发同一 offset（只是用每块的超时预算兜底）；**超前**的块直接报错 | [Identify 机制](identify.md)、[klippy-runtime](klippy-runtime.md) |
| 事件处理器里的配置错误 | `send_event` 向上抛（`_handle_mcu_identify` 里的 `add_stepper` 拒绝直接中断 bring-up） | 事件总线隔离处理器，`load_cell_probe` 的 Z 轴注册遇「共享轴跨 MCU」只 `tracing::error!` 记录后继续 bring-up（同一错误在端停路径上仍是配置错） | `src/core/klippy/extras/load_cell_probe.rs`、`event-system.md` |
| 等待与回调模型 | greenlet + `reactor.pause()` / `completion`（含 `register_fd`） | async/await + 定时器表；`set_latency_notifier` 纯诊断 | [reactor](reactor.md)、[klippy-runtime](klippy-runtime.md) |
| section → 对象表 | 运行时按文件导入 | 编译期由 `section!` 声明生成 | [声明式表生成](codegen.md) |
| 事件系统 | 运行时字符串事件、`*params` 可变参数、`try/except` 隔离、`assert_no_pause()` 强制 | 编译期枚举、变体字段、`catch_unwind`、不做运行时阻塞校验 | [事件系统 §6](event-system.md#6-与上游的差异) |
| 模板引擎 | Jinja2 | minijinja 适配层：`%`/`//` 取余、Strict 缺键时机、`|int`/`|float` 无默认时的不可转换值、`|min`/`|max` 空序列、部分 detail 措辞不同；标签输出走**自定义 formatter**（容器按 Python `str()` 拼写，`~` 拼接仍走引擎 Display）；`"…%s…" % x` 由编译期**源码重写**成 `|format(...)`（只收左侧为字符串字面量的形式，`%(key)s` 映射形态不支持）；`str`/`list` 的 Python 方法由 unknown-method callback 提供，而 **`append`/`extend`/`pop` 降级**（值语义，返回 `None` 且列表不变，收集写法拿到空列表） | [模块表](README.md)、[配置手册](../user-manual/config.md)、[G-Code 命令参考](../user-manual/gcode-commands.md) |
| 宏变量与 `SET_GCODE_VARIABLE` 的字面量 | Python `ast.literal_eval`（`gcode_macro.py:153-162`） | 同一解析器（`py_literal`）另外接受 JSON 写法 `true`/`false`/`null`（旧行为保留）；dict 的数字/布尔 key 按上游 `json.dumps` 的形态转字符串 | [模块表](README.md)、[配置手册](../user-manual/config.md) |
| 无配置节也加载的模块 | `toolhead.py:604-614` 无条件 load `gcode_move`/`homing`/`idle_timeout`/`statistics`/`manual_probe`/`tuning_tower`/`garbage_collection` | 只补了 `idle_timeout`（`load.rs` 的 `DEFAULT_MODULES`，缺失的节交给同一个工厂读默认值）；其余四个本仓尚无对应实现，属未覆盖而非有意偏移 | [模块表](README.md)、[配置手册](../user-manual/config.md) |
| 运动数学与步进压缩 | `chelper/` 的 C 代码 | Rust 重写；回零/探针位置按**指令位置**返回（上游用触发步数反算，见 TODO H9 的精度单元）、`home_start` 的 `rest_time` 硬编码 | [klippy-runtime](klippy-runtime.md)、[模块表](README.md) |
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
| `trsync_state` 的报告处理 | 每次 `MCU_trsync.start()` 重新绑处理器（`mcu.py:250-260`），`can_trigger=0` 且无待完成 completion 的迟到报告静默忽略（`:228-240`）；固件无 `trsync_stop`，周期报告只由 `trsync_trigger`/停机清 | 绑定提前到**会话 `attach` 时一次装好、整会话不解除**，无主 oid 静默；`on_restart` 列表发 `trsync_start(oid,0,0,0)` 断掉上一轮遗留的周期报告（2026-10-18）。行为对齐，实现上不靠每次 start 重绑 | [模块表](README.md)、[MCU 连接](../user-manual/mcu-connection.md) |
| 归零等待的兜底 | `homing.py` 等触发带超时（`TRSYNC_TIMEOUT`），超时报 `Timeout during homing` | 同语义（2026-10-18）：5 s 兜底 + 收尾发 `trsync_trigger` + **失败释放 gcode 锁**；此前无界，丢一份报告就把整台机器锁死（连 `M112` 都排不进去） | [模块表](README.md) |
| TMC 相位偏移的读取时机 | `_handle_sync_mcu_pos` 用 `stepper.get_mcu_position()`（solver 位置，永远可读） | 同语义，但本仓现取 `ToolHeadObject::stepper_mcu_position`，而 `probing_move`/`flush_step_generation` 把 toolhead 的 `Connected` 整个 `take()` 出去 → 使能回调恰好在其间拿到 `None`（`Unable to obtain tmc … phase`，53 µs 早退）。现改取 solver 位置（2026-10-18） | [模块表](README.md)、[TESTING](../../../TESTING.md) |
| MCU 连接 `restart_method` | 值域 `command`/`arduino`/`cheetah`/`rpi_usb`，都是「复位这条连接」 | 多一个上游没有的 **`admin`**：经桥转发 `RESTART_DEVICE (0x05)`，让**桥下游的命名设备**重启自己的固件（进引导的 `REQUEST_BOOTLOADER (0x02)` 属刷写工具、**不属** `restart_method`）；桥下游 legacy 设备的线级复位改由**端口节**的 `uart_restart_method` 决定；共享串口仍在解析层被强制成 `command` | [MCU 连接方式](../user-manual/mcu-connection.md)、[串口复用协议](serial-mux-protocol.md) |
| 固件构建的源文件去重 | 每个 `src-$(CONFIG_*) +=` 各列各的；各载体在上游天然互斥，从不重复 | 架构 makefile include 之后加一行 `src-y := $(call uniq,$(src-y))`（**保序去重，不是 `sort`**）：本仓的 `MUX_SERIAL` 可与 `USBSERIAL` 同开，多个载体都会列 `<arch>/chipid.c` ⇒ 重复链接（并让 `compile_time_request` 计入两次）。**保序本身是 ABI**：字典的 message id 按 `src-y` 顺序发放（`buildcommands.py`），排序会把每个目标的字典重新编号，而测试、golden 数据与已记录的线上形状都引用那些 id。顶层 `Makefile` 只包装调用它（`OUT`/`KCONFIG_CONFIG` 外移，见 4.3），不改 `src-y` | `third_party/klipper/Makefile` |
| API 若干已知差异 | 上游 `machine`/`calibration`/`get_status` 等更完整 | 缺 `probe_path`/`rapid_path`、部分 `get_status` 字段未返回等 | [API 参考](../third-party-dev/api-reference.md) |

## 3. 暂缓/未实现（本仓没做，方向上不排除）

| 项 | 现状 | 详情 |
|---|---|---|
| 上游 host `Stats …` 日志行 / `gcodein` | 无 host 统计采集器 | 本条 1.1 |
| `query_adc` / `QUERY_ADC` | 未实现（`adc_scaled` 只用 config 输入量程） | [配置手册](../user-manual/config.md)、[模块表](README.md) |
| `canbus_nodeid` 选项 | 上游没有这个选项（CAN 节点号一律由 `canbus_ids` 按声明顺序发）；且上游的分配器只服务 CAN，节点号只存在于它的 CAN 连接路径里（`klippy/mcu.py:858-862` 经 `canbus_ids.get_nodeid` 交给 `serialhdl.connect_canbus`；本仓的 `Transport` 枚举是另一层说法） | 本仓保留它作显式覆盖：写了就用，且装载时**预扫全配置**把所有显式号先保留（`reserve_explicit`，读节的原始参数、不经 accessor），自动号取 `4` 起的**最小空位**并绕开它们——显式号写在哪个位置都生效，结果与声明顺序无关（上游 `len(ids)+4` 的前提是「没有显式号」，一旦有显式号这个游标就不再等价，故本仓不再用它）；装载期只在**两个显式号互撞**（点名双方）或号段 `4..=255` 占满（点名 MCU）时报配置错，而不到固件 `can_id_conflict` 才停机。更进一步：把 **node id 提升为 MCU 层 id**（`McuConfig.id`），节名改为 `[mcu_ids]`（历史拼写 `[canbus_ids]` 兼容）、分配器（`mcu/ids.rs`）泛化成所有接口共用一个编号序列——CAN 用它寻址，**共享串口**用同一套 `0x100+2*id` 做帧头寻址并在分配端口时同步握手（独占串口不使用；见[串口复用协议](serial-mux-protocol.md)）、`host_library`/`test:` 同样领到 id、只是暂不上线，`connect` 打开传输层前必须持有 | [配置手册](../user-manual/config.md)、[模块表](README.md) |
| 帧级 id 指派信道 | 上游无此机制：串口固件没有任何 id 概念，admin 帧（`0x3f0`）是 CAN 总线层的东西，上游也不在 identify 前对串口做任何指派 | 本仓在**串口线内**开一条复用信道（帧头与伪帧编解码/握手在 `devices/serial.rs`，仲裁 id 映射共用 `assign.rs`，见[串口复用协议](serial-mux-protocol.md)）：同一 `serial:` 路径被 ≥2 颗 `[mcu]` 引用时自动进入**共享模式**——帧头 `2 字节带 4 位检错的码字`（10 数据位 = 仲裁 id、应答 `+1`；**与上游的 Klipper VLQ 不同**，是本仓自定的线上格式）、单元 = 头 + 原样帧、端口只开一次/单 reader 按 id 分派/引用计数关闭；每颗 MCU 在**分配端口时**同步握手（默认 500 ms × 3，无回执即失败，identify 之前）。伪帧 `[09][20]["KXID"][crc16][0x7e]` 仍会被旧固件在 seq 检查处无害丢弃（`command.c` 可执行移植钉住），所以误配到旧固件只是超时、不搞乱会话。另**镜像 CAN 的 admin 命令族**：`0x00` 广播查询未分配设备、`0x20` 应答（uuid + application）、`0x01` 按 uuid 点名指派 id——`attach` 在节上带 `uuid` 时先发 SET_NODEID 再发重确认请求，另有 `klipperx admin scan` 工具（`--interface <bus>` 扫 CAN、`--port <tty>` 扫串口、`--bridge <tty>` 扫桥下游各口，传输必须显式给，输出同形）；上游只在 CAN 侧有这套（`canbus_query.py`），串口/桥侧没有（冲突检测留给固件端结合物理接口）。**独占串口不发任何指派帧、线上无头**（本轮之前的「fire-and-forget 指派帧 + `FrameStream` 回执支路 + `Mcu::assignment_confirmed`」已全部移除）；上游协议允许帧内容含 `0x7e`，故帧头也可为 `0x7e`（`id ∈ 120..=127`）——而**帧体 CRC 不覆盖帧头**，所以本仓另加 4 位校验（缩短汉明码、只检不纠：1/2 位错一律丢单元） | [MCU 连接方式](../user-manual/mcu-connection.md)、[模块表](README.md) |
| 端口级 admin 命令族与配置期 UART 对象 | 上游串口线内没有 admin/寻址概念、也没有桥：`0x3f0` 只是 CAN 总线层的扫描/引导对话 | 本仓在**桥自己**的 admin 通道上定义了 `SET_PORT_NODEID (0x03)`、`RESET_PORT (0x04)`、`REQUEST_BOOTLOADER (0x02)`、`RESTART_DEVICE (0x05)`（面向桥自己、**全无回执**、参数不足/越界静默丢弃；`index` 指不到本机在用端口时**穿透**给本机设备半），端口则在**配置期**由主机用 `config_uart oid=%c baud=%u slot=%c`（UART 口）/ `config_can oid=%c bitrate=%u slot=%c`（CAN 口，**速率运行期给**）+ `config_bridge_port oid=%c port=%c` 声明成**端口对象**（`slot` = 端口 `index` = 端口节声明顺序；这一层本仓叫 `port` 而不是 `uart`，因为它装的不只是 UART）；**没有**编译期端口表，也没有 identify 字典端口表，端口 tty 由 `[uart_device] uart_path:` 在库 `init` 前**运行期**登记（不是编译期烘死）——上游的串口固件既没有「端口」也没有「按主机声明的 UART 对象」这一层 | [串口复用协议](serial-mux-protocol.md)、[MCU 连接方式](../user-manual/mcu-connection.md) |
| mux 的固件载体与身份来源 | 上游固件没有任何 mux；控制台的线格式就是裸帧 | 本仓把 mux 接进了五处载体：**通用串口控制台**（`src/generic/serial_irq.c`，`MUX_SERIAL` 的载体条件是 `SERIAL \|\| USBSERIAL`）、**USB CDC 控制台**（`src/generic/usb_cdc.c`）、**CAN 节点字节流**（`src/generic/canserial.c`，`MUX_CAN`）、**linux 进程控制台**（`src/linux/console.c`，身份来自命令行 `-U <12 hex>`）与 **host 库控制台**（`HOST_MUX_SERIAL`，身份来自 `klipper_host_set_uuid`）；身份统一是 chip id 的 6 字节派生（`src/generic/chipid_uuid.h`，与 CAN uuid 同一实现同一 seed）。**都不默认开**：选了它，控制台线格式就不再是上游 klippy 主机说的裸帧，必须是刻意选择 | [串口复用协议](serial-mux-protocol.md) |
| `[mcu] host_library` 的身份与 mux 判定 | 上游没有 host 库这个接口，更没有"这条库路径是否按 id 寻址"的概念 | 本仓允许 `host_library:` 节写 `uuid:`/`canbus_uuid:`（指明该路径上被寻址的设备），并在**配置层**判定该路径的帧是否带 mux 头（`mcu/host_libraries.rs`：1 节无 `uuid` = 裸帧；1 节有 `uuid`、或同路径 ≥2 节 = 按 id 寻址；≥2 节时最多一节可省略 `uuid` 认领桥自身设备；≥2 节都省略、或两节同 uuid 在装载期报错）。固件侧 host 库另有**单元收发接口**（`klipper_host_input_unit`/`klipper_host_output_unit`：一次一整单元，头 + 帧，需帧接口开关 **且**桥模式）、**身份投递**（`klipper_host_set_uuid`，必须在 `init` 前）与**端口 tty 登记**（`klipper_host_port(slot, path)`，同样必须在 `init` 前；槽数 `KLIPPER_HOST_PORT_SLOTS = 4`，未登记的槽回落构建期默认路径）——上游的 host 库 API 只有字节流与整帧两种。另外本仓的 host 库**不再限制一份**（2026-10-16 起：同路径多节仍只加载一次并共享引用，但同进程可跑多份不同文件的拷贝；按路径拒绝第二条的两处守卫已去掉，见[串口复用协议](serial-mux-protocol.md)的设计规则总表）。**判定为 mux 的路径连接时按 id 寻址**（装载期为该路径指派一条**确定性身份**——`FNV-1a(路径)` 并跳过配置里所有已知 uuid，跨重启稳定；`init` 前投递给库；`SET_NODEID` + 重确认握手；收发带帧头），非 mux 路径仍是裸帧 | [MCU 连接方式](../user-manual/mcu-connection.md)、[串口复用协议](serial-mux-protocol.md) |
| `restart_method: admin` 用在**桥自己**的 `[mcu]` 上 | 上游的 `admin` 只对桥端口子节有意义（由桥转发给下游设备）；本仓额外允许串口 `[mcu]` 写 `admin`，语义是**连接前**在对端 admin 通道上发 `[0x05][0xFF]`（固件把超出端口数的 index 当"本机"），因此不需要 identify，卡在陈旧会话的板子也能靠它自恢复 | [MCU 连接方式](../user-manual/mcu-connection.md#固件重启方式restart_method) |
| 身份键 `uuid` 与扫描输出 | 上游身份键叫 `canbus_uuid`，`canbus_query.py` 输出 `Found canbus_uuid=…`；本仓规范名为 `uuid`（`canbus_uuid` 仍兼容、同节双写报错），`klipperx admin scan` 输出 `Found uuid=…`，重复报文文案 `Duplicate uuid`（上游 `Duplicate canbus_uuid`） | 语义与格式完全一致，仅键名/前缀换字：现有 Klipper 配置照用（`canbus_uuid` 读得动）。见[配置手册](../user-manual/config.md)、[MCU 连接方式](../user-manual/mcu-connection.md) |
| `steppers` 的资格过滤 | 未实现（本仓 homing 事件无载荷） | [配置手册](../user-manual/config.md)、[模块表](README.md) |
| TMC 的活线与运行期链路 | **已补齐并真机验过（2026-10-18，F446 + TMC2209 单线 UART）**：`DUMP_TMC` 读到芯片真值（`IOIN.version=0x21`）、`SET_TMC_CURRENT` 写进去、`MSCNT` 随转动变化、`mcu_phase_offset` 有了写入者。行为偏移四条：① **写要等固件自己的 `tmcuart_response`**（写也回一个空载荷应答，那是「这帧已发完」的完成信号；上游靠阻塞传输天然有它，本仓不收集就会读到写前的旧值）；② SPI 的「写 + 回读」是 `spi_send` 紧跟一次 `spi_transfer`（两条消息、两个 ack），上游 `spi_transfer_with_preface` 是一次片选内连发两帧；③ `TmcRegister.receive_time` 取调用返回后的 reactor 时刻再换 print_time，上游取响应的 `#receive_time`，最多差一个往返；④ `tmc/stallguard_dump` 的读路径按芯片能力判定（`DRV_STATUS.sg_result` 或 `SG_RESULT`/`SG4_RESULT`），上游按 `spi_status` 猜出的 `optimized_spi` 判定（本仓无该字段，节拍统一 5 ms）。另：UART 交换超时 **500 ms**（上游无限等）；both-edge（`dedge`）未启用（`get_pulse_duration()` 第二项恒 `false`） | [配置手册](../user-manual/config.md)、[模块表](README.md)、[真机记录](../../TESTING.md) |
| `TEMPLATE=` 形式的菜单/显示选项 | 未实现（菜单不渲染） | [配置手册](../user-manual/config.md) |
| MCU `output` 表的异步投递等 | 见该页「当前未实现」 | [MCU 协议](mcu-protocol.md#当前未实现) |
| IDEX 第二滑车（`idex_modes`）的步进 | **真实缺口**（2026-10-18 发现）：`[dual_carriage]` 的 `PrinterStepper` 从不调 `take_stepper()`、也从不进 `MotionQueuing`，所以它的 solver 没挂任何 trapq → **不但 `STEPPER_BUZZ` 不出步，这一路根本没有步进生成**。上游每个 `MCU_stepper` 建时即加入生成集，所以 IDEX 机器在那里是正常的。语料此前是「假绿」（只要求命令不报错） | `src/core/klippy/extras/idex_modes.rs`、[模块表](README.md) |
| 事件名无发送方的其余项 | `stepper:sync_mcu_position`、`stepper:set_dir_inverted`、`menu:populate`/`init`/`begin`/`exit`、`dual_carriage:update_kinematics` | [事件系统](event-system.md#22-覆盖范围) |

## 4. 本仓特有（上游没有）

这一节反过来：**上游没有、本仓自己加的**内容。做第三方兼容性判断时（比如「某个客户端会不会
看到多余的东西」）要知道它们存在。

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

### 4.3 测试与构建基建

| 项 | 位置 | 干什么 |
|---|---|---|
| `KLIPPERX_*` 环境变量族 | [`README.md` → 环境变量](../../../README.md#环境变量开发与测试)、[测试](testing.md) | 构建/测试/诊断开关（语料范围、字典集、TRACE、硬件串口…） |
| 顶层固件构建（根 `Makefile` / `Kconfig` / `scripts/firmware-config.py`） | `Makefile`、`Kconfig`、`kconfig/katapult.kconfig`、`firmware.config`、[构建 MCU 固件](../user-manual/firmware-build.md) | 一次共享配置编 klipper 与 katapult 两份：根 `Makefile` 只是包装调用两个子模组的 Makefile（`OUT`/`KCONFIG_CONFIG` 外移到 `out/firmware/`，子模组只读），配置由根 `Kconfig`（klipper 树 + katapult 专属菜单）解析后投影/派生。上游只有单目标构建 |
| 上游语料复用 harness | `src/core/klippy/upstream.rs`、[回归测试](regression-tests.md) | 把上游 `test/klippy/*.test` 当只读 fixture 跑（上游只有自己的 `test_klippy.py`） |
| `crates/test-support` | [回归测试](regression-tests.md) | 构建期按 `KLIPPERX_ARCHES` 编 `.dict`、解析 klipper 检出位置 |
| 字典驱动应答机 `SimulatorDevice` | `src/core/klippy/interface/devices/simulator.rs`、[回归测试](regression-tests.md) | test-only 假 MCU：按字典**应答** identify/配置握手/时钟/序号，走真实主机路径（上游回归是 `-d`/`-o` 短路、不回应，语义不同） |
| `KLIPPERX_TRACE` 的 `SIM-DIAG:` / `MCU DROP` 诊断行 | `src/core/klippy/mod.rs`、[日志与调试](../user-manual/logging.md) | 追模拟器/MCU 拆除时的挂起 |

### 4.4 桥与串口单元的固件侧（上游没有）

这三项都是**固件**侧的东西（`third_party/klipper` 与 `third_party/katapult` 子模组）；上游既没有桥也没有
单元格式，所以它们不存在「上游对应物」一列可写，也不属于 §3 的未实现清单——它们已经落地。

| 项 | 上游 | 本仓 | 详情 |
|---|---|---|---|
| 桥的三个端口后端与 `BRIDGE_UART_COUNT` | 上游没有桥，也没有「一个 UART 对象该接到哪条流」这一层 | 固件里四个后端各自实现 `port_backend_open`/`port_backend_port_ops`：`src/stm32/bridge_serial.c`（总开关 `CONFIG_STM32_BRIDGE_SERIAL` 下**多选** USART1/2/3、UART4/5，勾几个就有几个口，槽序 = 选中的 USART 按编号升序）、`src/linux/bridge.c`（`KLIPPER_BRIDGE_PORT_0` 一条静态流）、`src/host/bridge.c`（每槽一路流，`KLIPPER_HOST_PORT_SLOTS` = 4）。四个后端（含 `generic/bridge_can.c`：一个 bxCAN 控制器当口，槽排在 USART 之后，`RESERVE_PINS_bridge_can`；端口型 build **不是 CAN 节点**，所以 `CANBUS` 保持 n、`CANBUS_FREQUENCY` 常量不声明，驱动由 `BRIDGE_PORT_CAN` 的 Makefile 行拉入，速率由 `config_can` 经 `canhw_set_rate` 运行期设）各自 `DECL_CONSTANT("BRIDGE_UART_COUNT", …)`（选中个数 + CAN 槽 / 1 / 4），主机据此列 `--port`——固件不再报端口表；stm32 还逐个 USART/CAN 报 `RESERVE_PINS_bridge_serial*` / `RESERVE_PINS_bridge_can`（`RX,TX`；CAN 那对由 `src/stm32/can.c` 在 `CONFIG_BRIDGE_PORT_CAN` 下声明，`generic/bridge_can.c` 不再声明——一个引脚只能有一个 owner），`admin bridge config` 的槽名就来自它。另外两件事上游也没有：**端口表只在桥的 RAM 里**（复位即空，每条命令都得重新声明），以及 `config_uart`/`config_can`/`config_bridge_port` **打开即生效、无 finalize**（`src/port.c`） | [串口复用协议](serial-mux-protocol.md)、[MCU 连接方式](../user-manual/mcu-connection.md) |
| 口形态（UNITS/legacy）与隧道上行 | 上游的串口只有一个线格式 | 口形态是**主机每次命令的选择**（`SET_PORT_NODEID` 的 `flags` bit0 `MUX_PORT_UNITS`），不是编译期事实；隧道模式（`0x06`）与它正交。**UNITS 口也能上行隧道块**：`SET_PORT_NODEID` 无论 `flags` 都记 `node_id`，所以 `bridge_port_tunnel_up` 在两种口上都能给块加应答头（此前只在 legacy 口记 id，UNITS 口的隧道上行静默丢；2026-10-17 修） | [串口复用协议](serial-mux-protocol.md) |
| **katapult fork 的 mux 构建**（`MUX_SERIAL`） | 上游 katapult 的串口控制台只会说裸帧，因此只能直连刷写 | 本仓的 katapult 子模组新增 `src/mux.c` / `src/mux.h` / `src/generic/mux_chipid.c`，把**引导加载程序自身**接进同一套单元：单元头、`KXID` admin 信道（`0x00`/`0x01`/`0x02`/`0x05`）、自己的帧出线时加 `reply_arb_id` 应答头（无 id 时不加）、身份 = `fasthash64(芯片 UID, 0xA16231A7)` 低 6 字节（与 klipper/CAN 同派生）。**默认 n**（`src/Kconfig`，`depends on SERIAL && HAVE_CHIPID`），帮助文本自己声明与上游 flashtool 不兼容——它只服务「经桥寻址的部署」。根构建在串口目标上把 klipper 的 `MUX_SERIAL` 同步给它（`make menuconfig`） | [串口复用协议](serial-mux-protocol.md)、[TESTING.md](../../../TESTING.md) |

| **CAN 控制器当桥的下游口，以及端口上的 CAN admin 代理** | 上游的 CAN 只有两种身份：本机控制台（`CANSERIAL`）或 USB-CAN；「一块板把 CAN 控制器**转给桥**」不存在，把 `0x3f0`/`0x3f1` 的单帧对话**双向翻译**成 mux 单元更不存在 | `generic/bridge_can.c`：`write` 解单元头得 arb 后**剥头**，只把 klipper 帧按 8 B 切片赶上总线（**单元头不上线**：在 CAN 上地址已由 arb 承载，头是主机↔桥那一段的寻址层；legacy/隧道口本来无头，用端口 `node_id`），收侧按 node 分桶重组后交路由器；`SET_PORT_NODEID` 经 `bridge_port_ops.node_id` 钩子装过滤器（`canhw_add_filter`，bit0 掩码一格盯 `id`/`id+1` 一对）；**单帧 admin 代理**：KXID `0x00`/`0x01`（+`0x11`）/`0x02` → `0x3f0` 单帧，翻译即全部下发（不再重发单元），`0x3f1` 的 `[0x20][uuid][app]` → 带端口路径的单元上行；**唯一**整单元下发的 admin 命令是没有 CAN 原生形态的 `0x05 RESTART_DEVICE`（头在那里是通道标记，不是地址）。另加 `canhw_set_rate()`（运行期改 BTR 并重装过滤器）与 `CANBUS_FILTER_MULTI` 能力位。**F1 上 `STM32_BRIDGE_CAN_*` `depends on !(MACH_STM32F1 && USBSERIAL)`**（USB 与 CAN1 RX0 共用向量 20，与上游 `HAVE_STM32_USBCANBUS depends on !MACH_STM32F1` 同因） | [串口复用协议](serial-mux-protocol.md)、[MCU 连接方式](../user-manual/mcu-connection.md) |
| **console 的 ack 预留与 mux 应答缓冲（上游 bug 的修复）** | 上游同样有：`encode_acknak` 只声明 `max_size`（`min_size` 为 0），`console_sendf` 便按 0 预留空间，`transmit_room()` 压缩后可以把 5 字节的 ack 放到缓冲区之外（`transmit_buf[96..100]`）；`mux_need_nodeid_via` 的答案缓冲也比最坏情况少一字节 | 本仓修了两处：`encode_acknak` 声明 `min_size = MESSAGE_MIN`，`command_encodef` 的 ack 短路也遵守 `buf_size`（放不下就丢 ack，不写越界）；`mux_need_nodeid_via` 的缓冲按「标记 + uuid + app + 满路径 + 两个追加字节」取足。**这是真机压测发现的**：250 kbaud 饱和链路上 ack 几乎每次都走压缩分支，越界的最后一个字节（帧尾 sync `0x7e`）正好落在 `mux_identity[0]`，把设备的 mux 身份从 `97abe031988e` 改成 `7eabe031988e`——之后主机拿配置里的旧 uuid 指派不生效，重连永远握不上手，只有重新上电才恢复 | [串口复用协议](serial-mux-protocol.md)、[MCU 连接方式](../user-manual/mcu-connection.md) |

### 4.5 一条负向结论

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
