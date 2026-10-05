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
- `gcode:debuginput_exit` 事件只有声明、**永不触发**（见[事件系统](event-system.md#12-事件清单)）；
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
| `BED_SCREWS_ADJUST` 命令族 | 段与参数已解析，**命令族未移植**（会话起不来） | [配置手册 `[bed_screws]`](../user-manual/config.md)、[模块表](README.md) |
| `[display]` 菜单 | 选项只被读取，**菜单内容不渲染**（LED/屏输出本身可用） | [配置手册 `[display]`](../user-manual/config.md) |
| 上游无、本仓有 | 通用 `[i2c_device]` / `[spi_device]` 节与 `IIC_WRITE` / `IIC_READ` / `SPI_TRANSFER` / `SPI_SEND` 调试命令、`KlipperX` 自有调试总线命令 —— 属**本仓扩展**，上游没有对应物 | [配置手册](../user-manual/config.md)、[G-Code 命令参考](../user-manual/gcode-commands.md) |

## 2. 行为偏移（同功能，两种表现）

| 项 | 上游 | 本仓 | 详情 |
|---|---|---|---|
| 脚本执行与错误策略 | 行协议输入下错误不中断；`run_script` 持 `gcode` mutex 串行化 | 只有"报告 + 中止整段脚本"一种策略（`need_ack` 已删）；脚本在调用任务上跑完，不再有 mutex | [klippy-runtime](klippy-runtime.md)、`src/core/klippy/gcode.rs` 模块文档 |
| mux 命令的"值不合法"提示 | 按 dict 迭代序取最后一个匹配候选 | 对候选排序后取第一个（消息稳定） | [模块表](README.md) |
| 越界报错的限制值写法 | `0.0` 这类浮点写法 | 整数限制值打印成 `0` | [模块表](README.md)、[配置手册](../user-manual/config.md) |
| MCU 配置 CRC | 上游的 CRC 算法 | **算的不是同一样东西**（本层唯一有意偏离） | [MCU 配置构建](mcu-config.md) |
| identify offset 不匹配 | 不追加数据、用同一 offset 重试 | 直接报错 | [Identify 机制](identify.md)、[klippy-runtime](klippy-runtime.md) |
| 等待与回调模型 | greenlet + `reactor.pause()` / `completion`（含 `register_fd`） | async/await + 定时器表；`set_latency_notifier` 纯诊断 | [reactor](reactor.md)、[klippy-runtime](klippy-runtime.md) |
| section → 对象表 | 运行时按文件导入 | 编译期由 `section!` 声明生成 | [声明式表生成](codegen.md) |
| 事件系统 | 运行时字符串事件、`*params` 可变参数、`try/except` 隔离、`assert_no_pause()` 强制 | 编译期枚举、变体字段、`catch_unwind`、不做运行时阻塞校验 | [事件系统 §6](event-system.md#6-与上游的差异) |
| 模板引擎 | Jinja2 | minijinja 适配层（`%`/`//` 取余、Strict 缺键时机与部分 detail 措辞不同） | [模块表](README.md)、[配置手册](../user-manual/config.md)、[G-Code 命令参考](../user-manual/gcode-commands.md) |
| 运动数学与步进压缩 | `chelper/` 的 C 代码 | Rust 重写；回零单程、`endstop_phase` 未实现、`G28` 逐轴 | [klippy-runtime](klippy-runtime.md) |
| 时间敏感回调的锁 | 单线程 reactor，回调自然串行 | `GCodeRequestQueue` 的 sink 回调在**锁外**执行（gcode/flush 双线程） | [模块表](README.md) |
| `start_reason` | 主循环每轮写回 `start_args` | 只在一处表示差异 | [klippy-runtime](klippy-runtime.md) |
| `webhooks` 实现形态 | 一个文件同时是传输/派发机器与一个 printer object | 拆成 `klippy-api`（传输/派发）与 `core/klippy/api`（端点） | [klippy-runtime](klippy-runtime.md)、[运行时编排](runtime.md) |
| `[verify_heater <不存在的 heater>]` | 报 `Unknown heater` | 报装载器的 `Section '…' is not a valid config section` | [配置手册](../user-manual/config.md) |
| `[virtual_sdcard]` 细节 | `path` 做 `expanduser`/`normpath`；回放让出用 `gcode.get_mutex().test()`；有 `stats` | `path` 原样用于目录列举；让出用 `tokio::task::yield_now()`；无 `stats` | [模块表](README.md)、[G-Code 命令参考](../user-manual/gcode-commands.md) |
| MCU 连接 `restart_method` | — | 只对串口 MCU 有意义；CAN 与 `host_library` 一律按 `command` 处理 | [MCU 连接方式](../user-manual/mcu-connection.md) |
| API 若干已知差异 | 上游 `machine`/`calibration`/`get_status` 等更完整 | 缺 `probe_path`/`rapid_path`、部分 `get_status` 字段未返回等 | [API 参考](../third-party-dev/api-reference.md) |

## 3. 暂缓/未实现（本仓没做，方向上不排除）

| 项 | 现状 | 详情 |
|---|---|---|
| 上游 host `Stats …` 日志行 / `gcodein` | 无 host 统计采集器 | 本条 1.1 |
| `gcode:debuginput_exit` | 事件已声明，无发送方（只有行协议输入会发） | [事件系统](event-system.md#12-事件清单) |
| `query_adc` / `QUERY_ADC` | 未实现（`adc_scaled` 只用 config 输入量程） | [配置手册](../user-manual/config.md)、[模块表](README.md) |
| `steppers` 的资格过滤 | 未实现（本仓 homing 事件无载荷） | [配置手册](../user-manual/config.md)、[模块表](README.md) |
| TMC 部分型号与 SPI 传输 | `tmc2130`/`tmc2660`/`tmc5160`/`tmc2240` 等未实现 | [配置手册](../user-manual/config.md) |
| `TEMPLATE=` 形式的菜单/显示选项 | 未实现（菜单不渲染） | [配置手册](../user-manual/config.md) |
| MCU `output` 表的异步投递等 | 见该页「当前未实现」 | [MCU 协议](mcu-protocol.md#当前未实现) |
| 事件名无发送方的其余项 | `toolhead:sync_print_time`、`stepper:*`、`extruder:activate_extruder`、`menu:*`、`dual_carriage:update_kinematics` | [事件系统](event-system.md#22-覆盖范围) |

## 4. 计数与清单（改了要连带重核的地方）

| 数字 | 位置 |
|---|---|
| 上游语料声明运行数 / 通过数 / 跳过数 | [`README.md` → 当前状态](../../../README.md#当前状态) |
| 端点总数（16 条注册路径 + 内建 `list_endpoints`） | [模块表](README.md) |
| 事件总数与"已发出/无发送方" | [事件系统](event-system.md) |
| `[extras]` 模块数与 `section!` 数 | [`README.md` → 当前状态](../../../README.md#当前状态) |
| 二进制体积 | [模块表](README.md) |

## 5. 上游行号引用的核验

本仓文档与源码注释里大量引上游 `xxx.py:NN-NN`。改动上游相关行为后这些行号会漂移，
核验方式是 `python3 scripts/pyref-audit.py --docs`（以及不加 `--docs` 扫源码），细则与
噪声说明见[测试 → 文档同步](testing.md#文档同步)。

---

- [← 开发手册首页](README.md)
