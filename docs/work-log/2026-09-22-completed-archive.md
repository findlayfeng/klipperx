# 已完成条目归档（大项，截至 2026-09-24）

> 按 `TODO.md`「更新规则」第 3 条：这里**只保留大项**（子系统/框架队列/里程碑级），
> 细项（收尾、单命令/单端点/单验证、小修）随「完成即删除」直接消失，需要时查 git 历史。

本文件是 `TODO.md` 原有「已完成（留档）」一节的全文，为了把 TODO 收窄到**只剩未决项**而
单独归档。它不是规范，也不必随实现更新：细节在各模块自己的文档与
`docs/klippy/developer-manual/`，这里每条只留一行索引，最近完成的在前。


- **2026-09-23 完成情况审计移入（逐条对源码复核，证据见括注）**：
  - **D1 剩余接线与 `M115` 版本号**（原 D1/G1b 条目）：`StartArgs` 结构体已带
        `apiserver`/`start_reason`/`debug_input`/`debug_output`/`device`/`linux_version`，
        宿主填 apiserver 并 `set_start_args` 注入（`src/klippy.rs:323-324`）；`M115` 改读
        `printer.software_version()`（`gcode.rs:747-767`，单测
        `test_m115_reports_the_host_software_version`）。剩余（CLI `debuginput`/
        `debugoutput` 解析、每 MCU 字典路径）留在正文 D1。
  - **G1b 行为差异三项**：`request_restart` 停机前动作（`get_last_move_time` +
        `gcode:request_restart` 事件 + `dwell(0.500)` + `wait_moves()`，`gcode.rs:1186-1193`，
        同上游 `gcode.py:352-365`）；`Coord`（`mathutil.rs:36`）；参数访问器（FW4 已记，
        正文索引同步去重）。
  - **H1 大部**：`heaters` 控制环 bang-bang/PID 与 `SET_HEATER_TEMPERATURE`
        （`heaters.rs`，含单测；`:499-516`）；`heater_bed`/`heater_generic` 住户与
        `M140`/`M190`（`heater_bed.rs:24` `:52`；M190/M109 等待循环仍缺）；温度传感器族
        6 件（`temperature_sensor`/`thermistor`/`adc_temperature`/`spi_temperature`/
        `temperature_combined`/`temperature_mcu`，与 T7 重合）。
  - **H2/H10/H11/H12 零散**：`static_digital_output`（T9 阶段 0，正文已去重）；
        `stepper_enable`（T2）；`motion_queuing`（`motion/queuing.rs` 对照上游
        `motion_queuing.py:63-67`）；`error_mcu`（FW3/FW7，`extras/error_mcu.rs`）；
        前缀式 `lookup_objects`/`statuses`/`status_of`（FW2，`printer.rs:622` `:654` `:676`）。
  - **事件触发点落地**（正文事件清单同步改「实现依赖」列）：`homing:*` 四个
        （`extras/toolhead.rs:910` `:923` `:974` `:1008`）、`toolhead:set_position`
        （`:1091`）、`gcode:request_restart`（`gcode.rs:1191`）、`stepper_enable:motor_off`
        （`stepper_enable.rs:338`）。

- **上游 `.test` 语料框架与 T1（首个用例转绿）**：`src/core/klippy/upstream.rs` 的 harness 与
      runner（按 `CONFIG` 拆运行、字典齐备才启用、`IGNORED` 留档必然失败的用例）；
      `interface/devices/simulator.rs` 的字典驱动应答机（identify / 配置握手 / 时钟 / ack）；
      `crates/test-support/build.rs` 按 `KLIPPERX_ARCHES` / `KLIPPERX_ALL_ARCHES` 编字典；
      并顺带对齐配置解析（多行值 / `=` / 节头与 `;` 注释）、实现 `deprecate`；
      `linuxtest.test` 端到端通过（`kinematics: none` + `heaters` + `temperature_sensor` +
      `ds18b20`）。后续推进见正文 **T**。

- **框架队列 FW1–FW9（全部完成）**：FW1 配置装载、FW2 对象模型、FW3 错误词汇、FW4 G-Code、
      FW5 运动（a–f）、FW6 资源与触发、FW7 MCU 与传输、FW8 主机层与重启、FW9 API；验收以
      「最小模块在 host 单测 + 假 MCU 上跑通」为准，真板项见 [`TESTING.md`](../../TESTING.md)。
      两个子项 `[~]` 暂缓：`GCodeIO`（不做 OctoPrint 串口仿真，见正文 G1b）、
      `MCU_bus_digital_out` 包装（能力已由 `DigitalOut::queue_digital_out` 提供，随 H8 显示接）。
      分阶段细节见各 FW 的工作记录（`docs/work-log/2026-09-21-fw*-notes.md` 等）；
      已完结的几篇随收口清理（见目录 README 说明）。

- **toolhead 与 kinematics（C1 / FW5a–f）**：Rust 分层重写（不引 FFI）——`Coord` 与
      `clocksync` 回归、`Move`/`LookAheadQueue`/`trapq`、`itersolve`+`kin_cartesian`、
      `MotionQueuing`/`ToolHead`/`McuStepper`、`Kinematics`+`cartesian`+`[stepper_*]`/`[printer]`+`G1`、
      `stepcompress` 完整压缩；`kinematics: none` 随 T1 补上。设计取舍与逐阶段验收见
      FW5 动工前调查（已完结清理）；回零协议的 `get_trigger_position` /
      `set_stepper_adjustment` 随 `endstop_phase` 后置（H9）。

- **重启循环与 `restart_method` 分派（D2 大部）**：`klippy_process` 就地重建
      （`Printer::reset_for_restart` + `load_config` + `bring_up`，同一个 `Arc<Printer>`），
      以及 `command` / `arduino` / `cheetah` / `rpi_usb` 四种物理复位与连接期门控
      （`src/klippy.rs`、`mcu/object.rs`、`mcu/restart.rs`、`interface/usb.rs`）。

- **GCODE 调度器（G1）**：命令表 / `register_mux_command` / `run_script` / 输出处理器 /
      内置命令，`load_config` 里最先注册（`gcode.rs`）；剩余行为差异见 **G1b**。

- **pin 解析与 `pins`（F2）**：`PrinterPins` / `PinResolver` 的别名与保留，
      `RESERVE_PINS_*` 在 connect 预留；`pins` 注册但不可查询（`pins.rs`、`printer.rs`、
      `mcu/object.rs`）。

- **MCU 配置构建层（F1）**：oid 发号、`config` / `restart` / `init` 三张命令表、config 回调、
      CRC + `finalize_config`，`configure()` 的 `get_config` 两段式下发（`mcu/config.rs`）。

- **机器与 API 分 runtime（A3）**：机器跑在专用多线程 runtime（`worker_threads(2)`，worker 名
      `klippy-mcu`）上，由一条专用 OS 线程（`klippy-machine`）驱动 `klippy_process`；API 保留
      进程本来的多线程 runtime（`klippy-api`）跑 accept / 每连接 / attachment / `ctrl_c`。reactor
      显式建在机器 handle 上；`load_config` 跑在 `machine_handle.enter()` 下，所以
      `Interface::with_transport` 捕获到的是机器 handle。跨 runtime 只靠 `Arc<Printer>` 与
      `request_exit` 的 `Condvar`。停机顺序：`request_exit` → `run()` 返回 → `teardown`（机器
      runtime 尚在）→ 机器线程 drop runtime → join → abort 监听与 server（`src/klippy.rs`；
      `docs/klippy/developer-manual/runtime.md`）。

- **主机层串起来**：`klippy_process`（建机器 → `api::register` → bind → `load_config`，
      失败即 `invoke_shutdown` → `bring_up` → `run`）、`info` 端点、`StartArgs`
      （`src/klippy.rs`、`api/endpoints/info.rs`、`api/start_args.rs`）。

- **配置驱动装载**：静态工厂表、两段式构造顺序、section 级合法性校验（`load.rs`）。
