# 框架队列收尾记录（FW1/C2、FW3/A2、FW4、FW7、FW8、FW9）

日期：2026-09-22。对应 `TODO.md` 的「框架优先」表：把 FW1–FW9 的**遗留框架点**按顺序
做完（标 `[~]` 的 `GCodeIO`、`MCU_bus_digital_out` 不动）。软件判据：`cargo test
--workspace` 全绿（lib 838），真板项见 [`TESTING.md`](../../TESTING.md)。

上游参考：`third_party/klipper/`（commit `02e71b9`）。

---

## FW1 / C2 — 配置 getter 与文案

上游 `configfile.py` 的范围/取值检查是 getter 的一部分；之前散在各 extras 里手写。

- `config/wrapper.rs` 新增：
  - `get_float_bounded(option, default, minval, maxval, above, below)` → `"must have
    minimum of N"` / `"must be above N"` / `"must be below N"`
    （`configfile.py:getfloat`）；
  - `get_int_bounded(option, default, minval, maxval)`；
  - `get_choice(option, choices, default)` → `"Choice 'x' for option 'y' in section 'z'
    is not a valid choice"`（`configfile.py:getchoice`）。
- 改用方：`extras/stepper.rs`（`rotation_distance`/`microsteps`/`step_pulse_duration`/
  `position_max`/`homing_speed`）、`extras/toolhead.rs`（`max_velocity`/`max_accel`/
  `min_cruise_ratio`/`square_corner_velocity`/`max_z_velocity`/`max_z_accel`，并按上游
  `cartesian.py` 把 Z 默认值改为 `max_velocity`/`max_accel`）、`extras/output_pin.rs`
  （`value`/`shutdown_value`/`cycle_time`）。
- `autosave` / `SAVE_CONFIG` / `deprecate` 归**模块**（不是框架），未做。

## FW3 / A2 — 错误词汇与 connect 失败分类

- `Printer::require_object` / `require_object_as::<T>`：`"Unknown config object 'x'"`
  （上游 `lookup_object` 的 `error`）。
- `bring_up` 的所有 connect 失败改走 `set_error_state`（`PrinterState::Error`，可
  `RESTART`），不再 `invoke_shutdown`；MCU 对象先 `set_error_state(短消息)` 再发
  `klippy:notify_mcu_error`，与上游 `_connect` 的 except 分支一致
  （`klippy/klippy.py:136-158`）。
- `classify_mcu_error`（自由函数）把失败分成 `Protocol error` / `MCU error during
  connect`，字段含义与上游一致。
- `mcu/object.rs`：设备 `open` 失败 → `KlippyError::Connection`，`builder.build()` 的
  `McuError::Config` → `KlippyError::Config`。

## FW4 — `gcode:request_restart`

上游 `GCodeDispatch.request_restart`（`klippy/gcode.py:352-362`）在 printer ready 时取
`toolhead.get_last_move_time()`、发 `gcode:request_restart`、`dwell(0.500)`、
`wait_moves()`，再 `request_exit`。

- `printer.rs` 新增 `RestartHooks` trait（`get_last_move_time`/`dwell`/`wait_moves`）
  与 `register_restart_hooks`/`restart_hooks`，`reset_for_restart` 清空。
- `extras/toolhead.rs` 在 `connect` 注册 `ToolHeadRestartHooks`（持有
  `Arc<Mutex<Option<Connected>>>`，不依赖 `Arc<Self>`）。
- `gcode.rs` 的 `Inner::request_restart`；`RESTART`/`FIRMWARE_RESTART` 改走它。

## FW7 — `error_mcu`

上游 `klippy/extras/error_mcu.py`：监听 `klippy:analyze_shutdown` 与
`klippy:notify_mcu_error`，把简短状态消息换成带 MCU 名/原因/提示/后续操作的文本。

- `printer.rs`：`invoke_shutdown_with(msg, details)`（MCU 停止发短消息 `"MCU shutdown"` +
  `{mcu, reason, event_type}`）；`update_error_msg(old, new)`（只在状态消息仍等于 `old`
  且非 ready/startup 时替换，语义同上游）；`software_version()`（见 FW8）。
- `mcu/object.rs`：`report_mcu_shutdown` 带 details；`Starting` 对齐上游
  `"MCU 'x' spontaneous restart"`，且仅在未 stop 时报。
- 新增 `extras/error_mcu.rs`（随首个 `[mcu]` 装载，上游 `mcu.py:1159`）：`error_hint`
  表、shutdown 展开（`prefix+reason+clarify+hint+message_shutdown`）、protocol 错误逐 MCU
  比对 `mcu_version` 与 `software_version`、connect 错误提示；`add_clarify` 预留给
  `adc_temperature`。对象无状态、`is_queryable=false`，不进 `objects/list`。

## FW8 — `StartArgs` 其余字段 + `M115`

- `api/start_args.rs`：补 `apiserver`/`start_reason`/`debug_input`/`debug_output`/
  `device`/`linux_version`；`device_info()`/`linux_version()` 按上游 `util.py`
  （`get_device_info`/`get_linux_version`）读 `/proc`。
- `printer.rs`：保存 host 的 `StartArgs`（上游 `printer.start_args`），
  `set_start_args`/`start_args`，`software_version` 由其中同步。
- `gcode.rs`：`M115` 读 `printer.software_version()`（未设置回退 crate 版本）。
- `klippy.rs`：host 先建 `StartArgs`、填 `apiserver`、`set_start_args`，再交给
  `api::register`，保证 printer 与 `info` 端点共用一份。

## FW9 — API 收尾

- 新增 `api/endpoints/register_remote_method.rs`：客户端给自己连接注册
  `remote_method` + 可选 `response_template`（`ResponseTemplate::from_params`），返回 `{}`。
- `api/webhooks.rs`：`webhooks` printer object 升级为**服务器入口**——
  - `install()` 幂等注册（上游 `add_early_printer_objects`）；
  - `register_mux_endpoint(path, key, value, handler)`：同一 path 只能一个 key、同实例不可
    重复，文案同上游；`take_mux_endpoints()` 交给 `api::register` 倒入 `Api::register_mux`；
  - `set_api(Arc<Api>)` / `call_remote_method(method, params)`：转发给
    `Api::call_remote_method`（上游 `WebHooks.call_remote_method`）。

### 关于「为什么 `core/klippy/api/` 下还有一个 `webhooks`」

上游 `webhooks.py` 一个文件同时是**传输/派发机器**和**一个 printer object**。按 crate
边界拆开：

| 上游 | 这里 |
|---|---|
| `ServerSocket`/`ClientConnection`/分帧/`_process_request` | `crates/klippy-api`（`server.rs`/`protocol.rs`） |
| `WebHooks._endpoints`/`_handle_mux`/`_remote_methods` | `crates/klippy-api/src/registry.rs`（`Api`） |
| `add_object('webhooks', WebHooks(printer))`、`get_status` | `core/klippy/api/webhooks.rs`（唯一 `webhooks` 对象） |
| `_handle_*_request` | `core/klippy/api/endpoints/*.rs` |
| `WebHooks.__init__` 的注册顺序 | `core/klippy/api/mod.rs::register` + `endpoints/mod.rs` 的 `endpoint!` 表 |

`webhooks.rs` 只是 printer object 那一面的**薄适配**：extras 按上游方式
`lookup_object('webhooks').register_mux_endpoint(...)` / `.call_remote_method(...)`，而表在
`Api` 里、extras 看不到，于是由它转发。

### 顺序调整

`klippy.rs` 改为：**先在 `load_config` 之前装 `webhooks` 对象**（extras 装载时能按名拿到
服务器并注册 mux）→ 读配置 → 建 `Api` 表（倒入 mux）→ 绑监听 → `webhooks.set_api`。

---

## 测试与状态

- `cargo test --workspace`：lib 838 全绿；clippy 只剩两条既有告警
  （`pins.rs setup_adc_sample` 参数过多、`adc.rs` 类型复杂度）。
- 提交：`aad5ed1`（FW1/C2+FW3/A2+FW4）、`a5e1779`（FW7）、`0818412`（FW8）、
  `e345470`（FW9）。
- 仍标 `[~]`：`GCodeIO`（不做 OctoPrint 串口仿真）、`MCU_bus_digital_out` 包装（随 H8）。
- 真板验证：`TESTING.md` T1–T7（三轴 `G1`/`G28`、`M119`、双板时基/漂移、`rpi_usb`、
  外设、extruder/stepper_enable/回零精度）。
