# TODO — 主机（klippy）侧的待办与未决问题

本文件只记**打算做什么**和**还没定的事**，不重复已经定下的设计（那些写在各模块文档和
`docs/klippy/developer-manual/` 里）。括号里的 `klippy/xxx.py:NN` 指上游参考实现
（`third_party/klipper/`），用来在动手前核对行为。

## 已定

- **Printer 是一台机器，且只有单实现**：`src/core/klippy/printer.rs` 里就是一个 `Printer`
  结构体（无 trait、无工厂），一个主机进程只跑一个；机器拥有它的组成部分（config
  section 对应的 printer object）与生命周期；进程级的东西（命令行、日志、API server、
runtime、重启循环）是它上面的一层。
- **`Kinematics` trait 与 `kinematics/` 已删**（连同 `load_kinematics` 工厂）：kinematics
  不是 Printer 的分类依据，它是 toolhead 持有的一个对象；**等实现 toolhead 时再加回**。
  上游证据：`klippy/toolhead.py:242` 是唯一装载点，`Printer` 类不知道 kinematics。
- **MCU 一侧的分层已定**：`msg → mcu → cmd`，`event` / `identify` 平级
  （`docs/klippy/developer-manual/architecture.md`、`README.md` 的分层表）。
- **客户端 API 的线形状**以 `docs/klippy/third-party-dev/api-reference.md` 为准。

## 待办

### T1 printer object：先做「状态表」，不做「服务定位器」

- [ ] `name -> Arc<dyn StatusSource>`（`get_status(eventtime) -> Value`）：`objects/list`
      只列有状态的、`objects/query` 直接用它（`klippy/webhooks.py:484`、`:510`）。
- [ ] 第一个住户：`[mcu]` —— `src/core/klippy/config/mcu.rs` 已经能解析这个 section，
      上游 `klippy/mcu.py:1234` 的 `get_status` 是现成的返回形状。
- [ ] 先不做 downcast、不做工厂表：等第一个「模块之间按名字互相找」的需求出现再说
      （上游 `lookup_object` 有 424 处，但我们目前还没有第二个对象）。
- [ ] `objects/subscribe`：0.25s 轮询 + `response_template`（`klippy/webhooks.py:490-560`），
      推送走已有的 `PushTarget`。

### T2 主机层（机器外面的东西）

- [ ] `src/klippy.rs::klippy_process` 现在只有一个 `ctrl_c`：改成「建机器 → 起 runtime →
      跑机器 → 按结果重启或退出」的循环（上游 `klippy/klippy.py:355` 的 `while 1`）。
- [ ] `api → printer` 这条边：`EndpointContext` 目前只有 `api` + `client`，`info` 的
      handler 还是 `todo!()`（`src/core/klippy/api/endpoints/info.rs`）；`get_state_message()`
      + `PrinterState::as_category()` 已经就绪，接上即可。
- [ ] start args / rollover info / 日志（`get_start_args` 29 处、`set_rollover_info` 7 处）
      归主机层，不进机器。

### T3 reactor 抽象与 `run()` 的形态

- [ ] 机器需要一个定时器/延后回调接口（上游 `get_reactor()` 110 处），但 `Printer`
      与单测不该被拖着起 tokio。先定一个最小 trait（`call_later`、`register_callback`）。
- [ ] `Printer::run()` 目前是同步阻塞（`src/core/klippy/printer.rs` 用 `Condvar` 等
      `request_exit`）；接上 runtime 后决定：async run / 内部 `block_on` / 主机只 spawn。

### T4 toolhead 与 kinematics（kinematics 已删，从这里重新开始）

- [ ] 先立 **toolhead 对象**：位置记忆（`commanded_pos`）、trapq、速度/加速度上限，
      回零与移动的入口（上游 `klippy/toolhead.py:389` `:400` `:482` `:507` `:522`）。
- [ ] 再加回 **`Kinematics` trait 与 `kinematics/`**：按上游由 toolhead 读
      `[printer] kinematics` 装载（`klippy/toolhead.py:242`），不是交给 Printer。
- [ ] `calc_position` 的返回类型：上游允许逐轴为 `None`
      （`extras/homing.py:245` 判空，`mathutil.py:152` 的 `gaussian_solve` 会返回
      `None`），`Coord { x: f64, .. }`（已随 kinematics 一起删除）需要重新定形状。
- [ ] 回零协议：上游 kinematics 调 `homing_state.home_rails(rails, forcepos, movepos)`、
      `set_homed_position(pos)`、`get_trigger_position`、`set_stepper_adjustment`。
      没有这些，任何真实 kinematics 的 `home()` 都写不出来。
- [ ] stepper 句柄：上游能 `get_commanded_position()` / `get_step_dist()` / `set_trapq()` /
      `setup_itersolve()`；`calc_position` 的输入就从这里来。
- [ ] step 生成层的运动学（上游 `rail.setup_itersolve('cartesian_stepper_alloc', axis)`、
      `kinematics/kinematic_stepper.py`）在我们这儿还没有对应物，运动规划整个未开始。

### T5 配置驱动装载（推迟到有第二个对象）

- [ ] 静态工厂表：`section 名 -> fn(&Config, &ConfigSection) -> Result<Arc<dyn PrinterObject>>`，
      `load_config` / `load_config_prefix` 两条（`klippy/klippy.py:90`、
      `klippy/extras/temperature_sensor.py:41`）。
- [ ] 两段式构造：`new` 只登记自己，接线放 connect 阶段（上游靠 `add_printer_objects`
      分批 + `klippy:connect` 延后），否则互相 lookup 的模块会死锁或拿到半成品。
- [ ] section 合法性校验：上游 `klippy/configfile.py:429` 直接拿注册表当 schema
      （`Section 'x' is not a valid config section`）。

### T6 错误词汇

- [ ] `CommandError` / `ConfigError`（上游 extras 里 94 / 41 处）；`KlippyError` 现在只有
      通信类 5 个变体（`src/core/klippy/error.rs`）。

### T7 文档

- [ ] `docs/klippy/developer-manual/`：补 printer 一节，并在 README 的分层表里加
      `printer` 一行（现在只有 msg / mcu / cmd / event / identify / api）。
## 未决问题

- [ ] **Q2 事件系统的形状**：封闭枚举（现状 `PrinterEvent`）还是开放总线（上游 30+ 个
      自定义事件名 + 各事件自定参数，`klippy/klippy.py:224-227`）。枚举表达不了
      `idle_timeout:ready` 这类名字。
- [ ] **Q3 `PrinterEvent` 是否恢复 `McuIdentify` / `AnalyzeShutdown` / `NotifyMcuError`**
      （后两个的 handler 需要 msg + details）。
- [ ] **Q4 `get_status` 的返回形状**：`serde_json::Value`（贴上游、客户端零适配）还是
      typed + serde。
- [ ] **Q5 要不要反射式能力**：`lookup_objects(module)` 前缀遍历、`gcode_macro` 的
      `printer.objects`（`klippy/extras/gcode_macro.py:41`）。
- [ ] **Q6 退出结果的语义**：`"exit" / "error_exit" / "firmware_restart"` 由谁解释、
      `run()` 的返回值怎么变成进程退出码（`klippy/klippy.py:355-370`）。

## 证据索引（上游，供回头分析时查）

| 主题 | 位置 |
|---|---|
| Printer 生命周期、状态、事件 | `klippy/klippy.py:25-236` |
| 事件总线（开放名字 + 参数） | `klippy/klippy.py:224-227` |
| 对象注册表（add / lookup / load） | `klippy/klippy.py:70-113` |
| `objects/list`、`query`、`subscribe` | `klippy/webhooks.py:480-560` |
| section 校验用注册表 | `klippy/configfile.py:425-445` |
| kinematics 的装载与接缝 | `klippy/toolhead.py:235-252`、`:389` `:400` `:482` `:507` `:522` |
| 各 kinematics 的差异 | `klippy/kinematics/*.py`（`home` / `check_move` / `calc_position` / `get_status`） |
| 通用回零驱动 | `klippy/extras/homing.py:165-300` |
| IDEX / 双滑车 | `klippy/kinematics/idex_modes.py`、`klippy/kinematics/cartesian.py:19-30` |
| step 生成层的运动学 | `klippy/kinematics/kinematic_stepper.py`、`rail.setup_itersolve(...)` |
| 惰性装载 | `klippy/extras/adc_temperature.py:51` `load_object(config, 'query_adc')` |
