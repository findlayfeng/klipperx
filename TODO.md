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

- [x] `name -> Arc<dyn StatusSource>`（`get_status(eventtime) -> Value`）在 `Printer` 上：
      `add_status_object` / `status_objects` / `status_of`（`src/core/klippy/printer.rs`）。
- [x] `objects/list` 与 `objects/query`（`src/core/klippy/api/endpoints/objects_{list,query}.rs`）：
      上游的三个可见行为都对齐 —— 未知对象回 `{}` 而不是报错、不存在的字段回 `null`、
      `null` 字段列表取全部；`objects` 参数的三类错误文本也一致（`Invalid argument`）。
- [x] 先不做 downcast、不做工厂表：等第一个「模块之间按名字互相找」的需求出现再说
      （上游 `lookup_object` 有 424 处，但我们目前还没有第二个对象）。
- [x] 第一个住户是 `webhooks`，装在**服务器这一侧**（`src/core/klippy/api/webhooks.rs`）：
      对象名与字段（`state` / `state_message`）对齐上游 `webhooks.get_status`，但它属于
      API 服务器而不是机器 —— 上游也是 `webhooks` 模块把自己登记成打印机对象的
      （`klippy/webhooks.py:564`，而它读的是 `printer.get_state_message()`）。
- [x] `api::register(api, printer)`（`src/core/klippy/api/mod.rs`）一次装完服务器这一侧：
      先登 `webhooks`、再登端点；之后 `Printer::new()` 的状态表是空的（机器没有组成部分），
      客户端看到的 `objects/list` 从一开始就含 `webhooks`。
- [ ] **端点与对象尚未接入主机**：需要在建 `Printer` 之后、**bind 之前**调用一次
      `api::register`（上游的时序：`Printer.__init__` 先登 `webhooks`（内部就 bind 了
      socket），配置里的一切 `configfile` / `mcu*` / `toolhead` 都是之后才登的 ——
      所以「连上了就一定有 `webhooks`」这条保证靠的是顺序，不是占位对象）。
      注册先于 bind 这条只能靠代码结构保证，单测只能锁住注册函数的效果。
- [ ] `[mcu]` 作为住户：**卡在 T2**。上游 `mcu.get_status` 报的是 identify 的
      `mcu_version` / `mcu_build_versions` / `mcu_constants` 加 `last_stats`
      （`klippy/mcu.py:922-975`），而这些要 MCU 连上（有字典）之后才有；
      `config/mcu.rs` 只给传输配置，不是上游那个 status。等 T2 连 MCU 时一并加。
- [ ] `objects/subscribe`：0.25s 轮询 + `response_template`（`klippy/webhooks.py:490-560`），
      推送走已有的 `PushTarget`。它需要一个定时器（T3）与「只推变化」的比对（上游
      用全局 `last_query` 与每个订阅自己的字段表），所以跟在 T3 后面做。

### T2 主机层（机器外面的东西）

- [ ] `src/klippy.rs::klippy_process` 现在只有一个 `ctrl_c`：改成「建机器 → 起 runtime →
      跑机器 → 按结果重启或退出」的循环（上游 `klippy/klippy.py:355` 的 `while 1`）。
- [ ] `api → printer` 这条边：`objects/*` 与 `webhooks` 已经写好在 `api::register` 一处，
      主机建出 `Printer` 后在 **bind 之前**调它即可（顺序要求见 T1 最后一条未完成项）；
      `info` 的 handler 还是 `todo!()`（`src/core/klippy/api/endpoints/info.rs`），
      `get_state_message()` + `PrinterState::as_category()` 已经就绪，接上即可。
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

### T8 `python_path` 的取消（**远期，依赖外部项目**）

- [ ] **现状**：`info` 里这个字段只被 Moonraker 使用，且它把它当 Klipper 的
      virtualenv 解释器（`update_manager/app_deploy.py` 的 `_configure_virtualenv`），
      因此我们的主机目前只能报一个**不存在的路径**（让它的 Klipper 更新项退化为
      no-op），不能报自己的二进制（会让 Moonraker 报 `Invalid virtualenv` 而起不来）。
      字段本身还必须存在：它直接下标取 `self._klippy_info["python_path"]`
      （`klippy_connection.py` 的 `_save_path_info`）。
- [ ] **想做的**：这是一个 Klipper 实现细节，本主机没有解释器也没有 Klipper 源码树，
      报一个不存在（或任何）路径都是在编数据；理想是**取消**这个字段。
- [ ] **阻塞在外部**：取消会让 Moonraker 的 `_save_path_info` 抛 `KeyError`（不在它的
      `except ServerError` 里），连接任务出错、它反复重连。上游把下标取值改成
      `.get()`（或用 `client_info` 判类型）之后才能自由。所以这条要等与 Moonraker 的
      沟通/上游改动，排在很后面。
- [ ] **过渡期的可选做法：按请求认出 Moonraker，只对它发这个字段**（其余客户端不发）。
      可行手段：① `client_info.program == "Moonraker"`
      （`moonraker/components/klippy_apis.py`：首次 info 带 `{'client_info': {'program': "Moonraker", 'version': …}}`）
      —— 自报、可缺、且**只有识别那次请求带**，因此要按连接记住；
      ② `SO_PEERCRED` 取对端 pid，再看 `/proc/<pid>/comm`（Moonraker 现在是 Python 进程，
      comm 未必是 moonraker）—— 更硬但要先把对端身份从 server 层传到
      `EndpointContext`（现在没有）。上游只把 `client_info` 当日志用，故这属于本项目的
      自定义兼容层，要有到期日。

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
