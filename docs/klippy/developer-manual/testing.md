# 测试

> **真板/外设验证不在主线任务里**：需要真实 MCU 或外设的验证列在仓库根的
> [`TESTING.md`](../../../TESTING.md)，不阻塞开发；主线以本文件的 host 单测 + 假 MCU 验收。

测试与被测代码同文件，位于各模块的 `#[cfg(test)] mod tests`，不需要外部进程或真实串口。底层 IO 由 `interface::devices::frame_mock::FrameMock` 模拟：它按 FIFO 逐条比对收到的帧，并把预设的输出帧排队给 `receive()`。

### klipper 检出的位置（`KLIPPERX_KLIPPER_DIR`）

语料、`.test` 用例与 kconfig 片段都读自一个 klipper 检出，默认是仓库内的子模组
`third_party/klipper`。`KLIPPERX_KLIPPER_DIR` 可以把它指到别处——**git worktree 用这个变量
指向主检出的 `third_party/klipper`，就不必拷贝子模组**：

```bash
KLIPPERX_KLIPPER_DIR=<主检出>/third_party/klipper cargo test --workspace
```

为什么可以共享一份检出：`crates/test-support/build.rs` 调 `make` 时把 `KCONFIG_CONFIG` 与 `OUT`
都指到自己的 `OUT_DIR`（`target/.../klipper-targets/`），只读子模组的源码与 Makefile，不写它自己的
`.config`/`out/`，因此多个 worktree 并发构建也不会互相污染。拷贝子模组的做法反而有害：子模组内的
`.git` 是相对路径，拷到 worktree 后会解析失败，使该 worktree 里任何 `git status` 都报错。

变量由 `crates/test-support`（`build.rs` 与 `klipper_dir()`）与 `src/core/klippy/upstream.rs` 统一解析
（后者直接用前者的函数），构建期与运行期看到的是同一个路径；它也进了 `build.rs` 的
`cargo:rerun-if-env-changed`，改指向会触发重建。

另有一个**真实设备**测试：`interface::host::HostDevice` 通过 `dlopen` 加载 klipper 的 host 库（`third_party/klipper/src/host/`），并在其中跑一个 `get_clock` 往返。该库由 dev-dependency `klipperx-test-support` 的 `build.rs` 调 `make` 构建，所以需要 `make` 与 C 工具链；库的全局状态决定了一个进程同一时刻只能有一个 `HostDevice`。

构建**不复用 klipper 子模块的 `.config` 与 `out/`**（那是开发者给自己编固件用的），而是把它们放在 build script 自己的 `OUT_DIR` 下（`target/debug/build/klipperx-test-support-*/out/klipper-host/`），并把路径通过 `cargo:rustc-env=KLIPPER_HOST_LIB` 传给测试。配置片段在 `build.rs` 里写明——kconfig 的机型 `choice` **没有默认值**（空配置会选中第一项 AVR，编出个 `klipper.elf`），而 `CONFIG_HOST_AR_LIBRARY` **默认是静态库**（产出 `libklipper_host.a` 而非 `.so`），两条都必须显式声明：

```
CONFIG_MACH_HOST=y
# CONFIG_HOST_AR_LIBRARY is not set
```

因此：全新检出即可复现（`make olddefconfig` 补齐其余默认值，`build.rs` 在产物缺失时报错并指出要检查哪两条），开发者正在编的固件配置与 `out/` 不受影响，测试也不会用到别人的配置。

## 运行

```bash
cargo test --workspace                 # 全部单元测试（三个包）
cargo test -p klipperx --lib mcu       # mcu 层
cargo test -p klipperx --lib cmd::tests # 单个命令模块
cargo test -p klippy-api               # API 层（会真开 socket 的部分在 server.rs）
cargo test -p klippy-client            # 客户端（同样会真开 socket）
cargo test -p klipperx --lib test_install_skips  # 单个用例（按名过滤）
```

> **注意 `--workspace`**：这是 workspace，而 `cargo test` 在非虚拟 workspace 里只跑**根包**（也就是主机）。`--lib` 后面那些过滤词同理只作用于被选中的包。想覆盖 klippy-api / klippy-client，要么 `--workspace`，要么 `-p <包名>`。

## 真机测试

少数用例需要真实硬件，目前只有 `mcu/mod.rs` 中的 `test_frame_sequence_sync_against_a_real_board`。此类用例遵循两条约定：

1. **默认不执行。** 用例以 `#[ignore]` 标注；设备地址由环境变量给出（该用例为 `KLIPPERX_HW_SERIAL`），不写入仓库中的任何配置文件，也不假定某台机器的固定设备路径。因此 `cargo test` 与 `cargo test --workspace` 在没有硬件的机器上必须全部通过，且不得打开串口或 USB 设备。
2. **被显式请求时不得静默通过。** 以 `--ignored` 单独请求真机用例而未提供环境变量时，用例必须失败，并在消息中指出缺少的变量。Rust 测试框架没有在运行期将用例标记为 ignored 的接口，测试体开头的提前返回会被记为通过，因此这种情况只能按失败处理。

运行方式：

```bash
KLIPPERX_HW_SERIAL=/dev/ttyACM1 \
  cargo test -p klipperx --lib test_frame_sequence_sync_against_a_real_board \
  -- --ignored --nocapture
```

以 `--ignored` 运行时若未设置 `KLIPPERX_HW_SERIAL`，用例失败并打印所需变量；普通的 `cargo test` 不执行该用例。

## 格式化与提交

提交前代码要过 `cargo fmt --all`。仓库自带的 pre-commit 钩子会替你做这件事：它先格式化，再把已暂存的 `.rs` 重新入索引，最后用 `cargo fmt --all -- --check` 兜底；实在格式不了（语法错误之类）就中止提交。

钩子放在版本库里的 `.githooks/`，新检出后启用一次即可（这是每人的本地配置，不进仓库）：

```bash
cargo fmt --all                 # 手动跑一次

git config core.hooksPath .githooks
```

> 钩子按**整个文件**重新入索引，所以一个既暂存了部分改动、又有未暂存改动的 `.rs` 文件会把未暂存的那半也一并带上；在意的话先格式化再暂存。

## 覆盖范围

### `msg`

| 模块 | 覆盖 |
|------|------|
| `proto.rs` | 各类型 push/pop 往返、VLQ 有符号编码与 Klipper 对齐、边界长度、`try_convert_to` 范围检查、`try_merge` 超限不动原值 |
| `mod.rs` | `Msg::parse` 全部类型、错误路径、`format()` 往返、`Hash`/`Eq` |
| `parser.rs` | 注册（重复 id / name 拒绝）、`register_all`、`lookup`、编码/解码往返、批量解码、未知 id、回调绑定与替换、`Arc` 共享 |

### `mcu`

| 模块 | 覆盖 |
|------|------|
| `pending.rs` | 注册/配对/取消、未知名字不消费、先到先得、接收端已关闭、只取消一条、`abort_all` 唤醒全部等待（`test_abort_all_wakes_every_waiter_at_once`） |
| `dictionary.rs` | 三张消息表的解析（含 `output` 原样保留）、枚举单值与区间展开、常量、各类畸形输入、`install` 的跳过语义与不注册 `output` |
| `mod.rs` | 构造后未识别（`new` 只注册 identify 一对）、发送错误路径、`Drop` 中止接收任务并释放阻塞读；序号（假设备）：**接管一块还在跑的板子**（首帧是 NAK 号 → 采纳、换号重发同一请求、调用成功、`took_over_session()` 为真、记录器显示发的是 `[0, 9]`）、**首个新序号才吃豁免**（开场先重复本会话初号的遗留帧不消耗豁免——`test_the_first_new_sequence_is_adopted_after_a_repeated_frame`：其后的高号 acknak 照样被采纳对齐、调用成功）、刚开机的固件不接管也不重发、首个新序号之后的越号帧被丢且不扰动本次交换；**改号（静默即 nak）**：`test_renumber_adopts_the_firmware_sequence_and_clears_the_window`——改号采纳固件报告的号并清空在途窗口，发起者只有 identify 的重试（`settle` 刻意不对空帧单独改号）；**字典前噪音帧**：`test_an_unknown_id_frame_before_the_dictionary_is_skipped`——`stats`(id=-12) 噪音与响应同号交错时静默跳过、不误消费 pending、不引出任何重传/改号（把跳过突变成 break 即转红）；**RTT 估计与告警**：共 14 条——首样本 ×10 保守起步、双侧平滑、`max(4·rttvar,1ms)` 下限、25ms/5s 夹取（同钉住两常数未改）、估计器记录与派生、告警状态机三态（首超阈一次、翻倍再告、回落不重告）、文案含实测值与排查提示、ack 出样本、**重传作废样本**（后两条为 FrameMock 时序真断言；公式突变两处均转红）；**RTO 消费**再 4 条——无样本地板+翻倍维持现状、样本到达接管等待值、成功回估计值而非地板（成功分支突变回 `MIN_RTO` 即转红）、翻倍值被新样本拉回；**时钟估计（C3）**再 5 条——中点半程 RTT offset 三元组断言、窗口最小二乘拟合频率（1000 ppm 漂移）、旧样本清出窗口、无漂移与旧快照等价、种子端到端夹逼（两轮转红：中点改 `received` / 拟合改单点）；**B5 接收窗与改号（三处转红）**：`test_a_frame_past_the_window_still_reports_where_the_firmware_is`（丢帧仍记固件上报号——删 store 即红 left:110/right:112）、`test_a_renumber_rearms_connection_init_for_the_answer_behind_it`（改号重挂一次豁免——删 connection_init 即红 Ahead/Adopt）、`test_a_congruent_frame_is_no_decision_at_all`（本仓 0 相位：同余首帧零决策，按上游相位解读即红，钉住 `seen==0 ⇔ receive_seq==1`）；**C4 两道闸与 move 池**：`test_min_clock_holds_a_message_until_its_release`、`test_req_clock_orders_messages_inside_the_lead_window`、`test_slot_release_and_start_clock_bracket_the_send`（早于侧以**起点 clock** 为基准）、`test_a_full_move_capacity_releases_one_slot_at_a_time`（五相位、`queue_digital_out` 占同池）、`test_move_slots_floor_is_the_slot_freeing_completion`、`test_an_unknown_clock_never_blocks_a_gated_message`（时钟未知全放行）——四组转红（min 闸 / req 提前窗 / 槽位削峰 / 未知兑底各剪即红）；**B6 显式关旧与重绑（三处转红）**：`test_reconnect_closes_the_old_session_and_binds_the_new_sessions_clock`（真 `reconnect` 打 `test:` 假件：100 ms 内旧任务终止、`Arc` 仍 ≥2、send 被拒、时钟 `ptr_eq` 新会话）、`test_a_closed_session_lets_the_reopened_identify_complete`（共享 RecordingWire：旧 next/seen=107/106 → close → 新会话 identify 一次过、无旧序号 drop）、`test_close_stops_the_session_while_the_arc_is_still_shared`（`strong_count==2` 时 close 仍停双任务 + 拒 send/flush）——转红 A：剪 `previous.close()`；B：剪 `install_clock`；C′：只剪 `closed` 旗；夹具 `RecordingWire` 改 **per-view**（独立收队列+关闭旗、共享记录与应答）。**`min_schedule_time`**——`test_min_schedule_time_is_the_upstream_0_100_schedule_lead`（返回 0.100 且等于 `MIN_REQTIME_DELTA`，防两个常量漂移；供 `GCodeRequestQueue` 对齐 `next_min_flush_time`）。另有**要真硬件的**一例（`test_frame_sequence_sync_against_a_real_board`，`#[ignore]` + `KLIPPERX_HW_SERIAL`）：对同一块不停机的板子连两次，第一次完成 identify 并跨过 4 位回绕，第二次必须报告接管、采纳固件当前的号并继续 `get_clock` |
| `object.rs` | `McuObject`：主/前缀 section 的名字（`[mcu]` → `mcu`，`[mcu zboard]` → `zboard`）、配置构建器在建对象时就可用（可在 connect 前领 oid）、未连接时报 `{}`、连接后报 identify 快照（`mcu_version` / `mcu_build_versions` / `mcu_constants`）、section 没有可用接口时 `connect` 报错、两个对象不能用同一个 chip 名；**握手期停机**（先绑只记录处理器）：假件用 `is_shutdown` 回 `get_config`（`test_a_stop_during_the_handshake_fails_the_connect_at_once_with_its_reason`：<1 s 失败且文案含 MCU 名与原因；剪掉 watcher 即退化成 5.001 s 超时转红）、固件主动发 `shutdown` 帧（`test_an_unsolicited_shutdown_during_the_handshake_fails_the_connect_at_once`）、首条原因先到先得（`test_the_first_stop_reason_is_the_one_kept`）、重开清槽（`test_a_reopen_starts_with_a_clean_connect_shutdown_slot`）；**就地复位有界重试**：回来仍停机则再复位并最终连上（`test_a_reset_that_comes_back_stopped_is_reset_again_and_connects`，只保留 `ResetRequired` 判定即转红）、次数上界（`test_a_come_back_stopped_is_retried_only_a_bounded_number_of_times`）、用尽后报 `MCU '<名>'` + 原因（`test_a_bring_up_that_runs_out_of_resets_reports_the_mcus_name_and_reason`）；**固件停机**：收到 `shutdown` 帧后打印机进 shutdown 且状态消息带原因；**`rpi_usb` 没法复位固件时**（`usb_reset_unusable`：hub 报不支持端口供电切换、开关本身失败、固件还在旧会话里、握手后它仍带着配置）把这个 MCU 的 `restart_method` 记成 `command`（内存里，`Printer::override_config`），第一种在真正去切电之前就发生；**`mcu_clock_poll`**：种子成功后注册 1 s 轮询（排程 0.5+0.5 断言、线上恰 1 帧 `get_clock`、样本折进回归、查询失败不喂不改排程且不 panic、`release_cycles` 后停火——转红：剪断取样路径前两条变红）；**`before_firmware_restart`**（在拆机之前、活连接上）：`command` 发 `reset` 并 flush（`test_a_firmware_restart_resets_on_the_live_connection`），物理方式（`rpi_usb` 等）不发（`test_a_physical_restart_method_does_not_reset_on_the_live_connection`） |
| `config.rs` | CRC 标准校验值；oid 从 0 单调发号、走完 `MAX_OIDS` 报错不回绕、定稿后不能再领；`build`：空配置只有 `allocate_oids` + `finalize_config`、`allocate_oids` 带最终计数、命令按加入顺序、CRC 确定且对值与 oid 数敏感、`restart`/`init` 不入 CRC、config 回调在 build 时跑且可继续领 oid/加命令、二次 `build` 报错且不重跑回调、定稿后再加命令/回调/队列槽被拒、未 identify 报 `NotIdentified`、移动队列槽计数；`seconds_to_clock` 用 `CLOCK_FREQ`；`configure`：未配置时把整份配置加 `get_config` 一帧发出并确认、停机或 CRC 不一致时先 `config_reset`（运行中的固件先 `emergency_stop`）再配置、无 `config_reset` 时分别报停机 / CRC 两种配置错误；`Configured` 三个字段：`crc` / `move_count` / `reused`，加 `already_running`（首个 `get_config` 就报已配置或已停机 = 板子没重启） |
| `restart.rs` | 空实现（`command`）与一条不是 USB tty 的串口路径各自的路由与报错；启动探测 `check_usb_power` 只对“串口 + `rpi_usb`”给结论，别的组合一律 `None`（不夺走调用方的 `rpi_usb`）。**要真硬件的没测**：端口开关、`wait_for_new_device` 的重枚举判定、hub 端口的供电能力（`usb::port_power`）都在真机上手工验过 |
| `resource/pin.rs` | `McuChip` 经 `pins` 注册后被 `setup_digital_out` 派发；`McuDigitalOut` 的 build：`config_digital_out` 的 oid/pin 编号/value/default_value/max_duration（2 s × CLOCK_FREQ）、`update_digital_out` 进 restart 列表、`!` 翻转电平、`max_duration` 与 start/shutdown 不一致报错、枚举里没有的引脚报 `Pin 'X' is not a valid pin name on mcu 'Y'`、保留引脚报错；运行期：attach 后 `update`/`queue` 可发送（名字与参数可编码），未 build 与未 connect 各自报错；`resolve_bus_name`：按 `BUS_PINS_<bus>` 预留固件声明的引脚、缺省取名为 0 的总线、`Unknown spi_bus` / `Must specify spi_bus` 两种错误、无总线枚举时原样透传；**析构**：资源注册的 config 回调必须 `Weak` 持有 pins registry（否则 `registry → chip → config → callback → registry` 成环，跨 `teardown` 留住 chip 与其 MCU 连接）—— `test_a_resource_does_not_keep_the_pin_registry_alive` |
| `resource/pwm.rs` | 硬件路径建 `config_pwm_out`（`PWM_MAX` 满量程、restart 的 `queue_pwm_out`）；软件路径建 `config_digital_out` + `set_digital_out_pwm_cycle` + init 的 `queue_digital_out`；`shutdown_value` 非 0/1 的软件 PWM 报错、`max_duration` 与 start/shutdown 不一致报错、`!` 翻转；`next_aligned_clock` 对软件 PWM 按周期上取整、满/全关与硬件 PWM 不调整；`update_pwm` 用估计时钟发送，未连接报错 |
| `resource/adc.rs` | 批量 `query_analog_in`（`bytes_per_report`）/ 旧格式按字典格式串选择；`ADC_MAX` 与 `sample_count*ADC_MAX < 2^16` 上限；`sample_count=0` 不建任何命令；`get_query_slot` 把首报排在估计时钟 +1.5 s；`analog_in_state` 旧格式单值缩放、新格式按 report 周期给每个样本打时钟 |
| `resource/stepper.rs` | 方向变化变 `set_next_step_dir`、连续步变 `queue_step`；`!` 翻转方向线上的方向位；负 `add` 窄化后仍正确；**C5**：`test_a_stepper_reanchors_the_firmware_chain_on_a_reused_firmware`（复用分支 `built.restart` 必含 `reset_step_clock oid=0 clock=0`、且不进 config 哈希——回退 `add_restart_cmd` 即红，同模块另 4 条照常绿） |
| `resource/endstop.rs` | `home_start` 同时武装 endstop 与 trsync（async）；`home_wait` 对主机请求（无固件触发）回 0 |
| `resource/trsync.rs` | 状态报告完成触发组、次级 MCU 报文把组超时拉到最慢那颗；registry 按 oid 路由、同一 MCU 上两个 endstop 共享 registry；一个 trsync 停住多个 stepper；共享轴跨 MCU 被拒（上游 `TriggerDispatch` 同规则）；raw reason 贯通：未知 raw 照样完成 completion、typed 视图折叠、reason 0 不完成（1-4 行为零变化的守卫） |
| `resource/trigger_analog.rs` | 5 命令 + state 响应对字典编解码、错误码四类字典文案与 `SENSOR_SPECIFIC` 走传感器回调、SOS 去重缓存（仅变更才发、state+active 每次发）、超量段/状态数不匹配报错、range/trigger 去重、双 trigger_analog 的 oid 互异且各恰一次 config、非正采样率拒绝；e2e：`set_trigger→home` 首条 move 在监控窗内完成、无样本时 MONITOR 到期回 `Trigger analog error: MONITOR` 不挂起；M5b：`&dyn HomingEndstop` 端到端 arm/wait、stub 生产者投样与错误码回调消费（低于 `SENSOR_SPECIFIC` 走字典） |
| `resource/i2c.rs` | 总线错误（非 SUCCESS）按上游把机器停机 |
| `resource/spi.rs` | **无独立测试**：编码路径由 `cmd/spi.rs` 覆盖，`McuSpi::transfer`/`send` 的线上行为靠 `spi_device` 的节测试与真板手工验证 |
| `events.rs` | 按 id 查回调、绑未注册消息报错、后绑替换旧绑（`McuEvents` 的三条语义） |

### `pins`

| 模块 | 覆盖 |
|------|------|
| `pins.rs` | 描述解析：裸名默认 `mcu`、`chip:pin` 选片、`!`/`^`/`~` 只在允许时生效、未知 chip、畸形描述带上格式提示、重名 chip 被拒；`lookup_pin`：重复使用报错、同 `share_type` 可共享且返回首次的参数、共享极性必须一致、多用途引脚、`reset_pin_sharing` 释放；`PinResolver`：别名解析、别名链、别名不能重映射、别名目标必须是裸名、保留引脚被拒、同引脚两个名字报“is an alias for”、保留冲突、每 chip 一份；对象可查性（`get_status` 空、`is_queryable` 假） |

### `identify`

| 模块 | 覆盖 |
|------|------|
| `identify.rs` | 单块与多块拼装（含短末块与 4 位序号回绕）、offset 错位、zlib 损坏、**裸 deflate 被拒**（必须是 zlib 包装）、非 JSON、MCU 静默（现为「静默即 nak」：块窗口内无响应 → 改号重发，用尽上限才报错）、zip bomb 上限、`Mcu::identify` 与 `Mcu::connect` 全流程；分块夹具按真固件的号（响应与 ack 都盖 `N+1`）；**接管之后握手仍失败时报 `OldSession` 而不是裸超时**（对照：刚开机的板子静默时保持原错误）；**改号重发四件套**（回退其一即转红）：固件领先一号 → 改号后 identify 完成（`test_a_firmware_one_ahead_is_renumbered_and_identifies`）、首答既不改号也不重发（`test_a_healthy_first_answer_is_neither_renumbered_nor_repeated`）、响应紧跟的空帧仍按 ack 处理、会话不受扰（`test_an_ack_with_the_response_right_behind_it_stays_an_ack`）、重试上限边界（`test_identify_gives_up_after_its_retry_cap`，自定 3 次不抄上游 ×5）；**同余噪音并存**：`test_a_congruent_stats_frame_does_not_disturb_the_renumber`——stats 帧与宿主 `seen` 同余 + nak 静默的时序下，首窗（500ms）耗尽才改号、发帧序列恰为映射输入，噪音既不推进 `seen` 也不提前触发改号；**B5 重连护栏三条**（FrameMock 复刻真机 vA 时序；单线程恒绿=集成护栏，论证见模块注释）：`test_a_congruent_first_frame_completes_the_transfer_in_phase`（同余首帧）、`test_an_empty_ack_first_frame_adopts_the_response_behind_it`（空 ack 首帧吃豁免）、`test_a_response_behind_a_renumber_is_still_accepted`（改号后响应不再被判 never-sent），夹具新增 `chunked_mappings_from` 分段映射 |

### `cmd`

| 模块 | 覆盖 |
|------|------|
| `mod.rs` | `Params` 按名取参（含无参消息与 `declared()`）、无损转换与拒绝收窄、未声明参数报已声明列表、类型不符报两侧类型、字符串/字节互换与非法 UTF-8、`get_enum` 的命名 / `?<value>` 回退 / 两类错误；`send_msg` 的握手门禁、成功上线、未知消息、参数不匹配；`call_msg` 的往返解码、超时、未知响应名、解码失败 |
| `identify.rs` | 两个视图对 `IDENTIFY_MESSAGES` 的双向校验（`args()` 的字节形状、编码后解码与 `args()` 一致、按名取 `offset` / `data`）、空 `data` 的完成标记、参数类型或名字不符时报 `Decode`；`IDENTIFY_CHUNK_SIZE` 与 Klipper 的 `count=40` 一致（端到端分块流程见 `identify.rs` 的测试） |
| `gpio.rs` | `config_digital_out` / `update_digital_out` / `queue_digital_out` / `set_digital_out_pwm_cycle` 的 `args()` 与固件格式一致（编码后解码回到同一组值） |
| `pwm.rs` | `config_pwm_out` / `queue_pwm_out` 的 `args()` 与固件格式一致 |
| `adc.rs` | `config_analog_in` / `query_analog_in`（新旧两种）的 `args()` 与固件格式一致；`analog_in_state`（批量 `%*s`）解出 oid / next_clock / LE `u16` 样本；用旧格式问批量声明报 `Decode` |
| `ds18b20.rs` | `config_ds18b20` / `query_ds18b20`（含 `%s` 序列号与 `%i` 毫度范围）的编解码往返与固件格式一致；`ds18b20_result` 解出 oid / next_clock / value / fault |
| `allocate_oids.rs` | `allocate_oids` 的线上形状（id 2 的 VLQ + `%c` 计数）、`u8::MAX` 往返编码一致 |
| `config.rs` | `get_config` / `finalize_config` 的编码形状；`config` 响应按名解码（已配置 / 未配置且已停机两态）、参数类型不符报 `Decode` |
| `uptime.rs` | `get_uptime` 的编码形状；`uptime` 两段重组为 64 位时钟、跨 32 位回绕时排序正确、参数类型不符报 `Decode` |
| `shutdown.rs` | `emergency_stop` / `clear_shutdown` 两个无参命令的线上 id |
| `clock.rs` | 读取时钟、32 位回绕值、握手前失败、超时；另有不依赖 MCU 的 `ClockSync` 实现，验证 trait 作为测试缝可用（文件随 `pub mod clock;` 一并编译）；查询喂时钟估计（`test_a_query_feeds_the_mcu_clock_estimate`） |
| `debug.rs` | `debug_read` / `debug_write` / `debug_ping` / `debug_nop` 与固件格式一致；`debug_result` / `pong` 解码（缺参数报错）；读/写/ping/nop 各自走虚拟 MCU 的整条往返与上线 |
| `spi.rs` | `config_spi`（含 active-high、无片选两变体）/ `spi_set_bus` / `spi_set_sw_bus` 编码；新旧软件总线命令的优先选择（这三例是 async 真调，`mcu_with` + `FrameMock`）、两个都没有时报错；`spi_send` / `spi_transfer` 的编码与 `spi_transfer` 响应解码；`config_spi_shutdown` |
| `i2c.rs` | `config_i2c` / `i2c_set_bus` / `i2c_set_software_bus` 编码；新旧软件总线命令选择（async 真调）；`i2c_transfer` / `i2c_write` / `i2c_read` 的编码与响应解码（`i2c_response` / `i2c_read_response` / `i2c_bus_status`） |
| `thermocouple.rs` | 命令与固件格式一致；芯片类型号与固件枚举对齐；`thermocouple_result` 解码；整条经虚拟 MCU 的往返 |
| `stepper.rs` | `config_stepper` / `queue_step` 的 `args()` 与固件参数序一致（编码后解码回同一组值） |
| `endstop.rs` | `config_endstop` / `endstop_home` 的参数序与固件一致；`pull_up` 负值按字节编码；disable 全零；`home_wait` 的 32 位触发时钟以**本次 move 的 arm clock** 为纪元参考（打印时间远超 MCU 自报时钟时会差一整圈：2³²/16 MHz = 268.44 s）；时钟门控：`test_home_start_waits_for_its_reqclock_window`、`test_query_endstop_holds_until_its_minclock` |
| `trsync.rs` | `trsync_start` 参数序与固件一致；`trigger_reason` 枚举号与固件对齐；`raw_failure_classification` 覆盖 typed 与 trigger_analog 码（1-3 非失败、4 与 5-8 失败且 5-8 无 typed 变体） |
| `trigger_analog.rs` | 5 命令逐字段对 `atmega2560.dict`、`trigger_analog_type` 枚举逐值一致（`abs_ge`/`gt`/`diff_peak_gt`）、home 载荷字节序与全零 disable、state 响应经字典格式串编解码往返 |
| `trigger_analog.rs`（主机侧 design，M5b） | `to_fixed_32` 缩放与舍入（ties-to-even 同 Python）及上游溢出文案、`calc_frac_bits`（整数→0、|x|<1→31、按 bit_length 收窄（表 2.0→29）、舍入溢出回退一位 30→29）、SOS 表命中与三路 miss、tap 设计→表段→导数序、scipy 表外错误不改设计、定点系数 29 位精确值、col3≠1/超宽拒绝、静止态按起始值换算 |
| `sos_filter.rs` | 5 条 SOS 命令逐字段对字典（含 `%i` 负数有符号）、`set_section` 5 系数顺序与负值编码 |
| `ldc1612.rs` | 5 条命令与 `sensor_bulk_status`/`sensor_bulk_data` 响应逐字段对照语料字典；按名解码 6 个 bulk 状态字段 |

### `event`

| 模块 | 覆盖 |
|------|------|
| `mod.rs` | `Mcu::bind_event` 端到端投递（绑定的回调经 `Parser` 回调收到事件帧）、握手前 `NotIdentified`、字典缺失报 `UnknownMessage`、占位日志订阅可绑定 |
| `stats.rs` | `stats` 事件按名解码（`count` / `sum` / `sumsq`）、参数类型不符报 `Decode` |
| `shutdown.rs` | `shutdown` 解出 `static_string_id` 原因与可选 `clock`、`is_shutdown` 只解原因、`starting` 无参；原因不在枚举里时报 `?N` 而不编造 |

### `config`

| 模块 | 覆盖 |
|------|------|
| `mod.rs` | 解析语法与上游 `configparser` 对齐：节头可带 `#` / `;` 行内注释、`:` 与 `=` 等价且取最先出现者、非空首行的缩进续行（值以换行连接）、缩进的 `[x]` 是续行而非节头、`;` 仅在行首或前为空白时开始注释、引号内的 `#` 保留；**选项名统一小写**（`optionxform = str.lower`），节名与值保留原样；重复选项（同节内大小写不同）按上游行为取最后写入者 |
| `wrapper.rs` | 类型化 getter 与范围/取值文案（`get_choice`、`get_float_bounded` 等）、`get_list` 记账；`deprecate` 只对写过的选项记一条警告；**选项名大小写折叠**：任意查询大小写可读、`must be specified` 保留调用方大小写、同节大小写重复后者胜、`prefix_options` 返回小写名、access 键小写、节名与多行值保留原样 |
| `object.rs` | `configfile` 状态形状与 `set`/`remove_section` 的 pending 记账（含节移除记为 `null`）；`warnings` 的五种形状（`deprecated_option` / `deprecated_value` / `deprecated_gcode` / `deprecated_mcu_code` / `runtime_warning`）的字段与上游文案、按序列化键去重；`set`/`remove_section` 同步维护块 fileconfig（`autosave_fileconfig`/`has_pending_sections`） |
| `save_config.rs` | `SAVE_CONFIG` 回写：把待写项追加进 `#*#` 块（正文保留、块加新值）并以带时间戳备份换旧文件；无预先块时新建块；无待写项时文件原样不动；正文里与块重复的选项被注释掉（块获胜）；多行值写盘回读回环（`write_config_round_trips_a_multiline_value`，2026-10-03 +1） |
| `access.rs` | 读取账本：按名（大小写不敏感）登记、按节分组、每节只列一次 |
| `section.rs` | 节存储：按 id+sub 区分同名选项、替换不重复、保持插入序、按 id 过滤遍历；`get_list` 去空白丢空项并拼多行值、缺项报错、`get_list_of_lists` 解析 `名:值` 对与条数不对的报错；`get`/`get_str`/`get_text`/`has` 按小写键查询（存储侧已是小写）、节名不折叠 |
| `source.rs` | 配置来源的 `Display` |
| `mcu.rs` | `[mcu]` 解析：名字与 `restart_method` 拼写、串口默认 `arduino` / 非串口默认 `command` 且忽略该选项、未知 `restart_method` 报错、`usb_power` 默认 `auto` 并校验、传输键二选一（`serial` / `canbus_*` / `host_library` / `test`）、`canbus_uuid` 按上游格式解析与 `canbus_nodeid` 校验、无 uuid 时的报错、波特率在开 port 前拒绝 |
| `validate.rs` | `check_unused`：没人认领/读过的节与没人读过的选项各自报上游文案；只读节合法；名字大小写不敏感 |

### `motion`

规划与步进生成（上游的 `toolhead.py` + C `chelper/` + `motion_quuing.py` 三处的 Rust 对应）。
| 模块 | 覆盖 |
|------|------|
| `kinematics.rs` | 未 homed 的 move 被拒、homed 轴在界内接受、越界拒绝；对角 move 被 Z 限速；corexy/corexz 把 rail 位置映射到台面轴、hybrid 只映 X；extrude-only move 不归运动学管；`get_status` 报 homed 轴；`calc_position` 从 stepper 读轴；`none` 运动学照单全收；`home` 算出 force/move 两端（1.5 倍轴长） |
| `plan.rs` | move 剖成加速-巡航-减速、extrude-only 无运动学距离；拐角被 junction 速度限制、直线保持巡航；前瞻攒够时间才 flush、偷懒 flush 不弄坏短队列 |
| `stepcompress.rs` | 等间隔→一条 move、加速/减速/二次曲线段与上游对拍；方向切换插 dir 命令、大空隙变单步重锚、SDS 过滤在反向前丢步；flush 等 move clock、`set_last_position` 记 history 标记、`find_past_position` 与发出的时刻表一致、history 过期清理；抖动序列与上游对拍；一整段压缩重建出请求的每一步 |
| `trapq.rs` | 梯形变三段、相位连续、时间空隙填静止段、坐标沿加速度走；`finalize_moves` 过期段、`extract_old` 按窗口取段 |
| `itersolve.rs` | cartesian 只读自己的轴、corexy 读 x±y、corexz 读 x±z；不动的 stepper 是惰性的；步距处生成步进、`generate` 走完整 trapq、位置坐标往返 |
| `stepper.rs` | cartesian stepper 只动自己的轴、每个 stepper 在自己的 MCU 时钟里生成、`mcu_position` / `past_position`、`generate` 交出该 stepper 的命令 |
| `toolhead.rs` | 额外轴（挤出机）在自己的 trapq 检查与排队、限制拐角；move 到 trapq 生成 `queue_step`；两条共线 move 保住拐角速度；`dwell` 推进 print time；`drip_move` 直灌 trapq（零长度不做事）；未 homed 轴拒绝、零长 move 忽略；**C5 五测（两组转红）**：实时 est 打底（`print_time ≥ live_est + BUFFER_TIME_START`，回退成 connect 快照即 5 条全红）、`need_prime_rearm::{wait_moves,dwell,flush_step_generation}` 三条复位路径（去复位恰好 3 条红）、入队批 `req`/`completion` 域界（`req ≤ est+0.100`、`completion ≥ est`） |
| `queuing.rs` | `append` 与 `generate` 共用同一 trapq；没事做的 stepper 静默；两个 stepper 在同一 MCU 上都生成；`register_flush_callback` 的回调在 `generate(flush_time)` 开头**按注册序**触发、**无 stepper 也照跑**（`test_flush_callbacks_run_in_registration_order_even_without_steppers`） |
| `extra.rs` / `mod.rs` | 无独立测试（额外轴的检查/排队由 `toolhead.rs` 覆盖） |

### `gcode`

| 模块 | 覆盖 |
|------|------|
| `gcode.rs` | 解析：传统命令拆字母+值、行号 `N…` 跳过、`;` 注释与空行、扩展命令 `KEY=VALUE`（带引号/注释）、畸形参数报 `Malformed command`、传统/扩展与扩展名校验；分派：注册后运行、重名/非法名被拒、未知命令只提示不报错、`CommandError` 停下脚本并回 `!!`、未 ready 时 ready-only 命令报状态、内置 `M115` 在未 ready 时也可用、`M112` 触发 shutdown、`HELP` 列命令；mux：按 key 选处理器、未注册值列出选项、只能一个 key；参数访问器：缺参/解析失败/默认值/范围；`get_status` 报命令表；**析构**：命令表不能反过来持有 dispatcher（内建命令与 mux dispatcher 都用 `Weak<Inner>`），否则整份 dispatcher 连同其 handler 捕获的资源——直到一条仍在读串口的 MCU 连接——会跨 `teardown` 泄漏，`firmware_restart` 时新旧连接抢帧—— `test_dropping_the_dispatcher_frees_its_handlers` |

### `printer`

| 模块 | 覆盖 |
|------|------|
| `printer.rs` | 状态与事件名即线上名；生命周期：新机器是 `startup`、`bring_up` 先按注册顺序 `connect` 每个对象再上线到 `ready` 并按序发 `connect`/`ready`/（firmware_restart）/`disconnect`、对象 `connect` 失败即 `invoke_shutdown` 并带上原因、已停机的机器 `bring_up` 不 connect 任何对象、handler 按注册顺序调用、`run` 等另一线程的 `request_exit`（先在另一线程起 `run`）、先请求退出则不等待、首个退出结果固定、`invoke_shutdown` 只接受首条消息、停机后 `bring_up` 不会变成 `ready`；对象表：**新机器没有任何对象**（`webhooks` 是主机侧的）、注册顺序、重名被拒且首个注册保留、按名 `lookup_object` 拿得到且未注册返回 `None`、`connect` 默认是空实现；时间：`eventtime` 就是机器的 reactor 的钟（`ManualReactor` 拨表后跟着变）、`Printer::reactor()` 交回的正是建机器时给的那个；**内存覆盖**：`override_config` 按 section 分存、重记即覆盖、别的 section 不受影响；`prepare_firmware_restart` 按注册顺序 await 每个对象的 `before_firmware_restart`（`test_prepare_firmware_restart_awaits_every_part`） |
| `mathutil.rs` | `coordinate_descent`：二次函数收敛到 1e-4；耦合残差的平面拟合恢复已知平面到 1e-3（实测 4.4e-6）；误差永不改善时的步长阈值退出；每轮都改善时的 10000 轮上限（精确断言 20001 次误差调用，防死循环） |
| `load.rs` | `Printer::load_config`：主 section 先于前缀 section、按 section identifier 登记、`[mcu]`→`mcu` 与 `[mcu x]`→`mcu x`、未知 section 报上游原文 `Section 'x' is not a valid config section`、空配置装载为空、坏接口要到 `connect` 才报、**交给工厂的 section 带上打印机记的内存覆盖**（`override_config` 改的选项真的会被读到，其键名与解析器一样按 `optionxform` 折叠）、**工厂调用 `ConfigWrapper::deprecate` 时警告落到装载器带进来的 `configfile` 对象**；**`configfile` 对象带上 `#*#` 块并注册 `SAVE_CONFIG` 命令**（命令用 `printer.start_args().config_file` 落盘并 `request_restart`） |

### `reactor`

时间与定时器。`ManualReactor` 的测试不需要 runtime，直接拨表；`TokioReactor` 用 `#[tokio::test(start_paused = true)]` 与 `tokio::time::advance` 驱动暂停钟。

| 模块 | 覆盖 |
|------|------|
| `reactor.rs` | 契约：回调返回值即下次唤醒时间（`Some` 续、`None` 退）、未到点不跑、`advance` 把钟拨到每个唤醒时间（回调看到的是被唤醒的时刻，不是终点；周期定时器一个周期跑一次）、同时到期按唤醒时间稳定排序、`NOW`（注册在过去）当前就跑、取消后不再跑且可重复取消；`ManualReactor`：从 0 起、`run_due` 不拨钟、回调里可再注册（不在锁下跑回调）、`call_later` 只跑一次；`TokioReactor`：到点才跑、按周期续跑、取消从长睡眠中立即返回且不再跑、已自退的定时器再取消也安全、丢句柄不取消；**串行 dispatcher**：同一时刻到期的按注册顺序一次跑一个（回调互不重叠）、回调里再注册不死锁；**延迟度量**（真时间）：慢回调被报出且带名字、快的一轮不报、被慢回调挡住的后继定时器 `lateness` 超阈 |

### `klippy`（主机编排）

| 模块 | 覆盖 |
|------|------|
| `klippy.rs` | `is_restart` 只认 `restart` / `firmware_restart`；`klippy_process` 一回合内 `restart` 重建、`exit` 结束（用 `RestartOnce` 替身对象）；重启从磁盘重读配置（`test_a_restart_rereads_the_config_file_from_disk`，以 `configfile` 的 raw config 从 `50` 变 `120` 为证）；配置文件解析失败时进 `error` 而循环不挂死（`test_a_restart_whose_config_no_longer_parses_reports_error`）；**A3 机制**：在 `machine_handle.enter()` 之下建的 `test:` 接口捕获到的是机器 runtime，而不是 ambient 的 API runtime（`Interface::with_transport` 的 `Handle::current()`） |

| `stress.rs` | 段计算与引脚解析：按节名找 MCU（带名的不拿裸 `[mcu]`、空名拿裸的）、引脚名的 chip 副本只在有冒号时出现、别的 MCU 上的 stepper 被跳过；一段填满时长且间隔均匀、时钟变慢拉长间隔而时长不变、间隔不到 0 被截且命令有上界；引脚经字典枚举解析；**多机**：选择与汇总（`mcus_are_selected_by_name_or_all_and_summarised_per_board`：多值/去重/`--all-mcus`/互斥、前缀与收尾文案）、旧单机命令行等价（`the_old_single_board_command_line_still_selects_the_default_mcu`）、双假设备**并发 identify+驱动**不串（`two_fake_boards_identify_and_drive_concurrently`，两字典命令数不同为证、回调各自恰 25 次）、**帧级隔离**（`concurrent_sessions_send_only_their_own_frames_to_their_own_recorder`，两 Recorder 各恰 8 帧、无异板 payload）；**多板 stepper 归属与并发**：`a_stepper_belongs_to_the_board_whose_step_pin_it_uses`（同板多节全收、endstop/enable 落他板不改变归属）、`a_dir_pin_on_another_board_rejects_only_that_section`（固件文案、按节拒）、`all_mcus_drives_the_board_with_a_stepper_and_skips_the_one_without`（核心回归：单板有节时 `--all-mcus` 照开、跳过行、点名仍硬错误）、`one_stage_drives_every_stepper_of_the_board_with_the_same_schedule`（同板并发、同一 ramp、按轮交错）、`every_stepper_gets_its_own_oid_in_one_config_round`（一轮配置装全部：allocate+N×config_stepper+finalize）、`a_skipped_board_keeps_the_exit_clean_but_a_run_that_skipped_everything_fails`（跳过≠失败、全跳过才报）；回退转红：剪断多节收集（`take(1)`）即红；**M3 会话所有权**（现 23 条）：`the_reset_reconnect_drops_the_old_session_before_reopening`（`test:` 夹具强制走 ResetRequired：Weak 监视旧会话→15ms 排空→断言 `strong_count==1` 才 drop→250ms 重连→新会话 identify+配置+ramp 全通→`weak.upgrade().is_none()` 证 Drop 执行）+ `run_board` 入口 `strong_count==1` 断言；回退两形态均红（字面 `mcu.clone()` 跨 join→入口断言 left:2；链内中途 clone→重连断言 left:2）；完整双板 ramp 并发需真板（待验项见 stress.md） |
| `main.rs` | CLI 形状：不给子命令时跑主机、选项随默认子命令走、显式拼法同效、单独给子命令回帮助；`--api-server` 每处默认一致、空值=主机不开服务；`--tui` 是 CLI 自己的、客户端子命令不受影响；`--logfile` 两种拼法都是主机的；裸调用打帮助、缺配置文件的旗标后置才报、主机子命令单跑打帮助、主机参数与子命令不可混用 |
| `logging.rs` | 窗口收到主机记录；`--verbose` 与 `RUST_LOG` 取更详细者；窗口槽在层级查找处；`--logfile` 收到格式化字节、打不开则降级 stdout；rollover info 排序并清空 |

### `klippy-api`

两层测法：协议与寄存器层**不需要 socket**（`ClientConnection::receive` 直接吃字节，推送由实现了 `PushTarget` 的测试替身接住）；监听与连接层用**真的 socket**，在临时目录里 bind Unix socket、在 `127.0.0.1:0` 上 bind TCP，然后真连上去发请求。手工验证用 `klippy-client api` / `klippy-client console`（见 [第三方开发手册](../third-party-dev/README.md)）。

| 模块 | 覆盖 |
|------|------|
| `address.rs` | 裸值是 socket 路径（上游形式）、`unix:` 前缀的两种写法、`tcp:` / 裸 `host:port` / 名字 / IPv6 / `:port` 都识别为 TCP、`Display` 带传输前缀、空值与 `unix:` 无路径被拒、`tcp:` 后面端口不是数字 / 缺端口被拒、未知 scheme（`http://…`）报错而不是变成文件名 |
| `protocol.rs` | 分帧（一次读里多条、一条被拆成多次读、前导分隔符产生的空消息）、`encode` 的分隔符结尾；请求解析（`id` 缺失与为 `null` 均视为不要应答、非对象 / 无 `method` / `params` 非对象一律拒绝、非字符串 `id` 原样保留）；应答形状（result / error、无 `id` 时失败也静默）；`Params` 区分缺失与类型错、整数可作浮点而浮点不可作整数、`true` 不是整数、`get_or` 不检查默认值；模板合并（`params` 冲突时模板优先，同上游）、模板可省略且类型受检 |
| `registry.rs` | 路径唯一（普通端点与 mux 路径同一命名空间）、mux 各实例的 key 必须一致且 value 不重复、`list_endpoints` 排序并含 mux 路径、按名分发与未知方法报错、端点拿得到自己的连接、mux 按 key 选实例 / 缺 key / 未知值 / 非字符串值、注册 `None` 时 key 可省略；**运行期可变 mux 表**：建表后仍可 `register_mux`（`test_a_mux_endpoint_can_be_registered_after_the_registry_is_built`）、`clear_mux` 清空路径（`test_clear_mux_drops_every_instance`）并对每个实例调 `detach`（`test_clear_mux_detaches_every_instance`）、清空后同 `(path,value)` 换新 handler 再注册且分派到新的（`test_a_cleared_path_can_be_registered_again_with_a_new_handler`）、`mux_registrations` 报路径当前的 key 与实例值（`test_mux_registrations_reports_what_a_path_has`）；remote method 的模板合并推送、多连接、重复注册替换模板、已断开连接被清理、无活动连接与未注册两种错误 |
| `server.rs` | **真 socket**：Unix 与 TCP 各一个往返、TCP 上报的 target 是绑定后的地址（不是 `:0`）、未知方法与 handler 失败都回 `error`、一次写里两条请求按序应答、跨 TCP 分段的消息只应答一次、两个客户端同时被服务。**socket 文件**：遗留文件名被 bind 替换成真 socket、server 被 abort 后文件消失。**单连接**（无 socket）：应答字段顺序（`id` 在前，靠不经 `Value` 直接序列化保证）、无 `id` 不应答、畸形消息被跳过而连接继续可用、推送与应答同样入队、连接关闭后不再入队、`push` 能唤醒等待方（用 `Notify`，带超时断言）、空发件箱 flush 直接成功、写不进去的客户端被 5 秒超时切断（用 `start_paused` 让暂停时钟直接跳过这 5 秒，无需真等） |
| `error.rs` | `Bind` / `Connect` / `Io` 的 Display 直接给出调用处拼好的文本，`Closed` 给出固定文案 |
| （主机侧）`endpoints/info.rs` | 端点路径、`client_info` 可省略且必须是对象、响应 12 个字段与文档逐个对齐、`log_file` 为 `None` 时是 `null` 而非缺字段、响应不回显 `client_info`；**handler**：状态跟着机器（`startup`→`shutdown`）、四个 start args 与 `process_id` 真的进响应、`klipper_path` / `python_path` 非空且确实不存在 |
| （主机侧）`api/start_args.rs` | CPU 描述的解析（processor 计数 + model name、缺 model 时问号）、`collect` 带上配置路径 / 版本、`log_file` 为 `None` |
| （主机侧）`api/webhooks.rs` | 对象名是 `webhooks`、对象报的就是打印机状态（`startup` / `ready` / `shutdown` 三种都跟得上，因为它是读状态而不是存状态）；**mux 表生命周期**：`set_api` 后注册直连 `Api`（无需 drain）、`Printer::teardown` 经 `release_cycles` 清表并 `detach`，随后同 `(path, value)` 可重新注册（`test_a_registration_after_set_api_reaches_the_table_and_teardown_clears_it`）；两条装载路径的冲突检查与文案一致（`test_a_conflict_after_set_api_uses_the_same_message`） |
| （主机侧）`api/mod.rs` | `register` 一次装完服务器这一侧：装完 `objects/list` 含 `webhooks`、端点表含 13 条路径（`info` / `emergency_stop` / `list_endpoints` / `objects/*` / 五个 `gcode/*` / `query_endstops/status` / `register_remote_method`）；装完后客户端能按名查到 `info`、能按名查到 `webhooks` 并跟着状态变；**重复注册报错**且归为「自己接错线」的 `RegistrationError`（不是客户端能引起的错误） |
| （主机侧）`endpoints/objects_list.rs` | 端点路径、没有组成部分的机器列表为空、按注册顺序列出多个对象、不看参数（上游的 handler 不读任何参数） |
| （主机侧）`endpoints/objects_query.rs` | 端点路径、`null` 取全部字段 / 列表取指定字段 / 不存在的字段回 `null`、未知对象回 `{}`（不报错）、对象名与应答的 `eventtime` 一致且真的传给了源、服务器那个 `webhooks` 对象随机器 `startup`→`ready`→`shutdown` 变化、`objects` 缺失 / 非对象 / 值非 `null` 或字符串数组分别报三种错（文本对齐上游的 `Invalid argument`）、空字段列表取空、一次查询多个对象、空 `objects` 是空 `status` |
| （主机侧）`endpoints/objects_subscribe.rs` | 订阅请求**立即**回一份全量快照（所有请求字段都返回）、0.25 s 后只推变化的字段、无变化不推、`response_template` 包住每次推送、`null` 字段列表展开为对象当时的字段、字段从缺到有会推而一直缺不推、未知对象回 `{}` 且不推、连接关闭后下一 tick 清理并自行停掉定时器（再没有 tick）、同一连接再订阅是替换不是追加、一个定时器服务多个订阅者、注册表按路径可达、参数校验与 `objects/query` 一致、`response_template` 非对象被拒 |
| （主机侧）`endpoints/gcode.rs` | 五条路径名；`gcode/help` 返回扁平命令表；`gcode/script` 执行并回 `{}`；处理器错误变成**不关停 klippy** 的 `ApiError::CommandError`；缺 `script` 报 `MissingArgument`；`gcode/firmware_restart` 走到内置命令并让 `run()` 返回 `firmware_restart`；`gcode/subscribe_output` 把 `// …` 输出按模板推到连接；`gcode/restart` / `gcode/firmware_restart` 在 `gcode` 缺席时回 `{}` 并让 `run()` 返回对应结果（`test_restart_without_a_dispatcher_still_restarts`、`test_firmware_restart_without_a_dispatcher_still_restarts`）；`gcode/script` 在 `gcode` 还没注册时仍报打印机状态 |

| （主机侧）`endpoints/emergency_stop.rs` | 请求把打印机停机并回 `{}`（上游 `emergency_stop` 语义） |
| （主机侧）`endpoints/query_endstops.rs` | 路径是上游那条 `query_endstops/status`；无 endstop 时回空对象 |
| （主机侧）`endpoints/register_remote_method.rs` | 注册的方法收到模板与参数；`response_template` 可选；缺 `remote_method` 报参数错；只推给注册的那条连接 |

### `klippy-client`

两段：会话层与连接层用**真 socket**（每个用例自己起一个 `Server`，因此测的是「两边真能对话」）；窗口是**纯渲染测试**，用 ratatui 的 `TestBackend` 画进内存缓冲再断言每行文字，不需要终端。

| 模块 | 覆盖 |
|------|------|
| `lib.rs` | `PARAMS` 必须是 JSON 对象（数组 / 数字 / 非 JSON 分别报错并说明实际类型）、`--api-server` 与主机同一套解析（socket 路径、`tcp:`、`http://` 被拒） |
| `tests/gcode_params_table.rs` | 签入的内建参数表 = 新鲜扫描（`gen-gcode-params` 重生成的表才过）；`call_sites` 钉住总数（现 81，防扫描器静默丢调用点）；`KNOWN_UNRESOLVED` 钉住未解析调用点数（4 条）与所在文件集（`gcode_macro.rs`/`gcode.rs`）——主机侧新增注册点时本测先红，提示重生成 |
| `connection.rs` | 请求得到应答且 `id` 与发出的方法名配对、错误应答仍然是应答（可读到 `error.message`）、无 `id` 的消息归为推送、`"id": null` 发出后不等待也不登记、一次读里的多条消息都会被依次交出、连接被挂断时报连接错误而不是挂着、连不上时错误里带上尝试过的地址、**被取消的读不会影响下一次读**（窗口与行模式每读一行输入都会与 socket 竞争，这条是回归测试）、TCP 也能连；`forget_pending` 撤销超时请求的 id（两条测试：撤销后不再欠它、迟到的应答仍被读且 `method` 为 `None`） |
| `session.rs` | 握手发出 `info` 并把应答登记、裸方法名会补 `id`、方法 + YAML 参数（`{value: 3}` 无需引号）、参数写错只提示不结束会话（YAML 序列 / 标量分别报“必须是 mapping”、未闭合的 mapping 才报解析错）、整条请求对象可以是 YAML（`{method: echo, params: {value: 7}}`，显式 `id` 不被覆盖、缺 `id` 时补上、`"id": null` 保留且不登记）、既不是本地命令也不是请求对象的行仍按 `method [params]` 处理、缺 `method` 本地拒绝、空行什么也不做、本地命令不发给服务端（含 `/quit` 的三种写法）、`/subscribe <名字>` 的请求体与模板、`/subscribe` 无参数时先 `objects/list` 再订阅全部（两个请求都要登记）、失败应答的文本、每条请求都记成 `Sent`、`Entry::text()` 的四种形状、`drain` 会打齐欠着的应答且无事可做时不等待、**g-code 模式**把整行发 `gcode/script`（`/` 开头的 `/quit` 仍是本地命令）、`subscribe_gcode_output` 发出的模板；`/reload` / `/reload_config` 发 `gcode/restart`（空 params，两种输入模式都回 `Continue`）；`LOCAL_COMMANDS` 与 matcher / `usage()` 的漂移守卫（每个规范名与别名都走一遍 `handle_line` 断言不报 unknown，`usage()` 覆盖全部规范名）；`gcode/help` 的三种结果（成功→按命令名排序、被拒→`None`、不答→超时 `None` 且 pending 清空（不再欠这条应答）） |
| `tui.rs` | 行编辑（光标处插入 / 删除 / 行首行尾 / `^A` `^E` `^U`）、历史前进后退到空行、长行按光标滚动（游标要占一格）、渲染：状态行随 `info`、`objects/subscribe` 应答的快照与 `webhooks` 推送更新（窗口连上后自己订阅 `webhooks`，`:sub` 只是手动再看一遍）、日志里的应答/推送/错误/通知各自成行、应答/推送/发出的请求都以 `<`/`>` 开头空一格再写正文，正文默认 **YAML**（`/json` 切回紧凑 JSON，标记与空格不变；错误仍是单句）、相邻两条消息用两种前景色交替（`Color::Cyan` / `Color::White`；错误仍红；交替只数消息，中间的日志行不打乱）、`/yaml`/`/json`/`/help` 由窗口自己接（`/help` 在会话说明后补 Window 一节），`! ` 前缀、翻页提示与回到底部、新条目回到最新处而翻回去看的人保持原位、输入行与 `local>` 提示符、**g-code 模式**的 `gcode>` 提示符与 `^G` / `/gcode` 切换（本地命令仍用 `local>`）、**g-code 模式剥掉 API 信封**：`gcode/script` 请求显示为 `> <脚本>`、`gcode:output` 推送显示为 `< ` 标记的打印机原文（`// ` / `!! ` 前缀不动，多行回答续行缩进两格）、成功的 `gcode/script` 应答与 `gcode/subscribe_output` 等 `gcode/*` 管线请求/应答被丢弃、**失败应答保留**（调度器还不存在时订阅也失败，没有 `!! …` 输出行可兜底，应答是错误唯一的去处）、请求模式下同样的管线条目照常显示、非 g-code 通道仍带信封、g-code 交换按方向分色（`>` 青、`<` 白、`< !!` 红；仍计入双色交替序号）、改变 render 选项（进出 g-code 模式）后按新选项重测日志高度、切回请求模式后还留着的条目恢复信封、断线在状态行显示、样式分类（日志行按 tracing 五级配色、消息青/白交替、错误应答=红）；**命令补全**（`Tab` 补**光标所在的那个词**——首位补命令名、非首位在 `=` 左边补参数名（详见本行末「参数名补全」）、带缩进的 `/` 行也认、词从光标处整体替换、`/` 词来自会话+窗口命令表且去重、g-code 模式按 ASCII 大小写不敏感匹配并插入打印机拼写、唯一命中直接补、多候选补到公共前缀并开层且**公共前缀不得比已输入更短**、`Tab`/`BackTab` 循环含环绕、其它键先收层再照常生效（`Backspace` 收层后仍删一个字符，`Esc`/`Enter`/`^C` 等不被吞）、`objects/query` 的 `gcode.commands`（全部命令名，含 `M115`/`M110`/`ECHO` 这类无描述的）问到答案为止（被拒不算答案，按 `Tab` 会重试）、空行按 `Tab` 列出全部候选、无匹配写一行提示（本地词与 g-code 词分别措辞）、纯空白行仍静默、候选层画在输入行上方并遮盖日志、空间不足时不画、长列表跟随选中项滚动、footer 保持在 100 列内）；**参数名补全**（`=` 左边才算、值里不补、替换止于 `=` 所以循环不动值、非首位空词列出该命令全部参数名、打印机 `parameters` 优先、没给该命令时用签入的内建表、整次拿不到与「主机一条参数都没报」两类共用同一个已提示标志（最多一条、只在补参数名时发）、部分缺参数静默、小写命令名也命中主机自己的参数）；按宽度折行：同一条目自己换行的续行**跟在后面**（否则日志从底往上收集会把一个条目的行反过来）、多行条目保持行序、新条目在底部；长状态消息的状态行会折行（不再被截掉） |

窗口本身（raw mode、备用屏幕、按键线程、`select!` 主循环）没有自动化测试：那需要真终端。补全那部分是例外：键位仲裁（`completion_key`）、候选匹配与候选层渲染都有单测，只有“`handle_key` → `input.take()` 之间候选层先收起”这一步靠代码路径而非用例担保。验证方式是在 tmux 里跑一遍（`tmux new-session -d … 'klippy-client console …'` + `capture-pane`），杀掉主机看它是否带着错误退出并把终端还回去，以及在 `klipperx --tui`（两种写法都试）下按 `^C` 看主机是否随窗口一起退出、socket 文件是否被清掉。

主机端到端（装载 `[mcu]` → 起服务 → 查询）也没有自动化测试：集成测试要一个开了 `CONFIG_HOST_FRAME_API` 的 host 库，而 `crates/test-support` 建的是逐字节那份（非 `cfg(test)` 编译要前者）。手工验证方式：写一个只有 `[mcu]` 的配置（`host_library:` 指向一份开了该选项的库），起 `klippy`，然后 `klipperx api objects/list` 应为 `["webhooks", "mcu"]`，`objects/query '{"objects": {"mcu": null}}'` 应为 identify 快照（`mcu_version` / `mcu_build_versions` / `mcu_constants`），`objects/query webhooks` 应为 `ready`。接口不可用时同一条链应变成 `state: shutdown` 且 `state_message` 里带对象名与原因。

`klipperx` 侧的日志合流（`src/logging.rs`）有单测：装一个线程内的 subscriber，断言字面量消息与带参数消息两条路径都能到达窗口、等级正确、顺序正确，并且 `WindowGuard` 落下之后不再复制。窗口槽是进程级的，所以这两个用例用一把互斥锁串起来跑。

### 帧与字节流

| 模块 | 覆盖 |
|------|------|
| `frame.rs` | `Frame` 编解码与 CRC/SYNC/序号校验；`FrameStream` 的分包重组（整帧未到不吐帧、一次读里多帧、读边界落在帧中间）、乱码后按下一个 SYNC 重新同步、CRC 损坏帧被跳过且不影响其后的帧、整段无 SYNC 时保持失步 |

**所有字节流设备共用 `frame::FrameStream`**：它把「一段字节里哪儿是帧」这件事收在一处。写新设备（串口、socket 之类）时不要自己再实现一遍同步逻辑。

### `extras`

| 模块 | 覆盖 |
|------|------|
| `output_pin.rs` | `value` / `shutdown_value` 落到 `setup_start_value`，且无条件 `setup_max_duration(0)`（所以 `value: 1` + 默认 `shutdown_value: 0` 合法）；`SET_PIN PIN=… VALUE=…` 驱动输出（`>=0.5` 为开）并更新 `get_status`；缺 `VALUE` 报错；两个 pin 各自独立；缺 `pin` / 非数字 `value` / 非布尔 `pwm` 各自报配置错误；`pwm: true` 走 `setup_pwm` 并把 `cycle_time` / `hardware_pwm` / `value` 落到资源，`SET_PIN` 调 `update_pwm`；`cycle_time <= 0` 报错；**按打印时间排队（新增 7 测）**：带 clock 的 `queue_digital_out` 且 `update_*` 零调用（`test_a_queued_set_pin_sends_a_clocked_change`）、两帧 clock 差 == 两 `print_time` 之差 × 频率、同值重复 `SET_PIN` 不发第二帧（sink `Discard`）、无 toolhead 兑底立即、资源不可调度（`min_schedule_time()` 为 `None`）兑底立即、0.5 在排队路径仍当开、软 PWM 对齐到周期边界（`test_a_queued_pwm_change_is_aligned_to_its_cycle`） |
| `bltouch.rs` | 选项默认与语料全量认领；缺 `sensor_pin`/坏 `set_output_mode`/未知 chip/控制脚带上拉的上游文案；占空比=命令宽/信号周期；`Commands` 序列与 `BLTOUCH_STORE` 下发序列逐字节断言；装载注册 `probe` chip 与 `probe` 对象；虚拟 chip 的其它 pin 名与 `!`/`^` 拒绝；同步更新工厂表顺序断言 |
| `board_pins.rs` | `aliases` 与 `aliases_*` 都注册；`mcu` 列表指定目标 chip；`<...>` 值走保留；未知 chip、缺元素、别名冲突各自报错（冲突带 section 前缀）；对象不可查询 |
| `heaters.rs` | 传感器工厂表：未知 `sensor_type` 报上游文案 `Unknown temperature sensor 'x'`；`register_sensor` 把 section 名计入 `available_sensors`；`ensure` 幂等，并把 `DS18B20` 工厂带进来（对应上游 `temperature_sensors.cfg`）；`get_status` 的三个列表；`lookup_heater` 逐字 `Unknown heater 'nope'`、`get_temp`、空节 `[verify_heater heater_bed]` 经真 loader 装载、停滞床的端到端 shutdown 文案（批 #20 共 +4 测）；`TEMPERATURE_WAIT`（2026-10-03 +10 测）：注册/help/参数表、未知 sensor 的 mux 文案、缺 `SENSOR`、缺两参、`MAXIMUM≤MINIMUM`、有注册无对象、闭边界触发与零轮等待、暂停时钟两轮 `T:0` 后触发、运行中 shutdown 收口、fileoutput 早退；`set_temperature(.., wait)` 与 `HeaterControl::set_control`（清零 target）由 `pid_calibrate` 的 12 测覆盖（2026-10-03） |
| `pid_calibrate.rs`（12 测） | ZN 数学钉死值、确定性假 heater 收敛、双相摆动/`TUNE_PID_DELTA`、dump 格式、命令注册与参数声明、缺 `HEATER`/未知 heater/`TARGET` 越界三错、中断+控制环恢复、`WRITE_FILE` 先于中断判定、暂停时钟全流程（respond_info 三行 + `save_config_pending_items` 四条 + 收尾 target=0） |
| `extras/verify_heater.rs`（批 #20，8 测） | 四个选项默认值与 60/20 分档、四条越界文案逐字、`target<=0` 与滞后内永不故障、停滞加热器在 t=61 触发且 shutdown 文案（含 `HINT_THERMAL`）逐字、`heating_gain` 爬升期 `error=0`、fileoutput 不建定时器、无同名 heater 不建定时器 |
| `extras/idle_timeout.rs`（批 #21，11 测） | `get_status` 三键与初值、`SET_IDLE_TIMEOUT` 回文/默认值/越界、默认模板渲染逐字（含与上游 Jinja2 的空行差异；渲染断言自 `0960f44` 起带解析器保留的前导空行 `"\nM84"`）、`ready`/`idle` 载荷、Ready→Idle 迁移与重臂（`ManualReactor` 驱动）、`filament_motion_sensor` 对载荷的消费 |
| `temperature_sensor.rs` | **本文件无独立测试**：其行为（`min_temp` 默认 `KELVIN_TO_CELSIUS`、`max_temp` 须高于 min、`sensor_type` 交给 `heaters`、`get_status` 的 `round(…, 2)` 与读数 0 不计入 min/max）目前只经 `load.rs` 的工厂表与上游语料的端到端用例间接碰到；`setup_minmax`/`setup_callback` 的落点由 `heaters.rs` 与各传感器工厂的测试覆盖 |
| `ds18b20.rs` | `serial_no` → 小写 hex、`ds18_report_time`（≥ `DS18_MIN_REPORT_TIME`）、`sensor_mcu` 找 MCU 并领 oid；build 加 `config_ds18b20` 与 `query_ds18b20`（init）；post-init 按 oid 绑定 `ds18b20_result`（每 MCU 一个 registry），fault 丢弃，`next_clock - report_clock` 映射回 print time |
| `extruder.rs` | `[extruder]` 装载并注册命令；编号兄弟（`extruder1`…）经主节连带读取；`SET_PRESSURE_ADVANCE` 不给 `EXTRUDER=` 时命中默认项并转给活动挤出机、给了不存在的名字报可选项列表；挤出检查按上游（`max_extrude_*` 越界拒绝）；拐角用 `instantaneous_corner_velocity` |
| `heater_bed.rs` | `[heater_bed]` 装载并注册 `M140`；`M140` 设目标、`M190` 也设目标（不等温，与文档一致），`get_status` 的 `temperature` 是数 |
| `heater_generic.rs` | **无独立测试**：工厂在装载表中（`load.rs`），加热器选项与控制环由 `heaters.rs` 的测试覆盖 |
| `fan.rs` | 上游默认值装载、`shutdown_speed` 被 `max_power` 截顶；`M106` 设速 / `M107` 关、负值拒绝；kick-start 满速后回落、新请求覆盖挂起的 kick；`off_below` 把小请求归零；`max_power` 截顶；`enable_pin` 只在 0→非 0 翻转；`gcode:request_restart` 停风；**`tachometer_pin` 接通**（批 #8：`rpm: 0.0` 报数、`tachometer_ppr<1`/`poll_interval≤0` 拒、`^` 允许 `!` 拒绝，fan 24 测；2026-10-03 +6：`test_m106_is_queued_at_the_lookahead_print_time`、`test_a_later_queued_request_overrides_the_earlier_one`、`test_the_kick_start_tail_is_a_queue_rerun`、`test_without_a_toolhead_m106_drives_the_pin_immediately`、`test_a_fan_that_cannot_schedule_drives_immediately`、`test_a_restart_request_stops_the_fan_through_the_queue`）；缺 `pin` 点名、越界报哪一边、坏数字报原文 |
| `pulse_counter.rs`（批 #8，10 测） | 命令过字典编解码往返、config 装载（pin=32/pull_up=1、init 留空）、`arm_query` 槽位（poll/sample tick 与 clock 估算）、`counter_state` 按 oid 路由与 clock→print time 映射、32 位进位、每份报告都回调（首份不吞）、首样本只锚定、频率=Δcount/Δtime、无新时间读 0 |
| generic_cartesian/idex/trsync（批 #10/#12/#27，11 测） | `the_corexyuv_config_homes_against_the_fake_firmware`、`a_generic_cartesian_z_home_fires_the_trsync`（超时即失败的有界归零）、`the_corexyuv_case_runs_every_gcode_line`（整例 27 行逐行）、`the_frames_follow_the_homed_carriages_to_their_endstops`（帧随归零到 endstop）、`the_generic_cartesian_kinematics_registers_the_idex_commands`；批 #27 新增（6 测）：`a_dual_carriage_without_a_primary_is_a_primary_of_its_axis`、`a_dual_carriage_with_neither_an_axis_nor_a_primary_is_refused`、`a_dual_carriage_naming_an_unknown_primary_is_refused`、`a_dual_carriage_names_its_primary_and_keeps_its_safe_distance`、`two_primaries_on_one_axis_each_keep_their_own_dual`、`two_dual_carriages_on_one_primary_are_refused` |
| `gcode_move.rs` | `G1` 解析轴与速度、未点名的轴不动、记住上一笔速度、非正进给拒绝；G90/G91 切换、G92 锚定后下一笔在界内、裸 `G92` 全零；M83 的 E 相对而轴绝对；M220/M221 缩放速度与挤出；锚定只重挂 homed 的轴；SAVE/RESTORE 状态（未存的名字报错）；`M114` 报 G-Code 位置；`get_status` 对齐上游；第二个坐标系不能默默夺槽；英寸制拒绝 |
| `toolhead.rs` | `[printer]` 的轴索引与 move 上下文；不支持的 `kinematics` 报配置错、`none` 不要 stepper；`stepper_z1` 并入 Z rail；corexy 族装载建 rail；MCU 错误带节名；限值来自 `[printer]`；move 到规划器、未 homed 轴拒绝；`G4` 推进 print time；`SET_KINEMATIC_POSITION` 回零并清状态；探针式回零 `probing_move`：滴满整段（事件顺序 `homing_move_begin` 先于 `home_start`）、无触发报 `No trigger on probe after full movement`、零位移与亚纳米（<1e-9，与 `Move::new` 同口径）返回当前位置且不 arm endstop（`test_probing_move_zero_distance…`、`…sub_nanometer_distance_returns_without_arming`）；`flush_step_generation`：入队的 move 冲刷后 commanded/history/print_time 可见、`set_position` 先冲刷再改位并标 homing 轴与恰发一次 `toolhead:set_position` 事件、`z_stepper_names` 按 Z 轨配置序（`kinematics: none` 为空）；停止后把指令位置设到**移动终点**并返回它——滴满整段模拟上游 file-output 的 `wait_end` 后 complete（同上游 file-output 口径：`trigger_analog.py:409` 的 `trigger_time = home_end_time` / `mcu.py:325 wait_end(end_time)`）；**预瞻/flush 两个注册口**（connect-safe）：未连接注册→connect 后安装并触发、连接后空 lookahead 立即带 `get_last_move_time()` 触发、有 move 时带该 move 末尾时间触发（`test_callbacks_registered_before_connect_fire_after_connect`、`test_a_lookahead_callback_registered_after_connect_fires_with_the_last_move_time`、`test_a_lookahead_callback_registered_with_a_queued_move_fires_at_its_end_time`） |
| `stepper.rs` | 节名→轴、轴索引与 mathutil 一致；步距按几何算；节装成 stepper 对象；`endstop_pin` 建 rail 的 endstop 与 `HomingInfo`；endstop 居中推不出方向时报错；`gear_ratio` 除进步距；缺 pin 点名节、不同 MCU 的同轴引脚被拒、`position_endstop` 越界被拒 |
| `stepper_enable.rs` | 节装载；无 `enable_pin` 时是“永远使能”；写了则建使能脚（共享/取反路径） |
| `bed_mesh.rs` | 语料选项全量认领（含 `faulty_region_*` 对）；矩形网格按行 zigzag、间距下取整到百分位；圆床按 `mesh_radius` 过滤且用 `round_probe_count`；过近点报 `bed_mesh: min/max points too close together`；批 #11（13 测）：`probe_count`/`mesh_pps` 单值与非法值文案逐字对齐（`malformed`/`Unable to parse`/`minimum of 3`）、round 床 `mesh_min` 缺省与边界推导、`invalid min/max points` |
| `bed_tilt.rs` | 选项默认 0；无 `points` 不注册命令、`points` <3 点报上游 `Need at least 3 probe points`；平面拟合恢复已知平面（含探针偏移修正）且 configfile pending 为 `%.6f`；`get_position` 减 / `move_to` 加回的往返一致；`update_adjust` 重锚坐标并记三项 pending |
| `z_tilt.rs` | `z_positions` 项数/缺项/坏项的上游文案、至少 2 点；`RetryHelper` 范围文案与上限、上升即中止、`error_msg_extra` 追加、无重试则静默；`applied` 标志与 motor_off 复位；平面拟合恢复已知平面；`adjust_steppers` 按 `-a` 排序逐步挂回的顺序录音 + 失败后全部挂回 |
| `quad_gantry_level.rs` | `linefit` 直线与斜率（含退化）；四角高度恢复已知点；超 `max_adjust` 中止文案；恰好 4 点、`gantry_corners` >=2、缺项上游文案 |
| `screws_tilt_adjust.rs` | screwN 数到首个缺失即停与默认名 `screw at %.3f,%.3f`、螺丝<3 报错、`screw_thread` 8 项选择表与默认 `CW-M3`、`threads_factor` 换算、方向表与 `HH:MM`（含 0.001 阈值）、基准螺丝（第 1 颗 / `DIRECTION` 极值）、`MAX_DEVIATION` 延迟报错与 `DIRECTION` 非法文案、`get_status` 的 error/max_deviation/results 形状 |
| `extras/axis_twist_compensation.rs`（批 #36，11 测） | 选项默认值与越界、插值（含端点外推语义）、`z_compensations` 长度校验、`probe:update_results` 载荷原地改 Z（probe/eddy 两处上报读回）、`AXIS_TWIST_COMPENSATION_CALIBRATE` 注册与首点派发 |
| `extras/gcode_button.rs`（批 #37，8 测） | 语料节选项全读、按下/释放各渲染派发、空 `release_gcode` 不跑、`QUERY_BUTTON` 与 `get_status`、缺 `pin`/`press_gcode` 文案、`debounce_delay` 下界、`analog_range` 读取后的明确拒绝与上游解析/越界文案 |
| `gcode_arcs.rs` | `resolution` 默认 `1.` 记账、显式值解析、`0`/`-1`/非数拒绝文案对上游、经 loader 认领并注册（4 测） |
| `bed_screws.rs` | 全选项 `check_unused` 直证、行进默认 50/5/5/0 与默认名、螺丝缺失即停（access 无残留）、<3 与两元素/解析/fine_adjust 上游文案、above 0 边界、静止态 status（6 测） |
| `pwm_cycle_time.rs` | 选项矩阵与默认、`SET_PIN` 值域与 `CYCLE_TIME`、重复值丢弃、无 `hardware_pwm` 恒软件路径（11 测） |
| `pwm_tool.rs` | 选项矩阵与默认、prefix 命名错误、`cycle_time` 值域、`maximum_mcu_duration` minval 0.5、配对约束 build 期拒绝（15 测；2026-10-03 +4 队列化：前瞻钉时→flush 落 clocked、后请求覆盖、两条兕底、同构对接） |
| `temperature_fan.rs` | 6 实例选项矩阵、`max_temp<40` 取目标、pid 选项与上下界、bang-bang 驱动、`SET_TEMPERATURE_FAN_TARGET` 命令与三段错误文案、未知 sensor/非法 control、缺 pin 前缀名（11 测） |
| `temperature_host.rs` | `sensor_path` 默认/显式解析、开文件错误逐字 `Unable to open temperature file '<path>'`、对象名取末词、临时文件假读数驱动 poll（首采 42.5 → 改写文件 + `advance(1.0)` 恰一次触发 → 43.1 与 `get_status` 形状、回调序列）、读失败置 0 并退役定时器、min/max 越界 shutdown 文案双侧、裸节全链路装载（工厂注册 + 对象 + 不可 query + 消费者 `get_status` 形状）、裸节拒收选项（8 测） |
| `temperature_probe.rs`（40 测，`f6ea201` + `dfaf418` + `704ee0d` + `0960f44`） | A：`Polynomial2d` 求值/拟合恢复系数/无点无解/上游格式化；选项默认值与 9 条边界措辞、`calibration_position` 的 count 措辞、check_unused 全选项记账；读数平滑同上游、`get_status` 六键形状、`stats` 行、装载按节名注册对象。B（+14）：命令注册/help/`homed` 门（走真实 mux 分发）、无 probe、链接不符、手动探针冲突、TARGET/STEP/样本数文案、临时命令被占、启动+手动探针会话、升温 kick 下一轮、COMPLETE 短样 abort / ≥3 静默完成、ABORT 收尾、初始移动失败回滚、热膨胀累计与清零、加热脚本逐字、ENABLE no-op（C 起接真）。C（+13）：四选项默认与三类拒绝文案、`_check_calibration` 交叉消息、adjust/unadjust 三分支+两闸门+往返+传感器温度、`finish` 无 run / 九段 fit+configfile 写回+交叉拒绝、ENABLE 真开关三态、collect_sample 无/有 helper 分支、状态机 start/finish 接线、SweepState 窗口记账与聚合、接线闸门三态。回环（+1，`0960f44`）：`finish_calibrations_curves_survive_a_save_config_round_trip`。**注**：测试机 `kinematics: none` 恒 unhomed → `_check_homed` 成功分支与门后 `cmd_calibrate` 端到端归真机 |
| `probe_eddy_current.rs`（5 测，`704ee0d`，本文件首现） | 无注册时原表行为不变、`apply_calibration` 过 adjust、`height_to_freq` 过 unadjust、覆盖式注册、越界哨兵不被挪动 |
| `controller_fan.rs` | 默认值、全选项+覆盖+跟踪器矩阵、缺 pin 前缀名、未知 stepper/heater 上游文案（connect 解析）、运行→怠速→停 tick 状态机（6 测） |
| `gcode_macro.rs` | 真语料八实例全选项入 access、八个大写命令+帮助、变量 status、畸形节上游文案、`rename_existing` 类型检查与延迟注册、裸节认领、宏体渲染→派发 e2e、递归防护、`SET_GCODE_VARIABLE`、未知语句装载报错（9 测；未知语句的 fixture 用 `{% foo %}`——`{% block %}` 现由引擎解析，与 Jinja2 一致）；**宏把参数名带进 `status.gcode.commands`**：扫模板体里的 `params.NAME` / `params['NAME']` / `params["NAME"]`（去重、首次出现序），扫不到则退回 `variable_*` 名，两者都无则不带 `parameters` 键（4 条测试）；**装载期静态命令存在性检查**：`strip_template_tags` 把 `{% %}`/`{# #}`/`{ }` 标签内容置空（保留换行与字面文本，字符串/花括号深度不误闭），`static_command_names` 抽出每行字面首词（去重首次序），`klippy:ready` 时逐个查 `GCodeDispatch::command_exists`，未注册者 `respond_info` 告警（不拒绝），动态算出的命令（`{{ cmd }}`）不抽取故不告警（5 测） |
| `gcode_request_queue.rs`（12 测） | 上游 `output_pin.py:40-90` 的队列语义逐条：覆盖压缩只发盖过的那条、`min_schedule_time` 的间隔与对齐、`must_flush_time` 未到整队不出、`discard`/`reschedule`/`repeat` 三分支各自对前缀与 floor 的处置、乱序 push 按到达序覆盖、`send_async_request` 直通及其三种 action、**双 push 线程 + 单 flush 线程的并发冒烟**（无死锁、队列排空、每值不早于自身时刻发出） |
| `led.rs` | 六段选项矩阵与上游文案（`No LED pin definitions found`/`color_order does not match chain_count`/`neopixel chain too long`/同 mcu 约束）、`LEDHelper` 颜色记账与初始值边界、`lookup_display_templates` 惰性单例（`is_queryable=false`）等（21 测） |
| `extruder_stepper.rs` | 段全选项与步距 28.2/(200·16)、默认 PA `0.`/`0.040` 与越界文案、绑定校验逐字 `'bogus' is not a valid extruder.`、按名挂值与主挤出机槽位隔离、命令值域（7 测） |
| `exclude_object.rs` | 零选项段+reset、get_status 三键、START/END 跟踪（大写化/隐式 define）、EXCLUDE 按名/当前/RESET+排序、`There is no current object to cancel`、DEFINE CENTER/POLYGON JSON 与 RESET、排除区丢弃/区外转发、四命令 help（9 测） |
| `virtual_sdcard.rs` | 选项矩阵与默认 `on_error_gcode`、显式值、缺 `path` 文案；`get_file_list`（顶层 + 递归 + 缺目录）；`M20`/`M21`/`M23`（打开/不存在/大小写不敏感/去前导 `/`）、`M26`、`M27`、`SDCARD_RESET_FILE`/`SDCARD_PRINT_FILE`（子目录）、`M28`–`M30`（`cmd_error`）、`do_cancel`、`get_status` 两态（30 测）；回放循环：EOF 完成 + `progress`=1.0 + `Done printing file`、`is_active` 随 task、`M25` 暂停 + `note_pause`、错误触发 `note_error` + `on_error_gcode` 渲染运行、`do_resume` 活动时拒 `SD busy` |
| `print_stats.rs` | reset 初值、`set_current_file`、`note_start`→`note_pause`→`note_complete` 流转与 duration、`note_error`/`note_cancel`/`note_pause` 不覆盖 error、`SET_PRINT_STATS_INFO` layer 逻辑（0 清空/切换 total 重置/截断/无参保持）、`get_status` standby/printing/paused 三态、命令注册、`ensure` 单例、filament 随 E 累积（19 测） |
| `display_status.rs` | 裸段+`check_unused`+状态形状（1 测） |
| `homing_override.rs` | 语料选项矩阵、默认 `XYZ`/无强制位、parse 与 `must be specified` 文案、G28 语句轴掩码=上游 `cmd_G28:33-46`（4 测） |
| `sdcard_loop.rs` | 裸段、sd 内外 BEGIN/END、count 0/1/>1 索引、空栈+嵌套、DESIST（5 测） |
| `servo.rs` | 段选项全读+`SET_SERVO` 注册、脉宽↔占空比公式、mux ANGLE/WIDTH/缺参、`maximum_pulse_width` 下界措辞、`initial_angle`（11 测；2026-10-03 +6 队列化：前瞻钉时落 clocked `set_pwm`、后请求覆盖、两条兕底、sink `reschedule`/`discard`） |
| `idex_modes.rs` | cartesian 认领（主轨 stepper_x+初始 0）、corexy 不认领、对象三命令与 SAVE→RESTORE 还原、四类上游措辞、`safe_distance` 下界、双挤出超槽守卫双侧语义（6 测）；批 #4 增：轨间坐标交接（SET/RESTORE 携 gcode 坐标到目标帧） |
| `config/mod.rs`（同名段合并，M5d） | 重复段选项并集、同段重复选项后者胜、合并保首现位、非重复段零变化（4 测）；多行值回环（2026-10-03 `0960f44` +2）：`a_multiline_block_value_is_written_with_indented_continuations`、`a_multiline_block_value_round_trips_verbatim`（写 → 剥 `#*# ` → 重解析逐字还原）；`an_equals_option_may_continue_from_an_empty_value` 按 configparser 语义重钉（空首行保留 `['', …]`） |
| `interface/devices/simulator.rs`（M5d 策略 b） | `trigger_analog_sample_activity_pushes_the_monitor_deadline`：活动顺延 + 非活动不顺延双向断言 |
| `mathutil.rs`（M5d 校准数学） | `gaussian_solve_recovers_known_values`、`gaussian_solve_refuses_a_singular_system`、`solve_linear_equations_fits_a_quadratic_and_substitutes_back`、`mat_mul_transp_matches_the_reference_product`（4 测；另修 `mat_mul_transp` 参照积笔误 a·aᵀ） |
| `template.rs`（minijinja 适配层，21 测）+ `gcode_macro.rs` 宏体 e2e（4）+ 排除区 E 补偿（2）+ idex 帧交接（1），共 28 测 | 引擎侧 21：Python 拼写（`True/False/None/3.0/0.0`）、`if/elif/else` 与 `for range`、`in`/`not in`/`is (not) defined`、`set` 三态作用域 + 语料 288 行真句、过滤器形态（`default`/`float`/`int` 含 Jinja2 可选默认参与关键字形态）、`min`/`max` 大小写不敏感与四类报错（非序列/跨类型/空序列/超参）、列表字面量、`rawparams`、M300 与 iqex/itex 语料整句、printer 状态数组的 `.x/.y/.z/.e`（域外数组仍是普通列表）、`'name' in printer`、`action_respond_info`、装载/求值两相位的上游错误帧与行号 **1 基**、三个步进宏字面用例（含 `namespace(phase=0)` + `{% set count.phase %}`）编译+渲染双向（DIR 正反转、`G4 P` 值；宏体不带 `#` 注释行——config 解析器在值到达引擎前已剥掉）；宏体渲染→gcode 派发、递归防护、`SET_GCODE_VARIABLE`、未知语句 `{% foo %}` 装载报错（gcode_macro 4）；排除区 E 补偿 2、idex 帧交接 1。语料回归 `upstream` 160 绿为最终仲裁 |
| `extras/dac084s085.rs`（批 #23，8 测） | 语料字节逐字 `[0x19,0x90]`/`[0x59,0x90]`/`[0x99,0x90]`/`[0xd6,0x60]`、**截断**语义（`*255/scale` 无 `+0.5`）、`scale`/通道越界、缺 `enable_pin`、通道缺省不写、SPI mode 1 与 10 MHz 默认、`section!` 名大小写（小写会报「not a valid config section」） |
| `motion/kinematics.rs`（polar）+ `extras/stepper.rs`（两轨段）+ `toolhead.rs` polar 分支（批 #5，17 测） | 已知构型正/逆回代、±π 解卷与单次移位边界、`check_move` 门与中心减速、两轨段认领与选项矩阵、G28 联合回零（XY 后 Z）、`Error loading kinematics` 文案 |
| `motion/rotary_delta.rs` + 工具头分支 + `extras/delta_calibrate.rs` 的 `KinematicsCalibration` 分派（批 #40，10 测） | 三肩逆解与 `rotary_delta_position_fn` 对拍、`rotary_two_arm_calc`、`home`/`unified_home`、锥形圆柱 `check_move`、`get_status`；`KinematicsCalibration` 枚举让 `[delta_calibrate]` 同时驱动线性/旋转两条校准路径 |
| `motion/delta.rs` + `extras/delta_calibrate.rs` + delta 段/工具头分支（批 #5，24 测） | 三角测量已知构型回代、同步 home、三塔段认领（无 position_max、b/c 继承 a 的 endstop）、SAVE_CONFIG 块解析（header 逐字节/剥前缀/正文优先/无块零变化）、假 MCU 多端停 per-oid 多槽（同 arm 同触发/单端停回归/按 oid 摘除）、弧度 gear_ratio 推断 |
| `extras/shaper_defs.rs` + `extras/input_shaper.rs` + `mathutil.rs::pseudo_inverse`（wave-2，13 测） | 整形系数与上游 Python 逐位对齐（`mzv(5,0.6)`/`2hump_ei`/`ei(v_tol=)`/`zv`/`zvd`/`3hump_ei` 金值）、括号参数与 `get_shaper_cfg` 元数据、错误路径（`Too small n=…`/`Too large t=…`/`Unsupported arguments…`）、两条语料行的上报文案（x→y→z）、`dual_carriage` 的 connect 期 config_error 与运行期允许 |
| `extras/adxl345.rs` + `cmd/adxl345.rs`（wave-2，9 测） | `rate` 默认 3200 与非法值文案、`axes_map: -x,-y,z` → `[(0,-1),(1,-1),(2,1)]`、mux 端点 `adxl345/dump_adxl345`（key `sensor`）、带名节 identifier 注册、`cmd` 命令名/参数与语料字典一致 |
| `extras/mpu9250.rs` + `cmd/mpu9250.rs`（wave-2，19 测） | 真 `Printer::load_config` 下按 identifier 注册（`mpu9250 my_mpu`）、默认值 4000/0x68/400000、`mpu9250/dump_mpu9250` 的 value、`convert_samples`/`read_axes_map` 换算；bulk 数据通路显式登记为 gap |
| `extras/filament_switch_sensor.rs` + `extras/filament_motion_sensor.rs` + `buttons.rs`（wave-2） | 选项默认值与下界（`pause_on_runout` 默认 True、`pause_delay` 0.5、`event_delay` 3.0、`detection_length` 7.0）、`extruder` 值按段标识取对象、两条 mux 命令注册；`SYNC_EXTRUDER_MOTION` 空值解绑/非挤出机名文案、`SET_EXTRUDER_ROTATION_DISTANCE` 的 0 与负值分支（`extruders.test` 转绿即其验收） |
| `extras/pause_resume.rs`（批 #15，11 测） | `[pause_resume]` 节装载与 `recover_velocity` 默认/越界、四条命令注册与 help 逐字、`PAUSE`/`RESUME`/`CLEAR_PAUSE`/`CANCEL_PRINT` 状态机四种文案、`CANCEL_PRINT` 两分支、`get_status` 的 `is_paused` |
| `extras/heater_fan.rs`（批 #6，8 测） | 默认值（heater=extruder / heater_temp=50 / fan_speed=1）、越界拒绝、关机 PWM=1.0、tick 四反例（含「速度未变不重复写 PWM」）、未知 heater 文案、`check_unused`、整份配置装载 |
| `extras/fan_generic.rs`（批 #18，6 测） | `shutdown_speed` 默认 0.0（非 heater_fan 的 1.0）、整份最小配置装载与对象名/`get_status`、`SET_FAN_SPEED SPEED=` 写 PWM（负值 `minimum of 0`、`SPEED=2` 被 `max_power` 封顶）、`SPEED`/`TEMPLATE` 互斥逐字、`TEMPLATE` 拒绝文案、缺 `pin` 前缀名 |
| `extras/safe_z_home.rs`（批 #6，17 测） | 选项默认与下界、`home_xy_position` 缺失/单值拒绝、z 端停两分支（`stepper_z` / `carriage axis=z`）、与 `homing_override` 互斥、`G28 Z` 未归零 X/Y 文案、G28 合成语句 params、hop 顺序反例（`set_position` → `move` → `clear_homing_state`）、`z_hop=0` 跳过、`move_to_previous`、**装载顺序钉住**（注册后原 G28 handler 为 `Some`） |
| `extras/manual_stepper.rs` + `extras/force_move.rs`（批 #6，11 测） | SPEED/ACCEL 缺省取节值、`Move out of range`、`calc_move_time` 与上游 Python 逐位相等、GCODE_AXIS 四条（先 upper 再校验）、无 endstop 文案；`manual_stepper.test` 已按此转绿 |
| `extras/display/{display,st7920,hd44780,hd44780_spi,uc1701,ssd1306,aip31068_spi}.rs` + vendored `display.cfg`（批 #7+#13+#28+#30，**115 测**） | 节装载与选项全读、按需创建 `display_status`、`lcd_type`/`display_group`/glyph/模板参数四条文案、异 mcu 拒绝、init 字节序列与首帧真发送（防假绿，四驱动各自钉）、4-bit 引脚校验与合批规则、不支持变体拒收、`display.cfg` 漂移守卫（需 `KLIPPERX_KLIPPER_DIR`） |
| `cmd/hx71x.rs` + `extras/{hx71x,load_cell}.rs`（批 #14，24 测） | 节选项与校验文案逐字（`must be specified`/`Choice …`/同 MCU/`must have minimum of 1.0`）、config/query 按 atmega2560+hc32f460 双字典编码、`"<i"` 小端解码与 `samples_per_block==12`、HX711/HX717 分型默认值、`dump_force` 四列表头与 counts→grams；`DumpForceEndpoint::detach` 清该 cell 的推送客户端并置 detached（竞态中的请求回 `UnknownMuxValue`）、cell 已消失时不 panic（`test_dump_force_endpoint_detach_clears_the_cells_clients`），已 detached 的实例被请求时回 `UnknownMuxValue`（`test_a_detached_dump_force_endpoint_answers_unknown_mux_value`） |
| `extras/tmc.rs` + `extras/tmc_uart.rs` + `extras/tmc_spi.rs` + `extras/tmc2208.rs`（批 #39 补测试模块：`DUMP_TMC` 的 IOIN 字段转写）+ `extras/tmc2209.rs` + `extras/tmc2130.rs`（批 #7+#W0+#34，`--lib tmc` 41 测） | 寄存器字段表与 `set_config_field`、四条 mux 命令与 help 原文、`Unknown field/register name`、`Run Current: 0.70A`、fileoutput 总线短路（不发 `tmcuart_send`）、缺 `uart_pin` 与缺宿主 `[stepper_*]` 文案、`get_phase_offset()` 契约、echeck 不误 shutdown、虚拟端停绑定与缺 diag pin 拒绝 |
| `bulk_sensor.rs` | 51 字节/4 = 12 样本每块与固件消息尺寸一致；时钟回归一次 update 斜率精确恢复采样率并外推；切片与时间戳公式；16 位序号回绕与符号扩展；`apply_status` 跨回绕计数与 msg_count→chip 映射；超长 query 时长滤波只跳样本不污染时钟；批循环首客户端启动恰好一次、末客户端注销停循环（start_paused 异步）；`stop()` 标记 detached、清客户端、拒绝新客户端并跳过 stop 回调（`test_batch_helper_stop_detaches_the_clients_and_the_loop_finishes`）、`MuxBatchEndpoint::detach` 走 `stop()`（`test_batch_mux_endpoint_detach_stops_the_helpers_clients`）、已 detach 的实例被请求时回 `UnknownMuxValue`（`test_a_detached_mux_endpoint_answers_unknown_mux_value`） |
| `ldc1612.rs` | `sensor_div`/`freq_conv` 换算（含 raw↔Hz 往返）；`convert_samples` 各错误分支（固件编码错误丢样、under-range/watchdog 保留）与计数；`reg_drive_current` 提取含高位掩蔽；attach 钩子 init 命令绑定 M5a trigger_analog oid；`dump_ldc1612` 端点注册不重名、按 sensor 路由与客户端注销 |
| `manual_probe.rs` | 二分插入点（`bisect_left`）、空闲状态形状；交互路径（`TESTZ` 移动、`ACCEPT` 校验、`ABORT` 收尾、命令注销）由上游语料端到端覆盖 |
| `probe.rs` | `ProbePointsHelper`：`points` 的换行/逗号行解析、`move_target`（`use_xy_offsets` 减探针偏移）、越界点报错、`minimum_points`/`update_probe_points` 的上游文案；选项全量认领与默认值（`speed` 5.0、`samples` 1、`sample_retract_dist` 2.0、`samples_result` median、`samples_tolerance` 0.100、`deactivate_on_each_sample` true）；`lift_speed` 缺省回退 `speed`；`samples_result` 非法值报上游文案；虚拟端停校验：`z_virtual_endstop` 通过、其它 pin 名报 `Probe virtual endstop only useful as endstop pin`、`!`/`^` 报 `Can not pullup/invert probe virtual endstop`；归并算法：`average` 逐轴平均、`median` 按 Z 取中位（偶数样本取中间两者均值）；命令与会话路径由上游语料端到端覆盖；trait 化（M5b）：同一 `LiveRound` 分发点驱动真 z 实现与 stub 第二实现（调用序与互不串扰、session-mismatch、`Printer is not ready` 口径）、`SampleDelivery` 投递→会话收样 |
| `upstream.rs`（每例独立 runtime） | 语料驱动（字典、CONFIG/文件输出、SHOULD_FAIL）之外，另有 **6 条聚焦 E2E**：普通端停 `G28 Z`、`probe:z_virtual_endstop` 的 `G28 Z`、`G28 + PROBE`、`G28 + PROBE_CALIBRATE/TESTZ/ACCEPT`、`G28 + BED_MESH_CALIBRATE`（3×3）——把「端停/探针真的能驱动一次回零」钉在假 MCU 上 ；**`a_case_runtime_shuts_down`**：一个用例跑完后它的 runtime 必须在 `CASE_SHUTDOWN_TIMEOUT`(5s) 内收尾——泄漏部件（`Mcu` 不析构 → 阻塞读挂着）会超时，这正是「整套跑完却不退出」的探针 |
| `query_endstops.rs` | 全部限位读一遍并记住、取反的限位翻转电平、`M119` 逐个报（经虚拟字典帧解码） |
| `i2c_device.rs` | 硬件设备要地址、地址越界拒；注册两条调试命令；只给一个软件引脚报错、未知 MCU 点名节、软件引脚须同 MCU、就绪后 `get_status` 报地址与速度 |
| `smart_effector.rs` | 选项默认与语料认领；`ACCEL`/`RECOVERY_TIME` 负值拒绝、`SENSITIVITY`/`ACCEL` 越界拒绝（含上游文案）；`SET` 参数解析与默认、无 `control_pin` 时拒绝 `SENSITIVITY`；位流逐字节成帧；`control_pin` 预留与二次使用拒绝；装载注册 chip/对象（async：加载+bring_up+`G28`+两命令端到端）；无 `control_pin` 时 `RESET` 不注册 |
| `spi_device.rs` | 注册两条调试命令；无片选允许；`spi_mode`/`spi_speed` 越界拒、部分软件引脚报错、片选须在指定 MCU、未知 MCU 点名节；就绪后 `get_status` 报配置；软件设备接受本 MCU 引脚 |
| `static_digital_output.rs` | 每个引脚都被预留、取反的引脚有记录、缺 `pins` 报错 |
| `adc_temperature.rs` | 线性插值正反向、热敏电阻 Steinhart-Hart 与 Beta 模型（与上游公式对拍）；自定义 `[thermistor]` 与 `[adc_temperature]` 定义写在消费者前/后两种布局都能装载（`phase = early` 保证顺序无关；批 #22 与 `372cd93` 各 +1 测） |
| `extras/adc_scaled.rs`（批 #19，15 测） | 四个文案逐字（`must be specified`/`must be above 0`/`vref and vssa must be on same mcu`/`adc_scaled only supports adc pins`）、装载顺序（缺节时 `sensor_pin: Unknown pin chip name 'vref_scaled'`，有的则通）、参考采样 `(0.300,0.001,8,1,0.,1.,0)`、每 `sensor_pin` 各建一个内层 ADC、换算方向与 `calc_smooth=min(Δt/smooth_time,1)`、只取末样本、参考未就绪时的 0/0（NaN/±inf，照上游复现） |
| `extras/multi_pin.rs`（批 #26，10 测） | 两个 `[multi_pin]` 节都能装载（`DuplicateChip` 吞掉）、`set_pwm`/`update_pwm`/`setup_cycle_time`/`setup_start_value`/`setup_max_duration` 逐子 pin 转发、`next_aligned_clock` 原样、`multi_pin <name> not configured` 与 `Can't setup multi_pin <name> twice` 逐字、缺 `pins` 文案、`!` 反相前缀 |
| `extras/respond.rs`（批 #29，14 测） | `default_type` 三取值与 `default_prefix` 覆盖、非法 choice 文案；`M118` 原样透传（含引号/空格与大空白）；`RESPOND` 的 `TYPE` 四分支（含 `echo_no_space` 无空格）、`PREFIX` 覆盖、`MSG` 缺省为空、非法 TYPE 逐字文案；两条命令就绪前可用 |
| `extras/mcp4018.rs`（批 #33，17 测） | `i2c_address` 默认 `0x2f` 与显式覆盖（经 `i2c_set_bus` 的 address 断言）、`scale` 默认与越界、`wiper` 必填/上下界、`tap_value` 的 `int(v*127/scale+.5)`（含 0/顶值）、connect 首写字节、`SET_DIGIPOT` 命中/未给 `WIPER` 不写/越界、`wiper_response` 的 `%.2f` |
| `extras/homing_heaters.rs`（批 #31，6 测） | 归零开始把选中 heater 目标置 0、结束恢复原目标、未知 heater 名文案 `One or more of these heaters are unknown: ['x']`、`steppers` 分支、空节装载、`get_status == {}` |
| `extras/firmware_retraction.rs`（批 #32，13 测） | 四个选项默认值与 `minval` 文案、`G10`/`G11` 的 E 位移与进给（含重复 `G10` 的 no-op、`retract_length=0`）、`SET_RETRACTION` 覆盖与 `GET_RETRACTION` 逐字回复、`get_status` |
| `spi_temperature.rs` | MAX6675/MAX31855 转换、符号位负温、MAX31856 与 MAX31865 转换 |
| `extras/ad5206.rs`（批 #16，7 测） | `enable_pin` 作 CS 的 SPI mode 0 / 25 MHz 默认、`scale` 默认 1.0 与 `above=0.` 越界文案、`channel_1..6` 的 `minval=0.`/`maxval=scale` 文案、`int(val*256/scale+.5)` 换算（含 scale 顶值与未给通道跳过）、写入顺序与 bring-up 后经 post-init 回调发出 |
| `extras/mcp4451.rs`（批 #25，11 测） | smoothieboard/azteeg 两组语料字节序列逐字（含无条件两条 `0xff`）、`int(val*255/scale+.5)` 舍入、寄存器打包 `[(reg<<4)|high, value]`、未给 wiper 不写、`must be specified`/`must be above 0`/wiper `min`/`max`、`i2c_address` 的 0..127 与 44..47 两条文案、装载期写经 post-init 回调 spawn |
| `temperature_mcu.rs` | 单点直线、两点标定、手动标定读上游选项（`temperature_sensor` 节上的标定点） |
| `temperature_combined.rs` | 三种合并方式（`min`/`max`/`mean`）与舍入 |
| `bus_debug.rs` | `DATA=` 十六进制往返（`test_hex_round_trips`） |
| `error_mcu.rs` | 已知固件消息拿到它的提示、MCU shutdown 被扩成原因+提示、`is_shutdown` 说“此前已停”、无关停机仍告诉用户敲什么；连接错误拿到 firmware_restart 提示；协议错误列出需要升级的 MCU（6 例，无配置节、由第一个 `[mcu]` 拉起） |

### `interface`

| 模块 | 覆盖 |
|------|------|
| `canserial.rs` | **链路层全部单测**：节点号→仲裁 ID 的映射（`0x100+2n`，回包用 +1）、字节流按 8 字节切成 CAN 帧（含整除时不多出空帧）、按帧重组回消息块（最后一帧才成帧）、非本节点的帧被忽略、CAN 帧 ABI 布局（id/dlc/data 偏移与 16 字节大小）、节点指派报文与 Klipper 一致（`CMD_SET_NODEID` + UUID + nodeid）、打不开的 CAN 接口报错并带上名字。**socket 层没有测试**：本环境没有 CAN 接口，`vcan` 又需要特权加载，所以 `CanSerialDevice` 的 socket 部分只经过编译，未在真实总线上跑过（真实 `can0` 的验收需要一台有 CAN 的机器） |
| `serial.rs` | 用**虚拟串口**（`posix_openpt` 开的 pty 对）验证：`send` 写出的就是线上的整帧（raw 模式没有做任何转换）、`receive` 把分片的字节重新拼成帧、`shutdown` 让阻塞中的 `receive` 返回 `None`、打不开的端口报错并带上路径；另有一例走 `Interface` 的异步收发 |
| `simulator.rs` | 字典驱动的应答机（`test: dict=<file>`）：坏字典路径报错；对着它走**真实** `Mcu::connect`——identify 分块回 zlib 字典、装字典、块级 ack，再由 `get_clock` 经普通调用路径拿回响应（验证序号与发送窗口确实被推进）；**响应/回执盖 `seq+1`**（`command.c:305` 口径，与 `mcu/mod.rs settle` 的 `*seq < seen` 对齐——批 #10；`FrameMock` 仍同号，为第二套已知口径）；`trigger_analog_home` 假行为：arm 后首条 move 发 `trigger_reason=1` 一次且不复发、监控窗到期由 `receive()` 自主报 `error_reason+MONITOR`、全零 disable 不触发且保留 arm 时钟、**传感器 attach 后 monitor 窗由固件侧样本续命**（`an_attached_sensor_slides_the_monitor_window`，样本不走线上）；**步进时序模型**（Q10）：per-oid 固件步进链 `config_stepper` 归零 / `queue_step` 空闲首拍-忙延展-首拍过期三态 / `reset_step_clock` 忙拒+重锚，`timer_is_before` u32 回绕（含 2³¹ 假阳边界），过期与忙拒走字典 `static_string_id` 发 shutdown 帧并置 `get_config is_shutdown`、关机后拒步——`timer_is_before_follows_the_firmware_wrap`、`an_expired_first_step_reports_timer_too_close`、`a_stale_chain_expires_until_reset_step_clock_reanchors`（两会话红/绿对）、`reset_of_a_running_chain_reports_the_firmware_error` 四测 + 转红实证（剪过期判定恰红 2 条） |
| `host.rs` | 库路径不存在时报错；对着**真实 host 库**走完整 identify 引导（见 `identify` 一节）+ `shutdown` 后 `receive()` 返回 `None`；**同进程第二次连接接管仍在跑的固件**——序号是库里的静态量（真 MCU 上就是没被复位），所以第二块 `Mcu` 必须采纳它的号才能接上（测试用多一个 `dlopen` 句柄把映射钉住，否则 `dlclose` 会把固件状态一起初始化掉；再开一个设备要等传输任务收尾，库同一进程只允许一个）（帧的重组逻辑由 `frame::FrameStream` 的测试覆盖）。测试构建走**逐字节**读写，所以这条引导的每一帧都真的经历了完整重组；发布构建走库的**整帧接口**（`CONFIG_HOST_FRAME_API`），该路径 `cargo test` 覆盖不到（`cfg(test)` 恒定成立），只有 `cargo build` 的编译校验，曾用一份开了该选项的库手工跑通 identify + `get_clock` |
| `usb.rs` | 拓扑发现用假 sysfs 树：tty 上溯到 USB 设备、取**紧邻**它的 hub 与端口号（`<hub>.<port>`、根 hub 的 `<bus>-<port>`）、非 USB tty 报错；**多层 hub 取最内层那颗**（外层 hub 与它同型号也不受影响）；开关文件查找：`port<N>` / `<hub>-port<N>` 两种命名、根 hub 的 `<usb>-port<N>`、`probe` 交出该路径；告警里的两条规则（含与 `scripts/klipperx-usb-udev.sh` 同一个 glob——比 hub **深一层**，`include_str!` 对脚本兜底）。**要真硬件的几条没自动化**：hub 端口的供电能力（hub 类描述符低两位：`per-port` / `ganged` / `no power switching`，解析部分用真描述符字节对了；读描述符要能开 hub 节点）、`open_hub` 把**根 hub** 也算进来（`nusb::list_devices` 按设计不给 `usbN`，只能从 `nusb::list_buses` 取；MCU 直插机器 USB 口就是这种），以及控制传输本身；两者都在真板/真 hub 上手工验过（根 hub 收下 `SET/CLEAR_FEATURE(PORT_POWER)`，设备断开重枚举） |

| `mod.rs` | `Interface` 的收发：单发单收、多次收发、比对不上报错、无映射条目、克隆共享同一设备、一次发多条输出、帧负载保真、发送错误保留消息文本 |
| `frame_mock.rs` | 夹具自身：单次/多次收发、帧不匹配报错、无映射条目、多输出、空输出、无匹配不发、负载保真、并发收发 |
| `pty.rs` | （仅 `cfg(test)` 的夹具，无独立测试；串口测试用它开真 pty） |

### 上游语料（`src/core/klippy/upstream.rs`）

上游的主机回归测试语料（`test/klippy/*.test` 与数据字典）作为**只读 fixture** 使用，harness 与
用例位于 `src/core/klippy/upstream.rs`（`#[cfg(test)] mod upstream`）。其运行机制、语料结构、
复用分层与当前状态见 [回归测试（`.test` 与数据字典）](regression-tests.md)。

其中 `every_upstream_printer_config_parses` 覆盖 259 份上游 `.cfg`，现已全部通过；它最初暴露的
四处解析器分歧（多行值、`=` 分隔、节头行内注释、`;` 行内注释）已修复，并各有单测。

端到端执行由 `test-support/build.rs` 在 test 构建时生成：扫描 `test/klippy/*.test`，为每个 `CONFIG` 块
生成一个独立 `#[test]`（命名 `upstream_<stem>_config_<n>_<cfg>`），config 路径 / 字典路径 / g-code /
`SHOULD_FAIL` 标志全固化在生成代码里。每个 `[mcu]` 换成 `test: dict=<字典>`，由
`interface/devices/simulator.rs` 的字典驱动应答机跑真实协议路径（identify、配置握手、时钟、ack）。
激活单位是「按 `CONFIG` 拆出的**运行**」，启用条件是**该运行声明的全部字典都已构建**：架构列表
`KLIPPERX_ARCHES`（默认 `linux` + `avr` + 各 ARM 家族，即主机 / `avr-gcc` / `arm-none-eabi` 三类工具链）
与 `KLIPPERX_ALL_ARCHES`（全开）在**构建阶段**过滤 `test/configs/*.config` 并产出同名 `.dict`
（构建失败即报错）；未构建字典的运行生成时带 `#[ignore = "dictionary <name> not built"]`，
不拿别的目标顶替。忽略列表（`IGNORED`，权威在 `crates/test-support/build.rs`，按生成函数名匹配）
现登记 0 条（`out_of_bounds.test` 曾因 harness 双重反转 bug 误登记，已修复移除）；详见
[回归测试](regression-tests.md)。

全语料 37 份文件共 **239 次运行**；默认构建下 2 条（`printers.test` 里 `DICTIONARY pru.dict` 下的
`generic-cramps.cfg` 与 `generic-replicape.cfg`）因未构建 `pru` 字典生成为 `#[ignore]`，
其余 **237 条全部运行并通过（0 失败）**（`out_of_bounds.test` 的双重反转 bug 已修复，
`G1 Y9999` 正确报 `Move out of range`，`SHOULD_FAIL` 满足）；用例的内联 g-code 由端到端运行
真送进 dispatcher，更有独立的内联解析阶段（`upstream_inline_gcode_parses`，仍 `#[ignore]`）。上游 `configparser` 的 `optionxform = str.lower` 已对齐（`mod.rs` 存储侧小写 + `section.rs` 查询侧小写），`Option 'pid_Kp' … must be specified` 类的 49 次回归失败已归零。

生成式运行器取代了旧的 `upstream_test_cases_run`（单个 `#[test]` 串行跑 239 case，十几分钟易超时
且无法单跑定位）；旧环境变量 `KLIPPERX_UPSTREAM_ALL` / `KLIPPERX_UPSTREAM_FILTER` 不再适用。

```bash
cargo test -p klipperx --lib upstream                 # 语料相关的全部用例（含生成的 239 个 #[test]）
cargo test -p klipperx --lib upstream_bed_mesh        # 单条：只跑 bed_mesh.test 的那个 case
cargo test -p klipperx --lib --ignored upstream      # 跑全部被忽略的 case（含 IGNORED 与字典未构建）
KLIPPERX_ARCHES=linux \
  cargo test -p klipperx --lib upstream               # 只编 linux 一份字典（最快）
KLIPPERX_ALL_ARCHES=1 \
  cargo test -p klipperx --lib upstream               # 构建全部目标（需所有交叉工具链）
```

## 写 MCU 相关测试的两个要点

1. **帧要比得完整**：`FrameMock` 比对的是 `Frame`（seq + payload）。请求 payload 可以直接用 `Payload::push_*` 拼，或 `Parser::encode` 得到。
2. **序号对齐**：发送任务每批 +1；接收侧按**块**接受序号，可取「正在等的块」或「下一个块」，也只取低 4 位。**例外：会话的首个新序号**——即使越号也采纳（connection-init 对齐，见 `test_the_first_new_sequence_is_adopted_after_a_repeated_frame`）；首个新序号之后的越号帧一律丢弃（`test_a_mid_session_frame_past_our_next_is_dropped_not_adopted`）。假设备（`FrameMock`）让第 i 个响应用序号 `i & 0xf`（与请求同号）即可——它落在「正在等的块」这一侧；同一序号可以连续出现多帧，这正是真实固件的行为（一条响应 + 一帧空载荷 ack，见 `mod.rs::test_acks_and_repeated_sequences_are_accepted`）。
   序号只有 4 位，超过 16 次交换会回绕，忘记 `& 0x0f` 会让第 17 帧起被判为序号不匹配而丢弃（表现为超时）。

`HostDevice` 的往返测试示范了怎么对付一个**阻塞**的 `receive()`：它从独立线程调用并把结果送回 channel，主线程用 `recv_timeout` 给出 5 秒上限——否则一个真出了问题的手感就是测试永久挂住。

握手测试的现成写法见 `identify.rs` 的 `chunked_mappings`：它按块大小生成「请求帧 → 响应帧」映射，并用 `flate2` 现场压缩字典内容。

## 文档同步

**每次修改完成后，必须在同一批改动里完成受影响的手册修改**，与代码一起提交（仓库级
总则与「改什么 → 同步哪本」的对应表见 [`AGENTS.md`](../../../AGENTS.md) 的「改完必须
同步手册」）。本节是手册侧的细则。

最容易漏的两类：改动 `msg` / `mcu` / `cmd` / `event` / `identify` / `api` 的公开 API 或
分层职责时，更新本手册对应页面（见 [开发手册首页](README.md) 的目录）；改动
`crates/klippy-api` / `crates/klippy-client` 时同理（它们的公开 API 就是别人依赖的协议）。
写进本手册的断言要能当场对着源码成立：**写死的数字与名字**（测试名、端点条数、计数、
二进制大小、工单号）在改动后逐个重核。

### 任务标号的引用规则

`TODO.md` 与 `docs/work-log/` 里的任务/问答标号（`D3`、`F8b`、`T3`、`Q2`、`H1-3` …）
**一旦对应任务完成，就不得再出现在本手册（及其他文档）与源码注释中**：引用处改为
陈述现状，或注明「已归档」；连带的「见 TODO X」「属 X」「X 待办」一并清理。

**豁免范围**：`TODO.md` 与 `docs/work-log/` 自身是台账与档案，其中的历史标号照旧保留，
本规则不适用于这两个位置。悬空标号（查无此号）同样要清——它和过时断言是一类问题。

`cargo doc --no-deps --lib` 的警告数应与改动前一致（根包目前有 2 条残留于 `frame.rs` / `msg/parser.rs`；`klippy-api` 与 `klippy-client` 是 0 条）。新增模块时注意两个陷阱：

1. **模块的文档链接是在它的 `mod` 声明所在作用域里解析的**，不是在被声明模块自己的作用域里。`klippy/mod.rs` 里的 `pub mod …;` 因此都不带 `///` 文档。
2. **把私有模块提升为 `pub mod` 会激活它的公开文档检查**：模块文档里指向 `pub(crate)` 项的链接会报 `links to private item`。`identify` 从 `mcu` 的子模块提升为顶层公开模块时就遇到这一点，需要把这类链接改成纯代码 span。

---

- [← 开发手册首页](README.md)
- [内部架构 ←](architecture.md)
