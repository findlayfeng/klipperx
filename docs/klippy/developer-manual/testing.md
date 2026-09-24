# 测试

> **真板/外设验证不在主线任务里**：需要真实 MCU 或外设的验证列在仓库根的
> [`TESTING.md`](../../../TESTING.md)，不阻塞开发；主线以本文件的 host 单测 + 假 MCU 验收。

测试与被测代码同文件，位于各模块的 `#[cfg(test)] mod tests`，不需要外部进程或真实串口。底层 IO 由 `interface::test::FrameMock` 模拟：它按 FIFO 逐条比对收到的帧，并把预设的输出帧排队给 `receive()`。

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
| `pending.rs` | 注册/配对/取消、未知名字不消费、先到先得、接收端已关闭、只取消一条 |
| `dictionary.rs` | 三张消息表的解析（含 `output` 原样保留）、枚举单值与区间展开、常量、各类畸形输入、`install` 的跳过语义与不注册 `output` |
| `mod.rs` | 构造后未识别（`new` 只注册 identify 一对）、发送错误路径、`Drop` 中止接收任务并释放阻塞读；序号（假设备）：**接管一块还在跑的板子**（首帧是 NAK 号 → 采纳、换号重发同一请求、调用成功、`took_over_session()` 为真、记录器显示发的是 `[0, 9]`）、刚开机的固件不接管也不重发、首帧之后的越号帧被丢且不扰动本次交换。另有**要真硬件的**一例（`test_frame_sequence_sync_against_a_real_board`，`#[ignore]` + `KLIPPERX_HW_SERIAL`）：对同一块不停机的板子连两次，第一次完成 identify 并跨过 4 位回绕，第二次必须报告接管、采纳固件当前的号并继续 `get_clock` |
| `object.rs` | `McuObject`：主/前缀 section 的名字（`[mcu]` → `mcu`，`[mcu zboard]` → `zboard`）、配置构建器在建对象时就可用（可在 connect 前领 oid）、未连接时报 `{}`、连接后报 identify 快照（`mcu_version` / `mcu_build_versions` / `mcu_constants`）、section 没有可用接口时 `connect` 报错、两个对象不能用同一个 chip 名；**固件停机**：收到 `shutdown` 帧后打印机进 shutdown 且状态消息带原因；**`rpi_usb` 没法复位固件时**（`usb_reset_unusable`：hub 报不支持端口供电切换、开关本身失败、固件还在旧会话里、握手后它仍带着配置）把这个 MCU 的 `restart_method` 记成 `command`（内存里，`Printer::override_config`），第一种在真正去切电之前就发生；**`before_firmware_restart`**（在拆机之前、活连接上）：`command` 发 `reset` 并 flush（`test_a_firmware_restart_resets_on_the_live_connection`），物理方式（`rpi_usb` 等）不发（`test_a_physical_restart_method_does_not_reset_on_the_live_connection`） |
| `config.rs` | CRC 标准校验值；oid 从 0 单调发号、走完 `MAX_OIDS` 报错不回绕、定稿后不能再领；`build`：空配置只有 `allocate_oids` + `finalize_config`、`allocate_oids` 带最终计数、命令按加入顺序、CRC 确定且对值与 oid 数敏感、`restart`/`init` 不入 CRC、config 回调在 build 时跑且可继续领 oid/加命令、二次 `build` 报错且不重跑回调、定稿后再加命令/回调/队列槽被拒、未 identify 报 `NotIdentified`、移动队列槽计数；`seconds_to_clock` 用 `CLOCK_FREQ`；`configure`：未配置时把整份配置加 `get_config` 一帧发出并确认、停机或 CRC 不一致时先 `config_reset`（运行中的固件先 `emergency_stop`）再配置、无 `config_reset` 时分别报停机 / CRC 两种配置错误；`Configured` 三个字段：`crc` / `move_count` / `reused`，加 `already_running`（首个 `get_config` 就报已配置或已停机 = 板子没重启） |
| `restart.rs` | 空实现（`command`）与一条不是 USB tty 的串口路径各自的路由与报错；启动探测 `check_usb_power` 只对“串口 + `rpi_usb`”给结论，别的组合一律 `None`（不夺走调用方的 `rpi_usb`）。**要真硬件的没测**：端口开关、`wait_for_new_device` 的重枚举判定、hub 端口的供电能力（`usb::port_power`）都在真机上手工验过 |
| `resource/pin.rs` | `McuChip` 经 `pins` 注册后被 `setup_digital_out` 派发；`McuDigitalOut` 的 build：`config_digital_out` 的 oid/pin 编号/value/default_value/max_duration（2 s × CLOCK_FREQ）、`update_digital_out` 进 restart 列表、`!` 翻转电平、`max_duration` 与 start/shutdown 不一致报错、枚举里没有的引脚报 `Pin 'X' is not a valid pin name on mcu 'Y'`、保留引脚报错；运行期：attach 后 `update`/`queue` 可发送（名字与参数可编码），未 build 与未 connect 各自报错；`resolve_bus_name`：按 `BUS_PINS_<bus>` 预留固件声明的引脚、缺省取名为 0 的总线、`Unknown spi_bus` / `Must specify spi_bus` 两种错误、无总线枚举时原样透传；**析构**：资源注册的 config 回调必须 `Weak` 持有 pins registry（否则 `registry → chip → config → callback → registry` 成环，跨 `teardown` 留住 chip 与其 MCU 连接）—— `test_a_resource_does_not_keep_the_pin_registry_alive` |
| `resource/pwm.rs` | 硬件路径建 `config_pwm_out`（`PWM_MAX` 满量程、restart 的 `queue_pwm_out`）；软件路径建 `config_digital_out` + `set_digital_out_pwm_cycle` + init 的 `queue_digital_out`；`shutdown_value` 非 0/1 的软件 PWM 报错、`max_duration` 与 start/shutdown 不一致报错、`!` 翻转；`next_aligned_clock` 对软件 PWM 按周期上取整、满/全关与硬件 PWM 不调整；`update_pwm` 用估计时钟发送，未连接报错 |
| `resource/adc.rs` | 批量 `query_analog_in`（`bytes_per_report`）/ 旧格式按字典格式串选择；`ADC_MAX` 与 `sample_count*ADC_MAX < 2^16` 上限；`sample_count=0` 不建任何命令；`get_query_slot` 把首报排在估计时钟 +1.5 s；`analog_in_state` 旧格式单值缩放、新格式按 report 周期给每个样本打时钟 |
| `resource/stepper.rs` | 方向变化变 `set_next_step_dir`、连续步变 `queue_step`；`!` 翻转方向线上的方向位；负 `add` 窄化后仍正确 |
| `resource/endstop.rs` | `home_start` 同时武装 endstop 与 trsync（async）；`home_wait` 对主机请求（无固件触发）回 0 |
| `resource/trsync.rs` | 状态报告完成触发组、次级 MCU 报文把组超时拉到最慢那颗；registry 按 oid 路由、同一 MCU 上两个 endstop 共享 registry；一个 trsync 停住多个 stepper；共享轴跨 MCU 被拒（上游 `TriggerDispatch` 同规则） |
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
| `identify.rs` | 单块与多块拼装（含短末块与 4 位序号回绕）、offset 错位、zlib 损坏、**裸 deflate 被拒**（必须是 zlib 包装）、非 JSON、MCU 静默、zip bomb 上限、`Mcu::identify` 与 `Mcu::connect` 全流程；分块夹具按真固件的号（响应与 ack 都盖 `N+1`）；**接管之后握手仍失败时报 `OldSession` 而不是裸超时**（对照：刚开机的板子静默时保持原错误） |

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
| `clock.rs` | 读取时钟、32 位回绕值、握手前失败、超时；另有不依赖 MCU 的 `ClockSync` 实现，验证 trait 作为测试缝可用（文件随 `pub mod clock;` 一并编译） |
| `debug.rs` | `debug_read` / `debug_write` / `debug_ping` / `debug_nop` 与固件格式一致；`debug_result` / `pong` 解码（缺参数报错）；读/写/ping/nop 各自走虚拟 MCU 的整条往返与上线 |
| `spi.rs` | `config_spi`（含 active-high、无片选两变体）/ `spi_set_bus` / `spi_set_sw_bus` 编码；新旧软件总线命令的优先选择（这三例是 async 真调，`mcu_with` + `FrameMock`）、两个都没有时报错；`spi_send` / `spi_transfer` 的编码与 `spi_transfer` 响应解码；`config_spi_shutdown` |
| `i2c.rs` | `config_i2c` / `i2c_set_bus` / `i2c_set_software_bus` 编码；新旧软件总线命令选择（async 真调）；`i2c_transfer` / `i2c_write` / `i2c_read` 的编码与响应解码（`i2c_response` / `i2c_read_response` / `i2c_bus_status`） |
| `thermocouple.rs` | 命令与固件格式一致；芯片类型号与固件枚举对齐；`thermocouple_result` 解码；整条经虚拟 MCU 的往返 |
| `stepper.rs` | `config_stepper` / `queue_step` 的 `args()` 与固件参数序一致（编码后解码回同一组值） |
| `endstop.rs` | `config_endstop` / `endstop_home` 的参数序与固件一致；`pull_up` 负值按字节编码；disable 全零；`home_wait` 的 32 位触发时钟以**本次 move 的 arm clock** 为纪元参考（打印时间远超 MCU 自报时钟时会差一整圈：2³²/16 MHz = 268.44 s） |
| `trsync.rs` | `trsync_start` 参数序与固件一致；`trigger_reason` 枚举号与固件对齐 |

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
| `object.rs` | `configfile` 状态形状与 `set`/`remove_section` 的 pending 记账（含节移除记为 `null`）；`warnings` 的五种形状（`deprecated_option` / `deprecated_value` / `deprecated_gcode` / `deprecated_mcu_code` / `runtime_warning`）的字段与上游文案、按序列化键去重 |
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
| `toolhead.rs` | 额外轴（挤出机）在自己的 trapq 检查与排队、限制拐角；move 到 trapq 生成 `queue_step`；两条共线 move 保住拐角速度；`dwell` 推进 print time；`drip_move` 直灌 trapq（零长度不做事）；未 homed 轴拒绝、零长 move 忽略 |
| `queuing.rs` | `append` 与 `generate` 共用同一 trapq；没事做的 stepper 静默；两个 stepper 在同一 MCU 上都生成 |
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
| `load.rs` | `Printer::load_config`：主 section 先于前缀 section、按 section identifier 登记、`[mcu]`→`mcu` 与 `[mcu x]`→`mcu x`、未知 section 报上游原文 `Section 'x' is not a valid config section`、空配置装载为空、坏接口要到 `connect` 才报、**交给工厂的 section 带上打印机记的内存覆盖**（`override_config` 改的选项真的会被读到，其键名与解析器一样按 `optionxform` 折叠）、**工厂调用 `ConfigWrapper::deprecate` 时警告落到装载器带进来的 `configfile` 对象** |

### `reactor`

时间与定时器。`ManualReactor` 的测试不需要 runtime，直接拨表；`TokioReactor` 用 `#[tokio::test(start_paused = true)]` 与 `tokio::time::advance` 驱动暂停钟。

| 模块 | 覆盖 |
|------|------|
| `reactor.rs` | 契约：回调返回值即下次唤醒时间（`Some` 续、`None` 退）、未到点不跑、`advance` 把钟拨到每个唤醒时间（回调看到的是被唤醒的时刻，不是终点；周期定时器一个周期跑一次）、同时到期按唤醒时间稳定排序、`NOW`（注册在过去）当前就跑、取消后不再跑且可重复取消；`ManualReactor`：从 0 起、`run_due` 不拨钟、回调里可再注册（不在锁下跑回调）、`call_later` 只跑一次；`TokioReactor`：到点才跑、按周期续跑、取消从长睡眠中立即返回且不再跑、已自退的定时器再取消也安全、丢句柄不取消；**串行 dispatcher**：同一时刻到期的按注册顺序一次跑一个（回调互不重叠）、回调里再注册不死锁；**延迟度量**（真时间）：慢回调被报出且带名字、快的一轮不报、被慢回调挡住的后继定时器 `lateness` 超阈 |

### `klippy`（主机编排）

| 模块 | 覆盖 |
|------|------|
| `klippy.rs` | `is_restart` 只认 `restart` / `firmware_restart`；`klippy_process` 一回合内 `restart` 重建、`exit` 结束（用 `RestartOnce` 替身对象）；**A3 机制**：在 `machine_handle.enter()` 之下建的 `test:` 接口捕获到的是机器 runtime，而不是 ambient 的 API runtime（`Interface::with_transport` 的 `Handle::current()`） |

| `stress.rs` | 段计算与引脚解析：按节名找 MCU（带名的不拿裸 `[mcu]`、空名拿裸的）、引脚名的 chip 副本只在有冒号时出现、别的 MCU 上的 stepper 被跳过；一段填满时长且间隔均匀、时钟变慢拉长间隔而时长不变、间隔不到 0 被截且命令有上界；引脚经字典枚举解析 |
| `main.rs` | CLI 形状：不给子命令时跑主机、选项随默认子命令走、显式拼法同效、单独给子命令回帮助；`--api-server` 每处默认一致、空值=主机不开服务；`--tui` 是 CLI 自己的、客户端子命令不受影响；`--logfile` 两种拼法都是主机的；裸调用打帮助、缺配置文件的旗标后置才报、主机子命令单跑打帮助、主机参数与子命令不可混用 |
| `logging.rs` | 窗口收到主机记录；`--verbose` 与 `RUST_LOG` 取更详细者；窗口槽在层级查找处；`--logfile` 收到格式化字节、打不开则降级 stdout；rollover info 排序并清空 |

### `klippy-api`

两层测法：协议与寄存器层**不需要 socket**（`ClientConnection::receive` 直接吃字节，推送由实现了 `PushTarget` 的测试替身接住）；监听与连接层用**真的 socket**，在临时目录里 bind Unix socket、在 `127.0.0.1:0` 上 bind TCP，然后真连上去发请求。手工验证用 `klippy-client api` / `klippy-client console`（见 [第三方开发手册](../third-party-dev/README.md)）。

| 模块 | 覆盖 |
|------|------|
| `address.rs` | 裸值是 socket 路径（上游形式）、`unix:` 前缀的两种写法、`tcp:` / 裸 `host:port` / 名字 / IPv6 / `:port` 都识别为 TCP、`Display` 带传输前缀、空值与 `unix:` 无路径被拒、`tcp:` 后面端口不是数字 / 缺端口被拒、未知 scheme（`http://…`）报错而不是变成文件名 |
| `protocol.rs` | 分帧（一次读里多条、一条被拆成多次读、前导分隔符产生的空消息）、`encode` 的分隔符结尾；请求解析（`id` 缺失与为 `null` 均视为不要应答、非对象 / 无 `method` / `params` 非对象一律拒绝、非字符串 `id` 原样保留）；应答形状（result / error、无 `id` 时失败也静默）；`Params` 区分缺失与类型错、整数可作浮点而浮点不可作整数、`true` 不是整数、`get_or` 不检查默认值；模板合并（`params` 冲突时模板优先，同上游）、模板可省略且类型受检 |
| `registry.rs` | 路径唯一（普通端点与 mux 路径同一命名空间）、mux 各实例的 key 必须一致且 value 不重复、`list_endpoints` 排序并含 mux 路径、按名分发与未知方法报错、端点拿得到自己的连接、mux 按 key 选实例 / 缺 key / 未知值 / 非字符串值、注册 `None` 时 key 可省略；remote method 的模板合并推送、多连接、重复注册替换模板、已断开连接被清理、无活动连接与未注册两种错误 |
| `server.rs` | **真 socket**：Unix 与 TCP 各一个往返、TCP 上报的 target 是绑定后的地址（不是 `:0`）、未知方法与 handler 失败都回 `error`、一次写里两条请求按序应答、跨 TCP 分段的消息只应答一次、两个客户端同时被服务。**socket 文件**：遗留文件名被 bind 替换成真 socket、server 被 abort 后文件消失。**单连接**（无 socket）：应答字段顺序（`id` 在前，靠不经 `Value` 直接序列化保证）、无 `id` 不应答、畸形消息被跳过而连接继续可用、推送与应答同样入队、连接关闭后不再入队、`push` 能唤醒等待方（用 `Notify`，带超时断言）、空发件箱 flush 直接成功、写不进去的客户端被 5 秒超时切断（用 `start_paused` 让暂停时钟直接跳过这 5 秒，无需真等） |
| `error.rs` | `Bind` / `Connect` / `Io` 的 Display 直接给出调用处拼好的文本，`Closed` 给出固定文案 |
| （主机侧）`endpoints/info.rs` | 端点路径、`client_info` 可省略且必须是对象、响应 12 个字段与文档逐个对齐、`log_file` 为 `None` 时是 `null` 而非缺字段、响应不回显 `client_info`；**handler**：状态跟着机器（`startup`→`shutdown`）、四个 start args 与 `process_id` 真的进响应、`klipper_path` / `python_path` 非空且确实不存在 |
| （主机侧）`api/start_args.rs` | CPU 描述的解析（processor 计数 + model name、缺 model 时问号）、`collect` 带上配置路径 / 版本、`log_file` 为 `None` |
| （主机侧）`api/webhooks.rs` | 对象名是 `webhooks`、对象报的就是打印机状态（`startup` / `ready` / `shutdown` 三种都跟得上，因为它是读状态而不是存状态） |
| （主机侧）`api/mod.rs` | `register` 一次装完服务器这一侧：装完 `objects/list` 含 `webhooks`、端点表含 13 条路径（`info` / `emergency_stop` / `list_endpoints` / `objects/*` / 五个 `gcode/*` / `query_endstops/status` / `register_remote_method`）；装完后客户端能按名查到 `info`、能按名查到 `webhooks` 并跟着状态变；**重复注册报错**且归为「自己接错线」的 `RegistrationError`（不是客户端能引起的错误） |
| （主机侧）`endpoints/objects_list.rs` | 端点路径、没有组成部分的机器列表为空、按注册顺序列出多个对象、不看参数（上游的 handler 不读任何参数） |
| （主机侧）`endpoints/objects_query.rs` | 端点路径、`null` 取全部字段 / 列表取指定字段 / 不存在的字段回 `null`、未知对象回 `{}`（不报错）、对象名与应答的 `eventtime` 一致且真的传给了源、服务器那个 `webhooks` 对象随机器 `startup`→`ready`→`shutdown` 变化、`objects` 缺失 / 非对象 / 值非 `null` 或字符串数组分别报三种错（文本对齐上游的 `Invalid argument`）、空字段列表取空、一次查询多个对象、空 `objects` 是空 `status` |
| （主机侧）`endpoints/objects_subscribe.rs` | 订阅请求**立即**回一份全量快照（所有请求字段都返回）、0.25 s 后只推变化的字段、无变化不推、`response_template` 包住每次推送、`null` 字段列表展开为对象当时的字段、字段从缺到有会推而一直缺不推、未知对象回 `{}` 且不推、连接关闭后下一 tick 清理并自行停掉定时器（再没有 tick）、同一连接再订阅是替换不是追加、一个定时器服务多个订阅者、注册表按路径可达、参数校验与 `objects/query` 一致、`response_template` 非对象被拒 |
| （主机侧）`endpoints/gcode.rs` | 五条路径名；`gcode/help` 返回扁平命令表；`gcode/script` 执行并回 `{}`；处理器错误变成**不关停 klippy** 的 `ApiError::CommandError`；缺 `script` 报 `MissingArgument`；`gcode/firmware_restart` 走到内置命令并让 `run()` 返回 `firmware_restart`；`gcode/subscribe_output` 把 `// …` 输出按模板推到连接；`gcode` 还没注册时报打印机状态 |

| （主机侧）`endpoints/emergency_stop.rs` | 请求把打印机停机并回 `{}`（上游 `emergency_stop` 语义） |
| （主机侧）`endpoints/query_endstops.rs` | 路径是上游那条 `query_endstops/status`；无 endstop 时回空对象 |
| （主机侧）`endpoints/register_remote_method.rs` | 注册的方法收到模板与参数；`response_template` 可选；缺 `remote_method` 报参数错；只推给注册的那条连接 |

### `klippy-client`

两段：会话层与连接层用**真 socket**（每个用例自己起一个 `Server`，因此测的是「两边真能对话」）；窗口是**纯渲染测试**，用 ratatui 的 `TestBackend` 画进内存缓冲再断言每行文字，不需要终端。

| 模块 | 覆盖 |
|------|------|
| `lib.rs` | `PARAMS` 必须是 JSON 对象（数组 / 数字 / 非 JSON 分别报错并说明实际类型）、`--api-server` 与主机同一套解析（socket 路径、`tcp:`、`http://` 被拒） |
| `connection.rs` | 请求得到应答且 `id` 与发出的方法名配对、错误应答仍然是应答（可读到 `error.message`）、无 `id` 的消息归为推送、`"id": null` 发出后不等待也不登记、一次读里的多条消息都会被依次交出、连接被挂断时报连接错误而不是挂着、连不上时错误里带上尝试过的地址、**被取消的读不会影响下一次读**（窗口与行模式每读一行输入都会与 socket 竞争，这条是回归测试）、TCP 也能连 |
| `session.rs` | 握手发出 `info` 并把应答登记、裸方法名会补 `id`、方法 + YAML 参数（`{value: 3}` 无需引号）、参数写错只提示不结束会话（YAML 序列 / 标量分别报“必须是 mapping”、未闭合的 mapping 才报解析错）、整条请求对象可以是 YAML（`{method: echo, params: {value: 7}}`，显式 `id` 不被覆盖、缺 `id` 时补上、`"id": null` 保留且不登记）、既不是本地命令也不是请求对象的行仍按 `method [params]` 处理、缺 `method` 本地拒绝、空行什么也不做、本地命令不发给服务端（含 `.quit` 的三种写法）、`.subscribe <名字>` 的请求体与模板、`.subscribe` 无参数时先 `objects/list` 再订阅全部（两个请求都要登记）、失败应答的文本、每条请求都记成 `Sent`、`Entry::text()` 的四种形状、`drain` 会打齐欠着的应答且无事可做时不等待、**g-code 模式**把整行发 `gcode/script`（`.` 开头的 `.quit` 仍是本地命令）、`subscribe_gcode_output` 发出的模板 |
| `tui.rs` | 行编辑（光标处插入 / 删除 / 行首行尾 / `^A` `^E` `^U`）、历史前进后退到空行、长行按光标滚动（游标要占一格）、渲染：状态行随 `info` 与 `webhooks` 推送更新、日志里的应答/推送/错误/通知各自成行、应答/推送/发出的请求都以 `<`/`>` 开头空一格再写正文，正文默认 **YAML**（`.json` 切回紧凑 JSON，标记与空格不变；错误仍是单句）、相邻两条消息用两种前景色交替（`Color::LightYellow` / `Color::LightGreen`；错误仍红；交替只数消息，中间的日志行不打乱）、`.yaml`/`.json`/`.help` 由窗口自己接（`.help` 在会话说明后补 Window 一节），`! ` 前缀、翻页提示与回到底部、新条目回到最新处而翻回去看的人保持原位、输入行与 `local>` 提示符、**g-code 模式**的 `gcode>` 提示符与 `^G` / `.gcode` 切换（本地命令仍用 `local>`）、断线在状态行显示、样式分类（发出=暗、错误=红）；按宽度折行：同一条目自己换行的续行**跟在后面**（否则日志从底往上收集会把一个条目的行反过来）、多行条目保持行序、新条目在底部；长状态消息的状态行会折行（不再被截掉） |

窗口本身（raw mode、备用屏幕、按键线程、`select!` 主循环）没有自动化测试：那需要真终端。验证方式是在 tmux 里跑一遍（`tmux new-session -d … 'klippy-client console …'` + `capture-pane`），杀掉主机看它是否带着错误退出并把终端还回去，以及在 `klipperx --tui`（两种写法都试）下按 `^C` 看主机是否随窗口一起退出、socket 文件是否被清掉。

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
| `output_pin.rs` | `value` / `shutdown_value` 落到 `setup_start_value`，且无条件 `setup_max_duration(0)`（所以 `value: 1` + 默认 `shutdown_value: 0` 合法）；`SET_PIN PIN=… VALUE=…` 驱动输出（`>=0.5` 为开）并更新 `get_status`；缺 `VALUE` 报错；两个 pin 各自独立；缺 `pin` / 非数字 `value` / 非布尔 `pwm` 各自报配置错误；`pwm: true` 走 `setup_pwm` 并把 `cycle_time` / `hardware_pwm` / `value` 落到资源，`SET_PIN` 调 `update_pwm`；`cycle_time <= 0` 报错 |
| `board_pins.rs` | `aliases` 与 `aliases_*` 都注册；`mcu` 列表指定目标 chip；`<...>` 值走保留；未知 chip、缺元素、别名冲突各自报错（冲突带 section 前缀）；对象不可查询 |
| `heaters.rs` | 传感器工厂表：未知 `sensor_type` 报上游文案 `Unknown temperature sensor 'x'`；`register_sensor` 把 section 名计入 `available_sensors`；`ensure` 幂等，并把 `DS18B20` 工厂带进来（对应上游 `temperature_sensors.cfg`）；`get_status` 的三个列表 |
| `temperature_sensor.rs` | **本文件无独立测试**：其行为（`min_temp` 默认 `KELVIN_TO_CELSIUS`、`max_temp` 须高于 min、`sensor_type` 交给 `heaters`、`get_status` 的 `round(…, 2)` 与读数 0 不计入 min/max）目前只经 `load.rs` 的工厂表与上游语料的端到端用例间接碰到；`setup_minmax`/`setup_callback` 的落点由 `heaters.rs` 与各传感器工厂的测试覆盖 |
| `ds18b20.rs` | `serial_no` → 小写 hex、`ds18_report_time`（≥ `DS18_MIN_REPORT_TIME`）、`sensor_mcu` 找 MCU 并领 oid；build 加 `config_ds18b20` 与 `query_ds18b20`（init）；post-init 按 oid 绑定 `ds18b20_result`（每 MCU 一个 registry），fault 丢弃，`next_clock - report_clock` 映射回 print time |
| `extruder.rs` | `[extruder]` 装载并注册命令；编号兄弟（`extruder1`…）经主节连带读取；`SET_PRESSURE_ADVANCE` 不给 `EXTRUDER=` 时命中默认项并转给活动挤出机、给了不存在的名字报可选项列表；挤出检查按上游（`max_extrude_*` 越界拒绝）；拐角用 `instantaneous_corner_velocity` |
| `heater_bed.rs` | `[heater_bed]` 装载并注册 `M140`；`M140` 设目标、`M190` 也设目标（不等温，与文档一致），`get_status` 的 `temperature` 是数 |
| `heater_generic.rs` | **无独立测试**：工厂在装载表中（`load.rs`），加热器选项与控制环由 `heaters.rs` 的测试覆盖 |
| `fan.rs` | 上游默认值装载、`shutdown_speed` 被 `max_power` 截顶；`M106` 设速 / `M107` 关、负值拒绝；kick-start 满速后回落、新请求覆盖挂起的 kick；`off_below` 把小请求归零；`max_power` 截顶；`enable_pin` 只在 0→非 0 翻转；`gcode:request_restart` 停风；**`tachometer_pin` 拒收**（而非静默 `rpm: null`）；缺 `pin` 点名、越界报哪一边、坏数字报原文 |
| `gcode_move.rs` | `G1` 解析轴与速度、未点名的轴不动、记住上一笔速度、非正进给拒绝；G90/G91 切换、G92 锚定后下一笔在界内、裸 `G92` 全零；M83 的 E 相对而轴绝对；M220/M221 缩放速度与挤出；锚定只重挂 homed 的轴；SAVE/RESTORE 状态（未存的名字报错）；`M114` 报 G-Code 位置；`get_status` 对齐上游；第二个坐标系不能默默夺槽；英寸制拒绝 |
| `toolhead.rs` | `[printer]` 的轴索引与 move 上下文；不支持的 `kinematics` 报配置错、`none` 不要 stepper；`stepper_z1` 并入 Z rail；corexy 族装载建 rail；MCU 错误带节名；限值来自 `[printer]`；move 到规划器、未 homed 轴拒绝；`G4` 推进 print time；`SET_KINEMATIC_POSITION` 回零并清状态；探针式回零 `probing_move`：触发即停、事件顺序（`homing_move_begin` 先于 `home_start`）、无触发报 `No trigger on probe after full movement`、零位移报 `Probe triggered prior to movement`；`flush_step_generation`：入队的 move 冲刷后 commanded/history/print_time 可见、`set_position` 先冲刷再改位并标 homing 轴与恰发一次 `toolhead:set_position` 事件、`z_stepper_names` 按 Z 轨配置序（`kinematics: none` 为空）；停止后把指令位置设到**停止点**（drip 循环停下那一刻的 trapq 位置）并返回它——上游按触发时钟读固件步数，语料假 MCU 无步数模型，故以此为替身 |
| `stepper.rs` | 节名→轴、轴索引与 mathutil 一致；步距按几何算；节装成 stepper 对象；`endstop_pin` 建 rail 的 endstop 与 `HomingInfo`；endstop 居中推不出方向时报错；`gear_ratio` 除进步距；缺 pin 点名节、不同 MCU 的同轴引脚被拒、`position_endstop` 越界被拒 |
| `stepper_enable.rs` | 节装载；无 `enable_pin` 时是“永远使能”；写了则建使能脚（共享/取反路径） |
| `bed_mesh.rs` | 语料选项全量认领（含 `faulty_region_*` 对）；矩形网格按行 zigzag、间距下取整到百分位；圆床按 `mesh_radius` 过滤且用 `round_probe_count`；过近点报 `bed_mesh: min/max points too close together` |
| `manual_probe.rs` | 二分插入点（`bisect_left`）、空闲状态形状；交互路径（`TESTZ` 移动、`ACCEPT` 校验、`ABORT` 收尾、命令注销）由上游语料端到端覆盖 |
| `probe.rs` | 选项全量认领与默认值（`speed` 5.0、`samples` 1、`sample_retract_dist` 2.0、`samples_result` median、`samples_tolerance` 0.100、`deactivate_on_each_sample` true）；`lift_speed` 缺省回退 `speed`；`samples_result` 非法值报上游文案；虚拟端停校验：`z_virtual_endstop` 通过、其它 pin 名报 `Probe virtual endstop only useful as endstop pin`、`!`/`^` 报 `Can not pullup/invert probe virtual endstop`；归并算法：`average` 逐轴平均、`median` 按 Z 取中位（偶数样本取中间两者均值）；命令与会话路径由上游语料端到端覆盖 |
| `upstream.rs` | 语料驱动（字典、CONFIG/文件输出、SHOULD_FAIL）之外，另有 **5 条聚焦 E2E**：普通端停 `G28 Z`、`probe:z_virtual_endstop` 的 `G28 Z`、`G28 + PROBE`、`G28 + PROBE_CALIBRATE/TESTZ/ACCEPT`、`G28 + BED_MESH_CALIBRATE`（3×3）——把「端停/探针真的能驱动一次回零」钉在假 MCU 上 |
| `query_endstops.rs` | 全部限位读一遍并记住、取反的限位翻转电平、`M119` 逐个报（经虚拟字典帧解码） |
| `i2c_device.rs` | 硬件设备要地址、地址越界拒；注册两条调试命令；只给一个软件引脚报错、未知 MCU 点名节、软件引脚须同 MCU、就绪后 `get_status` 报地址与速度 |
| `spi_device.rs` | 注册两条调试命令；无片选允许；`spi_mode`/`spi_speed` 越界拒、部分软件引脚报错、片选须在指定 MCU、未知 MCU 点名节；就绪后 `get_status` 报配置；软件设备接受本 MCU 引脚 |
| `static_digital_output.rs` | 每个引脚都被预留、取反的引脚有记录、缺 `pins` 报错 |
| `adc_temperature.rs` | 线性插值正反向、热敏电阻 Steinhart-Hart 与 Beta 模型（与上游公式对拍） |
| `spi_temperature.rs` | MAX6675/MAX31855 转换、符号位负温、MAX31856 与 MAX31865 转换 |
| `temperature_mcu.rs` | 单点直线、两点标定、手动标定读上游选项（`temperature_sensor` 节上的标定点） |
| `temperature_combined.rs` | 三种合并方式（`min`/`max`/`mean`）与舍入 |
| `bus_debug.rs` | `DATA=` 十六进制往返（`test_hex_round_trips`） |
| `error_mcu.rs` | 已知固件消息拿到它的提示、MCU shutdown 被扩成原因+提示、`is_shutdown` 说“此前已停”、无关停机仍告诉用户敲什么；连接错误拿到 firmware_restart 提示；协议错误列出需要升级的 MCU（6 例，无配置节、由第一个 `[mcu]` 拉起） |

### `interface`

| 模块 | 覆盖 |
|------|------|
| `canserial.rs` | **链路层全部单测**：节点号→仲裁 ID 的映射（`0x100+2n`，回包用 +1）、字节流按 8 字节切成 CAN 帧（含整除时不多出空帧）、按帧重组回消息块（最后一帧才成帧）、非本节点的帧被忽略、CAN 帧 ABI 布局（id/dlc/data 偏移与 16 字节大小）、节点指派报文与 Klipper 一致（`CMD_SET_NODEID` + UUID + nodeid）、打不开的 CAN 接口报错并带上名字。**socket 层没有测试**：本环境没有 CAN 接口，`vcan` 又需要特权加载，所以 `CanSerialDevice` 的 socket 部分只经过编译，未在真实总线上跑过（真实 `can0` 的验收需要一台有 CAN 的机器） |
| `serial.rs` | 用**虚拟串口**（`posix_openpt` 开的 pty 对）验证：`send` 写出的就是线上的整帧（raw 模式没有做任何转换）、`receive` 把分片的字节重新拼成帧、`shutdown` 让阻塞中的 `receive` 返回 `None`、打不开的端口报错并带上路径；另有一例走 `Interface` 的异步收发 |
| `simulator.rs` | 字典驱动的应答机（`test: dict=<file>`）：坏字典路径报错；对着它走**真实** `Mcu::connect`——identify 分块回 zlib 字典、装字典、块级 ack，再由 `get_clock` 经普通调用路径拿回响应（验证序号与发送窗口确实被推进） |
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

端到端执行（`upstream_test_cases_run`）把每个 `[mcu]` 换成 `test: dict=<字典>`，由
`interface/devices/simulator.rs` 的字典驱动应答机跑真实协议路径（identify、配置握手、时钟、ack）。
激活单位是「按 `CONFIG` 拆出的**运行**」，启用条件是**该运行声明的全部字典都已构建**：架构列表
`KLIPPERX_ARCHES`（默认 `linux` + `avr` + 各 ARM 家族，即主机 / `avr-gcc` / `arm-none-eabi` 三类工具链）
与 `KLIPPERX_ALL_ARCHES`（全开）在**构建阶段**过滤 `test/configs/*.config` 并产出同名 `.dict`
（构建失败即报错）；未构建字典的运行直接跳过，不拿别的目标顶替。另有忽略列表登记因缺配置节而必然
失败的 `.test`；`KLIPPERX_UPSTREAM_ALL=1` 只作用于该列表，不能让字典未构建的运行跑起来。内联 g-code
阶段仍以 `#[ignore]` 保留（需要同一批缺失的节）。

全语料 37 份文件共 **239 次运行**；默认构建下只有 2 条（引用 `pru`）因字典未构建跳过，其余 237 条
全部可用；其中 **`linuxtest.test`、`commands.test`、`out_of_bounds.test` 已转绿**（T1 与 b39750f），其余 234 条运行在忽略列表里（34 个 `.test` 文件）。上游 `configparser` 的 `optionxform = str.lower` 已对齐（`mod.rs` 存储侧小写 + `section.rs` 查询侧小写），`Option 'pid_Kp' … must be specified` 类的 49 次回归失败已归零。

```bash
cargo test -p klipperx --lib upstream                 # 语料相关的全部用例
cargo test -p klipperx --lib upstream -- --ignored    # 内联 g-code 阶段（未实现，会失败）
KLIPPERX_ARCHES=linux \
  cargo test -p klipperx --lib upstream_test_cases_run # 只编 linux 一份（最快）
KLIPPERX_ALL_ARCHES=1 \
  cargo test -p klipperx --lib upstream_test_cases_run # 构建全部目标（需所有交叉工具链）
KLIPPERX_UPSTREAM_ALL=1 \
  cargo test -p klipperx --lib upstream_test_cases_run # 只去掉忽略列表，跑全部可用运行
```

## 写 MCU 相关测试的两个要点

1. **帧要比得完整**：`FrameMock` 比对的是 `Frame`（seq + payload）。请求 payload 可以直接用 `Payload::push_*` 拼，或 `Parser::encode` 得到。
2. **序号对齐**：发送任务每批 +1；接收侧按**块**接受序号，可取「正在等的块」或「下一个块」，也只取低 4 位。假设备（`FrameMock`）让第 i 个响应用序号 `i & 0xf`（与请求同号）即可——它落在「正在等的块」这一侧；同一序号可以连续出现多帧，这正是真实固件的行为（一条响应 + 一帧空载荷 ack，见 `mod.rs::test_acks_and_repeated_sequences_are_accepted`）。
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
