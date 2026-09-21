# 测试

测试与被测代码同文件，位于各模块的 `#[cfg(test)] mod tests`，不需要外部进程或真实串口。底层 IO 由 `interface::test::TestDevice` 模拟：它按 FIFO 逐条比对收到的帧，并把预设的输出帧排队给 `receive()`。

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
| `allocate_oids.rs` | `allocate_oids` 的线上形状（id 2 的 VLQ + `%c` 计数）、`u8::MAX` 往返编码一致 |
| `config.rs` | `get_config` / `finalize_config` 的编码形状；`config` 响应按名解码（已配置 / 未配置且已停机两态）、参数类型不符报 `Decode` |
| `uptime.rs` | `get_uptime` 的编码形状；`uptime` 两段重组为 64 位时钟、跨 32 位回绕时排序正确、参数类型不符报 `Decode` |
| `shutdown.rs` | `emergency_stop` / `clear_shutdown` 两个无参命令的线上 id |
| `clock.rs` | 读取时钟、32 位回绕值、握手前失败、超时；另有不依赖 MCU 的 `ClockSync` 实现，验证 trait 作为测试缝可用。**该文件目前不参与编译**（`pub mod clock;` 被注释），这 5 个测试与文件一起休眠，恢复时自动回归 |

### `event`

| 模块 | 覆盖 |
|------|------|
| `mod.rs` | `Mcu::bind_event` 端到端投递（绑定的回调经 `Parser` 回调收到事件帧）、握手前 `NotIdentified`、字典缺失报 `UnknownMessage`、占位日志订阅可绑定 |
| `stats.rs` | `stats` 事件按名解码（`count` / `sum` / `sumsq`）、参数类型不符报 `Decode` |
| `shutdown.rs` | `shutdown` 解出 `static_string_id` 原因与可选 `clock`、`is_shutdown` 只解原因、`starting` 无参；原因不在枚举里时报 `?N` 而不编造 |

### `gcode`

| 模块 | 覆盖 |
|------|------|
| `gcode.rs` | 解析：传统命令拆字母+值、行号 `N…` 跳过、`;` 注释与空行、扩展命令 `KEY=VALUE`（带引号/注释）、畸形参数报 `Malformed command`、传统/扩展与扩展名校验；分派：注册后运行、重名/非法名被拒、未知命令只提示不报错、`CommandError` 停下脚本并回 `!!`、未 ready 时 ready-only 命令报状态、内置 `M115` 在未 ready 时也可用、`M112` 触发 shutdown、`HELP` 列命令；mux：按 key 选处理器、未注册值列出选项、只能一个 key；参数访问器：缺参/解析失败/默认值/范围；`get_status` 报命令表；**析构**：命令表不能反过来持有 dispatcher（内建命令与 mux dispatcher 都用 `Weak<Inner>`），否则整份 dispatcher 连同其 handler 捕获的资源——直到一条仍在读串口的 MCU 连接——会跨 `teardown` 泄漏，`firmware_restart` 时新旧连接抢帧—— `test_dropping_the_dispatcher_frees_its_handlers` |

### `printer`

| 模块 | 覆盖 |
|------|------|
| `printer.rs` | 状态与事件名即线上名；生命周期：新机器是 `startup`、`bring_up` 先按注册顺序 `connect` 每个对象再上线到 `ready` 并按序发 `connect`/`ready`/（firmware_restart）/`disconnect`、对象 `connect` 失败即 `invoke_shutdown` 并带上原因、已停机的机器 `bring_up` 不 connect 任何对象、handler 按注册顺序调用、`run` 等另一线程的 `request_exit`（先在另一线程起 `run`）、先请求退出则不等待、首个退出结果固定、`invoke_shutdown` 只接受首条消息、停机后 `bring_up` 不会变成 `ready`；对象表：**新机器没有任何对象**（`webhooks` 是主机侧的）、注册顺序、重名被拒且首个注册保留、按名 `lookup_object` 拿得到且未注册返回 `None`、`connect` 默认是空实现；时间：`eventtime` 就是机器的 reactor 的钟（`ManualReactor` 拨表后跟着变）、`Printer::reactor()` 交回的正是建机器时给的那个；**内存覆盖**：`override_config` 按 section 分存、重记即覆盖、别的 section 不受影响；`prepare_firmware_restart` 按注册顺序 await 每个对象的 `before_firmware_restart`（`test_prepare_firmware_restart_awaits_every_part`） |
| `load.rs` | `Printer::load_config`：主 section 先于前缀 section、按 section identifier 登记、`[mcu]`→`mcu` 与 `[mcu x]`→`mcu x`、未知 section 报上游原文 `Section 'x' is not a valid config section`、空配置装载为空、坏接口要到 `connect` 才报、**交给工厂的 section 带上打印机记的内存覆盖**（`override_config` 改的选项真的会被读到） |

### `reactor`

时间与定时器。`ManualReactor` 的测试不需要 runtime，直接拨表；`TokioReactor` 用 `#[tokio::test(start_paused = true)]` 与 `tokio::time::advance` 驱动暂停钟。

| 模块 | 覆盖 |
|------|------|
| `reactor.rs` | 契约：回调返回值即下次唤醒时间（`Some` 续、`None` 退）、未到点不跑、`advance` 把钟拨到每个唤醒时间（回调看到的是被唤醒的时刻，不是终点；周期定时器一个周期跑一次）、同时到期按唤醒时间稳定排序、`NOW`（注册在过去）当前就跑、取消后不再跑且可重复取消；`ManualReactor`：从 0 起、`run_due` 不拨钟、回调里可再注册（不在锁下跑回调）、`call_later` 只跑一次；`TokioReactor`：到点才跑、按周期续跑、取消从长睡眠中立即返回且不再跑、已自退的定时器再取消也安全、丢句柄不取消；**串行 dispatcher**：同一时刻到期的按注册顺序一次跑一个（回调互不重叠）、回调里再注册不死锁；**延迟度量**（真时间）：慢回调被报出且带名字、快的一轮不报、被慢回调挡住的后继定时器 `lateness` 超阈 |

### `klippy`（主机编排）

| 模块 | 覆盖 |
|------|------|
| `klippy.rs` | `is_restart` 只认 `restart` / `firmware_restart`；`klippy_process` 一回合内 `restart` 重建、`exit` 结束（用 `RestartOnce` 替身对象）；**A3 机制**：在 `machine_handle.enter()` 之下建的 `test:` 接口捕获到的是机器 runtime，而不是 ambient 的 API runtime（`Interface::with_transport` 的 `Handle::current()`） |

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
| （主机侧）`api/mod.rs` | `register` 一次装完服务器这一侧：装完 `objects/list` 含 `webhooks`、端点表含九条路径（`info` / `objects/*` / 四个 `gcode/*`）；装完后客户端能按名查到 `info`、能按名查到 `webhooks` 并跟着状态变；**重复注册报错**且归为「自己接错线」的 `RegistrationError`（不是客户端能引起的错误） |
| （主机侧）`endpoints/objects_list.rs` | 端点路径、没有组成部分的机器列表为空、按注册顺序列出多个对象、不看参数（上游的 handler 不读任何参数） |
| （主机侧）`endpoints/objects_query.rs` | 端点路径、`null` 取全部字段 / 列表取指定字段 / 不存在的字段回 `null`、未知对象回 `{}`（不报错）、对象名与应答的 `eventtime` 一致且真的传给了源、服务器那个 `webhooks` 对象随机器 `startup`→`ready`→`shutdown` 变化、`objects` 缺失 / 非对象 / 值非 `null` 或字符串数组分别报三种错（文本对齐上游的 `Invalid argument`）、空字段列表取空、一次查询多个对象、空 `objects` 是空 `status` |
| （主机侧）`endpoints/objects_subscribe.rs` | 订阅请求**立即**回一份全量快照（所有请求字段都返回）、0.25 s 后只推变化的字段、无变化不推、`response_template` 包住每次推送、`null` 字段列表展开为对象当时的字段、字段从缺到有会推而一直缺不推、未知对象回 `{}` 且不推、连接关闭后下一 tick 清理并自行停掉定时器（再没有 tick）、同一连接再订阅是替换不是追加、一个定时器服务多个订阅者、注册表按路径可达、参数校验与 `objects/query` 一致、`response_template` 非对象被拒 |
| （主机侧）`endpoints/gcode.rs` | 五条路径名；`gcode/help` 返回扁平命令表；`gcode/script` 执行并回 `{}`；处理器错误变成**不关停 klippy** 的 `ApiError::CommandError`；缺 `script` 报 `MissingArgument`；`gcode/firmware_restart` 走到内置命令并让 `run()` 返回 `firmware_restart`；`gcode/subscribe_output` 把 `// …` 输出按模板推到连接；`gcode` 还没注册时报打印机状态 |

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

### `interface`

| 模块 | 覆盖 |
|------|------|
| `canserial.rs` | **链路层全部单测**：节点号→仲裁 ID 的映射（`0x100+2n`，回包用 +1）、字节流按 8 字节切成 CAN 帧（含整除时不多出空帧）、按帧重组回消息块（最后一帧才成帧）、非本节点的帧被忽略、CAN 帧 ABI 布局（id/dlc/data 偏移与 16 字节大小）、节点指派报文与 Klipper 一致（`CMD_SET_NODEID` + UUID + nodeid）、打不开的 CAN 接口报错并带上名字。**socket 层没有测试**：本环境没有 CAN 接口，`vcan` 又需要特权加载，所以 `CanSerialDevice` 的 socket 部分只经过编译，未在真实总线上跑过（真实 `can0` 的验收需要一台有 CAN 的机器） |
| `serial.rs` | 用**虚拟串口**（`posix_openpt` 开的 pty 对）验证：`send` 写出的就是线上的整帧（raw 模式没有做任何转换）、`receive` 把分片的字节重新拼成帧、`shutdown` 让阻塞中的 `receive` 返回 `None`、打不开的端口报错并带上路径；另有一例走 `Interface` 的异步收发 |
| `host.rs` | 库路径不存在时报错；对着**真实 host 库**走完整 identify 引导（见 `identify` 一节）+ `shutdown` 后 `receive()` 返回 `None`；**同进程第二次连接接管仍在跑的固件**——序号是库里的静态量（真 MCU 上就是没被复位），所以第二块 `Mcu` 必须采纳它的号才能接上（测试用多一个 `dlopen` 句柄把映射钉住，否则 `dlclose` 会把固件状态一起初始化掉；再开一个设备要等传输任务收尾，库同一进程只允许一个）（帧的重组逻辑由 `frame::FrameStream` 的测试覆盖）。测试构建走**逐字节**读写，所以这条引导的每一帧都真的经历了完整重组；发布构建走库的**整帧接口**（`CONFIG_HOST_FRAME_API`），该路径 `cargo test` 覆盖不到（`cfg(test)` 恒定成立），只有 `cargo build` 的编译校验，曾用一份开了该选项的库手工跑通 identify + `get_clock` |
| `usb.rs` | 拓扑发现用假 sysfs 树：tty 上溯到 USB 设备、取**紧邻**它的 hub 与端口号（`<hub>.<port>`、根 hub 的 `<bus>-<port>`）、非 USB tty 报错；**多层 hub 取最内层那颗**（外层 hub 与它同型号也不受影响）；开关文件查找：`port<N>` / `<hub>-port<N>` 两种命名、根 hub 的 `<usb>-port<N>`、`probe` 交出该路径；告警里的两条规则（含与 `scripts/klipperx-usb-udev.sh` 同一个 glob——比 hub **深一层**，`include_str!` 对脚本兜底）。**要真硬件的几条没自动化**：hub 端口的供电能力（hub 类描述符低两位：`per-port` / `ganged` / `no power switching`，解析部分用真描述符字节对了；读描述符要能开 hub 节点）、`open_hub` 把**根 hub** 也算进来（`nusb::list_devices` 按设计不给 `usbN`，只能从 `nusb::list_buses` 取；MCU 直插机器 USB 口就是这种），以及控制传输本身；两者都在真板/真 hub 上手工验过（根 hub 收下 `SET/CLEAR_FEATURE(PORT_POWER)`，设备断开重枚举） |

## 写 MCU 相关测试的两个要点

1. **帧要比得完整**：`TestDevice` 比对的是 `Frame`（seq + payload）。请求 payload 可以直接用 `Payload::push_*` 拼，或 `Parser::encode` 得到。
2. **序号对齐**：发送任务每批 +1；接收侧按**块**接受序号，可取「正在等的块」或「下一个块」，也只取低 4 位。假设备（`TestDevice`）让第 i 个响应用序号 `i & 0xf`（与请求同号）即可——它落在「正在等的块」这一侧；同一序号可以连续出现多帧，这正是真实固件的行为（一条响应 + 一帧空载荷 ack，见 `mod.rs::test_acks_and_repeated_sequences_are_accepted`）。
   序号只有 4 位，超过 16 次交换会回绕，忘记 `& 0x0f` 会让第 17 帧起被判为序号不匹配而丢弃（表现为超时）。

`HostDevice` 的往返测试示范了怎么对付一个**阻塞**的 `receive()`：它从独立线程调用并把结果送回 channel，主线程用 `recv_timeout` 给出 5 秒上限——否则一个真出了问题的手感就是测试永久挂住。

握手测试的现成写法见 `identify.rs` 的 `chunked_mappings`：它按块大小生成「请求帧 → 响应帧」映射，并用 `flate2` 现场压缩字典内容。

## 文档同步

改动 `msg` / `mcu` / `cmd` / `event` / `identify` / `api` 的公开 API 或分层职责时，请同时更新本手册对应页面（见 [开发手册首页](README.md) 的目录）；改动 `crates/klippy-api` / `crates/klippy-client` 时同理（它们的公开 API 就是别人依赖的协议）。

`cargo doc --no-deps --lib` 的警告数应与改动前一致（根包目前有 2 条残留于 `frame.rs` / `msg/parser.rs`；`klippy-api` 与 `klippy-client` 是 0 条）。新增模块时注意两个陷阱：

1. **模块的文档链接是在它的 `mod` 声明所在作用域里解析的**，不是在被声明模块自己的作用域里。`klippy/mod.rs` 里的 `pub mod …;` 因此都不带 `///` 文档。
2. **把私有模块提升为 `pub mod` 会激活它的公开文档检查**：模块文档里指向 `pub(crate)` 项的链接会报 `links to private item`。`identify` 从 `mcu` 的子模块提升为顶层公开模块时就遇到这一点，需要把这类链接改成纯代码 span。

---

- [← 开发手册首页](README.md)
- [内部架构 ←](architecture.md)
