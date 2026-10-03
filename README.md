> ⚠️ **本项目尚未进入可用状态：请不要把它接到真实打印机上，也不要用于任何生产用途。**
> 它目前是一个**移植工程**，验收标准是「跑通上游 Klipper 的回归语料」，而不是「能打印」。
> 离「一份真实打印任务」还缺输入回放、温度等待、运行时参数等一批东西——详见下面的
> [当前状态](#当前状态) 与 [离「真能打印」还缺什么](#离真能打印还缺什么)。
>
> ⚠️ **本项目绝大部分代码、测试与手册由 AI 编码工具生成**，由人类负责方向、拆解与验收；
> 它**没有经过逐行的人工审计**，也没有经过真实硬件的系统性验证。请把它当成「一份可执行的
> 移植草案」：复用之前自己审一遍，尤其是加热、急停、运动这些安全相关路径。

# KlipperX

**Klipper 宿主（`klippy`）的 Rust 重写。** 不引 FFI、不移植固件：MCU 侧仍然跑**上游 Klipper
编译出来的固件**，本项目实现的是**宿主侧的全部逻辑**——MCU 通信协议（identify/字典下发/配置
CRC/时钟同步/序号与重传）、运动规划与步进压缩、G-Code 调度、`[extras]` 各模块、以及给客户端
用的 API（unix socket）与终端客户端。

判断「做得对不对」的唯一标准是**上游语料**：`third_party/klipper/test/klippy/*.test` 里每个
用例都被当成一次真实运行来跑（字典驱动的假 MCU，走真实的 identify、配置与 g-code 路径），
而不是拿本项目的实现互相印证。设计取舍、与上游的差异、已知缺口都逐条写进文档，不做静默降级。

- 上游基线：`third_party/klipper/`（git 子模组，钉住一份上游版本；上游源码、`config/` 与语料
  都从这一份读，不拿机器上别的 klipper 检出当参照）
- 许可：GPL-3.0（派生自 [Klipper](https://github.com/Klipper3d/klipper)，Kevin O'Connor 等）

## 关于「谁写的」：AI 生成声明

**本仓库的代码、测试与手册绝大部分由 AI 编码工具生成。** 这里的日常开发形态就是「人类下判断 +
AI 写实现」：人类负责方向与取舍、任务拆解、验收判定、把结论写回台账；逐行实现（含调试、补测、
写文档草稿）主要交给 AI 代理完成。分工与纪律（一任务一分支、执行者不改文档、结论必须带实测
证据等）写在 [`AGENTS.md`](AGENTS.md)。

这对读代码的人意味着：

- **不要以「已经有人在生产环境跑过」为前提读它。** 真机接触只有 [`TESTING.md`](TESTING.md)
  里列出的几项冒烟（单轴运动与位置读回、SPI flash、`stats`、`output_pin`、`mcu_temp`）。
- **验收靠可执行仲裁，而不是人的逐行审阅。** 唯一硬标准是**上游 Klipper 的回归语料**：每个
  用例都被当成一次真实运行来跑（字典驱动的假 MCU，走真实 identify/配置/g-code 路径）。所以
  「语料全绿」的含义是「在这些用例上的行为与上游一致」，**不包括用例没覆盖到的路径**；仓库里
  每个模块文档都带自己的「已知缺口」清单，正是为了让这句话可核对。
- **安全相关路径请重点复核**：加热控制与急停（`M112`/`emergency_stop`）、归零与探针触发
  （endstop/trsync）、步进时序与 `minclock` 一类时序原语。这些地方 AI 写出来的东西「看起来对」
  与「真的对」之间的差距，恰恰需要人类工程师的判断。
- **AI 会写错，也会过度自信。** 因此本仓库要求每个结论都附实测证据（提交信息、测试名、真机
  日志、`git log`）；遇到没有证据支撑的说法，**以源码与实跑为准**，不要以文档为准。

## 当前状态

| 项 | 状态 |
|---|---|
| 上游语料（`KLIPPERX_UPSTREAM_ALL=1`） | **239 条声明运行**：其中 **237 条字典齐备、全部通过 / 0 条失败**；另 2 条（`generic-cramps.cfg`、`generic-replicape.cfg`，BeagleBone/PRU 板）因未构建 `pru` 字典而跳过——它们也正是静态缺口报告里**仅剩**的一处（`replicape` 节未实现）。忽略列表**已清空** |
| 单进程闸门 | `cargo test -p klipperx --lib` → **2572 通过 / 0 失败 / 4 忽略**（含全部语料；4 忽略 = `pru` 字典未构建×2、内联 gcode 解析、真机用例；忽略列表守卫是 `KLIPPERX_UPSTREAM_GUARD`，见[环境变量](#环境变量开发与测试)；计数 2026-10-04 实测重核） |
| 真机 | 只有少量冒烟（单轴运动与位置读回、SPI flash、`stats`、`output_pin`、`mcu_temp`）；完整三轴/归零待接线，见 [`TESTING.md`](TESTING.md) |
| `[extras]` 覆盖 | `src/core/klippy/extras/` 104 个模块、117 个 `section!` 声明（装载 id 同数，由 `test_the_factory_table_is_in_load_order` 钉住；2026-10-03 实测重核）；覆盖范围与逐模块说明见[开发手册模块表](docs/klippy/developer-manual/README.md) |
| 输入通道 | API（`-a` unix socket，Moonraker 语义）与终端客户端**可用**；`virtual_sdcard` 的文件回放（`M20`–`M27`）**已实现**，`GCodeIO`（伪 tty）**暂缓** |

### 能做什么

- 起宿主、连真实 MCU、装字典、配置、同步时钟、跑 g-code：`klipperx <config.cfg>`
- 开 API 给客户端（`-a <socket>`），或在本进程里开一个终端窗口：`--tui`
- 独立客户端：`klipperx console`（交互式窗口）/ `klipperx api`（发一条请求）/ `klipperx stress`（压测步进或链路）
- 大部分运动学（cartesian/corexy/corexz/hybrid/polar/delta/generic_cartesian/rotary_delta/winch/deltesian）、探针与调平族、TMC 驱动（UART 与 SPI 六个）、面板四驱动、断料/称重/风扇/加热等

### 离「真能打印」还缺什么

按「打印任务」链路列出（细节与证据见 [`TODO.md`](TODO.md) 与[客户端/开发手册](docs/README.md)）：

1. **输入回放**：`virtual_sdcard` 的文件回放（`M20`–`M27`）、`print_stats` 与
   `pause_resume/{pause,resume,cancel}` 三个端点都已落地（`api/endpoints/pause_resume.rs`）；
   剩下 `sdcard_loop` 的 `SDCARD_LOOP_*` 与文件回放未接。
2. **温度语义**：`M105` 的 gcode_id 表已接线（`heaters.rs:417` 的 `gcode_id_to_sensor`，
   没有登记传感器时才回 `T:0`）；`M190`/`M109` 按等待版本注册（`extruder.rs:263`、
   `heater_bed.rs:53`），会等到温度到位；`TEMPERATURE_WAIT` 亦可等待（2026-10-03，`b5da84e`）。
3. **运行时参数**：`M204`/`SET_VELOCITY_LIMIT` 已注册（2026-10-04），改完**同时**进状态与
   规划器；`SET_PRESSURE_ADVANCE` 会记录但**不作用于运动**（PA 只存值，运动层未消费）。
4. **常见宏依赖**：`save_variables`（`SAVE_VARIABLE`）已落地（2026-10-04），宏可读写
   `printer.save_variables.variables`；仍缺 `G2/G3`（`gcode_arcs`）与 `M600`。
5. **真机时序**：`minclock` / `send_wait_ack` 等上游时序原语未建模，真机上尚未验证（见 [`TESTING.md`](TESTING.md)）。
6. **硬件侧**：三轴 step/dir 与 endstop 接线、挤出机与加热器接线、以及按接线写一份 config。

## 快速上手

```sh
# 取上游子模组（上游源码、`config/` 与回归语料都在里面；克隆时带了 --recurse-submodules 可跳过）
git submodule update --init third_party/klipper

# 构建（宿主二进制 klipperx 与 klippy）
cargo build --release

# 单进程跑全部单测 + 上游语料（推荐日常用这一条）
cargo test -p klipperx --lib

# 全部 crate
cargo test --workspace

# 只看语料：每个用例打一行（约 1 分钟）
cargo test -p klipperx --lib upstream_test_cases_run -- --nocapture

# 跑宿主：连配置里写的 MCU，并起 API / 开窗口
cargo run --release -- config.cfg                 # 只起宿主
cargo run --release -- config.cfg -a /tmp/klippy_uds
cargo run --release -- config.cfg --tui           # 本进程内开一个终端客户端窗口
klipperx console -a /tmp/klippy_uds               # 另开终端连上去

# 真机（需要一块板子；见 TESTING.md；这个变量没有默认值，必须给出）
KLIPPERX_HW_SERIAL=/dev/ttyACM0 cargo test -p klipperx --lib \
  test_frame_sequence_sync_against_a_real_board -- --ignored --nocapture
```

真机 / 外设验证清单见 [`TESTING.md`](TESTING.md)。

## 环境变量（开发与测试）

多数变量只影响构建与测试；跑宿主本身只用到 `RUST_LOG`（可选再加 `KLIPPERX_TRACE`）。
「默认」一栏就是不设置时的行为。

| 变量 | 默认 | 作用 |
|---|---|---|
| `KLIPPERX_KLIPPER_DIR` | `<仓库>/third_party/klipper` | 语料、kconfig 片段与 host 库从哪个 klipper 检出读。**git worktree 里没有子模组，靠它指向主检出的那一份**。构建期与运行期读同一个值，改指向会触发重建。 |
| `KLIPPERX_ARCHES` | `linux,avr,stm32,atsam,atsamd,lpc176x,rpxxxx,hc32f460` | 逗号分隔的 MCU 家族列表：构建阶段只为这些家族的 `test/configs/*.config` 生成 `.dict`（各需其工具链：主机 `cc`、`avr-gcc`、`arm-none-eabi-gcc`）；其余运行因缺字典跳过。 |
| `KLIPPERX_ALL_ARCHES` | 未设置 | 设成任意值即忽略上面的列表，构建 `test/configs/` 下的**全部**目标（含 `pru`、`ar100`、`simu`，需要各自的工具链）。 |
| `KLIPPERX_UPSTREAM_ALL` | 未设置 | 设成任意值即取消语料忽略列表（当前列表为空，故这只是口径上的保险；字典未构建的运行仍然跑不起来）。 |
| `KLIPPERX_UPSTREAM_GUARD` | 未设置 | 设成任意值则额外跑「忽略列表守卫」：以 load-only（不建连接、不跑 g-code）方式重跑忽略用例，某个文件的所有运行都能装载，守卫就报失败，提示该核实并从忽略列表里删掉。 |
| `KLIPPERX_UPSTREAM_VERBOSE` | 未设置 | 设成任意值后每个语料用例打一行 `VERBOSE <耗时> ok/fail <用例>`。 |
| `KLIPPERX_UPSTREAM_FILTER` | 未设置 | 只跑名字匹配的语料用例（迭代加速用；**不设置 = 237 全量原样**，默认路径零变化）。 |
| `KLIPPERX_KEEP_GATES` | 未设置 | 设成任意值后，`test:` fake 传输也**不放行**两道时钟闸——复现「闸×语料」交互用（如 iqex 的 `Invalid sequence` 配方），默认 fake 放行。 |
| `KLIPPERX_TRACE` | 未设置（关闭） | 非 `0` 值打开诊断 trace：模拟器与 MCU 拆除打的 `SIM-DIAG: …` / `MCU DROP …` 行（追挂起用，平时是噪声）。 |
| `KLIPPERX_HW_SERIAL` | 未设置 | 真机用例要用的串口路径。以 `--ignored` 显式请求而没设它时用例**失败**；不带 `--ignored` 不执行。 |
| `KLIPPERX_USB_IDS` | `1d50:614e 1d50:606f` | `scripts/klipperx-usb-udev.sh` 按空格分隔的 `vendor:product` 找 Klipper 设备；自编固件改了 USB id 时用。 |
| `RUST_LOG` | `info` | 宿主与客户端的日志过滤器；与 `--verbose` 取更详细者（`--verbose` 保底 `debug`），解析不了则忽略。 |

`KLIPPER_HOST_LIB` 与 `KLIPPERX_TEST_DICTS` 不在此列：由 `crates/test-support/build.rs` 通过
`cargo:rustc-env` 写死给测试读，不是给人设的。

## 仓库布局

```
src/                     宿主（klippy）实现
  klippy.rs              主机入口：起 printer、装配置、跑 API 与运行循环
  main.rs                命令行（klipperx / klippy：宿主、console、api、stress）
  core/klippy/           协议与运行时
    mcu/                 MCU 连接、字典、资源（pin/pwm/stepper/endstop/trsync/i2c/spi…）
    motion/              规划器、步进压缩、trapq、运动学
    extras/              各配置节（fan/heater/tmc/display/probe/bed_mesh/load_cell… 100+ 文件）
    config/ gcode.rs pins.rs printer.rs load.rs  配置、G-Code 调度、引脚、对象注册表
    upstream.rs          上游语料 harness（字典驱动假 MCU，验收的标准）
  api/                   unix socket API（Moonraker 语义）与端点
crates/
  klippy-api/            客户端用的 API 协议（编码、传输、地址）
  klippy-client/         客户端：`api` / `console`（TUI）/ TUI 窗口
  test-support/          测试支撑：按架构构建 MCU 字典、语料目录解析
docs/                    文档（见下）
third_party/klipper/     上游 Klipper（git 子模组：语料 + 上游 config + 固件源码）
```

## 文档

| 文档 | 内容 |
|---|---|
| [`AGENTS.md`](AGENTS.md) | **开工前先读**：分支/提交约定、worker 纪律、文档同步义务 |
| [`docs/README.md`](docs/README.md) | 文档总索引 |
| [用户手册](docs/klippy/user-manual/README.md) | 配置文件参考、G-Code 命令、客户端使用、日志与调试 |
| [开发手册](docs/klippy/developer-manual/README.md) | 模块架构、消息协议、运动层、测试与回归口径 |
| [第三方开发手册](docs/klippy/third-party-dev/README.md) | Klippy API 接口定义（端点状态表） |
| [`TODO.md`](TODO.md) | 当前待办、工单与账本（含语料进度） |
| [`TESTING.md`](TESTING.md) | 真板/外设验证清单（不阻塞主线） |
| [`docs/work-log/`](docs/work-log/README.md) | 一次性分析记录（非规范，完结即清理） |

## 参与约定（摘要）

- 一个任务一条分支，从 `work` 切出；并行任务用独立 worktree；**不操作任何远程仓库**。
- 合入 `work`、清理 worktree、代写文档由主代理（main）负责；执行者只在自己的分支上提交。
- 改动落地后**必须**在同一批改动里同步受影响的手册（对照表见 `AGENTS.md`）。
- 上游语料是仲裁：验收标准是「对应 `.test` 从忽略列表移除后通过」，不是「某个报错不再出现」。
