# 构建 MCU 固件（klipper 与 katapult）

仓库根目录的 `Makefile` + `Kconfig` 是固件的统一构建入口：**选一次目标板，编出两份固件**——
应用 `klipper.bin` 与引导 `katapult.bin`（以及随配置派生的 `deployer.bin`）。
两份固件都用 `third_party/` 下各自子模组**自己的** Makefile 与 Kconfig 构建，根构建只负责
「一份共享配置 + 两个构建目录」的接线；子模组检出始终是只读的（不往里写 `.config`、`out/`
或 `__pycache__`）。

前置：`make` 与目标工具链（STM32/ARM 目标是 `arm-none-eabi-gcc`）；kconfig 解析用仓库根的
`venv/bin/python`（没有就用 `python3`），不需要额外安装 kconfig 工具。

## 一次配置，两份固件

```sh
make                  # 两份都编：klipper.bin + katapult.bin（+ deployer.bin 若配置要求）
make klipper          # 只编应用
make katapult         # 只编引导（含 deployer）
make menuconfig       # 合并菜单：共享目标 + klipper 选项 + katapult 专属项
make olddefconfig     # 只重解配置、派生两份构建配置（不编译）
make clean            # 删 out/firmware
make distclean        # 连工作配置 .config 一起删
make help             # 命令一览
```

产物：

| 文件 | 内容 |
|---|---|
| `out/firmware/klipper/klipper.bin` | 应用镜像（`make klipper` 与 `make` 都产出；RP2040 选 "No bootloader" 时产物是 `klipper.uf2`） |
| `out/firmware/katapult/katapult.bin` | 引导镜像 |
| `out/firmware/katapult/deployer.bin` | 引导自举镜像；**只有**应用基址 ≠ 引导基址时才有（见下） |
| `out/firmware/klipper/klipper.elf`、`out/firmware/katapult/katapult.elf`、`out/firmware/katapult/deployer.elf` | SWD 刷写/排障要用的 ELF（`pyocd load` 用 `.elf`） |
| `out/firmware/klipper.config`、`out/firmware/katapult.config` | 从共享配置派生出的两份构建配置（可当证据看） |
| `out/firmware/katapult/canboot.bin` | katapult 构建自带的旧名副本（兼容上游脚本） |

`out/`、`.config` 都在 `.gitignore` 里——**仓库不带任何 `.config` 种子**（同上游：种子只能对应某一台板）。同一份文件只有一个位置：
`make klipper` / `make katapult` / `make` 产出的路径完全一样。

## 配置文件从哪来

```
（无种子）标板的开关见本文末「参考板的选择」 ──make menuconfig──▶  .config
.config           工作配置（menuconfig 改这个）   ──make / olddefconfig──▶  out/firmware/klipper.config
                                                                      └▶  out/firmware/katapult.config
```

- **新检出没有 `.config`**：`make` 会停下并提示 `run 'make menuconfig'`（不会替你写一份）。照着本文末
  「参考板的选择」那张表在 `menuconfig` 里选一遍即可；换板子也只是换这几项。
- `make` 每次都会重解配置（kconfig 解析很便宜）；工作配置 `.config` 会按 kconfig 的规范形式重写
  （首次会把注释与无效行规范化），而派生出的 `out/firmware/*.config` **只在与上次内容不同时**才改写，
  所以两个子模组的增量构建不受打扰。
- 想推翻重来：`make distclean && make menuconfig`（`distclean` 连 `.config` 一起删）。

共享配置由根 `Kconfig` 描述：目标选择（架构、处理器型号、时钟、通信接口、引导偏移）直接取自
`third_party/klipper/src/Kconfig`（katapult 的 Kconfig 对这些符号用同样的名字和取值），
katapult 专属的引导侧选项在 [`kconfig/katapult.kconfig`](../../../kconfig/katapult.kconfig)。
`scripts/firmware-config.py` 负责把共享答案投影到 katapult 的 Kconfig，并做一致性校验。

## 共享项与各自专属项

| 项 | 归属 |
|---|---|
| 架构、处理器型号、外部晶振、通信接口（USB/串口/CAN 及引脚）、引导偏移、串口波特率、CAN 速率、启动引脚 `INITIAL_PINS`、USB VID/序列号 | **共享**，问一次，两份固件都拿到 |
| `MCU`、`CLOCK_FREQ`、`FLASH_SIZE`、`RAM_*`、`STACK_SIZE`、`HAVE_*` 能力位、katapult 的 `FLASH_START`/`BLOCK_SIZE` | **派生**，各自树的 Kconfig 从上面那些答案算出来 |
| 应用基址 ↔ 引导跳转地址 | **派生**：klipper 的 `FLASH_APPLICATION_ADDRESS` 就是 katapult 的 `LAUNCH_APP_ADDRESS`（STM32 上还映射 `STM32_FLASH_START_*` → `STM32_APP_START_*`） |
| `deployer.bin` | **派生**：应用基址 ≠ 引导基址时 katapult 才构建部署器 |
| USB **设备** ID | **不合并**：klipper 用 `0x614e`，katapult 用 `0x6177`（刷写工具靠后者找引导） |
| katapult 的双击复位 / 按键 / 状态灯、优化等级（katapult 固定 `-Os`） | **引导专属**，在 `make menuconfig` 的 "Katapult bootloader" 菜单里 |

`make` 的配置阶段会打印这一轮的目标、哪项只对一侧生效（例如 USB 构建里 katapult 不提供
`MUX_SERIAL`），以及没有合并的项。

**串口波特率（`SERIAL_BAUD`）有个前提**：这个符号只有开了 `LOW_LEVEL_OPTIONS` 才有 prompt，
否则每次配置都会把它推回默认 250000（`olddefconfig`/`genconfig.py` 都按默认值算）。所以想把
下游设备的控制台编成 500 k（见 [MCU 连接方式 → 桥下游链路的吞吐与余量](mcu-connection.md#桥下游链路的吞吐与余量真机实测)），
要在那份配置里先 `CONFIG_LOW_LEVEL_OPTIONS=y` 再设 `CONFIG_SERIAL_BAUD=500000`。
katapult 侧的 `SERIAL_BAUD` 同样是默认 250 k（同样的 prompt 规则），所以**隧道与引导永远按 250 k 走**，
只有应用可以换速——换速的两步命令见上面那一节。

## 限制与拒绝构建的情形

- **目标型号必须是 katapult 认识的那一颗**：katapult 的架构只有 STM32、LPC176x、RP2040/RP235x
  （共享菜单是 klipper 的，它更大）；架构 / 型号 / 变体 / 通信接口 / 引导偏移这些选择项，
  katapult 必须也能选中同一项。它没有的项（例如 N32G452/G455、STM32F070x6 变体、klipper 新增的
  USART6 接口）会在配置阶段被点名拒绝，而不是回落到第一项编成另一块板。两棵树对同一型号派生的
  `FLASH_SIZE` 也必须一致（不一致时拒绝；例如 STM32F031 的 katapult 按 16 KiB、芯片是 32 KiB，
  STM32F103x6 变体的 katapult 按 64 KiB、芯片是 32 KiB）；`RAM_SIZE` 只影响各自镜像自己的链接预留，
  不一致只打一行 note。
- **STM32 引导偏移受 katapult 的 `STM32_APP_START_*` 约束**，而且那些选项按处理器有条件：
  参考板 STM32F103 只提供 4/8 KiB，F2/F4/H750 才有 16/32 KiB，H7 才有 128 KiB。
  klipper 的偏移菜单更大（2/20/28/34/36/48/64/128+0.5 KiB）；选到 katapult 在当前芯片上不提供的
  偏移（或 "No bootloader"——katapult 本身就是引导，谈不上和它同址）时，配置阶段会列出该芯片
  实际可选的偏移并拒绝构建。4 KiB 槽只有 4096 B，参考板 USB 构建的引导镜像比它大，所以真要
  选 4 KiB 时会被 katapult 自己的尺寸检查拦下（提示增大偏移或减特性）。
- **主频与 `RAM_SIZE` 可以不同**：两份固件各自按自己树的值链接/运行（G431/H723 的默认主频、
  LPC176x 的 RAM 预留都不同），`make` 会打印一行 note；它们不会同时运行，不需要对齐。
- **`make klipper` 只派生应用侧**、不做上面这些 katapult 校验（只需要 klipper 树支持目标），所以
  它适用于 klipper 支持的任何目标；`make`（两份）、`make katapult` 与 `make config` /
  `make olddefconfig` 会派生并校验引导侧。
- **引导镜像必须放得进偏移留出的槽**。katapult 自己的构建会用 `buildbinary.py` 检查
  `LAUNCH_APP_ADDRESS - FLASH_START` 上限（8 KiB 偏移 = 8192 B），超了它会报
  "The Katapult binary is too large…" 并失败；此时换大一点的偏移或减掉引导侧特性。
- **USB CDC 构建里 katapult 不开 mux**：katapult 的 Kconfig 只在串口控制台上提供 `MUX_SERIAL`。
  应用侧照常有 mux + 桥；引导态由 USB 直连（`1d50:6177`）访问。串口构建则两侧都开 mux，
  这正是一块板当桥、经桥把下游刷进引导所需要的（见 [MCU 连接方式](mcu-connection.md)）。
- **USB-CAN 桥（`USBCANBUS`）在 katapult 侧没有支持**：katapult 的 `HAVE_STM32_USBCANBUS`
  没有默认值，这个接口选项在它的树里不可见，所以两份一起编时会在配置阶段被拒；
  只要应用的话用 `make klipper`。

## 参考板的选择（在 `make menuconfig` 里照这张表选）

台面板是 STM32F103ZE，`menuconfig` 里对应这几项（其余项保持默认）：

| 菜单项 | 取值 |
|---|---|
| Micro-controller Architecture | STMicroelectronics STM32 |
| Processor model | STM32F103 |
| Clock reference | 8 MHz crystal |
| Communication interface | USB (on PA11/PA12) |
| Bootloader offset | 8 KiB bootloader（应用在 `0x8002000`） |
| `CONFIG_MUX_SERIAL`（klipperx 自有项） | 开（宿主按 id 寻址、可被 `admin` 扫描） |
| `CONFIG_ENABLE_DOUBLE_RESET`（katapult 侧） | 开 |
| `CONFIG_ENABLE_LED` / `CONFIG_STATUS_LED_PIN` | 开 / `PC13` |

换板子就是换这张表里的项——**没有种子文件可改**（同上游：种子只能对应一台板）。

## 在 worktree 里构建

子模组是只读的，构建产物与配置都写当前检出自己的 `out/firmware/` 与 `.config`，所以
git worktree 里（`third_party/` 是空目录）也能编，只要把两个子模组变量指向主检出——
与测试用的是同一对变量、同一套解析约定：

```sh
KLIPPERX_KLIPPER_DIR=<主检出>/third_party/klipper \
KLIPPERX_KATAPULT_DIR=<主检出>/third_party/katapult \
make
```

同一份子模组源码可以被多个 worktree 同时构建：每个 worktree 有自己的 `out/firmware/`，
构建不往子模组里写任何东西。`scripts/firmware-config.py` 也接受 `--root/--klipper/--katapult/--config/--out-dir`，
可在不跑 `make` 的情况下单独重解配置。

## 与测试构建的关系

`cargo test` 的 MCU 字典构建（`crates/test-support/build.rs`）**直接**对 `third_party/klipper`
发起 `make`，用 `test/configs/*.config` 作为目标，并把 `OUT`/`KCONFIG_CONFIG` 外移到 Cargo 的
`OUT_DIR`。它不经过根 `Makefile`，也不读写仓库根的 `.config`/`out/firmware/`；两者可以并行，
互不影响。根构建只面向「开发者自己的板子」。

## 在卡片机（RK3328 / RK3566 / CM4 这一级）上构建

宿主的 `cargo build --release` 与上面板子固件的 `make` 是两件事：这里编的是**主机二进制**
（`klipperx`），固件仍然是 `make`。卡片机上它慢，慢在 CPU 与内存，不在盘。

**实测（3.9 GB / 4×A53 的 RK3566，增量一次 30 个 crate）**：`12m57s`，≈26 s/crate。
参考对照（32×Zen4）：`klipperx` 自己编一次 **19 s、峰值 RSS 约 2.5 GB**。也就是说这个 crate
在 A53 上大致 3–5 分钟，而 `icu_normalizer`（`url → idna` 带进来的 ICU4X 数据）、
`rustls`/`rustls-webpki`/`ring`、`minijinja`、`ratatui` 各是几十秒级。

### 先定位，别猜

```bash
cargo build --release --timings        # target/cargo-timings/*.html，看哪条独占、并行度在哪塌
touch src/lib.rs  && /usr/bin/time -f "lib %e s %M kB" cargo build --release -p klipperx --lib
touch src/main.rs && /usr/bin/time -f "bin+link %e s %M kB" cargo build --release -p klipperx --bin klipperx
vmstat 1                               # si/so 非零 = 在换页（在 TF 上换页等于自杀）；wa 高 = 在等盘
CARGO_LOG=cargo::core::compiler::fingerprint=info cargo build --release 2>&1 | grep -i dirty
```

判读：`klipperx` 这一段吃 **2.5 GB**，4 核上 `cargo` 默认 `-j4` 同时编几个大 crate 就会超 3.9 GB →
一旦 `vmstat` 的 `si/so` 非零，进度条「不动」其实是 swap 落在慢盘上。`build.rs` 负责嵌入资产
（Mainsail + config/docs），它对 `assets/mainsail` 与那两棵树做了 `rerun-if-changed`，慢盘上每次
构建都要 `stat` 几千个文件——所以 build script 段异常长也可能是盘。

### 三个提速手段（按性价比）

1. **交叉编译（最有效）**：卡上只跑二进制。**用仓库里的脚本**，别手拼环境变量
   （`ring` 的 C/asm 走的是 `CC_<triple>`，cargo 走的是 `CARGO_TARGET_<TRIPLE>_LINKER`，
   两套名字容易漏）：
   ```bash
   scripts/build-target.sh                    # aarch64 + x86_64，glibc
   scripts/build-target.sh --musl             # 同样两个，静态 musl（推荐给卡片机）
   scripts/build-target.sh --all --dry-run    # 先看它会跑什么、缺什么
   ```
   产物落在 `out/cross/<triple>/`（二进制 + sha256 + `file` 判定），不碰 `target/`。
   缺 rust target 或交叉 gcc 时脚本会跳过并打印该装的包。

   **`gnu` 还是 `musl` 比架构更重要**：`*-gnu` 链的是**构建机**的 glibc，在新发行版上编出来的
   二进制到 Debian 12（glibc 2.36）会直接 `GLIBC_2.38 not found`。脚本会给每个 gnu 产物打印
   它要求的最低 `GLIBC_x.y`，拿去和卡上 `ldd --version` 对。`*-musl` 是静态的、哪都能跑，
   代价是**静态 musl 不能 `dlopen`**，所以 `host_library:`（`[mcu]` 用 klipper 主机库）在那个
   构建里不可用；USB/串口/CAN/TUI/Mainsail 辅助/URL include 都是纯系统调用，照常工作。

2. **限并发 + 换 target 目录**：`CARGO_BUILD_JOBS=2`（别让 rustc 把内存吃满换页），
   `CARGO_TARGET_DIR=/mnt/ssd/kx-target`（USB SSD/eMMC 都行；`/dev/shm` 在 3.9 GB 机器上放不下全量）。
3. **少写产物**：`[profile.release] strip = "symbols"` 已在仓库里——release 二进制从 26.1 MB
   降到 **20.8 MB**（`klippy` 18.4 MB），链接阶段少写 5.3 MB，运行期无影响。**不要开 LTO**：
   默认就是关的，开了在 A53 上是几十分钟的量级，我们这点负载换不回来。

### 这个项目的依赖里，谁需要 C 编译器

只有 **`ring`**（`ureq → rustls → ring`，TLS）。`flate2` 用纯 Rust 后端（`miniz_oxide`），
没有 zlib；`openssl` 已经完全不在依赖树里（`cargo tree -i openssl-sys` 应为空）。所以卡上只需要
`gcc`，不需要 `libssl-dev`/`pkg-config`。
