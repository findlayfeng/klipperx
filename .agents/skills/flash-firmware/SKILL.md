---
name: flash-firmware
description: 构建并刷写 Klipper 目标板固件、用调试器（SWD/JTAG）排障。当要刷板、验证「固件到底刷上没有」、或遇到「刷写假成功 / 读回全是 0xFF」时使用。
---

# Klipper 固件刷写与调试器排障

## 先确定三件事（它们决定后面每条命令）

1. **目标是什么**：哪颗 MCU/架构（stm32f1/f4、rp2040、lpc176x、avr…）、片上有没有**常驻 bootloader**
   （CanBoot / USB-HID / DFU 之类）。
2. **用什么刷/调**：ST-Link、J-Link、Black Magic、CMSIS-DAP……工具各异（`pyocd` / `st-flash` /
   J-Link Commander / OpenOCD / `dfu-util` / `picotool` / `avrdude`），但下面**频率与 halt 两条原则通用**。
3. **设备与主机之间是什么链路**（决定「刷后核对」怎么取证）：USB CDC、USB-串口适配器接 UART、CAN、
   host 库，**或者根本没有主机链路**——只靠调试器回读与行为观察。**别假设是 USB。**

## 第一原则：调试时钟/频率必须保守（最容易踩的坑）

- **频率过高会静默失败**：工具照样报 `programmed N bytes`、甚至 `identical M bytes`，但板上跑的还是
  **旧镜像**（实测据此误判过很久）。
- **从最低档起、按设备调整**：先低后高；**换板 / 换 target / 换线缆 / 换调试器克隆都要重试频率**。
  各工具的旋钮：`pyocd -O frequency=<Hz>`（如 `1000000`）、J-Link `-speed`、OpenOCD `adapter speed`。
- **连接即停核**：目标在 sleep（klipper 空闲即 `WFI`）时，刷写算法与大块读都不可靠——
  `pyocd -O connect_mode=halt`（其它工具用各自等价的 halt-on-connect）。
- 必要时**整片擦除**（`pyocd -e chip`）：只按扇区擦可能留下旧页。

本仓实测过的配方（STM32F103ZE + ST-Link，**仅作示例，频率按你的板重调**）：

```bash
./venv/bin/pyocd load -O connect_mode=halt -O frequency=1000000 -e chip \
  --target stm32f103ze third_party/klipper/out/klipper.elf
```

## 第二原则：读内存/flash 前必须停核

任何调试器都一样：目标在 sleep 时经 SWD 读**内存与 flash 会全返回 `0xFF`**，看起来像「片子是空的 /
被擦干净了」，而代码其实在跑。先 `halt` 再读：

```bash
printf 'halt\nread32 0x1FFFF7E8\nread32 0x08000000\nreset\nquit\n' | \
  ./venv/bin/pyocd commander -O connect_mode=halt --target stm32f103ze
```

- 想确认「调试器连的是哪块板」：读该 MCU 的 **chip UID**（地址查数据手册；上例是 STM32F1 的
  `0x1FFFF7E8`，96 位/12 字节），与设备侧自报的身份/UUID 对照。
- `0x08000000` 首字应是向量表初始 SP（`0x2000xxxx` 一类）；读到 `FFFFFFFF` 先怀疑是不是没 `halt`。

## 第三原则：刷后核对——按链路选法，「刷上没上」要有硬证据

**通用一招：问设备自己**（版本/身份），比看刷写工具的输出可靠得多：

- **有主机链路**：
  - 跑起来的宿主上取 `mcu_version`，与本地构建对照（`make` 输出里的 `Version: <hash>`）：
    `klipperx api -a /tmp/klippy_uds objects/query '{"objects":{"mcu child":null}}'`；
  - mux/CAN 设备还会答扫描：`klipperx scan --port <tty>`（USB CDC / UART 适配器）或
    `klipperx scan --interface <can0>`（CAN）。其身份由 chipid 派生，**跨复位稳定**。
- **没有主机链路**：`halt` 后 `savemem <flash 起址> <长度> /tmp/board.bin`，与
  `third_party/klipper/out/klipper.bin` 比对。**必须在 halt 态读**；同一会话连读两次都应对得上
  （读不可重复 ⇒ 读法不对，别拿它当证据）。
- **链路的额外注意（别把链路事实当固件事实）**：
  - **USB CDC**：设备复位/刷写后会**重新枚举**，tty 号可能变（`ttyACM1`→`ttyACM2`）——配置与脚本
    一律用 **by-id/by-path**，别写死 `/dev/ttyACM0`。固件若开了 `USB_SERIAL_NUMBER_CHIPID`，by-id
    里的串号就是 chip UID（这既是「哪块板」也是「固件换过没有」的旁证）。
  - **UART 适配器**：tty 属于适配器，通常不随目标复位改名；对端设备身份只能靠它自己的应答。
  - **CAN**：没有 tty；用 `ip -br link` / `candump` 之类看总线，身份靠扫描应答。
  - 因此 `ls /dev/serial/by-id/` 之类只能证明「链路在」，**不能**证明「固件版本已更新」。

## 构建（本仓）

- 板子的 kconfig 在子模组：`third_party/klipper/.config`；就地构建 `cd third_party/klipper && make`
  → `out/klipper.elf`、`out/klipper.bin`。
- **改了 `Makefile`/`Kconfig`/源文件后先 `rm -rf out` 再 make**：`src-y` 顺序变化不会让
  `compile_time_request` 的 recipe 重跑，`out/` 会残留旧序号的产物（曾因此把旧镜像刷进板子）。
- 构建矩阵目标在 `third_party/klipper/test/configs/*.config`：要让某条固件路径进闸门
  （例如 USB CDC 的 mux 接线、MCU 的 `mcu_restart()`），就加一份目标。
- 只读分支/并行任务的验证构建，把 `KCONFIG_CONFIG`/`OUT` 指到自建目录，别覆盖板子那份 `.config`/`out/`。

## 排障经验

- **「日志说重启了、板子却没反应」**：先抓链路字节确认命令真的上了线，再查设备侧，别只看宿主一面。
  抓法按链路：USB/UART 用 `socat -x -v pty,raw,echo=0,link=/tmp/relay /dev/ttyACMx,raw,b115200` 串在中间；
  CAN 用 `candump`/`cansniffer`。
- **刷完不枚举 / 不进主机链路**：先区分「没刷上」还是「刷上了但进不来」。
  - 没刷上：回到第一原则（频率、整片擦除、停核）并做回读比对；
  - 刷上了但进不来：查**引导模式**（STM32 是 BOOT0/BOOT1 跳线；其它架构另有机制）。ROM/DFU 引导
    通常只由引脚或专用请求选择；**软件只能「写一个挺过复位的标记 + 复位」，且需要板上常驻一个
    bootloader 来消费它**——没有就是设计内的 no-op（不是 bug）。
- 非 USB 链路同理：先确认设备到底在跑哪个固件，别假设它没刷上。
- **收尾**：真机跑完杀掉后台宿主与夹具进程、清理临时文件与 `/tmp` 里的取证文件；
  `kill -9` 夹具会漏掉它 spawn 的 `klipper.elf` 子进程（`pgrep -x klipper.elf` 自查）。
