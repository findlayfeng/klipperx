---
name: flash-firmware
description: 构建并刷写 Klipper 目标板固件、用调试器（SWD）排障。当要刷板、验证「固件到底刷上没有」、或遇到「刷写假成功 / 读回全是 0xFF」时使用。
---

# Klipper 固件刷写与调试器排障

**本页只记录已经实测过的组合**：**STM32F103ZE + ST-Link + `pyocd` + USB CDC 链路**（主机侧
`/dev/serial/by-id/usb-Klipper_stm32f103xe_…-if00`）。

目标 MCU、调试器/刷写工具、主机链路都**按任务给的设备来定**。遇到本页没覆盖的组合（别的 MCU、
别的调试器、UART 适配器/CAN 链路、根本没有主机链路……）：**先在真机上把它跑通一遍**，再把实测到的
目标名、频率、命令与核对手法补进本节——**不要照抄别处的参数或凭经验猜**。

## 第一原则：调试频率必须保守（实测踩过）

- **频率过高会静默失败**：`pyocd` 照样报 `programmed N bytes`、甚至 `identical M bytes`，但板上跑的
  还是**旧镜像**（本轮据此误判过很久）。
- **从低档起、按设备调整**：先 `frequency=1000000`（1 MHz）跑通，再视需要调高；**换板 / 换 target /
  换线缆 / 换调试器克隆都要重试这个值**。
- **连接即停核**：目标在 sleep（klipper 空闲即 `WFI`）时刷写算法本身不可靠，用 `connect_mode=halt`。
- 必要时**整片擦除**（`-e chip`）：只按扇区擦可能留下旧页。

本仓实测配方（STM32F103ZE + ST-Link；**换板请按上面的原则重调频率**）：

```bash
./venv/bin/pyocd load -O connect_mode=halt -O frequency=1000000 -e chip \
  --target stm32f103ze out/firmware/klipper/klipper.elf
```

## 第二原则：读内存/flash 前必须停核

目标在 sleep 时经 SWD 读**内存与 flash 会全返回 `0xFF`**，看起来像「片子是空的 / 被擦干净了」，
而代码其实在跑。先 `halt` 再读：

```bash
printf 'halt\nread32 0x1FFFF7E8\nread32 0x08000000\nreset\nquit\n' | \
  ./venv/bin/pyocd commander -O connect_mode=halt --target stm32f103ze
```

- `0x1FFFF7E8`：**STM32F1** 的 96 位 chip UID（12 字节，其它 MCU 地址不同，按数据手册取）。
  用它确认「调试器连的是哪块板」——本板固件开了 `USB_SERIAL_NUMBER_CHIPID`，所以
  `/dev/serial/by-id/` 里的串号就是它（实测 `39FFD7054D47323924610951`）。
- `0x08000000` 首字应是向量表初始 SP（本板 `20005000`）；读到 `FFFFFFFF` 先怀疑是不是没 `halt`。

## 第三原则：刷后核对——「刷上没上」要有硬证据

当前链路是 USB CDC，核对按这三步（缺一不可）：

1. **问设备自报版本**（比看刷写工具的输出可靠）：
   - mux 设备：`target/debug/klipperx scan --port <by-id 路径>`——身份由 chipid 派生，**跨复位稳定**
     （实测本板 `e512c60ac0e6`，与用仓库自带 C 版 fasthash64 从 UID 复算一致）；
   - 跑起来的宿主上取 `mcu_version` 与本地构建对照：
     `klipperx api -a /tmp/klippy_uds objects/query '{"objects":{"mcu child":null}}'`
     ↔ `make` 输出里的 `Version: <hash>`（实测刷对时两者一致）。
2. **必要时回读比对**：`halt` 后 `savemem 0x08000000 0x10000 /tmp/board.bin`，与
   `out/firmware/klipper/klipper.bin` 比对。**必须在 halt 态读**；同一会话连读两次都应对得上
   （读不可重复 ⇒ 读法不对，别拿它当证据）。
3. **USB 链路的注意**：设备复位/刷写后会**重新枚举**，tty 号可能变（实测 `ttyACM1`→`ttyACM2`）——
   配置与脚本一律用 **by-id/by-path**，别写死 `/dev/ttyACM0`。`ls /dev/serial/by-id/` 只能证明
   「链路在」，**不能**证明「固件版本已更新」。

## 构建（本仓）

- 固件在**仓库根**构建（一次共享配置出两份）：`make`（klipper + katapult）、`make klipper`、
  `make katapult`；配置用 `make menuconfig`（**仓库不带种子**，工作配置 `.config`；台面板的选项照
  `docs/klippy/user-manual/firmware-build.md` 末节的表选），
  细则见 [构建 MCU 固件](../../../docs/klippy/user-manual/firmware-build.md)。
  产物：`out/firmware/klipper/klipper.{bin,elf}`、`out/firmware/katapult/{katapult,deployer}.{bin,elf}`。
- **改了子模组的 `Makefile`/`Kconfig`/源文件后先 `make clean` 再 `make`**：`src-y` 顺序变化不会让
  `compile_time_request` 的 recipe 重跑，`out/firmware/` 会残留旧序号的产物（曾因此把旧镜像刷进板子）。
- 构建矩阵目标在 `third_party/klipper/test/configs/*.config`：那是**闸门**目标（`cargo test` 的字典构建），
  不是用户侧配置；要让某条固件路径进闸门（例如 USB CDC 的 mux 接线、MCU 的 `mcu_restart()`），就加一份目标。
  用户侧配置走 `make menuconfig`。
- worktree/并行任务的验证构建：仓库根 `make` 本来就只写本检出的 `out/firmware/` 与 `.config`，子模组只读；
  在 worktree 里配 `KLIPPERX_KLIPPER_DIR` / `KLIPPERX_KATAPULT_DIR` 指向主检出即可。

## 排障经验（均来自本板实测）

- **「日志说重启了、板子却没反应」**：先抓链路字节确认命令真的上了线，再查设备侧，别只看宿主一面。
  本板用 `socat` 中继：`socat -x -v pty,raw,echo=0,link=/tmp/relay /dev/ttyACMx,raw,b115200`
  串在宿主库桥与板子之间（实测就是这样看到 `0x05` 单元确实发出、却被打板子的桥半吞掉）。
- **刷完不枚举**：先看**引导模式**（本板是 STM32 的 BOOT0/BOOT1 跳线——曾因跳线配错而根本没跑固件）。
  ROM/DFU 引导只由引脚在复位时选择；**软件只能「写一个挺过复位的标记 + 复位」，且需要板上常驻一个
  bootloader 来消费它**——本板 `FLASH_START_0000` 且无 CanBoot，所以 `REQUEST_BOOTLOADER (0x02)`
  实测就是设计内的 no-op（不是 bug）。
- **收尾**：真机跑完杀掉后台宿主与夹具进程、清理临时文件与 `/tmp` 里的取证文件；
  `kill -9` 夹具会漏掉它 spawn 的 `klipper.elf` 子进程（`pgrep -x klipper.elf` 自查）。
