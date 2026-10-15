---
name: flash-firmware
description: 给 Klipper 目标板构建并刷写固件、用 SWD 排障（pyocd/ST-Link）。当要刷板、验证「固件到底刷上没有」、或遇到「读回全是 0xFF / 刷了没生效」时使用。
---

# Klipper 固件刷写与 SWD 排障

## 第一原则：SWD 频率必须保守（这是最容易踩的坑）

`pyocd` 默认频率偏高时，刷写会**静默失败**：它照样报 `programmed N bytes` 甚至
`identical M bytes`，但板上跑的还是旧镜像。实测据此误判过「固件没生效」很久。

- **从 1 MHz 起，按设备调整**：先低后高，越低越稳；低频率稳定后才试更高。
- **换板/换 target/换 ST-Link 都要重试频率**——克隆调试器、线长、MCU 型号都会影响。
- 目标处于 sleep（klipper 空闲即 `WFI`）时，刷写算法本身也不可靠：**连接即停核**。

本仓实测可用的配方（STM32F103ZE + ST-Link，板子由 SWD 供电/调试）：

```bash
./venv/bin/pyocd load -O connect_mode=halt -O frequency=1000000 -e chip \
  --target stm32f103ze third_party/klipper/out/klipper.elf
```

- `-e chip`：整片擦除（只按扇区擦可能留下旧页）。
- `connect_mode=halt`：连接即停核，让刷写算法与大块读都可靠。
- **不要只凭输出的成功字样收工**：按下面「刷后核对」执行。

## 第二原则：读内存/flash 前必须 `halt`

目标在 sleep 时经 SWD 读**内存与 flash 会全返回 `0xFF`**，看起来像「片子是空的 / 被擦干净了」，
而实际上代码在跑。先停核再读：

```bash
printf 'halt\nread32 0x1FFFF7E8\nread32 0x08000000\nreset\nquit\n' | \
  ./venv/bin/pyocd commander -O connect_mode=halt --target stm32f103ze
```

- `0x1FFFF7E8`：STM32F1 的 96 位 UID（12 字节）。可用来核对「调试器连的是不是那台设备」——
  klipper 若开了 `USB_SERIAL_NUMBER_CHIPID`，`/dev/serial/by-id/` 里的串号就是它。
- `0x08000000` 首字应是向量表初始 SP（如 `2000xxxx`）；若是 `FFFFFFFF` 再确认一次是不是没 `halt`。

## 刷后核对（三步，缺一不可）

1. **等重新枚举**：`ls /dev/serial/by-id/`。注意 **tty 号会变**（`ttyACM1`→`ttyACM2`）——
   配置里一律用 **by-id/by-path**，别写死 `/dev/ttyACM0`。
2. **问设备自报版本**（判断「刷上没上」唯一可靠的手段）：
   - mux 设备：`target/debug/klipperx scan --port <by-id 路径>`（身份是 chipid 派生，跨复位稳定）；
   - 跑起来的宿主上：`klipperx api -a /tmp/klippy_uds objects/query '{"objects":{"mcu child":null}}'`
     取 `mcu_version`，与本地 `make` 输出的 `Version: <hash>` 对照。
3. **回读校验**（必要时）：`halt` 后
   `savemem 0x08000000 0x10000 /tmp/board.bin`，与 `third_party/klipper/out/klipper.bin` 比对。
   **必须在 halt 态读**，否则读到的是 0xFF；同一会话连读两次都应对得上（读也不可重复
   就说明读法不对，别拿它当证据）。

## 构建（本仓）

- 板子配置在子模组 `third_party/klipper/.config`；就地构建：
  `cd third_party/klipper && make` → `out/klipper.elf`、`out/klipper.bin`。
- **改了 `Makefile`/`Kconfig`/源文件后先 `rm -rf out` 再 make**：`src-y` 顺序变化不会让
  `compile_time_request` 的 recipe 重跑，`out/` 会残留旧序号的产物（曾因此把旧镜像刷进板子）。
- 构建矩阵目标在 `third_party/klipper/test/configs/*.config`：要让某条固件路径进闸门
  （例如 USB CDC 的 mux 接线、MCU 的 `mcu_restart()`），就加一份目标。
- 只读分支的验证构建请把 `KCONFIG_CONFIG`/`OUT` 指到自建目录，别覆盖板子那份 `.config`/`out/`。

## 排障经验

- **「日志说重启了、板子却没反应」**：先抓链路字节确认命令真的上了线
  （`socat -x -v pty,raw,echo=0,link=/tmp/relay /dev/ttyACMx,raw,b115200` 串在中间），
  再查设备侧；不要只看宿主一面就下结论。
- **刷完不枚举**：先看**引导模式**（BOOT0/BOOT1 跳线）。STM32 的 ROM bootloader 只由引脚在
  复位时选择，软件进不去；软件能做的只是「写一个挺过复位的标记 + 复位」，且需要板上**常驻**
  一个 bootloader（CanBoot / USB-HID）来消费它——没有的话这类请求就是设计内的 no-op。
- **收尾**：真机跑完杀掉后台宿主与夹具进程、清理临时文件与 `/tmp` 里的取证文件；
  `kill -9` 夹具会漏掉它 spawn 的 `klipper.elf` 子进程，用 `pgrep -x klipper.elf` 自查。
