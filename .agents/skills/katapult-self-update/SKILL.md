---
name: katapult-self-update
description: 不用编程器给板子更新/开发 katapult 自身：deployer 自举的构建配置、刷前静态核验、flash 自举、读回验证与排障。当要迭代开发 katapult、板上已装 katapult 而调试器不可用、或需要验证「新 bootloader 到底刷上没有」时使用。
---

# Katapult 自举更新（deployer 自更新）

> 2026-10-16 在 **STM32F103ZE + USB CDC** 真机上走通的一次完整自举
> （构建 → 静态核验 → `flash` 刷 deployer → 双复位 → 三段读回全对上），**同日第二轮换血测试**
> 进一步真执行了 deployer 写引导区循环（新镜像 serial `devloop1` 活体 + 读回 `boot==新 katapult.bin`）。
> 协议层禁止直接自写，唯一正路是 deployer；本页记录「怎么把它做对、怎么证明做对了」。

## 第一原则：直接自写被协议封死，只走 deployer

- `third_party/katapult/src/flashcmd.c:90-92`：`command_write_block` 拒绝
  `block_address < CONFIG_LAUNCH_APP_ADDRESS`——**引导区从协议层写不进去**（读不受限，
  `command_read_block` 无地址下限 ⇒ 读回是免费的硬证据）。
- 官方通道 **deployer**（`README.md`「Katapult Deployer」、`src/deployer.c`）：把内嵌新
  katapult 镜像的 `deployer.bin` **当应用**经现有 katapult 刷入；它上电 100ms 后裸
  `flash_write_block` 把镜像写到引导区，然后 `try_request_canboot()`（写 RAM 签名 +
  复位）弹回 katapult。
- 配置开关是 `src/stm32/Kconfig:234-236` 的 choice **「Build Katapult deployment application」**
  （成员 `STM32_FLASH_START_*`，**8KiB 引导 ⇒ 选 `STM32_FLASH_START_2000`**）；
  `BUILD_DEPLOYER` 仅在 `FLASH_APPLICATION_ADDRESS != FLASH_BOOT_ADDRESS` 时自动开
  （`src/Kconfig:141-144`）——两边相等（如默认 0000）就**没有** `deployer.bin`。

## 符号表（改 .config 前先对齐语义，别被名字骗）

| 符号 | 语义 | 本机取值 |
|---|---|---|
| `FLASH_BOOT_ADDRESS` / `FLASH_START` | katapult 自身落位 **且 deployer 写入目标**（`deployer.c` 的 `CONFIG_FLASH_START`） | 0x08000000 |
| `FLASH_APPLICATION_ADDRESS` | **deployer 的链接基址**（`armcm_deployer.lds.S:14` ORIGIN；`LAUNCH > APP` 时先垫 `LAUNCH-APP` 再放代码 ⇒ bin 内容实际落在 LAUNCH，objcopy 剥前导空隙） | 0x08002000（选 2000 后） |
| `LAUNCH_APP_ADDRESS`（choice `STM32_APP_START_*`） | **CONNECT 自报的 start**（`flashcmd.c:26`）+ 写保护下限 + katapult 跳应用的地址 | 0x08002000 |

**唯一要改的一行**（实测）：`.config` 里 `CONFIG_STM32_FLASH_START_0000=y` →
`CONFIG_STM32_FLASH_START_2000=y`（choice 有无条件 prompt，`olddefconfig` 不会重算掉），
然后 `make olddefconfig && make`。

## 自举四步（本机实测命令）

### 1. 构建

```sh
cd third_party/katapult
# 按上表核对/修改 .config 后：
make olddefconfig
make        # 产出 out/katapult.bin（=新引导镜像）与 out/deployer.bin（内嵌它）
```

### 2. 刷前静态核验（不过不刷；没恢复舱时尤其）

- `katapult.bin` 尺寸 ≤ 应用偏移（4304 ≤ 8192）；
- `readelf -l`：`katapult.elf` LOAD `VMA=0x08000000`、`deployer.elf` 第一 LOAD `VMA=0x08002000`
  （必须等于 CONNECT 自报 start，否则首块地址对不上）；
- SP/reset 合理性：`katapult.bin[0]`=RAM 栈顶、`[4]`=0x08000xxx；deployer 同理落在其区间；
- **内嵌检查**：`deployer.bin` 里能找到与 `katapult.bin` **逐字节相等**的整段（payload 由
  `katapult_payload.o` 链入，实测 offset 0x3ac 找到 4304 B）。

### 3. 刷前读回（只读、零风险的终极保险）

用 `REQUEST_BLOCK(0x14)` 把 0x08000000 起 `len(katapult.bin)` 字节读回与 `out/katapult.bin`
比对——**True 才代表「板上在跑的 == 我们这份构建」**，deployer 写入即零变化彩排（首跑必做）。
应答布局（踩过坑）：payload = `orig_cmd(4) + addr(4) + data`——**addr 在帧偏移 8、data 从 12 起**
（别拿 4 当 addr）。USB 双缓冲要先 prime：`01 88 90 00 e5 e9 99 03`（CRC 值 0xE9E5，**LE 上线**）。

### 4. 刷 deployer + 验证

```sh
cargo build --bin flash            # 本仓 T4b 的独立 bin（会自动 prime）
target/debug/flash --device /dev/serial/by-id/usb-katapult_stm32f103xe_<chipid>-if00 \
    third_party/katapult/out/deployer.bin
```

预期输出：`connect: … start 0x08002000` → N/N written → `eof: 3 pages` → N/N verify →
`complete: 5252 bytes in 83 blocks` → rc=0。

**刷后有双复位窗口（别慌）**：COMPLETE 后 katapult 跳应用（deployer）→ deployer 内嵌一致
→ 签名+复位弹回 katapult，约两轮重枚举、数秒。第一次 CONNECT 失败就等 2s 重试（实测第 1 次
撞进窗口失败，第 2 次成功）。

**三条读回硬证据（缺一不可的验收）**：
1. CONNECT 存活（start=0x08002000、block=64、mcu 对）；
2. 应用区 0x08002000 == `deployer.bin`（新东西真落板）；
3. 引导区 0x08000000 == `katapult.bin`（bootloader 完好）。

## 掉电与「像不像更新了」两件事

- **deployer 留在应用区是自恢复的**：掉电→katapult 跳 deployer→发现 flash 与内嵌一致→
  `try_request_canboot()`（`armcm_reset.c`：写签名+复位）弹回 katapult——**不会互跳挂死**；
  真正的砖只有「写坏了」（deployer 写失败才进 `halt()`），对应 README 的配置警告。
- **版本串可能不变**：`make-version.sh` 走 `git describe`（`.git` 优先于 `.version`），只改
  `.config` 不产生 `-dirty` ⇒ 新旧镜像同串（本机两版都是 `3b973a4`）。**版本串不是本场景的
  验证依据——以第 4 步的读回比对为准**；真正改了源码后版本才会变（hash/`-dirty`）。

## 恢复舱（按第一性原理备着）

- **UART ROM（BOOT0=1 + UART1 PA9/PA10）**：STM32F103 只有 UART ROM、无 USB DFU；配 USB-UART
  （如 CH340）+ `stm32flash` 可全片重刷，是 README 要求的「备用恢复手段」。
- 应用区需要「无效」以让 katapult 常驻引导时：经协议写一页全 `0xFF`（`flash_write_block`
  写 FF 会先擦页 ⇒ 读回全 FF ⇒ katapult 判应用无效、每次开机都留在引导）。
- 恢复链断了（如 CH340 没接）期间只走 USB、只做能读回验证的操作。

## 排障

| 症状 | 首因 | 处置 |
|---|---|---|
| CONNECT 无应答（USB） | 双缓冲 CDC 未唤醒 | 先发 prime 帧再 CONNECT；`flash` bin 已内置 |
| 读回应答解析错位 | 把 `orig_cmd` 当 `addr` | addr 在帧偏移 8、data 从 12 |
| `deployer.bin` 不存在 | `FLASH_APPLICATION_ADDRESS == FLASH_BOOT_ADDRESS` | choice 选 `STM32_FLASH_START_2000` 后重 build |
| deployer 后设备短暂消失 | 双复位窗口（跳 app→弹回） | 等 2–4s 重试，别连开 |
| 新旧版本分不清 | 只改了 .config，git describe 不变 | 以读回比对为准，不看版本串 |
| deployer 写失败后死机 | `halt()`（写坏场景） | UART ROM 恢复；写前静态核验别跳步 |
