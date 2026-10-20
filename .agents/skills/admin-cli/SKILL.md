---
name: admin-cli
description: 用 `klipperx admin` 直接操作板子（扫描/刷写/复位/准备桥）的实操：寻址、命令顺序、失败判据、刷后核验、以及踩过的坑。当要刷固件、复位僵住的板子、给桥开端口、或排查「刷不进/刷错板/识别不到」时使用。
---

# `klipperx admin` 实操

> 政策在 [`AGENTS.md`](../../../AGENTS.md)（刷板纪律、子模组改动先报告）；本页只写**怎么做**与踩过的坑。

## 子命令与寻址

| 子命令 | 寻址方式 |
|--------|----------|
| `scan` | `--interface can0`（宿主直连 CAN）/ `--port`（桥端口）/ 无参走默认 |
| `flash` | **三种**：`--device <tty>`（板子自己答 katapult）、`--interface can0 --uuid <hex>`（宿主直连 CAN 上的 katapult）、`--bridge <tty> --port <i>`（经桥的隧道刷下游） |
| `reset` | `--bridge <tty> [--port <i>]`；`--interface can0 --uuid <hex>`；加 `--bootloader` 把**桥自己**停进 katapult |
| `bridge config` | `--bridge <tty> [--bridge-uuid <hex>]` → 列出可用端口（`uart <i>: … (CAN1 PB8/PB9)`） |
| `bridge port` | `--port <i> --port-nodeid <id>` 开某端口的隧道 |

- **`--bridge-uuid`** 只在桥还没被分配 node id 时才必需（它决定要不要先走 discovery）；已经分配过的桥不写也行。
- **`--port <i>` 的 i 必须从 `bridge config` 读出来**，绝不要猜。今天的 P1 就是「`--port N` 会顺带复位编号小于 N 的端口」——已改成只复位目标口，但 index 猜错仍是刷错板。

## 刷写：顺序与判据

**桥自己（USB CDC）**

```bash
klipperx admin reset --bridge <tty> --bridge-uuid <hex> --bootloader   # 停进 katapult，tty 变成 usb-katapult_…
klipperx admin flash <image> --device /dev/serial/by-id/usb-katapult_…  # katapult 收 64 B 块
```

**桥下游（经 CAN 隧道）**——目标若还在跑 klipper，**必须** `--port-reset bootloader`，默认的 `reset-port`（线脉冲）不会让它进 bootloader：

```bash
klipperx admin flash <image> --bridge <tty> --bridge-uuid <hex> \
  --port <i> --port-nodeid 4 --assign-uuid <目标 uuid> --can-bitrate 1000000 \
  --port-reset bootloader
```

**宿主直连 CAN**：`--interface can0 --uuid <hex> [--nodeid N]`；`-q/--query` 只列未分配节点。

成功的样子：`verify N/N` 逐块 + `complete: <bytes> bytes in <n> blocks`。**没有 `complete` 就等于没刷完**，板子仍在 katapult（安全、可重试）。

## 复位：先 `reset --bridge`，再 `bridge config`

- `bridge config`（及任何 attach）**要求先握手**（`SET_NODEID` + 重确认）——桥不应答就 10 s 超时。
- `reset --bridge` 的路径是「先试 attach，失败退回 `attach_downstream` + 向 admin id `0x3f0` 发 `RESTART_DEVICE`」，**不需要握手**。所以僵住的桥能被它救回；反过来先跑 `bridge config` 只会看到「did not identify」。
- 只有连 admin 单元都送不进去（USB 不再完成事务）才需要真断电。

## 失败判据（今天现场学到的）

| 现象 | 含义 / 下一步 |
|------|---------------|
| `transport took 0 of N bytes within 2s` | 一个字节都没进内核队列 ⇒ 设备不取数据（僵态或端点未武装）。**不要**以为是写超时太小；先断电/reset，本仓的刷写 tty 写是直接 `write`，不过 `poll(POLLOUT)` 闸门 |
| `command 0x11 failed after 5 tries` | `0x11` = `CONNECT`：目标根本不在 katapult（还在跑 klipper）→ `--port-reset bootloader` |
| `serial write … Input/output error (os error 5)` | 旧 fd 已死（板子复位/重新枚举）→ 重新 `reset --bridge` 再连；别复用旧会话 |
| `MCU 'x': firmware shutdown during connect: oids already allocated` | `admin` 会话把固件留在「已配置」态，属正常：klippy 会 config_reset 一次再握手 |
| 压测 `link gave out` 之后 `did not identify` | 直接用 `reset --bridge`（见上），不必断电 |

## 刷后核验（每次都要）

1. `identify` 回来（`MCU 'x' identified: N messages registered`），桥再加一条 `bridge config` 看端口表；
2. 目标会话能起来（`identify` + 配置握手 + 一次 `get_clock`）；
3. `pgrep -x klipper.elf` 自查残留进程（`kill -9` 夹具会漏子进程）；
4. 桥/下游都在时，确认 `stats` 每 5 s 进日志（`stats count=…`）且没有 `Unhandled message stats`。

## 坑

- **压测/饱和后板子不会自愈**：`stress` 打到 `link gave out` 之后要显式复位，别假设它还活着。
- **`--task step` 会真动轴**：需要接电机 + 行程够，bench 空载量不出有意义的上限。
- **子模组刷写工具与固件同源**：刷写入参（block size 64、帧上限 128 B）来自 katapult，改协议要两边一起改。
- **不要并行跑两个 `admin` 打同一块板**：一个 tty 只能有一个占用者，会互相把对方的 fd 弄死（EIO）。
