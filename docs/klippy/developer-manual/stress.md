# 压力测试（`klipperx stress`）

一个跑在板子上的台架工具：给**一块指定的 MCU** 逐步加大步进负载，直到它出错。它不属于主机
运行时，也不需要一个完整的 `printer.cfg`。

```
klipperx stress [OPTIONS] <CONFIG_FILE> [MCU]
```

- `CONFIG_FILE` 用来取 `[mcu …]`（传输方式）和一个 stepper 的 `step_pin` / `dir_pin`；
- `MCU` 省略或为空即裸 `[mcu]`；`[mcu zboard]` 要写 `zboard`；
- `--rate-step`（默认 `1.25`）是每段相对上一段的倍数，越小包围盒越紧、跑得越久；
- `--stage-seconds`（默认 `0.5`）是每段持续多久。

## 负载是什么

负载选自上游的**步进引擎**（`src/stepper.c`）——MCU 的主要工作就是按 `queue_step` 生成步进脉冲。
工具借用配置里某个 `[stepper_*]` / `[manual_stepper]` 的 step/dir 引脚，用 `ConfigBuilder`
配置**一个自己的 stepper**（不是那个 section 的 stepper），然后按段加大步频。

每段的目标是**一段持续、均匀的步频**：

| 量 | 取法 |
|---|---|
| 间隔 | `interval = round(clock_freq / rate)` 个 tick（最少 1） |
| 每条命令 | `count = round(actual_rate * SLICE_SECONDS)` 步（约 10 ms 一条，上限 `u16`） |
| 命令条数 | `round(stage_seconds * clock_freq / (count*interval))`，封顶 `MOVE_SLOTS`（64） |
| 收尾 | 等本段跑完（时长 + 余量），再 `get_config` 问固件还活着吗 |

步频从 `START_RATE`（10 kHz）乘 `--rate-step` 一路升到 `MAX_RATE`（20 MHz）；高段是任何 MCU 都
做不到的，所以正常会停在某一段。**结果是一对包围盒**：最后一个活下来的步频、和第一个挂掉的
步频（工具报告的是实际步频 `freq/interval`，不是目标值）。

两个正确性前提：

1. **每段前 `reset_step_clock` 重锚到「现在 + 1 ms」**，否则下一段第一步排在“过去”，直接触发
   下面的错误；
2. **重锚前先等步进器真的停下**（轮询 `stepper_get_position` 到两次读数相同）。固件在还有 move
   未执行时**拒绝** `reset_step_clock`：`shutdown("Can't reset time when stepper active")`
   （`src/stepper.c:311`）——如果不先等，这个错误会被误当成速率上限。

发送时一段的命令分批发（每批 `SEND_BATCH` 条，之间 `flush`）：主机出站通道只有 32 格
（`mcu/mod.rs`），一段 50 条命令直接灌会报 `no available capacity`。分批只给发送节奏，不影响固件
看到的步进时刻。

## 出错长什么样

三种都由固件 `shutdown`，工具把原因（`shutdown` / `is_shutdown` 事件的 `static_string_id`，经字典
解出）一并报出：

- **步进的时间落在过去** → `Stepper too far in past`（`src/stepper.c:108`）——最典型的一种，
  说明 MCU 已经跟不上下一个步进时刻；
- **新定时器的时间已经在过去** → `Timer too close`（`src/sched.c:94`）；
- **排进 move queue 的 move 超过它的容量** → `Move queue overflow`（`src/basecmd.c:90`）。

报告形如「在第 X 步频 shut down（原因 …），上一段活到 Y 步频」。

### 实测

一块 STM32F103（`stm32f103xe`，72 MHz，Klipper 固件，`config_stepper` 可用），`--rate-step 1.1`：

```
    339623 steps/s (interval 212 ticks, 50x3396 = 169800 steps over 500 ms): queued
    375000 steps/s (interval 192 ticks, 50x3750 = 187500 steps over 500 ms): queued
  firmware SHUT DOWN at 375000 steps/s: Stepper too far in past
  last rate it survived: 339623 steps/s
```

即真值落在 **339 623 – 375 000 步/秒**之间（10.4% 的包围盒）；这块板大约 34 万步/秒就能稳定跑。

## 它会怎么对待板子

这是一次**接管**：`ConfigBuilder` 的握手会给一块跑着别的配置（或已 shutdown）的板子发
`config_reset`，**没有 `config_reset` 的固件则发 `reset` 并重连**（`ResetRequired` 路径，与
`McuObject` 一致），然后配置上这个压力 stepper。测试结束时板子通常停在 shutdown 状态，下一次
运行（或正常的主机）会重新配置它。

## 要求与缺口

- 配置里必须有 `[mcu …]`（或 `[mcu]`）和一个带 `step_pin`/`dir_pin` 的 stepper section；没有就
  直接报错。
- 引脚名支持 `PA0`、`mcu:PA0`、`<chip>:PA0` 和尾随 `!`（忽略）；**别名（`[board_pins]`）还没
  解析**。
- 压力 stepper 用 `invert_step = 0`、`step_pulse_ticks = 0`；`[stepper_*]` 的 `invert_step` /
  `microsteps` / `enable_pin` 等还没有读取（那是 C1 的 stepper 资源）。
- 夹具每次 reset + reconnect（无 `config_reset` 的固件）约 0.5 s。
- 端到端只在真板上手工跑过；单测覆盖的是段计算、引脚解析与命令编码。

TODO 里记着这些剩余项（**S1**）。

---

- [← 开发手册首页](README.md)
- [测试](testing.md) · [MCU 协议与数据字典](mcu-protocol.md)
