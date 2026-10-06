# 压力测试（`klipperx stress`）

一个跑在板子上的台架工具：给**一块指定的 MCU** 逐步加大负载，直到它出错。它不属于主机
运行时，也不需要一个完整的 `printer.cfg`。

```
klipperx stress [OPTIONS] <CONFIG_FILE> [MCU]
```

- `CONFIG_FILE` 用来取 `[mcu …]`（传输方式）；`--task step` / `--task motion` 还要该板名下
  带 `step_pin` / `dir_pin` 的 stepper 节（按 `step_pin` 的 chip 归属，同板多个全收；`motion`
  取第一个可用节）；
- `MCU` 省略或为空即裸 `[mcu]`；`[mcu zboard]` 要写 `zboard`；
- `--task`（默认 `step`）选做什么：`step` 压步进生成，`comm` 压主机↔MCU 链路，
  `motion` 跑一遍完整运动链路的真板冒烟；
- `--rate-step`（默认 `1.25`）是每段相对上一段的倍数，越小包围盒越紧、跑得越久；
- `--stage-seconds`（默认 `0.5`）是每段持续多久。

`step` / `comm` 是**升序 ramp + 遇错即停**，结果是一对包围盒（最后一个撑住的 / 第一个挂掉的）；
`motion` 是单次固定动作，通过即 `motion smoke OK`。

## 任务一：步进生成（`--task step`，默认）

负载选自上游的**步进引擎**（`src/stepper.c`）——MCU 的主要工作就是按 `queue_step` 生成步进脉冲。
工具按**参加测试的板**收集配置里全部 `step_pin` 归属该板的 `[stepper_*]` / `[manual_stepper]`
节（归属只看 `step_pin` 的 chip；`enable_pin`/`endstop_pin` 落在别的板不改变归属；本板自己的
`enable_pin` 会被读出并置使能，2026-10-06 起），用 `ConfigBuilder` 在**同一轮配置**里给每节各配
一个自己的压力 stepper（各自 oid——固件
`finalize_config` 后再握手会 `config_reset` 掉先前配置，故不能逐节握手），同板并发驱动、同一
ramp 同步下发，然后按段加大步频。`dir_pin` 与 `step_pin` 不同板的节**按节拒绝**（固件口径
`Stepper dir pin must be on same mcu as step pin`），不拖累同板其它节。

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

1. **每段前 `reset_step_clock` 重锚到「现在 + 提前量」**，提前量由 `anchor_margin(rtt)` 按本次
   `get_clock` **实测往返时延（RTT）**推导：`2×RTT + 1 ms 余量`，下限 `1 ms`（RTT 极小时不比
   早先写死的 1 ms 差），RTT 测量不可用时回退 1 ms。提前量必须盖住「取样到 `reset_step_clock`
   送达固件」的整段链路延迟——慢链路（如 UART 桥）上固定 1 ms 会让锚点到达时已过期，第一档
   就 `Timer too close`；
2. **重锚前先等步进器真的停下**（轮询 `stepper_get_position` 到两次读数相同）。固件在还有 move
   未执行时**拒绝** `reset_step_clock`：`shutdown("Can't reset time when stepper active")`
   （`src/stepper.c:311`）——如果不先等，这个错误会被误当成速率上限。

发送时一段的命令分批发（每批 `SEND_BATCH` 条，之间 `flush`）：主机出站通道容量是
`SEND_QUEUE_CAPACITY`（`mcu/mod.rs`，512 格；同步 `send` 队列满时**有界退避等待**（总 ≤ `SYNC_SEND_WAIT` 1s），
超时才报 `no available capacity`（错误现带命令名与水位）；能等待的路径走 `send_payload`，且剩余 ≤16 格（`SYNC_SEND_HEADROOM`）时让位等排空，给同步发送常备余量）。分批仍保留，只给发送节奏，
不影响固件看到的步进时刻。

### 步进任务的出错形态

三种都由固件 `shutdown`，工具把原因（`shutdown` / `is_shutdown` 事件的 `static_string_id`，经字典
解出）一并报出：

- **步进的时间落在过去** → `Stepper too far in past`（`src/stepper.c:108`）——最典型的一种，
  说明 MCU 已经跟不上下一个步进时刻；
- **新定时器的时间已经在过去** → `Timer too close`（`src/sched.c:94`）；
- **排进 move queue 的 move 超过它的容量** → `Move queue overflow`（`src/basecmd.c:90`）。

报告形如「在第 X 步频 shut down（原因 …），上一段活到 Y 步频」。

### 实测（步进）

一块 STM32F103（`stm32f103xe`，72 MHz，Klipper 固件，`config_stepper` 可用），`--rate-step 1.1`：

```
    339623 steps/s (interval 212 ticks, 50x3396 = 169800 steps over 500 ms): queued
    375000 steps/s (interval 192 ticks, 50x3750 = 187500 steps over 500 ms): queued
  firmware SHUT DOWN at 375000 steps/s: Stepper too far in past
  last rate it survived: 339623 steps/s
```

即真值落在 **339 623 – 375 000 步/秒**之间（10.4% 的包围盒）；这块板大约 34 万步/秒就能稳定跑。

## 任务二：运动链路冒烟（`--task motion`）

把**主机运动链路**在真板上跑一遍：用与 `--task step` 相同的 step/dir 引脚配置一个
stepper，然后

```
Trapq（一段匀速 5 mm 移动）
  → itersolve（stepsolver）
  → stepcompress（完整压缩）
  → queue_step 下发（分批 + flush）
  → stepper_get_position 读回
```

把固件读回的步数与 `距离 / step_dist` 对比，相等则 `motion smoke OK`。它不写 `[printer]`
也不需要三个轴，因此可以在只知道一个轴引脚的板子上验证压缩器；一次 5 mm/10 mm/s 的小移动。

实测（STM32F103，当时的 `step_dist = 0.01`）：500 步被压成 **3 条命令**（1 条 `set_next_step_dir`
+ 2 条 `queue_step`），固件读回正好 `500 step(s)`。（2026-10-06 起 `step_dist` 由节的
`microsteps`/`rotation_distance` 等几何算出，不再是固定的 0.01；这条实测数字是改前的留档。）

## 任务三：命令往返（`--task comm`）

压的是**主机↔MCU 链路**，不是机器：不用配置固件（`get_clock` 是 `HF_IN_SHUTDOWN` 的基础命令），
以 `--rate-step` 从 `COMM_START_RATE`（100 req/s）升到 `COMM_MAX_RATE`（200k req/s），每段按目标
速率发 `--stage-seconds` 秒的 `get_clock`。

- `get_clock` 的响应名是固定的（`clock`），主机不能流水线多个 `call`，所以请求是 fire-and-forget、
  由一个绑在 `clock` 上的回调计数；
- 按墙钟配速：一次循环把「到点该发的」都发出去，追上了就睡一下；出站通道（32 格）满时先 `flush`
  再重试一次，重试还发不出去就是链路给不起这个速率；
- 每段结束 `flush`，再等 50 ms 收响应，比较发出与收到。

失败判据（任一）：固件 shutdown、**积压**（发出远比收到多，超过 `COMM_BACKLOG_LIMIT`）、或**达不到
目标速率**（发送被反压拖住）。这些都是通信侧的症状，与 MCU 算力无关。

### 实测（链路）

同一块 F103（USB CDC，名义 250000 baud；实际是 USB 全速）：

```
      2842 req/s: sent 1420, answered 1410, achieved 2840 req/s, backlog 10
      3553 req/s: sent 1776, answered 1709, achieved 3552 req/s, backlog 67
      4441 req/s: sent 2213, answered 1904, achieved 4426 req/s, backlog 309
  link gave out at 4441 req/s: 309 requests went unanswered
  last rate it carried: 3552 req/s
```

即链路能稳定扛住约 **3.5k 往返/秒**，再高响应就开始积压。注意这与“名义 250000 baud”无关——
USB CDC 走的是 USB 全速，真正的瓶颈在固件的命令处理与响应队列。

## 多机（同时打多块板）

`[MCU]...` 是多值参数：**省略 = 只连裸 `[mcu]`**（与旧单机行为一致），可给一个或多个名字
（重复自动去重）；`--all-mcus` 枚举配置里全部 `[mcu …]` 节（按配置顺序，裸 `[mcu]` 计一个），
与显式名单互斥（同给报错）。

```sh
klipperx stress config.cfg                       # 单板（默认 [mcu]）
klipperx stress config.cfg mcu2                  # 指定一块
klipperx stress config.cfg mcu mcu2 --task step  # 两板并发
klipperx stress config.cfg --all-mcus --task comm
```

每板一路 future 由 `join_all` 并发轮询（板数运行期才知道，等价于逐 future `join!`；单板连接
失败不阻断其它板，记入汇总）。**所有输出行带 `[<mcu>] ` 前缀**（含单板运行），收尾按板汇总：
step 打 `last rate it survived`（两条退出路径都打）、comm 打 `last rate it carried`、到顶打
`no failure up to …`。退出码：任一板**硬错误**（连接/配置失败）→ 1；各板「找到上限」仍算成功。
`--task step` 时每板驱动其名下**全部** stepper 节（按 `step_pin` 归属）；没有 stepper 节的板打
`[<mcu>] no stepper section on this MCU; skipping step task` 跳过（不算硬错误），多 stepper 板的段行
带 `; N steppers` 注记、配置行形如 `configured 2 steppers (oids 0, 1; …)`。

运行时仍是 `worker_threads(2)`（`run()` 处有注释）：阻塞 I/O 走 blocking 池，每板只占
send/recv 两个后台任务——**双路 ramp 的余量未实测**，出现调度延迟再调。

## 它会怎么对待板子

这是一次**接管**：`ConfigBuilder` 的握手会给一块跑着别的配置（或已 shutdown）的板子发
`config_reset`，**没有 `config_reset` 的固件则发 `reset` 并重连**（`ResetRequired` 路径，与
`McuObject` 一致）——重连的顺序是：`reset` 发出并 flush → **15 ms 排空停顿**（`RESET_DISCONNECT_DELAY`，
对齐上游 `klippy/mcu.py` 的 `_disconnect()` 口径）→ **先 drop 旧会话**（中止接收任务、关闭口）→
再重开；不先关旧会话的话，重连循环期间两个读端会抢同一 tty，新会话被饿死直到 identify 超时——
**但 `drop` 只有在 `Arc::strong_count==1` 时才会真正关旧**：多板重构（M1）曾把会话 clone 进 `boards`
横跨 `join_all` 存活，使 `drop` 形同虚设（真机 396 条 `answers block 110` 恒定 + identify 10s 死循环）；
现由 `drive_boards` 把会话所有权下移（`unzip` 后 move 进逐板 future，持有者与 join 同寿命），并在
`run_board` 入口与重连前**断言 `strong_count==1`**（两条新单测，两种回退形态均转红实录）。然后
配置上这个压力 stepper。测试结束时板子通常停在 shutdown 状态，下一次
运行（或正常的主机）会重新配置它。

## 要求与缺口

- 配置里必须有 `[mcu …]`（或 `[mcu]`）；`--task step` 还要该板名下带 `step_pin`/`dir_pin` 的
  stepper section：**显式点名**的板没有 → 直接报错（原文案）；**`--all-mcus` 扫出**的板没有 →
  打跳过行、不改退出码，全部板都没有才整体报错（`--task comm` 不需要 stepper）。
- 引脚名支持 `PA0`、`mcu:PA0`、`<chip>:PA0`；取反是**前导** `!`（`!PA0` / `!mcu:alias`，即 `mcu:!PA0` 是非法描述）；
  **`[board_pins]` 别名已走 F2 的正式装载器解析**（2026-10-06）。
- 压力 stepper **复用真资源（2026-10-06）**：经 `PrinterPins::setup_stepper` 建出正式的 `McuStepper`，
  所以固件 `config_stepper` 带的是该节自己的 `invert_step`（步进脚的前导 `!`）与 `step_pulse_ticks`
  （来自 `step_pulse_duration`，默认 2 µs，上下界同 `extras/stepper.rs`）；`--task motion` 的步距
  由 `microsteps`/`rotation_distance`/`full_steps_per_rotation`/`gear_ratio` 算出（节里没写几何时
  退回 0.01 mm，使只用引脚的节仍可用）；本板自己的 `enable_pin` 会被读出并置于使能（共用线去重）。
- 夹具每次 reset + reconnect（无 `config_reset` 的固件）约 0.5 s。
- 端到端：**假板上的 step ramp / comm ramp / `ResetRequired` 重连已有自动化用例**（2026-10-06，
  用 `test: dict=` 的 `SimulatorDevice`）；真板上只手工跑过单轴冒烟。**双板并发的真板验证项**：
  双 step ramp、双 comm ramp 的调度余量、`--all-mcus` 真实枚举、单板旧命令行回归、
  一板连接失败另一板照常+退出码 1；未覆盖的还有 `--task motion` 的假板 e2e（假件
  `stepper_get_position` 恒答 0，位置断言无法通过）。单测另覆盖段计算、引脚解析、命令编码与
  多机选择/双假设备并发的帧隔离（见 [测试](testing.md)）。

TODO 里记着这些剩余项（**S1**；2026-10-06 后仅剩 `--task motion` 的假板 e2e 与真板双机验证）。

---

- [← 开发手册首页](README.md)
- [测试](testing.md) · [MCU 协议与数据字典](mcu-protocol.md)
