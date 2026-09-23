# klippy 运行机制

klippy 是**单线程事件循环**上的一个对象图：主线程运行 reactor，reactor 上挂回调与定时器；
`Printer` 持有对象注册表与事件表；配置装载把每个 section 变成一个对象；对象之间通过事件总线
协作、通过 MCU 对象与固件通信；g-code 输入经调度器分派给处理器，运动数据经规划、trapq、步进
生成与压缩下发到 MCU。停机与重启都是这个对象图上的状态转换。

本页描述的是**上游 klippy 与本项目共有的运行机制**，以上游实现为参照给出出处。凡是本项目与
上游不同的地方，用「与上游的差异」单独标注。各环节的深入说明见对应专题页；本仓库的模块
划分见[开发手册首页](README.md)的「分层结构」。

## 进程与主循环

```
伪代码：进程主循环

main():
    start_args = 解析命令行()
    循环:
        reactor = 新建 Reactor()
        printer = 新建 Printer(reactor, 日志器, start_args)
        result  = printer.run()
        若 result ∈ {exit, error_exit}: 跳出
        等待 1 秒
        start_args.start_reason = result        # restart / firmware_restart
```

`start_args` 携带 `config_file`、`apiserver`、`start_reason`、`debuginput`、`debugoutput`、
`dictionary`，以及 `software_version`、`cpu_info`、`device`、`linux_version`。
`RESTART` 与 `FIRMWARE_RESTART` 因此不是重启进程，而是换一个新的对象图在同一个进程里继续
运行；`error_exit` 最终以非零码退出进程。出处：`klippy/klippy.py:354-374`。

> **与上游的差异**：`start_args` 中的 `debuginput`、`debugoutput`、`dictionary` 在上游可组成
> 「文件输出 + 数据字典」的**无固件运行模式**。本项目的等价物分两半：`start_args.debug_output`
> 字段与 `Printer::is_fileoutput()` 已就位（T3，回归 harness 在装载前填它，`-o`/`-i` 的命令行入口
> 尚未做），字典则由应答机 `SimulatorDevice` 走真实的 identify 路径下发（不是直接注入），见
> [回归测试](regression-tests.md)。

## 机器状态

```
  构造
   │
   ▼
 startup ──失败(Config / Protocol / MCU / Internal)──▶ error ──RESTART───────────┐
   │                                                                              │
   │ _set_state(ready)                                                            │
   ▼                                                                              │
 ready ──invoke_shutdown──▶ shutdown ──FIRMWARE_RESTART───────────────────────────┤
                                                                                  │
                    主循环下一轮：新 Reactor + 新 Printer ◀──────────────────────┘
```

| 对外类别 | 条件（`klippy/klippy.py:45-55`） |
|----------|----------------------------------|
| `ready` | `state_message` 为 ready |
| `startup` | `state_message` 为 startup |
| `shutdown` | `in_shutdown_state` 为真 |
| `error` | 其余（配置错误等） |

- `_set_state`（`:57`）仅在当前处于 ready 或 startup 时改写消息；
- `invoke_shutdown`（`:204`）置 `in_shutdown_state` 后依次派发 `klippy:shutdown` 与
  `klippy:analyze_shutdown`；
- `update_error_msg`（`:63`）允许消费者在消息未被改写的前提下替换为更详细的文本；
- `request_exit(result)`（`:228`）记录退出结果并结束 reactor，该结果即主循环看到的 `res`。

启动期如果 `start_args` 带 `debuginput`（回归测试的输入文件模式），上游会在非 ready 的新状态上直接
`request_exit('error_exit')`——它以进程退出码判定用例成败（`klippy/klippy.py:57-62`）。
本项目**没有**复刻这条路径：`set_error_state` 只改状态不退进程，回归判定靠 `upstream::run_phases`
的两段返回值（`load_config`/`bring_up` 失败与 g-code 阶段失败分开），见 [回归测试](regression-tests.md)。

## reactor 与回调模型

```
reactor.run()
   │
   ├─ 到期定时器 / 已注册回调 / 异步回调
   │        └─▶ 当前回调（同一时刻仅一个）
   │                 ├─ 返回下次唤醒时间 → 重新排入定时器表
   │                 └─ pause(waketime)  → 让出，循环继续调度其他回调
   └─ end() → 退出循环
```

`register_timer`、`register_callback`、`register_async_callback` 注册回调，`run()` 进入循环，
`end()` 结束（`klippy/reactor.py`）。需要等待的长操作（等 MCU 响应、等定时器）以 greenlet 的
`pause(waketime)` 让出，事件循环据此继续调度其他回调。**同一时刻只有一个回调在运行**，因此
对象图内部不使用锁。

> **与上游的差异**：本项目用 async/await 表达上游的 `pause` / `completion`，
> `Reactor` 只保留「时间 + 定时器」；并且把上游「一个主线程 + greenlet 交替」拆成机器与 API
> 两条 runtime，机器侧的串行 dispatcher 保留了「回调互不重叠」的确定性。见
> [时钟与定时器](reactor.md)与[运行时编排](runtime.md)。

## 启动时序

```
Printer.__init__          _connect（reactor 回调）                 对象图
      │                            │                                  │
      ├─ add_object(gcode)         │                                  │
      ├─ add_object(webhooks)      │                                  │
      ├─ register_callback(_connect)                                  │
      │                            │                                  │
      │                            ├─ _read_config() ─── 建对象图 ───▶ configfile
      │                            │                                  pins / mcu
      │                            │                                  <前缀节>
      │                            │                                  toolhead
      │                            ├─ send_event("klippy:mcu_identify")▶ 字典协商
      │                            ├─ send_event("klippy:connect") ───▶ 解析自身
      │                            │                                  打开设备
      │                            │                                  下发配置
      │                            ├─ _set_state(ready)               │
      │                            └─ send_event("klippy:ready") ─────▶ │
```

任一步失败都按异常类型写入状态；进程随后退出或等待重启。出处：`klippy/klippy.py:128-168`。

## 配置装载与对象模型

```
[stepper_x]         ──▶ extras/stepper.py    : load_config
[output_pin fan]    ──▶ extras/output_pin.py : load_config_prefix
                        （节名首词 = 模块名；对象以节名登记）
                                    │
                                    ▼
                     Printer.objects（有序字典）
   ┌──────────────────────────────────────────────┐
   │ gcode  webhooks   ← 早期对象（__init__ 装入） │
   │ configfile                                    │
   │ pins   mcu        ← [mcu] / [mcu zboard]      │
   │ …      <带子名的节>                            │
   │ toolhead          ← [printer]                 │
   └──────────────────────────────────────────────┘
   get_status(eventtime) → objects/list 只列出实现了它的对象
```

`_read_config`（`klippy/klippy.py:114-127`）按上表顺序建立对象；`load_object`（`:90-112`）
完成节名到模块的映射，重复装载直接返回已有对象。配置项的读取经 `ConfigWrapper` 记录，
`check_unused_options` 据此拒绝未被任何对象读过的 section 或 option——**读取记录即 schema**。
`objects/list` 的过滤见 `klippy/webhooks.py:484`。

> **与上游的差异**：本项目的 section → 对象表在编译期生成（各模块的 `section!` 声明，见
> [声明式表生成](codegen.md)），不依赖运行时按文件导入。配置解析器读取上游全部 259 份 `.cfg`
> （由 [回归测试](regression-tests.md) 覆盖），仅保留一处宽松：引号内的 `#` 不作注释，而上游
> 在第一个 `#` 处截断。`deprecate` / `deprecate_gcode` / `deprecate_mcu_code` / `runtime_warning`
> 与 `configfile` 的 `warnings` 已实现；`autosave` / `SAVE_CONFIG` 尚未实现。

## 事件总线

```
注册：register_event_handler(name, cb) ──┐
                                        ▼
                        Printer.event_handlers
                        ┌───────────────────────────────┐
                        │ "klippy:connect" → [cb, cb2]  │
                        │ "klippy:ready"   → [ … ]      │
                        └───────────────────────────────┘
                                        │ send_event(name, *params)
                                        ▼
                                  cb(...) → cb2(...)   按注册顺序

固件事件：shutdown / stats 帧 ──▶ Parser ──▶ register_response 绑定的回调
```

打印机事件（`klippy:` 前缀）由主机触发：`mcu_identify`、`connect`、`ready`、`shutdown`、
`analyze_shutdown`、`disconnect`、`firmware_restart`、`notify_mcu_error`。固件事件由固件上报，
按响应名绑定（`MCU.register_response`）。`run()` 结束时（`klippy/klippy.py:186-196`）：若结果为
`firmware_restart` 则先派发 `klippy:firmware_restart`，随后一律派发 `klippy:disconnect`。
注册与派发见 `:224`、`:226`；对应关系见[事件系统](event-system.md)。

## MCU 通路

```
主机 MCU 对象                                          固件
      │                                                  │
      │ identify(offset) ───────────────────────────────▶│
      │ ◀──────────────────── identify_response(data)     │
      │  zlib 解压 → 数据字典 → 装入 Parser                 │
      │                                                  │
      │ allocate_oids(count) ────────────────────────────▶│
      │ config…（add_config_cmd 累积）─────────────────────▶│
      │ finalize_config(crc) ────────────────────────────▶│
      │ get_config ──────────────────────────────────────▶│
      │ ◀──────── config(is_config, crc, is_shutdown)      │
      │                                                  │
      │ 运行期：send() 单向；查询命令等待响应               │
      │ ClockSync：print_time ⇄ MCU clock（回归）          │
```

每个 `[mcu]` 对应一个 `MCU` 对象。identify 与数据字典见 [Identify 机制](identify.md) 与
[MCU 协议与数据字典](mcu-protocol.md)；配置握手（`allocate_oids` / `finalize_config` / CRC）
见 [MCU 配置构建](mcu-config.md)。

> **与上游的差异**：identify 的 offset 不匹配在本项目直接报错，不按同一 offset 重试；配置 CRC
> 哈希的是类型化命令的 wire 字节，而非上游的命令文本；CRC 不一致时优先用固件 `reset` 真重启。

## G-Code 与运动数据流

```
g-code 输入通道
      │
      ▼
GCodeDispatch._process_commands        解析命令名与参数，查表分派
      │
      ▼
ToolHead.move()                        Move + LookAheadQueue（规划与前瞻）
      │
      ▼
trapq                                  按 print_time 的梯形段
      │
      ▼
itersolve + kin_*                      每个 stepper 的位置函数
      │
      ▼
stepcompress                           压缩为 queue_step(interval,count,add)
      │
      ▼
MotionQueuing（定时器）                 决定何时 flush
      │                                 （时机依据 ClockSync.estimated_print_time）
      ▼
固件
```

回零走同一份运动数据，但停止由固件触发：

```
G28 ──▶ Homing.home_rails
          ├─ set_position(forcepos)
          ├─ endstop.home_start ──▶ trsync_start + stepper_stop_on_trigger + endstop_home
          ├─ ToolHead.drip_move ──▶ trapq 直灌 + 推进 print_time
          └─ endstop.home_wait  ──▶ 等固件触发
固件：按 rest_ticks 轮询 endstop → 命中 → trsync_do_trigger → 停步并记录位置
主机：按 stepcompress 的 history 反推触发时刻的位置 → set_position(haltpos)
```

> **与上游的差异**：运动数学与步进压缩在本项目以 Rust 重写，上游在 `chelper/` 的 C 里；
> 运动学已有 `none` / `cartesian` / `corexy` / `corexz` / `hybrid_corexy` / `hybrid_corexz`
> （delta 族与 generic_cartesian 待做，C1c），回零目前是**单程**（`homing_retract_dist` /
> `second_homing_speed` 已读入 `HomingInfo` 但二次回零未接，`endstop_phase` 未实现），`G28`
> 按请求的轴**逐个**回（一次一轴，与上游 cartesian 一致）。`GCodeIO` 的输入抽象（伪 tty / 文件 / `stats gcodein`）暂缓；mux 命令的
> 「取值不合法」提示取排序后的第一个候选，上游取字典序最后一个。见
> [延迟与抖动](latency.md)与 `TODO.md`。

## 停机与重启

```
固件 shutdown 帧 ──▶ MCU._handle_shutdown ──▶ Printer.invoke_shutdown ──▶ shutdown 状态

Printer.invoke_shutdown ──▶ send_event("klippy:shutdown")
                                ├─▶ MCU._shutdown ──▶ 固件 emergency_stop
                                └─▶ error_mcu：把简短消息展开为原因 + 提示
                                    （另监听 klippy:analyze_shutdown）

run() 返回 ──▶ res = firmware_restart│restart ──▶ 主循环下一轮
           └─▶ res = exit│error_exit ──▶ 结束（error_exit 非零退出）
```

固件自报停机时 `MCU` 将其转为打印机停机；主机侧停机时 `MCU` 监听 `klippy:shutdown` 向固件发送
`emergency_stop`，使两端同步停下。`error_mcu`（`klippy/extras/error_mcu.py`）负责展开消息。

## 客户端侧

```
客户端 ── 0x03 分帧 JSON ──▶ ServerSocket ──▶ ClientConnection ──▶ _process_request
                                                                    │
                                            端点表 / mux / remote method（注册在 WebHooks）
                                                                    │
                                                          printer / gcode
        ◀──────────────── 应答与推送（连接.send）──────────────────┘
```

API 服务（`klippy/webhooks.py`）运行在同一个 reactor 上：请求按 `0x03` 分帧后在 reactor 回调里
分发到端点；端点与远端方法在 `WebHooks` 上注册，推送经连接的 `send`。`webhooks` 自身是一个
printer object，`info` 与 `objects/list` 端点都建立在它之上。线上形状见
[Klippy API 参考](../third-party-dev/api-reference.md)。

> **与上游的差异**：上游 `webhooks.py` 一个文件同时是传输/派发机器与一个 printer object；本
> 项目把协议与服务端拆到 `crates/klippy-api`，主机侧只保留 `webhooks` 这一面的适配，并将 API
> 服务放在独立的 runtime 上。`info` 端点的 `python_path` 在本项目没有对应解释器（无 Python），
> 仅按上游形状返回。

---

- [← 开发手册首页](README.md)
- [事件系统 ←](event-system.md)
