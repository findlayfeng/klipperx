# 运行时编排（机器与 API）

主机用**两个 runtime**：机器一个，API 一个。机器那个是专用多线程 runtime，由一条专用 OS 线程
驱动；API 那个是进程本来的多线程 runtime，跑 accept 循环、每连接任务和 attachment。本文说明
为什么这么分、谁跑在哪边、边界上有什么约定。（对应 TODO **A3**，已落地。）

## 为什么分

A1 把机器从 runtime 里解耦出来：`Printer` 持 `Arc<dyn Reactor>`，从不命名 tokio，谁建机器谁
决定什么驱动时间（`reactor.rs:23-30`）。A1b 的串行 dispatcher 又保证了**同一 reactor 的定时
回调之间**一次只跑一个。

这两件事都管不到一条边界：机器与 API 若共享一个多线程 runtime，API 的每连接任务、MCU 收发
任务、reactor 的 dispatcher 会被调度到同一批 worker 上。打印的硬实时在 MCU，主机只是软实时；
一旦 toolhead / trapq 这类运动状态进了定时回调，API 侧的一件慢活（一个客户端的重请求、一次
`get_status` 里的耗时对象）就可能占住 worker、推迟机器回调。

目标不是「实时」，而是**隔离**：机器的时间线不应被客户端流量拖慢。选**多线程**机器 runtime
而不是单线程，是因为机器自己的重活（MCU 收帧、CRC、identify 解压、host_library）不应挤在
dispatcher 前面——那只是把抖动从 API 换成了机器内部。

## 两个 runtime

| | 机器 runtime | API runtime |
|---|---|---|
| 形态 | `new_multi_thread`，`worker_threads(2)` | `new_multi_thread`，默认 worker 数 |
| 线程名 | `klippy-mcu`（worker），`klippy-machine`（驱动线程） | `klippy-api` |
| 驱动 | 一条专用 OS 线程 `block_on(klippy_process(...))` | 主线程 `block_on(…)` |
| 建在哪 | `src/klippy.rs` 的 `machine_runtime` | `src/klippy.rs` 的 `api_runtime` |

**机器侧**（全在机器 runtime 上）：reactor 的 dispatcher、MCU 发送/接收任务、设备的阻塞 I/O
（机器 runtime 的 blocking pool）、`bring_up` / `load_config` / `reset_for_restart` / `teardown`、
restart 循环、以及同步阻塞的 `printer.run()`（机器 runtime 的 `spawn_blocking`）。

**API 侧**（全在 API runtime 上）：`api::register`、`Server::bind` 与 accept 循环、每连接任务、
`Attachment`（`--tui` 的 in-process server）、以及 `ctrl_c` 监听。

`block_on` 不能嵌套，所以机器 runtime 不能由 API runtime 驱动，必须有自己的驱动线程
（`src/klippy.rs` 的 `machine_runtime.block_on(klippy_process(…))`）。驱动线程全程停在 `block_on`，这没问题——机器的任务跑在那个 runtime
的 worker 上，不在驱动线程上。

## 边界上的约定

- **reactor 显式建在机器 handle 上**：`TokioReactor::new(machine_handle)`（`src/klippy.rs`，
  在 `machine_runtime` 之后），reactor 不再问 ambient runtime。
- **机器侧的 spawn 一律走存下来的 handle**：`Interface` 存 `handle`（设备 I/O 走 `off_runtime`），
  `Mcu` 从 `interface.handle()` 取一份存字段（收发任务走它），`restart.rs` 的 `spawn_blocking`
  显式收 `&Handle`。机器侧没有裸 `tokio::spawn` / `spawn_blocking`。
- **唯一的 ambient 捕获点**：`Interface::with_transport`（`interface/mod.rs`）的
  `Handle::current()`。它在设备**打开之后**捕获，所以打不开的传输根本不需要 runtime；而
  `load_config` 跑在 `machine_handle.enter()` 之下（`src/klippy.rs`，在 `run()` 内），所以连它捕获到的也是
  机器 handle。这条约定由单测守着（`klippy.rs` 的
  `test_a_transport_captures_the_machine_runtime_not_the_ambient_one`）。
- **跨 runtime 只靠 `Arc<Printer>` 与 `request_exit`**：退出信号是 `Mutex` + `Condvar`
  （`printer.rs` 的 `wait_for_exit`），从任何线程 / 任务调用都安全；端点读机器状态（`status_of`
  等）也都是同步、线程安全的。不共享 runtime，也不跨 runtime 传任务。

## 停机顺序

停机顺序是固定的，且必须保证 `teardown` 早于机器 runtime 的 drop：

1. `request_exit`（来自 API 侧的 `ctrl_c` 监听、attachment 结束，或机器自身的停机条件）；
2. `printer.run()` 从 `Condvar` 醒来、返回，`klippy_process` 的循环结束；
3. `printer.teardown()`（`src/klippy.rs`，`klippy_process` 返回后的第一句）——它会丢掉配置装载的部件、关掉设备，释放停在
   blocking read 上的设备线程；这一步必须在建它的机器 runtime 还活着时做；
4. 机器线程的 `block_on` 返回，`machine_runtime` 被 drop；
5. API 侧 `machine_thread.join()` 返回，abort `ctrl_c` 监听与 API server，`run()` 返回。

重启（`restart` / `firmware_restart`）不换 runtime、不换线程：`klippy_process` 在**同一个**
机器 runtime 上 `reset_for_restart` → **从磁盘重读配置文件** → `load_config` → 再 `bring_up`，
所以 `Arc<Printer>`、endpoints 和 attachment 全程有效。

## 与上游的对应

上游是一个主 reactor + 一条主线程，机器与 API 在同一线程上靠 greenlet 交替。我们把它拆成两条
时间线：机器一条（reactor + MCU），API 一条。上游那种「一个线程」的确定性由 A1b 的串行
dispatcher 在**机器 runtime 内部**保留；两个 runtime 之间只靠同步原语通信，不共享状态机。

## 还没有的 / 未定

- **worker 数**：机器 runtime 固定 `worker_threads(2)`，是个估计（接收任务、发送/控制各一），
  没有实测过。真出现 worker 不够或浪费，再调。
- **机器 runtime 抽象**：`Interface::handle()` / `Mcu::handle()` 现在直接露
  `tokio::runtime::Handle`。若将来想把机器类型与 tokio 彻底解耦，可以换成一个小 trait，但不急。
- **验收尚未量化**：能确认两个 runtime（线程名可分辨）、能干净停机。机器侧现在有 reactor 的
  延迟度量（A1b）可以把“慢回调”量化，但“慢端点不再推迟机器时间线”还只是结构上的保证，
  没有专门对着它跑过的延迟数据；要不要为此做 CPU 绑核见 [延迟与抖动](latency.md)。

## 非目标

- 不动 `Reactor` trait 与 `TokioReactor` 的定时器语义（那是 A1 / A1b 的事）。
- 不引入实时调度、不承诺硬实时——硬实时在 MCU。

---

- [← 开发手册首页](README.md)
- [时钟与定时器 ←](reactor.md)
