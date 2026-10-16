# MCU 架构的桥端口后端（`SERIAL_UART` / `uart_backend_*`）

> **这是什么**：2026-10-16 的只读侦察报告（原样留档），为「让 STM32 板子能把硬件 UART 交出来当桥的端口」
> 这个缺口提供可直接拄进任务书的规格。背景：桥端口后端今天**只有 host 库实现**
> （`third_party/klipper/src/host/bridge.c`），`src/Kconfig:200-207` 的注释原文即「An arch selects this
> once it can hand a UART object out as a bridge port」——MCU 架构（stm32）未 select `SERIAL_UART`。
> **状态**：规格已产出；实现**待派**（建议拆分：① linux 架构后端 + 离机端到端 ② stm32 架构后端 + 真机）。
> 非规范文档；实现落地后本文件可删（按 work-log 惯例，删前先分流）。

---

# 原始侦察报告

> 只读侦察。未改任何文件、未执行 git 写操作、未委派。上游对照只读 `third_party/klipper/`。
> 本文所有 `file:line` 均相对仓库根 `/home/fengcheng/project/klipperx/`。凡标注「推测」的是我推断、未在代码里找到直接实现证据。

---

## 0. 结论摘要（先给一句）

- 桥端口后端**只有一个实现**：host 库 `third_party/klipper/src/host/bridge.c`（`CONFIG_BRIDGE_PORT_SERIAL`）。
- 契约面（`uart.h` / `bridge.h` / `uart.c`）是**架构无关、已经写死且不需要改**的；MCU 架构要补的只是「实现 3 个 `uart_backend_*` + 一个 `bridge_port_ops` + 自己的 rx/tx 缓冲」。
- **契约里没有「轮询任务」这一项**——host 靠 host 专属的 `bridge_poll()` 挂在 `irq_wait()` 上；MCU 必须在自己的后端里补一个 `DECL_TASK`。
- `SERIAL_UART` 已经 `default y if BRIDGE_PORT_SERIAL`（`src/Kconfig:204-207`），所以**架构只需 `select BRIDGE_PORT_SERIAL`**，`UART`/`SERIAL_UART` 自动点亮。
- **linux 架构可以端到端离机测**（Q7 = 可），代价是一条与 host 后端同形的 linux 后端（新文件 + 一个新 `test/configs/*.config`），复用 `LinuxFirmware` 夹具 + 现有 linuxprocess 构建流水线。
- `reset` 在 MCU UART 上无 modem 线 ⇒ **`NULL`，`RESET_PORT (0x04)` 静默丢弃**；这是诚实的最小实现。

---

## 1. 后端契约（Q1）

### 1.1 `struct bridge_port_ops` — `src/bridge.h:27-44`

```c
struct bridge_port_ops {
    int (*write)(void *ctx, const uint8_t *data, size_t len); // :33
    int (*reset)(void *ctx);                                  // :42
    void *ctx;                                                // :43
};
```

语义（`bridge.h:29-44` 注释 + 实证调用点）：

- **`write(ctx, data, len)`**：把一个**整 blob**（单元 = 头 + 帧，或单设备路径上的裸帧）交给端口。**必须整块排队**（后端自己缓冲设备暂时收不下的部分），保证端口上单元不交错。返回 0 = 已排队，-1 = 后端自己已上报的错误。
  - 调用点全在 `bridge.c`：`bridge_write_unit`（`src/bridge.c` UNITS 端口）、`bridge_forward_to_port`（legacy/剥头，`:158` 一带）、`bridge_route_unit`（`src/bridge.c:635` 一带）、`bridge_probe_unassigned`（fan-out）、`bridge_admin_unit` 的广播扇出（`src/bridge.c:545` 一带）。
  - **返回值被忽略**：这些调用点都不检查 `write()` 的返回值（`src/bridge.c` 各处以 `p->ops.write(p->ops.ctx, ...)` 直接调用）。错误上报只能靠后端自己 `shutdown()`/stderr。
  - **调用上下文**：全部来自上游 mux 输入路径——`src/mux.c:452 bridge_admin_unit()` 与 `src/mux.c:479 bridge_route_unit()`，即 `console_task`/`mux_input` 的任务上下文。**不是 ISR 上下文**，但**禁止阻塞/忙等**（见 reset 注释，是全端口约定）。
  - **最大 blob**：`MUX_HEADER_BYTES + MESSAGE_MAX`（host 的 tx 缓冲即 `2 * (MUX_HEADER_BYTES + MESSAGE_MAX)`，`src/host/bridge.c:82`）。
- **`reset(ctx)`**：脉冲「端口后面设备的复位线」。**可选，`NULL` = 无复位线，`RESET_PORT` 静默丢弃**（`bridge.h:36-44`；消费点 `src/bridge.c` 的 `MUX_CMD_RESET_PORT` 分支检查 `if (p->ops.reset)`）。返回 0 = 已启动脉冲，-1 = 无 modem 线（如 pty）。**返回值同样被丢弃**；**禁止阻塞/忙等**，必须「现在断言、调度稍后释放」。
- **`ctx`**：端口所属的指针，由 `bridge_add_port(ops, ctx)` 存进端口表；这里就是 `struct uart_s *`。

### 1.2 `struct uart_s` — `src/uart.c:30-33`

```c
struct uart_s {
    uint32_t baud;   // 主机在 config_uart 里要的线速率
    void *ctx;       // 后端放每对象状态，创建时置 NULL
};
```

### 1.3 三个 `uart_backend_*` — `src/uart.h:39,45,50`（声明）/ `src/uart.c:54,89,93`（调用）

| 函数 | 声明 | 何时调 / 语义 |
|------|------|----------------|
| `enum uart_open_result uart_backend_open(struct uart_s *u, uint32_t baud, int slot)` | `uart.h:39` | **配置期每对象一次**，来自 `command_config_uart`（`src/uart.c:54`）。`slot` = 端口在桥上的 index（host 命名 tty 用的那个数）。返回枚举，`uart.c:55-64` 据结果 `shutdown()`（⇒ 配置阶段显式报错）。 |
| `const struct bridge_port_ops *uart_backend_port_ops(struct uart_s *u)` | `uart.h:45` | `command_config_bridge_port` 里 `bridge_add_port(uart_backend_port_ops(u), u)`（`src/uart.c:89`）。返回 u 的 ops（host 返回一份共享表，靠 `ctx` 区分）。 |
| `void uart_backend_set_port(struct uart_s *u, int index)` | `uart.h:50` | `config_bridge_port` 拿到 `bridge_add_port()` 返回的 index 后调用（`src/uart.c:93`）。后端记下 index，用来把该口读到的字节喂给 `bridge_port_input(index, ...)`。 |

枚举 `enum uart_open_result`（`src/uart.h:26-33`）：
`UART_OPEN_OK=0`、`UART_OPEN_NO_DEVICE`（打不开）、`UART_OPEN_BAD_BAUD`（打开但速率设不上）、`UART_OPEN_BUSY`（该槽已有对象）。

> `command_config_uart` 的线上形状：`config_uart oid=%c baud=%u slot=%c`（`src/uart.c:66`）。`baud==0` 直接 `shutdown`（`src/uart.c:52-53`）。
> `command_config_bridge_port`：`config_bridge_port oid=%c uart=%c`（`src/uart.c:96`）。

### 1.4 host 实现逐函数对照 — `src/host/bridge.c`（`CONFIG_BRIDGE_PORT_SERIAL`）

- **静态表**：`bridge_streams[KLIPPER_HOST_PORT_SLOTS]`（4 槽，`src/host/bridge.c:76-91`；`KLIPPER_HOST_PORT_SLOTS=4`，`src/host/klipper_host.h:33`）。每槽：`fd`、`owner`（开它的 uart_s）、`port`（router index）、`tx[2*(HEADER+MAX)]`+`tx_len`、reset 状态机三字段。
- **`uart_backend_open`**（`src/host/bridge.c:143-186`）：槽越界 ⇒ `UART_OPEN_NO_DEVICE`（`:150`）；槽已被 owner 占 ⇒ `UART_OPEN_BUSY`（`:158`）；路径 = `klipper_host_port_path(slot)` 否则 `CONFIG_HOST_BRIDGE_SERIAL_PATH`；`open(O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC)` 失败 ⇒ `NO_DEVICE`；`set_raw_mode`（termios2/BOTHER，`src/host/bridge.c:120-142`）失败 ⇒ `BAD_BAUD`。**全部成功后才写 owner/fd**（`:176-180`）。
- **`uart_backend_port_ops`**（`src/host/bridge.c:~316-320`）：返回共享 `bridge_serial_ops = { .write=bridge_port_write, .reset=bridge_port_reset }`。
- **`write`=`bridge_port_write`**（`src/host/bridge.c:227-236`）：`bridge_stream_for(ctx)` 找槽；`len > 剩余容量` ⇒ -1；`memcpy` 入队；`bridge_flush`。
- **`bridge_flush`**（`src/host/bridge.c:203-225`）：非阻塞 `write` 推队列，`EAGAIN` 留到下次任务；`EINTR` 重试；其它错误清队。
- **`reset`=`bridge_port_reset`**（`src/host/bridge.c:250-283`）：`TIOCMGET` 失败（pty）⇒ -1；`TIOCMSET` 置 `DTR|RTS`；记录 `reset_restore_at = now + 250ms`、`reset_active=1`，返回 0。**在飞脉冲期间重复请求按已完成返回 0**（`:259`，防止把断言态当原态回存）。释放由 `bridge_reset_poll`（`:288-305`）在轮询里做。
- **`uart_backend_set_port`**（`src/host/bridge.c:327-334`）：`bridge_stream_for(u)->port = index`。
- **`bridge_poll`**（`src/host/bridge.c:347-374`）：对每个已开且已 set_port 的槽：`reset_poll` → `flush` → 循环 `read(256)` → `bridge_port_input(port, buf, n)`，直到 `EAGAIN`。

### 1.5 MCU 实现「必须提供什么 / 可以怎么简化」

**必须提供**（否则 `uart.c`/`bridge.c` 链不起来）：`uart_backend_open`、`uart_backend_port_ops`、`uart_backend_set_port` 三个符号；一个 `bridge_port_ops` 实例；一个把收到的字节喂进 `bridge_port_input(index, ...)` 的定时/任务路径。

**可以简化**：
- `ctx` 不必是「槽表 + fd」——MCU 通常只有**一条**端口 UART，`ctx` 可指向固定的后端状态单例。
- `write()` 直接推 UART TX 环形缓冲并开 TXE 中断即可（无需像 host 那样处理 `EAGAIN`/fd）。
- `reset()` = `NULL`（见 Q6）。
- 速率：**不要**用 `CONFIG_SERIAL_BAUD`；用 `uart_backend_open` 收到的 `baud` 在配置期算 BRR/置寄存器。
- 无 `klipper_host_port`/路径概念——`slot` 只用来做一致性检查（见 Q5）。

---

## 2. 桥如何消费一个端口的字节流 / 驱动模型（Q2）

- **`bridge.c` 里没有任何 task / 定时 / 唤醒钩子**。`bridge_port_ops` 里也没有「poll」回调——端口读入的方向是**推模式**：后端把字节 `bridge_port_input(port, data, len)` 推进去（`src/bridge.h:59`）。
- `bridge_port_input` 做缓冲（`p->in[2*MUX_TUNNEL_FRAME_MAX]`）并调 `bridge_port_scan`（`src/bridge.c:411` 一带），把整单元/裸帧拆出、`mux_send` 上行；`scan` 会 `memmove`，**不应在 ISR 里直接调**。
- **上游调用点**：`src/mux.c:452` `bridge_admin_unit(arb, frame, frame_len)`、`src/mux.c:479` `bridge_route_unit(arb, frame, frame_len)`，都在 `mux_input()` 里（即 `console_task` 内）。
- **host 是怎么被驱动的**：`src/host/timer.c:135-138`，`irq_wait()` 末尾 `#if CONFIG_BRIDGE && CONFIG_BRIDGE_PORT_SERIAL → bridge_poll();`。host 的 `irq_wait` 每次睡眠 1ms（`IRQ_WAIT_MSECS`，`src/host/timer.c:25`），`bridge_poll` 就在这 1ms 节拍上轮询非阻塞 fd。**这是 host 专属**，`linux/timer.c` 的 `irq_wait`（`src/linux/timer.c:268-278`）**没有** `bridge_poll`。
- **结论**：MCU/linux 侧**需要自己的轮询任务**。落点建议放**后端文件**（如新 `src/<arch>/bridge_serial.c` 或 `src/generic/bridge_serial_irq.c`）里的 `DECL_TASK`，**不要**塞进 `bridge.c`（它是传输无关的；host 之所以把 `bridge_poll` 放 host 侧正是这个原因）。任务体：把后端 rx 环形缓冲（ISR 填）里的字节批量 `bridge_port_input` 出去，并驱动 tx flush + （若实现）reset 释放。

---

## 3. stm32 现有 UART 资产（Q3）

- **`src/stm32/serial.c:1-116`**（F1/F4 等）：`CONFIG_STM32_SERIAL_USARTx` 编译期**硬绑一个** USART + 引脚；`serial_init` `DECL_INIT`（`:96-116`）用 `CONFIG_SERIAL_BAUD` 算 BRR 并 `armcm_enable_irq(USARTx_IRQHandler, ...)`；`USARTx_IRQHandler`（`:62-78`）RX 调 `serial_rx_byte()`、TX 调 `serial_get_tx_byte()`（都是 console 的全局缓冲）。
- **`src/stm32/stm32f0_serial.c:1-176`**：F0/G0/G4/H7 的同构版本（寄存器名差异），同样 `DECL_INIT` + 单 USART + 绑 `serial_rx_byte`/`serial_get_tx_byte`。
- **`src/generic/serial_irq.c`**：console 侧。全局 `receive_buf[192]`/`transmit_buf[96]`（`:20-25`）、`serial_rx_byte`（`:36`）、`console_task` = `DECL_TASK`（`:136`）、mux 感知的 `mux_send`（`:185`）。**单份全局缓冲，绑定 console 那一个 USART。**
- **`src/generic/usb_cdc.c`**：另一条 console 载体（`usb_bulk_out_task`/`usb_ep0_task` = `DECL_TASK`，`:200,:601`），**不提供空闲 UART**。
- **复用判断（代码证实）**：
  - 上面所有 console 资产都**硬绑一个 USART 且共用一份全局缓冲**，**不能直接拿来做第二个并发 UART**。端口 UART **必须新写**一个后端文件：自己选一个**不同的** USART（Kconfig 选）、自己的 IRQ handler（调本后端的 rx/tx）、自己的缓冲 +1 个 `DECL_TASK`。
  - **冲突点（明确）**：每个 serial 文件通过 `armcm_enable_irq(USARTx_IRQHandler, USARTx_IRQn, ...)` 独占一个 USART 向量；同一 USART **不能既当 console 又当端口**（同一 `USARTx_IRQHandler` 符号、同一组寄存器、同一份缓冲）。选端口 USART 时必须与 console 的 `CONFIG_STM32_SERIAL_USARTx` 不同。
  - F103（STMF103）：有 USART1/2/3。若 console 走 USB CDC（`stm32f103-usb-mux.config`），USART1/2/3 都空闲；若 console 走 USART1（`stm32f103-serial-mux.config`），端口只能用 USART2/3。**推测**：最小落点是 USART2（PA2/PA3，F1 上 AF 7；注意 `src/stm32/serial.c:31-34` 已给 USART2=PA3/PA2 的 AF 定义可抄）。

---

## 4. Kconfig 接线（Q4）

现有依赖链（`third_party/klipper/src/Kconfig`，逐行核实）：

| 符号 | 行 | 关键内容 |
|------|----|----------|
| `config MUX` | 131 | bool |
| `config MUX_SERIAL` | 133-140 | `depends on SERIAL || USBSERIAL`；`depends on HAVE_CHIPID || MACH_HOST || MACH_LINUX`；`select MUX` |
| `config BRIDGE` | 190-193 | `select MUX`；`default y if MUX_SERIAL || MUX_CAN` |
| `config BRIDGE_PORT_SERIAL` | 194 | bool，**无依赖** |
| `config UART` | 199 | bool |
| `config SERIAL_UART` | 204-207 | `select UART`；`default y if BRIDGE_PORT_SERIAL` |

关键事实（代码证实）：**`SERIAL_UART` 已 `default y if BRIDGE_PORT_SERIAL`**（`:207`）⇒ 架构只要 `select BRIDGE_PORT_SERIAL` 就能同时点亮 `SERIAL_UART` + `UART`（`src/Makefile:12` 编 `uart.c`）。host 就是这么挂的：`src/host/Kconfig:57-61 HOST_BRIDGE_SERIAL select BRIDGE_PORT_SERIAL + select MUX_SERIAL`。

**新增一个 arch 需要 select 什么**：一个 arch 自己的符号（放在 arch 的 Kconfig），内容 `select BRIDGE_PORT_SERIAL`（`SERIAL_UART`/`UART` 自动）+ 端口 USART 的选择。并让它 `default n`，由 config 显式打开。

**`src/stm32/Kconfig`**：需要新增（**推测**为最小形态）：
- 一个 `config STM32_BRIDGE_SERIAL`（或直接复用 serial 的 USART 选择再排他），`select BRIDGE_PORT_SERIAL`；
- 端口 USART 的选择项（如 `STM32_BRIDGE_SERIAL_USART2`，PA2/PA3，`depends on` 该 USART 存在的芯片、且与 `CONFIG_STM32_SERIAL_USARTx` 互斥——可用 `depends on !STM32_SERIAL_USART2`，若 console 用 USART1 则天然不冲突）；
- 引脚来自 USART 选择（编译期定），**速率不写死**（来自 `config_uart baud`）。
现有 serial 选择块在 `src/stm32/Kconfig:404-447`（USART1/2/3/… 各 `select SERIAL`），新块应照其形状放同一 `choice` 附近（注意与 `choice "Communication interface"` 的排他关系——**推测**端口 USART 应放在该 choice 之外，否则会与 console 的 USB/SERIAL 选项互斥而选不了）。

**构建闸门需要的 config**：`crates/test-support/build.rs:150-195` 会遍历 `third_party/klipper/test/configs/*.config`、按其中的架构（在默认集合内）逐个 `make` 并收 `.dict`。所以**新增一条路径就必须新增一份 config**，否则这条代码路径不会被编译进闸门。
- 现有 `stm32f103-serial-mux.config`（`CONFIG_MACH_STM32F103 + STM32_SERIAL_USART1 + MUX_SERIAL`）与 `stm32f103-usb-mux.config`（F103 + MUX_SERIAL）**都没有 `CONFIG_BRIDGE_PORT_SERIAL=y`**，**不能直接用**来编端口后端——需要一份新 config（如 `stm32f103-serial-mux-bridge.config`，在 serial-mux 基础上加端口 USART 选择）。
- linux 若做后端，同样需要新 config，如 `linuxprocess-bridge.config`（`CONFIG_MACH_LINUX=y + CONFIG_LINUX_BRIDGE_SERIAL=y`）。注意 `build.rs:177-183` 只把 `linuxprocess`/`linuxprocess-mux` 认作进程夹具；新 config 会被编译并产 `.dict`，但**不会自动变成 `LinuxFirmware` 夹具**——需要 `test-support`/`build.rs` 再暴露一个 elf 路径（推测改动点，约 10 行）。

---

## 5. 槽号 ↔ UART 映射与 baud（Q5）

- **host**：`slot` 的运行期语义 = 主机的端口槽，tty 由 `klipper_host_port(slot, path)` 在库 `init` 前登记（`src/host/bridge.c:160 klipper_host_port_path`；Rust 侧 `mcu/bridge_ports.rs:383-384` 把 `slot: index` 交给 `config_uart`）。
- **MCU 侧没有 host 表**（`src/bridge.c:66-77` 明确：`KLIPPER_HOST_PORT_SLOTS` 约束只在 `CONFIG_MACH_HOST` 下成立，header 仅在该 build 读入）。
- **最小且诚实的映射方案**（建议写进任务书）：
  - 端口 UART **编译期由 Kconfig 固定一处**（一个 USART + 引脚），**没有 slot→USART 表**（多端口表需更大的 Kconfig/结构，先不做）。
  - `uart_backend_open(u, baud, slot)`：只接受 `slot == 0`（第一声明的端口节拿到 index 0，见 `mcu/bridge_ports.rs:383-384` 与 `src/uart.c:89` 的 `bridge_add_port` 顺序）；`slot != 0` ⇒ `UART_OPEN_NO_DEVICE`；端口已被本对象/前一个对象占用 ⇒ `UART_OPEN_BUSY`（用 uart_s 身份判）。
  - **限制**（诚实声明）：MCU 桥**只支持一个下游 UART 端口**，且它必须是第一个声明的端口节；多端口留待后续（需要 slot→USART 编译期表）。
  - **baud 来自 `config_uart`**（`src/uart.c:44-54` 已把 `baud` 传给 `uart_backend_open`）；后端在 `uart_backend_open` 里按 `baud` 算 BRR/使能，**不要用 `CONFIG_SERIAL_BAUD`，不要 `DECL_INIT` 固定速率**。

---

## 6. reset 语义（Q6）

- host `reset` = `TIOCMGET`/`TIOCMSET` 拉 `DTR|RTS` 250ms（`src/host/bridge.c:250-283`）。
- MCU UART **无 DTR/RTS 等 modem 线** ⇒ **`reset = NULL`**：`MUX_CMD_RESET_PORT (0x04)` 的消费点会 `if (p->ops.reset)` 才调，`NULL` 就**静默丢弃**（`src/bridge.c` 的 `MUX_CMD_RESET_PORT` 分支；文档亦如此描述 `serial-mux-protocol.md:270`）。
- 后果：端口节的 **`uart_restart_method`（arduino/cheetah，= 发 `RESET_PORT`）在 MCU 桥上静默无效**（配 `config.md:291` 的说明）。**命名设备**（写了 `uuid:`）不受影响——仍走 `restart_method: admin` = 转发 `RESTART_DEVICE (0x05)`，由下游设备自己重启（`src/bridge.c` 的 `MUX_CMD_RESTART_DEVICE` 分支）。**legacy 下游设备在 MCU 桥上没有线级复位手段**（除非以后加 GPIO 控制的复位脚——那是另一个后端，不是本单元）。
- 建议：任务书里把 `reset=NULL` 写成显式设计决策，并在文档同步时注明该限制（worker 只列清单，main 改文档）。

---

## 7. 可测性：linux 架构能否端到端（Q7）

**结论：可以（可行）**，代价中等，能复用现有夹具。

- **可行性依据（代码证实）**：
  - `MACH_LINUX` 满足 `MUX_SERIAL` 的身份依赖（`src/Kconfig:133-140` 的 `HAVE_CHIPID || MACH_HOST || MACH_LINUX`），且 `src/linux/Kconfig` 已 `config SERIAL default y`；linux 也有自己的任务循环（`DECL_TASK`，如 `src/linux/console.c:266`）。
  - `SERIAL_UART` 只需 `BRIDGE_PORT_SERIAL`（`:207`）。
  - linux 已经会 `openpty` 并把 slave 挂到 `-I` 路径（`src/linux/console.c:95-120 console_setup`），`LinuxFirmware` 夹具靠这个 symlink 判就绪（`crates/test-support/src/linux_firmware.rs:60-130`）。
- **代价 / 要新写**：
  1. 新文件（推测命名 `src/linux/bridge.c`）：实现 `uart_backend_open`/`uart_backend_port_ops`/`uart_backend_set_port` + 一个 `DECL_TASK`（轮询端口 fd）。约 120–180 行，可大量照抄 `src/host/bridge.c`，把 `bridge_poll` 从 `irq_wait` 改成一个 `DECL_TASK`，把 `klipper_host_port`/路径回落换成 linux 自己的槽→路径来源。
  2. **槽→路径来源**（linux 无 host 表）：最省的做法是**环境变量**（如 `KLIPPER_BRIDGE_PORT_<slot>=<path>`，在 `uart_backend_open` 里 `getenv`），或后端自己 `openpty` 并按 `-I` 同款 symlink 暴露。前者改动最小（推测推荐）。
  3. `src/linux/Kconfig` 加 `config LINUX_BRIDGE_SERIAL select BRIDGE_PORT_SERIAL`；`src/linux/Makefile` 加 `src-$(CONFIG_SERIAL_UART) += linux/bridge.c`（或按新符号）。
  4. 新 `test/configs/linuxprocess-bridge.config`；`crates/test-support/build.rs` 暴露第二个 linux elf（沿用 `linuxprocess-mux` 那段模式，`build.rs:196-228`）；可能加一个 `LinuxFirmware` 变体（或在测试里直接 spawn elf 路径）。
- **复用的夹具**：
  - `crates/test-support/src/linux_firmware.rs`（`LinuxFirmware::spawn` / `spawn_named`）——桥进程 = 一份 linux `klipper.elf`，其 console pty 就是「上游」；
  - `crates/test-support/build.rs` 的 config 遍历 + `linuxprocess`/`linuxprocess-mux` 暴露模式（`build.rs:150-228`）；
  - `src/core/klippy/interface/devices/host.rs` 里真库桥端口测试（`test_the_bridge_port_commands_over_the_real_library`、`klipper_host_port` 用法，`:2956+`）——可作端到端协议脚本的模板；
  - `src/flash/fake.rs`（`fake::FakeBootloader`，脚本化的 katapult 应答，用于下游/隧道场景）；
  - `src/flash/bridge_session.rs`（桥会话/端口扇出逻辑，`MAX_BRIDGE_PORTS`）。
  - **说明**：端口 pty 无 modem 线，`TIOCMGET` 会失败 ⇒ linux 桥的 `reset` 也应当是 `NULL`（或返回 -1）——这恰好与 MCU 语义一致，端到端测的是「无 reset 的端口」。
- **限制（诚实）**：linux 后端验证的是「arch 驱动 + 任务循环 + 路由 + 隧道」这条**通用**路径，**不**验证 stm32 的寄存器/波特率/IRQ/引脚——后一部分仍需真机或至少编译闸门。

---

## 8. 回归面、工作量与拆分（Q8）

### 8.1 受影响构建

- **host 库两份必须零变化**：`build.rs:84-118` 编 `host`（plain）与 `host-mux`（`HOST_BRIDGE_SERIAL`）两份 `.so`。只要新改动都落在**新文件 + 新 arch Kconfig/config**、不动 `src/bridge.c` / `src/bridge.h` / `src/uart.c` / `src/uart.h` / `src/host/*` / 公共 `src/Kconfig`（除必要时新增符号），host 两份产物与行为不变（`SERIAL_UART` 的 `default y if BRIDGE_PORT_SERIAL` 已存在，host 早已走到）。
- **stm32 各目标**：现有 `stm32f103-serial-mux` / `stm32f103-usb-mux` **不**打开新符号 ⇒ 不变；新增 `stm32f103-*-bridge.config` 让闸门编到新路径。
- **linux 目标**：现有 `linuxprocess` / `linuxprocess-mux` 不变；新增 `linuxprocess-bridge.config`。

### 8.2 最小可交付切片

先 **TX/RX、`reset=NULL`、单端口、`slot==0`**；多端口、线级复位（GPIO）、CAN 端口后端（`BRIDGE_PORT_CAN`，见 `mcu/bridge_attach.rs:41` 注释，固件侧尚不存在）都不在本切片。

### 8.3 按文件改动面与顺序（建议）

1. **（通用，可选）** 无——`uart.c/bridge.c/uart.h/bridge.h` 无需改；`SERIAL_UART` 已接线。
2. **linux 侧**（建议先做，离机验证）：新 `src/linux/bridge.c`；`src/linux/Makefile` 加源；`src/linux/Kconfig` 加 `LINUX_BRIDGE_SERIAL`；新 `test/configs/linuxprocess-bridge.config`；`crates/test-support/build.rs` 暴露 elf。
3. **stm32 侧**：新 `src/stm32/bridge_serial.c`（或 `generic/` 共享形态）；`src/stm32/Makefile` 加源；`src/stm32/Kconfig` 加端口 USART 选择（select `BRIDGE_PORT_SERIAL`）；新 `test/configs/stm32f103-*-bridge.config`。
4. **文档**（main 代改）：`docs/klippy/user-manual/mcu-connection.md:111,123`、`config.md:237,291`、`docs/klippy/developer-manual/serial-mux-protocol.md:270+`、`third_party/klipper/src/host/README.md:89,97,197`（说明 MCU/linux 端口后端已落地 + reset=NULL 限制）。

### 8.4 最高风险（含最小验证）

1. **rx 缓冲的 ISR/task 竞态**（ISR 填、任务消费）——最小验证：单测/真机用边界帧（含 `MESSAGE_SYNC` 唤醒）灌满 192B 缓冲边界。
2. **速率设错/用了编译期 `CONFIG_SERIAL_BAUD`**——最小验证：`config_uart baud=…` 后回读 BRR 或实测波特率；错误路径应 `UART_OPEN_BAD_BAUD` 触发 `shutdown`。
3. **USART/引脚/IRQ 与 console 冲突**——最小验证：编译期 `depends on !STM32_SERIAL_USARTx` 排他 + 闸门编出一份 console=USB、端口=USART2 的 config（保证两份 IRQ handler 不同符号）。

### 8.5 建议单元拆分

- **单元 ①（linux 后端 + 离机端到端）**：新 `src/linux/bridge.c` + Kconfig/Makefile + `linuxprocess-bridge.config` + test-support 暴露 + 端到端测试（桥=linux 进程、端口=pty、reset=NULL）。独立可验收、可先做。
- **单元 ②（stm32 后端 + 真机/编译闸门）**：新 `src/stm32/bridge_serial.c` + `src/stm32/Kconfig` 端口 USART 选择 + `stm32f103-*-bridge.config`；先用闸门证明「能编出这条路径」，真机留最后。
- 可选 **单元 ③（文档同步）** 由 main 承担（规则 7）。
- ①②**无相互依赖**（一个 linux 文件、一个 stm32 文件、各自 Kconfig/config），可并行；接缝是公共契约（不变）与 `test-support/build.rs`（若两单元都改它，需串行或按行隔离——**推测**：只有单元①改 build.rs，单元②只加 `test/configs` 即可，`build.rs` 自动遍历 → 真正无写冲突）。

---

## 不确定点（未在代码中找到直接证据）

- `src/stm32/Kconfig` 端口 USART 选项的**确切命名与放置**（是否与 `choice "Communication interface"` 排他）——需施工时定；文中给了安全形态建议。
- linux 槽→路径来源用**环境变量**是最省的推测方案，代码里无先例（host 用 `klipper_host_port`；linux 无对应物）。
- `test-support/build.rs` 暴露 linux 桥 elf 的具体成本（约 10 行）为推测，未逐行核对 `LinuxFirmware` 全部辅助函数。
- F103 端口 USART 选 USART2 是推测（PA2/PA3 可用、AF 定义可抄）；真机引脚需按板子确认。
- host 的 `uart_backend_open` 行号 `:143`、ops 表 `:~316-320`、`bridge_poll:347` 已核实；`bridge_route_unit`/`bridge_admin_unit` 在 `bridge.c` 的精确消费行（`MUX_CMD_RESET_PORT` 分支）本文以语义描述，未逐行给号。

## 停止条件声明

- 已完成：8 问逐条作答 + 每条带 `file:line`（代码证实 vs 推测已区分）。第 7 问给出「可 + 代价 + 复用夹具」明确结论；第 8 问给出可直接变任务书的改动清单、验收项、单元拆分与风险。
- 未做（受只读约束）：未写任何代码、未改文件、未执行 git 写操作、未委派、未落任何非指定文件。
- 若需要，下一步可由写手按 §8.3 顺序落地；真机部分按 `flash-firmware` skill 执行。
