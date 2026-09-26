# 最小打印任务 · 真实测试待测列表

本文件只放**必须真实硬件才能做的测试**：按「一次最小打印任务」的执行顺序，排成一张待测清单；
每项写清它做的**真实测试**是什么、怎么算过。host 单测 + 上游语料（字典驱动假 MCU）已覆盖的
行为不重复列——这里只留单测测不到的**电气、物理量级、真机时序、接线拓扑**。清单**不阻塞**
`TODO.md` 的开发任务，约定同 [`AGENTS.md`](AGENTS.md)。

**「最小打印任务」的定义**：一块 STM32F103 板 + 下面的最小接线，从上电连接到打完一个
20×20 mm 单壁方框（单层）再安全收尾——连接 → 归零 → 预热 → 逐行喂 gcode 打印 →
关加热、断使能、退出。**待测列表全绿 = 能完整跑一个最小打印任务。**

## 前置（硬件与运行方式）

- **板子**：STM32F103xe（72 MHz，USB `usb-Klipper_stm32f103xe_39FFD7054D47323924610951-if00`
  → `/dev/ttyACM0`），固件 `third_party/klipper/out/klipper.bin`（含 `config_stepper` 等）。
- **最小接线**：X（`PB0`/`PB1`，`config.cfg` 里只有注释）与 Y 的 `step_pin`/`dir_pin`、
  X 的 `endstop_pin`、挤出机步进（E 轴）、加热棒 + 热敏电阻（`[extruder]`）。
  **目前只确认过 X 的引脚**，其余未知——写 config 前逐个确认，Z/热床可选。
- **喂 gcode 的方式**：`virtual_sdcard` 没有文件回放（无 `M20`–`M27`），标准的
  「上传 → 开打 → 看进度」工作流尚不存在。最小任务用**客户端 g-code 模式逐行发**
  （`Ctrl+G`）或 `gcode/script` 端点顶替。
- **跑起来**：

  ```sh
  cargo run --release -- config.cfg -a /tmp/klippy_uds   # 起宿主（连真板）
  klipperx console -a /tmp/klippy_uds                    # 另开窗口；^G 进 g-code 模式
  ```

## 待测列表（按一次打印的顺序）

- [ ] **R1 上电连接与配置下发** —— 真实测试：插板起宿主，走完 identify、字典下发、
  `get_config`/配置 CRC 与板端 `finalize_config`、时钟同步；`STATUS`/`M115` 有应答，
  日志无 `!!`。可加跑真机帧序用例：`KLIPPERX_HW_SERIAL=/dev/ttyACM0 cargo test -p klipperx
  --lib test_frame_sequence_sync_against_a_real_board -- --ignored --nocapture`。
  判定：会话建立、配置被固件接受、无重传/超时报错。

- [ ] **R2 软复位与会话恢复** —— 真实测试：`FIRMWARE_RESTART`（`restart_method: command`）
  连续做 3 轮，每轮重新 identify + 配置 + 订阅。判定：板子每次都能重启回来，
  API 订阅与 g-code 通道不断（主机层已留档，见下），不出现串口打不开或设备节点漂移。

- [ ] **R3 端停电平与极性（`M119`）** —— 真实测试：X endstop 在**断开 / 手动短接**两种
  物理状态下各读一次 `M119`（或 `query_endstops/status`）。判定：`open` ↔ `TRIGGERED`
  随真实电平翻转，`!`/`^` 极性与上拉符合配置。这是 R4 回零的前置。

- [ ] **R4 归零（`G28`）** —— 真实测试：`homing_speed` 先设小值（如 5 mm/s），
  `G28 X`（接线齐后再 `G28`全轴）。判定：向 `position_endstop` 方向移动、触碰即停、
  回抽后二次回零（`homing_retract_dist`）；`homed_axes` 含 `x`、`position` 落在
  `position_endstop`（`max_error` 内），不撞机不越界。

- [ ] **R5 运动对账（`G1` + `M400`）** —— 真实测试：三轴各走一段已知距离（如
  `G1 X10` / `G1 X0 Y10` / `G1 Z1`），`M400` 后读 `stepper_get_position`。判定：
  电机方向/距离与指令一致，固件步数 = 距离 / `step_dist`（单轴 500 步对账已留档，
  见下；三轴待接线）。单轴冒烟可用
  `cargo run --release -- stress <config.cfg> mcu --task motion` 顶替。

- [ ] **R6 温度链路与加热闭环（`M104`）** —— 真实测试：挤出机加热棒 + 热敏电阻上电，
  `temperature_sensor`/`extruder` 的 ADC 读数与接触温度计对照；`M104 S150` 后看温度
  爬升并稳定、`M104 S0` 后回落。判定：读数误差可解释（同量级、单调）、PID 收敛不
  发散、`verify_heater` 不误触发。**注意**：`M109`/`M190` 目前不等温、`M105` 回
  `T:0`、`TEMPERATURE_WAIT` 未注册——预热完成只能靠外部读数判断（缺口见 `TODO.md` H1）。

- [ ] **R7 挤出机进料（E 轴）** —— 真实测试：热态下 `G1 E10`（配合 `M83`）送料，
  量实际挤出长度；冷态再发同一条。判定：进料量与 `rotation_distance`/齿轮比对账；
  低于 `min_extrude_temp` 时被 `Extrude below minimum temp` 拒绝（保护生效）。

- [ ] **R8 最小任务回放（单壁方框）** —— 真实测试：用 g-code 模式（或循环调
  `gcode/script`）把一段 20×20 mm 单壁方框的 gcode **逐行喂完**（含归零、预热到位后的
  打印段与 `M400` 收尾）。判定：全程无 `!!` 错误、行间流控不卡不丢、
  `objects/query toolhead` 的 `position` 走完全部路径、打完的方框形状/尺寸与指令一致
  （进度只能靠日志与位置核对，`print_stats` 缺失）。

- [ ] **R9 打印收尾与静止** —— 真实测试：结尾发 `M104 S0`、`M18`（或等 `idle_timeout`）。
  判定：加热器停在关断值、电机失能、`last_stats`/`stats` 正常上报、宿主保持连接仍能
  接受命令、无残留加热。

- [ ] **R10 急停安全（打印中 `M112`）** —— 真实测试：**再打一遍并在打印中**发 `M112`
  （TUI 里 `Esc`×3）。判定：立即急停——加热器断电（`shutdown_value`）、步进停止、
  宿主报错退出且退出码非零；`FIRMWARE_RESTART` 后能恢复并可继续跑任务。
  这是唯一一条**必须在带加热的真实任务里**验的安全路径。

- [ ] **R11 连续运行的真机时序** —— 真实测试：把 R1–R9 的最小任务**循环跑满一段较长
  时间**（分钟到小时级，或用 `stress --task step` 顶着上限压）。判定：无
  `Stepper too far in past` / `Timer too close`、无丢步、USB 不掉线；`minclock` /
  `send_wait_ack` 等上游时序原语真机上尚未建模（见 README「离真能打印还缺什么」），
  本项正是要暴露它们。

## 已完成真机验证（留档）

| 项 | 结果 |
|---|---|
| FW5 完整压缩 `--task motion`（单轴） | ✅ 5 mm/10 mm/s、`step_dist=0.01`：500 步压成 3 条命令，`stepper_get_position` 读回 500 |
| `klipperx stress --task step` | ✅ STM32F103 存活到 ~339 623 步/秒，375 000 步/秒时 `Stepper too far in past` |
| F6 SPI（W25 flash，CS=PA15，SPI1 重映射 PB3/PB4/PB5） | ✅ 硬件 `spi1a` 与软件 bit-bang 都读出 JEDEC `ef 30 13`、状态 `0x00`、地址 0x00 数据 |
| `last_stats`（MCU `stats` 上报） | ✅ 真板确认 |
| `--logfile`、`error_exit` 非零、重启后订阅不断 | ✅（主机层，代码 + 真板启动路径） |
| `[output_pin]` 数字/PWM、`SET_PIN` | ✅ 真板端到端（F3/F4 索引） |
| `temperature_mcu`（MCU 内置温度） | ✅ STM32F103 `[temperature_sensor mcu_temp] sensor_type: temperature_mcu` 读数约 **35.1 °C**（校准 `base=357.558 / slope=-767.442`，`<mcu>:ADC_TEMPERATURE`） |

## 范围外的真板项（不属最小打印任务，未排期）

- **双板时序/漂移**：第二块板（`[mcu zboard]`）跨 MCU 停轴、小时级运行的实际晶振漂移与
  USB/CAN 抖动（软件侧已用 ±100 ppm/1 h 模拟；`SecondarySync` 周期重校准）。
- **`rpi_usb` 物理复位**：per-port 断电重启、换电后 identify 不误判、拔插（`restart_method: rpi_usb`，
  代码与决策逻辑已就绪）。
- **未接的外设**：buttons / pulse_counter / trigger_analog、sdcard、LCD、neopixel/dotstar、
  `sensor_bulk` 与各类 SPI/I2C 传感器、tmcuart——各接一个真实外设后验。
- **回零精度数字**：`homing_retract_dist` 二次回零与 `endstop_phase` 相位调节的**实际精度**
  （功能已软件落地，真板只差精度测量）。

## 怎么算清零

每项做到「按上面判定通过 → 把结果（命令、关键日志、数字）写回留档表」即可。真板项
**不阻止**任何 `TODO.md` 任务从「待办」移入「已完成（留档）」。历史清单与更细的旧编号
（T1–T7）见 `git log -- TESTING.md`。
