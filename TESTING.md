# 真板测试待办（不在主线任务里）

本文件只放**需要真实 MCU / 外设**才能做的验证。它**不阻塞** `TODO.md` 里的开发任务：
代码/单测验收先行，真板项攒在这里，等有条件的窗口一次性做。

- 现有板子：STM32F103xe（72 MHz，USB `usb-Klipper_stm32f103xe_39FFD7054D47323924610951-if00` →
  `/dev/ttyACM0`），固件 `third_party/klipper/out/klipper.bin`（含 `config_stepper` 等）。
- **已知接线限制**：`config.cfg` 只（注释）给了 X 的 `PB0/PB1`；Y/Z 与 endstop 引脚未知，
  所以完整 `[printer]` 三轴 `G1`/`G28` 需要先确认接线（或只做单轴）。
- 纯软件已覆盖的（见下）不需要真板；只有**物理量级/电气/拓扑**才列在这里。
- 新增的 `debug_read`/`debug_write`/`debug_ping`/`debug_nop` 用**虚拟 MCU**（`FrameMock` + 命令层单测）
  验证，不需要真板；`spi_temperature`（MAX6675/31855/31856/31865 的换算与 `thermocouple_result`
  路由）同样用虚拟 MCU 验证，接真实热电偶时才需真板。

## 0. 快速命令

```sh
# 已能用的真板冒烟（单轴，借用 [stepper_x] 的 step/dir 引脚）
cargo run --bin klipperx -- stress <config.cfg> mcu --task motion
```

## 1. 已完成的真板验证（留档）

| 项 | 结果 |
|---|---|
| FW5f 完整压缩 `--task motion`（单轴） | ✅ 5 mm/10 mm/s、`step_dist=0.01`：500 步压成 3 条命令，`stepper_get_position` 读回 500 |
| `klipperx stress --task step` | ✅ STM32F103 存活到 ~339 623 步/秒，375 000 步/秒时 `Stepper too far in past` |
| F6 SPI（W25 flash，CS=PA15，SPI1 重映射 PB3/PB4/PB5） | ✅ 硬件 `spi1a` 与软件 bit-bang 都读出 JEDEC `ef 30 13`、状态 `0x00`、地址 0x00 数据 |
| `last_stats`（MCU `stats` 上报） | ✅ 真板确认 |
| `--logfile`、`error_exit` 非零、重启后订阅不断 | ✅（主机层，代码 + 真板启动路径） |
| `[output_pin]` 数字/PWM、`SET_PIN` | ✅ 真板端到端（F3/F4 索引） |
| `temperature_mcu`（MCU 内置温度） | ✅ STM32F103 `[temperature_sensor mcu_temp] sensor_type: temperature_mcu` 读数约 **35.1 °C**（校准 `base=357.558 / slope=-767.442`，`<mcu>:ADC_TEMPERATURE`） |

## 2. 待真板验证

### T1. FW5/FW6：三轴 `G1`（需要 X/Y/Z 接线）

- **前置**：确认 Y/Z 的 `step_pin`/`dir_pin` 与 `rotation_distance`/`microsteps`/`position_max`。
- **步骤**：`SET_KINEMATIC_POSITION` 标定 → `G1 X10`、`G1 X0 Y10`、`G1 Z1`（三轴 + `[extruder]` 若有）。
- **判定**：电机按预期方向/距离动；`objects/query toolhead` 的 `position`/`homed_axes` 正确；
  `M400` 后固件步数与命令距离一致（`stepper_get_position`）。
- **依据**：`TODO.md` FW5 行；`docs/work-log/2026-09-21-fw5e-notes.md`。

### T2. FW6：`M119` / `query_endstops/status`（需要接一个 endstop）

- **前置**：一个 endstop 接到已知引脚（例如 X 的 `endstop_pin`），配 `[stepper_x] endstop_pin`。
- **步骤**：`M119`（或 `query_endstops/status`）在断开/闭合（手动短接）两种状态各读一次。
- **判定**：`open` ↔ `TRIGGERED` 随电平翻转；`!`/`^` 极性/上拉符合配置。
- **依据**：`TODO.md` FW6c；`docs/work-log/2026-09-21-fw6c-notes.md`。

### T3. FW6：单轴 `G28`（硬件回零，需要 endstop + 电机）

- **前置**：T2 的 endstop + X 电机；`homing_speed` 设小值（如 5 mm/s）。
- **步骤**：`G28 X`。
- **判定**：向 `position_endstop` 方向移动、触发即停；`homed_axes` 含 `x`；`position` 落在
  `position_endstop`（在 `max_error` 内）；不撞机、不越界。
- **依据**：`TODO.md` FW6e；`docs/work-log/2026-09-21-fw6de-notes.md`。

### T4. FW6：双板时序/漂移（需要两块板或一个副 MCU）

- **前置**：第二块板（`[mcu zboard]`，独立供电/USB 或 CAN），两端各自接一个 endstop/stepper。
- **步骤**：
  1. 两板同时使用（一轴在主、一轴在副）做 `G1`/`G28`；
  2. 长时间运行（小时级）观察次 MCU 是否有步进滑移/超时。
- **判定**：跨 MCU 停轴同时生效；长时间后次 MCU 的 print-time 映射仍对齐
  （软件侧已用 ±100 ppm/1 h 模拟，误差 <10 ms；真板验**实际晶振漂移量**与
  电气/传输时序、USB/CAN 抖动与重传）。
- **依据**：`docs/work-log/2026-09-21-fw6de-notes.md`；`SecondarySync` 周期重校准。

### T5. FW7/FW8：`rpi_usb` 与物理复位

- **前置**：带可控 USB 供电口的板（hub 支持 per-port power switching，或 udev 规则已按
  `scripts/klipperx-usb-udev.sh` 安装）。
- **步骤**：`restart_method: rpi_usb` 下 `FIRMWARE_RESTART`；断电重启；拔插。
- **判定**：端口能真正断电使板重启；换电后 identify 会话不误判；失败则回退 `command` 并给提示
  （代码已就绪，只有决策逻辑单测）。D2 的启动抖动为**已归档非阻塞观察项**，不再排期。
- **依据**：`TODO.md` D2/FW8；`mcu/restart.rs`。

### T6. F9 / H5–H8：其余固件资源与外设（按需）

- buttons / pulse_counter / trigger_analog、sdcard、LCD、neopixel/dotstar、
  `sensor_bulk` 与各类 SPI/I2C 传感器（`sensor_adxl345` 等）、tmcuart、`temperature_mcu` 等。
- 各自接一个真实外设后验；未接外设前只做 host 单测/假 MCU。
- **依据**：`TODO.md` F9 与 H5–H8。

### T7. `[extruder]` / `stepper_enable` / 回零精度细节

- `[extruder]` + `stepper_enable`（`enable_pin`、`M18`/`M84`）真板基本动作。
- 回零精度细节（`homing_retract_dist` 回抽 + 二次回零、`endstop_phase` 的
  `get_trigger_position`/`set_stepper_adjustment`）：**这些是未实现的软件功能**，不是纯硬件验证；
  实现后再谈真板。
- **依据**：`TODO.md` C1/F8；`docs/work-log/2026-09-21-fw6de-notes.md` §已知限制。

## 3. 什么时候算“真板项清零”

每项做到「按上面判定通过 → 把结果写回本表 §1（留档）」即可。真板项**不阻止**任何
`TODO.md` 任务从「待办」移入「已完成（留档）」。
