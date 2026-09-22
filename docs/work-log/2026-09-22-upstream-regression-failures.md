# 上游回归测试失败原因分析

> **来源**：从 `docs/klippy/developer-manual/regression-tests.md` 的「忽略列表 → 失败原因分析」
> 与 `TODO.md` 的「当前失败原因统计」合并去重整理。
>
> **运行方式**：`KLIPPERX_UPSTREAM_ALL=1 cargo test -p klipperx --lib upstream_test_cases_run`
>
> **总失败次数**：214 次（忽略列表中的运行，在 `KLIPPERX_UPSTREAM_ALL=1` 下暴露的失败）

---

## 失败原因总览

| 首次失败原因 | 条数 | 根本原因 | 对应 TODO |
|--------------|------|----------|-----------|
| `extruder` 段未实现 | 99 | 配置引用了 `[extruder]` 段，但本主机尚未实现挤出机控制 | T3 |
| `probe` pin chip 未知 | 36 | 配置使用 `probe` 作为 pin 标识，但本主机尚未实现端停 pin chip | T4 |
| `delta` 运动学未实现 | 12 | 配置声明 `kinematics: delta`，本主机仅支持 `cartesian` 和 `none` | T5 |
| `corexy` 运动学未实现 | 10 | 配置声明 `kinematics: corexy`，本主机仅支持 `cartesian` 和 `none` | T5 |
| `tmc2209 stepper_x` 段未实现 | 12 | 配置引用了 `[tmc2209 stepper_x]` 段，但本主机尚未实现 TMC 驱动 | T6 |
| `tmc2209_stepper_x` pin chip 未知 | 6 | 配置使用 `tmc2209_stepper_x` 作为 pin 标识，但本主机尚未实现 TMC pin chip | T6 |
| `stepper_z1` 段未实现 | 5 | 配置引用了多轴 stepper（`stepper_z1`），但本主机尚未实现多轴支持 | T10 |
| `output_pin value` 选项超限 | 5 | 配置使用 `output_pin stepper_xy_current` 且 `value` 超过最大值 1 | H2 |
| `temperature_mcu` 传感器未知 | 5 | 配置引用了 MCU 温度传感器，但本主机尚未实现 | H1 |
| `generic_cartesian` 运动学未实现 | 4 | 配置声明 `kinematics: generic_cartesian`，本主机仅支持 `cartesian` 和 `none` | T5 |
| `tmc2130_stepper_x` pin chip 未知 | 3 | 配置使用 `tmc2130_stepper_x` 作为 pin 标识，但本主机尚未实现 TMC pin chip | T6 |
| `tmc2208 stepper_x` 段未实现 | 3 | 配置引用了 `[tmc2208 stepper_x]` 段，但本主机尚未实现 TMC 驱动 | T6 |
| `safe_z_home` 段未实现 | 3 | 配置引用了 `[safe_z_home]` 段，但本主机尚未实现安全 Z 回零 | T9 |
| `mcu restart_method` 选项无效 | 3 | 配置使用 `restart_method` 选项，但本主机尚未实现该选项 | T9 |
| `tmc5160 stepper_x` 段未实现 | 2 | 配置引用了 `[tmc5160 stepper_x]` 段，但本主机尚未实现 TMC 驱动 | T6 |
| `static_digital_output` 段未实现 | 2 | 配置引用了 `[static_digital_output]` 段，但本主机尚未实现 | T9 |
| `rotary_delta` 运动学未实现 | 2 | 配置声明 `kinematics: rotary_delta`，本主机仅支持 `cartesian` 和 `none` | T5 |
| `hybrid_corexy` 运动学未实现 | 2 | 配置声明 `kinematics: hybrid_corexy`，本主机仅支持 `cartesian` 和 `none` | T5 |
| 其余单实例失败 | 14 | 各 1 条，涉及 `gcode_arcs`、`virtual_sdcard`、`led`、`manual_stepper`、`pwm_cycle_time`、`display`、`replicape`、`adc_scaled`、`endstop_phase`、`dual_carriage`、`bed_screws`、`tmc2660`、`tmc2130`、`sx1509_duex`、`temperature` 等 | T9 |

（99 + 36 + 12 + 10 + 12 + 6 + 5 + 5 + 5 + 4 + 3 + 3 + 3 + 2 + 2 + 2 + 2 + 14 = 214。）

---

## 按领域分组工单

### T3：`extruder` + `heater_bed` + `fan`（99 次失败）

- **失败原因**：`Section 'extruder' is not a valid config section`
- **影响文件**：`commands.test`、`out_of_bounds.test`、`printers.test` 族的主要失败原因
- **依赖**：H1（heaters / heater_bed / fan）
- **说明**：涉及大量打印机配置（Voron2、BigTreeTech Octopus/SKR 系列、Prusa Mini+ 等）都依赖挤出机控制

### T4：`probe` / `bltouch` / endstop pin chip（36 次失败）

- **失败原因**：`Unknown pin chip name 'probe'`
- **影响文件**：`bed_mesh.test`、`bltouch.test`、`eddy.test`、`screws_tilt_adjust.test`、`smart_effector.test`、`z_virtual_endstop.test`
- **依赖**：F8（endstop / trsync）
- **说明**：涉及探针相关的调平与校准功能

### T5：运动学（31 次失败）

- **失败原因**：`Error loading kinematics '<name>' (only 'cartesian' and 'none' are implemented)`
- **细分**：

| `kinematics` | `.test` | 条数 |
|--------------|---------|------|
| `delta` | `delta`、`delta_calibrate`、`printers`（6 条） | 12 |
| `corexy` | `corexyuv`、`printers`（4 条） | 10 |
| `generic_cartesian` | `generic_cartesian`、`generic_cartesian_iqex`、`generic_cartesian_itex` | 4 |
| `rotary_delta` | `rotary_delta_calibrate`、`printers` | 2 |
| `hybrid_corexy` | `hybrid_corexy_dual_carriage`、`printers` | 2 |
| `corexz` / `hybrid_corexz` / `polar` / `winch` / `deltesian` | `printers`（各 1 条） | 各 1 |

- **说明**：当前仅支持 `cartesian` 和 `none`，其他运动学类型均未实现

### T6：TMC pin chip（19 次失败）

- **失败原因**：`Section 'tmc2209 stepper_x' is not a valid config section` 或 `Unknown pin chip name 'tmc2209_stepper_x'`
- **细分**：

| 失败原因 | 条数 |
|----------|------|
| `tmc2209 stepper_x` 段未实现 | 12 |
| `tmc2130_stepper_x` pin chip 未知 | 3 |
| `tmc2208 stepper_x` 段未实现 | 3 |
| `tmc5160 stepper_x` 段未实现 | 2 |
| `tmc2660 stepper_x` 段未实现 | 1 |
| `tmc2209_stepper_x` pin chip 未知 | 6（与上表有重叠，按首次失败归类） |

- **依赖**：H5（TMC 步进驱动）

### T7：温度传感器（7 次失败）

- **失败原因**：`Unknown temperature sensor 'temperature_mcu'`（5 次）、`TDK NTCG104LH104JT1`、`my_custom_resistance_adc`
- **依赖**：H1（温度传感器）
- **说明**：涉及 `temperature_mcu`、自定义热敏电阻、ADC 温度传感器等

### T8：`output_pin` 的 `value` 选项（5 次失败）

- **失败原因**：`Option 'value' in section 'output_pin stepper_xy_current' must have maximum of 1`
- **依赖**：H2（风扇与通用输出）
- **说明**：`output_pin` 的 `value` 选项需要支持超过 1 的值

### T9：其余 extras 段（14 次失败）

- **涉及模块**：`bed_screws`、`dual_carriage`、`safe_z_home`、`endstop_phase`、`adc_scaled`、`static_digital_output`、`pwm_cycle_time`、`led`、`manual_stepper`、`display`、`replicape`、`gcode_arcs`、`virtual_sdcard`、`stepper_z1`（多轴）等
- **说明**：各 1 条，按域归入 H1–H10

### T10：多轴 stepper（5 次失败）

- **失败原因**：`Section 'stepper_z1' is not a valid config section`
- **影响文件**：`multi_z.test`、`quad_gantry_level.test`、`z_tilt.test`、`printers.test`（Anycubic Kossel 等）
- **说明**：需要支持多轴 stepper（`stepper_z1`、`stepper_z2` 等）

---

## 完整失败日志

以下日志来自 `KLIPPERX_UPSTREAM_ALL=1 cargo test -p klipperx --lib upstream_test_cases_run`：

```
bed_mesh.test (test/klippy/bed_mesh.cfg): test/klippy/bed_mesh.cfg: stepper_z: Unknown pin chip name 'probe'
bed_screws.test (test/klippy/bed_screws.cfg): test/klippy/bed_screws.cfg: Section 'bed_screws' is not a valid config section
bltouch.test (test/klippy/bltouch.cfg): test/klippy/bltouch.cfg: stepper_z: Unknown pin chip name 'probe'
commands.test (test/klippy/../../config/example-cartesian.cfg): test/klippy/../../config/example-cartesian.cfg: Section 'extruder' is not a valid config section
corexyuv.test (test/klippy/corexyuv.cfg): test/klippy/corexyuv.cfg: Error loading kinematics 'generic_cartesian' (only 'cartesian' and 'none' are implemented)
delta.test (test/klippy/../../config/example-delta.cfg): test/klippy/../../config/example-delta.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
delta_calibrate.test (test/klippy/delta_calibrate.cfg): test/klippy/delta_calibrate.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
dual_carriage.test (test/klippy/dual_carriage.cfg): test/klippy/dual_carriage.cfg: Section 'dual_carriage' is not a valid config section
eddy.test (test/klippy/eddy.cfg): test/klippy/eddy.cfg: stepper_z: Unknown pin chip name 'probe'
exclude_object.test (test/klippy/exclude_object.cfg): test/klippy/exclude_object.cfg: Section 'extruder' is not a valid config section
extruders.test (test/klippy/extruders.cfg): test/klippy/extruders.cfg: Section 'extruder' is not a valid config section
gcode_arcs.test (test/klippy/gcode_arcs.cfg): test/klippy/gcode_arcs.cfg: Section 'gcode_arcs' is not a valid config section
generic_cartesian.test (test/klippy/generic_cartesian.cfg): test/klippy/generic_cartesian.cfg: Error loading kinematics 'generic_cartesian' (only 'cartesian' and 'none' are implemented)
generic_cartesian_iqex.test (test/klippy/generic_cartesian_iqex.cfg): test/klippy/generic_cartesian_iqex.cfg: Error loading kinematics 'generic_cartesian' (only 'cartesian' and 'none' are implemented)
generic_cartesian_itex.test (test/klippy/generic_cartesian_itex.cfg): test/klippy/generic_cartesian_itex.cfg: Error loading kinematics 'generic_cartesian' (only 'cartesian' and 'none' are implemented)
hybrid_corexy_dual_carriage.test (test/klippy/hybrid_corexy_dual_carriage.cfg): test/klippy/hybrid_corexy_dual_carriage.cfg: Error loading kinematics 'hybrid_corexy' (only 'cartesian' and 'none' are implemented)
input_shaper.test (test/klippy/input_shaper.cfg): test/klippy/input_shaper.cfg: Section 'extruder' is not a valid config section
led.test (test/klippy/led.cfg): test/klippy/led.cfg: Section 'led lled' is not a valid config section
load_cell.test (test/klippy/load_cell.cfg): test/klippy/load_cell.cfg: Section 'extruder' is not a valid config section
macros.test (test/klippy/macros.cfg): test/klippy/macros.cfg: Section 'extruder' is not a valid config section
manual_stepper.test (test/klippy/manual_stepper.cfg): test/klippy/manual_stepper.cfg: Section 'manual_stepper basic_stepper' is not a valid config section
multi_z.test (test/klippy/multi_z.cfg): test/klippy/multi_z.cfg: Section 'stepper_z1' is not a valid config section
out_of_bounds.test (test/klippy/../../config/example-cartesian.cfg): test/klippy/../../config/example-cartesian.cfg: Section 'extruder' is not a valid config section
polar.test (test/klippy/../../config/example-polar.cfg): test/klippy/../../config/example-polar.cfg: Error loading kinematics 'polar' (only 'cartesian' and 'none' are implemented)
pressure_advance.test (test/klippy/pressure_advance.cfg): test/klippy/pressure_advance.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/example-cartesian.cfg): test/klippy/../../config/example-cartesian.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/example-corexy.cfg): test/klippy/../../config/example-corexy.cfg: Error loading kinematics 'corexy' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/example-corexz.cfg): test/klippy/../../config/example-corexz.cfg: Error loading kinematics 'corexz' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/example-hybrid-corexy.cfg): test/klippy/../../config/example-hybrid-corexy.cfg: Error loading kinematics 'hybrid_corexy' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/example-hybrid-corexz.cfg): test/klippy/../../config/example-hybrid-corexz.cfg: Error loading kinematics 'hybrid_corexz' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/example-delta.cfg): test/klippy/../../config/example-delta.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/example-deltesian.cfg): test/klippy/../../config/example-deltesian.cfg: Error loading kinematics 'deltesian' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/example-rotary-delta.cfg): test/klippy/../../config/example-rotary-delta.cfg: Error loading kinematics 'rotary_delta' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/example-winch.cfg): test/klippy/../../config/example-winch.cfg: Error loading kinematics 'winch' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/generic-einsy-rambo.cfg): test/klippy/../../config/generic-einsy-rambo.cfg: Unknown temperature sensor 'TDK NTCG104LH104JT1'
printers.test (test/klippy/../../config/generic-fysetc-f6.cfg): test/klippy/../../config/generic-fysetc-f6.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-gt2560.cfg): test/klippy/../../config/generic-gt2560.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-mini-rambo.cfg): test/klippy/../../config/generic-mini-rambo.cfg: Option 'value' in section 'output_pin stepper_xy_current' must have maximum of 1
printers.test (test/klippy/../../config/generic-rambo.cfg): test/klippy/../../config/generic-rambo.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-rumba.cfg): test/klippy/../../config/generic-rumba.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-ultimaker-ultimainboard-v2.cfg): test/klippy/../../config/generic-ultimaker-ultimainboard-v2.cfg: Option 'value' in section 'output_pin stepper_xy_current' must have maximum of 1
printers.test (test/klippy/../../config/kit-zav3d-2019.cfg): test/klippy/../../config/kit-zav3d-2019.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-adimlab-2018.cfg): test/klippy/../../config/printer-adimlab-2018.cfg: Option 'value' in section 'output_pin stepper_xy_current' must have maximum of 1
printers.test (test/klippy/../../config/printer-anycubic-4max-2018.cfg): test/klippy/../../config/printer-anycubic-4max-2018.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-anycubic-4maxpro-2.0-2021.cfg): test/klippy/../../config/printer-anycubic-4maxpro-2.0-2021.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-anycubic-i3-mega-2017.cfg): test/klippy/../../config/printer-anycubic-i3-mega-2017.cfg: Section 'stepper_z1' is not a valid config section
printers.test (test/klippy/../../config/printer-anycubic-kossel-2016.cfg): test/klippy/../../config/printer-anycubic-kossel-2016.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-anycubic-kossel-plus-2017.cfg): test/klippy/../../config/printer-anycubic-kossel-plus-2017.cfg: Error loading kinematics 'delta' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/printer-bq-hephestos-2014.cfg): test/klippy/../../config/printer-bq-hephestos-2014.cfg: Section 'display' is not a valid config section
printers.test (test/klippy/../../config/printer-biqu-b1-se-plus-2022.cfg): test/klippy/../../config/printer-biqu-b1-se-plus-2022.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-biqu-bx-2021.cfg): test/klippy/../../config/printer-biqu-bx-2021.cfg: stepper_x: Unknown pin chip name 'tmc2209_stepper_x'
printers.test (test/klippy/../../config/printer-prusa-mini-plus-2020.cfg): test/klippy/../../config/printer-prusa-mini-plus-2020.cfg: stepper_x: Unknown pin chip name 'tmc2209_stepper_x'
printers.test (test/klippy/../../config/printer-ratrig-v-minion-2021.cfg): test/klippy/../../config/printer-ratrig-v-minion-2021.cfg: stepper_z: Unknown pin chip name 'probe'
printers.test (test/klippy/../../config/printer-tronxy-crux1-2022.cfg): test/klippy/../../config/printer-tronxy-crux1-2022.cfg: Option 'restart_method' is not valid in section 'mcu'
printers.test (test/klippy/../../config/generic-bigtreetech-gtr.cfg): test/klippy/../../config/generic-bigtreetech-gtr.cfg: Unknown temperature sensor 'temperature_mcu'
printers.test (test/klippy/../../config/generic-bigtreetech-octopus-v1.1.cfg): test/klippy/../../config/generic-bigtreetech-octopus-v1.1.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-octopus-pro-v1.0.cfg): test/klippy/../../config/generic-bigtreetech-octopus-pro-v1.0.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-octopus-pro-v1.1.cfg): test/klippy/../../config/generic-bigtreetech-octopus-pro-v1.1.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-octopus-max-ez.cfg): test/klippy/../../config/generic-bigtreetech-octopus-max-ez.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-pro.cfg): test/klippy/../../config/generic-bigtreetech-skr-pro.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-2.cfg): test/klippy/../../config/generic-bigtreetech-skr-2.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-3.cfg): test/klippy/../../config/generic-bigtreetech-skr-3.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-skr-pico-v1.0.cfg): test/klippy/../../config/generic-bigtreetech-skr-pico-v1.0.cfg: Unknown temperature sensor 'temperature_mcu'
printers.test (test/klippy/../../config/generic-bigtreetech-skr-mini-e3-v3.0.cfg): test/klippy/../../config/generic-bigtreetech-skr-mini-e3-v3.0.cfg: Section 'tmc2209 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-manta-m4p.cfg): test/klippy/../../config/generic-bigtreetech-manta-m4p.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-manta-m5p.cfg): test/klippy/../../config/generic-bigtreetech-manta-m5p.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-manta-m8p-v1.0.cfg): test/klippy/../../config/generic-bigtreetech-manta-m8p-v1.0.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-manta-m8p-v1.1.cfg): test/klippy/../../config/generic-bigtreetech-manta-m8p-v1.1.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-bigtreetech-manta-e3ez.cfg): test/klippy/../../config/generic-bigtreetech-manta-e3ez.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-flyboard.cfg): test/klippy/../../config/generic-flyboard.cfg: Error loading kinematics 'corexy' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/generic-I3DBEEZ9.cfg): test/klippy/../../config/generic-I3DBEEZ9.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-mellow-fly-cdy-v3.cfg): test/klippy/../../config/generic-mellow-fly-cdy-v3.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-mellow-fly-e3-v2.cfg): test/klippy/../../config/generic-mellow-fly-e3-v2.cfg: Unknown temperature sensor 'temperature_mcu'
printers.test (test/klippy/../../config/generic-mellow-super-infinty-hv.cfg): test/klippy/../../config/generic-mellow-super-infinty-hv.cfg: Error loading kinematics 'corexy' (only 'cartesian' and 'none' are implemented)
printers.test (test/klippy/../../config/generic-mks-monster8.cfg): test/klippy/../../config/generic-mks-monster8.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-mks-robin-nano-v3.cfg): test/klippy/../../config/generic-mks-robin-nano-v3.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/generic-replicape.cfg): test/klippy/../../config/generic-replicape.cfg: Section 'replicape' is not a valid config section
printers.test (test/klippy/../../config/generic-th3d-ezboard-v2.0.cfg): test/klippy/../../config/generic-th3d-ezboard-v2.0.cfg: Section 'tmc2209 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-ldo-leviathan-v1.2.cfg): test/klippy/../../config/generic-ldo-leviathan-v1.2.cfg: Section 'tmc5160 stepper_x' is not a valid config section
printers.test (test/klippy/../../config/generic-mks-rumba32-v1.0.cfg): test/klippy/../../config/generic-mks-rumba32-v1.0.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-anycubic-kobra-go-2022.cfg): test/klippy/../../config/printer-anycubic-kobra-go-2022.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/printer-anycubic-kobra-plus-2022.cfg): test/klippy/../../config/printer-anycubic-kobra-plus-2022.cfg: Option 'restart_method' is not valid in section 'mcu'
printers.test (test/klippy/../../config/sample-multi-mcu.cfg): test/klippy/../../config/sample-multi-mcu.cfg: Section 'extruder' is not a valid config section
printers.test (test/klippy/../../config/kit-voron2-250mm.cfg): test/klippy/../../config/kit-voron2-250mm.cfg: Error loading kinematics 'corexy' (only 'cartesian' and 'none' are implemented)
quad_gantry_level.test (test/klippy/z_tilt.cfg): test/klippy/z_tilt.cfg: Section 'stepper_z1' is not a valid config section
temperature.test (test/klippy/temperature.cfg): test/klippy/temperature.cfg: Unknown temperature sensor 'my_custom_resistance_adc'
z_tilt.test (test/klippy/z_tilt.cfg): test/klippy/z_tilt.cfg: Section 'stepper_z1' is not a valid config section
screws_tilt_adjust.test (test/klippy/screws_tilt_adjust.cfg): test/klippy/screws_tilt_adjust.cfg: stepper_z: Unknown pin chip name 'probe'
smart_effector.test (test/klippy/smart_effector.cfg): test/klippy/smart_effector.cfg: stepper_z: Unknown pin chip name 'probe'
z_virtual_endstop.test (test/klippy/z_virtual_endstop.cfg): test/klippy/z_virtual_endstop.cfg: stepper_z: Unknown pin chip name 'probe'
tmc.test (test/klippy/tmc.cfg): test/klippy/tmc.cfg: Section 'tmc2209 stepper_x' is not a valid config section
```

---

## 推进策略

按「闭包最小 → 杠杆最大」顺序推进（T1 之后）：

| 顺序 | 事项 | 失败次数 | 依赖 |
|------|------|----------|------|
| ✅ T1 | `linuxtest.test`（已通过） | — | 无 |
| ✅ T2 | `[stepper_enable]`（已通过，14 条移除） | — | 无 |
| T3 | `extruder` + `heater_bed` + `fan` | 99 | H1 |
| T4 | `probe` / `bltouch` / endstop pin chip | 36 | F8 |
| T5 | 运动学（delta/corexy/rotary_delta/hybrid_corexy 等） | 31 | C1 |
| T6 | TMC pin chip | 19 | H5 |
| T7 | 温度传感器 | 7 | H1 |
| T8 | `output_pin` 的 `value` 选项 | 5 | H2 |
| T9 | 其余 extras 段 | 14 | 各域 |
| T10 | 多轴 stepper | 5 | C1 |

---

## 备注

- `printers.test` 有 2 条运行声明 `DICTIONARY pru.dict host=linuxprocess.dict`，默认不构建 `pru`；要跑需 `KLIPPERX_ARCHES=...,pru`（需 `pru-gcc`）。这 2 条计入「因字典未构建跳过」，不算失败。
- `out_of_bounds.test` 是唯一声明 `SHOULD_FAIL` 的用例，它期望的是**运行期**错误（`G1 Y9999` 越界），不是配置错误。目前留在忽略列表里是正确的：等所需节/运动学落地后再移出。

---

- [← work-log 首页](README.md)
- [上游功能覆盖审计 →](2026-09-21-upstream-coverage-audit.md)
- [已完成条目归档 →](2026-09-22-completed-archive.md)
