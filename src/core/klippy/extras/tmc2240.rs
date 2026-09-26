//! `[tmc2240 <stepper>]` — the TMC2240 driver (upstream's
//! `klippy/extras/tmc2240.py`).
//!
//! The newest SPI TMC chip: like the 2130/5160 it speaks the 5-byte daisy-chain
//! frame ([`TmcSpiChain`], 12.5 MHz), it registers the virtual-endstop chip
//! ([`TmcVirtualPin`]) and it loads the microstep wave table. What sets it apart
//! is [`Tmc2240Current`]: the chip has **no `sense_resistor`** — the full-scale
//! current follows from `rref` and the `DRV_CONF.current_range` the helper picks
//! — and it is the one SPI driver whose section may name a `uart_pin`, in which
//! case it reaches the chip over UART instead (upstream `tmc2240.py:352-359`).
//!
//! # What is not here
//!
//! The `heaters.register_monitor` call upstream makes for `adc_temp`
//! (`tmc.py:128-131`, so the `[heaters]` status lists the section among its
//! `available_monitors`) is not made: [`TmcErrorCheck`] does not read the
//! temperature register, so nothing registers the monitor. The
//! `tmc/stallguard_dump` bulk endpoint (`tmc.TMCStallguardDump`) is likewise
//! absent, as it is for every driver in this host.
//!
//! [`TmcErrorCheck`]: crate::core::klippy::extras::tmc::TmcErrorCheck

use std::collections::HashMap;
use std::sync::Arc;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::tmc::{
    stealthchop_helper, vcoolthrs_helper, vhigh_helper, wave_table_helper, FieldHelper,
    Tmc2240Current, TmcDriver, TmcTransport, TmcVirtualPin,
};
use crate::core::klippy::extras::tmc2130::field_formatters as tmc2130_field_formatters;
use crate::core::klippy::extras::tmc_spi::TmcSpiChain;
use crate::core::klippy::extras::tmc_uart::TmcUart;
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

// Loaded after the pins and the MCU's SPI resource; alongside the other
// `order = 40` device sections.
section!("tmc2240", order = 40, prefix = load_config_prefix);

/// The chip's internal TSTEP frequency (`TMC_FREQUENCY`).
pub const TMC_FREQUENCY: f64 = 12_500_000.;

/// The largest `uart_address` the chip answers (`MCU_TMC_uart(..., 7, ...)`).
pub const MAX_ADDR: i64 = 7;

/// Register name → address (`Registers`).
pub fn registers() -> HashMap<String, u8> {
    [
        ("GCONF", 0x00),
        ("GSTAT", 0x01),
        ("IFCNT", 0x02),
        ("NODECONF", 0x03),
        ("IOIN", 0x04),
        ("DRV_CONF", 0x0a),
        ("GLOBALSCALER", 0x0b),
        ("IHOLD_IRUN", 0x10),
        ("TPOWERDOWN", 0x11),
        ("TSTEP", 0x12),
        ("TPWMTHRS", 0x13),
        ("TCOOLTHRS", 0x14),
        ("THIGH", 0x15),
        ("DIRECT_MODE", 0x2d),
        ("ENCMODE", 0x38),
        ("X_ENC", 0x39),
        ("ENC_CONST", 0x3a),
        ("ENC_STATUS", 0x3b),
        ("ENC_LATCH", 0x3c),
        ("ADC_VSUPPLY_AIN", 0x50),
        ("ADC_TEMP", 0x51),
        ("OTW_OV_VTH", 0x52),
        ("MSLUT0", 0x60),
        ("MSLUT1", 0x61),
        ("MSLUT2", 0x62),
        ("MSLUT3", 0x63),
        ("MSLUT4", 0x64),
        ("MSLUT5", 0x65),
        ("MSLUT6", 0x66),
        ("MSLUT7", 0x67),
        ("MSLUTSEL", 0x68),
        ("MSLUTSTART", 0x69),
        ("MSCNT", 0x6a),
        ("MSCURACT", 0x6b),
        ("CHOPCONF", 0x6c),
        ("COOLCONF", 0x6d),
        ("DRV_STATUS", 0x6f),
        ("PWMCONF", 0x70),
        ("PWM_SCALE", 0x71),
        ("PWM_AUTO", 0x72),
        ("SG4_THRS", 0x74),
        ("SG4_RESULT", 0x75),
        ("SG4_IND", 0x76),
    ]
    .into_iter()
    .map(|(name, addr)| (name.to_string(), addr))
    .collect()
}

/// The registers `DUMP_TMC` reads (`ReadRegisters`).
pub fn read_registers() -> Vec<String> {
    [
        "GCONF",
        "GSTAT",
        "IOIN",
        "DRV_CONF",
        "GLOBALSCALER",
        "IHOLD_IRUN",
        "TPOWERDOWN",
        "TSTEP",
        "TPWMTHRS",
        "TCOOLTHRS",
        "THIGH",
        "ADC_VSUPPLY_AIN",
        "ADC_TEMP",
        "OTW_OV_VTH",
        "MSCNT",
        "MSCURACT",
        "CHOPCONF",
        "COOLCONF",
        "DRV_STATUS",
        "PWMCONF",
        "PWM_SCALE",
        "PWM_AUTO",
        "SG4_THRS",
        "SG4_RESULT",
        "SG4_IND",
    ]
    .into_iter()
    .map(str::to_string)
    .collect()
}

/// The fields whose register value is two's-complement signed (`SignedFields`).
pub const SIGNED_FIELDS: [&str; 5] = ["cur_a", "cur_b", "sgt", "pwm_scale_auto", "offset_sin90"];

/// The chip's register/field layout (`Fields`).
pub fn fields() -> HashMap<String, HashMap<String, u32>> {
    let mut fields: HashMap<String, HashMap<String, u32>> = HashMap::new();
    macro_rules! reg {
        ($name:literal, { $($field:literal: $mask:expr),* $(,)? }) => {
            fields.insert(
                $name.to_string(),
                HashMap::from([$(($field.to_string(), $mask as u32)),*]),
            );
        };
    }
    reg!("COOLCONF", {
        "semin": 0x0f, "seup": 0x03 << 5, "semax": 0x0f << 8, "sedn": 0x03 << 13,
        "seimin": 0x01 << 15, "sgt": 0x7f << 16, "sfilt": 0x01 << 24
    });
    reg!("CHOPCONF", {
        "toff": 0x0f, "hstrt": 0x07 << 4, "hend": 0x0f << 7, "fd3": 0x01 << 11,
        "disfdcc": 0x01 << 12, "chm": 0x01 << 14, "tbl": 0x03 << 15, "vhighfs": 0x01 << 18,
        "vhighchm": 0x01 << 19, "tpfd": 0x0f << 20, "mres": 0x0f << 24, "intpol": 0x01 << 28,
        "dedge": 0x01 << 29, "diss2g": 0x01 << 30, "diss2vs": 0x01 << 31
    });
    reg!("DRV_STATUS", {
        "sg_result": 0x3ff, "s2vsa": 0x01 << 12, "s2vsb": 0x01 << 13, "stealth": 0x01 << 14,
        "fsactive": 0x01 << 15, "cs_actual": 0x1f << 16, "stallguard": 0x01 << 24,
        "ot": 0x01 << 25, "otpw": 0x01 << 26, "s2ga": 0x01 << 27, "s2gb": 0x01 << 28,
        "ola": 0x01 << 29, "olb": 0x01 << 30, "stst": 0x01 << 31
    });
    reg!("GCONF", {
        "faststandstill": 0x01 << 1, "en_pwm_mode": 0x01 << 2, "multistep_filt": 0x01 << 3,
        "shaft": 0x01 << 4, "diag0_error": 0x01 << 5, "diag0_otpw": 0x01 << 6,
        "diag0_stall": 0x01 << 7, "diag1_stall": 0x01 << 8, "diag1_index": 0x01 << 9,
        "diag1_onstate": 0x01 << 10, "diag0_pushpull": 0x01 << 12, "diag1_pushpull": 0x01 << 13,
        "small_hysteresis": 0x01 << 14, "stop_enable": 0x01 << 15, "direct_mode": 0x01 << 16
    });
    reg!("GSTAT", {
        "reset": 0x01, "drv_err": 0x01 << 1, "uv_cp": 0x01 << 2, "register_reset": 0x01 << 3,
        "vm_uvlo": 0x01 << 4
    });
    reg!("GLOBALSCALER", { "globalscaler": 0xff });
    reg!("IHOLD_IRUN", {
        "ihold": 0x1f, "irun": 0x1f << 8, "iholddelay": 0x0f << 16, "irundelay": 0x0f << 24
    });
    reg!("IOIN", {
        "step": 0x01, "dir": 0x01 << 1, "encb": 0x01 << 2, "enca": 0x01 << 3,
        "drv_enn": 0x01 << 4, "encn": 0x01 << 5, "uart_en": 0x01 << 6, "comp_a": 0x01 << 8,
        "comp_b": 0x01 << 9, "comp_a1_a2": 0x01 << 10, "comp_b1_b2": 0x01 << 11,
        "output": 0x01 << 12, "ext_res_det": 0x01 << 13, "ext_clk": 0x01 << 14,
        "adc_err": 0x01 << 15, "silicon_rv": 0x07 << 16, "version": 0xff << 24
    });
    reg!("MSLUT0", { "mslut0": 0xffffffff });
    reg!("MSLUT1", { "mslut1": 0xffffffff });
    reg!("MSLUT2", { "mslut2": 0xffffffff });
    reg!("MSLUT3", { "mslut3": 0xffffffff });
    reg!("MSLUT4", { "mslut4": 0xffffffff });
    reg!("MSLUT5", { "mslut5": 0xffffffff });
    reg!("MSLUT6", { "mslut6": 0xffffffff });
    reg!("MSLUT7", { "mslut7": 0xffffffff });
    reg!("MSLUTSEL", {
        "x3": 0xff << 24, "x2": 0xff << 16, "x1": 0xff << 8,
        "w3": 0x03 << 6, "w2": 0x03 << 4, "w1": 0x03 << 2, "w0": 0x03
    });
    reg!("MSLUTSTART", {
        "start_sin": 0xff, "start_sin90": 0xff << 16, "offset_sin90": 0xff << 24
    });
    reg!("MSCNT", { "mscnt": 0x3ff });
    reg!("MSCURACT", { "cur_a": 0x1ff, "cur_b": 0x1ff << 16 });
    reg!("PWM_AUTO", { "pwm_ofs_auto": 0xff, "pwm_grad_auto": 0xff << 16 });
    reg!("PWMCONF", {
        "pwm_ofs": 0xff, "pwm_grad": 0xff << 8, "pwm_freq": 0x03 << 16,
        "pwm_autoscale": 0x01 << 18, "pwm_autograd": 0x01 << 19, "freewheel": 0x03 << 20,
        "pwm_meas_sd_enable": 0x01 << 22, "pwm_dis_reg_stst": 0x01 << 23,
        "pwm_reg": 0x0f << 24, "pwm_lim": 0x0f << 28
    });
    reg!("PWM_SCALE", { "pwm_scale_sum": 0x3ff, "pwm_scale_auto": 0x1ff << 16 });
    reg!("TPOWERDOWN", { "tpowerdown": 0xff });
    reg!("TPWMTHRS", { "tpwmthrs": 0xfffff });
    reg!("TCOOLTHRS", { "tcoolthrs": 0xfffff });
    reg!("TSTEP", { "tstep": 0xfffff });
    reg!("THIGH", { "thigh": 0xfffff });
    reg!("DRV_CONF", { "current_range": 0x03, "slope_control": 0x03 << 4 });
    reg!("ADC_VSUPPLY_AIN", { "adc_vsupply": 0x1fff, "adc_ain": 0x1fff << 16 });
    reg!("ADC_TEMP", { "adc_temp": 0x1fff });
    reg!("OTW_OV_VTH", { "overvoltage_vth": 0x1fff, "overtempprewarning_vth": 0x1fff << 16 });
    reg!("SG4_THRS", { "sg4_thrs": 0xff, "sg4_filt_en": 0x01 << 8, "sg4_angle_offset": 0x01 << 9 });
    reg!("SG4_RESULT", { "sg4_result": 0x3ff });
    reg!("SG4_IND", {
        "sg4_ind_0": 0xff, "sg4_ind_1": 0xff << 8, "sg4_ind_2": 0xff << 16, "sg4_ind_3": 0xff << 24
    });
    fields
}

/// The `DUMP_TMC` field formatters (`FieldFormatters`): the TMC2130's, plus the
/// seven the 2240 adds.
pub fn field_formatters() -> HashMap<String, fn(i64) -> String> {
    fn s2vsa(v: i64) -> String {
        if v != 0 {
            "1(ShortToSupply_A!)".into()
        } else {
            String::new()
        }
    }
    fn s2vsb(v: i64) -> String {
        if v != 0 {
            "1(ShortToSupply_B!)".into()
        } else {
            String::new()
        }
    }
    fn adc_temp(v: i64) -> String {
        format!("{:#06x}({:.1}C)", v, (v - 2038) as f64 / 7.7)
    }
    fn adc_vsupply(v: i64) -> String {
        format!("{:#06x}({:.3}V)", v, v as f64 * 0.009732)
    }
    fn adc_ain(v: i64) -> String {
        format!("{:#06x}({:.3}mV)", v, v as f64 * 0.3052)
    }
    fn overvoltage_vth(v: i64) -> String {
        format!("{:#06x}({:.3}V)", v, v as f64 * 0.009732)
    }
    fn overtempprewarning_vth(v: i64) -> String {
        format!("{:#06x}({:.1}C)", v, (v - 2038) as f64 / 7.7)
    }
    let mut formatters = tmc2130_field_formatters();
    formatters.extend([
        ("s2vsa".to_string(), s2vsa as fn(i64) -> String),
        ("s2vsb".to_string(), s2vsb),
        ("adc_temp".to_string(), adc_temp),
        ("adc_vsupply".to_string(), adc_vsupply),
        ("adc_ain".to_string(), adc_ain),
        ("overvoltage_vth".to_string(), overvoltage_vth),
        ("overtempprewarning_vth".to_string(), overtempprewarning_vth),
    ]);
    formatters
}

/// Upstream's `load_config_prefix` for `[tmc2240 <stepper>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let field_helper = Arc::new(FieldHelper::new(
        fields(),
        &SIGNED_FIELDS,
        field_formatters(),
    ));
    // A `uart_pin` selects the UART link, otherwise the SPI chain
    // (`tmc2240.py:352-359`).
    let transport: Arc<dyn TmcTransport> = if config.get("uart_pin", None).is_ok() {
        Arc::new(TmcUart::new(
            config,
            printer,
            registers(),
            MAX_ADDR,
            TMC_FREQUENCY,
        )?)
    } else {
        Arc::new(TmcSpiChain::new(
            config,
            printer,
            registers(),
            TMC_FREQUENCY,
        )?)
    };
    let current = Arc::new(Tmc2240Current::new(
        config,
        Arc::clone(&field_helper),
        Arc::clone(&transport),
    )?);
    let driver = TmcDriver::new(
        config,
        printer,
        Arc::clone(&field_helper),
        Arc::clone(&transport),
        current,
        read_registers(),
        None,
    )?;
    // Allow virtual pins to be created.
    TmcVirtualPin::new(config, printer, &driver)?;
    // Setup basic register values.
    wave_table_helper(config, &field_helper)?;
    field_helper.set_config_field(config, "offset_sin90", 0)?;
    stealthchop_helper(config, &field_helper, transport.as_ref())?;
    vcoolthrs_helper(config, &field_helper, transport.as_ref())?;
    vhigh_helper(config, &field_helper, transport.as_ref())?;
    let set = |field: &str, default: i64| field_helper.set_config_field(config, field, default);
    // GCONF
    set("multistep_filt", 1)?;
    // CHOPCONF
    set("toff", 3)?;
    set("hstrt", 5)?;
    set("hend", 2)?;
    set("fd3", 0)?;
    set("disfdcc", 0)?;
    set("chm", 0)?;
    set("tbl", 2)?;
    set("vhighfs", 0)?;
    set("vhighchm", 0)?;
    set("tpfd", 4)?;
    set("diss2g", 0)?;
    set("diss2vs", 0)?;
    // COOLCONF
    set("semin", 0)?;
    set("seup", 0)?;
    set("semax", 0)?;
    set("sedn", 0)?;
    set("seimin", 0)?;
    set("sgt", 0)?;
    set("sfilt", 0)?;
    // IHOLDIRUN
    set("iholddelay", 6)?;
    set("irundelay", 4)?;
    // PWMCONF
    set("pwm_ofs", 29)?;
    set("pwm_grad", 0)?;
    set("pwm_freq", 0)?;
    set("pwm_autoscale", 1)?;
    set("pwm_autograd", 1)?;
    set("freewheel", 0)?;
    set("pwm_reg", 4)?;
    set("pwm_lim", 12)?;
    // TPOWERDOWN
    set("tpowerdown", 10)?;
    // SG4_THRS
    set("sg4_thrs", 0)?;
    set("sg4_angle_offset", 1)?;
    // DRV_CONF
    set("slope_control", 0)?;
    Ok(driver)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex as StdMutex;

    use crate::core::klippy::config::Config;
    use crate::core::klippy::extras::tmc::TmcDriver;
    use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
    use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
    use crate::core::klippy::printer::PrinterState;
    use crate::core::klippy::reactor::TokioReactor;

    fn fake_dictionary() -> Option<std::path::PathBuf> {
        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        dict.is_file().then_some(dict)
    }

    /// A cartesian machine with one `[tmc2240 stepper_x]`, shaped like the
    /// corpus printer configs (`test/klippy/tmc.cfg`). `tmc_body` carries the
    /// transport options, so the same machine exercises the SPI and UART paths.
    fn machine_config(dict: &std::path::Path, tmc_body: &str) -> Config {
        let text = format!(
            "[mcu]\ntest: dict={}\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n\
             max_z_velocity: 5\nmax_z_accel: 100\n\
             [stepper_x]\nstep_pin: PF0\ndir_pin: PF1\nenable_pin: !PD7\nmicrosteps: 16\n\
             rotation_distance: 40\nendstop_pin: ^PE5\nposition_endstop: 0\nposition_max: 200\n\
             homing_speed: 50\n\
             [stepper_y]\nstep_pin: PF6\ndir_pin: !PF7\nenable_pin: !PF2\nmicrosteps: 16\n\
             rotation_distance: 40\nendstop_pin: ^PJ1\nposition_endstop: 0\nposition_max: 200\n\
             homing_speed: 50\n\
             [stepper_z]\nstep_pin: PL3\ndir_pin: PL1\nenable_pin: !PK0\nmicrosteps: 16\n\
             rotation_distance: 8\nendstop_pin: ^PJ2\nposition_endstop: 0\nposition_max: 200\n\
             [tmc2240 stepper_x]\n{tmc_body}",
            dict.display()
        );
        Config::from_text(&text).expect("the config parses").0
    }

    /// The SPI transport options (`test/klippy/tmc.cfg`'s `[tmc2240 stepper_z2]`).
    const SPI_BODY: &str = "cs_pin: PA4\nrun_current: 0.5\n";
    /// A UART transport, as a driver with `uart_pin` is wired.
    const UART_BODY: &str = "uart_pin: PC4\ntx_pin: PC3\nrun_current: 0.5\n";

    /// Load the config, and connect the fake firmware so the stepper lookup and
    /// register init run. The printer is returned for tear-down before
    /// asserting, as `upstream.rs::run_phases` does.
    async fn up_machine(
        dict: &std::path::Path,
        tmc_body: &str,
    ) -> (Arc<Printer>, Result<(), String>) {
        let config = machine_config(dict, tmc_body);
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let printer = Arc::new(Printer::new(reactor));
        let mut start_args = crate::core::klippy::api::StartArgs::collect("tmc2240.cfg", None);
        start_args.debug_output = Some("_test_output".to_string());
        printer.set_start_args(Arc::new(start_args));
        let setup = async {
            printer
                .load_config(&config)
                .map_err(|err| err.to_string())?;
            if tokio::time::timeout(std::time::Duration::from_secs(10), printer.bring_up())
                .await
                .is_err()
            {
                return Err("bring_up timed out".to_string());
            }
            let state = printer.get_state_message();
            if state.category != PrinterState::Ready {
                return Err(format!("not ready: {}", state.message));
            }
            Ok(())
        }
        .await;
        (printer, setup)
    }

    /// The driver object a section registered.
    fn driver(printer: &Arc<Printer>) -> Arc<TmcDriver> {
        printer
            .lookup_object_as::<TmcDriver>("tmc2240 stepper_x")
            .expect("the driver is registered under its section id")
    }

    /// One field, read back through the driver's cache.
    fn field_value(printer: &Arc<Printer>, name: &str) -> i64 {
        driver(printer).fields().get_field(name, None, None)
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_spi_section_loads_and_initializes_its_registers() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, SPI_BODY).await;
        let run: Result<(), String> = async {
            setup?;
            let driver = driver(&printer);
            let fields = driver.fields();
            let field = |name: &str| fields.get_field(name, None, None);
            // CHOPCONF: toff 3, hstrt 5, hend 2, tbl 2, tpfd 4, and the
            // microstep/interpolate codes (`microsteps: 16` → mres 4).
            assert_eq!(field("toff"), 3);
            assert_eq!(field("hstrt"), 5);
            assert_eq!(field("hend"), 2);
            assert_eq!(field("fd3"), 0);
            assert_eq!(field("disfdcc"), 0);
            assert_eq!(field("chm"), 0);
            assert_eq!(field("tbl"), 2);
            assert_eq!(field("vhighfs"), 0);
            assert_eq!(field("vhighchm"), 0);
            assert_eq!(field("tpfd"), 4);
            assert_eq!(field("diss2g"), 0);
            assert_eq!(field("diss2vs"), 0);
            assert_eq!(field("mres"), 4);
            assert_eq!(field("intpol"), 1);
            // GCONF
            assert_eq!(field("multistep_filt"), 1);
            // COOLCONF
            for name in ["semin", "seup", "semax", "sedn", "seimin", "sgt", "sfilt"] {
                assert_eq!(field(name), 0, "{name}");
            }
            // IHOLDIRUN and PWMCONF
            assert_eq!(field("iholddelay"), 6);
            assert_eq!(field("irundelay"), 4);
            assert_eq!(field("pwm_ofs"), 29);
            assert_eq!(field("pwm_grad"), 0);
            assert_eq!(field("pwm_freq"), 0);
            assert_eq!(field("pwm_autoscale"), 1);
            assert_eq!(field("pwm_autograd"), 1);
            assert_eq!(field("freewheel"), 0);
            assert_eq!(field("pwm_reg"), 4);
            assert_eq!(field("pwm_lim"), 12);
            // TPOWERDOWN
            assert_eq!(field("tpowerdown"), 10);
            // SG4_THRS: the sensorless `sg4` default, and its angle offset on.
            assert_eq!(field("sg4_thrs"), 0);
            assert_eq!(field("sg4_angle_offset"), 1);
            // DRV_CONF
            assert_eq!(field("slope_control"), 0);
            // The wave table default, with `offset_sin90` zeroed.
            assert_eq!(field("mslut0"), 0xAAAAB554);
            assert_eq!(field("start_sin90"), 247);
            assert_eq!(field("offset_sin90"), 0);
            // The register cache the connect phase sends. `current_range` is 0
            // and `globalscaler` 185 for `run_current: 0.5` at `rref` 12000.
            let registers: HashMap<String, u32> = driver.registers().into_iter().collect();
            assert_eq!(registers["DRV_CONF"], 0x0);
            assert_eq!(registers["GLOBALSCALER"], 185);
            // ihold 31, irun 31, iholddelay 6, irundelay 4
            assert_eq!(registers["IHOLD_IRUN"], 0x0406_1F1F);
            // toff 3, hstrt 5, hend 2, tbl 2, tpfd 4, mres 4, intpol 1
            assert_eq!(registers["CHOPCONF"], 0x1441_0153);
            // pwm_ofs 29, pwm_autoscale/autograd on, pwm_reg 4, pwm_lim 12
            assert_eq!(registers["PWMCONF"], 0xC40C_001D);
            assert_eq!(registers["TPOWERDOWN"], 10);
            // sg4_thrs 0, sg4_angle_offset 1
            assert_eq!(registers["SG4_THRS"], 0x200);
            // GCONF: multistep_filt only (no `stealthchop_threshold` here).
            assert_eq!(registers["GCONF"], 0x8);
            // The virtual-endstop chip `<driver>_<stepper>`.
            let pins = printer
                .lookup_object_as::<PrinterPins>(PINS_OBJECT)
                .expect("the loader registers `pins`");
            assert!(
                pins.chips().iter().any(|name| name == "tmc2240_stepper_x"),
                "the virtual pin chip is not registered: {:?}",
                pins.chips()
            );
            // `init_registers` sends every cached register without a bus.
            driver
                .init_registers(Some(0.))
                .map_err(|err| err.to_string())?;
            Ok(())
        }
        .await;
        printer.teardown();
        run.expect("the SPI section loads and fills its register cache");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_uart_section_loads() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        // A `uart_pin` swaps the transport for `TmcUart` (`tmc2240.py:352-359`).
        let (printer, setup) = up_machine(&dict, UART_BODY).await;
        let run: Result<(), String> = async {
            setup?;
            let driver = driver(&printer);
            let field = |name: &str| driver.fields().get_field(name, None, None);
            // The register defaults are the same as the SPI path ...
            assert_eq!(field("iholddelay"), 6);
            assert_eq!(field("irundelay"), 4);
            assert_eq!(field("pwm_ofs"), 29);
            // ... and the UART link reports the chip's frequency.
            assert_eq!(driver.transport().get_tmc_frequency(), Some(TMC_FREQUENCY));
            let pins = printer
                .lookup_object_as::<PrinterPins>(PINS_OBJECT)
                .expect("the loader registers `pins`");
            assert!(
                pins.chips().iter().any(|name| name == "tmc2240_stepper_x"),
                "the virtual pin chip is not registered: {:?}",
                pins.chips()
            );
            driver
                .init_registers(Some(0.))
                .map_err(|err| err.to_string())?;
            Ok(())
        }
        .await;
        printer.teardown();
        run.expect("the UART section loads");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_current_options_use_the_rref_model() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        // `run_current: 0.5`, `rref` 12000 → range 0, globalscaler 185,
        // irun/ihold 31.
        let (printer, setup) = up_machine(&dict, SPI_BODY).await;
        let run: Result<(), String> = async {
            setup?;
            assert_eq!(field_value(&printer, "current_range"), 0);
            assert_eq!(field_value(&printer, "globalscaler"), 185);
            assert_eq!(field_value(&printer, "irun"), 31);
            assert_eq!(field_value(&printer, "ihold"), 31);
            // `get_status` reads the current back through the field cache:
            // 185 * 32 * ifs_rms(0) / (256 * 32), i.e. run_current ≈ 0.5004.
            let status = driver(&printer).get_status(0.);
            let run = status["run_current"].as_f64().expect("a number");
            assert!((run - 0.5003).abs() < 1e-3, "run_current {run}");
            Ok(())
        }
        .await;
        printer.teardown();
        run.expect("the rref current model seeds the registers");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn set_tmc_current_rewrites_the_scale() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, SPI_BODY).await;
        let run: Result<(Vec<String>, i64, i64, i64), String> = async {
            setup?;
            let gcode = printer
                .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                .expect("the loader registers `gcode`");
            let replies = Arc::new(StdMutex::new(Vec::<String>::new()));
            {
                let replies = Arc::clone(&replies);
                gcode.register_output_handler(Arc::new(move |line: &str| {
                    replies.lock().unwrap().push(line.to_string());
                }));
            }
            // A smaller current keeps `current_range` at 0 and lands in the
            // 32-bit `GLOBALSCALER` floor: 32 * 7 * ifs_rms(0) / 8192 ≈ 0.0189.
            gcode
                .run_script("SET_TMC_CURRENT STEPPER=stepper_x CURRENT=.02")
                .await
                .map_err(|err| err.to_string())?;
            let captured = replies.lock().unwrap().clone();
            Ok((
                captured,
                field_value(&printer, "globalscaler"),
                field_value(&printer, "irun"),
                field_value(&printer, "ihold"),
            ))
        }
        .await;
        printer.teardown();
        let (replies, globalscaler, irun, ihold) =
            run.expect("the machine loads and the command runs");
        // set_current rewrites `GLOBALSCALER` and `IHOLD_IRUN` in place.
        assert_eq!(globalscaler, 32);
        assert_eq!(irun, 6);
        assert_eq!(ihold, 6);
        assert!(
            replies
                .iter()
                .any(|line| line.contains("Run Current: 0.02A")),
            "no reply has the run current: {replies:?}"
        );
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_command_surface_matches_upstream() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, SPI_BODY).await;
        let run: Result<Vec<String>, String> = async {
            setup?;
            let gcode = printer
                .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                .expect("the loader registers `gcode`");
            let help = gcode.command_help();
            for (cmd, desc) in [
                ("SET_TMC_FIELD", "Set a register field of a TMC driver"),
                ("INIT_TMC", "Initialize TMC stepper driver registers"),
                ("SET_TMC_CURRENT", "Set the current of a TMC driver"),
                ("DUMP_TMC", "Read and display TMC stepper driver registers"),
            ] {
                assert_eq!(help.get(cmd).map(String::as_str), Some(desc), "{cmd}");
            }
            let replies = Arc::new(StdMutex::new(Vec::<String>::new()));
            {
                let replies = Arc::clone(&replies);
                gcode.register_output_handler(Arc::new(move |line: &str| {
                    replies.lock().unwrap().push(line.to_string());
                }));
            }
            // Each command runs without a bus. `SET_TMC_CURRENT`'s bound is
            // the current range's full-scale RMS (ifs_rms(0) ≈ 0.6924).
            gcode
                .run_script("SET_TMC_CURRENT STEPPER=stepper_x CURRENT=.5")
                .await
                .map_err(|err| err.to_string())?;
            gcode
                .run_script("SET_TMC_FIELD STEPPER=stepper_x FIELD=SGT VALUE=3")
                .await
                .map_err(|err| err.to_string())?;
            gcode
                .run_script("INIT_TMC STEPPER=stepper_x")
                .await
                .map_err(|err| err.to_string())?;
            gcode
                .run_script("DUMP_TMC STEPPER=stepper_x")
                .await
                .map_err(|err| err.to_string())?;
            let captured = replies.lock().unwrap().clone();
            Ok(captured)
        }
        .await;
        printer.teardown();
        let replies = run.expect("the machine loads and the commands run");
        assert!(
            replies.iter().any(|line| line.contains("Run Current:")),
            "no reply has the run current: {replies:?}"
        );
        assert!(
            replies
                .iter()
                .any(|line| line.contains("========== Write-only registers ==========")),
            "DUMP_TMC has no write-only section: {replies:?}"
        );
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn fileoutput_never_touches_the_bus() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, SPI_BODY).await;
        let run: Result<(), String> = async {
            setup?;
            assert!(printer.is_fileoutput());
            let driver = driver(&printer);
            // The transport reports 0 for every read and accepts (drops) every
            // write without sending a frame.
            assert_eq!(driver.transport().get_register("DRV_STATUS").unwrap(), 0);
            assert_eq!(driver.transport().get_register("GCONF").unwrap(), 0);
            driver
                .transport()
                .set_register("GCONF", 0x1234, None)
                .expect("a write is dropped, not sent");
            assert_eq!(driver.transport().get_tmc_frequency(), Some(TMC_FREQUENCY));
            assert!(driver.transport().name_to_reg().contains_key("DRV_STATUS"));
            assert!(driver.transport().name_to_reg().contains_key("ADC_TEMP"));
            Ok(())
        }
        .await;
        printer.teardown();
        run.expect("reads answer 0 and writes are dropped under file output");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_phase_offset_interface_is_callable() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, SPI_BODY).await;
        let run: Result<(), String> = async {
            setup?;
            let (offset, phases) = driver(&printer).get_phase_offset();
            assert_eq!(offset, None);
            // `(256 >> mres) * 4` with `microsteps: 16` → mres 4.
            assert_eq!(phases, 64);
            Ok(())
        }
        .await;
        printer.teardown();
        run.expect("get_phase_offset is on the object");
    }

    #[test]
    fn the_adc_formatters_match_upstream() {
        let formatters = field_formatters();
        let f = |name: &str| *formatters.get(name).expect(name);
        // `(v - 2038) / 7.7` → "%.1fC".
        assert_eq!(f("adc_temp")(0), "0x0000(-264.7C)");
        assert_eq!(f("overtempprewarning_vth")(2115), "0x0843(10.0C)");
        // `v * 0.009732` → "%.3fV".
        assert_eq!(f("adc_vsupply")(1000), "0x03e8(9.732V)");
        assert_eq!(f("overvoltage_vth")(1000), "0x03e8(9.732V)");
        // `v * 0.3052` → "%.3fmV".
        assert_eq!(f("adc_ain")(1000), "0x03e8(305.200mV)");
        // The short-to-supply flags only print when set.
        assert_eq!(f("s2vsa")(1), "1(ShortToSupply_A!)");
        assert_eq!(f("s2vsa")(0), "");
        assert_eq!(f("s2vsb")(1), "1(ShortToSupply_B!)");
        // The 2130's formatters carry through.
        assert_eq!(f("mres")(4), "4(16usteps)");
    }
}
