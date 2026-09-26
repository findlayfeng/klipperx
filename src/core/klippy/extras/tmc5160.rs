//! `[tmc5160 <stepper>]` — the TMC5160 SPI driver (upstream's
//! `klippy/extras/tmc5160.py`).
//!
//! The chip is the high-current member of the SPI family: it reaches the bus
//! through the shared chain transport
//! ([`crate::core::klippy::extras::tmc_spi::TmcSpiChain`], the same 5-byte frame
//! as `tmc2130`) and scales its current with `GLOBALSCALER` instead of the
//! 2208/2209's `vsense`
//! ([`crate::core::klippy::extras::tmc::Tmc5160Current`]). Its `TSTEP` frequency
//! is 12 MHz, and because it has a wave table it initializes one before the
//! per-field defaults.
//!
//! The register/field tables are this module's; the behaviour (fields, G-Code
//! commands, the current model) lives in
//! [`crate::core::klippy::extras::tmc`].

use std::collections::HashMap;
use std::sync::Arc;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::tmc::{
    stealthchop_helper, vcoolthrs_helper, vhigh_helper, wave_table_helper, FieldHelper,
    Tmc5160Current, TmcDriver, TmcTransport, TmcVirtualPin,
};
use crate::core::klippy::extras::tmc_spi::TmcSpiChain;
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

// SPI mode 3 at 4 MHz, as the chain's other chips.
section!("tmc5160", order = 40, prefix = load_config_prefix);

/// The chip's internal TSTEP frequency (`TMC_FREQUENCY`).
pub const TMC_FREQUENCY: f64 = 12_000_000.;

/// Register name → bus address (`Registers`).
pub fn registers() -> HashMap<String, u8> {
    [
        ("GCONF", 0x00),
        ("GSTAT", 0x01),
        ("IFCNT", 0x02),
        ("SLAVECONF", 0x03),
        ("IOIN", 0x04),
        ("X_COMPARE", 0x05),
        ("OTP_READ", 0x07),
        ("FACTORY_CONF", 0x08),
        ("SHORT_CONF", 0x09),
        ("DRV_CONF", 0x0A),
        ("GLOBALSCALER", 0x0B),
        ("OFFSET_READ", 0x0C),
        ("IHOLD_IRUN", 0x10),
        ("TPOWERDOWN", 0x11),
        ("TSTEP", 0x12),
        ("TPWMTHRS", 0x13),
        ("TCOOLTHRS", 0x14),
        ("THIGH", 0x15),
        ("RAMPMODE", 0x20),
        ("XACTUAL", 0x21),
        ("VACTUAL", 0x22),
        ("VSTART", 0x23),
        ("A1", 0x24),
        ("V1", 0x25),
        ("AMAX", 0x26),
        ("VMAX", 0x27),
        ("DMAX", 0x28),
        ("D1", 0x2A),
        ("VSTOP", 0x2B),
        ("TZEROWAIT", 0x2C),
        ("XTARGET", 0x2D),
        ("VDCMIN", 0x33),
        ("SW_MODE", 0x34),
        ("RAMP_STAT", 0x35),
        ("XLATCH", 0x36),
        ("ENCMODE", 0x38),
        ("X_ENC", 0x39),
        ("ENC_CONST", 0x3A),
        ("ENC_STATUS", 0x3B),
        ("ENC_LATCH", 0x3C),
        ("ENC_DEVIATION", 0x3D),
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
        ("MSCNT", 0x6A),
        ("MSCURACT", 0x6B),
        ("CHOPCONF", 0x6C),
        ("COOLCONF", 0x6D),
        ("DCCTRL", 0x6E),
        ("DRV_STATUS", 0x6F),
        ("PWMCONF", 0x70),
        ("PWM_SCALE", 0x71),
        ("PWM_AUTO", 0x72),
        ("LOST_STEPS", 0x73),
    ]
    .into_iter()
    .map(|(name, addr)| (name.to_string(), addr))
    .collect()
}

/// The registers `DUMP_TMC` reads (`ReadRegisters`).
pub fn read_registers() -> Vec<String> {
    [
        "GCONF",
        "CHOPCONF",
        "GSTAT",
        "DRV_STATUS",
        "FACTORY_CONF",
        "IOIN",
        "LOST_STEPS",
        "MSCNT",
        "MSCURACT",
        "OTP_READ",
        "PWM_SCALE",
        "PWM_AUTO",
        "TSTEP",
    ]
    .into_iter()
    .map(str::to_string)
    .collect()
}

/// The fields whose register value is two's-complement signed (`SignedFields`).
///
/// The two ramp targets (`xactual`/`vactual`) and the torque reading
/// (`sg_result`'s companion) are the additions over `tmc2130`.
pub const SIGNED_FIELDS: [&str; 6] = [
    "cur_a",
    "cur_b",
    "sgt",
    "xactual",
    "vactual",
    "pwm_scale_auto",
];

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
        "semin": 0x0F, "seup": 0x03 << 5, "semax": 0x0F << 8, "sedn": 0x03 << 13,
        "seimin": 0x01 << 15, "sgt": 0x7F << 16, "sfilt": 0x01 << 24
    });
    reg!("CHOPCONF", {
        "toff": 0x0F, "hstrt": 0x07 << 4, "hend": 0x0F << 7, "fd3": 0x01 << 11,
        "disfdcc": 0x01 << 12, "chm": 0x01 << 14, "tbl": 0x03 << 15, "vhighfs": 0x01 << 18,
        "vhighchm": 0x01 << 19, "tpfd": 0x0F << 20, "mres": 0x0F << 24, "intpol": 0x01 << 28,
        "dedge": 0x01 << 29, "diss2g": 0x01 << 30, "diss2vs": 0x01 << 31
    });
    reg!("DRV_CONF", {
        "bbmtime": 0x1F, "bbmclks": 0x0F << 8, "otselect": 0x03 << 16,
        "drvstrength": 0x03 << 18, "filt_isense": 0x03 << 20
    });
    reg!("DRV_STATUS", {
        "sg_result": 0x3FF, "s2vsa": 0x01 << 12, "s2vsb": 0x01 << 13, "stealth": 0x01 << 14,
        "fsactive": 0x01 << 15, "cs_actual": 0x1F << 16, "stallguard": 0x01 << 24,
        "ot": 0x01 << 25, "otpw": 0x01 << 26, "s2ga": 0x01 << 27, "s2gb": 0x01 << 28,
        "ola": 0x01 << 29, "olb": 0x01 << 30, "stst": 0x01 << 31
    });
    reg!("FACTORY_CONF", { "factory_conf": 0x1F });
    reg!("GCONF", {
        "recalibrate": 0x01, "faststandstill": 0x01 << 1, "en_pwm_mode": 0x01 << 2,
        "multistep_filt": 0x01 << 3, "shaft": 0x01 << 4, "diag0_error": 0x01 << 5,
        "diag0_otpw": 0x01 << 6, "diag0_stall": 0x01 << 7, "diag1_stall": 0x01 << 8,
        "diag1_index": 0x01 << 9, "diag1_onstate": 0x01 << 10, "diag1_steps_skipped": 0x01 << 11,
        "diag0_int_pushpull": 0x01 << 12, "diag1_poscomp_pushpull": 0x01 << 13,
        "small_hysteresis": 0x01 << 14, "stop_enable": 0x01 << 15, "direct_mode": 0x01 << 16,
        "test_mode": 0x01 << 17
    });
    reg!("GSTAT", { "reset": 0x01, "drv_err": 0x01 << 1, "uv_cp": 0x01 << 2 });
    reg!("GLOBALSCALER", { "globalscaler": 0xFF });
    reg!("IHOLD_IRUN", {
        "ihold": 0x1F, "irun": 0x1F << 8, "iholddelay": 0x0F << 16
    });
    reg!("IOIN", {
        "refl_step": 0x01, "refr_dir": 0x01 << 1, "encb_dcen_cfg4": 0x01 << 2,
        "enca_dcin_cfg5": 0x01 << 3, "drv_enn": 0x01 << 4, "enc_n_dco_cfg6": 0x01 << 5,
        "sd_mode": 0x01 << 6, "swcomp_in": 0x01 << 7, "version": 0xFF << 24
    });
    reg!("LOST_STEPS", { "lost_steps": 0xfffff });
    reg!("MSLUT0", { "mslut0": 0xffffffff });
    reg!("MSLUT1", { "mslut1": 0xffffffff });
    reg!("MSLUT2", { "mslut2": 0xffffffff });
    reg!("MSLUT3", { "mslut3": 0xffffffff });
    reg!("MSLUT4", { "mslut4": 0xffffffff });
    reg!("MSLUT5", { "mslut5": 0xffffffff });
    reg!("MSLUT6", { "mslut6": 0xffffffff });
    reg!("MSLUT7", { "mslut7": 0xffffffff });
    reg!("MSLUTSEL", {
        "x3": 0xFF << 24, "x2": 0xFF << 16, "x1": 0xFF << 8, "w3": 0x03 << 6, "w2": 0x03 << 4,
        "w1": 0x03 << 2, "w0": 0x03
    });
    reg!("MSLUTSTART", { "start_sin": 0xFF, "start_sin90": 0xFF << 16 });
    reg!("MSCNT", { "mscnt": 0x3ff });
    reg!("MSCURACT", { "cur_a": 0x1ff, "cur_b": 0x1ff << 16 });
    reg!("OTP_READ", {
        "otp_fclktrim": 0x1f, "otp_s2_level": 0x01 << 5, "otp_bbm": 0x01 << 6,
        "otp_tbl": 0x01 << 7
    });
    reg!("PWM_AUTO", { "pwm_ofs_auto": 0xff, "pwm_grad_auto": 0xff << 16 });
    reg!("PWMCONF", {
        "pwm_ofs": 0xFF, "pwm_grad": 0xFF << 8, "pwm_freq": 0x03 << 16,
        "pwm_autoscale": 0x01 << 18, "pwm_autograd": 0x01 << 19, "freewheel": 0x03 << 20,
        "pwm_reg": 0x0F << 24, "pwm_lim": 0x0F << 28
    });
    reg!("PWM_SCALE", { "pwm_scale_sum": 0xff, "pwm_scale_auto": 0x1ff << 16 });
    reg!("TPOWERDOWN", { "tpowerdown": 0xff });
    reg!("TPWMTHRS", { "tpwmthrs": 0xfffff });
    reg!("TCOOLTHRS", { "tcoolthrs": 0xfffff });
    reg!("TSTEP", { "tstep": 0xfffff });
    reg!("THIGH", { "thigh": 0xfffff });
    fields
}

/// The `DUMP_TMC` field formatters (`FieldFormatters`, from `tmc2130` plus the
/// two supply-short flags this chip adds).
pub fn field_formatters() -> HashMap<String, fn(i64) -> String> {
    fn extvref(v: i64) -> String {
        if v != 0 {
            "1(ExtVREF)".into()
        } else {
            String::new()
        }
    }
    fn reverse(v: i64) -> String {
        if v != 0 {
            "1(Reverse)".into()
        } else {
            String::new()
        }
    }
    fn reset(v: i64) -> String {
        if v != 0 {
            "1(Reset)".into()
        } else {
            String::new()
        }
    }
    fn drv_err(v: i64) -> String {
        if v != 0 {
            "1(ErrorShutdown!)".into()
        } else {
            String::new()
        }
    }
    fn uv_cp(v: i64) -> String {
        if v != 0 {
            "1(Undervoltage!)".into()
        } else {
            String::new()
        }
    }
    fn version(v: i64) -> String {
        format!("{:#x}", v)
    }
    fn mres(v: i64) -> String {
        format!("{v}({}usteps)", 0x100i64 >> v)
    }
    fn otpw(v: i64) -> String {
        if v != 0 {
            "1(OvertempWarning!)".into()
        } else {
            String::new()
        }
    }
    fn ot(v: i64) -> String {
        if v != 0 {
            "1(OvertempError!)".into()
        } else {
            String::new()
        }
    }
    fn s2ga(v: i64) -> String {
        if v != 0 {
            "1(ShortToGND_A!)".into()
        } else {
            String::new()
        }
    }
    fn s2gb(v: i64) -> String {
        if v != 0 {
            "1(ShortToGND_B!)".into()
        } else {
            String::new()
        }
    }
    fn ola(v: i64) -> String {
        if v != 0 {
            "1(OpenLoad_A!)".into()
        } else {
            String::new()
        }
    }
    fn olb(v: i64) -> String {
        if v != 0 {
            "1(OpenLoad_B!)".into()
        } else {
            String::new()
        }
    }
    fn cs_actual(v: i64) -> String {
        if v != 0 {
            v.to_string()
        } else {
            "0(Reset?)".to_string()
        }
    }
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
    HashMap::from([
        ("i_scale_analog".to_string(), extvref as fn(i64) -> String),
        ("shaft".to_string(), reverse),
        ("reset".to_string(), reset),
        ("drv_err".to_string(), drv_err),
        ("uv_cp".to_string(), uv_cp),
        ("version".to_string(), version),
        ("mres".to_string(), mres),
        ("otpw".to_string(), otpw),
        ("ot".to_string(), ot),
        ("s2ga".to_string(), s2ga),
        ("s2gb".to_string(), s2gb),
        ("ola".to_string(), ola),
        ("olb".to_string(), olb),
        ("cs_actual".to_string(), cs_actual),
        ("s2vsa".to_string(), s2vsa),
        ("s2vsb".to_string(), s2vsb),
    ])
}

/// Upstream's `load_config_prefix` for `[tmc5160 <stepper>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let field_helper = Arc::new(FieldHelper::new(
        fields(),
        &SIGNED_FIELDS,
        field_formatters(),
    ));
    let transport: Arc<dyn TmcTransport> = Arc::new(TmcSpiChain::new(
        config,
        printer,
        registers(),
        TMC_FREQUENCY,
    )?);
    let current = Arc::new(Tmc5160Current::new(
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
    // DRV_CONF
    set("drvstrength", 0)?;
    set("bbmclks", 4)?;
    set("bbmtime", 0)?;
    set("filt_isense", 0)?;
    // IHOLDIRUN
    set("iholddelay", 6)?;
    // PWMCONF
    set("pwm_ofs", 30)?;
    set("pwm_grad", 0)?;
    set("pwm_freq", 0)?;
    set("pwm_autoscale", 1)?;
    set("pwm_autograd", 1)?;
    set("freewheel", 0)?;
    set("pwm_reg", 4)?;
    set("pwm_lim", 12)?;
    // TPOWERDOWN
    set("tpowerdown", 10)?;
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
    use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
    use crate::core::klippy::printer::PrinterState;
    use crate::core::klippy::reactor::TokioReactor;

    fn fake_dictionary() -> Option<std::path::PathBuf> {
        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        dict.is_file().then_some(dict)
    }

    /// A cartesian machine with three steppers, shaped like the corpus printer
    /// configs. `tmc_sections` carries the `[tmc5160 …]` text.
    fn machine_config(dict: &std::path::Path, tmc_sections: &str, endstop_pin: &str) -> Config {
        let text = format!(
            "[mcu]\ntest: dict={}\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n\
             max_z_velocity: 5\nmax_z_accel: 100\n\
             [stepper_x]\nstep_pin: PF0\ndir_pin: PF1\nenable_pin: !PD7\nmicrosteps: 16\n\
             rotation_distance: 40\nendstop_pin: {endstop_pin}\nposition_endstop: 0\nposition_max: 200\n\
             homing_speed: 50\n\
             [stepper_y]\nstep_pin: PF6\ndir_pin: !PF7\nenable_pin: !PF2\nmicrosteps: 16\n\
             rotation_distance: 40\nendstop_pin: ^PJ1\nposition_endstop: 0\nposition_max: 200\n\
             homing_speed: 50\n\
             [stepper_z]\nstep_pin: PL3\ndir_pin: PL1\nenable_pin: !PK0\nmicrosteps: 16\n\
             rotation_distance: 8\nendstop_pin: ^PJ2\nposition_endstop: 0\nposition_max: 200\n{tmc_sections}",
            dict.display()
        );
        Config::from_text(&text).expect("the config parses").0
    }

    /// The options a plain `[tmc5160 stepper_x]` section carries.
    const PLAIN: &str = "[tmc5160 stepper_x]\ncs_pin: PK1\nrun_current: 1.0\n";

    /// Load the config, and connect the fake firmware so the stepper lookup and
    /// register init run. The printer is returned for teardown before asserting,
    /// as `upstream.rs::run_phases` does.
    async fn up_machine(
        dict: &std::path::Path,
        tmc_sections: &str,
        endstop_pin: &str,
    ) -> (Arc<Printer>, Result<(), String>) {
        let config = machine_config(dict, tmc_sections, endstop_pin);
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let printer = Arc::new(Printer::new(reactor));
        let mut start_args = crate::core::klippy::api::StartArgs::collect("tmc5160.cfg", None);
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
            .lookup_object_as::<TmcDriver>("tmc5160 stepper_x")
            .expect("the driver is registered under its section id")
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_section_loads_and_names_its_chip() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        // The stepper's virtual endstop resolves `tmc5160_stepper_x`, so the
        // section is only loadable if the chip registered under that name.
        let sections = "[tmc5160 stepper_x]\ncs_pin: PK1\ndiag0_pin: ^PE2\nrun_current: 1.0\n";
        let (printer, setup) =
            up_machine(&dict, sections, "tmc5160_stepper_x:virtual_endstop").await;
        printer.teardown();
        setup.expect("the virtual-endstop config loads and connects");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_default_registers_match_upstream() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, PLAIN, "^PE5").await;
        let run: Result<(), String> = async {
            setup?;
            let driver = driver(&printer);
            let fields = driver.fields();
            // The raw registers the section writes at connect, with
            // `microsteps: 16` (mres 4) and `interpolate` defaulting true.
            assert_eq!(fields.register_value("GCONF"), Some(0x08));
            assert_eq!(fields.register_value("CHOPCONF"), Some(0x1441_0153));
            assert_eq!(fields.register_value("DRV_CONF"), Some(0x400));
            assert_eq!(fields.register_value("PWMCONF"), Some(0xc40c_001e));
            assert_eq!(fields.register_value("TPOWERDOWN"), Some(10));
            // The named fields behind those words.
            for (field, expected) in [
                ("toff", 3),
                ("hstrt", 5),
                ("hend", 2),
                ("fd3", 0),
                ("disfdcc", 0),
                ("chm", 0),
                ("tbl", 2),
                ("vhighfs", 0),
                ("vhighchm", 0),
                ("tpfd", 4),
                ("diss2g", 0),
                ("diss2vs", 0),
                ("mres", 4),
                ("intpol", 1),
            ] {
                assert_eq!(fields.get_field(field, None, None), expected, "{field}");
            }
            for (field, expected) in [
                ("multistep_filt", 1),
                ("semin", 0),
                ("seup", 0),
                ("semax", 0),
                ("sedn", 0),
                ("seimin", 0),
                ("sgt", 0),
                ("sfilt", 0),
                ("drvstrength", 0),
                ("bbmclks", 4),
                ("bbmtime", 0),
                ("filt_isense", 0),
                ("iholddelay", 6),
                ("pwm_ofs", 30),
                ("pwm_grad", 0),
                ("pwm_freq", 0),
                ("pwm_autoscale", 1),
                ("pwm_autograd", 1),
                ("freewheel", 0),
                ("pwm_reg", 4),
                ("pwm_lim", 12),
                ("tpowerdown", 10),
            ] {
                assert_eq!(fields.get_field(field, None, None), expected, "{field}");
            }
            // The wave table is loaded before the per-field defaults.
            assert_eq!(fields.register_value("MSLUT0"), Some(0xAAAAB554));
            assert_eq!(fields.register_value("MSLUTSTART"), Some(0x00F7_0000));
            Ok(())
        }
        .await;
        printer.teardown();
        run.expect("the default registers match upstream");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_current_scales_the_chip() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        // The Duet3 config's numbers: `run_current: 1`, `sense_resistor: 0.05`.
        let sections = "[tmc5160 stepper_x]\ncs_pin: PK1\nrun_current: 1\nsense_resistor: 0.05\n";
        let (printer, setup) = up_machine(&dict, sections, "^PE5").await;
        let run: Result<(), String> = async {
            setup?;
            let driver = driver(&printer);
            let fields = driver.fields();
            assert_eq!(fields.get_field("globalscaler", None, None), 56);
            assert_eq!(fields.get_field("irun", None, None), 31);
            assert_eq!(fields.get_field("ihold", None, None), 31);
            assert_eq!(fields.register_value("GLOBALSCALER"), Some(56));
            assert_eq!(fields.register_value("IHOLD_IRUN"), Some(0x6_1f1f));
            Ok(())
        }
        .await;
        printer.teardown();
        run.expect("the current converts to the chip's GLOBALSCALER model");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn fileoutput_never_touches_the_bus() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, PLAIN, "^PE5").await;
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
            assert!(driver.transport().name_to_reg().contains_key("DRV_STATUS"));
            Ok(())
        }
        .await;
        printer.teardown();
        run.expect("reads answer 0 and writes are dropped under file output");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_command_surface_matches_upstream() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, PLAIN, "^PE5").await;
        let run: Result<Vec<String>, String> = async {
            setup?;
            let gcode = printer
                .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                .expect("the loader registers `gcode`");
            // 1. All four mux commands exist, with upstream's help text.
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
            // 2. An unknown field is refused with upstream's wording.
            let err = gcode
                .run_script("SET_TMC_FIELD STEPPER=stepper_x FIELD=nosuch VALUE=1")
                .await
                .expect_err("an unknown field is refused");
            assert_eq!(err.to_string(), "Unknown field name 'nosuch'");
            // 3. A field write and a register read both run.
            gcode
                .run_script("SET_TMC_FIELD STEPPER=stepper_x FIELD=toff VALUE=6")
                .await
                .map_err(|err| err.to_string())?;
            assert_eq!(
                driver(&printer).fields().get_field("toff", None, None),
                6,
                "SET_TMC_FIELD stored the value"
            );
            gcode
                .run_script("DUMP_TMC STEPPER=stepper_x REGISTER=CHOPCONF")
                .await
                .map_err(|err| err.to_string())?;
            // 4. SET_TMC_CURRENT reports upstream's run-current line.
            gcode
                .run_script("SET_TMC_CURRENT STEPPER=stepper_x CURRENT=.7")
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
            replies
                .iter()
                .any(|line| line.contains("Run Current: 0.69A")),
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
    async fn the_error_check_never_shuts_the_printer_down() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, PLAIN, "^PE5").await;
        let run: Result<(), String> = async {
            setup?;
            let driver = driver(&printer);
            driver
                .error_check()
                .check_once()
                .map_err(|e| e.to_string())?;
            assert_eq!(
                printer.get_state_message().category,
                PrinterState::Ready,
                "the error check must not shut the printer down"
            );
            Ok(())
        }
        .await;
        printer.teardown();
        run.expect("a check under file output reports no fault");
    }

    // -- the SPI chain -----------------------------------------------------

    /// The two sections of a Duet3-style chain, on one chip select, each with
    /// its own length and position so the disagreement cases can be written.
    fn chain_sections(length: (i64, i64), position: (i64, i64)) -> String {
        let (length_x, length_y) = length;
        let (position_x, position_y) = position;
        format!(
            "[tmc5160 stepper_x]\ncs_pin: PK1\nspi_bus: spi\nchain_length: {length_x}\n\
             chain_position: {position_x}\nrun_current: 1\nsense_resistor: 0.05\n\
             [tmc5160 stepper_y]\ncs_pin: PK1\nspi_bus: spi\nchain_length: {length_y}\n\
             chain_position: {position_y}\nrun_current: 1\nsense_resistor: 0.05\n"
        )
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn a_chain_shares_one_chip_select() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        // The Duet3-6HC's numbers: a length-6 chain, positions 1 and 2.
        let (printer, setup) = up_machine(&dict, &chain_sections((6, 6), (1, 2)), "^PE5").await;
        printer.teardown();
        setup.expect("both sections of the chain load and connect");
    }

    #[test]
    fn a_chain_rejects_a_mismatched_length() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        // The sections agree here, so the chain loads.
        let config = machine_config(&dict, &chain_sections((6, 6), (1, 2)), "^PE5");
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        printer
            .load_config(&config)
            .expect("a matching chain loads");
        // A second section on the same chip select disagreeing on the length.
        let config = machine_config(&dict, &chain_sections((6, 5), (1, 2)), "^PE5");
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let err = printer
            .load_config(&config)
            .expect_err("a different length is refused");
        assert_eq!(err.to_string(), "TMC SPI chain must have same length");
    }

    #[test]
    fn a_chain_rejects_a_duplicate_position() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let config = machine_config(&dict, &chain_sections((6, 6), (1, 1)), "^PE5");
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let err = printer
            .load_config(&config)
            .expect_err("a repeated position is refused");
        assert_eq!(
            err.to_string(),
            "TMC SPI chain can not have duplicate position"
        );
    }

    #[test]
    fn an_out_of_range_chain_position_is_refused() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let config = machine_config(
            &dict,
            "[tmc5160 stepper_x]\ncs_pin: PK1\nchain_length: 3\nchain_position: 4\n\
             run_current: 1\n",
            "^PE5",
        );
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let err = printer
            .load_config(&config)
            .expect_err("a position past the chain is refused");
        assert_eq!(
            err.to_string(),
            "Option 'chain_position' in section 'tmc5160 stepper_x' must have maximum of 3"
        );
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn a_chain_without_a_length_is_a_single_device() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        // No `chain_length`: upstream's `lookup_tmc_spi_chain` reads no
        // `chain_position` either, and the bus is a plain single-device one.
        let (printer, setup) = up_machine(&dict, PLAIN, "^PE5").await;
        printer.teardown();
        setup.expect("a section without a chain loads and connects");
    }
}
