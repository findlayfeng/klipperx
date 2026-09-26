//! `[tmc2130 <stepper>]` — the TMC2130 SPI driver (upstream's
//! `klippy/extras/tmc2130.py`).
//!
//! The first SPI TMC driver: it spells out the register/field tables upstream's
//! `tmc2208`/`tmc2209` build on, and reaches its chip through the 5-byte daisy-
//! chain transport ([`TmcSpiChain`]). It is also the driver whose `[stepper_x]`
//! can name `tmc2130_stepper_x:virtual_endstop` — sensorless homing off the
//! chip's `diag0_pin`/`diag1_pin`, through [`TmcVirtualPin`].
//!
//! Unlike the UART chips, the 2130's current model is the plain
//! [`TmcCurrent`] (`run_current`/`hold_current`/`sense_resistor` →
//! `vsense`/`irun`/`ihold`), and its setup also loads the microstep wave table
//! ([`wave_table_helper`]) before the `CHOPCONF`/`COOLCONF`/`PWMCONF` defaults.
//!
//! # What is not here
//!
//! The `tmc/stallguard_dump` bulk endpoint (`tmc.TMCStallguardDump`,
//! `tmc.py:228-317`) is not registered, so `available_sensors` does not list it
//! and the `stallguard_dump` query is unavailable. Nothing else is missing.

use std::collections::HashMap;
use std::sync::Arc;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::tmc::{
    stealthchop_helper, vcoolthrs_helper, vhigh_helper, wave_table_helper, FieldHelper, TmcCurrent,
    TmcDriver, TmcTransport, TmcVirtualPin,
};
use crate::core::klippy::extras::tmc_spi::TmcSpiChain;
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

// Loaded after the pins and the MCU's SPI resource; alongside the other
// `order = 40` device sections.
section!("tmc2130", order = 40, prefix = load_config_prefix);

/// The chip's internal TSTEP frequency (`TMC_FREQUENCY`).
pub const TMC_FREQUENCY: f64 = 13_200_000.;

/// Register name → SPI address (`Registers`).
pub fn registers() -> HashMap<String, u8> {
    [
        ("GCONF", 0x00),
        ("GSTAT", 0x01),
        ("IOIN", 0x04),
        ("IHOLD_IRUN", 0x10),
        ("TPOWERDOWN", 0x11),
        ("TSTEP", 0x12),
        ("TPWMTHRS", 0x13),
        ("TCOOLTHRS", 0x14),
        ("THIGH", 0x15),
        ("XDIRECT", 0x2d),
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
        ("DCCTRL", 0x6e),
        ("DRV_STATUS", 0x6f),
        ("PWMCONF", 0x70),
        ("PWM_SCALE", 0x71),
        ("ENCM_CTRL", 0x72),
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
        "GSTAT",
        "IOIN",
        "TSTEP",
        "XDIRECT",
        "MSCNT",
        "MSCURACT",
        "CHOPCONF",
        "DRV_STATUS",
        "PWM_SCALE",
        "LOST_STEPS",
    ]
    .into_iter()
    .map(str::to_string)
    .collect()
}

/// The fields whose register value is two's-complement signed (`SignedFields`).
pub const SIGNED_FIELDS: [&str; 3] = ["cur_a", "cur_b", "sgt"];

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
    reg!("GCONF", {
        "i_scale_analog": 0x01, "internal_rsense": 0x01 << 1, "en_pwm_mode": 0x01 << 2,
        "enc_commutation": 0x01 << 3, "shaft": 0x01 << 4, "diag0_error": 0x01 << 5,
        "diag0_otpw": 0x01 << 6, "diag0_stall": 0x01 << 7, "diag1_stall": 0x01 << 8,
        "diag1_index": 0x01 << 9, "diag1_onstate": 0x01 << 10, "diag1_steps_skipped": 0x01 << 11,
        "diag0_int_pushpull": 0x01 << 12, "diag1_pushpull": 0x01 << 13,
        "small_hysteresis": 0x01 << 14, "stop_enable": 0x01 << 15, "direct_mode": 0x01 << 16,
        "test_mode": 0x01 << 17
    });
    reg!("GSTAT", { "reset": 0x01, "drv_err": 0x01 << 1, "uv_cp": 0x01 << 2 });
    reg!("IOIN", {
        "step": 0x01, "dir": 0x01 << 1, "dcen_cfg4": 0x01 << 2, "dcin_cfg5": 0x01 << 3,
        "drv_enn_cfg6": 0x01 << 4, "dco": 0x01 << 5, "version": 0xff << 24
    });
    reg!("IHOLD_IRUN", {
        "ihold": 0x1f, "irun": 0x1f << 8, "iholddelay": 0x0f << 16
    });
    reg!("TPOWERDOWN", { "tpowerdown": 0xff });
    reg!("TSTEP", { "tstep": 0xfffff });
    reg!("TPWMTHRS", { "tpwmthrs": 0xfffff });
    reg!("TCOOLTHRS", { "tcoolthrs": 0xfffff });
    reg!("THIGH", { "thigh": 0xfffff });
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
        "start_sin": 0xff, "start_sin90": 0xff << 16
    });
    reg!("MSCNT", { "mscnt": 0x3ff });
    reg!("MSCURACT", { "cur_a": 0x1ff, "cur_b": 0x1ff << 16 });
    reg!("CHOPCONF", {
        "toff": 0x0f, "hstrt": 0x07 << 4, "hend": 0x0f << 7, "fd3": 0x01 << 11,
        "disfdcc": 0x01 << 12, "rndtf": 0x01 << 13, "chm": 0x01 << 14, "tbl": 0x03 << 15,
        "vsense": 0x01 << 17, "vhighfs": 0x01 << 18, "vhighchm": 0x01 << 19, "sync": 0x0f << 20,
        "mres": 0x0f << 24, "intpol": 0x01 << 28, "dedge": 0x01 << 29, "diss2g": 0x01 << 30
    });
    reg!("COOLCONF", {
        "semin": 0x0f, "seup": 0x03 << 5, "semax": 0x0f << 8, "sedn": 0x03 << 13,
        "seimin": 0x01 << 15, "sgt": 0x7f << 16, "sfilt": 0x01 << 24
    });
    reg!("DRV_STATUS", {
        "sg_result": 0x3ff, "fsactive": 0x01 << 15, "cs_actual": 0x1f << 16,
        "stallguard": 0x01 << 24, "ot": 0x01 << 25, "otpw": 0x01 << 26, "s2ga": 0x01 << 27,
        "s2gb": 0x01 << 28, "ola": 0x01 << 29, "olb": 0x01 << 30, "stst": 0x01 << 31
    });
    reg!("PWMCONF", {
        "pwm_ampl": 0xff, "pwm_grad": 0xff << 8, "pwm_freq": 0x03 << 16,
        "pwm_autoscale": 0x01 << 18, "pwm_symmetric": 0x01 << 19, "freewheel": 0x03 << 20
    });
    reg!("PWM_SCALE", { "pwm_scale": 0xff });
    reg!("LOST_STEPS", { "lost_steps": 0xfffff });
    fields
}

/// The `DUMP_TMC` field formatters (`FieldFormatters`).
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
    ])
}

/// Upstream's `load_config_prefix` for `[tmc2130 <stepper>]`.
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
    let current = Arc::new(TmcCurrent::new(
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
    // CHOPCONF
    set("toff", 4)?;
    set("hstrt", 0)?;
    set("hend", 7)?;
    set("tbl", 1)?;
    set("vhighfs", 0)?;
    set("vhighchm", 0)?;
    // COOLCONF
    set("semin", 0)?;
    set("seup", 0)?;
    set("semax", 0)?;
    set("sedn", 0)?;
    set("seimin", 0)?;
    set("sgt", 0)?;
    set("sfilt", 0)?;
    // IHOLDIRUN
    set("iholddelay", 8)?;
    // PWMCONF
    set("pwm_ampl", 128)?;
    set("pwm_grad", 4)?;
    set("pwm_freq", 1)?;
    set("pwm_autoscale", 1)?;
    set("freewheel", 0)?;
    // TPOWERDOWN
    set("tpowerdown", 0)?;
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

    /// A cartesian machine with one `[tmc2130 stepper_x]`, shaped like the
    /// corpus printer configs.
    fn machine_config(dict: &std::path::Path, tmc_extra: &str, endstop_pin: &str) -> Config {
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
             rotation_distance: 8\nendstop_pin: ^PJ2\nposition_endstop: 0\nposition_max: 200\n\
             [tmc2130 stepper_x]\ncs_pin: PA4\nrun_current: 0.5\n\
             stealthchop_threshold: 999999\n{tmc_extra}",
            dict.display()
        );
        Config::from_text(&text).expect("the config parses").0
    }

    /// Load the config, and connect the fake firmware so the stepper lookup and
    /// register init run. The printer is returned for tear-down before
    /// asserting, as `upstream.rs::run_phases` does.
    async fn up_machine(
        dict: &std::path::Path,
        tmc_extra: &str,
        endstop_pin: &str,
    ) -> (Arc<Printer>, Result<(), String>) {
        let config = machine_config(dict, tmc_extra, endstop_pin);
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let printer = Arc::new(Printer::new(reactor));
        let mut start_args = crate::core::klippy::api::StartArgs::collect("tmc2130.cfg", None);
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
            .lookup_object_as::<TmcDriver>("tmc2130 stepper_x")
            .expect("the driver is registered under its section id")
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_section_loads_and_initializes_its_registers() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, "", "^PE5").await;
        let run: Result<(), String> = async {
            setup?;
            let driver = driver(&printer);
            // The chip's installed defaults, read back through the field cache.
            let fields = driver.fields();
            let field = |name: &str| fields.get_field(name, None, None);
            // CHOPCONF
            assert_eq!(field("toff"), 4);
            assert_eq!(field("hstrt"), 0);
            assert_eq!(field("hend"), 7);
            assert_eq!(field("tbl"), 1);
            assert_eq!(field("vhighfs"), 0);
            assert_eq!(field("vhighchm"), 0);
            // `microsteps: 16` → mres 4; interpolate defaults on.
            assert_eq!(field("mres"), 4);
            assert_eq!(field("intpol"), 1);
            // COOLCONF
            for name in ["semin", "seup", "semax", "sedn", "seimin", "sgt", "sfilt"] {
                assert_eq!(field(name), 0, "{name}");
            }
            // IHOLDIRUN and PWMCONF
            assert_eq!(field("iholddelay"), 8);
            assert_eq!(field("pwm_ampl"), 128);
            assert_eq!(field("pwm_grad"), 4);
            assert_eq!(field("pwm_freq"), 1);
            assert_eq!(field("pwm_autoscale"), 1);
            assert_eq!(field("freewheel"), 0);
            // TPOWERDOWN
            assert_eq!(field("tpowerdown"), 0);
            // `stealthchop_threshold` arms stealthchop (`en_pwm_mode`).
            assert_eq!(field("en_pwm_mode"), 1);
            // The wave table default (`TMCWaveTableHelper`).
            assert_eq!(field("mslut0"), 0xAAAAB554);
            assert_eq!(field("start_sin90"), 247);
            // `init_registers` sends every cached register without a bus.
            driver
                .init_registers(Some(0.))
                .map_err(|err| err.to_string())?;
            Ok(())
        }
        .await;
        printer.teardown();
        run.expect("the section loads and fills its register cache");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_virtual_pin_chip_is_registered() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        // `diag1_pin` + the virtual endstop in `[stepper_x]`, as the corpus
        // Einsy RAMBo / Lulzbot configs do.
        let (printer, setup) = up_machine(
            &dict,
            "diag1_pin: ^PK2\ndriver_SGT: 4\n",
            "tmc2130_stepper_x:virtual_endstop",
        )
        .await;
        let run: Result<(), String> = async {
            setup?;
            let pins = printer
                .lookup_object_as::<PrinterPins>(PINS_OBJECT)
                .expect("the loader registers `pins`");
            assert!(
                pins.chips().iter().any(|name| name == "tmc2130_stepper_x"),
                "the virtual pin chip is not registered: {:?}",
                pins.chips()
            );
            Ok(())
        }
        .await;
        printer.teardown();
        run.expect("the virtual-endstop config loads and registers its chip");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_command_surface_matches_upstream() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, "", "^PE5").await;
        let run: Result<Vec<String>, String> = async {
            setup?;
            let gcode = printer
                .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                .expect("the loader registers `gcode`");
            // All four mux commands exist, with upstream's help text.
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
            // An unknown field is refused with upstream's wording.
            let err = gcode
                .run_script("SET_TMC_FIELD STEPPER=stepper_x FIELD=nosuch VALUE=1")
                .await
                .expect_err("an unknown field is refused");
            assert_eq!(err.to_string(), "Unknown field name 'nosuch'");
            // Each command runs without a bus.
            gcode
                .run_script("SET_TMC_CURRENT STEPPER=stepper_x CURRENT=.7")
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
            replies
                .iter()
                .any(|line| line.contains("Run Current: 0.70A")),
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
        let (printer, setup) = up_machine(&dict, "", "^PE5").await;
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
    async fn the_phase_offset_interface_is_callable() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, "", "^PE5").await;
        let run: Result<(), String> = async {
            setup?;
            let driver = driver(&printer);
            let (offset, phases) = driver.get_phase_offset();
            assert_eq!(offset, None);
            assert_eq!(phases, 64);
            Ok(())
        }
        .await;
        printer.teardown();
        run.expect("get_phase_offset is on the object");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_chain_options_are_read() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        // A `chain_length`/`chain_position` pair on the section loads and is
        // validated by the transport.
        let (printer, setup) =
            up_machine(&dict, "chain_length: 2\nchain_position: 1\n", "^PE5").await;
        printer.teardown();
        setup.expect("a chain section loads");
    }

    #[test]
    fn a_missing_stepper_section_is_refused() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let text = format!(
            "[mcu]\ntest: dict={}\n\
             [tmc2130 stepper_y]\ncs_pin: PA4\nrun_current: 0.5\n",
            dict.display()
        );
        let config = Config::from_text(&text).unwrap().0;
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let err = printer.load_config(&config).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Could not find config section '[stepper_y]' required by tmc driver"
        );
    }

    #[test]
    fn a_chain_position_past_the_length_is_refused() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let text = format!(
            "[mcu]\ntest: dict={}\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n\
             max_z_velocity: 5\nmax_z_accel: 100\n\
             [stepper_x]\nstep_pin: PF0\ndir_pin: PF1\nenable_pin: !PD7\nmicrosteps: 16\n\
             rotation_distance: 40\nendstop_pin: ^PE5\nposition_endstop: 0\nposition_max: 200\n\
             [tmc2130 stepper_x]\ncs_pin: PA4\nrun_current: 0.5\n\
             chain_length: 2\nchain_position: 3\n",
            dict.display()
        );
        let config = Config::from_text(&text).unwrap().0;
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let err = printer.load_config(&config).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'chain_position' in section 'tmc2130 stepper_x' must have maximum of 2"
        );
    }
}
