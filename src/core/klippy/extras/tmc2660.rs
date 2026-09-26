//! `[tmc2660 <stepper>]` — the TMC2660 SPI driver (upstream's
//! `klippy/extras/tmc2660.py`).
//!
//! The chip speaks a 3-byte SPI frame through [`Tmc2660Spi`], and it is the one
//! TMC driver whose current model is not `irun`/`ihold`: it programs `cs` in
//! `SGCSCONF` and `vsense` in `DRVCONF`, with **no hold current** — so
//! `SET_TMC_CURRENT` answers with the run current alone. [`Tmc2660Current`]
//! carries that model; `run_current` and `sense_resistor` are both required.
//!
//! It also reads its fault status indirectly: there is no `DRV_STATUS`, the chip
//! answers whichever of `READRSP@RDSEL0`..`2` the `DRVCONF.rdsel` field selects
//! (`tmc2660.py:15`), which is why the read-register list is those three names
//! and [`crate::core::klippy::extras::tmc::TmcErrorCheck`] reads `RDSEL2`.
//!
//! # What is not here
//!
//! **No virtual pin.** Upstream builds the 2660 without a `TMCVirtualPinHelper`
//! (`tmc2660.py:242-283`), so no `<driver>_<stepper>` chip is registered and
//! `<driver>_<stepper>:virtual_endstop` cannot be named. **No wave table, no
//! stealthchop and no `tcoolthrs`** — the chip has none of `mslut*`/`en_pwm_mode`
//! and upstream's setup omits them. The `tmc/stallguard_dump` bulk endpoint is
//! likewise absent, as it is for every driver in this host.

use std::collections::HashMap;
use std::sync::Arc;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::tmc::{FieldHelper, Tmc2660Current, TmcDriver, TmcTransport};
use crate::core::klippy::extras::tmc2130::field_formatters as tmc2130_field_formatters;
use crate::core::klippy::extras::tmc_spi::Tmc2660Spi;
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

// Loaded after the pins and the MCU's SPI resource; alongside the other
// `order = 40` device sections.
section!("tmc2660", order = 40, prefix = load_config_prefix);

/// Upstream's error when the spread-cycle default has more than 15 blank-time
/// clocks (`tmc2660.py:276`).
const HEND_HSTRT_ERROR: &str = "driver_HEND + driver_HSTRT must be <= 15";

/// Register name → SPI address (`Registers`).
pub fn registers() -> HashMap<String, u8> {
    [
        ("DRVCONF", 0xE),
        ("SGCSCONF", 0xC),
        ("SMARTEN", 0xA),
        ("CHOPCONF", 0x8),
        ("DRVCTRL", 0x0),
    ]
    .into_iter()
    .map(|(name, addr)| (name.to_string(), addr))
    .collect()
}

/// The registers `DUMP_TMC` reads, in `rdsel` order (`ReadRegisters`).
pub fn read_registers() -> Vec<String> {
    ["READRSP@RDSEL0", "READRSP@RDSEL1", "READRSP@RDSEL2"]
        .into_iter()
        .map(str::to_string)
        .collect()
}

/// The fields whose register value is two's-complement signed (`SignedFields`).
pub const SIGNED_FIELDS: [&str; 1] = ["sgt"];

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
    reg!("DRVCTRL", {
        "mres": 0x0f, "dedge": 0x01 << 8, "intpol": 0x01 << 9
    });
    reg!("CHOPCONF", {
        "toff": 0x0f, "hstrt": 0x07 << 4, "hend": 0x0f << 7, "hdec": 0x03 << 11,
        "rndtf": 0x01 << 13, "chm": 0x01 << 14, "tbl": 0x03 << 15
    });
    reg!("SMARTEN", {
        "semin": 0x0f, "seup": 0x03 << 5, "semax": 0x0f << 8, "sedn": 0x03 << 13,
        "seimin": 0x01 << 15
    });
    reg!("SGCSCONF", {
        "cs": 0x1f, "sgt": 0x7f << 8, "sfilt": 0x01 << 16
    });
    reg!("DRVCONF", {
        "rdsel": 0x03 << 4, "vsense": 0x01 << 6, "sdoff": 0x01 << 7, "ts2g": 0x03 << 8,
        "diss2g": 0x01 << 10, "slpl": 0x03 << 12, "slph": 0x03 << 14, "tst": 0x01 << 16
    });
    reg!("READRSP@RDSEL0", {
        "stallguard": 0x01 << 4, "ot": 0x01 << 5, "otpw": 0x01 << 6, "s2ga": 0x01 << 7,
        "s2gb": 0x01 << 8, "ola": 0x01 << 9, "olb": 0x01 << 10, "stst": 0x01 << 11,
        "mstep": 0x3ff << 14
    });
    reg!("READRSP@RDSEL1", {
        "stallguard": 0x01 << 4, "ot": 0x01 << 5, "otpw": 0x01 << 6, "s2ga": 0x01 << 7,
        "s2gb": 0x01 << 8, "ola": 0x01 << 9, "olb": 0x01 << 10, "stst": 0x01 << 11,
        "sg_result": 0x3ff << 14
    });
    reg!("READRSP@RDSEL2", {
        "stallguard": 0x01 << 4, "ot": 0x01 << 5, "otpw": 0x01 << 6, "s2ga": 0x01 << 7,
        "s2gb": 0x01 << 8, "ola": 0x01 << 9, "olb": 0x01 << 10, "stst": 0x01 << 11,
        "se": 0x1f << 14, "sg_result@rdsel2": 0x1f << 19
    });
    fields
}

/// The `DUMP_TMC` field formatters (`FieldFormatters`): the TMC2130's, plus the
/// five the 2660 adds.
pub fn field_formatters() -> HashMap<String, fn(i64) -> String> {
    fn chm(v: i64) -> String {
        if v != 0 {
            "1(constant toff)".into()
        } else {
            "0(spreadCycle)".into()
        }
    }
    fn vsense(v: i64) -> String {
        if v != 0 {
            "1(165mV)".into()
        } else {
            "0(305mV)".into()
        }
    }
    fn sdoff(v: i64) -> String {
        if v != 0 {
            "1(Step/Dir disabled!)".into()
        } else {
            String::new()
        }
    }
    fn diss2g(v: i64) -> String {
        if v != 0 {
            "1(Short to GND disabled!)".into()
        } else {
            String::new()
        }
    }
    fn se(v: i64) -> String {
        if v != 0 {
            v.to_string()
        } else {
            "0(Reset?)".to_string()
        }
    }
    let mut formatters = tmc2130_field_formatters();
    formatters.extend([
        ("chm".to_string(), chm as fn(i64) -> String),
        ("vsense".to_string(), vsense),
        ("sdoff".to_string(), sdoff),
        ("diss2g".to_string(), diss2g),
        ("se".to_string(), se),
    ]);
    formatters
}

/// Upstream's `load_config_prefix` for `[tmc2660 <stepper>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let field_helper = Arc::new(FieldHelper::new(
        fields(),
        &SIGNED_FIELDS,
        field_formatters(),
    ));
    // Access DRVCTRL in step/dir mode (`tmc2660.py:259`).
    field_helper.set_field("sdoff", 0, None, None);
    let transport: Arc<dyn TmcTransport> = Arc::new(Tmc2660Spi::new(
        config,
        printer,
        registers(),
        read_registers(),
        Arc::clone(&field_helper),
    )?);
    let current = Tmc2660Current::new(
        config,
        printer,
        Arc::clone(&field_helper),
        Arc::clone(&transport),
    )?;
    let driver = TmcDriver::new(
        config,
        printer,
        Arc::clone(&field_helper),
        Arc::clone(&transport),
        current,
        read_registers(),
        None,
    )?;
    // No virtual pin: the 2660 has no `TMCVirtualPinHelper` upstream.
    let set = |field: &str, default: i64| field_helper.set_config_field(config, field, default);
    // CHOPCONF
    set("tbl", 2)?;
    set("rndtf", 0)?;
    set("hdec", 0)?;
    set("chm", 0)?;
    set("hend", 3)?;
    set("hstrt", 3)?;
    set("toff", 4)?;
    if field_helper.get_field("chm", None, None) == 0
        && field_helper.get_field("hstrt", None, None) + field_helper.get_field("hend", None, None)
            > 15
    {
        return Err(ConfigError::new(HEND_HSTRT_ERROR));
    }
    // SMARTEN
    set("seimin", 0)?;
    set("sedn", 0)?;
    set("semax", 0)?;
    set("seup", 0)?;
    set("semin", 0)?;
    // SGCSCONF
    set("sfilt", 0)?;
    set("sgt", 0)?;
    // DRVCONF
    set("slph", 0)?;
    set("slpl", 0)?;
    set("diss2g", 0)?;
    set("ts2g", 3)?;
    Ok(driver)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex as StdMutex;

    use crate::core::klippy::api::StartArgs;
    use crate::core::klippy::config::section::ConfigSection;
    use crate::core::klippy::config::value::ConfigValue;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
    use crate::core::klippy::mcu::McuObject;
    use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
    use crate::core::klippy::printer::PrinterState;
    use crate::core::klippy::reactor::{ManualReactor, TokioReactor};

    fn fake_dictionary() -> Option<std::path::PathBuf> {
        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        dict.is_file().then_some(dict)
    }

    /// A cartesian machine with one `[tmc2660 stepper_x]`, shaped like the
    /// corpus printer configs (`config/generic-duet2.cfg`).
    fn machine_config(dict: &std::path::Path, tmc_extra: &str) -> Config {
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
             [tmc2660 stepper_x]\ncs_pin: PA4\nrun_current: 0.5\nsense_resistor: 0.220\n\
             {tmc_extra}",
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
    ) -> (Arc<Printer>, Result<(), String>) {
        let config = machine_config(dict, tmc_extra);
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let printer = Arc::new(Printer::new(reactor));
        let mut start_args = crate::core::klippy::api::StartArgs::collect("tmc2660.cfg", None);
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
            .lookup_object_as::<TmcDriver>("tmc2660 stepper_x")
            .expect("the driver is registered under its section id")
    }

    /// One field, read back through the driver's cache.
    fn field_value(printer: &Arc<Printer>, name: &str) -> i64 {
        driver(printer).fields().get_field(name, None, None)
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_section_loads_and_initializes_its_registers() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, "").await;
        let run: Result<(), String> = async {
            setup?;
            let driver = driver(&printer);
            let fields = driver.fields();
            let field = |name: &str| fields.get_field(name, None, None);
            // The chip answers step/dir mode only with `sdoff` cleared.
            assert_eq!(field("sdoff"), 0);
            // CHOPCONF
            assert_eq!(field("tbl"), 2);
            assert_eq!(field("rndtf"), 0);
            assert_eq!(field("hdec"), 0);
            assert_eq!(field("chm"), 0);
            assert_eq!(field("hend"), 3);
            assert_eq!(field("hstrt"), 3);
            assert_eq!(field("toff"), 4);
            // SMARTEN
            for name in ["seimin", "sedn", "semax", "seup", "semin"] {
                assert_eq!(field(name), 0, "{name}");
            }
            // SGCSCONF
            assert_eq!(field("sfilt"), 0);
            assert_eq!(field("sgt"), 0);
            // DRVCONF
            assert_eq!(field("slph"), 0);
            assert_eq!(field("slpl"), 0);
            assert_eq!(field("diss2g"), 0);
            assert_eq!(field("ts2g"), 3);
            // `microsteps: 16` → mres 4; interpolate defaults on.
            assert_eq!(field("mres"), 4);
            assert_eq!(field("intpol"), 1);
            // The register cache the connect phase sends, one value per
            // register: DRVCTRL(mres 4, intpol 1), CHOPCONF(toff 4, hstrt 3,
            // hend 3, tbl 2), SMARTEN(0), SGCSCONF(cs 29), DRVCONF(vsense 1,
            // ts2g 3, sdoff 0).
            let registers: HashMap<String, u32> = driver.registers().into_iter().collect();
            assert_eq!(registers["DRVCTRL"], 0x204);
            assert_eq!(registers["CHOPCONF"], 0x1_01B4);
            assert_eq!(registers["SMARTEN"], 0x0);
            assert_eq!(registers["SGCSCONF"], 0x1D);
            assert_eq!(registers["DRVCONF"], 0x340);
            // No virtual pin chip: upstream's 2660 registers none.
            let pins = printer
                .lookup_object_as::<PrinterPins>(PINS_OBJECT)
                .expect("the loader registers `pins`");
            assert!(
                !pins.chips().iter().any(|name| name == "tmc2660_stepper_x"),
                "the 2660 must not register a virtual pin chip: {:?}",
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
        run.expect("the section loads and fills its register cache");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_current_options_seed_cs_and_vsense() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        // `run_current=0.5`, `sense_resistor=0.220` → vsense 1, cs 29.
        let (printer, setup) = up_machine(&dict, "").await;
        let run: Result<(), String> = async {
            setup?;
            assert_eq!(field_value(&printer, "vsense"), 1);
            assert_eq!(field_value(&printer, "cs"), 29);
            // The chip has no hold current in its status.
            let status = driver(&printer).get_status(0.);
            assert_eq!(status["run_current"], 0.5);
            assert_eq!(status["hold_current"], serde_json::Value::Null);
            Ok(())
        }
        .await;
        printer.teardown();
        run.expect("the current helper seeds cs/vsense and reports no hold current");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_idle_current_callbacks_lower_and_restore_cs() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, "idle_current_percent: 50\n").await;
        let run: Result<(), String> = async {
            setup?;
            assert_eq!(field_value(&printer, "cs"), 29);
            // Going ready drops to `idle_current_percent` of the run current;
            // printing restores it.
            printer.send_event(&KlippyEvent::IdleTimeoutReady { print_time: 1. });
            assert_eq!(field_value(&printer, "cs"), 14);
            printer.send_event(&KlippyEvent::IdleTimeoutPrinting { print_time: 2. });
            assert_eq!(field_value(&printer, "cs"), 29);
            Ok(())
        }
        .await;
        printer.teardown();
        run.expect("the idle timeout events rewrite cs");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn set_tmc_current_replies_with_one_line() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, "").await;
        let run: Result<Vec<String>, String> = async {
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
            gcode
                .run_script("SET_TMC_CURRENT STEPPER=stepper_x CURRENT=.7")
                .await
                .map_err(|err| err.to_string())?;
            let captured = replies.lock().unwrap().clone();
            Ok(captured)
        }
        .await;
        printer.teardown();
        let replies = run.expect("the machine loads and the command runs");
        // One line, no hold current.
        let current_lines: Vec<&String> = replies
            .iter()
            .filter(|line| line.contains("Current"))
            .collect();
        assert_eq!(current_lines.len(), 1, "one line only: {replies:?}");
        // `respond_info` prefixes the message with `// `, as upstream does.
        assert_eq!(current_lines[0], "// Run Current: 0.70A");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn fileoutput_never_touches_the_bus() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, "").await;
        let run: Result<(), String> = async {
            setup?;
            assert!(printer.is_fileoutput());
            let driver = driver(&printer);
            // The transport reports 0 for every read and accepts (drops) every
            // write without sending a frame.
            assert_eq!(
                driver.transport().get_register("READRSP@RDSEL2").unwrap(),
                0
            );
            driver
                .transport()
                .set_register("CHOPCONF", 0x1234, None)
                .expect("a write is dropped, not sent");
            assert_eq!(driver.transport().get_tmc_frequency(), None);
            assert!(driver.transport().name_to_reg().contains_key("DRVCONF"));
            Ok(())
        }
        .await;
        printer.teardown();
        run.expect("reads answer 0 and writes are dropped under file output");
    }

    /// Load one machine config, returning its error when it is refused.
    fn load_result(dict: &std::path::Path, tmc_extra: &str) -> Result<(), String> {
        let config = machine_config(dict, tmc_extra);
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .load_config(&config)
            .map(|_| ())
            .map_err(|err| err.to_string())
    }

    #[test]
    fn the_hend_hstrt_sum_is_refused() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        // Defaults are 3 + 3; `driver_HEND: 15` pushes the sum over 15.
        assert_eq!(
            load_result(&dict, "driver_HEND: 15\n"),
            Err(HEND_HSTRT_ERROR.to_string())
        );
        // The check is bypassed once `chm` selects constant off-time.
        assert_eq!(
            load_result(&dict, "driver_HEND: 15\ndriver_CHM: 1\n"),
            Ok(())
        );
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_phase_offset_interface_is_callable() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, "").await;
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

    // -- the RDSEL read path -----------------------------------------------

    /// A `[tmc2660 stepper_x]`-shaped section's options.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("tmc2660", Some("stepper_x"));
        for (key, value) in options {
            section.parameters.insert(
                key.to_lowercase(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A printer with `pins` and one MCU, shaped like a file-output test run.
    fn bare_printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        let object = Arc::new(
            McuObject::new(ConfigSection::new("mcu", None), &printer)
                .expect("the MCU registers its chip"),
        );
        printer
            .add_object("mcu", object as Arc<dyn PrinterObject>)
            .unwrap();
        let mut start_args = StartArgs::collect("tmc2660.cfg", None);
        start_args.debug_output = Some("_test_output".to_string());
        printer.set_start_args(Arc::new(start_args));
        printer
    }

    #[test]
    fn the_read_path_rewrites_rdsel_through_this_tables_address() {
        let printer = bare_printer();
        let fields = Arc::new(FieldHelper::new(
            fields(),
            &SIGNED_FIELDS,
            field_formatters(),
        ));
        let spi = Tmc2660Spi::new(
            &ConfigWrapper::untracked(&section(&[("cs_pin", "PA4")])),
            &printer,
            registers(),
            read_registers(),
            Arc::clone(&fields),
        )
        .expect("the 2660 section loads");
        // `DRVCONF` is 0x0E and `rdsel` sits at bits 4-5; RDSEL0 is index 0,
        // which the field already is, so no setup frame is queued.
        assert_eq!(
            spi.read_command("READRSP@RDSEL0").unwrap(),
            (false, [0x0E, 0x00, 0x00])
        );
        assert_eq!(
            spi.read_command("READRSP@RDSEL1").unwrap(),
            (true, [0x0E, 0x00, 0x10])
        );
        assert_eq!(fields.get_field("rdsel", None, None), 1);
        assert_eq!(
            spi.read_command("READRSP@RDSEL2").unwrap(),
            (true, [0x0E, 0x00, 0x20])
        );
        // A register the chip does not answer through `rdsel` is refused.
        assert!(spi.read_command("CHOPCONF").is_err());
        // A write shifts `[(val >> 16) | reg, val >> 8, val]`; the configured
        // CHOPCONF (`toff=4, hstrt=3, hend=3, tbl=2`) is 0x101b4.
        assert_eq!(
            spi.write_command("CHOPCONF", 0x0001_01B4).unwrap(),
            [0x09, 0x01, 0xB4]
        );
        assert_eq!(
            spi.write_command("SGCSCONF", 0x0000_001D).unwrap(),
            [0x0C, 0x00, 0x1D]
        );
        // A read response is three bytes big-endian.
        assert_eq!(
            Tmc2660Spi::decode_data(&[0xAB, 0xCD, 0xEF]),
            Some(0x00AB_CDEF)
        );
        assert_eq!(Tmc2660Spi::decode_data(&[0x00, 0x01]), None);
    }
}
