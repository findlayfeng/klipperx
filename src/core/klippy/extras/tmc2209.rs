//! `[tmc2209 <stepper>]` — the TMC2209 UART driver (upstream's
//! `klippy/extras/tmc2209.py`).
//!
//! It reuses the TMC2208 tables ([`crate::core::klippy::extras::tmc2208`]) the
//! way upstream does, adding the StallGuard/CoolStep registers and — the reason
//! this driver and not `tmc2208` — the virtual-endstop chip
//! ([`TmcVirtualPin`]): `[stepper_x]`'s `endstop_pin:
//! tmc2209_stepper_x:virtual_endstop`.
//!
//! Notable differences from `tmc2208`: `uart_address` may be 0..=3,
//! `senddelay=2` avoids tx errors on a shared uart, and `coolstep_threshold`
//! (→ `TCOOLTHRS`) is read.

use std::collections::HashMap;
use std::sync::Arc;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::tmc::{
    stealthchop_helper, vcoolthrs_helper, FieldHelper, TmcCurrent, TmcDriver, TmcTransport,
    TmcVirtualPin,
};
use crate::core::klippy::extras::tmc2208;
use crate::core::klippy::extras::tmc_uart::TmcUart;
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("tmc2209", order = 40, prefix = load_config_prefix);

/// The chip's internal TSTEP frequency (`TMC_FREQUENCY`).
pub const TMC_FREQUENCY: f64 = tmc2208::TMC_FREQUENCY;

/// The largest `uart_address` the chip answers (`MCU_TMC_uart(..., 3, ...)`).
pub const MAX_ADDR: i64 = 3;

/// Register name → UART address (`Registers`).
pub fn registers() -> HashMap<String, u8> {
    let mut registers = tmc2208::registers();
    for (name, addr) in [
        ("TCOOLTHRS", 0x14),
        ("COOLCONF", 0x42),
        ("SGTHRS", 0x40),
        ("SG_RESULT", 0x41),
    ] {
        registers.insert(name.to_string(), addr);
    }
    registers
}

/// The registers `DUMP_TMC` reads (`ReadRegisters`).
pub fn read_registers() -> Vec<String> {
    let mut read = tmc2208::read_registers();
    read.push("SG_RESULT".to_string());
    read
}

/// The TMC2209's register/field layout (`Fields`, over `tmc2208`'s).
pub fn fields() -> HashMap<String, HashMap<String, u32>> {
    let mut fields = tmc2208::fields();
    fields.insert(
        "COOLCONF".to_string(),
        HashMap::from([
            ("semin".to_string(), 0x0f),
            ("seup".to_string(), 0x03 << 5),
            ("semax".to_string(), 0x0f << 8),
            ("sedn".to_string(), 0x03 << 13),
            ("seimin".to_string(), 0x01 << 15),
        ]),
    );
    fields.insert(
        "IOIN".to_string(),
        HashMap::from([
            ("enn".to_string(), 0x01),
            ("ms1".to_string(), 0x01 << 2),
            ("ms2".to_string(), 0x01 << 3),
            ("diag".to_string(), 0x01 << 4),
            ("pdn_uart".to_string(), 0x01 << 6),
            ("step".to_string(), 0x01 << 7),
            ("spread_en".to_string(), 0x01 << 8),
            ("dir".to_string(), 0x01 << 9),
            ("version".to_string(), 0xff << 24),
        ]),
    );
    fields.insert(
        "SGTHRS".to_string(),
        HashMap::from([("sgthrs".to_string(), 0xff)]),
    );
    fields.insert(
        "SG_RESULT".to_string(),
        HashMap::from([("sg_result".to_string(), 0x3ff)]),
    );
    fields.insert(
        "TCOOLTHRS".to_string(),
        HashMap::from([("tcoolthrs".to_string(), 0xfffff)]),
    );
    fields
}

/// Upstream's `load_config_prefix` for `[tmc2209 <stepper>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let field_helper = Arc::new(FieldHelper::new(
        fields(),
        &tmc2208::SIGNED_FIELDS,
        tmc2208::field_formatters(),
    ));
    let transport: Arc<dyn TmcTransport> = Arc::new(TmcUart::new(
        config,
        printer,
        registers(),
        MAX_ADDR,
        TMC_FREQUENCY,
    )?);
    // Setup fields for UART.
    field_helper.set_field("pdn_disable", 1, None, None);
    // Avoid tx errors on shared uart.
    field_helper.set_field("senddelay", 2, None, None);
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
    field_helper.set_field("mstep_reg_select", 1, None, None);
    stealthchop_helper(config, &field_helper, transport.as_ref())?;
    vcoolthrs_helper(config, &field_helper, transport.as_ref())?;
    let set = |field: &str, default: i64| field_helper.set_config_field(config, field, default);
    // GCONF
    set("multistep_filt", 1)?;
    // CHOPCONF
    set("toff", 3)?;
    set("hstrt", 5)?;
    set("hend", 0)?;
    set("tbl", 2)?;
    // COOLCONF
    set("semin", 0)?;
    set("seup", 0)?;
    set("semax", 0)?;
    set("sedn", 0)?;
    set("seimin", 0)?;
    // IHOLDIRUN
    set("iholddelay", 8)?;
    // PWMCONF
    set("pwm_ofs", 36)?;
    set("pwm_grad", 14)?;
    set("pwm_freq", 1)?;
    set("pwm_autoscale", 1)?;
    set("pwm_autograd", 1)?;
    set("freewheel", 0)?;
    set("pwm_reg", 8)?;
    set("pwm_lim", 12)?;
    // TPOWERDOWN
    set("tpowerdown", 20)?;
    // SGTHRS
    set("sgthrs", 0)?;
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
    use crate::core::klippy::printer::PrinterState;
    use crate::core::klippy::reactor::TokioReactor;

    fn fake_dictionary() -> Option<std::path::PathBuf> {
        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        dict.is_file().then_some(dict)
    }

    /// A cartesian machine with one `[tmc2209 stepper_x]`, shaped like the
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
             [tmc2209 stepper_x]\nuart_pin: PC4\ntx_pin: PC3\nuart_address: 0\n\
             run_current: 0.5\nstealthchop_threshold: 999999\n{tmc_extra}",
            dict.display()
        );
        Config::from_text(&text).expect("the config parses").0
    }

    /// Load the config, and (when `bring_up`) connect the fake firmware so the
    /// stepper lookup and register init run. The printer is returned for tear-
    /// down before asserting, as `upstream.rs::run_phases` does.
    async fn up_machine(
        dict: &std::path::Path,
        tmc_extra: &str,
        endstop_pin: &str,
    ) -> (Arc<Printer>, Result<(), String>) {
        let config = machine_config(dict, tmc_extra, endstop_pin);
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let printer = Arc::new(Printer::new(reactor));
        let mut start_args = crate::core::klippy::api::StartArgs::collect("tmc2209.cfg", None);
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
            .lookup_object_as::<TmcDriver>("tmc2209 stepper_x")
            .expect("the driver is registered under its section id")
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
            // 3. An unknown register is refused too.
            let err = gcode
                .run_script("DUMP_TMC STEPPER=stepper_x REGISTER=NOPE")
                .await
                .expect_err("an unknown register is refused");
            assert_eq!(err.to_string(), "Unknown register name 'NOPE'");
            // 4. SET_TMC_CURRENT reports upstream's run-current line.
            gcode
                .run_script("SET_TMC_CURRENT STEPPER=stepper_x CURRENT=.7")
                .await
                .map_err(|err| err.to_string())?;
            // INIT_TMC and a full DUMP_TMC run without a bus.
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
            // write without sending a frame — upstream's `debugoutput` branch.
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
    async fn the_error_check_never_shuts_the_printer_down() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, "", "^PE5").await;
        let run: Result<(), String> = async {
            setup?;
            let driver = driver(&printer);
            // `run_current: 0.5` seeds a non-zero `ihold`; under file output the
            // check reads 0 from both registers and must not fault.
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

    #[tokio::test(flavor = "multi_thread")]
    async fn the_phase_offset_interface_is_callable() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, "", "^PE5").await;
        let run: Result<(), String> = async {
            setup?;
            let driver = driver(&printer);
            // `endstop_phase`'s contract: `(mcu_phase_offset, phases)` with
            // `phases = (256 >> mres) * 4`; `microsteps: 16` → mres 4 → 64.
            let (offset, phases) = driver.get_phase_offset();
            assert_eq!(offset, None);
            assert_eq!(phases, 64);
            Ok(())
        }
        .await;
        printer.teardown();
        run.expect("get_phase_offset is on the object");
    }

    #[test]
    fn a_missing_uart_pin_is_refused() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let text = format!(
            "[mcu]\ntest: dict={}\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n\
             max_z_velocity: 5\nmax_z_accel: 100\n\
             [stepper_x]\nstep_pin: PF0\ndir_pin: PF1\nenable_pin: !PD7\nmicrosteps: 16\n\
             rotation_distance: 40\nendstop_pin: ^PE5\nposition_endstop: 0\nposition_max: 200\n\
             [tmc2209 stepper_x]\nrun_current: 0.5\n",
            dict.display()
        );
        let config = Config::from_text(&text).unwrap().0;
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let err = printer.load_config(&config).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'uart_pin' in section 'tmc2209 stepper_x' must be specified"
        );
    }

    #[test]
    fn a_missing_stepper_section_is_refused() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let text = format!(
            "[mcu]\ntest: dict={}\n\
             [tmc2208 stepper_y]\nuart_pin: PC4\nrun_current: 0.5\n",
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

    #[tokio::test(flavor = "multi_thread")]
    async fn the_virtual_endstop_binds_and_loads() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        // `diag_pin` + the virtual endstop in `[stepper_x]`, as the corpus
        // Prusa Mini config does.
        let (printer, setup) = up_machine(
            &dict,
            "diag_pin: ^PE2\ndriver_SGTHRS: 130\n",
            "tmc2209_stepper_x:virtual_endstop",
        )
        .await;
        printer.teardown();
        setup.expect("the virtual-endstop config loads and connects");
    }

    #[test]
    fn a_virtual_endstop_without_a_diag_pin_is_refused() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let text = format!(
            "[mcu]\ntest: dict={}\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n\
             max_z_velocity: 5\nmax_z_accel: 100\n\
             [stepper_x]\nstep_pin: PF0\ndir_pin: PF1\nenable_pin: !PD7\nmicrosteps: 16\n\
             rotation_distance: 40\nendstop_pin: tmc2209_stepper_x:virtual_endstop\n\
             position_endstop: 0\nposition_max: 200\n\
             [tmc2209 stepper_x]\nuart_pin: PC4\nrun_current: 0.5\n",
            dict.display()
        );
        let config = Config::from_text(&text).unwrap().0;
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let err = printer.load_config(&config).unwrap_err();
        assert!(
            err.to_string()
                .ends_with("tmc virtual endstop requires diag pin config"),
            "{err}"
        );
    }
}
