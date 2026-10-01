//! `[fan_generic <name>]` — a fan a client drives with `SET_FAN_SPEED`.
//!
//! Upstream's `klippy/extras/fan_generic.py`: the section has **no options of
//! its own** — it hands the whole section to a
//! [`Fan`](crate::core::klippy::extras::fan) core built with
//! `default_shutdown_speed=0.` (not `heater_fan`'s 1, so the fan stops with
//! klippy) and registers one value of the `SET_FAN_SPEED` mux command, keyed by
//! the section's sub-name.
//!
//! | option | meaning |
//! |---|---|
//! | `pin` | the fan's PWM pin, required (via the `Fan` core) |
//! | the rest | the `Fan` core's options — `max_power`, `kick_start_time`,
//!   `off_below`, `cycle_time`, `hardware_pwm`, `shutdown_speed`, `enable_pin`,
//!   `tachometer_*` |
//!
//! `SET_FAN_SPEED FAN=<name> SPEED=<0..>` sets the speed (`SPEED` has
//! upstream's `minval=0.` and no upper bound — `max_power` caps the duty inside
//! the `Fan` core). `get_status` is the fan's own: `speed` and `rpm`.
//!
//! # What is not here
//!
//! * **`TEMPLATE=`.** Upstream evaluates a display template and re-applies the
//!   rendered value every 0.5 s (`fan_generic.py:36-42` →
//!   `output_pin.lookup_template_eval`). This port has no template evaluator
//!   (`output_pin` leaves its own `template` / `static_value` out for the same
//!   reason), so a `TEMPLATE` parameter is **rejected** with a command error
//!   instead of being silently ignored — the difference from upstream this
//!   module is allowed to have. No config in the corpus uses it.

use std::sync::Arc;

use serde_json::Value;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::fan::Fan;
use crate::core::klippy::gcode::{
    parse_float, sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form (`[fan_generic <name>]`) exists upstream
// (`fan_generic.py:44`).
section!("fan_generic", order = 20, prefix = load_config_prefix);

/// One `[fan_generic <name>]`.
pub struct PrinterFanGeneric {
    fan: Arc<Fan>,
    /// The name in `SET_FAN_SPEED FAN=<name>`: the section's sub.
    name: String,
}

impl PrinterFanGeneric {
    /// Read the section, build the fan, and register `SET_FAN_SPEED`.
    ///
    /// # Errors
    /// A missing option, one out of range, a pin that cannot be set up, or a
    /// `SET_FAN_SPEED` value already taken by another section.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let name = config.section().sub.clone().ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{identifier}' must be a '[fan_generic <name>]' section"
            ))
        })?;

        // `fan_generic.py:13`: `default_shutdown_speed=0.`.
        let fan = Fan::new(config, printer, 0.0)?;

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let speed = Arc::clone(&fan);
        let handler: CommandHandler = sync(move |gcmd| cmd_set_fan_speed(&speed, gcmd));
        gcode
            .register_mux_command_with_params(
                "SET_FAN_SPEED",
                "FAN",
                Some(&name),
                handler,
                Some("Sets the speed of a fan"),
                &["SPEED", "TEMPLATE"],
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        Ok(Self { fan, name })
    }
}

impl PrinterObject for PrinterFanGeneric {
    /// Upstream's `PrinterFanGeneric.get_status`: the fan's own.
    fn get_status(&self, eventtime: f64) -> Value {
        self.fan.get_status(eventtime)
    }
}

impl std::fmt::Debug for PrinterFanGeneric {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterFanGeneric")
            .field("name", &self.name)
            .finish_non_exhaustive()
    }
}

/// `gcmd.get_float('SPEED', None, 0.)`: an absent word is `None`, a present one
/// is parsed and bounded below by 0 (upstream's `minval`).
fn speed_word(gcmd: &GcodeCommand) -> Result<Option<f64>, CommandError> {
    if !gcmd.get_command_parameters().contains_key("SPEED") {
        return Ok(None);
    }
    Ok(Some(gcmd.get(
        "SPEED",
        None,
        parse_float,
        Some(0.0),
        None,
        None,
        None,
    )?))
}

/// `SET_FAN_SPEED FAN=<name> SPEED=<0..>`: set the fan's speed.
///
/// Upstream's `cmd_SET_FAN_SPEED` (`fan_generic.py:34-43`): exactly one of
/// `SPEED` and `TEMPLATE` must be given. `TEMPLATE` is answered with this
/// port's own refusal — see the module docs.
fn cmd_set_fan_speed(fan: &Arc<Fan>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let speed = speed_word(gcmd)?;
    // Upstream: `gcmd.get('TEMPLATE', None)` — present or absent, never parsed.
    let template = gcmd.get_command_parameters().get("TEMPLATE").cloned();
    if speed.is_none() == template.is_none() {
        return Err(CommandError::new(
            "SET_FAN_SPEED must specify SPEED or TEMPLATE",
        ));
    }
    let Some(speed) = speed else {
        return Err(CommandError::new(
            "SET_FAN_SPEED TEMPLATE is not supported: the template evaluator is not implemented",
        ));
    };
    fan.set_speed_from_command(speed)
}

/// The factory `section!` names (`fan_generic.py:44-45`).
///
/// # Errors
/// As [`PrinterFanGeneric::new`].
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PrinterFanGeneric::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{Config, ConfigSection, ConfigValue};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::mcu::McuError;
    use crate::core::klippy::pins::{
        DigitalOut, PinChip, PinError, PinParams, PrinterPins, PwmOut, PINS_OBJECT,
    };
    use crate::core::klippy::reactor::ManualReactor;
    use std::sync::Mutex;

    /// A PWM that records what it was told.
    #[derive(Default)]
    struct FakePwm {
        start_value: Mutex<(f64, f64)>,
        updates: Mutex<Vec<f64>>,
    }

    impl PwmOut for FakePwm {
        fn setup_max_duration(&self, _max_duration: f64) {}
        fn setup_cycle_time(&self, _cycle_time: f64, _hardware_pwm: bool) {}
        fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
            *self.start_value.lock().unwrap() = (start_value, shutdown_value);
        }
        fn set_pwm(&self, _clock: u32, value: f64) -> Result<(), McuError> {
            self.updates.lock().unwrap().push(value);
            Ok(())
        }
        fn update_pwm(&self, value: f64) -> Result<(), McuError> {
            self.updates.lock().unwrap().push(value);
            Ok(())
        }
        fn next_aligned_clock(&self, clock: u32, _allow_early: f64) -> Result<u32, McuError> {
            Ok(clock)
        }
    }

    /// A chip that hands out a [`FakePwm`] per setup.
    #[derive(Default)]
    struct FakeChip {
        pwms: Mutex<Vec<Arc<FakePwm>>>,
    }

    impl PinChip for FakeChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            Err(PinError::Unsupported("digital_out".to_string()))
        }

        fn setup_pwm(&self, _params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
            let pwm = Arc::new(FakePwm::default());
            self.pwms.lock().unwrap().push(Arc::clone(&pwm));
            Ok(pwm)
        }
    }

    /// A printer with `gcode` and `pins` over a fake chip.
    fn printer() -> (Arc<Printer>, Arc<FakeChip>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let pins = Arc::new(PrinterPins::new());
        let chip = Arc::new(FakeChip::default());
        pins.register_chip("mcu", chip.clone()).unwrap();
        printer.add_object(PINS_OBJECT, pins).unwrap();
        // The dispatcher refuses commands until the printer is ready.
        printer.send_event(&KlippyEvent::KlippyReady);
        (printer, chip)
    }

    /// A `[fan_generic <name>]` section with `pin: <pin>` plus `options`.
    fn section(name: &str, pin: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("fan_generic", Some(name));
        section
            .parameters
            .insert("pin".to_string(), ConfigValue::Single(pin.to_string()));
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    fn load(
        printer: &Arc<Printer>,
        section: &ConfigSection,
    ) -> Result<Arc<PrinterFanGeneric>, ConfigError> {
        let fan = load_config_prefix(&wrap(section), printer)?;
        let identifier = section.identifier();
        printer
            .add_object(identifier.as_str(), fan)
            .expect("fresh printer");
        Ok(printer
            .lookup_object_as::<PrinterFanGeneric>(&identifier)
            .expect("the factory builds a PrinterFanGeneric"))
    }

    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap()
    }

    fn pwm(chip: &FakeChip, index: usize) -> Arc<FakePwm> {
        chip.pwms.lock().unwrap()[index].clone()
    }

    fn updates(pwm: &FakePwm) -> Vec<f64> {
        pwm.updates.lock().unwrap().clone()
    }

    fn speed(fan: &PrinterFanGeneric) -> f64 {
        fan.get_status(0.0)["speed"]
            .as_f64()
            .expect("speed is a number")
    }

    #[test]
    fn test_a_fan_generic_is_a_fan_core_that_stops_with_klippy() {
        let (printer, chip) = printer();
        let fan = load(&printer, &section("side_fan", "PH0", &[])).unwrap();

        // A fan starts at 0; the shutdown duty is 0 too, unlike `heater_fan`'s
        // 1 (`fan_generic.py:13`).
        assert_eq!(*pwm(&chip, 0).start_value.lock().unwrap(), (0.0, 0.0));
        assert_eq!(speed(&fan), 0.0);
        assert!(fan.get_status(0.0)["rpm"].is_null());
    }

    #[test]
    fn test_set_fan_speed_drives_the_pwm() {
        let (printer, chip) = printer();
        let fan = load(
            &printer,
            &section("side_fan", "PH0", &[("kick_start_time", "0")]),
        )
        .unwrap();

        gcode(&printer)
            .run_script_sync("SET_FAN_SPEED FAN=side_fan SPEED=0.5")
            .unwrap();
        assert_eq!(updates(&pwm(&chip, 0)), [0.5]);
        assert_eq!(speed(&fan), 0.5);

        // `SPEED` has upstream's `minval=0.`, and no upper bound of its own.
        let err = gcode(&printer)
            .run_script_sync("SET_FAN_SPEED FAN=side_fan SPEED=-0.1")
            .unwrap_err();
        assert!(err.to_string().contains("minimum of 0"), "{err}");

        gcode(&printer)
            .run_script_sync("SET_FAN_SPEED FAN=side_fan SPEED=2")
            .unwrap();
        assert_eq!(
            updates(&pwm(&chip, 0)),
            [0.5, 1.0],
            "no upper bound on SPEED: the `Fan` core's `max_power` caps the duty"
        );
        assert_eq!(speed(&fan), 1.0);
    }

    #[test]
    fn test_speed_and_template_are_mutually_exclusive() {
        let (printer, _chip) = printer();
        load(&printer, &section("side_fan", "PH0", &[])).unwrap();

        for line in [
            "SET_FAN_SPEED FAN=side_fan",
            "SET_FAN_SPEED FAN=side_fan SPEED=0.5 TEMPLATE=hot",
        ] {
            let err = gcode(&printer).run_script_sync(line).unwrap_err();
            assert_eq!(
                err.to_string(),
                "SET_FAN_SPEED must specify SPEED or TEMPLATE",
                "{line}"
            );
        }
    }

    #[test]
    fn test_template_is_rejected_as_not_implemented() {
        let (printer, chip) = printer();
        load(&printer, &section("side_fan", "PH0", &[])).unwrap();

        let err = gcode(&printer)
            .run_script_sync("SET_FAN_SPEED FAN=side_fan TEMPLATE=hot")
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "SET_FAN_SPEED TEMPLATE is not supported: the template evaluator is not implemented"
        );
        assert!(updates(&pwm(&chip, 0)).is_empty(), "nothing was driven");
    }

    #[test]
    fn test_a_missing_pin_names_the_prefixed_section() {
        let (printer, _chip) = printer();
        let mut section = ConfigSection::new("fan_generic", Some("side_fan"));

        let err = load(&printer, &section).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'pin' in section 'fan_generic side_fan' must be specified"
        );

        // A bare `[fan_generic]` has no mux value to register under.
        section.sub = None;
        let err = load(&printer, &section).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Section 'fan_generic' must be a '[fan_generic <name>]' section"
        );
    }

    #[test]
    fn test_the_section_loads_from_a_full_config() {
        // What the corpus configs do — an `[mcu]` with `[fan_generic <name>]` —
        // must survive the real loader, which also runs the undefined-option
        // check over this section (the unit tests above use a bare wrapper).
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let text = "[mcu]\nserial: /dev/not-opened-yet\n\
                    [fan_generic side_fan]\npin: PH0\n";
        let (config, _) = Config::from_text(text).expect("the config parses");
        printer.load_config(&config).expect("the config loads");
        printer.send_event(&KlippyEvent::KlippyReady);

        let fan = printer
            .lookup_object_as::<PrinterFanGeneric>("fan_generic side_fan")
            .expect("the prefixed section is registered");

        // The fan is the section's own: off, with no tachometer to report an
        // RPM from. Its `SET_FAN_SPEED` value cannot be driven here — the MCU
        // is not connected (the fake-chip tests above cover that path).
        assert_eq!(fan.get_status(0.0)["speed"], 0.0);
        assert!(fan.get_status(0.0)["rpm"].is_null());
    }
}
