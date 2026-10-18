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
//! # `TEMPLATE=`
//!
//! `SET_FAN_SPEED FAN=<name> TEMPLATE=<display_template> [PARAM_*=<literal>...]`
//! subscribes the fan to the shared
//! [`template_evaluator`](crate::core::klippy::extras::template_evaluator)
//! (`fan_generic.py:15-32`): the template is rendered every 0.5 s and each
//! rendered text is parsed as a number and handed to
//! [`Fan::set_speed`](crate::core::klippy::extras::fan::Fan::set_speed), which
//! sends it at the estimated print time. A text that is not a number, or a
//! template body that fails, drives `0.` with a logged error. `SPEED` and
//! `TEMPLATE` are mutually exclusive in both directions, and an empty
//! `TEMPLATE=` cancels the subscription.

use std::sync::Arc;

use serde_json::Value;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::fan::Fan;
use crate::core::klippy::extras::template_evaluator::{
    lookup_template_eval, PrinterTemplateEvaluator,
};
use crate::core::klippy::gcode::{
    parse_float, sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};
use tracing::warn;

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
        let template_eval = lookup_template_eval(printer)?;
        let speed = Arc::clone(&fan);
        let handler: CommandHandler = {
            let template_eval = Arc::clone(&template_eval);
            let fan_name = name.clone();
            sync(move |gcmd| cmd_set_fan_speed(&speed, &fan_name, &template_eval, gcmd))
        };
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

/// `SET_FAN_SPEED FAN=<name> SPEED=<0..>`: set the fan's speed, or
/// `... TEMPLATE=<display_template>`: subscribe it to a template.
///
/// Upstream's `cmd_SET_FAN_SPEED` (`fan_generic.py:34-43`): exactly one of
/// `SPEED` and `TEMPLATE` must be given.
fn cmd_set_fan_speed(
    fan: &Arc<Fan>,
    fan_name: &str,
    template_eval: &Arc<PrinterTemplateEvaluator>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let speed = speed_word(gcmd)?;
    // Upstream: `gcmd.get('TEMPLATE', None)` — present or absent, never parsed.
    let template = gcmd.get_command_parameters().get("TEMPLATE").cloned();
    if speed.is_none() == template.is_none() {
        return Err(CommandError::new(
            "SET_FAN_SPEED must specify SPEED or TEMPLATE",
        ));
    }
    let Some(speed) = speed else {
        // `TEMPLATE=` (possibly empty: an unsubscribe, which drives nothing).
        let fan = Arc::clone(fan);
        let fan_name = fan_name.to_string();
        return template_eval.set_template(
            &format!("fan_generic {fan_name}"),
            gcmd,
            Arc::new(move |text| set_rendered_speed(&fan, &fan_name, text)),
        );
    };
    fan.set_speed_from_command(speed)
}

/// One rendered template's text as a fan speed: upstream's `_template_update`
/// (`fan_generic.py:27-32`) parses `float(text)`, falls back to `0.` with a
/// logged error, and hands the result to `Fan.set_speed`.
///
/// The timer callback has no error channel, so a failed send is logged the way
/// upstream logs it.
fn set_rendered_speed(fan: &Arc<Fan>, fan_name: &str, text: &str) {
    let speed = match text.trim().parse::<f64>() {
        Ok(speed) if speed.is_finite() => speed,
        _ => {
            warn!("fan_generic template render error: '{text}' is not a number");
            0.0
        }
    };
    if let Err(err) = fan.set_speed(speed) {
        warn!("Error setting fan_generic speed for fan '{fan_name}': {err}");
    }
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

    /// A `[display_template <name>]` section filed into the shared registry.
    fn add_display_template(printer: &Arc<Printer>, name: &str, text: &str) {
        let mut section = ConfigSection::new("display_template", Some(name));
        section
            .parameters
            .insert("text".to_string(), ConfigValue::Single(text.to_string()));
        crate::core::klippy::extras::led::load_display_template(&wrap(&section), printer)
            .expect("the template loads");
    }

    /// The shared evaluator a `TEMPLATE=` subscription lands in.
    fn evaluator(printer: &Arc<Printer>) -> Arc<PrinterTemplateEvaluator> {
        crate::core::klippy::extras::template_evaluator::lookup_template_eval(printer)
            .expect("the evaluator is created on first use")
    }

    #[test]
    fn test_set_fan_speed_template_re_renders_the_speed() {
        let (printer, chip) = printer();
        add_display_template(&printer, "slow", "0.25");
        add_display_template(&printer, "fast", "0.5");
        let fan = load(
            &printer,
            &section("side_fan", "PH0", &[("kick_start_time", "0")]),
        )
        .unwrap();
        let eval = evaluator(&printer);

        gcode(&printer)
            .run_script_sync("SET_FAN_SPEED FAN=side_fan TEMPLATE=slow")
            .unwrap();
        assert_eq!(eval.render_once(0.0), Some(0.5));
        assert_eq!(updates(&pwm(&chip, 0)), [0.25]);
        assert_eq!(speed(&fan), 0.25);

        // A second subscription replaces the first (one per fan), and the next
        // render period drives the new value.
        gcode(&printer)
            .run_script_sync("SET_FAN_SPEED FAN=side_fan TEMPLATE=fast")
            .unwrap();
        assert_eq!(eval.render_once(0.5), Some(1.0));
        assert_eq!(updates(&pwm(&chip, 0)), [0.25, 0.5]);
        assert_eq!(speed(&fan), 0.5);

        // `TEMPLATE=` cancels it: with nothing subscribed the timer retires and
        // the fan stays where it was.
        gcode(&printer)
            .run_script_sync("SET_FAN_SPEED FAN=side_fan TEMPLATE=")
            .unwrap();
        assert_eq!(eval.render_once(1.0), None);
        assert_eq!(updates(&pwm(&chip, 0)), [0.25, 0.5]);
    }

    #[test]
    fn test_set_fan_speed_template_must_name_a_display_template() {
        let (printer, chip) = printer();
        load(&printer, &section("side_fan", "PH0", &[])).unwrap();

        let err = gcode(&printer)
            .run_script_sync("SET_FAN_SPEED FAN=side_fan TEMPLATE=nosuch")
            .unwrap_err();

        assert_eq!(err.to_string(), "Unknown display_template 'nosuch'");
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
