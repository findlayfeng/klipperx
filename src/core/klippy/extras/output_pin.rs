//! `[output_pin <name>]` — a pin a client can set with `SET_PIN`.
//!
//! The first *consumer* of the pin stack: it reads a `pin` description, asks
//! `pins` for a digital output or a PWM (`mcu/resource/pin.rs`, `mcu/resource/pwm.rs`), tells it
//! the start and shutdown values, and registers `SET_PIN PIN=<name> VALUE=<0..1>`
//! with the G-Code dispatcher.
//!
//! Upstream is `klippy/extras/output_pin.py`. This port covers the **output**
//! subset:
//!
//! | option | meaning |
//! |---|---|
//! | `pin` | the pin description, required |
//! | `value` | value to drive at startup (default 0) |
//! | `shutdown_value` | value to fall back to on shutdown (default 0) |
//! | `pwm` | use a PWM rather than a plain digital output (default false) |
//! | `cycle_time` | PWM period in seconds (default 0.1) |
//! | `hardware_pwm` | use the firmware's hardware PWM (default false, software PWM) |
//!
//! `maximum_mcu_duration` is deliberately **not** an option: upstream's
//! `PrinterOutputPin` calls `setup_max_duration(0.)` unconditionally
//! (`klippy/extras/output_pin.py:217`), so the firmware's "return to the
//! shutdown value" limit is off and `value` and `shutdown_value` may differ.
//!
//! # What is not here
//!
//! * **Scheduling.** Upstream queues `SET_PIN` through the toolhead so the change
//!   lands at a print time. With no motion or print-time clock, this port calls
//!   the immediate forms (`update_digital_out`, `update_pwm`); the clocked
//!   `queue_digital_out` / `set_pwm` are used as soon as the clock layer exists
//!   (TODO C1). For a software PWM that means `update_pwm` aligns the change to
//!   the PWM cycle using the estimated clock.
//! * **`scale` / `static_value` / `template`**: the display-template machinery.

use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::gcode::{CommandError, CommandHandler, GCodeDispatch, GCODE_OBJECT};
use crate::core::klippy::load::section;
use crate::core::klippy::pins::{DigitalOut, PrinterPins, PwmOut, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form (`[output_pin <name>]`) exists upstream.
section!("output_pin", order = 20, prefix = load_config_prefix);

/// One configured `[output_pin <name>]`.
///
/// The resource itself is owned by the `SET_PIN` handler (the only thing that
/// drives it); this object keeps the name and the last value for `get_status`.
pub struct OutputPin {
    /// The name in `SET_PIN PIN=<name>`: the section's sub.
    name: String,
    /// The value last set, for `get_status`; shared with the `SET_PIN` handler.
    value: Arc<Mutex<f64>>,
}

/// What `SET_PIN` drives: a plain output or a PWM.
///
/// The two have different trait objects but the same `0..=1` client interface,
/// so the section keeps the choice behind one enum and the handler matches on it.
enum PinHandle {
    Digital(Arc<dyn DigitalOut>),
    Pwm(Arc<dyn PwmOut>),
}

impl OutputPin {
    /// Build the pin from its section and register `SET_PIN`.
    ///
    /// # Errors
    /// Returns a config error (a message naming the section) when an option is
    /// missing, unparseable, or asks for something this port does not do yet.
    pub fn new(config: &ConfigWrapper, printer: &Printer) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let name = config.section().sub.clone().ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{identifier}' must be a '[output_pin <name>]' section"
            ))
        })?;

        let pin_desc = config.get("pin", None)?;

        let value =
            config.get_float_bounded("value", Some(0.0), Some(0.0), Some(1.0), None, None)?;
        let shutdown_value = config.get_float_bounded(
            "shutdown_value",
            Some(0.0),
            Some(0.0),
            Some(1.0),
            None,
            None,
        )?;

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");

        // Upstream disables the firmware's max-duration limit for an
        // `output_pin` unconditionally, which is what lets `value` and
        // `shutdown_value` differ.
        let handle = if config.get_bool("pwm", Some(false))? {
            let pwm = pins
                .setup_pwm(&pin_desc, None)
                .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
            let cycle_time =
                config.get_float_bounded("cycle_time", Some(0.100), None, None, Some(0.0), None)?;
            let hardware_pwm = config.get_bool("hardware_pwm", Some(false))?;
            pwm.setup_cycle_time(cycle_time, hardware_pwm);
            pwm.setup_max_duration(0.0);
            pwm.setup_start_value(value, shutdown_value);
            PinHandle::Pwm(pwm)
        } else {
            let pin = pins
                .setup_digital_out(&pin_desc, None)
                .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
            pin.setup_max_duration(0.0);
            pin.setup_start_value(value >= 0.5, shutdown_value >= 0.5);
            PinHandle::Digital(pin)
        };

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let value_slot = Arc::new(Mutex::new(value));
        let handle = Arc::new(handle);
        let handler: CommandHandler = {
            let handle = Arc::clone(&handle);
            let value_slot = Arc::clone(&value_slot);
            Arc::new(move |gcmd| cmd_set_pin(&handle, &value_slot, gcmd))
        };
        gcode
            .register_mux_command(
                "SET_PIN",
                "PIN",
                Some(&name),
                handler,
                Some("Set the value of a pin"),
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        Ok(Self {
            name,
            value: value_slot,
        })
    }

    /// The name `SET_PIN` addresses this pin by.
    pub fn name(&self) -> &str {
        &self.name
    }

    fn lock(&self) -> MutexGuard<'_, f64> {
        self.value
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl PrinterObject for OutputPin {
    /// The value last set, as upstream's `PrinterOutputPin.get_status`.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "value": *self.lock() })
    }
}

impl std::fmt::Debug for OutputPin {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OutputPin")
            .field("name", &self.name)
            .finish_non_exhaustive()
    }
}

/// `SET_PIN PIN=<name> VALUE=<0..1>`: drive the pin.
///
/// Upstream schedules this at a print time; without a clock layer the change
/// happens through the resource's immediate path. A digital output treats
/// `VALUE >= 0.5` as "on" (so a config that writes `0.5` does what upstream
/// does); a PWM takes the value as a duty.
fn cmd_set_pin(
    handle: &PinHandle,
    value_slot: &Arc<Mutex<f64>>,
    gcmd: &crate::core::klippy::gcode::GcodeCommand,
) -> Result<(), CommandError> {
    let value = gcmd
        .get_float_range("VALUE", 0.0, 1.0)
        .map_err(|err| CommandError::new(err.to_string()))?;
    let result = match handle {
        PinHandle::Digital(pin) => pin.update_digital_out(value >= 0.5),
        PinHandle::Pwm(pin) => pin.update_pwm(value),
    };
    result.map_err(|err| CommandError::new(err.to_string()))?;
    *value_slot
        .lock()
        .unwrap_or_else(|poison| poison.into_inner()) = value;
    Ok(())
}

/// Upstream's `load_config_prefix` for `[output_pin <name>]`.
///
/// The loader registers the object under the section identifier
/// (`output_pin fan`); `SET_PIN` addresses it by the sub (`fan`).
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(OutputPin::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{ConfigSection, ConfigValue};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::mcu::McuError;
    use crate::core::klippy::pins::{PinChip, PinError, PinParams, PwmOut};
    use crate::core::klippy::reactor::ManualReactor;

    /// A digital output that records what it was told.
    #[derive(Default)]
    struct FakeDigitalOut {
        max_duration: Mutex<f64>,
        start_value: Mutex<(bool, bool)>,
        updates: Mutex<Vec<bool>>,
    }

    impl DigitalOut for FakeDigitalOut {
        fn setup_max_duration(&self, max_duration: f64) {
            *self.max_duration.lock().unwrap() = max_duration;
        }
        fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
            *self.start_value.lock().unwrap() = (start_value, shutdown_value);
        }
        fn queue_digital_out(&self, _clock: u32, _value: bool) -> Result<(), McuError> {
            Ok(())
        }
        fn update_digital_out(&self, value: bool) -> Result<(), McuError> {
            self.updates.lock().unwrap().push(value);
            Ok(())
        }
    }

    /// A PWM that records what it was told.
    #[derive(Default)]
    struct FakePwm {
        max_duration: Mutex<f64>,
        cycle_time: Mutex<(f64, bool)>,
        start_value: Mutex<(f64, f64)>,
        updates: Mutex<Vec<f64>>,
    }

    impl PwmOut for FakePwm {
        fn setup_max_duration(&self, max_duration: f64) {
            *self.max_duration.lock().unwrap() = max_duration;
        }
        fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) {
            *self.cycle_time.lock().unwrap() = (cycle_time, hardware_pwm);
        }
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

    /// A chip that hands out a [`FakeDigitalOut`] or [`FakePwm`] per setup.
    #[derive(Default)]
    struct FakeChip {
        created: Mutex<Vec<Arc<FakeDigitalOut>>>,
        pwms: Mutex<Vec<Arc<FakePwm>>>,
    }

    impl PinChip for FakeChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            let out = Arc::new(FakeDigitalOut::default());
            self.created.lock().unwrap().push(Arc::clone(&out));
            Ok(out)
        }

        fn setup_pwm(&self, _params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
            let pwm = Arc::new(FakePwm::default());
            self.pwms.lock().unwrap().push(Arc::clone(&pwm));
            Ok(pwm)
        }
    }

    /// A ready printer with `gcode` and `pins` over a fake chip.
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
        printer.send_event(&KlippyEvent::KlippyReady);
        (printer, chip)
    }

    /// An `[output_pin <name>]` section with `pin: <pin>` plus `options`.
    fn section(name: &str, pin: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("output_pin", Some(name));
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

    /// Wrap a hand-built section the way the loader does.
    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap()
    }

    fn created(chip: &FakeChip, index: usize) -> Arc<FakeDigitalOut> {
        chip.created.lock().unwrap()[index].clone()
    }

    #[test]
    fn test_a_digital_output_is_configured_with_its_levels() {
        let (printer, chip) = printer();
        let section = section("fan", "PA1", &[("value", "1"), ("shutdown_value", "0")]);

        OutputPin::new(&wrap(&section), &printer).unwrap();

        let out = created(&chip, 0);
        assert_eq!(*out.max_duration.lock().unwrap(), 0.0);
        assert_eq!(*out.start_value.lock().unwrap(), (true, false));
    }

    #[test]
    fn test_set_pin_drives_the_output() {
        let (printer, chip) = printer();
        let pin = OutputPin::new(&wrap(&section("fan", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=fan VALUE=1")
            .unwrap();
        assert_eq!(*created(&chip, 0).updates.lock().unwrap(), [true]);
        assert_eq!(pin.get_status(0.0)["value"], 1.0);

        gcode(&printer)
            .run_script("SET_PIN PIN=fan VALUE=0")
            .unwrap();
        assert_eq!(*created(&chip, 0).updates.lock().unwrap(), [true, false]);
        assert_eq!(pin.get_status(0.0)["value"], 0.0);
    }

    #[test]
    fn test_set_pin_treats_a_half_as_on() {
        let (printer, chip) = printer();
        OutputPin::new(&wrap(&section("fan", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=fan VALUE=0.5")
            .unwrap();

        assert_eq!(*created(&chip, 0).updates.lock().unwrap(), [true]);
    }

    #[test]
    fn test_set_pin_requires_a_value() {
        let (printer, _chip) = printer();
        OutputPin::new(&wrap(&section("fan", "PA1", &[])), &printer).unwrap();

        let err = gcode(&printer).run_script("SET_PIN PIN=fan").unwrap_err();

        assert!(err.to_string().contains("missing VALUE"), "{err}");
    }

    #[test]
    fn test_two_pins_are_driven_independently() {
        let (printer, chip) = printer();
        OutputPin::new(&wrap(&section("fan", "PA1", &[])), &printer).unwrap();
        OutputPin::new(&wrap(&section("light", "PA2", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=light VALUE=1")
            .unwrap();

        assert!(created(&chip, 0).updates.lock().unwrap().is_empty());
        assert_eq!(*created(&chip, 1).updates.lock().unwrap(), [true]);
    }

    #[test]
    fn test_a_missing_pin_names_the_section() {
        let (printer, _chip) = printer();
        let section = ConfigSection::new("output_pin", Some("fan"));

        let err = OutputPin::new(&wrap(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'pin' in section 'output_pin fan' must be specified"
        );
    }

    #[test]
    fn test_an_unparseable_value_is_reported() {
        let (printer, _chip) = printer();

        let err = OutputPin::new(&wrap(&section("fan", "PA1", &[("value", "abc")])), &printer)
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Unable to parse option 'value' in section 'output_pin fan'"
        );
    }

    #[test]
    fn test_a_pwm_output_is_configured_and_driven() {
        let (printer, chip) = printer();
        let section = section(
            "fan",
            "PA1",
            &[("pwm", "true"), ("cycle_time", "0.05"), ("value", "0.5")],
        );

        OutputPin::new(&wrap(&section), &printer).unwrap();

        let pwm = chip.pwms.lock().unwrap()[0].clone();
        assert_eq!(*pwm.max_duration.lock().unwrap(), 0.0);
        assert_eq!(*pwm.cycle_time.lock().unwrap(), (0.05, false));
        assert_eq!(*pwm.start_value.lock().unwrap(), (0.5, 0.0));

        gcode(&printer)
            .run_script("SET_PIN PIN=fan VALUE=0.25")
            .unwrap();
        assert_eq!(*pwm.updates.lock().unwrap(), [0.25]);
    }

    #[test]
    fn test_a_hardware_pwm_is_selected_by_its_option() {
        let (printer, chip) = printer();
        let section = section("fan", "PA1", &[("pwm", "true"), ("hardware_pwm", "true")]);

        OutputPin::new(&wrap(&section), &printer).unwrap();

        assert_eq!(
            *chip.pwms.lock().unwrap()[0].cycle_time.lock().unwrap(),
            (0.1, true)
        );
    }

    #[test]
    fn test_a_non_positive_cycle_time_is_reported() {
        let (printer, _chip) = printer();
        let section = section("fan", "PA1", &[("pwm", "true"), ("cycle_time", "0")]);

        let err = OutputPin::new(&wrap(&section), &printer).unwrap_err();

        assert!(err.to_string().contains("cycle_time"), "{err}");
        assert!(err.to_string().contains("above 0"), "{err}");
    }

    #[test]
    fn test_an_unparseable_boolean_is_reported() {
        let (printer, _chip) = printer();

        let err = OutputPin::new(&wrap(&section("fan", "PA1", &[("pwm", "maybe")])), &printer)
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Unable to parse option 'pwm' in section 'output_pin fan'"
        );
    }
}
