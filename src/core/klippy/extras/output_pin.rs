//! `[output_pin <name>]` — a pin a client can set with `SET_PIN`.
//!
//! The first *consumer* of the pin stack: it reads a `pin` description, asks
//! `pins` for a digital output (`mcu/pin.rs`), tells it the start and shutdown
//! levels, and registers `SET_PIN PIN=<name> VALUE=<0..1>` with the G-Code
//! dispatcher.
//!
//! Upstream is `klippy/extras/output_pin.py`. This port covers the **digital
//! output** subset:
//!
//! | option | meaning |
//! |---|---|
//! | `pin` | the pin description, required |
//! | `value` | level to drive at startup (default 0) |
//! | `shutdown_value` | level the firmware falls back to (default 0) |
//! | `maximum_mcu_duration` | longest a scheduled change may be outstanding, seconds (default 2) |
//!
//! # What is not here
//!
//! * **PWM** (`pwm` / `cycle_time`): the pin stack has no PWM resource yet
//!   (TODO F4). A section that asks for it is refused rather than silently
//!   driven as a digital output.
//! * **Scheduling.** Upstream queues `SET_PIN` through the toolhead so the change
//!   lands at a print time. With no motion or print-time clock, this port calls
//!   `update_digital_out` — the change happens now. `McuDigitalOut` already
//!   exposes the clocked `queue_digital_out`, so the scheduled path is a
//!   drop-in once the clock layer exists.
//! * **`scale` / `static_value` / `template`**: the display-template machinery.

use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::config::ConfigSection;
use crate::core::klippy::gcode::{CommandError, CommandHandler, GCodeDispatch, GCODE_OBJECT};
use crate::core::klippy::pins::{DigitalOut, PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The default `maximum_mcu_duration`, matching upstream.
const DEFAULT_MAX_DURATION: f64 = 2.0;

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

impl OutputPin {
    /// Build the pin from its section and register `SET_PIN`.
    ///
    /// # Errors
    /// Returns a config error (a message naming the section) when an option is
    /// missing, unparseable, or asks for something this port does not do yet.
    pub fn new(section: &ConfigSection, printer: &Printer) -> Result<Self, String> {
        let identifier = section.identifier();
        let name = section.sub.clone().ok_or_else(|| {
            format!("Section '{identifier}' must be a '[output_pin <name>]' section")
        })?;

        let pin_desc = section
            .get_str("pin")
            .ok_or_else(|| format!("Option 'pin' in section '{identifier}' is not specified"))?;
        if section.get_str("pwm").is_some() {
            return Err(format!(
                "Option 'pwm' in section '{identifier}' is not supported yet \
                 (see TODO F4); this host drives digital outputs only"
            ));
        }

        let value = get_float(section, "value")?.unwrap_or(0.0);
        let shutdown_value = get_float(section, "shutdown_value")?.unwrap_or(0.0);
        let maximum_mcu_duration =
            get_float(section, "maximum_mcu_duration")?.unwrap_or(DEFAULT_MAX_DURATION);
        for (option, v) in [("value", value), ("shutdown_value", shutdown_value)] {
            if !(0.0..=1.0).contains(&v) {
                return Err(format!(
                    "Option '{option}' in section '{identifier}' must be between 0 and 1"
                ));
            }
        }
        if maximum_mcu_duration < 0.0 {
            return Err(format!(
                "Option 'maximum_mcu_duration' in section '{identifier}' must not be negative"
            ));
        }

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let pin = pins
            .setup_digital_out(pin_desc, None)
            .map_err(|err| format!("{identifier}: {err}"))?;
        pin.setup_max_duration(maximum_mcu_duration);
        pin.setup_start_value(value >= 0.5, shutdown_value >= 0.5);

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let value_slot = Arc::new(Mutex::new(value));
        let handler: CommandHandler = {
            let pin = Arc::clone(&pin);
            let value_slot = Arc::clone(&value_slot);
            Arc::new(move |gcmd| cmd_set_pin(&pin, &value_slot, gcmd))
        };
        gcode
            .register_mux_command(
                "SET_PIN",
                "PIN",
                Some(&name),
                handler,
                Some("Set the value of a pin"),
            )
            .map_err(|err| format!("{identifier}: {err}"))?;

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
/// happens immediately. `VALUE >= 0.5` is "on", so a config that writes `0.5`
/// for a digital output does what upstream does.
fn cmd_set_pin(
    pin: &Arc<dyn DigitalOut>,
    value_slot: &Arc<Mutex<f64>>,
    gcmd: &crate::core::klippy::gcode::GcodeCommand,
) -> Result<(), CommandError> {
    let value = gcmd
        .get_float_range("VALUE", 0.0, 1.0)
        .map_err(|err| CommandError::new(err.to_string()))?;
    pin.update_digital_out(value >= 0.5)
        .map_err(|err| CommandError::new(err.to_string()))?;
    *value_slot
        .lock()
        .unwrap_or_else(|poison| poison.into_inner()) = value;
    Ok(())
}

/// Read a float option, reporting upstream's parse error when it is malformed.
fn get_float(section: &ConfigSection, name: &str) -> Result<Option<f64>, String> {
    let Some(text) = section.get_str(name) else {
        return Ok(None);
    };
    text.trim().parse::<f64>().map(Some).map_err(|_| {
        format!(
            "Unable to parse option '{name}' in section '{}'",
            section.identifier()
        )
    })
}

/// Upstream's `load_config_prefix` for `[output_pin <name>]`.
///
/// The loader registers the object under the section identifier
/// (`output_pin fan`); `SET_PIN` addresses it by the sub (`fan`).
pub fn load_config_prefix(
    section: &ConfigSection,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, String> {
    Ok(Arc::new(OutputPin::new(section, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::ConfigValue;
    use crate::core::klippy::mcu::McuError;
    use crate::core::klippy::pins::{PinChip, PinError, PinParams};
    use crate::core::klippy::printer::PrinterEvent;
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

    /// A chip that hands out a [`FakeDigitalOut`] per setup.
    #[derive(Default)]
    struct FakeChip {
        created: Mutex<Vec<Arc<FakeDigitalOut>>>,
    }

    impl PinChip for FakeChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            let out = Arc::new(FakeDigitalOut::default());
            self.created.lock().unwrap().push(Arc::clone(&out));
            Ok(out)
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
        printer.send_event(&PrinterEvent::Ready);
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
        let section = section(
            "fan",
            "PA1",
            &[
                ("value", "1"),
                ("shutdown_value", "0"),
                ("maximum_mcu_duration", "0"),
            ],
        );

        OutputPin::new(&section, &printer).unwrap();

        let out = created(&chip, 0);
        assert_eq!(*out.max_duration.lock().unwrap(), 0.0);
        assert_eq!(*out.start_value.lock().unwrap(), (true, false));
    }

    #[test]
    fn test_set_pin_drives_the_output() {
        let (printer, chip) = printer();
        let pin = OutputPin::new(&section("fan", "PA1", &[]), &printer).unwrap();

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
        OutputPin::new(&section("fan", "PA1", &[]), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=fan VALUE=0.5")
            .unwrap();

        assert_eq!(*created(&chip, 0).updates.lock().unwrap(), [true]);
    }

    #[test]
    fn test_set_pin_requires_a_value() {
        let (printer, _chip) = printer();
        OutputPin::new(&section("fan", "PA1", &[]), &printer).unwrap();

        let err = gcode(&printer).run_script("SET_PIN PIN=fan").unwrap_err();

        assert!(err.to_string().contains("missing VALUE"), "{err}");
    }

    #[test]
    fn test_two_pins_are_driven_independently() {
        let (printer, chip) = printer();
        OutputPin::new(&section("fan", "PA1", &[]), &printer).unwrap();
        OutputPin::new(&section("light", "PA2", &[]), &printer).unwrap();

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

        let err = OutputPin::new(&section, &printer).unwrap_err();

        assert_eq!(
            err,
            "Option 'pin' in section 'output_pin fan' is not specified"
        );
    }

    #[test]
    fn test_an_unparseable_value_is_reported() {
        let (printer, _chip) = printer();

        let err =
            OutputPin::new(&section("fan", "PA1", &[("value", "abc")]), &printer).unwrap_err();

        assert_eq!(
            err,
            "Unable to parse option 'value' in section 'output_pin fan'"
        );
    }

    #[test]
    fn test_pwm_is_refused_rather_than_driven_as_digital() {
        let (printer, _chip) = printer();

        let err = OutputPin::new(&section("fan", "PA1", &[("pwm", "True")]), &printer).unwrap_err();

        assert!(err.contains("'pwm'"), "{err}");
        assert!(err.contains("not supported yet"), "{err}");
    }
}
