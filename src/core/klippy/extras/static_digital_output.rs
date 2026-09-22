//! `[static_digital_output <name>]` — hold a list of pins at a fixed level.
//!
//! A board whose stepper drivers select microstepping through jumper pins can
//! drive those pins once at startup instead of wiring them. Each pin in `pins`
//! is set to its active level for the whole session with the firmware's
//! `set_digital_out` — no oid, no resource, nothing to change later.
//!
//! Upstream is `klippy/extras/static_digital_output.py`:
//!
//! ```python
//! pin_list = config.getlist('pins')
//! for pin_desc in pin_list:
//!     pin_params = ppins.lookup_pin(pin_desc, can_invert=True)
//!     mcu.add_config_cmd("set_digital_out pin=%s value=%d"
//!                        % (pin_params['pin'], not pin_params['invert']))
//! ```
//!
//! The shape maps one to one: [`ConfigWrapper::get_list`] for `getlist`,
//! [`PrinterPins::setup_static_digital_out`] for the `lookup_pin` +
//! `add_config_cmd` pair. The pin **name** is resolved inside the MCU's config
//! callback, the first moment the firmware dictionary exists, the same as the
//! other pin resources.
//!
//! # Repeated `pins:` lines
//!
//! The config parser keeps the **last** value of a repeated option (upstream's
//! `RawConfigParser(strict=False)` does the same), so a section written as four
//! `pins:` lines configures only the last one. That is upstream's behavior, not
//! a shortcut here; a single comma-separated `pins:` line is how to set more
//! than one.
//!
//! Like upstream's object, this one has no `get_status`, so it is not listed by
//! `objects/list`.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form (`[static_digital_output <name>]`) exists upstream.
// Order 35 puts it after `[board_pins]` (30), so an alias may name a pin.
section!(
    "static_digital_output",
    order = 35,
    prefix = load_config_prefix
);

/// One configured `[static_digital_output <name>]`.
pub struct StaticDigitalOutput {
    /// The section's identifier, for logging and `Debug`.
    identifier: String,
    /// How many pins were set.
    pins: usize,
}

impl StaticDigitalOutput {
    /// Reserve each pin and add its `set_digital_out` config command.
    ///
    /// # Errors
    /// Returns a config error when `pins` is missing, or when a pin
    /// description is invalid, reserved, or used elsewhere.
    pub fn new(config: &ConfigWrapper, printer: &Printer) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let pin_list = config.get_list("pins", ',').ok_or_else(|| {
            ConfigError::new(format!(
                "Option 'pins' in section '{identifier}' must be specified"
            ))
        })?;

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");

        for pin_desc in &pin_list {
            pins.setup_static_digital_out(pin_desc)
                .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        }

        Ok(Self {
            identifier,
            pins: pin_list.len(),
        })
    }

    /// The section identifier.
    pub fn identifier(&self) -> &str {
        &self.identifier
    }

    /// How many pins this section holds.
    pub fn pin_count(&self) -> usize {
        self.pins
    }
}

impl PrinterObject for StaticDigitalOutput {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for StaticDigitalOutput {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("StaticDigitalOutput")
            .field("identifier", &self.identifier)
            .field("pins", &self.pins)
            .finish()
    }
}

/// Upstream's `load_config_prefix` for `[static_digital_output <name>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(StaticDigitalOutput::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::section::ConfigSection;
    use crate::core::klippy::config::value::ConfigValue;
    use crate::core::klippy::pins::{DigitalOut, PinChip, PinError, PinParams};
    use crate::core::klippy::reactor::ManualReactor;
    use std::sync::Mutex;

    /// A chip that records the static outputs it was asked to add.
    #[derive(Default)]
    struct RecordingChip {
        added: Mutex<Vec<(String, bool)>>,
    }

    impl PinChip for RecordingChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            unreachable!("static outputs do not build a resource")
        }

        fn setup_static_digital_out(&self, params: &PinParams) -> Result<(), PinError> {
            self.added
                .lock()
                .unwrap()
                .push((params.pin.clone(), params.invert));
            Ok(())
        }
    }

    /// A machine with `pins` over a recording chip.
    fn machine() -> (Arc<Printer>, Arc<RecordingChip>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let pins = Arc::new(PrinterPins::new());
        let chip = Arc::new(RecordingChip::default());
        pins.register_chip("mcu", chip.clone()).unwrap();
        printer.add_object(PINS_OBJECT, pins).unwrap();
        (printer, chip)
    }

    /// An `[static_digital_output <name>]` section with the given options.
    fn section(name: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("static_digital_output", Some(name));
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    #[test]
    fn test_a_static_output_reserves_each_pin() {
        let (printer, chip) = machine();
        let section = section("jumpers", &[("pins", "PC10, PC29")]);

        let object =
            StaticDigitalOutput::new(&ConfigWrapper::untracked(&section), &printer).unwrap();

        assert_eq!(object.pin_count(), 2);
        assert_eq!(
            *chip.added.lock().unwrap(),
            [("PC10".to_string(), false), ("PC29".to_string(), false)]
        );
    }

    #[test]
    fn test_an_inverted_pin_is_recorded() {
        let (printer, chip) = machine();
        let section = section("jumpers", &[("pins", "!PC10")]);

        StaticDigitalOutput::new(&ConfigWrapper::untracked(&section), &printer).unwrap();

        assert_eq!(*chip.added.lock().unwrap(), [("PC10".to_string(), true)]);
    }

    #[test]
    fn test_a_missing_pins_option_is_reported() {
        let (printer, _chip) = machine();
        let section = section("jumpers", &[]);

        let err =
            StaticDigitalOutput::new(&ConfigWrapper::untracked(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'pins' in section 'static_digital_output jumpers' must be specified"
        );
    }
}
