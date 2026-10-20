//! `[duplicate_pin_override]` — let a list of pins be used by more than one
//! section.
//!
//! Upstream's `klippy/extras/duplicate_pin_override.py`, all fifteen lines of
//! it:
//!
//! ```python
//! for pin_desc in config.getlist('pins'):
//!     ppins.allow_multi_use_pin(pin_desc)
//! ```
//!
//! The section exists because the pin registry refuses a second owner for a pin
//! unless both users name the same share type. A board that genuinely drives
//! one pin from two places — the corpus has none, but the escape hatch is
//! upstream's — declares that pin here once, before the sections that use it,
//! and the second lookup is allowed through.
//!
//! | option | meaning |
//! |---|---|
//! | `pins` | comma-separated pin descriptions, required |
//!
//! Nothing else is built: no resource, no oid, no firmware command, no
//! `get_status` (so `objects/list` does not show it). The one change it makes is
//! to [`PrinterPins::allow_multi_use_pin`]'s table.
//!
//! A decoration is not part of the declaration: [`PrinterPins::allow_multi_use_pin`]
//! parses with `can_invert`/`can_pullup` off, as upstream does, so `pins: !PA1`
//! is an invalid description. The polarity is written on the section that
//! actually drives the pin.
//!
//! # Load phase
//!
//! `phase = early` with `order = 17`. Upstream loads a section when the config
//! file reaches it, so where the user puts `[duplicate_pin_override]` decides
//! what it can cover; this loader walks a fixed table, so the position is the
//! declaration's. It has to come before the sections that reuse a pin — the
//! table is consulted on the *second* lookup — and after the sections that
//! *register a chip*, because a description naming an unregistered chip is
//! refused with `Unknown pin chip name` while this section loads. The early
//! phase holds `[mcu]` (10), `[multi_pin]` (15) and `[sx1509]` (16); 17 is
//! after all three and still ahead of `[force_move]` (18), the bridge-port
//! sections (19) and the whole generic block (`[adc_scaled]`, `[extruder]`,
//! `[output_pin]`, … at 20 and up).
//!
//! A chip registered later than 17 (`[adc_scaled]`'s virtual pins, for one)
//! cannot be named in `pins:`; upstream could, by putting this section after
//! that one. No upstream config uses the section at all
//! (`Config_Reference.md` documents it, `test/config/` has no fixture), so the
//! fixed table is the whole difference.
//!
//! # The name is tracked as written
//!
//! The multi-use table is keyed `chip:pin` with the description the user wrote
//! (upstream `pins.py:132-135`; here [`PinParams`](crate::core::klippy::pins::PinParams)'s
//! share name), and a `[board_pins]` alias becomes a pin number only when the
//! firmware command is built. So `pins: my_alias` covers the sections that also
//! spell `my_alias`, not the ones that spell its target: the two spellings of
//! one pin are two rows in the table, as they are upstream. An alias may be
//! declared here even though `[board_pins]` (order 30) loads later — parsing a
//! description needs only the chip name.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the plain form (`[duplicate_pin_override]`) exists upstream
// (`duplicate_pin_override.py:14-15`); `phase = early` is load-bearing, see the
// module docs.
section!(
    "duplicate_pin_override",
    order = 17,
    phase = early,
    load = load_config
);

/// The configured `[duplicate_pin_override]`.
pub struct DuplicatePinOverride {
    /// The section's identifier, for logging and `Debug`.
    identifier: String,
    /// How many pins were declared multi-use.
    pins: usize,
}

impl DuplicatePinOverride {
    /// Declare each pin in `pins` multi-use.
    ///
    /// # Errors
    /// Returns a config error when `pins` is missing, or when a description is
    /// malformed or names an unknown chip.
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
            pins.allow_multi_use_pin(pin_desc)
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

    /// How many pins this section declared multi-use.
    pub fn pin_count(&self) -> usize {
        self.pins
    }
}

impl PrinterObject for DuplicatePinOverride {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for DuplicatePinOverride {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DuplicatePinOverride")
            .field("identifier", &self.identifier)
            .field("pins", &self.pins)
            .finish()
    }
}

/// Upstream's `load_config` for `[duplicate_pin_override]`.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(DuplicatePinOverride::new(config, printer)?))
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

    /// A chip that builds nothing: only description parsing matters here.
    struct InertChip;

    impl PinChip for InertChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            unreachable!("[duplicate_pin_override] builds no resource")
        }
    }

    /// A machine with `pins` over an inert chip.
    fn machine() -> (Arc<Printer>, Arc<PrinterPins>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let pins = Arc::new(PrinterPins::new());
        pins.register_chip("mcu", Arc::new(InertChip)).unwrap();
        printer.add_object(PINS_OBJECT, pins.clone()).unwrap();
        (printer, pins)
    }

    /// A `[duplicate_pin_override]` section with the given options.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("duplicate_pin_override", None);
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    #[test]
    fn test_a_listed_pin_may_be_used_by_two_owners() {
        let (printer, pins) = machine();
        let section = section(&[("pins", "PA1")]);

        let object =
            DuplicatePinOverride::new(&ConfigWrapper::untracked(&section), &printer).unwrap();

        assert_eq!(object.pin_count(), 1);
        pins.lookup_pin("PA1", false, false, None).unwrap();
        pins.lookup_pin("PA1", false, false, None).unwrap();
    }

    #[test]
    fn test_an_unlisted_pin_still_refuses_the_second_owner() {
        let (printer, pins) = machine();
        let section = section(&[("pins", "PA1")]);

        DuplicatePinOverride::new(&ConfigWrapper::untracked(&section), &printer).unwrap();

        pins.lookup_pin("PA2", false, false, None).unwrap();
        let err = pins.lookup_pin("PA2", false, false, None).unwrap_err();
        assert_eq!(err, PinError::UsedMultipleTimes("PA2".to_string()));
    }

    #[test]
    fn test_a_missing_pins_option_is_reported() {
        let (printer, _pins) = machine();
        let section = section(&[]);

        let err =
            DuplicatePinOverride::new(&ConfigWrapper::untracked(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'pins' in section 'duplicate_pin_override' must be specified"
        );
    }

    #[test]
    fn test_an_alias_is_declared_under_the_name_it_is_written() {
        let (printer, pins) = machine();
        pins.alias_pin("mcu", "my_alias", "PA1").unwrap();
        let section = section(&[("pins", "my_alias")]);

        DuplicatePinOverride::new(&ConfigWrapper::untracked(&section), &printer).unwrap();

        pins.lookup_pin("my_alias", false, false, None).unwrap();
        pins.lookup_pin("my_alias", false, false, None).unwrap();

        // The target spelled out is a different row, so it is still single-use.
        pins.lookup_pin("PA1", false, false, None).unwrap();
        assert_eq!(
            pins.lookup_pin("PA1", false, false, None).unwrap_err(),
            PinError::UsedMultipleTimes("PA1".to_string())
        );
    }

    #[test]
    fn test_the_section_is_not_queryable() {
        let (printer, _pins) = machine();
        let section = section(&[("pins", "PA1")]);

        let object =
            DuplicatePinOverride::new(&ConfigWrapper::untracked(&section), &printer).unwrap();

        assert!(!object.is_queryable());
        assert_eq!(object.get_status(0.0), json!({}));
    }
}
