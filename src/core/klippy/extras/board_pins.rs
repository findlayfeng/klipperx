//! `[board_pins]` — user-defined pin aliases and reservations.
//!
//! A board's schematic names its headers, not the MCU's pins. This section
//! lets a config say `EXP1_1=PA0` once and then use `EXP1_1` wherever a pin is
//! expected. A value wrapped in angle brackets (`EXP1_9=<GND>`) reserves the
//! pin instead: it is not an alias to use, it is a statement that the pin is
//! spoken for, so a later config that tries to drive it fails with
//! `pin EXP1_9 is reserved for <GND>`.
//!
//! Upstream is `klippy/extras/board_pins.py`, and the shape maps one to one:
//!
//! | upstream | here |
//! |---|---|
//! | `config.getlist('mcu', ('mcu',))` | [`ConfigSection::get_list`] |
//! | `options = ["aliases"] + config.get_prefix_options("aliases_")` | `aliases` then the sorted `aliases_*` |
//! | `config.getlists(opt, seps=('=', ','), count=2)` | [`ConfigSection::get_list_of_lists`] |
//! | `pin_resolver.reserve_pin(name, value)` | [`PrinterPins::reserve_pin`] |
//! | `pin_resolver.alias_pin(name, value)` | [`PrinterPins::alias_pin`] |
//!
//! # Two deviations
//!
//! * **Option order.** Upstream walks `aliases` and then the `aliases_*` options
//!   in the order the file declares them. Our parser stores a section's
//!   parameters in a `HashMap`, which has no order, so the `aliases_*` options
//!   are sorted by name. The entries are independent in practice (an alias
//!   target is a pin name, and a reservation only conflicts on the same pin), so
//!   only the wording of a conflicting error could differ.
//! * **`get_status`.** Upstream's object has none, so `objects/list` leaves it
//!   out; this one reports an empty object and is not queryable, the same way
//!   [`PrinterPins`](crate::core::klippy::pins::PrinterPins) is handled.
//!
//! The section is a *prefix* section with or without a sub (`[board_pins]` and
//! `[board_pins my_aliases]` are both valid), which is why the loader registers
//! both entry points.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::ConfigSection;
use crate::core::klippy::load::section;
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Both `[board_pins]` and `[board_pins <name>]` are valid.
section!(
    "board_pins",
    order = 30,
    load = load_config,
    prefix = load_config_prefix
);

/// One `[board_pins]` / `[board_pins <name>]` section.
pub struct BoardPins {
    /// The section's identifier, for logging and `Debug`.
    identifier: String,
    /// How many aliases were created (`aliases_*` included).
    aliases: usize,
    /// How many pins were reserved with the `<…>` form.
    reserved: usize,
}

impl BoardPins {
    /// Apply the section's aliases and reservations to the pin registry.
    ///
    /// # Errors
    /// Returns a config-error message when the `mcu` list names an unknown
    /// chip, when an option is malformed, or when an alias/reservation
    /// conflicts with one already recorded.
    pub fn new(section: &ConfigSection, printer: &Printer) -> Result<Self, String> {
        let identifier = section.identifier();
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");

        // The MCUs this section applies to; a missing option means the main
        // `mcu`, as upstream's `config.getlist('mcu', ('mcu',))` does.
        let mcu_names = section
            .get_list("mcu", ',')
            .unwrap_or_else(|| vec!["mcu".to_string()]);
        for name in &mcu_names {
            if !pins.chips().iter().any(|chip| chip == name) {
                return Err(format!("Unknown chip name '{name}'"));
            }
        }

        // `aliases` first, then the `aliases_*` options in a deterministic
        // order (see the module docs).
        let mut options: Vec<String> = Vec::new();
        if section.has("aliases") {
            options.push("aliases".to_string());
        }
        let mut prefixed: Vec<String> = section
            .parameters
            .keys()
            .filter(|key| key.starts_with("aliases_"))
            .cloned()
            .collect();
        prefixed.sort();
        options.extend(prefixed);

        let mut aliases = 0;
        let mut reserved = 0;
        for option in options {
            let groups = section.get_list_of_lists(&option, ',', '=', 2)?;
            for group in groups {
                let (name, value) = (&group[0], &group[1]);
                if value.starts_with('<') && value.ends_with('>') {
                    for chip in &mcu_names {
                        pins.reserve_pin(chip, name, value)
                            .map_err(|err| format!("{identifier}: {err}"))?;
                    }
                    reserved += 1;
                } else {
                    for chip in &mcu_names {
                        pins.alias_pin(chip, name, value)
                            .map_err(|err| format!("{identifier}: {err}"))?;
                    }
                    aliases += 1;
                }
            }
        }

        Ok(Self {
            identifier,
            aliases,
            reserved,
        })
    }

    /// The section this object was built from.
    pub fn identifier(&self) -> &str {
        &self.identifier
    }

    /// Number of aliases created.
    pub fn aliases(&self) -> usize {
        self.aliases
    }

    /// Number of pins reserved with the `<…>` form.
    pub fn reserved(&self) -> usize {
        self.reserved
    }
}

impl PrinterObject for BoardPins {
    /// Never called through the API: [`PrinterObject::is_queryable`] is false.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for BoardPins {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BoardPins")
            .field("identifier", &self.identifier)
            .field("aliases", &self.aliases)
            .field("reserved", &self.reserved)
            .finish()
    }
}

/// Upstream's `load_config` / `load_config_prefix` for `[board_pins]`.
///
/// Both entry points do the same work; the loader calls one for `[board_pins]`
/// and the other for `[board_pins <name>]`.
pub fn load_config(
    section: &ConfigSection,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, String> {
    Ok(Arc::new(BoardPins::new(section, printer)?))
}

/// The prefix (`[board_pins <name>]`) entry point.
pub fn load_config_prefix(
    section: &ConfigSection,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, String> {
    Ok(Arc::new(BoardPins::new(section, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::ConfigValue;
    use crate::core::klippy::pins::{
        DigitalOut, PinChip, PinError, PinParams, PrinterPins, PINS_OBJECT,
    };
    use crate::core::klippy::reactor::ManualReactor;

    /// A chip that builds nothing; the tests here only exercise the registry.
    #[derive(Default)]
    struct NoopChip;

    impl PinChip for NoopChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            Err(PinError::Unsupported("digital_out".to_string()))
        }
    }

    fn printer_with(chips: &[&str]) -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let pins = Arc::new(PrinterPins::new());
        for chip in chips {
            pins.register_chip(chip, Arc::new(NoopChip)).unwrap();
        }
        printer.add_object(PINS_OBJECT, pins).unwrap();
        printer
    }

    fn section(name: Option<&str>, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("board_pins", name);
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    fn pins(printer: &Arc<Printer>) -> Arc<PrinterPins> {
        printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .unwrap()
    }

    #[test]
    fn test_aliases_are_registered_on_the_main_mcu() {
        let printer = printer_with(&["mcu"]);
        let section = section(None, &[("aliases", "EXP1_1=PA0, EXP1_2=PA1")]);

        let object = BoardPins::new(&section, &printer).unwrap();

        assert_eq!(object.aliases(), 2);
        assert_eq!(object.reserved(), 0);
        let pins = pins(&printer);
        assert_eq!(pins.resolve_pin("mcu", "EXP1_1").unwrap(), "PA0");
        assert_eq!(pins.resolve_pin("mcu", "EXP1_2").unwrap(), "PA1");
    }

    #[test]
    fn test_an_aliases_prefix_option_is_read_too() {
        let printer = printer_with(&["mcu"]);
        let section = section(None, &[("aliases", "A=PA0"), ("aliases_extra", "B=PA1")]);

        let object = BoardPins::new(&section, &printer).unwrap();

        assert_eq!(object.aliases(), 2);
        let pins = pins(&printer);
        assert_eq!(pins.resolve_pin("mcu", "A").unwrap(), "PA0");
        assert_eq!(pins.resolve_pin("mcu", "B").unwrap(), "PA1");
    }

    #[test]
    fn test_the_mcu_option_targets_other_chips() {
        let printer = printer_with(&["mcu", "zboard"]);
        let section = section(Some("other"), &[("mcu", "zboard"), ("aliases", "X=PA0")]);

        BoardPins::new(&section, &printer).unwrap();

        let pins = pins(&printer);
        assert_eq!(pins.resolve_pin("zboard", "X").unwrap(), "PA0");
        // The main mcu does not know the alias.
        assert_eq!(pins.resolve_pin("mcu", "X").unwrap(), "X");
    }

    #[test]
    fn test_an_angle_bracket_value_reserves_the_pin() {
        let printer = printer_with(&["mcu"]);
        let section = section(None, &[("aliases", "GND_PIN=<GND>, A=PA0")]);

        let object = BoardPins::new(&section, &printer).unwrap();

        assert_eq!(object.aliases(), 1);
        assert_eq!(object.reserved(), 1);
        let err = pins(&printer).resolve_pin("mcu", "GND_PIN").unwrap_err();
        assert!(err.to_string().contains("reserved for <GND>"), "{err}");
    }

    #[test]
    fn test_an_unknown_mcu_chip_is_reported() {
        let printer = printer_with(&["mcu"]);
        let section = section(None, &[("mcu", "nope"), ("aliases", "A=PA0")]);

        let err = BoardPins::new(&section, &printer).unwrap_err();

        assert_eq!(err, "Unknown chip name 'nope'");
    }

    #[test]
    fn test_a_malformed_alias_names_the_section() {
        let printer = printer_with(&["mcu"]);
        let section = section(Some("bad"), &[("aliases", "A=PA0=PB0")]);

        let err = BoardPins::new(&section, &printer).unwrap_err();

        assert_eq!(
            err,
            "Option 'aliases' in section 'board_pins bad' must have 2 elements"
        );
    }

    #[test]
    fn test_a_conflicting_alias_is_wrapped_with_the_section() {
        let printer = printer_with(&["mcu"]);
        pins(&printer).alias_pin("mcu", "A", "PA9").unwrap();
        let section = section(None, &[("aliases", "A=PA0")]);

        let err = BoardPins::new(&section, &printer).unwrap_err();

        assert!(err.starts_with("board_pins: "), "{err}");
        assert!(err.contains("Alias A mapped to PA9"), "{err}");
    }

    #[test]
    fn test_a_missing_aliases_option_is_allowed() {
        let printer = printer_with(&["mcu"]);
        let object = BoardPins::new(&section(None, &[]), &printer).unwrap();

        assert_eq!(object.aliases(), 0);
        assert_eq!(object.reserved(), 0);
    }

    #[test]
    fn test_the_object_is_not_queryable() {
        let printer = printer_with(&["mcu"]);
        let object = BoardPins::new(&section(None, &[]), &printer).unwrap();

        assert_eq!(object.get_status(0.0), json!({}));
        assert!(!object.is_queryable());
    }

    #[test]
    fn test_the_loader_entry_points_both_build_the_object() {
        let printer = printer_with(&["mcu"]);
        let bare = section(None, &[("aliases", "A=PA0")]);
        let prefixed = section(Some("second"), &[("aliases", "B=PA1")]);

        load_config(&bare, &printer).unwrap();
        load_config_prefix(&prefixed, &printer).unwrap();

        let pins = pins(&printer);
        assert_eq!(pins.resolve_pin("mcu", "A").unwrap(), "PA0");
        assert_eq!(pins.resolve_pin("mcu", "B").unwrap(), "PA1");
    }
}
