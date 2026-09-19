//! Config-driven object loading: section id → factory.
//!
//! Upstream's `Printer._read_config` (`klippy/klippy.py:111`) turns the parsed
//! config into printer objects: it asks the modules that must exist before
//! anything else for their objects, then walks every prefix section, then
//! validates that no section was left unused (`klippy/configfile.py:425`). This
//! module is that step, with upstream's dynamic module lookup replaced by a
//! static table.
//!
//! The table **is** the schema: a section id is valid exactly when some factory
//! claims it. `[mcu]` is claimed by a `load_config`, `[mcu zboard]` by a
//! `load_config_prefix` — the same two entry points upstream looks up on the
//! module (`klippy/klippy.py:90-99`).
//!
//! The step itself is [`Printer::load_config`], defined here rather than in
//! `printer.rs` so that the machine's core does not import its parts — the same
//! split as `Mcu::connect` living in `identify.rs`.
//!
//! # Order
//!
//! Main sections first, in table order, then prefix sections, in table order.
//! Upstream loads its up-front modules the same way (`mcu` before the generic
//! prefix walk), and the order matters: an object may look up one an earlier
//! entry registered.
//!
//! # What is not here
//!
//! Option-level validation. Upstream records every option each object reads and
//! rejects anything unread (`klippy/configfile.py:435-441`); that needs access
//! tracking in [`ConfigSection`], which does not exist yet. Only whole sections
//! are validated so far.

use std::sync::Arc;

use crate::core::klippy::config::{Config, ConfigSection};
use crate::core::klippy::error::KlippyError;
use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
use crate::core::klippy::mcu::{load_config as load_mcu, load_config_prefix as load_mcu_prefix};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// Builds one printer object from a config section.
///
/// The loader registers what a factory returns under the section's identifier
/// (`mcu`, `mcu zboard`), so a factory never names its own object, and two
/// sections cannot silently claim one name. The printer is passed because an
/// object may wire itself up as it is built.
///
/// `Err` is the factory's own complaint about the section, reported by the
/// loader as a config error. (A config-error type is still missing — see the
/// `TODO`.)
pub type LoadConfig = fn(&ConfigSection, &Arc<Printer>) -> Result<Arc<dyn PrinterObject>, String>;

/// One section id's two entry points, as upstream's `load_config` and
/// `load_config_prefix`.
#[derive(Clone, Copy)]
pub struct Factories {
    /// Builds the bare `[<id>]` section.
    pub load_config: Option<LoadConfig>,
    /// Builds each `[<id> <name>]` section.
    pub load_config_prefix: Option<LoadConfig>,
}

/// Every section id this host knows, in load order.
const FACTORIES: &[(&str, Factories)] = &[(
    "mcu",
    Factories {
        load_config: Some(load_mcu),
        load_config_prefix: Some(load_mcu_prefix),
    },
)];

impl Printer {
    /// Load every printer object the config describes into this machine.
    ///
    /// The receiver is `&Arc<Self>` rather than `&self` because a factory is
    /// handed the shared handle: an object may hold on to the machine it belongs
    /// to — to register handlers, or to look another object up as it is built.
    /// Defining this next to the table keeps the machine's core free of its
    /// parts.
    ///
    /// The printer is expected to be freshly built: loading twice would trip the
    /// duplicate-name check, which is the intent — a name is registered once.
    ///
    /// # Order
    ///
    /// This is not the first thing that happens to a fresh machine. The API
    /// server's own object (`webhooks`) is registered *before* the config is
    /// loaded, so that `objects/list` starts with it as upstream's does (see
    /// [`api::register`](crate::core::klippy::api::register)); a host that loads
    /// first and registers it afterwards reorders that list.
    ///
    /// [`PrinterPins`](crate::core::klippy::pins::PrinterPins) is registered
    /// next, unconditionally, because every resource and `[board_pins]` reaches
    /// it while sections are being loaded — and the MCU objects register
    /// themselves as chips as they are built. Upstream loads the same two
    /// modules up front (`pins` then `mcu`, `klippy/klippy.py:118-119`).
    /// `pins` is registered but never queryable, so `objects/list` still starts
    /// with `webhooks`.
    ///
    /// [`GCodeDispatch`](crate::core::klippy::gcode::GCodeDispatch) comes first
    /// of all, because `pins` sections and resources register commands with it
    /// as they are built. Upstream registers the same object in
    /// `Printer.__init__` (`klippy/klippy.py:36-40`), before the config is read.
    ///
    /// # Errors
    /// Returns [`KlippyError::Internal`] if a factory rejects a section, if a
    /// name is already taken, or if a section nothing claims is left over — the
    /// last is upstream's `Section '%s' is not a valid config section`
    /// (`klippy/configfile.py:431`).
    pub fn load_config(self: &Arc<Self>, config: &Config) -> Result<(), KlippyError> {
        self.add_object(GCODE_OBJECT, Arc::new(GCodeDispatch::new(Arc::clone(self))))?;
        self.add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))?;

        let mut claimed: Vec<String> = Vec::new();

        for (id, factories) in FACTORIES {
            let Some(load) = factories.load_config else {
                continue;
            };
            let Some(section) = config.get_section(id) else {
                continue;
            };
            register(load, section, self, &mut claimed)?;
        }

        for (id, factories) in FACTORIES {
            let Some(load) = factories.load_config_prefix else {
                continue;
            };
            for section in config.get_sections_by_id(id) {
                // The bare `[id]` is the main section above; only `[id <name>]`
                // is a prefix section, which is what upstream's
                // `get_prefix_sections` returns.
                if section.sub.is_none() {
                    continue;
                }
                register(load, section, self, &mut claimed)?;
            }
        }

        for section in config.sections_vec() {
            let identifier = section.identifier();
            if !claimed.iter().any(|name| name == &identifier) {
                return Err(KlippyError::Internal(format!(
                    "Section '{identifier}' is not a valid config section"
                )));
            }
        }

        Ok(())
    }
}

/// Build and register one section, recording it as claimed.
fn register(
    load: LoadConfig,
    section: &ConfigSection,
    printer: &Arc<Printer>,
    claimed: &mut Vec<String>,
) -> Result<(), KlippyError> {
    let object = load(section, printer).map_err(KlippyError::Internal)?;
    let identifier = section.identifier();
    printer.add_object(&identifier, object)?;
    claimed.push(identifier);
    Ok(())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::reactor::ManualReactor;

    /// Parse a config from its text, as the host does from a file.
    fn config(text: &str) -> Config {
        Config::from_str(text).expect("the test config parses").0
    }

    fn load(text: &str) -> (Arc<Printer>, Result<(), KlippyError>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let result = printer.load_config(&config(text));
        (printer, result)
    }

    #[test]
    fn test_the_main_mcu_section_becomes_the_mcu_object() {
        let (printer, result) = load("[mcu]\nserial: /dev/not-opened-yet\n");

        result.unwrap();
        // `pins` is registered before the table (upstream loads `pins` and
        // `mcu` up front), then the section's own object.
        assert_eq!(printer.objects(), ["gcode", "pins", "mcu"]);
    }

    #[test]
    fn test_prefix_sections_become_objects_of_their_own() {
        let (printer, result) = load(
            "[mcu]\nserial: /dev/a\n\
             [mcu zboard]\nserial: /dev/b\n\
             [mcu toolhead]\nserial: /dev/c\n",
        );

        result.unwrap();
        // The main section first, then the prefix sections in config order —
        // upstream's `add_printer_objects` order (`klippy/mcu.py:1239-1246`).
        assert_eq!(
            printer.objects(),
            ["gcode", "pins", "mcu", "mcu zboard", "mcu toolhead"]
        );
    }

    #[test]
    fn test_an_unknown_section_is_rejected_the_way_upstream_rejects_it() {
        let (_printer, result) = load("[mcu]\nserial: /dev/a\n[made_up]\n");

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("Section 'made_up' is not a valid config section"),
            "{err}"
        );
    }

    #[test]
    fn test_a_config_with_no_objects_loads_only_the_builtins() {
        let (printer, result) = load("");

        result.unwrap();
        // `gcode` and `pins` are unconditional; no section contributed anything else.
        assert_eq!(printer.objects(), ["gcode", "pins"]);
    }

    #[test]
    fn test_a_bad_interface_is_not_noticed_until_connect() {
        // Two-phase construction: loading only builds the object. The section is
        // parsed — and the device opened — by `McuObject::connect`, so a config
        // that cannot connect still *loads*, and the error is reported when the
        // printer comes up rather than when the file is read.
        let (printer, result) = load("[mcu]\nserial: /dev/not-a-serial-port\n");

        result.unwrap();
        assert_eq!(printer.objects(), ["gcode", "pins", "mcu"]);
    }
}
