//! `[gcode_arcs]` — arc support for the `G2`/`G3` commands
//! (upstream `klippy/extras/gcode_arcs.py`).
//!
//! Upstream's `ArcSupport` reads one option and registers five commands:
//!
//! | option | default | role |
//! |---|---|---|
//! | `resolution` | `1.` | millimetres per arc segment (`above=0.0`) |
//!
//! This port lands the **section** now so the corpus's `gcode_arcs.test` loads:
//! the option is read with upstream's default and bound, and the object is
//! registered so `check_unused` accepts the section. The command side —
//! `G2`/`G3` (arc planning through `planArc`) and the `G17`/`G18`/`G19` plane
//! selectors — is **not implemented yet**; that belongs with the rest of the
//! H10 motion extras. Unregistered commands are reported as unknown and pass
//! through (`gcode.rs`, "Unknown command" respond), which is what lets the
//! corpus case run green at the section-level acceptance tier.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("gcode_arcs", order = 30, load = load_config);

/// The `[gcode_arcs]` section (`gcode_arcs.py:ArcSupport`).
#[derive(Debug)]
pub struct GCodeArcs {
    /// `resolution`: millimetres per arc segment — upstream's
    /// `mm_per_arc_segment` (`gcode_arcs.py:19`), kept for the `G2`/`G3`
    /// implementation that is still pending.
    mm_per_arc_segment: f64,
}

impl GCodeArcs {
    /// Read the section.
    ///
    /// # Errors
    /// An unparseable `resolution`, or one at or below `0.` — upstream's
    /// `config.getfloat('resolution', 1., above=0.0)`
    /// (`gcode_arcs.py:19`) with the shared wording.
    pub fn new(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let mm_per_arc_segment =
            config.get_float_bounded("resolution", Some(1.), None, None, Some(0.0), None)?;
        Ok(Self { mm_per_arc_segment })
    }

    /// The configured millimetres per arc segment.
    pub fn resolution(&self) -> f64 {
        self.mm_per_arc_segment
    }
}

impl PrinterObject for GCodeArcs {
    /// Upstream's `ArcSupport` defines no `get_status`.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }
}

/// The factory `section!` names (`gcode_arcs.py:load_config`).
pub fn load_config(
    config: &ConfigWrapper,
    _printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(GCodeArcs::new(config)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, ConfigSection, ConfigValue};
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[gcode_arcs]` section with the given options, as the parser would
    /// build it.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("gcode_arcs", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A wrapper that records into `access`, as the loader builds it.
    fn wrapper<'a>(section: &'a ConfigSection, access: &Arc<AccessTracking>) -> ConfigWrapper<'a> {
        ConfigWrapper::new(section, Arc::clone(access))
    }

    /// The default `1.` is used and recorded when `resolution` is absent, so
    /// the access check accepts a bare `[gcode_arcs]` (`config/validate.rs`)
    /// — the corpus's own `gcode_arcs.cfg` writes no options at all
    /// (`gcode_arcs.py:19`).
    #[test]
    fn a_missing_resolution_defaults_to_one_and_is_recorded() {
        let sect = section(&[]);
        let access = AccessTracking::shared();
        let arcs = GCodeArcs::new(&wrapper(&sect, &access)).expect("the empty section loads");
        assert_eq!(arcs.resolution(), 1.0);
        assert!(access.contains("gcode_arcs", "resolution"));
    }

    /// A written `resolution` is parsed and recorded as the number
    /// (`gcode_arcs.py:19`).
    #[test]
    fn a_written_resolution_is_parsed_and_recorded() {
        let sect = section(&[("resolution", "0.5")]);
        let access = AccessTracking::shared();
        let arcs = GCodeArcs::new(&wrapper(&sect, &access)).expect("0.5 parses");
        assert_eq!(arcs.resolution(), 0.5);
        assert!(access.contains("gcode_arcs", "resolution"));
    }

    /// `above=0.0` refuses `0.` and negatives with the shared bound wording,
    /// and a non-number keeps the parser's wording (`gcode_arcs.py:19`,
    /// `configfile.py:44/55`).
    #[test]
    fn a_resolution_at_or_below_zero_and_a_non_number_are_refused() {
        let sect = section(&[("resolution", "0")]);
        let access = AccessTracking::shared();
        assert_eq!(
            GCodeArcs::new(&wrapper(&sect, &access))
                .unwrap_err()
                .to_string(),
            "Option 'resolution' in section 'gcode_arcs' must be above 0"
        );

        let sect = section(&[("resolution", "-1")]);
        let access = AccessTracking::shared();
        assert_eq!(
            GCodeArcs::new(&wrapper(&sect, &access))
                .unwrap_err()
                .to_string(),
            "Option 'resolution' in section 'gcode_arcs' must be above 0"
        );

        let sect = section(&[("resolution", "abc")]);
        let access = AccessTracking::shared();
        assert_eq!(
            GCodeArcs::new(&wrapper(&sect, &access))
                .unwrap_err()
                .to_string(),
            "Unable to parse option 'resolution' in section 'gcode_arcs'"
        );
    }

    /// The section loads through the ordinary loader: it is claimed by its
    /// factory, the default `resolution` is recorded, and the object lands
    /// under the section's own name.
    #[test]
    fn the_section_loads_through_the_loader() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = crate::core::klippy::config::Config::from_text("[gcode_arcs]\n")
            .expect("the section parses");
        printer.load_config(&config).expect("the section loads");

        assert_eq!(
            printer.objects(),
            ["gcode", "configfile", "pins", "gcode_arcs"]
        );
        let object = printer
            .lookup_object("gcode_arcs")
            .expect("the object is registered");
        assert_eq!(object.get_status(0.0), json!({}));
    }
}
