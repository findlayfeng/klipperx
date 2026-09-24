//! `[exclude_object]` — the object-exclusion state machine
//! (upstream `klippy/extras/exclude_object.py`).
//!
//! Upstream's `ExcludeObject` reads **no options** — it subscribes to event
//! handlers, registers four commands (`EXCLUDE_OBJECT_START` / `_END` /
//! `EXCLUDE_OBJECT` / `EXCLUDE_OBJECT_DEFINE`) and installs a `gcode_move`
//! transform on demand (`exclude_object.py:16-46`).
//!
//! This port lands the **section** now so the corpus's `exclude_object.test`
//! loads: the object is registered under its section name and exposes the
//! upstream `get_status` shape (`exclude_object.py:174-181`), so the
//! `[gcode_macro M486]` body's `printer.exclude_object…` lookups resolve. The
//! command side and the move transform — the actual object tracking and
//! extrusion-offset bookkeeping (`exclude_object.py:60-172`) — is **not
//! implemented yet**; that belongs with the rest of the H4 work. Unregistered
//! commands are reported as unknown and pass through (`gcode.rs`, "Unknown
//! command" respond), which is what lets the corpus case run green at the
//! section-level acceptance tier.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("exclude_object", order = 30, load = load_config);

/// The `[exclude_object]` section (`exclude_object.py:ExcludeObject`).
#[derive(Debug, Default)]
pub struct ExcludeObject {
    /// Defined objects, sorted by name upstream
    /// (`exclude_object.py:_add_object_definition`).
    objects: Vec<Value>,
    /// Excluded object names, sorted (`exclude_object.py:_exclude_object`).
    excluded_objects: Vec<String>,
    /// The name of the object currently being printed, if any
    /// (`exclude_object.py:cmd_EXCLUDE_OBJECT_START`).
    current_object: Option<String>,
}

impl ExcludeObject {
    /// Read the section — upstream reads no options
    /// (`exclude_object.py:16-46`), so the bare `[exclude_object]` in the
    /// corpus's `exclude_object.cfg:70` needs no access tracking beyond the
    /// factory claiming the section.
    pub fn new(_config: &ConfigWrapper) -> Result<Self, ConfigError> {
        Ok(Self::default())
    }

    /// The empty state upstream's `_reset_state` starts from
    /// (`exclude_object.py:70-75`).
    pub fn new_reset() -> Self {
        Self::default()
    }
}

impl PrinterObject for ExcludeObject {
    /// Upstream's `get_status`: `objects`, `excluded_objects`,
    /// `current_object` (`exclude_object.py:174-181`).
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({
            "objects": self.objects,
            "excluded_objects": self.excluded_objects,
            "current_object": self.current_object,
        })
    }
}

/// The factory `section!` names (`exclude_object.py:303`).
pub fn load_config(
    config: &ConfigWrapper,
    _printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(ExcludeObject::new(config)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, ConfigSection};

    /// A `[exclude_object]` section with the given options, as the parser
    /// would build it.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("exclude_object", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                crate::core::klippy::config::ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A wrapper that records into `access`, as the loader builds it.
    fn wrapper<'a>(section: &'a ConfigSection, access: &Arc<AccessTracking>) -> ConfigWrapper<'a> {
        ConfigWrapper::new(section, Arc::clone(access))
    }

    /// Upstream reads no options (`exclude_object.py:16-46`): the bare
    /// `[exclude_object]` in the corpus's `exclude_object.cfg:70` loads, and
    /// the factory claiming the section is what `check_unused` needs
    /// (`config/validate.rs:26-33`).
    #[test]
    fn a_bare_section_loads_with_the_reset_state() {
        let sect = section(&[]);
        let access = AccessTracking::shared();
        let exclude =
            ExcludeObject::new(&wrapper(&sect, &access)).expect("the empty section loads");
        assert_eq!(
            exclude.get_status(0.0),
            json!({
                "objects": [],
                "excluded_objects": [],
                "current_object": serde_json::Value::Null,
            })
        );
    }

    /// The reset state matches upstream's `_reset_state`
    /// (`exclude_object.py:70-75`): no objects, none excluded, no current
    /// object.
    #[test]
    fn new_reset_matches_upstreams_reset_state() {
        let exclude = ExcludeObject::new_reset();
        let status = exclude.get_status(0.0);
        assert_eq!(status["objects"], json!([]));
        assert_eq!(status["excluded_objects"], json!([]));
        assert!(status["current_object"].is_null());
    }

    /// The status keys are exactly upstream's three (`exclude_object.py:174-181`)
    /// — the M486 macro reads `printer.exclude_object.current_object`.
    #[test]
    fn get_status_exposes_exactly_the_upstream_keys() {
        let exclude = ExcludeObject::new_reset();
        let status = exclude.get_status(0.0);
        let mut keys: Vec<&str> = status
            .as_object()
            .unwrap()
            .keys()
            .map(|k| k.as_str())
            .collect();
        keys.sort_unstable();
        assert_eq!(keys, ["current_object", "excluded_objects", "objects"]);
    }

    /// The section loads through the ordinary loader: it is claimed by its
    /// factory and the object lands under the section's own name.
    #[test]
    fn the_section_loads_through_the_loader() {
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let (config, _) = crate::core::klippy::config::Config::from_text("[exclude_object]\n")
            .expect("the section parses");
        printer.load_config(&config).expect("the section loads");

        let object = printer
            .lookup_object("exclude_object")
            .expect("the object is registered");
        assert_eq!(
            object.get_status(0.0),
            json!({
                "objects": [],
                "excluded_objects": [],
                "current_object": serde_json::Value::Null,
            })
        );
    }
}
