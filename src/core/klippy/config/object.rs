//! The `configfile` printer object — the config as a client sees it.
//!
//! Upstream registers `PrinterConfig` before the config is loaded
//! (`klippy/klippy.py:115`) and it reports:
//!
//! | field | upstream | here |
//! |---|---|---|
//! | `config` | every section/option as written (`status_raw_config`) | the snapshot [`PrinterConfig::new`] was handed |
//! | `warnings` | deprecated options, runtime warnings | always empty (no `deprecate` yet) |
//! | `settings` | every option a module read, parsed (`ConfigValidate`) | the live [`AccessTracking`] |
//! | `save_config_pending` / `_items` | `SAVE_CONFIG` state | always empty (auto-save is a separate task) |
//!
//! `settings` is read live rather than snapshotted because parts read their
//! sections as they connect; `config` is a snapshot because it never changes.

use std::sync::Arc;

use serde_json::{json, Map, Value};

use crate::core::klippy::config::access::AccessTracking;
use crate::core::klippy::config::Config;
use crate::core::klippy::printer::PrinterObject;

/// The name other modules use to find the config object.
pub const CONFIGFILE_OBJECT: &str = "configfile";

/// The `configfile` object: what the config file said, and what was read of it.
pub struct PrinterConfig {
    /// The reads recorded during this config load.
    access: Arc<AccessTracking>,
    /// Every section and option as written, for the `config` status.
    raw_config: Map<String, Value>,
}

impl PrinterConfig {
    /// Build the object over the reads of one config load.
    pub fn new(access: Arc<AccessTracking>, raw_config: Map<String, Value>) -> Self {
        Self { access, raw_config }
    }

    /// Build the `config` status snapshot from a parsed config.
    ///
    /// Every option, read with no access tracking, as upstream's
    /// `_build_status_config` does (`klippy/configfile.py:544-549`): the
    /// snapshot is data about the file, not something a module consumed.
    pub fn raw_config(config: &Config) -> Map<String, Value> {
        let mut snapshot = Map::new();
        for section in config.sections() {
            let mut options = Map::new();
            for (option, value) in &section.parameters {
                options.insert(option.clone(), json!(value.as_str()));
            }
            snapshot.insert(section.identifier(), Value::Object(options));
        }
        snapshot
    }
}

impl PrinterObject for PrinterConfig {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({
            "config": self.raw_config,
            "warnings": Vec::<Value>::new(),
            "settings": self.access.settings(),
            "save_config_pending": false,
            "save_config_pending_items": Map::<String, Value>::new(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_status_shape_matches_upstream() {
        let (config, _) = Config::from_text(
            "[mcu]\nserial: /dev/a\n\
             [output_pin fan]\npin: PA0\n",
        )
        .unwrap();
        let object =
            PrinterConfig::new(AccessTracking::shared(), PrinterConfig::raw_config(&config));
        object.access.note("output_pin fan", "pin", json!("PA0"));

        let status = object.get_status(0.0);
        assert_eq!(status["config"]["mcu"]["serial"], json!("/dev/a"));
        assert_eq!(status["config"]["output_pin fan"]["pin"], json!("PA0"));
        assert_eq!(status["settings"]["output_pin fan"]["pin"], json!("PA0"));
        assert_eq!(status["warnings"], json!([]));
        assert_eq!(status["save_config_pending"], json!(false));
    }
}
