//! The `configfile` printer object — the config as a client sees it.
//!
//! Upstream registers `PrinterConfig` before the config is loaded
//! (`klippy/klippy.py:115`) and it reports:
//!
//! | field | upstream | here |
//! |---|---|---|
//! | `config` | every section/option as written (`status_raw_config`) | the snapshot [`PrinterConfig::new`] was handed |
//! | `warnings` | deprecated options, runtime warnings | the recorded warnings, deduplicated as upstream does |
//! | `settings` | every option a module read, parsed (`ConfigValidate`) | the live [`AccessTracking`] |
//! | `save_config_pending` / `_items` | `SAVE_CONFIG` state | the pending autosave values (`set` / `remove_section`); writing the file back is still to come |
//!
//! `settings` is read live rather than snapshotted because parts read their
//! sections as they connect; `config` is a snapshot because it never changes.

use std::collections::HashSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use serde_json::{json, Map, Value};
use tracing::{info, warn};

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
    /// Deprecation and runtime warnings, in first-seen order.
    warnings: Mutex<Vec<Value>>,
    /// Serialized keys of [`PrinterConfig::warnings`], for upstream's dedup
    /// (`_add_deprecated`, `klippy/configfile.py:491-499`).
    seen: Mutex<HashSet<String>>,
    /// The autosave values waiting for `SAVE_CONFIG`
    /// (`ConfigAutoSave.status_save_pending`): a section maps to its pending
    /// options, or to `Null` when the section is to be removed.
    pending: Mutex<Map<String, Value>>,
    /// Whether anything is waiting to be written back.
    save_pending: AtomicBool,
}

impl PrinterConfig {
    /// Build the object over the reads of one config load.
    pub fn new(access: Arc<AccessTracking>, raw_config: Map<String, Value>) -> Self {
        Self {
            access,
            raw_config,
            warnings: Mutex::new(Vec::new()),
            seen: Mutex::new(HashSet::new()),
            pending: Mutex::new(Map::new()),
            save_pending: AtomicBool::new(false),
        }
    }

    /// Record an autosave value (`ConfigAutoSave.set`,
    /// `klippy/configfile.py:317-330`).
    ///
    /// The value is only remembered: upstream writes it back at `SAVE_CONFIG`.
    pub fn set(&self, section: &str, option: &str, value: &str) {
        {
            let mut pending = self.pending.lock().unwrap_or_else(|p| p.into_inner());
            let entry = pending
                .entry(section.to_string())
                .or_insert_with(|| Value::Object(Map::new()));
            if !entry.is_object() {
                *entry = Value::Object(Map::new());
            }
            if let Value::Object(options) = entry {
                options.insert(option.to_string(), json!(value));
            }
        }
        self.save_pending.store(true, Ordering::SeqCst);
        info!("save_config: set [{section}] {option} = {value}");
    }

    /// Drop a section at the next `SAVE_CONFIG` (`ConfigAutoSave.remove_section`,
    /// `klippy/configfile.py:331-343`).
    pub fn remove_section(&self, section: &str) {
        self.pending
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(section.to_string(), Value::Null);
        self.save_pending.store(true, Ordering::SeqCst);
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

    /// Whether `option` was written in `section`.
    ///
    /// Upstream's `fileconfig.has_option(section, option)`: option names are
    /// compared without case because `configparser` lowercases them.
    pub fn has_option(&self, section: &str, option: &str) -> bool {
        let Some(options) = self.raw_config.get(section).and_then(Value::as_object) else {
            return false;
        };
        options.keys().any(|key| key.eq_ignore_ascii_case(option))
    }

    /// Append a warning, unless the same one was already recorded.
    ///
    /// Returns whether it was added, like upstream's `_add_deprecated`
    /// (`klippy/configfile.py:491`).
    fn add_warning(&self, warning: Value) -> bool {
        let key = warning.to_string();
        let mut seen = self.seen.lock().unwrap_or_else(|p| p.into_inner());
        if !seen.insert(key) {
            return false;
        }
        self.warnings
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push(warning);
        true
    }

    /// Record a deprecated option or value, when it was written.
    ///
    /// Upstream `PrinterConfig.deprecate` (`klippy/configfile.py:503`).
    pub fn deprecate(
        &self,
        section: &str,
        option: &str,
        value: Option<Value>,
        message: Option<&str>,
    ) {
        let mut warning = Map::new();
        match &value {
            None => {
                warning.insert("type".into(), json!("deprecated_option"));
                let default = format!("Option '{option}' in section '{section}' is deprecated.");
                warning.insert("message".into(), json!(message.unwrap_or(&default)));
            }
            Some(value) => {
                warning.insert("type".into(), json!("deprecated_value"));
                warning.insert("value".into(), value.clone());
                let default = format!(
                    "Value '{value}' in option '{option}' in section '{section}' is deprecated."
                );
                warning.insert("message".into(), json!(message.unwrap_or(&default)));
            }
        }
        warning.insert("section".into(), json!(section));
        warning.insert("option".into(), json!(option));
        self.add_warning(Value::Object(warning));
    }

    /// Record a deprecated g-code command, parameter, or value.
    ///
    /// Upstream `PrinterConfig.deprecate_gcode` (`klippy/configfile.py:518`).
    pub fn deprecate_gcode(
        &self,
        command: &str,
        parameter: Option<&str>,
        value: Option<&str>,
        message: Option<&str>,
    ) {
        let default = match (parameter, value) {
            (None, _) => format!("Command '{command}' is deprecated."),
            (Some(parameter), None) => {
                format!("Parameter '{parameter}' in command '{command}' is deprecated.")
            }
            (Some(parameter), Some(value)) => {
                format!("Value '{parameter}={value}' in command '{command}' is deprecated.")
            }
        };
        self.add_warning(json!({
            "type": "deprecated_gcode",
            "message": message.unwrap_or(&default),
            "command": command,
            "parameter": parameter,
            "value": value,
        }));
    }

    /// Record that an MCU is missing a feature the host now expects.
    ///
    /// Upstream `PrinterConfig.deprecate_mcu_code` (`klippy/configfile.py:532`);
    /// the versions are passed in rather than taken from an MCU object, so this
    /// stays free of the transport layer.
    pub fn deprecate_mcu_code(
        &self,
        mcu: &str,
        mcu_version: &str,
        host_version: &str,
        feature: &str,
        message: Option<&str>,
    ) {
        let default = format!(
            "MCU '{mcu}' has deprecated code (it is missing feature '{feature}'). \
             Recompiling and flashing is recommended (MCU version '{mcu_version}', \
             host version '{host_version}')."
        );
        self.add_warning(json!({
            "type": "deprecated_mcu_code",
            "message": message.unwrap_or(&default),
            "mcu": mcu,
            "feature": feature,
        }));
    }

    /// Record a runtime warning, deduplicated like a deprecation.
    ///
    /// Upstream `PrinterConfig.runtime_warning` (`klippy/configfile.py:497`):
    /// the warning is also logged, once.
    pub fn runtime_warning(&self, message: &str) {
        let warning = json!({"type": "runtime_warning", "message": message});
        if self.add_warning(warning) {
            warn!("{message}");
        }
    }
}

impl PrinterObject for PrinterConfig {
    fn get_status(&self, _eventtime: f64) -> Value {
        let warnings = self
            .warnings
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        json!({
            "config": self.raw_config,
            "warnings": warnings,
            "settings": self.access.settings(),
            "save_config_pending": self.save_pending.load(Ordering::SeqCst),
            "save_config_pending_items": self
                .pending
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .clone(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn object(text: &str) -> PrinterConfig {
        let (config, _) = Config::from_text(text).unwrap();
        PrinterConfig::new(AccessTracking::shared(), PrinterConfig::raw_config(&config))
    }

    #[test]
    fn set_and_remove_section_record_pending_autosave_values() {
        let object = object("[probe]\nz_offset: 1.0\n");
        assert_eq!(object.get_status(0.0)["save_config_pending"], json!(false));

        object.set("probe", "z_offset", "2.000");
        object.set("probe", "x_offset", "1.000");
        object.remove_section("gone");

        let status = object.get_status(0.0);
        assert_eq!(status["save_config_pending"], json!(true));
        assert_eq!(
            status["save_config_pending_items"]["probe"]["z_offset"],
            json!("2.000")
        );
        assert_eq!(
            status["save_config_pending_items"]["probe"]["x_offset"],
            json!("1.000")
        );
        assert_eq!(status["save_config_pending_items"]["gone"], Value::Null);
    }

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

    #[test]
    fn a_deprecated_option_keeps_upstreams_shape() {
        let object = object("[output_pin fan]\npin: PA0\n");
        object.deprecate("output_pin fan", "pin", None, None);

        assert_eq!(
            object.get_status(0.0)["warnings"],
            json!([{
                "type": "deprecated_option",
                "message": "Option 'pin' in section 'output_pin fan' is deprecated.",
                "section": "output_pin fan",
                "option": "pin",
            }])
        );
    }

    #[test]
    fn a_deprecated_value_names_the_value() {
        let object = object("[probe_eddy_current eddy]\nz_offset: 1\n");
        object.deprecate("probe_eddy_current eddy", "z_offset", Some(json!(1)), None);

        let warnings = object.get_status(0.0)["warnings"].clone();
        assert_eq!(warnings[0]["type"], json!("deprecated_value"));
        assert_eq!(warnings[0]["value"], json!(1));
        assert_eq!(
            warnings[0]["message"],
            json!("Value '1' in option 'z_offset' in section 'probe_eddy_current eddy' is deprecated.")
        );
    }

    #[test]
    fn a_warning_is_only_recorded_once() {
        let object = object("[output_pin fan]\npin: PA0\n");
        object.deprecate("output_pin fan", "pin", None, None);
        object.deprecate("output_pin fan", "pin", None, None);

        assert_eq!(
            object.get_status(0.0)["warnings"].as_array().unwrap().len(),
            1
        );
    }

    #[test]
    fn a_custom_message_replaces_the_default() {
        let object = object("[s]\nkey: 1\n");
        object.deprecate("s", "key", None, Some("use `other` instead"));

        assert_eq!(
            object.get_status(0.0)["warnings"][0]["message"],
            json!("use `other` instead")
        );
    }

    #[test]
    fn a_deprecated_gcode_keeps_upstreams_shape() {
        let object = object("[s]\nkey: 1\n");
        object.deprecate_gcode("MANUAL_STEPPER", Some("STOP_ON_ENDSTOP"), Some("1"), None);

        let warning = object.get_status(0.0)["warnings"][0].clone();
        assert_eq!(warning["type"], json!("deprecated_gcode"));
        assert_eq!(warning["command"], json!("MANUAL_STEPPER"));
        assert_eq!(warning["parameter"], json!("STOP_ON_ENDSTOP"));
        assert_eq!(warning["value"], json!("1"));
        assert_eq!(
            warning["message"],
            json!("Value 'STOP_ON_ENDSTOP=1' in command 'MANUAL_STEPPER' is deprecated.")
        );
    }

    #[test]
    fn a_deprecated_mcu_code_names_both_versions() {
        let object = object("[s]\nkey: 1\n");
        object.deprecate_mcu_code("mcu", "v0.12", "v0.13", "STEPPER_STEP_BOTH_EDGE", None);

        let warning = object.get_status(0.0)["warnings"][0].clone();
        assert_eq!(warning["type"], json!("deprecated_mcu_code"));
        assert_eq!(warning["mcu"], json!("mcu"));
        assert_eq!(warning["feature"], json!("STEPPER_STEP_BOTH_EDGE"));
        assert_eq!(
            warning["message"],
            json!(
                "MCU 'mcu' has deprecated code (it is missing feature \
                 'STEPPER_STEP_BOTH_EDGE'). Recompiling and flashing is recommended \
                 (MCU version 'v0.12', host version 'v0.13')."
            )
        );
    }

    #[test]
    fn a_runtime_warning_is_deduplicated() {
        let object = object("[s]\nkey: 1\n");
        object.runtime_warning("temporary pin override");
        object.runtime_warning("temporary pin override");

        assert_eq!(
            object.get_status(0.0)["warnings"],
            json!([{
                "type": "runtime_warning",
                "message": "temporary pin override",
            }])
        );
    }
}
