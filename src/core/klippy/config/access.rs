//! What a config section read, recorded as it is read.
//!
//! Upstream's `ConfigWrapper` keeps an `access_tracking` dict shared by every
//! wrapper and records `(section, option) -> value` on each get
//! (`klippy/configfile.py:29-64`). That dict is the schema: after the config is
//! loaded, `ConfigValidate.check_unused` rejects any option nobody read
//! (`klippy/configfile.py:424-446`), and `_build_status_settings` turns it into
//! the `configfile` object's `settings` status (`klippy/configfile.py:447-452`).
//!
//! Keys are lowercased, section and option both, so a config that writes `[MCU]`
//! and one that writes `[mcu]` validate the same way. The parser in this crate
//! preserves case (unlike `configparser`), so the lowercasing happens here.
//!
//! The value stored is the *parsed* value, as a JSON value, because that is
//! what upstream records and what `settings` reports: a float for `getfloat`, a
//! string for `get`, a list for `getlist`.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Mutex;

use serde_json::{Map, Value};

/// The reads recorded for one config load.
///
/// Shared (`Arc`) between the loader, every [`ConfigWrapper`] and the
/// `configfile` object, so a part that reads its section during connect is
/// recorded alongside the parts that read during the load walk.
///
/// [`ConfigWrapper`]: super::wrapper::ConfigWrapper
#[derive(Debug, Default)]
pub struct AccessTracking {
    /// `(section, option) -> parsed value`, both names lowercased.
    reads: Mutex<BTreeMap<(String, String), Value>>,
}

impl AccessTracking {
    /// An empty record.
    pub fn new() -> Self {
        Self::default()
    }

    /// An empty record behind a new `Arc`, ready to be shared.
    pub fn shared() -> std::sync::Arc<Self> {
        std::sync::Arc::new(Self::new())
    }

    /// Record that `option` of `section` was read, with its parsed `value`.
    pub fn note(&self, section: &str, option: &str, value: Value) {
        let key = (section.to_lowercase(), option.to_lowercase());
        self.lock().insert(key, value);
    }

    /// Whether `(section, option)` was read, comparing case-insensitively.
    pub fn contains(&self, section: &str, option: &str) -> bool {
        let key = (section.to_lowercase(), option.to_lowercase());
        self.lock().contains_key(&key)
    }

    /// Every section that had at least one option read, lowercased.
    pub fn sections(&self) -> BTreeSet<String> {
        self.lock()
            .keys()
            .map(|(section, _)| section.clone())
            .collect()
    }

    /// The record as the `configfile` object's `settings` status:
    /// `{section: {option: value}}` with lowercased names, as upstream's
    /// `_build_status_settings` builds it (`klippy/configfile.py:447-449`).
    pub fn settings(&self) -> Map<String, Value> {
        let mut settings: Map<String, Value> = Map::new();
        for ((section, option), value) in self.lock().iter() {
            let fields = settings
                .entry(section.clone())
                .or_insert_with(|| Value::Object(Map::new()));
            if let Value::Object(fields) = fields {
                fields.insert(option.clone(), value.clone());
            }
        }
        settings
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, BTreeMap<(String, String), Value>> {
        self.reads
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn note_and_contains_lowercase_both_names() {
        let access = AccessTracking::new();
        access.note("Output_Pin Fan", "Pin", json!("PA0"));

        assert!(access.contains("output_pin fan", "pin"));
        assert!(access.contains("OUTPUT_PIN FAN", "PIN"));
        assert!(!access.contains("output_pin fan", "value"));
    }

    #[test]
    fn settings_groups_by_section() {
        let access = AccessTracking::new();
        access.note("output_pin fan", "pin", json!("PA0"));
        access.note("output_pin fan", "value", json!(0.5));
        access.note("mcu", "serial", json!("/dev/a"));

        let settings = access.settings();
        assert_eq!(settings["output_pin fan"]["pin"], json!("PA0"));
        assert_eq!(settings["output_pin fan"]["value"], json!(0.5));
        assert_eq!(settings["mcu"]["serial"], json!("/dev/a"));
    }

    #[test]
    fn sections_lists_each_section_once() {
        let access = AccessTracking::new();
        access.note("mcu", "serial", json!("/dev/a"));
        access.note("mcu", "baud", json!(250000));
        access.note("output_pin fan", "pin", json!("PA0"));

        let sections: Vec<String> = access.sections().into_iter().collect();
        assert_eq!(sections, ["mcu", "output_pin fan"]);
    }
}
