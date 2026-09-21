//! `ConfigWrapper` — the tracked, typed view of one config section.
//!
//! Upstream hands every module a `ConfigWrapper` instead of the raw section:
//! the getters parse the value, check its range, and record the read in the
//! shared `access_tracking` dict (`klippy/configfile.py:19-95`). This is the
//! same thing: a factory receives a wrapper, not a [`ConfigSection`], so every
//! option a module reads is recorded as it is read, and the loader can reject
//! the ones nobody read.
//!
//! The section itself stays a plain value ([`ConfigSection`]); the wrapper is
//! the temporary borrow that carries the tracking handle. That way a section
//! can be cloned into an object (an MCU keeps its section for connect time)
//! without dragging a record of who read what along with it.
//!
//! Wording is upstream's, because a config error is text the user reads:
//! `Option 'x' in section 'y' must be specified`, `Unable to parse option …`.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::access::AccessTracking;
use crate::core::klippy::config::section::{split_list, ConfigSection};
use crate::core::klippy::error::ConfigError;

/// A config section, read through the tracking that validates it.
///
/// Built by the loader for each section it hands a factory
/// ([`Printer::load_config`](crate::core::klippy::printer::Printer::load_config)),
/// and again by a part that stored its section and reads it later — an MCU
/// parses `[mcu]` at connect time, on the printer's tracker, so its options are
/// recorded too.
pub struct ConfigWrapper<'a> {
    /// The section as parsed.
    section: &'a ConfigSection,
    /// Where reads are recorded (shared with the loader and `configfile`).
    access: Arc<AccessTracking>,
}

impl<'a> ConfigWrapper<'a> {
    /// Wrap `section`, recording every read in `access`.
    pub fn new(section: &'a ConfigSection, access: Arc<AccessTracking>) -> Self {
        Self { section, access }
    }

    /// Wrap `section` without recording anything.
    ///
    /// For tests and for code that holds a bare section but does not validate
    /// it. The loader and every factory use [`ConfigWrapper::new`]: an
    /// untracked read is one the option check cannot see.
    pub fn untracked(section: &'a ConfigSection) -> Self {
        Self::new(section, untracked_access())
    }

    /// The section being read.
    pub fn section(&self) -> &ConfigSection {
        self.section
    }

    /// The section's identifier, e.g. `mcu zboard` or `output_pin fan`.
    pub fn identifier(&self) -> String {
        self.section.identifier()
    }

    /// The shared access record, for a part that wants to keep reading later.
    pub fn access(&self) -> Arc<AccessTracking> {
        Arc::clone(&self.access)
    }

    // -----------------------------------------------------------------------
    // Raw string options
    // -----------------------------------------------------------------------

    /// A string option, or `None` when it is absent. A present value is
    /// recorded (as the string it was written as).
    ///
    /// This is where a module that parses an option its own way reads it; the
    /// typed getters below are preferred when the parse is a standard one,
    /// because they record the parsed value.
    pub fn get_str(&self, option: &str) -> Option<String> {
        let text = self.section.get_text(option)?;
        self.note(option, json!(text));
        Some(text)
    }

    /// Upstream's `get(option, default)`: the string, or `default`, or an error.
    ///
    /// A default that is used is recorded too, as upstream does
    /// (`klippy/configfile.py:33-36`), so `settings` reports it.
    pub fn get(&self, option: &str, default: Option<&str>) -> Result<String, ConfigError> {
        if let Some(text) = self.section.get_text(option) {
            self.note(option, json!(text));
            return Ok(text);
        }
        match default {
            Some(default) => {
                self.note(option, json!(default));
                Ok(default.to_string())
            }
            None => Err(self.must_be_specified(option)),
        }
    }

    // -----------------------------------------------------------------------
    // Typed options
    // -----------------------------------------------------------------------

    /// A float option, or `default` (which is also recorded), or an error.
    pub fn get_float(&self, option: &str, default: Option<f64>) -> Result<f64, ConfigError> {
        if let Some(value) = self.parse_float(option)? {
            return Ok(value);
        }
        match default {
            Some(default) => {
                self.note(option, json!(default));
                Ok(default)
            }
            None => Err(self.must_be_specified(option)),
        }
    }

    /// A float option as `Option`, recording only a value that was present.
    pub fn get_optional_float(&self, option: &str) -> Result<Option<f64>, ConfigError> {
        self.parse_float(option)
    }

    /// An integer option, or `default` (which is also recorded), or an error.
    pub fn get_int(&self, option: &str, default: Option<i64>) -> Result<i64, ConfigError> {
        if let Some(value) = self.parse_int(option)? {
            return Ok(value);
        }
        match default {
            Some(default) => {
                self.note(option, json!(default));
                Ok(default)
            }
            None => Err(self.must_be_specified(option)),
        }
    }

    /// An integer option as `Option`, recording only a value that was present.
    pub fn get_optional_int(&self, option: &str) -> Result<Option<i64>, ConfigError> {
        self.parse_int(option)
    }

    /// A boolean option, or `default` (which is also recorded), or an error.
    pub fn get_bool(&self, option: &str, default: Option<bool>) -> Result<bool, ConfigError> {
        if let Some(value) = self.parse_bool(option)? {
            return Ok(value);
        }
        match default {
            Some(default) => {
                self.note(option, json!(default));
                Ok(default)
            }
            None => Err(self.must_be_specified(option)),
        }
    }

    /// A boolean option as `Option`, recording only a value that was present.
    pub fn get_optional_bool(&self, option: &str) -> Result<Option<bool>, ConfigError> {
        self.parse_bool(option)
    }

    // -----------------------------------------------------------------------
    // Lists
    // -----------------------------------------------------------------------

    /// Upstream's `getlist`: split on `sep`, trim, drop empties.
    ///
    /// `None` when the option is absent; a present (possibly empty) list is
    /// recorded as an array.
    pub fn get_list(&self, option: &str, sep: char) -> Option<Vec<String>> {
        let text = self.section.get_text(option)?;
        let items = split_list(&text, sep);
        self.note(option, json!(items));
        Some(items)
    }

    /// Upstream's `getlists` with two separators and a per-group count.
    ///
    /// A missing option yields an empty vector, so "not set" and "set to
    /// nothing" are alike, as upstream's `default` of an empty list is.
    pub fn get_list_of_lists(
        &self,
        option: &str,
        outer: char,
        inner: char,
        count: usize,
    ) -> Result<Vec<Vec<String>>, ConfigError> {
        let Some(text) = self.section.get_text(option) else {
            return Ok(Vec::new());
        };
        let mut groups = Vec::new();
        for group in split_list(&text, outer) {
            let items = split_list(&group, inner);
            if items.len() != count {
                return Err(ConfigError::new(format!(
                    "Option '{option}' in section '{}' must have {count} elements",
                    self.identifier()
                )));
            }
            groups.push(items);
        }
        self.note(option, json!(groups));
        Ok(groups)
    }

    // -----------------------------------------------------------------------
    // Presence and enumeration
    // -----------------------------------------------------------------------

    /// Whether the option is present. Never records a read, as upstream's
    /// `has_option` does not.
    pub fn has(&self, option: &str) -> bool {
        self.section.has(option)
    }

    /// Every option name in the section, without recording anything.
    ///
    /// Upstream's `fileconfig.options(section)`, which `get_prefix_options`
    /// filters. Order is the section's (the option map is ordered).
    pub fn option_names(&self) -> Vec<String> {
        self.section.parameters.keys().cloned().collect()
    }

    /// The section's options whose name starts with `prefix`, as upstream's
    /// `get_prefix_options`. Reading them is the caller's job.
    pub fn prefix_options(&self, prefix: &str) -> Vec<String> {
        self.section
            .parameters
            .keys()
            .filter(|option| option.starts_with(prefix))
            .cloned()
            .collect()
    }

    // -----------------------------------------------------------------------
    // Internals
    // -----------------------------------------------------------------------

    /// Record `option` as read with its parsed `value`.
    fn note(&self, option: &str, value: Value) {
        self.access.note(&self.identifier(), option, value);
    }

    fn parse_float(&self, option: &str) -> Result<Option<f64>, ConfigError> {
        let Some(text) = self.section.get_text(option) else {
            return Ok(None);
        };
        match text.trim().parse::<f64>() {
            Ok(value) => {
                self.note(option, json!(value));
                Ok(Some(value))
            }
            Err(_) => Err(self.unparseable(option)),
        }
    }

    fn parse_int(&self, option: &str) -> Result<Option<i64>, ConfigError> {
        let Some(text) = self.section.get_text(option) else {
            return Ok(None);
        };
        match text.trim().parse::<i64>() {
            Ok(value) => {
                self.note(option, json!(value));
                Ok(Some(value))
            }
            Err(_) => Err(self.unparseable(option)),
        }
    }

    fn parse_bool(&self, option: &str) -> Result<Option<bool>, ConfigError> {
        let Some(text) = self.section.get_text(option) else {
            return Ok(None);
        };
        match text.trim().to_ascii_lowercase().as_str() {
            "1" | "yes" | "true" | "on" => {
                self.note(option, json!(true));
                Ok(Some(true))
            }
            "0" | "no" | "false" | "off" => {
                self.note(option, json!(false));
                Ok(Some(false))
            }
            _ => Err(self.unparseable(option)),
        }
    }

    fn must_be_specified(&self, option: &str) -> ConfigError {
        ConfigError::new(format!(
            "Option '{option}' in section '{}' must be specified",
            self.identifier()
        ))
    }

    fn unparseable(&self, option: &str) -> ConfigError {
        ConfigError::new(format!(
            "Unable to parse option '{option}' in section '{}'",
            self.identifier()
        ))
    }
}

/// A tracker for wrappers that deliberately do not track.
///
/// Leaked once; every [`ConfigWrapper::untracked`] shares it, so the writes
/// pile up harmlessly instead of allocating per call.
fn untracked_access() -> Arc<AccessTracking> {
    use std::sync::OnceLock;
    static UNTRACKED: OnceLock<Arc<AccessTracking>> = OnceLock::new();
    Arc::clone(UNTRACKED.get_or_init(AccessTracking::shared))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{Config, ConfigValue};

    /// A section with the given options, as the parser would build it.
    fn section(id: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new(id, None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    #[test]
    fn a_read_is_recorded_with_its_parsed_value() {
        let access = AccessTracking::shared();
        let section = section("output_pin fan", &[("value", "0.5"), ("pwm", "true")]);
        let config = ConfigWrapper::new(&section, Arc::clone(&access));

        assert_eq!(config.get_float("value", None).unwrap(), 0.5);
        assert!(config.get_bool("pwm", None).unwrap());

        assert_eq!(access.settings()["output_pin fan"]["value"], json!(0.5));
        assert_eq!(access.settings()["output_pin fan"]["pwm"], json!(true));
    }

    #[test]
    fn a_default_is_recorded_when_it_is_used() {
        let access = AccessTracking::shared();
        let section = section("output_pin fan", &[("pin", "PA0")]);
        let config = ConfigWrapper::new(&section, Arc::clone(&access));

        assert_eq!(config.get_float("value", Some(0.0)).unwrap(), 0.0);
        assert_eq!(access.settings()["output_pin fan"]["value"], json!(0.0));
    }

    #[test]
    fn a_missing_required_option_is_a_config_error() {
        let section = section("output_pin fan", &[]);
        let config = ConfigWrapper::new(&section, AccessTracking::shared());

        let err = config.get("pin", None).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'pin' in section 'output_pin fan' must be specified"
        );
    }

    #[test]
    fn an_unparseable_option_is_a_config_error() {
        let section = section("output_pin fan", &[("value", "abc")]);
        let config = ConfigWrapper::new(&section, AccessTracking::shared());

        let err = config.get_float("value", None).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Unable to parse option 'value' in section 'output_pin fan'"
        );
    }

    #[test]
    fn has_does_not_record_a_read() {
        let access = AccessTracking::shared();
        let section = section("mcu", &[("serial", "/dev/a")]);
        let config = ConfigWrapper::new(&section, Arc::clone(&access));

        assert!(config.has("serial"));
        assert_eq!(access.sections().len(), 0);
    }

    #[test]
    fn get_list_records_the_parsed_list() {
        let access = AccessTracking::shared();
        let section = section("board_pins", &[("mcu", "mcu, zboard,")]);
        let config = ConfigWrapper::new(&section, Arc::clone(&access));

        assert_eq!(
            config.get_list("mcu", ',').unwrap(),
            ["mcu".to_string(), "zboard".to_string()]
        );
        assert_eq!(
            access.settings()["board_pins"]["mcu"],
            json!(["mcu", "zboard"])
        );
    }

    #[test]
    fn get_list_of_lists_checks_the_group_size() {
        let section = section("board_pins", &[("aliases", "A=PA0, B")]);
        let config = ConfigWrapper::new(&section, AccessTracking::shared());

        let err = config
            .get_list_of_lists("aliases", ',', '=', 2)
            .unwrap_err();
        assert!(err.to_string().contains("must have 2 elements"), "{err}");
    }

    #[test]
    fn from_text_parsed_sections_wrap_cleanly() {
        // The wrapper is what the loader builds from a real parse, so the
        // section's own accessors and the wrapper agree on the value.
        let (config, _) = Config::from_text("[output_pin fan]\npin: PA1\n").unwrap();
        let section = config.get_section("output_pin fan").unwrap();
        let wrapper = ConfigWrapper::new(section, AccessTracking::shared());

        assert_eq!(wrapper.get("pin", None).unwrap(), "PA1");
    }
}
