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
use crate::core::klippy::config::object::PrinterConfig;
use crate::core::klippy::config::section::{split_list, ConfigSection};
use crate::core::klippy::config::Config;
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
    /// The `configfile` object, so [`ConfigWrapper::deprecate`] can record a
    /// warning on it. Absent for a wrapper built outside the loader (tests, and
    /// parts that read a stored section without recording warnings).
    configfile: Option<Arc<PrinterConfig>>,
    /// The whole config, for [`ConfigWrapper::sibling`]. Only the loader sets
    /// it: a part that stored its section reads it later through
    /// [`ConfigWrapper::with_configfile`], which has no config to offer.
    config: Option<&'a Config>,
}

impl<'a> ConfigWrapper<'a> {
    /// Wrap `section`, recording every read in `access`.
    pub fn new(section: &'a ConfigSection, access: Arc<AccessTracking>) -> Self {
        Self {
            section,
            access,
            configfile: None,
            config: None,
        }
    }

    /// Wrap `section` with the whole `config`, so [`ConfigWrapper::sibling`]
    /// can read a section that has no factory of its own (a `[stepper_z1]` read
    /// by the rail that owns `[stepper_z]`).
    pub fn with_config(
        section: &'a ConfigSection,
        access: Arc<AccessTracking>,
        configfile: Option<Arc<PrinterConfig>>,
        config: &'a Config,
    ) -> Self {
        Self {
            section,
            access,
            configfile,
            config: Some(config),
        }
    }

    /// Wrap `section` with the `configfile` object the loader registered, so a
    /// module can call [`ConfigWrapper::deprecate`].
    pub fn with_configfile(
        section: &'a ConfigSection,
        access: Arc<AccessTracking>,
        configfile: Arc<PrinterConfig>,
    ) -> Self {
        Self {
            section,
            access,
            configfile: Some(configfile),
            config: None,
        }
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

    /// A sibling section, read through the same access tracker.
    ///
    /// The loader hands a factory its own section only; a part that owns other
    /// sections (`[stepper_z]` owning `[stepper_z1]`, `[printer]` reading the
    /// rails) reaches them here, as upstream's `ConfigWrapper.getsection` does.
    /// The sibling's options are recorded when they are read, which is what
    /// makes a section with no factory of its own valid to `check_unused`.
    ///
    /// `None` when the section does not exist, or when this wrapper was built
    /// without the whole config (an untracked test wrapper, or a part reading a
    /// stored section back).
    pub fn sibling(&self, identifier: &str) -> Option<ConfigWrapper<'a>> {
        let config = self.config?;
        let section = config.get_section(identifier)?;
        Some(ConfigWrapper {
            section,
            access: Arc::clone(&self.access),
            configfile: self.configfile.clone(),
            config: self.config,
        })
    }

    /// Whether a sibling section exists. Reads nothing, records nothing.
    pub fn has_sibling(&self, identifier: &str) -> bool {
        self.config
            .map(|config| config.has_section(identifier))
            .unwrap_or(false)
    }

    /// The shared access record, for a part that wants to keep reading later.
    pub fn access(&self) -> Arc<AccessTracking> {
        Arc::clone(&self.access)
    }

    /// Record a deprecation warning for `option`, if it was written.
    ///
    /// Upstream `ConfigWrapper.deprecate` (`klippy/configfile.py:130`): an
    /// option that is absent is not deprecated, and the warning goes on the
    /// `configfile` object (`warnings` in its status). `value` is the deprecated
    /// value for a `deprecated_value` warning, or `None` for a whole option.
    pub fn deprecate(&self, option: &str, value: Option<Value>) {
        let Some(configfile) = &self.configfile else {
            return;
        };
        let section = self.identifier();
        if !configfile.has_option(&section, option) {
            return;
        }
        configfile.deprecate(&section, option, value, None);
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

    /// A float option with upstream's bounds (`getfloat`'s `minval`/`maxval`/
    /// `above`/`below`).
    ///
    /// # Errors
    /// As [`ConfigWrapper::get_float`], plus the bound wording upstream uses
    /// (`klippy/configfile.py:49-59`).
    #[allow(clippy::too_many_arguments)]
    pub fn get_float_bounded(
        &self,
        option: &str,
        default: Option<f64>,
        minval: Option<f64>,
        maxval: Option<f64>,
        above: Option<f64>,
        below: Option<f64>,
    ) -> Result<f64, ConfigError> {
        let value = self.get_float(option, default)?;
        let identifier = self.identifier();
        if let Some(min) = minval {
            if value < min {
                return Err(ConfigError::new(format!(
                    "Option '{option}' in section '{identifier}' must have minimum of {min}"
                )));
            }
        }
        if let Some(max) = maxval {
            if value > max {
                return Err(ConfigError::new(format!(
                    "Option '{option}' in section '{identifier}' must have maximum of {max}"
                )));
            }
        }
        if let Some(above) = above {
            if value <= above {
                return Err(ConfigError::new(format!(
                    "Option '{option}' in section '{identifier}' must be above {above}"
                )));
            }
        }
        if let Some(below) = below {
            if value >= below {
                return Err(ConfigError::new(format!(
                    "Option '{option}' in section '{identifier}' must be below {below}"
                )));
            }
        }
        Ok(value)
    }

    /// An integer option with `minval`/`maxval` (`getint`).
    ///
    /// # Errors
    /// As [`ConfigWrapper::get_int`], plus the bound wording.
    pub fn get_int_bounded(
        &self,
        option: &str,
        default: Option<i64>,
        minval: Option<i64>,
        maxval: Option<i64>,
    ) -> Result<i64, ConfigError> {
        let value = self.get_int(option, default)?;
        let identifier = self.identifier();
        if let Some(min) = minval {
            if value < min {
                return Err(ConfigError::new(format!(
                    "Option '{option}' in section '{identifier}' must have minimum of {min}"
                )));
            }
        }
        if let Some(max) = maxval {
            if value > max {
                return Err(ConfigError::new(format!(
                    "Option '{option}' in section '{identifier}' must have maximum of {max}"
                )));
            }
        }
        Ok(value)
    }

    /// A string option that must be one of `choices` (`getchoice`).
    ///
    /// # Errors
    /// As [`ConfigWrapper::get`], plus upstream's
    /// `Choice 'x' for option 'y' in section 'z' is not a valid choice`.
    pub fn get_choice(
        &self,
        option: &str,
        choices: &[&str],
        default: Option<&str>,
    ) -> Result<String, ConfigError> {
        let value = self.get(option, default)?;
        if !choices.is_empty() && !choices.contains(&value.as_str()) {
            return Err(ConfigError::new(format!(
                "Choice '{value}' for option '{option}' in section '{}' is not a valid choice",
                self.identifier()
            )));
        }
        Ok(value)
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
    fn test_get_choice_keeps_upstream_wording() {
        let section = section("printer", &[("kinematics", "delta")]);
        let wrapper = ConfigWrapper::untracked(&section);

        let err = wrapper
            .get_choice("kinematics", &["cartesian", "none"], None)
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Choice 'delta' for option 'kinematics' in section 'printer' is not a valid choice"
        );
        assert_eq!(
            wrapper
                .get_choice("kinematics", &["delta", "cartesian"], None)
                .unwrap(),
            "delta"
        );
    }

    #[test]
    fn test_bounded_getters_keep_upstream_wording() {
        let section = section(
            "stepper_x",
            &[
                ("rotation_distance", "0"),
                ("microsteps", "0"),
                ("speed", "10"),
            ],
        );
        let wrapper = ConfigWrapper::untracked(&section);

        assert_eq!(
            wrapper
                .get_float_bounded("rotation_distance", None, None, None, Some(0.0), None)
                .unwrap_err()
                .to_string(),
            "Option 'rotation_distance' in section 'stepper_x' must be above 0"
        );
        assert_eq!(
            wrapper
                .get_int_bounded("microsteps", None, Some(1), None)
                .unwrap_err()
                .to_string(),
            "Option 'microsteps' in section 'stepper_x' must have minimum of 1"
        );
        assert!(wrapper
            .get_float_bounded("speed", None, None, None, Some(0.0), Some(100.0))
            .is_ok());
        assert_eq!(
            wrapper
                .get_float_bounded("speed", None, None, None, None, Some(5.0))
                .unwrap_err()
                .to_string(),
            "Option 'speed' in section 'stepper_x' must be below 5"
        );
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

    #[test]
    fn deprecate_records_a_written_option_and_ignores_an_absent_one() {
        use crate::core::klippy::printer::PrinterObject;

        let (config, _) = Config::from_text("[output_pin fan]\npin: PA0\n").unwrap();
        let section = config.get_section("output_pin fan").unwrap();
        let configfile = Arc::new(PrinterConfig::new(
            AccessTracking::shared(),
            PrinterConfig::raw_config(&config),
        ));
        let wrapper = ConfigWrapper::with_configfile(
            section,
            AccessTracking::shared(),
            Arc::clone(&configfile),
        );

        wrapper.deprecate("pin", None);
        wrapper.deprecate("not_written", None);

        let status = configfile.get_status(0.0);
        assert_eq!(status["warnings"].as_array().unwrap().len(), 1);
        assert_eq!(status["warnings"][0]["option"], json!("pin"));
    }

    // -----------------------------------------------------------------------
    // case folding — upstream's optionxform = str.lower
    // -----------------------------------------------------------------------

    #[test]
    fn lowercase_storage_uppercase_query_is_readable() {
        // The parser lowercases keys, so "pid_kp" is stored.
        let access = AccessTracking::shared();
        let section = section("extruder", &[("pid_kp", "1.0")]);
        let wrapper = ConfigWrapper::new(&section, Arc::clone(&access));

        // A query with different casing still finds it.
        assert_eq!(wrapper.get_str("pid_Kp"), Some("1.0".to_string()));
        assert_eq!(wrapper.get_str("PID_KP"), Some("1.0".to_string()));
    }

    #[test]
    fn must_be_specified_keeps_caller_casing() {
        // The error message must use the caller's original casing.
        let access = AccessTracking::shared();
        let section = section("extruder", &[]);
        let wrapper = ConfigWrapper::new(&section, Arc::clone(&access));

        let err = wrapper.get_float("pid_Kp", None).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'pid_Kp' in section 'extruder' must be specified"
        );

        let err2 = wrapper.get_float("PID_KI", None).unwrap_err();
        assert_eq!(
            err2.to_string(),
            "Option 'PID_KI' in section 'extruder' must be specified"
        );
    }

    #[test]
    fn duplicate_options_same_section_last_write_wins() {
        // The parser lowercases keys, so a duplicate key overwrites the previous one.
        let (config, _) = Config::from_text("[s]\na: first\nA: second\n").unwrap();
        let section = config.get_section("s").unwrap();
        let wrapper = ConfigWrapper::untracked(section);

        assert_eq!(wrapper.get_str("a"), Some("second".to_string()));
        assert_eq!(wrapper.get_str("A"), Some("second".to_string()));
    }

    #[test]
    fn prefix_options_returns_lowercase_names() {
        // The parser lowercases keys, so prefix_options returns lowercase names.
        let (config, _) =
            Config::from_text("[board_pins]\nmcu: main\naliases: A=PA0\naliases_extra: B=PB0\n")
                .unwrap();
        let section = config.get_section("board_pins").unwrap();
        let wrapper = ConfigWrapper::untracked(section);

        let prefixed = wrapper.prefix_options("aliases");
        assert_eq!(
            prefixed,
            vec!["aliases".to_string(), "aliases_extra".to_string()]
        );
    }

    #[test]
    fn access_tracking_keys_are_lowercase() {
        // Access tracking keys are lowercase regardless of the caller's casing.
        let access = AccessTracking::shared();
        let section = section("extruder", &[("pid_kp", "1.0")]);
        let wrapper = ConfigWrapper::new(&section, Arc::clone(&access));

        wrapper.get_float("pid_Kp", None).unwrap();

        // The key in settings is lowercase.
        assert_eq!(access.settings()["extruder"]["pid_kp"], json!(1.0));
    }

    #[test]
    fn section_id_and_multiline_value_keep_case() {
        // Section names and values are NOT lowercased.
        let (config, _) =
            Config::from_text("[gcode_macro TEST_unicode]\nvariable_ABC: 25\n").unwrap();
        let section = config.get_section("gcode_macro TEST_unicode").unwrap();
        // `id` is the first part only; `identifier()` is the full name.
        assert_eq!(section.identifier(), "gcode_macro TEST_unicode");
        assert_eq!(section.get_str("variable_abc"), Some("25"));
    }
}
