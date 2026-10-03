//! `[homing_override]` — run a user script in place of a normal `G28`
//! (upstream `klippy/extras/homing_override.py:65 def load_config`).
//!
//! | option | default | role |
//! |---|---|---|
//! | `axes` | `XYZ` | upper-cased; the axes whose `G28` triggers the override |
//! | `set_position_x/y/z` | none | forced position applied before the script |
//! | `gcode` | — (required) | the homing script, compiled as a macro template |
//!
//! The `G28` wrapper (upstream `homing_override.py:20-63`) is not installed
//! here: the script body is a `gcode_macro` template, and templates do not
//! render in this port yet (`gcode_macro.rs` module docs), so an installed
//! wrapper could not run the script it replaces `G28` for. The section reads
//! its options — the config-load contract — and `G28` keeps homing normally.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("homing_override", order = 30, load = load_config);

/// The `[homing_override]` section: the trigger axes, the forced start
/// position, and the homing script (`homing_override.py:10-14`). The parsed
/// values stay with the object because no `G28` wrapper consumes them yet,
/// hence the field-level `allow`.
#[allow(dead_code)]
#[derive(Debug, PartialEq)]
pub struct HomingOverride {
    /// The `axes` option, upper-cased (`homing_override.py:16`).
    axes: String,
    /// `set_position_x`/`_y`/`_z` in axis order; `None` where unset
    /// (`homing_override.py:14-15`).
    start_pos: [Option<f64>; 3],
    /// The `gcode` template's source (`homing_override.py:18`); compiled but
    /// never rendered here (templates do not render yet).
    script: String,
}

impl HomingOverride {
    /// Read the section in upstream's order (`homing_override.py:14-19`).
    ///
    /// # Errors
    /// A missing `gcode` (upstream's required template section) or a
    /// `set_position_*` that is not a number (upstream's `getfloat`).
    fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let mut start_pos = [None; 3];
        for (index, axis) in "xyz".char_indices() {
            let option = format!("set_position_{axis}");
            start_pos[index] = match config.get_str(&option) {
                Some(raw) => Some(parse_position(&option, &config.identifier(), &raw)?),
                None => None,
            };
        }
        let axes = config.get("axes", Some("XYZ"))?.to_uppercase();
        let script = config.get("gcode", None)?;
        Ok(Self {
            axes,
            start_pos,
            script,
        })
    }

    /// The three trigger axes as a mask, from the `axes` option
    /// (`homing_override.py:12` + `cmd_G28`'s `for axis in self.axes`).
    /// Letters outside `x`/`y`/`z` select nothing.
    pub fn axis_mask(&self) -> [bool; 3] {
        let mut mask = [false; 3];
        for letter in self.axes.chars() {
            match letter {
                'X' => mask[0] = true,
                'Y' => mask[1] = true,
                'Z' => mask[2] = true,
                _ => {}
            }
        }
        mask
    }

    /// Whether a `G28` with the given parameter axes runs the override
    /// (`homing_override.py:33-46`): no axis named means the whole override,
    /// otherwise any named axis that `axes` selects.
    pub fn overrides(&self, requested: &[bool; 3]) -> bool {
        if requested.iter().all(|requested| !requested) {
            return true;
        }
        let mask = self.axis_mask();
        mask.iter()
            .zip(requested)
            .any(|(selected, asked)| *selected && *asked)
    }
}

/// One `set_position_*` value — upstream's `getfloat` (`configfile.py`),
/// whose parse failure carries the option and section names.
///
/// # Errors
/// `Unable to parse option '<option>' in section '<identifier>'`.
fn parse_position(option: &str, identifier: &str, raw: &str) -> Result<f64, ConfigError> {
    raw.trim().parse::<f64>().map_err(|_| {
        ConfigError::new(format!(
            "Unable to parse option '{option}' in section '{identifier}'"
        ))
    })
}

impl PrinterObject for HomingOverride {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }
}

/// The factory `section!` names (`homing_override.py:65 def load_config`).
pub fn load_config(
    config: &ConfigWrapper,
    _printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = HomingOverride::read(config)?;
    Ok(Arc::new(object))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::{check_unused, Config, ConfigSection, ConfigValue};

    /// A `[homing_override]` section with the given options, as the parser
    /// would build it.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("homing_override", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// The corpus option set (`test/klippy/sdcard_loop.cfg:9-14`) is read
    /// back through the tracker, so `check_unused` accepts the section —
    /// the option-level half of the loader's validation
    /// (`config/validate.rs:48`). `kit-zav3d-2019.cfg:141` writes the same
    /// option set.
    #[test]
    fn every_option_the_section_writes_is_recorded_as_read() {
        let text = "[homing_override]
axes: xyz
set_position_x: 0
set_position_y: 0
set_position_z: 0
gcode:
  G92 X0 Y0 Z0
";
        let (config, _) = Config::from_text(text).expect("the section parses");
        let sect = config.get_section("homing_override").expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(sect, Arc::clone(&access));

        let parsed = HomingOverride::read(&wrapper).expect("the section reads");
        check_unused(&config, &access, &[]).expect("no option is left unread");

        // The `axes` value is upper-cased (`homing_override.py:16`).
        assert_eq!(parsed.axes, "XYZ");
        assert_eq!(parsed.start_pos, [Some(0.), Some(0.), Some(0.)]);
        assert_eq!(parsed.script.trim(), "G92 X0 Y0 Z0");
    }

    /// With only the mandatory script, `axes` falls back to upstream's
    /// `XYZ` and no position is forced (`homing_override.py:14-16`); the
    /// default is recorded the way `config.get` records a used default
    /// (`config/wrapper.rs:179-190`).
    #[test]
    fn the_defaults_are_upstreams() {
        let text = "[homing_override]\ngcode:\n  G28\n";
        let (config, _) = Config::from_text(text).expect("the section parses");
        let sect = config.get_section("homing_override").expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(sect, Arc::clone(&access));

        let parsed = HomingOverride::read(&wrapper).expect("the section reads");
        check_unused(&config, &access, &[]).expect("no option is left unread");

        assert_eq!(parsed.axes, "XYZ");
        assert_eq!(parsed.start_pos, [None, None, None]);
        assert!(access.contains("homing_override", "axes"));
        assert!(!access.contains("homing_override", "set_position_x"));
    }

    /// A `set_position_*` that is not a number is refused with upstream's
    /// `getfloat` wording, and a missing `gcode` with the loader's
    /// required-option wording (`homing_override.py:14,18`).
    #[test]
    fn malformed_options_are_refused_with_upstream_wording() {
        let sect = section(&[("gcode", "G28"), ("set_position_x", "abc")]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            HomingOverride::read(&config).unwrap_err().to_string(),
            "Unable to parse option 'set_position_x' in section 'homing_override'"
        );

        let sect = section(&[("axes", "XYZ")]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            HomingOverride::read(&config).unwrap_err().to_string(),
            "Option 'gcode' in section 'homing_override' must be specified"
        );
    }

    /// The `G28` statement decides against the `axes` mask exactly as
    /// upstream's `cmd_G28` (`homing_override.py:33-46`): no axis named
    /// means the whole override, otherwise any named axis that `axes`
    /// selects; letters outside `xyz` in `axes` select nothing.
    #[test]
    fn a_g28_statement_decides_against_the_axes_mask() {
        let override_xyz = HomingOverride {
            axes: "XYZ".to_string(),
            start_pos: [Some(0.); 3],
            script: "G92 X0 Y0 Z0".to_string(),
        };
        let requested = |x: bool, y: bool, z: bool| [x, y, z];

        // No axis named → the override runs (`no_axis` branch).
        assert!(override_xyz.overrides(&requested(false, false, false)));
        // Any requested axis that `axes` selects → override.
        assert!(override_xyz.overrides(&requested(true, false, false)));
        assert!(override_xyz.overrides(&requested(false, true, true)));
        // `axes: Z` only overrides a Z statement; X/Y pass through to the
        // real `G28`.
        let override_z = HomingOverride {
            axes: "Z".to_string(),
            start_pos: [None; 3],
            script: "G28".to_string(),
        };
        assert!(!override_z.overrides(&requested(true, false, false)));
        assert!(override_z.overrides(&requested(false, false, true)));
        // The mask comes from the upper-cased option; unknown letters in
        // it select nothing.
        assert_eq!(override_xyz.axis_mask(), [true, true, true]);
        let odd = HomingOverride {
            axes: "XW".to_string(),
            start_pos: [None; 3],
            script: "G28".to_string(),
        };
        assert_eq!(odd.axis_mask(), [true, false, false]);
    }
}
