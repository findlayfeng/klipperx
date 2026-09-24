//! `[homing_override]` — run a user script in place of a normal `G28`
//! (upstream `klippy/extras/homing_override.py:65 def load_config`).
//!
//! | option | default | role |
//! |---|---|---|
//! | `axes` | `XYZ` | upper-cased; the axes whose `G28` triggers the override |
//! | `set_position_x/y/z` | none | forced position applied before the script |
//! | `gcode` | — (required) | the homing script, compiled as a macro template |
//!
//! The `G28` wrapper (upstream `homing_override.py:31-64`) is not installed
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
/// position, and the homing script (`homing_override.py:14-19`). The parsed
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
    /// (`homing_override.py:16` + `cmd_G28`'s `for axis in self.axes`).
    /// Letters outside `x`/`y`/`z` select nothing.
    fn axis_mask(&self) -> [bool; 3] {
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
    fn overrides(&self, requested: &[bool; 3]) -> bool {
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
