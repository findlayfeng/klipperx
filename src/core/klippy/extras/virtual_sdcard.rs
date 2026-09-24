//! `[virtual_sdcard]` — print files directly from a host g-code file
//! (upstream `klippy/extras/virtual_sdcard.py:322 def load_config`).
//!
//! | option | default | role |
//! |---|---|---|
//! | `path` | — (required) | the directory print files live in |
//! | `on_error_gcode` | upstream's `DEFAULT_ERROR_GCODE` | script to run after a file error |
//!
//! The file-replay loop itself (the `M20`..`M27` family,
//! `SDCARD_RESET_FILE`/`SDCARD_PRINT_FILE`, the work timer and its file
//! position tracking) is not ported: those commands stay unregistered, so the
//! dispatcher passes them through as unknown commands
//! (`gcode.rs`, unknown-command path) and this port stores the parsed options
//! without replaying files. The section exists so the config loads.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("virtual_sdcard", order = 30, load = load_config);

/// Upstream's `DEFAULT_ERROR_GCODE` — the default of `on_error_gcode`
/// (`virtual_sdcard.py:14-18`). Read for the option check; not rendered
/// (templates render nowhere in this port yet, see `gcode_macro`).
const DEFAULT_ERROR_GCODE: &str = "
{% if 'heaters' in printer %}
   TURN_OFF_HEATERS
{% endif %}
";

/// The `[virtual_sdcard]` section: the file directory, nothing else.
/// (`virtual_sdcard.py:14-19`.)
#[derive(Debug)]
pub struct VirtualSdCard {
    /// The `path` option as written (`virtual_sdcard.py:17-19`); upstream
    /// runs `normpath(expanduser(…))` over it, which every corpus path is
    /// already through.
    pub path: String,
}

impl VirtualSdCard {
    /// Read the section in upstream's order (`virtual_sdcard.py:15-31`).
    ///
    /// # Errors
    /// A missing `path` — upstream's `config.get('path')` required-option
    /// error, through this port's [`ConfigWrapper::get`].
    fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let path = config.get("path", None)?;
        config.get("on_error_gcode", Some(DEFAULT_ERROR_GCODE))?;
        Ok(Self { path })
    }
}

impl PrinterObject for VirtualSdCard {
    /// The rest state upstream's `get_status` answers
    /// (`virtual_sdcard.py`, `get_status`): no file is ever open here, so the
    /// filename is empty and the counters are zero.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({
            "filename": "",
            "progress": 0.,
            "is_active": false,
            "file_position": 0,
            "file_size": 0,
        })
    }
}

/// The factory `section!` names (`virtual_sdcard.py:322 def load_config`).
pub fn load_config(
    config: &ConfigWrapper,
    _printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = VirtualSdCard::read(config)?;
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

    /// A `[virtual_sdcard]` section with the given options, as the parser
    /// would build it.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("virtual_sdcard", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// The corpus option set (`test/klippy/sdcard_loop.cfg:2-3`) is read
    /// back through the tracker, so `check_unused` accepts the section —
    /// the option-level half of the loader's validation
    /// (`config/validate.rs:48`). `on_error_gcode` is absent; its upstream
    /// default is recorded the way `config.get` records a used default
    /// (`config/wrapper.rs:179-190`).
    #[test]
    fn every_option_the_section_writes_is_recorded_as_read() {
        let text = "[virtual_sdcard]\npath: test/klippy/sdcard_loop\n";
        let (config, _) = Config::from_text(text).expect("the section parses");
        let sect = config.get_section("virtual_sdcard").expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(sect, Arc::clone(&access));

        let parsed = VirtualSdCard::read(&wrapper).expect("the section reads");
        check_unused(&config, &access, &[]).expect("no option is left unread");

        assert_eq!(parsed.path, "test/klippy/sdcard_loop");
        assert!(access.contains("virtual_sdcard", "path"));
        assert!(access.contains("virtual_sdcard", "on_error_gcode"));
    }

    /// An explicit `on_error_gcode` is read and kept as written
    /// (`virtual_sdcard.py:27-31`).
    #[test]
    fn an_explicit_on_error_gcode_is_read() {
        let sect = section(&[("path", "/tmp/gcodes"), ("on_error_gcode", "M112")]);
        let config = ConfigWrapper::untracked(&sect);
        let parsed = VirtualSdCard::read(&config).expect("the section reads");
        assert_eq!(parsed.path, "/tmp/gcodes");
    }

    /// A missing `path` is refused with the loader's required-option
    /// wording — upstream's `config.get('path')` (`virtual_sdcard.py:17`).
    #[test]
    fn a_missing_path_is_refused_with_the_loader_wording() {
        let sect = section(&[]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            VirtualSdCard::read(&config).unwrap_err().to_string(),
            "Option 'path' in section 'virtual_sdcard' must be specified"
        );
    }

    /// With no file ever open here, `get_status` reports the rest state
    /// (`virtual_sdcard.py` `get_status`).
    #[test]
    fn the_rest_state_reports_no_file() {
        let sect = section(&[("path", "/tmp/gcodes")]);
        let config = ConfigWrapper::untracked(&sect);
        let object = VirtualSdCard::read(&config).expect("the section reads");
        assert_eq!(
            object.get_status(0.0),
            json!({
                "filename": "",
                "progress": 0.,
                "is_active": false,
                "file_position": 0,
                "file_size": 0,
            })
        );
    }
}
