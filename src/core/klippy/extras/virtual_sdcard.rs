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
pub struct VirtualSdCard {
    /// The `path` option, normalized as upstream's `normpath(expanduser(…))`
    /// would leave a plain path (`virtual_sdcard.py:17-19`).
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
