//! `[display_status]` — the `M73`/`M117` progress and message state
//! (upstream `klippy/extras/display_status.py:49 def load_config`).
//!
//! | option | default | role |
//! |---|---|---|
//! | — | — | the section carries no options of its own |
//!
//! Upstream registers `M73`, `M117` and `SET_DISPLAY_TEXT`
//! (`display_status.py:19-25`); those stay unregistered here, so the
//! dispatcher passes them through as unknown commands and the state below is
//! only ever the rest state.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("display_status", order = 30, load = load_config);

/// The `[display_status]` section: the progress percentage and the message
/// (`display_status.py:12-16`). Nothing sets them while `M73`/`M117` are
/// unported, hence the field-level `allow`.
#[allow(dead_code)]
#[derive(Default)]
pub struct DisplayStatus {
    /// The clamped `M73` progress, `None` once expired
    /// (`display_status.py:14-15`).
    progress: Option<f64>,
    /// The `M117`/`SET_DISPLAY_TEXT` message (`display_status.py:15`).
    message: Option<String>,
}

impl PrinterObject for DisplayStatus {
    /// The rest state upstream's `get_status` answers
    /// (`display_status.py:27-41`): no `M73` has run, and this port's
    /// `virtual_sdcard` reports `progress: 0`, so the fallback progress is
    /// `0.` and the message is `None`.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({
            "progress": self.progress.unwrap_or(0.),
            "message": self.message,
        })
    }
}

/// The factory `section!` names (`display_status.py:49 def load_config`).
pub fn load_config(
    _config: &ConfigWrapper,
    _printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = DisplayStatus::default();
    Ok(Arc::new(object))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::{check_unused, Config};
    use crate::core::klippy::reactor::ManualReactor;

    /// The bare `[display_status]` section loads with no options to read —
    /// upstream's class reads none of its own either
    /// (`display_status.py:12-16`), so `check_unused` has nothing to flag.
    #[test]
    fn the_bare_section_loads_and_leaves_no_option_unread() {
        let text = "[display_status]\n";
        let (config, _) = Config::from_text(text).expect("the section parses");
        let sect = config.get_section("display_status").expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(sect, Arc::clone(&access));

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let object = load_config(&wrapper, &printer).expect("the section loads");
        check_unused(&config, &access, &["display_status".to_string()])
            .expect("no option is left unread");

        // No `M73` has run and no `virtual_sdcard` progress exists yet, so
        // the fallback progress is `0.` and the message is unset
        // (`display_status.py:27-41`).
        assert_eq!(
            object.get_status(0.0),
            json!({ "progress": 0., "message": Value::Null })
        );
    }
}
