//! `[pause_resume]` — pause/resume (upstream `klippy/extras/pause_resume.py`).
//!
//! Only what the filament sensors need is here: the module object
//! (`printer.load_object(config, 'pause_resume')`) and
//! [`PauseResume::send_pause_command`], which a runout event calls
//! (`filament_switch_sensor.py:48-53`).
//!
//! [`PauseResume::ensure`] reads `recover_velocity` from whichever config
//! triggered the load: upstream's `load_object` passes the caller's config to
//! the new module's `load_config`, and `PauseResume.__init__` reads
//! `recover_velocity` from it (`pause_resume.py:9`).
//!
//! # What is not here
//!
//! The `PAUSE` / `RESUME` / `CLEAR_PAUSE` / `CANCEL_PRINT` commands, the
//! position save/restore, and the `save_gcode_state` hooks. A sensor's runout
//! event is the only caller reachable here, and on the corpus's fake firmware no
//! button event is ever generated, so `send_pause_command` is never reached.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The name the sensors look the object up by (`load_object(config,
/// 'pause_resume')`).
pub const PAUSE_RESUME_OBJECT: &str = "pause_resume";

/// The `[pause_resume]` module object (upstream's `PauseResume`).
#[derive(Default)]
pub struct PauseResume {
    /// `is_paused`: set by the (unimplemented) `PAUSE` command.
    is_paused: AtomicBool,
    /// `pause_command_sent`: upstream's guard so a runout does not pause twice.
    pause_command_sent: AtomicBool,
    /// The machine, to report `action:paused` the way upstream's
    /// `send_pause_command` does.
    printer: Option<Weak<Printer>>,
}

impl PauseResume {
    /// The single `pause_resume` object; the first caller creates it, as
    /// upstream's `printer.load_object(config, 'pause_resume')` does.
    ///
    /// # Errors
    /// An unparsable `recover_velocity`, or a duplicate registration.
    pub fn ensure(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
    ) -> Result<Arc<PauseResume>, ConfigError> {
        if let Some(existing) = printer.lookup_object_as::<PauseResume>(PAUSE_RESUME_OBJECT) {
            return Ok(existing);
        }
        // Upstream reads `recover_velocity` from whichever config triggered the
        // load (`pause_resume.py:9`); `load_object` passes the caller's config.
        config.get_float("recover_velocity", Some(50.0))?;
        let object = Arc::new(PauseResume {
            is_paused: AtomicBool::new(false),
            pause_command_sent: AtomicBool::new(false),
            printer: Some(Arc::downgrade(printer)),
        });
        printer.add_object(
            PAUSE_RESUME_OBJECT,
            Arc::clone(&object) as Arc<dyn PrinterObject>,
        )?;
        Ok(object)
    }

    /// Whether the print is (officially) paused, for `get_status`.
    pub fn is_paused(&self) -> bool {
        self.is_paused.load(Ordering::SeqCst)
    }

    /// Upstream's `send_pause_command` (`pause_resume.py:46-56`): pause from
    /// inside an event, once. The virtual-SD branch is not reachable without a
    /// `virtual_sdcard` object, so this is the `respond_info("action:paused")`
    /// branch.
    pub fn send_pause_command(&self) {
        if self.pause_command_sent.load(Ordering::SeqCst) {
            return;
        }
        if let Some(printer) = self.printer.as_ref().and_then(Weak::upgrade) {
            if let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) {
                gcode.respond_info("action:paused", true);
            }
        }
        self.pause_command_sent.store(true, Ordering::SeqCst);
    }
}

impl std::fmt::Debug for PauseResume {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PauseResume")
            .field("is_paused", &self.is_paused())
            .finish()
    }
}

impl PrinterObject for PauseResume {
    /// Upstream's `PauseResume.get_status` (`pause_resume.py:41-44`).
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "is_paused": self.is_paused() })
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::reactor::ManualReactor;

    fn wrapper(text: &str) -> (crate::core::klippy::config::Config, ConfigWrapper<'static>) {
        // Leak the section so the wrapper can borrow it for the test's life; a
        // test-only convenience, not how the loader builds wrappers.
        let (config, _) = Config::from_text(text).expect("the config parses");
        let section: &'static _ = Box::leak(Box::new(
            config
                .get_section("probe")
                .expect("the section exists")
                .clone(),
        ));
        (config, ConfigWrapper::untracked(section))
    }

    #[test]
    fn test_ensure_reads_recover_velocity_and_shares_one_object() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (_config, wrapper) = wrapper("[probe]\nrecover_velocity: 25\n");
        let first = PauseResume::ensure(&wrapper, &printer).expect("the object registers");
        let second = PauseResume::ensure(&wrapper, &printer).expect("the object is reused");
        assert!(Arc::ptr_eq(&first, &second));
        assert!(!first.is_paused());
    }

    #[test]
    fn test_send_pause_command_is_idempotent() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (_config, wrapper) = wrapper("[probe]\n");
        let object = PauseResume::ensure(&wrapper, &printer).expect("the object registers");
        object.send_pause_command();
        assert!(object.pause_command_sent.load(Ordering::SeqCst));
    }
}
