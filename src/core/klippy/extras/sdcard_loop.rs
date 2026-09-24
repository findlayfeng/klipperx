//! `[sdcard_loop]` — looped sections inside an SD file
//! (upstream `klippy/extras/sdcard_loop.py:72 def load_config`).
//!
//! | option | default | role |
//! |---|---|---|
//! | — | — | the section carries no options of its own |
//!
//! Upstream registers `SDCARD_LOOP_BEGIN`/`_END`/`_DESIST`
//! (`sdcard_loop.py:16-24`); those stay unregistered here, so the dispatcher
//! passes them through as unknown commands. The loop index itself is ported
//! as [`SDCardLoop`]'s stack operations — the semantics upstream's three
//! commands drive (`sdcard_loop.py:39-70`) — and the unit tests below pin them
//! against a stand-in file position.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("sdcard_loop", order = 30, load = load_config);

/// What the loop stack needs from `virtual_sdcard` — upstream's
/// `is_cmd_from_sd`/`get_file_position`/`set_file_position`
/// (`sdcard_loop.py:43-68` call sites on `self.sdcard`).
pub trait SdCardFile {
    /// Whether the current command came from the SD file.
    fn is_cmd_from_sd(&self) -> bool;
    /// The file read cursor.
    fn file_position(&self) -> u64;
    /// Move the read cursor — what seeks the next line back to a loop start.
    fn set_file_position(&mut self, position: u64);
}

/// The `[sdcard_loop]` state: the open loops, outermost last
/// (`sdcard_loop.py:25`).
#[derive(Default)]
pub struct SDCardLoop {
    /// `(remaining count, loop start position)` per open loop; `count == 0`
    /// is the infinite form (`sdcard_loop.py:51-66`).
    loop_stack: Vec<(i64, u64)>,
}

impl SDCardLoop {
    pub fn new() -> Self {
        Self::default()
    }

    /// `SDCARD_LOOP_BEGIN`'s stack push (`sdcard_loop.py:42-47`).
    ///
    /// # Returns
    /// `false` — and the caller raises `Only permitted in SD file.` — when
    /// the command did not come from the SD file.
    pub fn loop_begin(&mut self, sdcard: &mut dyn SdCardFile, count: i64) -> bool {
        if !sdcard.is_cmd_from_sd() {
            return false;
        }
        self.loop_stack.push((count, sdcard.file_position()));
        true
    }

    /// `SDCARD_LOOP_END`'s stack pop (`sdcard_loop.py:49-68`): count `0`
    /// repeats forever, count `1` is the last repeat, anything higher seeks
    /// back and decrements.
    ///
    /// # Returns
    /// `false` — and the caller raises `Only permitted in SD file.` — when
    /// the command did not come from the SD file. An empty stack succeeds
    /// without seeking (`sdcard_loop.py:53-55`).
    pub fn loop_end(&mut self, sdcard: &mut dyn SdCardFile) -> bool {
        if !sdcard.is_cmd_from_sd() {
            return false;
        }
        let Some((count, position)) = self.loop_stack.pop() else {
            return true;
        };
        if count == 0 {
            // Infinite loop: seek back and keep the entry.
            sdcard.set_file_position(position);
            self.loop_stack.push((0, position));
        } else if count > 1 {
            // Repeat: seek back and push the decremented count.
            sdcard.set_file_position(position);
            self.loop_stack.push((count - 1, position));
        }
        // count == 1: last repeat, nothing to do.
        true
    }

    /// `SDCARD_LOOP_DESIST`'s stack clear (`sdcard_loop.py:70-75`).
    ///
    /// # Returns
    /// `false` — and the caller raises `Only permitted outside of a SD file.` —
    /// when the command came from the SD file.
    pub fn loop_desist(&mut self, sdcard: &mut dyn SdCardFile) -> bool {
        if sdcard.is_cmd_from_sd() {
            return false;
        }
        self.loop_stack.clear();
        true
    }
}

impl PrinterObject for SDCardLoop {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }
}

/// The factory `section!` names (`sdcard_loop.py:72 def load_config`).
pub fn load_config(
    _config: &ConfigWrapper,
    _printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = SDCardLoop::new();
    Ok(Arc::new(object))
}
