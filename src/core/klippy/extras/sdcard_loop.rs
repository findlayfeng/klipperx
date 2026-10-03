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

    /// `SDCARD_LOOP_DESIST`'s stack clear (`sdcard_loop.py:69`).
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

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::{check_unused, Config};

    /// A stand-in file position: whether the command came from the SD file,
    /// and where the read cursor sits.
    struct FakeSdCard {
        from_sd: bool,
        position: u64,
    }

    impl SdCardFile for FakeSdCard {
        fn is_cmd_from_sd(&self) -> bool {
            self.from_sd
        }
        fn file_position(&self) -> u64 {
            self.position
        }
        fn set_file_position(&mut self, position: u64) {
            self.position = position;
        }
    }

    /// The bare `[sdcard_loop]` section loads with no options to read —
    /// upstream's class reads none either (`sdcard_loop.py:11-26`) — and
    /// `check_unused` has nothing to flag.
    #[test]
    fn the_bare_section_loads_and_leaves_no_option_unread() {
        let text = "[sdcard_loop]\n";
        let (config, _) = Config::from_text(text).expect("the section parses");
        let sect = config.get_section("sdcard_loop").expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(sect, Arc::clone(&access));

        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let object = load_config(&wrapper, &printer).expect("the section loads");
        check_unused(&config, &access, &["sdcard_loop".to_string()])
            .expect("no option is left unread");
        assert_eq!(object.get_status(0.0), json!({}));
    }

    /// BEGIN/END only work inside the SD file: outside, both refuse (the
    /// caller raises `Only permitted in SD file.`) and the stack is left
    /// alone (`sdcard_loop.py:41-55`).
    #[test]
    fn begin_and_end_only_work_inside_the_sd_file() {
        let mut loop_ = SDCardLoop::new();
        let mut sd = FakeSdCard {
            from_sd: false,
            position: 64,
        };

        assert!(!loop_.loop_begin(&mut sd, 3));
        assert!(!loop_.loop_end(&mut sd));
        assert!(loop_.loop_stack.is_empty());

        sd.from_sd = true;
        assert!(loop_.loop_begin(&mut sd, 3));
        assert_eq!(loop_.loop_stack, [(3, 64)]);
    }

    /// END's count semantics (`sdcard_loop.py:49-66`): `0` repeats
    /// forever (seek back, entry kept), `1` is the last repeat (pop, no
    /// seek), anything higher seeks back and decrements.
    #[test]
    fn end_repeats_per_the_upstream_count_semantics() {
        // count == 0: infinite — seek back, keep the entry.
        let mut loop_ = SDCardLoop::new();
        let mut sd = FakeSdCard {
            from_sd: true,
            position: 100,
        };
        assert!(loop_.loop_begin(&mut sd, 0));
        sd.position = 140; // the body of the loop ran on
        assert!(loop_.loop_end(&mut sd));
        assert_eq!(sd.position, 100, "the infinite loop seeks back");
        assert_eq!(loop_.loop_stack, [(0, 100)]);

        // count == 1: last repeat — a fresh loop pops without seeking.
        let mut loop_ = SDCardLoop::new();
        let mut sd = FakeSdCard {
            from_sd: true,
            position: 100,
        };
        assert!(loop_.loop_begin(&mut sd, 1));
        sd.position = 140;
        assert!(loop_.loop_end(&mut sd));
        assert_eq!(sd.position, 140, "the last repeat does not seek");
        assert!(loop_.loop_stack.is_empty());

        // count > 1: seek back and decrement.
        let mut loop_ = SDCardLoop::new();
        let mut sd = FakeSdCard {
            from_sd: true,
            position: 100,
        };
        assert!(loop_.loop_begin(&mut sd, 3));
        sd.position = 140;
        assert!(loop_.loop_end(&mut sd));
        assert_eq!(sd.position, 100, "a pending repeat seeks back");
        assert_eq!(loop_.loop_stack, [(2, 100)]);

        // END outside the SD file refuses (the caller raises
        // `Only permitted in SD file.`).
        sd.from_sd = false;
        assert!(!loop_.loop_end(&mut sd));
        assert_eq!(loop_.loop_stack, [(2, 100)], "the stack is untouched");
    }

    /// END with an empty stack succeeds without seeking, and nested loops
    /// unwind outermost-last (`sdcard_loop.py:53-55,44`).
    #[test]
    fn end_with_an_empty_stack_seeks_nothing_and_nests_last() {
        let mut loop_ = SDCardLoop::new();
        let mut sd = FakeSdCard {
            from_sd: true,
            position: 10,
        };
        assert!(loop_.loop_end(&mut sd), "an empty stack is a no-op");
        assert_eq!(sd.position, 10);

        // Outer at 10, inner at 50; the inner end pops first.
        assert!(loop_.loop_begin(&mut sd, 2));
        sd.position = 50;
        assert!(loop_.loop_begin(&mut sd, 2));
        assert_eq!(loop_.loop_stack, [(2, 10), (2, 50)]);
        sd.position = 60;
        assert!(loop_.loop_end(&mut sd));
        assert_eq!(sd.position, 50, "the inner loop seeks to its own start");
        assert_eq!(loop_.loop_stack, [(2, 10), (1, 50)]);
    }

    /// DESIST only works outside the SD file: inside, it refuses (the
    /// caller raises `Only permitted outside of a SD file.`) and keeps the
    /// stack; outside, it clears every open loop
    /// (`sdcard_loop.py:34-37,69-75`).
    #[test]
    fn desist_clears_the_stack_only_outside_the_sd_file() {
        let mut loop_ = SDCardLoop::new();
        let mut sd = FakeSdCard {
            from_sd: true,
            position: 10,
        };
        assert!(loop_.loop_begin(&mut sd, 0));

        assert!(!loop_.loop_desist(&mut sd));
        assert_eq!(loop_.loop_stack, [(0, 10)], "the stack survives");

        sd.from_sd = false;
        assert!(loop_.loop_desist(&mut sd));
        assert!(loop_.loop_stack.is_empty());
    }
}
