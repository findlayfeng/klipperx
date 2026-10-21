//! `[sdcard_loop]` — looped sections inside an SD file
//! (upstream `klippy/extras/sdcard_loop.py:72 def load_config`).
//!
//! | option | default | role |
//! |---|---|---|
//! | — | — | the section carries no options of its own |
//!
//! Upstream registers `SDCARD_LOOP_BEGIN`/`_END`/`_DESIST`
//! (`sdcard_loop.py:14-22`) and drives them from [`SDCardLoop`]'s stack
//! operations (`sdcard_loop.py:39-70`). `_BEGIN` and `_END` only work inside
//! the file — they record / rewind the `virtual_sdcard` read cursor — while
//! `_DESIST` only works from outside it, which is why the three refuse each
//! other's context with upstream's own wording.
//!
//! The `virtual_sdcard` handle is resolved per command rather than at load:
//! `[sdcard_loop]` and `[virtual_sdcard]` share an `order`, and `sdcard_loop`
//! sorts first, so the object it needs does not exist yet when this section
//! loads (upstream's `printer.load_object(config, "virtual_sdcard")`,
//! `sdcard_loop.py:13`, builds it on demand instead).

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::virtual_sdcard::{VirtualSdCard, VIRTUAL_SDCARD_OBJECT};
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("sdcard_loop", order = 30, load = load_config);

/// `SDCARD_LOOP_BEGIN`'s help text (`sdcard_loop.py:23`).
const SDCARD_LOOP_BEGIN_HELP: &str = "Begins a looped section in the SD file.";

/// `SDCARD_LOOP_END`'s help text (`sdcard_loop.py:29`).
const SDCARD_LOOP_END_HELP: &str = "Ends a looped section in the SD file.";

/// `SDCARD_LOOP_DESIST`'s help text (`sdcard_loop.py:35`).
const SDCARD_LOOP_DESIST_HELP: &str = "Stops iterating the current loop stack.";

/// What the loop stack needs from `virtual_sdcard` — upstream's
/// `is_cmd_from_sd`/`get_file_position`/`set_file_position`
/// (`sdcard_loop.py:38-65` call sites on `self.sdcard`).
///
/// The cursor moves through `&self`: a command holds the `sdcard` object as
/// shared state, and the file module keeps its position behind a lock.
pub trait SdCardFile {
    /// Whether the current command came from the SD file.
    fn is_cmd_from_sd(&self) -> bool;
    /// The file read cursor.
    fn file_position(&self) -> u64;
    /// Move the read cursor — what seeks the next line back to a loop start.
    fn set_file_position(&self, position: u64);
}

/// The `[sdcard_loop]` state: the open loops, outermost last
/// (`sdcard_loop.py:25`).
pub struct SDCardLoop {
    /// The machine, for the `virtual_sdcard` handle each command needs. Weak
    /// because the machine owns this object.
    printer: Weak<Printer>,
    /// `(remaining count, loop start position)` per open loop; `count == 0`
    /// is the infinite form (`sdcard_loop.py:51-66`). A `Mutex` because the
    /// command handlers hold the object shared.
    loop_stack: Mutex<Vec<(i64, u64)>>,
}

impl SDCardLoop {
    fn new(printer: &Arc<Printer>) -> Self {
        Self {
            printer: Arc::downgrade(printer),
            loop_stack: Mutex::new(Vec::new()),
        }
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Vec<(i64, u64)>> {
        self.loop_stack.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// The `virtual_sdcard` object the commands operate on
    /// (`sdcard_loop.py:13`).
    ///
    /// # Errors
    /// A config with no `[virtual_sdcard]` section, reported with upstream's
    /// lookup wording. Upstream reaches the same dead end while loading the
    /// config (`printer.load_object` builds the module from an empty section,
    /// whose required `path` is missing); here the section may load first, so
    /// the check happens when a loop command runs.
    fn sdcard(&self) -> Result<Arc<VirtualSdCard>, CommandError> {
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| CommandError::new("the printer is gone"))?;
        printer
            .lookup_object_as::<VirtualSdCard>(VIRTUAL_SDCARD_OBJECT)
            .ok_or_else(|| {
                CommandError::new(format!("Unknown config object '{VIRTUAL_SDCARD_OBJECT}'"))
            })
    }

    /// `SDCARD_LOOP_BEGIN`'s stack push (`sdcard_loop.py:42-47`).
    ///
    /// # Returns
    /// `false` — and the caller raises `Only permitted in SD file.` — when
    /// the command did not come from the SD file.
    pub fn loop_begin(&self, sdcard: &dyn SdCardFile, count: i64) -> bool {
        if !sdcard.is_cmd_from_sd() {
            return false;
        }
        self.lock().push((count, sdcard.file_position()));
        true
    }

    /// `SDCARD_LOOP_END`'s stack pop (`sdcard_loop.py:50-62`): count `0`
    /// repeats forever, count `1` is the last repeat, anything higher seeks
    /// back and decrements.
    ///
    /// # Returns
    /// `false` — and the caller raises `Only permitted in SD file.` — when
    /// the command did not come from the SD file. An empty stack succeeds
    /// without seeking (`sdcard_loop.py:47-49`).
    pub fn loop_end(&self, sdcard: &dyn SdCardFile) -> bool {
        if !sdcard.is_cmd_from_sd() {
            return false;
        }
        let mut stack = self.lock();
        let Some((count, position)) = stack.pop() else {
            return true;
        };
        if count == 0 {
            // Infinite loop: seek back and keep the entry.
            sdcard.set_file_position(position);
            stack.push((0, position));
        } else if count > 1 {
            // Repeat: seek back and push the decremented count.
            sdcard.set_file_position(position);
            stack.push((count - 1, position));
        }
        // count == 1: last repeat, nothing to do.
        true
    }

    /// `SDCARD_LOOP_DESIST`'s stack clear (`sdcard_loop.py:69`).
    ///
    /// # Returns
    /// `false` — and the caller raises `Only permitted outside of a SD file.` —
    /// when the command came from the SD file.
    pub fn loop_desist(&self, sdcard: &dyn SdCardFile) -> bool {
        if sdcard.is_cmd_from_sd() {
            return false;
        }
        tracing::info!("Desisting existing SD loops");
        self.lock().clear();
        true
    }

    /// The stack as `(count, position)` pairs, for the tests.
    #[cfg(test)]
    fn stack(&self) -> Vec<(i64, u64)> {
        self.lock().clone()
    }
}

impl PrinterObject for SDCardLoop {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }
}

// ---------------------------------------------------------------------------
// G-Code command handlers
// ---------------------------------------------------------------------------

/// `SDCARD_LOOP_BEGIN [COUNT=<n>]` (`sdcard_loop.py:24-27`).
fn cmd_sdcard_loop_begin(
    object: &Arc<SDCardLoop>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    // `gcmd.get_int("COUNT", minval=0)` (`sdcard_loop.py:26`): a required
    // integer, no repeats below zero (0 is the infinite loop).
    let count = gcmd.get_int_bounded("COUNT", Some(0), None)?;
    let sdcard = object.sdcard()?;
    if !object.loop_begin(&*sdcard, count) {
        return Err(CommandError::new("Only permitted in SD file."));
    }
    Ok(())
}

/// `SDCARD_LOOP_END` (`sdcard_loop.py:30-33`).
fn cmd_sdcard_loop_end(object: &Arc<SDCardLoop>, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let sdcard = object.sdcard()?;
    if !object.loop_end(&*sdcard) {
        return Err(CommandError::new("Only permitted in SD file."));
    }
    Ok(())
}

/// `SDCARD_LOOP_DESIST` (`sdcard_loop.py:36-39`).
fn cmd_sdcard_loop_desist(
    object: &Arc<SDCardLoop>,
    _gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let sdcard = object.sdcard()?;
    if !object.loop_desist(&*sdcard) {
        return Err(CommandError::new("Only permitted outside of a SD file."));
    }
    Ok(())
}

/// A command handler: the object plus the command it was given.
type CommandFn = fn(&Arc<SDCardLoop>, &GcodeCommand) -> Result<(), CommandError>;

/// The three registrations (`sdcard_loop.py:14-22`), with the parameter
/// tables upstream's `get_int`/`get` calls declare.
const COMMANDS: &[(&str, CommandFn, &str, &[&str])] = &[
    (
        "SDCARD_LOOP_BEGIN",
        cmd_sdcard_loop_begin,
        SDCARD_LOOP_BEGIN_HELP,
        &["COUNT"],
    ),
    (
        "SDCARD_LOOP_END",
        cmd_sdcard_loop_end,
        SDCARD_LOOP_END_HELP,
        &[],
    ),
    (
        "SDCARD_LOOP_DESIST",
        cmd_sdcard_loop_desist,
        SDCARD_LOOP_DESIST_HELP,
        &[],
    ),
];

/// The factory `section!` names (`sdcard_loop.py:72 def load_config`), which
/// also registers the three commands (`sdcard_loop.py:11-22`).
///
/// # Errors
/// A printer without a g-code dispatcher, or a command name already taken.
pub fn load_config(
    _config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let gcode = printer
        .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
        .ok_or_else(|| ConfigError::new("the g-code dispatcher is not registered".to_string()))?;
    let object = Arc::new(SDCardLoop::new(printer));
    for &(name, command, desc, params) in COMMANDS {
        let handler: CommandHandler = {
            // Weak: this object keeps the machine, the machine's dispatcher
            // keeps this table, so a strong capture here would close the loop
            // and keep the whole printer alive (`virtual_sdcard` holds its
            // handlers the same way).
            let object = Arc::downgrade(&object);
            sync(move |gcmd| {
                let Some(object) = object.upgrade() else {
                    return Err(CommandError::new("the sdcard_loop object is gone"));
                };
                command(&object, gcmd)
            })
        };
        gcode
            .register_command_with_params(name, handler, Some(desc), params, false)
            .map_err(ConfigError::new)?;
    }
    Ok(object as Arc<dyn PrinterObject>)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::reactor::ManualReactor;

    /// A stand-in file position: whether the command came from the SD file,
    /// and where the read cursor sits.
    struct FakeSdCard {
        from_sd: bool,
        position: Mutex<u64>,
    }

    impl FakeSdCard {
        fn new(from_sd: bool, position: u64) -> Self {
            Self {
                from_sd,
                position: Mutex::new(position),
            }
        }

        fn position(&self) -> u64 {
            *self.position.lock().unwrap()
        }

        fn set_from_sd(&mut self, from_sd: bool) {
            self.from_sd = from_sd;
        }
    }

    impl SdCardFile for FakeSdCard {
        fn is_cmd_from_sd(&self) -> bool {
            self.from_sd
        }
        fn file_position(&self) -> u64 {
            self.position()
        }
        fn set_file_position(&self, position: u64) {
            *self.position.lock().unwrap() = position;
        }
    }

    /// Load `text` into a fresh machine and bring it to ready, as the host
    /// does — commands only run, and only appear in the status, once ready.
    fn load(text: &str) -> Arc<Printer> {
        let (config, _) = Config::from_text(text).expect("the config parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer.load_config(&config).expect("the config loads");
        printer.send_event(&KlippyEvent::KlippyReady);
        printer
    }

    /// The bare `[sdcard_loop]` section loads with no options to read —
    /// upstream's class reads none either (`sdcard_loop.py:11-26`).
    #[test]
    fn the_bare_section_loads_and_reports_no_status() {
        let printer = load("[sdcard_loop]\n");
        let object = printer
            .lookup_object_as::<SDCardLoop>("sdcard_loop")
            .expect("the section registers the object");
        assert_eq!(object.get_status(0.0), json!({}));
    }

    /// The three commands are registered with upstream's help text and
    /// parameter table (`sdcard_loop.py:14-22`).
    #[test]
    fn the_three_commands_are_registered() {
        let printer = load("[sdcard_loop]\n");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        let status = gcode.get_status(0.0);
        let commands = &status["commands"];

        assert_eq!(
            commands["SDCARD_LOOP_BEGIN"]["help"].as_str(),
            Some("Begins a looped section in the SD file.")
        );
        assert_eq!(
            commands["SDCARD_LOOP_BEGIN"]["parameters"],
            json!(["COUNT"]),
            "`COUNT` is the only parameter"
        );
        assert_eq!(
            commands["SDCARD_LOOP_END"]["help"].as_str(),
            Some("Ends a looped section in the SD file.")
        );
        assert!(
            commands["SDCARD_LOOP_END"].get("parameters").is_none(),
            "`_END` declares no parameter"
        );
        assert_eq!(
            commands["SDCARD_LOOP_DESIST"]["help"].as_str(),
            Some("Stops iterating the current loop stack.")
        );
        assert!(
            commands["SDCARD_LOOP_DESIST"].get("parameters").is_none(),
            "`_DESIST` declares no parameter"
        );
    }

    /// `COUNT` is required and never negative — upstream
    /// `gcmd.get_int("COUNT", minval=0)` passes no default
    /// (`sdcard_loop.py:26`; `gcode.py:91-92`), so the argument is parsed
    /// before the SD-file context is looked at.
    #[test]
    fn count_is_required_and_never_negative() {
        let printer = load("[sdcard_loop]\n");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");

        let err = gcode
            .run_script_sync("SDCARD_LOOP_BEGIN")
            .expect_err("COUNT has no default");
        assert_eq!(
            err.to_string(),
            "Error on 'SDCARD_LOOP_BEGIN': missing COUNT"
        );

        let err = gcode
            .run_script_sync("SDCARD_LOOP_BEGIN COUNT=-1")
            .expect_err("a repeat count below zero is refused");
        assert_eq!(
            err.to_string(),
            "Error on 'SDCARD_LOOP_BEGIN COUNT=-1': COUNT must have minimum of 0"
        );
    }

    /// Without a `[virtual_sdcard]` section the commands report upstream's
    /// lookup error rather than silently doing nothing (`sdcard_loop.py:13`).
    #[test]
    fn a_missing_virtual_sdcard_is_reported() {
        let printer = load("[sdcard_loop]\n");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        let err = gcode
            .run_script_sync("SDCARD_LOOP_DESIST")
            .expect_err("no virtual_sdcard to desist");
        assert_eq!(err.to_string(), "Unknown config object 'virtual_sdcard'");
    }

    /// BEGIN/END only work inside the SD file: outside, both refuse (the
    /// caller raises `Only permitted in SD file.`) and the stack is left
    /// alone (`sdcard_loop.py:38-46`).
    #[test]
    fn begin_and_end_only_work_inside_the_sd_file() {
        let loop_ = SDCardLoop::new(&Arc::new(Printer::new(ManualReactor::shared())));
        let mut sd = FakeSdCard::new(false, 64);

        assert!(!loop_.loop_begin(&sd, 3));
        assert!(!loop_.loop_end(&sd));
        assert!(loop_.stack().is_empty());

        sd.set_from_sd(true);
        assert!(loop_.loop_begin(&sd, 3));
        assert_eq!(loop_.stack(), [(3, 64)]);
    }

    /// END's count semantics (`sdcard_loop.py:50-62`): `0` repeats
    /// forever (seek back, entry kept), `1` is the last repeat (pop, no
    /// seek), anything higher seeks back and decrements.
    #[test]
    fn end_repeats_per_the_upstream_count_semantics() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));

        // count == 0: infinite — seek back, keep the entry.
        let loop_ = SDCardLoop::new(&printer);
        let sd = FakeSdCard::new(true, 100);
        assert!(loop_.loop_begin(&sd, 0));
        sd.set_file_position(140); // the body of the loop ran on
        assert!(loop_.loop_end(&sd));
        assert_eq!(sd.position(), 100, "the infinite loop seeks back");
        assert_eq!(loop_.stack(), [(0, 100)]);

        // count == 1: last repeat — a fresh loop pops without seeking.
        let loop_ = SDCardLoop::new(&printer);
        let sd = FakeSdCard::new(true, 100);
        assert!(loop_.loop_begin(&sd, 1));
        sd.set_file_position(140);
        assert!(loop_.loop_end(&sd));
        assert_eq!(sd.position(), 140, "the last repeat does not seek");
        assert!(loop_.stack().is_empty());

        // count > 1: seek back and decrement.
        let loop_ = SDCardLoop::new(&printer);
        let mut sd = FakeSdCard::new(true, 100);
        assert!(loop_.loop_begin(&sd, 3));
        sd.set_file_position(140);
        assert!(loop_.loop_end(&sd));
        assert_eq!(sd.position(), 100, "a pending repeat seeks back");
        assert_eq!(loop_.stack(), [(2, 100)]);

        // END outside the SD file refuses (the caller raises
        // `Only permitted in SD file.`).
        sd.set_from_sd(false);
        assert!(!loop_.loop_end(&sd));
        assert_eq!(loop_.stack(), [(2, 100)], "the stack is untouched");
    }

    /// END with an empty stack succeeds without seeking, and nested loops
    /// unwind outermost-last (`sdcard_loop.py:47-51`).
    #[test]
    fn end_with_an_empty_stack_seeks_nothing_and_nests_last() {
        let loop_ = SDCardLoop::new(&Arc::new(Printer::new(ManualReactor::shared())));
        let sd = FakeSdCard::new(true, 10);
        assert!(loop_.loop_end(&sd), "an empty stack is a no-op");
        assert_eq!(sd.position(), 10);

        // Outer at 10, inner at 50; the inner end pops first.
        assert!(loop_.loop_begin(&sd, 2));
        sd.set_file_position(50);
        assert!(loop_.loop_begin(&sd, 2));
        assert_eq!(loop_.stack(), [(2, 10), (2, 50)]);
        sd.set_file_position(60);
        assert!(loop_.loop_end(&sd));
        assert_eq!(sd.position(), 50, "the inner loop seeks to its own start");
        assert_eq!(loop_.stack(), [(2, 10), (1, 50)]);
    }

    /// DESIST only works outside the SD file: inside, it refuses (the
    /// caller raises `Only permitted outside of a SD file.`) and keeps the
    /// stack; outside, it clears every open loop
    /// (`sdcard_loop.py:34-36, 64-70`).
    #[test]
    fn desist_clears_the_stack_only_outside_the_sd_file() {
        let loop_ = SDCardLoop::new(&Arc::new(Printer::new(ManualReactor::shared())));
        let mut sd = FakeSdCard::new(true, 10);
        assert!(loop_.loop_begin(&sd, 3));

        assert!(!loop_.loop_desist(&sd));
        assert_eq!(loop_.stack(), [(3, 10)], "the stack survives");

        sd.set_from_sd(false);
        assert!(loop_.loop_desist(&sd));
        assert!(loop_.stack().is_empty());

        // A cleared stack no longer repeats: the next END inside the file
        // has nothing to pop and seeks nowhere.
        sd.set_from_sd(true);
        sd.set_file_position(40);
        assert!(loop_.loop_end(&sd));
        assert_eq!(sd.position(), 40, "the loop is gone, the cursor stays");
    }
}
