//! `[pause_resume]` — pause/resume with position capture/restore
//! (upstream `klippy/extras/pause_resume.py`).
//!
//! The section reads one option, `recover_velocity` (default `50.`), and
//! registers the four commands upstream's `__init__` does
//! (`pause_resume.py:33-46`):
//!
//! | command | role (`pause_resume.py`) |
//! |---|---|
//! | `PAUSE` | park the g-code state and mark the print paused (`:60-66`) |
//! | `RESUME` | move the state back and mark it running (`:68-76`) |
//! | `CLEAR_PAUSE` | forget the paused state without resuming (`:79-81`) |
//! | `CANCEL_PRINT` | cancel the SD print or report `action:cancel` (`:84-89`) |
//!
//! [`PauseResume`] is the module object upstream loads as `pause_resume`; the
//! filament sensors reach it through [`PauseResume::ensure`]
//! (`printer.load_object(config, 'pause_resume')`) and call
//! [`PauseResume::send_pause_command`] on a runout
//! (`filament_switch_sensor.py:48-53`).
//!
//! While a file replay runs, the four commands take upstream's SD branches:
//! the file itself is paused, resumed or cancelled and no `action:*` line is
//! reported; with no replay running they answer through `respond_info`
//! (`pause_resume.py:45-46,47-59,68-75,92-97`).
//!
//! # What is not here
//!
//! - **`do_pause` now waits** for the replay task to exit, matching
//!   upstream's `virtual_sdcard.do_pause` spin (`virtual_sdcard.py:123-127`).
//!   The `SdCard` seam's `do_pause`/`do_cancel` return boxed futures so the
//!   wait is async; `send_pause_command` and `cancel_sd_print` are async to
//!   propagate it. The `cmd_from_sd` guard skips the wait when the pause is
//!   called from a replayed line (upstream `not self.cmd_from_sd`).
//! - **The three webhooks endpoints** (`pause_resume/cancel|pause|resume`,
//!   `pause_resume.py:26-32`) live on the API side, in
//!   `api/endpoints/pause_resume.rs`: each one runs the matching command
//!   (`CANCEL_PRINT`/`PAUSE`/`RESUME`) through the dispatcher, looked up per
//!   request because the endpoints are installed before this object is built.

use std::future::Future;
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::virtual_sdcard::VirtualSdCard;
use crate::core::klippy::gcode::{
    CommandError, CommandFuture, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The name the sensors look the object up by (`load_object(config,
/// 'pause_resume')`), which is also the section id.
pub const PAUSE_RESUME_OBJECT: &str = "pause_resume";

/// The object upstream holds at connect for `is_sd_active`
/// (`pause_resume.py:33-34`).
const VIRTUAL_SDCARD_OBJECT: &str = "virtual_sdcard";

section!("pause_resume", order = 30, load = load_config);

/// What `pause_resume` needs from `virtual_sdcard`
/// (`pause_resume.py:45-46,47-59,68-75,92-97`).
///
/// Two implementors: `RegisteredSdCard`, which the connect handler builds
/// from this port's registered `[virtual_sdcard]` object and which drives the
/// real replay primitives, and a test stand-in — the same shape as
/// `sdcard_loop.rs`'s `SdCardFile`.
pub trait SdCard: Send + Sync {
    /// `is_active` (`pause_resume.py:46`): a file replay is running.
    fn is_active(&self) -> bool;
    /// `do_pause`: stop the replay (`pause_resume.py:55`). Returns a boxed
    /// future because `do_pause` now waits for the replay task to exit
    /// (upstream `virtual_sdcard.py:123-127`).
    fn do_pause(&self) -> Pin<Box<dyn Future<Output = ()> + Send + '_>>;
    /// `do_resume`: continue the replay; `Err("SD busy")` while one is still
    /// running (`pause_resume.py:71`, `virtual_sdcard.py:128-133`).
    fn do_resume(&self) -> Result<(), CommandError>;
    /// `do_cancel`: cancel the running print (`pause_resume.py:94`). Returns
    /// a boxed future because `do_cancel` waits for the replay task to exit
    /// via `do_pause`.
    fn do_cancel(&self) -> Pin<Box<dyn Future<Output = ()> + Send + '_>>;
}

/// The registered `virtual_sdcard` object behind the [`SdCard`] seam — the
/// production implementor, holding this port's real replay object
/// (`extras/virtual_sdcard.rs`) and forwarding to its primitives.
struct RegisteredSdCard(Arc<VirtualSdCard>);

impl SdCard for RegisteredSdCard {
    fn is_active(&self) -> bool {
        self.0.is_active()
    }

    fn do_pause(&self) -> Pin<Box<dyn Future<Output = ()> + Send + '_>> {
        Box::pin(self.0.do_pause())
    }

    fn do_resume(&self) -> Result<(), CommandError> {
        self.0.do_resume()
    }

    fn do_cancel(&self) -> Pin<Box<dyn Future<Output = ()> + Send + '_>> {
        Box::pin(self.0.do_cancel())
    }
}

/// The `[pause_resume]` module object (upstream's `PauseResume`).
pub struct PauseResume {
    /// `is_paused`: set by `PAUSE`, cleared by `RESUME`/`CLEAR_PAUSE`.
    is_paused: AtomicBool,
    /// `sd_paused`: set when the pause went to the SD file, so the resume
    /// knows to continue it (`pause_resume.py:18,50,60`).
    sd_paused: AtomicBool,
    /// `pause_command_sent`: upstream's guard so a runout does not pause twice
    /// (`pause_resume.py:19,49`).
    pause_command_sent: AtomicBool,
    /// `recover_velocity`: the default `RESUME` move speed
    /// (`pause_resume.py:9,71`).
    recover_velocity: f64,
    /// The machine, to report the `action:*` lines and, at connect, to find
    /// the SD object.
    printer: Option<Weak<Printer>>,
    /// The SD replay object upstream holds for `is_sd_active`, behind the
    /// [`SdCard`] seam (module docs).
    v_sd: Mutex<Option<Arc<dyn SdCard>>>,
}

impl PauseResume {
    /// Read the section upstream's `__init__` reads
    /// (`pause_resume.py:7-9`).
    ///
    /// # Errors
    /// An unparsable `recover_velocity`.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let recover_velocity = config.get_float("recover_velocity", Some(50.0))?;
        Ok(Self {
            is_paused: AtomicBool::new(false),
            sd_paused: AtomicBool::new(false),
            pause_command_sent: AtomicBool::new(false),
            recover_velocity,
            printer: Some(Arc::downgrade(printer)),
            v_sd: Mutex::new(None),
        })
    }

    /// The single `pause_resume` object; the first caller creates it, as
    /// upstream's `printer.load_object(config, 'pause_resume')` does.
    ///
    /// Reads `recover_velocity` from whichever config triggered the load:
    /// upstream's `load_object` passes the caller's config to the new module's
    /// `load_config`, and `PauseResume.__init__` reads `recover_velocity` from
    /// it (`pause_resume.py:9`).
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
        let object = Arc::new(PauseResume::new(config, printer)?);
        object.register_commands(printer)?;
        object.register_handlers(printer);
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

    /// Upstream's `is_sd_active` (`pause_resume.py:45-46`): a file is being
    /// replayed from `virtual_sdcard`, read from the real replay state
    /// through the [`SdCard`] seam.
    fn is_sd_active(&self) -> bool {
        self.v_sd().is_some_and(|sd| sd.is_active())
    }

    /// Upstream's `send_pause_command` (`pause_resume.py:47-59`): pause from
    /// inside an event, once. With a replay running the file is paused and
    /// nothing is reported; otherwise this is the
    /// `respond_info("action:paused")` branch.
    ///
    /// Async because `do_pause` now waits for the replay task to exit
    /// (upstream `virtual_sdcard.py:123-127`).
    pub async fn send_pause_command(&self) {
        if self.pause_command_sent.load(Ordering::SeqCst) {
            return;
        }
        if self.is_sd_active() {
            self.sd_paused.store(true, Ordering::SeqCst);
            if let Some(sd) = self.v_sd() {
                sd.do_pause().await;
            }
        } else {
            self.sd_paused.store(false, Ordering::SeqCst);
            self.respond_info("action:paused");
        }
        self.pause_command_sent.store(true, Ordering::SeqCst);
    }

    /// Upstream's `send_resume_command` (`pause_resume.py:68-75`): continue
    /// the replay — one still winding down is refused with `SD busy`, which
    /// surfaces out of `RESUME` as upstream's exception does
    /// (`virtual_sdcard.py:128-133`) — or report `action:resumed`, and arm
    /// the next pause.
    fn send_resume_command(&self) -> Result<(), CommandError> {
        if self.sd_paused.load(Ordering::SeqCst) {
            if let Some(sd) = self.v_sd() {
                sd.do_resume()?;
            }
            self.sd_paused.store(false, Ordering::SeqCst);
        } else {
            self.respond_info("action:resumed");
        }
        self.pause_command_sent.store(false, Ordering::SeqCst);
        Ok(())
    }

    /// Upstream's `cmd_CLEAR_PAUSE` body (`pause_resume.py:79-81`): forget
    /// the paused and the command-sent state. `sd_paused` is left as it is,
    /// as upstream leaves it.
    fn clear_pause(&self) {
        self.is_paused.store(false, Ordering::SeqCst);
        self.pause_command_sent.store(false, Ordering::SeqCst);
    }

    /// The SD side of `CANCEL_PRINT` (`pause_resume.py:93-94`): cancel the
    /// running file. A no-op without an SD object — the only caller reaches it
    /// through `is_sd_active`/`sd_paused`, which need one.
    ///
    /// Async because `do_cancel` waits for the replay task to exit via
    /// `do_pause`.
    async fn cancel_sd_print(&self) {
        if let Some(sd) = self.v_sd() {
            sd.do_cancel().await;
        }
    }

    /// Upstream's `handle_connect` (`pause_resume.py:33-34`): hold the
    /// `virtual_sdcard` object for `is_sd_active`.
    fn handle_connect(&self) {
        let Some(printer) = self.printer.as_ref().and_then(Weak::upgrade) else {
            return;
        };
        let Some(sd) = printer.lookup_object_as::<VirtualSdCard>(VIRTUAL_SDCARD_OBJECT) else {
            return;
        };
        *self.lock() = Some(Arc::new(RegisteredSdCard(sd)));
    }

    /// The SD seam, copied out of its lock.
    fn v_sd(&self) -> Option<Arc<dyn SdCard>> {
        self.lock().clone()
    }

    /// The `v_sd` lock, poisoning treated as continued unwinding
    /// (`gcode_move.rs` convention).
    fn lock(&self) -> MutexGuard<'_, Option<Arc<dyn SdCard>>> {
        self.v_sd
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The dispatcher, for the `action:*` reports and `run_script_from_command`
    /// (upstream keeps `self.gcode` from `config.get_printer()`,
    /// `pause_resume.py:8`).
    fn gcode(&self) -> Option<Arc<GCodeDispatch>> {
        self.printer
            .as_ref()
            .and_then(Weak::upgrade)
            .and_then(|printer| printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT))
    }

    /// Upstream's `self.gcode.respond_info` (logged, as upstream's default is).
    fn respond_info(&self, message: &str) {
        if let Some(gcode) = self.gcode() {
            gcode.respond_info(message, true);
        }
    }

    /// Register the four commands, capturing a handle to this object.
    fn register_commands(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        /// The one word `cmd_RESUME` reads (`pause_resume.py:68-76`); the
        /// other three commands read none.
        const RESUME_PARAMS: &[&str] = &["VELOCITY"];
        type Command = for<'a> fn(&'a Arc<PauseResume>, &'a GcodeCommand) -> CommandFuture<'a>;
        const COMMANDS: &[(&str, Command, &str, &[&str])] = &[
            ("PAUSE", cmd_pause, "Pauses the current print", &[]),
            (
                "RESUME",
                cmd_resume,
                "Resumes the print from a pause",
                RESUME_PARAMS,
            ),
            (
                "CLEAR_PAUSE",
                cmd_clear_pause,
                "Clears the current paused state without resuming the print",
                &[],
            ),
            (
                "CANCEL_PRINT",
                cmd_cancel_print,
                "Cancel the current print",
                &[],
            ),
        ];
        for &(name, command, help, params) in COMMANDS {
            let handler: CommandHandler = {
                let object = Arc::clone(self);
                Arc::new(move |gcmd| {
                    let object = Arc::clone(&object);
                    Box::pin(async move { command(&object, gcmd).await })
                })
            };
            gcode
                .register_command_with_params(name, handler, Some(help), params, false)
                .map_err(ConfigError::new)?;
        }
        Ok(())
    }

    /// The `klippy:connect` handler upstream registers
    /// (`pause_resume.py:27-28`).
    fn register_handlers(self: &Arc<Self>, printer: &Arc<Printer>) {
        printer.register_event_handler(
            KlippyEvent::KlippyConnect,
            Box::new({
                let object = Arc::clone(self);
                move |_| object.handle_connect()
            }),
        );
    }
}

impl std::fmt::Debug for PauseResume {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PauseResume")
            .field("is_paused", &self.is_paused())
            .field("sd_paused", &self.sd_paused.load(Ordering::SeqCst))
            .field("recover_velocity", &self.recover_velocity)
            .finish()
    }
}

impl PrinterObject for PauseResume {
    /// Upstream's `PauseResume.get_status` (`pause_resume.py:41-44`).
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "is_paused": self.is_paused() })
    }
}

/// The factory `section!` names (`pause_resume.py:91-92`).
///
/// # Errors
/// An unparsable `recover_velocity`, an already-registered object, or a
/// command name this dispatcher refuses.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    // A `[filament_switch_sensor]` normally loads after this main section, but
    // `ensure` may have created the object first; reuse it either way. The
    // loader registers whatever this returns under `pause_resume`.
    if let Some(existing) = printer.lookup_object_as::<PauseResume>(PAUSE_RESUME_OBJECT) {
        return Ok(existing as Arc<dyn PrinterObject>);
    }
    let object = Arc::new(PauseResume::new(config, printer)?);
    object.register_commands(printer)?;
    object.register_handlers(printer);
    Ok(object as Arc<dyn PrinterObject>)
}

/// `PAUSE` (`pause_resume.py:60-66`): the g-code state is parked under
/// `PAUSE_STATE` and the paused flag goes up. An already-paused print is
/// reported and left alone.
fn cmd_pause<'a>(object: &'a Arc<PauseResume>, gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
    Box::pin(async move {
        if object.is_paused() {
            gcmd.respond_info("Print already paused");
            return Ok(());
        }
        object.send_pause_command().await;
        object
            .gcode()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?
            .run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE_STATE")
            .await?;
        object.is_paused.store(true, Ordering::SeqCst);
        Ok(())
    })
}

/// `RESUME` (`pause_resume.py:68-76`): move the state back at `VELOCITY`
/// (default `recover_velocity`) and mark the print running. A print that is
/// not paused is reported and left alone.
fn cmd_resume<'a>(object: &'a Arc<PauseResume>, gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
    Box::pin(async move {
        if !object.is_paused() {
            gcmd.respond_info("Print is not paused, resume aborted");
            return Ok(());
        }
        let velocity = gcmd.get_float_default("VELOCITY", object.recover_velocity)?;
        object
            .gcode()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?
            .run_script_from_command(&format!(
                "RESTORE_GCODE_STATE NAME=PAUSE_STATE MOVE=1 MOVE_SPEED={velocity:.4}"
            ))
            .await?;
        object.send_resume_command()?;
        object.is_paused.store(false, Ordering::SeqCst);
        Ok(())
    })
}

/// `CLEAR_PAUSE` (`pause_resume.py:79-81`): drop the paused state without
/// resuming the print.
fn cmd_clear_pause<'a>(object: &'a Arc<PauseResume>, _gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
    Box::pin(async move {
        object.clear_pause();
        Ok(())
    })
}

/// `CANCEL_PRINT` (`pause_resume.py:84-89`): cancel the SD print, or report
/// `action:cancel`, then clear the paused state.
fn cmd_cancel_print<'a>(object: &'a Arc<PauseResume>, gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
    Box::pin(async move {
        if object.is_sd_active() || object.sd_paused.load(Ordering::SeqCst) {
            object.cancel_sd_print().await;
        } else {
            gcmd.respond_info("action:cancel");
        }
        object.clear_pause();
        Ok(())
    })
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::extras::gcode_move::{self, MoveTarget};
    use crate::core::klippy::extras::print_stats::{PrintStats, PRINT_STATS_OBJECT};
    use crate::core::klippy::mathutil::Coord;
    use crate::core::klippy::reactor::{ManualReactor, Reactor};
    use std::path::{Path, PathBuf};
    use std::sync::atomic::AtomicUsize;

    /// A printer with a section loaded and the ready lamp lit, as the loader
    /// leaves it. `KlippyReady` is what makes the four commands reachable
    /// (they are not `when_not_ready` commands).
    fn loaded(text: &str) -> Arc<Printer> {
        let (config, _) = Config::from_text(text).expect("the config parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer.load_config(&config).expect("the config loads");
        printer.send_event(&KlippyEvent::KlippyReady);
        printer
    }

    /// A printer with `pause_resume` and `gcode_move` (so `PAUSE`/`RESUME` can
    /// reach `SAVE_GCODE_STATE`/`RESTORE_GCODE_STATE`) and a move target, so
    /// the `RESUME` move-back completes.
    fn machine() -> (Arc<Printer>, Arc<GCodeDispatch>, Arc<PauseResume>) {
        let printer = loaded("[pause_resume]\n");
        let gcode_move = gcode_move::ensure(&printer).expect("gcode_move registers");
        gcode_move
            .set_move_transform(Arc::new(FakeTarget), true)
            .expect("the slot is free");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the dispatcher is registered");
        let object = printer
            .lookup_object_as::<PauseResume>(PAUSE_RESUME_OBJECT)
            .expect("the section registered the object");
        (printer, gcode, object)
    }

    /// A move target that stands still — enough for the `RESUME` move-back.
    struct FakeTarget;

    impl MoveTarget for FakeTarget {
        fn move_to(&self, _position: Coord, _speed: f64) -> Result<(), CommandError> {
            Ok(())
        }

        fn position(&self) -> Coord {
            Coord::default()
        }
    }

    /// An SD stand-in that records the control calls.
    #[derive(Default)]
    struct FakeSd {
        active: bool,
        pauses: AtomicUsize,
        resumes: AtomicUsize,
        cancels: AtomicUsize,
    }

    impl SdCard for FakeSd {
        fn is_active(&self) -> bool {
            self.active
        }
        fn do_pause(&self) -> Pin<Box<dyn Future<Output = ()> + Send + '_>> {
            self.pauses.fetch_add(1, Ordering::SeqCst);
            Box::pin(async {})
        }
        fn do_resume(&self) -> Result<(), CommandError> {
            self.resumes.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }
        fn do_cancel(&self) -> Pin<Box<dyn Future<Output = ()> + Send + '_>> {
            self.cancels.fetch_add(1, Ordering::SeqCst);
            Box::pin(async {})
        }
    }

    /// Everything `gcode` reported through `respond_info`, one entry per line
    /// (`// …` prefixes included, as a client sees them).
    fn captured_lines(printer: &Arc<Printer>) -> Arc<Mutex<Vec<String>>> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        let lines = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&lines);
        gcode.register_output_handler(Arc::new(move |line: &str| {
            sink.lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(line.to_string());
        }));
        lines
    }

    fn emitted(lines: &Arc<Mutex<Vec<String>>>) -> Vec<String> {
        lines.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }

    /// The section reads `recover_velocity` and registers the object under its
    /// own name (`pause_resume.py:7-9,91-92`).
    #[test]
    fn the_section_loads_and_reads_recover_velocity() {
        let printer = loaded("[pause_resume]\nrecover_velocity: 25\n");
        let object = printer
            .lookup_object_as::<PauseResume>(PAUSE_RESUME_OBJECT)
            .expect("the section registered the object");
        assert_eq!(object.recover_velocity, 25.0);
        assert_eq!(object.get_status(0.0), json!({ "is_paused": false }));
    }

    /// The default is upstream's `50.` (`pause_resume.py:9`); an unparsable
    /// value is the loader's wording for a bad float.
    #[test]
    fn recover_velocity_defaults_to_50_and_rejects_garbage() {
        let printer = loaded("[pause_resume]\n");
        let object = printer
            .lookup_object_as::<PauseResume>(PAUSE_RESUME_OBJECT)
            .expect("the section registered the object");
        assert_eq!(object.recover_velocity, 50.0);

        let (config, _) = Config::from_text("[pause_resume]\nrecover_velocity: nope\n")
            .expect("the section parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let error = printer
            .load_config(&config)
            .expect_err("a bad float is refused");
        assert_eq!(
            error.to_string(),
            "Unable to parse option 'recover_velocity' in section 'pause_resume'"
        );
    }

    /// `ensure` reads `recover_velocity` from whichever config triggered the
    /// load and shares one object across callers (`pause_resume.py:9`).
    #[test]
    fn ensure_reads_recover_velocity_and_shares_one_object() {
        let (_, _gcode, object) = machine();
        assert!(!object.is_paused());
        // A second `ensure` (a sensor's `load_object`) reuses the object.
        let (config, _) = Config::from_text("[filament_switch_sensor s]\nswitch_pin: PA0\n")
            .expect("the section parses");
        let section = config
            .get_section("filament_switch_sensor s")
            .expect("the section")
            .clone();
        let wrapper = ConfigWrapper::untracked(&section);
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                PAUSE_RESUME_OBJECT,
                Arc::clone(&object) as Arc<dyn PrinterObject>,
            )
            .expect("the object is free");
        let again = PauseResume::ensure(&wrapper, &printer).expect("the object is reused");
        assert!(Arc::ptr_eq(&object, &again));
    }

    /// The four commands register with upstream's help text — a duplicate or
    /// invalid name would already have failed the load
    /// (`pause_resume.py:29-46`).
    #[test]
    fn the_four_commands_are_registered_with_upstreams_help_text() {
        let (_printer, gcode, _object) = machine();
        let help = gcode.command_help();
        assert_eq!(
            help.get("PAUSE").map(String::as_str),
            Some("Pauses the current print")
        );
        assert_eq!(
            help.get("RESUME").map(String::as_str),
            Some("Resumes the print from a pause")
        );
        assert_eq!(
            help.get("CLEAR_PAUSE").map(String::as_str),
            Some("Clears the current paused state without resuming the print")
        );
        assert_eq!(
            help.get("CANCEL_PRINT").map(String::as_str),
            Some("Cancel the current print")
        );
    }

    /// The state machine's two reports per command, and the `PAUSE_STATE`
    /// save/restore the two real commands do (`pause_resume.py:60-76`).
    #[test]
    fn pause_and_resume_report_once_and_track_the_paused_state() {
        let (printer, gcode, object) = machine();
        let lines = captured_lines(&printer);

        gcode.run_script_sync("PAUSE").unwrap();
        assert_eq!(emitted(&lines), ["// action:paused"]);
        assert!(object.is_paused());

        gcode.run_script_sync("PAUSE").unwrap();
        assert_eq!(
            emitted(&lines),
            ["// action:paused", "// Print already paused"]
        );

        // `RESUME` restores the parked state, so `PAUSE` must have saved one.
        gcode.run_script_sync("RESUME").unwrap();
        assert_eq!(
            emitted(&lines),
            [
                "// action:paused",
                "// Print already paused",
                "// action:resumed"
            ]
        );
        assert!(!object.is_paused());

        gcode.run_script_sync("RESUME").unwrap();
        assert_eq!(
            emitted(&lines),
            [
                "// action:paused",
                "// Print already paused",
                "// action:resumed",
                "// Print is not paused, resume aborted"
            ]
        );
    }

    /// `RESUME`'s default move speed is `recover_velocity`, and `VELOCITY`
    /// overrides it (`pause_resume.py:71`).
    #[test]
    fn resume_uses_recover_velocity_or_the_velocity_override() {
        let printer = loaded("[pause_resume]\nrecover_velocity: 25\n");
        gcode_move::ensure(&printer)
            .expect("gcode_move registers")
            .set_move_transform(Arc::new(FakeTarget), true)
            .expect("the slot is free");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the dispatcher");
        let object = printer
            .lookup_object_as::<PauseResume>(PAUSE_RESUME_OBJECT)
            .expect("the object");

        // `VELOCITY=10` is accepted (a bad value would be a parse error) and
        // the move-back runs; the state ends running either way.
        gcode.run_script_sync("PAUSE").unwrap();
        gcode.run_script_sync("RESUME VELOCITY=10").unwrap();
        assert!(!object.is_paused());

        gcode.run_script_sync("PAUSE").unwrap();
        gcode.run_script_sync("RESUME").unwrap();
        assert!(!object.is_paused());
    }

    /// `CLEAR_PAUSE` drops the paused and command-sent state without resuming
    /// (`pause_resume.py:79-81`).
    #[test]
    fn clear_pause_forgets_the_pause() {
        let (printer, gcode, object) = machine();
        gcode.run_script_sync("PAUSE").unwrap();
        assert!(object.is_paused());
        assert!(object.pause_command_sent.load(Ordering::SeqCst));

        gcode.run_script_sync("CLEAR_PAUSE").unwrap();
        assert!(!object.is_paused());
        assert!(!object.pause_command_sent.load(Ordering::SeqCst));
        drop(printer);
    }

    /// `CANCEL_PRINT` without an active SD print reports `action:cancel` and
    /// clears the paused state (`pause_resume.py:84-89`).
    #[test]
    fn cancel_print_without_an_sd_print_reports_action_cancel() {
        let (printer, gcode, object) = machine();
        let lines = captured_lines(&printer);

        gcode.run_script_sync("PAUSE").unwrap();
        gcode.run_script_sync("CANCEL_PRINT").unwrap();
        assert_eq!(emitted(&lines), ["// action:paused", "// action:cancel"]);
        assert!(!object.is_paused());
        assert!(!object.pause_command_sent.load(Ordering::SeqCst));
    }

    /// With an active SD print the cancel goes to the file and no
    /// `action:cancel` line is emitted (`pause_resume.py:86-87`).
    #[test]
    fn cancel_print_with_an_active_sd_print_cancels_the_file() {
        let (printer, gcode, object) = machine();
        let lines = captured_lines(&printer);
        let sd = Arc::new(FakeSd {
            active: true,
            ..Default::default()
        });
        *object.lock() = Some(Arc::clone(&sd) as Arc<dyn SdCard>);

        gcode.run_script_sync("CANCEL_PRINT").unwrap();
        assert_eq!(sd.cancels.load(Ordering::SeqCst), 1);
        assert!(emitted(&lines).is_empty(), "no `action:cancel` line");
        assert!(!object.is_paused());
    }

    /// The SD branches of the pause/resume helpers
    /// (`pause_resume.py:49-56,57-65`): the file is told, and the
    /// `action:*` reports are skipped.
    #[tokio::test]
    async fn an_active_sd_print_is_paused_and_resumed_through_the_file() {
        let (printer, gcode, object) = machine();
        let lines = captured_lines(&printer);
        let sd = Arc::new(FakeSd {
            active: true,
            ..Default::default()
        });
        *object.lock() = Some(Arc::clone(&sd) as Arc<dyn SdCard>);

        object.send_pause_command().await;
        assert_eq!(sd.pauses.load(Ordering::SeqCst), 1);
        assert!(object.sd_paused.load(Ordering::SeqCst));
        assert!(emitted(&lines).is_empty(), "no `action:paused` line");

        object.send_resume_command().expect("the SD resume runs");
        assert_eq!(sd.resumes.load(Ordering::SeqCst), 1);
        assert!(!object.sd_paused.load(Ordering::SeqCst));
        assert!(emitted(&lines).is_empty(), "no `action:resumed` line");

        // The guard still makes a second pause a no-op.
        object.pause_command_sent.store(true, Ordering::SeqCst);
        object.send_pause_command().await;
        assert_eq!(sd.pauses.load(Ordering::SeqCst), 1);
        drop(gcode);
    }

    /// `handle_connect` holds the `virtual_sdcard` object, whose `is_active`
    /// gates the SD branch (`pause_resume.py:33-34,45-46`); with no replay
    /// running the status reads `false`.
    #[test]
    fn the_connect_handler_holds_the_virtual_sdcard_object() {
        let printer =
            loaded("[mcu]\nserial: /dev/a\n[pause_resume]\n[virtual_sdcard]\npath: /tmp\n");
        // The connect event is sent by `bring_up`; a load-only test fires it
        // directly, as the host does after the MCUs are up.
        printer.send_event(&KlippyEvent::KlippyConnect);
        let object = printer
            .lookup_object_as::<PauseResume>(PAUSE_RESUME_OBJECT)
            .expect("the object");
        assert!(object.v_sd().is_some(), "the SD object is held");
        assert!(!object.is_sd_active(), "no replay is running");
    }

    // -- the SD branch against the real `virtual_sdcard` ----------------

    /// A temporary directory that removes itself on drop (the
    /// `virtual_sdcard` tests' pattern).
    struct TempDir(PathBuf);

    impl TempDir {
        fn new(name: &str) -> Self {
            static COUNTER: AtomicUsize = AtomicUsize::new(0);
            let path = std::env::temp_dir().join(format!(
                "klipperx-pr-sd-{}-{}-{}",
                std::process::id(),
                name,
                COUNTER.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir_all(&path).expect("cannot create the test directory");
            Self(path)
        }

        fn path(&self) -> &Path {
            &self.0
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// A printer with `pause_resume`, `virtual_sdcard` (pointing at `dir`) and
    /// `gcode_move` — so the four commands reach both the SD primitives and
    /// the `SAVE`/`RESTORE_GCODE_STATE` they run — plus a reactor the test
    /// steps itself: `ManualReactor::run_due` is what fires the replay task's
    /// one-shot timer.
    fn sd_machine(
        dir: &Path,
    ) -> (
        Arc<ManualReactor>,
        Arc<Printer>,
        Arc<GCodeDispatch>,
        Arc<PauseResume>,
        Arc<VirtualSdCard>,
    ) {
        let text = format!(
            "[pause_resume]\n[virtual_sdcard]\npath: {}\n",
            dir.display()
        );
        let (config, _) = Config::from_text(&text).expect("the config parses");
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
        printer.load_config(&config).expect("the config loads");
        gcode_move::ensure(&printer)
            .expect("gcode_move registers")
            .set_move_transform(Arc::new(FakeTarget), true)
            .expect("the slot is free");
        printer.send_event(&KlippyEvent::KlippyReady);
        // `handle_connect` is what binds `pause_resume` to `virtual_sdcard`
        // (`pause_resume.py:16-17,33-34`).
        printer.send_event(&KlippyEvent::KlippyConnect);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the dispatcher is registered");
        let object = printer
            .lookup_object_as::<PauseResume>(PAUSE_RESUME_OBJECT)
            .expect("the section registered the object");
        let vsd = printer
            .lookup_object_as::<VirtualSdCard>(VIRTUAL_SDCARD_OBJECT)
            .expect("the section registered the object");
        (reactor, printer, gcode, object, vsd)
    }

    /// Let a spawned task run (`ManualReactor` runs no tasks, so the test
    /// drives the runtime itself) — the `virtual_sdcard` tests' helper.
    async fn settle() {
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
    }

    /// `print_stats.state`, the replay's lifecycle word.
    fn print_state(printer: &Arc<Printer>) -> String {
        printer
            .lookup_object_as::<PrintStats>(PRINT_STATS_OBJECT)
            .expect("print_stats is registered")
            .get_status(0.0)["state"]
            .as_str()
            .unwrap_or_default()
            .to_string()
    }

    /// `PAUSE` during a replay takes the SD branch (`pause_resume.py:47-59`):
    /// the file is told to pause — the replay task exits before dispatching a
    /// single line — and no `action:paused` line is reported.
    #[tokio::test]
    async fn pause_during_a_replay_pauses_the_file() {
        let dir = TempDir::new("pause_replay");
        std::fs::write(dir.path().join("job.gcode"), "M21\nM21\n").expect("the file");
        let (reactor, printer, gcode, object, vsd) = sd_machine(dir.path());
        let lines = captured_lines(&printer);

        gcode
            .run_script("M23 job.gcode")
            .await
            .expect("the file loads");
        gcode.run_script("M24").await.expect("the replay arms");
        assert!(vsd.is_active(), "the replay is armed");

        // Fire the timer so the replay task is spawned; `do_pause` will wait
        // for it to exit. The task sees the pause flag (set by `do_pause`
        // before it yields) and exits before dispatching a single line.
        reactor.run_due();

        gcode.run_script("PAUSE").await.expect("PAUSE runs");
        assert!(object.is_paused());
        assert!(!vsd.is_active(), "the replay task exited");

        let out = emitted(&lines);
        assert!(
            !out.iter().any(|l| l == "SD card ok"),
            "a file line was replayed in {out:?}"
        );
        assert!(
            !out.iter().any(|l| l == "Done printing file"),
            "the file ran to EOF in {out:?}"
        );
        assert!(
            !out.iter().any(|l| l.contains("action:paused")),
            "the SD branch reports nothing in {out:?}"
        );
        assert_eq!(print_state(&printer), "paused");
        assert_eq!(
            vsd.get_status(0.0)["file_path"],
            "job.gcode",
            "the file stays open under the pause"
        );
    }

    /// `RESUME` after an SD pause takes `sd_paused`'s branch
    /// (`pause_resume.py:68-75`): the replay really restarts, and no
    /// `action:resumed` line is reported.
    #[tokio::test]
    async fn resume_restarts_a_paused_replay() {
        let dir = TempDir::new("resume_replay");
        std::fs::write(dir.path().join("job.gcode"), "M21\nM21\n").expect("the file");
        let (reactor, printer, gcode, object, vsd) = sd_machine(dir.path());
        let lines = captured_lines(&printer);

        gcode
            .run_script("M23 job.gcode")
            .await
            .expect("the file loads");
        gcode.run_script("M24").await.expect("the replay arms");
        // Fire the timer so the replay task is spawned; `do_pause` will wait
        // for it to exit.
        reactor.run_due();
        gcode.run_script("PAUSE").await.expect("PAUSE runs");

        // `PAUSE` held the file: nothing has been replayed yet.
        let out = emitted(&lines);
        assert!(
            !out.iter().any(|l| l == "Done printing file"),
            "the file ran away before RESUME in {out:?}"
        );
        assert_eq!(print_state(&printer), "paused");

        gcode.run_script("RESUME").await.expect("RESUME runs");
        assert!(!object.is_paused());
        reactor.run_due();
        settle().await;

        let out = emitted(&lines);
        assert!(
            out.iter().any(|l| l == "Done printing file"),
            "the replay restarted and finished in {out:?}"
        );
        assert_eq!(
            out.iter().filter(|l| *l == "SD card ok").count(),
            2,
            "both file lines replayed in {out:?}"
        );
        assert!(
            !out.iter().any(|l| l.contains("action:")),
            "the SD branch reports nothing in {out:?}"
        );
        assert_eq!(print_state(&printer), "complete");
        assert!(!vsd.is_active());
    }

    /// After `do_pause` waits for the replay task to exit, `RESUME`
    /// succeeds instead of being refused with `SD busy`. Previously, when
    /// `do_pause` returned immediately, a `RESUME` that outran the exiting
    /// task was refused; now `do_pause` blocks until `work_active` is
    /// `false`, so `do_resume` finds the slot free.
    #[tokio::test]
    async fn resume_after_pause_succeeds_when_task_has_exited() {
        let dir = TempDir::new("resume_after_pause");
        std::fs::write(dir.path().join("job.gcode"), "M21\nM21\n").expect("the file");
        let (reactor, printer, gcode, object, vsd) = sd_machine(dir.path());
        let _lines = captured_lines(&printer);

        gcode
            .run_script("M23 job.gcode")
            .await
            .expect("the file loads");
        gcode.run_script("M24").await.expect("the replay arms");
        // Fire the timer so the replay task is spawned; `do_pause` will wait
        // for it to exit.
        reactor.run_due();

        gcode.run_script("PAUSE").await.expect("PAUSE runs");
        // `do_pause` has waited for the task to exit.
        assert!(!vsd.is_active(), "the replay task exited");
        assert!(object.is_paused());

        // `RESUME` succeeds because `work_active` is `false`.
        gcode.run_script("RESUME").await.expect("RESUME runs");
        assert!(!object.is_paused());
    }

    /// `CANCEL_PRINT` during a replay takes the SD branch
    /// (`pause_resume.py:92-97`): the file is closed, the counters are
    /// cleared, `print_stats` goes `cancelled`, and no `action:cancel` line
    /// is reported.
    #[tokio::test]
    async fn cancel_print_during_a_replay_cancels_the_file() {
        let dir = TempDir::new("cancel_replay");
        std::fs::write(dir.path().join("job.gcode"), "M21\nM21\n").expect("the file");
        let (reactor, printer, gcode, object, vsd) = sd_machine(dir.path());
        let lines = captured_lines(&printer);

        gcode
            .run_script("M23 job.gcode")
            .await
            .expect("the file loads");
        gcode.run_script("M24").await.expect("the replay arms");
        // The replay task has not started, so `print_stats` has no start time
        // yet and would ignore `note_cancel`; the `virtual_sdcard` tests
        // start one the same way.
        printer
            .lookup_object_as::<PrintStats>(PRINT_STATS_OBJECT)
            .expect("print_stats is registered")
            .note_start();
        // Fire the timer so the replay task is spawned; `do_pause` will wait
        // for it to exit.
        reactor.run_due();
        gcode.run_script("PAUSE").await.expect("PAUSE runs");
        assert!(object.is_paused());

        gcode
            .run_script("CANCEL_PRINT")
            .await
            .expect("CANCEL_PRINT runs");
        assert!(!object.is_paused());
        assert!(!object.pause_command_sent.load(Ordering::SeqCst));
        let status = vsd.get_status(0.0);
        assert!(status["file_path"].is_null(), "the file is closed");
        assert_eq!(status["file_size"], 0, "the counters are cleared");
        assert_eq!(status["file_position"], 0);
        assert_eq!(print_state(&printer), "cancelled");
        assert!(
            !emitted(&lines).iter().any(|l| l.contains("action:cancel")),
            "the SD branch reports nothing"
        );
        assert!(!vsd.is_active());
    }

    /// With the SD object held but no replay running, `is_sd_active()` is
    /// `false` and every command keeps the `respond_info` side of its branch:
    /// the `action:*` lines are reported
    /// (`pause_resume.py:47-59,68-75,92-97`).
    #[test]
    fn an_idle_sd_card_still_reports_the_action_lines() {
        let dir = TempDir::new("idle_sd");
        let (_reactor, printer, gcode, object, _vsd) = sd_machine(dir.path());
        let lines = captured_lines(&printer);

        gcode.run_script_sync("PAUSE").expect("PAUSE runs");
        assert!(object.is_paused());
        gcode.run_script_sync("RESUME").expect("RESUME runs");
        gcode
            .run_script_sync("CANCEL_PRINT")
            .expect("CANCEL_PRINT runs");

        assert_eq!(
            emitted(&lines),
            ["// action:paused", "// action:resumed", "// action:cancel"]
        );
    }
}
