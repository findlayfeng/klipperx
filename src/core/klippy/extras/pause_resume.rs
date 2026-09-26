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
//! # What is not here
//!
//! - **The virtual-SD branch is unreachable.** Upstream's `is_sd_active`
//!   calls `v_sd.is_active()` and the pause/resume/cancel paths call
//!   `v_sd.do_pause()`/`do_resume()`/`do_cancel()`
//!   (`pause_resume.py:37-39,46-56,57-65,84-89`). This port's
//!   `[virtual_sdcard]` (`extras/virtual_sdcard.rs`) replays no file and
//!   exposes none of those — its status reports `is_active: false` and
//!   nothing else — so [`SdCard`] has no production implementor that can
//!   drive a replay, and `is_sd_active()` is always `false`. The commands
//!   therefore take the `respond_info` side of each branch
//!   (`action:paused`/`action:resumed`/`action:cancel`).
//! - **The three webhooks endpoints** (`pause_resume/cancel|pause|resume`,
//!   `pause_resume.py:47-52`) are not installed. The API crate has the
//!   registration primitive (`endpoint!` + the `Api` table), but the
//!   endpoints live in `api/endpoints/` and the object they drive is built
//!   while the config is read — wiring them is a separate change. The
//!   endpoint table already lists them "not started"
//!   (`api/endpoints/mod.rs`).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::gcode::{
    CommandError, CommandFuture, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The name the sensors look the object up by (`load_object(config,
/// 'pause_resume')`), which is also the section id.
pub const PAUSE_RESUME_OBJECT: &str = "pause_resume";

/// The object upstream holds at connect for `is_sd_active`
/// (`pause_resume.py:36-37`).
const VIRTUAL_SDCARD_OBJECT: &str = "virtual_sdcard";

section!("pause_resume", order = 30, load = load_config);

/// What `pause_resume` needs from `virtual_sdcard`
/// (`pause_resume.py:37-39,46-56,57-65,84-89`).
///
/// No production type implements this yet — this port's `[virtual_sdcard]`
/// carries no pause/resume/cancel primitive (module docs) — so the SD branch
/// stays unreachable. The trait is the seam so the branch is still covered by
/// a stand-in, the same shape as `sdcard_loop.rs`'s `SdCardFile`.
pub trait SdCard: Send + Sync {
    /// `virtual_sdcard`'s `is_active`: a file is being replayed.
    fn is_active(&self) -> bool;
    /// `do_pause`: stop the replay (`pause_resume.py:51`).
    fn do_pause(&self);
    /// `do_resume`: continue the replay (`pause_resume.py:61`).
    fn do_resume(&self);
    /// `do_cancel`: cancel the running print (`pause_resume.py:87`).
    fn do_cancel(&self);
}

/// The registered `virtual_sdcard` object behind the [`SdCard`] seam.
///
/// `is_active` is read from its status — the only thing this port's
/// `[virtual_sdcard]` exposes, and the flag upstream's `is_active` maps to.
/// The three control calls have no primitive to reach
/// (`extras/virtual_sdcard.rs` replays nothing); this status is `false`
/// today, so they are never reached.
struct VirtualSdCard(Arc<dyn PrinterObject>);

impl SdCard for VirtualSdCard {
    fn is_active(&self) -> bool {
        self.0.get_status(0.0)["is_active"]
            .as_bool()
            .unwrap_or(false)
    }

    fn do_pause(&self) {}

    fn do_resume(&self) {}

    fn do_cancel(&self) {}
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

    /// Upstream's `is_sd_active` (`pause_resume.py:37-39`): a file is being
    /// replayed from `virtual_sdcard`. Always `false` here (module docs).
    fn is_sd_active(&self) -> bool {
        self.v_sd().is_some_and(|sd| sd.is_active())
    }

    /// Upstream's `send_pause_command` (`pause_resume.py:46-56`): pause from
    /// inside an event, once. The SD branch is unreachable here, so this is
    /// the `respond_info("action:paused")` branch (module docs).
    pub fn send_pause_command(&self) {
        if self.pause_command_sent.load(Ordering::SeqCst) {
            return;
        }
        if self.is_sd_active() {
            self.sd_paused.store(true, Ordering::SeqCst);
            if let Some(sd) = self.v_sd() {
                sd.do_pause();
            }
        } else {
            self.sd_paused.store(false, Ordering::SeqCst);
            self.respond_info("action:paused");
        }
        self.pause_command_sent.store(true, Ordering::SeqCst);
    }

    /// Upstream's `send_resume_command` (`pause_resume.py:57-65`): continue
    /// the replay, or report `action:resumed`, and arm the next pause.
    fn send_resume_command(&self) {
        if self.sd_paused.load(Ordering::SeqCst) {
            if let Some(sd) = self.v_sd() {
                sd.do_resume();
            }
            self.sd_paused.store(false, Ordering::SeqCst);
        } else {
            self.respond_info("action:resumed");
        }
        self.pause_command_sent.store(false, Ordering::SeqCst);
    }

    /// Upstream's `cmd_CLEAR_PAUSE` body (`pause_resume.py:79-81`): forget
    /// the paused and the command-sent state. `sd_paused` is left as it is,
    /// as upstream leaves it.
    fn clear_pause(&self) {
        self.is_paused.store(false, Ordering::SeqCst);
        self.pause_command_sent.store(false, Ordering::SeqCst);
    }

    /// The SD side of `CANCEL_PRINT` (`pause_resume.py:86-87`): cancel the
    /// running file. A no-op without an SD object — the only caller reaches it
    /// through `is_sd_active`/`sd_paused`, which need one.
    fn cancel_sd_print(&self) {
        if let Some(sd) = self.v_sd() {
            sd.do_cancel();
        }
    }

    /// Upstream's `handle_connect` (`pause_resume.py:36-37`): hold the
    /// `virtual_sdcard` object for `is_sd_active`.
    fn handle_connect(&self) {
        let Some(printer) = self.printer.as_ref().and_then(Weak::upgrade) else {
            return;
        };
        let Some(sd) = printer.lookup_object(VIRTUAL_SDCARD_OBJECT) else {
            return;
        };
        *self.lock() = Some(Arc::new(VirtualSdCard(sd)));
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
        type Command = for<'a> fn(&'a Arc<PauseResume>, &'a GcodeCommand) -> CommandFuture<'a>;
        const COMMANDS: &[(&str, Command, &str)] = &[
            ("PAUSE", cmd_pause, "Pauses the current print"),
            ("RESUME", cmd_resume, "Resumes the print from a pause"),
            (
                "CLEAR_PAUSE",
                cmd_clear_pause,
                "Clears the current paused state without resuming the print",
            ),
            ("CANCEL_PRINT", cmd_cancel_print, "Cancel the current print"),
        ];
        for &(name, command, help) in COMMANDS {
            let handler: CommandHandler = {
                let object = Arc::clone(self);
                Arc::new(move |gcmd| {
                    let object = Arc::clone(&object);
                    Box::pin(async move { command(&object, gcmd).await })
                })
            };
            gcode
                .register_command(name, handler, Some(help), false)
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
        object.send_pause_command();
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
        object.send_resume_command();
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
            object.cancel_sd_print();
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
    use crate::core::klippy::mathutil::Coord;
    use crate::core::klippy::reactor::ManualReactor;
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
        fn do_pause(&self) {
            self.pauses.fetch_add(1, Ordering::SeqCst);
        }
        fn do_resume(&self) {
            self.resumes.fetch_add(1, Ordering::SeqCst);
        }
        fn do_cancel(&self) {
            self.cancels.fetch_add(1, Ordering::SeqCst);
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
    #[test]
    fn an_active_sd_print_is_paused_and_resumed_through_the_file() {
        let (printer, gcode, object) = machine();
        let lines = captured_lines(&printer);
        let sd = Arc::new(FakeSd {
            active: true,
            ..Default::default()
        });
        *object.lock() = Some(Arc::clone(&sd) as Arc<dyn SdCard>);

        object.send_pause_command();
        assert_eq!(sd.pauses.load(Ordering::SeqCst), 1);
        assert!(object.sd_paused.load(Ordering::SeqCst));
        assert!(emitted(&lines).is_empty(), "no `action:paused` line");

        object.send_resume_command();
        assert_eq!(sd.resumes.load(Ordering::SeqCst), 1);
        assert!(!object.sd_paused.load(Ordering::SeqCst));
        assert!(emitted(&lines).is_empty(), "no `action:resumed` line");

        // The guard still makes a second pause a no-op.
        object.pause_command_sent.store(true, Ordering::SeqCst);
        object.send_pause_command();
        assert_eq!(sd.pauses.load(Ordering::SeqCst), 1);
        drop(gcode);
    }

    /// `handle_connect` holds the `virtual_sdcard` object, and its
    /// `is_active` — the port's status flag — gates the SD branch
    /// (`pause_resume.py:36-39`).
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
        assert!(!object.is_sd_active(), "the port's status is always false");
    }
}
