//! `print_stats` — print statistics: state, filament, duration
//! (upstream `klippy/extras/print_stats.py`).
//!
//! Tracks the lifecycle of a print (`standby` → `printing` → `paused`/`complete`/
//! `cancelled`/`error`) and reports duration, filament used, and slicer layer
//! info. The `virtual_sdcard` object creates this through [`PrintStats::ensure`]
//! (upstream's `printer.load_object(config, 'print_stats')`) and drives the
//! state machine through the `note_*` methods as a file is replayed.
//!
//! | command | role (`print_stats.py`) |
//! |---|---|
//! | `SET_PRINT_STATS_INFO` | pass slicer layer info (`TOTAL_LAYER`/`CURRENT_LAYER`) |
//!
//! The section reads no config options (upstream's `load_config` creates the
//! object without reading any). It is still a `load_object` target —
//! [`PrintStats::ensure`] is the lazy-creation entry point a caller like
//! `virtual_sdcard` uses.
//!
//! # What is not here
//!
//! - **`_handle_activate_extruder` is not wired.** Upstream registers a handler
//!   for `extruder:activate_extruder` that re-anchors `last_epos` when the
//!   active extruder changes mid-print (`print_stats.py:22-24`). This port's
//!   `extruder.rs` declares the event but does not fire it, so the handler would
//!   never run. The `last_epos` is only re-anchored at `note_start`; a mid-print
//!   extruder swap will produce a spurious filament-used spike. Wire the handler
//!   once `extruder.rs` fires the event.
//!
//! - **No config options.** Upstream's `PrintStats.__init__` reads nothing from
//!   the config; the section exists only so `load_object` can create it.

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::ConfigError;
use crate::core::klippy::extras::gcode_move::{GCodeMove, GCODE_MOVE_OBJECT};
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The name the loader and `virtual_sdcard` look the object up by
/// (`printer.load_object(config, 'print_stats')`), which is also the section id.
pub const PRINT_STATS_OBJECT: &str = "print_stats";

/// The threshold below which no positive extrusion has been detected
/// (upstream's `0.0000001`).
const FILAMENT_USED_EPSILON: f64 = 1e-7;

section!("print_stats", order = 30, load = load_config);

/// The mutable state machine (`print_stats.py:60-72` `reset`).
#[derive(Debug)]
struct PrintStatsState {
    filename: String,
    error_message: String,
    state: String,
    prev_pause_duration: f64,
    last_epos: f64,
    filament_used: f64,
    total_duration: f64,
    print_start_time: Option<f64>,
    last_pause_time: Option<f64>,
    init_duration: f64,
    info_total_layer: Option<i64>,
    info_current_layer: Option<i64>,
}

impl PrintStatsState {
    /// Upstream's `reset` (`print_stats.py:60-72`).
    fn reset(&mut self) {
        self.filename = String::new();
        self.error_message = String::new();
        self.state = "standby".to_string();
        self.prev_pause_duration = 0.0;
        self.last_epos = 0.0;
        self.filament_used = 0.0;
        self.total_duration = 0.0;
        self.print_start_time = None;
        self.last_pause_time = None;
        self.init_duration = 0.0;
        self.info_total_layer = None;
        self.info_current_layer = None;
    }
}

impl Default for PrintStatsState {
    fn default() -> Self {
        let mut state = Self {
            filename: String::new(),
            error_message: String::new(),
            state: "standby".to_string(),
            prev_pause_duration: 0.0,
            last_epos: 0.0,
            filament_used: 0.0,
            total_duration: 0.0,
            print_start_time: None,
            last_pause_time: None,
            init_duration: 0.0,
            info_total_layer: None,
            info_current_layer: None,
        };
        // `default` and `reset` produce the same fields; calling `reset` keeps
        // the two in sync if upstream's initial values ever diverge.
        state.reset();
        state
    }
}

/// The `print_stats` module object (upstream's `PrintStats`).
pub struct PrintStats {
    /// The machine, to reach the reactor and `gcode_move`.
    printer: Weak<Printer>,
    state: Mutex<PrintStatsState>,
}

impl PrintStats {
    fn new(printer: &Arc<Printer>) -> Self {
        Self {
            printer: Arc::downgrade(printer),
            state: Mutex::new(PrintStatsState::default()),
        }
    }

    /// The single `print_stats`; the first caller creates it, as upstream's
    /// `printer.load_object(config, 'print_stats')` does.
    ///
    /// # Errors
    /// A duplicate registration or a g-code name this dispatcher refuses.
    pub fn ensure(printer: &Arc<Printer>) -> Result<Arc<PrintStats>, ConfigError> {
        if let Some(existing) = printer.lookup_object_as::<PrintStats>(PRINT_STATS_OBJECT) {
            return Ok(existing);
        }
        let object = Arc::new(Self::new(printer));
        object.register_commands(printer)?;
        printer.add_object(
            PRINT_STATS_OBJECT,
            Arc::clone(&object) as Arc<dyn PrinterObject>,
        )?;
        Ok(object)
    }

    /// Upstream's `reset` (`print_stats.py:60-72`).
    pub fn reset(&self) {
        self.lock().reset();
    }

    /// Upstream's `set_current_file` (`print_stats.py:26-28`): `reset` then
    /// set the filename.
    pub fn set_current_file(&self, filename: &str) {
        let mut s = self.lock();
        s.reset();
        s.filename = filename.to_string();
    }

    /// Upstream's `note_start` (`print_stats.py:29-39`).
    pub fn note_start(&self) {
        let t = self.reactor_monotonic();
        let mut s = self.lock();
        if s.print_start_time.is_none() {
            s.print_start_time = Some(t);
        } else if let Some(last_pause) = s.last_pause_time {
            s.prev_pause_duration += t - last_pause;
            s.last_pause_time = None;
        }
        // Re-anchor `last_epos` to the current E position.
        s.last_epos = self.read_e_position(t);
        s.state = "printing".to_string();
        s.error_message = String::new();
    }

    /// Upstream's `note_pause` (`print_stats.py:41-47`).
    pub fn note_pause(&self) {
        let t = self.reactor_monotonic();
        let mut s = self.lock();
        if s.last_pause_time.is_none() {
            s.last_pause_time = Some(t);
            self.update_filament_usage(t, &mut s);
        }
        if s.state != "error" {
            s.state = "paused".to_string();
        }
    }

    /// Upstream's `note_complete` (`print_stats.py:48-49`).
    pub fn note_complete(&self) {
        self.note_finish("complete", "");
    }

    /// Upstream's `note_error` (`print_stats.py:50-51`).
    pub fn note_error(&self, message: &str) {
        self.note_finish("error", message);
    }

    /// Upstream's `note_cancel` (`print_stats.py:52-53`).
    pub fn note_cancel(&self) {
        self.note_finish("cancelled", "");
    }

    /// Upstream's `_note_finish` (`print_stats.py:54-59`).
    fn note_finish(&self, state: &str, error_message: &str) {
        let mut s = self.lock();
        if s.print_start_time.is_none() {
            return;
        }
        s.state = state.to_string();
        s.error_message = error_message.to_string();
        let eventtime = self.reactor_monotonic();
        s.total_duration = eventtime - s.print_start_time.expect("checked above");
        if s.filament_used < FILAMENT_USED_EPSILON {
            // No positive extrusion detected during the print.
            s.init_duration = s.total_duration - s.prev_pause_duration;
        }
        s.print_start_time = None;
    }

    /// Upstream's `_update_filament_usage` (`print_stats.py:25-31`).
    ///
    /// Called with the state lock held; reads `gcode_move.get_status` which
    /// locks a separate `Mutex`, so there is no deadlock.
    fn update_filament_usage(&self, eventtime: f64, state: &mut PrintStatsState) {
        let Some(gc_status) = self.gcode_move_status(eventtime) else {
            return;
        };
        let cur_epos = gc_status["position"][3].as_f64().unwrap_or_else(|| {
            tracing::warn!("gcode_move status missing position[3]; defaulting to 0.0");
            0.0
        });
        let extrude_factor = gc_status["extrude_factor"].as_f64().unwrap_or_else(|| {
            tracing::warn!("gcode_move status missing extrude_factor; defaulting to 0.0");
            0.0
        });
        state.filament_used += (cur_epos - state.last_epos) / extrude_factor;
        state.last_epos = cur_epos;
    }

    /// Read the current E position from `gcode_move.get_status` (used by
    /// `note_start` to re-anchor `last_epos`).
    fn read_e_position(&self, eventtime: f64) -> f64 {
        self.gcode_move_status(eventtime)
            .and_then(|s| s["position"][3].as_f64())
            .unwrap_or_else(|| {
                tracing::warn!("gcode_move status missing position[3]; defaulting to 0.0");
                0.0
            })
    }

    /// Look up `gcode_move` and call `get_status(eventtime)`.
    fn gcode_move_status(&self, eventtime: f64) -> Option<Value> {
        let printer = self.printer.upgrade()?;
        let gcode_move = printer.lookup_object_as::<GCodeMove>(GCODE_MOVE_OBJECT)?;
        Some(gcode_move.get_status(eventtime))
    }

    /// The reactor's monotonic clock (`self.reactor.monotonic()`).
    fn reactor_monotonic(&self) -> f64 {
        self.printer
            .upgrade()
            .map(|p| p.reactor().monotonic())
            .unwrap_or(0.0)
    }

    /// The state lock, poisoning treated as continued unwinding
    /// (`gcode_move.rs` convention).
    fn lock(&self) -> MutexGuard<'_, PrintStatsState> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// Register `SET_PRINT_STATS_INFO`, capturing a handle to this object.
    fn register_commands(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        const PARAMS: &[&str] = &["TOTAL_LAYER", "CURRENT_LAYER"];
        let handler: CommandHandler = {
            let object = Arc::clone(self);
            sync(move |gcmd| object.cmd_set_print_stats_info(gcmd))
        };
        gcode
            .register_command_with_params(
                "SET_PRINT_STATS_INFO",
                handler,
                Some("Pass slicer info like layer act and total to klipper"),
                PARAMS,
                false,
            )
            .map_err(ConfigError::new)?;
        Ok(())
    }

    /// Upstream's `cmd_SET_PRINT_STATS_INFO` (`print_stats.py:73-84`).
    fn cmd_set_print_stats_info(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let mut s = self.lock();
        // `get_int(name, default, minval=0)`: when the parameter is absent, the
        // default (which may be `None`) is used; when present, it is parsed as
        // an int with `minval=0`.
        let total_layer = if gcmd.get_command_parameters().contains_key("TOTAL_LAYER") {
            Some(gcmd.get_int_bounded("TOTAL_LAYER", Some(0), None)?)
        } else {
            s.info_total_layer
        };
        let current_layer = if gcmd.get_command_parameters().contains_key("CURRENT_LAYER") {
            Some(gcmd.get_int_bounded("CURRENT_LAYER", Some(0), None)?)
        } else {
            s.info_current_layer
        };
        if total_layer == Some(0) {
            s.info_total_layer = None;
            s.info_current_layer = None;
        } else if total_layer != s.info_total_layer {
            s.info_total_layer = total_layer;
            s.info_current_layer = Some(0);
        }
        if let (Some(info_total), Some(cur)) = (s.info_total_layer, current_layer) {
            if Some(cur) != s.info_current_layer {
                s.info_current_layer = Some(cur.min(info_total));
            }
        }
        Ok(())
    }
}

impl std::fmt::Debug for PrintStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = self.lock();
        f.debug_struct("PrintStats")
            .field("state", &s.state)
            .field("filename", &s.filename)
            .field("filament_used", &s.filament_used)
            .field("total_duration", &s.total_duration)
            .finish_non_exhaustive()
    }
}

impl PrinterObject for PrintStats {
    /// Upstream's `get_status` (`print_stats.py:86-100`).
    fn get_status(&self, eventtime: f64) -> Value {
        let mut s = self.lock();
        let mut time_paused = s.prev_pause_duration;
        if let Some(start) = s.print_start_time {
            if let Some(last_pause) = s.last_pause_time {
                // Total time spent paused during the print.
                time_paused += eventtime - last_pause;
            } else {
                // Accumulate filament if not paused.
                self.update_filament_usage(eventtime, &mut s);
            }
            s.total_duration = eventtime - start;
            if s.filament_used < FILAMENT_USED_EPSILON {
                // Track duration prior to extrusion.
                s.init_duration = s.total_duration - time_paused;
            }
        }
        let print_duration = s.total_duration - s.init_duration - time_paused;
        json!({
            "filename": &s.filename,
            "total_duration": s.total_duration,
            "print_duration": print_duration,
            "filament_used": s.filament_used,
            "state": &s.state,
            "message": &s.error_message,
            "info": {
                "total_layer": s.info_total_layer,
                "current_layer": s.info_current_layer,
            }
        })
    }
}

/// The factory `section!` names (`print_stats.py:102-103`).
///
/// # Errors
/// An already-registered object, or a g-code name this dispatcher refuses.
pub fn load_config(
    _config: &crate::core::klippy::config::ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    // `ensure` may have created the object first (e.g. `virtual_sdcard` calling
    // `PrintStats::ensure`); reuse it either way. The loader registers whatever
    // this returns under `print_stats`.
    if let Some(existing) = printer.lookup_object_as::<PrintStats>(PRINT_STATS_OBJECT) {
        return Ok(existing as Arc<dyn PrinterObject>);
    }
    let object = Arc::new(PrintStats::new(printer));
    object.register_commands(printer)?;
    Ok(object as Arc<dyn PrinterObject>)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::extras::gcode_move::{self, MoveTarget};
    use crate::core::klippy::mathutil::Coord;
    use crate::core::klippy::reactor::{ManualReactor, Reactor};

    /// A move target that records what it was asked and stands where the last
    /// move left it — enough for `G1` to change the E position.
    struct FakeTarget {
        position: Mutex<Coord>,
    }

    impl FakeTarget {
        fn new(position: Coord) -> Self {
            Self {
                position: Mutex::new(position),
            }
        }
    }

    impl MoveTarget for FakeTarget {
        fn move_to(&self, position: Coord, _speed: f64) -> Result<(), CommandError> {
            *self.position.lock().unwrap() = position;
            Ok(())
        }

        fn position(&self) -> Coord {
            *self.position.lock().unwrap()
        }
    }

    /// A ready printer with `gcode`, `gcode_move` (with a move target) and
    /// `print_stats` on it.
    fn machine() -> (
        Arc<ManualReactor>,
        Arc<Printer>,
        Arc<GCodeDispatch>,
        Arc<PrintStats>,
    ) {
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let gcode_move = gcode_move::ensure(&printer).unwrap();
        gcode_move
            .set_move_transform(
                Arc::new(FakeTarget::new(Coord::default())) as Arc<dyn MoveTarget>,
                true,
            )
            .unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        let object = PrintStats::ensure(&printer).unwrap();
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap();
        (reactor, printer, gcode, object)
    }

    /// Read a field from `get_status` as a string.
    fn status_str(object: &PrintStats, eventtime: f64, key: &str) -> String {
        object.get_status(eventtime)[key]
            .as_str()
            .unwrap_or_default()
            .to_string()
    }

    // -- reset / initial state -------------------------------------------

    /// `reset` (and the initial state) match upstream's `reset`
    /// (`print_stats.py:60-72`).
    #[test]
    fn reset_sets_all_fields_to_their_initial_values() {
        let (_reactor, _printer, _gcode, object) = machine();
        // Mutate something, then reset and verify.
        object.set_current_file("test.gcode");
        object.note_start();
        object.note_pause();
        object.reset();

        let s = object.get_status(0.0);
        assert_eq!(s["filename"], "");
        assert_eq!(s["state"], "standby");
        assert_eq!(s["message"], "");
        assert_eq!(s["total_duration"], 0.0);
        assert_eq!(s["print_duration"], 0.0);
        assert_eq!(s["filament_used"], 0.0);
        assert_eq!(s["info"]["total_layer"], json!(null));
        assert_eq!(s["info"]["current_layer"], json!(null));
    }

    // -- set_current_file ------------------------------------------------

    /// `set_current_file` resets then sets the filename
    /// (`print_stats.py:26-28`).
    #[test]
    fn set_current_file_resets_then_sets_the_filename() {
        let (_reactor, _printer, _gcode, object) = machine();
        // Pollute the state first.
        object.note_start();
        object.note_pause();

        object.set_current_file("cube.gcode");
        let s = object.get_status(0.0);
        assert_eq!(s["filename"], "cube.gcode");
        assert_eq!(s["state"], "standby");
        assert_eq!(s["filament_used"], 0.0);
        assert_eq!(s["total_duration"], 0.0);
    }

    // -- note_start / note_pause / note_complete -------------------------

    /// `note_start` sets `state="printing"` and clears `error_message`
    /// (`print_stats.py:29-39`).
    #[test]
    fn note_start_sets_state_to_printing() {
        let (_reactor, _printer, _gcode, object) = machine();
        object.note_start();
        assert_eq!(status_str(&object, 0.0, "state"), "printing");
        assert_eq!(status_str(&object, 0.0, "message"), "");
    }

    /// `note_start` → `note_pause` → `note_complete` flow: the state
    /// transitions and duration accounting match upstream
    /// (`print_stats.py:29-59`).
    #[test]
    fn start_pause_complete_tracks_state_and_duration() {
        let (reactor, _printer, _gcode, object) = machine();

        // Start the print at t=0.
        object.note_start();
        assert_eq!(status_str(&object, 0.0, "state"), "printing");

        // Advance to t=10, then pause.
        reactor.advance(10.0);
        object.note_pause();
        assert_eq!(status_str(&object, 10.0, "state"), "paused");

        // While paused, get_status accumulates time_paused.
        let s_paused = object.get_status(15.0);
        assert_eq!(s_paused["state"], "paused");
        // total_duration = 15 - 0 = 15; time_paused = 0 + (15 - 10) = 5;
        // init_duration = 15 - 5 = 10 (no extrusion); print_duration = 15 - 10 - 5 = 0.
        assert_eq!(s_paused["total_duration"], 15.0);
        assert_eq!(s_paused["print_duration"], 0.0);

        // Advance to t=15 and resume (note_start with existing print_start_time
        // and last_pause_time).
        reactor.advance(5.0);
        object.note_start();
        assert_eq!(status_str(&object, 15.0, "state"), "printing");
        // prev_pause_duration should now be 15 - 10 = 5.

        // Advance to t=25 and complete.
        reactor.advance(10.0);
        object.note_complete();
        assert_eq!(status_str(&object, 25.0, "state"), "complete");

        // After completion: total_duration = 25 - 0 = 25;
        // prev_pause_duration = 5; no extrusion → init_duration = 25 - 5 = 20;
        // print_duration = 25 - 20 - 5 = 0.
        let s_done = object.get_status(25.0);
        assert_eq!(s_done["total_duration"], 25.0);
        assert_eq!(s_done["print_duration"], 0.0);
        assert_eq!(s_done["state"], "complete");
    }

    /// `note_complete` without a prior `note_start` is a no-op
    /// (`print_stats.py:54-55`).
    #[test]
    fn note_complete_without_start_is_a_noop() {
        let (_reactor, _printer, _gcode, object) = machine();
        object.note_complete();
        assert_eq!(status_str(&object, 0.0, "state"), "standby");
        assert_eq!(object.get_status(0.0)["total_duration"], 0.0);
    }

    // -- note_error / note_cancel ----------------------------------------

    /// `note_error` sets the state and message (`print_stats.py:50-51`).
    #[test]
    fn note_error_sets_state_and_message() {
        let (reactor, _printer, _gcode, object) = machine();
        object.note_start();
        reactor.advance(5.0);
        object.note_error("Thermistor failed");

        let s = object.get_status(5.0);
        assert_eq!(s["state"], "error");
        assert_eq!(s["message"], "Thermistor failed");
    }

    /// `note_cancel` sets the state to "cancelled" (`print_stats.py:52-53`).
    #[test]
    fn note_cancel_sets_state_to_cancelled() {
        let (reactor, _printer, _gcode, object) = machine();
        object.note_start();
        reactor.advance(5.0);
        object.note_cancel();

        let s = object.get_status(5.0);
        assert_eq!(s["state"], "cancelled");
        assert_eq!(s["message"], "");
    }

    /// `note_pause` does not overwrite an error state (`print_stats.py:46`).
    #[test]
    fn note_pause_does_not_overwrite_error_state() {
        let (reactor, _printer, _gcode, object) = machine();
        object.note_start();
        object.note_error("boom");
        object.note_pause();
        assert_eq!(status_str(&object, 0.0, "state"), "error");
    }

    // -- SET_PRINT_STATS_INFO --------------------------------------------

    /// `SET_PRINT_STATS_INFO TOTAL_LAYER=0` clears both layer fields
    /// (`print_stats.py:77-78`).
    #[test]
    fn total_layer_zero_clears_both_layer_fields() {
        let (_reactor, _printer, gcode, object) = machine();
        // Set some layer info first.
        gcode
            .run_script_sync("SET_PRINT_STATS_INFO TOTAL_LAYER=10 CURRENT_LAYER=5")
            .unwrap();
        assert_eq!(object.get_status(0.0)["info"]["total_layer"], 10);
        assert_eq!(object.get_status(0.0)["info"]["current_layer"], 5);

        // TOTAL_LAYER=0 clears.
        gcode
            .run_script_sync("SET_PRINT_STATS_INFO TOTAL_LAYER=0")
            .unwrap();
        assert_eq!(object.get_status(0.0)["info"]["total_layer"], json!(null));
        assert_eq!(object.get_status(0.0)["info"]["current_layer"], json!(null));
    }

    /// A new `TOTAL_LAYER` (different from the current one) resets
    /// `current_layer` to 0 when `CURRENT_LAYER` is also 0
    /// (`print_stats.py:79-81`). Without an explicit `CURRENT_LAYER`, the
    /// default is the old `info_current_layer`, so the second `if` would set
    /// it back — matching upstream's behavior.
    #[test]
    fn a_new_total_layer_with_current_zero_resets_current_layer() {
        let (_reactor, _printer, gcode, object) = machine();
        gcode
            .run_script_sync("SET_PRINT_STATS_INFO TOTAL_LAYER=10 CURRENT_LAYER=5")
            .unwrap();
        assert_eq!(object.get_status(0.0)["info"]["current_layer"], 5);

        // Different total with explicit CURRENT_LAYER=0 → current is 0.
        gcode
            .run_script_sync("SET_PRINT_STATS_INFO TOTAL_LAYER=20 CURRENT_LAYER=0")
            .unwrap();
        assert_eq!(object.get_status(0.0)["info"]["total_layer"], 20);
        assert_eq!(object.get_status(0.0)["info"]["current_layer"], 0);
    }

    /// Without an explicit `CURRENT_LAYER`, a new `TOTAL_LAYER` resets
    /// `info_current_layer` to 0 but the second `if` restores it from the
    /// default — matching upstream's behavior exactly
    /// (`print_stats.py:79-84`).
    #[test]
    fn a_new_total_layer_without_current_keeps_the_default_current() {
        let (_reactor, _printer, gcode, object) = machine();
        gcode
            .run_script_sync("SET_PRINT_STATS_INFO TOTAL_LAYER=10 CURRENT_LAYER=5")
            .unwrap();
        // TOTAL_LAYER=20 without CURRENT_LAYER: current_layer defaults to 5,
        // the reset sets it to 0, then the second if sets it to min(5, 20) = 5.
        gcode
            .run_script_sync("SET_PRINT_STATS_INFO TOTAL_LAYER=20")
            .unwrap();
        assert_eq!(object.get_status(0.0)["info"]["total_layer"], 20);
        assert_eq!(object.get_status(0.0)["info"]["current_layer"], 5);
    }

    /// `CURRENT_LAYER` is truncated to `info_total_layer`
    /// (`print_stats.py:83-84`).
    #[test]
    fn current_layer_is_truncated_to_total() {
        let (_reactor, _printer, gcode, object) = machine();
        gcode
            .run_script_sync("SET_PRINT_STATS_INFO TOTAL_LAYER=10")
            .unwrap();

        // CURRENT_LAYER=15 > TOTAL_LAYER=10 → clamped to 10.
        gcode
            .run_script_sync("SET_PRINT_STATS_INFO CURRENT_LAYER=15")
            .unwrap();
        assert_eq!(object.get_status(0.0)["info"]["current_layer"], 10);

        // CURRENT_LAYER=3 < TOTAL_LAYER=10 → 3.
        gcode
            .run_script_sync("SET_PRINT_STATS_INFO CURRENT_LAYER=3")
            .unwrap();
        assert_eq!(object.get_status(0.0)["info"]["current_layer"], 3);
    }

    /// `SET_PRINT_STATS_INFO` with no parameters keeps the current values
    /// (upstream's `get_int(name, default)` uses `info_*` as default).
    #[test]
    fn set_print_stats_info_without_params_keeps_current_values() {
        let (_reactor, _printer, gcode, object) = machine();
        gcode
            .run_script_sync("SET_PRINT_STATS_INFO TOTAL_LAYER=10 CURRENT_LAYER=3")
            .unwrap();
        gcode.run_script_sync("SET_PRINT_STATS_INFO").unwrap();
        assert_eq!(object.get_status(0.0)["info"]["total_layer"], 10);
        assert_eq!(object.get_status(0.0)["info"]["current_layer"], 3);
    }

    // -- get_status shapes ------------------------------------------------

    /// `get_status` in `standby` (no print started) reports zeros and the
    /// standby state.
    #[test]
    fn get_status_in_standby_reports_zeros() {
        let (_reactor, _printer, _gcode, object) = machine();
        let s = object.get_status(0.0);
        assert_eq!(s["filename"], "");
        assert_eq!(s["state"], "standby");
        assert_eq!(s["total_duration"], 0.0);
        assert_eq!(s["print_duration"], 0.0);
        assert_eq!(s["filament_used"], 0.0);
        assert_eq!(s["message"], "");
        assert_eq!(s["info"]["total_layer"], json!(null));
        assert_eq!(s["info"]["current_layer"], json!(null));
    }

    /// `get_status` in `printing` reports the accumulated durations.
    #[test]
    fn get_status_in_printing_reports_durations() {
        let (reactor, _printer, _gcode, object) = machine();
        object.set_current_file("job.gcode");
        object.note_start();
        reactor.advance(5.0);

        let s = object.get_status(5.0);
        assert_eq!(s["filename"], "job.gcode");
        assert_eq!(s["state"], "printing");
        assert_eq!(s["total_duration"], 5.0);
        // No extrusion → init_duration = total_duration - time_paused = 5 - 0 = 5.
        // print_duration = 5 - 5 - 0 = 0.
        assert_eq!(s["print_duration"], 0.0);
    }

    /// `get_status` in `paused` accumulates `time_paused` from
    /// `last_pause_time`.
    #[test]
    fn get_status_in_paused_accumulates_time_paused() {
        let (reactor, _printer, _gcode, object) = machine();
        object.note_start();
        reactor.advance(10.0);
        object.note_pause();
        // At t=15, paused since t=10: time_paused = 0 + (15 - 10) = 5.
        let s = object.get_status(15.0);
        assert_eq!(s["state"], "paused");
        assert_eq!(s["total_duration"], 15.0);
        // init_duration = 15 - 5 = 10; print_duration = 15 - 10 - 5 = 0.
        assert_eq!(s["print_duration"], 0.0);
    }

    /// The command is registered with upstream's help text
    /// (`print_stats.py:71-72`).
    #[test]
    fn the_command_is_registered_with_upstreams_help_text() {
        let (_reactor, _printer, gcode, _object) = machine();
        let help = gcode.command_help();
        assert_eq!(
            help.get("SET_PRINT_STATS_INFO").map(String::as_str),
            Some("Pass slicer info like layer act and total to klipper")
        );
    }

    /// `ensure` shares one object across callers.
    #[test]
    fn ensure_shares_one_object_across_callers() {
        let (_reactor, printer, _gcode, object) = machine();
        let again = PrintStats::ensure(&printer).unwrap();
        assert!(Arc::ptr_eq(&object, &again));
    }

    /// Filament usage accumulates when the E position changes between calls
    /// (`print_stats.py:25-31,40-42`). E words are absolute by default (M82),
    /// so `G1 E10` sets E to 10 and `G1 E15` sets E to 15.
    #[test]
    fn filament_used_accumulates_with_e_position_changes() {
        let (reactor, _printer, gcode, object) = machine();
        object.note_start();
        // Move E to 10 mm (extrude_factor defaults to 1.0).
        reactor.advance(1.0);
        gcode.run_script_sync("G1 E10").unwrap();

        // Advance and pause — note_pause calls _update_filament_usage.
        reactor.advance(1.0);
        object.note_pause();

        // filament_used should be 10.0 (10 - 0) / 1.0.
        let s = object.get_status(2.0);
        assert_eq!(s["filament_used"], 10.0);

        // Resume and extrude more (absolute E=15).
        reactor.advance(1.0);
        object.note_start();
        reactor.advance(1.0);
        gcode.run_script_sync("G1 E15").unwrap();
        reactor.advance(1.0);
        object.note_pause();

        // filament_used should be 10.0 + (15 - 10) / 1.0 = 15.0.
        let s = object.get_status(5.0);
        assert_eq!(s["filament_used"], 15.0);
    }
}
