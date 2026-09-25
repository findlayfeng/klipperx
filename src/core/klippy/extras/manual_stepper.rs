//! `[manual_stepper <name>]` — a motor driven by hand, and as a G-Code axis.
//!
//! Upstream `klippy/extras/manual_stepper.py`: a single stepper section (the
//! motor options through `stepper.PrinterStepper`, or through a rail when
//! `endstop_pin` is present) with its own trapezoid queue and the
//! `MANUAL_STEPPER STEPPER=<name>` command. The same object can also register
//! with the toolhead as an extra G-Code axis (`GCODE_AXIS=`), which the
//! `G1 A…` word then drives.
//!
//! | option | meaning |
//! |---|---|
//! | `step_pin` / `dir_pin` / `enable_pin` | the motor pins (`PrinterStepper`) |
//! | `microsteps` / `rotation_distance` | the step geometry (`PrinterStepper`) |
//! | `endstop_pin` | when present, the stepper can home (upstream's `LookupRail(need_position_minmax=False)`) |
//! | `velocity` | `MOVE` default speed, mm/s (default 5, `> 0`) |
//! | `accel` | `MOVE` default acceleration, mm/s² (default 0, `>= 0`) |
//! | `position_min` / `position_max` | the travel bounds, when set (no bound when absent) |
//!
//! `MANUAL_STEPPER` parameters (`manual_stepper.py:94-137`): `ENABLE`,
//! `SET_POSITION`, `SPEED`, `ACCEL`, `MOVE`, `SYNC`, `STOP_ON_ENDSTOP`, and the
//! `GCODE_AXIS` group below.
//!
//! # G-Code axis registration (`manual_stepper.py:139-167`)
//!
//! `GCODE_AXIS=<letter>` adds this stepper to the toolhead's extra axes;
//! `GCODE_AXIS=` takes it back off. The letter is **upper-cased before it is
//! validated** (upstream `gcmd.get('GCODE_AXIS').upper()`, `:152`), so a
//! lowercase request is accepted as its uppercase form; `F`, multi-character
//! values, and non-letters are refused with `Not a valid GCODE_AXIS`.
//!
//! # Timelines: this port's gap
//!
//! Upstream gives every manual stepper its own trapq
//! (`motion_queuing.allocate_trapq`) and appends a move's trapezoid to it
//! (`_submit_move`), so the motor really steps. This port has **no** public
//! path to allocate a trapq and add a stepper to the toolhead's motion queue
//! after connect, so the timeline here is **dwell-only**: `do_move` advances
//! the planner's print time by `calc_move_time`'s trapezoid duration
//! (`toolhead.dwell`) and moves `commanded_pos`, but **no trapq is appended and
//! no steps are generated for a `MOVE`**. The corpus exercises the command
//! surface, not the motion, so this is enough for `manual_stepper.test`; the
//! step generation is the gap to close with a motion-queue seam.
//!
//! `do_set_position` also skips upstream's `toolhead.flush_step_generation()`
//! and the rail position write for the same reason. `STOP_ON_ENDSTOP` is
//! parsed and answers `No endstop for this manual stepper` when the section
//! has none, but an actual endstop-triggered homing run is not driven (there
//! is no `homing.manual_home` in this port).

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::object::{PrinterConfig, CONFIGFILE_OBJECT};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::force_move::calc_move_time;
use crate::core::klippy::extras::stepper::{PrinterStepper, RailGeometry};
use crate::core::klippy::extras::stepper_enable::PrinterStepperEnable;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    parse_float, sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::motion::extra::ExtraAxis;
use crate::core::klippy::motion::itersolve::{cartesian_active_flags, cartesian_position_fn, Axis};
use crate::core::klippy::motion::kinematics::MoveContext;
use crate::core::klippy::motion::plan::Move;
use crate::core::klippy::motion::queuing::MotionQueuing;
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form exists upstream (`manual_stepper.py:233`).
section!("manual_stepper", order = 20, prefix = load_config_prefix);

/// One configured `[manual_stepper <name>]`.
pub struct ManualStepper {
    /// The full section name (`manual_stepper basic_stepper`), which is also the
    /// stepper name `stepper_enable` knows it by.
    name: String,
    /// The section suffix (`basic_stepper`): `MANUAL_STEPPER STEPPER=<short>`.
    short_name: String,
    /// Whether the section named an endstop (`can_home`).
    can_home: bool,
    /// The motor options and (when present) the endstop.
    stepper: Arc<PrinterStepper>,
    /// `MOVE` default speed, mm/s.
    velocity: f64,
    /// `MOVE` default acceleration, mm/s² (also the homing acceleration).
    accel: f64,
    /// `position_min`, when the section set one.
    pos_min: Option<f64>,
    /// `position_max`, when the section set one.
    pos_max: Option<f64>,
    /// The planner print time this stepper has reached.
    next_cmd_time: Mutex<f64>,
    /// Where the stepper has been commanded to.
    commanded_pos: Mutex<f64>,
    /// The G-Code letter this stepper is registered under, if any.
    axis_gcode_id: Mutex<Option<String>>,
    /// The corner speed `GCODE_AXIS` registered (`INSTANTANEOUS_CORNER_VELOCITY`).
    instant_corner_v: Mutex<f64>,
    /// The `LIMIT_VELOCITY` / `LIMIT_ACCEL` caps `GCODE_AXIS` registered.
    gaxis_limit_velocity: Mutex<f64>,
    gaxis_limit_accel: Mutex<f64>,
    /// The machine, to find the toolhead and `stepper_enable` at command time.
    printer: Weak<Printer>,
}

impl ManualStepper {
    /// Build the section: read the options, build the stepper, install the
    /// cartesian X solver (upstream's `cartesian_stepper_alloc, b'x'`).
    ///
    /// # Errors
    /// A missing or out-of-range option, or any option [`PrinterStepper`]
    /// refuses (a missing `microsteps`/`rotation_distance`, a bad pin, …).
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let name = config.section().identifier();
        let short_name = config
            .section()
            .sub
            .clone()
            .unwrap_or_else(|| config.section().id.clone());
        let can_home = config.has("endstop_pin");

        // The motor rides a bare stepper regardless: with an `endstop_pin` the
        // geometry still just needs the pins (upstream's rail sets
        // `need_position_minmax=False`, which this port's `BareMotor` already
        // means — it reads no `position_max`).
        let stepper = Arc::new(PrinterStepper::with_geometry(
            config,
            printer,
            Axis::X,
            RailGeometry::BareMotor,
        )?);
        stepper.setup_itersolve(
            cartesian_position_fn(Axis::X),
            cartesian_active_flags(Axis::X),
        );

        // `velocity` (default 5, `above=0`) and `accel` (default 0,
        // `minval=0`), read exactly as upstream's `getfloat`s are.
        let velocity =
            config.get_float_bounded("velocity", Some(5.0), None, None, Some(0.0), None)?;
        let accel = config.get_float_bounded("accel", Some(0.0), Some(0.0), None, None, None)?;
        let pos_min = config.get_optional_float("position_min")?;
        let pos_max = config.get_optional_float("position_max")?;

        Ok(Self {
            name,
            short_name,
            can_home,
            stepper,
            velocity,
            accel,
            pos_min,
            pos_max,
            next_cmd_time: Mutex::new(0.0),
            commanded_pos: Mutex::new(0.0),
            axis_gcode_id: Mutex::new(None),
            instant_corner_v: Mutex::new(0.0),
            gaxis_limit_velocity: Mutex::new(0.0),
            gaxis_limit_accel: Mutex::new(0.0),
            printer: Arc::downgrade(printer),
        })
    }

    /// The full section name (`manual_stepper basic_stepper`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Register `MANUAL_STEPPER STEPPER=<short_name>` (`manual_stepper.py:40-44`).
    ///
    /// The mux value is the **short** name (`self.name.split()[1]`), not the
    /// full section name; the handler upgrades a `Weak` back to the object so
    /// the dispatcher does not keep it (and, through it, the machine) alive.
    ///
    /// # Errors
    /// A duplicate mux value or command name.
    pub fn register_commands(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let weak = Arc::downgrade(self);
        let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
            let Some(object) = weak.upgrade() else {
                return Err(CommandError::new("The manual stepper is gone"));
            };
            object.cmd_manual_stepper(gcmd)
        });
        gcode
            .register_mux_command(
                "MANUAL_STEPPER",
                "STEPPER",
                Some(&self.short_name),
                handler,
                Some("Command a manually configured stepper"),
            )
            .map_err(ConfigError::new)?;
        Ok(())
    }

    /// The toolhead, or the "not ready" error upstream would raise looking it up.
    fn toolhead(&self) -> Result<Arc<ToolHeadObject>, CommandError> {
        self.printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>("toolhead"))
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    /// `sync_print_time` (`manual_stepper.py:47-53`): pull the planner's print
    /// time up to this stepper's, dwelling when it is ahead.
    fn sync_print_time(&self) -> Result<(), CommandError> {
        let toolhead = self.toolhead()?;
        let print_time = toolhead.get_last_move_time();
        let next = *self.lock_next_cmd_time();
        if next > print_time {
            toolhead.dwell(next - print_time);
        } else {
            *self.lock_next_cmd_time() = print_time;
        }
        Ok(())
    }

    /// `_submit_move` (`manual_stepper.py:63-72`) minus the trapq append.
    ///
    /// Returns the print time the move ends at. The trapezoid is *not* queued
    /// (see the module docs); only the time and `commanded_pos` advance.
    fn submit_move(&self, movetime: f64, movepos: f64, speed: f64, accel: f64) -> f64 {
        let cp = *self.lock_commanded_pos();
        let dist = movepos - cp;
        let (_axis_r, accel_t, cruise_t, _cruise_v) = calc_move_time(dist, speed, accel);
        *self.lock_commanded_pos() = movepos;
        movetime + accel_t + cruise_t + accel_t
    }

    /// `do_move` (`manual_stepper.py:73-80`).
    fn do_move(
        &self,
        movepos: f64,
        speed: f64,
        accel: f64,
        sync: bool,
    ) -> Result<(), CommandError> {
        self.sync_print_time()?;
        let start = *self.lock_next_cmd_time();
        let end = self.submit_move(start, movepos, speed, accel);
        *self.lock_next_cmd_time() = end;
        if sync {
            self.sync_print_time()?;
        }
        Ok(())
    }

    /// `do_enable` (`manual_stepper.py:54-57`): drive the enable line through
    /// `stepper_enable`.
    fn do_enable(&self, enable: bool) -> Result<(), CommandError> {
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let stepper_enable = printer
            .lookup_object_as::<PrinterStepperEnable>("stepper_enable")
            .ok_or_else(|| CommandError::new("stepper_enable is not registered"))?;
        let names = vec![self.stepper.name().to_string()];
        stepper_enable.set_motors_enable(&names, enable);
        Ok(())
    }

    /// `do_set_position` (`manual_stepper.py:58-62`), minus the flush and the
    /// rail write (module docs).
    fn do_set_position(&self, setpos: f64) {
        *self.lock_commanded_pos() = setpos;
    }

    /// `do_homing_move` (`manual_stepper.py:81-92`).
    ///
    /// A section without an endstop answers with upstream's wording; an
    /// endstop-triggered run is the documented gap (no `homing.manual_home`
    /// in this port).
    fn do_homing_move(&self) -> Result<(), CommandError> {
        if !self.can_home {
            return Err(CommandError::new("No endstop for this manual stepper"));
        }
        Err(CommandError::new(
            "Manual stepper homing is not implemented in this host",
        ))
    }

    /// `cmd_MANUAL_STEPPER` (`manual_stepper.py:94-137`).
    fn cmd_manual_stepper(self: &Arc<Self>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        // `gcmd.get('GCODE_AXIS', None) is not None` — presence, even empty.
        if gcmd.get_command_parameters().contains_key("GCODE_AXIS") {
            return self.command_with_gcode_axis(gcmd);
        }
        if self.lock_axis_gcode_id().is_some() {
            return Err(CommandError::new("Must unregister from gcode axis first"));
        }
        if gcmd.get_command_parameters().contains_key("ENABLE") {
            let enable = gcmd.get_int("ENABLE")?;
            self.do_enable(enable != 0)?;
        }
        if gcmd.get_command_parameters().contains_key("SET_POSITION") {
            let setpos = gcmd.get_float("SET_POSITION")?;
            self.do_set_position(setpos);
        }
        let speed = gcmd.get(
            "SPEED",
            Some(self.velocity),
            parse_float,
            None,
            None,
            Some(0.0),
            None,
        )?;
        let accel = gcmd.get(
            "ACCEL",
            Some(self.accel),
            parse_float,
            Some(0.0),
            None,
            None,
            None,
        )?;
        if gcmd
            .get_command_parameters()
            .contains_key("STOP_ON_ENDSTOP")
        {
            let homing_move = gcmd.get_str("STOP_ON_ENDSTOP")?;
            let homing_move = self.parse_stop_on_endstop(&homing_move)?;
            let _is_probe = homing_move == "probe";
            let movepos = gcmd.get_float("MOVE")?;
            self.check_move_range(movepos)?;
            self.do_homing_move()?;
        } else if gcmd.get_command_parameters().contains_key("MOVE") {
            let movepos = gcmd.get_float("MOVE")?;
            self.check_move_range(movepos)?;
            let sync = gcmd.get_int_default("SYNC", 1)? != 0;
            self.do_move(movepos, speed, accel, sync)?;
        } else if gcmd.get_int_default("SYNC", 0)? != 0 {
            self.sync_print_time()?;
        }
        Ok(())
    }

    /// Translate and validate a `STOP_ON_ENDSTOP` value
    /// (`manual_stepper.py:112-125`): the deprecated numeric forms map to their
    /// names, then `try_` / `inverted_` prefixes strip to a bare
    /// `probe` / `home`.
    ///
    /// # Errors
    /// `Unknown STOP_ON_ENDSTOP request` for anything else.
    fn parse_stop_on_endstop(&self, value: &str) -> Result<String, CommandError> {
        let mapped = match value {
            "-2" => Some("try_inverted_home"),
            "-1" => Some("inverted_home"),
            "1" => Some("home"),
            "2" => Some("try_home"),
            _ => None,
        };
        let value = match mapped {
            Some(name) => {
                if let Some(printer) = self.printer.upgrade() {
                    if let Some(config) =
                        printer.lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
                    {
                        config.deprecate_gcode(
                            "MANUAL_STEPPER",
                            Some("STOP_ON_ENDSTOP"),
                            Some(value),
                            None,
                        );
                    }
                }
                name.to_string()
            }
            None => value.to_string(),
        };
        let rest = value.strip_prefix("try_").unwrap_or(&value);
        let rest = rest.strip_prefix("inverted_").unwrap_or(rest);
        if rest != "probe" && rest != "home" {
            return Err(CommandError::new("Unknown STOP_ON_ENDSTOP request"));
        }
        Ok(rest.to_string())
    }

    /// The `Move out of range` bounds check (`manual_stepper.py:126,133`).
    fn check_move_range(&self, movepos: f64) -> Result<(), CommandError> {
        if self.pos_min.is_some_and(|min| movepos < min)
            || self.pos_max.is_some_and(|max| movepos > max)
        {
            return Err(CommandError::new("Move out of range"));
        }
        Ok(())
    }

    /// `command_with_gcode_axis` (`manual_stepper.py:139-167`).
    fn command_with_gcode_axis(self: &Arc<Self>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let requested = gcmd.get_str("GCODE_AXIS")?;
        let instant_corner_v = gcmd.get(
            "INSTANTANEOUS_CORNER_VELOCITY",
            Some(1.0),
            parse_float,
            Some(0.0),
            None,
            None,
            None,
        )?;
        let limit_velocity = gcmd.get(
            "LIMIT_VELOCITY",
            Some(999999.9),
            parse_float,
            None,
            None,
            Some(0.0),
            None,
        )?;
        let limit_accel = gcmd.get(
            "LIMIT_ACCEL",
            Some(999999.9),
            parse_float,
            None,
            None,
            Some(0.0),
            None,
        )?;

        let current = self.lock_axis_gcode_id().clone();
        let existing: Vec<String> = self
            .toolhead()?
            .get_extra_axes()
            .iter()
            .filter_map(|axis| axis.axis_gcode_id())
            .collect();
        match plan_gcode_axis(current.as_deref(), &requested, &existing)? {
            GcodeAxisPlan::Noop => Ok(()),
            GcodeAxisPlan::Unregister => {
                let me: Arc<dyn ExtraAxis> = self.clone();
                self.toolhead()?.remove_extra_axis(&me)?;
                *self.lock_axis_gcode_id() = None;
                Ok(())
            }
            GcodeAxisPlan::Register(id) => {
                *self.lock_instant_corner_v() = instant_corner_v;
                *self.lock_gaxis_limit_velocity() = limit_velocity;
                *self.lock_gaxis_limit_accel() = limit_accel;
                *self.lock_axis_gcode_id() = Some(id);
                let me: Arc<dyn ExtraAxis> = self.clone();
                self.toolhead()?.add_extra_axis(me)?;
                Ok(())
            }
        }
    }

    fn lock_next_cmd_time(&self) -> MutexGuard<'_, f64> {
        self.next_cmd_time
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    fn lock_commanded_pos(&self) -> MutexGuard<'_, f64> {
        self.commanded_pos
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    fn lock_axis_gcode_id(&self) -> MutexGuard<'_, Option<String>> {
        self.axis_gcode_id
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    fn lock_instant_corner_v(&self) -> MutexGuard<'_, f64> {
        self.instant_corner_v
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    fn lock_gaxis_limit_velocity(&self) -> MutexGuard<'_, f64> {
        self.gaxis_limit_velocity
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    fn lock_gaxis_limit_accel(&self) -> MutexGuard<'_, f64> {
        self.gaxis_limit_accel
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

/// What a `GCODE_AXIS` request means, once validated
/// (`manual_stepper.py:141-167`).
#[derive(Debug, PartialEq, Eq)]
enum GcodeAxisPlan {
    /// An empty value while nothing is registered: upstream returns silently.
    Noop,
    /// An empty value while an axis is registered: take it off the toolhead.
    Unregister,
    /// A valid, unused letter, **already upper-cased**: register it.
    Register(String),
}

/// Decide what a `GCODE_AXIS=<requested>` does, given the axis already bound
/// (if any) and the letters other extra axes hold.
///
/// The validation runs **after** `requested.to_uppercase()`, matching upstream
/// (`manual_stepper.py:152`), so `a` registers as `A`; `F`, multi-character
/// values, and non-letters are refused. [`GcodeAxisPlan::Register`] carries the
/// upper-cased letter, which is what the caller binds.
fn plan_gcode_axis(
    current: Option<&str>,
    requested: &str,
    existing: &[String],
) -> Result<GcodeAxisPlan, CommandError> {
    let gcode_axis = requested.to_uppercase();
    if current.is_some() {
        if gcode_axis.is_empty() {
            // Request to unregister a registered axis.
            return Ok(GcodeAxisPlan::Unregister);
        }
        return Err(CommandError::new("Must unregister axis first"));
    }
    if gcode_axis.chars().count() != 1
        || !gcode_axis.chars().all(|c| c.is_uppercase())
        || "XYZEFN".contains(&gcode_axis)
    {
        if gcode_axis.is_empty() {
            // Request to unregister an already unregistered axis.
            return Ok(GcodeAxisPlan::Noop);
        }
        return Err(CommandError::new("Not a valid GCODE_AXIS"));
    }
    if existing.iter().any(|id| id == &gcode_axis) {
        return Err(CommandError::new(format!(
            "Axis '{gcode_axis}' already registered"
        )));
    }
    Ok(GcodeAxisPlan::Register(gcode_axis))
}

impl ExtraAxis for ManualStepper {
    fn name(&self) -> &str {
        &self.name
    }

    fn check_move(&self, ctx: &mut MoveContext<'_>, ea_index: usize) -> Result<(), CommandError> {
        let movepos = ctx.end_pos()[ea_index];
        if self.pos_min.is_some_and(|min| movepos < min)
            || self.pos_max.is_some_and(|max| movepos > max)
        {
            return Err(ctx.out_of_range());
        }
        let axis_ratio = ctx.move_d() / ctx.axes_d()[ea_index].abs();
        let limit_velocity = *self.lock_gaxis_limit_velocity() * axis_ratio;
        let mut limit_accel = *self.lock_gaxis_limit_accel() * axis_ratio;
        if !ctx.is_kinematic_move() && self.accel != 0.0 {
            limit_accel = limit_accel.min(self.accel * axis_ratio);
        }
        ctx.limit_speed(limit_velocity, limit_accel);
        Ok(())
    }

    fn calc_junction(&self, prev: &Move, cur: &Move, ea_index: usize) -> f64 {
        let diff_r = cur.axes_r[ea_index] - prev.axes_r[ea_index];
        if diff_r != 0.0 {
            (*self.lock_instant_corner_v() / diff_r.abs()).powi(2)
        } else {
            cur.max_cruise_v2
        }
    }

    fn process_move(
        &self,
        _queuing: &mut MotionQueuing,
        _print_time: f64,
        move_: &Move,
        ea_index: usize,
    ) {
        // Upstream appends to this stepper's own trapq; this port has none, so
        // only the commanded position follows the planned move (module docs).
        *self.lock_commanded_pos() = move_.end_pos[ea_index];
    }

    fn find_past_position(&self, _print_time: f64) -> f64 {
        *self.lock_commanded_pos()
    }

    fn get_status(&self) -> Value {
        json!({})
    }

    fn axis_gcode_id(&self) -> Option<String> {
        self.lock_axis_gcode_id().clone()
    }
}

impl PrinterObject for ManualStepper {
    /// Upstream's `ManualStepper` defines no `get_status`, so it is not in
    /// `objects/list`; this keeps the same surface.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for ManualStepper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ManualStepper")
            .field("name", &self.name)
            .field("can_home", &self.can_home)
            .finish_non_exhaustive()
    }
}

/// The factory the section declaration names (`manual_stepper.py:233`).
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = Arc::new(ManualStepper::new(config, printer)?);
    object.register_commands(printer)?;
    Ok(object)
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

    /// The corpus's `basic_stepper` body (`test/klippy/manual_stepper.cfg:2-9`).
    const BASIC: &str = "step_pin: PF0\ndir_pin: PF1\nenable_pin: !PD7\n\
        microsteps: 16\nrotation_distance: 40\nvelocity: 7\naccel: 500\n";

    /// A `kinematics: none` printer whose `[manual_stepper basic_stepper]`
    /// carries `section` as its options.
    fn config(section: &str) -> String {
        format!(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [printer]\nkinematics: none\nmax_velocity: 300\nmax_accel: 3000\n\
             [manual_stepper basic_stepper]\n{section}"
        )
    }

    fn load(text: &str) -> (Arc<Printer>, Result<(), ConfigError>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(text).expect("the config parses");
        let result = printer.load_config(&config);
        (printer, result)
    }

    fn load_ok(text: &str) -> Arc<Printer> {
        let (printer, result) = load(text);
        result.expect("the config loads");
        printer
    }

    fn the_stepper(printer: &Arc<Printer>) -> Arc<ManualStepper> {
        printer
            .lookup_object_as::<ManualStepper>("manual_stepper basic_stepper")
            .expect("[manual_stepper basic_stepper] is registered")
    }

    /// The corpus's options are all read (the undefined-option check is what
    /// fails when one is not), and the velocity/accel defaults follow upstream.
    #[test]
    fn test_the_prefix_section_reads_every_corpus_option() {
        let printer = load_ok(&config(BASIC));
        let stepper = the_stepper(&printer);
        assert_eq!(stepper.name(), "manual_stepper basic_stepper");
        assert_eq!(stepper.short_name, "basic_stepper");
        assert_eq!(stepper.velocity, 7.0);
        assert_eq!(stepper.accel, 500.0);
        assert!(!stepper.can_home);
        assert_eq!(stepper.pos_min, None);
        assert_eq!(stepper.pos_max, None);
    }

    /// `velocity` defaults to 5 and `accel` to 0 (`manual_stepper.py:23-24`).
    #[test]
    fn test_the_velocity_and_accel_defaults_follow_upstream() {
        let printer = load_ok(&config(
            "step_pin: PF0\ndir_pin: PF1\nmicrosteps: 16\nrotation_distance: 40\n",
        ));
        let stepper = the_stepper(&printer);
        assert_eq!(stepper.velocity, 5.0);
        assert_eq!(stepper.accel, 0.0);
    }

    /// An `endstop_pin` sets `can_home` and reads the endstop
    /// (`manual_stepper.py:14-18`).
    #[test]
    fn test_an_endstop_pin_marks_the_stepper_homeable() {
        let printer = load_ok(&config(&format!(
            "step_pin: PF6\ndir_pin: !PF7\nmicrosteps: 16\nrotation_distance: 40\n\
             endstop_pin: ^PJ1\n"
        )));
        let stepper = the_stepper(&printer);
        assert!(stepper.can_home);
        assert!(stepper.stepper.endstop().is_some());
    }

    /// `SPEED`/`ACCEL` fall back to the section's `velocity`/`accel`
    /// (`manual_stepper.py:106-107`). The dwell-only timeline leaves
    /// `next_cmd_time` at the trapezoid's duration, which is the observable.
    #[test]
    fn test_speed_and_accel_default_to_the_section_values() {
        let printer = load_ok(&config(BASIC));
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        // `MOVE=10 SPEED=10` — the explicit pair.
        gcode
            .run_script_sync("MANUAL_STEPPER STEPPER=basic_stepper MOVE=10 SPEED=10 ACCEL=100")
            .unwrap();
        let stepper = the_stepper(&printer);
        let explicit = *stepper.lock_next_cmd_time();
        // SPEED=10 ACCEL=100 over 10mm: accel_t=0.1, accel_decel_d=1, cruise_t=0.9.
        assert!((explicit - 1.1).abs() < 1e-9, "{explicit}");

        // A second move at the defaults: velocity=7, accel=500 over 5mm. The
        // timeline is cumulative, so measure the increment.
        let before = {
            let stepper = the_stepper(&printer);
            gcode
                .run_script_sync("MANUAL_STEPPER STEPPER=basic_stepper SET_POSITION=0")
                .unwrap();
            let before = *stepper.lock_next_cmd_time();
            before
        };
        gcode
            .run_script_sync("MANUAL_STEPPER STEPPER=basic_stepper MOVE=5")
            .unwrap();
        let stepper = the_stepper(&printer);
        let delta = *stepper.lock_next_cmd_time() - before;
        // velocity=7, accel=500: accel_t=0.014, accel_decel_d=0.098,
        // cruise_t=(5-0.098)/7=0.7002857…, so 2*0.014 + 0.7002857… = 0.7282857…
        let expected = 2.0 * (7.0 / 500.0) + (5.0 - (7.0 / 500.0) * 7.0) / 7.0;
        assert!((delta - expected).abs() < 1e-9, "{delta} vs {expected}");
    }

    /// A `MOVE` outside `position_min`/`position_max` is refused with upstream's
    /// wording (`manual_stepper.py:126,133`).
    #[test]
    fn test_a_move_out_of_range_is_refused() {
        let printer = load_ok(&config(&format!("{BASIC}position_max: 100\n")));
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        let err = gcode
            .run_script_sync("MANUAL_STEPPER STEPPER=basic_stepper MOVE=300")
            .unwrap_err();
        assert!(err.to_string().contains("Move out of range"), "{err}");
    }

    /// A section without an endstop refuses `STOP_ON_ENDSTOP`
    /// (`manual_stepper.py:82-84`), and an unknown request is refused too
    /// (`:121`).
    #[test]
    fn test_stop_on_endstop_needs_an_endstop() {
        let printer = load_ok(&config(BASIC));
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        let err = gcode
            .run_script_sync("MANUAL_STEPPER STEPPER=basic_stepper STOP_ON_ENDSTOP=1 MOVE=10")
            .unwrap_err();
        assert!(
            err.to_string()
                .contains("No endstop for this manual stepper"),
            "{err}"
        );

        // Upstream maps the old numeric `1` to `home` and accepts it; an
        // unknown name is not in `["probe", "home"]`.
        let err = gcode
            .run_script_sync("MANUAL_STEPPER STEPPER=basic_stepper STOP_ON_ENDSTOP=nope MOVE=10")
            .unwrap_err();
        assert!(
            err.to_string().contains("Unknown STOP_ON_ENDSTOP request"),
            "{err}"
        );
    }

    /// The four `GCODE_AXIS` transitions (`manual_stepper.py:141-167`). The
    /// validation happens after the value is upper-cased (`:152`), so `a` is
    /// accepted as `A`.
    #[test]
    fn test_the_gcode_axis_transitions_follow_upstream() {
        // Already registered, a non-empty request: "Must unregister axis first".
        let err = plan_gcode_axis(Some("A"), "A", &[]).unwrap_err();
        assert_eq!(err.to_string(), "Must unregister axis first");

        // A lowercase letter is accepted, bound as its upper case.
        assert_eq!(
            plan_gcode_axis(None, "a", &[]).unwrap(),
            GcodeAxisPlan::Register("A".to_string())
        );

        // F, multi-character, and non-letter values are refused.
        for bad in ["F", "AB", "1", "-"] {
            let err = plan_gcode_axis(None, bad, &[]).unwrap_err();
            assert_eq!(
                err.to_string(),
                "Not a valid GCODE_AXIS",
                "requested {bad:?}"
            );
        }

        // A letter another extra axis holds: "Axis 'A' already registered".
        let err = plan_gcode_axis(None, "A", &["A".to_string()]).unwrap_err();
        assert_eq!(err.to_string(), "Axis 'A' already registered");

        // Empty value, registered: unregister. Empty value, not registered:
        // silently nothing. Then the same letter registers again.
        assert_eq!(
            plan_gcode_axis(Some("A"), "", &[]).unwrap(),
            GcodeAxisPlan::Unregister
        );
        assert_eq!(plan_gcode_axis(None, "", &[]).unwrap(), GcodeAxisPlan::Noop);
        assert_eq!(
            plan_gcode_axis(None, "A", &[]).unwrap(),
            GcodeAxisPlan::Register("A".to_string())
        );
    }
}
