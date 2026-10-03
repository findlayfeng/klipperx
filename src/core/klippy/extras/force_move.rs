//! `[force_move]` — move one stepper by hand, and the timing math that needs.
//!
//! Upstream's `klippy/extras/force_move.py`: a utility module for manually
//! driven moves. It registers a `STEPPER_BUZZ` for every configured stepper and
//! — when `enable_force_move` is set — a `FORCE_MOVE`; both drive a single motor
//! outside the planner and the kinematics. It also owns
//! [`calc_move_time`], which the manual stepper uses to turn a
//! distance/speed/acceleration into a trapezoid (`manual_stepper.py:66`).
//!
//! | command | registered when | meaning |
//! |---|---|---|
//! | `STEPPER_BUZZ STEPPER=<name>` | always, per stepper | oscillate a motor 10 times to identify it |
//! | `FORCE_MOVE STEPPER=<name> DISTANCE= VELOCITY= [ACCEL=]` | `enable_force_move` | move a motor in its own coordinates |
//!
//! The commands are **mux** commands keyed on `STEPPER`, one value per stepper,
//! registered as the sections are built (`force_move.py:48-59`), so an unknown
//! name is refused by the mux table ("The value '…' is not valid for STEPPER").
//! The object itself is created on first use when the config named no
//! `[force_move]` section ([`ForceMove::ensure`]): every stepper registers
//! through `PrinterStepper` (`klippy/stepper.py:282-285`), so every config with
//! a motor gets `STEPPER_BUZZ`.
//!
//! # The move itself
//!
//! `manual_move` is upstream's `ForceMove.manual_move` (`force_move.py:75-91`),
//! carried by [`ToolHeadObject::manual_move`]: flush, swap in a cartesian
//! single-axis solver and a force-move-only trapq, zero the solver's position,
//! append the trapezoid, dwell its duration, generate the steps, then restore
//! the solver and trapq and wipe the force-move queue. The toolhead's commanded
//! position is deliberately untouched — the move invalidates the kinematics,
//! and a `SET_KINEMATIC_POSITION` re-syncs it.
//!
//! Two adaptations to this host's step generation:
//!
//! * the second flush generates the **target stepper alone**
//!   (`MotionQueuing::generate_stepper`), to the move's end rather than the
//!   background horizon. Upstream's `flush_step_generation` generates every
//!   stepper to the content end; here that would advance other steppers' solvers
//!   past the horizon every later pass is bounded by, and the next pass would
//!   rewind them onto moves already generated.
//! * a stepper `force_move` knows but the toolhead's motion queue does not is
//!   carried on the timeline only. This port builds a `PrinterStepper` for the
//!   manual stepper and the IDEX second carriage without adding it to the
//!   motion queue (a documented gap of those modules), so there is no motor to
//!   drive; upstream would move it. `STEPPER_BUZZ` / `FORCE_MOVE` still answer.
//!
//! `SET_KINEMATIC_POSITION` is **not** registered here: upstream registers it
//! from `force_move` (only with `enable_force_move`), but this port's toolhead
//! owns it unconditionally (`extras::toolhead`), so registering it again would
//! collide. That is the one deliberate shape difference; the command's behavior
//! is the toolhead's.
//!
//! `motion_queuing.note_mcu_movequeue_activity` has no equivalent: this host's
//! step generation is driven by a fixed 10 ms tick rather than by a
//! queue-length estimate, so there is no low-water mark to note.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};
use tracing::info;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::stepper_enable::PrinterStepperEnable;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    parse_float, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

/// A buzz's travel, millimetres, and its speed (`BUZZ_DISTANCE`,
/// `BUZZ_VELOCITY`, `force_move.py:9-12`): one millimetre, moving off in a
/// quarter second.
pub const BUZZ_DISTANCE: f64 = 1.0;
/// The buzz speed for a millimetre-mode stepper, mm/s.
pub const BUZZ_VELOCITY: f64 = BUZZ_DISTANCE / 0.250;
/// A buzz's travel for a degrees-mode (radians) stepper: one degree.
pub const BUZZ_RADIANS_DISTANCE: f64 = std::f64::consts::PI / 180.0;
/// The buzz speed for a degrees-mode stepper, rad/s.
pub const BUZZ_RADIANS_VELOCITY: f64 = BUZZ_RADIANS_DISTANCE / 0.250;

/// How many forward/reverse pairs `STEPPER_BUZZ` runs (`for i in range(10)`).
const BUZZ_CYCLES: usize = 10;
/// The hold after the forward buzz, seconds (`force_move.py:103`).
const BUZZ_FORWARD_DWELL: f64 = 0.050;
/// The hold after the reverse buzz, seconds (`force_move.py:105`).
const BUZZ_REVERSE_DWELL: f64 = 0.450;

/// The printer-object name under which the loader registers `[force_move]`.
const FORCE_MOVE_OBJECT: &str = "force_move";

/// Calculate a move's `(axis_r, accel_t, cruise_t, cruise_v)` trapezoid
/// (`force_move.calc_move_time`, `force_move.py:15-28`).
///
/// `dist` is signed: a negative distance flips `axis_r` and the magnitude is
/// used for the profile. With no acceleration, or no distance, the move is a
/// constant-speed cruise at the requested `speed` (`accel_t` is zero). The
/// capped case slows `cruise_v` to the speed the distance can reach under
/// `accel`, so the trapezoid degenerates to a triangle with no cruise phase.
pub fn calc_move_time(dist: f64, speed: f64, accel: f64) -> (f64, f64, f64, f64) {
    let mut axis_r = 1.0;
    let mut dist = dist;
    if dist < 0.0 {
        axis_r = -1.0;
        dist = -dist;
    }
    if accel == 0.0 || dist == 0.0 {
        return (axis_r, 0.0, dist / speed, speed);
    }
    let mut speed = speed;
    let max_cruise_v2 = dist * accel;
    if max_cruise_v2 < speed.powf(2.0) {
        speed = max_cruise_v2.sqrt();
    }
    let accel_t = speed / accel;
    let cruise_t = (dist - accel_t * speed) / speed;
    (axis_r, accel_t, cruise_t, speed)
}

// The `[force_move]` section. Loaded before every section that builds a stepper
// (`extruder`/`manual_stepper` at order 20, `stepper_*` at 50), so an explicit
// section's `enable_force_move` is settled before the first stepper registers
// with it.
section!("force_move", order = 18, load = load_config);

/// One `[force_move]` section, and the per-stepper command registrations.
pub struct ForceMove {
    /// Whether `FORCE_MOVE` is offered (`enable_force_move`, default false).
    enable_force_move: bool,
    /// Every stepper that registered, mapped to whether its units are radians.
    /// Upstream keeps the `MCU_stepper` objects (`force_move.py:31,48-50`); this
    /// port only needs the name (to validate and to move) and the radians flag
    /// (to pick the buzz distance).
    steppers: Mutex<HashMap<String, bool>>,
    /// The machine, to find the toolhead and `stepper_enable` at command time.
    printer: Weak<Printer>,
}

impl ForceMove {
    fn new(enable_force_move: bool, printer: &Arc<Printer>) -> Self {
        Self {
            enable_force_move,
            steppers: Mutex::new(HashMap::new()),
            printer: Arc::downgrade(printer),
        }
    }

    /// The `force_move` object, creating it if the config named no section.
    ///
    /// Upstream's `PrinterStepper` calls `printer.load_object(config,
    /// 'force_move')` for every stepper (`klippy/stepper.py:282-285`), so a
    /// config with no `[force_move]` still gets the object and its
    /// `STEPPER_BUZZ`. The created object registers itself in the printer
    /// registry, exactly as the `[force_move]` factory would.
    pub fn ensure(printer: &Arc<Printer>) -> Arc<Self> {
        if let Some(existing) = printer.lookup_object_as::<Self>(FORCE_MOVE_OBJECT) {
            return existing;
        }
        let object = Arc::new(Self::new(false, printer));
        printer
            .add_object(FORCE_MOVE_OBJECT, object.clone())
            .expect("`force_move` is registered once per machine");
        object
    }

    /// Register a stepper's `STEPPER_BUZZ`, and a `FORCE_MOVE` when enabled
    /// (`force_move.py:48-59`).
    ///
    /// `units_in_radians` is the stepper's
    /// [`PrinterStepper::units_in_radians`](crate::core::klippy::extras::stepper::PrinterStepper::units_in_radians),
    /// read at registration because that is where upstream reads the
    /// `MCU_stepper`.
    ///
    /// # Errors
    /// A missing `gcode` object, or a duplicate mux value or command name.
    pub fn register_stepper(
        self: &Arc<Self>,
        name: &str,
        units_in_radians: bool,
    ) -> Result<(), ConfigError> {
        self.lock_steppers()
            .insert(name.to_string(), units_in_radians);
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| ConfigError::new("force_move has no printer"))?;
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .ok_or_else(|| ConfigError::new("the gcode dispatcher is not registered"))?;

        let weak = Arc::downgrade(self);
        let buzz: CommandHandler = Arc::new(move |gcmd: &GcodeCommand| {
            let weak = weak.clone();
            Box::pin(async move {
                let object = weak
                    .upgrade()
                    .ok_or_else(|| CommandError::new("The force move object is gone"))?;
                object.cmd_stepper_buzz(gcmd).await
            })
        });
        gcode
            .register_mux_command(
                "STEPPER_BUZZ",
                "STEPPER",
                Some(name),
                buzz,
                Some("Oscillate a given stepper to help id it"),
            )
            .map_err(ConfigError::new)?;

        if self.enable_force_move {
            let weak = Arc::downgrade(self);
            let force: CommandHandler = Arc::new(move |gcmd: &GcodeCommand| {
                let weak = weak.clone();
                Box::pin(async move {
                    let object = weak
                        .upgrade()
                        .ok_or_else(|| CommandError::new("The force move object is gone"))?;
                    object.cmd_force_move(gcmd).await
                })
            });
            gcode
                .register_mux_command(
                    "FORCE_MOVE",
                    "STEPPER",
                    Some(name),
                    force,
                    Some("Manually move a stepper; invalidates kinematics"),
                )
                .map_err(ConfigError::new)?;
        }
        Ok(())
    }

    /// Upstream's `lookup_stepper` (`force_move.py:60-63`): the stepper's
    /// radians flag, or the error upstream raises for a name it does not know.
    ///
    /// The mux table already filters by value, so this only fires for a caller
    /// that asks by name (a future `tmc`/`angle`); it is kept for that and for
    /// upstream's `Unknown stepper` wording.
    fn lookup_stepper(&self, name: &str) -> Result<bool, CommandError> {
        self.lock_steppers()
            .get(name)
            .copied()
            .ok_or_else(|| CommandError::new(format!("Unknown stepper {name}")))
    }

    /// The toolhead, or the "not ready" error upstream would raise looking it
    /// up.
    fn toolhead(&self) -> Result<Arc<ToolHeadObject>, CommandError> {
        self.printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>("toolhead"))
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    /// `_force_enable` (`force_move.py:64-68`): turn the motor on, reporting
    /// whether the call changed it.
    fn force_enable(&self, name: &str) -> Result<bool, CommandError> {
        let stepper_enable = self.stepper_enable()?;
        Ok(stepper_enable.set_motors_enable(&[name.to_string()], true))
    }

    /// `_restore_enable` (`force_move.py:69-74`): turn the motor off again, but
    /// only when `_force_enable` was the one that turned it on.
    fn restore_enable(&self, name: &str, did_enable: bool) -> Result<(), CommandError> {
        if !did_enable {
            return Ok(());
        }
        let stepper_enable = self.stepper_enable()?;
        stepper_enable.set_motors_enable(&[name.to_string()], false);
        Ok(())
    }

    fn stepper_enable(&self) -> Result<Arc<PrinterStepperEnable>, CommandError> {
        self.printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<PrinterStepperEnable>("stepper_enable"))
            .ok_or_else(|| CommandError::new("stepper_enable is not registered"))
    }

    /// `cmd_STEPPER_BUZZ` (`force_move.py:92-106`): ten forward/reverse pairs of
    /// a one-millimetre (or one-degree) move, at four units a second.
    async fn cmd_stepper_buzz(self: &Arc<Self>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let name = gcmd.get_str("STEPPER")?;
        let units_in_radians = self.lookup_stepper(&name)?;
        info!("Stepper buzz {name}");
        let did_enable = self.force_enable(&name)?;
        let toolhead = self.toolhead()?;
        let (dist, speed) = if units_in_radians {
            (BUZZ_RADIANS_DISTANCE, BUZZ_RADIANS_VELOCITY)
        } else {
            (BUZZ_DISTANCE, BUZZ_VELOCITY)
        };
        for _ in 0..BUZZ_CYCLES {
            toolhead.manual_move(&name, dist, speed, 0.0).await?;
            toolhead.dwell(BUZZ_FORWARD_DWELL);
            toolhead.manual_move(&name, -dist, speed, 0.0).await?;
            toolhead.dwell(BUZZ_REVERSE_DWELL);
        }
        self.restore_enable(&name, did_enable)?;
        Ok(())
    }

    /// `cmd_FORCE_MOVE` (`force_move.py:107-116`): one move, with `DISTANCE`
    /// required, `VELOCITY` above zero, and `ACCEL` at or above zero.
    async fn cmd_force_move(self: &Arc<Self>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let name = gcmd.get_str("STEPPER")?;
        self.lookup_stepper(&name)?;
        let distance = gcmd.get_float("DISTANCE")?;
        let speed = gcmd.get_float_bounded("VELOCITY", Some(0.0), None)?;
        let accel = gcmd.get("ACCEL", Some(0.0), parse_float, Some(0.0), None, None, None)?;
        info!("FORCE_MOVE {name} distance={distance:.3} velocity={speed:.3} accel={accel:.3}");
        self.force_enable(&name)?;
        self.toolhead()?
            .manual_move(&name, distance, speed, accel)
            .await?;
        Ok(())
    }

    fn lock_steppers(&self) -> MutexGuard<'_, HashMap<String, bool>> {
        self.steppers
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl PrinterObject for ForceMove {
    /// Upstream's `ForceMove` defines no `get_status`, so it is not in
    /// `objects/list`; this keeps the same surface.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for ForceMove {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ForceMove")
            .field("enable_force_move", &self.enable_force_move)
            .finish_non_exhaustive()
    }
}

/// The factory `[force_move]` names (`force_move.py:139-140`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let enable_force_move = config.get_bool("enable_force_move", Some(false))?;
    Ok(Arc::new(ForceMove::new(enable_force_move, printer)))
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
    /// A `kinematics: none` printer with one `[manual_stepper buzzer]`, so a
    /// stepper registers without a full cartesian machine; `section` is the
    /// optional `[force_move]` body.
    fn config(section: &str) -> String {
        format!(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [printer]\nkinematics: none\nmax_velocity: 300\nmax_accel: 3000\n\
             [manual_stepper buzzer]\nstep_pin: PF0\ndir_pin: PF1\n\
             microsteps: 16\nrotation_distance: 40\n{section}"
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

    /// A loaded, **ready** printer: G-code commands only run once `klippy:ready`
    /// has fired (the dispatcher answers "Starting up" before that).
    fn ready(text: &str) -> Arc<Printer> {
        let printer = load_ok(text);
        printer.send_event(&KlippyEvent::KlippyReady);
        printer
    }

    fn dispatcher(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`")
    }

    /// The loudest stepper name the mux table knows, `.`-trimmed for the
    /// message assertions.
    fn stepper_name() -> &'static str {
        "manual_stepper buzzer"
    }

    /// The two profiles the upstream function is documented on: a negative
    /// distance flips `axis_r` and a distance too short to reach `speed` slows
    /// the cruise (the triangle case).
    #[test]
    fn test_calc_move_time_matches_the_upstream_formula() {
        // `calc_move_time(-2., 10., 100.)` in CPython returns
        // `(-1.0, 0.1, 0.1, 10.0)`.
        assert_eq!(calc_move_time(-2.0, 10.0, 100.0), (-1.0, 0.1, 0.1, 10.0));
    }

    /// A distance that cannot reach `speed` slows the cruise to the speed
    /// `sqrt(dist * accel)` and leaves no cruise phase.
    #[test]
    fn test_a_short_move_slows_the_cruise_to_the_reachable_speed() {
        let (axis_r, accel_t, cruise_t, cruise_v) = calc_move_time(2.0, 100.0, 100.0);
        assert_eq!(axis_r, 1.0);
        let expected = 200.0f64.sqrt();
        assert!(
            (cruise_v - expected).abs() < 1e-12,
            "{cruise_v} vs {expected}"
        );
        assert!((accel_t - expected / 100.0).abs() < 1e-12, "{accel_t}");
        assert!(cruise_t.abs() < 1e-12, "{cruise_t}");
    }

    /// No acceleration is a constant-speed cruise; no distance is a no-op.
    #[test]
    fn test_no_accel_or_no_distance_is_a_plain_cruise() {
        assert_eq!(calc_move_time(5.0, 10.0, 0.0), (1.0, 0.0, 0.5, 10.0));
        assert_eq!(calc_move_time(0.0, 10.0, 100.0), (1.0, 0.0, 0.0, 10.0));
    }

    /// The number the manual-stepper corpus move uses (`MOVE=300 SPEED=10
    /// ACCEL=2000`): the distance is long enough to reach `speed`, so the
    /// cruise is 10 mm/s for the bulk of the move.
    ///
    /// The expected tuple is CPython's, value for value: running the upstream
    /// function itself (`force_move.py:15-28` under `python3`) on
    /// `calc_move_time(300., 10., 2000.)` returns
    /// `(1.0, 0.005, 29.994999999999997, 10.0)`, and every field here compares
    /// equal to it bit for bit rather than within a tolerance.
    #[test]
    fn test_the_long_manual_move_is_a_full_trapezoid() {
        assert_eq!(
            calc_move_time(300.0, 10.0, 2000.0),
            (1.0, 0.005, 29.994_999_999_999_997, 10.0)
        );
    }

    /// The buzz constants are upstream's, value for value
    /// (`force_move.py:9-12`), and a buzz move is a constant-speed cruise:
    /// one millimetre at 4 mm/s, no acceleration (`calc_move_time`'s `accel ==
    /// 0` branch).
    #[test]
    fn test_buzz_constants_and_profile_match_upstream() {
        assert_eq!(BUZZ_DISTANCE, 1.0);
        assert_eq!(BUZZ_VELOCITY, 4.0);
        assert_eq!(BUZZ_RADIANS_DISTANCE, 0.017_453_292_519_943_295);
        assert_eq!(BUZZ_RADIANS_VELOCITY, BUZZ_RADIANS_DISTANCE / 0.250);
        // A buzz move: dist=1, speed=4, no accel → a 0.25 s cruise, speed 4.
        assert_eq!(
            calc_move_time(1.0, BUZZ_VELOCITY, 0.0),
            (1.0, 0.0, 0.25, 4.0)
        );
        // The reverse half is the same move with `axis_r` flipped.
        assert_eq!(
            calc_move_time(-1.0, BUZZ_VELOCITY, 0.0),
            (-1.0, 0.0, 0.25, 4.0)
        );
    }

    /// `STEPPER_BUZZ` is offered for every stepper, with or without
    /// `enable_force_move`; `FORCE_MOVE` only with it (`force_move.py:43-59`).
    #[test]
    fn test_force_move_is_gated_on_enable_force_move() {
        let without = load_ok(&config(""));
        let gcode = dispatcher(&without);
        assert!(gcode.command_exists("STEPPER_BUZZ"));
        assert!(!gcode.command_exists("FORCE_MOVE"));

        let with = load_ok(&config("[force_move]\nenable_force_move: True\n"));
        let gcode = dispatcher(&with);
        assert!(gcode.command_exists("STEPPER_BUZZ"));
        assert!(gcode.command_exists("FORCE_MOVE"));

        // The default is off (`getboolean("enable_force_move", False)`), so an
        // explicit section that does not set it keeps `FORCE_MOVE` away.
        let default = load_ok(&config("[force_move]\n"));
        assert!(!dispatcher(&default).command_exists("FORCE_MOVE"));
    }

    /// An unknown stepper is refused by the mux table, which lists what is
    /// available (`force_move.py:51-52` registers one value per stepper).
    #[test]
    fn test_an_unknown_stepper_name_is_refused_by_the_mux() {
        let printer = ready(&config("[force_move]\nenable_force_move: True\n"));
        let gcode = dispatcher(&printer);

        let err = gcode
            .run_script_sync("STEPPER_BUZZ STEPPER=nope")
            .unwrap_err();
        assert!(
            err.to_string().contains("is not valid for STEPPER"),
            "{err}"
        );
        let err = gcode
            .run_script_sync("FORCE_MOVE STEPPER=nope DISTANCE=1 VELOCITY=1")
            .unwrap_err();
        assert!(
            err.to_string().contains("is not valid for STEPPER"),
            "{err}"
        );
    }

    /// `FORCE_MOVE`'s parameter rules (`force_move.py:110-112`): `DISTANCE` is
    /// required, `VELOCITY` must be above zero, and `ACCEL` may not be below
    /// zero. The checks run before the move, so no connected toolhead is
    /// needed.
    #[test]
    fn test_force_move_argument_bounds_match_upstream() {
        let printer = ready(&config("[force_move]\nenable_force_move: True\n"));
        let gcode = dispatcher(&printer);
        let stepper = stepper_name();

        let err = gcode
            .run_script_sync(&format!("FORCE_MOVE STEPPER=\"{stepper}\" VELOCITY=10"))
            .unwrap_err();
        assert!(err.to_string().contains("DISTANCE"), "{err}");

        // `above=0.` upstream (`force_move.py:111`). This port's shared `get`
        // prints an `above` bound as "must have above of …" where upstream
        // prints "must be above …" (`gcode.rs`; the same known divergence
        // `temperature_probe` works around), so only the bound itself is
        // pinned here.
        let err = gcode
            .run_script_sync(&format!(
                "FORCE_MOVE STEPPER=\"{stepper}\" DISTANCE=10 VELOCITY=0"
            ))
            .unwrap_err();
        assert!(
            err.to_string().contains("VELOCITY must have above of 0"),
            "{err}"
        );

        // `ACCEL` defaults to 0 with `minval=0.` (`force_move.py:112`), whose
        // wording is upstream's.
        let err = gcode
            .run_script_sync(&format!(
                "FORCE_MOVE STEPPER=\"{stepper}\" DISTANCE=10 VELOCITY=10 ACCEL=-1"
            ))
            .unwrap_err();
        assert!(
            err.to_string().contains("ACCEL must have minimum of 0"),
            "{err}"
        );
    }

    /// The radians flag reaches the buzz through the registration hook: the
    /// polar bed is the one stepper upstream reports in radians
    /// (`stepper.py:302-304`), so its buzz would use the degree distance.
    #[test]
    fn test_a_bare_geared_stepper_registers_as_radians() {
        // `[stepper_bed]`'s shape: no `rotation_distance`, a `gear_ratio`.
        let printer = load_ok(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [printer]\nkinematics: none\nmax_velocity: 300\nmax_accel: 3000\n\
             [stepper_bed]\nstep_pin: PA0\ndir_pin: PA1\nmicrosteps: 16\n\
             gear_ratio: 48:16\n",
        );
        let force_move = printer
            .lookup_object_as::<ForceMove>(FORCE_MOVE_OBJECT)
            .expect("the stepper registered `force_move`");
        assert_eq!(force_move.lookup_stepper("stepper_bed"), Ok(true));
    }

    // ------------------------------------------------------------------
    // End to end, against the fake firmware
    // ------------------------------------------------------------------

    /// The corpus-shaped cartesian printer plus `[force_move]`, over the fake
    /// firmware (`[mcu] test: dict=…`), as the corpus cases bring a machine up.
    fn machine_config(dict: &std::path::Path) -> Config {
        let text = format!(
            "[mcu]\ntest: dict={}\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n\
             max_z_velocity: 5\nmax_z_accel: 100\n\
             [stepper_x]\nstep_pin: PF0\ndir_pin: PF1\nenable_pin: !PD7\nmicrosteps: 16\n\
             rotation_distance: 40\nendstop_pin: ^PE5\nposition_endstop: 0\nposition_max: 200\nhoming_speed: 50\n\
             [stepper_y]\nstep_pin: PF6\ndir_pin: !PF7\nenable_pin: !PF2\nmicrosteps: 16\n\
             rotation_distance: 40\nendstop_pin: ^PJ1\nposition_endstop: 0\nposition_max: 200\nhoming_speed: 50\n\
             [stepper_z]\nstep_pin: PL3\ndir_pin: PL1\nenable_pin: !PK0\nmicrosteps: 16\n\
             rotation_distance: 8\nendstop_pin: ^PD3\nposition_endstop: 0\nposition_max: 200\nhoming_speed: 20\n\
             [force_move]\nenable_force_move: True\n",
            dict.display()
        );
        Config::from_text(&text).expect("the config parses").0
    }

    async fn up_machine(dict: &std::path::Path) -> (Arc<Printer>, Result<(), String>) {
        let config = machine_config(dict);
        let reactor = Arc::new(crate::core::klippy::reactor::TokioReactor::new(
            tokio::runtime::Handle::current(),
        ));
        let printer = Arc::new(Printer::new(reactor));
        let mut start_args = crate::core::klippy::api::StartArgs::collect("force_move.cfg", None);
        start_args.debug_output = Some("_test_output".to_string());
        printer.set_start_args(Arc::new(start_args));
        let setup = async {
            printer.load_config(&config).map_err(|e| e.to_string())?;
            if tokio::time::timeout(std::time::Duration::from_secs(10), printer.bring_up())
                .await
                .is_err()
            {
                return Err("bring_up timed out".to_string());
            }
            let state = printer.get_state_message();
            if state.category != crate::core::klippy::printer::PrinterState::Ready {
                return Err(format!("not ready: {}", state.message));
            }
            Ok(())
        }
        .await;
        (printer, setup)
    }

    /// `STEPPER_BUZZ` and `FORCE_MOVE` end to end: the buzz runs ten
    /// forward/reverse pairs with upstream's dwells, neither command moves the
    /// planner, and a normal `G1` still works on the original kinematics
    /// afterwards.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_a_buzz_and_force_moves_drive_the_motor_without_moving_the_planner() {
        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        if !dict.is_file() {
            return;
        }
        let (printer, setup) = up_machine(&dict).await;
        let outcome = async {
            setup?;
            let gcode = dispatcher(&printer);
            let toolhead = printer
                .lookup_object_as::<ToolHeadObject>("toolhead")
                .expect("the toolhead is registered");
            // Home so a normal move is allowed.
            gcode
                .run_script("SET_KINEMATIC_POSITION X=0 Y=0 Z=0")
                .await
                .map_err(|e| e.to_string())?;

            let before = toolhead.print_time();
            gcode
                .run_script("STEPPER_BUZZ STEPPER=stepper_x")
                .await
                .map_err(|e| format!("buzz: {e}"))?;
            let elapsed = toolhead.print_time() - before;
            // Ten rounds of 0.25 + 0.05 + 0.25 + 0.45 = 10 s, plus the first
            // move's 0.25 s prime.
            if !(10.0..=10.6).contains(&elapsed) {
                return Err(format!("the buzz planned {elapsed}s, expected ~10.25 s"));
            }
            let pos = toolhead.position().expect("connected");
            if pos.x() != 0.0 || pos.y() != 0.0 || pos.z() != 0.0 {
                return Err(format!("the buzz moved the toolhead to {pos:?}"));
            }

            // A force move also leaves the planner's position alone...
            gcode
                .run_script("FORCE_MOVE STEPPER=stepper_x DISTANCE=10 VELOCITY=100")
                .await
                .map_err(|e| format!("force move x: {e}"))?;
            let pos = toolhead.position().expect("connected");
            if pos.x() != 0.0 {
                return Err(format!("FORCE_MOVE moved commanded_pos to {pos:?}"));
            }

            // ...and a normal move still works on the original kinematics.
            gcode
                .run_script("G1 X10 F600")
                .await
                .map_err(|e| format!("g1: {e}"))?;
            let pos = toolhead.position().expect("connected");
            if (pos.x() - 10.0).abs() > 1e-9 {
                return Err(format!("G1 after the buzz landed at {pos:?}"));
            }

            // Two force moves in a row do not deadlock or lose the state.
            gcode
                .run_script("FORCE_MOVE STEPPER=stepper_y DISTANCE=5 VELOCITY=50")
                .await
                .map_err(|e| format!("force move y: {e}"))?;
            gcode
                .run_script("FORCE_MOVE STEPPER=stepper_y DISTANCE=-5 VELOCITY=50")
                .await
                .map_err(|e| format!("force move -y: {e}"))?;
            Ok::<(), String>(())
        }
        .await;
        printer.teardown();
        outcome.unwrap();
    }
}
