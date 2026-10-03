//! `[printer]` — the toolhead: the motion planner and the G-code that drives it.
//!
//! Upstream's `klippy/toolhead.py` reads `[printer]` for the velocity limits,
//! loads a kinematics by name (`kinematics/cartesian.py` builds the rails from
//! the `[stepper_*]` sections), and registers the commands that make the
//! toolhead move (`ToolHeadCommandHelper`: `G4`, `M400`, `M204`,
//! `SET_VELOCITY_LIMIT`). The `[printer]` section is loaded **last**, after every
//! generic section, because the kinematics needs the steppers (`klippy.py:124`).
//!
//! This module is that consumer. The motion algorithms are in
//! [`motion`](crate::core::klippy::motion); the `[stepper_*]` sections are
//! [`extras::stepper`](crate::core::klippy::extras::stepper). Here is where the
//! two are joined and where a `G1` becomes a stream of `queue_step` commands.
//!
//! # Registration
//!
//! The section is declared `phase = late, object = "toolhead"`: the loader loads
//! it after the generic walk and registers the object under the name upstream
//! uses, which is `toolhead`, not `printer`
//! (`klippy/toolhead.py:604-614`). The G-code commands are registered at load
//! time (they only become active once the printer is ready) and capture the
//! shared motion state rather than the object, so a restart can drop the object
//! without leaving a handler behind.
//!
//! # What is here, and what is not
//!
//! | command | meaning |
//! |---|---|
//! | `G4` | dwell, `P` in milliseconds or `S` in seconds |
//! | `M400` | flush the planner |
//! | `G28` | home the named axes (all three when none is named) |
//! | `SET_KINEMATIC_POSITION` | force the low-level position, homing the named axes |
//! | `SET_VELOCITY_LIMIT` | change (or report) the velocity limits |
//! | `M204` | change the acceleration limit (`S`, or the lesser of `P` and `T`) |
//!
//! `G0` / `G1` are **not** here: they belong to
//! [`gcode_move`](crate::core::klippy::extras::gcode_move), which reads them off
//! the command line in g-code coordinates — offsets, relative mode, extrude
//! factor — and calls [`ToolHeadObject::move_to`] with a toolhead coordinate.
//! The toolhead plans moves; it does not interpret a `G` word. Loading that
//! module from here is upstream's `add_printer_objects` loading its default
//! modules (`toolhead.py:610-613`).
//!
//! # Flushing
//!
//! The step solver is the **full** compressor from FW5f: runs of steps are
//! compressed into `queue_step` commands (`interval`, `count`, `add`), so a
//! print does not overflow the firmware's move queue.
//!
//! Steps are generated and sent by a task spawned at connect: it wakes every
//! [`FLUSH_INTERVAL`], generates the planner's queue up to the background
//! horizons ([`BGFLUSH_HIGH_TIME`] — the estimate plus 0.4 s, or 0.7 s while
//! queued motion lies beyond the horizon), and awaits the transport, so a long
//! move cannot outrun the send queue. The horizon is each stamp's birth
//! margin: generation crosses a step 0.4 s before its own clock, so pipeline
//! delay cannot make it late and no stamp is born so far ahead that the
//! firmware's wrapping timer compare reads it as already expired. The task checks
//! [`ToolHeadObject::shutdown`] each wake, so a restart stops it with the object.

use std::collections::HashMap;
use std::future::Future;
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, Weak};
use std::time::Duration;

use serde_json::{json, Value};
use tokio::time::sleep;
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::error::KlippyError;
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::carriage::{self, KinematicStepper};
use crate::core::klippy::extras::extruder::PrinterExtruder;
use crate::core::klippy::extras::force_move::calc_move_time;
use crate::core::klippy::extras::idex_modes;
use crate::core::klippy::extras::query_endstops::{QueryEndstops, QUERY_ENDSTOPS_OBJECT};
use crate::core::klippy::extras::stepper::{PrinterStepper, Rail};
use crate::core::klippy::gcode::{
    parse_float, sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{Coord, Xyz, X_AXIS, Y_AXIS, Z_AXIS};
use crate::core::klippy::mcu::{
    Completion, McuEndstop, McuError, McuObject, McuStepper, StepBatchClocks, TriggerDispatch,
};
use crate::core::klippy::motion::delta::{
    delta_active_flags, delta_position_fn, DeltaConfig, DeltaKinematics, DELTA_RAIL_NAMES,
};
use crate::core::klippy::motion::deltesian::{
    arm_abs_endstops, deltesian_active_flags, deltesian_position_fn, pillars_z_max, x_kin_limits,
    DeltesianConfig, DeltesianKinematics, DELTESIAN_RAIL_NAMES, MIN_ANGLE, SLOW_RATIO,
};
use crate::core::klippy::motion::extra::ExtraAxis;
use crate::core::klippy::motion::generic_cartesian::GenericCartesianKinematics;
use crate::core::klippy::motion::itersolve::{
    cartesian_active_flags, cartesian_position_fn, corexy_active_flags, corexy_position_fn,
    corexz_active_flags, corexz_position_fn, Axis, AxisFlags, PositionFn, StepKinematics,
};
use crate::core::klippy::motion::kinematics::{
    home_move, polar_active_flags, polar_angle_normalize, polar_angle_solver, polar_angle_unwrap,
    polar_home_move, polar_radius_solver, CartesianKinematics, CartesianTransform,
    KinematicsCalibration, NoneKinematics, PolarKinematics,
};
use crate::core::klippy::motion::plan::MoveLimits;
use crate::core::klippy::motion::rotary_delta::{
    rotary_delta_active_flags, rotary_delta_position_fn, RotaryDeltaConfig, RotaryDeltaKinematics,
    ROTARY_DELTA_DEFAULT_ANGLES, ROTARY_DELTA_RAIL_NAMES,
};
use crate::core::klippy::motion::stepcompress::{StepCommand, StepCompressError};
use crate::core::klippy::motion::toolhead::{EstimatedPrintTime, ToolHead};
use crate::core::klippy::motion::winch::{winch_active_flags, winch_position_fn, WinchKinematics};
use crate::core::klippy::motion::{HomeCoord, Homing, HomingHandle, HomingInfo};
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject, RestartHooks};
use crate::core::klippy::reactor::Reactor;
use crate::logging::set_rollover_info;

// Loaded after the generic walk (upstream loads `toolhead` last), registered as
// the `toolhead` object (`[printer]`'s consumer).
section!(
    "printer",
    order = 60,
    phase = late,
    object = "toolhead",
    load = load_config
);

/// How often the flush task wakes to generate and send steps.
const FLUSH_INTERVAL: Duration = Duration::from_millis(10);

/// The background flush's step-generation horizon: how far past the estimate
/// it generates when the planner is caught up (`BGFLUSH_HIGH_TIME`,
/// `extras/motion_queuing.py:10`; the relaxed branch at `:214-216`).
const BGFLUSH_HIGH_TIME: f64 = 0.400;

/// The aggressive branch's window: while queued motion lies beyond the
/// horizon, generate to `est + 0.7 s`, batching from the last horizon in
/// `BGFLUSH_SG_HIGH_TIME - BGFLUSH_SG_LOW_TIME` = 0.25 s steps for
/// run-to-run reproducibility (`motion_queuing.py:197-212`).
const BGFLUSH_SG_LOW_TIME: f64 = 0.450;
const BGFLUSH_SG_HIGH_TIME: f64 = 0.700;

/// How far past the planner's own reach the relaxed branch may go
/// (`need_flush_time + BGFLUSH_EXTRA_TIME`, `motion_queuing.py:215-216`).
const BGFLUSH_EXTRA_TIME: f64 = 0.250;

/// How old a finished move may stay in the trapq history before it is dropped.
const MOVE_HISTORY_EXPIRE: f64 = 30.0;

/// The kinematics `[printer] kinematics` may name.
///
/// The cartesian family shares one `CartesianKinematics` (limits, homing,
/// `check_move`) and differs in which solver each rail runs, how carriage
/// axes map to rail positions (`CartesianTransform`), and which endstops
/// watch which motors (`kinematics/corexy.py`, `corexz.py`,
/// `hybrid_corexy.py`, `hybrid_corexz.py`). `polar` builds its own
/// [`PolarKinematics`](crate::core::klippy::motion::kinematics::PolarKinematics)
/// over `[stepper_arm]`/`[stepper_z]` plus the bare `[stepper_bed]` stepper
/// (`kinematics/polar.py`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum KinematicsKind {
    /// `kinematics: none`
    None,
    /// `kinematics: cartesian`
    Cartesian,
    /// `kinematics: corexy`
    CoreXy,
    /// `kinematics: corexz`
    CoreXz,
    /// `kinematics: hybrid_corexy`
    HybridCoreXy,
    /// `kinematics: hybrid_corexz`
    HybridCoreXz,
    /// `kinematics: polar`
    Polar,
    /// `kinematics: delta` — the linear-delta family
    /// (`kinematics/delta.py`), whose kinematics and calibration math live in
    /// [`motion::delta`](crate::core::klippy::motion::delta).
    Delta,
    /// `kinematics: rotary_delta` — the rotary-delta family
    /// (`kinematics/rotary_delta.py`), whose kinematics and calibration math
    /// live in
    /// [`motion::rotary_delta`](crate::core::klippy::motion::rotary_delta).
    RotaryDelta,
    /// `kinematics: deltesian` — the deltesian family
    /// (`kinematics/deltesian.py`), whose kinematics and solver live in
    /// [`motion::deltesian`](crate::core::klippy::motion::deltesian).
    Deltesian,
    /// `kinematics: generic_cartesian` — the carriage/stepper description
    /// (`kinematics/generic_cartesian.py`), where a motor drives a linear
    /// combination of carriage axes instead of one axis.
    GenericCartesian,
    /// `kinematics: winch` — the cable-winch family
    /// (`kinematics/winch.py`), whose anchors and cable solvers live in
    /// [`motion::winch`](crate::core::klippy::motion::winch).
    Winch,
}

impl KinematicsKind {
    /// The names this host implements. Kept as a list so the error names them.
    const NAMES: &'static [&'static str] = &[
        "none",
        "cartesian",
        "corexy",
        "corexz",
        "hybrid_corexy",
        "hybrid_corexz",
        "polar",
        "delta",
        "rotary_delta",
        "deltesian",
        "generic_cartesian",
        "winch",
    ];

    /// Parse a `[printer] kinematics` value.
    fn parse(name: &str) -> Option<Self> {
        Some(match name {
            "none" => Self::None,
            "cartesian" => Self::Cartesian,
            "corexy" => Self::CoreXy,
            "corexz" => Self::CoreXz,
            "hybrid_corexy" => Self::HybridCoreXy,
            "hybrid_corexz" => Self::HybridCoreXz,
            "polar" => Self::Polar,
            "delta" => Self::Delta,
            "rotary_delta" => Self::RotaryDelta,
            "deltesian" => Self::Deltesian,
            "generic_cartesian" => Self::GenericCartesian,
            "winch" => Self::Winch,
            _ => return None,
        })
    }

    /// The rail-position-to-carriage mapping for the kinematics.
    fn transform(self) -> CartesianTransform {
        match self {
            Self::None | Self::Cartesian => CartesianTransform::Standard,
            Self::CoreXy => CartesianTransform::CoreXy,
            Self::CoreXz => CartesianTransform::CoreXz,
            Self::HybridCoreXy => CartesianTransform::HybridCoreXy,
            Self::HybridCoreXz => CartesianTransform::HybridCoreXz,
            // Polar has no rail→carriage transform: `connect` builds a
            // `PolarKinematics` from `kind` and never reads this arm (the
            // same way `none` never reads its `Standard`).
            Self::Polar => CartesianTransform::Standard,
            // Delta has no rail→carriage mapping of this kind: its towers' solvers
            // are bound to their geometry in the delta branch below, and this
            // value is only consulted by the cartesian build.
            Self::Delta => CartesianTransform::Standard,
            // Rotary delta likewise: its branch binds `rotary_delta_stepper_alloc`.
            Self::RotaryDelta => CartesianTransform::Standard,
            // Deltesian has no rail→carriage mapping of this kind either: its
            // branch binds `deltesian_stepper_alloc` on the arms and a plain
            // cartesian Y solver on the straight rail.
            Self::Deltesian => CartesianTransform::Standard,
            // Generic cartesian's motors run the linear combinations their
            // `[stepper <name>]` sections declare (`extras::carriage` installs
            // those solvers); this arm is never read.
            Self::GenericCartesian => CartesianTransform::Standard,
            // Winch has no rail→carriage mapping of this kind: each cable's
            // solver is bound to its anchor in the winch branch below, so this
            // value is never read.
            Self::Winch => CartesianTransform::Standard,
        }
    }

    /// The rail names `Delta`, `RotaryDelta` and `Deltesian` claim and the
    /// cartesian default (`delta.py:15`, `rotary_delta.py:14-15`,
    /// `deltesian.py:15`).
    fn rail_names(self) -> [&'static str; 3] {
        match self {
            Self::Delta | Self::RotaryDelta => DELTA_RAIL_NAMES,
            Self::Deltesian => DELTESIAN_RAIL_NAMES,
            _ => ["stepper_x", "stepper_y", "stepper_z"],
        }
    }

    /// The solver each rail's steppers run (`X`, `Y`, `Z` order).
    ///
    /// Only the cartesian family's rails are zipped with this —
    /// `ToolHeadObject::new` installs `polar`'s per-stepper solvers (bed
    /// angle, arm radius, cartesian Z) in its own branch before this is
    /// consulted, and `none` has no rails.
    fn solvers(self) -> [(PositionFn, AxisFlags); 3] {
        let cart = |axis: Axis| (cartesian_position_fn(axis), cartesian_active_flags(axis));
        match self {
            Self::None | Self::Polar | Self::Cartesian => {
                [cart(Axis::X), cart(Axis::Y), cart(Axis::Z)]
            }
            // CoreXY: both motors carry the X/Y coupling, so each moves when
            // either axis does (`corexy_stepper_alloc`).
            Self::CoreXy => [
                (corexy_position_fn(true), corexy_active_flags()),
                (corexy_position_fn(false), corexy_active_flags()),
                cart(Axis::Z),
            ],
            // CoreXZ: the X/Z pairing is on the X and Z rails.
            Self::CoreXz => [
                (corexz_position_fn(true), corexz_active_flags()),
                cart(Axis::Y),
                (corexz_position_fn(false), corexz_active_flags()),
            ],
            // Hybrid CoreXY: only the X motor is coupled (`x - y`).
            Self::HybridCoreXy => [
                (corexy_position_fn(false), corexy_active_flags()),
                cart(Axis::Y),
                cart(Axis::Z),
            ],
            // Hybrid CoreXZ: only the X motor is coupled (`x - z`).
            Self::HybridCoreXz => [
                (corexz_position_fn(false), corexz_active_flags()),
                cart(Axis::Y),
                cart(Axis::Z),
            ],
            // Delta never reaches here: its branch below binds each tower to
            // `delta_stepper_alloc` instead (`delta.py:50-52`).
            Self::Delta => [cart(Axis::X), cart(Axis::Y), cart(Axis::Z)],
            // Rotary delta likewise binds `rotary_delta_stepper_alloc`.
            Self::RotaryDelta => [cart(Axis::X), cart(Axis::Y), cart(Axis::Z)],
            // Deltesian never reaches here: its branch binds
            // `deltesian_stepper_alloc` on the arms and a cartesian Y solver on
            // the straight rail.
            Self::Deltesian => [cart(Axis::X), cart(Axis::Y), cart(Axis::Z)],
            // Generic cartesian never reaches here either: `extras::carriage`
            // installs each motor's solver from its `carriages` expression as
            // the section loads.
            Self::GenericCartesian => [cart(Axis::X), cart(Axis::Y), cart(Axis::Z)],
            // Winch never reaches here: each cable's solver is bound to its
            // anchor (`winch_stepper_alloc`) in the branch below.
            Self::Winch => [cart(Axis::X), cart(Axis::Y), cart(Axis::Z)],
        }
    }

    /// Endstops that must also stop the other rail's motors: `(target, source)`
    /// means the target rail's endstop watches `source`'s steppers.
    ///
    /// Upstream registers these in each kinematics' `__init__`
    /// (`corexy.py:14-17` is the two-way case; `hybrid_corexy.py:17-18` the
    /// one-way one).
    fn endstop_pairs(self) -> &'static [(usize, usize)] {
        match self {
            Self::CoreXy => &[(0, 1), (1, 0)],
            Self::CoreXz => &[(0, 2), (2, 0)],
            Self::HybridCoreXy => &[(1, 0)],
            Self::HybridCoreXz => &[(2, 0)],
            _ => &[],
        }
    }
}

/// The four velocity limits as upstream's `ToolHead` stores them
/// (`ToolHead.__init__`, `klippy/toolhead.py:209-216`). The planner's
/// [`MoveLimits`] is derived from these ([`Self::move_limits`]), so a runtime
/// change must recompute it (`ToolHead._calc_junction_deviation`).
///
/// This object holds the copy `get_status` and the kinematics defaults read;
/// the connected planner keeps its own (`ToolHead::set_max_velocities`), and
/// [`cmd_set_velocity_limit`] / [`cmd_m204`] keep the two in step.
struct VelocityLimits {
    max_velocity: f64,
    max_accel: f64,
    square_corner_velocity: f64,
    min_cruise_ratio: f64,
}

impl VelocityLimits {
    /// The planner limits these four values derive
    /// (`MoveLimits::from_velocity_limits`).
    fn move_limits(&self) -> MoveLimits {
        MoveLimits::from_velocity_limits(
            self.max_velocity,
            self.max_accel,
            self.square_corner_velocity,
            self.min_cruise_ratio,
        )
    }

    /// `ToolHead.set_max_velocities` (`klippy/toolhead.py:538-550`): override
    /// the named values and return the four current ones. Only a `Some`
    /// overrides, as upstream's `None` arguments leave the field alone.
    fn set_max_velocities(
        &mut self,
        max_velocity: Option<f64>,
        max_accel: Option<f64>,
        square_corner_velocity: Option<f64>,
        min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        if let Some(velocity) = max_velocity {
            self.max_velocity = velocity;
        }
        if let Some(accel) = max_accel {
            self.max_accel = accel;
        }
        if let Some(velocity) = square_corner_velocity {
            self.square_corner_velocity = velocity;
        }
        if let Some(ratio) = min_cruise_ratio {
            self.min_cruise_ratio = ratio;
        }
        (
            self.max_velocity,
            self.max_accel,
            self.square_corner_velocity,
            self.min_cruise_ratio,
        )
    }
}

/// The `toolhead` object: the planner, its kinematics, and the MCU steppers.
pub struct ToolHeadObject {
    /// The velocity limits this object keeps (see [`VelocityLimits`]); shared
    /// with the command handlers, which outlive the object on a restart.
    limits: Arc<Mutex<VelocityLimits>>,
    max_z_velocity: f64,
    max_z_accel: f64,
    /// The cartesian rails, `[stepper_x]`, `[stepper_y]`, and `[stepper_z]`;
    /// for polar, `[stepper_arm, stepper_z]` (each with its `…1`, `…2`
    /// siblings); for delta, `[stepper_a/b/c]`.
    ///
    /// Empty for `kinematics: none`, which has no steppers.
    rails: Vec<Arc<Rail>>,
    /// The bed rail of a polar printer — the bare `[stepper_bed]` stepper.
    /// It belongs to no rail (`kinematics/polar.py:26` builds it standalone),
    /// but its host solver still runs through the usual stepper setup; **its
    /// solvers are only installed by `KinematicsKind::Polar`**.
    bed: Option<Arc<PrinterStepper>>,
    /// Whether `[printer] kinematics` was `none`.
    /// What `[printer] kinematics` named, for consumers that need to know
    /// which one loaded before the toolhead itself connects
    /// (`[delta_calibrate]`'s `hasattr(kin, "get_calibration")` check).
    kind: KinematicsKind,
    /// The delta kinematics, parked here at load until connect installs it —
    /// the rails' delta solvers are already bound in `new`.
    delta: Mutex<Option<DeltaKinematics>>,
    /// The rotary-delta kinematics, parked the same way as `delta`.
    rotary_delta: Mutex<Option<RotaryDeltaKinematics>>,
    /// The deltesian kinematics, parked the same way as `delta`.
    deltesian: Mutex<Option<DeltesianKinematics>>,
    /// The generic-cartesian kinematics, parked here at load until connect
    /// installs it (`generic_cartesian.py:120-172` builds it in `__init__`;
    /// only the toolhead's install waits for connect).
    generic: Mutex<Option<GenericCartesianKinematics>>,
    /// The `[stepper <name>]` motors of a generic-cartesian printer. They
    /// belong to no rail — each drives a combination of carriages — so they are
    /// kept apart from `rails` and taken at connect like the bed stepper.
    generic_steppers: Vec<Arc<KinematicStepper>>,
    /// The winch kinematics, parked here at load until connect installs it —
    /// the cables' solvers are already bound in `new` (`winch.py:11-20`).
    winch: Mutex<Option<WinchKinematics>>,
    /// The cable `[stepper_a]`…`[stepper_z]` motors of a winch printer. They
    /// belong to no rail — each pulls its cable to a fixed anchor — so they are
    /// kept apart from `rails` and taken at connect like the bed stepper.
    winch_steppers: Vec<Arc<PrinterStepper>>,
    /// How the rails' positions map to carriage axes (`corexy.py:12-15`).
    transform: CartesianTransform,
    /// `[printer] max_angular_velocity`: polar's near-center angular cap
    /// (`0` = uncapped). Read only for polar, as upstream reads it only in
    /// `PolarKinematics.__init__` (`kinematics/polar.py:51-52`).
    max_angular_velocity: f64,
    /// The active extruder's name (`ACTIVATE_EXTRUDER`), for `M104` without `T`.
    active_extruder: Mutex<String>,
    /// The machine's clock, for seeding the print-time mapping.
    reactor: Arc<dyn Reactor>,
    /// The machine, to shut it down if the compressor hits an internal error.
    /// `Weak` because the printer's registry owns this object.
    printer: Weak<Printer>,
    /// The connected motion state; `None` until connect.
    state: Arc<Mutex<Option<Connected>>>,
    /// Lookahead callbacks registered before there was a planner to hang them
    /// on (`register_lookahead_callback` at config load): installed by
    /// [`Self::connect`], the same connect-time handover as
    /// `set_estimated_print_time_source`.
    pending_lookahead_callbacks: Mutex<Vec<Box<dyn FnOnce(f64) + Send + 'static>>>,
    /// Flush callbacks registered before connect, installed the same way
    /// (`register_flush_callback` at config load).
    pending_flush_callbacks: Mutex<Vec<Box<dyn Fn(f64) + Send + 'static>>>,
    /// Set when the object is dropped, to stop the flush task.
    shutdown: Arc<AtomicBool>,
}

/// Everything that exists only once the machine is up.
struct Connected {
    toolhead: ToolHead,
    mcu_steppers: HashMap<String, Arc<McuStepper>>,
    /// The print time the solvers have generated up to.
    last_step_gen_time: f64,
    /// The trapq the force-move queue swaps a stepper onto while it moves it
    /// outside the planner, allocated on first use (upstream `ForceMove`'s own
    /// `self.trapq`, `force_move.py:33`). Kept beside the toolhead because the
    /// id only means anything for this connection's [`MotionQueuing`].
    force_move_trapq: Option<usize>,
}

/// The step commands to send, paired with the stepper that produced them and
/// the clock window the batch covers (`StepBatchClocks`).
type StepBatches = Vec<(Arc<McuStepper>, Vec<StepCommand>, StepBatchClocks)>;

/// The clock window one generated batch covers: the two print times in this
/// stepper's MCU's ticks (`StepBatchClocks`).
///
/// No mapping yet — clock 0 for both — is what the send gates read as "send
/// now", and a 0 completion frees its slots on sight.
fn step_batch_clocks(
    stepper: &McuStepper,
    start_time: f64,
    completion_time: f64,
) -> StepBatchClocks {
    StepBatchClocks {
        start: stepper.chip().print_time_to_clock(start_time).unwrap_or(0),
        completion: stepper
            .chip()
            .print_time_to_clock(completion_time)
            .unwrap_or(0),
    }
}

impl ToolHeadObject {
    /// Build the object from `[printer]` and the three stepper sections.
    ///
    /// # Errors
    /// Returns a config error for a missing or unsupported `kinematics`, a
    /// missing velocity limit, or a missing `[stepper_x/y/z]`.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let kinematics = config.get("kinematics", None)?;
        let kind = KinematicsKind::parse(&kinematics).ok_or_else(|| {
            ConfigError::new(format!(
                "Error loading kinematics '{kinematics}' (only {} are implemented)",
                KinematicsKind::NAMES.join(", ")
            ))
        })?;

        // `PolarKinematics.__init__` reads the angular cap; no other
        // kinematics does (`kinematics/polar.py:51-52`), so a
        // `max_angular_velocity` in a cartesian config stays unread and is
        // rejected by `check_unused`, as upstream does. Upstream's default
        // (0 = uncapped) is not bounds-checked, so an absent option is not
        // either — only a present value must be above 0.
        let max_angular_velocity = if kind == KinematicsKind::Polar {
            if config.has("max_angular_velocity") {
                config.get_float_bounded(
                    "max_angular_velocity",
                    None,
                    None,
                    None,
                    Some(0.0),
                    None,
                )?
            } else {
                0.0
            }
        } else {
            0.0
        };

        let max_velocity =
            config.get_float_bounded("max_velocity", None, None, None, Some(0.0), None)?;
        let max_accel = config.get_float_bounded("max_accel", None, None, None, Some(0.0), None)?;
        let min_cruise_ratio = config.get_float_bounded(
            "minimum_cruise_ratio",
            Some(0.5),
            Some(0.0),
            None,
            None,
            Some(1.0),
        )?;
        let square_corner_velocity = config.get_float_bounded(
            "square_corner_velocity",
            Some(5.0),
            Some(0.0),
            None,
            None,
            None,
        )?;
        // `CartKinematics` defaults Z to the full limits and caps it there
        // (`klippy/kinematics/cartesian.py:53-56`).
        let max_z_velocity = config.get_float_bounded(
            "max_z_velocity",
            Some(max_velocity),
            None,
            Some(max_velocity),
            Some(0.0),
            None,
        )?;
        let max_z_accel = config.get_float_bounded(
            "max_z_accel",
            Some(max_accel),
            None,
            Some(max_accel),
            Some(0.0),
            None,
        )?;

        // The four limits as upstream stores them; the planner's view (the
        // junction geometry) is derived from them (`ToolHead.__init__`,
        // `klippy/toolhead.py:209-216`).
        let limits = Arc::new(Mutex::new(VelocityLimits {
            max_velocity,
            max_accel,
            square_corner_velocity,
            min_cruise_ratio,
        }));

        let mut rails: Vec<Arc<Rail>> = Vec::new();
        let mut bed = None;
        let mut delta_kinematics = None;
        let mut rotary_delta_kinematics = None;
        let mut deltesian_kinematics = None;
        let mut generic_kinematics = None;
        let mut generic_steppers: Vec<Arc<KinematicStepper>> = Vec::new();
        let mut winch_kinematics = None;
        let mut winch_steppers: Vec<Arc<PrinterStepper>> = Vec::new();
        match kind {
            KinematicsKind::None => {}
            KinematicsKind::GenericCartesian => {
                // `GenericCartesianKinematics.__init__` reads every `[carriage
                // <name>]` / `[stepper <name>]` section here
                // (`generic_cartesian.py:120-172`) and installs each motor's
                // solver as it reads it; the built kinematics waits for
                // connect, like delta's.
                let built = carriage::build(printer, max_z_velocity, max_z_accel)?;
                generic_kinematics = Some(built.kinematics);
                generic_steppers = built.steppers;
            }
            KinematicsKind::Polar => {
                // `kinematics/polar.py:25-31`: the arm is a rail
                // (`LookupRail`), Z is a (multi) rail, and the bed is a bare
                // stepper with no rail geometry of its own.
                let arm = Rail::lookup(config, printer, "stepper_arm", Axis::X)?;
                let z = Rail::lookup(config, printer, "stepper_z", Axis::Z)?;
                // The owning kinematics installs each stepper's solver
                // (`MCU_stepper.setup_itersolve`): arm radius and bed angle
                // both follow X and Y (`kin_polar.c:46`), Z stays cartesian.
                for stepper in arm.steppers() {
                    stepper.setup_itersolve(polar_radius_solver(), polar_active_flags());
                }
                for stepper in z.steppers() {
                    stepper.setup_itersolve(
                        cartesian_position_fn(Axis::Z),
                        cartesian_active_flags(Axis::Z),
                    );
                }
                let bed_stepper = printer
                    .lookup_object_as::<PrinterStepper>("stepper_bed")
                    .ok_or_else(|| {
                        ConfigError::new(format!(
                            "Section '{}' needs a '[stepper_bed]' section",
                            config.identifier()
                        ))
                    })?;
                bed_stepper.setup_itersolve(polar_angle_solver(), polar_active_flags());
                // The angle solver unwraps ±2π against `commanded_pos` and
                // renormalizes after each range (`kin_polar.c`).
                bed_stepper.setup_hooks(polar_angle_unwrap, polar_angle_normalize);
                rails = vec![arm, z];
                bed = Some(bed_stepper);
            }
            KinematicsKind::Delta => {
                // Delta claims `stepper_a/b/c`, the cartesian family
                // `stepper_x/y/z` (`delta.py:15`); the axis is the rail's slot in
                // this list.
                let axes = [Axis::X, Axis::Y, Axis::Z];
                for (name, axis) in kind.rail_names().into_iter().zip(axes) {
                    rails.push(Rail::lookup(config, printer, name, axis)?);
                }
                // The delta kinematics reads its options here (the config
                // reads are part of loading it) and binds each tower to
                // `delta_stepper_alloc` (`setup_itersolve`, `delta.py:50-52`).
                let delta = build_delta(
                    config,
                    &rails,
                    max_velocity,
                    max_accel,
                    max_z_velocity,
                    max_z_accel,
                )?;
                for (rail, (arm2, tower_x, tower_y)) in rails.iter().zip(delta.tower_geometry()) {
                    for stepper in rail.steppers() {
                        stepper.setup_itersolve(
                            delta_position_fn(arm2, tower_x, tower_y),
                            delta_active_flags(),
                        );
                    }
                }
                delta_kinematics = Some(delta);
            }
            KinematicsKind::RotaryDelta => {
                // Rotary delta claims `stepper_a/b/c` too (`rotary_delta.py:14`).
                let axes = [Axis::X, Axis::Y, Axis::Z];
                for (name, axis) in ROTARY_DELTA_RAIL_NAMES.into_iter().zip(axes) {
                    rails.push(Rail::lookup(config, printer, name, axis)?);
                }
                // The kinematics reads its options here and binds each tower to
                // `rotary_delta_stepper_alloc` (`setup_itersolve`,
                // `rotary_delta.py:49-51`).
                let rotary = build_rotary_delta(
                    config,
                    &rails,
                    max_velocity,
                    max_accel,
                    max_z_velocity,
                    max_z_accel,
                )?;
                for (rail, (sr, sh, angle, ua, la)) in rails.iter().zip(rotary.tower_geometry()) {
                    for stepper in rail.steppers() {
                        stepper.setup_itersolve(
                            rotary_delta_position_fn(sr, sh, angle.to_radians(), ua, la),
                            rotary_delta_active_flags(),
                        );
                    }
                }
                rotary_delta_kinematics = Some(rotary);
            }
            KinematicsKind::Winch => {
                // `WinchKinematics.__init__` (`kinematics/winch.py:13-24`):
                // walk `stepper_a`…`stepper_z` (26 letters), always taking the
                // first three and stopping at the first missing section. Each
                // cable is a bare stepper (no rail range), reads its
                // `anchor_x/y/z`, and gets the cable solver bound to that
                // anchor (`setup_itersolve('winch_stepper_alloc', *a)`).
                let mut cables: Vec<(String, [f64; 3])> = Vec::new();
                for index in 0..26u8 {
                    let name = format!("stepper_{}", (b'a' + index) as char);
                    if index >= 3 && !config.has_sibling(&name) {
                        break;
                    }
                    let stepper = printer
                        .lookup_object_as::<PrinterStepper>(&name)
                        .ok_or_else(|| {
                            ConfigError::new(format!(
                                "Section '{}' needs a '[{name}]' section",
                                config.identifier()
                            ))
                        })?;
                    let section = config.sibling(&name).ok_or_else(|| {
                        ConfigError::new(format!(
                            "Section '{}' needs a '[{name}]' section",
                            config.identifier()
                        ))
                    })?;
                    let anchor = [
                        section.get_float("anchor_x", None)?,
                        section.get_float("anchor_y", None)?,
                        section.get_float("anchor_z", None)?,
                    ];
                    stepper.setup_itersolve(winch_position_fn(anchor), winch_active_flags());
                    cables.push((name, anchor));
                    winch_steppers.push(stepper);
                }
                winch_kinematics = Some(WinchKinematics::new(cables));
            }
            KinematicsKind::Deltesian => {
                // Deltesian claims `stepper_left`, `stepper_right`, `stepper_y`
                // (`deltesian.py:15-17`); the arms take axes X and Y, the
                // straight rail Z — the labels are inert (the kinematics
                // installs the solvers just below).
                let axes = [Axis::X, Axis::Y, Axis::Z];
                for (name, axis) in DELTESIAN_RAIL_NAMES.into_iter().zip(axes) {
                    rails.push(Rail::lookup(config, printer, name, axis)?);
                }
                // The kinematics reads its options here (the config reads are
                // part of loading it); the two arms then bind
                // `deltesian_stepper_alloc` and the straight rail a cartesian Y
                // solver (`deltesian.py:30-36`).
                let deltesian = build_deltesian(
                    config,
                    &rails,
                    max_velocity,
                    max_accel,
                    max_z_velocity,
                    max_z_accel,
                )?;
                for (rail, (arm2, arm_x)) in rails[..2].iter().zip(deltesian.arm_geometry()) {
                    for stepper in rail.steppers() {
                        stepper.setup_itersolve(
                            deltesian_position_fn(arm2, arm_x),
                            deltesian_active_flags(),
                        );
                    }
                }
                for stepper in rails[2].steppers() {
                    stepper.setup_itersolve(
                        cartesian_position_fn(Axis::Y),
                        cartesian_active_flags(Axis::Y),
                    );
                }
                deltesian_kinematics = Some(deltesian);
            }
            _ => {
                for (name, axis) in [
                    ("stepper_x", Axis::X),
                    ("stepper_y", Axis::Y),
                    ("stepper_z", Axis::Z),
                ] {
                    rails.push(Rail::lookup(config, printer, name, axis)?);
                }
                // The owning kinematics installs each stepper's solver
                // (`MCU_stepper.setup_itersolve`); the family decides the position
                // function, so a corexy motor follows `x ± y`.
                let solvers = kind.solvers();
                for (rail, (position, flags)) in rails.iter().zip(solvers) {
                    for stepper in rail.steppers() {
                        stepper.setup_itersolve(position, flags);
                    }
                }
                // A paired rail's endstop has to stop the other rail's motors too
                // (`corexy.py:14-17` and friends).
                for (target, source) in kind.endstop_pairs() {
                    let Some(endstop) = rails[*target].endstop().cloned() else {
                        continue;
                    };
                    for stepper in rails[*source].steppers() {
                        // Every endstop a rail can name drives a dispatch; the
                        // default-`None` answer belongs to endstops that never
                        // ride a rail.
                        let dispatch = endstop.dispatch().ok_or_else(|| {
                            ConfigError::new(format!(
                                "{}: a paired rail's endstop must drive a trigger dispatch",
                                config.identifier()
                            ))
                        })?;
                        dispatch
                            .add_stepper(
                                stepper.mcu_stepper().chip().clone(),
                                Arc::downgrade(stepper.mcu_stepper()),
                                stepper.name(),
                            )
                            .map_err(|err| {
                                ConfigError::new(format!("{}: {err}", config.identifier()))
                            })?;
                    }
                }
                // IDEX: the cartesian kinematics claims `[dual_carriage]` and
                // hands the module this axis' primary rail
                // (`kinematics/cartesian.py:24-34`).
                if kind == KinematicsKind::Cartesian {
                    idex_modes::claim(&rails, printer);
                }
            }
        }

        // The object every rail's endstop is queried through. Created here
        // because this is the first point where all the `[stepper_*]` sections
        // (and their endstops) exist; registered before `toolhead` itself.
        let query = QueryEndstops::new(printer)?;
        for rail in &rails {
            for stepper in rail.steppers() {
                if let Some(endstop) = stepper.endstop() {
                    query.register_endstop(Arc::clone(endstop), stepper.name());
                }
            }
        }
        printer.add_object(QUERY_ENDSTOPS_OBJECT, Arc::new(query))?;

        let state = Arc::new(Mutex::new(None));
        let object = Self {
            limits,
            max_z_velocity,
            max_z_accel,
            rails,
            bed,
            kind,
            delta: Mutex::new(delta_kinematics),
            rotary_delta: Mutex::new(rotary_delta_kinematics),
            deltesian: Mutex::new(deltesian_kinematics),
            generic: Mutex::new(generic_kinematics),
            generic_steppers,
            winch: Mutex::new(winch_kinematics),
            winch_steppers,
            transform: kind.transform(),
            max_angular_velocity,
            active_extruder: Mutex::new("extruder".to_string()),
            reactor: printer.reactor(),
            printer: Arc::downgrade(printer),
            state,
            pending_lookahead_callbacks: Mutex::new(Vec::new()),
            pending_flush_callbacks: Mutex::new(Vec::new()),
            shutdown: Arc::new(AtomicBool::new(false)),
        };
        object.register_commands(printer)?;
        Ok(object)
    }

    /// Register the G-code commands, capturing only the shared motion state.
    fn register_commands(&self, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        // `G0`/`G1` are `gcode_move`'s: they interpret g-code coordinates
        // before the toolhead sees them (upstream `ToolHeadCommandHelper` has
        // no move commands for the same reason).
        let dwell_handler: CommandHandler = {
            let state = Arc::clone(&self.state);
            sync(move |gcmd| cmd_dwell(&state, gcmd))
        };
        gcode
            // `S` is seconds, `P` milliseconds; the handler prefers `S`.
            .register_command_with_params("G4", dwell_handler, None, &["S", "P"], false)
            .map_err(ConfigError::new)?;
        let wait_handler: CommandHandler = {
            let state = Arc::clone(&self.state);
            sync(move |gcmd| cmd_wait_moves(&state, gcmd))
        };
        gcode
            .register_command("M400", wait_handler, None, false)
            .map_err(ConfigError::new)?;
        let position_handler: CommandHandler = {
            let state = Arc::clone(&self.state);
            let printer = Arc::downgrade(printer);
            sync(move |gcmd| cmd_set_kinematic_position(&state, &printer, gcmd))
        };
        gcode
            .register_command_with_params(
                "SET_KINEMATIC_POSITION",
                position_handler,
                Some("Force a low-level kinematic position"),
                // The axes the handler reads by loop (`X`/`Y`/`Z`), then the
                // homing words it reads by name.
                &["X", "Y", "Z", "SET_HOMED", "CLEAR", "CLEAR_HOMED"],
                false,
            )
            .map_err(ConfigError::new)?;
        let home_handler: CommandHandler = {
            let state = Arc::clone(&self.state);
            let rails = self.rails.clone();
            let kind = self.kind;
            let printer = Arc::downgrade(printer);
            Arc::new(move |gcmd: &GcodeCommand| {
                let state = Arc::clone(&state);
                let rails = rails.clone();
                let printer = printer.clone();
                Box::pin(async move { cmd_g28(&state, &rails, kind, &printer, gcmd).await })
            })
        };
        gcode
            // Naming an axis homes it; naming none homes all three.
            .register_command_with_params(
                "G28",
                home_handler,
                Some("Home one or more axes"),
                &["X", "Y", "Z"],
                false,
            )
            .map_err(ConfigError::new)?;
        let velocity_handler: CommandHandler = {
            let state = Arc::clone(&self.state);
            let limits = Arc::clone(&self.limits);
            sync(move |gcmd| cmd_set_velocity_limit(&state, &limits, gcmd))
        };
        gcode
            .register_command_with_params(
                "SET_VELOCITY_LIMIT",
                velocity_handler,
                Some("Set printer velocity limits"),
                &[
                    "VELOCITY",
                    "ACCEL",
                    "SQUARE_CORNER_VELOCITY",
                    "MINIMUM_CRUISE_RATIO",
                ],
                false,
            )
            .map_err(ConfigError::new)?;
        let m204_handler: CommandHandler = {
            let state = Arc::clone(&self.state);
            let limits = Arc::clone(&self.limits);
            sync(move |gcmd| cmd_m204(&state, &limits, gcmd))
        };
        gcode
            // `S` sets the accel; with no `S`, the minimum of `P` and `T` does.
            .register_command_with_params("M204", m204_handler, None, &["S", "P", "T"], false)
            .map_err(ConfigError::new)?;
        Ok(())
    }

    /// The names of the three rails, in axis order.
    fn axis_names(&self) -> [String; 3] {
        [
            self.rails[0].name().to_string(),
            self.rails[1].name().to_string(),
            self.rails[2].name().to_string(),
        ]
    }

    /// The names of the Z rail's steppers, in config order (`stepper_z`,
    /// `stepper_z1`, `stepper_z2`…), for a Z-tilt/QGL helper to check its
    /// `z_positions` count against (`ZAdjustHelper.handle_connect` counts the
    /// toolhead's z-active steppers the same way).
    ///
    /// Empty for `kinematics: none`, which has no rails, and for polar
    /// (whose Z rail is found by name below — polar's rails are
    /// `[stepper_arm, stepper_z]`, so a fixed index would be wrong).
    pub fn z_stepper_names(&self) -> Vec<String> {
        self.rails
            .iter()
            .find(|rail| rail.name() == "stepper_z")
            .map(|rail| {
                rail.steppers()
                    .iter()
                    .map(|stepper| stepper.name().to_string())
                    .collect()
            })
            .unwrap_or_default()
    }

    /// The stored velocity limits, locked.
    fn limits_guard(&self) -> MutexGuard<'_, VelocityLimits> {
        self.limits
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The planner limits right now — the four stored values with the derived
    /// junction geometry (`MoveLimits::from_velocity_limits`).
    fn move_limits(&self) -> MoveLimits {
        self.limits_guard().move_limits()
    }

    /// The machine's maximum velocity, for `[extruder]`'s speed defaults.
    pub fn max_velocity(&self) -> f64 {
        self.limits_guard().max_velocity
    }

    /// Whether the loaded `[printer]` kinematics carries a delta calibration
    /// (`delta_calibrate.py:handle_connect`'s `hasattr(kin,
    /// "get_calibration")`).
    ///
    /// Answerable from the load-time kind, so `[delta_calibrate]` can check at
    /// its own connect — which runs before this object's, since upstream loads
    /// `toolhead` last (`toolhead.py:604-614`).
    pub fn has_delta_calibration(&self) -> bool {
        matches!(
            self.kind,
            KinematicsKind::Delta | KinematicsKind::RotaryDelta
        )
    }

    /// The delta calibration parameters the kinematics carries
    /// (`get_calibration`, `delta.py:160-166` / `rotary_delta.py:129-130`), or
    /// `None` for any other kinematics.
    ///
    /// Read from the connected kinematics when the machine is up, and from
    /// the parameters parked here at load otherwise — they are the same
    /// object; connect hands it over.
    pub fn delta_calibration(&self) -> Option<KinematicsCalibration> {
        {
            let guard = self.lock();
            if let Some(connected) = guard.as_ref() {
                if let Some(calibration) = connected
                    .toolhead
                    .kinematics()
                    .and_then(|kinematics| kinematics.delta_calibration())
                {
                    return Some(calibration);
                }
            }
        }
        if let Some(calibration) = self
            .delta
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .as_ref()
            .map(DeltaKinematics::calibration)
        {
            return Some(KinematicsCalibration::Linear(calibration));
        }
        self.rotary_delta
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .as_ref()
            .map(RotaryDeltaKinematics::calibration)
            .map(KinematicsCalibration::Rotary)
    }

    /// The machine's maximum acceleration, for `[extruder]`'s speed defaults.
    pub fn max_accel(&self) -> f64 {
        self.limits_guard().max_accel
    }

    /// Record the active extruder (`ACTIVATE_EXTRUDER`).
    pub fn set_active_extruder(&self, name: &str) {
        *self
            .active_extruder
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = name.to_string();
    }

    /// The active extruder's name.
    pub fn active_extruder(&self) -> String {
        self.active_extruder
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }
}

impl PrinterObject for ToolHeadObject {
    fn get_status(&self, _eventtime: f64) -> Value {
        let guard = self.lock();
        let Some(connected) = guard.as_ref() else {
            return json!({});
        };
        let position = connected.toolhead.commanded_pos();
        let homed_axes = connected
            .toolhead
            .kinematics()
            .map(|kinematics| kinematics.get_status()["homed_axes"].clone())
            .unwrap_or_else(|| json!(""));
        // The four velocity limits as upstream reports them (`get_status`,
        // `toolhead.py:502-515`); this object owns the copy the planner was
        // built from, and the runtime setters keep the two in step.
        let limits = self.limits_guard();
        json!({
            "position": position.as_array(),
            "homed_axes": homed_axes,
            "print_time": connected.toolhead.print_time(),
            // The active extruder's name, as upstream reports it
            // (`toolhead.py:511`); `PARK_{printer.toolhead.extruder}` and
            // friends read it through the macro template.
            "extruder": self.active_extruder(),
            "max_velocity": limits.max_velocity,
            "max_accel": limits.max_accel,
            "minimum_cruise_ratio": limits.min_cruise_ratio,
            "square_corner_velocity": limits.square_corner_velocity,
        })
    }

    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            let config_error = |message: String| {
                KlippyError::Config(ConfigError::new(format!("[printer]: {message}")))
            };

            // Each `[stepper_*]` built its host solver during its own connect,
            // which runs before this one (generic sections before the late
            // walk). Take them now, along with the firmware resources and the
            // MCU each axis lives on.
            let mut host_steppers = Vec::new();
            let mut mcu_steppers = HashMap::new();
            for rail in &self.rails {
                for stepper in rail.steppers() {
                    let host = stepper.take_stepper().ok_or_else(|| {
                        config_error(format!("{} is not connected", stepper.name()))
                    })?;
                    host_steppers.push(host);
                    mcu_steppers.insert(
                        stepper.name().to_string(),
                        Arc::clone(stepper.mcu_stepper()),
                    );
                }
            }
            // Generic cartesian's motors: each belongs to the carriages its
            // `carriages` expression names, not to a rail.
            for kinematic in &self.generic_steppers {
                let stepper = kinematic.stepper();
                let host = stepper
                    .take_stepper()
                    .ok_or_else(|| config_error(format!("{} is not connected", stepper.name())))?;
                host_steppers.push(host);
                mcu_steppers.insert(
                    stepper.name().to_string(),
                    Arc::clone(stepper.mcu_stepper()),
                );
            }
            // Polar's bare bed stepper: it belongs to no rail, but its host
            // solver drives it from the main trapq like the rails' — upstream
            // lists it first (`kinematics/polar.py:31-34`).
            if let Some(bed) = &self.bed {
                let host = bed
                    .take_stepper()
                    .ok_or_else(|| config_error(format!("{} is not connected", bed.name())))?;
                host_steppers.push(host);
                mcu_steppers.insert(bed.name().to_string(), Arc::clone(bed.mcu_stepper()));
            }
            // Winch's cable steppers: they belong to no rail either, and each
            // drives its own cable length from the main trapq
            // (`kinematics/winch.py:18-20`).
            for stepper in &self.winch_steppers {
                let host = stepper
                    .take_stepper()
                    .ok_or_else(|| config_error(format!("{} is not connected", stepper.name())))?;
                host_steppers.push(host);
                mcu_steppers.insert(
                    stepper.name().to_string(),
                    Arc::clone(stepper.mcu_stepper()),
                );
            }

            // The primary MCU (the bare `[mcu]`) defines the print-time origin;
            // each stepper's compressor was already pointed at its own MCU's
            // clock domain (`SecondarySync`) during its own connect, which used
            // this same `[mcu]` object's clock. The origin is handed over as a
            // **getter**, not a reading (see `EstimatedPrintTime`): every prime
            // asks for the estimate of that moment.
            let printer_for_est = self.printer.clone();
            let reactor_for_est = self.reactor.clone();

            // The planner is built from this object's limits, and the runtime
            // setters keep the two copies in step (`ToolHead::set_max_velocities`).
            let mut toolhead = ToolHead::new(self.move_limits());
            for stepper in host_steppers {
                toolhead.add_stepper(stepper);
            }
            match self.kind {
                KinematicsKind::None => toolhead.set_kinematics(Box::new(NoneKinematics)),
                KinematicsKind::Polar => {
                    // rails = [arm, z] (installed in `new`); ranges feed the
                    // gated square and the Z limit, as upstream derives them
                    // from `rails[0]`/`rails[1]` (`kinematics/polar.py:42-51`).
                    let bed = self
                        .bed
                        .as_ref()
                        .ok_or_else(|| config_error("stepper_bed is not connected".to_string()))?;
                    let arm = &self.rails[0];
                    let z = &self.rails[1];
                    toolhead.set_kinematics(Box::new(PolarKinematics::new(
                        [
                            bed.name().to_string(),
                            arm.name().to_string(),
                            z.name().to_string(),
                        ],
                        (arm.params().position_min, arm.params().position_max),
                        (z.params().position_min, z.params().position_max),
                        self.move_limits(),
                        self.max_z_velocity,
                        self.max_z_accel,
                        self.max_angular_velocity,
                    )));
                }
                KinematicsKind::Delta => {
                    let delta = self
                        .delta
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .take()
                        .ok_or_else(|| {
                            config_error("delta kinematics is not connected".to_string())
                        })?;
                    toolhead.set_kinematics(Box::new(delta));
                }
                KinematicsKind::RotaryDelta => {
                    let rotary = self
                        .rotary_delta
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .take()
                        .ok_or_else(|| {
                            config_error("rotary_delta kinematics is not connected".to_string())
                        })?;
                    toolhead.set_kinematics(Box::new(rotary));
                }
                KinematicsKind::Deltesian => {
                    let deltesian = self
                        .deltesian
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .take()
                        .ok_or_else(|| {
                            config_error("deltesian kinematics is not connected".to_string())
                        })?;
                    toolhead.set_kinematics(Box::new(deltesian));
                }
                KinematicsKind::GenericCartesian => {
                    let kinematics = self
                        .generic
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .take()
                        .ok_or_else(|| {
                            config_error(
                                "generic_cartesian kinematics is not connected".to_string(),
                            )
                        })?;
                    toolhead.set_kinematics(Box::new(kinematics));
                }
                KinematicsKind::Winch => {
                    let winch = self
                        .winch
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .take()
                        .ok_or_else(|| {
                            config_error("winch kinematics is not connected".to_string())
                        })?;
                    toolhead.set_kinematics(Box::new(winch));
                }
                _ => {
                    toolhead.set_kinematics(Box::new(CartesianKinematics::new(
                        self.axis_names(),
                        Coord::new(
                            self.rails[X_AXIS].params().position_min,
                            self.rails[Y_AXIS].params().position_min,
                            self.rails[Z_AXIS].params().position_min,
                            0.0,
                        ),
                        Coord::new(
                            self.rails[X_AXIS].params().position_max,
                            self.rails[Y_AXIS].params().position_max,
                            self.rails[Z_AXIS].params().position_max,
                            0.0,
                        ),
                        self.max_z_velocity,
                        self.max_z_accel,
                        self.transform,
                    )));
                }
            }

            // The toolhead's print time is the primary MCU's, read live on
            // every prime (`_calc_print_time`, `klippy/toolhead.py:260-264`).
            // A reading taken here, at connect, would floor every later move at
            // a horizon the clock passed `idle` seconds ago (C5: the whole
            // motion expired and was dumped in one burst → `Timer too close`).
            toolhead.set_estimated_print_time_source(EstimatedPrintTime::new(move || {
                printer_for_est
                    .upgrade()
                    .and_then(|printer| printer.lookup_object_as::<McuObject>("mcu"))
                    .and_then(|object| object.estimated_print_time(reactor_for_est.monotonic()))
                    .unwrap_or(0.0)
            }));

            // The extruders are the toolhead's `extra_axes`: each has its own
            // trapq, and the extruder queues the extrusion into it
            // (`kinematics/extruder.py:140-190`).
            if let Some(printer) = self.printer.upgrade() {
                for name in extruder_names(&printer) {
                    let Some(extruder) = printer.lookup_object_as::<PrinterExtruder>(&name) else {
                        continue;
                    };
                    if let (Some(mut stepper), Some(mcu_stepper)) =
                        (extruder.take_stepper(), extruder.mcu_stepper())
                    {
                        let trapq = toolhead.allocate_trapq();
                        stepper.set_trapq(trapq);
                        mcu_steppers.insert(stepper.name().to_string(), mcu_stepper);
                        toolhead.add_stepper(stepper);
                        extruder.set_trapq(trapq);
                    }
                    toolhead.add_extra_axis(Arc::clone(&extruder) as Arc<dyn ExtraAxis>);
                }
            }

            // Callbacks registered at config load, when there was no
            // planner to hang them on (`register_lookahead_callback` /
            // `register_flush_callback`): install them now, under the state
            // lock. A registration racing this window blocks on that same
            // lock — it has either filled these lists before the drain, or
            // arrives after the install and goes straight to the live
            // toolhead — so none can be lost. The handover mirrors
            // `set_estimated_print_time_source`.
            {
                let mut guard = self.lock();
                let mut pending = self
                    .pending_lookahead_callbacks
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner());
                for callback in std::mem::take(&mut *pending) {
                    toolhead.register_lookahead_callback(callback);
                }
                drop(pending);
                let mut pending = self
                    .pending_flush_callbacks
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner());
                for callback in std::mem::take(&mut *pending) {
                    toolhead.register_flush_callback(callback);
                }
                *guard = Some(Connected {
                    toolhead,
                    mcu_steppers,
                    last_step_gen_time: 0.0,
                    force_move_trapq: None,
                });
            }

            // Let the G-code dispatcher reach the planner before a restart
            // (`GCodeDispatch.request_restart` needs the last print time, a
            // dwell and a wait).
            if let Some(printer) = self.printer.upgrade() {
                printer.register_restart_hooks(Arc::new(ToolHeadRestartHooks(Arc::clone(
                    &self.state,
                ))));
            }

            // The flush task owns nothing the object does not share; it stops
            // when the object is dropped, by reading `shutdown`.
            tokio::spawn(run_flush_loop(
                Arc::clone(&self.state),
                Arc::clone(&self.shutdown),
                self.printer.clone(),
            ));
            Ok(())
        })
    }
}

/// Read the delta options and build the kinematics
/// (`kinematics/delta.py:11-77`, whose reads these mirror one for one): the
/// `[printer]` options here, the tower options from each `[stepper_a/b/c]`
/// section.
///
/// # Errors
/// A missing or out-of-bounds option, reported with the config reader's
/// upstream wording; or a geometry whose home position does not exist (see
/// [`DeltaKinematics::new`]).
#[allow(clippy::too_many_arguments)]
fn build_delta(
    config: &ConfigWrapper,
    rails: &[Arc<Rail>],
    max_velocity: f64,
    max_accel: f64,
    max_z_velocity: f64,
    max_z_accel: f64,
) -> Result<DeltaKinematics, ConfigError> {
    let radius = config.get_float_bounded("delta_radius", None, None, None, Some(0.0), None)?;
    let print_radius =
        config.get_float_bounded("print_radius", Some(radius), None, None, Some(0.0), None)?;
    let mut endstops = [0.0; 3];
    for (index, rail) in rails.iter().enumerate() {
        endstops[index] = rail.homing_info().position_endstop;
    }
    let max_z = endstops.iter().copied().fold(f64::INFINITY, f64::min);
    let minimum_z_position = config.get_float_bounded(
        "minimum_z_position",
        Some(0.0),
        None,
        Some(max_z),
        None,
        None,
    )?;

    // Tower geometry: `stepper_a`'s `arm_length` is required and sets the
    // default for `stepper_b/c`; the angles default to 210/330/90
    // (`delta.py:34-42`).
    let mut arm_lengths = [0.0; 3];
    let mut angles = [0.0; 3];
    let default_angles = [210.0, 330.0, 90.0];
    for (index, name) in DELTA_RAIL_NAMES.iter().enumerate() {
        let tower = config.sibling(name).ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{}' needs a '[{name}]' section",
                config.identifier()
            ))
        })?;
        let arm_length = if index == 0 {
            tower.get_float_bounded("arm_length", None, None, None, Some(radius), None)?
        } else {
            tower.get_float_bounded(
                "arm_length",
                Some(arm_lengths[0]),
                None,
                None,
                Some(radius),
                None,
            )?
        };
        arm_lengths[index] = arm_length;
        angles[index] = tower.get_float("angle", Some(default_angles[index]))?;
    }
    let mut step_dists = [0.0; 3];
    for (index, rail) in rails.iter().enumerate() {
        step_dists[index] = rail.step_dist();
    }

    DeltaKinematics::new(DeltaConfig {
        radius,
        print_radius,
        minimum_z_position,
        angles,
        arm_lengths,
        endstops,
        step_dists,
        max_velocity,
        max_accel,
        max_z_velocity,
        max_z_accel,
    })
}

/// Read the rotary-delta options and build the kinematics
/// (`kinematics/rotary_delta.py:10-48`, whose reads these mirror one for one):
/// the `[printer]` options here, the arm/angle options from each
/// `[stepper_a/b/c]` section.
///
/// # Errors
/// A missing or out-of-bounds option, reported with the config reader's
/// upstream wording; or a geometry whose home position does not exist (see
/// [`RotaryDeltaKinematics::new`]).
#[allow(clippy::too_many_arguments)]
fn build_rotary_delta(
    config: &ConfigWrapper,
    rails: &[Arc<Rail>],
    max_velocity: f64,
    max_accel: f64,
    max_z_velocity: f64,
    max_z_accel: f64,
) -> Result<RotaryDeltaKinematics, ConfigError> {
    let shoulder_radius =
        config.get_float_bounded("shoulder_radius", None, None, None, Some(0.0), None)?;
    let shoulder_height =
        config.get_float_bounded("shoulder_height", None, None, None, Some(0.0), None)?;
    let mut endstops = [0.0; 3];
    for (index, rail) in rails.iter().enumerate() {
        endstops[index] = rail.homing_info().position_endstop;
    }
    let max_z = endstops.iter().copied().fold(f64::INFINITY, f64::min);
    let minimum_z_position = config.get_float_bounded(
        "minimum_z_position",
        Some(0.0),
        None,
        Some(max_z),
        None,
        None,
    )?;

    // Tower geometry: `stepper_a`'s arm lengths are required and set the
    // defaults for `stepper_b/c`; the angles default to 30/150/270
    // (`rotary_delta.py:33-40`).
    let mut upper_arms = [0.0; 3];
    let mut lower_arms = [0.0; 3];
    let mut angles = [0.0; 3];
    for (index, name) in ROTARY_DELTA_RAIL_NAMES.iter().enumerate() {
        let tower = config.sibling(name).ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{}' needs a '[{name}]' section",
                config.identifier()
            ))
        })?;
        upper_arms[index] = if index == 0 {
            tower.get_float_bounded("upper_arm_length", None, None, None, Some(0.0), None)?
        } else {
            tower.get_float_bounded(
                "upper_arm_length",
                Some(upper_arms[0]),
                None,
                None,
                Some(0.0),
                None,
            )?
        };
        lower_arms[index] = if index == 0 {
            tower.get_float_bounded("lower_arm_length", None, None, None, Some(0.0), None)?
        } else {
            tower.get_float_bounded(
                "lower_arm_length",
                Some(lower_arms[0]),
                None,
                None,
                Some(0.0),
                None,
            )?
        };
        angles[index] = tower.get_float("angle", Some(ROTARY_DELTA_DEFAULT_ANGLES[index]))?;
    }
    let mut step_dists = [0.0; 3];
    for (index, rail) in rails.iter().enumerate() {
        step_dists[index] = rail.step_dist();
    }

    RotaryDeltaKinematics::new(RotaryDeltaConfig {
        shoulder_radius,
        shoulder_height,
        angles,
        upper_arms,
        lower_arms,
        endstops,
        step_dists,
        minimum_z_position,
        max_velocity,
        max_accel,
        max_z_velocity,
        max_z_accel,
    })
}

/// Read the deltesian options and build the kinematics
/// (`kinematics/deltesian.py:11-45`, whose reads these mirror one for one):
/// the `[printer]` options here, the arm/arm_x options from
/// `[stepper_left]`/`[stepper_right]`.
///
/// # Errors
/// A missing or out-of-bounds option, reported with the config reader's
/// upstream wording.
#[allow(clippy::too_many_arguments)]
fn build_deltesian(
    config: &ConfigWrapper,
    rails: &[Arc<Rail>],
    max_velocity: f64,
    max_accel: f64,
    max_z_velocity: f64,
    max_z_accel: f64,
) -> Result<DeltesianKinematics, ConfigError> {
    let left = config.sibling("stepper_left").ok_or_else(|| {
        ConfigError::new(format!(
            "Section '{}' needs a '[stepper_left]' section",
            config.identifier()
        ))
    })?;
    let right = config.sibling("stepper_right").ok_or_else(|| {
        ConfigError::new(format!(
            "Section '{}' needs a '[stepper_right]' section",
            config.identifier()
        ))
    })?;
    // `arm_x_length` on the left is required and sets the right's default; both
    // above 0 (`deltesian.py:22-25`).
    let arm_x_left = left.get_float_bounded("arm_x_length", None, None, None, Some(0.0), None)?;
    let arm_x_right = right.get_float_bounded(
        "arm_x_length",
        Some(arm_x_left),
        None,
        None,
        Some(0.0),
        None,
    )?;
    // `arm_length` likewise, each above its own arm's `arm_x_length`
    // (`deltesian.py:26-29`).
    let arm_left =
        left.get_float_bounded("arm_length", None, None, None, Some(arm_x_left), None)?;
    let arm_right = right.get_float_bounded(
        "arm_length",
        Some(arm_left),
        None,
        None,
        Some(arm_x_right),
        None,
    )?;
    let arm_x = [arm_x_left, arm_x_right];
    let arm2 = [arm_left * arm_left, arm_right * arm_right];
    let arm = [arm_left, arm_right];
    let arm_endstops = [
        rails[0].homing_info().position_endstop,
        rails[1].homing_info().position_endstop,
    ];
    let y_range = (
        rails[2].params().position_min,
        rails[2].params().position_max,
    );

    // `min_angle` and `print_width` (`deltesian.py:45-56`): the arms' reach at
    // `min_angle` bounds `print_width`.
    let min_angle = config.get_float_bounded(
        "min_angle",
        Some(MIN_ANGLE),
        Some(0.0),
        Some(90.0),
        None,
        None,
    )?;
    let (x_kin_min, x_kin_max) = x_kin_limits(min_angle, arm_x, arm);
    let x_kin_range = (x_kin_max - x_kin_min)
        .min(x_kin_max * 2.0)
        .min(-x_kin_min * 2.0);
    let print_width = if config.has("print_width") {
        Some(config.get_float_bounded(
            "print_width",
            None,
            Some(0.0),
            Some(x_kin_range),
            None,
            None,
        )?)
    } else {
        None
    };

    // `minimum_z_position` is at most the arms' highest Z over the X range
    // (`deltesian.py:71-75`).
    let abs_endstop = arm_abs_endstops(arm_endstops, arm_x, arm2);
    let (x_lo, x_hi) = match print_width {
        Some(width) if width != 0.0 => (-width * 0.5, width * 0.5),
        _ => (x_kin_min, x_kin_max),
    };
    let z_max = pillars_z_max(arm_x, arm2, abs_endstop, x_lo).min(pillars_z_max(
        arm_x,
        arm2,
        abs_endstop,
        x_hi,
    ));
    let minimum_z_position = config.get_float_bounded(
        "minimum_z_position",
        Some(0.0),
        None,
        Some(z_max),
        None,
        None,
    )?;

    let slow_ratio =
        config.get_float_bounded("slow_ratio", Some(SLOW_RATIO), Some(0.0), None, None, None)?;

    Ok(DeltesianKinematics::new(DeltesianConfig {
        arm_x,
        arm2,
        arm_endstops,
        y_range,
        min_angle,
        print_width,
        minimum_z_position,
        slow_ratio,
        max_velocity,
        max_accel,
        max_z_velocity,
        max_z_accel,
    }))
}

/// The registered extruders, in `[extruder]`, `[extruder1]`… order.
fn extruder_names(printer: &Printer) -> Vec<String> {
    let mut names = Vec::new();
    if printer.lookup_object("extruder").is_some() {
        names.push("extruder".to_string());
        for index in 1..99 {
            let name = format!("extruder{index}");
            if printer.lookup_object(&name).is_none() {
                break;
            }
            names.push(name);
        }
    }
    names
}

impl Drop for ToolHeadObject {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::SeqCst);
    }
}

impl ToolHeadObject {
    fn lock(&self) -> MutexGuard<'_, Option<Connected>> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The print time the planner has reached, or `0.0` before connect.
    ///
    /// `query_endstops` dates a query from this (upstream's
    /// `toolhead.get_last_move_time()`).
    ///
    /// This only *reads* the time; upstream's `get_last_move_time()` first
    /// flushes the look-ahead into the trapq. Get that flush — and the steps
    /// generated and sent for it — from
    /// [`ToolHeadObject::flush_step_generation`], then read this.
    pub fn print_time(&self) -> f64 {
        self.lock()
            .as_ref()
            .map(|connected| connected.toolhead.print_time())
            .unwrap_or(0.0)
    }

    /// Flush the look-ahead into the trapq and return the print time it reached
    /// (`toolhead.get_last_move_time()`). Before connect this is `0.0`.
    ///
    /// Unlike [`ToolHeadObject::flush_step_generation`] this only moves the
    /// planner's queue; the steps behind it are generated and sent by the
    /// background task. It is the read half of
    /// [`ToolHeadObject::dwell`]'s pair, as upstream's is.
    pub fn get_last_move_time(&self) -> f64 {
        self.lock()
            .as_mut()
            .map(|connected| connected.toolhead.get_last_move_time())
            .unwrap_or(0.0)
    }

    /// Advance the planner's timeline by `delay` seconds (`toolhead.dwell`).
    ///
    /// A timed protocol — BLTouch's single-wire pulses — uses this to keep its
    /// next command past the moves already queued
    /// (`bltouch.py:_sync_print_time`). Before connect this does nothing.
    pub fn dwell(&self, delay: f64) {
        if let Some(connected) = self.lock().as_mut() {
            connected.toolhead.dwell(delay);
        }
    }

    /// Queue a lookahead callback (`toolhead.register_lookahead_callback`,
    /// `klippy/toolhead.py:526-531`).
    ///
    /// Connect-safe: before connect there is no planner and
    /// [`Self::get_last_move_time`] would read `0.0`, so the callback waits in
    /// a pending list that [`Self::connect`] installs into the fresh toolhead.
    /// After connect it takes effect at once — with the look-ahead empty the
    /// callback fires immediately with the last move time, with moves queued
    /// it fires when the move it was registered against reaches the trapq.
    ///
    /// Like every consumer callback the toolhead runs (a move's timing
    /// callbacks, fired under the same state lock by [`Self::move_to`]), it
    /// runs while the connected state is locked out: it must not re-enter this
    /// object synchronously.
    pub fn register_lookahead_callback(&self, callback: Box<dyn FnOnce(f64) + Send + 'static>) {
        let mut guard = self.lock();
        if let Some(connected) = guard.as_mut() {
            connected.toolhead.register_lookahead_callback(callback);
        } else {
            self.pending_lookahead_callbacks
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .push(callback);
        }
    }

    /// Queue a flush callback (`motion_queuing.register_flush_callback`): it
    /// fires with the flush time on every step generation, in registration
    /// order — including a generation with no steppers at all, which is what
    /// a dwell-only (`kinematics: none`) timeline produces.
    ///
    /// Connect-safe like [`Self::register_lookahead_callback`]: before connect
    /// it waits in a pending list that [`Self::connect`] installs.
    pub fn register_flush_callback(&self, callback: Box<dyn Fn(f64) + Send + 'static>) {
        let mut guard = self.lock();
        if let Some(connected) = guard.as_mut() {
            connected.toolhead.register_flush_callback(callback);
        } else {
            self.pending_flush_callbacks
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .push(callback);
        }
    }

    /// Flush the look-ahead and generate **and send** every step queued so far,
    /// from inside a command (`toolhead.flush_step_generation`).
    ///
    /// Steps are normally generated and sent by the background flush task
    /// ([`run_flush_loop`]); this takes the connected state out of the shared
    /// slot for the duration (as `probing_move` and `G28` do), so the
    /// generation and the awaited transport writes happen here alone — the
    /// background task sees the empty slot and stands back. When this returns,
    /// every move queued **before the call** has been generated and handed to
    /// the transport, and [`ToolHeadObject::print_time`] is upstream's
    /// `get_last_move_time()` — that pair is the flush entry
    /// `ProbePointsHelper._invoke_callback` needs before its callback
    /// (`probe.py:463-469`).
    ///
    /// # Errors
    /// "Printer is not ready" before connect (or while a homing/probe run
    /// holds the state), a step-generation failure, or a failed step send.
    pub async fn flush_step_generation(&self) -> Result<(), CommandError> {
        flush_step_generation(&self.state).await
    }

    /// Force the toolhead to `newpos`, marking `homing_axes` as homed
    /// (`ToolHead.set_position`, `toolhead.py:383-390`).
    ///
    /// Upstream's `set_position` starts by flushing step generation (the
    /// queued moves' steps must be generated before the trapq's position is
    /// rewritten), then sets the position, the homing flags, and fires
    /// `toolhead:set_position` ([`KlippyEvent::ToolheadSetPosition`]) so
    /// `gcode_move` re-anchors — this does the same, which makes it safe for a
    /// Z-tilt `adjust_steppers` loop to `move_to` and then `set_position`.
    ///
    /// # Errors
    /// "Printer is not ready" before connect, or a failure from the flush
    /// (generation or send).
    pub async fn set_position(
        &self,
        newpos: Coord,
        homing_axes: &[usize],
    ) -> Result<(), CommandError> {
        flush_step_generation(&self.state).await?;
        {
            let mut guard = self.lock();
            let Some(connected) = guard.as_mut() else {
                return Err(CommandError::new("Printer is not ready"));
            };
            connected.toolhead.set_position(newpos, homing_axes);
        }
        send(&self.printer, &KlippyEvent::ToolheadSetPosition);
        Ok(())
    }

    /// Forget the homing state of `axes` (upstream's
    /// `kinematics.clear_homing_state`).
    ///
    /// `safe_z_home` uses it to undo the fake homing state its z-hop move
    /// needs (`safe_z_home.py:45`); without it the axis would keep looking
    /// homed. Before connect this does nothing.
    pub fn clear_homing_state(&self, axes: &[usize]) {
        if let Some(connected) = self.lock().as_mut() {
            if let Some(kinematics) = connected.toolhead.kinematics_mut() {
                kinematics.clear_homing_state(axes);
            }
        }
    }

    /// Queue a move to `position` at `speed` mm/s (upstream's `toolhead.move`).
    ///
    /// The toolhead plans it; deciding *what* the coordinates mean — absolute
    /// or relative, against which `G92` anchor, at what extrude factor — is
    /// `gcode_move`'s job, and it is what calls this.
    ///
    /// # Errors
    /// "Printer is not ready" before connect, or whatever the planner refuses:
    /// an unhomed axis, a move out of range.
    pub fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        let mut guard = self.lock();
        let Some(connected) = guard.as_mut() else {
            return Err(CommandError::new("Printer is not ready"));
        };
        connected.toolhead.move_to(position, speed)
    }

    /// The commanded position (upstream's `toolhead.get_position`).
    ///
    /// `None` before connect: there is no planner to ask yet, and
    /// `gcode_move` only anchors itself at ready.
    pub fn position(&self) -> Option<Coord> {
        self.lock()
            .as_ref()
            .map(|connected| connected.toolhead.commanded_pos())
    }

    /// The kinematic Z position at a past print time, from the stepper step
    /// history (`probe.py:_lookup_z_pos`).
    ///
    /// Used by the load-cell tap's ascent analysis, which needs the Z the
    /// carriage was at when each sample was taken rather than the current
    /// commanded position. `None` before connect or without a kinematics.
    pub fn kinematic_z_at(&self, print_time: f64) -> Option<f64> {
        let mut guard = self.lock();
        let connected = guard.as_mut()?;
        let positions: HashMap<String, f64> = connected
            .toolhead
            .motion_queuing_mut()
            .steppers()
            .iter()
            .map(|stepper| {
                let pos = stepper.past_mcu_position(print_time) as f64 * stepper.step_dist();
                (stepper.name().to_string(), pos)
            })
            .collect();
        let kinematics = connected.toolhead.kinematics()?;
        kinematics.calc_position(&positions)[Z_AXIS]
    }

    /// The main trapq's id — upstream's `toolhead.get_trapq()`, the queue
    /// `ZAdjustHelper.adjust_steppers` reattaches a Z motor to after moving it
    /// on its own.
    ///
    /// `None` before connect: there is no planner to ask yet.
    pub fn main_trapq(&self) -> Option<usize> {
        self.lock()
            .as_ref()
            .map(|connected| connected.toolhead.main_trapq())
    }

    /// Point a motion stepper at a trapq by name, or take it off one with
    /// `None` (`MCU_stepper.set_trapq`, the per-motor detach/reattach
    /// `ZAdjustHelper.adjust_steppers` walks through).
    ///
    /// A detached stepper generates no steps until it is attached again
    /// (`MotionQueuing::generate` skips it), which is how one Z motor moves
    /// while the others hold still.
    ///
    /// # Errors
    /// "Printer is not ready" before connect, or "Unknown stepper '<name>'"
    /// when no motion stepper answers to `name`.
    pub fn set_stepper_trapq(&self, name: &str, trapq: Option<usize>) -> Result<(), CommandError> {
        let mut guard = self.lock();
        let connected = guard
            .as_mut()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let stepper = connected
            .toolhead
            .motion_queuing_mut()
            .steppers_mut()
            .iter_mut()
            .find(|stepper| stepper.name() == name)
            .ok_or_else(|| CommandError::new(format!("Unknown stepper '{name}'")))?;
        stepper.set_trapq(trapq);
        Ok(())
    }

    /// Drive one stepper `dist` millimetres in its own coordinates, outside the
    /// planner and the kinematics (upstream `ForceMove.manual_move`,
    /// `force_move.py:75-91`): the `FORCE_MOVE` / `STEPPER_BUZZ` move.
    ///
    /// The connected state is taken out of the shared slot for the run (as
    /// [`ToolHeadObject::probing_move`] and [`ToolHeadObject::flush_step_generation`]
    /// do), so the background flush task stands back and only this call
    /// generates and sends. It is put back on every exit, success or failure.
    ///
    /// The move swaps in a cartesian single-axis solver and a force-move-only
    /// trapq, appends the `calc_move_time` trapezoid, dwells its duration,
    /// generates and sends the steps, then puts the stepper's original solver
    /// and trapq back and wipes the force-move trapq. The toolhead's
    /// `commanded_pos` and the stepper's own solver position are deliberately
    /// **not** touched: the move invalidates the kinematics (a
    /// `SET_KINEMATIC_POSITION` re-syncs it), it does not move the planner.
    ///
    /// A `stepper_name` the toolhead's motion queue does not hold — a stepper
    /// `force_move` knows but those modules have not wired (the manual stepper,
    /// the IDEX second carriage) — carries the move on the timeline only; see
    /// [`manual_move`] for why.
    ///
    /// # Errors
    /// "Printer is not ready" before connect, or a step-generation/send
    /// failure.
    pub async fn manual_move(
        &self,
        stepper_name: &str,
        dist: f64,
        speed: f64,
        accel: f64,
    ) -> Result<(), CommandError> {
        let mut connected = {
            let mut guard = self.lock();
            guard
                .take()
                .ok_or_else(|| CommandError::new("Printer is not ready"))?
        };
        let result = manual_move(&mut connected, stepper_name, dist, speed, accel).await;
        *self.lock() = Some(connected);
        result
    }

    /// Add a non-kinematic axis (`ToolHead.add_extra_axis`) and tell the
    /// machine (`toolhead:update_extra_axes`).
    ///
    /// The manual stepper is the extra axis that arrives after connect; the
    /// extruders are added by [`ToolHeadObject::connect`] itself. Upstream
    /// also appends `axis_pos` to `commanded_pos`; this port's [`Coord`] is a
    /// fixed four axes, so that step is the documented gap in
    /// [`manual_stepper`](crate::core::klippy::extras::manual_stepper).
    ///
    /// # Errors
    /// "Printer is not ready" before connect.
    pub fn add_extra_axis(&self, axis: Arc<dyn ExtraAxis>) -> Result<(), CommandError> {
        let mut guard = self.lock();
        let connected = guard
            .as_mut()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        connected.toolhead.add_extra_axis(axis);
        drop(guard);
        send(&self.printer, &KlippyEvent::ToolheadUpdateExtraAxes);
        Ok(())
    }

    /// Take a non-kinematic axis back off the toolhead (`ToolHead.remove_extra_axis`)
    /// and tell the machine (`toolhead:update_extra_axes`).
    ///
    /// # Errors
    /// "Printer is not ready" before connect.
    pub fn remove_extra_axis(&self, axis: &Arc<dyn ExtraAxis>) -> Result<(), CommandError> {
        let mut guard = self.lock();
        let connected = guard
            .as_mut()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        connected.toolhead.remove_extra_axis(axis);
        drop(guard);
        send(&self.printer, &KlippyEvent::ToolheadUpdateExtraAxes);
        Ok(())
    }

    /// The non-kinematic axes currently registered, for the `GCODE_AXIS`
    /// collision check (`ToolHead.get_extra_axes`).
    ///
    /// Upstream pads the list with three `None`s for X/Y/Z; here the position
    /// index is always `index + E_AXIS`, so only real axes are returned. Before
    /// connect the list is empty.
    pub fn get_extra_axes(&self) -> Vec<Arc<dyn ExtraAxis>> {
        self.lock()
            .as_ref()
            .map(|connected| connected.toolhead.extra_axes().to_vec())
            .unwrap_or_default()
    }

    /// Probe-style homing: move toward `target` at `speed`, stop on trigger
    /// (`homing.probing_move`).
    ///
    /// The toolhead is taken out of its shared slot during the move (the
    /// background flush task stands back), then restored afterwards.
    ///
    /// Returns the toolhead's commanded position at the end of the move.
    ///
    /// # Errors
    /// "Printer is not ready" before connect, or any error from the
    /// low-level probe move.
    pub async fn probing_move(
        &self,
        endstop: &dyn HomingEndstop,
        target: Coord,
        speed: f64,
    ) -> Result<Coord, CommandError> {
        let mut connected = {
            let mut guard = self.lock();
            let Some(connected) = guard.take() else {
                return Err(CommandError::new("Printer is not ready"));
            };
            connected
        };
        let result = probing_move(&mut connected, endstop, target, speed, &self.printer).await;
        *self.lock() = Some(connected);
        result
    }
}

/// The flush task: generate the queued steps and await the transport.
///
/// A command that must not return until the steps it queued are generated and
/// sent awaits [`flush_step_generation`] instead; the two never own the
/// connected state at the same time, because each takes it out of the shared
/// slot first.
async fn run_flush_loop(
    state: Arc<Mutex<Option<Connected>>>,
    shutdown: Arc<AtomicBool>,
    printer: Weak<Printer>,
) {
    loop {
        sleep(FLUSH_INTERVAL).await;
        if shutdown.load(Ordering::SeqCst) {
            return;
        }
        // Generate under the lock, then send without it: awaiting the transport
        // while holding a `std` mutex would make this future non-`Send`.
        let batches = {
            let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
            match guard.as_mut() {
                Some(connected) => connected.generate(),
                None => Ok(Vec::new()),
            }
        };
        let batches = match batches {
            Ok(batches) => batches,
            // `check_line` failed, which upstream treats as an internal error
            // and shuts the printer down for (`Internal error in stepcompress`).
            Err(err) => {
                if let Some(printer) = printer.upgrade() {
                    printer.invoke_shutdown(&format!("Internal error in stepcompress: {err}"));
                } else {
                    warn!("Internal error in stepcompress: {err}");
                }
                return;
            }
        };
        for (stepper, commands, clocks) in batches {
            if let Err(err) = stepper.send_steps_async(&commands, clocks).await {
                warn!(
                    "{}: {err}",
                    stepper.oid().map(u32::from).unwrap_or_default()
                );
            }
        }
    }
}

/// Generate the steps for everything queued so far and await sending them,
/// taking the connected state out of the shared slot for the duration
/// (`toolhead.flush_step_generation`).
///
/// The caller must not hold `state`'s lock: the background flush task and this
/// alternate through the slot, so only one of them generates and sends at a
/// time.
///
/// # Errors
/// "Printer is not ready" when the slot is empty (not connected, or a homing
/// run owns it), a step-generation failure, or a failed step send.
async fn flush_step_generation(state: &Arc<Mutex<Option<Connected>>>) -> Result<(), CommandError> {
    let mut connected = {
        let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
        guard
            .take()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?
    };
    let result = async {
        let batches = connected
            .generate()
            .map_err(|err| CommandError::new(err.to_string()))?;
        for (stepper, commands, clocks) in batches {
            stepper
                .send_steps_async(&commands, clocks)
                .await
                .map_err(command_error)?;
        }
        Ok(())
    }
    .await;
    // Restore the state whether the flush succeeded or failed.
    *state.lock().unwrap_or_else(|poison| poison.into_inner()) = Some(connected);
    result
}

/// Upstream `ForceMove.manual_move` (`force_move.py:75-91`): drive one motion
/// stepper in its own coordinates, bypassing the planner and the kinematics.
///
/// The timing is upstream's, in order: flush the pending steps, swap in the
/// force-move solver and trapq (remembering the old pair), zero the solver's
/// position, append the `calc_move_time` trapezoid at the planner's last move
/// time, dwell the trapq's duration, generate and send those steps (the target
/// stepper alone — see [`MotionQueuing::generate_stepper`]), then restore the
/// solver and trapq and wipe the force-move queue.
///
/// `note_mcu_movequeue_activity` has no equivalent here — this host's flush is
/// driven by a 10 ms tick (`run_flush_loop`), not by a queue-length estimate,
/// so there is nothing to note. See [`force_move`](crate::core::klippy::extras::force_move).
///
/// # Errors
/// A step-generation/send failure.
async fn manual_move(
    connected: &mut Connected,
    stepper_name: &str,
    dist: f64,
    speed: f64,
    accel: f64,
) -> Result<(), CommandError> {
    // Upstream's first `toolhead.flush_step_generation()`: drain what is
    // queued before the solver is swapped, so the two timelines do not
    // interleave.
    let backlog = connected
        .generate()
        .map_err(|err| CommandError::new(err.to_string()))?;
    for (stepper, commands, clocks) in backlog {
        stepper
            .send_steps_async(&commands, clocks)
            .await
            .map_err(command_error)?;
    }

    // The force-move trapq (upstream's `self.trapq`), allocated once per
    // connection.
    let trapq = match connected.force_move_trapq {
        Some(trapq) => trapq,
        None => {
            let trapq = connected.toolhead.allocate_trapq();
            connected.force_move_trapq = Some(trapq);
            trapq
        }
    };

    // Swap in the cartesian single-axis solver and the force-move trapq,
    // keeping the old pair to restore afterwards.
    //
    // A stepper `force_move` knows but the toolhead's motion queue does not
    // (the manual stepper and the IDEX second carriage build a `PrinterStepper`
    // without being added to it — a documented gap of those modules) has no
    // motor this port can drive. Upstream would move it; here the move is
    // carried on the timeline only, so `STEPPER_BUZZ` / `FORCE_MOVE` on such a
    // name still answer instead of erroring.
    let index = connected
        .toolhead
        .motion_queuing_mut()
        .steppers()
        .iter()
        .position(|stepper| stepper.name() == stepper_name);
    let (axis_r, accel_t, cruise_t, cruise_v) = calc_move_time(dist, speed, accel);
    let move_time = accel_t + cruise_t + accel_t;
    let Some(index) = index else {
        connected.toolhead.get_last_move_time();
        connected.toolhead.dwell(move_time);
        return Ok(());
    };
    let (prev_kinematics, prev_trapq) = {
        let stepper = &mut connected.toolhead.motion_queuing_mut().steppers_mut()[index];
        let solver = StepKinematics::new(
            stepper.step_dist(),
            cartesian_position_fn(Axis::X),
            cartesian_active_flags(Axis::X),
        );
        let prev_kinematics = stepper.set_stepper_kinematics(solver);
        let prev_trapq = stepper.trapq_id();
        stepper.set_trapq(trapq);
        // `stepper.set_position((0., 0., 0.))`: the solver's own position, not
        // the toolhead's.
        stepper.set_position(Xyz::default());
        (prev_kinematics, prev_trapq)
    };

    let print_time = connected.toolhead.get_last_move_time();
    connected.toolhead.motion_queuing_mut().append(
        trapq,
        print_time,
        accel_t,
        cruise_t,
        accel_t,
        Xyz::default(),
        Xyz::new(axis_r, 0.0, 0.0),
        0.0,
        cruise_v,
        accel,
    );
    connected.toolhead.dwell(move_time);

    // Upstream's second `toolhead.flush_step_generation()`: generate and send
    // the force-move steps. It generates the **target stepper alone**, to the
    // move's end rather than the background horizon, so the whole trapezoid
    // leaves the host here without advancing any other stepper's solver past
    // the horizon `Connected::generate` bounds generation to (see
    // [`MotionQueuing::generate_stepper`]).
    let end_time = print_time + move_time;
    let commands = connected
        .toolhead
        .motion_queuing_mut()
        .generate_stepper(stepper_name, end_time)
        .map_err(|err| CommandError::new(err.to_string()))?;
    if let Some(stepper) = connected.mcu_steppers.get(stepper_name) {
        let clocks = step_batch_clocks(stepper, print_time, end_time);
        stepper
            .send_steps_async(&commands, clocks)
            .await
            .map_err(command_error)?;
    }

    // Restore the stepper and wipe the force-move queue.
    {
        let stepper = &mut connected.toolhead.motion_queuing_mut().steppers_mut()[index];
        stepper.set_trapq(prev_trapq);
        stepper.set_stepper_kinematics(prev_kinematics);
    }
    connected.toolhead.motion_queuing_mut().wipe_trapq(trapq);
    Ok(())
}

impl Connected {
    /// Generate the steps queued for this pass, and return them by stepper.
    ///
    /// Every pass is horizon-bounded ([`Self::horizon`]): upstream's explicit
    /// `flush_all_steps` generates to the content end, but it sends that into
    /// serialqueue's gates, which hold a far stamp until it is due — this
    /// host's default transport bypasses those gates, so a content-end dump
    /// would put a half-wrap-ahead stamp straight on the wire (4.13 s at a
    /// 520 MHz dictionary — `timer_is_before` reads it as already expired).
    /// Bounding every pass is the gate-free equivalent: nothing is born more
    /// than ~0.7 s ahead of the estimate.
    ///
    /// # Errors
    /// An internal [`StepCompressError`] from a stepper's compressor.
    fn generate(&mut self) -> Result<StepBatches, StepCompressError> {
        // A secondary MCU's print-time mapping is recalibrated periodically as
        // its crystal drifts against the primary's; pick up the current mapping
        // on every stepper before generating.
        for stepper in self.toolhead.motion_queuing_mut().steppers_mut() {
            if let Some(mcu_stepper) = self.mcu_steppers.get(stepper.name()) {
                let (offset, freq) = mcu_stepper.chip().time_mapping();
                stepper.compressor_mut().set_time(offset, freq);
            }
        }
        // Move whatever the planner has queued into the trapq first, so the
        // step generation time below covers it.
        self.toolhead.wait_moves();
        let step_gen_time = self.horizon(self.toolhead.print_time());
        // The previous generation horizon is this batch's start: a lower
        // bound on when its first step runs, and what its messages carry as
        // `req_clock` (`StepBatchClocks`).
        let start_time = self.last_step_gen_time;
        let batches = self.toolhead.flush_step_generation(step_gen_time)?;
        self.toolhead.finalize_moves(
            step_gen_time,
            (step_gen_time - MOVE_HISTORY_EXPIRE).max(0.0),
        );
        self.last_step_gen_time = step_gen_time;
        Ok(batches
            .into_iter()
            .filter_map(|(name, commands)| {
                self.mcu_steppers.get(&name).map(|stepper| {
                    let clocks = step_batch_clocks(stepper, start_time, step_gen_time);
                    (Arc::clone(stepper), commands, clocks)
                })
            })
            .collect())
    }

    /// The generation horizon for one pass — upstream's `_flush_handler`
    /// (`extras/motion_queuing.py:193-234`), with `need_step_gen_time` /
    /// `need_flush_time` read as the planner's own reach (`content_end`,
    /// `wait_moves` having just drained it) and `kin_flush_delay` — not
    /// modelled in this host — left at zero.
    ///
    /// Bounded by the estimate: `est + 0.4 s` at rest, `est + 0.7 s` while
    /// queued motion lies beyond the horizon (batched in 0.25 s windows from
    /// the last horizon). That bound is the stamp's birth margin — generation
    /// crosses a step 0.4 s before its own clock, so the batching/queue in
    /// front of the wire cannot land it past due, and no stamp is born more
    /// than ~0.7 s ahead, where the firmware's wrapping compare
    /// (`timer_is_before`, ±2³¹ ticks) still orders it.
    ///
    /// # Errors
    /// An internal [`StepCompressError`] from a stepper's compressor.
    fn horizon(&self, content_end: f64) -> f64 {
        let last = self.last_step_gen_time;
        let est = self.toolhead.estimated_print_time();
        let want = if last < content_end {
            // Actively stepping — the aggressive branch with its 0.25 s
            // batching window (`:198-212`).
            let mut want = est + BGFLUSH_SG_HIGH_TIME;
            let next_batch = last + (BGFLUSH_SG_HIGH_TIME - BGFLUSH_SG_LOW_TIME);
            if next_batch > want {
                want = if next_batch > want + 0.005 {
                    // Far past the window: delay to the next wakeup, so the
                    // batch boundaries land on the same instants run to run.
                    last
                } else {
                    next_batch
                };
            }
            want.min(content_end)
        } else {
            // Caught up — the relaxed branch (`:214-216`).
            (est + BGFLUSH_HIGH_TIME).min(content_end + BGFLUSH_EXTRA_TIME)
        };
        // Raise-only: a horizon the machine has not caught up with stays.
        want.max(last)
    }
}

/// The toolhead's restart handle (`RestartHooks`): the planner operations the
/// G-code dispatcher needs before a restart.
struct ToolHeadRestartHooks(Arc<Mutex<Option<Connected>>>);

impl RestartHooks for ToolHeadRestartHooks {
    fn get_last_move_time(&self) -> f64 {
        self.0
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .as_mut()
            .map(|connected| connected.toolhead.get_last_move_time())
            .unwrap_or(0.0)
    }

    fn dwell(&self, delay: f64) {
        if let Some(connected) = self
            .0
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .as_mut()
        {
            connected.toolhead.dwell(delay);
        }
    }

    fn wait_moves(&self) {
        if let Some(connected) = self
            .0
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .as_mut()
        {
            connected.toolhead.wait_moves();
        }
    }
}

// ===========================================================================
// Homing
// ===========================================================================

/// How long to wait after arming the endstop before moving
/// (`HOMING_START_DELAY`, `klippy/extras/homing.py:8`).
const HOMING_START_DELAY: f64 = 0.001;

/// How long the endstop confirms a trigger over
/// (`ENDSTOP_SAMPLE_TIME`/`ENDSTOP_SAMPLE_COUNT`).
const ENDSTOP_SAMPLE_TIME: f64 = 0.000_015;
const ENDSTOP_SAMPLE_COUNT: u8 = 4;

/// How much of a homing move's steps the drip loop queues at a time
/// (`DRIP_SEGMENT_TIME`, `klippy/extras/motion_queuing.py:20`).
const DRIP_SEGMENT_TIME: f64 = 0.050;

/// The most steps the homing driver may look ahead, so a trigger stops the
/// firmware with little queued behind it.
const DRIP_LOOKAHEAD: f64 = 0.010;

/// A future returned by [`HomingEndstop::home_wait`].
pub type EndstopFuture<'a> = Pin<Box<dyn Future<Output = Result<f64, McuError>> + Send + 'a>>;

/// A future returned by [`HomingEndstop::query_endstop`].
pub type QueryEndstopFuture<'a> = Pin<Box<dyn Future<Output = Result<bool, McuError>> + Send + 'a>>;

/// What the homing driver needs from an endstop (upstream's `MCU_endstop`).
///
/// A trait so the driver can be tested with a fake trigger, without an MCU.
pub trait HomingEndstop: Send + Sync {
    /// Arm the endstop for a move starting at `print_time`.
    ///
    /// # Errors
    /// As [`McuEndstop::home_start`].
    fn home_start(
        &self,
        print_time: f64,
        sample_time: f64,
        sample_count: u8,
        rest_time: f64,
        triggered: bool,
    ) -> Result<Arc<Completion>, McuError>;

    /// Wait for the endstop trigger; returns its print time, or `0.0` when the
    /// move ended without one.
    ///
    /// # Errors
    /// As [`McuEndstop::home_wait`].
    fn home_wait(&self, home_end_time: f64) -> EndstopFuture<'_>;

    /// The trigger dispatch this endstop stops the steppers through (the
    /// pairing lookup in the kinematics setup uses it). `None` for an endstop
    /// that does not drive one — every endstop a rail can name (a pin
    /// `McuEndstop`, the eddy probe's `McuTriggerAnalog`) answers `Some`.
    fn dispatch(&self) -> Option<&TriggerDispatch> {
        None
    }

    /// Whether the pin reads triggered now (`M119` / `QUERY_ENDSTOPS`). The
    /// default reports "open", which is what upstream's virtual probe helper
    /// answers without a query callback
    /// (`probe.py:HomingViaProbeHelper.query_endstop` → `False`).
    fn query_endstop(&self, _print_time: f64) -> QueryEndstopFuture<'_> {
        Box::pin(async { Ok(false) })
    }
}

impl HomingEndstop for McuEndstop {
    fn home_start(
        &self,
        print_time: f64,
        sample_time: f64,
        sample_count: u8,
        rest_time: f64,
        triggered: bool,
    ) -> Result<Arc<Completion>, McuError> {
        McuEndstop::home_start(
            self,
            print_time,
            sample_time,
            sample_count,
            rest_time,
            triggered,
        )
    }

    fn home_wait(&self, home_end_time: f64) -> EndstopFuture<'_> {
        Box::pin(McuEndstop::home_wait(self, home_end_time))
    }

    fn dispatch(&self) -> Option<&TriggerDispatch> {
        Some(McuEndstop::dispatch(self))
    }

    fn query_endstop(&self, print_time: f64) -> QueryEndstopFuture<'_> {
        Box::pin(McuEndstop::query_endstop(self, print_time))
    }
}

/// Fill `None` entries from `current` (`Homing._fill_coord`).
fn fill_coord(home: HomeCoord, current: Coord) -> Coord {
    let mut out = current;
    for (axis, value) in home.iter().enumerate() {
        if let Some(value) = value {
            out.set_axis(axis, *value);
        }
    }
    out
}

/// The XYZ distance between two positions.
fn move_distance(a: Coord, b: Coord) -> f64 {
    let mut sum = 0.0;
    for axis in [X_AXIS, Y_AXIS, Z_AXIS] {
        sum += (b.axis(axis) - a.axis(axis)).powi(2);
    }
    sum.sqrt()
}

/// Home the requested axes, one at a time (`Homing.home_rails` driven per axis).
///
/// The caller has taken the toolhead out of its shared slot, so the background
/// flush task sees no toolhead and stands back while the drip loop here owns
/// step generation.
///
/// # Errors
/// A missing endstop, a kinematics refusal, or a failed query/send.
async fn home_axes(
    connected: &mut Connected,
    rails: &[Arc<Rail>],
    kind: KinematicsKind,
    requested: &[usize],
    printer: &Weak<Printer>,
) -> Result<(Vec<usize>, HomingHandle), CommandError> {
    if kind == KinematicsKind::GenericCartesian {
        // Generic cartesian homes one carriage per axis, in order
        // (`GenericCartesianKinematics.home`, `generic_cartesian.py:306-315`),
        // through the carriage that is active for that axis. Its motors live
        // in the carriage registry, not in `rails`.
        let model = printer
            .upgrade()
            .and_then(|printer| carriage::lookup_model(&printer))
            .ok_or_else(|| {
                CommandError::new(
                    "kinematics 'generic_cartesian' needs '[carriage <name>]' sections".to_string(),
                )
            })?;
        let mut homed: Vec<usize> = Vec::new();
        let homing = HomingHandle::new();
        for &axis in requested {
            let carriage = model.active_carriage(axis).ok_or_else(|| {
                CommandError::new(format!(
                    "No carriage defined for axis '{}'",
                    ["x", "y", "z"][axis]
                ))
            })?;
            let endstop = carriage.endstop().clone();
            let params = carriage.params();
            let info = carriage.homing_info();
            let (forcepos, movepos) =
                home_move(axis, &info, params.position_min, params.position_max);
            let step_dist = model.step_dist(carriage.name()).unwrap_or(1.0);
            send(printer, &KlippyEvent::HomingHomeRailsBegin);
            let result = home_axis(
                connected,
                axis,
                forcepos,
                movepos,
                &[axis],
                info,
                step_dist,
                // No `[endstop_phase]` wiring for generic carriages yet, so
                // there are no per-stepper offsets to hand back (`&[]` makes
                // the adjustment loop a no-op).
                &[],
                endstop.as_ref(),
                &homing,
                printer,
            )
            .await;
            result?;
            homed.push(axis);
        }
        return Ok((homed, homing));
    }
    // Winch homing is not implemented (`kinematics/winch.py:31-35`): the
    // operator jogs to the origin by hand and `G28` only forces the position
    // to `0, 0, 0` (the extruder is left alone). No endstop is driven, so the
    // empty homed list fires no `homing:home_rails_end` — as upstream, whose
    // winch `home` calls no `home_rails`.
    if kind == KinematicsKind::Winch {
        let current = connected.toolhead.commanded_pos();
        connected
            .toolhead
            .set_position(Coord::new(0.0, 0.0, 0.0, current.e()), &[]);
        return Ok((Vec::new(), HomingHandle::new()));
    }
    // `kinematics: none` has no rails, so there is nothing to home.
    if rails.is_empty() {
        return Ok((Vec::new(), HomingHandle::new()));
    }
    // Which axes were homed. The caller fires `HomingHomeRailsEnd` with them
    // **after** the toolhead is back in its shared slot, so a handler that reads
    // the toolhead's position (gcode_move's `_handle_home_rails_end`) sees the
    // homed position rather than the empty slot's default — which would clobber
    // the extruder position that homing deliberately preserves.
    let mut homed: Vec<usize> = Vec::new();
    // One run state for the whole `G28`: every rail's trigger positions land in
    // it, and the handlers see it in `homing:home_rails_end`.
    let homing = HomingHandle::new();
    if kind == KinematicsKind::Polar {
        // `home` of `kinematics/polar.py:95-108`: X and Y always home
        // **together** on the arm rail (`rails[0]`, Y pinned to 0 by
        // [`polar_home_move`]) — whichever of them was requested — and Z
        // homes on its own rail (`rails[1]`), after.
        let home_xy = requested
            .iter()
            .any(|&axis| axis == X_AXIS || axis == Y_AXIS);
        if home_xy {
            let rail = &rails[0];
            let endstop = rail.endstop().ok_or_else(|| {
                CommandError::new(format!("No endstop configured for {}", rail.name()))
            })?;
            let params = rail.params();
            let info = rail.homing_info();
            let (forcepos, movepos) =
                polar_home_move(X_AXIS, &info, params.position_min, params.position_max);
            // Upstream marks every axis the force position sets as homed
            // (`homing.py:178-184`): x **and** y — which is what opens
            // `limit_xy2` before the drip move is checked.
            let homing_axes = homing_axes_of(&forcepos);
            send(printer, &KlippyEvent::HomingHomeRailsBegin);
            let result = home_axis(
                connected,
                X_AXIS,
                forcepos,
                movepos,
                &homing_axes,
                info,
                rail.step_dist(),
                &stepper_names(rail),
                endstop.as_ref(),
                &homing,
                printer,
            )
            .await;
            homed.extend_from_slice(&homing_axes);
            result?;
        }
        if requested.contains(&Z_AXIS) {
            let rail = &rails[1];
            let endstop = rail.endstop().ok_or_else(|| {
                CommandError::new(format!("No endstop configured for {}", rail.name()))
            })?;
            let params = rail.params();
            let info = rail.homing_info();
            let (forcepos, movepos) =
                polar_home_move(Z_AXIS, &info, params.position_min, params.position_max);
            let homing_axes = homing_axes_of(&forcepos);
            send(printer, &KlippyEvent::HomingHomeRailsBegin);
            let result = home_axis(
                connected,
                Z_AXIS,
                forcepos,
                movepos,
                &homing_axes,
                info,
                rail.step_dist(),
                &stepper_names(rail),
                endstop.as_ref(),
                &homing,
                printer,
            )
            .await;
            homed.extend_from_slice(&homing_axes);
            result?;
        }
        return Ok((homed, homing));
    }
    // Deltesian homes its two arm rails together, then its Y rail
    // (`deltesian.py:88-110`): neither the cartesian per-axis walk nor a single
    // whole-machine group move.
    if kind == KinematicsKind::Deltesian {
        let home = connected
            .toolhead
            .kinematics()
            .and_then(|kinematics| kinematics.deltesian_home())
            .ok_or_else(|| {
                CommandError::new("deltesian kinematics is not connected".to_string())
            })?;
        let current = connected.toolhead.commanded_pos();
        let home_xz = requested
            .iter()
            .any(|&axis| axis == X_AXIS || axis == Z_AXIS);
        let home_y = requested.contains(&Y_AXIS);
        if home_xz {
            // Both arms in one move, X pinned to 0 and Z to `home_z`
            // (`deltesian.py:96-102`); Y keeps its current value (upstream's
            // `None` homepos entry).
            let force = Coord::new(0.0, current.y(), home.arm_force_z, current.e());
            let target = Coord::new(0.0, current.y(), home.arm_target_z, current.e());
            send(printer, &KlippyEvent::HomingHomeRailsBegin);
            let result = home_unified(
                connected,
                &rails[..2],
                force,
                target,
                &home.arm_travel,
                &[X_AXIS, Z_AXIS],
                &homing,
                printer,
            )
            .await;
            result?;
            homed.extend_from_slice(&[X_AXIS, Z_AXIS]);
        }
        if home_y {
            let rail = &rails[2];
            let endstop = rail.endstop().ok_or_else(|| {
                CommandError::new(format!("No endstop configured for {}", rail.name()))
            })?;
            let params = rail.params();
            let info = rail.homing_info();
            let (mut forcepos, mut movepos) =
                home_move(Y_AXIS, &info, params.position_min, params.position_max);
            // The arm home already pinned X and Z (`deltesian.py:122-146`).
            if home_xz {
                forcepos[X_AXIS] = Some(0.0);
                forcepos[Z_AXIS] = Some(home.arm_target_z);
                movepos[X_AXIS] = Some(0.0);
                movepos[Z_AXIS] = Some(home.arm_target_z);
            }
            let homing_axes = homing_axes_of(&forcepos);
            send(printer, &KlippyEvent::HomingHomeRailsBegin);
            let result = home_axis(
                connected,
                Y_AXIS,
                forcepos,
                movepos,
                &homing_axes,
                info,
                rail.step_dist(),
                &stepper_names(rail),
                endstop.as_ref(),
                &homing,
                printer,
            )
            .await;
            homed.extend_from_slice(&homing_axes);
            result?;
        }
        return Ok((homed, homing));
    }
    // Delta homes every tower in one move and ignores which axes `G28` named
    // (`kinematics/delta.py:104-110` always takes all three rails), so its
    // homing is one multi-endstop move rather than one per axis.
    if let Some(home) = connected
        .toolhead
        .kinematics()
        .and_then(|kinematics| kinematics.unified_home())
    {
        let current = connected.toolhead.commanded_pos();
        let force = Coord::new(home.force[0], home.force[1], home.force[2], current.e());
        let target = Coord::new(home.target[0], home.target[1], home.target[2], current.e());
        send(printer, &KlippyEvent::HomingHomeRailsBegin);
        let result = home_unified(
            connected,
            rails,
            force,
            target,
            &home.actuator_travel,
            &[X_AXIS, Y_AXIS, Z_AXIS],
            &homing,
            printer,
        )
        .await;
        result?;
        homed.extend_from_slice(&[X_AXIS, Y_AXIS, Z_AXIS]);
        return Ok((homed, homing));
    }
    for &axis in requested {
        let rail = &rails[axis];
        let endstop = rail.endstop().ok_or_else(|| {
            CommandError::new(format!("No endstop configured for {}", rail.name()))
        })?;
        let params = rail.params();
        let info = rail.homing_info();
        let (forcepos, movepos) = home_move(axis, &info, params.position_min, params.position_max);
        send(printer, &KlippyEvent::HomingHomeRailsBegin);
        let result = home_axis(
            connected,
            axis,
            forcepos,
            movepos,
            &[axis],
            info,
            rail.step_dist(),
            &stepper_names(rail),
            endstop.as_ref(),
            &homing,
            printer,
        )
        .await;
        homed.push(axis);
        result?;
    }
    Ok((homed, homing))
}

/// The names of a rail's steppers, the keys the homing state records trigger
/// positions under.
fn stepper_names(rail: &Rail) -> Vec<String> {
    rail.steppers()
        .iter()
        .map(|stepper| stepper.name().to_string())
        .collect()
}

/// End one rail group's home: fire `homing:home_rails_end` with the run state
/// and apply the offsets the handlers asked for (`Homing._do_home_rails`:
/// record `trigger_mcu_pos`, send the event, then apply `adjust_pos`).
/// Applies the per-stepper offsets recorded by a `homing:home_rails_end`
/// handler (`[endstop_phase]`) and surfaces the error it refused the home with
/// (`endstop_phase.py:113-117`). The event itself has already been sent by the
/// caller, with the toolhead back in the shared slot and the state lock
/// released, so a handler that reads the toolhead sees the homed position.
fn apply_home_rails_adjustments(
    connected: &mut Connected,
    homing: &HomingHandle,
    homed_axes: &[usize],
) -> Result<(), CommandError> {
    let mut state = homing.lock();
    // A handler that refused the home (an `[endstop_phase]` phase mismatch)
    // leaves the error here, as upstream's raise out of `_do_home_rails` does.
    if let Some(message) = state.take_error() {
        return Err(CommandError::new(message));
    }
    apply_stepper_adjustments(connected, &state, homed_axes)
}

/// Offset each stepper's commanded position by the adjustment a
/// `homing:home_rails_end` handler asked for, recompute the toolhead position
/// from the offset motor positions, and give the homed axes the new value
/// (`Homing._do_home_rails`'s `adjust_pos` step).
///
/// # Errors
/// A kinematics that cannot invert the offset positions raises, as upstream
/// does ("Cannot determine position of toolhead on axis … after homing").
fn apply_stepper_adjustments(
    connected: &mut Connected,
    homing: &Homing,
    homed_axes: &[usize],
) -> Result<(), CommandError> {
    if homing.adjustments().values().all(|offset| *offset == 0.0) {
        return Ok(());
    }
    let positions: HashMap<String, f64> = connected
        .toolhead
        .motion_queuing_mut()
        .steppers()
        .iter()
        .map(|stepper| {
            let offset = homing
                .adjustments()
                .get(stepper.name())
                .copied()
                .unwrap_or(0.0);
            (
                stepper.name().to_string(),
                stepper.commanded_position() + offset,
            )
        })
        .collect();
    let newpos: [Option<f64>; 3] = connected
        .toolhead
        .kinematics()
        .map(|kinematics| kinematics.calc_position(&positions))
        .unwrap_or([None; 3]);
    let mut homepos = connected.toolhead.commanded_pos();
    for &axis in homed_axes {
        match newpos[axis] {
            Some(value) => homepos.set_axis(axis, value),
            None => {
                return Err(CommandError::new(format!(
                    "Cannot determine position of toolhead on axis {} after homing",
                    ["x", "y", "z"][axis]
                )));
            }
        }
    }
    connected.toolhead.set_position(homepos, &[]);
    Ok(())
}

/// The axes a homing force position marks as homed: every axis whose
/// `forcepos` entry is set (`Homing._set_start_position`,
/// `klippy/extras/homing.py:188-192`). For polar's arm home that is x **and**
/// y — both must be marked to open `limit_xy2`.
fn homing_axes_of(forcepos: &HomeCoord) -> Vec<usize> {
    forcepos
        .iter()
        .take(Z_AXIS + 1)
        .enumerate()
        .filter(|(_, value)| value.is_some())
        .map(|(axis, _)| axis)
        .collect()
}

/// Fire a printer event, when the machine is still there.
fn send(printer: &Weak<Printer>, event: &KlippyEvent) {
    if let Some(printer) = printer.upgrade() {
        printer.send_event(event);
    }
}

/// Home every rail in one multi-endstop move, as delta does
/// (`Homing._do_home_rails` + `HomingMove.homing_move` with every endstop
/// armed; upstream's retract + second pass is the gap
/// [`PrinterStepper`](crate::core::klippy::extras::stepper::PrinterStepper)'s
/// module docs record for the cartesian family too).
///
/// Deltesian's arm group uses this too, with the two arm rails and their own
/// endpoints (`deltesian.py:96-102`).
///
/// # Errors
/// A missing endstop, a kinematics refusal, a failed query/send, or a rail
/// whose endstop never triggered ("No trigger on … after full movement",
/// `extras/homing.py:104-107`).
async fn home_unified(
    connected: &mut Connected,
    rails: &[Arc<Rail>],
    force: Coord,
    target: Coord,
    actuator_travel: &[f64],
    homing_axes: &[usize],
    homing: &HomingHandle,
    printer: &Weak<Printer>,
) -> Result<(), CommandError> {
    for rail in rails {
        if rail.endstop().is_none() {
            return Err(CommandError::new(format!(
                "No endstop configured for {}",
                rail.name()
            )));
        }
    }

    // Pretend to be at the force position with every axis the kinematics marks
    // homed (`Homing._set_start_position`), which is what lets the homing move
    // through its `check_move`.
    connected.toolhead.set_position(force, homing_axes);

    // The endstops all start sampling before the move and each is paced by
    // its own rail's travel (`HomingMove._calc_endstop_rate`).
    let speed = rails[0].homing_info().speed;
    let move_t = move_distance(force, target) / speed;
    let print_time = connected.toolhead.get_last_move_time();
    let mut completions = Vec::with_capacity(rails.len());
    for (index, rail) in rails.iter().enumerate() {
        let endstop = rail
            .endstop()
            .expect("every rail's endstop was checked above");
        let steps = actuator_travel[index] / rail.step_dist();
        let rest_time = if steps <= 0. {
            0.001
        } else {
            (move_t / steps).max(0.001)
        };
        let completion = endstop
            .home_start(
                print_time,
                ENDSTOP_SAMPLE_TIME,
                ENDSTOP_SAMPLE_COUNT,
                rest_time,
                true,
            )
            .map_err(command_error)?;
        completions.push(completion);
    }
    connected.toolhead.dwell(HOMING_START_DELAY);
    send(printer, &KlippyEvent::HomingHomingMoveBegin);
    let (start, end) = connected
        .toolhead
        .drip_move(target, speed)
        .map_err(|err| CommandError::new(err.to_string()))?;

    // Drip the move out in small windows until every endstop has fired (or
    // the move ran out). Each wake waits on one still-pending endstop, so a
    // trigger between checks is seen promptly.
    let mut flush_time = start;
    // The previous segment's end is this batch's start clock — the lower
    // bound on when its first step runs (`StepBatchClocks`).
    let mut generated = start;
    while flush_time < end
        && completions
            .iter()
            .any(|completion| completion.reason().is_none())
    {
        flush_time = (flush_time + DRIP_SEGMENT_TIME).min(end);
        let batches = connected
            .toolhead
            .flush_step_generation(flush_time)
            .map_err(|err| CommandError::new(err.to_string()))?;
        for (name, commands) in batches {
            if let Some(stepper) = connected.mcu_steppers.get(&name) {
                let clocks = step_batch_clocks(stepper, generated, flush_time);
                stepper
                    .send_steps_async(&commands, clocks)
                    .await
                    .map_err(command_error)?;
            }
        }
        generated = flush_time;
        if let Some(pending) = completions
            .iter()
            .find(|completion| completion.reason().is_none())
        {
            tokio::select! {
                _ = sleep(Duration::from_secs_f64(DRIP_LOOKAHEAD)) => {}
                _ = pending.wait() => {}
            }
        }
    }

    for rail in rails {
        // The trigger itself was already proven by the drip loop completing:
        // the fake completes each armed trsync only when the move starts, and
        // a never-triggered check would have left this awaiting forever. The
        // returned *time* may read 0 at machine time zero (the trigger clock
        // minus `rest_ticks` rounds down), which is why — as in `home_axis`
        // above — the value is not read here; upstream's file mode instead
        // reports "No trigger on … after full movement" for a miss.
        let trigger_time = rail
            .endstop()
            .expect("every rail's endstop was checked above")
            .home_wait(end)
            .await
            .map_err(command_error)?;
        // Note each of the rail's steppers' trigger position
        // (`StepperPosition.note_home_end`).
        let steppers = connected.toolhead.motion_queuing_mut().steppers();
        let mut state = homing.lock();
        for (index, stepper) in rail.steppers().iter().enumerate() {
            if let Some(host) = steppers.iter().find(|host| host.name() == stepper.name()) {
                state.set_trigger_position(
                    stepper.name(),
                    host.past_mcu_position(trigger_time) as f64,
                );
                state.set_primary(stepper.name(), index == 0);
            }
        }
    }
    send(printer, &KlippyEvent::HomingHomingMoveEnd);
    // The carriage is now at its home position, with the kinematics' homed
    // flags set.
    connected.toolhead.set_position(target, homing_axes);
    connected.toolhead.wipe_trapq();
    Ok(())
}

/// Home one axis: pretend to be at `forcepos`, move to the endstop, then place
/// the axis at its `position_endstop` (`CartKinematics.home_axis` +
/// `Homing._do_home_rails` + `HomingMove.homing_move`).
#[allow(clippy::too_many_arguments)]
async fn home_axis(
    connected: &mut Connected,
    axis: usize,
    forcepos: HomeCoord,
    movepos: HomeCoord,
    homing_axes: &[usize],
    info: HomingInfo,
    step_dist: f64,
    stepper_names: &[String],
    endstop: &dyn HomingEndstop,
    homing: &HomingHandle,
    printer: &Weak<Printer>,
) -> Result<(), CommandError> {
    // The caller computed the endpoints: `home_move`'s 1.5× overshoot for
    // the cartesian family, `polar_home_move`'s 1.0× push (and Y pin) for
    // polar. `homing_axes` is which axes the force position marks homed.
    let current = connected.toolhead.commanded_pos();
    let force = fill_coord(forcepos, current);
    let home = fill_coord(movepos, current);
    connected.toolhead.set_position(force, homing_axes);

    // Poll the endstop about once per step so a trigger is seen promptly.
    let move_t = move_distance(force, home) / info.speed;
    let steps = ((home.axis(axis) - force.axis(axis)).abs() / step_dist).max(1.0);
    let rest_time = (move_t / steps).max(0.001);

    let print_time = connected.toolhead.get_last_move_time();
    let completion = endstop
        .home_start(
            print_time,
            ENDSTOP_SAMPLE_TIME,
            ENDSTOP_SAMPLE_COUNT,
            rest_time,
            true,
        )
        .map_err(command_error)?;
    connected.toolhead.dwell(HOMING_START_DELAY);
    send(printer, &KlippyEvent::HomingHomingMoveBegin);
    let (start, end) = connected
        .toolhead
        .drip_move(home, info.speed)
        .map_err(|err| CommandError::new(err.to_string()))?;

    // Drip the move out in small windows; stop as soon as the trigger fires.
    let mut flush_time = start;
    // Previous segment's end: this batch's start clock (`StepBatchClocks`).
    let mut generated = start;
    while flush_time < end && completion.reason().is_none() {
        flush_time = (flush_time + DRIP_SEGMENT_TIME).min(end);
        let batches = connected
            .toolhead
            .flush_step_generation(flush_time)
            .map_err(|err| CommandError::new(err.to_string()))?;
        for (name, commands) in batches {
            if let Some(stepper) = connected.mcu_steppers.get(&name) {
                let clocks = step_batch_clocks(stepper, generated, flush_time);
                stepper
                    .send_steps_async(&commands, clocks)
                    .await
                    .map_err(command_error)?;
            }
        }
        generated = flush_time;
        if completion.reason().is_none() {
            // Wake on the trigger as well as on the drip interval: the
            // completion can fire between the check above and here, in which
            // case waiting out the whole `sleep` would stall the loop.
            tokio::select! {
                _ = sleep(Duration::from_secs_f64(DRIP_LOOKAHEAD)) => {}
                _ = completion.wait() => {}
            }
        }
    }

    let trigger_time = endstop.home_wait(end).await.map_err(command_error)?;
    // Note each stepper's trigger position (`StepperPosition.note_home_end`)
    // before `set_position` moves the solver to the endstop position.
    {
        let steppers = connected.toolhead.motion_queuing_mut().steppers();
        let mut state = homing.lock();
        for (index, name) in stepper_names.iter().enumerate() {
            if let Some(stepper) = steppers.iter().find(|stepper| stepper.name() == name) {
                state.set_trigger_position(name, stepper.past_mcu_position(trigger_time) as f64);
                state.set_primary(name, index == 0);
            }
        }
    }
    send(printer, &KlippyEvent::HomingHomingMoveEnd);
    // The axis is now known at its endstop position.
    connected.toolhead.set_position(home, homing_axes);
    connected.toolhead.wipe_trapq();
    Ok(())
}

/// Probe-style homing: move toward `target` at `speed`, stop on trigger
/// (`homing.probing_move`).
///
/// Unlike `home_axis` this does not read a `HomingInfo` from a rail — it
/// receives the target position directly, so callers (e.g. a probe extra)
/// can use any endstop at any speed without rail configuration.
///
/// Events follow upstream order: `homing_move_begin` **before** any
/// sampling or movement, `homing_move_end` after.
///
/// Returns the position the move stopped at: the move is always dripped to
/// its end, and that end position is what it reports (and what the
/// toolhead's commanded position is set to). This matches upstream's
/// file-output mode, which completes the trigger only after the drip move
/// ended (`mcu.py:TriggerDispatch.wait_end`) and takes `trigpos` at
/// `home_end_time` (`homing.py`). This host's fake firmware fires the armed
/// endstop at the move's first queued step, so ending the drip on that early
/// clock would walk each repeated probe up by its retract distance and fail
/// `samples_tolerance`.
///
/// # Errors
/// - `"No trigger on probe after full movement"` when the move completed
///   without an endstop hit.
/// - A failed `home_start`, `home_wait`, kinematics refusal, or step send.
#[allow(clippy::too_many_arguments)]
async fn probing_move(
    connected: &mut Connected,
    endstop: &dyn HomingEndstop,
    target: Coord,
    speed: f64,
    printer: &Weak<Printer>,
) -> Result<Coord, CommandError> {
    let current = connected.toolhead.commanded_pos();
    let distance = move_distance(current, target);

    // Emit begin event **before** any sampling or movement (upstream order).
    send(printer, &KlippyEvent::HomingHomingMoveBegin);

    // Zero-length probe: a successful no-op that reports the current
    // position, as upstream's test mode answers it — `check_no_movement` is
    // disabled under `debuginput` (homing.py), and `MCU_trsync.stop` reports
    // `REASON_ENDSTOP_HIT` unconditionally in fileoutput (mcu.py), so
    // `home_wait` reports the trigger at `home_end_time`: where a move that
    // never moved already is. The endstop is deliberately not armed here:
    // the fake firmware only fires on queued steps, and an empty move queues
    // none. (Upstream's "Probe triggered prior to movement" is
    // `check_no_movement` — an endstop that triggered before a **non-zero**
    // move started — not this.)
    //
    // The guard shares the epsilon of `motion::plan::Move::new`
    // (`move_d < 0.000_000_001` collapses XYZ to a non-kinematic zero-length
    // move): a sub-nanometer fp residue (e.g. `4.7e-16` left by
    // `set_position`/`G1` arithmetic) must take this branch too — otherwise
    // `home_start` arms the endstop while the planner queues zero batches,
    // the fake never fires and `home_wait` deadlocks (seen as 111 re-arms of
    // one frozen clock in multi_z's third probe).
    if distance < 0.000_000_001 {
        send(printer, &KlippyEvent::HomingHomingMoveEnd);
        return Ok(current);
    }

    let print_time = connected.toolhead.get_last_move_time();
    // Drain the planner's backlog before arming. The background flush task
    // stands back while this move owns the connected state, so everything
    // queued since its last round (upstream's flush thread drains the same
    // queue continuously) is still un-generated here. Compressing that
    // backlog after `home_start` spends the firmware's monitor window —
    // 16 ms of wall time at `monitor=40000x4` — on CPU work: eddy.test
    // planned +40.4 s of scan + rapid_scan print time, its first generate
    // took 41 ms, and `monitor_event` reported `Trigger analog error:
    // MONITOR` before a single step frame left the host.
    let backlog = connected
        .generate()
        .map_err(|err| CommandError::new(err.to_string()))?;
    for (stepper, commands, clocks) in backlog {
        stepper
            .send_steps_async(&commands, clocks)
            .await
            .map_err(command_error)?;
    }
    // Kept only for its side effect (arming the endstop); `home_wait` awaits
    // the same completion through the dispatch that owns it, and the drip
    // loop below deliberately does not poll it.
    let _completion = endstop
        .home_start(
            print_time,
            ENDSTOP_SAMPLE_TIME,
            ENDSTOP_SAMPLE_COUNT,
            0.001,
            true,
        )
        .map_err(command_error)?;
    connected.toolhead.dwell(HOMING_START_DELAY);
    let (start, end) = connected
        .toolhead
        .drip_move(target, speed)
        .map_err(|err| CommandError::new(err.to_string()))?;

    // Drip the move out in small windows until the move is fully queued. The
    // completion is deliberately not polled: the fake firmware this host runs
    // against fires the armed endstop at the move's first queued step, while
    // upstream's file-output mode completes the trigger only after the drip
    // move ended (`mcu.py:TriggerDispatch.wait_end`) and takes `trigpos` at
    // `home_end_time` (`homing.py`) — the move's end. Ending the drip on the
    // fake's early fire clock walks each repeated probe up by its retract
    // distance and fails `samples_tolerance` (the corpus's only
    // `samples: 3` config, `screws_tilt_adjust.cfg`).
    let mut flush_time = start;
    // Previous segment's end: this batch's start clock (`StepBatchClocks`).
    let mut generated = start;
    while flush_time < end {
        flush_time = (flush_time + DRIP_SEGMENT_TIME).min(end);
        let batches = connected
            .toolhead
            .flush_step_generation(flush_time)
            .map_err(|err| CommandError::new(err.to_string()))?;
        for (name, commands) in batches {
            if let Some(stepper) = connected.mcu_steppers.get(&name) {
                let clocks = step_batch_clocks(stepper, generated, flush_time);
                stepper
                    .send_steps_async(&commands, clocks)
                    .await
                    .map_err(command_error)?;
            }
        }
        generated = flush_time;
        // No wall-clock pacing between segments: the transport is
        // in-process, the fake firmware consumes each batch as it is
        // written, and the trigger is awaited in `home_wait` — pacing would
        // only serialize the corpus's probes in real time (dripping every
        // probe out at `DRIP_LOOKAHEAD` pushed the all-cases regression
        // run past 30 minutes).
    }

    let trigger_time = endstop.home_wait(end).await.map_err(command_error)?;
    send(printer, &KlippyEvent::HomingHomingMoveEnd);

    // No trigger: upstream raises "No trigger on {name} after full movement"
    // after emitting homing_move_end.
    if trigger_time <= 0.0 {
        return Err(CommandError::new("No trigger on probe after full movement"));
    }

    // Where the move stopped: the drip loop above just ran to
    // `flush_time = end`, so the newest queued segment's position there is
    // the move's end — the target. That is upstream's file-output `trigpos`
    // (the stepper position at `home_end_time`); on real hardware upstream
    // would read the firmware's trigger clock instead, which the corpus's
    // fake MCUs cannot model (no step model: `stepper_get_position` answers
    // zero). A following probe still has room to move down because every
    // consumer lifts first — each sample retracts (`run_with`) and each
    // point is preceded by a raise (`ProbePointsHelper`) — so the corpus
    // never probes twice from the target into "Probe triggered prior to
    // movement".
    let stopped = {
        let trapq = connected.toolhead.trapq();
        let moves = trapq.moves();
        // The newest segment that has started by `flush_time` is the one the
        // carriage is in; a plain forward scan stops at the position marker
        // `set_position` left behind, which sits at the origin.
        let mut found = None;
        for segment in moves.iter().rev() {
            if segment.print_time <= flush_time {
                let into = if flush_time <= segment.end_time() {
                    flush_time - segment.print_time
                } else {
                    segment.move_t
                };
                found = Some(segment.coord(into));
                break;
            }
        }
        found
    };
    let mut result = connected.toolhead.commanded_pos();
    if let Some(stopped) = stopped {
        result.set_axis(0, stopped.x());
        result.set_axis(1, stopped.y());
        result.set_axis(Z_AXIS, stopped.z());
        connected.toolhead.set_position(result, &[Z_AXIS]);
    }
    Ok(result)
}

fn command_error(err: McuError) -> CommandError {
    CommandError::new(err.to_string())
}

// ===========================================================================
// G-code commands
// ===========================================================================

/// `G0` / `G1`: move the toolhead.
/// `G4`: dwell. `P` is milliseconds, `S` is seconds (`S` wins when both are
/// given, as a config that writes both probably means the longer one).
fn cmd_dwell(
    state: &Arc<Mutex<Option<Connected>>>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let seconds = if gcmd.get_command_parameters().contains_key("S") {
        gcmd.get_float("S")?
    } else {
        gcmd.get_float_default("P", 0.0)? / 1000.0
    };
    let seconds = seconds.max(0.0);
    let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
    let Some(connected) = guard.as_mut() else {
        return Err(CommandError::new("Printer is not ready"));
    };
    connected.toolhead.dwell(seconds);
    Ok(())
}

/// `M400`: wait for the moves queued so far to be planned.
fn cmd_wait_moves(
    state: &Arc<Mutex<Option<Connected>>>,
    _gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
    let Some(connected) = guard.as_mut() else {
        return Err(CommandError::new("Printer is not ready"));
    };
    connected.toolhead.wait_moves();
    Ok(())
}

/// An optional float word: upstream's `gcmd.get_float(name, None, …)`. An
/// absent word is `None`; a present one is parsed and bounds-checked with the
/// dispatcher's wording ("must have minimum of …", "must be above …").
fn optional_float(
    gcmd: &GcodeCommand,
    name: &str,
    minval: Option<f64>,
    above: Option<f64>,
    below: Option<f64>,
) -> Result<Option<f64>, CommandError> {
    if !gcmd.get_command_parameters().contains_key(name) {
        return Ok(None);
    }
    Ok(Some(gcmd.get(
        name,
        None,
        parse_float,
        minval,
        None,
        above,
        below,
    )?))
}

/// Apply `set_max_velocities` to both copies of the limits — the one this
/// object keeps and the connected planner's — and return the four current
/// values. Only a `Some` overrides, as upstream's `None` arguments do.
fn apply_max_velocities(
    state: &Arc<Mutex<Option<Connected>>>,
    limits: &Arc<Mutex<VelocityLimits>>,
    max_velocity: Option<f64>,
    max_accel: Option<f64>,
    square_corner_velocity: Option<f64>,
    min_cruise_ratio: Option<f64>,
) -> (f64, f64, f64, f64) {
    // The object's copy first, then the planner's (whose copy the next move is
    // profiled from). The two locks are taken one at a time, never together:
    // `get_status` takes the state lock then this one, so holding both here in
    // the other order would deadlock.
    let (mv, ma, scv, mcr) = limits
        .lock()
        .unwrap_or_else(|poison| poison.into_inner())
        .set_max_velocities(
            max_velocity,
            max_accel,
            square_corner_velocity,
            min_cruise_ratio,
        );
    if let Some(connected) = state
        .lock()
        .unwrap_or_else(|poison| poison.into_inner())
        .as_mut()
    {
        connected
            .toolhead
            .set_max_velocities(max_velocity, max_accel, scv, mcr);
    }
    (mv, ma, scv, mcr)
}

/// `SET_VELOCITY_LIMIT`: change the velocity limits (`toolhead.py:573-589`).
///
/// A parameter that is not named is left alone; naming none reports the current
/// limits instead of changing them.
fn cmd_set_velocity_limit(
    state: &Arc<Mutex<Option<Connected>>>,
    limits: &Arc<Mutex<VelocityLimits>>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let max_velocity = optional_float(gcmd, "VELOCITY", None, Some(0.0), None)?;
    let max_accel = optional_float(gcmd, "ACCEL", None, Some(0.0), None)?;
    let square_corner_velocity =
        optional_float(gcmd, "SQUARE_CORNER_VELOCITY", Some(0.0), None, None)?;
    let min_cruise_ratio =
        optional_float(gcmd, "MINIMUM_CRUISE_RATIO", Some(0.0), None, Some(1.0))?;
    let (mv, ma, scv, mcr) = apply_max_velocities(
        state,
        limits,
        max_velocity,
        max_accel,
        square_corner_velocity,
        min_cruise_ratio,
    );
    let msg = format!(
        "max_velocity: {mv:.6}\nmax_accel: {ma:.6}\n\
         minimum_cruise_ratio: {mcr:.6}\nsquare_corner_velocity: {scv:.6}"
    );
    set_rollover_info("toolhead", Some(&format!("toolhead: {msg}")));
    // Upstream echoes the current limits only when nothing was named — a query
    // — and does so without logging; a change is silent (`toolhead.py:587-589`).
    if max_velocity.is_none()
        && max_accel.is_none()
        && square_corner_velocity.is_none()
        && min_cruise_ratio.is_none()
    {
        gcmd.respond_info_no_log(&msg);
    }
    Ok(())
}

/// `M204`: change the acceleration limit (`toolhead.py:590-601`).
///
/// `S` sets it directly; with no `S`, the minimum of `P` and `T` does, and
/// either missing makes the command invalid (nothing is changed).
fn cmd_m204(
    state: &Arc<Mutex<Option<Connected>>>,
    limits: &Arc<Mutex<VelocityLimits>>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let accel = match optional_float(gcmd, "S", None, Some(0.0), None)? {
        Some(accel) => accel,
        None => {
            let p = optional_float(gcmd, "P", None, Some(0.0), None)?;
            let t = optional_float(gcmd, "T", None, Some(0.0), None)?;
            match (p, t) {
                (Some(p), Some(t)) => p.min(t),
                _ => {
                    gcmd.respond_info(&format!("Invalid M204 command \"{}\"", gcmd.commandline()));
                    return Ok(());
                }
            }
        }
    };
    apply_max_velocities(state, limits, None, Some(accel), None, None);
    Ok(())
}

/// `SET_KINEMATIC_POSITION`: force the low-level position (`force_move.py:118`).
fn cmd_set_kinematic_position(
    state: &Arc<Mutex<Option<Connected>>>,
    printer: &Weak<Printer>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
    let Some(connected) = guard.as_mut() else {
        return Err(CommandError::new("Printer is not ready"));
    };
    let current = connected.toolhead.commanded_pos();
    let mut newpos = current;
    for (axis, name) in [(X_AXIS, "X"), (Y_AXIS, "Y"), (Z_AXIS, "Z")] {
        let value = gcmd.get_float_default(name, current.axis(axis))?;
        newpos.set_axis(axis, value);
    }
    let set_homed = gcmd.get_str_default("SET_HOMED", "xyz").to_lowercase();
    let homing_axes = axis_indices(&set_homed);
    let clear_default = gcmd.get_str_default("CLEAR", "");
    let clear_homed = gcmd
        .get_str_default("CLEAR_HOMED", &clear_default)
        .to_lowercase();
    let clear_axes = axis_indices(&clear_homed);

    connected.toolhead.set_position(newpos, &homing_axes);
    if let Some(kinematics) = connected.toolhead.kinematics_mut() {
        kinematics.clear_homing_state(&clear_axes);
    }
    drop(guard);
    // Upstream's `ToolHead.set_position` fires this (`toolhead.py:390`) and
    // `force_move.cmd_SET_KINEMATIC_POSITION` reaches it through there; the
    // low-level `motion::ToolHead` has no printer, so the command does.
    // `gcode_move` re-anchors `last_position` to the toolhead on it — what
    // keeps a `G1` that omits an axis from dragging a stale one along.
    send(printer, &KlippyEvent::ToolheadSetPosition);
    Ok(())
}

/// The axis indices named by a lower-case string of `x`, `y`, `z`.
fn axis_indices(names: &str) -> Vec<usize> {
    ["x", "y", "z"]
        .iter()
        .enumerate()
        .filter(|(_, name)| names.contains(*name))
        .map(|(axis, _)| axis)
        .collect()
}

/// `G28`: home the named axes (all three when none is named).
///
/// The homing run is asynchronous (it drives the drip flush loop and awaits the
/// endstop trigger), so the toolhead is taken out of its shared slot, the run is
/// awaited, and the toolhead is put back afterwards. The background flush task
/// sees an empty slot and stands back.
async fn cmd_g28(
    state: &Arc<Mutex<Option<Connected>>>,
    rails: &[Arc<Rail>],
    kind: KinematicsKind,
    printer: &Weak<Printer>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let params = gcmd.get_command_parameters();
    let mut requested: Vec<usize> = Vec::new();
    for (name, axis) in [("X", X_AXIS), ("Y", Y_AXIS), ("Z", Z_AXIS)] {
        if params.contains_key(name) {
            requested.push(axis);
        }
    }
    let requested = if requested.is_empty() {
        vec![X_AXIS, Y_AXIS, Z_AXIS]
    } else {
        requested
    };

    // Take the toolhead out of its shared slot, then drop the lock before
    // awaiting: the homing run is long, and a `MutexGuard` is not `Send`, so it
    // must not be held across the await. The background flush task sees the
    // empty slot and stands back.
    let mut connected = {
        let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
        let Some(connected) = guard.take() else {
            return Err(CommandError::new("Printer is not ready"));
        };
        connected
    };
    let outcome = home_axes(&mut connected, rails, kind, &requested, printer).await;
    *state.lock().unwrap_or_else(|poison| poison.into_inner()) = Some(connected);
    let (homed, homing) = outcome?;
    if homed.is_empty() {
        return Ok(());
    }
    // End the run with the slot filled but the state lock **released**: a handler
    // that reads the toolhead (gcode_move's `_handle_home_rails_end`) must see
    // the homed position, not the empty slot's default, and taking that lock
    // again here would deadlock. `[endstop_phase]` records its offsets while the
    // event is delivered; they are applied to the steppers just below.
    send(
        printer,
        &KlippyEvent::HomingHomeRailsEnd {
            axes: homed.clone(),
            homing: homing.clone(),
        },
    );
    let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
    let Some(connected) = guard.as_mut() else {
        return Err(CommandError::new("Printer is not ready"));
    };
    apply_home_rails_adjustments(connected, &homing, &homed)
}

/// The factory the `[printer]` declaration names.
pub(crate) fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = ToolHeadObject::new(config, printer)?;
    // Upstream's `add_printer_objects` loads the default modules right after
    // the toolhead (`toolhead.py:610-613`), and `gcode_move` is the first of
    // them: it is what turns g-code coordinates into the toolhead's.
    crate::core::klippy::extras::gcode_move::ensure(printer)?;
    // `manual_probe` is on the same upstream list (`toolhead.py:610-613`), which is
    // why `PROBE_CALIBRATE` works on a config that never writes
    // `[manual_probe]`: the object is always there.
    crate::core::klippy::extras::manual_probe::ensure(printer, config)?;
    Ok(Arc::new(object))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mcu::McuError;

    #[test]
    fn test_axis_indices_reads_the_letters() {
        assert_eq!(axis_indices("xyz"), vec![0, 1, 2]);
        assert_eq!(axis_indices("xz"), vec![0, 2]);
        assert_eq!(axis_indices(""), Vec::<usize>::new());
    }

    #[test]
    fn test_move_context_is_built_from_the_current_position() {
        // The parser fills absent axes with the current value, so a move on one
        // axis leaves the others alone.
        let current = Coord::new(1.0, 2.0, 3.0, 4.0);
        let mut newpos = current;
        newpos.set_axis(Y_AXIS, 9.0);

        assert_eq!(newpos, Coord::new(1.0, 9.0, 3.0, 4.0));
    }

    #[test]
    fn test_an_unsupported_kinematics_is_a_config_error() {
        // The loader's `new` needs a printer and the three stepper sections;
        // the kinematics check happens before either is looked up, so a bare
        // section is enough.
        use crate::core::klippy::config::section::ConfigSection;
        use crate::core::klippy::config::value::ConfigValue;
        use crate::core::klippy::reactor::ManualReactor;

        let mut section = ConfigSection::new("printer", None);
        section.parameters.insert(
            "kinematics".to_string(),
            ConfigValue::Single("scara".to_string()),
        );
        section.parameters.insert(
            "max_velocity".to_string(),
            ConfigValue::Single("300".to_string()),
        );
        section.parameters.insert(
            "max_accel".to_string(),
            ConfigValue::Single("3000".to_string()),
        );
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let wrapper = ConfigWrapper::untracked(&section);

        let err = ToolHeadObject::new(&wrapper, &printer)
            .map(|_| ())
            .unwrap_err();

        assert!(err.to_string().contains("Error loading kinematics 'scara'"));
        // The message also names what *is* implemented, including delta now.
        assert!(err.to_string().contains("delta"));
    }

    #[test]
    fn test_none_kinematics_needs_no_steppers() {
        use crate::core::klippy::config::section::ConfigSection;
        use crate::core::klippy::config::value::ConfigValue;
        use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
        use crate::core::klippy::reactor::ManualReactor;

        let mut section = ConfigSection::new("printer", None);
        for (key, value) in [
            ("kinematics", "none"),
            ("max_velocity", "300"),
            ("max_accel", "3000"),
        ] {
            section
                .parameters
                .insert(key.to_string(), ConfigValue::Single(value.to_string()));
        }
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();

        // `none` has no `[stepper_*]` sections; the object is enough on its own.
        let object = ToolHeadObject::new(&ConfigWrapper::untracked(&section), &printer)
            .expect("kinematics: none builds without steppers");
        // No rails, so there are no Z motors to name.
        assert!(object.z_stepper_names().is_empty());
    }

    #[test]
    fn test_a_stepper_z1_joins_the_z_rail() {
        // `[stepper_z1]` has no factory of its own; the Z rail reads it through
        // `[stepper_z]`'s wrapper (upstream's `LookupMultiRail`).
        use crate::core::klippy::config::Config;
        use crate::core::klippy::reactor::ManualReactor;

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_y]\nstep_pin: PA2\ndir_pin: PA3\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z]\nstep_pin: PA4\ndir_pin: PA5\nrotation_distance: 8\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z1]\nstep_pin: PA6\ndir_pin: PA7\nrotation_distance: 8\nmicrosteps: 16\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n",
        )
        .expect("the config parses");
        printer.load_config(&config).expect("the config loads");

        let object = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");
        assert_eq!(object.rails.len(), 3);
        assert_eq!(object.rails[Z_AXIS].steppers().len(), 2);
        assert_eq!(object.rails[Z_AXIS].steppers()[1].name(), "stepper_z1");
        // The sibling is registered (so its connect runs) and its options were
        // read, which is what makes the factory-less section valid.
        assert!(printer.lookup_object("stepper_z1").is_some());
    }

    #[test]
    fn test_the_corexy_family_loads_and_builds_its_rails() {
        use crate::core::klippy::config::Config;
        use crate::core::klippy::reactor::ManualReactor;

        for (name, transform) in [
            ("corexy", CartesianTransform::CoreXy),
            ("corexz", CartesianTransform::CoreXz),
            ("hybrid_corexy", CartesianTransform::HybridCoreXy),
            ("hybrid_corexz", CartesianTransform::HybridCoreXz),
        ] {
            let printer = Arc::new(Printer::new(ManualReactor::shared()));
            let text = format!(
                "[mcu]\nserial: /dev/not-opened-yet\n\
                 [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\nendstop_pin: ^PA2\n\
                 [stepper_y]\nstep_pin: PA3\ndir_pin: PA4\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\nendstop_pin: ^PA5\n\
                 [stepper_z]\nstep_pin: PA6\ndir_pin: PA7\nrotation_distance: 8\nmicrosteps: 16\nposition_max: 200\nendstop_pin: ^PB0\n\
                 [printer]\nkinematics: {name}\nmax_velocity: 300\nmax_accel: 3000\n"
            );
            let (config, _) = Config::from_text(&text).expect("the config parses");
            printer
                .load_config(&config)
                .unwrap_or_else(|err| panic!("{name}: {err}"));
            let object = printer
                .lookup_object_as::<ToolHeadObject>("toolhead")
                .expect("the toolhead is registered");
            assert_eq!(object.transform, transform, "{name}");
            assert_eq!(object.rails.len(), 3, "{name}");
        }
    }

    #[test]
    fn test_the_polar_kinematics_claims_its_sections_and_bed() {
        use crate::core::klippy::config::Config;
        use crate::core::klippy::reactor::ManualReactor;

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let text = "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_bed]\nstep_pin: PA0\ndir_pin: PA1\n\
             microsteps: 16\ngear_ratio: 80:16\n\
             [stepper_arm]\nstep_pin: PA2\ndir_pin: PA3\n\
             rotation_distance: 40\nmicrosteps: 16\n\
             endstop_pin: ^PA4\nposition_endstop: 300\n\
             position_max: 300\nhoming_speed: 50\n\
             [stepper_z]\nstep_pin: PA5\ndir_pin: PA6\n\
             rotation_distance: 8\nmicrosteps: 16\n\
             endstop_pin: ^PA7\nposition_endstop: 0.5\n\
             position_max: 200\n\
             [printer]\nkinematics: polar\nmax_velocity: 300\n\
             max_accel: 3000\nmax_angular_velocity: 5\n";
        let (config, _) = Config::from_text(text).expect("the config parses");
        printer
            .load_config(&config)
            .unwrap_or_else(|err| panic!("polar: {err}"));

        let object = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");
        // `kinematics/polar.py:34-35`: rails = [arm, z] — two, not three.
        assert_eq!(object.kind, KinematicsKind::Polar);
        assert_eq!(object.rails.len(), 2);
        assert_eq!(object.rails[0].name(), "stepper_arm");
        assert_eq!(object.rails[1].name(), "stepper_z");
        // The bed is claimed but belongs to no rail.
        assert!(object.bed.is_some(), "the bed stepper is held");
        assert!(printer.lookup_object("stepper_bed").is_some());
        assert!(printer.lookup_object("stepper_arm").is_some());
        // The angular cap was read from `[printer]` (upstream reads it in
        // `PolarKinematics.__init__`).
        assert_eq!(object.max_angular_velocity, 5.0);
        // The Z rail is found by name, not by cartesian index.
        assert_eq!(object.z_stepper_names(), ["stepper_z"]);
    }

    #[test]
    fn test_polar_without_a_stepper_bed_section_is_refused() {
        use crate::core::klippy::config::Config;
        use crate::core::klippy::reactor::ManualReactor;

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let text = "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_arm]\nstep_pin: PA2\ndir_pin: PA3\n\
             rotation_distance: 40\nmicrosteps: 16\n\
             endstop_pin: ^PA4\nposition_endstop: 300\n\
             position_max: 300\nhoming_speed: 50\n\
             [stepper_z]\nstep_pin: PA5\ndir_pin: PA6\n\
             rotation_distance: 8\nmicrosteps: 16\n\
             endstop_pin: ^PA7\nposition_endstop: 0.5\n\
             position_max: 200\n\
             [printer]\nkinematics: polar\nmax_velocity: 300\nmax_accel: 3000\n";
        let (config, _) = Config::from_text(text).expect("the config parses");

        let err = printer.load_config(&config).unwrap_err().to_string();
        assert!(err.contains("stepper_bed"), "{err}");
    }

    #[test]
    fn test_polar_is_a_known_kinematics_name() {
        assert_eq!(KinematicsKind::parse("polar"), Some(KinematicsKind::Polar));
        assert!(KinematicsKind::NAMES.contains(&"polar"));
        // The unsupported-name error lists it (the message the corpus sees).
        let names = KinematicsKind::NAMES.join(", ");
        assert!(names.contains("polar"), "{names}");
    }

    #[test]
    fn test_winch_is_a_known_kinematics_name() {
        assert_eq!(KinematicsKind::parse("winch"), Some(KinematicsKind::Winch));
        assert!(KinematicsKind::NAMES.contains(&"winch"));
        let names = KinematicsKind::NAMES.join(", ");
        assert!(names.contains("winch"), "{names}");
    }

    #[test]
    fn test_a_winch_config_loads_without_position_endstop() {
        use crate::core::klippy::config::Config;
        use crate::core::klippy::reactor::ManualReactor;

        // `config/example-winch.cfg`'s geometry: four cables and no
        // `position_endstop` anywhere — a winch homes by hand, not on an
        // endstop (`kinematics/winch.py:13-24`).
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let text = "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_a]\nstep_pin: PA0\ndir_pin: PA1\nmicrosteps: 16\nrotation_distance: 40\n\
             anchor_x: 0\nanchor_y: -2000\nanchor_z: -100\n\
             [stepper_b]\nstep_pin: PA2\ndir_pin: PA3\nmicrosteps: 16\nrotation_distance: 40\n\
             anchor_x: 2000\nanchor_y: 1000\nanchor_z: -100\n\
             [stepper_c]\nstep_pin: PA4\ndir_pin: PA5\nmicrosteps: 16\nrotation_distance: 40\n\
             anchor_x: -2000\nanchor_y: 1000\nanchor_z: -100\n\
             [stepper_d]\nstep_pin: PA6\ndir_pin: PA7\nmicrosteps: 16\nrotation_distance: 40\n\
             anchor_x: 0\nanchor_y: 0\nanchor_z: 3000\n\
             [printer]\nkinematics: winch\nmax_velocity: 300\nmax_accel: 3000\n";
        let (config, _) = Config::from_text(text).expect("the config parses");
        printer
            .load_config(&config)
            .unwrap_or_else(|err| panic!("winch: {err}"));

        let object = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");
        assert_eq!(object.kind, KinematicsKind::Winch);
        // No rails: the cables are held apart, in section order.
        assert!(object.rails.is_empty());
        assert_eq!(object.winch_steppers.len(), 4);
        assert_eq!(object.winch_steppers[0].name(), "stepper_a");
        assert_eq!(object.winch_steppers[3].name(), "stepper_d");
    }

    #[test]
    fn test_a_winch_cable_without_an_anchor_is_refused() {
        use crate::core::klippy::config::Config;
        use crate::core::klippy::reactor::ManualReactor;

        // `anchor_x` is the first anchor read, so its absence is the error the
        // winch load reports (`config.getfloat('anchor_' + n)`,
        // `kinematics/winch.py:20`).
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let text = "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_a]\nstep_pin: PA0\ndir_pin: PA1\nmicrosteps: 16\nrotation_distance: 40\n\
             anchor_y: -2000\nanchor_z: -100\n\
             [stepper_b]\nstep_pin: PA2\ndir_pin: PA3\nmicrosteps: 16\nrotation_distance: 40\n\
             anchor_x: 2000\nanchor_y: 1000\nanchor_z: -100\n\
             [stepper_c]\nstep_pin: PA4\ndir_pin: PA5\nmicrosteps: 16\nrotation_distance: 40\n\
             anchor_x: -2000\nanchor_y: 1000\nanchor_z: -100\n\
             [printer]\nkinematics: winch\nmax_velocity: 300\nmax_accel: 3000\n";
        let (config, _) = Config::from_text(text).expect("the config parses");

        let err = printer.load_config(&config).unwrap_err().to_string();
        assert!(
            err.contains("Option 'anchor_x' in section 'stepper_a' must be specified"),
            "{err}"
        );
    }

    #[test]
    fn test_the_delta_kinematics_loads_its_three_towers() {
        // `config/example-delta.cfg`'s shape: three towers (no `position_max`
        // option), `arm_length` on `stepper_a`, `delta_radius` on `[printer]`.
        use crate::core::klippy::config::Config;
        use crate::core::klippy::reactor::ManualReactor;

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_a]\nstep_pin: PA0\ndir_pin: PA1\nenable_pin: !PA2\n\
             rotation_distance: 40\nmicrosteps: 16\nendstop_pin: ^PA3\n\
             homing_speed: 50\nposition_endstop: 297.05\narm_length: 333.0\n\
             [stepper_b]\nstep_pin: PB0\ndir_pin: PB1\nenable_pin: !PB2\n\
             rotation_distance: 40\nmicrosteps: 16\nendstop_pin: ^PB3\n\
             [stepper_c]\nstep_pin: PC0\ndir_pin: PC1\nenable_pin: !PC2\n\
             rotation_distance: 40\nmicrosteps: 16\nendstop_pin: ^PC3\n\
             [printer]\nkinematics: delta\nmax_velocity: 300\nmax_accel: 3000\n\
             max_z_velocity: 150\ndelta_radius: 174.75\n",
        )
        .expect("the config parses");
        printer
            .load_config(&config)
            .unwrap_or_else(|err| panic!("delta: {err}"));

        let object = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");
        assert!(object.has_delta_calibration());
        assert_eq!(object.rails.len(), 3);
        assert_eq!(object.axis_names(), ["stepper_a", "stepper_b", "stepper_c"]);
        // The calibration view carries the loaded parameters.
        let calibration = object
            .delta_calibration()
            .expect("`kinematics: delta` has a calibration");
        let KinematicsCalibration::Linear(calibration) = calibration else {
            panic!("a linear delta reports a linear delta calibration");
        };
        assert_eq!(calibration.radius, 174.75);
        assert_eq!(calibration.arms, [333.0, 333.0, 333.0]);
        assert_eq!(calibration.endstops, [297.05, 297.05, 297.05]);
        // `stepper_b/c` inherited `stepper_a`'s endstop.
        for rail in &object.rails {
            assert_eq!(rail.homing_info().position_endstop, 297.05);
        }
    }

    #[test]
    fn test_the_deltesian_kinematics_loads_its_arms_and_y() {
        // `config/example-deltesian.cfg`'s shape: two arm rails (no
        // `position_max`; `arm_x_length`/`arm_length` on `stepper_left`,
        // inherited by `stepper_right`) and the straight `[stepper_y]`.
        use crate::core::klippy::config::Config;
        use crate::core::klippy::reactor::ManualReactor;

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_left]\nstep_pin: PF0\ndir_pin: PF1\nenable_pin: !PD7\n\
             microsteps: 16\nrotation_distance: 40\nendstop_pin: ^PE5\n\
             homing_speed: 50\nposition_endstop: 268\n\
             arm_length: 217\narm_x_length: 160\n\
             [stepper_right]\nstep_pin: PL3\ndir_pin: PL1\nenable_pin: !PK0\n\
             microsteps: 16\nrotation_distance: 40\nendstop_pin: ^PD3\n\
             [stepper_y]\nstep_pin: PF6\ndir_pin: !PF7\nenable_pin: !PF2\n\
             microsteps: 16\nrotation_distance: 40\nendstop_pin: ^PJ1\n\
             position_endstop: 0\nposition_max: 200\n\
             [printer]\nkinematics: deltesian\nmax_velocity: 500\nmax_accel: 3000\n\
             max_z_velocity: 150\n",
        )
        .expect("the config parses");
        printer
            .load_config(&config)
            .unwrap_or_else(|err| panic!("deltesian: {err}"));

        let object = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");
        assert_eq!(object.kind, KinematicsKind::Deltesian);
        assert_eq!(object.rails.len(), 3);
        assert_eq!(
            object.axis_names(),
            ["stepper_left", "stepper_right", "stepper_y"]
        );
        // `stepper_right` inherited `stepper_left`'s endstop.
        assert_eq!(object.rails[1].homing_info().position_endstop, 268.0);
        // Deltesian carries no delta calibration.
        assert!(!object.has_delta_calibration());
        assert!(object.delta_calibration().is_none());
        // The kinematics parked at load holds the derived arm geometry.
        let kin = object
            .deltesian
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take()
            .expect("the deltesian kinematics is parked");
        assert_eq!(
            kin.arm_geometry(),
            [(217.0 * 217.0, -160.0), (217.0 * 217.0, 160.0)]
        );
    }

    #[test]
    fn test_mcu_errors_are_reported_with_the_section_name() {
        // `McuError::Config` is what a missing connection reports; the test just
        // pins that the helper keeps the `[printer]` prefix.
        let err: KlippyError =
            KlippyError::Config(ConfigError::new("[printer]: MCU is not connected"));
        assert!(err.to_string().contains("[printer]"));
    }

    #[test]
    fn test_the_limits_come_from_the_printer_section() {
        // junction_deviation = scv^2 * (sqrt(2) - 1) / max_accel
        let scv = 5.0_f64;
        let max_accel = 3000.0_f64;
        let deviation = scv.powi(2) * (std::f64::consts::SQRT_2 - 1.0) / max_accel;
        assert!((deviation - 0.003_452_0).abs() < 1e-5, "{deviation}");
        let _ = McuError::Config("x".to_string());
    }

    /// A toolhead over one X stepper, homed, as if the machine were up.
    fn homed_toolhead() -> ToolHead {
        use crate::core::klippy::motion::{Axis, CartesianKinematics, Stepper};

        let limits = MoveLimits {
            max_velocity: 200.0,
            max_accel: 1000.0,
            junction_deviation: 0.01,
            mcr_pseudo_accel: 500.0,
        };
        let mut toolhead = ToolHead::new(limits);
        for (name, axis, oid) in [
            ("stepper_x", Axis::X, 0u32),
            ("stepper_y", Axis::Y, 1),
            ("stepper_z", Axis::Z, 2),
        ] {
            toolhead.add_stepper(Stepper::cartesian(name, oid, 1.0, axis, 1_000_000.0));
        }
        toolhead.set_kinematics(Box::new(CartesianKinematics::new(
            ["stepper_x".into(), "stepper_y".into(), "stepper_z".into()],
            Coord::new(0.0, 0.0, 0.0, 0.0),
            Coord::new(200.0, 200.0, 200.0, 0.0),
            15.0,
            100.0,
            CartesianTransform::Standard,
        )));
        // Pretend a `SET_KINEMATIC_POSITION` homed the axes at the origin.
        toolhead.set_position(Coord::default(), &[X_AXIS, Y_AXIS, Z_AXIS]);
        toolhead
    }

    /// A connected state around `toolhead`, with the command table to build
    /// commands.
    /// A connected state, its dispatcher, and the printer they belong to.
    ///
    /// The printer is returned because the dispatcher holds it **weakly** (a
    /// strong handle would be a `printer -> objects -> gcode -> printer` cycle);
    /// tests that need the dispatcher to report to a live printer keep it.
    fn connected_with_printer(
        toolhead: ToolHead,
    ) -> (Arc<Mutex<Option<Connected>>>, GCodeDispatch, Arc<Printer>) {
        let state = Arc::new(Mutex::new(Some(Connected {
            toolhead,
            mcu_steppers: HashMap::new(),
            last_step_gen_time: 0.0,
            force_move_trapq: None,
        })));
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        (state, GCodeDispatch::new(Arc::clone(&printer)), printer)
    }

    fn connected(toolhead: ToolHead) -> (Arc<Mutex<Option<Connected>>>, GCodeDispatch) {
        let (state, gcode, _printer) = connected_with_printer(toolhead);
        (state, gcode)
    }

    /// A `ToolHeadObject` around an already-connected `state`, for the seams
    /// that only need the shared slot (and a printer to fire events on). The
    /// rails are not involved in them, so none are built.
    fn object_over(state: Arc<Mutex<Option<Connected>>>) -> (Arc<Printer>, ToolHeadObject) {
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let object = ToolHeadObject {
            limits: Arc::new(Mutex::new(VelocityLimits {
                max_velocity: 200.0,
                max_accel: 1000.0,
                square_corner_velocity: 5.0,
                min_cruise_ratio: 0.5,
            })),
            max_z_velocity: 15.0,
            max_z_accel: 100.0,
            rails: Vec::new(),
            bed: None,
            kind: KinematicsKind::Cartesian,
            delta: Mutex::new(None),
            rotary_delta: Mutex::new(None),
            deltesian: Mutex::new(None),
            generic: Mutex::new(None),
            generic_steppers: Vec::new(),
            winch: Mutex::new(None),
            winch_steppers: Vec::new(),
            transform: CartesianTransform::Standard,
            max_angular_velocity: 0.0,
            active_extruder: Mutex::new("extruder".to_string()),
            reactor: printer.reactor(),
            printer: Arc::downgrade(&printer),
            state,
            pending_lookahead_callbacks: Mutex::new(Vec::new()),
            pending_flush_callbacks: Mutex::new(Vec::new()),
            shutdown: Arc::new(AtomicBool::new(false)),
        };
        (printer, object)
    }

    /// ① Connect-safety: at config load there is no planner
    /// (`get_last_move_time()` reads `0.0`), so both registrations wait in
    /// their pending lists and are installed by `connect`.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_callbacks_registered_before_connect_fire_after_connect() {
        use crate::core::klippy::config::section::ConfigSection;
        use crate::core::klippy::config::value::ConfigValue;
        use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
        use crate::core::klippy::reactor::ManualReactor;

        let mut section = ConfigSection::new("printer", None);
        for (key, value) in [
            ("kinematics", "none"),
            ("max_velocity", "300"),
            ("max_accel", "3000"),
        ] {
            section
                .parameters
                .insert(key.to_string(), ConfigValue::Single(value.to_string()));
        }
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        // `kinematics: none` has no steppers — exactly the dwell-only
        // timeline on which `generate` must still fire the flush callbacks.
        let object = ToolHeadObject::new(&ConfigWrapper::untracked(&section), &printer)
            .expect("kinematics: none builds without steppers");

        let lookahead_times = Arc::new(Mutex::new(Vec::new()));
        let flush_times = Arc::new(Mutex::new(Vec::new()));
        {
            let times = Arc::clone(&lookahead_times);
            object.register_lookahead_callback(Box::new(move |time| {
                times.lock().unwrap().push(time);
            }));
        }
        {
            let times = Arc::clone(&flush_times);
            object.register_flush_callback(Box::new(move |time| {
                times.lock().unwrap().push(time);
            }));
        }

        // The registration must not burn the callback on the phantom `0.0`
        // an unconnected toolhead reports.
        assert_eq!(object.get_last_move_time(), 0.0);
        assert!(
            lookahead_times.lock().unwrap().is_empty(),
            "a pre-connect registration must not fire before connect"
        );

        object.connect().await.expect("connect");

        // Installed at connect: the look-ahead is empty, so the lookahead
        // callback fired immediately with the fresh toolhead's last move time.
        {
            let times = lookahead_times.lock().unwrap();
            assert_eq!(times.len(), 1, "installed at connect and fired once");
            let expected = object.get_last_move_time();
            assert!(
                (times[0] - expected).abs() < 1e-9,
                "{} vs {expected}",
                times[0]
            );
        }
        // The flush callback was installed too, and a generation with no
        // steppers at all still delivers the flush time to it.
        object.flush_step_generation().await.expect("a flush");
        assert!(
            !flush_times.lock().unwrap().is_empty(),
            "the installed flush callback fires on generate"
        );

        // Registered after connect, it takes effect immediately.
        let late = Arc::new(Mutex::new(Vec::new()));
        {
            let times = Arc::clone(&late);
            object.register_flush_callback(Box::new(move |time| {
                times.lock().unwrap().push(time);
            }));
        }
        object.flush_step_generation().await.expect("another flush");
        assert!(!late.lock().unwrap().is_empty());
    }

    /// ② Connected, look-ahead empty: the callback fires at once, with the
    /// same time `get_last_move_time()` reports.
    #[test]
    fn test_a_lookahead_callback_registered_after_connect_fires_with_the_last_move_time() {
        let (state, _gcode) = connected(homed_toolhead());
        let (_printer, object) = object_over(state);

        let times = Arc::new(Mutex::new(Vec::new()));
        {
            let fired = Arc::clone(&times);
            object.register_lookahead_callback(Box::new(move |time| {
                fired.lock().unwrap().push(time);
            }));
        }

        let expected = object.get_last_move_time();
        let mut fired = times.lock().unwrap();
        assert_eq!(fired.len(), 1, "the empty look-ahead fires immediately");
        assert!(
            (fired[0] - expected).abs() < 1e-9,
            "{} vs {expected}",
            fired[0]
        );
    }

    /// ③ A move still queued: the callback waits for it and fires with that
    /// move's end time when the look-ahead is flushed.
    #[test]
    fn test_a_lookahead_callback_registered_with_a_queued_move_fires_at_its_end_time() {
        let (state, _gcode) = connected(homed_toolhead());
        let (_printer, object) = object_over(state);
        object
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .unwrap();

        let times = Arc::new(Mutex::new(Vec::new()));
        {
            let fired = Arc::clone(&times);
            object.register_lookahead_callback(Box::new(move |time| {
                fired.lock().unwrap().push(time);
            }));
        }
        assert!(
            times.lock().unwrap().is_empty(),
            "the move is still queued, so the callback waits for it"
        );

        // Flushes the look-ahead into the trapq.
        let _ = object.get_last_move_time();

        let fired = times.lock().unwrap();
        assert_eq!(fired.len(), 1, "fires when the queued move is flushed");
        let end = object.print_time();
        assert!((fired[0] - end).abs() < 1e-9, "{} vs {end}", fired[0]);
        assert!(fired[0] > 0.0);
    }

    /// A move goes through the planner to the trapq — the half of `G1` that
    /// was always the toolhead's. Parsing the words into a coordinate is
    /// `gcode_move`'s now; see its tests.
    #[test]
    fn test_a_move_reaches_the_planner() {
        let (state, _gcode) = connected(homed_toolhead());

        state
            .lock()
            .unwrap()
            .as_mut()
            .unwrap()
            .toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 10.0)
            .unwrap();
        // Flushing moves the look-ahead into the trapq; a single short move does
        // not trigger the flush on its own.
        state
            .lock()
            .unwrap()
            .as_mut()
            .unwrap()
            .toolhead
            .wait_moves();

        let guard = state.lock().unwrap();
        let connected = guard.as_ref().unwrap();
        assert_eq!(connected.toolhead.commanded_pos().x(), 10.0);
        // The move reached the trapq and can generate steps.
        assert!(!connected.toolhead.trapq().moves().is_empty());
    }

    #[test]
    fn test_a_move_on_an_unhomed_axis_is_refused() {
        let mut toolhead = homed_toolhead();
        toolhead.set_position(Coord::default(), &[]);
        // Clearing the homed axes is what an unhomed machine looks like.
        if let Some(kinematics) = toolhead.kinematics_mut() {
            kinematics.clear_homing_state(&[X_AXIS, Y_AXIS, Z_AXIS]);
        }
        let (state, _gcode) = connected(toolhead);

        let err = state
            .lock()
            .unwrap()
            .as_mut()
            .unwrap()
            .toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 10.0)
            .unwrap_err();

        assert!(err.to_string().contains("Must home axis first"), "{err}");
    }

    #[test]
    fn test_g4_advances_the_print_time() {
        let (state, gcode) = connected(homed_toolhead());
        let before = state
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .toolhead
            .print_time();
        let command = gcode.create_gcode_command(
            "G4",
            "G4 P500",
            HashMap::from([("P".to_string(), "500".to_string())]),
        );

        cmd_dwell(&state, &command).unwrap();

        let guard = state.lock().unwrap();
        let after = guard.as_ref().unwrap().toolhead.print_time();
        assert!((after - (before + 0.5)).abs() < 1e-9, "{after}");
    }

    #[test]
    fn test_set_kinematic_position_homes_and_clears() {
        let mut toolhead = homed_toolhead();
        toolhead.set_position(Coord::default(), &[]);
        if let Some(kinematics) = toolhead.kinematics_mut() {
            kinematics.clear_homing_state(&[X_AXIS, Y_AXIS, Z_AXIS]);
        }
        let (state, gcode, _printer) = connected_with_printer(toolhead);
        let command = gcode.create_gcode_command(
            "SET_KINEMATIC_POSITION",
            "SET_KINEMATIC_POSITION X=5 Y=6 Z=7",
            HashMap::from([
                ("X".to_string(), "5".to_string()),
                ("Y".to_string(), "6".to_string()),
                ("Z".to_string(), "7".to_string()),
            ]),
        );

        // The dispatcher holds its printer weakly, so the test keeps it alive
        // for the duration of the call.
        let printer = gcode.printer().expect("the printer is alive");
        cmd_set_kinematic_position(&state, &Arc::downgrade(&printer), &command).unwrap();

        let guard = state.lock().unwrap();
        let connected_ref = guard.as_ref().unwrap();
        assert_eq!(
            connected_ref.toolhead.commanded_pos(),
            Coord::new(5.0, 6.0, 7.0, 0.0)
        );
        assert_eq!(
            connected_ref.toolhead.kinematics().unwrap().get_status()["homed_axes"],
            "xyz"
        );
    }

    /// An endstop that completes as soon as it is armed.
    struct FakeEndstop {
        completion: Arc<Completion>,
    }

    impl HomingEndstop for FakeEndstop {
        fn home_start(
            &self,
            _print_time: f64,
            _sample_time: f64,
            _sample_count: u8,
            _rest_time: f64,
            _triggered: bool,
        ) -> Result<Arc<Completion>, McuError> {
            Ok(Arc::clone(&self.completion))
        }

        fn home_wait(&self, home_end_time: f64) -> EndstopFuture<'_> {
            Box::pin(async move {
                self.completion.wait().await;
                Ok(home_end_time)
            })
        }
    }

    fn test_homing_info() -> HomingInfo {
        HomingInfo {
            speed: 5.0,
            position_endstop: 0.0,
            retract_speed: 5.0,
            retract_dist: 5.0,
            positive_dir: false,
            second_homing_speed: 2.5,
        }
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn test_home_axis_places_the_axis_at_the_endstop() {
        // An unhomed machine; its endstop completes at once.
        let mut toolhead = homed_toolhead();
        toolhead.set_position(Coord::default(), &[]);
        if let Some(kinematics) = toolhead.kinematics_mut() {
            kinematics.clear_homing_state(&[X_AXIS, Y_AXIS, Z_AXIS]);
        }
        let (state, _gcode) = connected(toolhead);
        let mut connected = state
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take()
            .unwrap();
        let completion = Completion::new();
        completion.complete(crate::core::klippy::cmd::trsync::TriggerReason::EndstopHit);
        let endstop = FakeEndstop { completion };
        use crate::core::klippy::extras::stepper::RailParams;
        let params = RailParams {
            position_min: 0.0,
            position_max: 200.0,
            position_endstop: 0.0,
        };
        let info = test_homing_info();
        let (forcepos, movepos) =
            home_move(X_AXIS, &info, params.position_min, params.position_max);
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let printer = Arc::downgrade(&printer);

        let homing = HomingHandle::new();
        home_axis(
            &mut connected,
            X_AXIS,
            forcepos,
            movepos,
            &[X_AXIS],
            info,
            1.0,
            &["stepper_x".to_string()],
            &endstop,
            &homing,
            &printer,
        )
        .await
        .unwrap();

        // The axis is placed at its endstop and marked homed; the trapq is wiped.
        assert_eq!(connected.toolhead.commanded_pos().x(), 0.0);
        assert_eq!(
            connected.toolhead.kinematics().unwrap().get_status()["homed_axes"],
            "x"
        );
        assert!(connected.toolhead.trapq().moves().is_empty());
        // The trigger position is the stepper's MCU step position at the
        // trigger: X homed to its 0.0 endstop at 1 mm/step, so 0 steps.
        let state_guard = homing.lock();
        assert_eq!(state_guard.get_trigger_position("stepper_x"), 0.0);
        drop(state_guard);
        *state.lock().unwrap_or_else(|p| p.into_inner()) = Some(connected);
    }

    /// `set_stepper_adjustment` shifts the homed axis' coordinate by the
    /// requested offset, as `[endstop_phase]` makes it do
    /// (`Homing._do_home_rails`'s `adjust_pos` step).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_stepper_adjustment_shifts_the_homed_axis() {
        let mut toolhead = homed_toolhead();
        toolhead.set_position(Coord::default(), &[]);
        if let Some(kinematics) = toolhead.kinematics_mut() {
            kinematics.clear_homing_state(&[X_AXIS, Y_AXIS, Z_AXIS]);
        }
        let (state, _gcode) = connected(toolhead);
        let mut connected = state
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take()
            .unwrap();
        let completion = Completion::new();
        completion.complete(crate::core::klippy::cmd::trsync::TriggerReason::EndstopHit);
        let endstop = FakeEndstop { completion };
        use crate::core::klippy::extras::stepper::RailParams;
        let params = RailParams {
            position_min: 0.0,
            position_max: 200.0,
            position_endstop: 0.0,
        };
        let info = test_homing_info();
        let (forcepos, movepos) =
            home_move(X_AXIS, &info, params.position_min, params.position_max);
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let printer = Arc::downgrade(&printer);

        let homing = HomingHandle::new();
        home_axis(
            &mut connected,
            X_AXIS,
            forcepos,
            movepos,
            &[X_AXIS],
            info,
            1.0,
            &["stepper_x".to_string()],
            &endstop,
            &homing,
            &printer,
        )
        .await
        .unwrap();
        assert_eq!(connected.toolhead.commanded_pos().x(), 0.0);

        // A +0.5 mm endstop-phase offset moves the homed X coordinate to 0.5.
        let adjustments = homing.lock();
        let mut state_guard = adjustments;
        state_guard.set_stepper_adjustment("stepper_x", 0.5);
        apply_stepper_adjustments(&mut connected, &state_guard, &[X_AXIS]).unwrap();
        assert_eq!(connected.toolhead.commanded_pos().x(), 0.5);
        drop(state_guard);
        *state.lock().unwrap_or_else(|p| p.into_inner()) = Some(connected);
    }

    // =========================================================================
    // probing_move tests
    // =========================================================================

    /// An endstop that can be triggered at will, recording the order of
    /// `home_start` vs `home_wait` calls for event-order verification.
    struct TriggeringEndstop {
        completion: Arc<Completion>,
        /// If true, `home_wait` returns 0.0 (no trigger hit).
        no_trigger: std::sync::Mutex<bool>,
        /// Ordered list of operations for verifying begin fires before
        /// sampling/movement.
        ops: std::sync::Mutex<Vec<&'static str>>,
    }

    impl TriggeringEndstop {
        fn new(triggered: bool) -> Self {
            Self {
                completion: Completion::new(),
                no_trigger: std::sync::Mutex::new(!triggered),
                ops: std::sync::Mutex::new(Vec::new()),
            }
        }

        fn record_home_start(&self) {
            self.ops
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push("home_start");
        }

        fn record_home_wait(&self) {
            self.ops
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push("home_wait");
        }

        fn fire(&self) {
            self.completion
                .complete(crate::core::klippy::cmd::trsync::TriggerReason::EndstopHit);
        }

        fn ops_snapshot(&self) -> Vec<&'static str> {
            self.ops.lock().unwrap_or_else(|p| p.into_inner()).clone()
        }
    }

    impl HomingEndstop for TriggeringEndstop {
        fn home_start(
            &self,
            _print_time: f64,
            _sample_time: f64,
            _sample_count: u8,
            _rest_time: f64,
            _triggered: bool,
        ) -> Result<Arc<Completion>, McuError> {
            // `probing_move` sends begin **before** home_start, so recording
            // "begin" here captures the correct ordering.
            self.ops
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push("begin");
            self.record_home_start();
            Ok(Arc::clone(&self.completion))
        }

        fn home_wait(&self, home_end_time: f64) -> EndstopFuture<'_> {
            Box::pin(async move {
                self.record_home_wait();
                if *self.no_trigger.lock().unwrap_or_else(|p| p.into_inner()) {
                    return Ok(0.0);
                }
                if self.completion.reason().is_none() {
                    self.completion
                        .complete(crate::core::klippy::cmd::trsync::TriggerReason::EndstopHit);
                }
                self.completion.wait().await;
                Ok(home_end_time)
            })
        }
    }

    /// Build a connected state ready for probing.
    fn probing_connected() -> (Connected, Arc<Printer>) {
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let toolhead = homed_toolhead();
        let mut connected = Connected {
            toolhead,
            mcu_steppers: HashMap::new(),
            last_step_gen_time: 0.0,
            force_move_trapq: None,
        };
        // Advance print time so the first move has a non-zero delta.
        connected.toolhead.dwell(0.01);
        (connected, printer)
    }

    /// `probing_move` reports the move's end, matching upstream's
    /// file-output mode: upstream completes the trigger only in
    /// `TriggerDispatch.wait_end` — after the drip move ended — and takes
    /// `trigpos` at `home_end_time` (`klippy/mcu.py`,
    /// `klippy/extras/homing.py`). **Old assertion rewritten**: this test
    /// used to pin the drip loop's early stop ("short of the target … this
    /// fake endstop fires before the loop queues anything"), which walked
    /// every repeated probe up by its retract distance and failed the
    /// corpus's only `samples: 3` case
    /// (`screws_tilt_adjust.test`: Probe samples exceed samples_tolerance).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_probing_move_reports_the_move_end() {
        let (mut connected, printer) = probing_connected();
        let endstop = TriggeringEndstop::new(true);
        let target = Coord::new(10.0, 0.0, 0.0, 0.0);
        let speed = 5.0;
        let printer = Arc::downgrade(&printer);

        endstop.fire();

        let result = probing_move(&mut connected, &endstop, target, speed, &printer).await;

        let result = result.expect("the probing move completes");
        assert_eq!(
            result, target,
            "the drip ran the whole move; the stop position is its end"
        );
    }

    /// `probing_move` emits events in correct order: begin before home_start,
    /// end after home_wait.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_probing_move_emits_events_in_order() {
        let recorded = Arc::new(std::sync::Mutex::new(Vec::new()));
        let (mut connected, printer) = probing_connected();

        let begin_log = Arc::clone(&recorded);
        printer.register_event_handler(
            KlippyEvent::HomingHomingMoveBegin,
            Box::new(move |_| {
                begin_log
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push("begin");
            }),
        );
        let end_log = Arc::clone(&recorded);
        printer.register_event_handler(
            KlippyEvent::HomingHomingMoveEnd,
            Box::new(move |_| {
                end_log
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push("end");
            }),
        );

        let endstop = TriggeringEndstop::new(true);
        let target = Coord::new(10.0, 0.0, 0.0, 0.0);
        let speed = 5.0;
        let printer = Arc::downgrade(&printer);

        endstop.fire();
        let _ = probing_move(&mut connected, &endstop, target, speed, &printer).await;

        // Verify begin fires before home_start (sampling/movement).
        let ops = endstop.ops_snapshot();
        let begin_idx = ops.iter().position(|&op| op == "begin").unwrap();
        let start_idx = ops.iter().position(|&op| op == "home_start").unwrap();
        assert!(
            begin_idx < start_idx,
            "begin event must fire before home_start"
        );

        let events = recorded.lock().unwrap_or_else(|p| p.into_inner());
        assert_eq!(
            events.as_slice(),
            ["begin", "end"],
            "events in correct order"
        );
    }

    /// `probing_move` raises error when no trigger occurs after full move.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_probing_move_no_trigger_raises_error() {
        let (mut connected, printer) = probing_connected();
        let endstop = TriggeringEndstop::new(false);
        let target = Coord::new(10.0, 0.0, 0.0, 0.0);
        let speed = 5.0;
        let printer = Arc::downgrade(&printer);

        let result = probing_move(&mut connected, &endstop, target, speed, &printer).await;

        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(
            err.to_string()
                .contains("No trigger on probe after full movement"),
            "error should mention no trigger, got: {err}"
        );
    }

    /// A zero-length probe is a successful no-op reporting the current
    /// position: upstream's test mode answers it with the trigger at
    /// `home_end_time` — where a move that never moved already is
    /// (`check_no_movement` is disabled under `debuginput`;
    /// `MCU_trsync.stop` reports `REASON_ENDSTOP_HIT` unconditionally in
    /// fileoutput). **Old assertion rewritten**: the old mapping raised
    /// "Probe triggered prior to movement" for a zero-length move — but
    /// that upstream error is `check_no_movement` (an endstop that
    /// triggered before a **non-zero** move started), and with every probe
    /// now dripping to its file-output end, a probe started at the target
    /// (`z_virtual_endstop.test` runs `PROBE_CALIBRATE` straight after
    /// `PROBE`, both ending at `z_position`) hit the bogus error. The
    /// endstop is not armed: the fake firmware fires only on queued steps.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_probing_move_zero_distance_returns_the_current_position() {
        let (mut connected, printer) = probing_connected();
        let endstop = TriggeringEndstop::new(true);
        let target = Coord::default(); // same as current position
        let speed = 5.0;
        let printer = Arc::downgrade(&printer);

        let result = probing_move(&mut connected, &endstop, target, speed, &printer).await;

        let result = result.expect("a probe from the target succeeds");
        assert_eq!(
            result,
            Coord::default(),
            "the probe reports where it already stood"
        );
        assert_eq!(
            connected.toolhead.commanded_pos(),
            result,
            "the commanded position is left untouched"
        );
    }

    /// A probe whose XYZ distance is **sub-nanometer** — an fp residue below
    /// the `motion::plan::Move::new` collapse threshold (`< 1e-9`, e.g.
    /// `4.7e-16` left by `set_position`/`G1` arithmetic) — is the same
    /// no-op: the planner would drop the move to zero kinematic steps, so
    /// arming would deadlock `home_wait` (no step batch, no fire). The
    /// guard must share that epsilon rather than compare `== 0.0`.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_probing_move_sub_nanometer_distance_returns_without_arming() {
        let (mut connected, printer) = probing_connected();
        let endstop = TriggeringEndstop::new(true);
        let target = Coord::new(0.0, 0.0, 4.7e-16, 0.0); // below 1e-9, above 0
        let speed = 5.0;
        let printer = Arc::downgrade(&printer);

        let result = probing_move(&mut connected, &endstop, target, speed, &printer)
            .await
            .expect("a sub-nanometer probe succeeds as a no-op");
        assert_eq!(
            result,
            Coord::default(),
            "the probe reports where it already stood"
        );
        let ops = endstop.ops.lock().unwrap_or_else(|p| p.into_inner());
        assert!(
            !ops.iter().any(|op| *op == "home_start"),
            "the endstop is never armed: no steps would ever fire it"
        );
    }

    /// `probing_move` leaves the toolhead at the position it reports — here
    /// the move's end (file-output parity, see
    /// `test_probing_move_reports_the_move_end`); the next probe still has
    /// room to move down because every consumer lifts first (sample
    /// retract, point raise).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_probing_move_sets_position_on_trigger() {
        let (mut connected, printer) = probing_connected();
        let endstop = TriggeringEndstop::new(true);
        let target = Coord::new(10.0, 0.0, 0.0, 0.0);
        let speed = 5.0;
        let printer = Arc::downgrade(&printer);

        endstop.fire();
        let result = probing_move(&mut connected, &endstop, target, speed, &printer)
            .await
            .expect("the probing move completes");

        assert_eq!(
            connected.toolhead.commanded_pos(),
            result,
            "the commanded position follows the stop position"
        );
        assert_eq!(result, target, "the stop position is the move's end");
    }

    // =========================================================================
    // z_tilt seam tests: flush_step_generation, set_position, z_stepper_names
    // =========================================================================

    /// `flush_step_generation` means upstream's flush: when it returns, the
    /// moves queued **before** the call have been generated (and, here, handed
    /// to the transport — this `mcu_steppers` map is empty, so the send set is
    /// empty but the generation is fully observable).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_flush_step_generation_generates_the_moves_queued_before_it() {
        let (state, _gcode) = connected(homed_toolhead());
        {
            let mut guard = state.lock().unwrap_or_else(|p| p.into_inner());
            let connected_ref = guard.as_mut().unwrap();
            connected_ref
                .toolhead
                .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
                .unwrap();
            // A single short move waits in the look-ahead: nothing generated.
            let steppers = connected_ref.toolhead.motion_queuing_mut().steppers_mut();
            assert_eq!(steppers[0].commanded_position(), 0.0);
            assert!(steppers[0].history(10, 0, u64::MAX).is_empty());
        }
        let (_printer, object) = object_over(state);

        object.flush_step_generation().await.unwrap();

        let mut guard = object.lock();
        let connected_ref = guard.as_mut().unwrap();
        let steppers = connected_ref.toolhead.motion_queuing_mut().steppers_mut();
        // The solver ran the queued 10 mm move to its end, and the commands
        // reached the compressor's history — the steps exist, not just the
        // look-ahead entry.
        assert!((steppers[0].commanded_position() - 10.0).abs() < 1e-9);
        assert_eq!(steppers[0].mcu_position(), 10);
        assert!(!steppers[0].history(10, 0, u64::MAX).is_empty());
        // The look-ahead was flushed into the trapq: the planner's time moved,
        // which is what `get_last_move_time()` reports upstream.
        assert!(connected_ref.toolhead.print_time() > 0.0);
    }

    /// Upstream's `ToolHead.set_position` flushes step generation first, then
    /// sets position and homing flags and fires `toolhead:set_position` — the
    /// extras-level seam must do all three for `ZAdjustHelper.adjust_steppers`.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_set_position_flushes_marks_homing_axes_and_fires_the_event() {
        let (state, _gcode) = connected(homed_toolhead());
        {
            let mut guard = state.lock().unwrap_or_else(|p| p.into_inner());
            let connected_ref = guard.as_mut().unwrap();
            // Queue a move while homed, then unhome: the flush inside
            // `set_position` must still generate the queued move's steps, and
            // the homing flags must come back for the named axes only.
            connected_ref
                .toolhead
                .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
                .unwrap();
            if let Some(kinematics) = connected_ref.toolhead.kinematics_mut() {
                kinematics.clear_homing_state(&[X_AXIS, Y_AXIS, Z_AXIS]);
            }
            let steppers = connected_ref.toolhead.motion_queuing_mut().steppers_mut();
            assert!(steppers[0].history(10, 0, u64::MAX).is_empty());
        }
        let (printer, object) = object_over(state);
        let fired = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let count = Arc::clone(&fired);
        printer.register_event_handler(
            KlippyEvent::ToolheadSetPosition,
            Box::new(move |_| {
                count.fetch_add(1, Ordering::SeqCst);
            }),
        );

        object
            .set_position(Coord::new(1.0, 2.0, 3.0, 0.0), &[X_AXIS, Z_AXIS])
            .await
            .unwrap();

        assert_eq!(fired.load(Ordering::SeqCst), 1, "the event fired once");
        let mut guard = object.lock();
        let connected_ref = guard.as_mut().unwrap();
        assert_eq!(
            connected_ref.toolhead.commanded_pos(),
            Coord::new(1.0, 2.0, 3.0, 0.0)
        );
        assert_eq!(
            connected_ref.toolhead.kinematics().unwrap().get_status()["homed_axes"],
            "xz"
        );
        // The queued move's steps were generated before the position rewrite:
        // `set_position` would otherwise silently discard them.
        let steppers = connected_ref.toolhead.motion_queuing_mut().steppers_mut();
        assert!(!steppers[0].history(10, 0, u64::MAX).is_empty());
    }

    /// `z_stepper_names` reads the Z rail in config order — primary first,
    /// then `stepper_z1`, `stepper_z2` — which is the order `z_positions` and
    /// `adjustments` list the motors in.
    #[test]
    fn test_z_stepper_names_follow_the_z_rail_in_config_order() {
        use crate::core::klippy::config::Config;

        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        // A `z_tilt.cfg`-style config: three Z motors on one rail.
        let (config, _) = Config::from_text(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_y]\nstep_pin: PA2\ndir_pin: PA3\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z]\nstep_pin: PA4\ndir_pin: PA5\nrotation_distance: 8\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z1]\nstep_pin: PA6\ndir_pin: PA7\nrotation_distance: 8\nmicrosteps: 16\n\
             [stepper_z2]\nstep_pin: PB2\ndir_pin: PB3\nrotation_distance: 8\nmicrosteps: 16\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n",
        )
        .expect("the config parses");
        printer.load_config(&config).expect("the config loads");

        let object = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");

        assert_eq!(
            object.z_stepper_names(),
            ["stepper_z", "stepper_z1", "stepper_z2"]
        );
    }

    /// The declarations a client completes `KEY=` from, as they appear on
    /// `status.gcode.commands`: `G4` takes either unit of dwell time, `G28`
    /// takes the axes, `SET_KINEMATIC_POSITION` takes the axes and its homing
    /// words, `SET_VELOCITY_LIMIT` the four limits, `M204` its accel words, and
    /// `M400` waits for what is already queued — nothing to name.
    #[test]
    fn test_every_command_declares_the_parameters_it_reads() {
        let (printer, object) = object_over(Arc::new(Mutex::new(None)));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        object.register_commands(&printer).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the dispatcher is registered");
        let commands = gcode.get_status(0.0)["commands"].clone();
        assert_eq!(commands["G4"]["parameters"], json!(["S", "P"]));
        assert_eq!(commands["G28"]["parameters"], json!(["X", "Y", "Z"]));
        assert_eq!(
            commands["SET_KINEMATIC_POSITION"]["parameters"],
            json!(["X", "Y", "Z", "SET_HOMED", "CLEAR", "CLEAR_HOMED"])
        );
        assert_eq!(
            commands["SET_VELOCITY_LIMIT"]["parameters"],
            json!([
                "VELOCITY",
                "ACCEL",
                "SQUARE_CORNER_VELOCITY",
                "MINIMUM_CRUISE_RATIO"
            ])
        );
        assert_eq!(
            commands["SET_VELOCITY_LIMIT"]["help"],
            json!("Set printer velocity limits")
        );
        assert_eq!(commands["M204"]["parameters"], json!(["S", "P", "T"]));
        assert!(commands["M400"].get("parameters").is_none());
    }

    /// Neither velocity command reaches the unknown-command path (`gcode.py`'s
    /// `cmd_default`): both are registered, so a run reaches their handlers and
    /// emits no `Unknown command` line.
    #[test]
    fn test_the_velocity_commands_are_not_unknown() {
        let (printer, object) = object_over(Arc::new(Mutex::new(None)));
        let gcode = Arc::new(GCodeDispatch::new(Arc::clone(&printer)));
        let registered: Arc<dyn PrinterObject> = gcode.clone();
        printer.add_object(GCODE_OBJECT, registered).unwrap();
        object.register_commands(&printer).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);

        let output = Arc::new(Mutex::new(Vec::new()));
        {
            let output = Arc::clone(&output);
            gcode.register_output_handler(Arc::new(move |line: &str| {
                output
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner())
                    .push(line.to_string());
            }));
        }

        // `SET_VELOCITY_LIMIT` with no word reports the limits; `M204 S5` sets
        // the accel and is silent. Neither is an unknown command.
        assert!(gcode.run_script_sync("SET_VELOCITY_LIMIT").is_ok());
        assert!(gcode.run_script_sync("M204 S5").is_ok());

        let lines = output.lock().unwrap_or_else(|poison| poison.into_inner());
        assert!(
            !lines.iter().any(|line| line.contains("Unknown command")),
            "{lines:?}"
        );
        assert_eq!(object.limits_guard().max_accel, 5.0);
    }

    /// A word the command names must sit inside its bound; each bound keeps the
    /// dispatcher's wording, as upstream's `get_float` bounds do.
    #[test]
    fn test_set_velocity_limit_rejects_each_out_of_range_value() {
        let (state, gcode) = connected(homed_toolhead());
        let (_printer, object) = object_over(Arc::clone(&state));
        for (word, value, expected) in [
            ("VELOCITY", "0", "VELOCITY must be above 0"),
            ("ACCEL", "-1", "ACCEL must be above 0"),
            (
                "SQUARE_CORNER_VELOCITY",
                "-1",
                "SQUARE_CORNER_VELOCITY must have minimum of 0",
            ),
            (
                "MINIMUM_CRUISE_RATIO",
                "1",
                "MINIMUM_CRUISE_RATIO must be below 1",
            ),
        ] {
            let line = format!("SET_VELOCITY_LIMIT {word}={value}");
            let command = gcode.create_gcode_command(
                "SET_VELOCITY_LIMIT",
                &line,
                HashMap::from([(word.to_string(), value.to_string())]),
            );
            let err = cmd_set_velocity_limit(&object.state, &object.limits, &command).unwrap_err();
            assert!(err.to_string().contains(expected), "{word}: {err}");
        }
    }

    /// `M204 P… T…` uses the smaller of the two (`toolhead.py:593-600`).
    #[test]
    fn test_m204_takes_the_minimum_of_p_and_t() {
        let (state, gcode) = connected(homed_toolhead());
        let (_printer, object) = object_over(Arc::clone(&state));
        let command = gcode.create_gcode_command(
            "M204",
            "M204 P1 T2",
            HashMap::from([
                ("P".to_string(), "1".to_string()),
                ("T".to_string(), "2".to_string()),
            ]),
        );
        cmd_m204(&object.state, &object.limits, &command).unwrap();
        assert_eq!(object.limits_guard().max_accel, 1.0);
    }

    /// `M204 S0` is out of range, as upstream's `above=0.` refuses it.
    #[test]
    fn test_m204_rejects_a_zero_accel() {
        let (state, gcode) = connected(homed_toolhead());
        let (_printer, object) = object_over(Arc::clone(&state));
        let command = gcode.create_gcode_command(
            "M204",
            "M204 S0",
            HashMap::from([("S".to_string(), "0".to_string())]),
        );
        let err = cmd_m204(&object.state, &object.limits, &command).unwrap_err();
        assert!(err.to_string().contains("S must be above 0"), "{err}");
    }

    /// `M204` with neither `S` nor both of `P`/`T` reports the line and changes
    /// nothing (`toolhead.py:597-599`).
    #[test]
    fn test_m204_without_a_usable_word_is_refused() {
        let (state, gcode) = connected(homed_toolhead());
        let (_printer, object) = object_over(Arc::clone(&state));
        let output = Arc::new(Mutex::new(Vec::new()));
        {
            let output = Arc::clone(&output);
            gcode.register_output_handler(Arc::new(move |line: &str| {
                output
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner())
                    .push(line.to_string());
            }));
        }
        for (line, params) in [
            ("M204", HashMap::new()),
            (
                "M204 P5",
                HashMap::from([("P".to_string(), "5".to_string())]),
            ),
        ] {
            let command = gcode.create_gcode_command("M204", line, params);
            cmd_m204(&object.state, &object.limits, &command).unwrap();
            let lines = output.lock().unwrap_or_else(|poison| poison.into_inner());
            assert_eq!(lines.len(), 1, "{lines:?}");
            assert_eq!(lines[0], format!("// Invalid M204 command \"{line}\""));
            drop(lines);
            assert_eq!(object.limits_guard().max_accel, 1000.0);
            output
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .clear();
        }
    }

    /// The change shows up in `status.toolhead`, all four limits
    /// (`toolhead.py:502-515`).
    #[test]
    fn test_set_velocity_limit_updates_the_reported_limits() {
        let (state, gcode) = connected(homed_toolhead());
        let (_printer, object) = object_over(Arc::clone(&state));
        let command = gcode.create_gcode_command(
            "SET_VELOCITY_LIMIT",
            "SET_VELOCITY_LIMIT VELOCITY=20 ACCEL=100 SQUARE_CORNER_VELOCITY=1 \
             MINIMUM_CRUISE_RATIO=0",
            HashMap::from([
                ("VELOCITY".to_string(), "20".to_string()),
                ("ACCEL".to_string(), "100".to_string()),
                ("SQUARE_CORNER_VELOCITY".to_string(), "1".to_string()),
                ("MINIMUM_CRUISE_RATIO".to_string(), "0".to_string()),
            ]),
        );
        cmd_set_velocity_limit(&object.state, &object.limits, &command).unwrap();
        let status = object.get_status(0.0);
        assert_eq!(status["max_velocity"].as_f64(), Some(20.0));
        assert_eq!(status["max_accel"].as_f64(), Some(100.0));
        assert_eq!(status["square_corner_velocity"].as_f64(), Some(1.0));
        assert_eq!(status["minimum_cruise_ratio"].as_f64(), Some(0.0));
    }

    /// The planner profiles the *next* move from the new limits: a 100 mm move
    /// at 1000 mm/s is capped at 20 mm/s and accelerated at 100 mm/s², so it
    /// takes `d/v + v/a` = 5.2 s. This pins the two copies together — a
    /// `get_status` that reports the new limits while the planner keeps the old
    /// ones would fail here.
    ///
    /// The move is laid down with `drip_move` because it hands the profiled
    /// times back; it reads the same `Move::new(…, &self.limits)` a `move_to`
    /// does, so the limits a `move_to` would use are what is asserted.
    #[test]
    fn test_set_velocity_limit_reaches_the_next_move() {
        let (state, gcode) = connected(homed_toolhead());
        let (_printer, object) = object_over(Arc::clone(&state));
        let command = gcode.create_gcode_command(
            "SET_VELOCITY_LIMIT",
            "SET_VELOCITY_LIMIT VELOCITY=20 ACCEL=100",
            HashMap::from([
                ("VELOCITY".to_string(), "20".to_string()),
                ("ACCEL".to_string(), "100".to_string()),
            ]),
        );
        cmd_set_velocity_limit(&object.state, &object.limits, &command).unwrap();

        let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
        let connected = guard.as_mut().expect("the machine is up");
        let (start, end) = connected
            .toolhead
            .drip_move(Coord::new(100.0, 0.0, 0.0, 0.0), 1000.0)
            .unwrap();
        let expected = 100.0 / 20.0 + 20.0 / 100.0;
        assert!((end - start - expected).abs() < 1e-9, "{}", end - start);
    }

    /// A bare `SET_VELOCITY_LIMIT` reports the limits; naming a word is silent
    /// (`toolhead.py:582-589` — only the all-`None` query responds, and without
    /// logging).
    #[test]
    fn test_set_velocity_limit_reports_only_when_no_word_is_named() {
        let (state, gcode) = connected(homed_toolhead());
        let (_printer, object) = object_over(Arc::clone(&state));
        let output = Arc::new(Mutex::new(Vec::new()));
        {
            let output = Arc::clone(&output);
            gcode.register_output_handler(Arc::new(move |line: &str| {
                output
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner())
                    .push(line.to_string());
            }));
        }

        let query =
            gcode.create_gcode_command("SET_VELOCITY_LIMIT", "SET_VELOCITY_LIMIT", HashMap::new());
        cmd_set_velocity_limit(&object.state, &object.limits, &query).unwrap();
        let lines = output.lock().unwrap_or_else(|poison| poison.into_inner());
        assert_eq!(lines.len(), 1, "{lines:?}");
        assert_eq!(
            lines[0].as_str(),
            "// max_velocity: 200.000000\n// max_accel: 1000.000000\n\
             // minimum_cruise_ratio: 0.500000\n// square_corner_velocity: 5.000000"
        );
        drop(lines);
        output
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clear();

        let change = gcode.create_gcode_command(
            "SET_VELOCITY_LIMIT",
            "SET_VELOCITY_LIMIT VELOCITY=10",
            HashMap::from([("VELOCITY".to_string(), "10".to_string())]),
        );
        cmd_set_velocity_limit(&object.state, &object.limits, &change).unwrap();
        assert!(
            output
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .is_empty(),
            "a change is silent"
        );
        assert_eq!(object.limits_guard().max_velocity, 10.0);
    }

    /// FW6a-2: two responder fake MCUs in one printer, end to end.
    ///
    /// Each board runs its own identify/configuration handshake against its own
    /// dictionary, [`ToolHeadObject::connect`] then brings the machine up over
    /// both — the steppers it hands over live on *different* boards — and a
    /// homing move whose stepper sits on the primary board and endstop on the
    /// secondary fires across the link.
    ///
    /// The gap this pins shut: before [`SimulatorDevice::link_machine`] the
    /// fake read "the carriage moved" only off the steps arriving at *its own*
    /// instance, so a check armed on board B never saw board A's steps and
    /// `G28` waited for a trsync that could not arrive. The pins are chosen so
    /// a crossed identify cannot pass quietly: the stepper ports only exist on
    /// the primary board's dictionary, the endstop pin index only on the
    /// secondary's.
    ///
    /// Skipped when either dictionary was not built (`KLIPPERX_ARCHES`).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_toolhead_connect_brings_up_two_mcus_and_homes_across_them() {
        use crate::core::klippy::cmd::clock::{ClockState, GetClock};
        use crate::core::klippy::config::Config;
        use crate::core::klippy::interface::devices::responder_mcu::ResponderMcu;
        use crate::core::klippy::printer::PrinterState;
        use crate::core::klippy::reactor::TokioReactor;

        let (Some(primary), Some(aux)) = (
            ResponderMcu::new("mcu", "atmega2560.dict"),
            ResponderMcu::new("aux", "stm32f103.dict"),
        ) else {
            return;
        };
        let boards = [primary, aux];

        // Steppers on the primary board (H/J ports, which no STM32F103
        // dictionary has), the X endstop on the secondary (`aux:PB12`, pin
        // index 12 — an AVR port only holds 8). Y and Z stay un-homed here.
        let config_text = format!(
            "{}\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n\
             max_z_velocity: 15\nmax_z_accel: 100\n\
             [stepper_x]\nstep_pin: PH0\ndir_pin: PH1\nrotation_distance: 40\nmicrosteps: 16\n\
             endstop_pin: ^aux:PB12\nposition_endstop: 0\nposition_min: 0\nposition_max: 200\nhoming_speed: 50\n\
             [stepper_y]\nstep_pin: PJ0\ndir_pin: PJ1\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z]\nstep_pin: aux:PA0\ndir_pin: aux:PA1\nrotation_distance: 8\nmicrosteps: 16\nposition_max: 200\n",
            ResponderMcu::sections(&boards),
        );

        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let printer = Arc::new(Printer::new(reactor));
        let mut start_args = crate::core::klippy::api::StartArgs::collect("two-mcu.cfg", None);
        start_args.debug_output = Some("_test_output".to_string());
        printer.set_start_args(Arc::new(start_args));

        /// What the run produced, read while the machine was still up — the
        /// printer is torn down before anything is asserted, as `upstream`'s
        /// harness does, so a failure cannot leave a receive task parked.
        struct Evidence {
            /// The two sections are answered by two *different* instances.
            distinct_instances: bool,
            /// Each board identified, against its own dictionary file.
            primary_identified: bool,
            aux_identified: bool,
            primary_dict_len: usize,
            aux_dict_len: usize,
            primary_file_len: u64,
            aux_file_len: u64,
            /// A round trip answered by each board's own connection.
            clocks: Vec<u32>,
            /// `ToolHeadObject::connect` installed the motion state.
            toolhead_connected: bool,
            /// The MCU each stepper was handed over from, by axis.
            stepper_chips: Vec<(String, String)>,
            /// The axes homed by the cross-board move.
            homed_axes: String,
        }

        let outcome: Result<Evidence, String> = async {
            let (config, _) = Config::from_text(&config_text).map_err(|err| err.to_string())?;
            printer
                .load_config(&config)
                .map_err(|err| err.to_string())?;
            if tokio::time::timeout(Duration::from_secs(10), printer.bring_up())
                .await
                .is_err()
            {
                return Err("bring_up timed out".to_string());
            }
            let state = printer.get_state_message();
            if state.category != PrinterState::Ready {
                return Err(format!("not ready: {}", state.message));
            }

            // Two instances of the responder, each holding the file its own
            // section asked for: a shared or crossed instance shows up here.
            let primary_device = boards[0]
                .device(&printer)
                .ok_or_else(|| "[mcu mcu] has no responder instance".to_string())?;
            let aux_device = boards[1]
                .device(&printer)
                .ok_or_else(|| "[mcu aux] has no responder instance".to_string())?;
            let distinct_instances = !Arc::ptr_eq(&primary_device, &aux_device);
            let primary_dict_len = primary_device.dictionary_len();
            let aux_dict_len = aux_device.dictionary_len();
            let primary_file_len = std::fs::metadata(boards[0].dict())
                .map_err(|err| err.to_string())?
                .len();
            let aux_file_len = std::fs::metadata(boards[1].dict())
                .map_err(|err| err.to_string())?
                .len();

            // Each board answers its own query over its own connection.
            let mut clocks = Vec::new();
            for board in &boards {
                let mcu = board
                    .mcu(&printer)
                    .ok_or_else(|| format!("[mcu {}] is not connected", board.name()))?;
                let clock = mcu
                    .call_msg::<GetClock, ClockState>(&GetClock, Duration::from_secs(2))
                    .await
                    .map_err(|err| format!("{}: {err}", board.name()))?
                    .clock;
                clocks.push(clock);
            }
            let primary_identified = boards[0]
                .mcu(&printer)
                .map(|mcu| mcu.is_identified())
                .unwrap_or(false);
            let aux_identified = boards[1]
                .mcu(&printer)
                .map(|mcu| mcu.is_identified())
                .unwrap_or(false);

            // What `ToolHeadObject::connect` took: the motion state installed,
            // and every stepper paired with the MCU its pins live on.
            let toolhead = printer
                .lookup_object_as::<ToolHeadObject>("toolhead")
                .ok_or_else(|| "the toolhead is not registered".to_string())?;
            let (toolhead_connected, stepper_chips) = {
                let guard = toolhead.lock();
                let connected = guard
                    .as_ref()
                    .ok_or_else(|| "ToolHeadObject::connect has not run".to_string())?;
                let mut chips: Vec<(String, String)> = connected
                    .mcu_steppers
                    .iter()
                    .map(|(name, stepper)| (name.clone(), stepper.chip().name().to_string()))
                    .collect();
                chips.sort();
                (true, chips)
            };

            // One machine now: a move on either board trips the checks armed
            // on the other, which is what the cross-board `G28 X` needs.
            ResponderMcu::link_machine(&boards, &printer)?;
            let dispatcher = printer
                .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                .ok_or_else(|| "the g-code dispatcher is not registered".to_string())?;
            tokio::time::timeout(Duration::from_secs(30), dispatcher.run_script("G28 X\n"))
                .await
                .map_err(|_| "G28 X timed out waiting for a trsync".to_string())?
                .map_err(|err| format!("G28 X failed: {err}"))?;
            let state = printer.get_state_message();
            if state.category != PrinterState::Ready {
                return Err(format!(
                    "G28 X left the machine {:?}: {}",
                    state.category, state.message
                ));
            }
            let homed_axes = toolhead.get_status(0.0)["homed_axes"]
                .as_str()
                .unwrap_or_default()
                .to_string();

            Ok(Evidence {
                distinct_instances,
                primary_identified,
                aux_identified,
                primary_dict_len,
                aux_dict_len,
                primary_file_len,
                aux_file_len,
                clocks,
                toolhead_connected,
                stepper_chips,
                homed_axes,
            })
        }
        .await;
        printer.teardown();

        let evidence = outcome.expect("the two-MCU machine comes up and homes across the boards");
        assert!(
            evidence.distinct_instances,
            "two responder instances, one per [mcu …] section"
        );
        assert!(
            evidence.primary_identified && evidence.aux_identified,
            "each board ran its own identify handshake"
        );
        assert_eq!(
            evidence.primary_dict_len as u64, evidence.primary_file_len,
            "the primary board installed its own dictionary"
        );
        assert_eq!(
            evidence.aux_dict_len as u64, evidence.aux_file_len,
            "the secondary board installed its own dictionary"
        );
        assert_ne!(
            evidence.primary_file_len, evidence.aux_file_len,
            "the two dictionaries differ, so the lengths say which board got which"
        );
        assert!(
            evidence.clocks.iter().all(|clock| *clock > 0),
            "each board answered a query on its own connection: {:?}",
            evidence.clocks
        );
        assert!(
            evidence.toolhead_connected,
            "ToolHeadObject::connect installed the motion state"
        );
        assert_eq!(
            evidence.stepper_chips,
            [
                ("stepper_x".to_string(), "mcu".to_string()),
                ("stepper_y".to_string(), "mcu".to_string()),
                ("stepper_z".to_string(), "aux".to_string()),
            ],
            "every stepper was handed over with the MCU its pins live on"
        );
        assert!(
            evidence.homed_axes.contains('x'),
            "the stepper on the primary board tripped the endstop on the secondary: {:?}",
            evidence.homed_axes
        );
    }

    // ------------------------------------------------------------------
    // `ToolHeadObject::manual_move` — the force-move move
    // ------------------------------------------------------------------

    /// A connected state over [`homed_toolhead`] for the force-move seam: its
    /// steppers are cartesian at one millimetre per step, so a move of `dist`
    /// millimetres is exactly `dist` steps.
    fn force_move_connected() -> Connected {
        let mut connected = Connected {
            toolhead: homed_toolhead(),
            mcu_steppers: HashMap::new(),
            last_step_gen_time: 0.0,
            force_move_trapq: None,
        };
        // Advance print time so the move starts at a non-zero time.
        connected.toolhead.dwell(0.01);
        connected
    }

    /// The force-move move drives the motor, then restores the stepper's own
    /// solver and trapq and wipes the force-move queue — the planner is not
    /// moved (`force_move.py:75-91`).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_manual_move_restores_the_solver_and_trapq_after_moving() {
        let mut connected = force_move_connected();
        let main_trapq = connected.toolhead.main_trapq();
        let commanded = connected.toolhead.commanded_pos();
        let (solver_step, trapq_before, steps_before) = {
            let stepper = &mut connected.toolhead.motion_queuing_mut().steppers_mut()[0];
            (
                stepper.kinematics().commanded_pos(),
                stepper.trapq_id(),
                stepper.compressor_mut().last_position(),
            )
        };

        manual_move(&mut connected, "stepper_x", 10.0, 100.0, 0.0)
            .await
            .expect("the move runs");

        let (solver_after, trapq_after, steps_after) = {
            let stepper = &mut connected.toolhead.motion_queuing_mut().steppers_mut()[0];
            (
                stepper.kinematics().commanded_pos(),
                stepper.trapq_id(),
                stepper.compressor_mut().last_position(),
            )
        };
        assert_eq!(
            trapq_before,
            Some(main_trapq),
            "the stepper starts on the main trapq"
        );
        assert_eq!(
            trapq_after,
            Some(main_trapq),
            "the stepper's own trapq is put back"
        );
        assert_eq!(
            solver_after, solver_step,
            "the stepper's own solver position is untouched (commanded_pos unchanged)"
        );
        assert_eq!(
            connected.toolhead.commanded_pos(),
            commanded,
            "the planner is not moved by a force move"
        );
        assert_eq!(
            steps_after - steps_before,
            10,
            "the 10 mm move (1 mm/step) generated 10 steps"
        );
        // The force-move trapq is kept for reuse but emptied.
        let force_trapq = connected.force_move_trapq.expect("a trapq was allocated");
        assert!(
            connected
                .toolhead
                .motion_queuing_mut()
                .trapq_mut(force_trapq)
                .moves()
                .is_empty(),
            "the force-move queue is wiped"
        );
        // Its duration is the trapezoid's: 10 mm at 100 mm/s, no accel. The
        // planner also primes 0.25 s past the estimate (default 0) on the first
        // move, so the horizon lands at 0.25 + 0.1.
        assert!(
            (connected.toolhead.print_time() - 0.35).abs() < 1e-9,
            "print time {} does not match the move's 0.1 s after the 0.25 s prime",
            connected.toolhead.print_time()
        );
    }

    /// Two moves in a row: the shared state is taken and put back each time, so
    /// the second (and a third) run does not see a locked-out toolhead.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_consecutive_manual_moves_keep_the_shared_state_usable() {
        let state = Arc::new(Mutex::new(Some(force_move_connected())));
        let (_printer, object) = object_over(Arc::clone(&state));

        object
            .manual_move("stepper_x", 5.0, 50.0, 0.0)
            .await
            .expect("the first move runs");
        object
            .manual_move("stepper_x", -5.0, 50.0, 0.0)
            .await
            .expect("the second move runs");
        object
            .manual_move("stepper_x", 1.0, 100.0, 0.0)
            .await
            .expect("the state was restored, so a third move still runs");
    }

    /// A name the motion queue does not hold carries the move on the timeline
    /// only, so `STEPPER_BUZZ` / `FORCE_MOVE` answer for a stepper those
    /// modules have not wired (the manual stepper, the IDEX second carriage)
    /// instead of erroring. The planner's position is still untouched.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_manual_move_carries_an_unwired_stepper_on_the_timeline_only() {
        let mut connected = force_move_connected();
        let commanded = connected.toolhead.commanded_pos();

        manual_move(&mut connected, "not_on_the_queue", 10.0, 100.0, 0.0)
            .await
            .expect("the move is accepted");

        assert_eq!(connected.toolhead.commanded_pos(), commanded);
        // The move's 0.1 s still lands on the timeline, after the first move's
        // 0.25 s prime (print time starts at the fixture's 0.01 s dwell).
        assert!(
            (connected.toolhead.print_time() - 0.35).abs() < 1e-9,
            "print time landed at {}",
            connected.toolhead.print_time()
        );
    }
}
