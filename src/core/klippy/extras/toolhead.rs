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
//! (`klippy/toolhead.py:604-615`). The G-code commands are registered at load
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
//! [`FLUSH_INTERVAL`], generates everything the planner has queued, and awaits
//! the transport, so a long move cannot outrun the send queue. The task checks
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
use crate::core::klippy::extras::extruder::PrinterExtruder;
use crate::core::klippy::extras::query_endstops::{QueryEndstops, QUERY_ENDSTOPS_OBJECT};
use crate::core::klippy::extras::stepper::{Rail, RailParams};
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{Coord, X_AXIS, Y_AXIS, Z_AXIS};
use crate::core::klippy::mcu::{Completion, McuEndstop, McuError, McuObject, McuStepper};
use crate::core::klippy::motion::extra::ExtraAxis;
use crate::core::klippy::motion::itersolve::{
    cartesian_active_flags, cartesian_position_fn, corexy_active_flags, corexy_position_fn,
    corexz_active_flags, corexz_position_fn, Axis, AxisFlags, PositionFn,
};
use crate::core::klippy::motion::kinematics::{
    home_move, CartesianKinematics, CartesianTransform, NoneKinematics,
};
use crate::core::klippy::motion::plan::MoveLimits;
use crate::core::klippy::motion::stepcompress::{StepCommand, StepCompressError};
use crate::core::klippy::motion::toolhead::ToolHead;
use crate::core::klippy::motion::{HomeCoord, HomingInfo};
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject, RestartHooks};
use crate::core::klippy::reactor::Reactor;

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

/// How old a finished move may stay in the trapq history before it is dropped.
const MOVE_HISTORY_EXPIRE: f64 = 30.0;

/// The cartesian-family kinematics `[printer] kinematics` may name.
///
/// They share one `CartesianKinematics` (limits, homing, `check_move`) and
/// differ in which solver each rail runs, how carriage axes map to rail
/// positions (`CartesianTransform`), and which endstops watch which motors
/// (`kinematics/corexy.py`, `corexz.py`, `hybrid_corexy.py`, `hybrid_corexz.py`).
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
        }
    }

    /// The solver each rail's steppers run (`X`, `Y`, `Z` order).
    fn solvers(self) -> [(PositionFn, AxisFlags); 3] {
        let cart = |axis: Axis| (cartesian_position_fn(axis), cartesian_active_flags(axis));
        match self {
            Self::None | Self::Cartesian => [cart(Axis::X), cart(Axis::Y), cart(Axis::Z)],
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

/// The `toolhead` object: the planner, its kinematics, and the MCU steppers.
pub struct ToolHeadObject {
    limits: MoveLimits,
    max_z_velocity: f64,
    max_z_accel: f64,
    /// The cartesian rails, `[stepper_x]`, `[stepper_y]`, `[stepper_z]` (each
    /// with its `…1`, `…2` siblings).
    ///
    /// Empty for `kinematics: none`, which has no steppers.
    rails: Vec<Arc<Rail>>,
    /// Whether `[printer] kinematics` was `none`.
    none: bool,
    /// How the rails' positions map to carriage axes.
    transform: CartesianTransform,
    /// The active extruder's name (`ACTIVATE_EXTRUDER`), for `M104` without `T`.
    active_extruder: Mutex<String>,
    /// The machine's clock, for seeding the print-time mapping.
    reactor: Arc<dyn Reactor>,
    /// The machine, to shut it down if the compressor hits an internal error.
    /// `Weak` because the printer's registry owns this object.
    printer: Weak<Printer>,
    /// The connected motion state; `None` until connect.
    state: Arc<Mutex<Option<Connected>>>,
    /// Set when the object is dropped, to stop the flush task.
    shutdown: Arc<AtomicBool>,
}

/// Everything that exists only once the machine is up.
struct Connected {
    toolhead: ToolHead,
    mcu_steppers: HashMap<String, Arc<McuStepper>>,
    /// The print time the solvers have generated up to.
    last_step_gen_time: f64,
}

/// The step commands to send, paired with the stepper that produced them.
type StepBatches = Vec<(Arc<McuStepper>, Vec<StepCommand>)>;

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
        let none = matches!(kind, KinematicsKind::None);

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

        // The junction geometry (`ToolHead._calc_junction_deviation`).
        let junction_deviation =
            square_corner_velocity.powi(2) * (std::f64::consts::SQRT_2 - 1.0) / max_accel;
        let limits = MoveLimits {
            max_velocity,
            max_accel,
            junction_deviation,
            mcr_pseudo_accel: max_accel * (1.0 - min_cruise_ratio),
        };

        let mut rails = Vec::new();
        if !none {
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
                    endstop
                        .dispatch()
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
            none,
            transform: kind.transform(),
            active_extruder: Mutex::new("extruder".to_string()),
            reactor: printer.reactor(),
            printer: Arc::downgrade(printer),
            state,
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
            .register_command("G4", dwell_handler, None, false)
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
            .register_command(
                "SET_KINEMATIC_POSITION",
                position_handler,
                Some("Force a low-level kinematic position"),
                false,
            )
            .map_err(ConfigError::new)?;
        let home_handler: CommandHandler = {
            let state = Arc::clone(&self.state);
            let rails = self.rails.clone();
            let printer = Arc::downgrade(printer);
            Arc::new(move |gcmd: &GcodeCommand| {
                let state = Arc::clone(&state);
                let rails = rails.clone();
                let printer = printer.clone();
                Box::pin(async move { cmd_g28(&state, &rails, &printer, gcmd).await })
            })
        };
        gcode
            .register_command("G28", home_handler, Some("Home one or more axes"), false)
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

    /// The machine's maximum velocity, for `[extruder]`'s speed defaults.
    pub fn max_velocity(&self) -> f64 {
        self.limits.max_velocity
    }

    /// The machine's maximum acceleration, for `[extruder]`'s speed defaults.
    pub fn max_accel(&self) -> f64 {
        self.limits.max_accel
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
        json!({
            "position": position.as_array(),
            "homed_axes": homed_axes,
            "print_time": connected.toolhead.print_time(),
            "max_velocity": self.limits.max_velocity,
            "max_accel": self.limits.max_accel,
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

            // The primary MCU (the bare `[mcu]`) defines the print-time origin;
            // each stepper's compressor was already pointed at its own MCU's
            // clock domain (`SecondarySync`) during its own connect, which used
            // this same `[mcu]` object's clock.
            let main_print_time = self
                .printer
                .upgrade()
                .and_then(|printer| printer.lookup_object_as::<McuObject>("mcu"))
                .and_then(|object| object.estimated_print_time(self.reactor.monotonic()))
                .unwrap_or(0.0);

            let mut toolhead = ToolHead::new(self.limits);
            for stepper in host_steppers {
                toolhead.add_stepper(stepper);
            }
            if self.none {
                toolhead.set_kinematics(Box::new(NoneKinematics));
            } else {
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

            // The toolhead's print time is the primary MCU's.
            toolhead.set_estimated_print_time(main_print_time);

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

            *self.lock() = Some(Connected {
                toolhead,
                mcu_steppers,
                last_step_gen_time: 0.0,
            });

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
    pub fn print_time(&self) -> f64 {
        self.lock()
            .as_ref()
            .map(|connected| connected.toolhead.print_time())
            .unwrap_or(0.0)
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
        for (stepper, commands) in batches {
            if let Err(err) = stepper.send_steps_async(&commands).await {
                warn!(
                    "{}: {err}",
                    stepper.oid().map(u32::from).unwrap_or_default()
                );
            }
        }
    }
}

impl Connected {
    /// Generate the steps for everything queued, and return them by stepper.
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
        let step_gen_time = self.toolhead.print_time().max(self.last_step_gen_time);
        let batches = self.toolhead.flush_step_generation(step_gen_time)?;
        self.toolhead.finalize_moves(
            step_gen_time,
            (step_gen_time - MOVE_HISTORY_EXPIRE).max(0.0),
        );
        self.last_step_gen_time = step_gen_time;
        Ok(batches
            .into_iter()
            .filter_map(|(name, commands)| {
                self.mcu_steppers
                    .get(&name)
                    .map(|stepper| (Arc::clone(stepper), commands))
            })
            .collect())
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
/// (`HOMING_START_DELAY`, `klippy/extras/homing.py:9`).
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
    requested: &[usize],
    printer: &Weak<Printer>,
) -> Result<(), CommandError> {
    // `kinematics: none` has no rails, so there is nothing to home.
    if rails.is_empty() {
        return Ok(());
    }
    for &axis in requested {
        let rail = &rails[axis];
        let endstop = rail.endstop().ok_or_else(|| {
            CommandError::new(format!("No endstop configured for {}", rail.name()))
        })?;
        send(printer, &KlippyEvent::HomingHomeRailsBegin);
        let result = home_axis(
            connected,
            axis,
            rail.homing_info(),
            rail.params(),
            rail.step_dist(),
            endstop.as_ref(),
            printer,
        )
        .await;
        send(
            printer,
            &KlippyEvent::HomingHomeRailsEnd { axes: vec![axis] },
        );
        result?;
    }
    Ok(())
}

/// Fire a printer event, when the machine is still there.
fn send(printer: &Weak<Printer>, event: &KlippyEvent) {
    if let Some(printer) = printer.upgrade() {
        printer.send_event(event);
    }
}

/// Home one axis: pretend to be at `forcepos`, move to the endstop, then place
/// the axis at its `position_endstop` (`CartKinematics.home_axis` +
/// `Homing._do_home_rails` + `HomingMove.homing_move`).
#[allow(clippy::too_many_arguments)]
async fn home_axis(
    connected: &mut Connected,
    axis: usize,
    info: HomingInfo,
    params: RailParams,
    step_dist: f64,
    endstop: &dyn HomingEndstop,
    printer: &Weak<Printer>,
) -> Result<(), CommandError> {
    // Start 1.5 axis-lengths past the far end so the move always approaches the
    // endstop from the correct side.
    let (forcepos, movepos) = home_move(axis, &info, params.position_min, params.position_max);
    let current = connected.toolhead.commanded_pos();
    let force = fill_coord(forcepos, current);
    let home = fill_coord(movepos, current);
    connected.toolhead.set_position(force, &[axis]);

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
    while flush_time < end && completion.reason().is_none() {
        flush_time = (flush_time + DRIP_SEGMENT_TIME).min(end);
        let batches = connected
            .toolhead
            .flush_step_generation(flush_time)
            .map_err(|err| CommandError::new(err.to_string()))?;
        for (name, commands) in batches {
            if let Some(stepper) = connected.mcu_steppers.get(&name) {
                stepper
                    .send_steps_async(&commands)
                    .await
                    .map_err(command_error)?;
            }
        }
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

    endstop.home_wait(end).await.map_err(command_error)?;
    send(printer, &KlippyEvent::HomingHomingMoveEnd);
    // The axis is now known at its endstop position.
    connected.toolhead.set_position(home, &[axis]);
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
/// Returns the toolhead's commanded position at the end of the move.  This
/// is the position the planner computed for `target` — not a position
/// derived from trigger-step counting (that infrastructure does not exist
/// in this port yet).
///
/// # Errors
/// - `"Probe triggered prior to movement"` when the endstop was already
///   triggered before any motion started.
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

    // Zero-length move: nothing to move, probe triggered before movement.
    if distance == 0.0 {
        return Err(CommandError::new("Probe triggered prior to movement"));
    }

    let print_time = connected.toolhead.get_last_move_time();
    let completion = endstop
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

    // Drip the move out in small windows; stop as soon as the trigger fires.
    let mut flush_time = start;
    while flush_time < end && completion.reason().is_none() {
        flush_time = (flush_time + DRIP_SEGMENT_TIME).min(end);
        let batches = connected
            .toolhead
            .flush_step_generation(flush_time)
            .map_err(|err| CommandError::new(err.to_string()))?;
        for (name, commands) in batches {
            if let Some(stepper) = connected.mcu_steppers.get(&name) {
                stepper
                    .send_steps_async(&commands)
                    .await
                    .map_err(command_error)?;
            }
        }
        if completion.reason().is_none() {
            tokio::select! {
                _ = sleep(Duration::from_secs_f64(DRIP_LOOKAHEAD)) => {}
                _ = completion.wait() => {}
            }
        }
    }

    let trigger_time = endstop.home_wait(end).await.map_err(command_error)?;
    send(printer, &KlippyEvent::HomingHomingMoveEnd);

    // No trigger: upstream raises "No trigger on {name} after full movement"
    // after emitting homing_move_end.
    if trigger_time <= 0.0 {
        return Err(CommandError::new("No trigger on probe after full movement"));
    }

    // Return the commanded position after the move.
    Ok(connected.toolhead.commanded_pos())
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
    let result = home_axes(&mut connected, rails, &requested, printer).await;
    *state.lock().unwrap_or_else(|poison| poison.into_inner()) = Some(connected);
    result
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
            ConfigValue::Single("delta".to_string()),
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

        assert!(err.to_string().contains("Error loading kinematics 'delta'"));
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
        ToolHeadObject::new(&ConfigWrapper::untracked(&section), &printer)
            .expect("kinematics: none builds without steppers");
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
    fn connected(toolhead: ToolHead) -> (Arc<Mutex<Option<Connected>>>, GCodeDispatch) {
        let state = Arc::new(Mutex::new(Some(Connected {
            toolhead,
            mcu_steppers: HashMap::new(),
            last_step_gen_time: 0.0,
        })));
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        (state, GCodeDispatch::new(printer))
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
        let (state, gcode) = connected(toolhead);
        let command = gcode.create_gcode_command(
            "SET_KINEMATIC_POSITION",
            "SET_KINEMATIC_POSITION X=5 Y=6 Z=7",
            HashMap::from([
                ("X".to_string(), "5".to_string()),
                ("Y".to_string(), "6".to_string()),
                ("Z".to_string(), "7".to_string()),
            ]),
        );

        cmd_set_kinematic_position(&state, &Arc::downgrade(&gcode.printer()), &command).unwrap();

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
        let params = RailParams {
            position_min: 0.0,
            position_max: 200.0,
            position_endstop: 0.0,
        };
        let printer = Arc::new(Printer::new(
            crate::core::klippy::reactor::ManualReactor::shared(),
        ));
        let printer = Arc::downgrade(&printer);

        home_axis(
            &mut connected,
            X_AXIS,
            test_homing_info(),
            params,
            1.0,
            &endstop,
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
        };
        // Advance print time so the first move has a non-zero delta.
        connected.toolhead.dwell(0.01);
        (connected, printer)
    }

    /// `probing_move` stops on trigger and returns the commanded position.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_probing_move_stops_on_trigger() {
        let (mut connected, printer) = probing_connected();
        let endstop = TriggeringEndstop::new(true);
        let target = Coord::new(10.0, 0.0, 0.0, 0.0);
        let speed = 5.0;
        let printer = Arc::downgrade(&printer);

        endstop.fire();

        let result = probing_move(&mut connected, &endstop, target, speed, &printer).await;

        assert!(result.is_ok());
        assert_eq!(
            result.unwrap(),
            target,
            "returned position should match target"
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

    /// `probing_move` raises error when distance is zero (probe triggered
    /// before any movement).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_probing_move_zero_distance_raises_error() {
        let (mut connected, printer) = probing_connected();
        let endstop = TriggeringEndstop::new(true);
        let target = Coord::default(); // same as current position
        let speed = 5.0;
        let printer = Arc::downgrade(&printer);

        let result = probing_move(&mut connected, &endstop, target, speed, &printer).await;

        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(
            err.to_string()
                .contains("Probe triggered prior to movement"),
            "zero distance should raise prior-movement error, got: {err}"
        );
    }

    /// `probing_move` sets the toolhead position to the target after trigger.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_probing_move_sets_position_on_trigger() {
        let (mut connected, printer) = probing_connected();
        let endstop = TriggeringEndstop::new(true);
        let target = Coord::new(10.0, 0.0, 0.0, 0.0);
        let speed = 5.0;
        let printer = Arc::downgrade(&printer);

        endstop.fire();
        let _ = probing_move(&mut connected, &endstop, target, speed, &printer).await;

        assert_eq!(
            connected.toolhead.commanded_pos(),
            target,
            "toolhead position should match target after probe"
        );
    }
}
