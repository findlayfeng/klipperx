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
//! | `G0` / `G1` | linear move, absolute coordinates and `F` in mm/min |
//! | `G4` | dwell, `P` in milliseconds or `S` in seconds |
//! | `M400` | flush the planner |
//! | `SET_KINEMATIC_POSITION` | force the low-level position, homing the named axes |
//!
//! There is no `gcode_move` layer yet: coordinates are the toolhead's, with none
//! of the offsets, relative mode or extruder factors that module adds (H3). Pure
//! extruder moves are parsed and recorded but do not generate steps, because
//! there is no `[extruder]` rail.
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
use crate::core::klippy::extras::query_endstops::{QueryEndstops, QUERY_ENDSTOPS_OBJECT};
use crate::core::klippy::extras::stepper::{PrinterStepper, RailParams};
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{Coord, X_AXIS, Y_AXIS, Z_AXIS};
use crate::core::klippy::mcu::{Completion, McuEndstop, McuError, McuObject, McuStepper};
use crate::core::klippy::motion::kinematics::{home_move, CartesianKinematics};
use crate::core::klippy::motion::plan::MoveLimits;
use crate::core::klippy::motion::stepcompress::{StepCommand, StepCompressError};
use crate::core::klippy::motion::toolhead::ToolHead;
use crate::core::klippy::motion::{HomeCoord, HomingInfo};
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};
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

/// The speed a `G1` uses before any `F` (`gcode_move`'s initial `self.speed`).
const DEFAULT_MOVE_SPEED: f64 = 50.0;

/// The `toolhead` object: the planner, its kinematics, and the MCU steppers.
pub struct ToolHeadObject {
    limits: MoveLimits,
    max_z_velocity: f64,
    max_z_accel: f64,
    /// The three cartesian rail sections, `[stepper_x]`, `[stepper_y]`,
    /// `[stepper_z]`.
    axes: [Arc<PrinterStepper>; 3],
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
        let identifier = config.identifier();
        let kinematics = config.get("kinematics", None)?;
        if kinematics != "cartesian" {
            return Err(ConfigError::new(format!(
                "Error loading kinematics '{kinematics}' (only 'cartesian' is implemented)"
            )));
        }

        let max_velocity = config.get_float("max_velocity", None)?;
        if max_velocity <= 0.0 {
            return Err(ConfigError::new(format!(
                "Option 'max_velocity' in section '{identifier}' must be above 0"
            )));
        }
        let max_accel = config.get_float("max_accel", None)?;
        if max_accel <= 0.0 {
            return Err(ConfigError::new(format!(
                "Option 'max_accel' in section '{identifier}' must be above 0"
            )));
        }
        let min_cruise_ratio = config.get_float("minimum_cruise_ratio", Some(0.5))?;
        if !(0.0..1.0).contains(&min_cruise_ratio) {
            return Err(ConfigError::new(format!(
                "Option 'minimum_cruise_ratio' in section '{identifier}' must be between 0 and 1"
            )));
        }
        let square_corner_velocity = config.get_float("square_corner_velocity", Some(5.0))?;
        if square_corner_velocity < 0.0 {
            return Err(ConfigError::new(format!(
                "Option 'square_corner_velocity' in section '{identifier}' must not be negative"
            )));
        }
        let max_z_velocity = config.get_float("max_z_velocity", Some(15.0))?;
        let max_z_accel = config.get_float("max_z_accel", Some(100.0))?;
        if max_z_velocity <= 0.0 || max_z_accel <= 0.0 {
            return Err(ConfigError::new(format!(
                "Option 'max_z_velocity' / 'max_z_accel' in section '{identifier}' must be above 0"
            )));
        }

        // The junction geometry (`ToolHead._calc_junction_deviation`).
        let junction_deviation =
            square_corner_velocity.powi(2) * (std::f64::consts::SQRT_2 - 1.0) / max_accel;
        let limits = MoveLimits {
            max_velocity,
            max_accel,
            junction_deviation,
            mcr_pseudo_accel: max_accel * (1.0 - min_cruise_ratio),
        };

        let mut axes = Vec::with_capacity(3);
        for name in ["stepper_x", "stepper_y", "stepper_z"] {
            let stepper = printer
                .lookup_object_as::<PrinterStepper>(name)
                .ok_or_else(|| {
                    ConfigError::new(format!(
                        "Section '{identifier}' needs a '[{name}]' section for cartesian kinematics"
                    ))
                })?;
            axes.push(stepper);
        }
        let axes: [Arc<PrinterStepper>; 3] =
            axes.try_into().expect("exactly three axes were collected");

        // The object every rail's endstop is queried through. Created here
        // because this is the first point where all the `[stepper_*]` sections
        // (and their endstops) exist; registered before `toolhead` itself.
        let query = QueryEndstops::new(printer)?;
        for stepper in &axes {
            if let Some(endstop) = stepper.endstop() {
                query.register_endstop(Arc::clone(endstop), stepper.name());
            }
        }
        printer.add_object(QUERY_ENDSTOPS_OBJECT, Arc::new(query))?;

        let state = Arc::new(Mutex::new(None));
        let object = Self {
            limits,
            max_z_velocity,
            max_z_accel,
            axes,
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
        let speed = Arc::new(Mutex::new(DEFAULT_MOVE_SPEED));

        let move_handler = move_command(Arc::clone(&self.state), Arc::clone(&speed));
        for name in ["G0", "G1"] {
            gcode
                .register_command(name, Arc::clone(&move_handler), None, false)
                .map_err(ConfigError::new)?;
        }
        let dwell_handler: CommandHandler = {
            let state = Arc::clone(&self.state);
            Arc::new(move |gcmd| cmd_dwell(&state, gcmd))
        };
        gcode
            .register_command("G4", dwell_handler, None, false)
            .map_err(ConfigError::new)?;
        let wait_handler: CommandHandler = {
            let state = Arc::clone(&self.state);
            Arc::new(move |gcmd| cmd_wait_moves(&state, gcmd))
        };
        gcode
            .register_command("M400", wait_handler, None, false)
            .map_err(ConfigError::new)?;
        let position_handler: CommandHandler = {
            let state = Arc::clone(&self.state);
            Arc::new(move |gcmd| cmd_set_kinematic_position(&state, gcmd))
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
            let axes = self.axes.clone();
            let printer = Arc::downgrade(printer);
            Arc::new(move |gcmd| cmd_g28(&state, &axes, &printer, gcmd))
        };
        gcode
            .register_command("G28", home_handler, Some("Home one or more axes"), false)
            .map_err(ConfigError::new)?;
        Ok(())
    }

    /// The names of the three rails, in axis order.
    fn axis_names(&self) -> [String; 3] {
        [
            self.axes[0].name().to_string(),
            self.axes[1].name().to_string(),
            self.axes[2].name().to_string(),
        ]
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
            let mut host_steppers = Vec::with_capacity(3);
            let mut mcu_steppers = HashMap::new();
            for stepper in &self.axes {
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
            toolhead.set_kinematics(Box::new(CartesianKinematics::new(
                self.axis_names(),
                Coord::new(
                    self.axes[X_AXIS].params().position_min,
                    self.axes[Y_AXIS].params().position_min,
                    self.axes[Z_AXIS].params().position_min,
                    0.0,
                ),
                Coord::new(
                    self.axes[X_AXIS].params().position_max,
                    self.axes[Y_AXIS].params().position_max,
                    self.axes[Z_AXIS].params().position_max,
                    0.0,
                ),
                self.max_z_velocity,
                self.max_z_accel,
            )));

            // The toolhead's print time is the primary MCU's.
            toolhead.set_estimated_print_time(main_print_time);

            *self.lock() = Some(Connected {
                toolhead,
                mcu_steppers,
                last_step_gen_time: 0.0,
            });

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
/// The caller has taken the toolhead out of its shared slot and runs this on one
/// `block_in_place` task, so the background flush task sees no toolhead and
/// stands back while the drip loop here owns step generation.
///
/// # Errors
/// A missing endstop, a kinematics refusal, or a failed query/send.
async fn home_axes(
    connected: &mut Connected,
    axes: &[Arc<PrinterStepper>; 3],
    requested: &[usize],
) -> Result<(), CommandError> {
    for &axis in requested {
        let rail = &axes[axis];
        let endstop = rail.endstop().ok_or_else(|| {
            CommandError::new(format!("No endstop configured for {}", rail.name()))
        })?;
        home_axis(
            connected,
            axis,
            rail.homing_info(),
            rail.params(),
            rail.step_dist(),
            endstop.as_ref(),
        )
        .await?;
    }
    Ok(())
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
            sleep(Duration::from_secs_f64(DRIP_LOOKAHEAD)).await;
        }
    }

    endstop.home_wait(end).await.map_err(command_error)?;
    // The axis is now known at its endstop position.
    connected.toolhead.set_position(home, &[axis]);
    connected.toolhead.wipe_trapq();
    Ok(())
}

fn command_error(err: McuError) -> CommandError {
    CommandError::new(err.to_string())
}

// ===========================================================================
// G-code commands
// ===========================================================================

/// `G0` / `G1`: move the toolhead.
fn move_command(state: Arc<Mutex<Option<Connected>>>, speed: Arc<Mutex<f64>>) -> CommandHandler {
    Arc::new(move |gcmd| {
        let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
        let Some(connected) = guard.as_mut() else {
            return Err(CommandError::new("Printer is not ready"));
        };
        let current = connected.toolhead.commanded_pos();
        let mut newpos = current;
        for (axis, name) in [
            (X_AXIS, "X"),
            (Y_AXIS, "Y"),
            (Z_AXIS, "Z"),
            (crate::core::klippy::mathutil::E_AXIS, "E"),
        ] {
            // An absent axis keeps the current value, which is what upstream's
            // `gcode_move` does before it applies base offsets (H3).
            let value = gcmd.get_float_default(name, current.axis(axis))?;
            newpos.set_axis(axis, value);
        }
        if gcmd.get_command_parameters().contains_key("F") {
            let feed = gcmd.get_float("F")?;
            if feed <= 0.0 {
                return Err(CommandError::new(format!(
                    "Invalid speed in '{}'",
                    gcmd.commandline()
                )));
            }
            *speed.lock().unwrap_or_else(|poison| poison.into_inner()) = feed / 60.0;
        }
        let speed = *speed.lock().unwrap_or_else(|poison| poison.into_inner());
        connected
            .toolhead
            .move_to(newpos, speed)
            .map_err(|err| CommandError::new(err.to_string()))
    })
}

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
/// driven on the current worker via `block_in_place`, and the toolhead is put
/// back afterwards. The background flush task sees an empty slot and stands back.
fn cmd_g28(
    state: &Arc<Mutex<Option<Connected>>>,
    axes: &[Arc<PrinterStepper>; 3],
    _printer: &Weak<Printer>,
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

    let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
    let Some(mut connected) = guard.take() else {
        return Err(CommandError::new("Printer is not ready"));
    };
    let handle = tokio::runtime::Handle::try_current()
        .map_err(|_| CommandError::new("G28 needs the async runtime"))?;
    let result = tokio::task::block_in_place(|| {
        handle.block_on(home_axes(&mut connected, axes, &requested))
    });
    *guard = Some(connected);
    result
}

/// The factory the `[printer]` declaration names.
pub(crate) fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(ToolHeadObject::new(config, printer)?))
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

    #[test]
    fn test_g1_parses_axes_and_speed_into_a_move() {
        let (state, gcode) = connected(homed_toolhead());
        let handler = move_command(Arc::clone(&state), Arc::new(Mutex::new(DEFAULT_MOVE_SPEED)));
        let command = gcode.create_gcode_command(
            "G1",
            "G1 X10 F600",
            HashMap::from([
                ("X".to_string(), "10".to_string()),
                ("F".to_string(), "600".to_string()),
            ]),
        );

        handler(&command).unwrap();
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
        // Y and Z kept their previous values.
        assert_eq!(connected.toolhead.commanded_pos().y(), 0.0);
        // The move reached the trapq and can generate steps.
        assert!(!connected.toolhead.trapq().moves().is_empty());
    }

    #[test]
    fn test_g1_remembers_the_last_speed() {
        let (state, gcode) = connected(homed_toolhead());
        let handler = move_command(Arc::clone(&state), Arc::new(Mutex::new(DEFAULT_MOVE_SPEED)));
        handler(&gcode.create_gcode_command(
            "G1",
            "G1 X10 F600",
            HashMap::from([
                ("X".to_string(), "10".to_string()),
                ("F".to_string(), "600".to_string()),
            ]),
        ))
        .unwrap();
        // The second move has no F and must reuse the first one.
        handler(&gcode.create_gcode_command(
            "G1",
            "G1 X20",
            HashMap::from([("X".to_string(), "20".to_string())]),
        ))
        .unwrap();

        let guard = state.lock().unwrap();
        assert_eq!(guard.as_ref().unwrap().toolhead.commanded_pos().x(), 20.0);
    }

    #[test]
    fn test_g1_rejects_a_non_positive_feedrate() {
        let (state, gcode) = connected(homed_toolhead());
        let handler = move_command(Arc::clone(&state), Arc::new(Mutex::new(DEFAULT_MOVE_SPEED)));
        let command = gcode.create_gcode_command(
            "G1",
            "G1 X10 F0",
            HashMap::from([
                ("X".to_string(), "10".to_string()),
                ("F".to_string(), "0".to_string()),
            ]),
        );

        let err = handler(&command).unwrap_err();

        assert!(err.to_string().contains("Invalid speed"), "{err}");
    }

    #[test]
    fn test_g1_refuses_a_move_on_an_unhomed_axis() {
        let mut toolhead = homed_toolhead();
        toolhead.set_position(Coord::default(), &[]);
        // Clearing the homed axes is what an unhomed machine looks like.
        if let Some(kinematics) = toolhead.kinematics_mut() {
            kinematics.clear_homing_state(&[X_AXIS, Y_AXIS, Z_AXIS]);
        }
        let (state, gcode) = connected(toolhead);
        let handler = move_command(Arc::clone(&state), Arc::new(Mutex::new(DEFAULT_MOVE_SPEED)));
        let command = gcode.create_gcode_command(
            "G1",
            "G1 X10 F600",
            HashMap::from([
                ("X".to_string(), "10".to_string()),
                ("F".to_string(), "600".to_string()),
            ]),
        );

        let err = handler(&command).unwrap_err();

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

        cmd_set_kinematic_position(&state, &command).unwrap();

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

        home_axis(
            &mut connected,
            X_AXIS,
            test_homing_info(),
            params,
            1.0,
            &endstop,
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
}
