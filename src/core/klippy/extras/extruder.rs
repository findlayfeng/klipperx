//! `[extruder]` / `[extruder1]` — the extruder: hotend, extrusion motion, the E axis.
//!
//! Upstream keeps this in `klippy/kinematics/extruder.py`: `PrinterExtruder`
//! owns the hotend heater and the extrusion motion (a separate trapq), and wraps
//! an `ExtruderStepper` around a `PrinterStepper` whose solver reads the
//! extrusion amount. `ToolHead.extra_axes` holds the extruders at position index
//! 3 and up.
//!
//! This port follows that split, with two adjustments for the loader:
//!
//! * `[extruder1]`, `[extruder2]`… have no factory of their own. Each is read
//!   through the `[extruder]` factory's wrapper (the same sibling mechanism
//!   `[stepper_z1]` uses) and registered under its section identifier.
//! * The extruder's `PrinterStepper` is registered as a hidden printer object
//!   (`extruder_stepper <name>`) so its `connect` builds the host `Stepper`,
//!   which the toolhead then takes.
//!
//! The heater runs through `heaters::setup_heater`: options and sensor are set
//! up there, the bang-bang/PID control loop is in place, and so are the
//! `[verify_heater]` check over it and the `pid_calibrate` object
//! (`PID_CALIBRATE`). `M109` waits for the target via
//! `PrinterHeaters::set_temperature(.., wait=true)`, the same loop
//! `PID_CALIBRATE` drives; `M104` sets the target and returns
//! (`set_temperature(.., wait=false)`).

use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::extruder_stepper::{
    cmd_set_extruder_rotation_distance, cmd_sync_extruder_motion,
};
use crate::core::klippy::extras::heaters::{self, Heater};
use crate::core::klippy::extras::stepper::PrinterStepper;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::extras::z_tilt::block_in_command;
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::McuStepper;
use crate::core::klippy::motion::extra::ExtraAxis;
use crate::core::klippy::motion::itersolve::{extruder_active_flags, extruder_position_fn, Axis};
use crate::core::klippy::motion::kinematics::MoveContext;
use crate::core::klippy::motion::plan::Move;
use crate::core::klippy::motion::pressure_advance::ExtruderPressureAdvance;
use crate::core::klippy::motion::queuing::MotionQueuing;
use crate::core::klippy::motion::stepper::Stepper;
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};

// `extruder` (and its numbered siblings, read by this factory) and the
// `[extruder_stepper <name>]` extra steppers for one filament.
section!("extruder", order = 20, load = load_config);

/// One configured `[extruder]` / `[extruder1]`.
pub struct PrinterExtruder {
    /// The section identifier (`extruder`, `extruder1`).
    name: String,
    /// The extruder index (`0` for `[extruder]`), used for the `T` g-code ids.
    index: usize,
    /// The hotend heater (C1b stub).
    heater: Arc<Heater>,
    /// The stepper, when the section has motor options.
    stepper: Option<Arc<PrinterStepper>>,
    /// The filament's cross section, for the extrusion checks.
    filament_area: f64,
    /// The largest `axes_r[E]` a move may carry.
    max_extrude_ratio: f64,
    /// The default max extrude ratio, for the extrude-only speed defaults.
    def_max_extrude_ratio: f64,
    /// The nozzle diameter, for the "tiny extrusion" allowance.
    nozzle_diameter: f64,
    /// `max_extrude_only_velocity`, or `None` to default from the toolhead.
    max_e_velocity_opt: Option<f64>,
    /// `max_extrude_only_accel`, or `None` to default from the toolhead.
    max_e_accel_opt: Option<f64>,
    /// `max_extrude_only_distance`.
    max_e_dist: f64,
    /// `instantaneous_corner_velocity`, for the junction limit.
    instant_corner_v: f64,
    /// The extrusion limits resolved at connect.
    max_e_velocity: Mutex<f64>,
    max_e_accel: Mutex<f64>,
    /// The extruder's own trapq id, set by the toolhead when it attaches this
    /// axis. `None` before the machine is up.
    trapq: Mutex<Option<usize>>,
    /// The last extruded position, for `last_position`/status.
    last_position: Mutex<f64>,
    /// `motion_queue`: the extruder this stepper is bound to via
    /// `SYNC_EXTRUDER_MOTION`. `Arc` so the command handler shares the slot.
    motion_queue: Arc<Mutex<Option<String>>>,
    /// Pressure advance, for `SET_PRESSURE_ADVANCE` and status, shared with the
    /// stepper's solver: the position function adds
    /// `pressure_advance * velocity` and smooths it over `smooth_time`
    /// (`chelper/kin_extruder.c`).
    ///
    /// `Arc` so the command handler and the solver own handles to the same
    /// parameters.
    pressure_advance: Arc<ExtruderPressureAdvance>,
    /// The machine, to find the toolhead's limits at connect.
    printer: std::sync::Weak<Printer>,
}

impl PrinterExtruder {
    /// Build one extruder from its section.
    ///
    /// # Errors
    /// A missing or invalid option, an unknown sensor, or a pin the `pins`
    /// layer refuses.
    pub fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        index: usize,
    ) -> Result<Self, ConfigError> {
        let name = config
            .section()
            .sub
            .clone()
            .unwrap_or_else(|| config.section().id.clone());

        let heaters = heaters::ensure(printer)?;
        let gcode_id = format!("T{index}");
        let heater = heaters.setup_heater(config, printer, Some(&gcode_id))?;

        let nozzle_diameter =
            config.get_float_bounded("nozzle_diameter", None, None, None, Some(0.0), None)?;
        let filament_diameter = config.get_float_bounded(
            "filament_diameter",
            None,
            Some(nozzle_diameter),
            None,
            None,
            None,
        )?;
        let filament_area = std::f64::consts::PI * (filament_diameter * 0.5).powi(2);
        let def_max_cross_section = 4.0 * nozzle_diameter.powi(2);
        let def_max_extrude_ratio = def_max_cross_section / filament_area;
        let max_cross_section = config.get_float_bounded(
            "max_extrude_cross_section",
            Some(def_max_cross_section),
            None,
            None,
            Some(0.0),
            None,
        )?;
        let max_extrude_ratio = max_cross_section / filament_area;

        let max_e_velocity_opt = config.get_optional_float("max_extrude_only_velocity")?;
        let max_e_accel_opt = config.get_optional_float("max_extrude_only_accel")?;
        let max_e_dist = config.get_float_bounded(
            "max_extrude_only_distance",
            Some(50.0),
            Some(0.0),
            None,
            None,
            None,
        )?;
        let instant_corner_v = config.get_float_bounded(
            "instantaneous_corner_velocity",
            Some(1.0),
            Some(0.0),
            None,
            None,
            None,
        )?;
        let pressure_advance_value =
            config.get_float_bounded("pressure_advance", Some(0.0), Some(0.0), None, None, None)?;
        let smooth_time = config.get_float_bounded(
            "pressure_advance_smooth_time",
            Some(0.040),
            None,
            Some(0.200),
            Some(0.0),
            None,
        )?;

        // Upstream only builds the stepper when the section names one of the
        // motor options (`kinematics/extruder.py:172-177`).
        let pressure_advance = ExtruderPressureAdvance::new(pressure_advance_value, smooth_time);
        let stepper = if config.section().has("step_pin")
            || config.section().has("dir_pin")
            || config.section().has("rotation_distance")
        {
            let stepper = Arc::new(PrinterStepper::new(config, printer, Axis::X, false)?);
            stepper.setup_itersolve(extruder_position_fn(), extruder_active_flags());
            // The solver smooths its position with the parameters
            // `SET_PRESSURE_ADVANCE` writes (`kin_extruder.c:124-136`).
            stepper.setup_pressure_advance(Arc::clone(&pressure_advance));
            // A hidden object so the stepper's own `connect` runs and builds the
            // host `Stepper` the toolhead will take.
            printer.add_object(
                &format!("extruder_stepper {name}"),
                Arc::clone(&stepper) as Arc<dyn PrinterObject>,
            )?;
            Some(stepper)
        } else {
            None
        };

        let extruder = Self {
            name: name.clone(),
            index,
            heater,
            stepper,
            filament_area,
            max_extrude_ratio,
            def_max_extrude_ratio,
            nozzle_diameter,
            max_e_velocity_opt,
            max_e_accel_opt,
            max_e_dist,
            instant_corner_v,
            max_e_velocity: Mutex::new(0.0),
            max_e_accel: Mutex::new(0.0),
            trapq: Mutex::new(None),
            last_position: Mutex::new(0.0),
            motion_queue: Arc::new(Mutex::new(None)),
            pressure_advance,
            printer: Arc::downgrade(printer),
        };
        extruder.register_commands(printer)?;
        // Register the extruder's own queue with `motion_report`, which streams
        // it under this section's name. Upstream registers it in
        // `klippy:connect` (`motion_report.py:173-179`); the mux table here is
        // filled from the load path, so the instance is registered by name and
        // the trapq id resolved per batch.
        crate::core::klippy::extras::motion_report::MotionReport::ensure(printer)
            .register_extruder_trapq(&name)?;
        Ok(extruder)
    }

    /// The section identifier (`extruder`, `extruder1`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The hotend heater.
    pub fn heater(&self) -> &Arc<Heater> {
        &self.heater
    }

    /// The wrapped stepper, when the section names motor options.
    pub fn printer_stepper(&self) -> Option<&Arc<PrinterStepper>> {
        self.stepper.as_ref()
    }

    /// Take the host stepper the toolhead will drive.
    pub fn take_stepper(&self) -> Option<Stepper> {
        self.stepper
            .as_ref()
            .and_then(|stepper| stepper.take_stepper())
    }

    /// The firmware stepper, to send its steps.
    pub fn mcu_stepper(&self) -> Option<Arc<McuStepper>> {
        self.stepper
            .as_ref()
            .map(|stepper| Arc::clone(stepper.mcu_stepper()))
    }

    /// Point this extruder at its trapq (the toolhead does this when attaching).
    pub fn set_trapq(&self, trapq: usize) {
        *self.trapq.lock().unwrap_or_else(|p| p.into_inner()) = Some(trapq);
    }

    /// This extruder's own trapq, once the toolhead has attached it
    /// (`MCU_stepper.get_trapq`). `None` before connect.
    pub fn trapq(&self) -> Option<usize> {
        *Self::lock(&self.trapq)
    }

    /// Register the extruder's g-code commands
    /// (`kinematics/extruder.py:34-50`, `:180-190`).
    fn register_commands(&self, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");

        // M104/M109 belong to the primary extruder, as upstream registers them
        // only when the name is `extruder` (`kinematics/extruder.py:185-188`).
        if self.name == "extruder" {
            for (name, wait) in [("M104", false), ("M109", true)] {
                let printer = Arc::downgrade(printer);
                let handler: CommandHandler = Arc::new(move |gcmd: &GcodeCommand| {
                    let printer = printer.clone();
                    Box::pin(async move { cmd_set_temperature(&printer, gcmd, wait).await })
                });
                gcode
                    .register_command_with_params(
                        name,
                        handler,
                        Some("Set extruder temperature"),
                        M104_M109_PARAMS,
                        false,
                    )
                    .map_err(ConfigError::new)?;
            }
        }

        // SET_PRESSURE_ADVANCE is a per-extruder mux command; the solver reads
        // the new parameters from the print time the extruder has planned up
        // to (`extruder.py:67-88`).
        {
            let pa = Arc::clone(&self.pressure_advance);
            let printer = Arc::downgrade(printer);
            let handler: CommandHandler =
                sync(move |gcmd: &GcodeCommand| set_pressure_advance(gcmd, &pa, &printer));
            gcode
                .register_mux_command_with_params(
                    "SET_PRESSURE_ADVANCE",
                    "EXTRUDER",
                    Some(&self.name),
                    handler,
                    Some("Set pressure advance parameters"),
                    SET_PRESSURE_ADVANCE_PARAMS,
                )
                .map_err(ConfigError::new)?;
        }

        // …and an `EXTRUDER` **default** for a line that leaves the word out —
        // registered only by the extruder literally named `extruder`, and
        // forwarding to whichever extruder the toolhead says is active
        // (`kinematics/extruder.py:29-31, 90-97`). Without it a bare
        // `SET_PRESSURE_ADVANCE ADVANCE=.002` fails on the missing key.
        if self.name == "extruder" {
            let printer = Arc::downgrade(printer);
            let handler: CommandHandler =
                sync(move |gcmd: &GcodeCommand| forward_pressure_advance(&printer, gcmd));
            gcode
                .register_mux_command_with_params(
                    "SET_PRESSURE_ADVANCE",
                    "EXTRUDER",
                    None,
                    handler,
                    Some("Set pressure advance parameters"),
                    SET_PRESSURE_ADVANCE_PARAMS,
                )
                .map_err(ConfigError::new)?;
        }

        // ACTIVATE_EXTRUDER: record the active extruder on the toolhead, then
        // announce the switch (`kinematics/extruder.py:281-289`). The event
        // carries no payload upstream and is what re-anchors the consumers:
        // `gcode_move` resets the E position and the extrude factor, and
        // `print_stats` re-bases its filament usage. A missing toolhead means
        // nothing was switched, so nothing is announced.
        {
            let name = self.name.clone();
            let printer = Arc::downgrade(printer);
            let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
                let Some(printer) = printer.upgrade() else {
                    return Err(CommandError::new("printer is gone"));
                };
                if let Some(toolhead) = printer.lookup_object_as::<ToolHeadObject>("toolhead") {
                    toolhead.set_active_extruder(&name);
                    printer.send_event(&KlippyEvent::ExtruderActivateExtruder);
                }
                gcmd.respond_info(&format!("Activating extruder {name}"));
                let _ = gcmd;
                Ok(())
            });
            gcode
                .register_mux_command(
                    "ACTIVATE_EXTRUDER",
                    "EXTRUDER",
                    Some(&self.name),
                    handler,
                    Some("Change the active extruder"),
                )
                .map_err(ConfigError::new)?;
        }

        // The primary extruder's stepper registers the two motion-sync commands
        // its `[extruder_stepper]` siblings register, for the name `extruder`
        // (`kinematics/extruder.py:34-50`).
        if let Some(stepper) = &self.stepper {
            let stepper = Arc::clone(stepper);
            let name = self.name.clone();
            let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
                cmd_set_extruder_rotation_distance(gcmd, &name, &stepper)
            });
            gcode
                .register_mux_command_with_params(
                    "SET_EXTRUDER_ROTATION_DISTANCE",
                    "EXTRUDER",
                    Some(&self.name),
                    handler,
                    Some("Set extruder rotation distance"),
                    // The mux key `EXTRUDER` is prepended by the registration.
                    &["DISTANCE"],
                )
                .map_err(ConfigError::new)?;

            let queue = Arc::clone(&self.motion_queue);
            let name = self.name.clone();
            let printer = Arc::downgrade(printer);
            let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
                cmd_sync_extruder_motion(gcmd, &name, &printer, &queue)
            });
            gcode
                .register_mux_command_with_params(
                    "SYNC_EXTRUDER_MOTION",
                    "EXTRUDER",
                    Some(&self.name),
                    handler,
                    Some("Set extruder stepper motion queue"),
                    // The mux key `EXTRUDER` is prepended by the registration.
                    &["MOTION_QUEUE"],
                )
                .map_err(ConfigError::new)?;
        }

        Ok(())
    }

    fn lock<T>(slot: &Mutex<T>) -> MutexGuard<'_, T> {
        slot.lock().unwrap_or_else(|p| p.into_inner())
    }
}

/// The words `SET_PRESSURE_ADVANCE` reads (`kinematics/extruder.py:93-97`):
/// shared by the per-extruder value and the `EXTRUDER` default, which run the
/// same body against a different extruder.
const SET_PRESSURE_ADVANCE_PARAMS: &[&str] = &["ADVANCE", "SMOOTH_TIME"];

/// `SET_PRESSURE_ADVANCE`'s body: validate both values, apply them (see
/// [`apply_pressure_advance`]) and report them (upstream
/// `extruder_stepper.cmd_SET_PRESSURE_ADVANCE`, `kinematics/extruder.py:93-107`;
/// the bounds are its `minval=0.` on `ADVANCE` and `minval=0., maxval=.200` on
/// `SMOOTH_TIME`).
fn set_pressure_advance(
    gcmd: &GcodeCommand,
    pressure_advance: &Arc<ExtruderPressureAdvance>,
    printer: &std::sync::Weak<Printer>,
) -> Result<(), CommandError> {
    let (advance, smooth_time) = pressure_advance.values();
    let value = gcmd.get_float_default("ADVANCE", advance)?;
    let smooth = gcmd.get_float_default("SMOOTH_TIME", smooth_time)?;
    if value < 0.0 {
        return Err(CommandError::new(format!(
            "ADVANCE must be at least 0.0, got {value}"
        )));
    }
    if !(0.0..=0.200).contains(&smooth) {
        return Err(CommandError::new(format!(
            "SMOOTH_TIME must be between 0.0 and 0.2, got {smooth}"
        )));
    }
    apply_pressure_advance(pressure_advance, printer, value, smooth)?;
    gcmd.respond_info(&format!(
        "pressure_advance: {value:.6}\npressure_advance_smooth_time: {smooth:.6}",
    ));
    Ok(())
}

/// Put already-validated `SET_PRESSURE_ADVANCE` values in force. Every extruder
/// stepper runs this same body — the primary `[extruder]` and each
/// `[extruder_stepper <name>]` (upstream's `_set_pressure_advance`,
/// `kinematics/extruder.py:67-88`) — against its own parameters.
///
/// The new value is active from the print time the extruder has already
/// planned up to: the command registers a lookahead callback, which fires with
/// exactly that time (`extruder.py:83-86` → `toolhead.py:526-531`) — so segments
/// the step generator has stepped over keep the value they were stepped with.
///
/// A changed **smooth time** is the exception: the smoothing filter's width is
/// part of how a segment is stepped, so upstream flushes step generation first
/// and applies the new parameters from time zero (`extruder.py:75-83`). The
/// values upstream compares are the *effective* smooth times — either side is
/// zero when its pressure advance is zero (`:67-74`).
pub(crate) fn apply_pressure_advance(
    pressure_advance: &Arc<ExtruderPressureAdvance>,
    printer: &std::sync::Weak<Printer>,
    value: f64,
    smooth: f64,
) -> Result<(), CommandError> {
    // The values in force now, read before [`ExtruderPressureAdvance::set_reported`]
    // overwrites them: they date the old side of the comparison.
    let (advance, smooth_time) = pressure_advance.values();
    let toolhead = printer
        .upgrade()
        .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>("toolhead"));
    // Upstream's two "effective" smooth times (`kinematics/extruder.py:67-74`).
    let old_smooth_time = if advance == 0.0 { 0.0 } else { smooth_time };
    let new_smooth_time = if value == 0.0 { 0.0 } else { smooth };
    let smooth_time_changed = new_smooth_time != old_smooth_time;
    // "Need full kinematic flush to change the smooth time"
    // (`kinematics/extruder.py:79-80`): the width of the smoothing filter is
    // part of how a segment is stepped, so every segment planned with the old
    // one has to be generated before the new one starts. Nothing is queued
    // before connect, so there is nothing to flush there.
    if smooth_time_changed {
        if let Some(toolhead) = toolhead.as_ref().filter(|toolhead| toolhead.is_connected()) {
            block_in_command(toolhead.flush_step_generation())?;
        }
    }
    // With the plan generated, upstream dates the new value at time zero
    // (`espa(sk, 0., …)`, `:81`); without a smooth time change it registers a
    // lookahead callback that writes it when the look-ahead flushes
    // (`:83-86` → `toolhead.py:526-531`).
    //
    // The reported values are the command's, immediately either way: upstream's
    // `self.pressure_advance = pressure_advance` runs in the command body
    // (`:87-88`), not when the scheduled change lands.
    pressure_advance.set_reported(value, smooth);
    if smooth_time_changed {
        pressure_advance.set_from(0.0, value, smooth);
    } else if let Some(toolhead) = &toolhead {
        // The callback fires with the end time of the move it was parked
        // against — or at once, with the last move time, when the look-ahead
        // is empty — so nothing the step generator already stepped over moves.
        let pressure_advance = Arc::clone(pressure_advance);
        toolhead.register_lookahead_callback(Box::new(move |print_time| {
            pressure_advance.set_from(print_time, value, smooth);
        }));
    } else {
        // No toolhead: nothing has been planned, so there is no flush point to
        // wait for.
        pressure_advance.set_from(0.0, value, smooth);
    }
    // The extruder stepper's step generation window is half its smooth time, so
    // only a changed smooth time changes how far generated steps are still open
    // to rewriting: rescan and update the flush delay
    // (`motion_queuing.check_step_generation_scan_windows`,
    // `kinematics/extruder.py:82`; the detach/bind scan at `:57,66` has no
    // counterpart here yet).
    if smooth_time_changed {
        if let Some(toolhead) = &toolhead {
            toolhead.recompute_scan_windows();
        }
    }
    Ok(())
}

/// The `EXTRUDER` default: apply to the extruder the toolhead says is active
/// (`kinematics/extruder.py:90-97`, whose stepper checks are ours' one check
/// — a `[extruder]` here always has its stepper when it has motor options).
fn forward_pressure_advance(
    printer: &std::sync::Weak<Printer>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let active = |message: &str| CommandError::new(message.to_string());
    let printer = printer
        .upgrade()
        .ok_or_else(|| active("Active extruder does not have a stepper"))?;
    let toolhead = printer
        .lookup_object_as::<ToolHeadObject>("toolhead")
        .ok_or_else(|| active("Active extruder does not have a stepper"))?;
    let extruder = printer
        .lookup_object_as::<PrinterExtruder>(&toolhead.active_extruder())
        .ok_or_else(|| active("Active extruder does not have a stepper"))?;
    if extruder.stepper.is_none() {
        return Err(active("Active extruder does not have a stepper"));
    }
    set_pressure_advance(gcmd, &extruder.pressure_advance, &Arc::downgrade(&printer))
}

impl PrinterObject for PrinterExtruder {
    fn get_status(&self, _eventtime: f64) -> Value {
        let mut status = self.heater.get_status();
        if let Value::Object(map) = &mut status {
            let (pressure_advance, smooth_time) = self.pressure_advance.values();
            map.insert("pressure_advance".to_string(), json!(pressure_advance));
            map.insert("smooth_time".to_string(), json!(smooth_time));
            map.insert("can_extrude".to_string(), json!(self.heater.can_extrude()));
            map.insert(
                "motion_queue".to_string(),
                json!(Self::lock(&self.motion_queue).clone()),
            );
        }
        status
    }

    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            // Resolve the extrude-only limits now that the toolhead object
            // exists (all sections are loaded before anything connects).
            let (max_velocity, max_accel) = self
                .printer
                .upgrade()
                .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>("toolhead"))
                .map(|toolhead| (toolhead.max_velocity(), toolhead.max_accel()))
                .unwrap_or((0.0, 0.0));
            *Self::lock(&self.max_e_velocity) = self
                .max_e_velocity_opt
                .unwrap_or(max_velocity * self.def_max_extrude_ratio);
            *Self::lock(&self.max_e_accel) = self
                .max_e_accel_opt
                .unwrap_or(max_accel * self.def_max_extrude_ratio);
            Ok(())
        })
    }
}

impl ExtraAxis for PrinterExtruder {
    fn name(&self) -> &str {
        &self.name
    }

    fn check_move(&self, ctx: &mut MoveContext<'_>, ea_index: usize) -> Result<(), CommandError> {
        if !self.heater.can_extrude() {
            return Err(ctx.move_error(
                "Extrude below minimum temp\nSee the 'min_extrude_temp' config option for details",
            ));
        }
        let axis_r = ctx.axes_r()[ea_index];
        let axis_d = ctx.axes_d()[ea_index];
        let axes_d = ctx.axes_d();
        if (axes_d[0] == 0.0 && axes_d[1] == 0.0) || axis_r < 0.0 {
            // Extrude-only or retraction: limit the speed and acceleration.
            if axis_d.abs() > self.max_e_dist {
                return Err(ctx.move_error(&format!(
                    "Extrude only move too long ({:.3}mm vs {:.3}mm)\n\
                     See the 'max_extrude_only_distance' config option for details",
                    axis_d, self.max_e_dist
                )));
            }
            let inv_extrude_r = 1.0 / axis_r.abs();
            ctx.limit_speed(
                *Self::lock(&self.max_e_velocity) * inv_extrude_r,
                *Self::lock(&self.max_e_accel) * inv_extrude_r,
            );
        } else if axis_r > self.max_extrude_ratio {
            if axis_d <= self.nozzle_diameter * self.max_extrude_ratio {
                return Ok(());
            }
            let area = axis_r * self.filament_area;
            return Err(ctx.move_error(&format!(
                "Move exceeds maximum extrusion ({:.3}mm^2 vs {:.3}mm^2)\n\
                 See the 'max_extrude_cross_section' config option for details",
                area,
                self.max_extrude_ratio * self.filament_area
            )));
        }
        Ok(())
    }

    fn calc_junction(&self, prev: &Move, cur: &Move, ea_index: usize) -> f64 {
        let diff_r = cur.axes_r[ea_index] - prev.axes_r[ea_index];
        if diff_r != 0.0 {
            (self.instant_corner_v / diff_r.abs()).powi(2)
        } else {
            cur.max_cruise_v2
        }
    }

    fn process_move(
        &self,
        queuing: &mut MotionQueuing,
        print_time: f64,
        move_: &Move,
        ea_index: usize,
    ) {
        let Some(trapq) = *Self::lock(&self.trapq) else {
            return;
        };
        let axis_r = move_.axes_r[ea_index];
        let can_pressure_advance =
            axis_r > 0.0 && (move_.axes_d[0] != 0.0 || move_.axes_d[1] != 0.0);
        // The extruder trapq's x is the extrusion amount and y the pressure
        // advance flag (`kinematics/extruder.py:221-232`).
        queuing.trapq_mut(trapq).append(
            print_time,
            move_.accel_t,
            move_.cruise_t,
            move_.decel_t,
            crate::core::klippy::mathutil::Xyz::new(move_.start_axis(ea_index), 0.0, 0.0),
            crate::core::klippy::mathutil::Xyz::new(1.0, can_pressure_advance as u8 as f64, 0.0),
            move_.start_v * axis_r,
            move_.cruise_v * axis_r,
            move_.accel * axis_r,
        );
        *Self::lock(&self.last_position) = move_.end_axis(ea_index);
    }

    fn find_past_position(&self, print_time: f64) -> f64 {
        // Upstream: `mcu_pos = stepper.get_past_mcu_position(print_time)` then
        // `stepper.mcu_to_commanded_position(mcu_pos)`
        // (`kinematics/extruder.py:47-49`). The host stepper belongs to the
        // toolhead after connect, so both halves are reached through it.
        let past = self
            .printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>("toolhead"))
            .zip(self.stepper.as_ref())
            .and_then(|(toolhead, stepper)| {
                stepper
                    .get_past_mcu_position(&toolhead, print_time)
                    .map(|mcu_pos| stepper.mcu_to_commanded_position(mcu_pos))
            });
        // Before connect there is no compressor to ask: the planned position is
        // the only answer, and it is what `add_extra_axis` seeds the axis slot
        // with (nothing has been generated yet, so it holds either way).
        past.unwrap_or_else(|| *Self::lock(&self.last_position))
    }

    fn get_status(&self) -> Value {
        PrinterObject::get_status(self, 0.0)
    }
}

impl std::fmt::Debug for PrinterExtruder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterExtruder")
            .field("name", &self.name)
            .field("index", &self.index)
            .finish_non_exhaustive()
    }
}

/// `M104`/`M109`: set the (optionally `T`-indexed) extruder temperature.
/// The words `M104`/`M109` read (`kinematics/extruder.py:180-190`). Both names
/// share `cmd_set_temperature`; `wait=true` (M109) waits for the heater to
/// reach the target via `PrinterHeaters::set_temperature`, `wait=false`
/// (M104) sets the target and returns.
const M104_M109_PARAMS: &[&str] = &["S", "T"];

async fn cmd_set_temperature(
    printer: &std::sync::Weak<Printer>,
    gcmd: &GcodeCommand,
    wait: bool,
) -> Result<(), CommandError> {
    let temp = gcmd.get_float_default("S", 0.0)?;
    let index = gcmd.get_int_default("T", 0)?;
    let Some(printer) = printer.upgrade() else {
        return Err(CommandError::new("printer is gone"));
    };
    let section = if index == 0 {
        "extruder".to_string()
    } else {
        format!("extruder{index}")
    };
    let extruder = printer
        .lookup_object_as::<PrinterExtruder>(&section)
        .ok_or_else(|| CommandError::new("Extruder not configured"))?;
    let pheaters = printer
        .lookup_object_as::<heaters::PrinterHeaters>(heaters::HEATERS_OBJECT)
        .ok_or_else(|| CommandError::new("Heaters not configured"))?;
    pheaters
        .set_temperature(extruder.heater(), temp, wait)
        .await?;
    Ok(())
}

/// The `[extruder]` factory: build the primary and its numbered siblings.
///
/// Upstream's `kinematics.extruder.add_printer_objects` loops `extruder`,
/// `extruder1`… (`kinematics/extruder.py:310-319`); here the numbered sections
/// are read through the primary's wrapper and registered by hand.
pub(crate) fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let primary = Arc::new(PrinterExtruder::new(config, printer, 0)?);
    for index in 1..99 {
        let identifier = format!("extruder{index}");
        let Some(sibling) = config.sibling(&identifier) else {
            break;
        };
        let extruder = Arc::new(PrinterExtruder::new(&sibling, printer, index)?);
        printer.add_object(&identifier, extruder as Arc<dyn PrinterObject>)?;
    }
    Ok(primary)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::interface::devices::responder_mcu::ResponderMcu;
    use crate::core::klippy::mathutil::{Coord, E_AXIS};
    use crate::core::klippy::motion::plan::MoveLimits;
    use crate::core::klippy::motion::Move;
    use crate::core::klippy::printer::PrinterState;
    use crate::core::klippy::reactor::{ManualReactor, TokioReactor};
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::time::Duration;

    /// A cartesian printer config with one extruder.
    fn extruder_config(extra: &str) -> String {
        format!(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_y]\nstep_pin: PA2\ndir_pin: PA3\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z]\nstep_pin: PA4\ndir_pin: PA5\nrotation_distance: 8\nmicrosteps: 16\nposition_max: 200\n\
             [extruder]\nstep_pin: PA6\ndir_pin: PA7\nrotation_distance: 33.5\nmicrosteps: 16\n\
             nozzle_diameter: 0.4\nfilament_diameter: 1.75\nheater_pin: PB0\n\
             sensor_type: temperature_mcu\ncontrol: pid\npid_Kp: 1\npid_Ki: 0.1\npid_Kd: 10\n\
             min_temp: 0\nmax_temp: 250\nmin_extrude_temp: 0\n{extra}\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n"
        )
    }

    fn load_ok(text: &str) -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(text).expect("the config parses");
        printer.load_config(&config).expect("the config loads");
        printer
    }

    /// `extruder_config` with a `kinematics: none` toolhead: nothing is planned,
    /// so a flush only runs the registered flush callbacks — the fixture a test
    /// that counts flushes needs.
    fn extruder_config_none_kinematics() -> String {
        extruder_config("").replace("kinematics: cartesian", "kinematics: none")
    }

    /// Run one g-code line under an overall bound: an await a test hangs on is
    /// a CI hang.
    async fn run_bounded(gcode: &GCodeDispatch, script: &str) {
        tokio::time::timeout(Duration::from_secs(30), gcode.run_script(script))
            .await
            .unwrap_or_else(|_| panic!("{script} did not finish within the bound"))
            .unwrap_or_else(|error| panic!("{script}: {error}"));
    }

    /// `commands.test` sends `SET_PRESSURE_ADVANCE ADVANCE=.002
    /// SMOOTH_TIME=.001` with **no** `EXTRUDER=` word. Upstream answers that
    /// through the `EXTRUDER` default — registered only by the extruder
    /// literally named `extruder` — which forwards to whichever extruder the
    /// toolhead says is active (`kinematics/extruder.py:29-31, 90-97`).
    #[test]
    fn test_set_pressure_advance_without_a_key_reaches_the_active_extruder() {
        let printer = load_ok(&extruder_config(""));
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        gcode
            .run_script_sync("SET_PRESSURE_ADVANCE ADVANCE=.002 SMOOTH_TIME=.001")
            .unwrap();

        let extruder = printer
            .lookup_object_as::<PrinterExtruder>("extruder")
            .expect("[extruder] is registered");
        let status = PrinterObject::get_status(&*extruder, 0.0);
        assert_eq!(status["pressure_advance"], 0.002);
        assert_eq!(status["smooth_time"], 0.001);
    }

    /// A **changed smooth time** flushes step generation before the new value
    /// is recorded; a value-only change does not (`kinematics/extruder.py:79-86`:
    /// "Need full kinematic flush to change the smooth time").
    ///
    /// The spy is a flush callback ([`ToolHeadObject::register_flush_callback`]),
    /// which fires on every step generation — including one that produces no
    /// steps — so with `kinematics: none` the count is exactly the number of
    /// flushes the command ran.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_a_changed_smooth_time_flushes_step_generation_first() {
        let printer = load_ok(&extruder_config_none_kinematics());
        printer.send_event(&KlippyEvent::KlippyReady);
        let toolhead = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");
        toolhead.connect().await.expect("the toolhead connects");
        let flushes = Arc::new(AtomicUsize::new(0));
        toolhead.register_flush_callback(Box::new({
            let flushes = Arc::clone(&flushes);
            move |_| {
                flushes.fetch_add(1, Ordering::SeqCst);
            }
        }));
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        // Enabling pressure advance with a smooth time is a smooth time change
        // (the configured `ADVANCE` is 0, so it started out as 0).
        gcode
            .run_script("SET_PRESSURE_ADVANCE ADVANCE=.002 SMOOTH_TIME=.04")
            .await
            .expect("the line applies");
        assert_eq!(
            flushes.load(Ordering::SeqCst),
            1,
            "a changed smooth time flushes the plan"
        );

        // Only `ADVANCE` moves now: the smooth time is unchanged, so the new
        // value rides a lookahead callback and nothing is flushed.
        gcode
            .run_script("SET_PRESSURE_ADVANCE ADVANCE=.003 SMOOTH_TIME=.04")
            .await
            .expect("the line applies");
        assert_eq!(
            flushes.load(Ordering::SeqCst),
            1,
            "a value-only change flushes nothing"
        );

        // A different smooth time is a different filter width: flush first.
        gcode
            .run_script("SET_PRESSURE_ADVANCE ADVANCE=.002 SMOOTH_TIME=.01")
            .await
            .expect("the line applies");
        assert_eq!(
            flushes.load(Ordering::SeqCst),
            2,
            "a changed smooth time flushes the plan exactly once"
        );

        let extruder = printer
            .lookup_object_as::<PrinterExtruder>("extruder")
            .expect("[extruder] is registered");
        let status = PrinterObject::get_status(&*extruder, 0.0);
        assert_eq!(status["smooth_time"], 0.01);
    }

    /// Upstream compares the **effective** smooth times: with pressure advance
    /// 0 the smooth time counts as 0 (`kinematics/extruder.py:67-74`), so
    /// turning `ADVANCE` off flushes even though the `SMOOTH_TIME` word did not
    /// move — and turning it back on flushes again.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_switching_pressure_advance_off_flushes_the_smoothing_away() {
        let printer = load_ok(&extruder_config_none_kinematics());
        printer.send_event(&KlippyEvent::KlippyReady);
        let toolhead = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");
        toolhead.connect().await.expect("the toolhead connects");
        let flushes = Arc::new(AtomicUsize::new(0));
        toolhead.register_flush_callback(Box::new({
            let flushes = Arc::clone(&flushes);
            move |_| {
                flushes.fetch_add(1, Ordering::SeqCst);
            }
        }));
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        for (script, expected) in [
            ("SET_PRESSURE_ADVANCE ADVANCE=0 SMOOTH_TIME=.04", 0),
            ("SET_PRESSURE_ADVANCE ADVANCE=0 SMOOTH_TIME=.04", 0),
            ("SET_PRESSURE_ADVANCE ADVANCE=.05 SMOOTH_TIME=.04", 1),
            ("SET_PRESSURE_ADVANCE ADVANCE=.05 SMOOTH_TIME=.04", 1),
            ("SET_PRESSURE_ADVANCE ADVANCE=0 SMOOTH_TIME=.04", 2),
        ] {
            gcode.run_script(script).await.expect("the line applies");
            assert_eq!(flushes.load(Ordering::SeqCst), expected, "after {script}");
        }
    }

    /// A **value-only** change is dated by the lookahead callback, not by the
    /// command (`kinematics/extruder.py:83-86` → `klippy/toolhead.py:526-531`):
    /// with a move still queued the schedule the solver reads is untouched when
    /// the command returns, and the new value lands at that move's end time
    /// when the look-ahead flushes.
    ///
    /// Positions are a pure function of that schedule (see the
    /// `pressure_advance` module docs), so an unchanged schedule is an
    /// unchanged position for every segment — including every segment the step
    /// generator has already stepped over.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_a_value_only_change_waits_for_the_lookahead_flush() {
        let printer = load_ok(&extruder_config_none_kinematics());
        printer.send_event(&KlippyEvent::KlippyReady);
        let toolhead = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");
        toolhead.connect().await.expect("the toolhead connects");
        let extruder = printer
            .lookup_object_as::<PrinterExtruder>("extruder")
            .expect("[extruder] is registered");
        // Pressure advance on, with a smooth time: the fixture a value-only
        // change then has to leave alone.
        extruder.pressure_advance.set(0.0, 0.05, 0.04);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        // One move is queued when the command runs, so the callback has a move
        // to park on.
        toolhead
            .move_to(Coord::new(10.0, 0.0, 0.0, 0.0), 100.0)
            .expect("the move is planned");
        let before = extruder.pressure_advance.schedule();

        run_bounded(&gcode, "SET_PRESSURE_ADVANCE ADVANCE=.02").await;

        assert_eq!(
            extruder.pressure_advance.schedule(),
            before,
            "nothing the solver reads changes before the flush point"
        );
        // What the command *reports* moves at once (`extruder.py:87-88`).
        assert_eq!(extruder.pressure_advance.values(), (0.02, 0.04));

        // Flushing the look-ahead is what fires the callback.
        let flush_time = toolhead.get_last_move_time();
        let schedule = extruder.pressure_advance.schedule();
        assert_eq!(schedule.len(), before.len() + 1, "the entry is recorded");
        let landed = schedule.last().expect("the new entry is the last one");
        assert!(
            (landed.print_time - flush_time).abs() < 1e-9,
            "dated at the queued move's end time: {} vs {flush_time}",
            landed.print_time
        );
        assert_eq!((landed.pressure_advance, landed.smooth_time), (0.02, 0.04));
        // The solver's lookup takes the last entry at or before a segment's own
        // print time (`pa_move_integrate`): the segment generated at the flush
        // time and every later one reads the new value, the ones before it keep
        // the old.
        let in_force = |print_time: f64| {
            schedule
                .iter()
                .rev()
                .find(|entry| entry.print_time <= print_time)
                .map(|entry| entry.pressure_advance)
        };
        assert_eq!(in_force(flush_time), Some(0.02));
        assert_eq!(in_force(flush_time - 1e-6), Some(0.05));
    }

    /// With the look-ahead empty, `register_lookahead_callback` fires **at
    /// once**, with the last move time (`klippy/toolhead.py:526-531`) — so a
    /// value-only change on an idle planner is in force as soon as the command
    /// returns, and dated at that same time rather than at zero.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_a_value_only_change_with_nothing_queued_lands_at_the_last_move_time() {
        let printer = load_ok(&extruder_config_none_kinematics());
        printer.send_event(&KlippyEvent::KlippyReady);
        let toolhead = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");
        toolhead.connect().await.expect("the toolhead connects");
        let extruder = printer
            .lookup_object_as::<PrinterExtruder>("extruder")
            .expect("[extruder] is registered");
        extruder.pressure_advance.set(0.0, 0.05, 0.04);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        let last_move_time = toolhead.get_last_move_time();
        run_bounded(&gcode, "SET_PRESSURE_ADVANCE ADVANCE=.02").await;

        let schedule = extruder.pressure_advance.schedule();
        let landed = schedule.last().expect("the new entry is the last one");
        assert_eq!((landed.pressure_advance, landed.smooth_time), (0.02, 0.04));
        assert!(
            (landed.print_time - last_move_time).abs() < 1e-9,
            "the empty look-ahead fires at once with the last move time: {} vs {last_move_time}",
            landed.print_time
        );
        assert_eq!(extruder.pressure_advance.values(), (0.02, 0.04));
    }

    /// The other branch is untouched: a **changed smooth time** still flushes
    /// step generation and dates the new parameters at time zero
    /// (`kinematics/extruder.py:75-83`), not at the look-ahead's flush point.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_a_smooth_time_change_still_lands_at_time_zero() {
        let printer = load_ok(&extruder_config_none_kinematics());
        printer.send_event(&KlippyEvent::KlippyReady);
        let toolhead = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");
        toolhead.connect().await.expect("the toolhead connects");
        let extruder = printer
            .lookup_object_as::<PrinterExtruder>("extruder")
            .expect("[extruder] is registered");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        // The configured `ADVANCE` is 0, so the old *effective* smooth time is
        // 0 and enabling pressure advance is a smooth time change.
        run_bounded(&gcode, "SET_PRESSURE_ADVANCE ADVANCE=.05 SMOOTH_TIME=.02").await;

        let schedule = extruder.pressure_advance.schedule();
        assert_eq!(schedule.len(), 1, "the flushed write replaces the schedule");
        assert_eq!(schedule[0].print_time, 0.0, "dated at time zero");
        assert_eq!(
            (schedule[0].pressure_advance, schedule[0].smooth_time),
            (0.05, 0.02)
        );
    }

    /// With a default registered, a key that names something else still has to
    /// name a real extruder — the request reports what is available rather
    /// than falling back to the default (upstream `_cmd_mux`).
    #[test]
    fn test_an_unknown_extruder_name_reports_what_is_available() {
        let printer = load_ok(&extruder_config(""));
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        let err = gcode
            .run_script_sync("SET_PRESSURE_ADVANCE EXTRUDER=nope ADVANCE=.5")
            .unwrap_err();

        assert!(err.to_string().contains("extruder"), "{err}");
        assert!(!err.to_string().contains("missing"), "{err}");
    }

    #[test]
    fn test_the_extruder_section_loads_and_registers_commands() {
        // `commands.test`'s `M104`/`M109` must exist.
        let printer = load_ok(&extruder_config(""));
        let extruder = printer
            .lookup_object_as::<PrinterExtruder>("extruder")
            .expect("the extruder is registered");
        assert_eq!(extruder.name(), "extruder");
        assert!(extruder.printer_stepper().is_some());
        assert!(extruder.heater().can_extrude());
        // The hidden stepper object was registered so its connect will run.
        assert!(printer.lookup_object("extruder_stepper extruder").is_some());
    }

    /// `extruder_config` plus a second `[extruder1]` section, so an
    /// `ACTIVATE_EXTRUDER` switch has somewhere to move.
    fn two_extruder_config() -> String {
        extruder_config(
            "[extruder1]\nstep_pin: PB1\ndir_pin: PB2\nrotation_distance: 33.5\nmicrosteps: 16\n\
             nozzle_diameter: 0.4\nfilament_diameter: 1.75\nheater_pin: PB3\n\
             sensor_type: EPCOS 100K B57560G104F\nsensor_pin: PB5\ncontrol: pid\npid_Kp: 1\npid_Ki: 0.1\npid_Kd: 10\n\
             min_temp: 0\nmax_temp: 250\n",
        )
    }

    #[test]
    fn test_numbered_extruder_sections_are_read_through_the_primary() {
        let printer = load_ok(&two_extruder_config());
        let extruder1 = printer
            .lookup_object_as::<PrinterExtruder>("extruder1")
            .expect("extruder1 is registered");
        assert_eq!(extruder1.name(), "extruder1");
    }

    /// `ACTIVATE_EXTRUDER` records the new active extruder on the toolhead and
    /// then announces it (`kinematics/extruder.py:281-289`) — the event carries
    /// no payload upstream, and it is what re-anchors the E positions of
    /// `gcode_move` and `print_stats`.
    #[test]
    fn test_activate_extruder_switches_and_announces_the_switch() {
        let printer = load_ok(&two_extruder_config());
        printer.send_event(&KlippyEvent::KlippyReady);
        let announced = Arc::new(AtomicUsize::new(0));
        printer.register_event_handler(
            KlippyEvent::ExtruderActivateExtruder,
            Box::new({
                let announced = Arc::clone(&announced);
                move |_| {
                    announced.fetch_add(1, Ordering::SeqCst);
                }
            }),
        );
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        gcode
            .run_script_sync("ACTIVATE_EXTRUDER EXTRUDER=extruder1")
            .unwrap();

        let toolhead = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");
        assert_eq!(toolhead.active_extruder(), "extruder1");
        assert_eq!(announced.load(Ordering::SeqCst), 1);
    }

    /// An `EXTRUDER=` that names no configured extruder is rejected by the mux
    /// before any extruder's handler runs (upstream `_cmd_mux`), so nothing is
    /// switched and nothing is announced.
    #[test]
    fn test_activate_extruder_announces_nothing_for_an_unknown_name() {
        let printer = load_ok(&two_extruder_config());
        printer.send_event(&KlippyEvent::KlippyReady);
        let announced = Arc::new(AtomicUsize::new(0));
        printer.register_event_handler(
            KlippyEvent::ExtruderActivateExtruder,
            Box::new({
                let announced = Arc::clone(&announced);
                move |_| {
                    announced.fetch_add(1, Ordering::SeqCst);
                }
            }),
        );
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        let err = gcode
            .run_script_sync("ACTIVATE_EXTRUDER EXTRUDER=nope")
            .unwrap_err();

        assert!(err.to_string().contains("nope"), "{err}");
        let toolhead = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");
        assert_eq!(toolhead.active_extruder(), "extruder");
        assert_eq!(announced.load(Ordering::SeqCst), 0);
    }

    #[test]
    fn test_the_extrusion_checks_follow_upstream() {
        // Build an extruder by hand and drive `check_move` through a move.
        let printer = load_ok(&extruder_config(""));
        let extruder = printer
            .lookup_object_as::<PrinterExtruder>("extruder")
            .unwrap();
        // The limits were not resolved (connect did not run), so pin them.
        *PrinterExtruder::lock(&extruder.max_e_velocity) = 50.0;
        *PrinterExtruder::lock(&extruder.max_e_accel) = 100.0;

        let limits = MoveLimits {
            max_velocity: 300.0,
            max_accel: 3000.0,
            junction_deviation: 0.01,
            mcr_pseudo_accel: 1500.0,
        };
        // A retraction (negative E) is checked and its speed limited.
        let mut move_ = Move::new(
            Coord::new(0.0, 0.0, 0.0, 0.0),
            Coord::new(0.0, 0.0, 0.0, -5.0),
            100.0,
            &limits,
        );
        let mut ctx = MoveContext::new(&mut move_);
        assert!(extruder.check_move(&mut ctx, E_AXIS).is_ok());
        assert!(move_.max_cruise_v2 <= 50.0 * 50.0 + 1e-6);

        // Too long an extrude-only move is refused.
        let mut long = Move::new(
            Coord::default(),
            Coord::new(0.0, 0.0, 0.0, 100.0),
            1.0,
            &limits,
        );
        let mut ctx = MoveContext::new(&mut long);
        let err = extruder.check_move(&mut ctx, E_AXIS).unwrap_err();
        assert!(
            err.to_string().contains("Extrude only move too long"),
            "{err}"
        );
    }

    #[test]
    fn test_the_junction_uses_the_instantaneous_corner_velocity() {
        let printer = load_ok(&extruder_config(""));
        let extruder = printer
            .lookup_object_as::<PrinterExtruder>("extruder")
            .unwrap();
        let limits = MoveLimits {
            max_velocity: 300.0,
            max_accel: 3000.0,
            junction_deviation: 0.01,
            mcr_pseudo_accel: 1500.0,
        };
        let prev = Move::new(
            Coord::default(),
            Coord::new(10.0, 0.0, 0.0, 0.0),
            100.0,
            &limits,
        );
        let cur = Move::new(
            Coord::new(10.0, 0.0, 0.0, 0.0),
            Coord::new(20.0, 0.0, 0.0, 1.0),
            100.0,
            &limits,
        );
        // A change in E direction cosine yields the corner-velocity limit.
        let v2 = ExtraAxis::calc_junction(extruder.as_ref(), &prev, &cur, E_AXIS);
        assert!(v2 > 0.0 && v2 < 10_000.0, "{v2}");
        // A move with no E change is not limited by the extruder.
        let cur2 = Move::new(
            Coord::new(10.0, 0.0, 0.0, 0.0),
            Coord::new(20.0, 0.0, 0.0, 0.0),
            100.0,
            &limits,
        );
        let v2 = ExtraAxis::calc_junction(extruder.as_ref(), &prev, &cur2, E_AXIS);
        assert_eq!(v2, cur2.max_cruise_v2);
    }

    /// `M109 S<temp>` waits for the heater to reach the target via
    /// `PrinterHeaters::set_temperature(.., wait=true)`
    /// (`kinematics/extruder.py:260-279`). Under file-output the wait loop
    /// returns at once (`heaters.py:350-351`), so the observable effect is the
    /// target being set — the same path `PID_CALIBRATE` uses.
    #[tokio::test]
    async fn test_m109_with_wait_sets_the_target_through_pheaters() {
        let printer = load_ok(&extruder_config(""));
        // File-output mode: `wait_for_temperature` returns immediately.
        let mut args = crate::core::klippy::api::StartArgs::collect(
            "/tmp/printer.cfg",
            None,
            crate::core::klippy::api::endpoints::info::NO_KLIPPER_PATH,
        );
        args.debug_output = Some("_test_output".to_string());
        printer.set_start_args(Arc::new(args));
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        gcode
            .run_script("M109 S200")
            .await
            .expect("M109 sets the target");

        let extruder = printer
            .lookup_object_as::<PrinterExtruder>("extruder")
            .expect("the extruder is registered");
        assert_eq!(extruder.heater().target_temp(), 200.0);
    }

    /// `M104 S<temp>` sets the target and returns without waiting
    /// (`set_temperature(.., wait=false)`).
    #[tokio::test]
    async fn test_m104_sets_the_target_without_waiting() {
        let printer = load_ok(&extruder_config(""));
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        gcode
            .run_script("M104 S180")
            .await
            .expect("M104 sets the target");

        let extruder = printer
            .lookup_object_as::<PrinterExtruder>("extruder")
            .expect("the extruder is registered");
        assert_eq!(extruder.heater().target_temp(), 180.0);
    }

    /// The `atmega2560` dictionary a fake board serves, or `None` when
    /// `build.rs` did not build it (`KLIPPERX_ARCHES` decides which exist), so a
    /// test skips instead of failing on a product it never got.
    fn board() -> Option<ResponderMcu> {
        ResponderMcu::new("mcu", "atmega2560.dict")
    }

    /// A `[extruder]` over the fake firmware, with `kinematics: none` — the
    /// extruder's moves queue on its own trapq, which is all a past-position
    /// lookup needs.
    fn machine_config(board: &ResponderMcu) -> String {
        format!(
            "[mcu]\ntest: dict={}\n\
             [printer]\nkinematics: none\nmax_velocity: 300\nmax_accel: 3000\n\
             [extruder]\nstep_pin: PF2\ndir_pin: PF3\nmicrosteps: 16\n\
             rotation_distance: 33.500\nnozzle_diameter: 0.400\nfilament_diameter: 1.750\n\
             heater_pin: PB4\nsensor_type: EPCOS 100K B57560G104F\nsensor_pin: PK5\n\
             control: pid\npid_Kp: 22.2\npid_Ki: 1.08\npid_Kd: 114\n\
             min_temp: 0\nmax_temp: 300\nmin_extrude_temp: 0\n",
            board.dict().display()
        )
    }

    /// Load and bring the machine up; `Err` says why it did not come up.
    async fn up_machine(board: &ResponderMcu) -> (Arc<Printer>, Result<(), String>) {
        let config = Config::from_text(&machine_config(board))
            .expect("the config parses")
            .0;
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let printer = Arc::new(Printer::new(reactor));
        let mut start_args = crate::core::klippy::api::StartArgs::collect(
            "extruder.cfg",
            None,
            crate::core::klippy::api::endpoints::info::NO_KLIPPER_PATH,
        );
        start_args.debug_output = Some("_test_output".to_string());
        printer.set_start_args(Arc::new(start_args));
        let setup = async {
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
            Ok(())
        }
        .await;
        (printer, setup)
    }

    /// `find_past_position` answers from the stepper's step compressor, as
    /// upstream's `mcu_to_commanded_position(get_past_mcu_position(t))` does
    /// (`kinematics/extruder.py:47-49`): before a move it is the position the
    /// motor started at, mid-move it is where the generated steps had reached,
    /// and at the move's end it is the end position. A stub returning
    /// `last_position` (the planned end) or `0.0` fails the first two or the
    /// last check.
    ///
    /// The move is short (1 mm at F600 = 0.1 s) so that it sits inside the step
    /// generation window: a time past the last generation holds the counter it
    /// reached, by design.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_find_past_position_follows_the_generated_steps() {
        use crate::core::klippy::motion::extra::ExtraAxis;

        let Some(board) = board() else {
            return;
        };
        let (printer, setup) = up_machine(&board).await;
        let outcome = async {
            setup?;
            let gcode = printer
                .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                .expect("the loader registers `gcode`")
                .clone();
            let toolhead = printer
                .lookup_object_as::<ToolHeadObject>("toolhead")
                .ok_or_else(|| "the toolhead is not registered".to_string())?;
            let run = |script: &'static str| {
                let gcode = Arc::clone(&gcode);
                let toolhead = Arc::clone(&toolhead);
                async move {
                    tokio::time::timeout(Duration::from_secs(10), gcode.run_script(script))
                        .await
                        .map_err(|_| format!("`{script}` timed out"))?
                        .map_err(|err| format!("`{script}`: {err}"))?;
                    tokio::time::timeout(Duration::from_secs(10), toolhead.flush_step_generation())
                        .await
                        .map_err(|_| format!("`{script}` flush timed out"))?
                        .map_err(|err| err.to_string())
                }
            };

            run("G92 E0").await?;
            run("G1 E1 F600").await?;
            let extruder = printer
                .lookup_object_as::<PrinterExtruder>("extruder")
                .ok_or_else(|| "the extruder is not registered".to_string())?;
            // The move's own span, from the extruder's trapq: the gap before it
            // is a null move that does not travel (its `axes_r` is zero).
            let trapq = extruder
                .trapq()
                .ok_or_else(|| "the extruder has no trapq".to_string())?;
            let segments: Vec<_> = toolhead
                .trapq_extract(trapq, 64, 0.0, 1e18)
                .ok_or_else(|| "the toolhead is not connected".to_string())?
                .into_iter()
                .filter(|segment| segment.axes_r.x() != 0.0)
                .collect();
            let (start, end) = match (
                segments
                    .iter()
                    .map(|segment| segment.print_time)
                    .reduce(f64::min),
                segments
                    .iter()
                    .map(|segment| segment.end_time())
                    .reduce(f64::max),
            ) {
                (Some(start), Some(end)) => (start, end),
                _ => return Err("the move is not in the extruder's trapq".to_string()),
            };
            let at = |time: f64| ExtraAxis::find_past_position(&*extruder, time);

            // Before the move: the stepper had not stepped yet.
            let before = at(start - 0.01);
            if before.abs() > 0.02 {
                return Err(format!("before the move the position is {before}, not 0"));
            }
            // Half way along: about half a millimetre of filament.
            let middle = at(start + 0.5 * (end - start));
            if (middle - 0.5).abs() > 0.05 {
                return Err(format!("mid-move the position is {middle}, not ~0.5"));
            }
            // At the move's end: the full millimetre (to the last whole step).
            let after = at(end);
            if (after - 1.0).abs() > 0.02 {
                return Err(format!("at the move's end the position is {after}, not 1"));
            }
            Ok(())
        }
        .await;
        printer.teardown();
        outcome.unwrap();
    }
}
