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
//! up there, the bang-bang/PID control loop is in place, and so is the
//! `[verify_heater]` check over it. What is still open is the `M109`
//! wait-for-temperature loop (`_wait` is accepted and ignored) and
//! `pid_calibrate`.

use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::extruder_stepper::{
    cmd_set_extruder_rotation_distance, cmd_sync_extruder_motion,
};
use crate::core::klippy::extras::heaters::{self, Heater};
use crate::core::klippy::extras::stepper::PrinterStepper;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::McuStepper;
use crate::core::klippy::motion::extra::ExtraAxis;
use crate::core::klippy::motion::itersolve::{extruder_active_flags, extruder_position_fn, Axis};
use crate::core::klippy::motion::kinematics::MoveContext;
use crate::core::klippy::motion::plan::Move;
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
    /// Pressure advance, for `SET_PRESSURE_ADVANCE` and status (no motion effect
    /// yet: the smooth filter is part of the extruder solver, H6).
    ///
    /// `Arc` so the command handler can own a handle to the same slot.
    pressure_advance: Arc<Mutex<f64>>,
    pressure_advance_smooth_time: Arc<Mutex<f64>>,
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
        let pressure_advance =
            config.get_float_bounded("pressure_advance", Some(0.0), Some(0.0), None, None, None)?;
        let pressure_advance_smooth_time = config.get_float_bounded(
            "pressure_advance_smooth_time",
            Some(0.040),
            None,
            Some(0.200),
            Some(0.0),
            None,
        )?;

        // Upstream only builds the stepper when the section names one of the
        // motor options (`kinematics/extruder.py:172-177`).
        let stepper = if config.section().has("step_pin")
            || config.section().has("dir_pin")
            || config.section().has("rotation_distance")
        {
            let stepper = Arc::new(PrinterStepper::new(config, printer, Axis::X, false)?);
            stepper.setup_itersolve(extruder_position_fn(), extruder_active_flags());
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
            pressure_advance: Arc::new(Mutex::new(pressure_advance)),
            pressure_advance_smooth_time: Arc::new(Mutex::new(pressure_advance_smooth_time)),
            printer: Arc::downgrade(printer),
        };
        extruder.register_commands(printer)?;
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
                let handler: CommandHandler =
                    sync(move |gcmd: &GcodeCommand| cmd_set_temperature(&printer, gcmd, wait));
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

        // SET_PRESSURE_ADVANCE is a per-extruder mux command; it records the
        // value for status. The smooth filter on the solver is H6.
        {
            let pa = Arc::clone(&self.pressure_advance);
            let smooth = Arc::clone(&self.pressure_advance_smooth_time);
            let handler: CommandHandler =
                sync(move |gcmd: &GcodeCommand| set_pressure_advance(gcmd, &pa, &smooth));
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

        // ACTIVATE_EXTRUDER: record the active extruder on the toolhead.
        {
            let name = self.name.clone();
            let printer = Arc::downgrade(printer);
            let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
                let Some(printer) = printer.upgrade() else {
                    return Err(CommandError::new("printer is gone"));
                };
                if let Some(toolhead) = printer.lookup_object_as::<ToolHeadObject>("toolhead") {
                    toolhead.set_active_extruder(&name);
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

/// `SET_PRESSURE_ADVANCE`'s body: record both values and report them
/// (upstream `extruder_stepper.cmd_SET_PRESSURE_ADVANCE`).
fn set_pressure_advance(
    gcmd: &GcodeCommand,
    advance: &Arc<Mutex<f64>>,
    smooth_time: &Arc<Mutex<f64>>,
) -> Result<(), CommandError> {
    let value = gcmd.get_float_default("ADVANCE", *PrinterExtruder::lock(advance))?;
    let smooth = gcmd.get_float_default("SMOOTH_TIME", *PrinterExtruder::lock(smooth_time))?;
    *PrinterExtruder::lock(advance) = value;
    *PrinterExtruder::lock(smooth_time) = smooth;
    gcmd.respond_info(&format!(
        "pressure_advance: {value:.6}\npressure_advance_smooth_time: {smooth:.6}",
    ));
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
    set_pressure_advance(
        gcmd,
        &extruder.pressure_advance,
        &extruder.pressure_advance_smooth_time,
    )
}

impl PrinterObject for PrinterExtruder {
    fn get_status(&self, _eventtime: f64) -> Value {
        let mut status = self.heater.get_status();
        if let Value::Object(map) = &mut status {
            map.insert(
                "pressure_advance".to_string(),
                json!(*Self::lock(&self.pressure_advance)),
            );
            map.insert(
                "smooth_time".to_string(),
                json!(*Self::lock(&self.pressure_advance_smooth_time)),
            );
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
        let axes_d = *ctx.axes_d();
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
            crate::core::klippy::mathutil::Xyz::new(move_.start_pos.axis(ea_index), 0.0, 0.0),
            crate::core::klippy::mathutil::Xyz::new(1.0, can_pressure_advance as u8 as f64, 0.0),
            move_.start_v * axis_r,
            move_.cruise_v * axis_r,
            move_.accel * axis_r,
        );
        *Self::lock(&self.last_position) = move_.end_pos.axis(ea_index);
    }

    fn find_past_position(&self, _print_time: f64) -> f64 {
        // Needs the host stepper, which the toolhead owns; only a motion report
        // asks for this, and it is not wired yet.
        *Self::lock(&self.last_position)
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
/// share `cmd_set_temperature`; `wait` only selects the not-yet-wired wait
/// loop.
const M104_M109_PARAMS: &[&str] = &["S", "T"];

fn cmd_set_temperature(
    printer: &std::sync::Weak<Printer>,
    gcmd: &GcodeCommand,
    _wait: bool,
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
    extruder.heater().set_temp(temp)?;
    Ok(())
}

/// The `[extruder]` factory: build the primary and its numbered siblings.
///
/// Upstream's `kinematics.extruder.add_printer_objects` loops `extruder`,
/// `extruder1`… (`kinematics/extruder.py:314-320`); here the numbered sections
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
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::mathutil::{Coord, E_AXIS};
    use crate::core::klippy::motion::plan::MoveLimits;
    use crate::core::klippy::motion::Move;
    use crate::core::klippy::reactor::ManualReactor;

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

    #[test]
    fn test_numbered_extruder_sections_are_read_through_the_primary() {
        let printer = load_ok(&extruder_config(
            "[extruder1]\nstep_pin: PB1\ndir_pin: PB2\nrotation_distance: 33.5\nmicrosteps: 16\n\
             nozzle_diameter: 0.4\nfilament_diameter: 1.75\nheater_pin: PB3\n\
             sensor_type: EPCOS 100K B57560G104F\nsensor_pin: PB5\ncontrol: pid\npid_Kp: 1\npid_Ki: 0.1\npid_Kd: 10\n\
             min_temp: 0\nmax_temp: 250\n",
        ));
        let extruder1 = printer
            .lookup_object_as::<PrinterExtruder>("extruder1")
            .expect("extruder1 is registered");
        assert_eq!(extruder1.name(), "extruder1");
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
}
