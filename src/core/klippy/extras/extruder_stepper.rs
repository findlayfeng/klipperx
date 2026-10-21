//! `[extruder_stepper <name>]` — an extra stepper fed by a filament extruder.
//!
//! Upstream `klippy/extras/extruder_stepper.py` wraps `kinematics/extruder.py`'s
//! `ExtruderStepper`: a `PrinterStepper` whose solver reads the extrusion
//! amount, bound at `klippy:connect` to the `[extruder]` the section's
//! `extruder` option names (`sync_to_extruder` points the stepper's trapq at
//! that extruder's).
//!
//! This port reads the section the same way — the motor options through
//! [`PrinterStepper`], plus `extruder` and the two pressure-advance options —
//! installs the section's **own** pressure-advance parameters on that stepper's
//! solver (`ExtruderStepper.__init__` reads them from the section's config, not
//! from the `[extruder]` it follows: `kinematics/extruder.py:12-16`), registers
//! the stepper's own `SET_PRESSURE_ADVANCE` mux value, and binds to the extruder
//! at connect, complaining with upstream's wording when the name is not an
//! extruder.
//!
//! What binds it to the extruder's **motion** is upstream's `sync_to_extruder`
//! (`kinematics/extruder.py:50-66`): the stepper's solver reads the extruder's
//! own trapq, whose x is the extrusion amount, so the motor takes exactly the
//! steps that extruder takes. The host stepper joins the toolhead's motion queue
//! at the toolhead's own connect — the first moment the extruder's trapq exists
//! (it is allocated there) — and `SYNC_EXTRUDER_MOTION` re-points it while the
//! machine runs. An unbound stepper stays detached (`set_trapq(None)`) and
//! generates no steps, as upstream leaves it.
//!
//! `SET_EXTRUDER_ROTATION_DISTANCE` rewrites the step distance, which both the
//! recorded geometry ([`PrinterStepper::step_dist`]) and the live solver's
//! conversion follow (`ToolHeadObject::set_stepper_step_dist`).

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::error::KlippyError;
use crate::core::klippy::extras::extruder::{apply_pressure_advance, PrinterExtruder};
use crate::core::klippy::extras::stepper::PrinterStepper;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::McuStepper;
use crate::core::klippy::motion::itersolve::{extruder_active_flags, extruder_position_fn, Axis};
use crate::core::klippy::motion::pressure_advance::ExtruderPressureAdvance;
use crate::core::klippy::motion::stepper::Stepper;
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};

// Only the prefix form exists upstream (`extruder_stepper.py:23`).
section!("extruder_stepper", order = 20, prefix = load_config_prefix);

/// The toolhead object, as the loader registers `[printer]`.
const TOOLHEAD_OBJECT: &str = "toolhead";

/// One configured `[extruder_stepper <name>]`.
pub struct PrinterExtruderStepper {
    /// The section suffix (`my_extra_stepper`): upstream's mux value and the
    /// name `ExtruderStepper` takes from `config.get_name().split()[-1]`.
    name: String,
    /// The extruder to bind to (the section's `extruder` option).
    extruder_name: String,
    /// The wrapped stepper: the motor options are read through it, and it runs
    /// the extruder position function.
    stepper: Arc<PrinterStepper>,
    /// The live parameters `SET_PRESSURE_ADVANCE` writes: status reports them
    /// and this stepper's own solver smooths with them — the same shared block
    /// the primary `[extruder]` hands its stepper (`extruder_stepper_alloc` +
    /// `extruder_set_pressure_advance`). They start at the section's
    /// `pressure_advance`/`pressure_advance_smooth_time` (upstream's
    /// `config_pa`/`config_smooth_time`, applied from time zero at connect).
    pressure_advance: Arc<ExtruderPressureAdvance>,
    /// `motion_queue`: the extruder whose trapq this stepper's motion follows,
    /// or `None` when it is detached (`kinematics/extruder.py:59-65`).
    /// `Arc` so the `SYNC_EXTRUDER_MOTION` handler shares the same slot.
    motion_queue: Arc<Mutex<Option<String>>>,
    /// The machine, to find the extruder at connect.
    printer: Weak<Printer>,
}

impl PrinterExtruderStepper {
    /// Build the section: read every option, build the stepper, register the
    /// stepper's `SET_PRESSURE_ADVANCE` mux value.
    ///
    /// # Errors
    /// A missing option (`extruder`), an out-of-range pressure-advance option,
    /// or any option [`PrinterStepper`] refuses.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let name = config
            .section()
            .sub
            .clone()
            .unwrap_or_else(|| config.section().id.clone());
        let extruder_name = config.get("extruder", None)?;
        // `pressure_advance` (min 0) and `pressure_advance_smooth_time`
        // (default .040, above 0, max .200), as upstream's `ExtruderStepper`
        // reads them (`kinematics/extruder.py:14-16`).
        let config_pa =
            config.get_float_bounded("pressure_advance", Some(0.0), Some(0.0), None, None, None)?;
        let config_smooth_time = config.get_float_bounded(
            "pressure_advance_smooth_time",
            Some(0.040),
            None,
            Some(0.200),
            Some(0.0),
            None,
        )?;

        let stepper = Arc::new(PrinterStepper::new(config, printer, Axis::X, false)?);
        stepper.setup_itersolve(extruder_position_fn(), extruder_active_flags());
        // The section's own pressure advance drives its own solver, from print
        // time zero (`extruder_stepper_alloc`; upstream's `_handle_connect`
        // calls `espa(sk, 0., config_pa, config_smooth_time)`,
        // `kinematics/extruder.py:40-41`). An unconfigured section (`pa = 0`)
        // keeps the plain position function: the parameters zero their own
        // smooth time, which is the solver's off switch (`kin_extruder.c:129-132`).
        let pressure_advance = ExtruderPressureAdvance::new(config_pa, config_smooth_time);
        stepper.setup_pressure_advance(Arc::clone(&pressure_advance));

        let extruder_stepper = Self {
            name,
            extruder_name,
            stepper,
            pressure_advance,
            motion_queue: Arc::new(Mutex::new(None)),
            printer: Arc::downgrade(printer),
        };
        extruder_stepper.register_commands(printer)?;
        Ok(extruder_stepper)
    }

    /// The section suffix (`my_extra_stepper`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The extruder this stepper is configured to follow.
    pub fn extruder_name(&self) -> &str {
        &self.extruder_name
    }

    /// The extruder this stepper's motion follows right now — upstream's
    /// `sync_to_extruder`'s `self.motion_queue` (`kinematics/extruder.py:59`).
    pub fn bound_extruder(&self) -> Option<String> {
        lock(&self.motion_queue).clone()
    }

    /// Take the host stepper the toolhead will drive: the same handover
    /// `[extruder]` makes through its own `take_stepper`.
    pub fn take_stepper(&self) -> Option<Stepper> {
        self.stepper.take_stepper()
    }

    /// The firmware stepper this section's steps are sent through.
    pub fn mcu_stepper(&self) -> Arc<McuStepper> {
        Arc::clone(self.stepper.mcu_stepper())
    }

    /// Register `SET_PRESSURE_ADVANCE EXTRUDER=<name>` — the stepper's own mux
    /// value (`kinematics/extruder.py:32-34`) — plus the two motion-sync
    /// commands every extruder stepper registers (`:37-42`):
    /// `SET_EXTRUDER_ROTATION_DISTANCE` on this stepper and
    /// `SYNC_EXTRUDER_MOTION` on its binding. Both flush step generation first
    /// (`toolhead.flush_step_generation`, `:120,135`), so the handlers are
    /// futures like upstream's own bodies are allowed to be elsewhere; nothing
    /// queued is stepped over by the change.
    fn register_commands(&self, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        {
            let advance = Arc::clone(&self.pressure_advance);
            let printer = Arc::downgrade(printer);
            let handler: CommandHandler =
                sync(move |gcmd: &GcodeCommand| cmd_set_pressure_advance(gcmd, &advance, &printer));
            gcode
                .register_mux_command_with_params(
                    "SET_PRESSURE_ADVANCE",
                    "EXTRUDER",
                    Some(&self.name),
                    handler,
                    Some("Set pressure advance parameters"),
                    &["ADVANCE", "SMOOTH_TIME"],
                )
                .map_err(ConfigError::new)?;
        }

        // The motion-sync commands every extruder stepper registers
        // (`kinematics/extruder.py:37-42`): the rotation distance is set on the
        // wrapped stepper, the motion queue on this object's binding.
        {
            let stepper = Arc::clone(&self.stepper);
            let name = self.name.clone();
            let printer = Arc::downgrade(printer);
            let handler: CommandHandler = Arc::new(move |gcmd: &GcodeCommand| {
                let stepper = Arc::clone(&stepper);
                let name = name.clone();
                let printer = printer.clone();
                Box::pin(async move {
                    cmd_set_extruder_rotation_distance(gcmd, &name, &stepper, &printer).await
                })
            });
            gcode
                .register_mux_command_with_params(
                    "SET_EXTRUDER_ROTATION_DISTANCE",
                    "EXTRUDER",
                    Some(&self.name),
                    handler,
                    Some("Set extruder rotation distance"),
                    &["DISTANCE"],
                )
                .map_err(ConfigError::new)?;
        }
        {
            let queue = Arc::clone(&self.motion_queue);
            let stepper = Arc::clone(&self.stepper);
            let name = self.name.clone();
            let printer = Arc::downgrade(printer);
            let handler: CommandHandler = Arc::new(move |gcmd: &GcodeCommand| {
                let queue = Arc::clone(&queue);
                let stepper = Arc::clone(&stepper);
                let name = name.clone();
                let printer = printer.clone();
                Box::pin(async move {
                    cmd_sync_extruder_motion(gcmd, &name, &printer, &queue, Some(&stepper)).await
                })
            });
            gcode
                .register_mux_command_with_params(
                    "SYNC_EXTRUDER_MOTION",
                    "EXTRUDER",
                    Some(&self.name),
                    handler,
                    Some("Set extruder stepper motion queue"),
                    &["MOTION_QUEUE"],
                )
                .map_err(ConfigError::new)?;
        }
        Ok(())
    }

    /// Bind to the configured extruder (upstream's
    /// `ExtruderStepper.sync_to_extruder`, `kinematics/extruder.py:50-66`): an
    /// empty name detaches, any other must name an `[extruder]`.
    ///
    /// The trapq repointing happens here too, through
    /// [`ToolHeadObject::set_stepper_trapq`] once the machine is up; at the
    /// section's own connect the toolhead is not connected yet and its connect
    /// does the pointing, from [`PrinterExtruderStepper::bound_extruder`].
    ///
    /// # Errors
    /// Upstream's wording when the name does not resolve to an extruder.
    pub(crate) fn sync_to_extruder(&self, printer: &Arc<Printer>) -> Result<(), KlippyError> {
        bind_motion_queue(
            printer,
            &self.motion_queue,
            Some(&self.stepper),
            &self.extruder_name,
        )
        .map_err(KlippyError::Config)
    }
}

impl PrinterObject for PrinterExtruderStepper {
    /// Upstream's `ExtruderStepper.get_status`: pressure advance, smooth time
    /// and the motion queue (`kinematics/extruder.py:43-46`).
    fn get_status(&self, _eventtime: f64) -> Value {
        let (pressure_advance, smooth_time) = self.pressure_advance.values();
        json!({
            "pressure_advance": pressure_advance,
            "smooth_time": smooth_time,
            "motion_queue": lock(&self.motion_queue).clone(),
        })
    }

    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            // The host stepper builds its solver here, as every `[stepper_*]`
            // does at its own connect; the section's pressure advance was
            // installed on it at load, so the solver smooths with it from the
            // first generated step.
            self.stepper.connect().await?;
            let printer = self.printer.upgrade().ok_or_else(|| {
                KlippyError::Internal("the printer dropped before connect".to_string())
            })?;
            // `PrinterExtruderStepper.handle_connect` binds the extruder
            // (`extruder_stepper.py:19-21`).
            self.sync_to_extruder(&printer)
        })
    }
}

impl std::fmt::Debug for PrinterExtruderStepper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterExtruderStepper")
            .field("name", &self.name)
            .field("extruder_name", &self.extruder_name)
            .finish_non_exhaustive()
    }
}

/// `SET_PRESSURE_ADVANCE`'s body for one extra stepper: validate the two values
/// against upstream's bounds (the section's own values are the defaults), then
/// apply them to this stepper's solver the way the primary extruder does
/// (upstream `ExtruderStepper.cmd_SET_PRESSURE_ADVANCE`,
/// `kinematics/extruder.py:98-107` → `_set_pressure_advance`, `:67-88`).
fn cmd_set_pressure_advance(
    gcmd: &GcodeCommand,
    pressure_advance: &Arc<ExtruderPressureAdvance>,
    printer: &Weak<Printer>,
) -> Result<(), CommandError> {
    let (advance, smooth_time) = pressure_advance.values();
    let parse = |raw: &str| raw.parse::<f64>().ok();
    let value = gcmd.get("ADVANCE", Some(advance), parse, Some(0.0), None, None, None)?;
    let smooth = gcmd.get(
        "SMOOTH_TIME",
        Some(smooth_time),
        parse,
        Some(0.0),
        Some(0.200),
        None,
        None,
    )?;
    apply_pressure_advance(pressure_advance, printer, value, smooth)?;
    gcmd.respond_info(&format!(
        "pressure_advance: {value:.6}\npressure_advance_smooth_time: {smooth:.6}",
    ));
    Ok(())
}

/// `SET_EXTRUDER_ROTATION_DISTANCE`'s body for one extruder's stepper
/// (upstream `ExtruderStepper.cmd_SET_E_ROTATION_DISTANCE`,
/// `kinematics/extruder.py:111-131`).
///
/// A missing `DISTANCE` reports the current value; a zero is refused; a
/// negative value flips the direction and stores the absolute distance. Each
/// `[extruder]` and `[extruder_stepper <name>]` registers this for its own name.
/// Upstream flushes step generation before rebuilding the solver (`:120-122`)
/// and the live stepper's conversion follows the new distance
/// (`MCU_stepper.set_rotation_distance`, `klippy/stepper.py:138-143`).
///
/// # Errors
/// Upstream's wording when `DISTANCE` is zero, plus the parameter errors.
pub(crate) async fn cmd_set_extruder_rotation_distance(
    gcmd: &GcodeCommand,
    name: &str,
    stepper: &PrinterStepper,
    printer: &Weak<Printer>,
) -> Result<(), CommandError> {
    let rotation_dist = if gcmd.get_command_parameters().contains_key("DISTANCE") {
        let distance = gcmd.get_float("DISTANCE")?;
        if distance == 0.0 {
            return Err(CommandError::new("Rotation distance can not be zero"));
        }
        let (_, orig_invert_dir) = stepper.get_dir_inverted();
        let mut next_invert_dir = orig_invert_dir;
        let mut distance = distance;
        if distance < 0.0 {
            next_invert_dir = !orig_invert_dir;
            distance = -distance;
        }
        // Upstream flushes step generation before rebuilding the solver
        // (`kinematics/extruder.py:120-122`): the segments already planned were
        // stepped with the old distance, so they are generated before the new
        // one is in force.
        let toolhead = connected_toolhead(printer);
        if let Some(toolhead) = &toolhead {
            toolhead.flush_step_generation().await?;
        }
        stepper.set_rotation_distance(distance);
        stepper.set_dir_inverted(next_invert_dir);
        // The live solver keeps its own copy of the step distance; upstream's
        // `MCU_stepper.set_rotation_distance` rewrites it in place.
        if let Some(toolhead) = &toolhead {
            toolhead.set_stepper_step_dist(name, stepper.step_dist())?;
        }
        distance
    } else {
        stepper.get_rotation_distance().0
    };
    let (invert_dir, orig_invert_dir) = stepper.get_dir_inverted();
    let rotation_dist = if invert_dir != orig_invert_dir {
        -rotation_dist
    } else {
        rotation_dist
    };
    gcmd.respond_info(&format!(
        "Extruder '{name}' rotation distance set to {rotation_dist:.6}"
    ));
    Ok(())
}

/// `SYNC_EXTRUDER_MOTION`'s body for one extruder's stepper (upstream
/// `cmd_SYNC_EXTRUDER_MOTION`, `kinematics/extruder.py:133-137`).
///
/// An empty `MOTION_QUEUE` detaches — upstream's `sync_to_extruder(""` branch,
/// which must **not** be treated as an invalid name; any other value must name
/// an `[extruder]`. Step generation is flushed first (`:52`), so everything
/// planned into the old queue is stepped before the stepper leaves it.
///
/// `stepper` is the motion stepper to point at the named extruder's trapq. The
/// primary `[extruder]` passes `None`: its stepper's trapq *is* its own queue,
/// so a `SYNC_EXTRUDER_MOTION` for it only records the binding (see the module
/// docs).
///
/// # Errors
/// Upstream's wording when the name does not resolve to an extruder.
pub(crate) async fn cmd_sync_extruder_motion(
    gcmd: &GcodeCommand,
    name: &str,
    printer: &Weak<Printer>,
    motion_queue: &Mutex<Option<String>>,
    stepper: Option<&Arc<PrinterStepper>>,
) -> Result<(), CommandError> {
    let ename = gcmd.get_str_default("MOTION_QUEUE", "");
    let printer = printer
        .upgrade()
        .ok_or_else(|| CommandError::new("printer is gone"))?;
    if let Some(toolhead) = connected_toolhead(&Arc::downgrade(&printer)) {
        toolhead.flush_step_generation().await?;
    }
    bind_motion_queue(&printer, motion_queue, stepper.map(Arc::as_ref), &ename)
        .map_err(|err| CommandError::new(err.to_string()))?;
    gcmd.respond_info(&format!("Extruder '{name}' now syncing with '{ename}'"));
    Ok(())
}

/// Upstream's `ExtruderStepper.sync_to_extruder` (`kinematics/extruder.py:50-66`):
/// record `extruder_name` as the motion queue and point `stepper` at that
/// extruder's trapq, or detach it for an empty name.
///
/// # Errors
/// Upstream's wording when the name is not an `[extruder]`.
fn bind_motion_queue(
    printer: &Arc<Printer>,
    motion_queue: &Mutex<Option<String>>,
    stepper: Option<&PrinterStepper>,
    extruder_name: &str,
) -> Result<(), ConfigError> {
    if extruder_name.is_empty() {
        *lock(motion_queue) = None;
        return attach_trapq(printer, stepper, None);
    }
    let extruder = printer
        .lookup_object_as::<PrinterExtruder>(extruder_name)
        .ok_or_else(|| ConfigError::new(format!("'{extruder_name}' is not a valid extruder.")))?;
    *lock(motion_queue) = Some(extruder_name.to_string());
    attach_trapq(printer, stepper, Some(&extruder))
}

/// Point `stepper` at `extruder`'s trapq with its position where that extruder
/// last planned extrusion (upstream's
/// `set_position([extruder.last_position, 0., 0.])` then
/// `set_trapq(extruder.get_trapq())`, `kinematics/extruder.py:63-65`), or take
/// it off every queue with `None` (`:56-58`).
///
/// Before the machine is up there is no planner to point anything at: nothing
/// has been generated, so there is no position to seed, and the toolhead's own
/// connect does the pointing from
/// [`PrinterExtruderStepper::bound_extruder`].
fn attach_trapq(
    printer: &Arc<Printer>,
    stepper: Option<&PrinterStepper>,
    extruder: Option<&Arc<PrinterExtruder>>,
) -> Result<(), ConfigError> {
    let (Some(stepper), Some(toolhead)) = (
        stepper,
        printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT),
    ) else {
        return Ok(());
    };
    if !toolhead.is_connected() {
        return Ok(());
    }
    let name = stepper.name();
    if let Some(extruder) = extruder {
        toolhead
            .set_stepper_position(name, extruder.last_position())
            .map_err(|err| ConfigError::new(err.to_string()))?;
        toolhead
            .set_stepper_trapq(name, extruder.trapq())
            .map_err(|err| ConfigError::new(err.to_string()))?;
    } else {
        toolhead
            .set_stepper_trapq(name, None)
            .map_err(|err| ConfigError::new(err.to_string()))?;
    }
    Ok(())
}

/// The toolhead, when the machine is up far enough to have one: a command run
/// before connect has nothing queued and nothing to flush.
fn connected_toolhead(printer: &Weak<Printer>) -> Option<Arc<ToolHeadObject>> {
    printer
        .upgrade()
        .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT))
        .filter(|toolhead| toolhead.is_connected())
}

/// The factory the section declaration names (`extruder_stepper.py:23`).
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PrinterExtruderStepper::new(config, printer)?))
}

/// The wrapped stepper's position at a past print time (upstream's
/// `PrinterExtruderStepper.find_past_position`).
impl PrinterExtruderStepper {
    pub fn find_past_position(&self, print_time: f64) -> f64 {
        // Upstream's `mcu_to_commanded_position(get_past_mcu_position(t))`; the
        // host stepper belongs to the toolhead after connect, so both halves
        // are reached through it.
        self.printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>("toolhead"))
            .and_then(|toolhead| {
                self.stepper
                    .get_past_mcu_position(&toolhead, print_time)
                    .map(|mcu_pos| self.stepper.mcu_to_commanded_position(mcu_pos))
            })
            // Before connect there is no compressor to ask; nothing has been
            // generated, so no position has been reached yet.
            .unwrap_or(0.0)
    }
}

fn lock<T>(slot: &Mutex<T>) -> MutexGuard<'_, T> {
    slot.lock().unwrap_or_else(|poison| poison.into_inner())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::mathutil::{Coord, Xyz};
    use crate::core::klippy::motion::trapq::Trapq;
    use crate::core::klippy::motion::Stepper;
    use crate::core::klippy::reactor::ManualReactor;
    use std::time::Duration;

    /// The fake firmware's clock rate, for a solver built without one.
    const MCU_FREQ: f64 = 1_000_000.0;

    /// The corpus's section body (`test/klippy/pressure_advance.cfg:52-58`).
    const EXTRA_STEPPER: &str = "extruder: extruder\n\
        step_pin: PH5\ndir_pin: PH6\nenable_pin: !PB5\n\
        microsteps: 16\nrotation_distance: 28.2\n";

    /// A cartesian printer config whose `[extruder_stepper my_extra_stepper]`
    /// carries `section` as its options.
    fn config(section: &str) -> String {
        format!(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_y]\nstep_pin: PA2\ndir_pin: PA3\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z]\nstep_pin: PA4\ndir_pin: PA5\nrotation_distance: 8\nmicrosteps: 16\nposition_max: 200\n\
             [extruder]\nstep_pin: PA6\ndir_pin: PA7\nrotation_distance: 33.5\nmicrosteps: 16\n\
             nozzle_diameter: 0.4\nfilament_diameter: 1.75\nheater_pin: PB0\n\
             sensor_type: temperature_mcu\ncontrol: pid\npid_Kp: 1\npid_Ki: 0.1\npid_Kd: 10\n\
             min_temp: 0\nmax_temp: 250\nmin_extrude_temp: 0\n\
             [extruder_stepper my_extra_stepper]\n{section}\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n"
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

    fn the_stepper(printer: &Arc<Printer>) -> Arc<PrinterExtruderStepper> {
        printer
            .lookup_object_as::<PrinterExtruderStepper>("extruder_stepper my_extra_stepper")
            .expect("[extruder_stepper my_extra_stepper] is registered")
    }

    /// Every option the corpus's section carries is read — the undefined-
    /// option check is what fails when one is not.
    #[test]
    fn test_the_prefix_section_reads_every_corpus_option() {
        let printer = load_ok(&config(EXTRA_STEPPER));

        let stepper = the_stepper(&printer);
        assert_eq!(stepper.name(), "my_extra_stepper");
        assert_eq!(stepper.extruder_name(), "extruder");
        // The motor options went through `PrinterStepper`:
        // 28.2 mm / (200 full steps * 16 microsteps).
        let step_dist = 28.2 / (200.0 * 16.0);
        assert!((stepper.stepper.step_dist() - step_dist).abs() < 1e-12);
    }

    /// The pressure-advance options default as upstream's `ExtruderStepper`
    /// defines them (`kinematics/extruder.py:14-16`), and explicit values are
    /// taken as written — both reach the stepper's parameters.
    #[test]
    fn test_the_pressure_advance_option_defaults_follow_upstream() {
        let printer = load_ok(&config(EXTRA_STEPPER));
        let stepper = the_stepper(&printer);
        assert_eq!(stepper.pressure_advance.values(), (0.0, 0.040));

        let printer = load_ok(&config(&format!(
            "{EXTRA_STEPPER}pressure_advance: 0.05\npressure_advance_smooth_time: 0.02\n"
        )));
        let stepper = the_stepper(&printer);
        assert_eq!(stepper.pressure_advance.values(), (0.05, 0.02));
    }

    /// The bounds upstream puts on the two options (`minval=0.` /
    /// `above=0., maxval=.200`) are refused with the config wording.
    #[test]
    fn test_an_out_of_range_pressure_advance_option_is_refused() {
        let (_, result) = load(&config(&format!("{EXTRA_STEPPER}pressure_advance: -1\n")));
        let err = result.unwrap_err().to_string();
        assert!(err.contains("must have minimum of"), "{err}");

        let (_, result) = load(&config(&format!(
            "{EXTRA_STEPPER}pressure_advance_smooth_time: 0\n"
        )));
        let err = result.unwrap_err().to_string();
        assert!(err.contains("must be above"), "{err}");

        let (_, result) = load(&config(&format!(
            "{EXTRA_STEPPER}pressure_advance_smooth_time: 0.5\n"
        )));
        let err = result.unwrap_err().to_string();
        assert!(err.contains("must have maximum of"), "{err}");
    }

    /// Binding at connect resolves the `extruder` option to an `[extruder]` —
    /// a name that is not one is refused with upstream's exact complaint
    /// (`kinematics/extruder.py:61-62`).
    #[test]
    fn test_a_bad_extruder_binding_complains_the_way_upstream_does() {
        let printer = load_ok(&config(
            &EXTRA_STEPPER.replace("extruder: extruder", "extruder: bogus"),
        ));

        let err = the_stepper(&printer)
            .sync_to_extruder(&printer)
            .unwrap_err()
            .to_string();
        assert_eq!(err, "'bogus' is not a valid extruder.");
    }

    /// A good binding records the motion queue; an empty `extruder` detaches,
    /// as upstream's `sync_to_extruder` does for an empty name.
    #[test]
    fn test_a_good_binding_records_the_motion_queue_and_empty_detaches() {
        let printer = load_ok(&config(EXTRA_STEPPER));
        let stepper = the_stepper(&printer);
        stepper
            .sync_to_extruder(&printer)
            .expect("the name resolves");
        let status = PrinterObject::get_status(&*stepper, 0.0);
        assert_eq!(status["motion_queue"], "extruder");

        let printer = load_ok(&config(
            "extruder: \nstep_pin: PH5\ndir_pin: PH6\n\
                                       microsteps: 16\nrotation_distance: 28.2\n",
        ));
        let stepper = the_stepper(&printer);
        stepper.sync_to_extruder(&printer).expect("empty detaches");
        let status = PrinterObject::get_status(&*stepper, 0.0);
        assert!(status["motion_queue"].is_null(), "{status}");
    }

    fn ready_printer(text: &str) -> Arc<Printer> {
        let printer = load_ok(text);
        printer.send_event(&KlippyEvent::KlippyReady);
        printer
    }

    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`")
    }

    /// `SET_EXTRUDER_ROTATION_DISTANCE` by name: a zero is refused, a negative
    /// flips the direction, and the value is stored (upstream's wording).
    #[test]
    fn test_set_extruder_rotation_distance_by_name() {
        let printer = ready_printer(&config(EXTRA_STEPPER));
        let gcode = gcode(&printer);

        gcode
            .run_script_sync(
                "SET_EXTRUDER_ROTATION_DISTANCE EXTRUDER=my_extra_stepper DISTANCE=33.2",
            )
            .expect("a positive distance is accepted");
        let stepper = the_stepper(&printer);
        assert!((stepper.stepper.get_rotation_distance().0 - 33.2).abs() < 1e-9);

        let err = gcode
            .run_script_sync("SET_EXTRUDER_ROTATION_DISTANCE EXTRUDER=my_extra_stepper DISTANCE=0")
            .unwrap_err()
            .to_string();
        assert!(err.contains("Rotation distance can not be zero"), "{err}");

        // A negative distance is accepted with the direction flipped; the
        // stored distance is the absolute value, as upstream stores it.
        let (before_invert, orig) = stepper.stepper.get_dir_inverted();
        gcode
            .run_script_sync(
                "SET_EXTRUDER_ROTATION_DISTANCE EXTRUDER=my_extra_stepper DISTANCE=-33.1",
            )
            .expect("a negative distance is accepted");
        let (after_invert, orig2) = stepper.stepper.get_dir_inverted();
        assert_eq!(orig, orig2);
        assert_ne!(after_invert, before_invert);
        assert!((stepper.stepper.get_rotation_distance().0 - 33.1).abs() < 1e-9);
    }

    /// `SYNC_EXTRUDER_MOTION` by name: an empty `MOTION_QUEUE` detaches without
    /// error, a good name binds, a bad one complains with upstream's exact
    /// wording.
    #[test]
    fn test_sync_extruder_motion_by_name() {
        let printer = ready_printer(&config(EXTRA_STEPPER));
        let gcode = gcode(&printer);

        gcode
            .run_script_sync("SYNC_EXTRUDER_MOTION EXTRUDER=my_extra_stepper MOTION_QUEUE=")
            .expect("an empty motion queue detaches");
        let status = PrinterObject::get_status(&*the_stepper(&printer), 0.0);
        assert!(status["motion_queue"].is_null(), "{status}");

        gcode
            .run_script_sync("SYNC_EXTRUDER_MOTION EXTRUDER=my_extra_stepper MOTION_QUEUE=extruder")
            .expect("a good name binds");
        let status = PrinterObject::get_status(&*the_stepper(&printer), 0.0);
        assert_eq!(status["motion_queue"], "extruder");

        let err = gcode
            .run_script_sync("SYNC_EXTRUDER_MOTION EXTRUDER=my_extra_stepper MOTION_QUEUE=bogus")
            .unwrap_err()
            .to_string();
        assert_eq!(err, "'bogus' is not a valid extruder.");
    }

    /// The primary `[extruder]` registers the same two commands for its own
    /// name, so the corpus's `EXTRUDER=extruder` lines reach a handler.
    #[test]
    fn test_the_primary_extruder_registers_the_motion_commands() {
        let printer = ready_printer(&config(EXTRA_STEPPER));
        let gcode = gcode(&printer);

        gcode
            .run_script_sync("SET_EXTRUDER_ROTATION_DISTANCE EXTRUDER=extruder DISTANCE=33.2")
            .expect("the primary registers the command");
        gcode
            .run_script_sync("SYNC_EXTRUDER_MOTION EXTRUDER=extruder MOTION_QUEUE=")
            .expect("the primary registers the command");

        let extruder = printer
            .lookup_object_as::<PrinterExtruder>("extruder")
            .expect("[extruder] is registered");
        let status = PrinterObject::get_status(&*extruder, 0.0);
        assert!(status["motion_queue"].is_null(), "{status}");
    }

    /// The corpus sets pressure advance **on the extra stepper** by name
    /// (`SET_PRESSURE_ADVANCE EXTRUDER=my_extra_stepper`): its mux value is
    /// registered and keeps its own slots.
    #[test]
    fn test_set_pressure_advance_by_name_updates_only_this_stepper() {
        let printer = load_ok(&config(EXTRA_STEPPER));
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        gcode
            .run_script_sync("SET_PRESSURE_ADVANCE EXTRUDER=my_extra_stepper ADVANCE=0.02")
            .unwrap();

        let status = PrinterObject::get_status(&*the_stepper(&printer), 0.0);
        assert_eq!(status["pressure_advance"], 0.02);
        // The primary extruder keeps its own value.
        let extruder = printer
            .lookup_object_as::<PrinterExtruder>("extruder")
            .expect("[extruder] is registered");
        let status = PrinterObject::get_status(&*extruder, 0.0);
        assert_eq!(status["pressure_advance"], 0.0);
    }

    /// A negative advance or an out-of-band smooth time is refused the way
    /// upstream's `cmd_SET_PRESSURE_ADVANCE` bounds them.
    #[test]
    fn test_an_out_of_range_command_value_is_refused() {
        let printer = load_ok(&config(EXTRA_STEPPER));
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`");

        let err = gcode
            .run_script_sync("SET_PRESSURE_ADVANCE EXTRUDER=my_extra_stepper ADVANCE=-1")
            .unwrap_err()
            .to_string();
        assert!(err.contains("must have minimum of"), "{err}");

        let err = gcode
            .run_script_sync("SET_PRESSURE_ADVANCE EXTRUDER=my_extra_stepper SMOOTH_TIME=0.5")
            .unwrap_err()
            .to_string();
        assert!(err.contains("must have maximum of"), "{err}");
    }

    /// The extruder trapq the solver tests use: 0.3 s of extrusion at 10 mm/s in
    /// three 0.1 s segments, the first of which may not advance
    /// (`axes_r.y` is `can_pressure_advance`, `kinematics/extruder.py:221-232`).
    fn extruder_trapq() -> Trapq {
        let mut trapq = Trapq::new();
        let mut position = 0.0;
        for (print_time, can_advance) in [(0.0, false), (0.1, true), (0.2, true)] {
            trapq.append(
                print_time,
                0.0,
                0.1,
                0.0,
                Xyz::new(position, 0.0, 0.0),
                Xyz::new(1.0, f64::from(u8::from(can_advance)), 0.0),
                10.0,
                10.0,
                0.0,
            );
            position += 1.0;
        }
        trapq
    }

    /// The section's own `pressure_advance` reaches this stepper's solver: at a
    /// constant 10 mm/s the position leads the nominal extrusion by
    /// `pressure_advance * velocity` and the step generation window is the half
    /// smooth time — the same assertion the primary extruder's solver test makes
    /// (`itersolve`'s `test_pressure_advance_drives_the_extruder_steps`).
    #[test]
    fn test_a_configured_pressure_advance_leads_the_steppers_position() {
        let printer = load_ok(&config(&format!("{EXTRA_STEPPER}pressure_advance: 0.05\n")));
        let stepper = the_stepper(&printer);
        let mut host = stepper.stepper.build_host_stepper(0, MCU_FREQ);

        assert!(
            host.kinematics().pressure_advance().is_some(),
            "the section's parameters must be installed on its own solver"
        );
        assert_eq!(
            host.kinematics().effective_generation_window(),
            (0.02, 0.02)
        );

        host.generate(&extruder_trapq(), 0.2)
            .expect("the flush runs");
        assert!(
            (host.commanded_position() - 2.5).abs() < 0.02,
            "expected 2 mm nominal + the 0.5 mm lead, got {}",
            host.commanded_position()
        );
    }

    /// Without `pressure_advance` the section adds nothing: the solver keeps the
    /// plain position function — the parameters zero the smooth time when the
    /// advance is zero (`kin_extruder.c`'s `sa->half_smooth_time`, the solver's
    /// off switch, `:129-132`) — so the same trapq generates the same position at
    /// every flush point as a solver built with no parameters at all.
    #[test]
    fn test_an_unconfigured_pressure_advance_leaves_the_positions_alone() {
        let printer = load_ok(&config(EXTRA_STEPPER));
        let stepper = the_stepper(&printer);
        let scheduled = *stepper
            .pressure_advance
            .schedule()
            .last()
            .expect("the config entry");
        assert_eq!(scheduled.pressure_advance, 0.0);
        assert_eq!(
            scheduled.smooth_time, 0.0,
            "pa = 0 must zero the smooth time, or the solver would smooth"
        );
        let mut host = stepper.stepper.build_host_stepper(0, MCU_FREQ);

        let mut plain = Stepper::new(
            "plain_extruder",
            0,
            stepper.stepper.step_dist(),
            extruder_position_fn(),
            extruder_active_flags(),
            MCU_FREQ,
        );
        let trapq = extruder_trapq();
        for flush in [0.05, 0.1, 0.15, 0.2] {
            host.generate(&trapq, flush).expect("the flush runs");
            plain.generate(&trapq, flush).expect("the flush runs");
            assert_eq!(
                host.commanded_position(),
                plain.commanded_position(),
                "pressure advance off must not change the position at {flush} s"
            );
        }
        assert!(
            (host.commanded_position() - 2.0).abs() < 0.02,
            "expected the nominal 2 mm, got {}",
            host.commanded_position()
        );
    }

    /// `SET_PRESSURE_ADVANCE EXTRUDER=<section>` changes what status reports at
    /// once and puts the value in force when the look-ahead flushes — never on
    /// the moves already parked — the way the primary extruder does
    /// (`kinematics/extruder.py:67-88`).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_set_pressure_advance_lands_at_the_lookahead_flush() {
        let printer = load_ok(
            &config(&format!(
                "{EXTRA_STEPPER}pressure_advance: 0.05\npressure_advance_smooth_time: 0.04\n"
            ))
            .replace("kinematics: cartesian", "kinematics: none"),
        );
        printer.send_event(&KlippyEvent::KlippyReady);
        let toolhead = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .expect("the toolhead is registered");
        tokio::time::timeout(Duration::from_secs(30), toolhead.connect())
            .await
            .expect("the toolhead connects within the bound")
            .expect("the toolhead connects");
        let gcode = gcode(&printer);
        let stepper = the_stepper(&printer);
        let config_entry = |stepper: &PrinterExtruderStepper| {
            *stepper
                .pressure_advance
                .schedule()
                .last()
                .expect("the config entry")
        };
        assert_eq!(config_entry(&stepper).pressure_advance, 0.05);
        // A move still in the look-ahead: the change has a flush point to wait
        // for, so nothing the generator could already have stepped over moves.
        toolhead
            .move_to(Coord::new(0.0, 0.0, 0.0, 5.0), 10.0)
            .expect("an E-only move is accepted");

        tokio::time::timeout(
            Duration::from_secs(30),
            gcode.run_script("SET_PRESSURE_ADVANCE EXTRUDER=my_extra_stepper ADVANCE=0.1"),
        )
        .await
        .expect("the line applies within the bound")
        .expect("the line applies");

        // Reported at once …
        let status = PrinterObject::get_status(&*stepper, 0.0);
        assert_eq!(status["pressure_advance"], 0.1);
        // … while the solver keeps the value it was stepped with until the
        // look-ahead flushes.
        assert_eq!(config_entry(&stepper).pressure_advance, 0.05);

        // Flushing the look-ahead is the flush point: the new value takes over
        // from the end of the parked move, not from time zero.
        let move_end = toolhead.get_last_move_time();
        let scheduled = config_entry(&stepper);
        assert_eq!(scheduled.pressure_advance, 0.1);
        assert_eq!(scheduled.smooth_time, 0.04);
        assert!(move_end > 0.0, "the parked move must take time: {move_end}");
        assert_eq!(scheduled.print_time, move_end);
    }
}
