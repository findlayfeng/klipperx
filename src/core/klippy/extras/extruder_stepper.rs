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
//! registers the stepper's own `SET_PRESSURE_ADVANCE` mux value, and binds to
//! the extruder at connect, complaining with upstream's wording when the name
//! is not an extruder.
//!
//! What is still open (tracked as the H10 motion-sync gap): the bound
//! extruder's trapq is allocated by the toolhead, which connects **after** this
//! generic section, and only the toolhead adds a host stepper to the step
//! generation — so the binding is recorded (status `motion_queue`) but the
//! stepper does not yet follow the extruder's motion. `SET_EXTRUDER_ROTATION_DISTANCE`
//! records its new distance and direction but does not rebuild the solver; the
//! corpus's fake firmware only needs the commands to run without error.

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::error::KlippyError;
use crate::core::klippy::extras::extruder::PrinterExtruder;
use crate::core::klippy::extras::stepper::PrinterStepper;
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::motion::itersolve::{extruder_active_flags, extruder_position_fn, Axis};
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};

// Only the prefix form exists upstream (`extruder_stepper.py:23`).
section!("extruder_stepper", order = 20, prefix = load_config_prefix);

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
    /// `pressure_advance` from the config, applied at connect (upstream's
    /// `config_pa`).
    config_pa: f64,
    /// `pressure_advance_smooth_time` from the config (upstream's
    /// `config_smooth_time`).
    config_smooth_time: f64,
    /// The live values `SET_PRESSURE_ADVANCE` writes and status reports
    /// (`Arc` so the mux handler shares the same slots).
    pressure_advance: Arc<Mutex<f64>>,
    pressure_advance_smooth_time: Arc<Mutex<f64>>,
    /// `motion_queue`: the extruder this stepper is bound to once connected.
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

        let extruder_stepper = Self {
            name,
            extruder_name,
            stepper,
            config_pa,
            config_smooth_time,
            pressure_advance: Arc::new(Mutex::new(0.0)),
            pressure_advance_smooth_time: Arc::new(Mutex::new(0.0)),
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

    /// Register `SET_PRESSURE_ADVANCE EXTRUDER=<name>` — the stepper's own mux
    /// value (`kinematics/extruder.py:32-34`).
    ///
    /// `SYNC_EXTRUDER_MOTION` and `SET_EXTRUDER_ROTATION_DISTANCE` are not
    /// registered; the dispatcher reports them as unknown commands (see the
    /// module docs).
    fn register_commands(&self, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let advance = Arc::clone(&self.pressure_advance);
        let smooth = Arc::clone(&self.pressure_advance_smooth_time);
        let handler: CommandHandler =
            sync(move |gcmd: &GcodeCommand| cmd_set_pressure_advance(gcmd, &advance, &smooth));
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

        // The motion-sync commands every extruder stepper registers
        // (`kinematics/extruder.py:37-42`): the rotation distance is set on the
        // wrapped stepper, the motion queue on this object's binding.
        {
            let stepper = Arc::clone(&self.stepper);
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
                    &["DISTANCE"],
                )
                .map_err(ConfigError::new)?;
        }
        {
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
    /// The trapq repointing lives in the toolhead (see the module docs); here
    /// the binding is validated and recorded as the status's `motion_queue`.
    ///
    /// # Errors
    /// Upstream's wording when the name does not resolve to an extruder.
    pub(crate) fn sync_to_extruder(&self, printer: &Arc<Printer>) -> Result<(), KlippyError> {
        if self.extruder_name.is_empty() {
            *lock(&self.motion_queue) = None;
            return Ok(());
        }
        printer
            .lookup_object_as::<PrinterExtruder>(&self.extruder_name)
            .ok_or_else(|| {
                KlippyError::Config(ConfigError::new(format!(
                    "'{}' is not a valid extruder.",
                    self.extruder_name
                )))
            })?;
        *lock(&self.motion_queue) = Some(self.extruder_name.clone());
        Ok(())
    }
}

impl PrinterObject for PrinterExtruderStepper {
    /// Upstream's `ExtruderStepper.get_status`: pressure advance, smooth time
    /// and the motion queue (`kinematics/extruder.py:43-46`).
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({
            "pressure_advance": *lock(&self.pressure_advance),
            "smooth_time": *lock(&self.pressure_advance_smooth_time),
            "motion_queue": lock(&self.motion_queue).clone(),
        })
    }

    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            // The host stepper builds its solver here, as every `[stepper_*]`
            // does at its own connect.
            self.stepper.connect().await?;
            let printer = self.printer.upgrade().ok_or_else(|| {
                KlippyError::Internal("the printer dropped before connect".to_string())
            })?;
            // Upstream's `_handle_connect` applies the configured values before
            // `PrinterExtruderStepper.handle_connect` binds the extruder.
            *lock(&self.pressure_advance) = self.config_pa;
            *lock(&self.pressure_advance_smooth_time) = self.config_smooth_time;
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

/// `SET_PRESSURE_ADVANCE`'s body for one extra stepper: record both values and
/// report them (upstream `ExtruderStepper.cmd_SET_PRESSURE_ADVANCE`,
/// `kinematics/extruder.py:98-107`).
fn cmd_set_pressure_advance(
    gcmd: &GcodeCommand,
    advance: &Arc<Mutex<f64>>,
    smooth_time: &Arc<Mutex<f64>>,
) -> Result<(), CommandError> {
    let parse = |raw: &str| raw.parse::<f64>().ok();
    let value = gcmd.get(
        "ADVANCE",
        Some(*lock(advance)),
        parse,
        Some(0.0),
        None,
        None,
        None,
    )?;
    let smooth = gcmd.get(
        "SMOOTH_TIME",
        Some(*lock(smooth_time)),
        parse,
        Some(0.0),
        Some(0.200),
        None,
        None,
    )?;
    *lock(advance) = value;
    *lock(smooth_time) = smooth;
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
///
/// # Errors
/// Upstream's wording when `DISTANCE` is zero, plus the parameter errors.
pub(crate) fn cmd_set_extruder_rotation_distance(
    gcmd: &GcodeCommand,
    name: &str,
    stepper: &PrinterStepper,
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
        // (`kinematics/extruder.py:120-122`); the solver half is the H10 gap.
        stepper.set_rotation_distance(distance);
        stepper.set_dir_inverted(next_invert_dir);
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

/// `SYNC_EXTRUDER_MOTION`'s body for one extruder stepper (upstream
/// `cmd_SYNC_EXTRUDER_MOTION`, `kinematics/extruder.py:133-137`).
///
/// An empty `MOTION_QUEUE` detaches — upstream's `sync_to_extruder("")` branch,
/// which must **not** be treated as an invalid name; any other value must name
/// an `[extruder]`.
///
/// # Errors
/// Upstream's wording when the name does not resolve to an extruder.
pub(crate) fn cmd_sync_extruder_motion(
    gcmd: &GcodeCommand,
    name: &str,
    printer: &Weak<Printer>,
    motion_queue: &Mutex<Option<String>>,
) -> Result<(), CommandError> {
    let ename = gcmd.get_str_default("MOTION_QUEUE", "");
    let printer = printer
        .upgrade()
        .ok_or_else(|| CommandError::new("printer is gone"))?;
    if ename.is_empty() {
        *lock(motion_queue) = None;
    } else {
        if printer
            .lookup_object_as::<PrinterExtruder>(&ename)
            .is_none()
        {
            return Err(CommandError::new(format!(
                "'{ename}' is not a valid extruder."
            )));
        }
        *lock(motion_queue) = Some(ename.clone());
    }
    gcmd.respond_info(&format!("Extruder '{name}' now syncing with '{ename}'"));
    Ok(())
}

/// The factory the section declaration names (`extruder_stepper.py:23`).
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PrinterExtruderStepper::new(config, printer)?))
}

/// The wrapped stepper's position at a past print time (upstream's
/// `PrinterExtruderStepper.find_past_position`). The host stepper is owned by
/// the toolhead after connect and exposes no position, so this reports
/// `0.0` — the same H10 motion-sync gap the module docs describe.
impl PrinterExtruderStepper {
    pub fn find_past_position(&self, _print_time: f64) -> f64 {
        0.0
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
    use crate::core::klippy::reactor::ManualReactor;

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
    /// defines them, and explicit values are taken as written.
    #[test]
    fn test_the_pressure_advance_option_defaults_follow_upstream() {
        let printer = load_ok(&config(EXTRA_STEPPER));
        let stepper = the_stepper(&printer);
        assert_eq!(stepper.config_pa, 0.0);
        assert_eq!(stepper.config_smooth_time, 0.040);

        let printer = load_ok(&config(&format!(
            "{EXTRA_STEPPER}pressure_advance: 0.05\npressure_advance_smooth_time: 0.02\n"
        )));
        let stepper = the_stepper(&printer);
        assert_eq!(stepper.config_pa, 0.05);
        assert_eq!(stepper.config_smooth_time, 0.02);
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
}
