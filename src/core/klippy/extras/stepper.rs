//! `[stepper_x]` / `[stepper_y]` / `[stepper_z]` — one motor on one axis.
//!
//! Upstream builds these from `klippy/stepper.py`: `PrinterStepper` looks up the
//! step and direction pins, `MCU_stepper` owns the oid and the wire commands, and
//! `GenericPrinterRail` adds the axis range the kinematics needs. A stepper
//! section is **not** a printer object there (`objects/list` does not show
//! `stepper_x`); the toolhead reads the sections straight from the config.
//!
//! This port's loader needs every section claimed, so the same piece of work is
//! split across two layers:
//!
//! * this module is the `[stepper_*]` section: it parses the motor geometry,
//!   builds the [`McuStepper`] resource (which registers `config_stepper`), and
//!   exposes the rail range for the kinematics. The object is registered but
//!   [`PrinterObject::is_queryable`] is false, so `objects/list` still leaves it
//!   out, as upstream does;
//! * [`ToolHeadObject`](crate::core::klippy::extras::toolhead) takes the
//!   host-side [`Stepper`] this module builds at connect and drives it.
//!
//! # What is here
//!
//! | option | meaning |
//! |---|---|
//! | `step_pin` | the step pin (required) |
//! | `dir_pin` | the direction pin, same MCU as the step pin (required) |
//! | `rotation_distance` | millimetres per full motor rotation (required) |
//! | `microsteps` | microsteps per full step (required) |
//! | `full_steps_per_rotation` | full steps per rotation (default 200) |
//! | `gear_ratio` | `g1:g2` pairs multiplied into the step distance |
//! | `step_pulse_duration` | step pulse width in seconds (default 2 µs) |
//! | `position_min` / `position_max` | the axis range (required) |
//! | `position_endstop` | where the endstop sits (stored for FW6 homing) |
//!
//! Homing itself is FW6/F8: `endstop_pin`, `homing_speed` and the rest arrive
//! with `HomingState`. Until then an axis is homed only by
//! `SET_KINEMATIC_POSITION`.

use std::sync::{Arc, Mutex, MutexGuard, Weak};
use std::time::Duration;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::error::KlippyError;
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{X_AXIS, Y_AXIS, Z_AXIS};
use crate::core::klippy::mcu::{McuEndstop, McuStepper};
use crate::core::klippy::motion::{Axis, HomingInfo, Stepper};
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};

// The three cartesian axes. `[printer]` is a late section, so these are
// registered before it and it can look them up as it is built.
section!("stepper_x", order = 20, load = load_config);
section!("stepper_y", order = 20, load = load_config);
section!("stepper_z", order = 20, load = load_config);

/// The default pulse width upstream uses when the option is absent
/// (`klippy/stepper.py:80`).
const DEFAULT_STEP_PULSE_DURATION: f64 = 0.000_002;

/// How long the connect-time position read may take.
///
/// The same order as the other connect-time reads; a board that does not answer
/// only loses the alignment, not the connection.
const POSITION_TIMEOUT: Duration = Duration::from_secs(1);

/// One `[stepper_*]` section's rail parameters.
///
/// The subset of upstream's `GenericPrinterRail` the cartesian kinematics needs
/// today: the travel limits. The homing fields (`position_endstop` and the
/// speeds) are parsed and kept for FW6.
#[derive(Debug, Clone, Copy)]
pub struct RailParams {
    /// Minimum axis position.
    pub position_min: f64,
    /// Maximum axis position.
    pub position_max: f64,
    /// Where the endstop trips, used by homing (FW6).
    pub position_endstop: f64,
}

/// One configured `[stepper_x]` / `[stepper_y]` / `[stepper_z]`.
pub struct PrinterStepper {
    name: String,
    axis: Axis,
    /// Millimetres per step, after rotation distance, microsteps and gearing.
    step_dist: f64,
    /// The range and homing point the kinematics reads.
    params: RailParams,
    /// The firmware side: oid, pins and the wire commands.
    mcu_stepper: Arc<McuStepper>,
    /// The endstop this rail homes to, when the section names one.
    endstop: Option<Arc<McuEndstop>>,
    /// The homing parameters (`homing.py`'s input).
    homing: HomingInfo,
    /// The machine, to find this MCU's clock/offset at connect. `Weak` because
    /// the printer's registry owns this object.
    printer: Weak<Printer>,
    /// The host solver and compressor, built at connect when the oid and the MCU
    /// frequency exist.
    ///
    /// The toolhead's connect takes it out and owns it from then on, so this is
    /// `None` after the machine is up.
    inner: Mutex<Option<Stepper>>,
}

impl PrinterStepper {
    /// Build the section: parse the motor geometry and register the firmware
    /// stepper.
    ///
    /// # Errors
    /// Returns a config error naming the section when an option is missing,
    /// malformed, out of range, or names a pin the `pins` layer refuses.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let name = config.section().id.clone();
        let axis = axis_from_name(&identifier)?;

        let step_pin = config.get("step_pin", None)?;
        let dir_pin = config.get("dir_pin", None)?;
        let rotation_distance =
            config.get_float_bounded("rotation_distance", None, None, None, Some(0.0), None)?;
        let microsteps = config.get_int_bounded("microsteps", None, Some(1), None)?;
        let full_steps = config.get_int("full_steps_per_rotation", Some(200))?;
        if full_steps < 1 || full_steps % 4 != 0 {
            return Err(ConfigError::new(format!(
                "full_steps_per_rotation invalid in section '{identifier}'"
            )));
        }
        let gear_ratio = config
            .get_list_of_lists("gear_ratio", ',', ':', 2)?
            .into_iter()
            .map(|pair| {
                let first = pair[0].trim().parse::<f64>().map_err(|_| {
                    ConfigError::new(format!(
                        "Unable to parse option 'gear_ratio' in section '{identifier}'"
                    ))
                })?;
                let second = pair[1].trim().parse::<f64>().map_err(|_| {
                    ConfigError::new(format!(
                        "Unable to parse option 'gear_ratio' in section '{identifier}'"
                    ))
                })?;
                if second == 0.0 {
                    return Err(ConfigError::new(format!(
                        "Option 'gear_ratio' in section '{identifier}' must not divide by zero"
                    )));
                }
                Ok(first / second)
            })
            .collect::<Result<Vec<f64>, ConfigError>>()?
            .into_iter()
            .product::<f64>()
            .max(f64::MIN_POSITIVE);
        let step_pulse_duration = config.get_float_bounded(
            "step_pulse_duration",
            Some(DEFAULT_STEP_PULSE_DURATION),
            Some(0.0),
            Some(0.001),
            None,
            None,
        )?;

        let position_min = config.get_float("position_min", Some(0.0))?;
        let position_max =
            config.get_float_bounded("position_max", None, None, None, Some(position_min), None)?;
        let position_endstop = config.get_float("position_endstop", Some(position_min))?;
        if position_endstop < position_min || position_endstop > position_max {
            return Err(ConfigError::new(format!(
                "position_endstop in section '{identifier}' must be between position_min and position_max"
            )));
        }

        // `rotation_distance` is millimetres per full rotation; the divisor is
        // full steps times microsteps times any gearing
        // (`parse_step_distance`, `klippy/stepper.py:307-323`).
        let step_dist = rotation_distance / (full_steps as f64 * microsteps as f64 * gear_ratio);

        // Homing parameters and the endstop. `endstop_pin` is optional here:
        // a `[stepper_*]` with no endstop still loads (upstream requires it for
        // a cartesian rail; `G28` will report the missing endstop instead).
        let homing = read_homing_info(
            config,
            &identifier,
            position_min,
            position_max,
            position_endstop,
        )?;

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        // The step pin's `!` is upstream's `invert_step` (`0`/`1`); the direction
        // pin's `!` is applied on the wire by the resource.
        let mcu_stepper = pins
            .setup_stepper(&step_pin, &dir_pin, step_pulse_duration)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        let endstop = match config.get_str("endstop_pin") {
            Some(pin) => Some(
                pins.setup_endstop(&pin, None)
                    .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?,
            ),
            None => None,
        };
        // Register the stepper with the endstop's trigger dispatch now, at load:
        // the dispatch creates a per-MCU trsync (and the config callback that
        // reserves its oid) before the configuration is built. This is also
        // where upstream rejects a shared axis whose steppers are on different
        // MCUs (`TriggerDispatch.add_stepper`).
        if let Some(endstop) = &endstop {
            endstop
                .dispatch()
                .add_stepper(
                    mcu_stepper.chip().clone(),
                    Arc::downgrade(&mcu_stepper),
                    &name,
                )
                .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        }

        Ok(Self {
            name,
            axis,
            step_dist,
            params: RailParams {
                position_min,
                position_max,
                position_endstop,
            },
            mcu_stepper,
            endstop,
            homing,
            printer: Arc::downgrade(printer),
            inner: Mutex::new(None),
        })
    }

    /// The section's name (`stepper_x`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The axis this stepper drives.
    pub fn axis(&self) -> Axis {
        self.axis
    }

    /// Millimetres per step.
    pub fn step_dist(&self) -> f64 {
        self.step_dist
    }

    /// The rail range and homing point.
    pub fn params(&self) -> RailParams {
        self.params
    }

    /// The endstop this rail homes to, if the section named one.
    pub fn endstop(&self) -> Option<&Arc<McuEndstop>> {
        self.endstop.as_ref()
    }

    /// The homing parameters (`homing.py`'s input).
    pub fn homing_info(&self) -> HomingInfo {
        self.homing
    }

    /// The firmware stepper resource.
    pub fn mcu_stepper(&self) -> &Arc<McuStepper> {
        &self.mcu_stepper
    }

    /// Take the host-side stepper the toolhead will drive.
    ///
    /// `None` before connect, or after the toolhead has taken it. The toolhead
    /// calls this once, in its own connect.
    pub fn take_stepper(&self) -> Option<Stepper> {
        self.lock().take()
    }

    /// This MCU's print-time-to-clock offset, found through the chip name.
    ///
    /// Zero when the MCU object cannot be found (a standalone stepper with no
    /// `[mcu]` object), which is the primary's offset anyway.
    /// This MCU's print-time `(offset, frequency)` mapping, found through the
    /// chip name.
    ///
    /// `None` when the MCU object cannot be found (a standalone stepper with no
    /// `[mcu]` object); the caller then uses `(0.0, mcu_freq)`.
    fn time_mapping(&self, mcu: &Arc<crate::core::klippy::mcu::Mcu>) -> Option<(f64, f64)> {
        self.printer.upgrade().and_then(|printer| {
            printer
                .lookup_objects_as::<crate::core::klippy::mcu::McuObject>(Some("mcu"))
                .into_iter()
                .find(|(_, object)| object.name() == mcu.name())
                .map(|(_, object)| object.time_mapping())
        })
    }

    fn lock(&self) -> MutexGuard<'_, Option<Stepper>> {
        self.inner
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl PrinterObject for PrinterStepper {
    /// Never called through the API: a stepper section is not a printer object
    /// upstream, so it is not queryable here either.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }

    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            let config_error = |message: String| {
                KlippyError::Config(ConfigError::new(format!("{}: {message}", self.name)))
            };
            let mcu = self
                .mcu_stepper
                .mcu()
                .ok_or_else(|| config_error("MCU is not connected".to_string()))?;
            let freq = mcu
                .clock_freq()
                .map_err(|err| config_error(err.to_string()))?;
            let oid = self
                .mcu_stepper
                .oid()
                .map_err(|err| config_error(err.to_string()))?;

            let mut stepper = Stepper::cartesian(
                self.name.clone(),
                u32::from(oid),
                self.step_dist,
                self.axis,
                freq,
            );
            // Read the board's step counter and align the solver with it, as
            // upstream's `_query_mcu_position` does at connect
            // (`klippy/stepper.py:212-228`). This is what makes the host's
            // position agree with the firmware's after a restart or a reset.
            let steps = self
                .mcu_stepper
                .query_position(POSITION_TIMEOUT)
                .await
                .map_err(|err| config_error(err.to_string()))?;
            stepper.kinematics_mut().commanded_pos = f64::from(steps) * self.step_dist;
            // Point the compressor at this MCU's clock domain. The `[mcu]`
            // object already built the estimate and the `SecondarySync` offset at
            // its own connect (which runs before any stepper); find it by chip
            // name and use it. A secondary's mapping is recalibrated later, and
            // the toolhead's flush loop re-reads it before generating.
            let (offset, mapping_freq) = self.time_mapping(&mcu).unwrap_or((0.0, freq));
            stepper.compressor_mut().set_time(offset, mapping_freq);
            // Record where the firmware's counter is (`set_last_position` only
            // flushes the pending step — none yet — and records the position).
            if let Some(clock) = mcu.estimated_clock() {
                stepper
                    .compressor_mut()
                    .set_last_position(clock, i64::from(steps))
                    .map_err(|err| config_error(err.to_string()))?;
            }

            *self.lock() = Some(stepper);
            Ok(())
        })
    }
}

impl std::fmt::Debug for PrinterStepper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterStepper")
            .field("name", &self.name)
            .field("step_dist", &self.step_dist)
            .finish_non_exhaustive()
    }
}

/// The factory the three section declarations name.
pub(crate) fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PrinterStepper::new(config, printer)?))
}

/// Parse the homing parameters of a `[stepper_*]` rail
/// (`GenericPrinterRail.__init__`, `klippy/stepper.py:347-390`).
fn read_homing_info(
    config: &ConfigWrapper,
    identifier: &str,
    position_min: f64,
    position_max: f64,
    position_endstop: f64,
) -> Result<HomingInfo, ConfigError> {
    let speed = config.get_float_bounded("homing_speed", Some(5.0), None, None, Some(0.0), None)?;
    let second_homing_speed = config.get_float("second_homing_speed", Some(speed / 2.0))?;
    let retract_speed = config.get_float("homing_retract_speed", Some(speed))?;
    let retract_dist = config.get_float("homing_retract_dist", Some(5.0))?;
    let positive_dir = match config.get_optional_bool("homing_positive_dir")? {
        Some(positive) => positive,
        None => {
            // Infer from where the endstop sits: near the low end means homing
            // moves negative, near the high end positive, and anywhere in the
            // middle is ambiguous.
            let axis_len = position_max - position_min;
            if position_endstop <= position_min + axis_len / 4.0 {
                false
            } else if position_endstop >= position_max - axis_len / 4.0 {
                true
            } else {
                return Err(ConfigError::new(format!(
                    "Unable to infer homing_positive_dir in section '{identifier}'"
                )));
            }
        }
    };
    if (positive_dir && position_endstop == position_min)
        || (!positive_dir && position_endstop == position_max)
    {
        return Err(ConfigError::new(format!(
            "Invalid homing_positive_dir / position_endstop in '{identifier}'"
        )));
    }
    Ok(HomingInfo {
        speed,
        position_endstop,
        retract_speed,
        retract_dist,
        positive_dir,
        second_homing_speed,
    })
}

/// The axis a section name selects: `stepper_x` → [`Axis::X`].
fn axis_from_name(identifier: &str) -> Result<Axis, ConfigError> {
    match identifier.strip_prefix("stepper_") {
        Some("x") => Ok(Axis::X),
        Some("y") => Ok(Axis::Y),
        Some("z") => Ok(Axis::Z),
        _ => Err(ConfigError::new(format!(
            "Unable to map section '{identifier}' to a cartesian axis"
        ))),
    }
}

/// The axis index [`PrinterStepper`] reports (`mathutil`'s constants).
pub fn axis_index(axis: Axis) -> usize {
    match axis {
        Axis::X => X_AXIS,
        Axis::Y => Y_AXIS,
        Axis::Z => Z_AXIS,
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::reactor::ManualReactor;

    /// Load a config text the way the host does.
    fn load(text: &str) -> (Arc<Printer>, Result<(), ConfigError>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = crate::core::klippy::config::Config::from_text(text)
            .expect("the test config parses")
            .0;
        let result = printer.load_config(&config);
        (printer, result)
    }

    /// An `[mcu]` plus an X stepper with the given extra options.
    fn config_with_x(extra: &str) -> String {
        format!(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\n\
             rotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n{extra}"
        )
    }

    #[test]
    fn test_section_names_map_to_axes() {
        assert_eq!(axis_from_name("stepper_x").unwrap(), Axis::X);
        assert_eq!(axis_from_name("stepper_y").unwrap(), Axis::Y);
        assert_eq!(axis_from_name("stepper_z").unwrap(), Axis::Z);
        assert!(axis_from_name("stepper_e").is_err());
    }

    #[test]
    fn test_axis_indexes_are_the_mathutil_ones() {
        assert_eq!(axis_index(Axis::X), X_AXIS);
        assert_eq!(axis_index(Axis::Y), Y_AXIS);
        assert_eq!(axis_index(Axis::Z), Z_AXIS);
    }

    #[test]
    fn test_the_step_distance_follows_the_geometry() {
        // 40 mm per rotation, 200 full steps, 16 microsteps, no gearing:
        // 40 / (200 * 16) = 0.0125 mm per step.
        let step_dist: f64 = 40.0 / (200.0 * 16.0 * 1.0);
        assert!((step_dist - 0.0125).abs() < 1e-12);
    }

    #[test]
    fn test_a_stepper_section_loads_as_a_stepper_object() {
        let (printer, result) = load(&config_with_x(""));

        result.unwrap();
        let stepper = printer
            .lookup_object_as::<PrinterStepper>("stepper_x")
            .expect("the section registered a stepper object");
        assert_eq!(stepper.name(), "stepper_x");
        assert_eq!(stepper.axis(), Axis::X);
        assert!((stepper.step_dist() - 0.0125).abs() < 1e-12);
        assert_eq!(stepper.params().position_min, 0.0);
        assert_eq!(stepper.params().position_max, 200.0);
        // Registered but not queryable, as upstream's non-object stepper is.
        assert!(!printer
            .queryable_objects()
            .contains(&"stepper_x".to_string()));
        // No `endstop_pin`: the rail has no endstop yet.
        assert!(stepper.endstop().is_none());
    }

    #[test]
    fn test_an_endstop_pin_builds_the_rail_endstop_and_homing_info() {
        let (printer, result) = load(&config_with_x("endstop_pin: PA2\n"));

        result.unwrap();
        let stepper = printer
            .lookup_object_as::<PrinterStepper>("stepper_x")
            .unwrap();
        assert!(stepper.endstop().is_some());
        let info = stepper.homing_info();
        assert_eq!(info.position_endstop, 0.0);
        // The endstop sits at the low end, so homing moves negative.
        assert!(!info.positive_dir);
        assert_eq!(info.speed, 5.0);
        assert_eq!(info.second_homing_speed, 2.5);
        assert_eq!(info.retract_dist, 5.0);
    }

    #[test]
    fn test_an_endstop_in_the_middle_cannot_infer_the_direction() {
        let (_, result) = load(&config_with_x("endstop_pin: PA2\nposition_endstop: 100\n"));

        let err = result.unwrap_err().to_string();
        assert!(err.contains("Unable to infer homing_positive_dir"), "{err}");
    }

    #[test]
    fn test_a_gear_ratio_divides_into_the_step_distance() {
        let (printer, result) = load(&config_with_x("gear_ratio: 2:1\n"));

        result.unwrap();
        let stepper = printer
            .lookup_object_as::<PrinterStepper>("stepper_x")
            .unwrap();
        assert!((stepper.step_dist() - 0.00625).abs() < 1e-12);
    }

    #[test]
    fn test_a_missing_pin_names_the_section() {
        let (_, result) = load(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\n\
             rotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n",
        );

        let err = result.unwrap_err().to_string();
        assert!(err.contains("dir_pin"), "{err}");
    }

    #[test]
    fn test_pins_on_different_mcus_are_refused() {
        let (_, result) = load(
            "[mcu]\nserial: /dev/a\n\
             [mcu zboard]\nserial: /dev/b\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: zboard:PA1\n\
             rotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n",
        );

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("Stepper dir pin must be on same mcu as step pin"),
            "{err}"
        );
    }

    #[test]
    fn test_a_position_endstop_outside_the_range_is_refused() {
        let (_, result) = load(&config_with_x("position_endstop: 250\n"));

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("must be between position_min and position_max"),
            "{err}"
        );
    }
}
