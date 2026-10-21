//! `z_thermal_adjust` — compensate Z for the thermal expansion of the frame.
//!
//! Upstream's `klippy/extras/z_thermal_adjust.py`. A temperature sensor (one
//! coupled to a vertical section of the frame) is read on the sensor's own
//! report cadence, its reading is low-pass filtered, and the difference from a
//! reference temperature — taken at the end of the last Z homing — is turned
//! into a Z offset that every move carries:
//!
//! - [`ZThermalAdjust::position`] (upstream's `get_position`) **subtracts** the
//!   standing offset, so the g-code coordinate keeps reading the uncompensated
//!   Z;
//! - [`ZThermalAdjust::move_to`] (upstream's `move`) **adds** it before the
//!   chain below the transform queues the move.
//!
//! | option | default | role |
//! |---|---|---|
//! | `temp_coeff` | `0.` | mm/°C; **negative** of the offset per °C |
//! | `smooth_time` | `2.` | low-pass window (`adj = min(dt / smooth_time, 1)`) |
//! | `z_adjust_off_above` | `99999999.` | Z above which the offset stops updating |
//! | `max_z_adjustment` | `99999999.` | clamp on the absolute offset |
//! | `sensor_type` / `sensor_pin` | — | the sensor, built by `heaters` |
//! | `min_temp` / `max_temp` | — | the sensor's range (required) |
//! | `gcode_id` | — | the `M105` slot, if any |
//!
//! # The filter
//!
//! Upstream does not run a timer: `temperature_callback` is the sensor's own
//! report callback, so it samples as often as the sensor reports. Each reading
//! moves the smoothed value a fraction of the way to the raw one:
//! `adj_time = min((read_time - last_temp_time) / smooth_time, 1)` and
//! `smoothed += (temp - smoothed) * adj_time`. A gap longer than `smooth_time`
//! jumps straight to the reading.
//!
//! # The transform chain
//!
//! Registration happens at `klippy:connect` — after the toolhead and the
//! steppers exist — and **takes** `gcode_move`'s transform slot (`force`), as
//! upstream's `set_move_transform(self, force=True)` does. The occupant it
//! displaces (`bed_mesh`, `bed_tilt`, …) becomes the chain below this one, the
//! way `exclude_object` chains: forwarding goes to it, and [`position`] reads
//! through it. Nothing unregisters the transform, as upstream never does
//! either.
//!
//! [`position`]: MoveTarget::position

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};
use tracing::error;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::gcode_move::{self, GCodeMove, MoveTarget, GCODE_MOVE_OBJECT};
use crate::core::klippy::extras::heaters::{self, GcodeTempSensor, Sensor};
use crate::core::klippy::extras::stepper::PrinterStepper;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{self, CommandError, GCodeDispatch, GcodeCommand, GCODE_OBJECT};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{Coord, Z_AXIS};
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("z_thermal_adjust", order = 30, load = load_config);

/// `KELVIN_TO_CELSIUS`, upstream's lower bound for every temperature here.
const KELVIN_TO_CELSIUS: f64 = -273.15;

/// Upstream's "no limit" default for `z_adjust_off_above`/`max_z_adjustment`.
const UNLIMITED: f64 = 99_999_999.0;

/// The toolhead's object name (`[printer]` is registered as `toolhead`).
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The stepper whose step distance is the minimum adjustment upstream applies.
const Z_STEPPER: &str = "stepper_z";

/// The Z axis index (`mathutil::Z_AXIS`, named here for the reads below).
const Z: usize = Z_AXIS;

/// Upstream's `SET_Z_THERMAL_ADJUST` help text.
const SET_Z_THERMAL_ADJUST_HELP: &str = "Set/query Z Thermal Adjust parameters.";

/// Everything the sensor callback and the move transform both touch
/// (upstream's scattered `self.*` fields, kept under one lock because the
/// callback may run on the sensor's thread while a move runs on the reactor's).
#[derive(Debug)]
struct State {
    /// The last raw reading and when it came (upstream `last_temp` /
    /// `last_temp_time`).
    last_temp: f64,
    last_temp_time: f64,
    /// The filtered reading, in °C.
    smoothed_temp: f64,
    /// The extremes the filtered reading has reached.
    measured_min: f64,
    measured_max: f64,
    /// mm/°C, the negative of the offset per °C (`TEMP_COEFF` rewrites it).
    temp_coeff: f64,
    /// The temperature the offset is measured against (zeroed by Z homing).
    ref_temperature: f64,
    /// Whether `REF_TEMP` set the reference by hand instead of a homing.
    ref_temp_override: bool,
    /// The offset the moves carry, and the one the last move carried.
    z_adjust_mm: f64,
    last_z_adjust_mm: f64,
    /// `ENABLE=0` stops applying the offset at all.
    adjust_enable: bool,
    /// The last **g-code** position, which is what tells an extrusion-only
    /// move from a travel.
    last_position: Coord,
    /// Minimum change worth applying; `stepper_z`'s step distance once
    /// connected (upstream reads it from the kinematics at connect).
    z_step_dist: f64,
}

impl State {
    fn new(temp_coeff: f64) -> Self {
        Self {
            last_temp: 0.0,
            last_temp_time: 0.0,
            smoothed_temp: 0.0,
            measured_min: 0.0,
            measured_max: 0.0,
            temp_coeff,
            ref_temperature: 0.0,
            ref_temp_override: false,
            z_adjust_mm: 0.0,
            last_z_adjust_mm: 0.0,
            adjust_enable: true,
            last_position: Coord::default(),
            z_step_dist: 0.0,
        }
    }
}

/// One `[z_thermal_adjust]` section (`z_thermal_adjust.py:ZThermalAdjuster`).
pub struct ZThermalAdjust {
    /// The machine, to find `gcode_move`/`toolhead`/`stepper_z` when the
    /// transform registers and when the command re-anchors the position.
    printer: Weak<Printer>,
    /// The filtered reading and the offset, shared with the sensor callback.
    state: Arc<Mutex<State>>,
    /// `z_adjust_off_above`: moves at or above this Z keep the last offset.
    off_above_z: f64,
    /// `max_z_adjustment`: the clamp on the offset's absolute value.
    max_z_adjust_mm: f64,
    /// `1. / smooth_time`, the filter's per-second gain.
    inv_smooth_time: f64,
    /// The sensor the readings come from. Held (not just a weak handle) so the
    /// ADC/report callback that drives it keeps upgrading, as
    /// `temperature_sensor` does.
    #[allow(dead_code)]
    sensor: Arc<dyn Sensor>,
    /// The toolhead captured at `klippy:connect` — the chain's end when the
    /// g-code slot was empty (upstream's `self.toolhead`).
    target: Mutex<Option<Arc<dyn MoveTarget>>>,
    /// The transform this one displaced from `gcode_move`'s slot (upstream
    /// `next_transform`, `z_thermal_adjust.py:44`); `None` while the slot was
    /// empty when this one took it.
    next_transform: Mutex<Option<Arc<dyn MoveTarget>>>,
}

impl ZThermalAdjust {
    /// Read the section, build its sensor and register it with `heaters`
    /// (`ZThermalAdjuster.__init__`).
    ///
    /// # Errors
    /// When an option is malformed or out of range, the sensor type is
    /// unknown, or the section's sensor name is already registered.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let temp_coeff =
            config.get_float_bounded("temp_coeff", Some(0.0), Some(-1.0), Some(1.0), None, None)?;
        let off_above_z = config.get_float("z_adjust_off_above", Some(UNLIMITED))?;
        let max_z_adjust_mm = config.get_float("max_z_adjustment", Some(UNLIMITED))?;
        let smooth_time =
            config.get_float_bounded("smooth_time", Some(2.0), None, None, Some(0.0), None)?;
        // Upstream reads both without a default, so a `[z_thermal_adjust]`
        // section must name its sensor's range (`Config_Reference.md`).
        let min_temp = config.get_float_bounded(
            "min_temp",
            None,
            Some(KELVIN_TO_CELSIUS),
            None,
            None,
            None,
        )?;
        let max_temp =
            config.get_float_bounded("max_temp", None, None, None, Some(min_temp), None)?;

        let heaters = heaters::ensure(printer)?;
        let sensor = heaters.setup_sensor(config, printer)?;
        sensor.setup_minmax(min_temp, max_temp);

        let inv_smooth_time = 1.0 / smooth_time;
        let state = Arc::new(Mutex::new(State::new(temp_coeff)));
        let callback_state = Arc::clone(&state);
        sensor.setup_callback(Box::new(move |read_time, temp| {
            temperature_callback(&callback_state, inv_smooth_time, read_time, temp);
        }));

        let object = Arc::new(Self {
            printer: Arc::downgrade(printer),
            state,
            off_above_z,
            max_z_adjust_mm,
            inv_smooth_time,
            sensor,
            target: Mutex::new(None),
            next_transform: Mutex::new(None),
        });
        // Upstream's `pheaters.register_sensor(config, self)`: the section's
        // name joins `TEMPERATURE_WAIT`, and `gcode_id` (when present) puts the
        // filtered reading into `M105`.
        heaters.register_sensor(
            config,
            Some(Arc::clone(&object) as Arc<dyn GcodeTempSensor>),
            None,
        )?;
        Ok(object)
    }

    /// The events upstream's `__init__` subscribes to
    /// (`z_thermal_adjust.py:27-31`).
    fn register_handlers(self: &Arc<Self>, printer: &Arc<Printer>) {
        printer.register_event_handler(
            KlippyEvent::KlippyConnect,
            Box::new({
                let object = Arc::clone(self);
                move |_| ZThermalAdjust::handle_connect(&object)
            }),
        );
        printer.register_event_handler(
            KlippyEvent::HomingHomeRailsEnd {
                axes: Vec::new(),
                homing: crate::core::klippy::motion::HomingHandle::new(),
            },
            Box::new({
                let object = Arc::clone(self);
                move |event| {
                    if let KlippyEvent::HomingHomeRailsEnd { axes, .. } = event {
                        object.handle_home_rails_end(axes);
                    }
                }
            }),
        );
    }

    /// Upstream's `handle_connect`: take `gcode_move`'s transform slot
    /// (`force=True`) and pull `stepper_z`'s step distance.
    fn handle_connect(object: &Arc<Self>) {
        let Some(printer) = object.printer.upgrade() else {
            return;
        };
        if let Some(toolhead) = printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT) {
            *object.target.lock().unwrap_or_else(|p| p.into_inner()) =
                Some(Arc::new(ToolheadMove(toolhead)));
        }
        if let Some(stepper) = printer.lookup_object_as::<PrinterStepper>(Z_STEPPER) {
            object.lock_state().z_step_dist = stepper.step_dist();
        }
        let Some(gcode_move) = printer.lookup_object_as::<GCodeMove>(GCODE_MOVE_OBJECT) else {
            error!("z_thermal_adjust: gcode_move is not loaded");
            return;
        };
        match gcode_move.set_move_transform(Arc::clone(object) as Arc<dyn MoveTarget>, true) {
            // `next_transform` (`z_thermal_adjust.py:44`): whatever held the
            // slot — `bed_mesh`/`bed_tilt` when one of them registered first —
            // is what the forwarded moves go through.
            Ok(previous) => {
                if let Some(previous) = previous {
                    *object
                        .next_transform
                        .lock()
                        .unwrap_or_else(|p| p.into_inner()) = Some(previous);
                }
            }
            Err(err) => error!("z_thermal_adjust: cannot take the move transform: {err}"),
        }
    }

    /// Upstream's `handle_homing_move_end`: Z homing re-references the offset.
    fn handle_home_rails_end(&self, axes: &[usize]) {
        if !axes.contains(&Z) {
            return;
        }
        let mut state = self.lock_state();
        state.ref_temperature = state.smoothed_temp;
        state.ref_temp_override = false;
        state.z_adjust_mm = 0.0;
    }

    /// The state lock, poisoning treated as continued unwinding
    /// (`gcode_move.rs` convention).
    fn lock_state(&self) -> MutexGuard<'_, State> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The transform chain below this one: the displaced occupant once
    /// registered, the connect-time toolhead before that.
    fn target(&self) -> Option<Arc<dyn MoveTarget>> {
        let next = self
            .next_transform
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        next.or_else(|| {
            self.target
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .clone()
        })
    }

    /// Upstream's `calc_adjust`: the toolhead position for a g-code move —
    /// refresh the offset from the current reading, then add it.
    fn calc_adjust(&self, state: &mut State, position: Coord) -> Coord {
        if position.z() < self.off_above_z {
            let delta_t = state.smoothed_temp - state.ref_temperature;
            // The coefficient is "mm the frame grows per °C", so the
            // correction is its negative.
            let adjust = -state.temp_coeff * delta_t;
            // `sign = 1 - (adjust <= 0)*2`: the side the clamp stays on.
            let sign = if adjust <= 0.0 { -1.0 } else { 1.0 };
            // Don't apply adjustments smaller than one step
            // (`if abs(adjust - self.z_adjust_mm) > self.z_step_dist`), and
            // clamp with `min([max*sign, adjust], key=abs)`.
            if (adjust - state.z_adjust_mm).abs() > state.z_step_dist {
                state.z_adjust_mm = if adjust.abs() > self.max_z_adjust_mm {
                    sign * self.max_z_adjust_mm
                } else {
                    adjust
                };
            }
        }
        let mut adjusted = position;
        adjusted.set_axis(Z, position.z() + state.z_adjust_mm);
        state.last_z_adjust_mm = state.z_adjust_mm;
        adjusted
    }

    /// Upstream's `calc_unadjust`: the g-code position behind a toolhead one.
    fn calc_unadjust(state: &State, position: Coord) -> Coord {
        let mut unadjusted = position;
        unadjusted.set_axis(Z, position.z() - state.z_adjust_mm);
        unadjusted
    }

    /// Register the section's command (`z_thermal_adjust.py:44-46`).
    fn register_commands(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");
        gcode
            .register_command_with_params(
                "SET_Z_THERMAL_ADJUST",
                gcode::sync({
                    let object = Arc::clone(self);
                    move |gcmd| object.cmd_set_z_thermal_adjust(gcmd)
                }),
                Some(SET_Z_THERMAL_ADJUST_HELP),
                &["ENABLE", "TEMP_COEFF", "REF_TEMP"],
                false,
            )
            .map_err(ConfigError::new)
    }

    /// Upstream's `cmd_SET_Z_THERMAL_ADJUST`: set and/or report the parameters.
    ///
    /// A bare `SET_Z_THERMAL_ADJUST` reports; every parameter it does carry is
    /// applied. Enabling or disabling re-anchors `gcode_move`'s position,
    /// because the offset the g-code coordinate carries just changed.
    fn cmd_set_z_thermal_adjust(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let given = gcmd.get_command_parameters();
        let enable = if given.contains_key("ENABLE") {
            Some(gcmd.get_int_bounded("ENABLE", Some(0), Some(1))? != 0)
        } else {
            None
        };
        let coeff = if given.contains_key("TEMP_COEFF") {
            Some(gcmd.get_float_range("TEMP_COEFF", -1.0, 1.0)?)
        } else {
            None
        };
        let ref_temp = if given.contains_key("REF_TEMP") {
            // Upstream's `minval=KELVIN_TO_CELSIUS`, which is inclusive; the
            // `get` bounds here are minval/maxval for the same reason.
            Some(gcmd.get(
                "REF_TEMP",
                None,
                |text| text.parse::<f64>().ok(),
                Some(KELVIN_TO_CELSIUS),
                None,
                None,
                None,
            )?)
        } else {
            None
        };

        let reset_position;
        let msg;
        {
            let mut state = self.lock_state();
            if let Some(ref_temp) = ref_temp {
                state.ref_temperature = ref_temp;
                state.ref_temp_override = true;
            }
            if let Some(coeff) = coeff {
                state.temp_coeff = coeff;
            }
            reset_position = match enable {
                Some(enable) if enable != state.adjust_enable => {
                    state.adjust_enable = enable;
                    true
                }
                _ => false,
            };
            let state_text = if state.adjust_enable {
                "1 (enabled)"
            } else {
                "0 (disabled)"
            };
            let override_text = if state.ref_temp_override {
                " (manual)"
            } else {
                ""
            };
            msg = format!(
                "enable: {state_text}\n\
                 temp_coeff: {:.6} mm/degC\n\
                 ref_temp: {:.2} degC{override_text}\n\
                 -------------------\n\
                 Current Z temp: {:.2} degC\n\
                 Applied Z adjustment: {:.4} mm",
                state.temp_coeff, state.ref_temperature, state.smoothed_temp, state.z_adjust_mm
            );
        }
        // `gcode_move.reset_last_position()` — outside the lock: it reads this
        // transform's own position, which takes the same lock.
        if reset_position {
            if let Some(printer) = self.printer.upgrade() {
                if let Some(gcode_move) = printer.lookup_object_as::<GCodeMove>(GCODE_MOVE_OBJECT) {
                    gcode_move.reset_last_position();
                }
            }
        }
        gcmd.respond_info(&msg);
        Ok(())
    }
}

/// Upstream's `temperature_callback`: the low-pass filter over the raw
/// readings.
///
/// `adj_time` is the share of the gap the reading is allowed to move the
/// smoothed value, capped at 1 so a reading after a long gap lands on it.
fn temperature_callback(state: &Mutex<State>, inv_smooth_time: f64, read_time: f64, temp: f64) {
    let mut state = state.lock().unwrap_or_else(|poison| poison.into_inner());
    let time_diff = read_time - state.last_temp_time;
    state.last_temp = temp;
    state.last_temp_time = read_time;
    let temp_diff = temp - state.smoothed_temp;
    let adj_time = (time_diff * inv_smooth_time).min(1.0);
    state.smoothed_temp += temp_diff * adj_time;
    state.measured_min = state.measured_min.min(state.smoothed_temp);
    state.measured_max = state.measured_max.max(state.smoothed_temp);
}

impl MoveTarget for ZThermalAdjust {
    /// Upstream's `move` (`z_thermal_adjust.py:106-114`): an extrusion-only
    /// move (X and Y unchanged) keeps the offset the last real move applied; a
    /// travel recomputes it. Disabled means the last offset is still applied —
    /// `adjust_enable` only stops the *updates*.
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        let Some(target) = self.target() else {
            // No toolhead and nothing below (`klippy:connect` has not run):
            // upstream would crash on `None`; here the move is refused.
            return Err(CommandError::new("Printer is not ready"));
        };
        let forward = {
            let mut state = self.lock_state();
            let xy_unchanged =
                position.x() == state.last_position.x() && position.y() == state.last_position.y();
            let forward = if xy_unchanged || !state.adjust_enable {
                let mut forward = position;
                forward.set_axis(Z, position.z() + state.last_z_adjust_mm);
                forward
            } else {
                self.calc_adjust(&mut state, position)
            };
            state.last_position = position;
            forward
        };
        target.move_to(forward, speed)
    }

    /// Upstream's `get_position` (`z_thermal_adjust.py:99-103`): the chain's
    /// position with the offset **removed**, and the cached last position
    /// re-derived through the current offset.
    fn position(&self) -> Coord {
        let Some(target) = self.target() else {
            // As above: before connect there is nothing to read.
            return Coord::default();
        };
        let position = Self::calc_unadjust(&self.lock_state(), target.position());
        let mut state = self.lock_state();
        state.last_position = self.calc_adjust(&mut state, position);
        position
    }
}

impl GcodeTempSensor for ZThermalAdjust {
    /// Upstream's `get_temp`: the filtered reading, with no target.
    fn get_temp(&self, _eventtime: f64) -> (f64, f64) {
        (self.lock_state().smoothed_temp, 0.0)
    }
}

impl PrinterObject for ZThermalAdjust {
    /// Upstream's `get_status` (`z_thermal_adjust.py:63-72`).
    fn get_status(&self, _eventtime: f64) -> Value {
        let state = self.lock_state();
        json!({
            "temperature": state.smoothed_temp,
            "measured_min_temp": round2(state.measured_min),
            "measured_max_temp": round2(state.measured_max),
            "current_z_adjust": state.z_adjust_mm,
            "z_adjust_ref_temperature": state.ref_temperature,
            "enabled": state.adjust_enable,
        })
    }
}

impl std::fmt::Debug for ZThermalAdjust {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let state = self.lock_state();
        f.debug_struct("ZThermalAdjust")
            .field("temp_coeff", &state.temp_coeff)
            .field("smoothed_temp", &state.smoothed_temp)
            .field("ref_temperature", &state.ref_temperature)
            .field("z_adjust_mm", &state.z_adjust_mm)
            .field("adjust_enable", &state.adjust_enable)
            .finish_non_exhaustive()
    }
}

/// Upstream's `round(value, 2)`.
fn round2(value: f64) -> f64 {
    (value * 100.0).round() / 100.0
}

/// The toolhead behind the transform: what `position` reads and `move_to`
/// feeds (upstream passes `self.toolhead` around directly).
struct ToolheadMove(Arc<ToolHeadObject>);

impl MoveTarget for ToolheadMove {
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        self.0.move_to(position, speed)
    }

    fn move_to_extended(
        &self,
        position: Coord,
        extras: &[f64],
        speed: f64,
    ) -> Result<(), CommandError> {
        self.0.move_to_extended(position, extras, speed)
    }

    fn position(&self) -> Coord {
        self.0.position().unwrap_or_default()
    }
}

/// Upstream's `load_config` (`z_thermal_adjust.py:177-178`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = ZThermalAdjust::new(config, printer)?;
    object.register_handlers(printer);
    object.register_commands(printer)?;
    // `gcode_move` exists by now (the loader registers it first), but the slot
    // is taken at `klippy:connect`: upstream needs the toolhead and the
    // steppers, which do not exist yet.
    gcode_move::ensure(printer)?;
    Ok(object)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{ConfigSection, ConfigValue};
    use crate::core::klippy::extras::heaters::SensorCallback;
    use crate::core::klippy::motion::HomingHandle;
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[z_thermal_adjust]` section with a `fake` sensor and the given
    /// options on top of the required ones.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("z_thermal_adjust", None);
        let mut put = |name: &str, value: &str| {
            section
                .parameters
                .insert(name.to_string(), ConfigValue::Single(value.to_string()));
        };
        put("sensor_type", "fake");
        put("min_temp", "0");
        put("max_temp", "200");
        for (name, value) in options {
            put(name, value);
        }
        section
    }

    /// A sensor whose readings the test drives by hand.
    struct FakeSensor {
        callback: Arc<Mutex<Option<SensorCallback>>>,
    }

    impl std::fmt::Debug for FakeSensor {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            f.debug_struct("FakeSensor").finish_non_exhaustive()
        }
    }

    impl Sensor for FakeSensor {
        fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) {}

        fn setup_callback(&self, callback: SensorCallback) {
            *self.callback.lock().unwrap_or_else(|p| p.into_inner()) = Some(callback);
        }

        fn get_report_time_delta(&self) -> f64 {
            0.3
        }
    }

    /// A machine with `gcode` and `heaters`, the section built through its own
    /// factory, and the sensor callback in hand.
    struct Harness {
        printer: Arc<Printer>,
        adjust: Arc<ZThermalAdjust>,
        callback: Arc<Mutex<Option<SensorCallback>>>,
    }

    impl Harness {
        fn new(options: &[(&str, &str)]) -> Self {
            Self::try_new(options).expect("the section builds")
        }

        /// [`Harness::new`] for the cases that have to fail.
        fn try_new(options: &[(&str, &str)]) -> Result<Self, ConfigError> {
            let printer = Arc::new(Printer::new(ManualReactor::shared()));
            printer
                .add_object(
                    GCODE_OBJECT,
                    Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
                )
                .expect("gcode registers");
            let heaters = heaters::ensure(&printer).expect("heaters builds");
            let callback: Arc<Mutex<Option<SensorCallback>>> = Arc::new(Mutex::new(None));
            let factory_callback = Arc::clone(&callback);
            heaters.add_sensor_factory(
                "fake",
                Arc::new(move |_config, _printer| {
                    Ok(Arc::new(FakeSensor {
                        callback: Arc::clone(&factory_callback),
                    }) as Arc<dyn Sensor>)
                }),
            );
            let adjust =
                ZThermalAdjust::new(&ConfigWrapper::untracked(&section(options)), &printer)?;
            adjust.register_handlers(&printer);
            adjust
                .register_commands(&printer)
                .expect("the command registers");
            gcode_move::ensure(&printer).expect("gcode_move registers");
            Ok(Self {
                printer,
                adjust,
                callback,
            })
        }

        /// Feed one reading, as the sensor would.
        fn read(&self, read_time: f64, temp: f64) {
            let callback = self.callback.lock().unwrap_or_else(|p| p.into_inner());
            let callback = callback.as_ref().expect("the sensor callback is installed");
            callback(read_time, temp);
        }

        /// The filtered value, once the filter has taken readings in.
        fn smoothed(&self) -> f64 {
            self.adjust.lock_state().smoothed_temp
        }

        /// Register the transform as `klippy:connect` does, with `below` as the
        /// occupied `gcode_move` slot, then anchor the g-code position through
        /// the finished chain.
        fn connect_onto(&self, below: Arc<dyn MoveTarget>) {
            let gcode_move = gcode_move::ensure(&self.printer).expect("gcode_move registers");
            gcode_move
                .set_move_transform(below, false)
                .expect("the slot is empty");
            ZThermalAdjust::handle_connect(&self.adjust);
            self.printer.send_event(&KlippyEvent::ToolheadSetPosition);
        }

        fn gcode(&self) -> Arc<GCodeDispatch> {
            self.printer
                .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                .expect("gcode registers")
        }
    }

    /// A move target that records what it was asked and stands where the last
    /// move left it.
    struct FakeTarget {
        position: Mutex<Coord>,
        moves: Mutex<Vec<(Coord, f64)>>,
    }

    impl FakeTarget {
        fn new(position: Coord) -> Self {
            Self {
                position: Mutex::new(position),
                moves: Mutex::new(Vec::new()),
            }
        }

        fn moves(&self) -> Vec<(Coord, f64)> {
            self.moves.lock().unwrap_or_else(|p| p.into_inner()).clone()
        }

        fn last_z(&self) -> f64 {
            let moves = self.moves();
            moves.last().expect("a move was recorded").0.z()
        }
    }

    impl MoveTarget for FakeTarget {
        fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
            *self.position.lock().unwrap_or_else(|p| p.into_inner()) = position;
            self.moves
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push((position, speed));
            Ok(())
        }

        fn position(&self) -> Coord {
            *self.position.lock().unwrap_or_else(|p| p.into_inner())
        }
    }

    /// Capture everything the dispatcher sends to a client.
    fn capture_output(gcode: &Arc<GCodeDispatch>) -> Arc<Mutex<Vec<String>>> {
        let output = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&output);
        gcode.register_output_handler(Arc::new(move |line: &str| {
            sink.lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(line.to_string());
        }));
        output
    }

    fn emitted(output: &Arc<Mutex<Vec<String>>>) -> String {
        output.lock().unwrap_or_else(|p| p.into_inner()).join("\n")
    }

    #[test]
    fn test_the_options_default_and_read_the_configured_values() {
        let harness = Harness::new(&[]);
        {
            let state = harness.adjust.lock_state();
            assert_eq!(state.temp_coeff, 0.0);
            assert!(state.adjust_enable);
            assert_eq!(state.ref_temperature, 0.0);
        }
        assert_eq!(harness.adjust.off_above_z, UNLIMITED);
        assert_eq!(harness.adjust.max_z_adjust_mm, UNLIMITED);
        assert!((harness.adjust.inv_smooth_time - 0.5).abs() < 1e-12);

        let configured = Harness::new(&[
            ("temp_coeff", "0.012"),
            ("smooth_time", "4"),
            ("z_adjust_off_above", "50"),
            ("max_z_adjustment", "0.5"),
        ]);
        assert_eq!(configured.adjust.lock_state().temp_coeff, 0.012);
        assert!((configured.adjust.inv_smooth_time - 0.25).abs() < 1e-12);
        assert_eq!(configured.adjust.off_above_z, 50.0);
        assert_eq!(configured.adjust.max_z_adjust_mm, 0.5);
    }

    #[test]
    fn test_out_of_range_options_are_refused() {
        for (options, expected) in [
            (vec![("temp_coeff", "1.5")], "temp_coeff"),
            (vec![("temp_coeff", "-2")], "temp_coeff"),
            (vec![("smooth_time", "0")], "smooth_time"),
            (vec![("max_temp", "-1")], "max_temp"),
        ] {
            let error = match Harness::try_new(&options) {
                Err(error) => error,
                Ok(_) => panic!("{options:?} should have been refused"),
            };
            assert!(error.to_string().contains(expected), "{options:?}: {error}");
        }

        // `min_temp`/`max_temp` have no default, as upstream reads them.
        let mut missing = ConfigSection::new("z_thermal_adjust", None);
        missing.parameters.insert(
            "sensor_type".to_string(),
            ConfigValue::Single("fake".to_string()),
        );
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .expect("gcode registers");
        let error = ZThermalAdjust::new(&ConfigWrapper::untracked(&missing), &printer)
            .expect_err("min_temp is required");
        assert!(error.to_string().contains("min_temp"), "{error}");
    }

    #[test]
    fn test_the_filter_moves_a_share_of_the_way_to_each_reading() {
        // `smooth_time` 2 s, readings every 0.5 s: `adj_time` is 0.25.
        let harness = Harness::new(&[]);
        harness.read(0.5, 100.0);
        assert!((harness.smoothed() - 25.0).abs() < 1e-9, "100 * 0.25");

        // Each further reading covers a quarter of the remaining gap: the
        // value rises monotonically and stays under the reading.
        let mut previous = harness.smoothed();
        for step in 1..=20 {
            harness.read(0.5 + 0.5 * step as f64, 100.0);
            let current = harness.smoothed();
            assert!(current > previous, "{current} should exceed {previous}");
            assert!(current < 100.0, "{current} should stay under 100");
            previous = current;
        }
        assert!(
            (previous - 100.0).abs() < 1.0,
            "it converges on the reading: {previous}"
        );

        // A gap longer than the window jumps straight to the reading.
        harness.read(100.0, 30.0);
        assert!((harness.smoothed() - 30.0).abs() < 1e-9);

        // The extremes track the filtered value, not the raw one.
        let status = harness.adjust.get_status(0.0);
        assert!(status["measured_max_temp"].as_f64().unwrap() < 100.0);
        assert!(status["measured_min_temp"].as_f64().unwrap() >= 0.0);
    }

    #[test]
    fn test_the_offset_follows_the_coefficient_and_the_reading() {
        let harness = Harness::new(&[("temp_coeff", "0.01")]);
        let target = Arc::new(FakeTarget::new(Coord::default()));
        harness.connect_onto(Arc::clone(&target) as Arc<dyn MoveTarget>);

        // Reference 20 °C (a Z homing), then 10 °C above it.
        harness.read(10.0, 20.0);
        harness
            .printer
            .send_event(&KlippyEvent::HomingHomeRailsEnd {
                axes: vec![2],
                homing: HomingHandle::new(),
            });
        harness.read(20.0, 30.0);
        harness
            .adjust
            .move_to(Coord::new(1.0, 2.0, 10.0, 0.0), 50.0)
            .unwrap();
        assert!(
            (target.last_z() - 9.9).abs() < 1e-9,
            "-0.01 * 10 = -0.1, so Z 10 becomes 9.9: {}",
            target.last_z()
        );
        let status = harness.adjust.get_status(0.0);
        assert!((status["current_z_adjust"].as_f64().unwrap() + 0.1).abs() < 1e-9);
        assert_eq!(status["z_adjust_ref_temperature"], json!(20.0));
    }

    #[test]
    fn test_a_change_smaller_than_a_step_is_not_applied() {
        let harness = Harness::new(&[("temp_coeff", "0.01")]);
        let target = Arc::new(FakeTarget::new(Coord::default()));
        harness.connect_onto(Arc::clone(&target) as Arc<dyn MoveTarget>);
        {
            let mut state = harness.adjust.lock_state();
            state.z_step_dist = 0.05;
            state.smoothed_temp = 30.0;
            state.ref_temperature = 20.0;
        }
        // -0.1, applied from zero: bigger than one step.
        harness
            .adjust
            .move_to(Coord::new(1.0, 1.0, 5.0, 0.0), 50.0)
            .unwrap();
        assert!((target.last_z() - 4.9).abs() < 1e-9);

        // -0.13 differs from the standing -0.1 by 0.03, under one step: kept.
        harness.adjust.lock_state().smoothed_temp = 33.0;
        harness
            .adjust
            .move_to(Coord::new(2.0, 1.0, 5.0, 0.0), 50.0)
            .unwrap();
        assert!(
            (target.last_z() - 4.9).abs() < 1e-9,
            "the -0.1 correction is kept: {}",
            target.last_z()
        );

        // -0.4 differs by enough: it is applied.
        harness.adjust.lock_state().smoothed_temp = 60.0;
        harness
            .adjust
            .move_to(Coord::new(3.0, 1.0, 5.0, 0.0), 50.0)
            .unwrap();
        assert!((target.last_z() - 4.6).abs() < 1e-9);
    }

    #[test]
    fn test_max_z_adjustment_clamps_the_offset() {
        let harness = Harness::new(&[("temp_coeff", "0.01"), ("max_z_adjustment", "0.05")]);
        let target = Arc::new(FakeTarget::new(Coord::default()));
        harness.connect_onto(Arc::clone(&target) as Arc<dyn MoveTarget>);
        {
            let mut state = harness.adjust.lock_state();
            state.smoothed_temp = 30.0;
            state.ref_temperature = 20.0;
        }
        // -0.1 would apply; the clamp keeps it on the same side: -0.05.
        harness
            .adjust
            .move_to(Coord::new(1.0, 1.0, 5.0, 0.0), 50.0)
            .unwrap();
        assert!((target.last_z() - 4.95).abs() < 1e-9, "{}", target.last_z());

        // The other side clamps too.
        harness.adjust.lock_state().smoothed_temp = 10.0;
        harness
            .adjust
            .move_to(Coord::new(2.0, 1.0, 5.0, 0.0), 50.0)
            .unwrap();
        assert!((target.last_z() - 5.05).abs() < 1e-9, "{}", target.last_z());
    }

    #[test]
    fn test_z_adjust_off_above_keeps_the_last_correction() {
        let harness = Harness::new(&[("temp_coeff", "0.01"), ("z_adjust_off_above", "5")]);
        let target = Arc::new(FakeTarget::new(Coord::default()));
        harness.connect_onto(Arc::clone(&target) as Arc<dyn MoveTarget>);
        {
            let mut state = harness.adjust.lock_state();
            state.smoothed_temp = 30.0;
            state.ref_temperature = 20.0;
        }
        harness
            .adjust
            .move_to(Coord::new(1.0, 1.0, 4.0, 0.0), 50.0)
            .unwrap();
        assert!((target.last_z() - 3.9).abs() < 1e-9);

        // Above the limit the offset stops updating: the standing -0.1 stays
        // even though the reading would now ask for -0.4.
        harness.adjust.lock_state().smoothed_temp = 60.0;
        harness
            .adjust
            .move_to(Coord::new(2.0, 1.0, 10.0, 0.0), 50.0)
            .unwrap();
        assert!((target.last_z() - 9.9).abs() < 1e-9, "{}", target.last_z());

        // Back below the limit it updates again.
        harness
            .adjust
            .move_to(Coord::new(3.0, 1.0, 4.0, 0.0), 50.0)
            .unwrap();
        assert!((target.last_z() - 3.6).abs() < 1e-9, "{}", target.last_z());
    }

    #[test]
    fn test_a_g_code_move_reaches_the_toolhead_with_the_adjustment() {
        let harness = Harness::new(&[("temp_coeff", "0.01")]);
        harness.read(10.0, 30.0);
        harness.adjust.lock_state().ref_temperature = 20.0;
        let toolhead = Arc::new(FakeTarget::new(Coord::default()));
        harness.connect_onto(Arc::clone(&toolhead) as Arc<dyn MoveTarget>);
        harness.printer.send_event(&KlippyEvent::KlippyReady);

        harness
            .gcode()
            .run_script_sync("G90\nG1 X10 Y10 Z5 F600")
            .expect("the move runs");

        assert_eq!(toolhead.moves().len(), 1, "{:?}", toolhead.moves());
        let moved = toolhead.moves()[0].0;
        assert_eq!((moved.x(), moved.y()), (10.0, 10.0));
        assert!(
            (moved.z() - 4.9).abs() < 1e-9,
            "Z 5 carries the -0.1 correction: {}",
            moved.z()
        );
    }

    #[test]
    fn test_an_extrusion_only_move_keeps_the_standing_correction() {
        let harness = Harness::new(&[("temp_coeff", "0.01")]);
        let target = Arc::new(FakeTarget::new(Coord::default()));
        harness.connect_onto(Arc::clone(&target) as Arc<dyn MoveTarget>);
        {
            let mut state = harness.adjust.lock_state();
            state.smoothed_temp = 30.0;
            state.ref_temperature = 20.0;
        }
        harness
            .adjust
            .move_to(Coord::new(2.0, 3.0, 5.0, 0.0), 25.0)
            .unwrap();
        assert!((target.last_z() - 4.9).abs() < 1e-9);

        // The reading changes, but the move only extrudes: no recomputation.
        harness.adjust.lock_state().smoothed_temp = 60.0;
        harness
            .adjust
            .move_to(Coord::new(2.0, 3.0, 5.0, 1.5), 25.0)
            .unwrap();
        assert!((target.last_z() - 4.9).abs() < 1e-9, "{}", target.last_z());
    }

    #[test]
    fn test_position_removes_the_correction_from_the_toolhead() {
        let harness = Harness::new(&[("temp_coeff", "0.01")]);
        let target = Arc::new(FakeTarget::new(Coord::new(0.0, 0.0, 4.9, 0.0)));
        harness.connect_onto(Arc::clone(&target) as Arc<dyn MoveTarget>);
        {
            let mut state = harness.adjust.lock_state();
            state.smoothed_temp = 30.0;
            state.ref_temperature = 20.0;
            state.z_adjust_mm = -0.1;
            state.last_z_adjust_mm = -0.1;
        }

        let position = harness.adjust.position();
        assert!(
            (position.z() - 5.0).abs() < 1e-9,
            "the g-code space reads the uncompensated 5.0: {}",
            position.z()
        );
        // Upstream also re-derives the cached last position through the offset.
        assert!((harness.adjust.lock_state().last_position.z() - 4.9).abs() < 1e-9);
    }

    #[test]
    fn test_z_homing_re_references_the_offset() {
        let harness = Harness::new(&[("temp_coeff", "0.01")]);
        harness.read(10.0, 25.0);
        harness.adjust.lock_state().z_adjust_mm = -0.3;

        // Homing another axis leaves the reference alone.
        harness
            .printer
            .send_event(&KlippyEvent::HomingHomeRailsEnd {
                axes: vec![0, 1],
                homing: HomingHandle::new(),
            });
        assert_eq!(
            harness.adjust.get_status(0.0)["current_z_adjust"],
            json!(-0.3)
        );

        harness
            .printer
            .send_event(&KlippyEvent::HomingHomeRailsEnd {
                axes: vec![0, 1, 2],
                homing: HomingHandle::new(),
            });
        let status = harness.adjust.get_status(0.0);
        assert_eq!(status["current_z_adjust"], json!(0.0));
        assert_eq!(status["z_adjust_ref_temperature"], json!(25.0));
        assert_eq!(status["enabled"], json!(true));
    }

    #[test]
    fn test_the_command_enables_disables_and_queries() {
        let harness = Harness::new(&[("temp_coeff", "0.01")]);
        harness.printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = harness.gcode();
        let output = capture_output(&gcode);

        // A bare command reports.
        gcode.run_script_sync("SET_Z_THERMAL_ADJUST").unwrap();
        let text = emitted(&output);
        assert!(text.contains("enable: 1 (enabled)"), "{text}");
        assert!(text.contains("temp_coeff: 0.010000 mm/degC"), "{text}");

        gcode
            .run_script_sync("SET_Z_THERMAL_ADJUST ENABLE=0")
            .unwrap();
        assert_eq!(harness.adjust.get_status(0.0)["enabled"], json!(false));
        assert!(emitted(&output).contains("enable: 0 (disabled)"));

        gcode
            .run_script_sync("SET_Z_THERMAL_ADJUST ENABLE=1 TEMP_COEFF=0.02 REF_TEMP=25")
            .unwrap();
        let status = harness.adjust.get_status(0.0);
        assert_eq!(status["enabled"], json!(true));
        assert_eq!(status["z_adjust_ref_temperature"], json!(25.0));
        assert_eq!(harness.adjust.lock_state().temp_coeff, 0.02);
        let text = emitted(&output);
        assert!(text.contains("ref_temp: 25.00 degC (manual)"), "{text}");

        // Out of range parameters are refused, as upstream's bounds are.
        assert!(gcode
            .run_script_sync("SET_Z_THERMAL_ADJUST ENABLE=2")
            .is_err());
        assert!(gcode
            .run_script_sync("SET_Z_THERMAL_ADJUST TEMP_COEFF=1.5")
            .is_err());
        assert!(gcode
            .run_script_sync("SET_Z_THERMAL_ADJUST REF_TEMP=-300")
            .is_err());
    }

    #[test]
    fn test_the_status_carries_upstreams_keys() {
        let harness = Harness::new(&[]);
        harness.read(10.0, 21.234);
        let status = harness.adjust.get_status(0.0);
        let keys: Vec<&str> = {
            let mut keys: Vec<&str> = status
                .as_object()
                .expect("a JSON object")
                .keys()
                .map(String::as_str)
                .collect();
            keys.sort_unstable();
            keys
        };
        let mut expected = [
            "temperature",
            "measured_min_temp",
            "measured_max_temp",
            "current_z_adjust",
            "z_adjust_ref_temperature",
            "enabled",
        ];
        expected.sort_unstable();
        assert_eq!(keys, expected);
        assert_eq!(status["temperature"], json!(21.234));
        // Both extremes start at 0 (upstream's `measured_min = measured_max =
        // 0.`), so a warm reading only moves the maximum; both are rounded.
        assert_eq!(status["measured_min_temp"], json!(0.0));
        assert_eq!(status["measured_max_temp"], json!(21.23));
        assert_eq!(status["current_z_adjust"], json!(0.0));
        assert_eq!(harness.adjust.get_temp(0.0), (21.234, 0.0));
    }

    #[test]
    fn test_the_transform_chains_to_the_occupant_it_displaced() {
        let harness = Harness::new(&[("temp_coeff", "0.01")]);
        harness.read(10.0, 30.0);
        harness.adjust.lock_state().ref_temperature = 20.0;
        // A `bed_mesh`-like occupant of the g-code slot: the transform takes
        // the slot from it and forwards to it.
        let occupant = Arc::new(FakeTarget::new(Coord::new(0.0, 0.0, 1.0, 0.0)));
        harness.connect_onto(Arc::clone(&occupant) as Arc<dyn MoveTarget>);
        assert!(
            harness
                .adjust
                .next_transform
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .is_some(),
            "the displaced occupant becomes the chain below"
        );

        harness
            .adjust
            .move_to(Coord::new(4.0, 0.0, 5.0, 0.0), 10.0)
            .unwrap();
        assert!(
            (occupant.last_z() - 4.9).abs() < 1e-9,
            "{}",
            occupant.last_z()
        );

        // Reading back goes through the same chain.
        assert!((harness.adjust.position().z() - 5.0).abs() < 1e-9);

        // And the g-code slot is this transform's, so a second registration
        // without `force` is refused — as upstream's `set_move_transform`.
        let gcode_move = gcode_move::ensure(&harness.printer).expect("gcode_move registers");
        assert!(gcode_move
            .set_move_transform(Arc::new(FakeTarget::new(Coord::default())), false)
            .is_err());
    }

    #[test]
    fn test_the_section_loads_through_the_factory() {
        use crate::core::klippy::config::Config;

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let heaters = heaters::ensure(&printer).expect("heaters builds");
        heaters.add_sensor_factory(
            "fake",
            Arc::new(|_config, _printer| {
                Ok(Arc::new(FakeSensor {
                    callback: Arc::new(Mutex::new(None)),
                }) as Arc<dyn Sensor>)
            }),
        );
        let parsed = Config::from_text(
            "[z_thermal_adjust]\nsensor_type: fake\nmin_temp: 0\nmax_temp: 200\n\
             temp_coeff: 0.02\n",
        )
        .expect("the config parses")
        .0;

        printer.load_config(&parsed).expect("the section loads");

        let object = printer
            .lookup_object_as::<ZThermalAdjust>("z_thermal_adjust")
            .expect("the section is registered under its name");
        assert_eq!(object.lock_state().temp_coeff, 0.02);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode registers");
        assert!(gcode.command_exists("SET_Z_THERMAL_ADJUST"));
        // The sensor name joins `TEMPERATURE_WAIT`, as upstream's
        // `register_sensor` does.
        let heaters = heaters::ensure(&printer).expect("heaters builds");
        assert_eq!(
            heaters.get_status(0.0)["available_sensors"],
            json!(["z_thermal_adjust"])
        );
    }

    #[test]
    fn test_an_unknown_option_is_refused() {
        use crate::core::klippy::config::Config;

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let heaters = heaters::ensure(&printer).expect("heaters builds");
        heaters.add_sensor_factory(
            "fake",
            Arc::new(|_config, _printer| {
                Ok(Arc::new(FakeSensor {
                    callback: Arc::new(Mutex::new(None)),
                }) as Arc<dyn Sensor>)
            }),
        );
        let parsed = Config::from_text(
            "[z_thermal_adjust]\nsensor_type: fake\nmin_temp: 0\nmax_temp: 200\nmade_up: 1\n",
        )
        .expect("the config parses")
        .0;

        let error = printer
            .load_config(&parsed)
            .expect_err("the option is unknown");
        assert!(error.to_string().contains("made_up"), "{error}");
    }
}
