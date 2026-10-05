//! `[heaters]` — the temperature-sensor registry.
//!
//! Upstream's `heaters.py` is two things: the registry every
//! `[temperature_sensor]`, `[extruder]` and `[heater_bed]` sets its sensor up
//! through, and the heater control loops. Both are here: [`PrinterHeaters`]
//! holds the sensor factories and the heaters, [`Heater`] is the control loop
//! `[extruder]` / `[heater_bed]` / `[heater_generic]` build, and
//! [`PrinterHeaters::setup_heater`] also gives each heater its
//! `[verify_heater <name>]` check, and the first heater loads the
//! `pid_calibrate` object with it (`heaters.py:64-65`) — where `PID_CALIBRATE`
//! and its [`HeaterControl`] takeover come from
//! (`crate::core::klippy::extras::pid_calibrate`).
//!
//! The object has no `[heaters]` section of its own — upstream loads it by name
//! (`printer.load_object(config, 'heaters')`) and so does [`ensure`], which is
//! also where the built-in sensor modules are brought in (upstream reads
//! `temperature_sensors.cfg` for that).

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, Weak};
use std::time::Duration;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::adc_temperature;
use crate::core::klippy::extras::ds18b20;
use crate::core::klippy::extras::heater_generic::PrinterHeaterGeneric;
use crate::core::klippy::extras::pid_calibrate;
use crate::core::klippy::extras::spi_temperature;
use crate::core::klippy::extras::temperature_combined;
use crate::core::klippy::extras::temperature_mcu;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::extras::verify_heater;
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::pins::{PrinterPins, PwmOut, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject, PrinterState};

/// The name other modules look the registry up by.
pub const HEATERS_OBJECT: &str = "heaters";

/// The longest a heater PWM change may sit before the firmware falls back
/// (upstream `MAX_HEAT_TIME`, `heaters.py:14`).
#[allow(dead_code)]
const MAX_HEAT_TIME: f64 = 3.0;
/// The temperature the PID's first derivative is measured against
/// (upstream `AMBIENT_TEMP`, `heaters.py:15`).
const AMBIENT_TEMP: f64 = 25.0;
/// The divisor upstream stores PID constants over (`PID_PARAM_BASE`) — what
/// `pid_calibrate` multiplies its tuned gains by before storing them
/// (`pid_calibrate.py:124`).
pub const PID_PARAM_BASE: f64 = 255.0;
/// How close a PID must settle before its target counts reached
/// (`PID_SETTLE_DELTA`/`PID_SETTLE_SLOPE`, read by [`Heater::check_busy`]).
/// Upstream polls it in `_wait_for_temperature` while `M109`/`M190` wait
/// (`set_temperature(.., wait)`); `TEMPERATURE_WAIT` does not use it — its
/// loop waits on the reading itself.
const PID_SETTLE_DELTA: f64 = 1.0;
const PID_SETTLE_SLOPE: f64 = 0.1;
/// `cmd_TEMPERATURE_WAIT_help` (`heaters.py:366`).
const TEMPERATURE_WAIT_HELP: &str = "Wait for a temperature on a sensor";

/// The toolhead a `set_temperature` wait flushes through, upstream's
/// `lookup_object("toolhead")` (`heaters.py:352,361`).
const TOOLHEAD_OBJECT: &str = "toolhead";

/// Called with `(read_time, temperature)` for every reading.
pub type SensorCallback = Box<dyn Fn(f64, f64) + Send + Sync>;

/// What a temperature sensor provides — the part of upstream's sensor interface
/// the registry and its consumers need.
pub trait Sensor: Send + Sync + std::fmt::Debug {
    /// The range a reading is allowed to fall in.
    fn setup_minmax(&self, min_temp: f64, max_temp: f64);

    /// Where readings are delivered.
    fn setup_callback(&self, callback: SensorCallback);
}

/// What the M105 g-code-id table stores — any object that can report its
/// `(current, target)` temperature, as upstream's `gcode_id_to_sensor` does
/// (`heaters.py:247`). A heater answers with its smoothed reading and
/// target; a sensor-only section with `gcode_id` would answer with its last
/// reading and a target of zero.
pub trait GcodeTempSensor: Send + Sync {
    /// The current and target temperatures (`Heater.get_temp`,
    /// `heaters.py:116-122`).
    fn get_temp(&self, eventtime: f64) -> (f64, f64);
}

impl GcodeTempSensor for Heater {
    fn get_temp(&self, _eventtime: f64) -> (f64, f64) {
        // The existing `Heater::get_temp` does not take `eventtime` — the
        // stale-reading check upstream does (`QUELL_STALE_TIME`) needs a
        // print-time clock this host does not have (see `Heater::get_temp`).
        self.get_temp()
    }
}

/// Builds one sensor from its section (upstream's `sensor_factories` entry).
pub type SensorFactory = Arc<
    dyn Fn(&ConfigWrapper, &Arc<Printer>) -> Result<Arc<dyn Sensor>, ConfigError> + Send + Sync,
>;

/// A control algorithm a heater runs over each reading.
///
/// Upstream spells these as classes with `temperature_update` / `check_busy`
/// (`ControlBangBang` and `ControlPID` here, `ControlAutoTune` in
/// `pid_calibrate.py`); the trait is what lets a calibration take the
/// heater's control over for the length of a run and hand the old one back
/// ([`Heater::set_control`], upstream's `Heater.set_control`,
/// `heaters.py:127-132`).
pub trait HeaterControl: Send {
    /// Compute the PWM value for one reading (`temperature_update`).
    ///
    /// `target` is the heater's own target, passed so a control may rewrite
    /// it — that is upstream's `Heater.alter_target` (`heaters.py:133-136`),
    /// which `ControlAutoTune` uses to swing between the calibration target
    /// and `TUNE_PID_DELTA` below it.
    fn update(&mut self, read_time: f64, temp: f64, target: &mut f64, max_power: f64) -> f64;

    /// Whether a requested temperature has not been reached yet
    /// (`check_busy`).
    fn check_busy(&self, smoothed_temp: f64, target: f64) -> bool;
}

/// The configured control algorithms (upstream's `ControlBangBang` / `ControlPID`).
enum Control {
    /// `control: watermark`.
    BangBang { max_delta: f64, heating: bool },
    /// `control: pid`.
    Pid {
        kp: f64,
        ki: f64,
        kd: f64,
        min_deriv_time: f64,
        temp_integ_max: f64,
        prev_temp: f64,
        prev_temp_time: f64,
        prev_temp_deriv: f64,
        prev_temp_integ: f64,
    },
}

impl HeaterControl for Control {
    /// One reading: the PWM value the heater applies (`temperature_update`).
    fn update(&mut self, read_time: f64, temp: f64, target: &mut f64, max_power: f64) -> f64 {
        match self {
            Control::BangBang { max_delta, heating } => {
                let target = *target;
                if *heating && temp >= target + *max_delta {
                    *heating = false;
                } else if !*heating && temp <= target - *max_delta {
                    *heating = true;
                }
                if *heating {
                    max_power
                } else {
                    0.0
                }
            }
            Control::Pid {
                kp,
                ki,
                kd,
                min_deriv_time,
                temp_integ_max,
                prev_temp,
                prev_temp_time,
                prev_temp_deriv,
                prev_temp_integ,
            } => {
                let target = *target;
                let time_diff = read_time - *prev_temp_time;
                let temp_diff = temp - *prev_temp;
                let temp_deriv = if time_diff >= *min_deriv_time {
                    temp_diff / time_diff
                } else {
                    (*prev_temp_deriv * (*min_deriv_time - time_diff) + temp_diff) / *min_deriv_time
                };
                let temp_err = target - temp;
                let temp_integ =
                    (*prev_temp_integ + temp_err * time_diff).clamp(0.0, *temp_integ_max);
                let co = *kp * temp_err + *ki * temp_integ - *kd * temp_deriv;
                let bounded = co.clamp(0.0, max_power);
                *prev_temp = temp;
                *prev_temp_time = read_time;
                *prev_temp_deriv = temp_deriv;
                if co == bounded {
                    *prev_temp_integ = temp_integ;
                }
                bounded
            }
        }
    }

    /// Whether a requested temperature has not been reached yet
    /// (`check_busy`).
    fn check_busy(&self, smoothed_temp: f64, target: f64) -> bool {
        match self {
            Control::BangBang { max_delta, .. } => smoothed_temp < target - *max_delta,
            Control::Pid {
                prev_temp_deriv, ..
            } => {
                (target - smoothed_temp).abs() > PID_SETTLE_DELTA
                    || prev_temp_deriv.abs() > PID_SETTLE_SLOPE
            }
        }
    }
}

/// One configured heater (an extruder hotend, a bed, a generic heater).
///
/// Upstream's `Heater` (`klippy/extras/heaters.py:21-160`): it owns the sensor
/// callback, the bang-bang/PID control loop and the PWM output. The periodic
/// check over it is [`verify_heater::HeaterCheck`], which [`PrinterHeaters::setup_heater`]
/// gives each heater.
pub struct Heater {
    /// The section's short name (`extruder`, `heater_bed`).
    name: String,
    /// The section it was configured under (`heater_generic myheater`),
    /// upstream's `Heater.get_name()` — a superset of [`Self::name`], and the
    /// name `PID_CALIBRATE` stages its result under for `SAVE_CONFIG`.
    section_name: String,
    /// The sensor built from the heater's section.
    sensor: Arc<dyn Sensor>,
    /// The heater's PWM output, when the pin was set up.
    pwm: Option<Arc<dyn PwmOut>>,
    min_temp: f64,
    max_temp: f64,
    /// `min_extrude_temp`: the reading below which extrusion is refused.
    min_extrude_temp: f64,
    max_power: f64,
    /// `1 / smooth_time`, for the smoothed temperature.
    inv_smooth_time: f64,
    state: Mutex<HeaterState>,
}

/// The mutable half of a [`Heater`].
struct HeaterState {
    target_temp: f64,
    last_temp: f64,
    smoothed_temp: f64,
    last_temp_time: f64,
    can_extrude: bool,
    last_pwm_value: f64,
    control: Box<dyn HeaterControl>,
}

impl Heater {
    /// The heater's short name (`extruder`, `heater_bed`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The section the heater was configured under (`Heater.get_name`,
    /// `heaters.py:101-102`).
    pub fn section_name(&self) -> &str {
        &self.section_name
    }

    /// The configured power ceiling (`Heater.get_max_power`,
    /// `heaters.py:105-106`).
    pub fn max_power(&self) -> f64 {
        self.max_power
    }

    /// The target the heater is holding (`Heater.target_temp`).
    pub fn target_temp(&self) -> f64 {
        self.lock().target_temp
    }

    /// The sensor this heater reads.
    pub fn sensor(&self) -> &Arc<dyn Sensor> {
        &self.sensor
    }

    /// Whether a move may extrude (`PrinterExtruder.check_move`).
    pub fn can_extrude(&self) -> bool {
        self.lock().can_extrude
    }

    /// The configured temperature range.
    pub fn temperature_range(&self) -> (f64, f64) {
        (self.min_temp, self.max_temp)
    }

    /// Set the target temperature (`Heater.set_temp`).
    ///
    /// # Errors
    /// The requested temperature is outside the configured range, as
    /// upstream's `SET_HEATER_TEMPERATURE` checks (with upstream's own
    /// wording, `heaters.py:109-113`).
    pub fn set_temp(&self, degrees: f64) -> Result<(), CommandError> {
        if degrees != 0.0 && (degrees < self.min_temp || degrees > self.max_temp) {
            return Err(CommandError::new(format!(
                "Requested temperature ({degrees:.1}) out of range ({:.1}:{:.1})",
                self.min_temp, self.max_temp
            )));
        }
        self.lock().target_temp = degrees;
        Ok(())
    }

    /// Swap in another control algorithm, returning the one that was running
    /// (`Heater.set_control`, `heaters.py:127-132`). `PID_CALIBRATE` installs
    /// its autotune for the length of a run and hands the configured control
    /// back afterwards — and, as upstream does, each swap leaves the heater
    /// with no target: the command sets the calibration target through
    /// [`Self::set_temp`] straight afterwards, and restoring the old control
    /// is what turns the heater off at the end of the run.
    pub fn set_control(&self, control: Box<dyn HeaterControl>) -> Box<dyn HeaterControl> {
        let mut state = self.lock();
        let old = std::mem::replace(&mut state.control, control);
        state.target_temp = 0.0;
        old
    }

    /// One sensor reading: run the control loop and update the smoothed
    /// temperature (`Heater.temperature_callback`).
    pub fn temperature_callback(&self, read_time: f64, temp: f64) {
        let mut state = self.lock();
        let time_diff = read_time - state.last_temp_time;
        state.last_temp = temp;
        state.last_temp_time = read_time;
        let HeaterState {
            control,
            target_temp,
            ..
        } = &mut *state;
        let value = control.update(read_time, temp, target_temp, self.max_power);
        // Upstream schedules the change at `read_time + pwm_delay`; print-time
        // scheduling is C1d, so the output goes out immediately.
        if let Some(pwm) = &self.pwm {
            let _ = pwm.update_pwm(value);
        }
        let temp_diff = temp - state.smoothed_temp;
        let adj_time = (time_diff * self.inv_smooth_time).min(1.0);
        state.smoothed_temp += temp_diff * adj_time;
        state.can_extrude = state.smoothed_temp >= self.min_extrude_temp;
        state.last_pwm_value = value;
    }

    /// Whether a `target` has not settled yet (`Heater.check_busy`).
    ///
    /// Upstream polls it in `_wait_for_temperature` while `M109`/`M190` wait
    /// for their target (`heaters.py:348-359`); both waits are wired through
    /// [`Self::set_temperature`] (2026-10-03, `6900894` / `6745e83`).
    /// `TEMPERATURE_WAIT` does not use it — it waits on the reading itself.
    pub fn check_busy(&self, target: f64) -> bool {
        let state = self.lock();
        state.control.check_busy(state.smoothed_temp, target)
    }

    /// What `verify_heater` reads each second (`Heater.get_temp`).
    ///
    /// Upstream returns `(0., target)` when the newest reading is older than
    /// `QUELL_STALE_TIME` (7 s), so a sensor that went quiet counts as a cold
    /// heater (`heaters.py:18,116-122`). That comparison needs the reading's
    /// time and `estimated_print_time(eventtime)` on one clock, which this host
    /// does not have: the ADC sensors pass the raw firmware clock through
    /// (`pins.rs:220-225`, `adc_temperature.rs:710`) while the serial ones map
    /// theirs with `clock_to_print_time` (`ds18b20.rs:250`). The smoothed
    /// temperature is returned as it stands; see the `verify_heater` module
    /// docs for what that changes.
    pub fn get_temp(&self) -> (f64, f64) {
        let state = self.lock();
        (state.smoothed_temp, state.target_temp)
    }

    /// `Heater.get_status`.
    pub fn get_status(&self) -> Value {
        let state = self.lock();
        json!({
            "temperature": (state.smoothed_temp * 100.0).round() / 100.0,
            "target": state.target_temp,
            "power": state.last_pwm_value,
        })
    }

    fn lock(&self) -> MutexGuard<'_, HeaterState> {
        self.state.lock().unwrap_or_else(|p| p.into_inner())
    }
}

impl std::fmt::Debug for Heater {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Heater").field("name", &self.name).finish()
    }
}

/// The `heaters` object: the sensor factory table and what is registered.
///
/// The heater table is what upstream's `PrinterHeaters.lookup_heater` reads
/// (`heaters.py:288-292`): a name (the heater section's short name) to the
/// heater itself. It is what lets a consumer reach a heater by the name it is
/// configured under — `controller_fan` resolves its `heater` option against
/// object names instead (`controller_fan.rs:22-30`), because that table did not
/// exist when it was written.
pub struct PrinterHeaters {
    /// The machine this registry serves: what `set_temperature(.., wait)`
    /// reaches for the toolhead and the shutdown check, and what the
    /// `TEMPERATURE_WAIT` handler resolves its sensor with (upstream's bound
    /// method reaches the same state through `gcmd`'s printer). `Weak`, so
    /// the command table the printer keeps does not keep the printer alive.
    printer: Weak<Printer>,
    factories: Mutex<BTreeMap<String, SensorFactory>>,
    sensors: Mutex<Vec<String>>,
    monitors: Mutex<Vec<String>>,
    heaters: Mutex<BTreeMap<String, Arc<Heater>>>,
    /// Upstream's `gcode_id_to_sensor` (`heaters.py:247`): the g-code-id
    /// table M105 reports and TEMPERATURE_WAIT echoes.
    gcode_id_to_sensor: Mutex<BTreeMap<String, Arc<dyn GcodeTempSensor>>>,
    /// Upstream's `has_started` (`heaters.py:251`): set on `klippy:ready`,
    /// gates `_get_temp` so M105 before ready reports `T:0`.
    has_started: AtomicBool,
}

impl PrinterHeaters {
    fn new(printer: Weak<Printer>) -> Self {
        Self {
            printer,
            factories: Mutex::new(BTreeMap::new()),
            sensors: Mutex::new(Vec::new()),
            monitors: Mutex::new(Vec::new()),
            heaters: Mutex::new(BTreeMap::new()),
            gcode_id_to_sensor: Mutex::new(BTreeMap::new()),
            has_started: AtomicBool::new(false),
        }
    }

    /// Upstream's `_get_temp` (`heaters.py:331-340`): the M105 temperature
    /// line.
    ///
    /// After `has_started`, each registered g-code-id sensor reports
    /// `"<id>:{cur:.1} /{target:.1}"`, sorted by id (BTreeMap order); an empty
    /// or pre-start table reports `"T:0"`.
    fn _get_temp(&self, eventtime: f64) -> String {
        let mut out = Vec::new();
        if self.has_started.load(Ordering::SeqCst) {
            for (gcode_id, sensor) in self
                .gcode_id_to_sensor
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .iter()
            {
                let (cur, target) = sensor.get_temp(eventtime);
                out.push(format!("{gcode_id}:{cur:.1} /{target:.1}"));
            }
        }
        if out.is_empty() {
            "T:0".to_string()
        } else {
            out.join(" ")
        }
    }

    /// Register the M105 command and wire `klippy:ready` to set `has_started`
    /// (upstream's `PrinterHeaters.__init__`, `heaters.py:243-259,329-330`).
    fn register_commands(self: &Arc<Self>, printer: &Arc<Printer>) {
        let weak_heaters = Arc::downgrade(self);
        printer.register_event_handler(
            KlippyEvent::KlippyReady,
            Box::new(move |_| {
                if let Some(heaters) = weak_heaters.upgrade() {
                    heaters.has_started.store(true, Ordering::SeqCst);
                }
            }),
        );

        let weak_heaters = Arc::downgrade(self);
        let weak_printer = Arc::downgrade(printer);
        // `ensure` may run in test setups without a `gcode` object; skip M105
        // registration then (upstream registers it in `__init__` where `gcode`
        // always exists — `heaters.py:259`).
        if let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) {
            gcode
                .register_command(
                    "M105",
                    sync(move |gcmd: &GcodeCommand| {
                        let msg = match (weak_heaters.upgrade(), weak_printer.upgrade()) {
                            (Some(heaters), Some(printer)) => {
                                heaters._get_temp(printer.eventtime())
                            }
                            _ => "T:0".to_string(),
                        };
                        gcmd.respond_raw(&msg);
                        Ok(())
                    }),
                    None,
                    true,
                )
                .expect("M105 is a valid, unique command name");
        }
    }

    /// The heater registered under `name` (`PrinterHeaters.lookup_heater`).
    ///
    /// # Errors
    /// No heater has that name, with upstream's wording
    /// (`heaters.py:288-292`).
    pub fn lookup_heater(&self, name: &str) -> Result<Arc<Heater>, ConfigError> {
        self.heaters
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(name)
            .cloned()
            .ok_or_else(|| ConfigError::new(format!("Unknown heater '{name}'")))
    }

    /// Give a heater a target, waiting for it when asked
    /// (`PrinterHeaters.set_temperature`, `heaters.py:360-365`).
    ///
    /// Upstream's wait loop is [`Self::wait_for_temperature`]; `wait` is what
    /// `PID_CALIBRATE` asks for. `M109`/`M190` want the same wait once their
    /// own loop is wired — today they set their target and return.
    ///
    /// # Errors
    /// The target is outside the heater's configured range (upstream raises
    /// the same `command_error` from `heater.set_temp`).
    pub async fn set_temperature(
        &self,
        heater: &Heater,
        temp: f64,
        wait: bool,
    ) -> Result<(), CommandError> {
        // Upstream registers a no-op lookahead callback first, so the planner
        // keeps handing moves to the trapq while the wait runs
        // (`heaters.py:361-362`).
        if let Some(toolhead) = self
            .printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT))
        {
            toolhead.register_lookahead_callback(Box::new(|_print_time| {}));
        }
        heater.set_temp(temp)?;
        if wait && temp != 0.0 {
            self.wait_for_temperature(heater).await;
        }
        Ok(())
    }

    /// Wait until the heater's control says the target has settled, or the
    /// printer gives up (`PrinterHeaters._wait_for_temperature`,
    /// `heaters.py:348-359`).
    ///
    /// What differs from upstream:
    ///
    /// * the per-second M105 line is echoed via `_get_temp`, as upstream does
    ///   (`heaters.py:358`);
    /// * a file-output run returns at once, as upstream's does
    ///   (`heaters.py:350-351`);
    /// * the sleep is a `tokio` timer rather than `reactor.pause`.
    async fn wait_for_temperature(&self, heater: &Heater) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        if printer.is_fileoutput() {
            return;
        }
        let gcode = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT);
        loop {
            if matches!(
                printer.get_state_message().category,
                PrinterState::Shutdown | PrinterState::Error
            ) {
                // Upstream's `not printer.is_shutdown()`.
                break;
            }
            if !heater.check_busy(heater.target_temp()) {
                break;
            }
            // Keep the look-ahead moving, as upstream does each second.
            if let Some(toolhead) = printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT) {
                let _ = toolhead.get_last_move_time();
            }
            if let Some(gcode) = &gcode {
                gcode.respond_raw(&self._get_temp(printer.eventtime()));
            }
            tokio::time::sleep(std::time::Duration::from_secs(1)).await;
        }
    }

    /// Register a sensor type, upstream's `add_sensor_factory`.
    pub fn add_sensor_factory(&self, sensor_type: &str, factory: SensorFactory) {
        self.factories
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(sensor_type.to_string(), factory);
    }

    /// Build the sensor a section asks for, upstream's `setup_sensor`.
    ///
    /// # Errors
    /// A missing `sensor_type`, or one no module has registered.
    pub fn setup_sensor(
        &self,
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
    ) -> Result<Arc<dyn Sensor>, ConfigError> {
        let sensor_type = config.get("sensor_type", None)?;
        let factory = self
            .factories
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&sensor_type)
            .cloned()
            .ok_or_else(|| {
                ConfigError::new(format!("Unknown temperature sensor '{sensor_type}'"))
            })?;
        factory(config, printer)
    }

    /// Note a sensor that was set up, upstream's `register_sensor`
    /// (`heaters.py:301-315`).
    ///
    /// The sensor's section name joins `available_sensors` and becomes one
    /// value of the `TEMPERATURE_WAIT` mux command, keyed by `SENSOR` — every
    /// sensor section answers the command under its own name, as upstream
    /// registers it right here. When a `gcode_id` is provided (or found in the
    /// section), the sensor is also stored in the M105 g-code-id table.
    ///
    /// `sensor` is the object whose `(cur, target)` M105 reports; heaters pass
    /// their `Arc<Heater>`, and sensor-only sections that do not yet carry a
    /// g-code-id object pass `None`.
    ///
    /// # Errors
    /// The mux value is already registered (one section loaded twice), or a
    /// g-code-id is already in the table, as upstream reports it.
    pub fn register_sensor(
        &self,
        config: &ConfigWrapper,
        sensor: Option<Arc<dyn GcodeTempSensor>>,
        gcode_id: Option<&str>,
    ) -> Result<(), ConfigError> {
        let identifier = config.identifier();
        self.sensors
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push(identifier.clone());
        let printer = self
            .printer
            .upgrade()
            .expect("the printer outlives the registry it built");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        gcode
            .register_mux_command_with_params(
                "TEMPERATURE_WAIT",
                "SENSOR",
                Some(identifier.as_str()),
                temperature_wait_handler(Weak::clone(&self.printer)),
                Some(TEMPERATURE_WAIT_HELP),
                &["MINIMUM", "MAXIMUM"],
            )
            .map_err(ConfigError::new)?;

        // Upstream's g-code-id registration (`heaters.py:308-315`): if no id
        // was passed, read it from the section; if the section has none
        // either, the sensor does not appear in M105.
        let gcode_id = match gcode_id {
            Some(id) => Some(id.to_string()),
            None => config.get_str("gcode_id"),
        };
        if let Some(gcode_id) = gcode_id {
            let mut table = self
                .gcode_id_to_sensor
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            if table.contains_key(&gcode_id) {
                return Err(ConfigError::new(format!(
                    "G-Code sensor id {gcode_id} already registered"
                )));
            }
            if let Some(sensor) = sensor {
                table.insert(gcode_id, sensor);
            }
        }
        Ok(())
    }

    /// Where `TEMPERATURE_WAIT SENSOR=<sensor_name>` reads its temperature
    /// from (`heaters.py:376-379`): the heaters table first, then the object
    /// the section registered under that name.
    ///
    /// # Errors
    /// No object answers to `sensor_name`, with upstream's `lookup_object`
    /// wording (`klippy/klippy.py:75-80`).
    fn wait_sensor(
        &self,
        printer: &Arc<Printer>,
        sensor_name: &str,
    ) -> Result<WaitSensor, CommandError> {
        if let Some(heater) = self
            .heaters
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(sensor_name)
            .cloned()
        {
            return Ok(WaitSensor::Heater(heater));
        }
        // A `[heater_generic <name>]` is the one heater whose section name is
        // not the short name the table is keyed by; its object wraps the same
        // heater upstream's `lookup_object` would hand back.
        if let Some(generic) = printer.lookup_object_as::<PrinterHeaterGeneric>(sensor_name) {
            return Ok(WaitSensor::Heater(Arc::clone(generic.heater())));
        }
        let object = printer
            .lookup_object(sensor_name)
            .ok_or_else(|| CommandError::new(format!("Unknown config object '{sensor_name}'")))?;
        Ok(WaitSensor::Object {
            name: sensor_name.to_string(),
            object,
        })
    }

    /// The registered monitor sections, for `get_status`.
    pub fn register_monitor(&self, config: &ConfigWrapper) {
        self.monitors
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push(config.identifier());
    }

    /// Build a heater from its section (`PrinterHeaters.setup_heater`).
    ///
    /// Reads and claims the heater options, builds the sensor and wires each of
    /// its readings into the control loop (`Heater::temperature_callback`),
    /// which is what drives the PWM: there is no timer of its own, so a heater
    /// with no readings never changes its output.
    ///
    /// `can_extrude` starts as upstream's `min_extrude_temp <= 0. or
    /// is_fileoutput` (`heaters.py:37-39`) and every reading recomputes it.
    /// File-output mode is how upstream runs its own test cases, where nothing
    /// answers the temperature queries — see [`Printer::is_fileoutput`].
    ///
    /// # Errors
    /// A duplicate heater name, an unknown sensor, or an invalid option.
    pub fn setup_heater(
        &self,
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        gcode_id: Option<&str>,
    ) -> Result<Arc<Heater>, ConfigError> {
        let identifier = config.identifier();
        let short_name = config
            .section()
            .sub
            .clone()
            .unwrap_or_else(|| config.section().id.clone());
        if self
            .heaters
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .contains_key(&short_name)
        {
            return Err(ConfigError::new(format!(
                "Heater {short_name} already registered"
            )));
        }

        let sensor = self.setup_sensor(config, printer)?;
        let min_temp = config.get_float("min_temp", None)?;
        let max_temp =
            config.get_float_bounded("max_temp", None, None, None, Some(min_temp), None)?;
        // Upstream returns the default without range-checking it
        // (`configfile._get_wrapper`), which matters here: a bed's
        // `max_temp` (e.g. 130) is below the default `min_extrude_temp` (170).
        let min_extrude_temp = match config.get_optional_float("min_extrude_temp")? {
            Some(value) => {
                if value < min_temp {
                    return Err(ConfigError::new(format!(
                        "Option 'min_extrude_temp' in section '{identifier}' must have minimum of {min_temp}"
                    )));
                }
                if value > max_temp {
                    return Err(ConfigError::new(format!(
                        "Option 'min_extrude_temp' in section '{identifier}' must have maximum of {max_temp}"
                    )));
                }
                value
            }
            None => 170.0,
        };
        let max_power =
            config.get_float_bounded("max_power", Some(1.0), None, Some(1.0), Some(0.0), None)?;
        let smooth_time =
            config.get_float_bounded("smooth_time", Some(1.0), None, None, Some(0.0), None)?;
        let pwm_cycle_time =
            config.get_float_bounded("pwm_cycle_time", Some(0.100), None, None, Some(0.0), None)?;

        // Set up the heater pin as a PWM output, as upstream does
        // (`heaters.py:56-61`). The `max_duration` limit is off: the host
        // drives the pin immediately, not on a print-time schedule (C1d).
        let heater_pin = config.get("heater_pin", None)?;
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let pwm = pins
            .setup_pwm(&heater_pin, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        pwm.setup_cycle_time(pwm_cycle_time, false);
        pwm.setup_max_duration(0.0);
        pwm.setup_start_value(0.0, 0.0);

        // Build the control algorithm.
        let control = match config
            .get_choice("control", &["watermark", "pid"], None)?
            .as_str()
        {
            "watermark" => Control::BangBang {
                max_delta: config.get_float_bounded(
                    "max_delta",
                    Some(2.0),
                    None,
                    None,
                    Some(0.0),
                    None,
                )?,
                heating: false,
            },
            _ => {
                let kp = config.get_float("pid_Kp", None)? / PID_PARAM_BASE;
                let ki = config.get_float("pid_Ki", None)? / PID_PARAM_BASE;
                let kd = config.get_float("pid_Kd", None)? / PID_PARAM_BASE;
                Control::Pid {
                    kp,
                    ki,
                    kd,
                    min_deriv_time: smooth_time,
                    temp_integ_max: if ki != 0.0 { max_power / ki } else { 0.0 },
                    prev_temp: AMBIENT_TEMP,
                    prev_temp_time: 0.0,
                    prev_temp_deriv: 0.0,
                    prev_temp_integ: 0.0,
                }
            }
        };

        sensor.setup_minmax(min_temp, max_temp);

        let heater = Arc::new(Heater {
            name: short_name.clone(),
            section_name: identifier,
            sensor,
            pwm: Some(pwm),
            min_temp,
            max_temp,
            min_extrude_temp,
            max_power,
            inv_smooth_time: 1.0 / smooth_time,
            state: Mutex::new(HeaterState {
                target_temp: 0.0,
                last_temp: 0.0,
                smoothed_temp: 0.0,
                last_temp_time: 0.0,
                // Upstream: `min_extrude_temp <= 0. or is_fileoutput`
                // (`heaters.py:37-39`). File-output mode is how upstream runs
                // its own cases, where the temperature queries are never
                // answered and so no reading ever flips this on again.
                can_extrude: min_extrude_temp <= 0.0 || printer.is_fileoutput(),
                last_pwm_value: 0.0,
                control: Box::new(control),
            }),
        });
        // The sensor delivers each reading to the control loop through a weak
        // handle, so the sensor does not keep the heater alive.
        let weak = Arc::downgrade(&heater);
        heater
            .sensor
            .setup_callback(Box::new(move |read_time, temp| {
                if let Some(heater) = weak.upgrade() {
                    heater.temperature_callback(read_time, temp);
                }
            }));
        // Upstream's `Heater.__init__` loads its own `verify_heater <name>`
        // object (`heaters.py:64`); here the heater, which owns the sibling
        // section's identity, builds it. A config with no such section reads as
        // all-defaults, and one that has it is claimed by these reads
        // (`config/wrapper.rs`, `ConfigWrapper::sibling`).
        let check_identifier = format!("verify_heater {short_name}");
        let check = verify_heater::HeaterCheck::new(
            config.sibling(&check_identifier).as_ref(),
            &short_name,
            printer,
        )?;
        printer.add_object(&check_identifier, check)?;
        // Upstream's `Heater.__init__` loads `pid_calibrate` right after
        // `verify_heater` (`heaters.py:64-65`), so `PID_CALIBRATE` exists as
        // soon as the first heater does — and not before that, as upstream.
        pid_calibrate::ensure(printer)?;
        self.heaters
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(short_name.clone(), Arc::clone(&heater));
        self.register_sensor(
            config,
            Some(Arc::clone(&heater) as Arc<dyn GcodeTempSensor>),
            gcode_id,
        )?;
        self.register_heater_command(printer, &short_name, Arc::clone(&heater))?;
        Ok(heater)
    }

    /// Register `SET_HEATER_TEMPERATURE` for one heater
    /// (`heaters.py:63-66`, `:362`).
    fn register_heater_command(
        &self,
        printer: &Arc<Printer>,
        short_name: &str,
        heater: Arc<Heater>,
    ) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
            let target = gcmd.get_float_default("TARGET", 0.0)?;
            heater.set_temp(target)
        });
        gcode
            .register_mux_command_with_params(
                "SET_HEATER_TEMPERATURE",
                "HEATER",
                Some(short_name),
                handler,
                Some("Set a heater temperature"),
                &["TARGET"],
            )
            .map_err(ConfigError::new)
    }

    /// The registered heaters' names (`PrinterHeaters.get_all_heaters`,
    /// `heaters.py:286-287`). Upstream reports its dict's insertion order; this
    /// is the map's name order, which nothing reads positionally.
    pub fn get_all_heaters(&self) -> Vec<String> {
        self.heaters
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .keys()
            .cloned()
            .collect()
    }

    fn available_sensors(&self) -> Vec<String> {
        self.sensors
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }
}

// ===========================================================================
// TEMPERATURE_WAIT
// ===========================================================================

/// Where `TEMPERATURE_WAIT` polls its temperature (`heaters.py:376-379`).
enum WaitSensor {
    /// A heater from the table — its own reading, unrounded
    /// (`Heater::get_temp`).
    Heater(Arc<Heater>),
    /// The object the section registered under its name, read from the
    /// `temperature` it reports. Upstream reads `psensor.get_temp(eventtime)`
    /// here; the sensor objects (`temperature_sensor.rs`, `temperature_fan.rs`)
    /// keep their reading behind the status they report, which rounds to two
    /// decimals — up to 0.005 °C at the wait's bounds.
    Object {
        name: String,
        object: Arc<dyn PrinterObject>,
    },
}

impl WaitSensor {
    /// One reading: the current temperature the command compares
    /// (`heaters.py:384-386`; upstream's `get_temp` pair minus the target it
    /// never uses).
    ///
    /// # Errors
    /// The object reports no `temperature`, which upstream's `get_temp` would
    /// fail on too — reported with the same wording its failed
    /// `lookup_object` has.
    fn get_temp(&self, eventtime: f64) -> Result<f64, CommandError> {
        match self {
            Self::Heater(heater) => Ok(heater.get_temp().0),
            Self::Object { name, object } => object
                .get_status(eventtime)
                .get("temperature")
                .and_then(Value::as_f64)
                .ok_or_else(|| CommandError::new(format!("Unknown config object '{name}'"))),
        }
    }
}

/// The `TEMPERATURE_WAIT` handler the mux stores — one per sensor value, all
/// reading the machine up at dispatch time (upstream registers its bound
/// method the same way, `heaters.py:305-307`).
fn temperature_wait_handler(printer: Weak<Printer>) -> CommandHandler {
    Arc::new(move |gcmd| {
        let printer = Weak::clone(&printer);
        Box::pin(async move { cmd_temperature_wait(&printer, gcmd).await })
    })
}

/// `TEMPERATURE_WAIT`: wait until a sensor's reading is within the bounds
/// (`heaters.py:367-389`), checked in upstream's order — the parameters first,
/// then file-output mode, then the sensor, then the polling loop.
///
/// # Errors
/// As upstream, verbatim: an unparseable parameter, `MAXIMUM` not above
/// `MINIMUM`, neither bound given, or a sensor name nothing answers to.
async fn cmd_temperature_wait(
    printer: &Weak<Printer>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let printer = printer
        .upgrade()
        .ok_or_else(|| CommandError::new("printer is gone"))?;
    let sensor_name = gcmd.get_str("SENSOR")?;
    let min_temp = gcmd.get_float_default("MINIMUM", f64::NEG_INFINITY)?;
    let max_temp = gcmd.get_float_default("MAXIMUM", f64::INFINITY)?;
    // Upstream reads `MAXIMUM` with `above=min_temp` (`heaters.py:370`), whose
    // shared `get` checks a bound only when the parameter was actually given
    // (a default returns before the bounds, `gcode.py:65-86`). The check is
    // written out here because the limit has to render the way Python's `%s`
    // would (`10.0`); the shared bound path formats f64 Rust-style (`10`) — see
    // `gcode.rs` `range_error`.
    if gcmd.get_command_parameters().contains_key("MAXIMUM") && max_temp <= min_temp {
        return Err(CommandError::new(format!(
            "Error on '{}': MAXIMUM must be above {min_temp:?}",
            gcmd.commandline()
        )));
    }
    if min_temp == f64::NEG_INFINITY && max_temp == f64::INFINITY {
        return Err(CommandError::new(
            "Error on 'TEMPERATURE_WAIT': missing MINIMUM or MAXIMUM.".to_string(),
        ));
    }
    if printer.is_fileoutput() {
        // Upstream's `debugoutput` return (`heaters.py:374-375`): its own cases
        // run against an input that never answers a temperature query.
        return Ok(());
    }
    let heaters = printer
        .lookup_object_as::<PrinterHeaters>(HEATERS_OBJECT)
        .ok_or_else(|| CommandError::new(format!("Unknown config object '{HEATERS_OBJECT}'")))?;
    let sensor = heaters.wait_sensor(&printer, &sensor_name)?;
    loop {
        // Upstream's `while not self.printer.is_shutdown()` (`heaters.py:383`).
        if printer.get_state_message().category == PrinterState::Shutdown {
            return Ok(());
        }
        let temp = sensor.get_temp(printer.eventtime())?;
        if temp >= min_temp && temp <= max_temp {
            return Ok(());
        }
        gcmd.respond_raw(&heaters._get_temp(printer.eventtime()));
        // Upstream parks the greenlet for a second (`reactor.pause`,
        // `heaters.py:389`); this reactor has no pause on purpose — the wait is
        // an ordinary timer the async command sleeps on.
        tokio::time::sleep(Duration::from_secs(1)).await;
    }
}

impl PrinterObject for PrinterHeaters {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({
            "available_heaters": self.get_all_heaters(),
            "available_sensors": self.available_sensors(),
            "available_monitors": self
                .monitors
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .clone(),
        })
    }
}

impl std::fmt::Debug for PrinterHeaters {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterHeaters")
            .field("sensors", &self.available_sensors())
            .finish_non_exhaustive()
    }
}

/// The single `heaters` object; the first caller creates it.
///
/// This is where the built-in sensor modules are loaded, since upstream reaches
/// them through `temperature_sensors.cfg` when the registry is first used.
///
/// # Errors
/// Registering the object, or bringing in a sensor module.
pub fn ensure(printer: &Arc<Printer>) -> Result<Arc<PrinterHeaters>, ConfigError> {
    if let Some(existing) = printer.lookup_object_as::<PrinterHeaters>(HEATERS_OBJECT) {
        return Ok(existing);
    }
    let heaters = Arc::new(PrinterHeaters::new(Arc::downgrade(printer)));
    printer.add_object(
        HEATERS_OBJECT,
        Arc::clone(&heaters) as Arc<dyn PrinterObject>,
    )?;
    heaters.register_commands(printer);
    ds18b20::ensure(&heaters)?;
    adc_temperature::ensure(&heaters)?;
    temperature_mcu::ensure(&heaters)?;
    spi_temperature::ensure(&heaters)?;
    temperature_combined::ensure(&heaters)?;
    Ok(heaters)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, Config, ConfigSection, ConfigValue};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
    use crate::core::klippy::mcu::McuError;
    use crate::core::klippy::pins::{
        DigitalOut, PinChip, PinError, PinParams, PrinterPins, PwmOut, PINS_OBJECT,
    };
    use crate::core::klippy::printer::PrinterState;
    use crate::core::klippy::reactor::{ManualReactor, Reactor};

    /// A sensor that accepts everything, for `setup_heater` tests.
    #[derive(Debug)]
    struct FakeSensor;

    impl Sensor for FakeSensor {
        fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) {}
        fn setup_callback(&self, _callback: SensorCallback) {}
    }

    /// A sensor whose readings the test delivers by hand.
    #[derive(Default)]
    struct ScriptedSensor {
        callback: Mutex<Option<SensorCallback>>,
    }

    impl std::fmt::Debug for ScriptedSensor {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            f.debug_struct("ScriptedSensor").finish_non_exhaustive()
        }
    }

    impl ScriptedSensor {
        /// Deliver one reading, as the sensor layer would.
        fn read(&self, read_time: f64, temp: f64) {
            let callback = self.callback.lock().unwrap_or_else(|p| p.into_inner());
            if let Some(callback) = callback.as_ref() {
                callback(read_time, temp);
            }
        }
    }

    impl Sensor for ScriptedSensor {
        fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) {}

        fn setup_callback(&self, callback: SensorCallback) {
            *self.callback.lock().unwrap_or_else(|p| p.into_inner()) = Some(callback);
        }
    }

    /// A PWM that accepts everything.
    #[derive(Debug)]
    struct FakePwm;

    impl PwmOut for FakePwm {
        fn setup_max_duration(&self, _max_duration: f64) {}
        fn setup_cycle_time(&self, _cycle_time: f64, _hardware: bool) {}
        fn setup_start_value(&self, _start: f64, _shutdown: f64) {}
        fn set_pwm(&self, _clock: u32, _value: f64) -> Result<(), McuError> {
            Ok(())
        }
        fn update_pwm(&self, _value: f64) -> Result<(), McuError> {
            Ok(())
        }
        fn next_aligned_clock(&self, clock: u32, _allow_early: f64) -> Result<u32, McuError> {
            Ok(clock)
        }
    }

    /// A chip that only exists so a pin description resolves.
    #[derive(Debug)]
    struct NoopChip;

    impl PinChip for NoopChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            Err(PinError::Unsupported("digital_out".to_string()))
        }

        fn setup_pwm(&self, _params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
            Ok(Arc::new(FakePwm))
        }
    }

    fn section(sensor_type: &str) -> ConfigSection {
        let mut section = ConfigSection::new("temperature_sensor", Some("probe"));
        section.parameters.insert(
            "sensor_type".to_string(),
            ConfigValue::Single(sensor_type.to_string()),
        );
        section
    }

    /// A printer with `gcode` and `pins` over a no-op chip.
    fn ready_printer() -> Arc<Printer> {
        ready_printer_on(ManualReactor::shared())
    }

    /// As [`ready_printer`], on a clock the test can step.
    fn ready_printer_on(reactor: Arc<dyn Reactor>) -> Arc<Printer> {
        let printer = Arc::new(Printer::new(reactor));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let pins = Arc::new(PrinterPins::new());
        pins.register_chip("mcu", Arc::new(NoopChip)).unwrap();
        printer.add_object(PINS_OBJECT, pins).unwrap();
        printer
    }

    #[test]
    fn test_a_factory_builds_the_sensor_it_registered() {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();

        assert!(heaters
            .setup_sensor(&ConfigWrapper::untracked(&section("made_up")), &printer)
            .unwrap_err()
            .to_string()
            .contains("Unknown temperature sensor 'made_up'"));
        // DS18B20 is brought in by `ensure`, as upstream's config does.
        let sensor = heaters
            .setup_sensor(&ConfigWrapper::untracked(&section("DS18B20")), &printer)
            .unwrap_err();
        assert!(sensor.to_string().contains("serial_no"), "{sensor}");
    }

    #[test]
    fn test_the_status_lists_what_was_registered() {
        // `register_sensor` now also registers the `TEMPERATURE_WAIT` mux
        // value, so the dispatcher has to be there.
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        heaters
            .register_sensor(&ConfigWrapper::untracked(&section("Fake")), None, None)
            .unwrap();

        assert_eq!(
            heaters.get_status(0.0)["available_sensors"],
            json!(["temperature_sensor probe"])
        );
    }

    #[test]
    fn test_setup_heater_claims_its_options_and_registers() {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(|_config, _printer| Ok(Arc::new(FakeSensor) as Arc<dyn Sensor>)),
        );

        let mut heater_section = ConfigSection::new("extruder", None);
        for (key, value) in [
            ("sensor_type", "Fake"),
            ("heater_pin", "PA0"),
            ("min_temp", "0"),
            ("max_temp", "250"),
            ("control", "pid"),
            ("pid_kp", "1"),
            ("pid_ki", "0.1"),
            ("pid_kd", "10"),
            ("min_extrude_temp", "0"),
        ] {
            heater_section
                .parameters
                .insert(key.to_string(), ConfigValue::Single(value.to_string()));
        }

        let heater = heaters
            .setup_heater(&ConfigWrapper::untracked(&heater_section), &printer, None)
            .unwrap();

        assert!(heater.can_extrude());
        heater.set_temp(200.0).unwrap();
        assert_eq!(heater.get_status()["target"], 200.0);
        assert!(heater.set_temp(300.0).is_err());
        assert_eq!(
            heaters.get_status(0.0)["available_heaters"],
            json!(["extruder"])
        );
    }

    /// `heaters.py:37-39`: file-output mode (upstream's `-o`, which
    /// `test_klippy.py` runs every case with) lets a heater extrude from a cold
    /// start — such a run never answers its temperature queries, so no reading
    /// would ever turn the flag on again.
    #[test]
    fn test_file_output_may_extrude_without_a_reading() {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(|_config, _printer| Ok(Arc::new(FakeSensor) as Arc<dyn Sensor>)),
        );
        let mut args = crate::core::klippy::api::StartArgs::collect("/tmp/printer.cfg", None);
        args.debug_output = Some("_test_output".to_string());
        printer.set_start_args(Arc::new(args));

        // No `min_extrude_temp`: the 170 default is far above anything this
        // run will ever read, and it reads nothing at all.
        let section = heater_section(&[
            ("sensor_type", "Fake"),
            ("heater_pin", "PA0"),
            ("min_temp", "0"),
            ("max_temp", "250"),
            ("control", "pid"),
            ("pid_kp", "1"),
            ("pid_ki", "0.1"),
            ("pid_kd", "10"),
        ]);
        let heater = heaters
            .setup_heater(&ConfigWrapper::untracked(&section), &printer, None)
            .unwrap();

        assert!(heater.can_extrude());
    }

    /// A `[extruder]`-style heater section with `options`.
    fn heater_section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("extruder", None);
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    #[test]
    fn test_the_bang_bang_control_toggles_the_output() {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(|_config, _printer| Ok(Arc::new(FakeSensor) as Arc<dyn Sensor>)),
        );
        let section = heater_section(&[
            ("sensor_type", "Fake"),
            ("heater_pin", "PA0"),
            ("min_temp", "0"),
            ("max_temp", "250"),
            ("min_extrude_temp", "50"),
            ("control", "watermark"),
            ("max_delta", "2"),
        ]);
        let heater = heaters
            .setup_heater(&ConfigWrapper::untracked(&section), &printer, None)
            .unwrap();

        // Below `min_extrude_temp`: no extrusion.
        heater.temperature_callback(0.0, 25.0);
        assert!(!heater.can_extrude());

        heater.set_temp(100.0).unwrap();
        // 90 is below target - max_delta, so the heater turns on.
        heater.temperature_callback(1.0, 90.0);
        assert_eq!(heater.get_status()["power"], 1.0);
        assert!(heater.can_extrude());
        // 103 is above target + max_delta, so it turns off.
        heater.temperature_callback(2.0, 103.0);
        assert_eq!(heater.get_status()["power"], 0.0);
    }

    #[test]
    fn test_the_pid_control_output_is_bounded() {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(|_config, _printer| Ok(Arc::new(FakeSensor) as Arc<dyn Sensor>)),
        );
        let section = heater_section(&[
            ("sensor_type", "Fake"),
            ("heater_pin", "PA0"),
            ("min_temp", "0"),
            ("max_temp", "250"),
            ("min_extrude_temp", "0"),
            ("control", "pid"),
            ("pid_kp", "64"),
            ("pid_ki", "1.4"),
            ("pid_kd", "128"),
        ]);
        let heater = heaters
            .setup_heater(&ConfigWrapper::untracked(&section), &printer, None)
            .unwrap();
        heater.set_temp(200.0).unwrap();

        heater.temperature_callback(0.0, 25.0);
        let power = heater.get_status()["power"].as_f64().unwrap();
        assert!((0.0..=1.0).contains(&power), "{power}");
        // Far below target, the PID output should be saturated high.
        assert!(power > 0.5, "{power}");
    }

    /// `PrinterHeaters.lookup_heater` (`heaters.py:288-292`).
    #[test]
    fn test_lookup_heater_names_the_heater_it_could_not_find() {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(|_config, _printer| Ok(Arc::new(FakeSensor) as Arc<dyn Sensor>)),
        );

        let err = heaters.lookup_heater("nope").unwrap_err();
        assert_eq!(err.to_string(), "Unknown heater 'nope'");

        let section = heater_section(&[
            ("sensor_type", "Fake"),
            ("heater_pin", "PA0"),
            ("min_temp", "0"),
            ("max_temp", "250"),
            ("control", "watermark"),
        ]);
        let heater = heaters
            .setup_heater(&ConfigWrapper::untracked(&section), &printer, None)
            .unwrap();
        assert!(Arc::ptr_eq(
            &heaters.lookup_heater("extruder").unwrap(),
            &heater
        ));
    }

    /// What `verify_heater` reads each second (`Heater.get_temp`).
    #[test]
    fn test_get_temp_reports_the_smoothed_temperature_and_the_target() {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(|_config, _printer| Ok(Arc::new(FakeSensor) as Arc<dyn Sensor>)),
        );
        let section = heater_section(&[
            ("sensor_type", "Fake"),
            ("heater_pin", "PA0"),
            ("min_temp", "0"),
            ("max_temp", "250"),
            ("control", "watermark"),
        ]);
        let heater = heaters
            .setup_heater(&ConfigWrapper::untracked(&section), &printer, None)
            .unwrap();

        // Nothing read yet, nothing asked for.
        assert_eq!(heater.get_temp(), (0.0, 0.0));
        // One reading smooths to itself (`smooth_time` 1 s).
        heater.temperature_callback(1.0, 20.0);
        heater.set_temp(200.0).unwrap();
        assert_eq!(heater.get_temp(), (20.0, 200.0));
    }

    /// A bed plus an option-less `[verify_heater heater_bed]`.
    const BED_WITH_EMPTY_CHECK: &str = "[mcu]\nserial: /dev/not-opened-yet\n\
        [heater_bed]\nheater_pin: PB1\nsensor_type: EPCOS 100K B57560G104F\n\
        sensor_pin: PK6\ncontrol: watermark\nmin_temp: 0\nmax_temp: 130\n\
        [verify_heater heater_bed]\n";

    /// `[verify_heater heater_bed]` with no options is a valid section: the
    /// check reads its four options with their defaults, which is what claims
    /// it (`verify_heater.py:22-29`, `config/validate.rs`).
    #[test]
    fn test_an_empty_verify_heater_section_is_claimed_not_rejected() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(BED_WITH_EMPTY_CHECK).expect("the config parses");

        printer
            .load_config(&config)
            .expect("an option-less [verify_heater heater_bed] loads");

        let check = printer
            .lookup_object("verify_heater heater_bed")
            .expect("the bed's check is registered");
        // Upstream's `HeaterCheck` has no `get_status` (`verify_heater.py`).
        assert!(!check.is_queryable());
    }

    /// A bed whose sensor never warms up: the check starts at `klippy:connect`
    /// and shuts the printer down once `check_gain_time` has passed with no
    /// gain, with upstream's message (`verify_heater.py:34-90`).
    #[test]
    fn test_a_stalled_heater_shuts_the_printer_down() {
        let reactor = Arc::new(ManualReactor::new());
        let printer = ready_printer_on(Arc::clone(&reactor) as Arc<dyn Reactor>);
        let heaters = ensure(&printer).unwrap();
        let sensor = Arc::new(ScriptedSensor::default());
        let built = Arc::clone(&sensor);
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(move |_config, _printer| Ok(Arc::clone(&built) as Arc<dyn Sensor>)),
        );
        let text = "[heater_bed]\n\
                    sensor_type: Fake\n\
                    heater_pin: PA0\n\
                    min_temp: 0\n\
                    max_temp: 250\n\
                    control: watermark\n\
                    [verify_heater heater_bed]\ncheck_gain_time: 5\n";
        let (config, _) = Config::from_text(text).expect("the config parses");
        let section = config
            .get_section("heater_bed")
            .expect("the bed's section exists");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        let heater = heaters.setup_heater(&wrapper, &printer, None).unwrap();
        heater.set_temp(200.0).unwrap();

        printer.send_event(&KlippyEvent::KlippyConnect);
        for tick in 1..=10 {
            // Stuck at 20 °C, 180 °C below the target.
            sensor.read(f64::from(tick), 20.0);
            reactor.advance(1.0);
        }

        assert_eq!(printer.get_state_message().category, PrinterState::Shutdown);
        assert_eq!(
            printer.get_state_message().message,
            "Heater heater_bed not heating at expected rate\n\
             See the 'verify_heater' section in docs/Config_Reference.md\n\
             for the parameters that control this check.\n"
        );
    }

    // ------------------------------------------------------------------
    // TEMPERATURE_WAIT
    // ------------------------------------------------------------------

    /// A printer with `[temperature_sensor probe]` loaded through its factory
    /// and registered the way the loader registers the section — the
    /// `TEMPERATURE_WAIT` value `register_sensor` grants included — plus the
    /// sensor whose readings the test delivers by hand.
    fn temperature_sensor_printer() -> (Arc<Printer>, Arc<GCodeDispatch>, Arc<ScriptedSensor>) {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        let sensor = Arc::new(ScriptedSensor::default());
        let built = Arc::clone(&sensor);
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(move |_config, _printer| Ok(Arc::clone(&built) as Arc<dyn Sensor>)),
        );
        let object = crate::core::klippy::extras::temperature_sensor::load_config_prefix(
            &ConfigWrapper::untracked(&section("Fake")),
            &printer,
        )
        .expect("the section loads");
        printer
            .add_object("temperature_sensor probe", object)
            .expect("one object per section");
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        (printer, gcode, sensor)
    }

    /// Everything `gcode` reported from here on, one entry per line.
    fn captured_lines(gcode: &Arc<GCodeDispatch>) -> Arc<Mutex<Vec<String>>> {
        let lines = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&lines);
        gcode.register_output_handler(Arc::new(move |line: &str| {
            sink.lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(line.to_string());
        }));
        lines
    }

    fn emitted(lines: &Arc<Mutex<Vec<String>>>) -> Vec<String> {
        lines.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }

    /// `register_sensor` gives every sensor section the mux value, and the
    /// command comes out with upstream's help text and its parameter names
    /// (`heaters.py:305-307`, `:366`).
    #[test]
    fn test_temperature_wait_is_registered_for_every_sensor() {
        let (_printer, gcode, _sensor) = temperature_sensor_printer();
        assert_eq!(
            gcode
                .command_help()
                .get("TEMPERATURE_WAIT")
                .map(String::as_str),
            Some("Wait for a temperature on a sensor")
        );
        assert_eq!(
            gcode.get_status(0.0)["commands"]["TEMPERATURE_WAIT"]["parameters"],
            json!(["SENSOR", "MINIMUM", "MAXIMUM"])
        );
    }

    /// A value no sensor registered is refused by the mux, with the listing of
    /// the values that do exist (`_cmd_mux`, `gcode.py:317-336`).
    #[tokio::test]
    async fn test_temperature_wait_names_an_unregistered_sensor() {
        let (_printer, gcode, _sensor) = temperature_sensor_printer();
        let err = gcode
            .run_script("TEMPERATURE_WAIT SENSOR=nope MINIMUM=1")
            .await
            .expect_err("no sensor answers to that name");
        assert_eq!(
            err.to_string(),
            "The value 'nope' is not valid for SENSOR. Options: 'temperature_sensor probe'"
        );
    }

    /// `SENSOR` is the mux key: without it the dispatch stops before the
    /// command runs (`gcode.py:322`, "missing SENSOR").
    #[tokio::test]
    async fn test_temperature_wait_requires_sensor() {
        let (_printer, gcode, _sensor) = temperature_sensor_printer();
        let err = gcode
            .run_script("TEMPERATURE_WAIT MINIMUM=1")
            .await
            .expect_err("the sensor is missing");
        assert_eq!(
            err.to_string(),
            "Error on 'TEMPERATURE_WAIT MINIMUM=1': missing SENSOR"
        );
    }

    /// Neither bound given, with upstream's own message
    /// (`heaters.py:371-373`).
    #[tokio::test]
    async fn test_temperature_wait_needs_minimum_or_maximum() {
        let (_printer, gcode, _sensor) = temperature_sensor_printer();
        let err = gcode
            .run_script("TEMPERATURE_WAIT SENSOR=\"temperature_sensor probe\"")
            .await
            .expect_err("both bounds are missing");
        assert_eq!(
            err.to_string(),
            "Error on 'TEMPERATURE_WAIT': missing MINIMUM or MAXIMUM."
        );
    }

    /// `MAXIMUM` is read with `above=min_temp` (`heaters.py:370`): the line
    /// comes back with upstream's bound wording.
    #[tokio::test]
    async fn test_temperature_wait_maximum_must_be_above_minimum() {
        let (_printer, gcode, _sensor) = temperature_sensor_printer();
        let line = "TEMPERATURE_WAIT SENSOR=\"temperature_sensor probe\" MINIMUM=10 MAXIMUM=5";
        let err = gcode.run_script(line).await.expect_err("5 is not above 10");
        assert_eq!(
            err.to_string(),
            format!("Error on '{line}': MAXIMUM must be above 10.0")
        );
    }

    /// A value the mux let through but no object answers to: upstream's
    /// `lookup_object` message (`klippy/klippy.py:75-80`).
    #[tokio::test]
    async fn test_temperature_wait_names_a_sensor_that_registered_no_object() {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        heaters
            .register_sensor(&ConfigWrapper::untracked(&section("Fake")), None, None)
            .unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        let err = gcode
            .run_script("TEMPERATURE_WAIT SENSOR=\"temperature_sensor probe\" MINIMUM=1")
            .await
            .expect_err("the section registered no object");
        assert_eq!(
            err.to_string(),
            "Unknown config object 'temperature_sensor probe'"
        );
    }

    /// A heater is read from the heaters table (`heaters.py:376-377`), and
    /// `MINIMUM` is inclusive (`temp >= min_temp`, `heaters.py:385`): a reading
    /// exactly at the bound returns at once, without one poll.
    #[tokio::test]
    async fn test_temperature_wait_reads_a_heater_from_the_heaters_table() {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(|_config, _printer| Ok(Arc::new(FakeSensor) as Arc<dyn Sensor>)),
        );
        let section = heater_section(&[
            ("sensor_type", "Fake"),
            ("heater_pin", "PA0"),
            ("min_temp", "0"),
            ("max_temp", "250"),
            ("min_extrude_temp", "0"),
            ("control", "watermark"),
        ]);
        let heater = heaters
            .setup_heater(&ConfigWrapper::untracked(&section), &printer, None)
            .unwrap();
        heater.temperature_callback(1.0, 30.0);
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        gcode
            .run_script("TEMPERATURE_WAIT SENSOR=extruder MINIMUM=30")
            .await
            .expect("the reading is exactly the minimum");
    }

    /// The loop polls about once a second and answers between polls — with
    /// upstream's `_get_temp` over an empty g-code-id table, `T:0` — until the
    /// reading enters the range (`heaters.py:383-389`).
    #[tokio::test(start_paused = true)]
    async fn test_temperature_wait_polls_until_the_reading_reaches_the_minimum() {
        let (_printer, gcode, sensor) = temperature_sensor_printer();
        let lines = captured_lines(&gcode);
        let reader = Arc::clone(&sensor);
        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(1_500)).await;
            reader.read(2.0, 55.0);
        });
        gcode
            .run_script("TEMPERATURE_WAIT SENSOR=\"temperature_sensor probe\" MINIMUM=50")
            .await
            .expect("the wait ends once the reading reaches the minimum");
        // One report per unsatisfied poll: at t=0 and t=1 s; the reading
        // arrives at t=1.5 s and the poll at t=2 s finds it in range.
        assert_eq!(emitted(&lines), ["T:0", "T:0"]);
    }

    /// The loop stops with the printer — upstream's
    /// `while not self.printer.is_shutdown()` (`heaters.py:383`) — ending the
    /// command without the bounds ever being met.
    #[tokio::test(start_paused = true)]
    async fn test_temperature_wait_stops_when_the_printer_shuts_down() {
        let (printer, gcode, _sensor) = temperature_sensor_printer();
        let lines = captured_lines(&gcode);
        let stopping = Arc::clone(&printer);
        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(500)).await;
            stopping.invoke_shutdown("test shutdown");
        });
        gcode
            .run_script("TEMPERATURE_WAIT SENSOR=\"temperature_sensor probe\" MINIMUM=100")
            .await
            .expect("the shutdown ends the wait");
        assert_eq!(emitted(&lines), ["T:0", "// Klipper state: Shutdown"]);
    }

    /// File-output mode returns instead of waiting (`heaters.py:374-375`):
    /// the input never answers a temperature query, so there is nothing to
    /// wait for. Without the return this would never end — `MINIMUM=100` is
    /// far above anything the sensor reports.
    #[tokio::test(start_paused = true)]
    async fn test_temperature_wait_returns_in_file_output_mode() {
        let (printer, gcode, _sensor) = temperature_sensor_printer();
        let mut args = crate::core::klippy::api::StartArgs::collect("/tmp/printer.cfg", None);
        args.debug_output = Some("_test_output".to_string());
        printer.set_start_args(Arc::new(args));
        gcode
            .run_script("TEMPERATURE_WAIT SENSOR=\"temperature_sensor probe\" MINIMUM=100")
            .await
            .expect("file output does not wait");
    }

    // ------------------------------------------------------------------
    // M105 g-code-id temperature reporting
    // ------------------------------------------------------------------

    /// A printer with one `[extruder]` heater whose g-code-id is `T0`,
    /// ready to report.
    fn m105_printer() -> (Arc<Printer>, Arc<GCodeDispatch>, Arc<Heater>) {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        let sensor = Arc::new(ScriptedSensor::default());
        let built = Arc::clone(&sensor);
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(move |_config, _printer| Ok(Arc::clone(&built) as Arc<dyn Sensor>)),
        );
        let section = heater_section(&[
            ("sensor_type", "Fake"),
            ("heater_pin", "PA0"),
            ("min_temp", "0"),
            ("max_temp", "250"),
            ("min_extrude_temp", "0"),
            ("control", "watermark"),
        ]);
        let heater = heaters
            .setup_heater(&ConfigWrapper::untracked(&section), &printer, Some("T0"))
            .unwrap();
        heater.temperature_callback(1.0, 20.0);
        heater.set_temp(200.0).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        (printer, gcode, heater)
    }

    /// M105 reports each registered g-code-id sensor as `"<id>:{cur:.1} /{target:.1}"`
    /// (`heaters.py:331-340`).
    #[tokio::test]
    async fn test_m105_reports_a_registered_gcode_id_sensor() {
        let (_printer, gcode, _heater) = m105_printer();
        let lines = captured_lines(&gcode);
        gcode.run_script("M105").await.expect("M105 runs");
        assert_eq!(emitted(&lines), ["T0:20.0 /200.0"]);
    }

    /// With no g-code-id sensors registered, M105 reports `"T:0"`
    /// (`heaters.py:338`).
    #[tokio::test]
    async fn test_m105_reports_t0_with_no_gcode_id_sensors() {
        let printer = ready_printer();
        let _heaters = ensure(&printer).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        let lines = captured_lines(&gcode);
        gcode.run_script("M105").await.expect("M105 runs");
        assert_eq!(emitted(&lines), ["T:0"]);
    }

    /// Before `klippy:ready`, `has_started` is false so M105 reports `"T:0"`
    /// even with a registered sensor (`heaters.py:334`).
    #[tokio::test]
    async fn test_m105_reports_t0_before_ready() {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(|_config, _printer| Ok(Arc::new(FakeSensor) as Arc<dyn Sensor>)),
        );
        let section = heater_section(&[
            ("sensor_type", "Fake"),
            ("heater_pin", "PA0"),
            ("min_temp", "0"),
            ("max_temp", "250"),
            ("min_extrude_temp", "0"),
            ("control", "watermark"),
        ]);
        let heater = heaters
            .setup_heater(&ConfigWrapper::untracked(&section), &printer, Some("T0"))
            .unwrap();
        heater.temperature_callback(1.0, 20.0);
        heater.set_temp(200.0).unwrap();
        // No KlippyReady event — has_started stays false.
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        let lines = captured_lines(&gcode);
        gcode.run_script("M105").await.expect("M105 runs");
        assert_eq!(emitted(&lines), ["T:0"]);
    }

    /// A duplicate g-code-id is rejected with upstream's message
    /// (`heaters.py:312-314`).
    #[test]
    fn test_duplicate_gcode_id_is_rejected() {
        let printer = ready_printer();
        let heaters = ensure(&printer).unwrap();
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(|_config, _printer| Ok(Arc::new(FakeSensor) as Arc<dyn Sensor>)),
        );
        let section = heater_section(&[
            ("sensor_type", "Fake"),
            ("heater_pin", "PA0"),
            ("min_temp", "0"),
            ("max_temp", "250"),
            ("min_extrude_temp", "0"),
            ("control", "watermark"),
        ]);
        heaters
            .setup_heater(&ConfigWrapper::untracked(&section), &printer, Some("T0"))
            .unwrap();
        // A second heater under a different name with the same g-code-id.
        let mut section2 = ConfigSection::new("heater_generic", Some("extra"));
        for (key, value) in [
            ("sensor_type", "Fake"),
            ("heater_pin", "PA1"),
            ("min_temp", "0"),
            ("max_temp", "250"),
            ("control", "watermark"),
        ] {
            section2
                .parameters
                .insert(key.to_string(), ConfigValue::Single(value.to_string()));
        }
        let err = heaters
            .setup_heater(&ConfigWrapper::untracked(&section2), &printer, Some("T0"))
            .unwrap_err();
        assert_eq!(err.to_string(), "G-Code sensor id T0 already registered");
    }

    /// TEMPERATURE_WAIT echoes the real M105 temperature report — not the
    /// hardcoded `T:0` — when a g-code-id sensor is registered
    /// (`heaters.py:388`).
    #[tokio::test(start_paused = true)]
    async fn test_temperature_wait_reports_real_gcode_id_temp() {
        let (printer, gcode, heater) = m105_printer();
        let lines = captured_lines(&gcode);
        // The reading (20 °C) is below MINIMUM=100, so the wait polls.
        let reader = Arc::clone(&heater);
        tokio::spawn(async move {
            tokio::time::sleep(Duration::from_millis(1_500)).await;
            reader.temperature_callback(2.0, 105.0);
        });
        gcode
            .run_script("TEMPERATURE_WAIT SENSOR=extruder MINIMUM=100")
            .await
            .expect("the wait ends once the reading reaches the minimum");
        // Each poll reports the real table: `T0:20.0 /200.0` at t=0 and t=1 s.
        assert_eq!(emitted(&lines), ["T0:20.0 /200.0", "T0:20.0 /200.0"]);
    }
}
