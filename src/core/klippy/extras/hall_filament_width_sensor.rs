//! `[hall_filament_width_sensor]` — filament width from two analog Hall
//! sensors (upstream `klippy/extras/hall_filament_width_sensor.py`).
//!
//! Two analog inputs (`adc1`, `adc2`) sit either side of the filament. The sum
//! of their raw counts maps to a diameter through the calibration line
//! `Cal_dia1`/`Cal_dia2` × `Raw_dia1`/`Raw_dia2` (`:122-125`), smoothed 5/6 old
//! + 1/6 new on every `adc2` report (`:126`). Every `measurement_interval` mm of
//! extrusion the smoothed diameter is queued `measurement_delay` mm ahead of the
//! nozzle (`:130-149`). The queue decides the flow compensation (`M221`) the way
//! upstream's `extrude_factor_update_event` does (`:151-209`), and the same
//! diameter drives the shared [`RunoutHelper`] as a runout condition
//! (`min_diameter`..`max_diameter`, `:175-176`).
//!
//! # The timer
//!
//! Upstream registers the timer in `__init__` and arms it at `klippy:ready`
//! (`:74-75`, `:109`); here it is created at ready ([`HallFilamentWidthSensor::arm`]),
//! so a config that never becomes ready leaves no timer behind. One period is
//! [`HallFilamentWidthSensor::update_at`], which the tests step directly instead
//! of waiting on a clock.
//!
//! A timer callback here is synchronous and `GCodeDispatch::run_script` is not,
//! so the `M221` calls are run detached — the shape `delayed_gcode` uses.
//!
//! # What is not here
//!
//! Upstream's `toolhead.get_position()[3]` (`:170-171`) is
//! [`ToolHeadObject::position`], which answers `None` before the planner exists
//! (a config without `[printer]`, or before connect); such a period is skipped
//! and the schedule kept. Upstream cannot see that state — its timer only starts
//! at `klippy:ready`.

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::filament_switch_sensor::RunoutHelper;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{sync, CommandError, GCodeDispatch, GcodeCommand, GCODE_OBJECT};
use crate::core::klippy::load::section;
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};
use crate::core::klippy::reactor::TimerHandle;

// The bare section only: upstream has `load_config`, no prefix form
// (`hall_filament_width_sensor.py:311`).
section!("hall_filament_width_sensor", order = 30, load = load_config);

/// The sampling both inputs are configured with
/// (`hall_filament_width_sensor.py:8-10`); the remaining `setup_adc_sample`
/// arguments keep upstream's defaults (`minval=0`, `maxval=1`,
/// `range_check_count=0`, `batch_num=1`).
const ADC_REPORT_TIME: f64 = 0.500;
const ADC_SAMPLE_TIME: f64 = 0.03;
const ADC_SAMPLE_COUNT: u32 = 15;

/// The `toolhead` object upstream reads the extruder position from
/// (`hall_filament_width_sensor.py:170`).
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The name the timer reports in a latency warning.
const TIMER_NAME: &str = "hall_filament_width_sensor";

/// The raw-count scale of both inputs: the ADC hands a `0.0..=1.0` fraction and
/// upstream rounds `read_value * 10000` (`:114`, `:119`).
const RAW_SCALE: f64 = 10000.0;

/// The options, read once and never written again
/// (`hall_filament_width_sensor.py:16-49`).
struct HallConfig {
    /// `Cal_dia1` / `Cal_dia2`: the widths the two calibration counts mean.
    dia1: f64,
    dia2: f64,
    /// `Raw_dia1` / `Raw_dia2`: the summed raw counts at those widths.
    raw_dia1: i64,
    raw_dia2: i64,
    /// `measurement_interval`: millimetres of extrusion between readings.
    measurement_interval: f64,
    /// `default_nominal_filament_diameter`.
    nominal_dia: f64,
    /// `measurement_delay`: millimetres between the sensor and the nozzle.
    measurement_delay: f64,
    /// The flow-compensation window — upstream's `min_diameter`/`max_diameter`,
    /// `nominal ± max_difference` (`:27-30`).
    flow_min_dia: f64,
    flow_max_dia: f64,
    /// The runout window — upstream's `runout_dia_min`/`runout_dia_max`, fed by
    /// the `min_diameter`/`max_diameter` options (`:33-34`). Two different pairs
    /// of bounds in upstream, hence two names here.
    runout_min_dia: f64,
    runout_max_dia: f64,
    /// `use_current_dia_while_delay`.
    use_current_dia_while_delay: bool,
}

/// The sensor state upstream keeps in plain attributes (`:31`, `:50-59`).
struct HallState {
    /// `is_active`: whether readings are acted on.
    is_active: bool,
    /// The smoothed diameter (`self.diameter`).
    diameter: f64,
    /// `lastFilamentWidthReading` / `lastFilamentWidthReading2`, the raw counts.
    last_reading1: i64,
    last_reading2: i64,
    /// `filament_array`: `(extruder position, diameter)` pairs, oldest first.
    filament_array: Vec<(f64, f64)>,
    /// The width in effect at the nozzle (`self.filament_width`).
    filament_width: f64,
    /// `firstExtruderUpdatePosition`: where the first queued reading takes over.
    first_update_position: f64,
    /// `logging`.
    is_log: bool,
    /// `enable_flow_compensation`.
    flow_compensation: bool,
    /// The deferred disable/clear handshake (`:50-53`).
    sensor_disable_pending: bool,
    flow_comp_disable_pending: bool,
    array_clear_pending: bool,
}

/// One period's decisions: the scripts upstream hands to `gcode.run_script`, the
/// lines it responds with, and the next wake time (`None` retires the timer).
struct UpdateOutcome {
    scripts: Vec<String>,
    log_lines: Vec<String>,
    next: Option<f64>,
}

/// One `[hall_filament_width_sensor]`.
pub struct HallFilamentWidthSensor {
    /// The machine, for its reactor and the toolhead; weak so holding the object
    /// does not keep the machine alive.
    printer: Weak<Printer>,
    /// The dispatcher the `M221` calls, the log lines and the commands go
    /// through.
    gcode: Arc<GCodeDispatch>,
    config: HallConfig,
    state: Mutex<HallState>,
    /// Upstream's `RunoutHelper` for this section (`:76-77`): the width sensor
    /// reports presence through it, so `QUERY_FILAMENT_SENSOR` and the runout
    /// policy options work unchanged.
    runout_helper: Arc<RunoutHelper>,
    /// The measurement timer; armed at ready and by `ENABLE_FILAMENT_WIDTH_SENSOR`.
    timer: Mutex<Option<TimerHandle>>,
    /// The handle the timer callback upgrades.
    self_ref: Weak<HallFilamentWidthSensor>,
}

impl HallFilamentWidthSensor {
    /// Read the section and build the two analog inputs
    /// (`hall_filament_width_sensor.py:16-72`).
    ///
    /// # Errors
    /// A missing/invalid option, a calibration whose two raw counts agree, a
    /// `max_difference` that swallows the nominal diameter, a pin the pin layer
    /// refuses, or any option the runout helper refuses.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let pin1 = config.get("adc1", None)?;
        let pin2 = config.get("adc2", None)?;
        let dia1 = config.get_float("Cal_dia1", Some(1.5))?;
        let dia2 = config.get_float("Cal_dia2", Some(2.0))?;
        let raw_dia1 = config.get_int("Raw_dia1", Some(9500))?;
        let raw_dia2 = config.get_int("Raw_dia2", Some(10500))?;
        let measurement_interval = config.get_int("measurement_interval", Some(10))? as f64;
        let nominal_dia = config.get_float_bounded(
            "default_nominal_filament_diameter",
            None,
            None,
            None,
            Some(1.0),
            None,
        )?;
        let measurement_delay =
            config.get_float_bounded("measurement_delay", None, None, None, Some(0.0), None)?;
        let measurement_max_difference = config.get_float("max_difference", Some(0.2))?;
        let is_active = config.get_bool("enable", Some(false))?;
        // `min_diameter` defaults to 1.0, not to `nominal - max_difference`
        // (`:33`); `max_diameter` defaults to the flow window's upper bound
        // (`:34`).
        let runout_min_dia = config.get_float("min_diameter", Some(1.0))?;
        let runout_max_dia = config.get_float(
            "max_diameter",
            Some(nominal_dia + measurement_max_difference),
        )?;
        let is_log = config.get_bool("logging", Some(false))?;
        let flow_compensation = config.get_bool("enable_flow_compensation", Some(true))?;
        let use_current_dia_while_delay =
            config.get_bool("use_current_dia_while_delay", Some(false))?;

        if raw_dia1 == raw_dia2 {
            return Err(ConfigError::new(
                "hall_filament_width_sensor: raw_dia1 and raw_dia2 must be different",
            ));
        }
        let flow_max_dia = nominal_dia + measurement_max_difference;
        let flow_min_dia = nominal_dia - measurement_max_difference;
        if flow_min_dia <= 0.0 {
            return Err(ConfigError::new(
                "hall_filament_width_sensor: max_difference must be less than \
                 default_nominal_filament_diameter",
            ));
        }

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .ok_or_else(|| ConfigError::new("adc1: the pins object is not registered"))?;
        let adc1 = pins
            .setup_adc(&pin1, None)
            .map_err(|err| ConfigError::new(format!("adc1: {err}")))?;
        let adc2 = pins
            .setup_adc(&pin2, None)
            .map_err(|err| ConfigError::new(format!("adc2: {err}")))?;
        for adc in [&adc1, &adc2] {
            adc.setup_adc_sample(
                ADC_REPORT_TIME,
                ADC_SAMPLE_TIME,
                ADC_SAMPLE_COUNT,
                1,
                0.0,
                1.0,
                0,
            );
        }

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .ok_or_else(|| {
                ConfigError::new("the gcode dispatcher is not registered".to_string())
            })?;
        let runout_helper = Arc::new(RunoutHelper::new(config, printer)?);
        runout_helper.attach(printer)?;

        let hall_config = HallConfig {
            dia1,
            dia2,
            raw_dia1,
            raw_dia2,
            measurement_interval,
            nominal_dia,
            measurement_delay,
            flow_min_dia,
            flow_max_dia,
            runout_min_dia,
            runout_max_dia,
            use_current_dia_while_delay,
        };

        let object = Arc::new_cyclic(|weak: &Weak<HallFilamentWidthSensor>| {
            let for_adc1 = weak.clone();
            adc1.setup_adc_callback(Box::new(move |samples| {
                if let Some(sensor) = for_adc1.upgrade() {
                    sensor.handle_adc1_report(samples);
                }
            }));
            let for_adc2 = weak.clone();
            adc2.setup_adc_callback(Box::new(move |samples| {
                if let Some(sensor) = for_adc2.upgrade() {
                    sensor.handle_adc2_report(samples);
                }
            }));
            Self {
                printer: Arc::downgrade(printer),
                gcode,
                config: hall_config,
                state: Mutex::new(HallState {
                    is_active,
                    diameter: nominal_dia,
                    last_reading1: 0,
                    last_reading2: 0,
                    filament_array: Vec::new(),
                    filament_width: nominal_dia,
                    first_update_position: 0.0,
                    is_log,
                    flow_compensation,
                    sensor_disable_pending: false,
                    flow_comp_disable_pending: false,
                    array_clear_pending: false,
                }),
                runout_helper,
                timer: Mutex::new(None),
                self_ref: weak.clone(),
            }
        });
        // Upstream sets the runout helper's flag from `enable` at load (`:78`).
        object.runout_helper.set_sensor_enabled(is_active);
        Ok(object)
    }

    /// Register the seven commands and the `klippy:ready` handler
    /// (`hall_filament_width_sensor.py:78-108`).
    ///
    /// # Errors
    /// A command name already taken.
    fn attach(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        self.register_command(
            "QUERY_FILAMENT_WIDTH",
            QUERY_FILAMENT_WIDTH_HELP,
            Self::cmd_query_filament_width,
        )?;
        self.register_command(
            "RESET_FILAMENT_WIDTH_SENSOR",
            RESET_FILAMENT_WIDTH_SENSOR_HELP,
            Self::cmd_clear_filament_array,
        )?;
        self.register_command(
            "DISABLE_FILAMENT_WIDTH_SENSOR",
            DISABLE_FILAMENT_WIDTH_SENSOR_HELP,
            Self::cmd_disable,
        )?;
        self.register_command(
            "ENABLE_FILAMENT_WIDTH_SENSOR",
            ENABLE_FILAMENT_WIDTH_SENSOR_HELP,
            Self::cmd_enable,
        )?;
        self.register_command(
            "QUERY_RAW_FILAMENT_WIDTH",
            QUERY_RAW_FILAMENT_WIDTH_HELP,
            Self::cmd_get_raw_values,
        )?;
        self.register_command(
            "ENABLE_FILAMENT_WIDTH_LOG",
            ENABLE_FILAMENT_WIDTH_LOG_HELP,
            Self::cmd_log_enable,
        )?;
        self.register_command(
            "DISABLE_FILAMENT_WIDTH_LOG",
            DISABLE_FILAMENT_WIDTH_LOG_HELP,
            Self::cmd_log_disable,
        )?;

        let weak = Arc::downgrade(self);
        printer.register_event_handler(
            KlippyEvent::KlippyReady,
            Box::new(move |_| {
                if let Some(sensor) = weak.upgrade() {
                    sensor.arm();
                }
            }),
        );
        Ok(())
    }

    /// Register one command through a `Weak`: the command table outlives the
    /// object on a restart, so a strong handle would close a cycle.
    fn register_command(
        self: &Arc<Self>,
        name: &str,
        help: &str,
        handler: fn(&Self, &GcodeCommand) -> Result<(), CommandError>,
    ) -> Result<(), ConfigError> {
        let weak = Arc::downgrade(self);
        self.gcode
            .register_command(
                name,
                sync(move |gcmd: &GcodeCommand| {
                    let sensor = weak.upgrade().ok_or_else(|| {
                        CommandError::new("the hall filament width sensor is gone")
                    })?;
                    handler(&sensor, gcmd)
                }),
                Some(help),
                false,
            )
            .map_err(ConfigError::new)
    }

    /// Upstream's `adc_callback` (`:111-114`): store the first input's raw
    /// count.
    fn handle_adc1_report(&self, samples: &[(f64, f64)]) {
        let Some((_read_time, read_value)) = samples.last() else {
            return;
        };
        self.lock(&self.state).last_reading1 = py_round_int(read_value * RAW_SCALE);
    }

    /// Upstream's `adc2_callback` (`:116-128`): store the second input's raw
    /// count and, when the sensor is active, fit the diameter from the two
    /// counts and smooth it; an inactive sensor pins it to the nominal value.
    fn handle_adc2_report(&self, samples: &[(f64, f64)]) {
        let Some((_read_time, read_value)) = samples.last() else {
            return;
        };
        let config = &self.config;
        let mut state = self.lock(&self.state);
        state.last_reading2 = py_round_int(read_value * RAW_SCALE);
        if state.is_active {
            let raw_sum = (state.last_reading1 + state.last_reading2) as f64;
            let diameter_new = py_round_digits(
                (config.dia2 - config.dia1) / (config.raw_dia2 - config.raw_dia1) as f64
                    * (raw_sum - config.raw_dia1 as f64)
                    + config.dia1,
                2,
            );
            state.diameter = (5.0 * state.diameter + diameter_new) / 6.0;
        } else {
            state.diameter = config.nominal_dia;
        }
    }

    /// Arm the measurement timer: upstream's
    /// `reactor.update_timer(self.extrude_factor_update_timer, reactor.NOW)`
    /// (`:109`, `:263-264`).
    ///
    /// This host's reactor has no `update_timer` (`delayed_gcode` documents the
    /// same gap), so the previous handle is cancelled and a fresh timer takes its
    /// place at the current instant.
    fn arm(&self) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        let reactor = printer.reactor();
        let weak = self.self_ref.clone();
        let handle = reactor.register_timer_named(
            TIMER_NAME,
            Box::new(move |eventtime| match weak.upgrade() {
                Some(sensor) => sensor.extrude_factor_update_event(eventtime),
                None => None,
            }),
            reactor.monotonic(),
        );
        let mut timer = self.lock(&self.timer);
        if let Some(previous) = timer.replace(handle) {
            previous.cancel();
        }
    }

    /// One period (`:151-209`), with the extruder position read from the
    /// toolhead.
    fn extrude_factor_update_event(&self, eventtime: f64) -> Option<f64> {
        let Some(last_epos) = self.toolhead_e_position() else {
            // No toolhead to read (module docs): skip the period and keep the
            // schedule.
            return self.next_wake(eventtime);
        };
        let outcome = self.update_at(eventtime, last_epos);
        for line in &outcome.log_lines {
            self.gcode.respond_info(line, true);
        }
        for script in &outcome.scripts {
            self.run_script(script);
        }
        outcome.next
    }

    /// The wake time the current `is_active` implies (`:206-209`).
    fn next_wake(&self, eventtime: f64) -> Option<f64> {
        if self.lock(&self.state).is_active {
            Some(eventtime + 1.0)
        } else {
            None
        }
    }

    /// One period of upstream's timer body for a known extruder position.
    ///
    /// The position is a parameter rather than a toolhead read so a test can
    /// step a period without a connected planner; [`Self::extrude_factor_update_event`]
    /// is the only production caller that reads it (`:170-171`).
    fn update_at(&self, eventtime: f64, last_epos: f64) -> UpdateOutcome {
        let mut log_lines = Vec::new();
        let config = &self.config;
        let mut state = self.lock(&self.state);

        let mut scripts = Vec::new();
        // The pending disable/clear handshake: the first of the three flags to
        // be set resets the extruder factor before anything else (`:152-156`).
        if state.sensor_disable_pending
            || state.flow_comp_disable_pending
            || state.array_clear_pending
        {
            scripts.push("M221 S100".to_string());
        }
        if state.array_clear_pending {
            state.array_clear_pending = false;
            state.filament_array.clear();
        }
        if state.flow_comp_disable_pending {
            state.flow_comp_disable_pending = false;
        }
        if state.sensor_disable_pending {
            state.sensor_disable_pending = false;
            state.filament_array.clear();
            state.is_active = false;
            drop(state);
            self.runout_helper.set_sensor_enabled(false);
            self.runout_helper.note_filament_present(eventtime, true);
            return UpdateOutcome {
                scripts,
                log_lines,
                next: None,
            };
        }

        self.update_filament_array(&mut state, last_epos, &mut log_lines);

        // The runout condition is a window on the diameter, not on the queued
        // width (`:175-176`).
        self.runout_helper.note_filament_present(
            eventtime,
            config.runout_min_dia <= state.diameter && state.diameter <= config.runout_max_dia,
        );

        if state.diameter > config.runout_min_dia {
            if let Some(&(pending_position, _)) = state.filament_array.first() {
                if pending_position <= last_epos {
                    let (_position, width) = state.filament_array.remove(0);
                    state.filament_width = width;
                } else if config.use_current_dia_while_delay
                    && state.first_update_position == pending_position
                {
                    state.filament_width = state.diameter;
                } else if state.first_update_position == pending_position {
                    state.filament_width = config.nominal_dia;
                }
                if state.flow_compensation {
                    if state.filament_width <= config.flow_max_dia
                        && state.filament_width >= config.flow_min_dia
                    {
                        let percentage = py_round_int(
                            config.nominal_dia.powi(2) / state.filament_width.powi(2) * 100.0,
                        );
                        scripts.push(format!("M221 S{percentage}"));
                    } else {
                        scripts.push("M221 S100".to_string());
                    }
                }
            }
        } else {
            if state.flow_compensation {
                scripts.push("M221 S100".to_string());
            }
            state.filament_array.clear();
        }

        let next = if state.is_active {
            Some(eventtime + 1.0)
        } else {
            None
        };
        UpdateOutcome {
            scripts,
            log_lines,
            next,
        }
    }

    /// Upstream's `update_filament_array` (`:130-149`): queue a reading every
    /// `measurement_interval` mm, `measurement_delay` mm ahead of the nozzle.
    fn update_filament_array(
        &self,
        state: &mut HallState,
        last_epos: f64,
        log_lines: &mut Vec<String>,
    ) {
        let config = &self.config;
        match state.filament_array.last() {
            Some(&(last_position, _)) => {
                let next_reading_position = last_position + config.measurement_interval;
                if next_reading_position <= last_epos + config.measurement_delay {
                    state
                        .filament_array
                        .push((last_epos + config.measurement_delay, state.diameter));
                    if state.is_log {
                        log_lines.push(format!("Filament width:{:.3}", state.diameter));
                    }
                }
            }
            None => {
                state
                    .filament_array
                    .push((config.measurement_delay + last_epos, state.diameter));
                state.first_update_position = config.measurement_delay + last_epos;
            }
        }
    }

    /// The extruder position upstream reads as `toolhead.get_position()[3]`
    /// (`:170-171`), or `None` before the planner exists.
    fn toolhead_e_position(&self) -> Option<f64> {
        let printer = self.printer.upgrade()?;
        let toolhead = printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)?;
        toolhead.position().map(|position| position.e())
    }

    /// Run one of upstream's `gcode.run_script` calls.
    ///
    /// A timer callback is synchronous and `GCodeDispatch::run_script` is not,
    /// so the script runs detached (`delayed_gcode` uses the same shape); a host
    /// with no runtime to spawn on logs a warning instead.
    fn run_script(&self, script: &str) {
        let gcode = Arc::clone(&self.gcode);
        let script = script.to_string();
        match tokio::runtime::Handle::try_current() {
            Ok(handle) => {
                handle.spawn(async move {
                    if let Err(error) = gcode.run_script(&script).await {
                        tracing::error!("Script running error: {error}");
                    }
                });
            }
            Err(_) => tracing::warn!(
                "hall_filament_width_sensor: running '{script}' needs an async runtime"
            ),
        }
    }

    /// Upstream's `cmd_M407` (`:213-231`).
    fn cmd_query_filament_width(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let (is_active, diameter, flow_compensation) = {
            let state = self.lock(&self.state);
            (state.is_active, state.diameter, state.flow_compensation)
        };
        let mut response = String::from("Filament diameter: ");
        if is_active {
            response.push_str(&format!("{diameter:.3} mm"));
        } else {
            response.push_str("sensor disabled");
        }
        response.push_str("\nFilament state:");
        if is_active {
            if self.runout_helper.filament_present() {
                response.push_str(" present");
            } else {
                response.push_str(" not present");
            }
        } else {
            response.push_str(" sensor disabled");
        }
        response.push_str("\nWidth sensor:");
        response.push_str(if is_active { " ON" } else { " OFF" });
        response.push_str("\nFlow compensation:");
        response.push_str(if flow_compensation { " ON" } else { " OFF" });
        gcmd.respond_info(&response);
        Ok(())
    }

    /// Upstream's `cmd_ClearFilamentArray` (`:234-240`).
    fn cmd_clear_filament_array(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let is_active = {
            let mut state = self.lock(&self.state);
            state.filament_array.clear();
            if state.is_active {
                state.array_clear_pending = true;
            }
            state.is_active
        };
        if !is_active {
            self.run_script("M221 S100");
        }
        gcmd.respond_info("Filament width measurements cleared!");
        Ok(())
    }

    /// Upstream's `cmd_M405` (`:245-265`): enable the sensor, optionally
    /// switching flow compensation.
    fn cmd_enable(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        // `gcmd.get_int("FLOW_COMPENSATION", None, minval=0, maxval=1)` (`:246`)
        // — the parameter is optional *and* bounded, which no shorthand getter
        // spells.
        let flow_comp = if gcmd
            .get_command_parameters()
            .contains_key("FLOW_COMPENSATION")
        {
            Some(gcmd.get(
                "FLOW_COMPENSATION",
                None::<i64>,
                parse_int,
                Some(0),
                Some(1),
                None,
                None,
            )?)
        } else {
            None
        };
        let (was_active, flow_compensation) = {
            let mut state = self.lock(&self.state);
            if let Some(flow_comp) = flow_comp {
                if state.flow_compensation && flow_comp == 0 {
                    state.flow_comp_disable_pending = true;
                    state.flow_compensation = false;
                } else if !state.flow_compensation && flow_comp != 0 {
                    state.flow_comp_disable_pending = false;
                    state.flow_compensation = true;
                }
            }
            let was_active = state.is_active;
            state.is_active = true;
            if !was_active {
                state.sensor_disable_pending = false;
            }
            (was_active, state.flow_compensation)
        };
        self.runout_helper.set_sensor_enabled(true);
        let response = format!(
            "Filament width sensor: ON\nFlow compensation: {}",
            if flow_compensation { "ON" } else { "OFF" }
        );
        if !was_active {
            self.arm();
        }
        gcmd.respond_info(&response);
        Ok(())
    }

    /// Upstream's `cmd_M406` (`:269-278`): a running sensor is disabled on the
    /// next period (so `M221 S100` lands before the readings stop), a stopped one
    /// is tidied up here.
    fn cmd_disable(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let response = "Filament width sensor: OFF";
        let was_active = {
            let mut state = self.lock(&self.state);
            if state.is_active {
                state.sensor_disable_pending = true;
            } else {
                state.filament_array.clear();
            }
            state.is_active
        };
        if !was_active {
            self.runout_helper.set_sensor_enabled(false);
            self.runout_helper.note_filament_present(self.now(), true);
            self.run_script("M221 S100");
        }
        gcmd.respond_info(response);
        Ok(())
    }

    /// Upstream's `cmd_Get_Raw_Values` (`:282-288`).
    fn cmd_get_raw_values(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let (reading1, reading2) = {
            let state = self.lock(&self.state);
            (state.last_reading1, state.last_reading2)
        };
        gcmd.respond_info(&format!(
            "ADC1={reading1} ADC2={reading2} RAW={}",
            reading1 + reading2
        ));
        Ok(())
    }

    /// Upstream's `cmd_log_enable` (`:301-303`).
    fn cmd_log_enable(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        self.lock(&self.state).is_log = true;
        gcmd.respond_info("Filament width logging: ON");
        Ok(())
    }

    /// Upstream's `cmd_log_disable` (`:307-309`).
    fn cmd_log_disable(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        self.lock(&self.state).is_log = false;
        gcmd.respond_info("Filament width logging: OFF");
        Ok(())
    }

    /// The machine's monotonic clock, or `0.0` before the printer is reachable.
    fn now(&self) -> f64 {
        self.printer
            .upgrade()
            .map(|printer| printer.reactor().monotonic())
            .unwrap_or(0.0)
    }

    fn lock<'a, T>(&self, slot: &'a Mutex<T>) -> MutexGuard<'a, T> {
        slot.lock().unwrap_or_else(|poison| poison.into_inner())
    }
}

impl PrinterObject for HallFilamentWidthSensor {
    /// Upstream's `get_status` (`:290-297`): the runout helper's two keys plus
    /// the width sensor's own four.
    fn get_status(&self, _eventtime: f64) -> Value {
        let (diameter, raw, is_active, flow_compensation) = {
            let state = self.lock(&self.state);
            (
                state.diameter,
                state.last_reading1 + state.last_reading2,
                state.is_active,
                state.flow_compensation,
            )
        };
        json!({
            "filament_detected": self.runout_helper.filament_present(),
            "enabled": self.runout_helper.sensor_enabled(),
            "Diameter": diameter,
            "Raw": raw,
            "is_active": is_active,
            "flow_compensation_enabled": flow_compensation,
        })
    }
}

impl std::fmt::Debug for HallFilamentWidthSensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let state = self.lock(&self.state);
        f.debug_struct("HallFilamentWidthSensor")
            .field("diameter", &state.diameter)
            .field("is_active", &state.is_active)
            .field("flow_compensation", &state.flow_compensation)
            .field("filament_array", &state.filament_array.len())
            .finish_non_exhaustive()
    }
}

/// Upstream's command help texts, verbatim
/// (`hall_filament_width_sensor.py:211,233,242,267,280,299,305`).
const QUERY_FILAMENT_WIDTH_HELP: &str = "Report the filament width in mm and show sensor status";
const RESET_FILAMENT_WIDTH_SENSOR_HELP: &str = "Clear all filament width readings";
const ENABLE_FILAMENT_WIDTH_SENSOR_HELP: &str =
    "Enable the filament width sensor and enable or disable flow compensation";
const DISABLE_FILAMENT_WIDTH_SENSOR_HELP: &str = "Disable the filament width sensor";
const QUERY_RAW_FILAMENT_WIDTH_HELP: &str = "Report the raw filament width sensor values";
const ENABLE_FILAMENT_WIDTH_LOG_HELP: &str = "Enable filament width sensor logging";
const DISABLE_FILAMENT_WIDTH_LOG_HELP: &str = "Disable filament width sensor logging";

/// Python's `round(value)` for a whole float: halves go to the even neighbour.
fn py_round_int(value: f64) -> i64 {
    let floor = value.floor();
    let fraction = value - floor;
    let rounded = if fraction > 0.5 {
        floor + 1.0
    } else if fraction < 0.5 || (floor as i64) % 2 == 0 {
        floor
    } else {
        floor + 1.0
    };
    rounded as i64
}

/// Python's `round(value, digits)`: the float's exact decimal expansion rounded
/// half to even, which is what `format!`'s fixed precision does.
fn py_round_digits(value: f64, digits: usize) -> f64 {
    format!("{value:.digits$}").parse().unwrap_or(value)
}

/// An integer parameter, as `parse_int` in the dispatcher spells it.
fn parse_int(value: &str) -> Option<i64> {
    value.parse::<i64>().ok()
}

/// The factory the section declaration names
/// (`hall_filament_width_sensor.py:311 def load_config`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let sensor = HallFilamentWidthSensor::new(config, printer)?;
    sensor.attach(printer)?;
    Ok(sensor as Arc<dyn PrinterObject>)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::gcode::OutputHandler;
    use crate::core::klippy::reactor::{ManualReactor, Reactor};

    /// The section with every option at its upstream default except the four
    /// that are required.
    const CONFIG: &str = "[mcu]\nserial: /dev/not-opened-yet\n\
         [hall_filament_width_sensor]\nadc1: PA0\nadc2: PA1\n\
         default_nominal_filament_diameter: 1.75\nmeasurement_delay: 50\n";

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

    /// A loaded printer that has passed `klippy:ready`, which is when the
    /// dispatcher starts accepting the commands registered here.
    fn load_ready(text: &str) -> Arc<Printer> {
        let printer = load_ok(text);
        printer.send_event(&KlippyEvent::KlippyReady);
        printer
    }

    fn the_sensor(printer: &Arc<Printer>) -> Arc<HallFilamentWidthSensor> {
        printer
            .lookup_object_as::<HallFilamentWidthSensor>("hall_filament_width_sensor")
            .expect("[hall_filament_width_sensor] is registered")
    }

    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode`")
    }

    /// The line the dispatcher emits for one command, for the commands that only
    /// report.
    fn respond_line(gcode: &Arc<GCodeDispatch>, script: &str) -> String {
        let lines = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&lines);
        let handler: Arc<dyn OutputHandler> = Arc::new(move |line: &str| {
            sink.lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .push(line.to_string())
        });
        gcode.register_output_handler(handler);
        gcode.run_script_sync(script).expect("the command runs");
        let joined = lines
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .join("\n");
        joined
    }

    /// Every option defaults as upstream defaults it, and the two required ones
    /// have to be there.
    #[test]
    fn test_the_section_reads_every_option_at_its_default() {
        let printer = load_ok(CONFIG);
        let sensor = the_sensor(&printer);
        assert_eq!(sensor.config.dia1, 1.5);
        assert_eq!(sensor.config.dia2, 2.0);
        assert_eq!(sensor.config.raw_dia1, 9500);
        assert_eq!(sensor.config.raw_dia2, 10500);
        assert_eq!(sensor.config.measurement_interval, 10.0);
        assert_eq!(sensor.config.nominal_dia, 1.75);
        assert_eq!(sensor.config.measurement_delay, 50.0);
        // `max_difference` 0.2 either way.
        assert_eq!(sensor.config.flow_min_dia, 1.55);
        assert_eq!(sensor.config.flow_max_dia, 1.95);
        // `min_diameter` 1.0, `max_diameter` = nominal + max_difference.
        assert_eq!(sensor.config.runout_min_dia, 1.0);
        assert_eq!(sensor.config.runout_max_dia, 1.95);
        assert!(!sensor.config.use_current_dia_while_delay);

        let state = sensor.lock(&sensor.state);
        assert!(!state.is_active, "`enable` defaults to False");
        assert!(!state.is_log, "`logging` defaults to False");
        assert!(
            state.flow_compensation,
            "`enable_flow_compensation` defaults to True"
        );
        assert_eq!(state.diameter, 1.75);
        assert_eq!(state.filament_width, 1.75);
        assert!(state.filament_array.is_empty());
        assert_eq!(state.last_reading1, 0);
        assert_eq!(state.last_reading2, 0);
        drop(state);
        // The runout helper's flag follows `enable` at load (`:78`).
        assert!(!sensor.runout_helper.sensor_enabled());
    }

    /// The bounds upstream puts on the options are enforced with its wording,
    /// and its two cross-option checks with its messages (`:23-25`, `:42-49`).
    #[test]
    fn test_the_option_bounds_and_wording() {
        let (_, result) = load(&CONFIG.replace(
            "default_nominal_filament_diameter: 1.75",
            "default_nominal_filament_diameter: 1",
        ));
        let err = result.unwrap_err().to_string();
        assert!(err.contains("must be above"), "{err}");

        let (_, result) = load(&CONFIG.replace("measurement_delay: 50", "measurement_delay: 0"));
        let err = result.unwrap_err().to_string();
        assert!(err.contains("must be above"), "{err}");

        // A required option that is absent is refused, as `config.get` does.
        let (_, result) = load(&CONFIG.replace("adc2: PA1\n", ""));
        let err = result.unwrap_err().to_string();
        assert!(err.contains("adc2"), "{err}");

        // Equal calibration counts cannot define a line.
        let (_, result) = load(&format!("{CONFIG}Raw_dia2: 9500\n"));
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("raw_dia1 and raw_dia2 must be different"),
            "{err}"
        );

        // `max_difference` may not swallow the nominal diameter (`:46-49`).
        let (_, result) = load(&format!("{CONFIG}max_difference: 2\n"));
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("max_difference must be less than default_nominal_filament_diameter"),
            "{err}"
        );
    }

    /// Both raw counts are the ADC fraction scaled by 10000 (`:114`, `:119`).
    #[test]
    fn test_the_reports_scale_to_raw_counts() {
        let printer = load_ready(CONFIG);
        let sensor = the_sensor(&printer);
        // 0.5 * 10000 and 0.504 * 10000.
        sensor.handle_adc1_report(&[(0.0, 0.5)]);
        sensor.handle_adc2_report(&[(0.0, 0.504)]);
        let state = sensor.lock(&sensor.state);
        assert_eq!(state.last_reading1, 5000);
        assert_eq!(state.last_reading2, 5040);
        drop(state);

        assert_eq!(
            respond_line(&gcode(&printer), "QUERY_RAW_FILAMENT_WIDTH"),
            "// ADC1=5000 ADC2=5040 RAW=10040"
        );
    }

    /// The fit and the smoothing (`:122-126`): with
    /// `Cal_dia1/Cal_dia2 = 1.5/2.0` and `Raw_dia1/Raw_dia2 = 9500/10500`, the
    /// counts 5000 + 5040 sum to 10040, so the new diameter is
    /// `1.5 + (2.0 - 1.5) / (10500 - 9500) * (10040 - 9500) = 1.77`, and the
    /// first reading smooths to `(5 * 1.75 + 1.77) / 6 = 1.75333…`.
    #[test]
    fn test_the_fit_and_the_smoothing_follow_upstream() {
        let printer = load_ok(&format!("{CONFIG}enable: True\n"));
        let sensor = the_sensor(&printer);
        assert!(sensor.lock(&sensor.state).is_active);

        sensor.handle_adc1_report(&[(0.0, 0.5)]);
        sensor.handle_adc2_report(&[(0.0, 0.504)]);
        let first = (5.0 * 1.75 + 1.77) / 6.0;
        assert_eq!(sensor.lock(&sensor.state).diameter, first);

        // A second report averages on top of the first: 5/6 of the previous
        // diameter plus 1/6 of the newly fitted one.
        sensor.handle_adc2_report(&[(0.0, 0.504)]);
        assert_eq!(
            sensor.lock(&sensor.state).diameter,
            (5.0 * first + 1.77) / 6.0
        );
    }

    /// The fit keeps two places (`:122-125`) and Python's `round()` breaks a tie
    /// towards the even digit: counts 4875 + 4875 = 9750 give
    /// `1.5 + 0.0005 * (9750 - 9500) = 1.625`, which rounds to 1.62 (not 1.63), so
    /// the smoothed first reading is `(5 * 1.75 + 1.62) / 6 = 1.72833…`.
    #[test]
    fn test_the_fit_rounds_to_two_places() {
        let printer = load_ok(&format!("{CONFIG}enable: True\n"));
        let sensor = the_sensor(&printer);
        sensor.handle_adc1_report(&[(0.0, 0.4875)]);
        sensor.handle_adc2_report(&[(0.0, 0.4875)]);
        assert_eq!(
            sensor.lock(&sensor.state).diameter,
            (5.0 * 1.75 + 1.62) / 6.0
        );
    }

    /// An inactive sensor pins the diameter to the nominal value whatever the
    /// inputs report (`:127-128`), while the raw counts are still stored.
    #[test]
    fn test_an_inactive_sensor_reports_the_nominal_diameter() {
        let printer = load_ok(CONFIG);
        let sensor = the_sensor(&printer);
        sensor.handle_adc1_report(&[(0.0, 0.5)]);
        sensor.handle_adc2_report(&[(0.0, 0.504)]);
        let state = sensor.lock(&sensor.state);
        assert_eq!(state.diameter, 1.75);
        assert_eq!(state.last_reading1, 5000);
        assert_eq!(state.last_reading2, 5040);
    }

    /// The runout state is the *diameter* against `min_diameter`..
    /// `max_diameter`, reported through the shared runout helper (`:175-176`).
    #[test]
    fn test_the_diameter_drives_the_runout_state() {
        let printer = load_ok(&format!("{CONFIG}enable: True\n"));
        let sensor = the_sensor(&printer);
        sensor.handle_adc1_report(&[(0.0, 0.5)]);
        sensor.handle_adc2_report(&[(0.0, 0.504)]);
        sensor.update_at(0.0, 0.0);
        assert!(
            sensor.runout_helper.filament_present(),
            "1.75 mm is inside 1.0..1.95"
        );

        // The same counts with `min_diameter: 1.6`: 2500 + 2500 fits to
        // 1.5 + 0.0005 * (5000 - 9500) = -0.75, smoothed to
        // (5 * 1.75 - 0.75) / 6 = 1.333…, below the window.
        let printer = load_ok(&format!(
            "{CONFIG}enable: True\nmin_diameter: 1.6\nmax_diameter: 2.0\n"
        ));
        let sensor = the_sensor(&printer);
        sensor.handle_adc1_report(&[(0.0, 0.25)]);
        sensor.handle_adc2_report(&[(0.0, 0.25)]);
        assert_eq!(
            sensor.lock(&sensor.state).diameter,
            (5.0 * 1.75 - 0.75) / 6.0
        );
        assert!(
            !sensor.runout_helper.filament_present(),
            "not initialised yet"
        );
        sensor.update_at(0.0, 0.0);
        assert!(
            !sensor.runout_helper.filament_present(),
            "1.333 mm is below min_diameter 1.6"
        );
    }

    /// The flow compensation (`:178-200`): the queued width inside
    /// `nominal ± max_difference` sets `M221` to
    /// `round(nominal² / width² * 100)`, outside it back to `M221 S100`.
    #[test]
    fn test_the_queued_width_sets_the_flow_compensation() {
        let printer = load_ok(&format!("{CONFIG}enable: True\nmeasurement_delay: 20\n"));
        let sensor = the_sensor(&printer);
        // The queue holds the smoothed diameter at the time of the measurement;
        // 1.625 mm is inside 1.55..1.95 and gives 1.75² / 1.625² * 100 =
        // 115.976…, which rounds to 116.
        sensor.lock(&sensor.state).diameter = 1.625;

        // First period: the reading is queued 20 mm ahead of the nozzle, which
        // has not reached it yet — the width stays nominal (100 %).
        let first = sensor.update_at(0.0, 0.0);
        assert_eq!(first.scripts, vec!["M221 S100".to_string()]);
        assert_eq!(
            sensor.lock(&sensor.state).filament_array,
            vec![(20.0, 1.625)]
        );

        // The nozzle reaches the queued position: the width takes over.
        let second = sensor.update_at(1.0, 20.0);
        assert_eq!(second.scripts, vec!["M221 S116".to_string()]);
    }

    /// Outside the flow window the percentage goes back to 100 (`:199-200`), and
    /// a diameter at or below `min_diameter` clears the queue as well
    /// (`:201-204`).
    #[test]
    fn test_out_of_window_widths_reset_the_flow_and_a_runout_clears_it() {
        let printer = load_ok(&format!("{CONFIG}enable: True\nmax_difference: 0.05\n"));
        let sensor = the_sensor(&printer);
        // 1.625 mm is outside nominal ± 0.05 (1.70 .. 1.80), and the queue says
        // it is already at the nozzle.
        {
            let mut state = sensor.lock(&sensor.state);
            state.diameter = 1.625;
            state.filament_array = vec![(0.0, 1.625)];
        }
        assert_eq!(
            sensor.update_at(1.0, 0.0).scripts,
            vec!["M221 S100".to_string()]
        );

        // A diameter below `min_diameter` is a runout: the queue is dropped and
        // the flow reset.
        let printer = load_ok(&format!("{CONFIG}enable: True\nmin_diameter: 1.7\n"));
        let sensor = the_sensor(&printer);
        sensor.handle_adc1_report(&[(0.0, 0.5)]);
        sensor.handle_adc2_report(&[(0.0, 0.504)]);
        sensor.update_at(0.0, 0.0);
        assert_eq!(sensor.lock(&sensor.state).filament_array.len(), 1);
        // 0.1 + 0.1 raw counts fit far below `min_diameter`.
        sensor.handle_adc1_report(&[(0.0, 0.1)]);
        sensor.handle_adc2_report(&[(0.0, 0.1)]);
        let period = sensor.update_at(1.0, 10.0);
        assert_eq!(period.scripts, vec!["M221 S100".to_string()]);
        assert!(sensor.lock(&sensor.state).filament_array.is_empty());
    }

    /// `use_current_dia_while_delay` uses the live diameter, not the nominal one,
    /// until the first queued reading reaches the nozzle (`:187-192`).
    #[test]
    fn test_use_current_dia_while_delay() {
        let printer = load_ok(&format!(
            "{CONFIG}enable: True\nuse_current_dia_while_delay: True\n"
        ));
        let sensor = the_sensor(&printer);
        sensor.lock(&sensor.state).diameter = 1.625;
        // The first period queues the reading 50 mm ahead and, because it is the
        // first queued position, the live diameter takes effect at once.
        let period = sensor.update_at(0.0, 0.0);
        assert_eq!(period.scripts, vec!["M221 S116".to_string()]);
        assert_eq!(sensor.lock(&sensor.state).filament_width, 1.625);
    }

    /// `QUERY_FILAMENT_WIDTH` reports the width, the runout state and the two
    /// switches (`:213-231`).
    #[test]
    fn test_the_query_command_reports_status() {
        let printer = load_ready(&format!("{CONFIG}enable: True\n"));
        let sensor = the_sensor(&printer);
        sensor.lock(&sensor.state).diameter = 1.753;

        assert_eq!(
            respond_line(&gcode(&printer), "QUERY_FILAMENT_WIDTH"),
            "// Filament diameter: 1.753 mm\n\
             // Filament state: not present\n\
             // Width sensor: ON\n\
             // Flow compensation: ON"
        );
    }

    /// Disabled, the same command says so in three of the four lines (`:215-230`).
    #[test]
    fn test_the_query_command_says_disabled() {
        let printer = load_ready(CONFIG);
        assert_eq!(
            respond_line(&gcode(&printer), "QUERY_FILAMENT_WIDTH"),
            "// Filament diameter: sensor disabled\n\
             // Filament state: sensor disabled\n\
             // Width sensor: OFF\n\
             // Flow compensation: ON"
        );
    }

    /// `ENABLE`/`DISABLE_FILAMENT_WIDTH_SENSOR` move `is_active`, the runout
    /// helper's flag and the status together (`:245-278`).
    #[test]
    fn test_enable_and_disable_move_the_flags_and_the_status() {
        let printer = load_ready(CONFIG);
        let sensor = the_sensor(&printer);
        let gcode = gcode(&printer);

        assert_eq!(
            respond_line(&gcode, "ENABLE_FILAMENT_WIDTH_SENSOR"),
            "// Filament width sensor: ON\n// Flow compensation: ON"
        );
        assert!(sensor.lock(&sensor.state).is_active);
        assert!(sensor.runout_helper.sensor_enabled());
        let status = PrinterObject::get_status(&*sensor, 0.0);
        assert_eq!(status["is_active"], true);
        assert_eq!(status["enabled"], true);
        assert_eq!(status["flow_compensation_enabled"], true);

        // A running sensor is disabled on the next period, so `M221 S100` is
        // sent before the readings stop.
        assert_eq!(
            respond_line(&gcode, "DISABLE_FILAMENT_WIDTH_SENSOR"),
            "// Filament width sensor: OFF"
        );
        assert!(
            sensor.lock(&sensor.state).is_active,
            "the disable is deferred"
        );
        let period = sensor.update_at(2.0, 10.0);
        assert_eq!(period.scripts, vec!["M221 S100".to_string()]);
        assert_eq!(period.next, None, "the timer retires");
        assert!(!sensor.lock(&sensor.state).is_active);
        assert!(!sensor.runout_helper.sensor_enabled());
        assert!(sensor.runout_helper.filament_present());
        let status = PrinterObject::get_status(&*sensor, 0.0);
        assert_eq!(status["is_active"], false);
        assert_eq!(status["enabled"], false);
    }

    /// `ENABLE ... FLOW_COMPENSATION=0` turns compensation off, `=1` turns it
    /// back on, and a value outside 0..1 is refused with the bound's wording
    /// (`:246`).
    #[test]
    fn test_the_flow_compensation_parameter() {
        let printer = load_ready(&format!("{CONFIG}enable: True\n"));
        let sensor = the_sensor(&printer);
        let gcode = gcode(&printer);

        respond_line(&gcode, "ENABLE_FILAMENT_WIDTH_SENSOR FLOW_COMPENSATION=0");
        assert!(!sensor.lock(&sensor.state).flow_compensation);
        // The disable is deferred like a reset: the next period sends the
        // `M221 S100` that goes with it (`:248-253`, `:152-156`), and the one
        // after has nothing to send at all.
        sensor.handle_adc1_report(&[(0.0, 0.5)]);
        sensor.handle_adc2_report(&[(0.0, 0.504)]);
        assert_eq!(
            sensor.update_at(0.0, 0.0).scripts,
            vec!["M221 S100".to_string()]
        );
        assert!(sensor.update_at(1.0, 5.0).scripts.is_empty());

        respond_line(&gcode, "ENABLE_FILAMENT_WIDTH_SENSOR FLOW_COMPENSATION=1");
        assert!(sensor.lock(&sensor.state).flow_compensation);
        assert_eq!(
            sensor.update_at(1.0, 10.0).scripts,
            vec!["M221 S100".to_string()]
        );

        let err = gcode
            .run_script_sync("ENABLE_FILAMENT_WIDTH_SENSOR FLOW_COMPENSATION=2")
            .unwrap_err()
            .to_string();
        assert!(err.contains("must have maximum of 1"), "{err}");
    }

    /// `RESET_FILAMENT_WIDTH_SENSOR` clears the queue at once; the `M221 S100`
    /// that goes with it waits for the next period while the sensor runs
    /// (`:234-240`, `:157-159`).
    #[test]
    fn test_reset_clears_the_queue() {
        let printer = load_ready(&format!("{CONFIG}enable: True\n"));
        let sensor = the_sensor(&printer);
        sensor.handle_adc1_report(&[(0.0, 0.5)]);
        sensor.handle_adc2_report(&[(0.0, 0.504)]);
        sensor.update_at(0.0, 0.0);
        assert_eq!(sensor.lock(&sensor.state).filament_array.len(), 1);

        assert_eq!(
            respond_line(&gcode(&printer), "RESET_FILAMENT_WIDTH_SENSOR"),
            "// Filament width measurements cleared!"
        );
        assert!(sensor.lock(&sensor.state).filament_array.is_empty());
        // Two `M221 S100` lines, as upstream sends them: one for the pending
        // clear (`:157-159`) and one from the flow compensation of the period
        // that just queued the next reading (`:193-200`).
        assert_eq!(
            sensor.update_at(1.0, 10.0).scripts,
            vec!["M221 S100".to_string(), "M221 S100".to_string()]
        );
    }

    /// `ENABLE`/`DISABLE_FILAMENT_WIDTH_LOG` gate the per-measurement line, which
    /// is upstream's `Filament width:%.3f` (`:140-142`).
    #[test]
    fn test_the_log_toggle_gates_the_measurement_line() {
        let printer = load_ready(&format!(
            "{CONFIG}enable: True\nlogging: True\nmeasurement_delay: 1\nmeasurement_interval: 1\n"
        ));
        let sensor = the_sensor(&printer);
        sensor.handle_adc1_report(&[(0.0, 0.5)]);
        sensor.handle_adc2_report(&[(0.0, 0.504)]);
        sensor.update_at(0.0, 0.0);
        let diameter = sensor.lock(&sensor.state).diameter;
        assert_eq!(
            sensor.update_at(1.0, 5.0).log_lines,
            vec![format!("Filament width:{diameter:.3}")]
        );

        let gcode = gcode(&printer);
        assert_eq!(
            respond_line(&gcode, "DISABLE_FILAMENT_WIDTH_LOG"),
            "// Filament width logging: OFF"
        );
        assert!(!sensor.lock(&sensor.state).is_log);
        assert!(sensor.update_at(2.0, 10.0).log_lines.is_empty());
        assert_eq!(
            respond_line(&gcode, "ENABLE_FILAMENT_WIDTH_LOG"),
            "// Filament width logging: ON"
        );
        assert!(sensor.lock(&sensor.state).is_log);
    }

    /// How many timer callbacks one second of clock runs after `klippy:ready`
    /// for `text`.
    ///
    /// The always-loaded modules (`idle_timeout`, `printer_stats`) schedule their
    /// own timers, so a caller subtracts the count of the same config without this
    /// sensor rather than asserting an absolute number.
    fn callbacks_in_first_second(text: &str) -> usize {
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
        let (config, _) = Config::from_text(text).expect("the config parses");
        printer.load_config(&config).expect("the config loads");
        printer.send_event(&KlippyEvent::KlippyReady);
        reactor.advance(1.0)
    }

    /// The timer is armed at `klippy:ready` and runs at `NOW`; a disabled sensor
    /// then retires it, while an enabled one keeps one-second periods
    /// (`:109`, `:206-209`).
    #[test]
    fn test_ready_arms_the_timer_and_the_period_follows_is_active() {
        let baseline = callbacks_in_first_second("[mcu]\nserial: /dev/not-opened-yet\n");
        let retired = callbacks_in_first_second(CONFIG);
        assert_eq!(
            retired,
            baseline + 1,
            "one period at NOW, then the timer retires"
        );
        let running = callbacks_in_first_second(&format!("{CONFIG}enable: True\n"));
        assert_eq!(running, baseline + 2, "the periods at NOW and at 1.0");
    }

    /// A period with no toolhead to read measures nothing and keeps the
    /// schedule (module docs).
    #[test]
    fn test_a_period_without_a_toolhead_keeps_the_schedule() {
        let printer = load_ok(&format!("{CONFIG}enable: True\n"));
        let sensor = the_sensor(&printer);
        assert_eq!(sensor.extrude_factor_update_event(7.0), Some(8.0));
        assert!(sensor.lock(&sensor.state).filament_array.is_empty());

        // With the sensor disabled the same call retires the timer.
        let printer = load_ok(CONFIG);
        let sensor = the_sensor(&printer);
        assert_eq!(sensor.extrude_factor_update_event(7.0), None);
    }
}
