//! `[tsl1401cl_filament_width_sensor]` — filament width from a TSL1401CL
//! linear CCD (upstream `klippy/extras/tsl1401cl_filament_width_sensor.py`).
//!
//! One analog input (`pin`) reads the CCD's own width output; the reading is
//! already a millimetre figure — `round(read_value * 5, 2)` (`:64`) — so there is
//! no calibration line to fit, unlike
//! [`hall_filament_width_sensor`](super::hall_filament_width_sensor). Every
//! `MEASUREMENT_INTERVAL_MM` (10) mm of extrusion the reading is queued
//! `measurement_delay` mm ahead of the nozzle (`:66-79`), and the reading that
//! reaches the nozzle decides the flow compensation (`M221`) the way upstream's
//! `extrude_factor_update_event` does (`:81-110`).
//!
//! # The timer
//!
//! Upstream registers the timer in `__init__`, arms it at `klippy:ready`
//! (`:40-41`, `:58-59`), and stops it from `DISABLE_FILAMENT_WIDTH_SENSOR`
//! (`:145-146`). Here it is created at ready and cancelled by that command. One
//! period is [`FilamentWidthSensor::update_at`], which the tests step directly
//! instead of waiting on a clock.
//!
//! A timer callback here is synchronous and `GCodeDispatch::run_script` is not,
//! so the `M221` calls are run detached — the shape `delayed_gcode` uses.
//!
//! # What is not here
//!
//! Upstream's object has no `get_status`, so it is not client-visible
//! ([`PrinterObject::is_queryable`] is `false`). `toolhead.get_position()[3]`
//! (`:83-84`) is [`ToolHeadObject::position`], which answers `None` before the
//! planner exists (a config without `[printer]`, or before connect); such a
//! period is skipped and the schedule kept.

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{py_float_str, ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{sync, CommandError, GCodeDispatch, GcodeCommand, GCODE_OBJECT};
use crate::core::klippy::load::section;
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};
use crate::core::klippy::reactor::TimerHandle;

// The bare section only: upstream has `load_config`, no prefix form
// (`tsl1401cl_filament_width_sensor.py:153`).
section!(
    "tsl1401cl_filament_width_sensor",
    order = 30,
    load = load_config
);

/// The input's sampling (`tsl1401cl_filament_width_sensor.py:7-9`); the rest of
/// `setup_adc_sample`'s arguments keep upstream's defaults (`minval=0`,
/// `maxval=1`, `range_check_count=0`, `batch_num=1`).
const ADC_REPORT_TIME: f64 = 0.500;
const ADC_SAMPLE_TIME: f64 = 0.001;
const ADC_SAMPLE_COUNT: u32 = 8;

/// Millimetres of extrusion between readings (`:10`). A module constant
/// upstream, not an option — unlike the Hall sensor's `measurement_interval`.
const MEASUREMENT_INTERVAL_MM: f64 = 10.0;

/// The CCD's own scale: its report is a `0.0..=1.0` fraction of 5 mm
/// (`:64`).
const READING_SCALE: f64 = 5.0;

/// The reading above which filament counts as present (`:88`).
const FILAMENT_PRESENT_MM: f64 = 0.5;

/// The `toolhead` object upstream reads the extruder position from (`:83`).
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The name the timer reports in a latency warning.
const TIMER_NAME: &str = "tsl1401cl_filament_width_sensor";

/// The options, read once and never written again
/// (`tsl1401cl_filament_width_sensor.py:16-25`).
struct WidthConfig {
    /// `default_nominal_filament_diameter`.
    nominal_dia: f64,
    /// `measurement_delay`: millimetres between the sensor and the nozzle.
    measurement_delay: f64,
    /// The flow-compensation window, `nominal ± max_difference` (`:22-25`).
    min_dia: f64,
    max_dia: f64,
}

/// The sensor state upstream keeps in plain attributes (`:26-29`).
struct WidthState {
    /// `is_active`: whether the timer runs.
    is_active: bool,
    /// `filament_array`: `(extruder position, width)` pairs, oldest first.
    filament_array: Vec<(f64, f64)>,
    /// `lastFilamentWidthReading`, in millimetres.
    last_reading: f64,
}

/// One period's decisions: the scripts upstream hands to `gcode.run_script` and
/// the next wake time (`None` retires the timer).
struct UpdateOutcome {
    scripts: Vec<String>,
    next: Option<f64>,
}

/// One `[tsl1401cl_filament_width_sensor]`.
pub struct FilamentWidthSensor {
    /// The machine, for its reactor and the toolhead; weak so holding the object
    /// does not keep the machine alive.
    printer: Weak<Printer>,
    /// The dispatcher the `M221` calls and the commands go through.
    gcode: Arc<GCodeDispatch>,
    config: WidthConfig,
    state: Mutex<WidthState>,
    /// The measurement timer; created at ready, cancelled by
    /// `DISABLE_FILAMENT_WIDTH_SENSOR`.
    timer: Mutex<Option<TimerHandle>>,
    /// The handle the timer callback upgrades.
    self_ref: Weak<FilamentWidthSensor>,
}

impl FilamentWidthSensor {
    /// Read the section and build the analog input
    /// (`tsl1401cl_filament_width_sensor.py:16-38`).
    ///
    /// # Errors
    /// A missing/invalid option, or a pin the pin layer refuses.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let pin = config.get("pin", None)?;
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
        let measurement_max_difference =
            config.get_float_bounded("max_difference", None, None, None, Some(0.0), None)?;

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .ok_or_else(|| ConfigError::new("pin: the pins object is not registered"))?;
        let adc = pins
            .setup_adc(&pin, None)
            .map_err(|err| ConfigError::new(format!("pin: {err}")))?;
        adc.setup_adc_sample(
            ADC_REPORT_TIME,
            ADC_SAMPLE_TIME,
            ADC_SAMPLE_COUNT,
            1,
            0.0,
            1.0,
            0,
        );

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .ok_or_else(|| {
                ConfigError::new("the gcode dispatcher is not registered".to_string())
            })?;

        let object = Arc::new_cyclic(|weak: &Weak<FilamentWidthSensor>| {
            let for_adc = weak.clone();
            adc.setup_adc_callback(Box::new(move |samples| {
                if let Some(sensor) = for_adc.upgrade() {
                    sensor.handle_adc_report(samples);
                }
            }));
            Self {
                printer: Arc::downgrade(printer),
                gcode,
                config: WidthConfig {
                    nominal_dia,
                    measurement_delay,
                    min_dia: nominal_dia - measurement_max_difference,
                    max_dia: nominal_dia + measurement_max_difference,
                },
                state: Mutex::new(WidthState {
                    is_active: true,
                    filament_array: Vec::new(),
                    last_reading: 0.0,
                }),
                timer: Mutex::new(None),
                self_ref: weak.clone(),
            }
        });
        Ok(object)
    }

    /// Register the four commands and the `klippy:ready` handler
    /// (`tsl1401cl_filament_width_sensor.py:42-50`).
    ///
    /// Upstream passes no `desc=` for any of them, so neither does this.
    ///
    /// # Errors
    /// A command name already taken.
    fn attach(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        self.register_command("QUERY_FILAMENT_WIDTH", Self::cmd_query_filament_width)?;
        self.register_command(
            "RESET_FILAMENT_WIDTH_SENSOR",
            Self::cmd_clear_filament_array,
        )?;
        self.register_command("DISABLE_FILAMENT_WIDTH_SENSOR", Self::cmd_disable)?;
        self.register_command("ENABLE_FILAMENT_WIDTH_SENSOR", Self::cmd_enable)?;

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
        handler: fn(&Self, &GcodeCommand) -> Result<(), CommandError>,
    ) -> Result<(), ConfigError> {
        let weak = Arc::downgrade(self);
        self.gcode
            .register_command(
                name,
                sync(move |gcmd: &GcodeCommand| {
                    let sensor = weak.upgrade().ok_or_else(|| {
                        CommandError::new("the tsl1401cl filament width sensor is gone")
                    })?;
                    handler(&sensor, gcmd)
                }),
                None,
                false,
            )
            .map_err(ConfigError::new)
    }

    /// Upstream's `adc_callback` (`:61-64`): the CCD's fraction of 5 mm,
    /// rounded to two places.
    fn handle_adc_report(&self, samples: &[(f64, f64)]) {
        let Some((_read_time, read_value)) = samples.last() else {
            return;
        };
        self.lock(&self.state).last_reading = py_round_digits(read_value * READING_SCALE, 2);
    }

    /// The reading in millimetres (the corpus-free getter the tests and
    /// `QUERY_FILAMENT_WIDTH` share).
    pub fn last_reading(&self) -> f64 {
        self.lock(&self.state).last_reading
    }

    /// Whether the timer runs (`is_active`).
    pub fn is_active(&self) -> bool {
        self.lock(&self.state).is_active
    }

    /// The queued readings, oldest first.
    #[cfg(test)]
    fn filament_array(&self) -> Vec<(f64, f64)> {
        self.lock(&self.state).filament_array.clone()
    }

    /// Start the measurement timer at the current instant: upstream's
    /// `reactor.update_timer(self.extrude_factor_update_timer, reactor.NOW)`
    /// (`:58-59`, `:134-135`).
    ///
    /// This host's reactor has no `update_timer` (`delayed_gcode` documents the
    /// same gap), so the previous handle is cancelled and a fresh timer takes its
    /// place.
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

    /// Stop the measurement timer: upstream's
    /// `reactor.update_timer(self.extrude_factor_update_timer, reactor.NEVER)`
    /// (`:145-146`).
    fn stop(&self) {
        let handle = self.lock(&self.timer).take();
        if let Some(handle) = handle {
            handle.cancel();
        }
    }

    /// One period (`:81-110`), with the extruder position read from the
    /// toolhead.
    fn extrude_factor_update_event(&self, eventtime: f64) -> Option<f64> {
        let Some(last_epos) = self.toolhead_e_position() else {
            // No toolhead to read (module docs): skip the period and keep the
            // schedule.
            return self.next_wake(eventtime);
        };
        let outcome = self.update_at(eventtime, last_epos);
        for script in &outcome.scripts {
            self.run_script(script);
        }
        outcome.next
    }

    /// The wake time the current `is_active` implies (`:107-110`).
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
    /// is the only production caller that reads it (`:83-84`).
    fn update_at(&self, eventtime: f64, last_epos: f64) -> UpdateOutcome {
        let config = &self.config;
        let mut state = self.lock(&self.state);
        let mut scripts = Vec::new();

        self.update_filament_array(&mut state, last_epos);

        if state.last_reading > FILAMENT_PRESENT_MM {
            if let Some(&(pending_position, _)) = state.filament_array.first() {
                if pending_position <= last_epos {
                    let (_position, filament_width) = state.filament_array.remove(0);
                    if filament_width <= config.max_dia && filament_width >= config.min_dia {
                        let percentage = py_round_int(
                            config.nominal_dia.powi(2) / filament_width.powi(2) * 100.0,
                        );
                        scripts.push(format!("M221 S{percentage}"));
                    } else {
                        scripts.push("M221 S100".to_string());
                    }
                }
            }
        } else {
            scripts.push("M221 S100".to_string());
            state.filament_array.clear();
        }

        let next = if state.is_active {
            Some(eventtime + 1.0)
        } else {
            None
        };
        UpdateOutcome { scripts, next }
    }

    /// Upstream's `update_filament_array` (`:66-79`): queue the last reading every
    /// `MEASUREMENT_INTERVAL_MM`, `measurement_delay` mm ahead of the nozzle.
    fn update_filament_array(&self, state: &mut WidthState, last_epos: f64) {
        let config = &self.config;
        match state.filament_array.last() {
            Some(&(last_position, _)) => {
                if last_position + MEASUREMENT_INTERVAL_MM <= last_epos + config.measurement_delay {
                    state
                        .filament_array
                        .push((last_epos + config.measurement_delay, state.last_reading));
                }
            }
            None => state
                .filament_array
                .push((config.measurement_delay + last_epos, state.last_reading)),
        }
    }

    /// The extruder position upstream reads as `toolhead.get_position()[3]`
    /// (`:83-84`), or `None` before the planner exists.
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
                "tsl1401cl_filament_width_sensor: running '{script}' needs an async runtime"
            ),
        }
    }

    /// Upstream's `cmd_M407` (`:112-119`).
    fn cmd_query_filament_width(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let reading = self.last_reading();
        let response = if reading > 0.0 {
            format!("Filament dia (measured mm): {}", py_float_str(reading))
        } else {
            "Filament NOT present".to_string()
        };
        gcmd.respond_info(&response);
        Ok(())
    }

    /// Upstream's `cmd_ClearFilamentArray` (`:121-125`).
    fn cmd_clear_filament_array(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        self.lock(&self.state).filament_array.clear();
        gcmd.respond_info("Filament width measurements cleared!");
        self.run_script("M221 S100");
        Ok(())
    }

    /// Upstream's `cmd_M405` (`:127-136`).
    fn cmd_enable(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let already = {
            let mut state = self.lock(&self.state);
            let already = state.is_active;
            state.is_active = true;
            already
        };
        if !already {
            self.arm();
        }
        let response = if already {
            "Filament width sensor is already On"
        } else {
            "Filament width sensor Turned On"
        };
        gcmd.respond_info(response);
        Ok(())
    }

    /// Upstream's `cmd_M406` (`:138-151`).
    fn cmd_disable(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let was_active = {
            let mut state = self.lock(&self.state);
            let was_active = state.is_active;
            state.is_active = false;
            state.filament_array.clear();
            was_active
        };
        let response = if was_active {
            self.stop();
            self.run_script("M221 S100");
            "Filament width sensor Turned Off"
        } else {
            "Filament width sensor is already Off"
        };
        gcmd.respond_info(response);
        Ok(())
    }

    fn lock<'a, T>(&self, slot: &'a Mutex<T>) -> MutexGuard<'a, T> {
        slot.lock().unwrap_or_else(|poison| poison.into_inner())
    }
}

impl PrinterObject for FilamentWidthSensor {
    /// Upstream's object defines no `get_status`; this is the data it would
    /// report.
    fn get_status(&self, _eventtime: f64) -> Value {
        let state = self.lock(&self.state);
        json!({
            "filament_width": state.last_reading,
            "is_active": state.is_active,
        })
    }

    /// Not client-visible, matching upstream's `get_status`-less object.
    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for FilamentWidthSensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let state = self.lock(&self.state);
        f.debug_struct("FilamentWidthSensor")
            .field("last_reading", &state.last_reading)
            .field("is_active", &state.is_active)
            .field("filament_array", &state.filament_array.len())
            .finish_non_exhaustive()
    }
}

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

/// The factory the section declaration names
/// (`tsl1401cl_filament_width_sensor.py:153 def load_config`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let sensor = FilamentWidthSensor::new(config, printer)?;
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
         [tsl1401cl_filament_width_sensor]\npin: PA0\n\
         default_nominal_filament_diameter: 1.75\nmeasurement_delay: 50\n\
         max_difference: 0.2\n";

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

    fn the_sensor(printer: &Arc<Printer>) -> Arc<FilamentWidthSensor> {
        printer
            .lookup_object_as::<FilamentWidthSensor>("tsl1401cl_filament_width_sensor")
            .expect("[tsl1401cl_filament_width_sensor] is registered")
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

    /// The options read as written, and the two windows come from
    /// `nominal ± max_difference` (`:17-25`).
    #[test]
    fn test_the_section_reads_its_options() {
        let printer = load_ok(CONFIG);
        let sensor = the_sensor(&printer);
        assert_eq!(sensor.config.nominal_dia, 1.75);
        assert_eq!(sensor.config.measurement_delay, 50.0);
        assert_eq!(sensor.config.min_dia, 1.55);
        assert_eq!(sensor.config.max_dia, 1.95);
        // The sensor is active from the moment it exists (`:26`).
        assert!(sensor.is_active());
        assert_eq!(sensor.last_reading(), 0.0);
        assert!(sensor.filament_array().is_empty());
    }

    /// The bounds upstream puts on the three numeric options are enforced with
    /// its wording (`:17-21`), and a missing `pin` is refused (`:16`).
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

        // `max_difference` is `above=0.`, so 0 itself is refused.
        let (_, result) = load(&CONFIG.replace("max_difference: 0.2", "max_difference: 0"));
        let err = result.unwrap_err().to_string();
        assert!(err.contains("must be above"), "{err}");

        let (_, result) = load(&CONFIG.replace("pin: PA0\n", ""));
        let err = result.unwrap_err().to_string();
        assert!(err.contains("pin"), "{err}");
    }

    /// The reading is the CCD's fraction of 5 mm, two places (`:64`):
    /// `0.35 * 5 = 1.75`, and `0.325 * 5 = 1.625` rounds to 1.62 — Python's
    /// `round()` breaks the tie towards the even digit.
    #[test]
    fn test_the_report_scales_to_millimetres() {
        let printer = load_ok(CONFIG);
        let sensor = the_sensor(&printer);
        sensor.handle_adc_report(&[(0.0, 0.35)]);
        assert_eq!(sensor.last_reading(), 1.75);
        sensor.handle_adc_report(&[(0.0, 0.325)]);
        assert_eq!(sensor.last_reading(), 1.62);
        // The last sample of a batch is the one read (`:63`).
        sensor.handle_adc_report(&[(0.0, 0.1), (0.5, 0.2)]);
        assert_eq!(sensor.last_reading(), 1.0);
    }

    /// `QUERY_FILAMENT_WIDTH` reports the measured width, or says there is no
    /// filament (`:112-119`).
    #[test]
    fn test_the_query_command_reports_the_width() {
        let printer = load_ready(CONFIG);
        let sensor = the_sensor(&printer);
        let gcode = gcode(&printer);

        assert_eq!(
            respond_line(&gcode, "QUERY_FILAMENT_WIDTH"),
            "// Filament NOT present"
        );

        sensor.handle_adc_report(&[(0.0, 0.35)]);
        assert_eq!(
            respond_line(&gcode, "QUERY_FILAMENT_WIDTH"),
            "// Filament dia (measured mm): 1.75"
        );

        // A whole number keeps Python's `1.0` spelling (`str(1.0)`).
        sensor.handle_adc_report(&[(0.0, 0.2)]);
        assert_eq!(
            respond_line(&gcode, "QUERY_FILAMENT_WIDTH"),
            "// Filament dia (measured mm): 1.0"
        );
    }

    /// The queued reading sets the flow compensation once the nozzle reaches it:
    /// `round(nominal² / width² * 100)` inside the window, `M221 S100`
    /// outside (`:88-102`).
    #[test]
    fn test_the_queued_reading_sets_the_flow_compensation() {
        let printer = load_ok(CONFIG);
        let sensor = the_sensor(&printer);
        sensor.handle_adc_report(&[(0.0, 0.325)]);
        assert_eq!(sensor.last_reading(), 1.62);

        // First period: the reading is queued 50 mm ahead, so the queue holds it
        // and nothing reaches the nozzle yet.
        let first = sensor.update_at(0.0, 0.0);
        assert!(first.scripts.is_empty());
        assert_eq!(sensor.filament_array(), vec![(50.0, 1.62)]);

        // At the queued position 1.62 mm is inside 1.55..1.95 and would ask for
        // 1.75² / 1.62² * 100 = 116.69…, which rounds to 117.
        let second = sensor.update_at(1.0, 50.0);
        assert_eq!(second.scripts, vec!["M221 S117".to_string()]);
        assert_eq!(second.next, Some(2.0));
    }

    /// Outside the window the flow goes back to 100 % (`:101-102`), and a
    /// reading at or below 0.5 mm clears the queue (`:103-105`).
    #[test]
    fn test_out_of_window_readings_reset_the_flow() {
        let printer = load_ok(&CONFIG.replace("max_difference: 0.2", "max_difference: 0.05"));
        let sensor = the_sensor(&printer);
        sensor.handle_adc_report(&[(0.0, 0.325)]);
        // 1.62 mm is outside nominal ± 0.05 (1.70 .. 1.80).
        {
            let mut state = sensor.lock(&sensor.state);
            state.filament_array = vec![(0.0, 1.62)];
        }
        assert_eq!(
            sensor.update_at(1.0, 0.0).scripts,
            vec!["M221 S100".to_string()]
        );

        // A reading of 0.5 mm is not filament: `lastFilamentWidthReading > 0.5`
        // fails at equality.
        let printer = load_ok(CONFIG);
        let sensor = the_sensor(&printer);
        sensor.handle_adc_report(&[(0.0, 0.1)]);
        assert_eq!(sensor.last_reading(), 0.5);
        {
            let mut state = sensor.lock(&sensor.state);
            state.filament_array = vec![(0.0, 1.75)];
        }
        let period = sensor.update_at(1.0, 5.0);
        assert_eq!(period.scripts, vec!["M221 S100".to_string()]);
        assert!(sensor.filament_array().is_empty());
    }

    /// `RESET_FILAMENT_WIDTH_SENSOR` clears the queue and resets the factor
    /// (`:121-125`).
    #[test]
    fn test_reset_clears_the_queue() {
        let printer = load_ready(CONFIG);
        let sensor = the_sensor(&printer);
        sensor.handle_adc_report(&[(0.0, 0.35)]);
        sensor.update_at(0.0, 0.0);
        assert_eq!(sensor.filament_array().len(), 1);

        assert_eq!(
            respond_line(&gcode(&printer), "RESET_FILAMENT_WIDTH_SENSOR"),
            "// Filament width measurements cleared!"
        );
        assert!(sensor.filament_array().is_empty());
    }

    /// `ENABLE`/`DISABLE_FILAMENT_WIDTH_SENSOR` say whether they changed
    /// anything, and the disable drops the queue and retires the timer
    /// (`:127-151`).
    #[test]
    fn test_enable_and_disable() {
        let printer = load_ready(CONFIG);
        let sensor = the_sensor(&printer);
        let gcode = gcode(&printer);

        // Active from load, so the first enable reports "already On".
        assert_eq!(
            respond_line(&gcode, "ENABLE_FILAMENT_WIDTH_SENSOR"),
            "// Filament width sensor is already On"
        );
        assert!(sensor.is_active());

        assert_eq!(
            respond_line(&gcode, "DISABLE_FILAMENT_WIDTH_SENSOR"),
            "// Filament width sensor Turned Off"
        );
        assert!(!sensor.is_active());
        assert!(
            sensor.filament_array().is_empty(),
            "the disable drops the queue"
        );
        // The reading still updates: the ADC callback is upstream's
        // unconditional one (`:61-64`).
        sensor.handle_adc_report(&[(0.0, 0.35)]);
        assert_eq!(sensor.last_reading(), 1.75);
        // The timer is cancelled, and a period that did run would retire.
        assert_eq!(sensor.extrude_factor_update_event(2.0), None);

        assert_eq!(
            respond_line(&gcode, "ENABLE_FILAMENT_WIDTH_SENSOR"),
            "// Filament width sensor Turned On"
        );
        assert!(sensor.is_active());
        assert_eq!(
            respond_line(&gcode, "DISABLE_FILAMENT_WIDTH_SENSOR"),
            "// Filament width sensor Turned Off"
        );
        assert_eq!(
            respond_line(&gcode, "DISABLE_FILAMENT_WIDTH_SENSOR"),
            "// Filament width sensor is already Off"
        );
    }

    /// The timer is armed at `klippy:ready` and keeps one-second periods until
    /// a disable retires it (`:58-59`, `:107-110`).
    ///
    /// The always-loaded modules (`idle_timeout`, `printer_stats`) schedule their
    /// own timers, so the count is measured against the same config without this
    /// sensor.
    #[test]
    fn test_ready_arms_the_timer_until_a_disable_retires_it() {
        let callbacks = |text: &str| {
            let reactor = Arc::new(ManualReactor::new());
            let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
            let (config, _) = Config::from_text(text).expect("the config parses");
            printer.load_config(&config).expect("the config loads");
            printer.send_event(&KlippyEvent::KlippyReady);
            reactor.advance(1.0)
        };
        let baseline = callbacks("[mcu]\nserial: /dev/not-opened-yet\n");
        // The periods at `NOW` and at 1.0.
        assert_eq!(callbacks(CONFIG), baseline + 2);

        // Disabled before the clock moves: only the period at `NOW` runs, and
        // the timer is cancelled.
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
        let (config, _) = Config::from_text(CONFIG).expect("the config parses");
        printer.load_config(&config).expect("the config loads");
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = gcode(&printer);
        gcode
            .run_script_sync("DISABLE_FILAMENT_WIDTH_SENSOR")
            .expect("the command runs");
        assert_eq!(reactor.advance(1.0), baseline);
    }
}
