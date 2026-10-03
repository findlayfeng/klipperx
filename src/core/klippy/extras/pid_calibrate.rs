//! `PID_CALIBRATE` — tune a heater's PID constants.
//!
//! Upstream's `klippy/extras/pid_calibrate.py`. It is loaded the way upstream
//! loads it: by name, from the first `Heater.__init__`
//! (`heaters.py:64-65`), which here is
//! [`PrinterHeaters::setup_heater`] calling [`ensure`] — so the
//! command exists as soon as the first heater does, and a printer with no
//! heater has none, exactly as upstream.
//!
//! `PID_CALIBRATE HEATER=<name> TARGET=<temp> [WRITE_FILE=<0/1>]`
//! (`cmd_PID_CALIBRATE`, `pid_calibrate.py:16-51`):
//!
//! 1. look the heater up (`PrinterHeaters::lookup_heater`) and flush the
//!    toolhead's look-ahead (`toolhead.get_last_move_time()`);
//! 2. swap [`ControlAutoTune`] in for the heater's control
//!    (`Heater::set_control`) and ask
//!    [`PrinterHeaters::set_temperature`] for the target **and the
//!    wait** — the wait ends when the autotune has recorded its twelve peaks,
//!    or when the printer shuts down;
//! 3. hand the configured control back (on the error path too), dump the
//!    samples when `WRITE_FILE` is set, and fail with
//!    `pid_calibrate interrupted` if the run did not finish;
//! 4. report `Kp`/`Ki`/`Kd` and stage `control` + the three constants for
//!    `SAVE_CONFIG` (`configfile.set`, four values, `pid_calibrate.py:48-51`).
//!
//! [`ControlAutoTune`] is a bang-bang that never settles: heating it runs at
//! full power and records the trough, on crossing the target it drops the
//! power, records the peak, and swings the target [`TUNE_PID_DELTA`] below so
//! the next rise starts from a known point (`temperature_update`,
//! `pid_calibrate.py:77-99`). Twelve peaks later `calc_final_pid` estimates
//! the ultimate period and gain (Åström–Hägglund) and derives
//! Ziegler–Nichols constants from them.
//!
//! # What differs from upstream
//!
//! * **No `M105` line during the wait.** Upstream answers the wait loop with
//!   the temperature report every second (`heaters.py:359`); the `M105`
//!   g-code-id table is not wired here yet (`TODO H1`), so there is nothing
//!   to report and the loop only waits — see
//!   [`PrinterHeaters::set_temperature`].
//! * **`write_file` sample times carry no `pwm_delay`.** Upstream dates a PWM
//!   change at `read_time + pwm_delay` (`pid_calibrate.py:74`); this host
//!   applies the output immediately (`Heater::temperature_callback`), so the
//!   sample keeps the reading's own time.
//! * **The wait sleeps on a `tokio` timer** rather than `reactor.pause`.
//! * **An unwritable dump file is a command error** with the path in the
//!   message; upstream would let the `OSError` escape as an internal error.
//! * **No `[pid_calibrate]` section factory.** Upstream reaches the module
//!   only through `load_object` from `Heater.__init__` and no config writes
//!   the section; a hand-written `[pid_calibrate]` is therefore rejected as
//!   an unknown section here, where upstream would read it as empty.

use std::f64::consts::PI;
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};
use tracing::info;

use crate::core::klippy::config::object::{PrinterConfig, CONFIGFILE_OBJECT};
use crate::core::klippy::config::ConfigError;
use crate::core::klippy::extras::heaters::{
    Heater, HeaterControl, PrinterHeaters, HEATERS_OBJECT, PID_PARAM_BASE,
};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// What the object is registered under — upstream's
/// `printer.load_object(config, "pid_calibrate")` (`heaters.py:65`).
const PID_CALIBRATE_OBJECT: &str = "pid_calibrate";

/// The toolhead the command flushes before it starts
/// (`pid_calibrate.py:25`).
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The words `PID_CALIBRATE` reads, in source order (`pid_calibrate.py:17-19`).
const PARAMS: &[&str] = &["HEATER", "TARGET", "WRITE_FILE"];

/// Upstream's `cmd_PID_CALIBRATE_help` (`pid_calibrate.py:15`).
const HELP: &str = "Run PID calibration test";

/// How far below the target the autotune swings (`TUNE_PID_DELTA`,
/// `pid_calibrate.py:53`).
const TUNE_PID_DELTA: f64 = 5.0;

/// How many peaks the autotune records before it is done — upstream's
/// `len(self.peaks) < 12` (`pid_calibrate.py:101`).
const PEAKS_TO_FINISH: usize = 12;

/// Where `WRITE_FILE` dumps the samples (`pid_calibrate.py:35`).
const DEBUG_FILE: &str = "/tmp/heattest.txt";

// ===========================================================================
// The `pid_calibrate` object and its command
// ===========================================================================

/// The `pid_calibrate` printer object: what registers `PID_CALIBRATE`
/// (`PIDCalibrate`, `pid_calibrate.py:9-51`). Upstream builds it from the
/// config section, which it never reads; here it is the printer handle its
/// command needs.
///
/// Like upstream's, it has no status of its own, so it stays out of
/// `objects/list` ([`PrinterObject::is_queryable`]).
pub struct PIDCalibrate {
    /// The printer, as upstream's `self.printer`. `Weak`, because the g-code
    /// table holds this object for as long as the printer lives.
    printer: Weak<Printer>,
}

impl PIDCalibrate {
    /// `cmd_PID_CALIBRATE` (`pid_calibrate.py:16-51`), one run.
    ///
    /// # Errors
    /// A missing parameter, an unknown heater, a target outside the heater's
    /// range, a failed `WRITE_FILE` dump, or `pid_calibrate interrupted` when
    /// the run ended before twelve peaks were recorded.
    async fn calibrate(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let heater_name = gcmd.get_str("HEATER")?;
        let target = gcmd.get_float("TARGET")?;
        let write_file = gcmd.get_int_default("WRITE_FILE", 0)?;

        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| CommandError::new("printer is gone"))?;
        let pheaters = printer
            .lookup_object_as::<PrinterHeaters>(HEATERS_OBJECT)
            .ok_or_else(|| {
                CommandError::new(format!("Unknown config object '{HEATERS_OBJECT}'"))
            })?;
        // Upstream wraps this `config_error` in `gcmd.error`
        // (`pid_calibrate.py:21-23`); a command error is this host's spelling.
        let heater = pheaters
            .lookup_heater(&heater_name)
            .map_err(|err| CommandError::new(err.to_string()))?;
        let toolhead = printer
            .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)
            .ok_or_else(|| {
                CommandError::new(format!("Unknown config object '{TOOLHEAD_OBJECT}'"))
            })?;
        toolhead.get_last_move_time();

        let calibrate = Arc::new(Mutex::new(ControlAutoTune::new(&heater, target)));
        let old_control = heater.set_control(Box::new(AutoTuneControl(Arc::clone(&calibrate))));
        // Upstream puts the old control back on the error path too
        // (`pid_calibrate.py:27-33`), so it is swapped back *before* the
        // result is looked at either way.
        let result = pheaters.set_temperature(&heater, target, true).await;
        heater.set_control(old_control);
        result?;

        let (kp, ki, kd) = {
            let tune = calibrate.lock().unwrap_or_else(|p| p.into_inner());
            if write_file != 0 {
                tune.write_file(DEBUG_FILE)?;
            }
            // Upstream: `if calibrate.check_busy(0., 0., 0.): raise
            // gcmd.error("pid_calibrate interrupted")` (`pid_calibrate.py:36-37`).
            // Its `eventtime` argument has no counterpart in this host's
            // `check_busy`, hence the two zeroes.
            if tune.check_busy(0.0, 0.0) {
                return Err(CommandError::new("pid_calibrate interrupted"));
            }
            tune.calc_final_pid()
        };
        info!("Autotune: final: Kp={kp:.6} Ki={ki:.6} Kd={kd:.6}");
        gcmd.respond_info(&format!(
            "PID parameters: pid_Kp={kp:.3} pid_Ki={ki:.3} pid_Kd={kd:.3}\n\
             The SAVE_CONFIG command will update the printer config file\n\
             with these parameters and restart the printer."
        ));

        // Store the result for SAVE_CONFIG (`configfile.set`, four values,
        // `pid_calibrate.py:48-51`).
        let configfile = printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .ok_or_else(|| {
                CommandError::new(format!("Unknown config object '{CONFIGFILE_OBJECT}'"))
            })?;
        let cfgname = heater.section_name();
        configfile.set(cfgname, "control", "pid");
        configfile.set(cfgname, "pid_Kp", &format!("{kp:.3}"));
        configfile.set(cfgname, "pid_Ki", &format!("{ki:.3}"));
        configfile.set(cfgname, "pid_Kd", &format!("{kd:.3}"));
        Ok(())
    }
}

impl PrinterObject for PIDCalibrate {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    /// Upstream's object has no `get_status`, so `objects/list` skips it.
    fn is_queryable(&self) -> bool {
        false
    }
}

/// Build the object and register `PID_CALIBRATE`, once — upstream's
/// `printer.load_object(config, "pid_calibrate")`, which
/// `Heater.__init__` calls for every heater and which answers an already
/// loaded module with the object it holds (`klippy/klippy.py:90-113`).
///
/// # Errors
/// The object or the command name is already taken (a wiring mistake), or
/// `gcode` has not been registered yet.
pub fn ensure(printer: &Arc<Printer>) -> Result<(), ConfigError> {
    if printer.lookup_object(PID_CALIBRATE_OBJECT).is_some() {
        return Ok(());
    }
    let object = Arc::new(PIDCalibrate {
        printer: Arc::downgrade(printer),
    });
    printer.add_object(
        PID_CALIBRATE_OBJECT,
        Arc::clone(&object) as Arc<dyn PrinterObject>,
    )?;
    let handler: CommandHandler = {
        let object = Arc::clone(&object);
        Arc::new(move |gcmd| {
            let object = Arc::clone(&object);
            Box::pin(async move { object.calibrate(gcmd).await })
        })
    };
    let gcode = printer
        .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
        .ok_or_else(|| ConfigError::new("Unknown config object 'gcode'"))?;
    gcode
        .register_command_with_params("PID_CALIBRATE", handler, Some(HELP), PARAMS, false)
        .map_err(ConfigError::new)?;
    Ok(())
}

// ===========================================================================
// ControlAutoTune
// ===========================================================================

/// The autotune control algorithm (`ControlAutoTune`, `pid_calibrate.py:55-142`).
///
/// Upstream keeps the heater beside the algorithm (it calls back into
/// `heater.set_pwm` / `heater.alter_target`); here the PWM value is the
/// return value of [`HeaterControl::update`] and the target it swings is the
/// `&mut f64` that update is handed, so the control never re-enters the
/// heater's lock. The heater's range and power ceiling are read once in
/// [`ControlAutoTune::new`].
pub struct ControlAutoTune {
    /// `heater.get_max_power()` at construction (`pid_calibrate.py:58`) — the
    /// full power the heating half runs at, and the ceiling `calc_pid`
    /// estimates the gain against.
    heater_max_power: f64,
    /// The configured range, what upstream's `alter_target` clamps against
    /// (`heaters.py:133-136`).
    min_temp: f64,
    max_temp: f64,
    /// The `TARGET` the run was started with (`calibrate_temp`).
    calibrate_temp: f64,
    /// Whether the heater is on — the heating half of the swing.
    heating: bool,
    /// The running extremum of the current half (`peak`) and when it was read
    /// (`peak_time`).
    peak: f64,
    peak_time: f64,
    /// The recorded extrema as `(temperature, time)`: upstream's first entry
    /// is the still-untouched `(0., 0.)` pushed at the first crossing, then
    /// trough, peak, trough, peak… (`check_peaks`, `pid_calibrate.py:105-113`).
    peaks: Vec<(f64, f64)>,
    /// The last PWM value handed on, to record only the changes.
    last_pwm: f64,
    /// The PWM changes as `(time, value)` (`set_pwm`, `pid_calibrate.py:71-76`).
    pwm_samples: Vec<(f64, f64)>,
    /// Every reading as `(time, temperature)` (`pid_calibrate.py:78`).
    temp_samples: Vec<(f64, f64)>,
}

impl ControlAutoTune {
    /// Start a run for `target` (`ControlAutoTune.__init__`,
    /// `pid_calibrate.py:56-69`).
    pub fn new(heater: &Heater, target: f64) -> Self {
        let (min_temp, max_temp) = heater.temperature_range();
        Self {
            heater_max_power: heater.max_power(),
            min_temp,
            max_temp,
            calibrate_temp: target,
            heating: false,
            peak: 0.0,
            peak_time: 0.0,
            peaks: Vec::new(),
            last_pwm: 0.0,
            pwm_samples: Vec::new(),
            temp_samples: Vec::new(),
        }
    }

    /// One reading: record it, run the two-phase swing, and return the PWM
    /// value to apply (`temperature_update`, `pid_calibrate.py:77-99`).
    ///
    /// `target` is the heater's current target, read before the swing so a
    /// phase change takes effect on the next reading — the order upstream
    /// has between its `target_temp` argument and `alter_target`.
    pub fn temperature_update(&mut self, read_time: f64, temp: f64, target: &mut f64) -> f64 {
        self.temp_samples.push((read_time, temp));
        // Check if the temperature has crossed the target and
        // enable/disable the heater if so.
        if self.heating && temp >= *target {
            self.heating = false;
            self.check_peaks();
            *target = self.alter_target(self.calibrate_temp - TUNE_PID_DELTA);
        } else if !self.heating && temp <= *target {
            self.heating = true;
            self.check_peaks();
            *target = self.alter_target(self.calibrate_temp);
        }
        // Check if this temperature is a peak and record it if so.
        if self.heating {
            let value = self.set_pwm(read_time, self.heater_max_power);
            if temp < self.peak {
                self.peak = temp;
                self.peak_time = read_time;
            }
            value
        } else {
            let value = self.set_pwm(read_time, 0.0);
            if temp > self.peak {
                self.peak = temp;
                self.peak_time = read_time;
            }
            value
        }
    }

    /// Whether the run is over: twelve peaks recorded and the last half has
    /// crossed back (`check_busy`, `pid_calibrate.py:100-103`). The
    /// temperature arguments are what the trait passes and, as upstream's
    /// `eventtime`/`smoothed_temp`/`target_temp`, they decide nothing here.
    pub fn check_busy(&self, _smoothed_temp: f64, _target: f64) -> bool {
        self.heating || self.peaks.len() < PEAKS_TO_FINISH
    }

    /// Hand the value on and let the heater apply it (`set_pwm`,
    /// `pid_calibrate.py:71-76`), recording a change.
    ///
    /// Upstream also dates the sample `read_time + pwm_delay`; this host
    /// applies the output immediately (see the module docs), so the sample
    /// keeps `read_time`.
    fn set_pwm(&mut self, read_time: f64, value: f64) -> f64 {
        if value != self.last_pwm {
            self.pwm_samples.push((read_time, value));
            self.last_pwm = value;
        }
        value
    }

    /// Clamp a new target into the configured range (`Heater.alter_target`,
    /// `heaters.py:133-136`: a falsy target is stored as it stands).
    fn alter_target(&self, target_temp: f64) -> f64 {
        if target_temp == 0.0 {
            return target_temp;
        }
        target_temp.clamp(self.min_temp, self.max_temp)
    }

    /// Record the extremum of the half that just ended and re-estimate
    /// (`check_peaks`, `pid_calibrate.py:105-113`).
    fn check_peaks(&mut self) {
        self.peaks.push((self.peak, self.peak_time));
        if self.heating {
            self.peak = 9999999.;
        } else {
            self.peak = -9999999.;
        }
        if self.peaks.len() < 4 {
            return;
        }
        let _ = self.calc_pid(self.peaks.len() - 1);
    }

    /// Estimate `Ku`/`Tu` from one peak and derive the PID constants
    /// (`calc_pid`, `pid_calibrate.py:114-129`).
    ///
    /// `pos` is the index of a peak: `temp_diff` is peak against the trough
    /// before it, `time_diff` the same phase one cycle back. Returns
    /// `(Kp, Ki, Kd)`.
    fn calc_pid(&self, pos: usize) -> (f64, f64, f64) {
        let temp_diff = self.peaks[pos].0 - self.peaks[pos - 1].0;
        let time_diff = self.peaks[pos].1 - self.peaks[pos - 2].1;
        // Use Astrom-Hagglund method to estimate Ku and Tu
        let amplitude = 0.5 * temp_diff.abs();
        let ku = 4.0 * self.heater_max_power / (PI * amplitude);
        let tu = time_diff;
        // Use Ziegler-Nichols method to generate PID parameters
        let ti = 0.5 * tu;
        let td = 0.125 * tu;
        let kp = 0.6 * ku * PID_PARAM_BASE;
        let ki = kp / ti;
        let kd = kp * td;
        info!(
            "Autotune: raw={temp_diff:.6}/{:.6} Ku={ku:.6} Tu={tu:.6}  Kp={kp:.6} Ki={ki:.6} Kd={kd:.6}",
            self.heater_max_power
        );
        (kp, ki, kd)
    }

    /// The constants to store: `calc_pid` over the middle of the recorded
    /// cycles (`calc_final_pid`, `pid_calibrate.py:130-134`).
    ///
    /// # Panics
    /// Fewer than five recorded peaks — unreachable through the command,
    /// which asks [`Self::check_busy`] first (upstream would `IndexError`
    /// there too).
    pub fn calc_final_pid(&self) -> (f64, f64, f64) {
        let mut cycle_times: Vec<(f64, usize)> = (4..self.peaks.len())
            .map(|pos| (self.peaks[pos].1 - self.peaks[pos - 2].1, pos))
            .collect();
        // Python's `sorted` over `(time, pos)` tuples breaks ties by index.
        cycle_times.sort_by(|left, right| {
            left.0
                .partial_cmp(&right.0)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(left.1.cmp(&right.1))
        });
        let midpoint_pos = cycle_times[cycle_times.len() / 2].1;
        self.calc_pid(midpoint_pos)
    }

    /// Dump the samples for offline analysis (`write_file`,
    /// `pid_calibrate.py:136-142`): one `pwm: <time> <value>` line per PWM
    /// change, then one `<time> <temperature>` line per reading, joined with
    /// newlines and with no trailing one.
    ///
    /// # Errors
    /// The file cannot be written (see the module docs for the difference
    /// from upstream).
    pub fn write_file(&self, filename: &str) -> Result<(), CommandError> {
        let mut lines: Vec<String> = self
            .pwm_samples
            .iter()
            .map(|(time, value)| format!("pwm: {time:.3} {value:.3}"))
            .collect();
        lines.extend(
            self.temp_samples
                .iter()
                .map(|(time, temp)| format!("{time:.3} {temp:.3}")),
        );
        std::fs::write(filename, lines.join("\n"))
            .map_err(|err| CommandError::new(format!("Unable to write {filename}: {err}")))
    }
}

/// [`ControlAutoTune`] as the heater runs it.
///
/// The heater owns the control it was given, while the command needs the
/// algorithm back to read the result — upstream has one object in both
/// places, so here the two share one [`ControlAutoTune`] behind a
/// [`Mutex`] (the command takes it only while no reading is being handled,
/// so the two never block each other).
struct AutoTuneControl(Arc<Mutex<ControlAutoTune>>);

impl HeaterControl for AutoTuneControl {
    fn update(&mut self, read_time: f64, temp: f64, target: &mut f64, _max_power: f64) -> f64 {
        // `_max_power` is the heater's ceiling, which the algorithm already
        // read in `ControlAutoTune::new` (`heater.get_max_power()`,
        // `pid_calibrate.py:58`).
        self.0
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .temperature_update(read_time, temp, target)
    }

    fn check_busy(&self, smoothed_temp: f64, target: f64) -> bool {
        self.0
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .check_busy(smoothed_temp, target)
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, ConfigSection, ConfigValue, ConfigWrapper};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::extras::heaters::{self, Sensor, SensorCallback};
    use crate::core::klippy::mcu::McuError;
    use crate::core::klippy::pins::{
        DigitalOut, PinChip, PinError, PinParams, PrinterPins, PwmOut, PINS_OBJECT,
    };
    use crate::core::klippy::reactor::ManualReactor;

    /// A sensor that accepts everything; the tests drive readings through the
    /// heater itself.
    #[derive(Debug)]
    struct FakeSensor;

    impl Sensor for FakeSensor {
        fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) {}
        fn setup_callback(&self, _callback: SensorCallback) {}
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

    /// A chip that only exists so `heater_pin` resolves.
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

    /// A section with the given options, as the parser would build it.
    fn section(id: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new(id, None);
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A printer with `gcode` (already ready), `pins`, `configfile`, a
    /// `toolhead` (`kinematics: none`), the `heaters` registry with a `Fake`
    /// sensor — and one `[extruder]` heater, which is what registers
    /// `PID_CALIBRATE` (`heaters.py:64-65`).
    fn machine() -> (Arc<Printer>, Arc<GCodeDispatch>, Arc<Heater>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
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
            .add_object(
                CONFIGFILE_OBJECT,
                Arc::new(PrinterConfig::new(
                    AccessTracking::shared(),
                    serde_json::Map::new(),
                )),
            )
            .unwrap();
        // The toolhead the command flushes first; `kinematics: none` builds
        // on its own (`toolhead.rs`, `test_none_kinematics_needs_no_steppers`).
        let toolhead_section = section(
            "printer",
            &[
                ("kinematics", "none"),
                ("max_velocity", "300"),
                ("max_accel", "3000"),
            ],
        );
        let toolhead = ToolHeadObject::new(&ConfigWrapper::untracked(&toolhead_section), &printer)
            .expect("kinematics: none builds");
        printer
            .add_object(TOOLHEAD_OBJECT, Arc::new(toolhead))
            .unwrap();

        let heaters = heaters::ensure(&printer).expect("the registry loads");
        heaters.add_sensor_factory(
            "Fake",
            Arc::new(|_config, _printer| Ok(Arc::new(FakeSensor) as Arc<dyn Sensor>)),
        );
        // The dispatcher only offers non-built-in commands once the printer is
        // ready (`gcode.rs`, `Commands::active`).
        printer.send_event(&KlippyEvent::KlippyReady);

        let heater = heaters
            .setup_heater(
                &ConfigWrapper::untracked(&section(
                    "extruder",
                    &[
                        ("sensor_type", "Fake"),
                        ("heater_pin", "PA0"),
                        ("min_temp", "0"),
                        ("max_temp", "250"),
                        ("control", "watermark"),
                        ("max_delta", "2"),
                    ],
                )),
                &printer,
                None,
            )
            .expect("the heater is set up");
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        (printer, gcode, heater)
    }

    /// Every line the dispatcher emitted, in order.
    #[derive(Clone, Default)]
    struct Lines(Arc<Mutex<Vec<String>>>);

    impl Lines {
        fn capture(gcode: &Arc<GCodeDispatch>) -> Self {
            let lines = Self::default();
            let sink = lines.clone();
            gcode.register_output_handler(Arc::new(move |line: &str| {
                sink.0
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push(line.to_string());
            }));
            lines
        }

        fn emitted(&self) -> Vec<String> {
            self.0.lock().unwrap_or_else(|p| p.into_inner()).clone()
        }
    }

    /// The command exists as soon as a heater does, with the three parameters
    /// upstream reads — and the object carries no status, as upstream's does
    /// not (`pid_calibrate.py:9-19`).
    #[test]
    fn test_the_command_is_registered_with_its_parameters() {
        let (printer, gcode, _heater) = machine();

        assert!(gcode.command_exists("PID_CALIBRATE"));
        let command = &gcode.get_status(0.0)["commands"]["PID_CALIBRATE"];
        assert_eq!(command["help"], HELP);
        assert_eq!(
            command["parameters"],
            json!(["HEATER", "TARGET", "WRITE_FILE"])
        );

        let object = printer
            .lookup_object(PID_CALIBRATE_OBJECT)
            .expect("the object is registered");
        assert!(!object.is_queryable());
        assert_eq!(object.get_status(0.0), json!({}));
    }

    /// `HEATER` is required (`pid_calibrate.py:17`).
    #[test]
    fn test_a_missing_heater_parameter_is_refused() {
        let (_printer, gcode, _heater) = machine();

        let err = gcode
            .run_script_sync("PID_CALIBRATE TARGET=100")
            .unwrap_err();
        assert!(err.to_string().contains("missing HEATER"), "{err}");
    }

    /// An unknown heater is reported as a command error, upstream's wording
    /// (`pid_calibrate.py:21-23` over `heaters.py:289-292`).
    #[test]
    fn test_an_unknown_heater_is_reported_by_name() {
        let (_printer, gcode, _heater) = machine();

        let err = gcode
            .run_script_sync("PID_CALIBRATE HEATER=nope TARGET=100")
            .unwrap_err();
        assert_eq!(err.to_string(), "Unknown heater 'nope'");
    }

    /// `TARGET` above `max_temp` fails in `heater.set_temp`, with upstream's
    /// message (`heaters.py:109-113`) — and the configured control is back
    /// even though the run failed (`pid_calibrate.py:27-32`).
    #[test]
    fn test_a_target_out_of_the_heaters_range_is_refused() {
        let (_printer, gcode, heater) = machine();

        let err = gcode
            .run_script_sync("PID_CALIBRATE HEATER=extruder TARGET=300")
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Requested temperature (300.0) out of range (0.0:250.0)"
        );
        // `set_control` zeroes the target on the way out (`heaters.py:127-132`),
        // so nothing is left heating.
        assert_eq!(heater.get_status()["target"], 0.0);
    }

    /// A run that ends before twelve peaks reports the interruption and runs
    /// the configured control again (`pid_calibrate.py:36-37`).
    ///
    /// `TARGET=0` is upstream's shortest such run: no wait, no peaks.
    #[test]
    fn test_an_interrupted_run_reports_and_restores_the_control() {
        let (_printer, gcode, heater) = machine();

        let err = gcode
            .run_script_sync("PID_CALIBRATE HEATER=extruder TARGET=0")
            .unwrap_err();
        assert_eq!(err.to_string(), "pid_calibrate interrupted");

        // The watermark control is running again: settled at 200 °C against a
        // 100 °C target the heater is not busy, while the autotune — which
        // recorded no peak at all — would still call itself busy.
        heater.set_temp(100.0).unwrap();
        heater.temperature_callback(10.0, 200.0);
        assert!(!heater.check_busy(100.0));
        assert_eq!(heater.get_status()["target"], 100.0);
    }

    /// `WRITE_FILE` dumps the samples *before* the interruption check
    /// (`pid_calibrate.py:34-37`), so even a run that ends at once writes
    /// upstream's fixed path.
    #[test]
    fn test_write_file_dumps_before_the_interruption_is_reported() {
        let (_printer, gcode, _heater) = machine();
        let _ = std::fs::remove_file(DEBUG_FILE);

        let err = gcode
            .run_script_sync("PID_CALIBRATE HEATER=extruder TARGET=0 WRITE_FILE=1")
            .unwrap_err();
        assert_eq!(err.to_string(), "pid_calibrate interrupted");

        // No reading was ever delivered, so the dump has no line to hold.
        let dump = std::fs::read_to_string(DEBUG_FILE).expect("the dump was written");
        assert_eq!(dump, "");
        let _ = std::fs::remove_file(DEBUG_FILE);
    }

    /// The two-phase swing: heat at full power to the target, then cool with
    /// [`TUNE_PID_DELTA`] below it and back (`temperature_update`,
    /// `pid_calibrate.py:77-99`).
    #[test]
    fn test_the_autotune_swings_the_target_by_tune_pid_delta() {
        let (_printer, _gcode, heater) = machine();
        let mut tune = ControlAutoTune::new(&heater, 200.0);
        let mut target = 200.0;

        // The first reading flips into the heating half and records upstream's
        // untouched first peak `(0., 0.)`.
        assert_eq!(tune.temperature_update(0.0, 20.0, &mut target), 1.0);
        assert_eq!(target, 200.0);
        assert_eq!(tune.peaks, [(0.0, 0.0)]);

        // Crossing the target drops the power and swings the target down.
        assert_eq!(tune.temperature_update(1.0, 200.0, &mut target), 0.0);
        assert_eq!(target, 195.0);

        // Falling through it puts the target back and the power on.
        assert_eq!(tune.temperature_update(2.0, 195.0, &mut target), 1.0);
        assert_eq!(target, 200.0);
        // Each of the three readings crossed the target once, so three peaks
        // are on the list (the first of them upstream's untouched `(0., 0.)`).
        assert_eq!(tune.peaks.len(), 3);
        assert!(tune.check_busy(0.0, 0.0));
    }

    /// `calc_pid` is Åström–Hägglund plus Ziegler–Nichols, and
    /// `calc_final_pid` picks the middle of the recorded cycles
    /// (`pid_calibrate.py:114-134`).
    #[test]
    fn test_the_autotune_math_follows_ziegler_nichols() {
        let (_printer, _gcode, heater) = machine();
        let mut tune = ControlAutoTune::new(&heater, 200.0);
        assert_eq!(tune.heater_max_power, 1.0);
        // A recorded run: upstream's untouched first entry, then trough/peak
        // pairs with a 30 °C swing and a 15 s cycle.
        tune.peaks = vec![
            (0.0, 0.0),
            (180.0, 10.0),
            (210.0, 15.0),
            (180.0, 25.0),
            (210.0, 30.0),
            (180.0, 40.0),
            (210.0, 45.0),
            (180.0, 55.0),
            (210.0, 60.0),
        ];

        let (kp, ki, kd) = tune.calc_final_pid();
        // amplitude 15 °C → Ku = 4·max_power / (π·15), Tu = 15 s, and the
        // Ziegler-Nichols divisors Ti = Tu/2, Td = Tu/8.
        let ku = 4.0 / (PI * 15.0);
        assert!((kp - 0.6 * ku * PID_PARAM_BASE).abs() < 1e-12, "{kp}");
        assert!((ki - kp / 7.5).abs() < 1e-12, "{ki}");
        assert!((kd - kp * 1.875).abs() < 1e-12, "{kd}");
        // The five candidate cycles all tie at 15 s, so the middle index (2 of
        // 0..5) is peak 6 — the constants below are that peak's.
        assert!((kp - 12.98704335536732).abs() < 1e-9, "{kp}");
    }

    /// The run stays busy until twelve peaks are recorded and the last half
    /// has crossed back (`pid_calibrate.py:100-103`).
    #[test]
    fn test_the_run_is_busy_until_twelve_peaks_are_recorded() {
        let (_printer, _gcode, heater) = machine();
        let mut tune = ControlAutoTune::new(&heater, 200.0);
        assert!(tune.check_busy(0.0, 0.0));

        tune.peaks = vec![(200.0, 1.0); PEAKS_TO_FINISH];
        tune.heating = false;
        assert!(!tune.check_busy(0.0, 0.0));
        tune.heating = true;
        assert!(tune.check_busy(0.0, 0.0));
    }

    /// A deterministic thermal model driven reading by reading: the autotune
    /// runs both halves, collects its peaks and finishes with finite, positive
    /// constants.
    #[test]
    fn test_the_autotune_converges_on_a_fake_heater() {
        let (_printer, _gcode, heater) = machine();
        let tune = Arc::new(Mutex::new(ControlAutoTune::new(&heater, 200.0)));
        let old_control = heater.set_control(Box::new(AutoTuneControl(Arc::clone(&tune))));
        heater.set_temp(200.0).unwrap();

        // 120 °C/s of heat against a 0.5/s loss to a 20 °C ambient, sampled
        // every 100 ms: a limit cycle around the target with no randomness in
        // it at all.
        let mut temp = 20.0;
        let mut time = 0.0;
        let mut powers = Vec::new();
        for _ in 0..100_000 {
            time += 0.1;
            let power = heater.get_status()["power"].as_f64().unwrap();
            temp += (power * 120.0 - (temp - 20.0) * 0.5) * 0.1;
            heater.temperature_callback(time, temp);
            powers.push(heater.get_status()["power"].as_f64().unwrap());
            if tune
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .check_busy(0.0, 0.0)
            {
                continue;
            }
            break;
        }

        let (kp, ki, kd) = {
            let tune = tune.lock().unwrap_or_else(|p| p.into_inner());
            assert!(
                !tune.check_busy(0.0, 0.0),
                "the run never reached its twelve peaks"
            );
            assert!(tune.peaks.len() >= PEAKS_TO_FINISH, "{}", tune.peaks.len());
            // Both halves ran: full power while heating, off while cooling.
            assert!(powers.contains(&1.0) && powers.contains(&0.0), "{powers:?}");
            tune.calc_final_pid()
        };
        for (name, value) in [("Kp", kp), ("Ki", ki), ("Kd", kd)] {
            assert!(value.is_finite() && value > 0.0, "{name}={value}");
        }

        heater.set_control(old_control);
    }

    /// The dump lists every PWM change, then every reading, joined with
    /// newlines and with no trailing one (`write_file`,
    /// `pid_calibrate.py:136-142`).
    #[test]
    fn test_the_dump_lists_the_pwm_changes_then_the_readings() {
        let (_printer, _gcode, heater) = machine();
        let mut tune = ControlAutoTune::new(&heater, 200.0);
        let mut target = 200.0;
        tune.temperature_update(0.0, 20.0, &mut target);
        tune.temperature_update(1.0, 200.0, &mut target);

        let unique = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path = std::env::temp_dir().join(format!("klipperx_heattest_{unique}.txt"));
        let name = path.to_str().expect("the path is utf-8");
        tune.write_file(name).expect("the dump is written");
        let dump = std::fs::read_to_string(&path).expect("the dump is readable");
        let _ = std::fs::remove_file(&path);

        assert_eq!(
            dump,
            "pwm: 0.000 1.000\npwm: 1.000 0.000\n0.000 20.000\n1.000 200.000"
        );
    }

    /// The whole command: wait through a (simulated) calibration, report the
    /// constants, stage them for `SAVE_CONFIG`, and leave the heater off with
    /// its own control back (`pid_calibrate.py:33-51`).
    ///
    /// The clock is paused, so the wait loop's second sleeps and the feeder's
    /// tenth-of-a-second readings advance in simulated time.
    #[tokio::test(start_paused = true)]
    async fn test_a_completed_run_reports_the_pid_and_stages_it() {
        let (printer, gcode, heater) = machine();
        let lines = Lines::capture(&gcode);

        // Feed the heater the same deterministic model the convergence test
        // uses, while the command waits.
        let feed = Arc::clone(&heater);
        let feeder = tokio::spawn(async move {
            let mut temp = 20.0;
            let mut time = 0.0;
            loop {
                let power = feed.get_status()["power"].as_f64().unwrap();
                time += 0.1;
                temp += (power * 120.0 - (temp - 20.0) * 0.5) * 0.1;
                feed.temperature_callback(time, temp);
                tokio::time::sleep(std::time::Duration::from_millis(100)).await;
            }
        });

        let run = gcode.run_script("PID_CALIBRATE HEATER=extruder TARGET=200");
        tokio::time::timeout(std::time::Duration::from_secs(3600), run)
            .await
            .expect("the calibration finished within a simulated hour")
            .expect("PID_CALIBRATE succeeded");
        feeder.abort();

        // The four values are staged for SAVE_CONFIG.
        let configfile = printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .expect("configfile is registered");
        let pending = configfile.get_status(0.0)["save_config_pending_items"]["extruder"].clone();
        assert_eq!(pending["control"], json!("pid"));
        let mut constants = Vec::new();
        for option in ["pid_Kp", "pid_Ki", "pid_Kd"] {
            let value = pending[option]
                .as_str()
                .unwrap_or_else(|| panic!("{option} is a string: {pending}"));
            assert_eq!(value.split('.').nth(1).map(str::len), Some(3), "{option}");
            assert!(value.parse::<f64>().unwrap() > 0.0, "{option}={value}");
            constants.push(value.to_string());
        }

        // …and the same numbers are reported. `respond_info` hands the whole
        // three-line message to the output as one entry, with every line
        // after the first prefixed (`gcode.rs`, `Inner::respond_info`).
        // The `set_temperature(.., wait)` run also echoes an `M105` line each
        // second it waits (`T:0` here — `has_started` is false in this test
        // so the gcode-id table reports the empty default), matching
        // upstream's `_wait_for_temperature` (`heaters.py:349-360`).
        let expected_pid = format!(
            "// PID parameters: pid_Kp={} pid_Ki={} pid_Kd={}\n// The SAVE_CONFIG command will update the printer config file\n// with these parameters and restart the printer.",
            constants[0], constants[1], constants[2]
        );
        let emitted = lines.emitted();
        assert_eq!(emitted.last(), Some(&expected_pid), "the PID line is last");
        assert!(
            emitted[..emitted.len() - 1].iter().all(|l| l == "T:0"),
            "every earlier line is the empty-table M105 report"
        );

        // The run ends with the heater off and its own control back.
        assert_eq!(heater.get_status()["target"], 0.0);
    }
}
