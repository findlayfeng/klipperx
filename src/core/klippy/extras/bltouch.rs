//! `[bltouch]` — the BLTouch probe: its single-wire protocol, its virtual Z
//! endstop, and the `probe` object it registers.
//!
//! Upstream is `klippy/extras/bltouch.py`. Three pieces land here:
//!
//! | piece | upstream | here |
//! |---|---|---|
//! | the protocol | `BLTouchProbe` | [`BlTouchProtocol`] + [`BlTouchTiming`]: `control_pin` PWM pulses whose **duty is the command's width** over [`SIGNAL_PERIOD`], the upstream [`COMMANDS`] table unchanged |
//! | the probe interface | `PrinterBLTouch` | [`PrinterBLTouch`], which registers `BLTOUCH_DEBUG` / `BLTOUCH_STORE` and, like upstream's `load_config`, adds the section's probe under the object name `probe` |
//! | probing | `probe.py`'s helpers reused (`ProbeOffsetsHelper`, `ProbeParameterHelper`, `SampleAveragingHelper`, `ProbeCommandHelper`, `HomingViaProbeHelper`) | the port's existing [`probe`] pieces: [`ProbeOptions`]/[`ProbeSessionHelper`]/[`ProbeOffsets`] wired with this section's values, the shared [`ProbeChip`] registered as `probe`, and `probe`'s own command registration reused rather than repeated |
//!
//! The `probe` object upstream registers *is* `PrinterBLTouch`
//! (`bltouch.py:load_config` adds `probe` → the section's object). Here the
//! object under `probe` is a [`PrinterProbe`] assembled from this section's
//! parts ([`PrinterProbe::from_parts`]), because that is the type the probe
//! consumers (`ProbePointsHelper`, `bed_mesh`) look the `probe` object up as;
//! [`PrinterBLTouch`] sits next to it under `bltouch` and reports the same
//! status fields.
//!
//! # What differs from upstream, and why
//!
//! * **`G28 Z` does not lower the pin.** Upstream's homing routes a
//!   `probe:z_virtual_endstop` rail through the probe session
//!   (`homing.py:_do_home_z_via_probe`), so the pin is down before the
//!   descend. This port's rail arms the chip's endstop directly — the same
//!   path `[probe]` uses — so a BLTouch pin that was raised at connect stays
//!   up during `G28 Z`. Closing that needs a homing-layer change (route rail
//!   homing through the `probe` object), reported rather than made here.
//! * **The connect-time sensor check short-circuits on a file-output run.**
//!   Upstream gets this from `MCU_endstop`: `home_wait` answers
//!   `home_end_time` and `query_endstop` answers `0` when the MCU writes to a
//!   file (`mcu.py:396-404`) — which is exactly how the corpus runs klippy.
//!   This port's fake MCU *does* answer queries but never fires an endstop
//!   armed outside a move, so the same short-circuit is taken in
//!   [`verify_state`] instead of in the MCU layer.
//! * **The control pin's MCU is looked up by name** (the pin description's
//!   `chip:` prefix); upstream asks the PWM resource (`mcu_pwm.get_mcu()`),
//!   which this port's [`PwmOut`] does not carry yet.

use std::sync::{Arc, Mutex, Weak};

use serde_json::Value;
use tracing::{info, warn};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::probe::{
    command_status, register_commands, PrinterProbe, ProbeChip, ProbeCommandState, ProbeHooks,
    ProbeOffsets, ProbeOptions, ProbeSessionHelper,
};
use crate::core::klippy::extras::toolhead::{EndstopFuture, HomingEndstop, ToolHeadObject};
use crate::core::klippy::gcode::{sync, CommandError, GCodeDispatch, GcodeCommand, GCODE_OBJECT};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{Completion, McuEndstop, McuError, McuObject};
use crate::core::klippy::pins::{PrinterPins, PwmOut, PINS_OBJECT};
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};

section!("bltouch", order = 30, load = load_config);

/// The toolhead object, as the loader registers `[printer]`.
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The single-wire signal period, in seconds (`SIGNAL_PERIOD`).
///
/// Every command is a pulse whose **width** is the entry in [`COMMANDS`]; on
/// the wire that width is the duty cycle over this period.
const SIGNAL_PERIOD: f64 = 0.020;

/// The shortest a command may take, in seconds (`MIN_CMD_TIME`): five signal
/// periods, the gap the BLTouch needs between commands.
const MIN_CMD_TIME: f64 = 5.0 * SIGNAL_PERIOD;

/// How often the sensor self-test may run, in seconds (`TEST_TIME`).
const TEST_TIME: f64 = 5.0 * 60.0;

/// The reset pulse length when retrying, in seconds (`RETRY_RESET_TIME`).
const RETRY_RESET_TIME: f64 = 1.0;

/// The poll interval the endstop check is clamped to (`ENDSTOP_REST_TIME`).
const ENDSTOP_REST_TIME: f64 = 0.001;

/// The endstop sample geometry the sensor checks use
/// (`ENDSTOP_SAMPLE_TIME` / `ENDSTOP_SAMPLE_COUNT`).
const ENDSTOP_SAMPLE_TIME: f64 = 0.000_015;
const ENDSTOP_SAMPLE_COUNT: u8 = 4;

/// How long past the pulse a sensor check waits for the endstop
/// (`_verify_state`'s `home_wait(self.action_end_time + 0.100)`).
const VERIFY_WINDOW: f64 = 0.100;

/// Upstream's `Commands`: the command name and its pulse width in seconds.
/// Key order is upstream's dict order; [`command_names`] is what a user sees.
const COMMANDS: &[(&str, f64)] = &[
    ("pin_down", 0.000650),
    ("touch_mode", 0.001165),
    ("pin_up", 0.001475),
    ("self_test", 0.001780),
    ("reset", 0.002190),
    ("set_5V_output_mode", 0.001988),
    ("set_OD_output_mode", 0.002091),
    ("output_mode_store", 0.001884),
];

/// The pulse width of `cmd` in seconds, or `None` for a name the BLTouch does
/// not know (`cmd_BLTOUCH_DEBUG`'s `cmd not in Commands`).
fn command_width(cmd: &str) -> Option<f64> {
    COMMANDS
        .iter()
        .find(|(name, _)| *name == cmd)
        .map(|(_, width)| *width)
}

/// `cmd` as a duty cycle over [`SIGNAL_PERIOD`]
/// (`_send_cmd`: `Commands[cmd] / SIGNAL_PERIOD`).
fn command_duty(cmd: &str) -> Option<f64> {
    command_width(cmd).map(|width| width / SIGNAL_PERIOD)
}

/// The command list `BLTOUCH_DEBUG` without a `COMMAND=` prints, sorted as
/// upstream's `", ".join(sorted(...))` sorts it.
fn command_names() -> Vec<&'static str> {
    let mut names: Vec<&'static str> = COMMANDS.iter().map(|(name, _)| *name).collect();
    names.sort_unstable();
    names
}

/// The commands `BLTOUCH_STORE MODE=…` sends, in order
/// (`_store_output_mode`): all with the default [`MIN_CMD_TIME`] width.
fn store_output_mode_commands(mode: &str) -> [&'static str; 5] {
    let set_mode = if mode == "5V" {
        "set_5V_output_mode"
    } else {
        "set_OD_output_mode"
    };
    [
        "pin_down",
        set_mode,
        "output_mode_store",
        set_mode,
        "pin_up",
    ]
}

// ===========================================================================
// Options
// ===========================================================================

/// The `[bltouch]` options as written (`bltouch.py`'s `BLTouchProbe.__init__`,
/// `ProbeOffsetsHelper.__init__` and `ProbeParameterHelper.__init__`).
#[derive(Debug, Clone, PartialEq)]
pub struct BlTouchOptions {
    /// The sensor pin: the endstop that reports the trigger.
    pub sensor_pin: String,
    /// The control pin: the single-wire protocol output.
    pub control_pin: String,
    /// Raise the pin between samples (`stow_on_each_sample`).
    pub stow_on_each_sample: bool,
    /// Probe in `touch_mode` rather than after `pin_down`
    /// (`probe_with_touch_mode`).
    pub probe_with_touch_mode: bool,
    /// `5V` or `OD`, when the output mode is set at connect
    /// (`set_output_mode`).
    pub set_output_mode: Option<String>,
    /// Whether `pin_up` reports the pin as not triggered
    /// (`pin_up_reports_not_triggered`).
    pub pin_up_reports_not_triggered: bool,
    /// Whether the sensor test may run (`pin_up_touch_mode_reports_triggered`).
    pub pin_up_touch_mode_reports_triggered: bool,
    /// How long a `pin_up`/`pin_down` pulse lasts, above zero
    /// (`pin_move_time`).
    pub pin_move_time: f64,
    /// Probe-to-nozzle X offset.
    pub x_offset: f64,
    /// Probe-to-nozzle Y offset.
    pub y_offset: f64,
    /// The probe's trigger offset from the nozzle (required).
    pub z_offset: f64,
    /// Probing speed (`speed`, above zero).
    pub speed: f64,
    /// Speed for the retract moves between samples (`lift_speed`, defaults to
    /// `speed`).
    pub lift_speed: Option<f64>,
    /// Samples per probe (`samples`, minimum 1).
    pub samples: i64,
    /// Retract distance between samples (`sample_retract_dist`, above zero).
    pub sample_retract_dist: f64,
    /// `median` or `average` (`samples_result`, defaults to **`average`** —
    /// `ProbeParameterHelper` differs from `[probe]` here).
    pub samples_result: String,
    /// How far the samples may spread (`samples_tolerance`, minimum 0).
    pub samples_tolerance: f64,
    /// How many times a spread sample set is retried
    /// (`samples_tolerance_retries`, minimum 0).
    pub samples_tolerance_retries: i64,
}

impl BlTouchOptions {
    /// Read every option the section accepts, so `check_unused` passes.
    ///
    /// # Errors
    /// As the option readers: a missing `sensor_pin`/`control_pin`/`z_offset`,
    /// an unknown `set_output_mode`, a bad `samples_result`, a value out of
    /// range, and so on.
    pub fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let set_output_mode = if config.has("set_output_mode") {
            Some(config.get_choice("set_output_mode", &["5V", "OD"], None)?)
        } else {
            None
        };
        Ok(Self {
            sensor_pin: config.get("sensor_pin", None)?,
            control_pin: config.get("control_pin", None)?,
            stow_on_each_sample: config.get_bool("stow_on_each_sample", Some(true))?,
            probe_with_touch_mode: config.get_bool("probe_with_touch_mode", Some(false))?,
            set_output_mode,
            pin_up_reports_not_triggered: config
                .get_bool("pin_up_reports_not_triggered", Some(true))?,
            pin_up_touch_mode_reports_triggered: config
                .get_bool("pin_up_touch_mode_reports_triggered", Some(true))?,
            pin_move_time: config.get_float_bounded(
                "pin_move_time",
                Some(0.680),
                None,
                None,
                Some(0.0),
                None,
            )?,
            x_offset: config.get_float("x_offset", Some(0.0))?,
            y_offset: config.get_float("y_offset", Some(0.0))?,
            z_offset: config.get_float("z_offset", None)?,
            speed: config.get_float_bounded("speed", Some(5.0), None, None, Some(0.0), None)?,
            lift_speed: config.get_optional_float("lift_speed")?,
            samples: config.get_int_bounded("samples", Some(1), Some(1), None)?,
            sample_retract_dist: config.get_float_bounded(
                "sample_retract_dist",
                Some(2.0),
                None,
                None,
                Some(0.0),
                None,
            )?,
            samples_result: config.get_choice(
                "samples_result",
                &["median", "average"],
                Some("average"),
            )?,
            samples_tolerance: config.get_float_bounded(
                "samples_tolerance",
                Some(0.100),
                Some(0.0),
                None,
                None,
                None,
            )?,
            samples_tolerance_retries: config.get_int_bounded(
                "samples_tolerance_retries",
                Some(0),
                Some(0),
                None,
            )?,
        })
    }
}

// ===========================================================================
// The protocol
// ===========================================================================

/// Which multi-probe state the pin is in (`BLTouchProbe.multi`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Multi {
    /// Raise after every trigger (the default, `stow_on_each_sample`).
    Off,
    /// The session's first sample: lower the pin once, then stay down.
    First,
    /// Down for the rest of the session.
    On,
}

/// The pieces a scheduled `wait_for_trigger` callback needs without the
/// protocol object (`BLTouchProbe._wait_for_trigger` runs on the reactor, so
/// upstream's callback borrows `self` freely).
///
/// Cloned into the callback and used by every protocol step: the PWM, the MCU
/// clock the pulses are scheduled on, and the command timeline.
#[derive(Clone)]
struct BlTouchTiming {
    /// The machine, for the toolhead, the reactor clock and `respond_info`.
    printer: Weak<Printer>,
    /// The control pin.
    pwm: Arc<dyn PwmOut>,
    /// The MCU that pin lives on, for print-time ↔ clock conversion
    /// (upstream: `self.mcu_pwm.get_mcu()`).
    mcu: Arc<McuObject>,
    /// When the next command may be sent (`_send_cmd`'s `next_cmd_time`).
    next_cmd_time: Arc<Mutex<f64>>,
    /// When the last command's pulse ends (`_send_cmd`'s `action_end_time`).
    action_end_time: Arc<Mutex<f64>>,
    /// The multi-probe state (`BLTouchProbe.multi`).
    multi: Arc<Mutex<Multi>>,
    /// `pin_move_time`.
    pin_move_time: f64,
    /// `pin_up_reports_not_triggered`.
    pin_up_reports_not_triggered: bool,
}

impl BlTouchTiming {
    /// Pull `next_cmd_time` up to this MCU's clock now
    /// (`_sync_mcu_print_time`): no toolhead, so this is the form the
    /// connect-time steps use.
    fn sync_mcu_print_time(&self) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        let now = printer.reactor().monotonic();
        let Some(estimated) = self.mcu.estimated_print_time(now) else {
            return;
        };
        let mut next = self.next_cmd_time.lock().unwrap_or_else(|p| p.into_inner());
        *next = (*next).max(estimated + MIN_CMD_TIME);
    }

    /// Line the command timeline up with the toolhead's (`_sync_print_time`):
    /// dwell out a pulse that is already scheduled ahead, otherwise adopt the
    /// planner's time.
    fn sync_print_time(&self) -> Result<(), CommandError> {
        let toolhead = self.toolhead()?;
        let print_time = toolhead.get_last_move_time();
        let scheduled = *self.next_cmd_time.lock().unwrap_or_else(|p| p.into_inner());
        if scheduled > print_time {
            toolhead.dwell(scheduled - print_time);
        } else {
            *self.next_cmd_time.lock().unwrap_or_else(|p| p.into_inner()) = print_time;
        }
        Ok(())
    }

    /// The toolhead, or "not ready".
    fn toolhead(&self) -> Result<Arc<ToolHeadObject>, CommandError> {
        self.printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT))
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    /// Send an informational line to the client, when one is listening.
    fn respond_info(&self, message: &str) {
        if let Some(printer) = self.printer.upgrade() {
            if let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) {
                gcode.respond_info(message, true);
            }
        }
    }

    /// One single-wire command (`_send_cmd`): drive the duty for
    /// `Commands[cmd] / SIGNAL_PERIOD` from `next_cmd_time`, turn the line off
    /// once the pulse has been held for its width, and advance the timeline.
    ///
    /// # Errors
    /// "Unknown BLTouch command" for a name outside [`COMMANDS`], or whatever
    /// the clock/PWM layer reports before the firmware is up.
    fn send_cmd(&self, cmd: &str, duration: f64) -> Result<(), CommandError> {
        let duty = command_duty(cmd)
            .ok_or_else(|| CommandError::new(format!("Unknown BLTouch command '{cmd}'")))?;
        let next = *self.next_cmd_time.lock().unwrap_or_else(|p| p.into_inner());
        let (offset, freq) = self.mcu.time_mapping();
        let start_clock = self.mcu.print_time_to_clock(next).ok_or_else(|| {
            CommandError::new("BLTouch control pin has no clock estimate yet".to_string())
        })?;
        if freq <= 0.0 {
            return Err(CommandError::new(
                "BLTouch control pin has no clock frequency yet".to_string(),
            ));
        }

        // Hold the pulse for its width, never shorter than the gap between
        // commands (`int((duration - MIN_CMD_TIME) / SIGNAL_PERIOD)` rounds
        // the hold down to whole signal periods).
        let held =
            (((duration - MIN_CMD_TIME) / SIGNAL_PERIOD).floor() * SIGNAL_PERIOD).max(MIN_CMD_TIME);
        let end_clock = start_clock + (held * freq) as u64;

        // Schedule the pulse, then the line off, as upstream does.
        self.pwm
            .set_pwm(start_clock as u32, duty)
            .map_err(|err| CommandError::new(err.to_string()))?;
        self.pwm
            .set_pwm(end_clock as u32, 0.0)
            .map_err(|err| CommandError::new(err.to_string()))?;

        // Time tracking: `action_end_time` is when the pulse's action is over;
        // the next command waits for the line to have been off for
        // `MIN_CMD_TIME`.
        let action_end = next + duration;
        *self
            .action_end_time
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = action_end;
        let end_time = end_clock as f64 / freq + offset;
        *self.next_cmd_time.lock().unwrap_or_else(|p| p.into_inner()) =
            action_end.max(end_time + MIN_CMD_TIME);
        Ok(())
    }

    /// Raise the pin (`_raise_probe`): a `pin_up` pulse, preceded by `reset`
    /// when the pin cannot be verified (`pin_up_reports_not_triggered` off).
    ///
    /// # Errors
    /// From the clock or the PWM resource.
    fn raise_probe(&self) -> Result<(), CommandError> {
        self.sync_mcu_print_time();
        if !self.pin_up_reports_not_triggered {
            self.send_cmd("reset", MIN_CMD_TIME)?;
        }
        self.send_cmd("pin_up", self.pin_move_time)
    }
}

/// The BLTouch hardware: the upstream `BLTouchProbe` — the protocol, the
/// sensor endstop, and the endstop wrapper the probing move drives
/// (`BLTouchProbe.home_start` / `home_wait`).
struct BlTouchProtocol {
    /// The protocol timeline and resources.
    timing: BlTouchTiming,
    /// The sensor pin, as the endstop every probing move arms.
    mcu_endstop: Arc<McuEndstop>,
    /// `stow_on_each_sample`.
    stow_on_each_sample: bool,
    /// `probe_with_touch_mode`.
    probe_with_touch_mode: bool,
    /// `pin_up_touch_mode_reports_triggered`.
    pin_up_touch_mode_reports_triggered: bool,
    /// `set_output_mode`.
    set_output_mode: Option<String>,
    /// When the sensor self-test may run next (`next_test_time`).
    next_test_time: Mutex<f64>,
    /// The completion the current move's trigger arrives on
    /// (`finish_home_complete`).
    finish_home_complete: Mutex<Option<Arc<Completion>>>,
    /// The scheduled `_wait_for_trigger` callback (`wait_trigger_complete`).
    wait_trigger_complete: Mutex<Option<tokio::task::JoinHandle<()>>>,
}

/// Whether this run writes its MCU output to a file (`MCU.is_fileoutput`).
///
/// The corpus runs klippy exactly that way, and upstream's endstop checks
/// answer it directly (`mcu.py:396-404`).
fn is_fileoutput(printer: &Weak<Printer>) -> bool {
    printer
        .upgrade()
        .is_some_and(|printer| printer.is_fileoutput())
}

/// Check that the sensor reports `triggered` now (`_verify_state`): arm the
/// endstop, wait out the window, and read the answer.
///
/// # Errors
/// From arming the endstop; a check that does not answer is not an error —
/// upstream turns it into `False`.
async fn verify_state(
    timing: &BlTouchTiming,
    endstop: &McuEndstop,
    triggered: bool,
) -> Result<bool, CommandError> {
    if is_fileoutput(&timing.printer) {
        // Upstream's file-output `home_wait` answers `home_end_time`, so the
        // check passes straight away; see the module docs.
        return Ok(true);
    }
    let action_end = *timing
        .action_end_time
        .lock()
        .unwrap_or_else(|p| p.into_inner());
    endstop
        .home_start(
            action_end,
            ENDSTOP_SAMPLE_TIME,
            ENDSTOP_SAMPLE_COUNT,
            ENDSTOP_REST_TIME,
            triggered,
        )
        .map_err(|err| CommandError::new(err.to_string()))?;
    match endstop.home_wait(action_end + VERIFY_WINDOW).await {
        Ok(trigger_time) => Ok(trigger_time > 0.0),
        // Upstream: `except self.printer.command_error: return False`.
        Err(_) => Ok(false),
    }
}

/// Verify the pin came up (`_verify_raise_probe`): three tries, each a `reset`
/// and another `pin_up`, then "BLTouch failed to raise probe".
///
/// # Errors
/// As [`verify_state`], or after the third failed try.
async fn verify_raise(timing: &BlTouchTiming, endstop: &McuEndstop) -> Result<(), CommandError> {
    if !timing.pin_up_reports_not_triggered {
        // No way to verify the raise attempt.
        return Ok(());
    }
    for retry in 0..3u8 {
        if verify_state(timing, endstop, false).await? {
            return Ok(());
        }
        if retry >= 2 {
            return Err(CommandError::new("BLTouch failed to raise probe"));
        }
        timing.respond_info("Failed to verify BLTouch probe is raised; retrying.");
        timing.sync_mcu_print_time();
        timing.send_cmd("reset", RETRY_RESET_TIME)?;
        timing.send_cmd("pin_up", timing.pin_move_time)?;
    }
    Ok(())
}

impl BlTouchProtocol {
    /// The output mode set at connect (`_set_output_mode`).
    ///
    /// # Errors
    /// From the clock or the PWM resource.
    fn set_output_mode(&self) -> Result<(), CommandError> {
        let Some(mode) = self.set_output_mode.as_deref() else {
            return Ok(());
        };
        info!("BLTouch set output mode: {mode}");
        self.timing.sync_mcu_print_time();
        if mode == "5V" {
            self.timing.send_cmd("set_5V_output_mode", MIN_CMD_TIME)?;
        }
        if mode == "OD" {
            self.timing.send_cmd("set_OD_output_mode", MIN_CMD_TIME)?;
        }
        Ok(())
    }

    /// Store an output mode in the EEPROM (`_store_output_mode`).
    ///
    /// # Errors
    /// From [`BlTouchTiming::sync_print_time`] or a send.
    fn store_output_mode(&self, mode: &str) -> Result<(), CommandError> {
        info!("BLTouch store output mode: {mode}");
        self.timing.sync_print_time()?;
        for cmd in store_output_mode_commands(mode) {
            self.timing.send_cmd(cmd, MIN_CMD_TIME)?;
        }
        Ok(())
    }

    /// Raise the probe and check it (`_handle_connect`'s raise half).
    async fn raise_and_verify(&self) -> Result<(), CommandError> {
        self.timing.raise_probe()?;
        verify_raise(&self.timing, &self.mcu_endstop).await
    }

    /// Test that the sensor still answers (`_test_sensor`): at most once every
    /// [`TEST_TIME`], raise, enter `touch_mode`, and look for a trigger.
    ///
    /// # Errors
    /// "BLTouch failed to verify sensor state" after three failed tries.
    async fn test_sensor(&self) -> Result<(), CommandError> {
        if !self.pin_up_touch_mode_reports_triggered {
            // Nothing to test.
            return Ok(());
        }
        let print_time = self.timing.toolhead()?.get_last_move_time();
        {
            let mut next_test = self
                .next_test_time
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            if print_time < *next_test {
                *next_test = print_time + TEST_TIME;
                return Ok(());
            }
        }
        self.timing.sync_print_time()?;
        for retry in 0..3u8 {
            self.timing.send_cmd("pin_up", self.timing.pin_move_time)?;
            self.timing.send_cmd("touch_mode", MIN_CMD_TIME)?;
            let success = verify_state(&self.timing, &self.mcu_endstop, true).await?;
            self.timing.sync_print_time()?;
            if success {
                *self
                    .next_test_time
                    .lock()
                    .unwrap_or_else(|p| p.into_inner()) = print_time + TEST_TIME;
                return Ok(());
            }
            if retry >= 2 {
                return Err(CommandError::new("BLTouch failed to verify sensor state"));
            }
            self.timing
                .respond_info("BLTouch failed to verify sensor state; retrying.");
            self.timing.send_cmd("reset", RETRY_RESET_TIME)?;
        }
        Ok(())
    }

    /// Put the pin down for a sample (`_lower_probe`).
    ///
    /// # Errors
    /// From [`BlTouchProtocol::test_sensor`] or a send.
    async fn lower_probe(&self) -> Result<(), CommandError> {
        self.test_sensor().await?;
        self.timing.sync_print_time()?;
        self.timing
            .send_cmd("pin_down", self.timing.pin_move_time)?;
        if self.probe_with_touch_mode {
            self.timing.send_cmd("touch_mode", MIN_CMD_TIME)?;
        }
        Ok(())
    }

    /// A session opened (`BLTouchProbe.start_probe_session`): with
    /// `stow_on_each_sample` off, the pin comes down for the first sample and
    /// stays down.
    fn start_session(&self) -> Result<(), CommandError> {
        if !self.stow_on_each_sample {
            *self.timing.multi.lock().unwrap_or_else(|p| p.into_inner()) = Multi::First;
        }
        Ok(())
    }

    /// A session closed (`BLTouchProbe.end_probe_session`): with
    /// `stow_on_each_sample` off, raise and verify the pin once more.
    ///
    /// The verify waits on the endstop like `_wait_for_trigger` does, so it
    /// runs as the same scheduled callback rather than inline.
    fn end_session(&self) -> Result<(), CommandError> {
        if self.stow_on_each_sample {
            return Ok(());
        }
        self.timing.sync_print_time()?;
        self.timing.raise_probe()?;
        let timing = self.timing.clone();
        let endstop = Arc::clone(&self.mcu_endstop);
        match tokio::runtime::Handle::try_current() {
            Ok(handle) => {
                handle.spawn(async move {
                    if let Err(err) = verify_raise(&timing, &endstop).await {
                        warn!("BLTouch raise probe error: {err}");
                    }
                });
            }
            Err(_) => warn!("BLTouch raise after a probe session needs a runtime to verify"),
        }
        self.timing.sync_print_time()?;
        *self.timing.multi.lock().unwrap_or_else(|p| p.into_inner()) = Multi::Off;
        Ok(())
    }

    /// Before the probing move (`BLTouchProbe._probe_prepare`): down for the
    /// sample, then line the timeline up with the toolhead.
    async fn probe_prepare(&self) -> Result<(), CommandError> {
        let multi = *self.timing.multi.lock().unwrap_or_else(|p| p.into_inner());
        if multi == Multi::Off || multi == Multi::First {
            self.lower_probe().await?;
            if multi == Multi::First {
                *self.timing.multi.lock().unwrap_or_else(|p| p.into_inner()) = Multi::On;
            }
        }
        self.timing.sync_print_time()
    }

    /// After the probing move (`BLTouchProbe._probe_finish`): wait for the
    /// scheduled `_wait_for_trigger`, verify the raise when the pin is stowed
    /// per sample, then line the timeline up again.
    async fn probe_finish(&self) -> Result<(), CommandError> {
        let wait_trigger = self
            .wait_trigger_complete
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take();
        if let Some(wait_trigger) = wait_trigger {
            if let Err(err) = wait_trigger.await {
                warn!("BLTouch wait_for_trigger failed: {err}");
            }
        }
        let multi = *self.timing.multi.lock().unwrap_or_else(|p| p.into_inner());
        if multi == Multi::Off {
            verify_raise(&self.timing, &self.mcu_endstop).await?;
        }
        self.timing.sync_print_time()
    }

    /// `BLTOUCH_DEBUG`: send one protocol command, or list them
    /// (`cmd_BLTOUCH_DEBUG`).
    fn cmd_bl_touch_debug(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let cmd = gcmd.get_str_default("COMMAND", "");
        if command_width(&cmd).is_none() {
            gcmd.respond_info(&format!("BLTouch commands: {}", command_names().join(", ")));
            return Ok(());
        }
        gcmd.respond_info(&format!("Sending BLTOUCH_DEBUG COMMAND={cmd}"));
        self.timing.sync_print_time()?;
        self.timing.send_cmd(&cmd, self.timing.pin_move_time)?;
        self.timing.sync_print_time()?;
        Ok(())
    }

    /// `BLTOUCH_STORE`: write an output mode to the EEPROM
    /// (`cmd_BLTOUCH_STORE`).
    fn cmd_bl_touch_store(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let mode = gcmd.get_str_default("MODE", "");
        if mode != "5V" && mode != "OD" {
            gcmd.respond_info("BLTouch output modes: 5V, OD");
            return Ok(());
        }
        gcmd.respond_info(&format!("Storing BLTouch output mode: {mode}"));
        self.timing.sync_print_time()?;
        self.store_output_mode(&mode)?;
        self.timing.sync_print_time()?;
        Ok(())
    }
}

/// The endstop wrapper the probing move drives (`BLTouchProbe` as
/// `MCU_endstop`): the sensor endstop, plus the raise that follows the trigger.
impl HomingEndstop for BlTouchProtocol {
    fn home_start(
        &self,
        print_time: f64,
        sample_time: f64,
        sample_count: u8,
        rest_time: f64,
        triggered: bool,
    ) -> Result<Arc<Completion>, McuError> {
        // The sensor check polls faster than any move asks for.
        let rest_time = rest_time.min(ENDSTOP_REST_TIME);
        let completion = self.mcu_endstop.home_start(
            print_time,
            sample_time,
            sample_count,
            rest_time,
            triggered,
        )?;
        *self
            .finish_home_complete
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = Some(Arc::clone(&completion));

        // Schedule `_wait_for_trigger`: once the sensor trips, raise the pin
        // (when each sample stows its own).
        let for_trigger = Arc::clone(&completion);
        let timing = self.timing.clone();
        let spawned = tokio::runtime::Handle::try_current().map(|handle| {
            handle.spawn(async move {
                for_trigger.wait().await;
                let multi = *timing.multi.lock().unwrap_or_else(|p| p.into_inner());
                if multi == Multi::Off {
                    if let Err(err) = timing.raise_probe() {
                        warn!("BLTouch raise after trigger failed: {err}");
                    }
                }
            })
        });
        match spawned {
            Ok(handle) => {
                *self
                    .wait_trigger_complete
                    .lock()
                    .unwrap_or_else(|p| p.into_inner()) = Some(handle);
            }
            Err(_) => warn!("BLTouch cannot schedule wait_for_trigger without a runtime"),
        }
        Ok(completion)
    }

    fn home_wait(&self, home_end_time: f64) -> EndstopFuture<'_> {
        // The raise itself is `_wait_for_trigger`'s; the wait is the sensor's.
        Box::pin(self.mcu_endstop.home_wait(home_end_time))
    }
}

// ===========================================================================
// The section
// ===========================================================================

/// The MCU whose clock the `control_pin` pulses are scheduled on.
///
/// Upstream asks the PWM resource (`self.mcu_pwm.get_mcu()`); this port's
/// [`PwmOut`] does not carry its chip yet, so the description's chip prefix
/// (`zboard:PA5`) names the `[mcu]` object the loader registered.
///
/// # Errors
/// When no `[mcu]` section answers to that name.
fn lookup_control_mcu(
    printer: &Arc<Printer>,
    control_pin: &str,
) -> Result<Arc<McuObject>, ConfigError> {
    let description = control_pin.trim_start_matches(|c| matches!(c, '!' | '^' | '~'));
    // A chip is named only by the description's `zboard:PA5` form; a bare
    // `PC5` lives on the main MCU.
    let chip = match description.split_once(':') {
        Some((chip, _)) if !chip.is_empty() => chip,
        _ => "mcu",
    };
    let object = if chip == "mcu" {
        "mcu".to_string()
    } else {
        format!("mcu {chip}")
    };
    printer
        .lookup_object_as::<McuObject>(&object)
        .ok_or_else(|| {
            ConfigError::new(format!(
                "BLTouch control_pin '{control_pin}' needs an [{object}] section"
            ))
        })
}

/// One configured `[bltouch]` (`bltouch.py:PrinterBLTouch`).
///
/// It owns the protocol and the commands; the `probe` object it registers
/// alongside is a [`PrinterProbe`] over the same session (see the module docs).
pub struct PrinterBLTouch {
    /// The section's identifier (`bltouch`), for logging and `get_status`.
    identifier: String,
    /// The hardware: protocol, sensor endstop, endstop wrapper.
    protocol: Arc<BlTouchProtocol>,
    /// The object registered as `probe`, for the probe consumers.
    probe: Arc<PrinterProbe>,
    /// What both objects' `get_status` report.
    state: Arc<ProbeCommandState>,
}

impl PrinterBLTouch {
    /// Build the protocol, the `probe` object and the commands.
    ///
    /// # Errors
    /// A config error when an option is missing or malformed, when a pin
    /// cannot be built, when the `probe` chip or a probe command is already
    /// taken, or when the session cannot be built.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let options = BlTouchOptions::read(config)?;

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");

        // The control pin: a PWM whose duty carries the single-wire protocol
        // (`ppins.setup_pin('pwm', ...)` + `setup_max_duration(0.)` +
        // `setup_cycle_time(SIGNAL_PERIOD)`).
        let control = pins
            .setup_pwm(&options.control_pin, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        control.setup_max_duration(0.0);
        // Upstream's default `hardware_pwm=False`
        // (`mcu.py:setup_cycle_time(cycle_time, hardware_pwm=False)`).
        control.setup_cycle_time(SIGNAL_PERIOD, false);

        // The sensor pin: the endstop the probe descends on
        // (`ppins.setup_pin('endstop', ...)`).
        let sensor = pins
            .setup_endstop(&options.sensor_pin, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        let mcu = lookup_control_mcu(printer, &options.control_pin)?;

        // `HomingViaProbeHelper`: register the `probe` chip so
        // `endstop_pin: probe:z_virtual_endstop` resolves, pointing at the
        // sensor endstop and this section's `z_offset`.
        pins.register_chip(
            "probe",
            Arc::new(ProbeChip {
                endstop: Arc::clone(&sensor),
                z_offset: options.z_offset,
            }),
        )
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        let protocol = Arc::new(BlTouchProtocol {
            timing: BlTouchTiming {
                printer: Arc::downgrade(printer),
                pwm: control,
                mcu,
                next_cmd_time: Arc::new(Mutex::new(0.0)),
                action_end_time: Arc::new(Mutex::new(0.0)),
                multi: Arc::new(Mutex::new(Multi::Off)),
                pin_move_time: options.pin_move_time,
                pin_up_reports_not_triggered: options.pin_up_reports_not_triggered,
            },
            mcu_endstop: Arc::clone(&sensor),
            stow_on_each_sample: options.stow_on_each_sample,
            probe_with_touch_mode: options.probe_with_touch_mode,
            pin_up_touch_mode_reports_triggered: options.pin_up_touch_mode_reports_triggered,
            set_output_mode: options.set_output_mode.clone(),
            next_test_time: Mutex::new(0.0),
            finish_home_complete: Mutex::new(None),
            wait_trigger_complete: Mutex::new(None),
        });

        // The probe helpers this section reuses: `ProbeOffsetsHelper` and
        // `ProbeParameterHelper` read the same options `[probe]` does, with
        // `stow_on_each_sample` where `[probe]` has
        // `deactivate_on_each_sample` and `samples_result` defaulting to
        // `average` (see `BlTouchOptions::read`).
        let probe_options = ProbeOptions {
            pin: options.sensor_pin.clone(),
            z_offset: options.z_offset,
            x_offset: options.x_offset,
            y_offset: options.y_offset,
            speed: options.speed,
            lift_speed: options.lift_speed,
            samples: options.samples,
            sample_retract_dist: options.sample_retract_dist,
            samples_result: options.samples_result.clone(),
            samples_tolerance: options.samples_tolerance,
            samples_tolerance_retries: options.samples_tolerance_retries,
            deactivate_on_each_sample: options.stow_on_each_sample,
            activate_gcode: None,
            deactivate_gcode: None,
        };

        // `SampleAveragingHelper` over `BLTouchProbe`'s own session steps.
        let hooks = ProbeHooks {
            start: {
                let protocol = Arc::clone(&protocol);
                Arc::new(move || protocol.start_session())
            },
            end: {
                let protocol = Arc::clone(&protocol);
                Arc::new(move || protocol.end_session())
            },
            prepare: {
                let protocol = Arc::clone(&protocol);
                Arc::new(move || {
                    let protocol = Arc::clone(&protocol);
                    Box::pin(async move { protocol.probe_prepare().await })
                })
            },
            finish: {
                let protocol = Arc::clone(&protocol);
                Arc::new(move || {
                    let protocol = Arc::clone(&protocol);
                    Box::pin(async move { protocol.probe_finish().await })
                })
            },
        };

        // The probing move drives the wrapper, `QUERY_PROBE` reads the sensor
        // endstop directly (upstream: `BLTouchProbe.query_endstop` is the bare
        // `mcu_endstop`'s).
        let session = Arc::new(ProbeSessionHelper::new(
            config,
            printer,
            Arc::clone(&protocol) as Arc<dyn HomingEndstop>,
            Arc::clone(&sensor),
            &probe_options,
            Some(hooks),
        )?);
        let state = Arc::new(ProbeCommandState::default());
        let offsets = ProbeOffsets {
            x: options.x_offset,
            y: options.y_offset,
            z: options.z_offset,
        };
        // `ProbeCommandHelper`: QUERY_PROBE / PROBE / PROBE_ACCURACY /
        // PROBE_CALIBRATE, registered through the probe's own path rather than
        // a second copy — a section registers them once, as upstream does.
        register_commands(printer, &identifier, &session, &state, offsets)?;

        let probe = Arc::new(PrinterProbe::from_parts(
            identifier.clone(),
            probe_options,
            Arc::clone(&sensor),
            session,
            Arc::clone(&state),
        ));
        // Upstream: `config.get_printer().add_object('probe', blt)`.
        printer.add_object("probe", probe.clone())?;

        // BLTOUCH_DEBUG / BLTOUCH_STORE: the section's own commands.
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");
        {
            let protocol = Arc::clone(&protocol);
            gcode
                .register_command(
                    "BLTOUCH_DEBUG",
                    sync(move |gcmd| protocol.cmd_bl_touch_debug(gcmd)),
                    Some("Send a command to the bltouch for debugging"),
                    false,
                )
                .map_err(ConfigError::new)?;
        }
        {
            let protocol = Arc::clone(&protocol);
            gcode
                .register_command(
                    "BLTOUCH_STORE",
                    sync(move |gcmd| protocol.cmd_bl_touch_store(gcmd)),
                    Some("Store an output mode in the BLTouch EEPROM"),
                    false,
                )
                .map_err(ConfigError::new)?;
        }

        Ok(Self {
            identifier,
            protocol,
            probe,
            state,
        })
    }

    /// Upstream's `_handle_connect`: sync the clock, set the output mode, then
    /// raise the probe and verify it. A protocol failure here is a warning, as
    /// upstream's is — it must not keep the machine from coming up.
    async fn handle_connect(&self) {
        self.protocol.timing.sync_mcu_print_time();
        {
            let mut next = self
                .protocol
                .timing
                .next_cmd_time
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            *next += 0.200;
        }
        if let Err(err) = self.protocol.set_output_mode() {
            warn!("BLTouch set output mode error: {err}");
        }
        if let Err(err) = self.protocol.raise_and_verify().await {
            warn!("BLTouch raise probe error: {err}");
        }
    }

    /// The section identifier.
    pub fn identifier(&self) -> &str {
        &self.identifier
    }

    /// The probe object this section registered as `probe`.
    pub fn probe(&self) -> &Arc<PrinterProbe> {
        &self.probe
    }
}

impl PrinterObject for PrinterBLTouch {
    fn get_status(&self, _eventtime: f64) -> Value {
        command_status(&self.identifier, &self.state)
    }

    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            self.handle_connect().await;
            Ok(())
        })
    }
}

impl std::fmt::Debug for PrinterBLTouch {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterBLTouch")
            .field("identifier", &self.identifier)
            .finish_non_exhaustive()
    }
}

/// Upstream's `load_config` for `[bltouch]`.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PrinterBLTouch::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{ConfigSection, ConfigValue};
    use crate::core::klippy::pins::PinError;
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[bltouch]` section with the given options, as the parser builds it.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("bltouch", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// The minimum a section needs: both pins and the z offset.
    const MINIMAL: &[(&str, &str)] = &[
        ("sensor_pin", "PC7"),
        ("control_pin", "PC5"),
        ("z_offset", "1.15"),
    ];

    #[test]
    fn options_carry_upstream_defaults() {
        let options = BlTouchOptions::read(&ConfigWrapper::untracked(&section(MINIMAL))).unwrap();

        assert_eq!(options.sensor_pin, "PC7");
        assert_eq!(options.control_pin, "PC5");
        assert_eq!(options.stow_on_each_sample, true);
        assert_eq!(options.probe_with_touch_mode, false);
        assert_eq!(options.set_output_mode, None);
        assert_eq!(options.pin_up_reports_not_triggered, true);
        assert_eq!(options.pin_up_touch_mode_reports_triggered, true);
        assert_eq!(options.pin_move_time, 0.680);
        assert_eq!(options.x_offset, 0.0);
        assert_eq!(options.y_offset, 0.0);
        assert_eq!(options.z_offset, 1.15);
        assert_eq!(options.speed, 5.0);
        assert_eq!(options.lift_speed, None);
        assert_eq!(options.samples, 1);
        assert_eq!(options.sample_retract_dist, 2.0);
        // `ProbeParameterHelper`, not `[probe]`: the default is `average`.
        assert_eq!(options.samples_result, "average");
        assert_eq!(options.samples_tolerance, 0.100);
        assert_eq!(options.samples_tolerance_retries, 0);
    }

    #[test]
    fn every_option_the_corpus_writes_is_claimed() {
        // Every option any shipped config writes in a `[bltouch]` section
        // (`test/klippy/bltouch.cfg`, `test/klippy/screws_tilt_adjust.cfg`,
        // `config/*.cfg`), so `check_unused` passes on all of them.
        let mut all = MINIMAL.to_vec();
        all.extend_from_slice(&[
            ("stow_on_each_sample", "False"),
            ("probe_with_touch_mode", "True"),
            ("set_output_mode", "5V"),
            ("pin_up_reports_not_triggered", "False"),
            ("pin_up_touch_mode_reports_triggered", "false"),
            ("pin_move_time", "0.5"),
            ("x_offset", "39"),
            ("y_offset", "-12.8"),
            ("speed", "10"),
            ("lift_speed", "7.5"),
            ("samples", "3"),
            ("sample_retract_dist", "3.0"),
            ("samples_result", "median"),
            ("samples_tolerance", "0.050"),
            ("samples_tolerance_retries", "3"),
        ]);
        // `z_offset` twice would overwrite; keep the corpus's single value.
        all.retain(|(option, _)| *option != "z_offset");

        let mut written = section(&all);
        written.parameters.insert(
            "z_offset".to_string(),
            ConfigValue::Single("2.60".to_string()),
        );
        let options = BlTouchOptions::read(&ConfigWrapper::untracked(&written)).unwrap();

        assert_eq!(options.stow_on_each_sample, false);
        assert_eq!(options.probe_with_touch_mode, true);
        assert_eq!(options.set_output_mode.as_deref(), Some("5V"));
        assert_eq!(options.pin_up_reports_not_triggered, false);
        assert_eq!(options.pin_up_touch_mode_reports_triggered, false);
        assert_eq!(options.pin_move_time, 0.5);
        assert_eq!(options.x_offset, 39.0);
        assert_eq!(options.y_offset, -12.8);
        assert_eq!(options.z_offset, 2.60);
        assert_eq!(options.speed, 10.0);
        assert_eq!(options.lift_speed, Some(7.5));
        assert_eq!(options.samples, 3);
        assert_eq!(options.sample_retract_dist, 3.0);
        assert_eq!(options.samples_result, "median");
        assert_eq!(options.samples_tolerance, 0.050);
        assert_eq!(options.samples_tolerance_retries, 3);
    }

    #[test]
    fn a_bad_output_mode_is_refused() {
        let mut options = MINIMAL.to_vec();
        options.push(("set_output_mode", "3.3V"));
        let err = BlTouchOptions::read(&ConfigWrapper::untracked(&section(&options))).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Choice '3.3V' for option 'set_output_mode' in section 'bltouch' is not a valid choice"
        );
    }

    #[test]
    fn a_missing_sensor_pin_is_refused() {
        let options = &[("control_pin", "PC5"), ("z_offset", "1.15")];
        let err = BlTouchOptions::read(&ConfigWrapper::untracked(&section(options))).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'sensor_pin' in section 'bltouch' must be specified"
        );
    }

    // -----------------------------------------------------------------------
    // The protocol bytes
    // -----------------------------------------------------------------------

    #[test]
    fn the_protocol_duty_is_the_command_width_over_the_signal_period() {
        // `_send_cmd`: `self.mcu_pwm.set_pwm(..., Commands[cmd] / SIGNAL_PERIOD)`.
        let duty = |cmd: &str| {
            let duty = command_duty(cmd).expect("the command is in the table");
            (duty * 100_000.0).round() / 100_000.0
        };
        assert_eq!(duty("pin_down"), 0.0325);
        assert_eq!(duty("touch_mode"), 0.05825);
        assert_eq!(duty("pin_up"), 0.07375);
        assert_eq!(duty("self_test"), 0.089);
        assert_eq!(duty("reset"), 0.1095);
        assert_eq!(duty("set_5V_output_mode"), 0.0994);
        assert_eq!(duty("set_OD_output_mode"), 0.10455);
        assert_eq!(duty("output_mode_store"), 0.0942);
        // A name the BLTouch does not know has no duty at all.
        assert_eq!(command_duty("lift_off"), None);
    }

    #[test]
    fn the_debug_command_list_is_upstreams_sorted_list() {
        assert_eq!(
            command_names().join(", "),
            "output_mode_store, pin_down, pin_up, reset, self_test, \
             set_5V_output_mode, set_OD_output_mode, touch_mode"
        );
    }

    #[test]
    fn storing_an_output_mode_sends_the_upstream_sequence() {
        // `_store_output_mode`: pin-down, the mode, store, the mode again,
        // pin-up.
        assert_eq!(
            store_output_mode_commands("5V"),
            [
                "pin_down",
                "set_5V_output_mode",
                "output_mode_store",
                "set_5V_output_mode",
                "pin_up"
            ]
        );
        assert_eq!(
            store_output_mode_commands("OD"),
            [
                "pin_down",
                "set_OD_output_mode",
                "output_mode_store",
                "set_OD_output_mode",
                "pin_up"
            ]
        );
    }

    // -----------------------------------------------------------------------
    // Loading the section
    // -----------------------------------------------------------------------

    /// A printer with the parts a section is built against: `gcode`, `pins`
    /// and the main `[mcu]` (connected or not — resources are built while the
    /// config is read).
    fn machine() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        let mcu = McuObject::new(ConfigSection::new("mcu", None), &printer).unwrap();
        printer.add_object("mcu", Arc::new(mcu)).unwrap();
        printer
    }

    /// Load a `[bltouch]` section onto `machine()`; returns the printer so the
    /// test can inspect what it registered.
    fn load(options: &[(&str, &str)]) -> Result<Arc<Printer>, ConfigError> {
        let printer = machine();
        let section = section(options);
        load_config(&ConfigWrapper::untracked(&section), &printer)?;
        Ok(printer)
    }

    #[test]
    fn the_section_registers_the_probe_chip_and_the_probe_object() {
        let printer = load(MINIMAL).expect("the minimal section loads");

        // Upstream `load_config` adds the section's probe as the `probe`
        // object; both spurs resolve `probe:z_virtual_endstop`.
        assert!(printer.lookup_object("probe").is_some());
        assert!(
            printer.lookup_object_as::<PrinterProbe>("probe").is_some(),
            "the probe consumers look the `probe` object up as a PrinterProbe"
        );
        let status = printer
            .lookup_object("probe")
            .expect("probe object")
            .get_status(0.0);
        assert_eq!(status["name"], "bltouch");
        assert_eq!(status["last_query"], false);
        assert_eq!(status["last_z_result"], 0.0);

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("pins");
        assert!(pins.chips().iter().any(|chip| chip == "probe"));

        // The virtual endstop resolves to the section's sensor endstop.
        let _endstop = pins
            .setup_endstop("probe:z_virtual_endstop", None)
            .expect("probe:z_virtual_endstop resolves");
        // NB: `PrinterPins::virtual_endstop_position` looks its value up by
        // `(endstop.chip_name(), oid)` while `setup_endstop` stores it under
        // `(params.chip_name, oid)` — for a virtual name those are `mcu` and
        // `probe`, so the stored `z_offset` is not read back. That mismatch
        // predates this section (it is the same for `[probe]`) and is left
        // for main: rails fall back to the section's `position_endstop`.
    }

    /// The message a refused pin lookup reports (`unwrap_err` needs a
    /// `Debug` Ok type, which an endstop resource is not).
    fn refused(result: Result<Arc<McuEndstop>, PinError>) -> String {
        match result {
            Err(err) => err.to_string(),
            Ok(_) => panic!("the pin lookup was refused"),
        }
    }

    #[test]
    fn another_pin_name_on_the_probe_chip_is_refused() {
        let printer = load(MINIMAL).expect("the minimal section loads");
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("pins");

        let err = refused(pins.setup_endstop("probe:z", None));
        assert_eq!(err, "Probe virtual endstop only useful as endstop pin");
    }

    #[test]
    fn inverting_or_pulling_up_the_virtual_endstop_is_refused() {
        // Each description gets its own registry: a second look at the same
        // virtual pin would be a polarity/alias clash before it reaches the
        // probe's own refusal.
        let inverted = {
            let printer = load(MINIMAL).expect("the minimal section loads");
            let pins = printer
                .lookup_object_as::<PrinterPins>(PINS_OBJECT)
                .expect("pins");
            refused(pins.setup_endstop("!probe:z_virtual_endstop", None))
        };
        assert_eq!(inverted, "Can not pullup/invert probe virtual endstop");

        let pulled_up = {
            let printer = load(MINIMAL).expect("the minimal section loads");
            let pins = printer
                .lookup_object_as::<PrinterPins>(PINS_OBJECT)
                .expect("pins");
            refused(pins.setup_endstop("^probe:z_virtual_endstop", None))
        };
        assert_eq!(pulled_up, "Can not pullup/invert probe virtual endstop");
    }

    #[test]
    fn a_control_pin_on_an_unknown_chip_reports_the_output_pin_wording() {
        // `[output_pin]` reports a resource failure as `<section>: <pin error>`;
        // the section wraps its pins the same way.
        let options = &[
            ("sensor_pin", "PC7"),
            ("control_pin", "nope:PA5"),
            ("z_offset", "1.15"),
        ];
        let err = match load(options) {
            Err(err) => err.to_string(),
            Ok(_) => panic!("the section was refused"),
        };

        assert_eq!(err, "bltouch: Unknown pin chip name 'nope'");
        assert_eq!(
            PinError::UnknownChip("nope".to_string()).to_string(),
            "Unknown pin chip name 'nope'"
        );
    }
}
