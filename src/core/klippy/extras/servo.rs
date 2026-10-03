//! `[servo <name>]` — a hobby servo driven by a PWM pin
//! (upstream `klippy/extras/servo.py`).
//!
//! The section parses the pulse geometry upstream's `PrinterServo.__init__`
//! reads, builds the PWM output on the configured pin, and registers
//! `SET_SERVO` as a mux command keyed by `SERVO` (`servo.py:41-45`).
//!
//! | option | default | bounds | role |
//! |---|---|---|---|
//! | `minimum_pulse_width` | 0.001 s | `> 0`, `< 0.020` | pulse at angle 0 |
//! | `maximum_pulse_width` | 0.002 s | `> minimum_pulse_width`, `< 0.020` | pulse at `maximum_servo_angle` |
//! | `maximum_servo_angle` | 180 | — | angle the max pulse maps to |
//! | `initial_angle` | — | `0 ..= 360` | startup angle, else `initial_pulse_width` |
//! | `initial_pulse_width` | 0 s | `0 ..= maximum_pulse_width` | startup pulse when no angle |
//! | `pin` | — (required) | — | the PWM pin |
//!
//! # Scheduling
//!
//! `SET_SERVO` rides this port's print-time request queue, as upstream's does
//! (`servo.py:38-39` builds an `output_pin.GCodeRequestQueue`, `:66-73` queues
//! each value through `queue_gcode_request`): the duty is pinned to the
//! toolhead's lookahead time, a flush callback drains the queue, and the sink
//! below is upstream's `_set_pwm` (`servo.py:48-56`) one for one — a repeat of
//! the driven duty answers `discard`, the pulse is aligned to the servo's
//! 0.020 s cycle allowing [`RESCHEDULE_SLACK`] early (`servo.py:51`), an
//! alignment landing more than `RESCHEDULE_SLACK` late answers `reschedule`
//! with the aligned time (the queue retries there), and only then is the frame
//! sent. The lookahead/arm wiring (`queue_at_lookahead` in
//! [`output_pin`](super::output_pin)) is shared with [`pwm_tool`]; `servo`
//! keeps its own sink because only upstream's servo aligns this way.
//!
//! Two forks keep the **immediate** path (`update_pwm`), with no error and no
//! panic — the same two [`output_pin`](crate::core::klippy::extras::output_pin)
//! documents: no `toolhead` object (a config without `[printer]`), or the pin
//! resource cannot schedule yet (`min_schedule_time()` is `None`: its MCU is
//! not connected). A repeat of the driven duty is then skipped by the command
//! itself, where the queued path lets the sink discard it.

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::gcode_request_queue::{
    FlushAction, GCodeRequestQueue, RequestSink,
};
use crate::core::klippy::extras::output_pin::queue_at_lookahead;
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::pins::{PrinterPins, PwmOut, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form (`[servo <name>]`) exists upstream (`servo.py:75`).
section!("servo", order = 20, prefix = load_config_prefix);

/// The PWM period every servo runs at (`servo.py:8`).
const SERVO_SIGNAL_PERIOD: f64 = 0.020;

/// How far before the requested time a cycle-aligned pulse may land — and the
/// overshoot past it that makes `_set_pwm` answer `reschedule`
/// (`servo.py:9`, `:51-53`).
const RESCHEDULE_SLACK: f64 = 0.000500;

/// The pulse geometry of one servo: how angles and widths map to a duty
/// fraction of [`SERVO_SIGNAL_PERIOD`] (`servo.py:16-24`, `:57-64`).
#[derive(Debug, Clone, Copy)]
struct Geometry {
    /// Pulse width at angle 0 (`minimum_pulse_width`).
    min_width: f64,
    /// Pulse width at `maximum_servo_angle`.
    max_width: f64,
    /// The angle `max_width` is reached at (`maximum_servo_angle`).
    max_angle: f64,
}

impl Geometry {
    /// Seconds → duty fraction of the signal period.
    fn width_to_value(width: f64) -> f64 {
        width / SERVO_SIGNAL_PERIOD
    }

    /// The duty for an angle: clamped to `0 ..= max_angle`, mapped linearly
    /// onto `min_width ..= max_width` (`servo.py:57-60`).
    fn pwm_from_angle(&self, angle: f64) -> f64 {
        let angle = angle.max(0.).min(self.max_angle);
        let width = self.min_width + angle * self.angle_to_width();
        Self::width_to_value(width)
    }

    /// The duty for a raw pulse width: a zero width stays zero, anything else
    /// is clamped into `min_width ..= max_width` (`servo.py:61-64`).
    fn pwm_from_pulse_width(&self, width: f64) -> f64 {
        let width = if width != 0. {
            width.max(self.min_width).min(self.max_width)
        } else {
            0.
        };
        Self::width_to_value(width)
    }

    /// Millimetres of width one degree of angle adds (`servo.py:20`).
    fn angle_to_width(&self) -> f64 {
        (self.max_width - self.min_width) / self.max_angle
    }
}

/// One configured `[servo <name>]`.
pub struct PrinterServo {
    /// The scheduling state `SET_SERVO` and `get_status` share: the value
    /// slot, the pin, and the armed request queue.
    schedule: Arc<ServoSchedule>,
}

impl PrinterServo {
    /// Read the section, build the PWM pin, and register `SET_SERVO`.
    ///
    /// # Errors
    /// Returns a config error (a message naming the section) when the section
    /// has no name, an option is missing or out of bounds, or the pin cannot
    /// be built.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let name = config.section().sub.clone().ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{identifier}' must be a '[servo <name>]' section"
            ))
        })?;

        // Upstream's read order (`servo.py:12-26`): `minimum_pulse_width`,
        // `maximum_pulse_width`, `maximum_servo_angle`, `initial_angle` or
        // `initial_pulse_width`, `pin`.
        let min_width = config.get_float_bounded(
            "minimum_pulse_width",
            Some(0.001),
            None,
            None,
            Some(0.),
            Some(SERVO_SIGNAL_PERIOD),
        )?;
        let max_width = config.get_float_bounded(
            "maximum_pulse_width",
            Some(0.002),
            None,
            None,
            Some(min_width),
            Some(SERVO_SIGNAL_PERIOD),
        )?;
        let max_angle = config.get_float("maximum_servo_angle", Some(180.))?;
        let geometry = Geometry {
            min_width,
            max_width,
            max_angle,
        };
        // `initial_angle` and `initial_pulse_width` are alternatives; upstream
        // reads whichever the config gives (an absent option records nothing).
        let initial_value = if config.section().has("initial_angle") {
            let angle = config.get_float_bounded(
                "initial_angle",
                None,
                Some(0.),
                Some(360.),
                None,
                None,
            )?;
            geometry.pwm_from_angle(angle)
        } else if config.section().has("initial_pulse_width") {
            let width = config.get_float_bounded(
                "initial_pulse_width",
                None,
                Some(0.),
                Some(max_width),
                None,
                None,
            )?;
            geometry.pwm_from_pulse_width(width)
        } else {
            geometry.pwm_from_pulse_width(0.)
        };

        let pin_desc = config.get("pin", None)?;
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let pwm = pins
            .setup_pwm(&pin_desc, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        pwm.setup_cycle_time(SERVO_SIGNAL_PERIOD, false);
        pwm.setup_max_duration(0.);
        pwm.setup_start_value(initial_value, 0.);

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        // `last_value` starts at 0 whatever the startup pulse drives
        // (`servo.py:22`); the sink and `get_status` share the slot.
        let schedule = Arc::new(ServoSchedule {
            name: name.clone(),
            value: Arc::new(Mutex::new(0.)),
            pwm,
            printer: Arc::downgrade(printer),
            armed: Mutex::new(None),
        });
        let handler: CommandHandler = {
            let schedule = Arc::clone(&schedule);
            sync(move |gcmd| cmd_set_servo(&schedule, geometry, gcmd))
        };
        gcode
            // `WIDTH` and `ANGLE` are the two keys `cmd_set_servo` reads (in
            // that order); the mux `SERVO` key is prepended by the registrar.
            .register_mux_command_with_params(
                "SET_SERVO",
                "SERVO",
                Some(&name),
                handler,
                Some("Set servo angle"),
                &["WIDTH", "ANGLE"],
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        Ok(Self { schedule })
    }

    /// The name `SET_SERVO SERVO=<name>` addresses this servo by.
    pub fn name(&self) -> &str {
        &self.schedule.name
    }
}

impl PrinterObject for PrinterServo {
    /// The duty last driven, as upstream's `PrinterServo.get_status`
    /// (`servo.py:46-47`).
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "value": self.schedule.value() })
    }
}

impl std::fmt::Debug for PrinterServo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterServo")
            .field("name", &self.schedule.name)
            .finish_non_exhaustive()
    }
}

/// The scheduling state behind `SET_SERVO`: the shared value slot, the pin,
/// and the request queue once the first schedulable command arms it —
/// `output_pin::PinSchedule`'s counterpart for a sink that aligns.
struct ServoSchedule {
    /// The section's sub: names the servo in send-failure log lines.
    name: String,
    /// The duty last **driven** (upstream's `last_value`, `servo.py:22, 55`):
    /// moved when the frame lands — at flush time on the queued path, at once
    /// on the immediate one — and read by `get_status`.
    value: Arc<Mutex<f64>>,
    /// The PWM pin the sink aligns and drives.
    pwm: Arc<dyn PwmOut>,
    /// The printer, so a command can find the toolhead after config load.
    /// Weak: the printer owns the g-code handlers, a strong handle would close
    /// a `printer → objects → gcode → handler → printer` cycle.
    printer: Weak<Printer>,
    /// The queue, built and armed exactly once — by the first `SET_SERVO`
    /// that finds both a toolhead and a schedulable resource (see the module
    /// docs for the two fallbacks).
    armed: Mutex<Option<Arc<GCodeRequestQueue<ServoSink>>>>,
}

impl ServoSchedule {
    /// The value last driven.
    fn value(&self) -> f64 {
        *self
            .value
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

/// `SET_SERVO SERVO=<name> ANGLE=<a>` or `WIDTH=<seconds>`: queue the change
/// at the print time the toolhead gives it (upstream `cmd_SET_SERVO`,
/// `servo.py:66-73`), or drive the pin at once when no timeline can date it.
///
/// `WIDTH` wins when present; otherwise `ANGLE` is required
/// (`servo.py:66-73`). A repeat of the current duty sends nothing: the queued
/// path reaches that discard in [`ServoSink::set_at`] (upstream's `_set_pwm`,
/// `servo.py:49-50`), the immediate path guards it here.
fn cmd_set_servo(
    schedule: &ServoSchedule,
    geometry: Geometry,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let value = if gcmd.get_command_parameters().contains_key("WIDTH") {
        geometry.pwm_from_pulse_width(gcmd.get_float("WIDTH")?)
    } else {
        geometry.pwm_from_angle(gcmd.get_float("ANGLE")?)
    };
    let sink = ServoSink {
        name: schedule.name.clone(),
        pwm: Arc::clone(&schedule.pwm),
        value: Arc::clone(&schedule.value),
    };
    if queue_at_lookahead(
        &schedule.printer,
        &schedule.armed,
        schedule.pwm.min_schedule_time(),
        sink,
        value,
    ) {
        return Ok(());
    }
    // Immediate fallback (no toolhead, or the pin cannot schedule): the sink
    // would discard a repeat there, so the guard lives here instead.
    if value == schedule.value() {
        return Ok(());
    }
    schedule
        .pwm
        .update_pwm(value)
        .map_err(|err| CommandError::new(err.to_string()))?;
    *schedule
        .value
        .lock()
        .unwrap_or_else(|poison| poison.into_inner()) = value;
    Ok(())
}

/// The queue's downstream end: upstream's `PrinterServo._set_pwm`
/// (`servo.py:48-56`) — discard a repeat, align the pulse to the servo's
/// cycle, reschedule a late alignment, and only then send.
struct ServoSink {
    /// Names the servo in send-failure log lines.
    name: String,
    /// The pin to align and drive.
    pwm: Arc<dyn PwmOut>,
    /// Upstream's `last_value`.
    value: Arc<Mutex<f64>>,
}

impl RequestSink for ServoSink {
    fn set_at(&self, print_time: f64, value: f64) -> Option<(FlushAction, f64)> {
        {
            let last = self
                .value
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            if value == *last {
                // Upstream answers "discard", 0. (`servo.py:49-50`).
                return Some((FlushAction::Discard, 0.0));
            }
        }
        let Some(clock) = self.pwm.print_time_to_clock(print_time) else {
            // The MCU went away between arming and flush: keep the change by
            // the immediate form, as `output_pin`'s sink does.
            self.drive_now(value);
            return None;
        };
        let aligned = match self.pwm.next_aligned_clock(clock as u32, RESCHEDULE_SLACK) {
            Ok(aligned) => aligned,
            Err(err) => {
                // The flush callback has no error channel; upstream would
                // raise into the reactor. The frame is lost either way, and
                // `last_value` stays put so a later request retries.
                warn!("SET_SERVO SERVO={}: {err}", self.name);
                return None;
            }
        };
        // `servo.py:51-53`: an alignment more than RESCHEDULE_SLACK after the
        // requested time is rescheduled to the aligned time; the queue
        // retries the request there.
        if let Some(aligned_ptime) = aligned_print_time(&self.pwm, print_time, clock, aligned) {
            if aligned_ptime > print_time + RESCHEDULE_SLACK {
                return Some((FlushAction::Reschedule, aligned_ptime));
            }
        }
        // `servo.py:55-56`: `last_value` moves before the frame is sent.
        *self
            .value
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = value;
        if let Err(err) = self.pwm.set_pwm(aligned, value) {
            warn!("SET_SERVO SERVO={}: {err}", self.name);
        }
        None
    }
}

impl ServoSink {
    /// The immediate fallback: drive without a date and record the duty.
    fn drive_now(&self, value: f64) {
        *self
            .value
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = value;
        if let Err(err) = self.pwm.update_pwm(value) {
            warn!("SET_SERVO SERVO={}: {err}", self.name);
        }
    }
}

/// The print time `aligned` (a clock [`PwmOut::next_aligned_clock`] returned
/// for `print_time`) lands at — upstream's `clock_to_print_time` after
/// `next_aligned_print_time` (`servo.py:51, 55`), for the sink's
/// `reschedule` floor. `None` when the probes disagree (no connected MCU);
/// the sink then sends without a reschedule.
///
/// The `PwmOut` trait maps print time to a clock but exposes neither the
/// inverse nor the frequency (`pins.rs` stays untouched for this wiring), and
/// the mapping is affine — `clock = (print_time − offset) × freq` — so the
/// slope is read off one probe a second later: truncating the two `u64`
/// conversions costs at most a tick over that interval. `delta` is the
/// wrap-safe gap between the two clocks: alignment runs in the 32-bit
/// wire-clock domain, so `aligned` may sit up to `RESCHEDULE_SLACK` *before*
/// `clock` (legitimately early), or just below its `2^32` window.
fn aligned_print_time(
    pwm: &Arc<dyn PwmOut>,
    print_time: f64,
    clock: u64,
    aligned: u32,
) -> Option<f64> {
    let freq = pwm
        .print_time_to_clock(print_time + 1.0)?
        .checked_sub(clock)? as f64;
    if freq <= 0. {
        return None;
    }
    let mut delta = i64::from(aligned) - i64::from(clock as u32);
    if delta < -(1i64 << 31) {
        // The aligned clock wrapped below `clock`'s window: read it as the
        // next one up (the gap is far smaller than a whole window either way).
        delta += 1i64 << 32;
    }
    Some(print_time + delta as f64 / freq)
}

/// The factory `section!` names for each `[servo <name>]`
/// (`servo.py:75 def load_config_prefix`).
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = PrinterServo::new(config, printer)?;
    Ok(Arc::new(object))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{Config, ConfigSection, ConfigValue};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::extras::toolhead::ToolHeadObject;
    use crate::core::klippy::mcu::McuError;
    use crate::core::klippy::pins::{DigitalOut, PinChip, PinError, PinParams};
    use crate::core::klippy::reactor::ManualReactor;

    /// A printer with the given sections loaded (load only, no connect).
    /// A successful load fires `klippy:ready` so the dispatcher runs scripts,
    /// as temperature_fan's tests do.
    fn load(text: &str) -> (Arc<Printer>, Result<(), ConfigError>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = Config::from_text(text).expect("the test config parses").0;
        let result = printer.load_config(&config);
        if result.is_ok() {
            printer.send_event(&KlippyEvent::KlippyReady);
        }
        (printer, result)
    }

    /// The pulse geometry the corpus servo uses: 1 ms … 2 ms over 180°.
    fn geometry() -> Geometry {
        Geometry {
            min_width: 0.001,
            max_width: 0.002,
            max_angle: 180.,
        }
    }

    fn servo_config(extra: &str) -> String {
        format!("[mcu]\nserial: /dev/not-opened-yet\n[servo my_servo]\npin: PH4\n{extra}")
    }

    // -----------------------------------------------------------------------
    // A ready printer over a fake PWM chip, as pwm_tool's tests use: the real
    // MCU's PWM cannot be driven before a connect, the fake records the duty.
    // -----------------------------------------------------------------------

    /// The clock the fake resources map print time through, in Hz.
    const TEST_CLOCK_HZ: f64 = 1_000_000.0;

    /// The schedule floor the fake resources report (the real one comes from
    /// `Mcu::min_schedule_time`).
    const TEST_MIN_SCHEDULE_TIME: f64 = 0.1;

    /// A PWM that records what it was told: clocked changes (with the clock
    /// they went out at) and immediate ones separately, as `output_pin`'s
    /// fake does.
    #[derive(Default)]
    struct FakePwm {
        max_duration: Mutex<f64>,
        cycle_time: Mutex<(f64, bool)>,
        start_value: Mutex<(f64, f64)>,
        updates: Mutex<Vec<f64>>,
        queued: Mutex<Vec<(u32, f64)>>,
        /// The last duty set, for [`FakePwm::next_aligned_clock`] — the real
        /// `McuPwm` aligns against its own last clock and skips alignment
        /// while fully on/off.
        last_value: Mutex<f64>,
        /// Whether the fake models a connected MCU — a clock to convert print
        /// times with and a schedule floor to queue against.
        schedulable: bool,
    }

    impl PwmOut for FakePwm {
        fn setup_max_duration(&self, max_duration: f64) {
            *self.max_duration.lock().unwrap() = max_duration;
        }
        fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) {
            *self.cycle_time.lock().unwrap() = (cycle_time, hardware_pwm);
        }
        fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
            *self.start_value.lock().unwrap() = (start_value, shutdown_value);
            *self.last_value.lock().unwrap() = start_value;
        }
        fn set_pwm(&self, clock: u32, value: f64) -> Result<(), McuError> {
            self.queued.lock().unwrap().push((clock, value));
            *self.last_value.lock().unwrap() = value;
            Ok(())
        }
        fn update_pwm(&self, value: f64) -> Result<(), McuError> {
            self.updates.lock().unwrap().push(value);
            *self.last_value.lock().unwrap() = value;
            Ok(())
        }
        fn next_aligned_clock(&self, clock: u32, allow_early: f64) -> Result<u32, McuError> {
            // Mirrors `McuPwm`: a duty fully on/off has no cycle to land on.
            let (cycle_time, hardware) = *self.cycle_time.lock().unwrap();
            if hardware
                || *self.last_value.lock().unwrap() == 0.0
                || *self.last_value.lock().unwrap() == 1.0
            {
                return Ok(clock);
            }
            let cycle = (cycle_time * TEST_CLOCK_HZ) as u32;
            if cycle == 0 {
                return Ok(clock);
            }
            // Round up to the next cycle boundary, allowed early by
            // `allow_early` (upstream `req_ptime`, `servo.py:51` / `mcu.py:531-543`).
            let early = ((allow_early.min(0.5 * cycle_time)) * TEST_CLOCK_HZ) as u32;
            let req = clock.saturating_sub(early);
            Ok((req + cycle - 1) / cycle * cycle)
        }
        fn print_time_to_clock(&self, print_time: f64) -> Option<u64> {
            self.schedulable
                .then_some((print_time * TEST_CLOCK_HZ) as u64)
        }
        fn min_schedule_time(&self) -> Option<f64> {
            self.schedulable.then_some(TEST_MIN_SCHEDULE_TIME)
        }
    }

    /// A chip that hands out a [`FakePwm`] per setup; its resources model a
    /// connected MCU unless told otherwise.
    struct FakeChip {
        pwms: Mutex<Vec<Arc<FakePwm>>>,
        schedulable: bool,
    }

    impl Default for FakeChip {
        fn default() -> Self {
            Self {
                pwms: Mutex::new(Vec::new()),
                schedulable: true,
            }
        }
    }

    impl PinChip for FakeChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            Err(PinError::Unsupported("digital_out".to_string()))
        }

        fn setup_pwm(&self, _params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
            let pwm = Arc::new(FakePwm {
                schedulable: self.schedulable,
                ..FakePwm::default()
            });
            self.pwms.lock().unwrap().push(Arc::clone(&pwm));
            Ok(pwm)
        }
    }

    /// A ready printer with `gcode` and `pins` over `chip`.
    fn printer_with(chip: FakeChip) -> (Arc<Printer>, Arc<FakeChip>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let pins = Arc::new(PrinterPins::new());
        let chip = Arc::new(chip);
        pins.register_chip("mcu", chip.clone()).unwrap();
        printer.add_object(PINS_OBJECT, pins).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        (printer, chip)
    }

    /// Wrap a hand-built section the way the loader does.
    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    /// The corpus servo built on `printer`: `pin: PA4`, no initial options.
    fn servo_on(printer: &Arc<Printer>) -> PrinterServo {
        let mut section = ConfigSection::new("servo", Some("my_servo"));
        section
            .parameters
            .insert("pin".to_string(), ConfigValue::Single("PA4".to_string()));
        PrinterServo::new(&wrap(&section), printer).unwrap()
    }

    /// A ready printer with `gcode` and `pins` over a schedulable fake chip,
    /// plus the corpus servo built on it.
    fn servo_printer() -> (Arc<Printer>, PrinterServo, Arc<FakeChip>) {
        let (printer, chip) = printer_with(FakeChip::default());
        let servo = servo_on(&printer);
        (printer, servo, chip)
    }

    /// A connected `toolhead` registered on `printer` — `kinematics: none`,
    /// the dwell-only timeline whose flush callbacks must still run
    /// (`toolhead`'s own fixture, as `output_pin`'s tests use it).
    async fn add_toolhead(printer: &Arc<Printer>) -> Arc<ToolHeadObject> {
        let mut section = ConfigSection::new("printer", None);
        for (key, value) in [
            ("kinematics", "none"),
            ("max_velocity", "300"),
            ("max_accel", "3000"),
        ] {
            section
                .parameters
                .insert(key.to_string(), ConfigValue::Single(value.to_string()));
        }
        let object =
            ToolHeadObject::new(&wrap(&section), printer).expect("kinematics: none builds");
        printer.add_object("toolhead", Arc::new(object)).unwrap();
        let object = printer
            .lookup_object_as::<ToolHeadObject>("toolhead")
            .unwrap();
        object.connect().await.expect("the toolhead connects");
        object
    }

    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap()
    }

    fn created(chip: &FakeChip, index: usize) -> Arc<FakePwm> {
        chip.pwms.lock().unwrap()[index].clone()
    }

    /// The section loads: every option it carries (and the defaults it omits)
    /// land in the access record the option check runs on
    /// (`config/validate.rs:44-51`), and `SET_SERVO` registers as a mux
    /// command — the section's whole contract (`servo.py:12-45`).
    #[test]
    fn the_servo_section_reads_its_options_and_registers_set_servo() {
        let (printer, result) = load(&servo_config(""));
        result.unwrap();

        let access = printer.access_tracking();
        for option in [
            "pin",
            "minimum_pulse_width",
            "maximum_pulse_width",
            "maximum_servo_angle",
        ] {
            assert!(
                access.contains("servo my_servo", option),
                "unread option '{option}'"
            );
        }
        // `initial_angle` / `initial_pulse_width` are absent, so nothing to read.
        assert!(!access.contains("servo my_servo", "initial_angle"));

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        assert_eq!(
            gcode.command_help().get("SET_SERVO").map(String::as_str),
            Some("Set servo angle")
        );

        let servo = printer
            .lookup_object_as::<PrinterServo>("servo my_servo")
            .expect("the section registered a servo object");
        assert_eq!(servo.name(), "my_servo");
        // Before any `SET_SERVO`, `value` is 0 as upstream's `last_value`
        // (`servo.py:22`), even though the pin starts at the initial duty.
        assert_eq!(servo.get_status(0.0), json!({ "value": 0.0 }));
    }

    /// The angle/width → duty formulas are upstream's, one for one
    /// (`servo.py:16-24`, `:57-64`).
    #[test]
    fn the_pwm_value_follows_upstreams_pulse_math() {
        let geometry = geometry();
        // duty = width / 0.020 s: the 2 ms maximum pulse is duty 0.1.
        assert!((geometry.pwm_from_angle(180.) - 0.1).abs() < 1e-12);
        assert!((geometry.pwm_from_angle(90.) - 0.075).abs() < 1e-12);
        // Out-of-range angles clamp to the ends.
        assert!((geometry.pwm_from_angle(-5.) - 0.05).abs() < 1e-12);
        assert!((geometry.pwm_from_angle(200.) - 0.1).abs() < 1e-12);
        // Raw widths: clamped into min..=max; zero stays zero.
        assert!((geometry.pwm_from_pulse_width(0.001) - 0.05).abs() < 1e-12);
        assert!((geometry.pwm_from_pulse_width(0.005) - 0.1).abs() < 1e-12);
        assert_eq!(geometry.pwm_from_pulse_width(0.), 0.);
    }

    /// `SET_SERVO` drives the pin through the mux, by angle and by width
    /// (`servo.py:66-73`), and refuses a line with neither parameter.
    #[test]
    fn set_servo_drives_the_pin_through_the_mux() {
        let (printer, servo, chip) = servo_printer();
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap();

        gcode
            .run_script_sync("SET_SERVO SERVO=my_servo angle=160")
            .unwrap();
        let first = geometry().pwm_from_angle(160.);
        assert_eq!(servo.get_status(0.0), json!({ "value": first }));
        let pwm = Arc::clone(&chip.pwms.lock().unwrap()[0]);
        assert_eq!(*pwm.updates.lock().unwrap(), [first]);
        // The pin starts at duty 0 (no initial option) and runs at the
        // servo's fixed period, software PWM (`servo.py:33-36`).
        assert_eq!(
            *pwm.cycle_time.lock().unwrap(),
            (SERVO_SIGNAL_PERIOD, false)
        );
        assert_eq!(*pwm.max_duration.lock().unwrap(), 0.0);

        gcode
            .run_script_sync("SET_SERVO SERVO=my_servo WIDTH=0.0015")
            .unwrap();
        let second = geometry().pwm_from_pulse_width(0.0015);
        assert_eq!(servo.get_status(0.0), json!({ "value": second }));
        assert_eq!(
            *pwm.updates.lock().unwrap(),
            [first, second],
            "each changed duty reaches the pin"
        );

        let err = gcode
            .run_script_sync("SET_SERVO SERVO=my_servo")
            .unwrap_err();
        assert!(err.to_string().contains("missing ANGLE"), "{err}");
    }

    /// A `maximum_pulse_width` at or below `minimum_pulse_width` is refused
    /// with upstream's bound wording (`servo.py:16-18`).
    #[test]
    fn a_pulse_width_below_the_minimum_is_refused() {
        let (_, result) = load(&servo_config("maximum_pulse_width: 0.0005\n"));
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains(
                "Option 'maximum_pulse_width' in section 'servo my_servo' must be above 0.001"
            ),
            "{err}"
        );
    }

    /// `initial_angle` becomes the startup duty, as upstream computes it
    /// before `setup_start_value` (`servo.py:24-26`). The load itself proves
    /// the option is read; the duty is the pure formula above.
    #[test]
    fn an_initial_angle_is_read_and_bounds_checked() {
        let (printer, result) = load(&servo_config("initial_angle: 90\n"));
        result.unwrap();
        assert!(printer
            .access_tracking()
            .contains("servo my_servo", "initial_angle"));

        let (_, result) = load(&servo_config("initial_angle: 400\n"));
        let err = result.unwrap_err().to_string();
        assert!(err.contains("must have maximum of 360"), "{err}");
    }

    // -----------------------------------------------------------------------
    // The queued (print-time) path — see the module docs' Scheduling section
    // -----------------------------------------------------------------------

    /// A queued `SET_SERVO` is pinned to the toolhead's lookahead time
    /// (`register_lookahead_callback`) and lands as a **clocked** `set_pwm`
    /// on the flush — never the immediate `update_pwm` — with the status
    /// moving only when the frame lands.
    #[tokio::test]
    async fn set_servo_is_pinned_to_the_lookahead_time_and_lands_on_the_flush() {
        let (printer, chip) = printer_with(FakeChip::default());
        let toolhead = add_toolhead(&printer).await;
        let servo = servo_on(&printer);

        gcode(&printer)
            .run_script("SET_SERVO SERVO=my_servo ANGLE=90")
            .await
            .unwrap();

        let pwm = created(&chip, 0);
        // The request rides the lookahead/queue; nothing has been driven yet.
        assert!(pwm.queued.lock().unwrap().is_empty());
        assert!(pwm.updates.lock().unwrap().is_empty());
        assert_eq!(servo.get_status(0.0), json!({ "value": 0.0 }));

        let print_time = toolhead.print_time();
        toolhead.flush_step_generation().await.unwrap();

        let duty = geometry().pwm_from_angle(90.);
        assert_eq!(
            *pwm.queued.lock().unwrap(),
            [((print_time * TEST_CLOCK_HZ) as u64 as u32, duty)],
            "the frame carries the lookahead print time as a clock"
        );
        assert!(
            pwm.updates.lock().unwrap().is_empty(),
            "the immediate update_pwm path is not taken"
        );
        assert_eq!(servo.get_status(0.0), json!({ "value": duty }));
    }

    /// A later `SET_SERVO` pushed ahead of the same flush overrides the
    /// earlier one: the queue compresses them and only the covering duty is
    /// sent (`output_pin.py:35-38`).
    #[tokio::test]
    async fn a_later_set_servo_overrides_the_pending_request() {
        let (printer, chip) = printer_with(FakeChip::default());
        let toolhead = add_toolhead(&printer).await;
        let servo = servo_on(&printer);

        gcode(&printer)
            .run_script("SET_SERVO SERVO=my_servo ANGLE=90")
            .await
            .unwrap();
        gcode(&printer)
            .run_script("SET_SERVO SERVO=my_servo WIDTH=0.002")
            .await
            .unwrap();
        toolhead.flush_step_generation().await.unwrap();

        let pwm = created(&chip, 0);
        let print_time = toolhead.print_time();
        assert_eq!(
            *pwm.queued.lock().unwrap(),
            [((print_time * TEST_CLOCK_HZ) as u64 as u32, 0.1)],
            "only the covering duty reaches the pin"
        );
        assert!(pwm.updates.lock().unwrap().is_empty());
        assert_eq!(servo.get_status(0.0), json!({ "value": 0.1 }));
    }

    /// First fork: no `toolhead` object (a config without `[printer]`), even
    /// though the resource could schedule — `SET_SERVO` keeps the immediate
    /// path: driven at once, nothing queued.
    #[test]
    fn without_a_toolhead_set_servo_drives_the_pin_immediately() {
        let (printer, servo, chip) = servo_printer();

        gcode(&printer)
            .run_script_sync("SET_SERVO SERVO=my_servo ANGLE=90")
            .unwrap();

        let pwm = created(&chip, 0);
        assert_eq!(
            *pwm.updates.lock().unwrap(),
            [geometry().pwm_from_angle(90.)]
        );
        assert!(pwm.queued.lock().unwrap().is_empty());
        assert_eq!(
            servo.get_status(0.0),
            json!({ "value": geometry().pwm_from_angle(90.) })
        );
    }

    /// Second fork: a toolhead, but a pin resource whose MCU is not connected
    /// (no schedule floor to queue with) — immediate path, no error, no panic.
    #[tokio::test]
    async fn a_servo_that_cannot_schedule_is_driven_immediately() {
        let (printer, chip) = printer_with(FakeChip {
            schedulable: false,
            ..FakeChip::default()
        });
        add_toolhead(&printer).await;
        let servo = servo_on(&printer);

        gcode(&printer)
            .run_script("SET_SERVO SERVO=my_servo ANGLE=90")
            .await
            .unwrap();

        let pwm = created(&chip, 0);
        assert_eq!(
            *pwm.updates.lock().unwrap(),
            [geometry().pwm_from_angle(90.)]
        );
        assert!(pwm.queued.lock().unwrap().is_empty());
        assert_eq!(
            servo.get_status(0.0),
            json!({ "value": geometry().pwm_from_angle(90.) })
        );
    }

    /// Upstream's `_set_pwm` reschedule (`servo.py:51-53`): an alignment that
    /// would land more than [`RESCHEDULE_SLACK`] late answers `reschedule`
    /// with the aligned time and leaves `last_value` alone; the retry at that
    /// time then drives the pulse exactly on the cycle boundary.
    #[test]
    fn the_sink_reschedules_a_late_alignment_and_drives_on_the_retry() {
        let fake = Arc::new(FakePwm {
            schedulable: true,
            ..FakePwm::default()
        });
        fake.setup_cycle_time(SERVO_SIGNAL_PERIOD, false);
        let value = Arc::new(Mutex::new(0.));
        let sink = ServoSink {
            name: "my_servo".to_string(),
            pwm: Arc::clone(&fake) as Arc<dyn PwmOut>,
            value: Arc::clone(&value),
        };

        // First frame: the duty starts at 0, so there is no cycle to align
        // to (`pwm.rs`/`mcu.py:531-536` filter) — it lands as requested
        // (the fake's clock for 1.000001 s truncates to 1 000 000).
        assert_eq!(sink.set_at(1.000001, 0.05), None);
        assert_eq!(*fake.queued.lock().unwrap(), [(1_000_000, 0.05)]);
        assert_eq!(*value.lock().unwrap(), 0.05);

        // Requested at clock 1 010 000 (mid-cycle): the next boundary is
        // 1 020 000, more than RESCHEDULE_SLACK later — reschedule to it,
        // sending nothing.
        let Some((action, floor)) = sink.set_at(1.010001, 0.075) else {
            panic!("a late alignment must reschedule");
        };
        assert_eq!(action, FlushAction::Reschedule);
        // The probe reads the frequency through two truncating clock
        // conversions, so the floor carries a ~1e-8 s slop.
        assert!((floor - 1.020001).abs() < 1e-7, "floor {floor}");
        assert_eq!(fake.queued.lock().unwrap().len(), 1, "nothing sent yet");
        assert_eq!(*value.lock().unwrap(), 0.05, "last_value waits");

        // The retry at the aligned time passes the slack check and lands on
        // the boundary.
        assert_eq!(sink.set_at(1.020001, 0.075), None);
        assert_eq!(
            *fake.queued.lock().unwrap(),
            [(1_000_000, 0.05), (1_020_000, 0.075)]
        );
        assert_eq!(*value.lock().unwrap(), 0.075);
        assert!(fake.updates.lock().unwrap().is_empty());
    }

    /// The sink discards a repeat of the driven duty without sending
    /// (`servo.py:49-50`), and the discard carries no schedule floor.
    #[test]
    fn the_sink_discards_a_repeat_of_the_driven_duty() {
        let fake = Arc::new(FakePwm {
            schedulable: true,
            ..FakePwm::default()
        });
        fake.setup_cycle_time(SERVO_SIGNAL_PERIOD, false);
        let value = Arc::new(Mutex::new(0.));
        let sink = ServoSink {
            name: "my_servo".to_string(),
            pwm: Arc::clone(&fake) as Arc<dyn PwmOut>,
            value: Arc::clone(&value),
        };

        assert_eq!(sink.set_at(1.0, 0.05), None);
        assert_eq!(sink.set_at(2.0, 0.05), Some((FlushAction::Discard, 0.0)));
        assert_eq!(fake.queued.lock().unwrap().len(), 1);
    }
}
