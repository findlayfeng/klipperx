//! `[servo <name>]` — a hobby servo driven by a PWM pin
//! (upstream `klippy/extras/servo.py`).
//!
//! The section parses the pulse geometry upstream's `PrinterServo.__init__`
//! reads, builds the PWM output on the configured pin, and registers
//! `SET_SERVO` as a mux command keyed by `SERVO` (`servo.py:40-44`).
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
//! # Gaps this port does not close yet
//!
//! * **No print-time scheduling.** Upstream queues each update through
//!   `output_pin.GCodeRequestQueue` and aligns it to the next MCU cycle
//!   (`servo.py:47-61`, `RESCHEDULE_SLACK`); this port drives the pin through
//!   the resource's immediate path, like [`pwm_tool`](super::pwm_tool). The
//!   duty value is upstream's formula, so what is missing is *when* the pulse
//!   lands, not what it is.

use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::pins::{PrinterPins, PwmOut, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form (`[servo <name>]`) exists upstream (`servo.py:75`).
section!("servo", order = 20, prefix = load_config_prefix);

/// The PWM period every servo runs at (`servo.py:7`).
const SERVO_SIGNAL_PERIOD: f64 = 0.020;

/// The pulse geometry of one servo: how angles and widths map to a duty
/// fraction of [`SERVO_SIGNAL_PERIOD`] (`servo.py:16-24`, `:56-63`).
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
    /// onto `min_width ..= max_width` (`servo.py:56-59`).
    fn pwm_from_angle(&self, angle: f64) -> f64 {
        let angle = angle.max(0.).min(self.max_angle);
        let width = self.min_width + angle * self.angle_to_width();
        Self::width_to_value(width)
    }

    /// The duty for a raw pulse width: a zero width stays zero, anything else
    /// is clamped into `min_width ..= max_width` (`servo.py:60-63`).
    fn pwm_from_pulse_width(&self, width: f64) -> f64 {
        let width = if width != 0. {
            width.max(self.min_width).min(self.max_width)
        } else {
            0.
        };
        Self::width_to_value(width)
    }

    /// Millimetres of width one degree of angle adds (`servo.py:17`).
    fn angle_to_width(&self) -> f64 {
        (self.max_width - self.min_width) / self.max_angle
    }
}

/// One configured `[servo <name>]`.
pub struct PrinterServo {
    /// The name `SET_SERVO SERVO=<name>` addresses it by: the section's sub.
    name: String,
    /// The duty last sent, for `get_status` (`servo.py:45-46`); starts at 0
    /// as upstream's `last_value` does, regardless of the startup pulse.
    value: Arc<Mutex<f64>>,
}

impl PrinterServo {
    /// Read the section, build the PWM pin, and register `SET_SERVO`.
    ///
    /// # Errors
    /// Returns a config error (a message naming the section) when the section
    /// has no name, an option is missing or out of bounds, or the pin cannot
    /// be built.
    pub fn new(config: &ConfigWrapper, printer: &Printer) -> Result<Self, ConfigError> {
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
        let value_slot = Arc::new(Mutex::new(0.));
        let pwm = Arc::new(pwm);
        let handler: CommandHandler = {
            let pwm = Arc::clone(&pwm);
            let value_slot = Arc::clone(&value_slot);
            sync(move |gcmd| cmd_set_servo(&pwm, &value_slot, geometry, gcmd))
        };
        gcode
            .register_mux_command(
                "SET_SERVO",
                "SERVO",
                Some(&name),
                handler,
                Some("Set servo angle"),
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        Ok(Self {
            name,
            value: value_slot,
        })
    }

    /// The name `SET_SERVO SERVO=<name>` addresses this servo by.
    pub fn name(&self) -> &str {
        &self.name
    }

    fn lock(&self) -> MutexGuard<'_, f64> {
        self.value
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl PrinterObject for PrinterServo {
    /// The duty last sent, as upstream's `PrinterServo.get_status`
    /// (`servo.py:45-46`).
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "value": *self.lock() })
    }
}

impl std::fmt::Debug for PrinterServo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterServo")
            .field("name", &self.name)
            .finish_non_exhaustive()
    }
}

/// `SET_SERVO SERVO=<name> ANGLE=<a>` or `WIDTH=<seconds>`: drive the pin.
///
/// `WIDTH` wins when present; otherwise `ANGLE` is required
/// (`servo.py:64-71`). A repeat of the current duty sends nothing — upstream's
/// `GCodeRequestQueue` reaches the same discard in `_set_pwm` (`servo.py:50`).
fn cmd_set_servo(
    pwm: &Arc<dyn PwmOut>,
    value_slot: &Arc<Mutex<f64>>,
    geometry: Geometry,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let value = if gcmd.get_command_parameters().contains_key("WIDTH") {
        geometry.pwm_from_pulse_width(gcmd.get_float("WIDTH")?)
    } else {
        geometry.pwm_from_angle(gcmd.get_float("ANGLE")?)
    };
    let current = *value_slot
        .lock()
        .unwrap_or_else(|poison| poison.into_inner());
    if value == current {
        return Ok(());
    }
    pwm.update_pwm(value)
        .map_err(|err| CommandError::new(err.to_string()))?;
    *value_slot
        .lock()
        .unwrap_or_else(|poison| poison.into_inner()) = value;
    Ok(())
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

    /// A PWM that records what it was told.
    #[derive(Default)]
    struct FakePwm {
        max_duration: Mutex<f64>,
        cycle_time: Mutex<(f64, bool)>,
        start_value: Mutex<(f64, f64)>,
        updates: Mutex<Vec<f64>>,
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
        }
        fn set_pwm(&self, _clock: u32, value: f64) -> Result<(), McuError> {
            self.updates.lock().unwrap().push(value);
            Ok(())
        }
        fn update_pwm(&self, value: f64) -> Result<(), McuError> {
            self.updates.lock().unwrap().push(value);
            Ok(())
        }
        fn next_aligned_clock(&self, clock: u32, _allow_early: f64) -> Result<u32, McuError> {
            Ok(clock)
        }
    }

    /// A chip that hands out a [`FakePwm`] per setup.
    #[derive(Default)]
    struct FakeChip {
        pwms: Mutex<Vec<Arc<FakePwm>>>,
    }

    impl PinChip for FakeChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            Err(PinError::Unsupported("digital_out".to_string()))
        }

        fn setup_pwm(&self, _params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
            let pwm = Arc::new(FakePwm::default());
            self.pwms.lock().unwrap().push(Arc::clone(&pwm));
            Ok(pwm)
        }
    }

    /// A ready printer with `gcode` and `pins` over the fake chip, plus the
    /// corpus servo built on it.
    fn servo_printer() -> (Arc<Printer>, PrinterServo, Arc<FakeChip>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let pins = Arc::new(PrinterPins::new());
        let chip = Arc::new(FakeChip::default());
        pins.register_chip("mcu", chip.clone()).unwrap();
        printer.add_object(PINS_OBJECT, pins).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);

        let mut section = ConfigSection::new("servo", Some("my_servo"));
        section
            .parameters
            .insert("pin".to_string(), ConfigValue::Single("PA4".to_string()));
        let access = crate::core::klippy::config::AccessTracking::shared();
        let wrapper = ConfigWrapper::new(&section, access);
        let servo = PrinterServo::new(&wrapper, &printer).unwrap();
        (printer, servo, chip)
    }

    /// The section loads: every option it carries (and the defaults it omits)
    /// land in the access record the option check runs on
    /// (`config/validate.rs:44-51`), and `SET_SERVO` registers as a mux
    /// command — the section's whole contract (`servo.py:12-44`).
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
        // (`servo.py:20`), even though the pin starts at the initial duty.
        assert_eq!(servo.get_status(0.0), json!({ "value": 0.0 }));
    }

    /// The angle/width → duty formulas are upstream's, one for one
    /// (`servo.py:16-24`, `:56-63`).
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
    /// (`servo.py:64-71`), and refuses a line with neither parameter.
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
        // servo's fixed period, software PWM (`servo.py:33-37`).
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
    /// with upstream's bound wording (`servo.py:14-16`).
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
}
