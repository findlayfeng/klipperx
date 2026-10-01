//! `[fan]` — the printer cooling fan, driven by `M106`/`M107`.
//!
//! Upstream's `klippy/extras/fan.py`: it reads a `pin`, asks `pins` for a PWM,
//! and exposes the fan through `M106 S<0..255>` / `M107`. Half the file is the
//! **shared `Fan` core** — `fan_generic` (H2-2), `heater_fan` (H2-3) and
//! `controller_fan` (H2-4) build the same object and choose the speed some
//! other way — so the core lives here, next to the section that first needs it.
//!
//! | option | meaning |
//! |---|---|
//! | `pin` | the fan's PWM pin, required |
//! | `max_power` | duty ceiling, `0 < .. ≤ 1` (default 1) |
//! | `kick_start_time` | seconds at full power when starting from rest, or when stepping up by more than 0.5 (default 0.1, `≥ 0`) |
//! | `off_below` | a request under this runs the fan off, `0 ..= 1` (default 0) |
//! | `cycle_time` | PWM period in seconds (default 0.010, `> 0`) |
//! | `hardware_pwm` | the firmware's PWM rather than a software one (default false) |
//! | `shutdown_speed` | duty the firmware falls back to when klippy dies (default 0, `0 ..= 1`), capped by `max_power` |
//! | `enable_pin` | optional digital output powering the driver, flipped only on 0 ↔ non-zero |
//! | `tachometer_pin` | optional GPIO the tachometer counts edges on (`tachometer_ppr`, default 2; `tachometer_poll_interval`, default 0.0015) |
//!
//! `M106`'s `S` defaults to 255 and has no upper bound (only `minval=0.`), as
//! upstream's does; `max_power` is what caps the duty that results.
//!
//! # What is not here
//!
//! * **Print-time scheduling.** Upstream queues each change through
//!   `output_pin.GCodeRequestQueue`, so a speed lands at a print time, later
//!   requests override earlier ones, and the kick-start tail is a *re-run* of
//!   the queued request (`output_pin.py:15-73`). That queue needs
//!   `toolhead.register_lookahead_callback` +
//!   `motion_queuing.register_flush_callback` + `Mcu::min_schedule_time`, which
//!   are C1d. This port drives the pin immediately — the same trade
//!   [`output_pin`](crate::core::klippy::extras::output_pin) already makes —
//!   and gives the kick-start tail a reactor timer instead of a queue slot.
//!   When C1d lands, both switch to the queue in one step.
//!
//! The tachometer is not one of the gaps: `tachometer_pin` builds a
//! [`pulse_counter`](crate::core::klippy::extras::pulse_counter) frequency
//! counter, and `get_status` reports `rpm` from it exactly as upstream's
//! `FanTachometer` does — `null` for a section that has no tachometer pin.

use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::pulse_counter::FrequencyCounter;
use crate::core::klippy::gcode::{
    parse_float, sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::pins::{DigitalOut, PrinterPins, PwmOut, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};
use crate::core::klippy::reactor::Reactor;

// Only the bare form (`[fan]`) exists upstream.
section!("fan", order = 20, load = load_config);

/// What the fan is driving, and what was asked of it.
///
/// Upstream's `last_fan_value` / `last_req_value`: the first is the duty on the
/// pin (which during a kick start is the full power), the second is the duty the
/// last request wanted — the one `get_status` reports as `speed`.
#[derive(Debug, Default)]
struct FanState {
    last_fan_value: f64,
    last_req_value: f64,
    /// Generation counter for a kick start still in flight.
    ///
    /// Every request that actually changes the fan bumps it, so the tail of an
    /// older kick start finds a stale serial and does nothing — the same effect
    /// upstream gets by letting a later queued request override the pending one.
    kick_serial: u64,
}

/// Seconds between two tachometer samples (upstream's fixed `sample_time`).
const TACHOMETER_SAMPLE_TIME: f64 = 1.;

/// Upstream's `FanTachometer` (`fan.py:85-106`): the optional pulse counter
/// behind `tachometer_pin`, and the RPM its frequency becomes.
struct FanTachometer {
    /// Pulses per revolution (`tachometer_ppr`), upstream's `self.ppr`.
    ppr: f64,
    /// The frequency counter, when the section has a `tachometer_pin`.
    counter: Option<FrequencyCounter>,
}

impl FanTachometer {
    /// Read the tachometer options and build the counter (`FanTachometer.__init__`).
    ///
    /// # Errors
    /// `tachometer_ppr` below 1, `tachometer_poll_interval` at or below 0, or
    /// any complaint [`FrequencyCounter::new`] makes about the pin.
    fn new(
        config: &ConfigWrapper,
        identifier: &str,
        pins: &PrinterPins,
    ) -> Result<Self, ConfigError> {
        let Some(pin) = config.get_str("tachometer_pin") else {
            // No tachometer pin: upstream keeps the counter at `None` and
            // reports no RPM for this section.
            return Ok(Self {
                ppr: 2.,
                counter: None,
            });
        };
        let ppr = config.get_int_bounded("tachometer_ppr", Some(2), Some(1), None)?;
        let poll_time = config.get_float_bounded(
            "tachometer_poll_interval",
            Some(0.0015),
            None,
            None,
            Some(0.),
            None,
        )?;
        let counter = FrequencyCounter::new(pins, &pin, TACHOMETER_SAMPLE_TIME, poll_time)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        Ok(Self {
            ppr: ppr as f64,
            counter: Some(counter),
        })
    }

    /// Upstream's `FanTachometer.get_status`: no tachometer reads as `null`,
    /// one reads as the frequency scaled into RPM.
    fn rpm(&self) -> Value {
        match &self.counter {
            Some(counter) => json!(to_rpm(counter.get_frequency(), self.ppr)),
            None => Value::Null,
        }
    }
}

/// Upstream's `rpm = self._freq_counter.get_frequency() * 30. / self.ppr`
/// (`fan.py:98`): the frequency of a `tachometer_ppr`-pulse train in RPM.
fn to_rpm(frequency: f64, ppr: f64) -> f64 {
    frequency * 30. / ppr
}

/// The fan core: one PWM pin, its optional enable line, and how to drive them.
///
/// Shared with everything that chooses a speed: the `[fan]` object, its
/// `M106`/`M107` handlers, the `gcode:request_restart` handler, and the
/// kick-start timer.
pub struct Fan {
    reactor: Arc<dyn Reactor>,
    max_power: f64,
    kick_start_time: f64,
    off_below: f64,
    mcu_fan: Arc<dyn PwmOut>,
    enable_pin: Option<Arc<dyn DigitalOut>>,
    /// The optional tachometer behind `tachometer_pin`.
    tachometer: FanTachometer,
    state: Arc<Mutex<FanState>>,
}

impl Fan {
    /// Build the fan from its section, with upstream's option set.
    ///
    /// `default_shutdown_speed` is what upstream passes in: `0.` for `[fan]`
    /// and `fan_generic`, `1.` for `heater_fan` (a hotend fan that keeps
    /// running when klippy dies).
    ///
    /// # Errors
    /// A missing option, one out of range, or a pin that cannot be set up.
    pub fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        default_shutdown_speed: f64,
    ) -> Result<Arc<Self>, ConfigError> {
        let identifier = config.identifier();
        // Upstream: max_power above=0. maxval=1.; kick_start_time minval=0.;
        // off_below minval=0. maxval=1.; cycle_time above=0.
        let max_power =
            config.get_float_bounded("max_power", Some(1.0), None, Some(1.0), Some(0.0), None)?;
        let kick_start_time =
            config.get_float_bounded("kick_start_time", Some(0.1), Some(0.0), None, None, None)?;
        let off_below =
            config.get_float_bounded("off_below", Some(0.0), Some(0.0), Some(1.0), None, None)?;
        let cycle_time =
            config.get_float_bounded("cycle_time", Some(0.010), None, None, Some(0.0), None)?;
        let hardware_pwm = config.get_bool("hardware_pwm", Some(false))?;
        let shutdown_speed = config.get_float_bounded(
            "shutdown_speed",
            Some(default_shutdown_speed),
            Some(0.0),
            Some(1.0),
            None,
            None,
        )?;

        let pin_desc = config.get("pin", None)?;
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let mcu_fan = pins
            .setup_pwm(&pin_desc, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        mcu_fan.setup_max_duration(0.0);
        mcu_fan.setup_cycle_time(cycle_time, hardware_pwm);
        // A fan starts at 0; the shutdown duty is what the firmware falls back
        // to, capped by max_power (`fan.py:26`).
        let shutdown_power = shutdown_speed.clamp(0.0, max_power);
        mcu_fan.setup_start_value(0.0, shutdown_power);

        let enable_pin = match config.get_str("enable_pin") {
            Some(desc) => {
                let pin = pins
                    .setup_digital_out(&desc, None)
                    .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
                pin.setup_max_duration(0.0);
                Some(pin)
            }
            None => None,
        };

        // Upstream builds the tachometer after the pins (`fan.py:41`).
        let tachometer = FanTachometer::new(config, &identifier, pins.as_ref())?;

        let fan = Arc::new(Self {
            reactor: printer.reactor(),
            max_power,
            kick_start_time,
            off_below,
            mcu_fan,
            enable_pin,
            tachometer,
            state: Arc::new(Mutex::new(FanState::default())),
        });

        // Upstream stops every fan when a restart is requested
        // (`fan.py:30-31`), before the new object graph is built.
        let restart = Arc::clone(&fan);
        printer.register_event_handler(
            KlippyEvent::GcodeRequestRestart { print_time: 0.0 },
            Box::new(move |_| {
                let _ = restart.set_speed(0.0);
            }),
        );

        Ok(fan)
    }

    /// Choose the speed: upstream's `Fan.set_speed` (a print-time request).
    ///
    /// With no print-time queue (C1d) this drives the pin now; see the module
    /// docs.
    ///
    /// # Errors
    /// A failed PWM or enable-pin write.
    pub fn set_speed(&self, value: f64) -> Result<(), CommandError> {
        self.apply(value)
    }

    /// Choose the speed from a g-code line: upstream's
    /// `Fan.set_speed_from_command`, which waits for the toolhead's lookahead.
    ///
    /// Both entry points land in `Fan::apply` today, for the reason
    /// [`set_speed`](Fan::set_speed) gives.
    ///
    /// # Errors
    /// A failed PWM or enable-pin write.
    pub fn set_speed_from_command(&self, value: f64) -> Result<(), CommandError> {
        self.apply(value)
    }

    /// Drive the pin to `requested` (upstream `Fan._apply_speed`).
    ///
    /// `off_below` snaps small requests off, `max_power` caps them, an
    /// unchanged request is dropped, the enable line only moves on a 0 ↔
    /// non-zero transition, and a start from rest (or a step up of more than
    /// 0.5) runs at full power for `kick_start_time` first.
    ///
    /// # Errors
    /// A failed PWM or enable-pin write.
    fn apply(&self, requested: f64) -> Result<(), CommandError> {
        let mut state = self.lock();
        let requested = if requested < self.off_below {
            0.0
        } else {
            requested
        };
        let value = (requested * self.max_power).clamp(0.0, self.max_power);

        if value == state.last_fan_value {
            // Same as what is already driven: upstream drops it ("discard")
            // without touching any of the state below — including a kick start
            // still in flight, which keeps its tail and lands the value then.
            return Ok(());
        }

        // This request supersedes a kick start whose tail has not run yet.
        state.kick_serial = state.kick_serial.wrapping_add(1);
        let serial = state.kick_serial;

        if let Some(pin) = &self.enable_pin {
            if value > 0.0 && state.last_fan_value == 0.0 {
                pin.update_digital_out(true).map_err(mcu_error)?;
            } else if value == 0.0 && state.last_fan_value > 0.0 {
                pin.update_digital_out(false).map_err(mcu_error)?;
            }
        }

        if value > 0.0
            && self.kick_start_time > 0.0
            && (state.last_fan_value == 0.0 || value - state.last_fan_value > 0.5)
        {
            // Full power now, the requested duty after `kick_start_time`.
            // Upstream re-runs the queued request ("repeat") at that point;
            // without a queue the tail is a reactor timer — see module docs.
            state.last_req_value = value;
            state.last_fan_value = self.max_power;
            self.mcu_fan.update_pwm(self.max_power).map_err(mcu_error)?;
            let tail = KickTail {
                mcu_fan: Arc::clone(&self.mcu_fan),
                state: Arc::clone(&self.state),
            };
            let delay = self.kick_start_time;
            // Not holding the lock while registering: a dispatcher on another
            // thread could run the timer and wait on `state`.
            drop(state);
            self.reactor
                .call_later(delay, Box::new(move |_| tail.run(serial, value)));
            return Ok(());
        }

        state.last_fan_value = value;
        state.last_req_value = value;
        self.mcu_fan.update_pwm(value).map_err(mcu_error)
    }

    /// Upstream's `Fan.get_status`.
    pub fn get_status(&self, _eventtime: f64) -> Value {
        json!({
            "speed": self.lock().last_req_value,
            // Upstream reports `None` for a section without `tachometer_pin`
            // (`fan.py:99-102`); one that has a counter reports its RPM.
            "rpm": self.tachometer.rpm(),
        })
    }

    fn lock(&self) -> MutexGuard<'_, FanState> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl std::fmt::Debug for Fan {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Fan")
            .field("max_power", &self.max_power)
            .field("kick_start_time", &self.kick_start_time)
            .field("off_below", &self.off_below)
            .finish_non_exhaustive()
    }
}

/// The kick-start tail: settle the fan from full power to the duty requested.
///
/// Held by the reactor timer alone, so it carries what it needs rather than a
/// reference back to the section.
struct KickTail {
    mcu_fan: Arc<dyn PwmOut>,
    state: Arc<Mutex<FanState>>,
}

impl KickTail {
    fn run(&self, serial: u64, value: f64) {
        let mut state = self
            .state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        if state.kick_serial != serial {
            // A newer request took over while the kick was running.
            return;
        }
        // This tail has fired; anything after it is a fresh request.
        state.kick_serial = state.kick_serial.wrapping_add(1);
        if value == state.last_fan_value {
            // The requested duty is what full power already means (a fan with
            // `max_power: 1` asked for full speed): upstream discards here too.
            return;
        }
        state.last_fan_value = value;
        state.last_req_value = value;
        let _ = self.mcu_fan.update_pwm(value);
    }
}

/// An MCU write failure, as the command error it is reported through.
fn mcu_error(err: crate::core::klippy::mcu::McuError) -> CommandError {
    CommandError::new(err.to_string())
}

/// One `[fan]`, as upstream's `PrinterFan`.
pub struct PrinterFan {
    fan: Arc<Fan>,
}

impl PrinterFan {
    /// Build the fan and register `M106`/`M107`.
    ///
    /// # Errors
    /// As [`Fan::new`], or a g-code registration failure.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let fan = Fan::new(config, printer, 0.0)?;
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");

        // `M106 S<value>`: default 255, `minval=0.`, no upper bound — what
        // caps the duty is `max_power`, inside `_apply_speed` (`fan.py:117-120`).
        let speed = Arc::clone(&fan);
        let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
            let value =
                gcmd.get("S", Some(255.0), parse_float, Some(0.0), None, None, None)? / 255.;
            speed.set_speed_from_command(value)
        });
        gcode
            .register_command_with_params("M106", handler, None, &["S"], false)
            .map_err(ConfigError::new)?;

        let off = Arc::clone(&fan);
        let handler: CommandHandler =
            sync(move |_gcmd: &GcodeCommand| off.set_speed_from_command(0.));
        gcode
            .register_command("M107", handler, None, false)
            .map_err(ConfigError::new)?;

        Ok(Self { fan })
    }

    /// The fan this section drives; its `M106`/`M107` handlers share it.
    pub fn fan(&self) -> &Arc<Fan> {
        &self.fan
    }
}

impl PrinterObject for PrinterFan {
    fn get_status(&self, eventtime: f64) -> Value {
        self.fan.get_status(eventtime)
    }
}

impl std::fmt::Debug for PrinterFan {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterFan").finish_non_exhaustive()
    }
}

/// The factory `section!` names.
pub(crate) fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PrinterFan::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{ConfigSection, ConfigValue};
    use crate::core::klippy::mcu::{ConfigBuilder, McuChip, McuError};
    use crate::core::klippy::pins::{PinChip, PinError, PinParams};
    use crate::core::klippy::reactor::ManualReactor;

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

    /// A digital output that records what it was told.
    #[derive(Default)]
    struct FakeDigitalOut {
        max_duration: Mutex<f64>,
        updates: Mutex<Vec<bool>>,
    }

    impl DigitalOut for FakeDigitalOut {
        fn setup_max_duration(&self, max_duration: f64) {
            *self.max_duration.lock().unwrap() = max_duration;
        }
        fn setup_start_value(&self, _start_value: bool, _shutdown_value: bool) {}
        fn queue_digital_out(&self, _clock: u32, _value: bool) -> Result<(), McuError> {
            Ok(())
        }
        fn update_digital_out(&self, value: bool) -> Result<(), McuError> {
            self.updates.lock().unwrap().push(value);
            Ok(())
        }
    }

    /// A chip that hands out a [`FakePwm`] or [`FakeDigitalOut`] per setup.
    #[derive(Default)]
    struct FakeChip {
        pwms: Mutex<Vec<Arc<FakePwm>>>,
        digital: Mutex<Vec<Arc<FakeDigitalOut>>>,
    }

    impl PinChip for FakeChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            let out = Arc::new(FakeDigitalOut::default());
            self.digital.lock().unwrap().push(Arc::clone(&out));
            Ok(out)
        }

        fn setup_pwm(&self, _params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
            let pwm = Arc::new(FakePwm::default());
            self.pwms.lock().unwrap().push(Arc::clone(&pwm));
            Ok(pwm)
        }
    }

    /// A ready printer with `gcode` and `pins` over a fake chip, plus the
    /// reactor behind it — a test that runs the kick-start tail advances it.
    fn printer() -> (Arc<Printer>, Arc<FakeChip>, Arc<ManualReactor>) {
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(reactor.clone()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let pins = Arc::new(PrinterPins::new());
        let chip = Arc::new(FakeChip::default());
        pins.register_chip("mcu", chip.clone()).unwrap();
        // A `tachometer_pin` needs a real MCU chip — the counter takes its oid
        // there (`pulse_counter`) — while the fan's own pins stay on the fake.
        pins.register_chip(
            "counter",
            Arc::new(McuChip::new(
                "counter".to_string(),
                Arc::new(ConfigBuilder::new()),
                Arc::clone(&pins),
            )),
        )
        .unwrap();
        printer.add_object(PINS_OBJECT, pins).unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        (printer, chip, reactor)
    }

    /// A `[fan]` section with `pin: <pin>` plus `options`.
    fn section(pin: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("fan", None);
        section
            .parameters
            .insert("pin".to_string(), ConfigValue::Single(pin.to_string()));
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap()
    }

    fn pwm(chip: &FakeChip, index: usize) -> Arc<FakePwm> {
        chip.pwms.lock().unwrap()[index].clone()
    }

    fn updates(pwm: &FakePwm) -> Vec<f64> {
        pwm.updates.lock().unwrap().clone()
    }

    fn speed(fan: &PrinterFan) -> f64 {
        fan.get_status(0.0)["speed"]
            .as_f64()
            .expect("speed is a number")
    }

    #[test]
    fn test_a_fan_is_configured_with_upstream_defaults() {
        let (printer, chip, _reactor) = printer();
        PrinterFan::new(&wrap(&section("PA1", &[])), &printer).unwrap();

        let fan = pwm(&chip, 0);
        assert_eq!(*fan.max_duration.lock().unwrap(), 0.0);
        assert_eq!(*fan.cycle_time.lock().unwrap(), (0.010, false));
        // Off at start, and the firmware's shutdown duty is 0 for `[fan]`.
        assert_eq!(*fan.start_value.lock().unwrap(), (0.0, 0.0));
    }

    #[test]
    fn test_shutdown_speed_is_capped_by_max_power() {
        let (printer, chip, _reactor) = printer();
        PrinterFan::new(
            &wrap(&section(
                "PA1",
                &[("max_power", "0.8"), ("shutdown_speed", "1.0")],
            )),
            &printer,
        )
        .unwrap();

        assert_eq!(*pwm(&chip, 0).start_value.lock().unwrap(), (0.0, 0.8));
    }

    #[test]
    fn test_m106_sets_the_speed_and_m107_turns_it_off() {
        let (printer, chip, _reactor) = printer();
        // `kick_start_time: 0` keeps this test about the plain path.
        let fan = PrinterFan::new(
            &wrap(&section("PA1", &[("kick_start_time", "0")])),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script_sync("M106 S128").unwrap();
        assert_eq!(updates(&pwm(&chip, 0)), [128.0 / 255.0]);
        assert!((speed(&fan) - 128.0 / 255.0).abs() < 1e-9);

        // M106 without S is full speed (default 255).
        gcode(&printer).run_script_sync("M106").unwrap();
        assert_eq!(updates(&pwm(&chip, 0)), [128.0 / 255.0, 1.0]);
        assert_eq!(speed(&fan), 1.0);

        gcode(&printer).run_script_sync("M107").unwrap();
        assert_eq!(updates(&pwm(&chip, 0)), [128.0 / 255.0, 1.0, 0.0]);
        assert_eq!(speed(&fan), 0.0);
        assert!(fan.get_status(0.0)["rpm"].is_null());
    }

    #[test]
    fn test_m106_rejects_a_negative_speed() {
        let (printer, _chip, _reactor) = printer();
        PrinterFan::new(&wrap(&section("PA1", &[])), &printer).unwrap();

        let err = gcode(&printer).run_script_sync("M106 S-1").unwrap_err();

        assert!(err.to_string().contains("minimum of 0"), "{err}");
    }

    #[test]
    fn test_kick_start_runs_at_full_power_then_settles() {
        let (printer, chip, reactor) = printer();
        // Half power, so the kick (full *configured* power) is distinguishable
        // from the duty that was asked for.
        let fan = PrinterFan::new(
            &wrap(&section(
                "PA1",
                &[("max_power", "0.5"), ("kick_start_time", "0.1")],
            )),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script_sync("M106 S128").unwrap();
        let target = 128.0 / 255.0 * 0.5;
        assert_eq!(updates(&pwm(&chip, 0)), [0.5], "starts at full power");
        // Until the kick runs out, the *requested* speed is what is reported.
        assert!((speed(&fan) - target).abs() < 1e-9);

        reactor.advance(0.1);
        let driven = updates(&pwm(&chip, 0));
        assert_eq!(
            driven.len(),
            2,
            "the tail writes the requested duty: {driven:?}"
        );
        assert!((driven[1] - target).abs() < 1e-9, "{driven:?}");
        assert!((speed(&fan) - target).abs() < 1e-9);
    }

    #[test]
    fn test_a_new_request_supersedes_a_pending_kick() {
        let (printer, chip, reactor) = printer();
        PrinterFan::new(
            &wrap(&section("PA1", &[("kick_start_time", "0.1")])),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script_sync("M106 S128").unwrap();
        assert_eq!(updates(&pwm(&chip, 0)), [1.0], "kicking at full power");

        // A second request before the kick runs out owns the fan from here.
        gcode(&printer).run_script_sync("M106 S64").unwrap();
        reactor.advance(0.1);
        assert_eq!(
            updates(&pwm(&chip, 0)),
            [1.0, 64.0 / 255.0],
            "the stale tail must not write the first request's duty"
        );
    }

    #[test]
    fn test_off_below_snaps_a_small_request_to_zero() {
        let (printer, chip, _reactor) = printer();
        let fan = PrinterFan::new(
            &wrap(&section(
                "PA1",
                &[("kick_start_time", "0"), ("off_below", "0.2")],
            )),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script_sync("M106 S5").unwrap();
        assert!(updates(&pwm(&chip, 0)).is_empty(), "5/255 is below 0.2");
        assert_eq!(speed(&fan), 0.0);

        gcode(&printer).run_script_sync("M106 S200").unwrap();
        assert_eq!(updates(&pwm(&chip, 0)), [200.0 / 255.0]);
    }

    #[test]
    fn test_max_power_caps_the_speed() {
        let (printer, chip, _reactor) = printer();
        let fan = PrinterFan::new(
            &wrap(&section(
                "PA1",
                &[("kick_start_time", "0"), ("max_power", "0.5")],
            )),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script_sync("M106").unwrap();
        assert_eq!(updates(&pwm(&chip, 0)), [0.5]);
        assert_eq!(speed(&fan), 0.5);
    }

    #[test]
    fn test_the_enable_pin_moves_only_on_off_to_on() {
        let (printer, chip, _reactor) = printer();
        PrinterFan::new(
            &wrap(&section(
                "PA1",
                &[
                    ("kick_start_time", "0"),
                    ("enable_pin", "PB0"),
                    ("max_power", "1.0"),
                ],
            )),
            &printer,
        )
        .unwrap();
        let enable = chip.digital.lock().unwrap()[0].clone();
        assert_eq!(*enable.max_duration.lock().unwrap(), 0.0);

        gcode(&printer).run_script_sync("M106 S128").unwrap();
        gcode(&printer).run_script_sync("M106 S255").unwrap();
        gcode(&printer).run_script_sync("M107").unwrap();

        // On, then a change of speed (both non-zero: no edge), then off.
        assert_eq!(*enable.updates.lock().unwrap(), [true, false]);
        assert_eq!(
            updates(&pwm(&chip, 0)),
            [128.0 / 255.0, 1.0, 0.0],
            "the fan itself follows every request"
        );
    }

    #[test]
    fn test_a_restart_request_stops_the_fan() {
        let (printer, chip, _reactor) = printer();
        PrinterFan::new(
            &wrap(&section("PA1", &[("kick_start_time", "0")])),
            &printer,
        )
        .unwrap();

        gcode(&printer).run_script_sync("M106").unwrap();
        printer.send_event(&KlippyEvent::GcodeRequestRestart { print_time: 0.0 });

        assert_eq!(updates(&pwm(&chip, 0)), [1.0, 0.0]);
    }

    #[test]
    fn test_the_tachometer_pin_builds_a_counter_and_reports_zero_rpm() {
        let (printer, _chip, _reactor) = printer();

        let fan = PrinterFan::new(
            &wrap(&section("PA1", &[("tachometer_pin", "counter:PC0")])),
            &printer,
        )
        .unwrap();

        // No edge has been counted yet: 0 Hz scales to 0 RPM, which is a
        // number — upstream reports `None` only for a section with no
        // tachometer pin at all.
        assert_eq!(fan.get_status(0.0)["rpm"], json!(0.0));
    }

    #[test]
    fn test_the_tachometer_pin_may_carry_a_pull_up() {
        let (printer, _chip, _reactor) = printer();

        PrinterFan::new(
            &wrap(&section("PA1", &[("tachometer_pin", "^counter:PC0")])),
            &printer,
        )
        .unwrap();
    }

    #[test]
    fn test_a_tachometer_ppr_below_one_is_refused() {
        let (printer, _chip, _reactor) = printer();

        let err = PrinterFan::new(
            &wrap(&section(
                "PA1",
                &[("tachometer_pin", "counter:PC0"), ("tachometer_ppr", "0")],
            )),
            &printer,
        )
        .unwrap_err();

        assert!(err.to_string().contains("tachometer_ppr"), "{err}");
    }

    #[test]
    fn test_a_tachometer_poll_interval_must_be_above_zero() {
        let (printer, _chip, _reactor) = printer();

        let err = PrinterFan::new(
            &wrap(&section(
                "PA1",
                &[
                    ("tachometer_pin", "counter:PC0"),
                    ("tachometer_poll_interval", "0"),
                ],
            )),
            &printer,
        )
        .unwrap_err();

        assert!(err.to_string().contains("must be above 0"), "{err}");
    }

    #[test]
    fn test_rpm_is_the_frequency_scaled_by_the_ppr() {
        // 60 Hz over two pulses per revolution is 30 revolutions per second.
        assert_eq!(to_rpm(60., 2.), 900.);
        assert_eq!(to_rpm(0., 4.), 0.);
    }

    #[test]
    fn test_a_missing_pin_names_the_section() {
        let (printer, _chip, _reactor) = printer();

        let err = PrinterFan::new(&wrap(&ConfigSection::new("fan", None)), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'pin' in section 'fan' must be specified"
        );
    }

    #[test]
    fn test_an_option_out_of_range_says_which_bound() {
        let (printer, _chip, _reactor) = printer();

        let err =
            PrinterFan::new(&wrap(&section("PA1", &[("max_power", "0")])), &printer).unwrap_err();
        assert!(err.to_string().contains("must be above 0"), "{err}");

        let err = PrinterFan::new(
            &wrap(&section("PA1", &[("kick_start_time", "-0.1")])),
            &printer,
        )
        .unwrap_err();
        assert!(err.to_string().contains("minimum of 0"), "{err}");

        let err =
            PrinterFan::new(&wrap(&section("PA1", &[("cycle_time", "0")])), &printer).unwrap_err();
        assert!(err.to_string().contains("cycle_time"), "{err}");
    }

    #[test]
    fn test_an_unparseable_number_is_reported() {
        let (printer, _chip, _reactor) = printer();

        let err = PrinterFan::new(&wrap(&section("PA1", &[("off_below", "half")])), &printer)
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Unable to parse option 'off_below' in section 'fan'"
        );
    }
}
