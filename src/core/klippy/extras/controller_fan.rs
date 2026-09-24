//! `[controller_fan <name>]` — a fan that cools the controller board.
//!
//! Upstream's `klippy/extras/controller_fan.py`: the section is a
//! [`Fan`](crate::core::klippy::extras::fan) core whose speed is chosen once a
//! second — full `fan_speed` while any stepper it was given (or any stepper at
//! all) is enabled or any named heater has a target, `idle_speed` for
//! `idle_timeout` seconds after that, then off.
//!
//! | option | meaning |
//! |---|---|
//! | `pin` | the fan's PWM pin, required (via the `Fan` core) |
//! | `stepper` | steppers whose motion keeps the fan on (default: every stepper) |
//! | `heater` | heaters whose target keeps the fan on (default: `extruder`) |
//! | `fan_speed` | duty while active (default 1, `0 ..= 1`) |
//! | `idle_speed` | duty during the idle window (default: `fan_speed`, `0 ..= 1`) |
//! | `idle_timeout` | seconds of idle before the fan stops (default 30, `≥ 0`) |
//!
//! The per-second timer starts at `klippy:ready` (`PIN_MIN_TIME` after the
//! monotonic clock, then one tick per second), and the `stepper`/`heater`
//! references are resolved in `connect` — upstream's `handle_connect`, with
//! its error wording.
//!
//! # What is not here
//!
//! * **Heater names are object names.** Upstream looks heaters up in
//!   `heaters.lookup_heater`, which keys them by their *short* name
//!   (`extruder`, or a `[heater_generic <name>]`'s `<name>`); this port has no
//!   such registry yet (H1) and resolves against the printer object registry
//!   instead, so a `heater: <name>` must name an object. The corpus default
//!   (`extruder`) resolves either way.

use std::sync::{Arc, Mutex, Weak};

use serde_json::Value;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::fan::Fan;
use crate::core::klippy::extras::heaters;
use crate::core::klippy::extras::stepper_enable::PrinterStepperEnable;
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};
use crate::core::klippy::reactor::TimerHandle;

// Only the prefix form (`[controller_fan <name>]`) exists upstream
// (`controller_fan.py:68`).
section!("controller_fan", order = 30, prefix = load_config_prefix);

/// How long after `klippy:ready` the first check runs (`PIN_MIN_TIME`), and
/// the tick period that follows (upstream's callback returns `eventtime + 1.`).
const PIN_MIN_TIME: f64 = 0.100;
const TICK_PERIOD: f64 = 1.0;

/// One `[controller_fan <name>]`.
pub struct ControllerFan {
    fan: Arc<Fan>,
    /// The `stepper` option as configured; `None` means every stepper.
    configured_steppers: Option<Vec<String>>,
    /// The steppers to watch, resolved in `connect`.
    steppers: Mutex<Vec<String>>,
    /// The objects behind the `heater` option, resolved in `connect`.
    heaters: Mutex<Vec<Arc<dyn PrinterObject>>>,
    heater_names: Vec<String>,
    fan_speed: f64,
    idle_speed: f64,
    idle_timeout: i64,
    /// Seconds since the fan was last active (upstream's `last_on`, starting
    /// at `idle_timeout` so a fresh machine does not open the idle window).
    last_on: Mutex<i64>,
    last_speed: Mutex<f64>,
    stepper_enable: Arc<PrinterStepperEnable>,
    printer: WeakPrinter,
    /// The handle the `klippy:ready` handler creates, cancelled on drop.
    timer: Mutex<Option<TimerHandle>>,
    self_ref: Weak<ControllerFan>,
}

/// A weak handle to the printer, without importing `std::sync::Weak` twice.
type WeakPrinter = std::sync::Weak<Printer>;

impl ControllerFan {
    /// Read the section and build the fan (`ControllerFan.__init__`).
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        // Upstream reads `stepper` before the objects it loads.
        let configured_steppers = config.get_list("stepper", ',');
        let stepper_enable = PrinterStepperEnable::ensure(printer);
        heaters::ensure(printer)?;
        // `default_shutdown_speed` stays 0 for a controller fan.
        let fan = Fan::new(config, printer, 0.0)?;
        let fan_speed =
            config.get_float_bounded("fan_speed", Some(1.0), Some(0.0), Some(1.0), None, None)?;
        let idle_speed = config.get_float_bounded(
            "idle_speed",
            Some(fan_speed),
            Some(0.0),
            Some(1.0),
            None,
            None,
        )?;
        let idle_timeout = config.get_int_bounded("idle_timeout", Some(30), Some(0), None)?;
        // Upstream's default for `getlist("heater", ("extruder",))`.
        let heater_names = config
            .get_list("heater", ',')
            .unwrap_or_else(|| vec!["extruder".to_string()]);

        Ok(Arc::new_cyclic(|weak| Self {
            fan,
            configured_steppers,
            steppers: Mutex::new(Vec::new()),
            heaters: Mutex::new(Vec::new()),
            heater_names,
            fan_speed,
            idle_speed,
            idle_timeout,
            last_on: Mutex::new(idle_timeout),
            last_speed: Mutex::new(0.0),
            stepper_enable,
            printer: Arc::downgrade(printer),
            timer: Mutex::new(None),
            self_ref: weak.clone(),
        }))
    }

    /// Resolve `stepper` and `heater` names (`ControllerFan.handle_connect`).
    ///
    /// # Errors
    /// An unknown stepper or heater, with upstream's wording.
    fn resolve(&self) -> Result<(), ConfigError> {
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| ConfigError::new("the printer is gone".to_string()))?;

        let mut heaters = Vec::with_capacity(self.heater_names.len());
        for name in &self.heater_names {
            let heater = printer.lookup_object(name).ok_or_else(|| {
                // Upstream's `heaters.lookup_heater` (`heaters.py:288-292`).
                ConfigError::new(format!("Unknown heater '{name}'"))
            })?;
            heaters.push(heater);
        }
        *self.heaters.lock().unwrap_or_else(|p| p.into_inner()) = heaters;

        let all = self.stepper_enable.get_steppers();
        let resolved = match &self.configured_steppers {
            None => all,
            Some(names) => {
                if !names.iter().all(|name| all.contains(name)) {
                    let listed = names
                        .iter()
                        .map(|name| format!("'{name}'"))
                        .collect::<Vec<_>>()
                        .join(", ");
                    return Err(ConfigError::new(format!(
                        "One or more of these steppers are unknown: [{listed}] \
                         (valid steppers are: {})",
                        all.join(", ")
                    )));
                }
                names.clone()
            }
        };
        *self.steppers.lock().unwrap_or_else(|p| p.into_inner()) = resolved;
        Ok(())
    }

    /// Start the per-second check (`ControllerFan.handle_ready`).
    fn start_timer(&self) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        let reactor = printer.reactor();
        let weak = self.self_ref.clone();
        let handle = reactor.register_timer_named(
            "controller_fan",
            Box::new(move |eventtime| {
                let Some(this) = weak.upgrade() else {
                    return None;
                };
                Some(this.tick(eventtime))
            }),
            reactor.monotonic() + PIN_MIN_TIME,
        );
        *self.timer.lock().unwrap_or_else(|p| p.into_inner()) = Some(handle);
    }

    /// One check (`ControllerFan.callback`): choose the speed and return the
    /// next wake time.
    fn tick(&self, eventtime: f64) -> f64 {
        let active = self.any_stepper_enabled() || self.any_heater_targeting();
        let mut speed = 0.0;
        let mut last_on = self.last_on.lock().unwrap_or_else(|p| p.into_inner());
        if active {
            *last_on = 0;
            speed = self.fan_speed;
        } else if *last_on < self.idle_timeout {
            speed = self.idle_speed;
            *last_on += 1;
        }
        drop(last_on);

        let mut last_speed = self.last_speed.lock().unwrap_or_else(|p| p.into_inner());
        if speed != *last_speed {
            *last_speed = speed;
            let _ = self.fan.set_speed(speed);
        }
        eventtime + TICK_PERIOD
    }

    /// Whether any watched stepper's motor is on.
    fn any_stepper_enabled(&self) -> bool {
        let status = self.stepper_enable.get_status(0.0);
        let Some(enabled) = status.get("steppers").and_then(Value::as_object) else {
            return false;
        };
        let steppers = self.steppers.lock().unwrap_or_else(|p| p.into_inner());
        steppers
            .iter()
            .any(|name| enabled.get(name).and_then(Value::as_bool).unwrap_or(false))
    }

    /// Whether any watched heater has a target temperature.
    fn any_heater_targeting(&self) -> bool {
        let heaters = self.heaters.lock().unwrap_or_else(|p| p.into_inner());
        heaters.iter().any(|heater| {
            heater
                .get_status(0.0)
                .get("target")
                .and_then(Value::as_f64)
                .is_some_and(|target| target != 0.0)
        })
    }
}

impl PrinterObject for ControllerFan {
    /// Upstream's `ControllerFan.get_status`: the fan's own status.
    fn get_status(&self, eventtime: f64) -> Value {
        self.fan.get_status(eventtime)
    }

    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            self.resolve()
                .map_err(crate::core::klippy::error::KlippyError::Config)?;
            Ok(())
        })
    }
}

impl std::fmt::Debug for ControllerFan {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ControllerFan")
            .field("heater_names", &self.heater_names)
            .finish_non_exhaustive()
    }
}

impl Drop for ControllerFan {
    fn drop(&mut self) {
        if let Some(handle) = self.timer.lock().unwrap_or_else(|p| p.into_inner()).take() {
            handle.cancel();
        }
    }
}

/// Wire the built object into the printer's `klippy:ready` event.
///
/// Upstream registers that handler inside `__init__`; here the `Arc` exists
/// only after construction, so the handler is attached in `load_config_prefix`.
fn on_ready(printer: &Arc<Printer>, fan: &Arc<ControllerFan>) {
    let weak = Arc::downgrade(fan);
    printer.register_event_handler(
        KlippyEvent::KlippyReady,
        Box::new(move |_| {
            if let Some(this) = weak.upgrade() {
                this.start_timer();
            }
        }),
    );
}

/// Upstream's `load_config_prefix` for `[controller_fan <name>]`
/// (`controller_fan.py:68`).
///
/// # Errors
/// A missing or invalid option, or a pin that cannot be set up.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let fan = ControllerFan::new(config, printer)?;
    on_ready(printer, &fan);
    Ok(fan)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, Config, ConfigSection, ConfigValue};
    use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
    use crate::core::klippy::mcu::McuError;
    use crate::core::klippy::pins::{
        DigitalOut, PinChip, PinError, PinParams, PrinterPins, PwmOut, PINS_OBJECT,
    };
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::json;

    /// Drive a `connect` future on a private single-thread runtime.
    fn block_on<F: std::future::Future>(future: F) -> F::Output {
        tokio::runtime::Builder::new_current_thread()
            .build()
            .expect("a runtime for the test")
            .block_on(future)
    }

    /// A PWM that records what it was told.
    #[derive(Default)]
    struct FakePwm {
        updates: Mutex<Vec<f64>>,
    }

    impl PwmOut for FakePwm {
        fn setup_max_duration(&self, _max_duration: f64) {}
        fn setup_cycle_time(&self, _cycle_time: f64, _hardware_pwm: bool) {}
        fn setup_start_value(&self, _start: f64, _shutdown: f64) {}
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

    /// A printer with `gcode` and `pins` over a fake chip.
    fn printer() -> (Arc<Printer>, Arc<FakeChip>) {
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
        (printer, chip)
    }

    /// An object whose status reports `target`, standing in for a heater.
    struct FakeHeater {
        target: Mutex<f64>,
    }

    impl FakeHeater {
        fn new(target: f64) -> Self {
            Self {
                target: Mutex::new(target),
            }
        }

        fn set_target(&self, target: f64) {
            *self.target.lock().unwrap() = target;
        }
    }

    impl PrinterObject for FakeHeater {
        fn get_status(&self, _eventtime: f64) -> Value {
            json!({ "temperature": 25.0, "target": *self.target.lock().unwrap() })
        }
    }

    /// A `[controller_fan <name>]` section with `options`.
    fn section(name: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("controller_fan", Some(name));
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

    fn load(
        printer: &Arc<Printer>,
        section: &ConfigSection,
    ) -> Result<Arc<ControllerFan>, ConfigError> {
        let identifier = section.identifier();
        let object = load_config_prefix(&wrap(section), printer)?;
        printer
            .add_object(identifier.as_str(), object)
            .expect("one object per section in a fresh printer");
        Ok(printer
            .lookup_object_as::<ControllerFan>(&identifier)
            .expect("the factory builds a ControllerFan"))
    }

    fn pwm(chip: &FakeChip, index: usize) -> Arc<FakePwm> {
        chip.pwms.lock().unwrap()[index].clone()
    }

    fn updates(pwm: &FakePwm) -> Vec<f64> {
        pwm.updates.lock().unwrap().clone()
    }

    #[test]
    fn test_upstream_defaults_are_what_the_bare_section_gets() {
        // The upstream corpus section (`test/klippy/temperature.cfg:146`) sets
        // only `pin`; everything else is the default.
        let (printer, _chip) = printer();
        let cf = load(&printer, &section("test_controller_fan", &[("pin", "PH0")])).unwrap();

        assert_eq!(cf.fan_speed, 1.0);
        assert_eq!(cf.idle_speed, 1.0);
        assert_eq!(cf.idle_timeout, 30);
        assert_eq!(cf.heater_names, ["extruder"]);
        assert_eq!(cf.configured_steppers, None);
        // `last_on` starts at `idle_timeout`: no idle window before the fan
        // has been active once.
        assert_eq!(*cf.last_on.lock().unwrap(), 30);
    }

    #[test]
    fn test_every_option_is_read_and_overrides_reach_the_object() {
        let (printer, _chip) = printer();
        let text = "[controller_fan test_controller_fan]\n\
                    pin: PH0\n\
                    stepper: stepper_x, stepper_y\n\
                    heater: extruder, heater_bed\n\
                    fan_speed: 0.5\n\
                    idle_speed: 0.25\n\
                    idle_timeout: 10\n";
        let config = Config::from_text(text).expect("parses").0;
        let section = config
            .get_section("controller_fan test_controller_fan")
            .expect("the section parses");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::with_config(section, Arc::clone(&access), None, &config);

        let object = load_config_prefix(&wrapper, &printer).expect("the option set loads");
        printer
            .add_object("controller_fan test_controller_fan", object)
            .unwrap();
        let cf = printer
            .lookup_object_as::<ControllerFan>("controller_fan test_controller_fan")
            .unwrap();

        for option in section.parameters.keys() {
            assert!(
                access.contains("controller_fan test_controller_fan", option),
                "option '{option}' was not read"
            );
        }
        assert_eq!(cf.fan_speed, 0.5);
        assert_eq!(cf.idle_speed, 0.25);
        assert_eq!(cf.idle_timeout, 10);
        assert_eq!(
            cf.configured_steppers,
            Some(vec!["stepper_x".to_string(), "stepper_y".to_string()])
        );
        assert_eq!(
            cf.heater_names,
            ["extruder".to_string(), "heater_bed".to_string()]
        );
    }

    #[test]
    fn test_a_missing_pin_names_the_prefixed_section() {
        let (printer, _chip) = printer();

        let err = load(&printer, &section("test_controller_fan", &[])).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'pin' in section 'controller_fan test_controller_fan' must be specified"
        );
    }

    #[test]
    fn test_connect_rejects_an_unknown_stepper_the_way_upstream_does() {
        let (printer, _chip) = printer();
        // Upstream resolves `heater` names before `stepper` ones
        // (`controller_fan.py:41-53`), so the default `extruder` must exist.
        printer
            .add_object(
                "extruder",
                Arc::new(FakeHeater::new(0.0)) as Arc<dyn PrinterObject>,
            )
            .unwrap();
        let cf = load(
            &printer,
            &section(
                "test_controller_fan",
                &[("pin", "PH0"), ("stepper", "bogus_stepper")],
            ),
        )
        .unwrap();

        let err = block_on(cf.connect()).unwrap_err();

        let text = err.to_string();
        assert!(
            text.contains("One or more of these steppers are unknown: ['bogus_stepper']"),
            "{text}"
        );
        assert!(text.contains("(valid steppers are: )"), "{text}");
    }

    #[test]
    fn test_connect_rejects_an_unknown_heater_the_way_upstream_does() {
        let (printer, _chip) = printer();
        let cf = load(
            &printer,
            &section(
                "test_controller_fan",
                &[("pin", "PH0"), ("heater", "no_such_heater")],
            ),
        )
        .unwrap();

        let err = block_on(cf.connect()).unwrap_err();

        // `heaters.py:288` — `Unknown heater '%s'`.
        assert!(
            err.to_string().contains("Unknown heater 'no_such_heater'"),
            "{err}"
        );
    }

    #[test]
    fn test_the_fan_runs_full_speed_then_idles_then_stops() {
        let (printer, chip) = printer();
        let extruder = Arc::new(FakeHeater::new(1.0));
        printer
            .add_object("extruder", extruder.clone() as Arc<dyn PrinterObject>)
            .unwrap();
        let cf = load(
            &printer,
            &section(
                "test_controller_fan",
                &[
                    ("pin", "PH0"),
                    ("kick_start_time", "0"),
                    ("fan_speed", "0.5"),
                    ("idle_speed", "0.25"),
                    ("idle_timeout", "3"),
                ],
            ),
        )
        .unwrap();
        block_on(cf.connect()).unwrap();

        // A heater with a target: full configured speed.
        cf.tick(0.0);
        assert_eq!(updates(&pwm(&chip, 0)), [0.5]);

        // The target drops: `idle_speed` for `idle_timeout` seconds, then the
        // fan goes off — a speed is written only when it changes, so the
        // window shows up as one `idle_speed` write and one off write.
        extruder.set_target(0.0);
        cf.tick(1.0);
        cf.tick(2.0);
        cf.tick(3.0);
        cf.tick(4.0);
        assert_eq!(updates(&pwm(&chip, 0)), [0.5, 0.25, 0.0]);

        // Quiet after the window: no further writes.
        cf.tick(5.0);
        assert_eq!(updates(&pwm(&chip, 0)), [0.5, 0.25, 0.0]);
    }
}
