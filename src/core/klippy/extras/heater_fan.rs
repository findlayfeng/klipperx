//! `[heater_fan <name>]` — a fan that runs while a heater is hot or heating.
//!
//! Upstream's `klippy/extras/heater_fan.py`: the section is a
//! [`Fan`](crate::core::klippy::extras::fan) core whose speed is chosen once a
//! second — full `fan_speed` while any named heater has a target *or* its
//! measured temperature is above `heater_temp`, and off otherwise (there is no
//! hysteresis: the check is recomputed from scratch every tick).
//!
//! | option | meaning |
//! |---|---|
//! | `pin` | the fan's PWM pin, required (via the `Fan` core) |
//! | `heater` | heaters that switch the fan on (default: `extruder`) |
//! | `heater_temp` | temperature above which a heater switches the fan on (default 50) |
//! | `fan_speed` | duty while on (default 1, `0 ..= 1`) |
//!
//! The per-second timer starts at `klippy:ready` (`PIN_MIN_TIME` after the
//! monotonic clock, then one tick per second), and the `heater` references are
//! resolved in `connect` — upstream's `handle_ready`, with its error wording.
//! The fan keeps running when klippy dies: `Fan::new` is given a
//! `default_shutdown_speed` of 1 (`heater_fan.py:18`), so a hotend fan does not
//! stop with the host.
//!
//! # What is not here
//!
//! * **Heater names are object names.** Upstream looks heaters up in
//!   `heaters.lookup_heater`, which keys them by their *short* name
//!   (`extruder`, or a `[heater_generic <name>]`'s `<name>`); this port has no
//!   such registry yet (H1) and resolves against the printer object registry
//!   instead, so a `heater: <name>` must name an object. The corpus default
//!   (`extruder`) resolves either way.
//! * **The measured temperature is the status one.** Upstream reads the raw
//!   `heater.get_temp(eventtime)`; this port's `Heater` exposes only
//!   `get_status`, whose `temperature` is the smoothed value rounded to two
//!   decimals. Comparing that round number against `heater_temp` can differ
//!   from upstream only within half a hundredth of a degree.

use std::sync::{Arc, Mutex, Weak};

use serde_json::Value;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::fan::Fan;
use crate::core::klippy::extras::heaters;
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};
use crate::core::klippy::reactor::TimerHandle;

// Only the prefix form (`[heater_fan <name>]`) exists upstream
// (`heater_fan.py:39`).
section!("heater_fan", order = 30, prefix = load_config_prefix);

/// How long after `klippy:ready` the first check runs (`PIN_MIN_TIME`), and
/// the tick period that follows (upstream's callback returns `eventtime + 1.`).
const PIN_MIN_TIME: f64 = 0.100;
const TICK_PERIOD: f64 = 1.0;

/// One `[heater_fan <name>]`.
pub struct HeaterFan {
    fan: Arc<Fan>,
    /// The `heater` option as configured.
    heater_names: Vec<String>,
    heater_temp: f64,
    /// The objects behind the `heater` option, resolved in `connect`.
    heaters: Mutex<Vec<Arc<dyn PrinterObject>>>,
    fan_speed: f64,
    last_speed: Mutex<f64>,
    printer: WeakPrinter,
    /// The handle the `klippy:ready` handler creates, cancelled on drop.
    timer: Mutex<Option<TimerHandle>>,
    self_ref: Weak<HeaterFan>,
}

/// A weak handle to the printer, without importing `std::sync::Weak` twice.
type WeakPrinter = std::sync::Weak<Printer>;

impl HeaterFan {
    /// Read the section and build the fan (`PrinterHeaterFan.__init__`).
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        // Upstream reads `heater`/`heater_temp` before building the fan
        // (`heater_fan.py:13-18`).
        heaters::ensure(printer)?;
        // Upstream's default for `getlist("heater", ("extruder",))`.
        let heater_names = config
            .get_list("heater", ',')
            .unwrap_or_else(|| vec!["extruder".to_string()]);
        let heater_temp = config.get_float("heater_temp", Some(50.0))?;
        // A hotend fan survives klippy's death: `default_shutdown_speed` is 1.
        let fan = Fan::new(config, printer, 1.0)?;
        let fan_speed =
            config.get_float_bounded("fan_speed", Some(1.0), Some(0.0), Some(1.0), None, None)?;

        Ok(Arc::new_cyclic(|weak| Self {
            fan,
            heater_names,
            heater_temp,
            heaters: Mutex::new(Vec::new()),
            fan_speed,
            last_speed: Mutex::new(0.0),
            printer: Arc::downgrade(printer),
            timer: Mutex::new(None),
            self_ref: weak.clone(),
        }))
    }

    /// Resolve the `heater` names (`PrinterHeaterFan.handle_ready`).
    ///
    /// # Errors
    /// An unknown heater, with upstream's wording.
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
        Ok(())
    }

    /// Start the per-second check (`PrinterHeaterFan.handle_ready`).
    fn start_timer(&self) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        let reactor = printer.reactor();
        let weak = self.self_ref.clone();
        let handle = reactor.register_timer_named(
            "heater_fan",
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

    /// One check (`PrinterHeaterFan.callback`): choose the speed and return the
    /// next wake time.
    fn tick(&self, eventtime: f64) -> f64 {
        let speed = if self.any_heater_on() {
            self.fan_speed
        } else {
            0.0
        };
        let mut last_speed = self.last_speed.lock().unwrap_or_else(|p| p.into_inner());
        if speed != *last_speed {
            *last_speed = speed;
            let _ = self.fan.set_speed(speed);
        }
        eventtime + TICK_PERIOD
    }

    /// Whether any watched heater is heating or hotter than `heater_temp`
    /// (`if target_temp or current_temp > self.heater_temp`).
    fn any_heater_on(&self) -> bool {
        let heaters = self.heaters.lock().unwrap_or_else(|p| p.into_inner());
        heaters.iter().any(|heater| {
            let status = heater.get_status(0.0);
            let target = status.get("target").and_then(Value::as_f64).unwrap_or(0.0);
            let temperature = status
                .get("temperature")
                .and_then(Value::as_f64)
                .unwrap_or(0.0);
            target != 0.0 || temperature > self.heater_temp
        })
    }
}

impl PrinterObject for HeaterFan {
    /// Upstream's `PrinterHeaterFan.get_status`: the fan's own status.
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

impl std::fmt::Debug for HeaterFan {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("HeaterFan")
            .field("heater_names", &self.heater_names)
            .finish_non_exhaustive()
    }
}

impl Drop for HeaterFan {
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
fn on_ready(printer: &Arc<Printer>, fan: &Arc<HeaterFan>) {
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

/// Upstream's `load_config_prefix` for `[heater_fan <name>]`
/// (`heater_fan.py:39`).
///
/// # Errors
/// A missing or invalid option, or a pin that cannot be set up.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let fan = HeaterFan::new(config, printer)?;
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
        start_value: Mutex<(f64, f64)>,
        updates: Mutex<Vec<f64>>,
    }

    impl PwmOut for FakePwm {
        fn setup_max_duration(&self, _max_duration: f64) {}
        fn setup_cycle_time(&self, _cycle_time: f64, _hardware_pwm: bool) {}
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

    /// An object whose status reports `temperature`/`target`, standing in for a
    /// heater.
    struct FakeHeater {
        temperature: Mutex<f64>,
        target: Mutex<f64>,
    }

    impl FakeHeater {
        fn new(temperature: f64, target: f64) -> Self {
            Self {
                temperature: Mutex::new(temperature),
                target: Mutex::new(target),
            }
        }

        fn set(&self, temperature: f64, target: f64) {
            *self.temperature.lock().unwrap() = temperature;
            *self.target.lock().unwrap() = target;
        }
    }

    impl PrinterObject for FakeHeater {
        fn get_status(&self, _eventtime: f64) -> Value {
            json!({
                "temperature": *self.temperature.lock().unwrap(),
                "target": *self.target.lock().unwrap(),
            })
        }
    }

    /// A `[heater_fan <name>]` section with `options`.
    fn section(name: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("heater_fan", Some(name));
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
    ) -> Result<Arc<HeaterFan>, ConfigError> {
        let identifier = section.identifier();
        let object = load_config_prefix(&wrap(section), printer)?;
        printer
            .add_object(identifier.as_str(), object)
            .expect("one object per section in a fresh printer");
        Ok(printer
            .lookup_object_as::<HeaterFan>(&identifier)
            .expect("the factory builds a HeaterFan"))
    }

    /// Register a heater under `name`.
    fn add_heater(printer: &Arc<Printer>, name: &str, heater: &Arc<FakeHeater>) {
        printer
            .add_object(name, Arc::clone(heater) as Arc<dyn PrinterObject>)
            .unwrap();
    }

    fn pwm(chip: &FakeChip, index: usize) -> Arc<FakePwm> {
        chip.pwms.lock().unwrap()[index].clone()
    }

    fn updates(pwm: &FakePwm) -> Vec<f64> {
        pwm.updates.lock().unwrap().clone()
    }

    #[test]
    fn test_upstream_defaults_are_what_the_bare_section_gets() {
        let (printer, chip) = printer();
        let hf = load(&printer, &section("test_heater_fan", &[("pin", "PH0")])).unwrap();

        assert_eq!(hf.heater_names, ["extruder"]);
        assert_eq!(hf.heater_temp, 50.0);
        assert_eq!(hf.fan_speed, 1.0);
        assert_eq!(*hf.last_speed.lock().unwrap(), 0.0);
        // The fan starts at 0 and the firmware's shutdown duty is 1
        // (`heater_fan.py:18`), so a hotend fan keeps running when klippy dies.
        assert_eq!(*pwm(&chip, 0).start_value.lock().unwrap(), (0.0, 1.0));
    }

    #[test]
    fn test_every_option_is_read_and_overrides_reach_the_object() {
        let (printer, _chip) = printer();
        let text = "[heater_fan test_heater_fan]\n\
                    pin: PH0\n\
                    heater: extruder, heater_bed\n\
                    heater_temp: 60\n\
                    fan_speed: 0.5\n";
        let config = Config::from_text(text).expect("parses").0;
        let section = config
            .get_section("heater_fan test_heater_fan")
            .expect("the section parses");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::with_config(section, Arc::clone(&access), None, &config);

        let object = load_config_prefix(&wrapper, &printer).expect("the option set loads");
        printer
            .add_object("heater_fan test_heater_fan", object)
            .unwrap();
        let hf = printer
            .lookup_object_as::<HeaterFan>("heater_fan test_heater_fan")
            .unwrap();

        // No option is left unread (`check_unused`).
        for option in section.parameters.keys() {
            assert!(
                access.contains("heater_fan test_heater_fan", option),
                "option '{option}' was not read"
            );
        }
        assert_eq!(hf.heater_temp, 60.0);
        assert_eq!(hf.fan_speed, 0.5);
        assert_eq!(
            hf.heater_names,
            ["extruder".to_string(), "heater_bed".to_string()]
        );
    }

    #[test]
    fn test_a_missing_pin_names_the_prefixed_section() {
        let (printer, _chip) = printer();

        let err = load(&printer, &section("test_heater_fan", &[])).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'pin' in section 'heater_fan test_heater_fan' must be specified"
        );
    }

    #[test]
    fn test_fan_speed_out_of_range_is_rejected() {
        for bad in ["-0.1", "1.1"] {
            // A fresh printer per case: the pin is claimed once per printer.
            let (printer, _chip) = printer();
            let err = load(
                &printer,
                &section("test_heater_fan", &[("pin", "PH0"), ("fan_speed", bad)]),
            )
            .unwrap_err();
            assert!(
                err.to_string()
                    .contains("Option 'fan_speed' in section 'heater_fan test_heater_fan'"),
                "{err}"
            );
        }
    }

    #[test]
    fn test_connect_rejects_an_unknown_heater_the_way_upstream_does() {
        let (printer, _chip) = printer();
        let hf = load(
            &printer,
            &section(
                "test_heater_fan",
                &[("pin", "PH0"), ("heater", "no_such_heater")],
            ),
        )
        .unwrap();

        let err = block_on(hf.connect()).unwrap_err();

        // `heaters.py:288` — `Unknown heater '%s'`.
        assert!(
            err.to_string().contains("Unknown heater 'no_such_heater'"),
            "{err}"
        );
    }

    #[test]
    fn test_a_target_or_a_hot_heater_runs_the_fan_and_a_write_only_on_change() {
        let (printer, chip) = printer();
        let extruder = Arc::new(FakeHeater::new(25.0, 0.0));
        add_heater(&printer, "extruder", &extruder);
        let hf = load(
            &printer,
            &section(
                "test_heater_fan",
                &[("pin", "PH0"), ("kick_start_time", "0")],
            ),
        )
        .unwrap();
        block_on(hf.connect()).unwrap();

        // Cold and no target: the fan stays off; `0` is `last_speed` already,
        // so nothing is written.
        hf.tick(0.0);
        assert_eq!(updates(&pwm(&chip, 0)), Vec::<f64>::new());

        // ① A target below `heater_temp`: heating counts, full speed.
        extruder.set(25.0, 200.0);
        hf.tick(1.0);
        assert_eq!(updates(&pwm(&chip, 0)), [1.0]);

        // ④ The speed does not change: no further write.
        hf.tick(2.0);
        assert_eq!(updates(&pwm(&chip, 0)), [1.0]);

        // ② No target any more, but hotter than `heater_temp` (50): still on,
        // and still no write.
        extruder.set(80.0, 0.0);
        hf.tick(3.0);
        assert_eq!(updates(&pwm(&chip, 0)), [1.0]);

        // ③ Neither: the fan goes off.
        extruder.set(49.0, 0.0);
        hf.tick(4.0);
        assert_eq!(updates(&pwm(&chip, 0)), [1.0, 0.0]);

        // Off and staying off: no repeated write.
        hf.tick(5.0);
        assert_eq!(updates(&pwm(&chip, 0)), [1.0, 0.0]);
    }

    #[test]
    fn test_heater_temp_is_a_threshold_not_a_hysteresis() {
        let (printer, chip) = printer();
        let extruder = Arc::new(FakeHeater::new(51.0, 0.0));
        add_heater(&printer, "extruder", &extruder);
        let hf = load(
            &printer,
            &section(
                "test_heater_fan",
                &[
                    ("pin", "PH0"),
                    ("kick_start_time", "0"),
                    ("heater_temp", "50"),
                    ("fan_speed", "0.5"),
                ],
            ),
        )
        .unwrap();
        block_on(hf.connect()).unwrap();

        // Exactly at the threshold is not above it: off.
        extruder.set(50.0, 0.0);
        hf.tick(0.0);
        assert_eq!(updates(&pwm(&chip, 0)), Vec::<f64>::new());

        // One tenth above: on at the configured speed.
        extruder.set(50.1, 0.0);
        hf.tick(1.0);
        assert_eq!(updates(&pwm(&chip, 0)), [0.5]);

        // Back to the threshold: off again, with no damping in between.
        extruder.set(50.0, 0.0);
        hf.tick(2.0);
        assert_eq!(updates(&pwm(&chip, 0)), [0.5, 0.0]);
    }

    #[test]
    fn test_the_section_loads_from_a_full_config() {
        // What the corpus configs do — an `[mcu]` with `[heater_fan]` — must
        // survive the real loader, which also runs the undefined-option check
        // over this section (the unit tests above use a bare wrapper).
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let text = "[mcu]\nserial: /dev/not-opened-yet\n\
                    [heater_fan hotend_fan]\npin: PH0\n\
                    heater: extruder\nheater_temp: 60\nfan_speed: 0.5\n";
        let (config, _) = Config::from_text(text).expect("the config parses");
        printer.load_config(&config).expect("the config loads");

        let hf = printer
            .lookup_object_as::<HeaterFan>("heater_fan hotend_fan")
            .expect("the prefixed section is registered");
        assert_eq!(hf.heater_names, ["extruder"]);
        assert_eq!(hf.heater_temp, 60.0);
        assert_eq!(hf.fan_speed, 0.5);
    }
}
