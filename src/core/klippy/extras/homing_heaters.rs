//! `[homing_heaters]` — switch heaters off around a homing move (upstream
//! `klippy/extras/homing_heaters.py`).
//!
//! | option | default | role |
//! |---|---|---|
//! | `heaters` | every heater | the heaters switched to 0 while homing |
//! | `steppers` | every stepper | the steppers whose homing move triggers it |
//!
//! The section registers handlers for `homing:homing_move_begin` /
//! `homing:homing_move_end` (`homing_heaters.py:14-17`): at the start of a
//! homing move it saves each selected heater's target and sets it to 0, and at
//! the end it restores the saved target. The names are resolved at connect
//! (upstream's `handle_connect`), with upstream's error wording.
//!
//! # What is not here
//!
//! Upstream's `check_eligible` keeps the handlers out of a move that does not
//! home one of the `steppers` (`hmove.get_mcu_endstops()`,
//! `homing_heaters.py:42-48`). This port's `homing:homing_move_begin` /
//! `homing_move_end` events carry no homing-move payload (`event/decl/homing.rs`),
//! so the endstops of the move in progress are not available to a handler and
//! the filter cannot be evaluated. Every homing move is treated as eligible —
//! which is what a config that leaves `steppers` unset (upstream's own default)
//! expects; a config that sets `steppers` disables its heaters on more moves
//! than upstream would.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::heaters::{self, PrinterHeaters};
use crate::core::klippy::extras::stepper_enable::PrinterStepperEnable;
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};

section!("homing_heaters", order = 30, load = load_config);

/// One `[homing_heaters]` section.
pub struct HomingHeaters {
    /// The `heaters` option as configured; `None` means every heater
    /// (`homing_heaters.py:18`).
    configured_heaters: Option<Vec<String>>,
    /// The heaters to switch off, resolved in `connect`: the configured list,
    /// or every registered heater when none was given
    /// (`homing_heaters.py:25-32`).
    heaters: Mutex<Vec<String>>,
    /// The `steppers` option as configured; `None` means every stepper
    /// (`homing_heaters.py:19`).
    flaky_steppers: Option<Vec<String>>,
    /// The heater registry the names resolve against.
    pheaters: Arc<PrinterHeaters>,
    /// The stepper registry `steppers` is validated against.
    stepper_enable: Arc<PrinterStepperEnable>,
    /// The target saved for each heater at the start of a homing move
    /// (`homing_heaters.py:21,54`).
    target_save: Mutex<BTreeMap<String, f64>>,
}

impl HomingHeaters {
    /// Read the section and load its dependencies (`homing_heaters.py:12-21`).
    ///
    /// # Errors
    /// A dependency that fails to load.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let configured_heaters = config.get_list("heaters", ',');
        let flaky_steppers = config.get_list("steppers", ',');
        // Upstream's `self.printer.load_object(config, 'heaters')`.
        let pheaters = heaters::ensure(printer)?;
        let stepper_enable = PrinterStepperEnable::ensure(printer);
        Ok(Self {
            configured_heaters,
            heaters: Mutex::new(Vec::new()),
            flaky_steppers,
            pheaters,
            stepper_enable,
            target_save: Mutex::new(BTreeMap::new()),
        })
    }

    /// Resolve and validate the configured names (`homing_heaters.py:23-41`).
    ///
    /// # Errors
    /// A `heaters` or `steppers` name that is not registered, with upstream's
    /// wording.
    fn handle_connect(&self) -> Result<(), ConfigError> {
        let all_heaters = self.pheaters.get_all_heaters();
        let resolved = match &self.configured_heaters {
            None => all_heaters,
            Some(names) => {
                if !names.iter().all(|name| all_heaters.contains(name)) {
                    return Err(ConfigError::new(format!(
                        "One or more of these heaters are unknown: {}",
                        python_list(names)
                    )));
                }
                names.clone()
            }
        };
        *self.lock() = resolved;

        // `steppers` is only validated; see the module docs for the eligibility
        // filter that cannot be evaluated here.
        if let Some(names) = &self.flaky_steppers {
            let all_steppers = self.stepper_enable.get_steppers();
            if !names.iter().all(|name| all_steppers.contains(name)) {
                return Err(ConfigError::new(format!(
                    "One or more of these steppers are unknown: {}",
                    python_list(names)
                )));
            }
        }
        Ok(())
    }

    /// Save each selected heater's target and switch it off
    /// (`homing_heaters.py:49-55`).
    fn handle_homing_move_begin(&self) {
        let names = self.lock().clone();
        let mut save = self.target_save.lock().unwrap_or_else(|p| p.into_inner());
        for name in names {
            let Ok(heater) = self.pheaters.lookup_heater(&name) else {
                continue;
            };
            save.insert(name, heater.get_temp().1);
            let _ = heater.set_temp(0.0);
        }
    }

    /// Restore the targets saved by [`Self::handle_homing_move_begin`]
    /// (`homing_heaters.py:56-61`).
    fn handle_homing_move_end(&self) {
        let names = self.lock().clone();
        let mut save = self.target_save.lock().unwrap_or_else(|p| p.into_inner());
        for name in names {
            let Some(target) = save.remove(&name) else {
                continue;
            };
            let Ok(heater) = self.pheaters.lookup_heater(&name) else {
                continue;
            };
            // Upstream's `Heater.set_temp` is unguarded; here only a target
            // that was accepted once is being restored, so the range check
            // cannot reject it. `0` (a heater that had no target) always passes.
            let _ = heater.set_temp(target);
        }
    }

    fn lock(&self) -> MutexGuard<'_, Vec<String>> {
        self.heaters.lock().unwrap_or_else(|p| p.into_inner())
    }
}

/// A list in upstream's Python `str` shape (`['a', 'b']`), for the
/// `%s % (list,)` in its error messages.
fn python_list(names: &[String]) -> String {
    let listed = names
        .iter()
        .map(|name| format!("'{name}'"))
        .collect::<Vec<_>>()
        .join(", ");
    format!("[{listed}]")
}

impl PrinterObject for HomingHeaters {
    /// Upstream's `HomingHeaters` defines no `get_status`
    /// (`homing_heaters.py`).
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    /// Kept out of `objects/list`, like any object without `get_status`
    /// upstream.
    fn is_queryable(&self) -> bool {
        false
    }

    /// Upstream's `klippy:connect` handler (`homing_heaters.py:12-13,23`).
    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            self.handle_connect()
                .map_err(crate::core::klippy::error::KlippyError::Config)
        })
    }
}

impl std::fmt::Debug for HomingHeaters {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("HomingHeaters")
            .field("heaters", &self.lock())
            .finish_non_exhaustive()
    }
}

/// The factory `section!` names (`homing_heaters.py:63 def load_config`).
///
/// The homing-move handlers are attached here rather than in `HomingHeaters::new`
/// because they need the shared `Arc`, which exists only once the object is
/// built — the same split `controller_fan`'s `on_ready` uses.
///
/// # Errors
/// A dependency that fails to load.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = Arc::new(HomingHeaters::new(config, printer)?);
    register_handlers(printer, &object);
    Ok(object)
}

/// Wire the built object into the `homing:homing_move_begin`/`_end` events
/// (`homing_heaters.py:14-17`).
fn register_handlers(printer: &Arc<Printer>, object: &Arc<HomingHeaters>) {
    let weak = Arc::downgrade(object);
    printer.register_event_handler(
        KlippyEvent::HomingHomingMoveBegin,
        Box::new(move |_| {
            if let Some(this) = weak.upgrade() {
                this.handle_homing_move_begin();
            }
        }),
    );
    let weak = Arc::downgrade(object);
    printer.register_event_handler(
        KlippyEvent::HomingHomingMoveEnd,
        Box::new(move |_| {
            if let Some(this) = weak.upgrade() {
                this.handle_homing_move_end();
            }
        }),
    );
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{Config, ConfigSection, ConfigValue};
    use crate::core::klippy::extras::heaters::{Heater, Sensor, SensorCallback};
    use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
    use crate::core::klippy::mcu::McuError;
    use crate::core::klippy::pins::{
        DigitalOut, PinChip, PinError, PinParams, PrinterPins, PwmOut, PINS_OBJECT,
    };
    use crate::core::klippy::printer::Printer;
    use crate::core::klippy::reactor::ManualReactor;

    /// A sensor that accepts everything; the heater tests need no readings.
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

    /// A printer with `gcode` and `pins` over a no-op chip.
    fn ready_printer() -> Arc<Printer> {
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
    }

    /// A section with the given options, as the parser would build it.
    fn section(id: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new(id, None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A printer with the fake sensor registered and one `[extruder]` heater
    /// set up, so a `homing_heaters` section can name it.
    fn printer_with_heater() -> (Arc<Printer>, Arc<Heater>) {
        let printer = ready_printer();
        let registry = heaters::ensure(&printer).unwrap();
        registry.add_sensor_factory(
            "Fake",
            Arc::new(|_config, _printer| Ok(Arc::new(FakeSensor) as Arc<dyn Sensor>)),
        );
        let heater = registry
            .setup_heater(
                &ConfigWrapper::untracked(&section(
                    "extruder",
                    &[
                        ("sensor_type", "Fake"),
                        ("heater_pin", "PA0"),
                        ("min_temp", "0"),
                        ("max_temp", "250"),
                        ("control", "watermark"),
                    ],
                )),
                &printer,
                None,
            )
            .unwrap();
        (printer, heater)
    }

    /// Drive a `connect` future on a private single-thread runtime.
    fn block_on<F: std::future::Future>(future: F) -> F::Output {
        tokio::runtime::Builder::new_current_thread()
            .build()
            .expect("a runtime for the test")
            .block_on(future)
    }

    /// The begin event saves the target and switches the heater off, and the
    /// end event restores it (`homing_heaters.py:49-61`).
    #[test]
    fn test_the_homing_move_events_disable_and_restore_the_targets() {
        let (printer, heater) = printer_with_heater();
        let object = load_config(
            &ConfigWrapper::untracked(&section("homing_heaters", &[("heaters", "extruder")])),
            &printer,
        )
        .unwrap();
        block_on(object.connect()).unwrap();

        heater.set_temp(200.0).unwrap();
        assert_eq!(heater.get_temp().1, 200.0);

        printer.send_event(&KlippyEvent::HomingHomingMoveBegin);
        assert_eq!(heater.get_temp().1, 0.0);

        printer.send_event(&KlippyEvent::HomingHomingMoveEnd);
        assert_eq!(heater.get_temp().1, 200.0);
    }

    /// With no `heaters` option every registered heater is switched off
    /// (`homing_heaters.py:25-27`).
    #[test]
    fn test_the_default_is_every_heater() {
        let (printer, heater) = printer_with_heater();
        let object = load_config(
            &ConfigWrapper::untracked(&section("homing_heaters", &[])),
            &printer,
        )
        .unwrap();
        block_on(object.connect()).unwrap();

        heater.set_temp(150.0).unwrap();
        printer.send_event(&KlippyEvent::HomingHomingMoveBegin);
        assert_eq!(heater.get_temp().1, 0.0);
        printer.send_event(&KlippyEvent::HomingHomingMoveEnd);
        assert_eq!(heater.get_temp().1, 150.0);
    }

    /// A heater that had no target comes back to `0`, not to whatever it was
    /// set to mid-move (`homing_heaters.py:54,61`).
    #[test]
    fn test_the_restore_uses_the_target_saved_at_the_begin() {
        let (printer, heater) = printer_with_heater();
        let object = load_config(
            &ConfigWrapper::untracked(&section("homing_heaters", &[("heaters", "extruder")])),
            &printer,
        )
        .unwrap();
        block_on(object.connect()).unwrap();

        // No target set: the move both begins and ends at 0.
        printer.send_event(&KlippyEvent::HomingHomingMoveBegin);
        assert_eq!(heater.get_temp().1, 0.0);
        printer.send_event(&KlippyEvent::HomingHomingMoveEnd);
        assert_eq!(heater.get_temp().1, 0.0);
    }

    /// A `heaters` name that is not registered is refused at connect, with
    /// upstream's wording (`homing_heaters.py:29-32`).
    #[test]
    fn test_an_unknown_heater_is_refused_at_connect() {
        let (printer, _heater) = printer_with_heater();
        let object = load_config(
            &ConfigWrapper::untracked(&section("homing_heaters", &[("heaters", "bogus_extruder")])),
            &printer,
        )
        .unwrap();

        let err = block_on(object.connect()).unwrap_err();
        assert_eq!(
            err.to_string(),
            "One or more of these heaters are unknown: ['bogus_extruder']"
        );
    }

    /// A `steppers` name that is not registered is refused at connect
    /// (`homing_heaters.py:38-41`).
    #[test]
    fn test_an_unknown_stepper_is_refused_at_connect() {
        let (printer, _heater) = printer_with_heater();
        let object = load_config(
            &ConfigWrapper::untracked(&section("homing_heaters", &[("steppers", "bogus_z")])),
            &printer,
        )
        .unwrap();

        let err = block_on(object.connect()).unwrap_err();
        assert_eq!(
            err.to_string(),
            "One or more of these steppers are unknown: ['bogus_z']"
        );
    }

    /// An option-less `[homing_heaters]` is a valid section — the config-load
    /// contract — and has no `get_status` (`homing_heaters.py`).
    #[test]
    fn test_an_empty_section_loads_and_is_not_queryable() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) =
            Config::from_text("[mcu]\nserial: /dev/not-opened-yet\n[homing_heaters]\n")
                .expect("the config parses");

        printer
            .load_config(&config)
            .expect("an option-less [homing_heaters] loads");

        let object = printer
            .lookup_object("homing_heaters")
            .expect("the object is registered");
        assert!(!object.is_queryable());
        assert_eq!(object.get_status(0.0), json!({}));
    }
}
