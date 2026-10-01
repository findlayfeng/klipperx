//! `[pwm_tool <name>]` — a PWM pin with an optional firmware duration limit.
//!
//! Upstream is `klippy/extras/pwm_tool.py`: `PrinterOutputPin` over
//! `MCU_queued_pwm`, a PWM output whose `maximum_mcu_duration` makes the
//! firmware fall back to the shutdown duty when no update arrives for that
//! long (`pwm_tool.py:26-116`). Like [`output_pin`](crate::core::klippy::extras::output_pin)
//! it registers `SET_PIN PIN=<name> VALUE=<0..scale>`; unlike `output_pin` its
//! `SET_PIN` never takes a `CYCLE_TIME` — the period is fixed at load.
//!
//! | option | meaning |
//! |---|---|
//! | `pin` | the pin description, required |
//! | `cycle_time` | PWM period in seconds (default 0.1, `> 0`) |
//! | `hardware_pwm` | the firmware's PWM rather than a software one (default false) |
//! | `scale` | full-scale figure `VALUE` is bounded by (default 1, `> 0`) |
//! | `maximum_mcu_duration` | seconds the firmware may go without an update (default 0 = never fall back; a written value is `≥ 0.5`) |
//! | `value` | duty at startup (default 0, `0 ..= scale`, divided by `scale`) |
//! | `shutdown_value` | duty on shutdown (default 0, `0 ..= scale`, divided by `scale`) |
//!
//! Two bounds upstream applies have no counterpart here:
//!
//! * `cycle_time`'s and `maximum_mcu_duration`'s `maxval` are the chip's
//!   `max_nominal_duration`, which this port does not model
//!   (`pwm_tool.py:147-151`); the resource re-checks both figures at build
//!   (`PinError::PwmCycleTimeTooLarge` / `PwmMaxDurationTooLarge`).
//! * With a `maximum_mcu_duration` set, upstream requires `value` and
//!   `shutdown_value` to match (`pwm_tool.py:83-86`); the firmware build makes
//!   the same demand (`PinError::MaxDurationMismatch`), at build rather than
//!   load time.
//!
//! # What is not here
//!
//! * **Print-time scheduling.** Upstream queues `SET_PIN` through the toolhead
//!   and regenerates the duration-limit refreshes
//!   (`MCU_queued_pwm._gen_intermediate_updates`, `pwm_tool.py:117-146`);
//!   both need the print-time request queue (upstream `GCodeRequestQueue`),
//!   which is not ported (the print-time layer itself is: C1d). This port drives the pin through the
//!   resource's immediate path, like [`output_pin`](crate::core::klippy::extras::output_pin).

use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::gcode::{sync, CommandError, CommandHandler, GCodeDispatch, GCODE_OBJECT};
use crate::core::klippy::load::section;
use crate::core::klippy::pins::{PrinterPins, PwmOut, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form (`[pwm_tool <name>]`) exists upstream.
section!("pwm_tool", order = 20, prefix = load_config_prefix);

/// One configured `[pwm_tool <name>]`.
pub struct PwmTool {
    /// The name in `SET_PIN PIN=<name>`: the section's sub.
    name: String,
    /// The value last set, for `get_status`; shared with the `SET_PIN` handler.
    value: Arc<Mutex<f64>>,
}

impl PwmTool {
    /// Build the pin from its section and register `SET_PIN`.
    ///
    /// # Errors
    /// Returns a config error (a message naming the section) when the section
    /// has no name, an option is missing, unparseable, out of bounds, or the
    /// pin cannot be built.
    pub fn new(config: &ConfigWrapper, printer: &Printer) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let name = config.section().sub.clone().ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{identifier}' must be a '[pwm_tool <name>]' section"
            ))
        })?;

        // Upstream's read order (`pwm_tool.py:141-166`): `pin`, `cycle_time`,
        // `hardware_pwm`, `scale`, `maximum_mcu_duration`, `value`,
        // `shutdown_value`.
        let pin_desc = config.get("pin", None)?;
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let pwm = pins
            .setup_pwm(&pin_desc, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        let cycle_time =
            config.get_float_bounded("cycle_time", Some(0.100), None, None, Some(0.0), None)?;
        let hardware_pwm = config.get_bool("hardware_pwm", Some(false))?;
        let scale = config.get_float_bounded("scale", Some(1.0), None, None, Some(0.0), None)?;

        // Upstream returns a default before its bounds run
        // (`configfile.py:32-36`) — which is what lets `0.` be both the
        // default and below `minval=0.500` — so a written value is bounded and
        // an omitted one just records the default.
        let maximum_mcu_duration = if config.section().has("maximum_mcu_duration") {
            config.get_float_bounded("maximum_mcu_duration", None, Some(0.500), None, None, None)?
        } else {
            config.get_float("maximum_mcu_duration", Some(0.0))?
        };

        let value =
            config.get_float_bounded("value", Some(0.0), Some(0.0), Some(scale), None, None)?
                / scale;
        let shutdown_value = config.get_float_bounded(
            "shutdown_value",
            Some(0.0),
            Some(0.0),
            Some(scale),
            None,
            None,
        )? / scale;

        pwm.setup_cycle_time(cycle_time, hardware_pwm);
        pwm.setup_max_duration(maximum_mcu_duration);
        pwm.setup_start_value(value, shutdown_value);

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let value_slot = Arc::new(Mutex::new(value));
        let pwm = Arc::new(pwm);
        let handler: CommandHandler = {
            let pwm = Arc::clone(&pwm);
            let value_slot = Arc::clone(&value_slot);
            sync(move |gcmd| cmd_set_pin(&pwm, &value_slot, scale, gcmd))
        };
        gcode
            .register_mux_command_with_params(
                "SET_PIN",
                "PIN",
                Some(&name),
                handler,
                Some("Set the value of an output pin"),
                // `cmd_set_pin` reads the level; the mux key `PIN` is prepended
                // by the registration.
                &["VALUE"],
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        Ok(Self {
            name,
            value: value_slot,
        })
    }

    /// The name `SET_PIN` addresses this pin by.
    pub fn name(&self) -> &str {
        &self.name
    }

    fn lock(&self) -> MutexGuard<'_, f64> {
        self.value
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl PrinterObject for PwmTool {
    /// The value last set, as upstream's `PrinterOutputPin.get_status`.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "value": *self.lock() })
    }
}

impl std::fmt::Debug for PwmTool {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PwmTool")
            .field("name", &self.name)
            .finish_non_exhaustive()
    }
}

/// `SET_PIN PIN=<name> VALUE=<0..scale>`: drive the pin.
///
/// `VALUE` is bounded by the pin's `scale` and divided by it before driving
/// (`pwm_tool.py:174-177`); a repeat of the current duty sends nothing
/// (`pwm_tool.py:168-170`). No `CYCLE_TIME` parameter: upstream's `pwm_tool`
/// has none.
fn cmd_set_pin(
    pwm: &Arc<dyn PwmOut>,
    value_slot: &Arc<Mutex<f64>>,
    scale: f64,
    gcmd: &crate::core::klippy::gcode::GcodeCommand,
) -> Result<(), CommandError> {
    let value = gcmd.get_float_range("VALUE", 0.0, scale)? / scale;
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

/// Upstream's `load_config_prefix` for `[pwm_tool <name>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PwmTool::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{ConfigSection, ConfigValue};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::mcu::McuError;
    use crate::core::klippy::pins::{DigitalOut, PinChip, PinError, PinParams};
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

    /// A ready printer with `gcode` and `pins` over a fake chip.
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
        printer.send_event(&KlippyEvent::KlippyReady);
        (printer, chip)
    }

    /// A `[pwm_tool <name>]` section with `pin: <pin>` plus `options`.
    fn section(name: &str, pin: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("pwm_tool", Some(name));
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

    /// Wrap a hand-built section the way the loader does.
    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap()
    }

    fn created(chip: &FakeChip, index: usize) -> Arc<FakePwm> {
        chip.pwms.lock().unwrap()[index].clone()
    }

    #[test]
    fn test_upstream_defaults_are_applied() {
        let (printer, chip) = printer();
        let section = section("tool", "PA1", &[]);

        let pin = PwmTool::new(&wrap(&section), &printer).unwrap();

        // cycle_time 0.1 software, scale 1, no duration limit, value 0.
        let pwm = created(&chip, 0);
        assert_eq!(*pwm.cycle_time.lock().unwrap(), (0.1, false));
        assert_eq!(*pwm.max_duration.lock().unwrap(), 0.0);
        assert_eq!(*pwm.start_value.lock().unwrap(), (0.0, 0.0));
        assert_eq!(pin.get_status(0.0)["value"], 0.0);
    }

    #[test]
    fn test_the_section_options_reach_the_pin() {
        let (printer, chip) = printer();
        let section = section(
            "tool",
            "PA1",
            &[
                ("cycle_time", "0.02"),
                ("hardware_pwm", "true"),
                ("maximum_mcu_duration", "1.5"),
                ("scale", "2"),
                ("value", "1.0"),
                ("shutdown_value", "2.0"),
            ],
        );

        PwmTool::new(&wrap(&section), &printer).unwrap();

        let pwm = created(&chip, 0);
        assert_eq!(*pwm.cycle_time.lock().unwrap(), (0.02, true));
        assert_eq!(*pwm.max_duration.lock().unwrap(), 1.5);
        // value and shutdown_value are divided by the scale.
        assert_eq!(*pwm.start_value.lock().unwrap(), (0.5, 1.0));
    }

    #[test]
    fn test_a_section_without_a_name_is_reported() {
        // The prefix naming: `[pwm_tool]` carries no name to address by
        // (`pwm_tool.py:165` splits the section name for its `PIN` key).
        let (printer, _chip) = printer();
        let section = ConfigSection::new("pwm_tool", None);

        let err = PwmTool::new(&wrap(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Section 'pwm_tool' must be a '[pwm_tool <name>]' section"
        );
    }

    #[test]
    fn test_a_non_positive_cycle_time_is_reported() {
        let (printer, _chip) = printer();
        let section = section("tool", "PA1", &[("cycle_time", "0")]);

        let err = PwmTool::new(&wrap(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            // Rust's `f64` Display writes `0`, upstream Python writes `0.0`
            // (`configfile.py:54-56`); the shared formatter in
            // `config/wrapper.rs` decides this wording repo-wide.
            "Option 'cycle_time' in section 'pwm_tool tool' must be above 0"
        );
    }

    #[test]
    fn test_a_maximum_duration_below_the_minimum_is_reported() {
        let (printer, _chip) = printer();
        let section = section("tool", "PA1", &[("maximum_mcu_duration", "0.4")]);

        let err = PwmTool::new(&wrap(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'maximum_mcu_duration' in section 'pwm_tool tool' must have minimum of 0.5"
        );
    }

    #[test]
    fn test_set_pin_drives_the_pin_and_skips_a_duplicate() {
        let (printer, chip) = printer();
        let pin = PwmTool::new(&wrap(&section("tool", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=tool VALUE=1")
            .unwrap();
        assert_eq!(*created(&chip, 0).updates.lock().unwrap(), [1.0]);
        assert_eq!(pin.get_status(0.0)["value"], 1.0);

        // A repeat of the current duty sends nothing (`pwm_tool.py:168-170`).
        gcode(&printer)
            .run_script_sync("SET_PIN PIN=tool VALUE=1")
            .unwrap();
        gcode(&printer)
            .run_script_sync("SET_PIN PIN=tool VALUE=0.25")
            .unwrap();
        assert_eq!(*created(&chip, 0).updates.lock().unwrap(), [1.0, 0.25]);
    }

    #[test]
    fn test_set_pin_value_is_bounded_by_the_scale() {
        let (printer, chip) = printer();
        let section = section("tool", "PA1", &[("scale", "2")]);
        PwmTool::new(&wrap(&section), &printer).unwrap();

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=tool VALUE=2")
            .unwrap();
        // The duty is divided by the scale before it drives the pin.
        assert_eq!(*created(&chip, 0).updates.lock().unwrap(), [1.0]);

        let err = gcode(&printer)
            .run_script_sync("SET_PIN PIN=tool VALUE=3")
            .unwrap_err();
        assert!(err.to_string().contains("maximum"), "{err}");
    }

    #[test]
    fn test_set_pin_requires_a_value() {
        let (printer, _chip) = printer();
        PwmTool::new(&wrap(&section("tool", "PA1", &[])), &printer).unwrap();

        let err = gcode(&printer)
            .run_script_sync("SET_PIN PIN=tool")
            .unwrap_err();

        assert!(err.to_string().contains("missing VALUE"), "{err}");
    }

    #[test]
    fn test_a_missing_pin_names_the_section() {
        let (printer, _chip) = printer();
        let section = ConfigSection::new("pwm_tool", Some("tool"));

        let err = PwmTool::new(&wrap(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'pin' in section 'pwm_tool tool' must be specified"
        );
    }

    #[test]
    fn test_value_above_the_scale_is_reported() {
        let (printer, _chip) = printer();
        let section = section("tool", "PA1", &[("value", "1.5")]);

        let err = PwmTool::new(&wrap(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'value' in section 'pwm_tool tool' must have maximum of 1"
        );
    }

    #[test]
    fn test_an_unparseable_boolean_is_reported() {
        let (printer, _chip) = printer();
        let section = section("tool", "PA1", &[("hardware_pwm", "maybe")]);

        let err = PwmTool::new(&wrap(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Unable to parse option 'hardware_pwm' in section 'pwm_tool tool'"
        );
    }
}
