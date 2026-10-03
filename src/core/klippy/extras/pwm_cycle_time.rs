//! `[pwm_cycle_time <name>]` — a PWM pin whose cycle time a `SET_PIN` may change.
//!
//! Upstream is `klippy/extras/pwm_cycle_time.py`: an `output_pin` whose PWM is
//! always a software one (`MCU_pwm_cycle`, `pwm_cycle_time.py:7-73`) and whose
//! `SET_PIN` takes a `CYCLE_TIME=<seconds>` parameter on top of `VALUE`
//! (`pwm_cycle_time.py:114-123`). The firmware reprograms the period in place
//! (`set_digital_out_pwm_cycle`), so a client can retune the frequency while
//! the pin runs.
//!
//! | option | meaning |
//! |---|---|
//! | `pin` | the pin description, required |
//! | `scale` | full-scale figure the `VALUE` bound is written against (default 1, `> 0`) |
//! | `value` | duty at startup (default 0, `0 ..= scale`, divided by `scale`) |
//! | `shutdown_value` | duty on shutdown (default 0, `0 ..= scale`, divided by `scale`) |
//! | `cycle_time` | PWM period in seconds (default 0.1, `> 0`) |
//!
//! There is deliberately no `hardware_pwm` option: upstream's
//! `MCU_pwm_cycle` is always the software path (`pwm_cycle_time.py:7-73`
//! never asks for hardware), so the section configures the pin with
//! `hardware_pwm = false` unconditionally.
//!
//! # What is not here
//!
//! * **The runtime period change is host-side only.** Upstream's `SET_PIN
//!   CYCLE_TIME=` sends `set_digital_out_pwm_cycle` to reprogram the firmware's
//!   period (`pwm_cycle_time.py:56-62`). [`PwmOut`](crate::core::klippy::pins::PwmOut)
//!   configures the period before the build and has no runtime reprogram, so a
//!   changed `CYCLE_TIME` updates the host's bookkeeping
//!   (`setup_cycle_time`) and the duty is applied against the built period.
//!   Upstream does not queue this section either — `SET_PIN` pins the update
//!   through `register_lookahead_callback` and spaces updates by
//!   `min_schedule_time` itself (`pwm_cycle_time.py:102-122`), with no
//!   `GCodeRequestQueue` — and this port now does the same: `SET_PIN`
//!   registers a lookahead callback that lands the change at print time,
//!   spaced by `min_schedule_time`. When no `toolhead` object exists or the
//!   resource's MCU is not connected (no `min_schedule_time`), `SET_PIN`
//!   keeps an immediate fallback that drives the pin at once.
//! * **`cycle_time`'s `maxval`.** Upstream bounds it by the chip's
//!   `max_nominal_duration` (`pwm_cycle_time.py:69-70`); this port has no such
//!   figure, and the resource still refuses a period the scheduler cannot
//!   represent at build time (`PinError::PwmCycleTimeTooLarge`).

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    parse_float, sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::pins::{PrinterPins, PwmOut, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form (`[pwm_cycle_time <name>]`) exists upstream.
section!("pwm_cycle_time", order = 20, prefix = load_config_prefix);

/// The duty and period last set, for `get_status` and upstream's "a repeat of
/// the current setting sends nothing" rule (`pwm_cycle_time.py:96-98`).
#[derive(Clone, Copy, PartialEq)]
struct PinState {
    /// The duty last set, divided by `scale` (upstream's `last_value`).
    value: f64,
    /// The period last set, seconds (upstream's `last_cycle_time`).
    cycle_time: f64,
    /// The print time the last update landed at (upstream's
    /// `last_print_time`, `pwm_cycle_time.py:78`): the lookahead callback
    /// spaces the next send by `min_schedule_time` past this.
    last_print_time: f64,
}

/// One configured `[pwm_cycle_time <name>]`.
pub struct PwmCycleTime {
    /// The name in `SET_PIN PIN=<name>`: the section's sub.
    name: String,
    /// The value last set, for `get_status`; shared with the `SET_PIN` handler.
    state: Arc<Mutex<PinState>>,
}

impl PwmCycleTime {
    /// Build the pin from its section and register `SET_PIN`.
    ///
    /// # Errors
    /// Returns a config error (a message naming the section) when the section
    /// has no name, an option is missing, unparseable, out of bounds, or the
    /// pin cannot be built.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let name = config.section().sub.clone().ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{identifier}' must be a '[pwm_cycle_time <name>]' section"
            ))
        })?;

        // Upstream's read order (`pwm_cycle_time.py:74-82`): `scale`, `value`,
        // `shutdown_value`, then the pin, then `cycle_time`.
        let scale = config.get_float_bounded("scale", Some(1.0), None, None, Some(0.0), None)?;
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

        let pin_desc = config.get("pin", None)?;
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let pwm = pins
            .setup_pwm(&pin_desc, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        let cycle_time =
            config.get_float_bounded("cycle_time", Some(0.100), None, None, Some(0.0), None)?;
        // Always the software path — upstream never offers hardware here.
        pwm.setup_cycle_time(cycle_time, false);
        // Upstream's `MCU_pwm_cycle` builds `config_digital_out` with
        // `max_duration=0` (`pwm_cycle_time.py:40-42`): no return-to-shutdown
        // limit, so `value` and `shutdown_value` may differ.
        pwm.setup_max_duration(0.0);
        pwm.setup_start_value(value, shutdown_value);

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let state = Arc::new(Mutex::new(PinState {
            value,
            cycle_time,
            last_print_time: 0.0,
        }));
        let pwm = Arc::new(pwm);
        let handler: CommandHandler = {
            let pwm = Arc::clone(&pwm);
            let state = Arc::clone(&state);
            let printer_weak = Arc::downgrade(printer);
            sync(move |gcmd| cmd_set_pin(&pwm, &state, scale, cycle_time, &printer_weak, gcmd))
        };
        gcode
            .register_mux_command_with_params(
                "SET_PIN",
                "PIN",
                Some(&name),
                handler,
                Some("Set the value of an output pin"),
                // `cmd_set_pin` reads the level and then the optional cycle
                // time (`pwm_cycle_time.py:62-77`); the mux key `PIN` is
                // prepended by the registration.
                &["VALUE", "CYCLE_TIME"],
            )
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        Ok(Self { name, state })
    }

    /// The name `SET_PIN` addresses this pin by.
    pub fn name(&self) -> &str {
        &self.name
    }

    fn lock(&self) -> MutexGuard<'_, PinState> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl PrinterObject for PwmCycleTime {
    /// The value last set, as upstream's `PrinterOutputPWMCycle.get_status`.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "value": self.lock().value })
    }
}

impl std::fmt::Debug for PwmCycleTime {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PwmCycleTime")
            .field("name", &self.name)
            .finish_non_exhaustive()
    }
}

/// `SET_PIN PIN=<name> VALUE=<0..scale> [CYCLE_TIME=<seconds>]`: drive the pin.
///
/// `VALUE` is bounded by `scale` and divided by it; `CYCLE_TIME` defaults to
/// the section's period and must be `> 0` (`pwm_cycle_time.py:114-123`). A
/// request that repeats both the current duty and period sends nothing
/// (`pwm_cycle_time.py:96-98`). A changed period updates the host's cycle
/// bookkeeping — the firmware period itself is fixed at build (see the module
/// docs).
///
/// Upstream pins the update through `register_lookahead_callback` and spaces
/// sends by `min_schedule_time` (`pwm_cycle_time.py:102-122`). This port does
/// the same when a `toolhead` object exists and the resource's MCU is
/// connected (`min_schedule_time()` answers `Some`). Otherwise — no
/// `toolhead`, or the MCU not yet connected — `SET_PIN` drives the pin at once
/// as a fallback.
fn cmd_set_pin(
    pwm: &Arc<dyn PwmOut>,
    state: &Arc<Mutex<PinState>>,
    scale: f64,
    default_cycle_time: f64,
    printer: &Weak<Printer>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let value = gcmd.get_float_range("VALUE", 0.0, scale)? / scale;
    let cycle_time = gcmd.get(
        "CYCLE_TIME",
        Some(default_cycle_time),
        parse_float,
        None,
        None,
        Some(0.0),
        None,
    )?;
    // Upstream's `cmd_SET_PIN` registers a lookahead callback
    // (`pwm_cycle_time.py:118-122`). Take that path when a toolhead exists
    // and the resource can schedule (its MCU is connected); otherwise fall
    // back to the immediate path.
    if let Some(toolhead) = printer
        .upgrade()
        .and_then(|p| p.lookup_object_as::<ToolHeadObject>("toolhead"))
    {
        if pwm.min_schedule_time().is_some() {
            let pwm = Arc::clone(pwm);
            let state = Arc::clone(state);
            toolhead.register_lookahead_callback(Box::new(move |print_time| {
                set_pin_at_lookahead(&pwm, &state, print_time, value, cycle_time);
            }));
            return Ok(());
        }
    }
    // Immediate fallback (no toolhead, or MCU not connected).
    let current = state.lock().unwrap_or_else(|poison| poison.into_inner());
    if value == current.value && cycle_time == current.cycle_time {
        return Ok(());
    }
    if cycle_time != current.cycle_time {
        pwm.setup_cycle_time(cycle_time, false);
    }
    drop(current);
    pwm.update_pwm(value)
        .map_err(|err| CommandError::new(err.to_string()))?;
    *state.lock().unwrap_or_else(|poison| poison.into_inner()) = PinState {
        value,
        cycle_time,
        last_print_time: 0.0,
    };
    Ok(())
}

/// The lookahead callback: land the change at `print_time`, spaced by
/// `min_schedule_time` past the last send (upstream's `_set_pin`,
/// `pwm_cycle_time.py:102-112`).
///
/// A repeat of the current duty and period sends nothing. A changed
/// `cycle_time` updates the host's bookkeeping (`setup_cycle_time`). The duty
/// is driven at the print-time-derived clock (`set_pwm`), or at once
/// (`update_pwm`) when the clock is not available — a safety net, since the
/// lookahead path is only taken when `min_schedule_time()` is `Some`.
fn set_pin_at_lookahead(
    pwm: &Arc<dyn PwmOut>,
    state: &Arc<Mutex<PinState>>,
    print_time: f64,
    value: f64,
    cycle_time: f64,
) {
    let mut current = state.lock().unwrap_or_else(|poison| poison.into_inner());
    if value == current.value && cycle_time == current.cycle_time {
        return;
    }
    let min_schedule_time = pwm.min_schedule_time().unwrap_or(0.0);
    let print_time = f64::max(print_time, current.last_print_time + min_schedule_time);
    if cycle_time != current.cycle_time {
        pwm.setup_cycle_time(cycle_time, false);
    }
    let result = match pwm.print_time_to_clock(print_time) {
        Some(clock) => pwm.set_pwm(clock as u32, value),
        None => pwm.update_pwm(value),
    };
    if let Err(err) = result {
        warn!("SET_PIN PWM cycle_time: {err}");
    }
    *current = PinState {
        value,
        cycle_time,
        last_print_time: print_time,
    };
}

/// Upstream's `load_config_prefix` for `[pwm_cycle_time <name>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PwmCycleTime::new(config, printer)?))
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
    use crate::core::klippy::pins::{PinChip, PinError, PinParams};
    use crate::core::klippy::reactor::ManualReactor;

    /// The clock the fake resources map print time through, in Hz.
    const TEST_CLOCK_HZ: f64 = 1_000_000.0;

    /// The schedule floor the fake resources report (the real one is 0.100).
    const TEST_MIN_SCHEDULE_TIME: f64 = 0.1;

    /// A PWM that records what it was told, cycle changes included.
    ///
    /// `updates` are the immediate-path writes (`update_pwm`); `queued` are
    /// the clocked ones (`set_pwm` at `(clock, duty)`). `schedulable`
    /// models a connected MCU — a clock to convert print times with and a
    /// schedule floor.
    struct FakePwm {
        max_duration: Mutex<f64>,
        cycles: Mutex<Vec<(f64, bool)>>,
        start_value: Mutex<(f64, f64)>,
        updates: Mutex<Vec<f64>>,
        queued: Mutex<Vec<(u32, f64)>>,
        /// Whether the fake models a connected MCU.
        schedulable: bool,
    }

    impl Default for FakePwm {
        fn default() -> Self {
            Self {
                max_duration: Mutex::new(0.0),
                cycles: Mutex::new(Vec::new()),
                start_value: Mutex::new((0.0, 0.0)),
                updates: Mutex::new(Vec::new()),
                queued: Mutex::new(Vec::new()),
                schedulable: false,
            }
        }
    }

    impl PwmOut for FakePwm {
        fn setup_max_duration(&self, max_duration: f64) {
            *self.max_duration.lock().unwrap() = max_duration;
        }
        fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) {
            self.cycles.lock().unwrap().push((cycle_time, hardware_pwm));
        }
        fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
            *self.start_value.lock().unwrap() = (start_value, shutdown_value);
        }
        fn set_pwm(&self, clock: u32, value: f64) -> Result<(), McuError> {
            self.queued.lock().unwrap().push((clock, value));
            Ok(())
        }
        fn update_pwm(&self, value: f64) -> Result<(), McuError> {
            self.updates.lock().unwrap().push(value);
            Ok(())
        }
        fn next_aligned_clock(&self, clock: u32, _allow_early: f64) -> Result<u32, McuError> {
            Ok(clock)
        }
        fn print_time_to_clock(&self, print_time: f64) -> Option<u64> {
            self.schedulable
                .then_some((print_time * TEST_CLOCK_HZ) as u64)
        }
        fn min_schedule_time(&self) -> Option<f64> {
            self.schedulable.then_some(TEST_MIN_SCHEDULE_TIME)
        }
    }

    /// A chip that hands out a [`FakePwm`] per setup.
    struct FakeChip {
        pwms: Mutex<Vec<Arc<FakePwm>>>,
        /// Whether its resources model a connected MCU (the fixtures' default
        /// is `false` — the immediate fallback path).
        schedulable: bool,
    }

    impl Default for FakeChip {
        fn default() -> Self {
            Self {
                pwms: Mutex::new(Vec::new()),
                schedulable: false,
            }
        }
    }

    impl PinChip for FakeChip {
        fn setup_digital_out(
            &self,
            _params: &PinParams,
        ) -> Result<Arc<dyn crate::core::klippy::pins::DigitalOut>, PinError> {
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

    /// A ready printer with `gcode` and `pins` over the default fake chip
    /// (not schedulable — the immediate fallback path).
    fn printer() -> (Arc<Printer>, Arc<FakeChip>) {
        printer_with(FakeChip::default())
    }

    /// The same printer with a connected `toolhead` registered —
    /// `kinematics: none`, the dwell-only timeline whose flush callbacks must
    /// still run (the fixture `output_pin`'s queued tests use).
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

    /// A `[pwm_cycle_time <name>]` section with `pin: <pin>` plus `options`.
    fn section(name: &str, pin: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("pwm_cycle_time", Some(name));
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
        let section = section("cycle", "PA1", &[]);

        let pin = PwmCycleTime::new(&wrap(&section), &printer).unwrap();

        // scale 1, value 0, shutdown_value 0, cycle_time 0.1 — and the software
        // path with no max-duration limit.
        let pwm = created(&chip, 0);
        assert_eq!(*pwm.cycles.lock().unwrap(), [(0.1, false)]);
        assert_eq!(*pwm.max_duration.lock().unwrap(), 0.0);
        assert_eq!(*pwm.start_value.lock().unwrap(), (0.0, 0.0));
        assert_eq!(pin.get_status(0.0)["value"], 0.0);
    }

    #[test]
    fn test_the_section_options_reach_the_pin() {
        let (printer, chip) = printer();
        let section = section(
            "cycle",
            "PA1",
            &[
                ("scale", "2"),
                ("value", "1.0"),
                ("shutdown_value", "2.0"),
                ("cycle_time", "0.01"),
            ],
        );

        PwmCycleTime::new(&wrap(&section), &printer).unwrap();

        // value and shutdown_value are divided by the scale, as upstream does.
        let pwm = created(&chip, 0);
        assert_eq!(*pwm.start_value.lock().unwrap(), (0.5, 1.0));
        assert_eq!(*pwm.cycles.lock().unwrap(), [(0.01, false)]);
    }

    #[test]
    fn test_set_pin_drives_the_pin_and_reports_its_value() {
        let (printer, chip) = printer();
        let pin = PwmCycleTime::new(&wrap(&section("cycle", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=cycle VALUE=1")
            .unwrap();
        assert_eq!(*created(&chip, 0).updates.lock().unwrap(), [1.0]);
        assert_eq!(pin.get_status(0.0)["value"], 1.0);

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=cycle VALUE=0.5")
            .unwrap();
        assert_eq!(*created(&chip, 0).updates.lock().unwrap(), [1.0, 0.5]);
        assert_eq!(pin.get_status(0.0)["value"], 0.5);
    }

    #[test]
    fn test_a_cycle_time_parameter_reaches_the_pin_and_a_repeat_is_dropped() {
        let (printer, chip) = printer();
        PwmCycleTime::new(&wrap(&section("cycle", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=cycle VALUE=0.5 CYCLE_TIME=0.02")
            .unwrap();
        gcode(&printer)
            .run_script_sync("SET_PIN PIN=cycle VALUE=0.5 CYCLE_TIME=0.02")
            .unwrap();
        gcode(&printer)
            .run_script_sync("SET_PIN PIN=cycle VALUE=0.5 CYCLE_TIME=0.01")
            .unwrap();

        let pwm = created(&chip, 0);
        // The repeat of duty *and* period sends nothing
        // (`pwm_cycle_time.py:96-98`); a new period with the same duty still
        // updates the pin.
        assert_eq!(*pwm.updates.lock().unwrap(), [0.5, 0.5]);
        assert_eq!(
            *pwm.cycles.lock().unwrap(),
            [(0.1, false), (0.02, false), (0.01, false)]
        );
    }

    #[test]
    fn test_a_non_positive_cycle_time_parameter_is_rejected() {
        let (printer, _chip) = printer();
        PwmCycleTime::new(&wrap(&section("cycle", "PA1", &[])), &printer).unwrap();

        let err = gcode(&printer)
            .run_script_sync("SET_PIN PIN=cycle VALUE=0.5 CYCLE_TIME=0")
            .unwrap_err();

        assert!(err.to_string().contains("CYCLE_TIME"), "{err}");
        assert!(err.to_string().contains("above"), "{err}");
    }

    #[test]
    fn test_value_is_bounded_by_the_scale() {
        let (printer, _chip) = printer();
        PwmCycleTime::new(&wrap(&section("cycle", "PA1", &[])), &printer).unwrap();

        let err = gcode(&printer)
            .run_script_sync("SET_PIN PIN=cycle VALUE=1.5")
            .unwrap_err();

        assert!(err.to_string().contains("maximum"), "{err}");
    }

    #[test]
    fn test_a_missing_pin_names_the_section() {
        let (printer, _chip) = printer();
        let section = ConfigSection::new("pwm_cycle_time", Some("cycle"));

        let err = PwmCycleTime::new(&wrap(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'pin' in section 'pwm_cycle_time cycle' must be specified"
        );
    }

    #[test]
    fn test_a_section_without_a_name_is_reported() {
        let (printer, _chip) = printer();
        let section = ConfigSection::new("pwm_cycle_time", None);

        let err = PwmCycleTime::new(&wrap(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Section 'pwm_cycle_time' must be a '[pwm_cycle_time <name>]' section"
        );
    }

    #[test]
    fn test_a_non_positive_config_cycle_time_is_reported() {
        let (printer, _chip) = printer();
        let section = section("cycle", "PA1", &[("cycle_time", "0")]);

        let err = PwmCycleTime::new(&wrap(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            // Rust's `f64` Display writes `0`, upstream Python writes `0.0`
            // (`configfile.py:54-56`); the shared formatter in
            // `config/wrapper.rs` decides this wording repo-wide.
            "Option 'cycle_time' in section 'pwm_cycle_time cycle' must be above 0"
        );
    }

    #[test]
    fn test_value_above_the_scale_is_reported() {
        let (printer, _chip) = printer();
        let section = section("cycle", "PA1", &[("value", "1.5")]);

        let err = PwmCycleTime::new(&wrap(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'value' in section 'pwm_cycle_time cycle' must have maximum of 1"
        );
    }

    #[test]
    fn test_an_unparseable_option_is_reported() {
        let (printer, _chip) = printer();
        let section = section("cycle", "PA1", &[("cycle_time", "soon")]);

        let err = PwmCycleTime::new(&wrap(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Unable to parse option 'cycle_time' in section 'pwm_cycle_time cycle'"
        );
    }

    // -----------------------------------------------------------------------
    // The lookahead (print-time) path
    // -----------------------------------------------------------------------

    /// A `SET_PIN` with a toolhead and a schedulable resource lands as a
    /// **clocked** `set_pwm` dated by the toolhead's print time — never the
    /// immediate `update_pwm` — and only then does the status report the new
    /// value (upstream `last_value` moves when the change lands,
    /// `pwm_cycle_time.py:110-112`).
    #[tokio::test]
    async fn test_a_lookahead_set_pin_sends_a_clocked_change() {
        let (printer, chip) = printer_with(FakeChip {
            schedulable: true,
            ..FakeChip::default()
        });
        let toolhead = add_toolhead(&printer).await;
        let pin = PwmCycleTime::new(&wrap(&section("cycle", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=cycle VALUE=0.8")
            .await
            .unwrap();
        toolhead.flush_step_generation().await.unwrap();

        let pwm = created(&chip, 0);
        let print_time = toolhead.print_time();
        assert_eq!(
            *pwm.queued.lock().unwrap(),
            [((print_time * TEST_CLOCK_HZ) as u64 as u32, 0.8)],
            "the frame carries the print time as a clock"
        );
        assert!(
            pwm.updates.lock().unwrap().is_empty(),
            "the immediate update_pwm path is not taken"
        );
        assert_eq!(pin.get_status(0.0)["value"], 0.8);
    }

    /// Two `SET_PIN`s closer than `min_schedule_time` are spaced apart: the
    /// second lands at `last_print_time + min_schedule_time`, not at its own
    /// print time (upstream `pwm_cycle_time.py:108-109`).
    #[tokio::test]
    async fn test_two_changes_are_spaced_by_min_schedule_time() {
        let (printer, chip) = printer_with(FakeChip {
            schedulable: true,
            ..FakeChip::default()
        });
        let toolhead = add_toolhead(&printer).await;
        PwmCycleTime::new(&wrap(&section("cycle", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=cycle VALUE=0.5")
            .await
            .unwrap();
        let first_time = toolhead.print_time();
        toolhead.flush_step_generation().await.unwrap();

        // A second change with only a tiny dwell — closer than
        // `min_schedule_time` (0.1 s) — so the callback must space it.
        toolhead.dwell(0.01);
        gcode(&printer)
            .run_script("SET_PIN PIN=cycle VALUE=1.0")
            .await
            .unwrap();
        toolhead.flush_step_generation().await.unwrap();

        let pwm = created(&chip, 0);
        let queued = pwm.queued.lock().unwrap().clone();
        assert_eq!(queued.len(), 2, "{queued:?}");
        // The second clock should be at least `first_time + min_schedule_time`
        // (spaced), not at the dwell-advanced print time.
        let spaced_clock = ((first_time + TEST_MIN_SCHEDULE_TIME) * TEST_CLOCK_HZ) as u64 as u32;
        assert_eq!(
            queued[1].0, spaced_clock,
            "second change is spaced by min_schedule_time"
        );
        assert_eq!(queued[1].1, 1.0);
        assert!(pwm.updates.lock().unwrap().is_empty());
    }

    /// A `CYCLE_TIME` parameter on the lookahead path updates the host's
    /// bookkeeping and lands the duty at the print-time clock.
    #[tokio::test]
    async fn test_a_lookahead_cycle_time_parameter_updates_the_pin() {
        let (printer, chip) = printer_with(FakeChip {
            schedulable: true,
            ..FakeChip::default()
        });
        let toolhead = add_toolhead(&printer).await;
        PwmCycleTime::new(&wrap(&section("cycle", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=cycle VALUE=0.5 CYCLE_TIME=0.02")
            .await
            .unwrap();
        toolhead.flush_step_generation().await.unwrap();

        let pwm = created(&chip, 0);
        assert_eq!(
            *pwm.cycles.lock().unwrap(),
            [(0.1, false), (0.02, false)],
            "the cycle_time change reaches the host bookkeeping"
        );
        let print_time = toolhead.print_time();
        assert_eq!(
            *pwm.queued.lock().unwrap(),
            [((print_time * TEST_CLOCK_HZ) as u64 as u32, 0.5)]
        );
        assert!(pwm.updates.lock().unwrap().is_empty());
    }

    /// Repeating the value and cycle time already set is discarded in the
    /// callback: no second frame is sent
    /// (`pwm_cycle_time.py:104-105`).
    #[tokio::test]
    async fn test_a_lookahead_repeat_is_discarded() {
        let (printer, chip) = printer_with(FakeChip {
            schedulable: true,
            ..FakeChip::default()
        });
        let toolhead = add_toolhead(&printer).await;
        PwmCycleTime::new(&wrap(&section("cycle", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=cycle VALUE=0.8")
            .await
            .unwrap();
        toolhead.flush_step_generation().await.unwrap();
        assert_eq!(created(&chip, 0).queued.lock().unwrap().len(), 1);

        // A later request for the same value — the callback discards it.
        toolhead.dwell(0.25);
        gcode(&printer)
            .run_script("SET_PIN PIN=cycle VALUE=0.8")
            .await
            .unwrap();
        toolhead.flush_step_generation().await.unwrap();

        assert_eq!(
            created(&chip, 0).queued.lock().unwrap().len(),
            1,
            "the repeat sent no second frame"
        );
        assert!(created(&chip, 0).updates.lock().unwrap().is_empty());
    }

    /// No `toolhead` object (a config without `[printer]`), even though the
    /// resource could schedule: `SET_PIN` keeps the immediate fallback.
    #[test]
    fn test_without_a_toolhead_the_pin_is_set_immediately() {
        let (printer, chip) = printer_with(FakeChip {
            schedulable: true,
            ..FakeChip::default()
        });
        PwmCycleTime::new(&wrap(&section("cycle", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=cycle VALUE=1")
            .unwrap();

        let pwm = created(&chip, 0);
        assert_eq!(*pwm.updates.lock().unwrap(), [1.0]);
        assert!(pwm.queued.lock().unwrap().is_empty());
    }

    /// A toolhead, but a resource whose MCU is not connected (no
    /// `min_schedule_time`): immediate fallback, no error, no panic.
    #[tokio::test]
    async fn test_a_resource_that_cannot_schedule_is_set_immediately() {
        let (printer, chip) = printer();
        add_toolhead(&printer).await;
        PwmCycleTime::new(&wrap(&section("cycle", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=cycle VALUE=1")
            .await
            .unwrap();

        let pwm = created(&chip, 0);
        assert_eq!(*pwm.updates.lock().unwrap(), [1.0]);
        assert!(pwm.queued.lock().unwrap().is_empty());
    }
}
