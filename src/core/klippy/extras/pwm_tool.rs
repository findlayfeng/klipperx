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
//!   `shutdown_value` to match (`pwm_tool.py:52-54`); the firmware build makes
//!   the same demand (`PinError::MaxDurationMismatch`), at build rather than
//!   load time.
//!
//! # Scheduling
//!
//! `SET_PIN` rides this port's print-time request queue: the value is pinned
//! to the toolhead's lookahead time (where upstream pins it,
//! `pwm_tool.py:181-184`) and lands through the queue's sink — the shared
//! [`output_pin`](crate::core::klippy::extras::output_pin) schedule state,
//! which discards a repeat and sends `set_pwm` at the converted,
//! cycle-aligned clock. The queue's `min_schedule_time` floor replaces
//! upstream's `max(print_time, last_print_time)` (`pwm_tool.py:169-175`).
//! Two forks keep the **immediate** path (`update_pwm`), with no error and no
//! panic — the same two [`output_pin`](crate::core::klippy::extras::output_pin)
//! documents: no `toolhead` object (a config without `[printer]`), or a pin
//! resource whose `min_schedule_time()` is `None` (its MCU is not connected).
//! On the immediate path the repeat guard runs in the command
//! (`pwm_tool.py:169-171`); on the queued one the sink discards instead.
//!
//! Two upstream facts this wiring does **not** change:
//!
//! * `pwm_tool.py` itself never builds an `output_pin.GCodeRequestQueue` —
//!   it drives straight from the lookahead callback. Queueing `SET_PIN` here
//!   is this repo's direction (G2b/H2: every pin section switches onto the
//!   one ported queue), not a line-for-line port of that file's command.
//! * `maximum_mcu_duration`'s host-side refreshes
//!   (`MCU_queued_pwm._gen_intermediate_updates`, `pwm_tool.py:119-135`) ride
//!   the **motion-queue** flush callback (`pwm_tool.py:77-78`), not the
//!   g-code request queue. The firmware limit itself is configured here as
//!   before (`setup_max_duration`); the port has no host-side regeneration —
//!   a separate, resource-level gap this wiring does not close.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::output_pin::PinSchedule;
use crate::core::klippy::gcode::{sync, CommandError, CommandHandler, GCodeDispatch, GCODE_OBJECT};
use crate::core::klippy::load::section;
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form (`[pwm_tool <name>]`) exists upstream.
section!("pwm_tool", order = 20, prefix = load_config_prefix);

/// One configured `[pwm_tool <name>]`.
pub struct PwmTool {
    /// The scheduling state `SET_PIN` and `get_status` share — the one
    /// `output_pin` builds for itself.
    schedule: Arc<PinSchedule>,
}

impl PwmTool {
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
        // The status slot starts at the configured duty (upstream
        // `last_value`, `pwm_tool.py:156-157`); the schedule shares it with
        // the sink and the command.
        let schedule = Arc::new(PinSchedule::new_pwm(name.clone(), pwm, value, printer));
        let handler: CommandHandler = {
            let schedule = Arc::clone(&schedule);
            sync(move |gcmd| cmd_set_pin(&schedule, scale, gcmd))
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

        Ok(Self { schedule })
    }

    /// The name `SET_PIN` addresses this pin by.
    pub fn name(&self) -> &str {
        self.schedule.name()
    }
}

impl PrinterObject for PwmTool {
    /// The value last driven, as upstream's `PrinterOutputPin.get_status`.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "value": self.schedule.value() })
    }
}

impl std::fmt::Debug for PwmTool {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PwmTool")
            .field("name", &self.schedule.name())
            .finish_non_exhaustive()
    }
}

/// `SET_PIN PIN=<name> VALUE=<0..scale>`: queue the change at the print time
/// the toolhead gives it (upstream `cmd_SET_PIN`, `pwm_tool.py:177-184`), or
/// drive the pin at once when no timeline can date it.
///
/// `VALUE` is bounded by the pin's `scale` and divided by it before driving
/// (`pwm_tool.py:179-180`). No `CYCLE_TIME` parameter: upstream's `pwm_tool`
/// has none. A repeat of the current duty sends nothing: the queued path
/// reaches that discard in the sink (upstream `_set_pin`, `pwm_tool.py:169-171`),
/// the immediate path guards it here.
fn cmd_set_pin(
    schedule: &PinSchedule,
    scale: f64,
    gcmd: &crate::core::klippy::gcode::GcodeCommand,
) -> Result<(), CommandError> {
    let value = gcmd.get_float_range("VALUE", 0.0, scale)? / scale;
    if schedule.queue(value) {
        return Ok(());
    }
    // Immediate fallback: the queued path's sink discards a repeat, so the
    // guard lives here.
    if value == schedule.value() {
        return Ok(());
    }
    schedule
        .drive_now(value)
        .map_err(|err| CommandError::new(err.to_string()))?;
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
    use crate::core::klippy::extras::toolhead::ToolHeadObject;
    use crate::core::klippy::mcu::McuError;
    use crate::core::klippy::pins::{DigitalOut, PinChip, PinError, PinParams, PwmOut};
    use crate::core::klippy::reactor::ManualReactor;
    use std::sync::Mutex;

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
        /// `McuPwm` skips alignment while the duty is fully on/off.
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
            let last = *self.last_value.lock().unwrap();
            if hardware || last == 0.0 || last == 1.0 {
                return Ok(clock);
            }
            let cycle = (cycle_time * TEST_CLOCK_HZ) as u32;
            if cycle == 0 {
                return Ok(clock);
            }
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

    /// A ready printer with `gcode` and `pins` over the default fake chip.
    fn printer() -> (Arc<Printer>, Arc<FakeChip>) {
        printer_with(FakeChip::default())
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

    // -----------------------------------------------------------------------
    // The queued (print-time) path — see the module docs' Scheduling section
    // -----------------------------------------------------------------------

    /// A queued `SET_PIN` is pinned to the toolhead's lookahead time
    /// (`register_lookahead_callback`) and lands as a **clocked** `set_pwm`
    /// on the flush — never the immediate `update_pwm`.
    #[tokio::test]
    async fn test_a_queued_set_pin_lands_at_the_lookahead_time_on_the_flush() {
        let (printer, chip) = printer();
        let toolhead = add_toolhead(&printer).await;
        let pin = PwmTool::new(&wrap(&section("tool", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=tool VALUE=1")
            .await
            .unwrap();

        let pwm = created(&chip, 0);
        // Pinned to the lookahead and waiting in the queue: nothing driven yet.
        assert!(pwm.queued.lock().unwrap().is_empty());
        assert!(pwm.updates.lock().unwrap().is_empty());
        assert_eq!(pin.get_status(0.0)["value"], 0.0);

        let print_time = toolhead.print_time();
        toolhead.flush_step_generation().await.unwrap();

        assert_eq!(
            *pwm.queued.lock().unwrap(),
            [((print_time * TEST_CLOCK_HZ) as u64 as u32, 1.0)],
            "the frame carries the lookahead print time as a clock"
        );
        assert!(
            pwm.updates.lock().unwrap().is_empty(),
            "the immediate update_pwm path is not taken"
        );
        assert_eq!(pin.get_status(0.0)["value"], 1.0);
    }

    /// A later `SET_PIN` ahead of the same flush overrides the earlier one:
    /// the queue compresses them and only the covering duty is sent
    /// (`output_pin.py:35-38`).
    #[tokio::test]
    async fn test_a_later_set_pin_overrides_the_pending_request() {
        let (printer, chip) = printer();
        let toolhead = add_toolhead(&printer).await;
        PwmTool::new(&wrap(&section("tool", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=tool VALUE=0.25")
            .await
            .unwrap();
        gcode(&printer)
            .run_script("SET_PIN PIN=tool VALUE=1")
            .await
            .unwrap();
        toolhead.flush_step_generation().await.unwrap();

        let pwm = created(&chip, 0);
        let print_time = toolhead.print_time();
        assert_eq!(
            *pwm.queued.lock().unwrap(),
            [((print_time * TEST_CLOCK_HZ) as u64 as u32, 1.0)],
            "only the covering duty reaches the pin"
        );
        assert!(pwm.updates.lock().unwrap().is_empty());
    }

    /// First fork: no `toolhead` object (a config without `[printer]`), even
    /// though the resource could schedule — `SET_PIN` keeps the immediate
    /// path: driven at once, nothing queued.
    #[test]
    fn test_without_a_toolhead_set_pin_is_still_immediate() {
        let (printer, chip) = printer();
        PwmTool::new(&wrap(&section("tool", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script_sync("SET_PIN PIN=tool VALUE=1")
            .unwrap();

        let pwm = created(&chip, 0);
        assert_eq!(*pwm.updates.lock().unwrap(), [1.0]);
        assert!(pwm.queued.lock().unwrap().is_empty());
    }

    /// Second fork: a toolhead, but a pin resource whose MCU is not connected
    /// (no schedule floor to queue with) — immediate path, no error, no panic.
    #[tokio::test]
    async fn test_a_pin_that_cannot_schedule_is_set_immediately() {
        let (printer, chip) = printer_with(FakeChip {
            schedulable: false,
            ..FakeChip::default()
        });
        add_toolhead(&printer).await;
        PwmTool::new(&wrap(&section("tool", "PA1", &[])), &printer).unwrap();

        gcode(&printer)
            .run_script("SET_PIN PIN=tool VALUE=1")
            .await
            .unwrap();

        let pwm = created(&chip, 0);
        assert_eq!(*pwm.updates.lock().unwrap(), [1.0]);
        assert!(pwm.queued.lock().unwrap().is_empty());
    }
}
