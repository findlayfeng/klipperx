//! `[multi_pin <name>]` — one virtual pin that drives several real ones.
//!
//! Upstream's `klippy/extras/multi_pin.py`: the section lists `pins`, registers
//! a single `multi_pin` chip with the pin registry, and every consumer of
//! `multi_pin:<name>` gets back a fan-out object that repeats each call on every
//! pin in the list. The usual use is a heater or fan whose current is spread
//! across several outputs (the corpus uses it for `heater_pin` and a fan `pin`).
//!
//! | option | meaning |
//! |---|---|
//! | `pins` | comma-separated pin descriptions to drive, required |
//!
//! A consumer's description is `multi_pin:<name>`: the part after the colon is
//! the section's own sub, **not** a pin on an MCU. So `setup_pwm` /
//! `setup_digital_out` hand the sub to `lookup_object_as::<PrinterMultiPin>`,
//! exactly as upstream's `setup_pin` looks up `'multi_pin ' + pin_name`.
//!
//! # Load phase
//!
//! The section is `phase = early`, and unlike `[adc_scaled]` the plain `order`
//! is not enough. This loader walks a phase's **regular** sections first and its
//! **prefix** sections second (`load.rs:215-236`), while upstream loads *every*
//! prefix section before the generic walk (`klippy/klippy.py:111-121`). A
//! `[multi_pin …]` is prefix-only, and the sections that consume it — `[extruder]`
//! (`heater_pin: multi_pin:heater`) and `[heater_fan]` (`pin:
//! multi_pin:extruder_fans`) — are regular sections. With `order` alone the
//! consumer would still run before the provider and report `Unknown pin chip
//! name 'multi_pin'`; `phase = early` puts the registrations in an earlier phase,
//! where all of them load before any consumer. (`[mcu]` and `[adc_scaled]` are
//! early for the same reason.)
//!
//! # One chip, several sections
//!
//! Upstream registers the chip once per section and swallows the second
//! registration (`multi_pin.py:11-14`: `except ppins.error: pass`), so a config
//! may define several `[multi_pin …]` sections. Here the **first** section
//! becomes the registered dispatcher; later registration attempts get
//! [`PinError::DuplicateChip`] and ignore it. A call for another section's name
//! is forwarded to that section's object, and only the object whose name matches
//! builds the fan-out (and refuses a second build with `Can't setup multi_pin
//! <name> twice`).
//!
//! # What is not here
//!
//! * **`get_mcu`.** Upstream forwards it to the first child pin, which the host
//!   uses to place a pin on its MCU. This host's resources carry their MCU
//!   themselves and no caller asks a [`PwmOut`] / [`DigitalOut`] for one, so
//!   there is nothing to forward to.
//! * **Stepper pins.** The alias must not be used for a stepper's step/dir
//!   (`docs/Config_Reference.md` in upstream): those go through the stepper chip
//!   dispatch, which this virtual chip does not implement, so such a config
//!   reports the ordinary unsupported-type error.

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::McuError;
use crate::core::klippy::pins::{
    DigitalOut, PinChip, PinError, PinParams, PrinterPins, PwmOut, PINS_OBJECT,
};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The chip name every `[multi_pin …]` registers (`multi_pin.py:11`).
const CHIP_NAME: &str = "multi_pin";

// Only the prefix form (`[multi_pin <name>]`) exists upstream
// (`multi_pin.py:55-56`); `phase = early` is load-bearing, see the module docs.
section!(
    "multi_pin",
    order = 15,
    phase = early,
    prefix = load_config_prefix
);

/// One `[multi_pin <name>]`: the chip name it answers to plus the list of pins
/// to fan out to.
pub struct PrinterMultiPin {
    /// The section's sub: the `<name>` in `multi_pin:<name>`.
    name: String,
    /// The full section identifier, for logging and errors.
    identifier: String,
    /// The printer, weakly held: the object is stored in the printer's own
    /// registry and (once) in the pins registry, so a strong handle would be a
    /// cycle.
    printer: Weak<Printer>,
    /// The pins to drive, in the order written.
    pin_list: Vec<String>,
    /// Whether a resource has already been built for this name.
    state: Mutex<MultiPinState>,
}

/// The one-shot state upstream keeps in `pin_type`: `None` until the first
/// `setup_pin`, which is what makes the second one an error.
#[derive(Default)]
struct MultiPinState {
    configured: bool,
}

impl PrinterMultiPin {
    /// Read `pins`, register the `multi_pin` chip (ignoring a duplicate), and
    /// return the section's object.
    ///
    /// # Errors
    /// Returns a config error when `pins` is missing, when the pins object is
    /// absent, or when the chip name is taken by something other than an earlier
    /// `[multi_pin …]` (which is not possible, but keeps the error honest).
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let identifier = config.identifier();
        let name = config
            .section()
            .sub
            .clone()
            .ok_or_else(|| ConfigError::new(format!("[{identifier}] is missing a name")))?;
        let pin_list = config.get_list("pins", ',').ok_or_else(|| {
            ConfigError::new(format!(
                "Option 'pins' in section '{identifier}' must be specified"
            ))
        })?;

        let object = Arc::new(Self {
            name,
            identifier: identifier.clone(),
            printer: Arc::downgrade(printer),
            pin_list,
            state: Mutex::new(MultiPinState::default()),
        });

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        match pins.register_chip(CHIP_NAME, Arc::clone(&object) as Arc<dyn PinChip>) {
            Ok(()) => {}
            // Upstream's `except ppins.error: pass` (`multi_pin.py:11-14`): a
            // second `[multi_pin …]` must load even though the chip is taken.
            Err(PinError::DuplicateChip(_)) => {}
            Err(err) => return Err(ConfigError::new(format!("{identifier}: {err}"))),
        }
        Ok(object)
    }

    /// The section's sub.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The pins this section drives.
    pub fn pin_list(&self) -> &[String] {
        &self.pin_list
    }

    /// The printer's pin registry.
    fn pins(&self) -> Result<Arc<PrinterPins>, PinError> {
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| PinError::Message("the printer is gone".to_string()))?;
        printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .ok_or_else(|| PinError::Message("the pins object is not registered".to_string()))
    }

    /// Decide where a `multi_pin:<pin_params.pin>` call lands.
    ///
    /// Upstream's `setup_pin` body (`multi_pin.py:19-25`): the named object is
    /// either this one (build here), another `[multi_pin …]` (build there), or
    /// missing (report it).
    fn dispatch(&self, pin_name: &str) -> Result<Dispatch, PinError> {
        if pin_name == self.name {
            return Ok(Dispatch::Here);
        }
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| PinError::Message("the printer is gone".to_string()))?;
        match printer.lookup_object_as::<PrinterMultiPin>(&format!("{CHIP_NAME} {pin_name}")) {
            Some(other) => Ok(Dispatch::There(other)),
            None => Err(PinError::Message(format!(
                "{CHIP_NAME} {pin_name} not configured"
            ))),
        }
    }

    /// Mark this section as built, refusing the second call
    /// (`multi_pin.py:27-28`).
    fn claim(&self) -> Result<(), PinError> {
        let mut state = self.lock();
        if state.configured {
            return Err(PinError::Message(format!(
                "Can't setup {CHIP_NAME} {} twice",
                self.name
            )));
        }
        state.configured = true;
        Ok(())
    }

    /// The `!` upstream puts in front of every child when the consumer's pin was
    /// inverted (`multi_pin.py:29-32`).
    fn child_description(&self, pin_desc: &str, invert: bool) -> String {
        if invert {
            format!("!{pin_desc}")
        } else {
            pin_desc.to_string()
        }
    }

    fn lock(&self) -> MutexGuard<'_, MultiPinState> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

/// Where a named `multi_pin:` call is built.
enum Dispatch {
    /// The name is this object's own sub.
    Here,
    /// The name belongs to another `[multi_pin …]`.
    There(Arc<PrinterMultiPin>),
}

impl PrinterObject for PrinterMultiPin {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl PinChip for PrinterMultiPin {
    fn setup_digital_out(&self, params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
        match self.dispatch(&params.pin)? {
            Dispatch::There(other) => other.setup_digital_out(params),
            Dispatch::Here => {
                self.claim()?;
                let pins = self.pins()?;
                let mut children = Vec::with_capacity(self.pin_list.len());
                for pin_desc in &self.pin_list {
                    let desc = self.child_description(pin_desc, params.invert);
                    children.push(pins.setup_digital_out(&desc, None)?);
                }
                Ok(Arc::new(MultiPinDigital { pins: children }))
            }
        }
    }

    fn setup_pwm(&self, params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
        match self.dispatch(&params.pin)? {
            Dispatch::There(other) => other.setup_pwm(params),
            Dispatch::Here => {
                self.claim()?;
                let pins = self.pins()?;
                let mut children = Vec::with_capacity(self.pin_list.len());
                for pin_desc in &self.pin_list {
                    let desc = self.child_description(pin_desc, params.invert);
                    children.push(pins.setup_pwm(&desc, None)?);
                }
                Ok(Arc::new(MultiPinPwm { pins: children }))
            }
        }
    }
}

impl std::fmt::Debug for PrinterMultiPin {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterMultiPin")
            .field("identifier", &self.identifier)
            .field("pin_list", &self.pin_list)
            .finish()
    }
}

/// Upstream's fan-out object for a PWM consumer: every call repeats on each
/// child pin (`multi_pin.py:35-53`).
struct MultiPinPwm {
    pins: Vec<Arc<dyn PwmOut>>,
}

impl PwmOut for MultiPinPwm {
    fn setup_max_duration(&self, max_duration: f64) {
        for pin in &self.pins {
            pin.setup_max_duration(max_duration);
        }
    }

    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) {
        for pin in &self.pins {
            pin.setup_cycle_time(cycle_time, hardware_pwm);
        }
    }

    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        for pin in &self.pins {
            pin.setup_start_value(start_value, shutdown_value);
        }
    }

    fn set_pwm(&self, clock: u32, value: f64) -> Result<(), McuError> {
        for pin in &self.pins {
            pin.set_pwm(clock, value)?;
        }
        Ok(())
    }

    fn update_pwm(&self, value: f64) -> Result<(), McuError> {
        for pin in &self.pins {
            pin.update_pwm(value)?;
        }
        Ok(())
    }

    /// Upstream's `next_aligned_print_time` returns its argument unchanged
    /// (`multi_pin.py:49-50`): a fan-out has no single cycle to align to, so it
    /// leaves the choice to each child.
    fn next_aligned_clock(&self, clock: u32, _allow_early: f64) -> Result<u32, McuError> {
        Ok(clock)
    }
}

/// Upstream's fan-out object for a digital consumer (`multi_pin.py:43-45`).
struct MultiPinDigital {
    pins: Vec<Arc<dyn DigitalOut>>,
}

impl DigitalOut for MultiPinDigital {
    fn setup_max_duration(&self, max_duration: f64) {
        for pin in &self.pins {
            pin.setup_max_duration(max_duration);
        }
    }

    fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
        for pin in &self.pins {
            pin.setup_start_value(start_value, shutdown_value);
        }
    }

    fn queue_digital_out(&self, clock: u32, value: bool) -> Result<(), McuError> {
        for pin in &self.pins {
            pin.queue_digital_out(clock, value)?;
        }
        Ok(())
    }

    fn update_digital_out(&self, value: bool) -> Result<(), McuError> {
        for pin in &self.pins {
            pin.update_digital_out(value)?;
        }
        Ok(())
    }
}

/// Upstream's `load_config_prefix` for `[multi_pin <name>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = PrinterMultiPin::new(config, printer)?;
    Ok(object as Arc<dyn PrinterObject>)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::section::ConfigSection;
    use crate::core::klippy::config::value::ConfigValue;
    use crate::core::klippy::reactor::ManualReactor;
    use std::sync::Mutex;

    /// Shared call log the fake chip and its pins append to.
    #[derive(Default)]
    struct Log(Mutex<Vec<String>>);

    impl Log {
        fn push(&self, line: String) {
            self.0.lock().unwrap().push(line);
        }

        fn take(&self) -> Vec<String> {
            std::mem::take(&mut *self.0.lock().unwrap())
        }
    }

    /// The pin's label as written: `PB4`, or `!PB4` when inverted.
    fn label(params: &PinParams) -> String {
        if params.invert {
            format!("!{}", params.pin)
        } else {
            params.pin.clone()
        }
    }

    /// A pin resource that records every call it receives.
    struct FakePin {
        label: String,
        log: Arc<Log>,
    }

    impl PwmOut for FakePin {
        fn setup_max_duration(&self, max_duration: f64) {
            self.log
                .push(format!("{} max_duration {max_duration}", self.label));
        }

        fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) {
            self.log.push(format!(
                "{} cycle_time {cycle_time} {hardware_pwm}",
                self.label
            ));
        }

        fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
            self.log.push(format!(
                "{} start_value {start_value} {shutdown_value}",
                self.label
            ));
        }

        fn set_pwm(&self, clock: u32, value: f64) -> Result<(), McuError> {
            self.log
                .push(format!("{} set_pwm {clock} {value}", self.label));
            Ok(())
        }

        fn update_pwm(&self, value: f64) -> Result<(), McuError> {
            self.log.push(format!("{} update_pwm {value}", self.label));
            Ok(())
        }

        fn next_aligned_clock(&self, clock: u32, _allow_early: f64) -> Result<u32, McuError> {
            Ok(clock)
        }
    }

    impl DigitalOut for FakePin {
        fn setup_max_duration(&self, max_duration: f64) {
            self.log
                .push(format!("{} max_duration {max_duration}", self.label));
        }

        fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
            self.log.push(format!(
                "{} start_value {start_value} {shutdown_value}",
                self.label
            ));
        }

        fn queue_digital_out(&self, clock: u32, value: bool) -> Result<(), McuError> {
            self.log
                .push(format!("{} queue_digital_out {clock} {value}", self.label));
            Ok(())
        }

        fn update_digital_out(&self, value: bool) -> Result<(), McuError> {
            self.log
                .push(format!("{} update_digital_out {value}", self.label));
            Ok(())
        }
    }

    /// An MCU chip that hands out a [`FakePin`] per setup and logs the setup.
    struct FakeChip {
        log: Arc<Log>,
    }

    impl PinChip for FakeChip {
        fn setup_digital_out(&self, params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            self.log.push(format!("digital {}", label(params)));
            Ok(Arc::new(FakePin {
                label: label(params),
                log: Arc::clone(&self.log),
            }))
        }

        fn setup_pwm(&self, params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
            self.log.push(format!("pwm {}", label(params)));
            Ok(Arc::new(FakePin {
                label: label(params),
                log: Arc::clone(&self.log),
            }))
        }
    }

    /// A printer whose `mcu` chip is a [`FakeChip`].
    fn machine() -> (Arc<Printer>, Arc<Log>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let pins = Arc::new(PrinterPins::new());
        let log = Arc::new(Log::default());
        pins.register_chip(
            "mcu",
            Arc::new(FakeChip {
                log: Arc::clone(&log),
            }),
        )
        .unwrap();
        printer.add_object(PINS_OBJECT, pins).unwrap();
        (printer, log)
    }

    /// A `[multi_pin <name>]` section with the given options.
    fn section(name: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("multi_pin", Some(name));
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// Load a `[multi_pin <name>]` section and register it under the name a
    /// consumer would find (`multi_pin <name>`).
    fn add_multi_pin(printer: &Arc<Printer>, name: &str, pins: &str) -> Arc<PrinterMultiPin> {
        let section = section(name, &[("pins", pins)]);
        let object = PrinterMultiPin::new(&ConfigWrapper::untracked(&section), printer).unwrap();
        let registered: Arc<dyn PrinterObject> = object.clone();
        printer
            .add_object(&format!("multi_pin {name}"), registered)
            .unwrap();
        object
    }

    fn pins(printer: &Arc<Printer>) -> Arc<PrinterPins> {
        printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .unwrap()
    }

    #[test]
    fn test_two_sections_both_load_and_the_chip_registers_once() {
        let (printer, _log) = machine();

        let heater = add_multi_pin(&printer, "heater", "PB4,PB5,PB0");
        let fans = add_multi_pin(&printer, "extruder_fans", "PB7,PB8,PB9");

        assert_eq!(heater.name(), "heater");
        assert_eq!(heater.pin_list(), ["PB4", "PB5", "PB0"]);
        assert_eq!(fans.pin_list(), ["PB7", "PB8", "PB9"]);
        assert!(printer.objects().contains(&"multi_pin heater".to_string()));
        assert!(printer
            .objects()
            .contains(&"multi_pin extruder_fans".to_string()));
        // The second `register_chip` was refused and ignored, so the name is
        // registered exactly once.
        assert_eq!(
            pins(&printer)
                .chips()
                .iter()
                .filter(|chip| chip.as_str() == CHIP_NAME)
                .count(),
            1
        );
    }

    #[test]
    fn test_setup_pwm_fans_out_to_every_child() {
        let (printer, log) = machine();
        add_multi_pin(&printer, "heater", "PB4,PB5,PB0");

        let pwm = pins(&printer).setup_pwm("multi_pin:heater", None).unwrap();
        assert_eq!(log.take(), ["pwm PB4", "pwm PB5", "pwm PB0"]);

        pwm.setup_max_duration(0.5);
        pwm.setup_cycle_time(0.1, false);
        pwm.setup_start_value(0.0, 0.0);
        pwm.set_pwm(100, 0.5).unwrap();
        pwm.update_pwm(0.75).unwrap();

        assert_eq!(
            log.take(),
            [
                "PB4 max_duration 0.5",
                "PB5 max_duration 0.5",
                "PB0 max_duration 0.5",
                "PB4 cycle_time 0.1 false",
                "PB5 cycle_time 0.1 false",
                "PB0 cycle_time 0.1 false",
                "PB4 start_value 0 0",
                "PB5 start_value 0 0",
                "PB0 start_value 0 0",
                "PB4 set_pwm 100 0.5",
                "PB5 set_pwm 100 0.5",
                "PB0 set_pwm 100 0.5",
                "PB4 update_pwm 0.75",
                "PB5 update_pwm 0.75",
                "PB0 update_pwm 0.75",
            ]
        );
    }

    #[test]
    fn test_setup_digital_out_fans_out_to_every_child() {
        let (printer, log) = machine();
        add_multi_pin(&printer, "heater", "PB4,PB5");

        let out = pins(&printer)
            .setup_digital_out("multi_pin:heater", None)
            .unwrap();
        out.setup_max_duration(0.5);
        out.setup_start_value(true, false);
        out.queue_digital_out(100, true).unwrap();
        out.update_digital_out(false).unwrap();

        assert_eq!(
            log.take(),
            [
                "digital PB4",
                "digital PB5",
                "PB4 max_duration 0.5",
                "PB5 max_duration 0.5",
                "PB4 start_value true false",
                "PB5 start_value true false",
                "PB4 queue_digital_out 100 true",
                "PB5 queue_digital_out 100 true",
                "PB4 update_digital_out false",
                "PB5 update_digital_out false",
            ]
        );
    }

    #[test]
    fn test_next_aligned_clock_is_unchanged() {
        let (printer, _log) = machine();
        add_multi_pin(&printer, "heater", "PB4");

        let pwm = pins(&printer).setup_pwm("multi_pin:heater", None).unwrap();

        assert_eq!(pwm.next_aligned_clock(500, 0.02).unwrap(), 500);
    }

    #[test]
    fn test_a_call_for_another_section_is_forwarded() {
        let (printer, log) = machine();
        // The dispatcher is whichever section registered first.
        add_multi_pin(&printer, "heater", "PB4,PB5,PB0");
        add_multi_pin(&printer, "extruder_fans", "PB7,PB8,PB9");

        let pwm = pins(&printer)
            .setup_pwm("multi_pin:extruder_fans", None)
            .unwrap();
        assert_eq!(log.take(), ["pwm PB7", "pwm PB8", "pwm PB9"]);

        pwm.update_pwm(1.0).unwrap();
        assert_eq!(
            log.take(),
            ["PB7 update_pwm 1", "PB8 update_pwm 1", "PB9 update_pwm 1",]
        );
    }

    #[test]
    fn test_a_missing_section_is_reported() {
        let (printer, _log) = machine();
        add_multi_pin(&printer, "heater", "PB4");

        let err = pins(&printer)
            .setup_pwm("multi_pin:missing", None)
            .err()
            .unwrap();

        assert_eq!(err.to_string(), "multi_pin missing not configured");
    }

    #[test]
    fn test_a_second_setup_is_reported() {
        let (printer, _log) = machine();
        add_multi_pin(&printer, "heater", "PB4");
        // A second lookup of the same description reaches the chip only when the
        // caller names a share type (upstream's `lookup_pin`); without one the
        // pin registry itself refuses it as "used multiple times".
        pins(&printer)
            .setup_pwm("multi_pin:heater", Some("heater"))
            .unwrap();

        let err = pins(&printer)
            .setup_pwm("multi_pin:heater", Some("heater"))
            .err()
            .unwrap();

        assert_eq!(err.to_string(), "Can't setup multi_pin heater twice");
    }

    #[test]
    fn test_a_missing_pins_option_is_reported() {
        let (printer, _log) = machine();
        let section = section("heater", &[]);

        let err = PrinterMultiPin::new(&ConfigWrapper::untracked(&section), &printer).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'pins' in section 'multi_pin heater' must be specified"
        );
    }

    #[test]
    fn test_an_inverted_consumer_inverts_every_child() {
        let (printer, log) = machine();
        add_multi_pin(&printer, "heater", "PB4,PB5");

        pins(&printer).setup_pwm("!multi_pin:heater", None).unwrap();

        assert_eq!(log.take(), ["pwm !PB4", "pwm !PB5"]);
    }

    #[test]
    fn test_the_object_reports_no_status() {
        let (printer, _log) = machine();
        let object = add_multi_pin(&printer, "heater", "PB4");

        assert_eq!(object.get_status(0.0), json!({}));
        assert!(!object.is_queryable());
    }
}
