//! `[buttons]` — the button/switch registry (upstream `klippy/extras/buttons.py`).
//!
//! The module object a consumer loads by name (`printer.load_object(config,
//! 'buttons')`) rather than from a section of its own. Only the two entry
//! points the filament sensors use are here: [`PrinterButtons::register_debounce_button`]
//! (a single debounced switch, the filament *switch* sensor) and
//! [`PrinterButtons::register_buttons`] (one or more pins sharing a callback,
//! the filament *motion* sensor's encoder).
//!
//! Both record the registration; the pins come in as the string the caller read
//! from its own `switch_pin` option, so the option is read through the caller's
//! wrapper exactly as upstream reads it.
//!
//! # What is not here
//!
//! The firmware side. Upstream's `MCU_buttons` configures `config_buttons` /
//! `buttons_add` / `buttons_query` on the MCU and handles `buttons_state`, so a
//! registered callback fires when the pin changes. This port has no button
//! query: a callback is stored but never called. That is faithful on the
//! regression corpus, whose fake firmware never generates a button event, so a
//! sensor's `_button_handler` / `encoder_event` is never reached either way.

use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The name the sensors look the registry up by (`load_object(config,
/// 'buttons')`).
pub const BUTTONS_OBJECT: &str = "buttons";

/// A registered button state callback: `(eventtime, state)`, the signature
/// upstream's `_button_handler` / `encoder_event` take.
pub type ButtonCallback = Box<dyn Fn(f64, bool) + Send + Sync>;

/// One registration: the pins it watches and the callback to run.
struct ButtonRegistration {
    pins: Vec<String>,
    /// The handler to run on a state change. Retained (not invoked) because the
    /// firmware button query that would call it is not implemented — the module
    /// docs explain why a registration is still recorded.
    #[allow(dead_code)]
    callback: ButtonCallback,
}

/// The `[buttons]` module object (upstream's `PrinterButtons`).
#[derive(Default)]
pub struct PrinterButtons {
    registrations: Mutex<Vec<ButtonRegistration>>,
}

impl PrinterButtons {
    /// The single `buttons` object; the first caller creates it, as upstream's
    /// `printer.load_object(config, 'buttons')` does.
    ///
    /// # Errors
    /// A duplicate registration (a name already taken).
    pub fn ensure(printer: &Arc<Printer>) -> Result<Arc<PrinterButtons>, ConfigError> {
        if let Some(existing) = printer.lookup_object_as::<PrinterButtons>(BUTTONS_OBJECT) {
            return Ok(existing);
        }
        let object = Arc::new(PrinterButtons::default());
        printer.add_object(
            BUTTONS_OBJECT,
            Arc::clone(&object) as Arc<dyn PrinterObject>,
        )?;
        Ok(object)
    }

    /// Upstream's `register_debounce_button` (`buttons.py:296-298`): read the
    /// section's `debounce_delay` and register a single pin.
    ///
    /// The debounce delay is read from `config` (the caller's section), as
    /// upstream's `DebounceButton` reads it (`buttons.py:255`).
    ///
    /// # Errors
    /// A negative or unparsable `debounce_delay`.
    pub fn register_debounce_button(
        &self,
        config: &ConfigWrapper,
        pin: &str,
        callback: ButtonCallback,
    ) -> Result<(), ConfigError> {
        config.get_float_bounded("debounce_delay", Some(0.0), Some(0.0), None, None, None)?;
        self.record(vec![pin.to_string()], callback);
        Ok(())
    }

    /// Upstream's `register_buttons` (`buttons.py:309`): one or more pins that
    /// share a callback (a motion sensor's encoder drives one pin).
    pub fn register_buttons(&self, pins: Vec<String>, callback: ButtonCallback) {
        self.record(pins, callback);
    }

    fn record(&self, pins: Vec<String>, callback: ButtonCallback) {
        self.lock().push(ButtonRegistration { pins, callback });
    }

    /// How many registrations have been recorded (tests and diagnostics).
    pub fn registration_count(&self) -> usize {
        self.lock().len()
    }

    /// The registered pin groups, in registration order (tests and
    /// diagnostics); a debounced single button is one group of one pin.
    pub fn registered_pins(&self) -> Vec<Vec<String>> {
        self.lock().iter().map(|entry| entry.pins.clone()).collect()
    }

    fn lock(&self) -> MutexGuard<'_, Vec<ButtonRegistration>> {
        self.registrations
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl std::fmt::Debug for PrinterButtons {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterButtons")
            .field("registrations", &self.registration_count())
            .finish()
    }
}

impl PrinterObject for PrinterButtons {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::reactor::ManualReactor;

    fn printer() -> Arc<Printer> {
        Arc::new(Printer::new(ManualReactor::shared()))
    }

    /// The first `ensure` creates the object, the second returns the same one —
    /// upstream's `load_object` caches by name.
    #[test]
    fn test_ensure_returns_one_shared_object() {
        let printer = printer();
        let first = PrinterButtons::ensure(&printer).expect("the object registers");
        let second = PrinterButtons::ensure(&printer).expect("the object is reused");
        assert!(Arc::ptr_eq(&first, &second));
    }

    /// `register_debounce_button` reads the section's `debounce_delay` (so the
    /// option is accepted) and records the registration.
    #[test]
    fn test_register_debounce_button_reads_the_delay_and_records() {
        let printer = printer();
        let buttons = PrinterButtons::ensure(&printer).expect("the object registers");
        let (config, _) =
            Config::from_text("[buttons_test]\nswitch_pin: PA0\ndebounce_delay: 0.01\n")
                .expect("the config parses");
        let section = config
            .get_section("buttons_test")
            .expect("the section exists");
        let wrapper = ConfigWrapper::untracked(section);

        buttons
            .register_debounce_button(&wrapper, "PA0", Box::new(|_, _| {}))
            .expect("the delay is accepted");
        assert_eq!(buttons.registration_count(), 1);
    }

    /// A negative debounce delay is refused with the config wording, as
    /// upstream's `getfloat(..., minval=0.)` does.
    #[test]
    fn test_a_negative_debounce_delay_is_refused() {
        let printer = printer();
        let buttons = PrinterButtons::ensure(&printer).expect("the object registers");
        let (config, _) =
            Config::from_text("[buttons_test]\ndebounce_delay: -1\n").expect("the config parses");
        let section = config
            .get_section("buttons_test")
            .expect("the section exists");
        let wrapper = ConfigWrapper::untracked(section);

        let err = buttons
            .register_debounce_button(&wrapper, "PA0", Box::new(|_, _| {}))
            .unwrap_err()
            .to_string();
        assert!(err.contains("must have minimum of"), "{err}");
    }

    /// `register_buttons` records every pin of one registration, as upstream
    /// groups them.
    #[test]
    fn test_register_buttons_records_the_group() {
        let printer = printer();
        let buttons = PrinterButtons::ensure(&printer).expect("the object registers");
        buttons.register_buttons(
            vec!["PA0".to_string(), "PA1".to_string()],
            Box::new(|_, _| {}),
        );
        assert_eq!(buttons.registration_count(), 1);
    }
}
