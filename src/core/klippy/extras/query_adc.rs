//! `query_adc` — the analog inputs a module registered, reported by
//! `QUERY_ADC` (upstream `klippy/extras/query_adc.py`).
//!
//! The module object a consumer loads by name (`printer.load_object(config,
//! 'query_adc')`) rather than from a section of its own, like `[buttons]`. A
//! consumer that builds an analog input hands it here under a name
//! ([`QueryAdc::register_adc`]); `QUERY_ADC [NAME=] [PULLUP=]` then reports that
//! input's last sample, or — with no name, or one nobody registered — the names
//! that are available (`query_adc.py:16-34`).
//!
//! The value is the sample as the ADC resource scaled it (`0.0..=1.0`), and its
//! timestamp is the print time the resource dated it with. `PULLUP` asks for the
//! resistance the sample implies across a pull-up of that many ohms:
//! `r = pullup * v / (1 - v)` with `v` clamped away from both ends
//! (`query_adc.py:28-32`).
//!
//! # What registers
//!
//! `[buttons]` registers each analog button input as `adc_button:<pin>`
//! (`buttons.py:113-114`); that is the whole set today — the other upstream
//! registrations (`adc_temperature`, `adc_scaled`) are not made by this host's
//! ports of those modules.
//!
//! Unlike upstream's `MCU_adc`, which always has a `(0., 0.)` last value to
//! report, this host's [`Adc::get_last_value`] answers `None` until a report has
//! arrived, and a query for such an input says so instead of printing a made-up
//! zero.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::config::ConfigError;
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::pins::Adc;
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The name a consumer looks the registry up by (`load_object(config,
/// 'query_adc')`).
pub const QUERY_ADC_OBJECT: &str = "query_adc";

/// `cmd_QUERY_ADC_help` (`query_adc.py:17`).
const QUERY_ADC_HELP: &str = "Report the last value of an analog pin";

/// Upstream's clamp on a sample before the pull-up divider is applied
/// (`query_adc.py:29`).
const MIN_ADC: f64 = 0.00001;
const MAX_ADC: f64 = 0.99999;

/// The analog inputs, by the name their consumer registered them under.
pub struct QueryAdc {
    adc: Mutex<HashMap<String, Arc<dyn Adc>>>,
}

impl QueryAdc {
    /// The single `query_adc` object; the first caller creates it, as upstream's
    /// `printer.load_object(config, 'query_adc')` does — `[buttons]` is that
    /// caller (`buttons.py:287`).
    ///
    /// # Errors
    /// A duplicate registration (a name already taken), or a command table that
    /// already answers `QUERY_ADC`.
    pub fn ensure(printer: &Arc<Printer>) -> Result<Arc<QueryAdc>, ConfigError> {
        if let Some(existing) = printer.lookup_object_as::<QueryAdc>(QUERY_ADC_OBJECT) {
            return Ok(existing);
        }
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let object = Arc::new(QueryAdc {
            adc: Mutex::new(HashMap::new()),
        });
        // Weakly, as `gcode_button`'s `QUERY_BUTTON` does: the object holds the
        // dispatcher's table entry, so a strong capture here would close the
        // loop and keep the table (and everything it captured) alive past
        // `teardown` (`VirtualSdCard::register_commands` has the full story).
        let registry = Arc::downgrade(&object);
        let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
            let Some(registry) = registry.upgrade() else {
                return Err(CommandError::new("the query_adc object is gone"));
            };
            cmd_query_adc(&registry, gcmd)
        });
        gcode
            .register_command("QUERY_ADC", handler, Some(QUERY_ADC_HELP), false)
            .map_err(ConfigError::new)?;
        printer.add_object(
            QUERY_ADC_OBJECT,
            Arc::clone(&object) as Arc<dyn PrinterObject>,
        )?;
        Ok(object)
    }

    /// Upstream's `register_adc` (`query_adc.py:14-15`): a plain dictionary
    /// assignment, so registering the same name twice keeps the last input.
    pub fn register_adc(&self, name: &str, adc: Arc<dyn Adc>) {
        self.lock().insert(name.to_string(), adc);
    }

    /// The registered names, sorted (diagnostics and tests).
    pub fn names(&self) -> Vec<String> {
        let mut names: Vec<String> = self.lock().keys().cloned().collect();
        names.sort();
        names
    }

    fn lock(&self) -> MutexGuard<'_, HashMap<String, Arc<dyn Adc>>> {
        self.adc.lock().unwrap_or_else(|poison| poison.into_inner())
    }
}

impl std::fmt::Debug for QueryAdc {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("QueryAdc")
            .field("names", &self.names())
            .finish()
    }
}

impl PrinterObject for QueryAdc {
    /// Upstream has no `get_status` here; the object is a lookup table.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }
}

/// Upstream's `cmd_QUERY_ADC` (`query_adc.py:19-33`).
fn cmd_query_adc(registry: &QueryAdc, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let name = gcmd.get_str_default("NAME", "");
    let adc = registry.lock().get(&name).cloned();
    let Some(adc) = adc else {
        // Upstream keys off the same lookup: a missing `NAME` and an unknown one
        // both fall through to the list (`query_adc.py:20-24`).
        let objects: Vec<String> = registry
            .names()
            .into_iter()
            .map(|name| format!("\"{name}\""))
            .collect();
        gcmd.respond_info(&format!("Available ADC objects: {}", objects.join(", ")));
        return Ok(());
    };
    let Some((timestamp, value)) = adc.get_last_value() else {
        gcmd.respond_info(&format!(
            "ADC object \"{name}\" has not reported a value yet"
        ));
        return Ok(());
    };
    let mut msg = format!("ADC object \"{name}\" has value {value:.6} (timestamp {timestamp:.3})");
    // Upstream's `get_float('PULLUP', None, above=0.)` (`query_adc.py:28`):
    // absent is not an error, present is bounded.
    let pullup = match gcmd.get_command_parameters().get("PULLUP") {
        None => None,
        Some(_) => Some(gcmd.get_float_bounded("PULLUP", Some(0.0), None)?),
    };
    if let Some(pullup) = pullup {
        let v = value.clamp(MIN_ADC, MAX_ADC);
        let resistance = pullup * v / (1.0 - v);
        msg.push_str(&format!(
            "\n resistance {resistance:.3} (with {pullup:.0} pullup)"
        ));
    }
    gcmd.respond_info(&msg);
    Ok(())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::pins::{AdcCallback, PrinterPins, PINS_OBJECT};
    use crate::core::klippy::reactor::{ManualReactor, Reactor};

    /// An analog input that reports whatever a test tells it to.
    ///
    /// The real `McuAdc` only produces a last value from firmware reports; the
    /// command under test reads nothing else off the input, so a stand-in keeps
    /// the test at the command's own semantics.
    struct FakeAdc {
        last_value: Mutex<Option<(f64, f64)>>,
    }

    impl FakeAdc {
        fn new(last_value: Option<(f64, f64)>) -> Arc<FakeAdc> {
            Arc::new(FakeAdc {
                last_value: Mutex::new(last_value),
            })
        }
    }

    impl Adc for FakeAdc {
        fn setup_adc_sample(&self, _: f64, _: f64, _: u32, _: u32, _: f64, _: f64, _: u32) {}

        fn setup_adc_callback(&self, _: AdcCallback) {}

        fn get_last_value(&self) -> Option<(f64, f64)> {
            *self.last_value.lock().unwrap_or_else(|p| p.into_inner())
        }
    }

    /// A machine with `gcode`, and the lines every `respond_info` emitted.
    fn world() -> (Arc<Printer>, Arc<Mutex<Vec<String>>>, Arc<ManualReactor>) {
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
        let gcode = Arc::new(GCodeDispatch::new(Arc::clone(&printer)));
        let output = Arc::new(Mutex::new(Vec::new()));
        {
            let output = Arc::clone(&output);
            gcode.register_output_handler(Arc::new(move |line: &str| {
                output
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push(line.to_string());
            }));
        }
        printer
            .add_object(GCODE_OBJECT, Arc::clone(&gcode) as Arc<dyn PrinterObject>)
            .expect("gcode registers once");
        // The pins registry is only here for the ADC-button tests in
        // `buttons.rs`; the command itself needs nothing from it.
        let pins = Arc::new(PrinterPins::new());
        printer
            .add_object(PINS_OBJECT, Arc::clone(&pins) as Arc<dyn PrinterObject>)
            .expect("the pins registry registers once");
        // The dispatcher refuses scripts before the printer is ready; the
        // ready line it emits on the way is not part of what a test asserts.
        printer.send_event(&KlippyEvent::KlippyReady);
        output.lock().unwrap_or_else(|p| p.into_inner()).clear();
        (printer, output, reactor)
    }

    fn run(printer: &Arc<Printer>, script: &str) -> Result<(), String> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered")
            .run_script_sync(script)
            .map_err(|err| err.to_string())
    }

    fn emitted(output: &Arc<Mutex<Vec<String>>>) -> Vec<String> {
        output.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }

    /// ① `QUERY_ADC NAME=` reports the last sample with upstream's two
    /// numbers: the value to six places, the timestamp to three
    /// (`query_adc.py:26-27`).
    #[test]
    fn test_query_adc_reports_the_last_value_and_timestamp() {
        let (printer, output, _) = world();
        let registry = QueryAdc::ensure(&printer).expect("the object registers");
        registry.register_adc("sensor", FakeAdc::new(Some((12.3456789, 0.5))));

        run(&printer, "QUERY_ADC NAME=sensor").expect("the command runs");
        assert_eq!(
            emitted(&output),
            vec!["// ADC object \"sensor\" has value 0.500000 (timestamp 12.346)"]
        );
    }

    /// ① `PULLUP=` adds the resistance line, with the sample clamped away from
    /// the rails first (`query_adc.py:28-32`).
    #[test]
    fn test_query_adc_reports_the_pullup_resistance() {
        let (printer, output, _) = world();
        let registry = QueryAdc::ensure(&printer).expect("the object registers");
        // 1/4 of the rail: r = pullup * v / (1 - v) = 4700 * (1/3).
        registry.register_adc("sensor", FakeAdc::new(Some((1.0, 0.25))));

        run(&printer, "QUERY_ADC NAME=sensor PULLUP=4700").expect("the command runs");
        assert_eq!(
            emitted(&output),
            vec![
                "// ADC object \"sensor\" has value 0.250000 (timestamp 1.000)\n\
                 // resistance 1566.667 (with 4700 pullup)"
            ]
        );

        // A sample at the rail is clamped to 0.99999, so the division stays
        // finite; upstream's `min(.99999, …)` does the same.
        output.lock().unwrap_or_else(|p| p.into_inner()).clear();
        let registry = QueryAdc::ensure(&printer).expect("the object is reused");
        registry.register_adc("rail", FakeAdc::new(Some((2.0, 1.0))));
        run(&printer, "QUERY_ADC NAME=rail PULLUP=4700").expect("the command runs");
        assert_eq!(
            emitted(&output),
            vec![
                "// ADC object \"rail\" has value 1.000000 (timestamp 2.000)\n\
                 // resistance 469995300.002 (with 4700 pullup)"
            ]
        );
    }

    /// ① ③ A missing `NAME` and an unknown one both report the available
    /// names, sorted and quoted, exactly as upstream's fall-through does
    /// (`query_adc.py:20-24`).
    #[test]
    fn test_query_adc_lists_the_available_objects() {
        let (printer, output, _) = world();
        let registry = QueryAdc::ensure(&printer).expect("the object registers");
        registry.register_adc("b_sensor", FakeAdc::new(Some((0.0, 0.5))));
        registry.register_adc("a_sensor", FakeAdc::new(Some((0.0, 0.5))));

        run(&printer, "QUERY_ADC").expect("the command runs");
        run(&printer, "QUERY_ADC NAME=nope").expect("the command runs");
        assert_eq!(
            emitted(&output),
            vec![
                "// Available ADC objects: \"a_sensor\", \"b_sensor\"",
                "// Available ADC objects: \"a_sensor\", \"b_sensor\"",
            ]
        );
    }

    /// ③ `register_adc` is upstream's dictionary assignment: the last
    /// registration of a name is what a query reads.
    #[test]
    fn test_register_adc_keeps_the_last_input_of_a_name() {
        let (printer, output, _) = world();
        let registry = QueryAdc::ensure(&printer).expect("the object registers");
        registry.register_adc("sensor", FakeAdc::new(Some((1.0, 0.25))));
        registry.register_adc("sensor", FakeAdc::new(Some((2.0, 0.75))));
        assert_eq!(registry.names(), vec!["sensor".to_string()]);

        run(&printer, "QUERY_ADC NAME=sensor").expect("the command runs");
        assert_eq!(
            emitted(&output),
            vec!["// ADC object \"sensor\" has value 0.750000 (timestamp 2.000)"]
        );
    }

    /// An input that has not reported yet is reported as such rather than as a
    /// zero (this host's `get_last_value` is `None` until then).
    #[test]
    fn test_query_adc_says_so_when_there_is_no_sample_yet() {
        let (printer, output, _) = world();
        let registry = QueryAdc::ensure(&printer).expect("the object registers");
        registry.register_adc("sensor", FakeAdc::new(None));

        run(&printer, "QUERY_ADC NAME=sensor").expect("the command runs");
        assert_eq!(
            emitted(&output),
            vec!["// ADC object \"sensor\" has not reported a value yet"]
        );
    }

    /// `PULLUP` keeps upstream's `above=0.` bound.
    #[test]
    fn test_query_adc_refuses_a_non_positive_pullup() {
        let (printer, _output, _) = world();
        let registry = QueryAdc::ensure(&printer).expect("the object registers");
        registry.register_adc("sensor", FakeAdc::new(Some((0.0, 0.5))));

        let err = run(&printer, "QUERY_ADC NAME=sensor PULLUP=0").unwrap_err();
        assert!(err.contains("PULLUP must be above 0"), "{err}");
    }

    /// The registration is a module object: `ensure` twice is one object, and
    /// `QUERY_ADC` is in the command table with upstream's help text.
    #[test]
    fn test_ensure_returns_one_shared_object_with_the_command() {
        let (printer, _output, _) = world();
        let first = QueryAdc::ensure(&printer).expect("the object registers");
        let second = QueryAdc::ensure(&printer).expect("the object is reused");
        assert!(Arc::ptr_eq(&first, &second));
        assert!(printer
            .lookup_object_as::<QueryAdc>(QUERY_ADC_OBJECT)
            .is_some());
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        assert_eq!(
            gcode.command_help().get("QUERY_ADC"),
            Some(&QUERY_ADC_HELP.to_string())
        );
    }
}
