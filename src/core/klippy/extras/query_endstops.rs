//! `query_endstops` — the current level of every registered endstop.
//!
//! Upstream's `klippy/extras/query_endstops.py`: an object that collects the
//! `(endstop, name)` pairs the rails register, answers `M119`/`QUERY_ENDSTOPS`,
//! and serves the `query_endstops/status` endpoint. The endpoint returns
//! `"open"`/`"TRIGGERED"` per name; `get_status` reports the last query.
//!
//! The object is created by `[printer]` (the late section that also owns the
//! toolhead), because that is the first point in the load where the rails it
//! needs have all been built. Upstream loads it per rail via
//! `printer.load_object`; here the toolhead registers each rail's endstop with
//! it.
//!
//! # Waiting for the query
//!
//! `query_endstop` is async (`Mcu::call_msg`), but `M119` and the endpoint
//! handler are synchronous. They use the same `block_in_place` bridge the bus
//! debug commands use; the machine runtime is multi-threaded, so one worker
//! blocking on the exchange is fine.

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::ConfigError;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::mcu::{McuEndstop, McuError};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The name the toolhead registers this object under.
pub const QUERY_ENDSTOPS_OBJECT: &str = "query_endstops";

/// The object that answers `M119` and `query_endstops/status`.
pub struct QueryEndstops {
    endstops: Mutex<Vec<(Arc<McuEndstop>, String)>>,
    last_state: Mutex<Vec<(String, bool)>>,
}

impl QueryEndstops {
    /// Build it and register `M119` / `QUERY_ENDSTOPS`.
    ///
    /// # Errors
    /// Returns a config error if the command cannot be registered.
    pub fn new(printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let printer_weak = Arc::downgrade(printer);
        let handler: CommandHandler = Arc::new(move |gcmd: &GcodeCommand| {
            let printer = printer_weak.clone();
            Box::pin(async move { cmd_query_endstops(&printer, gcmd).await })
        });
        for name in ["QUERY_ENDSTOPS", "M119"] {
            gcode
                .register_command(
                    name,
                    Arc::clone(&handler),
                    Some("Report on the status of each endstop"),
                    false,
                )
                .map_err(ConfigError::new)?;
        }
        Ok(Self {
            endstops: Mutex::new(Vec::new()),
            last_state: Mutex::new(Vec::new()),
        })
    }

    /// Register one endstop under `name` (`GenericPrinterRail`,
    /// `klippy/stepper.py:423`).
    pub fn register_endstop(&self, endstop: Arc<McuEndstop>, name: &str) {
        self.endstops
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push((endstop, name.to_string()));
    }

    /// The last query's results, as `(name, triggered)`.
    pub fn last_query(&self) -> Vec<(String, bool)> {
        self.last_state
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }

    /// Query every endstop and remember the result.
    ///
    /// # Errors
    /// Returns the first [`McuError`] a query reports.
    pub async fn query_all(&self, print_time: f64) -> Result<Vec<(String, bool)>, McuError> {
        let endstops: Vec<(Arc<McuEndstop>, String)> = self
            .endstops
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        let mut state = Vec::with_capacity(endstops.len());
        for (endstop, name) in endstops {
            state.push((name, endstop.query_endstop(print_time).await?));
        }
        *self.last_state.lock().unwrap_or_else(|p| p.into_inner()) = state.clone();
        Ok(state)
    }

    /// [`QueryEndstops::query_all`] from a synchronous caller.
    ///
    /// **Temporary**: the `klippy-api` `Endpoint` trait is still synchronous, so
    /// `query_endstops/status` cannot `.await` this yet. Remove it once that
    /// trait is async.
    ///
    /// # Errors
    /// Returns a [`CommandError`] when a query fails or the runtime cannot block.
    pub fn query_all_blocking(&self, print_time: f64) -> Result<Vec<(String, bool)>, CommandError> {
        let handle = tokio::runtime::Handle::try_current()
            .map_err(|_| CommandError::new("endstop queries need the async runtime"))?;
        if handle.runtime_flavor() != tokio::runtime::RuntimeFlavor::MultiThread {
            return Err(CommandError::new(
                "endstop queries need the multi-threaded runtime",
            ));
        }
        tokio::task::block_in_place(|| handle.block_on(self.query_all(print_time)))
            .map_err(|err| CommandError::new(err.to_string()))
    }
}

impl PrinterObject for QueryEndstops {
    fn get_status(&self, _eventtime: f64) -> Value {
        let map: serde_json::Map<String, Value> = self
            .last_query()
            .into_iter()
            .map(|(name, triggered)| (name, Value::Bool(triggered)))
            .collect();
        json!({ "last_query": map })
    }
}

impl std::fmt::Debug for QueryEndstops {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("QueryEndstops").finish_non_exhaustive()
    }
}

/// The print time a query is dated from: the toolhead's if it is up, else `0`.
pub(crate) fn query_print_time(printer: &Printer) -> f64 {
    printer
        .lookup_object_as::<ToolHeadObject>("toolhead")
        .map(|toolhead| toolhead.print_time())
        .unwrap_or(0.0)
}

/// `M119` / `QUERY_ENDSTOPS`: report every endstop's current level.
async fn cmd_query_endstops(
    printer: &Weak<Printer>,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let printer = printer
        .upgrade()
        .ok_or_else(|| CommandError::new("Printer is not available"))?;
    let query = printer
        .lookup_object_as::<QueryEndstops>(QUERY_ENDSTOPS_OBJECT)
        .ok_or_else(|| CommandError::new("query_endstops is not available"))?;
    let state = query
        .query_all(query_print_time(&printer))
        .await
        .map_err(|err| CommandError::new(err.to_string()))?;
    let msg = state
        .iter()
        .map(|(name, triggered)| {
            format!("{name}:{}", if *triggered { "TRIGGERED" } else { "open" })
        })
        .collect::<Vec<_>>()
        .join(" ");
    gcmd.respond_raw(&msg);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::core::klippy::cmd::clock::McuClock;
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::devices::frame_mock::{FrameMock, MappingEntry};
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::{ConfigBuilder, Dictionary, Mcu, McuChip};
    use crate::core::klippy::msg::proto::{ArgValue, Payload};
    use crate::core::klippy::pins::{PinParams, PrinterPins};
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::json;

    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "endstop_query_state oid=%c": 42,
                "config_endstop oid=%c pin=%c pull_up=%c": 40,
                "config_trsync oid=%c": 30
            },
            "responses": {
                "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c": 43
            },
            "enumerations": {"pin": {"PA1": 1}},
            "config": {"CLOCK_FREQ": 1_000_000}
        }))
        .unwrap()
    }

    fn payload(values: &[ArgValue]) -> Vec<u8> {
        let mut out = Payload::new();
        for value in values {
            out.push_value(value).unwrap();
        }
        out.into_raw()
    }

    fn frame(parts: &[ArgValue]) -> Frame {
        Frame::new(0, payload(parts))
    }

    /// A chip over a test MCU, with the endstop query scripted to answer
    /// `pin_value`.
    fn endstop(pin_value: u8) -> Arc<McuEndstop> {
        let device = FrameMock::new(vec![MappingEntry {
            input: frame(&[ArgValue::UInt8(42), ArgValue::UInt8(0)]),
            outputs: vec![frame(&[
                ArgValue::UInt8(43),
                ArgValue::UInt8(0),
                ArgValue::UInt8(0),
                ArgValue::UInt32(0),
                ArgValue::UInt8(pin_value),
            ])],
        }]);
        let mcu = Arc::new(Mcu::for_test("mcu", Interface::new(device)));
        mcu.install_dictionary(dictionary()).unwrap();
        let chip = McuChip::new(
            "mcu".to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::new(PrinterPins::new()),
        );
        let clock = Arc::new(McuClock::new(Arc::clone(&mcu), ManualReactor::shared()));
        clock.seed(0.0, 0);
        chip.set_clock(clock, 0.0);
        chip.attach(mcu);
        Arc::new(
            McuEndstop::new(
                chip,
                &PinParams {
                    chip_name: "mcu".to_string(),
                    pin: "PA1".to_string(),
                    invert: false,
                    pullup: 0,
                    share_type: None,
                },
            )
            .unwrap(),
        )
    }

    fn query(endstop: Arc<McuEndstop>) -> QueryEndstops {
        QueryEndstops {
            endstops: Mutex::new(vec![(endstop, "x".to_string())]),
            last_state: Mutex::new(Vec::new()),
        }
    }

    #[tokio::test]
    async fn test_query_all_reads_the_endstop_and_remembers_it() {
        let query = query(endstop(1));

        let state = query.query_all(0.0).await.unwrap();

        assert_eq!(state, vec![("x".to_string(), true)]);
        assert_eq!(query.last_query(), vec![("x".to_string(), true)]);
        assert_eq!(
            query.get_status(0.0)["last_query"]["x"],
            serde_json::json!(true)
        );
    }

    #[tokio::test]
    async fn test_an_inverting_endstop_flips_the_level() {
        let mut params = PinParams {
            chip_name: "mcu".to_string(),
            pin: "PA1".to_string(),
            invert: true,
            pullup: 0,
            share_type: None,
        };
        // Rebuild the endstop with `!` on the pin; the fake still reports 1.
        let device = FrameMock::new(vec![MappingEntry {
            input: frame(&[ArgValue::UInt8(42), ArgValue::UInt8(0)]),
            outputs: vec![frame(&[
                ArgValue::UInt8(43),
                ArgValue::UInt8(0),
                ArgValue::UInt8(0),
                ArgValue::UInt32(0),
                ArgValue::UInt8(1),
            ])],
        }]);
        let mcu = Arc::new(Mcu::for_test("mcu", Interface::new(device)));
        mcu.install_dictionary(dictionary()).unwrap();
        let chip = McuChip::new(
            "mcu".to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::new(PrinterPins::new()),
        );
        let clock = Arc::new(McuClock::new(Arc::clone(&mcu), ManualReactor::shared()));
        clock.seed(0.0, 0);
        chip.set_clock(clock, 0.0);
        chip.attach(mcu);
        params.invert = true;
        let endstop = Arc::new(McuEndstop::new(chip, &params).unwrap());

        let state = query(endstop).query_all(0.0).await.unwrap();

        assert_eq!(state, vec![("x".to_string(), false)]);
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn test_m119_reports_each_endstop() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let query = QueryEndstops::new(&printer).unwrap();
        printer
            .add_object(QUERY_ENDSTOPS_OBJECT, Arc::new(query))
            .unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap();
        let lines = Arc::new(Mutex::new(Vec::new()));
        {
            let lines = Arc::clone(&lines);
            gcode.register_output_handler(Arc::new(move |line: &str| {
                lines
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push(line.to_string())
            }));
        }

        gcode.run_script("M119").await.unwrap();

        // No endstops registered: the report is the empty line.
        assert_eq!(*lines.lock().unwrap(), vec![String::new()]);
    }
}
