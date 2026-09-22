//! More verbose information on micro-controller errors.
//!
//! A direct port of `klippy/extras/error_mcu.py`: it listens for
//! `klippy:analyze_shutdown` and `klippy:notify_mcu_error` and replaces the
//! terse state message with one that names the MCU, gives the failure a hint
//! and tells the user what to do next.
//!
//! The module is loaded by the first `[mcu]` section (`mcu/object.rs`), as
//! upstream loads it from `MCU.__init__` (`klippy/mcu.py:1159`). It has no
//! status of its own, so it never shows up in `objects/list`.
//!
//! `adc_temperature` can add per-failure clarifications through
//! [`PrinterMcuError::add_clarify`]; the map is kept ready for it.

use std::collections::HashMap;
use std::sync::{Arc, Weak};

use serde_json::Value;
use tracing::warn;

use crate::core::klippy::error::ConfigError;
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The name this object is registered under.
pub const ERROR_MCU_OBJECT: &str = "error_mcu";

const MESSAGE_SHUTDOWN: &str = "\nOnce the underlying issue is corrected, use the\n\"FIRMWARE_RESTART\" command to reset the firmware, reload the\nconfig, and restart the host software.\nPrinter is shutdown\n";

const MESSAGE_PROTOCOL_ERROR1: &str = "\nThis is frequently caused by running an older version of the\nfirmware on the MCU(s). Fix by recompiling and flashing the\nfirmware.\n";

const MESSAGE_PROTOCOL_ERROR2: &str = "\nOnce the underlying issue is corrected, use the \"RESTART\"\ncommand to reload the config and restart the host software.\n";

const MESSAGE_MCU_CONNECT_ERROR: &str = "\nOnce the underlying issue is corrected, use the\n\"FIRMWARE_RESTART\" command to reset the firmware, reload the\nconfig, and restart the host software.\nError configuring printer\n";

/// The firmware messages that have a generic hint, and the hint for each.
///
/// The order matters the way it does upstream (`error_mcu.py:26-43`): the
/// first prefix that matches wins.
const COMMON_MCU_ERRORS: &[(&[&str], &str)] = &[
    (
        &["Timer too close"],
        "\nThis often indicates the host computer is overloaded. Check\nfor other processes consuming excessive CPU time, high swap\nusage, disk errors, overheating, unstable voltage, or\nsimilar system problems on the host computer.",
    ),
    (
        &["Missed scheduling of next "],
        "\nThis is generally indicative of an intermittent\ncommunication failure between micro-controller and host.",
    ),
    (
        &["ADC out of range"],
        "\nThis generally occurs when a heater temperature exceeds\nits configured min_temp or max_temp.",
    ),
    (
        &["Rescheduled timer in the past", "Stepper too far in past"],
        "\nThis generally occurs when the micro-controller has been\nrequested to step at a rate higher than it is capable of\nobtaining.",
    ),
    (
        &["Command request"],
        "\nThis generally occurs in response to an M112 G-Code command\nor in response to an internal error in the host software.",
    ),
];

/// The generic hint for a firmware error string, or `""`.
fn error_hint(msg: &str) -> &'static str {
    for (prefixes, help) in COMMON_MCU_ERRORS {
        for prefix in *prefixes {
            if msg.starts_with(prefix) {
                return help;
            }
        }
    }
    ""
}

/// A clarification callback for one firmware message.
type Clarify = Box<dyn Fn(&str, &HashMap<String, Value>) -> Option<String> + Send + Sync>;

/// The `error_mcu` module.
pub struct PrinterMcuError {
    /// Per-message clarification callbacks (`add_clarify`).
    clarify: HashMap<String, Vec<Clarify>>,
}

impl PrinterMcuError {
    /// Build the module and register its two event handlers.
    pub fn new(printer: &Arc<Printer>) -> Self {
        let weak = Arc::downgrade(printer);
        {
            let weak = weak.clone();
            printer.register_event_handler(
                KlippyEvent::KlippyAnalyzeShutdown {
                    msg: String::new(),
                    details: HashMap::new(),
                },
                Box::new(move |event| {
                    if let KlippyEvent::KlippyAnalyzeShutdown { msg, details } = event {
                        handle_analyze_shutdown(&weak, msg, details);
                    }
                }),
            );
        }
        {
            let weak = weak.clone();
            printer.register_event_handler(
                KlippyEvent::KlippyNotifyMcuError {
                    msg: String::new(),
                    details: HashMap::new(),
                },
                Box::new(move |event| {
                    if let KlippyEvent::KlippyNotifyMcuError { msg, details } = event {
                        handle_notify_mcu_error(&weak, msg, details);
                    }
                }),
            );
        }
        Self {
            clarify: HashMap::new(),
        }
    }

    /// Add a clarification for one firmware message (`adc_temperature` uses
    /// this to explain which sensor is out of range).
    pub fn add_clarify(
        &mut self,
        msg: impl Into<String>,
        callback: impl Fn(&str, &HashMap<String, Value>) -> Option<String> + Send + Sync + 'static,
    ) {
        self.clarify
            .entry(msg.into())
            .or_default()
            .push(Box::new(callback));
    }
}

impl PrinterObject for PrinterMcuError {
    fn get_status(&self, _eventtime: f64) -> Value {
        // Upstream has no `get_status`; `is_queryable` keeps it out of
        // `objects/list` either way.
        Value::Object(Default::default())
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

/// The single `error_mcu` object, created by the first `[mcu]` section.
///
/// Idempotent: the second `[mcu]` section finds the object already registered,
/// as upstream's `printer.load_object` caches by name (`klippy/mcu.py:1159`).
pub fn ensure(printer: &Arc<Printer>) -> Result<Arc<PrinterMcuError>, ConfigError> {
    if let Some(existing) = printer.lookup_object_as::<PrinterMcuError>(ERROR_MCU_OBJECT) {
        return Ok(existing);
    }
    let object = Arc::new(PrinterMcuError::new(printer));
    printer.add_object(ERROR_MCU_OBJECT, object.clone())?;
    Ok(object)
}

/// `_handle_analyze_shutdown`: enrich the shutdown message.
fn handle_analyze_shutdown(printer: &Weak<Printer>, msg: &str, details: &HashMap<String, Value>) {
    let Some(printer) = printer.upgrade() else {
        return;
    };
    if msg == "MCU shutdown" {
        let name = details
            .get("mcu")
            .and_then(Value::as_str)
            .unwrap_or("?")
            .to_string();
        let reason = details
            .get("reason")
            .and_then(Value::as_str)
            .unwrap_or("")
            .to_string();
        let event_type = details
            .get("event_type")
            .and_then(Value::as_str)
            .unwrap_or("shutdown");
        let prefix = if event_type == "is_shutdown" {
            format!("Previous MCU '{name}' shutdown: ")
        } else {
            format!("MCU '{name}' shutdown: ")
        };
        let hint = error_hint(&reason);
        let newmsg = format!("{prefix}{reason}{hint}{MESSAGE_SHUTDOWN}");
        printer.update_error_msg(msg, &newmsg);
    } else {
        printer.update_error_msg(msg, &format!("{msg}{MESSAGE_SHUTDOWN}"));
    }
}

/// `_handle_notify_mcu_error`: enrich a connect-time or protocol failure.
fn handle_notify_mcu_error(printer: &Weak<Printer>, msg: &str, details: &HashMap<String, Value>) {
    let Some(printer) = printer.upgrade() else {
        return;
    };
    let error = details
        .get("error")
        .and_then(Value::as_str)
        .unwrap_or("")
        .to_string();
    match msg {
        "Protocol error" => handle_protocol_error(&printer, msg, &error),
        "MCU error during connect" => {
            printer.update_error_msg(msg, &format!("{error}{MESSAGE_MCU_CONNECT_ERROR}"));
        }
        _ => {}
    }
}

/// `_check_protocol_error`: list the MCUs whose firmware version differs from
/// the host's.
fn handle_protocol_error(printer: &Arc<Printer>, msg: &str, error: &str) {
    let host_version = printer.software_version();
    let mut to_update = Vec::new();
    let mut updated = Vec::new();
    for (name, object) in printer.lookup_objects(Some("mcu")) {
        let status = object.get_status(0.0);
        let Some(mcu_version) = status.get("mcu_version").and_then(Value::as_str) else {
            warn!("Unable to retrieve mcu_version from {name}");
            continue;
        };
        let short = name.split_whitespace().last().unwrap_or(&name);
        let line = format!("{short}: Current version {mcu_version}");
        if mcu_version != host_version {
            to_update.push(line);
        } else {
            updated.push(line);
        }
    }
    if to_update.is_empty() {
        to_update.push("<none>".to_string());
    }
    if updated.is_empty() {
        updated.push("<none>".to_string());
    }

    let mut lines = vec![
        "MCU Protocol error".to_string(),
        MESSAGE_PROTOCOL_ERROR1.to_string(),
        format!("Your Klipper version is: {host_version}"),
        "MCU(s) which should be updated:".to_string(),
    ];
    lines.extend(to_update);
    lines.push("Up-to-date MCU(s):".to_string());
    lines.extend(updated);
    lines.push(MESSAGE_PROTOCOL_ERROR2.to_string());
    lines.push(error.to_string());
    printer.update_error_msg(msg, &lines.join("\n"));
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::printer::PrinterState;
    use crate::core::klippy::reactor::ManualReactor;

    fn printer() -> Arc<Printer> {
        Arc::new(Printer::new(ManualReactor::shared()))
    }

    /// Just enough of an MCU object for the protocol-error branch.
    struct FakeMcu(String);

    impl PrinterObject for FakeMcu {
        fn get_status(&self, _eventtime: f64) -> Value {
            serde_json::json!({ "mcu_version": self.0 })
        }
    }

    #[test]
    fn test_a_known_firmware_message_gets_its_hint() {
        let shutdown = error_hint("Timer too close");
        assert!(
            shutdown.contains("host computer is overloaded"),
            "{shutdown}"
        );
        assert!(shutdown.starts_with('\n'));
        assert_eq!(error_hint("something else"), "");
        // The two prefixes of the "rescheduled" group share one hint.
        assert_eq!(
            error_hint("Stepper too far in past"),
            error_hint("Rescheduled timer in the past")
        );
    }

    #[test]
    fn test_mcu_shutdown_is_expanded_with_the_reason_and_the_hint() {
        let printer = printer();
        // The handler is registered by the module; the MCU factory creates it.
        let _module = PrinterMcuError::new(&printer);

        printer.invoke_shutdown_with(
            "MCU shutdown",
            HashMap::from([
                ("mcu".to_string(), Value::String("mcu".to_string())),
                (
                    "reason".to_string(),
                    Value::String("Timer too close".to_string()),
                ),
                (
                    "event_type".to_string(),
                    Value::String("shutdown".to_string()),
                ),
            ]),
        );

        let status = printer.get_state_message();
        assert!(status
            .message
            .starts_with("MCU 'mcu' shutdown: Timer too close"));
        assert!(status.message.contains("host computer is overloaded"));
        assert!(status.message.contains("Printer is shutdown"));
        assert_eq!(status.category, PrinterState::Shutdown);
    }

    #[test]
    fn test_an_is_shutdown_event_says_previous() {
        let printer = printer();
        let _module = PrinterMcuError::new(&printer);

        printer.invoke_shutdown_with(
            "MCU shutdown",
            HashMap::from([
                ("mcu".to_string(), Value::String("aux".to_string())),
                ("reason".to_string(), Value::String("Lost".to_string())),
                (
                    "event_type".to_string(),
                    Value::String("is_shutdown".to_string()),
                ),
            ]),
        );

        assert!(printer
            .get_state_message()
            .message
            .starts_with("Previous MCU 'aux' shutdown: Lost"));
    }

    #[test]
    fn test_an_unrelated_shutdown_still_tells_the_user_what_to_type() {
        let printer = printer();
        let _module = PrinterMcuError::new(&printer);

        printer.invoke_shutdown("Lost communication with MCU 'mcu'");

        assert_eq!(
            printer.get_state_message().message,
            format!("Lost communication with MCU 'mcu'{MESSAGE_SHUTDOWN}")
        );
    }

    #[test]
    fn test_a_connect_error_gets_the_firmware_restart_hint() {
        let printer = printer();
        let _module = PrinterMcuError::new(&printer);
        printer.set_error_state("MCU error during connect");

        printer.send_event(&KlippyEvent::KlippyNotifyMcuError {
            msg: "MCU error during connect".to_string(),
            details: HashMap::from([(
                "error".to_string(),
                Value::String("Unable to open serial port".to_string()),
            )]),
        });

        let message = printer.get_state_message().message;
        assert!(message.starts_with("Unable to open serial port"));
        assert!(message.contains("Error configuring printer"));
    }

    #[test]
    fn test_the_protocol_error_lists_the_mcus_that_need_updating() {
        // A printer with two MCU objects, one on the host version and one not.
        let printer = printer();
        let _module = PrinterMcuError::new(&printer);
        printer.set_software_version("v0.13.0");
        for (name, version) in [("mcu", "v0.12.0"), ("mcu aux", "v0.13.0")] {
            printer
                .add_object(name, Arc::new(FakeMcu(version.to_string())))
                .unwrap();
        }
        printer.set_error_state("Protocol error");

        printer.send_event(&KlippyEvent::KlippyNotifyMcuError {
            msg: "Protocol error".to_string(),
            details: HashMap::from([(
                "error".to_string(),
                Value::String("Unexpected message".to_string()),
            )]),
        });

        let message = printer.get_state_message().message;
        assert!(message.starts_with("MCU Protocol error"));
        assert!(message.contains("Your Klipper version is: v0.13.0"));
        assert!(message.contains("mcu: Current version v0.12.0"));
        assert!(message.contains("aux: Current version v0.13.0"));
        assert!(message.contains("MCU(s) which should be updated:"));
        assert!(message.contains("Up-to-date MCU(s):"));
        assert!(message.ends_with("Unexpected message"));
    }
}
