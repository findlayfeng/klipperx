//! `gcode/help`, `gcode/script`, `gcode/restart`, `gcode/firmware_restart`.
//!
//! The API's door into the G-Code dispatcher (`core/klippy/gcode.rs`), upstream
//! `GCodeHelper` (`klippy/webhooks.py:429-452`):
//!
//! | Endpoint | Answer |
//! |---|---|
//! | `gcode/help` | the flat `{command: help}` table |
//! | `gcode/script` | `{}`, or an `error` reply with the command's message |
//! | `gcode/restart` | `{}`; runs the `RESTART` command |
//! | `gcode/firmware_restart` | `{}`; runs the `FIRMWARE_RESTART` command |
//!
//! # Why the dispatcher is looked up per request
//!
//! The endpoints are installed by `api::register`, which the host runs *before*
//! `load_config` creates the `gcode` object (upstream registers the object in
//! `Printer.__init__`, before its socket). Rather than change that order, each
//! handler resolves `gcode` from the printer when it is called — the same
//! `lookup_object` an upstream module would do. Before the config is loaded the
//! lookup fails and the request gets the printer's state message as a command
//! error, which is what a script sent that early deserves.
//!
//! # Not here
//!
//! * **`gcode/subscribe_output`**: the push side of G-Code output. It needs an
//!   output handler registered per connection and removed when that connection
//!   goes away, which the output-handler table does not support yet.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::api::protocol::{ApiError, Request};
use crate::core::klippy::api::registry::{Endpoint, EndpointContext};
use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
use crate::core::klippy::printer::Printer;

/// Resolve the dispatcher, reporting the printer state if it is not up yet.
fn gcode(printer: &Printer) -> Result<Arc<GCodeDispatch>, ApiError> {
    printer
        .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
        .ok_or_else(|| ApiError::CommandError(printer.get_state_message().message))
}

/// `gcode/help` — the registered commands and their help text.
pub struct GcodeHelp {
    printer: Arc<Printer>,
}

impl GcodeHelp {
    /// Build the endpoint over the machine whose dispatcher it reads.
    pub fn new(printer: Arc<Printer>) -> Self {
        Self { printer }
    }
}

impl Endpoint for GcodeHelp {
    fn path(&self) -> &'static str {
        "gcode/help"
    }

    fn handle(
        &self,
        _request: &Request,
        _context: &EndpointContext<'_>,
    ) -> Result<Value, ApiError> {
        let gcode = gcode(&self.printer)?;
        Ok(json!(gcode.command_help()))
    }
}

/// `gcode/script` — run a script and report the first command error.
pub struct GcodeScript {
    printer: Arc<Printer>,
}

impl GcodeScript {
    /// Build the endpoint over the machine whose dispatcher it drives.
    pub fn new(printer: Arc<Printer>) -> Self {
        Self { printer }
    }
}

impl Endpoint for GcodeScript {
    fn path(&self) -> &'static str {
        "gcode/script"
    }

    fn handle(&self, request: &Request, _context: &EndpointContext<'_>) -> Result<Value, ApiError> {
        let script = request.params().get_str("script")?;
        let gcode = gcode(&self.printer)?;
        gcode
            .run_script(script)
            .map_err(|err| ApiError::CommandError(err.to_string()))?;
        Ok(json!({}))
    }
}

/// `gcode/restart` and `gcode/firmware_restart` — run the restart command.
///
/// The command itself decides what a restart means (`request_exit`); the restart
/// loop that acts on the result is TODO D2, so today the host exits.
pub struct GcodeRestart {
    printer: Arc<Printer>,
    path: &'static str,
    script: &'static str,
}

impl GcodeRestart {
    /// `gcode/restart`.
    pub fn restart(printer: Arc<Printer>) -> Self {
        Self {
            printer,
            path: "gcode/restart",
            script: "restart",
        }
    }

    /// `gcode/firmware_restart`.
    pub fn firmware_restart(printer: Arc<Printer>) -> Self {
        Self {
            printer,
            path: "gcode/firmware_restart",
            script: "firmware_restart",
        }
    }
}

impl Endpoint for GcodeRestart {
    fn path(&self) -> &'static str {
        self.path
    }

    fn handle(
        &self,
        _request: &Request,
        _context: &EndpointContext<'_>,
    ) -> Result<Value, ApiError> {
        let gcode = gcode(&self.printer)?;
        gcode
            .run_script(self.script)
            .map_err(|err| ApiError::CommandError(err.to_string()))?;
        Ok(json!({}))
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::registry::Api;
    use crate::core::klippy::api::test_support::{context, silent_target};
    use crate::core::klippy::gcode::{CommandError, CommandHandler};
    use crate::core::klippy::reactor::ManualReactor;

    fn request(body: &str) -> Request {
        Request::parse(body.as_bytes()).expect("test body is a valid request")
    }

    /// A printer in the ready state with a `gcode` object.
    fn printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        printer.send_event(&crate::core::klippy::printer::PrinterEvent::Ready);
        printer
    }

    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap()
    }

    #[test]
    fn test_the_paths_are_the_documented_ones() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        assert_eq!(GcodeHelp::new(Arc::clone(&printer)).path(), "gcode/help");
        assert_eq!(
            GcodeScript::new(Arc::clone(&printer)).path(),
            "gcode/script"
        );
        assert_eq!(
            GcodeRestart::restart(Arc::clone(&printer)).path(),
            "gcode/restart"
        );
        assert_eq!(
            GcodeRestart::firmware_restart(printer).path(),
            "gcode/firmware_restart"
        );
    }

    #[test]
    fn test_help_returns_the_flat_command_table() {
        let printer = printer();
        let handler: CommandHandler = Arc::new(|_| Ok(()));
        gcode(&printer)
            .register_command("SET_PIN", handler, Some("Set a pin"), false)
            .unwrap();
        let api = Api::new();

        let response = GcodeHelp::new(Arc::clone(&printer))
            .handle(
                &request(r#"{"method":"gcode/help"}"#),
                &context(&api, silent_target()),
            )
            .unwrap();

        assert_eq!(response["SET_PIN"], "Set a pin");
    }

    #[test]
    fn test_script_runs_and_answers_empty() {
        let printer = printer();
        let api = Api::new();

        let response = GcodeScript::new(Arc::clone(&printer))
            .handle(
                &request(r#"{"method":"gcode/script","params":{"script":"M115"}}"#),
                &context(&api, silent_target()),
            )
            .unwrap();

        assert_eq!(response, json!({}));
    }

    #[test]
    fn test_a_command_error_becomes_an_error_reply() {
        let printer = printer();
        gcode(&printer)
            .register_command(
                "FAIL",
                Arc::new(|_| Err(CommandError::new("boom"))),
                None,
                false,
            )
            .unwrap();
        let api = Api::new();

        let err = GcodeScript::new(Arc::clone(&printer))
            .handle(
                &request(r#"{"method":"gcode/script","params":{"script":"FAIL"}}"#),
                &context(&api, silent_target()),
            )
            .unwrap_err();

        assert_eq!(err, ApiError::CommandError("boom".to_string()));
        assert!(!err.is_internal(), "a command error must not stop klippy");
    }

    #[test]
    fn test_script_is_required() {
        let printer = printer();
        let api = Api::new();

        let err = GcodeScript::new(printer)
            .handle(
                &request(r#"{"method":"gcode/script"}"#),
                &context(&api, silent_target()),
            )
            .unwrap_err();

        assert_eq!(err, ApiError::MissingArgument("script".to_string()));
    }

    #[test]
    fn test_restart_runs_the_restart_command() {
        let printer = printer();
        let api = Api::new();

        GcodeRestart::firmware_restart(Arc::clone(&printer))
            .handle(
                &request(r#"{"method":"gcode/firmware_restart"}"#),
                &context(&api, silent_target()),
            )
            .unwrap();

        // The built-in FIRMWARE_RESTART asks the printer to exit with that
        // result; `run` returns it without waiting because it is already set.
        assert_eq!(printer.run(), "firmware_restart");
    }

    #[test]
    fn test_a_request_before_the_dispatcher_exists_reports_the_state() {
        // No `gcode` object, as before the config is loaded.
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let api = Api::new();

        let err = GcodeScript::new(printer)
            .handle(
                &request(r#"{"method":"gcode/script","params":{"script":"M115"}}"#),
                &context(&api, silent_target()),
            )
            .unwrap_err();

        assert!(matches!(err, ApiError::CommandError(_)), "{err:?}");
        assert!(err.to_string().contains("Starting up"), "{err}");
    }
}
