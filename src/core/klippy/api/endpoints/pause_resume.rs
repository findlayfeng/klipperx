//! `pause_resume/pause`, `pause_resume/resume`, `pause_resume/cancel` — drive
//! the print's pause state from a client.
//!
//! Upstream's three `webhooks.register_endpoint` calls in `PauseResume.__init__`
//! (`klippy/extras/pause_resume.py:27-32`): each handler just runs the matching
//! g-code command —
//!
//! | Endpoint | Script |
//! |---|---|
//! | `pause_resume/pause` | `PAUSE` |
//! | `pause_resume/resume` | `RESUME` |
//! | `pause_resume/cancel` | `CANCEL_PRINT` |
//!
//! ```json
//! {"id": 1, "method": "pause_resume/pause"}
//! ```
//!
//! The requests carry no parameters (there is no `silent` in upstream's
//! handlers, and the reference documents none), and each answer is `{}`: the
//! upstream handler never calls `web_request.send`, so `WebRequest.reply`
//! falls back to the default response (`klippy/webhooks.py:110-114`). The
//! `PAUSE`/`RESUME`/`CANCEL_PRINT` command replies (`action:paused`,
//! `action:resumed`, `action:cancel`) travel as output lines, not in the
//! response — the same channel a `gcode/script` request sees them on.
//!
//! Like `gcode/script`, the dispatcher is resolved per request: `register`
//! runs before the config is read, so the `[pause_resume]` object and the
//! `gcode` object do not exist yet when this installs. Before the config the
//! lookup fails and the request gets the printer's state message as a command
//! error. Once the dispatcher exists but no `[pause_resume]` section was read,
//! the command is unregistered and the dispatcher answers
//! `Unknown command:"PAUSE"` as an output line (`gcode.rs`), leaving the `{}`
//! reply intact — upstream would not have installed the endpoint at all, since
//! its `PauseResume.__init__` registers the three paths.
//!
//! # Status
//!
//! Written, tested and registered by [`register`](super::super::register).

use std::sync::Arc;

use serde_json::json;

use crate::core::klippy::api::protocol::{ApiError, Request};
use crate::core::klippy::api::registry::{Endpoint, EndpointContext, EndpointFuture};
use crate::core::klippy::api::{Api, ApiWiring, RegistrationError};
use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
use crate::core::klippy::printer::Printer;

endpoint!(install);

/// Install the three `pause_resume/*` endpoints.
pub(crate) fn install(api: &mut Api, wiring: &ApiWiring<'_>) -> Result<(), RegistrationError> {
    let printer = Arc::clone(wiring.printer);
    api.register(PauseResumeEndpoint::pause(Arc::clone(&printer)))
        .map_err(RegistrationError::Endpoint)?;
    api.register(PauseResumeEndpoint::resume(Arc::clone(&printer)))
        .map_err(RegistrationError::Endpoint)?;
    api.register(PauseResumeEndpoint::cancel(printer))
        .map_err(RegistrationError::Endpoint)
}

/// Resolve the dispatcher, reporting the printer state if it is not up yet
/// (the `gcode/script` rule — module docs).
fn gcode(printer: &Printer) -> Result<Arc<GCodeDispatch>, ApiError> {
    printer
        .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
        .ok_or_else(|| ApiError::CommandError(printer.get_state_message().message))
}

/// One of the three `pause_resume/*` endpoints: a path and the command it runs.
pub struct PauseResumeEndpoint {
    printer: Arc<Printer>,
    path: &'static str,
    script: &'static str,
}

impl PauseResumeEndpoint {
    /// `pause_resume/pause` — run `PAUSE`.
    pub fn pause(printer: Arc<Printer>) -> Self {
        Self {
            printer,
            path: "pause_resume/pause",
            script: "PAUSE",
        }
    }

    /// `pause_resume/resume` — run `RESUME`.
    pub fn resume(printer: Arc<Printer>) -> Self {
        Self {
            printer,
            path: "pause_resume/resume",
            script: "RESUME",
        }
    }

    /// `pause_resume/cancel` — run `CANCEL_PRINT`.
    pub fn cancel(printer: Arc<Printer>) -> Self {
        Self {
            printer,
            path: "pause_resume/cancel",
            script: "CANCEL_PRINT",
        }
    }
}

impl Endpoint for PauseResumeEndpoint {
    fn path(&self) -> &'static str {
        self.path
    }

    fn handle<'a>(
        &'a self,
        _request: &'a Request,
        _context: &'a EndpointContext<'a>,
    ) -> EndpointFuture<'a> {
        Box::pin(async move {
            let gcode = gcode(&self.printer)?;
            gcode
                .run_script(self.script)
                .await
                .map_err(|err| ApiError::CommandError(err.to_string()))?;
            Ok(json!({}))
        })
    }
}

impl std::fmt::Debug for PauseResumeEndpoint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PauseResumeEndpoint")
            .field("path", &self.path)
            .field("script", &self.script)
            .finish_non_exhaustive()
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    use crate::core::klippy::api::test_support::{context, silent_target};
    use crate::core::klippy::config::Config;
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::extras::gcode_move::{self, MoveTarget};
    use crate::core::klippy::extras::pause_resume::{PauseResume, PAUSE_RESUME_OBJECT};
    use crate::core::klippy::gcode::{CommandError, GCodeDispatch, GCODE_OBJECT};
    use crate::core::klippy::mathutil::Coord;
    use crate::core::klippy::reactor::ManualReactor;

    fn request(method: &str) -> Request {
        let body = format!(r#"{{"id":1,"method":"{method}"}}"#);
        Request::parse(body.as_bytes()).expect("a valid request")
    }

    /// A move target that stands still — enough for `RESUME`'s move-back
    /// (`RESTORE_GCODE_STATE … MOVE=1` needs a transform to move through).
    struct FakeTarget;

    impl MoveTarget for FakeTarget {
        fn move_to(&self, _position: Coord, _speed: f64) -> Result<(), CommandError> {
            Ok(())
        }

        fn position(&self) -> Coord {
            Coord::default()
        }
    }

    /// A ready machine with `[pause_resume]` loaded, so all three commands are
    /// reachable (the same shape as the object's own tests).
    fn machine() -> (Arc<Printer>, Arc<PauseResume>) {
        let (config, _) = Config::from_text("[pause_resume]\n").expect("the config parses");
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer.load_config(&config).expect("the config loads");
        printer.send_event(&KlippyEvent::KlippyReady);
        gcode_move::ensure(&printer)
            .expect("gcode_move registers")
            .set_move_transform(Arc::new(FakeTarget), true)
            .expect("the slot is free");
        let object = printer
            .lookup_object_as::<PauseResume>(PAUSE_RESUME_OBJECT)
            .expect("the section registered the object");
        (printer, object)
    }

    /// Everything `gcode` reported through `respond_info`, one entry per line.
    fn captured_lines(printer: &Arc<Printer>) -> Arc<Mutex<Vec<String>>> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        let lines = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&lines);
        gcode.register_output_handler(Arc::new(move |line: &str| {
            sink.lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(line.to_string());
        }));
        lines
    }

    fn emitted(lines: &Arc<Mutex<Vec<String>>>) -> Vec<String> {
        lines.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }

    #[test]
    fn test_the_paths_are_the_documented_ones() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        assert_eq!(
            PauseResumeEndpoint::pause(Arc::clone(&printer)).path(),
            "pause_resume/pause"
        );
        assert_eq!(
            PauseResumeEndpoint::resume(Arc::clone(&printer)).path(),
            "pause_resume/resume"
        );
        assert_eq!(
            PauseResumeEndpoint::cancel(printer).path(),
            "pause_resume/cancel"
        );
    }

    #[tokio::test]
    async fn test_install_registers_the_three_endpoints() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let mut api = Api::new();
        let wiring = ApiWiring {
            printer: &printer,
            start_args: &crate::core::klippy::api::start_args::StartArgs::collect(
                "/tmp/printer.cfg",
                None,
            ),
        };

        install(&mut api, &wiring).unwrap();

        assert_eq!(
            api.endpoints(),
            [
                "list_endpoints",
                "pause_resume/cancel",
                "pause_resume/pause",
                "pause_resume/resume"
            ]
        );
    }

    #[tokio::test]
    async fn test_pause_runs_the_command_and_answers_empty() {
        let (printer, object) = machine();
        let lines = captured_lines(&printer);
        let api = Api::new();
        let endpoint = PauseResumeEndpoint::pause(Arc::clone(&printer));

        let response = endpoint
            .handle(
                &request("pause_resume/pause"),
                &context(&api, silent_target()),
            )
            .await
            .unwrap();

        assert_eq!(response, json!({}));
        assert!(object.is_paused());
        assert_eq!(emitted(&lines), ["// action:paused"]);
    }

    #[tokio::test]
    async fn test_resume_runs_the_command_and_answers_empty() {
        let (printer, object) = machine();
        let lines = captured_lines(&printer);
        let api = Api::new();
        PauseResumeEndpoint::pause(Arc::clone(&printer))
            .handle(
                &request("pause_resume/pause"),
                &context(&api, silent_target()),
            )
            .await
            .unwrap();

        let response = PauseResumeEndpoint::resume(Arc::clone(&printer))
            .handle(
                &request("pause_resume/resume"),
                &context(&api, silent_target()),
            )
            .await
            .unwrap();

        assert_eq!(response, json!({}));
        assert!(!object.is_paused());
        assert_eq!(emitted(&lines), ["// action:paused", "// action:resumed"]);
    }

    #[tokio::test]
    async fn test_cancel_runs_cancel_print_and_answers_empty() {
        let (printer, object) = machine();
        let lines = captured_lines(&printer);
        let api = Api::new();
        PauseResumeEndpoint::pause(Arc::clone(&printer))
            .handle(
                &request("pause_resume/pause"),
                &context(&api, silent_target()),
            )
            .await
            .unwrap();

        let response = PauseResumeEndpoint::cancel(printer)
            .handle(
                &request("pause_resume/cancel"),
                &context(&api, silent_target()),
            )
            .await
            .unwrap();

        assert_eq!(response, json!({}));
        assert!(!object.is_paused());
        assert_eq!(emitted(&lines), ["// action:paused", "// action:cancel"]);
    }

    #[tokio::test]
    async fn test_a_request_before_the_dispatcher_exists_reports_the_state() {
        // No `gcode` object, as before the config is loaded: the request gets
        // the printer's state message, exactly as `gcode/script` does.
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let api = Api::new();

        let err = PauseResumeEndpoint::pause(printer)
            .handle(
                &request("pause_resume/pause"),
                &context(&api, silent_target()),
            )
            .await
            .unwrap_err();

        assert!(matches!(err, ApiError::CommandError(_)), "{err:?}");
        assert!(err.to_string().contains("Starting up"), "{err}");
    }

    #[tokio::test]
    async fn test_without_the_section_the_unknown_command_is_answered_quietly() {
        // Upstream only installs these endpoints when `[pause_resume]` loads;
        // this port installs them unconditionally (like
        // `query_endstops/status`), and an unregistered command is an output
        // line rather than an error (`gcode.rs`), so the reply stays `{}`.
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        let lines = captured_lines(&printer);
        let api = Api::new();

        let response = PauseResumeEndpoint::pause(printer)
            .handle(
                &request("pause_resume/pause"),
                &context(&api, silent_target()),
            )
            .await
            .unwrap();

        assert_eq!(response, json!({}));
        assert_eq!(emitted(&lines), ["// Unknown command:\"PAUSE\""]);
    }
}
