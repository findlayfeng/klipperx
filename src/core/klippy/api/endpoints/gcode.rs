//! `gcode/help`, `gcode/script`, `gcode/restart`, `gcode/firmware_restart`,
//! `gcode/subscribe_output`.
//!
//! The API's door into the G-Code dispatcher (`core/klippy/gcode.rs`), upstream
//! `GCodeHelper` (`klippy/webhooks.py:429-452`):
//!
//! | Endpoint | Answer |
//! |---|---|
//! | `gcode/help` | the flat `{command: help}` table |
//! | `gcode/script` | `{}`, or an `error` reply with the command's message |
//! | `gcode/restart` | `{}`; runs `RESTART`, or restarts the machine directly |
//! | `gcode/firmware_restart` | `{}`; runs `FIRMWARE_RESTART`, or restarts the machine directly |
//! | `gcode/subscribe_output` | `{}`; later lines are pushed as `{response: line}` |
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
//! The two restart endpoints are the exception: a restart is the way out of a
//! machine that failed to come up, so it must not depend on the `gcode` object
//! that failure took down with it. With no dispatcher they call
//! `Printer::request_exit` themselves — the same thing the `RESTART` command
//! ends up doing — so a client that hit a config error can still ask for a
//! restart instead of being stuck until the host process is restarted.
//!
//! Nothing is lost by skipping the command: `RESTART` and `FIRMWARE_RESTART` are
//! registered `when_not_ready` (`gcode.rs`), and the ready-only preamble they
//! run (fire `gcode:request_restart`, dwell, wait for queued moves) is skipped
//! whenever the printer is not ready — which is exactly the state that removed
//! the dispatcher.
//!
//! # Output subscriptions
//!
//! A subscriber is an [`OutputHandler`] over the requesting connection: it
//! pushes `template + {response: line}` and reports itself closed with the
//! connection, so the dispatcher drops it at the next line. Upstream keeps the
//! same map and prunes it on disconnect.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use serde_json::json;

use crate::core::klippy::api::protocol::{ApiError, PushTarget, Request, ResponseTemplate};
use crate::core::klippy::api::registry::{Endpoint, EndpointContext, EndpointFuture};
use crate::core::klippy::api::{Api, ApiWiring, RegistrationError};
use crate::core::klippy::gcode::{GCodeDispatch, OutputHandler, GCODE_OBJECT};
use crate::core::klippy::printer::Printer;

endpoint!(install);

/// Install every `gcode/*` endpoint.
///
/// These resolve `gcode` per request: it is registered while the config is
/// loaded, after this runs (`api::register` is called before `load_config` so
/// that `webhooks` is in place before the socket).
pub(crate) fn install(api: &mut Api, wiring: &ApiWiring<'_>) -> Result<(), RegistrationError> {
    let printer = Arc::clone(wiring.printer);
    api.register(GcodeHelp::new(Arc::clone(&printer)))
        .map_err(RegistrationError::Endpoint)?;
    api.register(GcodeScript::new(Arc::clone(&printer)))
        .map_err(RegistrationError::Endpoint)?;
    api.register(GcodeRestart::restart(Arc::clone(&printer)))
        .map_err(RegistrationError::Endpoint)?;
    api.register(GcodeRestart::firmware_restart(Arc::clone(&printer)))
        .map_err(RegistrationError::Endpoint)?;
    let output = GcodeSubscribeOutput::new(Arc::clone(&printer));
    // Keep the subscriptions across a `RESTART`, which rebuilds the dispatcher
    // they are registered on.
    output.watch_restarts();
    api.register(output).map_err(RegistrationError::Endpoint)
}

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

    fn handle<'a>(
        &'a self,
        _request: &'a Request,
        _context: &'a EndpointContext<'a>,
    ) -> EndpointFuture<'a> {
        Box::pin(async move {
            let gcode = gcode(&self.printer)?;
            Ok(json!(gcode.command_help()))
        })
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

    fn handle<'a>(
        &'a self,
        request: &'a Request,
        _context: &'a EndpointContext<'a>,
    ) -> EndpointFuture<'a> {
        Box::pin(async move {
            let script = request.params().get_str("script")?;
            let gcode = gcode(&self.printer)?;
            gcode
                .run_script(script)
                .await
                .map_err(|err| ApiError::CommandError(err.to_string()))?;
            Ok(json!({}))
        })
    }
}

/// `gcode/restart` and `gcode/firmware_restart` — run the restart command.
///
/// The command itself decides what a restart means (`request_exit`); the restart
/// loop (`src/klippy.rs`) rebuilds the machine or exits on the result. When no
/// dispatcher exists (a config that failed to load), the endpoint skips the
/// command and asks the printer to exit with the same result itself.
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

    fn handle<'a>(
        &'a self,
        _request: &'a Request,
        _context: &'a EndpointContext<'a>,
    ) -> EndpointFuture<'a> {
        Box::pin(async move {
            // No dispatcher: the config that would have built it never got
            // there, and a restart is the only way to try again. The command's
            // ready-gated preamble would be skipped here anyway (see the module
            // note).
            let Ok(gcode) = gcode(&self.printer) else {
                self.printer.request_exit(self.script);
                return Ok(json!({}));
            };
            gcode
                .run_script(self.script)
                .await
                .map_err(|err| ApiError::CommandError(err.to_string()))?;
            Ok(json!({}))
        })
    }
}

/// One connection's output subscription.
///
/// Emits `template + {response: line}` and reports itself closed with the
/// connection, so the dispatcher drops it at the next line.
struct Subscription {
    client: Arc<dyn PushTarget>,
    template: ResponseTemplate,
}

impl OutputHandler for Subscription {
    fn emit(&self, line: &str) {
        if !self.client.is_closed() {
            let params = json!({ "response": line });
            self.client.push(self.template.message(params));
        }
    }

    fn is_closed(&self) -> bool {
        self.client.is_closed()
    }
}

/// How often the re-attach task checks whether the dispatcher was rebuilt.
///
/// A restart replaces the `gcode` object, and with it the dispatcher the
/// subscribers were registered on. The task notices the new one and puts the
/// live subscriptions back, so a client does not have to resubscribe after a
/// `RESTART` (the connection, and the endpoint, survive it).
const REATTACH_INTERVAL: Duration = Duration::from_millis(250);

/// Every connection's output subscription, shared with the re-attach task.
type Subscribers = Arc<Mutex<Vec<Arc<Subscription>>>>;

/// The dispatcher the subscriptions are currently registered on.
type Attached = Arc<Mutex<Option<Arc<GCodeDispatch>>>>;

/// `gcode/subscribe_output` — push every line the dispatcher emits.
pub struct GcodeSubscribeOutput {
    printer: Arc<Printer>,
    subscribers: Subscribers,
    attached: Attached,
}

impl GcodeSubscribeOutput {
    /// Build the endpoint over the machine whose output it subscribes to.
    pub fn new(printer: Arc<Printer>) -> Self {
        Self {
            printer,
            subscribers: Arc::new(Mutex::new(Vec::new())),
            attached: Arc::new(Mutex::new(None)),
        }
    }

    /// Start the task that keeps subscriptions across a rebuilt dispatcher.
    ///
    /// Does nothing when there is no runtime (the endpoint tests), which is
    /// also where nothing is restarted.
    pub fn watch_restarts(&self) {
        let Ok(handle) = tokio::runtime::Handle::try_current() else {
            return;
        };
        let printer = Arc::clone(&self.printer);
        let subscribers = Arc::clone(&self.subscribers);
        let attached = Arc::clone(&self.attached);
        handle.spawn(async move {
            loop {
                tokio::time::sleep(REATTACH_INTERVAL).await;
                reattach(&printer, &subscribers, &attached);
            }
        });
    }
}

impl Endpoint for GcodeSubscribeOutput {
    fn path(&self) -> &'static str {
        "gcode/subscribe_output"
    }

    fn handle<'a>(
        &'a self,
        request: &'a Request,
        context: &'a EndpointContext<'a>,
    ) -> EndpointFuture<'a> {
        Box::pin(async move {
            let template = ResponseTemplate::from_params(&request.params())?;
            let gcode = gcode(&self.printer)?;
            let subscription = Arc::new(Subscription {
                client: context.client.clone(),
                template,
            });
            self.subscribers
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(Arc::clone(&subscription));
            gcode.register_output_handler(subscription);
            *self.attached.lock().unwrap_or_else(|p| p.into_inner()) = Some(gcode);
            Ok(json!({}))
        })
    }
}

/// Put every live subscription on the dispatcher `printer` currently has.
///
/// Called when the dispatcher is not the one the subscriptions are on — a
/// restart built a new one — and when the subscriptions were just created. A
/// closed connection is dropped first, so a gone client is not reattached.
fn reattach(printer: &Printer, subscribers: &Subscribers, attached: &Attached) {
    let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) else {
        return;
    };
    let current = {
        let attached = attached.lock().unwrap_or_else(|p| p.into_inner());
        attached
            .as_ref()
            .is_some_and(|old| Arc::ptr_eq(old, &gcode))
    };
    if current {
        return;
    }
    let live: Vec<Arc<Subscription>> = {
        let mut subscribers = subscribers.lock().unwrap_or_else(|p| p.into_inner());
        subscribers.retain(|subscription| !subscription.is_closed());
        subscribers.clone()
    };
    for subscription in live {
        gcode.register_output_handler(subscription);
    }
    *attached.lock().unwrap_or_else(|p| p.into_inner()) = Some(gcode);
}
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::registry::Api;
    use crate::core::klippy::api::test_support::{context, silent_target};
    use crate::core::klippy::gcode::{sync, CommandError, CommandHandler};
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::Value;

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
        printer.send_event(&crate::core::klippy::event::KlippyEvent::KlippyReady);
        printer
    }

    fn gcode(printer: &Arc<Printer>) -> Arc<GCodeDispatch> {
        printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap()
    }

    #[tokio::test]
    async fn test_the_paths_are_the_documented_ones() {
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
            GcodeRestart::firmware_restart(Arc::clone(&printer)).path(),
            "gcode/firmware_restart"
        );
        assert_eq!(
            GcodeSubscribeOutput::new(printer).path(),
            "gcode/subscribe_output"
        );
    }

    #[tokio::test]
    async fn test_subscribing_pushes_output_lines() {
        use crate::core::klippy::api::test_support::RecordingTarget;

        let printer = printer();
        let api = Api::new();
        let target = RecordingTarget::new();
        let body = r#"{"method":"gcode/subscribe_output","params":{"response_template":{"method":"gcode:output","id":null}}}"#;

        GcodeSubscribeOutput::new(Arc::clone(&printer))
            .handle(&request(body), &context(&api, target.clone()))
            .await
            .unwrap();

        // Anything the dispatcher says now reaches the subscriber.
        gcode(&printer).run_script("M115").await.unwrap();

        let pushes = target.pushes();
        assert_eq!(pushes[0]["method"], "gcode:output");
        assert_eq!(pushes[0]["id"], Value::Null);
        assert!(
            pushes[0]["params"]["response"]
                .as_str()
                .unwrap()
                .contains("FIRMWARE_NAME"),
            "{pushes:?}"
        );
    }

    #[tokio::test]
    async fn test_subscriptions_survive_a_rebuilt_dispatcher() {
        // A `RESTART` replaces the `gcode` object, so the dispatcher a client
        // subscribed to is gone. The endpoint's re-attach puts the live
        // subscription on the new one instead of leaving it silent until the
        // client notices and resubscribes (`TODO` FW8).
        use crate::core::klippy::api::test_support::RecordingTarget;

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let first = Arc::new(GCodeDispatch::new(Arc::clone(&printer)));
        printer.add_object(GCODE_OBJECT, first.clone()).unwrap();

        let api = Api::new();
        let target = RecordingTarget::new();
        let endpoint = GcodeSubscribeOutput::new(Arc::clone(&printer));
        let body = r#"{"method":"gcode/subscribe_output","params":{"response_template":{"method":"gcode:output","id":null}}}"#;
        endpoint
            .handle(&request(body), &context(&api, target.clone()))
            .await
            .unwrap();

        first.run_script("M115").await.unwrap();
        let after_first = target.pushes().len();
        assert!(
            after_first > 0,
            "the first dispatcher reached the subscriber"
        );

        // What a restart does: the old dispatcher is dropped and a new one is
        // loaded, then the re-attach task runs.
        printer.reset_for_restart("restart");
        let second = Arc::new(GCodeDispatch::new(Arc::clone(&printer)));
        printer.add_object(GCODE_OBJECT, second.clone()).unwrap();

        reattach(&printer, &endpoint.subscribers, &endpoint.attached);
        second.run_script("M115").await.unwrap();

        let pushes = target.pushes();
        assert!(
            pushes.len() > after_first,
            "the rebuilt dispatcher reaches the same subscriber: {pushes:?}"
        );
    }

    #[tokio::test]
    async fn test_help_returns_the_flat_command_table() {
        let printer = printer();
        let handler: CommandHandler = sync(|_| Ok(()));
        gcode(&printer)
            .register_command("SET_PIN", handler, Some("Set a pin"), false)
            .unwrap();
        let api = Api::new();

        let response = GcodeHelp::new(Arc::clone(&printer))
            .handle(
                &request(r#"{"method":"gcode/help"}"#),
                &context(&api, silent_target()),
            )
            .await
            .unwrap();

        assert_eq!(response["SET_PIN"], "Set a pin");
    }

    #[tokio::test]
    async fn test_script_runs_and_answers_empty() {
        let printer = printer();
        let api = Api::new();

        let response = GcodeScript::new(Arc::clone(&printer))
            .handle(
                &request(r#"{"method":"gcode/script","params":{"script":"M115"}}"#),
                &context(&api, silent_target()),
            )
            .await
            .unwrap();

        assert_eq!(response, json!({}));
    }

    #[tokio::test]
    async fn test_a_command_error_becomes_an_error_reply() {
        let printer = printer();
        gcode(&printer)
            .register_command(
                "FAIL",
                sync(|_| Err(CommandError::new("boom"))),
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
            .await
            .unwrap_err();

        assert_eq!(err, ApiError::CommandError("boom".to_string()));
        assert!(!err.is_internal(), "a command error must not stop klippy");
    }

    #[tokio::test]
    async fn test_script_is_required() {
        let printer = printer();
        let api = Api::new();

        let err = GcodeScript::new(printer)
            .handle(
                &request(r#"{"method":"gcode/script"}"#),
                &context(&api, silent_target()),
            )
            .await
            .unwrap_err();

        assert_eq!(err, ApiError::MissingArgument("script".to_string()));
    }

    #[tokio::test]
    async fn test_restart_runs_the_restart_command() {
        let printer = printer();
        let api = Api::new();

        GcodeRestart::firmware_restart(Arc::clone(&printer))
            .handle(
                &request(r#"{"method":"gcode/firmware_restart"}"#),
                &context(&api, silent_target()),
            )
            .await
            .unwrap();

        // The built-in FIRMWARE_RESTART asks the printer to exit with that
        // result; `run` returns it without waiting because it is already set.
        assert_eq!(printer.run(), "firmware_restart");
    }

    #[tokio::test]
    async fn test_restart_without_a_dispatcher_still_restarts() {
        // No `gcode` object: the config that would build one failed to load.
        // The client must still be able to ask for a restart.
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let api = Api::new();

        let response = GcodeRestart::restart(Arc::clone(&printer))
            .handle(
                &request(r#"{"method":"gcode/restart"}"#),
                &context(&api, silent_target()),
            )
            .await
            .unwrap();

        assert_eq!(response, json!({}));
        assert_eq!(printer.run(), "restart");
    }

    #[tokio::test]
    async fn test_firmware_restart_without_a_dispatcher_still_restarts() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let api = Api::new();

        let response = GcodeRestart::firmware_restart(Arc::clone(&printer))
            .handle(
                &request(r#"{"method":"gcode/firmware_restart"}"#),
                &context(&api, silent_target()),
            )
            .await
            .unwrap();

        assert_eq!(response, json!({}));
        assert_eq!(printer.run(), "firmware_restart");
    }

    #[tokio::test]
    async fn test_a_request_before_the_dispatcher_exists_reports_the_state() {
        // No `gcode` object, as before the config is loaded.
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let api = Api::new();

        let err = GcodeScript::new(printer)
            .handle(
                &request(r#"{"method":"gcode/script","params":{"script":"M115"}}"#),
                &context(&api, silent_target()),
            )
            .await
            .unwrap_err();

        assert!(matches!(err, ApiError::CommandError(_)), "{err:?}");
        assert!(err.to_string().contains("Starting up"), "{err}");
    }
}
