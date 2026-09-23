//! The host's part of the API: the endpoints.
//!
//! The API itself — the `0x03`-framed JSON protocol, the socket it travels over,
//! and the server that dispatches requests — lives in the [`klippy_api`] crate,
//! because a client needs it too and a client must not depend on a printer. What
//! is left here is the one thing that does: the endpoints, each of which reads
//! printer state, drives g-code, or both.
//!
//! ```text
//!                     ┌──────────────┐
//!         client ───▶ │  klippy-api  │ ───▶ endpoints (this module)
//!                     └──────────────┘            │
//!                                                  ▼
//!                                          printer / gcode
//!                          ▲
//!     msg  ←──  mcu  ←──  cmd  ←──  event
//! ```
//!
//! [`klippy_api`]'s own documentation is the reference for the wire format and
//! for the server's concurrency model; this module only adds the endpoints. The
//! shared types are re-exported below, so host code can keep naming them through
//! this module rather than reaching across the crate boundary at every use.
//!
//! # Status
//!
//! [`register`] installs the server's own object and every endpoint that is
//! written: `webhooks`, `info`, `objects/list`, `objects/query`,
//! `objects/subscribe`, the five `gcode/*` endpoints, `emergency_stop`,
//! `query_endstops/status` and `register_remote_method`. The `*/dump_*` mux
//! endpoints are installed through the `webhooks` object —
//! [`WebhooksStatus::register_mux_endpoint`] — once an extras module registers
//! one. What is left of the documented surface is `pause_resume/*`, which waits
//! for the `pause_resume` object.
//!
//! The public reference for the endpoints themselves (paths, parameters,
//! response fields) is `docs/klippy/third-party-dev/api-reference.md`; keep the
//! two in step when an endpoint is added.

pub mod endpoints;
pub mod start_args;
pub mod webhooks;

#[cfg(test)]
pub(crate) mod test_support;

use std::fmt;
use std::sync::Arc;

use crate::core::klippy::error::ConfigError;
use crate::core::klippy::printer::Printer;

pub use endpoints::{
    GcodeHelp, GcodeRestart, GcodeScript, GcodeSubscribeOutput, Info, ObjectsList, ObjectsQuery,
    ObjectsSubscribe,
};
pub use start_args::StartArgs;
pub use webhooks::{WebhooksStatus, WEBHOOKS_OBJECT};

// The API, re-exported so that this module is the host's single name for it.
// `klippy_api`'s own `RegistrationError` is not re-exported flat: this module
// has a `RegistrationError` of its own, for the whole server side, and the
// crate's stays reachable as [`registry::RegistrationError`].
pub use klippy_api::{address, protocol, registry, server};
pub use klippy_api::{
    AddressError, Api, ApiTarget, ClientConnection, Endpoint, EndpointContext, EndpointFuture,
    MuxEndpoint, Server, Transport, TransportError,
};

// ===========================================================================
// Registering the server's side of the host
// ===========================================================================

/// What an endpoint installer is handed: the machine it serves and the host's
/// start arguments.
pub(crate) struct ApiWiring<'a> {
    /// The machine every endpoint reads or drives.
    pub(crate) printer: &'a Arc<Printer>,
    /// The host's start arguments; `info` reports them, the rest ignore them.
    pub(crate) start_args: &'a StartArgs,
}

/// The signature every `endpoint!` declaration names.
pub(crate) type EndpointInstaller = fn(&mut Api, &ApiWiring<'_>) -> Result<(), RegistrationError>;

// The endpoint table, generated from the `endpoint!` declarations (see
// `endpoints/mod.rs`).
include!(concat!(env!("OUT_DIR"), "/endpoint_installers.rs"));

/// Install everything the API server owns onto a machine.
///
/// One call, in one place, because the order it encodes is the contract:
///
/// 1. the server's own printer object is registered first, so that a client
///    that has connected can always ask for `webhooks` — upstream registers the
///    same object in `Printer.__init__`, *before* its socket exists
///    (`klippy/webhooks.py:564`, reached from `klippy/klippy.py:36-40`);
/// 2. then the endpoints, so that no path is half-built when a request arrives.
///
/// `start_args` is the host's own: the `info` endpoint reports it, and the
/// machine never sees it.
///
/// The caller must do this **before binding the listener**: a printer that is
/// served before its objects are registered is one a client can observe only
/// halfway up, and no client can provoke a duplicate registration to find out.
///
/// # Errors
/// Returns [`RegistrationError`] if a name or a path is already taken — a
/// wiring mistake in klippy, never something a client can cause.
pub fn register(
    api: &mut Api,
    printer: &Arc<Printer>,
    start_args: StartArgs,
) -> Result<(), RegistrationError> {
    let webhooks = webhooks::install(printer).map_err(RegistrationError::Status)?;
    // A request handler that fails on its own account takes the printer down,
    // as upstream's `_process_request` does (`klippy/webhooks.py:271-276`). The
    // decision belongs to the host, so the API crate only knows the hook.
    api.set_internal_error_hook({
        let printer = Arc::clone(printer);
        Arc::new(move |msg: &str| printer.invoke_shutdown(msg))
    });
    let wiring = ApiWiring {
        printer,
        start_args: &start_args,
    };
    for install in ENDPOINT_INSTALLERS {
        install(api, &wiring)?;
    }
    // Mux endpoints are registered by modules on the `webhooks` object while
    // the config is read, and the host builds this table afterwards, so what
    // they collected goes in here. Upstream registers the path on the first
    // instance (`klippy/webhooks.py:330-334`).
    for registration in webhooks.take_mux_endpoints() {
        api.register_mux(
            &registration.path,
            &registration.key,
            registration.value.as_deref(),
            registration.handler,
        )
        .map_err(RegistrationError::Endpoint)?;
    }
    Ok(())
}

/// A wiring mistake made while registering the host's API side.
///
/// Both variants mean the same thing: klippy tried to install the same name
/// twice. They are separate because the two halves have their own error types —
/// a printer object is the machine's, an endpoint path is the API crate's.
#[derive(Debug)]
pub enum RegistrationError {
    /// A printer object name was already taken.
    Status(ConfigError),
    /// An endpoint path was already taken.
    Endpoint(klippy_api::RegistrationError),
}

impl fmt::Display for RegistrationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RegistrationError::Status(err) => write!(f, "{err}"),
            RegistrationError::Endpoint(err) => write!(f, "{err}"),
        }
    }
}

impl std::error::Error for RegistrationError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            RegistrationError::Status(err) => Some(err),
            RegistrationError::Endpoint(err) => Some(err),
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::test_support::silent_target;
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::json;

    fn request(body: &str) -> klippy_api::Request {
        klippy_api::Request::parse(body.as_bytes()).expect("test body is a valid request")
    }

    /// The start arguments a host would have gathered.
    fn start_args() -> StartArgs {
        StartArgs::collect("/tmp/printer.cfg", None)
    }

    #[tokio::test]
    async fn test_registering_installs_the_servers_object_and_its_endpoints() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let mut api = Api::new();

        register(&mut api, &printer, start_args()).unwrap();

        assert_eq!(printer.objects(), [WEBHOOKS_OBJECT]);
        assert_eq!(
            api.endpoints(),
            [
                "emergency_stop",
                "gcode/firmware_restart",
                "gcode/help",
                "gcode/restart",
                "gcode/script",
                "gcode/subscribe_output",
                "info",
                "list_endpoints",
                "objects/list",
                "objects/query",
                "objects/subscribe",
                "query_endstops/status",
                "register_remote_method",
            ]
        );
    }

    #[tokio::test]
    async fn test_a_client_can_reach_info_through_the_registry() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let mut api = Api::new();
        register(&mut api, &printer, start_args()).unwrap();

        let response = api
            .dispatch(&request(r#"{"method":"info"}"#), silent_target())
            .await
            .unwrap();

        assert_eq!(response["state"], "startup");
        assert_eq!(response["config_file"], "/tmp/printer.cfg");
    }

    #[tokio::test]
    async fn test_a_mux_endpoint_registered_on_webhooks_reaches_the_table() {
        // The core integration: an extras module registers a mux endpoint on the
        // `webhooks` object while the config is read, and `register` installs it
        // into the API table afterwards.
        struct Dump(&'static str);

        impl crate::core::klippy::api::registry::MuxEndpoint for Dump {
            fn handle<'a>(
                &'a self,
                _request: &'a klippy_api::Request,
                _context: &'a EndpointContext<'a>,
            ) -> EndpointFuture<'a> {
                Box::pin(async move { Ok(json!({"instance": self.0})) })
            }
        }

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        // Installed before the config, as the host does.
        let webhooks = crate::core::klippy::api::webhooks::install(&printer).unwrap();
        webhooks
            .register_mux_endpoint(
                "adxl345/dump_adxl345",
                "sensor",
                Some("adxl345"),
                Arc::new(Dump("adxl345")),
            )
            .unwrap();
        webhooks
            .register_mux_endpoint(
                "adxl345/dump_adxl345",
                "sensor",
                Some("second"),
                Arc::new(Dump("second")),
            )
            .unwrap();
        let mut api = Api::new();
        register(&mut api, &printer, start_args()).unwrap();

        let response = api
            .dispatch(
                &request(
                    r#"{"method":"adxl345/dump_adxl345",
                       "params":{"sensor":"second"}}"#,
                ),
                silent_target(),
            )
            .await
            .unwrap();

        assert_eq!(response, json!({"instance": "second"}));
        assert!(api
            .endpoints()
            .contains(&"adxl345/dump_adxl345".to_string()));
    }

    #[tokio::test]
    async fn test_a_client_can_follow_the_state_through_the_registered_object() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let mut api = Api::new();
        register(&mut api, &printer, start_args()).unwrap();

        let response = api
            .dispatch(
                &request(r#"{"method":"objects/query","params":{"objects":{"webhooks":null}}}"#),
                silent_target(),
            )
            .await
            .unwrap();
        assert_eq!(
            response["status"]["webhooks"],
            json!({"state": "startup", "state_message": "Starting up"})
        );

        printer.invoke_shutdown("Printer is halted");

        let response = api
            .dispatch(
                &request(r#"{"method":"objects/query","params":{"objects":{"webhooks":null}}}"#),
                silent_target(),
            )
            .await
            .unwrap();
        assert_eq!(
            response["status"]["webhooks"],
            json!({"state": "shutdown", "state_message": "Printer is halted"})
        );
    }

    #[tokio::test]
    async fn test_registering_twice_is_a_wiring_mistake_not_a_client_error() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let mut api = Api::new();
        register(&mut api, &printer, start_args()).unwrap();

        // `webhooks` is idempotent (a host may install it before the config),
        // so the second pass trips on the endpoint paths a first pass took.
        let err = register(&mut api, &printer, start_args()).unwrap_err();

        assert!(
            matches!(err, RegistrationError::Endpoint(_)),
            "the endpoints are registered after the object: {err}"
        );
        assert!(err.to_string().contains("emergency_stop"), "{err}");
    }
}
