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
//! `info` is defined but only its *shape*: its handler body is a `todo!()`, so
//! it is deliberately **not registered** — a request to an unregistered path
//! gets `no registered callback`, whereas a registered `todo!()` would panic and
//! drop the connection. `objects/list` and `objects/query` are written and
//! tested but equally unregistered: [`register`] is the one call that installs
//! the server's objects and endpoints together, and no host calls it yet — the
//! printer's own lifecycle still waits for one. `objects/subscribe`,
//! `emergency_stop`, `register_remote_method`, the `gcode/*` family,
//! `pause_resume/*` and the `*/dump_*` mux endpoints are not written, so
//! `list_endpoints` currently reports only the built-in.
//!
//! The public reference for the endpoints themselves (paths, parameters,
//! response fields) is `docs/klippy/third-party-dev/api-reference.md`; keep the
//! two in step when an endpoint is added.

pub mod endpoints;
pub mod webhooks;

#[cfg(test)]
pub(crate) mod test_support;

use std::fmt;
use std::sync::Arc;

use crate::core::klippy::error::KlippyError;
use crate::core::klippy::printer::Printer;

pub use endpoints::{ObjectsList, ObjectsQuery};
pub use webhooks::{WebhooksStatus, WEBHOOKS_OBJECT};

// The API, re-exported so that this module is the host's single name for it.
// `klippy_api`'s own `RegistrationError` is not re-exported flat: this module
// has a `RegistrationError` of its own, for the whole server side, and the
// crate's stays reachable as [`registry::RegistrationError`].
pub use klippy_api::{address, protocol, registry, server};
pub use klippy_api::{
    AddressError, Api, ApiTarget, ClientConnection, Endpoint, EndpointContext, MuxEndpoint, Server,
    Transport, TransportError,
};

// ===========================================================================
// Registering the server's side of the host
// ===========================================================================

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
/// The caller must do this **before binding the listener**: a printer that is
/// served before its objects are registered is one a client can observe only
/// halfway up, and no client can provoke a duplicate registration to find out.
///
/// # Errors
/// Returns [`RegistrationError`] if a name or a path is already taken — a
/// wiring mistake in klippy, never something a client can cause.
pub fn register(api: &mut Api, printer: &Arc<Printer>) -> Result<(), RegistrationError> {
    printer
        .add_status_object(
            WEBHOOKS_OBJECT,
            Arc::new(WebhooksStatus::new(Arc::clone(printer))),
        )
        .map_err(RegistrationError::Status)?;
    api.register(ObjectsList::new(Arc::clone(printer)))
        .map_err(RegistrationError::Endpoint)?;
    api.register(ObjectsQuery::new(Arc::clone(printer)))
        .map_err(RegistrationError::Endpoint)?;
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
    Status(KlippyError),
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
    use serde_json::json;

    fn request(body: &str) -> klippy_api::Request {
        klippy_api::Request::parse(body.as_bytes()).expect("test body is a valid request")
    }

    #[test]
    fn test_registering_installs_the_servers_object_and_its_endpoints() {
        let printer = Arc::new(Printer::new());
        let mut api = Api::new();

        register(&mut api, &printer).unwrap();

        assert_eq!(printer.status_objects(), [WEBHOOKS_OBJECT]);
        assert_eq!(
            api.endpoints(),
            ["list_endpoints", "objects/list", "objects/query",]
        );
    }

    #[test]
    fn test_a_client_can_follow_the_state_through_the_registered_object() {
        let printer = Arc::new(Printer::new());
        let mut api = Api::new();
        register(&mut api, &printer).unwrap();

        let response = api
            .dispatch(
                &request(r#"{"method":"objects/query","params":{"objects":{"webhooks":null}}}"#),
                silent_target(),
            )
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
            .unwrap();
        assert_eq!(
            response["status"]["webhooks"],
            json!({"state": "shutdown", "state_message": "Printer is halted"})
        );
    }

    #[test]
    fn test_registering_twice_is_a_wiring_mistake_not_a_client_error() {
        let printer = Arc::new(Printer::new());
        let mut api = Api::new();
        register(&mut api, &printer).unwrap();

        let err = register(&mut api, &printer).unwrap_err();

        assert!(
            matches!(err, RegistrationError::Status(_)),
            "the object is registered before the endpoints: {err}"
        );
        assert!(err.to_string().contains("webhooks"), "{err}");
    }
}
