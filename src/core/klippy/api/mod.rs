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
//! Only `info` is defined, and only its *shape*: its handler body is a
//! `todo!()`, so it is deliberately **not registered** — a request to an
//! unregistered path gets `no registered callback`, whereas a registered
//! `todo!()` would panic and drop the connection. `emergency_stop`,
//! `register_remote_method`, the `objects/*` family, the `gcode/*` family,
//! `pause_resume/*` and the `*/dump_*` mux endpoints are not written yet, so
//! `list_endpoints` currently reports only the built-in.
//!
//! The public reference for the endpoints themselves (paths, parameters,
//! response fields) is `docs/klippy/third-party-dev/api-reference.md`; keep the
//! two in step when an endpoint is added.

pub mod endpoints;

// The API, re-exported so that this module is the host's single name for it.
pub use klippy_api::{address, protocol, registry, server};
pub use klippy_api::{
    AddressError, Api, ApiTarget, ClientConnection, Endpoint, EndpointContext, MuxEndpoint,
    RegistrationError, Server, Transport, TransportError,
};
