//! Client-facing API layer — the JSON protocol on the Unix Domain Socket.
//!
//! This is the layer external applications speak to: Fluidd, Mainsail,
//! Moonraker and KlipperScreen all connect to the socket klippy creates and
//! send it JSON requests. It is the counterpart of klipper's
//! `klippy/webhooks.py`, and it sits above every other module here — it is the
//! only layer that is allowed to know about printer objects, g-code and the
//! printer's lifecycle at the same time.
//!
//! ```text
//!                     ┌──────────┐
//!         client ───▶ │   api    │ ───▶ printer / gcode (endpoints only)
//!                     └──────────┘
//!                          ▲
//!     msg  ←──  mcu  ←──  cmd  ←──  event
//! ```
//!
//! # The wire
//!
//! Messages are JSON objects terminated by an ASCII `0x03` (ETX) byte:
//!
//! ```text
//! <json_1><0x03><json_2><0x03>...
//! ```
//!
//! Neither side waits for the peer: a single `recv` may carry several
//! messages, half a message, or half of one followed by a whole one, so the
//! byte stream has to be framed before it can be decoded (see [`Framing`]).
//!
//! A request is `{"id": …, "method": …, "params": {…}}`. `id` is echoed back so
//! a client can match replies to requests; a request that carries no `id` (or
//! `null`) is fire-and-forget and gets **no reply at all**, not even for an
//! error. A reply is `{"id": …, "result": …}` or
//! `{"id": …, "error": {"error": "WebRequestError", "message": …}}`.
//!
//! # Pushes
//!
//! Some endpoints answer and then keep sending. Those take a
//! `response_template` parameter and are handed the requesting connection, so
//! they can push messages built from that template (see [`ResponseTemplate`]
//! and [`PushTarget`]).
//!
//! # Modules
//!
//! | Module | Owns |
//! |---|---|
//! | [`protocol`] | framing, request/reply shapes, parameter access, errors |
//! | [`registry`] | the endpoint table, dispatch, mux endpoints, remote methods |
//! | [`server`] | the Unix Domain Socket and per-connection state |
//! | [`endpoints`] | one file per client-facing endpoint |
//!
//! # Status
//!
//! This module is a skeleton. The protocol and registry layers are complete and
//! tested; what is missing is deliberate:
//!
//! * the socket accept loop in [`server::Server::run`] is a `todo!()`;
//! * [`endpoints`] defines only the `info` endpoint, and only its *shape* — its
//!   handler body is a `todo!()`;
//! * `emergency_stop`, `register_remote_method`, the `objects/*` family, the
//!   `gcode/*` family and the `*/dump_*` mux endpoints are not written yet.
//!
//! The public reference for the endpoints themselves (paths, parameters,
//! response fields) is `docs/klippy/third-party-dev/api-reference.md`; keep the
//! two in step when an endpoint is added.

pub mod endpoints;
pub mod protocol;
pub mod registry;
pub mod server;

pub use protocol::{
    encode, ApiError, ApiErrorBody, Framing, MalformedRequest, Params, PushTarget, Request,
    Response, ResponseTemplate, DELIMITER,
};
pub use registry::{Api, Endpoint, EndpointContext, MuxEndpoint, RegistrationError};
pub use server::{ClientConnection, Server};
