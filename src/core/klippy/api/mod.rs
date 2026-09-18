//! Client-facing API layer — the JSON protocol the API server speaks.
//!
//! This is the layer external applications speak to: Fluidd, Mainsail,
//! Moonraker and KlipperScreen all connect to the API server klippy starts and
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
//! # Where it listens
//!
//! `--api-server` names the listener: a Unix Domain Socket path by default
//! (upstream's form), or a TCP address written `tcp:<host:port>` (see
//! [`ListenTarget`]). With no `--api-server` there is no server at all, which
//! is upstream's default too. Both transports carry the same protocol; TCP
//! exists so a client on another machine can reach the API, and it has no
//! authentication, so it belongs on a trusted network only.
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
//! # Concurrency
//!
//! One task accepts connections and one task serves each of them, so a client
//! that is slow, blocked or silent costs only its own task. Requests on one
//! connection stay ordered, which is what a client that pipelines expects,
//! while different connections make progress independently.
//!
//! The API server owns no thread of its own: it runs on whatever tokio runtime
//! the host runs on, and its socket I/O is asynchronous, unlike the serial port
//! and the host library, which need `spawn_blocking`.
//!
//! # Pushes
//!
//! Some endpoints answer and then keep sending. Those take a
//! `response_template` parameter and are handed the requesting connection as an
//! owned handle they may keep (see [`ResponseTemplate`] and [`PushTarget`]).
//! Pushing is synchronous and callable from any task or thread: it queues bytes
//! and wakes the connection's task, which is the only thing that touches the
//! socket.
//!
//! # Modules
//!
//! | Module | Owns |
//! |---|---|
//! | [`address`] | the `--api-server` value: socket path or TCP address |
//! | [`protocol`] | framing, request/reply shapes, parameter access, errors |
//! | [`registry`] | the endpoint table, dispatch, mux endpoints, remote methods |
//! | [`server`] | the listening socket and the per-connection task |
//! | [`endpoints`] | one file per client-facing endpoint |
//!
//! # Status
//!
//! The transport, protocol and registry layers are complete and tested; what is
//! missing is the endpoints themselves:
//!
//! * [`endpoints`] defines only the `info` endpoint, and only its *shape* — its
//!   handler body is a `todo!()`, so it is deliberately **not registered** yet:
//!   answering a panic would be worse than answering "no such endpoint";
//! * `emergency_stop`, `register_remote_method`, the `objects/*` family, the
//!   `gcode/*` family, `pause_resume/*` and the `*/dump_*` mux endpoints are not
//!   written yet, so `list_endpoints` reports only the built-in for now.
//!
//! The public reference for the endpoints themselves (paths, parameters,
//! response fields) is `docs/klippy/third-party-dev/api-reference.md`; keep the
//! two in step when an endpoint is added.

pub mod address;
pub mod endpoints;
pub mod protocol;
pub mod registry;
pub mod server;

pub use address::{AddressError, ListenTarget};
pub use protocol::{
    encode, ApiError, ApiErrorBody, Framing, MalformedRequest, Params, PushTarget, Request,
    Response, ResponseTemplate, DELIMITER,
};
pub use registry::{Api, Endpoint, EndpointContext, MuxEndpoint, RegistrationError};
pub use server::{ClientConnection, Server};
