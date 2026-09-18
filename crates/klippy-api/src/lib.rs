//! The Klipper API: the protocol the printer speaks, and the sockets it speaks
//! it over.
//!
//! This crate is the API itself, with no printer in it. It holds the wire format
//! ([`protocol`]), the address both sides meet at ([`address`]), and the server
//! that serves endpoints ([`server`] and [`registry`]). What it deliberately
//! does not hold is a single endpoint: the host supplies those, because an
//! endpoint is the only part of the API that has to know what a printer is.
//!
//! ```text
//!   klippy-api (this crate)          the host
//!   ┌───────────────────────┐       ┌──────────────────────┐
//!   │ protocol  address     │ ◀──── │ endpoints (info, …)  │
//!   │ registry  server      │       │ printer, gcode, mcu  │
//!   └───────────────────────┘       └──────────────────────┘
//!            ▲
//!            │                       a client
//!            └────────────────────── klippy-client, Moonraker, Fluidd
//! ```
//!
//! It is a crate of its own so that a client can depend on the protocol without
//! depending on the host: the two sides have to agree about framing, ids and
//! errors, and nothing else about them is shared. That is also why the server
//! lives here rather than with the endpoints — a client's tests want a real
//! server to talk to, and building one must not drag in a printer.
//!
//! # The wire
//!
//! Messages are JSON objects terminated by an ASCII `0x03` (ETX) byte:
//!
//! ```text
//! <json_1><0x03><json_2><0x03>...
//! ```
//!
//! Neither side waits for the peer: a single `recv` may carry several messages,
//! half a message, or half of one followed by a whole one, so the byte stream
//! has to be framed before it can be decoded (see [`Framing`]).
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
//! `response_template` parameter and are handed the requesting connection as an
//! owned handle they may keep (see [`ResponseTemplate`] and [`PushTarget`]).
//! Pushing is synchronous and callable from any task or thread: it queues bytes
//! and wakes the connection's task, which is the only thing that touches the
//! socket.
//!
//! # Where it listens
//!
//! `--api-server` names the listener: a Unix Domain Socket path by default
//! (upstream's form), or a TCP address written `tcp:<host>:port` (see
//! [`ApiTarget`]). Both transports carry the same protocol; TCP exists so a
//! client on another machine can reach the API, and it has no authentication, so
//! it belongs on a trusted network only.
//!
//! # Concurrency
//!
//! One task accepts connections and one task serves each of them, so a client
//! that is slow, blocked or silent costs only its own task. Requests on one
//! connection stay ordered, which is what a client that pipelines expects, while
//! different connections make progress independently. The API server owns no
//! thread of its own: it runs on whatever tokio runtime the host runs on, and its
//! socket I/O is asynchronous.
//!
//! # Modules
//!
//! | Module | Owns |
//! |---|---|
//! | [`address`] | the `--api-server` value: socket path or TCP address, and the socket both directions traffic in |
//! | [`protocol`] | framing, request/reply shapes, parameter access, errors |
//! | [`registry`] | the endpoint table, dispatch, mux endpoints, remote methods |
//! | [`server`] | the listening socket and the per-connection task |
//! | [`error`] | socket failures, as opposed to request failures |
//!
//! The public reference for the endpoints themselves (paths, parameters,
//! response fields) is `docs/klippy/third-party-dev/api-reference.md` in the
//! klipperx repository; keep the two in step when an endpoint is added.

pub mod address;
pub mod error;
pub mod protocol;
pub mod registry;
pub mod server;

pub use address::{AddressError, ApiTarget, Transport};
pub use error::TransportError;
pub use protocol::{
    encode, ApiError, ApiErrorBody, Framing, MalformedRequest, Params, PushTarget, Request,
    Response, ResponseTemplate, DELIMITER,
};
pub use registry::{Api, Endpoint, EndpointContext, MuxEndpoint, RegistrationError};
pub use server::{ClientConnection, Server};
