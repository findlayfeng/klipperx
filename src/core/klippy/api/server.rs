//! The Unix Domain Socket, and the per-connection state on top of it.
//!
//! klippy is started with `-a <path>`; it creates a Unix Domain Socket there
//! and every client that connects speaks the protocol in
//! [`super::protocol`]. Transport and protocol are split so that a connection
//! can be driven — framed, dispatched, replied to — without a socket, which is
//! what [`ClientConnection::receive`] does and what the tests exercise.
//!
//! # Status
//!
//! [`Server::run`] is not written yet: binding the socket, removing a stale
//! socket file, accepting connections, and driving each one from the reactor
//! are all `todo!()`. Everything a connection needs once it exists is here.
//!
//! # Shutdown
//!
//! A `Server` is dropped with the printer. Closing the listener and every
//! connection is a `todo!()` in [`Server::run`] for now; upstream also dumps a
//! per-client request log on an analysed shutdown, which belongs with the
//! socket write path.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use serde_json::Value;
use tracing::warn;

use crate::core::klippy::error::KlippyError;

use super::protocol::{encode, Framing, MalformedRequest, PushTarget, Request};
use super::registry::Api;

/// The API server: a socket path and the endpoint table it serves.
pub struct Server {
    socket_path: PathBuf,
    api: Arc<Api>,
}

impl Server {
    /// Create a server bound to `socket_path`, without touching the filesystem.
    pub fn new(socket_path: impl Into<PathBuf>, api: Arc<Api>) -> Self {
        Self {
            socket_path: socket_path.into(),
            api,
        }
    }

    /// The socket path clients connect to.
    pub fn socket_path(&self) -> &Path {
        &self.socket_path
    }

    /// The endpoint table this server serves.
    pub fn api(&self) -> &Arc<Api> {
        &self.api
    }

    /// Serve clients until the printer shuts down.
    ///
    /// # Errors
    /// Returns [`KlippyError`] if the socket cannot be created or served.
    pub async fn run(self) -> Result<(), KlippyError> {
        // TODO: remove a stale socket file at `self.socket_path`, bind a
        // `tokio::net::UnixListener`, and accept connections. Each accepted
        // socket becomes an `Arc<ClientConnection>`; reads go through
        // `ClientConnection::receive` and the bytes it queues are written back
        // when the reactor reports the socket writable.
        //
        // TODO: on shutdown, stop accepting, close every connection (which
        // makes `PushTarget::is_closed` true and drops subscriptions), and
        // remove the socket file.
        todo!("Unix Domain Socket accept loop is not implemented yet")
    }
}

/// One connected client.
///
/// A connection owns everything that is per-client: the framing state for bytes
/// that arrived in pieces, and the outbox of replies and pushes waiting to be
/// written. It is the [`PushTarget`] endpoints are handed, so a subscription
/// can be dropped the moment the client goes away.
pub struct ClientConnection {
    api: Arc<Api>,
    framing: Mutex<Framing>,
    outbox: Mutex<Vec<u8>>,
    closed: AtomicBool,
}

impl ClientConnection {
    /// Create a connection that dispatches against `api`.
    pub fn new(api: Arc<Api>) -> Arc<Self> {
        Arc::new(Self {
            api,
            framing: Mutex::new(Framing::new()),
            outbox: Mutex::new(Vec::new()),
            closed: AtomicBool::new(false),
        })
    }

    /// Feed bytes received from the socket, queueing a reply for each request.
    ///
    /// Returns the number of messages that were dropped because they were not
    /// valid requests. Those are logged, not answered: a body that cannot be
    /// decoded cannot be trusted to name an `id` to answer, and upstream drops
    /// them the same way. The connection stays usable either way.
    ///
    /// Requests without an `id` are dispatched but answered with silence, which
    /// is [`Request::respond`]'s decision, not this one's.
    pub fn receive(self: &Arc<Self>, chunk: &[u8]) -> usize {
        let bodies = self
            .framing
            .lock()
            .expect("framing state is not poisoned")
            .push(chunk);

        let mut dropped = 0;
        for body in bodies {
            let request = match Request::parse(&body) {
                Ok(request) => request,
                Err(error) => {
                    dropped += 1;
                    report_malformed(&error, &body);
                    continue;
                }
            };

            let outcome = self.api.dispatch(&request, self.as_ref());
            if let Some(response) = request.respond(outcome) {
                let message = serde_json::to_value(&response)
                    .expect("a Response always serializes to a JSON value");
                self.queue(&message);
            }
        }
        dropped
    }

    /// Take everything queued for writing, leaving the outbox empty.
    ///
    /// The caller writes the bytes to the socket; what comes back is a
    /// contiguous run of whole messages, each already delimiter-terminated.
    pub fn take_outbox(&self) -> Vec<u8> {
        std::mem::take(&mut *self.outbox.lock().expect("outbox is not poisoned"))
    }

    /// Whether bytes are waiting to be written.
    pub fn has_pending_output(&self) -> bool {
        !self
            .outbox
            .lock()
            .expect("outbox is not poisoned")
            .is_empty()
    }

    /// Mark the connection as gone and drop anything still queued.
    ///
    /// Afterwards [`PushTarget::is_closed`] is true, so pushes stop and every
    /// registration holding this connection can be cleaned up.
    pub fn close(&self) {
        self.closed.store(true, Ordering::SeqCst);
        self.outbox.lock().expect("outbox is not poisoned").clear();
    }

    /// Append one framed message to the outbox.
    fn queue(&self, message: &Value) {
        if self.is_closed() {
            return;
        }
        self.outbox
            .lock()
            .expect("outbox is not poisoned")
            .extend_from_slice(&encode(message));
    }
}

impl PushTarget for ClientConnection {
    fn is_closed(&self) -> bool {
        self.closed.load(Ordering::SeqCst)
    }

    fn push(&self, message: Value) {
        self.queue(&message);
    }
}

/// Log a dropped message. The body is truncated: it is client-supplied and may
/// be arbitrarily large.
fn report_malformed(error: &MalformedRequest, body: &[u8]) {
    const LIMIT: usize = 80;
    let body = String::from_utf8_lossy(body);
    let body = if body.len() > LIMIT {
        format!("{}...", &body[..LIMIT])
    } else {
        body.to_string()
    };
    warn!("api: dropping malformed request ({error}): {body}");
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::protocol::{ApiError, DELIMITER};
    use crate::core::klippy::api::registry::{Api, Endpoint, EndpointContext};
    use serde_json::{json, Value};

    /// An endpoint that echoes what it was asked, for round-trip tests.
    struct Echo;

    impl Endpoint for Echo {
        fn path(&self) -> &'static str {
            "echo"
        }

        fn handle(
            &self,
            request: &Request,
            _context: &EndpointContext<'_>,
        ) -> Result<Value, ApiError> {
            Ok(json!({ "method": request.method() }))
        }
    }

    struct Failing;

    impl Endpoint for Failing {
        fn path(&self) -> &'static str {
            "failing"
        }

        fn handle(
            &self,
            _request: &Request,
            _context: &EndpointContext<'_>,
        ) -> Result<Value, ApiError> {
            Err(ApiError::Internal("boom".to_string()))
        }
    }

    fn api() -> Arc<Api> {
        let mut api = Api::new();
        api.register(Echo).unwrap();
        api.register(Failing).unwrap();
        Arc::new(api)
    }

    /// Decode an outbox into the JSON messages it holds.
    fn replies(bytes: &[u8]) -> Vec<Value> {
        let mut framing = Framing::new();
        framing
            .push(bytes)
            .into_iter()
            .map(|body| serde_json::from_slice(&body).expect("a queued reply is JSON"))
            .collect()
    }

    #[test]
    fn test_a_request_produces_a_reply() {
        let connection = ClientConnection::new(api());
        assert_eq!(connection.receive(b"{\"id\":1,\"method\":\"echo\"}\x03"), 0);

        assert_eq!(
            replies(&connection.take_outbox()),
            vec![json!({
                "id": 1,
                "result": {"method": "echo"}
            })]
        );
        assert!(!connection.has_pending_output());
    }

    #[test]
    fn test_an_unknown_method_produces_an_error_reply() {
        let connection = ClientConnection::new(api());
        connection.receive(b"{\"id\":1,\"method\":\"nope\"}\x03");

        assert_eq!(
            replies(&connection.take_outbox()),
            vec![json!({
                "id": 1,
                "error": {
                    "error": "WebRequestError",
                    "message": "webhooks: No registered callback for path 'nope'"
                }
            })]
        );
    }

    #[test]
    fn test_a_handler_failure_produces_an_error_reply() {
        let connection = ClientConnection::new(api());
        connection.receive(b"{\"id\":\"a\",\"method\":\"failing\"}\x03");

        let replies = replies(&connection.take_outbox());
        assert_eq!(replies.len(), 1);
        assert_eq!(replies[0]["id"], json!("a"));
        assert_eq!(replies[0]["error"]["message"], json!("boom"));
    }

    #[test]
    fn test_requests_without_an_id_are_dispatched_but_not_answered() {
        let connection = ClientConnection::new(api());
        assert_eq!(connection.receive(b"{\"method\":\"echo\"}\x03"), 0);
        assert!(!connection.has_pending_output());
    }

    #[test]
    fn test_several_messages_in_one_read_are_answered_in_order() {
        let connection = ClientConnection::new(api());
        connection.receive(b"{\"id\":1,\"method\":\"echo\"}\x03{\"id\":2,\"method\":\"echo\"}\x03");

        let ids: Vec<Value> = replies(&connection.take_outbox())
            .into_iter()
            .map(|reply| reply["id"].clone())
            .collect();
        assert_eq!(ids, vec![json!(1), json!(2)]);
    }

    #[test]
    fn test_a_message_split_across_reads_is_answered_once() {
        let connection = ClientConnection::new(api());
        assert_eq!(connection.receive(b"{\"id\":1,\"met"), 0);
        assert!(!connection.has_pending_output());
        assert_eq!(connection.receive(b"hod\":\"echo\"}\x03"), 0);

        assert_eq!(replies(&connection.take_outbox()).len(), 1);
    }

    #[test]
    fn test_a_malformed_message_is_dropped_without_closing_the_connection() {
        let connection = ClientConnection::new(api());
        // The bad message is counted and skipped; the good one still answers.
        assert_eq!(
            connection.receive(b"not json\x03{\"id\":1,\"method\":\"echo\"}\x03"),
            1
        );

        let replies = replies(&connection.take_outbox());
        assert_eq!(replies.len(), 1);
        assert_eq!(replies[0]["id"], json!(1));
        assert!(!connection.is_closed());
    }

    #[test]
    fn test_pushes_are_queued_like_replies() {
        let connection = ClientConnection::new(api());
        connection.push(json!({"method": "printer:status", "params": {}}));

        assert!(connection.has_pending_output());
        assert_eq!(
            replies(&connection.take_outbox()),
            vec![json!({
                "method": "printer:status",
                "params": {}
            })]
        );
    }

    #[test]
    fn test_a_closed_connection_takes_no_more_output() {
        let connection = ClientConnection::new(api());
        connection.receive(b"{\"id\":1,\"method\":\"echo\"}\x03");
        assert!(!connection.take_outbox().is_empty());

        connection.close();
        assert!(connection.is_closed());
        // Replies and pushes are both dropped once the client is gone, so a
        // subscription cannot resurrect a closed connection.
        connection.receive(b"{\"id\":2,\"method\":\"echo\"}\x03");
        connection.push(json!({"method": "printer:status"}));
        assert!(!connection.has_pending_output());
    }

    #[test]
    fn test_queued_bytes_are_delimiter_terminated() {
        let connection = ClientConnection::new(api());
        connection.receive(b"{\"id\":1,\"method\":\"echo\"}\x03");

        let bytes = connection.take_outbox();
        assert_eq!(bytes.last(), Some(&DELIMITER));
        assert!(!bytes.contains(&b'\n'));
    }
}
