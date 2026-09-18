//! A connection to an API server: framing, request ids, and reply routing.
//!
//! This is the client half of the protocol in
//! [`api::protocol`](crate::core::klippy::api::protocol). It reuses that
//! module's framing and request shapes rather than re-deriving them — the
//! delimiter, the `id` echo and the `error` object are the same facts on both
//! sides, and a client that disagreed about them would be testing nothing.
//!
//! # What a connection tracks
//!
//! Only two things, and both exist so the user of a console can tell what they
//! are looking at:
//!
//! * **the next `id`** — so a caller does not have to invent one, and so every
//!   reply can be matched to the request that caused it;
//! * **the method each outstanding `id` belongs to** — so a reply can be
//!   labelled `2 (objects/query)` instead of just `2`.
//!
//! # Pushes
//!
//! A message without an `id` is a push: the server answers nothing, so there is
//! nothing to match it against. [`Incoming`] keeps those separate from replies
//! rather than pretending everything is a reply with some missing fields.
//! Note that `"id": null` counts as *no* id — that is how the protocol says
//! "fire and forget", and how a pushed message is written.

use std::collections::HashMap;
use std::collections::VecDeque;

use serde_json::{json, Map, Value};
use tokio::io::{AsyncReadExt, AsyncWriteExt, ReadHalf, WriteHalf};

use crate::core::klippy::api::address::{ApiTarget, Transport};
use crate::core::klippy::api::protocol::{encode, Framing};
use crate::core::klippy::error::KlippyError;

/// Bytes asked of the socket per read.
const READ_SIZE: usize = 4096;

/// What the server sent.
#[derive(Debug, Clone, PartialEq)]
pub enum Incoming {
    /// A reply, carrying the `id` of the request it answers.
    Reply(Reply),
    /// A message without an `id`: something the server sent on its own, such as
    /// a subscription update.
    Push(Value),
}

impl Incoming {
    /// The reply, if this is one.
    pub fn reply(&self) -> Option<&Reply> {
        match self {
            Incoming::Reply(reply) => Some(reply),
            Incoming::Push(_) => None,
        }
    }
}

/// A reply to a request.
#[derive(Debug, Clone, PartialEq)]
pub struct Reply {
    /// The `id`, echoed from the request.
    pub id: Value,
    /// The method that was sent with this `id`, when this connection sent it.
    pub method: Option<String>,
    /// The whole reply object, as received.
    pub message: Value,
}

impl Reply {
    /// The `result` payload, when the request succeeded.
    pub fn result(&self) -> Option<&Value> {
        self.message.get("result")
    }

    /// The `error` object, when the request failed.
    pub fn error(&self) -> Option<&Value> {
        self.message.get("error")
    }

    /// The error's message, when the request failed.
    pub fn error_message(&self) -> Option<&str> {
        self.error()?.get("message")?.as_str()
    }

    /// Whether the request failed.
    pub fn is_error(&self) -> bool {
        self.message.get("error").is_some()
    }

    /// What to show for this reply: the `result`, or the error object, or the
    /// message as received when it is neither (which the protocol does not
    /// produce, but a future or foreign server might).
    pub fn payload(&self) -> &Value {
        self.result()
            .or_else(|| self.error())
            .unwrap_or(&self.message)
    }
}

/// A live connection to an API server.
///
/// Reading and writing are separate halves, so a caller can wait for the next
/// message and send a new request from the same `select!` — which is exactly
/// what an interactive console does.
pub struct Connection {
    reader: ReadHalf<Box<dyn Transport>>,
    writer: WriteHalf<Box<dyn Transport>>,
    framing: Framing,
    /// Messages already read from the socket but not yet handed out. One read
    /// can complete several messages, and a push can arrive while the caller is
    /// waiting for a reply.
    queued: VecDeque<Value>,
    buffer: Vec<u8>,
    next_id: u64,
    /// Method name per outstanding `id`, keyed by the rendered id.
    pending: HashMap<String, String>,
}

impl Connection {
    /// Dial `target`.
    ///
    /// # Errors
    /// Returns [`KlippyError::Connection`] if the socket cannot be reached.
    pub async fn connect(target: &ApiTarget) -> Result<Self, KlippyError> {
        let (reader, writer) = tokio::io::split(target.connect().await?);
        Ok(Self {
            reader,
            writer,
            framing: Framing::new(),
            queued: VecDeque::new(),
            buffer: vec![0u8; READ_SIZE],
            next_id: 1,
            pending: HashMap::new(),
        })
    }

    /// Take the `id` for a new request, advancing the counter.
    ///
    /// Ids start at 1 and only go up, so a reply's id is also a rough count of
    /// how many requests this session has made.
    pub fn take_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    /// Whether a reply is still owed.
    ///
    /// True from the moment a request is sent until its reply has been handed
    /// out, and false for fire-and-forget requests, which by definition never
    /// get one. A caller that is about to stop reading — an interactive console
    /// on its way out — can use this to decide whether waiting is worthwhile.
    pub fn has_pending(&self) -> bool {
        !self.pending.is_empty()
    }

    /// The last `id` issued, or 0 before the first request.
    ///
    /// A reply carries this id, so a caller that did not keep the return value
    /// — an interactive console handing a line to a helper — can still say
    /// which request it is waiting for.
    pub fn last_id(&self) -> u64 {
        self.next_id.saturating_sub(1)
    }

    /// Build and send a request with a fresh `id`.
    ///
    /// # Errors
    /// Returns [`KlippyError::Connection`] if the request cannot be written.
    pub async fn request(
        &mut self,
        method: &str,
        params: Map<String, Value>,
    ) -> Result<u64, KlippyError> {
        let id = self.take_id();
        let message = json!({ "id": id, "method": method, "params": params });
        self.send(&message).await?;
        Ok(id)
    }

    /// Send an already-built message, verbatim.
    ///
    /// A message that will be answered has its `id` remembered so the reply can
    /// be labelled; a message with no `id` (or `"id": null`) is fire-and-forget
    /// and gets no reply at all, which is the protocol's rule and not this
    /// client's choice.
    ///
    /// # Errors
    /// Returns [`KlippyError::Connection`] if the message cannot be written.
    pub async fn send(&mut self, message: &Value) -> Result<(), KlippyError> {
        if let Some(id) = answerable_id(message) {
            if let Some(method) = message.get("method").and_then(Value::as_str) {
                self.pending.insert(id.to_string(), method.to_string());
            }
        }

        self.writer
            .write_all(&encode(message))
            .await
            .map_err(|err| {
                KlippyError::Connection(format!("cannot send to the API server: {err}"))
            })?;
        self.writer.flush().await.map_err(|err| {
            KlippyError::Connection(format!("cannot flush to the API server: {err}"))
        })?;
        Ok(())
    }

    /// The next message from the server.
    ///
    /// Cancel-safe: the only wait is on the socket read, and a cancelled read
    /// consumes nothing, so a caller may race this against other work (an
    /// interactive console races it against stdin).
    ///
    /// A message that is not JSON is dropped with a log line and the next one is
    /// returned, the way the server drops a malformed request: one bad message
    /// must not end a session.
    ///
    /// # Errors
    /// Returns [`KlippyError::Connection`] when the server closes the
    /// connection, which is how it reports a shutdown.
    pub async fn receive(&mut self) -> Result<Incoming, KlippyError> {
        loop {
            if let Some(message) = self.queued.pop_front() {
                return Ok(self.classify(message));
            }

            // Two disjoint fields, borrowed at once on purpose: the buffer must
            // stay where it is. Taking it out would leave it empty, and a read
            // cancelled by the caller's `select!` — which is how the console
            // races the socket against stdin — would then leave the next read
            // with a zero-length buffer, which reads as a closed connection.
            let read = self.reader.read(&mut self.buffer[..]).await;
            let read = match read {
                Ok(0) => {
                    return Err(KlippyError::Connection(
                        "the API server closed the connection".to_string(),
                    ));
                }
                Ok(read) => read,
                Err(err) => {
                    return Err(KlippyError::Connection(format!(
                        "cannot read from the API server: {err}"
                    )));
                }
            };

            let bodies = self.framing.push(&self.buffer[..read]);
            for body in bodies {
                match serde_json::from_slice(&body) {
                    Ok(message) => self.queued.push_back(message),
                    Err(err) => tracing::warn!(
                        "dropping a message that is not JSON: {err}: {}",
                        String::from_utf8_lossy(&body)
                    ),
                }
            }
        }
    }

    /// Turn a received message into a reply or a push.
    fn classify(&mut self, message: Value) -> Incoming {
        let Some(id) = answerable_id(&message).cloned() else {
            return Incoming::Push(message);
        };
        let method = self.pending.remove(&id.to_string());
        Incoming::Reply(Reply {
            id,
            method,
            message,
        })
    }
}

/// The `id` that will be answered, if the message expects a reply at all.
///
/// Both a missing `id` and `"id": null` mean nobody is waiting, which is the
/// protocol's "fire and forget" and also how pushed messages are written.
fn answerable_id(message: &Value) -> Option<&Value> {
    message.get("id").filter(|id| !id.is_null())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::protocol::{ApiError, Request};
    use crate::core::klippy::api::registry::{Api, Endpoint, EndpointContext};
    use crate::core::klippy::api::server::Server;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;
    use std::time::Duration;

    /// An endpoint that echoes its parameters, so a round trip is visible.
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
            Ok(json!({ "params": request.params().get_or("value", &Value::Null).clone() }))
        }
    }

    /// An endpoint that fails, for the error path.
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
            Err(ApiError::InvalidArgument)
        }
    }

    /// An endpoint that pushes once, from another task, to exercise the push
    /// path against a real socket.
    struct PushLater;

    impl Endpoint for PushLater {
        fn path(&self) -> &'static str {
            "push_later"
        }

        fn handle(
            &self,
            _request: &Request,
            context: &EndpointContext<'_>,
        ) -> Result<Value, ApiError> {
            let client = Arc::clone(&context.client);
            tokio::spawn(async move {
                tokio::time::sleep(Duration::from_millis(10)).await;
                client.push(json!({"method": "klippy:status", "params": {"n": 1}}));
            });
            Ok(json!({ "ok": true }))
        }
    }

    /// A socket path in a directory that removes itself.
    struct SocketPath(std::path::PathBuf);

    impl SocketPath {
        fn new(name: &str) -> Self {
            static COUNTER: AtomicUsize = AtomicUsize::new(0);
            let dir = std::env::temp_dir().join(format!(
                "klipperx-client-{}-{}-{}",
                std::process::id(),
                name,
                COUNTER.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir_all(&dir).expect("cannot create the test directory");
            Self(dir.join("klippy_uds"))
        }

        fn target(&self) -> ApiTarget {
            ApiTarget::Unix(self.0.clone())
        }
    }

    impl Drop for SocketPath {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(self.0.parent().expect("the path has a parent"));
        }
    }

    /// Serve `api` on `path` until the test ends.
    async fn serve(path: &SocketPath) -> tokio::task::JoinHandle<()> {
        let mut api = Api::new();
        api.register(Echo).unwrap();
        api.register(Failing).unwrap();
        api.register(PushLater).unwrap();
        let server = Server::bind(path.target(), Arc::new(api))
            .await
            .expect("cannot bind");
        tokio::spawn(async move {
            let _ = server.run().await;
        })
    }

    /// Connect, or fail the test.
    async fn connect(path: &SocketPath) -> Connection {
        Connection::connect(&path.target())
            .await
            .expect("cannot connect")
    }

    #[tokio::test]
    async fn test_a_request_is_answered_and_labelled() {
        let path = SocketPath::new("roundtrip");
        let task = serve(&path).await;
        let mut connection = connect(&path).await;

        let mut params = Map::new();
        params.insert("value".to_string(), json!(7));
        let id = connection.request("echo", params).await.unwrap();

        let reply = match connection.receive().await.unwrap() {
            Incoming::Reply(reply) => reply,
            other => panic!("expected a reply, got {other:?}"),
        };
        assert_eq!(reply.id, json!(id));
        // The id is labelled with the method that asked for it.
        assert_eq!(reply.method.as_deref(), Some("echo"));
        assert_eq!(reply.result(), Some(&json!({"params": 7})));
        assert!(!reply.is_error());

        // The id is forgotten once its reply has been handed out.
        assert!(connection.pending.is_empty());

        task.abort();
    }

    #[tokio::test]
    async fn test_an_error_reply_is_a_reply_not_a_failure() {
        let path = SocketPath::new("error");
        let task = serve(&path).await;
        let mut connection = connect(&path).await;

        let id = connection.request("failing", Map::new()).await.unwrap();
        let reply = match connection.receive().await.unwrap() {
            Incoming::Reply(reply) => reply,
            other => panic!("expected a reply, got {other:?}"),
        };

        assert_eq!(reply.id, json!(id));
        assert!(reply.is_error());
        assert_eq!(reply.error_message(), Some("Invalid argument"));
        assert_eq!(reply.result(), None);
        // What a console shows for a failed request is the error object.
        assert_eq!(
            reply.payload(),
            &json!({
                "error": "WebRequestError",
                "message": "Invalid argument"
            })
        );

        task.abort();
    }

    #[tokio::test]
    async fn test_a_message_without_an_id_is_a_push() {
        let path = SocketPath::new("push");
        let task = serve(&path).await;
        let mut connection = connect(&path).await;

        connection.request("push_later", Map::new()).await.unwrap();

        // The reply comes first, then the push: one read, or two, but the
        // order is the server's.
        let mut saw_push = false;
        for _ in 0..2 {
            match connection.receive().await.unwrap() {
                Incoming::Reply(reply) => assert_eq!(reply.method.as_deref(), Some("push_later")),
                Incoming::Push(message) => {
                    assert_eq!(message["method"], json!("klippy:status"));
                    saw_push = true;
                }
            }
        }
        assert!(saw_push, "the push never arrived");

        task.abort();
    }

    #[tokio::test]
    async fn test_fire_and_forget_is_sent_but_not_awaited() {
        let path = SocketPath::new("fire");
        let task = serve(&path).await;
        let mut connection = connect(&path).await;

        // An explicit `"id": null` is the protocol's way of saying nobody is
        // waiting; it must not be remembered as outstanding.
        connection
            .send(&json!({"id": null, "method": "echo", "params": {}}))
            .await
            .unwrap();
        assert!(connection.pending.is_empty());
        assert!(answerable_id(&json!({"method": "echo"})).is_none());
        assert!(answerable_id(&json!({"id": null})).is_none());
        assert!(answerable_id(&json!({"id": 0})).is_some());

        // Nothing comes back, so a read has to time out rather than end.
        let quiet = tokio::time::timeout(Duration::from_millis(100), connection.receive()).await;
        assert!(quiet.is_err(), "a fire-and-forget request was answered");

        task.abort();
    }

    #[tokio::test]
    async fn test_a_reply_is_owed_only_until_it_arrives() {
        let path = SocketPath::new("pending");
        let task = serve(&path).await;
        let mut connection = connect(&path).await;

        assert!(!connection.has_pending());
        let id = connection.request("echo", Map::new()).await.unwrap();
        assert!(connection.has_pending(), "a request owes a reply");

        let _ = connection.receive().await.unwrap();
        assert!(!connection.has_pending(), "id {id} was answered");

        // Fire-and-forget never owes anything.
        connection
            .send(&json!({"id": null, "method": "echo"}))
            .await
            .unwrap();
        assert!(!connection.has_pending());

        task.abort();
    }

    #[tokio::test]
    async fn test_a_cancelled_read_leaves_the_connection_usable() {
        let path = SocketPath::new("cancel");
        let task = serve(&path).await;
        let mut connection = connect(&path).await;

        // Wait for nothing, and lose: this is the race the console runs on every
        // line of input it reads.
        tokio::select! {
            _ = tokio::time::sleep(Duration::from_millis(50)) => {}
            _ = connection.receive() => panic!("nothing had been sent yet"),
        }

        // The connection must be untouched, buffer included.
        let id = connection.request("echo", Map::new()).await.unwrap();
        let reply = match connection.receive().await.unwrap() {
            Incoming::Reply(reply) => reply,
            other => panic!("expected a reply, got {other:?}"),
        };
        assert_eq!(reply.id, json!(id));

        task.abort();
    }

    #[tokio::test]
    async fn test_a_closed_connection_is_reported_as_such() {
        let path = SocketPath::new("closed");
        // A listener that accepts and immediately hangs up: the client has to
        // report that rather than wait forever.
        let listener = tokio::net::UnixListener::bind(&path.0).expect("cannot bind");
        let mut connection = connect(&path).await;
        let (stream, _) = listener.accept().await.expect("nothing connected");
        drop(stream);

        let error = connection
            .receive()
            .await
            .expect_err("a hung-up connection must be an error");
        assert!(matches!(error, KlippyError::Connection(_)), "{error:?}");
        assert!(
            error.to_string().contains("closed the connection"),
            "{error}"
        );
    }

    /// Connect where nothing is listening, and return the error.
    async fn fail(target: ApiTarget) -> KlippyError {
        match target.connect().await {
            Ok(_) => panic!("connecting to nothing must fail"),
            Err(error) => error,
        }
    }

    #[tokio::test]
    async fn test_a_bad_target_reports_where_it_tried_to_connect() {
        // `Box<dyn Transport>` is not `Debug`, so the failure is taken apart by
        // hand rather than with `expect_err`.
        let error = fail(ApiTarget::Unix("/nonexistent/klipperx/nope".into())).await;
        assert!(
            error.to_string().contains("/nonexistent/klipperx/nope"),
            "{error}"
        );

        let error = fail(ApiTarget::Tcp("127.0.0.1:1".to_string())).await;
        assert!(error.to_string().contains("127.0.0.1:1"), "{error}");
    }

    #[tokio::test]
    async fn test_several_messages_in_one_read_are_all_handed_out() {
        let path = SocketPath::new("batch");
        let task = serve(&path).await;
        let mut connection = connect(&path).await;

        // Two requests, without reading in between: the server's replies may
        // well arrive together, and both must still be delivered.
        let first = connection.request("echo", Map::new()).await.unwrap();
        let second = connection.request("echo", Map::new()).await.unwrap();

        let mut ids = Vec::new();
        for _ in 0..2 {
            let reply = match connection.receive().await.unwrap() {
                Incoming::Reply(reply) => reply,
                other => panic!("expected a reply, got {other:?}"),
            };
            ids.push(reply.id);
        }
        assert_eq!(ids, vec![json!(first), json!(second)]);

        task.abort();
    }

    #[tokio::test]
    async fn test_the_client_speaks_tcp_too() {
        let mut api = Api::new();
        api.register(Echo).unwrap();
        let server = Server::bind(ApiTarget::Tcp("127.0.0.1:0".to_string()), Arc::new(api))
            .await
            .unwrap();
        let target = ApiTarget::Tcp(server.local_addr().unwrap().to_string());
        let task = tokio::spawn(server.run());

        let mut connection = Connection::connect(&target).await.unwrap();
        let id = connection.request("echo", Map::new()).await.unwrap();
        let reply = match connection.receive().await.unwrap() {
            Incoming::Reply(reply) => reply,
            other => panic!("expected a reply, got {other:?}"),
        };
        assert_eq!(reply.id, json!(id));

        task.abort();
    }
}
