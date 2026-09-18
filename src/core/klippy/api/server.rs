//! The listening socket, and the task that serves each connection.
//!
//! klippy is started with `-a <target>` and the API server listens there: a
//! Unix Domain Socket by default, or TCP when the target says so (see
//! [`ApiTarget`]). One task accepts connections, and each connection gets a
//! task of its own.
//!
//! # Why a task per connection
//!
//! Upstream runs one thread and one reactor: every connection is a file
//! descriptor with read and write callbacks, and a handler that has to wait for
//! another module suspends its greenlet rather than the thread. Here the same
//! shape is expressed with tasks — a blocked connection parks only itself, and
//! refusing to park is what a `todo!()` endpoint would do. So several clients
//! are served concurrently, while the requests *on one connection* stay
//! ordered, which is what a client that pipelines expects.
//!
//! # Writing, and the push wakeup
//!
//! A connection task has two reasons to write: a request it just answered, and
//! something pushed to it by an endpoint it subscribed to earlier. The second
//! one can come from any task or thread, so [`ClientConnection::push`] only
//! queues bytes and wakes the connection's own task through a `Notify` — the
//! task is the only thing that ever touches the socket, which is why no lock
//! protects the write half.
//!
//! Upstream drops a client that has not accepted a write for five seconds; the
//! same limit lives in `WRITE_TIMEOUT`, for the same reason: a client that
//! stopped reading must not be allowed to grow the outbox until the host runs
//! out of memory.
//!
//! # Not covered
//!
//! * No TLS, and no authentication: whoever can connect can drive the printer,
//!   exactly as with upstream's socket. A TCP listener therefore belongs on a
//!   trusted network only.
//! * Cleanup after an abrupt exit. `SIGTERM` and a panic end the process without
//!   unwinding, so the socket file stays behind; the next start removes it
//!   before binding, which is part of why binding removes it at all.
//! * Upstream's per-client request log (dumped on an analysed shutdown) is not
//!   kept.

use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use serde_json::Value;
use tokio::io::{AsyncReadExt, AsyncWrite, AsyncWriteExt};
use tokio::net::{TcpListener, UnixListener};
use tokio::sync::Notify;
use tokio::time::{timeout, Duration};
use tracing::warn;

use crate::core::klippy::error::KlippyError;

use super::address::{ApiTarget, Transport};
use super::protocol::{encode, Framing, MalformedRequest, PushTarget, Request};
use super::registry::Api;

/// Bytes asked of the socket per read.
const READ_SIZE: usize = 4096;

/// How long one write may make no progress before the client is dropped.
///
/// Upstream counts the same five seconds while its write callback stays
/// unwakeable.
const WRITE_TIMEOUT: Duration = Duration::from_secs(5);

// ===========================================================================
// Listener
// ===========================================================================

/// A bound listening socket.
enum Listener {
    Unix {
        listener: UnixListener,
        path: PathBuf,
    },
    Tcp(TcpListener),
}

impl Listener {
    /// Bind `target`, creating the socket file for a Unix target.
    async fn bind(target: &ApiTarget) -> Result<Self, KlippyError> {
        match target {
            ApiTarget::Unix(path) => {
                // A socket file left by a killed run would make this fail with
                // "address already in use", so it goes first — upstream removes
                // it for the same reason. Removing it while a live server holds
                // it is the operator's mistake, and the bind below then fails
                // loudly enough.
                match std::fs::remove_file(path) {
                    Ok(()) => (),
                    Err(err) if err.kind() == std::io::ErrorKind::NotFound => (),
                    Err(err) => {
                        return Err(KlippyError::Connection(format!(
                            "cannot remove stale socket {}: {err}",
                            path.display()
                        )))
                    }
                }
                let listener = UnixListener::bind(path).map_err(|err| {
                    KlippyError::Connection(format!(
                        "cannot bind unix socket {}: {err}",
                        path.display()
                    ))
                })?;
                Ok(Listener::Unix {
                    listener,
                    path: path.clone(),
                })
            }
            ApiTarget::Tcp(address) => {
                let listener = TcpListener::bind(address).await.map_err(|err| {
                    KlippyError::Connection(format!("cannot bind tcp {address}: {err}"))
                })?;
                Ok(Listener::Tcp(listener))
            }
        }
    }

    async fn accept(&self) -> std::io::Result<Box<dyn Transport>> {
        match self {
            Listener::Unix { listener, .. } => {
                let (stream, _) = listener.accept().await?;
                Ok(Box::new(stream))
            }
            Listener::Tcp(listener) => {
                let (stream, _) = listener.accept().await?;
                // Small replies, sent as soon as they exist: Nagle only adds
                // latency to the acknowledgment of the previous request.
                let _ = stream.set_nodelay(true);
                Ok(Box::new(stream))
            }
        }
    }

    /// The socket path, for a Unix listener.
    fn socket_path(&self) -> Option<&Path> {
        match self {
            Listener::Unix { path, .. } => Some(path),
            Listener::Tcp(_) => None,
        }
    }

    /// The bound address, for a TCP listener.
    ///
    /// Resolved, so a target of `127.0.0.1:0` reports the port the kernel
    /// actually chose.
    fn local_addr(&self) -> Option<SocketAddr> {
        match self {
            Listener::Unix { .. } => None,
            Listener::Tcp(listener) => listener.local_addr().ok(),
        }
    }

    /// `requested`, with a TCP port of `0` replaced by the bound one.
    fn resolved_target(&self, requested: &ApiTarget) -> ApiTarget {
        match (requested, self.local_addr()) {
            (ApiTarget::Tcp(_), Some(addr)) => ApiTarget::Tcp(addr.to_string()),
            _ => requested.clone(),
        }
    }
}

impl Drop for Listener {
    /// Remove the socket file, so a clean exit leaves nothing to trip over.
    ///
    /// Dropping is where this belongs rather than the end of
    /// [`Server::run`]: that future is normally cancelled, and a cancelled
    /// future never reaches its own cleanup.
    fn drop(&mut self) {
        if let Listener::Unix { path, .. } = self {
            let _ = std::fs::remove_file(path);
        }
    }
}

// ===========================================================================
// Server
// ===========================================================================

/// The API server: a bound listener and the endpoint table it serves.
pub struct Server {
    target: ApiTarget,
    listener: Listener,
    api: Arc<Api>,
}

impl Server {
    /// Bind `target` and hold it open, without accepting anything yet.
    ///
    /// Binding is separate from [`Server::run`] so that a caller can report the
    /// address it got — which is the only way to learn the port of a TCP target
    /// like `127.0.0.1:0` — and so that a bind failure is reported where the
    /// operator is looking, before any serving starts.
    ///
    /// # Errors
    /// Returns [`KlippyError::Connection`] if the listener cannot be created.
    pub async fn bind(target: ApiTarget, api: Arc<Api>) -> Result<Self, KlippyError> {
        let listener = Listener::bind(&target).await?;
        // Resolve once, here, so that `target` means "where this is listening"
        // everywhere else instead of "what the operator typed".
        let target = listener.resolved_target(&target);
        Ok(Self {
            target,
            listener,
            api,
        })
    }

    /// The target this server is bound to.
    ///
    /// Not necessarily the one asked for: a TCP port of `0` is replaced by the
    /// port the kernel chose, so this is always usable as an address.
    pub fn target(&self) -> &ApiTarget {
        &self.target
    }

    /// The socket path, if this server listens on a Unix Domain Socket.
    pub fn socket_path(&self) -> Option<&Path> {
        self.listener.socket_path()
    }

    /// The bound address, if this server listens on TCP.
    pub fn local_addr(&self) -> Option<SocketAddr> {
        self.listener.local_addr()
    }

    /// The endpoint table this server serves.
    pub fn api(&self) -> &Arc<Api> {
        &self.api
    }

    /// Accept connections until the printer shuts down.
    ///
    /// The future never completes on its own: it ends when the printer shuts
    /// down and drops it, or when the task running it is aborted. Either way
    /// the listener is dropped, which for a Unix target removes the socket
    /// file, and every connection task ends when its client goes away.
    ///
    /// The caller reports the bound address — [`Server::target`] — because that
    /// is where it knows whether the operator asked for one.
    ///
    /// # Errors
    /// Accept failures are logged and retried — a client that vanishes between
    /// the connection and the accept must not take the server down with it.
    /// Only dropping the server stops it, so this returns `Ok` only if it is
    /// ever given a way to stop.
    pub async fn run(self) -> Result<(), KlippyError> {
        loop {
            let stream = match self.listener.accept().await {
                Ok(stream) => stream,
                Err(err) => {
                    warn!("API server accept failed: {err}");
                    continue;
                }
            };
            let connection = ClientConnection::new(Arc::clone(&self.api));
            tokio::spawn(serve(connection, stream));
        }
    }
}

/// Serve one connection until the client goes away.
///
/// Both reasons to write are waited on at once, so a push is delivered as soon
/// as it is queued even while the client sends nothing, and a request is
/// answered as soon as it arrives even while nothing is being pushed.
async fn serve(connection: Arc<ClientConnection>, stream: Box<dyn Transport>) {
    let (mut reader, mut writer) = tokio::io::split(stream);
    let mut buf = vec![0u8; READ_SIZE];

    loop {
        // Register for a wakeup *before* waiting on either branch. A `Notified`
        // starts out disabled, so a `push` landing between the select decision
        // and the next iteration would otherwise be lost, leaving the push
        // queued until the client happened to send something.
        let notified = connection.wait_for_output();
        tokio::pin!(notified);
        notified.as_mut().enable();

        tokio::select! {
            read = reader.read(&mut buf) => match read {
                // EOF or a broken socket: the client is gone, and so is
                // anything still queued for it.
                Ok(0) | Err(_) => break,
                Ok(count) => {
                    connection.receive(&buf[..count]);
                }
            },
            () = &mut notified => {}
        }

        if !connection.flush(&mut writer).await {
            break;
        }
    }

    connection.close();
}

// ===========================================================================
// ClientConnection
// ===========================================================================

/// One connected client.
///
/// A connection owns everything that is per-client: the framing state for bytes
/// that arrived in pieces, and the outbox of replies and pushes waiting to be
/// written. It is the [`PushTarget`] endpoints are handed, so a subscription
/// can be dropped the moment the client goes away — and so that a push from any
/// task or thread lands in one place.
///
/// Only the connection's own task touches the socket; everything else reaches it
/// through [`PushTarget::push`], which is why pushing needs no runtime handle
/// and can be called from a synchronous context.
pub struct ClientConnection {
    api: Arc<Api>,
    framing: Mutex<Framing>,
    outbox: Mutex<Vec<u8>>,
    /// Wakes the connection's task when the outbox gains bytes.
    output_ready: Notify,
    closed: AtomicBool,
}

impl ClientConnection {
    /// Create a connection that dispatches against `api`.
    pub fn new(api: Arc<Api>) -> Arc<Self> {
        Arc::new(Self {
            api,
            framing: Mutex::new(Framing::new()),
            outbox: Mutex::new(Vec::new()),
            output_ready: Notify::new(),
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
    ///
    /// Synchronous on purpose: everything here is a short critical section that
    /// never waits on the socket, so the caller can invoke it straight from the
    /// connection task without risking a lock held across an `await`.
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

            // The handler gets the connection as a `PushTarget`, so an endpoint
            // that subscribes can hold on to it and keep pushing later.
            let client: Arc<dyn PushTarget> = self.clone();
            let outcome = self.api.dispatch(&request, client);
            if let Some(response) = request.respond(outcome) {
                self.queue(&response);
            }
        }
        dropped
    }

    /// Wait until something is queued for writing.
    ///
    /// The connection task selects on this alongside the socket, so a push is
    /// written when it happens rather than when the client next speaks.
    ///
    /// Returns the notification future rather than `async fn` so that the
    /// caller can `enable()` it before selecting: a `Notified` starts out
    /// disabled, and a push landing in that window would be lost.
    #[allow(clippy::type_complexity)]
    pub fn wait_for_output(&self) -> tokio::sync::futures::Notified<'_> {
        self.output_ready.notified()
    }

    /// Write everything queued, returning false when the client is gone.
    ///
    /// A write that cannot complete within `WRITE_TIMEOUT` means the client
    /// stopped reading; the connection is then dropped, which is upstream's
    /// remedy for the same situation. Discarding the unsent bytes is safe
    /// because the caller closes the connection right after.
    ///
    /// Loops after a successful write: bytes pushed while this was writing are
    /// picked up before returning, so a burst of pushes costs one wakeup.
    pub async fn flush<W>(&self, writer: &mut W) -> bool
    where
        W: AsyncWrite + Unpin,
    {
        loop {
            let bytes = self.take_outbox();
            if bytes.is_empty() {
                return true;
            }
            match timeout(WRITE_TIMEOUT, writer.write_all(&bytes)).await {
                Ok(Ok(())) => continue,
                Ok(Err(_)) => return false,
                Err(_) => {
                    warn!("API client stopped reading; dropping the connection");
                    return false;
                }
            }
        }
    }

    /// Take everything queued for writing, leaving the outbox empty.
    ///
    /// What comes back is a contiguous run of whole messages, each already
    /// delimiter-terminated.
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

    /// Append one framed message to the outbox and wake the connection task.
    fn queue<T: serde::Serialize>(&self, message: &T) {
        if self.is_closed() {
            return;
        }
        self.outbox
            .lock()
            .expect("outbox is not poisoned")
            .extend_from_slice(&encode(message));
        self.output_ready.notify_one();
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
    use crate::core::klippy::api::registry::{Endpoint, EndpointContext};
    use serde_json::json;
    use std::io::Write as _;
    use std::pin::Pin;
    use std::sync::atomic::AtomicUsize;
    use std::task::{Context, Poll};
    use tokio::io::AsyncRead;
    use tokio::net::{TcpStream, UnixStream};

    // -----------------------------------------------------------------------
    // Test doubles
    // -----------------------------------------------------------------------

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
            Err(ApiError::Internal("boom".to_string()))
        }
    }

    /// An endpoint that answers, then pushes something later — a subscription
    /// in miniature, and the only way to prove a push wakes the connection.
    ///
    /// It holds the connection the way a real subscription would: a cloned
    /// `Arc`, kept past the end of the request.
    struct PushAfterReply(Duration);

    impl Endpoint for PushAfterReply {
        fn path(&self) -> &'static str {
            "push_later"
        }

        fn handle(
            &self,
            _request: &Request,
            context: &EndpointContext<'_>,
        ) -> Result<Value, ApiError> {
            let client = Arc::clone(&context.client);
            let delay = self.0;
            tokio::spawn(async move {
                tokio::time::sleep(delay).await;
                client.push(json!({
                    "method": "printer:status",
                    "params": {"pushed": true}
                }));
            });
            Ok(json!({ "registered": true }))
        }
    }

    /// A writer that never accepts anything, to exercise the write-stall policy
    /// without needing a client that stops reading.
    struct StalledWriter;

    impl AsyncWrite for StalledWriter {
        fn poll_write(
            self: Pin<&mut Self>,
            _cx: &mut Context<'_>,
            _buf: &[u8],
        ) -> Poll<std::io::Result<usize>> {
            Poll::Pending
        }

        fn poll_flush(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<std::io::Result<()>> {
            Poll::Ready(Ok(()))
        }

        fn poll_shutdown(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<std::io::Result<()>> {
            Poll::Ready(Ok(()))
        }
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    fn api() -> Arc<Api> {
        let mut api = Api::new();
        api.register(Echo).unwrap();
        api.register(Failing).unwrap();
        api.register(PushAfterReply(Duration::from_millis(20)))
            .unwrap();
        Arc::new(api)
    }

    /// A directory that removes itself, socket file and all.
    struct TempDir(PathBuf);

    impl TempDir {
        fn new(name: &str) -> Self {
            static COUNTER: AtomicUsize = AtomicUsize::new(0);
            let path = std::env::temp_dir().join(format!(
                "klipperx-api-{}-{}-{}",
                std::process::id(),
                name,
                COUNTER.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir_all(&path).expect("cannot create the test directory");
            Self(path)
        }

        fn join(&self, name: &str) -> PathBuf {
            self.0.join(name)
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// Start a server on `target`, failing the test if it cannot bind.
    async fn bind(target: ApiTarget) -> Server {
        Server::bind(target, api()).await.expect("cannot bind")
    }

    /// Whether `path` is a socket, as opposed to a leftover plain file.
    fn is_socket(path: &Path) -> bool {
        use std::os::unix::fs::FileTypeExt;
        std::fs::metadata(path)
            .expect("the socket file exists")
            .file_type()
            .is_socket()
    }

    /// Read `count` messages from a client socket.
    async fn recv<R>(reader: &mut R, count: usize) -> Vec<Value>
    where
        R: AsyncRead + Unpin,
    {
        let mut framing = Framing::new();
        let mut messages = Vec::new();
        let mut buf = vec![0u8; READ_SIZE];
        while messages.len() < count {
            // One read at a time, each with its own limit: a client that
            // dribbles bytes still makes progress, and a server that answers
            // nothing fails the test instead of hanging it.
            let read = timeout(Duration::from_secs(5), reader.read(&mut buf))
                .await
                .unwrap_or_else(|_| panic!("timed out with {} of {count} messages", messages.len()))
                .expect("read failed");
            assert!(
                read > 0,
                "connection closed with {} of {count}",
                messages.len()
            );
            for body in framing.push(&buf[..read]) {
                messages.push(serde_json::from_slice(&body).expect("message is JSON"));
            }
        }
        messages
    }

    async fn send(stream: &mut (impl AsyncWrite + Unpin), message: &str) {
        stream
            .write_all(message.as_bytes())
            .await
            .expect("write failed");
        stream.write_all(&[DELIMITER]).await.expect("write failed");
    }

    // -----------------------------------------------------------------------
    // Serving
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_a_request_is_answered_over_a_unix_socket() {
        let dir = TempDir::new("unix");
        let path = dir.join("klippy_uds");
        let server = bind(ApiTarget::Unix(path.clone())).await;
        assert_eq!(server.socket_path(), Some(path.as_path()));
        assert_eq!(server.local_addr(), None);
        let task = tokio::spawn(server.run());

        let mut stream = UnixStream::connect(&path).await.expect("cannot connect");
        send(&mut stream, r#"{"id":1,"method":"echo"}"#).await;
        let replies = recv(&mut stream, 1).await;

        assert_eq!(
            replies,
            vec![json!({
                "id": 1,
                "result": {"method": "echo"}
            })]
        );

        task.abort();
        let _ = task.await;
    }

    #[tokio::test]
    async fn test_a_request_is_answered_over_tcp() {
        // Port 0: the kernel picks, and the server reports what it chose.
        let server = bind(ApiTarget::Tcp("127.0.0.1:0".to_string())).await;
        let addr = server.local_addr().expect("a TCP server has an address");
        assert_eq!(server.socket_path(), None);
        // The reported target is the bound address, not `127.0.0.1:0`.
        assert_eq!(server.target().to_string(), format!("tcp:{addr}"));
        let task = tokio::spawn(server.run());

        let mut stream = TcpStream::connect(addr).await.expect("cannot connect");
        send(&mut stream, r#"{"id":"a","method":"echo"}"#).await;
        let replies = recv(&mut stream, 1).await;

        // The id comes back exactly as sent, string and all.
        assert_eq!(
            replies,
            vec![json!({
                "id": "a",
                "result": {"method": "echo"}
            })]
        );

        task.abort();
        let _ = task.await;
    }

    #[tokio::test]
    async fn test_an_unknown_method_and_a_failing_handler_both_answer() {
        let dir = TempDir::new("errors");
        let path = dir.join("klippy_uds");
        let server = bind(ApiTarget::Unix(path.clone())).await;
        let task = tokio::spawn(server.run());

        let mut stream = UnixStream::connect(&path).await.expect("cannot connect");
        // Two requests in one write, answered in order.
        send(
            &mut stream,
            "{\"id\":1,\"method\":\"nope\"}\x03{\"id\":2,\"method\":\"failing\"}",
        )
        .await;
        let replies = recv(&mut stream, 2).await;

        assert_eq!(replies[0]["error"]["error"], json!("WebRequestError"));
        assert_eq!(
            replies[0]["error"]["message"],
            json!("webhooks: No registered callback for path 'nope'")
        );
        assert_eq!(replies[1]["error"]["message"], json!("boom"));

        task.abort();
        let _ = task.await;
    }

    #[tokio::test]
    async fn test_a_relayed_push_arrives_without_further_requests() {
        let dir = TempDir::new("push");
        let path = dir.join("klippy_uds");
        let server = bind(ApiTarget::Unix(path.clone())).await;
        let task = tokio::spawn(server.run());

        let mut stream = UnixStream::connect(&path).await.expect("cannot connect");
        send(&mut stream, r#"{"id":1,"method":"push_later"}"#).await;
        // Nothing else is sent: the second message can only arrive because the
        // push woke the connection task.
        let messages = recv(&mut stream, 2).await;

        assert_eq!(messages[0]["result"], json!({"registered": true}));
        assert_eq!(
            messages[1],
            json!({
                "method": "printer:status",
                "params": {"pushed": true}
            })
        );

        task.abort();
        let _ = task.await;
    }

    #[tokio::test]
    async fn test_a_request_split_across_tcp_segments_is_answered_once() {
        let server = bind(ApiTarget::Tcp("127.0.0.1:0".to_string())).await;
        let addr = server.local_addr().unwrap();
        let task = tokio::spawn(server.run());

        let mut stream = TcpStream::connect(addr).await.expect("cannot connect");
        // Two writes for one message: framing has to hold the first half.
        stream.write_all(b"{\"id\":1,\"met").await.unwrap();
        stream.flush().await.unwrap();
        send(&mut stream, "hod\":\"echo\"}").await;

        assert_eq!(recv(&mut stream, 1).await.len(), 1);

        task.abort();
        let _ = task.await;
    }

    #[tokio::test]
    async fn test_a_stale_socket_file_is_replaced() {
        let dir = TempDir::new("stale");
        let path = dir.join("klippy_uds");
        // A file left behind by a killed run.
        std::fs::File::create(&path)
            .expect("cannot create the stale file")
            .write_all(b"stale")
            .unwrap();

        // The bind must replace the stale file with a real listening socket.
        let server = bind(ApiTarget::Unix(path.clone())).await;
        assert!(is_socket(&path));
        drop(server);
    }

    #[tokio::test]
    async fn test_dropping_the_server_removes_the_socket_file() {
        let dir = TempDir::new("cleanup");
        let path = dir.join("klippy_uds");
        let server = bind(ApiTarget::Unix(path.clone())).await;
        let task = tokio::spawn(server.run());
        assert!(path.exists());

        task.abort();
        let _ = task.await;

        assert!(
            !path.exists(),
            "a cleaned-up server must leave no socket file behind"
        );
    }

    #[tokio::test]
    async fn test_two_clients_are_served_at_the_same_time() {
        let dir = TempDir::new("clients");
        let path = dir.join("klippy_uds");
        let server = bind(ApiTarget::Unix(path.clone())).await;
        let task = tokio::spawn(server.run());

        let mut first = UnixStream::connect(&path).await.expect("cannot connect");
        let mut second = UnixStream::connect(&path).await.expect("cannot connect");
        // Interleaved on purpose: neither connection may block the other.
        send(&mut first, r#"{"id":"first","method":"echo"}"#).await;
        send(&mut second, r#"{"id":"second","method":"echo"}"#).await;

        assert_eq!(recv(&mut first, 1).await[0]["id"], json!("first"));
        assert_eq!(recv(&mut second, 1).await[0]["id"], json!("second"));

        task.abort();
        let _ = task.await;
    }

    // -----------------------------------------------------------------------
    // ClientConnection
    // -----------------------------------------------------------------------

    #[test]
    fn test_a_request_produces_a_reply() {
        let connection = ClientConnection::new(api());
        assert_eq!(connection.receive(b"{\"id\":1,\"method\":\"echo\"}\x03"), 0);

        let bytes = connection.take_outbox();
        assert_eq!(bytes.last(), Some(&DELIMITER));
        // `id` comes first on the wire, as the reference documents it. That
        // order only survives because replies skip the `Value` round-trip.
        assert!(
            bytes.starts_with(b"{\"id\":1,"),
            "unexpected reply: {}",
            String::from_utf8_lossy(&bytes)
        );
        assert_eq!(
            serde_json::from_slice::<Value>(&bytes[..bytes.len() - 1]).unwrap(),
            json!({"id": 1, "result": {"method": "echo"}})
        );
        assert!(!connection.has_pending_output());
    }

    #[test]
    fn test_no_id_means_no_reply_and_a_bad_body_means_no_crash() {
        let connection = ClientConnection::new(api());
        assert_eq!(connection.receive(b"{\"method\":\"echo\"}\x03"), 0);
        assert!(!connection.has_pending_output());

        // A malformed body is counted and skipped; the connection lives on.
        assert_eq!(
            connection.receive(b"not json\x03{\"id\":1,\"method\":\"echo\"}\x03"),
            1
        );
        assert_eq!(
            recv_bodies(&connection.take_outbox()).len(),
            1,
            "the good request must still be answered"
        );
    }

    #[tokio::test]
    async fn test_pushing_wakes_a_waiter() {
        let connection = ClientConnection::new(api());
        let pushed = tokio::spawn({
            let connection = Arc::clone(&connection);
            async move {
                tokio::time::sleep(Duration::from_millis(10)).await;
                connection.push(json!({"method": "printer:status"}));
            }
        });

        tokio::time::timeout(Duration::from_secs(5), connection.wait_for_output())
            .await
            .expect("a push must wake the connection task");
        pushed.await.unwrap();
        assert_eq!(
            recv_bodies(&connection.take_outbox()),
            vec![json!({
                "method": "printer:status"
            })]
        );
    }

    #[tokio::test(start_paused = true)]
    async fn test_a_client_that_stops_reading_is_cut_off() {
        let connection = ClientConnection::new(api());
        connection.push(json!({"method": "printer:status"}));

        // Paused time makes the five-second limit elapse at once.
        assert!(!connection.flush(&mut StalledWriter).await);
    }

    #[tokio::test]
    async fn test_a_closed_connection_takes_no_more_output() {
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

    #[tokio::test]
    async fn test_flushing_an_empty_outbox_succeeds() {
        let connection = ClientConnection::new(api());
        let mut writer = Vec::new();
        assert!(connection.flush(&mut writer).await);
        assert!(writer.is_empty());
    }

    /// Split an outbox into the JSON messages it holds.
    fn recv_bodies(bytes: &[u8]) -> Vec<Value> {
        let mut framing = Framing::new();
        framing
            .push(bytes)
            .into_iter()
            .map(|body| serde_json::from_slice(&body).expect("a queued message is JSON"))
            .collect()
    }
}
