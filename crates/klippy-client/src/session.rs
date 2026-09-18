//! The protocol side of a session, shared by both front-ends.
//!
//! A session is what happens between a terminal and a server: it interprets a
//! typed line, sends the request it names, keeps track of what is owed, and
//! turns whatever comes back into [`Entry`] values. It never prints — the two
//! front-ends do that, and they could hardly be more different:
//!
//! | Front-end | Renders to |
//! |---|---|
//! | [`console`](super::console) | stdout, one line at a time — for a pipe or a scrollback |
//! | [`tui`](super::tui) | a full-screen window with a live log |
//!
//! Keeping this half free of both is what makes the plain mode testable without
//! a terminal, and the TUI's log just a list of entries.
//!
//! # Entries
//!
//! [`Entry`] is deliberately close to what happened rather than to how it looks:
//! a request went out, the server replied, the server pushed, the client has
//! something to say. A front-end decides whether a `Sent` entry is worth showing
//! (the TUI shows it, the line mode does not — its prompt already had it) and
//! what colour an error is.
//!
//! # Typing
//!
//! Before a line is sent it is looked at three ways, in this order: empty, a
//! `.`-prefixed local command, a whole request object, or `method` followed by
//! optional parameters. Requests and parameters are written in YAML — of which
//! JSON is a subset, so a line that was valid as JSON still is — because a
//! request is a small tree and YAML is what a tree is written in without
//! quoting every key. Only the last three can produce a request, and a
//! malformed one is a notice rather than an error — the session stays up.

use serde_json::{json, Map, Value};

use klippy_api::address::{ApiTarget, Transport};
use klippy_api::TransportError;

use crate::connection::{Connection, Incoming, Reply};

/// How loud a line from the host's own log is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogLevel {
    /// Development detail, shown with `--verbose`.
    Debug,
    /// Something happened.
    Info,
    /// Something is off but the host carries on.
    Warn,
    /// Something failed.
    Error,
}

impl LogLevel {
    /// The tag a front-end shows in front of the line.
    pub fn tag(&self) -> &'static str {
        match self {
            LogLevel::Debug => "DEBUG",
            LogLevel::Info => "INFO ",
            LogLevel::Warn => "WARN ",
            LogLevel::Error => "ERROR",
        }
    }
}

/// How loud a client notice is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Notice {
    /// Ordinary information: usage text, a confirmation.
    Info,
    /// A hint about what will happen (an unanswered request, say).
    Hint,
    /// The typed line could not be used.
    Problem,
    /// The connection is gone.
    Failure,
}

/// One thing that happened during a session.
#[derive(Debug, Clone, PartialEq)]
pub enum Entry {
    /// A request this client sent.
    Sent {
        /// The `id` it carried, if it will be answered.
        id: Option<u64>,
        /// The method it names.
        method: String,
        /// The message as it went on the wire.
        message: Value,
    },
    /// A reply from the server.
    Reply(Reply),
    /// Something the server sent on its own.
    Push(Value),
    /// Something the client has to say.
    Notice { kind: Notice, text: String },
    /// A line from the host's own log.
    ///
    /// Only a host that embedded a client can produce these — nothing arrives
    /// on the wire for them — which is why they are separate from [`Notice`]:
    /// they are the host talking about itself, and a front-end shows the two
    /// differently.
    Log { level: LogLevel, text: String },
}

impl Entry {
    /// A client notice.
    pub fn notice(kind: Notice, text: impl Into<String>) -> Self {
        Entry::Notice {
            kind,
            text: text.into(),
        }
    }

    /// The text a front-end shows for this entry, without any decoration.
    pub fn text(&self) -> String {
        match self {
            Entry::Sent { id, message, .. } => match id {
                Some(id) => format!("{id} > {}", compact(message)),
                None => format!("> {}", compact(message)),
            },
            Entry::Reply(reply) => {
                let method = reply.method.as_deref().unwrap_or("?");
                let id = &reply.id;
                match reply.error_message() {
                    Some(message) => format!("! {id} ({method}) {message}"),
                    None => format!("{id} ({method}) {}", compact(reply.payload())),
                }
            }
            Entry::Push(message) => format!("< {}", compact(message)),
            Entry::Notice { text, .. } => text.clone(),
            Entry::Log { level, text } => format!("{} {text}", level.tag()),
        }
    }
}

impl From<Incoming> for Entry {
    fn from(incoming: Incoming) -> Self {
        match incoming {
            Incoming::Reply(reply) => Entry::Reply(reply),
            Incoming::Push(message) => Entry::Push(message),
        }
    }
}

/// Where a session's entries go.
pub trait Output {
    /// Take one entry. Called once per thing that happens.
    fn write(&mut self, entry: Entry);
}

/// What a typed line asked for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Control {
    /// Keep the session up.
    Continue,
    /// The user asked to leave.
    Quit,
}

/// A session with an API server.
pub struct Session {
    /// What to call the other end, for the greeting and for errors.
    label: String,
    connection: Connection,
}

impl Session {
    /// Connect to `target`.
    ///
    /// # Errors
    /// Returns [`TransportError`] if the server cannot be reached.
    pub async fn connect(target: ApiTarget) -> Result<Self, TransportError> {
        let connection = Connection::connect(&target).await?;
        Ok(Self {
            label: target.to_string(),
            connection,
        })
    }

    /// A session over a transport that is already connected.
    ///
    /// `label` is what the session calls the other end — a host that embedded a
    /// client has no address to name, and says so.
    pub fn from_transport(transport: Box<dyn Transport>, label: impl Into<String>) -> Self {
        Self {
            label: label.into(),
            connection: Connection::from_transport(transport),
        }
    }

    /// What this session calls the other end.
    pub fn label(&self) -> &str {
        &self.label
    }

    /// Ask the server who it is, the way every real client starts.
    ///
    /// Failures are reported and survived: an uninteresting `info` is no reason
    /// to refuse a session, and seeing the error is often the point.
    pub async fn handshake(&mut self, out: &mut impl Output) -> Result<(), TransportError> {
        let id = self.request("info", Map::new(), out).await?;
        let Some(reply) = self.await_reply(id, out).await? else {
            return Ok(());
        };
        if reply.is_error() {
            out.write(Entry::notice(
                Notice::Problem,
                format!(
                    "info: {}",
                    reply.error_message().unwrap_or("the request failed")
                ),
            ));
            return Ok(());
        }
        out.write(Entry::Reply(reply));
        Ok(())
    }

    /// Send a request, recording it so that every request a session makes shows
    /// up — including the ones a local command makes on the user's behalf.
    async fn request(
        &mut self,
        method: &str,
        params: Map<String, Value>,
        out: &mut impl Output,
    ) -> Result<u64, TransportError> {
        let id = self.connection.request(method, params.clone()).await?;
        out.write(Entry::Sent {
            id: Some(id),
            method: method.to_string(),
            message: json!({ "id": id, "method": method, "params": params }),
        });
        Ok(id)
    }

    /// Interpret one typed line.
    ///
    /// # Errors
    /// Returns [`TransportError`] if the request cannot be sent, or if a reply
    /// that a local command is waiting for never arrives because the connection
    /// went away.
    pub async fn handle_line(
        &mut self,
        line: &str,
        out: &mut impl Output,
    ) -> Result<Control, TransportError> {
        let line = line.trim();
        if line.is_empty() {
            return Ok(Control::Continue);
        }
        if let Some(command) = line.strip_prefix('.') {
            return self.local_command(command, out).await;
        }

        // A whole request object is taken as written, so every field the
        // protocol has — `id`, `params`, anything a future version adds — is
        // reachable from the session. Only a missing `id` is filled in.
        if line.starts_with('{') {
            return self.send_object(line, out).await;
        }

        // Otherwise: `method` and optional parameters.
        let (method, params) = match line.split_once(char::is_whitespace) {
            None => (line, "{}"),
            Some((method, rest)) => (method, rest.trim()),
        };
        let params = match parse_params(params) {
            Ok(params) => params,
            Err(problem) => {
                out.write(Entry::notice(Notice::Problem, problem));
                return Ok(Control::Continue);
            }
        };
        self.request(method, params, out).await?;
        Ok(Control::Continue)
    }

    /// Send a line the user wrote as a whole request object.
    ///
    /// The line is YAML, so a request can be written without quoting its keys;
    /// JSON is a subset of YAML, so the wire form is still accepted as written.
    async fn send_object(
        &mut self,
        line: &str,
        out: &mut impl Output,
    ) -> Result<Control, TransportError> {
        let mut message: Value = match serde_yaml::from_str(line) {
            Ok(message) => message,
            Err(err) => {
                out.write(Entry::notice(Notice::Problem, format!("not YAML: {err}")));
                return Ok(Control::Continue);
            }
        };
        let Some(object) = message.as_object_mut() else {
            out.write(Entry::notice(
                Notice::Problem,
                "a request must be a mapping",
            ));
            return Ok(Control::Continue);
        };
        let Some(method) = object
            .get("method")
            .and_then(Value::as_str)
            .map(str::to_string)
        else {
            out.write(Entry::notice(
                Notice::Problem,
                "a request needs a \"method\"",
            ));
            return Ok(Control::Continue);
        };

        if !object.contains_key("id") {
            // Answered by default: a session exists to show replies, so the
            // deliberate silence has to be asked for.
            object.insert("id".to_string(), json!(self.connection.take_id()));
        } else if object.get("id").is_some_and(Value::is_null) {
            out.write(Entry::notice(
                Notice::Hint,
                "id is null: sending it without expecting a reply",
            ));
        }
        self.connection.send(&message).await?;
        out.write(Entry::Sent {
            id: answerable_id(&message),
            method,
            message,
        });
        Ok(Control::Continue)
    }

    /// Handle a `.`-prefixed line, which the server never sees.
    async fn local_command(
        &mut self,
        command: &str,
        out: &mut impl Output,
    ) -> Result<Control, TransportError> {
        let (name, rest) = match command.split_once(char::is_whitespace) {
            None => (command, ""),
            Some((name, rest)) => (name, rest.trim()),
        };

        match name {
            "help" | "h" | "?" => out.write(Entry::notice(Notice::Info, usage())),
            "quit" | "exit" | "q" => return Ok(Control::Quit),
            // The client's own shortcut: a subscription cannot be built without
            // first asking which objects exist, and every real client does this
            // pair of calls at startup.
            "subscribe" | "sub" => self.subscribe(rest, out).await?,
            other => out.write(Entry::notice(
                Notice::Problem,
                format!("unknown command '.{other}'; try '.help'"),
            )),
        }
        Ok(Control::Continue)
    }

    /// Subscribe to objects, so their updates start arriving as pushes.
    pub async fn subscribe(
        &mut self,
        selection: &str,
        out: &mut impl Output,
    ) -> Result<(), TransportError> {
        let requested: Vec<&str> = selection.split_whitespace().collect();

        let names = if requested.is_empty() {
            // Every object, which is what watching a printer wants.
            let id = self.request("objects/list", Map::new(), out).await?;
            let Some(reply) = self.await_reply(id, out).await? else {
                return Ok(());
            };
            if reply.is_error() {
                out.write(Entry::notice(
                    Notice::Problem,
                    format!(
                        "objects/list failed: {}",
                        reply.error_message().unwrap_or("the request failed")
                    ),
                ));
                return Ok(());
            }
            reply
                .result()
                .and_then(|result| result.get("objects"))
                .and_then(Value::as_array)
                .map(|objects| {
                    objects
                        .iter()
                        .filter_map(Value::as_str)
                        .map(str::to_string)
                        .collect::<Vec<_>>()
                })
                .unwrap_or_default()
        } else {
            requested.iter().map(|name| name.to_string()).collect()
        };

        if names.is_empty() {
            out.write(Entry::notice(Notice::Info, "nothing to subscribe to"));
            return Ok(());
        }

        // `null` per object means "every field", and the template names the
        // pushes so they are readable on their own.
        let objects: Map<String, Value> = names
            .iter()
            .map(|name| (name.clone(), Value::Null))
            .collect();
        let mut params = Map::new();
        params.insert("objects".to_string(), Value::Object(objects));
        params.insert(
            "response_template".to_string(),
            json!({"id": null, "method": "klippy:status"}),
        );
        self.request("objects/subscribe", params, out).await?;
        out.write(Entry::notice(
            Notice::Info,
            format!("Subscribed to {} object(s).", names.len()),
        ));
        Ok(())
    }

    /// Read one message from the server.
    ///
    /// Cancel-safe, so a front-end may race it against its own input.
    ///
    /// # Errors
    /// Returns [`TransportError`] when the connection is gone.
    pub async fn receive(&mut self) -> Result<Incoming, TransportError> {
        self.connection.receive().await
    }

    /// Whether a reply is still owed.
    pub fn has_pending(&self) -> bool {
        self.connection.has_pending()
    }

    /// Read the replies still owed, for at most a moment.
    ///
    /// A front-end that is about to stop reading uses this so that requests it
    /// just sent are not dropped on the floor. Waiting only while something is
    /// actually owed is what keeps leaving instant in the normal case.
    pub async fn drain(&mut self, out: &mut impl Output, grace: std::time::Duration) {
        let deadline = tokio::time::Instant::now() + grace;
        while self.connection.has_pending() {
            match tokio::time::timeout_at(deadline, self.connection.receive()).await {
                Err(_) => return,
                Ok(Ok(message)) => out.write(message.into()),
                Ok(Err(err)) => {
                    out.write(Entry::notice(Notice::Failure, err.to_string()));
                    return;
                }
            }
        }
    }

    /// Wait for the reply to `id`, handing over anything that arrives first.
    ///
    /// A local command that needs an answer has to read it itself — the
    /// front-end's own loop is not running while a command is being handled —
    /// so this is the one place that consumes messages outside it. Anything
    /// skipped is written out, so nothing is silently dropped.
    async fn await_reply(
        &mut self,
        id: u64,
        out: &mut impl Output,
    ) -> Result<Option<Reply>, TransportError> {
        loop {
            match self.connection.receive().await? {
                Incoming::Reply(reply) if reply.id == json!(id) => return Ok(Some(reply)),
                other => out.write(other.into()),
            }
        }
    }
}

/// The text `.help` prints, and the TUI shows in its footer.
pub fn usage() -> &'static str {
    "\
Type a request: a method name (`info`), a method and parameters
(`objects/query {objects: {toolhead: null}}`), or a whole request object.
They are YAML — JSON is YAML too. An `id` is added when you leave it out;
`{id: null, ...}` sends it unanswered.

Local commands:
  .help          this text
  .subscribe     watch every object (`objects/list` + `objects/subscribe`)
  .subscribe a b watch only the named objects
  .quit          leave, after printing any reply still owed (also ^D)

Replies and pushes carry their direction; line mode prints one compact JSON line
each, the window shows the body as YAML by default."
}

/// One line, as the protocol is written on the wire.
pub fn compact(value: &Value) -> String {
    serde_json::to_string(value).unwrap_or_else(|_| "null".to_string())
}

/// Parse the parameters of a shorthand request.
///
/// YAML, so `{objects: {toolhead: null}}` needs no quoting; JSON is a subset of
/// YAML, so anything that used to parse still does.
fn parse_params(value: &str) -> Result<Map<String, Value>, String> {
    if value.is_empty() {
        return Ok(Map::new());
    }
    match serde_yaml::from_str::<Value>(value) {
        Ok(Value::Object(params)) => Ok(params),
        Ok(other) => Err(format!(
            "parameters must be a mapping, not {}",
            kind_of(&other)
        )),
        Err(err) => Err(format!("parameters are not YAML: {err}")),
    }
}

/// Name a JSON value's kind, for error messages.
fn kind_of(value: &Value) -> &'static str {
    match value {
        Value::Null => "null",
        Value::Bool(_) => "a boolean",
        Value::Number(_) => "a number",
        Value::String(_) => "a string",
        Value::Array(_) => "an array",
        Value::Object(_) => "an object",
    }
}

/// The `id` a message will be answered with, when this client issued it.
fn answerable_id(message: &Value) -> Option<u64> {
    message.get("id").and_then(Value::as_u64)
}

/// Whether `line` is a local command rather than a request.
pub fn is_local(line: &str) -> bool {
    line.trim_start().starts_with('.')
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use klippy_api::protocol::{ApiError, Request};
    use klippy_api::registry::{Api, Endpoint, EndpointContext};
    use klippy_api::server::Server;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;
    use std::time::Duration;

    /// An endpoint that answers with whatever it was given, so a typed line can
    /// be followed all the way to an entry.
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
            Ok(json!({ "got": request.params().get_or("value", &Value::Null).clone() }))
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
            Err(ApiError::InvalidArgumentType("value".to_string()))
        }
    }

    /// What `objects/list` reports, so `.subscribe` has something to find.
    struct ListObjects;

    impl Endpoint for ListObjects {
        fn path(&self) -> &'static str {
            "objects/list"
        }

        fn handle(
            &self,
            _request: &Request,
            _context: &EndpointContext<'_>,
        ) -> Result<Value, ApiError> {
            Ok(json!({"objects": ["toolhead", "extruder"]}))
        }
    }

    /// Records what it was subscribed to, and pushes an update.
    struct Subscribe;

    impl Endpoint for Subscribe {
        fn path(&self) -> &'static str {
            "objects/subscribe"
        }

        fn handle(
            &self,
            request: &Request,
            context: &EndpointContext<'_>,
        ) -> Result<Value, ApiError> {
            let objects = request.params().get_opt("objects").cloned();
            let client = Arc::clone(&context.client);
            tokio::spawn(async move {
                tokio::time::sleep(Duration::from_millis(10)).await;
                client.push(json!({
                    "method": "klippy:status",
                    "params": {"status": {"toolhead": {"position": [0, 0, 0, 0]}}}
                }));
            });
            Ok(json!({"subscribed": objects}))
        }
    }

    /// The handshake asks for `info`, so the harness answers it.
    struct Info;

    impl Endpoint for Info {
        fn path(&self) -> &'static str {
            "info"
        }

        fn handle(
            &self,
            _request: &Request,
            _context: &EndpointContext<'_>,
        ) -> Result<Value, ApiError> {
            Ok(json!({"state": "ready", "state_message": "Printer is ready"}))
        }
    }

    /// A socket path in a directory that removes itself.
    struct SocketDir(std::path::PathBuf);

    impl SocketDir {
        fn new(name: &str) -> Self {
            static COUNTER: AtomicUsize = AtomicUsize::new(0);
            let dir = std::env::temp_dir().join(format!(
                "klipperx-session-{}-{}-{}",
                std::process::id(),
                name,
                COUNTER.fetch_add(1, Ordering::Relaxed)
            ));
            std::fs::create_dir_all(&dir).expect("cannot create the test directory");
            Self(dir)
        }

        fn target(&self) -> ApiTarget {
            ApiTarget::Unix(self.0.join("klippy_uds"))
        }
    }

    impl Drop for SocketDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// Collects entries instead of rendering them.
    #[derive(Default)]
    struct Recording(Vec<Entry>);

    impl Recording {
        fn texts(&self) -> Vec<String> {
            self.0.iter().map(Entry::text).collect()
        }

        /// The entries of one kind, as text.
        fn sent(&self) -> Vec<&Entry> {
            self.0
                .iter()
                .filter(|entry| matches!(entry, Entry::Sent { .. }))
                .collect()
        }
    }

    impl Output for Recording {
        fn write(&mut self, entry: Entry) {
            self.0.push(entry);
        }
    }

    /// A server with the endpoints these tests talk to.
    async fn server(dir: &SocketDir) -> tokio::task::JoinHandle<()> {
        let mut api = Api::new();
        api.register(Info).unwrap();
        api.register(Echo).unwrap();
        api.register(Failing).unwrap();
        api.register(ListObjects).unwrap();
        api.register(Subscribe).unwrap();
        let server = Server::bind(dir.target(), Arc::new(api))
            .await
            .expect("cannot bind");
        tokio::spawn(async move {
            let _ = server.run().await;
        })
    }

    /// A connected session whose handshake is done.
    ///
    /// The handshake's own output is thrown away — it has a test of its own, and
    /// every other test here is about what a *typed* line produces. The
    /// handshake still happens, which is why the first request in these tests
    /// carries id 2.
    async fn session(dir: &SocketDir) -> (Session, Recording, tokio::task::JoinHandle<()>) {
        let task = server(dir).await;
        let mut session = Session::connect(dir.target())
            .await
            .expect("cannot connect");
        session
            .handshake(&mut Recording::default())
            .await
            .expect("handshake failed");
        (session, Recording::default(), task)
    }

    /// Read what the server has to say until it goes quiet, and record it.
    async fn settle(session: &mut Session, out: &mut Recording) {
        while let Ok(Ok(message)) =
            tokio::time::timeout(Duration::from_millis(50), session.receive()).await
        {
            out.write(message.into());
        }
    }

    // -----------------------------------------------------------------------
    // Typing
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_the_handshake_asks_who_the_server_is() {
        let dir = SocketDir::new("handshake");
        let task = server(&dir).await;
        let mut session = Session::connect(dir.target())
            .await
            .expect("cannot connect");
        let mut out = Recording::default();
        session.handshake(&mut out).await.expect("handshake failed");

        // The request it made, and the answer to it.
        assert_eq!(out.0.len(), 2, "{:?}", out.texts());
        match &out.0[0] {
            Entry::Sent { id, method, .. } => {
                assert_eq!(*id, Some(1));
                assert_eq!(method, "info");
            }
            other => panic!("expected the info request, got {other:?}"),
        }
        match &out.0[1] {
            Entry::Reply(reply) => {
                assert_eq!(reply.method.as_deref(), Some("info"));
                assert_eq!(reply.result().unwrap()["state"], json!("ready"));
            }
            other => panic!("expected the info reply, got {other:?}"),
        }
        task.abort();
    }

    #[tokio::test]
    async fn test_a_bare_method_is_sent_with_an_id() {
        let dir = SocketDir::new("method");
        let (mut session, mut out, task) = session(&dir).await;

        assert_eq!(
            session.handle_line("echo", &mut out).await.unwrap(),
            Control::Continue
        );
        settle(&mut session, &mut out).await;

        // What went out, and what came back.
        let sent = out.sent();
        assert_eq!(sent.len(), 1);
        match sent[0] {
            Entry::Sent { id, method, .. } => {
                assert_eq!(*id, Some(2), "the handshake took id 1");
                assert_eq!(method, "echo");
            }
            other => panic!("expected a sent entry, got {other:?}"),
        }
        assert!(
            out.texts().iter().any(|text| text.contains("2 (echo)")),
            "{:?}",
            out.texts()
        );
        task.abort();
    }

    #[tokio::test]
    async fn test_a_method_and_parameters_are_parsed() {
        let dir = SocketDir::new("params");
        let (mut session, mut out, task) = session(&dir).await;

        session
            .handle_line("echo {\"value\": 3}", &mut out)
            .await
            .unwrap();
        settle(&mut session, &mut out).await;

        assert!(
            out.texts().iter().any(|text| text.contains("\"got\":3")),
            "{:?}",
            out.texts()
        );
        task.abort();
    }

    #[tokio::test]
    async fn test_bad_parameters_are_a_notice_not_a_request() {
        let dir = SocketDir::new("badparams");
        let (mut session, mut out, task) = session(&dir).await;

        for line in ["echo [1,2,3]", "echo not a mapping", "echo {a: [1, 2"] {
            assert_eq!(
                session.handle_line(line, &mut out).await.unwrap(),
                Control::Continue
            );
        }

        assert!(out.sent().is_empty(), "{:?}", out.texts());
        // A YAML sequence and a YAML scalar are both readable, just not
        // parameters; only the unterminated mapping is a parse error.
        assert_eq!(out.texts()[0], "parameters must be a mapping, not an array");
        assert_eq!(out.texts()[1], "parameters must be a mapping, not a string");
        assert!(
            out.texts()[2].starts_with("parameters are not YAML:"),
            "{:?}",
            out.texts()
        );
        task.abort();
    }

    #[tokio::test]
    async fn test_yaml_parameters_need_no_quotes() {
        let dir = SocketDir::new("yamlparams");
        let (mut session, mut out, task) = session(&dir).await;

        session
            .handle_line("echo {value: 3}", &mut out)
            .await
            .unwrap();
        settle(&mut session, &mut out).await;

        assert!(
            out.texts().iter().any(|text| text.contains("\"got\":3")),
            "{:?}",
            out.texts()
        );
        task.abort();
    }

    #[tokio::test]
    async fn test_a_whole_request_object_may_be_yaml() {
        let dir = SocketDir::new("yamlobject");
        let (mut session, mut out, task) = session(&dir).await;

        session
            .handle_line("{method: echo, params: {value: 7}}", &mut out)
            .await
            .unwrap();
        settle(&mut session, &mut out).await;

        assert!(
            out.texts().iter().any(|text| text.contains("\"got\":7")),
            "{:?}",
            out.texts()
        );
        task.abort();
    }

    #[tokio::test]
    async fn test_a_whole_json_object_is_taken_as_written() {
        let dir = SocketDir::new("json");
        let (mut session, mut out, task) = session(&dir).await;

        // An explicit id is honoured, not overwritten.
        session
            .handle_line(
                r#"{"id": 42, "method": "echo", "params": {"value": "x"}}"#,
                &mut out,
            )
            .await
            .unwrap();
        settle(&mut session, &mut out).await;

        match out.sent()[0] {
            Entry::Sent { id, .. } => assert_eq!(*id, Some(42)),
            other => panic!("expected a sent entry, got {other:?}"),
        }
        assert!(
            out.texts().iter().any(|text| text.contains("42 (echo)")),
            "{:?}",
            out.texts()
        );
        task.abort();
    }

    #[tokio::test]
    async fn test_a_json_object_without_an_id_gets_one() {
        let dir = SocketDir::new("noid");
        let (mut session, mut out, task) = session(&dir).await;

        session
            .handle_line(r#"{"method": "echo"}"#, &mut out)
            .await
            .unwrap();

        match out.sent()[0] {
            Entry::Sent { id, .. } => assert_eq!(*id, Some(2)),
            other => panic!("expected a sent entry, got {other:?}"),
        }
        task.abort();
    }

    #[tokio::test]
    async fn test_a_json_object_with_id_null_is_left_unanswered() {
        let dir = SocketDir::new("nullid");
        let (mut session, mut out, task) = session(&dir).await;

        session
            .handle_line(r#"{"id": null, "method": "echo"}"#, &mut out)
            .await
            .unwrap();

        // It went out, but nothing is owed for it.
        match out.sent()[0] {
            Entry::Sent { id, .. } => assert_eq!(*id, None),
            other => panic!("expected a sent entry, got {other:?}"),
        }
        assert!(
            out.texts()
                .iter()
                .any(|text| text.contains("without expecting a reply")),
            "{:?}",
            out.texts()
        );
        assert!(!session.has_pending());
        task.abort();
    }

    #[tokio::test]
    async fn test_a_line_that_is_not_a_request_is_refused_locally() {
        let dir = SocketDir::new("notrequest");
        let (mut session, mut out, task) = session(&dir).await;

        for line in [r#"{"params": {}}"#, "echo [1,2]"] {
            assert_eq!(
                session.handle_line(line, &mut out).await.unwrap(),
                Control::Continue
            );
        }

        assert!(out.sent().is_empty(), "{:?}", out.texts());
        let texts = out.texts();
        assert!(texts[0].contains("needs a \"method\""), "{texts:?}");
        assert!(texts[1].contains("must be a mapping"), "{texts:?}");

        // A line that is neither local, nor a request object, nor a method
        // followed by parameters is still taken as `method [params]` — whatever
        // the method name happens to look like. The server is the one that
        // rejects it.
        session.handle_line("[1,2]", &mut out).await.unwrap();
        let Entry::Sent { id, method, .. } = out.sent()[0] else {
            panic!("expected a sent entry")
        };
        assert_eq!(*id, Some(2));
        assert_eq!(method, "[1,2]");
        task.abort();
    }

    #[tokio::test]
    async fn test_an_empty_line_does_nothing() {
        let dir = SocketDir::new("empty");
        let (mut session, mut out, task) = session(&dir).await;

        session.handle_line("   ", &mut out).await.unwrap();
        assert!(out.0.is_empty(), "{:?}", out.texts());
        task.abort();
    }

    // -----------------------------------------------------------------------
    // Local commands
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_local_commands_never_reach_the_server() {
        let dir = SocketDir::new("local");
        let (mut session, mut out, task) = session(&dir).await;

        session.handle_line(".help", &mut out).await.unwrap();
        session.handle_line(".nonsense", &mut out).await.unwrap();

        assert!(out.sent().is_empty(), "{:?}", out.texts());
        let texts = out.texts();
        assert!(texts[0].contains("Local commands:"), "{texts:?}");
        assert!(
            texts[1].contains("unknown command '.nonsense'"),
            "{texts:?}"
        );

        for line in [".quit", ".exit", ".q"] {
            assert_eq!(
                session.handle_line(line, &mut out).await.unwrap(),
                Control::Quit,
                "{line}"
            );
        }
        task.abort();
    }

    #[tokio::test]
    async fn test_subscribe_names_the_requested_objects() {
        let dir = SocketDir::new("sub");
        let (mut session, mut out, task) = session(&dir).await;

        session
            .handle_line(".subscribe toolhead extruder", &mut out)
            .await
            .unwrap();
        settle(&mut session, &mut out).await;

        let sent = out.sent();
        assert_eq!(sent.len(), 1);
        let Entry::Sent { message, .. } = sent[0] else {
            panic!("expected a sent entry")
        };
        assert_eq!(message["method"], json!("objects/subscribe"));
        // `null` per object means every field, and the template names the pushes.
        assert_eq!(
            message["params"]["objects"],
            json!({"toolhead": null, "extruder": null})
        );
        assert_eq!(
            message["params"]["response_template"],
            json!({"id": null, "method": "klippy:status"})
        );
        assert!(
            out.texts()
                .iter()
                .any(|text| text.contains("Subscribed to 2")),
            "{:?}",
            out.texts()
        );

        // The endpoint pushes, which is what a subscription is for.
        assert!(
            out.texts()
                .iter()
                .any(|text| text.starts_with("< {\"method\":\"klippy:status\"")),
            "{:?}",
            out.texts()
        );
        task.abort();
    }

    #[tokio::test]
    async fn test_subscribe_without_arguments_asks_which_objects_exist() {
        let dir = SocketDir::new("suball");
        let (mut session, mut out, task) = session(&dir).await;

        session.handle_line(".subscribe", &mut out).await.unwrap();
        settle(&mut session, &mut out).await;

        // `objects/list` first, then a subscription built from what it reported.
        let sent = out.sent();
        let methods: Vec<String> = sent
            .iter()
            .map(|entry| match entry {
                Entry::Sent { method, .. } => method.clone(),
                other => panic!("expected a sent entry, got {other:?}"),
            })
            .collect();
        assert_eq!(methods, vec!["objects/list", "objects/subscribe"]);

        let Entry::Sent { message, .. } = sent.last().unwrap() else {
            panic!("expected a sent entry")
        };
        assert_eq!(
            message["params"]["objects"],
            json!({"toolhead": null, "extruder": null})
        );
        task.abort();
    }

    #[tokio::test]
    async fn test_an_error_reply_is_an_entry_with_the_server_message() {
        let dir = SocketDir::new("fail");
        let (mut session, mut out, task) = session(&dir).await;

        session.handle_line("failing", &mut out).await.unwrap();
        settle(&mut session, &mut out).await;

        assert!(
            out.texts()
                .iter()
                .any(|text| text == "! 2 (failing) Invalid Argument Type [value]"),
            "{:?}",
            out.texts()
        );
        task.abort();
    }

    // -----------------------------------------------------------------------
    // Leaving
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_drain_prints_what_was_still_owed() {
        let dir = SocketDir::new("drain");
        let (mut session, mut out, task) = session(&dir).await;

        // A request whose reply has not been read yet.
        session.handle_line("echo", &mut out).await.unwrap();
        assert!(session.has_pending());

        session.drain(&mut out, Duration::from_secs(5)).await;
        assert!(!session.has_pending());
        assert!(
            out.texts().iter().any(|text| text.contains("2 (echo)")),
            "{:?}",
            out.texts()
        );
        task.abort();
    }

    #[tokio::test]
    async fn test_drain_returns_at_once_when_nothing_is_owed() {
        let dir = SocketDir::new("drainfast");
        let (mut session, mut out, task) = session(&dir).await;

        let started = std::time::Instant::now();
        session.drain(&mut out, Duration::from_secs(30)).await;
        assert!(
            started.elapsed() < Duration::from_secs(1),
            "a session with nothing outstanding must not wait"
        );
        task.abort();
    }

    // -----------------------------------------------------------------------
    // Entries
    // -----------------------------------------------------------------------

    #[test]
    fn test_entry_text_is_what_a_front_end_shows() {
        let mut out = Recording::default();
        out.write(Entry::notice(Notice::Info, "hello"));

        assert_eq!(
            Entry::notice(Notice::Info, "hello").text(),
            "hello",
            "a notice is its text"
        );
        assert_eq!(
            Entry::Sent {
                id: Some(7),
                method: "echo".to_string(),
                message: json!({"id": 7, "method": "echo"}),
            }
            .text(),
            "7 > {\"id\":7,\"method\":\"echo\"}"
        );
        assert_eq!(
            Entry::Sent {
                id: None,
                method: "echo".to_string(),
                message: json!({"method": "echo"}),
            }
            .text(),
            "> {\"method\":\"echo\"}",
            "an unanswered request has no id to show"
        );
        assert_eq!(
            Entry::Push(json!({"method": "klippy:status"})).text(),
            "< {\"method\":\"klippy:status\"}"
        );
    }

    #[test]
    fn test_client_lines_are_told_apart() {
        assert!(is_local(".help"));
        assert!(is_local("  .quit"));
        assert!(!is_local("info"));
        assert!(!is_local(r#"{"method": "info"}"#));
    }

    #[test]
    fn test_usage_names_every_local_command() {
        let usage = usage();
        for command in [".help", ".subscribe", ".quit"] {
            assert!(usage.contains(command), "{command} is not documented");
        }
    }
}
