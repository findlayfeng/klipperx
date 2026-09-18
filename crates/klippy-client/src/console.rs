//! The interactive console: `klippy-client console`.
//!
//! A session is two independent streams — what the user types and what the
//! server sends — so the loop `select!`s between them. That is the whole point
//! of the console: a subscription keeps printing while the prompt is idle,
//! which is the one thing `scripts/whconsole.py` (and a `telnet`-style client)
//! cannot do, because it only reads the socket while stdin has nothing.
//!
//! # Typing a request
//!
//! Three forms, in increasing order of explicitness:
//!
//! ```text
//! klippy> info
//! klippy> objects/query {"objects": {"toolhead": ["position"]}}
//! klippy> {"id": 9, "method": "gcode/script", "params": {"script": "M115"}}
//! ```
//!
//! The first two are shorthand: the method, then optional parameters as a JSON
//! object. In all three, an `id` is supplied when the input did not name one, so
//! a typed request is answered visibly; `"id": null` is left alone, because that
//! is the protocol's way of asking for no reply at all.
//!
//! # Reading the output
//!
//! ```text
//! 1 (info) { … }                     a reply: the id, the method that asked, the result
//! ! 2 (objects/query) Missing Argument [objects]     a failed request
//! < {"method": "klippy:status", …}   a push: something the server sent unprompted
//! ```
//!
//! # Local commands
//!
//! A line starting with `.` is the console's own, not the server's; `.help`
//! lists them.
//!
//! # Input that is not a terminal
//!
//! A piped stdin — `printf 'list_endpoints\n' | klipperx console` — is served
//! the same way, minus the banner and the prompt, and leaving — by ^D, by `.quit`,
//! or by the pipe running out — first prints the replies still owed, for up to a
//! second: requests that were just sent have not been answered yet, and dropping
//! them would make a simple pipeline print nothing at all. A session with nothing
//! outstanding leaves at once. For scripting, `klipperx api` is the tool.
//!
//! # Known gaps
//!
//! * No reconnection: when the server goes away — which it does on `RESTART`,
//!   `FIRMWARE_RESTART` and on any shutdown — the session ends with a message.
//!   Reconnecting would mean re-establishing every subscription and re-numbering
//!   ids, which is a client's decision to make, not a console's.
//! * No line editing or history: stdin is read line by line. A push printed
//!   while a line is being typed lands in the middle of it, as it does in the
//!   clients upstream ships.

use std::io::{IsTerminal, Write as _};
use std::time::Duration;

use serde_json::{json, Map, Value};
use tokio::io::AsyncBufReadExt;

use klippy_api::address::ApiTarget;
use klippy_api::TransportError;

use super::connection::{Connection, Incoming, Reply};

/// How long to keep listening at most, after input ends or `.quit` is typed, so
/// that replies already on their way are printed before the session ends.
///
/// This is the ceiling, not the wait: a session with nothing outstanding leaves
/// at once (see [`Console::drain`]).
const EOF_GRACE: Duration = Duration::from_secs(1);

/// What a local command tells the loop to do.
enum Control {
    /// Keep reading.
    Continue,
    /// Leave the session.
    Quit,
}

/// An interactive session against an API server.
pub struct Console {
    target: ApiTarget,
    connection: Connection,
}

impl Console {
    /// Connect and greet the user.
    ///
    /// # Errors
    /// Returns [`TransportError::Connect`] if the server cannot be reached
    pub async fn new(target: ApiTarget) -> Result<Self, TransportError> {
        let connection = Connection::connect(&target).await?;
        println!("Connected to {target}.");
        let mut console = Self { target, connection };
        console.handshake().await?;
        Ok(console)
    }

    /// Ask the server who it is, the way every real client starts.
    ///
    /// Failures are reported and survived: an uninteresting `info` is no reason
    /// to refuse a session, and seeing the error is often the point.
    async fn handshake(&mut self) -> Result<(), TransportError> {
        let id = self.connection.request("info", Map::new()).await?;
        match self.await_reply(id).await? {
            None => println!("info: no reply"),
            Some(reply) if reply.is_error() => {
                println!(
                    "info: {}",
                    reply.error_message().unwrap_or("the request failed")
                );
            }
            Some(reply) => {
                let result = reply.result().cloned().unwrap_or(Value::Null);
                let field = |name: &str| {
                    result
                        .get(name)
                        .and_then(Value::as_str)
                        .unwrap_or("?")
                        .to_string()
                };
                println!(
                    "Printer is {} — {} ({}, {})",
                    field("state"),
                    field("state_message").replace('\n', " "),
                    field("software_version"),
                    field("cpu_info"),
                );
            }
        }
        Ok(())
    }

    /// Read requests until the user leaves or the server goes away.
    ///
    /// # Errors
    /// Returns [`TransportError`] only for a read that failed outright; the server
    /// closing the connection is reported and ends the session normally.
    pub async fn run(mut self) -> Result<(), TransportError> {
        // A terminal is told what the syntax is and shown a prompt; a pipe is
        // not, so that its output is only the answers.
        let interactive = std::io::stdin().is_terminal();
        if interactive {
            self.usage();
        }
        let mut lines = tokio::io::BufReader::new(tokio::io::stdin()).lines();

        loop {
            if interactive {
                print!("klippy> ");
                let _ = std::io::stdout().flush();
            }

            tokio::select! {
                line = lines.next_line() => match line {
                    // End of input: the user pressed ^D, or a pipe ran out.
                    Ok(None) => break,
                    Err(err) => {
                        println!("cannot read input: {err}");
                        break;
                    }
                    Ok(Some(line)) => match self.handle_line(&line).await? {
                        Control::Continue => (),
                        // `.quit` stops taking input; what is still owed is read
                        // below, the same way it is when input ends on its own.
                        Control::Quit => break,
                    },
                },
                // A push, or a late reply. Printed as it arrives, which is what
                // lets a subscription be watched without typing anything.
                message = self.connection.receive() => match message {
                    Ok(message) => print_incoming(&message),
                    Err(err) => {
                        println!("{err}");
                        println!("Disconnected from {}.", self.target);
                        return Ok(());
                    }
                },
            }
        }

        self.drain().await;
        println!("Disconnected from {}.", self.target);
        Ok(())
    }

    /// Print the replies still owed, for at most a moment, before leaving.
    ///
    /// The requests just read may not have been answered yet — a pipe delivers
    /// every line before the server has seen the first one — so ending here would
    /// drop the very output the session was started for. Waiting only while
    /// something is actually owed is what keeps leaving instant in the normal
    /// case, where every request has already been answered on screen.
    async fn drain(&mut self) {
        let deadline = tokio::time::Instant::now() + EOF_GRACE;
        while self.connection.has_pending() {
            match tokio::time::timeout_at(deadline, self.connection.receive()).await {
                // The grace period is over.
                Err(_) => return,
                Ok(Ok(message)) => print_incoming(&message),
                Ok(Err(err)) => {
                    println!("{err}");
                    return;
                }
            }
        }
    }

    /// Handle one typed line.
    async fn handle_line(&mut self, line: &str) -> Result<Control, TransportError> {
        let line = line.trim();
        if line.is_empty() {
            return Ok(Control::Continue);
        }
        if let Some(command) = line.strip_prefix('.') {
            return self.local_command(command).await;
        }

        // A whole JSON object is taken as written, so every field the protocol
        // has — `id`, `params`, anything a future version adds — is reachable
        // from the console. Only a missing `id` is filled in.
        if line.starts_with('{') {
            return self.send_json(line).await;
        }

        // Otherwise: `method` and optional parameters.
        let (method, params) = match line.split_once(char::is_whitespace) {
            None => (line, "{}"),
            Some((method, rest)) => (method, rest.trim()),
        };
        let params = match parse_params(params) {
            Ok(params) => params,
            Err(err) => {
                println!("{err}");
                return Ok(Control::Continue);
            }
        };
        self.connection.request(method, params).await?;
        Ok(Control::Continue)
    }

    /// Send a line the user wrote as a JSON object.
    async fn send_json(&mut self, line: &str) -> Result<Control, TransportError> {
        let mut message: Value = match serde_json::from_str(line) {
            Ok(message) => message,
            Err(err) => {
                println!("not JSON: {err}");
                return Ok(Control::Continue);
            }
        };
        let Some(object) = message.as_object_mut() else {
            println!("a request must be a JSON object");
            return Ok(Control::Continue);
        };
        if !object.contains_key("method") {
            println!("a request needs a \"method\"");
            return Ok(Control::Continue);
        }

        if !object.contains_key("id") {
            // Answered by default: a console exists to show replies, so the
            // deliberate silence has to be asked for.
            object.insert("id".to_string(), json!(self.connection.take_id()));
        } else if object.get("id").is_some_and(Value::is_null) {
            println!("id is null: sending it without expecting a reply");
        }
        self.connection.send(&message).await?;
        Ok(Control::Continue)
    }

    /// Handle a `.`-prefixed line, which the server never sees.
    async fn local_command(&mut self, command: &str) -> Result<Control, TransportError> {
        let (name, rest) = match command.split_once(char::is_whitespace) {
            None => (command, ""),
            Some((name, rest)) => (name, rest.trim()),
        };

        match name {
            "help" | "h" | "?" => self.usage(),
            "quit" | "exit" | "q" => return Ok(Control::Quit),
            // The client's own shortcut: a subscription cannot be built without
            // first asking which objects exist, and every real client does this
            // pair of calls at startup.
            "subscribe" | "sub" => self.subscribe(rest).await?,
            other => println!("unknown command '.{other}'; try '.help'"),
        }
        Ok(Control::Continue)
    }

    fn usage(&self) {
        println!(
            "Type a request: a method name (`info`), a method and parameters\n\
             (`objects/query {{\"objects\": {{\"toolhead\": null}}}}`), or a whole JSON object.\n\
             An `id` is added when you leave it out; `\"id\": null` sends it unanswered.\n\
             \n\
             Local commands:\n\
             \x20 .help          this text\n\
             \x20 .subscribe     watch every object (`objects/list` + `objects/subscribe`)\n\
             \x20 .subscribe a b watch only the named objects\n\
             \x20 .quit          leave, after printing any reply still owed (also ^D)\n\
             \n\
             Replies print as `<id> (<method>) <result>`; pushes print as `< <message>`."
        );
    }

    /// Subscribe to objects, so their updates start arriving as pushes.
    async fn subscribe(&mut self, selection: &str) -> Result<(), TransportError> {
        let requested: Vec<&str> = selection.split_whitespace().collect();

        let names = if requested.is_empty() {
            // Every object, which is what a console wants to watch.
            let id = self.connection.request("objects/list", Map::new()).await?;
            let Some(reply) = self.await_reply(id).await? else {
                return Ok(());
            };
            if reply.is_error() {
                println!(
                    "objects/list failed: {}",
                    reply.error_message().unwrap_or("the request failed")
                );
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
            println!("nothing to subscribe to");
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
        self.connection.request("objects/subscribe", params).await?;

        println!(
            "Subscribed to {} object(s); updates print as `<`.",
            names.len()
        );
        Ok(())
    }

    /// Wait for the reply to `id`, printing anything else that arrives first.
    ///
    /// A local command that needs an answer has to read it itself — the console
    /// loop is not running while a command is being handled — so this is the one
    /// place that consumes messages outside the loop. Anything skipped is
    /// printed, so nothing is silently dropped.
    async fn await_reply(&mut self, id: u64) -> Result<Option<Reply>, TransportError> {
        loop {
            match self.connection.receive().await? {
                Incoming::Reply(reply) if reply.id == json!(id) => return Ok(Some(reply)),
                other => print_incoming(&other),
            }
        }
    }
}

/// Print one received message the way the console's usage text describes.
fn print_incoming(incoming: &Incoming) {
    match incoming {
        Incoming::Push(message) => println!("< {}", compact(message)),
        Incoming::Reply(reply) => {
            let method = reply.method.as_deref().unwrap_or("?");
            let id = &reply.id;
            if reply.is_error() {
                println!(
                    "! {id} ({method}) {}",
                    reply.error_message().unwrap_or("the request failed")
                );
            } else {
                println!("{}", indent(&format!("{id} ({method}) "), reply.payload()));
            }
        }
    }
}

/// Render a payload under its label.
///
/// A payload that fits on one line stays beside the label; a structured one goes
/// below it, indented by two. Aligning the continuation under the label instead
/// would push the data off the right edge as soon as the method name is long.
fn indent(label: &str, payload: &Value) -> String {
    let pretty = match serde_json::to_string_pretty(payload) {
        Ok(pretty) => pretty,
        Err(_) => compact(payload),
    };
    if !pretty.contains('\n') {
        return format!("{label}{pretty}");
    }
    format!("{label}\n  {}", pretty.replace('\n', "\n  "))
}

/// One line, as the protocol is written on the wire.
fn compact(value: &Value) -> String {
    serde_json::to_string(value).unwrap_or_else(|_| "null".to_string())
}

/// Parse the parameters of a shorthand request.
fn parse_params(value: &str) -> Result<Map<String, Value>, String> {
    if value.is_empty() {
        return Ok(Map::new());
    }
    match serde_json::from_str::<Value>(value) {
        Ok(Value::Object(params)) => Ok(params),
        Ok(other) => Err(format!(
            "parameters must be a JSON object, not {}",
            match other {
                Value::Null => "null",
                Value::Bool(_) => "a boolean",
                Value::Number(_) => "a number",
                Value::String(_) => "a string",
                Value::Array(_) => "an array",
                Value::Object(_) => unreachable!("handled above"),
            }
        )),
        Err(err) => Err(format!("parameters are not JSON: {err}")),
    }
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

    /// An endpoint that answers with whatever it was given, so a typed request
    /// can be followed all the way to a printed reply.
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

    /// The two object names `.subscribe` should find.
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

    struct SocketDir(std::path::PathBuf);

    impl SocketDir {
        fn new(name: &str) -> Self {
            static COUNTER: AtomicUsize = AtomicUsize::new(0);
            let dir = std::env::temp_dir().join(format!(
                "klipperx-console-{}-{}-{}",
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

    /// A console attached to a fresh server, with no user at the keyboard.
    async fn console(dir: &SocketDir) -> (Console, tokio::task::JoinHandle<()>) {
        let mut api = Api::new();
        api.register(Echo).unwrap();
        api.register(Failing).unwrap();
        api.register(ListObjects).unwrap();
        api.register(Subscribe).unwrap();
        let server = Server::bind(dir.target(), Arc::new(api))
            .await
            .expect("cannot bind");
        let task = tokio::spawn(async move {
            let _ = server.run().await;
        });
        let console = Console::new(dir.target()).await.expect("cannot connect");
        (console, task)
    }

    // -----------------------------------------------------------------------
    // Typing
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_a_bare_method_is_sent_with_an_id() {
        let dir = SocketDir::new("method");
        let (mut console, task) = console(&dir).await;

        assert!(matches!(
            console.handle_line("echo").await.unwrap(),
            Control::Continue
        ));
        // The id was supplied, so the reply is attributable.
        let reply = console
            .await_reply(console.connection.last_id())
            .await
            .unwrap();
        assert_eq!(reply.unwrap().result(), Some(&json!({"got": null})));

        task.abort();
    }

    #[tokio::test]
    async fn test_a_method_and_parameters_are_parsed() {
        let dir = SocketDir::new("params");
        let (mut console, task) = console(&dir).await;

        console.handle_line("echo {\"value\": 3}").await.unwrap();
        let id = console.connection.last_id();
        let reply = console.await_reply(id).await.unwrap().unwrap();
        assert_eq!(reply.result(), Some(&json!({"got": 3})));

        task.abort();
    }

    #[tokio::test]
    async fn test_bad_parameters_do_not_end_the_session() {
        let dir = SocketDir::new("badparams");
        let (mut console, task) = console(&dir).await;

        let before = console.connection.last_id();
        assert!(matches!(
            console.handle_line("echo [1,2,3]").await.unwrap(),
            Control::Continue
        ));
        assert!(matches!(
            console.handle_line("echo not json").await.unwrap(),
            Control::Continue
        ));
        // Nothing was sent, so the id counter did not move.
        assert_eq!(console.connection.last_id(), before);

        task.abort();
    }

    #[tokio::test]
    async fn test_a_whole_json_object_is_taken_as_written() {
        let dir = SocketDir::new("json");
        let (mut console, task) = console(&dir).await;

        // An explicit id is honoured, not overwritten.
        console
            .handle_line(r#"{"id": 42, "method": "echo", "params": {"value": "x"}}"#)
            .await
            .unwrap();
        let reply = console.await_reply(42).await.unwrap().unwrap();
        assert_eq!(reply.id, json!(42));
        assert_eq!(reply.result(), Some(&json!({"got": "x"})));

        task.abort();
    }

    #[tokio::test]
    async fn test_a_json_object_without_an_id_gets_one() {
        let dir = SocketDir::new("noid");
        let (mut console, task) = console(&dir).await;

        console.handle_line(r#"{"method": "echo"}"#).await.unwrap();
        let id = console.connection.last_id();
        let reply = console.await_reply(id).await.unwrap().unwrap();
        assert_eq!(reply.id, json!(id));

        task.abort();
    }

    #[tokio::test]
    async fn test_a_json_object_with_id_null_is_left_unanswered() {
        let dir = SocketDir::new("nullid");
        let (mut console, task) = console(&dir).await;

        console
            .handle_line(r#"{"id": null, "method": "echo"}"#)
            .await
            .unwrap();
        // Nothing is outstanding, so nothing can be awaited.
        let quiet =
            tokio::time::timeout(Duration::from_millis(100), console.connection.receive()).await;
        assert!(quiet.is_err(), "a null-id request was answered");

        task.abort();
    }

    #[tokio::test]
    async fn test_a_line_without_a_method_is_refused_locally() {
        let dir = SocketDir::new("nomethod");
        let (mut console, task) = console(&dir).await;

        let before = console.connection.last_id();
        console.handle_line(r#"{"params": {}}"#).await.unwrap();
        assert_eq!(console.connection.last_id(), before);

        task.abort();
    }

    // -----------------------------------------------------------------------
    // Local commands
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_local_commands_never_reach_the_server() {
        let dir = SocketDir::new("local");
        let (mut console, task) = console(&dir).await;

        // `.help` and an unknown command are answered by the console itself;
        // `.subscribe` is checked separately because it does talk to the server.
        let before = console.connection.last_id();
        console.handle_line(".help").await.unwrap();
        console.handle_line(".nonsense").await.unwrap();
        assert_eq!(console.connection.last_id(), before);

        assert!(matches!(
            console.handle_line(".quit").await.unwrap(),
            Control::Quit
        ));
        assert!(matches!(
            console.handle_line(".exit").await.unwrap(),
            Control::Quit
        ));

        task.abort();
    }

    #[tokio::test]
    async fn test_subscribe_names_the_requested_objects() {
        let dir = SocketDir::new("sub");
        let (mut console, task) = console(&dir).await;

        console
            .handle_line(".subscribe toolhead extruder")
            .await
            .unwrap();
        let id = console.connection.last_id();
        let reply = console.await_reply(id).await.unwrap().unwrap();

        // `null` per object means every field.
        assert_eq!(
            reply.result(),
            Some(&json!({
                "subscribed": {"toolhead": null, "extruder": null}
            }))
        );

        task.abort();
    }

    #[tokio::test]
    async fn test_subscribe_without_arguments_asks_which_objects_exist() {
        let dir = SocketDir::new("suball");
        let (mut console, task) = console(&dir).await;

        console.handle_line(".subscribe").await.unwrap();
        let id = console.connection.last_id();
        let reply = console.await_reply(id).await.unwrap().unwrap();

        // The pair of calls a real client makes: `objects/list`, then subscribe
        // to what it reported.
        assert_eq!(
            reply.result(),
            Some(&json!({
                "subscribed": {"toolhead": null, "extruder": null}
            }))
        );

        task.abort();
    }

    #[tokio::test]
    async fn test_a_failed_reply_is_reported_as_a_failure() {
        let dir = SocketDir::new("fail");
        let (mut console, task) = console(&dir).await;

        console.handle_line("failing").await.unwrap();
        let id = console.connection.last_id();
        let reply = console.await_reply(id).await.unwrap().unwrap();

        assert!(reply.is_error());
        assert_eq!(reply.error_message(), Some("Invalid Argument Type [value]"));

        task.abort();
    }

    // -----------------------------------------------------------------------
    // Formatting
    // -----------------------------------------------------------------------

    #[test]
    fn test_a_scalar_payload_stays_on_the_label_line() {
        assert_eq!(indent("1 (info) ", &json!(7)), "1 (info) 7");
    }

    #[test]
    fn test_a_structured_payload_goes_below_its_label() {
        let text = indent("2 (objects/query) ", &json!({"state": "ready"}));
        assert_eq!(
            text,
            "2 (objects/query) \n  {\n    \"state\": \"ready\"\n  }"
        );
    }

    #[test]
    fn test_compaction_matches_the_wire() {
        assert_eq!(compact(&json!({"a": 1, "b": [2]})), r#"{"a":1,"b":[2]}"#);
    }
}
