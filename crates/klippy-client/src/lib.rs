//! The client side: talking to an API server.
//!
//! The host serves the API; this module is what connects to it. It exists
//! because nothing else in the tree can: the clients upstream ships
//! (`scripts/whconsole.py`, `scripts/motan/data_logger.py`) only speak Unix
//! sockets and only do what their one job needs, so they cannot exercise a TCP
//! listener, cannot correlate replies, and print pushes as if they were replies.
//!
//! Two subcommands, one connection layer:
//!
//! | Subcommand | What it is |
//! |---|---|
//! | `api` | one request, print the reply, exit — for scripts |
//! | `console` | an interactive session: a window on a terminal, lines in a pipe |
//!
//! The window is [`tui`]; a pipe — or `--plain` — gets [`console`], which prints
//! the same events one line at a time. Both drive the same [`session`], so the
//! difference between them is exactly the difference in how they render.
//!
//! Both take the same `-a/--api-server` value as the host takes, and both reuse
//! the host's parser for it ([`ApiTarget`]), so `unix:` paths and `tcp:` addresses
//! work identically in either direction.
//!
//! # Where the protocol lives
//!
//! The protocol is not re-implemented here. [`connection`] reuses
//! [`klippy_api::protocol`]'s framing and the same `0x03`-delimited JSON the
//! server writes, and the endpoints it calls are the ones in
//! `docs/klippy/third-party-dev/api-reference.md`. A client that disagreed with
//! the server about any of that would only be testing itself.
//!
//! It is a crate of its own, depending on [`klippy_api`] and nothing else of
//! this project, so that this binary does not compile or link the host: no
//! `reqwest`, no `flate2`, no `libloading`. What a client shares with the host is
//! the API, which is exactly what that crate is.

pub mod connection;
pub mod console;
pub mod gcode_params;
#[doc(hidden)]
pub mod gcode_params_scan;
pub mod session;
pub mod tui;

use clap::Args;
use serde_json::{Map, Value};

use klippy_api::address::ApiTarget;

use connection::{Connection, Incoming, Reply};
pub use session::Session;

/// Where the API server is, for the client subcommands.
#[derive(Args, Debug)]
pub struct ApiServerArg {
    /// API server address: a socket path, or `tcp:<host>:<port>`
    #[arg(
        short,
        long,
        value_name = "ADDR",
        default_value = klippy_api::address::DEFAULT_API_SERVER
    )]
    pub api_server: String,
}

/// Send one request and print the reply.
#[derive(Args, Debug)]
pub struct ApiArgs {
    #[command(flatten)]
    pub server: ApiServerArg,

    /// Endpoint path, e.g. `info`, `objects/query`, `gcode/script`
    pub method: String,

    /// Request parameters, as a JSON object
    #[arg(value_name = "PARAMS", default_value = "{}")]
    pub params: String,

    /// Seconds to wait for the reply
    #[arg(long, value_name = "SECS", default_value_t = 10.)]
    pub timeout: f64,
}

/// An interactive session against the API server.
#[derive(Args, Debug)]
pub struct ConsoleArgs {
    #[command(flatten)]
    pub server: ApiServerArg,

    /// Print lines instead of opening a window, even on a terminal
    #[arg(long)]
    pub plain: bool,
}

/// Run `klipperx api`: one request, one reply.
///
/// # Errors
/// Returns an error if the target is malformed, the server cannot be reached,
/// the reply does not arrive in time, or the server answers with an error.
pub fn run_api(args: ApiArgs) -> Result<(), Box<dyn std::error::Error>> {
    let target = parse_target(&args.server.api_server)?;
    let params = parse_params(&args.params)?;
    let timeout = std::time::Duration::from_secs_f64(args.timeout.max(0.001));

    let reply: Result<Reply, Box<dyn std::error::Error>> = runtime()?.block_on(async {
        let wait = async {
            let mut connection = Connection::connect(&target).await?;
            let id = connection.request(&args.method, params).await?;
            loop {
                match connection.receive().await? {
                    // The reply to *this* request, which is the only thing a
                    // one-shot call waits for. Anything else — a reply to
                    // another id, or a push from a subscription this process
                    // never made — is not this process's business.
                    Incoming::Reply(reply) if reply.id == serde_json::json!(id) => {
                        break Ok(reply);
                    }
                    _ => continue,
                }
            }
        };
        // A one-shot call must not hang: without a deadline a server that is up
        // but silent — a stuck endpoint, an MCU that never answers — would leave
        // a script waiting forever.
        match tokio::time::timeout(timeout, wait).await {
            Ok(result) => result,
            Err(_) => Err(format!("no reply to '{}' within {}s", args.method, args.timeout).into()),
        }
    });

    let reply = reply?;
    if reply.is_error() {
        return Err(reply
            .error_message()
            .unwrap_or("the API server reported an error")
            .into());
    }
    println!("{}", serde_json::to_string_pretty(reply.payload())?);
    Ok(())
}

/// Run `klipperx console`: an interactive session.
///
/// A terminal gets the full-screen window; a pipe, or `--plain`, gets one line
/// per event. The window needs both ends to be a terminal — a window drawn into
/// a pipe would be a screenful of escape codes.
///
/// # Errors
/// Returns an error if the target is malformed or the server cannot be reached,
/// and — from the window, where a lost connection has nowhere to be shown after
/// the terminal is restored — if the connection goes away mid-session.
pub fn run_console(args: ConsoleArgs) -> Result<(), Box<dyn std::error::Error>> {
    let target = parse_target(&args.server.api_server)?;
    let windowed = !args.plain && tui::is_available();
    runtime()?.block_on(async move {
        // The window takes a session rather than a target, so that a host can
        // hand it one over a pipe instead — see `klippy::run`.
        if windowed {
            tui::run(target).await
        } else {
            console::run(target).await
        }
    })?;
    Ok(())
}

/// A runtime for a client command.
///
/// Single-threaded on purpose: a client does one thing at a time (send, wait,
/// print), and unlike the host it has no long-lived tasks to keep responsive.
/// The blocking `stdin` read the console does is handled by the runtime's own
/// blocking pool either way.
fn runtime() -> Result<tokio::runtime::Runtime, Box<dyn std::error::Error>> {
    Ok(tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?)
}

/// Parse the shared `--api-server` value.
fn parse_target(value: &str) -> Result<ApiTarget, Box<dyn std::error::Error>> {
    match value.parse::<ApiTarget>() {
        Ok(target) => Ok(target),
        Err(err) => Err(format!("{err}\n\nFor more information, try '--help'.").into()),
    }
}

/// Parse the `PARAMS` argument, which the reference documents as an object.
fn parse_params(value: &str) -> Result<Map<String, Value>, Box<dyn std::error::Error>> {
    let parsed: Value = serde_json::from_str(value).map_err(|err| {
        format!("PARAMS is not JSON: {err}\n\nexpected a JSON object, for example '{{\"objects\": null}}'")
    })?;
    match parsed {
        Value::Object(params) => Ok(params),
        other => Err(format!("PARAMS must be a JSON object, not {}", json_kind(&other)).into()),
    }
}

/// Name a JSON value's kind, for error messages.
fn json_kind(value: &Value) -> &'static str {
    match value {
        Value::Null => "null",
        Value::Bool(_) => "a boolean",
        Value::Number(_) => "a number",
        Value::String(_) => "a string",
        Value::Array(_) => "an array",
        Value::Object(_) => "an object",
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn test_params_are_parsed_as_an_object() {
        assert!(parse_params("{}").unwrap().is_empty());
        assert_eq!(
            parse_params(r#"{"objects": {"toolhead": null}}"#)
                .unwrap()
                .get("objects")
                .unwrap(),
            &json!({"toolhead": null})
        );
    }

    #[test]
    fn test_params_that_are_not_an_object_are_rejected_with_their_kind() {
        assert!(parse_params("[]")
            .unwrap_err()
            .to_string()
            .contains("an array"));
        assert!(parse_params("7")
            .unwrap_err()
            .to_string()
            .contains("a number"));
        assert!(parse_params("not json")
            .unwrap_err()
            .to_string()
            .contains("not JSON"));
    }

    #[test]
    fn test_the_target_is_parsed_the_way_the_host_parses_it() {
        assert_eq!(
            parse_target("/tmp/klippy_uds").unwrap().to_string(),
            "unix:/tmp/klippy_uds"
        );
        assert_eq!(
            parse_target("tcp:127.0.0.1:7125").unwrap().to_string(),
            "tcp:127.0.0.1:7125"
        );
        // The mistake worth catching: Moonraker's HTTP port is not this socket.
        assert!(parse_target("http://127.0.0.1:7125")
            .unwrap_err()
            .to_string()
            .contains("unknown scheme"));
    }
}
