//! `info` — printer state and host information.
//!
//! The first request every client makes: it establishes which host it is
//! talking to, what version is running, and whether the printer is ready. It
//! takes no required parameters, and the only optional one, `client_info`, is
//! not answered — it is recorded so the klippy log says who connected.
//!
//! ```json
//! {"id": 1, "method": "info", "params": {"client_info": {"name": "Moonraker"}}}
//! ```
//!
//! # Status
//!
//! Written and registered (see [`super::super::register`]). The values come from two
//! places:
//!
//! * `state` / `state_message` from the printer's state message;
//! * `hostname`, the two paths, the three ids, `software_version` and
//!   `cpu_info` from the host process, the last two through [`StartArgs`].
//!
//! `log_file` is `null` when the host is not logging to a file, which it never
//! is yet: the host logs to stdout.
//!
//! ```json
//! {
//!   "id": 1,
//!   "result": {
//!     "state": "ready",
//!     "state_message": "Printer is ready",
//!     "hostname": "klipper",
//!     "klipper_path": "/nonexistent/klipper",
//!     "python_path": "/nonexistent/python3",
//!     "process_id": 12345,
//!     "user_id": 1000,
//!     "group_id": 1000,
//!     "log_file": null,
//!     "config_file": "/home/pi/printer.cfg",
//!     "software_version": "v0.13.0-734-gfe4eb86-0.1.0",
//!     "cpu_info": "4 core ARMv7 Processor rev 4 (v7l)"
//!   }
//! }
//! ```

use std::sync::Arc;

use serde::Serialize;
use serde_json::Value;
use tracing::info;

use crate::core::klippy::api::protocol::{ApiError, Params, Request};
use crate::core::klippy::api::registry::{Endpoint, EndpointContext, EndpointFuture};
use crate::core::klippy::api::start_args::StartArgs;
use crate::core::klippy::api::{Api, ApiWiring, RegistrationError};
use crate::core::klippy::printer::Printer;

endpoint!(install);

/// Install the `info` endpoint.
pub(crate) fn install(api: &mut Api, wiring: &ApiWiring<'_>) -> Result<(), RegistrationError> {
    api.register(Info::new(
        Arc::clone(wiring.printer),
        wiring.start_args.clone(),
    ))
    .map_err(RegistrationError::Endpoint)
}

/// The sentinel test fixtures pass for `klipper_path`.
///
/// Both path keys must always be present strings in an `info` response:
/// Moonraker indexes them without a default
/// (`moonraker/components/klippy_connection.py`, `_save_path_info`), and the
/// resulting `KeyError` is not a `ServerError`, so it is not caught by the
/// `except ServerError` around the initialization task
/// (`klippy_connection.py:418-427`): that task dies with nobody awaiting it,
/// the socket stays connected, `klippy_state` stays `startup` forever and no
/// reconnect follows.
///
/// In production `klipper_path` is the released asset root the host gathered
/// at start-up (see [`InfoResponse::klipper_path`]); a test fixture that does
/// not release one passes this nonexistent path instead.
#[cfg(test)]
pub(crate) const NO_KLIPPER_PATH: &str = "/nonexistent/klipper";

/// The interpreter this host does not have. See [`NO_KLIPPER_PATH`].
const NO_PYTHON_PATH: &str = "/nonexistent/python3";

/// The `info` endpoint.
pub struct Info {
    printer: Arc<Printer>,
    start_args: StartArgs,
}

impl Info {
    /// Build the endpoint over the machine whose state it reports and the
    /// arguments this host was started with.
    pub fn new(printer: Arc<Printer>, start_args: StartArgs) -> Self {
        Self {
            printer,
            start_args,
        }
    }

    /// The response: the printer's state, and the host's own facts.
    fn response(&self) -> InfoResponse {
        let state = self.printer.get_state_message();
        InfoResponse {
            state: state.category.as_category().to_string(),
            state_message: state.message,
            hostname: hostname(),
            klipper_path: self.start_args.klipper_path.clone(),
            python_path: NO_PYTHON_PATH.to_string(),
            process_id: std::process::id(),
            user_id: current_user_id(),
            group_id: current_group_id(),
            log_file: self.start_args.log_file.clone(),
            config_file: self.start_args.config_file.clone(),
            software_version: self.start_args.software_version.clone(),
            cpu_info: self.start_args.cpu_info.clone(),
        }
    }
}

impl Endpoint for Info {
    fn path(&self) -> &'static str {
        "info"
    }

    fn handle<'a>(
        &'a self,
        request: &'a Request,
        _context: &'a EndpointContext<'a>,
    ) -> EndpointFuture<'a> {
        Box::pin(async move {
            let params = InfoParams::from_request(request)?;

            // Upstream records `client_info` on the connection so an analysed
            // shutdown can print who was connected (`WebRequest.set_client_info`);
            // this host keeps no per-connection record yet, so the identity is
            // logged and otherwise ignored.
            if let Some(client_info) = &params.client_info {
                info!("Client info: {client_info}");
            }

            Ok(self.response().into_value())
        })
    }
}

/// The host name, as upstream's `socket.gethostname()` reports it.
fn hostname() -> String {
    let mut buf = [0 as libc::c_char; 256];
    // SAFETY: `buf` is a writable buffer of exactly the length passed, which is
    // what `gethostname` requires.
    let result = unsafe { libc::gethostname(buf.as_mut_ptr(), buf.len()) };
    if result != 0 {
        return "?".to_string();
    }
    // The name is NUL-terminated unless it was truncated to fit; taking the
    // bytes up to the first NUL (or the whole buffer) covers both.
    let bytes: Vec<u8> = buf
        .iter()
        .take_while(|byte| **byte != 0)
        .map(|byte| *byte as u8)
        .collect();
    String::from_utf8_lossy(&bytes).into_owned()
}

/// The effective user id, as upstream's `os.getuid()` reports it.
fn current_user_id() -> u32 {
    // SAFETY: `getuid` takes no arguments and cannot fail.
    unsafe { libc::getuid() }
}

/// The effective group id, as upstream's `os.getgid()` reports it.
fn current_group_id() -> u32 {
    // SAFETY: `getgid` takes no arguments and cannot fail.
    unsafe { libc::getgid() }
}

/// The `info` request parameters.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct InfoParams {
    /// Client identity, for the log only. Never echoed in the response.
    ///
    /// Upstream accepts any value here and logs its `repr`; this keeps the raw
    /// [`Value`] for the same reason, and only rejects a non-object so that a
    /// client sending nonsense is told rather than silently ignored.
    pub client_info: Option<Value>,
}

impl InfoParams {
    /// Read the optional `client_info` parameter.
    ///
    /// # Errors
    /// Returns [`ApiError::InvalidArgumentType`] if `client_info` is present
    /// but is not an object.
    pub fn from_request(request: &Request) -> Result<Self, ApiError> {
        Self::from_params(&request.params())
    }

    /// Read the optional `client_info` parameter from decoded parameters.
    ///
    /// # Errors
    /// As [`InfoParams::from_request`].
    pub fn from_params(params: &Params<'_>) -> Result<Self, ApiError> {
        let client_info = match params.get_opt("client_info") {
            None => None,
            Some(value @ Value::Object(_)) => Some(value.clone()),
            Some(_) => return Err(ApiError::InvalidArgumentType("client_info".to_string())),
        };
        Ok(Self { client_info })
    }
}

/// The `info` response.
///
/// Field names and types are the wire contract; the reference documentation
/// lists them in the same order. `log_file` is the only optional field —
/// upstream omits the key entirely when klippy runs without a log file, which
/// serializes as `null` here.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct InfoResponse {
    /// Printer state category: `startup`, `ready`, `shutdown` or `error`.
    pub state: String,
    /// Human-readable description of that state.
    pub state_message: String,
    /// Host name of the machine running klippy.
    pub hostname: String,
    /// Directory of the Klipper assets this host releases at start-up.
    ///
    /// The root the embedded trees were written to ([`crate::assets::release`]):
    /// `config/` (the upstream configuration examples), `docs/` (this host's
    /// user manuals plus a generated index) and `docs/klipper/` (the upstream
    /// documentation, at the subpath Moonraker's documentation lookup expects
    /// under this path). Upstream reports its own checkout here
    /// (`klippy/webhooks.py:390`); this host reports what it carries.
    pub klipper_path: String,
    /// Interpreter running the host software, or a path that does not exist.
    ///
    /// Upstream reports its own interpreter here, and this field is the one
    /// part of the response that is both Klipper-specific and load-bearing:
    /// **Moonraker is its only consumer**, and it reads it as the *virtualenv*
    /// of the Klipper installation — `env` in its update manager, from which it
    /// derives `<venv>/bin/python` and runs `-m pip` with it
    /// (`update_manager/app_deploy.py`, `_configure_virtualenv`). Three shapes
    /// of value matter there:
    ///
    /// * a real `<venv>/bin/python` makes Moonraker manage the Klipper repo;
    /// * an existing executable that is *not* in a virtualenv makes it fail to
    ///   start (`Invalid virtualenv at path …`, because `<parent>/bin/activate`
    ///   is missing) — so this host must **not** report its own binary here;
    /// * a path that does not exist leaves the Klipper updater a no-op
    ///   (`update_manager.py`: the deploy class is only upgraded when the path
    ///   exists), which is what a host with no Klipper checkout wants.
    ///
    /// The key must still be present: Moonraker indexes
    /// `self._klippy_info["python_path"]` without a default
    /// (`klippy_connection.py`, `_save_path_info`). Frontends do not read it
    /// at all, and Moonraker's own docs mark it "moonraker use only".
    pub python_path: String,
    /// Process id of the host software.
    pub process_id: u32,
    /// User id the host software runs as.
    pub user_id: u32,
    /// Group id the host software runs as.
    pub group_id: u32,
    /// Log file path, or `null` when not logging to a file.
    pub log_file: Option<String>,
    /// Configuration file the host software was started with.
    pub config_file: String,
    /// Version of the host software: the pinned upstream baseline plus this
    /// crate's version ([`crate::version::software_version`]).
    pub software_version: String,
    /// CPU description, e.g. `"4 core ARMv7 Processor rev 4 (v7l)"`.
    ///
    /// A plain string, not an object: clients display it verbatim.
    pub cpu_info: String,
}

impl InfoResponse {
    /// Serialize for the reply payload.
    pub fn into_value(&self) -> Value {
        serde_json::to_value(self).expect("an InfoResponse always serializes")
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::registry::Api;
    use crate::core::klippy::api::test_support::{context, silent_target};
    use crate::core::klippy::api::WebhooksStatus;
    use crate::core::klippy::printer::PrinterObject;
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::json;

    fn request(body: &str) -> Request {
        Request::parse(body.as_bytes()).expect("test body is a valid request")
    }

    /// The start arguments a host would have gathered.
    fn start_args() -> StartArgs {
        StartArgs {
            config_file: "/home/pi/printer.cfg".to_string(),
            log_file: None,
            klipper_path: NO_KLIPPER_PATH.to_string(),
            software_version: "0.1.0".to_string(),
            cpu_info: "4 core ARMv7 Processor rev 4 (v7l)".to_string(),
            apiserver: None,
            start_reason: "startup".to_string(),
            debug_output: None,
            device: "Raspberry Pi 4 Model B".to_string(),
            linux_version: "Linux version 6.1.0".to_string(),
        }
    }

    fn endpoint() -> (Info, Arc<Printer>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        (Info::new(Arc::clone(&printer), start_args()), printer)
    }

    /// Run one `info` request through the endpoint.
    async fn ask(endpoint: &Info, body: &str) -> Value {
        let api = Api::new();
        endpoint
            .handle(&request(body), &context(&api, silent_target()))
            .await
            .unwrap()
    }

    fn sample() -> InfoResponse {
        InfoResponse {
            state: "ready".to_string(),
            state_message: "Printer is ready".to_string(),
            hostname: "klipper".to_string(),
            klipper_path: "/home/pi/klipper".to_string(),
            python_path: "/usr/bin/python3".to_string(),
            process_id: 12345,
            user_id: 1000,
            group_id: 1000,
            log_file: Some("/tmp/klippy.log".to_string()),
            config_file: "/home/pi/printer.cfg".to_string(),
            software_version: "v0.12.0-123-gabcdef".to_string(),
            cpu_info: "4 core ARMv7 Processor rev 4 (v7l)".to_string(),
        }
    }

    #[tokio::test]
    async fn test_the_endpoint_path_is_the_documented_one() {
        let (endpoint, _printer) = endpoint();
        assert_eq!(endpoint.path(), "info");
    }

    #[tokio::test]
    async fn test_the_handler_reports_the_printers_state() {
        let (endpoint, printer) = endpoint();

        let response = ask(&endpoint, r#"{"method":"info"}"#).await;
        assert_eq!(response["state"], "startup");
        assert_eq!(response["state_message"], "Starting up");

        printer.invoke_shutdown("Printer is halted");

        let response = ask(&endpoint, r#"{"method":"info"}"#).await;
        assert_eq!(response["state"], "shutdown");
        assert_eq!(response["state_message"], "Printer is halted");
    }

    #[tokio::test]
    async fn test_the_handler_reports_the_host_arguments() {
        let (endpoint, _printer) = endpoint();

        let response = ask(&endpoint, r#"{"method":"info"}"#).await;

        assert_eq!(response["config_file"], "/home/pi/printer.cfg");
        assert_eq!(response["software_version"], "0.1.0");
        assert_eq!(response["cpu_info"], "4 core ARMv7 Processor rev 4 (v7l)");
        assert_eq!(response["log_file"], Value::Null);
        assert_eq!(response["process_id"], std::process::id());
    }

    #[tokio::test]
    async fn test_klipper_path_is_the_start_args_value_and_python_stays_absent() {
        // See `NO_KLIPPER_PATH`: Moonraker indexes both keys without a default
        // and only sets up its Klipper updater when both paths exist — which
        // this host keeps a no-op by reporting no interpreter, while
        // `klipper_path` is the released asset root the start args carry.
        let (endpoint, _printer) = endpoint();

        let response = ask(&endpoint, r#"{"method":"info"}"#).await;

        assert_eq!(response["klipper_path"], NO_KLIPPER_PATH);
        // `python_path` may not name something this host actually has: a real
        // interpreter or a virtualenv would make Moonraker manage it (see
        // `InfoResponse::python_path`), so pin that it is neither this crate's
        // source tree nor the running binary — the two "real path" values a
        // future edit is most likely to reach for.
        let manifest_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let executable = std::env::current_exe().expect("the test binary has a path");
        let path = response["python_path"].as_str().expect("a string");
        assert!(!path.is_empty(), "python_path is empty");
        assert!(
            !std::path::Path::new(path).exists(),
            "python_path exists: {path}"
        );
        assert_ne!(
            std::path::Path::new(path),
            manifest_dir,
            "python_path is this crate"
        );
        assert_ne!(
            std::path::Path::new(path),
            executable,
            "python_path is this binary"
        );
    }

    #[tokio::test]
    async fn test_the_state_is_one_moonraker_accepts_and_webhooks_agrees() {
        // Moonraker maps the reported state through its own enum and only knows
        // `startup|ready|error|shutdown`: `KlippyState.from_string` raises
        // `ValueError` for anything else (`moonraker/common.py:137-151`), from
        // call sites that are not behind a `ServerError` guard
        // (`klippy_connection.py:456,583`) — the host would sit in `startup`
        // with the socket connected. `info` and `webhooks` both report the
        // state, so both must draw from that same four-name domain and agree on
        // the value.
        let (endpoint, printer) = endpoint();
        let webhooks = WebhooksStatus::new(Arc::clone(&printer));
        let accepted = ["startup", "ready", "error", "shutdown"];

        for halted in [false, true] {
            if halted {
                printer.invoke_shutdown("Printer is halted");
            }
            let info = ask(&endpoint, r#"{"method":"info"}"#).await;
            let info_state = info["state"].as_str().expect("a string").to_string();
            let webhooks_state = webhooks.get_status(0.0)["state"]
                .as_str()
                .expect("a string")
                .to_string();

            assert_eq!(info_state, webhooks_state, "the two state reports differ");
            assert!(
                accepted.contains(&info_state.as_str()),
                "{info_state} is not a state Moonraker accepts"
            );
        }
    }

    #[tokio::test]
    async fn test_client_info_is_logged_but_never_answered() {
        let (endpoint, _printer) = endpoint();

        let response = ask(
            &endpoint,
            r#"{"method":"info","params":{"client_info":{"name":"Moonraker"}}}"#,
        )
        .await;

        assert!(!response.as_object().unwrap().contains_key("client_info"));
    }

    #[tokio::test]
    async fn test_client_info_is_optional() {
        assert_eq!(
            InfoParams::from_request(&request(r#"{"method":"info"}"#)).unwrap(),
            InfoParams::default()
        );
        assert_eq!(
            InfoParams::from_request(&request(
                r#"{"method":"info","params":{"client_info":{"name":"Moonraker"}}}"#
            ))
            .unwrap()
            .client_info,
            Some(json!({"name": "Moonraker"}))
        );
    }

    #[tokio::test]
    async fn test_client_info_must_be_an_object() {
        let error = InfoParams::from_request(&request(
            r#"{"method":"info","params":{"client_info":"Moonraker"}}"#,
        ))
        .unwrap_err();
        assert_eq!(
            error,
            ApiError::InvalidArgumentType("client_info".to_string())
        );
    }

    #[tokio::test]
    async fn test_the_response_carries_every_documented_field() {
        assert_eq!(
            sample().into_value(),
            json!({
                "state": "ready",
                "state_message": "Printer is ready",
                "hostname": "klipper",
                "klipper_path": "/home/pi/klipper",
                "python_path": "/usr/bin/python3",
                "process_id": 12345,
                "user_id": 1000,
                "group_id": 1000,
                "log_file": "/tmp/klippy.log",
                "config_file": "/home/pi/printer.cfg",
                "software_version": "v0.12.0-123-gabcdef",
                "cpu_info": "4 core ARMv7 Processor rev 4 (v7l)"
            })
        );
    }

    #[tokio::test]
    async fn test_a_missing_log_file_is_null_not_absent() {
        // Upstream always sends the key; only its value can be null.
        let response = InfoResponse {
            log_file: None,
            ..sample()
        };
        let value = response.into_value();
        assert!(value.as_object().unwrap().contains_key("log_file"));
        assert_eq!(value["log_file"], Value::Null);
    }

    #[tokio::test]
    async fn test_the_response_never_echoes_client_info() {
        let keys: Vec<String> = sample()
            .into_value()
            .as_object()
            .unwrap()
            .keys()
            .cloned()
            .collect();
        assert!(!keys.contains(&"client_info".to_string()));
        assert_eq!(keys.len(), 12);
    }
}
