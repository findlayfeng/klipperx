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
//! The parameters and the response shape are defined here; gathering the values
//! is a `todo!()`. Nothing about the wire contract is open:
//!
//! * `state` / `state_message` come from the printer's state message;
//! * `hostname`, the two paths, the three ids, `software_version` and
//!   `cpu_info` come from the host process;
//! * `log_file` is `null` when klippy was started without a log file.
//!
//! ```json
//! {
//!   "id": 1,
//!   "result": {
//!     "state": "ready",
//!     "state_message": "Printer is ready",
//!     "hostname": "klipper",
//!     "klipper_path": "/home/pi/klipper",
//!     "python_path": "/usr/bin/python3",
//!     "process_id": 12345,
//!     "user_id": 1000,
//!     "group_id": 1000,
//!     "log_file": "/tmp/klippy.log",
//!     "config_file": "/home/pi/printer.cfg",
//!     "software_version": "v0.12.0-123-gabcdef",
//!     "cpu_info": "4 core ARMv7 Processor rev 4 (v7l)"
//!   }
//! }
//! ```

use serde::Serialize;
use serde_json::Value;

use crate::core::klippy::api::protocol::{ApiError, Params, Request};
use crate::core::klippy::api::registry::{Endpoint, EndpointContext};

/// The `info` endpoint.
pub struct Info;

impl Endpoint for Info {
    fn path(&self) -> &'static str {
        "info"
    }

    fn handle(&self, request: &Request, _context: &EndpointContext<'_>) -> Result<Value, ApiError> {
        let params = InfoParams::from_request(request)?;

        // TODO: log `params.client_info` as the connection's rollover
        // information, as upstream's `set_client_info` does, so a connection
        // can be told apart in the klippy log.
        //
        // TODO: build the response. The printer's state message supplies
        // `state` and `state_message`; the remaining fields come from the host
        // process (hostname, the klipper and interpreter paths, the process and
        // user/group ids, the version, the log and config paths, and the CPU
        // description). Then return `InfoResponse::into_value`.
        let _ = params;
        todo!("info: gather the printer state and host information")
    }
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
    /// Directory holding the klipper installation.
    pub klipper_path: String,
    /// Interpreter running the host software.
    ///
    /// Upstream reports its own interpreter here; a host software that is not
    /// interpreted reports the path of the executable it runs under.
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
    /// Version of the host software.
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
    use serde_json::json;

    fn request(body: &str) -> Request {
        Request::parse(body.as_bytes()).expect("test body is a valid request")
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

    #[test]
    fn test_the_endpoint_path_is_the_documented_one() {
        assert_eq!(Info.path(), "info");
    }

    #[test]
    fn test_client_info_is_optional() {
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

    #[test]
    fn test_client_info_must_be_an_object() {
        let error = InfoParams::from_request(&request(
            r#"{"method":"info","params":{"client_info":"Moonraker"}}"#,
        ))
        .unwrap_err();
        assert_eq!(
            error,
            ApiError::InvalidArgumentType("client_info".to_string())
        );
    }

    #[test]
    fn test_the_response_carries_every_documented_field() {
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

    #[test]
    fn test_a_missing_log_file_is_null_not_absent() {
        // Upstream always sends the key; only its value can be null.
        let response = InfoResponse {
            log_file: None,
            ..sample()
        };
        let value = response.into_value();
        assert!(value.as_object().unwrap().contains_key("log_file"));
        assert_eq!(value["log_file"], Value::Null);
    }

    #[test]
    fn test_the_response_never_echoes_client_info() {
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
