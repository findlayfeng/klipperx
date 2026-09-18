//! Wire protocol — the `0x03`-framed JSON that clients speak.
//!
//! Everything in this module is transport-independent: it frames a byte stream,
//! decodes requests, reads parameters, and shapes replies. Nothing here knows
//! about sockets, printer objects, or what any endpoint does — [`Api`] in the
//! parent module joins the two.
//!
//! # Framing
//!
//! A message is a JSON object followed by one `0x03` byte. The delimiter is the
//! only length prefix the protocol has, so a reader cannot assume a `recv`
//! returns whole messages: [`Framing`] buffers bytes until a delimiter shows up
//! and hands back only complete bodies.
//!
//! # Replies
//!
//! [`Request::respond`] is the single place that decides whether a reply is
//! sent at all. Upstream, a request without an `id` is dropped silently, and no
//! endpoint can override that — keeping the rule in one function means no
//! handler can accidentally answer a fire-and-forget request.
//!
//! [`Api`]: crate::core::klippy::api::Api

use serde::Serialize;
use serde_json::{Map, Value};
use std::fmt;

/// Message delimiter: ASCII `ETX`.
///
/// Every JSON body on the socket is followed by this byte, in both directions.
pub const DELIMITER: u8 = 0x03;

// ===========================================================================
// Framing
// ===========================================================================

/// Splits the socket's byte stream into complete messages.
///
/// Bytes that do not contain a delimiter yet are held as [`Framing::partial`];
/// everything before a delimiter is returned as a body. Bodies are returned
/// untouched — not even validated as JSON — so a malformed message is reported
/// per message instead of desynchronising the stream.
///
/// An empty body (two delimiters in a row, or a leading delimiter) is returned
/// as an empty slice of bytes, exactly as upstream sees it: it fails to decode
/// and is dropped.
///
/// # Example
///
/// ```ignore
/// let mut framing = Framing::new();
/// assert_eq!(framing.push(b"{\"a\":1}\x03{\"b\""), vec![b"{\"a\":1}".to_vec()]);
/// assert_eq!(framing.partial(), b"{\"b\"");
/// ```
#[derive(Debug, Default)]
pub struct Framing {
    partial: Vec<u8>,
}

impl Framing {
    /// Create an empty framer.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add `chunk` to the stream and return every message it completed.
    ///
    /// The returned bodies are in arrival order and exclude the delimiters.
    pub fn push(&mut self, chunk: &[u8]) -> Vec<Vec<u8>> {
        self.partial.extend_from_slice(chunk);

        let mut bodies = Vec::new();
        let mut start = 0;
        while let Some(offset) = self.partial[start..]
            .iter()
            .position(|byte| *byte == DELIMITER)
        {
            let end = start + offset;
            bodies.push(self.partial[start..end].to_vec());
            start = end + 1;
        }
        self.partial.drain(..start);
        bodies
    }

    /// Bytes received so far that do not yet form a complete message.
    pub fn partial(&self) -> &[u8] {
        &self.partial
    }
}

/// Serialize `message` and append the delimiter, ready to be written.
///
/// Compaction matches the protocol's single-line framing; a
/// [`serde_json::Value`] has no serializer that can fail, so this is infallible.
pub fn encode(message: &Value) -> Vec<u8> {
    let mut body = serde_json::to_vec(message).expect("a serde_json::Value always serializes");
    body.push(DELIMITER);
    body
}

// ===========================================================================
// Requests
// ===========================================================================

/// A request body that is not a valid request.
///
/// This is *not* an API error: upstream logs these and drops them without a
/// reply, because a body that cannot be decoded cannot be trusted to name a
/// method or an `id` to answer. A client that sends garbage therefore never
/// receives an `error` response for it, and the connection stays open.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MalformedRequest(String);

impl MalformedRequest {
    /// Describe why the body was rejected.
    pub fn new(message: impl Into<String>) -> Self {
        Self(message.into())
    }

    /// The reason, without the framing context.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for MalformedRequest {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for MalformedRequest {}

/// A decoded client request.
///
/// The `id` is kept as an untyped [`Value`] because the protocol does not
/// constrain it: clients are expected to get back exactly what they sent, and
/// Moonraker uses strings while others use integers.
#[derive(Debug, Clone, PartialEq)]
pub struct Request {
    id: Option<Value>,
    method: String,
    params: Map<String, Value>,
}

impl Request {
    /// Decode one framed message body.
    ///
    /// # Errors
    ///
    /// Returns [`MalformedRequest`] when the body is not JSON, is not an
    /// object, has no string `method`, or has a `params` that is not an object.
    /// A missing `params` is not an error: it means the empty parameter set.
    pub fn parse(body: &[u8]) -> Result<Self, MalformedRequest> {
        let value: Value = serde_json::from_slice(body)
            .map_err(|err| MalformedRequest::new(format!("invalid JSON: {err}")))?;
        let object = value
            .as_object()
            .ok_or_else(|| MalformedRequest::new("not a JSON object"))?;

        let method = object
            .get("method")
            .and_then(Value::as_str)
            .ok_or_else(|| MalformedRequest::new("missing or non-string 'method'"))?
            .to_string();

        // `id: null` and an absent `id` are the same thing: no reply wanted.
        let id = match object.get("id") {
            None | Some(Value::Null) => None,
            Some(id) => Some(id.clone()),
        };

        let params = match object.get("params") {
            None => Map::new(),
            Some(Value::Object(params)) => params.clone(),
            Some(_) => return Err(MalformedRequest::new("'params' is not an object")),
        };

        Ok(Self { id, method, params })
    }

    /// The identifier to echo back, if the client asked for a reply.
    pub fn id(&self) -> Option<&Value> {
        self.id.as_ref()
    }

    /// The endpoint path this request is addressed to.
    pub fn method(&self) -> &str {
        &self.method
    }

    /// A read-only view over the request's parameters.
    pub fn params(&self) -> Params<'_> {
        Params::new(&self.params)
    }

    /// Whether the client expects a reply.
    ///
    /// False for fire-and-forget requests, which are never answered.
    pub fn expects_reply(&self) -> bool {
        self.id.is_some()
    }

    /// Turn a handler outcome into the reply to send, if any.
    ///
    /// Returns `None` for a request without an `id`: those are answered with
    /// silence, including when the handler failed. A handler that produced no
    /// result at all still gets a reply, because an endpoint that returns
    /// `Ok(Value::Null)` is answering "nothing to report" — upstream's
    /// equivalent is the empty object it sends when no error was set.
    pub fn respond(&self, outcome: Result<Value, ApiError>) -> Option<Response> {
        let id = self.id.clone()?;
        Some(match outcome {
            Ok(result) => Response::Result { id, result },
            Err(error) => Response::Error {
                id,
                error: error.body(),
            },
        })
    }
}

// ===========================================================================
// Replies
// ===========================================================================

/// A reply to a request that carried an `id`.
///
/// Serialized as a single JSON object with the `id` first, so the wire shape
/// matches the reference documentation exactly.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(untagged)]
pub enum Response {
    /// `{"id": …, "result": …}`
    Result { id: Value, result: Value },
    /// `{"id": …, "error": {"error": "WebRequestError", "message": …}}`
    Error { id: Value, error: ApiErrorBody },
}

/// The `error` object of a failed request.
///
/// The `error` field is always `"WebRequestError"`: one error type covers every
/// request-level failure, exactly as upstream does.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct ApiErrorBody {
    error: &'static str,
    message: String,
}

impl ApiErrorBody {
    /// The only error name the API reports.
    pub const KIND: &'static str = "WebRequestError";

    fn new(message: String) -> Self {
        Self {
            error: Self::KIND,
            message,
        }
    }

    /// The error name, as it appears on the wire.
    pub fn error(&self) -> &'static str {
        self.error
    }

    /// The human-readable message.
    pub fn message(&self) -> &str {
        &self.message
    }
}

/// A request-level failure, reported to the client as an `error` reply.
///
/// The messages are kept byte-for-byte identical to upstream's, because
/// Moonraker surfaces them to users and tools grep them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ApiError {
    /// A required parameter was not supplied: `Missing Argument [x]`.
    MissingArgument(String),
    /// A parameter was supplied with the wrong JSON type:
    /// `Invalid Argument Type [x]`.
    InvalidArgumentType(String),
    /// A parameter was well-typed but not acceptable: `Invalid argument`.
    InvalidArgument,
    /// The method named no registered endpoint.
    UnknownEndpoint(String),
    /// A mux endpoint's key parameter named no registered instance.
    UnknownMuxValue { key: String, value: String },
    /// A host-side caller asked for a remote method nobody registered.
    ///
    /// Raised by [`Api::call_remote_method`](super::Api::call_remote_method),
    /// not by an endpoint.
    RemoteMethodNotRegistered(String),
    /// Every connection registered for a remote method had gone away.
    ///
    /// Raised by [`Api::call_remote_method`](super::Api::call_remote_method),
    /// not by an endpoint.
    NoActiveConnections(String),
    /// A handler failed in a way it did not expect.
    ///
    /// Upstream logs these and shuts klippy down as well as answering the
    /// client; the server owns that decision, this variant only carries the
    /// text.
    Internal(String),
}

impl ApiError {
    /// The `error` object to send for this failure.
    pub fn body(&self) -> ApiErrorBody {
        ApiErrorBody::new(self.to_string())
    }

    /// Whether this failure should also take klippy down.
    ///
    /// Only [`ApiError::Internal`] does: the other variants mean the client
    /// asked for something that does not exist, which is not the printer's
    /// problem.
    pub fn is_internal(&self) -> bool {
        matches!(self, ApiError::Internal(_))
    }
}

impl fmt::Display for ApiError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ApiError::MissingArgument(item) => write!(f, "Missing Argument [{item}]"),
            ApiError::InvalidArgumentType(item) => write!(f, "Invalid Argument Type [{item}]"),
            ApiError::InvalidArgument => f.write_str("Invalid argument"),
            // The `webhooks:` prefix is upstream's, kept verbatim so clients
            // that match on the text keep working.
            ApiError::UnknownEndpoint(path) => {
                write!(f, "webhooks: No registered callback for path '{path}'")
            }
            ApiError::UnknownMuxValue { key, value } => {
                write!(f, "The value '{value}' is not valid for {key}")
            }
            ApiError::RemoteMethodNotRegistered(method) => {
                write!(f, "Remote method '{method}' not registered")
            }
            ApiError::NoActiveConnections(method) => {
                write!(f, "No active connections for method '{method}'")
            }
            ApiError::Internal(message) => f.write_str(message),
        }
    }
}

impl std::error::Error for ApiError {}

// ===========================================================================
// Parameters
// ===========================================================================

/// A read-only view over a request's parameters.
///
/// Accessors mirror upstream's `WebRequest.get_*` family: a missing key is
/// [`ApiError::MissingArgument`], a key holding the wrong JSON type is
/// [`ApiError::InvalidArgumentType`], and a key holding the wrong *value* is
/// left to the caller.
///
/// Use [`Params::get_opt`] for optional parameters — the `Option`/`Result`
/// split is how optionality is expressed here, rather than a sentinel default.
#[derive(Debug, Clone, Copy)]
pub struct Params<'a> {
    values: &'a Map<String, Value>,
}

impl<'a> Params<'a> {
    /// Wrap a decoded parameter object.
    pub fn new(values: &'a Map<String, Value>) -> Self {
        Self { values }
    }

    /// Number of parameters supplied.
    pub fn len(&self) -> usize {
        self.values.len()
    }

    /// Whether no parameters were supplied.
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    /// Whether `item` was supplied at all.
    pub fn has(&self, item: &str) -> bool {
        self.values.contains_key(item)
    }

    /// The parameters that were supplied.
    pub fn names(&self) -> impl Iterator<Item = &'a str> {
        self.values.keys().map(String::as_str)
    }

    /// The raw value of `item`, or `None` if it was not supplied.
    pub fn get_opt(&self, item: &str) -> Option<&'a Value> {
        self.values.get(item)
    }

    /// The raw value of `item`.
    ///
    /// # Errors
    /// Returns [`ApiError::MissingArgument`] if `item` was not supplied.
    pub fn get(&self, item: &str) -> Result<&'a Value, ApiError> {
        self.get_opt(item)
            .ok_or_else(|| ApiError::MissingArgument(item.to_string()))
    }

    /// The value of `item`, or `default` if it was not supplied.
    pub fn get_or(&self, item: &str, default: &'a Value) -> &'a Value {
        self.get_opt(item).unwrap_or(default)
    }

    /// The string value of `item`.
    ///
    /// # Errors
    /// Returns [`ApiError::MissingArgument`] if absent, or
    /// [`ApiError::InvalidArgumentType`] if it is not a string.
    pub fn get_str(&self, item: &str) -> Result<&'a str, ApiError> {
        match self.get(item)? {
            Value::String(text) => Ok(text.as_str()),
            _ => Err(ApiError::InvalidArgumentType(item.to_string())),
        }
    }

    /// The integer value of `item`.
    ///
    /// # Errors
    /// Returns [`ApiError::MissingArgument`] if absent, or
    /// [`ApiError::InvalidArgumentType`] if it is not an integer. JSON booleans
    /// are not integers here, even though they are in upstream's Python.
    pub fn get_int(&self, item: &str) -> Result<i64, ApiError> {
        match self.get(item)? {
            Value::Number(number) => number
                .as_i64()
                .ok_or_else(|| ApiError::InvalidArgumentType(item.to_string())),
            _ => Err(ApiError::InvalidArgumentType(item.to_string())),
        }
    }

    /// The numeric value of `item`, accepting integers and floats.
    ///
    /// # Errors
    /// Returns [`ApiError::MissingArgument`] if absent, or
    /// [`ApiError::InvalidArgumentType`] if it is not a number.
    pub fn get_float(&self, item: &str) -> Result<f64, ApiError> {
        match self.get(item)? {
            Value::Number(number) => number
                .as_f64()
                .ok_or_else(|| ApiError::InvalidArgumentType(item.to_string())),
            _ => Err(ApiError::InvalidArgumentType(item.to_string())),
        }
    }

    /// The boolean value of `item`.
    ///
    /// # Errors
    /// Returns [`ApiError::MissingArgument`] if absent, or
    /// [`ApiError::InvalidArgumentType`] if it is not a boolean.
    pub fn get_bool(&self, item: &str) -> Result<bool, ApiError> {
        match self.get(item)? {
            Value::Bool(value) => Ok(*value),
            _ => Err(ApiError::InvalidArgumentType(item.to_string())),
        }
    }

    /// The object value of `item`.
    ///
    /// # Errors
    /// Returns [`ApiError::MissingArgument`] if absent, or
    /// [`ApiError::InvalidArgumentType`] if it is not an object.
    pub fn get_dict(&self, item: &str) -> Result<&'a Map<String, Value>, ApiError> {
        match self.get(item)? {
            Value::Object(object) => Ok(object),
            _ => Err(ApiError::InvalidArgumentType(item.to_string())),
        }
    }

    /// The array value of `item`.
    ///
    /// # Errors
    /// Returns [`ApiError::MissingArgument`] if absent, or
    /// [`ApiError::InvalidArgumentType`] if it is not an array.
    pub fn get_array(&self, item: &str) -> Result<&'a Vec<Value>, ApiError> {
        match self.get(item)? {
            Value::Array(array) => Ok(array),
            _ => Err(ApiError::InvalidArgumentType(item.to_string())),
        }
    }
}

// ===========================================================================
// Pushes
// ===========================================================================

/// The connection a request arrived on, as far as an endpoint may use it.
///
/// Endpoints that push (subscriptions, the `*/dump_*` mux endpoints, remote
/// methods) need the connection, but they must not need the socket: this trait
/// is the whole of what they get, which keeps the endpoint layer testable
/// without a Unix Domain Socket.
pub trait PushTarget: Send + Sync {
    /// Whether the connection is gone.
    ///
    /// A closed target must be dropped from any registration list instead of
    /// being pushed to.
    fn is_closed(&self) -> bool;

    /// Queue `message` for delivery.
    fn push(&self, message: Value);
}

/// The `response_template` a client may pass when registering for pushes.
///
/// The template supplies whatever the client's parser needs around the
/// payload — usually `{"id": null, "method": "printer:status"}` so that a push
/// is indistinguishable from a notification. The server merges the payload in
/// as `params` and keeps every other field of the template, which is exactly
/// upstream's `out = {"params": kwargs}; out.update(template)`.
///
/// That merge order means a template carrying its own `params` field wins over
/// the payload. That is upstream's behaviour, kept for compatibility, and it is
/// why a template should carry `id`/`method` and nothing else.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ResponseTemplate(Map<String, Value>);

impl ResponseTemplate {
    /// Wrap an already-decoded template object.
    pub fn new(template: Map<String, Value>) -> Self {
        Self(template)
    }

    /// Read the optional `response_template` parameter of a request.
    ///
    /// # Errors
    /// Returns [`ApiError::InvalidArgumentType`] if the parameter is present
    /// but is not an object.
    pub fn from_params(params: &Params<'_>) -> Result<Self, ApiError> {
        match params.get_opt("response_template") {
            None => Ok(Self::default()),
            Some(Value::Object(template)) => Ok(Self(template.clone())),
            Some(_) => Err(ApiError::InvalidArgumentType(
                "response_template".to_string(),
            )),
        }
    }

    /// The fields the client wants around every push.
    pub fn as_map(&self) -> &Map<String, Value> {
        &self.0
    }

    /// Build one pushed message carrying `params`.
    pub fn message(&self, params: Value) -> Value {
        let mut message = Map::new();
        message.insert("params".to_string(), params);
        for (key, value) in &self.0 {
            message.insert(key.clone(), value.clone());
        }
        Value::Object(message)
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

    // -----------------------------------------------------------------------
    // Framing
    // -----------------------------------------------------------------------

    #[test]
    fn test_framing_splits_on_the_delimiter() {
        let mut framing = Framing::new();
        // One read can carry two whole messages.
        assert_eq!(
            framing.push(b"{\"a\":1}\x03{\"b\":2}\x03"),
            vec![b"{\"a\":1}".to_vec(), b"{\"b\":2}".to_vec()]
        );
        assert!(framing.partial().is_empty());
    }

    #[test]
    fn test_framing_holds_an_incomplete_message() {
        let mut framing = Framing::new();
        // The complete message is returned, the partial tail is kept.
        assert_eq!(
            framing.push(b"{\"a\":1}\x03{\"b\":"),
            vec![b"{\"a\":1}".to_vec()]
        );
        assert_eq!(framing.partial(), b"{\"b\":");
        // ...and completed by the next read rather than lost.
        assert_eq!(framing.push(b"2}\x03"), vec![b"{\"b\":2}".to_vec()]);
    }

    #[test]
    fn test_framing_splits_a_message_across_reads() {
        let mut framing = Framing::new();
        assert!(framing.push(b"{\"met").is_empty());
        assert!(framing.push(b"hod\":\"info\"}").is_empty());
        assert_eq!(
            framing.push(b"\x03"),
            vec![b"{\"method\":\"info\"}".to_vec()]
        );
    }

    #[test]
    fn test_framing_returns_empty_bodies() {
        // A leading delimiter is an empty message, which fails to decode
        // downstream — reported per message rather than desynchronising.
        let mut framing = Framing::new();
        assert_eq!(
            framing.push(b"\x03{}\x03"),
            vec![Vec::new(), b"{}".to_vec()]
        );
    }

    #[test]
    fn test_encode_appends_the_delimiter() {
        assert_eq!(encode(&json!({"a": 1})), b"{\"a\":1}\x03");
    }

    // -----------------------------------------------------------------------
    // Requests
    // -----------------------------------------------------------------------

    #[test]
    fn test_request_reads_id_method_and_params() {
        let request = request(r#"{"id":"1","method":"objects/query","params":{"x":2}}"#);
        assert_eq!(request.id(), Some(&json!("1")));
        assert_eq!(request.method(), "objects/query");
        assert_eq!(request.params().get_int("x").unwrap(), 2);
        assert!(request.expects_reply());
    }

    #[test]
    fn test_request_treats_a_missing_or_null_id_as_no_reply() {
        for body in [r#"{"method":"info"}"#, r#"{"id":null,"method":"info"}"#] {
            let request = request(body);
            assert_eq!(request.id(), None, "{body}");
            assert!(!request.expects_reply(), "{body}");
        }
    }

    #[test]
    fn test_request_defaults_params_to_empty() {
        let request = request(r#"{"method":"list_endpoints"}"#);
        assert!(request.params().is_empty());
        assert_eq!(request.params().len(), 0);
    }

    #[test]
    fn test_request_keeps_a_non_string_id_verbatim() {
        // The protocol does not constrain the type, so neither do we.
        let request = request(r#"{"id":{"tag":7},"method":"info"}"#);
        assert_eq!(request.id(), Some(&json!({"tag": 7})));
    }

    #[test]
    fn test_request_rejects_malformed_bodies() {
        for body in [
            "",
            "not json",
            "[1,2,3]",
            r#"{"id":1}"#,
            r#"{"method":7}"#,
            r#"{"method":"info","params":[]}"#,
        ] {
            assert!(
                Request::parse(body.as_bytes()).is_err(),
                "{body} should not parse"
            );
        }
    }

    #[test]
    fn test_malformed_request_reports_why() {
        let error = Request::parse(b"not json").unwrap_err();
        assert!(error.as_str().contains("invalid JSON"), "{error}");
        assert!(error.to_string().contains("invalid JSON"), "{error}");
    }

    // -----------------------------------------------------------------------
    // Replies
    // -----------------------------------------------------------------------

    #[test]
    fn test_respond_wraps_a_result() {
        let response = request(r#"{"id":1,"method":"info"}"#)
            .respond(Ok(json!({"state": "ready"})))
            .unwrap();
        assert_eq!(
            serde_json::to_value(response).unwrap(),
            json!({"id": 1, "result": {"state": "ready"}})
        );
    }

    #[test]
    fn test_respond_wraps_an_error() {
        let response = request(r#"{"id":1,"method":"info"}"#)
            .respond(Err(ApiError::MissingArgument("script".to_string())))
            .unwrap();
        assert_eq!(
            serde_json::to_value(response).unwrap(),
            json!({
                "id": 1,
                "error": {"error": "WebRequestError", "message": "Missing Argument [script]"}
            })
        );
    }

    #[test]
    fn test_respond_is_silent_without_an_id() {
        // Including on failure: a fire-and-forget request is never answered.
        let request = request(r#"{"method":"gcode/script"}"#);
        assert!(request.respond(Ok(json!({}))).is_none());
        assert!(request.respond(Err(ApiError::InvalidArgument)).is_none());
    }

    #[test]
    fn test_error_messages_match_the_reference() {
        assert_eq!(
            ApiError::InvalidArgumentType("objects".to_string()).to_string(),
            "Invalid Argument Type [objects]"
        );
        assert_eq!(
            ApiError::UnknownEndpoint("nope".to_string()).to_string(),
            "webhooks: No registered callback for path 'nope'"
        );
        assert_eq!(
            ApiError::UnknownMuxValue {
                key: "sensor".to_string(),
                value: "nope".to_string()
            }
            .to_string(),
            "The value 'nope' is not valid for sensor"
        );
        assert_eq!(
            ApiError::NoActiveConnections("printer_event".to_string()).to_string(),
            "No active connections for method 'printer_event'"
        );
    }

    #[test]
    fn test_only_internal_errors_ask_for_a_shutdown() {
        assert!(ApiError::Internal("boom".to_string()).is_internal());
        assert!(!ApiError::InvalidArgument.is_internal());
        assert!(!ApiError::UnknownEndpoint("nope".to_string()).is_internal());
    }

    // -----------------------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------------------

    fn params_of(value: Value) -> Map<String, Value> {
        value
            .as_object()
            .expect("test params are an object")
            .clone()
    }

    #[test]
    fn test_params_distinguish_missing_from_mistyped() {
        let object = params_of(json!({"name": "x"}));
        let params = Params::new(&object);

        assert!(params.has("name"));
        assert!(!params.has("absent"));
        assert_eq!(params.get_opt("absent"), None);
        assert_eq!(
            params.get("absent").unwrap_err(),
            ApiError::MissingArgument("absent".to_string())
        );
        assert_eq!(
            params.get_str("absent").unwrap_err(),
            ApiError::MissingArgument("absent".to_string())
        );
        assert_eq!(
            params.get_int("name").unwrap_err(),
            ApiError::InvalidArgumentType("name".to_string())
        );
    }

    #[test]
    fn test_params_read_numbers_booleans_and_containers() {
        let object = params_of(json!({
            "count": 3,
            "speed": 1.5,
            "flag": true,
            "objects": {"toolhead": null},
            "items": [1, 2]
        }));
        let params = Params::new(&object);

        assert_eq!(params.get_int("count").unwrap(), 3);
        // Integers are valid floats, as upstream's get_float allows.
        assert_eq!(params.get_float("count").unwrap(), 3.0);
        assert_eq!(params.get_float("speed").unwrap(), 1.5);
        assert!(params.get_bool("flag").unwrap());
        assert!(params.get_dict("objects").unwrap().contains_key("toolhead"));
        assert_eq!(params.get_array("items").unwrap().len(), 2);
        // A float is not an integer, and 1 is not `true`.
        assert!(params.get_int("speed").is_err());
        assert!(params.get_bool("count").is_err());
    }

    #[test]
    fn test_params_get_or_falls_back_without_type_checking() {
        let object = params_of(json!({"count": 3}));
        let params = Params::new(&object);
        let default = json!(7);

        assert_eq!(params.get_or("count", &default), &json!(3));
        // Upstream's default is returned as-is even when it does not match the
        // type the caller would have asked for.
        assert_eq!(params.get_or("absent", &default), &default);
        assert_eq!(params.names().collect::<Vec<_>>(), vec!["count"]);
    }

    // -----------------------------------------------------------------------
    // Pushes
    // -----------------------------------------------------------------------

    #[test]
    fn test_template_merges_payload_into_params() {
        let template =
            ResponseTemplate::new(params_of(json!({"id": null, "method": "printer:status"})));

        assert_eq!(
            template.message(json!({"eventtime": 1.0})),
            json!({
                "id": null,
                "method": "printer:status",
                "params": {"eventtime": 1.0}
            })
        );
    }

    #[test]
    fn test_template_wins_a_params_collision() {
        // Upstream's merge order, kept for compatibility: the template is
        // applied last, so its own `params` would shadow the payload.
        let template = ResponseTemplate::new(params_of(json!({"params": "shadowed"})));
        assert_eq!(
            template.message(json!({"a": 1})),
            json!({"params": "shadowed"})
        );
    }

    #[test]
    fn test_template_is_optional_and_type_checked() {
        let empty = params_of(json!({}));
        assert_eq!(
            ResponseTemplate::from_params(&Params::new(&empty)).unwrap(),
            ResponseTemplate::default()
        );

        let wrong = params_of(json!({"response_template": 7}));
        assert_eq!(
            ResponseTemplate::from_params(&Params::new(&wrong)).unwrap_err(),
            ApiError::InvalidArgumentType("response_template".to_string())
        );
    }
}
