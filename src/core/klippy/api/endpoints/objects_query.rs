//! `objects/query` — the status of one or more printer objects, once.
//!
//! ```json
//! {"id": 1, "method": "objects/query",
//!  "params": {"objects": {"toolhead": ["position"], "webhooks": null}}}
//! ```
//!
//! `objects` maps an object name to the fields wanted, or to `null` for all of
//! them. The answer carries the printer's clock and one status per name:
//!
//! ```json
//! {"eventtime": 12.5,
//!  "status": {"toolhead": {"position": [20.0, 30.0, 5.0, 0.0]},
//!             "webhooks": {"state": "ready", "state_message": "Printer is ready"}}}
//! ```
//!
//! Three behaviours are upstream's, and a client can see all three:
//!
//! * an **unknown object** is not an error: it answers `{}`, and each field the
//!   client asked for comes back `null`;
//! * a **field an object does not have** comes back `null` rather than being
//!   left out, so a client can tell "not asked" from "not there";
//! * requesting `null` fields returns whatever the object reports *now*;
//!   upstream also rewrites the request to the field list it found, which only
//!   `objects/subscribe` can observe.
//!
//! # Status
//!
//! Written and tested; **not registered** yet, because no host builds a printer
//! to hand it (see the `TODO`). `objects/subscribe` — the same query on a
//! 0.25 s timer, pushing what changed — is not written: it needs the reactor
//! the machine does not have yet.

use std::sync::Arc;

use serde_json::{json, Map, Value};

use crate::core::klippy::api::protocol::{ApiError, Params, Request};
use crate::core::klippy::api::registry::{Endpoint, EndpointContext};
use crate::core::klippy::printer::Printer;

/// The `objects/query` endpoint.
pub struct ObjectsQuery {
    printer: Arc<Printer>,
}

impl ObjectsQuery {
    /// Build the endpoint over the machine it queries.
    pub fn new(printer: Arc<Printer>) -> Self {
        Self { printer }
    }
}

impl Endpoint for ObjectsQuery {
    fn path(&self) -> &'static str {
        "objects/query"
    }

    fn handle(&self, request: &Request, _context: &EndpointContext<'_>) -> Result<Value, ApiError> {
        let params = ObjectsQueryParams::from_request(request)?;
        Ok(params.query(&self.printer))
    }
}

/// The `objects` request parameter.
///
/// Each name maps to the fields wanted, or to `null` for all of them. The
/// parameter is required, and its shape is checked here rather than at use, so
/// a malformed query fails before any object is asked.
#[derive(Debug, Clone)]
pub struct ObjectsQueryParams {
    /// Object name to the fields wanted; JSON `null` means every field.
    objects: Map<String, Value>,
}

impl ObjectsQueryParams {
    /// Read the required `objects` parameter.
    ///
    /// # Errors
    /// Returns [`ApiError::MissingArgument`] if `objects` is absent,
    /// [`ApiError::InvalidArgumentType`] if it is not an object, and
    /// [`ApiError::InvalidArgument`] if a value is neither `null` nor an array
    /// of strings — the three answers upstream's `_handle_query` gives.
    pub fn from_request(request: &Request) -> Result<Self, ApiError> {
        Self::from_params(&request.params())
    }

    /// Read the required `objects` parameter from decoded parameters.
    ///
    /// # Errors
    /// As [`ObjectsQueryParams::from_request`].
    pub fn from_params(params: &Params<'_>) -> Result<Self, ApiError> {
        let objects = params.get_dict("objects")?;
        // A JSON object key is always a string; the field list is what a client
        // can get wrong.
        for fields in objects.values() {
            match fields {
                Value::Null => {}
                Value::Array(fields) if fields.iter().all(Value::is_string) => {}
                _ => return Err(ApiError::InvalidArgument),
            }
        }
        Ok(Self {
            objects: objects.clone(),
        })
    }

    /// Answer the query against `printer`.
    ///
    /// The printer's clock is read once, so every object in the answer is dated
    /// the same — and sources are handed that same value.
    pub fn query(&self, printer: &Printer) -> Value {
        let eventtime = printer.eventtime();
        let mut status = Map::new();
        for (name, fields) in &self.objects {
            let full = match printer.status_of(name, eventtime) {
                Some(Value::Object(full)) => full,
                // An unknown object (or one reporting something other than an
                // object) has no fields to give.
                _ => Map::new(),
            };
            let selected = match fields {
                Value::Null => Value::Object(full),
                Value::Array(fields) => {
                    let mut selected = Map::new();
                    for field in fields {
                        let field = field.as_str().expect("validated to be strings");
                        // A field the object does not have is answered with
                        // `null`, not left out.
                        let value = full.get(field).cloned().unwrap_or(Value::Null);
                        selected.insert(field.to_string(), value);
                    }
                    Value::Object(selected)
                }
                _ => unreachable!("validated in from_params"),
            };
            status.insert(name.clone(), selected);
        }
        json!({ "eventtime": eventtime, "status": status })
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::registry::Api;
    use crate::core::klippy::api::test_support::{
        context, silent_target, EchoEventtime, FixedStatus,
    };
    use crate::core::klippy::api::webhooks::{WebhooksStatus, WEBHOOKS_OBJECT};
    use serde_json::json;

    fn request(body: &str) -> Request {
        Request::parse(body.as_bytes()).expect("test body is a valid request")
    }

    /// An endpoint over a printer with one extra object, and a registry to
    /// build a context from.
    fn endpoint() -> (ObjectsQuery, Arc<Printer>, Api) {
        let printer = Arc::new(Printer::new());
        // What a host installs before it serves anything: the API server's own
        // object, then whatever parts exist.
        printer
            .add_object(
                WEBHOOKS_OBJECT,
                Arc::new(WebhooksStatus::new(Arc::clone(&printer))),
            )
            .unwrap();
        printer
            .add_object(
                "toolhead",
                Arc::new(FixedStatus(json!({
                    "position": [20.0, 30.0, 5.0, 0.0],
                    "max_velocity": 300.0,
                }))),
            )
            .unwrap();
        (ObjectsQuery::new(Arc::clone(&printer)), printer, Api::new())
    }

    #[test]
    fn test_the_endpoint_path_is_the_documented_one() {
        let (endpoint, _printer, _api) = endpoint();
        assert_eq!(endpoint.path(), "objects/query");
    }

    #[test]
    fn test_a_null_field_list_asks_for_every_field() {
        let (endpoint, _printer, api) = endpoint();
        let request =
            request(r#"{"method":"objects/query","params":{"objects":{"toolhead":null}}}"#);

        let response = endpoint
            .handle(&request, &context(&api, silent_target()))
            .unwrap();

        assert_eq!(
            response["status"]["toolhead"],
            json!({"position": [20.0, 30.0, 5.0, 0.0], "max_velocity": 300.0})
        );
    }

    #[test]
    fn test_only_the_requested_fields_come_back() {
        let (endpoint, _printer, api) = endpoint();
        let request = request(
            r#"{"method":"objects/query","params":{"objects":{"toolhead":["max_velocity"]}}}"#,
        );

        let response = endpoint
            .handle(&request, &context(&api, silent_target()))
            .unwrap();

        assert_eq!(
            response["status"]["toolhead"],
            json!({"max_velocity": 300.0})
        );
    }

    #[test]
    fn test_a_field_the_object_does_not_have_is_null() {
        let (endpoint, _printer, api) = endpoint();
        let request =
            request(r#"{"method":"objects/query","params":{"objects":{"toolhead":["nope"]}}}"#);

        let response = endpoint
            .handle(&request, &context(&api, silent_target()))
            .unwrap();

        assert_eq!(response["status"]["toolhead"], json!({"nope": null}));
    }

    #[test]
    fn test_an_unknown_object_answers_empty_not_an_error() {
        let (endpoint, _printer, api) = endpoint();
        let request = request(
            r#"{"method":"objects/query","params":{"objects":{"nope":null,"also_nope":["a"]}}}"#,
        );

        let response = endpoint
            .handle(&request, &context(&api, silent_target()))
            .unwrap();

        assert_eq!(response["status"]["nope"], json!({}));
        assert_eq!(response["status"]["also_nope"], json!({"a": null}));
    }

    #[test]
    fn test_the_answer_carries_the_eventtime_the_sources_were_given() {
        let printer = Arc::new(Printer::new());
        printer.add_object("echo", Arc::new(EchoEventtime)).unwrap();
        let endpoint = ObjectsQuery::new(Arc::clone(&printer));
        let api = Api::new();
        let request = request(r#"{"method":"objects/query","params":{"objects":{"echo":null}}}"#);

        let response = endpoint
            .handle(&request, &context(&api, silent_target()))
            .unwrap();

        assert_eq!(
            response["status"]["echo"]["eventtime"],
            response["eventtime"]
        );
        assert!(response["eventtime"].as_f64().unwrap() >= 0.0);
    }

    #[tokio::test]
    async fn test_the_servers_object_reports_the_printers_state() {
        let (endpoint, printer, api) = endpoint();
        let request =
            request(r#"{"method":"objects/query","params":{"objects":{"webhooks":null}}}"#);
        let ask = || {
            endpoint
                .handle(&request, &context(&api, silent_target()))
                .unwrap()["status"]["webhooks"]
                .clone()
        };

        assert_eq!(
            ask(),
            json!({"state": "startup", "state_message": "Starting up"})
        );

        printer.bring_up().await;
        assert_eq!(
            ask(),
            json!({"state": "ready", "state_message": "Printer is ready"})
        );

        printer.invoke_shutdown("Printer is halted");
        assert_eq!(
            ask(),
            json!({"state": "shutdown", "state_message": "Printer is halted"})
        );
    }

    #[test]
    fn test_objects_must_be_present() {
        let error = ObjectsQueryParams::from_request(&request(r#"{"method":"objects/query"}"#))
            .unwrap_err();
        assert_eq!(error, ApiError::MissingArgument("objects".to_string()));
    }

    #[test]
    fn test_objects_must_be_an_object() {
        let error = ObjectsQueryParams::from_request(&request(
            r#"{"method":"objects/query","params":{"objects":["toolhead"]}}"#,
        ))
        .unwrap_err();
        assert_eq!(error, ApiError::InvalidArgumentType("objects".to_string()));
    }

    #[test]
    fn test_a_field_list_must_be_null_or_strings() {
        for body in [
            r#"{"method":"objects/query","params":{"objects":{"toolhead":1}}}"#,
            r#"{"method":"objects/query","params":{"objects":{"toolhead":"position"}}}"#,
            r#"{"method":"objects/query","params":{"objects":{"toolhead":["position",3]}}}"#,
        ] {
            let error = ObjectsQueryParams::from_request(&request(body)).unwrap_err();
            assert_eq!(error, ApiError::InvalidArgument, "{body}");
            assert_eq!(error.to_string(), "Invalid argument", "{body}");
        }
    }

    #[test]
    fn test_an_empty_field_list_asks_for_nothing() {
        let (endpoint, _printer, api) = endpoint();
        let request = request(r#"{"method":"objects/query","params":{"objects":{"toolhead":[]}}}"#);

        let response = endpoint
            .handle(&request, &context(&api, silent_target()))
            .unwrap();

        assert_eq!(response["status"]["toolhead"], json!({}));
    }

    #[test]
    fn test_one_query_may_name_several_objects() {
        let (endpoint, _printer, api) = endpoint();
        let request = request(
            r#"{"method":"objects/query","params":{"objects":{"webhooks":["state"],"toolhead":["max_velocity"]}}}"#,
        );

        let response = endpoint
            .handle(&request, &context(&api, silent_target()))
            .unwrap();

        assert_eq!(response["status"]["webhooks"], json!({"state": "startup"}));
        assert_eq!(
            response["status"]["toolhead"],
            json!({"max_velocity": 300.0})
        );
    }

    #[test]
    fn test_the_registry_reaches_the_endpoint_by_path() {
        let (endpoint, _printer, mut api) = endpoint();
        api.register(endpoint).unwrap();
        let request =
            request(r#"{"method":"objects/query","params":{"objects":{"webhooks":["state"]}}}"#);

        let response = api.dispatch(&request, silent_target()).unwrap();

        assert_eq!(response["status"]["webhooks"], json!({"state": "startup"}));
    }

    #[test]
    fn test_no_objects_asked_for_is_an_empty_status() {
        let (endpoint, _printer, api) = endpoint();
        let request = request(r#"{"method":"objects/query","params":{"objects":{}}}"#);

        let response = endpoint
            .handle(&request, &context(&api, silent_target()))
            .unwrap();

        assert_eq!(response["status"], json!({}));
        assert!(response["eventtime"].as_f64().is_some());
    }
}
