//! `register_remote_method` — let a client be pushed to later.
//!
//! Upstream's `WebHooks._handle_rpc_registration`
//! (`klippy/webhooks.py:385-392`): a client that wants to receive pushes
//! registers a method name and a response template for **its own connection**.
//! The host then calls `webhooks.call_remote_method(name, **params)` (a macro's
//! `action_call_remote_method`, for instance) and the push reaches exactly the
//! connections that registered that name, wrapped in their own template.
//!
//! ```json
//! {"id": 1, "method": "register_remote_method",
//!  "params": {"remote_method": "notify_shutdown",
//!             "response_template": {"method": "notify_shutdown"}}}
//! ```
//!
//! `response_template` is optional upstream (`get_dict(..., {})`); the answer is
//! `{}`, and it says only that the registration was accepted.
//!
//! The template is part of the reply envelope the connection normally builds,
//! so the host does not know what shape a client expects — the client supplies
//! it here and [`Api::call_remote_method`] merges `{"params": …}` into it.
//!
//! # Status
//!
//! Written, tested and registered by [`register`](super::super::register).

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::api::protocol::{ApiError, Request, ResponseTemplate};
use crate::core::klippy::api::registry::{Endpoint, EndpointContext};
use crate::core::klippy::api::{Api, ApiWiring, RegistrationError};

endpoint!(install);

/// Install the `register_remote_method` endpoint.
pub(crate) fn install(api: &mut Api, _wiring: &ApiWiring<'_>) -> Result<(), RegistrationError> {
    api.register(RegisterRemoteMethod)
        .map_err(RegistrationError::Endpoint)
}

/// The `register_remote_method` endpoint.
pub struct RegisterRemoteMethod;

impl Endpoint for RegisterRemoteMethod {
    fn path(&self) -> &'static str {
        "register_remote_method"
    }

    fn handle(&self, request: &Request, context: &EndpointContext<'_>) -> Result<Value, ApiError> {
        let params = request.params();
        // Upstream reads the method first, then the optional template; a
        // missing `remote_method` is the only hard error.
        let method = params.get_str("remote_method")?;
        let template = ResponseTemplate::from_params(&params)?;
        context
            .api
            .register_remote_method(method, Arc::clone(&context.client), template);
        Ok(json!({}))
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::protocol::PushTarget;
    use crate::core::klippy::api::test_support::RecordingTarget;

    fn request(body: &str) -> Request {
        Request::parse(body.as_bytes()).expect("a valid request")
    }

    fn context(api: &Api, client: Arc<dyn PushTarget>) -> EndpointContext<'_> {
        EndpointContext { api, client }
    }

    #[test]
    fn test_a_registered_method_receives_its_template_and_params() {
        let api = Api::new();
        let recording = RecordingTarget::new();
        let client: Arc<dyn PushTarget> = recording.clone();
        let body = r#"{"id":1,"method":"register_remote_method","params":
            {"remote_method":"notify_shutdown",
             "response_template":{"method":"notify_shutdown"}}}"#;

        let reply = RegisterRemoteMethod
            .handle(&request(body), &context(&api, client))
            .unwrap();

        assert_eq!(reply, json!({}));
        // Nothing is pushed until somebody calls the method.
        assert!(recording.pushes().is_empty());

        api.call_remote_method("notify_shutdown", json!({"reason": "test"}))
            .unwrap();

        assert_eq!(
            recording.pushes(),
            [json!({"method": "notify_shutdown", "params": {"reason": "test"}})]
        );
    }

    #[test]
    fn test_the_response_template_is_optional() {
        let api = Api::new();
        let recording = RecordingTarget::new();
        let client: Arc<dyn PushTarget> = recording.clone();

        RegisterRemoteMethod
            .handle(
                &request(
                    r#"{"id":1,"method":"register_remote_method","params":
                       {"remote_method":"notify"}}"#,
                ),
                &context(&api, client),
            )
            .unwrap();
        api.call_remote_method("notify", json!({"a": 1})).unwrap();

        // With no template the push is still enveloped; `params` is the whole
        // body, which is what upstream's `{'params': kwargs}` becomes.
        assert_eq!(recording.pushes(), [json!({"params": {"a": 1}})]);
    }

    #[test]
    fn test_a_missing_method_name_is_an_argument_error() {
        let api = Api::new();
        let client = crate::core::klippy::api::test_support::silent_target();

        let err = RegisterRemoteMethod
            .handle(
                &request(r#"{"id":1,"method":"register_remote_method"}"#),
                &context(&api, client),
            )
            .unwrap_err();

        assert_eq!(err, ApiError::MissingArgument("remote_method".to_string()));
    }

    #[test]
    fn test_only_the_registered_connection_is_pushed_to() {
        let api = Api::new();
        let quiet = RecordingTarget::new();
        let loud = RecordingTarget::new();
        let body = r#"{"id":1,"method":"register_remote_method","params":
            {"remote_method":"notify","response_template":{"m":1}}}"#;

        RegisterRemoteMethod
            .handle(
                &request(body),
                &context(&api, Arc::clone(&quiet) as Arc<dyn PushTarget>),
            )
            .unwrap();
        RegisterRemoteMethod
            .handle(
                &request(body),
                &context(&api, Arc::clone(&loud) as Arc<dyn PushTarget>),
            )
            .unwrap();
        // The quiet connection went away; its registration must not be pushed
        // to, and the loud one must still get the message.
        quiet.close();

        api.call_remote_method("notify", json!({})).unwrap();

        assert!(quiet.pushes().is_empty());
        assert_eq!(loud.pushes(), [json!({"m": 1, "params": {}})]);
    }
}
