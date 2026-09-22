//! `query_endstops/status` — every endstop's current level.
//!
//! Upstream's `QueryEndstops._handle_web_request`
//! (`klippy/extras/query_endstops.py:23-31`): run the queries against the last
//! move time, then answer `{name: "open"|"TRIGGERED"}` per endstop.
//!
//! ```json
//! {"id": 1, "method": "query_endstops/status"}
//! ```
//!
//! The reply is the endpoint's own value (not wrapped): a JSON object mapping
//! each registered endstop name to `"open"` or `"TRIGGERED"`.
//!
//! # Status
//!
//! Written, tested and registered by [`register`](super::super::register).

use std::sync::Arc;

use serde_json::{Map, Value};

use crate::core::klippy::api::protocol::{ApiError, Request};
use crate::core::klippy::api::registry::{Endpoint, EndpointContext};
use crate::core::klippy::api::{Api, ApiWiring, RegistrationError};
use crate::core::klippy::extras::query_endstops::{
    query_print_time, QueryEndstops, QUERY_ENDSTOPS_OBJECT,
};
use crate::core::klippy::printer::Printer;

endpoint!(install);

/// Install the `query_endstops/status` endpoint.
pub(crate) fn install(api: &mut Api, wiring: &ApiWiring<'_>) -> Result<(), RegistrationError> {
    api.register(QueryEndstopsStatus::new(Arc::clone(wiring.printer)))
        .map_err(RegistrationError::Endpoint)
}

/// The `query_endstops/status` endpoint.
pub struct QueryEndstopsStatus {
    printer: Arc<Printer>,
}

impl QueryEndstopsStatus {
    /// Build the endpoint over the machine it queries.
    pub fn new(printer: Arc<Printer>) -> Self {
        Self { printer }
    }
}

impl Endpoint for QueryEndstopsStatus {
    fn path(&self) -> &'static str {
        "query_endstops/status"
    }

    fn handle(
        &self,
        _request: &Request,
        _context: &EndpointContext<'_>,
    ) -> Result<Value, ApiError> {
        let query = self
            .printer
            .lookup_object_as::<QueryEndstops>(QUERY_ENDSTOPS_OBJECT)
            .ok_or_else(|| ApiError::Internal("query_endstops is not available".to_string()))?;
        let state = query
            .query_all_blocking(query_print_time(&self.printer))
            .map_err(|err| ApiError::Internal(err.to_string()))?;
        let mut map = Map::with_capacity(state.len());
        for (name, triggered) in state {
            map.insert(
                name,
                Value::String(if triggered { "TRIGGERED" } else { "open" }.to_string()),
            );
        }
        Ok(Value::Object(map))
    }
}

impl std::fmt::Debug for QueryEndstopsStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("QueryEndstopsStatus")
            .finish_non_exhaustive()
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::test_support::silent_target;
    use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
    use crate::core::klippy::reactor::ManualReactor;

    fn request() -> Request {
        Request::parse(br#"{"id":1,"method":"query_endstops/status"}"#).expect("a valid request")
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn test_the_endpoint_answers_an_empty_object_with_no_endstops() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let query = QueryEndstops::new(&printer).unwrap();
        printer
            .add_object(QUERY_ENDSTOPS_OBJECT, Arc::new(query))
            .unwrap();
        let endpoint = QueryEndstopsStatus::new(Arc::clone(&printer));
        let api = crate::core::klippy::api::Api::new();
        let context = EndpointContext {
            api: &api,
            client: silent_target(),
        };

        let value = endpoint.handle(&request(), &context).unwrap();

        assert_eq!(value, serde_json::json!({}));
    }

    #[test]
    fn test_the_path_is_the_upstream_one() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let endpoint = QueryEndstopsStatus::new(printer);
        assert_eq!(endpoint.path(), "query_endstops/status");
    }
}
