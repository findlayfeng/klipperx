//! `objects/list` — the printer objects that can report status.
//!
//! The cheapest endpoint of the family: it names every object a client may ask
//! about. It takes no parameters, and answers in registration order, which is
//! the order upstream's object registry uses.
//!
//! ```json
//! {"id": 1, "method": "objects/list"}
//! ```
//!
//! Upstream filters its registry down to the objects that define
//! `get_status`; here the registry *is* that filter — an object is in it
//! because it implements [`PrinterObject`](crate::core::klippy::printer::PrinterObject),
//! so every name it holds is one `objects/query` can answer for.
//!
//! # Status
//!
//! Written, tested and registered by [`register`](super::super::register).

use std::sync::Arc;

use serde_json::json;

use crate::core::klippy::api::protocol::Request;
use crate::core::klippy::api::registry::{Endpoint, EndpointContext, EndpointFuture};
use crate::core::klippy::api::{Api, ApiWiring, RegistrationError};
use crate::core::klippy::printer::Printer;

endpoint!(install);

/// Install the `objects/list` endpoint.
pub(crate) fn install(api: &mut Api, wiring: &ApiWiring<'_>) -> Result<(), RegistrationError> {
    api.register(ObjectsList::new(Arc::clone(wiring.printer)))
        .map_err(RegistrationError::Endpoint)
}

/// The `objects/list` endpoint.
pub struct ObjectsList {
    printer: Arc<Printer>,
}

impl ObjectsList {
    /// Build the endpoint over the machine whose objects it lists.
    pub fn new(printer: Arc<Printer>) -> Self {
        Self { printer }
    }
}

impl Endpoint for ObjectsList {
    fn path(&self) -> &'static str {
        "objects/list"
    }

    fn handle<'a>(
        &'a self,
        _request: &'a Request,
        _context: &'a EndpointContext<'a>,
    ) -> EndpointFuture<'a> {
        Box::pin(async move { Ok(json!({ "objects": self.printer.queryable_objects() })) })
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::registry::Api;
    use crate::core::klippy::api::test_support::{context, silent_target, FixedStatus};
    use crate::core::klippy::pins::PrinterPins;
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::json;

    #[tokio::test]
    async fn test_the_endpoint_path_is_the_documented_one() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        assert_eq!(ObjectsList::new(printer).path(), "objects/list");
    }

    #[tokio::test]
    async fn test_a_printer_with_no_parts_lists_nothing() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let request = Request::parse(br#"{"method":"objects/list"}"#).unwrap();
        let api = Api::new();

        let response = ObjectsList::new(printer)
            .handle(&request, &context(&api, silent_target()))
            .await
            .unwrap();

        // A host's list is never empty in practice: it registers the API
        // server's `webhooks` before it serves anything.
        assert_eq!(response, json!({ "objects": [] }));
    }

    #[tokio::test]
    async fn test_the_list_is_in_registration_order() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        for name in ["webhooks", "extruder", "heater_bed", "toolhead"] {
            printer
                .add_object(name, Arc::new(FixedStatus(json!({}))))
                .unwrap();
        }
        let request = Request::parse(br#"{"method":"objects/list"}"#).unwrap();
        let api = Api::new();

        let response = ObjectsList::new(printer)
            .handle(&request, &context(&api, silent_target()))
            .await
            .unwrap();

        assert_eq!(
            response,
            json!({ "objects": ["webhooks", "extruder", "heater_bed", "toolhead"] })
        );
    }

    #[tokio::test]
    async fn test_the_endpoint_takes_no_parameters() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        // A client sending junk is not rejected: upstream's handler reads no
        // parameter, so there is nothing to validate.
        let request = Request::parse(br#"{"method":"objects/list","params":{"x":1}}"#).unwrap();
        let api = Api::new();

        assert!(ObjectsList::new(printer)
            .handle(&request, &context(&api, silent_target()))
            .await
            .is_ok());
    }

    #[tokio::test]
    async fn test_a_registered_but_unqueryable_object_is_not_listed() {
        // `pins` is a printer object but has no status, so upstream's
        // `objects/list` leaves it out — the registry and the queryable set
        // differ.
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object("pins", Arc::new(PrinterPins::new()))
            .unwrap();
        printer
            .add_object("toolhead", Arc::new(FixedStatus(json!({}))))
            .unwrap();
        let request = Request::parse(br#"{"method":"objects/list"}"#).unwrap();
        let api = Api::new();

        let response = ObjectsList::new(printer)
            .handle(&request, &context(&api, silent_target()))
            .await
            .unwrap();

        assert_eq!(response, json!({ "objects": ["toolhead"] }));
    }

    #[tokio::test]
    async fn test_the_registry_reaches_the_endpoint_by_path() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object("extruder", Arc::new(FixedStatus(json!({}))))
            .unwrap();
        let mut api = Api::new();
        api.register(ObjectsList::new(printer)).unwrap();
        let request = Request::parse(br#"{"method":"objects/list"}"#).unwrap();

        assert_eq!(
            api.dispatch(&request, silent_target()).await.unwrap(),
            json!({"objects": ["extruder"]})
        );
    }
}
