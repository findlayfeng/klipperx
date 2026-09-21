//! `emergency_stop` — halt the printer from a client.
//!
//! Upstream's `WebHooks._handle_estop_request`
//! (`klippy/webhooks.py:382-383`): the request carries no parameters and the
//! only thing it does is put the printer into its shutdown state, which is what
//! sends `emergency_stop` to every MCU (the `klippy:shutdown` handler each MCU
//! registers). The printer stays up afterwards, so the client can still read
//! *why* it stopped.
//!
//! ```json
//! {"id": 1, "method": "emergency_stop"}
//! ```
//!
//! The answer is `{}`: the request was accepted, not that the machine is fine.
//!
//! # Status
//!
//! Written, tested and registered by [`register`](super::register).

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::api::protocol::{ApiError, Request};
use crate::core::klippy::api::registry::{Endpoint, EndpointContext};
use crate::core::klippy::api::{Api, ApiWiring, RegistrationError};
use crate::core::klippy::printer::Printer;

endpoint!(install);

/// Install the `emergency_stop` endpoint.
pub(crate) fn install(api: &mut Api, wiring: &ApiWiring<'_>) -> Result<(), RegistrationError> {
    api.register(EmergencyStop::new(Arc::clone(wiring.printer)))
        .map_err(RegistrationError::Endpoint)
}

/// The `emergency_stop` endpoint.
pub struct EmergencyStop {
    printer: Arc<Printer>,
}

impl EmergencyStop {
    /// Build the endpoint over the machine it halts.
    pub fn new(printer: Arc<Printer>) -> Self {
        Self { printer }
    }
}

impl Endpoint for EmergencyStop {
    fn path(&self) -> &'static str {
        "emergency_stop"
    }

    fn handle(
        &self,
        _request: &Request,
        _context: &EndpointContext<'_>,
    ) -> Result<Value, ApiError> {
        // Upstream's exact wording: it is what a client that only sees the log
        // greps for, and what `info`'s `state_message` then reports.
        self.printer
            .invoke_shutdown("Shutdown due to webhooks request");
        Ok(json!({}))
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::test_support::silent_target;
    use crate::core::klippy::printer::PrinterState;
    use crate::core::klippy::reactor::ManualReactor;

    fn request() -> Request {
        Request::parse(br#"{"id":1,"method":"emergency_stop"}"#).expect("a valid request")
    }

    #[test]
    fn test_the_request_halts_the_printer_and_answers_ok() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let api = Api::new();
        let endpoint = EmergencyStop::new(Arc::clone(&printer));
        let context = EndpointContext {
            api: &api,
            client: silent_target(),
        };

        let reply = endpoint.handle(&request(), &context).unwrap();

        assert_eq!(reply, json!({}));
        let state = printer.get_state_message();
        assert_eq!(state.category, PrinterState::Shutdown);
        assert_eq!(state.message, "Shutdown due to webhooks request");
    }
}
