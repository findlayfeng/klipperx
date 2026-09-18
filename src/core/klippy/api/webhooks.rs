//! The API server's own printer object, `webhooks`.
//!
//! Upstream's `webhooks` module *is* the API server (the socket, the client
//! connections, the endpoint table), and it registers itself as a printer
//! object in `Printer.__init__` so that every printer reports one object before
//! a client can connect:
//!
//! ```python
//! def add_early_printer_objects(printer):            # klippy/webhooks.py:564
//!     printer.add_object('webhooks', WebHooks(printer))
//! ```
//!
//! That object's status is a *view of the printer's state*, not state the
//! server keeps of its own:
//!
//! ```python
//! def get_status(self, eventtime):                   # klippy/webhooks.py:404
//!     state_message, state = self.printer.get_state_message()
//!     return {'state': state, 'state_message': state_message}
//! ```
//!
//! The name is upstream's, and clients — Moonraker among them — ask for it by
//! name, so it stays on the wire even though this host calls the same component
//! the API server (`klippy-api` and [`super::endpoints`]).
//!
//! It lives here, and not on the machine, because that is whose object it is:
//! the machine owns the state, the server owns the object that reports it.
//! [`super::register`] installs it.

use std::sync::Arc;

use serde_json::{json, Value};

use crate::core::klippy::printer::{Printer, PrinterObject};

/// The object name clients ask for, as upstream registers it.
pub const WEBHOOKS_OBJECT: &str = "webhooks";

/// The API server's printer object: the printer's state, for clients.
pub struct WebhooksStatus {
    printer: Arc<Printer>,
}

impl WebhooksStatus {
    /// Build the object over the machine whose state it reports.
    pub fn new(printer: Arc<Printer>) -> Self {
        Self { printer }
    }
}

impl PrinterObject for WebhooksStatus {
    fn get_status(&self, _eventtime: f64) -> Value {
        let state = self.printer.get_state_message();
        json!({
            "state": state.category.as_category(),
            "state_message": state.message,
        })
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::reactor::ManualReactor;

    #[test]
    fn test_the_object_name_is_the_documented_one() {
        assert_eq!(WEBHOOKS_OBJECT, "webhooks");
    }

    #[test]
    fn test_the_object_reports_the_printers_state() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let status = WebhooksStatus::new(Arc::clone(&printer));

        assert_eq!(
            status.get_status(0.0),
            json!({"state": "startup", "state_message": "Starting up"})
        );

        printer.invoke_shutdown("Printer is halted");

        assert_eq!(
            status.get_status(0.0),
            json!({"state": "shutdown", "state_message": "Printer is halted"})
        );
    }

    #[tokio::test]
    async fn test_the_object_follows_the_printer_into_the_ready_state() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let status = WebhooksStatus::new(Arc::clone(&printer));
        printer.bring_up().await;
        printer.request_exit("exit");

        assert_eq!(printer.run(), "exit");

        assert_eq!(
            status.get_status(0.0),
            json!({"state": "ready", "state_message": "Printer is ready"})
        );
    }
}
