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
//! It is also how a module reaches the server after it exists. Two things go
//! through it, because a module is loaded while the API table is being built
//! and cannot hold it:
//!
//! * [`WebhooksStatus::register_mux_endpoint`] — upstream's
//!   `register_mux_endpoint` (`klippy/webhooks.py:329-343`). An `[adxl345]`
//!   section registers `adxl345/dump_adxl345` keyed by `sensor` while the
//!   config is read; [`super::register`] moves the registrations into the table
//!   afterwards.
//! * [`WebhooksStatus::call_remote_method`] — upstream's `call_remote_method`
//!   (`klippy/webhooks.py:411-420`), the push side of
//!   `register_remote_method`. A `gcode_macro`'s `action_call_remote_method`
//!   uses it. The table is put in with [`WebhooksStatus::set_api`] once the
//!   server has been built.
//!
//! The name is upstream's, and clients — Moonraker among them — ask for it by
//! name, so it stays on the wire even though this host calls the same component
//! the API server (`klippy-api` and [`super::endpoints`]).
//!
//! It lives here, and not on the machine, because that is whose object it is:
//! the machine owns the state, the server owns the object that reports it.
//! [`super::register`] installs it.

use std::sync::{Arc, Mutex};

use serde_json::{json, Value};

use crate::core::klippy::api::protocol::ApiError;
use crate::core::klippy::api::registry::{Api, MuxEndpoint};
use crate::core::klippy::error::ConfigError;
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The object name clients ask for, as upstream registers it.
pub const WEBHOOKS_OBJECT: &str = "webhooks";

/// Register the `webhooks` object on a machine, once.
///
/// Upstream's `add_early_printer_objects` (`klippy/webhooks.py:563-565`), which
/// the printer calls before the config is read so that a module can find the
/// server by name while it loads. Idempotent, so a caller that is not sure
/// whether the host already installed it can call it again.
pub fn install(printer: &Arc<Printer>) -> Result<Arc<WebhooksStatus>, ConfigError> {
    if let Some(existing) = printer.lookup_object_as::<WebhooksStatus>(WEBHOOKS_OBJECT) {
        return Ok(existing);
    }
    let object = Arc::new(WebhooksStatus::new(Arc::clone(printer)));
    printer.add_object(
        WEBHOOKS_OBJECT,
        Arc::clone(&object) as Arc<dyn PrinterObject>,
    )?;
    Ok(object)
}

/// One mux endpoint a module registered before the API table existed.
///
/// [`super::register`] drains these into [`Api::register_mux`] once the table
/// can be built.
pub struct MuxRegistration {
    /// The path clients use, e.g. `"adxl345/dump_adxl345"`.
    pub path: String,
    /// The request key that selects the instance, e.g. `"sensor"`.
    pub key: String,
    /// The instance name, or `None` for the optional default instance.
    pub value: Option<String>,
    /// The handler for this instance.
    pub handler: Arc<dyn MuxEndpoint>,
}

/// The API server's printer object: the printer's state, for clients.
pub struct WebhooksStatus {
    printer: Arc<Printer>,
    /// Mux endpoints registered while the config is read.
    mux: Mutex<Vec<MuxRegistration>>,
    /// The API table, set by the host once the server is built.
    api: Mutex<Option<Arc<Api>>>,
}

impl WebhooksStatus {
    /// Build the object over the machine whose state it reports.
    pub fn new(printer: Arc<Printer>) -> Self {
        Self {
            printer,
            mux: Mutex::new(Vec::new()),
            api: Mutex::new(None),
        }
    }

    /// Register one instance of a mux endpoint.
    ///
    /// Upstream's `WebHooks.register_mux_endpoint` (`klippy/webhooks.py:329`):
    /// a path may serve several instances, all selected by the same key. Two
    /// registrations for a path with different keys, or the same instance
    /// twice, are config errors.
    pub fn register_mux_endpoint(
        &self,
        path: &str,
        key: &str,
        value: Option<&str>,
        handler: Arc<dyn MuxEndpoint>,
    ) -> Result<(), ConfigError> {
        let mut mux = self.mux.lock().unwrap_or_else(|p| p.into_inner());
        let shown = value.unwrap_or("None");
        if let Some(first) = mux.iter().find(|registration| registration.path == path) {
            if first.key != key {
                return Err(ConfigError::new(format!(
                    "mux endpoint {path} {key} {shown} may have only one key ({})",
                    first.key
                )));
            }
        }
        if mux
            .iter()
            .any(|registration| registration.path == path && registration.value.as_deref() == value)
        {
            let registered: Vec<&str> = mux
                .iter()
                .filter(|registration| registration.path == path)
                .map(|registration| registration.value.as_deref().unwrap_or("None"))
                .collect();
            return Err(ConfigError::new(format!(
                "mux endpoint {path} {key} {shown} already registered ({})",
                registered.join(", ")
            )));
        }
        mux.push(MuxRegistration {
            path: path.to_string(),
            key: key.to_string(),
            value: value.map(str::to_string),
            handler,
        });
        Ok(())
    }

    /// Take the mux registrations out, for [`super::register`] to install.
    pub(crate) fn take_mux_endpoints(&self) -> Vec<MuxRegistration> {
        std::mem::take(&mut *self.mux.lock().unwrap_or_else(|p| p.into_inner()))
    }

    /// Give the object the API table, once the server has been built.
    ///
    /// Until this is called, a `call_remote_method` reports the method as
    /// unregistered, which is also what upstream does when nobody registered
    /// it: the table does not exist, so no connection could have registered.
    pub fn set_api(&self, api: Arc<Api>) {
        *self.api.lock().unwrap_or_else(|p| p.into_inner()) = Some(api);
    }

    /// Push `params` to every connection registered for `method`.
    ///
    /// Upstream's `WebHooks.call_remote_method` (`klippy/webhooks.py:411`):
    /// `command_error` when the method is unknown becomes
    /// [`ApiError::RemoteMethodNotRegistered`] here, and every connection that
    /// registered it receives the push.
    pub fn call_remote_method(&self, method: &str, params: Value) -> Result<(), ApiError> {
        let api = self.api.lock().unwrap_or_else(|p| p.into_inner()).clone();
        match api {
            Some(api) => api.call_remote_method(method, params),
            None => Err(ApiError::RemoteMethodNotRegistered(method.to_string())),
        }
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
    use crate::core::klippy::api::protocol::{PushTarget, Request};
    use crate::core::klippy::api::registry::{EndpointContext, EndpointFuture};
    use crate::core::klippy::api::test_support::RecordingTarget;
    use crate::core::klippy::reactor::ManualReactor;

    /// A mux handler that records the instance it was registered for.
    struct Echo(&'static str);

    impl MuxEndpoint for Echo {
        fn handle<'a>(
            &'a self,
            _request: &'a Request,
            _context: &'a EndpointContext<'a>,
        ) -> EndpointFuture<'a> {
            Box::pin(async move { Ok(json!(self.0)) })
        }
    }

    fn status() -> WebhooksStatus {
        WebhooksStatus::new(Arc::new(Printer::new(ManualReactor::shared())))
    }

    #[tokio::test]
    async fn test_the_object_name_is_the_documented_one() {
        assert_eq!(WEBHOOKS_OBJECT, "webhooks");
    }

    #[tokio::test]
    async fn test_the_object_reports_the_printers_state() {
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
    async fn test_mux_instances_share_a_path_and_a_key() {
        let object = status();
        object
            .register_mux_endpoint(
                "adxl345/dump_adxl345",
                "sensor",
                Some("adxl345"),
                Arc::new(Echo("a")),
            )
            .unwrap();
        object
            .register_mux_endpoint(
                "adxl345/dump_adxl345",
                "sensor",
                Some("second"),
                Arc::new(Echo("b")),
            )
            .unwrap();
        // The optional instance registers with no value, as upstream's `None`.
        object
            .register_mux_endpoint("adxl345/dump_adxl345", "sensor", None, Arc::new(Echo("c")))
            .unwrap();
        assert_eq!(object.take_mux_endpoints().len(), 3);
    }

    #[tokio::test]
    async fn test_a_mux_path_may_have_only_one_key() {
        let object = status();
        object
            .register_mux_endpoint("path", "sensor", Some("one"), Arc::new(Echo("a")))
            .unwrap();

        let err = object
            .register_mux_endpoint("path", "name", Some("two"), Arc::new(Echo("b")))
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "mux endpoint path name two may have only one key (sensor)"
        );
    }

    #[tokio::test]
    async fn test_a_mux_instance_cannot_be_registered_twice() {
        let object = status();
        object
            .register_mux_endpoint("path", "sensor", Some("one"), Arc::new(Echo("a")))
            .unwrap();

        let err = object
            .register_mux_endpoint("path", "sensor", Some("one"), Arc::new(Echo("b")))
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "mux endpoint path sensor one already registered (one)"
        );
    }

    #[tokio::test]
    async fn test_a_remote_method_needs_a_registered_connection() {
        let object = status();
        assert_eq!(
            object.call_remote_method("notify", json!({})).unwrap_err(),
            ApiError::RemoteMethodNotRegistered("notify".to_string())
        );
    }

    #[tokio::test]
    async fn test_a_called_remote_method_reaches_its_connection() {
        let object = status();
        let api = Arc::new(Api::new());
        let recording = RecordingTarget::new();
        let target: Arc<dyn PushTarget> = recording.clone();
        api.register_remote_method(
            "notify",
            target,
            crate::core::klippy::api::protocol::ResponseTemplate::new(
                json!({"method": "notify"}).as_object().unwrap().clone(),
            ),
        );
        object.set_api(Arc::clone(&api));

        object
            .call_remote_method("notify", json!({"a": 1}))
            .unwrap();

        assert_eq!(
            recording.pushes(),
            [json!({"method": "notify", "params": {"a": 1}})]
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
