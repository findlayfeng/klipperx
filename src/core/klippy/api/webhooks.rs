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
//!   config is read.
//! * [`WebhooksStatus::call_remote_method`] — upstream's `call_remote_method`
//!   (`klippy/webhooks.py:411-420`), the push side of
//!   `register_remote_method`. A `gcode_macro`'s `action_call_remote_method`
//!   uses it. The table is put in with [`WebhooksStatus::set_api`] once the
//!   server has been built.
//!
//! # The mux table's lifecycle
//!
//! A module registers a mux endpoint while the config is read, so it cannot
//! hold the API table: the table is built *after* the read, and a reload
//! replaces the modules while the same table keeps serving. This object is
//! therefore the table's lifecycle driver:
//!
//! 1. **before the table exists** (the initial load, which
//!    [`super::register`] follows) `register_mux_endpoint` buffers what it is
//!    given in [`WebhooksStatus`]'s pending list;
//! 2. `super::register` **drains** that list into the table, exactly once, when
//!    it builds it;
//! 3. the host hands the table over with [`WebhooksStatus::set_api`]; from then
//!    on every `register_mux_endpoint` goes **straight into the table** — a
//!    reload re-registers its instances that way, with no drain in between;
//! 4. when the config that registered them goes away, the table is **cleared**:
//!    [`PrinterObject::release_cycles`] (which [`Printer::teardown`] calls, and
//!    so does every restart) drops every mux registration, detaching the
//!    instances that belonged to the config that just left. The next load
//!    registers its own.
//!
//! Without step 4 a second restart would find the first load's registrations
//! still in the table and fail with `already registered`, while the old
//! handlers — and every module object they hold — kept being served.
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
/// [`WebhooksStatus`] holds these as its pending list; [`super::register`]
/// drains them into [`Api::register_mux`] once the table can be built.
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
    /// Mux endpoints registered while the API table does not exist yet.
    ///
    /// Only the initial load buffers here: [`super::register`] drains the list
    /// when it builds the table, and once [`WebhooksStatus::set_api`] has run,
    /// [`WebhooksStatus::register_mux_endpoint`] registers straight into
    /// [`Api`] and this stays empty.
    pending: Mutex<Vec<MuxRegistration>>,
    /// The API table, set by the host once the server is built.
    api: Mutex<Option<Arc<Api>>>,
}

impl WebhooksStatus {
    /// Build the object over the machine whose state it reports.
    pub fn new(printer: Arc<Printer>) -> Self {
        Self {
            printer,
            pending: Mutex::new(Vec::new()),
            api: Mutex::new(None),
        }
    }

    /// Register one instance of a mux endpoint.
    ///
    /// Upstream's `WebHooks.register_mux_endpoint` (`klippy/webhooks.py:329`):
    /// a path may serve several instances, all selected by the same key. Two
    /// registrations for a path with different keys, or the same instance
    /// twice, are config errors.
    ///
    /// Once the API table exists the registration goes straight into it. Before
    /// that — during the initial load only — it is buffered, because the table
    /// has not been built yet; [`super::register`] drains the buffer into the
    /// table afterwards, so the same checks apply either way. The *wording* of a
    /// rejected registration differs, though: the buffered path formats
    /// upstream's message, while the direct path surfaces the API table's own
    /// (`RegistrationError`).
    pub fn register_mux_endpoint(
        &self,
        path: &str,
        key: &str,
        value: Option<&str>,
        handler: Arc<dyn MuxEndpoint>,
    ) -> Result<(), ConfigError> {
        let api = self.api.lock().unwrap_or_else(|p| p.into_inner()).clone();
        if let Some(api) = api {
            return api
                .register_mux(path, key, value, handler)
                .map_err(|err| ConfigError::new(err.to_string()));
        }
        let mut pending = self.pending.lock().unwrap_or_else(|p| p.into_inner());
        let shown = value.unwrap_or("None");
        if let Some(first) = pending
            .iter()
            .find(|registration| registration.path == path)
        {
            if first.key != key {
                return Err(ConfigError::new(format!(
                    "mux endpoint {path} {key} {shown} may have only one key ({})",
                    first.key
                )));
            }
        }
        if pending
            .iter()
            .any(|registration| registration.path == path && registration.value.as_deref() == value)
        {
            let registered: Vec<&str> = pending
                .iter()
                .filter(|registration| registration.path == path)
                .map(|registration| registration.value.as_deref().unwrap_or("None"))
                .collect();
            return Err(ConfigError::new(format!(
                "mux endpoint {path} {key} {shown} already registered ({})",
                registered.join(", ")
            )));
        }
        pending.push(MuxRegistration {
            path: path.to_string(),
            key: key.to_string(),
            value: value.map(str::to_string),
            handler,
        });
        Ok(())
    }

    /// Take the pending mux registrations out, for [`super::register`] to
    /// install — the one drain, when the table is built.
    pub(crate) fn take_mux_endpoints(&self) -> Vec<MuxRegistration> {
        std::mem::take(&mut *self.pending.lock().unwrap_or_else(|p| p.into_inner()))
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

    fn release_cycles(&self) {
        // This object outlives the config (it is a host object), but the mux
        // instances in the table do not: each one was registered by a module of
        // the config that is being dropped and holds that module's objects.
        // Clearing detaches every instance — which stops whatever stream it
        // started — and releases those objects, so the next load starts from an
        // empty table rather than colliding with this one's paths.
        //
        // `release_cycles` is the moment because `Printer::teardown` calls it on
        // every object before truncating the config's parts, and a restart goes
        // through the same teardown.
        let api = self.api.lock().unwrap_or_else(|p| p.into_inner()).clone();
        if let Some(api) = api {
            api.clear_mux();
        }
        self.pending
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clear();
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
    use crate::core::klippy::api::test_support::{silent_target, RecordingTarget};
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

    /// A mux request for the instance the regression test registers.
    fn dump_request() -> Request {
        Request::parse(r#"{"method":"sensors/dump","params":{"sensor":"a"}}"#.as_bytes())
            .expect("test body is a valid request")
    }

    /// Dispatch that request and return what the mux instance answered.
    async fn dispatch(api: &Api) -> Value {
        api.dispatch(&dump_request(), silent_target())
            .await
            .expect("the mux instance answers")
    }

    /// The regression: a second restart must not find the previous load's mux
    /// instances still in the table.
    ///
    /// Once [`WebhooksStatus::set_api`] has run, a registration goes straight
    /// into the API — no drain as at startup. `Printer::teardown` (which
    /// `reset_for_restart` also reaches) then takes them out again, so the next
    /// load registers the same `(path, value)` with a fresh handler instead of
    /// failing with `already registered` while the old handler keeps being
    /// served.
    #[tokio::test]
    async fn test_a_registration_after_set_api_reaches_the_table_and_teardown_clears_it() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let webhooks = install(&printer).unwrap();
        let api = Arc::new(Api::new());
        webhooks.set_api(Arc::clone(&api));

        webhooks
            .register_mux_endpoint("sensors/dump", "sensor", Some("a"), Arc::new(Echo("first")))
            .unwrap();
        assert!(api.endpoints().contains(&"sensors/dump".to_string()));
        assert_eq!(dispatch(&api).await, json!("first"));

        printer.teardown();

        assert!(!api.endpoints().contains(&"sensors/dump".to_string()));
        assert_eq!(
            api.dispatch(&dump_request(), silent_target())
                .await
                .unwrap_err(),
            ApiError::UnknownEndpoint("sensors/dump".to_string())
        );

        // The next load re-registers the instance; the table serves its
        // handler, not the one the config that left installed.
        webhooks
            .register_mux_endpoint(
                "sensors/dump",
                "sensor",
                Some("a"),
                Arc::new(Echo("second")),
            )
            .unwrap();
        assert_eq!(dispatch(&api).await, json!("second"));
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
