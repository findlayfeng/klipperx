//! Shared fixtures for the endpoint tests.
//!
//! Compiled under `cfg(test)` only. Endpoints are tested through
//! [`Endpoint::handle`], which needs an [`EndpointContext`] — and building one
//! needs a connection. These endpoints never push to it, so the connection is
//! one that goes nowhere.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use serde_json::Value;

use crate::core::klippy::api::protocol::PushTarget;
use crate::core::klippy::api::registry::Api;
use crate::core::klippy::api::EndpointContext;
use crate::core::klippy::printer::PrinterObject;

/// A connection whose pushes are dropped.
pub struct SilentTarget;

impl PushTarget for SilentTarget {
    fn is_closed(&self) -> bool {
        false
    }

    fn push(&self, _message: Value) {}
}

/// The connection an endpoint test passes in.
pub fn silent_target() -> Arc<dyn PushTarget> {
    Arc::new(SilentTarget)
}

/// A connection that records what was pushed to it.
///
/// For subscription tests, which need to see the pushes that arrive after the
/// request is answered — the same handle can be handed to several requests, and
/// [`RecordingTarget::close`] makes it look like the client went away.
pub struct RecordingTarget {
    pushes: Mutex<Vec<Value>>,
    closed: AtomicBool,
}

impl RecordingTarget {
    /// A live connection with an empty outbox.
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            pushes: Mutex::new(Vec::new()),
            closed: AtomicBool::new(false),
        })
    }

    /// Everything pushed so far, in order.
    pub fn pushes(&self) -> Vec<Value> {
        self.pushes
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }

    /// Mark the connection gone, as a dropped socket would.
    pub fn close(&self) {
        self.closed.store(true, Ordering::SeqCst);
    }
}

impl PushTarget for RecordingTarget {
    fn is_closed(&self) -> bool {
        self.closed.load(Ordering::SeqCst)
    }

    fn push(&self, message: Value) {
        self.pushes
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push(message);
    }
}

/// An object with a fixed status, for tests that need an object whose answer
/// the test wrote.
pub struct FixedStatus(pub Value);

impl PrinterObject for FixedStatus {
    fn get_status(&self, _eventtime: f64) -> Value {
        self.0.clone()
    }
}

/// An object whose status the test writes while the printer is running.
///
/// `objects/subscribe` only pushes what changed, so its tests have to make a
/// value change between ticks — which a [`FixedStatus`] cannot do.
pub struct MutableStatus(Mutex<Value>);

impl MutableStatus {
    /// An object reporting `status` until [`MutableStatus::set`] changes it.
    pub fn new(status: Value) -> Self {
        Self(Mutex::new(status))
    }

    /// Replace what the object reports.
    pub fn set(&self, status: Value) {
        *self.0.lock().unwrap_or_else(|p| p.into_inner()) = status;
    }
}

impl PrinterObject for MutableStatus {
    fn get_status(&self, _eventtime: f64) -> Value {
        self.0.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }
}

/// An object that echoes the `eventtime` it was asked with, so a test can
/// check what the endpoint passed down.
pub struct EchoEventtime;

impl PrinterObject for EchoEventtime {
    fn get_status(&self, eventtime: f64) -> Value {
        serde_json::json!({ "eventtime": eventtime })
    }
}

/// An `EndpointContext` over a fresh registry, borrowed for the call.
///
/// The registry is a parameter rather than built here because
/// [`EndpointContext`] borrows it.
pub fn context<'a>(api: &'a Api, client: Arc<dyn PushTarget>) -> EndpointContext<'a> {
    EndpointContext { api, client }
}
