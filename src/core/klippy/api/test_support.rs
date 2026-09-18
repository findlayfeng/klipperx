//! Shared fixtures for the endpoint tests.
//!
//! Compiled under `cfg(test)` only. Endpoints are tested through
//! [`Endpoint::handle`], which needs an [`EndpointContext`] — and building one
//! needs a connection. These endpoints never push to it, so the connection is
//! one that goes nowhere.

use std::sync::Arc;

use serde_json::Value;

use crate::core::klippy::api::protocol::PushTarget;
use crate::core::klippy::api::registry::Api;
use crate::core::klippy::api::EndpointContext;
use crate::core::klippy::printer::StatusSource;

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

/// A status source with a fixed status, for tests that need an object whose
/// answer the test wrote.
pub struct FixedStatus(pub Value);

impl StatusSource for FixedStatus {
    fn get_status(&self, _eventtime: f64) -> Value {
        self.0.clone()
    }
}

/// A status source that echoes the `eventtime` it was asked with, so a test can
/// check what the endpoint passed down.
pub struct EchoEventtime;

impl StatusSource for EchoEventtime {
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
