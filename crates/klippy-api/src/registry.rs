//! Endpoint registry and dispatch.
//!
//! This is the table clients address by name, plus the two mechanisms that do
//! not fit a plain name-to-handler map:
//!
//! * **mux endpoints** — one path serving several instances, selected by a
//!   parameter (`adxl345/dump_adxl345` with `"sensor": "adxl345"`);
//! * **remote methods** — the reverse direction, where the host pushes to a
//!   connection that previously registered itself (`register_remote_method`).
//!
//! # Lifecycle
//!
//! Plain endpoints are registered while printer objects are being created and
//! the table is frozen before the socket is served, so [`Api::register`] takes
//! `&mut self` and dispatch needs no lock on the hot path. Two tables are
//! mutable at runtime: mux instances, which a configuration reload replaces
//! ([`Api::register_mux`], [`Api::clear_mux`]), and remote methods, which
//! clients register and drop while serving. Dispatch takes their locks only
//! long enough to resolve a request, never across an `.await`.
//!
//! Registration failures are [`RegistrationError`], not [`ApiError`]: a
//! duplicated path is a bug in klippy that no client can provoke, and it should
//! fail loudly at startup instead of becoming an error reply.
//!
//! # Mux instances
//!
//! Upstream registers the *value* `None` for a mux path whose instance name
//! should be optional; a request may then omit the key parameter and reach the
//! `None` handler. Instances are addressed by the key parameter, whose name
//! (`sensor`, `name`, `load_cell`, …) is fixed by the first registration.

use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::fmt;
use std::future::Future;
use std::panic::AssertUnwindSafe;
use std::pin::Pin;
use std::sync::{Arc, RwLock};

use super::protocol::{ApiError, PushTarget, Request, ResponseTemplate};

/// What an endpoint returns: a boxed future, so a handler may `.await`
/// (`gcode/script` runs homing, which waits on the firmware).
pub type EndpointFuture<'a> = Pin<Box<dyn Future<Output = Result<Value, ApiError>> + Send + 'a>>;

// ===========================================================================
// Endpoints
// ===========================================================================

/// One client-facing endpoint.
///
/// The path lives in the type so that registration cannot disagree with what
/// the handler thinks it is called.
pub trait Endpoint: Send + Sync {
    /// The path clients use, e.g. `"info"` or `"objects/query"`.
    ///
    /// An instance method rather than an associated constant, because the
    /// registry stores endpoints behind `dyn Endpoint` and associated constants
    /// are not dyn-compatible.
    fn path(&self) -> &'static str;

    /// Handle one request.
    ///
    /// A handler returns its payload, which the server wraps in the reply
    /// envelope; it never builds the envelope itself, so it cannot get the
    /// `id` wrong. Returning `Ok(Value::Null)` is a valid answer meaning "no
    /// data" — it is not the same as sending nothing.
    ///
    /// # Errors
    /// Any [`ApiError`] becomes an `error` reply. Failures that are not the
    /// client's fault should be [`ApiError::Internal`], which the server also
    /// treats as a reason to shut klippy down.
    fn handle<'a>(
        &'a self,
        request: &'a Request,
        context: &'a EndpointContext<'a>,
    ) -> EndpointFuture<'a>;
}

/// A handler for one instance of a [mux endpoint](self#mux-instances).
pub trait MuxEndpoint: Send + Sync {
    /// Handle one request addressed to this instance.
    ///
    /// # Errors
    /// As [`Endpoint::handle`].
    fn handle<'a>(
        &'a self,
        request: &'a Request,
        context: &'a EndpointContext<'a>,
    ) -> EndpointFuture<'a>;

    /// Called when this registration is dropped.
    ///
    /// [`Api::clear_mux`] invokes it for every instance when the host rebuilds
    /// the configuration that registered them — a reload replaced this
    /// instance or its section went away. An instance that started background
    /// work or registered clients must stop them here; otherwise the old
    /// instance, and every object it holds, outlives the configuration it
    /// belonged to. Does nothing by default.
    ///
    /// It runs with the registry lock released, but it must not call back into
    /// the [`Api`] (registering or dispatching from here would re-enter the
    /// table the caller is clearing).
    fn detach(&self) {}
}

/// What a handler is given besides the request itself.
pub struct EndpointContext<'a> {
    /// The registry, for endpoints that describe it (`list_endpoints`) or push
    /// through it (remote methods).
    pub api: &'a Api,
    /// The connection the request arrived on, as an owned handle.
    ///
    /// Request/response endpoints ignore it. Endpoints that keep sending after
    /// they reply — `objects/subscribe`, `gcode/subscribe_output`, the
    /// `*/dump_*` family — clone it and keep it, which is what lets a
    /// subscription outlive its request; pushes stop when the client goes away
    /// ([`PushTarget::is_closed`]).
    pub client: Arc<dyn PushTarget>,
}

// ===========================================================================
// Registration errors
// ===========================================================================

/// A wiring mistake made while registering endpoints.
///
/// These are startup failures, never wire errors: they mean klippy tried to
/// register the same name twice, or two mux registrations disagreed about the
/// key parameter.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RegistrationError {
    /// A path was registered twice, by an endpoint and/or a mux endpoint.
    DuplicatePath(String),
    /// A mux path was registered with a different key parameter than before.
    ///
    /// All instances of a mux path must agree on the parameter that names them.
    MuxKeyConflict {
        /// The mux path.
        path: String,
        /// The key parameter of the first registration.
        expected: String,
        /// The key parameter of the rejected registration.
        found: String,
    },
    /// A mux path already had an instance registered under the same value.
    DuplicateMuxValue {
        /// The mux path.
        path: String,
        /// The instance value that was already taken. `None` is the default
        /// instance.
        value: Option<String>,
    },
}

impl fmt::Display for RegistrationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RegistrationError::DuplicatePath(path) => {
                write!(f, "path already registered to an endpoint: '{path}'")
            }
            RegistrationError::MuxKeyConflict {
                path,
                expected,
                found,
            } => write!(
                f,
                "mux endpoint '{path}' may have only one key ('{expected}', not '{found}')"
            ),
            RegistrationError::DuplicateMuxValue { path, value } => match value {
                Some(value) => write!(
                    f,
                    "mux endpoint '{path}' value '{value}' already registered"
                ),
                None => write!(
                    f,
                    "mux endpoint '{path}' default instance already registered"
                ),
            },
        }
    }
}

impl std::error::Error for RegistrationError {}

// ===========================================================================
// Registry
// ===========================================================================

/// A registered mux path: its key parameter and its instances.
struct Mux {
    key: String,
    /// Instance name to handler; `None` is the default instance, addressable by
    /// omitting the key parameter.
    values: BTreeMap<Option<String>, Arc<dyn MuxEndpoint>>,
}

impl Mux {
    /// Pick the instance a request addresses.
    ///
    /// The handler comes back as an owned `Arc` rather than a borrow of the
    /// table, so the caller can drop its lock before awaiting the handler.
    fn resolve(&self, request: &Request) -> Result<Arc<dyn MuxEndpoint>, ApiError> {
        let params = request.params();

        // A non-string value cannot name an instance; a missing one is only
        // allowed when a default instance was registered.
        let requested = match params.get_opt(&self.key) {
            Some(Value::String(name)) => Some(name.clone()),
            Some(other) => {
                return Err(ApiError::UnknownMuxValue {
                    key: self.key.clone(),
                    value: other.to_string(),
                })
            }
            None if self.values.contains_key(&None) => None,
            None => return Err(ApiError::MissingArgument(self.key.clone())),
        };

        match self.values.get(&requested) {
            Some(handler) => Ok(Arc::clone(handler)),
            None => Err(ApiError::UnknownMuxValue {
                key: self.key.clone(),
                value: requested.unwrap_or_default(),
            }),
        }
    }
}

/// One connection registered for a remote method.
struct RemoteRegistration {
    target: Arc<dyn PushTarget>,
    template: ResponseTemplate,
}

/// Called when a request handler fails on its own account.
///
/// The message is what upstream passes to `invoke_shutdown`
/// (`Internal Error on WebRequest: <method>`); see
/// [`Api::set_internal_error_hook`].
pub type InternalErrorHook = Arc<dyn Fn(&str) + Send + Sync>;

/// The endpoint table, and the entry point for dispatching requests.
///
/// Build it while creating printer objects, then share it with the
/// [`Server`](super::Server).
pub struct Api {
    endpoints: BTreeMap<String, Arc<dyn Endpoint>>,
    mux: RwLock<BTreeMap<String, Mux>>,
    remote: RwLock<BTreeMap<String, Vec<RemoteRegistration>>>,
    /// What to tell the host when a handler fails on its own account.
    ///
    /// Upstream shuts klippy down when a `webhooks` callback raises anything
    /// that is not a command error (`klippy/webhooks.py:271-276`). That is the
    /// host's decision, not the API crate's, so the crate calls this hook —
    /// `api::register` points it at `Printer::invoke_shutdown`. `None` means
    /// "just answer the error", which is what the crate's own tests want.
    internal_error: Option<InternalErrorHook>,
}

impl Api {
    /// A registry with the built-in endpoints registered.
    ///
    /// Only `list_endpoints` is built in: it is the registry describing itself,
    /// not a printer operation. The documented built-ins that do printer work
    /// (`info`, `emergency_stop`, `register_remote_method`) are endpoints like
    /// any other and are added by their own modules.
    pub fn new() -> Self {
        let mut api = Self {
            endpoints: BTreeMap::new(),
            mux: RwLock::new(BTreeMap::new()),
            remote: RwLock::new(BTreeMap::new()),
            internal_error: None,
        };
        api.register(ListEndpoints)
            .expect("the built-in path is free in a new registry");
        api
    }

    /// Install what to call when a handler fails on its own account.
    ///
    /// The hook is given the message to report (`Internal Error on WebRequest:
    /// <method>`), exactly as upstream passes it to `invoke_shutdown`. It must
    /// not block.
    pub fn set_internal_error_hook(&mut self, hook: InternalErrorHook) {
        self.internal_error = Some(hook);
    }

    /// Register an endpoint under its own [`Endpoint::path`].
    ///
    /// # Errors
    /// Returns [`RegistrationError::DuplicatePath`] if the path is taken.
    pub fn register<E: Endpoint + 'static>(
        &mut self,
        endpoint: E,
    ) -> Result<(), RegistrationError> {
        let path = endpoint.path().to_string();
        if self.path_taken(&path) {
            return Err(RegistrationError::DuplicatePath(path));
        }
        self.endpoints.insert(path, Arc::new(endpoint));
        Ok(())
    }

    /// Register one instance of a mux endpoint.
    ///
    /// `value` is the instance name this handler answers to; `None` registers
    /// the default instance, reachable by omitting `key` from the parameters.
    /// `key` must be the same for every instance of `path`.
    ///
    /// # Errors
    /// Returns [`RegistrationError::DuplicatePath`] if `path` is already a
    /// plain endpoint, [`RegistrationError::MuxKeyConflict`] if a previous
    /// instance used a different key, or
    /// [`RegistrationError::DuplicateMuxValue`] if `value` is taken.
    pub fn register_mux(
        &self,
        path: &str,
        key: &str,
        value: Option<&str>,
        handler: Arc<dyn MuxEndpoint>,
    ) -> Result<(), RegistrationError> {
        let value = value.map(str::to_string);

        // One write lock decides the whole registration: `endpoints` needs no
        // lock of its own, and checking the path through [`Self::path_taken`]
        // here would take the read lock a second time and deadlock.
        let mut mux = self.mux.write().expect("mux table is not poisoned");
        match mux.get_mut(path) {
            Some(existing) => {
                if existing.key != key {
                    return Err(RegistrationError::MuxKeyConflict {
                        path: path.to_string(),
                        expected: existing.key.clone(),
                        found: key.to_string(),
                    });
                }
                if existing.values.contains_key(&value) {
                    return Err(RegistrationError::DuplicateMuxValue {
                        path: path.to_string(),
                        value,
                    });
                }
                existing.values.insert(value, handler);
            }
            None => {
                if self.endpoints.contains_key(path) {
                    return Err(RegistrationError::DuplicatePath(path.to_string()));
                }
                let mut values = BTreeMap::new();
                values.insert(value, handler);
                mux.insert(
                    path.to_string(),
                    Mux {
                        key: key.to_string(),
                        values,
                    },
                );
            }
        }
        Ok(())
    }

    /// The key and instance values already registered for mux `path`.
    ///
    /// `None` when the path has no instances. The host-side `webhooks` object
    /// uses this to reject a conflict with the same message it would produce
    /// while buffering a first load, so one bad registration reads the same
    /// before and after a reload.
    pub fn mux_registrations(&self, path: &str) -> Option<(String, Vec<Option<String>>)> {
        let mux = self.mux.read().expect("mux table is not poisoned");
        let entry = mux.get(path)?;
        Some((entry.key.clone(), entry.values.keys().cloned().collect()))
    }

    /// Drop every mux registration, telling each instance to
    /// [`detach`](MuxEndpoint::detach) first.
    ///
    /// The host calls this when the configuration that registered these
    /// instances is rebuilt: they belong to the configuration that just went
    /// away. The same `(path, value)` may be registered again afterwards, with
    /// a fresh handler.
    pub fn clear_mux(&self) {
        // Drain under the lock, detach after releasing it: `detach` is
        // implementation code that may take locks of its own, and holding the
        // table would also block every dispatch that resolves an instance.
        let drained: Vec<Arc<dyn MuxEndpoint>> = {
            let mut mux = self.mux.write().expect("mux table is not poisoned");
            let handlers: Vec<Arc<dyn MuxEndpoint>> = mux
                .values()
                .flat_map(|mux_entry| mux_entry.values.values().cloned())
                .collect();
            mux.clear();
            handlers
        };
        for handler in drained {
            handler.detach();
        }
    }

    /// Register a remote method for one connection.
    ///
    /// Every push for `method` is `{"params": …}` merged with `template`. The
    /// same connection may register the same method more than once; the last
    /// template wins, as upstream's map assignment does.
    pub fn register_remote_method(
        &self,
        method: &str,
        target: Arc<dyn PushTarget>,
        template: ResponseTemplate,
    ) {
        let mut remote = self
            .remote
            .write()
            .expect("remote method table is not poisoned");
        let registrations = remote.entry(method.to_string()).or_default();
        registrations.retain(|registration| !Arc::ptr_eq(&registration.target, &target));
        registrations.push(RemoteRegistration { target, template });
    }

    /// Push `params` to every connection registered for `method`.
    ///
    /// Connections that have gone away are dropped as they are found, so the
    /// registrations do not accumulate. A method with no live connection is
    /// forgotten, so the next call reports it as unregistered.
    ///
    /// # Errors
    /// Returns [`ApiError::RemoteMethodNotRegistered`] if nobody ever
    /// registered `method`, or [`ApiError::NoActiveConnections`] if everybody
    /// who did has disconnected.
    pub fn call_remote_method(&self, method: &str, params: Value) -> Result<(), ApiError> {
        let mut remote = self
            .remote
            .write()
            .expect("remote method table is not poisoned");

        let registrations = remote
            .get_mut(method)
            .ok_or_else(|| ApiError::RemoteMethodNotRegistered(method.to_string()))?;
        registrations.retain(|registration| !registration.target.is_closed());
        if registrations.is_empty() {
            remote.remove(method);
            return Err(ApiError::NoActiveConnections(method.to_string()));
        }

        for registration in registrations.iter() {
            registration
                .target
                .push(registration.template.message(params.clone()));
        }
        Ok(())
    }

    /// Every registered path, sorted.
    ///
    /// Mux paths are included once each: to a client they are endpoints like
    /// any other, and `list_endpoints` is how it discovers them.
    pub fn endpoints(&self) -> Vec<String> {
        let mut paths: Vec<String> = self.endpoints.keys().cloned().collect();
        paths.extend(
            self.mux
                .read()
                .expect("mux table is not poisoned")
                .keys()
                .cloned(),
        );
        paths.sort();
        paths.dedup();
        paths
    }

    /// Run one request through the endpoint it names.
    ///
    /// `client` is taken by value because an endpoint that subscribes keeps it;
    /// the caller passes its own handle, so the clone costs one atomic
    /// increment per request.
    ///
    /// # Errors
    /// Returns [`ApiError::UnknownEndpoint`] if no endpoint or mux path matches
    /// the method, otherwise whatever the handler returned.
    ///
    /// A handler that panics is reported as [`ApiError::Internal`] and the
    /// internal-error hook is called: upstream catches the same exception, shuts
    /// the printer down, and still answers the client
    /// (`klippy/webhooks.py:271-276`). An [`ApiError::Internal`] a handler
    /// returns deliberately goes the same way.
    pub async fn dispatch(
        &self,
        request: &Request,
        client: Arc<dyn PushTarget>,
    ) -> Result<Value, ApiError> {
        let context = EndpointContext { api: self, client };

        // Resolve the mux instance under a read lock and release it: the lock
        // must not be held across the handler's `.await`, or a reload needing
        // the write lock would wait on a request that waits on the handler. The
        // owned `Arc` keeps the instance alive for the request without
        // borrowing the table.
        let mux_handler = {
            let mux = self.mux.read().expect("mux table is not poisoned");
            match mux.get(request.method()) {
                Some(mux) => Some(mux.resolve(request)?),
                None => None,
            }
        };

        let fut: EndpointFuture<'_> = if let Some(handler) = &mux_handler {
            handler.handle(request, &context)
        } else {
            match self.endpoints.get(request.method()) {
                Some(endpoint) => endpoint.handle(request, &context),
                None => {
                    return Err(ApiError::UnknownEndpoint(request.method().to_string()));
                }
            }
        };
        let outcome = catch_poll(fut).await;

        match outcome {
            Ok(result) => {
                if let Err(err) = &result {
                    if err.is_internal() {
                        self.report_internal(&err.to_string());
                    }
                }
                result
            }
            Err(_) => {
                let msg = format!("Internal Error on WebRequest: {}", request.method());
                self.report_internal(&msg);
                Err(ApiError::Internal(msg))
            }
        }
    }

    /// Tell the host a handler failed on its own account, if it wants to know.
    fn report_internal(&self, msg: &str) {
        if let Some(hook) = &self.internal_error {
            hook(msg);
        }
    }

    /// Whether `path` is already an endpoint or a mux path.
    fn path_taken(&self, path: &str) -> bool {
        self.endpoints.contains_key(path)
            || self
                .mux
                .read()
                .expect("mux table is not poisoned")
                .contains_key(path)
    }
}

impl Default for Api {
    fn default() -> Self {
        Self::new()
    }
}

// ===========================================================================
// Built-in endpoints
// ===========================================================================

/// `list_endpoints` — the registry describing itself.
struct ListEndpoints;

impl Endpoint for ListEndpoints {
    fn path(&self) -> &'static str {
        "list_endpoints"
    }

    fn handle<'a>(
        &'a self,
        _request: &'a Request,
        context: &'a EndpointContext<'a>,
    ) -> EndpointFuture<'a> {
        Box::pin(async move { Ok(json!({ "endpoints": context.api.endpoints() })) })
    }
}

// ===========================================================================
// Tests
// ===========================================================================

/// Run an endpoint future, turning a panic in a poll into an `Err` payload.
///
/// Replaces the synchronous `catch_unwind` the dispatcher used before handlers
/// could await: a panic cannot be caught across an `.await`, so it is caught
/// around each poll instead. This is what keeps
/// `Internal Error on WebRequest: <method>` working.
async fn catch_poll<'a>(
    mut fut: EndpointFuture<'a>,
) -> Result<Result<Value, ApiError>, Box<dyn std::any::Any + Send>> {
    std::future::poll_fn(move |cx| {
        match std::panic::catch_unwind(AssertUnwindSafe(|| fut.as_mut().poll(cx))) {
            Ok(poll) => poll.map(Ok),
            Err(payload) => std::task::Poll::Ready(Err(payload)),
        }
    })
    .await
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{Params, ResponseTemplate};
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    use std::sync::Mutex;

    // -----------------------------------------------------------------------
    // Test doubles
    // -----------------------------------------------------------------------

    /// A connection that records what was pushed to it.
    struct RecordingTarget {
        pushes: Mutex<Vec<Value>>,
        closed: AtomicBool,
    }

    impl RecordingTarget {
        fn new() -> Arc<Self> {
            Arc::new(Self {
                pushes: Mutex::new(Vec::new()),
                closed: AtomicBool::new(false),
            })
        }

        fn pushes(&self) -> Vec<Value> {
            self.pushes.lock().unwrap().clone()
        }

        fn close(&self) {
            self.closed.store(true, Ordering::SeqCst);
        }
    }

    impl PushTarget for RecordingTarget {
        fn is_closed(&self) -> bool {
            self.closed.load(Ordering::SeqCst)
        }

        fn push(&self, message: Value) {
            self.pushes.lock().unwrap().push(message);
        }
    }

    /// An endpoint that echoes the method it was called with.
    struct Echo;

    impl Endpoint for Echo {
        fn path(&self) -> &'static str {
            "echo"
        }

        fn handle<'a>(
            &'a self,
            request: &'a Request,
            _context: &'a EndpointContext<'a>,
        ) -> EndpointFuture<'a> {
            Box::pin(async move { Ok(json!({ "method": request.method() })) })
        }
    }

    /// An endpoint that always fails, for the error path.
    struct Failing;

    impl Endpoint for Failing {
        fn path(&self) -> &'static str {
            "failing"
        }

        fn handle<'a>(
            &'a self,
            _request: &'a Request,
            _context: &'a EndpointContext<'a>,
        ) -> EndpointFuture<'a> {
            Box::pin(async move { Err(ApiError::Internal("boom".to_string())) })
        }
    }

    /// An endpoint that refuses the request, for the non-shutdown path.
    struct CommandFailing;

    impl Endpoint for CommandFailing {
        fn path(&self) -> &'static str {
            "command_failing"
        }

        fn handle<'a>(
            &'a self,
            _request: &'a Request,
            _context: &'a EndpointContext<'a>,
        ) -> EndpointFuture<'a> {
            Box::pin(async move { Err(ApiError::CommandError("bad input".to_string())) })
        }
    }

    /// An endpoint that panics, for the internal-error path.
    struct Panicking;

    impl Endpoint for Panicking {
        fn path(&self) -> &'static str {
            "panicking"
        }

        fn handle<'a>(
            &'a self,
            _request: &'a Request,
            _context: &'a EndpointContext<'a>,
        ) -> EndpointFuture<'a> {
            Box::pin(async move { panic!("handler bug") })
        }
    }

    /// A mux handler that answers with its instance name.
    struct Instance(&'static str);

    impl MuxEndpoint for Instance {
        fn handle<'a>(
            &'a self,
            _request: &'a Request,
            _context: &'a EndpointContext<'a>,
        ) -> EndpointFuture<'a> {
            Box::pin(async move { Ok(json!(self.0)) })
        }
    }

    /// A mux handler that answers with its instance name and counts detaches.
    struct Detachable {
        answer: &'static str,
        detaches: Arc<AtomicUsize>,
    }

    impl MuxEndpoint for Detachable {
        fn handle<'a>(
            &'a self,
            _request: &'a Request,
            _context: &'a EndpointContext<'a>,
        ) -> EndpointFuture<'a> {
            Box::pin(async move { Ok(json!(self.answer)) })
        }

        fn detach(&self) {
            self.detaches.fetch_add(1, Ordering::SeqCst);
        }
    }

    /// A handler that records the connection it was given.
    struct ConnectionRecorder;

    impl Endpoint for ConnectionRecorder {
        fn path(&self) -> &'static str {
            "connection"
        }

        fn handle<'a>(
            &'a self,
            _request: &'a Request,
            context: &'a EndpointContext<'a>,
        ) -> EndpointFuture<'a> {
            Box::pin(async move { Ok(json!({ "closed": context.client.is_closed() })) })
        }
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    fn request(body: &str) -> Request {
        Request::parse(body.as_bytes()).expect("test body is a valid request")
    }

    /// Block on a future on a fresh current-thread runtime (tests are sync).
    fn block_on<F: std::future::Future>(fut: F) -> F::Output {
        tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("a current-thread runtime")
            .block_on(fut)
    }

    /// Dispatch `body` against `api` with a throwaway connection.
    fn dispatch(api: &Api, body: &str) -> Result<Value, ApiError> {
        let target: Arc<dyn PushTarget> = RecordingTarget::new();
        block_on(api.dispatch(&request(body), target))
    }

    fn template(fields: Value) -> ResponseTemplate {
        ResponseTemplate::new(fields.as_object().unwrap().clone())
    }

    // -----------------------------------------------------------------------
    // Registration
    // -----------------------------------------------------------------------

    #[test]
    fn test_a_new_registry_lists_only_the_built_in() {
        let api = Api::new();
        assert_eq!(api.endpoints(), vec!["list_endpoints"]);
    }

    #[test]
    fn test_paths_are_sorted_and_include_mux_paths() {
        let mut api = Api::new();
        api.register(Echo).unwrap();
        api.register_mux("sensors/dump", "sensor", Some("a"), Arc::new(Instance("a")))
            .unwrap();
        api.register_mux("sensors/dump", "sensor", Some("b"), Arc::new(Instance("b")))
            .unwrap();

        assert_eq!(
            api.endpoints(),
            vec!["echo", "list_endpoints", "sensors/dump"]
        );
    }

    #[test]
    fn test_a_path_cannot_be_registered_twice() {
        let mut api = Api::new();
        api.register(Echo).unwrap();

        assert_eq!(
            api.register(Echo).unwrap_err(),
            RegistrationError::DuplicatePath("echo".to_string())
        );
        // A mux path collides with a plain endpoint, and vice versa.
        assert_eq!(
            api.register_mux("echo", "sensor", Some("a"), Arc::new(Instance("a")))
                .unwrap_err(),
            RegistrationError::DuplicatePath("echo".to_string())
        );
        api.register_mux("sensors/dump", "sensor", None, Arc::new(Instance("d")))
            .unwrap();
        assert_eq!(
            api.register(ListEndpoints).unwrap_err(),
            RegistrationError::DuplicatePath("list_endpoints".to_string())
        );
    }

    #[test]
    fn test_mux_instances_must_agree_on_their_key() {
        let api = Api::new();
        api.register_mux("sensors/dump", "sensor", Some("a"), Arc::new(Instance("a")))
            .unwrap();

        assert_eq!(
            api.register_mux("sensors/dump", "name", Some("b"), Arc::new(Instance("b")))
                .unwrap_err(),
            RegistrationError::MuxKeyConflict {
                path: "sensors/dump".to_string(),
                expected: "sensor".to_string(),
                found: "name".to_string(),
            }
        );
        assert_eq!(
            api.register_mux("sensors/dump", "sensor", Some("a"), Arc::new(Instance("a")))
                .unwrap_err(),
            RegistrationError::DuplicateMuxValue {
                path: "sensors/dump".to_string(),
                value: Some("a".to_string()),
            }
        );
    }

    #[test]
    fn test_registration_errors_read_well() {
        assert_eq!(
            RegistrationError::DuplicatePath("info".to_string()).to_string(),
            "path already registered to an endpoint: 'info'"
        );
        assert_eq!(
            RegistrationError::DuplicateMuxValue {
                path: "p".to_string(),
                value: None
            }
            .to_string(),
            "mux endpoint 'p' default instance already registered"
        );
    }

    // -----------------------------------------------------------------------
    // Dispatch
    // -----------------------------------------------------------------------

    #[test]
    fn test_dispatch_calls_the_named_endpoint() {
        let mut api = Api::new();
        api.register(Echo).unwrap();

        assert_eq!(
            dispatch(&api, r#"{"method":"echo"}"#).unwrap(),
            json!({"method": "echo"})
        );
    }

    #[test]
    fn test_dispatch_reports_an_unknown_method() {
        let api = Api::new();
        assert_eq!(
            dispatch(&api, r#"{"method":"nope"}"#).unwrap_err(),
            ApiError::UnknownEndpoint("nope".to_string())
        );
    }

    #[test]
    fn test_dispatch_propagates_a_handler_failure() {
        let mut api = Api::new();
        api.register(Failing).unwrap();
        let hooks = Arc::new(Mutex::new(Vec::new()));
        {
            let hooks = Arc::clone(&hooks);
            api.set_internal_error_hook(Arc::new(move |msg: &str| {
                hooks.lock().unwrap().push(msg.to_string());
            }));
        }

        let error = dispatch(&api, r#"{"method":"failing"}"#).unwrap_err();
        assert!(error.is_internal());
        assert_eq!(error.to_string(), "boom");
        // An internal error is also reported to the host, which shuts the
        // printer down (upstream `webhooks.py:271-276`).
        assert_eq!(*hooks.lock().unwrap(), ["boom"]);
    }

    #[test]
    fn test_a_panicking_handler_is_an_internal_error_and_calls_the_hook() {
        let mut api = Api::new();
        api.register(Panicking).unwrap();
        let hooks = Arc::new(Mutex::new(Vec::new()));
        {
            let hooks = Arc::clone(&hooks);
            api.set_internal_error_hook(Arc::new(move |msg: &str| {
                hooks.lock().unwrap().push(msg.to_string());
            }));
        }

        let error = dispatch(&api, r#"{"method":"panicking"}"#).unwrap_err();
        assert_eq!(
            error,
            ApiError::Internal("Internal Error on WebRequest: panicking".to_string())
        );
        assert_eq!(
            *hooks.lock().unwrap(),
            ["Internal Error on WebRequest: panicking"]
        );
    }

    #[test]
    fn test_a_command_error_does_not_call_the_internal_hook() {
        // The client's own mistake must not take the printer down.
        let mut api = Api::new();
        api.register(CommandFailing).unwrap();
        let hooks = Arc::new(Mutex::new(Vec::new()));
        {
            let hooks = Arc::clone(&hooks);
            api.set_internal_error_hook(Arc::new(move |msg: &str| {
                hooks.lock().unwrap().push(msg.to_string());
            }));
        }

        let error = dispatch(&api, r#"{"method":"command_failing"}"#).unwrap_err();
        assert_eq!(error, ApiError::CommandError("bad input".to_string()));
        assert!(hooks.lock().unwrap().is_empty());
    }

    #[test]
    fn test_dispatch_hands_the_endpoint_its_connection() {
        let mut api = Api::new();
        api.register(ConnectionRecorder).unwrap();
        let target = RecordingTarget::new();

        let result =
            block_on(api.dispatch(&request(r#"{"method":"connection"}"#), target.clone())).unwrap();
        assert_eq!(result, json!({"closed": false}));

        // The endpoint is handed the connection itself, not a snapshot: what
        // the target reports changes underneath the same handle.
        target.close();
        let result =
            block_on(api.dispatch(&request(r#"{"method":"connection"}"#), target.clone())).unwrap();
        assert_eq!(result, json!({"closed": true}));
    }

    #[test]
    fn test_list_endpoints_describes_the_registry() {
        let mut api = Api::new();
        api.register(Echo).unwrap();

        assert_eq!(
            dispatch(&api, r#"{"method":"list_endpoints"}"#).unwrap(),
            json!({"endpoints": ["echo", "list_endpoints"]})
        );
    }

    // -----------------------------------------------------------------------
    // Mux dispatch
    // -----------------------------------------------------------------------

    #[test]
    fn test_mux_selects_an_instance_by_key() {
        let api = Api::new();
        api.register_mux("sensors/dump", "sensor", Some("a"), Arc::new(Instance("a")))
            .unwrap();
        api.register_mux("sensors/dump", "sensor", Some("b"), Arc::new(Instance("b")))
            .unwrap();

        assert_eq!(
            dispatch(&api, r#"{"method":"sensors/dump","params":{"sensor":"b"}}"#).unwrap(),
            json!("b")
        );
    }

    #[test]
    fn test_mux_requires_the_key_unless_a_default_exists() {
        let api = Api::new();
        api.register_mux("sensors/dump", "sensor", Some("a"), Arc::new(Instance("a")))
            .unwrap();

        // Missing, unknown, and non-string values all fail, and none of them
        // reaches a handler.
        assert_eq!(
            dispatch(&api, r#"{"method":"sensors/dump"}"#).unwrap_err(),
            ApiError::MissingArgument("sensor".to_string())
        );
        assert_eq!(
            dispatch(&api, r#"{"method":"sensors/dump","params":{"sensor":"z"}}"#).unwrap_err(),
            ApiError::UnknownMuxValue {
                key: "sensor".to_string(),
                value: "z".to_string()
            }
        );
        assert_eq!(
            dispatch(&api, r#"{"method":"sensors/dump","params":{"sensor":7}}"#).unwrap_err(),
            ApiError::UnknownMuxValue {
                key: "sensor".to_string(),
                value: "7".to_string()
            }
        );
    }

    #[test]
    fn test_mux_default_instance_makes_the_key_optional() {
        let api = Api::new();
        api.register_mux(
            "sensors/dump",
            "sensor",
            None,
            Arc::new(Instance("default")),
        )
        .unwrap();
        api.register_mux("sensors/dump", "sensor", Some("a"), Arc::new(Instance("a")))
            .unwrap();

        assert_eq!(
            dispatch(&api, r#"{"method":"sensors/dump"}"#).unwrap(),
            json!("default")
        );
        assert_eq!(
            dispatch(&api, r#"{"method":"sensors/dump","params":{"sensor":"a"}}"#).unwrap(),
            json!("a")
        );
        // The default does not make an unknown name acceptable.
        assert!(dispatch(&api, r#"{"method":"sensors/dump","params":{"sensor":"z"}}"#).is_err());
    }

    #[test]
    fn test_a_mux_endpoint_can_be_registered_after_the_registry_is_built() {
        // Registration is not build-time only: a configuration reload replaces
        // mux instances while the socket is being served.
        let api = Api::new();
        api.register_mux("p", "k", Some("a"), Arc::new(Instance("h1")))
            .unwrap();

        assert!(api.endpoints().contains(&"p".to_string()));
        assert_eq!(
            dispatch(&api, r#"{"method":"p","params":{"k":"a"}}"#).unwrap(),
            json!("h1")
        );
    }

    /// What a path already has, for the host-side checks (and their message).
    #[test]
    fn test_mux_registrations_reports_what_a_path_has() {
        let api = Api::new();
        assert_eq!(api.mux_registrations("p"), None);

        api.register_mux("p", "k", Some("a"), Arc::new(Instance("a")))
            .unwrap();
        api.register_mux("p", "k", None, Arc::new(Instance("d")))
            .unwrap();

        assert_eq!(
            api.mux_registrations("p"),
            Some(("k".to_string(), vec![None, Some("a".to_string())]))
        );
    }

    #[test]
    fn test_clear_mux_drops_every_instance() {
        let api = Api::new();
        api.register_mux("p", "k", Some("a"), Arc::new(Instance("a")))
            .unwrap();

        api.clear_mux();

        assert!(!api.endpoints().contains(&"p".to_string()));
        assert_eq!(
            dispatch(&api, r#"{"method":"p","params":{"k":"a"}}"#).unwrap_err(),
            ApiError::UnknownEndpoint("p".to_string())
        );
    }

    #[test]
    fn test_clear_mux_detaches_every_instance() {
        // The instances belong to the configuration going away, so each one is
        // told to stop before its registration is dropped.
        let api = Api::new();
        let detaches = Arc::new(AtomicUsize::new(0));
        for value in ["a", "b"] {
            api.register_mux(
                "p",
                "k",
                Some(value),
                Arc::new(Detachable {
                    answer: value,
                    detaches: Arc::clone(&detaches),
                }),
            )
            .unwrap();
        }

        api.clear_mux();

        assert_eq!(detaches.load(Ordering::SeqCst), 2);
    }

    #[test]
    fn test_a_cleared_path_can_be_registered_again_with_a_new_handler() {
        // Nothing of the old instance survives the clear: dispatch reaches the
        // fresh registration, not a stale snapshot of the old one.
        let api = Api::new();
        api.register_mux("p", "k", Some("a"), Arc::new(Instance("h1")))
            .unwrap();
        api.clear_mux();
        api.register_mux("p", "k", Some("a"), Arc::new(Instance("h2")))
            .unwrap();

        assert_eq!(
            dispatch(&api, r#"{"method":"p","params":{"k":"a"}}"#).unwrap(),
            json!("h2")
        );
    }

    // -----------------------------------------------------------------------
    // Remote methods
    // -----------------------------------------------------------------------

    #[test]
    fn test_remote_method_pushes_the_template_around_params() {
        let api = Api::new();
        let target = RecordingTarget::new();
        api.register_remote_method(
            "printer_event",
            target.clone(),
            template(json!({"method": "printer:event", "id": null})),
        );

        api.call_remote_method("printer_event", json!({"state": "ready"}))
            .unwrap();

        assert_eq!(
            target.pushes(),
            vec![json!({
                "method": "printer:event",
                "id": null,
                "params": {"state": "ready"}
            })]
        );
    }

    #[test]
    fn test_remote_method_pushes_to_every_registration() {
        let api = Api::new();
        let first = RecordingTarget::new();
        let second = RecordingTarget::new();
        api.register_remote_method("printer_event", first.clone(), ResponseTemplate::default());
        api.register_remote_method("printer_event", second.clone(), ResponseTemplate::default());

        api.call_remote_method("printer_event", json!({"state": "ready"}))
            .unwrap();

        assert_eq!(first.pushes().len(), 1);
        assert_eq!(second.pushes().len(), 1);
    }

    #[test]
    fn test_registering_a_remote_method_twice_replaces_the_template() {
        let api = Api::new();
        let target = RecordingTarget::new();
        api.register_remote_method("printer_event", target.clone(), template(json!({"a": 1})));
        api.register_remote_method("printer_event", target.clone(), template(json!({"b": 2})));

        api.call_remote_method("printer_event", json!({})).unwrap();

        // One push, not two: the second registration replaced the first.
        assert_eq!(target.pushes(), vec![json!({"b": 2, "params": {}})]);
    }

    #[test]
    fn test_remote_method_forgets_connections_that_went_away() {
        let api = Api::new();
        let target = RecordingTarget::new();
        api.register_remote_method("printer_event", target.clone(), ResponseTemplate::default());
        target.close();

        assert_eq!(
            api.call_remote_method("printer_event", json!({}))
                .unwrap_err(),
            ApiError::NoActiveConnections("printer_event".to_string())
        );
        // The dead registration was dropped, so the method is now unknown.
        assert_eq!(
            api.call_remote_method("printer_event", json!({}))
                .unwrap_err(),
            ApiError::RemoteMethodNotRegistered("printer_event".to_string())
        );
        assert!(target.pushes().is_empty());
    }

    #[test]
    fn test_remote_method_without_the_second_connection_still_pushes_to_the_first() {
        let api = Api::new();
        let live = RecordingTarget::new();
        let dead = RecordingTarget::new();
        api.register_remote_method("printer_event", live.clone(), ResponseTemplate::default());
        api.register_remote_method("printer_event", dead.clone(), ResponseTemplate::default());
        dead.close();

        api.call_remote_method("printer_event", json!({"n": 1}))
            .unwrap();

        assert_eq!(live.pushes(), vec![json!({"params": {"n": 1}})]);
        assert!(dead.pushes().is_empty());
    }

    #[test]
    fn test_calling_an_unregistered_remote_method_fails() {
        let api = Api::new();
        assert_eq!(
            api.call_remote_method("nobody", json!({})).unwrap_err(),
            ApiError::RemoteMethodNotRegistered("nobody".to_string())
        );
    }

    // -----------------------------------------------------------------------
    // Params plumbing
    // -----------------------------------------------------------------------

    #[test]
    fn test_endpoints_read_parameters_through_the_request() {
        // `Params` is reachable from a handler without any extra plumbing, so
        // an endpoint never has to re-parse the body.
        struct ReadCount;

        impl Endpoint for ReadCount {
            fn path(&self) -> &'static str {
                "read_count"
            }

            fn handle<'a>(
                &'a self,
                request: &'a Request,
                _context: &'a EndpointContext<'a>,
            ) -> EndpointFuture<'a> {
                Box::pin(async move {
                    let params: Params<'_> = request.params();
                    Ok(json!({ "count": params.get_int("count")? }))
                })
            }
        }

        let mut api = Api::new();
        api.register(ReadCount).unwrap();

        assert_eq!(
            dispatch(&api, r#"{"method":"read_count","params":{"count":3}}"#).unwrap(),
            json!({"count": 3})
        );
        assert_eq!(
            dispatch(&api, r#"{"method":"read_count"}"#).unwrap_err(),
            ApiError::MissingArgument("count".to_string())
        );
    }
}
