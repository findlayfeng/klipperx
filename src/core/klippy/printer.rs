// Printer — the machine a host runs.
//
// A printer is one machine: one host process runs exactly one, the machine its
// config describes, and the machine owns what it is made of — the printer
// objects of the config's sections — along with its own lifecycle: coming up,
// saying what state it is in, halting, and idling until something ends it.
// Starting a process, logging, serving the API, and building the next printer
// after a restart are the host's, not the machine's.
//
// The machine has a registry of parts and a two-phase lifecycle: objects are
// built and registered (the config-driven loader does that — see `load.rs`),
// then connected, then the printer idles until something asks it to exit. The
// machine has no parts of its own: the API server's `webhooks` is the host's,
// and the rest come from the config.
//
// Time comes from a reactor (`reactor.rs`), which the machine is handed rather
// than builds: a printer does not own a runtime, so whoever brings it up chooses
// what drives timers and what the clock reads.
//
// This module defines:
// - `PrinterState`: printer state categories
// - `StateMessage`: state message returned by `Printer::get_state_message`
// - `Printer`: the machine
//
// The events a printer fires at its handlers are [`KlippyEvent`]s, defined in
// `event/printer_bus.rs` and generated from `event/decl/`; the printer only
// registers and dispatches them.

use std::any::Any;
use std::collections::HashMap;
use std::future::Future;
use std::pin::Pin;
use std::sync::{Arc, Condvar, Mutex, MutexGuard};

use serde_json::Value;
use tracing::error;

use crate::core::klippy::config::access::AccessTracking;
use crate::core::klippy::config::value::ConfigValue;
use crate::core::klippy::error::{ConfigError, KlippyError};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::reactor::Reactor;

// ===========================================================================
// PrinterState
// ===========================================================================

/// Printer state categories reported by [`Printer::get_state_message`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PrinterState {
    /// During startup, before the config is fully loaded
    Startup,
    /// Config loaded, printer ready to accept commands
    Ready,
    /// Printer is shutting down (RESTART / FIRMWARE_RESTART)
    Shutdown,
    /// The printer is halted: it failed to come up
    Error,
}

impl PrinterState {
    /// Get the state category string ("startup", "ready", "shutdown", "error").
    ///
    /// These are the four values the `info` endpoint may report as `state`.
    pub fn as_category(&self) -> &'static str {
        match self {
            PrinterState::Startup => "startup",
            PrinterState::Ready => "ready",
            PrinterState::Shutdown => "shutdown",
            PrinterState::Error => "error",
        }
    }
}

// ===========================================================================
// StateMessage
// ===========================================================================

/// State message pair returned by [`Printer::get_state_message`].
///
/// `message` is for the user (`info`'s `state_message`); `category` is for the
/// client (`info`'s `state`).
#[derive(Debug, Clone)]
pub struct StateMessage {
    /// The full state message string
    pub message: String,
    /// The state category (startup/ready/shutdown/error)
    pub category: PrinterState,
}

// ===========================================================================
// Printer
// ===========================================================================

/// The message reported from construction until the printer is ready.
const MESSAGE_STARTUP: &str = "Starting up";
/// The message reported once the printer is ready.
const MESSAGE_READY: &str = "Printer is ready";

// ===========================================================================
// PrinterObject
// ===========================================================================

/// A printer object's connection step, as a future the machine can await.
///
/// A boxed future rather than an `async fn` because the machine holds its parts
/// as `Arc<dyn PrinterObject>`, and an `async fn` in a trait is not
/// object-safe. Nothing here is tokio's: the type is [`std::future::Future`],
/// so the machine needs no runtime of its own — whoever drives
/// [`Printer::bring_up`] brings the executor.
pub type ConnectFuture<'a> = Pin<Box<dyn Future<Output = Result<(), KlippyError>> + Send + 'a>>;

/// A future returned by [`PrinterObject::before_firmware_restart`].
///
/// Boxed for the same reason as [`ConnectFuture`], and carrying no result: a
/// part that cannot get ready is not a reason to abort the restart — the parts
/// are about to be dropped anyway.
pub type RestartFuture<'a> = Pin<Box<dyn Future<Output = ()> + Send + 'a>>;

/// The toolhead's part in a restart.
///
/// `GCodeDispatch.request_restart` (`klippy/gcode.py:352-362`) reaches the
/// toolhead by name to read the last print time, dwell, and wait for the queued
/// moves before the host exits. The dispatcher is core and the toolhead is an
/// extra, so the machine holds this small handle instead: the toolhead registers
/// it at connect and the dispatcher uses it if it is there.
pub trait RestartHooks: Send + Sync {
    /// The print time the planner has reached (`toolhead.get_last_move_time`).
    fn get_last_move_time(&self) -> f64;

    /// Advance the planner by `delay` seconds without moving
    /// (`toolhead.dwell`).
    fn dwell(&self, delay: f64);

    /// Wait for the queued moves to be planned (`toolhead.wait_moves`).
    fn wait_moves(&self);
}

/// A part of the machine.
///
/// One registered object is one printer object as far as `objects/list`,
/// `objects/query` and `objects/subscribe` are concerned: its status keys are
/// the fields a client may ask for. Upstream's objects opt in by defining
/// `get_status(eventtime)`; here the trait is the opt-in, so the machine's
/// registry holds exactly the objects a client can see.
///
/// `eventtime` is the printer's monotonic clock ([`Printer::eventtime`]), which
/// an object may use to date what it reports — upstream passes the reactor's
/// clock for the same reason.
pub trait PrinterObject: Any + Send + Sync {
    /// Report this object's status as a JSON object.
    ///
    /// Must not block: it is called on whatever thread asks, including the API
    /// connection that is waiting for the reply.
    fn get_status(&self, eventtime: f64) -> Value;

    /// Whether this part appears in `objects/list` and can be queried.
    ///
    /// Upstream's `objects/list` keeps only objects that define `get_status`,
    /// but `pins` is registered without one
    /// (`klippy/pins.py`: `PrinterPins` has no `get_status`). Here the trait is
    /// the registration and this is the filter, so a part that is in the
    /// registry but not client-visible overrides this to `false`. A query for
    /// such a name still answers `{}`, as upstream's "no `get_status`" path
    /// does.
    fn is_queryable(&self) -> bool {
        true
    }

    /// Connect this object: the second half of two-phase construction.
    ///
    /// Objects are built and registered first, then connected in registration
    /// order by [`Printer::bring_up`], so an object may look up another one
    /// that was registered before it without ever seeing a half-built one.
    /// Upstream has the same split but spells the second half as
    /// `klippy:connect` handlers; here it is a method, and the event is left to
    /// observers.
    ///
    /// The default is "nothing to do", which is what an object that needs no
    /// connection uses.
    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async { Ok::<(), KlippyError>(()) })
    }

    /// Prepare for a `FIRMWARE_RESTART` while the parts are still up.
    ///
    /// The host calls this on every object when the run loop ended because a
    /// `FIRMWARE_RESTART` was asked for, **before**
    /// [`Printer::reset_for_restart`] tears the parts down (`klippy.rs`).
    /// Upstream spells the same step as the `klippy:firmware_restart` event
    /// (`klippy/mcu.py:754`), and an MCU uses it to send the firmware's own
    /// `reset` on the **live** connection: the command has to reach the firmware
    /// before the restart drops the connection, and sending it here is what lets
    /// the reconnect after the reboot skip a second identify.
    ///
    /// The default is "nothing to do".
    fn before_firmware_restart<'a>(&'a self) -> RestartFuture<'a> {
        Box::pin(async {})
    }
}

/// The machine a host runs: its lifecycle, the state it reports, and the parts
/// that can report status of their own.
///
/// Shared rather than borrowed (`&self` throughout): an exit request arrives
/// from another thread — the API server, in the host — while the run loop is
/// idling, and status queries are answered on the connection that asked.
pub struct Printer {
    inner: Mutex<Inner>,
    /// Paired with `inner`, to wake the run loop when an exit is requested.
    exit_requested: Condvar,
    /// What status queries are dated from, and what timers are scheduled on.
    ///
    /// The machine is handed this rather than building one: a printer does not
    /// own a runtime, so the host decides what drives time (see `reactor.rs`).
    reactor: Arc<dyn Reactor>,
    /// The machine's parts, in registration order.
    ///
    /// Empty until a part registers itself: the machine has none of its own,
    /// and the API server's `webhooks` object is the host's, not the machine's.
    objects: Mutex<Vec<(String, Arc<dyn PrinterObject>)>>,
    /// Option values that override the **in-memory** config, per section.
    ///
    /// A restart reloads the same parsed config and never re-reads the file
    /// (`klippy.rs`), and [`Printer::reset_for_restart`] drops the objects built
    /// from it, so something a part learns about its own section at run time has
    /// to be kept here to reach the next bring-up. The loader applies these as it
    /// hands a section over (`load.rs`); the file on disk is the operator's.
    config_overrides: Mutex<HashMap<String, HashMap<String, ConfigValue>>>,
    /// The reads recorded by the config load that is currently loaded.
    ///
    /// [`Printer::load_config`] replaces it at the start of every load, and the
    /// `configfile` object, every [`ConfigWrapper`] and any part that reads its
    /// section later (an MCU at connect time) share the same handle. It is how
    /// the undefined-option check sees a read that happened after the load walk
    /// ([`AccessTracking`]).
    ///
    /// [`ConfigWrapper`]: crate::core::klippy::config::ConfigWrapper
    config_access: Mutex<Arc<AccessTracking>>,
}

/// A registered event handler.
///
/// The callback receives the event it was registered for, so one that needs the
/// payload can read it and one that does not can ignore the argument.
type EventHandler = Arc<dyn Fn(&KlippyEvent) + Send + Sync>;

struct Inner {
    /// The message the printer reports, for the user.
    message: String,
    /// What the printer is doing.
    category: PrinterState,
    /// Whether `invoke_shutdown` has run; the first message stands.
    shutdown: bool,
    /// What `run` returns once the loop ends, set by `request_exit`.
    run_result: Option<String>,
    /// Handlers per event name, in registration order.
    ///
    /// Keyed by [`KlippyEvent::name`] rather than by the enum so that
    /// [`KlippyEvent::Unknown`] — whose name is not known at compile time —
    /// shares the same table.
    handlers: HashMap<String, Vec<EventHandler>>,
    /// How many parts the registry held before the config was loaded — the
    /// host's own (`webhooks`). [`Printer::reset_for_restart`] keeps these and
    /// drops everything after them.
    host_objects: Option<usize>,
    /// Why the current bring-up is happening: the result `run` returned before a
    /// restart, or `None` for the first start. Upstream keeps the same fact in
    /// the printer's start args (`klippy/klippy.py:283`), and the MCU restart
    /// reads it to decide whether to reset the firmware (`mcu/restart.rs`).
    start_reason: Option<String>,
    /// The toolhead's restart handle, registered by it at connect.
    restart_hooks: Option<Arc<dyn RestartHooks>>,
    /// The host software version, as `M115` and `error_mcu` report it
    /// (upstream's `start_args['software_version']`).
    software_version: String,
}

/// Classify an MCU connect failure the way upstream's `_connect` except
/// clauses do (`klippy/klippy.py:143-155`): the short state message and the
/// details for `klippy:notify_mcu_error`.
fn classify_mcu_error(err: &KlippyError) -> (String, HashMap<String, Value>) {
    match err {
        KlippyError::Connection(reason) => {
            // Distinguish protocol-level failures from plain connect
            // failures (missing device, permission denied, etc.).
            let is_protocol = reason.contains("Protocol")
                || reason.contains("dictionary")
                || reason.contains("identify")
                || reason.contains("session");
            (
                if is_protocol {
                    "Protocol error"
                } else {
                    "MCU error during connect"
                }
                .to_string(),
                HashMap::from([("error".into(), serde_json::json!(reason))]),
            )
        }
        KlippyError::Protocol(reason) => (
            "Protocol error".to_string(),
            HashMap::from([("error".into(), serde_json::json!(reason))]),
        ),
        _ => (
            "MCU error during connect".to_string(),
            HashMap::from([("error".into(), serde_json::json!(err.to_string()))]),
        ),
    }
}

impl Printer {
    /// Create a printer that has not come up yet, with no parts registered.
    ///
    /// The reactor is taken rather than built so that the machine has no
    /// runtime of its own: the host passes one over the runtime it already
    /// runs ([`TokioReactor`](crate::core::klippy::reactor::TokioReactor)), and
    /// a test passes one it can step by hand
    /// ([`ManualReactor`](crate::core::klippy::reactor::ManualReactor)).
    ///
    /// Nothing reports status until a part registers itself — and the first one
    /// to do so is the host's API server, with the `webhooks` object upstream
    /// registers in `Printer.__init__`. A client cannot be connected before
    /// that: the host registers the server's objects and endpoints before it
    /// binds the socket.
    pub fn new(reactor: Arc<dyn Reactor>) -> Self {
        Self {
            inner: Mutex::new(Inner {
                message: MESSAGE_STARTUP.to_string(),
                category: PrinterState::Startup,
                shutdown: false,
                run_result: None,
                handlers: HashMap::new(),
                host_objects: None,
                start_reason: None,
                restart_hooks: None,
                software_version: env!("CARGO_PKG_VERSION").to_string(),
            }),
            exit_requested: Condvar::new(),
            reactor,
            objects: Mutex::new(Vec::new()),
            config_overrides: Mutex::new(HashMap::new()),
            config_access: Mutex::new(AccessTracking::shared()),
        }
    }

    /// The reads recorded for the config that is currently loaded.
    ///
    /// A part that kept its section and reads it after the load walk (an MCU
    /// parses `[mcu]` at connect time) wraps the section with this tracker, so
    /// its options are recorded before the undefined-option check runs.
    pub fn access_tracking(&self) -> Arc<AccessTracking> {
        Arc::clone(&self.config_access.lock().unwrap_or_else(|p| p.into_inner()))
    }

    /// Start recording a fresh config's reads.
    ///
    /// `load_config` calls this once per load, before any section is read, and
    /// hands the same handle to the `configfile` object.
    pub(crate) fn set_access_tracking(&self, access: Arc<AccessTracking>) {
        *self.config_access.lock().unwrap_or_else(|p| p.into_inner()) = access;
    }

    /// Override `<option> = <value>` in the section named `identifier` — in
    /// memory only, for the rest of the process.
    ///
    /// For a part that finds out at run time that an option cannot work as
    /// written: the next bring-up reads this section again, and the loader applies
    /// what is recorded here on top of it. The config file is untouched, and
    /// restarting the process reads it as written.
    pub fn override_config(&self, identifier: &str, option: &str, value: ConfigValue) {
        self.config_overrides
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .entry(identifier.to_string())
            .or_default()
            .insert(option.to_string(), value);
    }

    /// The overrides recorded for `identifier`, for the loader to apply.
    pub(crate) fn overrides_for(&self, identifier: &str) -> Vec<(String, ConfigValue)> {
        self.config_overrides
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(identifier)
            .map(|options| {
                options
                    .iter()
                    .map(|(option, value)| (option.clone(), value.clone()))
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Register the toolhead's restart handle (`RestartHooks`).
    ///
    /// Called by the toolhead at connect; replaced by the next bring-up's
    /// toolhead. Kept out of [`PrinterObject`] because it is not a client-facing
    /// status.
    pub fn register_restart_hooks(&self, hooks: Arc<dyn RestartHooks>) {
        self.lock().restart_hooks = Some(hooks);
    }

    /// The host software version.
    ///
    /// Upstream reads `start_args['software_version']`; the host sets it from
    /// its [`crate::core::klippy::api::StartArgs`] at startup, and it defaults
    /// to this crate's version so tests and the `M115` reply agree.
    pub fn software_version(&self) -> String {
        self.lock().software_version.clone()
    }

    /// Set the host software version (called once by the host at startup).
    pub fn set_software_version(&self, version: impl Into<String>) {
        self.lock().software_version = version.into();
    }

    /// The toolhead's restart handle, if one registered itself.
    pub fn restart_hooks(&self) -> Option<Arc<dyn RestartHooks>> {
        self.lock().restart_hooks.clone()
    }

    /// The reactor this printer was built with.
    ///
    /// Upstream's `get_reactor`: objects ask the printer for it to schedule a
    /// timer (121 call sites there), and it is the same clock
    /// [`Printer::eventtime`] reports.
    pub fn reactor(&self) -> Arc<dyn Reactor> {
        Arc::clone(&self.reactor)
    }

    /// Seconds since the reactor was built, the clock status is dated with.
    ///
    /// Monotonic and near zero at startup, which is what a client needs to tell
    /// one report from the next. This is the reactor's `monotonic()`
    /// (`klippy/reactor.py:111`), read through the one clock the machine has;
    /// the machine keeps no time of its own.
    pub fn eventtime(&self) -> f64 {
        self.reactor.monotonic()
    }

    /// Register a part of the machine.
    ///
    /// Registration order is the order `objects/list` reports, the order
    /// [`Printer::bring_up`] connects in, and the order upstream's registry
    /// uses.
    ///
    /// # Errors
    /// Returns [`ConfigError`] if `name` is already taken: two parts answering
    /// to one name is a wiring mistake in klippy that no client can provoke.
    /// Upstream raises its config error here too
    /// (`klippy/klippy.py:71-73`).
    pub fn add_object(
        &self,
        name: &str,
        object: Arc<dyn PrinterObject>,
    ) -> Result<(), ConfigError> {
        let mut objects = self.objects.lock().unwrap_or_else(|p| p.into_inner());
        if objects.iter().any(|(taken, _)| taken == name) {
            return Err(ConfigError::new(format!(
                "Printer object '{name}' already created"
            )));
        }
        objects.push((name.to_string(), object));
        Ok(())
    }

    /// Remember how many parts the host registered before the config is loaded.
    ///
    /// [`Printer::load_config`] calls this first; everything from that index on
    /// is the config's, and a restart drops it while keeping the host's parts
    /// (the API server's `webhooks`). Only the first call counts, so reloading
    /// the config does not move the boundary.
    pub(crate) fn mark_host_objects(&self) {
        let base = self.objects.lock().unwrap_or_else(|p| p.into_inner()).len();
        let mut inner = self.lock();
        if inner.host_objects.is_none() {
            inner.host_objects = Some(base);
        }
    }

    /// The names of the registered objects, in registration order.
    pub fn objects(&self) -> Vec<String> {
        self.objects
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
            .map(|(name, _)| name.clone())
            .collect()
    }

    /// The names of the objects a client may query, in registration order.
    ///
    /// What `objects/list` reports. A registered part that reports no status
    /// (`pins`) is left out, which is upstream's `hasattr(o, 'get_status')`
    /// filter.
    pub fn queryable_objects(&self) -> Vec<String> {
        self.objects
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
            .filter(|(_, object)| object.is_queryable())
            .map(|(name, _)| name.clone())
            .collect()
    }

    /// Look up one registered object and return it as its concrete type.
    ///
    /// The escape hatch for the few places that need to *use* an object rather
    /// than ask it for status — a resource reaching `pins`, for instance. It is
    /// the Rust spelling of upstream's `printer.lookup_object('pins')`, and it
    /// returns `None` for a name that is unregistered or is a different type.
    pub fn lookup_object_as<T: PrinterObject>(&self, name: &str) -> Option<Arc<T>> {
        let object = self.lookup_object(name)?;
        let any: Arc<dyn Any + Send + Sync> = object;
        any.downcast::<T>().ok()
    }

    /// Every registered object a module name selects, as its concrete type.
    ///
    /// [`Printer::lookup_objects`] with a downcast, for the callers that need to
    /// *use* the objects rather than ask them for status. The one that matters
    /// here is finding an `McuObject` by its chip name: `lookup_objects(Some("mcu"))`
    /// returns `mcu` and `mcu <name>` (whose registered names differ from their
    /// chip names), so a resource filters on the chip name it was built for.
    pub fn lookup_objects_as<T: PrinterObject>(
        &self,
        module: Option<&str>,
    ) -> Vec<(String, Arc<T>)> {
        self.lookup_objects(module)
            .into_iter()
            .filter_map(|(name, object)| {
                let any: Arc<dyn Any + Send + Sync> = object;
                any.downcast::<T>().ok().map(|typed| (name, typed))
            })
            .collect()
    }

    /// Look up one registered object by name.
    ///
    /// `None` for a name nobody registered. The handle is cloned out and the
    /// lock released, so the caller may use the object freely — including
    /// asking it for status, or asking the printer something else it needs.
    pub fn lookup_object(&self, name: &str) -> Option<Arc<dyn PrinterObject>> {
        self.objects
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
            .find(|(taken, _)| taken == name)
            .map(|(_, object)| Arc::clone(object))
    }

    /// Look up one registered object, reporting a config error when absent
    /// (upstream's `lookup_object` with no default: `Unknown config object
    /// 'x'`, `klippy/klippy.py:79-82`).
    ///
    /// # Errors
    /// Returns [`ConfigError`] naming the object when nothing registered it.
    pub fn require_object(&self, name: &str) -> Result<Arc<dyn PrinterObject>, ConfigError> {
        self.lookup_object(name)
            .ok_or_else(|| ConfigError::new(format!("Unknown config object '{name}'")))
    }

    /// [`Printer::require_object`] with the concrete type.
    ///
    /// # Errors
    /// Returns [`ConfigError`] when the name is absent or is a different type.
    pub fn require_object_as<T: PrinterObject>(&self, name: &str) -> Result<Arc<T>, ConfigError> {
        self.lookup_object_as::<T>(name)
            .ok_or_else(|| ConfigError::new(format!("Unknown config object '{name}'")))
    }

    /// Every registered object a module name selects, in registration order.
    ///
    /// Upstream's `lookup_objects(module)` (`klippy/klippy.py:81-88`): with a
    /// module name, the object registered under exactly that name comes first
    /// (when there is one), then every object whose name starts with
    /// `"<module> "`. With `None`, every object. This is the reflection the
    /// modules use to find their siblings without naming each one:
    /// `lookup_objects('mcu')` is `mcu` plus `mcu <name>`; `statistics` walks
    /// `None` and picks the objects that offer `stats`.
    ///
    /// The handles are cloned out and the lock released, so a caller may ask
    /// each object for status while it holds the list.
    pub fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, Arc<dyn PrinterObject>)> {
        let objects = self.objects.lock().unwrap_or_else(|p| p.into_inner());
        match module {
            None => objects.clone(),
            Some(module) => {
                let prefix = format!("{module} ");
                let mut found: Vec<(String, Arc<dyn PrinterObject>)> = objects
                    .iter()
                    .filter(|(name, _)| name == module)
                    .cloned()
                    .collect();
                found.extend(
                    objects
                        .iter()
                        .filter(|(name, _)| name.starts_with(&prefix))
                        .cloned(),
                );
                found
            }
        }
    }

    /// Every queryable object's status, keyed by name, dated once.
    ///
    /// The snapshot a template's `printer.objects` iterates: the names come from
    /// [`Printer::lookup_objects`] and each status from
    /// [`PrinterObject::get_status`]. Like [`Printer::status_of`], the calls are
    /// made without the registry lock — an object may ask the printer something
    /// of its own — so the object handles are collected first and queried after.
    ///
    /// Objects that report nothing (`is_queryable` is false) are left out, which
    /// is what upstream's `hasattr(o, 'get_status')` filter does too.
    pub fn statuses(&self, eventtime: f64) -> serde_json::Map<String, Value> {
        let objects: Vec<(String, Arc<dyn PrinterObject>)> = self
            .lookup_objects(None)
            .into_iter()
            .filter(|(_, object)| object.is_queryable())
            .collect();
        objects
            .into_iter()
            .map(|(name, object)| (name, object.get_status(eventtime)))
            .collect()
    }

    /// Ask one registered object for its status.
    ///
    /// Returns `None` for a name nobody registered; the caller decides what an
    /// unknown object means (upstream answers an empty status rather than
    /// failing the request).
    ///
    /// The object is called without the registry's lock held: an object may ask
    /// the printer something of its own — the API server's `webhooks` object
    /// reads the state this very type keeps — and a lock held across it would
    /// deadlock there.
    pub fn status_of(&self, name: &str, eventtime: f64) -> Option<Value> {
        Some(self.lookup_object(name)?.get_status(eventtime))
    }

    /// Get the current state message and category.
    pub fn get_state_message(&self) -> StateMessage {
        let inner = self.lock();
        StateMessage {
            message: inner.message.clone(),
            category: inner.category.clone(),
        }
    }

    /// Why the machine is being brought up: the result `run` returned before a
    /// restart, or `None` for the first start.
    pub fn start_reason(&self) -> Option<String> {
        self.lock().start_reason.clone()
    }

    /// Register a callback for a specific event.
    ///
    /// Handlers run in registration order, on the thread that fires the event,
    /// and receive the event itself: a handler that needs the payload reads it
    /// from the variant, and one that does not ignores the argument. Like the
    /// printer's own callbacks upstream, they must not block. Boxed because a
    /// handler list holds many of them.
    pub fn register_event_handler(
        &self,
        event: KlippyEvent,
        callback: Box<dyn Fn(&KlippyEvent) + Send + Sync>,
    ) {
        self.lock()
            .handlers
            .entry(event.name().to_string())
            .or_default()
            .push(Arc::from(callback));
    }

    /// Fire an event at the handlers registered for it.
    ///
    /// The handlers are collected under the lock and run without it: a handler
    /// may itself ask the printer to exit, and a lock held across callbacks
    /// would deadlock there.
    ///
    /// Each handler is isolated, as upstream's `try`/`except` isolates the
    /// handlers of `klippy:shutdown` and `klippy:analyze_shutdown`: a panic is
    /// logged and the remaining handlers still run. An event with no registered
    /// handlers is ignored, except that an
    /// [`Unknown`](KlippyEvent::Unknown) event is warned about, since it means
    /// upstream sent a name this build does not know.
    pub fn send_event(&self, event: &KlippyEvent) {
        let handlers = match self.lock().handlers.get(event.name()) {
            Some(handlers) => handlers.clone(),
            None => {
                if matches!(event, KlippyEvent::Unknown { .. }) {
                    tracing::warn!(event = event.name(), "unhandled unknown event");
                }
                return;
            }
        };
        for handler in handlers {
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                handler(event);
            }));
            if let Err(panic) = result {
                error!(event = event.name(), "event handler panicked: {panic:?}");
            }
        }
    }

    /// Let every part prepare for a firmware restart, before it is torn down.
    ///
    /// See [`PrinterObject::before_firmware_restart`]. The objects are collected
    /// under the lock and awaited without it, the way [`Printer::send_event`]
    /// collects its handlers: a part may reach for another object while it
    /// prepares.
    pub async fn prepare_firmware_restart(&self) {
        let objects: Vec<Arc<dyn PrinterObject>> = self
            .objects
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .iter()
            .map(|(_, object)| Arc::clone(object))
            .collect();
        for object in objects {
            object.before_firmware_restart().await;
        }
    }

    /// Fire `klippy:notify_mcu_error` for an MCU connection failure.
    ///
    /// Upstream sends this event during `_connect` when the identify handshake
    /// or MCU connection fails, giving downstream handlers (e.g. the
    /// `error_mcu` module) a chance to enrich the error message before
    /// shutdown. The event carries a short `msg` describing the failure class
    /// and a `details` map with the raw error text.
    fn notify_mcu_error(&self, err: &KlippyError) {
        let (msg, details) = classify_mcu_error(err);
        self.send_event(&KlippyEvent::KlippyNotifyMcuError { msg, details });
    }

    /// Halt the printer with a message for the user.
    ///
    /// The printer moves to the `shutdown` category and fires
    /// `klippy:shutdown` and then `klippy:analyze_shutdown`, the latter with
    /// the message and empty details, as upstream does. Halting does not end
    /// the run loop: the printer stays up so that clients can still read why it
    /// stopped, until something asks it to exit. The first message stands;
    /// later ones are ignored, so the shutdown events fire once.
    pub fn invoke_shutdown(&self, msg: &str) {
        self.invoke_shutdown_with(msg, HashMap::new());
    }

    /// [`Printer::invoke_shutdown`] with the structured details upstream
    /// passes to `klippy:analyze_shutdown` (`klippy/klippy.py:204-220`).
    ///
    /// The MCU shutdown path uses this: it reports the generic message
    /// `"MCU shutdown"` and puts the MCU name, reason and event type in the
    /// details, so that `error_mcu` can build the user-facing text.
    pub fn invoke_shutdown_with(&self, msg: &str, details: HashMap<String, Value>) {
        {
            let mut inner = self.lock();
            if inner.shutdown {
                return;
            }
            inner.shutdown = true;
            inner.message = msg.to_string();
            inner.category = PrinterState::Shutdown;
        }

        error!("Transition to shutdown state: {msg}");
        self.send_event(&KlippyEvent::KlippyShutdown);
        self.send_event(&KlippyEvent::KlippyAnalyzeShutdown {
            msg: msg.to_string(),
            details,
        });
    }

    /// Replace the state message, for handlers that enrich a shutdown.
    ///
    /// Upstream's `Printer.update_error_msg` (`klippy/klippy.py:63-69`): only
    /// the message the shutdown was reported with is replaced, and only while
    /// the printer is neither ready nor starting up. The `error_mcu` module
    /// uses it to append its hints.
    pub fn update_error_msg(&self, oldmsg: &str, newmsg: &str) {
        let mut inner = self.lock();
        if inner.message != oldmsg
            || matches!(inner.category, PrinterState::Ready | PrinterState::Startup)
        {
            return;
        }
        inner.message = newmsg.to_string();
        error!("{newmsg}");
    }

    /// Put the printer in the `error` category with a message.
    ///
    /// Upstream's `_set_state` (`klippy/klippy.py:57-62`): the message is what
    /// the user reads and the category is `error`, **not** `shutdown` — the
    /// printer is halted but can be brought back with `RESTART`. A config the
    /// loader rejected and an MCU error during connect both land here; only a
    /// failure of klippy itself is an [`Printer::invoke_shutdown`].
    ///
    /// The first state stands: once the printer is ready or already in an error
    /// state, a later error does not overwrite it, as upstream's `_set_state`
    /// only writes from `startup`/`ready`.
    pub fn set_error_state(&self, msg: &str) {
        let mut inner = self.lock();
        if matches!(inner.category, PrinterState::Startup | PrinterState::Ready) {
            inner.message = msg.to_string();
            inner.category = PrinterState::Error;
        }
        error!("Printer error: {msg}");
    }

    /// Ask the printer to leave its run loop.
    ///
    /// The `result` decides what happens once the loop ends:
    /// - `"exit"` / `"error_exit"`: the host process terminates
    /// - `"firmware_restart"`: the host starts the printer again
    pub fn request_exit(&self, result: &str) {
        {
            let mut inner = self.lock();
            if inner.run_result.is_none() {
                inner.run_result = Some(result.to_string());
            }
        }
        // Wake the run loop, whether or not this call is the one that set the
        // result: a second request must not find the loop asleep on it.
        self.exit_requested.notify_all();
    }

    /// Idle until the printer is asked to exit, and return what it was asked
    /// with.
    ///
    /// This is the run loop without the bring-up: callers await
    /// [`Printer::bring_up`] first, then block here — which is why bring-up is
    /// a future while this stays a plain blocking call. Blocks until
    /// [`Printer::request_exit`] is called, then fires `klippy:disconnect` —
    /// plus `klippy:firmware_restart` when that is what the exit asked for.
    ///
    /// A printer that was already asked to exit does not wait.
    ///
    /// Returns the result the exit was requested with.
    pub fn run(&self) -> String {
        let result = self.wait_for_exit();

        // What upstream does when the reactor loop ends: a run that was asked
        // for a firmware restart says so before everyone is told the printer is
        // gone.
        if result == "firmware_restart" {
            self.send_event(&KlippyEvent::KlippyFirmwareRestart);
        }
        self.send_event(&KlippyEvent::KlippyDisconnect);

        result
    }

    /// Drop the parts the config loaded, and the event handlers they registered.
    ///
    /// This is what shuts their devices down. A transport can have a blocking
    /// read parked on a worker thread — an MCU's receive task always runs one —
    /// and only dropping the part releases it (see [`Mcu`](crate::core::klippy::mcu::Mcu)'s
    /// `Drop`). The host calls this before the runtime it built is dropped:
    /// waiting for the whole `Printer` to drop would leave that read parked,
    /// because the API endpoints (and the subscription timers they own) keep the
    /// printer alive past the run loop.
    ///
    /// The reactor and the host's own objects are kept: they belong to the
    /// process, not to the config.
    pub fn teardown(&self) {
        let keep = self.lock().host_objects.unwrap_or(0);
        self.objects
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .truncate(keep);
        self.lock().handlers.clear();
    }

    /// Take the machine down so it can be brought up again on the same printer.
    ///
    /// Drops every part the config loaded — which shuts their devices down and
    /// closes them (see [`Printer::teardown`]) — and keeps the parts the host
    /// registered before the config, i.e. the API server's `webhooks`. The state
    /// goes back to `Startup`, the exit request is cleared, and the event
    /// handlers are forgotten (the parts that registered them are gone).
    /// [`Printer::load_config`] + [`Printer::bring_up`] + [`Printer::run`] can
    /// then run again on the same `Arc<Printer>`, which is what keeps the
    /// endpoints and any attachment valid across a restart.
    ///
    /// The reactor and the host's parts are kept: they belong to the process,
    /// not to the config.
    ///
    /// `reason` is what the next bring-up reports as [`Printer::start_reason`];
    /// a `firmware_restart` is what makes an MCU reset its firmware rather than
    /// just reconnect (`mcu/restart.rs`).
    pub fn reset_for_restart(&self, reason: &str) {
        self.teardown();

        let mut inner = self.lock();
        inner.message = MESSAGE_STARTUP.to_string();
        inner.category = PrinterState::Startup;
        inner.shutdown = false;
        inner.run_result = None;
        inner.start_reason = Some(reason.to_string());
        // The handle points at the toolhead being dropped; the next toolhead
        // registers its own at connect.
        inner.restart_hooks = None;
    }

    /// Lock the state.
    ///
    /// A poisoned lock means an earlier holder panicked. Every critical section
    /// here only reads or assigns fields, so the state is still consistent and
    /// the printer keeps going: losing a printer to an unrelated panic in a
    /// handler would be worse than reading an unchanged flag.
    fn lock(&self) -> MutexGuard<'_, Inner> {
        self.inner
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// Bring the machine up: connect its parts, then report ready.
    ///
    /// Upstream's `_connect`. Objects are connected in registration order — the
    /// second half of two-phase construction — and an object that fails to
    /// connect halts the printer with the reason, as upstream's `_connect`
    /// does. The state is re-checked after every step: a printer that shut down
    /// while connecting never becomes ready, and the `klippy:connect` event is
    /// only fired once every object is up. `klippy:mcu_identify` is fired first,
    /// before any object connects.
    pub async fn bring_up(&self) {
        // A printer that was already halted — a config the loader rejected, a
        // shutdown that raced this call — has nothing to bring up. Connecting
        // its parts anyway would reach a device for a machine that is stopped.
        if self.category() != PrinterState::Startup {
            return;
        }

        // Upstream fires this right after the config is read and before the
        // objects connect (`klippy/klippy.py`: `_connect`), so an object that
        // wants to react to the config does so before the MCUs are up.
        self.send_event(&KlippyEvent::KlippyMcuIdentify);

        for (name, object) in self.registry() {
            if let Err(err) = object.connect().await {
                // Every connect failure puts the printer in the `error` state,
                // which a `RESTART` can fix; none of them halts the host. Upstream
                // classifies them in `_connect`'s except clauses
                // (`klippy/klippy.py:136-158`): a protocol error and an MCU
                // connect error are reported with their own short state message
                // (which `error_mcu` then expands), a config error with its text.
                if (name == "mcu" || name.starts_with("mcu "))
                    && !matches!(err, KlippyError::Config(_))
                {
                    // The state is set before the event, as upstream's
                    // `_set_state(msg)` runs before `send_event` — the
                    // `error_mcu` handler reads the state message.
                    self.set_error_state(&classify_mcu_error(&err).0);
                    self.notify_mcu_error(&err);
                } else {
                    self.set_error_state(&format!("{name}: {err}"));
                }
                return;
            }
            if self.category() != PrinterState::Startup {
                return;
            }
        }

        self.send_event(&KlippyEvent::KlippyConnect);

        {
            let mut inner = self.lock();
            if inner.category != PrinterState::Startup {
                return;
            }
            inner.message = MESSAGE_READY.to_string();
            inner.category = PrinterState::Ready;
        }

        self.send_event(&KlippyEvent::KlippyReady);
    }

    /// The registry as a snapshot, so connecting does not hold its lock across
    /// an await.
    fn registry(&self) -> Vec<(String, Arc<dyn PrinterObject>)> {
        self.objects
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }

    /// What the printer is doing.
    fn category(&self) -> PrinterState {
        self.lock().category.clone()
    }

    /// Block until an exit has been requested, and return its result.
    ///
    /// An exit requested before the loop started is found here immediately, so
    /// a printer that was already asked to stop does not wait for another one.
    fn wait_for_exit(&self) -> String {
        let mut inner = self.lock();
        while inner.run_result.is_none() {
            inner = self
                .exit_requested
                .wait(inner)
                .unwrap_or_else(|poison| poison.into_inner());
        }
        inner
            .run_result
            .clone()
            .expect("the loop exits only once a result is set")
    }
}

// A `Default` implementation is deliberately absent: building a printer means
// choosing what drives its clock, and a type that picked one for the caller
// would be hiding the runtime it promised not to depend on.

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::reactor::ManualReactor;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::mpsc;
    use std::sync::Weak;
    use std::thread;

    /// A printer on a clock the test controls.
    ///
    /// No runtime: the machine's tests exercise the lifecycle, not timers, and
    /// a reactor the test can step is deterministic when one is added.
    fn new_printer() -> Printer {
        Printer::new(Arc::new(ManualReactor::new()))
    }

    /// Register a handler that records the name it was called for.
    fn record(
        printer: &Printer,
        name: &'static str,
        event: KlippyEvent,
        log: &Arc<Mutex<Vec<&'static str>>>,
    ) {
        let log = Arc::clone(log);
        printer.register_event_handler(
            event,
            Box::new(move |_| log.lock().unwrap_or_else(|p| p.into_inner()).push(name)),
        );
    }

    /// An object whose firmware-restart hook records that it ran.
    struct Restarts(Arc<Mutex<Vec<&'static str>>>);

    impl PrinterObject for Restarts {
        fn get_status(&self, _eventtime: f64) -> Value {
            serde_json::json!({})
        }
        fn before_firmware_restart<'a>(&'a self) -> RestartFuture<'a> {
            Box::pin(async move {
                self.0
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner())
                    .push("ran");
            })
        }
    }

    #[tokio::test]
    async fn test_prepare_firmware_restart_awaits_every_part() {
        // The hook runs while the parts are still up — it is where an MCU sends
        // the firmware's `reset` on the live connection (`mcu/object.rs`) — and
        // in registration order, like `bring_up`.
        let printer = new_printer();
        let seen = Arc::new(Mutex::new(Vec::new()));
        printer
            .add_object("a", Arc::new(Restarts(Arc::clone(&seen))))
            .unwrap();
        printer
            .add_object("b", Arc::new(Restarts(Arc::clone(&seen))))
            .unwrap();

        printer.prepare_firmware_restart().await;

        assert_eq!(*seen.lock().unwrap(), vec!["ran", "ran"]);
    }

    #[test]
    fn test_state_categories_are_the_wire_names() {
        assert_eq!(PrinterState::Startup.as_category(), "startup");
        assert_eq!(PrinterState::Ready.as_category(), "ready");
        assert_eq!(PrinterState::Shutdown.as_category(), "shutdown");
        assert_eq!(PrinterState::Error.as_category(), "error");
    }

    #[test]
    fn test_event_names_are_the_wire_names() {
        assert_eq!(KlippyEvent::KlippyMcuIdentify.name(), "klippy:mcu_identify");
        assert_eq!(KlippyEvent::KlippyConnect.name(), "klippy:connect");
        assert_eq!(KlippyEvent::KlippyReady.name(), "klippy:ready");
        assert_eq!(KlippyEvent::KlippyShutdown.name(), "klippy:shutdown");
        assert_eq!(KlippyEvent::KlippyDisconnect.name(), "klippy:disconnect");
        assert_eq!(
            KlippyEvent::KlippyFirmwareRestart.name(),
            "klippy:firmware_restart"
        );
        assert_eq!(
            KlippyEvent::KlippyAnalyzeShutdown {
                msg: String::new(),
                details: HashMap::new(),
            }
            .name(),
            "klippy:analyze_shutdown"
        );
        assert_eq!(
            KlippyEvent::Unknown {
                name: "future:event".to_string(),
                params: HashMap::new(),
            }
            .name(),
            "future:event"
        );
    }

    #[test]
    fn test_a_new_printer_is_starting_up() {
        let printer = new_printer();

        let state = printer.get_state_message();
        assert_eq!(state.message, MESSAGE_STARTUP);
        assert_eq!(state.category, PrinterState::Startup);
    }

    #[tokio::test]
    async fn test_bring_up_comes_up_ready_and_reports_it() {
        let printer = new_printer();
        let ready = Arc::new(AtomicUsize::new(0));
        let counter = Arc::clone(&ready);
        printer.register_event_handler(
            KlippyEvent::KlippyReady,
            Box::new(move |_| {
                counter.fetch_add(1, Ordering::SeqCst);
            }),
        );

        printer.bring_up().await;

        assert_eq!(ready.load(Ordering::SeqCst), 1);
        let state = printer.get_state_message();
        assert_eq!(state.message, MESSAGE_READY);
        assert_eq!(state.category, PrinterState::Ready);
    }

    #[tokio::test]
    async fn test_the_lifecycle_events_fire_in_order() {
        let printer = new_printer();
        let log = Arc::new(Mutex::new(Vec::new()));
        record(
            &printer,
            "klippy:mcu_identify",
            KlippyEvent::KlippyMcuIdentify,
            &log,
        );
        record(&printer, "klippy:connect", KlippyEvent::KlippyConnect, &log);
        record(&printer, "klippy:ready", KlippyEvent::KlippyReady, &log);
        record(
            &printer,
            "klippy:firmware_restart",
            KlippyEvent::KlippyFirmwareRestart,
            &log,
        );
        record(
            &printer,
            "klippy:disconnect",
            KlippyEvent::KlippyDisconnect,
            &log,
        );
        printer.request_exit("firmware_restart");

        printer.bring_up().await;
        assert_eq!(printer.run(), "firmware_restart");
        assert_eq!(
            *log.lock().unwrap(),
            [
                "klippy:mcu_identify",
                "klippy:connect",
                "klippy:ready",
                "klippy:firmware_restart",
                "klippy:disconnect",
            ]
        );
    }

    #[test]
    fn test_handlers_run_in_registration_order() {
        let printer = new_printer();
        let order = Arc::new(Mutex::new(Vec::new()));
        for name in ["first", "second"] {
            let order = Arc::clone(&order);
            printer.register_event_handler(
                KlippyEvent::KlippyReady,
                Box::new(move |_| order.lock().unwrap().push(name)),
            );
        }

        printer.send_event(&KlippyEvent::KlippyReady);

        assert_eq!(*order.lock().unwrap(), ["first", "second"]);
    }

    #[tokio::test]
    async fn test_run_waits_for_an_exit_request_from_another_thread() {
        let printer = Arc::new(new_printer());
        printer.bring_up().await;
        assert_eq!(printer.get_state_message().category, PrinterState::Ready);

        let (started_tx, started_rx) = mpsc::channel();
        let runner = {
            let printer = Arc::clone(&printer);
            thread::spawn(move || {
                started_tx.send(()).expect("the test is still listening");
                printer.run()
            })
        };

        // The loop parks on the exit condition; the signal proves it is running
        // on another thread, and the wakeup below is what proves it was parked
        // there.
        started_rx.recv().expect("the run loop started");
        printer.request_exit("exit");

        assert_eq!(runner.join().expect("the run loop returned"), "exit");
    }

    #[test]
    fn test_a_printer_asked_to_exit_before_it_ran_does_not_wait() {
        let printer = new_printer();
        printer.request_exit("error_exit");

        assert_eq!(printer.run(), "error_exit");
    }

    #[tokio::test]
    async fn test_reset_for_restart_keeps_the_hosts_parts_and_drops_the_configs() {
        let printer = new_printer();
        // The host registers `webhooks` before the config; the config's parts
        // come after `mark_host_objects`.
        printer
            .add_object("webhooks", Arc::new(Fixed(serde_json::json!({}))))
            .unwrap();
        printer.mark_host_objects();
        printer
            .add_object("gcode", Arc::new(Fixed(serde_json::json!({}))))
            .unwrap();
        printer.bring_up().await;
        assert_eq!(printer.get_state_message().category, PrinterState::Ready);

        printer.reset_for_restart("firmware_restart");

        // The host's part stays, the config's is gone, and the state is back to
        // `Startup` so the machine can be brought up again.
        assert_eq!(printer.objects(), ["webhooks"]);
        let state = printer.get_state_message();
        assert_eq!(state.category, PrinterState::Startup);
        assert_eq!(state.message, MESSAGE_STARTUP);
    }

    #[tokio::test]
    async fn test_teardown_drops_the_configs_parts_and_keeps_the_state() {
        let printer = new_printer();
        printer
            .add_object("webhooks", Arc::new(Fixed(serde_json::json!({}))))
            .unwrap();
        printer.mark_host_objects();
        printer
            .add_object("gcode", Arc::new(Fixed(serde_json::json!({}))))
            .unwrap();
        printer.bring_up().await;

        printer.teardown();

        // The config's parts are gone — dropping them is what closes their
        // devices — while the host's `webhooks` stays. Unlike a restart, the
        // state is left alone: this is the host's last teardown, not a rebuild.
        assert_eq!(printer.objects(), ["webhooks"]);
        assert_eq!(printer.get_state_message().category, PrinterState::Ready);
    }

    #[test]
    fn test_overrides_are_remembered_per_section() {
        let printer = new_printer();
        assert!(printer.overrides_for("mcu").is_empty());

        printer.override_config(
            "mcu",
            "restart_method",
            ConfigValue::Single("command".to_string()),
        );
        printer.override_config(
            "mcu zboard",
            "restart_method",
            ConfigValue::Single("arduino".to_string()),
        );

        assert_eq!(
            printer.overrides_for("mcu"),
            vec![(
                "restart_method".to_string(),
                ConfigValue::Single("command".to_string())
            )]
        );
        // Another section is untouched, and re-recording replaces.
        assert_eq!(
            printer.overrides_for("mcu zboard"),
            vec![(
                "restart_method".to_string(),
                ConfigValue::Single("arduino".to_string())
            )]
        );
        printer.override_config(
            "mcu",
            "restart_method",
            ConfigValue::Single("cheetah".to_string()),
        );
        assert_eq!(
            printer.overrides_for("mcu"),
            vec![(
                "restart_method".to_string(),
                ConfigValue::Single("cheetah".to_string())
            )]
        );
    }

    #[test]
    fn test_reset_for_restart_forgets_handlers_and_the_exit_request() {
        let printer = new_printer();
        let fired = Arc::new(AtomicUsize::new(0));
        let counter = Arc::clone(&fired);
        printer.register_event_handler(
            KlippyEvent::KlippyReady,
            Box::new(move |_| {
                counter.fetch_add(1, Ordering::SeqCst);
            }),
        );
        printer.request_exit("exit");
        assert_eq!(printer.start_reason(), None);

        printer.reset_for_restart("firmware_restart");

        // The handler was registered by a part that is gone, the old exit must
        // not decide the next run, and the next bring-up knows what it is for.
        assert_eq!(printer.start_reason().as_deref(), Some("firmware_restart"));
        printer.send_event(&KlippyEvent::KlippyReady);
        printer.request_exit("firmware_restart");
        assert_eq!(fired.load(Ordering::SeqCst), 0);
        assert_eq!(printer.run(), "firmware_restart");
    }

    #[test]
    fn test_the_first_exit_result_stands() {
        let printer = new_printer();
        printer.request_exit("exit");
        printer.request_exit("firmware_restart");

        assert_eq!(printer.run(), "exit");
    }

    #[test]
    fn test_invoke_shutdown_halts_the_printer_and_fires_the_event() {
        let printer = new_printer();
        let halts = Arc::new(AtomicUsize::new(0));
        let counter = Arc::clone(&halts);
        printer.register_event_handler(
            KlippyEvent::KlippyShutdown,
            Box::new(move |_| {
                counter.fetch_add(1, Ordering::SeqCst);
            }),
        );

        printer.invoke_shutdown("Printer is halted");

        assert_eq!(halts.load(Ordering::SeqCst), 1);
        let state = printer.get_state_message();
        assert_eq!(state.message, "Printer is halted");
        assert_eq!(state.category, PrinterState::Shutdown);
    }

    #[test]
    fn test_set_error_state_reports_error_not_shutdown() {
        // A bad config is an `error` the operator can fix with `RESTART`, not a
        // shutdown (upstream `_set_state`, `klippy/klippy.py:57-62`).
        let printer = new_printer();
        let halts = Arc::new(AtomicUsize::new(0));
        let counter = Arc::clone(&halts);
        printer.register_event_handler(
            KlippyEvent::KlippyShutdown,
            Box::new(move |_| {
                counter.fetch_add(1, Ordering::SeqCst);
            }),
        );

        printer.set_error_state("Option 'pinn' is not valid in section 'output_pin fan'");

        let state = printer.get_state_message();
        assert_eq!(state.category, PrinterState::Error);
        assert!(state.message.contains("pinn"), "{}", state.message);
        assert_eq!(halts.load(Ordering::SeqCst), 0, "no shutdown event");
    }

    #[test]
    fn test_set_error_state_does_not_overwrite_a_shutdown() {
        let printer = new_printer();
        printer.invoke_shutdown("the MCU died");

        printer.set_error_state("a config problem");

        let state = printer.get_state_message();
        assert_eq!(state.category, PrinterState::Shutdown);
        assert_eq!(state.message, "the MCU died");
    }

    #[test]
    fn test_only_the_first_shutdown_message_is_reported() {
        let printer = new_printer();
        printer.invoke_shutdown("Printer is halted");
        printer.invoke_shutdown("something else went wrong");

        assert_eq!(printer.get_state_message().message, "Printer is halted");
    }

    #[test]
    fn test_analyze_shutdown_carries_the_message() {
        let printer = new_printer();
        let seen = Arc::new(Mutex::new(Vec::new()));
        let captured = Arc::clone(&seen);
        printer.register_event_handler(
            KlippyEvent::KlippyAnalyzeShutdown {
                msg: String::new(),
                details: HashMap::new(),
            },
            Box::new(move |event| {
                if let KlippyEvent::KlippyAnalyzeShutdown { msg, .. } = event {
                    captured.lock().unwrap().push(msg.clone());
                }
            }),
        );

        printer.invoke_shutdown("Printer is halted");

        assert_eq!(*seen.lock().unwrap(), ["Printer is halted"]);
    }

    #[test]
    fn test_a_panicking_handler_does_not_stop_the_others() {
        let printer = new_printer();
        let fired = Arc::new(AtomicUsize::new(0));
        let counter = Arc::clone(&fired);
        printer.register_event_handler(
            KlippyEvent::KlippyReady,
            Box::new(|_| panic!("handler failed")),
        );
        printer.register_event_handler(
            KlippyEvent::KlippyReady,
            Box::new(move |_| {
                counter.fetch_add(1, Ordering::SeqCst);
            }),
        );

        printer.send_event(&KlippyEvent::KlippyReady);

        assert_eq!(fired.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn test_an_unknown_event_is_dispatched_by_name() {
        // Upstream may send a name this build has no variant for. It reaches a
        // handler registered under that name, and firing one with no handler
        // must not panic.
        let printer = new_printer();
        let fired = Arc::new(AtomicUsize::new(0));
        let counter = Arc::clone(&fired);
        printer.register_event_handler(
            KlippyEvent::Unknown {
                name: "future:event".to_string(),
                params: HashMap::new(),
            },
            Box::new(move |_| {
                counter.fetch_add(1, Ordering::SeqCst);
            }),
        );

        printer.send_event(&KlippyEvent::Unknown {
            name: "future:event".to_string(),
            params: HashMap::new(),
        });
        printer.send_event(&KlippyEvent::Unknown {
            name: "never:seen".to_string(),
            params: HashMap::new(),
        });

        assert_eq!(fired.load(Ordering::SeqCst), 1);
    }

    #[tokio::test]
    async fn test_a_printer_that_shut_down_before_it_ran_never_becomes_ready() {
        let printer = new_printer();
        printer.invoke_shutdown("Printer is halted");
        printer.request_exit("exit");

        printer.bring_up().await;
        assert_eq!(printer.run(), "exit");

        let state = printer.get_state_message();
        assert_eq!(state.message, "Printer is halted");
        assert_eq!(state.category, PrinterState::Shutdown);
    }

    /// A part that records that it was connected, and can be made to fail.
    struct Part {
        name: &'static str,
        log: Arc<Mutex<Vec<&'static str>>>,
        fails: bool,
    }

    impl PrinterObject for Part {
        fn get_status(&self, _eventtime: f64) -> Value {
            serde_json::json!({})
        }

        fn connect<'a>(&'a self) -> ConnectFuture<'a> {
            let name = self.name;
            let log = Arc::clone(&self.log);
            let fails = self.fails;
            Box::pin(async move {
                log.lock().unwrap_or_else(|p| p.into_inner()).push(name);
                if fails {
                    return Err(KlippyError::Internal(format!("{name} is broken")));
                }
                Ok(())
            })
        }
    }

    #[tokio::test]
    async fn test_bring_up_connects_the_objects_in_registration_order() {
        let printer = new_printer();
        let log = Arc::new(Mutex::new(Vec::new()));
        for name in ["first", "second"] {
            printer
                .add_object(
                    name,
                    Arc::new(Part {
                        name,
                        log: Arc::clone(&log),
                        fails: false,
                    }),
                )
                .unwrap();
        }

        printer.bring_up().await;

        assert_eq!(*log.lock().unwrap(), ["first", "second"]);
        assert_eq!(printer.get_state_message().category, PrinterState::Ready);
    }

    #[test]
    fn test_require_object_reports_upstream_wording() {
        let printer = new_printer();

        let err = printer
            .require_object("nope")
            .map(|_| ())
            .unwrap_err()
            .to_string();

        assert_eq!(err, "Unknown config object 'nope'");
    }

    #[tokio::test]
    async fn test_an_object_that_fails_to_connect_puts_the_printer_in_error() {
        let printer = new_printer();
        printer
            .add_object(
                "broken",
                Arc::new(Part {
                    name: "broken",
                    log: Arc::new(Mutex::new(Vec::new())),
                    fails: true,
                }),
            )
            .unwrap();

        printer.bring_up().await;

        // A connect failure is the `error` state, which `RESTART` can fix; it
        // does not halt the host (upstream `_connect` -> `_set_state`).
        let state = printer.get_state_message();
        assert_eq!(state.category, PrinterState::Error);
        assert!(state.message.contains("broken"), "{}", state.message);
    }

    /// A part whose connect fails with a config error.
    struct ConfigPart;

    impl PrinterObject for ConfigPart {
        fn get_status(&self, _eventtime: f64) -> Value {
            serde_json::json!({})
        }

        fn connect<'a>(&'a self) -> ConnectFuture<'a> {
            Box::pin(async { Err(KlippyError::Config(ConfigError::new("bad option"))) })
        }
    }

    #[tokio::test]
    async fn test_a_config_error_during_connect_is_an_error_state_not_a_shutdown() {
        // A config problem found while connecting can be fixed with `RESTART`,
        // so it sets the `error` state and fires no shutdown event (upstream
        // `_connect`, `klippy/klippy.py:136-139`).
        let printer = new_printer();
        let halts = Arc::new(AtomicUsize::new(0));
        let counter = Arc::clone(&halts);
        printer.register_event_handler(
            KlippyEvent::KlippyShutdown,
            Box::new(move |_| {
                counter.fetch_add(1, Ordering::SeqCst);
            }),
        );
        printer.add_object("part", Arc::new(ConfigPart)).unwrap();

        printer.bring_up().await;

        let state = printer.get_state_message();
        assert_eq!(state.category, PrinterState::Error);
        assert!(state.message.contains("bad option"), "{}", state.message);
        assert_eq!(halts.load(Ordering::SeqCst), 0);
    }

    /// A part named `mcu` whose connect fails with a protocol error.
    struct BrokenMcu;

    impl PrinterObject for BrokenMcu {
        fn get_status(&self, _eventtime: f64) -> Value {
            serde_json::json!({})
        }

        fn connect<'a>(&'a self) -> ConnectFuture<'a> {
            Box::pin(async {
                Err(KlippyError::Connection(
                    "Protocol error: bad message".to_string(),
                ))
            })
        }
    }

    #[tokio::test]
    async fn test_an_mcu_protocol_failure_is_expanded_by_error_mcu() {
        // `bring_up` sets the short state message the `klippy:notify_mcu_error`
        // event carries; the `error_mcu` module turns it into the text the user
        // reads (upstream `_connect` -> `_set_state` -> `error_mcu`).
        let printer = Arc::new(new_printer());
        crate::core::klippy::extras::error_mcu::ensure(&printer).unwrap();
        printer.add_object("mcu", Arc::new(BrokenMcu)).unwrap();

        printer.bring_up().await;

        let state = printer.get_state_message();
        assert_eq!(state.category, PrinterState::Error);
        assert!(
            state.message.starts_with("MCU Protocol error"),
            "{}",
            state.message
        );
        assert!(
            state.message.contains("Protocol error: bad message"),
            "{}",
            state.message
        );
        assert!(
            state.message.contains("Your Klipper version is:"),
            "{}",
            state.message
        );
    }

    #[tokio::test]
    async fn test_a_printer_that_shut_down_does_not_connect_its_objects() {
        let printer = new_printer();
        let log = Arc::new(Mutex::new(Vec::new()));
        printer
            .add_object(
                "part",
                Arc::new(Part {
                    name: "part",
                    log: Arc::clone(&log),
                    fails: false,
                }),
            )
            .unwrap();
        printer.invoke_shutdown("Printer is halted");

        printer.bring_up().await;

        assert!(
            log.lock().unwrap().is_empty(),
            "an object was connected for a machine that is stopped"
        );
        assert_eq!(printer.get_state_message().category, PrinterState::Shutdown);
    }

    /// A part whose status the test wrote.
    struct Fixed(Value);

    impl PrinterObject for Fixed {
        fn get_status(&self, _eventtime: f64) -> Value {
            self.0.clone()
        }
    }

    #[test]
    fn test_a_new_printer_has_no_objects() {
        // The machine has no parts yet, and the first object a host registers —
        // the API server's `webhooks` — is the host's, not the machine's.
        let printer = new_printer();

        assert_eq!(printer.objects(), Vec::<String>::new());
        assert!(printer.lookup_object("webhooks").is_none());
        assert_eq!(printer.status_of("webhooks", printer.eventtime()), None);
    }

    #[test]
    fn test_objects_come_back_in_registration_order() {
        let printer = new_printer();
        for name in ["webhooks", "extruder", "heater_bed"] {
            printer
                .add_object(name, Arc::new(Fixed(serde_json::json!({}))))
                .unwrap();
        }

        assert_eq!(printer.objects(), ["webhooks", "extruder", "heater_bed"]);
    }

    #[test]
    fn test_a_registered_object_can_be_looked_up_by_name() {
        let printer = new_printer();
        printer
            .add_object(
                "webhooks",
                Arc::new(Fixed(serde_json::json!({ "who": "webhooks" }))),
            )
            .unwrap();

        let object = printer.lookup_object("webhooks").expect("registered");

        assert_eq!(
            object.get_status(0.0),
            serde_json::json!({"who": "webhooks"})
        );
        assert!(printer.lookup_object("nope").is_none());
    }

    #[test]
    fn test_queryable_objects_leave_out_parts_that_report_no_status() {
        // `objects/list` shows only parts a client can query; `pins` is
        // registered but has no status, so the two sets differ.
        struct Hidden;
        impl PrinterObject for Hidden {
            fn get_status(&self, _eventtime: f64) -> Value {
                serde_json::json!({})
            }
            fn is_queryable(&self) -> bool {
                false
            }
        }
        let printer = new_printer();
        printer.add_object("pins", Arc::new(Hidden)).unwrap();
        printer
            .add_object("toolhead", Arc::new(Fixed(serde_json::json!({}))))
            .unwrap();

        assert_eq!(printer.objects(), ["pins", "toolhead"]);
        assert_eq!(printer.queryable_objects(), ["toolhead"]);
    }

    #[test]
    fn test_lookup_objects_selects_a_module_and_its_prefix() {
        // Upstream's `lookup_objects('mcu')`: the exact name first, then the
        // `mcu <name>` sections in registration order.
        let printer = new_printer();
        for name in ["mcu", "output_pin fan", "mcu zboard", "mcu toolhead"] {
            printer
                .add_object(name, Arc::new(Fixed(serde_json::json!({}))))
                .unwrap();
        }

        let names = |module: Option<&str>| -> Vec<String> {
            printer
                .lookup_objects(module)
                .into_iter()
                .map(|(name, _)| name)
                .collect()
        };

        assert_eq!(names(Some("mcu")), ["mcu", "mcu zboard", "mcu toolhead"]);
        // A module with no exact object still finds its prefixed ones.
        assert_eq!(names(Some("output_pin")), ["output_pin fan"]);
        // Nothing matches: an empty list, not an error.
        assert!(names(Some("nope")).is_empty());
        // `None` is everything, in registration order.
        assert_eq!(
            names(None),
            ["mcu", "output_pin fan", "mcu zboard", "mcu toolhead"]
        );
    }

    #[test]
    fn test_statuses_reads_every_queryable_object_without_the_lock() {
        // The reflection read a template's `printer.objects` does: one snapshot,
        // every queryable object's status, and a status that itself asks the
        // printer something must not deadlock.
        struct ReadsThePrinter(Weak<Printer>);
        impl PrinterObject for ReadsThePrinter {
            fn get_status(&self, _eventtime: f64) -> Value {
                let ready = self
                    .0
                    .upgrade()
                    .map(|printer| printer.objects().len())
                    .unwrap_or(0);
                serde_json::json!({ "objects": ready })
            }
        }

        let printer = Arc::new(new_printer());
        printer
            .add_object(
                "chatty",
                Arc::new(ReadsThePrinter(Arc::downgrade(&printer))),
            )
            .unwrap();
        printer
            .add_object("quiet", Arc::new(Fixed(serde_json::json!({"a": 1}))))
            .unwrap();

        let statuses = printer.statuses(0.0);

        assert_eq!(statuses["chatty"], serde_json::json!({"objects": 2}));
        assert_eq!(statuses["quiet"], serde_json::json!({"a": 1}));
    }

    #[test]
    fn test_lookup_object_as_returns_the_concrete_type() {
        let printer = new_printer();
        printer
            .add_object("fixed", Arc::new(Fixed(serde_json::json!({"a": 1}))))
            .unwrap();

        let fixed = printer
            .lookup_object_as::<Fixed>("fixed")
            .expect("same type");
        assert_eq!(fixed.0, serde_json::json!({"a": 1}));
        // A different type under the same name, and an unregistered name, are
        // both `None`.
        assert!(printer.lookup_object_as::<Part>("fixed").is_none());
        assert!(printer.lookup_object_as::<Fixed>("nope").is_none());
    }

    #[test]
    fn test_a_duplicate_object_is_rejected() {
        let printer = new_printer();
        printer
            .add_object(
                "webhooks",
                Arc::new(Fixed(serde_json::json!({"by": "first"}))),
            )
            .unwrap();

        let err = printer
            .add_object(
                "webhooks",
                Arc::new(Fixed(serde_json::json!({"by": "second"}))),
            )
            .unwrap_err();

        assert!(err.to_string().contains("webhooks"), "{err}");
        // The first registration stands.
        assert_eq!(
            printer.status_of("webhooks", 0.0),
            Some(serde_json::json!({"by": "first"}))
        );
    }

    #[test]
    fn test_an_unregistered_object_has_no_status() {
        let printer = new_printer();

        assert_eq!(printer.status_of("nope", printer.eventtime()), None);
    }

    #[test]
    fn test_status_queries_are_handed_the_object_that_was_asked_for() {
        let printer = new_printer();
        printer
            .add_object("echo", Arc::new(Fixed(serde_json::json!({"who": "echo"}))))
            .unwrap();
        printer
            .add_object(
                "other",
                Arc::new(Fixed(serde_json::json!({"who": "other"}))),
            )
            .unwrap();

        assert_eq!(
            printer.status_of("other", 0.0),
            Some(serde_json::json!({"who": "other"}))
        );
    }

    #[test]
    fn test_eventtime_is_the_reactors_clock() {
        // The machine keeps no clock of its own: `eventtime` is whatever the
        // reactor it was handed reads, which is what makes a fake reactor a
        // complete fake of time.
        let manual = Arc::new(ManualReactor::new());
        let printer = Printer::new(manual.clone());

        let first = printer.eventtime();
        assert_eq!(first, 0.0);

        manual.advance(1.5);
        let second = printer.eventtime();
        assert_eq!(second, 1.5);
        assert!(second > first, "{second} <= {first}");
    }

    #[test]
    fn test_the_printer_hands_back_the_reactor_it_was_built_with() {
        let manual = Arc::new(ManualReactor::new());
        let printer = Printer::new(manual.clone());

        assert_eq!(printer.reactor().monotonic(), printer.eventtime());

        manual.advance(2.0);

        assert_eq!(printer.reactor().monotonic(), 2.0);
    }
}
