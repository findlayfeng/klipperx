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
// - `PrinterEvent`: the lifecycle events a printer fires at its handlers
// - `Printer`: the machine

use std::any::Any;
use std::collections::HashMap;
use std::future::Future;
use std::pin::Pin;
use std::sync::{Arc, Condvar, Mutex, MutexGuard};

use serde_json::Value;
use tracing::error;

use crate::core::klippy::error::KlippyError;
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
// PrinterEvent
// ===========================================================================

/// An event a printer fires at its handlers.
///
/// Each variant maps to a wire event name (see [`PrinterEvent::as_str`]).
///
/// Upstream's event set is larger. Three events are absent here:
///
/// * `klippy:mcu_identify` — fired between loading the config and connecting
///   the MCUs, which is a step a printer with no MCUs does not have;
/// * `klippy:analyze_shutdown` and `klippy:notify_mcu_error` — upstream calls
///   their handlers with the message and details being reported, which a
///   zero-argument handler cannot receive. They come back with the error
///   reporting that needs them (a payload-carrying event type).
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum PrinterEvent {
    /// Every MCU is connected
    Connect,
    /// The printer is ready to accept commands
    Ready,
    /// The printer has halted
    Shutdown,
    /// The run loop has ended
    Disconnect,
    /// The run loop ended because a firmware restart was asked for
    FirmwareRestart,
}

impl PrinterEvent {
    /// Get the event name string (e.g. "klippy:ready")
    pub fn as_str(&self) -> &'static str {
        match self {
            PrinterEvent::Connect => "klippy:connect",
            PrinterEvent::Ready => "klippy:ready",
            PrinterEvent::Shutdown => "klippy:shutdown",
            PrinterEvent::Disconnect => "klippy:disconnect",
            PrinterEvent::FirmwareRestart => "klippy:firmware_restart",
        }
    }
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
}

struct Inner {
    /// The message the printer reports, for the user.
    message: String,
    /// What the printer is doing.
    category: PrinterState,
    /// Whether `invoke_shutdown` has run; the first message stands.
    shutdown: bool,
    /// What `run` returns once the loop ends, set by `request_exit`.
    run_result: Option<String>,
    /// Handlers per event, in registration order.
    handlers: HashMap<PrinterEvent, Vec<Arc<dyn Fn() + Send + Sync>>>,
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
            }),
            exit_requested: Condvar::new(),
            reactor,
            objects: Mutex::new(Vec::new()),
        }
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
    /// Returns [`KlippyError::Internal`] if `name` is already taken: two parts
    /// answering to one name is a wiring mistake in klippy that no client can
    /// provoke. (It deserves an error type of its own; the vocabulary is still
    /// missing — see the `TODO`.)
    pub fn add_object(
        &self,
        name: &str,
        object: Arc<dyn PrinterObject>,
    ) -> Result<(), KlippyError> {
        let mut objects = self.objects.lock().unwrap_or_else(|p| p.into_inner());
        if objects.iter().any(|(taken, _)| taken == name) {
            return Err(KlippyError::Internal(format!(
                "printer object '{name}' already registered"
            )));
        }
        objects.push((name.to_string(), object));
        Ok(())
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

    /// Register a callback for a specific event.
    ///
    /// Handlers run in registration order, on the thread that fires the event,
    /// and take no arguments. Like the printer's own callbacks upstream, they
    /// must not block. Boxed because a handler list holds many of them.
    pub fn register_event_handler(
        &self,
        event: PrinterEvent,
        callback: Box<dyn Fn() + Send + Sync>,
    ) {
        self.lock()
            .handlers
            .entry(event)
            .or_default()
            .push(Arc::from(callback));
    }

    /// Fire an event at the handlers registered for it.
    ///
    /// The handlers are collected under the lock and run without it: a handler
    /// may itself ask the printer to exit, and a lock held across callbacks
    /// would deadlock there.
    pub fn send_event(&self, event: &PrinterEvent) {
        let handlers = match self.lock().handlers.get(event) {
            Some(handlers) => handlers.clone(),
            None => return,
        };
        for handler in handlers {
            handler();
        }
    }

    /// Halt the printer with a message for the user.
    ///
    /// The printer moves to the `shutdown` category and fires
    /// `klippy:shutdown`. Halting does not end the run loop: the printer stays
    /// up so that clients can still read why it stopped, until something asks
    /// it to exit. The first message stands; later ones are ignored, as
    /// upstream does.
    pub fn invoke_shutdown(&self, msg: &str) {
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
        self.send_event(&PrinterEvent::Shutdown);
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
            self.send_event(&PrinterEvent::FirmwareRestart);
        }
        self.send_event(&PrinterEvent::Disconnect);

        result
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
    /// only fired once every object is up.
    pub async fn bring_up(&self) {
        // A printer that was already halted — a config the loader rejected, a
        // shutdown that raced this call — has nothing to bring up. Connecting
        // its parts anyway would reach a device for a machine that is stopped.
        if self.category() != PrinterState::Startup {
            return;
        }

        for (name, object) in self.registry() {
            if let Err(err) = object.connect().await {
                self.invoke_shutdown(&format!("{name}: {err}"));
                return;
            }
            if self.category() != PrinterState::Startup {
                return;
            }
        }

        self.send_event(&PrinterEvent::Connect);

        {
            let mut inner = self.lock();
            if inner.category != PrinterState::Startup {
                return;
            }
            inner.message = MESSAGE_READY.to_string();
            inner.category = PrinterState::Ready;
        }

        self.send_event(&PrinterEvent::Ready);
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
    use std::thread;

    /// A printer on a clock the test controls.
    ///
    /// No runtime: the machine's tests exercise the lifecycle, not timers, and
    /// a reactor the test can step is deterministic when one is added.
    fn new_printer() -> Printer {
        Printer::new(Arc::new(ManualReactor::new()))
    }

    /// Register a handler that records the event name it was called for.
    fn record(printer: &Printer, event: PrinterEvent, log: &Arc<Mutex<Vec<&'static str>>>) {
        let log = Arc::clone(log);
        let name = event.as_str();
        printer.register_event_handler(
            event,
            Box::new(move || log.lock().unwrap_or_else(|p| p.into_inner()).push(name)),
        );
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
        assert_eq!(PrinterEvent::Connect.as_str(), "klippy:connect");
        assert_eq!(PrinterEvent::Ready.as_str(), "klippy:ready");
        assert_eq!(PrinterEvent::Shutdown.as_str(), "klippy:shutdown");
        assert_eq!(PrinterEvent::Disconnect.as_str(), "klippy:disconnect");
        assert_eq!(
            PrinterEvent::FirmwareRestart.as_str(),
            "klippy:firmware_restart"
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
            PrinterEvent::Ready,
            Box::new(move || {
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
        for event in [
            PrinterEvent::Connect,
            PrinterEvent::Ready,
            PrinterEvent::Shutdown,
            PrinterEvent::FirmwareRestart,
            PrinterEvent::Disconnect,
        ] {
            record(&printer, event, &log);
        }
        printer.request_exit("firmware_restart");

        printer.bring_up().await;
        assert_eq!(printer.run(), "firmware_restart");
        assert_eq!(
            *log.lock().unwrap(),
            [
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
                PrinterEvent::Ready,
                Box::new(move || order.lock().unwrap().push(name)),
            );
        }

        printer.send_event(&PrinterEvent::Ready);

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
            PrinterEvent::Shutdown,
            Box::new(move || {
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
    fn test_only_the_first_shutdown_message_is_reported() {
        let printer = new_printer();
        printer.invoke_shutdown("Printer is halted");
        printer.invoke_shutdown("something else went wrong");

        assert_eq!(printer.get_state_message().message, "Printer is halted");
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

    #[tokio::test]
    async fn test_an_object_that_fails_to_connect_halts_the_printer() {
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

        let state = printer.get_state_message();
        assert_eq!(state.category, PrinterState::Shutdown);
        assert!(state.message.contains("broken"), "{}", state.message);
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
