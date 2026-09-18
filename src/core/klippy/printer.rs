// Printer — the machine a host runs.
//
// A printer is one machine: one host process runs exactly one, the machine its
// config describes, and the machine owns what it is made of — the printer
// objects of the config's sections — along with its own lifecycle: coming up,
// saying what state it is in, halting, and idling until something ends it.
// Starting a process, logging, serving the API, and building the next printer
// after a restart are the host's, not the machine's.
//
// Today the machine has no parts: there are no MCUs, no toolhead, no kinematics
// and no printer objects, so what is here is a lifecycle and a state machine
// with nothing to drive. The parts arrive with their layers, and this type
// grows with them.
//
// This module defines:
// - `PrinterState`: printer state categories
// - `StateMessage`: state message returned by `Printer::get_state_message`
// - `PrinterEvent`: the lifecycle events a printer fires at its handlers
// - `Printer`: the machine

use std::collections::HashMap;
use std::sync::{Arc, Condvar, Mutex, MutexGuard};

use tracing::error;

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

/// The machine a host runs: its lifecycle, and the state it reports.
///
/// Shared rather than borrowed (`&self` throughout): an exit request arrives
/// from another thread — the API server, in the host — while the run loop is
/// idling.
pub struct Printer {
    inner: Mutex<Inner>,
    /// Paired with `inner`, to wake the run loop when an exit is requested.
    exit_requested: Condvar,
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
    /// Create a printer that has not come up yet.
    pub fn new() -> Self {
        Self {
            inner: Mutex::new(Inner {
                message: MESSAGE_STARTUP.to_string(),
                category: PrinterState::Startup,
                shutdown: false,
                run_result: None,
                handlers: HashMap::new(),
            }),
            exit_requested: Condvar::new(),
        }
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

    /// Run the printer: bring it up, then idle until it is asked to exit.
    ///
    /// Startup is upstream's `_connect`: fire `klippy:connect`, and — if that
    /// left the printer starting up — report it ready and fire `klippy:ready`.
    /// The call then blocks until [`Printer::request_exit`] is called, and fires
    /// `klippy:disconnect` — plus `klippy:firmware_restart` when that is what
    /// the exit asked for — before returning the result.
    ///
    /// A printer that was already halted before it ran never comes up, and one
    /// that was already asked to exit does not wait.
    ///
    /// Returns the result the exit was requested with.
    pub fn run(&self) -> String {
        self.come_up();

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

    /// Come up: connect, then report ready.
    ///
    /// Upstream's `_connect`, without the MCUs there are none of. Both callbacks
    /// can halt the printer, and upstream re-checks its state between them: a
    /// printer that shut down while connecting never becomes ready.
    fn come_up(&self) {
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

impl Default for Printer {
    fn default() -> Self {
        Self::new()
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::mpsc;
    use std::thread;

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
        let printer = Printer::new();

        let state = printer.get_state_message();
        assert_eq!(state.message, MESSAGE_STARTUP);
        assert_eq!(state.category, PrinterState::Startup);
    }

    #[test]
    fn test_run_comes_up_ready_and_reports_it() {
        let printer = Printer::new();
        let ready = Arc::new(AtomicUsize::new(0));
        let counter = Arc::clone(&ready);
        printer.register_event_handler(
            PrinterEvent::Ready,
            Box::new(move || {
                counter.fetch_add(1, Ordering::SeqCst);
            }),
        );
        printer.request_exit("exit");

        assert_eq!(printer.run(), "exit");

        assert_eq!(ready.load(Ordering::SeqCst), 1);
        let state = printer.get_state_message();
        assert_eq!(state.message, MESSAGE_READY);
        assert_eq!(state.category, PrinterState::Ready);
    }

    #[test]
    fn test_run_fires_the_lifecycle_events_in_order() {
        let printer = Printer::new();
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
        let printer = Printer::new();
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

    #[test]
    fn test_run_waits_for_an_exit_request_from_another_thread() {
        let printer = Arc::new(Printer::new());
        let (ready_tx, ready_rx) = mpsc::channel();
        printer.register_event_handler(
            PrinterEvent::Ready,
            Box::new(move || {
                ready_tx.send(()).expect("the test is still listening");
            }),
        );

        let runner = {
            let printer = Arc::clone(&printer);
            thread::spawn(move || printer.run())
        };

        // The printer keeps idling until the exit arrives; waiting for `ready`
        // rather than sleeping is what makes the ordering deterministic.
        ready_rx.recv().expect("run fires klippy:ready");
        assert_eq!(printer.get_state_message().category, PrinterState::Ready);
        printer.request_exit("exit");

        assert_eq!(runner.join().expect("the run loop returned"), "exit");
    }

    #[test]
    fn test_a_printer_asked_to_exit_before_it_ran_does_not_wait() {
        let printer = Printer::new();
        printer.request_exit("error_exit");

        assert_eq!(printer.run(), "error_exit");
    }

    #[test]
    fn test_the_first_exit_result_stands() {
        let printer = Printer::new();
        printer.request_exit("exit");
        printer.request_exit("firmware_restart");

        assert_eq!(printer.run(), "exit");
    }

    #[test]
    fn test_invoke_shutdown_halts_the_printer_and_fires_the_event() {
        let printer = Printer::new();
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
        let printer = Printer::new();
        printer.invoke_shutdown("Printer is halted");
        printer.invoke_shutdown("something else went wrong");

        assert_eq!(printer.get_state_message().message, "Printer is halted");
    }

    #[test]
    fn test_a_printer_that_shut_down_before_it_ran_never_becomes_ready() {
        let printer = Printer::new();
        printer.invoke_shutdown("Printer is halted");
        printer.request_exit("exit");

        assert_eq!(printer.run(), "exit");

        let state = printer.get_state_message();
        assert_eq!(state.message, "Printer is halted");
        assert_eq!(state.category, PrinterState::Shutdown);
    }
}
