// Dummy "none" printer (for developer testing)
//
// This module implements the host of a machine whose config says
// `kinematics: none` — a machine with no steppers, no MCUs and no toolhead. It
// is the printer counterpart of `NoneKinematics`, and serves the same purpose:
// running host-side logic without a real printer attached.
//
// What is left of a printer when there is no hardware is its lifecycle. This
// one comes up ready, firing `klippy:connect` and then `klippy:ready`, and then
// idles until it is asked to exit. It has nothing to move and nothing to home,
// and it cannot fail to connect, so it never reports the `error` category — a
// printer that loads a config and talks to MCUs is where startup can go wrong.

use std::collections::HashMap;
use std::sync::{Arc, Condvar, Mutex, MutexGuard};

use tracing::error;

use super::printer::{EventHandler, Printer, PrinterEvent, PrinterState, StateMessage};

/// The message reported from construction until the printer is ready.
const MESSAGE_STARTUP: &str = "Starting up";
/// The message reported once the printer is ready.
const MESSAGE_READY: &str = "Printer is ready";

/// A printer with no hardware: a lifecycle and nothing else.
pub struct NonePrinter {
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

impl NonePrinter {
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

    /// Fire an event at the handlers registered for it.
    ///
    /// The handlers are collected under the lock and run without it: a handler
    /// may itself ask the printer to exit, and a lock held across callbacks
    /// would deadlock there.
    fn fire(&self, event: &PrinterEvent) {
        let handlers = match self.lock().handlers.get(event) {
            Some(handlers) => handlers.clone(),
            None => return,
        };
        for handler in handlers {
            handler();
        }
    }

    /// Come up: connect, then report ready.
    ///
    /// Upstream's `_connect`, without the MCUs there are none of. Both callbacks
    /// can halt the printer, and upstream re-checks its state between them: a
    /// printer that shut down while connecting never becomes ready.
    fn come_up(&self) {
        self.fire(&PrinterEvent::Connect);

        {
            let mut inner = self.lock();
            if inner.category != PrinterState::Startup {
                return;
            }
            inner.message = MESSAGE_READY.to_string();
            inner.category = PrinterState::Ready;
        }

        self.fire(&PrinterEvent::Ready);
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

impl Default for NonePrinter {
    fn default() -> Self {
        Self::new()
    }
}

impl Printer for NonePrinter {
    fn get_state_message(&self) -> StateMessage {
        let inner = self.lock();
        StateMessage {
            message: inner.message.clone(),
            category: inner.category.clone(),
        }
    }

    fn register_event_handler(&self, event: PrinterEvent, callback: EventHandler) {
        self.lock()
            .handlers
            .entry(event)
            .or_default()
            .push(Arc::from(callback));
    }

    fn send_event(&self, event: &PrinterEvent) {
        self.fire(event);
    }

    fn invoke_shutdown(&self, msg: &str) {
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
        self.fire(&PrinterEvent::Shutdown);
    }

    fn request_exit(&self, result: &str) {
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

    fn run(&self) -> String {
        self.come_up();

        let result = self.wait_for_exit();

        // What upstream does when the reactor loop ends: a run that was asked
        // for a firmware restart says so before everyone is told the printer is
        // gone.
        if result == "firmware_restart" {
            self.fire(&PrinterEvent::FirmwareRestart);
        }
        self.fire(&PrinterEvent::Disconnect);

        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::mpsc;
    use std::thread;

    /// Register a handler that records the event name it was called for.
    fn record(printer: &NonePrinter, event: PrinterEvent, log: &Arc<Mutex<Vec<&'static str>>>) {
        let log = Arc::clone(log);
        let name = event.as_str();
        printer.register_event_handler(
            event,
            Box::new(move || log.lock().unwrap_or_else(|p| p.into_inner()).push(name)),
        );
    }

    #[test]
    fn test_a_new_printer_is_starting_up() {
        let printer = NonePrinter::new();

        let state = printer.get_state_message();
        assert_eq!(state.message, MESSAGE_STARTUP);
        assert_eq!(state.category, PrinterState::Startup);
    }

    #[test]
    fn test_run_comes_up_ready_and_reports_it() {
        let printer = NonePrinter::new();
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
        let printer = NonePrinter::new();
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
        let printer = NonePrinter::new();
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
        let printer = Arc::new(NonePrinter::new());
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
        let printer = NonePrinter::new();
        printer.request_exit("error_exit");

        assert_eq!(printer.run(), "error_exit");
    }

    #[test]
    fn test_the_first_exit_result_stands() {
        let printer = NonePrinter::new();
        printer.request_exit("exit");
        printer.request_exit("firmware_restart");

        assert_eq!(printer.run(), "exit");
    }

    #[test]
    fn test_invoke_shutdown_halts_the_printer_and_fires_the_event() {
        let printer = NonePrinter::new();
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
        let printer = NonePrinter::new();
        printer.invoke_shutdown("Printer is halted");
        printer.invoke_shutdown("something else went wrong");

        assert_eq!(printer.get_state_message().message, "Printer is halted");
    }

    #[test]
    fn test_a_printer_that_shut_down_before_it_ran_never_becomes_ready() {
        let printer = NonePrinter::new();
        printer.invoke_shutdown("Printer is halted");
        printer.request_exit("exit");

        assert_eq!(printer.run(), "exit");

        let state = printer.get_state_message();
        assert_eq!(state.message, "Printer is halted");
        assert_eq!(state.category, PrinterState::Shutdown);
    }
}
