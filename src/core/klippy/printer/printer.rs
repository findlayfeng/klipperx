// Printer — the interface every printer implementation shares.
//
// This module defines:
// - `PrinterState`: printer state categories
// - `StateMessage`: state message returned by `Printer::get_state_message`
// - `PrinterEvent`: the events a printer fires at its handlers
// - `Printer`: the trait of a printer lifecycle
// - `load_printer`: create the printer that goes with a config's kinematics
//
// A printer owns the host's lifecycle: it comes up, says what state it is in,
// fires the events the rest of the host waits for, and runs until something
// asks it to stop. What it does *not* own is the hardware — MCUs, steppers and
// the object registry are layers of their own, and are reached from the
// printer rather than defined by it.

use crate::core::klippy::error::KlippyError;

use super::none::NonePrinter;

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
// EventHandler
// ===========================================================================

/// A callback registered for a [`PrinterEvent`].
///
/// Boxed rather than generic so that [`Printer`] stays object-safe and a host
/// can hold its printer as a `Box<dyn Printer>`. Infallible: a handler that
/// cannot do its job reports the failure itself (by shutting the printer down),
/// there is no caller to hand a `Result` back to.
pub type EventHandler = Box<dyn Fn() + Send + Sync>;

// ===========================================================================
// Printer trait
// ===========================================================================

/// Abstract interface for a printer's lifecycle.
///
/// It provides:
/// - **State**: what the printer is doing, for `info` and for the run loop
/// - **Events**: callbacks for the points in the lifecycle others wait for
/// - **Shutdown and exit**: how the printer is halted and how the run loop ends
///
/// Which implementation a host runs is decided by the machine's kinematics —
/// see [`load_printer`].
///
/// # Example
/// ```ignore
/// let printer = load_printer("none")?;
/// printer.register_event_handler(PrinterEvent::Ready, Box::new(|| {
///     println!("Printer is ready!");
/// }));
/// let result = printer.run();
/// ```
pub trait Printer: Send + Sync {
    /// Get the current state message and category.
    fn get_state_message(&self) -> StateMessage;

    /// Register a callback for a specific event.
    ///
    /// Handlers run in registration order, on the thread that fires the event.
    /// Like the printer's own callbacks upstream, they must not block.
    fn register_event_handler(&self, event: PrinterEvent, callback: EventHandler);

    /// Fire an event at the handlers registered for it.
    fn send_event(&self, event: &PrinterEvent);

    /// Halt the printer with a message for the user.
    ///
    /// The printer moves to the `shutdown` category and fires
    /// `klippy:shutdown`. Halting does not end the run loop: the printer stays
    /// up so that clients can still read why it stopped, until something asks
    /// it to exit. The first message stands; later ones are ignored, as
    /// upstream does.
    fn invoke_shutdown(&self, msg: &str);

    /// Ask the printer to leave its run loop.
    ///
    /// The `result` decides what happens once the loop ends:
    /// - `"exit"` / `"error_exit"`: the host process terminates
    /// - `"firmware_restart"`: the host starts the printer again
    fn request_exit(&self, result: &str);

    /// Run the printer: bring it up, then idle until it is asked to exit.
    ///
    /// Takes `&self`: an exit request arrives from another thread (the API
    /// server, in the host) while this one is idling, so the printer is shared
    /// rather than borrowed for the duration.
    ///
    /// Returns the result the exit was requested with.
    fn run(&self) -> String;
}

// ===========================================================================
// load_printer
// ===========================================================================

/// Create the printer for a machine whose config declares `kinematics`.
///
/// The kinematics decides which printer a host runs: `none` is a machine with
/// no steppers at all, and its printer is the equally empty [`NonePrinter`].
/// Upstream reads this name from the `[printer]` section and loads the
/// kinematics module by it; a host that wants a printer reads it the same way
/// and passes it here.
///
/// # Errors
/// Returns [`KlippyError::Internal`] for a kinematics this host has no printer
/// for. Falling back to a dummy printer would run a host that reports itself
/// ready for a machine it cannot drive.
pub fn load_printer(kinematics: &str) -> Result<Box<dyn Printer>, KlippyError> {
    match kinematics {
        "none" => Ok(Box::new(NonePrinter::new())),
        other => Err(KlippyError::Internal(format!(
            "no printer for kinematics '{other}': only 'none' is implemented"
        ))),
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

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
    fn test_load_printer_picks_the_printer_the_kinematics_asks_for() {
        let printer = load_printer("none").expect("'none' has a printer");
        assert_eq!(printer.get_state_message().category, PrinterState::Startup);
    }

    #[test]
    fn test_load_printer_refuses_a_kinematics_it_cannot_drive() {
        // `Box<dyn Printer>` is not `Debug`, so match rather than `expect_err`.
        let err = match load_printer("cartesian") {
            Ok(_) => panic!("a cartesian printer is not implemented"),
            Err(err) => err,
        };
        assert!(err.to_string().contains("cartesian"), "{err}");
    }
}
