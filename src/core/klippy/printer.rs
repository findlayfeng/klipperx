// Printer — abstract interface for the printer object registry and lifecycle.
//
// This module defines:
// - `PrinterState`: printer state categories
// - `StateMessage`: state message returned by `Printer::get_state_message`
// - `PrinterEvent`: Klipper event types for the event handler system
// - `Printer`: abstract trait for printer object registry and lifecycle
// - `PrinterObject`: trait for objects registered in the Printer's object registry

use std::collections::HashMap;
use std::sync::Arc;

use super::error::KlippyError;


// ===========================================================================
// PrinterState
// ===========================================================================

/// Printer state categories reported by `Printer::get_state_message`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PrinterState {
    /// During startup, before config is fully loaded
    Startup,
    /// Configuration loaded, all MCUs connected, ready to accept commands
    Ready,
    /// Printer is shutting down (RESTART / FIRMWARE_RESTART)
    Shutdown,
    /// Error state - printer is halted
    Error,
}

impl PrinterState {
    /// Get the state category string ("startup", "ready", "shutdown", "error")
    pub fn as_category(&self) -> &'static str {
        match self {
            PrinterState::Startup => "startup",
            PrinterState::Ready => "ready",
            PrinterState::Shutdown => "shutdown",
            PrinterState::Error => "error",
        }
    }
}

impl Default for PrinterState {
    fn default() -> Self {
        PrinterState::Startup
    }
}

// ===========================================================================
// StateMessage
// ===========================================================================

/// State message pair returned by `Printer::get_state_message`.
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

/// Printer event types used in the event handler system.
///
/// Each variant maps to a wire event name (see [`PrinterEvent::as_str`]):
///   klippy:mcu_identify, klippy:connect, klippy:ready,
///   klippy:shutdown, klippy:analyze_shutdown, klippy:disconnect,
///   klippy:firmware_restart, klippy:notify_mcu_error
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum PrinterEvent {
    McuIdentify,
    Connect,
    Ready,
    Shutdown,
    AnalyzeShutdown,
    Disconnect,
    FirmwareRestart,
    NotifyMcuError(String),
}

impl PrinterEvent {
    /// Get the event name string (e.g. "klippy:ready")
    pub fn as_str(&self) -> &str {
        match self {
            PrinterEvent::McuIdentify => "klippy:mcu_identify",
            PrinterEvent::Connect => "klippy:connect",
            PrinterEvent::Ready => "klippy:ready",
            PrinterEvent::Shutdown => "klippy:shutdown",
            PrinterEvent::AnalyzeShutdown => "klippy:analyze_shutdown",
            PrinterEvent::Disconnect => "klippy:disconnect",
            PrinterEvent::FirmwareRestart => "klippy:firmware_restart",
            PrinterEvent::NotifyMcuError(_) => "klippy:notify_mcu_error",
        }
    }
}

// ===========================================================================
// Printer trait
// ===========================================================================

/// Abstract interface for the printer object registry and lifecycle.
///
/// It provides:
/// - **Object registry**: add/lookup printer objects (steppers, extruders, etc.)
/// - **State management**: track printer state (startup/ready/shutdown/error)
/// - **Event system**: register callbacks for printer lifecycle events
/// - **Lifecycle control**: connect, run, shutdown
///
/// Implementations may be:
/// - A full internal Printer for running Klipper logic in-process
/// - A mock/stub for testing application logic without a real printer
///
/// # Example
/// ```ignore
/// let mut printer = MyPrinterImpl::new(&start_args);
/// printer.register_event_handler(PrinterEvent::Ready, Box::new(|| {
///     println!("Printer is ready!");
/// }));
/// let result = printer.run();
/// ```
pub trait Printer: Send + Sync {
    /// Get the current state message and category.
    fn get_state_message(&self) -> StateMessage;

    /// Check if the printer is in the shutdown state.
    fn is_shutdown(&self) -> bool;

    /// Add an object to the printer's object registry.
    ///
    /// Objects are identified by a string name. Adding an object
    /// with a duplicate name will return an error.
    fn add_object(&self, name: &str, obj: Arc<dyn PrinterObject>) -> Result<(), KlippyError>;

    /// Look up an object by name from the registry.
    fn lookup_object<T: PrinterObject + 'static>(&self, name: &str) -> Option<Arc<T>>;

    /// Look up objects whose names start with a given prefix.
    ///
    /// Returns a list of (name, object) pairs.
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, Arc<dyn PrinterObject>)>;

    /// Register an event handler callback for a specific event.
    ///
    /// Events include: klippy:connect, klippy:ready, klippy:shutdown,
    /// klippy:disconnect, klippy:firmware_restart, etc.
    fn register_event_handler<F>(&self, event: PrinterEvent, callback: F)
    where
        F: Fn() + Send + Sync + 'static;

    /// Send an event to all registered handlers.
    ///
    /// Returns a list of results from each handler.
    fn send_event(&self, event: &PrinterEvent) -> Vec<Result<(), KlippyError>>;

    /// Invoke the shutdown sequence.
    ///
    /// This will:
    /// 1. Set the printer state to shutdown/error
    /// 2. Fire all "klippy:shutdown" handlers
    /// 3. Fire all "klippy:analyze_shutdown" handlers with the error message and details
    fn invoke_shutdown(&self, msg: &str, details: Option<HashMap<String, String>>);

    /// Request the printer to exit its main run loop.
    ///
    /// The `result` determines what happens after the run loop exits:
    /// - "exit" / "error_exit": terminate the process
    /// - "firmware_restart": restart the printer
    fn request_exit(&self, result: &str);

    /// Run the printer's main event loop.
    ///
    /// This is the main entry point that starts the reactor loop
    /// and processes events until the printer is shut down.
    fn run(&mut self) -> String;
}

// ===========================================================================
// PrinterObject trait
// ===========================================================================

/// Trait for objects registered in the Printer's object registry.
///
/// Each Klipper config section (e.g., [stepper_x], [extruder], [printer])
/// creates a printer object. These objects are stored in the Printer's
/// object registry and can be looked up by name.
pub trait PrinterObject: Send + Sync {
    /// Get the name/identifier of this object
    fn name(&self) -> &str;

    /// Get the config type (section type) of this object
    fn config_type(&self) -> &str;

    /// Get a human-readable description
    fn description(&self) -> &str;
}
