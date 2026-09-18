// Printer module
//
// This module provides printer implementations for Klipper hosts.
// A printer owns the host's lifecycle: it comes up, reports the state it is in,
// fires the events the rest of the host waits for, and runs until something
// asks it to stop.
//
// Which implementation a host runs is decided by the machine's kinematics
// (`[printer] kinematics`). A machine with no steppers — `kinematics: none` —
// is a host with no hardware, and runs the equally empty `NonePrinter`.
//
// Available implementations:
// - `none`: host-only printer for `kinematics: none` (developer testing)

pub mod none;
pub mod printer;

// Re-export common types
pub use none::NonePrinter;
pub use printer::{load_printer, EventHandler, Printer, PrinterEvent, PrinterState, StateMessage};
