// Klippy module - abstract interface for Klipper device communication
//
// This module provides:
// - Interface enum: abstract interface for printer communication
// - SerialDevice: a real MCU on a serial port (see interface/serial.rs)
// - HostDevice: real implementation running klipper's host library, loaded at
//   runtime (see interface/host.rs)

pub mod cmd;
pub mod config;
pub mod error;
pub mod event;
pub mod frame;
pub mod identify;
pub mod kinematics;
pub mod mcu;
pub mod msg;
pub mod printer;
// pub mod toolhead;

// Client-facing API over the Unix Domain Socket
pub mod api;

// Dynamic library loading interface
pub mod interface;

// Re-export common types for convenience
pub use error::KlippyError;
pub use interface::{HostDevice, Interface, SerialDevice};
pub use msg::proto::Payload;
pub use msg::Msg;
pub use printer::{load_printer, Printer, PrinterEvent, PrinterState, StateMessage};
// pub use interface::canbus::CanbusInterface;
// pub use mcu::{MCU, McuError, McuPin, PinParams};
// pub use mcu::{add_printer_objects, get_printer_mcu};
// pub use msg::proto::{ArgType, MsgParams, MsgValue, crc32, msg_params};
