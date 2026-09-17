// Klippy module - abstract interface for Klipper device communication
//
// This module provides:
// - Interface enum: abstract interface for printer communication
// - SerialInterface: serial interface implementation
// - LibInterface: real implementation using klipper host library (temporarily disabled)
//   See interface/host.rs for the original implementation (commented out from build)

pub mod config;
pub mod error;
pub mod feature;
pub mod frame;
pub mod kinematics;
pub mod mcu;
pub mod msg;
pub mod printer;
// pub mod toolhead;
pub mod traits;

// Dynamic library loading interface
pub mod interface;

// Re-export common types for convenience
pub use error::KlippyError;
pub use interface::Interface;
pub use msg::proto::Payload;
pub use msg::Msg;
pub use traits::{
    InterfaceEvent, Printer, PrinterEvent, PrinterObject, PrinterState, StateMessage,
};
// pub use interface::serial::SerialInterface;
// pub use interface::canbus::CanbusInterface;
// pub use mcu::{MCU, McuError, McuPin, PinParams};
// pub use mcu::{add_printer_objects, get_printer_mcu};
// pub use msg::proto::{ArgType, MsgParams, MsgValue, crc32, msg_params};
