// Klippy module - abstract interface for Klipper device communication
//
// This module provides:
// - KlippyInterface trait: abstract interface for printer communication
// - SerialInterface: serial interface implementation
// - LibInterface: real implementation using klipper host library (temporarily disabled)
//   See interface/host.rs for the original implementation (commented out from build)

pub mod config;
pub mod error;
pub mod frame;
pub mod kinematics;
pub mod mcu;
pub mod msg;
pub mod printer;
// pub mod toolhead;
pub mod traits;

// Dynamic library loading interface
#[allow(dead_code)]
pub mod interface {
    // Serial interface implementation (default interface)
    // pub mod serial;

    // Canbus interface implementation
    // pub mod canbus;

    // Klipper host library interface (dynamic linking)
    // Loads libklipper_host.so at runtime and resolves function pointers
    // Temporarily disabled from build — see interface/host.rs
    // pub mod host;

    // Test interface - provides pre-defined responses for deterministic testing
    #[cfg(test)]
    pub mod test;
}

// Re-export common types for convenience
pub use error::KlippyError;
pub use msg::proto::Payload;
pub use msg::MsgBase;
pub use traits::{
    InterfaceEvent, KlippyInterface, Printer, PrinterEvent, PrinterObject, PrinterState,
    StateMessage,
};
// pub use interface::serial::SerialInterface;
// pub use interface::canbus::CanbusInterface;
// pub use mcu::{MCU, McuError, McuPin, PinParams};
// pub use mcu::{add_printer_objects, get_printer_mcu};
// pub use msg::proto::{ArgType, MsgParams, MsgValue, crc32, msg_params};
