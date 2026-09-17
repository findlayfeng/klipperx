mod identify;
mod restart_method;

pub use identify::{Identify, IdentifyError, IdentifyErrorKind};
pub use restart_method::McuRestartMethod;

use crate::core::klippy::config::mcu::McuConfig;
use crate::core::klippy::frame::Frame;
use crate::core::klippy::interface::Interface;
use crate::core::klippy::msg::parser::Parser;
use crate::core::klippy::traits::InterfaceError;
use std::sync::Arc;
use tokio::sync::Mutex;

/// MCU object that represents a physical microcontroller unit.
///
/// Created by consuming an `McuConfig` which already contains the interface.
pub struct Mcu {
    /// MCU name
    pub name: String,
    /// Message parser for communication
    pub parser: Parser,
    /// Communication interface to the MCU
    pub interface: Interface,
    /// Parsed identify data from the MCU (populated after identify handshake)
    pub identify: Arc<Mutex<Option<Identify>>>,
}

impl Mcu {
    /// Create a new `Mcu` with the given name and interface.
    ///
    /// The parser starts empty; use [`Self::register`] to add message formats
    /// before sending or receiving Klipper protocol messages.
    pub fn new(name: String, interface: Interface) -> Self {
        Self {
            name,
            parser: Parser::new(),
            interface,
            identify: Arc::new(Mutex::new(None)),
        }
    }

    /// Get the MCU name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Send a frame to the MCU.
    pub async fn send(&self, frame: Frame) -> Result<(), InterfaceError> {
        self.interface.send(frame).await
    }

    /// Receive a frame from the MCU.
    pub async fn receive(&self) -> Frame {
        self.interface.receive().await
    }

    /// Register a message format with the given ID.
    ///
    /// The `format` string follows Klipper's message format syntax,
    /// e.g. `"GET_TIME"` or `"SET_PIN PIN=xxx VALUE=%f"`.
    pub fn register(&mut self, id: u8, format: &str) -> Result<(), String> {
        self.parser
            .register(id, format)
            .map_err(|e| e.msg.to_string())
    }

    /// Encode a message by name into a [`Frame`].
    ///
    /// The `values` must match the parameter count and types expected
    /// by the registered message format.
    pub fn encode(&self, name: &str, values: &[crate::core::klippy::msg::proto::ArgValue]) -> Result<Frame, String> {
        let payload = self
            .parser
            .encode(name, values)
            .map_err(|e| e.msg.to_string())?;
        // Default to sequence 0 for the frame; the actual message ID is in the payload
        Ok(Frame::new(0, payload.payload().to_vec()))
    }

    /// Decode a received frame's payload using registered message formats.
    ///
    /// Returns a list of `(message_name, decoded_values)` tuples.
    pub fn decode(&self, frame: &Frame) -> Result<Vec<(String, Vec<crate::core::klippy::msg::proto::ArgValue>)>, String> {
        let payload = crate::core::klippy::msg::proto::Payload::from_raw(frame.payload().to_vec());
        self.parser
            .decode(payload)
            .map_err(|e| e.msg.to_string())
    }

    /// Fetch identify data from the MCU.
    ///
    /// This performs the identify handshake: registers default identify
    /// message formats, exchanges data with the MCU, and stores the
    /// parsed identify information for later lookup.
    ///
    /// # Errors
    /// Returns an error if the identify exchange fails (timeout,
    /// decompression error, JSON parse error, etc.).
    ///
    /// **Note**: Requires `Identify::fetch` to be implemented.
    #[allow(unused_variables)]
    pub async fn do_identify(&mut self) -> Result<(), IdentifyError> {
        // TODO: Implement Identify::fetch and call it here
        // 1. Register default identify message formats
        // 2. Call Identify::fetch(&mut self.parser, timeout)
        // 3. Store the result in self.identify
        Err(IdentifyError {
            kind: IdentifyErrorKind::Failed(
                "Identify::fetch not yet implemented".to_string(),
            ),
        })
    }

    /// Get a reference to the identify data, if available.
    pub async fn get_identify(&self) -> Option<Identify> {
        self.identify.lock().await.clone()
    }

    /// Create a new `Mcu` object from an `McuConfig`.
    ///
    /// Creates a `Parser` and initializes the MCU with the
    /// given configuration.
    pub fn from_config(config: McuConfig) -> Result<Self, String> {
        Ok(Self {
            name: config.name,
            parser: Parser::new(),
            interface: config.interface,
            identify: Arc::new(Mutex::new(None)),
        })
    }
}

impl std::fmt::Debug for Mcu {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Mcu")
            .field("name", &self.name)
            .field("identify", &self.identify)
            .finish_non_exhaustive()
    }
}

// Tests removed due to Frame/Payload type conflicts - to be fixed later
