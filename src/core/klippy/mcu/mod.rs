mod identify;
mod restart_method;

pub use identify::{Identify, IdentifyError, IdentifyErrorKind};
pub use restart_method::McuRestartMethod;

use crate::core::klippy::config::mcu::McuConfig;
use crate::core::klippy::frame::{Frame, MESSAGE_PAYLOAD_MAX};
use crate::core::klippy::interface::Interface;
use crate::core::klippy::mcu::identify::DEFAULT_MESSAGES;
use crate::core::klippy::msg::error::MsgError;
use crate::core::klippy::msg::parser::Parser;
use crate::core::klippy::msg::proto::{ArgValue, Payload};
use std::sync::Arc;
use tokio::sync::{mpsc, Mutex};
use tokio::time::{sleep, Duration};

/// MCU object that represents a physical microcontroller unit.
///
/// Created by consuming an `McuConfig` which already contains the interface.
pub struct Mcu {
    /// MCU name
    name: String,
    /// Message parser for communication
    parser: Parser,
    /// Parsed identify data from the MCU (populated after identify handshake)
    identify: Arc<Mutex<Option<Identify>>>,
    /// Sender for outbound payload queue
    send_buf_tx: mpsc::Sender<Payload>,
}

impl Mcu {
    /// Initialize the parser with default identify messages.
    fn init_parser(parser: &mut Parser) {
        for (id, fmt) in DEFAULT_MESSAGES {
            parser
                .register(*id, fmt)
                .expect("default identify message format must be valid");
        }
    }

    /// Create a new MCU from a name and interface.
    fn from_parts(name: String, interface: Interface) -> Self {
        let (send_buf_tx, mut send_buf_rx) = mpsc::channel::<Payload>(32);
        let interface_for_send = interface.clone();

        tokio::spawn(async move {
            let mut seq = 0u8;

            loop {
                // Wait for at least one payload to start a batch
                let mut payload = match send_buf_rx.recv().await {
                    Some(p) => p,
                    None => break, // channel closed
                };

                // Try to batch more payloads
                loop {
                    // Check if buffer is sufficiently full
                    if payload.len() >= MESSAGE_PAYLOAD_MAX * 2 / 3 {
                        break;
                    }

                    // Wait for more data or timeout
                    tokio::select! {
                        maybe_next = send_buf_rx.recv() => {
                            match maybe_next {
                                Some(next_payload) => {
                                    if payload.try_merge(&next_payload).is_err() {
                                        // Merge failed (would exceed max), send current batch first
                                        Self::send_batch(&interface_for_send, &mut seq, payload).await;
                                        // Start new batch with next
                                        payload = next_payload;
                                        continue;
                                    }
                                }
                                None => {
                                    // Channel closed
                                    break;
                                }
                            }
                        }
                        _ = sleep(Duration::from_millis(1)) => {
                            // Timeout, send accumulated batch
                            break;
                        }
                    }
                }

                // send the batched payload to the MCU
                Self::send_batch(&interface_for_send, &mut seq, payload).await;
            }
        });

        let mut parser = Parser::new();
        Self::init_parser(&mut parser);
        let parser_for_task = parser.clone();

        tokio::spawn(async move {
            let mut seq = 0u8;

            loop {
                let frame = interface.receive().await;
                if frame.seq() != seq {
                    todo!()
                }

                seq += 1;

                let _ = parser_for_task.decode(frame.into()).expect("todo");
            }
        });

        Self {
            name,
            parser,
            identify: Arc::new(Mutex::new(None)),
            send_buf_tx,
        }
    }

    /// Send a batched payload to the MCU.
    ///
    /// Increments the sequence number on success, logs errors.
    async fn send_batch(interface: &Interface, seq: &mut u8, payload: Payload) {
        let seq_num = *seq;
        match interface
            .send(Frame::new(seq_num, payload.into_raw()))
            .await
        {
            Ok(()) => *seq = (seq_num + 1) & 0xf,
            Err(e) => eprintln!("[mcu:{seq_num}] send failed: {e}"),
        }
    }

    /// Get the MCU name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Encode and send a command to the MCU.
    ///
    /// Converts the command name and arguments to a [`Payload`] using the
    /// registered message format, then queues it for sending.
    ///
    /// # Errors
    /// Returns [`MsgError`] if the message name is unknown, the arguments
    /// don't match the expected parameter count, or the send buffer is full.
    pub fn send(&self, name: &str, args: &[ArgValue]) -> Result<(), MsgError> {
        let payload = self.parser.encode(name, args)?;
        self.send_buf_tx
            .try_send(payload)
            .map_err(|e| MsgError::new(e.to_string()))?;
        Ok(())
    }
}

impl From<McuConfig> for Mcu {
    fn from(value: McuConfig) -> Self {
        Self::from_parts(value.name, value.interface)
    }
}

impl From<(String, Interface)> for Mcu {
    fn from((name, interface): (String, Interface)) -> Self {
        Self::from_parts(name, interface)
    }
}
