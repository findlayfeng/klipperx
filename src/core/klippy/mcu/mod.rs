mod identify;
mod restart_method;

pub use identify::{Identify, IdentifyError, IdentifyErrorKind};
pub use restart_method::McuRestartMethod;

use crate::core::klippy::config::mcu::McuConfig;
use crate::core::klippy::frame::{Frame, MESSAGE_PAYLOAD_MAX};
use crate::core::klippy::interface::Interface;
use crate::core::klippy::mcu::identify::DEFAULT_MESSAGES;
use crate::core::klippy::msg::parser::Parser;
use crate::core::klippy::msg::proto::Payload;
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
    /// Communication interface to the MCU
    interface: Interface,
    /// Parsed identify data from the MCU (populated after identify handshake)
    identify: Arc<Mutex<Option<Identify>>>,
}

impl From<McuConfig> for Mcu {
    fn from(value: McuConfig) -> Self {
        let mut mcu: Self = (value.name, value.interface).into();

        for (id, fmt) in DEFAULT_MESSAGES {
            mcu.parser
                .register(*id, fmt)
                .expect("default identify message format must be valid");
        }

        mcu
    }
}

impl From<(String, Interface)> for Mcu {
    fn from((name, interface): (String, Interface)) -> Self {
        Self {
            name,
            parser: Parser::new(),
            interface,
            identify: Arc::new(Mutex::new(None)),
        }
    }
}

impl Mcu {
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

    /// Run the MCU communication loop in a separate async task.
    ///
    /// This method shares the MCU state via Arc and spawns a new tokio task
    /// that runs the main communication loop. The loop batches outbound payloads
    /// until the buffer is sufficiently full or a timeout expires, then sends them.
    ///
    /// Returns a sender channel for batching outbound payloads before sending.
    /// When the sender is dropped, the task will clean up automatically.
    pub fn run(&self) -> mpsc::Sender<Payload> {
        let interface = self.interface.clone();
        let (send_buf_tx, mut send_buf_rx) = mpsc::channel::<Payload>(32);

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
                                        Self::send_batch(&interface, &mut seq, payload).await;
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
                Self::send_batch(&interface, &mut seq, payload).await;
            }
        });

        send_buf_tx
    }
}
