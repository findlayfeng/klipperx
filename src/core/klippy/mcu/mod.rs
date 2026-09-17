mod error;
mod identify;
mod restart_method;

pub use error::McuCallError;
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
use tokio::sync::{mpsc, oneshot, Mutex};
use tokio::time::{sleep, Duration};
use tracing::{debug, error, info, warn};

/// A pending call waiting for a response from the MCU.
struct PendingCall {
    /// Name of the response message to match against.
    response_name: String,
    /// Sender to deliver the decoded parameters.
    response_tx: oneshot::Sender<Vec<ArgValue>>,
}

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
    /// Pending synchronous calls waiting for responses.
    pending_calls: Arc<Mutex<Vec<PendingCall>>>,
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
        info!("Creating MCU: {name}");
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
                debug!("Sending batch: {} bytes", payload.len());
                Self::send_batch(&interface_for_send, &mut seq, payload).await;
            }
        });

        let mut parser = Parser::new();
        Self::init_parser(&mut parser);
        let parser_for_task = parser.clone();
        let pending_calls: Arc<Mutex<Vec<PendingCall>>> =
            Arc::new(Mutex::new(Vec::new()));
        let pending_calls_for_task = pending_calls.clone();

        tokio::spawn(async move {
            let mut seq = 0u8;

            loop {
                let frame = interface.receive().await;

                if frame.seq() != seq {
                    warn!(
                        "Seq mismatch: expected {seq}, got {}",
                        frame.seq()
                    );
                    seq += 1;
                    continue;
                }

                seq += 1;

                let decoded = match parser_for_task.decode(frame.into()) {
                    Ok(msgs) => {
                        debug!("Decoded {} messages", msgs.len());
                        msgs
                    }
                    Err(e) => {
                        error!("Decode error: {e}");
                        continue;
                    }
                };

                for (msg, params) in decoded {
                    // Pending call has priority — if matched, consume and skip callback.
                    let mut pending = pending_calls_for_task.lock().await;
                    if let Some(idx) = pending
                        .iter()
                        .position(|pc| pc.response_name == msg.name)
                    {
                        debug!(
                            "Pending call matched: {} (id={}), delivering {} params",
                            msg.name, msg.id, params.len()
                        );
                        let call = pending.remove(idx);
                        let _ = call.response_tx.send(params);
                        continue;
                    }
                    // No pending call — fall back to callback.
                    if let Some(callback) = &msg.callback {
                        debug!(
                            "Invoking callback for {} (id={})",
                            msg.name, msg.id
                        );
                        let mut cb = callback.lock().unwrap();
                        cb(params.as_slice());
                    } else {
                        // No callback and no pending call — discard with warning.
                        warn!(
                            "Unhandled message {} (id={}), discarding",
                            msg.name, msg.id
                        );
                    }
                }
            }
        });

        Self {
            name,
            parser,
            identify: Arc::new(Mutex::new(None)),
            send_buf_tx,
            pending_calls,
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
            Err(e) => error!("Send failed (seq={seq_num}): {e}"),
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

    /// Send a command and wait for the response message.
    ///
    /// This is a synchronous request/response pattern: the command is sent,
    /// then the caller blocks (async) until the response message arrives or
    /// `timeout` elapses.
    ///
    /// # Requirements
    /// - `command` must be registered in the message parser.
    /// - `command` must **not** have a callback bound (callback-registered
    ///   messages are for asynchronous notification, not request/response).
    /// - The response message identified by `response_name` must also be
    ///   registered (with or without a callback).
    ///
    /// # Errors
    /// Returns [`McuCallError`] if the command is unknown, already has a
    /// callback, the send buffer is full, or the response does not arrive
    /// within `timeout`.
    pub async fn call(
        &self,
        command: &str,
        args: &[ArgValue],
        response_name: &str,
        timeout: Duration,
    ) -> Result<Vec<ArgValue>, McuCallError> {
        // 1. Verify command is registered; warn if it has a callback.
        if !self.parser.is_registered(command) {
            return Err(McuCallError::CommandNotFound(command.to_string()));
        }
        if self.parser.has_callback(command) {
            warn!(
                "Command '{}' already has a callback, call may not work as expected",
                command
            );
        }
        info!("Calling command: {command} (response: {response_name}, timeout: {:?})", timeout);

        // 2. Create a oneshot channel for the response.
        let (tx, rx) = oneshot::channel::<Vec<ArgValue>>();

        // 3. Register the pending call.
        {
            let mut pending = self.pending_calls.lock().await;
            pending.push(PendingCall {
                response_name: response_name.to_string(),
                response_tx: tx,
            });
        }

        // 4. Send the command.
        if let Err(e) = self.send(command, args) {
            warn!("Failed to send command '{}': {e}", command);
            // Clean up the pending call on send failure.
            self.pending_calls
                .lock()
                .await
                .retain(|pc| pc.response_name != response_name);
            return Err(McuCallError::SendFailed(e.msg));
        }
        debug!("Command '{}' sent, waiting for response '{}'", command, response_name);

        // 5. Wait for the response.
        match tokio::time::timeout(timeout, rx).await {
            Ok(Ok(params)) => {
                debug!("Response received for '{}': {} params", response_name, params.len());
                // Clean up the pending call.
                self.pending_calls
                    .lock()
                    .await
                    .retain(|pc| pc.response_name != response_name);
                Ok(params)
            }
            Ok(Err(_recv)) => {
                // Receiver dropped (shouldn't happen in normal flow).
                self.pending_calls
                    .lock()
                    .await
                    .retain(|pc| pc.response_name != response_name);
                error!("Response receiver dropped for '{}'", response_name);
                Err(McuCallError::SendFailed("response receiver dropped".to_string()))
            }
            Err(_) => {
                // Timeout — clean up the pending call.
                self.pending_calls
                    .lock()
                    .await
                    .retain(|pc| pc.response_name != response_name);
                warn!(
                    "Timeout waiting for response '{}'",
                    response_name
                );
                Err(McuCallError::Timeout(format!(
                    "no response for {} within {:?}",
                    response_name, timeout
                )))
            }
        }
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

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::interface::test::TestDevice;
    use crate::core::klippy::interface::Interface;

    fn make_frame(seq: u8, payload: &[u8]) -> Frame {
        Frame::new(seq, payload.to_vec())
    }

    // -----------------------------------------------------------------------
    // Mcu creation
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_mcu_creation() {
        let device = TestDevice::new(vec![]);
        let interface = Interface::new(device);
        let mcu = Mcu::from(("test_mcu".to_string(), interface));

        assert_eq!(mcu.name(), "test_mcu");
    }

    // -----------------------------------------------------------------------
    // Parser helpers (no background tasks needed)
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_send_invalid_command() {
        let device = TestDevice::new(vec![]);
        let interface = Interface::new(device);
        let mcu = Mcu::from(("test_mcu".to_string(), interface));

        let result = mcu.send("nonexistent_cmd", &[ArgValue::UInt32(0)]);
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_send_wrong_param_count() {
        let device = TestDevice::new(vec![]);
        let interface = Interface::new(device);
        let mcu = Mcu::from(("test_mcu".to_string(), interface));

        // %u requires one param
        let result = mcu.send("test_cmd", &[]);
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_send_wrong_param_type() {
        let device = TestDevice::new(vec![]);
        let interface = Interface::new(device);
        let mcu = Mcu::from(("test_mcu".to_string(), interface));

        // test_cmd expects x=%u but we pass a string
        let result = mcu.send("test_cmd", &[ArgValue::Str("bad".to_string())]);
        assert!(result.is_err());
    }

    // -----------------------------------------------------------------------
    // Parser encode / decode roundtrip (no background tasks)
    // -----------------------------------------------------------------------

    #[test]
    fn test_parser_encode_decode_roundtrip() {
        let mut parser = Parser::new();
        parser.register(100, "test_cmd x=%u").unwrap();
        parser.register(101, "test_resp val=%u data=%.*s").unwrap();

        // Encode
        let payload = parser.encode("test_cmd", &[ArgValue::UInt32(42)]).unwrap();
        let raw = payload.into_raw();

        // Decode
        let frame = make_frame(0, &raw);
        let decoded = parser.decode(frame.into()).unwrap();
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].0.name, "test_cmd");
        assert_eq!(decoded[0].0.id, 100);
        assert_eq!(decoded[0].1.len(), 1);
        assert_eq!(decoded[0].1[0], ArgValue::UInt32(42));
    }

    #[test]
    fn test_parser_decode_response_message() {
        let mut parser = Parser::new();
        parser.register(101, "test_resp val=%u data=%.*s").unwrap();

        let mut payload = Payload::new();
        payload.push_i16(101).unwrap();
        payload.push_u32(99).unwrap();
        payload.push_bytes(b"ok").unwrap();

        let frame = make_frame(0x10, &payload.into_raw());
        let decoded = parser.decode(frame.into()).unwrap();
        assert_eq!(decoded.len(), 1);
        assert_eq!(decoded[0].0.name, "test_resp");
        assert_eq!(decoded[0].1.len(), 2);
        assert_eq!(decoded[0].1[0], ArgValue::UInt32(99));
        assert_eq!(decoded[0].1[1], ArgValue::Bytes(b"ok".to_vec()));
    }
}
