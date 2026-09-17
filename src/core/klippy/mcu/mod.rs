//! The transport layer: frames, the message parser, the firmware data dictionary,
//! and the bare `send` / `call` pair that take message names as strings.
//!
//! Construction ([`Mcu::new`]) only brings the transport up: the parser knows the
//! host's identify formats and nothing else, so typed commands answer
//! [`McuError::NotIdentified`] until the identify handshake installs a dictionary.
//! That handshake is [`Mcu::connect`], defined in [`identify`]
//! together with the transfer it drives — this module does not call it.
//!
//! Anything that names a message in the type system —
//! [`McuCommand`](super::cmd::McuCommand), [`McuResponse`](super::cmd::McuResponse),
//! [`Params`](super::cmd::Params), and the typed calls — lives beside this module
//! in [`cmd`](super::cmd), together with the command modules themselves. Identify
//! straddles both: its formats, its transfer, and its entry points are transport
//! work in [`identify`], while its two typed views are defined in
//! [`cmd::identify`](super::cmd::identify).
//!
//! The one thing the transport asks of the modules beside it is the parser it
//! starts with: [`Mcu::new`] calls `identify::new_parser`, because the two formats
//! it has to know before anything else are identify's.

mod dictionary;
mod error;
mod pending;
mod restart_method;

pub use dictionary::{Dictionary, Enumeration, MessageDef, OutputDef};
pub use error::{McuCallError, McuError};
pub use restart_method::McuRestartMethod;

use crate::core::klippy::config::mcu::McuConfig;
use crate::core::klippy::frame::{Frame, MESSAGE_PAYLOAD_MAX};
use crate::core::klippy::identify;
use crate::core::klippy::interface::Interface;
use crate::core::klippy::mcu::pending::PendingCalls;
use crate::core::klippy::msg::error::MsgError;
use crate::core::klippy::msg::parser::Parser;
use crate::core::klippy::msg::proto::{ArgValue, Payload};
use crate::core::klippy::msg::Msg;
use std::sync::{Arc, Mutex as StdMutex};
use tokio::sync::{mpsc, oneshot};
use tokio::time::{sleep, Duration};
use tracing::{debug, error, info, warn};
/// MCU object that represents a physical microcontroller unit.
///
/// Two steps, in this order:
///
/// 1. [`Mcu::new`] brings up the transport. It is infallible, and the result is
///    not usable yet: only the host's identify messages are registered.
/// 2. [`Mcu::connect`] performs the identify handshake, which installs the
///    firmware's data dictionary and thereby makes every command the firmware
///    implements available.
///
/// [`Mcu::install_dictionary`] is the second half of step 2 on its own, for when
/// the dictionary does not come from this MCU.
///
/// The host owns no message formats beyond the identify pair, and the typed
/// command API ([`Mcu::send_msg`] / [`Mcu::call_msg`], defined in [`cmd`](super::cmd)
/// next to the vocabulary they use) refuses to run before the dictionary is in
/// place.
pub struct Mcu {
    /// MCU name
    name: String,
    /// Message parser for communication
    parser: Parser,
    /// The firmware's data dictionary, installed after the identify handshake.
    ///
    /// Protected by a plain mutex rather than an async one: it is only read and
    /// written in short, non-awaiting critical sections, and `send_msg` needs to
    /// check it from a synchronous context.
    dictionary: StdMutex<Option<Arc<Dictionary>>>,
    /// Sender for outbound payload queue
    send_buf_tx: mpsc::Sender<Payload>,
    /// Pending synchronous calls waiting for responses.
    pending_calls: PendingCalls,
    /// Interface clone kept so the device can be shut down on drop.
    interface: Interface,
    /// Handle to the receive task, used to abort it on drop.
    recv_handle: Option<tokio::task::JoinHandle<()>>,
}

impl std::fmt::Debug for Mcu {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // The parser, interface, and channels have no useful representation, so
        // report the identifying facts only.
        f.debug_struct("Mcu")
            .field("name", &self.name)
            .field("identified", &self.is_identified())
            .finish_non_exhaustive()
    }
}

impl Mcu {
    /// Create a new MCU from a name and interface.
    fn from_parts(name: String, interface: Interface) -> Self {
        info!("Creating MCU: {name}");
        // The parser starts out knowing only the two formats the host owns; every
        // other format arrives with the firmware dictionary. It is built before the
        // receive task starts, and that task gets a clone of the same registry, so
        // installing a dictionary later needs no restart (see
        // [`Mcu::install_dictionary`]).
        let parser = identify::new_parser();
        let parser_for_task = parser.clone();
        let pending_calls = PendingCalls::new();
        let pending_calls_for_task = pending_calls.clone();

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

        let interface_for_recv = interface.clone();
        let recv_handle = tokio::spawn(async move {
            let mut seq = 0u8;

            loop {
                let frame = match interface_for_recv.receive().await {
                    Some(frame) => frame,
                    // Device shut down: no more frames will arrive.
                    None => break,
                };

                // The MCU stamps every frame it sends while handling a block with
                // that block's sequence, so one request can produce several frames
                // sharing a sequence: a response per message, plus the ack that
                // carries no payload. Sequence numbers therefore identify the block
                // being answered, not the individual frame, and only ever move
                // forward — a frame is either another answer to the block we are
                // waiting on (`seq`), or the first answer to the block after it
                // (`seq + 1`). Klipper's client tracks the same thing with a send
                // window; this transport sends one block at a time and never
                // retransmits, so the window collapses to those two values.
                if frame.seq() != seq && frame.seq() != (seq + 1) & 0xf {
                    warn!(
                        "Seq mismatch: expected {} or {}, got {}",
                        seq,
                        (seq + 1) & 0xf,
                        frame.seq()
                    );
                    continue;
                }
                seq = frame.seq();

                // An empty frame is the MCU's acknowledgement of a block: it exists
                // to advance the sequence, and carries nothing to decode.
                if frame.payload().is_empty() {
                    debug!("Ack for block {}", frame.seq());
                    continue;
                }

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
                    if pending_calls_for_task.resolve(&msg.name, &params).await {
                        debug!(
                            "Pending call matched: {} (id={}), delivering {} params",
                            msg.name,
                            msg.id,
                            params.len()
                        );
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
            dictionary: StdMutex::new(None),
            send_buf_tx,
            pending_calls,
            interface,
            recv_handle: Some(recv_handle),
        }
    }

    /// Build the transport for `config`: the parser with the host's identify
    /// formats, the send task, and the receive task.
    ///
    /// The result is **not identified**. Until a dictionary is installed, the
    /// only message that can be exchanged is identify itself, and the typed
    /// command API answers [`McuError::NotIdentified`]. [`Mcu::connect`] is the
    /// normal entry point; use this one when the interface has to be brought up
    /// before the handshake — to inspect it, to retry it with a custom timeout,
    /// or because the dictionary comes from somewhere else.
    ///
    /// The two background tasks outlive this call and are stopped by [`Drop`].
    ///
    /// `config.restart_method` is not read here: restarting the firmware is a
    /// planned feature, and the field is only carried until then (see
    /// [`McuRestartMethod`]).
    pub fn new(config: McuConfig) -> Self {
        Self::from_parts(config.name, config.interface)
    }

    /// Install the firmware's data dictionary.
    ///
    /// Called by [`Mcu::identify`] once the identify payload has been fetched; it
    /// is public because a dictionary can also arrive from elsewhere, such as a
    /// cached copy or a test fixture. Two things happen:
    ///
    /// * commands and responses are registered with the parser. The receive task
    ///   holds a clone of the same [`Parser`], so decoded messages become
    ///   available immediately — no task restart, no parser rebuild.
    /// * the dictionary itself is retained, for enumerations and constants.
    ///
    /// Returns the number of messages that were newly registered.
    ///
    /// # Errors
    /// Returns [`McuError`] if a format string cannot be parsed, or if an id or
    /// name collides with a different, already-registered message.
    pub fn install_dictionary(&self, dictionary: Dictionary) -> Result<usize, McuError> {
        // `Parser` is a thin handle over shared state, and `register` only needs
        // `&mut` on the handle, so cloning it is enough to install while `&self`
        // is borrowed. The clone shares the registry the receive task uses.
        let mut parser = self.parser.clone();
        let installed = dictionary.install(&mut parser)?;

        let mut slot = self.dictionary.lock().expect("dictionary lock poisoned");
        *slot = Some(Arc::new(dictionary));
        Ok(installed)
    }

    /// The installed data dictionary, or `None` before the identify handshake.
    pub fn dictionary(&self) -> Option<Arc<Dictionary>> {
        self.dictionary
            .lock()
            .expect("dictionary lock poisoned")
            .clone()
    }

    /// Whether the identify handshake has installed a dictionary.
    pub fn is_identified(&self) -> bool {
        self.dictionary
            .lock()
            .expect("dictionary lock poisoned")
            .is_some()
    }

    /// Require an installed dictionary before running a typed command.
    pub(crate) fn require_dictionary(&self) -> Result<Arc<Dictionary>, McuError> {
        self.dictionary().ok_or(McuError::NotIdentified)
    }

    /// Look up a registered message, failing fast when it is unknown.
    ///
    /// The typed calls in [`crate::core::klippy::cmd`] resolve both names through
    /// this before sending anything, which is what turns an unimplemented message
    /// into an immediate error instead of a timeout.
    pub(crate) fn require_message(&self, name: &str) -> Result<Arc<Msg>, McuError> {
        self.parser
            .lookup(name)
            .ok_or_else(|| McuError::UnknownMessage(name.to_string()))
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
        self.pending_calls
            .register(response_name.to_string(), tx)
            .await;

        // 4. Send the command.
        if let Err(e) = self.send(command, args) {
            warn!("Failed to send command '{}': {e}", command);
            // Clean up the pending call on send failure.
            self.pending_calls.cancel(response_name).await;
            return Err(McuCallError::SendFailed(e.msg));
        }
        debug!("Command '{}' sent, waiting for response '{}'", command, response_name);

        // 5. Wait for the response. A successful resolve already consumed the
        // registration, so only the failure paths need to clean up.
        match tokio::time::timeout(timeout, rx).await {
            Ok(Ok(params)) => {
                debug!("Response received for '{}': {} params", response_name, params.len());
                Ok(params)
            }
            Ok(Err(_recv)) => {
                // Receiver dropped (shouldn't happen in normal flow).
                self.pending_calls.cancel(response_name).await;
                error!("Response receiver dropped for '{}'", response_name);
                Err(McuCallError::SendFailed("response receiver dropped".to_string()))
            }
            Err(_) => {
                // Timeout — clean up the pending call.
                self.pending_calls.cancel(response_name).await;
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

    /// Bind a callback to a registered message, replacing any existing one.
    ///
    /// This is the callback counterpart of [`Mcu::send`] / [`Mcu::call`]: the
    /// typed entry point is `Mcu::bind_event` in
    /// [`event`](crate::core::klippy::event), which decodes the values by name
    /// before handing them to the handler. The callback receives the decoded
    /// parameter values in message declaration order and runs on the receive
    /// task.
    ///
    /// # Errors
    /// Returns [`McuError::Msg`] if no message called `name` is registered.
    pub(crate) fn bind_callback(
        &self,
        name: &str,
        callback: impl FnMut(&[ArgValue]) + Send + 'static,
    ) -> Result<(), McuError> {
        // `Parser` is a thin handle over shared state, and `bind` only needs
        // `&mut` on the handle, so cloning it is enough to bind while `&self`
        // is borrowed. The clone shares the registry the receive task reads, so
        // the callback is live immediately.
        let mut parser = self.parser.clone();
        parser.bind(name, callback)?;
        Ok(())
    }
}

impl Drop for Mcu {
    /// Shut down the interface and abort the receive task when `Mcu` is dropped.
    ///
    /// The receive task runs an infinite loop calling `interface.receive().await`,
    /// so it has no natural exit condition. That call is backed by a synchronous
    /// device read inside `spawn_blocking`, which **cannot** be cancelled by
    /// aborting the async task. If the blocked read is not released, its thread
    /// stays parked forever and the runtime hangs during shutdown. Calling
    /// [`Interface::shutdown`] first unblocks that read; the abort then
    /// guarantees the task itself is torn down promptly.
    fn drop(&mut self) {
        self.interface.shutdown();
        if let Some(handle) = self.recv_handle.take() {
            handle.abort();
        }
    }
}

#[cfg(test)]
impl Mcu {
    /// Build a transport over a bare interface, without a config.
    ///
    /// Tests talk to a [`TestDevice`](crate::core::klippy::interface::test::TestDevice)
    /// rather than a real `McuConfig`, and most of them never identify.
    pub(crate) fn for_test(name: impl Into<String>, interface: Interface) -> Self {
        Self::from_parts(name.into(), interface)
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::interface::test::{MappingEntry, TestDevice};
    use crate::core::klippy::interface::Interface;

    fn make_frame(seq: u8, payload: &[u8]) -> Frame {
        Frame::new(seq, payload.to_vec())
    }

    // -----------------------------------------------------------------------
    // Mcu creation
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_new_starts_unidentified() {
        let mcu = Mcu::new(McuConfig {
            name: "test_mcu".to_string(),
            restart_method: McuRestartMethod::Command,
            interface: Interface::new(TestDevice::new(vec![])),
        });

        assert_eq!(mcu.name(), "test_mcu");

        // Construction is transport only: the parser knows the host's identify
        // pair and nothing else. Everything else arrives with `Mcu::connect`.
        assert!(!mcu.is_identified());
        assert!(mcu.parser.is_registered("identify"));
        assert!(mcu.parser.is_registered("identify_response"));
    }

    /// The shapes a real MCU produces for one request: the response, and the ack
    /// that carries no payload at all.
    #[tokio::test]
    async fn test_acks_and_repeated_sequences_are_accepted() {
        let mut parser = Parser::new();
        parser.register(5, "get_clock").unwrap();
        parser.register(18, "clock clock=%u").unwrap();

        let request = make_frame(0, &[5]);
        let mut answer = Payload::new();
        answer.push_i16(18).unwrap();
        answer.push_u32(0x1234).unwrap();

        // The firmware stamps everything it sends while handling a block with that
        // block's sequence, so the ack repeats the response's sequence. A transport
        // that expected a new number per frame would drop the ack, advance its
        // counter, and then reject the next exchange's answer.
        let device = TestDevice::new(vec![
            MappingEntry {
                input: request.clone(),
                outputs: vec![
                    make_frame(0, &answer.clone().into_raw()),
                    make_frame(0, &[]), // ack: no payload
                ],
            },
            MappingEntry {
                input: make_frame(1, &[5]),
                outputs: vec![make_frame(1, &answer.into_raw())],
            },
        ]);
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        let dictionary = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": 5},
            "responses": {"clock clock=%u": 18}
        }))
        .unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        // Both exchanges have to complete: the first proves the ack was ignored
        // rather than counted, the second proves it did not desynchronise the
        // stream.
        for _ in 0..2 {
            let params = mcu
                .call("get_clock", &[], "clock", Duration::from_millis(500))
                .await
                .expect("both exchanges must complete");
            assert_eq!(params, vec![ArgValue::UInt32(0x1234)]);
        }
    }

    #[tokio::test]
    async fn test_stale_sequence_is_dropped() {
        let mut parser = Parser::new();
        parser.register(5, "get_clock").unwrap();
        parser.register(18, "clock clock=%u").unwrap();
        let mut answer = Payload::new();
        answer.push_i16(18).unwrap();
        answer.push_u32(1).unwrap();

        // A frame numbered well behind the block being answered is not part of
        // this exchange, so it is dropped and the call times out.
        let device = TestDevice::new(vec![MappingEntry {
            input: make_frame(0, &[5]),
            outputs: vec![make_frame(9, &answer.into_raw())],
        }]);
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        let dictionary = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": 5},
            "responses": {"clock clock=%u": 18}
        }))
        .unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        let err = mcu
            .call("get_clock", &[], "clock", Duration::from_millis(50))
            .await
            .unwrap_err();
        assert!(
            matches!(err, McuCallError::Timeout(_)),
            "expected a timeout, got {err:?}"
        );
    }

    // -----------------------------------------------------------------------
    // Parser helpers (no background tasks needed)
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_send_invalid_command() {
        let device = TestDevice::new(vec![]);
        let interface = Interface::new(device);
        let mcu = Mcu::for_test("test_mcu", interface);

        let result = mcu.send("nonexistent_cmd", &[ArgValue::UInt32(0)]);
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_send_wrong_param_count() {
        let device = TestDevice::new(vec![]);
        let interface = Interface::new(device);
        let mcu = Mcu::for_test("test_mcu", interface);

        // %u requires one param
        let result = mcu.send("test_cmd", &[]);
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_send_wrong_param_type() {
        let device = TestDevice::new(vec![]);
        let interface = Interface::new(device);
        let mcu = Mcu::for_test("test_mcu", interface);

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

    // -----------------------------------------------------------------------
    // Drop / shutdown
    // -----------------------------------------------------------------------

    /// Dropping the `Mcu` must shut down cleanly even when the underlying
    /// device keeps its receive channel open forever.
    ///
    /// The `TestDevice` is filled with extra (unconsumed) mappings on purpose:
    /// `TestDevice::send` only drops the last `buf_tx` sender once all mappings
    /// have been consumed, so with mappings still queued the frame channel stays
    /// open and `receive()` blocks indefinitely. This guarantees the receive
    /// task does not end on its own, so the test truly exercises the shutdown
    /// path added by `impl Drop for Mcu`.
    ///
    /// The test verifies two things:
    /// 1. the async receive task is aborted, and
    /// 2. the blocking device read is released (via `Interface::shutdown`),
    ///    so the runtime shuts down instead of hanging on a parked
    ///    `spawn_blocking` thread.
    #[tokio::test]
    async fn test_drop_aborts_receive_task_with_open_interface() {
        let mappings: Vec<MappingEntry> = (0..4)
            .map(|i| MappingEntry {
                input: make_frame(i, b"cmd"),
                outputs: vec![make_frame(i + 16, b"resp")],
            })
            .collect();

        let device = TestDevice::new(mappings);
        let interface = Interface::new(device);
        let mcu = Mcu::for_test("drop_test", interface);

        // `JoinHandle` is not `Clone`; an `AbortHandle` lets us observe the
        // receive task after the `Mcu` (and its `JoinHandle`) is gone.
        let abort_handle = mcu
            .recv_handle
            .as_ref()
            .expect("receive task handle must be present")
            .abort_handle();

        // Let the receive task start and block inside `interface.receive()`.
        sleep(Duration::from_millis(20)).await;
        assert!(
            !abort_handle.is_finished(),
            "receive task should still be running while the interface is open"
        );

        // Actively drop the `Mcu`; this must abort the receive task.
        drop(mcu);

        // Give the runtime a chance to process the cancellation.
        sleep(Duration::from_millis(20)).await;
        assert!(
            abort_handle.is_finished(),
            "receive task should be aborted after the Mcu is dropped"
        );
    }
}
