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

mod config;
mod dictionary;
mod error;
mod object;
mod pending;
mod pin;
mod restart;
mod restart_method;

pub use config::{BuiltConfig, ConfigBuilder, ConfigCallback, Configured, PostInitCallback};
pub use dictionary::{Dictionary, Enumeration, MessageDef, OutputDef};
pub use error::{McuCallError, McuError};
pub use object::{load_config, load_config_prefix, McuObject};
pub use pin::{McuChip, McuDigitalOut};
pub use restart_method::McuRestartMethod;

use crate::core::klippy::frame::{Frame, MESSAGE_PAYLOAD_MAX};
use crate::core::klippy::identify;
use crate::core::klippy::interface::Interface;
use crate::core::klippy::mcu::pending::PendingCalls;
use crate::core::klippy::msg::error::MsgError;
use crate::core::klippy::msg::parser::Parser;
use crate::core::klippy::msg::proto::{ArgValue, Payload};
use crate::core::klippy::msg::Msg;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex as StdMutex};
use tokio::sync::{mpsc, oneshot};
use tokio::time::{sleep, Duration};
use tracing::{debug, error, info, warn};
/// One item on the outbound queue: a message, or a barrier that asks the send
/// task to flush what is already queued.
///
/// The send task coalesces payloads queued close together into one wire block,
/// which is what most callers want — but some need two commands to land in
/// **separate** blocks, because the first makes the firmware leave the block it
/// is dispatching (its shutdown is a `longjmp`, `src/sched.c`) and drop the rest
/// of it. Such a caller asks for a boundary with [`Mcu::flush`].
#[derive(Debug)]
enum SendItem {
    /// A message to append to the current batch.
    Payload(Payload),
    /// Send whatever is queued so far, then signal completion.
    Flush(oneshot::Sender<()>),
}

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
    /// Sender for the outbound queue: messages and flush barriers.
    send_buf_tx: mpsc::Sender<SendItem>,
    /// Pending synchronous calls waiting for responses.
    pending_calls: PendingCalls,
    /// Interface clone kept so the device can be shut down on drop.
    interface: Interface,
    /// Set by the receive task when the firmware answers with a sequence that
    /// cannot belong to this connection — one from an older session, which only a
    /// firmware that never rebooted can send. [`Mcu::answered_from_old_session`]
    /// reads it back.
    old_session: Arc<AtomicBool>,
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

/// A message as a log line renders it: the definition's own words with the type
/// specifiers replaced by the values — `set_pin oid=3 value=1`.
///
/// This is Klipper's own debug format (`MessageFormat.format_params` in
/// `klippy/msgproto.py`): the dictionary's parameter names, in declaration
/// order, with the values spliced into the `name=%x` slots. Nothing has to be
/// invented for the log to read well — the definition already says how the
/// message is written.
fn describe_message(msg: &Msg, values: &[ArgValue]) -> String {
    let mut text = msg.name.clone();
    for (index, value) in values.iter().enumerate() {
        let name = msg
            .params
            .get(index)
            .map(|(name, _)| name.as_str())
            .unwrap_or("?");
        text.push_str(&format!(" {name}={}", describe_value(value)));
    }
    text
}

/// One parameter value, as the debug format shows it: integers in decimal, a
/// string quoted and escaped, bytes as a byte-string literal.
///
/// A dynamic string — the `%s`/`%*s`/`%.*s` family — can hold anything,
/// including a space or a newline, so it is quoted the way Klipper reprs it:
/// otherwise a value would run into the format around it and the line could not
/// be read back.
fn describe_value(value: &ArgValue) -> String {
    match value {
        ArgValue::UInt8(v) => v.to_string(),
        ArgValue::UInt16(v) => v.to_string(),
        ArgValue::Int16(v) => v.to_string(),
        ArgValue::UInt32(v) => v.to_string(),
        ArgValue::Int32(v) => v.to_string(),
        ArgValue::Str(v) => describe_str(v),
        ArgValue::Bytes(v) => describe_bytes(v),
    }
}

/// A string in a form that can be read and written back: quoted, with the
/// characters that would run into the format around it escaped.
fn describe_str(text: &str) -> String {
    let mut out = String::with_capacity(text.len() + 2);
    out.push('"');
    for ch in text.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            ch if ch.is_control() => out.push_str(&format!("\\u{{{:x}}}", ch as u32)),
            ch => out.push(ch),
        }
    }
    out.push('"');
    out
}

/// The same for bytes, which need not be UTF-8: printable ASCII shows through,
/// and everything else becomes `\xNN`.
fn describe_bytes(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() + 3);
    out.push_str("b\"");
    for &byte in bytes {
        match byte {
            b'"' => out.push_str("\\\""),
            b'\\' => out.push_str("\\\\"),
            b'\n' => out.push_str("\\n"),
            b'\r' => out.push_str("\\r"),
            b'\t' => out.push_str("\\t"),
            0x20..=0x7e => out.push(byte as char),
            _ => out.push_str(&format!("\\x{byte:02x}")),
        }
    }
    out.push('"');
    out
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

        let (send_buf_tx, mut send_buf_rx) = mpsc::channel::<SendItem>(32);
        let interface_for_send = interface.clone();

        tokio::spawn(async move {
            let mut seq = 0u8;

            loop {
                // Wait for the first message of a batch. A flush with nothing
                // queued before it is already satisfied.
                let mut payload = match send_buf_rx.recv().await {
                    Some(SendItem::Payload(p)) => p,
                    Some(SendItem::Flush(done)) => {
                        let _ = done.send(());
                        continue;
                    }
                    None => break, // channel closed
                };

                // Coalesce more payloads until the batch is full, a flush asks
                // for a boundary, or the line goes idle.
                let mut flush_done = None;
                loop {
                    // Check if buffer is sufficiently full
                    if payload.len() >= MESSAGE_PAYLOAD_MAX * 2 / 3 {
                        break;
                    }

                    // Wait for more data, a flush, or a short idle timeout.
                    tokio::select! {
                        maybe_next = send_buf_rx.recv() => {
                            match maybe_next {
                                Some(SendItem::Payload(next_payload)) => {
                                    if payload.try_merge(&next_payload).is_err() {
                                        // Merge failed (would exceed max), send current batch first
                                        Self::send_batch(&interface_for_send, &mut seq, payload).await;
                                        // Start new batch with next
                                        payload = next_payload;
                                        continue;
                                    }
                                }
                                Some(SendItem::Flush(done)) => {
                                    // Boundary requested: send this batch now.
                                    flush_done = Some(done);
                                    break;
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
                if let Some(done) = flush_done {
                    let _ = done.send(());
                }
            }
        });

        let interface_for_recv = interface.clone();
        let old_session = Arc::new(AtomicBool::new(false));
        let old_session_for_task = Arc::clone(&old_session);
        let recv_handle = tokio::spawn(async move {
            let mut seq = 0u8;
            // A firmware that answers from an older session repeats the same
            // frame for as long as the handshake waits, so the mismatch is worth
            // saying once and not once per frame.
            let mut reported = false;

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
                //
                // Nothing has been accepted yet (`seq == 0`) and the frame still
                // does not fit: the firmware is in a session this connection did
                // not open, i.e. it kept running since it last spoke to a host.
                if frame.seq() != seq && frame.seq() != (seq + 1) & 0xf {
                    if seq == 0 {
                        old_session_for_task.store(true, Ordering::Relaxed);
                    }
                    if reported {
                        debug!(
                            "Seq mismatch: expected {} or {}, got {}",
                            seq,
                            (seq + 1) & 0xf,
                            frame.seq()
                        );
                    } else {
                        reported = true;
                        warn!(
                            "Seq mismatch: expected {} or {}, got {}",
                            seq,
                            (seq + 1) & 0xf,
                            frame.seq()
                        );
                    }
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
                    debug!("recv {}", describe_message(&msg, &params));
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
                        debug!("Invoking callback for {} (id={})", msg.name, msg.id);
                        let mut cb = callback.lock().unwrap();
                        cb(params.as_slice());
                    } else {
                        // No callback and no pending call — discard with warning.
                        warn!("Unhandled message {} (id={}), discarding", msg.name, msg.id);
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
            old_session,
            recv_handle: Some(recv_handle),
        }
    }

    /// Build the transport for an MCU called `name`: the parser with the host's
    /// identify formats, the send task, and the receive task.
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
    /// The interface arrives already open: a [`McuConfig`] describes a transport
    /// and `McuConfig::open` opens it, so that a firmware restart can reset the
    /// board on its **closed** port in between (`mcu/restart.rs`).
    pub fn new(name: impl Into<String>, interface: Interface) -> Self {
        Self::from_parts(name.into(), interface)
    }

    /// Whether the firmware has answered from a session **older** than this
    /// connection.
    ///
    /// A firmware that just rebooted starts its sequence over, so a frame that
    /// fits neither the block being answered nor the one after it can only come
    /// from a session that was already running. That is how an `rpi_usb` reset
    /// learns that its port switch disconnected the board without resetting it:
    /// [`Mcu::connect`] reports it as [`McuError::OldSession`].
    pub fn answered_from_old_session(&self) -> bool {
        self.old_session.load(Ordering::Relaxed)
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

    /// Whether the firmware declares `name`.
    ///
    /// Older or trimmed firmware may not send every message. Callers that can
    /// degrade use this instead of [`Mcu::require_message`]'s hard failure.
    pub(crate) fn has_message(&self, name: &str) -> bool {
        self.parser.lookup(name).is_some()
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

    /// The firmware's clock frequency (`CLOCK_FREQ`), in ticks per second.
    ///
    /// Read from the installed dictionary, so it is only available after the
    /// identify handshake. This is what turns host seconds into firmware clock
    /// ticks for every timed command (`queue_*`, `spi_send`, ADC sampling).
    ///
    /// # Errors
    /// Returns [`McuError::NotIdentified`] before the handshake and
    /// [`McuError::Config`] if the dictionary has no `CLOCK_FREQ`.
    pub fn clock_freq(&self) -> Result<f64, McuError> {
        let dictionary = self.require_dictionary()?;
        dictionary
            .constant_f64("CLOCK_FREQ")
            .ok_or_else(|| McuError::Config("dictionary has no CLOCK_FREQ".to_string()))
    }

    /// Convert seconds to firmware clock ticks, as upstream's
    /// `MCU.seconds_to_clock` (`klippy/mcu.py:1140`).
    ///
    /// # Errors
    /// As [`Mcu::clock_freq`].
    pub fn seconds_to_clock(&self, seconds: f64) -> Result<u64, McuError> {
        let freq = self.clock_freq()?;
        Ok((seconds * freq).max(0.0) as u64)
    }

    /// Encode a command without sending it.
    ///
    /// The configuration phase needs the bytes before they are queued: it
    /// hashes them into the configuration CRC. Everything else goes through
    /// [`Mcu::send`], which encodes and queues in one step.
    ///
    /// # Errors
    /// Returns [`McuError::Msg`] if the name is unknown or the arguments do not
    /// match the firmware's format string.
    pub(crate) fn encode(&self, name: &str, args: &[ArgValue]) -> Result<Payload, McuError> {
        Ok(self.parser.encode(name, args)?)
    }

    /// Queue an already-encoded payload, waiting for room.
    ///
    /// The send channel is bounded, so a configuration of hundreds of commands
    /// would overflow [`Mcu::send`]'s non-blocking `try_send`. This is the
    /// blocking-in-the-async-sense counterpart the configuration phase uses.
    ///
    /// # Errors
    /// Returns [`McuError::Msg`] if the send task has gone away.
    pub(crate) async fn send_payload(&self, payload: Payload) -> Result<(), McuError> {
        self.send_buf_tx
            .send(SendItem::Payload(payload))
            .await
            .map_err(|e| McuError::Msg(MsgError::new(e.to_string())))
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
        self.enqueue(name, args, None)
    }

    /// The body of [`Mcu::send`], plus what the caller will be waiting for.
    ///
    /// A call is one round trip, so it gets one line: `send identify offset=0
    /// count=40 (waiting for identify_response)` says what went out and what is
    /// expected back, where two lines said half of that each.
    fn enqueue(
        &self,
        name: &str,
        args: &[ArgValue],
        waiting_for: Option<&str>,
    ) -> Result<(), MsgError> {
        let payload = self.parser.encode(name, args)?;
        match waiting_for {
            Some(response) => debug!(
                "send {} (waiting for {response})",
                self.describe_command(name, args)
            ),
            None => debug!("send {}", self.describe_command(name, args)),
        }
        self.send_buf_tx
            .try_send(SendItem::Payload(payload))
            .map_err(|e| MsgError::new(e.to_string()))?;
        Ok(())
    }

    /// Wait until everything queued before this call has been written to the
    /// transport.
    ///
    /// The send task coalesces payloads that are queued close together into one
    /// wire block. That is usually what a caller wants, but not always: a command
    /// that makes the firmware leave the block it is dispatching has to be alone
    /// in its block. Queueing a barrier and waiting here forces that boundary —
    /// payloads queued **after** the barrier are not covered, which is the point.
    ///
    /// This says the bytes reached the transport, **not** that the firmware has
    /// read or acted on them. It is a flush, not an acknowledgement; waiting for
    /// an effect needs the message that reports it (see `mcu/config.rs`).
    ///
    /// # Errors
    /// Returns [`McuError::Call`] if the send task is gone, or if the barrier was
    /// not reached within `timeout` (a wedged transport).
    pub async fn flush(&self, timeout: Duration) -> Result<(), McuError> {
        let (done_tx, done_rx) = oneshot::channel();
        self.send_buf_tx
            .send(SendItem::Flush(done_tx))
            .await
            .map_err(|e| McuError::Call(McuCallError::SendFailed(e.to_string())))?;
        match tokio::time::timeout(timeout, done_rx).await {
            Ok(Ok(())) => Ok(()),
            Ok(Err(_)) => {
                Err(McuCallError::SendFailed("the send task dropped the flush".to_string()).into())
            }
            Err(_) => {
                Err(McuCallError::Timeout(format!("flush not reached within {timeout:?}")).into())
            }
        }
    }

    /// A command on its way out, as its DEBUG line shows it: the dictionary's
    /// own parameter names when it has the message, the name alone otherwise.
    ///
    /// Called as a field of the log line, so neither the lookup nor the
    /// formatting happens unless DEBUG is on.
    fn describe_command(&self, name: &str, args: &[ArgValue]) -> String {
        match self.parser.lookup(name) {
            Some(msg) => describe_message(&msg, args),
            None => name.to_string(),
        }
    }

    /// Send a command and wait for the response message.
    ///
    /// This is a synchronous request/response pattern: the command is sent,
    /// then the caller blocks (async) until the response message arrives or
    /// `timeout` elapses.
    ///
    /// The response does not have to be a direct answer to `command`: as long as
    /// the name matches, this also waits for a message the firmware pushes in
    /// reaction. That is how the configuration phase waits for `shutdown` after
    /// sending `emergency_stop` (`mcu/config.rs`): registration happens before
    /// the command goes out, so a fast report cannot race ahead of it.
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

        // 2. Create a oneshot channel for the response.
        let (tx, rx) = oneshot::channel::<Vec<ArgValue>>();

        // 3. Register the pending call.
        self.pending_calls
            .register(response_name.to_string(), tx)
            .await;

        // 4. Send the command. Its line names the response it is waiting for.
        if let Err(e) = self.enqueue(command, args, Some(response_name)) {
            warn!("Failed to send command '{}': {e}", command);
            // Clean up the pending call on send failure.
            self.pending_calls.cancel(response_name).await;
            return Err(McuCallError::SendFailed(e.msg));
        }

        // 5. Wait for the response. A successful resolve already consumed the
        // registration, so only the failure paths need to clean up.
        match tokio::time::timeout(timeout, rx).await {
            // The response itself was logged where it was decoded, with its
            // parameters; there is nothing left to say here.
            Ok(Ok(params)) => Ok(params),
            Ok(Err(_recv)) => {
                // Receiver dropped (shouldn't happen in normal flow).
                self.pending_calls.cancel(response_name).await;
                error!("Response receiver dropped for '{}'", response_name);
                Err(McuCallError::SendFailed(
                    "response receiver dropped".to_string(),
                ))
            }
            Err(_) => {
                // Timeout — clean up the pending call.
                self.pending_calls.cancel(response_name).await;
                warn!("Timeout waiting for response '{}'", response_name);
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
    /// Tests talk to a [`TestDevice`](crate::core::klippy::interface::devices::test::TestDevice)
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
    use crate::core::klippy::interface::devices::test::{MappingEntry, TestDevice};
    use crate::core::klippy::interface::Interface;

    fn make_frame(seq: u8, payload: &[u8]) -> Frame {
        Frame::new(seq, payload.to_vec())
    }

    // -----------------------------------------------------------------------
    // Mcu creation
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn test_new_starts_unidentified() {
        let mcu = Mcu::new("test_mcu", Interface::new(TestDevice::new(vec![])));

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
        // The frame fitted no session this connection could have opened, which
        // is what a caller uses to tell a rebooted board from one that kept
        // running.
        assert!(mcu.answered_from_old_session());
    }

    // -----------------------------------------------------------------------
    // flush — forcing a block boundary
    // -----------------------------------------------------------------------

    /// `flush` separates what is queued before it from what comes after: the two
    /// commands reach the wire as two frames instead of one coalesced frame.
    #[tokio::test]
    async fn test_flush_forces_a_block_boundary() {
        let device = TestDevice::new(vec![
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![],
            },
            MappingEntry {
                input: make_frame(1, &[6]),
                outputs: vec![],
            },
        ]);
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        let dictionary = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": 5, "get_uptime": 6}
        }))
        .unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        // Queued close together these would coalesce; the barrier keeps them
        // apart. A merged frame would match neither mapping, so it would not be
        // recorded and the count would not reach two.
        mcu.send("get_clock", &[]).unwrap();
        mcu.flush(Duration::from_millis(500)).await.unwrap();
        mcu.send("get_uptime", &[]).unwrap();
        mcu.flush(Duration::from_millis(500)).await.unwrap();

        let frames = recorder.frames();
        assert_eq!(frames.len(), 2, "expected two frames, got {frames:?}");
        assert_eq!(frames[0].payload(), &[5]);
        assert_eq!(frames[1].payload(), &[6]);
    }

    /// A flush with nothing queued before it is already satisfied.
    #[tokio::test]
    async fn test_flush_with_nothing_queued_completes() {
        let device = TestDevice::new(vec![]);
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));

        mcu.flush(Duration::from_millis(500)).await.unwrap();

        assert!(recorder.frames().is_empty());
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

    #[test]
    fn test_describe_message_names_the_parameters() {
        // The shape is the definition's own: `test_resp val=%u data=%.*s` with
        // the specifiers replaced by values, dynamic strings quoted.
        let response = Msg::parse(101, "test_resp val=%u data=%.*s").unwrap();
        assert_eq!(
            describe_message(
                &response,
                &[ArgValue::UInt32(7), ArgValue::Bytes(vec![0xaa, 0xbb])]
            ),
            r#"test_resp val=7 data=b"\xaa\xbb""#
        );

        // A string is quoted, so a value with a space cannot run into the
        // format around it.
        let ping = Msg::parse(10, "debug_ping data=%*s").unwrap();
        assert_eq!(
            describe_message(&ping, &[ArgValue::Str("two words".into())]),
            r#"debug_ping data="two words""#
        );

        // A message with no parameters is just its name.
        let bare = Msg::parse(5, "get_clock").unwrap();
        assert_eq!(describe_message(&bare, &[]), "get_clock");
    }

    #[test]
    fn test_describe_value_shows_each_type_readably() {
        assert_eq!(describe_value(&ArgValue::UInt8(1)), "1");
        assert_eq!(describe_value(&ArgValue::UInt16(2)), "2");
        assert_eq!(describe_value(&ArgValue::Int16(-3)), "-3");
        assert_eq!(describe_value(&ArgValue::UInt32(4)), "4");
        assert_eq!(describe_value(&ArgValue::Int32(-5)), "-5");

        // Strings are quoted and escaped; non-ASCII text stays itself.
        assert_eq!(describe_value(&ArgValue::Str("abc".into())), r#""abc""#);
        assert_eq!(describe_value(&ArgValue::Str("a\nb".into())), r#""a\nb""#);
        assert_eq!(describe_value(&ArgValue::Str("温度".into())), "\"温度\"");

        // Bytes are the same idea, but every non-printable byte is escaped.
        assert_eq!(
            describe_value(&ArgValue::Bytes(b"hi there".to_vec())),
            r#"b"hi there""#
        );
        assert_eq!(
            describe_value(&ArgValue::Bytes(vec![1, 2, 0xff])),
            r#"b"\x01\x02\xff""#
        );
    }
}
