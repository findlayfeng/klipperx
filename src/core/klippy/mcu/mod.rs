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
mod resource;
mod restart;
mod restart_method;

pub use config::{BuiltConfig, ConfigBuilder, ConfigCallback, Configured, PostInitCallback};
pub use dictionary::{Dictionary, Enumeration, MessageDef, OutputDef};
pub use error::{McuCallError, McuError};
pub use object::{load_config, load_config_prefix, McuObject};
pub use resource::{McuAdc, McuChip, McuDigitalOut, McuPwm};
pub use restart_method::McuRestartMethod;

use crate::core::klippy::frame::{Frame, MESSAGE_PAYLOAD_MAX};
use crate::core::klippy::identify;
use crate::core::klippy::interface::Interface;
use crate::core::klippy::mcu::pending::PendingCalls;
use crate::core::klippy::msg::error::MsgError;
use crate::core::klippy::msg::parser::Parser;
use crate::core::klippy::msg::proto::{ArgValue, Payload};
use crate::core::klippy::msg::Msg;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex as StdMutex};
use std::time::Instant;
use tokio::sync::{mpsc, oneshot, watch};
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
    /// The runtime this connection's transport tasks run on.
    ///
    /// Taken from the [`Interface`] at construction so the sender task, the
    /// receiver task and the device's own blocking I/O all share one runtime.
    /// It is the machine runtime; a future split from the API runtime (TODO A3)
    /// changes only where the interface picked it up.
    handle: tokio::runtime::Handle,
    /// The sequence state of this connection, shared with both of its tasks: the
    /// number the next block carries, and whether the connection had to take over a
    /// session that was already running (see [`Wire`]).
    wire: Arc<Wire>,
    /// A base point for estimating the firmware's free-running clock: the host
    /// instant the clock was read, paired with the reading. See
    /// [`Mcu::estimated_clock`]; `None` until something seeds it (the MCU
    /// object does, right after identify).
    clock_base: StdMutex<Option<(Instant, u64)>>,
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

/// How many blocks may be on the wire unanswered.
///
/// The bound upstream puts on its send window (`third_party/klipper/klippy/chelper/
/// serialqueue.c:109`, `MAX_PENDING_BLOCKS = 12`): an MCU that has stopped
/// answering has to back the host up rather than let the queue of unacknowledged
/// blocks grow without end.
const MAX_PENDING_BLOCKS: usize = 12;

/// The sequence state one connection shares with the firmware.
///
/// Klipper's firmware keeps one `next_sequence` (`third_party/klipper/src/command.c:16`):
/// every frame it sends is stamped with it, a block is accepted only when it
/// carries it, and accepting a block advances it **before** the block is
/// dispatched (`:301-305`). So the two ends share one number, and the firmware's
/// own frames are what say where it is: **the sequence of an empty (ack/nak) frame
/// is the number the next block has to carry** — the ack that follows block N
/// carries N+1, and the nak for a block the firmware would not take carries
/// whatever it is still waiting for.
///
/// The send task owns the number: it is the only writer to the wire. What comes
/// back is where the firmware is, and when that is not what this connection would
/// send next, the firmware is in a session this connection did not open — a board
/// that never rebooted — or a block went missing. Upstream reads the number the
/// same way, and adopts it for the same reason (`serialqueue.c:196-201`: "Got an
/// ack for a message not sent; must be connection init").
#[derive(Debug, Default)]
struct Wire {
    /// The sequence the **next** new block will carry: the firmware's
    /// `next_sequence`, as far as this connection knows.
    ///
    /// Unwrapped into a monotonic counter, because the 4-bit value on the wire
    /// only ever moves forward: a frame that looks behind is really ahead.
    next: AtomicU64,
    /// Set when the connection's first frame showed a firmware that was already
    /// mid-session, i.e. one nothing had reset.
    took_over: AtomicBool,
}

/// The send task's side of a connection.
///
/// It is the only writer to the wire, so the sequence and the blocks waiting to be
/// acknowledged live here and nowhere else.
struct Sender {
    /// The sequence state, shared with the receive side (see [`Wire`]).
    wire: Arc<Wire>,
    /// Blocks that went out and have not been answered, oldest first. Their
    /// payloads are what a nak — or a number from an earlier session — has to put
    /// back on the wire.
    in_flight: VecDeque<(u64, Frame)>,
    /// The last ack/nak this connection acted on, and the value a retransmit has
    /// already been done for: the firmware repeats its ack/nak for as long as it
    /// waits, and one retransmit per value is what upstream allows
    /// (`serialqueue.c:451-454`, `ignore_nak_seq`).
    acked: Option<u64>,
    retransmitted: Option<u64>,
}

impl Sender {
    fn new(wire: Arc<Wire>) -> Self {
        Self {
            wire,
            in_flight: VecDeque::new(),
            acked: None,
            retransmitted: None,
        }
    }

    /// Whether the window is full: the host has to wait for answers before it puts
    /// another block on the wire.
    fn is_full(&self) -> bool {
        self.in_flight.len() >= MAX_PENDING_BLOCKS
    }

    /// Put one block on the wire, carrying the sequence this connection is at, and
    /// remember it until the firmware answers it.
    async fn send_block(&mut self, interface: &Interface, payload: Vec<u8>) {
        // The counter is unwrapped here and only its low four bits go on the wire
        // (`Frame` keeps what it is given and masks when encoding), but the frame
        // this connection remembers has to carry those four bits too: that is what
        // a frame parsed off the wire carries, and the two are compared.
        let seq = self.wire.next.load(Ordering::Relaxed);
        let frame = Frame::new((seq & 0xf) as u8, payload);
        // Advance **before** writing. The firmware can answer while the write is
        // still in flight, and the receive side reads this number to tell an answer
        // to a block this connection sent from one that belongs to an earlier
        // session: with the number still one behind, a perfectly good response
        // would look like it answered a block that was never sent. Upstream orders
        // it the same way (`serialqueue.c`: `build_and_send_command` advances
        // `send_seq`, its caller writes the block).
        self.wire.next.store(seq + 1, Ordering::Relaxed);
        match interface.send(frame.clone()).await {
            Ok(()) => self.in_flight.push_back((seq, frame)),
            Err(e) => {
                // Nothing went out, so the number was not used after all.
                error!("Send failed (seq={}): {e}", seq & 0xf);
                self.wire.next.store(seq, Ordering::Relaxed);
            }
        }
    }

    /// Put one block back on the wire **as it was**, sequence and all.
    ///
    /// That is what the firmware's numbering asks for: it is waiting for exactly
    /// that block, and a block it had already taken would be answered with a nak
    /// instead of being run twice.
    async fn resend_block(&mut self, interface: &Interface, seq: u64, frame: Frame) {
        match interface.send(frame.clone()).await {
            Ok(()) => {
                debug!("Block {seq} sent again");
                self.in_flight.push_back((seq, frame));
            }
            Err(e) => error!("Retransmit failed (seq={}): {e}", frame.seq()),
        }
    }

    /// Take in what the firmware's counter has been seen at.
    ///
    /// `seen` is that number in this connection's unwrapped sequence. Everything
    /// below it is a block the firmware accepted. What is left is one of two
    /// things, told apart by which side of `next` the number is on:
    ///
    /// * **ahead of `next`** — the number comes from a session that was already
    ///   running; only the connection's first frame can be (see [`Wire`]). Adopt
    ///   it, and renumber and resend everything unacknowledged: the nak that
    ///   carried it is proof the firmware never ran those blocks, while a block it
    ///   did run would come back as a nak rather than run twice.
    /// * **not newer than the last one** — the same ack/nak again, i.e. the
    ///   firmware is still waiting for the block this connection sent and it never
    ///   arrived. Put the unacknowledged blocks back **as they are**: their
    ///   sequences are the ones being waited for.
    async fn settle(&mut self, interface: &Interface, seen: u64) {
        let next = self.wire.next.load(Ordering::Relaxed);
        if seen > next {
            // First, because the numbering is not shared yet: nothing the firmware
            // has said about its own counter can say anything about the blocks this
            // connection numbered by itself — in particular it must not be read as
            // "those blocks were accepted", or the block that has to go out again
            // would be dropped instead.
            debug!("Firmware is at {seen}, this connection would send {next}: taking it over");
            self.wire.next.store(seen, Ordering::Relaxed);
            self.acked = None;
            self.retransmitted = None;
            let again: Vec<(u64, Frame)> = self.in_flight.drain(..).collect();
            for (_, frame) in again {
                self.send_block(interface, frame.payload().to_vec()).await;
            }
            return;
        }

        // The numbering is shared, so the firmware's counter says which blocks it
        // took: everything below it.
        while self.in_flight.front().is_some_and(|(seq, _)| *seq < seen) {
            let (seq, _) = self.in_flight.pop_front().expect("checked just above");
            debug!("Block {seq} acknowledged");
        }

        match self.acked {
            Some(previous) if seen <= previous => {
                // The firmware saying the same thing again is a nak
                // (`serialqueue.c:291-293`): what it is waiting for never arrived.
                if self.retransmitted != Some(seen) {
                    self.retransmitted = Some(seen);
                    let again: Vec<(u64, Frame)> = self.in_flight.drain(..).collect();
                    for (seq, frame) in again {
                        self.resend_block(interface, seq, frame).await;
                    }
                }
            }
            _ => {
                self.acked = Some(seen);
                self.retransmitted = None;
            }
        }
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
        // Both transport tasks run on the interface's runtime, not the ambient
        // one. Cloned once and shared, since `Handle::spawn` only borrows it.
        let handle = interface.handle().clone();

        let wire = Arc::new(Wire::default());
        let (send_buf_tx, mut send_buf_rx) = mpsc::channel::<SendItem>(32);
        // Where the firmware's counter has been seen at, one value per ack/nak
        // frame (see `Wire`). A watch channel: only the newest value matters, the
        // receive task must never be held up by the send task, and every change
        // wakes it — including a repeated value, which is a nak (`Sender::settle`).
        let (acks_tx, mut acks_rx) = watch::channel(0u64);
        let interface_for_send = interface.clone();
        let wire_for_send = Arc::clone(&wire);

        handle.spawn(async move {
            let mut sender = Sender::new(Arc::clone(&wire_for_send));

            loop {
                // Wait for the first message of a batch — or for the firmware's
                // counter to move, which is what asks for a block to go out again.
                // A flush with nothing queued before it is already satisfied.
                let mut payload = tokio::select! {
                    item = send_buf_rx.recv() => match item {
                        Some(SendItem::Payload(p)) => p,
                        Some(SendItem::Flush(done)) => {
                            let _ = done.send(());
                            continue;
                        }
                        None => break, // channel closed
                    },
                    changed = acks_rx.changed() => {
                        if changed.is_err() {
                            break; // receive task gone: the device is shutting down
                        }
                        let seen = *acks_rx.borrow_and_update();
                        sender.settle(&interface_for_send, seen).await;
                        continue;
                    }
                };

                // Coalesce more payloads until the batch is full, a flush asks
                // for a boundary, or the line goes idle.
                let mut flush_done = None;
                loop {
                    // Check if buffer is sufficiently full
                    if payload.len() >= MESSAGE_PAYLOAD_MAX * 2 / 3 {
                        break;
                    }

                    // Wait for more data, a flush, an ack, or a short idle timeout.
                    tokio::select! {
                        maybe_next = send_buf_rx.recv() => {
                            match maybe_next {
                                Some(SendItem::Payload(next_payload)) => {
                                    if payload.try_merge(&next_payload).is_err() {
                                        // Merge failed (would exceed max), send current batch first
                                        sender
                                            .send_block(&interface_for_send, payload.into_raw())
                                            .await;
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
                        changed = acks_rx.changed() => {
                            if changed.is_err() {
                                break;
                            }
                            let seen = *acks_rx.borrow_and_update();
                            sender.settle(&interface_for_send, seen).await;
                        }
                        _ = sleep(Duration::from_millis(1)) => {
                            // Timeout, send accumulated batch
                            break;
                        }
                    }
                }

                // An MCU that has stopped answering has to back the host up
                // instead of growing the queue of unacknowledged blocks.
                while sender.is_full() {
                    if acks_rx.changed().await.is_err() {
                        break; // receive task gone: the device is shutting down
                    }
                    let seen = *acks_rx.borrow_and_update();
                    sender.settle(&interface_for_send, seen).await;
                }

                // send the batched payload to the MCU
                debug!("Sending batch: {} bytes", payload.len());
                sender
                    .send_block(&interface_for_send, payload.into_raw())
                    .await;
                if let Some(done) = flush_done {
                    let _ = done.send(());
                }
            }
        });

        let interface_for_recv = interface.clone();
        let wire_for_recv = Arc::clone(&wire);
        let recv_handle = handle.spawn(async move {
            // The firmware's counter in this connection's unwrapped numbering, and
            // how many frames have been accepted: the first frame is what says
            // whether the firmware was already running (`Wire::took_over`).
            let mut seen = 0u64;
            let mut frames = 0usize;

            loop {
                let frame = match interface_for_recv.receive().await {
                    Some(frame) => frame,
                    // Device shut down: no more frames will arrive.
                    None => break,
                };

                // The firmware stamps everything it sends with its one counter, and
                // that counter only ever moves forward (`src/command.c:16,208,301-305`):
                // map the 4-bit value onto this connection's unwrapped one, so a
                // frame that looks behind is read as ahead.
                let delta = (frame.seq().wrapping_sub(seen as u8) & 0xf) as u64;
                let rseq = seen + delta;
                if delta != 0 {
                    // A new number: it answers a block. The firmware's counter is
                    // the authority on which blocks those are, so a number past
                    // anything this connection sent comes from a session that came
                    // before it — which the *first* frame of a connection always is
                    // when the board never rebooted. That is how a running firmware
                    // is taken over (`serialqueue.c:196-201`); the send task adopts
                    // the number and puts what was not accepted back on the wire.
                    let next = wire_for_recv.next.load(Ordering::Relaxed);
                    if frames > 0 && rseq > next {
                        warn!(
                            "Frame with sequence {rseq} answers block {next} or later, which this \
                             connection never sent; dropping it"
                        );
                        continue;
                    }
                    seen = rseq;
                    if frames == 0 && rseq > 1 {
                        // A firmware that just booted answers this connection's
                        // first block with 0 or 1 (`src/command.c`, whose counter
                        // starts at 0). Anything else was already running when the
                        // port was opened — a board nothing reset.
                        wire_for_recv.took_over.store(true, Ordering::Relaxed);
                        info!(
                            "Firmware is still in an earlier session (sequence {rseq}); \
                             taking it over"
                        );
                    }
                    // A data frame is a response, and the ack that follows it (in
                    // the normal flow) carries the same number — so a takeover is
                    // the only thing about a data frame the send task has to know.
                    if rseq > next {
                        let _ = acks_tx.send(seen);
                    }
                }
                frames += 1;

                // An empty frame is the MCU's ack of the block it took, or its nak
                // of one it would not take. Either way its number is where the
                // firmware is — the same number its response carried — and the send
                // task is the one that acts on it: an ack drops what it proves was
                // accepted, a nak puts it back on the wire.
                if frame.payload().is_empty() {
                    debug!("Ack for block {}", frame.seq());
                    let _ = acks_tx.send(seen);
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
            handle,
            wire,
            clock_base: StdMutex::new(None),
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

    /// The runtime this connection's transport tasks run on.
    ///
    /// Taken from the [`Interface`] when it was opened, so the sender task, the
    /// receiver task and the device's own blocking I/O all share it. A caller
    /// that needs to schedule work alongside a connection uses this rather than
    /// the ambient runtime.
    pub fn handle(&self) -> &tokio::runtime::Handle {
        &self.handle
    }

    /// Whether this connection had to take over a session that was already
    /// running.
    ///
    /// A firmware that just booted answers the connection's first block with
    /// sequence 0 or 1, because its counter starts over (`src/command.c:16`).
    /// Anything else is a board that was already mid-session, and the transport
    /// adopts the number the firmware is waiting for and puts the unanswered
    /// blocks back on the wire (see `Wire`, the transport's shared sequence).
    ///
    /// That is the one thing a reconnect can observe to tell "the board rebooted"
    /// from "the board was only disconnected": an `rpi_usb` port switch that does
    /// not really switch power leaves the firmware running, and the caller that
    /// asked for the reset reads this to find out (`mcu/object.rs`).
    pub fn took_over_session(&self) -> bool {
        self.wire.took_over.load(Ordering::Relaxed)
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

    /// Record the firmware clock read at this moment, so [`Mcu::estimated_clock`]
    /// can extrapolate from it.
    ///
    /// This is the smallest useful piece of upstream's clock sync: one read at
    /// connect, then host time. It is enough to answer "what clock is it about
    /// now", which is what an unclocked resource needs for an immediate update
    /// and what a periodic query needs for a first sample time. It does not
    /// track drift, and there is no print time — that is the motion layer's
    /// (TODO C1).
    pub fn set_clock_base(&self, clock64: u64) {
        *self.clock_base.lock().expect("clock base lock poisoned") =
            Some((Instant::now(), clock64));
    }

    /// The firmware clock, extrapolated from the last [`Mcu::set_clock_base`].
    ///
    /// Returns `None` before a base has been recorded, or if the firmware
    /// frequency is unknown. The value is 64-bit; a clocked command carries its
    /// low word. Clocked commands do not advance it (this is not a scheduler), so
    /// two calls close together agree.
    pub fn estimated_clock(&self) -> Option<u64> {
        let (base_instant, base_clock) =
            (*self.clock_base.lock().expect("clock base lock poisoned"))?;
        let freq = self.clock_freq().ok()?;
        let elapsed = base_instant.elapsed().as_secs_f64() * freq;
        Some(base_clock + elapsed as u64)
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
    async fn test_a_running_firmware_is_taken_over() {
        let mut parser = Parser::new();
        parser.register(5, "get_clock").unwrap();
        parser.register(18, "clock clock=%u").unwrap();
        let mut answer = Payload::new();
        answer.push_i16(18).unwrap();
        answer.push_u32(1).unwrap();

        // A board that never rebooted is still at some number from the session
        // before this one, and naks the block it cannot place — with a frame
        // carrying that number (`src/command.c:331`, an empty ack/nak frame). The
        // transport has to adopt it, put the request back on the wire under it, and
        // carry on: the second mapping is the firmware accepting *that* block.
        let device = TestDevice::new(vec![
            MappingEntry {
                input: make_frame(0, &[5]),
                outputs: vec![make_frame(9, &[])],
            },
            MappingEntry {
                input: make_frame(9, &[5]),
                outputs: vec![make_frame(9, &answer.clone().into_raw())],
            },
        ]);
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        let dictionary = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": 5},
            "responses": {"clock clock=%u": 18}
        }))
        .unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        let params = mcu
            .call("get_clock", &[], "clock", Duration::from_millis(200))
            .await
            .expect("the request is answered once the session is taken over");
        assert_eq!(params.len(), 1);
        assert!(
            mcu.took_over_session(),
            "the firmware was mid-session: the caller has to be able to tell"
        );

        // The request went out twice: once at the number this connection started
        // with, and once at the firmware's own.
        let sent = recorder.frames();
        let seqs: Vec<u8> = sent.iter().map(|frame| frame.seq()).collect();
        assert_eq!(seqs, [0, 9], "the block is renumbered, not just repeated");
    }

    #[tokio::test]
    async fn test_a_fresh_firmware_needs_no_taking_over() {
        let mut parser = Parser::new();
        parser.register(5, "get_clock").unwrap();
        parser.register(18, "clock clock=%u").unwrap();
        let mut answer = Payload::new();
        answer.push_i16(18).unwrap();
        answer.push_u32(1).unwrap();

        // What a board that just booted does: it answers block 0 with 1 (the number
        // its counter moved to), and then acks it with the same number. Nothing is
        // sent a second time, and no takeover is reported.
        let device = TestDevice::new(vec![MappingEntry {
            input: make_frame(0, &[5]),
            outputs: vec![
                make_frame(1, &answer.clone().into_raw()),
                make_frame(1, &[]),
            ],
        }]);
        let recorder = device.recorder();
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        let dictionary = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": 5},
            "responses": {"clock clock=%u": 18}
        }))
        .unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        mcu.call("get_clock", &[], "clock", Duration::from_millis(200))
            .await
            .expect("the request is answered");

        assert!(!mcu.took_over_session());
        let sent = recorder.frames();
        assert_eq!(sent.len(), 1, "an ack is not a nak: nothing goes out again");
    }

    #[tokio::test]
    async fn test_a_frame_answering_a_block_we_never_sent_is_dropped() {
        let mut parser = Parser::new();
        parser.register(5, "get_clock").unwrap();
        parser.register(18, "clock clock=%u").unwrap();
        let mut answer = Payload::new();
        answer.push_i16(18).unwrap();
        answer.push_u32(1).unwrap();

        // The answer, and then a frame numbered past anything this connection sent.
        // Only the *first* frame of a connection may be that (it is a session to
        // take over); later ones are dropped, and an exchange that follows must not
        // be disturbed by them.
        let device = TestDevice::new(vec![MappingEntry {
            input: make_frame(0, &[5]),
            outputs: vec![
                make_frame(1, &answer.clone().into_raw()),
                make_frame(5, &[]),
            ],
        }]);
        let mcu = Mcu::for_test("test_mcu", Interface::new(device));
        let dictionary = Dictionary::from_json(serde_json::json!({
            "commands": {"get_clock": 5},
            "responses": {"clock clock=%u": 18}
        }))
        .unwrap();
        mcu.install_dictionary(dictionary).unwrap();

        mcu.call("get_clock", &[], "clock", Duration::from_millis(200))
            .await
            .expect("the request is answered");

        assert!(
            !mcu.took_over_session(),
            "a stray frame after the first one is not a session to take over"
        );
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
