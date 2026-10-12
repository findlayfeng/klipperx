//! A serial port that several MCUs share.
//!
//! One tty can carry more than one MCU once every unit on it is prefixed with
//! its sender's arbitration id ([`assign::arb_id`]). This device layer is the
//! host side of that: the port is opened **once**, a single reader thread cuts
//! the byte stream into whole units and hands each one to the MCU its header
//! names, and each MCU gets a handle that a [`Mcu`] can drive like any other
//! [`Device`].
//!
//! The split from the single-owner [`SerialDevice`](super::serial::SerialDevice)
//! is deliberate: here the port must outlive any one handle (the last one to be
//! dropped closes it, and only then), the reader must not hold the port open on
//! its own, and writes from different MCUs must not interleave inside a unit.
//!
//! # The frame header
//!
//! Every unit starts with a bare Klipper VLQ of the arbitration id. Over the
//! whole assigned range (`0x102..=0x2ff`, ids `1..=255` in both directions) the
//! header is always exactly two bytes. [`encode_header`] / [`decode_header`]
//! are the codec; the VLQ itself is the message system's own
//! ([`crate::core::klippy::msg::proto`]), not a copy, and the id mapping is
//! [`crate::core::klippy::assign`]'s.
//!
//! A VLQ of an id may end in `0x7e` (`mcu_id = 63, 127, 191, 255` all do). That
//! is not a problem: frame *content* is allowed to contain SYNC — upstream,
//! `docs/Protocol.md:178-183`: “a sync character is not exclusive to the
//! framing and may be present in the message block content” — so a SYNC inside
//! the header is ordinary data.
//!
//! # Attaching
//!
//! `attach` performs the frame-level id handshake synchronously: it writes the
//! assignment unit ([`assignment_request`]) and waits for the device's answer
//! ([`assignment_reply`]) before returning a handle. Attaches on the same path
//! are serialized, so two MCUs coming up together cannot interleave their
//! handshakes.
//!
//! # The assignment pseudo-frame
//!
//! Serial has no side-channel beside the byte stream, so this module defines one
//! *inside* the frame format: a frame whose sequence byte carries no
//! `MESSAGE_DEST` bit. The id travels once, as the unit's header; the
//! pseudo-frame body is fixed and identical in both directions:
//!
//! ```text
//! [vlq(arb)][len=9][0x20]["K" "X" "I" "D"][crc16 hi][crc16 lo][0x7e]
//!  \______/  \____________________________________________________/
//!   the id    a well-formed frame except for the sequence byte
//! ```
//!
//! * `len` is in `[MESSAGE_MIN, MESSAGE_MAX]`, the CRC is the ordinary
//!   CRC-16-CCITT over `[len, seq, payload]`. Everything except the sequence
//!   byte is a well-formed frame — so the frame is self-delimiting (its own
//!   trailing `0x7e`) and the payload could be validated if anyone chose to.
//! * `seq = 0x20`: `(0x20 & !MESSAGE_SEQ_MASK) != MESSAGE_DEST`
//!   (`0x20 != 0x10`). The sequence byte is the only field that differs, and
//!   `0x20` is a bit neither direction's ordinary traffic uses.
//! * the payload is the fixed marker `KXID` ([`MAGIC`]) and nothing else.
//!
//! [`assignment_request`] addresses a device on its listener id,
//! [`assignment_reply`] is the answer it sends on its reply id, and
//! [`parse_assignment_ack`] recognizes that answer.
//!
//! ## What old firmware does with it
//!
//! `command_find_block` (`third_party/klipper/src/command.c:276`) checks, in
//! order: length range (`:284`), **sequence byte (`:287`)**, SYNC, CRC, then
//! the sequence number. Our frame passes the length check and bails at the
//! sequence-byte check — *before* the payload is ever read, before the CRC is
//! ever computed, before dispatch could run. The error path (`:311-332`)
//! discards bytes up to and including the next `0x7e`, which is our own
//! trailer: exactly this frame is consumed, nothing behind it. Consequences,
//! all of them by construction:
//!
//! * no dispatch → no `shutdown("Invalid command")`, no command executed;
//! * `next_sequence` untouched → the identify exchange that follows starts
//!   exactly where it would have;
//! * at most **one** empty ack-shaped frame in reply (`need_sync` sends a nak
//!   only when `CF_NEED_VALID` was clear — `:327-331`) — an empty frame stamped
//!   with the firmware's current `next_sequence`, which the receive task reads
//!   as the protocol-consistent no-progress notice it is. Every later
//!   assignment frame is discarded **silently** (the flag stays set until a
//!   valid block clears it).
//!
//! So a capable device and an old one are told apart by what comes back: an
//! answer shaped the way only a parser of this pattern could produce (below),
//! or nothing / an ordinary empty frame. Both old-firmware outcomes are
//! harmless; neither advances anything. It is also what makes the handshake
//! safe to attempt: a shared serial port misconfigured onto an old firmware
//! discards the pseudo-frame and never answers, so the handshake **times out
//! cleanly** instead of disturbing the session already running on the wire.
//!
//! ## The answer
//!
//! A capable firmware replies with the same pseudo-frame in the opposite
//! direction, addressed back on its own arbitration id: [`assignment_reply`]'s
//! header, then the identical body. Old firmware can never emit it: every frame
//! it sends is built by `command_add_frame` (`command.c:207-212`), whose
//! sequence byte is `next_sequence`, always in `0x10..=0x1f` (`:16`, `:305`).
//! [`parse_assignment_ack`] recognizes the reply — the same check as the
//! request, opposite direction — and returns the id it names.
//!
//! # Scope
//!
//! * The handshake is a **synchronous** request/reply run by `attach` while a
//!   **shared** serial port is being assigned. An **exclusive** serial port
//!   sends nothing, and no transport sends the pattern fire-and-forget.
//! * `host_library` and the `test:` simulator do not take part; CAN keeps its
//!   own admin frame (`0x3f0` / `CMD_SET_NODEID`).
//! * The arbitration-id mapping these headers are built from is
//!   [`crate::core::klippy::assign`]'s, and only that.
//!
//! [`Mcu`]: crate::core::klippy::mcu::Mcu

use crate::core::klippy::assign;
use crate::core::klippy::config::mcu::MAX_MCU_ID;
use crate::core::klippy::frame::{
    Frame, FrameStream, MESSAGE_HEADER_SIZE, MESSAGE_MAX, MESSAGE_MIN, MESSAGE_POS_LEN,
    MESSAGE_POS_SEQ, MESSAGE_SYNC, MESSAGE_TRAILER_SIZE, MESSAGE_TRAILER_SYNC,
};
use crate::core::klippy::interface::error::InterfaceError;
use crate::core::klippy::interface::Device;
use crate::core::klippy::msg::proto::{vlq_decode, vlq_encode, VlqDecode};
use std::collections::HashMap;
use std::fs::{File, OpenOptions};
use std::io::{ErrorKind, Read, Write};
use std::os::unix::fs::OpenOptionsExt;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, OnceLock, Weak};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};
use tracing::{debug, info, warn};

use super::serial::{clear_nonblocking, set_raw_mode};

/// How long one attachment attempt waits for the device's answer.
pub const ASSIGN_ATTACH_TIMEOUT: Duration = Duration::from_millis(500);

/// How many times `attach` re-sends the request before giving up.
///
/// A lost or ignored frame must not cost the whole session: each retry re-sends
/// the request and waits another `ASSIGN_ATTACH_TIMEOUT`.
pub const ASSIGN_ATTACH_ATTEMPTS: u32 = 3;

/// How many bytes one `read` may bring in. Bounds the reader's stack buffer;
/// larger reads simply take another turn.
const READ_CHUNK: usize = 1024;

// ===========================================================================
// The frame header and the assignment pseudo-frame
// ===========================================================================

/// The sequence byte the assignment pattern uses, as a whole byte: bit 5 set,
/// the bits `MESSAGE_DEST` owns clear, the low nibble free.
///
/// Legitimate frames — in either direction — carry `0x10..=0x1f`: bit 4 set by
/// `MESSAGE_DEST`, bits 5-7 zero. `0x20` occupies a bit nothing else uses, so
/// neither direction's ordinary traffic can produce it by accident, and the
/// low nibble stays available for a sequence number should one matter.
pub const ASSIGN_SEQ_BYTE: u8 = 0x20;

/// The payload marker: the fixed bytes every device built to parse this pattern
/// looks for, and — once written out — the whole body of the pseudo-frame.
///
/// `0x7e` never appears in it, which keeps the frame's header and payload free
/// of an interior SYNC byte (see the module docs: the discard path stops at the
/// first `0x7e`). The two CRC bytes are computed, not chosen; for this fixed
/// body they are `0x4b`/`0x4f`, so the trailer's `0x7e` is the only SYNC in the
/// frame.
pub const MAGIC: &[u8; 4] = b"KXID";

/// Encode a frame header: the arbitration id as a bare Klipper VLQ.
///
/// Over the assigned range (`0x102..=0x2ff`) this is always two bytes.
pub fn encode_header(arb: u32) -> Vec<u8> {
    let mut out = Vec::with_capacity(2);
    vlq_encode(arb, &mut out);
    out
}

/// The outcome of reading a frame header off the front of a byte slice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeaderScan {
    /// A whole arbitration id, occupying `consumed` bytes.
    Complete { arb: u32, consumed: usize },
    /// The slice ends inside the id; more bytes could complete it.
    NeedMore,
    /// The bytes cannot be a `u32` VLQ (more than five continuation bytes).
    Invalid,
}

/// Decode the frame header at the front of `buf`.
pub fn decode_header(buf: &[u8]) -> HeaderScan {
    match vlq_decode(buf) {
        VlqDecode::Complete { value, consumed } => HeaderScan::Complete {
            arb: value,
            consumed,
        },
        VlqDecode::NeedMore => HeaderScan::NeedMore,
        VlqDecode::Invalid => HeaderScan::Invalid,
    }
}

/// The pseudo-frame an assignment unit carries after its header: the `KXID`
/// pattern with a payload of [`MAGIC`] alone.
///
/// The id is *not* in here — it is the unit's header — so the same body goes in
/// both directions, and the frame is
/// `[len=9][0x20]["KXID"][crc16 hi][crc16 lo][0x7e]`.
fn assignment_frame() -> Vec<u8> {
    Frame::with_wire_seq(ASSIGN_SEQ_BYTE, MAGIC.to_vec()).raw_bytes()
}

/// The assignment unit addressed to `mcu_id`: the header on its listener
/// arbitration id, then the pseudo-frame.
pub fn assignment_request(mcu_id: u32) -> Option<Vec<u8>> {
    let mut unit = encode_header(assign::arb_id(mcu_id)?);
    unit.extend_from_slice(&assignment_frame());
    Some(unit)
}

/// The answer `mcu_id` sends for [`assignment_request`]: the header on its
/// answer arbitration id, then the same pseudo-frame.
pub fn assignment_reply(mcu_id: u32) -> Option<Vec<u8>> {
    let mut unit = encode_header(assign::reply_arb_id(mcu_id)?);
    unit.extend_from_slice(&assignment_frame());
    Some(unit)
}

/// Recognize an assignment answer and return the id it names.
///
/// Everything must be self-consistent: a whole VLQ header whose arbitration id
/// is the reply direction ([`is_reply_arb`](assign::is_reply_arb)) and decodes
/// to an [`mcu_id`](assign::mcu_id_from_arb), then a pseudo-frame whose length
/// byte matches the bytes present, whose sequence byte is [`ASSIGN_SEQ_BYTE`],
/// whose trailer and CRC check out, and whose payload is exactly [`MAGIC`]. A
/// request (even header) is rejected by the direction check.
pub fn parse_assignment_ack(unit: &[u8]) -> Option<u32> {
    let (arb, consumed) = match decode_header(unit) {
        HeaderScan::Complete { arb, consumed } => (arb, consumed),
        HeaderScan::NeedMore | HeaderScan::Invalid => return None,
    };
    if !assign::is_reply_arb(arb) {
        return None;
    }
    let mcu_id = assign::mcu_id_from_arb(arb)?;
    let frame = &unit[consumed..];
    if frame.len() < MESSAGE_MIN {
        return None;
    }
    let len = frame[MESSAGE_POS_LEN] as usize;
    if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&len) || frame.len() != len {
        return None;
    }
    if frame[MESSAGE_POS_SEQ] != ASSIGN_SEQ_BYTE {
        return None;
    }
    if frame[len - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC {
        return None;
    }
    let expected = crate::crc16_ccitt!(&frame[..len - MESSAGE_TRAILER_SIZE]);
    let actual = ((frame[len - 3] as u16) << 8) | (frame[len - 2] as u16);
    if expected != actual {
        return None;
    }
    let payload = &frame[MESSAGE_HEADER_SIZE..len - MESSAGE_TRAILER_SIZE];
    if payload != &MAGIC[..] {
        return None;
    }
    Some(mcu_id)
}

/// A physical serial port, its one reader thread, and a slot per attached MCU.
pub struct SharedSerialPort {
    path: PathBuf,
    baud: u32,
    /// The opened tty. Shared with the reader thread so it can read without
    /// holding an `Arc<SharedSerialPort>` (which would keep the port from ever
    /// being dropped — see the reader).
    port: Arc<File>,
    /// One slot per attached MCU, keyed by the MCU id.
    slots: Mutex<HashMap<u32, Arc<McuSlot>>>,
    /// Serializes the handshakes on this port: two MCUs attaching together must
    /// not interleave their requests and answers.
    attach_lock: Mutex<()>,
    /// Serializes whole units on the write side (`send` and the handshake
    /// request): a unit is a header plus a frame, and the two must not be split
    /// by another writer.
    write_lock: Mutex<()>,
    stopped: AtomicBool,
    reader: Mutex<Option<JoinHandle<()>>>,
}

/// One attached MCU's per-port state.
struct McuSlot {
    /// Frame bytes addressed to this MCU, reassembled into whole frames.
    incoming: Mutex<FrameStream>,
    /// Notified whenever `incoming` gains bytes.
    cv: Condvar,
    /// The id an attach handshake is waiting for, filled by the reader when the
    /// device's answer to [`assignment_reply`] is recognized.
    ack: Mutex<Option<u32>>,
    /// Notified whenever `ack` is set.
    ack_cv: Condvar,
}

impl McuSlot {
    fn new() -> Self {
        Self {
            incoming: Mutex::new(FrameStream::new()),
            cv: Condvar::new(),
            ack: Mutex::new(None),
            ack_cv: Condvar::new(),
        }
    }
}

/// The handle one MCU holds on a [`SharedSerialPort`], implementing [`Device`].
///
/// `shutdown` stops only this handle: the port — and every other MCU on it —
/// stays up until the last handle is gone.
pub struct SharedSerialHandle {
    port: Arc<SharedSerialPort>,
    mcu_id: u32,
    slot: Arc<McuSlot>,
    stopped: AtomicBool,
}

/// The process-wide registry that makes a port a per-path singleton: attaches
/// on the same path share one port (and thus one reader).
fn ports() -> &'static Mutex<HashMap<PathBuf, Weak<SharedSerialPort>>> {
    static PORTS: OnceLock<Mutex<HashMap<PathBuf, Weak<SharedSerialPort>>>> = OnceLock::new();
    PORTS.get_or_init(|| Mutex::new(HashMap::new()))
}

/// The live port for `path`, opening it (and starting its reader) if the
/// registry holds no live one.
fn port_for(path: &Path, baud: u32) -> Result<Arc<SharedSerialPort>, InterfaceError> {
    let mut registry = ports().lock().unwrap();
    if let Some(existing) = registry.get(path).and_then(Weak::upgrade) {
        return Ok(existing);
    }
    // A dead weak entry (its port was dropped) is replaced by the fresh one.
    let port = SharedSerialPort::open(path, baud)?;
    registry.insert(path.to_path_buf(), Arc::downgrade(&port));
    Ok(port)
}

/// Open a port for one MCU, completing the frame-level id handshake.
///
/// The first attach on a path opens the tty and starts the reader; later ones
/// reuse both and only register a slot. The request is written and the answer
/// awaited `ASSIGN_ATTACH_ATTEMPTS` times, each for `timeout`; on exhaustion the
/// slot is unregistered and [`InterfaceError::Timeout`] returned.
///
/// # Errors
/// [`InterfaceError::Other`] for an id the wire format cannot carry or a port
/// that cannot be opened, [`InterfaceError::ConnectionLost`] if the port dies
/// mid-handshake, [`InterfaceError::Timeout`] if no answer arrives.
pub fn attach(
    path: &Path,
    baud: u32,
    mcu_id: u32,
    timeout: Duration,
) -> Result<Arc<SharedSerialHandle>, InterfaceError> {
    if assign::arb_id(mcu_id).is_none() {
        return Err(InterfaceError::Other(format!(
            "mcu id {mcu_id} is outside 1..={MAX_MCU_ID}"
        )));
    }

    let port = port_for(path, baud)?;
    let _attaching = port.attach_lock.lock().unwrap();
    if port.stopped.load(Ordering::SeqCst) {
        return Err(InterfaceError::ConnectionLost);
    }

    let slot = Arc::new(McuSlot::new());
    port.slots.lock().unwrap().insert(mcu_id, Arc::clone(&slot));

    let request = assignment_request(mcu_id).expect("arb_id checked above");
    let mut attempts = 0;
    loop {
        *slot.ack.lock().unwrap() = None;
        if let Err(e) = port.write_unit(&request) {
            port.slots.lock().unwrap().remove(&mcu_id);
            return Err(e);
        }
        if wait_for_ack(&slot, timeout) == Some(mcu_id) {
            // The handshake is done: release the port for the next attach, then
            // hand this one its handle.
            drop(_attaching);
            return Ok(Arc::new(SharedSerialHandle {
                port,
                mcu_id,
                slot,
                stopped: AtomicBool::new(false),
            }));
        }
        if port.stopped.load(Ordering::SeqCst) {
            port.slots.lock().unwrap().remove(&mcu_id);
            return Err(InterfaceError::ConnectionLost);
        }
        attempts += 1;
        if attempts >= ASSIGN_ATTACH_ATTEMPTS {
            break;
        }
    }

    port.slots.lock().unwrap().remove(&mcu_id);
    Err(InterfaceError::Timeout)
}

/// Wait up to `timeout` for the reader to record an assignment answer.
fn wait_for_ack(slot: &McuSlot, timeout: Duration) -> Option<u32> {
    let deadline = Instant::now() + timeout;
    let mut ack = slot.ack.lock().unwrap();
    loop {
        if let Some(id) = *ack {
            return Some(id);
        }
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return None;
        }
        let (guard, result) = slot.ack_cv.wait_timeout(ack, remaining).unwrap();
        ack = guard;
        // A timeout can still have raced the answer in; the loop re-checks.
        if result.timed_out() && ack.is_none() {
            return None;
        }
    }
}

impl SharedSerialPort {
    /// The port this was opened on.
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// The configured line speed.
    pub fn baud(&self) -> u32 {
        self.baud
    }

    /// Open the tty, put it into raw mode, and start the reader.
    fn open(path: &Path, baud: u32) -> Result<Arc<Self>, InterfaceError> {
        let port = OpenOptions::new()
            .read(true)
            .write(true)
            // As `SerialDevice::open`: no controlling terminal, and no blocking
            // inside `open` on a port without carrier detect.
            .custom_flags(libc::O_NOCTTY | libc::O_NONBLOCK)
            .open(path)
            .map_err(|e| {
                InterfaceError::Other(format!(
                    "failed to open serial port {}: {e}",
                    path.display()
                ))
            })?;

        set_raw_mode(&port, baud)?;
        clear_nonblocking(&port)?;

        info!("shared serial port {} open at {baud} baud", path.display());
        let shared = Arc::new(Self {
            path: path.to_path_buf(),
            baud,
            port: Arc::new(port),
            slots: Mutex::new(HashMap::new()),
            attach_lock: Mutex::new(()),
            write_lock: Mutex::new(()),
            stopped: AtomicBool::new(false),
            reader: Mutex::new(None),
        });
        shared.start_reader();
        Ok(shared)
    }

    /// Spawn the port's one reader thread.
    ///
    /// The thread holds only a [`Weak`] to the port, plus a clone of the file
    /// handle it reads: holding an `Arc<SharedSerialPort>` would keep the port
    /// alive forever, and no last handle could ever close it.
    fn start_reader(self: &Arc<Self>) {
        let weak = Arc::downgrade(self);
        let file = Arc::clone(&self.port);
        let handle = std::thread::spawn(move || reader_loop(weak, file));
        *self.reader.lock().unwrap() = Some(handle);
    }

    /// Mark the port stopped and wake everything waiting on it.
    fn stop(&self) {
        self.stopped.store(true, Ordering::SeqCst);
        if let Ok(slots) = self.slots.lock() {
            for slot in slots.values() {
                slot.cv.notify_all();
                slot.ack_cv.notify_all();
            }
        }
    }

    /// Write one whole unit under the write lock, so no other writer can split
    /// its header from its frame.
    fn write_unit(&self, unit: &[u8]) -> Result<(), InterfaceError> {
        let _writing = self.write_lock.lock().unwrap();
        (&*self.port).write_all(unit).map_err(|e| {
            InterfaceError::SendError(format!(
                "serial write to {} failed: {e}",
                self.path.display()
            ))
        })
    }
}

impl Drop for SharedSerialPort {
    fn drop(&mut self) {
        self.stop();
        // The reader may still be inside a `read` (bounded by `VTIME`); joining
        // it here is what lets the port and its descriptor go away. It does not
        // need the mutex to finish — it sees `stopped` and returns.
        if let Ok(mut guard) = self.reader.lock() {
            if let Some(handle) = guard.take() {
                let _ = handle.join();
            }
        }
    }
}

/// Read the port and dispatch whole units until stopped.
fn reader_loop(weak: Weak<SharedSerialPort>, file: Arc<File>) {
    let mut buf = [0u8; READ_CHUNK];
    // Partial unit carried across reads, and whether the stream is
    // desynchronised (bytes must be scanned for a SYNC before framing resumes).
    let mut pending: Vec<u8> = Vec::new();
    let mut needs_sync = false;

    loop {
        match (&*file).read(&mut buf) {
            Ok(0) => {} // the `VTIME` window elapsed with nothing waiting
            Ok(read) => {
                pending.extend_from_slice(&buf[..read]);
                let Some(port) = weak.upgrade() else { return };
                dispatch(&port, &mut pending, &mut needs_sync);
                // Drop the strong reference before blocking in `read` again, or
                // the last handle could never bring the port down.
                drop(port);
            }
            Err(e) if e.kind() == ErrorKind::Interrupted => continue,
            Err(e) => {
                if let Some(port) = weak.upgrade() {
                    warn!(
                        "shared serial read from {} failed: {e}",
                        port.path.display()
                    );
                    port.stop();
                }
                return;
            }
        }

        // Between reads: notice a shutdown.
        match weak.upgrade() {
            Some(port) if port.stopped.load(Ordering::SeqCst) => return,
            Some(_) => {}
            None => return,
        }
    }
}

/// Cut every whole unit out of `pending` and deliver it.
///
/// A unit is a bare VLQ header ([`decode_header`]) followed by one frame
/// (length byte + that many bytes). Anything that cannot be one — an invalid
/// header, an arbitration id outside the assigned range, or a length byte
/// outside `MESSAGE_MIN..=MESSAGE_MAX` — resynchronises on the next SYNC, the
/// same policy as [`FrameStream`]. Too few bytes for a unit just waits for the
/// next read.
fn dispatch(port: &SharedSerialPort, pending: &mut Vec<u8>, needs_sync: &mut bool) {
    loop {
        if *needs_sync {
            match pending.iter().position(|&b| b == MESSAGE_SYNC) {
                Some(pos) => {
                    pending.drain(..=pos);
                    *needs_sync = false;
                }
                None => {
                    pending.clear();
                    return;
                }
            }
        }

        let (arb, consumed) = match decode_header(pending) {
            HeaderScan::NeedMore => return,
            HeaderScan::Invalid => {
                drop_to_sync(pending, needs_sync);
                continue;
            }
            HeaderScan::Complete { arb, consumed } => (arb, consumed),
        };

        // Not an arbitration id this port routes (neither an MCU's listener id
        // nor an answer id): no unit can start here.
        if assign::mcu_id_from_arb(arb).is_none() {
            drop_to_sync(pending, needs_sync);
            continue;
        }

        if pending.len() < consumed + 1 {
            return; // the length byte is still in flight
        }
        let length = pending[consumed] as usize;
        if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&length) {
            drop_to_sync(pending, needs_sync);
            continue;
        }

        let unit_len = consumed + length;
        if pending.len() < unit_len {
            return; // the rest of the frame is still in flight
        }

        let unit = &pending[..unit_len];
        if let Some(mcu_id) = parse_assignment_ack(unit) {
            // The answer to a handshake: it goes to the ack slot, not to
            // `incoming`, where it would look like a malformed frame.
            route_ack(port, mcu_id);
        } else {
            let mcu_id = assign::mcu_id_from_arb(arb).expect("checked above");
            push_frame(port, mcu_id, &pending[consumed..unit_len]);
        }
        pending.drain(..unit_len);
    }
}

/// Discard up to and including the next SYNC; if there is none, drop it all and
/// stay desynchronised (the [`FrameStream`] policy, `frame.rs`).
fn drop_to_sync(pending: &mut Vec<u8>, needs_sync: &mut bool) {
    match pending.iter().position(|&b| b == MESSAGE_SYNC) {
        Some(pos) => {
            pending.drain(..=pos);
            *needs_sync = false;
        }
        None => {
            pending.clear();
            *needs_sync = true;
        }
    }
}

/// Deliver one frame's bytes to an MCU's reassembly stream and wake its reader.
fn push_frame(port: &SharedSerialPort, mcu_id: u32, frame: &[u8]) {
    let slot = port.slots.lock().unwrap().get(&mcu_id).cloned();
    match slot {
        Some(slot) => {
            let mut incoming = slot.incoming.lock().unwrap();
            incoming.push(frame);
            slot.cv.notify_all();
        }
        None => debug!(
            "shared serial {}: no slot for mcu {mcu_id}; dropping frame",
            port.path.display()
        ),
    }
}

/// Record a handshake answer on its MCU's slot and wake a waiting `attach`.
fn route_ack(port: &SharedSerialPort, mcu_id: u32) {
    let slot = port.slots.lock().unwrap().get(&mcu_id).cloned();
    match slot {
        Some(slot) => {
            *slot.ack.lock().unwrap() = Some(mcu_id);
            slot.ack_cv.notify_all();
        }
        None => debug!(
            "shared serial {}: assignment answer for unregistered mcu {mcu_id}",
            port.path.display()
        ),
    }
}

impl std::fmt::Debug for SharedSerialHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SharedSerialHandle")
            .field("path", &self.port.path)
            .field("mcu_id", &self.mcu_id)
            .field("stopped", &self.stopped.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Device for SharedSerialHandle {
    fn send(&self, frame: &Frame) -> Result<(), InterfaceError> {
        if self.stopped.load(Ordering::SeqCst) || self.port.stopped.load(Ordering::SeqCst) {
            return Err(InterfaceError::ConnectionLost);
        }
        let arb = assign::arb_id(self.mcu_id).expect("an attached mcu id is always addressable");
        let mut unit = encode_header(arb);
        unit.extend_from_slice(&frame.raw_bytes());
        self.port.write_unit(&unit)
    }

    fn receive(&self) -> Option<Frame> {
        let mut incoming = self.slot.incoming.lock().unwrap();
        loop {
            if let Some(frame) = incoming.next_frame() {
                return Some(frame);
            }
            if self.stopped.load(Ordering::SeqCst) || self.port.stopped.load(Ordering::SeqCst) {
                return None;
            }
            // `wait` releases the mutex, so the reader can push; it re-locks
            // before notifying, which is what keeps `next_frame` from racing
            // the wake-up.
            incoming = self.slot.cv.wait(incoming).unwrap();
        }
    }

    fn shutdown(&self) {
        // Only this handle: the port stays up for every other MCU on it.
        self.stopped.store(true, Ordering::SeqCst);
        let _guard = self.slot.incoming.lock().unwrap();
        self.slot.cv.notify_all();
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::frame::{MESSAGE_DEST, MESSAGE_SEQ_MASK};
    use crate::core::klippy::interface::devices::serial::DEFAULT_BAUD;
    use crate::core::klippy::interface::pty::Pty;
    use std::thread;

    /// Read exactly `want` bytes from the pty master (blocking, no polling).
    fn read_n(master: &File, want: usize) -> Vec<u8> {
        let mut master = master;
        let mut out = Vec::with_capacity(want);
        let mut buf = [0u8; 64];
        while out.len() < want {
            let read = master.read(&mut buf).expect("pty master read");
            assert_ne!(read, 0, "the pty closed before {want} bytes arrived");
            out.extend_from_slice(&buf[..read]);
        }
        out
    }

    fn master_write(master: &File, bytes: &[u8]) {
        let mut master = master;
        master.write_all(bytes).expect("pty master write");
    }

    /// Attach `mcu_id`, playing the device side of the handshake from the test.
    fn attach_with_reply(pty: &Pty, mcu_id: u32) -> Arc<SharedSerialHandle> {
        let request = assignment_request(mcu_id).expect("a valid mcu id");
        let path = pty.path.clone();
        let thread =
            thread::spawn(move || attach(&path, DEFAULT_BAUD, mcu_id, ASSIGN_ATTACH_TIMEOUT));
        let seen = read_n(&pty.master, request.len());
        assert_eq!(
            seen, request,
            "attach must send the assignment request verbatim"
        );
        master_write(
            &pty.master,
            &assignment_reply(mcu_id).expect("a valid mcu id"),
        );
        thread
            .join()
            .expect("attach thread")
            .expect("attach succeeds")
    }

    /// One device→host unit: the answer-direction header, then the frame bytes.
    fn device_unit(mcu_id: u32, frame: &[u8]) -> Vec<u8> {
        let mut unit = encode_header(assign::reply_arb_id(mcu_id).expect("a valid mcu id"));
        unit.extend_from_slice(frame);
        unit
    }

    /// Split a host→device byte run into `(arb, frame)` units, asserting each is
    /// whole (a valid header, then one parseable frame).
    fn split_units(mut wire: &[u8]) -> Vec<(u32, Frame)> {
        let mut units = Vec::new();
        while !wire.is_empty() {
            let (arb, consumed) = match decode_header(wire) {
                HeaderScan::Complete { arb, consumed } => (arb, consumed),
                other => panic!("a sent unit must start with a header, got {other:?}"),
            };
            let length = wire[consumed] as usize;
            let frame = Frame::parse(&wire[consumed..consumed + length]).expect("a whole frame");
            units.push((arb, frame));
            wire = &wire[consumed + length..];
        }
        units
    }

    #[test]
    fn attach_sends_the_request_verbatim_and_returns_a_handle() {
        let pty = Pty::new();
        // The helper asserts the exact bytes on the wire before answering.
        let handle = attach_with_reply(&pty, 4);

        // The returned handle is the caller's to use.
        handle.send(&Frame::new(0, b"hello".to_vec())).unwrap();
        let wire = read_n(
            &pty.master,
            2 + Frame::new(0, b"hello".to_vec()).raw_bytes().len(),
        );
        let units = split_units(&wire);
        assert_eq!(
            units,
            vec![(assign::arb_id(4).unwrap(), Frame::new(0, b"hello".to_vec()))]
        );
    }

    #[test]
    fn attach_times_out_when_no_reply_arrives() {
        let pty = Pty::new();
        let err = attach(&pty.path, DEFAULT_BAUD, 4, Duration::from_millis(50)).unwrap_err();
        assert_eq!(err, InterfaceError::Timeout);

        // Every attempt went out, byte for byte.
        let request = assignment_request(4).unwrap();
        let all = read_n(&pty.master, request.len() * ASSIGN_ATTACH_ATTEMPTS as usize);
        for chunk in all.chunks(request.len()) {
            assert_eq!(chunk, request.as_slice(), "one retry");
        }

        // The failed slot was unregistered and the port is still usable: a fresh
        // attach on the same path completes a handshake.
        let handle = attach_with_reply(&pty, 4);
        assert!(handle.send(&Frame::new(1, vec![0xab])).is_ok());
    }

    #[test]
    fn two_mcus_share_one_port_without_crosstalk() {
        let pty = Pty::new();
        let handle4 = attach_with_reply(&pty, 4);
        let handle5 = attach_with_reply(&pty, 5);

        let frame_a = Frame::encode(1, b"aaaa");
        let frame_b = Frame::encode(2, b"bbbb");
        let mut wire = device_unit(4, &frame_a);
        wire.extend_from_slice(&device_unit(5, &frame_b));
        master_write(&pty.master, &wire);

        // Each handle sees only its own MCU's frame, distinguished by both
        // sequence and payload.
        assert_eq!(handle4.receive(), Some(Frame::new(1, b"aaaa".to_vec())));
        assert_eq!(handle5.receive(), Some(Frame::new(2, b"bbbb".to_vec())));
    }

    #[test]
    fn sent_units_are_not_interleaved() {
        let pty = Pty::new();
        let handle4 = attach_with_reply(&pty, 4);
        let handle5 = attach_with_reply(&pty, 5);

        let frame_a = Frame::new(3, b"alpha".to_vec());
        let frame_b = Frame::new(4, b"beta".to_vec());
        handle4.send(&frame_a).unwrap();
        handle5.send(&frame_b).unwrap();

        let want = 2 + frame_a.raw_bytes().len() + 2 + frame_b.raw_bytes().len();
        let wire = read_n(&pty.master, want);
        let mut units = split_units(&wire);
        units.sort_by_key(|(arb, _)| *arb);

        let mut expected = vec![
            (assign::arb_id(4).unwrap(), frame_a),
            (assign::arb_id(5).unwrap(), frame_b),
        ];
        expected.sort_by_key(|(arb, _)| *arb);
        assert_eq!(units, expected);
    }

    #[test]
    fn the_last_handle_closes_the_port_and_the_others_keep_it_up() {
        let pty = Pty::new();
        let handle4 = attach_with_reply(&pty, 4);
        let handle5 = attach_with_reply(&pty, 5);

        // Dropping one handle leaves the port open for the other.
        drop(handle4);
        let frame = Frame::new(1, b"ping".to_vec());
        handle5.send(&frame).unwrap();
        let wire = read_n(&pty.master, 2 + frame.raw_bytes().len());
        assert_eq!(
            split_units(&wire),
            vec![(assign::arb_id(5).unwrap(), frame)]
        );
        master_write(&pty.master, &device_unit(5, &Frame::encode(2, b"pong")));
        assert_eq!(handle5.receive(), Some(Frame::new(2, b"pong".to_vec())));

        // The last handle brings the port down: its registry entry is dead, so
        // its `Drop` ran — and that is what joins the reader thread.
        drop(handle5);
        let dead = ports()
            .lock()
            .unwrap()
            .get(&pty.path)
            .and_then(Weak::upgrade)
            .is_none();
        assert!(dead, "the port must be dropped once every handle is gone");

        // A fresh attach on the same path builds a new port and a new reader.
        let handle = attach_with_reply(&pty, 4);
        assert!(handle.send(&Frame::new(9, vec![1])).is_ok());
    }

    #[test]
    fn a_frame_for_an_unregistered_mcu_is_dropped() {
        let pty = Pty::new();
        let handle4 = attach_with_reply(&pty, 4);

        // A unit for MCU 9 (never attached) sits in front of MCU 4's frame.
        let mut wire = device_unit(9, &Frame::encode(1, b"junk"));
        wire.extend_from_slice(&device_unit(4, &Frame::encode(5, b"real")));
        master_write(&pty.master, &wire);

        // MCU 4's frame still arrives, in order and intact.
        assert_eq!(handle4.receive(), Some(Frame::new(5, b"real".to_vec())));
    }

    #[test]
    fn a_bad_length_byte_resynchronizes_and_the_next_unit_is_dispatched() {
        let pty = Pty::new();
        let handle4 = attach_with_reply(&pty, 4);

        // Two impossible length bytes, each followed by a SYNC to land on.
        let mut wire = Vec::new();
        for bad in [0x00u8, 0xff] {
            wire.extend_from_slice(&encode_header(assign::reply_arb_id(4).unwrap()));
            wire.push(bad);
            wire.push(MESSAGE_SYNC);
        }
        wire.extend_from_slice(&device_unit(4, &Frame::encode(7, b"after")));
        master_write(&pty.master, &wire);

        // The port resynchronised and dispatched the valid unit behind the junk.
        assert_eq!(handle4.receive(), Some(Frame::new(7, b"after".to_vec())));
    }

    // -----------------------------------------------------------------------
    // The assignment pseudo-frame: shape, old-firmware behavior, and the
    // handshake's own codec.
    // -----------------------------------------------------------------------

    /// The patterned pseudo-frame as raw wire bytes: the fixed `KXID` body the
    /// handshake carries in both directions.
    fn pseudo_frame() -> Vec<u8> {
        assignment_frame()
    }

    #[test]
    fn the_pseudo_frame_is_a_well_formed_frame_with_only_the_seq_byte_wrong() {
        let bytes = pseudo_frame();
        // len = header(2) + payload("KXID" = 4) + trailer(3) = 9.
        assert_eq!(bytes.len(), 9);
        assert_eq!(bytes[MESSAGE_POS_LEN], 9);
        assert_eq!(bytes[MESSAGE_POS_SEQ], ASSIGN_SEQ_BYTE);
        // The one field that differs from an ordinary frame: bit 4 clear.
        assert_ne!(bytes[MESSAGE_POS_SEQ] & !MESSAGE_SEQ_MASK, MESSAGE_DEST);
        assert_eq!(&bytes[2..6], b"KXID");
        assert_eq!(*bytes.last().unwrap(), MESSAGE_SYNC);
        // The CRC is the ordinary one over [len, seq, payload] — the shape a
        // capable firmware can validate without inventing a checksum.
        let expected = crate::crc16_ccitt!(&bytes[..bytes.len() - MESSAGE_TRAILER_SIZE]);
        let actual = ((bytes[bytes.len() - 3] as u16) << 8) | bytes[bytes.len() - 2] as u16;
        assert_eq!(expected, actual);
    }

    #[test]
    fn the_body_and_header_are_free_of_an_interior_sync() {
        // The firmware's error path discards through the first 0x7e: with the
        // length byte, sequence byte and payload free of it, the frame is
        // consumed in one step. The two CRC bytes are computed, not chosen; for
        // this fixed body they are 0x4b/0x4f, so the trailer's own 0x7e is the
        // only SYNC in the frame.
        let bytes = pseudo_frame();
        let header_and_payload = &bytes[..bytes.len() - MESSAGE_TRAILER_SIZE];
        assert!(
            header_and_payload.iter().all(|&b| b != MESSAGE_SYNC),
            "a SYNC appeared in the header/payload: {bytes:02x?}"
        );
        assert!((MESSAGE_MIN..=MESSAGE_MAX).contains(&bytes.len()));
    }

    // -----------------------------------------------------------------------
    // Old-firmware behavior: a faithful port of `command_find_block`
    // (`third_party/klipper/src/command.c:276-333`), so the compatibility
    // claim above is executable. Structure and citation, not an approximation:
    // the checks run in the C order and the error path consumes through the
    // first SYNC exactly as `memchr` does.
    // -----------------------------------------------------------------------

    /// What one call to the firmware's block finder decides.
    #[derive(Debug, PartialEq, Eq)]
    enum Outcome {
        /// Not enough bytes yet; the buffer waits.
        NeedMore,
        /// A block passed structure, CRC and sequence: dispatch follows.
        Dispatch,
        /// Structure failed; bytes consumed through the SYNC (`pop_count`),
        /// and (only on the first failure) an empty ack frame is sent.
        Discarded { consumed: usize, nacked: bool },
        /// Structure and CRC passed but the sequence was wrong: nak, and the
        /// block is consumed (`*pop_count = msglen` before the check).
        SeqNak { consumed: usize },
    }

    /// The firmware's static state across calls (`command.c:276-281,299,307`).
    struct FwState {
        /// `next_sequence`, initialised to `MESSAGE_DEST` (`command.c:16`).
        next_sequence: u8,
        /// `CF_NEED_SYNC`.
        need_sync: bool,
        /// `CF_NEED_VALID`.
        need_valid: bool,
    }

    impl Default for FwState {
        fn default() -> Self {
            // `static uint8_t next_sequence = MESSAGE_DEST;` (`command.c:16`).
            Self {
                next_sequence: MESSAGE_DEST,
                need_sync: false,
                need_valid: false,
            }
        }
    }

    fn fw_find_block(buf: &[u8], st: &mut FwState) -> Outcome {
        // `if (buf_len && sync_state & CF_NEED_SYNC) goto need_sync;`
        if !buf.is_empty() && st.need_sync {
            return fw_need_sync(buf, st);
        }
        if buf.len() < MESSAGE_MIN {
            return Outcome::NeedMore;
        }
        let msglen = buf[MESSAGE_POS_LEN] as usize;
        // `:284` — length range.
        if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&msglen) {
            return fw_error(buf, st);
        }
        let msgseq = buf[MESSAGE_POS_SEQ];
        // `:287` — the check this pattern is built to meet.
        if (msgseq & !MESSAGE_SEQ_MASK) != MESSAGE_DEST {
            return fw_error(buf, st);
        }
        if buf.len() < msglen {
            return Outcome::NeedMore;
        }
        // `:291` — trailer SYNC.
        if buf[msglen - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC {
            return fw_error(buf, st);
        }
        // `:293-297` — CRC.
        let msgcrc = ((buf[msglen - 3] as u16) << 8) | buf[msglen - 2] as u16;
        let crc = crate::crc16_ccitt!(&buf[..msglen - MESSAGE_TRAILER_SIZE]);
        if crc != msgcrc {
            return fw_error(buf, st);
        }
        // `:298` — a structure-valid block clears NEED_VALID.
        st.need_valid = false;
        // `:301-303` — sequence.
        if msgseq != st.next_sequence {
            return Outcome::SeqNak { consumed: msglen };
        }
        st.next_sequence = ((msgseq + 1) & MESSAGE_SEQ_MASK) | MESSAGE_DEST;
        Outcome::Dispatch
    }

    fn fw_error(buf: &[u8], st: &mut FwState) -> Outcome {
        // `:311-316` — a leading SYNC is dropped alone, silently.
        if buf.first() == Some(&MESSAGE_SYNC) {
            return Outcome::Discarded {
                consumed: 1,
                nacked: false,
            };
        }
        // `:317` then fall into need_sync (`:318`).
        st.need_sync = true;
        fw_need_sync(buf, st)
    }

    fn fw_need_sync(buf: &[u8], st: &mut FwState) -> Outcome {
        // `:317-323`: discard through the first SYNC (or the whole buffer),
        // then the one-time nak.
        let consumed = match buf.iter().position(|&b| b == MESSAGE_SYNC) {
            Some(position) => {
                st.need_sync = false;
                position + 1
            }
            None => buf.len(),
        };
        if st.need_valid {
            return Outcome::Discarded {
                consumed,
                nacked: false,
            };
        }
        st.need_valid = true;
        Outcome::Discarded {
            consumed,
            nacked: true,
        }
    }

    /// An ordinary, correctly-sequenced host block (identify's request frame).
    fn identify_frame(seq_nibble: u8) -> Vec<u8> {
        let seq = (seq_nibble & MESSAGE_SEQ_MASK) | MESSAGE_DEST;
        Frame::encode(seq, b"\x01identify offset=0 count=112")
    }

    #[test]
    fn the_firmware_discards_the_request_without_dispatching_it() {
        let request = pseudo_frame();
        let mut st = FwState::default();
        // First failure: consumed exactly through our own trailer, one ack
        // frame in reply, no dispatch.
        assert_eq!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded {
                consumed: request.len(),
                nacked: true
            }
        );
        // `next_sequence` never moved — identify starts at its initial value.
        assert_eq!(st.next_sequence, MESSAGE_DEST);
        // A second assignment frame is discarded silently (NEED_VALID holds
        // until a valid block clears it).
        assert_eq!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded {
                consumed: request.len(),
                nacked: false
            }
        );
        // And the identify frame that follows dispatches: the pattern leaves
        // the stream exactly where it found it.
        let identify = identify_frame(MESSAGE_DEST);
        assert_eq!(fw_find_block(&identify, &mut st), Outcome::Dispatch);
    }

    #[test]
    fn a_request_sandwiched_before_identify_leaves_identify_intact() {
        // The wire order at connect: our frame, then identify's block. The
        // firmware consumes only ours; identify dispatches on the next call.
        let mut stream = pseudo_frame();
        let identify = identify_frame(MESSAGE_DEST);
        stream.extend_from_slice(&identify);

        let mut st = FwState::default();
        let (first, consumed) = match fw_find_block(&stream, &mut st) {
            outcome @ Outcome::Discarded { consumed, .. } => (outcome, consumed),
            other => panic!("the request must be discarded, got {other:?}"),
        };
        assert!(matches!(first, Outcome::Discarded { nacked: true, .. }));
        assert_eq!(consumed, 9, "only our frame is consumed");
        assert_eq!(
            fw_find_block(&stream[consumed..], &mut st),
            Outcome::Dispatch
        );
    }

    #[test]
    fn the_firmware_would_dispatch_garbage_in_the_pattern_but_never_our_bytes() {
        // The payload is never read on the error path — prove it by putting a
        // syntactically valid command in there with a *correct* CRC over it:
        // the sequence-byte check has already decided, so nothing downstream
        // is reached. (Removing the dest check would not make it dispatch
        // either: the sequence check runs next, `next_sequence` only ever holds
        // `0x10..=0x1f` and never `0x20`, so the block would be nakaed — both
        // checks refuse it.)
        let mut bytes = pseudo_frame();
        bytes[2..6].copy_from_slice(b"\x00sta"); // msgid 0 + text, CRC fixed:
        let crc = crate::crc16_ccitt!(&bytes[..bytes.len() - MESSAGE_TRAILER_SIZE]);
        let len = bytes.len();
        bytes[len - 3] = (crc >> 8) as u8;
        bytes[len - 2] = crc as u8;
        let mut st = FwState::default();
        assert!(matches!(
            fw_find_block(&bytes, &mut st),
            Outcome::Discarded { .. }
        ));
        assert_eq!(st.next_sequence, MESSAGE_DEST);
    }

    #[test]
    fn a_valid_block_rearms_the_one_nak_gate() {
        // `sync_state &= ~CF_NEED_VALID` (`command.c:298`) is the only thing
        // that re-arms the one-time nak: after it, the next structural error
        // nakks again, and the one after that is silent again.
        let mut st = FwState::default();
        let request = pseudo_frame();
        assert!(matches!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded { nacked: true, .. }
        ));
        // A valid identify block clears the flag on its way to dispatch.
        assert_eq!(
            fw_find_block(&identify_frame(MESSAGE_DEST), &mut st),
            Outcome::Dispatch
        );
        // So the *next* structural error nakks once more — re-armed …
        let request = pseudo_frame();
        assert!(matches!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded { nacked: true, .. }
        ));
        // … and damping resumes immediately after.
        assert!(matches!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded { nacked: false, .. }
        ));
        assert_eq!(st.next_sequence, MESSAGE_DEST | 1);
    }

    #[test]
    fn a_frame_split_across_reads_is_still_discarded_once_complete() {
        // Real links deliver bytes in arbitrary slices: the firmware buffers
        // (`need_more_data`, silent — `command.c:281-282`) until MESSAGE_MIN
        // bytes are present, then runs the same checks on the accumulated
        // buffer — our frame is discarded whole on the completing read.
        let request = pseudo_frame();
        let mut st = FwState::default();
        assert_eq!(fw_find_block(&request[..4], &mut st), Outcome::NeedMore);
        let mut buffered = request[..4].to_vec();
        buffered.extend_from_slice(&request[4..]);
        assert_eq!(
            fw_find_block(&buffered, &mut st),
            Outcome::Discarded {
                consumed: request.len(),
                nacked: true
            }
        );
        assert_eq!(st.next_sequence, MESSAGE_DEST);
    }

    #[test]
    fn a_request_after_identify_is_discarded_without_disturbing_the_session() {
        // The other half of the README's “before and after identify” claim:
        // arriving *after* a valid block, the flag was just cleared by it, so
        // this error nakks once (the harmless empty frame), then damps — never
        // dispatch, and the sequence identify advanced to stays put.
        let mut st = FwState::default();
        assert_eq!(
            fw_find_block(&identify_frame(MESSAGE_DEST), &mut st),
            Outcome::Dispatch
        );
        assert_eq!(st.next_sequence, MESSAGE_DEST | 1);

        let request = pseudo_frame();
        assert_eq!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded {
                consumed: request.len(),
                nacked: true
            }
        );
        assert_eq!(
            st.next_sequence,
            MESSAGE_DEST | 1,
            "the request must not move the sequence identify left behind"
        );
        assert!(matches!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded { nacked: false, .. }
        ));
    }

    // -----------------------------------------------------------------------
    // The frame header codec and the assignment unit
    // -----------------------------------------------------------------------

    #[test]
    fn headers_are_two_bytes_across_the_whole_id_range() {
        for id in 1..=MAX_MCU_ID {
            for arb in [
                assign::arb_id(id).unwrap(),
                assign::reply_arb_id(id).unwrap(),
            ] {
                let header = encode_header(arb);
                assert_eq!(header.len(), 2, "arb {arb:#x}");
                assert_eq!(
                    decode_header(&header),
                    HeaderScan::Complete { arb, consumed: 2 }
                );
            }
        }
        // The endpoints of the range are the two-byte values below.
        assert_eq!(encode_header(0x102), vec![0x82, 0x02]);
        assert_eq!(encode_header(0x2ff), vec![0x85, 0x7f]);

        // A truncated header waits for the rest instead of guessing.
        assert_eq!(decode_header(&[]), HeaderScan::NeedMore);
        assert_eq!(decode_header(&[0x82]), HeaderScan::NeedMore);
        let full = encode_header(0x2ff);
        assert_eq!(decode_header(&full[..1]), HeaderScan::NeedMore);
        assert_eq!(
            decode_header(&full),
            HeaderScan::Complete {
                arb: 0x2ff,
                consumed: 2
            }
        );
    }

    #[test]
    fn boundary_id_headers_may_contain_a_sync_byte() {
        // The protocol allows 0x7e in frame *content* (`docs/Protocol.md:178-183`:
        // “a sync character is not exclusive to the framing and may be present
        // in the message block content”). A VLQ whose last byte is 0x7e is
        // therefore acceptable, and these values pin that behavior on purpose.
        for (id, expected) in [
            (63u32, [0x82u8, 0x7e]),
            (127, [0x83, 0x7e]),
            (191, [0x84, 0x7e]),
            (255, [0x85, 0x7e]),
        ] {
            let arb = assign::arb_id(id).unwrap();
            let header = encode_header(arb);
            assert_eq!(header, expected, "id {id}");
            assert!(
                header.contains(&MESSAGE_SYNC),
                "id {id} should exercise the 0x7e content byte"
            );
            assert_eq!(
                decode_header(&header),
                HeaderScan::Complete { arb, consumed: 2 }
            );
        }
    }

    #[test]
    fn assignment_units_round_trip_through_the_ack_parser() {
        let request = assignment_request(42).unwrap();
        let reply = assignment_reply(42).unwrap();

        // Request: the listener header, then the fixed pseudo-frame.
        assert_eq!(
            &request[..2],
            encode_header(assign::arb_id(42).unwrap()).as_slice()
        );
        assert_eq!(request.len(), 2 + 9);
        assert_eq!(request[2], 9); // frame length byte
        assert_eq!(request[3], ASSIGN_SEQ_BYTE);
        assert_eq!(&request[4..8], MAGIC);
        assert_eq!(*request.last().unwrap(), MESSAGE_SYNC);

        // Reply: the answer header; the body is byte-for-byte the request's.
        assert_eq!(
            &reply[..2],
            encode_header(assign::reply_arb_id(42).unwrap()).as_slice()
        );
        assert_eq!(&reply[2..], &request[2..]);

        // The parser recognizes the reply and rejects the request's direction.
        assert_eq!(parse_assignment_ack(&reply), Some(42));
        assert_eq!(parse_assignment_ack(&request), None);

        for id in [1u32, 42, 200, 255] {
            assert_eq!(
                parse_assignment_ack(&assignment_reply(id).unwrap()),
                Some(id),
                "id {id}"
            );
        }

        // Out-of-range ids have no unit at all.
        assert_eq!(assignment_request(0), None);
        assert_eq!(assignment_request(256), None);
        assert_eq!(assignment_reply(0), None);
        assert_eq!(assignment_reply(256), None);
    }

    #[test]
    fn a_malformed_assignment_ack_is_rejected() {
        let reply = assignment_reply(7).unwrap();
        assert_eq!(parse_assignment_ack(&reply), Some(7));

        // Bad CRC — flip the trailer's high CRC byte.
        let mut bad_crc = reply.clone();
        let crc_at = bad_crc.len() - 3;
        bad_crc[crc_at] ^= 0xff;
        assert_eq!(parse_assignment_ack(&bad_crc), None);

        // Wrong sequence byte.
        let mut bad_seq = reply.clone();
        bad_seq[2 + MESSAGE_POS_SEQ] = 0x10;
        assert_eq!(parse_assignment_ack(&bad_seq), None);

        // Truncation: the trailer's SYNC removed, and the header cut short.
        assert_eq!(parse_assignment_ack(&reply[..reply.len() - 1]), None);
        assert_eq!(parse_assignment_ack(&reply[..1]), None);

        // No MAGIC: a structurally valid pseudo-frame with different payload.
        let mut wrong_magic = encode_header(assign::reply_arb_id(7).unwrap());
        wrong_magic.extend_from_slice(
            &Frame::with_wire_seq(ASSIGN_SEQ_BYTE, b"XXXX".to_vec()).raw_bytes(),
        );
        assert_eq!(parse_assignment_ack(&wrong_magic), None);

        // Wrong trailer SYNC byte.
        let mut bad_trailer = reply.clone();
        *bad_trailer.last_mut().unwrap() = 0x00;
        assert_eq!(parse_assignment_ack(&bad_trailer), None);

        // Trailing bytes after the frame: not a self-consistent unit.
        let mut extra = reply.clone();
        extra.push(0x00);
        assert_eq!(parse_assignment_ack(&extra), None);
    }
}
