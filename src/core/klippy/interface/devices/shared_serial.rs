//! A serial port that several MCUs share.
//!
//! One tty can carry more than one MCU once every unit on it is prefixed with
//! its sender's arbitration id ([`crate::core::klippy::assign`]). This device
//! layer is the host side of that: the port is opened **once**, a single reader
//! thread cuts the byte stream into whole units and hands each one to the MCU
//! its header names, and each MCU gets a handle that a [`Mcu`] can drive like
//! any other [`Device`].
//!
//! The split from the single-owner [`SerialDevice`](super::serial::SerialDevice)
//! is deliberate: here the port must outlive any one handle (the last one to be
//! dropped closes it, and only then), the reader must not hold the port open on
//! its own, and writes from different MCUs must not interleave inside a unit.
//!
//! # Attaching
//!
//! `attach` performs the frame-level id handshake synchronously: it writes the
//! assignment unit ([`assign::assignment_request`]) and waits for the device's
//! answer ([`assign::assignment_reply`]) before returning a handle. Attaches on
//! the same path are serialized, so two MCUs coming up together cannot
//! interleave their handshakes.
//!
//! [`Mcu`]: crate::core::klippy::mcu::Mcu
//! [`assign::assignment_request`]: crate::core::klippy::assign::assignment_request
//! [`assign::assignment_reply`]: crate::core::klippy::assign::assignment_reply

use crate::core::klippy::assign::{self, HeaderScan};
use crate::core::klippy::config::mcu::MAX_MCU_ID;
use crate::core::klippy::frame::{Frame, FrameStream, MESSAGE_MAX, MESSAGE_MIN, MESSAGE_SYNC};
use crate::core::klippy::interface::error::InterfaceError;
use crate::core::klippy::interface::Device;
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
    /// device's answer to [`assign::assignment_reply`] is recognized.
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

    let request = assign::assignment_request(mcu_id).expect("arb_id checked above");
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
/// A unit is a bare VLQ header ([`assign::decode_header`]) followed by one frame
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

        let (arb, consumed) = match assign::decode_header(pending) {
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
        if let Some(mcu_id) = assign::parse_assignment_ack(unit) {
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
        let mut unit = assign::encode_header(arb);
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
        let request = assign::assignment_request(mcu_id).expect("a valid mcu id");
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
            &assign::assignment_reply(mcu_id).expect("a valid mcu id"),
        );
        thread
            .join()
            .expect("attach thread")
            .expect("attach succeeds")
    }

    /// One device→host unit: the answer-direction header, then the frame bytes.
    fn device_unit(mcu_id: u32, frame: &[u8]) -> Vec<u8> {
        let mut unit = assign::encode_header(assign::reply_arb_id(mcu_id).expect("a valid mcu id"));
        unit.extend_from_slice(frame);
        unit
    }

    /// Split a host→device byte run into `(arb, frame)` units, asserting each is
    /// whole (a valid header, then one parseable frame).
    fn split_units(mut wire: &[u8]) -> Vec<(u32, Frame)> {
        let mut units = Vec::new();
        while !wire.is_empty() {
            let (arb, consumed) = match assign::decode_header(wire) {
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
        let request = assign::assignment_request(4).unwrap();
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
            wire.extend_from_slice(&assign::encode_header(assign::reply_arb_id(4).unwrap()));
            wire.push(bad);
            wire.push(MESSAGE_SYNC);
        }
        wire.extend_from_slice(&device_unit(4, &Frame::encode(7, b"after")));
        master_write(&pty.master, &wire);

        // The port resynchronised and dispatched the valid unit behind the junk.
        assert_eq!(handle4.receive(), Some(Frame::new(7, b"after".to_vec())));
    }
}
