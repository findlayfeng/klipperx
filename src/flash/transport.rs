//! The two direct transports a katapult flash runs over.
//!
//! The session ([`super::session`]) speaks any `Read + Write`; this module
//! supplies the two byte streams a board is reachable on without a running
//! klipper in the middle, matching what
//! `third_party/katapult/scripts/flashtool.py` does for its serial and CAN
//! paths:
//!
//! - [`open_tty`] — an exclusive serial port, which is also how a USB CDC
//!   device enumerating as katapult (`1d50:6177`) is reached. The port is
//!   opened in raw mode by [`SerialDevice`], then *primed* with one invalid
//!   command: a double-buffered USB device does not answer the first real
//!   command until something has been sent (`flashtool.py` `prime`). The
//!   error frame the prime provokes is discarded here, before the session
//!   starts, because the session aborts a command that is answered with
//!   0xF2.
//!
//! - [`open_can`] — a SocketCAN bus, where the board is addressed in three
//!   admin steps on `0x3f0` before the session runs on the node's data ids:
//!   clear every bootloader node id (`0x12`), assign this board the session
//!   node id (`0x11`, payload `[0x11][uuid][nodeid]` with `nodeid >= 128`),
//!   then — over the data channel the assignment opens (`tx = 0x100 +
//!   2×nodeid`, `rx = tx + 1`) — check with `GET_CANBUS_ID (0x16)` that the
//!   board answering really is the requested UUID. The katapult byte stream
//!   rides the bus as plain slices: ≤ 8 bytes per frame, no framing of its
//!   own, so the adapter here is only chunk-out / reassemble-in.

use std::cell::RefCell;
use std::collections::VecDeque;
use std::fmt;
use std::fs::File;
use std::io::{self, ErrorKind, Read, Write};
use std::os::fd::AsRawFd;
use std::time::{Duration, Instant};

use crate::core::klippy::interface::devices::canserial::{
    self, CanFrame, CanWire, CAN_DATA_BYTES, CAN_FD_FRAME_SIZE, CAN_FRAME_SIZE,
};
use crate::core::klippy::interface::devices::serial::SerialDevice;
use crate::core::klippy::interface::InterfaceError;

use super::frame::{self, FrameScanner};
use super::session::{self, FlashError, SessionConfig};

/// The command byte the prime uses: one katapult will not answer with an
/// ack, only with the error frame that wakes a double-buffered USB device.
const PRIME_CMD: u8 = 0x90;

/// How long [`open_tty`] waits to catch and discard the prime's error frame.
/// A link that answers at all answers within milliseconds; one that never
/// does is left alone after this window, and the session's own retries take
/// over from there.
const PRIME_DRAIN: Duration = Duration::from_secs(1);

/// How long the prime frame has to be taken by the port — the write window of
/// [`SessionConfig::default`], which is the one the session that follows the
/// prime runs on.
const PRIME_WRITE_WINDOW: Duration = Duration::from_secs(2);

/// The admin channel every board listens on before it has a node id
/// (`flashtool.py` `CANBUS_ID_ADMIN`).
const ADMIN_ID: u32 = 0x3f0;

/// The katapult admin commands — different values from klipper's own admin
/// protocol on the same channel (`flashtool.py`: `CANBUS_CMD_*`).
const CMD_SET_NODE_ID: u8 = 0x11;
const CMD_CLEAR_NODE_ID: u8 = 0x12;

/// The data ids of the node session: `tx = NODE_TX_BASE + 2×nodeid`,
/// `rx = tx + 1` (`flashtool.py`: `decoded_id = node_id * 2 + 0x100`).
const NODE_TX_BASE: u32 = 0x100;

/// One poll wait on a CAN socket, and one read attempt after it: short
/// enough that a session read window stays the deadline that matters.
const CAN_POLL: Duration = Duration::from_millis(100);

/// Everything that can stop a transport from delivering a byte stream.
#[derive(Debug)]
pub enum TransportError {
    /// The port, socket or wire itself failed.
    Io(io::Error),
    /// Opening or configuring the device failed.
    Interface(InterfaceError),
    /// The bootloader session itself failed mid-handshake.
    Session(FlashError),
    /// The session's UUID check found a different board than requested.
    UuidMismatch { expected: [u8; 6], got: [u8; 6] },
}

impl fmt::Display for TransportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TransportError::Io(e) => write!(f, "transport I/O error: {e}"),
            TransportError::Interface(e) => write!(f, "{e}"),
            TransportError::Session(e) => write!(f, "{e}"),
            TransportError::UuidMismatch { expected, got } => write!(
                f,
                "the board answering is {}, not the requested {}",
                hex_uuid(got),
                hex_uuid(expected)
            ),
        }
    }
}

impl std::error::Error for TransportError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            TransportError::Io(e) => Some(e),
            TransportError::Interface(e) => Some(e),
            TransportError::Session(e) => Some(e),
            TransportError::UuidMismatch { .. } => None,
        }
    }
}

impl From<io::Error> for TransportError {
    fn from(e: io::Error) -> Self {
        TransportError::Io(e)
    }
}

impl From<InterfaceError> for TransportError {
    fn from(e: InterfaceError) -> Self {
        TransportError::Interface(e)
    }
}

impl From<FlashError> for TransportError {
    fn from(e: FlashError) -> Self {
        TransportError::Session(e)
    }
}

/// A UUID the way this tool prints one: twelve hex digits, most significant
/// byte first — the order a `[mcu] uuid:` option is written in.
fn hex_uuid(uuid: &[u8; 6]) -> String {
    let mut out = String::with_capacity(12);
    for byte in uuid {
        out.push_str(&format!("{byte:02x}"));
    }
    out
}

// ===========================================================================
// Serial tty / USB CDC
// ===========================================================================

/// An open serial port, shared by the session's reader and writer halves.
///
/// The halves borrow one [`SerialDevice`], which already serialises its own
/// traffic; the session alternates between them and never holds both, so the
/// shared borrow cannot conflict.
pub struct TtyLink {
    dev: RefCell<SerialDevice>,
}

/// One half of a [`TtyLink`]: raw bytes off (or onto) the port.
pub struct TtyPort<'a> {
    dev: &'a RefCell<SerialDevice>,
}

impl Read for TtyPort<'_> {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.dev.borrow_mut().read(buf)
    }
}

impl Write for TtyPort<'_> {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        // A plain pass-through write, exactly as before the bounded write path
        // existed. The flash session's own window (`session::write_frame`) is
        // what bounds a frame, and a blocking `write` on this tty returns as soon
        // as the driver has taken the bytes.
        //
        // It must not go through `write_bounded`: that waits for `poll(POLLOUT)`
        // first, and on a real katapult flash the first block after `CONNECT`
        // found the port reporting no room for the whole window on a link that
        // had just carried the command — so the gate turned a working flash into
        // "transport took 0 of 76 bytes". The stall this module can act on is
        // "the session got nothing back in its read window", which it already
        // measures.
        let written = self.dev.borrow_mut().write(buf)?;
        Ok(written)
    }

    fn flush(&mut self) -> io::Result<()> {
        // The tty has no user-space buffer; a write is whole.
        Ok(())
    }
}

impl TtyLink {
    /// The reader and writer halves of the port, for the session's `Read +
    /// Write` pair.
    pub fn split(&self) -> (TtyPort<'_>, TtyPort<'_>) {
        (TtyPort { dev: &self.dev }, TtyPort { dev: &self.dev })
    }
}

/// Open `device` at `baud`, prime it, and return the port as the session's
/// byte stream.
///
/// The prime is the USB preamble of `flashtool.py`: one invalid command,
/// sent so a double-buffered CDC device starts answering before the first
/// real command is lost to buffering. The error frame the firmware answers with
/// is consumed here — if it reached the session it would abort the `CONNECT`
/// that follows it.
pub fn open_tty(device: &str, baud: u32) -> Result<TtyLink, TransportError> {
    let link = TtyLink {
        dev: RefCell::new(SerialDevice::open(device, baud)?),
    };
    prime(&link)?;
    Ok(link)
}

/// Send the prime command and discard the first complete frame that comes
/// back — the error ack it provokes, or the first slice of whatever the
/// device sent first.
///
/// A link that stays silent through the whole drain window is left as it
/// is: priming exists for USB buffering, and a device that will answer the
/// session will answer this too, inside the session's own read windows.
fn prime(link: &TtyLink) -> Result<(), TransportError> {
    let prime = frame::build_frame(PRIME_CMD, &[]);
    // The prime goes out under the same write window the session gives its own
    // frames, so a port that stops draining cannot stall the open.
    session::write_frame(&mut *link.dev.borrow_mut(), &prime, PRIME_WRITE_WINDOW)?;

    let deadline = Instant::now() + PRIME_DRAIN;
    let mut scanner = FrameScanner::new();
    let mut buf = [0u8; 256];
    loop {
        if scanner.next_frame().is_some() {
            return Ok(());
        }
        if Instant::now() >= deadline {
            return Ok(());
        }
        let n = link.dev.borrow_mut().read(&mut buf)?;
        if n > 0 {
            scanner.feed(&buf[..n]);
        }
    }
}

// ===========================================================================
// SocketCAN
// ===========================================================================

/// A SocketCAN data channel to one node: the byte stream of one katapult
/// session, carried as CAN frame payloads.
///
/// One socket, one node: writes go out sliced into [`CAN_DATA_BYTES`]-sized
/// frames on `tx_id`, reads come in from `rx_id` only — everything else on
/// the bus is consumed and dropped, which is what lets the admin answers and
/// other boards' traffic share the socket with the session.
pub struct CanBus {
    socket: File,
    /// The id the node listens on.
    tx_id: u32,
    /// The id the node answers on.
    rx_id: u32,
    /// Bytes read from accepted frames, not yet handed to the session.
    pending: VecDeque<u8>,
}

impl fmt::Debug for CanBus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CanBus")
            .field("tx_id", &format_args!("{:#x}", self.tx_id))
            .field("rx_id", &format_args!("{:#x}", self.rx_id))
            .finish_non_exhaustive()
    }
}

impl CanBus {
    /// A second handle to the same socket, for the session's reader/writer
    /// split. Reads and writes travel the one kernel socket; each handle
    /// keeps its own receive side.
    pub fn try_clone(&self) -> io::Result<CanBus> {
        Ok(CanBus {
            socket: self.socket.try_clone()?,
            tx_id: self.tx_id,
            rx_id: self.rx_id,
            pending: VecDeque::new(),
        })
    }

    /// A bus over any datagram socket: the kernel CAN socket delivers one
    /// frame per datagram, and so does a unix datagram pair — which is what
    /// the tests drive, needing no interface and no privileges.
    #[cfg(test)]
    pub(crate) fn over_dgram(socket: File, tx_id: u32, rx_id: u32) -> Self {
        Self {
            socket,
            tx_id,
            rx_id,
            pending: VecDeque::new(),
        }
    }

    /// The kernel's frame struct is what a socket read delivers, so a frame
    /// is one datagram: parse it back out of those bytes.
    fn recv_frame(&mut self) -> Result<Option<CanFrame>, TransportError> {
        loop {
            match canserial::wait_readable(self.socket.as_raw_fd(), CAN_POLL) {
                canserial::Wait::Readable => {
                    let mut raw = [0u8; CAN_FD_FRAME_SIZE];
                    let n = (&self.socket).read(&mut raw)?;
                    if n < CAN_FRAME_SIZE {
                        // A short datagram is not a classical CAN frame;
                        // nothing usable in it.
                        continue;
                    }
                    return Ok(Some(CanFrame::from_abi(CanWire::Classical, &raw)));
                }
                canserial::Wait::Retry => continue,
                canserial::Wait::Timeout => return Ok(None),
                canserial::Wait::Closed => {
                    return Err(TransportError::Io(io::Error::new(
                        ErrorKind::BrokenPipe,
                        "CAN interface went away",
                    )));
                }
            }
        }
    }
}

impl Read for CanBus {
    /// The next bytes the node sent: accepted frames reassembled, or `0`
    /// when a poll window passed with no frame — the session counts those
    /// against its own read deadline.
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        while self.pending.is_empty() {
            match self.recv_frame() {
                Ok(Some(frame)) => {
                    if frame.id() == self.rx_id {
                        self.pending.extend(frame.data());
                    }
                }
                Ok(None) => return Ok(0),
                Err(e) => return Err(io::Error::new(ErrorKind::Other, e.to_string())),
            }
        }
        let n = buf.len().min(self.pending.len());
        for byte in buf.iter_mut().take(n) {
            *byte = self.pending.pop_front().expect("pending is non-empty");
        }
        Ok(n)
    }
}

impl Write for CanBus {
    /// The byte stream, sliced into frames: one frame per [`CAN_DATA_BYTES`]
    /// chunk, exactly as the bus carries it — no framing of its own.
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        for chunk in buf.chunks(CAN_DATA_BYTES) {
            let frame =
                CanFrame::new(self.tx_id, chunk).expect("a chunk is at most one frame's worth");
            let abi = frame
                .to_abi(CanWire::Classical)
                .expect("a classical chunk fits a classical frame");
            (&self.socket).write_all(abi.as_bytes())?;
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        // One datagram per frame; nothing is buffered in user space.
        Ok(())
    }
}

/// Open `interface`, bring the board `uuid` up as node `nodeid`, verify the
/// UUID over the data channel, and return the session's reader/writer pair.
///
/// The three admin steps are `flashtool.py`'s canboot flow: clear every
/// bootloader node id once, assign this board the session node id, then ask
/// the data channel who it actually is before a single block is sent.
pub fn open_can(
    interface: &str,
    uuid: [u8; 6],
    nodeid: u32,
    cfg: &SessionConfig,
) -> Result<(CanBus, CanBus), TransportError> {
    let tx_id = NODE_TX_BASE + 2 * nodeid;
    let socket = canserial::open_socket(CanWire::Classical)?;
    let index = canserial::interface_index(interface)?;
    canserial::bind_interface(&socket, index)?;
    let mut bus = CanBus {
        socket,
        tx_id,
        rx_id: tx_id + 1,
        pending: VecDeque::new(),
    };

    // `flashtool.py` `_reset_nodes` and `_set_node_id`, below.
    assign_node(&mut bus, uuid, nodeid)?;

    // The UUID check: `GET_CANBUS_ID` over the data channel the assignment
    // just opened. Anything else answering (wrong UUID, wrong node id) is
    // caught before the flash starts.
    let mut reader = bus.try_clone()?;
    check_uuid(&mut reader, &mut bus, uuid, cfg)?;
    Ok((reader, bus))
}

/// The two admin frames that open a node session for `uuid`:
/// `flashtool.py`'s `_reset_nodes` (one broadcast, so no other board keeps
/// a stale node id) followed by `_set_node_id` (the command, the UUID most
/// significant byte first, the node id).
fn assign_node(bus: &mut CanBus, uuid: [u8; 6], nodeid: u32) -> Result<(), TransportError> {
    bus.write_frame(ADMIN_ID, &[CMD_CLEAR_NODE_ID])?;

    let mut payload = [0u8; CAN_DATA_BYTES];
    payload[0] = CMD_SET_NODE_ID;
    payload[1..7].copy_from_slice(&uuid);
    payload[7] = nodeid as u8;
    bus.write_frame(ADMIN_ID, &payload)?;
    Ok(())
}

/// Ask the data channel who it is, and refuse anyone but `uuid`.
fn check_uuid<R: Read, W: Write>(
    reader: &mut R,
    writer: &mut W,
    uuid: [u8; 6],
    cfg: &SessionConfig,
) -> Result<(), TransportError> {
    let got = session::get_canbus_id(reader, writer, cfg)?;
    if got != uuid {
        return Err(TransportError::UuidMismatch {
            expected: uuid,
            got,
        });
    }
    Ok(())
}

impl CanBus {
    /// Send one admin frame on the node-less channel.
    fn write_frame(&mut self, id: u32, data: &[u8]) -> Result<(), TransportError> {
        let frame = CanFrame::new(id, data)?;
        let abi = frame.to_abi(CanWire::Classical)?;
        (&self.socket).write_all(abi.as_bytes())?;
        Ok(())
    }
}

/// The admin answers id, for the tests that pin the channel pair.
#[cfg(test)]
pub(crate) const ADMIN_RESP: u32 = 0x3f1;

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::fd::{FromRawFd, IntoRawFd};
    use std::os::unix::net::UnixDatagram;

    /// The prime is the frame `flashtool.py` sends: invalid command 0x90,
    /// empty payload, the CRC over `[0x90, 0x00]` — value `0xE9E5`, on the
    /// wire little-endian (`struct.pack("<H")` in `flashtool.py`), so
    /// `… e5 e9 …`.
    #[test]
    fn test_prime_frame_bytes() {
        let prime = frame::build_frame(PRIME_CMD, &[]);
        assert_eq!(prime, [0x01, 0x88, 0x90, 0x00, 0xe5, 0xe9, 0x99, 0x03]);
    }

    /// The data ids of a node session: `tx = 0x100 + 2×nodeid`, `rx = tx+1`,
    /// the mapping `flashtool.py` derives.
    #[test]
    fn test_node_ids_follow_the_flashtool_mapping() {
        let (tx, rx) = (NODE_TX_BASE + 2 * 128, NODE_TX_BASE + 2 * 128 + 1);
        assert_eq!((tx, rx), (0x200, 0x201));
    }

    /// A UUID prints the way a `[mcu] uuid:` option is written.
    #[test]
    fn test_uuid_hex_order() {
        assert_eq!(
            hex_uuid(&[0x01, 0x23, 0x45, 0x67, 0x89, 0xab]),
            "0123456789ab"
        );
    }

    /// The tty path end to end: a pty pair plays the port, the scripted
    /// fake bootloader answers behind the master, and the full session runs
    /// over [`open_tty`]. The first bytes the device sees are the prime
    /// frame, golden byte for byte, before the `CONNECT` that follows it.
    #[test]
    fn test_open_tty_primes_then_the_session_flashes_over_the_pty() {
        use crate::core::klippy::interface::pty::Pty;
        use crate::flash::fake::FakeBootloader;
        use std::sync::atomic::{AtomicBool, Ordering};
        use std::sync::{Arc, Mutex};

        let pty = Pty::new();
        let fake = FakeBootloader::new();
        let raw = Arc::new(Mutex::new(Vec::new()));
        let stop = Arc::new(AtomicBool::new(false));

        // The board behind the master: every byte the host writes is fed to
        // the fake, every answer goes back out, and the raw wire bytes are
        // kept for the prime assertion.
        let bridge = {
            let raw = Arc::clone(&raw);
            let stop = Arc::clone(&stop);
            let mut writer = fake.writer();
            let mut reader = fake.reader();
            let mut master = pty.master.try_clone().expect("the master clones");
            std::thread::spawn(move || {
                let mut buf = [0u8; 256];
                let mut out = [0u8; 256];
                while !stop.load(Ordering::Relaxed) {
                    let mut poll = libc::pollfd {
                        fd: master.as_raw_fd(),
                        events: libc::POLLIN,
                        revents: 0,
                    };
                    let rc = unsafe { libc::poll(&mut poll, 1, 20) };
                    if rc > 0 {
                        let n = master.read(&mut buf).expect("the master reads");
                        if n == 0 {
                            break;
                        }
                        raw.lock().expect("raw lock").extend_from_slice(&buf[..n]);
                        writer.write_all(&buf[..n]).expect("the fake takes it");
                    }
                    match reader.read(&mut out) {
                        Ok(n) => master.write_all(&out[..n]).expect("the master takes it"),
                        Err(e) if e.kind() == io::ErrorKind::WouldBlock => {}
                        Err(e) => panic!("the fake reader: {e}"),
                    }
                }
            })
        };

        // Open (raw mode plus the prime) and run the whole flash.
        let link = open_tty(pty.path.to_str().expect("a utf-8 path"), 115_200)
            .expect("the port opens, primes and drains");
        let (mut reader, mut writer) = link.split();
        let payload = b"katapult direct tty".to_vec();
        let cfg = fast_session();
        let summary = session::flash_with_config(&mut reader, &mut writer, &payload, &cfg, |_| {})
            .expect("the session runs over the tty");
        assert_eq!(summary.bytes_flashed, payload.len());
        assert_eq!(summary.block_count, 1);

        stop.store(true, Ordering::Relaxed);
        bridge.join().expect("the bridge thread exits");

        // The prime came first on the wire: header, echo 0x88, the invalid
        // command, an empty payload, CRC 0xE9E5 little-endian, trailer.
        let raw = raw.lock().expect("raw lock");
        assert_eq!(&raw[..8], &[0x01, 0x88, 0x90, 0x00, 0xe5, 0xe9, 0x99, 0x03]);

        // The device stored exactly the payload, padded to the block size.
        let memory = fake.memory();
        assert_eq!(memory.len(), 1);
        let stored = memory.values().next().expect("one block stored");
        let mut want = payload;
        want.resize(64, 0xFF);
        assert_eq!(stored, &want);
    }

    /// The timing a transport test runs on: the session defaults, minus
    /// the seconds — the fake and the fixtures answer inside milliseconds.
    fn fast_session() -> SessionConfig {
        SessionConfig {
            tries: 3,
            read_timeout: Duration::from_millis(200),
            retry_delay: Duration::from_millis(10),
            busy_delay: Duration::from_millis(10),
        }
    }

    /// One classical frame as a socket delivers it: the ABI bytes of `id`
    /// carrying `data`.
    fn abi_datagram(id: u32, data: &[u8]) -> Vec<u8> {
        CanFrame::new(id, data)
            .expect("a chunk is at most one frame's worth")
            .to_abi(CanWire::Classical)
            .expect("a classical chunk fits a classical frame")
            .as_bytes()
            .to_vec()
    }

    /// One datagram off the device side of a pair, decoded as the frame it
    /// carries. The side must be non-blocking: an empty queue is a panic.
    fn recv_datagram(dev: &std::os::unix::net::UnixDatagram) -> CanFrame {
        let mut raw = [0u8; CAN_FD_FRAME_SIZE];
        let n = dev.recv(&mut raw).expect("one datagram");
        assert!(n >= CAN_FRAME_SIZE, "one datagram is one whole frame");
        CanFrame::from_abi(CanWire::Classical, &raw)
    }

    /// The bus is one datagram per frame both ways: a stream longer than a
    /// frame's worth goes out sliced into 8-byte frames, and a reply comes
    /// back reassembled — every frame that is not the node's dropped.
    #[test]
    fn test_can_slices_the_stream_out_and_reassembles_the_reply() {
        let (host, dev) = UnixDatagram::pair().expect("a datagram pair");
        // SAFETY: the raw fd is consumed by this `File` from here on.
        let host = unsafe { File::from_raw_fd(host.into_raw_fd()) };
        let (tx_id, rx_id) = (0x200, 0x201);
        let mut bus = CanBus::over_dgram(host, tx_id, rx_id);

        // Out: 20 bytes as 8 + 8 + 4, every slice on the node's tx id.
        let stream: Vec<u8> = (0u8..20).collect();
        bus.write_all(&stream).expect("the stream goes out");
        dev.set_nonblocking(true).expect("non-blocking reads");
        let mut sent = Vec::new();
        let mut raw = [0u8; CAN_FD_FRAME_SIZE];
        while let Ok(n) = dev.recv(&mut raw) {
            assert!(n >= CAN_FRAME_SIZE, "one datagram is one whole frame");
            let frame = CanFrame::from_abi(CanWire::Classical, &raw);
            assert_eq!(frame.id(), tx_id);
            sent.extend_from_slice(frame.data());
        }
        assert_eq!(sent, stream);

        // In: the reply on rx id with a foreign frame in the middle — only
        // the node's bytes are handed to the session.
        let reply: Vec<u8> = (100u8..116).collect();
        dev.send(&abi_datagram(ADMIN_ID, &[0xEE; 8]))
            .expect("a foreign frame");
        for chunk in reply.chunks(CAN_DATA_BYTES) {
            dev.send(&abi_datagram(rx_id, chunk))
                .expect("a reply frame");
        }
        let mut got = Vec::new();
        while got.len() < reply.len() {
            let n = bus.read(&mut raw).expect("the reply reads");
            assert_ne!(n, 0, "the reply arrived in full");
            got.extend_from_slice(&raw[..n]);
        }
        assert_eq!(got, reply);
    }

    /// The two admin frames `assign_node` sends, golden byte for byte: the
    /// clear broadcast, then the assignment — command 0x11, the UUID most
    /// significant byte first, the node id.
    #[test]
    fn test_assign_node_pins_the_admin_frame_bytes() {
        let (host, dev) = UnixDatagram::pair().expect("a datagram pair");
        let host = unsafe { File::from_raw_fd(host.into_raw_fd()) };
        let mut bus = CanBus::over_dgram(host, 0x200, 0x201);
        let uuid = [0x01, 0x23, 0x45, 0x67, 0x89, 0xab];
        assign_node(&mut bus, uuid, 0x83).expect("the assignment goes out");

        dev.set_nonblocking(true).expect("non-blocking reads");
        let clear = recv_datagram(&dev);
        assert_eq!(clear.id(), ADMIN_ID);
        assert_eq!(clear.data(), &[0x12]);
        let set = recv_datagram(&dev);
        assert_eq!(set.id(), ADMIN_ID);
        assert_eq!(
            set.data(),
            &[0x11, 0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0x83]
        );
        assert!(
            dev.recv(&mut [0u8; CAN_FD_FRAME_SIZE]).is_err(),
            "exactly two admin frames went out"
        );
    }

    /// The UUID check asks the data channel who it is (`GET_CANBUS_ID`,
    /// one frame on the node's tx id) and accepts the board it asked for.
    #[test]
    fn test_check_uuid_accepts_the_requested_board() {
        let (host, dev) = UnixDatagram::pair().expect("a datagram pair");
        let host = unsafe { File::from_raw_fd(host.into_raw_fd()) };
        let (tx_id, rx_id) = (0x200, 0x201);
        let mut bus = CanBus::over_dgram(host, tx_id, rx_id);
        let mut reader = bus.try_clone().expect("a second handle");
        let uuid = [0x01, 0x23, 0x45, 0x67, 0x89, 0xab];

        // The board's answer is queued before the ask: one socket carries
        // both directions, so no second thread is needed.
        let mut data = u32::from(frame::command::GET_CANBUS_ID)
            .to_le_bytes()
            .to_vec();
        data.extend_from_slice(&uuid);
        data.extend_from_slice(&[0x00, 0x00]);
        let answer = frame::build_frame(frame::ACK_SUCCESS, &data);
        for chunk in answer.chunks(CAN_DATA_BYTES) {
            dev.send(&abi_datagram(rx_id, chunk))
                .expect("an answer frame");
        }

        check_uuid(&mut reader, &mut bus, uuid, &fast_session())
            .expect("the board answers as asked");

        let ask = recv_datagram(&dev);
        assert_eq!(ask.id(), tx_id);
        assert_eq!(
            ask.data(),
            &frame::build_frame(frame::command::GET_CANBUS_ID, &[])[..]
        );
    }

    /// A different board answering the check is refused before the flash
    /// starts, and the error names both UUIDs.
    #[test]
    fn test_check_uuid_refuses_a_different_board() {
        let (host, dev) = UnixDatagram::pair().expect("a datagram pair");
        let host = unsafe { File::from_raw_fd(host.into_raw_fd()) };
        let (tx_id, rx_id) = (0x200, 0x201);
        let mut bus = CanBus::over_dgram(host, tx_id, rx_id);
        let mut reader = bus.try_clone().expect("a second handle");
        let wanted = [0x01, 0x23, 0x45, 0x67, 0x89, 0xab];
        let other = [0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54];

        let mut data = u32::from(frame::command::GET_CANBUS_ID)
            .to_le_bytes()
            .to_vec();
        data.extend_from_slice(&other);
        data.extend_from_slice(&[0x00, 0x00]);
        let answer = frame::build_frame(frame::ACK_SUCCESS, &data);
        for chunk in answer.chunks(CAN_DATA_BYTES) {
            dev.send(&abi_datagram(rx_id, chunk))
                .expect("an answer frame");
        }

        let err = check_uuid(&mut reader, &mut bus, wanted, &fast_session())
            .expect_err("another UUID is refused");
        match err {
            TransportError::UuidMismatch { expected, got } => {
                assert_eq!(expected, wanted);
                assert_eq!(got, other);
            }
            other_err => panic!("expected a UUID mismatch, got {other_err:?}"),
        }
    }

    /// The admin constants are the channel `flashtool.py` names, and the
    /// command bytes are katapult's — not klipper's admin values.
    #[test]
    fn test_admin_channel_and_commands() {
        assert_eq!(ADMIN_ID, 0x3f0);
        assert_eq!(ADMIN_RESP, 0x3f1);
        assert_eq!(CMD_SET_NODE_ID, 0x11);
        assert_eq!(CMD_CLEAR_NODE_ID, 0x12);
    }
}
