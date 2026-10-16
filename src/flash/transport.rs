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
        self.dev.borrow_mut().write(buf)
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
    link.dev.borrow_mut().write_all(&prime)?;

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

    // `flashtool.py` `_reset_nodes`: one broadcast, so no other board keeps
    // a stale node id from a previous session.
    bus.write_frame(ADMIN_ID, &[CMD_CLEAR_NODE_ID])?;

    // `flashtool.py` `_set_node_id`: the command, the UUID most significant
    // byte first, the node id.
    let mut payload = [0u8; CAN_DATA_BYTES];
    payload[0] = CMD_SET_NODE_ID;
    payload[1..7].copy_from_slice(&uuid);
    payload[7] = nodeid as u8;
    bus.write_frame(ADMIN_ID, &payload)?;

    // The UUID check: `GET_CANBUS_ID` over the data channel the assignment
    // just opened. Anything else answering (wrong UUID, wrong node id) is
    // caught before the flash starts.
    let mut reader = bus.try_clone()?;
    let got = session::get_canbus_id(&mut reader, &mut bus, cfg)?;
    if got != uuid {
        return Err(TransportError::UuidMismatch {
            expected: uuid,
            got,
        });
    }
    Ok((reader, bus))
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

    /// The prime is the frame `flashtool.py` sends: invalid command 0x90,
    /// empty payload, the CRC over `[0x90, 0x00]`.
    #[test]
    fn test_prime_frame_bytes() {
        let prime = frame::build_frame(PRIME_CMD, &[]);
        assert_eq!(prime, [0x01, 0x88, 0x90, 0x00, 0xe9, 0xe5, 0x99, 0x03]);
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
