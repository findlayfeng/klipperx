//! Klipper's message blocks carried over CAN — the "can serial" link.
//!
//! Klipper does not put its protocol *on* CAN: the firmware takes the same byte
//! stream it would write to a serial port (length, sequence, CRC, SYNC — see
//! [`Frame`]) and chops it into 8-byte CAN frames (`src/generic/canserial.c`).
//! The bus contributes exactly two things, the arbitration id that picks a node
//! and eight data bytes per frame, which is why this module sits next to the
//! serial device: reassembly is the same [`FrameStream`] every byte-stream device
//! uses, and the CAN part is a chunking rule plus addressing.
//!
//! Addressing follows Klipper (`klippy/serialhdl.py`): a node id maps to
//! `0x100 + 2 * nodeid`, the host writes to that id, and the MCU answers on the
//! next one. Only classical frames are used: 11-bit ids and 8 data bytes, never
//! extended ids or CAN FD.
//!
//! ```text
//!   send(Frame) ─► bytes ─► 8 byte CAN frames (id = tx_id) ──────► can0
//!   receive()   ◄── FrameStream ◄── data bytes (id = tx_id + 1) ◄─ can0
//! ```
//!
//! # Naming
//!
//! This is the *can serial* transport: Klipper's serial link, carried over CAN.
//! Type names therefore say `CanSerial`, and the name `Canbus` is kept for an
//! interface that speaks the CAN protocol itself rather than borrowing the bus as
//! a wire.
//!
//! Configuration keys are a different matter: they follow Klipper's `[mcu]`
//! vocabulary (`canbus_uuid`, `canbus_interface`, and `canbus_nodeid` as Klipper's
//! own console and `serialhdl.connect_canbus` spell it), because those describe
//! the printer's wiring rather than this implementation, and a Klipper config
//! should keep working.

use super::error::InterfaceError;
use super::Device;
use crate::core::klippy::frame::{Frame, FrameStream};
use std::ffi::CString;
use std::fmt;
use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::{AsRawFd, FromRawFd, RawFd};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use tracing::{debug, info, trace, warn};

/// Data bytes in one classical CAN frame.
pub const CAN_DATA_BYTES: usize = 8;

/// Size of the kernel's `struct can_frame`, which is what a raw socket reads and
/// writes one datagram at a time.
const CAN_FRAME_SIZE: usize = 16;

/// Klipper's node-id mapping: `nodeid` → `0x100 + 2 * nodeid`.
const NODE_ID_BASE: u32 = 0x100;

/// Klipper's admin arbitration id, and the command that tells an unassigned MCU
/// which node id to take (`klippy/serialhdl.py`).
const ADMIN_ID: u32 = 0x3f0;
const CMD_SET_NODEID: u8 = 0x01;

/// How long a receive waits for a frame before rechecking the stop flag.
const POLL_TIMEOUT_MS: libc::c_int = 100;

// From `linux/can.h` and `linux/can/raw.h`; `libc` does not expose these.
const CAN_SFF_MASK: u32 = 0x7ff;
const CAN_EFF_FLAG: u32 = 0x8000_0000;
const CAN_RTR_FLAG: u32 = 0x4000_0000;
const CAN_RAW: libc::c_int = 1;
const SOL_CAN_RAW: libc::c_int = 101;
const CAN_RAW_FILTER: libc::c_int = 1;

/// `struct sockaddr_can`, whose address union is unused for raw sockets.
#[repr(C)]
struct SockAddrCan {
    can_family: libc::sa_family_t,
    can_ifindex: libc::c_int,
    can_addr: [u8; 8],
}

/// `struct can_filter`: which arbitration ids the socket accepts.
#[repr(C)]
struct CanFilter {
    can_id: u32,
    can_mask: u32,
}

/// One classical CAN frame: an 11-bit arbitration id and up to 8 data bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFrame {
    id: u32,
    len: u8,
    data: [u8; CAN_DATA_BYTES],
}

impl CanFrame {
    /// A frame carrying `data`, which has to fit in one CAN frame.
    pub fn new(id: u32, data: &[u8]) -> Result<Self, InterfaceError> {
        if data.len() > CAN_DATA_BYTES {
            return Err(InterfaceError::Other(format!(
                "{} bytes do not fit in one CAN frame",
                data.len()
            )));
        }
        let mut bytes = [0u8; CAN_DATA_BYTES];
        bytes[..data.len()].copy_from_slice(data);
        Ok(Self {
            id,
            len: data.len() as u8,
            data: bytes,
        })
    }

    /// Arbitration id.
    pub fn id(&self) -> u32 {
        self.id
    }

    /// The data bytes this frame carries.
    pub fn data(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }

    /// The kernel's layout: native-endian id, length, three reserved bytes, then
    /// the eight data bytes.
    fn to_abi(self) -> [u8; CAN_FRAME_SIZE] {
        let mut raw = [0u8; CAN_FRAME_SIZE];
        raw[..4].copy_from_slice(&self.id.to_ne_bytes());
        raw[4] = self.len;
        raw[8..].copy_from_slice(&self.data);
        raw
    }

    /// Parse the kernel's layout. Ids longer than 11 bits are extended frames,
    /// which nothing here uses.
    fn from_abi(raw: &[u8; CAN_FRAME_SIZE]) -> Self {
        let id = u32::from_ne_bytes([raw[0], raw[1], raw[2], raw[3]]) & CAN_SFF_MASK;
        let len = raw[4].min(CAN_DATA_BYTES as u8);
        let mut data = [0u8; CAN_DATA_BYTES];
        data.copy_from_slice(&raw[8..]);
        Self { id, len, data }
    }
}

/// The admin frame that assigns `nodeid` to the MCU with `uuid`: the command byte,
/// the UUID most significant byte first, then the node id.
///
/// The MCU answers on its new id only once it has seen this, which is why a config
/// needs the UUID even when it states the node id.
fn set_nodeid_payload(uuid: [u8; 6], nodeid: u32) -> [u8; CAN_DATA_BYTES] {
    let mut data = [0u8; CAN_DATA_BYTES];
    data[0] = CMD_SET_NODEID;
    data[1..7].copy_from_slice(&uuid);
    data[7] = nodeid as u8;
    data
}

/// The half of the transport that touches no socket: the mapping between Klipper's
/// byte stream and CAN frames.
#[derive(Debug)]
pub struct CanSerialLink {
    /// The id the MCU listens on; it answers on `tx_id + 1`.
    tx_id: u32,
    /// Data bytes collected from accepted frames.
    incoming: FrameStream,
}

impl CanSerialLink {
    /// The link to the node `nodeid`, using Klipper's id mapping.
    pub fn for_node(nodeid: u32) -> Self {
        Self::with_arbitration_id(NODE_ID_BASE + 2 * nodeid)
    }

    /// The link whose MCU listens on `tx_id`.
    pub fn with_arbitration_id(tx_id: u32) -> Self {
        Self {
            tx_id,
            incoming: FrameStream::new(),
        }
    }

    /// The id the MCU listens on.
    pub fn tx_id(&self) -> u32 {
        self.tx_id
    }

    /// The id the MCU answers on, and the only one this link accepts.
    pub fn rx_id(&self) -> u32 {
        self.tx_id + 1
    }

    /// Split an outgoing byte stream into CAN frames.
    ///
    /// This is the whole of the CAN framing: the stream is cut every eight bytes,
    /// because the bus carries eight bytes per frame. Where a message block begins
    /// and ends is the block header's business, not the bus's.
    pub fn frames<'a>(&'a self, bytes: &'a [u8]) -> impl Iterator<Item = CanFrame> + 'a {
        let tx_id = self.tx_id;
        bytes
            .chunks(CAN_DATA_BYTES)
            .map(move |chunk| CanFrame::new(tx_id, chunk).expect("chunks are at most eight bytes"))
    }

    /// Take one frame off the bus.
    ///
    /// Returns `false` for a frame that is not addressed to this link, so a socket
    /// shared with other traffic can pass everything it reads.
    pub fn accept(&mut self, frame: &CanFrame) -> bool {
        if frame.id() != self.rx_id() {
            return false;
        }
        self.incoming.push(frame.data());
        true
    }

    /// The next complete message frame, once its bytes have arrived.
    pub fn next(&mut self) -> Option<Frame> {
        self.incoming.next()
    }
}

/// A [`Device`] on a SocketCAN interface (`can0`, `/sys/class/net/*` names).
///
/// The link half above is unit tested; the socket half cannot be exercised in this
/// repository's test environment — it has no CAN interface and `vcan` needs
/// privileges to load — so it is covered by compilation only. Its shape is the
/// usual SocketCAN recipe: raw socket, bind to the interface index, kernel filter
/// for the MCU's answer id, one `can_frame` per read and write.
pub struct CanSerialDevice {
    socket: File,
    interface: String,
    uuid: [u8; 6],
    nodeid: u32,
    link: Mutex<CanSerialLink>,
    stopped: AtomicBool,
}

impl CanSerialDevice {
    /// Returns a short identifier for logging: `"can0:0x300"`.
    fn id(&self) -> String {
        format!(
            "{}:{:#x}",
            self.interface,
            self.link.lock().unwrap().tx_id()
        )
    }

    /// Open `interface` and bring the MCU `uuid` up as node `nodeid`.
    ///
    /// The node id lives in the host, not in the MCU: this sends Klipper's admin
    /// frame before any traffic, and the MCU starts answering on the node's
    /// arbitration id only after it has seen its UUID and new id.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] when the interface does not exist, the socket
    /// cannot be created (no CAN support), it cannot be bound or filtered, or the
    /// node id could not be handed over.
    pub fn open(interface: &str, uuid: [u8; 6], nodeid: u32) -> Result<Self, InterfaceError> {
        let link = CanSerialLink::for_node(nodeid);
        let index = interface_index(interface)?;
        let socket = open_socket()?;
        bind_interface(&socket, index)?;
        filter_answers(&socket, link.rx_id())?;

        let assignment = CanFrame::new(ADMIN_ID, &set_nodeid_payload(uuid, nodeid))?;
        (&socket)
            .write_all(&assignment.to_abi())
            .map_err(|e| InterfaceError::Other(format!("failed to send the node id: {e}")))?;

        info!(
            "can serial link ready on {interface}: node {nodeid} assigned, writing to {:#x}, \
             reading {:#x}",
            link.tx_id(),
            link.rx_id()
        );
        Ok(Self {
            socket,
            interface: interface.to_string(),
            uuid,
            nodeid,
            link: Mutex::new(link),
            stopped: AtomicBool::new(false),
        })
    }

    /// The UUID this device assigned a node id to.
    pub fn uuid(&self) -> [u8; 6] {
        self.uuid
    }

    /// The interface this device opened.
    pub fn interface(&self) -> &str {
        &self.interface
    }

    /// The CAN node id from the configuration.
    pub fn nodeid(&self) -> u32 {
        self.nodeid
    }

    /// Whether the MCU is expected to answer on this arbitration id.
    pub fn rx_id(&self) -> u32 {
        self.link.lock().unwrap().rx_id()
    }
}

impl fmt::Debug for CanSerialDevice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CanSerialDevice")
            .field("interface", &self.interface)
            .field("nodeid", &self.nodeid)
            .field("stopped", &self.stopped.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Device for CanSerialDevice {
    fn send(&self, frame: &Frame) -> Result<(), InterfaceError> {
        if self.stopped.load(Ordering::Relaxed) {
            return Err(InterfaceError::ConnectionLost);
        }

        // The message block goes out as a burst of CAN frames; the kernel queues
        // them, so a full bus costs latency rather than a lost frame.
        let bytes = frame.raw_bytes();
        trace!(
            "tx frame [{}]: {}",
            self.id(),
            bytes
                .iter()
                .map(|b| format!("{b:02x}"))
                .collect::<Vec<_>>()
                .join(" ")
        );
        let frames: Vec<CanFrame> = self.link.lock().unwrap().frames(&bytes).collect();
        for can_frame in &frames {
            (&self.socket)
                .write_all(&can_frame.to_abi())
                .map_err(|e| InterfaceError::SendError(format!("CAN write failed: {e}")))?;
        }
        debug!(
            "sent {} bytes as {} CAN frame(s) to {:#x}",
            bytes.len(),
            frames.len(),
            self.link.lock().unwrap().tx_id()
        );
        Ok(())
    }

    fn receive(&self) -> Option<Frame> {
        let mut raw = [0u8; CAN_FRAME_SIZE];
        loop {
            if self.stopped.load(Ordering::Relaxed) {
                return None;
            }
            if let Some(frame) = self.link.lock().unwrap().next() {
                return Some(frame);
            }

            match wait_readable(self.socket.as_raw_fd()) {
                Wait::Readable => {}
                Wait::Timeout => continue,
                Wait::Closed => return None,
                Wait::Retry => continue,
            }

            match (&self.socket).read(&mut raw) {
                Ok(read) if read >= CAN_FRAME_SIZE => {
                    let can_frame = CanFrame::from_abi(&raw);
                    if self.link.lock().unwrap().accept(&can_frame) {
                        trace!(
                            "rx frame [{}]: {}",
                            self.id(),
                            can_frame
                                .data()
                                .iter()
                                .map(|b| format!("{b:02x}"))
                                .collect::<Vec<_>>()
                                .join(" ")
                        );
                        debug!("received CAN frame of {} bytes", can_frame.data().len());
                    } else {
                        debug!("ignoring CAN frame for id {:#x}", can_frame.id());
                    }
                }
                // A short read cannot happen on a datagram socket; treat it as a
                // frame from someone else rather than guessing.
                Ok(read) => debug!("ignoring {read} byte CAN read"),
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(e) => {
                    warn!("CAN read on {} failed: {e}", self.interface);
                    return None;
                }
            }
        }
    }

    fn shutdown(&self) {
        self.stopped.store(true, Ordering::SeqCst);
    }
}

/// What a `poll` on the socket said.
enum Wait {
    Readable,
    Timeout,
    Closed,
    Retry,
}

/// Wait up to [`POLL_TIMEOUT_MS`] for the socket to have a frame.
fn wait_readable(fd: RawFd) -> Wait {
    let mut poll_fd = libc::pollfd {
        fd,
        events: libc::POLLIN,
        revents: 0,
    };
    // SAFETY: `poll_fd` is one initialised pollfd, as poll expects.
    let ready = unsafe { libc::poll(&mut poll_fd, 1, POLL_TIMEOUT_MS) };
    if ready < 0 {
        let err = std::io::Error::last_os_error();
        return if err.kind() == std::io::ErrorKind::Interrupted {
            Wait::Retry
        } else {
            warn!("poll on CAN socket failed: {err}");
            Wait::Closed
        };
    }
    if ready == 0 {
        return Wait::Timeout;
    }
    if poll_fd.revents & libc::POLLIN != 0 {
        Wait::Readable
    } else {
        // POLLERR/POLLHUP/POLLNVAL: the interface went away.
        warn!("CAN socket reported {:x}", poll_fd.revents);
        Wait::Closed
    }
}

/// The interface index of `interface`, or an error naming it.
fn interface_index(interface: &str) -> Result<libc::c_uint, InterfaceError> {
    let name = CString::new(interface)
        .map_err(|_| InterfaceError::Other(format!("invalid CAN interface '{interface}'")))?;
    // SAFETY: `name` is a NUL-terminated interface name.
    let index = unsafe { libc::if_nametoindex(name.as_ptr()) };
    if index == 0 {
        return Err(InterfaceError::Other(format!(
            "no CAN interface named '{interface}': {}",
            std::io::Error::last_os_error()
        )));
    }
    Ok(index)
}

/// Create a raw CAN socket.
fn open_socket() -> Result<File, InterfaceError> {
    // SAFETY: a plain socket(2) call; the returned descriptor is checked and then
    // owned by the `File` below.
    let fd = unsafe { libc::socket(libc::AF_CAN, libc::SOCK_RAW, CAN_RAW) };
    if fd < 0 {
        return Err(InterfaceError::Other(format!(
            "failed to open a CAN socket: {}",
            std::io::Error::last_os_error()
        )));
    }
    // SAFETY: `fd` is a fresh descriptor owned by this process.
    Ok(unsafe { File::from_raw_fd(fd) })
}

/// Bind the socket to one CAN interface.
fn bind_interface(socket: &File, index: libc::c_uint) -> Result<(), InterfaceError> {
    let address = SockAddrCan {
        can_family: libc::AF_CAN as libc::sa_family_t,
        can_ifindex: index as libc::c_int,
        can_addr: [0; 8],
    };
    // SAFETY: the address matches the length passed, both from the same struct.
    let rc = unsafe {
        libc::bind(
            socket.as_raw_fd(),
            &address as *const SockAddrCan as *const libc::sockaddr,
            std::mem::size_of::<SockAddrCan>() as libc::socklen_t,
        )
    };
    if rc < 0 {
        return Err(InterfaceError::Other(format!(
            "failed to bind the CAN socket: {}",
            std::io::Error::last_os_error()
        )));
    }
    Ok(())
}

/// Ask the kernel for only the frames the MCU sends: the answer id, classical.
fn filter_answers(socket: &File, rx_id: u32) -> Result<(), InterfaceError> {
    let filter = CanFilter {
        can_id: rx_id,
        // The mask has to include the flag bits, or standard frames never match.
        can_mask: CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG,
    };
    // SAFETY: the option value is a `can_filter` of the length passed.
    let rc = unsafe {
        libc::setsockopt(
            socket.as_raw_fd(),
            SOL_CAN_RAW,
            CAN_RAW_FILTER,
            &filter as *const CanFilter as *const libc::c_void,
            std::mem::size_of::<CanFilter>() as libc::socklen_t,
        )
    };
    if rc < 0 {
        return Err(InterfaceError::Other(format!(
            "failed to filter the CAN socket: {}",
            std::io::Error::last_os_error()
        )));
    }
    Ok(())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// An encoded Klipper frame to feed through the link.
    fn message(seq: u8, payload: &[u8]) -> Frame {
        Frame::new(seq, payload.to_vec())
    }

    #[test]
    fn test_node_ids_follow_klipper() {
        // Klipper: `txid = canbus_nodeid * 2 + 256`, answers on the next id.
        let link = CanSerialLink::for_node(2);
        assert_eq!(link.tx_id(), 0x104);
        assert_eq!(link.rx_id(), 0x105);

        let link = CanSerialLink::for_node(1);
        assert_eq!((link.tx_id(), link.rx_id()), (0x102, 0x103));
    }

    #[test]
    fn test_a_block_is_cut_into_eight_byte_frames() {
        let link = CanSerialLink::for_node(3);
        let bytes: Vec<u8> = (0..20).collect();

        let frames: Vec<CanFrame> = link.frames(&bytes).collect();

        // 20 bytes is two full frames and a remainder, all addressed to the MCU.
        assert_eq!(frames.len(), 3);
        assert_eq!(frames[0].data(), &bytes[0..8]);
        assert_eq!(frames[1].data(), &bytes[8..16]);
        assert_eq!(frames[2].data(), &bytes[16..20]);
        assert!(frames.iter().all(|f| f.id() == link.tx_id()));

        // A block that is an exact multiple of eight does not get an empty tail.
        assert_eq!(link.frames(&bytes[..16]).count(), 2);
    }

    #[test]
    fn test_incoming_frames_rebuild_the_block_stream() {
        let mut sender = CanSerialLink::for_node(5);
        let mut receiver = CanSerialLink::for_node(5);

        // The MCU answers on the next id, so a frame written to `tx_id` and read
        // back on `rx_id` models a round trip.
        let block = message(1, &[0xaa; 30]).raw_bytes();
        let frames: Vec<CanFrame> = sender
            .frames(&block)
            .map(|chunk| CanFrame::new(receiver.rx_id(), chunk.data()).unwrap())
            .collect();
        // 30 payload bytes plus the 5 byte block overhead: 35 bytes, five frames.
        assert_eq!(frames.len(), 5, "35 bytes need five CAN frames");

        // Nothing is complete until the last frame has arrived.
        for (i, frame) in frames.iter().enumerate() {
            assert!(receiver.accept(frame));
            let decoded = receiver.next();
            if i < frames.len() - 1 {
                assert!(decoded.is_none(), "frame {i} should not complete a block");
            } else {
                assert_eq!(decoded, Some(message(1, &[0xaa; 30])));
            }
        }
        assert_eq!(receiver.next(), None);
    }

    #[test]
    fn test_frames_for_another_node_are_ignored() {
        let mut link = CanSerialLink::for_node(5);
        let foreign = CanFrame::new(0x123, &[1, 2, 3]).unwrap();

        assert!(!link.accept(&foreign));
        assert_eq!(link.next(), None);

        // ... and a frame of ours behind it is still accepted.
        let ours = CanFrame::new(link.rx_id(), &message(2, &[9]).raw_bytes()).unwrap();
        assert!(link.accept(&ours));
        assert_eq!(link.next(), Some(message(2, &[9])));
    }

    #[test]
    fn test_a_frame_carries_at_most_eight_bytes() {
        assert!(CanFrame::new(0x100, &[0; CAN_DATA_BYTES]).is_ok());
        let err = CanFrame::new(0x100, &[0; CAN_DATA_BYTES + 1]).unwrap_err();
        assert!(err.to_string().contains("do not fit"), "{err}");
    }

    #[test]
    fn test_can_frame_layout_matches_the_kernel() {
        // One datagram per frame, so the sizes and field offsets are the ABI. This
        // checks the layout against itself; agreement with the kernel is what a
        // machine with a CAN interface would have to confirm.
        assert_eq!(CAN_FRAME_SIZE, 16);
        assert_eq!(std::mem::size_of::<SockAddrCan>(), CAN_FRAME_SIZE);
        assert_eq!(std::mem::size_of::<CanFilter>(), 8);

        let frame = CanFrame::new(0x1ab, &[1, 2, 3]).unwrap();
        let raw = frame.to_abi();
        assert_eq!(u32::from_ne_bytes([raw[0], raw[1], raw[2], raw[3]]), 0x1ab);
        assert_eq!(raw[4], 3);
        assert_eq!(&raw[8..11], &[1, 2, 3]);
        assert_eq!(CanFrame::from_abi(&raw), frame);

        // Extended ids are not part of this transport: only the low 11 bits are
        // read back.
        let mut extended = raw;
        extended[..4].copy_from_slice(&(0x1ab | CAN_EFF_FLAG).to_ne_bytes());
        assert_eq!(CanFrame::from_abi(&extended).id(), 0x1ab);
    }

    #[test]
    fn test_node_assignment_frame_matches_klipper() {
        // `klippy/serialhdl.py`: CMD_SET_NODEID, the UUID most significant byte
        // first, then the node id.
        let payload = set_nodeid_payload([0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc], 2);
        assert_eq!(payload, [0x01, 0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc, 0x02]);
        assert_eq!(set_nodeid_payload([0; 6], 1)[7], 1);
    }

    #[test]
    fn test_open_reports_a_missing_interface() {
        // The interface is checked before anything is sent, so this needs no CAN
        // bus to assign a node on.
        let err = CanSerialDevice::open("can99", [0; 6], 2).unwrap_err();
        assert!(
            err.to_string().contains("no CAN interface named 'can99'"),
            "{err}"
        );
    }
}
