//! Klipper's message blocks carried over CAN — the "can serial" link.
//!
//! Klipper does not put its protocol *on* CAN: the firmware takes the same byte
//! stream it would write to a serial port (length, sequence, CRC, SYNC — see
//! [`Frame`]) and chops it into CAN frames (`src/generic/canserial.c`).
//! The bus contributes exactly two things, the arbitration id that picks a node
//! and a fixed number of data bytes per frame, which is why this module sits next
//! to the serial device: reassembly is the same [`FrameStream`] every byte-stream
//! device uses, and the CAN part is a chunking rule plus addressing.
//!
//! How many bytes a frame carries is the link's *wire* ([`CanWire`]): a classical
//! CAN link cuts the stream every 8 bytes, a CAN FD one every 64. The two are not
//! interchangeable — the kernel's frame structs are 16 and 72 bytes, and a socket
//! reads a datagram by its length — so a link says which wire it is once, and
//! everything downstream of it is sized from that.
//!
//! Addressing follows Klipper (`klippy/serialhdl.py`): a node id maps to an
//! arbitration id through the mapping every transport shares
//! ([`crate::core::klippy::assign::arb_id`]), the host writes to that id, and the
//! MCU answers on the next one. Both wires use 11-bit ids, never extended ones.
//!
//! ```text
//!   send(Frame) ─► bytes ─► 8/64 byte CAN frames (id = tx_id) ──────► can0
//!   receive()   ◄── FrameStream ◄── data bytes (id = tx_id + 1) ◄─ can0
//! ```
//!
//! The bus carries one more conversation, which is *not* the byte stream: the
//! **admin** broadcast on id `0x3f0`, which is how a board that has no node id is
//! found and then given one. It is the same firmware module (`src/generic/canserial.c`,
//! "admin command handling") and the same sockets, so it lives here too: see
//! [`CanbusAdminSocket`], which asks every unassigned board to identify itself and
//! reads the answers.
//!
//! The byte stream carries an admin channel of *its own*, and it is the one the
//! port-level commands travel: the units that name and reset the devices behind a
//! bridge's ports ([`crate::core::klippy::interface::devices::serial`]) go out as
//! ordinary units on the stream, which is what
//! [`CanSerialDevice::send_set_port_node_id`] and the two resets below write. A
//! bridge multiplexes N downstream devices onto one upstream link, so a
//! port-level command has to be *inside* the stream — with the mux header saying
//! which address the unit belongs to — rather than a frame of its own; the
//! id-`0x3f0` admin conversation above is a separate CAN channel and stays
//! outside it.
//!
//! # Naming
//!
//! This is the *can serial* transport: Klipper's serial link, carried over CAN.
//! Type names therefore say `CanSerial`, and the name `Canbus` is kept for an
//! interface that speaks the CAN protocol itself rather than borrowing the bus as
//! a wire — which is what [`CanbusAdminSocket`] does with the admin broadcast.
//!
//! Configuration keys are a different matter: they follow Klipper's `[mcu]`
//! vocabulary (`uuid`, `canbus_interface`, and `canbus_nodeid` as Klipper's
//! own console and `serialhdl.connect_canbus` spell it — `uuid` is this
//! crate's canonical name for the key upstream writes `canbus_uuid`), because
//! those describe the printer's wiring rather than this implementation, and a
//! Klipper config should keep working.

use crate::core::klippy::assign;
use crate::core::klippy::config::mcu::MAX_MCU_ID;
use crate::core::klippy::frame::{Frame, FrameStream};
use crate::core::klippy::interface::devices::serial::{
    admin_unit, encode_header, request_bootloader_unit, set_port_node_id, tunnel_block,
    TunnelRegistry, TunnelSink, UnitStream, CMD_REQUEST_BOOTLOADER, CMD_RESET_PORT,
    CMD_RESTART_DEVICE, CMD_SET_PORT_TUNNEL,
};
use crate::core::klippy::interface::error::InterfaceError;
use crate::core::klippy::interface::{describe_frame, Device};
use std::ffi::CString;
use std::fmt;
use std::fs::File;
use std::io::{Read, Write};
#[cfg(test)]
use std::os::fd::OwnedFd;
use std::os::fd::{AsRawFd, FromRawFd, RawFd};
#[cfg(test)]
use std::os::unix::net::UnixStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};
use tracing::{debug, info, trace, warn};

/// Data bytes in one classical CAN frame.
pub const CAN_DATA_BYTES: usize = 8;

/// Data bytes in one CAN FD frame.
pub const CAN_FD_DATA_BYTES: usize = 64;

/// Size of the kernel's `struct can_frame`, which is what a classical raw socket
/// reads and writes one datagram at a time.
pub(crate) const CAN_FRAME_SIZE: usize = 16;

/// Size of the kernel's `struct canfd_frame`: four bytes of header, then 64 data
/// bytes. Its `len` sits at the same offset 4 and its data at the same offset 8 as
/// the classical struct's, so one layout serves both and only the length differs.
pub(crate) const CAN_FD_FRAME_SIZE: usize = 72;

/// Klipper's admin arbitration id, and the commands and answers that travel on
/// it (`klippy/serialhdl.py`, `src/generic/canserial.c`): the one that tells an
/// unassigned MCU which node id to take, the broadcast that asks every board
/// without one to answer, the answer itself, and the application id CanBoot
/// reports in it.
const ADMIN_ID: u32 = 0x3f0;
const CMD_SET_NODEID: u8 = 0x01;
const CMD_QUERY_UNASSIGNED: u8 = 0x00;
const RESP_NEED_NODEID: u8 = 0x20;
const CMD_SET_CANBOOT_NODEID: u8 = 0x11;

/// How long a receive waits for a frame before rechecking the stop flag.
const POLL_TIMEOUT: Duration = Duration::from_millis(100);

/// How long one write may wait for the socket to take a frame.
///
/// A raw CAN socket blocks once its send buffer is full — a bus whose peer never
/// acknowledges, or one held down by another process, stops draining it — and a
/// write without a deadline then waits forever. The kernel turns this option into
/// `EAGAIN`/`EWOULDBLOCK` once it expires, which every write here reports as an
/// [`InterfaceError::SendError`] rather than retrying.
const SEND_TIMEOUT: Duration = Duration::from_millis(100);

// From `linux/can.h` and `linux/can/raw.h`; `libc` does not expose these.
const CAN_SFF_MASK: u32 = 0x7ff;
const CAN_EFF_FLAG: u32 = 0x8000_0000;
const CAN_RTR_FLAG: u32 = 0x4000_0000;
const CAN_RAW: libc::c_int = 1;
const SOL_CAN_RAW: libc::c_int = 101;
const CAN_RAW_FILTER: libc::c_int = 1;
const CAN_RAW_FD_FRAMES: libc::c_int = 5;

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

/// The shape of a CAN link: which kernel frame struct it carries, and with it how
/// many data bytes one frame holds.
///
/// A link is one wire or the other, never both. The kernel reads a datagram by its
/// length — `can_frame` at 16 bytes, `canfd_frame` at 72 — and a socket that has
/// not asked for CAN FD drops FD frames rather than failing loudly, so which wire a
/// link speaks is decided once, when its socket is opened
/// ([`CanSerialDevice::open`] for the classical wire,
/// [`CanSerialDevice::open_with_wire`] for either).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanWire {
    /// Classical CAN: `struct can_frame`, eight data bytes.
    Classical,
    /// CAN FD: `struct canfd_frame`, sixty-four data bytes.
    Fd,
}

impl CanWire {
    /// How many data bytes one frame on this wire carries.
    pub fn data_bytes(self) -> usize {
        match self {
            Self::Classical => CAN_DATA_BYTES,
            Self::Fd => CAN_FD_DATA_BYTES,
        }
    }

    /// How many bytes this wire's kernel frame struct is: the size of one
    /// datagram, read or written.
    fn frame_size(self) -> usize {
        match self {
            Self::Classical => CAN_FRAME_SIZE,
            Self::Fd => CAN_FD_FRAME_SIZE,
        }
    }

    /// The wire's name, as logs and errors spell it.
    fn name(self) -> &'static str {
        match self {
            Self::Classical => "classical CAN",
            Self::Fd => "CAN FD",
        }
    }
}

/// One frame as the kernel lays it out, ready to be written as a single datagram.
///
/// Both wires share the layout — native-endian id, the length at offset 4, the data
/// from offset 8 — and differ only in length, so the bytes are held in the larger
/// struct and the wire says how much of it goes out. Keeping the length with the
/// bytes is what stops a classical frame from leaving as 72 bytes, which the kernel
/// would read back as a CAN FD frame.
#[derive(Debug)]
pub(crate) struct AbiFrame {
    wire: CanWire,
    bytes: [u8; CAN_FD_FRAME_SIZE],
}

impl AbiFrame {
    /// The datagram: `wire`'s own number of bytes.
    pub(crate) fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.wire.frame_size()]
    }
}

/// One CAN frame: an 11-bit arbitration id and up to [`CAN_FD_DATA_BYTES`] data
/// bytes.
///
/// The type holds the largest frame either wire has. How many of those bytes a
/// frame may actually carry is the wire's business, and is enforced where a frame
/// meets one ([`CanFrame::to_abi`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFrame {
    id: u32,
    len: u8,
    data: [u8; CAN_FD_DATA_BYTES],
}

impl CanFrame {
    /// A frame carrying `data`, which has to fit in one CAN frame: sixty-four
    /// bytes, the larger of the two wires'.
    pub fn new(id: u32, data: &[u8]) -> Result<Self, InterfaceError> {
        if data.len() > CAN_FD_DATA_BYTES {
            return Err(InterfaceError::Other(format!(
                "{} bytes do not fit in one CAN frame",
                data.len()
            )));
        }
        let mut bytes = [0u8; CAN_FD_DATA_BYTES];
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

    /// The kernel's layout for `wire`: native-endian id, length, reserved bytes,
    /// then the data — sixteen bytes for classical CAN, seventy-two for CAN FD.
    ///
    /// # Errors
    /// A frame whose data does not fit `wire` has no such layout, its length byte
    /// being unable to describe it, and is refused rather than truncated or padded.
    /// This is where the two layouts are kept from crossing.
    pub(crate) fn to_abi(self, wire: CanWire) -> Result<AbiFrame, InterfaceError> {
        if self.len as usize > wire.data_bytes() {
            return Err(InterfaceError::Other(format!(
                "{} bytes do not fit a {} frame",
                self.len,
                wire.name()
            )));
        }
        let mut bytes = [0u8; CAN_FD_FRAME_SIZE];
        bytes[..4].copy_from_slice(&self.id.to_ne_bytes());
        bytes[4] = self.len;
        bytes[8..wire.frame_size()].copy_from_slice(&self.data[..wire.data_bytes()]);
        Ok(AbiFrame { wire, bytes })
    }

    /// Parse the kernel's layout for `wire`. Ids longer than 11 bits are extended
    /// frames, which nothing here uses.
    pub(crate) fn from_abi(wire: CanWire, raw: &[u8; CAN_FD_FRAME_SIZE]) -> Self {
        let id = u32::from_ne_bytes([raw[0], raw[1], raw[2], raw[3]]) & CAN_SFF_MASK;
        let len = raw[4].min(wire.data_bytes() as u8);
        let mut data = [0u8; CAN_FD_DATA_BYTES];
        data[..wire.data_bytes()].copy_from_slice(&raw[8..wire.frame_size()]);
        Self { id, len, data }
    }
}

/// One CAN frame as the bus carries it: its arbitration id, then its data bytes —
/// a *slice* of a message block rather than a block of its own.
///
/// The block is described with [`describe_frame`](crate::core::klippy::interface::describe_frame) once its
/// slices are together again; this is the other half of the story, and the two
/// are logged separately because they are different things. A block small enough
/// to fit one CAN frame still gets both lines.
fn describe_can_frame(id: u32, data: &[u8]) -> String {
    format!(
        "id {id:#x} data {}",
        crate::core::klippy::interface::hex_runs(data)
    )
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
    /// The same bytes, cut into whole **units**: a tunnel unit's block is not a
    /// klipper frame, so it never appears in `incoming`, and this is where the
    /// flashing tunnel's uplink is found ([`CanSerialLink::next_tunnel`]).
    units: UnitStream,
}

impl CanSerialLink {
    /// The link to the node `nodeid`, using the shared arbitration-id mapping
    /// ([`crate::core::klippy::assign::arb_id`]).
    ///
    /// `None` for a node id outside `1..=MAX_MCU_ID` (`0xff`), the range the wire
    /// format can address.
    pub fn for_node(nodeid: u32) -> Option<Self> {
        assign::arb_id(nodeid).map(Self::with_arbitration_id)
    }

    /// The link whose MCU listens on `tx_id`.
    pub fn with_arbitration_id(tx_id: u32) -> Self {
        Self {
            tx_id,
            incoming: FrameStream::new(),
            units: UnitStream::new(),
        }
    }

    /// The id the MCU listens on.
    pub fn tx_id(&self) -> u32 {
        self.tx_id
    }

    /// The id the MCU answers on, and the only one this link accepts.
    ///
    /// One past the listener id — the same `+1` as [`assign::reply_arb_id`],
    /// which the tests below pin over the whole id range. The link stores the
    /// resolved arbitration id (`tx_id`), not the node id, so the step is
    /// spelled here instead of re-derived from the mapping.
    pub fn rx_id(&self) -> u32 {
        self.tx_id + 1
    }

    /// Split an outgoing byte stream into CAN frames for `wire`.
    ///
    /// This is the whole of the CAN framing: the stream is cut every
    /// [`wire.data_bytes()`](CanWire::data_bytes) bytes, because that is how many
    /// the bus carries per frame. Where a message block begins and ends is the
    /// block header's business, not the bus's.
    pub fn frames<'a>(
        &'a self,
        bytes: &'a [u8],
        wire: CanWire,
    ) -> impl Iterator<Item = CanFrame> + 'a {
        let tx_id = self.tx_id;
        bytes.chunks(wire.data_bytes()).map(move |chunk| {
            CanFrame::new(tx_id, chunk).expect("a chunk is at most one frame's worth")
        })
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
        // The same bytes go to the unit stream too: the frame path cannot see a
        // tunnel unit (its first byte is no klipper length), and a tunnel unit
        // must not be missed by a reader that only reassembles frames.
        self.units.feed(frame.data());
        true
    }

    /// The next complete message frame, once its bytes have arrived.
    pub fn next_frame(&mut self) -> Option<Frame> {
        self.incoming.next_frame()
    }

    /// The next whole tunnel unit as `(arb, block)`, once its bytes have arrived.
    ///
    /// The link's arbitration id says the unit is this node's; the unit header's
    /// own id names which device behind a bridge it is for, and is the key a
    /// tunnel listener registers under. Ordinary klipper units the unit stream
    /// cuts are dropped here — [`CanSerialLink::next_frame`] is their reader.
    pub fn next_tunnel(&mut self) -> Option<(u32, Vec<u8>)> {
        loop {
            let (arb, unit) = self.units.take_unit_with_arb()?;
            if let Some(block) = tunnel_block(&unit) {
                return Some((arb, block.to_vec()));
            }
        }
    }
}

/// The broadcast that asks every board without a node id to identify itself.
///
/// [`CanSerialDevice`] addresses one node; this frame addresses none, which is
/// the point: a board that has never been given a node id cannot be addressed. It
/// goes out on the admin id with the query as its only byte, exactly as
/// `scripts/canbus_query.py` sends it.
pub fn query_unassigned_frame() -> CanFrame {
    CanFrame::new(ADMIN_ID, &[CMD_QUERY_UNASSIGNED])
        .expect("the query is one byte, which fits in a CAN frame")
}

/// The admin frame that asks the board with `uuid` to leave its application and
/// park in its bootloader.
///
/// The request byte, then the UUID most significant byte first — the seven data
/// bytes `cansend can0 3f0#02<uuid>` writes, and the only shape the device side
/// acts on (`can_process_request_bootloader` refuses a frame shorter than seven
/// bytes and one whose bytes 1..7 are another board's UUID,
/// `src/generic/canserial.c`). It is addressed to no node: the board still runs
/// an application whose node id it is about to lose, so the bus broadcast is
/// what reaches it — the same frame `scripts/flash_can.py`'s `_jump_to_bootloader`
/// sends.
///
/// Nothing answers this: a board that took the request stops answering the id it
/// had, which is what the caller is waiting for.
pub fn request_bootloader_frame(uuid: [u8; 6]) -> CanFrame {
    let mut data = [0u8; 7];
    data[0] = CMD_REQUEST_BOOTLOADER;
    data[1..].copy_from_slice(&uuid);
    CanFrame::new(ADMIN_ID, &data).expect("the request is seven bytes, which fits in a CAN frame")
}

/// One board's answer to [`query_unassigned_frame`]: the UUID it is remembered
/// by, and the application it runs.
///
/// The firmware answers with `RESP_NEED_NODEID`, the six UUID bytes most
/// significant first, and — in the builds that send it — a seventh data byte
/// naming the application (`src/generic/canserial.c`,
/// `docs/CANBUS_protocol.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnassignedNode {
    uuid: [u8; 6],
    application: Option<u8>,
}

impl UnassignedNode {
    /// The board a frame reports, or `None` for a frame that is not an answer.
    ///
    /// What makes a frame an answer is its shape: the admin answer id, at least
    /// seven data bytes, and `RESP_NEED_NODEID` in the first of them. Nothing
    /// else is checked — in particular a UUID of six zero bytes is reported
    /// rather than treated as unset, because the protocol has no such value and
    /// Klipper's own tool prints one too — and the application byte is optional.
    pub fn from_reply(frame: &CanFrame) -> Option<Self> {
        let data = frame.data();
        if frame.id() != ADMIN_ID + 1 || data.len() < 7 || data[0] != RESP_NEED_NODEID {
            return None;
        }
        let mut uuid = [0u8; 6];
        uuid.copy_from_slice(&data[1..7]);
        Some(Self {
            uuid,
            application: data.get(7).copied(),
        })
    }

    /// The UUID, most significant byte first.
    pub fn uuid(&self) -> [u8; 6] {
        self.uuid
    }

    /// The UUID as Klipper writes it: twelve hex digits (`%012x`), leading
    /// zeros and all.
    pub fn uuid_hex(&self) -> String {
        format!("{:012x}", self.uuid_number())
    }

    /// The UUID as one number, summed the way Klipper's own tool sums it.
    ///
    /// This is what pins the byte order: the byte after the answer's first is
    /// the *most* significant one.
    fn uuid_number(&self) -> u64 {
        self.uuid
            .iter()
            .fold(0u64, |value, byte| (value << 8) | u64::from(*byte))
    }

    /// The application byte the board sent, if it sent one.
    pub fn application(&self) -> Option<u8> {
        self.application
    }

    /// The application's name, as Klipper's tool names it in its `Found …` line.
    ///
    /// A board that sends no application byte — seven data bytes, the shape the
    /// protocol had before the byte existed — is named Klipper, which is what
    /// upstream's tool reads it as; an id that means neither Klipper nor CanBoot
    /// is reported as `Unknown` rather than guessed at.
    pub fn application_name(&self) -> &'static str {
        match self.application {
            None | Some(CMD_SET_NODEID) => "Klipper",
            Some(CMD_SET_CANBOOT_NODEID) => "CanBoot",
            Some(_) => "Unknown",
        }
    }
}

/// The boards one scan heard, in the order they first answered.
#[derive(Debug, Default)]
pub struct UnassignedScan {
    nodes: Vec<UnassignedNode>,
}

impl UnassignedScan {
    /// A scan that has heard nothing yet.
    pub fn new() -> Self {
        Self::default()
    }

    /// Take one frame: the node it reports, the first time that UUID is heard.
    ///
    /// A board answers with retries until the bus takes its frame
    /// (`can_process_query_unassigned` loops on `canbus_send`), so the same
    /// board can answer twice; the second frame is `None` here, and the board is
    /// listed once.
    pub fn feed(&mut self, frame: &CanFrame) -> Option<UnassignedNode> {
        let node = UnassignedNode::from_reply(frame)?;
        if self.nodes.iter().any(|seen| seen.uuid == node.uuid) {
            return None;
        }
        self.nodes.push(node);
        Some(node)
    }

    /// The boards found so far, in the order they first answered.
    pub fn nodes(&self) -> &[UnassignedNode] {
        &self.nodes
    }

    /// How many boards answered.
    pub fn total(&self) -> usize {
        self.nodes.len()
    }
}

/// The bus a scan runs on: a socket in production, a scripted list of answers in
/// tests, which is what lets the query window be exercised without a CAN
/// interface or a real clock.
///
/// The clock is part of the bus rather than a parameter of its own because from
/// the window's point of view the two are the same thing: what consumes the
/// window is the *waiting*, so a fake bus has to advance its clock by exactly
/// what the scan asked it to wait for.
pub trait AdminBus {
    /// The current time on this bus's clock.
    fn now(&self) -> Instant;

    /// Send one admin frame.
    fn send(&mut self, frame: &CanFrame) -> Result<(), InterfaceError>;

    /// Read one frame, waiting at most `wait` for it.
    ///
    /// `None` means nothing arrived in that time; an error means the bus failed.
    fn receive(&mut self, wait: Duration) -> Result<Option<CanFrame>, InterfaceError>;
}

/// Ask every board with no node id to identify itself, and read the answers for
/// `window`.
///
/// This is `query_unassigned` in `scripts/canbus_query.py`, and the window is the
/// same kind of thing it is there — a *deadline*, measured from the query, not an
/// idle timeout: every wait is shortened to what is left of it, and the scan ends
/// when that runs out, whatever has arrived by then. `report` is called as each
/// board is first heard, so a front-end can print answers while the window is
/// still open; the same boards come back in the returned scan.
pub fn query_unassigned(
    bus: &mut impl AdminBus,
    window: Duration,
    mut report: impl FnMut(UnassignedNode),
) -> Result<UnassignedScan, InterfaceError> {
    let started = bus.now();
    bus.send(&query_unassigned_frame())?;
    let mut scan = UnassignedScan::new();
    loop {
        let remaining = window.saturating_sub(bus.now().saturating_duration_since(started));
        if remaining.is_zero() {
            break;
        }
        let Some(frame) = bus.receive(remaining)? else {
            // The wait for what was left of the window came back empty, so
            // nothing more can arrive inside it.
            break;
        };
        if let Some(node) = scan.feed(&frame) {
            report(node);
        }
    }
    Ok(scan)
}

/// A [`Device`] on a SocketCAN interface (`can0`, `/sys/class/net/*` names).
///
/// The link half above is unit tested; the socket half cannot be exercised in this
/// repository's test environment — it has no CAN interface and `vcan` needs
/// privileges to load — so it is covered by compilation only. Its shape is the
/// usual SocketCAN recipe: raw socket, bind to the interface index, kernel filter
/// for the MCU's answer id, one frame per read and write, sized by [`CanWire`].
pub struct CanSerialDevice {
    socket: File,
    interface: String,
    uuid: [u8; 6],
    nodeid: u32,
    /// The link's shape: what one frame holds and how large a datagram is.
    wire: CanWire,
    link: Mutex<CanSerialLink>,
    /// The tunnel listeners on this link, keyed by the arbitration id their
    /// units are headed with — the uplink half of a flashing tunnel
    /// ([`CanSerialDevice::set_tunnel_sink`]).
    tunnels: TunnelRegistry,
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

    /// Open `interface` and bring the MCU `uuid` up as node `nodeid` — the
    /// classical CAN link.
    ///
    /// The node id lives in the host, not in the MCU: this sends Klipper's admin
    /// frame before any traffic, and the MCU starts answering on the node's
    /// arbitration id only after it has seen its UUID and new id. This is
    /// [`CanSerialDevice::open_with_wire`] on [`CanWire::Classical`].
    ///
    /// # Errors
    /// The same as [`CanSerialDevice::open_with_wire`].
    pub fn open(interface: &str, uuid: [u8; 6], nodeid: u32) -> Result<Self, InterfaceError> {
        Self::open_with_wire(interface, uuid, nodeid, CanWire::Classical)
    }

    /// Open `interface` on `wire` and bring the MCU `uuid` up as node `nodeid`.
    ///
    /// The wire is what the link is *sized* by: a CAN FD link carries sixty-four
    /// data bytes per frame and every datagram on its socket is seventy-two bytes.
    /// It is therefore not a preference but a pairing with the other end — an MCU
    /// built without CAN FD and a host that opened the FD wire (or the other way
    /// round) cannot hear each other, because the kernel drops the frames instead
    /// of reporting them.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] when the node id is outside `1..=MAX_MCU_ID`
    /// (`0xff`), the interface does not exist, the socket cannot be created (no
    /// CAN support), it cannot be bound or filtered, CAN FD cannot be enabled on
    /// it, or the node id could not be handed over.
    pub fn open_with_wire(
        interface: &str,
        uuid: [u8; 6],
        nodeid: u32,
        wire: CanWire,
    ) -> Result<Self, InterfaceError> {
        // Checked before the socket is opened: an unaddressable node id is a
        // configuration mistake, not a bus one, and the bus is not touched for it.
        let tx_id = assign::arb_id(nodeid).ok_or_else(|| {
            InterfaceError::Other(format!(
                "node id {nodeid} is outside the CAN range 1..={MAX_MCU_ID}"
            ))
        })?;
        let link = CanSerialLink::with_arbitration_id(tx_id);
        let index = interface_index(interface)?;
        let socket = open_socket(wire)?;
        bind_interface(&socket, index)?;
        filter_answers(&socket, link.rx_id())?;

        let assignment = CanFrame::new(ADMIN_ID, &set_nodeid_payload(uuid, nodeid))?;
        (&socket)
            .write_all(assignment.to_abi(wire)?.as_bytes())
            .map_err(|e| InterfaceError::Other(format!("failed to send the node id: {e}")))?;

        info!(
            "can serial link ready on {interface} ({}): node {nodeid} assigned, writing to \
             {:#x}, reading {:#x}",
            wire.name(),
            link.tx_id(),
            link.rx_id()
        );
        Ok(Self {
            socket,
            interface: interface.to_string(),
            uuid,
            nodeid,
            wire,
            link: Mutex::new(link),
            tunnels: TunnelRegistry::new(),
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

    /// This MCU's MCU-layer id, as the configuration assigned it: on this CAN
    /// link it is the node id, addressed by the shared mapping
    /// ([`crate::core::klippy::assign::arb_id`]).
    pub fn nodeid(&self) -> u32 {
        self.nodeid
    }

    /// Whether the MCU is expected to answer on this arbitration id.
    pub fn rx_id(&self) -> u32 {
        self.link.lock().unwrap().rx_id()
    }

    /// Write `bytes` to this node as a burst of CAN frames: the byte-stream half
    /// of [`Device::send`], shared with the port-level commands below.
    ///
    /// The kernel queues the frames, so a full bus costs latency rather than a
    /// lost one. What is logged is the block formatter's view of the bytes, and
    /// a unit — a mux header followed by a block, rather than a block of its own
    /// — falls back to plain runs there ([`describe_frame`]).
    fn send_bytes(&self, bytes: &[u8]) -> Result<(), InterfaceError> {
        if self.stopped.load(Ordering::Relaxed) {
            return Err(InterfaceError::ConnectionLost);
        }

        trace!("tx frame [{}]: {}", self.id(), describe_frame(bytes));
        let frames: Vec<CanFrame> = self.link.lock().unwrap().frames(bytes, self.wire).collect();
        for can_frame in &frames {
            // The block's bytes as the bus sees them, one CAN frame at a time.
            trace!(
                "tx can frame [{}]: {}",
                self.id(),
                describe_can_frame(can_frame.id(), can_frame.data())
            );
            (&self.socket)
                .write_all(can_frame.to_abi(self.wire)?.as_bytes())
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

    /// Tell the bridge which id the device behind port `index` answers to:
    /// [`set_port_node_id`]'s unit, byte for byte, cut into CAN frames.
    ///
    /// The port-level commands travel a bridge's admin channel, and on CAN that
    /// channel is *inside the byte stream* — the unit's header
    /// ([`ADMIN_LISTEN`](crate::core::klippy::interface::devices::serial::ADMIN_LISTEN),
    /// `0x3f0`) is what names it. There is no separate CAN frame shape for them,
    /// for three reasons:
    ///
    /// - a bridge multiplexes N downstream devices onto one upstream link and
    ///   has to say which of them a unit is for, which is exactly what the mux
    ///   header is; without it the CAN side could not tell whose frame it read;
    /// - reusing the serial side's unit means no new frame shape on the wire, and
    ///   no collision with the CAN admin broadcast above: that one is a
    ///   single-frame conversation on arbitration id `0x3f0`, a different channel
    ///   from these bytes, so the port-level family here cannot be read as that
    ///   channel's `REQUEST_BOOTLOADER`;
    /// - serial ports, CAN links and the host library then carry one and the same
    ///   unit ([`set_port_node_id`] is the builder all three call).
    ///
    /// # Errors
    /// The same as [`Device::send`]: a stopped device is a lost connection, and a
    /// failed socket write is an [`InterfaceError::SendError`].
    pub fn send_set_port_node_id(
        &self,
        index: u8,
        mcu_id: u8,
        flags: u8,
    ) -> Result<(), InterfaceError> {
        self.send_bytes(&set_port_node_id(index, mcu_id, flags))
    }

    /// Ask the device behind bridge port `index` to restart its firmware:
    /// [`CMD_RESTART_DEVICE`]'s unit, on the byte stream like
    /// [`CanSerialDevice::send_set_port_node_id`].
    ///
    /// # Errors
    /// The same as [`Device::send`].
    pub fn send_restart_device(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_bytes(&admin_unit(&[CMD_RESTART_DEVICE, index]))
    }

    /// Ask the bridge to pulse the lines of port `index`: [`CMD_RESET_PORT`]'s
    /// unit, on the byte stream like [`CanSerialDevice::send_set_port_node_id`].
    ///
    /// # Errors
    /// The same as [`Device::send`].
    pub fn send_reset_port(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_bytes(&admin_unit(&[CMD_RESET_PORT, index]))
    }

    /// Ask the device behind bridge port `index` to enter its bootloader:
    /// [`CMD_REQUEST_BOOTLOADER`]'s unit — the flashing tool's command — on the
    /// byte stream like [`CanSerialDevice::send_set_port_node_id`].
    ///
    /// # Errors
    /// The same as [`Device::send`].
    pub fn send_request_bootloader(
        &self,
        index: u8,
        uuid: Option<[u8; 6]>,
    ) -> Result<(), InterfaceError> {
        self.send_bytes(&request_bootloader_unit(index, uuid))
    }

    /// Toggle the flashing tunnel of bridge port `index`:
    /// [`CMD_SET_PORT_TUNNEL`]'s unit, on the byte stream like
    /// [`CanSerialDevice::send_set_port_node_id`].
    ///
    /// # Errors
    /// The same as [`Device::send`].
    pub fn send_set_port_tunnel(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_bytes(&admin_unit(&[CMD_SET_PORT_TUNNEL, index]))
    }

    /// Write one unit under an arbitrary arbitration id: the checked two-byte
    /// header for `arb` ([`encode_header`]), then `payload` exactly as given,
    /// cut into CAN frames like any byte stream
    /// ([`CanSerialDevice::send_bytes`]).
    ///
    /// This is the entry the other transports' `send_unit` mirrors, and what a
    /// tunnel adapter writes whole katapult blocks through: `payload` travels
    /// as it came, so the caller owns the framing.
    ///
    /// # Errors
    /// The same as [`Device::send`].
    pub(crate) fn send_unit(&self, arb: u32, payload: &[u8]) -> Result<(), InterfaceError> {
        let mut unit = encode_header(arb);
        unit.extend_from_slice(payload);
        self.send_bytes(&unit)
    }

    /// Route this link's tunnel units headed with `arb` to `sink`: the uplink
    /// half of a flashing tunnel, handed over whole.
    ///
    /// `arb` is the id the upstream tunnel units carry — a device behind a
    /// bridge port answers under `reply_arb_id(mcu_id)`, and the bridge heads
    /// its bootloader's blocks with it. Registering the same id twice replaces
    /// the first sink; [`Self::clear_tunnel_sink`] removes it, and so does
    /// dropping the receiver ([`TunnelSink::channel`]). Units no sink is
    /// registered for are dropped, exactly as before there were listeners.
    pub fn set_tunnel_sink(&self, arb: u32, sink: TunnelSink) {
        self.tunnels.register(arb, sink);
    }

    /// Stop routing `arb`'s tunnel units to the sink registered for it.
    pub fn clear_tunnel_sink(&self, arb: u32) {
        self.tunnels.clear(arb);
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
        self.send_bytes(&frame.raw_bytes())
    }

    fn receive(&self) -> Option<Frame> {
        // The largest datagram either wire delivers; a classical read fills the
        // first sixteen bytes of it.
        let mut raw = [0u8; CAN_FD_FRAME_SIZE];
        loop {
            if self.stopped.load(Ordering::Relaxed) {
                return None;
            }
            // A tunnel unit that arrived for this link goes to its listener
            // whole; the frame path cannot see it (its first byte is no klipper
            // length). With none, it is dropped.
            //
            // The result is taken into a local first so the guard is gone before
            // the body runs: an `if let` scrutinee temporary lives until the end
            // of the `if let` (Rust 2021), and the body's `self.id()` locks the
            // same mutex — that would re-enter it on this thread and deadlock.
            let tunnel = self.link.lock().unwrap().next_tunnel();
            if let Some((arb, block)) = tunnel {
                if !self.tunnels.deliver(arb, &block) {
                    debug!(
                        "[{}]: no tunnel listener for {arb:#x}; dropping {}-byte tunnel block",
                        self.id(),
                        block.len()
                    );
                }
            }
            // The slices came together: the block is logged the way the serial
            // port logs one — however many CAN frames it took, even one.
            // Same reason as above: `self.id()` below locks the link again.
            let frame = self.link.lock().unwrap().next_frame();
            if let Some(frame) = frame {
                trace!(
                    "rx frame [{}]: {}",
                    self.id(),
                    describe_frame(&frame.raw_bytes())
                );
                return Some(frame);
            }

            match wait_readable(self.socket.as_raw_fd(), POLL_TIMEOUT) {
                Wait::Readable => {}
                Wait::Timeout => continue,
                Wait::Closed => return None,
                Wait::Retry => continue,
            }

            match (&self.socket).read(&mut raw) {
                Ok(read) if read >= self.wire.frame_size() => {
                    let can_frame = CanFrame::from_abi(self.wire, &raw);
                    if self.link.lock().unwrap().accept(&can_frame) {
                        // One CAN frame's worth of the block, as the bus carried
                        // it; the assembled block is logged above, on its way out.
                        trace!(
                            "rx can frame [{}]: {}",
                            self.id(),
                            describe_can_frame(can_frame.id(), can_frame.data())
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

/// The device on a **wire a test can read**: a socket pair stands in for the CAN
/// socket, and the returned end carries the records the device writes, read back
/// with [`read_wire`].
///
/// Opening a real CAN interface needs privileges this repository's test
/// environment does not have, and the port-level commands use the device's write
/// half only — a socket pair serves that, the same way the admin scan takes an
/// [`AdminBus`] rather than a socket.
#[cfg(test)]
impl CanSerialDevice {
    /// A device on the classical wire's fake bus.
    pub(crate) fn test_wire(nodeid: u32) -> (Self, UnixStream) {
        Self::test_wire_on(nodeid, CanWire::Classical)
    }

    /// The same on `wire`: a CAN FD link's fake bus carries seventy-two byte
    /// records, and its device reads the same.
    pub(crate) fn test_wire_on(nodeid: u32, wire: CanWire) -> (Self, UnixStream) {
        let (socket, reader) = UnixStream::pair().expect("a socket pair");
        let device = Self {
            socket: File::from(OwnedFd::from(socket)),
            interface: "test0".to_string(),
            uuid: [0; 6],
            nodeid,
            wire,
            link: Mutex::new(
                CanSerialLink::for_node(nodeid).expect("a test node id is in the CAN range"),
            ),
            tunnels: TunnelRegistry::new(),
            stopped: AtomicBool::new(false),
        };
        (device, reader)
    }
}

/// Every CAN record the device on a [`test wire`](CanSerialDevice::test_wire) has
/// written, taken off the reader end the way a receiving link reads the classical
/// bus.
#[cfg(test)]
pub(crate) fn read_wire(wire: &mut UnixStream) -> Vec<CanFrame> {
    read_wire_on(wire, CanWire::Classical)
}

/// The same for a device on `can_wire`'s wire: a CAN FD link's records are
/// seventy-two bytes each, not sixteen.
///
/// A socket pair is a *stream*, so one read can come back short: this drains until
/// the wire is empty, then parses whole records out of the bytes.
#[cfg(test)]
fn read_wire_on(wire: &mut UnixStream, can_wire: CanWire) -> Vec<CanFrame> {
    wire.set_nonblocking(true)
        .expect("the test wire takes a non-blocking flag");
    let mut raw = Vec::new();
    let mut buf = [0u8; CAN_FD_FRAME_SIZE];
    loop {
        match wire.read(&mut buf) {
            Ok(0) => break,
            Ok(read) => raw.extend_from_slice(&buf[..read]),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
            Err(e) => panic!("reading the test wire failed: {e}"),
        }
    }
    assert_eq!(
        raw.len() % can_wire.frame_size(),
        0,
        "the device writes whole CAN records"
    );
    raw.chunks(can_wire.frame_size())
        .map(|record| {
            let mut abi = [0u8; CAN_FD_FRAME_SIZE];
            abi[..can_wire.frame_size()].copy_from_slice(record);
            CanFrame::from_abi(can_wire, &abi)
        })
        .collect()
}

/// A raw CAN socket for Klipper's **admin** frames: the broadcast that finds the
/// boards which have no node id yet.
///
/// [`CanSerialDevice`] cannot serve this — it opens a socket already addressed to
/// one node and assigns that node's id before anything else, while a board with
/// no node id cannot be addressed at all. The socket half below is the same
/// recipe and the same helpers: what differs is the filter (the admin answer id,
/// since a scan has nothing else to read) and that nothing is addressed.
///
/// As for [`CanSerialDevice`], this half cannot be exercised in this repository's
/// test environment: the frames it carries are unit tested, and the rest is
/// covered by compilation.
pub struct CanbusAdminSocket {
    socket: File,
    interface: String,
    /// The link's shape, as for [`CanSerialDevice`]: an admin scan on a CAN FD
    /// bus reads and writes seventy-two byte datagrams.
    wire: CanWire,
}

impl CanbusAdminSocket {
    /// Open `interface` and listen for answers to the admin broadcast — the
    /// classical CAN link.
    ///
    /// # Errors
    /// The same as [`CanbusAdminSocket::open_with_wire`].
    pub fn open(interface: &str) -> Result<Self, InterfaceError> {
        Self::open_with_wire(interface, CanWire::Classical)
    }

    /// Open `interface` on `wire` and listen for answers to the admin broadcast.
    ///
    /// The broadcast itself is the same either way — one data byte on the admin
    /// id, and the answers the same seven or eight — but the frames that carry it
    /// are the wire's size, so a scan has to be told which bus it is on.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] when the interface does not exist, or the
    /// socket cannot be created, bound, filtered, or given CAN FD frames.
    pub fn open_with_wire(interface: &str, wire: CanWire) -> Result<Self, InterfaceError> {
        let index = interface_index(interface)?;
        let socket = open_socket(wire)?;
        bind_interface(&socket, index)?;
        filter_answers(&socket, ADMIN_ID + 1)?;
        debug!(
            "admin socket ready on {interface} ({}), listening for {:#x}",
            wire.name(),
            ADMIN_ID + 1
        );
        Ok(Self {
            socket,
            interface: interface.to_string(),
            wire,
        })
    }
}

impl fmt::Debug for CanbusAdminSocket {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CanbusAdminSocket")
            .field("interface", &self.interface)
            .finish_non_exhaustive()
    }
}

impl AdminBus for CanbusAdminSocket {
    fn now(&self) -> Instant {
        Instant::now()
    }

    fn send(&mut self, frame: &CanFrame) -> Result<(), InterfaceError> {
        trace!(
            "tx admin frame on {}: {}",
            self.interface,
            describe_can_frame(frame.id(), frame.data())
        );
        (&self.socket)
            .write_all(frame.to_abi(self.wire)?.as_bytes())
            .map_err(|e| InterfaceError::SendError(format!("CAN write failed: {e}")))
    }

    fn receive(&mut self, wait: Duration) -> Result<Option<CanFrame>, InterfaceError> {
        // The largest datagram either wire delivers; a classical read fills the
        // first sixteen bytes of it.
        let mut raw = [0u8; CAN_FD_FRAME_SIZE];
        loop {
            match wait_readable(self.socket.as_raw_fd(), wait) {
                Wait::Readable => {}
                // The wait ran out: the slice of the window it was given is
                // spent, and the caller decides what is left of the window.
                Wait::Timeout => return Ok(None),
                // A signal: wait again rather than throwing the answer away.
                Wait::Retry => continue,
                Wait::Closed => return Err(InterfaceError::ConnectionLost),
            }
            match (&self.socket).read(&mut raw) {
                Ok(read) if read >= self.wire.frame_size() => {
                    return Ok(Some(CanFrame::from_abi(self.wire, &raw)))
                }
                // A datagram socket delivers whole frames, and the filter keeps
                // everything but the admin answers out; treat a short read as
                // someone else's frame rather than guessing.
                Ok(read) => debug!("ignoring {read} byte CAN read"),
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(e) => {
                    return Err(InterfaceError::Other(format!(
                        "CAN read on {} failed: {e}",
                        self.interface
                    )))
                }
            }
        }
    }
}

/// What a `poll` on the socket said.
pub(crate) enum Wait {
    Readable,
    Timeout,
    Closed,
    Retry,
}

/// Wait up to `timeout` for the socket to have a frame.
///
/// `poll` counts in whole milliseconds, so a timeout shorter than one waits that
/// millisecond rather than spinning on a zero timeout; one longer than a `c_int`
/// can hold is capped, which only a caller asking for tens of days would reach.
pub(crate) fn wait_readable(fd: RawFd, timeout: Duration) -> Wait {
    let mut poll_fd = libc::pollfd {
        fd,
        events: libc::POLLIN,
        revents: 0,
    };
    let millis = timeout.as_millis().clamp(1, libc::c_int::MAX as u128) as libc::c_int;
    // SAFETY: `poll_fd` is one initialised pollfd, as poll expects.
    let ready = unsafe { libc::poll(&mut poll_fd, 1, millis) };
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
pub(crate) fn interface_index(interface: &str) -> Result<libc::c_uint, InterfaceError> {
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

/// Create a raw CAN socket, CAN FD datagrams enabled when `wire` is
/// [`CanWire::Fd`].
pub(crate) fn open_socket(wire: CanWire) -> Result<File, InterfaceError> {
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
    let socket = unsafe { File::from_raw_fd(fd) };
    if wire == CanWire::Fd {
        enable_fd_frames(&socket)?;
    }
    // Not fatal: a socket that took the rest of the options still carries frames,
    // and a host that cannot set the option at all would otherwise fail to open a
    // bus it could use. It is said once per socket, and it says the wait is
    // unbounded from here on.
    if let Err(e) = set_send_timeout(&socket) {
        warn!("CAN socket write deadline not set, writes may wait indefinitely: {e}");
    }
    Ok(socket)
}

/// Give the socket a deadline for one write: `SEND_TIMEOUT`.
///
/// The option is `SO_SNDTIMEO`, so it bounds a *blocking* send rather than making
/// the socket non-blocking — a full send buffer is the only wait a CAN write has,
/// and leaving the descriptor blocking keeps every other call as it was.
fn set_send_timeout(socket: &File) -> Result<(), InterfaceError> {
    let timeout = libc::timeval {
        tv_sec: SEND_TIMEOUT.as_secs() as libc::time_t,
        tv_usec: SEND_TIMEOUT.subsec_micros() as libc::suseconds_t,
    };
    // SAFETY: the option value is a `timeval` of the length passed.
    let rc = unsafe {
        libc::setsockopt(
            socket.as_raw_fd(),
            libc::SOL_SOCKET,
            libc::SO_SNDTIMEO,
            &timeout as *const libc::timeval as *const libc::c_void,
            std::mem::size_of::<libc::timeval>() as libc::socklen_t,
        )
    };
    if rc < 0 {
        return Err(InterfaceError::Other(format!(
            "failed to set the CAN socket write deadline: {}",
            std::io::Error::last_os_error()
        )));
    }
    Ok(())
}

/// Ask the kernel to pass CAN FD frames on this socket.
///
/// Without the option the socket takes only `struct can_frame`: CAN FD traffic
/// from the bus is not delivered to it, and a CAN FD frame cannot be sent from
/// it. The option is part of a link's *shape* and is therefore set once, when the
/// socket is opened, rather than per frame. A socket that has it still carries
/// classical frames, so this is not what makes a CAN FD link talk to a classical
/// peer: the two wires are meant to be chosen on both ends
/// ([`CanSerialDevice::open_with_wire`]).
fn enable_fd_frames(socket: &File) -> Result<(), InterfaceError> {
    let on: libc::c_int = 1;
    // SAFETY: the option value is an `int` of the length passed.
    let rc = unsafe {
        libc::setsockopt(
            socket.as_raw_fd(),
            SOL_CAN_RAW,
            CAN_RAW_FD_FRAMES,
            &on as *const libc::c_int as *const libc::c_void,
            std::mem::size_of::<libc::c_int>() as libc::socklen_t,
        )
    };
    if rc < 0 {
        return Err(InterfaceError::Other(format!(
            "failed to enable CAN FD frames on the socket: {}",
            std::io::Error::last_os_error()
        )));
    }
    Ok(())
}

/// Bind the socket to one CAN interface.
pub(crate) fn bind_interface(socket: &File, index: libc::c_uint) -> Result<(), InterfaceError> {
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

/// Ask the kernel for only the frames the MCU sends: the answer id, standard
/// (11-bit) form.
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
    use crate::core::klippy::frame::MESSAGE_PAYLOAD_MAX;
    use crate::core::klippy::interface::devices::serial::{
        decode_header, HeaderScan, ADMIN_LISTEN, CMD_REQUEST_BOOTLOADER, CMD_SET_PORT_NODEID,
        HEADER_BYTES, MAGIC, MUX_PORT_UNITS,
    };

    /// An encoded Klipper frame to feed through the link.
    fn message(seq: u8, payload: &[u8]) -> Frame {
        Frame::new(seq, payload.to_vec())
    }

    #[test]
    fn test_node_ids_follow_klipper() {
        // Klipper: `txid = canbus_nodeid * 2 + 256`, answers on the next id.
        let link = CanSerialLink::for_node(2).unwrap();
        assert_eq!(link.tx_id(), 0x104);
        assert_eq!(link.rx_id(), 0x105);

        let link = CanSerialLink::for_node(1).unwrap();
        assert_eq!((link.tx_id(), link.rx_id()), (0x102, 0x103));
    }

    #[test]
    fn test_a_node_id_outside_the_range_has_no_link() {
        // An id is `1..=0xff`: zero names no node, and `0x100` is past the last
        // one. Both are `None` rather than a link folding onto someone else's
        // arbitration id.
        assert!(CanSerialLink::for_node(0).is_none());
        assert!(CanSerialLink::for_node(0x100).is_none());

        // The ends of the range do have a link.
        assert!(CanSerialLink::for_node(1).is_some());
        assert!(CanSerialLink::for_node(0xff).is_some());
    }

    #[test]
    fn test_the_link_and_the_shared_mapping_agree() {
        // CAN no longer owns the formula: the link is built from the mapping the
        // transports share, so a node's two directions have to be one object
        // rather than two spellings of it.
        for nodeid in [1, 4, 5, 255] {
            let link = CanSerialLink::for_node(nodeid).expect("this id is in range");
            let tx_id = link.tx_id();
            let rx_id = link.rx_id();

            assert_eq!(tx_id, assign::arb_id(nodeid).unwrap());
            assert_eq!(rx_id, assign::reply_arb_id(nodeid).unwrap());
            assert!(!assign::is_reply_arb(tx_id));
            assert!(assign::is_reply_arb(rx_id));
            assert_eq!(assign::mcu_id_from_arb(tx_id), Some(nodeid));
            assert_eq!(assign::mcu_id_from_arb(rx_id), Some(nodeid));
        }
    }

    #[test]
    fn test_a_block_is_cut_into_eight_byte_frames() {
        let link = CanSerialLink::for_node(3).unwrap();
        let bytes: Vec<u8> = (0..20).collect();

        let frames: Vec<CanFrame> = link.frames(&bytes, CanWire::Classical).collect();

        // 20 bytes is two full frames and a remainder, all addressed to the MCU.
        assert_eq!(frames.len(), 3);
        assert_eq!(frames[0].data(), &bytes[0..8]);
        assert_eq!(frames[1].data(), &bytes[8..16]);
        assert_eq!(frames[2].data(), &bytes[16..20]);
        assert!(frames.iter().all(|f| f.id() == link.tx_id()));

        // A block that is an exact multiple of eight does not get an empty tail.
        assert_eq!(link.frames(&bytes[..16], CanWire::Classical).count(), 2);
    }

    #[test]
    fn test_a_block_is_cut_into_sixty_four_byte_frames_on_the_can_fd_wire() {
        let link = CanSerialLink::for_node(3).unwrap();
        let bytes: Vec<u8> = (0..130).map(|byte| byte as u8).collect();

        let frames: Vec<CanFrame> = link.frames(&bytes, CanWire::Fd).collect();

        // 130 bytes is two full CAN FD frames and a two byte tail, all addressed
        // to the MCU: the same rule as the classical cut, with a wider frame.
        assert_eq!(frames.len(), 3);
        assert_eq!(frames[0].data(), &bytes[0..CAN_FD_DATA_BYTES]);
        assert_eq!(frames[1].data(), &bytes[64..128]);
        assert_eq!(frames[2].data(), &bytes[128..130]);
        assert_eq!(frames[2].data().len(), 2, "the tail is what is left");
        assert!(frames.iter().all(|f| f.id() == link.tx_id()));

        // An exact multiple of sixty-four does not get an empty tail either.
        assert_eq!(link.frames(&bytes[..128], CanWire::Fd).count(), 2);
    }

    #[test]
    fn test_the_bus_and_the_block_are_described_differently() {
        // A block small enough for one CAN frame: the bus line names the
        // arbitration id and shows the eight data bytes, while the block line is
        // the serial frame. Both are logged even though one CAN frame carried it
        // all, because a slice and a block are not the same thing.
        let block = message(1, &[0xaa, 0xbb]);
        let bytes = block.raw_bytes();
        assert_eq!(bytes.len(), 7);

        let link = CanSerialLink::for_node(5).unwrap();
        let slices: Vec<CanFrame> = link.frames(&bytes, CanWire::Classical).collect();
        assert_eq!(slices.len(), 1, "one CAN frame carries the whole block");

        assert_eq!(
            describe_can_frame(slices[0].id(), slices[0].data()),
            format!(
                "id {:#x} data 0711aabb {:02x}{:02x}7e",
                link.tx_id(),
                bytes[4],
                bytes[5]
            )
        );
        assert_eq!(
            describe_frame(&bytes),
            format!("0711 | aabb | {:02x}{:02x}7e", bytes[4], bytes[5])
        );
    }

    #[test]
    fn test_incoming_frames_rebuild_the_block_stream() {
        let sender = CanSerialLink::for_node(5).unwrap();
        let mut receiver = CanSerialLink::for_node(5).unwrap();

        // The MCU answers on the next id, so a frame written to `tx_id` and read
        // back on `rx_id` models a round trip.
        let block = message(1, &[0xaa; 30]).raw_bytes();
        let frames: Vec<CanFrame> = sender
            .frames(&block, CanWire::Classical)
            .map(|chunk| CanFrame::new(receiver.rx_id(), chunk.data()).unwrap())
            .collect();
        // 30 payload bytes plus the 5 byte block overhead: 35 bytes, five frames.
        assert_eq!(frames.len(), 5, "35 bytes need five CAN frames");

        // Nothing is complete until the last frame has arrived.
        for (i, frame) in frames.iter().enumerate() {
            assert!(receiver.accept(frame));
            let decoded = receiver.next_frame();
            if i < frames.len() - 1 {
                assert!(decoded.is_none(), "frame {i} should not complete a block");
            } else {
                assert_eq!(decoded, Some(message(1, &[0xaa; 30])));
            }
        }
        assert_eq!(receiver.next_frame(), None);
    }

    #[test]
    fn test_frames_for_another_node_are_ignored() {
        let mut link = CanSerialLink::for_node(5).unwrap();
        let foreign = CanFrame::new(0x123, &[1, 2, 3]).unwrap();

        assert!(!link.accept(&foreign));
        assert_eq!(link.next_frame(), None);

        // ... and a frame of ours behind it is still accepted.
        let ours = CanFrame::new(link.rx_id(), &message(2, &[9]).raw_bytes()).unwrap();
        assert!(link.accept(&ours));
        assert_eq!(link.next_frame(), Some(message(2, &[9])));
    }

    #[test]
    fn test_a_tunnel_unit_is_cut_out_of_the_incoming_stream() {
        use crate::core::klippy::interface::devices::serial::{tunnel_unit, TUNNEL_TAG};

        // The port's reply id heads both the CAN frames and the unit inside
        // them: the unit stream needs the header, the frame stream does not.
        let block = [0x01, 0x88, 0x11, 0x00, 0xf1, 0x7c, 0x99, 0x03];
        let mut receiver = CanSerialLink::for_node(5).unwrap();
        let reply = receiver.rx_id();
        let unit = tunnel_unit(reply, &block).expect("a whole block");
        assert_eq!(
            &unit[2..4],
            &TUNNEL_TAG[..],
            "the frame is the katapult block"
        );

        // A bare klipper frame still reassembles, and draws no tunnel: the
        // frame path is untouched.
        let bare = CanFrame::new(reply, &message(2, &[9]).raw_bytes()).unwrap();
        assert!(receiver.accept(&bare));
        assert_eq!(receiver.next_frame(), Some(message(2, &[9])));
        assert_eq!(receiver.next_tunnel(), None);

        // A tunnel unit is then cut whole out of the same stream — a unit
        // longer than one CAN frame and all — and the frame stream sees
        // nothing of it (a tunnel frame's first byte is no klipper length).
        for chunk in unit.chunks(CAN_DATA_BYTES) {
            assert!(receiver.accept(&CanFrame::new(reply, chunk).unwrap()));
        }
        assert_eq!(receiver.next_tunnel(), Some((reply, block.to_vec())));
        assert_eq!(receiver.next_tunnel(), None);
        assert_eq!(receiver.next_frame(), None);
    }

    #[test]
    fn test_a_frame_carries_at_most_its_wire_s_allowed_bytes() {
        // Sixty-four bytes is what the type holds, CAN FD's frame being the
        // larger of the two; one more has nowhere to go.
        assert!(CanFrame::new(0x100, &[0; CAN_FD_DATA_BYTES]).is_ok());
        let err = CanFrame::new(0x100, &[0; CAN_FD_DATA_BYTES + 1]).unwrap_err();
        assert!(err.to_string().contains("do not fit"), "{err}");

        // A classical frame holds eight, and the rest of the type's storage is not
        // a licence to use it: nine bytes fit CAN FD and are refused by the
        // classical wire rather than truncated into a 16 byte datagram.
        assert_eq!(CAN_DATA_BYTES, 8, "the classical wire's width is unchanged");
        let nine = CanFrame::new(0x100, &[0; CAN_DATA_BYTES + 1]).expect("nine fit CAN FD");
        let err = nine.to_abi(CanWire::Classical).unwrap_err();
        assert!(
            err.to_string().contains("do not fit a classical CAN frame"),
            "{err}"
        );
        assert_eq!(nine.to_abi(CanWire::Fd).unwrap().as_bytes().len(), 72);
    }

    #[test]
    fn test_can_frame_layout_matches_the_kernel() {
        // One datagram per frame, so the sizes and field offsets are the ABI. Two
        // independent facts: how long each wire's frame struct is, and how long
        // the address struct is — which is not the frame's size on either wire.
        // Agreement with the kernel is what a machine with a CAN interface would
        // have to confirm.
        assert_eq!(CAN_FRAME_SIZE, 16);
        assert_eq!(CAN_FD_FRAME_SIZE, 72);
        assert_eq!(std::mem::size_of::<SockAddrCan>(), 16);
        assert_eq!(std::mem::size_of::<CanFilter>(), 8);

        let frame = CanFrame::new(0x1ab, &[1, 2, 3]).unwrap();
        let raw = frame.to_abi(CanWire::Classical).unwrap();
        let abi = raw.as_bytes();
        assert_eq!(
            abi.len(),
            CAN_FRAME_SIZE,
            "a classical datagram is 16 bytes"
        );
        assert_eq!(u32::from_ne_bytes([abi[0], abi[1], abi[2], abi[3]]), 0x1ab);
        assert_eq!(abi[4], 3);
        assert_eq!(&abi[8..11], &[1, 2, 3]);
        assert_eq!(
            CanFrame::from_abi(CanWire::Classical, &abi_array(abi)),
            frame
        );

        // Extended ids are not part of this transport: only the low 11 bits are
        // read back.
        let mut extended = abi_array(abi);
        extended[..4].copy_from_slice(&(0x1ab | CAN_EFF_FLAG).to_ne_bytes());
        assert_eq!(
            CanFrame::from_abi(CanWire::Classical, &extended).id(),
            0x1ab
        );
    }

    #[test]
    fn test_the_can_fd_frame_layout_matches_the_kernel() {
        // `struct canfd_frame`: same header shape, sixty-four data bytes. The two
        // fields that matter downstream sit where the classical struct has them.
        let data = [0x11u8; CAN_FD_DATA_BYTES];
        let frame = CanFrame::new(0x1ab, &data).unwrap();
        let raw = frame.to_abi(CanWire::Fd).unwrap();
        let abi = raw.as_bytes();

        assert_eq!(
            abi.len(),
            CAN_FD_FRAME_SIZE,
            "a CAN FD datagram is 72 bytes"
        );
        assert_eq!(u32::from_ne_bytes([abi[0], abi[1], abi[2], abi[3]]), 0x1ab);
        assert_eq!(abi[4], CAN_FD_DATA_BYTES as u8, "the length is at offset 4");
        assert_eq!(
            abi[5], 0,
            "no flags: the kernel sets FDF for a 72 byte write"
        );
        assert_eq!(&abi[6..8], [0, 0], "the reserved bytes stay zero");
        assert_eq!(&abi[8..], data, "the data starts at offset 8");

        // Reading back gives the frame again: the same layout, the wire's size.
        assert_eq!(CanFrame::from_abi(CanWire::Fd, &abi_array(abi)), frame);
    }

    /// The datagram as [`CanFrame::from_abi`] takes it: a whole kernel struct,
    /// zero filled beyond the bytes `abi` holds.
    fn abi_array(abi: &[u8]) -> [u8; CAN_FD_FRAME_SIZE] {
        let mut raw = [0u8; CAN_FD_FRAME_SIZE];
        raw[..abi.len()].copy_from_slice(abi);
        raw
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

        // The admin socket is the same first step, and reports it the same way —
        // on either wire, since the interface is looked up before the socket.
        let err = CanbusAdminSocket::open("can99").unwrap_err();
        assert!(
            err.to_string().contains("no CAN interface named 'can99'"),
            "{err}"
        );
        let err = CanbusAdminSocket::open_with_wire("can99", CanWire::Fd).unwrap_err();
        assert!(
            err.to_string().contains("no CAN interface named 'can99'"),
            "{err}"
        );
    }

    #[test]
    fn test_open_rejects_a_node_id_outside_the_range() {
        // The node id is checked before the bus is touched, so it is reported
        // even for an interface that does not exist: were the socket opened
        // first, the missing interface would be the complaint instead.
        let err = CanSerialDevice::open("can99", [0; 6], 0x100).unwrap_err();
        assert_eq!(
            err.to_string(),
            format!("node id 256 is outside the CAN range 1..={MAX_MCU_ID}")
        );
        assert!(err.to_string().contains("1..=255"), "{err}");

        // Zero is outside the range too: those bits name no node.
        let err = CanSerialDevice::open("can99", [0; 6], 0).unwrap_err();
        assert!(err.to_string().contains("node id 0 is outside"), "{err}");
    }

    // ========================================================================
    // Finding the boards that have no node id yet
    // ========================================================================

    /// An answer frame from the board with `uuid`, as the firmware sends it: the
    /// response byte, the UUID, and the application byte when it sends one.
    fn answer(uuid: [u8; 6], application: Option<u8>) -> CanFrame {
        let mut data = vec![RESP_NEED_NODEID];
        data.extend_from_slice(&uuid);
        if let Some(application) = application {
            data.push(application);
        }
        CanFrame::new(ADMIN_ID + 1, &data).expect("an answer fits in one CAN frame")
    }

    #[test]
    fn test_the_query_frame_is_the_admin_broadcast() {
        // `scripts/canbus_query.py`: the broadcast id with the query as its only
        // byte — the bytes on the wire, not just the two fields.
        let frame = query_unassigned_frame();
        assert_eq!(frame.id(), 0x3f0);
        assert_eq!(frame.data(), [0x00]);

        let raw = frame.to_abi(CanWire::Classical).unwrap();
        let abi = raw.as_bytes();
        assert_eq!(abi.len(), CAN_FRAME_SIZE);
        assert_eq!(u32::from_ne_bytes([abi[0], abi[1], abi[2], abi[3]]), 0x3f0);
        assert_eq!(abi[4], 1, "one data byte");
        assert_eq!(&abi[8..9], [0x00]);
        assert_eq!(&abi[9..], [0; CAN_DATA_BYTES - 1], "no padding is sent");
    }

    #[test]
    fn test_the_request_bootloader_frame_is_the_admin_broadcast() {
        // `scripts/flash_can.py::_jump_to_bootloader`: the admin id with the
        // request byte and the board's UUID, most significant byte first — the
        // bytes `cansend can0 3f0#02<uuid>` writes.
        let uuid = [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc];
        let frame = request_bootloader_frame(uuid);
        assert_eq!(frame.id(), ADMIN_ID);
        assert_eq!(frame.data(), [0x02, 0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc]);

        // The literal the task's manual work used: `3f0#02` then the uuid.
        let raw = frame.to_abi(CanWire::Classical).unwrap();
        let abi = raw.as_bytes();
        assert_eq!(u32::from_ne_bytes([abi[0], abi[1], abi[2], abi[3]]), 0x3f0);
        assert_eq!(abi[4], 7, "seven data bytes, which is the device's minimum");
        assert_eq!(
            &abi[8..15],
            [0x02, 0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc],
            "the request byte, then the uuid"
        );
        assert_eq!(&abi[15..], [0; 1], "no padding is sent");
    }

    #[test]
    fn test_an_answer_gives_the_uuid_and_the_application() {
        let frame = answer([0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc], Some(CMD_SET_NODEID));

        let node = UnassignedNode::from_reply(&frame).expect("this is an answer");
        assert_eq!(node.uuid(), [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc]);
        assert_eq!(node.uuid_hex(), "11aa22bb33cc");
        assert_eq!(node.application(), Some(0x01));
        assert_eq!(node.application_name(), "Klipper");

        // CanBoot answers the same way, and names itself.
        let node = UnassignedNode::from_reply(&answer([0; 6], Some(CMD_SET_CANBOOT_NODEID)))
            .expect("this is an answer");
        assert_eq!(node.application_name(), "CanBoot");
    }

    #[test]
    fn test_the_uuid_is_read_most_significant_byte_first() {
        // Klipper sums the six bytes as `data[1]` shifted by 40 bits, so the
        // first byte after the response is the most significant one: reading
        // them the other way round would print a different number.
        let node = UnassignedNode::from_reply(&answer([0x01, 0, 0, 0, 0, 0], None)).unwrap();
        assert_eq!(node.uuid_hex(), "010000000000");

        let node = UnassignedNode::from_reply(&answer([0, 0, 0, 0, 0, 0x01], None)).unwrap();
        assert_eq!(node.uuid_hex(), "000000000001");

        // A UUID whose leading byte is non-zero, spelled out as Klipper's tool
        // computes it, so a whole-word byte swap cannot pass this.
        let expected = 0x11u64 << 40 | 0x22 << 32 | 0x33 << 24 | 0x44 << 16 | 0x55 << 8 | 0x66;
        let node = UnassignedNode::from_reply(&answer(
            [0x11, 0x22, 0x33, 0x44, 0x55, 0x66],
            Some(CMD_SET_NODEID),
        ))
        .unwrap();
        assert_eq!(node.uuid_hex(), format!("{expected:012x}"));
        assert_eq!(node.uuid_hex(), "112233445566");
    }

    #[test]
    fn test_a_frame_that_is_not_an_answer_is_ignored() {
        // Too few bytes to hold a UUID: `dlc < 7` in upstream's terms. Six is
        // the boundary — seven is an answer, from the next test on.
        let short = CanFrame::new(ADMIN_ID + 1, &[RESP_NEED_NODEID, 1, 2, 3, 4, 5]).unwrap();
        assert_eq!(UnassignedNode::from_reply(&short), None);

        // The response byte is not the one an unassigned board sends — an admin
        // answer to something else.
        let mut other = vec![0x21];
        other.extend_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x01]);
        assert_eq!(
            UnassignedNode::from_reply(&CanFrame::new(ADMIN_ID + 1, &other).unwrap()),
            None
        );

        // The query itself, or any other traffic on the bus: a scan sees frames
        // that are not answers and has to pass them by. The node's own link id
        // (0x102) is one of them.
        assert_eq!(UnassignedNode::from_reply(&query_unassigned_frame()), None);
        let mut foreign = vec![RESP_NEED_NODEID];
        foreign.extend_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x01]);
        assert_eq!(
            UnassignedNode::from_reply(&CanFrame::new(0x102, &foreign).unwrap()),
            None
        );
    }

    #[test]
    fn test_an_answer_without_the_application_byte_is_klipper() {
        // Seven data bytes: the answer the protocol had before the application
        // byte existed. Upstream's tool reads the missing byte as Klipper's own
        // application id, and so does this.
        let frame = answer([0x11, 0x22, 0x33, 0x44, 0x55, 0x66], None);
        assert_eq!(frame.data().len(), 7);

        let node = UnassignedNode::from_reply(&frame).expect("seven bytes are an answer");
        assert_eq!(node.application(), None);
        assert_eq!(node.application_name(), "Klipper");
        assert_eq!(node.uuid_hex(), "112233445566");
    }

    #[test]
    fn test_an_unknown_application_is_named_unknown() {
        // An application id that is neither Klipper nor CanBoot is reported as
        // it is, rather than guessed at — and a request command (0x02) is not an
        // application id at all.
        let node = UnassignedNode::from_reply(&answer([0; 6], Some(0x02))).unwrap();
        assert_eq!(node.application(), Some(0x02));
        assert_eq!(node.application_name(), "Unknown");
    }

    #[test]
    fn test_an_all_zero_uuid_is_still_reported() {
        // The protocol has no "no UUID programmed" value, and upstream's tool
        // prints this answer like any other, so the shape of the frame is all
        // that is checked here too.
        let node = UnassignedNode::from_reply(&answer([0; 6], Some(CMD_SET_NODEID)))
            .expect("an answer with a zero UUID is still an answer");
        assert_eq!(node.uuid(), [0; 6]);
        assert_eq!(node.uuid_hex(), "000000000000");
        assert_eq!(node.application_name(), "Klipper");
    }

    #[test]
    fn test_a_board_that_answers_twice_is_listed_once() {
        let first = answer([0x11, 0x22, 0x33, 0x44, 0x55, 0x66], Some(CMD_SET_NODEID));
        // The firmware retries its answer, so the same UUID arrives again.
        let again = answer([0x11, 0x22, 0x33, 0x44, 0x55, 0x66], Some(CMD_SET_NODEID));
        let other = answer(
            [0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff],
            Some(CMD_SET_CANBOOT_NODEID),
        );

        let mut scan = UnassignedScan::new();
        assert_eq!(
            scan.feed(&first).map(|node| node.uuid_hex()),
            Some("112233445566".to_string())
        );
        assert_eq!(
            scan.feed(&again),
            None,
            "the second frame of a UUID is not news"
        );
        assert!(scan.feed(&other).is_some());
        assert_eq!(scan.feed(&query_unassigned_frame()), None, "not an answer");

        assert_eq!(scan.total(), 2);
        assert_eq!(
            scan.nodes()
                .iter()
                .map(|node| node.uuid_hex())
                .collect::<Vec<_>>(),
            ["112233445566", "aabbccddeeff"],
            "listed once each, in the order they were first heard"
        );
    }

    /// What one [`AdminBus::receive`] on a scripted bus does.
    enum Reply {
        /// The frame arrives after this much of the wait.
        After(Duration, CanFrame),
        /// Nothing arrives: the wait runs out.
        Never,
    }

    /// A bus that answers from a script and a clock that moves only as far as
    /// the scan asks it to wait — the two are one object because the waiting is
    /// exactly what consumes the window (see [`AdminBus`]).
    struct ScriptedBus {
        start: Instant,
        /// How much of the window the receives so far have spent.
        waited: Duration,
        /// Every wait the scan asked for, in order.
        waits: Vec<Duration>,
        /// Every frame the scan sent.
        sent: Vec<CanFrame>,
        script: std::collections::VecDeque<Reply>,
    }

    impl ScriptedBus {
        fn new(script: Vec<Reply>) -> Self {
            Self {
                start: Instant::now(),
                waited: Duration::ZERO,
                waits: Vec::new(),
                sent: Vec::new(),
                script: script.into(),
            }
        }
    }

    impl AdminBus for ScriptedBus {
        fn now(&self) -> Instant {
            self.start + self.waited
        }

        fn send(&mut self, frame: &CanFrame) -> Result<(), InterfaceError> {
            self.sent.push(*frame);
            Ok(())
        }

        fn receive(&mut self, wait: Duration) -> Result<Option<CanFrame>, InterfaceError> {
            self.waits.push(wait);
            match self
                .script
                .pop_front()
                .expect("the test scripted every receive")
            {
                Reply::After(delay, frame) => {
                    self.waited += delay;
                    Ok(Some(frame))
                }
                Reply::Never => {
                    self.waited += wait;
                    Ok(None)
                }
            }
        }
    }

    #[test]
    fn test_the_query_window_is_a_deadline_from_the_broadcast() {
        // Two boards answer 300 ms and 400 ms in, the first twice; then the bus
        // goes quiet. The scan asks for what is left of the two second window
        // each time — 2.000, 1.700, 1.600, 1.500 — and stops when the last of it
        // runs out, without waiting two real seconds or opening a socket.
        let uuid = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        let mut bus = ScriptedBus::new(vec![
            Reply::After(
                Duration::from_millis(300),
                answer(uuid, Some(CMD_SET_NODEID)),
            ),
            Reply::After(
                Duration::from_millis(100),
                answer(uuid, Some(CMD_SET_NODEID)),
            ),
            Reply::After(
                Duration::from_millis(100),
                answer([0xaa; 6], Some(CMD_SET_CANBOOT_NODEID)),
            ),
            Reply::Never,
        ]);
        let mut reported = Vec::new();
        let scan = query_unassigned(&mut bus, Duration::from_secs(2), |node| reported.push(node))
            .expect("the scripted bus does not fail");

        // The broadcast goes out first, and the window is measured from it.
        assert_eq!(bus.sent, [query_unassigned_frame()]);
        assert_eq!(
            bus.waits,
            [
                Duration::from_secs(2),
                Duration::from_millis(1700),
                Duration::from_millis(1600),
                Duration::from_millis(1500),
            ]
        );

        // The repeated answer is reported once, and both boards are in the scan.
        assert_eq!(scan.total(), 2);
        assert_eq!(reported, scan.nodes());
        assert_eq!(
            reported
                .iter()
                .map(|node| node.uuid_hex())
                .collect::<Vec<_>>(),
            ["112233445566", "aaaaaaaaaaaa"]
        );
    }

    #[test]
    fn test_a_frame_at_the_deadline_ends_the_window_without_another_read() {
        // A board that answers as the window closes is still reported, and the
        // scan then stops without asking for a read that would run past the
        // deadline: the window is measured from the query, not from the last
        // answer.
        let mut bus = ScriptedBus::new(vec![Reply::After(
            Duration::from_secs(2),
            answer([0x11; 6], Some(CMD_SET_NODEID)),
        )]);
        let mut reported = Vec::new();
        let scan = query_unassigned(&mut bus, Duration::from_secs(2), |node| reported.push(node))
            .expect("the scripted bus does not fail");

        assert_eq!(
            bus.waits,
            [Duration::from_secs(2)],
            "the one read, and no more"
        );
        assert_eq!(scan.total(), 1);
        assert_eq!(reported, scan.nodes());
    }

    #[test]
    fn test_a_bus_that_fails_ends_the_scan() {
        // A read error is the bus failing, not the window closing: it is handed
        // back rather than reported as "no boards found".
        struct FailingBus;

        impl AdminBus for FailingBus {
            fn now(&self) -> Instant {
                Instant::now()
            }

            fn send(&mut self, _frame: &CanFrame) -> Result<(), InterfaceError> {
                Ok(())
            }

            fn receive(&mut self, _wait: Duration) -> Result<Option<CanFrame>, InterfaceError> {
                Err(InterfaceError::ConnectionLost)
            }
        }

        let err = query_unassigned(&mut FailingBus, Duration::from_secs(2), |_| {})
            .expect_err("the read failed");
        assert_eq!(err, InterfaceError::ConnectionLost);
    }

    // ========================================================================
    // The port-level commands on the byte stream
    // ========================================================================

    /// The unit `device` has just written, as it came off the wire: whole CAN
    /// records, all of them this node's, joining back into `unit` byte for byte.
    ///
    /// This is the "no new frame shape" claim measured rather than asserted: the
    /// expected bytes come from the serial side's own builders.
    fn assert_unit_on_the_wire(device: &CanSerialDevice, wire: &mut UnixStream, unit: &[u8]) {
        let frames = read_wire(wire);
        assert_eq!(
            frames.len(),
            unit.len().div_ceil(CAN_DATA_BYTES),
            "a classical CAN frame holds eight data bytes"
        );
        let tx_id = device.link.lock().unwrap().tx_id();
        assert!(
            frames.iter().all(|frame| frame.id() == tx_id),
            "every slice is this node's: {frames:?}"
        );
        let joined: Vec<u8> = frames.iter().flat_map(|f| f.data().to_vec()).collect();
        assert_eq!(joined, unit, "the unit is the serial side's own bytes");
    }

    #[test]
    fn test_the_port_commands_are_the_serial_sides_units_on_the_wire() {
        // The port-level commands on a CAN link: what the device puts on
        // the bus is `devices::serial`'s unit — the same bytes a shared serial
        // port carries, cut into CAN frames by the link.
        let (device, mut wire) = CanSerialDevice::test_wire(3);

        // SET_PORT_NODEID (0x03): names the device behind port 1.
        let set_port = set_port_node_id(1, 2, MUX_PORT_UNITS);
        device.send_set_port_node_id(1, 2, MUX_PORT_UNITS).unwrap();
        assert_unit_on_the_wire(&device, &mut wire, &set_port);

        // RESTART_DEVICE (0x05): for the device behind the port.
        let restart_device = admin_unit(&[CMD_RESTART_DEVICE, 1]);
        device.send_restart_device(1).unwrap();
        assert_unit_on_the_wire(&device, &mut wire, &restart_device);

        // RESET_PORT (0x04): for the bridge's own port.
        let reset_port = admin_unit(&[CMD_RESET_PORT, 1]);
        device.send_reset_port(1).unwrap();
        assert_unit_on_the_wire(&device, &mut wire, &reset_port);

        // REQUEST_BOOTLOADER (0x02): the flashing tool's command, the same
        // shape aimed at the device rather than at a port of its own.
        let bootloader = admin_unit(&[CMD_REQUEST_BOOTLOADER, 1]);
        device.send_request_bootloader(1, None).unwrap();
        assert_unit_on_the_wire(&device, &mut wire, &bootloader);

        // SET_PORT_TUNNEL (0x06): toggles port 1's flashing tunnel mode, the
        // same serial-side unit one command byte on.
        let set_port_tunnel = admin_unit(&[CMD_SET_PORT_TUNNEL, 1]);
        device.send_set_port_tunnel(1).unwrap();
        assert_unit_on_the_wire(&device, &mut wire, &set_port_tunnel);
    }

    #[test]
    fn test_a_unit_longer_than_a_can_frame_is_cut_and_rejoins() {
        // A unit is 15 bytes here, so it does not fit one CAN frame: the cut at
        // eight bytes lands *inside* the `KXID` block (the mux header shifts it),
        // six bytes into the block rather than between units.
        let unit = set_port_node_id(1, 2, MUX_PORT_UNITS);
        assert_eq!(unit.len(), 15, "the boundary case this test is for");

        let link = CanSerialLink::for_node(3).unwrap();
        let slices: Vec<CanFrame> = link.frames(&unit, CanWire::Classical).collect();
        assert_eq!(slices.len(), 2);
        assert_eq!(slices[0].data(), &unit[..CAN_DATA_BYTES]);
        assert_eq!(slices[1].data(), &unit[CAN_DATA_BYTES..]);
        assert_eq!(slices[0].data()[HEADER_BYTES..].len(), 6);

        // Rejoined, the slices carry the very same bytes, and they still read as
        // the admin unit they were built as: the checked header names the admin
        // id, and the command bytes sit inside the block. A unit is not a Klipper
        // message block — the `KXID` sequence byte fails `Frame::check` on purpose,
        // so a device cannot parse one by accident — which is why the command is
        // read by position here and not through a parsed `Frame`.
        let rejoined: Vec<u8> = slices.iter().flat_map(|f| f.data().to_vec()).collect();
        assert_eq!(rejoined, unit, "the cut loses nothing");
        assert_eq!(
            decode_header(&rejoined),
            HeaderScan::Complete {
                arb: ADMIN_LISTEN,
                consumed: HEADER_BYTES,
            }
        );
        assert_eq!(&rejoined[4..8], MAGIC);
        assert_eq!(
            &rejoined[8..12],
            &[CMD_SET_PORT_NODEID, 1, 2, MUX_PORT_UNITS]
        );
    }

    // ========================================================================
    // The CAN FD wire
    // ========================================================================

    /// The longest message the protocol carries: [`MESSAGE_PAYLOAD_MAX`] payload
    /// bytes, sixty-four with the header and trailer — exactly one CAN FD frame.
    fn longest_message(seq: u8) -> Frame {
        message(seq, &[0x5a; MESSAGE_PAYLOAD_MAX])
    }

    #[test]
    fn test_the_can_fd_wire_writes_seventy_two_byte_records() {
        // Two whole messages, each as large as the protocol allows, on a link
        // whose frame is wide enough for one of them: on the wire that is two 72
        // byte `canfd_frame` records, where a classical link would need ten 16
        // byte ones. The record size is what `read_wire_on` parses by, so a link
        // that wrote classical frames here would fail its own chunking.
        let (device, mut wire) = CanSerialDevice::test_wire_on(7, CanWire::Fd);
        let first = longest_message(1);
        let second = longest_message(2);
        assert_eq!(first.raw_bytes().len(), CAN_FD_DATA_BYTES);

        device.send(&first).unwrap();
        device.send(&second).unwrap();

        let records = read_wire_on(&mut wire, CanWire::Fd);
        assert_eq!(records.len(), 2, "one record per message");
        let tx_id = device.link.lock().unwrap().tx_id();
        assert!(records.iter().all(|record| record.id() == tx_id));
        assert_eq!(records[0].data(), first.raw_bytes().as_slice());
        assert_eq!(records[1].data(), second.raw_bytes().as_slice());
        assert_eq!(records[0].data().len(), CAN_FD_DATA_BYTES);
    }

    #[test]
    fn test_the_can_fd_wire_reads_seventy_two_byte_records() {
        // The same record coming the other way. The device polls its socket, so
        // the fake bus's far end writes the whole record first and `receive` then
        // finds a complete message in it.
        let (device, mut wire) = CanSerialDevice::test_wire_on(7, CanWire::Fd);
        let block = longest_message(3);

        let record = CanFrame::new(device.rx_id(), &block.raw_bytes()).unwrap();
        (&mut wire)
            .write_all(record.to_abi(CanWire::Fd).unwrap().as_bytes())
            .unwrap();

        assert_eq!(device.receive(), Some(block));
    }

    #[test]
    fn test_a_socket_write_has_a_deadline() {
        // A full send buffer is the one wait a CAN write has. The socket gets
        // `SO_SNDTIMEO`, and the kernel then reports the expiry instead of
        // blocking forever — which is what makes the write sites' `SendError`
        // reachable at all.
        let (socket, _reader) = UnixStream::pair().expect("a socket pair");
        let socket = File::from(OwnedFd::from(socket));
        set_send_timeout(&socket).expect("the deadline is set on a socket");

        // The option is the one asked for, and it is a whole `timeval`.
        let mut read_back = libc::timeval {
            tv_sec: -1,
            tv_usec: -1,
        };
        let mut len = std::mem::size_of::<libc::timeval>() as libc::socklen_t;
        // SAFETY: the option value is a `timeval`, and `len` says how much of it
        // the kernel may write.
        let rc = unsafe {
            libc::getsockopt(
                socket.as_raw_fd(),
                libc::SOL_SOCKET,
                libc::SO_SNDTIMEO,
                &mut read_back as *mut libc::timeval as *mut libc::c_void,
                &mut len,
            )
        };
        assert_eq!(rc, 0, "the option was set");
        // The kernel keeps this in timer ticks, so it reads back as the deadline
        // rounded *up* — never as the unbounded wait, which is zero.
        assert_eq!(read_back.tv_sec, 0, "a sub-second deadline");
        assert!(
            read_back.tv_usec >= SEND_TIMEOUT.as_micros() as libc::suseconds_t
                && read_back.tv_usec < 2 * SEND_TIMEOUT.as_micros() as libc::suseconds_t,
            "the deadline is {} ms, rounded up to a tick: {} us",
            SEND_TIMEOUT.as_millis(),
            read_back.tv_usec
        );

        // Fill the buffer with nobody reading: the write that cannot fit reports
        // the deadline (`EAGAIN`, which std calls `WouldBlock`) instead of
        // waiting for room that never comes.
        let started = Instant::now();
        let chunk = [0u8; 4096];
        let mut blocked = None;
        for _ in 0..1024 {
            match (&socket).write(&chunk) {
                Ok(_) => {}
                Err(e) => {
                    blocked = Some(e);
                    break;
                }
            }
            assert!(
                started.elapsed() < Duration::from_secs(10),
                "the test is bounded: filling the buffer must not take long"
            );
        }
        let err = blocked.expect("the send buffer fills, so one write has to wait");
        assert_eq!(
            err.kind(),
            std::io::ErrorKind::WouldBlock,
            "the expired deadline is reported, not another failure: {err}"
        );
        assert!(
            started.elapsed() < Duration::from_secs(5),
            "the wait ended at the deadline, not far past it: {:?}",
            started.elapsed()
        );
    }

    #[test]
    fn test_the_admin_socket_reads_and_writes_by_wire() {
        // An admin scan speaks the wire it was opened on: on CAN FD the broadcast
        // leaves as a 72 byte record and an answer arrives as one.
        let (socket, mut wire) = UnixStream::pair().expect("a socket pair");
        let mut admin = CanbusAdminSocket {
            socket: File::from(OwnedFd::from(socket)),
            interface: "test0".to_string(),
            wire: CanWire::Fd,
        };

        admin.send(&query_unassigned_frame()).unwrap();
        let records = read_wire_on(&mut wire, CanWire::Fd);
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].id(), ADMIN_ID);
        assert_eq!(records[0].data(), [CMD_QUERY_UNASSIGNED]);

        // The answer is the same frame shape as ever — the response byte, the six
        // UUID bytes — in the wire's larger envelope.
        let reply = CanFrame::new(
            ADMIN_ID + 1,
            &[RESP_NEED_NODEID, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66],
        )
        .unwrap();
        (&mut wire)
            .write_all(reply.to_abi(CanWire::Fd).unwrap().as_bytes())
            .unwrap();
        let received = admin
            .receive(Duration::from_millis(100))
            .expect("the fake bus does not fail")
            .expect("the record was there before the wait");
        assert_eq!(received, reply);
        assert_eq!(
            UnassignedNode::from_reply(&received).unwrap().uuid_hex(),
            "112233445566"
        );
    }
}
