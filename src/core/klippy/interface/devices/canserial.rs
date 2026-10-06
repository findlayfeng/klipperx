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
//! The bus carries one more conversation, which is *not* the byte stream: the
//! **admin** broadcast on id `0x3f0`, which is how a board that has no node id is
//! found and then given one. It is the same firmware module (`src/generic/canserial.c`,
//! "admin command handling") and the same sockets, so it lives here too: see
//! [`CanbusAdminSocket`], which asks every unassigned board to identify itself and
//! reads the answers.
//!
//! # Naming
//!
//! This is the *can serial* transport: Klipper's serial link, carried over CAN.
//! Type names therefore say `CanSerial`, and the name `Canbus` is kept for an
//! interface that speaks the CAN protocol itself rather than borrowing the bus as
//! a wire — which is what [`CanbusAdminSocket`] does with the admin broadcast.
//!
//! Configuration keys are a different matter: they follow Klipper's `[mcu]`
//! vocabulary (`canbus_uuid`, `canbus_interface`, and `canbus_nodeid` as Klipper's
//! own console and `serialhdl.connect_canbus` spell it), because those describe
//! the printer's wiring rather than this implementation, and a Klipper config
//! should keep working.

use crate::core::klippy::frame::{Frame, FrameStream};
use crate::core::klippy::interface::error::InterfaceError;
use crate::core::klippy::interface::{describe_frame, Device};
use std::ffi::CString;
use std::fmt;
use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::{AsRawFd, FromRawFd, RawFd};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};
use tracing::{debug, info, trace, warn};

/// Data bytes in one classical CAN frame.
pub const CAN_DATA_BYTES: usize = 8;

/// Size of the kernel's `struct can_frame`, which is what a raw socket reads and
/// writes one datagram at a time.
const CAN_FRAME_SIZE: usize = 16;

/// Klipper's node-id mapping: `nodeid` → `0x100 + 2 * nodeid`.
const NODE_ID_BASE: u32 = 0x100;

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

/// One CAN frame as the bus carries it: its arbitration id, then its eight data
/// bytes — a *slice* of a message block rather than a block of its own.
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
    pub fn next_frame(&mut self) -> Option<Frame> {
        self.incoming.next_frame()
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
        trace!("tx frame [{}]: {}", self.id(), describe_frame(&bytes));
        let frames: Vec<CanFrame> = self.link.lock().unwrap().frames(&bytes).collect();
        for can_frame in &frames {
            // The block's bytes as the bus sees them, one CAN frame at a time.
            trace!(
                "tx can frame [{}]: {}",
                self.id(),
                describe_can_frame(can_frame.id(), can_frame.data())
            );
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
            // The slices came together: the block is logged the way the serial
            // port logs one — however many CAN frames it took, even one.
            if let Some(frame) = self.link.lock().unwrap().next_frame() {
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
                Ok(read) if read >= CAN_FRAME_SIZE => {
                    let can_frame = CanFrame::from_abi(&raw);
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
}

impl CanbusAdminSocket {
    /// Open `interface` and listen for answers to the admin broadcast.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] when the interface does not exist, or the
    /// socket cannot be created, bound or filtered.
    pub fn open(interface: &str) -> Result<Self, InterfaceError> {
        let index = interface_index(interface)?;
        let socket = open_socket()?;
        bind_interface(&socket, index)?;
        filter_answers(&socket, ADMIN_ID + 1)?;
        debug!(
            "admin socket ready on {interface}, listening for {:#x}",
            ADMIN_ID + 1
        );
        Ok(Self {
            socket,
            interface: interface.to_string(),
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
            .write_all(&frame.to_abi())
            .map_err(|e| InterfaceError::SendError(format!("CAN write failed: {e}")))
    }

    fn receive(&mut self, wait: Duration) -> Result<Option<CanFrame>, InterfaceError> {
        let mut raw = [0u8; CAN_FRAME_SIZE];
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
                Ok(read) if read >= CAN_FRAME_SIZE => return Ok(Some(CanFrame::from_abi(&raw))),
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
enum Wait {
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
fn wait_readable(fd: RawFd, timeout: Duration) -> Wait {
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
    fn test_the_bus_and_the_block_are_described_differently() {
        // A block small enough for one CAN frame: the bus line names the
        // arbitration id and shows the eight data bytes, while the block line is
        // the serial frame. Both are logged even though one CAN frame carried it
        // all, because a slice and a block are not the same thing.
        let block = message(1, &[0xaa, 0xbb]);
        let bytes = block.raw_bytes();
        assert_eq!(bytes.len(), 7);

        let link = CanSerialLink::for_node(5);
        let slices: Vec<CanFrame> = link.frames(&bytes).collect();
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
        let sender = CanSerialLink::for_node(5);
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
        let mut link = CanSerialLink::for_node(5);
        let foreign = CanFrame::new(0x123, &[1, 2, 3]).unwrap();

        assert!(!link.accept(&foreign));
        assert_eq!(link.next_frame(), None);

        // ... and a frame of ours behind it is still accepted.
        let ours = CanFrame::new(link.rx_id(), &message(2, &[9]).raw_bytes()).unwrap();
        assert!(link.accept(&ours));
        assert_eq!(link.next_frame(), Some(message(2, &[9])));
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

        // The admin socket is the same first step, and reports it the same way.
        let err = CanbusAdminSocket::open("can99").unwrap_err();
        assert!(
            err.to_string().contains("no CAN interface named 'can99'"),
            "{err}"
        );
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

        let raw = frame.to_abi();
        assert_eq!(u32::from_ne_bytes([raw[0], raw[1], raw[2], raw[3]]), 0x3f0);
        assert_eq!(raw[4], 1, "one data byte");
        assert_eq!(&raw[8..9], [0x00]);
        assert_eq!(&raw[9..], [0; CAN_DATA_BYTES - 1], "no padding is sent");
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
}
