//! Klipper's message blocks carried over CAN **ISO-TP** (ISO 15765-2): the
//! segmented CAN link.
//!
//! [`CanSerialDevice`](super::canserial::CanSerialDevice) borrows the bus as a
//! byte stream: it cuts Klipper's blocks every eight bytes and reassembles the
//! stream on the other side, which works because a *stream* has no frame
//! boundaries to respect. ISO-TP takes the other road and makes each block a
//! **message** with boundaries of its own: a block that does not fit one CAN
//! frame is segmented into a first frame (`FF`) and consecutive frames (`CF`),
//! and the receiver answers with a flow control frame (`FC`) saying how much of
//! the rest may follow at once. That costs a round trip and a couple of frames
//! per block, and buys two things: a classical CAN link's eight bytes stop being
//! a limit — a Klipper block runs to
//! [`MESSAGE_MAX`](crate::core::klippy::frame::MESSAGE_MAX) — and a lost CAN
//! frame aborts one message instead of desynchronising the whole byte stream.
//!
//! The protocol itself is [`IsotpState`] in [`super::isotp_proto`]: pure, with
//! no socket, no clock and no thread. This module is the other half — a blocking
//! raw SocketCAN socket, one kernel frame per read and write, and the byte
//! stream ([`FrameStream`]) the reassembled messages feed. One rule joins them:
//! the state machine takes a *message* in and hands *frames* back, so a step
//! here writes the frames it names, in the order it names them, and pushes the
//! messages it reassembles into the stream.
//!
//! # The timebase
//!
//! [`IsotpState::step`] takes `now` as a [`Duration`] and only ever compares
//! such values; the clock is the caller's. That caller is this device, so the
//! origin ([`Instant::now`] at open) lives here, and so does the waiting: a
//! receive waits on the socket for the shorter of its poll interval and the
//! moment the state machine said a `Poll` could change something. That second
//! bound is what keeps a *waiting* transfer moving while the bus is silent —
//! a deadline that has run out, or the `STmin` gate on the next consecutive
//! frame.
//!
//! # Addressing and the wire
//!
//! The node's arbitration ids are the shared CAN mapping
//! ([`assign::arb_id`]): the host writes `0x100 + 2 * nodeid` and the MCU answers
//! on the next id, so `tx_id`/`rx_id` are the same two ids the can-serial link
//! uses and the kernel filter takes the same argument. The wire ([`CanWire`])
//! decides the frame capacity the state machine is built for ([`IsotpConfig`])
//! and cannot be changed at run time: a peer on the other wire is inaudible,
//! because the kernel drops the frames rather than reporting them.
//!
//! The node id is the host's to assign, over the admin channel, exactly as for
//! the can-serial link: the same frame on [`ADMIN_ID`], from the same socket,
//! before any data. It is a channel of its own — no ISO-TP message, no flow
//! control, no answer.
//!
//! # Not here
//!
//! Which `[mcu]` key asks for this transport, and the firmware's ISO-TP slave,
//! are later units; this one is the device and its wiring into
//! [`Interface`](super::super::Interface). The bridge port-level commands
//! (`SET_PORT_NODEID` and the resets) travel a *byte stream's* admin channel and
//! have no ISO-TP shaping yet, so an ISO-TP interface answers them the way a
//! transport with no bridge does.

use super::canserial::{
    bind_interface, describe_can_frame, filter_answers, interface_index, open_socket,
    set_nodeid_payload, wait_readable, CanFrame, CanWire, Wait, ADMIN_ID, CAN_FD_FRAME_SIZE,
    POLL_TIMEOUT,
};
use super::isotp_proto::{Action, Input, IsotpConfig, IsotpError, IsotpState};
use crate::core::klippy::assign;
use crate::core::klippy::config::mcu::MAX_MCU_ID;
use crate::core::klippy::frame::{Frame, FrameStream};
use crate::core::klippy::interface::error::InterfaceError;
use crate::core::klippy::interface::{describe_frame, Device};
use std::fmt;
use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::AsRawFd;
#[cfg(test)]
use std::os::fd::OwnedFd;
#[cfg(test)]
use std::os::unix::net::UnixDatagram;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};
use tracing::{debug, info, trace, warn};

/// The half of the transport that touches no socket: one ISO-TP state machine
/// and the Klipper byte stream its reassembled messages feed.
#[derive(Debug)]
struct IsotpLink {
    state: IsotpState,
    /// Whole messages reassembled so far, waiting to become complete frames.
    incoming: FrameStream,
    /// The earliest moment at which another `Poll` could change anything, on the
    /// device's timebase; `None` while nothing is in flight.
    ///
    /// This is the state machine's own `Wait`, remembered so the receive loop
    /// can bound its socket wait by it: waking earlier is a wakeup nobody
    /// needed, waking later is a timeout noticed late.
    next_poll: Option<Duration>,
}

impl IsotpLink {
    /// A link that has sent and received nothing.
    fn new(config: IsotpConfig) -> Self {
        Self {
            state: IsotpState::new(config),
            incoming: FrameStream::new(),
            next_poll: None,
        }
    }
}

/// What one [`IsotpState::step`] left the link in, once its actions have been
/// carried out.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stepped {
    /// Nothing is in flight: a receive should simply wait for the bus.
    Idle,
    /// Nothing can change before this moment (absolute, on the device's
    /// timebase): a receive should wake at it and step again.
    Wait(Duration),
    /// The transfer failed. Both directions were reset by the state machine, so
    /// the link is idle again and the next message can go; a send reports it, a
    /// receive logs it and carries on.
    Failed(IsotpError),
}

/// A [`Device`] on a SocketCAN interface (`can0`, `/sys/class/net/*` names)
/// carrying ISO-TP.
///
/// The link half is unit tested; the socket half cannot be exercised in this
/// repository's test environment — it has no CAN interface and `vcan` needs
/// privileges to load — so it is covered by compilation only, and its tests
/// drive the device over a socket pair standing in for the CAN socket. The shape
/// is the can-serial device's: raw socket, bind to the interface index, kernel
/// filter for the MCU's answer id, one frame per read and write, sized by
/// [`CanWire`].
pub struct IsotpDevice {
    socket: File,
    interface: String,
    /// This MCU's MCU-layer id, as the configuration assigned it: on this CAN
    /// link it is the node id.
    nodeid: u32,
    /// The link's shape: what one frame holds and how large a datagram is.
    wire: CanWire,
    /// The id the MCU listens on: `0x100 + 2 * nodeid`.
    tx_id: u32,
    /// The id the MCU answers on, and the only one this device reads.
    rx_id: u32,
    /// The state machine and the messages it reassembles, under one lock: one
    /// step advances both directions, so they are one object, not two.
    link: Mutex<IsotpLink>,
    stopped: AtomicBool,
    /// The origin of the state machine's timebase. The state machine only ever
    /// compares `Duration`s, so which instant they are measured from is the
    /// device's business.
    started: Instant,
}

impl IsotpDevice {
    /// Open `interface` on `wire` and bring the MCU `uuid` up as node `nodeid`.
    ///
    /// The node id lives in the host, not in the MCU: this sends Klipper's admin
    /// frame — the same one the can-serial link sends, from the same socket —
    /// before any traffic, and the MCU starts answering on the node's
    /// arbitration ids only once it has seen its UUID and new id. Nothing else
    /// is shared with that link: the socket is the same recipe, but the blocks
    /// that follow are ISO-TP messages rather than a chopped byte stream.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] when the node id is outside `1..=MAX_MCU_ID`
    /// (`0xff`), the interface does not exist, the socket cannot be created, it
    /// cannot be bound or filtered, CAN FD cannot be enabled on it, or the node
    /// id could not be handed over.
    pub fn open(
        interface: &str,
        uuid: [u8; 6],
        nodeid: u32,
        wire: CanWire,
    ) -> Result<Self, InterfaceError> {
        // Checked before the socket is opened: an unaddressable node id is a
        // configuration mistake, not a bus one, and the bus is not touched for
        // it. Both directions come from the one mapping, so they are resolved
        // together and fail together.
        let (tx_id, rx_id) = match (assign::arb_id(nodeid), assign::reply_arb_id(nodeid)) {
            (Some(tx_id), Some(rx_id)) => (tx_id, rx_id),
            _ => {
                return Err(InterfaceError::Other(format!(
                    "node id {nodeid} is outside the CAN range 1..={MAX_MCU_ID}"
                )))
            }
        };

        let index = interface_index(interface)?;
        let socket = open_socket(wire)?;
        bind_interface(&socket, index)?;
        // Only the node's answers are this link's business; the rest of the bus
        // belongs to whoever else is on it.
        filter_answers(&socket, rx_id)?;

        // The admin frame travels on its own arbitration id, outside both the
        // filter above (which is about what this socket reads) and the ISO-TP
        // addresses (which are about messages).
        let assignment = CanFrame::new(ADMIN_ID, &set_nodeid_payload(uuid, nodeid))?;
        (&socket)
            .write_all(assignment.to_abi(wire)?.as_bytes())
            .map_err(|e| InterfaceError::Other(format!("failed to send the node id: {e}")))?;

        info!(
            "ISO-TP link ready on {interface} ({}): node {nodeid} assigned, writing to \
             {tx_id:#x}, reading {rx_id:#x}",
            wire.name(),
        );
        Ok(Self {
            socket,
            interface: interface.to_string(),
            nodeid,
            wire,
            tx_id,
            rx_id,
            link: Mutex::new(IsotpLink::new(config_for(wire))),
            stopped: AtomicBool::new(false),
            started: Instant::now(),
        })
    }

    /// Returns a short identifier for logging: `"can0:0x106"`.
    fn id(&self) -> String {
        format!("{}:{:#x}", self.interface, self.tx_id)
    }

    /// The current time on the state machine's timebase.
    fn now(&self) -> Duration {
        self.started.elapsed()
    }

    /// Advance the state machine by one step and carry out what it says, in the
    /// order it says it.
    ///
    /// The frames are written after the lock is released, but still in order:
    /// one step can hand back several consecutive frames at once (`BS = 0` sends
    /// the whole remainder), and only their order keeps the sequence numbers
    /// meaningful.
    ///
    /// A protocol failure is logged here, whichever direction it was seen from,
    /// so a transfer that dies is never silent.
    ///
    /// # Errors
    /// The socket's: a frame could not be written.
    fn step(&self, now: Duration, input: Input<'_>) -> Result<Stepped, InterfaceError> {
        let (transmits, outcome) = {
            let mut link = self.link.lock().unwrap();
            let actions = link.state.step(now, input);
            let mut transmits = Vec::new();
            let mut outcome = Stepped::Idle;
            for action in actions {
                match action {
                    // Already framed — PCI in place, no padding, addressed to
                    // the node — so the wire's only business is the length.
                    Action::Transmit(data) => transmits.push(data),
                    Action::Received(message) => {
                        trace!("rx isotp message [{}]: {} bytes", self.id(), message.len());
                        link.incoming.push(&message);
                    }
                    Action::Wait { next_poll } => outcome = Stepped::Wait(next_poll),
                    Action::Failed(error) => outcome = Stepped::Failed(error),
                }
            }
            link.next_poll = match outcome {
                Stepped::Wait(deadline) => Some(deadline),
                Stepped::Idle | Stepped::Failed(_) => None,
            };
            (transmits, outcome)
        };
        if let Stepped::Failed(error) = outcome {
            warn!("ISO-TP transfer on {} failed: {error:?}", self.interface);
        }
        for data in &transmits {
            self.write_frame(data)?;
        }
        Ok(outcome)
    }

    /// Put one ISO-TP frame on the bus.
    ///
    /// `data` is a whole frame's data bytes with its PCI already in place
    /// ([`Action::Transmit`]); the wire decides whether it fits, and a frame
    /// that does not is refused rather than truncated.
    fn write_frame(&self, data: &[u8]) -> Result<(), InterfaceError> {
        trace!(
            "tx isotp frame [{}]: {}",
            self.id(),
            describe_can_frame(self.tx_id, data)
        );
        let frame = CanFrame::new(self.tx_id, data)?;
        (&self.socket)
            .write_all(frame.to_abi(self.wire)?.as_bytes())
            .map_err(|e| InterfaceError::SendError(format!("CAN write failed: {e}")))
    }

    /// How long a receive may wait on the socket before it has to step the state
    /// machine again.
    ///
    /// The shorter of the poll interval — which is what bounds how long a
    /// shutdown takes to be noticed — and what is left of the state machine's
    /// wait. A deadline already past means a whole millisecond, because `poll`
    /// counts in milliseconds and a zero timeout would spin.
    fn wait_timeout(&self) -> Duration {
        match self.link.lock().unwrap().next_poll {
            Some(deadline) => deadline.saturating_sub(self.now()).min(POLL_TIMEOUT),
            None => POLL_TIMEOUT,
        }
    }
}

impl fmt::Debug for IsotpDevice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("IsotpDevice")
            .field("interface", &self.interface)
            .field("nodeid", &self.nodeid)
            .field("wire", &self.wire)
            .field("stopped", &self.stopped.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Device for IsotpDevice {
    /// Segment `frame` and put its first frame or frames on the bus.
    ///
    /// A block that fits one CAN frame is one write, and the call is done. A
    /// longer block gets its first frame out and then waits for the peer's flow
    /// control frame, which arrives on the read side: the consecutive frames go
    /// out from [`Device::receive`]'s loop when that frame is read (or from the
    /// `Poll` it takes at the deadline if the link needs one). A caller that
    /// never reads therefore cannot finish a segmented message — which is fine,
    /// because a transport is driven by one thread that sends and receives at
    /// the same time.
    ///
    /// # Errors
    /// [`InterfaceError::ConnectionLost`] when the device is shut down; a
    /// protocol failure is reported rather than dropped, so the caller sees the
    /// frame did not go ([`IsotpError::TxBusy`] being the one a send can bring
    /// on itself by starting before the previous message finished — and the
    /// state machine resets both directions on any failure, so a refused send
    /// also abandons the message in flight); a socket write failure is an
    /// [`InterfaceError::SendError`].
    fn send(&self, frame: &Frame) -> Result<(), InterfaceError> {
        if self.stopped.load(Ordering::Relaxed) {
            return Err(InterfaceError::ConnectionLost);
        }
        let bytes = frame.raw_bytes();
        trace!("tx frame [{}]: {}", self.id(), describe_frame(&bytes));
        match self.step(self.now(), Input::Send(&bytes)) {
            Ok(Stepped::Failed(error)) => Err(send_failure(error)),
            Ok(_) => Ok(()),
            Err(error) => Err(error),
        }
    }

    /// Read frames until one whole Klipper block has been reassembled.
    ///
    /// This loop is the link's transmit pump as well: consecutive frames are
    /// written from here, when the flow control frame that lets them go is read
    /// (and, for a gate or a deadline, from the `Poll` the loop takes at the
    /// state machine's own time). `None` means the device was shut down or the
    /// socket failed — never "nothing yet".
    fn receive(&self) -> Option<Frame> {
        // The largest datagram either wire delivers; a classical read fills the
        // first sixteen bytes of it.
        let mut raw = [0u8; CAN_FD_FRAME_SIZE];
        loop {
            if self.stopped.load(Ordering::Relaxed) {
                return None;
            }
            // A whole block may already be here: an earlier step reassembled it,
            // from this loop or from a send's step, and several messages can be
            // waiting in the stream.
            if let Some(frame) = self.link.lock().unwrap().incoming.next_frame() {
                trace!(
                    "rx frame [{}]: {}",
                    self.id(),
                    describe_frame(&frame.raw_bytes())
                );
                return Some(frame);
            }

            let stepped = match wait_readable(self.socket.as_raw_fd(), self.wait_timeout()) {
                // A signal: wait again rather than throwing a frame away.
                Wait::Retry => continue,
                // The interface went away.
                Wait::Closed => return None,
                // Nothing on the bus in the time we were allowed to wait:
                // service the timers instead of leaving a transfer stuck until
                // some frame happens to arrive.
                Wait::Timeout => self.step(self.now(), Input::Poll),
                Wait::Readable => match (&self.socket).read(&mut raw) {
                    // One datagram is one frame, on either wire.
                    Ok(read) if read >= self.wire.frame_size() => {
                        let can_frame = CanFrame::from_abi(self.wire, &raw);
                        // The kernel filter should have kept everybody else's
                        // frames out, but it is not what makes the link correct:
                        // the id is.
                        if can_frame.id() != self.rx_id {
                            debug!("ignoring CAN frame for id {:#x}", can_frame.id());
                            continue;
                        }
                        trace!(
                            "rx isotp frame [{}]: {}",
                            self.id(),
                            describe_can_frame(can_frame.id(), can_frame.data())
                        );
                        self.step(self.now(), Input::Frame(can_frame.data()))
                    }
                    // A datagram socket delivers whole frames; treat a short
                    // read as someone else's frame rather than guessing.
                    Ok(read) => {
                        debug!("ignoring {read} byte CAN read");
                        continue;
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                    Err(e) => {
                        warn!("CAN read on {} failed: {e}", self.interface);
                        return None;
                    }
                },
            };
            // `Stepped::Failed` needs nothing here: `step` logged it, both
            // directions are already reset, and the next frame starts a fresh
            // message — so the loop goes on rather than returning a half frame.
            if let Err(error) = stepped {
                // The only error a step returns is the socket's: nothing could
                // be written, so there is no link left to read.
                warn!("CAN write on {} failed: {error}", self.interface);
                return None;
            }
        }
    }

    /// Set the stop flag. The read loop checks it between socket polls, so a
    /// blocked `receive` returns within one poll interval instead of waiting for
    /// a frame that may never come.
    fn shutdown(&self) {
        self.stopped.store(true, Ordering::SeqCst);
    }
}

/// The state machine capacity a wire implies: eight data bytes per frame on a
/// classical CAN link, sixty-four on CAN FD.
///
/// A link is one wire, never both ([`CanWire`]), so this is a translation rather
/// than a choice.
fn config_for(wire: CanWire) -> IsotpConfig {
    match wire {
        CanWire::Classical => IsotpConfig::classical(),
        CanWire::Fd => IsotpConfig::can_fd(),
    }
}

/// How an [`IsotpError`] is reported to a send.
///
/// The message never made it onto the wire, and which failure it was matters: a
/// send that starts while the previous message is still in flight is a caller
/// that has to wait for the link, not a broken one.
fn send_failure(error: IsotpError) -> InterfaceError {
    match error {
        IsotpError::TxBusy => {
            InterfaceError::Other("ISO-TP: the previous message is still in flight".to_string())
        }
        other => InterfaceError::SendError(format!("ISO-TP send failed: {other:?}")),
    }
}

/// The device on a **wire a test can read**: a socket pair stands in for the CAN
/// socket, and the returned end carries the records the device writes (read back
/// with [`read_wire_on`]) and the frames the peer sends.
///
/// The pair is a **datagram** one, not a stream: CAN is message oriented — one
/// read is one frame — and a stream socket would hide that by merging whatever
/// was written in one go, which is exactly the distinction this module rests on.
///
/// Opening a real CAN interface needs privileges this repository's test
/// environment does not have, so the unit tests drive the device the way the
/// can-serial device's do: over a socket pair, with the peer played by the test.
#[cfg(test)]
impl IsotpDevice {
    /// A device on `wire`'s fake bus.
    pub(crate) fn test_wire_on(nodeid: u32, wire: CanWire) -> (Self, UnixDatagram) {
        let (socket, peer) = UnixDatagram::pair().expect("a socket pair");
        let tx_id = assign::arb_id(nodeid).expect("a test node id is in the CAN range");
        let rx_id = assign::reply_arb_id(nodeid).expect("a test node id is in the CAN range");
        let device = Self {
            socket: File::from(OwnedFd::from(socket)),
            interface: "test0".to_string(),
            nodeid,
            wire,
            tx_id,
            rx_id,
            link: Mutex::new(IsotpLink::new(config_for(wire))),
            stopped: AtomicBool::new(false),
            started: Instant::now(),
        };
        (device, peer)
    }
}

/// Every CAN record the device on a [`test wire`](IsotpDevice::test_wire_on) has
/// written, taken off the peer end one datagram at a time, the way a receiving
/// link reads the bus.
///
/// The peer is left non-blocking, which is how a test tells "no more records"
/// from "wait for one".
#[cfg(test)]
pub(crate) fn read_wire_on(peer: &UnixDatagram, wire: CanWire) -> Vec<CanFrame> {
    peer.set_nonblocking(true)
        .expect("the test wire takes a non-blocking flag");
    let mut frames = Vec::new();
    let mut raw = [0u8; CAN_FD_FRAME_SIZE];
    loop {
        match peer.recv(&mut raw) {
            Ok(read) if read >= wire.frame_size() => frames.push(CanFrame::from_abi(wire, &raw)),
            Ok(read) => panic!("a CAN record is a whole frame, not {read} bytes"),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
            Err(e) => panic!("reading the test wire failed: {e}"),
        }
    }
    frames
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;
    use std::sync::Arc;

    /// A Klipper frame whose wire bytes run to `MESSAGE_MIN + payload.len()`.
    fn block(seq: u8, payload: &[u8]) -> Frame {
        Frame::new(seq, payload.to_vec())
    }

    /// The block `queue_step` sends: eleven payload bytes, sixteen on the wire —
    /// the shape the CAN ISO-TP notes single out as the segmented case.
    fn queue_step() -> Frame {
        block(1, &[0xab; 11])
    }

    /// One CAN record as the bus carries it, from the test's side of the socket
    /// pair: the wire's own layout, and its length.
    fn record(id: u32, data: &[u8], wire: CanWire) -> Vec<u8> {
        CanFrame::new(id, data)
            .expect("a test frame fits one CAN frame")
            .to_abi(wire)
            .expect("a test frame fits this wire")
            .as_bytes()
            .to_vec()
    }

    /// A sixteen byte block is a first frame and, once the peer clears it, two
    /// consecutive frames — on the node's arbitration ids, in sequence order.
    #[test]
    fn test_a_long_block_is_segmented_after_the_peers_flow_control() {
        let (device, mut peer) = IsotpDevice::test_wire_on(3, CanWire::Classical);
        let frame = queue_step();
        let raw = frame.raw_bytes();
        assert_eq!(raw.len(), 16, "eleven payload bytes plus the header");

        device.send(&frame).expect("the first frame goes out");

        // The first frame carries the length and the first six bytes. The
        // consecutive frames may not follow until the peer says so.
        let sent = read_wire_on(&mut peer, CanWire::Classical);
        assert_eq!(sent.len(), 1, "only the first frame until the peer clears");
        assert_eq!(sent[0].id(), 0x106, "the host writes 0x100 + 2 * nodeid");
        assert_eq!(sent[0].data(), [&[0x10u8, 16][..], &raw[..6]].concat());

        // The peer clears the block (CTS, BS = 0 — the whole remainder) and
        // sends a block of its own, so the receive loop has a frame to return.
        peer.send(&record(0x107, &[0x30, 0x00, 0x00], CanWire::Classical))
            .expect("the flow control frame");
        let answer = block(2, &[0x0d, 0x0e]);
        let mut single = vec![answer.raw_bytes().len() as u8];
        single.extend_from_slice(&answer.raw_bytes());
        peer.send(&record(0x107, &single, CanWire::Classical))
            .expect("the peer's single frame");

        assert_eq!(
            device.receive().expect("the peer's block").raw_bytes(),
            answer.raw_bytes()
        );

        // The two consecutive frames, carrying the rest of the block in order:
        // the sequence numbers are what makes an out-of-order write a different
        // message.
        let sent = read_wire_on(&mut peer, CanWire::Classical);
        assert_eq!(sent.len(), 2, "the two consecutive frames, once cleared");
        assert_eq!(sent[0].data(), [&[0x21u8][..], &raw[6..13]].concat());
        assert_eq!(sent[1].data(), [&[0x22u8][..], &raw[13..16]].concat());
    }

    /// The other direction: a first frame and two consecutive frames off the bus
    /// reassemble into one Klipper frame, and the device clears the block with a
    /// flow control frame of its own.
    #[test]
    fn test_an_inbound_segmented_message_becomes_one_frame() {
        let (device, mut peer) = IsotpDevice::test_wire_on(3, CanWire::Classical);
        let frame = queue_step();
        let raw = frame.raw_bytes();

        let mut first = vec![0x10u8, raw.len() as u8];
        first.extend_from_slice(&raw[..6]);
        peer.send(&record(0x107, &first, CanWire::Classical))
            .expect("the first frame");
        let mut consecutive = vec![0x21u8];
        consecutive.extend_from_slice(&raw[6..13]);
        peer.send(&record(0x107, &consecutive, CanWire::Classical))
            .expect("the first consecutive frame");
        consecutive = vec![0x22u8];
        consecutive.extend_from_slice(&raw[13..16]);
        peer.send(&record(0x107, &consecutive, CanWire::Classical))
            .expect("the second consecutive frame");

        assert_eq!(
            device.receive().expect("the reassembled block").raw_bytes(),
            raw
        );

        // A flow control frame of the device's own: CTS, BS = 0, STmin = 0, and
        // no padding — the bytes a sender is given are the bytes it reads.
        let sent = read_wire_on(&mut peer, CanWire::Classical);
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].id(), 0x106);
        assert_eq!(sent[0].data(), &[0x30u8, 0x00, 0x00][..]);
    }

    /// A CAN FD link carries sixty-four data bytes per frame, so the largest
    /// Klipper block is a first frame plus one consecutive frame — and every
    /// datagram on the wire is seventy-two bytes.
    #[test]
    fn test_a_can_fd_link_segments_at_its_own_frame_size() {
        let (device, mut peer) = IsotpDevice::test_wire_on(7, CanWire::Fd);
        let frame = block(1, &[0x5a; 59]);
        let raw = frame.raw_bytes();
        assert_eq!(raw.len(), 64, "the largest Klipper block");

        device.send(&frame).expect("the first frame goes out");

        let sent = read_wire_on(&mut peer, CanWire::Fd);
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].id(), 0x10e, "the host writes 0x100 + 2 * nodeid");
        assert_eq!(
            sent[0].data().len(),
            64,
            "a CAN FD frame carries sixty-four"
        );
        assert_eq!(sent[0].data(), [&[0x10u8, 64][..], &raw[..62]].concat());

        peer.send(&record(0x10f, &[0x30, 0x00, 0x00], CanWire::Fd))
            .expect("the flow control frame");
        let answer = block(2, &[0x0d, 0x0e]);
        let mut single = vec![answer.raw_bytes().len() as u8];
        single.extend_from_slice(&answer.raw_bytes());
        peer.send(&record(0x10f, &single, CanWire::Fd))
            .expect("the peer's single frame");

        assert_eq!(
            device.receive().expect("the peer's block").raw_bytes(),
            answer.raw_bytes()
        );

        let sent = read_wire_on(&mut peer, CanWire::Fd);
        assert_eq!(sent.len(), 1, "one consecutive frame carries the last two");
        assert_eq!(sent[0].data(), [&[0x21u8][..], &raw[62..64]].concat());
    }

    /// On a CAN FD link a sixteen byte block is **one** frame: sixty-two bytes
    /// fit a single frame, so the escape form carries the length and the block.
    #[test]
    fn test_a_can_fd_link_carries_a_short_block_in_one_frame() {
        let (device, mut peer) = IsotpDevice::test_wire_on(7, CanWire::Fd);
        let frame = queue_step();
        let raw = frame.raw_bytes();

        device.send(&frame).expect("one frame carries the block");

        let sent = read_wire_on(&mut peer, CanWire::Fd);
        assert_eq!(sent.len(), 1, "one frame, no flow control");
        assert_eq!(sent[0].data(), [&[0x00u8, 16][..], &raw[..]].concat());
    }

    /// A second block may not start while the first is still being segmented:
    /// the state machine refuses it and the send says so, rather than dropping
    /// the frame silently.
    #[test]
    fn test_a_send_while_a_block_is_in_flight_is_refused() {
        let (device, _peer) = IsotpDevice::test_wire_on(3, CanWire::Classical);

        device
            .send(&queue_step())
            .expect("the first frame goes out");

        assert_eq!(
            device.send(&block(2, &[0x01])).unwrap_err(),
            InterfaceError::Other("ISO-TP: the previous message is still in flight".to_string())
        );
    }

    /// The node id is checked before the bus is touched, so an unaddressable one
    /// is reported for an interface that does not exist: were the socket opened
    /// first, the missing interface would be the complaint instead.
    #[test]
    fn test_open_rejects_a_node_id_outside_the_range() {
        let error = IsotpDevice::open("can99", [0; 6], 0x100, CanWire::Classical).unwrap_err();
        assert_eq!(
            error.to_string(),
            format!("node id 256 is outside the CAN range 1..={MAX_MCU_ID}")
        );

        // Zero is outside the range too: those bits name no node.
        let error = IsotpDevice::open("can99", [0; 6], 0, CanWire::Classical).unwrap_err();
        assert!(
            error.to_string().contains("node id 0 is outside"),
            "{error}"
        );
    }

    /// One wire is one frame capacity, and the state machine is built for
    /// exactly it: a mismatch would size every frame of the link wrongly.
    #[test]
    fn test_a_wires_frame_capacity_is_the_state_machines() {
        for wire in [CanWire::Classical, CanWire::Fd] {
            assert_eq!(config_for(wire).frame_bytes, wire.data_bytes());
        }
    }

    /// A receive with nothing on the bus waits; `shutdown` is what ends it, and
    /// no frame is invented to return.
    ///
    /// The wait is watched from a thread, and the *result* is read with a
    /// timeout rather than by joining blind: a receive that ignored `shutdown`
    /// fails this test instead of hanging it.
    #[test]
    fn test_shutdown_ends_a_waiting_receive() {
        let (device, _peer) = IsotpDevice::test_wire_on(3, CanWire::Classical);
        let device = Arc::new(device);
        let reader = Arc::clone(&device);
        let (started, start) = mpsc::channel();
        let (done, finished) = mpsc::channel();
        let handle = std::thread::spawn(move || {
            started.send(()).expect("the test is listening");
            let _ = done.send(reader.receive());
        });
        start
            .recv_timeout(Duration::from_secs(1))
            .expect("the reader started");

        device.shutdown();

        assert_eq!(
            finished
                .recv_timeout(Duration::from_secs(1))
                .expect("shutdown must end the wait, not block until a frame arrives"),
            None
        );
        handle.join().expect("the reader thread");
    }
}
