//! Pure ISO 15765-2 (ISO-TP) segmentation and reassembly state machine.
//!
//! This module owns the ISO-TP wire protocol only: it turns one Klipper message
//! into the frame sequence (`SF` / `FF` / `CF`, separated by flow control
//! frames) and turns received frames back into one message. It performs no I/O,
//! spawns no thread, reads no clock and never sleeps — [`IsotpState::step`]
//! receives the caller's monotonic timebase and returns the [`Action`]s the
//! caller has to carry out.
//!
//! # Frame format
//!
//! | frame | byte 0 | byte 1 | byte 2… | payload |
//! |---|---|---|---|---|
//! | single frame (SF) | `0x0L`, `L = 1..=7` | data | … | `L` bytes |
//! | single frame, escape form | `0x00` | length | data | `length` bytes |
//! | first frame (FF) | `0x1H`, `H` = length bits 11:8 | length bits 7:0 | data | up to `frame_bytes - 2` |
//! | consecutive frame (CF) | `0x2N`, `N` = sequence number | data | … | up to `frame_bytes - 1` |
//! | flow control (FC) | `0x3F`, `F`: 0 = CTS, 1 = WAIT, 2 = OVFLW | block size | `STmin` | — |
//!
//! Sequence numbers start at 1, increment by one and wrap modulo 16. `STmin` is
//! the minimum gap between consecutive frames: `0x00..=0x7F` are milliseconds,
//! `0xF1..=0xF9` are 100…900 microseconds, reserved values are treated as
//! `0x7F` (127 ms) the way ISO 15765-2 requires of a receiver. `BS == 0` means
//! "send the remainder of the message without waiting for another flow control
//! frame"; `BS > 0` stops the sender after that many consecutive frames.
//!
//! The two timeouts are N_Bs (waiting for a flow control frame) and N_Cr
//! (waiting for the next consecutive frame), both [`N_BS_TIMEOUT`] /
//! [`N_CR_TIMEOUT`].
//!
//! # Frame capacity
//!
//! The number of data bytes in one CAN frame is a parameter
//! ([`IsotpConfig::frame_bytes`]), never a hard-coded 8. Classical CAN gives
//! `SF <= 7`, `FF <= 6`, `CF <= 7`; CAN-FD with `tx_dl = 64` gives `SF <= 62`
//! (escape form), `FF <= 62`, `CF <= 63`.
//!
//! # Full duplex
//!
//! Both directions run at the same time and are dispatched by PCI type, so an
//! incoming `FF` can be answered with a flow control frame while this state
//! machine is itself waiting for the flow control frame of the message it is
//! sending. `SF` / `FF` / `CF` belong to the receive direction, `FC` to the
//! send direction; a frame that no live direction can accept fails with
//! [`IsotpError::UnexpectedFrame`].
//!
//! # Failure handling
//!
//! A protocol failure yields [`Action::Failed`] and resets both directions; the
//! caller decides whether to retry the message or the connection. Truncated and
//! hostile frames are reported through [`IsotpError`], never by panicking.

use std::time::Duration;

/// Largest ISO-TP message payload: the first frame length field is 12 bits.
///
/// Messages longer than this are rejected with [`IsotpError::MessageTooLong`];
/// the 32-bit first frame escape form is not supported.
pub const MAX_MESSAGE_LEN: usize = 4095;

/// Data bytes in one classical CAN frame.
pub const CLASSICAL_FRAME_BYTES: usize = 8;

/// Data bytes in one CAN-FD frame with `tx_dl = 64`.
pub const CAN_FD_FRAME_BYTES: usize = 64;

/// N_Bs: how long the sender waits for a flow control frame.
///
/// 25 ms is this project's existing patience for one CAN hop: it is Klipper's
/// `MIN_RTO` (`klippy/chelper/serialqueue.c:107`), the floor the retransmit
/// timeout is clamped to — and that estimator already covers CAN links (the
/// same file switches on `SQT_CAN`). This host mirrors it as
/// `MIN_RTO`/`mcu/mod.rs`. The ISO value is 1000 ms, which is sized for a
/// diagnostic link shared with strangers; both ends here are ours, and a
/// worst-case 64 byte message is ~11 frames (~1.2 ms at 1 Mbit/s), so 25 ms
/// leaves a ~20x margin while failing fast.
///
/// ISO 15765-2 also asks the receiver's `N_Cs` (time to emit the flow control
/// frame) to stay under `0.9 * N_Bs`, so the firmware must answer within
/// ~22 ms.
pub const N_BS_TIMEOUT: Duration = Duration::from_millis(25);

/// N_Cr: how long the receiver waits for the next consecutive frame.
///
/// Same 25 ms as [`N_BS_TIMEOUT`] and for the same reason; the peer's `N_Br`
/// must stay under `0.9 * N_Cr`.
pub const N_CR_TIMEOUT: Duration = Duration::from_millis(25);

/// Default number of consecutive `FC.WAIT` frames accepted before the transfer
/// is abandoned (the peer's `wftmax`).
pub const DEFAULT_MAX_WAIT_FRAMES: u8 = 4;

/// PCI type, high nibble of the first frame byte.
const PCI_SF: u8 = 0x0;
/// PCI type, high nibble of the first frame byte.
const PCI_FF: u8 = 0x1;
/// PCI type, high nibble of the first frame byte.
const PCI_CF: u8 = 0x2;
/// PCI type, high nibble of the first frame byte.
const PCI_FC: u8 = 0x3;

/// Flow status, low nibble of an `FC` frame's first byte.
const FC_CTS: u8 = 0x0;
/// Flow status, low nibble of an `FC` frame's first byte.
const FC_WAIT: u8 = 0x1;
/// Flow status, low nibble of an `FC` frame's first byte.
const FC_OVFLW: u8 = 0x2;

/// Where the PCI type sits in the first frame byte.
const PCI_SHIFT: u8 = 4;
/// Mask for the low nibble of the first frame byte.
const PCI_MASK: u8 = 0x0f;
/// Sequence numbers wrap modulo this.
const SN_MODULO: u8 = 16;
/// `STmin` byte value whose meaning is "127 ms".
const STMIN_MAX_MILLIS: u64 = 0x7f;

/// Frame capacity of one CAN link, plus the local flow control limits.
///
/// The defaults are the project decision D3 (see the CAN ISO-TP notes):
/// classical CAN, `BS = 0`, `STmin = 0`, no transmit padding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IsotpConfig {
    /// Data bytes available in one CAN frame (`tx_dl`): [`CLASSICAL_FRAME_BYTES`]
    /// or [`CAN_FD_FRAME_BYTES`]. Must be at least 8 and at most 256.
    pub frame_bytes: usize,
    /// Consecutive `FC.WAIT` frames accepted before
    /// [`IsotpError::WaitOverflow`].
    pub max_wait_frames: u8,
}

impl IsotpConfig {
    /// Classical CAN: 8 data bytes, so `SF <= 7`, `FF <= 6`, `CF <= 7`.
    pub const fn classical() -> Self {
        Self {
            frame_bytes: CLASSICAL_FRAME_BYTES,
            max_wait_frames: DEFAULT_MAX_WAIT_FRAMES,
        }
    }

    /// CAN-FD with `tx_dl = 64`: `SF <= 62` (escape form), `FF <= 62`,
    /// `CF <= 63`.
    pub const fn can_fd() -> Self {
        Self {
            frame_bytes: CAN_FD_FRAME_BYTES,
            max_wait_frames: DEFAULT_MAX_WAIT_FRAMES,
        }
    }

    /// Largest payload a single frame can carry, escape form included.
    pub const fn sf_max(&self) -> usize {
        let normal = self.frame_bytes.saturating_sub(1);
        let normal = if normal > 7 { 7 } else { normal };
        let escape = self.frame_bytes.saturating_sub(2);
        if escape > normal {
            escape
        } else {
            normal
        }
    }

    /// Largest payload the first frame of a segmented message carries.
    pub const fn ff_max(&self) -> usize {
        self.frame_bytes.saturating_sub(2)
    }

    /// Largest payload one consecutive frame carries.
    pub const fn cf_max(&self) -> usize {
        self.frame_bytes.saturating_sub(1)
    }
}

impl Default for IsotpConfig {
    fn default() -> Self {
        Self::classical()
    }
}

/// One input to [`IsotpState::step`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Input<'a> {
    /// Nothing new arrived; only service the timers and the `STmin` gate.
    Poll,
    /// Segment and send one complete message (a whole Klipper frame).
    Send(&'a [u8]),
    /// A received CAN frame, given as its data bytes.
    Frame(&'a [u8]),
}

/// One thing the caller has to do as a result of [`IsotpState::step`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Action {
    /// Put this frame's data bytes on the wire (PCI already included, no
    /// padding). All transmitted frames use this node's transmit CAN id.
    Transmit(Vec<u8>),
    /// One message has been fully reassembled; the payload is a whole Klipper
    /// frame.
    Received(Vec<u8>),
    /// Nothing to transmit right now. Calling `step(Poll)` again earlier than
    /// `next_poll` cannot change anything, so sleep until then.
    Wait { next_poll: Duration },
    /// The protocol failed and both directions were reset.
    Failed(IsotpError),
}

/// Why a transfer failed. Every variant is terminal for the current message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IsotpError {
    /// The message payload does not fit the 12-bit first frame length field.
    MessageTooLong { len: usize },
    /// The message payload is empty; ISO-TP has no zero-length single frame.
    EmptyMessage,
    /// The first frame byte is not a legal PCI (unknown type, `SF` length 0 or
    /// 8…15, `SF` escape form on a classical link, `FF` for a payload that a
    /// single frame could carry, reserved flow status).
    InvalidPci { byte0: u8 },
    /// A frame is shorter than the PCI type it announces requires, or a
    /// payload byte is missing.
    TruncatedFrame { len: usize },
    /// A consecutive frame carried the wrong sequence number.
    SnMismatch { expected: u8, got: u8 },
    /// Waiting for a flow control frame exceeded N_Bs.
    FlowControlTimeout,
    /// Waiting for the next consecutive frame exceeded N_Cr.
    ConsecutiveFrameTimeout,
    /// The peer reported that its receive buffer overflowed (`FC.OVFLW`).
    FlowControlOverflow,
    /// The peer sent more consecutive `FC.WAIT` frames than the limit allows.
    WaitOverflow { limit: u8 },
    /// A frame arrived that no live direction can accept.
    UnexpectedFrame,
    /// `Input::Send` was called while the previous message was still being
    /// segmented.
    TxBusy,
}

/// Sender half: one message being segmented.
#[derive(Debug, Clone)]
struct TxState {
    /// The whole message, already accepted for sending.
    message: Vec<u8>,
    /// Bytes of `message` already placed in consecutive frames.
    sent: usize,
    /// Sequence number the next consecutive frame carries.
    next_sn: u8,
    /// Consecutive frames left in the current block; `None` means `BS == 0`.
    block_left: Option<u32>,
    /// Earliest time the next consecutive frame may go out (`STmin` gate).
    next_cf_at: Duration,
    /// `STmin` of the block in progress.
    stmin: Duration,
    /// Absolute deadline of the wait in progress (N_Bs).
    deadline: Duration,
    /// `FC.WAIT` frames seen since the current block started.
    waits: u8,
    /// True while consecutive frames may be streamed, false while waiting for a
    /// flow control frame.
    streaming: bool,
}

/// Receiver half: one message being reassembled.
#[derive(Debug, Clone)]
struct RxState {
    /// Payload bytes reassembled so far.
    buffer: Vec<u8>,
    /// Total payload length announced by the first frame.
    expected: usize,
    /// Sequence number the next consecutive frame must carry.
    next_sn: u8,
    /// Absolute deadline of the wait in progress (N_Cr).
    deadline: Duration,
}

/// The ISO-TP state machine: one sender half and one receiver half, driven by
/// the caller's timebase.
#[derive(Debug, Clone)]
pub struct IsotpState {
    config: IsotpConfig,
    tx: Option<TxState>,
    rx: Option<RxState>,
}

impl IsotpState {
    /// Create an idle state machine for the given link capacity.
    pub fn new(config: IsotpConfig) -> Self {
        debug_assert!(
            config.frame_bytes >= CLASSICAL_FRAME_BYTES && config.frame_bytes <= 256,
            "frame_bytes must fit the single frame length byte",
        );
        Self {
            config,
            tx: None,
            rx: None,
        }
    }

    /// Drop every in-flight transfer, leaving both directions idle.
    pub fn reset(&mut self) {
        self.tx = None;
        self.rx = None;
    }

    /// Advance the state machine by one caller-chosen step.
    ///
    /// `now` is the caller's monotonic timebase and only ever compared against
    /// other `now` values; the state machine never reads a clock itself. The
    /// returned actions are meant to be carried out in order: transmit the
    /// frames, deliver the messages, then sleep until `Wait.next_poll`.
    pub fn step(&mut self, now: Duration, input: Input<'_>) -> Vec<Action> {
        let mut actions = Vec::new();
        let outcome = match input {
            Input::Poll => Ok(()),
            Input::Send(message) => self.begin_send(now, message, &mut actions),
            Input::Frame(data) => self.handle_frame(now, data, &mut actions),
        };
        if let Err(error) = outcome {
            return self.fail(error, actions);
        }
        if let Err(error) = self.check_timeouts(now) {
            return self.fail(error, actions);
        }
        self.drive_tx(now, &mut actions);
        if let Some(next_poll) = self.next_poll() {
            actions.push(Action::Wait { next_poll });
        }
        actions
    }

    /// Report a failure and reset both directions.
    fn fail(&mut self, error: IsotpError, mut actions: Vec<Action>) -> Vec<Action> {
        self.reset();
        actions.push(Action::Failed(error));
        actions
    }

    /// Start segmenting one message: a single frame if it fits, otherwise a
    /// first frame followed by consecutive frames.
    fn begin_send(
        &mut self,
        now: Duration,
        message: &[u8],
        actions: &mut Vec<Action>,
    ) -> Result<(), IsotpError> {
        if self.tx.is_some() {
            return Err(IsotpError::TxBusy);
        }
        if message.is_empty() {
            return Err(IsotpError::EmptyMessage);
        }
        if message.len() > MAX_MESSAGE_LEN {
            return Err(IsotpError::MessageTooLong { len: message.len() });
        }

        if message.len() <= self.config.sf_max() {
            let mut frame = Vec::with_capacity(2 + message.len());
            if message.len() <= 7 {
                frame.push(message.len() as u8);
            } else {
                // The escape form: byte 0 = 0x00, byte 1 = length.
                frame.push(0x00);
                frame.push(message.len() as u8);
            }
            frame.extend_from_slice(message);
            actions.push(Action::Transmit(frame));
            return Ok(());
        }

        let ff_max = self.config.ff_max();
        let carried = ff_max.min(message.len());
        let mut frame = Vec::with_capacity(2 + carried);
        frame.push((PCI_FF << PCI_SHIFT) | ((message.len() >> 8) as u8 & PCI_MASK));
        frame.push((message.len() & 0xff) as u8);
        frame.extend_from_slice(&message[..carried]);
        actions.push(Action::Transmit(frame));
        self.tx = Some(TxState {
            message: message.to_vec(),
            sent: carried,
            next_sn: 1,
            block_left: None,
            next_cf_at: now,
            stmin: Duration::ZERO,
            deadline: now + N_BS_TIMEOUT,
            waits: 0,
            streaming: false,
        });
        Ok(())
    }

    /// Dispatch one received frame to the direction its PCI type belongs to.
    fn handle_frame(
        &mut self,
        now: Duration,
        data: &[u8],
        actions: &mut Vec<Action>,
    ) -> Result<(), IsotpError> {
        let Some(&byte0) = data.first() else {
            return Err(IsotpError::TruncatedFrame { len: 0 });
        };
        match byte0 >> PCI_SHIFT {
            PCI_SF => self.handle_single_frame(data, actions),
            PCI_FF => self.handle_first_frame(now, data, actions),
            PCI_CF => self.handle_consecutive_frame(now, data, actions),
            PCI_FC => self.handle_flow_control(now, data),
            _ => Err(IsotpError::InvalidPci { byte0 }),
        }
    }

    /// `SF`: the whole message is in this frame. Trailing padding is ignored.
    fn handle_single_frame(
        &mut self,
        data: &[u8],
        actions: &mut Vec<Action>,
    ) -> Result<(), IsotpError> {
        if self.rx.is_some() {
            return Err(IsotpError::UnexpectedFrame);
        }
        let mut index = 1;
        let length = match data[0] & PCI_MASK {
            0 => {
                // Escape form: only a link that can carry more than 7 payload
                // bytes in one frame may use it.
                if self.config.sf_max() <= 7 {
                    return Err(IsotpError::InvalidPci { byte0: data[0] });
                }
                let Some(&length) = data.get(1) else {
                    return Err(IsotpError::TruncatedFrame { len: data.len() });
                };
                index = 2;
                length as usize
            }
            1..=7 => usize::from(data[0] & PCI_MASK),
            // SF_DL 8..15 does not exist; such lengths need the escape form.
            _ => return Err(IsotpError::InvalidPci { byte0: data[0] }),
        };
        if length == 0 {
            return Err(IsotpError::InvalidPci { byte0: data[0] });
        }
        let Some(payload) = data.get(index..index + length) else {
            return Err(IsotpError::TruncatedFrame { len: data.len() });
        };
        actions.push(Action::Received(payload.to_vec()));
        Ok(())
    }

    /// `FF`: remember the announced length, answer with a flow control frame and
    /// wait for consecutive frames.
    fn handle_first_frame(
        &mut self,
        now: Duration,
        data: &[u8],
        actions: &mut Vec<Action>,
    ) -> Result<(), IsotpError> {
        if self.rx.is_some() {
            return Err(IsotpError::UnexpectedFrame);
        }
        let Some(&low) = data.get(1) else {
            return Err(IsotpError::TruncatedFrame { len: data.len() });
        };
        let length = (usize::from(data[0] & PCI_MASK) << 8) | usize::from(low);
        if length == 0 || length <= self.config.sf_max() {
            // A zero length is not encodable, and a payload that a single frame
            // could carry must not use the first frame form.
            return Err(IsotpError::InvalidPci { byte0: data[0] });
        }
        let carried = (data.len() - 2).min(length);
        let mut buffer = Vec::with_capacity(length);
        buffer.extend_from_slice(&data[2..2 + carried]);
        if buffer.len() == length {
            actions.push(Action::Received(buffer));
            return Ok(());
        }
        // The message fits our buffer by construction, so always clear to send.
        actions.push(Action::Transmit(vec![(PCI_FC << PCI_SHIFT) | FC_CTS, 0, 0]));
        self.rx = Some(RxState {
            buffer,
            expected: length,
            next_sn: 1,
            deadline: now + N_CR_TIMEOUT,
        });
        Ok(())
    }

    /// `CF`: append payload bytes, check the sequence number and finish the
    /// message when the announced length is reached.
    fn handle_consecutive_frame(
        &mut self,
        now: Duration,
        data: &[u8],
        actions: &mut Vec<Action>,
    ) -> Result<(), IsotpError> {
        let complete = {
            let Some(rx) = self.rx.as_mut() else {
                return Err(IsotpError::UnexpectedFrame);
            };
            let sn = data[0] & PCI_MASK;
            if sn != rx.next_sn {
                return Err(IsotpError::SnMismatch {
                    expected: rx.next_sn,
                    got: sn,
                });
            }
            let remaining = rx.expected - rx.buffer.len();
            let available = (data.len() - 1).min(remaining);
            if available == 0 {
                return Err(IsotpError::TruncatedFrame { len: data.len() });
            }
            rx.buffer.extend_from_slice(&data[1..1 + available]);
            rx.next_sn = (rx.next_sn + 1) % SN_MODULO;
            rx.deadline = now + N_CR_TIMEOUT;
            rx.buffer.len() == rx.expected
        };
        if complete {
            let rx = self.rx.take().expect("receiver present while completing");
            actions.push(Action::Received(rx.buffer));
        }
        Ok(())
    }

    /// `FC`: only meaningful for the sender, and only while it waits for one.
    fn handle_flow_control(&mut self, now: Duration, data: &[u8]) -> Result<(), IsotpError> {
        if data.len() < 3 {
            return Err(IsotpError::TruncatedFrame { len: data.len() });
        }
        let Some(tx) = self.tx.as_mut() else {
            return Err(IsotpError::UnexpectedFrame);
        };
        if tx.streaming {
            return Err(IsotpError::UnexpectedFrame);
        }
        match data[0] & PCI_MASK {
            FC_CTS => {
                let block_size = data[1];
                tx.block_left = if block_size == 0 {
                    None
                } else {
                    Some(u32::from(block_size))
                };
                tx.stmin = decode_stmin(data[2]);
                tx.next_cf_at = now + tx.stmin;
                tx.waits = 0;
                tx.streaming = true;
                Ok(())
            }
            FC_WAIT => {
                tx.waits += 1;
                if tx.waits > self.config.max_wait_frames {
                    return Err(IsotpError::WaitOverflow {
                        limit: self.config.max_wait_frames,
                    });
                }
                tx.deadline = now + N_BS_TIMEOUT;
                Ok(())
            }
            FC_OVFLW => Err(IsotpError::FlowControlOverflow),
            _ => Err(IsotpError::InvalidPci { byte0: data[0] }),
        }
    }

    /// Report the timeout of whichever wait is currently in progress.
    fn check_timeouts(&self, now: Duration) -> Result<(), IsotpError> {
        if let Some(tx) = &self.tx {
            if !tx.streaming && now >= tx.deadline {
                return Err(IsotpError::FlowControlTimeout);
            }
        }
        if let Some(rx) = &self.rx {
            if now >= rx.deadline {
                return Err(IsotpError::ConsecutiveFrameTimeout);
            }
        }
        Ok(())
    }

    /// Emit as many consecutive frames as `now`, `STmin` and the block size
    /// allow. Each iteration consumes at least one payload byte.
    fn drive_tx(&mut self, now: Duration, actions: &mut Vec<Action>) {
        while let Some(frame) = self.take_next_consecutive_frame(now) {
            actions.push(Action::Transmit(frame));
        }
    }

    /// Build the next consecutive frame, or `None` if none is due yet.
    fn take_next_consecutive_frame(&mut self, now: Duration) -> Option<Vec<u8>> {
        let (frame, finished, block_done) = {
            let tx = self.tx.as_mut()?;
            if !tx.streaming || now < tx.next_cf_at {
                return None;
            }
            if tx.block_left == Some(0) {
                // Loop guard: the block is exhausted, wait for an FC.
                tx.streaming = false;
                tx.deadline = now + N_BS_TIMEOUT;
                return None;
            }
            let remaining = tx.message.len() - tx.sent;
            let take = remaining.min(self.config.cf_max());
            let end = tx.sent + take;
            let mut frame = Vec::with_capacity(1 + take);
            frame.push((PCI_CF << PCI_SHIFT) | tx.next_sn);
            frame.extend_from_slice(&tx.message[tx.sent..end]);
            tx.sent = end;
            tx.next_sn = (tx.next_sn + 1) % SN_MODULO;
            tx.next_cf_at = now + tx.stmin;
            if let Some(left) = tx.block_left.as_mut() {
                *left -= 1;
            }
            let finished = tx.sent == tx.message.len();
            (frame, finished, tx.block_left == Some(0) && !finished)
        };
        if finished {
            self.tx = None;
        } else if block_done {
            let tx = self.tx.as_mut().expect("sender present after a frame");
            tx.streaming = false;
            tx.deadline = now + N_BS_TIMEOUT;
        }
        Some(frame)
    }

    /// The earliest time at which a `Poll` could change anything.
    fn next_poll(&self) -> Option<Duration> {
        let mut earliest: Option<Duration> = None;
        if let Some(tx) = &self.tx {
            let candidate = if tx.streaming {
                tx.next_cf_at
            } else {
                tx.deadline
            };
            earliest = Some(earliest.map_or(candidate, |current| current.min(candidate)));
        }
        if let Some(rx) = &self.rx {
            earliest = Some(earliest.map_or(rx.deadline, |current| current.min(rx.deadline)));
        }
        earliest
    }
}

/// Decode an `STmin` byte into the minimum gap between consecutive frames.
///
/// `0x00..=0x7F` are milliseconds and `0xF1..=0xF9` are 100…900 microseconds.
/// The reserved ranges `0x80..=0xF0` and `0xFA..=0xFF` are treated as `0x7F`,
/// that is 127 ms, as ISO 15765-2 requires of a receiver. This is not a guess:
/// the Linux kernel does exactly the same thing when it reads a peer's flow
/// control frame (`net/can/isotp.c`, `isotp_rcv_fc()`: `if (stmin > 0x7F &&
/// (stmin < 0xF1 || stmin > 0xF9)) stmin = 0x7F;`), so our sender and a
/// kernel peer agree byte for byte on this branch.
fn decode_stmin(raw: u8) -> Duration {
    match raw {
        0x00..=0x7f => Duration::from_millis(u64::from(raw)),
        0xf1..=0xf9 => Duration::from_micros(u64::from(raw - 0xf0) * 100),
        _ => Duration::from_millis(STMIN_MAX_MILLIS),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn transmits(actions: &[Action]) -> Vec<Vec<u8>> {
        actions
            .iter()
            .filter_map(|action| match action {
                Action::Transmit(frame) => Some(frame.clone()),
                _ => None,
            })
            .collect()
    }

    fn received(actions: &[Action]) -> Vec<Vec<u8>> {
        actions
            .iter()
            .filter_map(|action| match action {
                Action::Received(message) => Some(message.clone()),
                _ => None,
            })
            .collect()
    }

    fn error_of(actions: &[Action]) -> Option<IsotpError> {
        actions.iter().find_map(|action| match action {
            Action::Failed(error) => Some(*error),
            _ => None,
        })
    }

    fn first_frame(message: &[u8]) -> Vec<u8> {
        let mut frame = vec![
            0x10 | ((message.len() >> 8) as u8 & 0x0f),
            (message.len() & 0xff) as u8,
        ];
        frame.extend_from_slice(&message[..6]);
        frame
    }

    #[test]
    fn test_frame_capacity_is_parameterized() {
        let classical = IsotpConfig::classical();
        assert_eq!(classical.sf_max(), 7);
        assert_eq!(classical.ff_max(), 6);
        assert_eq!(classical.cf_max(), 7);

        let can_fd = IsotpConfig::can_fd();
        assert_eq!(can_fd.sf_max(), 62);
        assert_eq!(can_fd.ff_max(), 62);
        assert_eq!(can_fd.cf_max(), 63);
    }

    #[test]
    fn test_classical_single_frame_round_trip() {
        let message = vec![1, 2, 3, 4, 5];
        let mut tx = IsotpState::new(IsotpConfig::classical());
        let actions = tx.step(Duration::ZERO, Input::Send(&message));
        assert_eq!(transmits(&actions), vec![vec![0x05, 1, 2, 3, 4, 5]]);
        assert!(error_of(&actions).is_none());

        let mut rx = IsotpState::new(IsotpConfig::classical());
        let actions = rx.step(Duration::ZERO, Input::Frame(&[0x05, 1, 2, 3, 4, 5]));
        assert_eq!(received(&actions), vec![message]);
    }

    #[test]
    fn test_classical_segmented_round_trip() {
        let message: Vec<u8> = (0..16).collect();
        let mut tx = IsotpState::new(IsotpConfig::classical());

        let actions = tx.step(Duration::ZERO, Input::Send(&message));
        assert_eq!(transmits(&actions), vec![first_frame(&message)]);

        // The peer clears us for the whole remainder.
        let actions = tx.step(Duration::from_millis(1), Input::Frame(&[0x30, 0x00, 0x00]));
        let cf = transmits(&actions);
        assert_eq!(cf.len(), 2);
        assert_eq!(cf[0][0], 0x21);
        assert_eq!(&cf[0][1..], &message[6..13]);
        assert_eq!(cf[1][0], 0x22);
        assert_eq!(&cf[1][1..], &message[13..16]);
        // Once the last frame is out the send direction is idle again.
        assert!(tx.step(Duration::from_millis(2), Input::Poll).is_empty());

        let mut rx = IsotpState::new(IsotpConfig::classical());
        let actions = rx.step(Duration::ZERO, Input::Frame(&first_frame(&message)));
        assert_eq!(transmits(&actions), vec![vec![0x30, 0x00, 0x00]]);
        assert!(received(&actions).is_empty());
        assert!(received(&rx.step(Duration::from_millis(1), Input::Frame(&cf[0]))).is_empty());
        let actions = rx.step(Duration::from_millis(2), Input::Frame(&cf[1]));
        assert_eq!(received(&actions), vec![message]);
    }

    #[test]
    fn test_sequence_numbers_wrap_past_fifteen() {
        let message: Vec<u8> = (0..150u32).map(|value| (value % 251) as u8).collect();
        let mut tx = IsotpState::new(IsotpConfig::classical());

        let actions = tx.step(Duration::ZERO, Input::Send(&message));
        let ff = transmits(&actions);
        assert_eq!(ff.len(), 1);
        assert_eq!(ff[0][0], 0x10);
        assert_eq!(ff[0][1], 150);
        assert_eq!(&ff[0][2..], &message[..6]);

        let actions = tx.step(Duration::from_millis(1), Input::Frame(&[0x30, 0x00, 0x00]));
        let cf = transmits(&actions);
        assert_eq!(cf.len(), 21);
        let sns: Vec<u8> = cf.iter().map(|frame| frame[0] & 0x0f).collect();
        let expected: Vec<u8> = (1..=21u8).map(|number| number % SN_MODULO).collect();
        assert_eq!(sns, expected);

        let mut rx = IsotpState::new(IsotpConfig::classical());
        assert_eq!(
            transmits(&rx.step(Duration::ZERO, Input::Frame(&ff[0]))).len(),
            1
        );
        let mut out = Vec::new();
        for (index, frame) in cf.iter().enumerate() {
            let actions = rx.step(Duration::from_millis(2 + index as u64), Input::Frame(frame));
            out.extend(received(&actions));
        }
        assert_eq!(out, vec![message]);
    }

    #[test]
    fn test_can_fd_single_frame_uses_escape_form() {
        let message: Vec<u8> = (0..62).collect();
        let mut tx = IsotpState::new(IsotpConfig::can_fd());
        let actions = tx.step(Duration::ZERO, Input::Send(&message));
        let frames = transmits(&actions);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].len(), CAN_FD_FRAME_BYTES);
        assert_eq!(frames[0][0], 0x00);
        assert_eq!(frames[0][1], 62);
        assert_eq!(&frames[0][2..], &message[..]);

        let mut rx = IsotpState::new(IsotpConfig::can_fd());
        let actions = rx.step(Duration::ZERO, Input::Frame(&frames[0]));
        assert_eq!(received(&actions), vec![message]);
    }

    #[test]
    fn test_can_fd_lengths_63_and_64_use_segmentation() {
        for length in [63usize, 64] {
            let message: Vec<u8> = (0..length).map(|value| value as u8).collect();
            let mut tx = IsotpState::new(IsotpConfig::can_fd());

            let actions = tx.step(Duration::ZERO, Input::Send(&message));
            let ff = transmits(&actions);
            assert_eq!(ff.len(), 1, "length {length}");
            assert_eq!(ff[0].len(), CAN_FD_FRAME_BYTES);
            assert_eq!(ff[0][0], 0x10);
            assert_eq!(usize::from(ff[0][1]), length);
            assert_eq!(&ff[0][2..], &message[..62]);

            let actions = tx.step(Duration::from_millis(1), Input::Frame(&[0x30, 0x00, 0x00]));
            let cf = transmits(&actions);
            assert_eq!(cf.len(), 1, "length {length}");
            assert_eq!(cf[0][0], 0x21);
            assert_eq!(&cf[0][1..], &message[62..]);
        }
    }

    #[test]
    fn test_block_size_stops_after_the_requested_frames() {
        let message: Vec<u8> = (0..32).collect();
        let mut tx = IsotpState::new(IsotpConfig::classical());
        assert_eq!(
            transmits(&tx.step(Duration::ZERO, Input::Send(&message))).len(),
            1
        );

        // BS = 2: exactly two consecutive frames, then wait for another FC.
        let actions = tx.step(Duration::from_millis(1), Input::Frame(&[0x30, 2, 0x00]));
        assert_eq!(transmits(&actions).len(), 2);
        let next_poll = N_BS_TIMEOUT + Duration::from_millis(1);
        assert_eq!(actions.last(), Some(&Action::Wait { next_poll }));

        // Nothing more goes out: the sender waits for the next FC and that wait
        // times out after N_Bs.
        let actions = tx.step(Duration::from_millis(2), Input::Poll);
        assert!(transmits(&actions).is_empty());
        assert!(error_of(&actions).is_none());
        let actions = tx.step(next_poll, Input::Poll);
        assert_eq!(error_of(&actions), Some(IsotpError::FlowControlTimeout));

        // The next FC finishes the message.
        let mut tx = IsotpState::new(IsotpConfig::classical());
        tx.step(Duration::ZERO, Input::Send(&message));
        tx.step(Duration::from_millis(1), Input::Frame(&[0x30, 2, 0x00]));
        let actions = tx.step(Duration::from_millis(2), Input::Frame(&[0x30, 0, 0x00]));
        assert_eq!(transmits(&actions).len(), 2);
    }

    #[test]
    fn test_flow_control_wait_then_clear_to_send() {
        let message: Vec<u8> = (0..16).collect();
        let mut tx = IsotpState::new(IsotpConfig::classical());
        tx.step(Duration::ZERO, Input::Send(&message));

        // Both steps land at the same instant, halfway to the deadline: a WAIT
        // restarts N_Bs from there, and the CTS that follows in that same
        // instant is accepted. Times are derived from the timeout so this test
        // does not pin the constant's value.
        let mid = N_BS_TIMEOUT / 2;
        let actions = tx.step(mid, Input::Frame(&[0x31, 0x00, 0x00]));
        assert!(error_of(&actions).is_none());
        assert!(transmits(&actions).is_empty());
        assert_eq!(
            actions.last(),
            Some(&Action::Wait {
                next_poll: mid + N_BS_TIMEOUT
            })
        );

        let actions = tx.step(mid, Input::Frame(&[0x30, 0x00, 0x00]));
        assert_eq!(transmits(&actions).len(), 2);
    }

    #[test]
    fn test_too_many_wait_frames_fail() {
        let message: Vec<u8> = (0..16).collect();
        let mut tx = IsotpState::new(IsotpConfig::classical());
        tx.step(Duration::ZERO, Input::Send(&message));
        for index in 0..u64::from(DEFAULT_MAX_WAIT_FRAMES) {
            let actions = tx.step(
                Duration::from_millis(index),
                Input::Frame(&[0x31, 0x00, 0x00]),
            );
            assert!(
                error_of(&actions).is_none(),
                "wait {index} must be accepted"
            );
        }
        let actions = tx.step(
            Duration::from_millis(u64::from(DEFAULT_MAX_WAIT_FRAMES)),
            Input::Frame(&[0x31, 0x00, 0x00]),
        );
        assert_eq!(
            error_of(&actions),
            Some(IsotpError::WaitOverflow {
                limit: DEFAULT_MAX_WAIT_FRAMES
            })
        );
    }

    #[test]
    fn test_flow_control_overflow_fails() {
        let message: Vec<u8> = (0..16).collect();
        let mut tx = IsotpState::new(IsotpConfig::classical());
        tx.step(Duration::ZERO, Input::Send(&message));
        let actions = tx.step(Duration::from_millis(1), Input::Frame(&[0x32, 0x00, 0x00]));
        assert_eq!(error_of(&actions), Some(IsotpError::FlowControlOverflow));
    }

    #[test]
    fn test_waiting_for_flow_control_times_out() {
        let message: Vec<u8> = (0..16).collect();
        let mut tx = IsotpState::new(IsotpConfig::classical());
        tx.step(Duration::ZERO, Input::Send(&message));

        let actions = tx.step(N_BS_TIMEOUT - Duration::from_millis(1), Input::Poll);
        assert!(error_of(&actions).is_none());
        assert_eq!(
            actions.last(),
            Some(&Action::Wait {
                next_poll: N_BS_TIMEOUT
            })
        );

        let actions = tx.step(N_BS_TIMEOUT, Input::Poll);
        assert_eq!(error_of(&actions), Some(IsotpError::FlowControlTimeout));
        // The failure releases the sender: a new message can start right away.
        let actions = tx.step(N_BS_TIMEOUT, Input::Send(&message));
        assert_eq!(transmits(&actions).len(), 1);
    }

    #[test]
    fn test_waiting_for_consecutive_frame_times_out() {
        let message: Vec<u8> = (0..16).collect();
        let mut rx = IsotpState::new(IsotpConfig::classical());
        rx.step(Duration::ZERO, Input::Frame(&first_frame(&message)));

        let actions = rx.step(N_CR_TIMEOUT - Duration::from_millis(1), Input::Poll);
        assert!(error_of(&actions).is_none());

        let actions = rx.step(N_CR_TIMEOUT, Input::Poll);
        assert_eq!(
            error_of(&actions),
            Some(IsotpError::ConsecutiveFrameTimeout)
        );
    }

    #[test]
    fn test_wrong_sequence_number_fails() {
        let message: Vec<u8> = (0..16).collect();
        let mut rx = IsotpState::new(IsotpConfig::classical());
        rx.step(Duration::ZERO, Input::Frame(&first_frame(&message)));
        let actions = rx.step(
            Duration::from_millis(1),
            Input::Frame(&[0x22, 7, 8, 9, 10, 11, 12]),
        );
        assert_eq!(
            error_of(&actions),
            Some(IsotpError::SnMismatch {
                expected: 1,
                got: 2
            })
        );
    }

    #[test]
    fn test_single_frame_length_zero_fails() {
        let mut classical = IsotpState::new(IsotpConfig::classical());
        let actions = classical.step(Duration::ZERO, Input::Frame(&[0x00, 0x01, 0x02]));
        assert_eq!(
            error_of(&actions),
            Some(IsotpError::InvalidPci { byte0: 0x00 })
        );

        let mut can_fd = IsotpState::new(IsotpConfig::can_fd());
        let actions = can_fd.step(Duration::ZERO, Input::Frame(&[0x00, 0x00, 0x01, 0x02]));
        assert_eq!(
            error_of(&actions),
            Some(IsotpError::InvalidPci { byte0: 0x00 })
        );
    }

    #[test]
    fn test_stmin_gate_spaces_consecutive_frames() {
        let message: Vec<u8> = (0..16).collect();
        for (stmin, expected) in [
            (0x0au8, Duration::from_millis(10)),
            (0x00, Duration::ZERO),
            (0xf1, Duration::from_micros(100)),
            // Reserved values are read as 127 ms.
            (0x80, Duration::from_millis(127)),
        ] {
            let mut tx = IsotpState::new(IsotpConfig::classical());
            tx.step(Duration::ZERO, Input::Send(&message));
            let actions = tx.step(Duration::from_millis(1), Input::Frame(&[0x30, 0, stmin]));
            if expected.is_zero() {
                assert_eq!(transmits(&actions).len(), 2, "STmin {stmin:#04x}");
                continue;
            }
            assert!(transmits(&actions).is_empty(), "STmin {stmin:#04x}");
            let next_poll = Duration::from_millis(1) + expected;
            assert_eq!(
                actions.last(),
                Some(&Action::Wait { next_poll }),
                "STmin {stmin:#04x}"
            );
            let actions = tx.step(next_poll, Input::Poll);
            let cf = transmits(&actions);
            assert_eq!(cf.len(), 1, "STmin {stmin:#04x}");
            assert_eq!(cf[0][0], 0x21);
            // The second frame waits for the next gate.
            assert_eq!(
                actions.last(),
                Some(&Action::Wait {
                    next_poll: next_poll + expected
                })
            );
        }
    }

    #[test]
    fn test_trailing_padding_is_ignored() {
        // A single frame padded up to a full classical CAN frame.
        let mut rx = IsotpState::new(IsotpConfig::classical());
        let actions = rx.step(
            Duration::ZERO,
            Input::Frame(&[0x05, 1, 2, 3, 4, 5, 0xaa, 0xaa]),
        );
        assert_eq!(received(&actions), vec![vec![1, 2, 3, 4, 5]]);

        // A consecutive frame padded past the announced message length.
        let mut rx = IsotpState::new(IsotpConfig::classical());
        rx.step(
            Duration::ZERO,
            Input::Frame(&[0x10, 0x08, 1, 2, 3, 4, 5, 6]),
        );
        let actions = rx.step(
            Duration::from_millis(1),
            Input::Frame(&[0x21, 7, 8, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa]),
        );
        assert_eq!(received(&actions), vec![vec![1, 2, 3, 4, 5, 6, 7, 8]]);
    }

    #[test]
    fn test_message_longer_than_twelve_bits_is_refused() {
        let message = vec![0u8; MAX_MESSAGE_LEN + 1];
        let mut tx = IsotpState::new(IsotpConfig::classical());
        let actions = tx.step(Duration::ZERO, Input::Send(&message));
        assert_eq!(
            error_of(&actions),
            Some(IsotpError::MessageTooLong { len: 4096 })
        );
        assert!(transmits(&actions).is_empty());
    }

    #[test]
    fn test_empty_message_is_refused() {
        let mut tx = IsotpState::new(IsotpConfig::classical());
        let actions = tx.step(Duration::ZERO, Input::Send(&[]));
        assert_eq!(error_of(&actions), Some(IsotpError::EmptyMessage));
    }

    #[test]
    fn test_send_while_busy_is_refused() {
        let mut tx = IsotpState::new(IsotpConfig::classical());
        tx.step(Duration::ZERO, Input::Send(&[0u8; 16]));
        let actions = tx.step(Duration::from_millis(1), Input::Send(&[0u8; 16]));
        assert_eq!(error_of(&actions), Some(IsotpError::TxBusy));
    }

    #[test]
    fn test_full_duplex_flow_control_while_waiting_for_one() {
        let outgoing: Vec<u8> = (0..16).collect();
        let incoming: Vec<u8> = (100..112).collect();
        let mut state = IsotpState::new(IsotpConfig::classical());

        // Our own segmented message goes out first and now waits for an FC.
        assert_eq!(
            transmits(&state.step(Duration::ZERO, Input::Send(&outgoing))).len(),
            1
        );

        // The peer starts a message of its own: we owe it a flow control frame
        // even though our own transfer is mid-flight.
        let mut ff = vec![0x10, incoming.len() as u8];
        ff.extend_from_slice(&incoming[..6]);
        let actions = state.step(Duration::from_millis(1), Input::Frame(&ff));
        assert_eq!(transmits(&actions), vec![vec![0x30, 0x00, 0x00]]);
        assert!(error_of(&actions).is_none());

        // Our own clear-to-send arrives: our consecutive frames still follow.
        let actions = state.step(Duration::from_millis(2), Input::Frame(&[0x30, 0x00, 0x00]));
        assert_eq!(transmits(&actions).len(), 2);

        // And the peer's message completes on our side.
        let mut out = Vec::new();
        for (index, chunk) in incoming[6..].chunks(7).enumerate() {
            let mut frame = vec![0x20 | (index as u8 + 1) % SN_MODULO];
            frame.extend_from_slice(chunk);
            let actions = state.step(
                Duration::from_millis(3 + index as u64),
                Input::Frame(&frame),
            );
            out.extend(received(&actions));
        }
        assert_eq!(out, vec![incoming]);
    }

    #[test]
    fn test_truncated_and_hostile_frames_do_not_panic() {
        let cases: &[(&[u8], Option<IsotpError>)] = &[
            (&[], Some(IsotpError::TruncatedFrame { len: 0 })),
            (&[0x00], Some(IsotpError::InvalidPci { byte0: 0x00 })),
            (&[0x10], Some(IsotpError::TruncatedFrame { len: 1 })),
            (&[0x21], Some(IsotpError::UnexpectedFrame)),
            (&[0x30], Some(IsotpError::TruncatedFrame { len: 1 })),
            (&[0x30, 0x00], Some(IsotpError::TruncatedFrame { len: 2 })),
            (
                &[0x00, 0x05, 1, 2, 3, 4, 5],
                Some(IsotpError::InvalidPci { byte0: 0x00 }),
            ),
            (&[0x08, 1, 2], Some(IsotpError::InvalidPci { byte0: 0x08 })),
            (&[0x0f, 1, 2], Some(IsotpError::InvalidPci { byte0: 0x0f })),
            (&[0x02, 0x01], Some(IsotpError::TruncatedFrame { len: 2 })),
            (&[0x10, 0x00], Some(IsotpError::InvalidPci { byte0: 0x10 })),
            (
                &[0x10, 0x05, 1, 2, 3, 4, 5],
                Some(IsotpError::InvalidPci { byte0: 0x10 }),
            ),
            (&[0x30, 0x00, 0x00], Some(IsotpError::UnexpectedFrame)),
            (&[0x33, 0x00, 0x00], Some(IsotpError::UnexpectedFrame)),
            (&[0x23, 1, 2, 3], Some(IsotpError::UnexpectedFrame)),
            (
                &[0xf0, 0x00, 0x00],
                Some(IsotpError::InvalidPci { byte0: 0xf0 }),
            ),
        ];
        for (frame, expected) in cases {
            let mut state = IsotpState::new(IsotpConfig::classical());
            let actions = state.step(Duration::ZERO, Input::Frame(frame));
            assert_eq!(
                error_of(&actions).as_ref(),
                expected.as_ref(),
                "frame {frame:?}"
            );
            // The failure leaves the state machine idle and reusable.
            assert!(transmits(&state.step(Duration::from_millis(1), Input::Poll)).is_empty());
        }
    }

    #[test]
    fn test_first_frame_without_payload_is_reassembled_from_consecutive_frames() {
        let message: Vec<u8> = (0..20).collect();
        let mut rx = IsotpState::new(IsotpConfig::classical());
        let actions = rx.step(Duration::ZERO, Input::Frame(&[0x10, 0x14]));
        assert_eq!(transmits(&actions), vec![vec![0x30, 0x00, 0x00]]);

        let mut frame = vec![0x21];
        frame.extend_from_slice(&message[..7]);
        assert!(received(&rx.step(Duration::from_millis(1), Input::Frame(&frame))).is_empty());

        let mut frame = vec![0x22];
        frame.extend_from_slice(&message[7..14]);
        assert!(received(&rx.step(Duration::from_millis(2), Input::Frame(&frame))).is_empty());

        let mut frame = vec![0x23];
        frame.extend_from_slice(&message[14..]);
        let actions = rx.step(Duration::from_millis(3), Input::Frame(&frame));
        assert_eq!(received(&actions), vec![message]);
    }
}
