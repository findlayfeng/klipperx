//! In-process scripted katapult bootloader for tests.
//!
//! The fake speaks the real frame codec ([`super::frame`]) on both sides: the
//! writer half parses host request frames and queues responses, the reader
//! half serves them one chunk per `read`, so a test can deliver a response in
//! two reads just by enabling [`FakeBootloader::split_responses`]. Quirks
//! ([`Quirk`]) script one-shot misbehaviour — NACK, busy, device error,
//! silence, a corrupted CRC, corrupted stored data — keyed on command and
//! occurrence, so a scenario only misbehaves exactly once.
//!
//! Every `read` on an empty queue answers `WouldBlock`, which the session
//! treats as "no bytes in this window" until its deadline — the fake itself
//! never blocks, so a test can only hang if the session's bounds are broken.

use std::collections::{HashMap, VecDeque};
use std::io::{self, Read, Write};
use std::sync::{Arc, Mutex};

use super::frame::{
    build_frame, command, Frame, FrameScanner, ACK_BUSY, ACK_ERROR, ACK_SUCCESS, NACK,
};

/// One scripted misbehaviour, applied to command `cmd` on its `at`-th
/// occurrence (1-based) exactly once.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Quirk {
    /// Answer 0xF1: the session must resend the frame.
    Nack,
    /// Answer 0xF3: the session must wait `busy_delay` and retry.
    Busy,
    /// Answer 0xF2: the session must abort with `FlashError::Device`.
    Error,
    /// Never answer this command again: the session must exhaust its tries.
    Silent,
    /// Queue a response whose CRC is wrong, followed by the good one: the
    /// scanner must slide past the corrupt frame and accept the next.
    CorruptResponse,
    /// Accept the block but store it flipped: verify must fail.
    CorruptStored,
}

struct Scripted {
    cmd: u8,
    at: usize,
    kind: Quirk,
    used: bool,
}

#[derive(Default)]
struct State {
    /// Host → device request frames, cut by the real scanner.
    scanner: FrameScanner,
    /// Device → host response chunks, served front-first.
    out: VecDeque<Vec<u8>>,
    /// Blocks written by `SEND_BLOCK`, keyed by address.
    memory: HashMap<u32, Vec<u8>>,
    /// How many requests were received per command.
    seen: HashMap<u8, usize>,
    quirks: Vec<Scripted>,
    /// Commands that never get an answer ([`Quirk::Silent`]).
    muted: Vec<u8>,
    /// Deliver every response as two chunks (two `read`s).
    split: bool,
    nonempty_reads: usize,
    responses_served: usize,
    start_addr: u32,
    block_size: u32,
}

impl State {
    fn new() -> Self {
        Self {
            start_addr: 0x0800_2000,
            block_size: 64,
            ..Self::default()
        }
    }
}

/// The fake bootloader: clone cheap handles off it for the session's two
/// halves, then inspect it through the original handle after the run.
pub struct FakeBootloader {
    inner: Arc<Mutex<State>>,
}

pub struct FakeReader(Arc<Mutex<State>>);
pub struct FakeWriter(Arc<Mutex<State>>);

impl FakeBootloader {
    pub fn new() -> Self {
        Self {
            inner: Arc::new(Mutex::new(State::new())),
        }
    }

    pub fn reader(&self) -> FakeReader {
        FakeReader(Arc::clone(&self.inner))
    }

    pub fn writer(&self) -> FakeWriter {
        FakeWriter(Arc::clone(&self.inner))
    }

    /// Script `kind` to fire on the `at`-th (1-based) request for `cmd`.
    pub fn script(&self, cmd: u8, at: usize, kind: Quirk) {
        self.inner
            .lock()
            .expect("fake device lock")
            .quirks
            .push(Scripted {
                cmd,
                at,
                kind,
                used: false,
            });
    }

    /// Serve every response in two chunks, i.e. across two `read`s.
    pub fn split_responses(&self, split: bool) {
        self.inner.lock().expect("fake device lock").split = split;
    }

    pub fn memory(&self) -> HashMap<u32, Vec<u8>> {
        self.inner.lock().expect("fake device lock").memory.clone()
    }

    pub fn seen(&self, cmd: u8) -> usize {
        self.inner
            .lock()
            .expect("fake device lock")
            .seen
            .get(&cmd)
            .copied()
            .unwrap_or(0)
    }

    pub fn nonempty_reads(&self) -> usize {
        self.inner.lock().expect("fake device lock").nonempty_reads
    }

    pub fn responses_served(&self) -> usize {
        self.inner
            .lock()
            .expect("fake device lock")
            .responses_served
    }
}

impl Read for FakeReader {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut state = self.0.lock().expect("fake device lock");
        let Some(front) = state.out.front() else {
            // No bytes in this window; the session polls until its deadline.
            return Err(io::Error::new(io::ErrorKind::WouldBlock, "fake idle"));
        };
        let take = front.len().min(buf.len());
        buf[..take].copy_from_slice(&front[..take]);
        if take == state.out[0].len() {
            state.out.pop_front();
        } else {
            state.out[0].drain(..take);
        }
        state.nonempty_reads += 1;
        Ok(take)
    }
}

impl Write for FakeWriter {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut state = self.0.lock().expect("fake device lock");
        state.scanner.feed(buf);
        while let Some(frame) = state.scanner.next_frame() {
            handle(&mut state, frame);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// Queue a response frame: one chunk, or two when split mode is on.
fn push_response(state: &mut State, bytes: Vec<u8>) {
    state.responses_served += 1;
    if state.split && bytes.len() > 1 {
        let half = bytes.len() / 2;
        let (head, tail) = bytes.split_at(half);
        state.out.push_back(head.to_vec());
        state.out.push_back(tail.to_vec());
    } else {
        state.out.push_back(bytes);
    }
}

fn handle(state: &mut State, frame: Frame) {
    let cmd = frame.code;
    let seen = state.seen.entry(cmd).or_insert(0);
    *seen += 1;
    let at = *seen;
    if state.muted.contains(&cmd) {
        return;
    }
    let kind = state
        .quirks
        .iter_mut()
        .find(|q| !q.used && q.cmd == cmd && q.at == at)
        .map(|q| {
            q.used = true;
            q.kind
        });
    // NACK, busy and command error carry **no** payload word: katapult answers
    // them with `command_respond(out, RESPONSE_*, ARRAY_SIZE(out))` on a
    // two-word `out`, i.e. `(2 - 2) << 24` payload words
    // (`third_party/katapult/src/command.c`).
    match kind {
        Some(Quirk::Nack) => push_response(state, build_frame(NACK, &[])),
        Some(Quirk::Busy) => push_response(state, build_frame(ACK_BUSY, &[])),
        Some(Quirk::Error) => push_response(state, build_frame(ACK_ERROR, &[])),
        Some(Quirk::Silent) => state.muted.push(cmd),
        Some(Quirk::CorruptResponse) => {
            let good = respond(state, &frame, false);
            let mut bad = good.clone();
            // Flip a payload byte: the CRC no longer matches, trailer intact.
            bad[6] ^= 0xFF;
            push_response(state, bad);
            push_response(state, good);
        }
        Some(Quirk::CorruptStored) => {
            let resp = respond(state, &frame, true);
            push_response(state, resp);
        }
        None => {
            let resp = respond(state, &frame, false);
            push_response(state, resp);
        }
    }
}

/// Answer one request, mutating device state (`SEND_BLOCK` stores, `EOF`
/// reports the stored block count, `REQUEST_BLOCK` reads back).
fn respond(state: &mut State, frame: &Frame, corrupt_stored: bool) -> Vec<u8> {
    let orig = u32::from(frame.code).to_le_bytes();
    let payload = &frame.payload;
    match frame.code {
        command::CONNECT => {
            let mut data = orig.to_vec();
            data.extend_from_slice(&0x0001_0002u32.to_le_bytes()); // protocol 1.0.2
            data.extend_from_slice(&state.start_addr.to_le_bytes());
            data.extend_from_slice(&state.block_size.to_le_bytes());
            data.extend_from_slice(b"STM32F103\0\0\0"); // NUL-padded to 3 words
            build_frame(ACK_SUCCESS, &data)
        }
        command::SEND_BLOCK if payload.len() >= 4 => {
            let addr = u32::from_le_bytes(payload[..4].try_into().expect("four bytes"));
            let mut data = payload[4..].to_vec();
            if corrupt_stored && !data.is_empty() {
                data[0] ^= 0xFF;
            }
            state.memory.insert(addr, data);
            let mut data = orig.to_vec();
            data.extend_from_slice(&addr.to_le_bytes());
            build_frame(ACK_SUCCESS, &data)
        }
        command::EOF => {
            let mut data = orig.to_vec();
            data.extend_from_slice(&(state.memory.len() as u32).to_le_bytes());
            build_frame(ACK_SUCCESS, &data)
        }
        command::REQUEST_BLOCK if payload.len() >= 4 => {
            let addr = u32::from_le_bytes(payload[..4].try_into().expect("four bytes"));
            let stored = state
                .memory
                .get(&addr)
                .cloned()
                .unwrap_or_else(|| vec![0xFF; state.block_size as usize]);
            let mut data = orig.to_vec();
            data.extend_from_slice(&addr.to_le_bytes());
            data.extend_from_slice(&stored);
            build_frame(ACK_SUCCESS, &data)
        }
        command::COMPLETE => build_frame(ACK_SUCCESS, &orig),
        command::GET_CANBUS_ID => {
            let mut data = orig.to_vec();
            data.extend_from_slice(&[0x11, 0xAA, 0x22, 0xBB, 0x33, 0xCC, 0x00, 0x00]);
            build_frame(ACK_SUCCESS, &data)
        }
        // Unknown or malformed request: a command error, as the firmware does
        // (zero payload words, like the ack of no command at all).
        _ => build_frame(ACK_ERROR, &[]),
    }
}
