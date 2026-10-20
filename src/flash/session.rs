//! Transport-agnostic katapult flash session: connect → send_block × N →
//! eof → verify (request_block × N) → complete, plus the single-command
//! primitives underneath.
//!
//! The session owns no transport: it drives any `Read + Write` byte stream and
//! polls with deadlines the way `serial_query_unassigned` does — a zero read
//! or `WouldBlock`/`TimedOut` means "no bytes in this window", anything else
//! is an error. Timing follows upstream `scripts/flashtool.py`: five tries per
//! command, a two-second read window per try, 0.5 s between tries and 1.5 s
//! after a busy response. Every knob lives in [`SessionConfig`] so tests can
//! run on millisecond budgets, and every path is bounded: tries are counted,
//! each read waits at most `read_timeout`, each frame write must be taken
//! inside that same window, and all failures are `Result`s.

use std::io::{ErrorKind, Read, Write};
use std::thread;
use std::time::{Duration, Instant};

use super::frame::{self, build_frame, ConnectInfo, Frame, FrameScanner};

/// How long the session waits (and sleeps) between a read that returned
/// nothing and the next poll, so an empty window never spins.
const READ_POLL: Duration = Duration::from_millis(1);
/// How often `send_block`/`request_block` re-issue the command when the
/// device echoes the wrong address before giving up — `flashtool.py` retries
/// a block address mismatch three times.
const ADDRESS_TRIES: u32 = 3;

/// Timing knobs for one session. Defaults mirror `flashtool.py`
/// (`send_command`'s `tries = 5, read_timeout = 2.0`, the 0.5 s inter-try
/// sleep and the 1.5 s busy sleep).
#[derive(Debug, Clone)]
pub struct SessionConfig {
    /// Tries per command (a try ends on timeout, NACK, busy or a wrong
    /// acknowledged command).
    pub tries: u32,
    /// Window per try, both directions: a read waits at most this long, and a
    /// frame the transport will not take whole inside it fails the write (see
    /// [`write_frame`]).
    pub read_timeout: Duration,
    /// Sleep between two tries.
    pub retry_delay: Duration,
    /// Sleep after a busy (0xF3) response, before the next try.
    pub busy_delay: Duration,
}

impl Default for SessionConfig {
    fn default() -> Self {
        Self {
            tries: 5,
            read_timeout: Duration::from_secs(2),
            retry_delay: Duration::from_millis(500),
            busy_delay: Duration::from_millis(1500),
        }
    }
}

/// Everything that can stop a flash session.
#[derive(Debug)]
pub enum FlashError {
    /// The transport itself failed.
    Io(std::io::Error),
    /// The bootloader answered 0xF2 (command error); the session aborts.
    Device { cmd: u8 },
    /// All tries for one command ran out; `last` says why.
    RetriesExhausted {
        cmd: u8,
        tries: u32,
        last: &'static str,
    },
    /// `SEND_BLOCK`/`REQUEST_BLOCK` echoed a different address on every try.
    AddressMismatch { cmd: u8, expected: u32, got: u32 },
    /// Verify read back different bytes than were written.
    VerifyMismatch { addr: u32 },
    /// The wire or the device said something unusable.
    Protocol(String),
}

impl std::fmt::Display for FlashError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FlashError::Io(e) => write!(f, "transport I/O error: {e}"),
            FlashError::Device { cmd } => {
                write!(f, "device reported command error for 0x{cmd:02X}")
            }
            FlashError::RetriesExhausted { cmd, tries, last } => {
                write!(f, "command 0x{cmd:02X} failed after {tries} tries: {last}")
            }
            FlashError::AddressMismatch { cmd, expected, got } => write!(
                f,
                "command 0x{cmd:02X}: expected address 0x{expected:08X}, device echoed 0x{got:08X}"
            ),
            FlashError::VerifyMismatch { addr } => {
                write!(f, "verify mismatch at address 0x{addr:08X}")
            }
            FlashError::Protocol(msg) => write!(f, "protocol error: {msg}"),
        }
    }
}

impl std::error::Error for FlashError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            FlashError::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for FlashError {
    fn from(e: std::io::Error) -> Self {
        FlashError::Io(e)
    }
}

/// What a completed flash produced, for the caller to report.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Summary {
    pub start_addr: u32,
    pub block_size: u32,
    pub block_count: usize,
    /// Page count reported by the `EOF` ack.
    pub page_count: u32,
    /// Payload bytes handed to [`flash`].
    pub bytes_flashed: usize,
    pub mcu_type: String,
    pub proto_version: (u8, u8, u8),
    pub software_version: Option<String>,
}

/// How the session classifies one decoded response frame.
enum Outcome {
    Success(Vec<u8>),
    Abort(FlashError),
    /// Busy (0xF3): wait `busy_delay`, then spend a try.
    Busy,
    Retry(&'static str),
}

/// Classify one response frame, dispatching on its code first.
///
/// The bootloader answers NACK (0xF1), command error (0xF2) and busy (0xF3)
/// with a **zero-word** frame: `command_respond(out, RESPONSE_*, ARRAY_SIZE(out))`
/// with `out[2]`, i.e. `(2 - 2) << 24` payload words (`third_party/katapult/src/command.c`).
/// Only `ACK_SUCCESS` carries words, the first of them the command it
/// acknowledges — so requiring a payload word before looking at the code
/// would report every one of those three answers as a short response and make
/// `Device`/`Busy` unreachable.
fn classify(cmd: u8, frame: Frame) -> Outcome {
    match frame.code {
        frame::ACK_ERROR => Outcome::Abort(FlashError::Device { cmd }),
        frame::ACK_BUSY => Outcome::Busy,
        frame::NACK => Outcome::Retry("nack"),
        frame::ACK_SUCCESS => match frame.as_response() {
            None => Outcome::Retry("acknowledged without a command word"),
            Some(resp) if resp.orig_cmd == u32::from(cmd) => Outcome::Success(resp.data),
            Some(_) => Outcome::Retry("acknowledged a different command"),
        },
        _ => Outcome::Retry("unexpected response code"),
    }
}

/// Send one frame, bounded by `timeout`.
///
/// A transport that takes the bytes in slices, or reports `WouldBlock`/
/// `TimedOut` for a queue with no room yet (the tunnel writer in
/// [`super::bridge`], a non-blocking socket), is retried in [`READ_POLL`]
/// slices; when the window closes the write fails with [`ErrorKind::TimedOut`]
/// wrapped in [`FlashError::Io`] instead of letting the session sit in a write
/// forever. Anything else the transport reports is passed through as its own
/// I/O error. One blocking `write` on a tty is indivisible, which is why the
/// deadline for that transport is enforced here, around the whole frame, and
/// not inside the port.
pub(super) fn write_frame<W: Write>(
    writer: &mut W,
    bytes: &[u8],
    timeout: Duration,
) -> Result<(), FlashError> {
    let deadline = Instant::now() + timeout;
    let mut sent = 0;
    while sent < bytes.len() {
        let stalled = match writer.write(&bytes[sent..]) {
            Ok(0) => true,
            Ok(n) => {
                sent += n;
                false
            }
            Err(e)
                if e.kind() == ErrorKind::Interrupted
                    || e.kind() == ErrorKind::WouldBlock
                    || e.kind() == ErrorKind::TimedOut =>
            {
                true
            }
            Err(e) => return Err(FlashError::Io(e)),
        };
        if stalled {
            if Instant::now() >= deadline {
                return Err(FlashError::Io(std::io::Error::new(
                    ErrorKind::TimedOut,
                    format!(
                        "transport took {sent} of {} bytes within {timeout:?}",
                        bytes.len()
                    ),
                )));
            }
            thread::sleep(READ_POLL);
        }
    }
    writer.flush().map_err(FlashError::Io)
}

/// Send one command and wait for its ack: write the frame, scan replies until
/// the per-try read window closes, and count each timeout/NACK/busy/wrong-ack
/// as one spent try. A 0xF2 answer aborts immediately. Returns the ack
/// payload (bytes after the echoed command word).
pub fn command<R: Read, W: Write>(
    reader: &mut R,
    writer: &mut W,
    cmd: u8,
    payload: &[u8],
    cfg: &SessionConfig,
) -> Result<Vec<u8>, FlashError> {
    let frame = build_frame(cmd, payload);
    let mut scanner = FrameScanner::new();
    let mut buf = [0u8; 1024];
    let mut last = "no response";
    for attempt in 1..=cfg.tries.max(1) {
        if attempt > 1 {
            thread::sleep(cfg.retry_delay);
            // `flashtool.py` clears its read buffer before every retry.
            scanner.clear();
        }
        write_frame(writer, &frame, cfg.read_timeout)?;
        let deadline = Instant::now() + cfg.read_timeout;
        let mut reason: Option<&'static str> = None;
        loop {
            if let Some(frame) = scanner.next_frame() {
                match classify(cmd, frame) {
                    Outcome::Success(data) => return Ok(data),
                    Outcome::Abort(err) => return Err(err),
                    Outcome::Busy => {
                        thread::sleep(cfg.busy_delay);
                        reason = Some("busy");
                        break;
                    }
                    Outcome::Retry(why) => {
                        reason = Some(why);
                        break;
                    }
                }
            }
            if Instant::now() >= deadline {
                break;
            }
            match reader.read(&mut buf) {
                Ok(0) => thread::sleep(READ_POLL),
                Ok(n) => scanner.feed(&buf[..n]),
                Err(e) if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut => {
                    thread::sleep(READ_POLL)
                }
                Err(e) => return Err(FlashError::Io(e)),
            }
        }
        last = reason.unwrap_or("no response before read timeout");
    }
    Err(FlashError::RetriesExhausted {
        cmd,
        tries: cfg.tries.max(1),
        last,
    })
}

/// `CONNECT`: returns protocol version, start address, block size and MCU
/// identity; rejects a block size no katapult frame can carry.
///
/// A block arrives as one `SEND_BLOCK` frame: `12 + block_size` bytes (header and
/// trailer, the address word, the data). katapult's frames stop at `MESSAGE_MAX`
/// 128 B (`third_party/katapult/src/command.h`, enforced by `command_find_block`,
/// which NACKs and drops such a frame), so the 128, 256 and 512 B blocks
/// `flashtool.py` would accept cannot travel: the session would retry each one
/// out instead of saying why. 64 B — what katapult's arch Kconfigs build with by
/// default (`BLOCK_SIZE`) and what a real bootloader reports — is the largest of
/// those sizes that fits one frame, so a larger one is refused here.
pub fn connect<R: Read, W: Write>(
    reader: &mut R,
    writer: &mut W,
    cfg: &SessionConfig,
) -> Result<ConnectInfo, FlashError> {
    let data = command(reader, writer, frame::command::CONNECT, &[], cfg)?;
    let info = ConnectInfo::parse(&data)
        .ok_or_else(|| FlashError::Protocol("CONNECT ack shorter than 12 bytes".into()))?;
    if info.block_size == 0 || info.block_size > 64 {
        return Err(FlashError::Protocol(format!(
            "invalid block size {}: a SEND_BLOCK frame is 12 + block_size bytes and a katapult \
             frame stops at 128 bytes (MESSAGE_MAX), so only a 64-byte block fits",
            info.block_size
        )));
    }
    Ok(info)
}

/// `SEND_BLOCK`: one address + data block; retries the command while the
/// device echoes the wrong address (three times, like `flashtool.py`).
pub fn send_block<R: Read, W: Write>(
    reader: &mut R,
    writer: &mut W,
    addr: u32,
    data: &[u8],
    cfg: &SessionConfig,
) -> Result<(), FlashError> {
    let mut payload = addr.to_le_bytes().to_vec();
    payload.extend_from_slice(data);
    let mut got = 0u32;
    for _ in 1..=ADDRESS_TRIES {
        let resp = command(reader, writer, frame::command::SEND_BLOCK, &payload, cfg)?;
        if resp.len() != 4 {
            return Err(FlashError::Protocol(format!(
                "SEND_BLOCK ack is {} bytes, expected 4",
                resp.len()
            )));
        }
        got = u32::from_le_bytes(resp.as_slice().try_into().expect("four bytes"));
        if got == addr {
            return Ok(());
        }
    }
    Err(FlashError::AddressMismatch {
        cmd: frame::command::SEND_BLOCK,
        expected: addr,
        got,
    })
}

/// `EOF`: flushes the write phase, returns the bootloader's page count.
pub fn eof<R: Read, W: Write>(
    reader: &mut R,
    writer: &mut W,
    cfg: &SessionConfig,
) -> Result<u32, FlashError> {
    let resp = command(reader, writer, frame::command::EOF, &[], cfg)?;
    if resp.len() != 4 {
        return Err(FlashError::Protocol(format!(
            "EOF ack is {} bytes, expected 4",
            resp.len()
        )));
    }
    Ok(u32::from_le_bytes(
        resp.as_slice().try_into().expect("four bytes"),
    ))
}

/// `REQUEST_BLOCK`: reads one block back; like [`send_block`] it retries a
/// wrong echoed address three times and returns the data after the address.
pub fn request_block<R: Read, W: Write>(
    reader: &mut R,
    writer: &mut W,
    addr: u32,
    cfg: &SessionConfig,
) -> Result<Vec<u8>, FlashError> {
    let mut got = 0u32;
    for _ in 1..=ADDRESS_TRIES {
        let resp = command(
            reader,
            writer,
            frame::command::REQUEST_BLOCK,
            &addr.to_le_bytes(),
            cfg,
        )?;
        if resp.len() < 4 {
            return Err(FlashError::Protocol(format!(
                "REQUEST_BLOCK ack shorter than one address word: {} bytes",
                resp.len()
            )));
        }
        got = u32::from_le_bytes(resp[..4].try_into().expect("four bytes"));
        if got == addr {
            return Ok(resp[4..].to_vec());
        }
    }
    Err(FlashError::AddressMismatch {
        cmd: frame::command::REQUEST_BLOCK,
        expected: addr,
        got,
    })
}

/// `COMPLETE`: hands control back to the application; the payload (if any)
/// is ignored, `flashtool.py` does the same.
pub fn complete<R: Read, W: Write>(
    reader: &mut R,
    writer: &mut W,
    cfg: &SessionConfig,
) -> Result<(), FlashError> {
    command(reader, writer, frame::command::COMPLETE, &[], cfg)?;
    Ok(())
}

/// `GET_CANBUS_ID`: the six-byte CANbus UUID the bootloader reports.
pub fn get_canbus_id<R: Read, W: Write>(
    reader: &mut R,
    writer: &mut W,
    cfg: &SessionConfig,
) -> Result<[u8; 6], FlashError> {
    let resp = command(reader, writer, frame::command::GET_CANBUS_ID, &[], cfg)?;
    if resp.len() < 6 {
        return Err(FlashError::Protocol(format!(
            "GET_CANBUS_ID ack shorter than 6 bytes: {}",
            resp.len()
        )));
    }
    Ok(resp[..6].try_into().expect("six bytes"))
}

/// Flash `payload` with the default timing (see [`SessionConfig::default`]).
/// `progress` receives simple text lines as the session advances.
pub fn flash<R: Read, W: Write>(
    reader: &mut R,
    writer: &mut W,
    payload: &[u8],
    progress: impl FnMut(&str),
) -> Result<Summary, FlashError> {
    flash_with_config(reader, writer, payload, &SessionConfig::default(), progress)
}

/// [`flash`] with explicit timing — the knob tests use to stay fast and
/// bounded. Order: connect → every block → eof → verify every block →
/// complete; the last block is padded to `block_size` with `0xFF`, and verify
/// compares what the device read back against those exact padded bytes.
pub fn flash_with_config<R: Read, W: Write>(
    reader: &mut R,
    writer: &mut W,
    payload: &[u8],
    cfg: &SessionConfig,
    mut progress: impl FnMut(&str),
) -> Result<Summary, FlashError> {
    let info = connect(reader, writer, cfg)?;
    let block_size = info.block_size as usize;
    progress(&format!(
        "connect: MCU {}, block size {}, start 0x{:08X}",
        info.mcu_type, info.block_size, info.start_addr
    ));

    let blocks: Vec<Vec<u8>> = payload
        .chunks(block_size)
        .map(|chunk| {
            let mut block = chunk.to_vec();
            block.resize(block_size, 0xFF);
            block
        })
        .collect();

    for (i, block) in blocks.iter().enumerate() {
        let addr = info.start_addr + (i * block_size) as u32;
        send_block(reader, writer, addr, block, cfg)?;
        progress(&format!("block {}/{} written", i + 1, blocks.len()));
    }

    let page_count = eof(reader, writer, cfg)?;
    progress(&format!("eof: {page_count} pages"));

    for (i, want) in blocks.iter().enumerate() {
        let addr = info.start_addr + (i * block_size) as u32;
        let read_back = request_block(reader, writer, addr, cfg)?;
        if &read_back != want {
            return Err(FlashError::VerifyMismatch { addr });
        }
        progress(&format!("verify {}/{}", i + 1, blocks.len()));
    }

    complete(reader, writer, cfg)?;
    progress(&format!(
        "complete: {} bytes in {} blocks",
        payload.len(),
        blocks.len()
    ));

    Ok(Summary {
        start_addr: info.start_addr,
        block_size: info.block_size,
        block_count: blocks.len(),
        page_count,
        bytes_flashed: payload.len(),
        mcu_type: info.mcu_type,
        proto_version: info.proto_version,
        software_version: info.software_version,
    })
}
