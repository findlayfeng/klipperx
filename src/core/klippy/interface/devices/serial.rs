//! The serial device: a real MCU on a tty.
//!
//! Klipper's MCU link is a byte stream over a serial port, so this device encodes
//! frames on the way out and reassembles them on the way in through the shared
//! [`FrameStream`] — the same split as the host library device, and the same two
//! rules: `send` writes a whole frame or reports an error, and `receive` only ever
//! returns validated frames.
//!
//! Two details are specific to a tty:
//!
//! * the line has to be raw (no echo, no CR/LF translation, no flow control the
//!   protocol did not ask for), and the speed has to be set. Klipper's default is
//!   250000 baud, which is not one of the portable `B*` constants, so the speed
//!   goes through Linux's `termios2`/`BOTHER`, which takes an arbitrary rate;
//! * reads use a short timeout (`VMIN=0`, `VTIME=1`) instead of blocking forever.
//!   `shutdown` is therefore noticed within a tenth of a second, without closing
//!   the descriptor out from under a thread that is blocked in `read`.
//!
//! # The shared serial mux
//!
//! This file also carries the **serial mux**. When one `serial:` path is named
//! by two or more `[mcu]` sections (`mcu/serial_ports.rs` groups them at load
//! time), the port is opened **once** and every unit on the wire is prefixed
//! with its sender's arbitration id, so a single reader cuts the byte stream
//! into whole units and hands each to the MCU its header names
//! ([`SharedSerialPort`] / [`SharedSerialHandle`]). That half owns the mux wire
//! format — the checked 2-byte header, the KXID assignment pseudo-frame, the
//! assignment-time handshake, the discovery/assignment admin channel and the
//! tunnel unit (one katapult message block carried whole) — and
//! rides the shared reuse spine `resource_for`. It is serial wire logic only;
//! the wire spec is the developer manual's *serial-mux-protocol* page.
//!
//! [`FrameStream`]: crate::core::klippy::frame::FrameStream

use crate::core::klippy::assign;
use crate::core::klippy::config::mcu::MAX_MCU_ID;
use crate::core::klippy::frame::{
    Frame, FrameStream, MESSAGE_HEADER_SIZE, MESSAGE_MAX, MESSAGE_MIN, MESSAGE_POS_LEN,
    MESSAGE_POS_SEQ, MESSAGE_SYNC, MESSAGE_TRAILER_SIZE, MESSAGE_TRAILER_SYNC,
};
use crate::core::klippy::interface::error::InterfaceError;
use crate::core::klippy::interface::{describe_frame, Device};
use std::collections::HashMap;
use std::fmt;
use std::fs::{File, OpenOptions};
use std::io::{ErrorKind, Read, Write};
use std::os::fd::AsRawFd;
use std::os::unix::fs::OpenOptionsExt;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Condvar, Mutex, OnceLock, Weak};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};
use tracing::{debug, info, trace, warn};

use super::shared::resource_for;

/// Klipper's default line speed, used by a config that does not say otherwise.
pub const DEFAULT_BAUD: u32 = 250_000;

/// `VTIME` is in tenths of a second, and only applies while `VMIN` is 0.
const READ_TIMEOUT_TENTHS: u8 = 1;

/// A [`Device`] backed by a serial port.
pub struct SerialDevice {
    port: File,
    path: PathBuf,
    baud: u32,
    /// Bytes read from the port, reassembled into frames.
    stream: Mutex<FrameStream>,
    /// Set by `shutdown`; the read loop checks it between timeouts.
    stopped: AtomicBool,
}

impl SerialDevice {
    /// Returns a short identifier for logging: `"/dev/ttyACM0"`.
    fn id(&self) -> String {
        self.path.display().to_string()
    }

    /// Open `path` at `baud` and put the line into raw mode.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] when the port cannot be opened or configured.
    pub fn open(path: impl AsRef<Path>, baud: u32) -> Result<Self, InterfaceError> {
        let path = path.as_ref().to_path_buf();
        let port = OpenOptions::new()
            .read(true)
            .write(true)
            // `O_NOCTTY`: opening a printer's port must not hand this process a
            // controlling terminal. `O_NONBLOCK`: a port with no carrier detect
            // would otherwise block inside `open`; it is cleared again below.
            .custom_flags(libc::O_NOCTTY | libc::O_NONBLOCK)
            .open(&path)
            .map_err(|e| {
                InterfaceError::Other(format!(
                    "failed to open serial port {}: {e}",
                    path.display()
                ))
            })?;

        set_raw_mode(&port, baud)?;
        clear_nonblocking(&port)?;

        info!("serial port {} open at {baud} baud", path.display());
        Ok(Self {
            port,
            path,
            baud,
            stream: Mutex::new(FrameStream::new()),
            stopped: AtomicBool::new(false),
        })
    }

    /// The port this device was opened on.
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// The configured line speed.
    pub fn baud(&self) -> u32 {
        self.baud
    }

    /// Read raw bytes straight off the open port.
    ///
    /// A pass-through to the tty, for a bench tool that wants the byte stream
    /// rather than reassembled frames ([`Device::receive`]): a discovery scan
    /// needs the uninterpreted units, admin ids and all. The line is in raw
    /// mode, so the bytes come back exactly as they arrived; a read blocks at
    /// most the tty's `VTIME` window and returns `0` when nothing arrived in
    /// it.
    pub fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        (&self.port).read(buf)
    }

    /// Write raw bytes straight to the open port.
    ///
    /// A pass-through for the same reason as [`SerialDevice::read`]: a
    /// discovery scan sends units (header plus `KXID` frame) that are not
    /// ordinary encoded [`Frame`]s. The tty has no user-space buffering, so a
    /// write is whole, and raw mode means nothing is translated on the way.
    pub fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        (&self.port).write(buf)
    }

    /// Assert or deassert RTS on the open port.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] if the kernel rejects the ioctl.
    pub fn set_rts(&self, asserted: bool) -> Result<(), InterfaceError> {
        set_modem_line(&self.port, libc::TIOCM_RTS, asserted)
    }
}

impl fmt::Debug for SerialDevice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SerialDevice")
            .field("path", &self.path)
            .field("baud", &self.baud)
            .field("stopped", &self.stopped.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl std::io::Read for SerialDevice {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        SerialDevice::read(self, buf)
    }
}

impl std::io::Write for SerialDevice {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        SerialDevice::write(self, buf)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        // The tty has no user-space buffering, so there is nothing to flush.
        Ok(())
    }
}

impl Device for SerialDevice {
    fn send(&self, frame: &Frame) -> Result<(), InterfaceError> {
        if self.stopped.load(Ordering::Relaxed) {
            return Err(InterfaceError::ConnectionLost);
        }

        let bytes = frame.raw_bytes();
        trace!("tx frame [{}]: {}", self.id(), describe_frame(&bytes));
        (&self.port).write_all(&bytes).map_err(|e| {
            InterfaceError::SendError(format!(
                "serial write to {} failed: {e}",
                self.path.display()
            ))
        })?;
        debug!("sent {} bytes to {}", bytes.len(), self.path.display());
        Ok(())
    }

    fn receive(&self) -> Option<Frame> {
        let mut buf = [0u8; 256];
        loop {
            if self.stopped.load(Ordering::Relaxed) {
                return None;
            }
            if let Some(frame) = self.stream.lock().unwrap().next_frame() {
                return Some(frame);
            }

            // Blocks for at most READ_TIMEOUT_TENTHS, so the stop flag is checked
            // regularly without ever closing the descriptor behind a live `read`.
            match (&self.port).read(&mut buf) {
                Ok(0) => continue, // timeout: no bytes were waiting
                Ok(read) => {
                    trace!("rx frame [{}]: {}", self.id(), describe_frame(&buf[..read]));
                    debug!("received {} bytes from {}", read, self.path.display());
                    self.stream.lock().unwrap().push(&buf[..read]);
                }
                Err(e) if e.kind() == ErrorKind::Interrupted => continue,
                Err(e) => {
                    // A vanished adapter (EIO) ends the stream, like a shutdown.
                    warn!("serial read from {} failed: {e}", self.path.display());
                    return None;
                }
            }
        }
    }

    fn shutdown(&self) {
        self.stopped.store(true, Ordering::SeqCst);
    }
}

/// Modem-line control for a tty, for the firmware-restart sequences.
///
/// A reset needs the port to itself: it opens it briefly, at another rate, and
/// toggles the control lines. It therefore has to run while the device is
/// **closed**, which is why it is a short-lived handle rather than a device — it
/// never reads or writes frames (`klippy/serialhdl.py:365-402`).
pub struct ModemLines {
    port: File,
    path: PathBuf,
}

impl ModemLines {
    /// Open `path` at `baud` for control only.
    ///
    /// # Errors
    /// As [`SerialDevice::open`]: the port cannot be opened or put into raw
    /// mode.
    pub fn open(path: impl AsRef<Path>, baud: u32) -> Result<Self, InterfaceError> {
        let path = path.as_ref().to_path_buf();
        let port = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NOCTTY | libc::O_NONBLOCK)
            .open(&path)
            .map_err(|e| {
                InterfaceError::Other(format!(
                    "failed to open serial port {}: {e}",
                    path.display()
                ))
            })?;
        set_raw_mode(&port, baud)?;
        Ok(Self { port, path })
    }

    /// Read and discard one pending byte.
    ///
    /// The reset sequence drains the port first, the way upstream does
    /// (`serialhdl.py:396`): a byte already in flight must not be left to be
    /// read as the board's answer to the reset.
    pub fn drain(&self) {
        let mut scratch = [0u8; 1];
        let _ = (&self.port).read(&mut scratch);
    }

    /// Assert or deassert DTR — the line an Arduino-style board resets on.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] if the kernel rejects the ioctl.
    pub fn set_dtr(&self, asserted: bool) -> Result<(), InterfaceError> {
        set_modem_line(&self.port, libc::TIOCM_DTR, asserted)
    }

    /// Assert or deassert RTS.
    ///
    /// Cheetah boards need RTS deasserted for the whole connection, or a reset
    /// triggers the built-in bootloader (`klippy/mcu.py:703-705`).
    ///
    /// # Errors
    /// As [`ModemLines::set_dtr`].
    pub fn set_rts(&self, asserted: bool) -> Result<(), InterfaceError> {
        set_modem_line(&self.port, libc::TIOCM_RTS, asserted)
    }
}

impl fmt::Debug for ModemLines {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ModemLines")
            .field("path", &self.path)
            .finish_non_exhaustive()
    }
}

/// Set or clear one modem-control line with `TIOCMBIS`/`TIOCMBIC`.
fn set_modem_line(port: &File, line: libc::c_int, asserted: bool) -> Result<(), InterfaceError> {
    let fd = port.as_raw_fd();
    let mut bits: libc::c_int = line;
    let (request, call) = if asserted {
        (libc::TIOCMBIS, "TIOCMBIS")
    } else {
        (libc::TIOCMBIC, "TIOCMBIC")
    };
    // SAFETY: the ioctl takes a pointer to an `int` holding the modem bits.
    if unsafe { libc::ioctl(fd, request, &mut bits) } != 0 {
        return Err(last_error(call));
    }
    Ok(())
}

/// Put the tty into raw mode and set its speed.
///
/// The speed is set through `termios2`/`BOTHER`, which takes an arbitrary rate:
/// Klipper's 250000 is not one of the portable `B*` constants, and the usual
/// alternative (`TIOCSSERIAL`) needs privileges.
///
/// Raw mode is what `cfmakeraw` would do: no echo, no line editing, no signal
/// characters, no CR/LF translation and no software flow control, so the only
/// bytes on the wire are the ones the protocol put there.
pub(crate) fn set_raw_mode(port: &File, baud: u32) -> Result<(), InterfaceError> {
    let fd = port.as_raw_fd();

    // SAFETY: `fd` is an open tty owned by `port`, and `tio` is a plain struct the
    // kernel fills in; TCSETS2 reads it back.
    let mut tio: libc::termios2 = unsafe { std::mem::zeroed() };
    if unsafe { libc::ioctl(fd, libc::TCGETS2, &mut tio) } != 0 {
        return Err(last_error("TCGETS2"));
    }

    tio.c_iflag &= !(libc::IGNBRK
        | libc::BRKINT
        | libc::PARMRK
        | libc::ISTRIP
        | libc::INLCR
        | libc::IGNCR
        | libc::ICRNL
        | libc::IXON);
    tio.c_oflag &= !libc::OPOST;
    tio.c_lflag &= !(libc::ECHO | libc::ECHONL | libc::ICANON | libc::ISIG | libc::IEXTEN);
    tio.c_cflag &= !(libc::CSIZE | libc::PARENB);
    tio.c_cflag |= libc::CS8 | libc::CREAD | libc::CLOCAL;
    tio.c_cflag &= !libc::CRTSCTS;
    // Non-canonical reads: return as soon as anything is there, or after VTIME.
    tio.c_cc[libc::VMIN] = 0;
    tio.c_cc[libc::VTIME] = READ_TIMEOUT_TENTHS;
    // Arbitrary rate: the kernel takes the numbers from `c_ispeed`/`c_ospeed`
    // when `BOTHER` is selected.
    tio.c_cflag = (tio.c_cflag & !libc::CBAUD) | libc::BOTHER;
    tio.c_ispeed = baud;
    tio.c_ospeed = baud;

    if unsafe { libc::ioctl(fd, libc::TCSETS2, &tio) } != 0 {
        return Err(last_error("TCSETS2"));
    }
    Ok(())
}

/// Undo the `O_NONBLOCK` from `open`, so reads block (bounded by `VTIME`).
pub(crate) fn clear_nonblocking(port: &File) -> Result<(), InterfaceError> {
    let fd = port.as_raw_fd();
    // SAFETY: F_GETFL/F_SETFL on an open descriptor.
    let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
    if flags < 0 {
        return Err(last_error("F_GETFL"));
    }
    if unsafe { libc::fcntl(fd, libc::F_SETFL, flags & !libc::O_NONBLOCK) } < 0 {
        return Err(last_error("F_SETFL"));
    }
    Ok(())
}

/// The current `errno` as an `InterfaceError`, naming the call that failed.
pub(crate) fn last_error(call: &str) -> InterfaceError {
    InterfaceError::Other(format!(
        "{call} failed: {}",
        std::io::Error::last_os_error()
    ))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::interface::pty::Pty;
    use std::sync::Arc;
    use std::time::Duration;

    #[test]
    fn test_open_reports_a_missing_port() {
        let err = SerialDevice::open("/dev/not-a-serial-port", DEFAULT_BAUD).unwrap_err();
        assert!(matches!(err, InterfaceError::Other(_)), "{err:?}");
        assert!(err.to_string().contains("/dev/not-a-serial-port"), "{err}");
    }

    #[test]
    fn test_modem_lines_open_a_tty() {
        // A pty is a real tty, so opening it and putting it into raw mode is
        // exercised for real. The DTR/RTS ioctls are not: a pty slave does not
        // emulate the modem lines (`TIOCMBIS` is `ENOTTY` there), so toggling
        // them is left to the boards that have them.
        let pty = Pty::new();
        let lines = ModemLines::open(&pty.path, 2400).unwrap();
        lines.drain();
    }

    #[test]
    fn test_send_writes_the_encoded_frame() {
        let pty = Pty::new();
        let device = SerialDevice::open(&pty.path, DEFAULT_BAUD).unwrap();
        assert_eq!(device.path(), pty.path);
        assert_eq!(device.baud(), DEFAULT_BAUD);

        device.send(&Frame::new(3, vec![1, 2, 3])).unwrap();

        // What the port receives is the frame as it goes on the wire: header,
        // payload, CRC, SYNC. Raw mode means nothing was translated on the way.
        let mut buf = [0u8; 64];
        let read = (&pty.master).read(&mut buf).unwrap();
        assert_eq!(&buf[..read], Frame::encode(3, &[1, 2, 3]));
    }

    #[test]
    fn test_raw_read_and_write_pass_through_to_the_port() {
        // The discovery scan drives the port through these raw read/write
        // methods rather than through the Device trait, so the pass-through has
        // to move bytes both ways as-is, in raw mode.
        let pty = Pty::new();
        let mut device = SerialDevice::open(&pty.path, DEFAULT_BAUD).unwrap();

        device.write(b"ping").unwrap();
        let mut buf = [0u8; 8];
        let mut seen = 0;
        while seen < 4 {
            let read = (&pty.master).read(&mut buf[seen..]).unwrap();
            assert_ne!(read, 0, "the pty master closed");
            seen += read;
        }
        assert_eq!(&buf[..seen], b"ping");

        (&pty.master).write_all(b"pong").unwrap();
        let mut out = [0u8; 8];
        let mut got = 0;
        while got < 4 {
            let read = device.read(&mut out[got..]).unwrap();
            assert_ne!(read, 0, "the read window elapsed before the bytes");
            got += read;
        }
        assert_eq!(&out[..got], b"pong");
    }

    #[test]
    fn test_receive_reassembles_frames_from_the_port() {
        let pty = Pty::new();
        let device = SerialDevice::open(&pty.path, DEFAULT_BAUD).unwrap();

        // Two frames, the first one arriving in two pieces.
        let first = Frame::encode(1, b"one");
        let second = Frame::encode(2, b"two");
        (&pty.master).write_all(&first[..3]).unwrap();
        (&pty.master)
            .write_all(&[&first[3..], &second[..]].concat())
            .unwrap();

        assert_eq!(device.receive(), Some(Frame::new(1, b"one".to_vec())));
        assert_eq!(device.receive(), Some(Frame::new(2, b"two".to_vec())));
    }

    /// The same round trip through `Interface`, which is how a transport uses the
    /// device: send and receive run on the blocking pool.
    #[tokio::test]
    async fn test_interface_round_trip() {
        use crate::core::klippy::interface::Interface;

        let pty = Pty::new();
        let interface = Interface::serial(&pty.path, DEFAULT_BAUD).unwrap();

        interface.send(Frame::new(0, vec![7])).await.unwrap();
        let mut buf = [0u8; 16];
        let read = (&pty.master).read(&mut buf).unwrap();
        assert_eq!(&buf[..read], Frame::encode(0, &[7]));

        let answer = Frame::encode(1, &[9]);
        (&pty.master).write_all(&answer).unwrap();
        assert_eq!(interface.receive().await, Some(Frame::new(1, vec![9])));

        interface.shutdown();
        assert_eq!(interface.receive().await, None);
    }

    #[test]
    fn test_shutdown_unblocks_receive() {
        let pty = Pty::new();
        let device = Arc::new(SerialDevice::open(&pty.path, DEFAULT_BAUD).unwrap());

        // Nothing is written, so `receive` waits in the read timeout loop.
        let reader = Arc::clone(&device);
        let handle = std::thread::spawn(move || reader.receive());
        std::thread::sleep(Duration::from_millis(50));
        device.shutdown();

        assert_eq!(
            handle.join().expect("reader thread"),
            None,
            "shutdown must end the stream rather than block until a frame arrives"
        );
    }
}

// ===========================================================================
// The shared serial mux: one tty, several MCUs
// ===========================================================================
//
// One tty can carry more than one MCU once every unit on it is prefixed with
// its sender's arbitration id: the port is opened **once** (one reader cutting
// the byte stream into whole units, routed by a 2-byte checked header), and each
// MCU gets a handle ([`SharedSerialHandle`]). The reuse shape (one resource per
// path, opened once, torn down with its last reference) lives in
// [`shared::resource_for`]; everything here is serial wire logic. The wire
// format is specified in the developer manual's *serial-mux-protocol* page, and
// the grouping that decides exclusive vs. shared is `mcu/serial_ports.rs`.

pub const ASSIGN_ATTACH_TIMEOUT: Duration = Duration::from_millis(500);

/// How many times `attach` re-sends the request before giving up.
///
/// A lost or ignored frame must not cost the whole session: each retry re-sends
/// the request and waits another `ASSIGN_ATTACH_TIMEOUT`.
pub const ASSIGN_ATTACH_ATTEMPTS: u32 = 3;

/// How many bytes one `read` may bring in. Bounds the reader's stack buffer;
/// larger reads simply take another turn.
const READ_CHUNK: usize = 1024;

// ===========================================================================
// The frame header and the assignment pseudo-frame
// ===========================================================================

/// The sequence byte the assignment pattern uses, as a whole byte: bit 5 set,
/// the bits `MESSAGE_DEST` owns clear, the low nibble free.
///
/// Legitimate frames — in either direction — carry `0x10..=0x1f`: bit 4 set by
/// `MESSAGE_DEST`, bits 5-7 zero. `0x20` occupies a bit nothing else uses, so
/// neither direction's ordinary traffic can produce it by accident, and the
/// low nibble stays available for a sequence number should one matter.
pub const ASSIGN_SEQ_BYTE: u8 = 0x20;

/// The payload marker: the fixed bytes every device built to parse this pattern
/// looks for, and — once written out — the whole body of the pseudo-frame.
///
/// `0x7e` never appears in it, which keeps the frame's header and payload free
/// of an interior SYNC byte (see the module docs: the discard path stops at the
/// first `0x7e`). The two CRC bytes are computed, not chosen; for this fixed
/// body they are `0x4b`/`0x4f`, so the trailer's `0x7e` is the only SYNC in the
/// frame.
pub const MAGIC: &[u8; 4] = b"KXID";

/// The two bytes a frame header occupies.
pub const HEADER_BYTES: usize = 2;

/// The data bits the header carries.
const HEADER_DATA_BITS: u32 = 10;

/// The largest value the ten data bits hold.
pub const HEADER_MAX: u32 = (1 << HEADER_DATA_BITS) - 1;

/// The first value of the reserved band (`0x3f2`): legal codewords that name no
/// address. [`HEADER_ESCAPE`] is where a longer header would start, so a reader
/// that decodes any of them drops the unit.
pub const HEADER_RESERVED_FIRST: u32 = 0x3f2;

/// The escape value (`0x3ff`): the first byte of a longer, extended header if
/// one is ever defined. Nothing emits it today.
pub const HEADER_ESCAPE: u32 = 0x3ff;

/// The codeword's total width: ten data bits plus four check bits.
const HEADER_CODE_BITS: u32 = 14;

/// Where the four check bits sit in the codeword (1-indexed, most significant
/// bit first): the powers of two, as in a Hamming code.
const HEADER_CHECK_POSITIONS: [u32; 4] = [1, 2, 4, 8];

/// Where the ten data bits sit: every other position. Position 15 — the one a
/// full 15-bit Hamming code would add — is what this shortened code drops.
const HEADER_DATA_POSITIONS: [u32; 10] = [3, 5, 6, 7, 9, 10, 11, 12, 13, 14];

/// The codeword for `arb`: its ten data bits, plus the four check bits that
/// make every column of the parity-check matrix sum to even.
///
/// The matrix's columns are the 14 positions — every non-zero four-bit pattern
/// but 15 — so they are distinct and non-zero, the code's minimum distance is
/// three, and **every one- or two-bit error changes the syndrome**. That is the
/// point of the code (see the module docs); no error is ever corrected, only
/// caught.
fn header_codeword(arb: u32) -> u16 {
    let mut code = 0u16;
    for (bit, &position) in HEADER_DATA_POSITIONS.iter().enumerate() {
        if arb & (1 << bit) != 0 {
            code |= 1 << (HEADER_CODE_BITS - position);
        }
    }
    for (check, &position) in HEADER_CHECK_POSITIONS.iter().enumerate() {
        // This check bit covers the positions whose own encoding has bit
        // `check` set; it is set to make the group, plus itself, even.
        let mut covered = 0usize;
        for (bit, &data) in HEADER_DATA_POSITIONS.iter().enumerate() {
            if (data >> check) & 1 == 1 && arb & (1 << bit) != 0 {
                covered += 1;
            }
        }
        if covered % 2 == 1 {
            code |= 1 << (HEADER_CODE_BITS - position);
        }
    }
    code
}

/// The syndrome of a codeword: the XOR of the positions of its set bits, zero
/// exactly for a well-formed codeword.
///
/// A one-bit error leaves the syndrome equal to the flipped position and a
/// two-bit error to the XOR of the two — non-zero either way, because the 14
/// positions are distinct non-zero values.
fn header_syndrome(code: u16) -> u8 {
    let mut syndrome = 0u8;
    for position in 1..=HEADER_CODE_BITS {
        if code & (1 << (HEADER_CODE_BITS - position)) != 0 {
            syndrome ^= position as u8;
        }
    }
    syndrome
}

/// The ten data bits of a well-formed codeword.
fn header_data(code: u16) -> u32 {
    let mut arb = 0u32;
    for (bit, &position) in HEADER_DATA_POSITIONS.iter().enumerate() {
        if code & (1 << (HEADER_CODE_BITS - position)) != 0 {
            arb |= 1 << bit;
        }
    }
    arb
}

/// A codeword as the two bytes that go on the wire. The leading `0x80` and the
/// clear high bit of the second byte are the framing kept from the
/// variable-length encoding this replaces: they are what makes the two-byte
/// boundary unambiguous on the stream.
fn header_bytes(code: u16) -> [u8; HEADER_BYTES] {
    [0x80 | ((code >> 7) & 0x7f) as u8, (code & 0x7f) as u8]
}

/// Encode a frame header: the arbitration id as the checked, two-byte codeword
/// above.
///
/// # Panics
///
/// Panics if `arb` is in the reserved band ([`HEADER_RESERVED_FIRST`] and up),
/// which names no address — a programming error, since every caller validates
/// first ([`assign::arb_id`], the admin constants).
pub fn encode_header(arb: u32) -> Vec<u8> {
    assert!(
        arb < HEADER_RESERVED_FIRST,
        "arbitration id {arb:#x} is reserved (0x{HEADER_RESERVED_FIRST:x}..=0x{HEADER_MAX:x}) \
         and names no address"
    );
    header_bytes(header_codeword(arb)).to_vec()
}

/// The outcome of reading a frame header off the front of a byte slice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeaderScan {
    /// A whole arbitration id, occupying `consumed` bytes.
    Complete { arb: u32, consumed: usize },
    /// The slice ends inside the header; more bytes could complete it.
    NeedMore,
    /// The bytes are not a header: a framing bit is wrong, the check bits do
    /// not match (every one- and two-bit error), or the codeword is in the
    /// reserved band.
    Invalid,
}

/// Decode the frame header at the front of `buf`.
pub fn decode_header(buf: &[u8]) -> HeaderScan {
    if buf.len() < HEADER_BYTES {
        return HeaderScan::NeedMore;
    }
    let (first, second) = (buf[0], buf[1]);
    // The two framing bits are fixed: a pair that breaks them is not a header,
    // whatever its check bits say (this catches two more bit errors, and keeps
    // the two-byte boundary unambiguous).
    if first & 0x80 == 0 || second & 0x80 != 0 {
        return HeaderScan::Invalid;
    }
    let code = ((first & 0x7f) as u16) << 7 | (second & 0x7f) as u16;
    if header_syndrome(code) != 0 {
        return HeaderScan::Invalid;
    }
    let arb = header_data(code);
    // A well-formed codeword in the reserved band names no address: the unit is
    // dropped (this is where a longer header would be recognized instead).
    if arb >= HEADER_RESERVED_FIRST {
        return HeaderScan::Invalid;
    }
    HeaderScan::Complete {
        arb,
        consumed: HEADER_BYTES,
    }
}

/// The pseudo-frame an assignment unit carries after its header: the `KXID`
/// pattern with a payload of [`MAGIC`] alone.
///
/// The id is *not* in here — it is the unit's header — so the same body goes in
/// both directions, and the frame is
/// `[len=9][0x20]["KXID"][crc16 hi][crc16 lo][0x7e]`.
fn assignment_frame() -> Vec<u8> {
    kxid_unit(&[])
}

/// The assignment unit addressed to `mcu_id`: the header on its listener
/// arbitration id, then the pseudo-frame.
pub fn assignment_request(mcu_id: u32) -> Option<Vec<u8>> {
    let mut unit = encode_header(assign::arb_id(mcu_id)?);
    unit.extend_from_slice(&assignment_frame());
    Some(unit)
}

/// The answer `mcu_id` sends for [`assignment_request`]: the header on its
/// answer arbitration id, then the same pseudo-frame.
pub fn assignment_reply(mcu_id: u32) -> Option<Vec<u8>> {
    let mut unit = encode_header(assign::reply_arb_id(mcu_id)?);
    unit.extend_from_slice(&assignment_frame());
    Some(unit)
}

/// Recognize an assignment answer and return the id it names.
///
/// Everything must be self-consistent: a whole header whose arbitration id
/// is the reply direction ([`is_reply_arb`](assign::is_reply_arb)) and decodes
/// to an [`mcu_id`](assign::mcu_id_from_arb), then a pseudo-frame whose length
/// byte matches the bytes present, whose sequence byte is [`ASSIGN_SEQ_BYTE`],
/// whose trailer and CRC check out, and whose payload is exactly [`MAGIC`]. A
/// request (even header) is rejected by the direction check.
pub fn parse_assignment_ack(unit: &[u8]) -> Option<u32> {
    let (arb, consumed) = match decode_header(unit) {
        HeaderScan::Complete { arb, consumed } => (arb, consumed),
        HeaderScan::NeedMore | HeaderScan::Invalid => return None,
    };
    if !assign::is_reply_arb(arb) {
        return None;
    }
    let mcu_id = assign::mcu_id_from_arb(arb)?;
    let frame = &unit[consumed..];
    if frame.len() < MESSAGE_MIN {
        return None;
    }
    let len = frame[MESSAGE_POS_LEN] as usize;
    if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&len) || frame.len() != len {
        return None;
    }
    if frame[MESSAGE_POS_SEQ] != ASSIGN_SEQ_BYTE {
        return None;
    }
    if frame[len - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC {
        return None;
    }
    let expected = crate::crc16_ccitt!(&frame[..len - MESSAGE_TRAILER_SIZE]);
    let actual = ((frame[len - 3] as u16) << 8) | (frame[len - 2] as u16);
    if expected != actual {
        return None;
    }
    let payload = &frame[MESSAGE_HEADER_SIZE..len - MESSAGE_TRAILER_SIZE];
    if payload != &MAGIC[..] {
        return None;
    }
    Some(mcu_id)
}

// ===========================================================================
// Unassigned-device discovery: the admin broadcast, on the shared serial wire
// ===========================================================================

/// The discovery conversation, spelled with CAN's own values so an id or a
/// command means the same thing on either transport.
///
/// The admin ids sit outside the assigned range
/// (`0x102..=0x2ff`, [`crate::core::klippy::assign`]) the way they do for CAN,
/// and the parity convention is the same: the even id addresses the admin, the
/// odd id is its reply. The commands and the response byte are CAN's too — an
/// unassigned board answers discovery the same way however it is asked.
pub const ADMIN_LISTEN: u32 = 0x3f0;
/// The id the admin answer arrives on, one past [`ADMIN_LISTEN`].
pub const ADMIN_REPLY: u32 = 0x3f1;
/// The frame body byte that asks every board with no node id to answer.
pub const CMD_QUERY_UNASSIGNED: u8 = 0x00;
/// The frame body byte that hands a board the node id it is to take.
pub const CMD_SET_KLIPPER_NODEID: u8 = 0x01;
/// The frame body byte that asks the device behind a bridge port to enter its
/// bootloader — CAN's `CANBUS_CMD_REQUEST_BOOTLOADER` by value and meaning,
/// and the **flashing tool's** command: a board in its bootloader is what a
/// firmware write is addressed to. A bridge routes it by the port index that
/// follows; on a board with no bridge the device half runs it itself.
pub const CMD_REQUEST_BOOTLOADER: u8 = 0x02;
/// The frame body byte of the port-level command that names the device behind
/// a bridge port: `[CMD_SET_PORT_NODEID][index][id][flags]`. The bridge builds
/// its port tables from it and consumes it — no receipt, never forwarded.
pub const CMD_SET_PORT_NODEID: u8 = 0x03;
/// The frame body byte that asks a bridge to pulse the lines of the port that
/// follows — the only reset a device with no admin channel of its own can get.
pub const CMD_RESET_PORT: u8 = 0x04;
/// The frame body byte that asks the device behind a bridge port to restart
/// its firmware: `[CMD_RESTART_DEVICE][index]`, the command a
/// `restart_method: admin` child's `FIRMWARE_RESTART` sends. A bridge routes it
/// by the port index that follows; on a board with no bridge the device half
/// runs it itself.
pub const CMD_RESTART_DEVICE: u8 = 0x05;
/// The frame body byte of the port-level command that toggles the flashing
/// tunnel mode of the port that follows: `[CMD_SET_PORT_TUNNEL][index]`. The
/// firmware spells it `MUX_CMD_SET_PORT_TUNNEL 0x06` with payload length
/// `MUX_SET_PORT_TUNNEL_LEN` (`mux.h`); bridge side (`bridge.c`
/// `bridge_set_port_tunnel`) it is the bridge's own port state — consumed by
/// the bridge, never forwarded downstream, no receipt. The same command both
/// enters and leaves the mode, so a repeat flips back; an index the bridge's
/// port table does not list, or a short payload, is dropped in silence.
pub const CMD_SET_PORT_TUNNEL: u8 = 0x06;
/// The first body byte of an unassigned board's answer.
pub const RESP_NEED_NODEID: u8 = 0x20;
/// The application id CanBoot reports instead of Klipper's own
/// ([`CMD_SET_KLIPPER_NODEID`]), as CAN's `CMD_SET_CANBOOT_NODEID`.
const CMD_SET_CANBOOT_NODEID: u8 = 0x11;

/// A frame in the `KXID` pattern whose payload is [`MAGIC`] followed by
/// `payload`.
///
/// This is the same frame shape the assignment pseudo-frame uses
/// ([`assignment_frame`] is exactly `kxid_unit(&[])`), so the discovery
/// broadcast and the node-id assignment below ride the same self-delimiting
/// frame with one opcode byte — [`CMD_QUERY_UNASSIGNED`] /
/// [`CMD_SET_KLIPPER_NODEID`] — after the marker.
fn kxid_unit(payload: &[u8]) -> Vec<u8> {
    let mut body = MAGIC.to_vec();
    body.extend_from_slice(payload);
    Frame::with_wire_seq(ASSIGN_SEQ_BYTE, body).raw_bytes()
}

/// The broadcast that asks every board without a node id to identify itself:
/// the admin listen id as the unit's header, then the `KXID` frame carrying
/// [`CMD_QUERY_UNASSIGNED`].
///
/// A board that has never been handed a node id is addressed on
/// [`ADMIN_LISTEN`], the same id CAN's broadcast uses; the unit is
/// `[header(0x3f0)][len][0x20]["KXID"][0x00][crc16][0x7e]`.
pub fn discovery_query() -> Vec<u8> {
    admin_unit(&[CMD_QUERY_UNASSIGNED])
}

/// The admin unit that asks the device behind bridge port `index` to park in its
/// bootloader: `[header(0x3f0)][KXID: 0x02, index, uuid?]`.
///
/// The uuid is what makes the request reach a device whose own byte stream is
/// not slices: a CAN port translates a uuid-carrying request into the bus's
/// single admin frame (`0x3f0 [0x02][uuid]`, `generic/canserial.c`), which a
/// device reads whatever carrier it speaks — an ISO-TP application included.
/// Without it the request can only travel as the mux unit itself, which needs a
/// device that reads units. `None` keeps that older, unit-only shape.
pub(crate) fn request_bootloader_unit(index: u8, uuid: Option<[u8; 6]>) -> Vec<u8> {
    match uuid {
        Some(uuid) => {
            let mut body = vec![CMD_REQUEST_BOOTLOADER, index];
            body.extend_from_slice(&uuid);
            admin_unit(&body)
        }
        None => admin_unit(&[CMD_REQUEST_BOOTLOADER, index]),
    }
}

/// The broadcast that tells the board with `uuid` to take `mcu_id`: the admin
/// listen id as the header, then the `KXID` frame carrying
/// [`CMD_SET_KLIPPER_NODEID`], the six UUID bytes most significant first, and
/// the node id — CAN's `[0x01][uuid][nodeid]` inside the stream frame.
///
/// `None` for an id outside `1..=MAX_MCU_ID`, the range the wire format can
/// address — checked exactly the way CAN checks it ([`assign::arb_id`]).
pub fn set_node_id_for(uuid: [u8; 6], mcu_id: u32) -> Option<Vec<u8>> {
    if assign::arb_id(mcu_id).is_none() {
        return None;
    }
    let mut body = Vec::with_capacity(1 + 6 + 1);
    body.push(CMD_SET_KLIPPER_NODEID);
    body.extend_from_slice(&uuid);
    body.push(mcu_id as u8);
    Some(admin_unit(&body))
}

/// The admin-channel unit carrying `payload` after the `KXID` marker: the
/// header on [`ADMIN_LISTEN`], then the [`kxid_unit`] frame. Every command the
/// host addresses to the admin channel — the handshake above and the port-level
/// family below — travels this one shape.
pub(crate) fn admin_unit(payload: &[u8]) -> Vec<u8> {
    let mut unit = encode_header(ADMIN_LISTEN);
    unit.extend_from_slice(&kxid_unit(payload));
    unit
}

/// The port flag of [`CMD_SET_PORT_NODEID`] that says the device behind the
/// port speaks units: the port forwards units both ways unchanged. Clear, the
/// port is legacy — it strips the header off a unit going down and heads a raw
/// frame coming up with the reply id it was given — the wire format of a
/// device that predates the mux.
pub const MUX_PORT_UNITS: u8 = 0x01;

/// The port-level command that names the device behind port `index`: the admin
/// listen unit carrying `[CMD_SET_PORT_NODEID][index][id][flags]`.
///
/// The bridge builds its port tables from it and consumes it — no receipt, no
/// copy downstream, and a repeat simply writes the same values again (`mux.h`
/// in the firmware's serial mux). An index that names no port, or an id the
/// wire cannot address, is dropped in silence there, so this builder validates
/// nothing: it only spells the four payload bytes.
pub fn set_port_node_id(index: u8, mcu_id: u8, flags: u8) -> Vec<u8> {
    admin_unit(&[CMD_SET_PORT_NODEID, index, mcu_id, flags])
}

// ===========================================================================
// The tunnel unit: one katapult message block carried whole inside one mux unit
// ===========================================================================
//
// A flash session talks to a bootloader that speaks katapult's own message
// blocks, so each **whole** block travels wrapped in one mux unit and the far
// end unwraps it. A block is never split across units: splitting would put
// outer framing inside an inner block (and an inner tail or head into the
// next one), which the inner CRC then rejects.
//
// This is the contract both ends share — host `serial.rs` here, firmware
// `mux.c` on the other side (mirrored by T2b). The tag, the word-count
// position and the length rule below must stay byte-for-byte identical on
// both sides; change one, change both.

/// The two bytes that open a katapult message block — `MESSAGE_STX1` and
/// `MESSAGE_STX2` (`third_party/katapult/src/command.h`).
///
/// The signature sits at the **frame position**, the first two bytes after the
/// unit header, and can never collide with a klipper frame: a klipper frame's
/// first byte is its length (`MESSAGE_MIN..=MESSAGE_MAX`, 5..=64) and neither
/// `0x01` nor `0x88` is in that range.
pub const TUNNEL_TAG: [u8; 2] = [0x01, 0x88];

/// The frame position of the word count — katapult's `MESSAGE_POS_LEN`, the
/// byte after the command byte.
pub const TUNNEL_POS_WORDCNT: usize = 3;

/// The bounds on one tunnel **frame**: katapult's `MESSAGE_MIN`/`MESSAGE_MAX`.
///
/// A frame's length is [`tunnel_frame_len`] of its word count, so a legal word
/// count is `0..=30` (`30 * 4 + 8 == 128`); `31` would make 132 bytes, past
/// the limit, and is junk like any other word count that overshoots.
pub const TUNNEL_FRAME_MIN: usize = 8;
pub const TUNNEL_FRAME_MAX: usize = 128;

/// One tunnel frame's length for its word count — katapult's own rule
/// (`command.c`: `buf[MESSAGE_POS_LEN] * 4 + 8`).
///
/// Public because the cut rule is one contract: a consumer that cuts whole
/// katapult blocks out of a bare byte stream (the flashing tool's tunnel
/// adapter) uses this same rule rather than spelling the arithmetic again.
pub const fn tunnel_frame_len(word_cnt: u8) -> usize {
    word_cnt as usize * 4 + 8
}

/// The verdict for the bytes at a unit's front, its header already decoded:
/// the one cut rule [`UnitStream::take_unit_with_arb`] and [`dispatch`] both
/// run, so the two cannot drift apart.
///
/// The tunnel test comes **before** the klipper length range, because a tunnel
/// frame's first byte (`0x01`) is itself an illegal klipper length — today
/// that sends the stream into [`drop_to_sync`], which eats bytes up to the
/// next SYNC and would swallow the rest of the flash stream along with the
/// unit. Two tag bytes are needed to tell the cases apart, so a lone `0x01`
/// waits for the next byte instead of resynchronising, and `0x01` followed by
/// anything but `0x88` takes the old junk path unchanged.
#[derive(Debug, PartialEq, Eq)]
enum UnitCut {
    /// A whole unit: header plus one ordinary klipper frame of `frame` bytes.
    Klipper { frame: usize },
    /// A whole unit: header plus one tunnel frame of `frame` bytes, taken in
    /// one piece by its word count — never split, never partly consumed.
    Tunnel { frame: usize },
    /// Too few bytes are buffered to decide; wait for the next read.
    NeedMore,
    /// These bytes cannot start a unit: resynchronise on the next
    /// `MESSAGE_SYNC`.
    Junk,
}

/// Cut the frame that follows a decoded header into one whole unit — or not.
fn cut_unit(frame: &[u8]) -> UnitCut {
    let Some(&first) = frame.first() else {
        return UnitCut::NeedMore;
    };
    if first == TUNNEL_TAG[0] {
        // The tag is two bytes: until the second arrives, a tunnel frame and
        // junk that merely starts with 0x01 cannot be told apart.
        if frame.len() < TUNNEL_TAG.len() {
            return UnitCut::NeedMore;
        }
        if frame[1] != TUNNEL_TAG[1] {
            return UnitCut::Junk; // 0x01 is no klipper length either
        }
        if frame.len() <= TUNNEL_POS_WORDCNT {
            return UnitCut::NeedMore; // the word count is still in flight
        }
        let length = tunnel_frame_len(frame[TUNNEL_POS_WORDCNT]);
        if !(TUNNEL_FRAME_MIN..=TUNNEL_FRAME_MAX).contains(&length) {
            return UnitCut::Junk;
        }
        if frame.len() < length {
            return UnitCut::NeedMore;
        }
        return UnitCut::Tunnel { frame: length };
    }
    let length = first as usize;
    if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&length) {
        return UnitCut::Junk;
    }
    if frame.len() < length {
        return UnitCut::NeedMore;
    }
    UnitCut::Klipper { frame: length }
}

/// The tunnel unit carrying `katapult_block` under `arb`: the checked two-byte
/// header ([`encode_header`]), then the block exactly as given.
///
/// `katapult_block` must be **one whole, self-delimiting** katapult message
/// block: tagged [`TUNNEL_TAG`], `8..=128` bytes long, and exactly as long as
/// its own word count says ([`tunnel_frame_len`]) — precisely what
/// [`cut_unit`] needs to cut it back out at the far end. The unit layer does
/// **not** check the inner CRC and does **not** check the command byte: those
/// belong to the consumer that reads the block (the flash session), not to the
/// tunnel.
///
/// `None` when `arb` names no address or the block is not one whole message —
/// this builder never panics where [`encode_header`] would.
pub fn tunnel_unit(arb: u32, katapult_block: &[u8]) -> Option<Vec<u8>> {
    if arb >= HEADER_RESERVED_FIRST {
        return None;
    }
    if katapult_block.len() <= TUNNEL_POS_WORDCNT
        || katapult_block[..TUNNEL_TAG.len()] != TUNNEL_TAG
    {
        return None;
    }
    let length = tunnel_frame_len(katapult_block[TUNNEL_POS_WORDCNT]);
    if !(TUNNEL_FRAME_MIN..=TUNNEL_FRAME_MAX).contains(&length) || katapult_block.len() != length {
        return None;
    }
    let mut unit = encode_header(arb);
    unit.extend_from_slice(katapult_block);
    Some(unit)
}

/// The katapult block of `unit` — a whole unit, header included — when it is a
/// tunnel unit, `None` for anything else.
///
/// The cut is [`cut_unit`]'s, so this can never disagree with a reader about
/// what a tunnel unit is. A caller that has one whole unit routed by its
/// arbitration id (the host library's `route_unit`) uses this to tell the
/// tunnel's units from the klipper ones before deciding where to deliver.
pub(crate) fn tunnel_block(unit: &[u8]) -> Option<&[u8]> {
    let consumed = match decode_header(unit) {
        HeaderScan::Complete { consumed, .. } => consumed,
        _ => return None,
    };
    match cut_unit(&unit[consumed..]) {
        UnitCut::Tunnel { frame } if unit.len() == consumed + frame => Some(&unit[consumed..]),
        _ => None,
    }
}

/// The link end of one tunnel listener: the arbitration id whose tunnel units
/// are to be handed over, paired with the sender they go to.
///
/// A link's reader hands each whole katapult block over as one message — the
/// same block [`tunnel_unit`] wrapped, never split — so a consumer never
/// reassembles one itself. The receiving end is the [`mpsc::Receiver`]
/// [`TunnelSink::channel`] returns; dropping it is how a listener leaves, and
/// the link notices on the next delivery (see [`TunnelRegistry::deliver`]).
#[derive(Debug, Clone)]
pub struct TunnelSink {
    frames: mpsc::Sender<Vec<u8>>,
}

impl TunnelSink {
    /// A sink and the receiver it hands whole katapult blocks to.
    pub fn channel() -> (Self, mpsc::Receiver<Vec<u8>>) {
        let (frames, receiver) = mpsc::channel();
        (Self { frames }, receiver)
    }

    /// Hand one whole tunnel block over; `false` when the receiver is gone.
    fn deliver(&self, frame: &[u8]) -> bool {
        self.frames.send(frame.to_vec()).is_ok()
    }
}

/// One link's tunnel listeners, keyed by the arbitration id their units are
/// headed with.
///
/// A link's reader owns one of these (the shared serial port's reader, the host
/// library's reader, the CAN link's reader). Delivering to an id nobody
/// registered answers `false` and the unit is dropped — what the link did
/// before there were listeners; a receiver that has gone away is unregistered
/// the moment a delivery finds it, so no dead listener lingers in the table.
#[derive(Debug, Default)]
pub(crate) struct TunnelRegistry {
    sinks: Mutex<HashMap<u32, TunnelSink>>,
}

impl TunnelRegistry {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    /// Route `arb`'s tunnel units to `sink` from now on; a second registration
    /// under the same id replaces the first.
    pub(crate) fn register(&self, arb: u32, sink: TunnelSink) {
        self.sinks.lock().unwrap().insert(arb, sink);
    }

    /// Stop routing `arb`'s tunnel units; an unregistered id is a no-op.
    pub(crate) fn clear(&self, arb: u32) {
        self.sinks.lock().unwrap().remove(&arb);
    }

    /// Hand one whole tunnel frame to `arb`'s listener, if any.
    ///
    /// `true` when it was delivered. `false` when nobody listens — including a
    /// listener whose receiver has been dropped, which is removed here so a
    /// later delivery does not try it again.
    pub(crate) fn deliver(&self, arb: u32, frame: &[u8]) -> bool {
        let mut sinks = self.sinks.lock().unwrap();
        let delivered = sinks.get(&arb).map(|sink| sink.deliver(frame));
        match delivered {
            Some(true) => true,
            Some(false) => {
                sinks.remove(&arb);
                false
            }
            None => false,
        }
    }
}

/// One listener for whole units headed with one arbitration id: the scan end of
/// an admin channel.
///
/// A bridge's downstream devices answer a discovery broadcast on
/// [`ADMIN_REPLY`], an id no MCU owns, so those units reach no slot and the
/// reader drops them (see [`dispatch`]). A listener registered with
/// [`SharedSerialHandle::set_unit_sink`] gets each one whole — header included,
/// the shape [`UnassignedScan::feed`] takes — on the receiver
/// [`UnitSink::channel`] returns; dropping it is how a listener leaves.
#[derive(Debug, Clone)]
pub struct UnitSink {
    units: mpsc::Sender<Vec<u8>>,
}

impl UnitSink {
    /// A sink and the receiver it hands whole units to.
    pub fn channel() -> (Self, mpsc::Receiver<Vec<u8>>) {
        let (units, receiver) = mpsc::channel();
        (Self { units }, receiver)
    }

    /// Hand one whole unit over; `false` when the receiver is gone.
    fn deliver(&self, unit: &[u8]) -> bool {
        self.units.send(unit.to_vec()).is_ok()
    }
}

/// One link's unit listeners, keyed by the arbitration id their units are
/// headed with — the admin channel's counterpart to [`TunnelRegistry`].
///
/// The difference is what is handed over: a tunnel listener gets the frame's
/// bytes (a whole katapult block), a unit listener the unit as it arrived
/// ([`ADMIN_REPLY`]'s header included), because a scan answer is read as a
/// unit. Delivering to an id nobody registered answers `false`, and the reader
/// drops that unit the way it did before there were listeners; a receiver that
/// has gone away is unregistered on the delivery that finds it.
#[derive(Debug, Default)]
pub(crate) struct UnitRegistry {
    sinks: Mutex<HashMap<u32, UnitSink>>,
}

impl UnitRegistry {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    /// Route `arb`'s whole units to `sink` from now on; a second registration
    /// under the same id replaces the first.
    pub(crate) fn register(&self, arb: u32, sink: UnitSink) {
        self.sinks.lock().unwrap().insert(arb, sink);
    }

    /// Stop routing `arb`'s whole units; an unregistered id is a no-op.
    pub(crate) fn clear(&self, arb: u32) {
        self.sinks.lock().unwrap().remove(&arb);
    }

    /// Whether `arb` has a live listener — what the reader asks before it
    /// accepts a unit no MCU owns.
    pub(crate) fn has(&self, arb: u32) -> bool {
        self.sinks.lock().unwrap().contains_key(&arb)
    }

    /// Hand one whole unit to `arb`'s listener, if any. `false` when nobody
    /// listens — including a listener whose receiver has been dropped, which is
    /// removed here so a later delivery does not try it again.
    pub(crate) fn deliver(&self, arb: u32, unit: &[u8]) -> bool {
        let mut sinks = self.sinks.lock().unwrap();
        match sinks.get(&arb).map(|sink| sink.deliver(unit)) {
            Some(true) => true,
            Some(false) => {
                sinks.remove(&arb);
                false
            }
            None => false,
        }
    }
}

/// One board's answer to [`discovery_query`]: the UUID it is remembered by,
/// the application it runs, and the bridge ports it came through.
///
/// This mirrors CAN's
/// [`UnassignedNode`](crate::core::klippy::interface::devices::canserial::UnassignedNode):
/// the board answers with [`RESP_NEED_NODEID`], the six UUID bytes most
/// significant first, and — in the builds that send them — one more byte naming
/// the application and a port path naming the bridges the answer travelled
/// through. The UUID is read **by position against the frame's length byte**, so
/// a `0x7e` inside it is ordinary content: the protocol allows SYNC in message
/// block content.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UnassignedNode {
    /// The UUID, most significant byte first.
    pub uuid: [u8; 6],
    /// The application byte the board sent, if it sent one.
    pub application: Option<u8>,
    /// The bridge ports the answer travelled through, innermost first; empty
    /// when the board answered directly.
    ///
    /// Each bridge appends the index of the port the answer came up on, so a
    /// board behind one bridge reports one port and a board behind a chain of
    /// bridges reports one per bridge, in the order the answer passed them. A
    /// board wired to the host's own wire — or one whose bridge firmware
    /// predates the path — reports none, which is the same reading.
    pub ports: Vec<u8>,
}

/// The top bit of the byte after an answer's UUID: set, that byte is the head of
/// the answer's port path (its low bits are how many port numbers follow)
/// rather than the application byte, which never has the bit set.
const PATH_FLAG: u8 = 0x80;

/// How many port numbers an answer's path may carry — the depth bound the
/// firmware enforces when a bridge appends its own port (`mux.h`
/// `MUX_PATH_MAX`). A path past it is a field that does not parse, not one to
/// read.
const MAX_PATH_PORTS: usize = 8;

impl UnassignedNode {
    /// The application's name, as Klipper's tool names it in its `Found …`
    /// line.
    ///
    /// A board that sends no application byte — the shape the protocol had
    /// before the byte existed — is named Klipper, which is what upstream's
    /// tool reads it as; an id that means neither Klipper nor CanBoot is
    /// reported as `Unknown` rather than guessed at.
    pub fn application_name(&self) -> &'static str {
        match self.application {
            None | Some(CMD_SET_KLIPPER_NODEID) => "Klipper",
            Some(CMD_SET_CANBOOT_NODEID) => "CanBoot",
            Some(_) => "Unknown",
        }
    }

    /// The board a unit reports, or `None` for a unit that is not an answer.
    ///
    /// What makes a unit an answer is its shape, checked the way
    /// [`parse_assignment_ack`] checks the handshake's: a whole header
    /// naming [`ADMIN_REPLY`], then a frame whose length byte matches the
    /// bytes present, whose sequence byte is [`ASSIGN_SEQ_BYTE`], whose
    /// trailer and CRC check out, and whose content after [`MAGIC`] begins
    /// with [`RESP_NEED_NODEID`] followed by the six UUID bytes and the two
    /// optional fields [`Self::tail`] reads. Nothing else is checked — in
    /// particular a UUID of six zero bytes is reported rather than treated as
    /// unset, because the protocol has no such value.
    pub fn from_unit(unit: &[u8]) -> Option<Self> {
        let (arb, consumed) = match decode_header(unit) {
            HeaderScan::Complete { arb, consumed } => (arb, consumed),
            HeaderScan::NeedMore | HeaderScan::Invalid => return None,
        };
        if arb != ADMIN_REPLY {
            return None;
        }
        let frame = &unit[consumed..];
        if frame.len() < MESSAGE_MIN {
            return None;
        }
        let len = frame[MESSAGE_POS_LEN] as usize;
        if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&len) || frame.len() != len {
            return None;
        }
        if frame[MESSAGE_POS_SEQ] != ASSIGN_SEQ_BYTE {
            return None;
        }
        if frame[len - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC {
            return None;
        }
        let expected = crate::crc16_ccitt!(&frame[..len - MESSAGE_TRAILER_SIZE]);
        let actual = ((frame[len - 3] as u16) << 8) | (frame[len - 2] as u16);
        if expected != actual {
            return None;
        }
        let content = &frame[MESSAGE_HEADER_SIZE..len - MESSAGE_TRAILER_SIZE];
        if content.len() < MAGIC.len() + 1
            || &content[..MAGIC.len()] != &MAGIC[..]
            || content[MAGIC.len()] != RESP_NEED_NODEID
        {
            return None;
        }
        let after_cmd = &content[MAGIC.len() + 1..];
        if after_cmd.len() < 6 {
            return None;
        }
        let mut uuid = [0u8; 6];
        uuid.copy_from_slice(&after_cmd[..6]);
        let (application, ports) = Self::tail(&after_cmd[6..])?;
        Some(Self {
            uuid,
            application,
            ports,
        })
    }

    /// The two optional fields after an answer's UUID — the application byte,
    /// then the port path — read off the rest of the payload; `None` when those
    /// bytes are not fields of an answer at all.
    ///
    /// Both are optional and both are told apart by the top bit of the byte that
    /// starts them ([`PATH_FLAG`]): clear is the application byte, set is the
    /// path's head, whose low bits are how many port numbers follow. So a board
    /// that sends no application byte but travels a bridge (`[0x20][uuid]
    /// [0x81][port]`) reads as Klipper behind that port, and the oldest answer of
    /// all (`[0x20][uuid]`) reads as Klipper directly wired.
    ///
    /// A path that does not parse — a count of zero, past [`MAX_PATH_PORTS`], or
    /// not the number of bytes left in the payload — is dropped on its own rather
    /// than the whole answer with it: the UUID is still a board worth reporting,
    /// and a board whose path is unreadable is reported with none, exactly as one
    /// that answered directly. A byte that is neither a path head nor (any longer)
    /// an application byte is not a field of this shape at all, and the unit is
    /// refused as it was before the path existed.
    fn tail(after_uuid: &[u8]) -> Option<(Option<u8>, Vec<u8>)> {
        let mut rest = after_uuid;
        let mut application = None;
        if let Some((&first, tail)) = rest.split_first() {
            if first & PATH_FLAG == 0 {
                application = Some(first);
                rest = tail;
            }
        }
        let mut ports = Vec::new();
        if let Some((&head, tail)) = rest.split_first() {
            if head & PATH_FLAG != 0 {
                let count = usize::from(head & !PATH_FLAG);
                if !(1..=MAX_PATH_PORTS).contains(&count) || tail.len() != count {
                    return Some((application, ports));
                }
                ports.extend_from_slice(tail);
                rest = &[];
            }
        }
        if !rest.is_empty() {
            return None;
        }
        Some((application, ports))
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

    /// Take one unit: the node it reports, the first time that UUID is heard.
    ///
    /// A board answers with retries, so the same UUID can arrive twice; the
    /// second unit is `None`, and the board is listed once. A unit that is not
    /// an answer — garbage, or a frame addressed to a node that already has an
    /// id — is `None` too.
    pub fn feed(&mut self, unit: &[u8]) -> Option<UnassignedNode> {
        let node = UnassignedNode::from_unit(unit)?;
        if self.nodes.iter().any(|seen| seen.uuid == node.uuid) {
            return None;
        }
        self.nodes.push(node.clone());
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

/// A byte stream being cut into whole units — a two-byte header plus one frame
/// — with the same framing and resync policy as the reader's [`dispatch`].
///
/// [`dispatch`] routes each unit by its arbitration id, which is why it cannot
/// be used here: the units a discovery scan reads carry the admin ids
/// ([`ADMIN_LISTEN`]/[`ADMIN_REPLY`]) that are outside the assigned range, so
/// no id check is applied and the caller decides what each unit is. Everything
/// else is the shared codec ([`decode_header`], [`cut_unit`], [`drop_to_sync`],
/// the `MESSAGE_*` frame constants) — one framing, not a second one.
#[derive(Debug)]
pub(crate) struct UnitStream {
    pending: Vec<u8>,
    needs_sync: bool,
}

impl UnitStream {
    pub(crate) fn new() -> Self {
        Self {
            pending: Vec::new(),
            needs_sync: false,
        }
    }

    /// Append bytes to the stream; nothing is parsed until [`Self::take_unit`].
    pub(crate) fn feed(&mut self, bytes: &[u8]) {
        self.pending.extend_from_slice(bytes);
    }

    /// The next complete unit, or `None` if none is buffered.
    pub(crate) fn take_unit(&mut self) -> Option<Vec<u8>> {
        self.take_unit_with_arb().map(|(_, unit)| unit)
    }

    /// The next complete unit and the arbitration id its header names, or `None`
    /// if none is buffered.
    ///
    /// The id is what lets an addressed reader route the unit to the reference
    /// it belongs to — the host library's reader does exactly that with it
    /// (`host.rs`'s `route_unit`) — while a scan
    /// ([`serial_query_unassigned`]) reads units whole and ignores it.
    ///
    /// Exactly the [`dispatch`] policy: an invalid header, or bytes that
    /// cannot start a unit, resynchronise on the next [`MESSAGE_SYNC`] — and a
    /// tunnel frame (tag [`TUNNEL_TAG`]) is cut whole by its word count,
    /// never eaten by the resync.
    pub(crate) fn take_unit_with_arb(&mut self) -> Option<(u32, Vec<u8>)> {
        loop {
            if self.needs_sync {
                match self.pending.iter().position(|&b| b == MESSAGE_SYNC) {
                    Some(pos) => {
                        self.pending.drain(..=pos);
                        self.needs_sync = false;
                    }
                    None => {
                        self.pending.clear();
                        return None;
                    }
                }
            }
            let (arb, consumed) = match decode_header(&self.pending) {
                HeaderScan::NeedMore => return None,
                HeaderScan::Invalid => {
                    drop_to_sync(&mut self.pending, &mut self.needs_sync);
                    continue;
                }
                HeaderScan::Complete { arb, consumed } => (arb, consumed),
            };
            let frame_len = match cut_unit(&self.pending[consumed..]) {
                UnitCut::NeedMore => return None,
                UnitCut::Junk => {
                    drop_to_sync(&mut self.pending, &mut self.needs_sync);
                    continue;
                }
                UnitCut::Klipper { frame } | UnitCut::Tunnel { frame } => frame,
            };
            // `cut_unit` only answers with a whole frame, so the unit is here.
            let unit_len = consumed + frame_len;
            let unit: Vec<u8> = self.pending.drain(..unit_len).collect();
            return Some((arb, unit));
        }
    }
}

/// Ask every board with no node id on `port` to identify itself, and read the
/// answers for `timeout`.
///
/// Writes one [`discovery_query`], then cuts the incoming stream into units
/// with the same framing as the shared reader ([`UnitStream`]) and feeds each
/// to a scan; `report` is called as each board is first heard, so a front-end
/// can print answers while the window is still open.
///
/// The `timeout` is a **deadline**, measured from the moment the query goes
/// out, like CAN's scan window: every read is bounded by what is left of it,
/// and the scan ends when that runs out, whatever has arrived by then. A
/// blocking port says “no bytes in this read window” with a zero read, a
/// non-blocking one with `WouldBlock`/`TimedOut`; both just mean the loop has
/// to check the deadline, and any other read error is returned as it is.
/// Returns how many boards were found.
pub fn serial_query_unassigned(
    port: &mut (impl std::io::Read + std::io::Write),
    timeout: Duration,
    mut report: impl FnMut(UnassignedNode),
) -> std::io::Result<usize> {
    port.write_all(&discovery_query())?;
    let deadline = Instant::now() + timeout;
    let mut scan = UnassignedScan::new();
    let mut stream = UnitStream::new();
    let mut buf = [0u8; READ_CHUNK];
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            break;
        }
        match port.read(&mut buf) {
            Ok(0) => continue,
            Ok(read) => {
                stream.feed(&buf[..read]);
                while let Some(unit) = stream.take_unit() {
                    if let Some(node) = scan.feed(&unit) {
                        report(node);
                    }
                }
            }
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                continue;
            }
            Err(e) => return Err(e),
        }
    }
    Ok(scan.total())
}

/// A physical serial port, its one reader thread, and a slot per attached MCU.
pub struct SharedSerialPort {
    path: PathBuf,
    baud: u32,
    /// The opened tty. Shared with the reader thread so it can read without
    /// holding an `Arc<SharedSerialPort>` (which would keep the port from ever
    /// being dropped — see the reader).
    port: Arc<File>,
    /// One slot per attached MCU, keyed by the MCU id. A slot is registered by
    /// the attach that opens the id, and taken down again by
    /// [`SharedSerialHandle::unregister`] — a `shutdown`, the handle's last
    /// reference going away, or a failed attach — so a stopped id's frames are
    /// dropped instead of piling up in a slot nobody reads.
    slots: Mutex<HashMap<u32, Arc<McuSlot>>>,
    /// Serializes the handshakes on this port: two MCUs attaching together must
    /// not interleave their requests and answers.
    attach_lock: Mutex<()>,
    /// Serializes whole units on the write side (`send` and the handshake
    /// request): a unit is a header plus a frame, and the two must not be split
    /// by another writer.
    write_lock: Mutex<()>,
    /// The tunnel listeners on this port, keyed by the arbitration id their
    /// units are headed with — the uplink half of a flashing tunnel
    /// ([`SharedSerialHandle::set_tunnel_sink`]).
    tunnels: TunnelRegistry,
    /// The listeners for whole units under an id no MCU owns — a bridge's
    /// admin answers, which a scan reads
    /// ([`SharedSerialHandle::set_unit_sink`]).
    units: UnitRegistry,
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
    /// device's answer to [`assignment_reply`] is recognized.
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
/// stays up until the last handle is gone. The slot this handle registered goes
/// away with it, so the port stops feeding an id nobody reads.
pub struct SharedSerialHandle {
    port: Arc<SharedSerialPort>,
    mcu_id: u32,
    slot: Arc<McuSlot>,
    stopped: AtomicBool,
}

/// The process-wide registry that makes a serial port a per-path singleton:
/// attaches on the same path share one port (and thus one reader).
fn port_registry() -> &'static Mutex<HashMap<PathBuf, Weak<SharedSerialPort>>> {
    static PORTS: OnceLock<Mutex<HashMap<PathBuf, Weak<SharedSerialPort>>>> = OnceLock::new();
    PORTS.get_or_init(|| Mutex::new(HashMap::new()))
}

/// The live port for `path`, opening it (and starting its reader) if the
/// registry holds no live one.
fn port_for(path: &Path, baud: u32) -> Result<Arc<SharedSerialPort>, InterfaceError> {
    resource_for(port_registry(), path, |path| {
        SharedSerialPort::open(path, baud)
    })
}

/// Open a port for one MCU and complete the frame-level id handshake.
///
/// The first attach on a path opens the tty and starts the reader; later ones
/// reuse both and only register a slot. Once the slot is registered this is
/// [`SharedSerialHandle::handshake`] and nothing else: when `uuid` is `Some`,
/// the SET_NODEID unit naming the board ([`set_node_id_for`]) goes out first
/// and the assignment request follows — mirroring CAN's `CanSerialDevice::open`,
/// which sends its SET_NODEID when the link opens. `None` skips that and goes
/// straight to the request. On failure the slot this opened is unregistered, so
/// the port is left as it was found.
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
    uuid: Option<[u8; 6]>,
) -> Result<Arc<SharedSerialHandle>, InterfaceError> {
    if assign::arb_id(mcu_id).is_none() {
        return Err(InterfaceError::Other(format!(
            "mcu id {mcu_id} is outside 1..={MAX_MCU_ID}"
        )));
    }

    let port = port_for(path, baud)?;
    let slot = Arc::new(McuSlot::new());
    port.slots.lock().unwrap().insert(mcu_id, Arc::clone(&slot));
    let handle = Arc::new(SharedSerialHandle {
        port: Arc::clone(&port),
        mcu_id,
        slot,
        stopped: AtomicBool::new(false),
    });

    // The handshake is the one `handshake` runs; `attach` only registers the
    // slot in front of it. A failure takes the slot down again — a port that
    // never answered is left as `attach` found it.
    let handshake = match uuid {
        Some(uuid) => handle.handshake(uuid, timeout),
        None => handle.run_handshake(None, timeout),
    };
    if let Err(e) = handshake {
        // The slot this attach opened goes down through the same guarded
        // unregister the teardown paths use; the `ptr_eq` check only removes
        // the slot this handle registered.
        handle.unregister();
        return Err(e);
    }
    Ok(handle)
}

/// Register a **downstream** device on the shared serial port at `path`,
/// **without a handshake**.
///
/// The port is the same per-path singleton [`attach`] uses, and the device gets
/// its own slot (the registration [`attach`] makes before it handshakes), so
/// units headed with `mcu_id`'s listener or answer id reach it. Nothing is
/// written and no answer is awaited, though: a device behind a bridge port has
/// no admin channel of its own, so there is no assignment to run. The route to
/// it is the bridge's `SET_PORT_NODEID` ([`set_port_node_id`]), and **the
/// caller sends that first** — with [`SharedSerialHandle::send_set_port_node_id`]
/// on the bridge's own handle — because only the caller knows which port the
/// device sits behind. Until it does, `mcu_id` names no port and its units go
/// nowhere; a missing route shows up as a silent port, never as an error here.
///
/// # Errors
/// [`InterfaceError::Other`] for an id outside `1..=MAX_MCU_ID` — the same
/// rejection [`attach`] makes — and for a port that cannot be opened;
/// [`InterfaceError::ConnectionLost`] when the port is already stopped.
pub fn attach_downstream(
    path: impl AsRef<Path>,
    baud: u32,
    mcu_id: u32,
) -> Result<Arc<SharedSerialHandle>, InterfaceError> {
    if assign::arb_id(mcu_id).is_none() {
        return Err(InterfaceError::Other(format!(
            "mcu id {mcu_id} is outside 1..={MAX_MCU_ID}"
        )));
    }

    let port = port_for(path.as_ref(), baud)?;
    if port.stopped.load(Ordering::SeqCst) {
        return Err(InterfaceError::ConnectionLost);
    }

    let slot = Arc::new(McuSlot::new());
    port.slots.lock().unwrap().insert(mcu_id, Arc::clone(&slot));
    Ok(Arc::new(SharedSerialHandle {
        port,
        mcu_id,
        slot,
        stopped: AtomicBool::new(false),
    }))
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
            tunnels: TunnelRegistry::new(),
            units: UnitRegistry::new(),
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
    /// Stop the port, then take its reader down with it.
    ///
    /// Which thread this runs on is decided by whoever releases the last
    /// `Arc<SharedSerialPort>` — and that can be the **reader itself**: the
    /// reader holds a strong reference from its `weak.upgrade()` until it drops
    /// the port again after `dispatch`, so the last handle dropped inside that
    /// window leaves the reader's own `drop(port)` as the release that reaches
    /// zero. Joining a thread from within that thread never returns
    /// (`pthread_join` reports `EDEADLK`, and std turns that into a panic that
    /// would take the reader down mid-teardown), so the join is skipped when
    /// the handle is this thread's. The reader is not left alive by that: the
    /// loop checks the port again after every read, and there it finds the
    /// `Weak` dead — the same path that ends the reader when the last handle
    /// goes for any other reason.
    fn drop(&mut self) {
        self.stop();
        // The reader may still be inside a `read` (bounded by `VTIME`); joining
        // it here is what lets the port and its descriptor go away. It does not
        // need the mutex to finish — it sees `stopped` and returns.
        if let Ok(mut guard) = self.reader.lock() {
            if let Some(handle) = guard.take() {
                // Self-join guard: on the reader thread there is no other
                // thread to wait for — the drop itself is that thread leaving —
                // and the detached handle still joins nothing.
                if handle.thread().id() != std::thread::current().id() {
                    let _ = handle.join();
                }
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
                // the last handle could never bring the port down. This drop
                // can be the one that destroys the port — see `Drop for
                // SharedSerialPort` for why running that on this thread is
                // safe.
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
/// A unit is a header ([`decode_header`]) followed by one frame: at the tunnel
/// tag ([`TUNNEL_TAG`]) a whole katapult block cut by its word count
/// ([`cut_unit`]), otherwise an ordinary frame cut by its length byte.
/// Anything that cannot be one — an invalid header, an arbitration id outside
/// the assigned range, or a frame the cut rule rejects — resynchronises on the
/// next SYNC, the same policy as [`FrameStream`]. Too few bytes for a unit
/// just waits for the next read.
///
/// A tunnel unit is consumed as one piece and dropped here (with a debug
/// line): there is no tunnel listener yet. It must never reach
/// [`drop_to_sync`], which would eat through the stream instead of taking one
/// unit.
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

        let (arb, consumed) = match decode_header(pending) {
            HeaderScan::NeedMore => return,
            HeaderScan::Invalid => {
                drop_to_sync(pending, needs_sync);
                continue;
            }
            HeaderScan::Complete { arb, consumed } => (arb, consumed),
        };

        // Not an arbitration id this port routes (neither an MCU's listener id
        // nor an answer id): no unit can start here — unless a scan listener is
        // registered for it. A bridge's downstream devices answer on
        // `ADMIN_REPLY`, which no MCU owns, and a scan is exactly the listener
        // for those.
        let routed = assign::mcu_id_from_arb(arb).is_some();
        if !routed && !port.units.has(arb) {
            drop_to_sync(pending, needs_sync);
            continue;
        }

        let frame_len = match cut_unit(&pending[consumed..]) {
            UnitCut::NeedMore => return,
            UnitCut::Junk => {
                drop_to_sync(pending, needs_sync);
                continue;
            }
            UnitCut::Klipper { frame } => frame,
            UnitCut::Tunnel { frame } => {
                // One whole tunnel unit, consumed as one piece: the flashing
                // tunnel's uplink. A listener registered for this unit's
                // arbitration id gets the block whole; with none, the unit is
                // dropped here (never through `drop_to_sync`, which would eat
                // the stream around it).
                let block = &pending[consumed..consumed + frame];
                if !port.tunnels.deliver(arb, block) {
                    debug!(
                        "shared serial {}: no tunnel listener; dropping {}-byte tunnel unit for arb {arb:#x}",
                        port.path.display(),
                        HEADER_BYTES + frame,
                    );
                }
                pending.drain(..consumed + frame);
                continue;
            }
        };

        let unit_len = consumed + frame_len;
        let unit = &pending[..unit_len];
        if !routed {
            // A whole unit under an id no MCU owns: a scan answer, handed over
            // as it arrived. A listener that left between the check above and
            // here is a unit nobody wants, so the stream resynchronises as it
            // did before there were listeners.
            if !port.units.deliver(arb, unit) {
                drop_to_sync(pending, needs_sync);
                continue;
            }
            pending.drain(..unit_len);
            continue;
        }
        if let Some(mcu_id) = parse_assignment_ack(unit) {
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

impl SharedSerialHandle {
    /// Write one unit on this port under an arbitrary arbitration id: the
    /// checked two-byte header for `arb` ([`encode_header`]), then `payload`
    /// exactly as given.
    ///
    /// The whole unit goes out under the port's write lock, so no other writer
    /// can split its header from its frame. This is the entry the admin command
    /// below is built on; an `arb` in the header's reserved band has no address
    /// and panics in [`encode_header`].
    ///
    /// # Errors
    /// [`InterfaceError::ConnectionLost`] when this handle or the port has been
    /// shut down, [`InterfaceError::SendError`] when the write fails.
    pub(crate) fn send_unit(&self, arb: u32, payload: &[u8]) -> Result<(), InterfaceError> {
        if self.stopped.load(Ordering::SeqCst) || self.port.stopped.load(Ordering::SeqCst) {
            return Err(InterfaceError::ConnectionLost);
        }
        let mut unit = encode_header(arb);
        unit.extend_from_slice(payload);
        self.port.write_unit(&unit)
    }

    /// Send one **tunnel** unit on this port: a whole katapult message block
    /// under this handle's host→device arbitration id ([`assign::arb_id`]),
    /// wrapped by [`tunnel_unit`] — one unit, the block unsplit.
    ///
    /// The same write path as [`Self::send_unit`]: the header comes from
    /// [`encode_header`], the whole unit goes out under the port's write lock
    /// so no other writer can split it. What is checked here is the block's
    /// **shape** only (tag, word-count length, `8..=128`); the inner CRC and
    /// command byte are the consuming session's to check, not this writer's.
    ///
    /// # Errors
    /// [`InterfaceError::ConnectionLost`] when this handle or the port has been
    /// shut down, [`InterfaceError::SendError`] when the write fails or `block`
    /// is not one self-delimiting katapult message block.
    pub fn send_tunnel(&self, block: &[u8]) -> Result<(), InterfaceError> {
        if self.stopped.load(Ordering::SeqCst) || self.port.stopped.load(Ordering::SeqCst) {
            return Err(InterfaceError::ConnectionLost);
        }
        let arb = assign::arb_id(self.mcu_id).expect("an attached mcu id is always addressable");
        let unit = tunnel_unit(arb, block).ok_or_else(|| {
            InterfaceError::SendError(
                "tunnel block is not one self-delimiting katapult message block".to_string(),
            )
        })?;
        self.port.write_unit(&unit)
    }

    /// Route this port's tunnel units headed with `arb` to `sink`: the uplink
    /// half of a flashing tunnel, handed over whole.
    ///
    /// `arb` is the id the upstream tunnel units carry — for a device behind a
    /// bridge port that is `reply_arb_id(mcu_id)` of the device's own id, which
    /// is what the bridge heads them with. Registering the same id twice
    /// replaces the first sink; [`Self::clear_tunnel_sink`] removes it, and so
    /// does dropping the receiver (see [`TunnelSink::channel`]). Units no sink
    /// is registered for are dropped, exactly as before there were listeners.
    pub fn set_tunnel_sink(&self, arb: u32, sink: TunnelSink) {
        self.port.tunnels.register(arb, sink);
    }

    /// Stop routing `arb`'s tunnel units to the sink registered for it.
    pub fn clear_tunnel_sink(&self, arb: u32) {
        self.port.tunnels.clear(arb);
    }

    /// Route this port's **whole units** headed with `arb` to `sink`: how a
    /// scan reads a bridge's admin answers.
    ///
    /// `arb` is [`ADMIN_REPLY`] — the id every device behind the bridge answers
    /// a discovery broadcast on, which no MCU owns, so without a listener the
    /// reader drops those units. The sink gets each unit as it arrived (header
    /// included: [`UnassignedScan::feed`]'s shape). Registering the same id
    /// twice replaces the first sink; [`Self::clear_unit_sink`] removes it, and
    /// so does dropping the receiver (see [`UnitSink::channel`]).
    pub(crate) fn set_unit_sink(&self, arb: u32, sink: UnitSink) {
        self.port.units.register(arb, sink);
    }

    /// Stop routing `arb`'s whole units to the sink registered for it.
    pub(crate) fn clear_unit_sink(&self, arb: u32) {
        self.port.units.clear(arb);
    }

    /// Send one admin-channel unit to the bridge on this port: the
    /// [`ADMIN_LISTEN`] header, then `payload` — one `KXID` frame — exactly as
    /// given.
    ///
    /// The admin channel belongs to the bridge: this is how a device that has
    /// no admin channel of its own is named behind a port ([`set_port_node_id`]
    /// and the reset family travel this way). The unit names this handle's own
    /// `mcu_id` nowhere — the header is the fixed admin id, not the device's.
    ///
    /// # Errors
    /// The same as [`SharedSerialHandle::send_unit`].
    pub(crate) fn send_admin(&self, payload: &[u8]) -> Result<(), InterfaceError> {
        self.send_unit(ADMIN_LISTEN, payload)
    }

    /// Tell the bridge which id the device behind port `index` answers to:
    /// [`set_port_node_id`]'s unit, byte for byte, on the admin channel.
    ///
    /// The bridge builds its port table from it and consumes it — no receipt,
    /// nothing sent downstream, idempotent (`mux.h` in the firmware's serial
    /// mux) — so this is how a downstream device's route is established (see
    /// [`attach_downstream`]). The caller decides **when**; this only spells the
    /// command and writes it.
    ///
    /// # Errors
    /// The same as [`SharedSerialHandle::send_unit`].
    pub fn send_set_port_node_id(
        &self,
        index: u8,
        mcu_id: u8,
        flags: u8,
    ) -> Result<(), InterfaceError> {
        // The builder spells the whole admin unit (header + frame); `send_admin`
        // writes the header, so only the frame behind it goes to it.
        self.send_admin(&set_port_node_id(index, mcu_id, flags)[HEADER_BYTES..])
    }

    /// Ask the device behind port `index` to restart its firmware:
    /// [`CMD_RESTART_DEVICE`]'s admin unit, which the bridge forwards to that
    /// port and the device acts on.
    ///
    /// The restart a **named** bridge-port device gets — the one that answers the
    /// mux itself. No receipt: the bridge routes the command and the device
    /// either restarts or does not, so the caller confirms it by whether the
    /// device can be reached again (`mcu/restart.rs`).
    ///
    /// # Errors
    /// The same as [`SharedSerialHandle::send_unit`].
    pub fn send_restart_device(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_admin(&admin_unit(&[CMD_RESTART_DEVICE, index])[HEADER_BYTES..])
    }

    /// Ask the bridge to pulse the lines of port `index` — [`CMD_RESET_PORT`]'s
    /// admin unit. The bridge performs it; nothing is sent downstream.
    ///
    /// The reset a **legacy** bridge-port device gets: it has no admin channel of
    /// its own, so its bridge's port is the only thing that can reset it. No
    /// receipt, like [`SharedSerialHandle::send_restart_device`].
    ///
    /// # Errors
    /// The same as [`SharedSerialHandle::send_unit`].
    pub fn send_reset_port(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_admin(&admin_unit(&[CMD_RESET_PORT, index])[HEADER_BYTES..])
    }

    /// Ask the device behind port `index` to enter its bootloader:
    /// [`CMD_REQUEST_BOOTLOADER`]'s admin unit, the command a **flashing tool**
    /// addresses a board with. The bridge routes it to that port, so a device
    /// behind a bridge is reached the same way one on a plain port is.
    ///
    /// Nothing in the restart path sends this: a restart reloads a firmware
    /// that is running, while this command is for the case where it is about to
    /// be replaced.
    ///
    /// # Errors
    /// The same as [`SharedSerialHandle::send_unit`].
    pub fn send_request_bootloader(
        &self,
        index: u8,
        uuid: Option<[u8; 6]>,
    ) -> Result<(), InterfaceError> {
        self.send_admin(&request_bootloader_unit(index, uuid)[HEADER_BYTES..])
    }

    /// Toggle the flashing tunnel of bridge port `index` —
    /// [`CMD_SET_PORT_TUNNEL`]'s admin unit. The bridge flips its own port
    /// state; nothing is sent downstream.
    ///
    /// What a **flashing tool** turns on for the port a device in its
    /// bootloader sits behind: in tunnel mode the port carries a katapult raw
    /// byte stream rather than units. The same command leaves the mode — a
    /// repeat flips back, so the caller tracks the state it asked for. No
    /// receipt, like [`SharedSerialHandle::send_restart_device`].
    ///
    /// # Errors
    /// The same as [`SharedSerialHandle::send_unit`].
    pub fn send_set_port_tunnel(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_admin(&admin_unit(&[CMD_SET_PORT_TUNNEL, index])[HEADER_BYTES..])
    }

    /// Complete the frame-level id handshake on this already-open handle: name
    /// this device `uuid` ([`set_node_id_for`]) and have it re-confirm the
    /// assignment — exactly the segment [`attach`] runs once its slot is
    /// registered.
    ///
    /// This is the handshake a **downstream** device runs once its route
    /// exists. [`attach_downstream`] opens the handle without one, so the
    /// caller can first tell the bridge which port the device sits behind
    /// ([`SharedSerialHandle::send_set_port_node_id`] over the bridge's own
    /// handle) and only then name it — **the caller owns that order**: until
    /// the bridge knows the route, the assignment request reaches no device and
    /// the handshake times out like any other unanswered one.
    ///
    /// The handshake runs under the port's attach lock (the one [`attach`] takes),
    /// so two handshakes on one port cannot interleave.
    ///
    /// # Errors
    /// [`InterfaceError::ConnectionLost`] when the port has been shut down,
    /// [`InterfaceError::SendError`] when a write fails, and
    /// [`InterfaceError::Timeout`] when the device never answers. A failure
    /// leaves the handle live: its slot stays registered and the port keeps
    /// round-tripping frames — unless this very handle has already been shut
    /// down, which unregisters that slot.
    pub fn handshake(&self, uuid: [u8; 6], timeout: Duration) -> Result<(), InterfaceError> {
        self.run_handshake(Some(uuid), timeout)
    }

    /// The one implementation of the frame-level id handshake, shared by
    /// [`attach`] and [`SharedSerialHandle::handshake`]: the SET_NODEID unit
    /// first when a `uuid` names the board, then the assignment request retried
    /// [`ASSIGN_ATTACH_ATTEMPTS`] times for `timeout` each. Unlike [`attach`],
    /// a failure does not unregister the slot — the handle was the caller's
    /// before this ran.
    fn run_handshake(
        &self,
        uuid: Option<[u8; 6]>,
        timeout: Duration,
    ) -> Result<(), InterfaceError> {
        // The attach lock serializes the handshakes on this port; `attach`
        // takes it for the same reason (never both, so no re-entry).
        let _attaching = self.port.attach_lock.lock().unwrap();
        if self.port.stopped.load(Ordering::SeqCst) {
            return Err(InterfaceError::ConnectionLost);
        }

        // A discovered MCU is named before the assignment runs: frame order is
        // the SET_NODEID unit first, the assignment request second (mirrors
        // CAN's `CanSerialDevice::open`). `set_node_id_for` fails only for an id
        // the wire format cannot carry, which the handle was opened with.
        if let Some(uuid) = uuid {
            let unit = set_node_id_for(uuid, self.mcu_id)
                .expect("an attached mcu id is always addressable");
            self.port.write_unit(&unit)?;
        }

        let request =
            assignment_request(self.mcu_id).expect("an attached mcu id is always addressable");
        let mut attempts = 0;
        loop {
            *self.slot.ack.lock().unwrap() = None;
            self.port.write_unit(&request)?;
            if wait_for_ack(&self.slot, timeout) == Some(self.mcu_id) {
                return Ok(());
            }
            if self.port.stopped.load(Ordering::SeqCst) {
                return Err(InterfaceError::ConnectionLost);
            }
            attempts += 1;
            if attempts >= ASSIGN_ATTACH_ATTEMPTS {
                break;
            }
        }
        Err(InterfaceError::Timeout)
    }

    /// Take this handle's slot out of the port's map — only when the entry
    /// there is still the slot **this** handle registered.
    ///
    /// The one place a slot is unregistered: [`Drop`] (whichever thread drops
    /// the handle's last reference), [`Device::shutdown`] (the thread stopping
    /// this handle) and [`attach`]'s failure path (the attaching thread) all
    /// come here, each before the handle's last reference goes. The `ptr_eq`
    /// check keeps a handle from taking down a slot a later attach on the same
    /// id registered: two attaches on one id are not supported, but the
    /// failure path runs after a handshake another thread can race past, so the
    /// guard has to be the same everywhere.
    ///
    /// Once the entry is gone, `push_frame` finds no slot for this id and drops
    /// its frames — the "a failed `Sender` drops the frame" behaviour of the
    /// host side, and what keeps a shut-down handle's stream from growing
    /// unbounded (`FrameStream` is an unbounded `Vec`).
    fn unregister(&self) {
        let Ok(mut slots) = self.port.slots.lock() else {
            return; // a poisoned map is not repaired from a teardown
        };
        if slots
            .get(&self.mcu_id)
            .is_some_and(|slot| Arc::ptr_eq(slot, &self.slot))
        {
            slots.remove(&self.mcu_id);
        }
    }
}

impl Drop for SharedSerialHandle {
    /// Unregister this handle's slot, so the port stops feeding it.
    ///
    /// A live handle is the only reader of its slot's `incoming`; a slot left
    /// in the map after its handle is gone only accumulates bytes nobody will
    /// read (`push_frame` cannot tell an abandoned slot from a live one), for as
    /// long as any **other** handle keeps the port up. This runs on whichever
    /// thread drops the handle's last reference; [`SharedSerialHandle::unregister`]
    /// holds the map lock and the `ptr_eq` guard, so it cannot race `attach`'s
    /// insert nor take down a slot a later attach registered. `shutdown` and
    /// `attach`'s failure path have already unregistered by the time this runs
    /// — the unregister then finds no entry to take down and leaves the port as
    /// it found it.
    fn drop(&mut self) {
        self.unregister();
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
        let mut unit = encode_header(arb);
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
        // A stopped handle never reads its slot again (`receive` returns `None`),
        // so the slot goes with it: otherwise `push_frame` would keep pushing
        // this id's frames into a stream nobody drains, for as long as the port
        // lives (`FrameStream` is an unbounded `Vec`). Frames for this id are
        // dropped from here on, like a failed `Sender` on the host side.
        self.unregister();
        let _guard = self.slot.incoming.lock().unwrap();
        self.slot.cv.notify_all();
    }
}

#[cfg(test)]
mod mux_tests {
    use super::*;
    use crate::core::klippy::frame::{MESSAGE_DEST, MESSAGE_SEQ_MASK};
    use crate::core::klippy::interface::devices::serial::{SerialDevice, DEFAULT_BAUD};
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

    /// Read exactly `want` bytes from the pty master within `window`; panics
    /// when the window expires first, so a handshake that writes nothing fails
    /// the test instead of leaving it blocked on a silent port.
    fn read_n_within(master: &File, want: usize, window: Duration) -> Vec<u8> {
        let deadline = Instant::now() + window;
        let mut master = master;
        let mut out = Vec::with_capacity(want);
        let mut buf = [0u8; 64];
        while out.len() < want {
            let now = Instant::now();
            assert!(
                now < deadline,
                "only {} of {want} bytes arrived within {window:?}",
                out.len()
            );
            let mut poll = libc::pollfd {
                fd: std::os::fd::AsRawFd::as_raw_fd(master),
                events: libc::POLLIN,
                revents: 0,
            };
            let rc = unsafe {
                libc::poll(
                    &mut poll,
                    1,
                    deadline.saturating_duration_since(now).as_millis() as i32,
                )
            };
            assert!(
                rc >= 0,
                "polling the pty master: {}",
                std::io::Error::last_os_error()
            );
            let read = master.read(&mut buf).expect("pty master read");
            assert_ne!(read, 0, "the pty closed before {want} bytes arrived");
            out.extend_from_slice(&buf[..read]);
        }
        out
    }

    /// Whether the pty master has a byte waiting within `window` — how a test
    /// proves a step wrote **nothing** without blocking on a silent port.
    fn readable_within(master: &File, window: Duration) -> bool {
        let mut poll = libc::pollfd {
            fd: std::os::fd::AsRawFd::as_raw_fd(master),
            events: libc::POLLIN,
            revents: 0,
        };
        let rc = unsafe { libc::poll(&mut poll, 1, window.as_millis() as i32) };
        assert!(
            rc >= 0,
            "polling the pty master: {}",
            std::io::Error::last_os_error()
        );
        rc > 0
    }

    /// Attach `mcu_id`, playing the device side of the handshake from the test.
    fn attach_with_reply(pty: &Pty, mcu_id: u32) -> Arc<SharedSerialHandle> {
        let request = assignment_request(mcu_id).expect("a valid mcu id");
        let path = pty.path.clone();
        let thread =
            thread::spawn(move || attach(&path, DEFAULT_BAUD, mcu_id, ASSIGN_ATTACH_TIMEOUT, None));
        let seen = read_n(&pty.master, request.len());
        assert_eq!(
            seen, request,
            "attach must send the assignment request verbatim"
        );
        master_write(
            &pty.master,
            &assignment_reply(mcu_id).expect("a valid mcu id"),
        );
        thread
            .join()
            .expect("attach thread")
            .expect("attach succeeds")
    }

    /// One device→host unit: the answer-direction header, then the frame bytes.
    fn device_unit(mcu_id: u32, frame: &[u8]) -> Vec<u8> {
        let mut unit = encode_header(assign::reply_arb_id(mcu_id).expect("a valid mcu id"));
        unit.extend_from_slice(frame);
        unit
    }

    /// Split a host→device byte run into `(arb, frame)` units, asserting each is
    /// whole (a valid header, then one parseable frame).
    fn split_units(mut wire: &[u8]) -> Vec<(u32, Frame)> {
        let mut units = Vec::new();
        while !wire.is_empty() {
            let (arb, consumed) = match decode_header(wire) {
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

    /// With a uuid the handshake starts by naming the board: the SET_NODEID
    /// unit goes out first, then the ordinary assignment request, then the
    /// reply lets `attach` return a handle (mirrors CAN's SET_NODEID-on-open).
    #[test]
    fn attach_with_a_uuid_sends_set_node_id_before_the_assignment_request() {
        let pty = Pty::new();
        let uuid = [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc];
        let mcu_id = 6;

        let path = pty.path.clone();
        let thread = thread::spawn(move || {
            attach(
                &path,
                DEFAULT_BAUD,
                mcu_id,
                ASSIGN_ATTACH_TIMEOUT,
                Some(uuid),
            )
        });

        let set = set_node_id_for(uuid, mcu_id).expect("a valid mcu id");
        let request = assignment_request(mcu_id).expect("a valid mcu id");
        let wire = read_n(&pty.master, set.len() + request.len());
        assert_eq!(
            &wire[..set.len()],
            set.as_slice(),
            "the SET_NODEID unit goes out first"
        );
        assert_eq!(
            &wire[set.len()..],
            request.as_slice(),
            "the assignment request follows SET_NODEID"
        );

        master_write(
            &pty.master,
            &assignment_reply(mcu_id).expect("a valid mcu id"),
        );
        let handle = thread
            .join()
            .expect("attach thread")
            .expect("attach succeeds");
        // The returned handle is the caller's to use, like any other attach.
        assert!(handle.send(&Frame::new(0, vec![0xab])).is_ok());
    }

    #[test]
    fn attach_times_out_when_no_reply_arrives() {
        let pty = Pty::new();
        let err = attach(&pty.path, DEFAULT_BAUD, 4, Duration::from_millis(50), None).unwrap_err();
        assert_eq!(err, InterfaceError::Timeout);

        // Every attempt went out, byte for byte.
        let request = assignment_request(4).unwrap();
        let all = read_n(&pty.master, request.len() * ASSIGN_ATTACH_ATTEMPTS as usize);
        for chunk in all.chunks(request.len()) {
            assert_eq!(chunk, request.as_slice(), "one retry");
        }

        // The failed slot was unregistered and the port is still usable: a fresh
        // attach on the same path completes a handshake.
        let handle = attach_with_reply(&pty, 4);
        assert!(handle.send(&Frame::new(1, vec![0xab])).is_ok());
    }

    /// A downstream device attaches with **no handshake at all**: the peer never
    /// answers — a handshake on the same port times out — yet the downstream
    /// attach hands back a live handle and writes nothing. From then on the
    /// handle is an ordinary mux slot: a frame sent goes out headed with its
    /// id's listener header, a unit headed with either the listener or the reply
    /// id comes back as the bare frame, and an admin unit goes out to the bridge
    /// byte for byte.
    #[test]
    fn attach_downstream_does_not_handshake_and_round_trips_frames() {
        let pty = Pty::new();
        let mcu_id = 4;

        // An id the header cannot carry is refused the way `attach` refuses it.
        let err = attach_downstream(&pty.path, DEFAULT_BAUD, 0).unwrap_err();
        assert!(
            matches!(err, InterfaceError::Other(_)),
            "id 0 is outside the addressable range, got {err:?}"
        );

        // A port whose peer never answers: the handshake attach cannot succeed.
        let err = attach(
            &pty.path,
            DEFAULT_BAUD,
            mcu_id,
            Duration::from_millis(50),
            None,
        )
        .unwrap_err();
        assert_eq!(
            err,
            InterfaceError::Timeout,
            "the peer never answers the assignment"
        );
        // Drain the attempts the failed handshake wrote, so the byte assertions
        // below start from an empty port.
        let request = assignment_request(mcu_id).expect("a valid mcu id");
        let attempts = read_n(&pty.master, request.len() * ASSIGN_ATTACH_ATTEMPTS as usize);
        for chunk in attempts.chunks(request.len()) {
            assert_eq!(chunk, request.as_slice(), "one retry");
        }

        // The downstream attach succeeds where the handshake could not ...
        let handle = attach_downstream(&pty.path, DEFAULT_BAUD, mcu_id)
            .expect("a downstream attach does not handshake");
        // ... and it wrote nothing: there is no admin channel on this side, so
        // there is nothing to ask and no answer to wait for.
        assert!(
            !readable_within(&pty.master, Duration::from_millis(100)),
            "attach_downstream must write nothing — it has no handshake to run"
        );

        // Host → port: the frame is prefixed with the id's listener header.
        let frame = Frame::new(3, b"down".to_vec());
        handle.send(&frame).expect("the handle writes");
        let expected = {
            let mut unit = encode_header(assign::arb_id(mcu_id).expect("a valid mcu id"));
            unit.extend_from_slice(&frame.raw_bytes());
            unit
        };
        assert_eq!(read_n(&pty.master, expected.len()), expected);

        // Port → host: a unit headed with the listener id and one headed with
        // the reply id both decode to this slot.
        for arb in [
            assign::arb_id(mcu_id).expect("a valid mcu id"),
            assign::reply_arb_id(mcu_id).expect("a valid mcu id"),
        ] {
            let mut unit = encode_header(arb);
            unit.extend_from_slice(&Frame::encode(5, b"up"));
            master_write(&pty.master, &unit);
            assert_eq!(handle.receive(), Some(Frame::new(5, b"up".to_vec())));
        }

        // The admin unit for the bridge goes out exactly as the builder spells
        // it: the ADMIN_LISTEN header, the KXID frame and its trailing SYNC.
        let command = set_port_node_id(0, mcu_id as u8, MUX_PORT_UNITS);
        assert_eq!(*command.last().expect("a whole unit"), MESSAGE_SYNC);
        handle
            .send_set_port_node_id(0, mcu_id as u8, MUX_PORT_UNITS)
            .expect("the admin unit goes out");
        assert_eq!(read_n(&pty.master, command.len()), command);

        // The device-address commands of the same family: the restart a named
        // bridge-port device's `admin` method sends, and the bootloader request
        // the flashing tool sends.
        let restart = admin_unit(&[CMD_RESTART_DEVICE, 0]);
        handle
            .send_restart_device(0)
            .expect("the restart unit goes out");
        assert_eq!(read_n(&pty.master, restart.len()), restart);

        let bootloader = admin_unit(&[CMD_REQUEST_BOOTLOADER, 0]);
        handle
            .send_request_bootloader(0, None)
            .expect("the bootloader request goes out");
        assert_eq!(read_n(&pty.master, bootloader.len()), bootloader);

        // The flashing tunnel toggle: `[0x06][index]`, the same admin-unit
        // shape one command byte on.
        let tunnel = admin_unit(&[CMD_SET_PORT_TUNNEL, 3]);
        handle
            .send_set_port_tunnel(3)
            .expect("the tunnel toggle goes out");
        assert_eq!(read_n(&pty.master, tunnel.len()), tunnel);

        // An out-of-range index is not the host's to reject: the bridge drops
        // an index its port table does not list in silence, so the sender
        // writes it all the same.
        let out_of_range = admin_unit(&[CMD_SET_PORT_TUNNEL, 200]);
        handle
            .send_set_port_tunnel(200)
            .expect("an out-of-range index still goes out");
        assert_eq!(read_n(&pty.master, out_of_range.len()), out_of_range);
    }

    /// A downstream device joins the **same** port singleton an attached MCU
    /// opened: one reader routes both slots, so a unit for either id reaches
    /// its own handle.
    #[test]
    fn attach_downstream_shares_the_port_with_an_attached_mcu() {
        let pty = Pty::new();
        let upstream = attach_with_reply(&pty, 4);
        let downstream = attach_downstream(&pty.path, DEFAULT_BAUD, 5)
            .expect("a downstream attach on a live port");

        // The one reader routes them apart: each unit reaches its own handle.
        master_write(&pty.master, &device_unit(5, &Frame::encode(2, b"five")));
        master_write(&pty.master, &device_unit(4, &Frame::encode(3, b"four")));
        assert_eq!(downstream.receive(), Some(Frame::new(2, b"five".to_vec())));
        assert_eq!(upstream.receive(), Some(Frame::new(3, b"four".to_vec())));

        // And both write through that one port, each under its own header.
        let frame_a = Frame::new(6, b"aa".to_vec());
        let frame_b = Frame::new(7, b"bb".to_vec());
        upstream.send(&frame_a).expect("the attached MCU writes");
        downstream
            .send(&frame_b)
            .expect("the downstream device writes");
        let wire = read_n(
            &pty.master,
            2 + frame_a.raw_bytes().len() + 2 + frame_b.raw_bytes().len(),
        );
        let mut units = split_units(&wire);
        units.sort_by_key(|(arb, _)| *arb);
        let mut expected = vec![
            (assign::arb_id(4).unwrap(), frame_a),
            (assign::arb_id(5).unwrap(), frame_b),
        ];
        expected.sort_by_key(|(arb, _)| *arb);
        assert_eq!(units, expected);
    }

    /// The handshake a downstream device runs once its route exists: on a
    /// handle already open — [`attach_downstream`], which writes nothing —
    /// `handshake` sends exactly the segment [`attach`] runs after registering
    /// its slot: the SET_NODEID unit naming the board, then the assignment
    /// request, until the peer answers. A caller that wanted the bridge told
    /// the route first (`SET_PORT_NODEID`) could not express this with
    /// `attach`/`load_mux`, which run their handshake before handing back the
    /// handle.
    #[test]
    fn handshake_names_an_open_handle_and_completes_the_assignment() {
        let pty = Pty::new();
        let uuid = [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc];
        let mcu_id = 4;

        // The handle is opened without a handshake and writes nothing yet.
        let handle = attach_downstream(&pty.path, DEFAULT_BAUD, mcu_id)
            .expect("a downstream attach does not handshake");
        assert!(
            !readable_within(&pty.master, Duration::from_millis(100)),
            "attach_downstream must write nothing"
        );

        let set = set_node_id_for(uuid, mcu_id).expect("a valid mcu id");
        let request = assignment_request(mcu_id).expect("a valid mcu id");
        let worker = Arc::clone(&handle);
        let thread = thread::spawn(move || worker.handshake(uuid, ASSIGN_ATTACH_TIMEOUT));

        // The whole SET_NODEID unit — the admin listen header and the KXID
        // frame naming the board — goes out first, the assignment request after.
        let wire = read_n_within(
            &pty.master,
            set.len() + request.len(),
            Duration::from_secs(5),
        );
        assert_eq!(&wire[..set.len()], set.as_slice(), "SET_NODEID first");
        assert_eq!(
            &wire[set.len()..],
            request.as_slice(),
            "the assignment request follows"
        );

        master_write(
            &pty.master,
            &assignment_reply(mcu_id).expect("a valid mcu id"),
        );
        thread
            .join()
            .expect("the handshake thread")
            .expect("the peer answered, so the handshake completes");

        // The handle the caller already held is now an ordinary mux slot.
        let frame = Frame::new(1, b"after".to_vec());
        handle.send(&frame).expect("the handle writes");
        let mut expected = encode_header(assign::arb_id(mcu_id).expect("a valid mcu id"));
        expected.extend_from_slice(&frame.raw_bytes());
        assert_eq!(
            read_n_within(&pty.master, expected.len(), Duration::from_secs(5)),
            expected
        );
        master_write(
            &pty.master,
            &device_unit(mcu_id, &Frame::encode(2, b"back")),
        );
        assert_eq!(handle.receive(), Some(Frame::new(2, b"back".to_vec())));
    }

    /// A handshake that gets no answer times out and leaves the handle alive.
    /// Unlike [`attach`], which unregisters the slot it opened on failure, this
    /// handle was the caller's already — a missed handshake must not break the
    /// port it shares.
    #[test]
    fn a_handshake_that_times_out_leaves_the_handle_usable() {
        let pty = Pty::new();
        let uuid = [0x0f, 0x1e, 0x2d, 0x3c, 0x4b, 0x5a];
        let mcu_id = 4;

        let handle = attach_downstream(&pty.path, DEFAULT_BAUD, mcu_id)
            .expect("a downstream attach does not handshake");
        let err = handle
            .handshake(uuid, Duration::from_millis(50))
            .expect_err("the peer never answers");
        assert_eq!(err, InterfaceError::Timeout);

        // Every attempt went out: one SET_NODEID, the request per attempt.
        let set = set_node_id_for(uuid, mcu_id).expect("a valid mcu id");
        let request = assignment_request(mcu_id).expect("a valid mcu id");
        let mut expected = set.clone();
        for _ in 0..ASSIGN_ATTACH_ATTEMPTS {
            expected.extend_from_slice(&request);
        }
        assert_eq!(
            read_n_within(&pty.master, expected.len(), Duration::from_secs(5)),
            expected,
            "one SET_NODEID and a request per attempt"
        );

        // The failed handshake did not break the slot: frames still round-trip.
        let frame = Frame::new(5, b"still".to_vec());
        handle.send(&frame).expect("the handle still writes");
        let mut unit = encode_header(assign::arb_id(mcu_id).expect("a valid mcu id"));
        unit.extend_from_slice(&frame.raw_bytes());
        assert_eq!(
            read_n_within(&pty.master, unit.len(), Duration::from_secs(5)),
            unit
        );
        master_write(
            &pty.master,
            &device_unit(mcu_id, &Frame::encode(6, b"alive")),
        );
        assert_eq!(handle.receive(), Some(Frame::new(6, b"alive".to_vec())));
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
        let dead = port_registry()
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

    /// Poll `slot` for `frame` until `window` expires.
    ///
    /// Frames are consumed as they arrive, so a caller asks for them in the
    /// order they were sent. Polling — rather than a blocking `receive` — is
    /// what lets a test that expects *nothing* end: a frame that never arrives
    /// runs out the window instead of hanging.
    fn wait_for_frame(slot: &McuSlot, frame: &Frame, window: Duration) -> bool {
        let deadline = Instant::now() + window;
        loop {
            if let Ok(mut incoming) = slot.incoming.try_lock() {
                if incoming.next_frame().as_ref() == Some(frame) {
                    return true;
                }
            }
            if Instant::now() >= deadline {
                return false;
            }
            thread::sleep(Duration::from_millis(2));
        }
    }

    /// Whether the port for `path` is dropped before `window` expires: its
    /// registry entry is dead, which says its `Drop` ran to the end.
    fn port_is_gone(path: &Path, window: Duration) -> bool {
        let deadline = Instant::now() + window;
        loop {
            if port_registry()
                .lock()
                .unwrap()
                .get(path)
                .and_then(Weak::upgrade)
                .is_none()
            {
                return true;
            }
            if Instant::now() >= deadline {
                return false;
            }
            thread::sleep(Duration::from_millis(2));
        }
    }

    /// A handle's slot goes with the handle that registered it.
    ///
    /// `push_frame` cannot tell an abandoned slot from a live one, so a slot
    /// left in the map after its handle is gone only piles up bytes nobody
    /// will read — for as long as any **other** handle keeps the port up. The
    /// dropped id's units must go nowhere from then on, the port's other ids
    /// must be untouched, and a later attach on the same id must start on an
    /// empty stream: the queued units went with the slot they were queued in,
    /// they are not handed to the new handle.
    #[test]
    fn a_dropped_handle_unregisters_its_slot_and_the_next_attach_starts_empty() {
        let pty = Pty::new();
        let handle4 = attach_with_reply(&pty, 4);
        let handle5 = attach_with_reply(&pty, 5);
        // Holds the port up after every handle is gone, so the slot map is
        // still there to be looked at.
        let port = Arc::clone(&handle4.port);

        // One write, one read: MCU 5's frames are the markers, and units are
        // dispatched in order, so hearing the second one says the id-4 unit
        // between them is queued in slot 4. MCU 4 never reads it — that is the
        // backlog the slot must die with.
        let stale = Frame::new(1, b"stale".to_vec());
        let mut wire = device_unit(5, &Frame::encode(2, b"ahead"));
        wire.extend_from_slice(&device_unit(4, &stale.raw_bytes()));
        wire.extend_from_slice(&device_unit(5, &Frame::encode(3, b"later")));
        master_write(&pty.master, &wire);
        assert!(
            wait_for_frame(
                &handle5.slot,
                &Frame::new(2, b"ahead".to_vec()),
                Duration::from_secs(2)
            ),
            "the marker unit must be dispatched"
        );
        assert!(
            wait_for_frame(
                &handle5.slot,
                &Frame::new(3, b"later".to_vec()),
                Duration::from_secs(2)
            ),
            "the unit for the dropped id must have been dispatched in between"
        );

        // The slot leaves the map with the handle that registered it; another
        // MCU's slot stays.
        drop(handle4);
        {
            let slots = port.slots.lock().unwrap();
            assert!(
                slots.get(&4).is_none(),
                "the slot must be unregistered with its last handle"
            );
            assert!(slots.contains_key(&5), "another MCU's slot is untouched");
        }

        // A later attach on the same id gets a fresh slot: the stale bytes are
        // not delivered, and what follows them is.
        let handle4 = attach_with_reply(&pty, 4);
        assert!(
            handle4.slot.incoming.lock().unwrap().next_frame().is_none(),
            "a fresh slot must not inherit the abandoned slot's backlog"
        );
        let fresh = Frame::new(4, b"fresh".to_vec());
        master_write(&pty.master, &device_unit(4, &Frame::encode(4, b"fresh")));
        assert!(
            wait_for_frame(&handle4.slot, &fresh, Duration::from_secs(2)),
            "the fresh slot must receive the units that follow"
        );
    }

    /// `shutdown` unregisters the slot, not just the handle's stopped flag.
    ///
    /// A stopped handle never reads its slot again (`receive` returns `None`),
    /// so a slot left in the map only gives `push_frame` a place to pile up
    /// bytes nobody drains — for as long as the port lives. The slot must go
    /// with the shutdown, the port's other ids must be untouched, and the
    /// shut-down id's units must then be dispatched nowhere. The `Drop` path is
    /// the other caller of the same unregister; this pins the `shutdown` one.
    #[test]
    fn a_shutdown_handle_unregisters_its_slot_and_stops_being_fed() {
        let pty = Pty::new();
        let handle4 = attach_with_reply(&pty, 4);
        let handle5 = attach_with_reply(&pty, 5);
        // Keeps the port up through the asserts, so its registry entry is a
        // live `Weak` and there is a slot map left to look at.
        let port = Arc::clone(&handle5.port);

        handle4.shutdown();
        assert_eq!(
            handle4.receive(),
            None,
            "a shut-down handle delivers nothing"
        );
        {
            let slots = port.slots.lock().unwrap();
            assert!(
                slots.get(&4).is_none(),
                "shutdown must unregister the handle's slot"
            );
            assert!(slots.contains_key(&5), "another MCU's slot is untouched");
        }

        // The unit for the shut-down id sits in front of id 5's frame, so the
        // reader has already read it by the time the marker arrives: its bytes
        // are consumed and dropped, not queued in the orphaned slot.
        let mut wire = device_unit(4, &Frame::encode(4, b"stale"));
        wire.extend_from_slice(&device_unit(5, &Frame::encode(7, b"alive")));
        master_write(&pty.master, &wire);
        assert!(
            wait_for_frame(
                &handle5.slot,
                &Frame::new(7, b"alive".to_vec()),
                Duration::from_secs(2)
            ),
            "the still-registered id must keep receiving"
        );
        assert!(
            handle4.slot.incoming.lock().unwrap().next_frame().is_none(),
            "a shut-down handle's slot must not be fed"
        );
        assert!(
            port.slots.lock().unwrap().get(&4).is_none(),
            "the shut-down id must stay unregistered after the write"
        );

        // The port is untouched: still open, still registered, still serving id 5.
        assert!(
            port_registry()
                .lock()
                .unwrap()
                .get(&pty.path)
                .and_then(Weak::upgrade)
                .is_some(),
            "shutdown must not take the port down"
        );
    }

    /// The `ptr_eq` guard keeps a **stale** handle from unregistering a slot a
    /// newer attach owns: every attach replaces the map entry, so the older
    /// handle's `shutdown` (and its later `Drop`) must leave the live one
    /// alone. Both the `shutdown` teardown and the guarded attach-failure path
    /// depend on that branch.
    #[test]
    fn a_stale_handle_leaves_the_newer_attachs_slot_alone() {
        let pty = Pty::new();
        let first = attach_with_reply(&pty, 4);
        let second = attach_with_reply(&pty, 4);
        let port = Arc::clone(&second.port);

        first.shutdown();
        {
            let slots = port.slots.lock().unwrap();
            let live = slots
                .get(&4)
                .expect("the newer attach's slot is still registered");
            assert!(
                Arc::ptr_eq(live, &second.slot),
                "a stale handle must not take the newer slot down"
            );
        }

        master_write(&pty.master, &device_unit(4, &Frame::encode(3, b"live")));
        assert!(
            wait_for_frame(
                &second.slot,
                &Frame::new(3, b"live".to_vec()),
                Duration::from_secs(2)
            ),
            "the newer attach's handle must still be fed"
        );

        // Dropping the stale handle is the same guard from the other caller.
        drop(first);
        master_write(&pty.master, &device_unit(4, &Frame::encode(4, b"still")));
        assert!(
            wait_for_frame(
                &second.slot,
                &Frame::new(4, b"still".to_vec()),
                Duration::from_secs(2)
            ),
            "the live slot survives the stale handle's Drop too"
        );
    }

    /// Drop `pty`'s last two handles while the reader is inside `dispatch`
    /// holding the strong port reference it upgraded, and report the reader
    /// thread's id (in the `{:?}` form a panic hook reports it, which is what
    /// the caller has to match) together with whether that window was hit.
    ///
    /// The reader is pinned inside `dispatch` by holding the id-4 slot's
    /// `incoming` — the lock `push_frame` wants for the id-4 unit the reader
    /// has already read. The id-5 unit written in front of it is the proof the
    /// reader got that far (`push_frame` for it needs no lock the test holds),
    /// and from there the reader can neither finish the dispatch nor let go of
    /// the port until the lock is released. Only then do the handles go, which
    /// leaves the reader's own `drop(port)` as the release that reaches zero:
    /// `SharedSerialPort::drop` runs on the reader thread — the case that must
    /// not join the reader onto itself.
    ///
    /// `false` means the attempt missed the window (the reader looked the port
    /// up after the handles were already gone, so it returned without pushing
    /// anything): the caller then tries again on a fresh pty. The pinned frame
    /// landing in the slot is what proves the port was live for the reader —
    /// the observable a correctly torn-down port still produces.
    fn drop_last_two_handles_mid_dispatch(pty: &Pty) -> (String, bool) {
        let handle4 = attach_with_reply(pty, 4);
        let handle5 = attach_with_reply(pty, 5);
        let reader = format!(
            "{:?}",
            handle4
                .port
                .reader
                .lock()
                .unwrap()
                .as_ref()
                .expect("an open port has started its reader")
                .thread()
                .id()
        );
        let slot4 = Arc::clone(&handle4.slot);
        let incoming = slot4.incoming.lock().unwrap();

        let held = Frame::new(1, b"held".to_vec());
        let mut wire = device_unit(5, &Frame::encode(2, b"ahead"));
        wire.extend_from_slice(&device_unit(4, &Frame::encode(1, b"held")));
        master_write(&pty.master, &wire);
        assert!(
            wait_for_frame(
                &handle5.slot,
                &Frame::new(2, b"ahead".to_vec()),
                Duration::from_secs(2)
            ),
            "the reader dispatches the id-5 unit before it reaches the pinned id-4 one"
        );

        // The last handles go while the reader still holds its reference.
        drop(handle5);
        drop(handle4);
        drop(incoming);

        // The pinned frame arriving is the proof the reader had the port; and
        // with the reader as the dropping thread, the port is gone. A miss
        // costs the frame window and nothing else — the port is then gone
        // already, and only the frame distinguishes the two.
        let hit = wait_for_frame(&slot4, &held, Duration::from_millis(500))
            && port_is_gone(&pty.path, Duration::from_secs(1));
        (reader, hit)
    }

    /// Record the panics raised while `body` runs, as `(thread id, message)`.
    ///
    /// The reader thread is nobody's `JoinHandle` once the port's `Drop` has
    /// taken it, so a panic on it fails nothing and is invisible in the test
    /// result — a hook is the only way to assert on one. Every panic still goes
    /// to the hook that was in place, so panics from other tests (`#[should_panic]`
    /// ones run in the same process) behave exactly as they did. The hook is
    /// restored even when `body` panics: the panic is caught only long enough to
    /// put the hook back, then resumed.
    fn record_panics_during<T>(body: impl FnOnce() -> T) -> (T, Vec<(String, String)>) {
        /// Every panic seen while the hook below is installed.
        static PANICS: Mutex<Vec<(String, String)>> = Mutex::new(Vec::new());

        let previous = Arc::new(std::panic::take_hook());
        let forward = Arc::clone(&previous);
        std::panic::set_hook(Box::new(move |info| {
            PANICS
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner())
                .push((
                    format!("{:?}", std::thread::current().id()),
                    format!("{info}"),
                ));
            forward(info);
        }));
        // `catch_unwind` so the hook is restored even when `body` panics;
        // otherwise it would stay installed past this test and forward later
        // panics through `previous` here.
        let value = std::panic::catch_unwind(std::panic::AssertUnwindSafe(body));
        std::panic::set_hook(Box::new(move |info| previous(info)));
        let seen = std::mem::take(&mut *PANICS.lock().unwrap());
        match value {
            Ok(value) => (value, seen),
            Err(panic) => std::panic::resume_unwind(panic),
        }
    }

    /// The reader can be the thread that drops the port.
    ///
    /// The reader holds a strong reference from its `weak.upgrade()` until it
    /// drops the port again after `dispatch`, so the last handle dropped inside
    /// that window leaves the reader's own `drop(port)` as the release that
    /// reaches zero — and `SharedSerialPort::drop` then runs **on the reader
    /// thread**. Joining a thread from within it never returns (`pthread_join`
    /// reports `EDEADLK`, which std turns into a panic), which would tear the
    /// port down halfway and kill the reader in the middle of that teardown, so
    /// the drop has to recognise its own reader, skip the join, and still take
    /// the port down.
    ///
    /// The window is constructed rather than raced for — the attempt pins the
    /// reader inside `dispatch`, and a fixed port is what delivers the pinned
    /// frame and then dies — so a miss is a retry, not a verdict. A panic on
    /// the reader thread is what libtest cannot see; the hook is what turns it
    /// into the failure below.
    #[test]
    fn dropping_the_last_handle_mid_dispatch_does_not_join_the_reader_onto_itself() {
        let (outcome, panics) = record_panics_during(|| {
            let mut readers = Vec::new();
            let mut hit = false;
            for _ in 0..20 {
                let pty = Pty::new();
                let (reader, landed) = drop_last_two_handles_mid_dispatch(&pty);
                readers.push(reader);
                if !landed {
                    continue; // the window was missed; a fresh pty tries again
                }
                // The port went down on the reader thread; the same path then
                // opens a fresh port and routes frames again.
                let handle = attach_with_reply(&pty, 4);
                let frame = Frame::new(4, b"next".to_vec());
                master_write(&pty.master, &device_unit(4, &Frame::encode(4, b"next")));
                assert!(
                    wait_for_frame(&handle.slot, &frame, Duration::from_secs(2)),
                    "a fresh port on the same path must route frames again"
                );
                hit = true;
                break;
            }
            (readers, hit)
        });

        let (readers, hit) = outcome;
        assert!(
            hit,
            "the reader must be droppable mid-dispatch; the window is constructed, not raced for"
        );
        assert!(
            panics
                .iter()
                .all(|(thread, _)| !readers.iter().any(|reader| reader == thread)),
            "a reader thread ended in a panic — the port was dropped by the reader and joined \
             itself: {panics:?}"
        );
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
            wire.extend_from_slice(&encode_header(assign::reply_arb_id(4).unwrap()));
            wire.push(bad);
            wire.push(MESSAGE_SYNC);
        }
        wire.extend_from_slice(&device_unit(4, &Frame::encode(7, b"after")));
        master_write(&pty.master, &wire);

        // The port resynchronised and dispatched the valid unit behind the junk.
        assert_eq!(handle4.receive(), Some(Frame::new(7, b"after".to_vec())));
    }

    // -----------------------------------------------------------------------
    // The unit registry: whole units under an id no MCU owns, handed to the
    // listener registered for that id — a scan reading a bridge's admin answers.
    // -----------------------------------------------------------------------

    /// One whole unit under `arb`: the header [`encode_header`] writes, then one
    /// legal klipper frame — the shape a bridge's downstream devices answer with.
    fn unit_for(arb: u32, frame: &Frame) -> Vec<u8> {
        let mut unit = encode_header(arb);
        unit.extend_from_slice(&frame.raw_bytes());
        unit
    }

    #[test]
    fn a_unit_no_mcu_owns_is_delivered_whole_to_the_sink_of_its_arb() {
        let pty = Pty::new();
        let handle4 = attach_within(&pty, 4);
        let (sink, units) = UnitSink::channel();
        handle4.set_unit_sink(ADMIN_REPLY, sink);

        let unit = unit_for(ADMIN_REPLY, &Frame::new(1, b"answer".to_vec()));
        master_write(&pty.master, &unit);

        // Byte for byte, header included: the listener reassembles nothing.
        assert_eq!(
            units
                .recv_timeout(Duration::from_secs(2))
                .expect("the unit is delivered to its arb's sink"),
            unit,
            "the whole unit arrives exactly as it was written"
        );

        // And it went nowhere else: the unit names an id no MCU owns.
        assert!(
            !wait_for_frame(
                &handle4.slot,
                &Frame::new(1, b"answer".to_vec()),
                Duration::from_millis(200)
            ),
            "an admin answer never reaches an MCU slot"
        );
    }

    #[test]
    fn clearing_a_unit_sink_restores_the_drop_and_leaves_the_slot_alone() {
        let pty = Pty::new();
        let handle4 = attach_within(&pty, 4);
        let (sink, units) = UnitSink::channel();
        handle4.set_unit_sink(ADMIN_REPLY, sink);

        // A live listener takes the first answer.
        master_write(
            &pty.master,
            &unit_for(ADMIN_REPLY, &Frame::new(6, b"first".to_vec())),
        );
        assert!(
            units.recv_timeout(Duration::from_secs(2)).is_ok(),
            "the registered listener takes the unit"
        );

        // Cleared: the answer under the admin id is dropped again, the way it
        // was before there were listeners. The ordinary unit behind it is the
        // barrier — units are dispatched in order, so hearing it says the answer
        // before it is already handled (dropped, not queued, and the drop
        // stopped at that unit rather than eating the stream).
        handle4.clear_unit_sink(ADMIN_REPLY);
        let mut wire = unit_for(ADMIN_REPLY, &Frame::new(7, b"dropped".to_vec()));
        wire.extend_from_slice(&device_unit(4, &Frame::encode(8, b"alive")));
        master_write(&pty.master, &wire);
        assert!(
            wait_for_frame(
                &handle4.slot,
                &Frame::new(8, b"alive".to_vec()),
                Duration::from_secs(2)
            ),
            "the port keeps dispatching the units of a real MCU"
        );
        assert!(units.try_recv().is_err(), "a cleared listener gets nothing");
    }

    #[test]
    fn a_unit_sink_and_an_mcu_slot_do_not_take_each_others_units() {
        let pty = Pty::new();
        let handle4 = attach_within(&pty, 4);
        let (sink, units) = UnitSink::channel();
        handle4.set_unit_sink(ADMIN_REPLY, sink);

        // The slot's unit: an ordinary data frame under MCU 4's answer id.
        master_write(&pty.master, &device_unit(4, &Frame::encode(2, b"slot")));
        assert!(
            wait_for_frame(
                &handle4.slot,
                &Frame::new(2, b"slot".to_vec()),
                Duration::from_secs(2)
            ),
            "an MCU's own unit still reaches its slot"
        );
        assert!(
            units.recv_timeout(Duration::from_millis(200)).is_err(),
            "an MCU's own unit is not handed to the admin listener"
        );

        // The sink's unit: the admin id, which no MCU owns.
        let admin = unit_for(ADMIN_REPLY, &Frame::new(3, b"admin".to_vec()));
        master_write(&pty.master, &admin);
        assert_eq!(
            units
                .recv_timeout(Duration::from_secs(2))
                .expect("the admin unit is delivered"),
            admin,
            "the listener gets the admin unit whole"
        );
        assert!(
            !wait_for_frame(
                &handle4.slot,
                &Frame::new(3, b"admin".to_vec()),
                Duration::from_millis(200)
            ),
            "and the admin unit is not handed to a slot"
        );
    }

    #[test]
    fn registering_a_second_unit_sink_replaces_the_first() {
        let pty = Pty::new();
        let handle4 = attach_within(&pty, 4);

        let (first, first_units) = UnitSink::channel();
        handle4.set_unit_sink(ADMIN_REPLY, first);
        let (second, second_units) = UnitSink::channel();
        handle4.set_unit_sink(ADMIN_REPLY, second);

        // The newer listener takes the unit; the replaced one is gone with its
        // registration (its sender went with the map entry).
        let unit = unit_for(ADMIN_REPLY, &Frame::new(4, b"replaced".to_vec()));
        master_write(&pty.master, &unit);
        assert_eq!(
            second_units
                .recv_timeout(Duration::from_secs(2))
                .expect("the newer listener takes the unit"),
            unit
        );
        assert!(
            first_units
                .recv_timeout(Duration::from_millis(200))
                .is_err(),
            "the replaced listener gets nothing"
        );

        // One clear removes the replacement: the id has no listener at all, and
        // the unit is dropped (the ordinary unit behind it is the barrier).
        handle4.clear_unit_sink(ADMIN_REPLY);
        let mut wire = unit_for(ADMIN_REPLY, &Frame::new(5, b"after clear".to_vec()));
        wire.extend_from_slice(&device_unit(4, &Frame::encode(9, b"after clear")));
        master_write(&pty.master, &wire);
        assert!(
            wait_for_frame(
                &handle4.slot,
                &Frame::new(9, b"after clear".to_vec()),
                Duration::from_secs(2)
            ),
            "the port keeps dispatching the units of a real MCU"
        );
        assert!(
            second_units.try_recv().is_err(),
            "one clear unregisters the arb"
        );
        assert!(
            !wait_for_frame(
                &handle4.slot,
                &Frame::new(5, b"after clear".to_vec()),
                Duration::from_millis(200)
            ),
            "a cleared id's unit goes nowhere"
        );
    }

    // -----------------------------------------------------------------------
    // The assignment pseudo-frame: shape, old-firmware behavior, and the
    // handshake's own codec.
    // -----------------------------------------------------------------------

    /// The patterned pseudo-frame as raw wire bytes: the fixed `KXID` body the
    /// handshake carries in both directions.
    fn pseudo_frame() -> Vec<u8> {
        assignment_frame()
    }

    #[test]
    fn the_pseudo_frame_is_a_well_formed_frame_with_only_the_seq_byte_wrong() {
        let bytes = pseudo_frame();
        // len = header(2) + payload("KXID" = 4) + trailer(3) = 9.
        assert_eq!(bytes.len(), 9);
        assert_eq!(bytes[MESSAGE_POS_LEN], 9);
        assert_eq!(bytes[MESSAGE_POS_SEQ], ASSIGN_SEQ_BYTE);
        // The one field that differs from an ordinary frame: bit 4 clear.
        assert_ne!(bytes[MESSAGE_POS_SEQ] & !MESSAGE_SEQ_MASK, MESSAGE_DEST);
        assert_eq!(&bytes[2..6], b"KXID");
        assert_eq!(*bytes.last().unwrap(), MESSAGE_SYNC);
        // The CRC is the ordinary one over [len, seq, payload] — the shape a
        // capable firmware can validate without inventing a checksum.
        let expected = crate::crc16_ccitt!(&bytes[..bytes.len() - MESSAGE_TRAILER_SIZE]);
        let actual = ((bytes[bytes.len() - 3] as u16) << 8) | bytes[bytes.len() - 2] as u16;
        assert_eq!(expected, actual);
    }

    #[test]
    fn the_body_and_header_are_free_of_an_interior_sync() {
        // The firmware's error path discards through the first 0x7e: with the
        // length byte, sequence byte and payload free of it, the frame is
        // consumed in one step. The two CRC bytes are computed, not chosen; for
        // this fixed body they are 0x4b/0x4f, so the trailer's own 0x7e is the
        // only SYNC in the frame.
        let bytes = pseudo_frame();
        let header_and_payload = &bytes[..bytes.len() - MESSAGE_TRAILER_SIZE];
        assert!(
            header_and_payload.iter().all(|&b| b != MESSAGE_SYNC),
            "a SYNC appeared in the header/payload: {bytes:02x?}"
        );
        assert!((MESSAGE_MIN..=MESSAGE_MAX).contains(&bytes.len()));
    }

    // -----------------------------------------------------------------------
    // Old-firmware behavior: a faithful port of `command_find_block`
    // (`third_party/klipper/src/command.c:276-333`), so the compatibility
    // claim above is executable. Structure and citation, not an approximation:
    // the checks run in the C order and the error path consumes through the
    // first SYNC exactly as `memchr` does.
    // -----------------------------------------------------------------------

    /// What one call to the firmware's block finder decides.
    #[derive(Debug, PartialEq, Eq)]
    enum Outcome {
        /// Not enough bytes yet; the buffer waits.
        NeedMore,
        /// A block passed structure, CRC and sequence: dispatch follows.
        Dispatch,
        /// Structure failed; bytes consumed through the SYNC (`pop_count`),
        /// and (only on the first failure) an empty ack frame is sent.
        Discarded { consumed: usize, nacked: bool },
        /// Structure and CRC passed but the sequence was wrong: nak, and the
        /// block is consumed (`*pop_count = msglen` before the check).
        SeqNak { consumed: usize },
    }

    /// The firmware's static state across calls (`command.c:276-281,299,307`).
    struct FwState {
        /// `next_sequence`, initialised to `MESSAGE_DEST` (`command.c:16`).
        next_sequence: u8,
        /// `CF_NEED_SYNC`.
        need_sync: bool,
        /// `CF_NEED_VALID`.
        need_valid: bool,
    }

    impl Default for FwState {
        fn default() -> Self {
            // `static uint8_t next_sequence = MESSAGE_DEST;` (`command.c:16`).
            Self {
                next_sequence: MESSAGE_DEST,
                need_sync: false,
                need_valid: false,
            }
        }
    }

    fn fw_find_block(buf: &[u8], st: &mut FwState) -> Outcome {
        // `if (buf_len && sync_state & CF_NEED_SYNC) goto need_sync;`
        if !buf.is_empty() && st.need_sync {
            return fw_need_sync(buf, st);
        }
        if buf.len() < MESSAGE_MIN {
            return Outcome::NeedMore;
        }
        let msglen = buf[MESSAGE_POS_LEN] as usize;
        // `:284` — length range.
        if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&msglen) {
            return fw_error(buf, st);
        }
        let msgseq = buf[MESSAGE_POS_SEQ];
        // `:287` — the check this pattern is built to meet.
        if (msgseq & !MESSAGE_SEQ_MASK) != MESSAGE_DEST {
            return fw_error(buf, st);
        }
        if buf.len() < msglen {
            return Outcome::NeedMore;
        }
        // `:291` — trailer SYNC.
        if buf[msglen - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC {
            return fw_error(buf, st);
        }
        // `:293-297` — CRC.
        let msgcrc = ((buf[msglen - 3] as u16) << 8) | buf[msglen - 2] as u16;
        let crc = crate::crc16_ccitt!(&buf[..msglen - MESSAGE_TRAILER_SIZE]);
        if crc != msgcrc {
            return fw_error(buf, st);
        }
        // `:298` — a structure-valid block clears NEED_VALID.
        st.need_valid = false;
        // `:301-303` — sequence.
        if msgseq != st.next_sequence {
            return Outcome::SeqNak { consumed: msglen };
        }
        st.next_sequence = ((msgseq + 1) & MESSAGE_SEQ_MASK) | MESSAGE_DEST;
        Outcome::Dispatch
    }

    fn fw_error(buf: &[u8], st: &mut FwState) -> Outcome {
        // `:311-316` — a leading SYNC is dropped alone, silently.
        if buf.first() == Some(&MESSAGE_SYNC) {
            return Outcome::Discarded {
                consumed: 1,
                nacked: false,
            };
        }
        // `:317` then fall into need_sync (`:318`).
        st.need_sync = true;
        fw_need_sync(buf, st)
    }

    fn fw_need_sync(buf: &[u8], st: &mut FwState) -> Outcome {
        // `:317-323`: discard through the first SYNC (or the whole buffer),
        // then the one-time nak.
        let consumed = match buf.iter().position(|&b| b == MESSAGE_SYNC) {
            Some(position) => {
                st.need_sync = false;
                position + 1
            }
            None => buf.len(),
        };
        if st.need_valid {
            return Outcome::Discarded {
                consumed,
                nacked: false,
            };
        }
        st.need_valid = true;
        Outcome::Discarded {
            consumed,
            nacked: true,
        }
    }

    /// An ordinary, correctly-sequenced host block (identify's request frame).
    fn identify_frame(seq_nibble: u8) -> Vec<u8> {
        let seq = (seq_nibble & MESSAGE_SEQ_MASK) | MESSAGE_DEST;
        Frame::encode(seq, b"\x01identify offset=0 count=112")
    }

    #[test]
    fn the_firmware_discards_the_whole_unit_header_included() {
        // The pseudo-frame is what the old firmware's block parser chokes on,
        // but what arrives on the wire is the header *first*: its `0x80..=0xff`
        // first byte is already an out-of-range length, so the parser discards
        // through the next SYNC — for most ids our own trailer, and for the ids
        // whose header carries a `0x7e` (120..=127) that byte, leaving the
        // pseudo-frame to be discarded on the following call. Either way the unit
        // is thrown away in whole, with no dispatch and no sequence taken — the
        // "harmless to old firmware" property holds for the unit, not just its
        // body.
        for id in [1u32, 120, 255] {
            let unit = assignment_request(id).expect("a valid id");
            let mut st = FwState::default();
            let mut eaten = 0usize;
            let mut calls = 0;
            while eaten < unit.len() {
                match fw_find_block(&unit[eaten..], &mut st) {
                    Outcome::Discarded { consumed, .. } => {
                        assert!(consumed > 0, "id {id}: the parser must make progress");
                        eaten += consumed;
                    }
                    other => panic!("id {id}: expected a discard, got {other:?}"),
                }
                calls += 1;
                assert!(calls <= 4, "id {id}: the parser must not spin");
            }
            assert_eq!(eaten, unit.len(), "id {id}: the whole unit is discarded");
            assert_eq!(
                st.next_sequence, MESSAGE_DEST,
                "id {id}: no sequence consumed"
            );
        }
    }

    #[test]
    fn the_firmware_discards_the_request_without_dispatching_it() {
        let request = pseudo_frame();
        let mut st = FwState::default();
        // First failure: consumed exactly through our own trailer, one ack
        // frame in reply, no dispatch.
        assert_eq!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded {
                consumed: request.len(),
                nacked: true
            }
        );
        // `next_sequence` never moved — identify starts at its initial value.
        assert_eq!(st.next_sequence, MESSAGE_DEST);
        // A second assignment frame is discarded silently (NEED_VALID holds
        // until a valid block clears it).
        assert_eq!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded {
                consumed: request.len(),
                nacked: false
            }
        );
        // And the identify frame that follows dispatches: the pattern leaves
        // the stream exactly where it found it.
        let identify = identify_frame(MESSAGE_DEST);
        assert_eq!(fw_find_block(&identify, &mut st), Outcome::Dispatch);
    }

    #[test]
    fn a_request_sandwiched_before_identify_leaves_identify_intact() {
        // The wire order at connect: our frame, then identify's block. The
        // firmware consumes only ours; identify dispatches on the next call.
        let mut stream = pseudo_frame();
        let identify = identify_frame(MESSAGE_DEST);
        stream.extend_from_slice(&identify);

        let mut st = FwState::default();
        let (first, consumed) = match fw_find_block(&stream, &mut st) {
            outcome @ Outcome::Discarded { consumed, .. } => (outcome, consumed),
            other => panic!("the request must be discarded, got {other:?}"),
        };
        assert!(matches!(first, Outcome::Discarded { nacked: true, .. }));
        assert_eq!(consumed, 9, "only our frame is consumed");
        assert_eq!(
            fw_find_block(&stream[consumed..], &mut st),
            Outcome::Dispatch
        );
    }

    #[test]
    fn the_firmware_would_dispatch_garbage_in_the_pattern_but_never_our_bytes() {
        // The payload is never read on the error path — prove it by putting a
        // syntactically valid command in there with a *correct* CRC over it:
        // the sequence-byte check has already decided, so nothing downstream
        // is reached. (Removing the dest check would not make it dispatch
        // either: the sequence check runs next, `next_sequence` only ever holds
        // `0x10..=0x1f` and never `0x20`, so the block would be nakaed — both
        // checks refuse it.)
        let mut bytes = pseudo_frame();
        bytes[2..6].copy_from_slice(b"\x00sta"); // msgid 0 + text, CRC fixed:
        let crc = crate::crc16_ccitt!(&bytes[..bytes.len() - MESSAGE_TRAILER_SIZE]);
        let len = bytes.len();
        bytes[len - 3] = (crc >> 8) as u8;
        bytes[len - 2] = crc as u8;
        let mut st = FwState::default();
        assert!(matches!(
            fw_find_block(&bytes, &mut st),
            Outcome::Discarded { .. }
        ));
        assert_eq!(st.next_sequence, MESSAGE_DEST);
    }

    #[test]
    fn a_valid_block_rearms_the_one_nak_gate() {
        // `sync_state &= ~CF_NEED_VALID` (`command.c:298`) is the only thing
        // that re-arms the one-time nak: after it, the next structural error
        // nakks again, and the one after that is silent again.
        let mut st = FwState::default();
        let request = pseudo_frame();
        assert!(matches!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded { nacked: true, .. }
        ));
        // A valid identify block clears the flag on its way to dispatch.
        assert_eq!(
            fw_find_block(&identify_frame(MESSAGE_DEST), &mut st),
            Outcome::Dispatch
        );
        // So the *next* structural error nakks once more — re-armed …
        let request = pseudo_frame();
        assert!(matches!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded { nacked: true, .. }
        ));
        // … and damping resumes immediately after.
        assert!(matches!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded { nacked: false, .. }
        ));
        assert_eq!(st.next_sequence, MESSAGE_DEST | 1);
    }

    #[test]
    fn a_frame_split_across_reads_is_still_discarded_once_complete() {
        // Real links deliver bytes in arbitrary slices: the firmware buffers
        // (`need_more_data`, silent — `command.c:281-282`) until MESSAGE_MIN
        // bytes are present, then runs the same checks on the accumulated
        // buffer — our frame is discarded whole on the completing read.
        let request = pseudo_frame();
        let mut st = FwState::default();
        assert_eq!(fw_find_block(&request[..4], &mut st), Outcome::NeedMore);
        let mut buffered = request[..4].to_vec();
        buffered.extend_from_slice(&request[4..]);
        assert_eq!(
            fw_find_block(&buffered, &mut st),
            Outcome::Discarded {
                consumed: request.len(),
                nacked: true
            }
        );
        assert_eq!(st.next_sequence, MESSAGE_DEST);
    }

    #[test]
    fn a_request_after_identify_is_discarded_without_disturbing_the_session() {
        // The other half of the README's “before and after identify” claim:
        // arriving *after* a valid block, the flag was just cleared by it, so
        // this error nakks once (the harmless empty frame), then damps — never
        // dispatch, and the sequence identify advanced to stays put.
        let mut st = FwState::default();
        assert_eq!(
            fw_find_block(&identify_frame(MESSAGE_DEST), &mut st),
            Outcome::Dispatch
        );
        assert_eq!(st.next_sequence, MESSAGE_DEST | 1);

        let request = pseudo_frame();
        assert_eq!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded {
                consumed: request.len(),
                nacked: true
            }
        );
        assert_eq!(
            st.next_sequence,
            MESSAGE_DEST | 1,
            "the request must not move the sequence identify left behind"
        );
        assert!(matches!(
            fw_find_block(&request, &mut st),
            Outcome::Discarded { nacked: false, .. }
        ));
    }

    // -----------------------------------------------------------------------
    // The frame header codec and the assignment unit
    // -----------------------------------------------------------------------

    #[test]
    fn headers_are_two_bytes_across_the_whole_id_range() {
        for id in 1..=MAX_MCU_ID {
            for arb in [
                assign::arb_id(id).unwrap(),
                assign::reply_arb_id(id).unwrap(),
            ] {
                let header = encode_header(arb);
                assert_eq!(header.len(), 2, "arb {arb:#x}");
                assert_eq!(
                    decode_header(&header),
                    HeaderScan::Complete { arb, consumed: 2 }
                );
            }
        }
        // The endpoints of the assigned range are the two-byte codewords below;
        // the bytes carry check bits as well as value now, so they no longer
        // read as an integer the way a VLQ did.
        assert_eq!(encode_header(0x102), vec![0x84, 0x42]);
        assert_eq!(encode_header(0x2ff), vec![0xdf, 0x7d]);
        // The admin pair, whose headers are codewords for `0x3f0`/`0x3f1`.
        assert_eq!(encode_header(ADMIN_LISTEN), vec![0xe8, 0x3f]);
        assert_eq!(encode_header(ADMIN_REPLY), vec![0x98, 0x3f]);

        // A truncated header waits for the rest instead of guessing.
        assert_eq!(decode_header(&[]), HeaderScan::NeedMore);
        assert_eq!(decode_header(&[0x84]), HeaderScan::NeedMore);
        let full = encode_header(0x2ff);
        assert_eq!(decode_header(&full[..1]), HeaderScan::NeedMore);
        assert_eq!(
            decode_header(&full),
            HeaderScan::Complete {
                arb: 0x2ff,
                consumed: 2
            }
        );
    }

    #[test]
    fn boundary_id_headers_may_contain_a_sync_byte() {
        // The protocol allows 0x7e in frame *content* (`docs/Protocol.md:178-183`:
        // “a sync character is not exclusive to the framing and may be present
        // in the message block content”). A header whose second byte lands on
        // 0x7e is therefore acceptable: what delimits it is the fixed two bytes,
        // not the absence of SYNC. Id 120 is the pair that does it, and the
        // whole range is walked so that stays the only claim.
        let mut with_sync = Vec::new();
        for id in 1..=MAX_MCU_ID {
            let host = encode_header(assign::arb_id(id).unwrap());
            let reply = encode_header(assign::reply_arb_id(id).unwrap());
            for (arb, header) in [
                (assign::arb_id(id).unwrap(), &host),
                (assign::reply_arb_id(id).unwrap(), &reply),
            ] {
                assert_eq!(
                    decode_header(header),
                    HeaderScan::Complete { arb, consumed: 2 },
                    "id {id}"
                );
            }
            if host.contains(&MESSAGE_SYNC) || reply.contains(&MESSAGE_SYNC) {
                with_sync.push(id);
            }
        }
        assert_eq!(
            with_sync,
            (120..=127).collect::<Vec<u32>>(),
            "the ids whose header carries the SYNC byte"
        );
        assert_eq!(
            encode_header(assign::arb_id(120).unwrap()),
            vec![0xc0, 0x7e]
        );
        assert_eq!(
            encode_header(assign::reply_arb_id(120).unwrap()),
            vec![0xb0, 0x7e]
        );
    }

    #[test]
    fn a_single_bit_error_in_the_header_is_rejected() {
        // The check bits exist because the frame's CRC-16 does not cover the
        // header: every one-bit error in either byte is caught, so a corrupted
        // header can never quietly hand a valid frame to another MCU.
        for id in [1u32, 42, 120, 255] {
            for arb in [
                assign::arb_id(id).unwrap(),
                assign::reply_arb_id(id).unwrap(),
                ADMIN_LISTEN,
                ADMIN_REPLY,
            ] {
                let good = encode_header(arb);
                for bit in 0..8 * HEADER_BYTES {
                    let mut bad = good.clone();
                    bad[bit / 8] ^= 1 << (bit % 8);
                    assert_eq!(
                        decode_header(&bad),
                        HeaderScan::Invalid,
                        "arb {arb:#x}, bit {bit}"
                    );
                }
            }
        }
    }

    #[test]
    fn two_bit_errors_in_the_header_are_rejected() {
        // The parity-check columns are distinct and non-zero, so the code's
        // minimum distance is three: no two-bit error can land on another
        // codeword — the dangerous case, a wrong id that still checks out.
        for id in [1u32, 120, 255] {
            for arb in [
                assign::arb_id(id).unwrap(),
                assign::reply_arb_id(id).unwrap(),
            ] {
                let good = encode_header(arb);
                for first in 0..8 * HEADER_BYTES {
                    for second in first + 1..8 * HEADER_BYTES {
                        let mut bad = good.clone();
                        bad[first / 8] ^= 1 << (first % 8);
                        bad[second / 8] ^= 1 << (second % 8);
                        assert_eq!(
                            decode_header(&bad),
                            HeaderScan::Invalid,
                            "arb {arb:#x}, bits {first},{second}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn a_reserved_codeword_is_rejected_rather_than_routed() {
        // `0x3f2..=0x3ff` are well-formed codewords (their check bits are
        // computed the same way) that name no address: the escape value is where
        // a longer header would start, so a reader that meets one today drops
        // the unit instead of routing it. That is what keeps the door open for
        // extending the header later without a format change.
        for arb in HEADER_RESERVED_FIRST..=HEADER_MAX {
            let bytes = header_bytes(header_codeword(arb));
            assert_eq!(decode_header(&bytes), HeaderScan::Invalid, "arb {arb:#x}");
        }
    }

    #[test]
    #[should_panic(expected = "reserved")]
    fn encoding_a_reserved_id_panics() {
        // The encoder refuses the reserved band: emitting one would be a
        // programming error, not a config error (the callers validate first).
        encode_header(HEADER_RESERVED_FIRST);
    }

    #[test]
    fn assignment_units_round_trip_through_the_ack_parser() {
        let request = assignment_request(42).unwrap();
        let reply = assignment_reply(42).unwrap();

        // Request: the listener header, then the fixed pseudo-frame.
        assert_eq!(
            &request[..2],
            encode_header(assign::arb_id(42).unwrap()).as_slice()
        );
        assert_eq!(request.len(), 2 + 9);
        assert_eq!(request[2], 9); // frame length byte
        assert_eq!(request[3], ASSIGN_SEQ_BYTE);
        assert_eq!(&request[4..8], MAGIC);
        assert_eq!(*request.last().unwrap(), MESSAGE_SYNC);

        // Reply: the answer header; the body is byte-for-byte the request's.
        assert_eq!(
            &reply[..2],
            encode_header(assign::reply_arb_id(42).unwrap()).as_slice()
        );
        assert_eq!(&reply[2..], &request[2..]);

        // The parser recognizes the reply and rejects the request's direction.
        assert_eq!(parse_assignment_ack(&reply), Some(42));
        assert_eq!(parse_assignment_ack(&request), None);

        for id in [1u32, 42, 200, 255] {
            assert_eq!(
                parse_assignment_ack(&assignment_reply(id).unwrap()),
                Some(id),
                "id {id}"
            );
        }

        // Out-of-range ids have no unit at all.
        assert_eq!(assignment_request(0), None);
        assert_eq!(assignment_request(256), None);
        assert_eq!(assignment_reply(0), None);
        assert_eq!(assignment_reply(256), None);
    }

    #[test]
    fn a_malformed_assignment_ack_is_rejected() {
        let reply = assignment_reply(7).unwrap();
        assert_eq!(parse_assignment_ack(&reply), Some(7));

        // Bad CRC — flip the trailer's high CRC byte.
        let mut bad_crc = reply.clone();
        let crc_at = bad_crc.len() - 3;
        bad_crc[crc_at] ^= 0xff;
        assert_eq!(parse_assignment_ack(&bad_crc), None);

        // Wrong sequence byte.
        let mut bad_seq = reply.clone();
        bad_seq[2 + MESSAGE_POS_SEQ] = 0x10;
        assert_eq!(parse_assignment_ack(&bad_seq), None);

        // Truncation: the trailer's SYNC removed, and the header cut short.
        assert_eq!(parse_assignment_ack(&reply[..reply.len() - 1]), None);
        assert_eq!(parse_assignment_ack(&reply[..1]), None);

        // No MAGIC: a structurally valid pseudo-frame with different payload.
        let mut wrong_magic = encode_header(assign::reply_arb_id(7).unwrap());
        wrong_magic.extend_from_slice(
            &Frame::with_wire_seq(ASSIGN_SEQ_BYTE, b"XXXX".to_vec()).raw_bytes(),
        );
        assert_eq!(parse_assignment_ack(&wrong_magic), None);

        // Wrong trailer SYNC byte.
        let mut bad_trailer = reply.clone();
        *bad_trailer.last_mut().unwrap() = 0x00;
        assert_eq!(parse_assignment_ack(&bad_trailer), None);

        // Trailing bytes after the frame: not a self-consistent unit.
        let mut extra = reply.clone();
        extra.push(0x00);
        assert_eq!(parse_assignment_ack(&extra), None);
    }

    // -----------------------------------------------------------------------
    // Unassigned-device discovery on the shared serial wire
    // -----------------------------------------------------------------------

    /// A device→host admin answer as a whole unit: the admin reply header,
    /// then the `KXID` frame carrying [`RESP_NEED_NODEID`], `uuid`, and the
    /// application byte when there is one.
    fn admin_answer(uuid: [u8; 6], application: Option<u8>) -> Vec<u8> {
        let mut body = vec![RESP_NEED_NODEID];
        body.extend_from_slice(&uuid);
        body.extend(application);
        let mut unit = encode_header(ADMIN_REPLY);
        unit.extend_from_slice(&kxid_unit(&body));
        unit
    }

    /// The same answer with `tail` after the UUID, whatever it is: the shape a
    /// bridge's answer arrives in, where the application byte and the port path
    /// are each optional and each told apart by the top bit of its first byte.
    fn admin_answer_tail(uuid: [u8; 6], tail: &[u8]) -> Vec<u8> {
        let mut body = vec![RESP_NEED_NODEID];
        body.extend_from_slice(&uuid);
        body.extend_from_slice(tail);
        let mut unit = encode_header(ADMIN_REPLY);
        unit.extend_from_slice(&kxid_unit(&body));
        unit
    }

    #[test]
    fn the_discovery_query_is_the_admin_broadcast_unit() {
        // The bytes on the wire, exactly: the admin listen id as a checked
        // header, then the `KXID` frame with the query byte, its computed CRC
        // and its trailer — CAN's broadcast (`0x3f0`, `CMD_QUERY_UNASSIGNED`)
        // carried inside the stream frame. Only the frame's own trailer is a
        // SYNC.
        assert_eq!(
            discovery_query(),
            [0xe8, 0x3f, 0x0a, 0x20, 0x4b, 0x58, 0x49, 0x44, 0x00, 0x12, 0xd6, 0x7e]
        );
        let unit = discovery_query();
        assert_eq!(
            unit.iter().filter(|&&b| b == MESSAGE_SYNC).count(),
            1,
            "no interior 0x7e in the header/payload/CRC"
        );
    }

    #[test]
    fn set_node_id_matches_can_s_admin_frame() {
        // CAN's payload is `[0x01][uuid msb-first][nodeid]`; here the same
        // payload rides inside the `KXID` stream frame on the admin listen id.
        assert_eq!(
            set_node_id_for([0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc], 2)
                .expect("2 is an addressable id"),
            [
                0xe8, 0x3f, 0x11, 0x20, 0x4b, 0x58, 0x49, 0x44, 0x01, 0x11, 0xaa, 0x22, 0xbb, 0x33,
                0xcc, 0x02, 0xa0, 0x5e, 0x7e
            ]
        );
    }

    #[test]
    fn set_node_id_rejects_ids_outside_the_range() {
        // Zero names no node, and 0x100 is past the last one — neither has an
        // arbitration id, so neither has a unit.
        assert_eq!(set_node_id_for([0; 6], 0), None);
        assert_eq!(set_node_id_for([0; 6], 0x100), None);
        // The ends of the range do.
        assert!(set_node_id_for([0; 6], 1).is_some());
        assert!(set_node_id_for([0; 6], 0xff).is_some());
    }

    #[test]
    fn set_node_id_round_trips_a_uuid_containing_sync() {
        // The protocol allows SYNC in message block content, so a UUID byte of
        // 0x7e is ordinary content — the frame is delimited by its length
        // byte, and this golden frame pins that the length is authoritative.
        assert_eq!(
            set_node_id_for([0x7e, 0x11, 0x22, 0x33, 0x44, 0x55], 1)
                .expect("1 is an addressable id"),
            [
                0xe8, 0x3f, 0x11, 0x20, 0x4b, 0x58, 0x49, 0x44, 0x01, 0x7e, 0x11, 0x22, 0x33, 0x44,
                0x55, 0x01, 0xca, 0xcd, 0x7e
            ]
        );
    }

    /// `REQUEST_BOOTLOADER` (0x02) is the **flashing tool's** contract: the byte
    /// a device behind port 1 is addressed with when its firmware is about to be
    /// written, inside the same admin unit the port-level family travels as.
    #[test]
    fn the_bootloader_request_is_the_flashing_tools_command() {
        assert_eq!(CMD_REQUEST_BOOTLOADER, 0x02);
        assert_eq!(
            admin_unit(&[CMD_REQUEST_BOOTLOADER, 1]),
            [0xe8, 0x3f, 0x0b, 0x20, 0x4b, 0x58, 0x49, 0x44, 0x02, 0x01, 0x10, 0x2f, 0x7e]
        );
    }

    /// `RESTART_DEVICE` (0x05) is what a named bridge-port device's
    /// `restart_method: admin` sends: the device restarts its firmware rather
    /// than entering a bootloader.
    #[test]
    fn the_restart_device_command_is_the_firmwares_byte() {
        assert_eq!(CMD_RESTART_DEVICE, 0x05);
        assert_eq!(
            admin_unit(&[CMD_RESTART_DEVICE, 1]),
            [0xe8, 0x3f, 0x0b, 0x20, 0x4b, 0x58, 0x49, 0x44, 0x05, 0x01, 0x5d, 0x27, 0x7e]
        );
    }

    #[test]
    fn set_port_node_id_is_the_port_level_admin_unit() {
        // The port-level command bytes are the firmware's mux.h table
        // (0x02 REQUEST_BOOTLOADER / 0x03 SET_PORT_NODEID / 0x04 RESET_PORT /
        // 0x05 RESTART_DEVICE / 0x06 SET_PORT_TUNNEL) — pinned here so a drift
        // from the C side fails on the host too, not only in the real-library
        // probe.
        assert_eq!(CMD_REQUEST_BOOTLOADER, 0x02);
        assert_eq!(CMD_SET_PORT_NODEID, 0x03);
        assert_eq!(CMD_RESET_PORT, 0x04);
        assert_eq!(CMD_RESTART_DEVICE, 0x05);
        assert_eq!(CMD_SET_PORT_TUNNEL, 0x06);
        // The bytes on the wire, exactly: the admin listen id as a checked
        // header, then the `KXID` frame carrying `[0x03][index][id][flags]` —
        // the firmware's mux.h command table (SET_PORT_NODEID, payload
        // [0x03][index][id][flags]) spelled inside the stream frame.
        assert_eq!(
            set_port_node_id(0, 7, 0),
            [
                0xe8, 0x3f, 0x0d, 0x20, 0x4b, 0x58, 0x49, 0x44, 0x03, 0x00, 0x07, 0x00, 0x53, 0xc7,
                0x7e
            ]
        );
        // The flags byte is the port's own: MUX_PORT_UNITS turns the port to
        // unit mode, and the reserved bits the firmware masks off are ignored.
        assert_eq!(
            set_port_node_id(0, 8, MUX_PORT_UNITS),
            [
                0xe8, 0x3f, 0x0d, 0x20, 0x4b, 0x58, 0x49, 0x44, 0x03, 0x00, 0x08, 0x01, 0xc1, 0x86,
                0x7e
            ]
        );
        let unit = set_port_node_id(1, 0xff, MUX_PORT_UNITS);
        assert_eq!(
            unit.iter().filter(|&&b| b == MESSAGE_SYNC).count(),
            1,
            "no interior 0x7e in the header/payload/CRC"
        );
        // The frame behind the header: [len][seq]["KXID"]payload[crc16][sync],
        // so the four payload bytes sit right after the marker.
        assert_eq!(
            &unit[HEADER_BYTES + 1 + 1 + MAGIC.len()..unit.len() - 3],
            &[0x03, 0x01, 0xff, 0x01],
            "the payload is exactly [0x03][index][id][flags] behind the KXID marker"
        );
    }

    #[test]
    fn set_port_tunnel_is_the_port_level_admin_unit() {
        // The toggle's wire bytes, exactly, the way
        // [`set_port_node_id_is_the_port_level_admin_unit`] pins its own:
        // `[0x06][index]` after the admin listen id and the `KXID` marker, the
        // firmware's mux.h `MUX_CMD_SET_PORT_TUNNEL`. One index pinned whole so
        // the length byte, marker and CRC of this command are pinned too.
        assert_eq!(
            admin_unit(&[CMD_SET_PORT_TUNNEL, 3]),
            [0xe8, 0x3f, 0x0b, 0x20, 0x4b, 0x58, 0x49, 0x44, 0x06, 0x03, 0x54, 0x5d, 0x7e]
        );
    }

    #[test]
    fn an_admin_answer_gives_the_uuid_and_the_application() {
        let unit = admin_answer([0x11, 0x22, 0x33, 0x44, 0x55, 0x66], Some(0x01));
        let node = UnassignedNode::from_unit(&unit).expect("this is an answer");
        assert_eq!(node.uuid, [0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
        assert_eq!(node.application, Some(0x01));
        assert_eq!(node.application_name(), "Klipper");

        // CanBoot answers the same way and names itself.
        let node = UnassignedNode::from_unit(&admin_answer([0; 6], Some(0x11))).expect("an answer");
        assert_eq!(node.application_name(), "CanBoot");
    }

    #[test]
    fn the_uuid_is_read_most_significant_byte_first() {
        // The byte right after the response is the most significant one: read
        // the other way round, the two answers below would collapse.
        let node = UnassignedNode::from_unit(&admin_answer([0x01, 0, 0, 0, 0, 0], None)).unwrap();
        assert_eq!(node.uuid, [0x01, 0, 0, 0, 0, 0]);

        let node = UnassignedNode::from_unit(&admin_answer([0, 0, 0, 0, 0, 0x01], None)).unwrap();
        assert_eq!(node.uuid, [0, 0, 0, 0, 0, 0x01]);
    }

    #[test]
    fn an_answer_without_the_application_byte_is_klipper() {
        let unit = admin_answer([0x11, 0x22, 0x33, 0x44, 0x55, 0x66], None);
        let node = UnassignedNode::from_unit(&unit).expect("an answer");
        assert_eq!(node.application, None);
        assert_eq!(node.application_name(), "Klipper");
    }

    #[test]
    fn a_directly_wired_answer_names_no_port() {
        // The oldest shape there is — `[0x20][uuid]`, no application byte, no
        // path — and the one every board answers with: nothing to attribute, so
        // the board is the host's own wire's.
        let node = UnassignedNode::from_unit(&admin_answer([0x11; 6], None)).unwrap();
        assert_eq!(node.ports, Vec::<u8>::new());
    }

    #[test]
    fn the_application_byte_of_a_bridged_answer_is_read_before_its_path() {
        // `[0x20][uuid][0x01][0x81][0x00]`: CanBoot behind the answer's bridge
        // port 0. Both optional fields present, told apart by the top bit of the
        // byte that starts each — read the wrong way round, `0x01` would be a
        // path of one port and the answer would lose its application.
        let node = UnassignedNode::from_unit(&admin_answer_tail([0x11; 6], &[0x11, 0x81, 0x07]))
            .expect("an answer");
        assert_eq!(node.application, Some(0x11));
        assert_eq!(node.application_name(), "CanBoot");
        assert_eq!(node.ports, vec![0x07]);
    }

    #[test]
    fn a_bridged_answer_names_the_port_it_came_up_on() {
        // A board with no application byte — the shape the firmware's own
        // devices answer with — behind one bridge, which appended its port: the
        // byte after the UUID has its top bit set, so it is the path's head
        // (`0x80 | 1`), not an application.
        let node = UnassignedNode::from_unit(&admin_answer_tail(
            [0x0d, 0x1e, 0x2f, 0x3a, 0x4b, 0x5c],
            &[0x81, 0x00],
        ))
        .expect("an answer");
        assert_eq!(node.application, None);
        assert_eq!(node.application_name(), "Klipper");
        assert_eq!(node.ports, vec![0x00]);
    }

    #[test]
    fn a_chain_of_bridges_appends_its_ports_in_order() {
        // Two bridges passed the answer, innermost first: the count says two, and
        // the ports are read in the order they were appended — `0x82 0x00 0x02`
        // is "port 0 of the first, port 2 of the last", the chain the host
        // prints as `via ports 0>2`.
        let node = UnassignedNode::from_unit(&admin_answer_tail([0x11; 6], &[0x82, 0x00, 0x02]))
            .expect("an answer");
        assert_eq!(node.ports, vec![0x00, 0x02]);
        // The longest legal path is eight ports (`MAX_PATH_PORTS`), which is the
        // firmware's own bound (`mux.h` `MUX_PATH_MAX`).
        let mut longest = vec![0x80 | 8];
        longest.extend(0..8);
        let node =
            UnassignedNode::from_unit(&admin_answer_tail([0x11; 6], &longest)).expect("an answer");
        assert_eq!(node.ports, (0..8).collect::<Vec<u8>>());
    }

    #[test]
    fn a_path_that_does_not_parse_is_dropped_and_the_answer_kept() {
        // A path is the host's only record of where a board sits, so one that
        // cannot be read is no path at all — never half of one, and never a
        // panic. The UUID was still heard, so the board is still reported.
        for (what, tail) in [
            // A count of zero names no port.
            ("a count of zero", vec![0x80]),
            // Nine ports: past the firmware's bound of eight.
            ("a count past the bound", {
                let mut tail = vec![0x80 | 9];
                tail.extend(0..9);
                tail
            }),
            // A count of two with one port's byte behind it.
            ("a short path", vec![0x82, 0x00]),
            // A count of one with a byte too many.
            ("a long path", vec![0x81, 0x00, 0x01]),
        ] {
            let node = UnassignedNode::from_unit(&admin_answer_tail([0x11; 6], &tail))
                .unwrap_or_else(|| panic!("{what} is still an answer"));
            assert_eq!(node.uuid, [0x11; 6]);
            assert!(node.ports.is_empty(), "{what} names no port");
            assert_eq!(node.application, None, "{what} is no application byte");
        }
    }

    #[test]
    fn an_unknown_application_is_named_unknown() {
        let node = UnassignedNode::from_unit(&admin_answer([0; 6], Some(0x02))).unwrap();
        assert_eq!(node.application, Some(0x02));
        assert_eq!(node.application_name(), "Unknown");
    }

    #[test]
    fn an_all_zero_uuid_is_still_reported() {
        // The protocol has no “no UUID programmed” value, so the shape of the
        // unit is all that is checked, as it is for CAN.
        let node = UnassignedNode::from_unit(&admin_answer([0; 6], Some(0x01))).unwrap();
        assert_eq!(node.uuid, [0; 6]);
        assert_eq!(node.application_name(), "Klipper");
    }

    #[test]
    fn a_unit_that_is_not_an_answer_is_rejected() {
        // The query itself is addressed on the listen id, not the reply id.
        assert_eq!(UnassignedNode::from_unit(&discovery_query()), None);
        // A node's own assigned id is not an admin answer either.
        let mut foreign = encode_header(assign::arb_id(2).unwrap());
        foreign.extend_from_slice(&kxid_unit(&[RESP_NEED_NODEID, 1, 2, 3, 4, 5, 6, 0x01]));
        assert_eq!(UnassignedNode::from_unit(&foreign), None);
        // A truncated header.
        assert_eq!(UnassignedNode::from_unit(&discovery_query()[..1]), None);
        // Fewer than six UUID bytes after the response.
        let mut short = encode_header(ADMIN_REPLY);
        short.extend_from_slice(&kxid_unit(&[RESP_NEED_NODEID, 1, 2, 3]));
        assert_eq!(UnassignedNode::from_unit(&short), None);
        // A wrong sequence byte (with the CRC fixed, so the refusal is the
        // sequence, as it is for the handshake's ack).
        let mut bad_seq = admin_answer([0x11; 6], None);
        bad_seq[MESSAGE_HEADER_SIZE + MESSAGE_POS_SEQ] = 0x10;
        {
            let frame = &bad_seq[MESSAGE_HEADER_SIZE..];
            let len = frame[MESSAGE_POS_LEN] as usize;
            let crc = crate::crc16_ccitt!(&frame[..len - MESSAGE_TRAILER_SIZE]);
            bad_seq[MESSAGE_HEADER_SIZE + len - 3] = (crc >> 8) as u8;
            bad_seq[MESSAGE_HEADER_SIZE + len - 2] = crc as u8;
        }
        assert_eq!(UnassignedNode::from_unit(&bad_seq), None);
        // A response byte that is not RESP_NEED_NODEID.
        let mut bad_cmd = vec![0x21];
        bad_cmd.extend_from_slice(&[0x11; 6]);
        let mut unit_bad_cmd = encode_header(ADMIN_REPLY);
        unit_bad_cmd.extend_from_slice(&kxid_unit(&bad_cmd));
        assert_eq!(UnassignedNode::from_unit(&unit_bad_cmd), None);
        // A bad CRC.
        let mut bad_crc = admin_answer([0x11; 6], None);
        let crc_at = bad_crc.len() - 3;
        bad_crc[crc_at] ^= 0xff;
        assert_eq!(UnassignedNode::from_unit(&bad_crc), None);
        // A bad trailer SYNC.
        let mut bad_trailer = admin_answer([0x11; 6], None);
        *bad_trailer.last_mut().unwrap() = 0x00;
        assert_eq!(UnassignedNode::from_unit(&bad_trailer), None);
        // The length byte claims more bytes than the unit holds: the frame is
        // not self-consistent, so it is refused rather than guessed at.
        let mut stretched = admin_answer([0x11; 6], None);
        stretched[MESSAGE_HEADER_SIZE + MESSAGE_POS_LEN] += 1;
        assert_eq!(UnassignedNode::from_unit(&stretched), None);
        // Two application bytes: only one is legal, so this is not an answer.
        let mut two_app = vec![RESP_NEED_NODEID];
        two_app.extend_from_slice(&[0x11; 6]);
        two_app.extend_from_slice(&[0x01, 0x11]);
        let mut unit_two_app = encode_header(ADMIN_REPLY);
        unit_two_app.extend_from_slice(&kxid_unit(&two_app));
        assert_eq!(UnassignedNode::from_unit(&unit_two_app), None);
    }

    #[test]
    fn the_scan_deduplicates_a_uuid() {
        let uuid = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        let mut scan = UnassignedScan::new();
        assert_eq!(
            scan.feed(&admin_answer(uuid, Some(0x01))).unwrap().uuid,
            uuid
        );
        assert_eq!(
            scan.feed(&admin_answer(uuid, Some(0x01))),
            None,
            "the second frame of a UUID is not news"
        );
        assert!(scan.feed(&admin_answer([0xaa; 6], Some(0x11))).is_some());
        assert_eq!(scan.feed(&discovery_query()), None, "not an answer");

        assert_eq!(scan.total(), 2);
        assert_eq!(
            scan.nodes()
                .iter()
                .map(|node| node.uuid)
                .collect::<Vec<_>>(),
            vec![uuid, [0xaa; 6]],
            "listed once each, in the order they were first heard"
        );
    }

    #[test]
    fn garbage_without_a_unit_shape_does_not_enter_the_scan() {
        let mut scan = UnassignedScan::new();
        // A bare header with no frame, and bytes that never form a unit: the
        // extractor resynchronises past them, and the scan never lists them.
        assert_eq!(scan.feed(&encode_header(ADMIN_REPLY)), None);
        assert_eq!(scan.feed(b"no header here"), None);
        assert_eq!(scan.total(), 0);
    }

    /// Drive one scan against a pty and return `(total, reported)`.
    ///
    /// The scan runs on the slave side (a real [`SerialDevice`]); the caller's
    /// `responses` are written on the master once the discovery query has been
    /// seen, and the helper then waits for the window to close.
    fn run_scan_on_pty(
        timeout: Duration,
        responses: impl FnOnce(&File),
    ) -> (usize, Vec<UnassignedNode>) {
        let pty = Pty::new();
        let device = SerialDevice::open(&pty.path, DEFAULT_BAUD).expect("a pty opens");
        let handle = thread::spawn(move || {
            let mut device = device;
            let mut reported = Vec::new();
            let total = serial_query_unassigned(&mut device, timeout, |node| reported.push(node))
                .expect("the pty does not fail");
            (total, reported)
        });

        let query = discovery_query();
        let seen = read_n(&pty.master, query.len());
        assert_eq!(
            seen, query,
            "the scan must send the discovery query verbatim"
        );
        responses(&pty.master);

        handle.join().expect("scan thread join")
    }

    #[test]
    fn a_scan_collects_one_answer() {
        let uuid = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        let (total, reported) = run_scan_on_pty(Duration::from_millis(500), |master| {
            master_write(master, &admin_answer(uuid, Some(0x01)));
        });
        assert_eq!(total, 1);
        assert_eq!(reported.len(), 1);
        assert_eq!(reported[0].uuid, uuid);
        assert_eq!(reported[0].application_name(), "Klipper");
    }

    #[test]
    fn a_scan_collects_two_answers_and_deduplicates() {
        let uuid = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        let (total, reported) = run_scan_on_pty(Duration::from_millis(500), |master| {
            // The firmware retries its answer, so the same board is heard
            // again — and a second board answers too.
            master_write(master, &admin_answer(uuid, Some(0x01)));
            master_write(master, &admin_answer(uuid, Some(0x01)));
            master_write(master, &admin_answer([0xaa; 6], Some(0x11)));
        });
        assert_eq!(total, 2);
        assert_eq!(
            reported.iter().map(|node| node.uuid).collect::<Vec<_>>(),
            vec![uuid, [0xaa; 6]],
            "each board once, in the order it was first heard"
        );
    }

    #[test]
    fn a_scan_with_no_answer_finds_nothing() {
        // The query still goes out; the short window then closes on nothing.
        let (total, reported) = run_scan_on_pty(Duration::from_millis(150), |_| {});
        assert_eq!(total, 0);
        assert!(reported.is_empty());
    }

    #[test]
    fn a_scan_ignores_non_admin_units() {
        // A unit addressed to an assigned node (its own arbitration id, an
        // ordinary identify frame) sits in front of the answer: the extractor
        // cuts it whole by its length, and the scan passes it by.
        let mut regular = encode_header(assign::arb_id(2).unwrap());
        regular.extend_from_slice(&Frame::encode(1, b"identify"));
        let (total, reported) = run_scan_on_pty(Duration::from_millis(500), |master| {
            master_write(master, &regular);
            master_write(master, &admin_answer([0x11; 6], None));
        });
        assert_eq!(total, 1);
        assert_eq!(reported[0].uuid, [0x11; 6]);
    }

    // -----------------------------------------------------------------------
    // The tunnel unit: golden vectors (the byte-for-byte contract with the
    // firmware's mux.c, which mirrors these), the shared cut rule's bad cases,
    // and "a tunnel unit never eats the stream" at the reader.
    // -----------------------------------------------------------------------

    /// Golden vector 1 — a Connect request: 8 bytes, word count 0.
    ///
    /// `01 88 11 00 f1 7c 99 03` — tag, `CMD_CONNECT`, word count 0,
    /// `crc16_ccitt(11 00) = f1 7c` stored little-endian, trailer `99 03`.
    const TUNNEL_CONNECT: [u8; 8] = [0x01, 0x88, 0x11, 0x00, 0xf1, 0x7c, 0x99, 0x03];

    /// Golden vector 2 — a Send Block request (`CMD_RX_BLOCK`): 76 bytes,
    /// word count 17, carrying a 64-byte block at address `00 10 00 00`.
    ///
    /// One block, one unit, never split — a split would drop outer framing
    /// into the middle of this frame.
    const TUNNEL_SEND_BLOCK: [u8; 76] = [
        0x01, 0x88, 0x12, 0x11, // tag, CMD_RX_BLOCK, word count 17
        0x00, 0x10, 0x00, 0x00, // block address (little-endian)
        // the 64-byte block, bytes 0x00..=0x3f:
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
        0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
        0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c,
        0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
        0x3c, 0x3d, 0x3e, 0x3f, 0xda, 0x1e, 0x99, 0x03, // crc16 (little-endian), trailer
    ];

    /// Golden vector 3 — the device's ACK for that block: 16 bytes, word
    /// count 2, `RESPONSE_ACK` acknowledging `CMD_RX_BLOCK` — the
    /// device→host direction, headed by `reply_arb_id`.
    const TUNNEL_ACK: [u8; 16] = [
        0x01, 0x88, 0xa0, 0x02, // tag, RESPONSE_ACK, word count 2
        0x12, 0x00, 0x00, 0x00, // acknowledged command: CMD_RX_BLOCK
        0x00, 0x10, 0x00, 0x00, // block address (little-endian)
        0xbc, 0xdc, 0x99, 0x03, // crc16 (little-endian), trailer
    ];

    /// Whether the block carries the CRC katapult computes over its own bytes
    /// (`crc16_ccitt` from byte 2 to the trailer). The tunnel never checks
    /// this — the golden vectors must still be legal blocks.
    fn inner_crc_ok(block: &[u8]) -> bool {
        let stored = u16::from_le_bytes([block[block.len() - 4], block[block.len() - 3]]);
        crate::crc16_ccitt!(&block[2..block.len() - 4]) == stored
    }

    /// Attach `mcu_id` over `pty` with every wait bounded: the handshake
    /// bytes are read within a window, so a broken attach fails the test
    /// instead of blocking it.
    fn attach_within(pty: &Pty, mcu_id: u32) -> Arc<SharedSerialHandle> {
        let request = assignment_request(mcu_id).expect("a valid mcu id");
        let path = pty.path.clone();
        let thread =
            thread::spawn(move || attach(&path, DEFAULT_BAUD, mcu_id, ASSIGN_ATTACH_TIMEOUT, None));
        let seen = read_n_within(&pty.master, request.len(), Duration::from_secs(2));
        assert_eq!(
            seen, request,
            "attach must send the assignment request verbatim"
        );
        master_write(
            &pty.master,
            &assignment_reply(mcu_id).expect("a valid mcu id"),
        );
        thread
            .join()
            .expect("attach thread")
            .expect("attach succeeds")
    }

    #[test]
    fn a_tunnel_golden_connect_block_round_trips_in_one_unit() {
        assert!(
            inner_crc_ok(&TUNNEL_CONNECT),
            "the vector's CRC must be right"
        );
        let arb = assign::arb_id(1).expect("a valid mcu id");
        let unit = tunnel_unit(arb, &TUNNEL_CONNECT).expect("a whole block");

        // Out: the header, then the block verbatim — nothing added, split or
        // rewritten.
        let mut want = encode_header(arb);
        want.extend_from_slice(&TUNNEL_CONNECT);
        assert_eq!(unit, want);
        assert_eq!(
            unit.len(),
            HEADER_BYTES + 8,
            "word count 0 → an 8-byte frame"
        );

        // In: cut whole, same id, same bytes.
        let mut stream = UnitStream::new();
        stream.feed(&unit);
        assert_eq!(stream.take_unit_with_arb(), Some((arb, unit)));
    }

    #[test]
    fn a_tunnel_golden_send_block_is_cut_as_one_whole_unit() {
        assert!(
            inner_crc_ok(&TUNNEL_SEND_BLOCK),
            "the vector's CRC must be right"
        );
        assert_eq!(TUNNEL_SEND_BLOCK[TUNNEL_POS_WORDCNT], 17, "word count");
        assert_eq!(tunnel_frame_len(17), TUNNEL_SEND_BLOCK.len());

        let arb = assign::arb_id(1).expect("a valid mcu id");
        let unit = tunnel_unit(arb, &TUNNEL_SEND_BLOCK).expect("a whole block");

        let mut stream = UnitStream::new();
        stream.feed(&unit);
        // One take, the whole 76-byte frame: never split, never resynchronised.
        assert_eq!(stream.take_unit_with_arb(), Some((arb, unit)));
        // And nothing is left behind: the unit was consumed exactly.
        assert_eq!(stream.take_unit_with_arb(), None);
    }

    #[test]
    fn a_tunnel_golden_ack_round_trips_in_the_device_to_host_direction() {
        assert!(inner_crc_ok(&TUNNEL_ACK), "the vector's CRC must be right");
        let arb = assign::reply_arb_id(4).expect("a valid mcu id");
        let unit = tunnel_unit(arb, &TUNNEL_ACK).expect("a whole block");
        let mut want = encode_header(arb);
        want.extend_from_slice(&TUNNEL_ACK);
        assert_eq!(unit, want);

        let mut stream = UnitStream::new();
        stream.feed(&unit);
        assert_eq!(stream.take_unit_with_arb(), Some((arb, unit)));
    }

    #[test]
    fn the_tunnel_cut_rule_is_one_shared_contract_and_junk_stays_junk() {
        // The three golden vectors, each cut whole by its word count.
        assert_eq!(cut_unit(&TUNNEL_CONNECT), UnitCut::Tunnel { frame: 8 });
        assert_eq!(cut_unit(&TUNNEL_SEND_BLOCK), UnitCut::Tunnel { frame: 76 });
        assert_eq!(cut_unit(&TUNNEL_ACK), UnitCut::Tunnel { frame: 16 });

        // The size limits: word count 30 fills exactly the maximum frame…
        let mut at_limit = vec![0x01u8, 0x88, 0x12, 30];
        at_limit.extend(std::iter::repeat(0u8).take(TUNNEL_FRAME_MAX - 4));
        assert_eq!(
            cut_unit(&at_limit),
            UnitCut::Tunnel {
                frame: TUNNEL_FRAME_MAX
            }
        );
        // …and 31 would make 132 bytes: past the limit, so junk.
        assert_eq!(cut_unit(&[0x01, 0x88, 0x12, 31]), UnitCut::Junk);

        // `01` without `88`: the old junk path, byte for byte.
        assert_eq!(cut_unit(&[0x01, 0x5a]), UnitCut::Junk);

        // Too few bytes to tell tunnel from junk: each of these waits.
        let partials: [&[u8]; 4] = [&[], &[0x01], &[0x01, 0x88], &[0x01, 0x88, 0x12]];
        for partial in partials {
            assert_eq!(cut_unit(partial), UnitCut::NeedMore, "at {partial:02x?}");
        }

        // An ordinary frame still cuts by its length byte, and impossible
        // klipper lengths are still junk.
        assert_eq!(cut_unit(&assignment_frame()), UnitCut::Klipper { frame: 9 });
        assert_eq!(cut_unit(&[0x04, 0x20]), UnitCut::Junk); // below MESSAGE_MIN
        assert_eq!(cut_unit(&[0x41, 0x20]), UnitCut::Junk); // above MESSAGE_MAX
    }

    #[test]
    fn a_tunnel_unit_waits_out_a_short_buffer_and_then_comes_out_whole() {
        let arb = assign::arb_id(7).expect("a valid mcu id");
        let unit = tunnel_unit(arb, &TUNNEL_SEND_BLOCK).expect("a whole block");
        let mut stream = UnitStream::new();

        // One byte at a time: every prefix is short, so nothing is cut and
        // nothing is resynchronised away.
        for fed in 1..unit.len() {
            stream.feed(&unit[fed - 1..fed]);
            assert_eq!(stream.take_unit_with_arb(), None, "after {fed} bytes");
        }
        // The last byte completes it: the whole unit, in one piece.
        stream.feed(&unit[unit.len() - 1..]);
        assert_eq!(stream.take_unit_with_arb(), Some((arb, unit)));
    }

    #[test]
    fn a_tunnel_shaped_junk_resynchronises_and_the_unit_behind_it_survives() {
        let arb = assign::arb_id(2).expect("a valid mcu id");
        let mut behind = encode_header(arb);
        behind.extend_from_slice(&assignment_frame());

        // Both junk flavours: `01` without `88`, and a word count past 128.
        let junk_cases: [&[u8]; 2] = [
            &[0x01, 0x5a, 0x11, 0x00],
            &[0x01, 0x88, 0x12, 0x1f, 0x00, 0x00, 0x00, 0x00],
        ];
        for junk in junk_cases {
            let mut wire = encode_header(arb);
            wire.extend_from_slice(junk);
            wire.push(MESSAGE_SYNC); // what the junk path resynchronises on
            wire.extend_from_slice(&behind);

            let mut stream = UnitStream::new();
            stream.feed(&wire);
            // The junk never becomes a unit; the stream resynchronises and the
            // unit behind it comes out whole.
            assert_eq!(
                stream.take_unit_with_arb(),
                Some((arb, behind.clone())),
                "for junk {junk:02x?}"
            );
            assert_eq!(stream.take_unit_with_arb(), None, "for junk {junk:02x?}");
        }
    }

    #[test]
    fn an_invalid_header_never_reaches_the_tunnel_tag() {
        let arb = assign::arb_id(2).expect("a valid mcu id");
        // A framing-broken header in front of a perfect tunnel block: the
        // block never enters the tunnel rule, so no unit is cut from it.
        let mut wire = vec![0x00, 0x01]; // first byte misses the 0x80 framing bit
        wire.extend_from_slice(&TUNNEL_CONNECT);
        let mut stream = UnitStream::new();
        stream.feed(&wire);
        assert_eq!(stream.take_unit_with_arb(), None);

        // The stream recovers at the next SYNC and cuts the unit behind it.
        let mut behind = encode_header(arb);
        behind.extend_from_slice(&assignment_frame());
        stream.feed(&[MESSAGE_SYNC]);
        stream.feed(&behind);
        assert_eq!(stream.take_unit_with_arb(), Some((arb, behind)));
    }

    #[test]
    fn a_tunnel_unit_is_consumed_whole_and_the_next_unit_is_dispatched() {
        let pty = Pty::new();
        let handle4 = attach_within(&pty, 4);

        // A device→host tunnel unit nobody listens for, then an ordinary unit.
        let mut wire = device_unit(4, &TUNNEL_ACK);
        wire.extend_from_slice(&device_unit(4, &Frame::encode(7, b"after")));
        master_write(&pty.master, &wire);

        // The tunnel unit was cut and dropped (no listener), and the unit
        // behind it still arrived: the reader never fell into `drop_to_sync`,
        // which would have eaten through this whole run to the frame's trailer.
        assert!(
            wait_for_frame(
                &handle4.slot,
                &Frame::new(7, b"after".to_vec()),
                Duration::from_secs(2)
            ),
            "the unit behind the tunnel unit must be dispatched"
        );
    }

    #[test]
    fn a_tunnel_unit_is_delivered_whole_to_the_listener_of_its_arb() {
        let pty = Pty::new();
        let handle4 = attach_within(&pty, 4);
        let reply = assign::reply_arb_id(4).expect("a valid mcu id");

        let (sink, frames) = TunnelSink::channel();
        handle4.set_tunnel_sink(reply, sink);

        // The whole block, byte for byte: the listener reassembles nothing.
        master_write(&pty.master, &device_unit(4, &TUNNEL_SEND_BLOCK));
        assert_eq!(
            frames
                .recv_timeout(Duration::from_secs(2))
                .expect("the block is delivered"),
            TUNNEL_SEND_BLOCK.to_vec(),
            "the listener gets the whole katapult block"
        );

        // An ordinary klipper unit on the same port is untouched: it still
        // reaches its id's slot, not the tunnel listener.
        master_write(&pty.master, &device_unit(4, &Frame::encode(7, b"after")));
        assert!(
            wait_for_frame(
                &handle4.slot,
                &Frame::new(7, b"after".to_vec()),
                Duration::from_secs(2)
            ),
            "the klipper unit is routed as before"
        );
        assert!(
            frames.recv_timeout(Duration::from_millis(200)).is_err(),
            "a klipper unit is not handed to the tunnel listener"
        );
    }

    #[test]
    fn a_tunnel_listener_is_keyed_by_arb_and_leaves_by_being_cleared() {
        let pty = Pty::new();
        let handle4 = attach_within(&pty, 4);
        let reply = assign::reply_arb_id(4).expect("a valid mcu id");

        // A listener under another id gets nothing: the registry keys by the
        // arbitration id the unit is headed with.
        let (other, other_frames) = TunnelSink::channel();
        handle4.set_tunnel_sink(assign::arb_id(9).expect("a valid mcu id"), other);
        master_write(&pty.master, &device_unit(4, &TUNNEL_ACK));
        assert!(
            other_frames
                .recv_timeout(Duration::from_millis(200))
                .is_err(),
            "another arb's listener gets nothing"
        );

        // The registered listener gets the block; clearing stops delivery.
        let (sink, frames) = TunnelSink::channel();
        handle4.set_tunnel_sink(reply, sink);
        master_write(&pty.master, &device_unit(4, &TUNNEL_ACK));
        assert!(
            frames.recv_timeout(Duration::from_secs(2)).is_ok(),
            "the registered listener takes the block"
        );
        handle4.clear_tunnel_sink(reply);
        master_write(&pty.master, &device_unit(4, &TUNNEL_CONNECT));
        assert!(
            frames.recv_timeout(Duration::from_millis(200)).is_err(),
            "a cleared listener gets nothing"
        );

        // A dropped receiver unregisters itself, and the stream keeps working:
        // the ordinary unit behind the dropped listener's block still arrives.
        let (dropped, dropped_frames) = TunnelSink::channel();
        handle4.set_tunnel_sink(reply, dropped);
        drop(dropped_frames);
        master_write(&pty.master, &device_unit(4, &TUNNEL_CONNECT));
        master_write(&pty.master, &device_unit(4, &Frame::encode(8, b"alive")));
        assert!(
            wait_for_frame(
                &handle4.slot,
                &Frame::new(8, b"alive".to_vec()),
                Duration::from_secs(2)
            ),
            "the port keeps dispatching after a listener disappears"
        );
    }

    #[test]
    fn the_tunnel_registry_drops_a_dead_listener_and_keeps_the_rest() {
        let registry = TunnelRegistry::new();
        let arb = assign::reply_arb_id(4).expect("a valid mcu id");

        let (sink, frames) = TunnelSink::channel();
        registry.register(arb, sink);
        assert!(
            registry.deliver(arb, &TUNNEL_ACK),
            "a live listener takes the block"
        );
        assert_eq!(
            frames.recv_timeout(Duration::from_secs(1)).unwrap(),
            TUNNEL_ACK.to_vec()
        );

        // The receiver goes away: the next delivery fails and unregisters it,
        // so a later one does not even try.
        drop(frames);
        assert!(
            !registry.deliver(arb, &TUNNEL_ACK),
            "a dead listener is not delivered to"
        );
        assert!(
            !registry.deliver(arb, &TUNNEL_ACK),
            "and it is dropped, not retried"
        );

        // Clearing is independent of delivery, and a fresh registration works.
        let (sink, frames) = TunnelSink::channel();
        registry.register(arb, sink);
        registry.clear(arb);
        assert!(
            !registry.deliver(arb, &TUNNEL_ACK),
            "cleared means no listener"
        );
        assert!(frames.recv_timeout(Duration::from_millis(200)).is_err());
    }

    #[test]
    fn send_tunnel_writes_the_header_and_block_verbatim() {
        let pty = Pty::new();
        let handle4 = attach_within(&pty, 4);

        handle4
            .send_tunnel(&TUNNEL_SEND_BLOCK)
            .expect("a whole block");
        let want = tunnel_unit(assign::arb_id(4).unwrap(), &TUNNEL_SEND_BLOCK).unwrap();
        assert_eq!(read_n(&pty.master, want.len()), want);
    }

    #[test]
    fn send_tunnel_refuses_a_block_that_is_not_one_whole_message() {
        let pty = Pty::new();
        let handle4 = attach_within(&pty, 4);

        // Right tag, but word count 31 would claim 132 bytes: over the limit.
        let over = [0x01, 0x88, 0x12, 0x1f, 0x00, 0x00, 0x00, 0x00];
        let err = handle4.send_tunnel(&over).expect_err("must be refused");
        assert!(matches!(err, InterfaceError::SendError(_)), "{err:?}");
        assert!(
            !readable_within(&pty.master, Duration::from_millis(150)),
            "a refused block must write nothing"
        );
    }

    #[test]
    fn the_tunnel_unit_checks_the_frame_shape_but_not_the_inner_crc_or_cmd() {
        // Right shape, deliberately wrong CRC, an unlisted command byte: the
        // tunnel wraps bytes, the consumer checks them.
        let mut block = [
            0x01, 0x88, 0xf3, 0x01, // tag, cmd, word count 1
            0xde, 0xad, 0xbe, 0xef, 0x00, 0x00, 0x99, 0x03,
        ];
        assert!(!inner_crc_ok(&block), "the CRC is wrong on purpose");
        let arb = assign::arb_id(3).expect("a valid mcu id");
        assert_eq!(
            tunnel_unit(arb, &block).map(|unit| unit[HEADER_BYTES..].to_vec()),
            Some(block.to_vec())
        );

        // Shape is checked, though: nothing but one self-delimiting block gets
        // out.
        assert_eq!(
            tunnel_unit(arb, &block[..11]),
            None,
            "shorter than its own word count"
        );
        block[TUNNEL_POS_WORDCNT] = 30; // would claim 128 bytes
        assert_eq!(
            tunnel_unit(arb, &block),
            None,
            "longer than its own word count"
        );
        block[TUNNEL_POS_WORDCNT] = 31; // would claim 132: past the limit
        assert_eq!(tunnel_unit(arb, &block), None, "over the size limit");
        assert_eq!(
            tunnel_unit(arb, &[0x02, 0x88, 0x11, 0x00, 0x00, 0x00, 0x99, 0x03]),
            None,
            "wrong tag"
        );
        assert_eq!(
            tunnel_unit(HEADER_RESERVED_FIRST, &TUNNEL_CONNECT),
            None,
            "a reserved id names no address"
        );
    }
}

// ===========================================================================
// T1 双重封装原型通路：katapult 字节流裹进 klipper 帧载荷
// ===========================================================================
//
// 主机把 katapult 块按 klipper 单帧载荷上限（59 B）切片、裹成帧再加 2 字节
// 单元头发到线上；桥在 legacy 口剥掉单元头（`bridge_attach.rs:400`：
// “the bridge strips the header going down”），完整 klipper 帧逐字节落到设备；
// 设备侧由 katapult 的 `command_find_block`
// （`third_party/katapult/src/command.c:99-141`，本模块忠实移植——结构检查
// 顺序、resync 丢到下一个 `0x01`、`CF_NEED_VALID` 一次性 NACK 均按 C 语义）
// 逐字节吃这条流。四类载荷各给逐字节判定。
#[cfg(test)]
mod katapult_dual_encap_tests {
    use super::*;

    // katapult 块帧常量：`third_party/katapult/src/command.h:39-56`。
    const K_MIN: usize = 8; // MESSAGE_MIN
    const K_MAX: usize = 128; // MESSAGE_MAX
    const K_STX1: u8 = 0x01; // MESSAGE_STX1
    const K_STX2: u8 = 0x88; // MESSAGE_STX2
    const K_SYNC2: u8 = 0x99; // MESSAGE_SYNC2（尾 0x99，command.h:53）
    const K_SYNC: u8 = 0x03; // MESSAGE_SYNC（尾 0x03，command.h:54）
    const K_TRAILER: usize = 4; // MESSAGE_TRAILER_SIZE
    const CMD_CONNECT: u8 = 0x11; // command.h:14
    const CMD_RX_BLOCK: u8 = 0x12; // command.h:15（Send Block，host→设备）
                                   // `command.c:95`：`enum { CF_NEED_SYNC=1<<0, CF_NEED_VALID=1<<1 };`
    const CF_NEED_SYNC: u8 = 1 << 0;
    const CF_NEED_VALID: u8 = 1 << 1;

    /// klipper 单帧载荷上限（`frame.rs:26`：`MESSAGE_MAX - MESSAGE_MIN`）。
    const PAYLOAD_MAX: usize = crate::core::klippy::frame::MESSAGE_PAYLOAD_MAX;
    /// 外层帧尾的 SYNC（`frame.rs:45`）。
    const FRAME_SYNC: u8 = crate::core::klippy::frame::MESSAGE_SYNC;

    /// 本端口（裹 katapult 流的那个 MCU）与夹在混合流里的“其他 MCU”。
    fn our_arb() -> u32 {
        assign::arb_id(4).expect("id 4 has an arbitration id")
    }
    fn foreign_arb() -> u32 {
        assign::arb_id(9).expect("id 9 has an arbitration id")
    }

    // --------------------------------------------------------------------
    // 主机侧：katapult 块的构造与 klipper 帧封装
    // --------------------------------------------------------------------

    /// 一个 katapult 块帧：`<01><88><cmd><word_cnt><payload(4B倍数)>
    /// <crc16-LE><99><03>`；CRC 覆盖 `[cmd][word_cnt][payload]`
    /// （`command.c:120`：`crc16_ccitt(buf+2, msglen-4-2)`），低字节在前
    /// （`command.h:50` 的 `MESSAGE_TRAILER_CRC = 4`）。
    fn katapult_block(cmd: u8, payload: &[u8]) -> Vec<u8> {
        assert_eq!(payload.len() % 4, 0, "payload 必须按 word 对齐");
        let word_cnt = u8::try_from(payload.len() / 4).expect("word count fits a byte");
        let mut covered = vec![cmd, word_cnt];
        covered.extend_from_slice(payload);
        let crc = crate::crc16_ccitt!(&covered);
        let mut out = vec![K_STX1, K_STX2, cmd, word_cnt];
        out.extend_from_slice(payload);
        out.push(crc as u8);
        out.push((crc >> 8) as u8);
        out.push(K_SYNC2);
        out.push(K_SYNC);
        assert_eq!(out.len(), out[3] as usize * 4 + 8, "msglen = word_cnt*4+8");
        assert!((K_MIN..=K_MAX).contains(&out.len()));
        out
    }

    /// 8 B Connect 控制帧（word_cnt = 0，msglen = 8 = MESSAGE_MIN）。
    fn connect_block() -> Vec<u8> {
        let block = katapult_block(CMD_CONNECT, &[]);
        assert_eq!(block.len(), 8);
        block
    }

    /// 76 B Send Block：地址 word + 16 个数据 word = 68 B 载荷，
    /// `msglen = 17*4+8 = 76`；数据是真 64 B，CRC 真算。
    fn send_block(addr: u32) -> Vec<u8> {
        let data: [u8; 64] = std::array::from_fn(|i| b'A' + (i % 26) as u8);
        let mut payload = addr.to_le_bytes().to_vec();
        payload.extend_from_slice(&data);
        let block = katapult_block(CMD_RX_BLOCK, &payload);
        assert_eq!(block.len(), 76);
        block
    }

    /// 56 B 块（word_cnt = 12）：整段能落进一个 59 B 外层载荷，
    /// 用来把后面的小帧顶过切点。
    fn block_56() -> Vec<u8> {
        let data: [u8; 44] = std::array::from_fn(|i| b'a' + (i % 26) as u8);
        let mut payload = 0x2000u32.to_le_bytes().to_vec();
        payload.extend_from_slice(&data);
        let block = katapult_block(CMD_RX_BLOCK, &payload);
        assert_eq!(block.len(), 56);
        block
    }

    /// 主机侧封装：katapult 字节流按 59 B 载荷切片，每片裹成
    /// `[len][seq|0x10][payload][crc16][0x7e]`（`frame.rs:17-47`），
    /// 前面加 2 字节单元头——共享线上发出的单元序列。
    fn host_units(arb: u32, stream: &[u8]) -> Vec<Vec<u8>> {
        stream
            .chunks(PAYLOAD_MAX)
            .enumerate()
            .map(|(i, chunk)| {
                let mut unit = encode_header(arb);
                unit.extend_from_slice(&Frame::encode(i as u8, chunk));
                unit
            })
            .collect()
    }

    // --------------------------------------------------------------------
    // 桥的 legacy 下行
    // --------------------------------------------------------------------

    /// 逐单元剥掉 2 字节单元头、帧字节原样往下落（`bridge_attach.rs:400`）。
    /// `route = Some(ours)` 只下发本口单元（其他 MCU 的被路由走）；`None`
    /// 模拟最坏情况——整条线的帧都落到设备。
    fn bridge_down(wire: &[u8], route: Option<u32>) -> Vec<u8> {
        let mut out = Vec::new();
        let mut pos = 0;
        while pos < wire.len() {
            assert!(wire.len() - pos >= 3, "unit at {pos}: header + length byte");
            let arb = match decode_header(&wire[pos..pos + 2]) {
                HeaderScan::Complete { arb, .. } => arb,
                other => panic!("unit at {pos}: bad header {other:?}"),
            };
            let frame_len = wire[pos + 2] as usize;
            assert!(
                (5..=64).contains(&frame_len),
                "unit at {pos}: frame length {frame_len}"
            );
            let unit_len = 2 + frame_len;
            assert!(pos + unit_len <= wire.len(), "unit at {pos}: truncated");
            let deliver = match route {
                Some(want) => want == arb,
                None => true,
            };
            if deliver {
                out.extend_from_slice(&wire[pos + 2..pos + unit_len]);
            }
            pos += unit_len;
        }
        out
    }

    // --------------------------------------------------------------------
    // 设备侧：`command_find_block` 的忠实移植与逐字节驱动
    // --------------------------------------------------------------------

    /// 一次 `command_find_block` 调用（C 的返回值 + `pop_count` 出参）。
    #[derive(Debug)]
    struct Call {
        ret: i8,
        pop: usize,
        nacked: bool,
    }

    /// `command_find_block`（`third_party/katapult/src/command.c:99-141`）的
    /// 忠实移植。特别保留：错误时缓冲头恰是 `0x01` 则 `memchr` 命中自身、
    /// `pop_count = 0`、缓冲永不推进——固件的调用方（`serial_irq.c:71-86`
    /// 的 `console_task`）不做 0 推进保护，原地空转即停摆。
    #[derive(Default)]
    struct KatapultParser {
        /// C 的 `static uint8_t sync_state`，初始 0（`command.c:100`）。
        sync_state: u8,
    }

    impl KatapultParser {
        fn find_block(&mut self, buf: &[u8]) -> Call {
            // `command.c:101-102`
            if !buf.is_empty() && self.sync_state & CF_NEED_SYNC != 0 {
                return self.need_sync(buf);
            }
            // `command.c:103-104`
            if buf.len() < K_MIN {
                return Call {
                    ret: 0,
                    pop: 0,
                    nacked: false,
                };
            }
            // `command.c:105-106`
            if buf[0] != K_STX1 || buf[1] != K_STX2 {
                return self.error(buf);
            }
            // `command.c:107-108` —— msglen = word_cnt*4+8，须 8..=128
            let msglen = buf[3] as usize * 4 + 8;
            if !(K_MIN..=K_MAX).contains(&msglen) {
                return self.error(buf);
            }
            // `command.c:109-110`
            if buf.len() < msglen {
                return Call {
                    ret: 0,
                    pop: 0,
                    nacked: false,
                };
            }
            // `command.c:111-112` —— 尾部 0x99 / 0x03
            if buf[msglen - 2] != K_SYNC2 || buf[msglen - 1] != K_SYNC {
                return self.error(buf);
            }
            // `command.c:113-117` —— CRC16-LE，覆盖 buf+2 起 msglen-4-2 字节
            let msgcrc = buf[msglen - 4] as u16 | (buf[msglen - 3] as u16) << 8;
            let crc = crate::crc16_ccitt!(&buf[2..msglen - K_TRAILER]);
            if crc != msgcrc {
                return self.error(buf);
            }
            // `command.c:118` —— 有效块清 CF_NEED_VALID（重 arm 一次性 NACK）
            self.sync_state &= !CF_NEED_VALID;
            // `command.c:119`
            Call {
                ret: 1,
                pop: msglen,
                nacked: false,
            }
        }

        fn error(&mut self, buf: &[u8]) -> Call {
            // `command.c:135`
            self.sync_state |= CF_NEED_SYNC;
            self.need_sync(buf)
        }

        /// `command.c:136-148`：丢到下一个 `0x01`（命中才清 CF_NEED_SYNC，
        /// 否则整段丢弃）；`CF_NEED_VALID` 未置时发一次性 NACK。
        fn need_sync(&mut self, buf: &[u8]) -> Call {
            let pop = match buf.iter().position(|&b| b == K_STX1) {
                Some(at) => {
                    self.sync_state &= !CF_NEED_SYNC;
                    at
                }
                None => buf.len(),
            };
            if self.sync_state & CF_NEED_VALID != 0 {
                return Call {
                    ret: -1,
                    pop,
                    nacked: false,
                };
            }
            self.sync_state |= CF_NEED_VALID;
            Call {
                ret: -1,
                pop,
                nacked: true,
            }
        }
    }

    /// 设备侧接收缓冲与调用节奏：按 `console_task`（`serial_irq.c:71-86`）
    /// ——每收一字节就反复 `command_find_block` 直到 need_more；`ret≠0` 且
    /// `pop=0` 连续两次即 C 侧原地空转（缓冲头是错误的 `0x01`，`memchr`
    /// 永远命中自身），记录停摆位置，此后字节只进缓冲不再解析——与固件
    /// 一致：那些字节永远轮不到。
    #[derive(Default)]
    struct DeviceSide {
        parser: KatapultParser,
        buf: Vec<u8>,
        recovered: Vec<Vec<u8>>,
        nacks: usize,
        stalled_at: Option<usize>,
    }

    impl DeviceSide {
        fn feed(&mut self, stream: &[u8]) {
            let mut fed = 0usize;
            for &byte in stream {
                self.buf.push(byte);
                fed += 1;
                if self.stalled_at.is_some() {
                    continue;
                }
                let mut no_progress = 0u8;
                loop {
                    let call = self.parser.find_block(&self.buf);
                    if call.ret == 0 {
                        break;
                    }
                    // NACK 与 pop 无关（`command.c:146-147` 在 return 前发），
                    // 所以先记账再看推进。
                    if call.nacked {
                        self.nacks += 1;
                    }
                    if call.pop == 0 {
                        no_progress += 1;
                        if no_progress >= 2 {
                            self.stalled_at = Some(fed);
                            break;
                        }
                        continue;
                    }
                    no_progress = 0;
                    let drained: Vec<u8> = self.buf.drain(..call.pop).collect();
                    if call.ret > 0 {
                        self.recovered.push(drained);
                    }
                }
            }
        }
    }

    // --------------------------------------------------------------------
    // 断言与有界兜底
    // --------------------------------------------------------------------

    /// 整体有界兜底：主体跑在独立线程，超时未结束即失败，不让解析循环挂住 CI。
    fn bounded(seconds: u64, body: impl FnOnce() + Send + 'static) {
        let (tx, rx) = std::sync::mpsc::channel();
        let _ = std::thread::spawn(move || {
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(body));
            let _ = tx.send(result);
        });
        match rx.recv_timeout(Duration::from_secs(seconds)) {
            Ok(Ok(())) => {}
            Ok(Err(payload)) => std::panic::resume_unwind(payload),
            Err(_) => panic!("双重封装测试在 {seconds}s 内没有结束：解析循环失去有界性"),
        }
    }

    fn hex(bytes: &[u8]) -> String {
        bytes
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect::<Vec<_>>()
            .join(" ")
    }

    /// 逐字节差异：首个不同偏移 + 期望/实际字节，附完整十六进制行。
    fn byte_diff(expected: &[u8], actual: &[u8]) -> String {
        let first = expected.iter().zip(actual).position(|(e, a)| e != a);
        let mismatch = first.or_else(|| {
            (expected.len() != actual.len()).then_some(expected.len().min(actual.len()))
        });
        let head = match mismatch {
            Some(i) => {
                let show =
                    |v: Option<u8>| v.map_or_else(|| "<缺>".to_string(), |b| format!("{b:#04x}"));
                format!(
                    "首个差异 @偏移 {i}: 期望 {}, 实际 {}",
                    show(expected.get(i).copied()),
                    show(actual.get(i).copied())
                )
            }
            None => "两串逐字节一致".to_string(),
        };
        format!(
            "{head}\n  期望({}): {}\n  实际({}): {}",
            expected.len(),
            hex(expected),
            actual.len(),
            hex(actual)
        )
    }

    fn assert_recovered(actual: &[Vec<u8>], expected: &[Vec<u8>], ctx: &str) {
        let names = |frames: &[Vec<u8>]| {
            frames
                .iter()
                .map(|f| format!("[{}]", hex(f)))
                .collect::<Vec<_>>()
                .join(" ")
        };
        assert_eq!(
            actual.len(),
            expected.len(),
            "{ctx}: 捞出帧数不对\n  实际 {}: {}\n  期望 {}: {}",
            actual.len(),
            names(actual),
            expected.len(),
            names(expected)
        );
        for (i, (got, want)) in actual.iter().zip(expected).enumerate() {
            assert!(
                got == want,
                "{ctx}: 第 {i} 帧不一致\n{}",
                byte_diff(want, got)
            );
        }
    }

    // --------------------------------------------------------------------
    // 四类载荷
    // --------------------------------------------------------------------

    /// ① 8 B Connect：8 ≤ 59，单帧装得下 —— 完整捞出。剥头前的 [len][seq]
    /// 触发一次 resync 丢弃，按固件的一次性 NACK 语义恰好 NACK 一次；
    /// 内层帧之后只剩 3 字节外层尾（klipper crc+0x7e），永远不够 8 B、无害。
    #[test]
    fn a_connect_control_frame_survives_dual_encapsulation_intact() {
        bounded(10, || {
            let inner = connect_block();
            assert!(inner.len() <= PAYLOAD_MAX, "8 ≤ 59：单帧装得下");
            let units = host_units(our_arb(), &inner);
            assert_eq!(units.len(), 1, "Connect 只裹一个单元");
            let wire = units.concat();
            let device_bytes = bridge_down(&wire, Some(our_arb()));
            assert_eq!(
                device_bytes.len(),
                13,
                "[len][seq] + 8 B 内层 + [crc][0x7e]"
            );
            let mut dev = DeviceSide::default();
            dev.feed(&device_bytes);
            assert_recovered(&dev.recovered, &[inner], "① Connect");
            assert_eq!(dev.nacks, 1, "首个错误触发固件的一次性 NACK");
            assert_eq!(dev.stalled_at, None, "不得停摆");
            assert_eq!(dev.buf.len(), 3, "残留仅为外层帧尾 klipper crc+0x7e");
        });
    }

    /// ② 76 B Send Block：76 > 单帧载荷 59 ⇒ 必须切成 59+17 两帧。剥头后
    /// 设备流在内层偏移 59 处被外层帧尾 (crc16+0x7e) 与下一帧头 (len+seq)
    /// 撕开，内层 CRC 必炸 ⇒ 整帧捞不出；且解析器停摆——`command.c:141-143`
    /// 的 resync 在缓冲头就是 `0x01` 时 `memchr` 命中自身、`pop_count = 0`，
    /// 缓冲永不推进，后续字节永远轮不到。
    #[test]
    fn a_76_byte_send_block_is_torn_apart_by_the_klipper_frame_split() {
        bounded(10, || {
            let inner = send_block(0x1000);
            assert_eq!(inner.len(), 76, "msglen = 17*4+8");
            assert!(inner.len() > PAYLOAD_MAX, "关键算术：76 B > 59 ⇒ 必须分片");
            let sizes: Vec<usize> = inner.chunks(PAYLOAD_MAX).map(<[u8]>::len).collect();
            assert_eq!(sizes, vec![59, 17], "切点必然落在内层帧中间");
            let units = host_units(our_arb(), &inner);
            assert_eq!(units.len(), 2);
            let wire = units.concat();
            let device_bytes = bridge_down(&wire, Some(our_arb()));
            assert_eq!(device_bytes.len(), 86, "64 B 整帧 + 22 B 整帧");
            let mut dev = DeviceSide::default();
            dev.feed(&device_bytes);
            assert!(
                dev.recovered.is_empty(),
                "76 B Send Block 必须整帧捞不出，实捞 {}",
                dev.recovered
                    .iter()
                    .map(|f| hex(f))
                    .collect::<Vec<_>>()
                    .join(" | ")
            );
            assert_eq!(
                dev.nacks, 1,
                "只有首个错误发 NACK，其余被 CF_NEED_VALID 静默"
            );
            let stalled_at = dev
                .stalled_at
                .expect("分片撕裂后解析器必须停摆（pop 0、缓冲永不推进）");
            assert_eq!(stalled_at, 78, "内层第 76 B（= 流偏移 2+76）处判定失败");
            // 逐字节证据：停摆缓冲的头仍是我们内层帧的头；内层偏移 59..64
            // 正是被夹进来的外层帧1尾(crc16+0x7e)+帧2头(len+seq)。
            assert_eq!(
                &dev.buf[..4],
                &[K_STX1, K_STX2, CMD_RX_BLOCK, 17],
                "停摆在我们的内层帧头上"
            );
            assert_eq!(
                &dev.buf[59..64],
                &device_bytes[61..66],
                "内层偏移 59 起 = 外层帧1尾 + 帧2头"
            );
            assert_eq!(
                device_bytes[63], FRAME_SYNC,
                "外层帧1的 0x7e 插在内层偏移 61"
            );
            assert_eq!(device_bytes[64], 22, "外层帧2的 len 插在内层偏移 62");
        });
    }

    /// ③ 跨两个 klipper 帧的分片流：56 B 块整段落在第一帧的 59 B 载荷里，
    /// 8 B Connect 从内层偏移 56 起跨过 59 的切点 —— 前一个完整块照常捞出，
    /// 跨界的 Connect 被外层帧尾/下一帧头撕碎、捞不出：撕碎的头就压在缓冲
    /// 最前，word_cnt 位被外层 crc 字节污染、msglen 要么越界要么与外层尾
    /// 对不上，立刻报错 → resync 命中自身 pop 0 → 停摆。
    #[test]
    fn an_inner_frame_straddling_the_split_is_lost_while_the_one_before_survives() {
        bounded(10, || {
            let first = block_56();
            let straddler = connect_block();
            let mut inner = first.clone();
            inner.extend_from_slice(&straddler);
            assert_eq!(inner.len(), 64);
            let sizes: Vec<usize> = inner.chunks(PAYLOAD_MAX).map(<[u8]>::len).collect();
            assert_eq!(sizes, vec![59, 5], "Connect 从 56 起跨过 59 的切点");
            let units = host_units(our_arb(), &inner);
            assert_eq!(units.len(), 2);
            let wire = units.concat();
            let device_bytes = bridge_down(&wire, Some(our_arb()));
            let mut dev = DeviceSide::default();
            dev.feed(&device_bytes);
            assert_recovered(&dev.recovered, &[first], "③ 分片流：只有前一个完整块");
            // 有效块清过 CF_NEED_VALID 后，跨界帧的失败再 NACK 一次（前一次是
            // 丢 [len][seq]），随即 pop 0 停摆：撕碎的头让外层帧1的 crc 字节
            // 被读成 word_cnt，buf 一到 8 B 就报错（越界，或 msglen=8 但尾
            // 对不上外层的 0x0a），resync 命中自身、缓冲永不推进。
            assert_eq!(dev.nacks, 2, "丢 [len][seq] 一次、跨界帧失败一次");
            assert_eq!(
                dev.stalled_at,
                Some(66),
                "buf 到 8 B（= 流偏移 66）处报错并停摆"
            );
            assert_eq!(
                dev.buf.len(),
                16,
                "停摆时已 pop 2+56，余下 16 B 只进缓冲不再解析"
            );
            // 逐字节证据：内层偏移 3 起是被夹进来的外层帧1尾 + 帧2头。
            assert_eq!(
                &dev.buf[..8],
                &[
                    K_STX1,
                    K_STX2,
                    CMD_CONNECT,
                    device_bytes[61], // 外层帧1 crc 高
                    device_bytes[62], // 外层帧1 crc 低
                    device_bytes[63], // 外层帧1 0x7e
                    device_bytes[64], // 外层帧2 len
                    device_bytes[65], // 外层帧2 seq
                ],
                "撕碎的 Connect 头压在缓冲最前，外层帧尾/下一帧头穿在它中间"
            );
        });
    }

    /// 其他 MCU 的合法 klipper 单元：identify 请求帧（本仓既有测试的常形）。
    fn foreign_frame(seq: u8) -> Vec<u8> {
        Frame::encode(seq, b"identify offset=0 count=112")
    }
    fn foreign_unit(seq: u8) -> Vec<u8> {
        let mut unit = encode_header(foreign_arb());
        unit.extend_from_slice(&foreign_frame(seq));
        unit
    }

    /// ④ 混合流的线：其他 MCU 的单元前后夹着裹了 Connect 的我们的单元。
    fn mixed_wire(inner: &[u8]) -> Vec<u8> {
        let mut wire = foreign_unit(1);
        for unit in host_units(our_arb(), inner) {
            wire.extend_from_slice(&unit);
        }
        wire.extend_from_slice(&foreign_unit(2));
        wire
    }

    /// ④ 混合流：其他 MCU 的合法 klipper 单元夹着我们裹了 Connect 的单元。
    /// 桥按单元头路由时外层帧根本不落到本设备（a）；即使按最坏情况整条线
    /// 的帧都落到设备（b），外层帧不含 0x01、被 resync 整段丢弃，
    /// Connect 仍完整捞出。
    #[test]
    fn our_connect_survives_foreign_klipper_units_sandwiching_it() {
        bounded(10, || {
            let inner = connect_block();
            let wire = mixed_wire(&inner);

            // (a) 路由：其他 MCU 的单元被桥路由走，只有我们的帧落设备。
            let routed = bridge_down(&wire, Some(our_arb()));
            assert_eq!(routed.len(), 13, "外层帧路由走后只剩我们的 13 B 帧");
            let mut dev_a = DeviceSide::default();
            dev_a.feed(&routed);
            assert_recovered(&dev_a.recovered, &[inner.clone()], "④a 路由");
            assert_eq!(dev_a.nacks, 1);
            assert_eq!(dev_a.stalled_at, None);

            // (b) 最坏情况：整条线的帧都落到设备 —— 外层帧必须无 0x01
            // （resync 才能整段丢弃而不是停在它里面），Connect 仍捞出。
            for seq in [1u8, 2] {
                let frame = foreign_frame(seq);
                assert!(
                    !frame.contains(&K_STX1),
                    "外层帧必须不含 0x01 才能被 resync 整段丢弃: {}",
                    hex(&frame)
                );
            }
            let leaked = bridge_down(&wire, None);
            assert!(leaked.len() > routed.len(), "混合流确实混进了外层帧");
            let mut dev_b = DeviceSide::default();
            dev_b.feed(&leaked);
            assert_recovered(&dev_b.recovered, &[inner], "④b 混合流");
        });
    }
}

// ===========================================================================
// Phase-1 hardware probe: the mux device half on a real board
// ===========================================================================
//
// Drives a board that runs the firmware's serial-mux device half over its own
// tty, **with no bridge in the path** (the staging in
// `docs/work-log/2026-10-16-katapult-flash-task.md`, "真机测试分阶段口径"): every
// step sends one unit and reads what comes back, so each answer of the device
// half is pinned against real bytes instead of a pty fixture.
//
// The two tests split where the board's identity changes:
//
//   * [`hw_mux_device_answers_the_admin_commands`] — discovery, the assignment,
//     a bridge-only command (`SET_PORT_NODEID`) that a device must ignore, and a
//     tunnel unit (dropped whole, with the stream alive afterwards);
//   * [`hw_mux_device_bootloader_and_restart`] — the two commands that make the
//     device *leave*: `RESTART_DEVICE` (same firmware comes back) and
//     `REQUEST_BOOTLOADER` (katapult takes over).
//
// Both are `#[ignore]`d and take the tty from `KLIPPERX_HW_MUX_TTY`; without it
// they return immediately, so the default gate is untouched.
#[cfg(test)]
mod hw_mux_device_tests {
    use super::*;
    use crate::core::klippy::frame::{Frame, MESSAGE_DEST, MESSAGE_MAX, MESSAGE_MIN};

    /// The tty a board running the mux device half listens on.
    fn hw_tty() -> Option<PathBuf> {
        std::env::var_os("KLIPPERX_HW_MUX_TTY")
            .filter(|v| !v.is_empty())
            .map(PathBuf::from)
    }

    /// The identity the probe addresses the board by.
    ///
    /// This is the board's own (chipid-derived, stable across resets) identity;
    /// an assignment naming it is idempotent, which is what lets the two probes
    /// run in any order — a scan only answers while the board holds no node id,
    /// and a node id outlives a test run (it lives in the host, not the MCU).
    /// `KLIPPERX_HW_MUX_UUID` overrides it for another board.
    fn hw_uuid() -> [u8; 6] {
        let Some(raw) = std::env::var_os("KLIPPERX_HW_MUX_UUID") else {
            return [0xe5, 0x12, 0xc6, 0x0a, 0xc0, 0xe6];
        };
        let text = raw.to_string_lossy();
        let bytes: Vec<u8> = (0..6)
            .map(|i| u8::from_str_radix(&text[i * 2..i * 2 + 2], 16).expect("a hex uuid"))
            .collect();
        let mut uuid = [0u8; 6];
        uuid.copy_from_slice(&bytes);
        uuid
    }

    /// Bind the board to `mcu_id` (idempotent: the same identity keeps it).
    fn bind(port: &mut File, mcu_id: u32) {
        let assignment = set_node_id_for(hw_uuid(), mcu_id).expect("an addressable id");
        port.write_all(&assignment)
            .expect("the assignment goes out");
        std::thread::sleep(Duration::from_millis(200));
    }

    /// One data unit to `mcu_id`, and whether the board answered it — how a
    /// probe tells a bound, live device from one that never adopted the id.
    fn alive_on(port: &mut File, pending: &mut Vec<u8>, mcu_id: u32, window: Duration) -> bool {
        let frame = Frame::new(MESSAGE_DEST, vec![1]).raw_bytes();
        let mut unit = encode_header(assign::arb_id(mcu_id).expect("an addressable id"));
        unit.extend_from_slice(&frame);
        port.write_all(&unit).expect("the frame unit goes out");
        read_unit_for(
            port,
            pending,
            assign::reply_arb_id(mcu_id).expect("the id"),
            window,
        )
        .is_some()
    }

    /// Open the tty raw and non-blocking, the way the device layer opens one.
    ///
    /// **DTR and RTS are asserted**: a board's CDC console only hands its
    /// bytes over once the host raises the modem lines (the first hardware run
    /// got silence until they were), so a probe that opens without them sees
    /// a device that looks dead.
    fn open_hw_tty(path: &Path) -> File {
        let port = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NOCTTY | libc::O_NONBLOCK)
            .open(path)
            .unwrap_or_else(|e| panic!("opening {}: {e}", path.display()));
        set_raw_mode(&port, DEFAULT_BAUD).expect("the hardware tty goes raw");
        set_modem_line(&port, libc::TIOCM_DTR, true).expect("DTR is asserted");
        set_modem_line(&port, libc::TIOCM_RTS, true).expect("RTS is asserted");
        // Discard whatever accumulated while nobody was reading (a prior run's
        // late answers, the firmware's `stats` frames): a probe reads from its
        // own question onwards.
        // SAFETY: `tcflush` takes the fd and a queue selector.
        unsafe {
            libc::tcflush(port.as_raw_fd(), libc::TCIFLUSH);
        }
        // Let the board's console settle after the line change before the first
        // unit goes out (the working probe waited here).
        std::thread::sleep(Duration::from_millis(300));
        port
    }

    /// Drop bytes up to and including the next frame sync — the device layer's
    /// resynchronisation policy ([`drop_to_sync`]), which a probe needs for the
    /// same reason the reader does: a board also sends its own traffic (the
    /// firmware's once-a-second `stats` frame), and those bytes sit in front of
    /// the unit a probe is waiting for.
    fn skip_to_sync(pending: &mut Vec<u8>) {
        match pending.iter().position(|b| *b == MESSAGE_SYNC) {
            Some(idx) => {
                pending.drain(..=idx);
            }
            None => pending.clear(),
        }
    }

    /// A whole unit cut off the front of `pending`, if one is there.
    fn unit_from(pending: &mut Vec<u8>) -> Option<Vec<u8>> {
        loop {
            if pending.is_empty() {
                return None;
            }
            let consumed = match decode_header(pending) {
                HeaderScan::Complete { consumed, .. } => consumed,
                HeaderScan::NeedMore => return None,
                HeaderScan::Invalid => {
                    skip_to_sync(pending);
                    continue;
                }
            };
            if pending.len() < consumed + 1 {
                return None;
            }
            let length = pending[consumed] as usize;
            if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&length) {
                // A length no frame has: the header is not one after all.
                skip_to_sync(pending);
                continue;
            }
            if pending.len() < consumed + length {
                return None;
            }
            return Some(pending.drain(..consumed + length).collect());
        }
    }

    /// The next whole unit within `window`, or nothing if the port stayed
    /// silent — the two outcomes every step below distinguishes.
    fn read_unit(port: &File, pending: &mut Vec<u8>, window: Duration) -> Option<Vec<u8>> {
        let deadline = Instant::now() + window;
        let mut buf = [0u8; 256];
        // `File` reads through a shared reference, so a local binding is all the
        // `Read` methods need here.
        let mut port = port;
        loop {
            if let Some(unit) = unit_from(pending) {
                return Some(unit);
            }
            let now = Instant::now();
            if now >= deadline {
                return None;
            }
            let mut poll = libc::pollfd {
                fd: port.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            };
            let rc = unsafe {
                libc::poll(
                    &mut poll,
                    1,
                    deadline.saturating_duration_since(now).as_millis() as i32,
                )
            };
            assert!(rc >= 0, "poll: {}", std::io::Error::last_os_error());
            if rc == 0 {
                continue;
            }
            match port.read(&mut buf) {
                Ok(0) => return None,
                Ok(n) => pending.extend_from_slice(&buf[..n]),
                Err(e) if e.kind() == ErrorKind::WouldBlock => {}
                Err(e) => panic!("reading the hardware tty: {e}"),
            }
        }
    }

    /// The next unit **for `want_arb`** within `window`, discarding units for
    /// other ids — the dispatch rule the device layer's reader applies, and what
    /// a probe needs here for the same reason: a board also delivers what a
    /// previous run left in flight (an old reply header) and sends its own
    /// `stats` frames, so the front of the stream is not ours to assume.
    fn read_unit_for(
        port: &File,
        pending: &mut Vec<u8>,
        want_arb: u32,
        window: Duration,
    ) -> Option<Vec<u8>> {
        let deadline = Instant::now() + window;
        loop {
            let now = Instant::now();
            if now >= deadline {
                return None;
            }
            match read_unit(port, pending, deadline.saturating_duration_since(now)) {
                Some(unit) => {
                    if unit_parts(&unit).0 == want_arb {
                        return Some(unit);
                    }
                }
                None => return None,
            }
        }
    }

    /// The arbitration id and frame half of a unit.
    fn unit_parts(unit: &[u8]) -> (u32, &[u8]) {
        match decode_header(unit) {
            HeaderScan::Complete { arb, consumed } => (arb, &unit[consumed..]),
            other => panic!("not a unit: {other:?}"),
        }
    }

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02x}")).collect()
    }

    /// Wait until `path` exists again, up to `window`.
    fn wait_for_path(path: &Path, window: Duration) -> bool {
        let deadline = Instant::now() + window;
        while Instant::now() < deadline {
            if path.exists() {
                return true;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        false
    }

    /// Whether a `usb-katapult_…` node is enumerated right now.
    fn katapult_node_enumerated() -> Option<String> {
        let entries = std::fs::read_dir("/dev/serial/by-id").ok()?;
        for entry in entries.flatten() {
            let name = entry.file_name().to_string_lossy().to_string();
            if name.starts_with("usb-katapult_") {
                return Some(name);
            }
        }
        None
    }

    /// The answers a device half owes a host that is *directly* on its tty.
    #[test]
    #[ignore = "hardware: needs KLIPPERX_HW_MUX_TTY (a board running the mux device half)"]
    fn hw_mux_device_answers_the_admin_commands() {
        let Some(path) = hw_tty() else {
            eprintln!("KLIPPERX_HW_MUX_TTY is not set: nothing to probe");
            return;
        };
        let mut port = open_hw_tty(&path);
        let mut pending = Vec::new();
        let window = Duration::from_secs(3);
        let silent = Duration::from_millis(800);

        // 1. An unaddressed board answers a scan on the admin reply id. A board
        //    that already holds a node id (a previous run, or the sibling probe)
        //    stays quiet by design — that is what a scan lists — so the answer
        //    is checked when it comes and the step is a note when it does not.
        let query = discovery_query();
        eprintln!("1. discovery query = {}", hex(&query));
        port.write_all(&query).expect("the query goes out");
        match read_unit_for(&port, &mut pending, ADMIN_REPLY, window) {
            Some(answer) => {
                let node = UnassignedNode::from_unit(&answer).expect("a well-formed admin answer");
                assert_eq!(
                    node.uuid,
                    hw_uuid(),
                    "the board answers with its own identity"
                );
                eprintln!(
                    "1. discovery: uuid={}, application={:?}",
                    hex(&node.uuid),
                    node.application
                );
            }
            None => eprintln!(
                "1. no scan answer: the board already holds a node id (a scan lists the \
                 unaddressed only)"
            ),
        }

        // 2. The assignment is adopted, so a second scan draws silence.
        bind(&mut port, 5);
        port.write_all(&discovery_query())
            .expect("the second scan goes out");
        assert!(
            read_unit_for(&port, &mut pending, ADMIN_REPLY, silent).is_none(),
            "an addressed board must stay quiet in a scan"
        );
        eprintln!("2. the assignment was adopted (the second scan was silent)");

        // 3. `SET_PORT_NODEID` belongs to a bridge's own port table: a device
        //    must not execute it, and must not answer it.
        let port_command = admin_unit(&[CMD_SET_PORT_NODEID, 0, 5, 0]);
        port.write_all(&port_command)
            .expect("the port command goes out");
        assert!(
            read_unit_for(&port, &mut pending, ADMIN_REPLY, silent).is_none(),
            "SET_PORT_NODEID is a bridge's own command, so a device must ignore it"
        );
        eprintln!("3. SET_PORT_NODEID (0x03) drew no answer from the device");

        // 4. A tunnel unit is dropped whole, and the stream stays alive: a valid
        //    klipper frame still draws an answer afterwards.
        let katapult_frame = vec![0x01, 0x88, 0x11, 0x00, 0xf1, 0x7c, 0x99, 0x03];
        let tunnel = tunnel_unit(assign::arb_id(5).expect("id 5"), &katapult_frame)
            .expect("a whole katapult frame is one tunnel unit");
        port.write_all(&tunnel).expect("the tunnel unit goes out");
        if let Some(extra) = read_unit_for(&port, &mut pending, ADMIN_REPLY, silent) {
            panic!(
                "a tunnel unit is not the device half's to answer (got {})",
                hex(&extra)
            );
        }
        //    klipper frame still draws an answer afterwards. The tunnel unit
        //    itself leaves the device's frame parser resynchronising (it looks
        //    for the next SYNC), so the first frame after it can be swallowed —
        //    at most one, which is the property the tunnel unit test pins on the
        //    host side too. The first probe that *is* answered is the proof the
        //    stream came back.
        let mut answered = alive_on(&mut port, &mut pending, 5, window);
        let mut probes = 1;
        while !answered && probes < 4 {
            probes += 1;
            answered = alive_on(&mut port, &mut pending, 5, window);
        }
        assert!(answered, "the device answers again after it resynchronises");
        eprintln!(
            "4. a tunnel unit drew no answer; the stream answered after {probes} probe frames"
        );
    }

    /// The two commands that make a device leave, and what it comes back as.
    #[test]
    #[ignore = "hardware: needs KLIPPERX_HW_MUX_TTY (a board running the mux device half)"]
    fn hw_mux_device_bootloader_and_restart() {
        let Some(path) = hw_tty() else {
            eprintln!("KLIPPERX_HW_MUX_TTY is not set: nothing to probe");
            return;
        };
        let mut port = open_hw_tty(&path);
        let mut pending = Vec::new();
        let window = Duration::from_secs(3);

        // 0. Bind the board and prove it holds the id: the two commands below
        //    are transported as units addressed to it, so there is nothing to
        //    send if it never adopted the id.
        bind(&mut port, 5);
        assert!(
            alive_on(&mut port, &mut pending, 5, window),
            "the board holds node id 5 after the assignment"
        );
        eprintln!("0. bound: the board answers on id 5's reply header");

        // 1. `RESTART_DEVICE` reboots the firmware. The node id does not survive
        //    a reset (it is the host's to hand out), so the rebooted board must
        //    answer a scan again — the proof that this command really reset it.
        port.write_all(&admin_unit(&[CMD_RESTART_DEVICE, 0]))
            .expect("the restart goes out");
        drop(port);
        assert!(
            wait_for_path(&path, Duration::from_secs(15)),
            "the tty comes back after RESTART_DEVICE"
        );
        std::thread::sleep(Duration::from_millis(800));
        let mut port = open_hw_tty(&path);
        let mut pending = Vec::new();
        let mut rebooted = None;
        for _ in 0..6 {
            port.write_all(&discovery_query())
                .expect("the query goes out");
            if let Some(answer) =
                read_unit_for(&port, &mut pending, ADMIN_REPLY, Duration::from_secs(2))
            {
                if let Some(node) = UnassignedNode::from_unit(&answer) {
                    rebooted = Some(node);
                    break;
                }
            }
        }
        let node = rebooted.expect("the rebooted board answers a scan (its node id is gone)");
        assert_eq!(node.uuid, hw_uuid(), "the same board came back");
        eprintln!(
            "1. RESTART_DEVICE: back up unaddressed, uuid={}",
            hex(&node.uuid)
        );

        // 2. `REQUEST_BOOTLOADER` hands the board to katapult, whose USB id is
        //    its own (`usb-katapult_…`, the klipper node disappears).
        bind(&mut port, 5);
        port.write_all(&admin_unit(&[CMD_REQUEST_BOOTLOADER, 0]))
            .expect("the bootloader request goes out");
        drop(port);
        let deadline = Instant::now() + Duration::from_secs(20);
        let mut found = None;
        while Instant::now() < deadline {
            if let Some(name) = katapult_node_enumerated() {
                found = Some(name);
                break;
            }
            std::thread::sleep(Duration::from_millis(200));
        }
        let name = found.expect("REQUEST_BOOTLOADER leaves the board in katapult");
        eprintln!("2. REQUEST_BOOTLOADER: the board is in katapult as {name}");
    }
}
