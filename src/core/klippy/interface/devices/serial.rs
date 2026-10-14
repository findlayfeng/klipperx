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
//! assignment-time handshake and the discovery/assignment admin channel — and
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
/// The frame body byte that asks the device behind a bridge port to reset
/// itself — CAN's `CANBUS_CMD_REQUEST_BOOTLOADER` by value and meaning. A
/// bridge routes it by the port index that follows; on a board with no bridge
/// the device half runs it itself.
pub const CMD_RESET_DEVICE: u8 = 0x02;
/// The frame body byte of the port-level command that names the device behind
/// a bridge port: `[CMD_SET_PORT_NODEID][index][id][flags]`. The bridge builds
/// its port tables from it and consumes it — no receipt, never forwarded.
pub const CMD_SET_PORT_NODEID: u8 = 0x03;
/// The frame body byte that asks a bridge to pulse the lines of the port that
/// follows — the only reset a device with no admin channel of its own can get.
pub const CMD_RESET_PORT: u8 = 0x04;
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

/// One board's answer to [`discovery_query`]: the UUID it is remembered by,
/// and the application it runs.
///
/// This mirrors CAN's
/// [`UnassignedNode`](crate::core::klippy::interface::devices::canserial::UnassignedNode):
/// the board answers with [`RESP_NEED_NODEID`], the six UUID bytes most
/// significant first, and — in the builds that send it — one more byte naming
/// the application. The UUID and the application byte are read **by position
/// against the frame's length byte**, so a `0x7e` inside them is ordinary
/// content: the protocol allows SYNC in message block content.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UnassignedNode {
    /// The UUID, most significant byte first.
    pub uuid: [u8; 6],
    /// The application byte the board sent, if it sent one.
    pub application: Option<u8>,
}

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
    /// with [`RESP_NEED_NODEID`] followed by the six UUID bytes and an
    /// optional application byte. Nothing else is checked — in particular a
    /// UUID of six zero bytes is reported rather than treated as unset,
    /// because the protocol has no such value.
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
        // A whole 1-byte application is the only legal extra; more trailing
        // bytes mean the unit was not this shape.
        let application = match after_cmd.len() {
            6 => None,
            7 => Some(after_cmd[6]),
            _ => return None,
        };
        Some(Self { uuid, application })
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
/// else is the shared codec ([`decode_header`], [`drop_to_sync`], the
/// `MESSAGE_*` frame constants) — one framing, not a second one.
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
    /// Exactly the [`dispatch`] policy: an invalid header or a length byte
    /// outside the frame range resynchronises on the next [`MESSAGE_SYNC`].
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
            if self.pending.len() < consumed + 1 {
                return None; // the length byte is still in flight
            }
            let length = self.pending[consumed] as usize;
            if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&length) {
                drop_to_sync(&mut self.pending, &mut self.needs_sync);
                continue;
            }
            let unit_len = consumed + length;
            if self.pending.len() < unit_len {
                return None; // the rest of the frame is still in flight
            }
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
    /// One slot per attached MCU, keyed by the MCU id.
    slots: Mutex<HashMap<u32, Arc<McuSlot>>>,
    /// Serializes the handshakes on this port: two MCUs attaching together must
    /// not interleave their requests and answers.
    attach_lock: Mutex<()>,
    /// Serializes whole units on the write side (`send` and the handshake
    /// request): a unit is a header plus a frame, and the two must not be split
    /// by another writer.
    write_lock: Mutex<()>,
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
/// stays up until the last handle is gone.
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
        port.slots.lock().unwrap().remove(&mcu_id);
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
    fn drop(&mut self) {
        self.stop();
        // The reader may still be inside a `read` (bounded by `VTIME`); joining
        // it here is what lets the port and its descriptor go away. It does not
        // need the mutex to finish — it sees `stopped` and returns.
        if let Ok(mut guard) = self.reader.lock() {
            if let Some(handle) = guard.take() {
                let _ = handle.join();
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
                // the last handle could never bring the port down.
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
/// A unit is a header ([`decode_header`]) followed by one frame
/// (length byte + that many bytes). Anything that cannot be one — an invalid
/// header, an arbitration id outside the assigned range, or a length byte
/// outside `MESSAGE_MIN..=MESSAGE_MAX` — resynchronises on the next SYNC, the
/// same policy as [`FrameStream`]. Too few bytes for a unit just waits for the
/// next read.
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
        // nor an answer id): no unit can start here.
        if assign::mcu_id_from_arb(arb).is_none() {
            drop_to_sync(pending, needs_sync);
            continue;
        }

        if pending.len() < consumed + 1 {
            return; // the length byte is still in flight
        }
        let length = pending[consumed] as usize;
        if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&length) {
            drop_to_sync(pending, needs_sync);
            continue;
        }

        let unit_len = consumed + length;
        if pending.len() < unit_len {
            return; // the rest of the frame is still in flight
        }

        let unit = &pending[..unit_len];
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

    /// Ask the device behind port `index` to reset itself: [`CMD_RESET_DEVICE`]'s
    /// admin unit, which the bridge forwards to that port and the device acts on.
    ///
    /// The reset a **named** bridge-port device gets — the one that answers the
    /// mux itself. No receipt: the bridge routes the command and the device
    /// either resets or does not, so the caller confirms it by whether the
    /// device can be reached again (`mcu/restart.rs`).
    ///
    /// # Errors
    /// The same as [`SharedSerialHandle::send_unit`].
    pub fn send_reset_device(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_admin(&admin_unit(&[CMD_RESET_DEVICE, index])[HEADER_BYTES..])
    }

    /// Ask the bridge to pulse the lines of port `index` — [`CMD_RESET_PORT`]'s
    /// admin unit. The bridge performs it; nothing is sent downstream.
    ///
    /// The reset a **legacy** bridge-port device gets: it has no admin channel of
    /// its own, so its bridge's port is the only thing that can reset it. No
    /// receipt, like [`SharedSerialHandle::send_reset_device`].
    ///
    /// # Errors
    /// The same as [`SharedSerialHandle::send_unit`].
    pub fn send_reset_port(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_admin(&admin_unit(&[CMD_RESET_PORT, index])[HEADER_BYTES..])
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
    /// round-tripping frames.
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

    #[test]
    fn set_port_node_id_is_the_port_level_admin_unit() {
        // The three port-level command bytes are the firmware's mux.h table
        // (0x02 RESET_DEVICE / 0x03 SET_PORT_NODEID / 0x04 RESET_PORT) — pinned
        // here so a drift from the C side fails on the host too, not only in the
        // real-library probe.
        assert_eq!(CMD_RESET_DEVICE, 0x02);
        assert_eq!(CMD_SET_PORT_NODEID, 0x03);
        assert_eq!(CMD_RESET_PORT, 0x04);
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
}
