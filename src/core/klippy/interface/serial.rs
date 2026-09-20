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
//! [`FrameStream`]: crate::core::klippy::frame::FrameStream

use super::error::InterfaceError;
use super::{describe_frame, Device};
use crate::core::klippy::frame::{Frame, FrameStream};
use std::fmt;
use std::fs::{File, OpenOptions};
use std::io::{ErrorKind, Read, Write};
use std::os::fd::AsRawFd;
use std::os::unix::fs::OpenOptionsExt;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use tracing::{debug, info, trace, warn};

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
            if let Some(frame) = self.stream.lock().unwrap().next() {
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

/// Put the tty into raw mode and set its speed.
///
/// The speed is set through `termios2`/`BOTHER`, which takes an arbitrary rate:
/// Klipper's 250000 is not one of the portable `B*` constants, and the usual
/// alternative (`TIOCSSERIAL`) needs privileges.
///
/// Raw mode is what `cfmakeraw` would do: no echo, no line editing, no signal
/// characters, no CR/LF translation and no software flow control, so the only
/// bytes on the wire are the ones the protocol put there.
fn set_raw_mode(port: &File, baud: u32) -> Result<(), InterfaceError> {
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
fn clear_nonblocking(port: &File) -> Result<(), InterfaceError> {
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
fn last_error(call: &str) -> InterfaceError {
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
    use std::os::fd::FromRawFd;
    use std::sync::Arc;
    use std::time::Duration;

    /// A pseudo-terminal pair: the tests keep both ends, the device under test
    /// opens the slave path. Holding the ends open is what keeps the pair alive.
    struct Pty {
        master: File,
        _slave: File,
        path: PathBuf,
    }

    impl Pty {
        fn new() -> Self {
            let master_fd = unsafe { libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY) };
            assert!(
                master_fd >= 0,
                "posix_openpt: {}",
                last_error("posix_openpt")
            );
            assert_eq!(unsafe { libc::grantpt(master_fd) }, 0);
            assert_eq!(unsafe { libc::unlockpt(master_fd) }, 0);

            let mut number: libc::c_uint = 0;
            let rc = unsafe { libc::ioctl(master_fd, libc::TIOCGPTN, &mut number) };
            assert_eq!(rc, 0, "TIOCGPTN: {}", last_error("TIOCGPTN"));
            let path = PathBuf::from(format!("/dev/pts/{number}"));

            let slave = OpenOptions::new()
                .read(true)
                .write(true)
                .open(&path)
                .expect("the pty slave should open");

            Self {
                // SAFETY: the descriptors come from posix_openpt/open just above and
                // are owned by these `File`s from here on.
                master: unsafe { File::from_raw_fd(master_fd) },
                _slave: slave,
                path,
            }
        }
    }

    #[test]
    fn test_open_reports_a_missing_port() {
        let err = SerialDevice::open("/dev/not-a-serial-port", DEFAULT_BAUD).unwrap_err();
        assert!(matches!(err, InterfaceError::Other(_)), "{err:?}");
        assert!(err.to_string().contains("/dev/not-a-serial-port"), "{err}");
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
