//! A pseudo-terminal pair for tests.
//!
//! A tty is the one thing a serial test cannot fake: `termios2`, `TIOCMBIS` and
//! a real `read` all need a kernel tty, and `posix_openpt` is what gives one to
//! an unprivileged test process.

use std::fs::{File, OpenOptions};
use std::os::fd::FromRawFd;
use std::path::PathBuf;

/// Both ends of a pty, kept open.
///
/// The device under test opens the slave `path`; the test drives the `master`.
/// Holding both ends open is what keeps the pair alive.
pub(crate) struct Pty {
    /// The master end, for a test to read what was sent or write to be received.
    pub(crate) master: File,
    _slave: File,
    /// The slave path, which is what a `serial:` transport opens.
    pub(crate) path: PathBuf,
}

impl Pty {
    pub(crate) fn new() -> Self {
        let master_fd = unsafe { libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY) };
        assert!(
            master_fd >= 0,
            "posix_openpt: {}",
            std::io::Error::last_os_error()
        );
        assert_eq!(unsafe { libc::grantpt(master_fd) }, 0);
        assert_eq!(unsafe { libc::unlockpt(master_fd) }, 0);

        let mut number: libc::c_uint = 0;
        let rc = unsafe { libc::ioctl(master_fd, libc::TIOCGPTN, &mut number) };
        assert_eq!(rc, 0, "TIOCGPTN: {}", std::io::Error::last_os_error());
        let path = PathBuf::from(format!("/dev/pts/{number}"));

        let slave = OpenOptions::new()
            .read(true)
            .write(true)
            .open(&path)
            .expect("the pty slave should open");

        Self {
            // SAFETY: `master_fd` comes from `posix_openpt` above and is owned
            // by this `File` from here on.
            master: unsafe { File::from_raw_fd(master_fd) },
            _slave: slave,
            path,
        }
    }
}
