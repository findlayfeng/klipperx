// crates/test-support/src/linux_firmware.rs
//
// Lifecycle fixture for the linux process firmware: it runs the `klipper.elf`
// `build.rs` builds from `test/configs/linuxprocess.config` as a standalone
// process and takes the process back down when the fixture is dropped.
//
// Two things the fixture has to arrange itself:
//
//   * a serial path of its own. `-I` defaults to the fixed
//     `/tmp/klipper_host_mcu` (src/linux/main.c), which two fixtures — or two
//     test binaries — would fight over, so every fixture creates a directory
//     under the system temporary directory (`klipperx-linux-firmware-<pid>-<n>`)
//     and passes `<dir>/mcu` to `-I`;
//   * an end to the process that is not a message: the firmware never exits on
//     its own, and any byte sent down its pty may be `config_reset`, which
//     makes it `execv()` itself back to life. The fixture only kills.

use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

/// How long [`LinuxFirmware::spawn`] waits for the `-I` symlink to resolve.
///
/// The firmware creates it in `console_setup` right after `openpty`, so a
/// machine that has not shown it within five seconds is a machine where
/// something is wrong — report that instead of waiting forever.
const READY_TIMEOUT: Duration = Duration::from_secs(5);

/// Interval between readiness / reap polls.
const POLL_INTERVAL: Duration = Duration::from_millis(20);

/// Reap attempts [`LinuxFirmware::drop`] makes before it stops trying.
const REAP_ATTEMPTS: usize = 50;

/// Distinguishes the temporary directories of concurrently running fixtures.
static NEXT_ID: AtomicU64 = AtomicU64::new(0);

/// A running linux process firmware, shut down when it goes out of scope.
///
/// The firmware keeps running until it is killed, so dropping this type is the
/// only way the process ends: `drop` sends SIGKILL and reaps the child (never
/// panicking, never blocking on a child that has already exited), then removes
/// the fixture's temporary directory.
pub struct LinuxFirmware {
    child: Child,
    dir: PathBuf,
    serial_path: PathBuf,
}

impl LinuxFirmware {
    /// Start `klipper.elf -I <临时路径>` and wait until its console is up.
    ///
    /// Readiness is the firmware's own signal: `console_setup` (upstream
    /// `src/linux/console.c`) hangs the pty slave at the `-I` path as
    /// `unlink` + `symlink(ttyname, path)`, so a symlink there that resolves
    /// into `/dev/pts/*` means the process started and its console is
    /// listening.
    ///
    /// The wait is bounded by [`READY_TIMEOUT`]: a firmware that never gets
    /// there, or one that exits on the way up, fails with an error carrying its
    /// stderr tail rather than hanging the test run. The half-started process
    /// and its directory are cleaned up on that path too.
    pub fn spawn() -> io::Result<Self> {
        Self::spawn_elf(crate::linux_firmware_path(), None)
    }

    /// Start the mux-flavoured `klipper.elf` (built from
    /// `test/configs/linuxprocess-mux.config`) with a six-byte identity, and
    /// wait until its console is up.
    ///
    /// `-U <12 hex digits>` sets the identity the firmware answers a discovery
    /// scan with (`third_party/klipper/src/linux/main.c`), so a bridge told the
    /// same bytes in a child's `uuid:` can scan, match, and hand the device its
    /// node id — the named counterpart of [`spawn`], which leaves the identity
    /// all zero and speaks the legacy bare-frame protocol.
    pub fn spawn_named(uuid: [u8; 6]) -> io::Result<Self> {
        Self::spawn_elf(crate::linux_mux_firmware_path(), Some(hex_uuid(&uuid)))
    }

    /// The shared body of [`spawn`] and [`spawn_named`]: run `elf` with the
    /// fixture's own `-I` path (and `-U identity` when given), then wait as
    /// the doc on [`spawn`] describes.
    fn spawn_elf(elf: PathBuf, identity: Option<String>) -> io::Result<Self> {
        let dir = std::env::temp_dir().join(format!(
            "klipperx-linux-firmware-{}-{}",
            std::process::id(),
            NEXT_ID.fetch_add(1, Ordering::Relaxed)
        ));
        if let Err(e) = fs::create_dir_all(&dir) {
            return Err(io::Error::new(
                e.kind(),
                format!("failed to create {}: {e}", dir.display()),
            ));
        }
        let serial_path = dir.join("mcu");
        let stderr_log = dir.join("stderr.log");

        let stderr = match fs::File::create(&stderr_log) {
            Ok(file) => Stdio::from(file),
            Err(e) => {
                let _ = fs::remove_dir_all(&dir);
                return Err(io::Error::new(
                    e.kind(),
                    format!("failed to create {}: {e}", stderr_log.display()),
                ));
            }
        };
        let mut command = Command::new(&elf);
        command.arg("-I").arg(&serial_path);
        if let Some(identity) = &identity {
            command.arg("-U").arg(identity);
        }
        let mut child = match command
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(stderr)
            .spawn()
        {
            Ok(child) => child,
            Err(e) => {
                let _ = fs::remove_dir_all(&dir);
                return Err(io::Error::new(
                    e.kind(),
                    format!("failed to run {}: {e}", elf.display()),
                ));
            }
        };

        let ready_deadline = Instant::now() + READY_TIMEOUT;
        loop {
            if serial_link_ready(&serial_path) {
                return Ok(Self {
                    child,
                    dir,
                    serial_path,
                });
            }
            match child.try_wait() {
                Err(e) => {
                    let message = abort(
                        Some(&mut child),
                        &dir,
                        format!("waiting for {} failed: {e}", elf.display()),
                    );
                    return Err(io::Error::new(e.kind(), message));
                }
                Ok(Some(status)) => {
                    let message = abort(
                        Some(&mut child),
                        &dir,
                        format!(
                            "the firmware exited with {status} before {} appeared",
                            serial_path.display()
                        ),
                    );
                    return Err(io::Error::new(io::ErrorKind::Other, message));
                }
                Ok(None) => {}
            }
            if Instant::now() >= ready_deadline {
                let message = abort(
                    Some(&mut child),
                    &dir,
                    format!(
                        "timed out after {}s waiting for {} to resolve",
                        READY_TIMEOUT.as_secs(),
                        serial_path.display()
                    ),
                );
                return Err(io::Error::new(io::ErrorKind::TimedOut, message));
            }
            std::thread::sleep(POLL_INTERVAL);
        }
    }

    /// The firmware's process id.
    pub fn pid(&self) -> u32 {
        self.child.id()
    }

    /// The path handed to `-I`: a symlink to the pty slave the firmware opened.
    ///
    /// It exists only as long as the fixture does — `drop` takes the whole
    /// temporary directory down with the process.
    pub fn serial_path(&self) -> &Path {
        &self.serial_path
    }
}

impl Drop for LinuxFirmware {
    fn drop(&mut self) {
        // No bytes go to the firmware: a line it could read as `config_reset`
        // makes it `execv()` itself back to life, so the process only ends here.
        let _ = self.child.kill();
        // Bounded reap. `try_wait` never blocks, so a child that has already
        // exited (the first call reaps it) or one SIGKILL is already killing
        // cannot hang `drop`; giving up after `REAP_ATTEMPTS` is the price of
        // never wedging a test run — the last attempt is the plain `wait()`,
        // which by then only lasts as long as the kernel's kill delivery.
        for attempt in 0..REAP_ATTEMPTS {
            match self.child.try_wait() {
                Ok(Some(_)) => break, // reaped: no zombie, `kill(pid, 0)` says gone
                Err(_) => break,      // nothing left to do; `drop` must not panic
                Ok(None) if attempt + 1 == REAP_ATTEMPTS => {
                    let _ = self.child.wait();
                }
                Ok(None) => std::thread::sleep(POLL_INTERVAL),
            }
        }
        let _ = fs::remove_dir_all(&self.dir);
    }
}

/// The 12 lowercase hex digits `-U` takes for a six-byte identity (the
/// firmware accepts either case and no `0x` prefix, `src/linux/main.c`).
fn hex_uuid(uuid: &[u8; 6]) -> String {
    uuid.iter().map(|byte| format!("{byte:02x}")).collect()
}

/// Is the firmware's serial link up: a symlink at `path` pointing into
/// `/dev/pts/` whose target exists?
fn serial_link_ready(path: &Path) -> bool {
    match fs::read_link(path) {
        Ok(target) => target.starts_with("/dev/pts") && fs::metadata(path).is_ok(),
        Err(_) => false,
    }
}

/// End whatever is left of a spawn that cannot succeed — kill and reap the
/// child, take the fixture's directory down, and return the error message with
/// the firmware's stderr tail appended (that is where `console_setup` reports
/// why it could not set up).
fn abort(child: Option<&mut Child>, dir: &Path, message: String) -> String {
    if let Some(child) = child {
        let _ = child.kill();
        let _ = child.wait();
    }
    let detail = stderr_tail(&dir.join("stderr.log"));
    let _ = fs::remove_dir_all(dir);
    format!("{message}{detail}")
}

/// The last bytes of the firmware's stderr, formatted for an error message.
fn stderr_tail(path: &Path) -> String {
    const MAX: usize = 2048;
    let Ok(bytes) = fs::read(path) else {
        return String::new();
    };
    let tail = if bytes.len() > MAX {
        &bytes[bytes.len() - MAX..]
    } else {
        &bytes[..]
    };
    let text = String::from_utf8_lossy(tail);
    if text.trim().is_empty() {
        String::new()
    } else {
        format!("\nfirmware stderr:\n{text}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Does `pid` name a process? The probe behind `kill(pid, 0)`: a running
    /// process and a zombie that has not been reaped both hold their
    /// `/proc/<pid>` entry, and neither does once reaped — exactly the two
    /// states this fixture has to tell apart, without pulling `libc` into a
    /// crate that otherwise has no dependencies.
    fn process_exists(pid: u32) -> bool {
        Path::new(&format!("/proc/{pid}")).exists()
    }

    /// Poll `cond` until it holds or `timeout` runs out; `true` means it held.
    fn eventually(timeout: Duration, mut cond: impl FnMut() -> bool) -> bool {
        let deadline = Instant::now() + timeout;
        loop {
            if cond() {
                return true;
            }
            if Instant::now() >= deadline {
                return false;
            }
            std::thread::sleep(POLL_INTERVAL);
        }
    }

    /// Spawn the firmware, see its serial link and a live process, drop the
    /// fixture, and see both gone — every wait bounded.
    #[test]
    fn linux_firmware_spawn_and_drop() {
        let elf = crate::linux_firmware_path();
        assert!(
            elf.exists(),
            "{} is missing — the default KLIPPERX_ARCHES builds the linuxprocess target",
            elf.display()
        );

        let firmware = LinuxFirmware::spawn().expect("the linux firmware fixture should start");
        let pid = firmware.pid();
        let serial_path = firmware.serial_path().to_path_buf();

        assert!(
            process_exists(pid),
            "firmware {pid} should run right after spawn"
        );
        let target =
            fs::read_link(&serial_path).expect("spawn should have waited for the -I symlink");
        assert!(
            target.starts_with("/dev/pts"),
            "the -I path should be a symlink to a pty, got {}",
            target.display()
        );

        drop(firmware);

        assert!(
            eventually(Duration::from_secs(5), || !process_exists(pid)),
            "firmware {pid} should be killed and reaped by drop"
        );
        assert!(
            eventually(Duration::from_secs(1), || fs::symlink_metadata(
                &serial_path
            )
            .is_err()),
            "the fixture's directory should be gone with it"
        );
    }

    /// The mux-flavoured firmware starts with a `-U` identity and reaps the
    /// same way. What the identity does on the wire is the bridge test's
    /// subject; this checks the fixture can start that elf at all.
    #[test]
    fn linux_mux_firmware_spawn_named_and_drop() {
        let elf = crate::linux_mux_firmware_path();
        assert!(
            elf.exists(),
            "{} is missing — the default KLIPPERX_ARCHES builds the linuxprocess-mux target",
            elf.display()
        );

        let firmware = LinuxFirmware::spawn_named([0x0d, 0x1e, 0x2f, 0x3a, 0x4b, 0x5c])
            .expect("the mux firmware fixture should start");
        let pid = firmware.pid();

        assert!(
            process_exists(pid),
            "firmware {pid} should run right after spawn_named"
        );
        assert!(
            serial_link_ready(firmware.serial_path()),
            "spawn_named should have waited for the -I symlink"
        );

        drop(firmware);

        assert!(
            eventually(Duration::from_secs(5), || !process_exists(pid)),
            "firmware {pid} should be killed and reaped by drop"
        );
    }
}
