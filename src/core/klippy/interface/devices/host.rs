//! The real device: klipper's host library, loaded at runtime with `dlopen`.
//!
//! `libklipper_host.so` runs a Klipper firmware build as a library (see
//! `third_party/klipper/src/host/`). It has no `main()`, no stdin/stdout and no
//! peripherals: the host feeds it protocol bytes and reads protocol bytes back.
//!
//! Three things about it shape this file.
//!
//! **It is a byte stream, not a frame queue.** `klipper_host_output` hands back
//! raw protocol bytes, which may split a frame across calls or glue several
//! together, so [`HostDevice::receive`] buffers them through
//! [`FrameStream`](crate::core::klippy::frame::FrameStream) — the reassembler
//! every byte-stream device shares; `send` goes the other way and encodes a
//! frame to bytes.
//!
//! **Its main loop blocks.** `klipper_host_run` only returns once
//! `klipper_host_shutdown` is called, so it gets a thread of its own. Output is
//! pull-only, so a second thread waits in `klipper_host_output_wait` and
//! forwards what it reads to the references' receivers over a channel — when
//! that thread stops, the channel disconnects and a blocked `receive` returns
//! `None`, which is exactly [`Device::receive`]'s "no further frames" contract.
//!
//! **One library per path, several references.** `HostLibrary` is the loaded
//! library (one `dlopen`, one runtime thread, one reader), shared through the
//! same reuse spine the serial mux uses (`resource_for`);
//! each [`HostDevice`] is one communication reference on it — one per
//! `[mcu] host_library:` that names the same path. `shutdown` stops **only this
//! reference**; the library — and the firmware in it — keeps running until its
//! **last** reference is gone, at which point the threads are joined and the
//! library unloaded (a later load maps it afresh, exactly as if the board had
//! been power-cycled).
//!
//! The firmware side of sharing is **not implemented yet**: the current library
//! runs a single firmware instance, so against that old firmware only one
//! reference should do live I/O at a time. The references share one output
//! queue, and a frame is taken by whichever reference reads first — a competing
//! queue, not a broadcast — so two live readers would steal each other's frames;
//! each reference is nonetheless a fresh handle speaking the ordinary protocol.
//! What
//! already holds is the communication reuse itself: the library is loaded once,
//! several references can exist against it, and the identify handshake and
//! `get_clock` round trip run unchanged against a library that knows nothing of
//! sharing — which is what "old-firmware compatibility" means here. The bridge
//! that routes between live references will grow in the firmware later.
//!
//! **Two I/O modes.** The default mode is chosen at compile time: in a debug
//! build the library is read and written one byte per call, so every frame
//! travels the whole reassembly path against real firmware; in a release build
//! the library is asked for whole frames (`klipper_host_output_frame`, which it
//! only has when built with `CONFIG_HOST_FRAME_API`), and reassembly is skipped
//! entirely — the same thing `FrameMock` does, for the same reason: there is
//! nothing to reassemble when the boundary is already known.
//!
//! The other mode is the caller's, for a **bench scan**: `OutputMode::Bytes`,
//! which reads the library's byte stream in every build
//! ([`HostDevice::load_bytes`]). The serial mux wraps every frame in a
//! two-byte header (the developer manual's *serial-mux-protocol* page), so a
//! mux unit's first byte is a header byte, not the frame length the frame API
//! cuts the buffer by — that API cannot see a unit at all, in any build. A scan
//! therefore reads and writes the stream **as it is**
//! ([`HostDevice::read`]/[`HostDevice::write`]) and never reassembles it.

use crate::core::klippy::frame::Frame;
#[cfg(debug_assertions)]
use crate::core::klippy::frame::FrameStream;
use crate::core::klippy::interface::error::InterfaceError;
use crate::core::klippy::interface::{describe_frame, Device};
use crossbeam_channel::{unbounded, Receiver, RecvTimeoutError};
use libloading::Library;
use std::collections::HashMap;
use std::fmt;
use std::os::raw::{c_int, c_long, c_uchar};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock, Weak};
use std::thread::{self, JoinHandle};
use std::time::Duration;
use tracing::{debug, info, trace, warn};

use super::shared::resource_for;

/// The process-wide registry that makes a host library a per-path singleton:
/// references on the same path share one loaded [`HostLibrary`] (and thus one
/// runtime and one reader).
fn library_registry() -> &'static Mutex<HashMap<PathBuf, Weak<HostLibrary>>> {
    static LIBRARIES: OnceLock<Mutex<HashMap<PathBuf, Weak<HostLibrary>>>> = OnceLock::new();
    LIBRARIES.get_or_init(|| Mutex::new(HashMap::new()))
}

/// Serialises the tests that load the real host library.
///
/// The library registry is a per-path process-wide singleton, so two tests
/// sharing the one `.so` must not overlap: the second would meet — or, at the
/// end, unload — the first's library mid-handshake. A loaded library's output
/// mode and identity are fixed too ([`library_for`]), which puts a scan test
/// and an identify test on the same `.so` in exactly that pair.
#[cfg(test)]
pub(crate) static HOST_LIBRARY_TEST_LOCK: Mutex<()> = Mutex::new(());

/// Which of the library's two output APIs its reader thread reads.
///
/// A property of the **loaded library** (it picks the reader thread's read), so
/// it is chosen when the library is opened and every reference on it shares it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum OutputMode {
    /// The default: the build's own protocol path — a debug build reads the
    /// byte stream, a release build whole frames (see the module docs).
    Default,
    /// The byte stream (`klipper_host_output_wait`) in **every** build, because
    /// a mux unit is not a frame boundary (see the module docs).
    Bytes,
}

/// The live library for `path` in `mode` with `identity`, loading it (and
/// starting its threads) if the registry holds no live one.
///
/// # Errors
/// Returns [`InterfaceError`] when opening fails, and when a live library on
/// `path` disagrees with `mode` or `identity`: both are fixed when the library
/// is opened (the mode picks its reader, the identity is set before klipper
/// comes up), so reusing it under other ones would silently answer as the
/// wrong thing.
fn library_for(
    path: &Path,
    mode: OutputMode,
    identity: Option<[u8; 6]>,
) -> Result<Arc<HostLibrary>, InterfaceError> {
    let library = resource_for(library_registry(), path, |path| {
        HostLibrary::open(path, mode, identity)
    })?;
    let identity = identity.unwrap_or([0; 6]);
    if library.mode != mode || library.identity != identity {
        return Err(InterfaceError::Other(format!(
            "klipper host library {} is already loaded with a different output mode or \
             identity, which are fixed when the library is opened",
            library.library_path.display()
        )));
    }
    Ok(library)
}

// ===========================================================================
// The host library: the loaded mapping and its communication references
// ===========================================================================
//
// `libklipper_host.so`'s C API is `third_party/klipper/src/host/klipper_host.h`.
// [`HostLibrary`] is the one loaded mapping (one `dlopen`, one runtime thread,
// one reader); a [`HostDevice`] is one communication reference on it, sharing
// the library through the registry ([`library_for`]). See the module docs for
// the old-firmware reality (one live reference at a time) and where the future
// firmware bridge slots in.

// ===========================================================================
// The library's C API (third_party/klipper/src/host/klipper_host.h)
// ===========================================================================

type InitFn = unsafe extern "C" fn() -> c_int;
type InputFn = unsafe extern "C" fn(*const c_uchar, usize) -> c_long;
/// `klipper_host_set_uuid(const uint8_t uuid[6])` — the identity the device
/// answers the serial mux with, which the host sets before klipper comes up.
type SetUuidFn = unsafe extern "C" fn(*const c_uchar) -> c_int;
/// One read from the library: bytes into `buf`, up to `buf_size`, waiting up to
/// `timeout_ms`. Both the byte-stream and the frame API have this shape.
type OutputFn = unsafe extern "C" fn(*mut c_uchar, usize, u32) -> usize;
type RunFn = unsafe extern "C" fn();
type ShutdownFn = unsafe extern "C" fn();

/// Function pointers resolved out of the shared library.
///
/// Copied out of `libloading::Symbol` so they can be sent to the threads; sound
/// only while the [`Library`] they came from stays loaded, which is what
/// `HostLibrary._library` and the join in [`Drop`] guarantee.
#[derive(Clone, Copy)]
struct Symbols {
    init: InitFn,
    /// `klipper_host_set_uuid`, absent from a library built before the host
    /// could name the device — an identity is then refused rather than ignored
    /// (see [`HostLibrary::open`]).
    set_uuid: Option<SetUuidFn>,
    input: InputFn,
    /// `klipper_host_output_wait`, the byte stream, in **every** build: what a
    /// debug build reads by default and what `OutputMode::Bytes` always reads.
    output_wait: OutputFn,
    run: RunFn,
    shutdown: ShutdownFn,
}

impl Symbols {
    fn resolve(library: &Library) -> Result<Self, InterfaceError> {
        /// Resolve one symbol, naming it in the error.
        unsafe fn get<T: Copy>(library: &Library, name: &[u8]) -> Result<T, InterfaceError> {
            let symbol = unsafe { library.get::<T>(name) }.map_err(|e| {
                InterfaceError::Other(format!(
                    "klipper host library has no {}: {e}",
                    String::from_utf8_lossy(&name[..name.len() - 1])
                ))
            })?;
            Ok(*symbol)
        }

        Ok(Self {
            init: unsafe { get(library, b"klipper_host_init\0")? },
            // Optional: only a library that knows the host may name it exports
            // this. An identity asked of one that does not is an error, not a
            // silently unnamed device (see [`HostLibrary::open`]).
            set_uuid: unsafe { library.get::<SetUuidFn>(b"klipper_host_set_uuid\0") }
                .ok()
                .map(|symbol| *symbol),
            input: unsafe { get(library, b"klipper_host_input\0")? },
            output_wait: unsafe { get(library, b"klipper_host_output_wait\0")? },
            run: unsafe { get(library, b"klipper_host_run\0")? },
            shutdown: unsafe { get(library, b"klipper_host_shutdown\0")? },
        })
    }
}

/// One loaded mapping of a klipper host library, refcounted and shared by
/// every [`HostDevice`] opened on its path.
///
/// Klipper's state lives inside the library, so there is one instance per
/// loaded mapping: the one runtime thread, the one output reader, the one
/// output channel. Communication references ([`HostDevice`]) hold an `Arc` to
/// this; the library is loaded when the first reference needs it and unloaded
/// (threads joined, code unmapped) when the last one is dropped, which is also
/// what makes it safe to call `klipper_host_init` again for a later library:
/// unmapping and remapping starts the firmware from its initializers, exactly
/// as a power-cycled board would.
struct HostLibrary {
    /// Keeps the mapped code the symbols point into alive. Never read: the
    /// [`Drop`] impl joins the threads before any field is dropped, and
    /// `Drop::drop` runs ahead of field drops, so unmapping here is safe.
    _library: Library,
    symbols: Symbols,
    library_path: PathBuf,
    /// Which output API the reader thread reads (fixed when the library is
    /// opened; [`library_for`] refuses a reference that asks for the other).
    mode: OutputMode,
    /// The identity this library was given before klipper came up, all zeros
    /// when none was ([`HostLibrary::open`]). Printed by the firmware and
    /// answered to the mux, so a request for another one is refused.
    identity: [u8; 6],
    /// Raw bytes from Klipper, in arrival order. The library's own receiver is
    /// the template every [`HostDevice`] clones; it never reads it itself.
    output: Receiver<Vec<u8>>,
    /// Set by the final teardown, in [`Drop`], and polled by the reader thread
    /// it must join.
    stopped: Arc<AtomicBool>,
    threads: Mutex<Vec<JoinHandle<()>>>,
}

impl HostLibrary {
    /// Load `path`, give it `identity`, initialise klipper and start the two
    /// threads; `mode` picks the output API the reader thread reads.
    ///
    /// Shared by every reference on the same path; called by
    /// [`library_for`] only when no live library is registered for it yet.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] when the library cannot be loaded, does not
    /// export the expected symbols, or fails to initialize — and when an
    /// identity is asked of a library built without `klipper_host_set_uuid`.
    /// The runtime thread is stopped and joined again if the output reader
    /// cannot be spawned, so no thread is left inside a library that is about
    /// to be unmapped.
    fn open(
        path: &Path,
        mode: OutputMode,
        identity: Option<[u8; 6]>,
    ) -> Result<Arc<Self>, InterfaceError> {
        let library_path = path.to_path_buf();
        let library = unsafe { Library::new(&library_path) }.map_err(|e| {
            InterfaceError::Other(format!(
                "failed to load klipper host library {}: {e}",
                library_path.display()
            ))
        })?;

        let symbols = Symbols::resolve(&library)?;

        // The identity is fixed once the device is up, so it is set here,
        // before `init` — and only when the host names one: all zeros is what
        // the library starts with anyway.
        let identity = identity.unwrap_or([0; 6]);
        if identity != [0; 6] {
            let set_uuid = match symbols.set_uuid {
                Some(set_uuid) => set_uuid,
                None => {
                    return Err(InterfaceError::Other(format!(
                        "klipper host library {} does not support an identity: it was built \
                         without klipper_host_set_uuid",
                        library_path.display()
                    )))
                }
            };
            let result = unsafe { set_uuid(identity.as_ptr()) };
            if result != 0 {
                return Err(InterfaceError::Other(format!(
                    "klipper_host_set_uuid() returned {result}"
                )));
            }
        }

        // Ready klipper's globals. A fresh mapping has no shutdown latch to
        // clear, but on a library that was unloaded and mapped again `init` is
        // exactly the reset a power-cycled board would have had.
        let result = unsafe { (symbols.init)() };
        if result != 0 {
            return Err(InterfaceError::Other(format!(
                "klipper_host_init() returned {result}"
            )));
        }

        let (output_tx, output) = unbounded::<Vec<u8>>();
        let stopped = Arc::new(AtomicBool::new(false));
        let mut threads = Vec::with_capacity(2);

        // Klipper's main loop blocks until shutdown, and drives everything else,
        // so it has to be its own thread.
        let run = symbols.run;
        threads.push(
            thread::Builder::new()
                .name("klipper-runtime".to_string())
                .spawn(move || {
                    info!("klipper runtime started");
                    unsafe { run() };
                    info!("klipper runtime returned");
                })
                .map_err(|e| InterfaceError::Other(format!("failed to spawn runtime: {e}")))?,
        );

        // Output is pull-only, so it gets a thread of its own. That thread owns
        // the sender, so when it stops the channel disconnects and every blocked
        // `receive` wakes up with `None` — no separate "unblock" signal needed.
        // It holds only a clone of `stopped` (and no `Arc<HostLibrary>`), so it
        // can never keep the library alive on its own.
        let poller_stopped = Arc::clone(&stopped);
        // The reader's read is the mode's: the byte stream in a debug build and
        // in `OutputMode::Bytes`, whole frames in a release build's default mode.
        // The frame API only exists in a library built with
        // CONFIG_HOST_FRAME_API, and **only the default mode asks for it**: a
        // bytes-mode load (the bench scan) reads the stream, so it loads
        // against the byte-stream API that every build has.
        #[cfg(debug_assertions)]
        let poller_read = symbols.output_wait;
        #[cfg(not(debug_assertions))]
        let poller_read = match mode {
            OutputMode::Bytes => symbols.output_wait,
            OutputMode::Default => unsafe {
                library
                    .get::<OutputFn>(b"klipper_host_output_frame\0")
                    .map_err(|e| {
                        InterfaceError::Other(format!(
                            "klipper host library has no klipper_host_output_frame: rebuild it \
                             with CONFIG_HOST_FRAME_API=y, or use a debug build ({e})"
                        ))
                    })
                    .map(|symbol| *symbol)?
            },
        };
        let poller = thread::Builder::new()
            .name("klipper-output".to_string())
            .spawn(move || {
                let mut buf = vec![0u8; OUTPUT_BATCH];
                while !poller_stopped.load(Ordering::Relaxed) {
                    // Blocks in the library until data arrives or the wait expires,
                    // so no polling interval is needed here. How much arrives is
                    // the mode's business: a byte in a debug build, a whole frame in a
                    // release build.
                    let read = unsafe { poller_read(buf.as_mut_ptr(), buf.len(), OUTPUT_WAIT_MS) };
                    if read == 0 {
                        continue;
                    }
                    if output_tx.send(buf[..read].to_vec()).is_err() {
                        break; // nobody is receiving any more
                    }
                }
                debug!("klipper output reader stopped");
            });
        match poller {
            Ok(handle) => threads.push(handle),
            Err(e) => {
                // The runtime thread is already inside klipper. Stop it and wait
                // for it, because the `Library` below is about to be unloaded and
                // it must not be executing library code when that happens.
                unsafe { (symbols.shutdown)() };
                for handle in threads.drain(..) {
                    let _ = handle.join();
                }
                return Err(InterfaceError::Other(format!(
                    "failed to spawn poller: {e}"
                )));
            }
        }

        info!(
            "klipper host library loaded from {}",
            library_path.display()
        );
        Ok(Arc::new(Self {
            _library: library,
            symbols,
            library_path,
            mode,
            identity,
            output,
            stopped,
            threads: Mutex::new(threads),
        }))
    }
}

impl Drop for HostLibrary {
    fn drop(&mut self) {
        // Stops the runtime loop and wakes the reader; then both threads are
        // joined before `_library` is dropped — `dlclose` while a thread is
        // still executing library code would unmap it under that thread's feet.
        self.stopped.store(true, Ordering::SeqCst);
        unsafe { (self.symbols.shutdown)() };
        for handle in self.threads.lock().unwrap().drain(..) {
            if let Err(e) = handle.join() {
                warn!("klipper worker thread panicked: {e:?}");
            }
        }
    }
}

/// Bytes asked of the library per read.
///
/// A debug build reads a byte at a time, which is the hardest case for
/// reassembly: no read ever contains a whole frame, so every frame goes through
/// [`FrameStream`]. A release build asks for one frame at a time, and
/// [`MESSAGE_MAX`](crate::core::klippy::frame::MESSAGE_MAX) is the largest one
/// the protocol can produce — and a whole mux unit fits, so it is the batch a
/// release build's bytes-mode reader asks for too.
#[cfg(debug_assertions)]
const OUTPUT_BATCH: usize = 1;
#[cfg(not(debug_assertions))]
const OUTPUT_BATCH: usize = crate::core::klippy::frame::MESSAGE_MAX;

/// The next reference number, used to tell one handle from another in logs.
static NEXT_REFERENCE: AtomicUsize = AtomicUsize::new(1);

/// A [`Device`] backed by klipper's host library.
///
/// One communication reference on the shared `HostLibrary` for its path: it
/// holds the library alive and talks to the single firmware instance inside it.
/// `shutdown` stops **only this reference** — the library, and every other
/// reference on it, stays up until the last one is dropped.
///
/// [`Device`]: crate::core::klippy::interface::Device
pub struct HostDevice {
    /// The shared, loaded library this reference is on.
    library: Arc<HostLibrary>,
    /// This reference's number, to tell the handles on one library apart.
    reference: usize,
    /// Raw bytes from Klipper, in arrival order, for this reference.
    ///
    /// A clone of the library's one receiver over its one output channel: with
    /// the firmware's bridge unimplemented, all references share the single
    /// output queue and each frame goes to whichever reference reads it first
    /// (a competing queue, not a broadcast), so only one is live against old
    /// firmware at a time (see the module docs).
    output: Receiver<Vec<u8>>,
    /// Set by this reference's `shutdown`, checked by its `send`, `receive`
    /// and the poll loop. This is per-handle: the library's own `stopped` is
    /// set only by the final teardown.
    stopped: AtomicBool,
    /// Bytes pulled from `output`, reassembled into frames.
    ///
    /// A release build is handed whole frames and never needs this.
    #[cfg(debug_assertions)]
    stream: Mutex<FrameStream>,
    /// Bytes pulled from `output` by [`HostDevice::read`] but not handed out
    /// yet: one library read can deliver more than the caller's buffer holds.
    raw: Mutex<Vec<u8>>,
}

impl HostDevice {
    /// Returns a short identifier for logging: `"«path»<«reference»>"`.
    fn id(&self) -> String {
        format!(
            "{}<{}>",
            self.library.library_path.display(),
            self.reference
        )
    }

    /// Open a communication reference on the klipper host library at `path`.
    ///
    /// The library is loaded **once per path**: the first `load` dlopens it and
    /// starts its runtime and reader threads; later `load`s on the same path
    /// return further references on the same loaded library (`shared`'s
    /// one-port-many-handles split). The last reference to be dropped unloads
    /// it — threads joined, code unmapped — so a later `load` starts a fresh
    /// mapping, exactly as if the board had been power-cycled.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] when the library cannot be loaded, does not
    /// export the expected symbols, or fails to initialize.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, InterfaceError> {
        Self::open(path.as_ref(), OutputMode::Default, None)
    }

    /// Open a communication reference on the klipper host library at `path`
    /// **for a bench scan**: its reader reads the library's byte stream in every
    /// build (`OutputMode::Bytes`), so whole units can be read and written
    /// ([`HostDevice::read`]/[`HostDevice::write`]) — a mux unit is not a frame
    /// the frame API could hand over (see the module docs).
    ///
    /// `identity` is what the library answers the serial mux with, set before
    /// klipper comes up; `None` leaves it at the library's own default (all
    /// zeros), and a library built without `klipper_host_set_uuid` is only an
    /// error when an identity is actually asked for.
    ///
    /// The identity and the mode are fixed once the library is loaded, so
    /// asking for another one on a path whose library is already live fails
    /// (`library_for`) instead of answering as the wrong device.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] for the reasons [`HostDevice::load`] does,
    /// and when `identity` is asked of a library that has no
    /// `klipper_host_set_uuid`.
    pub fn load_bytes(
        path: impl AsRef<Path>,
        identity: Option<[u8; 6]>,
    ) -> Result<Self, InterfaceError> {
        Self::open(path.as_ref(), OutputMode::Bytes, identity)
    }

    /// Wrap the live library on `path` as a reference in `mode`.
    fn open(
        path: &Path,
        mode: OutputMode,
        identity: Option<[u8; 6]>,
    ) -> Result<Self, InterfaceError> {
        let library = library_for(path, mode, identity)?;
        let reference = NEXT_REFERENCE.fetch_add(1, Ordering::Relaxed);
        Ok(Self {
            output: library.output.clone(),
            library,
            reference,
            stopped: AtomicBool::new(false),
            #[cfg(debug_assertions)]
            stream: Mutex::new(FrameStream::new()),
            raw: Mutex::new(Vec::new()),
        })
    }

    /// Path of the library this reference was loaded from.
    pub fn library_path(&self) -> &Path {
        &self.library.library_path
    }

    /// Read raw protocol bytes from the library, for a bench scan.
    ///
    /// The byte stream as the library hands it over, without frame reassembly:
    /// a mux unit's two-byte header is not a frame, so a scan has to see the
    /// stream as it is (`OutputMode::Bytes`). A read blocks at most
    /// `OUTPUT_WAIT_MS` and returns `0` when nothing arrived in that window —
    /// the same contract as
    /// [`SerialDevice::read`](crate::core::klippy::interface::devices::serial::SerialDevice::read),
    /// so the same scan code runs over either transport.
    pub fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        if self.stopped.load(Ordering::Relaxed) {
            return Ok(0); // no further bytes will arrive
        }
        let mut pending = self.raw.lock().unwrap();
        if pending.is_empty() {
            match self
                .output
                .recv_timeout(Duration::from_millis(OUTPUT_WAIT_MS as u64))
            {
                Ok(bytes) => *pending = bytes,
                Err(RecvTimeoutError::Timeout) => return Ok(0),
                Err(RecvTimeoutError::Disconnected) => {
                    return Err(std::io::Error::other("klipper host library stopped"))
                }
            }
        }
        let read = buf.len().min(pending.len());
        buf[..read].copy_from_slice(&pending[..read]);
        pending.drain(..read);
        Ok(read)
    }

    /// Write raw protocol bytes to the library, for a bench scan.
    ///
    /// The unit as the mux puts it on the wire, header and all; the frame
    /// path's [`Device::send`] cannot write one, because a unit is not an
    /// encoded [`Frame`].
    pub fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        if self.stopped.load(Ordering::Relaxed) {
            return Err(std::io::Error::other(
                "klipper host library reference is shut down",
            ));
        }
        let written = unsafe { (self.library.symbols.input)(buf.as_ptr(), buf.len()) };
        if written != buf.len() as c_long {
            return Err(std::io::Error::other(format!(
                "klipper_host_input took {written} of {} bytes",
                buf.len()
            )));
        }
        Ok(written as usize)
    }
}

impl std::io::Read for HostDevice {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        HostDevice::read(self, buf)
    }
}

impl std::io::Write for HostDevice {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        HostDevice::write(self, buf)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        // Nothing is buffered on this side: a write is handed to klipper's
        // console buffer whole.
        Ok(())
    }
}

impl fmt::Debug for HostDevice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("HostDevice")
            .field("library", &self.library.library_path)
            .field("reference", &self.reference)
            .field("stopped", &self.stopped.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Device for HostDevice {
    fn send(&self, frame: &Frame) -> Result<(), InterfaceError> {
        if self.stopped.load(Ordering::Relaxed) {
            return Err(InterfaceError::ConnectionLost);
        }

        let bytes = frame.raw_bytes();
        trace!("tx frame [{}]: {}", self.id(), describe_frame(&bytes));

        // A debug build hands the frame over one byte per call, so the library's
        // receive path sees the stream the way a real link would deliver it. A
        // release build gives it the frame in one call.
        #[cfg(debug_assertions)]
        let written = {
            let mut taken = 0;
            for byte in &bytes {
                let n = unsafe { (self.library.symbols.input)(byte as *const c_uchar, 1) };
                if n != 1 {
                    break;
                }
                taken += n;
            }
            taken
        };
        #[cfg(not(debug_assertions))]
        let written = unsafe { (self.library.symbols.input)(bytes.as_ptr(), bytes.len()) };

        if written != bytes.len() as c_long {
            return Err(InterfaceError::SendError(format!(
                "klipper_host_input took {written} of {} bytes",
                bytes.len()
            )));
        }
        debug!("sent {} bytes to klipper", bytes.len());
        Ok(())
    }

    fn receive(&self) -> Option<Frame> {
        // A bytes-mode library hands over the mux byte stream, in which a unit
        // is not a frame — framing it here would read the unit's header byte as
        // a length. That mode belongs to the bench scan, which reads
        // [`HostDevice::read`] instead.
        if self.library.mode == OutputMode::Bytes {
            return None;
        }
        loop {
            if self.stopped.load(Ordering::Relaxed) {
                return None;
            }
            // A debug build is fed a byte stream, so the frame is logged here,
            // once it has been reassembled, instead of a raw byte at a time on
            // arrival — the way the serial port logs one whole frame.
            #[cfg(debug_assertions)]
            if let Some(frame) = self.stream.lock().unwrap().next_frame() {
                let bytes = frame.raw_bytes();
                trace!("rx frame [{}]: {}", self.id(), describe_frame(&bytes));
                debug!("received {} bytes from klipper", bytes.len());
                return Some(frame);
            }
            // `recv_timeout` rather than `recv`, so this reference's own
            // `shutdown` is noticed within one interval even though the library
            // — and its reader — keeps running for other references.
            match self
                .output
                .recv_timeout(Duration::from_millis(OUTPUT_WAIT_MS as u64))
            {
                Ok(bytes) => {
                    // A debug build reassembles the stream; a release build is
                    // handed whole frames and only has to validate them.
                    #[cfg(debug_assertions)]
                    self.stream.lock().unwrap().push(&bytes);
                    #[cfg(not(debug_assertions))]
                    {
                        trace!("rx frame [{}]: {}", self.id(), describe_frame(&bytes));
                        debug!("received {} bytes from klipper", bytes.len());
                        match Frame::parse(&bytes) {
                            Some(frame) => return Some(frame),
                            None => {
                                warn!("klipper sent {} bytes that are not a frame", bytes.len())
                            }
                        }
                    }
                }
                // Nothing arrived this interval: re-check `stopped` and wait again.
                Err(RecvTimeoutError::Timeout) => {}
                // The reader stopped: the stream is over.
                Err(RecvTimeoutError::Disconnected) => return None,
            }
        }
    }

    fn shutdown(&self) {
        if self.stopped.swap(true, Ordering::SeqCst) {
            return; // already shut down
        }
        // The reader keeps running — this only stops this reference: its
        // `receive` loop notices `stopped` within one wait interval, and the
        // library, and every other reference, carries on.
        #[cfg(debug_assertions)]
        drop(self.stream.lock().unwrap());
    }
}

// How long the output thread blocks in the library when there is nothing to
// read. Data wakes it immediately, so this only bounds how often it rechecks
// the stop flag — and klipper's own shutdown wakes it too.
//
// A reference's `receive` uses the same value as its polling interval, so a
// per-reference `shutdown` is noticed within one wait, even though the
// library's reader (and every other reference) keeps running.
const OUTPUT_WAIT_MS: u32 = 100;

#[cfg(test)]
mod host_tests {
    use super::*;
    use crate::core::klippy::config::mcu::McuConfig;
    use crate::core::klippy::config::{ConfigSection, ConfigValue, ConfigWrapper};
    use crate::core::klippy::mcu::Mcu;
    use crate::core::klippy::msg::proto::ArgValue;
    use crate::core::klippy::Interface;

    #[test]
    fn test_load_reports_a_missing_library() {
        let err = HostDevice::load("/nonexistent/libklipper_host.so").unwrap_err();
        match err {
            InterfaceError::Other(message) => {
                assert!(message.contains("failed to load"), "{message}");
                assert!(message.contains("libklipper_host"), "{message}");
            }
            other => panic!("expected Other, got {other:?}"),
        }
    }

    #[test]
    fn test_a_live_library_is_refused_in_another_mode_or_with_another_identity() {
        // The mode picks the reader thread's API and the identity is fixed when
        // klipper comes up, so a reference asking for either on a live library
        // is refused rather than served as the wrong device. A bytes-mode
        // reference is what a bench scan opens (`src/scan.rs`).
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let named =
            HostDevice::load_bytes(library_path(), Some([0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc]))
                .expect("the real library takes an identity");

        let err = HostDevice::load_bytes(library_path(), Some([0, 0, 0, 0, 0, 1]))
            .expect_err("another identity is not this library's");
        match err {
            InterfaceError::Other(message) => {
                assert!(message.contains("already loaded"), "{message}")
            }
            other => panic!("expected Other, got {other:?}"),
        }

        let err = HostDevice::load(library_path()).expect_err("the default mode is not the scan's");
        match err {
            InterfaceError::Other(message) => {
                assert!(message.contains("already loaded"), "{message}")
            }
            other => panic!("expected Other, got {other:?}"),
        }

        // Dropping the last reference unloads it, so the refusal was only about
        // the live library, not about the path.
        drop(named);
        HostDevice::load(library_path()).expect("the library is not loaded any more");
    }

    /// The host library is only built for tests, through `klipperx-test-support`.
    fn library_path() -> PathBuf {
        klipperx_test_support::klipper_host_lib_path()
    }

    /// An `[mcu]` section pointing at the library, so the test configures its MCU
    /// the way a user's config file would.
    fn mcu_section() -> ConfigSection {
        let mut section = ConfigSection::new("mcu", Some("host_test"));
        section.parameters.insert(
            "host_library".to_string(),
            ConfigValue::Single(library_path().display().to_string()),
        );
        section
    }

    /// Connect, identify, and prove the connection works with a `get_clock`
    /// round trip — the ordinary handshake against the real library, over the
    /// caller's `interface` (the one reference of the connection; it is what
    /// [`Device::shutdown`] later stops).
    async fn identify_and_clock(config: &McuConfig, interface: &Interface) -> Arc<Mcu> {
        let mcu = tokio::time::timeout(
            Duration::from_secs(20),
            Mcu::connect(&config.name, interface.clone()),
        )
        .await
        .expect("identify handshake timed out")
        .expect("identify handshake failed");

        assert!(mcu.is_identified());
        let dictionary = mcu.dictionary().expect("a dictionary was installed");

        // A 40 byte chunk cannot carry the compressed payload (it is ~700 bytes),
        // so reaching this point means the chunk loop reassembled several of them
        // - more than the sequence counter has values, since it wraps at 16.
        let message_count = dictionary.messages().count();
        assert!(
            message_count >= 20,
            "expected the firmware's whole dictionary, got {message_count} messages"
        );

        // The clock frequency is fixed by the configuration `test-support` builds
        // the library with, so it pins both the dictionary and that config.
        assert_eq!(dictionary.constant_f64("CLOCK_FREQ"), Some(20_000_000.0));

        // The two commands used below have to be there, and the host's own
        // definitions have to have survived the install: the identify pair is in
        // the firmware dictionary too, and re-registering it would have failed.
        let get_clock = dictionary.message("get_clock").expect("get_clock");
        let clock = dictionary.message("clock").expect("clock");
        assert_eq!(dictionary.message("identify").unwrap().id, 1);
        debug!(
            "firmware dictionary: {message_count} messages, get_clock id {}, clock id {}",
            get_clock.id, clock.id
        );

        // ... and the dictionary is not just installed but usable: `get_clock` is
        // answered with the low 32 bits of the firmware's tick counter. The ids are
        // deliberately not asserted - the point of the dictionary is that the host
        // does not care what they are.
        let params = mcu
            .call("get_clock", &[], "clock", Duration::from_secs(1))
            .await
            .expect("get_clock after identify");
        assert_eq!(params.len(), 1);
        assert!(
            matches!(params[0], ArgValue::UInt32(_)),
            "clock should be a `%u` value, got {:?}",
            params[0]
        );
        debug!("firmware clock reads {:?}", params[0]);

        mcu
    }

    /// Wait — bounded — for the shared host library to be unloaded.
    ///
    /// Dropping the last `Interface` does not unload it the very moment the
    /// caller lets go: a transport task that [`Mcu::close`] aborted may still
    /// hold the last `Arc<HostDevice>` — and thus the last `Arc<HostLibrary>` —
    /// for a moment while it unwinds. The unload happens only when every such
    /// reference is gone ([`HostLibrary`]'s `Drop`), which is what polling the
    /// registry's [`Weak`] entry observes. Bounds a hang in a test that would
    /// otherwise leave the library loaded into the next test (which must meet a
    /// fresh firmware, not this one's session).
    async fn wait_for_unload() {
        for _ in 0..200 {
            let dead = library_registry()
                .lock()
                .unwrap()
                .get(library_path().as_path())
                .and_then(Weak::upgrade)
                .is_none();
            if dead {
                return;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        panic!("the host library never unloaded");
    }

    /// The identify handshake against a real klipper inside the library, on a
    /// shared, load-once library.
    ///
    /// This is the whole bootstrap on real firmware: ask for the payload chunk by
    /// chunk, reassemble it, decompress it, parse it into a [`Dictionary`], install
    /// that, and then use what it installed. It is the only test that starts from
    /// bytes produced by klipper's own build rather than by a fixture, which is
    /// what makes it worth its cost: the ids, the format strings, and the clock
    /// frequency all come from the firmware.
    ///
    /// [`Dictionary`]: crate::core::klippy::mcu::Dictionary
    #[tokio::test]
    async fn test_identify_against_the_real_library() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let config =
            McuConfig::new(&ConfigWrapper::untracked(&mcu_section())).expect("the [mcu] section");
        assert_eq!(config.name, "host_test");
        // First reference: this is what loads the library — once.
        let interface = config.open().expect("the host library opens");

        let mcu = identify_and_clock(&config, &interface).await;

        // Dropping the MCU shuts this reference down but not the library: the
        // `interface` above still holds it, so the firmware keeps running.
        drop(mcu);
        assert_eq!(interface.receive().await, None);

        // A second reference in the same process finds the firmware exactly where
        // the first one left it, because it is the **same loaded library**: no
        // re-dlopen, no fresh initializers. klipper's sequence counter is one
        // static that only an accepted block moves (`src/command.c:16,301-305`),
        // and shutting a reference down does not clear it. That is the situation
        // any MCU is in when it was never power-cycled — the one
        // `restart_method: command` has to work in — and the transport takes that
        // session over instead of stalling on a sequence that does not start over.
        let interface2 = config.open().expect("the host library is still loaded");
        let mcu2 = identify_and_clock(&config, &interface2).await;
        assert!(
            mcu2.took_over_session(),
            "the firmware was still in the session the first connection left"
        );

        // And the adopted number is the one the firmware answers on: the same
        // command works over it.
        let params = mcu2
            .call("get_clock", &[], "clock", Duration::from_secs(1))
            .await
            .expect("get_clock after the takeover");
        assert!(
            matches!(params[0], ArgValue::UInt32(_)),
            "clock should be a `%u` value, got {:?}",
            params[0]
        );

        drop(mcu2);
        assert_eq!(interface2.receive().await, None);

        // With both references gone the library unloads with them. The next host
        // test must meet a *fresh* firmware (not this one's session), so wait
        // for the unload before returning.
        drop(interface2);
        drop(interface);
        wait_for_unload().await;
    }

    /// A library is unloaded when its last reference is dropped, and the next
    /// load maps it afresh: the firmware boots like a power-cycled board — the
    /// ordinary first-connection handshake, not a session takeover.
    ///
    /// This is the foil to [`test_identify_against_the_real_library`]: there a
    /// held reference keeps the one mapping (and its session) alive; here no
    /// reference survives, so the second connection meets a freshly loaded
    /// firmware whose sequence starts over.
    #[tokio::test]
    async fn test_the_library_unloads_with_its_last_reference() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let config =
            McuConfig::new(&ConfigWrapper::untracked(&mcu_section())).expect("the [mcu] section");

        // First lifetime: connect, identify, run a command, drop everything.
        let interface = config.open().expect("the host library opens");
        let mcu = identify_and_clock(&config, &interface).await;
        // A freshly loaded library is a freshly booted board: no session to adopt.
        assert!(!mcu.took_over_session(), "first boot is a fresh session");
        drop(mcu);
        assert_eq!(interface.receive().await, None);
        drop(interface);
        // The last reference is gone (the aborted transport tasks unwind, then
        // [`HostLibrary`]'s `Drop` joins the threads and unmaps the library): the
        // registry entry is dead, and a new open cannot reuse it.
        wait_for_unload().await;
        let dead = library_registry()
            .lock()
            .unwrap()
            .get(library_path().as_path())
            .and_then(Weak::upgrade)
            .is_none();
        assert!(
            dead,
            "the library should have been unloaded with its last reference"
        );

        // Second lifetime: a fresh mapping, so another ordinary first-connection
        // handshake — the firmware does not remember the first session.
        let interface = config.open().expect("the host library reloads");
        let mcu = identify_and_clock(&config, &interface).await;
        assert!(
            !mcu.took_over_session(),
            "a reloaded library is a freshly booted board"
        );
        drop(mcu);
        assert_eq!(interface.receive().await, None);
        drop(interface);
        wait_for_unload().await;
    }
}
