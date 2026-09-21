//! The real device: klipper's host library, loaded at runtime with `dlopen`.
//!
//! `libklipper_host.so` runs a Klipper firmware build as a library (see
//! `third_party/klipper/src/host/`). It has no main(), no stdin/stdout, and no
//! peripherals: the host feeds it protocol bytes and reads protocol bytes back.
//!
//! Three things about it shape this file.
//!
//! **It is a byte stream, not a frame queue.** `klipper_host_output` hands back
//! raw protocol bytes, which may split a frame across calls or glue several
//! together, so [`HostDevice::receive`] buffers them through
//! [`FrameStream`](crate::core::klippy::frame::FrameStream) — the reassembler every
//! byte-stream device shares; `send` goes
//! the other way and encodes a frame to bytes.
//! Neither side needs to know about the other's boundaries.
//!
//! **Its main loop blocks.** `klipper_host_run` only returns once
//! `klipper_host_shutdown` is called, so it gets a thread of its own. Output is
//! pull-only, so a second thread waits in `klipper_host_output_wait` and forwards
//! what it reads to [`HostDevice::receive`] over a channel — when that thread
//! stops, the channel disconnects and a blocked `receive` returns `None`, which
//! is exactly the "no further frames will ever arrive" contract of
//! [`Device::receive`].
//!
//! ```text
//!   send(Frame) ─► klipper_host_input ──────┐
//!                                           ▼
//!                                    klipper runtime thread
//!                                     (klipper_host_run)
//!                                           │
//!   receive() ◄── framing ◄── channel ◄── reader thread
//!                              (klipper_host_output_wait)
//! ```
//!
//! Both directions can block, and both are all-or-nothing: `klipper_host_input`
//! waits for room rather than truncating a frame, and the library grows its
//! output buffer rather than dropping a response, so a slow host costs latency
//! and not a desynchronised stream.
//!
//! **Two I/O modes, chosen at compile time.** In a debug build the library is
//! read and written one byte per call, so every frame travels the whole
//! reassembly path ([`FrameStream`](crate::core::klippy::frame::FrameStream))
//! against real firmware. In a release build the
//! library is asked for whole frames (`klipper_host_output_frame`, which it only
//! has when built with `CONFIG_HOST_FRAME_API`), and reassembly is skipped
//! entirely — the same thing `TestDevice` does, for the
//! same reason: there is nothing to reassemble when the boundary is already known.
//!
//! **Its state is process-global.** Klipper's globals live inside the shared
//! library, so one device per process is the supported configuration, and the
//! threads it spawns must be joined before the library is unloaded — hence the
//! explicit teardown in [`Drop`].

use crate::core::klippy::frame::Frame;
#[cfg(debug_assertions)]
use crate::core::klippy::frame::FrameStream;
use crate::core::klippy::interface::error::InterfaceError;
use crate::core::klippy::interface::{describe_frame, Device};
use crossbeam_channel::{unbounded, Receiver};
use libloading::Library;
use std::fmt;
use std::os::raw::{c_int, c_long, c_uchar};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use tracing::{debug, info, trace, warn};

/// Bytes asked of the library per read.
///
/// A debug build reads a byte at a time, which is the hardest case for
/// reassembly: no read ever contains a whole frame, so every frame goes through
/// [`FrameStream`]. A release build asks for one frame at a time, and
/// [`MESSAGE_MAX`](crate::core::klippy::frame::MESSAGE_MAX) is the largest one
/// the protocol can produce.
#[cfg(debug_assertions)]
const OUTPUT_BATCH: usize = 1;
#[cfg(not(debug_assertions))]
const OUTPUT_BATCH: usize = crate::core::klippy::frame::MESSAGE_MAX;

/// How long the output thread blocks in the library when there is nothing to
/// read. Data wakes it immediately, so this only bounds how often it rechecks
/// the stop flag — and klipper's own shutdown wakes it too.
const OUTPUT_WAIT_MS: u32 = 100;

// ===========================================================================
// The library's C API (third_party/klipper/src/host/klipper_host.h)
// ===========================================================================

type InitFn = unsafe extern "C" fn() -> c_int;
type InputFn = unsafe extern "C" fn(*const c_uchar, usize) -> c_long;
/// One read from the library: bytes into `buf`, up to `buf_size`, waiting up to
/// `timeout_ms`. Both the byte-stream and the frame API have this shape.
type OutputFn = unsafe extern "C" fn(*mut c_uchar, usize, u32) -> usize;
type RunFn = unsafe extern "C" fn();
type ShutdownFn = unsafe extern "C" fn();

/// Function pointers resolved out of the shared library.
///
/// Copied out of `libloading::Symbol` so they can be sent to the threads; sound
/// only while the [`Library`] they came from stays loaded, which is what
/// `HostDevice::library` and the join in [`Drop`] guarantee.
#[derive(Clone, Copy)]
struct Symbols {
    init: InitFn,
    input: InputFn,
    /// `klipper_host_output_wait` in a debug build, `klipper_host_output_frame` in a
    /// release build — see [`Symbols::resolve`].
    output: OutputFn,
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
            input: unsafe { get(library, b"klipper_host_input\0")? },
            // A debug build takes the byte stream apart itself; a release build
            // wants the library's frame API, which exists only when it was built
            // with CONFIG_HOST_FRAME_API.
            #[cfg(debug_assertions)]
            output: unsafe { get(library, b"klipper_host_output_wait\0")? },
            #[cfg(not(debug_assertions))]
            output: unsafe {
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
            run: unsafe { get(library, b"klipper_host_run\0")? },
            shutdown: unsafe { get(library, b"klipper_host_shutdown\0")? },
        })
    }
}

// ===========================================================================
// HostDevice
// ===========================================================================

/// Klipper's state lives inside the library, not in this struct, so a process
/// gets one device at a time. The claim is released when the device is dropped,
/// which is also what makes it safe to call `klipper_host_init` again for a
/// later device: the alternative — initialising once per process — would hand
/// out a device whose runtime thread returns immediately, because the previous
/// shutdown is still latched in klipper's globals.
static ACTIVE: AtomicBool = AtomicBool::new(false);

/// RAII guard for the process-wide device slot.
#[derive(Debug)]
struct Claim;

impl Claim {
    fn take() -> Result<Self, InterfaceError> {
        if ACTIVE.swap(true, Ordering::SeqCst) {
            return Err(InterfaceError::Other(
                "a klipper host device is already running in this process".to_string(),
            ));
        }
        Ok(Self)
    }
}

impl Drop for Claim {
    fn drop(&mut self) {
        ACTIVE.store(false, Ordering::SeqCst);
    }
}

/// A [`Device`] backed by klipper's host library.
///
/// [`Device`]: crate::core::klippy::interface::Device
pub struct HostDevice {
    /// Held for the lifetime of the device, so a second one is refused.
    _claim: Claim,
    symbols: Symbols,
    /// Keeps the mapped code the symbols point into alive. Never read: it is
    /// dropped last, after the threads have been joined.
    _library: Arc<Library>,
    library_path: PathBuf,
    /// Raw bytes from Klipper, in arrival order.
    output: Receiver<Vec<u8>>,
    /// Bytes pulled from `output`, reassembled into frames.
    ///
    /// A release build is handed whole frames and never needs this.
    #[cfg(debug_assertions)]
    stream: Mutex<FrameStream>,
    /// Set by `shutdown`, checked by both worker threads and by `receive`.
    stopped: Arc<AtomicBool>,
    threads: Mutex<Vec<JoinHandle<()>>>,
}

impl HostDevice {
    /// Returns a short identifier for logging: `"/path/to/libklipper_host.so"`.
    fn id(&self) -> String {
        self.library_path.display().to_string()
    }

    /// Load a klipper host library and start it.
    ///
    /// Klipper's state lives in the library rather than in this struct, so a
    /// process holds one device at a time: a second `load` while one is alive
    /// fails, and dropping the device frees the slot for the next one.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] when the library cannot be loaded, does not
    /// export the expected symbols, or fails to initialize.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, InterfaceError> {
        let library_path = path.as_ref().to_path_buf();
        let library = Arc::new(unsafe { Library::new(&library_path) }.map_err(|e| {
            InterfaceError::Other(format!(
                "failed to load klipper host library {}: {e}",
                library_path.display()
            ))
        })?);

        let symbols = Symbols::resolve(&library)?;
        let claim = Claim::take()?;

        // Resets klipper's globals, including the shutdown latch left behind by a
        // previous device — the claim above is what makes this safe to do again.
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
        let poller_stopped = Arc::clone(&stopped);
        let poller_read = symbols.output;
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

        Ok(Self {
            _claim: claim,
            symbols,
            _library: library,
            library_path,
            output,
            #[cfg(debug_assertions)]
            stream: Mutex::new(FrameStream::new()),
            stopped,
            threads: Mutex::new(threads),
        })
    }

    /// Path of the library this device was loaded from.
    pub fn library_path(&self) -> &Path {
        &self.library_path
    }
}

impl fmt::Debug for HostDevice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("HostDevice")
            .field("library", &self.library_path)
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
                let n = unsafe { (self.symbols.input)(byte as *const c_uchar, 1) };
                if n != 1 {
                    break;
                }
                taken += n;
            }
            taken
        };
        #[cfg(not(debug_assertions))]
        let written = unsafe { (self.symbols.input)(bytes.as_ptr(), bytes.len()) };

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
            match self.output.recv() {
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
                // The poller stopped: the stream is over.
                Err(_) => return None,
            }
        }
    }

    fn shutdown(&self) {
        if self.stopped.swap(true, Ordering::SeqCst) {
            return; // already shut down
        }
        // Wakes the thread blocked in `klipper_host_run`, which is our runtime
        // thread; the poller notices `stopped` within one poll interval.
        unsafe { (self.symbols.shutdown)() };
    }
}

impl Drop for HostDevice {
    fn drop(&mut self) {
        self.shutdown();
        // Must join before `library` is dropped: `dlclose` while a thread is
        // still executing library code would unmap it under that thread's feet.
        for handle in self.threads.lock().unwrap().drain(..) {
            if let Err(e) = handle.join() {
                warn!("klipper worker thread panicked: {e:?}");
            }
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::mcu::McuConfig;
    use crate::core::klippy::config::{ConfigSection, ConfigValue, ConfigWrapper};
    use crate::core::klippy::mcu::Mcu;
    use crate::core::klippy::msg::proto::ArgValue;
    use std::time::Duration;

    // -----------------------------------------------------------------------
    // The library itself
    // -----------------------------------------------------------------------

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

    /// Open the host library again, once the process-wide slot is free.
    ///
    /// One device at a time is the library's rule, and the previous one is only
    /// really gone a moment after it is dropped: `Mcu::drop` shuts the device down
    /// and the transport's tasks end afterwards, which is when their clones of the
    /// interface go. So the wait is on the *task* teardown, not on the drop.
    async fn reopen(config: &McuConfig) -> crate::core::klippy::Interface {
        for _ in 0..200 {
            match config.open() {
                Ok(interface) => return interface,
                Err(e) => {
                    debug!("host library not free yet: {e}");
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
            }
        }
        panic!("the previous klipper host device never went away");
    }

    /// The identify handshake against a real klipper inside the library.
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
        // Klipper's state — including the sequence counter that says where the
        // firmware is — lives in the library's statics, and `HostDevice::load`
        // dlopens it while `Drop` dlcloses it: a second device would map the code
        // again and start from the initializers, which is *not* what an MCU does
        // when a host disconnects. Holding one more handle keeps the mapping alive
        // across both devices, which is what makes the second connection below meet
        // a firmware that was never reset.
        let _pinned = unsafe { Library::new(library_path()) }.expect("the library maps");

        let config =
            McuConfig::new(&ConfigWrapper::untracked(&mcu_section())).expect("the [mcu] section");
        assert_eq!(config.name, "host_test");
        let interface = config.open().expect("the host library opens");

        // The handshake takes milliseconds; this only bounds a hang in a test that
        // would otherwise sit in the transport's own 10 second chunk timeout.
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

        // Dropping the MCU shuts the device down, which is what lets a process
        // build another one later; the interface clone shows the effect.
        drop(mcu);
        assert_eq!(interface.receive().await, None);
        drop(interface);

        // A second connection in the same process finds the firmware exactly where
        // the first one left it: klipper's sequence counter is one static that only
        // an accepted block moves (`src/command.c:16,301-305`), and shutting the
        // device down does not clear it. That is the situation any MCU is in when
        // it was never power-cycled — the one `restart_method: command` has to work
        // in — and the transport takes that session over instead of stalling on a
        // sequence that does not start over.
        let interface = reopen(&config).await;
        let mcu = tokio::time::timeout(
            Duration::from_secs(20),
            Mcu::connect(&config.name, interface.clone()),
        )
        .await
        .expect("the second identify handshake timed out")
        .expect("the second identify handshake failed");
        assert!(
            mcu.took_over_session(),
            "the firmware was still in the session the first connection left"
        );

        // And the adopted number is the one the firmware answers on: the same
        // command works over it.
        let params = mcu
            .call("get_clock", &[], "clock", Duration::from_secs(1))
            .await
            .expect("get_clock after the takeover");
        assert!(
            matches!(params[0], ArgValue::UInt32(_)),
            "clock should be a `%u` value, got {:?}",
            params[0]
        );

        drop(mcu);
        assert_eq!(interface.receive().await, None);
    }
}
