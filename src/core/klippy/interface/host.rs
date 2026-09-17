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
//! together, so [`HostDevice::receive`] buffers and frames them itself (the
//! `Framer` below); `send` goes the other way and encodes a frame to bytes.
//! Neither side needs to know about the other's boundaries.
//!
//! **Its main loop blocks.** `klipper_host_run` only returns once
//! `klipper_host_shutdown` is called, so it gets a thread of its own. Output is
//! pull-only, so a second thread polls `klipper_host_output` and forwards what it
//! reads to [`HostDevice::receive`] over a channel — when that thread stops, the
//! channel disconnects and a blocked `receive` returns `None`, which is exactly
//! the "no further frames will ever arrive" contract of [`Device::receive`].
//!
//! ```text
//!   send(Frame) ─► klipper_host_input ──────┐
//!                                           ▼
//!                                    klipper runtime thread
//!                                     (klipper_host_run)
//!                                           │
//!   receive() ◄── framing ◄── channel ◄── poller thread
//!                                  (klipper_host_output)
//! ```
//!
//! **Its state is process-global.** Klipper's globals live inside the shared
//! library, so one device per process is the supported configuration, and the
//! threads it spawns must be joined before the library is unloaded — hence the
//! explicit teardown in [`Drop`].

use super::Device;
use crate::core::klippy::frame::{Frame, MESSAGE_MAX, MESSAGE_MIN, MESSAGE_POS_LEN, MESSAGE_SYNC};
use crate::core::klippy::traits::InterfaceError;
use crossbeam_channel::{unbounded, Receiver};
use libloading::Library;
use std::fmt;
use std::os::raw::{c_int, c_long, c_uchar};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;
use tracing::{debug, info, warn};

/// Bytes asked of `klipper_host_output` per poll. Klipper emits at most
/// [`MESSAGE_MAX`] bytes per frame, so this drains several frames per call.
const OUTPUT_BATCH: usize = 512;

/// Idle time between polls when Klipper has nothing to send. Small enough to
/// stay well inside a command's round trip, large enough not to spin a core.
const POLL_INTERVAL: Duration = Duration::from_micros(200);

// ===========================================================================
// The library's C API (third_party/klipper/src/host/klipper_host.h)
// ===========================================================================

type InitFn = unsafe extern "C" fn() -> c_int;
type InputFn = unsafe extern "C" fn(*const c_uchar, usize) -> c_long;
type OutputFn = unsafe extern "C" fn(*mut c_uchar, usize) -> usize;
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
            output: unsafe { get(library, b"klipper_host_output\0")? },
            run: unsafe { get(library, b"klipper_host_run\0")? },
            shutdown: unsafe { get(library, b"klipper_host_shutdown\0")? },
        })
    }
}

// ===========================================================================
// Framing
// ===========================================================================

/// Reassembles protocol frames out of a byte stream.
///
/// The library hands over raw bytes, so a read can stop mid-frame, carry several
/// frames, or contain bytes corrupted in transit. This is the same recovery
/// Klipper's own `msgblock_check` (`klippy/chelper/msgblock.c`) implements, and
/// for the same reason: a frame ends with a SYNC byte, so the cheapest safe
/// resynchronisation point is the *next* SYNC rather than an educated guess at
/// how much of the buffer was junk.
///
/// Two consequences worth knowing, both inherited from Klipper:
///
/// * a bad frame costs every byte up to and including the following SYNC, so a
///   frame that arrives while [`Framer::need_sync`] is set can be skipped;
/// * if a read contains no SYNC at all, the whole read is dropped, and the
///   framer stays desynchronised until a SYNC shows up.
#[derive(Debug, Default)]
struct Framer {
    buffer: Vec<u8>,
    /// Set when an error was seen and no SYNC followed it: until the next SYNC,
    /// every byte in the stream is garbage.
    need_sync: bool,
}

impl Framer {
    /// Add bytes as they arrive.
    fn push(&mut self, bytes: &[u8]) {
        self.buffer.extend_from_slice(bytes);
    }

    /// Take the next complete frame, if one has arrived.
    ///
    /// Returns `None` both when more bytes are needed and when the buffer is
    /// unusable, so callers simply keep feeding this until a frame appears.
    fn next(&mut self) -> Option<Frame> {
        loop {
            if self.need_sync {
                self.skip_to_sync();
            }
            if self.buffer.len() < MESSAGE_MIN {
                return None;
            }

            let length = self.buffer[MESSAGE_POS_LEN] as usize;
            if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&length) {
                self.skip_to_sync();
                continue;
            }
            if self.buffer.len() < length {
                return None; // the rest of the frame is still in flight
            }

            match Frame::parse(&self.buffer[..length]) {
                Some(frame) => {
                    self.buffer.drain(..length);
                    return Some(frame);
                }
                // SYNC byte or CRC did not check out.
                None => self.skip_to_sync(),
            }
        }
    }

    /// Discard everything up to and including the next SYNC byte.
    ///
    /// When there is none, the buffer is entirely garbage: drop it and stay
    /// desynchronised, so the next read is scanned for a SYNC instead of being
    /// appended to bytes that can never frame anything.
    fn skip_to_sync(&mut self) {
        match self.buffer.iter().position(|&byte| byte == MESSAGE_SYNC) {
            Some(position) => {
                self.buffer.drain(..=position);
                self.need_sync = false;
            }
            None => {
                self.buffer.clear();
                self.need_sync = true;
            }
        }
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
/// [`Device`]: super::Device
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
    framer: Mutex<Framer>,
    /// Set by `shutdown`, checked by both worker threads and by `receive`.
    stopped: Arc<AtomicBool>,
    threads: Mutex<Vec<JoinHandle<()>>>,
}

impl HostDevice {
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

        // Output has to be polled. This thread owns the sender, so when it stops
        // the channel disconnects and every blocked `receive` wakes up with
        // `None` — no separate "unblock" signal needed.
        let poller_stopped = Arc::clone(&stopped);
        let poller_output = symbols.output;
        let poller = thread::Builder::new()
            .name("klipper-output".to_string())
            .spawn(move || {
                let mut buf = vec![0u8; OUTPUT_BATCH];
                while !poller_stopped.load(Ordering::Relaxed) {
                    let read = unsafe { poller_output(buf.as_mut_ptr(), buf.len()) };
                    if read == 0 {
                        thread::sleep(POLL_INTERVAL);
                        continue;
                    }
                    if output_tx.send(buf[..read].to_vec()).is_err() {
                        break; // nobody is receiving any more
                    }
                }
                debug!("klipper output poller stopped");
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
            framer: Mutex::new(Framer::default()),
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
            if let Some(frame) = self.framer.lock().unwrap().next() {
                return Some(frame);
            }
            match self.output.recv() {
                Ok(bytes) => {
                    debug!("received {} bytes from klipper", bytes.len());
                    self.framer.lock().unwrap().push(&bytes);
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
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::msg::proto::Payload;

    // -----------------------------------------------------------------------
    // Framing — no library needed
    // -----------------------------------------------------------------------

    /// Feed bytes to a framer the way the poller would: in chunks, pulling out
    /// whatever completes.
    fn frames_from(chunks: &[&[u8]]) -> Vec<Frame> {
        let mut framer = Framer::default();
        let mut frames = Vec::new();
        for chunk in chunks {
            framer.push(chunk);
            while let Some(frame) = framer.next() {
                frames.push(frame);
            }
        }
        frames
    }

    #[test]
    fn test_framer_waits_for_a_whole_frame() {
        let encoded = Frame::encode(3, b"hello");

        // Nothing until the last byte arrives.
        for split in 1..encoded.len() {
            let frames = frames_from(&[&encoded[..split]]);
            assert!(frames.is_empty(), "framed at {split} of {}", encoded.len());
        }
        assert_eq!(
            frames_from(&[&encoded]),
            vec![Frame::new(3, b"hello".to_vec())]
        );
    }

    #[test]
    fn test_framer_splits_a_concatenated_stream() {
        let first = Frame::encode(1, b"abc");
        let second = Frame::encode(2, b"def");
        let mut stream = first.clone();
        stream.extend_from_slice(&second);

        let expected = vec![
            Frame::new(1, b"abc".to_vec()),
            Frame::new(2, b"def".to_vec()),
        ];
        assert_eq!(frames_from(&[&stream]), expected);

        // Same result when the read boundary falls inside the first frame.
        let split = first.len() - 1;
        assert_eq!(frames_from(&[&stream[..split], &stream[split..]]), expected);
    }

    #[test]
    fn test_framer_resynchronizes_on_the_next_sync() {
        let good = Frame::encode(4, b"payload");
        let mut stream = vec![0xff, 0x00, MESSAGE_SYNC]; // not a frame
        stream.extend_from_slice(&good);

        assert_eq!(
            frames_from(&[&stream]),
            vec![Frame::new(4, b"payload".to_vec())]
        );
    }

    #[test]
    fn test_framer_drops_a_corrupt_frame_and_keeps_the_next() {
        let mut corrupt = Frame::encode(5, b"payload");
        corrupt[MESSAGE_MIN] ^= 0xff; // flip a payload byte: CRC no longer matches
        let good = Frame::encode(6, b"next");

        let mut stream = corrupt;
        stream.extend_from_slice(&good);

        // The corrupt frame is skipped up to its own trailing SYNC, so the frame
        // behind it survives — worth pinning, because dropping one byte at a time
        // instead would resynchronise on a length byte that may never come.
        assert_eq!(
            frames_from(&[&stream]),
            vec![Frame::new(6, b"next".to_vec())]
        );
    }

    #[test]
    fn test_framer_survives_a_read_without_sync() {
        let good = Frame::encode(7, b"later");
        let mut framer = Framer::default();

        // Too short to judge: nothing is framed and nothing is discarded.
        framer.push(&[0x01, 0x02, 0x03]);
        assert_eq!(framer.next(), None);
        assert!(!framer.need_sync);

        // A read long enough to hold a frame but with an impossible length byte,
        // and no SYNC to resynchronise on, is dropped whole: the framer stays
        // desynchronised instead of trying to frame the junk.
        framer.push(&[0xff, 0x00, 0x01, 0x02, 0x03]);
        assert_eq!(framer.next(), None);
        assert!(framer.need_sync);
        assert!(framer.buffer.is_empty());

        // The next SYNC ends the resynchronisation: bytes before it are dropped,
        // and a frame read after it is framed normally.
        framer.push(&good);
        assert_eq!(framer.next(), None, "expected to skip the first frame");

        framer.push(&good);
        assert_eq!(framer.next(), Some(Frame::new(7, b"later".to_vec())));
        assert!(!framer.need_sync);
    }

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

    /// A full exchange with a real klipper inside the library.
    ///
    /// Driven through [`Interface`] rather than [`HostDevice`] so the test also
    /// covers the dispatch `Interface` adds on top — `HostDevice`'s own building
    /// blocks are covered by the framer tests above.
    #[tokio::test]
    async fn test_round_trip_against_the_real_library() {
        const GET_CLOCK: i16 = 5;
        const CLOCK: i16 = 18;

        let interface = Interface::host(library_path()).unwrap();

        // `Interface::receive` blocks until a frame arrives, so every read gets a
        // deadline: a test that hangs is worse than one that fails.
        let read = |what: &'static str, message_id: u8| {
            let interface = interface.clone();
            async move {
                tokio::time::timeout(Duration::from_secs(5), async move {
                    while let Some(frame) = interface.receive().await {
                        if frame.payload().first() == Some(&message_id) {
                            return frame;
                        }
                    }
                    panic!("klipper stream ended before the {what} frame");
                })
                .await
                .unwrap_or_else(|_| panic!("no {what} frame within 5s"))
            }
        };

        // Klipper announces itself as soon as its runtime thread is up, which is
        // what proves the runtime thread, the poller, and the framing all work.
        let starting = read("starting", 13).await;
        assert_eq!(starting.payload()[0], 13);

        // `get_clock` takes no arguments, so the payload is just its message id —
        // the id the library was built with (5), as in its data dictionary.
        let mut request = Payload::new();
        request.push_i16(GET_CLOCK).unwrap();
        interface
            .send(Frame::new(0, request.into_raw()))
            .await
            .unwrap();

        // ... and the answer is id 18 followed by a u32 clock.
        let answer = read("clock", 18).await;
        let payload = Payload::from_raw(answer.payload().to_vec());
        let mut parser = payload.as_parser();
        assert_eq!(parser.pop_i16().unwrap(), CLOCK);
        let clock = parser.pop_u32().unwrap();
        assert!(parser.is_empty(), "trailing bytes after clock");
        debug!("klipper clock reads {clock}");

        // Shutting down stops the poller, which is what makes a blocked
        // `receive` return `None` instead of hanging forever.
        interface.shutdown();
        assert_eq!(interface.receive().await, None);
        assert_eq!(interface.receive().await, None, "shutdown is idempotent");
    }
}
