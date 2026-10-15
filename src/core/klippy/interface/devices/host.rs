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
//! pull-only, so a second thread waits in it and hands what it reads to the
//! references — when that thread stops, the channels it feeds disconnect and a
//! blocked `receive` returns `None`, which is exactly [`Device::receive`]'s "no
//! further frames" contract.
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
//! **One library per process.** One process may hold only one library, on one
//! path: klipper stops its main loop through a process-wide `SIGUSR1` handler
//! that every library installs over the last one, so a second path's library is
//! refused ([`refuse_a_second_library`]) rather than left to crash or hang the
//! process on the way out. The rule is per **live** path, not per config: many
//! references on the one path are fine, and a path may be loaded again once its
//! last reference is gone. The config layer refuses a config that names two
//! paths up front (`mcu/host_libraries.rs`).
//!
//! **One reader, one slot per addressed reference.** The library's output is a
//! single byte stream, so the library has **one** reader thread — and that
//! thread is the router: on a muxed path it cuts whole units out of the stream
//! and delivers each to the slot of the id its header names, exactly as the
//! shared serial port's reader does. A reference on such a path reads only its
//! own slot, so a live reference to the bridge's own device and a live
//! reference to a device behind one of its ports no longer take turns with each
//! other's bytes: each gets whole units, in order, and none of the other's.
//!
//! A **bare** path has no id to route by — the reference *is* the device — so
//! its references still share the one output channel, and against a library
//! whose firmware has no mux only one should do live I/O at a time. What holds
//! either way is the communication reuse itself: the library is loaded once,
//! several references can exist against it, and the identify handshake and
//! `get_clock` round trip run unchanged against a library that knows nothing of
//! sharing — which is what "old-firmware compatibility" means here.
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
//!
//! **A muxed path is addressed by id.** When the config says a `host_library:`
//! path carries the mux (`mcu/host_libraries.rs`), the reference is opened in
//! `OutputMode::Mux` and attached ([`HostDevice::load_mux`]): the library is
//! given the identity the host assigned the path before klipper comes up, the
//! attach names the device in a `SET_KLIPPER_NODEID` unit and re-confirms the
//! assignment, and from then on every frame sent carries the mux header naming
//! the reference's id, while `receive` keeps only the units addressed to it.
//! A path the config does **not** call muxed keeps the bare-frame wire format
//! exactly as before; whether a path is muxed is a config fact, never probed.

use crate::core::klippy::assign;
use crate::core::klippy::frame::Frame;
#[cfg(debug_assertions)]
use crate::core::klippy::frame::FrameStream;
use crate::core::klippy::interface::error::InterfaceError;
use crate::core::klippy::interface::{describe_frame, Device};
use crossbeam_channel::{unbounded, Receiver, RecvTimeoutError, Sender};
use libloading::Library;
use std::collections::HashMap;
use std::ffi::CString;
use std::fmt;
use std::os::raw::{c_char, c_int, c_long, c_uchar};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock, Weak};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};
use tracing::{debug, info, trace, warn};

use super::serial::{
    admin_unit, assignment_request, encode_header, parse_assignment_ack, set_node_id_for,
    set_port_node_id, UnitStream, ADMIN_LISTEN, ASSIGN_ATTACH_ATTEMPTS, ASSIGN_ATTACH_TIMEOUT,
    CMD_REQUEST_BOOTLOADER, CMD_RESET_PORT, CMD_RESTART_DEVICE, HEADER_BYTES,
};
use super::shared::resource_for;

/// The process-wide registry that makes a host library a per-path singleton:
/// references on the same path share one loaded [`HostLibrary`] (and thus one
/// runtime and one reader).
fn library_registry() -> &'static Mutex<HashMap<PathBuf, Weak<HostLibrary>>> {
    static LIBRARIES: OnceLock<Mutex<HashMap<PathBuf, Weak<HostLibrary>>>> = OnceLock::new();
    LIBRARIES.get_or_init(|| Mutex::new(HashMap::new()))
}

/// Serialises the tests that load — or try to load — the real host library.
///
/// The library registry is a per-path process-wide singleton, so two tests
/// sharing the one `.so` must not overlap: the second would meet — or, at the
/// end, unload — the first's library mid-handshake. A load now also consults
/// the registry for a **different** live path (one host library per process,
/// [`refuse_a_second_library`]), so an attempt on a path that does not exist
/// takes the lock too. A loaded library's output mode and identity are fixed
/// too ([`library_for`]), which puts a scan test and an identify test on the
/// same `.so` in exactly that pair.
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
    /// The byte stream in every build **and** the mux header on every frame: the
    /// mode of a reference on a `host_library:` path the config says is muxed
    /// ([`HostMux`]). It reads bytes because a unit is not a frame the frame API
    /// could hand over, exactly as `Bytes` does, but its `send`/`receive` address
    /// the frames with the header (see the module docs).
    Mux,
}

/// What a muxed `host_library:` path attaches with.
///
/// The path is addressed by id (`mcu/host_libraries.rs` decides that from the
/// config): the attach gives the library's own device `identity` before klipper
/// comes up, names `named` in a `SET_KLIPPER_NODEID` unit so the bridge hands the
/// device `mcu_id`, re-confirms the assignment, and from then on frames carry
/// the mux header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HostMux {
    /// The identity the library's own device answers the mux with, delivered by
    /// `klipper_host_set_uuid` before `klipper_host_init`.
    pub identity: [u8; 6],
    /// The uuid the attach names in `SET_KLIPPER_NODEID`: the section's own
    /// `uuid` when it wrote one, else `identity` (the bridge's own device).
    pub named: [u8; 6],
}

/// The live library for `path` in `mode` with `identity` and the bridge port
/// table `host_port_paths`, loading it (and starting its threads) if the
/// registry holds no live one.
///
/// # Errors
/// Returns [`InterfaceError`] when opening fails, and when a live library on
/// `path` disagrees with `mode`, `identity` or `host_port_paths`: all three are
/// fixed when the library is opened (the mode picks its reader, the identity is
/// set, and the ports are named, all before klipper comes up), so reusing it
/// under other ones would silently answer as the wrong thing — another port
/// table would leave the ports on the path the library was built with.
fn library_for(
    path: &Path,
    mode: OutputMode,
    identity: Option<[u8; 6]>,
    host_port_paths: &[(u8, String)],
) -> Result<Arc<HostLibrary>, InterfaceError> {
    refuse_a_second_library(path)?;
    let library = resource_for(library_registry(), path, |path| {
        HostLibrary::open(path, mode, identity, host_port_paths)
    })?;
    let identity = identity.unwrap_or([0; 6]);
    if library.router.mode != mode || library.identity != identity {
        return Err(InterfaceError::Other(format!(
            "klipper host library {} is already loaded with a different output mode or \
             identity, which are fixed when the library is opened",
            library.router.library_path.display()
        )));
    }
    // A port's tty is fixed when the library is opened too, so a reference that
    // names its own ports can only be served by the load that opens it. A
    // reference that names none — a device behind a bridge port — is served by
    // any live library, which is why only a non-empty table is checked.
    if !host_port_paths.is_empty() && library.port_paths.as_slice() != host_port_paths {
        return Err(InterfaceError::Other(format!(
            "klipper host library {} is already loaded with a different set of bridge port \
             ttys (loaded: {}; asked: {}), so the asked ttys cannot be named: a port's tty is \
             fixed when the library is opened, because klipper_host_port() must be called \
             before klipper_host_init()",
            library.router.library_path.display(),
            describe_port_paths(&library.port_paths),
            describe_port_paths(host_port_paths),
        )));
    }
    Ok(library)
}

/// A bridge port table as one line for an error: `0=/dev/ttyUSB0,
/// 1=/dev/ttyUSB1`, or `none` for the empty table (a library opened with no
/// ports named).
fn describe_port_paths(paths: &[(u8, String)]) -> String {
    if paths.is_empty() {
        return "none".to_string();
    }
    paths
        .iter()
        .map(|(slot, tty)| format!("{slot}={tty}"))
        .collect::<Vec<_>>()
        .join(", ")
}

/// Refuse to load `path` while a **different** host library is live in this
/// process.
///
/// A klipper host library stops its main loop through a **process-wide**
/// `SIGUSR1` handler that every library installs over the last one
/// (`third_party/klipper/src/host/host.c`), so a second library in one process
/// makes the process crash or hang on the way out, whichever of the two is
/// unloaded first (see [`one_library_per_process`]). The registry's live entries
/// ([`Weak::upgrade`]) say which other path — if any — is loaded right now, so
/// the rule is **by path**: any number of references on the one live path (a
/// mux, a device behind a port) load as before, and a path whose library has
/// been unloaded loads again — "one at a time", not "one ever".
///
/// # Errors
/// [`InterfaceError::Other`] naming both paths when another path is live.
fn refuse_a_second_library(path: &Path) -> Result<(), InterfaceError> {
    let registry = library_registry().lock().unwrap();
    let live_other = registry
        .iter()
        .find(|(loaded, weak)| loaded.as_path() != path && weak.upgrade().is_some());
    match live_other {
        None => Ok(()),
        Some((loaded, _)) => Err(InterfaceError::Other(one_library_per_process(path, loaded))),
    }
}

/// The message a load of `requested` gets when `loaded` is already live: one
/// host library per process, and why.
fn one_library_per_process(requested: &Path, loaded: &Path) -> String {
    format!(
        "klipper host library {} cannot be loaded: {} is already loaded in this process, and \
         only one host library per process is allowed. klipper stops its main loop through a \
         process-wide SIGUSR1 handler that each library installs over the last one, so two \
         libraries in one process meet either a jump into unloaded code (a crash) or a join \
         whose signal another library consumed (a hang); use several references on the one \
         library (a mux), or run the firmware images in separate processes",
        requested.display(),
        loaded.display()
    )
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
/// `klipper_host_port(int slot, const char *path)` — the tty bridge port `slot`
/// opens, which the host names before klipper comes up.
type SetPortFn = unsafe extern "C" fn(c_int, *const c_char) -> c_int;
/// How many bridge port ttys the library takes, `slot` being
/// `0..HOST_PORT_SLOTS` — `KLIPPER_HOST_PORT_SLOTS` in
/// `third_party/klipper/src/host/klipper_host.h`, the count the bridge's
/// `BRIDGE_MAX_PORTS` must not exceed (`third_party/klipper/src/bridge.c`).
/// Named in the error a refused `klipper_host_port` gets, so the slot it was
/// given need not be guessed at.
const HOST_PORT_SLOTS: i32 = 4;
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
    /// `klipper_host_port`, absent from a library built before the host could
    /// name a port's tty — naming one is then refused rather than half-applied
    /// (see [`HostLibrary::open`]).
    set_port: Option<SetPortFn>,
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
            // Optional, like the identity: only a library that can open a
            // bridge port's tty at the host's word exports this. Naming one to
            // a library that does not is an error, not a silent fallback to the
            // path the library was built with.
            set_port: unsafe { library.get::<SetPortFn>(b"klipper_host_port\0") }
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
/// loaded mapping: the one runtime thread, the one output reader, and the
/// per-id slots that reader routes a muxed path's units to. Communication
/// references ([`HostDevice`]) hold an `Arc` to this; the library is loaded
/// when the first reference needs it and unloaded (threads joined, code
/// unmapped) when the last one is dropped, which is also what makes it safe to
/// call `klipper_host_init` again for a later library: unmapping and remapping
/// starts the firmware from its initializers, exactly as a power-cycled board
/// would.
struct HostLibrary {
    /// Keeps the mapped code the symbols point into alive. Never read: the
    /// [`Drop`] impl joins the threads before any field is dropped, and
    /// `Drop::drop` runs ahead of field drops, so unmapping here is safe.
    _library: Library,
    symbols: Symbols,
    /// The path the library was loaded from, and the routing state its reader
    /// shares with its references. Both live in the [`Router`] because the
    /// reader holds an `Arc` to **that** — never to the library, which would
    /// let it postpone the unload (see [`Router`]).
    router: Arc<Router>,
    /// The read the reader thread makes — `klipper_host_output_wait` (the byte
    /// stream) or, in a release build's default mode,
    /// `klipper_host_output_frame` (whole frames). Resolved here because the
    /// frame symbol exists only in the [`Library`] this struct owns.
    read_output: OutputFn,
    /// The identity this library was given before klipper came up, all zeros
    /// when none was ([`HostLibrary::open`]). Printed by the firmware and
    /// answered to the mux, so a request for another one is refused.
    identity: [u8; 6],
    /// The bridge port ttys this library was opened with, as `(slot, tty)` in
    /// the order they were named, empty when it was opened with none
    /// ([`HostLibrary::open`]).
    ///
    /// Ports are fixed at `klipper_host_init`, so a later reference that names
    /// its own table cannot be served ([`library_for`]); keeping the table is
    /// what tells such a reference apart from one that names none — a device
    /// behind a bridge port — which the same library serves.
    port_paths: Vec<(u8, String)>,
    /// The one output channel a **bare** path's references share: bytes (or
    /// whole frames) from Klipper, in arrival order, the reader the only
    /// producer and every bare reference holding a clone of the receiver. A
    /// muxed path sends nothing here — its units go to the router's slots — so
    /// no reference reads it and the two ends of the channel just sit idle.
    output: Receiver<Vec<u8>>,
    /// Set by the final teardown, in [`Drop`], and polled by the reader thread
    /// it must join.
    stopped: Arc<AtomicBool>,
    threads: Mutex<Vec<JoinHandle<()>>>,
    /// Serializes one unit's bytes across **every reference** on this library.
    ///
    /// The console input is a byte stream: in a debug build
    /// [`HostDevice::write_raw`] feeds it one byte per `klipper_host_input`
    /// call, so two references writing at once interleave their bytes and the
    /// bridge resyncs on a header it never sent — silently dropping a unit that
    /// has no receipt (a `SET_PORT_NODEID`, a reset). One lock for the library,
    /// not one per reference, is what makes that hold when the writers are two
    /// references on the same library (the bridge and a downstream device). It
    /// is taken only around one unit's bytes and never across a wait, and the
    /// read path never takes it, so the two cannot deadlock.
    write_lock: Mutex<()>,
}

/// The routing state a [`HostLibrary`] and its reader share.
///
/// The library's one reader is the router of a muxed path: it cuts units out of
/// the byte stream and delivers each to the slot of the id its header names.
/// It does that through this structure — the slots, the mode that says whether
/// there is anything to route, and the path it names in its logs — and holds an
/// `Arc` to **it** rather than to the library. A reader holding the library, as
/// an `Arc` or as a `Weak` it upgrades, would keep the last reference count up:
/// the unload is that last reference's [`Drop`], so the reader could postpone it
/// — or, having held the last count itself, run it, and the join of its own
/// thread, on the reader thread.
struct Router {
    /// The path the library was loaded from; named in the reader's logs.
    library_path: PathBuf,
    /// Which output API the reader reads, and whether it routes at all (fixed
    /// when the library is opened; [`library_for`] refuses a reference that asks
    /// for another one).
    mode: OutputMode,
    /// One slot per muxed reference on a muxed path, keyed by the MCU id the
    /// unit headers name — the shape
    /// [`SharedSerialPort`](super::serial::SharedSerialPort)'s own slots have.
    /// The reader is the only producer ([`push_unit`]); a reference reads only
    /// its own slot, so one reference's units can never be taken by another.
    slots: Mutex<HashMap<u32, Sender<Vec<u8>>>>,
}

impl Router {
    /// Give `mcu_id` a slot and hand back the receiver its reference reads.
    ///
    /// Every unit the reader decodes to `mcu_id` is delivered here
    /// ([`push_unit`]) and nothing else is, which is what keeps two live
    /// references on one library from splitting the stream. An id that is
    /// already registered is replaced, so a later reference on the same id
    /// takes its units over.
    fn open_slot(&self, mcu_id: u32) -> Receiver<Vec<u8>> {
        let (sender, receiver) = unbounded::<Vec<u8>>();
        self.slots.lock().unwrap().insert(mcu_id, sender);
        receiver
    }
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
        host_port_paths: &[(u8, String)],
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

        // Where each of the bridge's ports is, likewise fixed once the device is
        // up (a port's tty opens when the host's `config_uart` arrives, after
        // nothing at all at init), so the pairs go in here, before `init`, with
        // the identity. A port the config named nothing for is simply absent
        // and opens the path the library was built with.
        if !host_port_paths.is_empty() {
            let set_port = match symbols.set_port {
                Some(set_port) => set_port,
                None => {
                    return Err(InterfaceError::Other(format!(
                        "klipper host library {} does not support run-time port paths: it was \
                         built without klipper_host_port",
                        library_path.display()
                    )))
                }
            };
            for (slot, tty) in host_port_paths {
                let tty = CString::new(tty.as_str()).map_err(|e| {
                    InterfaceError::Other(format!(
                        "klipper host library {}: the tty for bridge port {slot} is not a \
                         C string: {e}",
                        library_path.display()
                    ))
                })?;
                let result = unsafe { set_port(c_int::from(*slot), tty.as_ptr()) };
                if result != 0 {
                    return Err(InterfaceError::Other(format!(
                        "klipper_host_port({slot}, {}) returned {result}: the slot is outside \
                         the range this build takes (0..={}), or the path does not fit",
                        tty.to_string_lossy(),
                        HOST_PORT_SLOTS - 1
                    )));
                }
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

        // Klipper's main loop blocks until shutdown, and drives everything else,
        // so it has to be its own thread.
        let run = symbols.run;
        let runtime = thread::Builder::new()
            .name("klipper-runtime".to_string())
            .spawn(move || {
                info!("klipper runtime started");
                unsafe { run() };
                info!("klipper runtime returned");
            })
            .map_err(|e| InterfaceError::Other(format!("failed to spawn runtime: {e}")))?;

        // The reader's read is the mode's: the byte stream in a debug build and
        // in `OutputMode::Bytes`/`Mux`, whole frames in a release build's
        // default mode. The frame API only exists in a library built with
        // CONFIG_HOST_FRAME_API, and **only the default mode asks for it**: a
        // mux unit is not a frame boundary, so a bytes- or mux-mode load reads
        // the byte-stream API that every build has.
        #[cfg(debug_assertions)]
        let read_output = symbols.output_wait;
        #[cfg(not(debug_assertions))]
        let read_output = match mode {
            OutputMode::Bytes | OutputMode::Mux => symbols.output_wait,
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

        let shared = Arc::new(Self {
            _library: library,
            symbols,
            router: Arc::new(Router {
                library_path,
                mode,
                slots: Mutex::new(HashMap::new()),
            }),
            read_output,
            identity,
            port_paths: host_port_paths.to_vec(),
            output,
            stopped,
            threads: Mutex::new(vec![runtime]),
            write_lock: Mutex::new(()),
        });

        // Output is pull-only, so it gets a thread of its own. That thread owns
        // the bare channel's sender, so when it stops the channel disconnects
        // and every blocked `receive` wakes up with `None` — no separate
        // "unblock" signal needed. It holds the [`Router`] and never the
        // library, so it can neither keep the library alive nor run its unload.
        //
        // A failure to spawn it leaves `shared` to be dropped here, and
        // [`HostLibrary`]'s `Drop` stops the runtime thread and waits for it
        // before the library is unmapped.
        shared.start_reader(output_tx)?;

        info!(
            "klipper host library loaded from {}",
            shared.router.library_path.display()
        );
        Ok(shared)
    }

    /// Spawn the library's one output reader.
    ///
    /// One reader, not one per reference: the library's output is a single
    /// byte stream, and two readers taking turns with it would each hold half a
    /// unit ([`reader_loop`]). The thread holds the [`Router`] — the slots it
    /// routes into, never the library — and the bare channel's sender, which is
    /// what makes a stopped reader disconnect every bare reference's
    /// `receive`.
    fn start_reader(self: &Arc<Self>, bare: Sender<Vec<u8>>) -> Result<(), InterfaceError> {
        let router = Arc::clone(&self.router);
        let stopped = Arc::clone(&self.stopped);
        let read = self.read_output;
        let handle = thread::Builder::new()
            .name("klipper-output".to_string())
            .spawn(move || reader_loop(router, stopped, read, bare))
            .map_err(|e| InterfaceError::Other(format!("failed to spawn poller: {e}")))?;
        self.threads.lock().unwrap().push(handle);
        Ok(())
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

/// Read the library and hand what arrives to the references holding it, until
/// the library is unloaded.
///
/// The library's **only** producer, and on a muxed path its **router**: the
/// bytes are cut into whole units here ([`UnitStream`], which reassembles a
/// unit across reads — a debug build's read is a single byte) and each unit
/// goes to the slot of the id its header names ([`route_unit`]). Two references
/// on one library therefore never take turns with the stream, which is what a
/// reader per reference used to do.
///
/// A **bare** path has no id to route by, so nothing is decoded: the read goes
/// to the one channel its references share, exactly as the library handed it
/// over — raw bytes for a bench scan's `Bytes`, a frame or a byte for
/// `Default`.
fn reader_loop(
    router: Arc<Router>,
    stopped: Arc<AtomicBool>,
    read: OutputFn,
    bare: Sender<Vec<u8>>,
) {
    let mut buf = vec![0u8; OUTPUT_BATCH];
    let mut units = UnitStream::new();
    while !stopped.load(Ordering::Relaxed) {
        // Blocks in the library until data arrives or the wait expires, so no
        // polling interval is needed here. How much arrives is the mode's
        // business: a byte in a debug build, a whole frame or a mux unit's
        // worth of bytes in a release build.
        let taken = unsafe { read(buf.as_mut_ptr(), buf.len(), OUTPUT_WAIT_MS) };
        if taken == 0 {
            continue; // the wait expired with nothing to read
        }
        // Nothing here holds the library — see [`Router`] — so the last
        // reference's drop is what stops this read and unloads the library,
        // and it can never be this thread that runs that unload.
        match router.mode {
            OutputMode::Mux => {
                units.feed(&buf[..taken]);
                while let Some((arb, unit)) = units.take_unit_with_arb() {
                    route_unit(&router, arb, unit);
                }
            }
            OutputMode::Bytes | OutputMode::Default => {
                if bare.send(buf[..taken].to_vec()).is_err() {
                    break; // no reference is reading the bare channel any more
                }
            }
        }
    }
    debug!("klipper output reader stopped");
}

/// Route one whole unit the reader cut out of a muxed path's byte stream.
///
/// The policy is the shared serial port's router's: a unit whose header decodes
/// to an mcu id goes to that id's slot ([`push_unit`]), and a unit that names no
/// id is dropped. The admin ids
/// ([`ADMIN_LISTEN`]/[`ADMIN_REPLY`]) are not assigned ids
/// ([`assign::mcu_id_from_arb`] is `None` for them) — they belong to the bridge,
/// which consumes and answers them itself — so nothing upstream of the bridge
/// is ever their reader and both transports drop them for that reason.
///
/// The serial router also diverts the assignment answer to a field of its own,
/// because its slot reassembles *frames* and the answer's pseudo-frame would
/// look malformed there. A slot here carries whole **units**, which the
/// reference cuts with [`UnitStream`] — the same stream
/// [`HostDevice::attach`]'s handshake scans — so the answer is routed by its
/// header like everything else and reaches the reference waiting for it.
fn route_unit(router: &Router, arb: u32, unit: Vec<u8>) {
    match assign::mcu_id_from_arb(arb) {
        Some(mcu_id) => push_unit(router, mcu_id, unit),
        None => debug!(
            "klipper host library {}: {arb:#x} names no mcu id; dropping unit",
            router.library_path.display()
        ),
    }
}

/// Hand one whole unit to the reference `mcu_id` belongs to.
///
/// The send is into an unbounded channel and the slot map is locked only long
/// enough to take the sender out, so the reader never waits on a reference —
/// which is what keeps the two from deadlocking. A unit for an id no reference
/// holds (or whose reference is gone) is reported and dropped, like the shared
/// serial port's `push_frame` dropping a frame for an unregistered MCU.
fn push_unit(router: &Router, mcu_id: u32, unit: Vec<u8>) {
    let sender = router.slots.lock().unwrap().get(&mcu_id).cloned();
    let delivered = match sender {
        Some(sender) => sender.send(unit).is_ok(),
        None => false,
    };
    if !delivered {
        debug!(
            "klipper host library {}: no live reference for mcu {mcu_id}; dropping unit",
            router.library_path.display()
        );
    }
}

/// Bytes asked of the library per read.
///
/// A debug build reads a byte at a time, which is the hardest case for
/// reassembly: no read ever contains a whole frame, so every frame goes through
/// [`FrameStream`]. A release build asks for one frame at a time, and
/// [`MESSAGE_MAX`](crate::core::klippy::frame::MESSAGE_MAX) is the largest one
/// the protocol can produce — and a whole mux unit fits, so it is the batch a
/// release build's bytes- and mux-mode reader asks for too.
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
    /// What this reference reads, in arrival order.
    ///
    /// A **muxed** reference reads the slot its own id owns, which the
    /// library's reader fills with the units addressed to it ([`push_unit`]):
    /// no other reference's unit is ever in here, so both sides of a live mux
    /// session get whole units. A **bare** reference reads a clone of the
    /// library's one output channel: all bare references share that queue and a
    /// frame goes to whichever of them reads it first (a competing queue, not a
    /// broadcast), so only one is live against a library whose firmware has no
    /// mux (see the module docs).
    input: Receiver<Vec<u8>>,
    /// Set by this reference's `shutdown`, checked by its `send`, `receive`
    /// and the poll loop. This is per-handle: the library's own `stopped` is
    /// set only by the final teardown.
    stopped: AtomicBool,
    /// Bytes pulled from `input`, reassembled into frames.
    ///
    /// A release build is handed whole frames and never needs this.
    #[cfg(debug_assertions)]
    stream: Mutex<FrameStream>,
    /// Bytes pulled from `input` by [`HostDevice::read`] but not handed out
    /// yet: one library read can deliver more than the caller's buffer holds.
    raw: Mutex<Vec<u8>>,
    /// The mux id this reference is addressed by, or `None` for the bare-frame
    /// modes. Set when a muxed path is attached ([`HostDevice::load_mux`]).
    mux: Option<MuxRoute>,
    /// Units pulled from `input`, cut out of the stream. Only a muxed reference
    /// reads it (the handshake, then [`HostDevice::receive`]); the bare-frame
    /// modes reassemble frames instead.
    units: Mutex<UnitStream>,
}

/// The mux id a reference is addressed by: the id it sends under and keeps
/// units for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct MuxRoute {
    /// The MCU-layer id ([`McuIds`](crate::core::klippy::mcu::McuIds)) this
    /// reference was given; the header on everything it sends names it and it
    /// keeps only the units that decode back to it.
    mcu_id: u32,
    /// The arbitration id its frames go out under: `0x100 + 2 * mcu_id`.
    arb: u32,
}

impl HostDevice {
    /// Returns a short identifier for logging: `"«path»<«reference»>"`.
    fn id(&self) -> String {
        format!(
            "{}<{}>",
            self.library.router.library_path.display(),
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
        Self::open(path.as_ref(), OutputMode::Default, None, None, &[])
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
        Self::open(path.as_ref(), OutputMode::Bytes, identity, None, &[])
    }

    /// Open a communication reference on the klipper host library at `path`
    /// **for a muxed path**, and attach it: give the library its `identity`,
    /// name the device `mux.named` and hand it `mcu_id` with a
    /// `SET_KLIPPER_NODEID` unit, then re-confirm the assignment before klipper
    /// is identified.
    ///
    /// From then on [`Device::send`]/[`Device::receive`] carry the mux header:
    /// every frame sent is prefixed with the header naming `mcu_id`, and only
    /// the units that decode back to `mcu_id` are handed to `receive`.
    ///
    /// Which paths are muxed, and what `mux` holds, is the config layer's call
    /// (`mcu/host_libraries.rs`); this is only the wire half. A library that
    /// does not speak the mux never answers the handshake, so the attach times
    /// out — the same failure a shared serial port misses its handshake on.
    ///
    /// `host_port_paths` is the tty each of this MCU's bridge ports opens, as
    /// `(slot, path)` — the port index and the `uart_path:` the config wrote for
    /// it (`mcu/bridge_ports.rs`). It is handed to the library **before**
    /// klipper comes up (`klipper_host_port`), so a port's stream is what the
    /// host says it is rather than what the library was built with; the paths
    /// are only read when this call is the one that loads the library, exactly
    /// as the identity is.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] for the reasons [`HostDevice::load`] does,
    /// when `mcu_id` is outside the addressable range, and
    /// [`InterfaceError::Timeout`] when the device never confirms the
    /// assignment.
    pub fn load_mux(
        path: impl AsRef<Path>,
        mux: HostMux,
        mcu_id: u32,
        host_port_paths: &[(u8, String)],
    ) -> Result<Self, InterfaceError> {
        let device = Self::open(
            path.as_ref(),
            OutputMode::Mux,
            Some(mux.identity),
            Some(mcu_id),
            host_port_paths,
        )?;
        device.attach(mux.named, mcu_id)?;
        Ok(device)
    }

    /// Open a communication reference on the klipper host library at `path`
    /// **for a device behind a bridge port**, and do **not** attach it.
    ///
    /// This is [`HostDevice::load_mux`]'s first half only: the library is given
    /// `identity` before klipper comes up, and the reference is addressed
    /// by `mcu_id` (`MuxRoute::for_id`) — but there is no attach, because there
    /// is no handshake a device behind a port could answer. The bridge's own
    /// device is what `load_mux` names over the admin channel; a device behind
    /// one of its ports has no admin channel of its own, so the library names
    /// nothing and waits for nothing, and this load cannot time out.
    ///
    /// The route is the bridge's `SET_PORT_NODEID`, and **the caller sends it**
    /// — over the bridge's own handle, with
    /// [`HostDevice::send_set_port_node_id`] — because only the caller knows
    /// which port the device sits behind. Until it does, `mcu_id` names no port
    /// and units sent under it go nowhere, silently.
    ///
    /// Only the bridge's own device is named in a `SET_KLIPPER_NODEID` unit, and
    /// that is `load_mux`'s job — so this takes the path's `identity` alone, not
    /// a name to give the device.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] for the reasons [`HostDevice::load`] does —
    /// including a library already live on `path` in another mode or with
    /// another identity — and when `mcu_id` is outside the addressable range.
    pub fn load_downstream(
        path: impl AsRef<Path>,
        identity: [u8; 6],
        mcu_id: u32,
    ) -> Result<Arc<HostDevice>, InterfaceError> {
        Ok(Arc::new(Self::open(
            path.as_ref(),
            OutputMode::Mux,
            Some(identity),
            Some(mcu_id),
            &[],
        )?))
    }

    /// Wrap the live library on `path` as a reference in `mode`.
    ///
    /// `host_port_paths` reaches the library only when this call opens it (see
    /// [`HostDevice::load_mux`]): the paths are fixed at `klipper_host_init`, so
    /// every other caller — a bare load, a downstream reference — passes none.
    fn open(
        path: &Path,
        mode: OutputMode,
        identity: Option<[u8; 6]>,
        mcu_id: Option<u32>,
        host_port_paths: &[(u8, String)],
    ) -> Result<Self, InterfaceError> {
        let library = library_for(path, mode, identity, host_port_paths)?;
        let reference = NEXT_REFERENCE.fetch_add(1, Ordering::Relaxed);
        let mux = match (mode, mcu_id) {
            (OutputMode::Mux, Some(mcu_id)) => Some(MuxRoute::for_id(mcu_id)?),
            // A muxed reference without an id would read the bare channel,
            // while the library's reader only fills per-id slots in mux mode:
            // it would hear nothing at all. No caller builds that pair (a mux
            // path always names the id it addresses), so refuse it here rather
            // than hand back a deaf reference.
            (OutputMode::Mux, None) => {
                return Err(InterfaceError::Other(
                    "a muxed host library reference needs the mcu id it addresses".to_string(),
                ))
            }
            _ => None,
        };
        // Where this reference reads. A muxed one reads the slot its own id
        // owns, which the library's reader fills with the units addressed to
        // it; a bare one reads the library's whole output, which every bare
        // reference clones (see the module docs).
        let input = match mux {
            Some(route) => library.router.open_slot(route.mcu_id),
            None => library.output.clone(),
        };
        Ok(Self {
            input,
            library,
            reference,
            stopped: AtomicBool::new(false),
            #[cfg(debug_assertions)]
            stream: Mutex::new(FrameStream::new()),
            raw: Mutex::new(Vec::new()),
            mux,
            units: Mutex::new(UnitStream::new()),
        })
    }

    /// Path of the library this reference was loaded from.
    pub fn library_path(&self) -> &Path {
        &self.library.router.library_path
    }

    /// Whether this reference is muxed: only a muxed one is addressed by an id,
    /// and only a bridge addresses its device by one — so only a muxed reference
    /// has the admin channel the bridge commands ([`HostDevice::send_admin`] and
    /// the family built on it) are written to. A bare reference
    /// ([`HostDevice::load`]) is the device itself, with no bridge to tell.
    pub(crate) fn is_mux(&self) -> bool {
        self.mux.is_some()
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
                .input
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
        self.write_raw(buf)
            .map_err(|e| std::io::Error::other(e.to_string()))?;
        Ok(buf.len())
    }

    /// Hand `bytes` to the library's console input, whole.
    ///
    /// A debug build feeds it one byte per call, so the library's receive path
    /// sees the stream the way a real link would deliver it; a release build
    /// gives it the block in one call. The caller has already framed the bytes —
    /// a bare [`Frame`], a mux unit, or a handshake unit — so this only writes.
    ///
    /// The whole call goes in under the library's write lock, so another
    /// reference on the same library cannot split this unit's header from its
    /// frame: in a debug build each byte is its own `klipper_host_input` call,
    /// and a bridge that sees a header it never sent resyncs and drops the unit.
    /// The lock covers exactly one call, so the caller frames what it passes
    /// ([`HostDevice::send_admin`], [`HostDevice::attach`]).
    fn write_raw(&self, bytes: &[u8]) -> Result<(), InterfaceError> {
        if self.stopped.load(Ordering::Relaxed) {
            return Err(InterfaceError::ConnectionLost);
        }
        // Held across the bytes of this one call and released before returning,
        // so no wait ever runs inside it: the read path (`receive`,
        // `wait_for_assignment`) takes no write lock, so the two cannot deadlock.
        let _writing = self.library.write_lock.lock().unwrap();
        #[cfg(debug_assertions)]
        let written = {
            let mut taken = 0;
            for byte in bytes {
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
        Ok(())
    }

    /// Send one admin-channel unit to the bridge: the [`ADMIN_LISTEN`] header,
    /// then `payload` — one `KXID` frame — handed to the library's input as one
    /// whole unit, under the library's write lock so another reference on the
    /// same library cannot split its header from its frame.
    ///
    /// The admin channel belongs to the bridge: it is how a device with no admin
    /// channel of its own is named behind a port. The unit carries this
    /// reference's own mux id nowhere — the header is the fixed admin id, so
    /// unlike the frame path ([`Device::send`]) this deliberately does **not**
    /// prefix the reference's own arb.
    ///
    /// # Errors
    /// [`InterfaceError::ConnectionLost`] when this reference has been shut
    /// down, [`InterfaceError::SendError`] when the library refuses the bytes.
    pub fn send_admin(&self, payload: &[u8]) -> Result<(), InterfaceError> {
        let mut unit = encode_header(ADMIN_LISTEN);
        unit.extend_from_slice(payload);
        self.write_raw(&unit)
    }

    /// Tell the bridge which id the device behind port `index` answers to:
    /// [`set_port_node_id`]'s unit, byte for byte, on the admin channel.
    ///
    /// The bridge builds its port table from it and consumes it — no receipt,
    /// nothing sent downstream, idempotent — so this is how a downstream
    /// reference's route is established (see [`HostDevice::load_downstream`]).
    /// The caller decides **when**; this only spells the command and writes it.
    ///
    /// # Errors
    /// The same as [`HostDevice::send_admin`].
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
    /// mux itself. No receipt: the caller confirms it by whether the device can
    /// be reached again (`mcu/restart.rs`).
    ///
    /// # Errors
    /// The same as [`HostDevice::send_admin`].
    pub fn send_restart_device(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_admin(&admin_unit(&[CMD_RESTART_DEVICE, index])[HEADER_BYTES..])
    }

    /// Ask the bridge to pulse the lines of port `index` — [`CMD_RESET_PORT`]'s
    /// admin unit. The bridge performs it; nothing is sent downstream.
    ///
    /// The reset a **legacy** bridge-port device gets: it has no admin channel of
    /// its own, so its bridge's port is the only thing that can reset it. No
    /// receipt, like [`HostDevice::send_restart_device`].
    ///
    /// # Errors
    /// The same as [`HostDevice::send_admin`].
    pub fn send_reset_port(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_admin(&admin_unit(&[CMD_RESET_PORT, index])[HEADER_BYTES..])
    }

    /// Ask the device behind port `index` to enter its bootloader:
    /// [`CMD_REQUEST_BOOTLOADER`]'s admin unit, the command a **flashing tool**
    /// addresses a board with. Nothing in the restart path sends it: a restart
    /// reloads a firmware that is running, while this command is for the case
    /// where it is about to be replaced.
    ///
    /// # Errors
    /// The same as [`HostDevice::send_admin`].
    pub fn send_request_bootloader(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_admin(&admin_unit(&[CMD_REQUEST_BOOTLOADER, index])[HEADER_BYTES..])
    }

    /// Attach a muxed reference: name the device `named`, hand it `mcu_id`, and
    /// re-confirm the assignment.
    ///
    /// The frame order mirrors the shared serial port's attach
    /// ([`serial::attach`](super::serial::attach)): the `SET_KLIPPER_NODEID`
    /// unit first, then the assignment request, retried
    /// [`ASSIGN_ATTACH_ATTEMPTS`] times for [`ASSIGN_ATTACH_TIMEOUT`] each.
    ///
    /// [`HostDevice::load_mux`] is this run right after its reference opens;
    /// a reference opened by [`HostDevice::load_downstream`] calls it later,
    /// once the caller has told the bridge which port the device sits behind
    /// ([`HostDevice::send_set_port_node_id`]) — **the caller owns that
    /// order**, because only it knows the port. A device whose route does not
    /// exist yet is reached by nothing, so the handshake times out like any
    /// other unanswered one.
    ///
    /// Only a reference the bridge addresses by id can complete this: the mux is
    /// what answers the assignment, so a bare reference ([`HostDevice::load`],
    /// no mux) waits [`ASSIGN_ATTACH_ATTEMPTS`] × [`ASSIGN_ATTACH_TIMEOUT`] for
    /// nothing and times out.
    ///
    /// # Errors
    /// An id the wire cannot address, a write that fails, or
    /// [`InterfaceError::Timeout`] when the answer never arrives.
    pub fn attach(&self, named: [u8; 6], mcu_id: u32) -> Result<(), InterfaceError> {
        let set = set_node_id_for(named, mcu_id).ok_or_else(|| {
            InterfaceError::Other(format!(
                "mcu id {mcu_id} is outside the range a mux header can address"
            ))
        })?;
        let request = assignment_request(mcu_id).expect("the id was just checked");
        self.write_raw(&set)?;
        for _ in 0..ASSIGN_ATTACH_ATTEMPTS {
            self.write_raw(&request)?;
            if self.wait_for_assignment(mcu_id, ASSIGN_ATTACH_TIMEOUT) {
                return Ok(());
            }
            if self.stopped.load(Ordering::Relaxed) {
                return Err(InterfaceError::ConnectionLost);
            }
        }
        Err(InterfaceError::Timeout)
    }

    /// Wait up to `timeout` for the device to answer the assignment request.
    fn wait_for_assignment(&self, mcu_id: u32, timeout: Duration) -> bool {
        let deadline = Instant::now() + timeout;
        loop {
            {
                let mut units = self.units.lock().unwrap();
                while let Some(unit) = units.take_unit() {
                    if parse_assignment_ack(&unit) == Some(mcu_id) {
                        return true;
                    }
                }
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return false;
            }
            let window = remaining.min(Duration::from_millis(OUTPUT_WAIT_MS as u64));
            match self.input.recv_timeout(window) {
                Ok(bytes) => self.units.lock().unwrap().feed(&bytes),
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return false,
            }
        }
    }

    /// Receive on a muxed reference: the next unit addressed to this reference,
    /// its header stripped to the frame the protocol carries.
    ///
    /// The units come from this reference's own slot
    /// ([`Router::slots`]), into which the library's reader puts every unit
    /// whose header decodes to this reference's id — and nothing else. Nothing
    /// is filtered out here, and nothing another reference owns can be consumed
    /// here: the reader did both at the only place that sees the whole stream.
    fn receive_mux(&self) -> Option<Frame> {
        loop {
            if self.stopped.load(Ordering::Relaxed) {
                return None;
            }
            {
                let mut units = self.units.lock().unwrap();
                while let Some(unit) = units.take_unit() {
                    let raw = &unit[HEADER_BYTES..];
                    match Frame::parse(raw) {
                        Some(frame) => {
                            trace!("rx frame [{}]: {}", self.id(), describe_frame(raw));
                            debug!("received {} bytes from klipper", raw.len());
                            return Some(frame);
                        }
                        None => warn!(
                            "klipper sent {} bytes under a mux header that are not a frame",
                            raw.len()
                        ),
                    }
                }
            }
            // `recv_timeout` rather than `recv`, so this reference's own
            // `shutdown` is noticed within one interval even though the library
            // — and its reader — keeps running for other references.
            match self
                .input
                .recv_timeout(Duration::from_millis(OUTPUT_WAIT_MS as u64))
            {
                Ok(bytes) => self.units.lock().unwrap().feed(&bytes),
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return None,
            }
        }
    }
}

impl MuxRoute {
    /// The route for `mcu_id`, or an error once it leaves the addressable
    /// range — which the allocator never hands out, so a value that does is a
    /// programming error reported rather than a header built from nothing.
    fn for_id(mcu_id: u32) -> Result<Self, InterfaceError> {
        let arb = assign::arb_id(mcu_id).ok_or_else(|| {
            InterfaceError::Other(format!(
                "mcu id {mcu_id} is outside the range a mux header can address"
            ))
        })?;
        Ok(Self { mcu_id, arb })
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
            .field("library", &self.library.router.library_path)
            .field("reference", &self.reference)
            .field("stopped", &self.stopped.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Device for HostDevice {
    fn send(&self, frame: &Frame) -> Result<(), InterfaceError> {
        let raw = frame.raw_bytes();
        trace!("tx frame [{}]: {}", self.id(), describe_frame(&raw));
        // A muxed reference writes the header naming its id before the frame;
        // the bare-frame modes write the frame alone, today's wire format.
        let mut bytes = Vec::with_capacity(raw.len() + HEADER_BYTES);
        if let Some(mux) = &self.mux {
            bytes.extend_from_slice(&encode_header(mux.arb));
        }
        bytes.extend_from_slice(&raw);
        self.write_raw(&bytes)?;
        debug!("sent {} bytes to klipper", bytes.len());
        Ok(())
    }

    fn receive(&self) -> Option<Frame> {
        // A muxed reference reads the units routed to its own id.
        if self.mux.is_some() {
            return self.receive_mux();
        }
        // A bytes-mode library hands over the mux byte stream, in which a unit
        // is not a frame — framing it here would read the unit's header byte as
        // a length. That mode belongs to the bench scan, which reads
        // [`HostDevice::read`] instead.
        if self.library.router.mode == OutputMode::Bytes {
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
                .input
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
    use crate::core::klippy::assign;
    use crate::core::klippy::cmd::identify::{IdentifyChunk, IDENTIFY_CHUNK_SIZE};
    use crate::core::klippy::cmd::{McuResponse, Params};
    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::mcu::McuConfig;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::config::{ConfigSection, ConfigValue, ConfigWrapper};
    use crate::core::klippy::frame::{
        FrameStream, MESSAGE_DEST, MESSAGE_HEADER_SIZE, MESSAGE_MAX, MESSAGE_MIN, MESSAGE_SYNC,
    };
    use crate::core::klippy::identify::IDENTIFY_MESSAGES;
    use crate::core::klippy::interface::devices::serial::{
        admin_unit, assignment_request, decode_header, discovery_query, encode_header,
        set_node_id_for, set_port_node_id, HeaderScan, UnassignedNode, ADMIN_LISTEN, ADMIN_REPLY,
        ASSIGN_ATTACH_ATTEMPTS, CMD_REQUEST_BOOTLOADER, CMD_RESET_PORT, CMD_RESTART_DEVICE,
        CMD_SET_PORT_NODEID, HEADER_BYTES, MAGIC, MUX_PORT_UNITS, RESP_NEED_NODEID,
    };
    use crate::core::klippy::interface::pty::Pty;
    use crate::core::klippy::interface::SimulatorDevice;
    use crate::core::klippy::mcu::host_libraries::HostLibraries;
    use crate::core::klippy::mcu::{Dictionary, Mcu};
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::{ArgValue, Payload};
    use crate::core::klippy::msg::Msg;
    use crate::core::klippy::Interface;
    use std::io::{Read, Write};
    use std::os::fd::AsRawFd;
    use std::time::Instant;

    #[test]
    fn test_load_reports_a_missing_library() {
        // The loader consults the process-wide registry (one host library per
        // process), so even an attempt on a path that does not exist must not
        // race another test's live library — it takes the same serialization
        // every host-library test does.
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
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

    /// One host library per process: a second, **different** `host_library:`
    /// path cannot be loaded while another library is live, because klipper
    /// stops its main loop through a process-wide `SIGUSR1` handler that each
    /// library installs over the last one — two in one process meet a crash or a
    /// join that never returns, depending on which is unloaded first.
    ///
    /// The guard is **by path**: several references on the one live path (the
    /// mux shape) load as before, and once the first library has been unloaded
    /// another path loads as its own fresh image would.
    #[test]
    fn test_a_second_host_library_path_is_refused_while_one_is_live() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // Several references on the one path are one library, not two.
        let first = HostDevice::load_bytes(library_path(), None).expect("the first library loads");
        let same =
            HostDevice::load_bytes(library_path(), None).expect("the same path is one library");

        // A **different** path while that one is live is refused, naming both.
        let err = HostDevice::load_bytes(mux_library_path(), None)
            .expect_err("a second host library path is not allowed in one process");
        match err {
            InterfaceError::Other(message) => {
                assert!(
                    message.contains("one host library per process"),
                    "{message}"
                );
                assert!(
                    message.contains(&library_path().display().to_string()),
                    "{message}"
                );
                assert!(
                    message.contains(&mux_library_path().display().to_string()),
                    "{message}"
                );
            }
            other => panic!("expected Other, got {other:?}"),
        }

        // Unloading the live library frees the process for another path: the
        // guard is about still-live libraries, not about a path being used at
        // all.
        drop(same);
        drop(first);
        wait_for_unload_blocking(&library_path());

        let mux =
            HostDevice::load_bytes(mux_library_path(), None).expect("a later path loads again");
        drop(mux);
        wait_for_unload_blocking(&mux_library_path());
    }

    /// A live library is opened once, with one bridge port table: a later
    /// reference that names its own ttys cannot replace it, so it is refused by
    /// name instead of silently keeping the table the first load named — or,
    /// when that load named none, the path the library was built with.
    ///
    /// This is the reference-order trap of a muxed `host_library:` path: a
    /// device behind a bridge port names no ports of its own, so it may open the
    /// library before the bridge does; the bridge's ports would then be quietly
    /// ignored, which is exactly the fallback to `CONFIG_HOST_BRIDGE_SERIAL_PATH`
    /// a run-time port path exists to remove.
    #[tokio::test]
    async fn test_a_live_library_refuses_a_different_bridge_port_table() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        let identity = [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc];
        let mux = HostMux {
            identity,
            named: identity,
        };
        let table = vec![(0u8, "/dev/klipperx-port-0".to_string())];

        // The load that opens the library names the ports, fixing the table.
        let first = HostDevice::load_mux(mux_library_path(), mux, 1, &table)
            .expect("the mux library opens with the port table");

        // The same table, and no table at all, reuse the live library: a
        // reference that names none is a device behind a bridge port, which
        // names nothing and is served either way.
        let same = HostDevice::load_mux(mux_library_path(), mux, 2, &table)
            .expect("the same port table reuses the library");
        let downstream = HostDevice::load_mux(mux_library_path(), mux, 3, &[])
            .expect("a reference that names no ports reuses the library");

        // Another table is what the live library cannot serve: the ttys are
        // fixed when it was opened, so the reference is refused by name rather
        // than silently met by the table the first load named.
        let err = HostDevice::load_mux(
            mux_library_path(),
            mux,
            4,
            &[(0u8, "/dev/klipperx-port-other".to_string())],
        )
        .expect_err("a live library cannot take another reference's port table");
        match err {
            InterfaceError::Other(message) => {
                assert!(
                    message.contains("already loaded with a different set of bridge port ttys"),
                    "{message}"
                );
                assert!(
                    message
                        .contains("klipper_host_port() must be called before klipper_host_init()"),
                    "{message}"
                );
            }
            other => panic!("expected Other, got {other:?}"),
        }

        drop(downstream);
        drop(same);
        drop(first);
        wait_for_unload(&mux_library_path()).await;
    }

    /// A muxed attach to a library that does not run the mux firmware sends the
    /// handshake and never hears back, so it fails with a timeout — the same
    /// failure a shared serial port misses its handshake on, and never a broken
    /// session (`serial-mux-protocol.md`). The test library is built for the
    /// host without `CONFIG_MUX_SERIAL`, so it is exactly that old firmware
    /// (its parser drops the unit's first byte as a bad length and answers
    /// nothing).
    #[test]
    fn test_a_muxed_attach_times_out_against_a_library_without_the_mux() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let mux = HostMux {
            identity: [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc],
            named: [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc],
        };
        let err = HostDevice::load_mux(library_path(), mux, 1, &[])
            .expect_err("a plain library never answers the mux handshake");
        assert!(
            matches!(err, InterfaceError::Timeout),
            "expected Timeout, got {err:?}"
        );
        // The failed attach dropped its only reference, so the library went with
        // it; the next host test meets a fresh firmware.
        assert!(
            library_registry()
                .lock()
                .unwrap()
                .get(library_path().as_path())
                .and_then(Weak::upgrade)
                .is_none(),
            "the library should have been unloaded with the failed attach"
        );
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

    /// Wait — bounded — for the shared host library at `path` to be unloaded.
    ///
    /// Dropping the last `Interface` does not unload it the very moment the
    /// caller lets go: a transport task that [`Mcu::close`] aborted may still
    /// hold the last `Arc<HostDevice>` — and thus the last `Arc<HostLibrary>` —
    /// for a moment while it unwinds. The unload happens only when every such
    /// reference is gone ([`HostLibrary`]'s `Drop`), which is what polling the
    /// registry's [`Weak`] entry observes. Bounds a hang in a test that would
    /// otherwise leave the library loaded into the next test (which must meet a
    /// fresh firmware, not this one's session).
    ///
    /// `path` matters: the plain library and the mux one are two registry
    /// entries and two images, so a wait on the wrong one returns immediately
    /// and lets the *next* test on that path meet a library that is still
    /// initialized — `klipper_host_set_uuid` then answers -1, the identity
    /// having to be given before `init`.
    async fn wait_for_unload(path: &Path) {
        for _ in 0..200 {
            if library_unloaded(path) {
                return;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        panic!("the host library at {} never unloaded", path.display());
    }

    /// The same bounded wait for the tests that are not async. Today's two
    /// callers hold no transport task, so their drop is synchronous and this
    /// returns on its first poll; the wait is here to keep the rule — *no test
    /// leaves an initialized image of this path behind* — true if such a test
    /// ever grows a reference of its own.
    fn wait_for_unload_blocking(path: &Path) {
        for _ in 0..200 {
            if library_unloaded(path) {
                return;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        panic!("the host library at {} never unloaded", path.display());
    }

    /// Has every reference to the shared library at `path` been released (its
    /// registry `Weak` entry is dead)?
    ///
    /// Not quite "the image is unmapped": the strong count reaches zero before
    /// [`HostLibrary`]'s `Drop` has run, and the `dlclose` is inside that. A
    /// waiter therefore relies on the drop's tail being short — the functional
    /// probe for "this really is a fresh mapping" is the session-takeover
    /// assertion in [`test_the_library_unloads_with_its_last_reference`], not
    /// this predicate.
    fn library_unloaded(path: &Path) -> bool {
        library_registry()
            .lock()
            .unwrap()
            .get(path)
            .and_then(Weak::upgrade)
            .is_none()
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
    /// A muxed `host_library:` path attaches **through the mux**: the identity
    /// the load-time pass assigns the path is handed to the library before it
    /// comes up, the assignment names it, and the firmware's own device answers
    /// the handshake and speaks header-addressed frames from then on.
    ///
    /// Two sections name one library because that is what makes the path muxed,
    /// and the one without a `uuid` claims the bridge's own device - the one
    /// this test attaches to (the other section's device lives behind the
    /// bridge and needs a downstream board on the tty).
    #[tokio::test]
    async fn test_a_muxed_host_path_attaches_to_the_real_library() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // The bridge's port is opened when the host declares it at
        // configuration time, so the tty has
        // to exist first - the same trick the device-half test uses.
        let pty = Pty::new();
        let bridge_path = klipperx_test_support::klipper_host_mux_serial_path();
        let _ = std::fs::remove_file(&bridge_path);
        std::os::unix::fs::symlink(&pty.path, &bridge_path).expect("the bridge port path");

        let library = mux_library_path();
        let path = library.display().to_string();
        let text = format!(
            "[mcu]\nhost_library: {path}\n\
             [mcu child]\nhost_library: {path}\nuuid: 1234567890ab\n"
        );
        let (config, _) = Config::from_text(&text).expect("the config parses");
        let section = config
            .sections()
            .find(|section| section.id == "mcu")
            .expect("the config has an [mcu] section");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);

        // The config layer's own judgement: two sections on one path are muxed,
        // and the path gets an identity - the value the library is told.
        let libraries = HostLibraries::classify(&wrapper).expect("the path is told apart");
        assert!(libraries.is_mux(&path), "two sections on a path are muxed");
        let identity = libraries.identity(&path).expect("the path has an identity");

        let mut mcu = McuConfig::new(&wrapper).expect("the [mcu] section");
        mcu.id = Some(7);
        mcu.host_mux = Some(HostMux {
            identity,
            named: identity,
        });
        let interface = mcu.open().expect("the muxed library opens and attaches");
        let mcu = identify_and_clock(&mcu, &interface).await;
        assert!(mcu.is_identified());

        // The attach spawned a transport task; when the references go, that task
        // may still hold the last `Arc<HostDevice>` (and so the library) while it
        // unwinds. Wait for *this* path's unload - not the plain library's - so
        // the next test on it meets a fresh firmware rather than an image whose
        // `init` has already happened (`klipper_host_set_uuid` then answers -1).
        drop(mcu);
        drop(interface);
        wait_for_unload(&mux_library_path()).await;
    }

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
        wait_for_unload(&library_path()).await;
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
        wait_for_unload(&library_path()).await;
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
        wait_for_unload(&library_path()).await;
    }

    // =======================================================================
    // The firmware's serial-mux device half, over a real library
    // =======================================================================

    /// The mux build's library.
    ///
    /// This test lives with the other host-library tests because it drives the
    /// same device layer ([`HostDevice::load_bytes`]) against the same real
    /// firmware, and because it has to hold [`HOST_LIBRARY_TEST_LOCK`]: the
    /// library registry is a per-path process-wide singleton, so every test
    /// that loads a host library shares that one spine.
    fn mux_library_path() -> PathBuf {
        klipperx_test_support::klipper_host_mux_lib_path()
    }

    /// The data dictionary of the mux build, for the message ids the command
    /// frame below is built from — the same firmware dictionary an identify
    /// would fetch, read from the file `make` wrote instead.
    fn mux_dictionary() -> Dictionary {
        let path = klipperx_test_support::klipper_host_mux_dictionary_path();
        let raw =
            std::fs::read(&path).unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
        Dictionary::from_json(serde_json::from_slice(&raw).expect("JSON"))
            .expect("the mux build's data dictionary")
    }

    /// One command frame the host would send: the message `name` encoded with
    /// no arguments (id plus args, the way [`Parser::encode`] produces it) in a
    /// frame carrying the sequence byte `seq`.
    fn command_frame(parser: &Parser, name: &str, seq: u8) -> Vec<u8> {
        encode_command_frame(parser, name, &[], seq)
    }

    /// As [`command_frame`], with the command's arguments — the shape a
    /// configuration command (`config_uart`, `allocate_oids`) needs.
    fn encode_command_frame(parser: &Parser, name: &str, args: &[ArgValue], seq: u8) -> Vec<u8> {
        let payload = parser
            .encode(name, args)
            .unwrap_or_else(|e| panic!("encode {name}: {e}"));
        Frame::encode(seq, payload.payload())
    }

    /// Create the bridge's port 0 on a freshly loaded library: the oid table,
    /// the UART object on slot 0, and the port that carries it — the pair of
    /// configuration commands the loader queues (`mcu/bridge_ports.rs`,
    /// `third_party/klipper/src/uart.c`).
    ///
    /// The UART object travels with slot 0, the port's index on the bridge: the
    /// tty it opens is the one the library was told for that slot before it came
    /// up (`HostDevice::load_mux`), or the path this build baked in when the
    /// caller told it nothing.
    ///
    /// The bridge has no compile-time port, so every test that wants port 0
    /// starts here — after the tty exists. The three frames use advancing
    /// sequence bytes, which is the discipline the firmware's `next_sequence`
    /// checks (`command.c`): a repeated sequence is a lost message, discarded
    /// until retransmitted.
    fn configure_port_zero(device: &mut HostDevice) {
        let mut parser = Parser::new();
        mux_dictionary()
            .install(&mut parser)
            .expect("install the mux build's dictionary");
        for (seq, (name, args)) in [
            ("allocate_oids", vec![ArgValue::UInt8(2)]),
            (
                "config_uart",
                vec![
                    ArgValue::UInt8(0),
                    ArgValue::UInt32(250_000),
                    ArgValue::UInt8(0),
                ],
            ),
            (
                "config_bridge_port",
                vec![ArgValue::UInt8(1), ArgValue::UInt8(0)],
            ),
        ]
        .into_iter()
        .enumerate()
        {
            let frame = encode_command_frame(&parser, name, &args, seq as u8);
            device
                .write(&frame)
                .unwrap_or_else(|e| panic!("the {name} frame goes in: {e}"));
        }
    }

    /// A whole unit's arbitration id and the frame behind its header.
    fn unit_parts(unit: &[u8]) -> (u32, &[u8]) {
        match decode_header(unit) {
            HeaderScan::Complete { arb, consumed } => (arb, &unit[consumed..]),
            other => panic!("a unit starts with a header, got {other:?}"),
        }
    }

    /// Cut one whole unit — the two-byte header plus the frame behind it — off
    /// the front of `pending`, the framing
    /// [`serial_query_unassigned`](crate::core::klippy::interface::devices::serial::serial_query_unassigned)
    /// applies, including its resynchronisation: a byte that is not a header is
    /// dropped and the parse picks up at the next SYNC.
    ///
    /// The resync is what skips the bare `starting` frame a freshly booted
    /// firmware sends before it has a node id (see `sched.c`'s `sched_main`):
    /// a device with no node id keeps the old wire format, so the first thing
    /// on the stream is not a unit.
    fn take_unit(pending: &mut Vec<u8>) -> Option<Vec<u8>> {
        loop {
            let consumed = match decode_header(pending) {
                HeaderScan::Complete { consumed, .. } => consumed,
                HeaderScan::NeedMore => return None,
                HeaderScan::Invalid => {
                    resync(pending)?;
                    continue;
                }
            };
            if pending.len() < consumed + 1 {
                return None; // the length byte is still in flight
            }
            let length = pending[consumed] as usize;
            if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&length) {
                resync(pending)?;
                continue;
            }
            if pending.len() < consumed + length {
                return None; // the rest of the frame is still in flight
            }
            return Some(pending.drain(..consumed + length).collect());
        }
    }

    /// Drop everything up to and including the next SYNC; `None` when none is
    /// buffered yet (the caller waits for more bytes).
    fn resync(pending: &mut Vec<u8>) -> Option<()> {
        let position = pending.iter().position(|&byte| byte == MESSAGE_SYNC)?;
        pending.drain(..=position);
        Some(())
    }

    /// Read from `device` until one whole unit is buffered, for at most
    /// `timeout`; `None` when nothing arrived in the window.
    fn read_unit(
        device: &mut HostDevice,
        pending: &mut Vec<u8>,
        timeout: Duration,
    ) -> Option<Vec<u8>> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(unit) = take_unit(pending) {
                return Some(unit);
            }
            if Instant::now() >= deadline {
                return None;
            }
            let mut buf = [0u8; 256];
            match device.read(&mut buf) {
                Ok(0) => {} // the read window expired; recheck the deadline
                Ok(read) => pending.extend_from_slice(&buf[..read]),
                Err(e) => panic!("reading from the host library: {e}"),
            }
        }
    }

    /// `klipper_host_is_shutdown` out of the mapping the device already holds.
    ///
    /// The device's own `Library` is this test's handle to the one loaded
    /// mapping, so the symbol reads the same firmware the device talks to.
    fn is_shutdown_symbol(
        device: &HostDevice,
    ) -> libloading::Symbol<'_, unsafe extern "C" fn() -> c_uchar> {
        unsafe {
            device
                .library
                ._library
                .get::<unsafe extern "C" fn() -> c_uchar>(b"klipper_host_is_shutdown\0")
        }
        .expect("the mux host library exports klipper_host_is_shutdown")
    }

    /// The firmware's serial-mux device half, over a real library.
    ///
    /// A device with no node id answers the discovery broadcast with the
    /// identity the host named it; it adopts the id an assignment hands it and
    /// goes quiet in scans; a data unit addressed to that id has its header
    /// stripped and is dispatched, its answer led by the reply header; and an
    /// assignment handing the id to another identity gives it up and stops the
    /// device.
    ///
    /// The Rust side of these rules is covered by the serial mux's own tests;
    /// this one pins them against the firmware `make` actually built, so a
    /// regression in `third_party/klipper/src/mux.c` or in the console's
    /// `mux_input` hook turns it red instead of leaving the fixtures green.
    #[test]
    fn test_the_mux_device_half_over_the_real_library() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // The port is opened when the host declares it at configuration time
        // (`config_uart`), so the tty has to exist before that: a pty whose
        // slave is reachable at the path this build baked in. The `Pty` keeps
        // both ends open for the whole test. (This test sends no configuration
        // commands of its own, so the pty is only here to keep the path valid.)
        let pty = Pty::new();
        let bridge_path = klipperx_test_support::klipper_host_mux_serial_path();
        let _ = std::fs::remove_file(&bridge_path);
        std::os::unix::fs::symlink(&pty.path, &bridge_path).unwrap_or_else(|e| {
            panic!(
                "failed to link {} to {}: {e}",
                bridge_path.display(),
                pty.path.display()
            )
        });

        let identity = [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc];
        let mut device = HostDevice::load_bytes(mux_library_path(), Some(identity))
            .expect("the mux host library opens");
        let mut pending = Vec::new();
        let window = Duration::from_secs(5);

        let mut parser = Parser::new();
        mux_dictionary()
            .install(&mut parser)
            .expect("install the mux build's dictionary");

        // 1. The unnamed device answers the discovery broadcast with its
        //    identity, under the admin reply header.
        device
            .write(&discovery_query())
            .expect("the discovery query goes in");
        let answer = read_unit(&mut device, &mut pending, window).expect("the scan answer");
        let (arb, frame) = unit_parts(&answer);
        assert_eq!(arb, ADMIN_REPLY, "the answer is on the admin reply id");
        let node = UnassignedNode::from_unit(&answer).expect("a well-formed admin answer");
        assert_eq!(
            node.uuid, identity,
            "the device answers with the identity the host named"
        );
        assert_eq!(
            node.application, None,
            "the library sends no application byte"
        );
        let payload = &frame[MESSAGE_HEADER_SIZE..];
        assert_eq!(
            &payload[..MAGIC.len()],
            &MAGIC[..],
            "the answer rides the KXID frame"
        );
        assert_eq!(
            payload[MAGIC.len()],
            RESP_NEED_NODEID,
            "the answer asks for a node id"
        );

        // 2. The assignment is adopted, so a second scan is answered by
        //    silence — what makes a scan list only unaddressed boards.
        device
            .write(&set_node_id_for(identity, 5).expect("id 5 is addressable"))
            .expect("the assignment goes in");
        device
            .write(&discovery_query())
            .expect("the second scan goes in");
        assert!(
            read_unit(&mut device, &mut pending, Duration::from_millis(500)).is_none(),
            "an addressed device must stay quiet in a scan"
        );

        // 3. A data unit addressed to the adopted id has its header stripped and
        //    is dispatched: `get_clock` comes back as the framed `clock`
        //    response and the block-level ack, both led by the reply header.
        //    The frame carries MESSAGE_DEST because that is where a freshly
        //    booted firmware's receive sequence starts (`command.c`).
        let reply_arb = assign::reply_arb_id(5).expect("id 5");
        let mut unit = encode_header(assign::arb_id(5).expect("id 5"));
        unit.extend_from_slice(&command_frame(&parser, "get_clock", MESSAGE_DEST));
        device.write(&unit).expect("the get_clock unit goes in");

        let response = read_unit(&mut device, &mut pending, window).expect("the clock response");
        let (arb, frame) = unit_parts(&response);
        assert_eq!(arb, reply_arb, "our frames go out on our reply header");
        let clock = Frame::parse(frame).expect("the response is a whole frame");
        let decoded = parser
            .decode(Payload::from_raw(clock.payload().to_vec()))
            .expect("the response decodes against the dictionary");
        assert_eq!(decoded.len(), 1, "one message in the response frame");
        assert_eq!(decoded[0].0.name, "clock", "get_clock is answered by clock");
        assert!(
            matches!(decoded[0].1.as_slice(), [ArgValue::UInt32(_)]),
            "clock is a `%u` value, got {:?}",
            decoded[0].1
        );

        let ack = read_unit(&mut device, &mut pending, window).expect("the block ack");
        let (arb, frame) = unit_parts(&ack);
        assert_eq!(arb, reply_arb, "the ack is headed too");
        assert_eq!(
            frame.len(),
            MESSAGE_MIN,
            "the block ack is the empty ack frame (len, seq, crc, sync)"
        );

        // 4. An assignment naming another identity for the id we hold gives the
        //    id up and stops the device (the CAN id-conflict rule). It ends the
        //    session, so it comes last and is all that is left to assert.
        device
            .write(&set_node_id_for([0xde, 0xad, 0xbe, 0xef, 0x00, 0x01], 5).expect("id 5"))
            .expect("the conflicting assignment goes in");

        let is_shutdown = is_shutdown_symbol(&device);
        let deadline = Instant::now() + window;
        while unsafe { is_shutdown() } == 0 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(10));
        }
        assert_ne!(
            unsafe { is_shutdown() },
            0,
            "an assignment that hands our id to another identity shuts us down"
        );

        // Keep the invariant the mux tests share: this path is unloaded again
        // before the next test takes it. The drop here is synchronous, so the
        // wait is a no-op today — it is what keeps the rule true if this test
        // ever holds a transport task of its own.
        drop(device);
        wait_for_unload_blocking(&mux_library_path());
    }

    /// Read from the pty's master end for at most `window`: everything the
    /// firmware wrote to the port, handed over as soon as `at_least` bytes are
    /// in hand — empty when the port stays silent the whole window. Every wait
    /// is bounded, so no probe step can hang on a port that never writes.
    fn read_pty(master: &std::fs::File, at_least: usize, window: Duration) -> Vec<u8> {
        let deadline = Instant::now() + window;
        let mut out = Vec::new();
        let mut buf = [0u8; 256];
        // `&File` implements `Read`, so the shared handle reads through.
        let mut reader = master;
        while out.len() < at_least {
            let now = Instant::now();
            if now >= deadline {
                break;
            }
            let wait = deadline.saturating_duration_since(now).as_millis() as i32;
            let mut poll = libc::pollfd {
                fd: master.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            };
            let rc = unsafe { libc::poll(&mut poll, 1, wait.max(1)) };
            if rc < 0 {
                let error = std::io::Error::last_os_error();
                if error.kind() == std::io::ErrorKind::Interrupted {
                    continue;
                }
                panic!("polling the pty master: {error}");
            }
            if rc == 0 {
                break; // the window expired with nothing more to read
            }
            match reader.read(&mut buf) {
                Ok(0) => break,
                Ok(read) => out.extend_from_slice(&buf[..read]),
                Err(error) => panic!("reading the pty master: {error}"),
            }
        }
        out
    }

    /// The bridge's port-level command family, over a real library and a real
    /// tty: the port table in the dictionary, `SET_PORT_NODEID` routing in both
    /// wire formats, the unmapped-id negative, and the reset family — pinned
    /// against the firmware `make` actually built, the way
    /// [`test_the_mux_device_half_over_the_real_library`] pins `src/mux.c`.
    ///
    /// The pty on the bridge path is a dumb byte port: it never answers the
    /// admin channel, so every step here also proves the downstream side needs
    /// no admin participation — the route a device with no admin channel of
    /// its own gets.
    #[test]
    fn test_the_bridge_port_commands_over_the_real_library() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // The bridge opens its configured port while the host configures it,
        // so the tty has to exist first — the same setup the device-half test
        // uses.
        let pty = Pty::new();
        let bridge_path = klipperx_test_support::klipper_host_mux_serial_path();
        let _ = std::fs::remove_file(&bridge_path);
        std::os::unix::fs::symlink(&pty.path, &bridge_path).unwrap_or_else(|e| {
            panic!(
                "failed to link {} to {}: {e}",
                bridge_path.display(),
                pty.path.display()
            )
        });

        let identity = [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc];
        let mut device = HostDevice::load_bytes(mux_library_path(), Some(identity))
            .expect("the mux host library opens");
        let mut pending = Vec::new();
        let window = Duration::from_secs(5);
        // A short window for the negative steps: enough for the task loop to
        // have run, short enough to keep the silence checks cheap.
        let quiet = Duration::from_millis(500);

        // A. The mux build's dictionary carries the two configuration commands
        //    that create a bridge's ports (`config_uart` / `config_bridge_port`,
        //    `third_party/klipper/src/uart.c`) and no compile-time port table:
        //    the ports are the host's to name at configuration time.
        let dictionary = mux_dictionary();
        for name in ["config_uart", "config_bridge_port"] {
            assert!(
                dictionary.message(name).is_some(),
                "{name} is in the mux build's dictionary"
            );
        }
        for gone in [
            "BRIDGE_PORT_COUNT",
            "BRIDGE_PORT0_NAME",
            "BRIDGE_PORT0_TYPE",
            "BRIDGE_PORT0_PATH",
        ] {
            assert!(
                dictionary.constant(gone).is_none(),
                "the compile-time port table ({gone}) is gone: ports are declared at \
                 configuration time"
            );
        }
        let mut parser = Parser::new();
        dictionary
            .install(&mut parser)
            .expect("install the mux build's dictionary");

        // The firmware's port numbering starts from the host's configuration:
        // create port 0's stream (the tty) and hand it to the bridge.
        configure_port_zero(&mut device);

        // B. `SET_PORT_NODEID(0, 7, 0)` names a legacy device on port 0. It is
        //    the bridge's own business: no receipt comes back upstream and
        //    nothing is written to the port.
        device
            .write(&set_port_node_id(0, 7, 0))
            .expect("SET_PORT_NODEID goes in");
        assert!(
            read_pty(&pty.master, 1, quiet).is_empty(),
            "SET_PORT_NODEID must not be forwarded to the port"
        );
        assert!(
            read_unit(&mut device, &mut pending, quiet).is_none(),
            "SET_PORT_NODEID carries no receipt"
        );

        // C. A data unit for the port's id (7) reaches the device behind it
        //    stripped of its header — the legacy wire format.
        let down_frame = command_frame(&parser, "get_clock", MESSAGE_DEST);
        let mut down_unit = encode_header(assign::arb_id(7).expect("id 7"));
        down_unit.extend_from_slice(&down_frame);
        device.write(&down_unit).expect("the id 7 unit goes in");
        let down = read_pty(&pty.master, down_frame.len(), window);
        assert_eq!(
            down, down_frame,
            "a legacy port strips the header and sends the frame on unchanged"
        );

        // D. A raw frame from the device behind the port comes up headed with
        //    the reply id of the port's node id (7).
        let up_frame = command_frame(&parser, "get_clock", MESSAGE_DEST);
        (&pty.master)
            .write_all(&up_frame)
            .expect("the raw frame goes into the port");
        let up = read_unit(&mut device, &mut pending, window).expect("the frame comes up headed");
        let (arb, frame) = unit_parts(&up);
        assert_eq!(
            arb,
            assign::reply_arb_id(7).expect("id 7"),
            "a raw frame is headed with the reply id of the port's node id"
        );
        assert_eq!(
            frame,
            up_frame.as_slice(),
            "the frame body travels unchanged"
        );

        // E. Port 0 switches to units (`SET_PORT_NODEID(0, 8, MUX_PORT_UNITS)`):
        //    a unit for the port's new id travels whole — header and all — both
        //    ways.
        device
            .write(&set_port_node_id(0, 8, MUX_PORT_UNITS))
            .expect("SET_PORT_NODEID goes in");
        let unit_frame = command_frame(&parser, "get_clock", MESSAGE_DEST);
        let mut headed_down = encode_header(assign::arb_id(8).expect("id 8"));
        headed_down.extend_from_slice(&unit_frame);
        device.write(&headed_down).expect("the id 8 unit goes in");
        let down = read_pty(&pty.master, headed_down.len(), window);
        assert_eq!(
            down, headed_down,
            "a units port forwards the whole unit, header and all"
        );

        // A headed unit from the port is forwarded upstream unchanged. The
        // header is the reply id of 8 — an ordinary data id, so it is never
        // mistaken for an admin reply and starts no uuid learning.
        let mut headed_up = encode_header(assign::reply_arb_id(8).expect("id 8"));
        headed_up.extend_from_slice(&unit_frame);
        (&pty.master)
            .write_all(&headed_up)
            .expect("the unit goes into the port");
        let up = read_unit(&mut device, &mut pending, window).expect("the unit comes up");
        let (arb, frame) = unit_parts(&up);
        assert_eq!(
            arb,
            assign::reply_arb_id(8).expect("id 8"),
            "a headed unit keeps the header it arrived with"
        );
        assert_eq!(
            frame,
            unit_frame.as_slice(),
            "a headed unit is forwarded unchanged"
        );

        // F. A data unit for an id no port owns (9) goes nowhere.
        let mut unmapped = encode_header(assign::arb_id(9).expect("id 9"));
        unmapped.extend_from_slice(&command_frame(&parser, "get_clock", MESSAGE_DEST));
        device.write(&unmapped).expect("the id 9 unit goes in");
        assert!(
            read_pty(&pty.master, 1, quiet).is_empty(),
            "an id no port owns must not reach the port"
        );

        // G. The reset family. `[0x04][0]` pulses port 0's lines — a pty has no
        //    modem lines, so it is dropped in silence, with no receipt.
        //    Written twice back to back so that on real hardware the second
        //    lands inside the first pulse's 250 ms hold, where the firmware must
        //    take it as already done rather than re-read the asserted lines as
        //    "original state" (that would restore the asserted level at the
        //    deadline and hold the port in reset forever — the re-entry guard in
        //    `bridge_port_reset`). That guard is *not* what these two sends
        //    prove: a pty never arms a pulse (TIOCMGET/TIOCMSET cannot run), so
        //    all they pin here is consumption — no port bytes, no receipt.
        device
            .write(&admin_unit(&[CMD_RESET_PORT, 0]))
            .expect("RESET_PORT goes in");
        device
            .write(&admin_unit(&[CMD_RESET_PORT, 0]))
            .expect("a second RESET_PORT goes in");
        assert!(
            read_pty(&pty.master, 1, quiet).is_empty(),
            "RESET_PORT writes nothing to the port"
        );
        assert!(
            read_unit(&mut device, &mut pending, quiet).is_none(),
            "RESET_PORT carries no receipt"
        );

        // SET_PORT_NODEID drops bad arguments in silence too: an out-of-range
        // index and a short payload are consumed without reaching any port and
        // without a receipt.
        device
            .write(&set_port_node_id(99, 7, MUX_PORT_UNITS))
            .expect("an out-of-range SET_PORT_NODEID goes in");
        assert!(
            read_pty(&pty.master, 1, quiet).is_empty(),
            "an out-of-range port index reaches no port"
        );
        assert!(
            read_unit(&mut device, &mut pending, quiet).is_none(),
            "a rejected SET_PORT_NODEID carries no receipt"
        );
        device
            .write(&admin_unit(&[CMD_SET_PORT_NODEID, 0]))
            .expect("a short SET_PORT_NODEID goes in");
        assert!(
            read_pty(&pty.master, 1, quiet).is_empty(),
            "a short payload reaches no port"
        );
        assert!(
            read_unit(&mut device, &mut pending, quiet).is_none(),
            "a short payload carries no receipt"
        );

        // The flashing tool's `[0x02][0]` is forwarded to port 0 alone. Port 0
        // speaks units since step E, so the `KXID` frame goes down headed with
        // the arb it arrived on — the admin listen id.
        let reset = admin_unit(&[CMD_REQUEST_BOOTLOADER, 0]);
        device
            .send_request_bootloader(0)
            .expect("REQUEST_BOOTLOADER goes in");
        let down = read_pty(&pty.master, reset.len(), window);
        let (arb, frame) = unit_parts(&down);
        assert_eq!(
            arb, ADMIN_LISTEN,
            "the forwarded unit keeps the arb it arrived on (the port speaks units)"
        );
        assert_eq!(
            frame,
            &reset[HEADER_BYTES..],
            "the REQUEST_BOOTLOADER KXID frame travels unchanged"
        );

        // And the restart command the named child's `admin` method sends:
        // `[0x05][0]`, the same shape one command byte apart, reaching the port
        // whole for the same reason.
        let restart = admin_unit(&[CMD_RESTART_DEVICE, 0]);
        device
            .send_restart_device(0)
            .expect("RESTART_DEVICE goes in");
        let down = read_pty(&pty.master, restart.len(), window);
        let (arb, frame) = unit_parts(&down);
        assert_eq!(arb, ADMIN_LISTEN, "the restart unit keeps its header too");
        assert_eq!(
            frame,
            &restart[HEADER_BYTES..],
            "the RESTART_DEVICE KXID frame travels unchanged"
        );

        // `[0x02][99]` names no port: dropped in silence.
        device
            .write(&admin_unit(&[CMD_REQUEST_BOOTLOADER, 99]))
            .expect("REQUEST_BOOTLOADER goes in");
        assert!(
            read_pty(&pty.master, 1, quiet).is_empty(),
            "an out-of-range port index reaches no port"
        );

        // And none of the port-level commands touched the library itself: the
        // bridge consumed every one of them, so the device half never ran a
        // bootloader request of its own — the shutdown `is_shutdown` reads.
        let is_shutdown = is_shutdown_symbol(&device);
        assert_eq!(
            unsafe { is_shutdown() },
            0,
            "a port-level reset must not take the library down"
        );

        // Same invariant as the device-half test: leave this path unloaded for
        // whoever takes it next (see `wait_for_unload_blocking`).
        drop(device);
        wait_for_unload_blocking(&mux_library_path());
    }

    /// A downstream reference — a mux id loaded with **no** attach — is the
    /// host's channel to a device behind a bridge port, over a real library and
    /// a real tty.
    ///
    /// Loading it names nothing and waits for nothing (the pty never answers
    /// the admin channel, so an attach could not run), and the route is the
    /// bridge's `SET_PORT_NODEID`, sent by the caller over the new admin entry.
    /// Once the route is in: a frame sent goes down port 0 with its header
    /// stripped, a raw frame from the port comes up headed with the port's reply
    /// id, and an id no port owns reaches no port.
    #[test]
    fn test_a_downstream_reference_routes_through_the_bridge_port() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // The bridge's port is opened when the host declares it at
        // configuration time, so the tty has to
        // exist first — the same setup the port-command probe uses.
        let pty = Pty::new();
        let bridge_path = klipperx_test_support::klipper_host_mux_serial_path();
        let _ = std::fs::remove_file(&bridge_path);
        std::os::unix::fs::symlink(&pty.path, &bridge_path).unwrap_or_else(|e| {
            panic!(
                "failed to link {} to {}: {e}",
                bridge_path.display(),
                pty.path.display()
            )
        });

        let identity = [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc];
        let downstream_id = 7u32;
        let mut device = HostDevice::load_downstream(mux_library_path(), identity, downstream_id)
            .expect("a downstream reference loads without a handshake");

        let mut pending = Vec::new();
        let window = Duration::from_secs(5);
        // A short window for the negative steps, as in the port-command probe.
        let quiet = Duration::from_millis(500);

        // The bridge starts with no ports: the host creates them at
        // configuration time. Port 0 is a UART object handed to the bridge.
        configure_port_zero(Arc::get_mut(&mut device).expect("the test holds the only reference"));

        // Naming the device behind port 0 is the bridge's business: nothing goes
        // down the port and no receipt comes back.
        device
            .send_set_port_node_id(0, downstream_id as u8, 0)
            .expect("SET_PORT_NODEID goes in");
        assert!(
            read_pty(&pty.master, 1, quiet).is_empty(),
            "SET_PORT_NODEID must not be forwarded to the port"
        );
        assert!(
            read_unit(
                Arc::get_mut(&mut device).expect("the test holds the only reference"),
                &mut pending,
                quiet,
            )
            .is_none(),
            "SET_PORT_NODEID carries no receipt"
        );

        // Host → port: the reference's frame goes down port 0 with its header
        // stripped — the legacy wire format a device with no admin channel of
        // its own speaks.
        let down_frame = Frame::new(MESSAGE_DEST, b"downstream".to_vec());
        device.send(&down_frame).expect("the frame goes down");
        let down = read_pty(&pty.master, down_frame.raw_bytes().len(), window);
        assert_eq!(
            down,
            down_frame.raw_bytes(),
            "a legacy port strips the mux header and sends the frame on unchanged"
        );

        // Port → host: a raw frame from the device comes up headed with the
        // reply id of the port's node id (7).
        let up_frame = Frame::new(MESSAGE_DEST, b"upstream".to_vec());
        (&pty.master)
            .write_all(&up_frame.raw_bytes())
            .expect("the raw frame goes into the port");
        let up = read_unit(
            Arc::get_mut(&mut device).expect("the test holds the only reference"),
            &mut pending,
            window,
        )
        .expect("the frame comes up headed");
        let (arb, frame) = unit_parts(&up);
        assert_eq!(
            arb,
            assign::reply_arb_id(downstream_id).expect("id 7"),
            "a raw frame is headed with the reply id of the port's node id"
        );
        assert_eq!(
            frame,
            up_frame.raw_bytes().as_slice(),
            "the frame body travels unchanged"
        );

        // An id no port owns (9) goes nowhere: without a SET_PORT_NODEID the
        // reference's id is not routed either, which is what makes the admin
        // entry above load-bearing.
        let mut unmapped = encode_header(assign::arb_id(9).expect("id 9"));
        unmapped.extend_from_slice(&Frame::new(MESSAGE_DEST, b"junk".to_vec()).raw_bytes());
        Arc::get_mut(&mut device)
            .expect("the test holds the only reference")
            .write(&unmapped)
            .expect("the id 9 unit goes in");
        assert!(
            read_pty(&pty.master, 1, quiet).is_empty(),
            "an id no port owns must not reach the port"
        );

        // Same invariant as the other library tests: leave this path unloaded.
        drop(device);
        wait_for_unload_blocking(&mux_library_path());
    }

    /// A downstream reference's handshake, over a real library and a real tty:
    /// [`HostDevice::attach`] on the reference [`HostDevice::load_downstream`]
    /// opened, run **after** the bridge has been told the route.
    ///
    /// The handshake is the one `load_mux` runs — the SET_KLIPPER_NODEID unit
    /// naming the device, then the assignment request retried
    /// [`ASSIGN_ATTACH_ATTEMPTS`] times — but the firmware's own bridge carries
    /// it: the naming unit is broadcast to the port with its admin header, the
    /// assignment request is routed to id 7's port and kept whole there (a named
    /// device answers the admin channel itself, so its port speaks units). The
    /// pty never answers, so the handshake times out as it would against a
    /// device that is not there.
    ///
    /// The order the entry point exists for is what the byte assertion pins:
    /// with `load_mux` instead — open-and-handshake in one call, no route first —
    /// the request has no port to be routed to when it goes out and the bridge
    /// drops it, so the request units never reach the pty and this test is red.
    #[test]
    fn test_a_downstream_reference_handshakes_along_its_route() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // The bridge's port is opened when the host declares it at
        // configuration time, so the tty has to
        // exist first — the same setup the other port tests use.
        let pty = Pty::new();
        let bridge_path = klipperx_test_support::klipper_host_mux_serial_path();
        let _ = std::fs::remove_file(&bridge_path);
        std::os::unix::fs::symlink(&pty.path, &bridge_path).unwrap_or_else(|e| {
            panic!(
                "failed to link {} to {}: {e}",
                bridge_path.display(),
                pty.path.display()
            )
        });

        let identity = [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc];
        // The named device is **not** the bridge's own device: only the bridge's
        // own uuid is `load_mux`'s to name, and a different one is what makes
        // this a device behind a port.
        let named = [0x99, 0x88, 0x77, 0x66, 0x55, 0x44];
        let downstream_id = 7u32;
        let mut device = HostDevice::load_downstream(mux_library_path(), identity, downstream_id)
            .expect("a downstream reference loads without a handshake");
        configure_port_zero(Arc::get_mut(&mut device).expect("the test holds the only reference"));

        // The route first: port 0 owns id 7, and its device speaks units — it
        // answers the admin channel itself, which is what a named device does.
        device
            .send_set_port_node_id(0, downstream_id as u8, MUX_PORT_UNITS)
            .expect("SET_PORT_NODEID goes in");

        // Nothing behind the pty answers, so the handshake cannot complete.
        let err = device
            .attach(named, downstream_id)
            .expect_err("the pty never answers the assignment");
        assert_eq!(err, InterfaceError::Timeout);

        // Every byte of the handshake reached the port. Naming a uuid the
        // bridge does not know yet also makes it probe the port once
        // (`bridge_probe_unassigned`), which is the discovery query in front.
        let mut expected = discovery_query();
        expected.extend_from_slice(&set_node_id_for(named, downstream_id).expect("id 7"));
        let request = assignment_request(downstream_id).expect("id 7");
        for _ in 0..ASSIGN_ATTACH_ATTEMPTS {
            expected.extend_from_slice(&request);
        }
        let seen = read_pty(&pty.master, expected.len(), Duration::from_secs(5));
        assert_eq!(
            seen, expected,
            "the discovery probe, the SET_KLIPPER_NODEID unit, then a routed request per attempt"
        );

        // Same invariant as the other library tests: leave this path unloaded.
        drop(device);
        wait_for_unload_blocking(&mux_library_path());
    }

    /// Receive one frame from `device` within `window`.
    ///
    /// [`Device::receive`] blocks until a frame arrives, so the wait runs in a
    /// worker thread and this returns `None` when the window expires first: a
    /// routing failure is then a failed assertion, never a hung test. On the
    /// way out the worker is released with [`Device::shutdown`] — `receive`
    /// returns within one poll interval of it — and joined, so no thread is
    /// left holding the library (`wait_for_unload_blocking` would never see it
    /// go).
    ///
    /// A `None` therefore also means "this reference is now stopped", so a
    /// negative check on it belongs at the end of a test.
    fn receive_within(device: &Arc<HostDevice>, window: Duration) -> Option<Frame> {
        let (frames, received) = unbounded();
        let worker = Arc::clone(device);
        let handle = thread::spawn(move || {
            let _ = frames.send(worker.receive());
        });
        let frame = received.recv_timeout(window).ok().flatten();
        if frame.is_none() {
            device.shutdown(); // release the worker out of its blocking receive
        }
        let _ = handle.join();
        frame
    }

    /// Two live references on **one** library do not steal each other's units.
    ///
    /// The library's output is a single byte stream, so the library has **one**
    /// reader — and that reader, not each reference, cuts units out of the
    /// stream and routes each to the slot of the id its header names
    /// (`host.rs`'s router). Two references on the same loaded library, both
    /// receiving, is exactly the session that broke when each held a clone of
    /// the one output channel: the two readers split the bytes, so neither could
    /// assemble a unit.
    ///
    /// Both references are addressed the same way a live bridge reference and a
    /// live downstream one are — by the id in the unit's header — so they take
    /// the routing the real pair takes. The wire is a real pty on the port the
    /// mux build baked in, and that port has been told its device speaks units
    /// ([`MUX_PORT_UNITS`]): a headed unit written into it is forwarded upstream
    /// unchanged (the bridge attributes a unit to the port it *arrived on*, never
    /// to the id in its header), which is how units of each id are put on the one
    /// stream. Traffic is interleaved — one unit per id, in a fixed order — and
    /// each reference is read back in turn, so what is asserted is that each is
    /// handed only its own frames, in its own order.
    #[test]
    fn test_two_live_references_on_a_shared_library_do_not_steal_each_others_units() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // The bridge opens its port during init, so the tty this build baked in
        // has to exist before the library is loaded — the same setup the other
        // bridge tests use.
        let pty = Pty::new();
        let bridge_path = klipperx_test_support::klipper_host_mux_serial_path();
        let _ = std::fs::remove_file(&bridge_path);
        std::os::unix::fs::symlink(&pty.path, &bridge_path).expect("the bridge port path");

        let identity = [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc];
        let first_id = 7u32;
        let second_id = 9u32;

        // The first reference, and the port its id is routed to. The port is
        // declared at configuration time, so those frames go in before anything
        // is sent under an id.
        let mut first = HostDevice::load_downstream(mux_library_path(), identity, first_id)
            .expect("a downstream reference loads without a handshake");
        configure_port_zero(Arc::get_mut(&mut first).expect("the test holds the only reference"));
        first
            .send_set_port_node_id(0, first_id as u8, MUX_PORT_UNITS)
            .expect("SET_PORT_NODEID goes in");

        // The second reference, on the **same** library.
        let second = Arc::new(
            HostDevice::load_downstream(mux_library_path(), identity, second_id)
                .expect("a second reference shares the loaded library"),
        );
        assert_eq!(
            first.library_path(),
            second.library_path(),
            "both references are on the one library"
        );
        let library = library_registry()
            .lock()
            .unwrap()
            .get(mux_library_path().as_path())
            .and_then(Weak::upgrade)
            .expect("the library the references share is loaded");
        assert_eq!(
            library.router.slots.lock().unwrap().len(),
            2,
            "each reference has a slot of its own on the one library"
        );
        drop(library);

        // Interleaved traffic, one unit per id: the header names the id and the
        // frame behind it is what that reference must be handed. All four units
        // go into the port back to back, so the one reader has to cut them out
        // of the stream and route them apart — a reference per reader would
        // split these very bytes between them.
        let first_frame = Frame::new(0, b"first-one".to_vec());
        let second_frame = Frame::new(0, b"second-one".to_vec());
        let third_frame = Frame::new(0, b"first-two".to_vec());
        let fourth_frame = Frame::new(0, b"second-two".to_vec());
        for (id, frame) in [
            (first_id, &first_frame),
            (second_id, &second_frame),
            (first_id, &third_frame),
            (second_id, &fourth_frame),
        ] {
            let mut unit = encode_header(assign::reply_arb_id(id).expect("an addressable id"));
            unit.extend_from_slice(&frame.raw_bytes());
            (&pty.master)
                .write_all(&unit)
                .expect("the port unit goes in");
        }

        // Each reference is handed exactly its own frames, in its own order.
        // A split stream shows up right here: half a unit's bytes cannot parse
        // into its frame, so the frame never arrives and this runs out of
        // window.
        let window = Duration::from_secs(10);
        assert_eq!(
            receive_within(&first, window),
            Some(first_frame.clone()),
            "the first reference's own first frame"
        );
        assert_eq!(
            receive_within(&second, window),
            Some(second_frame.clone()),
            "the second reference's own first frame"
        );
        assert_eq!(
            receive_within(&first, window),
            Some(third_frame.clone()),
            "the first reference's own second frame, still in order"
        );
        assert_eq!(
            receive_within(&second, window),
            Some(fourth_frame.clone()),
            "the second reference's own second frame, still in order"
        );

        // And neither has anything else: every unit was delivered once, to the
        // reference its id names.
        let quiet = Duration::from_millis(500);
        assert!(
            receive_within(&first, quiet).is_none(),
            "the first reference has no unit but its own"
        );
        assert!(
            receive_within(&second, quiet).is_none(),
            "the second reference has no unit but its own"
        );

        // Same invariant as the other library tests: leave this path unloaded.
        drop(first);
        drop(second);
        wait_for_unload_blocking(&mux_library_path());
    }

    // =======================================================================
    // A device with no mux behind the bridge, over a real pty and the bridge
    // =======================================================================

    /// How long the adapter's port reader waits before it turns its loop
    /// around; the device side of the adapter is event driven
    /// ([`Device::receive`] blocks until an answer is queued), so this is the
    /// only poll interval there is.
    const PORT_POLL_MS: i32 = 5;

    /// The fake board behind the bridge's port: the repo's dictionary-driven
    /// responder ([`SimulatorDevice`]) with a byte adapter in front, so it meets
    /// the port the way a board on the wire would.
    ///
    /// The bridge's port is **legacy**: what it forwards down is the request
    /// frame with the mux header stripped, and what it expects up is a raw frame
    /// it heads itself. The responder speaks [`Frame`]s, so two threads bridge
    /// the gap:
    ///
    /// * *port → device* reframes the port's bytes ([`FrameStream`]) and hands
    ///   each [`Frame`] to [`Device::send`], recording every byte it read for
    ///   the test's byte-level assertions;
    /// * *device → port* writes every [`Device::receive`] answer back as raw
    ///   frame bytes.
    ///
    /// Both waits are bounded: the first thread polls the pty for
    /// [`PORT_POLL_MS`], the second blocks in `receive`, which
    /// [`Device::shutdown`] releases. [`Drop`] raises the stop flag, shuts the
    /// device down, and joins both threads, so a failed assertion leaves neither
    /// a thread nor a blocked receive behind.
    ///
    /// The board is simulated; the payload is not. The dictionary served is a
    /// real `.dict` built from the klipper source tree, so what crosses the
    /// bridge is a firmware's own identify body.
    struct DownstreamResponder {
        stop: Arc<AtomicBool>,
        device: Arc<SimulatorDevice>,
        threads: Vec<JoinHandle<()>>,
        /// Every byte the adapter read off the pty: what the bridge wrote down
        /// the port, in order, for the test's byte-level assertions.
        downstream: Arc<Mutex<Vec<u8>>>,
    }

    impl DownstreamResponder {
        /// Serve the data dictionary at `dict` on the far side of `master`, the
        /// pty's master end (the bridge holds the slave through its path).
        fn start(dict: &Path, master: &std::fs::File) -> Self {
            let device = Arc::new(
                SimulatorDevice::new(dict)
                    .unwrap_or_else(|e| panic!("the fake firmware for {}: {e}", dict.display())),
            );
            let stop = Arc::new(AtomicBool::new(false));
            let downstream = Arc::new(Mutex::new(Vec::new()));
            let mut threads = Vec::with_capacity(2);

            let master_in = master.try_clone().expect("the pty master clones");
            let device_in = Arc::clone(&device);
            let stop_in = Arc::clone(&stop);
            let seen = Arc::clone(&downstream);
            threads.push(
                thread::Builder::new()
                    .name("bridge-port-in".to_string())
                    .spawn(move || port_bytes_to_device(master_in, device_in, stop_in, seen))
                    .expect("the port reader thread spawns"),
            );

            let master_out = master.try_clone().expect("the pty master clones");
            let device_out = Arc::clone(&device);
            threads.push(
                thread::Builder::new()
                    .name("bridge-port-out".to_string())
                    .spawn(move || device_answers_to_port(master_out, device_out))
                    .expect("the port writer thread spawns"),
            );

            Self {
                stop,
                device,
                threads,
                downstream,
            }
        }

        /// The bytes the bridge has written down the port so far.
        fn downstream_bytes(&self) -> Vec<u8> {
            self.downstream.lock().unwrap().clone()
        }
    }

    impl Drop for DownstreamResponder {
        fn drop(&mut self) {
            // The stop flag retires the port reader; `shutdown` releases the
            // device reader out of its `receive`.
            self.stop.store(true, Ordering::SeqCst);
            self.device.shutdown();
            for thread in self.threads.drain(..) {
                let _ = thread.join();
            }
        }
    }

    /// Port → device: what the bridge forwarded down is a raw frame stream, so
    /// it is reframed and handed to the device one frame at a time.
    fn port_bytes_to_device(
        master: std::fs::File,
        device: Arc<SimulatorDevice>,
        stop: Arc<AtomicBool>,
        seen: Arc<Mutex<Vec<u8>>>,
    ) {
        let fd = master.as_raw_fd();
        let mut stream = FrameStream::new();
        let mut buf = [0u8; 256];
        let mut reader = &master;
        while !stop.load(Ordering::Relaxed) {
            let mut poll = libc::pollfd {
                fd,
                events: libc::POLLIN,
                revents: 0,
            };
            let ready = unsafe { libc::poll(&mut poll, 1, PORT_POLL_MS) };
            if ready <= 0 || (poll.revents & (libc::POLLIN | libc::POLLHUP | libc::POLLERR)) == 0 {
                continue;
            }
            match reader.read(&mut buf) {
                // The slave side is gone: nothing further can arrive (the same
                // reading of an empty pty read the port probes use).
                Ok(0) => break,
                Ok(read) => {
                    seen.lock().unwrap().extend_from_slice(&buf[..read]);
                    stream.push(&buf[..read]);
                    while let Some(frame) = stream.next_frame() {
                        if let Err(e) = device.send(&frame) {
                            debug!("the fake board refused a frame: {e}");
                        }
                    }
                }
                // A pty master reports EIO once the slave side is gone: nothing
                // further can arrive.
                Err(error) => {
                    debug!("the bridge port stopped: {error}");
                    break;
                }
            }
        }
    }

    /// Device → port: every answer the fake board queues goes back as one raw
    /// frame — the shape the bridge heads with the port's reply id. The loop
    /// ends when [`Device::shutdown`] makes `receive` return `None`.
    fn device_answers_to_port(master: std::fs::File, device: Arc<SimulatorDevice>) {
        while let Some(frame) = device.receive() {
            if (&master).write_all(&frame.raw_bytes()).is_err() {
                break;
            }
        }
    }

    /// Read units from the bridge until a frame decoding to `name` arrives on
    /// `reply_arb`, for at most `window`; the message and its parameters, or
    /// `None` when the window ran out.
    ///
    /// Everything else the port sends — the block ack behind each accepted
    /// request, any other answer — decodes to another message (or to nothing at
    /// all) and is skipped. The unit's arbitration id is asserted rather than
    /// skipped: everything the bridge forwards up from port 0 is headed with
    /// that port's reply id, so a unit under any other id is a routing failure
    /// reported where it happens instead of a silent timeout.
    fn read_reply(
        device: &mut HostDevice,
        pending: &mut Vec<u8>,
        parser: &Parser,
        reply_arb: u32,
        name: &str,
        window: Duration,
    ) -> Option<(Arc<Msg>, Vec<ArgValue>)> {
        let deadline = Instant::now() + window;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return None;
            }
            let unit = read_unit(device, pending, remaining)?;
            let (arb, frame) = unit_parts(&unit);
            assert_eq!(
                arb, reply_arb,
                "every unit from port 0 is headed with that port's reply id"
            );
            let Some(frame) = Frame::parse(frame) else {
                continue;
            };
            let Ok(decoded) = parser.decode(Payload::from_raw(frame.payload().to_vec())) else {
                continue;
            };
            let Some((message, values)) = decoded.into_iter().find(|(msg, _)| msg.name == name)
            else {
                continue;
            };
            return Some((message, values));
        }
    }

    /// The end-to-end shape: a device with **no mux** sits behind a bridge
    /// port, and the host reaches it *through* the bridge.
    ///
    /// The bridge is the real firmware build (`make`'s mux target, loaded
    /// through [`HostDevice::load_downstream`]), the port is a real pty, and the
    /// frames are the protocol's own — the device behind the port is the repo's
    /// dictionary-driven responder ([`DownstreamResponder`]) serving a real
    /// `.dict` the klipper tree built. The only thing simulated is the board.
    ///
    /// The bridge's port table is the route: port 0 holds a legacy device at id
    /// 7 ([`HostDevice::send_set_port_node_id`]), so a frame sent under that id
    /// goes down the port stripped of its mux header, and a raw frame coming
    /// back is headed with the reply id of 7.
    ///
    /// On top of the routing rules
    /// [`test_a_downstream_reference_routes_through_the_bridge_port`] pins with
    /// hand-built frames, this proves the traffic is *answerable* where it
    /// lands: the identify transfer runs to completion across the bridge —
    /// every chunk reassembled, decompressed, and parsed back into the
    /// dictionary that was built — and the dictionary it installs is usable
    /// through the same route, with a `get_clock` round trip.
    ///
    /// The route is load-bearing: without the `SET_PORT_NODEID` the bridge owns
    /// no id for port 0, the request reaches no port, and the first reply read
    /// runs out — the test is red at that `expect`.
    #[test]
    fn test_a_device_without_the_mux_behind_the_bridge_answers_identify() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // The bridge's port is opened when the host declares it at
        // configuration time, so the tty has to
        // exist first — the same setup the other port tests use.
        let pty = Pty::new();
        let bridge_path = klipperx_test_support::klipper_host_mux_serial_path();
        let _ = std::fs::remove_file(&bridge_path);
        std::os::unix::fs::symlink(&pty.path, &bridge_path).unwrap_or_else(|e| {
            panic!(
                "failed to link {} to {}: {e}",
                bridge_path.display(),
                pty.path.display()
            )
        });

        let identity = [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc];
        let downstream_id = 7u32;
        let mut device = HostDevice::load_downstream(mux_library_path(), identity, downstream_id)
            .expect("a downstream reference loads without a handshake");
        configure_port_zero(Arc::get_mut(&mut device).expect("the test holds the only reference"));

        // The bridge's port table: port 0 holds a legacy device at id 7. This is
        // the route — until it is in, nothing sent under id 7 reaches any port.
        device
            .send_set_port_node_id(0, downstream_id as u8, 0)
            .expect("SET_PORT_NODEID goes in");

        // The device behind the port: a real data dictionary served by the
        // repo's fake board, pumped against the pty's other end.
        let dict = klipperx_test_support::test_dicts_dir().join("linuxprocess.dict");
        let fixture = DownstreamResponder::start(&dict, &pty.master);

        // The parser knows only the two host-owned identify formats — exactly
        // what an MCU knows before its dictionary arrives.
        let mut parser = Parser::new();
        parser
            .register_all(IDENTIFY_MESSAGES)
            .expect("the host-owned identify formats");

        let reply_arb = assign::reply_arb_id(downstream_id).expect("id 7");
        let mut pending = Vec::new();
        // Generous, because each leg crosses a thread and a pty; it is also the
        // bound that makes a missing route a failure rather than a hang.
        let window = Duration::from_secs(10);

        // The identify transfer, chunk by chunk: ask for the payload so far,
        // take the `identify_response` for it off the port's reply header, repeat
        // until the empty answer. The sequence byte advances per request — the
        // discipline the firmware's `next_sequence` checks (`command.c`).
        let mut payload: Vec<u8> = Vec::new();
        let mut requests: Vec<Vec<u8>> = Vec::new();
        let mut seq = 0u8;
        loop {
            let request = parser
                .encode(
                    "identify",
                    &[
                        ArgValue::UInt32(payload.len() as u32),
                        ArgValue::UInt8(IDENTIFY_CHUNK_SIZE),
                    ],
                )
                .expect("the identify request encodes");
            let frame = Frame::new(seq, request.payload().to_vec());
            seq = seq.wrapping_add(1);
            requests.push(frame.raw_bytes());
            device
                .send(&frame)
                .expect("the request goes down the bridge");

            let (message, values) = read_reply(
                Arc::get_mut(&mut device).expect("the test holds the only reference"),
                &mut pending,
                &parser,
                reply_arb,
                "identify_response",
                window,
            )
            .expect("the device behind the port answers the identify request");
            let chunk = IdentifyChunk::decode(&Params::new(message, &values))
                .expect("the answer is an identify chunk");
            assert_eq!(
                chunk.offset as usize,
                payload.len(),
                "the answer names the offset that was asked for"
            );
            if chunk.data.is_empty() {
                break;
            }
            payload.extend_from_slice(&chunk.data);
        }

        // What came back is the firmware's own payload, reassembled chunk by
        // chunk above — and it is the dictionary file itself, byte for byte.
        let raw = std::fs::read(&dict).expect("the data dictionary file");
        let mut body = Vec::new();
        flate2::read::ZlibDecoder::new(&payload[..])
            .read_to_end(&mut body)
            .expect("the identify payload is zlib");
        assert_eq!(
            body, raw,
            "the payload the bridge carried is the built dictionary, unchanged"
        );
        let dictionary = Dictionary::from_json(
            serde_json::from_slice(&body).expect("the identify body is JSON"),
        )
        .expect("the firmware's data dictionary");
        let message_count = dictionary.messages().count();
        assert!(
            message_count >= 20,
            "expected the firmware's whole dictionary, got {message_count} messages"
        );
        for name in ["get_clock", "clock"] {
            assert!(
                dictionary.message(name).is_some(),
                "{name} is in the dictionary identify installed"
            );
        }
        assert!(
            dictionary.constant_f64("CLOCK_FREQ").is_some(),
            "the dictionary carries the clock its firmware runs at"
        );
        debug!(
            "identify through the bridge: {} compressed bytes, {message_count} messages",
            payload.len()
        );

        // The dictionary identify installed is usable through the same route: a
        // `get_clock` goes down port 0 and comes back as the framed `clock`
        // response.
        dictionary
            .install(&mut parser)
            .expect("install the identified dictionary");
        let request = parser
            .encode("get_clock", &[])
            .expect("the clock request encodes");
        let frame = Frame::new(seq, request.payload().to_vec());
        requests.push(frame.raw_bytes());
        device
            .send(&frame)
            .expect("the request goes down the bridge");

        let (message, values) = read_reply(
            Arc::get_mut(&mut device).expect("the test holds the only reference"),
            &mut pending,
            &parser,
            reply_arb,
            "clock",
            window,
        )
        .expect("the device behind the port answers get_clock");
        assert_eq!(message.name, "clock", "get_clock is answered by clock");
        assert!(
            matches!(values.as_slice(), [ArgValue::UInt32(_)]),
            "clock is a `%u` value, got {values:?}"
        );

        // The port's own side of the wire, byte for byte: every request reached
        // port 0 as its bare frame, mux header stripped, and nothing else was
        // written down the port.
        let expected: Vec<u8> = requests.iter().flatten().copied().collect();
        assert_eq!(
            fixture.downstream_bytes(),
            expected,
            "each request is routed to port 0 and forwarded as its bare frame"
        );

        // Same invariant as the other library tests: leave this path unloaded.
        drop(fixture);
        drop(device);
        wait_for_unload_blocking(&mux_library_path());
    }
}
