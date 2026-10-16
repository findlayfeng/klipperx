//! Flashing a board that hangs on a **bridge's downstream port**: the whole run
//! in one place — open the bridge, identify it, declare its ports, open the
//! flashing tunnel, drive the katapult session over that tunnel, and clear the
//! tunnel.
//!
//! This module is the seam between the two halves T5 built. [`open_bridge`] is
//! the front half (declaration, per-port reset, target routing, tunnel on);
//! [`TunnelBridge`] is the data path (a `Read + Write` face over the port's
//! tunnel units). Both need a **link handle** the host holds directly, and a
//! [`Mcu`] over it for the declaration commands. The flash tool therefore opens
//! the bridge itself — a shared serial handle ([`SharedSerialHandle`]) or a
//! host-library reference ([`HostDevice`]) — rather than unwrapping one out of
//! an [`Interface`], whose transport is private and, for the tunnel, insufficient
//! (`Interface` carries no `send_unit`/`set_tunnel_sink`). The `Interface` is
//! built *from* that handle for the two things only it needs it for: the Mcu's
//! connection and the port-level admin commands.
//!
//! The run is the sequence `docs/work-log/2026-10-16-katapult-flash-task.md`
//! spells out, in the order the bridge firmware requires:
//!
//! 1. **open + identify** ([`BridgeLink::open_host_library`] /
//!    [`BridgeLink::open_serial`], then [`BridgeLink::identify`]) — the
//!    declaration commands are klipper messages, so the dictionary has to come
//!    from the running bridge before anything of them can be encoded.
//! 2. **open_bridge** — the target's port is declared, its device reset, and the
//!    tunnel opened ([`BridgeLink::flash`]).
//! 3. **the session** — the bridge's port carries a katapult byte stream, the
//!    [`TunnelBridge`] adapter wraps the session's frames into tunnel units, and
//!    [`session::flash_with_config`] runs connect → send_block × N → eof →
//!    verify → complete over it.
//! 4. **clear the tunnel** — a repeat `SET_PORT_TUNNEL`, because the toggle is
//!    the bridge's own state and the bridge outlives this run. A port left in
//!    tunnel mode carries raw bytes rather than mux units, which is the wrong
//!    shape for anything but a bootloader handshake; the target has been handed
//!    back to its application by `complete`, so no reset is needed to finish.
//!
//! The module knows the bridge only through the link handle and the plan its
//! caller gives it: which port the target sits on, that device's id, and the
//! reset each declared port's device takes are the caller's to say
//! ([`BridgePort`], [`BridgeTarget`]), exactly as [`open_bridge`] documents.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use crate::core::klippy::assign;
use crate::core::klippy::interface::devices::host::HostDevice;
use crate::core::klippy::interface::devices::serial::{self, SharedSerialHandle, TunnelSink};
use crate::core::klippy::interface::error::InterfaceError;
use crate::core::klippy::interface::{HostMux, Interface};
use crate::core::klippy::mcu::{Mcu, McuError};

use super::bridge::{TunnelBridge, TunnelLink};
use super::bridge_session::{open_bridge, BridgeError, BridgePort, BridgeTarget};
use super::session::{self, FlashError, SessionConfig, Summary};
use std::fmt;

/// The id the bridge's own connection is attached as on a shared serial link.
///
/// A bridge is the one MCU the flash host is talking to, so the first id of the
/// wire's range names it; the assigned range is host-chosen, and nothing else is
/// on the link for the run to confuse it with.
pub const BRIDGE_MCU_ID: u32 = 1;

/// How long opening a **shared serial** bridge waits for its assignment
/// handshake, per attempt — the serial layer's own `ASSIGN_ATTACH_*` budget.
const SERIAL_ATTACH_TIMEOUT: Duration = Duration::from_millis(500);

/// How long the bridge's identify handshake may take before a run gives up.
///
/// The declaration commands cannot be encoded before it, so a bridge that
/// never answers is a flash that never starts; ten seconds is the same budget
/// the bridge-session tests give their real-library fixture.
pub const IDENTIFY_TIMEOUT: Duration = Duration::from_secs(10);

/// The bridge device the run holds directly, one variant per link it can be
/// opened on: the host library the config calls muxed, or a shared serial port.
///
/// Both speak the same port-level commands and the same tunnel `send_unit`, so
/// the flash half never matches on which one it has beyond building the two
/// handles ([`BridgeDevice::tunnel_link`], [`BridgeDevice::set_tunnel_sink`]).
enum BridgeDevice {
    /// A muxed host-library reference (`HostDevice::load_mux`) — the bridge is
    /// an in-process klipper library, and `host_port_paths` named its ports.
    Host(Arc<HostDevice>),
    /// A handle on a shared serial port (`serial::attach`) — the bridge is a
    /// board on a tty, addressed by id because its downstream devices share the
    /// wire.
    Shared(Arc<SharedSerialHandle>),
}

impl BridgeDevice {
    /// The transport half of the tunnel: the link a [`TunnelBridge`] writes
    /// whole katapult blocks through.
    fn tunnel_link(&self) -> Arc<dyn TunnelLink> {
        match self {
            BridgeDevice::Host(device) => Arc::clone(device) as Arc<dyn TunnelLink>,
            BridgeDevice::Shared(handle) => Arc::clone(handle) as Arc<dyn TunnelLink>,
        }
    }

    /// Route the port's uplink tunnel units (headed with `arb`) to `sink`.
    fn set_tunnel_sink(&self, arb: u32, sink: TunnelSink) {
        match self {
            BridgeDevice::Host(device) => device.set_tunnel_sink(arb, sink),
            BridgeDevice::Shared(handle) => handle.set_tunnel_sink(arb, sink),
        }
    }
}

/// A bridge the flash host has open, with the `Mcu` over it and the two faces
/// the run needs: the [`Interface`] for the port-level commands, and the device
/// itself for the tunnel's downlink and the sink registration.
///
/// Built by [`open_host_library`](BridgeLink::open_host_library) or
/// [`open_serial`](BridgeLink::open_serial), then identified
/// ([`identify`](BridgeLink::identify)), then flashed
/// ([`flash`](BridgeLink::flash)). Both constructors take their device's
/// blocking open, so they run inside a runtime — the `Interface` they build
/// captures the runtime its `Mcu` transport tasks will use.
pub struct BridgeLink {
    mcu: Mcu,
    link: Interface,
    device: BridgeDevice,
}

impl BridgeLink {
    /// Open a **host-library** bridge at `path`: a muxed reference
    /// (`HostDevice::load_mux`), addressed by `mcu_id`, whose ports open the
    /// ttys `host_port_paths` names as `(slot, path)`.
    ///
    /// `mux` is what the library is given and named
    /// (`HostMux { identity, named }`, `mcu/host_libraries.rs`): the config
    /// layer's judgment for the path, since the identity is a pure function of
    /// the path string and a flash tool has no config to re-derive it from.
    ///
    /// # Errors
    /// [`BridgeFlashError::Open`] when the library cannot be loaded, the
    /// reference cannot be attached, or `mcu_id` is outside the addressable
    /// range.
    pub fn open_host_library(
        path: &Path,
        mux: HostMux,
        mcu_id: u32,
        host_port_paths: &[(u8, String)],
    ) -> Result<Self, BridgeFlashError> {
        let device = Arc::new(
            HostDevice::load_mux(path, mux, mcu_id, host_port_paths)
                .map_err(BridgeFlashError::Open)?,
        );
        let link = Interface::from_host(Arc::clone(&device));
        Ok(Self::assemble(link.clone(), BridgeDevice::Host(device)))
    }

    /// Open a **shared serial** bridge on `path` at `baud`, attached as
    /// `mcu_id`.
    ///
    /// A bridge's link carries its downstream devices' frames too, so its port
    /// is opened shared and addressed by id (`serial::attach`), which also runs
    /// the assignment handshake. `uuid` is the bridge **board's** own identity,
    /// for a bridge that carries no node id yet: the assignment goes through
    /// `SET_NODEID`, and only that segment names an unassigned board — the
    /// plain re-confirmation a `None` uuid sends is answered by assigned
    /// devices alone, so an unassigned bridge times out instead (see
    /// [`BridgeFlashError::UnassignedSerial`]). A bridge that already carries
    /// an id needs no uuid: the re-confirmation is the ordinary handshake.
    ///
    /// # Errors
    /// [`BridgeFlashError::UnassignedSerial`] when the handshake timed out with
    /// no uuid given — the one failure that has an actionable answer — and
    /// [`BridgeFlashError::Open`] when the port cannot be opened, `mcu_id` is
    /// outside the addressable range, or the handshake times out or is refused
    /// with a uuid given.
    pub fn open_serial(
        path: &Path,
        baud: u32,
        mcu_id: u32,
        uuid: Option<[u8; 6]>,
    ) -> Result<Self, BridgeFlashError> {
        let handle = serial::attach(path, baud, mcu_id, SERIAL_ATTACH_TIMEOUT, uuid).map_err(
            |e| match (uuid, e) {
                (None, InterfaceError::Timeout) => BridgeFlashError::UnassignedSerial {
                    path: path.to_path_buf(),
                    baud,
                },
                (_, e) => BridgeFlashError::Open(e),
            },
        )?;
        let link = Interface::shared_serial(Arc::clone(&handle));
        Ok(Self::assemble(link.clone(), BridgeDevice::Shared(handle)))
    }

    /// Build the `Mcu` over the link the constructor made, with the interface
    /// kept for the port-level commands.
    fn assemble(link: Interface, device: BridgeDevice) -> Self {
        let mcu = Mcu::new("bridge", link.clone());
        Self { mcu, link, device }
    }

    /// Run the bridge's identify handshake: the dictionary the declaration
    /// commands are encoded from comes from the running firmware, so nothing of
    /// them can be sent before this returns.
    ///
    /// # Errors
    /// [`BridgeFlashError::Identify`] when the handshake fails or times out.
    pub async fn identify(&self, timeout: Duration) -> Result<(), BridgeFlashError> {
        self.mcu
            .identify(timeout)
            .await
            .map(|_| ())
            .map_err(BridgeFlashError::Identify)
    }

    /// The whole bridge flash: open the target's port
    /// ([`open_bridge`]), run the katapult session over its tunnel, then clear
    /// the tunnel.
    ///
    /// `ports` and `target` are the plan [`open_bridge`] takes — the port order
    /// is the bridge's numbering, and only a **legacy** route (no port flags)
    /// serves the raw uplink the tunnel carries, so [`open_bridge`] names the
    /// target that one way. `progress` is [`session::flash_with_config`]'s.
    ///
    /// The tunnel is cleared on every path, success or failure, because the
    /// bridge outlives this run; a clearing that fails after a successful flash
    /// is reported as [`BridgeFlashError::Teardown`], and a failed flash keeps
    /// its own error as the primary one.
    ///
    /// # Errors
    /// [`BridgeFlashError::Unaddressable`] when the target's node id names no
    /// arb, [`BridgeFlashError::Bridge`] when [`open_bridge`] fails,
    /// [`BridgeFlashError::Session`] when the katapult session fails, and
    /// [`BridgeFlashError::Teardown`] when only the tunnel clearing fails.
    pub async fn flash(
        &self,
        ports: &[BridgePort],
        target: BridgeTarget,
        payload: &[u8],
        cfg: &SessionConfig,
        progress: impl FnMut(&str),
    ) -> Result<Summary, BridgeFlashError> {
        let node_id = u32::from(target.node_id);
        let down_arb = assign::arb_id(node_id).ok_or(BridgeFlashError::Unaddressable {
            node_id: target.node_id,
        })?;
        let reply_arb = assign::reply_arb_id(node_id).expect("arb_id accepted the id");

        open_bridge(&self.mcu, &self.link, ports, target)
            .await
            .map_err(BridgeFlashError::Bridge)?;

        // The uplink listener is registered before the session's first block,
        // and the adapter below is what a session drives. The receiver lives in
        // the `TunnelBridge`, so dropping it below is also what unregisters the
        // sink.
        let (sink, frames) = TunnelSink::channel();
        self.device.set_tunnel_sink(reply_arb, sink);
        let bridge = TunnelBridge::new(down_arb, frames, self.device.tunnel_link());
        let (mut reader, mut writer) = bridge.split();
        let outcome = session::flash_with_config(&mut reader, &mut writer, payload, cfg, progress);
        drop(reader);
        drop(writer);
        drop(bridge);

        // The bridge stays up after the host exits, so the port must not be
        // left carrying the katapult raw stream: a repeat toggle flips it back
        // to the mux units its other users expect.
        let cleared = self.link.send_set_port_tunnel(target.index);
        match (outcome, cleared) {
            (Ok(summary), Ok(())) => Ok(summary),
            (Ok(_), Err(e)) => Err(BridgeFlashError::Teardown(e)),
            (Err(e), _) => Err(BridgeFlashError::Session(e)),
        }
    }

    /// Shut the bridge's connection down explicitly, before the handles go.
    ///
    /// The same step [`Mcu::close`] is elsewhere: the transport tasks hold
    /// `Interface` clones (and so the link's resources) until they are aborted
    /// and their blocking reads released, which dropping the `Mcu` alone does
    /// not force.
    pub fn close(&self) {
        self.mcu.close();
    }
}

/// Everything that can stop a bridge flash.
#[derive(Debug)]
pub enum BridgeFlashError {
    /// The bridge's device could not be opened or attached.
    Open(InterfaceError),
    /// A **serial** bridge never answered the assignment handshake and no
    /// `--bridge-uuid` named it: without an identity the host can only send the
    /// re-confirmation request, which an unassigned board does not answer (see
    /// [`BridgeLink::open_serial`]) — so a silent line is unassigned or absent,
    /// and only the caller can tell.
    UnassignedSerial {
        /// The tty the handshake went out on.
        path: PathBuf,
        /// The line speed it opened at.
        baud: u32,
    },
    /// The identify handshake failed — no dictionary, so no declaration.
    Identify(McuError),
    /// [`open_bridge`] refused or failed a step.
    Bridge(BridgeError),
    /// The katapult session over the tunnel failed.
    Session(FlashError),
    /// The target's node id names no arb (only `1..=255` are addressable).
    Unaddressable {
        /// The id the plan named.
        node_id: u8,
    },
    /// The tunnel could not be cleared after a successful session.
    Teardown(InterfaceError),
}

impl fmt::Display for BridgeFlashError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BridgeFlashError::Open(e) => write!(f, "opening the bridge: {e}"),
            BridgeFlashError::UnassignedSerial { path, baud } => write!(
                f,
                "opening the bridge: {} at {baud} baud did not answer; if the bridge has no \
                 node id yet it needs --bridge-uuid (take it from `klipperx scan --port {}`)",
                path.display(),
                path.display()
            ),
            BridgeFlashError::Identify(e) => write!(f, "identifying the bridge: {e}"),
            BridgeFlashError::Bridge(e) => write!(f, "opening the bridge port: {e}"),
            BridgeFlashError::Session(e) => write!(f, "flashing: {e}"),
            BridgeFlashError::Unaddressable { node_id } => write!(
                f,
                "target node id {node_id} names no arb (only 1..=255 are addressable)"
            ),
            BridgeFlashError::Teardown(e) => write!(f, "clearing the flashing tunnel: {e}"),
        }
    }
}

impl std::error::Error for BridgeFlashError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            BridgeFlashError::Open(e) | BridgeFlashError::Teardown(e) => Some(e),
            BridgeFlashError::Identify(e) => Some(e),
            BridgeFlashError::Bridge(e) => Some(e),
            BridgeFlashError::Session(e) => Some(e),
            BridgeFlashError::Unaddressable { .. } | BridgeFlashError::UnassignedSerial { .. } => {
                None
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
    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::{Config, ConfigWrapper};
    use crate::core::klippy::interface::devices::host::{HostDevice, HOST_LIBRARY_TEST_LOCK};
    use crate::core::klippy::interface::pty::Pty;
    use crate::core::klippy::interface::HostMux;
    use crate::core::klippy::mcu::host_libraries::HostLibraries;
    use crate::flash::bridge_session::{BridgePort, BridgeTarget, PortReset};
    use crate::flash::fake::FakeBootloader;
    use crate::flash::frame::{build_frame, command};
    use crate::flash::session::SessionConfig;
    use std::io::{Read, Write};
    use std::os::fd::AsRawFd;
    use std::path::Path;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Arc, Mutex};

    /// The node id the target's device answers to behind its port — a small,
    /// addressable id in the assigned range.
    const TARGET_NODE_ID: u8 = 7;

    /// Session timing for the test: same shape as the defaults, millisecond
    /// scale, so the run is fast but every wait is bounded.
    fn test_config() -> SessionConfig {
        SessionConfig {
            tries: 3,
            read_timeout: Duration::from_secs(2),
            retry_delay: Duration::from_millis(50),
            busy_delay: Duration::from_millis(50),
        }
    }

    /// The exact bytes the session writes for `payload` at `start_addr`: every
    /// command frame, in order, as [`frame`](crate::flash::frame) builds them —
    /// blocks padded to `block_size` with `0xFF` like the session's.
    fn session_frames(start_addr: u32, payload: &[u8], block_size: usize) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&build_frame(command::CONNECT, &[]));
        for (i, chunk) in payload.chunks(block_size).enumerate() {
            let mut block = chunk.to_vec();
            block.resize(block_size, 0xFF);
            let mut send = (start_addr + (i * block_size) as u32)
                .to_le_bytes()
                .to_vec();
            send.extend_from_slice(&block);
            out.extend_from_slice(&build_frame(command::SEND_BLOCK, &send));
        }
        out.extend_from_slice(&build_frame(command::EOF, &[]));
        for i in 0..payload.chunks(block_size).count() {
            let addr = start_addr + (i * block_size) as u32;
            out.extend_from_slice(&build_frame(command::REQUEST_BLOCK, &addr.to_le_bytes()));
        }
        out.extend_from_slice(&build_frame(command::COMPLETE, &[]));
        out
    }

    /// Pump the pty's far end through the fake bootloader: the bridge's frames
    /// (read on the master) go into the fake and are recorded, the fake's
    /// answers go back out the master to the bridge. Run until `stop` is set.
    fn start_fake_pump(
        pty: &Pty,
        fake: &FakeBootloader,
        recorded: Arc<Mutex<Vec<u8>>>,
        stop: Arc<AtomicBool>,
    ) -> std::thread::JoinHandle<()> {
        let mut fake_reader = fake.reader();
        let mut fake_writer = fake.writer();
        let mut master_write = pty.master.try_clone().expect("the pty master clones");
        let master_read = pty.master.try_clone().expect("the pty master clones");
        std::thread::spawn(move || {
            let mut from_bootloader = [0u8; 256];
            let mut from_bridge = [0u8; 256];
            let mut reader = &master_read;
            while !stop.load(Ordering::Relaxed) {
                match fake_reader.read(&mut from_bootloader) {
                    Ok(n) if n > 0 => master_write
                        .write_all(&from_bootloader[..n])
                        .expect("the pty takes the bootloader's answer"),
                    Ok(_) => {}
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                    Err(e) => panic!("the fake bootloader failed: {e}"),
                }
                let mut poll = libc::pollfd {
                    fd: master_read.as_raw_fd(),
                    events: libc::POLLIN,
                    revents: 0,
                };
                let ready = unsafe { libc::poll(&mut poll, 1, 5) };
                if ready > 0 {
                    match reader.read(&mut from_bridge) {
                        Ok(0) => {}
                        Ok(n) => {
                            recorded
                                .lock()
                                .unwrap()
                                .extend_from_slice(&from_bridge[..n]);
                            fake_writer
                                .write_all(&from_bridge[..n])
                                .expect("the fake bootloader takes the frame");
                        }
                        Err(e) => panic!("reading the pty master: {e}"),
                    }
                }
            }
        })
    }

    /// Block (bounded) until the process-wide library on `path` is unloaded,
    /// then return — and **fail here** rather than let the next library test
    /// meet a live image. Same probe as the bridge-session test:
    /// [`HostDevice::load_bytes`] fails while the mux library is live and
    /// succeeds (dropping the probe unloads it again) once it is gone.
    fn wait_for_library_unload(path: &Path) {
        for _ in 0..200 {
            match HostDevice::load_bytes(path, Some([0x5a; 6])) {
                Ok(probe) => {
                    drop(probe);
                    return;
                }
                Err(_) => std::thread::sleep(Duration::from_millis(10)),
            }
        }
        panic!(
            "the mux host library at {} was still loaded after 2s; the next \
             library test would meet a live image",
            path.display()
        );
    }

    /// The whole bridge flash over a **real** host library: the mux library as
    /// the bridge, one pty as its port 0, and [`FakeBootloader`] as the
    /// bootloader behind that port.
    ///
    /// The bridge's connection is built the way the tool builds one — the
    /// library reference loaded muxed, the `Mcu` over it, and an identify that
    /// fetches the dictionary (no dictionary file: this run's whole point). The
    /// session then runs over the tunnel, and the pty's bytes are the session's
    /// katapult frames exactly.
    ///
    /// `multi_thread` is not incidental: the connection's transport tasks hold
    /// `Interface` clones (and so the library) until their aborts land, and the
    /// unload wait below blocks the test thread — on the default current-thread
    /// runtime those aborts would only be processed after the wait, leaving the
    /// library live for the next test (see the bridge-session test's note).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_flash_via_bridge_over_the_real_library() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // The bridge's port 0 opens at configuration time, so its tty has to
        // exist before the library comes up — and it is named to the library as
        // the port table's slot 0.
        let port = Pty::new();
        let library = klipperx_test_support::klipper_host_mux_lib_path();
        let path = library.display().to_string();

        // The config layer's judgment for the path is what the library is given
        // and named with (`HostLibraries`, the same scan `McuConfig::open`
        // consumes): one `[mcu]` on the path, so the identity names its device.
        let text = format!("[mcu]\nhost_library: {path}\n");
        let (config, _) = Config::from_text(&text).expect("the config parses");
        let section = config
            .sections()
            .find(|section| section.id == "mcu")
            .expect("the config has an [mcu] section");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        let libraries = HostLibraries::classify(&wrapper).expect("the path is told apart");
        let identity = libraries.identity(&path).expect("the path has an identity");
        let mux = HostMux {
            identity,
            named: identity,
        };

        let link = BridgeLink::open_host_library(
            Path::new(&path),
            mux,
            BRIDGE_MCU_ID,
            &[(0u8, port.path.display().to_string())],
        )
        .expect("the muxed library opens and attaches");

        tokio::time::timeout(Duration::from_secs(20), link.identify(IDENTIFY_TIMEOUT))
            .await
            .expect("the identify handshake timed out")
            .expect("the bridge identifies");

        // The fake bootloader on the pty's far end, pumped by a thread. The
        // bootloader's blocks come back up the bridge's tunnel to the session's
        // sink; the frames the bridge writes to the pty are recorded.
        let fake = FakeBootloader::new();
        let recorded = Arc::new(Mutex::new(Vec::<u8>::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let pump = start_fake_pump(&port, &fake, Arc::clone(&recorded), Arc::clone(&stop));

        let ports = [BridgePort {
            baud: 250_000,
            reset: PortReset::ResetPort,
        }];
        let target = BridgeTarget {
            index: 0,
            node_id: TARGET_NODE_ID,
        };
        let payload: Vec<u8> = (0..100u8).collect();
        let summary = link
            .flash(&ports, target, &payload, &test_config(), |_| {})
            .await
            .expect("the bridge flash completes");

        assert_eq!(summary.start_addr, 0x0800_2000, "the fake's start address");
        assert_eq!(summary.block_size, 64);
        assert_eq!(summary.mcu_type, "STM32F103");
        assert_eq!(summary.bytes_flashed, payload.len());
        assert_eq!(summary.block_count, 2, "100 bytes is two 64-byte blocks");
        assert_eq!(fake.seen(command::CONNECT), 1);
        assert_eq!(fake.seen(command::SEND_BLOCK), 2, "both blocks written");
        assert_eq!(fake.seen(command::REQUEST_BLOCK), 2, "both blocks verified");
        assert_eq!(fake.seen(command::COMPLETE), 1);

        stop.store(true, Ordering::Relaxed);
        pump.join().expect("the pump thread");

        // The downlink: the pty's bytes are the session's katapult frames,
        // exactly and in order — no unit header anywhere, and each 76-byte Send
        // Block travels whole.
        let expected = session_frames(0x0800_2000, &payload, 64);
        assert_eq!(
            *recorded.lock().unwrap(),
            expected,
            "the pty carried bare katapult frames, byte for byte"
        );
        let mut send = 0x0800_2000u32.to_le_bytes().to_vec();
        let mut block = payload[..64].to_vec();
        block.resize(64, 0xFF);
        send.extend_from_slice(&block);
        let send_frame = build_frame(command::SEND_BLOCK, &send);
        assert_eq!(send_frame.len(), 76, "a 64 B Send Block is one 76 B frame");
        assert!(
            recorded
                .lock()
                .unwrap()
                .windows(send_frame.len())
                .any(|window| window == send_frame.as_slice()),
            "the 76-byte Send Block reaches the pty whole"
        );

        // The connection's transport tasks hold references to the library, so
        // its drop is not synchronous with `drop(link)`: close it, let the
        // blocked device read see the shutdown, then wait the library out — all
        // under `_serial`, whose scope is the whole test.
        link.close();
        drop(link);
        wait_for_library_unload(&library);
    }

    /// A serial bridge that never answers is reported as an **unassigned**
    /// link, naming the flag that fixes it: the bare serial layer's
    /// "Receive timeout" leaves the caller with nothing to act on.
    ///
    /// The pty is the silent bridge: nothing on its far end answers the
    /// handshake, which is exactly what an unassigned board does — an assigned
    /// one would answer the re-confirmation `uuid: None` sends.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_a_silent_serial_bridge_is_reported_as_unassigned() {
        let pty = Pty::new();
        let err = match BridgeLink::open_serial(&pty.path, 250_000, BRIDGE_MCU_ID, None) {
            Ok(_) => panic!("nothing answers on this pty, so the attach has to fail"),
            Err(e) => e,
        };
        let text = err.to_string();
        assert!(
            matches!(
                err,
                BridgeFlashError::UnassignedSerial { baud: 250_000, .. }
            ),
            "{text}"
        );
        assert!(text.contains("--bridge-uuid"), "{text}");
        assert!(text.contains("klipperx scan --port"), "{text}");
    }

    /// A `Read` that records the bytes it hands through: the tunnel's uplink
    /// half delivers whole katapult blocks, so this is where the test sees that
    /// the bridge's own console port carried them.
    struct RecordingReader<R> {
        inner: R,
        seen: Arc<Mutex<Vec<u8>>>,
    }

    impl<R: Read> Read for RecordingReader<R> {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            let n = self.inner.read(buf)?;
            self.seen.lock().unwrap().extend_from_slice(&buf[..n]);
            Ok(n)
        }
    }

    /// The whole bridge flash through a **real linux firmware process** as the
    /// bridge, with that process's **own arch backend** as the port: the
    /// firmware (`test/configs/linuxprocess-bridge.config`,
    /// `src/linux/bridge.c`) opens a pty as its single downstream port and
    /// [`FakeBootloader`] sits behind it.
    ///
    /// The host half is the flash run's own pieces, because the bridge is a
    /// process on a tty and not a library reference: `serial::attach` over the
    /// bridge's console pty (named with the identity it was started with, the
    /// `SET_NODEID` an unassigned mux device adopts an id by), the `Mcu` over
    /// it, then [`open_bridge`], the [`TunnelBridge`] and the katapult session —
    /// the same path `BridgeLink::flash` walks for a board on a shared serial
    /// port.
    ///
    /// What it shows, in the two directions a bridge has: the port's pty carried
    /// the session's katapult frames as bare frames (the tunnel's downlink), the
    /// console carried the bootloader's answers back as tunnel units (the
    /// uplink, seen here through the sink the port's reply arb routes to), and
    /// the session finished with the bootloader's own identity. Every wait is
    /// bounded and the process is killed on the way out.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_flash_via_a_real_linux_firmware_bridge_over_its_port_backend() {
        // The identity the process is started with, and the one the host names
        // it with: the bytes have to be the same or the firmware never adopts
        // its node id and the attach times out instead of failing clearly.
        const BRIDGE_UUID: [u8; 6] = [0x51, 0x52, 0x53, 0x54, 0x55, 0x56];

        // The downstream port: a pty of the test's own. The firmware's backend
        // opens it when the host declares the port (`config_uart`), and the test
        // drives the far end with the fake bootloader below.
        let port = Pty::new();
        let firmware =
            klipperx_test_support::LinuxFirmware::spawn_bridge_port(&port.path, BRIDGE_UUID)
                .expect("the bridge firmware process starts");

        // The upstream: the bridge's console pty, attached shared and given the
        // identity the process was started with — an unassigned mux device
        // answers nothing until a `SET_NODEID` names it.
        let handle = serial::attach(
            firmware.serial_path(),
            250_000,
            BRIDGE_MCU_ID,
            Duration::from_millis(500),
            Some(BRIDGE_UUID),
        )
        .expect("the bridge's console attaches and is named");
        let link = Interface::shared_serial(Arc::clone(&handle));
        let mcu = Mcu::new("bridge", link.clone());
        tokio::time::timeout(Duration::from_secs(20), mcu.identify(IDENTIFY_TIMEOUT))
            .await
            .expect("the identify handshake timed out")
            .expect("the linux bridge identifies");

        // The bootloader behind the port, pumped by a thread: the bridge's bare
        // frames go into the fake (recorded), its answers go back out to the
        // bridge.
        let fake = FakeBootloader::new();
        let recorded = Arc::new(Mutex::new(Vec::<u8>::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let pump = start_fake_pump(&port, &fake, Arc::clone(&recorded), Arc::clone(&stop));

        let ports = [BridgePort {
            baud: 250_000,
            reset: PortReset::ResetPort,
        }];
        let target = BridgeTarget {
            index: 0,
            node_id: TARGET_NODE_ID,
        };
        open_bridge(&mcu, &link, &ports, target)
            .await
            .expect("the linux bridge declares its port, resets it, and opens the tunnel");

        // The tunnel: the port's reply arb is where the bridge heads the frames
        // it reads from the port, and the uplink reader below is what the
        // session drives.
        let node_id = u32::from(target.node_id);
        let down_arb = assign::arb_id(node_id).expect("the target id is addressable");
        let reply_arb = assign::reply_arb_id(node_id).expect("the target id is addressable");
        let (sink, frames) = TunnelSink::channel();
        handle.set_tunnel_sink(reply_arb, sink);
        let bridge = TunnelBridge::new(down_arb, frames, Arc::clone(&handle));
        let (reader, mut writer) = bridge.split();
        let uplink = Arc::new(Mutex::new(Vec::<u8>::new()));
        let mut reader = RecordingReader {
            inner: reader,
            seen: Arc::clone(&uplink),
        };

        let payload: Vec<u8> = (0..100u8).collect();
        let outcome =
            session::flash_with_config(&mut reader, &mut writer, &payload, &test_config(), |_| {});
        // The bridge's own process is where a port that never opened, or one
        // that shut the firmware down, says so: report it with the failure.
        let stderr = firmware.stderr();
        let summary = outcome.unwrap_or_else(|e| {
            panic!(
                "the katapult session completes over the port's tunnel: {e}\n\
                 the port's pty carried {} bytes, the bootloader saw {} CONNECT \
                 frame(s) and served {} answer(s)\nfirmware stderr:\n{stderr}",
                recorded.lock().unwrap().len(),
                fake.seen(command::CONNECT),
                fake.responses_served(),
            )
        });

        // The session's own verdict, taken from the bootloader's answers: these
        // are values only the fake behind the port has.
        assert_eq!(summary.start_addr, 0x0800_2000, "the fake's start address");
        assert_eq!(summary.block_size, 64);
        assert_eq!(summary.mcu_type, "STM32F103");
        assert_eq!(summary.bytes_flashed, payload.len());
        assert_eq!(summary.block_count, 2, "100 bytes is two 64-byte blocks");
        assert_eq!(fake.seen(command::CONNECT), 1);
        assert_eq!(fake.seen(command::SEND_BLOCK), 2, "both blocks written");
        assert_eq!(fake.seen(command::REQUEST_BLOCK), 2, "both blocks verified");
        assert_eq!(fake.seen(command::COMPLETE), 1);

        // The bridge's own arch backend is where the port came from, and it says
        // so on its stderr when the host's declaration opened the tty.
        assert!(
            stderr.contains("linux bridge serial:") && stderr.contains("slot 0"),
            "the firmware's own port backend opened the tty: {stderr}"
        );

        // The tunnel is the bridge's own state and the bridge outlives the run:
        // a repeat toggle flips it back to the mux units its other users expect.
        link.send_set_port_tunnel(0).expect("the tunnel clears");

        stop.store(true, Ordering::Relaxed);
        pump.join().expect("the pump thread");

        // The downlink: the port's pty carried the session's katapult frames as
        // bare frames — no unit header anywhere — byte for byte, and the 76-byte
        // Send Block travelled whole.
        let expected = session_frames(0x0800_2000, &payload, 64);
        assert_eq!(
            *recorded.lock().unwrap(),
            expected,
            "the port's pty carried bare katapult frames, byte for byte"
        );
        let mut send = 0x0800_2000u32.to_le_bytes().to_vec();
        let mut block = payload[..64].to_vec();
        block.resize(64, 0xFF);
        send.extend_from_slice(&block);
        let send_frame = build_frame(command::SEND_BLOCK, &send);
        assert_eq!(send_frame.len(), 76, "a 64 B Send Block is one 76 B frame");
        assert!(
            recorded
                .lock()
                .unwrap()
                .windows(send_frame.len())
                .any(|window| window == send_frame.as_slice()),
            "the 76-byte Send Block reaches the port's pty whole"
        );

        // The uplink: the bridge's console carried the bootloader's answers back
        // as tunnel units — the sink is registered on the port's reply arb and
        // receives units headed with it, so what the session read here can only
        // have come over the tunnel — one whole katapult block each.
        let uplink = uplink.lock().unwrap().clone();
        assert!(
            !uplink.is_empty(),
            "the uplink delivered the bootloader's answers"
        );
        assert!(
            uplink.windows(9).any(|w| w == b"STM32F103"),
            "the bootloader's own identity came back over the tunnel"
        );
        let mut pos = 0usize;
        let mut frames = 0usize;
        while pos < uplink.len() {
            assert_eq!(uplink[pos], 0x01, "every uplink block is a katapult frame");
            let length = serial::tunnel_frame_len(uplink[pos + serial::TUNNEL_POS_WORDCNT]);
            assert!(
                (serial::TUNNEL_FRAME_MIN..=serial::TUNNEL_FRAME_MAX).contains(&length),
                "the block's word count is inside the katapult frame bounds"
            );
            assert!(
                pos + length <= uplink.len(),
                "a whole block arrived, never a slice of one"
            );
            pos += length;
            frames += 1;
        }
        assert_eq!(
            frames,
            fake.responses_served(),
            "one tunnel unit per answer the bootloader gave"
        );

        // The connection's transport tasks hold the link, so close it before the
        // fixture takes the process (and its pty) down.
        mcu.close();
        drop(mcu);
        drop(link);
        drop(handle);
        drop(firmware);
    }
}
