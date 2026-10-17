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
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::core::klippy::assign;
use crate::core::klippy::event::{McuEvent, Shutdown};
use crate::core::klippy::interface::devices::host::HostDevice;
use crate::core::klippy::interface::devices::serial::{self, SharedSerialHandle, TunnelSink};
use crate::core::klippy::interface::error::InterfaceError;
use crate::core::klippy::interface::{HostMux, Interface};
use crate::core::klippy::mcu::{Dictionary, Mcu, McuError};

use super::bridge::{TunnelBridge, TunnelLink};
use super::bridge_session::{
    declare_port as send_declare_port, declare_ports as send_declare_ports, open_bridge,
    open_target_port as send_open_target_port, reset_port as send_port_reset, BridgeError,
    BridgePort, BridgeTarget, PortReset, PortRoute,
};
use super::frame::command::CONNECT;
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

/// The index a `RESTART_DEVICE` carries to reach the **bridge itself**
/// ([`BridgeLink::reset`]).
///
/// A bridge's port routing consumes `RESTART_DEVICE [0x05][index]` only when
/// `index` names a port it has in use; any other index falls through to its own
/// device half, which restarts the firmware (`third_party/klipper/src/bridge.c`,
/// `mux.c` → `mcu_restart`). No bridge numbers a port this high
/// (`BRIDGE_MAX_PORTS` is 8), so this index always means the bridge.
const RESTART_DEVICE_TO_BRIDGE: u8 = 0xFF;

/// How long a restarted bridge may take to answer again before a run gives up.
const RESET_TIMEOUT: Duration = Duration::from_secs(10);

/// The identify budget of one reopen attempt while a restarted bridge is coming
/// back up; a boot that is not finished yet answers nothing, and the attempt is
/// retried until [`RESET_TIMEOUT`] is spent.
const RESET_IDENTIFY_TIMEOUT: Duration = Duration::from_secs(2);

/// How long to wait between reopen attempts.
const RESET_RETRY_DELAY: Duration = Duration::from_millis(100);

/// How long the discovery broadcast a run sends when it has **no**
/// `--bridge-uuid` waits for an answer, after the restart dropped the bridge's
/// node id.
///
/// The scan is the one `klipperx admin scan --port` uses; it runs to the end of this
/// window even after a board answers, so the budget is the price of not knowing
/// the identity up front. Long enough for a board to finish coming back up,
/// short enough that [`RESET_TIMEOUT`] still has room to retry.
const DISCOVER_WINDOW: Duration = Duration::from_secs(2);

/// The USARTs the stm32 bridge backend can put a port on, in the order it
/// numbers the slots, each with the name it reserves that USART's pins under.
///
/// `third_party/klipper/src/stm32/bridge_serial.c` numbers the slots by USART
/// number — USART1, USART2, USART3, UART4, UART5 — and USART1 keeps the
/// constant name the single-port backend always used
/// (`RESERVE_PINS_bridge_serial`), while the others are named after their own
/// USART (`RESERVE_PINS_bridge_serial_usart2`, …). A ticked USART is one whose
/// reserve constant the dictionary carries; so this list is the backend's own
/// slot order, not a guess from `MCU`.
const STM32_BRIDGE_SERIAL_UARTS: [(&str, &str); 5] = [
    ("USART1", "bridge_serial"),
    ("USART2", "bridge_serial_usart2"),
    ("USART3", "bridge_serial_usart3"),
    ("UART4", "bridge_serial_uart4"),
    ("UART5", "bridge_serial_uart5"),
];

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

/// How a [`BridgeLink`]'s transport was opened, kept so a run can open a fresh
/// one after the bridge's firmware restarts.
///
/// A bridge's port table lives in its firmware's RAM only, and a restart
/// re-creates the console it answers on, which kills the host's old handle: the
/// reopen therefore needs the same parameters the first open took.
#[derive(Clone)]
enum BridgeOpen {
    /// A muxed host-library reference.
    Host {
        path: PathBuf,
        mux: HostMux,
        mcu_id: u32,
        host_port_paths: Vec<(u8, String)>,
    },
    /// A shared serial port on a bridge board (or a linux firmware process).
    Serial {
        path: PathBuf,
        baud: u32,
        mcu_id: u32,
        uuid: Option<[u8; 6]>,
    },
}

/// Attach a **shared serial** bridge at `path`, naming it with `uuid` — or,
/// when `uuid` is `None`, finding it first.
///
/// A bridge's node id lives in its firmware's RAM, so a run with no
/// `--bridge-uuid` has to **find** the bridge. An unassigned one answers only
/// the discovery broadcast (the query `klipperx admin scan --port` makes); an assigned
/// one stays quiet on that broadcast and answers the plain re-confirmation
/// instead. So this broadcasts first and falls back to the re-confirmation: the
/// flag is an optional override, never a requirement, for both a first open and
/// a reopen after a restart.
///
/// # Errors
/// [`BridgeFlashError::AmbiguousBridge`] when more than one unassigned bridge
/// answers — naming the wrong one would restart and re-assign the wrong board —
/// [`BridgeFlashError::UnassignedSerial`] when neither the broadcast nor the
/// re-confirmation reaches a bridge, and [`BridgeFlashError::Open`] when the tty
/// cannot be opened.
fn attach_bridge(
    path: &Path,
    baud: u32,
    mcu_id: u32,
    uuid: Option<[u8; 6]>,
) -> Result<Arc<SharedSerialHandle>, BridgeFlashError> {
    let uuid = match uuid {
        Some(uuid) => uuid,
        None => match discover_bridge_uuid(path, baud) {
            Ok(uuid) => uuid,
            // Several bridges cannot be told apart, and a wrong guess would
            // restart and re-assign the wrong one: the caller has to choose.
            Err(e @ BridgeFlashError::AmbiguousBridge { .. }) => return Err(e),
            // Nothing unassigned answered: the bridge may already carry an id,
            // which answers the re-confirmation rather than the broadcast. A
            // silent line reaches neither, which is the unassigned report.
            Err(BridgeFlashError::Open(InterfaceError::Timeout)) => {
                return serial::attach(path, baud, mcu_id, SERIAL_ATTACH_TIMEOUT, None).map_err(
                    |e| match e {
                        InterfaceError::Timeout => BridgeFlashError::UnassignedSerial {
                            path: path.to_path_buf(),
                            baud,
                        },
                        e => BridgeFlashError::Open(e),
                    },
                );
            }
            Err(e) => return Err(e),
        },
    };
    serial::attach(path, baud, mcu_id, SERIAL_ATTACH_TIMEOUT, Some(uuid))
        .map_err(BridgeFlashError::Open)
}

impl BridgeOpen {
    /// Open the transport these parameters name, as a fresh link handle and its
    /// [`Interface`].
    ///
    /// # Errors
    /// As [`BridgeLink::open_host_library`] / [`BridgeLink::open_serial`]:
    /// [`BridgeFlashError::UnassignedSerial`] / [`BridgeFlashError::AmbiguousBridge`]
    /// for a serial bridge no uuid names, [`BridgeFlashError::Open`] otherwise.
    fn build(&self) -> Result<(Interface, BridgeDevice), BridgeFlashError> {
        match self {
            BridgeOpen::Host {
                path,
                mux,
                mcu_id,
                host_port_paths,
            } => {
                let device = Arc::new(
                    HostDevice::load_mux(path, *mux, *mcu_id, host_port_paths)
                        .map_err(BridgeFlashError::Open)?,
                );
                let link = Interface::from_host(Arc::clone(&device));
                Ok((link, BridgeDevice::Host(device)))
            }
            BridgeOpen::Serial {
                path,
                baud,
                mcu_id,
                uuid,
            } => {
                let handle = attach_bridge(path, *baud, *mcu_id, *uuid)?;
                let link = Interface::shared_serial(Arc::clone(&handle));
                Ok((link, BridgeDevice::Shared(handle)))
            }
        }
    }
}

/// Broadcast the unassigned query on the bridge's tty and return the identity
/// of the one bridge that answers.
///
/// A bridge's node id lives in its firmware's RAM, so the restart
/// [`BridgeLink::reset`] performs drops it; a run that has no `--bridge-uuid`
/// to name the bridge with has to **find** it, and the discovery broadcast is
/// how — the one query an unassigned board answers, the same call
/// `klipperx admin scan --port` makes.
///
/// # Errors
/// [`BridgeFlashError::AmbiguousBridge`] when more than one bridge answers:
/// naming the wrong one would restart and re-assign the wrong board, so this
/// refuses to guess. A tty that cannot be opened or read is
/// [`BridgeFlashError::Open`], and **no** answer at all is
/// [`InterfaceError::Timeout`] — which [`BridgeLink::reopen`] retries, because a
/// bridge that has not finished coming back up is not yet unassigned, it is
/// simply not there yet.
fn discover_bridge_uuid(path: &Path, baud: u32) -> Result<[u8; 6], BridgeFlashError> {
    let mut port = serial::SerialDevice::open(path, baud).map_err(BridgeFlashError::Open)?;
    let mut first: Option<[u8; 6]> = None;
    let total = serial::serial_query_unassigned(&mut port, DISCOVER_WINDOW, |node| {
        if first.is_none() {
            first = Some(node.uuid);
        }
    })
    .map_err(|e| {
        BridgeFlashError::Open(InterfaceError::Other(format!(
            "discovery scan on {}: {e}",
            path.display()
        )))
    })?;
    match total {
        0 => Err(BridgeFlashError::Open(InterfaceError::Timeout)),
        1 => Ok(first.expect("the scan reported exactly one bridge")),
        count => Err(BridgeFlashError::AmbiguousBridge {
            count,
            path: path.to_path_buf(),
        }),
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
/// The live connection behind a [`BridgeLink`]: the `Mcu`, the [`Interface`]
/// spelling its port-level commands, and the device both are faces of.
///
/// Kept as one value (in an `Option`) so [`reset`](BridgeLink::reset) can drop
/// the whole thing before it reopens: the shared-serial registry hands out one
/// port per path while a handle is alive, and the handle a restart killed would
/// otherwise be handed back as if it were fresh.
struct LiveBridge {
    mcu: Mcu,
    link: Interface,
    device: BridgeDevice,
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
    /// How the transport was opened, so [`reset`](BridgeLink::reset) can open a
    /// fresh one after the bridge's firmware restarts.
    open: BridgeOpen,
    /// The live connection, absent only while [`reset`](BridgeLink::reset) is
    /// between dropping the restarted one and opening a fresh one.
    live: Option<LiveBridge>,
    /// The reason the bridge's firmware gave when it last shut down. The
    /// `shutdown` event handler records it; a run reports it with the
    /// declaration that triggered it instead of letting the next step time out.
    shutdown: Arc<Mutex<Option<String>>>,
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
        Self::from_open(BridgeOpen::Host {
            path: path.to_path_buf(),
            mux,
            mcu_id,
            host_port_paths: host_port_paths.to_vec(),
        })
    }

    /// Open a **shared serial** bridge on `path` at `baud`, attached as
    /// `mcu_id`.
    ///
    /// A bridge's link carries its downstream devices' frames too, so its port
    /// is opened shared and addressed by id (`serial::attach`), which also runs
    /// the assignment handshake. `uuid` is the bridge **board's** own identity:
    /// naming it skips the search. With `None` the bridge is **found** — the
    /// discovery broadcast an unassigned board answers, or the plain
    /// re-confirmation an already-assigned one answers instead — so
    /// `--bridge-uuid` is an optional override, not a requirement.
    ///
    /// # Errors
    /// [`BridgeFlashError::UnassignedSerial`] when no uuid names the bridge and
    /// neither the discovery broadcast nor the re-confirmation reaches one,
    /// [`BridgeFlashError::AmbiguousBridge`] when several unassigned bridges
    /// answer the broadcast, and [`BridgeFlashError::Open`] when the port cannot
    /// be opened, `mcu_id` is outside the addressable range, or the handshake
    /// with a named bridge times out or is refused.
    pub fn open_serial(
        path: &Path,
        baud: u32,
        mcu_id: u32,
        uuid: Option<[u8; 6]>,
    ) -> Result<Self, BridgeFlashError> {
        Self::from_open(BridgeOpen::Serial {
            path: path.to_path_buf(),
            baud,
            mcu_id,
            uuid,
        })
    }

    /// Build the `Mcu` over the transport `open` names, with the interface kept
    /// for the port-level commands.
    fn from_open(open: BridgeOpen) -> Result<Self, BridgeFlashError> {
        let (link, device) = open.build()?;
        let mcu = Mcu::new("bridge", link.clone());
        Ok(Self {
            open,
            live: Some(LiveBridge { mcu, link, device }),
            shutdown: Arc::new(Mutex::new(None)),
        })
    }

    /// The live connection, which is absent only between the two halves of a
    /// [`reset`](BridgeLink::reset).
    fn live(&self) -> &LiveBridge {
        self.live
            .as_ref()
            .expect("a live connection is in place outside reset")
    }

    /// Record the bridge's `shutdown` reason for [`flash`](BridgeLink::flash)
    /// to report.
    ///
    /// The bridge is an MCU: its firmware pushes `shutdown`
    /// (`clock=%u static_string_id=%hu`) when a handler fails, and the tool is
    /// the only one listening — without a handler the message is discarded and
    /// a declaration the bridge refused looks like a session that never
    /// answers. The handler runs on the receive task and only stores the decoded
    /// reason, so it never blocks.
    fn watch_shutdown(&self) {
        let mcu = &self.live().mcu;
        if !mcu.has_message(Shutdown::NAME) {
            return;
        }
        let slot = Arc::clone(&self.shutdown);
        let _ = mcu.bind_event::<Shutdown, _>(move |event| {
            let mut reason = slot.lock().unwrap_or_else(|poison| poison.into_inner());
            if reason.is_none() {
                *reason = Some(event.reason);
            }
        });
    }

    /// The reason the bridge shut down, taking it so a later run sees only its
    /// own shutdowns.
    fn take_shutdown(&self) -> Option<String> {
        self.shutdown
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .take()
    }

    /// Run the bridge's identify handshake: the dictionary the declaration
    /// commands are encoded from comes from the running firmware, so nothing of
    /// them can be sent before this returns.
    ///
    /// # Errors
    /// [`BridgeFlashError::Identify`] when the handshake fails or times out.
    pub async fn identify(&self, timeout: Duration) -> Result<(), BridgeFlashError> {
        self.live()
            .mcu
            .identify(timeout)
            .await
            .map(|_| ())
            .map_err(BridgeFlashError::Identify)?;
        self.watch_shutdown();
        Ok(())
    }

    /// Restart the bridge so its port table is empty before a run declares
    /// ports, then reopen the link the restart killed and identify again.
    ///
    /// A bridge's port table lives in RAM only: it is built by the declaration
    /// commands (`config_uart` / `config_bridge_port`) and outlives the host's
    /// run, so a second run against the **same** bridge meets the previous
    /// run's open slot and the firmware shuts down (`third_party/klipper/src/
    /// uart.c`: `config_uart: the port slot is already in use`). The reset is
    /// what makes every run start from the same clean state.
    ///
    /// The restart is the port-level `RESTART_DEVICE` carrying an index that
    /// names no port of the bridge's (`[0x05][index]`): the bridge's port
    /// routing consumes it only when the index names a port it has in use, and
    /// otherwise falls through to its own device half, which restarts the
    /// firmware (`third_party/klipper/src/bridge.c`, `mux.c` → `mcu_restart`).
    ///
    /// A restart re-creates the console a real bridge answers on, so the old
    /// handle is dead with it: the whole old connection is dropped before the
    /// reopen (the shared-serial registry hands the same port back while a
    /// handle is alive), and the reopen + identify is retried until it answers
    /// or [`RESET_TIMEOUT`](BridgeLink::RESET_TIMEOUT) is spent.
    ///
    /// A **host library** bridge cannot restart — its `mcu_restart` is a no-op
    /// (`third_party/klipper/src/host/host.c`) and it is the process this host
    /// is linked into — and its `config_uart` is idempotent on a slot it already
    /// opened, so there is nothing to clear and nothing to reopen.
    ///
    /// # Errors
    /// [`BridgeFlashError::Reset`] when the restart command cannot be sent or
    /// the bridge does not come back and identify in time.
    pub async fn reset(&mut self) -> Result<(), BridgeFlashError> {
        // This run reports only its own shutdowns.
        *self
            .shutdown
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = None;

        if matches!(self.open, BridgeOpen::Host { .. }) {
            return Ok(());
        }

        self.live()
            .link
            .send_restart_device(RESTART_DEVICE_TO_BRIDGE)
            .map_err(|e| BridgeFlashError::Reset(format!("sending the restart failed: {e}")))?;

        // Drop the restarted connection whole before reopening: the serial
        // registry would otherwise hand back the port whose fd died with the
        // firmware that just re-exec'd.
        if let Some(old) = self.live.take() {
            old.mcu.close();
            drop(old);
        }
        let fresh = self.reopen().await?;
        self.live = Some(fresh);
        self.watch_shutdown();
        Ok(())
    }

    /// Open a fresh connection from [`open`](BridgeLink::open) and identify it,
    /// retried (bounded) while the restarted bridge is coming back up.
    ///
    /// The retry is also what absorbs a bridge that is not up yet: an attempt
    /// that finds nothing to attach to (its tty does not answer, or the scan a
    /// uuid-less run sends finds no unassigned bridge) is dropped and tried
    /// again until [`RESET_TIMEOUT`] is spent. A run that has to **find** the
    /// bridge's identity is the exception that does not retry: an answer that
    /// names several bridges is a hard error
    /// ([`BridgeFlashError::AmbiguousBridge`]), because a wrong guess would
    /// restart and re-assign the wrong board.
    async fn reopen(&self) -> Result<LiveBridge, BridgeFlashError> {
        let deadline = Instant::now() + RESET_TIMEOUT;
        loop {
            let last = match self.open_after_restart().await {
                Ok(live) => return Ok(live),
                Err(e @ BridgeFlashError::AmbiguousBridge { .. }) => return Err(e),
                Err(e) => e.to_string(),
            };
            if Instant::now() >= deadline {
                return Err(BridgeFlashError::Reset(format!(
                    "the bridge did not come back within {RESET_TIMEOUT:?}: {last}{}",
                    self.rediscover_hint()
                )));
            }
            tokio::time::sleep(RESET_RETRY_DELAY).await;
        }
    }

    /// One reopen attempt: open the transport the restart left behind, name the
    /// bridge when it has to be named, and identify.
    async fn open_after_restart(&self) -> Result<LiveBridge, BridgeFlashError> {
        let (link, device) = match &self.open {
            BridgeOpen::Serial {
                path,
                baud,
                mcu_id,
                uuid,
            } => {
                // A restart drops the bridge's node id (it lives in the
                // firmware's RAM), so a serial bridge has to be **named** again
                // before it answers: the uuid the caller gave, or one the
                // discovery scan finds on its now-unassigned line.
                let handle = attach_bridge(path, *baud, *mcu_id, *uuid)?;
                let link = Interface::shared_serial(Arc::clone(&handle));
                (link, BridgeDevice::Shared(handle))
            }
            // A host library never restarts, so `reset` never reopens one; the
            // shared construction keeps the two paths from drifting.
            BridgeOpen::Host { .. } => self.open.build()?,
        };
        let mcu = Mcu::new("bridge", link.clone());
        mcu.identify(RESET_IDENTIFY_TIMEOUT)
            .await
            .map_err(BridgeFlashError::Identify)?;
        Ok(LiveBridge { mcu, link, device })
    }

    /// The tail a reset timeout carries when the bridge had to be found rather
    /// than named: the flag that would have skipped the scan.
    fn rediscover_hint(&self) -> String {
        match &self.open {
            BridgeOpen::Serial {
                path, uuid: None, ..
            } => format!(
                "; no --bridge-uuid was given and the discovery scan found no unassigned \
                 bridge — pass --bridge-uuid (take it from `klipperx admin scan --port {}`)",
                path.display()
            ),
            _ => String::new(),
        }
    }

    /// Declare `ports` on the bridge — its oid table and its port table — and
    /// stop there.
    ///
    /// `admin bridge config` is the caller: the declaration is the whole job, so
    /// nothing is reset, no route is named, and no tunnel is opened. The
    /// declaration is what a later port-level command needs — the bridge drops a
    /// command that names a port it does not have, in silence — so it goes first.
    ///
    /// # Errors
    /// [`BridgeFlashError::Shutdown`] when the bridge's firmware shuts down while
    /// the ports are declared (its reason included), and
    /// [`BridgeFlashError::Bridge`] when [`send_declare_ports`] fails for any
    /// other reason.
    pub async fn declare_ports(&self, ports: &[BridgePort]) -> Result<(), BridgeFlashError> {
        match send_declare_ports(&self.live().mcu, ports).await {
            Ok(()) => match self.take_shutdown() {
                Some(reason) => Err(BridgeFlashError::Shutdown { reason }),
                None => Ok(()),
            },
            Err(e) => Err(match self.take_shutdown() {
                Some(reason) => BridgeFlashError::Shutdown { reason },
                None => BridgeFlashError::Bridge(e),
            }),
        }
    }

    /// Name the target's port on `route` and open its flashing tunnel —
    /// `admin bridge port`'s whole job (that command declares the port first,
    /// since a bridge's port table lives in RAM and starts empty).
    ///
    /// The port has to be one an earlier
    /// [`declare_ports`](BridgeLink::declare_ports) or
    /// [`declare_port`](BridgeLink::declare_port) made: a port-level command that
    /// names a port the bridge does not have is dropped in silence.
    ///
    /// # Errors
    /// [`BridgeFlashError::Bridge`] when the link refuses the write.
    pub fn open_target_port(
        &self,
        target: BridgeTarget,
        route: PortRoute,
    ) -> Result<(), BridgeFlashError> {
        send_open_target_port(&self.live().link, target, route).map_err(BridgeFlashError::Bridge)
    }

    /// Reset one **declared** port's device through the bridge's admin channel —
    /// `admin reset --port`. The same three units [`flash`](BridgeLink::flash)
    /// sends before a session (`0x02` into the bootloader, `0x04` a line pulse,
    /// `0x05` a restart), for a caller that only wants the reset.
    ///
    /// The port has to be one the bridge has already **declared** (an
    /// `admin bridge config`, or a flash run's own declaration): a port-level
    /// command that names a port the bridge does not have is dropped in silence,
    /// and no receipt comes back either way.
    ///
    /// # Errors
    /// [`BridgeFlashError::PortReset`] when the link refuses the write.
    pub fn reset_port(&self, index: u8, reset: PortReset) -> Result<(), BridgeFlashError> {
        send_port_reset(&self.live().link, index, reset).map_err(|e| match e {
            BridgeError::Link(e) => BridgeFlashError::PortReset(e),
            other => BridgeFlashError::Bridge(other),
        })
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
    /// A target whose plan reset is [`PortReset::Bootloader`] was **sent back to
    /// its bootloader** and is still coming up when the tunnel opens, so nothing
    /// answers the session's first `CONNECT` yet: that case gets the session's
    /// own bounds again — `cfg.tries` sessions, `cfg.read_timeout` apart — and
    /// the retries stop at the first session that gets past `CONNECT` (a failure
    /// after it has written a block is never retried). Every other reset keeps
    /// the single session, and a target that never answers still fails with the
    /// session's own error.
    ///
    /// The tunnel is cleared on every path, success or failure, because the
    /// bridge outlives this run; a clearing that fails after a successful flash
    /// is reported as [`BridgeFlashError::Teardown`], and a failed flash keeps
    /// its own error as the primary one.
    ///
    /// # Errors
    /// [`BridgeFlashError::Unaddressable`] when the target's node id names no
    /// arb, [`BridgeFlashError::Reset`] when the bridge could not be taken back
    /// to a clean state, [`BridgeFlashError::Shutdown`] when the bridge's
    /// firmware shuts down while the port is declared (its reason included),
    /// [`BridgeFlashError::Bridge`] when [`open_bridge`] fails for any other
    /// reason, [`BridgeFlashError::Session`] when the katapult session fails,
    /// and [`BridgeFlashError::Teardown`] when only the tunnel clearing fails.
    pub async fn flash(
        &mut self,
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

        // Every run starts from a clean bridge: the previous run's port table is
        // still in the firmware's RAM and would shut it down on this run's
        // declaration.
        self.reset().await?;

        if let Err(e) = open_bridge(&self.live().mcu, &self.live().link, ports, target).await {
            // A declaration the bridge refused shuts its firmware down, and the
            // reason is the whole story: report that, not the timeout behind it.
            return Err(match self.take_shutdown() {
                Some(reason) => BridgeFlashError::Shutdown { reason },
                None => BridgeFlashError::Bridge(e),
            });
        }
        if let Some(reason) = self.take_shutdown() {
            return Err(BridgeFlashError::Shutdown { reason });
        }

        // The uplink listener is registered before the session's first block,
        // and the adapter below is what a session drives. The receiver lives in
        // the `TunnelBridge`, so dropping it below is also what unregisters the
        // sink.
        let (sink, frames) = TunnelSink::channel();
        self.live().device.set_tunnel_sink(reply_arb, sink);
        let bridge = TunnelBridge::new(down_arb, frames, self.live().device.tunnel_link());
        let (mut reader, mut writer) = bridge.split();
        let outcome = run_session(
            ports,
            target,
            &mut reader,
            &mut writer,
            payload,
            cfg,
            progress,
        )
        .await;
        drop(reader);
        drop(writer);
        drop(bridge);

        // The bridge stays up after the host exits, so the port must not be
        // left carrying the katapult raw stream: a repeat toggle flips it back
        // to the mux units its other users expect.
        let cleared = self.live().link.send_set_port_tunnel(target.index);
        match (outcome, cleared) {
            (Ok(summary), Ok(())) => Ok(summary),
            (Ok(_), Err(e)) => Err(BridgeFlashError::Teardown(e)),
            // A firmware that shut down mid-session explains its own failure.
            (Err(e), _) => Err(match self.take_shutdown() {
                Some(reason) => BridgeFlashError::Shutdown { reason },
                None => BridgeFlashError::Session(e),
            }),
        }
    }

    /// Declare **one** port at `index` — the scattered-declaration sibling of
    /// [`declare_ports`](BridgeLink::declare_ports), for a caller that has work
    /// to do between declarations (a bridge scan declares a port, scans through
    /// it, then declares the next). Ports are declared in ascending index from
    /// 0, because the bridge numbers a port by the order the declaration rows
    /// reach it.
    ///
    /// # Errors
    /// [`BridgeFlashError::Shutdown`] when the bridge's firmware shuts down over
    /// the declaration (its reason included — a slot another object already
    /// opened is the common one), and [`BridgeFlashError::Bridge`] when the
    /// declaration fails for any other reason.
    pub async fn declare_port(&self, index: u8, baud: u32) -> Result<(), BridgeFlashError> {
        match send_declare_port(&self.live().mcu, index, baud).await {
            Ok(()) => match self.take_shutdown() {
                Some(reason) => Err(BridgeFlashError::Shutdown { reason }),
                None => Ok(()),
            },
            Err(e) => Err(match self.take_shutdown() {
                Some(reason) => BridgeFlashError::Shutdown { reason },
                None => BridgeFlashError::Bridge(e),
            }),
        }
    }

    /// How many UART slots the bridge's backend offers, from its
    /// `BRIDGE_UART_COUNT` constant — `None` on a firmware that predates it, so
    /// a caller can say so and fall back (`admin bridge config`).
    ///
    /// The count is the backend's: each one declares the slots it can open
    /// (`third_party/klipper/src/{linux,host}/bridge.c`,
    /// `src/stm32/bridge_serial.c`), so this is what the bridge can actually be
    /// given rather than what a command line assumes.
    pub fn uart_count(&self) -> Option<u8> {
        let dictionary = self.live().mcu.dictionary()?;
        let count = dictionary.constant("BRIDGE_UART_COUNT")?.as_u64()?;
        u8::try_from(count).ok()
    }

    /// The `MCU` constant the bridge's firmware reports (`stm32f103xe`,
    /// `linux`, …) — what `admin bridge config` names the arch by when the
    /// firmware names no slot ([`uart_slots`](BridgeLink::uart_slots)). Absent
    /// on a build that declares none (the host library).
    pub fn mcu_name(&self) -> Option<String> {
        let dictionary = self.live().mcu.dictionary()?;
        Some(dictionary.constant("MCU")?.as_str()?.to_string())
    }

    /// How the bridge's firmware names each of its UART slots, in slot order —
    /// one entry per slot [`uart_count`](BridgeLink::uart_count) counts, each as
    /// `<USART> <RX>/<TX>` (`USART1 PA10/PA9`). `None` on a backend that names
    /// no slot, so a caller can fall back to naming the arch.
    ///
    /// Only the stm32 serial backend names its slots in the dictionary: it
    /// reserves each ticked USART's pins under a constant whose value is the pin
    /// pair (`third_party/klipper/src/stm32/bridge_serial.c`). A slot is a
    /// ticked USART's place among the ticked ones, so an unticked USART leaves
    /// no entry and the slots after it move up — which is why this reads the
    /// backend's own declarations instead of deriving a USART from a slot
    /// number. The linux bridge's single tty and the host library's ttys declare
    /// only a count, so they name nothing here.
    pub fn uart_slots(&self) -> Option<Vec<String>> {
        let dictionary = self.live().mcu.dictionary()?;
        let slots = stm32_bridge_serial_slots(&dictionary);
        (!slots.is_empty()).then_some(slots)
    }

    /// The shared serial port a **serial** bridge's link is on — what a scan
    /// needs, because a downstream device's answer carries an id no MCU owns
    /// and only reaches a listener registered on the port itself
    /// ([`SharedSerialHandle::set_unit_sink`]). `None` for a host library.
    pub(crate) fn shared_handle(&self) -> Option<&Arc<SharedSerialHandle>> {
        match &self.live().device {
            BridgeDevice::Shared(handle) => Some(handle),
            BridgeDevice::Host(_) => None,
        }
    }

    /// Shut the bridge's connection down explicitly, before the handles go.
    ///
    /// The same step [`Mcu::close`] is elsewhere: the transport tasks hold
    /// `Interface` clones (and so the link's resources) until they are aborted
    /// and their blocking reads released, which dropping the `Mcu` alone does
    /// not force.
    pub fn close(&self) {
        self.live().mcu.close();
    }
}

/// The katapult session over an already-open tunnel — [`BridgeLink::flash`]'s
/// step 3, with the one retry a **bootloader-sent** target needs.
///
/// A target whose plan reset is [`PortReset::Bootloader`] was sent back to its
/// bootloader and is still starting when the tunnel opens, so the session's
/// first `CONNECT` may reach nothing: that case gets `cfg.tries` sessions,
/// `cfg.read_timeout` apart. Only a `CONNECT` that spent its own tries earns
/// another session — a failure after it has written a block is never retried,
/// because the device is then mid-flash — and any other reset keeps the single
/// session [`session::flash_with_config`] runs.
async fn run_session<R: std::io::Read, W: std::io::Write>(
    ports: &[BridgePort],
    target: BridgeTarget,
    reader: &mut R,
    writer: &mut W,
    payload: &[u8],
    cfg: &SessionConfig,
    mut progress: impl FnMut(&str),
) -> Result<Summary, FlashError> {
    let attempts = match ports.get(usize::from(target.index)) {
        Some(port) if port.reset == PortReset::Bootloader => cfg.tries.max(1),
        _ => 1,
    };
    let mut attempt = 1;
    loop {
        let outcome = session::flash_with_config(reader, writer, payload, cfg, &mut progress);
        match outcome {
            Err(FlashError::RetriesExhausted { cmd, .. })
                if cmd == CONNECT && attempt < attempts =>
            {
                attempt += 1;
                tokio::time::sleep(cfg.read_timeout).await;
            }
            other => return other,
        }
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
    /// The bridge could not be taken back to a clean state before the
    /// declaration: the self-restart could not be sent, or the bridge did not
    /// answer again within [`RESET_TIMEOUT`](BridgeLink::RESET_TIMEOUT).
    Reset(String),
    /// A port-level reset (`0x02`/`0x04`/`0x05`) could not be written — a closed
    /// link, a failed write, or a link that carries no port mux units at all.
    PortReset(InterfaceError),
    /// The bridge's firmware shut down while the port was declared — the reason
    /// it gave, resolved from its `static_string_id` enumeration. Reported
    /// instead of the session timeout that would otherwise follow, so a
    /// declaration the bridge refused says why.
    Shutdown {
        /// The firmware's own shutdown reason.
        reason: String,
    },
    /// Several unassigned bridges answered the discovery scan a serial bridge
    /// opened without a `--bridge-uuid` sends after its restart: which one this
    /// run means cannot be told, and a wrong guess would restart and re-assign
    /// the wrong board.
    AmbiguousBridge {
        /// How many distinct bridges answered.
        count: usize,
        /// The tty the scan ran on.
        path: PathBuf,
    },
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
                 node id yet it needs --bridge-uuid (take it from `klipperx admin scan --port {}`)",
                path.display(),
                path.display()
            ),
            BridgeFlashError::Identify(e) => write!(f, "identifying the bridge: {e}"),
            BridgeFlashError::Bridge(e) => write!(f, "opening the bridge port: {e}"),
            BridgeFlashError::Reset(message) => write!(f, "resetting the bridge: {message}"),
            BridgeFlashError::PortReset(e) => write!(f, "resetting the bridge port: {e}"),
            BridgeFlashError::Shutdown { reason } => {
                write!(f, "the bridge's firmware shut down: {reason}")
            }
            BridgeFlashError::AmbiguousBridge { count, path } => write!(
                f,
                "{count} unassigned bridges answered on {}; pass --bridge-uuid to name this \
                 one (take it from `klipperx admin scan --port {}`)",
                path.display(),
                path.display()
            ),
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
            BridgeFlashError::PortReset(e) => Some(e),
            BridgeFlashError::Identify(e) => Some(e),
            BridgeFlashError::Bridge(e) => Some(e),
            BridgeFlashError::Session(e) => Some(e),
            BridgeFlashError::Unaddressable { .. }
            | BridgeFlashError::UnassignedSerial { .. }
            | BridgeFlashError::Reset(_)
            | BridgeFlashError::AmbiguousBridge { .. }
            | BridgeFlashError::Shutdown { .. } => None,
        }
    }
}

/// Which of [`STM32_BRIDGE_SERIAL_UARTS`] a dictionary's bridge was built with,
/// in slot order and named the way the backend names it (`USART2 PA3/PA2`).
///
/// A ticked USART is one whose `RESERVE_PINS_*` constant the dictionary carries;
/// that constant's value is the pin pair it is wired to, RX first
/// (`PA3,PA2`), which reads better as `PA3/PA2`. Empty for a firmware that
/// declares none (every backend but the stm32 serial one).
fn stm32_bridge_serial_slots(dictionary: &Dictionary) -> Vec<String> {
    STM32_BRIDGE_SERIAL_UARTS
        .iter()
        .filter_map(|(usart, reserve)| {
            let pins = dictionary
                .constant(&format!("RESERVE_PINS_{reserve}"))?
                .as_str()?;
            Some(format!("{usart} {}", pins.replace(',', "/")))
        })
        .collect()
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
    use crate::flash::fake::{FakeBootloader, Quirk};
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

    /// The identity the linux bridge fixture is started with: the same bytes the
    /// link names it by, or the firmware never adopts its node id.
    const LINUX_BRIDGE_UUID: [u8; 6] = [0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6];

    /// How many UART slots the linux bridge backend serves — one static stream,
    /// slot 0 only (`third_party/klipper/src/linux/bridge.c`).
    const LINUX_BRIDGE_UARTS: u8 = 1;

    /// How many UART slots the host library serves: one per tty slot of the
    /// host's table (`KLIPPER_HOST_PORT_SLOTS`,
    /// `third_party/klipper/src/host/bridge.c`).
    const HOST_LIBRARY_UARTS: u8 = 4;

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

    /// The [`HostMux`] a `host_library:` config names for `path` — the config
    /// layer's judgment for a path, which is what the library is given and
    /// named with (`HostLibraries`, the same scan `McuConfig::open` consumes).
    fn host_mux_for(path: &str) -> HostMux {
        let text = format!("[mcu]\nhost_library: {path}\n");
        let (config, _) = Config::from_text(&text).expect("the config parses");
        let section = config
            .sections()
            .find(|section| section.id == "mcu")
            .expect("the config has an [mcu] section");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        let libraries = HostLibraries::classify(&wrapper).expect("the path is told apart");
        let identity = libraries.identity(path).expect("the path has an identity");
        HostMux {
            identity,
            named: identity,
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

    /// The fake's store for `payload` at `start_addr`, block for block: a block
    /// the session padded to `block_size` with `0xFF` is compared padded, so a
    /// short final block is checked exactly too.
    fn assert_fake_stored(
        fake: &FakeBootloader,
        start_addr: u32,
        payload: &[u8],
        block_size: usize,
    ) {
        let memory = fake.memory();
        for (i, chunk) in payload.chunks(block_size).enumerate() {
            let addr = start_addr + (i * block_size) as u32;
            let stored = memory
                .get(&addr)
                .unwrap_or_else(|| panic!("block {i} at {addr:#x} never arrived"));
            let mut expected = chunk.to_vec();
            expected.resize(block_size, 0xFF);
            assert_eq!(
                stored.as_slice(),
                expected.as_slice(),
                "block {i} at {addr:#x}"
            );
        }
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
        let mux = host_mux_for(&path);

        let mut link = BridgeLink::open_host_library(
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

    /// `admin bridge config` and `admin bridge port` over the **real** mux
    /// library: the bridge's ports are declared by the first, the target's port
    /// named legacy and its tunnel opened by the second — and nothing follows. No
    /// device is reset and no session runs, so the port's pty stays silent (a
    /// reset pulse or a katapult frame would show there), which is what
    /// [`BridgeLink::declare_ports`] and [`BridgeLink::open_target_port`] promise.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_the_config_and_port_halves_leave_the_port_silent() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        let port = Pty::new();
        let library = klipperx_test_support::klipper_host_mux_lib_path();
        let path = library.display().to_string();

        let link = BridgeLink::open_host_library(
            Path::new(&path),
            host_mux_for(&path),
            BRIDGE_MCU_ID,
            &[(0u8, port.path.display().to_string())],
        )
        .expect("the muxed library opens and attaches");
        tokio::time::timeout(Duration::from_secs(20), link.identify(IDENTIFY_TIMEOUT))
            .await
            .expect("the identify handshake timed out")
            .expect("the bridge identifies");

        let ports = [BridgePort {
            baud: 250_000,
            reset: PortReset::ResetPort,
        }];
        let target = BridgeTarget {
            index: 0,
            node_id: TARGET_NODE_ID,
        };
        link.declare_ports(&ports)
            .await
            .expect("the bridge declares its ports without resetting anything");
        link.open_target_port(target, PortRoute::Legacy)
            .expect("the target's port is named and its tunnel opened");

        // The port's pty is watched for the whole window: neither half writes
        // nothing down it, so the fake bootloader behind it hears no CONNECT and
        // no frame at all reaches the port.
        let fake = FakeBootloader::new();
        let recorded = Arc::new(Mutex::new(Vec::<u8>::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let pump = start_fake_pump(&port, &fake, Arc::clone(&recorded), Arc::clone(&stop));
        tokio::time::sleep(Duration::from_millis(300)).await;
        stop.store(true, Ordering::Relaxed);
        pump.join().expect("the pump thread");

        assert!(
            recorded.lock().unwrap().is_empty(),
            "the config and port halves write nothing to the port's pty, got {:?}",
            recorded.lock().unwrap()
        );
        assert_eq!(fake.seen(command::CONNECT), 0, "no session ran");

        link.close();
        drop(link);
        wait_for_library_unload(&library);
    }

    /// `admin reset --port 0 --port-reset bootloader`'s one write, over the
    /// **real** mux library: the bridge forwards the request-bootloader unit
    /// (`0x02`) to the port it names, and the port's pty is where that lands —
    /// the whole action a reset run has, with no receipt to wait for.
    ///
    /// The one unit is all `admin reset` can do: it has no id to name a port
    /// with. A flash run's bootloader step names the port **units** first, the
    /// only route that does not strip this very unit on its way down
    /// ([`request_bootloader`](crate::flash::bridge_session::request_bootloader)).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_admin_reset_sends_the_target_port_into_its_bootloader() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        let port = Pty::new();
        let library = klipperx_test_support::klipper_host_mux_lib_path();
        let path = library.display().to_string();

        let link = BridgeLink::open_host_library(
            Path::new(&path),
            host_mux_for(&path),
            BRIDGE_MCU_ID,
            &[(0u8, port.path.display().to_string())],
        )
        .expect("the muxed library opens and attaches");
        tokio::time::timeout(Duration::from_secs(20), link.identify(IDENTIFY_TIMEOUT))
            .await
            .expect("the identify handshake timed out")
            .expect("the bridge identifies");

        // The reset addresses a port the bridge has to have declared: a
        // port-level command naming a port it does not have is dropped in
        // silence, and the pty would stay quiet for the wrong reason.
        let ports = [BridgePort {
            baud: 250_000,
            reset: PortReset::Bootloader,
        }];
        link.declare_ports(&ports)
            .await
            .expect("the bridge declares its port");

        let fake = FakeBootloader::new();
        let recorded = Arc::new(Mutex::new(Vec::<u8>::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let pump = start_fake_pump(&port, &fake, Arc::clone(&recorded), Arc::clone(&stop));

        link.reset_port(0, PortReset::Bootloader)
            .expect("the bootloader request goes out");
        tokio::time::sleep(Duration::from_millis(300)).await;
        stop.store(true, Ordering::Relaxed);
        pump.join().expect("the pump thread");

        // The port's flags are the declaration's (legacy), so the bridge
        // forwards the unit without its mux header: the bytes on the port are
        // the KXID frame the host wrote, `[0x02][index]` and nothing else.
        let mut unit = serial::admin_unit(&[serial::CMD_REQUEST_BOOTLOADER, 0]);
        unit.drain(..serial::HEADER_BYTES);
        assert_eq!(
            *recorded.lock().unwrap(),
            unit,
            "the port carries the firmware's request-bootloader unit, and no katapult frame"
        );
        assert_eq!(fake.seen(command::CONNECT), 0, "no session ran");

        link.close();
        drop(link);
        wait_for_library_unload(&library);
    }

    /// A bridge's own firmware says how many UART slots its backend offers: the
    /// linux bridge serves one stream on slot 0, and reports exactly that — the
    /// count `admin bridge config` lists instead of guessing a `--port-count`.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_the_bridge_reports_its_uart_count() {
        let port = Pty::new();
        let firmware =
            klipperx_test_support::LinuxFirmware::spawn_bridge_port(&port.path, LINUX_BRIDGE_UUID)
                .expect("the bridge firmware process starts");
        let link = BridgeLink::open_serial(
            firmware.serial_path(),
            250_000,
            BRIDGE_MCU_ID,
            Some(LINUX_BRIDGE_UUID),
        )
        .expect("the bridge's console attaches and is named");
        tokio::time::timeout(Duration::from_secs(20), link.identify(IDENTIFY_TIMEOUT))
            .await
            .expect("the identify handshake timed out")
            .expect("the linux bridge identifies");

        assert_eq!(link.uart_count(), Some(LINUX_BRIDGE_UARTS));
        assert_eq!(link.mcu_name().as_deref(), Some("linux"));
        // The linux backend declares a count and no per-slot name, so the report
        // has only the arch's note to name its slot with.
        assert_eq!(link.uart_slots(), None);

        link.close();
        drop(link);
        drop(firmware);
    }

    /// Which USART each slot is comes from the backend's own reserved-pin
    /// constants, in the order it numbers its slots (`USART1`, `USART2`, …,
    /// `UART5`), with the pin pair read as `RX/TX`.
    ///
    /// `third_party/klipper/src/stm32/bridge_serial.c`: a ticked USART reserves
    /// `RESERVE_PINS_bridge_serial*` = its pins, and USART1 keeps the name the
    /// single-port backend always used.
    #[test]
    fn test_the_stm32_bridge_names_its_slots_from_its_reserved_pins() {
        let dictionary = Dictionary::from_json(serde_json::json!({
            "config": {
                "BRIDGE_UART_COUNT": 2,
                "MCU": "stm32f103xe",
                "RESERVE_PINS_bridge_serial": "PA10,PA9",
                "RESERVE_PINS_bridge_serial_usart2": "PA3,PA2",
            }
        }))
        .expect("a stm32 bridge dictionary");
        assert_eq!(
            stm32_bridge_serial_slots(&dictionary),
            vec!["USART1 PA10/PA9".to_string(), "USART2 PA3/PA2".to_string()]
        );
    }

    /// A slot is a ticked USART's place **among the ticked ones**, so ticking a
    /// later USART alone moves it to slot 0: naming slots off a fixed
    /// slot→USART table would call this one `USART1`, which is a port the
    /// firmware does not have.
    #[test]
    fn test_a_later_stm32_usart_alone_moves_up_to_slot_zero() {
        let dictionary = Dictionary::from_json(serde_json::json!({
            "config": {
                "BRIDGE_UART_COUNT": 2,
                "MCU": "stm32f103xe",
                "RESERVE_PINS_bridge_serial_usart3": "PB11,PB10",
                "RESERVE_PINS_bridge_serial_uart5": "PD2,PC12",
            }
        }))
        .expect("a stm32 bridge dictionary");
        assert_eq!(
            stm32_bridge_serial_slots(&dictionary),
            vec!["USART3 PB11/PB10".to_string(), "UART5 PD2/PC12".to_string()]
        );
    }

    /// A backend that reserves no USART pins names no slot — the host library's
    /// dictionary has no such constants, and neither does the linux bridge's.
    #[test]
    fn test_a_bridge_without_reserved_pins_names_no_slot() {
        let dictionary = Dictionary::from_json(serde_json::json!({
            "config": { "BRIDGE_UART_COUNT": 4 }
        }))
        .expect("a host library bridge dictionary");
        assert!(stm32_bridge_serial_slots(&dictionary).is_empty());
    }

    /// The host library's count is the host's own tty table: one UART slot per
    /// slot a tty could be named in, which is what its backend enforces
    /// (`klipper_host_port`, `KLIPPER_HOST_PORT_SLOTS`).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_the_host_library_bridge_reports_a_slot_per_tty() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        let port = Pty::new();
        let library = klipperx_test_support::klipper_host_mux_lib_path();
        let path = library.display().to_string();
        let link = BridgeLink::open_host_library(
            Path::new(&path),
            host_mux_for(&path),
            BRIDGE_MCU_ID,
            &[(0u8, port.path.display().to_string())],
        )
        .expect("the muxed library opens and attaches");
        tokio::time::timeout(Duration::from_secs(20), link.identify(IDENTIFY_TIMEOUT))
            .await
            .expect("the identify handshake timed out")
            .expect("the bridge identifies");

        assert_eq!(link.uart_count(), Some(HOST_LIBRARY_UARTS));
        // The host library declares no `MCU` constant, so the report cannot name
        // an arch for its slots — `admin bridge config` says `arch-named` — and
        // it names no slot either: its ttys are its host's to name.
        assert_eq!(link.mcu_name(), None);
        assert_eq!(link.uart_slots(), None);

        link.close();
        drop(link);
        wait_for_library_unload(&library);
    }

    /// `admin reset`'s **bridge** path: the self-restart goes out, the firmware
    /// re-execs, and the link comes back and identifies again — proven by a port
    /// declaration the restarted bridge accepts. This is the same restart
    /// [`BridgeLink::flash`] runs before it declares its ports.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_admin_reset_restarts_the_bridge_and_reidentifies() {
        const BRIDGE_UUID: [u8; 6] = [0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6];

        let port = Pty::new();
        let firmware =
            klipperx_test_support::LinuxFirmware::spawn_bridge_port(&port.path, BRIDGE_UUID)
                .expect("the bridge firmware process starts");

        let mut link = BridgeLink::open_serial(
            firmware.serial_path(),
            250_000,
            BRIDGE_MCU_ID,
            Some(BRIDGE_UUID),
        )
        .expect("the bridge's console attaches and is named");
        tokio::time::timeout(Duration::from_secs(20), link.identify(IDENTIFY_TIMEOUT))
            .await
            .expect("the identify handshake timed out")
            .expect("the linux bridge identifies");

        link.reset()
            .await
            .expect("the bridge restarts itself and answers again");

        // The restart really happened and the bridge is up: the declaration a
        // fresh firmware accepts is one the pre-restart bridge would have
        // refused (its port slot was in use).
        let ports = [BridgePort {
            baud: 250_000,
            reset: PortReset::ResetPort,
        }];
        link.declare_ports(&ports)
            .await
            .expect("the restarted bridge accepts a declaration");

        link.close();
        drop(link);
        drop(firmware);
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
        assert!(text.contains("klipperx admin scan --port"), "{text}");
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

    /// A declaration the bridge refuses is reported with the firmware's own
    /// reason. Here the port tty is not there, so the declaration's
    /// `config_uart` cannot open it and the firmware shuts down
    /// (`config_uart: the UART device could not be opened`); the run says so
    /// instead of leaving the session to time out behind it.
    ///
    /// A host-library bridge is used because the failure is the firmware's own
    /// shutdown, which a real bridge and a library reach the same way — only
    /// the tty the port is told to open differs.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_a_declaration_the_bridge_refuses_names_the_shutdown_reason() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        let library = klipperx_test_support::klipper_host_mux_lib_path();
        let path = library.display().to_string();
        // A tty the library is told its port 0 is on, but which is not there:
        // `config_uart` cannot open it, so the firmware shuts down.
        let missing = "/nonexistent/klipperx-bridge-port-0".to_string();

        let mut link = BridgeLink::open_host_library(
            Path::new(&path),
            host_mux_for(&path),
            BRIDGE_MCU_ID,
            &[(0u8, missing)],
        )
        .expect("the muxed library opens and attaches");
        tokio::time::timeout(Duration::from_secs(20), link.identify(IDENTIFY_TIMEOUT))
            .await
            .expect("the identify handshake timed out")
            .expect("the bridge identifies");

        let ports = [BridgePort {
            baud: 250_000,
            reset: PortReset::ResetPort,
        }];
        let target = BridgeTarget {
            index: 0,
            node_id: TARGET_NODE_ID,
        };
        let err = link
            .flash(&ports, target, &[0u8; 8], &test_config(), |_| {})
            .await
            .expect_err("a port that cannot be opened shuts the bridge down");

        let text = err.to_string();
        assert!(matches!(err, BridgeFlashError::Shutdown { .. }), "{text}");
        assert_eq!(
            text,
            "the bridge's firmware shut down: config_uart: the UART device could not be \
             opened",
            "the error carries the firmware's own reason"
        );

        link.close();
        drop(link);
        wait_for_library_unload(&library);
    }

    /// Two runs against the **same** bridge instance: each run resets the
    /// firmware before it declares its port, so the first run's port table is
    /// gone before the second declares its own. Both runs finish with a
    /// Summary, and neither leaves a bridge shutdown behind.
    ///
    /// The bridge is a real linux firmware process, whose `mcu_restart` re-execs
    /// it (the only offline fixture that can actually restart), so a
    /// host-library bridge — whose `mcu_restart` is a no-op — is not enough.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_two_runs_against_the_same_bridge_are_both_clean() {
        const BRIDGE_UUID: [u8; 6] = [0x71, 0x72, 0x73, 0x74, 0x75, 0x76];

        let port = Pty::new();
        let firmware =
            klipperx_test_support::LinuxFirmware::spawn_bridge_port(&port.path, BRIDGE_UUID)
                .expect("the bridge firmware process starts");

        let fake = FakeBootloader::new();
        let recorded = Arc::new(Mutex::new(Vec::<u8>::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let pump = start_fake_pump(&port, &fake, Arc::clone(&recorded), Arc::clone(&stop));

        let mut link = BridgeLink::open_serial(
            firmware.serial_path(),
            250_000,
            BRIDGE_MCU_ID,
            Some(BRIDGE_UUID),
        )
        .expect("the bridge's console attaches and is named");
        tokio::time::timeout(Duration::from_secs(20), link.identify(IDENTIFY_TIMEOUT))
            .await
            .expect("the identify handshake timed out")
            .expect("the linux bridge identifies");

        let ports = [BridgePort {
            baud: 250_000,
            reset: PortReset::ResetPort,
        }];
        let target = BridgeTarget {
            index: 0,
            node_id: TARGET_NODE_ID,
        };
        let payload: Vec<u8> = (0..100u8).collect();

        let first = link
            .flash(&ports, target, &payload, &test_config(), |_| {})
            .await
            .expect("the first run completes");
        // The second run starts against the firmware the first left — the same
        // process, restarted by the run's own reset — so this is one bridge
        // instance seen twice, which is exactly what shut the firmware down
        // before the reset existed.
        let second = link
            .flash(&ports, target, &payload, &test_config(), |_| {})
            .await
            .expect("the second run completes against the same bridge");

        for summary in [&first, &second] {
            assert_eq!(summary.start_addr, 0x0800_2000, "the fake's start address");
            assert_eq!(summary.block_size, 64);
            assert_eq!(summary.mcu_type, "STM32F103");
            assert_eq!(summary.bytes_flashed, payload.len());
            assert_eq!(summary.block_count, 2, "100 bytes is two 64-byte blocks");
        }
        assert_eq!(fake.seen(command::CONNECT), 2, "one session per run");
        assert_eq!(fake.seen(command::SEND_BLOCK), 4, "two blocks per run");
        assert_eq!(fake.seen(command::COMPLETE), 2);

        stop.store(true, Ordering::Relaxed);
        pump.join().expect("the pump thread");
        link.close();
        drop(link);
        drop(firmware);
    }

    /// A **whole image** through the bridge: 43 KiB, the size a real katapult
    /// payload reaches, so the sustained transfer path — every block queued and
    /// drained, verified one by one — is what actually runs, not a two-block
    /// warm-up. The fake's own memory is checked block for block, which is the
    /// strongest form of "not one byte was lost or reordered".
    #[tokio::test(flavor = "multi_thread")]
    async fn test_a_whole_image_flashes_through_the_bridge() {
        const BRIDGE_UUID: [u8; 6] = [0x81, 0x82, 0x83, 0x84, 0x85, 0x86];
        const IMAGE: usize = 43 * 1024;

        let port = Pty::new();
        let firmware =
            klipperx_test_support::LinuxFirmware::spawn_bridge_port(&port.path, BRIDGE_UUID)
                .expect("the bridge firmware process starts");

        let fake = FakeBootloader::new();
        let recorded = Arc::new(Mutex::new(Vec::<u8>::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let pump = start_fake_pump(&port, &fake, Arc::clone(&recorded), Arc::clone(&stop));

        let mut link = BridgeLink::open_serial(
            firmware.serial_path(),
            250_000,
            BRIDGE_MCU_ID,
            Some(BRIDGE_UUID),
        )
        .expect("the bridge's console attaches and is named");
        tokio::time::timeout(Duration::from_secs(20), link.identify(IDENTIFY_TIMEOUT))
            .await
            .expect("the identify handshake timed out")
            .expect("the linux bridge identifies");

        let ports = [BridgePort {
            baud: 250_000,
            reset: PortReset::ResetPort,
        }];
        let target = BridgeTarget {
            index: 0,
            node_id: TARGET_NODE_ID,
        };
        // 43 KiB is exactly 688 64-byte blocks, so no block is short and the
        // stored content is the payload itself.
        let payload: Vec<u8> = (0..IMAGE).map(|i| (i % 251) as u8).collect();
        let blocks = IMAGE / 64;

        let summary = link
            .flash(&ports, target, &payload, &test_config(), |_| {})
            .await
            .expect("the whole image flashes through the bridge");
        assert_eq!(summary.bytes_flashed, IMAGE);
        assert_eq!(summary.block_count, blocks);
        assert_eq!(fake.seen(command::CONNECT), 1);
        assert_eq!(
            fake.seen(command::SEND_BLOCK),
            blocks,
            "every block written"
        );
        assert_eq!(
            fake.seen(command::REQUEST_BLOCK),
            blocks,
            "every block verified"
        );
        assert_eq!(fake.seen(command::COMPLETE), 1);

        // The fake's store is the payload, block for block: a byte lost or
        // reordered anywhere in the sustained transfer would show here.
        assert_eq!(fake.memory().len(), blocks, "exactly one entry per block");
        assert_fake_stored(
            &fake,
            summary.start_addr,
            &payload,
            summary.block_size as usize,
        );

        stop.store(true, Ordering::Relaxed);
        pump.join().expect("the pump thread");
        link.close();
        drop(link);
        drop(firmware);
    }

    /// The defect the reset exists for, pinned: a **second** port declaration
    /// against the same bridge, with no reset in between, meets the state the
    /// first left — the firmware already holds its oid table and its port, and
    /// shuts down rather than rebuild them. That is the duplicate the tool's own
    /// second run hit; [`BridgeLink::reset`] is what clears it first, and
    /// [`test_two_runs_against_the_same_bridge_are_both_clean`] is the fix's
    /// side of the same fact.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_a_second_declaration_without_a_reset_shuts_the_bridge_down() {
        use crate::core::klippy::event::Shutdown;

        const BRIDGE_UUID: [u8; 6] = [0x91, 0x92, 0x93, 0x94, 0x95, 0x96];

        let port = Pty::new();
        let firmware =
            klipperx_test_support::LinuxFirmware::spawn_bridge_port(&port.path, BRIDGE_UUID)
                .expect("the bridge firmware process starts");
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

        // Capture the firmware's own reason the way the tool captures it.
        let reason: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
        let slot = Arc::clone(&reason);
        mcu.bind_event::<Shutdown, _>(move |event| {
            *slot.lock().unwrap_or_else(|poison| poison.into_inner()) = Some(event.reason);
        })
        .expect("the bridge's dictionary declares `shutdown`");

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
            .expect("the first declaration opens the port slot");

        // The second declaration, with no reset in between, meets what the
        // first left behind and the firmware shuts down.
        let second = open_bridge(&mcu, &link, &ports, target).await;
        // A successful return proves nothing: `open_bridge`'s flush only means
        // the bytes reached the transport, so the declaration is acked even
        // though the firmware is down — which is why a run has to watch for the
        // shutdown message instead of trusting this.
        assert!(
            second.is_ok(),
            "the second declaration is acknowledged, not refused: {second:?}"
        );

        let mut reported = None;
        for _ in 0..250 {
            if let Some(reason) = reason
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .clone()
            {
                reported = Some(reason);
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let reason = reported.expect("the bridge reported its shutdown");
        assert!(
            reason.contains("already"),
            "the firmware names the state the previous declaration left: {reason}"
        );

        mcu.close();
        drop(mcu);
        drop(link);
        drop(handle);
        drop(firmware);
    }

    /// Sustained transfer under repetition and changing size: four rounds, all
    /// through **one** bridge instance, so the reset, the declaration and the
    /// tunnel are rebuilt every round and the path is never warm for long. The
    /// sizes cross 40 KiB and include short final blocks; a small pause between
    /// rounds moves the transfer off whatever timing the previous round left.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_repeated_rounds_of_different_sizes_keep_every_byte() {
        const BRIDGE_UUID: [u8; 6] = [0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6];
        let sizes = [43 * 1024, 64, 40 * 1024 + 37, 7777];

        let port = Pty::new();
        let firmware =
            klipperx_test_support::LinuxFirmware::spawn_bridge_port(&port.path, BRIDGE_UUID)
                .expect("the bridge firmware process starts");
        let fake = FakeBootloader::new();
        let recorded = Arc::new(Mutex::new(Vec::<u8>::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let pump = start_fake_pump(&port, &fake, Arc::clone(&recorded), Arc::clone(&stop));

        let mut link = BridgeLink::open_serial(
            firmware.serial_path(),
            250_000,
            BRIDGE_MCU_ID,
            Some(BRIDGE_UUID),
        )
        .expect("the bridge's console attaches and is named");
        tokio::time::timeout(Duration::from_secs(20), link.identify(IDENTIFY_TIMEOUT))
            .await
            .expect("the identify handshake timed out")
            .expect("the linux bridge identifies");

        let ports = [BridgePort {
            baud: 250_000,
            reset: PortReset::ResetPort,
        }];
        let target = BridgeTarget {
            index: 0,
            node_id: TARGET_NODE_ID,
        };
        let mut blocks = 0usize;
        for (round, size) in sizes.iter().enumerate() {
            let payload: Vec<u8> = (0..*size).map(|i| ((i * 7 + round) % 256) as u8).collect();
            let expected_blocks = (payload.len() + 63) / 64;
            let summary = link
                .flash(&ports, target, &payload, &test_config(), |_| {})
                .await
                .unwrap_or_else(|e| panic!("round {round} ({size} bytes): {e}"));
            assert_eq!(summary.bytes_flashed, *size, "round {round}");
            assert_eq!(summary.block_count, expected_blocks, "round {round}");
            assert_fake_stored(
                &fake,
                summary.start_addr,
                &payload,
                summary.block_size as usize,
            );
            blocks += expected_blocks;
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
        assert_eq!(
            fake.seen(command::CONNECT),
            sizes.len(),
            "one session per round"
        );
        assert_eq!(
            fake.seen(command::SEND_BLOCK),
            blocks,
            "every block of every round reached the device"
        );
        assert_eq!(fake.seen(command::REQUEST_BLOCK), blocks);
        assert_eq!(fake.seen(command::COMPLETE), sizes.len());

        stop.store(true, Ordering::Relaxed);
        pump.join().expect("the pump thread");
        link.close();
        drop(link);
        drop(firmware);
    }

    /// A bridge→host answer to the discovery broadcast, as the firmware's
    /// `mux_need_nodeid` builds it and `klipperx admin scan --port` decodes: the admin
    /// reply header, then the `KXID` frame carrying [`RESP_NEED_NODEID`] and the
    /// six uuid bytes ([`serial::RESP_NEED_NODEID`], no application byte).
    fn unassigned_answer(uuid: [u8; 6]) -> Vec<u8> {
        use crate::core::klippy::frame::Frame;
        let mut body = serial::MAGIC.to_vec();
        body.push(serial::RESP_NEED_NODEID);
        body.extend_from_slice(&uuid);
        let mut unit = serial::encode_header(serial::ADMIN_REPLY);
        unit.extend_from_slice(&Frame::with_wire_seq(serial::ASSIGN_SEQ_BYTE, body).raw_bytes());
        unit
    }

    /// Two runs against one bridge with **no** `--bridge-uuid`: each run restarts
    /// the firmware, which drops its node id, finds it again with the discovery
    /// scan, re-assigns it, and flashes. That is the shape a re-used bridge takes
    /// when the caller only knows its tty — and the reassignment is proven by the
    /// session running over the tunnel at all, since the tunnel's route needs the
    /// id the scan found.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_two_runs_against_the_same_bridge_without_a_uuid() {
        const BRIDGE_UUID: [u8; 6] = [0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6];

        let port = Pty::new();
        let firmware =
            klipperx_test_support::LinuxFirmware::spawn_bridge_port(&port.path, BRIDGE_UUID)
                .expect("the bridge firmware process starts");
        let fake = FakeBootloader::new();
        let recorded = Arc::new(Mutex::new(Vec::<u8>::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let pump = start_fake_pump(&port, &fake, Arc::clone(&recorded), Arc::clone(&stop));

        // The bridge's id, as a run that had `--bridge-uuid` would have left it:
        // the runs below do not know it and have to find it again.
        drop(
            serial::attach(
                firmware.serial_path(),
                250_000,
                BRIDGE_MCU_ID,
                Duration::from_millis(500),
                Some(BRIDGE_UUID),
            )
            .expect("the bridge adopts its node id"),
        );

        let mut link =
            BridgeLink::open_serial(firmware.serial_path(), 250_000, BRIDGE_MCU_ID, None)
                .expect("the assigned bridge answers the re-confirmation");
        tokio::time::timeout(Duration::from_secs(20), link.identify(IDENTIFY_TIMEOUT))
            .await
            .expect("the identify handshake timed out")
            .expect("the linux bridge identifies");

        let ports = [BridgePort {
            baud: 250_000,
            reset: PortReset::ResetPort,
        }];
        let target = BridgeTarget {
            index: 0,
            node_id: TARGET_NODE_ID,
        };
        let payload: Vec<u8> = (0..100u8).collect();

        let first = link
            .flash(&ports, target, &payload, &test_config(), |_| {})
            .await
            .expect("the first run, after finding the bridge by scan");
        let second = link
            .flash(&ports, target, &payload, &test_config(), |_| {})
            .await
            .expect("the second run, after finding the bridge by scan again");

        // The two runs' own numbers, and the firmware's report of both port
        // opens: two restarts, two re-declarations, two assignments.
        println!(
            "two runs without a uuid:\n  first:  {first:?}\n  second: {second:?}\n\
             firmware stderr:\n{}",
            firmware.stderr()
        );

        for summary in [&first, &second] {
            assert_eq!(summary.start_addr, 0x0800_2000);
            assert_eq!(summary.block_size, 64);
            assert_eq!(summary.mcu_type, "STM32F103");
            assert_eq!(summary.bytes_flashed, payload.len());
            assert_eq!(summary.block_count, 2);
        }
        assert_eq!(fake.seen(command::CONNECT), 2, "one session per run");
        assert_eq!(fake.seen(command::COMPLETE), 2);

        stop.store(true, Ordering::Relaxed);
        pump.join().expect("the pump thread");
        link.close();
        drop(link);
        drop(firmware);
    }

    /// A tty that answers nothing to the discovery scan names no bridge: the
    /// scan is a bounded [`InterfaceError::Timeout`], which the reopen retries
    /// (the bridge may still be coming back up), not a guess.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_a_discovery_scan_with_no_answer_is_a_timeout() {
        let pty = Pty::new();
        let err =
            discover_bridge_uuid(&pty.path, 250_000).expect_err("a silent tty names no bridge");
        assert!(
            matches!(err, BridgeFlashError::Open(InterfaceError::Timeout)),
            "{err}"
        );
    }

    /// Two unassigned bridges on one line and no `--bridge-uuid`: naming either
    /// would restart and re-assign the other, so the run refuses to guess.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_a_discovery_scan_with_two_bridges_refuses_to_guess() {
        let pty = Pty::new();
        let mut master = pty.master.try_clone().expect("the pty master clones");
        let responder = std::thread::spawn(move || {
            // Bounded: the scan sends its broadcast at once, so a poll that runs
            // out means the fixture never got it and the join must not hang.
            let mut poll = libc::pollfd {
                fd: master.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            };
            if unsafe { libc::poll(&mut poll, 1, 5_000) } <= 0 {
                return;
            }
            let mut query = [0u8; 256];
            if master.read(&mut query).unwrap_or(0) == 0 {
                return;
            }
            master
                .write_all(&unassigned_answer([0x01, 0x02, 0x03, 0x04, 0x05, 0x06]))
                .expect("the first bridge answers");
            master
                .write_all(&unassigned_answer([0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f]))
                .expect("the second bridge answers");
        });

        let err =
            discover_bridge_uuid(&pty.path, 250_000).expect_err("two bridges cannot be told apart");
        assert!(
            matches!(err, BridgeFlashError::AmbiguousBridge { count: 2, .. }),
            "{err}"
        );
        assert!(err.to_string().contains("--bridge-uuid"), "{err}");
        responder.join().expect("the responder thread");
    }

    /// The **first** open with no `--bridge-uuid` finds an unassigned bridge:
    /// the firmware is spawned with an identity but no node id, so the plain
    /// re-confirmation would time out. The discovery broadcast is what names it
    /// ([`attach_bridge`]'s first half), and the identify that follows proves
    /// the bridge was adopted and reached — the flag is an optional override.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_the_first_open_without_a_uuid_finds_an_unassigned_bridge() {
        const BRIDGE_UUID: [u8; 6] = [0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6];

        let port = Pty::new();
        let firmware =
            klipperx_test_support::LinuxFirmware::spawn_bridge_port(&port.path, BRIDGE_UUID)
                .expect("the bridge firmware process starts");

        // Nothing names the bridge first: it carries no node id, so only the
        // discovery broadcast reaches it.
        let link = BridgeLink::open_serial(firmware.serial_path(), 250_000, BRIDGE_MCU_ID, None)
            .expect("the discovery scan finds and names the unassigned bridge");
        tokio::time::timeout(Duration::from_secs(20), link.identify(IDENTIFY_TIMEOUT))
            .await
            .expect("the identify handshake timed out")
            .expect("the found bridge identifies");

        link.close();
        drop(link);
        drop(firmware);
    }

    /// Session timing for the bootloader-retry cases: the same shape as
    /// [`test_config`], millisecond scale so a `CONNECT` that no one answers
    /// and the wait after it stay fast — the retry count is the session's
    /// `tries`, so shrinking the budget shrinks the whole bounded wait.
    fn bootloader_test_config() -> SessionConfig {
        SessionConfig {
            tries: 3,
            read_timeout: Duration::from_millis(100),
            retry_delay: Duration::from_millis(10),
            busy_delay: Duration::from_millis(10),
        }
    }

    /// A downstream that has not come up yet: the first `mute` frames written
    /// to it are swallowed and never answered, everything after them reaches
    /// `inner`. Each swallowed frame is the session's `CONNECT`, because a
    /// session that got no answer to it never writes anything else.
    struct NotReadyYet<W> {
        inner: W,
        /// Frames left to swallow; the count the test reads back.
        swallowed: usize,
        /// How many of `swallowed` are still to come.
        remaining: usize,
    }

    impl<W> NotReadyYet<W> {
        fn new(inner: W, mute: usize) -> Self {
            Self {
                inner,
                swallowed: 0,
                remaining: mute,
            }
        }
    }

    impl<W: Write> Write for NotReadyYet<W> {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            if self.remaining > 0 {
                self.remaining -= 1;
                self.swallowed += 1;
                return Ok(buf.len());
            }
            self.inner.write(buf)
        }

        fn flush(&mut self) -> std::io::Result<()> {
            self.inner.flush()
        }
    }

    /// A target whose plan reset is [`PortReset::Bootloader`] was just sent
    /// back to its bootloader, so the session's first `CONNECT`s reach nothing
    /// while it comes up: the run waits and tries again, and the first session
    /// that gets past `CONNECT` is the one that finishes.
    #[tokio::test]
    async fn test_a_bootloader_sent_target_is_retried_until_connect_answers() {
        let fake = FakeBootloader::new();
        let mut reader = fake.reader();
        let cfg = bootloader_test_config();
        // The device is still starting: one session's worth of CONNECTs goes
        // unanswered, and the next session's reaches the bootloader.
        let mut writer = NotReadyYet::new(fake.writer(), cfg.tries as usize);

        let ports = [BridgePort {
            baud: 250_000,
            reset: PortReset::Bootloader,
        }];
        let target = BridgeTarget {
            index: 0,
            node_id: TARGET_NODE_ID,
        };
        let payload: Vec<u8> = (0..100u8).collect();
        let summary = run_session(
            &ports,
            target,
            &mut reader,
            &mut writer,
            &payload,
            &cfg,
            |_| {},
        )
        .await
        .expect("the session after the silent one gets its CONNECT answered");

        assert_eq!(summary.bytes_flashed, payload.len());
        assert_eq!(summary.block_count, 2, "100 bytes is two 64-byte blocks");
        assert_eq!(
            writer.swallowed, cfg.tries as usize,
            "the first session spent its tries on a device that was not up yet"
        );
        assert_eq!(
            fake.seen(command::CONNECT),
            1,
            "the retried session's CONNECT is the one that reached the bootloader"
        );
        assert_eq!(
            fake.seen(command::COMPLETE),
            1,
            "one session ran to the end"
        );
    }

    /// A bootloader-sent target that never answers is still bounded: exactly
    /// `tries` sessions of `tries` `CONNECT`s each, and the session's own error
    /// (the one a plain reset would have given on the first session).
    #[tokio::test]
    async fn test_a_bootloader_sent_target_that_never_answers_keeps_the_session_error() {
        let fake = FakeBootloader::new();
        let mut reader = fake.reader();
        let cfg = bootloader_test_config();
        let attempts = cfg.tries as usize;
        let mut writer = NotReadyYet::new(fake.writer(), attempts * attempts);

        let ports = [BridgePort {
            baud: 250_000,
            reset: PortReset::Bootloader,
        }];
        let target = BridgeTarget {
            index: 0,
            node_id: TARGET_NODE_ID,
        };
        let err = run_session(
            &ports,
            target,
            &mut reader,
            &mut writer,
            &[0u8; 8],
            &cfg,
            |_| {},
        )
        .await
        .expect_err("nothing ever answers CONNECT");

        assert!(
            matches!(
                err,
                FlashError::RetriesExhausted {
                    cmd: CONNECT,
                    tries,
                    ..
                } if tries as usize == attempts
            ),
            "the session's own CONNECT error, unchanged: {err}"
        );
        assert_eq!(
            writer.swallowed,
            attempts * attempts,
            "one session per try, each spending its own tries"
        );
        assert_eq!(fake.seen(command::CONNECT), 0, "nothing got past the wire");
    }

    /// Any other reset keeps the single session: a silent `CONNECT` spends the
    /// session's tries once and fails, with no second attempt behind it.
    #[tokio::test]
    async fn test_a_plain_reset_gets_one_session_with_no_retry() {
        let fake = FakeBootloader::new();
        let mut reader = fake.reader();
        let mut writer = fake.writer();
        let cfg = bootloader_test_config();
        // The fake's silence is permanent for the command, which is exactly the
        // device a retry cannot help.
        fake.script(command::CONNECT, 1, Quirk::Silent);

        let ports = [BridgePort {
            baud: 250_000,
            reset: PortReset::ResetPort,
        }];
        let target = BridgeTarget {
            index: 0,
            node_id: TARGET_NODE_ID,
        };
        let err = run_session(
            &ports,
            target,
            &mut reader,
            &mut writer,
            &[0u8; 8],
            &cfg,
            |_| {},
        )
        .await
        .expect_err("nothing answers CONNECT");

        assert!(
            matches!(err, FlashError::RetriesExhausted { cmd: CONNECT, .. }),
            "the session's own CONNECT error: {err}"
        );
        assert_eq!(
            fake.seen(command::CONNECT),
            cfg.tries as usize,
            "one session's tries and no retry behind them"
        );
    }
}
