//! The bridge half of a flash run: declare the bridge's downstream ports, reset
//! their devices, and open the flashing tunnel on the port the target sits
//! behind.
//!
//! A board reached through a bridge is flashed over the **bridge's** own link,
//! and the bridge's port table lives in its RAM only — `config_uart` opens a
//! port's stream and `config_bridge_port` hands it to the router, both at
//! configuration time (`third_party/klipper/src/uart.c`, `bridge.c`). So a run
//! that follows a reset has to build that table again before any port-level
//! command can name a port. [`open_bridge`] is that front half, in the order the
//! firmware requires:
//!
//! 1. **declare** every port — `allocate_oids` for the object ids, then the
//!    stream command the port's bus takes ([`PortKind`]: `config_uart` on a UART
//!    slot, `config_can` on the CAN one) plus a `config_bridge_port`
//!    ([`crate::core::klippy::cmd::uart`]). Both commands take effect as they
//!    arrive: the stream is opened, the port is hung on the router and numbered
//!    by arrival order. There is no `finalize_config` in this flow; these
//!    commands alone are what initializes a bridge for a flash run.
//! 2. **reset the target's device**, the way the plan says ([`PortReset`]) —
//!    written at the line for a device with no admin channel, restarted through
//!    it for one that has, and for a device being sent into its bootloader a
//!    route of its own: units for the request, legacy again before the tunnel
//!    ([`request_bootloader`]). The plan's other ports are declared and left
//!    alone: only the port the run targets is reset.
//! 3. **name the target** and **open its tunnel** — `SET_PORT_NODEID` with no
//!    port flags (a *legacy* route) and `SET_PORT_TUNNEL`, the toggle that makes
//!    the port carry the katapult raw byte stream instead of mux units.
//!
//! What runs *over* that tunnel — the katapult session itself
//! ([`super::session`]) — is not this module's. [`open_bridge`] stops once the
//! target's port is ready for it, and the [`BridgeReady`] it returns records
//! what was done.
//!
//! Every step is a command on one link. The declaration commands are
//! [`Mcu::send`]'s — the bridge's own connection, whose dictionary the identify
//! handshake installed, which is also what owns the frame sequence — and the
//! port-level ones are the [`Interface`]'s (`send_set_port_node_id`,
//! `send_restart_device`, `send_reset_port`, `send_set_port_tunnel`), which
//! spell the admin unit for whichever wire the link is. `mcu` and `link` are two
//! handles on that one transport. Nothing here probes what is behind a port — a
//! bridge link carries no such probe — so which reset a port gets, and which
//! port the target is, are the caller's to say.

use crate::core::klippy::cmd::allocate_oids::AllocateOids;
use crate::core::klippy::cmd::uart::{ConfigBridgePort, PortConfig};
use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::interface::devices::serial::MUX_PORT_UNITS;
use crate::core::klippy::interface::{Interface, InterfaceError};
use crate::core::klippy::mcu::{Dictionary, Mcu, McuError};
use crate::core::klippy::msg::MsgError;
use std::fmt;
use std::time::Duration;

/// The port flags a target is named with: none at all — a **legacy** route.
///
/// Not a default. The katapult stream a flash run reads is the bridge's raw
/// uplink, and that path serves a legacy port only: a `MUX_PORT_UNITS` port
/// drops a raw frame without a word (`bridge_port_raw_up` in
/// `third_party/klipper/src/bridge.c`), which is the stream the tunnel carries.
/// A units route would therefore swallow it in silence, so a flashing target is
/// named this one way.
const LEGACY_PORT_FLAGS: u8 = 0;

/// The port flags a target is named with while its device is asked to enter its
/// bootloader: a **units** route, the only one that delivers that command.
///
/// The request goes down as an admin unit. A legacy port has the bridge strip
/// the mux header off it, and the device's parser then rejects the bare `KXID`
/// frame — the marker's sequence byte is not one it looks for — so a request
/// written onto a legacy route never arrives. A units port forwards the unit
/// whole, header included, and the device's admin channel acts on it
/// (`bridge_forward_to_port` in `third_party/klipper/src/bridge.c`).
///
/// Only that one request travels a units route, and only for a moment:
/// [`request_bootloader`] names the port back to a legacy route before it
/// returns, because the tunnel's uplink needs one.
const UNITS_PORT_FLAGS: u8 = MUX_PORT_UNITS;

/// How a port is named: which wire the device behind it speaks.
///
/// The host's call, never a build-time one (`mux.h` MUX_PORT_UNITS):
/// [`PortRoute::Units`] is a device that speaks the mux (the route a device is
/// **enabled** with — it is addressed by id and its uplink is forwarded whole),
/// and [`PortRoute::Legacy`] a device that predates it, whose raw frames the
/// port heads with its own reply id. A flash run names its target
/// [`PortRoute::Legacy`] because the katapult tunnel reads the raw uplink
/// ([`LEGACY_PORT_FLAGS`]); a stock katapult board behind a port is the same
/// case, which is why `admin bridge port --legacy` keeps it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortRoute {
    /// The device speaks units: `MUX_PORT_UNITS` set.
    Units,
    /// The device predates the mux: no flags.
    Legacy,
}

impl PortRoute {
    /// The `flags` byte [`Interface::send_set_port_node_id`] carries.
    fn flags(self) -> u8 {
        match self {
            Self::Units => UNITS_PORT_FLAGS,
            Self::Legacy => LEGACY_PORT_FLAGS,
        }
    }
}

/// How long a port is left in units mode after the bootloader request — time for
/// the device to write its bootloader marker and reset into katapult.
///
/// Nothing answers in that window: the device is leaving its firmware, so there
/// is no command to wait on. Nor is there a probe to wait on — a bridge link
/// carries none for what sits behind a port, so re-identifying the *bridge*
/// would only prove the bridge is alive, at the price of restarting the frame
/// sequence [`Mcu`] owns. A plain sleep is therefore the whole observable, and
/// the session's own `CONNECT` retries
/// ([`BridgeLink::flash`](crate::flash::bridge_flash::BridgeLink::flash))
/// cover a device that needs longer than this.
const BOOTLOADER_ENTRY_WAIT: Duration = Duration::from_secs(5);

/// The most ports one session can declare: two oids per port (its UART object
/// and the port object), and both counts travel in `allocate_oids count=%c`,
/// one byte.
const MAX_BRIDGE_PORTS: usize = u8::MAX as usize / 2;

/// How long the declaration barrier waits for the queued commands to reach the
/// transport before the session gives up.
///
/// The declaration has to be on the wire before any port-level command — the
/// bridge builds its port table from it, and a reset that names a port the
/// bridge does not have is dropped in silence — but [`Mcu::send`] only queues.
/// [`Mcu::flush`] is the barrier that makes the boundary real
/// ([`flush`](Mcu::flush)); this is how long it may take.
const DECLARE_FLUSH_TIMEOUT: Duration = Duration::from_secs(5);

/// How one port's device is taken back to its bootloader.
///
/// Which one a port gets is the caller's to say, and it is not guessable from
/// here: a bridge link carries no probe for what sits behind a port, and both
/// resets are silent — no receipt — so a session that picked one for itself
/// could not tell whether it picked right. The two are the ones the firmware
/// serves, and they do not substitute for each other:
///
/// - [`PortReset::RestartDevice`] asks the device to restart its **running
///   firmware**. Only a device that answers the bridge's admin channel of its
///   own acts on it; the bridge forwards the command whole for such a port and
///   strips the header for a legacy one (`bridge_forward_to_port` in
///   `third_party/klipper/src/bridge.c`), so a port whose device has no admin
///   channel is reset at the line instead.
/// - [`PortReset::ResetPort`] is the bridge pulsing the port's **lines**, which
///   is the reset a device with no admin channel can be reached by at all. The
///   bridge performs it and nothing goes downstream. It is also the fallback
///   that needs no answer: a pulse, then a wait for the device to come back, is
///   what is left when what is behind the port is unknown — or when the target
///   is already sitting in its bootloader.
/// - [`PortReset::Bootloader`] asks the device to **enter its bootloader**
///   (`CMD_REQUEST_BOOTLOADER`): it writes the bootloader-request marker and
///   resets, so it comes back in katapult instead of the firmware that was
///   running. That is what makes a flash run reachable from a running
///   application — the other two resets only reload or pulse what is there. A
///   device without a bootloader request (an older katapult, a board whose
///   `HAVE_BOOTLOADER_REQUEST` is off) ignores it by design, which is why the
///   flash still has to wait for the device to answer `CONNECT`
///   ([`flash`](crate::flash::bridge_flash::BridgeLink::flash)).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortReset {
    /// The named device's own restart (`CMD_RESTART_DEVICE`); needs a device
    /// that answers the admin channel.
    RestartDevice,
    /// The bridge's line pulse on the port (`CMD_RESET_PORT`); the reset for a
    /// legacy device, for a target sitting in its bootloader, and the fallback
    /// for a port whose device is unknown.
    ResetPort,
    /// The named device entering its bootloader (`CMD_REQUEST_BOOTLOADER`);
    /// needs a device that answers the admin channel, and it comes back as a
    /// katapult that takes time to answer. The request only travels a units
    /// route, so [`reset_ports`] sends it through one and names the port back
    /// to a legacy route afterwards ([`request_bootloader`]).
    Bootloader,
}

/// Which bus a bridge slot carries, and so which command opens its stream.
///
/// The bridge's slot order is the arch dispatcher's: **every UART slot first, the
/// CAN slot after them** (`third_party/klipper/src/stm32/bridge_port.c`, whose
/// `BRIDGE_UART_COUNT` is "UART slots + CAN slots" added up). Which kind a slot is
/// decides the one command that opens its stream ([`slot_kind`] reads it from the
/// firmware's own declarations); everything else in a declaration — the oid pair,
/// `config_bridge_port`, the slot number — is the same for both.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortKind {
    /// A serial slot: opened with `config_uart` at [`BridgePort::baud`].
    Uart,
    /// The CAN bus slot: opened with `config_can` at [`BridgePort::bitrate`].
    Can,
}

impl PortKind {
    /// The stream command that opens this kind of port at the plan's rate.
    fn stream_config(self, port: &BridgePort) -> PortConfig {
        match self {
            Self::Uart => PortConfig::Uart { baud: port.baud },
            Self::Can => PortConfig::Can {
                bitrate: port.bitrate,
            },
        }
    }
}

/// Which bus the bridge's slot `index` carries, read from the two constants the
/// firmware publishes for its slots.
///
/// `RESERVE_PINS_bridge_can` exists only in a build that has a CAN downstream
/// port (`third_party/klipper/src/{stm32/bridge_port.c,stm32/can.c}` — the same
/// constant `admin bridge config` names that slot from), and `BRIDGE_UART_COUNT`
/// is **every** slot the arch numbers. The CAN slot is the last of them, so its
/// number is that count minus one, and every slot below it is a UART slot. A
/// build with no CAN slot (the linux bridge, the host library) publishes no such
/// constant and every slot of it is a UART slot.
///
/// Everything else reads as [`PortKind::Uart`]: a firmware that names no CAN slot,
/// a slot past the ones it numbered (the CAN slot included), and a connection
/// whose dictionary never arrived — nothing said otherwise. `admin bridge config`
/// lists those slots in the same order ([`BridgeLink::uart_slots`]).
///
/// [`BridgeLink::uart_slots`]: crate::flash::bridge_flash::BridgeLink::uart_slots
pub(crate) fn slot_kind(dictionary: Option<&Dictionary>, index: u8) -> PortKind {
    let Some(dictionary) = dictionary else {
        return PortKind::Uart;
    };
    let has_can_slot = dictionary
        .constant("RESERVE_PINS_bridge_can")
        .and_then(|pins| pins.as_str())
        .is_some();
    let Some(total) = dictionary
        .constant("BRIDGE_UART_COUNT")
        .and_then(|count| count.as_u64())
    else {
        return PortKind::Uart;
    };
    let can_slot = total.saturating_sub(1);
    if has_can_slot && u64::from(index) == can_slot {
        PortKind::Can
    } else {
        PortKind::Uart
    }
}

/// One port of a bridge's downstream plan.
///
/// The plan's **order is the port numbering**: `config_bridge_port` makes the
/// bridge number a port by the order the command arrives, and that number is
/// also the slot the stream command carries and the number the host named the
/// port's tty with before the bridge came up (`third_party/klipper/src/uart.c`,
/// `mcu/bridge_ports.rs`). So the i-th entry of the slice [`open_bridge`] is
/// given is port `i` — the index every port-level command addresses. Which kind
/// of port that entry is comes from the bridge's own slot table ([`slot_kind`]),
/// not from the plan: the two rates below are carried so that either kind can be
/// declared from one entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BridgePort {
    /// The line rate the bridge opens a **UART** slot's stream at — the port
    /// section's `uart_baud:`, or
    /// [`DEFAULT_UART_BAUD`](crate::core::klippy::cmd::uart::DEFAULT_UART_BAUD)
    /// when it wrote none. A CAN slot ignores it.
    pub baud: u32,
    /// The bus rate the bridge opens the **CAN** slot at, in bits per second —
    /// `--can-bitrate`.
    ///
    /// The port's bitrate is the `config_can` rate and nothing else: a CAN port's
    /// stream *is* the bus, so this is where `canhw_set_rate` gets its number.
    /// A UART slot ignores it.
    pub bitrate: u32,
    /// Which reset this port's device gets before the tunnel opens — read for
    /// the entry the plan **targets** and for no other, since [`reset_ports`]
    /// resets the target's port alone. The entries below the target are there
    /// for the declaration and carry it unused, as in [`declare_ports`].
    pub reset: PortReset,
}

/// The port a flash run targets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BridgeTarget {
    /// The target's index: the plan entry whose port its device sits behind.
    pub index: u8,
    /// The id the target's device answers to.
    pub node_id: u8,
}

/// What a bridge session did, for the caller to report and to continue from.
///
/// The bridge's own frame sequence is not here: [`Mcu`] owns it now, so a
/// caller that sends anything else on that connection goes through the same
/// [`Mcu::send`] and the count stays with it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BridgeReady {
    /// The plan as it was carried out: every port declared, in index order.
    pub ports: Vec<BridgePort>,
    /// The target as it was named.
    pub target: BridgeTarget,
}

/// Everything that can stop a bridge session.
#[derive(Debug)]
pub enum BridgeError {
    /// The link refused a port-level write — a closed handle, a failed device
    /// write, or a link that carries no port mux units at all.
    Link(InterfaceError),
    /// The bridge's own connection refused the command: the dictionary cannot
    /// spell the name, the arguments do not match, the send queue is full, or
    /// the declaration barrier was not reached in time.
    Mcu(McuError),
    /// The plan names more ports than one `allocate_oids count=%c` can number
    /// (two oids per port), so nothing was sent.
    TooManyPorts {
        /// How many ports the plan asked for.
        ports: usize,
    },
}

impl fmt::Display for BridgeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BridgeError::Link(e) => write!(f, "bridge link: {e}"),
            BridgeError::Mcu(e) => write!(f, "bridge command: {e}"),
            BridgeError::TooManyPorts { ports } => write!(
                f,
                "{ports} ports ask for more oids than allocate_oids can number \
                 (at most {MAX_BRIDGE_PORTS})"
            ),
        }
    }
}

impl std::error::Error for BridgeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            BridgeError::Link(e) => Some(e),
            BridgeError::Mcu(e) => Some(e),
            BridgeError::TooManyPorts { .. } => None,
        }
    }
}

impl From<InterfaceError> for BridgeError {
    fn from(e: InterfaceError) -> Self {
        BridgeError::Link(e)
    }
}

impl From<McuError> for BridgeError {
    fn from(e: McuError) -> Self {
        BridgeError::Mcu(e)
    }
}

/// [`Mcu::send`]'s error lifts into the connection variant unchanged.
impl From<MsgError> for BridgeError {
    fn from(e: MsgError) -> Self {
        BridgeError::Mcu(McuError::Msg(e))
    }
}

/// Declare `ports` on the bridge, reset their devices, and open the tunnel on
/// `target` — the flashing prerequisites of a bridge-port flash, in one call.
///
/// `mcu` is the bridge's own connection: its identify handshake installed the
/// dictionary the declaration commands are encoded from, and it owns the frame
/// sequence, so nothing here counts bytes. `link` is the same bridge seen as its
/// port-level channel ([`Interface`]), which spells the admin unit for whichever
/// wire the link is. `ports` and `target` are the caller's plan.
///
/// The three steps and their order are in the module docs. Nothing is sent
/// until the plan has been checked, the declaration is flushed onto the wire
/// before any port-level command, and everything is sent in the order the
/// firmware needs, so a session either runs to the end or fails with the bridge
/// as it was.
///
/// The target's [`PortReset::Bootloader`] costs the session
/// [`BOOTLOADER_ENTRY_WAIT`] on top: its device has to leave its firmware before
/// the tunnel opens ([`request_bootloader`]).
///
/// # Errors
/// [`BridgeError::TooManyPorts`] when the plan is wider than the oid byte can
/// number, [`BridgeError::Mcu`] when the connection cannot spell or flush a
/// command, and [`BridgeError::Link`] when the link refuses a port-level write.
pub async fn open_bridge(
    mcu: &Mcu,
    link: &Interface,
    ports: &[BridgePort],
    target: BridgeTarget,
) -> Result<BridgeReady, BridgeError> {
    declare_ports(mcu, ports).await?;
    reset_ports(link, ports, target, None).await?;
    open_target_port(link, target, PortRoute::Legacy)?;
    Ok(BridgeReady {
        ports: ports.to_vec(),
        target,
    })
}

/// Declare `ports` on the bridge: the oid table, then one stream-and-port pair
/// per port in index order — the bridge numbers a port by the order
/// `config_bridge_port` reaches it — then the barrier that makes the
/// declaration real.
///
/// The declaration has to be on the wire before any port-level command: the
/// bridge builds its port table from it, and a command that names a port it
/// does not have is dropped in silence. `admin bridge config` is the standalone
/// caller: it declares the table and stops.
///
/// # Errors
/// [`BridgeError::TooManyPorts`] when the plan is wider than the oid byte can
/// number (nothing is sent), [`BridgeError::Mcu`] when a command cannot be
/// spelled or the barrier is not reached in time.
pub async fn declare_ports(mcu: &Mcu, ports: &[BridgePort]) -> Result<(), BridgeError> {
    if ports.len() > MAX_BRIDGE_PORTS {
        return Err(BridgeError::TooManyPorts { ports: ports.len() });
    }

    let dictionary = mcu.dictionary();
    let count = (ports.len() * 2) as u8;
    mcu.send(AllocateOids::NAME, &AllocateOids { count }.args())?;
    for (index, port) in ports.iter().enumerate() {
        send_declaration(mcu, index as u8, port, dictionary.as_deref())?;
    }
    mcu.flush(DECLARE_FLUSH_TIMEOUT).await?;
    Ok(())
}

/// Send one port's declaration pair: the stream command its bus takes, then the
/// `config_bridge_port` that hangs it on the bridge's router.
///
/// `index` is the port's number — the slot the stream command carries, and the
/// number the bridge assigns it by arrival order — so the oid pair is `index * 2`
/// for the stream object and `index * 2 + 1` for the port that carries it.
/// `dictionary` is the bridge's, whose constants say which bus the slot is
/// ([`slot_kind`]); a connection with none declares every port as a UART, since
/// nothing said otherwise.
fn send_declaration(
    mcu: &Mcu,
    index: u8,
    port: &BridgePort,
    dictionary: Option<&Dictionary>,
) -> Result<(), McuError> {
    let stream_oid = index * 2;
    let (name, args) = slot_kind(dictionary, index)
        .stream_config(port)
        .stream_command(stream_oid, index);
    mcu.send(name, &args)?;
    let bridge_port = ConfigBridgePort {
        oid: stream_oid + 1,
        port: stream_oid,
    };
    mcu.send(ConfigBridgePort::NAME, &bridge_port.args())?;
    Ok(())
}

/// Declare **one** port at `index` the way [`declare_ports`] declares the
/// plan's first `index + 1` — for a caller whose declarations are scattered by
/// work in between (a bridge scan declares a port, scans through it, then
/// declares the next).
///
/// The bridge numbers a port by the order `config_bridge_port` reaches it, so
/// ports are declared in ascending index from 0 with nothing else allocating
/// oids in between; `index` is that order's next free number, and the oid pair
/// follows [`declare_ports`]'s scheme (`index * 2` the stream object, `index * 2
/// + 1` the port that carries it). `port` is that port's plan entry — its two
/// rates, of which the one its bus takes is used; the entry's reset is unused
/// here, as in [`declare_ports`].
///
/// # Errors
/// [`BridgeError::TooManyPorts`] when `index` is past what the oid byte can
/// number (nothing is sent), [`BridgeError::Mcu`] when a command cannot be
/// spelled or the barrier is not reached in time.
pub async fn declare_port(mcu: &Mcu, index: u8, port: BridgePort) -> Result<(), BridgeError> {
    if usize::from(index) >= MAX_BRIDGE_PORTS {
        return Err(BridgeError::TooManyPorts {
            ports: usize::from(index) + 1,
        });
    }
    let dictionary = mcu.dictionary();
    mcu.send(AllocateOids::NAME, &AllocateOids { count: 2 }.args())?;
    send_declaration(mcu, index, &port, dictionary.as_deref())?;
    mcu.flush(DECLARE_FLUSH_TIMEOUT).await?;
    Ok(())
}
/// Reset the **target's** port's device — [`open_bridge`]'s step 2, after
/// [`declare_ports`] so the command addresses a port the bridge has.
///
/// Only the entry the plan **targets** is reset. The ports below it carry the
/// declaration and nothing else: the bridge numbers a port by the order
/// `config_bridge_port` reaches it, so a target at `index` needs the ports
/// under it declared — but a caller's plan says what to do with the device it
/// means to reach, not with the devices on the way there, and a line pulse is
/// not something to send to hardware nobody named.
///
/// The target's [`PortReset::Bootloader`] is the one case that cannot be one
/// unit ([`request_bootloader`]), and it is why `target` is here rather than
/// the reset alone: naming the port back to legacy takes the id its device
/// answers to, and a plan carries an id only for the target.
///
/// # Errors
/// [`BridgeError::Link`] when the link refuses a port-level write.
pub async fn reset_ports(
    link: &Interface,
    ports: &[BridgePort],
    target: BridgeTarget,
    uuid: Option<[u8; 6]>,
) -> Result<(), BridgeError> {
    let Some(port) = ports.get(usize::from(target.index)) else {
        // A plan with no entry for the target names no port to reset; the
        // declaration refused it, or [`open_target_port`] will.
        return Ok(());
    };
    match port.reset {
        PortReset::Bootloader => {
            request_bootloader(link, target.index, target.node_id, uuid).await?
        }
        reset => reset_port(link, target.index, reset, uuid)?,
    }
    Ok(())
}

/// Reset one declared port's device the way `reset` says: a line pulse on the
/// port ([`PortReset::ResetPort`]), the device's own restart
/// ([`PortReset::RestartDevice`]), or its entry into the bootloader
/// ([`PortReset::Bootloader`]) — one admin unit each. `admin reset --port` is the
/// caller that has an index and no id, and this is that call.
///
/// The bootloader unit here is the **bare** one, and a legacy route strips it on
/// its way down, so it only arrives on a port already named units. Naming a port
/// is what an id is needed for, and `admin reset --port` has none: its
/// `--port-reset bootloader` therefore reaches a device only behind a port a
/// previous `admin bridge port` (or flash run) named units, and it cannot put the
/// route back to legacy. [`reset_ports`] takes the whole route instead
/// ([`request_bootloader`]).
///
/// The port has to be one the bridge has **declared** — the firmware drops a
/// port-level command that names a port it does not have, in silence.
///
/// # Errors
/// [`BridgeError::Link`] when the link refuses a port-level write.
pub fn reset_port(
    link: &Interface,
    index: u8,
    reset: PortReset,
    uuid: Option<[u8; 6]>,
) -> Result<(), BridgeError> {
    match reset {
        PortReset::RestartDevice => link.send_restart_device(index)?,
        PortReset::ResetPort => link.send_reset_port(index)?,
        PortReset::Bootloader => link.send_request_bootloader(index, uuid)?,
    }
    Ok(())
}

/// Ask the device behind `index` to enter its bootloader, through the route that
/// can carry the request — [`PortReset::Bootloader`]'s whole sequence, three
/// units where the other two resets take one.
///
/// A **legacy** route cannot deliver this command (see [`UNITS_PORT_FLAGS`]), and
/// a **units** route cannot carry the tunnel a flash run then needs: a raw frame
/// coming up a units port is dropped without a word (`bridge_port_raw_up` in
/// `third_party/klipper/src/bridge.c`), and the katapult stream is exactly that.
/// So: name the port units, send the request, give the device
/// [`BOOTLOADER_ENTRY_WAIT`] to leave its firmware, and name the port legacy
/// again — ready for the tunnel. `node_id` is the id the device answers to, and
/// both namings carry it.
///
/// # Errors
/// [`BridgeError::Link`] when the link refuses a port-level write. A write that
/// fails after the first naming leaves the port **units**: the sequence cannot
/// restore a route it could not write.
pub async fn request_bootloader(
    link: &Interface,
    index: u8,
    node_id: u8,
    uuid: Option<[u8; 6]>,
) -> Result<(), BridgeError> {
    link.send_set_port_node_id(index, node_id, UNITS_PORT_FLAGS)?;
    link.send_request_bootloader(index, uuid)?;
    tokio::time::sleep(BOOTLOADER_ENTRY_WAIT).await;
    link.send_set_port_node_id(index, node_id, LEGACY_PORT_FLAGS)?;
    Ok(())
}

/// Name the target's port `route` says and open its flashing tunnel —
/// [`open_bridge`]'s step 3, after the declaration (and any resets) so the
/// bridge has the port to route and toggle. `admin bridge port` is the standalone
/// caller: it declares and names one port and stops there; a flash run names its
/// target [`PortRoute::Legacy`] because the tunnel's uplink is the raw frame
/// stream a legacy port serves.
///
/// # Errors
/// [`BridgeError::Link`] when the link refuses a port-level write.
pub fn open_target_port(
    link: &Interface,
    target: BridgeTarget,
    route: PortRoute,
) -> Result<(), BridgeError> {
    // The target's route first, then the tunnel that makes the port carry the
    // katapult stream.
    link.send_set_port_node_id(target.index, target.node_id, route.flags())?;
    link.send_set_port_tunnel(target.index)?;
    Ok(())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::assign;
    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::mcu::McuConfig;
    use crate::core::klippy::config::{Config, ConfigWrapper};
    use crate::core::klippy::frame::{MESSAGE_HEADER_SIZE, MESSAGE_TRAILER_SIZE};
    use crate::core::klippy::interface::devices::serial::{
        admin_unit, attach_downstream, decode_header, HeaderScan, ADMIN_LISTEN,
        CMD_REQUEST_BOOTLOADER, CMD_RESET_PORT, CMD_RESTART_DEVICE, CMD_SET_PORT_NODEID,
        CMD_SET_PORT_TUNNEL, HEADER_BYTES, MAGIC, MUX_PORT_UNITS,
    };
    use crate::core::klippy::interface::pty::Pty;
    use crate::core::klippy::interface::HostMux;
    use crate::core::klippy::mcu::host_libraries::HostLibraries;
    use crate::core::klippy::mcu::Dictionary;
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::{ArgValue, Payload};
    use crate::core::klippy::Interface;
    use std::io::Read;
    use std::os::fd::AsRawFd;
    use std::time::{Duration, Instant};

    /// The id the target device is named with behind its port.
    const TARGET_NODE_ID: u8 = 7;

    /// The bus rate the plans here carry for a bridge's CAN slot.
    ///
    /// Every fixture in this module declares **UART** slots (the mux dictionary
    /// and the muxed library have no CAN slot at all), so the declaration never
    /// reads it: the entry carries it because [`BridgePort`] carries both rates
    /// and the bridge's own slot table picks the one its slot takes.
    const CAN_BITRATE: u32 = 1_000_000;

    /// The id the bridge's own connection is attached as — the arbitration id
    /// its frames carry on a shared serial wire.
    const BRIDGE_MCU_ID: u32 = 1;

    /// The mux build's data dictionary: the same dictionary an identify would
    /// install, read from the file `make` wrote instead.
    fn mux_dictionary() -> Dictionary {
        let path = klipperx_test_support::klipper_host_mux_dictionary_path();
        let raw =
            std::fs::read(&path).unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
        Dictionary::from_json(serde_json::from_slice(&raw).expect("the dictionary file is JSON"))
            .expect("the mux build's data dictionary")
    }

    /// That dictionary in a parser, for decoding what a session put on the wire.
    fn mux_parser() -> Parser {
        let mut parser = Parser::new();
        mux_dictionary()
            .install(&mut parser)
            .expect("install the mux build's dictionary");
        parser
    }

    /// A bridge link on a pty: `attach_downstream` registers a slot with no
    /// handshake, so every unit the session writes lands on the pty master
    /// where a test can read it back — frames and admin units alike, in order.
    fn shared_serial_link(pty: &Pty) -> Interface {
        let handle = attach_downstream(&pty.path, 250_000, BRIDGE_MCU_ID)
            .expect("the shared serial port opens");
        Interface::shared_serial(handle)
    }

    /// A connection over `link` carrying the mux build's dictionary, installed
    /// directly: the declaration commands are encoded from it without an
    /// identify handshake, since this fixture speaks no firmware.
    fn bridge_mcu(name: &str, link: &Interface) -> Mcu {
        let mcu = Mcu::new(name, link.clone());
        mcu.install_dictionary(mux_dictionary())
            .expect("the mux build's dictionary installs");
        mcu
    }

    /// One call a session made, in order.
    #[derive(Debug, Clone, PartialEq, Eq)]
    enum Step {
        /// A command on the bridge's own connection: its name and arguments.
        Command(String, Vec<ArgValue>),
        /// One admin-channel unit, the bytes the bridge sees behind its mux
        /// header.
        Admin(Vec<u8>),
    }

    /// The admin unit for `command` **without** the mux header a shared serial
    /// wire wraps it in — the bytes the bridge parses.
    fn admin_payload(command: &[u8]) -> Vec<u8> {
        admin_unit(command)[HEADER_BYTES..].to_vec()
    }

    /// [`expected_steps`] is the plan `test_open_bridge_declares_resets_and_opens_the_tunnel_in_order`
    /// runs: two ports, the target at index 0 and port 1 asking for
    /// `RestartDevice`. The declaration pairs come in index order, then the
    /// target's reset, then the target named **legacy** and its tunnel opened.
    fn expected_steps(target_node_id: u8) -> Vec<Step> {
        vec![
            Step::Command("allocate_oids".to_string(), vec![ArgValue::UInt8(4)]),
            Step::Command(
                "config_uart".to_string(),
                vec![
                    ArgValue::UInt8(0),
                    ArgValue::UInt32(250_000),
                    ArgValue::UInt8(0),
                ],
            ),
            Step::Command(
                "config_bridge_port".to_string(),
                vec![ArgValue::UInt8(1), ArgValue::UInt8(0)],
            ),
            Step::Command(
                "config_uart".to_string(),
                vec![
                    ArgValue::UInt8(2),
                    ArgValue::UInt32(115_200),
                    ArgValue::UInt8(1),
                ],
            ),
            Step::Command(
                "config_bridge_port".to_string(),
                vec![ArgValue::UInt8(3), ArgValue::UInt8(2)],
            ),
            Step::Admin(admin_payload(&[CMD_RESET_PORT, 0])),
            // flags=0 is the legacy route the katapult stream needs: a units
            // port would drop the raw uplink in silence.
            Step::Admin(admin_payload(&[
                CMD_SET_PORT_NODEID,
                0,
                target_node_id,
                LEGACY_PORT_FLAGS,
            ])),
            Step::Admin(admin_payload(&[CMD_SET_PORT_TUNNEL, 0])),
        ]
    }

    /// The declaration's steps only: [`expected_steps`] up to the last
    /// `config_bridge_port`, with nothing after it.
    fn expected_declaration_steps() -> Vec<Step> {
        expected_steps(TARGET_NODE_ID)
            .into_iter()
            .filter(|step| matches!(step, Step::Command(..)))
            .collect()
    }

    /// The port half's steps only: the target named legacy and its tunnel
    /// opened, with nothing before them.
    fn expected_target_steps(target_node_id: u8) -> Vec<Step> {
        expected_steps(target_node_id)
            .into_iter()
            .filter(|step| matches!(step, Step::Admin(..)) && !is_reset(step))
            .collect()
    }

    /// [`expected_steps`]' admin half for a plan whose target (index 1) is sent
    /// into its bootloader: the target's three units — units naming, the
    /// request, legacy naming — and then [`open_target_port`]'s own legacy
    /// naming and the tunnel. The port below the target is declared and gets
    /// nothing: `admin flash --bridge --port 1` does not pulse port 0's device.
    ///
    /// The legacy naming appears twice on purpose: [`request_bootloader`]
    /// restores the route it broke, and [`open_target_port`] establishes the
    /// route it needs, each without knowing the other ran. Both are the same
    /// four-byte idempotent unit.
    fn expected_bootloader_admin_steps() -> Vec<Step> {
        vec![
            Step::Admin(admin_payload(&[
                CMD_SET_PORT_NODEID,
                1,
                TARGET_NODE_ID,
                MUX_PORT_UNITS,
            ])),
            Step::Admin(admin_payload(&[CMD_REQUEST_BOOTLOADER, 1])),
            Step::Admin(admin_payload(&[
                CMD_SET_PORT_NODEID,
                1,
                TARGET_NODE_ID,
                LEGACY_PORT_FLAGS,
            ])),
            Step::Admin(admin_payload(&[
                CMD_SET_PORT_NODEID,
                1,
                TARGET_NODE_ID,
                LEGACY_PORT_FLAGS,
            ])),
            Step::Admin(admin_payload(&[CMD_SET_PORT_TUNNEL, 1])),
        ]
    }

    /// Whether a step is the admin unit for `command` naming port `index`: the
    /// command byte and the port byte right after the `KXID` marker.
    fn is_command_for_port(step: &Step, command: u8, index: u8) -> bool {
        let Step::Admin(bytes) = step else {
            return false;
        };
        let Some(kxid) = bytes
            .windows(MAGIC.len())
            .position(|w| w == MAGIC.as_slice())
        else {
            return false;
        };
        bytes.get(kxid + MAGIC.len()) == Some(&command)
            && bytes.get(kxid + MAGIC.len() + 1) == Some(&index)
    }

    /// Whether a step is a port-reset unit (`0x04`/`0x05`): the admin unit's
    /// `KXID` frame carries its command right after [`MAGIC`].
    fn is_reset(step: &Step) -> bool {
        let Step::Admin(bytes) = step else {
            return false;
        };
        let Some(kxid) = bytes
            .windows(MAGIC.len())
            .position(|w| w == MAGIC.as_slice())
        else {
            return false;
        };
        matches!(
            bytes.get(kxid + MAGIC.len()),
            Some(&command) if command == CMD_RESET_PORT || command == CMD_RESTART_DEVICE
        )
    }

    /// Every unit a session sent, in wire order: an admin unit is one step, a
    /// command frame can hold several messages (the connection coalesces a
    /// batch into one block) and yields one step per message.
    fn steps(parser: &Parser, units: &[(u32, Vec<u8>)]) -> Vec<Step> {
        let bridge_arb = assign::arb_id(BRIDGE_MCU_ID).expect("a valid bridge id");
        let mut steps = Vec::new();
        for (arb, frame) in units {
            if *arb == ADMIN_LISTEN {
                steps.push(Step::Admin(frame.clone()));
                continue;
            }
            assert_eq!(
                *arb, bridge_arb,
                "the bridge's own connection sends under its own id"
            );
            let payload = Payload::from_raw(
                frame[MESSAGE_HEADER_SIZE..frame.len() - MESSAGE_TRAILER_SIZE].to_vec(),
            );
            for (msg, args) in parser.decode(payload).expect("the payload decodes") {
                steps.push(Step::Command(msg.name.clone(), args));
            }
        }
        steps
    }

    /// Split a host→device byte run into whole `(arbitration id, frame)` units,
    /// leaving a trailing partial unit unparsed. The length byte after the two
    /// header bytes is the whole unit's length for both a command frame and an
    /// admin unit.
    fn split_units(mut wire: &[u8]) -> Vec<(u32, Vec<u8>)> {
        let mut units = Vec::new();
        loop {
            match decode_header(wire) {
                HeaderScan::Complete { arb, consumed } => {
                    if wire.len() < consumed + 1 {
                        break;
                    }
                    let length = wire[consumed] as usize;
                    if wire.len() < consumed + length {
                        break;
                    }
                    units.push((arb, wire[consumed..consumed + length].to_vec()));
                    wire = &wire[consumed + length..];
                }
                HeaderScan::NeedMore => break,
                HeaderScan::Invalid => panic!("a sent unit must start with a header"),
            }
        }
        units
    }

    /// Read the pty master until `done` accepts what has arrived — or `window`
    /// expires — then hand back the whole units. A bounded wait, so a
    /// "nothing was written" check costs one window and a "these arrived"
    /// check stops as soon as the last one is in hand.
    fn read_units(
        master: &std::fs::File,
        window: Duration,
        done: impl Fn(&[(u32, Vec<u8>)]) -> bool,
    ) -> Vec<(u32, Vec<u8>)> {
        let deadline = Instant::now() + window;
        let mut wire = Vec::new();
        let mut buf = [0u8; 512];
        let mut reader = master;
        loop {
            let units = split_units(&wire);
            if done(&units) {
                return units;
            }
            let now = Instant::now();
            if now >= deadline {
                return units;
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
                return units; // the window expired with nothing more to read
            }
            match reader.read(&mut buf) {
                Ok(0) => return units,
                Ok(read) => wire.extend_from_slice(&buf[..read]),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
                Err(e) => panic!("reading the pty master: {e}"),
            }
        }
    }

    /// The three steps, in the order the firmware needs them, observed at the
    /// wire: a bridge link on a pty, the declaration as coalesced command
    /// frames, the port-level commands as admin units behind their mux header.
    ///
    /// The plan below asks for `RestartDevice` on port 1 and the run targets
    /// port 0, so port 1 is declared and its reset never goes out.
    #[tokio::test]
    async fn test_open_bridge_declares_resets_and_opens_the_tunnel_in_order() {
        let pty = Pty::new();
        let link = shared_serial_link(&pty);
        let mcu = bridge_mcu("bridge", &link);

        let ports = [
            BridgePort {
                baud: 250_000,
                bitrate: CAN_BITRATE,
                reset: PortReset::ResetPort,
            },
            BridgePort {
                baud: 115_200,
                bitrate: CAN_BITRATE,
                reset: PortReset::RestartDevice,
            },
        ];
        let target = BridgeTarget {
            index: 0,
            node_id: TARGET_NODE_ID,
        };
        let ready = open_bridge(&mcu, &link, &ports, target)
            .await
            .expect("the session runs");

        assert_eq!(
            ready.ports,
            ports.to_vec(),
            "the plan as it was carried out"
        );
        assert_eq!(ready.target, target);

        // The three admin units are the last things on the wire, so seeing them
        // all means every declaration frame is already in hand.
        let units = read_units(&pty.master, Duration::from_secs(2), |units| {
            units.iter().filter(|(arb, _)| *arb == ADMIN_LISTEN).count() >= 3
        });
        let parser = mux_parser();
        assert_eq!(
            steps(&parser, &units),
            expected_steps(TARGET_NODE_ID),
            "the declaration pairs in index order, then the target's reset as its plan \
             says, then the target named legacy and its tunnel opened"
        );
    }

    /// Only the **target's** entry is reset, whatever the plan says about the
    /// ports below it: a flash run that reaches port 2 declares ports 0 and 1 —
    /// the bridge numbers a port by the order its declaration rows arrive — and
    /// sends a line pulse or a bootloader request to neither of them.
    ///
    /// Before this, `admin flash --bridge --port N` pulsed the line of every
    /// port below N: a reset of devices the caller never named.
    #[tokio::test]
    async fn test_only_the_targets_port_is_reset() {
        let pty = Pty::new();
        let link = shared_serial_link(&pty);
        let mcu = bridge_mcu("bridge", &link);

        let ports = [
            BridgePort {
                baud: 250_000,
                bitrate: CAN_BITRATE,
                reset: PortReset::RestartDevice,
            },
            BridgePort {
                baud: 250_000,
                bitrate: CAN_BITRATE,
                reset: PortReset::Bootloader,
            },
            BridgePort {
                baud: 250_000,
                bitrate: CAN_BITRATE,
                reset: PortReset::ResetPort,
            },
        ];
        let target = BridgeTarget {
            index: 2,
            node_id: TARGET_NODE_ID,
        };
        open_bridge(&mcu, &link, &ports, target)
            .await
            .expect("the session runs");

        // The declaration is 3 pairs of commands and the port half 2 units, so
        // both are in hand once the tunnel is on the wire.
        let units = read_units(&pty.master, Duration::from_secs(2), |units| {
            units.iter().filter(|(arb, _)| *arb == ADMIN_LISTEN).count() >= 2
        });
        let parser = mux_parser();
        let observed: Vec<Step> = steps(&parser, &units)
            .into_iter()
            .filter(|step| matches!(step, Step::Admin(..)))
            .collect();
        assert_eq!(
            observed,
            vec![
                Step::Admin(admin_payload(&[CMD_RESET_PORT, 2])),
                Step::Admin(admin_payload(&[
                    CMD_SET_PORT_NODEID,
                    2,
                    TARGET_NODE_ID,
                    LEGACY_PORT_FLAGS,
                ])),
                Step::Admin(admin_payload(&[CMD_SET_PORT_TUNNEL, 2])),
            ],
            "the target's pulse and its port's two namings, and nothing for ports 0 and 1"
        );
        for (command, index) in [
            (CMD_RESET_PORT, 0),
            (CMD_RESTART_DEVICE, 0),
            (CMD_RESET_PORT, 1),
        ] {
            assert!(
                observed
                    .iter()
                    .all(|step| !is_command_for_port(step, command, index)),
                "port {index} is declared only: no reset unit names it"
            );
        }
        assert!(
            observed
                .iter()
                .all(|step| !is_command_for_port(step, CMD_REQUEST_BOOTLOADER, 1)),
            "port 1's plan reset is a bootloader request, and it is not sent either"
        );
    }

    /// `admin bridge config`'s one write: the declaration pairs in index order,
    /// and nothing else — no route, no tunnel, and above all no reset unit.
    ///
    /// The window is fixed because the silence is the evidence: no admin unit
    /// ever arrives, so there is nothing to wait for.
    #[tokio::test]
    async fn test_declare_ports_sends_the_declaration_and_no_reset() {
        let pty = Pty::new();
        let link = shared_serial_link(&pty);
        let mcu = bridge_mcu("bridge", &link);

        let ports = [
            BridgePort {
                baud: 250_000,
                bitrate: CAN_BITRATE,
                reset: PortReset::ResetPort,
            },
            BridgePort {
                baud: 115_200,
                bitrate: CAN_BITRATE,
                reset: PortReset::RestartDevice,
            },
        ];
        declare_ports(&mcu, &ports)
            .await
            .expect("the declaration is sent");

        let units = read_units(&pty.master, Duration::from_millis(500), |_| false);
        let parser = mux_parser();
        let observed = steps(&parser, &units);
        assert_eq!(
            observed,
            expected_declaration_steps(),
            "the declaration pairs in index order and nothing after them"
        );
        assert!(
            observed.iter().all(|step| !is_reset(step)),
            "declaring ports resets nothing"
        );
    }

    /// A bridge dictionary that has a CAN downstream slot, as the stm32 arch
    /// publishes it: `BRIDGE_UART_COUNT` is every slot it numbers (the UART ones
    /// and the CAN one), and `RESERVE_PINS_bridge_can` is the pin pair of the CAN
    /// port, which is how a build says it has one at all. `uart_slots` is how
    /// many UART slots come **before** the CAN slot.
    fn stm32_can_bridge_dictionary(uart_slots: u64) -> Dictionary {
        Dictionary::from_json(serde_json::json!({
            "config": {
                "MCU": "stm32f446xx",
                "BRIDGE_UART_COUNT": uart_slots + 1,
                "RESERVE_PINS_bridge_can": "PB8,PB9",
            },
            "commands": {
                // The identify pair owns id 1 in every parser, so a synthetic
                // dictionary numbers its own messages above it.
                "allocate_oids count=%c": 2,
                "config_uart oid=%c baud=%u slot=%c": 3,
                "config_can oid=%c bitrate=%u slot=%c": 4,
                "config_bridge_port oid=%c port=%c": 5,
            }
        }))
        .expect("an stm32 bridge dictionary with a CAN slot")
    }

    /// The slot order is the arch's: **every UART slot first, the CAN slot last**,
    /// and the two constants in the dictionary are what says so —
    /// `BRIDGE_UART_COUNT` (every slot the firmware numbers) and
    /// `RESERVE_PINS_bridge_can` (this build has a CAN port).
    #[test]
    fn test_the_can_slot_is_the_last_one_the_firmware_numbers() {
        let dictionary = stm32_can_bridge_dictionary(2);
        assert_eq!(slot_kind(Some(&dictionary), 0), PortKind::Uart);
        assert_eq!(slot_kind(Some(&dictionary), 1), PortKind::Uart);
        assert_eq!(slot_kind(Some(&dictionary), 2), PortKind::Can);
        // A slot past the ones the firmware numbered is not the CAN one by
        // accident: only the slot it named is.
        assert_eq!(slot_kind(Some(&dictionary), 3), PortKind::Uart);
        assert_eq!(slot_kind(Some(&dictionary), 255), PortKind::Uart);
    }

    /// A CAN-only bridge numbers one slot and it is the CAN one — no UART slot
    /// comes before it, since `BRIDGE_UART_COUNT` is 1 there.
    #[test]
    fn test_a_can_only_bridge_has_the_can_slot_at_zero() {
        let dictionary = stm32_can_bridge_dictionary(0);
        assert_eq!(slot_kind(Some(&dictionary), 0), PortKind::Can);
        assert_eq!(slot_kind(Some(&dictionary), 1), PortKind::Uart);
    }

    /// A backend with no CAN slot — the linux bridge, the host library — declares
    /// no such constant, so every slot it numbers is a UART slot. So does a
    /// connection whose dictionary never arrived: nothing said otherwise.
    #[test]
    fn test_a_bridge_without_a_can_slot_declares_uart_ports() {
        let dictionary = Dictionary::from_json(serde_json::json!({
            "config": { "BRIDGE_UART_COUNT": 4 }
        }))
        .expect("a host library bridge dictionary");
        assert_eq!(slot_kind(Some(&dictionary), 0), PortKind::Uart);
        assert_eq!(slot_kind(Some(&dictionary), 3), PortKind::Uart);
        assert_eq!(slot_kind(None, 0), PortKind::Uart);
    }

    /// The declaration takes its command from the slot: `config_uart` at the line
    /// rate on a UART slot, `config_can` at the plan's **bitrate** on the CAN one,
    /// and the same `config_bridge_port` pair after each — so one plan entry
    /// carries both rates and only the slot's own is used.
    #[tokio::test]
    async fn test_declare_ports_opens_the_can_slot_with_config_can_at_the_plan_bitrate() {
        let pty = Pty::new();
        let link = shared_serial_link(&pty);
        let mcu = Mcu::new("bridge", link.clone());
        let dictionary = stm32_can_bridge_dictionary(1);
        mcu.install_dictionary(dictionary.clone())
            .expect("the CAN bridge dictionary installs");
        let mut parser = Parser::new();
        dictionary
            .install(&mut parser)
            .expect("install the CAN bridge's dictionary");

        // One UART slot (index 0) and the CAN slot after it (index 1). The
        // bitrate is not the line rate and the line rate is not the bitrate, so
        // a port declared with the wrong one cannot pass.
        let ports = [
            BridgePort {
                baud: 250_000,
                bitrate: 500_000,
                reset: PortReset::ResetPort,
            },
            BridgePort {
                baud: 115_200,
                bitrate: 500_000,
                reset: PortReset::ResetPort,
            },
        ];
        declare_ports(&mcu, &ports)
            .await
            .expect("the declaration is sent");

        let units = read_units(&pty.master, Duration::from_millis(500), |_| false);
        assert_eq!(
            steps(&parser, &units),
            vec![
                Step::Command("allocate_oids".to_string(), vec![ArgValue::UInt8(4)]),
                Step::Command(
                    "config_uart".to_string(),
                    vec![
                        ArgValue::UInt8(0),
                        ArgValue::UInt32(250_000),
                        ArgValue::UInt8(0),
                    ],
                ),
                Step::Command(
                    "config_bridge_port".to_string(),
                    vec![ArgValue::UInt8(1), ArgValue::UInt8(0)],
                ),
                Step::Command(
                    "config_can".to_string(),
                    vec![
                        ArgValue::UInt8(2),
                        ArgValue::UInt32(500_000),
                        ArgValue::UInt8(1),
                    ],
                ),
                Step::Command(
                    "config_bridge_port".to_string(),
                    vec![ArgValue::UInt8(3), ArgValue::UInt8(2)],
                ),
            ],
            "a UART slot at its line rate, the CAN slot at the plan's bitrate"
        );
    }

    /// `admin bridge port`'s write: the target named legacy and its tunnel
    /// opened, and nothing else — no declaration, no reset.
    #[tokio::test]
    async fn test_open_target_port_names_and_tunnels_without_a_reset() {
        let pty = Pty::new();
        let link = shared_serial_link(&pty);

        let target = BridgeTarget {
            index: 0,
            node_id: TARGET_NODE_ID,
        };
        open_target_port(&link, target, PortRoute::Legacy)
            .expect("the route and the tunnel go out");

        // Two admin units are expected (the route and the tunnel), so seeing
        // both means the window is done.
        let units = read_units(&pty.master, Duration::from_secs(2), |units| {
            units.iter().filter(|(arb, _)| *arb == ADMIN_LISTEN).count() >= 2
        });
        let parser = mux_parser();
        let observed = steps(&parser, &units);
        assert_eq!(
            observed,
            expected_target_steps(TARGET_NODE_ID),
            "the target named legacy and its tunnel opened, with nothing before them"
        );
        assert!(
            observed.iter().all(|step| !is_reset(step)),
            "naming a port and opening its tunnel resets nothing"
        );
    }

    /// `admin bridge port`'s whole job on a port a device speaks units behind:
    /// the port is **declared** first (a port-level command that names a port the
    /// bridge does not have is dropped in silence), then named `MUX_PORT_UNITS`,
    /// then tunnelled — and that order is the wire order, in one connection.
    #[tokio::test]
    async fn test_a_units_port_is_declared_before_it_is_named_and_tunnelled() {
        let pty = Pty::new();
        let link = shared_serial_link(&pty);
        let mcu = bridge_mcu("bridge", &link);

        declare_port(
            &mcu,
            0,
            BridgePort {
                baud: 250_000,
                bitrate: CAN_BITRATE,
                reset: PortReset::ResetPort,
            },
        )
        .await
        .expect("the port declares");
        let target = BridgeTarget {
            index: 0,
            node_id: TARGET_NODE_ID,
        };
        open_target_port(&link, target, PortRoute::Units)
            .expect("the units route and the tunnel go out");

        let units = read_units(&pty.master, Duration::from_secs(2), |units| {
            units.iter().filter(|(arb, _)| *arb == ADMIN_LISTEN).count() >= 2
        });
        let parser = mux_parser();
        assert_eq!(
            steps(&parser, &units),
            vec![
                Step::Command("allocate_oids".to_string(), vec![ArgValue::UInt8(2)]),
                Step::Command(
                    "config_uart".to_string(),
                    vec![
                        ArgValue::UInt8(0),
                        ArgValue::UInt32(250_000),
                        ArgValue::UInt8(0),
                    ],
                ),
                Step::Command(
                    "config_bridge_port".to_string(),
                    vec![ArgValue::UInt8(1), ArgValue::UInt8(0)],
                ),
                Step::Admin(admin_payload(&[
                    CMD_SET_PORT_NODEID,
                    0,
                    TARGET_NODE_ID,
                    MUX_PORT_UNITS,
                ])),
                Step::Admin(admin_payload(&[CMD_SET_PORT_TUNNEL, 0])),
            ],
            "the declaration comes first, then the units naming, then the tunnel"
        );
    }

    /// The bootloader request's three units, in order: the port named **units**
    /// (the only route that delivers the request), the request itself, and the
    /// port named **legacy** again (the only route the tunnel's raw uplink
    /// serves). Time is paused, so the wait between the request and the restore
    /// costs the test nothing.
    #[tokio::test]
    async fn test_request_bootloader_names_units_sends_the_request_and_returns_to_legacy() {
        tokio::time::pause();
        let pty = Pty::new();
        let link = shared_serial_link(&pty);

        request_bootloader(&link, 1, TARGET_NODE_ID, None)
            .await
            .expect("the three units go out");

        let units = read_units(&pty.master, Duration::from_secs(2), |units| {
            units.iter().filter(|(arb, _)| *arb == ADMIN_LISTEN).count() >= 3
        });
        let parser = mux_parser();
        assert_eq!(
            steps(&parser, &units),
            vec![
                Step::Admin(admin_payload(&[
                    CMD_SET_PORT_NODEID,
                    1,
                    TARGET_NODE_ID,
                    MUX_PORT_UNITS,
                ])),
                Step::Admin(admin_payload(&[CMD_REQUEST_BOOTLOADER, 1])),
                Step::Admin(admin_payload(&[
                    CMD_SET_PORT_NODEID,
                    1,
                    TARGET_NODE_ID,
                    LEGACY_PORT_FLAGS,
                ])),
            ],
            "the request travels a units route, and the route is legacy again afterwards"
        );
    }

    /// A plan whose target is sent into its bootloader: the target's three
    /// units sit where its one reset unit would, and the port below it is
    /// declared and gets nothing — no pulse, no units naming, no request, no
    /// second naming.
    ///
    /// The wait inside the sequence is real time here: [`open_bridge`] also runs
    /// the declaration barrier, whose own timeout must not be auto-advanced. A
    /// fixture that acks nothing makes the connection re-send its declaration
    /// block through that wait, so only the admin units on the wire are compared
    /// — the declaration is [`test_open_bridge_declares_resets_and_opens_the_tunnel_in_order`]'s.
    #[tokio::test]
    async fn test_open_bridge_takes_a_bootloader_port_through_units_and_back_to_legacy() {
        let pty = Pty::new();
        let link = shared_serial_link(&pty);
        let mcu = bridge_mcu("bridge", &link);

        let ports = [
            BridgePort {
                baud: 250_000,
                bitrate: CAN_BITRATE,
                reset: PortReset::ResetPort,
            },
            BridgePort {
                baud: 115_200,
                bitrate: CAN_BITRATE,
                reset: PortReset::Bootloader,
            },
        ];
        let target = BridgeTarget {
            index: 1,
            node_id: TARGET_NODE_ID,
        };
        open_bridge(&mcu, &link, &ports, target)
            .await
            .expect("the session runs");

        // Five admin units close the sequence (the target's three, its naming
        // and the tunnel).
        let units = read_units(&pty.master, Duration::from_secs(2), |units| {
            units.iter().filter(|(arb, _)| *arb == ADMIN_LISTEN).count() >= 5
        });
        let parser = mux_parser();
        let observed: Vec<Step> = steps(&parser, &units)
            .into_iter()
            .filter(|step| matches!(step, Step::Admin(..)))
            .collect();
        assert_eq!(
            observed,
            expected_bootloader_admin_steps(),
            "the bootloader port's units where its reset would be, then its naming \
             and the tunnel"
        );
        assert_eq!(
            observed
                .iter()
                .filter(|step| is_command_for_port(step, CMD_RESET_PORT, 0))
                .count(),
            0,
            "port 0 is declared only: no pulse reaches the device below the target"
        );
        for command in [
            CMD_SET_PORT_NODEID,
            CMD_REQUEST_BOOTLOADER,
            CMD_SET_PORT_TUNNEL,
        ] {
            assert!(
                observed
                    .iter()
                    .all(|step| !is_command_for_port(step, command, 0)),
                "port 0 has no bootloader sequence and no tunnel: only the target's port does"
            );
        }
    }

    /// `admin reset --port`'s one write: the port-reset unit (`0x04`, a line pulse),
    /// the restart-device unit (`0x05`), or the request-bootloader unit (`0x02`),
    /// each naming the port it means. No receipt, so the wire bytes are the whole
    /// evidence.
    ///
    /// The bootloader unit here is the **bare** one [`reset_port`] sends: a legacy
    /// route strips it, which is why [`request_bootloader`] is the route a flash
    /// run takes, and this is the call `admin reset --port` can make without an
    /// id to name a port with.
    #[tokio::test]
    async fn test_reset_port_sends_the_one_port_reset_unit() {
        let pty = Pty::new();
        let link = shared_serial_link(&pty);

        reset_port(&link, 2, PortReset::ResetPort, None).expect("the pulse goes out");
        reset_port(&link, 3, PortReset::RestartDevice, None).expect("the restart goes out");
        reset_port(&link, 4, PortReset::Bootloader, None).expect("the bootloader request goes out");

        let units = read_units(&pty.master, Duration::from_secs(2), |units| {
            units.iter().filter(|(arb, _)| *arb == ADMIN_LISTEN).count() >= 3
        });
        let parser = mux_parser();
        assert_eq!(
            steps(&parser, &units),
            vec![
                Step::Admin(admin_payload(&[CMD_RESET_PORT, 2])),
                Step::Admin(admin_payload(&[CMD_RESTART_DEVICE, 3])),
                Step::Admin(admin_payload(&[CMD_REQUEST_BOOTLOADER, 4])),
            ],
            "the three resets are the firmware's own units, naming their port"
        );
    }

    /// The sequence count is the connection's own now, so what is left to pin is
    /// that a plan the oid byte cannot number is refused before anything is sent.
    #[tokio::test]
    async fn test_an_impossible_plan_is_refused_and_sends_nothing() {
        let pty = Pty::new();
        let link = shared_serial_link(&pty);
        let mcu = bridge_mcu("bridge", &link);

        let target = BridgeTarget {
            index: 0,
            node_id: TARGET_NODE_ID,
        };
        let one_too_many = vec![
            BridgePort {
                baud: 250_000,
                bitrate: CAN_BITRATE,
                reset: PortReset::ResetPort,
            };
            MAX_BRIDGE_PORTS + 1
        ];
        let err = open_bridge(&mcu, &link, &one_too_many, target)
            .await
            .expect_err("one port over the oid byte is refused");
        match err {
            BridgeError::TooManyPorts { ports } => assert_eq!(ports, MAX_BRIDGE_PORTS + 1),
            other => panic!("expected TooManyPorts, got {other:?}"),
        }

        let units = read_units(&pty.master, Duration::from_millis(300), |units| {
            !units.is_empty()
        });
        assert!(
            units.is_empty(),
            "a plan that cannot run sends nothing, got {units:?}"
        );
    }

    /// Read from a pty's master end for at most `window`: what the bridge wrote
    /// to that port, handed over as soon as `at_least` bytes are in hand — and
    /// empty when the port stays silent the whole window, so a "nothing was
    /// written" check costs one bounded wait.
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
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
                Err(e) => panic!("reading the pty master: {e}"),
            }
        }
        out
    }

    /// The session over a real bridge: the mux host library loaded as a bridge
    /// with two ports on real ptys — port 0 the flashing target (legacy, sitting
    /// in its bootloader), port 1 a named device the bridge restarts.
    ///
    /// The bridge's connection is built the way the config layer builds one —
    /// `McuConfig` → [`McuConfig::open`] → [`Mcu::new`] → [`Mcu::identify`] —
    /// which is the sequence a bridge-port flash uses, so this test also pins
    /// that it works. The two sections on one path are what makes it muxed; the
    /// port ttys are handed to the library as the `(slot, path)` pairs the
    /// `[uart_device]` sections resolve to at load.
    ///
    /// Every command's effect is pinned where the wire is. The two declaration
    /// commands are the library's (they are what opens a port's stream, and
    /// nothing of them reaches the line), the target's naming and its tunnel are
    /// the bridge's own (consumed, never written to a port, no receipt), and
    /// `RESTART_DEVICE` for port 1 is forwarded to it header-stripped — the bare
    /// `KXID` frame a legacy port's line carries.
    ///
    /// `multi_thread` is not incidental: the test thread blocks in the ptys'
    /// `poll` ([`read_pty`]), so the connection's transport tasks need worker
    /// threads of their own to run while it does.
    ///
    /// No [`HOST_LIBRARY_TEST_LOCK`]: this test redirects no stderr and loads a
    /// **private copy** of the library, so it is not the per-path singleton the
    /// other library tests share and may run beside them.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_open_bridge_over_the_real_library() {
        // A bridge opens a port's tty when the host declares it, so those ttys
        // have to exist when the library comes up, each on its own pty.
        let target_port = Pty::new();
        let named_port = Pty::new();

        let copy = klipperx_test_support::PrivateCopy::of(
            &klipperx_test_support::klipper_host_mux_lib_path(),
        )
        .expect("the private library copy is made");
        let library = copy.path().to_path_buf();
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
        // and the path gets an identity — the value the library is told.
        let libraries = HostLibraries::classify(&wrapper).expect("the path is told apart");
        let identity = libraries.identity(&path).expect("the path has an identity");

        let mut mcu_config = McuConfig::new(&wrapper).expect("the [mcu] section");
        mcu_config.id = Some(BRIDGE_MCU_ID);
        mcu_config.host_mux = Some(HostMux {
            identity,
            named: identity,
        });
        mcu_config.host_port_paths = vec![
            (0, target_port.path.to_string_lossy().into_owned()),
            (1, named_port.path.to_string_lossy().into_owned()),
        ];
        let link = mcu_config
            .open()
            .expect("the muxed library opens and attaches");

        let mcu = Mcu::new("bridge", link.clone());
        tokio::time::timeout(
            Duration::from_secs(20),
            mcu.identify(Duration::from_secs(10)),
        )
        .await
        .expect("the identify handshake timed out")
        .expect("the bridge identifies");

        let ports = [
            BridgePort {
                baud: 250_000,
                bitrate: CAN_BITRATE,
                reset: PortReset::ResetPort,
            },
            BridgePort {
                baud: 250_000,
                bitrate: CAN_BITRATE,
                reset: PortReset::RestartDevice,
            },
        ];
        let target = BridgeTarget {
            index: 0,
            node_id: TARGET_NODE_ID,
        };
        let ready = open_bridge(&mcu, &link, &ports, target)
            .await
            .expect("the session runs against a real bridge");
        assert_eq!(ready.target, target);

        let quiet = Duration::from_millis(500);

        // Port 1 is declared and is not the target, so its plan reset
        // (`RestartDevice`) is never sent: a bridge forwards a restart to its
        // port as the bare KXID frame, and here there is nothing to forward.
        assert!(
            read_pty(&named_port.master, 2, quiet).is_empty(),
            "a port below the target is declared only: no reset unit reaches its line"
        );

        // Port 0, the target: the declaration opened its stream and nothing of
        // it was written to the line, SET_PORT_NODEID and SET_PORT_TUNNEL are
        // the bridge's own, and RESET_PORT is the bridge pulsing the lines — a
        // pty has none, so it is dropped in silence. No residue is what a flash
        // run needs: the tunnel carries a katapult stream on this same line.
        assert!(
            read_pty(&target_port.master, 1, quiet).is_empty(),
            "the declaration, SET_PORT_NODEID, RESET_PORT and SET_PORT_TUNNEL write nothing \
             to the port"
        );

        // The connection's transport tasks hold references to the library, so
        // its drop is not synchronous with `drop(mcu)`: close it and let the
        // blocked device read see the shutdown. Nothing waits the copy out —
        // its path belongs to this test alone.
        mcu.close();
        drop(mcu);
        drop(link);
    }
}
