//! Arguments for the `admin flash` subcommand (its grammar the standalone `flash`
//! binary used to speak), plus `admin bridge config` / `admin bridge port` and
//! `admin reset`, and the plans they make.
//!
//! [`plan`] is the flash grammar as a pure function: it turns the command line
//! into one [`Action`] (query a CAN bus, or flash one board over one direct link)
//! or into an error naming the mistake, before anything is opened or sent. The
//! same grammar `scripts/flashtool.py` speaks, in this repo's spelling: a serial
//! board is addressed by its device path, a CAN board by its CANbus UUID plus the
//! node id the session assigns it. [`plan_bridge_config`], [`plan_bridge_port`]
//! and [`plan_reset`] are the bridge-side siblings: the same bridge grammar,
//! without a firmware to write and without a reset.

use std::path::{Path, PathBuf};
use std::time::Duration;

use clap::Parser;

use crate::core::klippy::interface::devices::canserial::{self, CanbusAdminSocket};
use crate::core::klippy::interface::HostMux;

use super::bridge_flash::{
    AssignReport, BridgeFlashError, BridgeLink, BRIDGE_MCU_ID, IDENTIFY_TIMEOUT,
};
use super::bridge_session::{BridgePort, BridgeTarget, PortKind, PortReset, PortRoute};
use super::session;
use super::transport;

/// The line speed a serial flash opens the port at when `--baud` is not
/// given, in baud — `flashtool.py`'s default.
const DEFAULT_BAUD: u32 = 250_000;

/// The bus rate a bridge's **CAN** slot is declared at, in bits per second —
/// 1 Mbit, the default `[canbus]` and the rate `flashtool.py` uses.
///
/// The two port kinds take different rates and neither is a default for the
/// other: `--baud` is a **UART** slot's line rate, this is the CAN slot's bus rate
/// (`config_can bitrate=%u`, which is the port's `canhw_set_rate`). Which slot is
/// which is the bridge's to say (`BridgeLink::slot_kind`), so one command line can
/// carry both with neither being a guess.
pub const CAN_DEFAULT_BITRATE: u32 = 1_000_000;

/// The node id a CAN flash assigns the board when `--nodeid` is not given:
/// `CANBUS_NODEID_OFFSET` in `flashtool.py`, the bottom of the range the
/// katapult admin commands accept (`nodeid >= 128`).
const DEFAULT_NODEID: u32 = 128;

/// How long `--query` listens for answers, in seconds — the same kind of
/// deadline window a `flashtool.py` query uses.
const QUERY_WINDOW: Duration = Duration::from_secs(2);

/// The command line of the `admin flash` subcommand.
#[derive(Parser, Debug)]
#[command(
    name = "flash",
    version,
    about = "Flash firmware to a board running the katapult bootloader, \
             over a direct serial tty/USB link or a SocketCAN bus, or through a \
             bridge's downstream port",
    arg_required_else_help = true
)]
pub struct FlashArgs {
    /// Serial device the bootloader answers on, tty or USB CDC
    /// (`/dev/ttyACM0`)
    //
    // One of this and `--interface` is required, and there is no default
    // link: naming neither is not a flash with a guess, it is a question
    // (see the `arg_required_else_help` above).
    #[arg(long)]
    pub device: Option<String>,

    /// CAN bus the bootloader answers on, as the network interface is named
    /// (`can0`)
    #[arg(long)]
    pub interface: Option<String>,

    /// CANbus UUID of the board to flash, twelve hex digits in the order a
    /// `[mcu] uuid:` option is written in (CAN flashes only)
    #[arg(long)]
    pub uuid: Option<String>,

    /// Node id the board is given for the flash session, 128..=255 (CAN
    /// flashes only; default 128)
    #[arg(long)]
    pub nodeid: Option<u32>,

    /// Line speed, in baud (serial flashes only; default 250000)
    #[arg(long)]
    pub baud: Option<u32>,

    /// Bus rate of a bridge's **CAN** slot, in bits per second (bridge flashes
    /// only; default 1000000)
    //
    // The other kind's rate: a UART slot is opened at `--baud`, the CAN slot at
    // this. Which slot a port is comes from the bridge's own dictionary.
    #[arg(long, value_name = "BPS")]
    pub can_bitrate: Option<u32>,

    /// Serial tty of a **bridge** to flash a downstream board through: the
    /// board behind the port named by `--port` is the target, reached over the
    /// bridge's flashing tunnel
    ///
    // A bridge's link carries its downstream devices' frames too, so it is
    // opened shared and addressed by id; the tool opens it directly (see
    // `flash::bridge_flash`).
    #[arg(long, value_name = "TTY")]
    pub bridge: Option<String>,

    /// Host library (`libklipper_host.so`) of a **bridge** to flash through,
    /// when the bridge is an in-process host library instead of a serial board
    #[arg(long, value_name = "SO")]
    pub bridge_library: Option<String>,

    /// Identity of the **serial bridge** board itself, twelve hex digits — the
    /// same spelling `--uuid` uses. Needed when that board carries no node id
    /// yet, since only `SET_NODEID` assigns one (only with `--bridge`)
    #[arg(long, value_name = "HEX")]
    pub bridge_uuid: Option<String>,

    /// Identity a host-library bridge answers the serial mux with, twelve hex
    /// digits (only with `--bridge-library`; default all zeros)
    #[arg(long, value_name = "HEX")]
    pub host_identity: Option<String>,

    /// A host-library bridge's port tty, `<slot>:<path>` (only with
    /// `--bridge-library`; repeatable; a port without one keeps the library's
    /// own path)
    #[arg(long = "port-tty", value_name = "SLOT:PATH")]
    pub port_tty: Vec<String>,

    /// Bridge port the target's device hangs on, the plan's index (bridge
    /// flashes only)
    #[arg(long, value_name = "INDEX")]
    pub port: Option<u8>,

    /// Node id the bridge names the target's port with, 1..=255 (bridge flashes
    /// only)
    #[arg(long, value_name = "ID")]
    pub port_nodeid: Option<u8>,

    /// Reset the target port's device gets before the tunnel opens (bridge
    /// flashes only; `reset-port`, `restart-device` or `bootloader`, default
    /// `reset-port`)
    #[arg(long, value_name = "METHOD")]
    pub port_reset: Option<String>,

    /// List the unassigned nodes on a CAN bus instead of flashing
    #[arg(short, long)]
    pub query: bool,

    /// Firmware image to flash (`out/klipper.bin`)
    pub firmware: Option<PathBuf>,
}

/// The command line of `admin bridge config`: the bridge to ask which UART slots
/// it offers.
///
/// The bridge side of [`FlashArgs`], and **only** the question: the bridge is
/// opened, identified, and asked what its backend can carry
/// (`BRIDGE_UART_COUNT`), and the command stops. Nothing is declared, nothing is
/// reset ([`admin reset`](ResetArgs) does that), no port is named or tunnelled
/// ([`admin bridge port`](BridgePortArgs) does that), and no firmware is written
/// ([`admin flash`](FlashArgs) does that).
#[derive(Parser, Debug)]
#[command(
    name = "config",
    version,
    about = "List the UART slots a bridge offers — one line per port it can be \
             given — without declaring a port, naming a target, opening a \
             tunnel, resetting a device, or flashing anything",
    arg_required_else_help = true
)]
pub struct BridgeConfigArgs {
    /// Serial tty of the **bridge** to ask (`/dev/ttyUSB0`)
    #[arg(long, value_name = "TTY")]
    pub bridge: Option<String>,

    /// Host library (`libklipper_host.so`) of a **bridge** to ask, when the
    /// bridge is an in-process host library instead of a serial board
    #[arg(long, value_name = "SO")]
    pub bridge_library: Option<String>,

    /// Identity of the **serial bridge** board itself, twelve hex digits — the
    /// same spelling `flash --bridge-uuid` uses
    ///
    // Optional: without it the bridge is found by the discovery scan when it
    // carries no node id yet.
    #[arg(long, value_name = "HEX")]
    pub bridge_uuid: Option<String>,

    /// Identity a host-library bridge answers the serial mux with, twelve hex
    /// digits (only with `--bridge-library`; default all zeros)
    #[arg(long, value_name = "HEX")]
    pub host_identity: Option<String>,

    /// A host-library bridge's port tty, `<slot>:<path>` (only with
    /// `--bridge-library`; repeatable)
    #[arg(long = "port-tty", value_name = "SLOT:PATH")]
    pub port_tty: Vec<String>,

    /// Line speed every listed slot opens at, in baud (default 250000)
    #[arg(long, value_name = "BAUD")]
    pub baud: Option<u32>,

    /// How many UART slots to assume when the bridge reports no
    /// `BRIDGE_UART_COUNT` (an older firmware; default 1)
    #[arg(long, value_name = "N")]
    pub port_count: Option<u8>,
}

/// The command line of `admin bridge port`: one declared port, **enabled** —
/// declared if the bridge does not have it, named, and tunnelled.
///
/// The bridge side of [`FlashArgs`] after
/// [`admin bridge config`](BridgeConfigArgs) has said which slots exist: the
/// port is declared on the bridge (`config_uart` + `config_bridge_port`), named
/// the **units** way by default (the wire format the mux's devices speak, which
/// is what makes the port's device addressable by id and its answers
/// forwardable), and its flashing tunnel is opened; the command stops. `--legacy`
/// names it the old way instead — no `MUX_PORT_UNITS` — which is what a stock
/// katapult board behind the port still needs. No device is reset
/// ([`admin reset`](ResetArgs) does that) and nothing is flashed
/// ([`admin flash`](FlashArgs) does that).
#[derive(Parser, Debug)]
#[command(
    name = "port",
    version,
    about = "Name one of a bridge's declared ports and open its flashing tunnel, \
             without resetting a device or flashing anything",
    arg_required_else_help = true
)]
pub struct BridgePortArgs {
    /// Serial tty of the **bridge** to open the port on (`/dev/ttyUSB0`)
    #[arg(long, value_name = "TTY")]
    pub bridge: Option<String>,

    /// Host library (`libklipper_host.so`) of a **bridge** to open the port on,
    /// when the bridge is an in-process host library instead of a serial board
    #[arg(long, value_name = "SO")]
    pub bridge_library: Option<String>,

    /// Identity of the **serial bridge** board itself, twelve hex digits — the
    /// same spelling `flash --bridge-uuid` uses
    ///
    // Optional: without it the bridge is found by the discovery scan when it
    // carries no node id yet.
    #[arg(long, value_name = "HEX")]
    pub bridge_uuid: Option<String>,

    /// Identity a host-library bridge answers the serial mux with, twelve hex
    /// digits (only with `--bridge-library`; default all zeros)
    #[arg(long, value_name = "HEX")]
    pub host_identity: Option<String>,

    /// A host-library bridge's port tty, `<slot>:<path>` (only with
    /// `--bridge-library`; repeatable)
    #[arg(long = "port-tty", value_name = "SLOT:PATH")]
    pub port_tty: Vec<String>,

    /// Bridge port to enable and tunnel, the declaration's index — the slot the
    /// bridge numbers the port with, which is the order its ports were declared
    /// in
    #[arg(long, value_name = "INDEX")]
    pub port: Option<u8>,

    /// Node id the bridge names that port with, 1..=255
    #[arg(long, value_name = "ID")]
    pub port_nodeid: Option<u8>,

    /// Identity of the board behind the port to **give** that node id, twelve
    /// hex digits (the UUID a bridge scan reported; only with `--port-nodeid`)
    //
    // Optional: without it the port is only named, and the board behind it keeps
    // whatever id it has — a board with none cannot be named at all.
    #[arg(long, value_name = "HEX")]
    pub assign_uuid: Option<String>,

    /// Line speed the port opens at, in baud (default 250000; a serial bridge's
    /// own link opens at the same rate, and only a **UART** slot uses it)
    #[arg(long, value_name = "BAUD")]
    pub baud: Option<u32>,

    /// Bus rate the **CAN** slot opens at, in bits per second (default 1000000)
    //
    // Not a line speed: a UART slot's stream is opened at `--baud`, the CAN
    // slot's at this rate (`config_can bitrate=%u`). The two never substitute for
    // each other, and which slot is which is the bridge's to say.
    #[arg(long, value_name = "BPS")]
    pub can_bitrate: Option<u32>,

    /// Name the port the legacy way instead of units: the wire format a device
    /// that predates the mux speaks — a stock katapult board behind the port,
    /// whose raw frames the port heads with its own reply id
    #[arg(long)]
    pub legacy: bool,
}

/// A `bridge config` command line, as a plan: the bridge's link and the speed
/// its slots would open at, all validated before any device is opened.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BridgeConfigPlan {
    /// How the bridge's own link is reached.
    pub source: BridgeLinkSource,
    /// The line speed a listed slot would open at.
    pub baud: u32,
    /// How many slots to assume when the bridge reports no
    /// `BRIDGE_UART_COUNT` — the fallback for a firmware that predates it.
    pub port_count: u8,
}

/// A `bridge port` command line, as a plan: the bridge's link and the port to
/// declare, name and tunnel, validated before any device is opened.
///
/// The link opens at `baud`, the same rate the port is declared at: a serial
/// bridge carries both on its own wire, exactly as a bridge flash does.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BridgePortPlan {
    /// How the bridge's own link is reached.
    pub source: BridgeLinkSource,
    /// The line speed the port is declared at (and the link opens at).
    pub baud: u32,
    /// The bus rate the CAN slot is declared at, when the port is that slot.
    pub bitrate: u32,
    /// The port to declare, name and tunnel.
    pub target: BridgeTarget,
    /// How the port is named: units (the default) or legacy.
    pub route: PortRoute,
    /// The board to **assign** [`BridgePortPlan::target`]'s node id to, when
    /// `--assign-uuid` named one.
    pub assign: Option<[u8; 6]>,
}

/// The command line of `admin reset`.
///
/// The bridge side of [`FlashArgs`], and one choice: without `--port` the
/// **bridge itself** is reset (the self-restart [`flash`](FlashArgs) does before
/// it declares ports), and with `--port` a **declared port's device** is, the way
/// `--port-reset` says.
#[derive(Parser, Debug)]
#[command(
    name = "reset",
    version,
    about = "Reset a bridge itself, or a device on one of its ports, through \
             the bridge's admin channel",
    arg_required_else_help = true
)]
pub struct ResetArgs {
    /// Serial tty of the **bridge** whose device to reset (`/dev/ttyUSB0`)
    #[arg(long, value_name = "TTY")]
    pub bridge: Option<String>,

    /// Host library (`libklipper_host.so`) of a **bridge** to reset through,
    /// when the bridge is an in-process host library instead of a serial board
    #[arg(long, value_name = "SO")]
    pub bridge_library: Option<String>,

    /// Identity of the **serial bridge** board itself, twelve hex digits — the
    /// same spelling `flash --bridge-uuid` uses
    #[arg(long, value_name = "HEX")]
    pub bridge_uuid: Option<String>,

    /// Identity a host-library bridge answers the serial mux with, twelve hex
    /// digits (only with `--bridge-library`; default all zeros)
    #[arg(long, value_name = "HEX")]
    pub host_identity: Option<String>,

    /// A host-library bridge's port tty, `<slot>:<path>` (only with
    /// `--bridge-library`; repeatable)
    #[arg(long = "port-tty", value_name = "SLOT:PATH")]
    pub port_tty: Vec<String>,

    /// Line speed the bridge's link opens at, in baud (default 250000)
    #[arg(long, value_name = "BAUD")]
    pub baud: Option<u32>,

    /// A declared bridge port whose device to reset; without it, the bridge
    /// itself
    #[arg(long, value_name = "INDEX")]
    pub port: Option<u8>,

    /// How a `--port` device is reset: `reset-port` (a line pulse),
    /// `restart-device` (its own restart) or `bootloader` (send it into its
    /// bootloader), default `reset-port`
    #[arg(long, value_name = "METHOD")]
    pub port_reset: Option<String>,

    /// Park the **bridge itself** in its bootloader instead of restarting its
    /// firmware (`--bridge` only; takes no `--port`)
    ///
    /// The board leaves the application, so its tty changes: katapult enumerates
    /// as `usb-katapult_<mcu>_<serial>`, which is what `admin flash --device`
    /// writes. A host library has no bootloader, and says so.
    #[arg(long)]
    pub bootloader: bool,
}

/// What a `reset` command line asks to reset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetTarget {
    /// The bridge itself, through its own admin channel.
    Bridge,
    /// The bridge itself, into its bootloader: the same admin channel, with the
    /// device half's `REQUEST_BOOTLOADER` instead of a restart, so the board
    /// parks in katapult and `flash --device` can write it.
    BridgeBootloader,
    /// A declared port's device, the way `reset` says.
    Port {
        /// The port index the device sits behind.
        index: u8,
        /// The reset that port's device gets.
        reset: PortReset,
    },
}

/// A `reset` command line, as a plan: the bridge's link and what to reset, all
/// validated before any device is opened.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResetPlan {
    /// How the bridge's own link is reached.
    pub source: BridgeLinkSource,
    /// The line speed the bridge's link opens at.
    pub baud: u32,
    /// What to reset.
    pub target: ResetTarget,
}

/// What a command line asks the tool to do: the two entry points the
/// [`crate::flash`] session has, over the transport the arguments named.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Action {
    /// Print every board on `interface` that has no node id yet.
    Query { interface: String, window: Duration },
    /// Flash one board over one direct link.
    Flash { target: Target, firmware: PathBuf },
    /// Flash one board that hangs on a bridge's downstream port.
    BridgeFlash {
        target: BridgeFlashTarget,
        firmware: PathBuf,
    },
}

/// The direct link a flash runs over.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Target {
    /// A serial port at a line speed. USB CDC devices enumerating as katapult
    /// (`1d50:6177`) are named the same way — their `/dev/ttyACM*` path.
    Tty { device: String, baud: u32 },
    /// A CAN bus, and the board on it: UUID to address, node id to assign
    /// for the session.
    Can {
        interface: String,
        uuid: [u8; 6],
        nodeid: u32,
    },
}

/// One bridge-port flash, as the arguments describe it: the bridge's link, the
/// port the target's device sits behind, and the reset that device gets.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BridgeFlashTarget {
    /// How the bridge's own link is reached.
    pub source: BridgeLinkSource,
    /// The line speed the target's port — and, for a serial bridge, its own
    /// link — opens at.
    pub baud: u32,
    /// The bus rate the CAN slot opens at, when the target's port is that slot.
    pub bitrate: u32,
    /// The target's port index on the bridge: the plan [`open_bridge`] is given
    /// has one entry per port up to and including it.
    ///
    /// [`open_bridge`]: super::bridge_session::open_bridge
    pub port: u8,
    /// The node id the bridge names that port with.
    pub node_id: u8,
    /// The reset the target port's device gets before the tunnel opens.
    pub reset: PortReset,
}

/// How a bridge's own link is reached.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BridgeLinkSource {
    /// A shared serial tty the bridge answers on; its downstream devices share
    /// the wire, so the link is addressed by id. `uuid` is the bridge board's
    /// own identity, to name it with when it is not assigned yet —
    /// [`serial::attach`](crate::core::klippy::interface::devices::serial::attach)
    /// assigns through `SET_NODEID`, which needs the identity; without one the
    /// bridge must already carry an id to answer the re-confirmation.
    Serial { tty: String, uuid: Option<[u8; 6]> },
    /// A muxed klipper host library loaded from `path`, given `identity` and
    /// its port ttys.
    Library {
        path: String,
        identity: [u8; 6],
        port_ttys: Vec<(u8, String)>,
    },
}

/// Turn a parsed command line into the action it asks for, or an error
/// naming the first mistake.
///
/// Every rejection here happens before any device is opened, so the messages
/// can name the argument pair that disagrees (`--uuid` without
/// `--interface`, a CAN node id for a serial board, a firmware file for a
/// listing query).
pub fn plan(args: &FlashArgs) -> Result<Action, String> {
    if args.query {
        return plan_query(args);
    }
    if args.bridge.is_some() || args.bridge_library.is_some() {
        return plan_bridge(args);
    }
    if let Some(flag) = stray_bridge_argument(args) {
        return Err(format!(
            "{flag} needs --bridge (or --bridge-library): it belongs to a bridge"
        ));
    }

    match (&args.device, &args.interface) {
        (Some(_), Some(_)) => {
            Err("give either --device (serial) or --interface (CAN), not both".into())
        }
        (None, None) => Err("give --device (serial) or --interface (CAN)".into()),
        (Some(device), None) => plan_tty(args, device),
        (None, Some(interface)) => plan_can(args, interface),
    }
}

/// The first bridge-only argument on a line that names no bridge, if any —
/// so the direct and query lines can name the option that belongs elsewhere
/// rather than ignore it.
fn stray_bridge_argument(args: &FlashArgs) -> Option<&'static str> {
    if args.port.is_some() {
        Some("--port")
    } else if args.port_nodeid.is_some() {
        Some("--port-nodeid")
    } else if args.port_reset.is_some() {
        Some("--port-reset")
    } else if args.host_identity.is_some() {
        Some("--host-identity")
    } else if args.bridge_uuid.is_some() {
        Some("--bridge-uuid")
    } else if args.can_bitrate.is_some() {
        Some("--can-bitrate")
    } else if !args.port_tty.is_empty() {
        Some("--port-tty")
    } else {
        None
    }
}

/// The `--bridge` / `--bridge-library` line: a bridge's link, the port the
/// target hangs on, and how the bridge reaches it.
///
/// Everything the direct lines spell differently is refused here so the option
/// that belongs on one line cannot be silently dropped on the other: a bridge
/// has one link (`--bridge` or `--bridge-library`, never both), the target is
/// selected by its port (`--port`) and named by a node id
/// (`--port-nodeid`) — not by `--uuid`/`--nodeid`, which are the CAN line's —
/// and the library-only options come only with `--bridge-library`.
fn plan_bridge(args: &FlashArgs) -> Result<Action, String> {
    if args.bridge.is_some() && args.bridge_library.is_some() {
        return Err(
            "give either --bridge (serial) or --bridge-library (host library), not both".into(),
        );
    }
    if args.device.is_some() {
        return Err("--bridge names the bridge's link; it takes no --device".into());
    }
    if args.interface.is_some() {
        return Err("--bridge names the bridge's link; it takes no --interface".into());
    }
    if args.uuid.is_some() {
        return Err("--uuid selects a CAN board; a bridge target is named by --port-nodeid".into());
    }
    if args.nodeid.is_some() {
        return Err("--nodeid is a CAN node id; a bridge target's id is --port-nodeid".into());
    }
    let port = args
        .port
        .ok_or("--bridge needs --port: which bridge port the target is on")?;
    let node_id = args
        .port_nodeid
        .ok_or("--bridge needs --port-nodeid: the id the bridge names the target port with")?;
    if node_id == 0 {
        return Err(format!("--port-nodeid must be in 1..=255, got {node_id}"));
    }
    if u32::from(node_id) == BRIDGE_MCU_ID {
        return Err(format!(
            "--port-nodeid {node_id} is the bridge's own link id; the target needs another, \
             or its tunnel units collide with the bridge's frames"
        ));
    }
    let reset = parse_port_reset(args.port_reset.as_deref())?;
    let firmware = firmware_or_err(args)?;
    let baud = args.baud.unwrap_or(DEFAULT_BAUD);
    let bitrate = args.can_bitrate.unwrap_or(CAN_DEFAULT_BITRATE);
    let source = plan_bridge_link(
        &args.bridge,
        &args.bridge_library,
        args.bridge_uuid.as_deref(),
        args.host_identity.as_deref(),
        &args.port_tty,
    )?;
    Ok(Action::BridgeFlash {
        target: BridgeFlashTarget {
            source,
            baud,
            bitrate,
            port,
            node_id,
            reset,
        },
        firmware,
    })
}

/// The bridge's own link, from the options that name it: `--bridge` (serial) or
/// `--bridge-library` (host library), never both and never neither, with each
/// side's own identity spelled by its own option.
///
/// Shared by every command whose bridge side is the same shape — `flash`'s
/// shared by every bridge line — `flash`'s bridge line and `admin bridge config`
/// and `admin bridge port` — so they cannot drift on which identity belongs to
/// which link.
///
/// # Errors
/// A message naming the first option out of place.
fn plan_bridge_link(
    bridge: &Option<String>,
    bridge_library: &Option<String>,
    bridge_uuid: Option<&str>,
    host_identity: Option<&str>,
    port_tty: &[String],
) -> Result<BridgeLinkSource, String> {
    match (bridge, bridge_library) {
        (Some(_), Some(_)) => {
            Err("give either --bridge (serial) or --bridge-library (host library), not both".into())
        }
        (None, None) => Err("give --bridge (serial) or --bridge-library (host library)".into()),
        (Some(tty), None) => {
            if host_identity.is_some() {
                return Err(
                    "--host-identity needs --bridge-library: a serial bridge has no library \
                     identity"
                        .into(),
                );
            }
            if !port_tty.is_empty() {
                return Err(
                    "--port-tty needs --bridge-library: a serial bridge opens its own ports".into(),
                );
            }
            let uuid = match bridge_uuid {
                Some(text) => Some(parse_bridge_uuid(text)?),
                None => None,
            };
            Ok(BridgeLinkSource::Serial {
                tty: tty.clone(),
                uuid,
            })
        }
        (None, Some(path)) => {
            if bridge_uuid.is_some() {
                return Err(
                    "--bridge-uuid names a serial bridge; a host library is named by \
                     --host-identity"
                        .into(),
                );
            }
            let identity = match host_identity {
                Some(text) => parse_host_identity(text)?,
                None => [0u8; 6],
            };
            let mut port_ttys = Vec::with_capacity(port_tty.len());
            for spec in port_tty {
                port_ttys.push(parse_port_tty(spec)?);
            }
            Ok(BridgeLinkSource::Library {
                path: path.clone(),
                identity,
                port_ttys,
            })
        }
    }
}

/// Turn a `bridge config` command line into the plan it asks for, or an error
/// naming the first mistake.
///
/// Every rejection happens before any device is opened. The bridge's link is the
/// same one [`plan_bridge`] builds (shared [`plan_bridge_link`]); the speed is
/// what a listed slot would open at, and `--port-count` is only the fallback for
/// a bridge that cannot answer with its own count.
pub fn plan_bridge_config(args: &BridgeConfigArgs) -> Result<BridgeConfigPlan, String> {
    let source = plan_bridge_link(
        &args.bridge,
        &args.bridge_library,
        args.bridge_uuid.as_deref(),
        args.host_identity.as_deref(),
        &args.port_tty,
    )?;
    let baud = args.baud.unwrap_or(DEFAULT_BAUD);
    let port_count = args.port_count.unwrap_or(1);
    if port_count == 0 {
        return Err("--port-count must be at least 1".into());
    }
    Ok(BridgeConfigPlan {
        source,
        baud,
        port_count,
    })
}

/// Turn a `bridge port` command line into the plan it asks for, or an error
/// naming the first mistake.
///
/// Every rejection happens before any device is opened. The bridge's link is the
/// same one [`plan_bridge`] builds (shared [`plan_bridge_link`]); `--port` says
/// which slot to enable and `--port-nodeid` the id the bridge names it with —
/// neither has a guessable default, and the id has to be a usable one (not
/// zero, not the bridge's own link id). `--legacy` picks the wire format the
/// port is named for, and `--assign-uuid` the board to give that id.
pub fn plan_bridge_port(args: &BridgePortArgs) -> Result<BridgePortPlan, String> {
    let source = plan_bridge_link(
        &args.bridge,
        &args.bridge_library,
        args.bridge_uuid.as_deref(),
        args.host_identity.as_deref(),
        &args.port_tty,
    )?;
    // The pair first: an assignment without an id is a half-instruction, and the
    // message should name that pair rather than only the missing half.
    let assign = plan_assign_uuid(args.assign_uuid.as_deref(), args.port_nodeid)?;
    let port = args
        .port
        .ok_or("bridge port needs --port: which declared bridge port the target is on")?;
    let node_id = args
        .port_nodeid
        .ok_or("bridge port needs --port-nodeid: the id the bridge names the target port with")?;
    if node_id == 0 {
        return Err(format!("--port-nodeid must be in 1..=255, got {node_id}"));
    }
    if u32::from(node_id) == BRIDGE_MCU_ID {
        return Err(format!(
            "--port-nodeid {node_id} is the bridge's own link id; the target needs another, \
             or its tunnel units collide with the bridge's frames"
        ));
    }
    Ok(BridgePortPlan {
        source,
        baud: args.baud.unwrap_or(DEFAULT_BAUD),
        bitrate: args.can_bitrate.unwrap_or(CAN_DEFAULT_BITRATE),
        target: BridgeTarget {
            index: port,
            node_id,
        },
        route: if args.legacy {
            PortRoute::Legacy
        } else {
            PortRoute::Units
        },
        assign,
    })
}

/// The `--assign-uuid` line: the identity of the board to give
/// `--port-nodeid`, or the mistake that keeps it from being sent.
///
/// The two go together and neither has a guessable default: an assignment names a
/// board by its UUID and gives it one id, so `--assign-uuid` without
/// `--port-nodeid` is a half-instruction rather than one to complete. The identity
/// itself is the same twelve hex digits `--uuid` takes, most significant byte
/// first — the order the admin `SET_NODEID` frame carries it in.
fn plan_assign_uuid(
    assign_uuid: Option<&str>,
    port_nodeid: Option<u8>,
) -> Result<Option<[u8; 6]>, String> {
    let Some(text) = assign_uuid else {
        return Ok(None);
    };
    if port_nodeid.is_none() {
        return Err("--assign-uuid needs --port-nodeid: the id the named board is given".into());
    }
    parse_hex6(text)
        .map(Some)
        .ok_or_else(|| format!("--assign-uuid expects twelve hex digits, got '{text}'"))
}

/// Turn a `reset` command line into the plan it asks for, or an error naming the
/// first mistake.
///
/// Every rejection happens before any device is opened. The bridge's link is the
/// same one the flash and configure lines build ([`plan_bridge_link`]); the only
/// choice is `--port` (a declared port's device, the way `--port-reset` says) or
/// no `--port` (the bridge itself). `--port-reset` without `--port` is a mistake:
/// the bridge self-restart is the one reset there is.
pub fn plan_reset(args: &ResetArgs) -> Result<ResetPlan, String> {
    let source = plan_bridge_link(
        &args.bridge,
        &args.bridge_library,
        args.bridge_uuid.as_deref(),
        args.host_identity.as_deref(),
        &args.port_tty,
    )?;
    let baud = args.baud.unwrap_or(DEFAULT_BAUD);
    let target = match (args.port, args.bootloader) {
        (Some(_), true) => {
            return Err("--bootloader resets the bridge itself; it takes no --port".into())
        }
        (None, true) => {
            if args.port_reset.is_some() {
                return Err("--bootloader and --port-reset name two different things: \
                            the first parks the bridge in its bootloader, the second \
                            only applies to --port"
                    .into());
            }
            if args.bridge_library.is_some() {
                return Err("--bootloader needs a board: a host library has no bootloader".into());
            }
            ResetTarget::BridgeBootloader
        }
        (None, false) => {
            if args.port_reset.is_some() {
                return Err(
                    "--port-reset needs --port: without one the bridge itself is reset".into(),
                );
            }
            ResetTarget::Bridge
        }
        (Some(index), false) => ResetTarget::Port {
            index,
            reset: parse_port_reset(args.port_reset.as_deref())?,
        },
    };
    Ok(ResetPlan {
        source,
        baud,
        target,
    })
}

/// Parse `--port-reset`: the three resets [`PortReset`] has, in the spelling the
/// port-level commands carry.
fn parse_port_reset(text: Option<&str>) -> Result<PortReset, String> {
    match text {
        None | Some("reset-port") => Ok(PortReset::ResetPort),
        Some("restart-device") => Ok(PortReset::RestartDevice),
        Some("bootloader") => Ok(PortReset::Bootloader),
        Some(other) => Err(format!(
            "--port-reset expects 'reset-port', 'restart-device' or 'bootloader', got '{other}'"
        )),
    }
}

/// Parse `--port-tty`: `<slot>:<path>`, the slot the port's tty is named for.
fn parse_port_tty(spec: &str) -> Result<(u8, String), String> {
    let (slot, path) = spec
        .split_once(':')
        .ok_or_else(|| format!("--port-tty expects <slot>:<path>, got '{spec}'"))?;
    let slot: u8 = slot
        .parse()
        .map_err(|_| format!("--port-tty expects a slot 0..=255 before ':', got '{slot}'"))?;
    if path.is_empty() {
        return Err(format!(
            "--port-tty needs a tty path after ':', got '{spec}'"
        ));
    }
    Ok((slot, path.to_string()))
}

/// Parse `--host-identity`: six bytes as twelve hex digits, most significant
/// byte first — the same grammar `--uuid` uses, under its own name.
fn parse_host_identity(text: &str) -> Result<[u8; 6], String> {
    parse_hex6(text)
        .ok_or_else(|| format!("--host-identity expects twelve hex digits, got '{text}'"))
}

/// Parse `--bridge-uuid`: the same twelve hex digits, most significant byte
/// first — the identity the bridge board is named with.
fn parse_bridge_uuid(text: &str) -> Result<[u8; 6], String> {
    parse_hex6(text).ok_or_else(|| format!("--bridge-uuid expects twelve hex digits, got '{text}'"))
}

/// The `--query` line: a bus to list and nothing that flashes.
fn plan_query(args: &FlashArgs) -> Result<Action, String> {
    if args.bridge.is_some() || args.bridge_library.is_some() {
        return Err("--query lists a bus; --bridge flashes through one".into());
    }
    if args.device.is_some() {
        return Err("--query lists a CAN bus; it takes --interface, not --device".into());
    }
    if args.uuid.is_some() {
        return Err("--query lists every node; --uuid selects one to flash".into());
    }
    if args.nodeid.is_some() {
        return Err("--query assigns no node; --nodeid belongs with --uuid".into());
    }
    if args.baud.is_some() {
        return Err("--query has no serial line; --baud needs --device".into());
    }
    if args.firmware.is_some() {
        return Err("--query lists nodes; it does not flash a firmware file".into());
    }
    if let Some(flag) = stray_bridge_argument(args) {
        return Err(format!("{flag} needs --bridge: it belongs to a bridge"));
    }
    let interface = args
        .interface
        .clone()
        .ok_or("--query needs --interface: which bus to list")?;
    Ok(Action::Query {
        interface,
        window: QUERY_WINDOW,
    })
}

/// The serial line: a device path, an optional line speed, and a firmware
/// file — with nothing CAN-flavoured allowed on the side.
fn plan_tty(args: &FlashArgs, device: &str) -> Result<Action, String> {
    if args.uuid.is_some() {
        return Err("--uuid needs --interface: a serial board is addressed by its port".into());
    }
    if args.nodeid.is_some() {
        return Err("--nodeid needs --interface: a serial board has no CAN node id".into());
    }
    let firmware = firmware_or_err(args)?;
    Ok(Action::Flash {
        target: Target::Tty {
            device: device.to_string(),
            baud: args.baud.unwrap_or(DEFAULT_BAUD),
        },
        firmware,
    })
}

/// The CAN line: a bus, the board's UUID, and the node id to assign it for
/// the session.
fn plan_can(args: &FlashArgs, interface: &str) -> Result<Action, String> {
    if args.baud.is_some() {
        return Err("--baud needs --device: a CAN flash has no line speed".into());
    }
    let uuid = args
        .uuid
        .as_deref()
        .ok_or("--interface needs --uuid: twelve hex digits")?;
    let uuid = parse_uuid(uuid)?;
    let nodeid = args.nodeid.unwrap_or(DEFAULT_NODEID);
    if !(DEFAULT_NODEID..=0xFF).contains(&nodeid) {
        return Err(format!(
            "--nodeid must be in {DEFAULT_NODEID}..=255, got {nodeid}"
        ));
    }
    let firmware = firmware_or_err(args)?;
    Ok(Action::Flash {
        target: Target::Can {
            interface: interface.to_string(),
            uuid,
            nodeid,
        },
        firmware,
    })
}

/// The firmware file every flash line needs.
fn firmware_or_err(args: &FlashArgs) -> Result<PathBuf, String> {
    args.firmware
        .clone()
        .ok_or_else(|| "give a firmware file to flash".to_string())
}

/// Parse `--uuid`: six bytes as twelve hex digits, most significant byte
/// first — the order a `[mcu] uuid:` option is written in, and the order the
/// admin `SET_NODEID` frame carries.
fn parse_uuid(text: &str) -> Result<[u8; 6], String> {
    parse_hex6(text).ok_or_else(|| format!("--uuid expects twelve hex digits, got '{text}'"))
}

/// Six bytes as twelve hex digits, most significant byte first, or `None` —
/// the grammar `--uuid`, `--host-identity` and `--bridge-uuid` share.
fn parse_hex6(text: &str) -> Option<[u8; 6]> {
    let bytes = text.as_bytes();
    if bytes.len() != 12 || !bytes.iter().all(u8::is_ascii_hexdigit) {
        return None;
    }
    let mut value = [0u8; 6];
    for (i, pair) in bytes.chunks(2).enumerate() {
        let text = std::str::from_utf8(pair).expect("ascii hex digits");
        value[i] = u8::from_str_radix(text, 16).expect("ascii hex digits");
    }
    Some(value)
}

/// Run the action a command line asks for: the binary's whole job after
/// parsing. Errors are the tool's exit path — the binary prints them and
/// exits non-zero.
pub fn run(args: FlashArgs) -> Result<(), Box<dyn std::error::Error>> {
    match plan(&args)? {
        Action::Query { interface, window } => run_query(&interface, window),
        Action::Flash { target, firmware } => run_flash(target, &firmware),
        Action::BridgeFlash { target, firmware } => run_bridge_flash(target, &firmware),
    }
}

/// `--query`: ask every node without a node id who it is, and print each
/// answer as it arrives — the same admin broadcast a `scan` runs, with the
/// same two-second deadline window.
fn run_query(interface: &str, window: Duration) -> Result<(), Box<dyn std::error::Error>> {
    let mut bus = CanbusAdminSocket::open(interface)?;
    let scan = canserial::query_unassigned(&mut bus, window, |node| {
        println!(
            "Found uuid={}, Application: {}",
            node.uuid_hex(),
            node.application_name()
        );
    })?;
    println!("{} nodes found", scan.total());
    Ok(())
}

/// One flash: open the named direct link, run the session over it, and
/// report what it did.
fn run_flash(target: Target, firmware: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let payload = std::fs::read(firmware)?;
    let cfg = session::SessionConfig::default();
    match target {
        Target::Tty { device, baud } => {
            let link = transport::open_tty(&device, baud)?;
            let (mut reader, mut writer) = link.split();
            report(session::flash_with_config(
                &mut reader,
                &mut writer,
                &payload,
                &cfg,
                |line| println!("{line}"),
            )?);
        }
        Target::Can {
            interface,
            uuid,
            nodeid,
        } => {
            let (mut reader, mut writer) = transport::open_can(&interface, uuid, nodeid, &cfg)?;
            report(session::flash_with_config(
                &mut reader,
                &mut writer,
                &payload,
                &cfg,
                |line| println!("{line}"),
            )?);
        }
    }
    Ok(())
}

/// One bridge flash: open the bridge's link, identify it, run the session over
/// its tunnel, and report what it did.
///
/// The bridge's connection is asynchronous — `Mcu` drives its transport tasks
/// on the interface's runtime — so this builds a small multi-thread runtime and
/// runs the whole flow on it. The link is opened **inside** the runtime because
/// the `Interface` captures the runtime its device I/O will run on.
fn run_bridge_flash(
    target: BridgeFlashTarget,
    firmware: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    let payload = std::fs::read(firmware)?;
    let cfg = session::SessionConfig::default();
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;
    runtime.block_on(async {
        let mut link = open_bridge_link(&target)?;
        tokio::time::timeout(IDENTIFY_TIMEOUT, link.identify(IDENTIFY_TIMEOUT))
            .await
            .map_err(|_| format!("the bridge did not identify within {IDENTIFY_TIMEOUT:?}"))??;

        // The plan the firmware numbers by arrival order: one entry per port up
        // to and including the target. The target's own reset is the caller's
        // choice; the ports before it are named the one way an unknown device
        // is reset (a line pulse, which needs no admin channel), since the
        // caller's plan describes the target's device and not theirs.
        let ports: Vec<BridgePort> = (0..=target.port)
            .map(|index| BridgePort {
                baud: target.baud,
                bitrate: target.bitrate,
                reset: if index == target.port {
                    target.reset
                } else {
                    PortReset::ResetPort
                },
            })
            .collect();
        let bridge_target = BridgeTarget {
            index: target.port,
            node_id: target.node_id,
        };
        let summary = link
            .flash(&ports, bridge_target, &payload, &cfg, |line| {
                println!("{line}")
            })
            .await?;
        link.close();
        report(summary);
        Ok::<(), Box<dyn std::error::Error>>(())
    })
}

/// Open the bridge's link a [`BridgeFlashTarget`] names: a shared serial tty,
/// or a muxed host library given its identity and port ttys.
fn open_bridge_link(target: &BridgeFlashTarget) -> Result<BridgeLink, BridgeFlashError> {
    open_link(&target.source, target.baud)
}

/// Open a bridge's link: a shared serial tty, or a muxed host library given its
/// identity and port ttys. The one opening `flash` and `admin bridge config` /
/// `admin bridge port` and `admin reset` share.
fn open_link(source: &BridgeLinkSource, baud: u32) -> Result<BridgeLink, BridgeFlashError> {
    match source {
        BridgeLinkSource::Serial { tty, uuid } => {
            BridgeLink::open_serial(Path::new(tty), baud, BRIDGE_MCU_ID, *uuid)
        }
        BridgeLinkSource::Library {
            path,
            identity,
            port_ttys,
        } => BridgeLink::open_host_library(
            Path::new(path),
            HostMux {
                identity: *identity,
                named: *identity,
            },
            BRIDGE_MCU_ID,
            port_ttys,
        ),
    }
}

/// `admin bridge config`: open the bridge, identify it, ask its firmware which
/// UART slots its backend offers — and stop.
///
/// Nothing is declared, reset, named or tunnelled: the bridge is left exactly as
/// it was, and the report says which slots a
/// [`admin bridge port`](BridgePortArgs) can enable. The slots come from the
/// firmware's own `BRIDGE_UART_COUNT`; a bridge that predates the constant is
/// reported as such and falls back to the plan's `--port-count`. The runtime is
/// built here for the same reason [`run_bridge_flash`] builds one — the link is
/// opened inside it.
pub fn run_bridge_config(args: BridgeConfigArgs) -> Result<(), Box<dyn std::error::Error>> {
    let plan = plan_bridge_config(&args)?;
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;
    runtime.block_on(async {
        let link = open_link(&plan.source, plan.baud)?;
        tokio::time::timeout(IDENTIFY_TIMEOUT, link.identify(IDENTIFY_TIMEOUT))
            .await
            .map_err(|_| format!("the bridge did not identify within {IDENTIFY_TIMEOUT:?}"))??;
        let uarts = link.uart_count();
        let slots = link.uart_slots();
        let mcu = link.mcu_name();
        link.close();
        for line in bridge_config_lines(&plan, uarts, slots.as_deref(), mcu.as_deref()) {
            println!("{line}");
        }
        Ok::<(), Box<dyn std::error::Error>>(())
    })
}

/// The last word of a successful `admin bridge config`: the slots the bridge's
/// backend offers, one line each with the speed a port on it would open at, and
/// the command that enables one.
///
/// `uarts` is the firmware's own `BRIDGE_UART_COUNT` — `None` on a firmware that
/// predates the constant, which the report names and then covers with the plan's
/// `--port-count` instead of pretending the bridge answered. `slots` is that
/// same firmware's per-slot naming ([`BridgeLink::uart_slots`]) when it has one,
/// in slot order; a slot it does not name falls back to [`uart_slot_note`].
fn bridge_config_lines(
    plan: &BridgeConfigPlan,
    uarts: Option<u8>,
    slots: Option<&[String]>,
    mcu: Option<&str>,
) -> Vec<String> {
    let mut lines = Vec::new();
    let count = match uarts {
        Some(count) => count,
        None => {
            lines.push(format!(
                "note: this bridge reports no BRIDGE_UART_COUNT (an older firmware); \
                 assuming {} slot(s) from --port-count",
                plan.port_count
            ));
            plan.port_count
        }
    };
    let assumed = if uarts.is_some() { "" } else { " (assumed)" };
    lines.push(format!("available uarts: {count}{assumed}"));
    for slot in 0..count {
        let note = slots
            .and_then(|slots| slots.get(usize::from(slot)))
            .map_or_else(|| uart_slot_note(mcu).to_string(), String::from);
        lines.push(format!(
            "  uart {slot}: {} baud default slot ({note})",
            plan.baud
        ));
    }
    lines.push("enable one with `admin bridge port --port <slot> --port-nodeid <id>`".to_string());
    lines
}

/// How the arch names a port's stream, for a line of `admin bridge config` whose
/// firmware named no slot.
///
/// The dictionary is what a slot is named from when the backend declares it
/// ([`BridgeLink::uart_slots`]); this is the fallback for the backends that
/// declare only a count, so it can say no more than where their stream comes
/// from. `MCU` names the arch (`third_party/klipper/src/{linux,host}/bridge.c`
/// for the two that reach here).
fn uart_slot_note(mcu: Option<&str>) -> &'static str {
    match mcu {
        // `third_party/klipper/src/linux/bridge.c`: the tty the launcher put in
        // `KLIPPER_BRIDGE_PORT_0`.
        Some("linux") => "arch-named tty from KLIPPER_BRIDGE_PORT_0",
        // The host library's slots are ttys its host names (`klipper_host_port`,
        // one per slot) and any other arch's are its own to name, so nothing
        // here can say which: not a USART number, not a path.
        _ => "arch-named",
    }
}

/// `admin bridge port`: open the bridge, identify it, restart it so its RAM port
/// table starts empty, declare the port, name it on the plan's route and open its
/// tunnel — and, when `--assign-uuid` named one, give that board the port's node
/// id — and stop.
///
/// Nothing is reset beyond the bridge's own restart and nothing is flashed. The
/// declaration comes first because a port-level command that names a port the
/// bridge does not have is dropped in silence, and the restart comes before both
/// because the table a previous run built is still there (`config_uart` on an
/// open slot shuts the firmware down). The assignment comes last: it addresses a
/// **board**, by UUID, and the port is what that board's answers will travel
/// once it has an id. The runtime is built here for the same reason
/// [`run_bridge_flash`] builds one — the link is opened inside it.
pub fn run_bridge_port(args: BridgePortArgs) -> Result<(), Box<dyn std::error::Error>> {
    let plan = plan_bridge_port(&args)?;
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;
    runtime.block_on(async {
        let mut link = open_link(&plan.source, plan.baud)?;
        tokio::time::timeout(IDENTIFY_TIMEOUT, link.identify(IDENTIFY_TIMEOUT))
            .await
            .map_err(|_| format!("the bridge did not identify within {IDENTIFY_TIMEOUT:?}"))??;
        link.reset().await?;
        link.declare_ports(&declared_ports(plan.target.index, plan.baud, plan.bitrate))
            .await?;
        link.open_target_port(plan.target, plan.route)?;
        // The slot the port is says what the assignment reached: the admin
        // channel is the bridge's, so it goes to every port, and a UART slot has
        // to say so rather than imply the unit was addressed to it.
        let kind = link.slot_kind(plan.target.index);
        let assigned = match plan.assign {
            Some(uuid) => Some((uuid, link.assign_node_id(uuid, plan.target.node_id)?)),
            None => None,
        };
        link.close();
        report_bridge_port(plan.target, plan.route);
        if let Some((uuid, outcome)) = assigned {
            report_assign(uuid, plan.target, outcome, kind);
        }
        Ok::<(), Box<dyn std::error::Error>>(())
    })
}

/// The declaration `admin bridge port` sends: every port up to and including
/// `index`, at one line speed and one bus rate.
///
/// The bridge numbers a port by the order its declaration rows reach it, so a
/// port at `index` exists only after the ports below it do. Neither rate is a
/// guess for the other: the bridge's own slot table says which kind each port is
/// ([`BridgeLink::slot_kind`]), and only the rate that kind takes is used. No
/// reset runs in this command, so each entry's reset is the unused placeholder
/// [`BridgePort::reset`] carries.
fn declared_ports(index: u8, baud: u32, bitrate: u32) -> Vec<BridgePort> {
    (0..=index)
        .map(|_| BridgePort {
            baud,
            bitrate,
            reset: PortReset::ResetPort,
        })
        .collect()
}

/// The last word of a successful `admin bridge port`: the port the bridge now has,
/// the id it routes it by, the wire format it was named for, and the tunnel it
/// opened.
fn report_bridge_port(target: BridgeTarget, route: PortRoute) {
    let route = match route {
        PortRoute::Units => "units",
        PortRoute::Legacy => "legacy",
    };
    println!(
        "target: port {} nodeid {}, {route} route, tunnel open",
        target.index, target.node_id
    );
}

/// The `--assign-uuid` line of a `admin bridge port` run: which board was given
/// which id, where the unit went, and whether the board answered.
///
/// The two outcomes are reported apart because only one of them is evidence.
/// katapult does not acknowledge `SET_NODEID`, so "sent, no confirmation" is the
/// normal result; a confirmed assignment means a device answered on the id that
/// names it. `kind` is the slot that was enabled, and it decides the scope line:
/// the unit travelled the **bridge's** admin channel, which every port receives,
/// so on a UART slot it reached all of them and the board with that UUID is the one
/// that adopts it (`BridgeLink::slot_kind`).
fn report_assign(uuid: [u8; 6], target: BridgeTarget, outcome: AssignReport, kind: PortKind) {
    let evidence = match outcome {
        AssignReport::Confirmed => "confirmed by the board".to_string(),
        AssignReport::Sent => "sent, no confirmation — katapult does not acknowledge it, which \
                               is normal"
            .to_string(),
    };
    let scope = match kind {
        PortKind::Can => format!("the CAN slot at port {}", target.index),
        PortKind::Uart => format!(
            "port {}, a UART slot — the bridge broadcasts the admin channel to every port, \
             so the assignment went to all of them",
            target.index
        ),
    };
    println!(
        "assign: board {} given nodeid {} via {scope} ({evidence})",
        uuid_hex(uuid),
        target.node_id
    );
}

/// Six bytes as the twelve hex digits the command line spells an identity with,
/// most significant byte first — the same order `--uuid` and `--assign-uuid` take
/// and the scan reports.
fn uuid_hex(uuid: [u8; 6]) -> String {
    uuid.iter().map(|byte| format!("{byte:02x}")).collect()
}

/// `admin reset`: open the bridge, identify it, and reset what the plan names —
/// the bridge itself, or a declared port's device.
///
/// The result line is the confirmation: the bridge's self-reset confirms itself
/// by answering again ([`BridgeLink::reset`]'s bounded reopen + identify), while
/// a port-level reset has no receipt and can only be reported as sent. The
/// runtime is built here for the same reason [`run_bridge_flash`] builds one.
pub fn run_reset(args: ResetArgs) -> Result<(), Box<dyn std::error::Error>> {
    let plan = plan_reset(&args)?;
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;
    runtime.block_on(async {
        let mut link = open_link(&plan.source, plan.baud)?;
        tokio::time::timeout(IDENTIFY_TIMEOUT, link.identify(IDENTIFY_TIMEOUT))
            .await
            .map_err(|_| format!("the bridge did not identify within {IDENTIFY_TIMEOUT:?}"))??;
        match plan.target {
            ResetTarget::Bridge => {
                link.reset().await?;
                report_reset_bridge(&plan.source);
            }
            ResetTarget::BridgeBootloader => {
                link.reset_to_bootloader().await?;
                report_reset_bridge_bootloader();
            }
            ResetTarget::Port { index, reset } => {
                link.reset_port(index, reset)?;
                report_reset_port(index, reset);
            }
        }
        link.close();
        Ok::<(), Box<dyn std::error::Error>>(())
    })
}

/// The last word of a bridge self-reset: the bridge came back and identified
/// again, except for a host library, whose `mcu_restart` is a no-op.
fn report_reset_bridge(source: &BridgeLinkSource) {
    match source {
        BridgeLinkSource::Serial { .. } => {
            println!("reset: bridge (restarted and identified again)")
        }
        BridgeLinkSource::Library { .. } => {
            println!("reset: bridge (host library, restart is a no-op)")
        }
    }
}

/// The last word of a bridge-to-bootloader reset: the request went out with no
/// receipt (the same silence every reset keeps), and the board is what answers
/// on its katapult identity from here on.
fn report_reset_bridge_bootloader() {
    println!(
        "reset: bridge (into its bootloader; write it with `admin flash --device \
         <its usb-katapult_… tty>`)"
    );
}

/// The last word of a port-device reset: the admin unit went out, with no
/// receipt — the same silence a reset has everywhere else.
fn report_reset_port(index: u8, reset: PortReset) {
    let method = match reset {
        PortReset::ResetPort => "reset-port",
        PortReset::RestartDevice => "restart-device",
        PortReset::Bootloader => "bootloader",
    };
    println!("reset: port {index} ({method}), sent with no receipt");
}

/// The last word of a successful flash.
fn report(summary: session::Summary) {
    println!(
        "flashed {} bytes: {} blocks of {} bytes, {}, protocol {}",
        summary.bytes_flashed,
        summary.block_count,
        summary.block_size,
        summary.mcu_type,
        summary.software_version.as_deref().unwrap_or("?"),
    );
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Parse a command line the way the shell would hand it over.
    fn parse(args: &[&str]) -> Result<FlashArgs, clap::Error> {
        FlashArgs::try_parse_from(std::iter::once("flash").chain(args.iter().copied()))
    }

    /// A parsed line, and the plan it makes: the shortcut the cases below
    /// share.
    fn planned(args: &[&str]) -> Result<Action, String> {
        let args = parse(args).expect("a command line clap accepted");
        plan(&args)
    }

    /// A bare `flash` asks what the options are, the same as `klippy`.
    #[test]
    fn test_a_bare_flash_prints_its_help() {
        let error = parse(&[]).expect_err("nothing was asked for");
        assert_eq!(
            error.kind(),
            clap::error::ErrorKind::DisplayHelpOnMissingArgumentOrSubcommand
        );
        assert!(error.to_string().contains("--device"), "{error}");
    }

    /// Two transports on one line is a mistake the arguments can name before
    /// any I/O.
    #[test]
    fn test_two_transports_are_not_one_flash() {
        let err = planned(&["--device", "/dev/ttyACM0", "--interface", "can0", "fw.bin"])
            .expect_err("a board is on one link or the other");
        assert!(err.contains("--device"), "{err}");
        assert!(err.contains("--interface"), "{err}");
    }

    /// Neither transport named is the same question a bare `flash` asks.
    #[test]
    fn test_a_link_is_required() {
        let err = planned(&["fw.bin"]).expect_err("a flash needs a link");
        assert!(err.contains("--device"), "{err}");
    }

    /// The serial line in full: device path, default baud, firmware file.
    #[test]
    fn test_a_serial_flash_plan() {
        let action = planned(&["--device", "/dev/ttyACM0", "fw.bin"]).expect("a serial plan");
        assert_eq!(
            action,
            Action::Flash {
                target: Target::Tty {
                    device: "/dev/ttyACM0".into(),
                    baud: DEFAULT_BAUD,
                },
                firmware: "fw.bin".into(),
            }
        );
    }

    /// An explicit `--baud` rides along; a CAN-only argument does not.
    #[test]
    fn test_an_explicit_baud_is_kept() {
        let action = planned(&["--device", "/dev/ttyUSB0", "--baud", "1000000", "fw.bin"])
            .expect("a serial plan");
        let Action::Flash {
            target: Target::Tty { baud, .. },
            ..
        } = action
        else {
            panic!("a serial line makes a serial plan");
        };
        assert_eq!(baud, 1_000_000);
    }

    /// `--uuid` belongs to a CAN line: a serial board is addressed by its
    /// port.
    #[test]
    fn test_a_uuid_on_a_serial_line_is_named() {
        let err = planned(&[
            "--device",
            "/dev/ttyACM0",
            "--uuid",
            "0123456789ab",
            "fw.bin",
        ])
        .expect_err("a serial board has no uuid to select");
        assert!(err.contains("--uuid"), "{err}");
        assert!(err.contains("--interface"), "{err}");
    }

    /// A CAN node id on a serial line is the same mistake.
    #[test]
    fn test_a_nodeid_on_a_serial_line_is_named() {
        let err = planned(&["--device", "/dev/ttyACM0", "--nodeid", "128", "fw.bin"])
            .expect_err("a serial board has no CAN node id");
        assert!(err.contains("--nodeid"), "{err}");
    }

    /// The CAN line in full: bus, UUID decoded most significant byte first,
    /// the default node id.
    #[test]
    fn test_a_can_flash_plan() {
        let action = planned(&["--interface", "can0", "--uuid", "0123456789ab", "fw.bin"])
            .expect("a CAN plan");
        assert_eq!(
            action,
            Action::Flash {
                target: Target::Can {
                    interface: "can0".into(),
                    uuid: [0x01, 0x23, 0x45, 0x67, 0x89, 0xab],
                    nodeid: DEFAULT_NODEID,
                },
                firmware: "fw.bin".into(),
            }
        );
    }

    /// An explicit `--nodeid` replaces the default, inside the range the
    /// admin command accepts.
    #[test]
    fn test_an_explicit_nodeid_is_kept() {
        let action = planned(&[
            "--interface",
            "can0",
            "--uuid",
            "0123456789ab",
            "--nodeid",
            "200",
            "fw.bin",
        ])
        .expect("a CAN plan");
        let Action::Flash {
            target: Target::Can { nodeid, .. },
            ..
        } = action
        else {
            panic!("a CAN line makes a CAN plan");
        };
        assert_eq!(nodeid, 200);
    }

    /// A node id below the katapult offset belongs to klipper's own admin
    /// protocol and is refused here.
    #[test]
    fn test_a_nodeid_below_the_offset_is_refused() {
        let err = planned(&[
            "--interface",
            "can0",
            "--uuid",
            "0123456789ab",
            "--nodeid",
            "127",
            "fw.bin",
        ])
        .expect_err("node ids below 128 are not katapult's");
        assert!(err.contains("--nodeid"), "{err}");
        assert!(err.contains("128"), "{err}");
    }

    /// A CAN line without a UUID cannot name the board.
    #[test]
    fn test_a_can_flash_needs_a_uuid() {
        let err = planned(&["--interface", "can0", "fw.bin"])
            .expect_err("a CAN board is addressed by uuid");
        assert!(err.contains("--uuid"), "{err}");
    }

    /// A line speed is a serial concept.
    #[test]
    fn test_a_baud_on_a_can_line_is_named() {
        let err = planned(&[
            "--interface",
            "can0",
            "--uuid",
            "0123456789ab",
            "--baud",
            "250000",
            "fw.bin",
        ])
        .expect_err("a CAN flash has no line speed");
        assert!(err.contains("--baud"), "{err}");
    }

    /// A UUID is twelve hex digits or it is an error naming what came
    /// instead.
    #[test]
    fn test_a_uuid_is_twelve_hex_digits() {
        let err = planned(&["--interface", "can0", "--uuid", "0123456789ag", "fw.bin"])
            .expect_err("g is not a hex digit");
        assert!(err.contains("twelve hex digits"), "{err}");

        let err = planned(&["--interface", "can0", "--uuid", "0123456789", "fw.bin"])
            .expect_err("ten digits are not twelve");
        assert!(err.contains("twelve hex digits"), "{err}");
    }

    /// A flash without a firmware file has nothing to send.
    #[test]
    fn test_a_flash_needs_a_firmware_file() {
        let err = planned(&["--device", "/dev/ttyACM0"]).expect_err("nothing to flash");
        assert!(err.contains("firmware"), "{err}");
    }

    /// A line speed is a serial concept — including on a query line.
    #[test]
    fn test_a_query_does_not_take_a_baud() {
        let err = planned(&["-q", "--interface", "can0", "--baud", "250000"])
            .expect_err("a listing has no serial line");
        assert!(err.contains("--baud"), "{err}");
    }

    /// A node id above the range the admin command accepts is refused —
    /// the top of the range is accepted.
    #[test]
    fn test_a_nodeid_above_the_range_is_refused() {
        let err = planned(&[
            "--interface",
            "can0",
            "--uuid",
            "0123456789ab",
            "--nodeid",
            "256",
            "fw.bin",
        ])
        .expect_err("node ids stop at 255");
        assert!(err.contains("--nodeid"), "{err}");
        assert!(err.contains("255"), "{err}");

        let action = planned(&[
            "--interface",
            "can0",
            "--uuid",
            "0123456789ab",
            "--nodeid",
            "255",
            "fw.bin",
        ])
        .expect("the top of the range is a node id");
        let Action::Flash {
            target: Target::Can { nodeid, .. },
            ..
        } = action
        else {
            panic!("a CAN line makes a CAN plan");
        };
        assert_eq!(nodeid, 255);
    }

    /// The firmware file is read before any port is opened: a missing file
    /// is the error that comes back, and the (also missing) serial port is
    /// never touched.
    #[test]
    fn test_the_firmware_file_is_read_before_the_port_is_opened() {
        let args = parse(&["--device", "/dev/not-a-serial-port", "/not/a/firmware.bin"])
            .expect("a command line clap accepted");
        let err = run(args).expect_err("nothing was flashed");
        let text = err.to_string();
        assert!(
            !text.contains("/dev/not-a-serial-port") && !text.contains("serial port"),
            "the port must not be opened before the file is read: {text}"
        );
    }

    /// The query line: a bus to list, and a fixed deadline window.
    #[test]
    fn test_a_query_plan() {
        let action = planned(&["--query", "--interface", "can0"]).expect("a query plan");
        assert_eq!(
            action,
            Action::Query {
                interface: "can0".into(),
                window: QUERY_WINDOW,
            }
        );
    }

    /// The short spelling `-q` is the same line.
    #[test]
    fn test_the_short_query_spelling() {
        let action = planned(&["-q", "--interface", "can0"]).expect("a query plan");
        assert!(matches!(action, Action::Query { .. }));
    }

    /// A query needs a bus to list.
    #[test]
    fn test_a_query_needs_an_interface() {
        let err = planned(&["--query"]).expect_err("which bus?");
        assert!(err.contains("--interface"), "{err}");
    }

    /// A query does not flash: a firmware file on the line is a mistake to
    /// name, not to ignore.
    #[test]
    fn test_a_query_does_not_take_a_firmware_file() {
        let err = planned(&["--query", "--interface", "can0", "fw.bin"])
            .expect_err("a listing is not a flash");
        assert!(err.contains("--query"), "{err}");
    }

    /// A query takes no flashing-only argument either.
    #[test]
    fn test_a_query_takes_no_flash_arguments() {
        let err = planned(&["-q", "--interface", "can0", "--uuid", "0123456789ab"])
            .expect_err("a listing lists everything");
        assert!(err.contains("--uuid"), "{err}");

        let err =
            planned(&["-q", "--interface", "can0", "--nodeid", "128"]).expect_err("no node here");
        assert!(err.contains("--nodeid"), "{err}");

        let err = planned(&["-q", "--device", "/dev/ttyACM0"]).expect_err("no tty here");
        assert!(err.contains("--device"), "{err}");
    }

    /// The serial bridge line in full: the bridge's tty, the port the target
    /// hangs on, the id the bridge names it with, and the firmware.
    #[test]
    fn test_a_serial_bridge_flash_plan() {
        let action = planned(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "1",
            "--port-nodeid",
            "7",
            "fw.bin",
        ])
        .expect("a bridge plan");
        assert_eq!(
            action,
            Action::BridgeFlash {
                target: BridgeFlashTarget {
                    source: BridgeLinkSource::Serial {
                        tty: "/dev/ttyUSB0".into(),
                        uuid: None,
                    },
                    baud: DEFAULT_BAUD,
                    bitrate: CAN_DEFAULT_BITRATE,
                    port: 1,
                    node_id: 7,
                    reset: PortReset::ResetPort,
                },
                firmware: "fw.bin".into(),
            }
        );
    }

    /// The `--port-reset bootloader` spelling on the flash line: the target
    /// port's device is sent into its bootloader ([`PortReset::Bootloader`]),
    /// which is the run [`admin flash`](FlashArgs) then waits for `CONNECT` on.
    #[test]
    fn test_a_bridge_flash_plan_can_send_the_target_to_its_bootloader() {
        let action = planned(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "1",
            "--port-nodeid",
            "7",
            "--port-reset",
            "bootloader",
            "fw.bin",
        ])
        .expect("a bridge plan");
        let Action::BridgeFlash { target, .. } = action else {
            panic!("a bridge line makes a bridge plan");
        };
        assert_eq!(target.reset, PortReset::Bootloader);
    }

    /// The host-library bridge line: the library, its identity, its port ttys,
    /// the target's port and id, and the target port's reset.
    #[test]
    fn test_a_host_library_bridge_flash_plan() {
        let action = planned(&[
            "--bridge-library",
            "/lib/libklipper_host.so",
            "--host-identity",
            "11aa22bb33cc",
            "--port-tty",
            "0:/dev/pts/3",
            "--port-tty",
            "1:/dev/pts/4",
            "--port",
            "1",
            "--port-nodeid",
            "9",
            "--port-reset",
            "restart-device",
            "--baud",
            "115200",
            "fw.bin",
        ])
        .expect("a host-library bridge plan");
        let Action::BridgeFlash { target, firmware } = action else {
            panic!("a bridge line makes a bridge plan");
        };
        assert_eq!(firmware, PathBuf::from("fw.bin"));
        assert_eq!(
            target,
            BridgeFlashTarget {
                source: BridgeLinkSource::Library {
                    path: "/lib/libklipper_host.so".into(),
                    identity: [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc],
                    port_ttys: vec![(0, "/dev/pts/3".into()), (1, "/dev/pts/4".into())],
                },
                baud: 115_200,
                bitrate: CAN_DEFAULT_BITRATE,
                port: 1,
                node_id: 9,
                reset: PortReset::RestartDevice,
            }
        );
    }

    /// A host-library bridge without `--host-identity` answers with the
    /// all-zero identity, the library's own none.
    #[test]
    fn test_a_host_library_bridge_defaults_its_identity_to_zero() {
        let action = planned(&[
            "--bridge-library",
            "/lib/libklipper_host.so",
            "--port",
            "0",
            "--port-nodeid",
            "7",
            "fw.bin",
        ])
        .expect("a host-library bridge plan");
        let Action::BridgeFlash {
            target:
                BridgeFlashTarget {
                    source: BridgeLinkSource::Library { identity, .. },
                    ..
                },
            ..
        } = action
        else {
            panic!("a library line makes a library plan");
        };
        assert_eq!(identity, [0u8; 6]);
    }

    /// A bridge flash names both the port and the id the bridge routes it to:
    /// neither has a guessable default.
    #[test]
    fn test_a_bridge_needs_a_port_and_a_target_id() {
        let err = planned(&["--bridge", "/dev/ttyUSB0", "--port-nodeid", "7", "fw.bin"])
            .expect_err("which port?");
        assert!(err.contains("--port"), "{err}");

        let err =
            planned(&["--bridge", "/dev/ttyUSB0", "--port", "1", "fw.bin"]).expect_err("which id?");
        assert!(err.contains("--port-nodeid"), "{err}");
    }

    /// There is one bridge link or the other, never both.
    #[test]
    fn test_two_bridge_links_are_not_one_flash() {
        let err = planned(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--bridge-library",
            "/lib/libklipper_host.so",
            "--port",
            "0",
            "--port-nodeid",
            "7",
            "fw.bin",
        ])
        .expect_err("one link or the other");
        assert!(err.contains("--bridge"), "{err}");
        assert!(err.contains("--bridge-library"), "{err}");
    }

    /// A bridge line takes the bridge's link, not a direct device or CAN bus.
    #[test]
    fn test_a_bridge_takes_no_device_or_interface() {
        let err = planned(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--device",
            "/dev/ttyACM0",
            "--port",
            "0",
            "--port-nodeid",
            "7",
            "fw.bin",
        ])
        .expect_err("the bridge is the link");
        assert!(err.contains("--device"), "{err}");

        let err = planned(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--interface",
            "can0",
            "--port",
            "0",
            "--port-nodeid",
            "7",
            "fw.bin",
        ])
        .expect_err("the bridge is the link");
        assert!(err.contains("--interface"), "{err}");
    }

    /// A bridge target is named by its port id, not by the CAN line's
    /// `--uuid`/`--nodeid`, and not by the id the bridge's own link uses.
    #[test]
    fn test_a_bridge_takes_no_uuid_or_can_nodeid() {
        let err = planned(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--uuid",
            "11aa22bb33cc",
            "--port",
            "0",
            "--port-nodeid",
            "7",
            "fw.bin",
        ])
        .expect_err("a bridge target has no uuid");
        assert!(err.contains("--port-nodeid"), "{err}");

        let err = planned(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--nodeid",
            "128",
            "--port",
            "0",
            "--port-nodeid",
            "7",
            "fw.bin",
        ])
        .expect_err("a bridge target's id is --port-nodeid");
        assert!(err.contains("--port-nodeid"), "{err}");

        let err = planned(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "0",
            "--port-nodeid",
            "1",
            "fw.bin",
        ])
        .expect_err("the bridge's own link id is not the target's");
        assert!(err.contains("--port-nodeid"), "{err}");
    }

    /// The bridge-only options name a bridge's port, so a direct line that
    /// carries one is told where it belongs.
    #[test]
    fn test_bridge_arguments_need_a_bridge() {
        let err = planned(&["--device", "/dev/ttyACM0", "--port", "1", "fw.bin"])
            .expect_err("no bridge here");
        assert!(err.contains("--port"), "{err}");
        assert!(err.contains("--bridge"), "{err}");
    }

    /// The bridge-only spellings are parsed where they are written: a method
    /// that is not a reset, an identity that is not twelve hex digits, and a
    /// port tty without a slot are each named.
    #[test]
    fn test_bridge_spellings_are_checked() {
        let err = planned(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "0",
            "--port-nodeid",
            "7",
            "--port-reset",
            "reboot",
            "fw.bin",
        ])
        .expect_err("not a reset");
        assert!(err.contains("reset-port"), "{err}");
        assert!(err.contains("restart-device"), "{err}");
        assert!(err.contains("bootloader"), "{err}");

        let err = planned(&[
            "--bridge-library",
            "/lib/libklipper_host.so",
            "--host-identity",
            "nope",
            "--port",
            "0",
            "--port-nodeid",
            "7",
            "fw.bin",
        ])
        .expect_err("not an identity");
        assert!(err.contains("--host-identity"), "{err}");

        let err = planned(&[
            "--bridge-library",
            "/lib/libklipper_host.so",
            "--port-tty",
            "/dev/pts/3",
            "--port",
            "0",
            "--port-nodeid",
            "7",
            "fw.bin",
        ])
        .expect_err("no slot");
        assert!(err.contains("--port-tty"), "{err}");
    }

    /// The library-only options have no meaning on a serial bridge.
    #[test]
    fn test_library_options_need_a_host_library_bridge() {
        let err = planned(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--host-identity",
            "11aa22bb33cc",
            "--port",
            "0",
            "--port-nodeid",
            "7",
            "fw.bin",
        ])
        .expect_err("a serial bridge has no identity");
        assert!(err.contains("--host-identity"), "{err}");

        let err = planned(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port-tty",
            "0:/dev/pts/3",
            "--port",
            "0",
            "--port-nodeid",
            "7",
            "fw.bin",
        ])
        .expect_err("a serial bridge opens its own ports");
        assert!(err.contains("--port-tty"), "{err}");
    }

    /// The serial bridge's own identity rides on the plan when the line names
    /// it: an unassigned bridge answers only the `SET_NODEID` segment, which
    /// needs exactly these twelve hex digits.
    #[test]
    fn test_a_serial_bridge_carries_its_uuid() {
        let action = planned(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--bridge-uuid",
            "11aa22bb33cc",
            "--port",
            "0",
            "--port-nodeid",
            "7",
            "fw.bin",
        ])
        .expect("a serial bridge plan with its identity");
        let Action::BridgeFlash {
            target:
                BridgeFlashTarget {
                    source: BridgeLinkSource::Serial { uuid, .. },
                    ..
                },
            ..
        } = action
        else {
            panic!("a serial line makes a serial plan");
        };
        assert_eq!(uuid, Some([0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc]));
    }

    /// `--bridge-uuid` is checked where it is written — twelve hex digits — and
    /// it belongs to a serial bridge alone: a host library is named by
    /// `--host-identity`.
    #[test]
    fn test_bridge_uuid_is_checked() {
        let err = planned(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--bridge-uuid",
            "nope",
            "--port",
            "0",
            "--port-nodeid",
            "7",
            "fw.bin",
        ])
        .expect_err("not an identity");
        assert!(err.contains("--bridge-uuid"), "{err}");

        let err = planned(&[
            "--bridge-library",
            "/lib/libklipper_host.so",
            "--bridge-uuid",
            "11aa22bb33cc",
            "--port",
            "0",
            "--port-nodeid",
            "7",
            "fw.bin",
        ])
        .expect_err("a host library has no serial bridge board");
        assert!(err.contains("--bridge-uuid"), "{err}");
        assert!(err.contains("--host-identity"), "{err}");

        let err = planned(&[
            "--device",
            "/dev/ttyACM0",
            "--bridge-uuid",
            "11aa22bb33cc",
            "fw.bin",
        ])
        .expect_err("no bridge here");
        assert!(err.contains("--bridge-uuid"), "{err}");
        assert!(err.contains("needs --bridge"), "{err}");
    }

    /// A query lists a bus; it does not flash through a bridge.
    #[test]
    fn test_a_query_takes_no_bridge() {
        let err = planned(&["-q", "--interface", "can0", "--bridge", "/dev/ttyUSB0"])
            .expect_err("a listing is not a flash");
        assert!(err.contains("--bridge"), "{err}");
    }

    /// Parse a `bridge config` command line the way the shell would.
    fn parse_bridge_config(args: &[&str]) -> Result<BridgeConfigArgs, clap::Error> {
        BridgeConfigArgs::try_parse_from(std::iter::once("config").chain(args.iter().copied()))
    }

    /// A parsed `bridge config` line and the plan it makes.
    fn planned_bridge_config(args: &[&str]) -> Result<BridgeConfigPlan, String> {
        let args = parse_bridge_config(args).expect("a command line clap accepted");
        plan_bridge_config(&args)
    }

    /// The serial config plan in full: the link, the speed a slot would open at,
    /// and the count to assume when the bridge cannot say — this command asks the
    /// bridge and stops, so it names no target and opens no tunnel.
    #[test]
    fn test_a_bridge_config_plan() {
        let plan =
            planned_bridge_config(&["--bridge", "/dev/ttyUSB0"]).expect("a serial config plan");
        assert_eq!(
            plan.source,
            BridgeLinkSource::Serial {
                tty: "/dev/ttyUSB0".into(),
                uuid: None,
            }
        );
        assert_eq!(plan.baud, DEFAULT_BAUD);
        assert_eq!(plan.port_count, 1);
    }

    /// The report is the slots the **bridge** offers, one line each, with the
    /// rate a port on it would open at and the firmware's own naming of each
    /// slot — and nothing that claims a port was declared.
    #[test]
    fn test_a_bridge_config_report_lists_the_uarts_the_bridge_offers() {
        let plan = planned_bridge_config(&["--bridge", "/dev/ttyUSB0", "--baud", "115200"])
            .expect("a bridge config plan");
        let slots = vec!["USART1 PA10/PA9".to_string(), "USART2 PA3/PA2".to_string()];
        assert_eq!(
            bridge_config_lines(&plan, Some(2), Some(&slots), Some("stm32f103xe")),
            vec![
                "available uarts: 2".to_string(),
                "  uart 0: 115200 baud default slot (USART1 PA10/PA9)".to_string(),
                "  uart 1: 115200 baud default slot (USART2 PA3/PA2)".to_string(),
                "enable one with `admin bridge port --port <slot> --port-nodeid <id>`".to_string(),
            ]
        );
    }

    /// A slot the firmware names nothing for falls back to the arch's note — the
    /// linux bridge's single tty, and an older firmware whose slots were assumed
    /// from `--port-count` rather than read.
    #[test]
    fn test_a_bridge_config_report_falls_back_to_the_arch_for_an_unnamed_slot() {
        let plan = planned_bridge_config(&["--bridge", "/dev/ttyUSB0", "--baud", "115200"])
            .expect("a bridge config plan");
        assert_eq!(
            bridge_config_lines(&plan, Some(2), None, Some("linux")),
            vec![
                "available uarts: 2".to_string(),
                "  uart 0: 115200 baud default slot (arch-named tty from KLIPPER_BRIDGE_PORT_0)"
                    .to_string(),
                "  uart 1: 115200 baud default slot (arch-named tty from KLIPPER_BRIDGE_PORT_0)"
                    .to_string(),
                "enable one with `admin bridge port --port <slot> --port-nodeid <id>`".to_string(),
            ]
        );
    }

    /// A bridge whose firmware predates `BRIDGE_UART_COUNT` is said to be one, and
    /// the count falls back to `--port-count` rather than pretending the bridge
    /// answered.
    #[test]
    fn test_a_bridge_config_report_falls_back_on_an_old_firmware() {
        let plan = planned_bridge_config(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port-count",
            "3",
            "--baud",
            "250000",
        ])
        .expect("a bridge config plan");
        assert_eq!(
            bridge_config_lines(&plan, None, None, Some("linux")),
            vec![
                "note: this bridge reports no BRIDGE_UART_COUNT (an older firmware); \
                 assuming 3 slot(s) from --port-count"
                    .to_string(),
                "available uarts: 3 (assumed)".to_string(),
                "  uart 0: 250000 baud default slot (arch-named tty from KLIPPER_BRIDGE_PORT_0)"
                    .to_string(),
                "  uart 1: 250000 baud default slot (arch-named tty from KLIPPER_BRIDGE_PORT_0)"
                    .to_string(),
                "  uart 2: 250000 baud default slot (arch-named tty from KLIPPER_BRIDGE_PORT_0)"
                    .to_string(),
                "enable one with `admin bridge port --port <slot> --port-nodeid <id>`".to_string(),
            ]
        );
    }

    /// The arch's note names where an unnamed slot's stream comes from — the
    /// linux tty variable — and a build that names no arch (the host library) or
    /// an arch whose slots are the host's to name says only that much.
    #[test]
    fn test_a_bridge_config_names_the_arch_that_fixes_the_uart() {
        assert_eq!(uart_slot_note(None), "arch-named");
        assert_eq!(
            uart_slot_note(Some("linux")),
            "arch-named tty from KLIPPER_BRIDGE_PORT_0"
        );
        // A stm32 bridge names its own slots in the dictionary (`uart_slots`), so
        // the note for it cannot claim a USART: it says only that the arch does.
        assert_eq!(uart_slot_note(Some("stm32f103xe")), "arch-named");
    }

    /// `--port-count` is the fallback count and `--baud` the speed a slot would
    /// open at: neither declares anything by itself.
    #[test]
    fn test_a_bridge_config_plan_carries_the_fallback_and_the_speed() {
        let plan = planned_bridge_config(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port-count",
            "3",
            "--baud",
            "115200",
        ])
        .expect("a three-slot fallback plan");
        assert_eq!(plan.port_count, 3);
        assert_eq!(plan.baud, 115_200);
    }

    /// The host-library bridge carries its identity and its port ttys, the same
    /// spelling the flash line uses.
    #[test]
    fn test_a_bridge_config_host_library_plan() {
        let plan = planned_bridge_config(&[
            "--bridge-library",
            "/lib/libklipper_host.so",
            "--host-identity",
            "11aa22bb33cc",
            "--port-tty",
            "0:/dev/pts/3",
        ])
        .expect("a host-library config plan");
        assert_eq!(
            plan.source,
            BridgeLinkSource::Library {
                path: "/lib/libklipper_host.so".into(),
                identity: [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc],
                port_ttys: vec![(0, "/dev/pts/3".into())],
            }
        );
    }

    /// A plan of no ports declares nothing, so the count starts at one.
    #[test]
    fn test_a_bridge_config_port_count_is_at_least_one() {
        let err = planned_bridge_config(&["--bridge", "/dev/ttyUSB0", "--port-count", "0"])
            .expect_err("no ports is not a declaration");
        assert!(err.contains("--port-count"), "{err}");
    }

    /// The bridge's link is one of the two, never both and never neither — the
    /// same rule the flash line enforces.
    #[test]
    fn test_a_bridge_config_link_is_required() {
        let err = planned_bridge_config(&["--port-count", "2"]).expect_err("which bridge?");
        assert!(err.contains("--bridge"), "{err}");

        let err = planned_bridge_config(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--bridge-library",
            "/lib/libklipper_host.so",
        ])
        .expect_err("one link or the other");
        assert!(err.contains("--bridge"), "{err}");
        assert!(err.contains("--bridge-library"), "{err}");
    }

    /// Parse a `bridge port` command line the way the shell would.
    fn parse_bridge_port(args: &[&str]) -> Result<BridgePortArgs, clap::Error> {
        BridgePortArgs::try_parse_from(std::iter::once("port").chain(args.iter().copied()))
    }

    /// A parsed `bridge port` line and the plan it makes.
    fn planned_bridge_port(args: &[&str]) -> Result<BridgePortPlan, String> {
        let args = parse_bridge_port(args).expect("a command line clap accepted");
        plan_bridge_port(&args)
    }

    /// The serial port plan in full: the port the declaration names and the id
    /// the bridge routes it to, with no port speed to choose (nothing is
    /// declared here).
    #[test]
    fn test_a_bridge_port_plan() {
        let plan = planned_bridge_port(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "1",
            "--port-nodeid",
            "7",
        ])
        .expect("a serial port plan");
        assert_eq!(
            plan.source,
            BridgeLinkSource::Serial {
                tty: "/dev/ttyUSB0".into(),
                uuid: None,
            }
        );
        assert_eq!(
            plan.target,
            BridgeTarget {
                index: 1,
                node_id: 7,
            }
        );
        // The two port kinds' rates are separate, and neither is the other's
        // default: a UART slot takes the line speed, the CAN slot this bitrate.
        assert_eq!(plan.baud, DEFAULT_BAUD);
        assert_eq!(plan.bitrate, CAN_DEFAULT_BITRATE);
    }

    /// `--can-bitrate` is the **CAN** slot's bus rate and `--baud` stays the UART
    /// slots' line speed: one plan carries both, and declaring a port uses the one
    /// its slot takes (`BridgeLink::slot_kind`).
    #[test]
    fn test_a_bridge_port_plan_carries_the_can_bitrate_beside_the_line_speed() {
        let plan = planned_bridge_port(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "0",
            "--port-nodeid",
            "7",
            "--baud",
            "115200",
            "--can-bitrate",
            "500000",
        ])
        .expect("a CAN slot's rate and a UART slot's speed, on one line");
        assert_eq!(plan.baud, 115_200);
        assert_eq!(plan.bitrate, 500_000);
    }

    /// A bridge **flash** carries the same pair: its plan declares the ports up to
    /// the target, so it needs the CAN slot's rate as much as `bridge port` does.
    #[test]
    fn test_a_bridge_flash_plan_carries_the_can_bitrate() {
        let action = planned(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "1",
            "--port-nodeid",
            "7",
            "--can-bitrate",
            "500000",
            "fw.bin",
        ])
        .expect("a bridge flash plan");
        let Action::BridgeFlash { target, .. } = action else {
            panic!("a bridge line makes a bridge plan");
        };
        assert_eq!(target.bitrate, 500_000);
        assert_eq!(target.baud, DEFAULT_BAUD);
    }

    /// `--can-bitrate` belongs to a bridge: a direct serial or CAN line has no
    /// bridge slot to open, and the option is named as the mistake it is.
    #[test]
    fn test_a_can_bitrate_needs_a_bridge() {
        let err = planned(&[
            "--device",
            "/dev/ttyACM0",
            "--can-bitrate",
            "500000",
            "fw.bin",
        ])
        .expect_err("a serial flash has no bridge CAN slot");
        assert!(err.contains("--can-bitrate"), "{err}");
        assert!(err.contains("--bridge"), "{err}");
    }

    /// Both the port and the id the bridge routes it to are required: neither has
    /// a guessable default.
    #[test]
    fn test_a_bridge_port_needs_a_port_and_a_target_id() {
        let err = planned_bridge_port(&["--bridge", "/dev/ttyUSB0", "--port-nodeid", "7"])
            .expect_err("which port?");
        assert!(err.contains("--port"), "{err}");

        let err = planned_bridge_port(&["--bridge", "/dev/ttyUSB0", "--port", "0"])
            .expect_err("which id?");
        assert!(err.contains("--port-nodeid"), "{err}");
    }

    /// Both numbers are one byte: a port index and a node id past 255 are
    /// refused at the command line, before any plan.
    #[test]
    fn test_a_bridge_port_index_and_id_must_fit_a_byte() {
        let err = parse_bridge_port(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "256",
            "--port-nodeid",
            "7",
        ])
        .expect_err("256 names no port");
        assert!(err.to_string().contains("--port"), "{err}");

        let err = parse_bridge_port(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "0",
            "--port-nodeid",
            "256",
        ])
        .expect_err("256 is past the last node id");
        assert!(err.to_string().contains("--port-nodeid"), "{err}");
    }

    /// The target's node id has to be a usable one: not zero, and not the id the
    /// bridge's own link carries.
    #[test]
    fn test_a_bridge_port_target_id_is_usable() {
        let err = planned_bridge_port(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "0",
            "--port-nodeid",
            "0",
        ])
        .expect_err("zero names no device");
        assert!(err.contains("--port-nodeid"), "{err}");

        let err = planned_bridge_port(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "0",
            "--port-nodeid",
            "1",
        ])
        .expect_err("the bridge's own link id is not the target's");
        assert!(err.contains("--port-nodeid"), "{err}");
    }

    /// `--assign-uuid` names the **board** the port's node id is given to: the
    /// UUID a scan reported, carried as its six bytes most significant first.
    #[test]
    fn test_a_bridge_port_plan_assigns_the_named_board() {
        let plan = planned_bridge_port(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "1",
            "--port-nodeid",
            "128",
            "--assign-uuid",
            "0123456789ab",
        ])
        .expect("an assignment plan");
        assert_eq!(plan.assign, Some([0x01, 0x23, 0x45, 0x67, 0x89, 0xab]));
        assert_eq!(plan.target.node_id, 128, "the id the board is given");

        // Naming a port without naming a board is the other half of the same
        // command line: it declares, names and tunnels, and assigns nothing.
        let plan = planned_bridge_port(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "1",
            "--port-nodeid",
            "7",
        ])
        .expect("a port with no board to assign");
        assert_eq!(plan.assign, None);
    }

    /// `--assign-uuid` without `--port-nodeid` is a half-instruction, and the
    /// message names the pair rather than only the missing half.
    #[test]
    fn test_an_assign_uuid_needs_a_target_id() {
        let err = planned_bridge_port(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "0",
            "--assign-uuid",
            "0123456789ab",
        ])
        .expect_err("an assignment needs an id to give");
        assert!(err.contains("--assign-uuid"), "{err}");
        assert!(err.contains("--port-nodeid"), "{err}");
    }

    /// The identity is the same twelve hex digits `--uuid` takes, and anything
    /// else is a mistyped board rather than one to round.
    #[test]
    fn test_an_assign_uuid_is_twelve_hex_digits() {
        for bad in ["0123456789a", "0123456789abc", "0123456789zz"] {
            let err = planned_bridge_port(&[
                "--bridge",
                "/dev/ttyUSB0",
                "--port",
                "0",
                "--port-nodeid",
                "128",
                "--assign-uuid",
                bad,
            ])
            .expect_err("not an identity");
            assert!(err.contains("--assign-uuid"), "{bad}: {err}");
        }
    }

    /// The bridge's link is one of the two here too.
    #[test]
    fn test_a_bridge_port_link_is_required() {
        let err =
            planned_bridge_port(&["--port", "0", "--port-nodeid", "7"]).expect_err("which bridge?");
        assert!(err.contains("--bridge"), "{err}");

        let err = planned_bridge_port(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--bridge-library",
            "/lib/libklipper_host.so",
            "--port",
            "0",
            "--port-nodeid",
            "7",
        ])
        .expect_err("one link or the other");
        assert!(err.contains("--bridge"), "{err}");
        assert!(err.contains("--bridge-library"), "{err}");
    }

    /// Parse a `reset` command line the way the shell would.
    fn parse_reset(args: &[&str]) -> Result<ResetArgs, clap::Error> {
        ResetArgs::try_parse_from(std::iter::once("reset").chain(args.iter().copied()))
    }

    /// A parsed `reset` line and the plan it makes.
    fn planned_reset(args: &[&str]) -> Result<ResetPlan, String> {
        let args = parse_reset(args).expect("a command line clap accepted");
        plan_reset(&args)
    }

    /// `--bootloader` parks the bridge itself in katapult, which is the state
    /// `admin flash --device` writes it from: the same admin channel as the
    /// restart, with the device half's request instead.
    #[test]
    fn test_a_reset_plan_for_the_bridge_into_its_bootloader() {
        let plan = planned_reset(&["--bridge", "/dev/ttyUSB0", "--bootloader"])
            .expect("a bridge bootloader plan");
        assert_eq!(plan.target, ResetTarget::BridgeBootloader);
    }

    /// The three ways `--bootloader` can be asked for the wrong thing: a port's
    /// reset method, a port index, and a host library (which has no
    /// bootloader). Each is named where the mistake is made.
    #[test]
    fn test_a_bootloader_reset_refuses_the_things_it_is_not() {
        let err = planned_reset(&["--bridge", "/dev/ttyUSB0", "--bootloader", "--port", "0"])
            .expect_err("a port index with --bootloader");
        assert!(err.contains("takes no --port"), "{err}");

        let err = planned_reset(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--bootloader",
            "--port-reset",
            "bootloader",
        ])
        .expect_err("a port reset method with --bootloader");
        assert!(err.contains("--bootloader and --port-reset"), "{err}");

        let err = planned_reset(&[
            "--bridge-library",
            "/lib/libklipper_host.so",
            "--bootloader",
        ])
        .expect_err("a host library has no bootloader");
        assert!(err.contains("has no bootloader"), "{err}");
    }

    /// No `--port` means the bridge itself: the self-restart the flash run does
    /// before it declares ports.
    #[test]
    fn test_a_reset_plan_for_the_bridge_itself() {
        let plan = planned_reset(&["--bridge", "/dev/ttyUSB0"]).expect("a bridge reset plan");
        assert_eq!(plan.target, ResetTarget::Bridge);
        assert_eq!(plan.baud, DEFAULT_BAUD);
        assert_eq!(
            plan.source,
            BridgeLinkSource::Serial {
                tty: "/dev/ttyUSB0".into(),
                uuid: None,
            }
        );
    }

    /// `--port N` resets that port's device, the way `--port-reset` says; the
    /// default is the line pulse.
    #[test]
    fn test_a_reset_plan_for_a_port_device() {
        let plan = planned_reset(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "1",
            "--port-reset",
            "restart-device",
        ])
        .expect("a port reset plan");
        assert_eq!(
            plan.target,
            ResetTarget::Port {
                index: 1,
                reset: PortReset::RestartDevice,
            }
        );

        let plan = planned_reset(&["--bridge", "/dev/ttyUSB0", "--port", "0"])
            .expect("the port's reset defaults to the line pulse");
        assert_eq!(
            plan.target,
            ResetTarget::Port {
                index: 0,
                reset: PortReset::ResetPort,
            }
        );

        // `bootloader` is the third spelling: the one a flash run reaches a
        // running application by.
        let plan = planned_reset(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "2",
            "--port-reset",
            "bootloader",
        ])
        .expect("a port reset plan into the bootloader");
        assert_eq!(
            plan.target,
            ResetTarget::Port {
                index: 2,
                reset: PortReset::Bootloader,
            }
        );
    }

    /// A reset method belongs to a port: without `--port` there is one reset, the
    /// bridge's own, and a method on this line is a mistake to name.
    #[test]
    fn test_a_reset_port_method_without_a_port_is_named() {
        let err = planned_reset(&["--bridge", "/dev/ttyUSB0", "--port-reset", "restart-device"])
            .expect_err("the bridge has one reset");
        assert!(err.contains("--port-reset"), "{err}");
        assert!(err.contains("--port"), "{err}");
    }

    /// The method is spelled like the flash line's `--port-reset`.
    #[test]
    fn test_a_reset_port_method_is_checked() {
        let err = planned_reset(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--port",
            "0",
            "--port-reset",
            "reboot",
        ])
        .expect_err("not a reset");
        assert!(err.contains("reset-port"), "{err}");
        assert!(err.contains("restart-device"), "{err}");
        assert!(err.contains("bootloader"), "{err}");
    }

    /// The bridge's link is one of the two, never both and never neither.
    #[test]
    fn test_a_reset_link_is_required() {
        let err = planned_reset(&["--port", "0"]).expect_err("which bridge?");
        assert!(err.contains("--bridge"), "{err}");

        let err = planned_reset(&[
            "--bridge",
            "/dev/ttyUSB0",
            "--bridge-library",
            "/lib/libklipper_host.so",
        ])
        .expect_err("one link or the other");
        assert!(err.contains("--bridge-library"), "{err}");
    }
}
