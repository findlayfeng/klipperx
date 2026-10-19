//! `klipperx admin scan` — list the boards that have no node id yet, on a CAN bus or
//! a serial port.
//!
//! One command covers both transports because the question is the same one:
//! which boards has a `printer.cfg` still to name? The answer is always a UUID
//! (what a `[mcu]` section then writes in `uuid:`) and an application name. The
//! transport is chosen by the arguments, not by a second command, and it must
//! be named: the subcommand has no default bus, so a bare `klipperx admin scan`
//! prints its help rather than guessing at one.
//!
//! - `--interface <name>` — that CAN bus, as Klipper's
//!   `scripts/canbus_query.py` does: one broadcast of `CMD_QUERY_UNASSIGNED` on
//!   the admin id (`0x3f0`), every board without a node id answering on the
//!   next id (`0x3f1`).
//! - `--port <tty>` — that serial port, where discovery runs **inside** the
//!   byte stream instead: one [`discovery_query`] unit, unseen by the data
//!   plane, and the same answers on the admin reply id.
//! - `--host-library <path.so>` — a klipper host library loaded from disk, the
//!   way a `[mcu] host_library:` section loads one, with `--host-identity`
//!   (twelve hex digits, all zeros by default) naming the device first: the
//!   same [`discovery_query`] unit, over the library's byte stream rather than
//!   its frame API, because a mux unit is not a frame.
//!
//! All three scan the same way underneath: a single broadcast, then a
//! **deadline** window (`--timeout`, two seconds by default) measured from the
//! query — whatever has arrived when it runs out is the answer, and a board
//! that answers late does not extend it.
//!
//! This is a bench tool, not part of the host: it needs no config file, and it
//! takes nothing over. The query changes no board's state — assigning a node id
//! does, and that stays where it is, in the config the host reads. Boards that
//! already have a node id stay quiet, so a scan lists exactly the boards a
//! `printer.cfg` still has to name.
//!
//! The output is upstream's, line for line, except the first word: upstream
//! still writes `canbus_uuid=` (`scripts/canbus_query.py`), this crate prints
//! `uuid=` for its canonical key.
//!
//! ```text
//! Found uuid=11aa22bb33cc, Application: Klipper
//! Total 1 uuids found
//! ```
//!
//! [`discovery_query`]: crate::core::klippy::interface::devices::serial::discovery_query

use clap::Args;
use std::path::Path;
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant};

use crate::core::klippy::interface::devices::canserial::{
    query_unassigned, CanbusAdminSocket, UnassignedNode as CanNode,
};
use crate::core::klippy::interface::devices::host::HostDevice;
use crate::core::klippy::interface::devices::serial::SerialDevice;
use crate::core::klippy::interface::devices::serial::{
    discovery_query, serial_query_unassigned, UnassignedNode as SerialNode, UnassignedScan,
    UnitSink, ADMIN_REPLY, HEADER_BYTES,
};
use crate::flash::cli::CAN_DEFAULT_BITRATE;
use crate::flash::{BridgeLink, BridgePort, PortReset, BRIDGE_MCU_ID, IDENTIFY_TIMEOUT};

/// How long a scan listens for one bridge port's answers, in seconds — the same
/// window a direct scan uses, and the value `--timeout` overrides.
const DEFAULT_TIMEOUT: f64 = 2.0;

/// The line speed a serial scan opens the port at when `--baud` is not given,
/// in baud.
const DEFAULT_BAUD: u32 = 250_000;

/// The quiet window a bridge scan waits out before a broadcast and after it, in
/// milliseconds.
///
/// A bridge's discovery broadcast is fanned out to **every** port it has, so the
/// answers it produces outlive the round that asked for them: whatever is still
/// arriving after a round is dropped rather than reported against the next one,
/// and the one before the first broadcast takes anything left over from an
/// earlier scan out of the stream. Three hundred milliseconds is several times a
/// round trip on the wire a bridge carries.
const BRIDGE_SILENCE_WINDOW: Duration = Duration::from_millis(300);

/// The port indexes a bridge scan enables and scans when `--ports` is not given:
/// the bridge's first port.
const DEFAULT_BRIDGE_PORTS: [u8; 1] = [0];

#[derive(Args, Debug)]
pub struct ScanArgs {
    /// The CAN bus to scan, as the network interface is named (`can0`)
    //
    // One of this and `--port` is required, and there is no default bus: naming
    // neither is not a CAN scan with a guess, it is a question (see the `scan`
    // variant in `main.rs`, which answers a bare `scan` with the help).
    // Optional rather than defaulted also so that giving it *and* `--port` can
    // be named as the mistake it is.
    #[arg(long)]
    pub interface: Option<String>,

    /// Serial port to scan instead of a CAN bus, as a device path
    /// (`/dev/ttyACM0`)
    #[arg(long)]
    pub port: Option<String>,

    /// A klipper host library to scan instead of a bus or a port, as a shared
    /// object path (`libklipper_host.so`)
    #[arg(long)]
    pub host_library: Option<String>,

    /// A **bridge** to scan instead of a bus, a port or a library: the device
    /// path of its console (`/dev/ttyACM0`). Each answer is reported with the
    /// port it came through — the one its bridge appended to it, or the
    /// `--ports` round it arrived in for a bridge that appends none
    #[arg(long, value_name = "TTY")]
    pub bridge: Option<String>,

    /// Identity of the **serial bridge** board itself, twelve hex digits — the
    /// same spelling `flash --bridge-uuid` uses (only with `--bridge`)
    //
    // Optional: without it the bridge is found by the discovery scan when it
    // carries no node id yet, exactly as the other bridge commands find it.
    #[arg(long, value_name = "HEX")]
    pub bridge_uuid: Option<String>,

    /// Bridge port indexes to enable and scan, in order (`0`; only with
    /// `--bridge`)
    ///
    /// All of them are scanned in the same round; only a bridge whose answers
    /// carry no port path needs one round per port to tell them apart.
    #[arg(long, value_name = "INDEX,INDEX")]
    pub ports: Option<String>,

    /// The identity the host library answers the serial mux with: twelve hex
    /// digits, the order a `[mcu] uuid:` option is written in (default
    /// `000000000000`; host library scans only)
    //
    // Optional rather than defaulted, so that giving it without
    // `--host-library` can be named as the mistake it is: the library is what
    // the identity belongs to, and nothing else here has one.
    #[arg(long)]
    pub host_identity: Option<String>,

    /// How long to wait for answers, in seconds
    #[arg(long, default_value_t = DEFAULT_TIMEOUT)]
    pub timeout: f64,

    /// Line speed, in baud (serial and bridge scans; a bridge opens its UART
    /// slots at the same rate)
    #[arg(long)]
    pub baud: Option<u32>,

    /// Bus rate a bridge opens its **CAN** slot at, in bits per second (bridge
    /// scans only; default 1000000)
    //
    // The other kind's rate: a UART slot is declared at `--baud`, the CAN slot at
    // this one, and which slot is which is the bridge's own to say.
    #[arg(long, value_name = "BPS")]
    pub can_bitrate: Option<u32>,
}

/// Entry point for the `scan` subcommand.
///
/// Each board is printed as it answers, so the answers are readable while the
/// window is still open; the total is the last line.
pub fn run(args: ScanArgs) -> Result<(), Box<dyn std::error::Error>> {
    if !args.timeout.is_finite() || args.timeout <= 0.0 {
        return Err("--timeout must be a positive number of seconds".into());
    }
    let window = Duration::from_secs_f64(args.timeout);

    // An identity is what a host library is given, and nothing else on this
    // command has one: giving it alone asks for a device that is not there.
    if args.host_identity.is_some() && args.host_library.is_none() {
        return Err("--host-identity needs --host-library".into());
    }

    // The bridge is a transport of its own, and its own flags only mean
    // something with it: `--bridge-uuid` names the board, `--ports` the ports a
    // scan round enables.
    if args.bridge.is_some() {
        if let Some(other) = bridge_conflict(&args) {
            return Err(format!("give either --bridge or {other}, not both").into());
        }
        let uuid = args
            .bridge_uuid
            .as_deref()
            .map(parse_identity)
            .transpose()?;
        let ports = match args.ports.as_deref() {
            Some(text) => parse_ports(text)?,
            None => DEFAULT_BRIDGE_PORTS.to_vec(),
        };
        let total = bridge_query_unassigned(
            args.bridge.as_deref().expect("checked above"),
            uuid,
            args.baud.unwrap_or(DEFAULT_BAUD),
            args.can_bitrate.unwrap_or(CAN_DEFAULT_BITRATE),
            &ports,
            window,
            |index, node| println!("{}", bridge_line(&node, index)),
        )?;
        println!("{}", total_line(total));
        return Ok(());
    }
    if args.bridge_uuid.is_some() {
        return Err("--bridge-uuid needs --bridge".into());
    }
    if args.can_bitrate.is_some() {
        return Err(
            "--can-bitrate needs --bridge: it is the rate a bridge's CAN slot opens at".into(),
        );
    }
    if args.ports.is_some() {
        return Err("--ports needs --bridge: which bridge's ports to scan".into());
    }

    match (&args.port, &args.interface, &args.host_library) {
        // Two transports named at once is a mistake, not a preference: the
        // serial one is not a fallback for a CAN bus, and neither is a host
        // library.
        (Some(_), Some(_), _) => {
            Err("give either --port (serial) or --interface (CAN), not both".into())
        }
        (Some(_), None, Some(_)) => {
            Err("give either --port (serial) or --host-library, not both".into())
        }
        (None, Some(_), Some(_)) => {
            Err("give either --interface (CAN) or --host-library, not both".into())
        }
        (Some(port), None, None) => {
            let baud = args.baud.unwrap_or(DEFAULT_BAUD);
            let mut device = SerialDevice::open(port, baud)?;
            let total = serial_query_unassigned(&mut device, window, |node| {
                println!("{}", serial_line(&node))
            })?;
            println!("{}", total_line(total));
            Ok(())
        }
        (None, Some(interface), None) => {
            if args.baud.is_some() {
                return Err("--baud needs --port: a CAN scan has no line speed".into());
            }
            let mut bus = CanbusAdminSocket::open(interface)?;
            let scan = query_unassigned(&mut bus, window, |node| println!("{}", can_line(&node)))?;
            println!("{}", total_line(scan.total()));
            Ok(())
        }
        (None, None, Some(library)) => {
            if args.baud.is_some() {
                return Err("--baud needs --port: a host library has no line speed".into());
            }
            let identity = args
                .host_identity
                .as_deref()
                .map(parse_identity)
                .transpose()?;
            let total = host_query_unassigned(library, identity, window, |node| {
                println!("{}", serial_line(&node))
            })?;
            println!("{}", total_line(total));
            Ok(())
        }
        // No transport named: reachable through `ScanArgs` but not from the
        // command line, where `arg_required_else_help` prints the help instead
        // (see the `scan` variant in `main.rs`).
        (None, None, None) => {
            Err("give --interface (CAN), --port (serial), --host-library or --bridge".into())
        }
    }
}

/// The other transport a `--bridge` command line also named, if any — the
/// argument the mistake is named after.
fn bridge_conflict(args: &ScanArgs) -> Option<&'static str> {
    match (&args.interface, &args.port, &args.host_library) {
        (Some(_), _, _) => Some("--interface (CAN)"),
        (_, Some(_), _) => Some("--port (serial)"),
        (_, _, Some(_)) => Some("--host-library"),
        _ => None,
    }
}

/// Parse `--ports`: the bridge port indexes a scan enables and scans, in the
/// order they are given.
///
/// The bridge numbers a port by declaration order, so an index is a slot a
/// backend offers — the same number `admin bridge config` lists — and a repeat is
/// a mistake rather than a second scan of the same port.
fn parse_ports(text: &str) -> Result<Vec<u8>, String> {
    let mut ports: Vec<u8> = Vec::new();
    for part in text.split(',') {
        let index = part.trim().parse::<u8>().map_err(|_| {
            format!(
                "--ports expects bridge port indexes like 0,1, got '{}",
                part.trim()
            )
        })?;
        if ports.contains(&index) {
            return Err(format!("--ports names port {index} twice"));
        }
        ports.push(index);
    }
    if ports.is_empty() {
        return Err("--ports needs at least one bridge port index".into());
    }
    Ok(ports)
}

/// Scan the devices behind `bridge`'s ports, reporting every answer with the
/// port it came through.
///
/// Which port an answer belongs to is the bridge's own to say: a bridge appends
/// the index of the port it heard the answer on to the answer's port path
/// (`mux.h` NEED_NODEID), so this declares every port it names, broadcasts
/// once, and takes each answer's port from the path it carries
/// ([`Attribution::Path`]). One round instead of one per port, and a chain of
/// bridges reads as a chain: each of them appends its own port.
///
/// A bridge whose firmware predates the path appends nothing, and an answer
/// naming no port cannot be attributed that way — a bridge fans every broadcast
/// out to **all** its ports, so such an answer says which device but never
/// where. Those get the rounds a bridge scan used to be alone
/// ([`Attribution::Declaration`]): one port declared per round, a broadcast
/// through the table as it grows, and an answer first heard in a round belongs
/// to that round's port. Both passes report into the same [`UnassignedScan`], so
/// an answer heard by the first is not reported again by the second.
///
/// This needs the bridge's own serial port ([`SharedSerialHandle`]) rather than
/// its frame channel: a downstream device answers on [`ADMIN_REPLY`], an id no
/// MCU owns, so the answers reach a listener registered on the port and never a
/// slot.
///
/// # Errors
/// Whatever opening, identifying or declaring the bridge fails with. A bridge
/// that answers nothing is **not** an error: the scan reports `Total 0 uuids
/// found`, exactly as a quiet bus does.
///
/// [`SharedSerialHandle`]: crate::core::klippy::interface::devices::serial::SharedSerialHandle
fn bridge_query_unassigned(
    tty: &str,
    uuid: Option<[u8; 6]>,
    baud: u32,
    bitrate: u32,
    ports: &[u8],
    window: Duration,
    mut report: impl FnMut(u8, SerialNode),
) -> Result<usize, Box<dyn std::error::Error>> {
    let mut scan = UnassignedScan::new();
    let unattributed = bridge_scan_pass(
        &mut scan,
        Attribution::Path,
        tty,
        uuid,
        baud,
        bitrate,
        ports,
        window,
        &mut report,
    )?;
    if unattributed {
        bridge_scan_pass(
            &mut scan,
            Attribution::Declaration,
            tty,
            uuid,
            baud,
            bitrate,
            ports,
            window,
            &mut report,
        )?;
    }
    Ok(scan.total())
}

/// How a bridge scan attributes an answer to a port.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Attribution {
    /// From the answer's own port path: every port declared up front, one
    /// broadcast, and each answer naming the port it came up on.
    Path,
    /// From the declaration round: one port declared per round, a broadcast
    /// through the table as it grows, and an answer heard for the first time in
    /// a round belongs to that round's port. The fallback for a bridge whose
    /// answers name no port.
    Declaration,
}

/// One scan of `bridge`'s ports — open, identify, clear the port table, declare,
/// broadcast, read — reporting into `scan` and answering whether any answer
/// arrived without a port path.
///
/// Each pass opens its own connection (a port can only be declared once, and the
/// table a previous run built still lives in the firmware's RAM), so the
/// fallback pass starts from an empty table without touching this one's link.
fn bridge_scan_pass(
    scan: &mut UnassignedScan,
    attribution: Attribution,
    tty: &str,
    uuid: Option<[u8; 6]>,
    baud: u32,
    bitrate: u32,
    ports: &[u8],
    window: Duration,
    report: &mut impl FnMut(u8, SerialNode),
) -> Result<bool, Box<dyn std::error::Error>> {
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;
    // The link is opened inside the runtime: its connection captures the
    // runtime's handle for the transport tasks it starts (`Interface::handle`).
    let link = runtime.block_on(async {
        let mut link = BridgeLink::open_serial(Path::new(tty), baud, BRIDGE_MCU_ID, uuid)?;
        tokio::time::timeout(IDENTIFY_TIMEOUT, link.identify(IDENTIFY_TIMEOUT))
            .await
            .map_err(|_| format!("the bridge did not identify within {IDENTIFY_TIMEOUT:?}"))??;
        // A port can only be declared once, and the table a previous run built
        // is still in the firmware's RAM: restart it so this run's declarations
        // start from an empty table.
        link.reset().await?;
        Ok::<BridgeLink, Box<dyn std::error::Error>>(link)
    })?;

    let handle = link
        .shared_handle()
        .expect("a serial bridge is on a shared serial port")
        .clone();
    let (sink, answers) = UnitSink::channel();
    handle.set_unit_sink(ADMIN_REPLY, sink);

    // Anything still arriving from an earlier broadcast is not this run's.
    discard_answers(&answers, BRIDGE_SILENCE_WINDOW);

    let mut unattributed = false;
    let mut declared: u8 = 0;
    match attribution {
        Attribution::Path => {
            for &index in ports {
                declare_ports(&runtime, &link, baud, bitrate, &mut declared, index)?;
            }
            handle.send_admin(&discovery_query()[HEADER_BYTES..])?;
            for unit in collect_answers(&answers, window) {
                let Some(heard) = SerialNode::from_unit(&unit) else {
                    continue;
                };
                if heard.ports.is_empty() {
                    // Not recorded: the fallback pass is what reports it, and a
                    // uuid already in `scan` would be deduplicated there.
                    unattributed = true;
                    continue;
                }
                if let Some(node) = scan.feed(&unit) {
                    // The port of the bridge this scan talks to: the last one
                    // the path names, its own append.
                    let port = *node
                        .ports
                        .last()
                        .expect("an answer with a path names a port");
                    report(port, node);
                }
            }
        }
        Attribution::Declaration => {
            for &index in ports {
                declare_ports(&runtime, &link, baud, bitrate, &mut declared, index)?;
                discard_answers(&answers, BRIDGE_SILENCE_WINDOW);
                handle.send_admin(&discovery_query()[HEADER_BYTES..])?;
                for unit in collect_answers(&answers, window) {
                    if let Some(node) = scan.feed(&unit) {
                        report(index, node);
                    }
                }
                // A late answer belongs to no port this run could name, so it is
                // dropped here instead of being reported against the next one.
                discard_answers(&answers, BRIDGE_SILENCE_WINDOW);
            }
        }
    }
    handle.clear_unit_sink(ADMIN_REPLY);
    link.close();
    Ok(unattributed)
}

/// Declare the bridge's ports up to `last`, in ascending index — the bridge
/// numbers a port by the order its declaration rows reach it, and a port can
/// only be declared once — remembering how many are declared. Each port is a plan
/// entry with both rates; the bridge's own slot table says which one its bus takes
/// ([`BridgeLink::declare_port`]), and the reset is the unused placeholder
/// [`BridgePort::reset`] carries.
fn declare_ports(
    runtime: &tokio::runtime::Runtime,
    link: &BridgeLink,
    baud: u32,
    bitrate: u32,
    declared: &mut u8,
    last: u8,
) -> Result<(), Box<dyn std::error::Error>> {
    while *declared <= last {
        let port = BridgePort {
            baud,
            bitrate,
            reset: PortReset::ResetPort,
        };
        runtime.block_on(link.declare_port(*declared, port))?;
        *declared += 1;
    }
    Ok(())
}

/// The units the scan listener collects within `window`, in arrival order.
fn collect_answers(answers: &Receiver<Vec<u8>>, window: Duration) -> Vec<Vec<u8>> {
    let deadline = Instant::now() + window;
    let mut units = Vec::new();
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return units;
        }
        match answers.recv_timeout(remaining) {
            Ok(unit) => units.push(unit),
            Err(_) => return units,
        }
    }
}

/// Drop whatever the scan listener collects for `window` — the quiet window
/// between two rounds.
fn discard_answers(answers: &Receiver<Vec<u8>>, window: Duration) {
    let deadline = Instant::now() + window;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return;
        }
        if answers.recv_timeout(remaining).is_err() {
            return;
        }
    }
}

/// Ask every board with no node id behind the host `library` to identify
/// itself, naming the device `identity` first, and read the answers for
/// `window`.
///
/// The same scan as [`serial_query_unassigned`] — one [`discovery_query`] unit,
/// then the same framing, deadline, dedup and reporting — because a host
/// library is another byte stream a mux unit travels on: reading it as bytes is
/// what the library device's scan mode is for. `report` is called as each board
/// is first heard; returns how many answered.
///
/// [`discovery_query`]: crate::core::klippy::interface::devices::serial::discovery_query
fn host_query_unassigned(
    library: &str,
    identity: Option<[u8; 6]>,
    window: Duration,
    report: impl FnMut(SerialNode),
) -> Result<usize, Box<dyn std::error::Error>> {
    let mut device = HostDevice::load_bytes(library, identity)?;
    Ok(serial_query_unassigned(&mut device, window, report)?)
}

/// Parse `--host-identity`: six bytes as twelve hex digits.
///
/// The same spelling a `[mcu] uuid:` option uses, with nothing else accepted:
/// the identity goes on the wire as six bytes, so a value that is not twelve
/// hex digits is a mistyped identity rather than one to round.
fn parse_identity(text: &str) -> Result<[u8; 6], String> {
    let digits = text.trim().as_bytes();
    if digits.len() != 12 || !digits.iter().all(u8::is_ascii_hexdigit) {
        return Err(format!(
            "--host-identity expects twelve hex digits, got '{}'",
            text.trim()
        ));
    }
    let mut identity = [0u8; 6];
    for (i, byte) in identity.iter_mut().enumerate() {
        // Both digits are ASCII hex (checked above), so this cannot fail.
        let pair = std::str::from_utf8(&digits[i * 2..i * 2 + 2]).expect("two ASCII digits");
        *byte = u8::from_str_radix(pair, 16).expect("hexadecimal");
    }
    Ok(identity)
}

/// One CAN answer, as Klipper's own tool prints it.
fn can_line(node: &CanNode) -> String {
    found_line(node.uuid_hex(), node.application_name())
}

/// One serial answer, in the same shape (the serial node keeps its UUID as
/// bytes, so the twelve hex digits are formatted here), and the port path the
/// answer travelled when it names one.
fn serial_line(node: &SerialNode) -> String {
    let line = found_line(uuid_hex(node.uuid), node.application_name());
    match node.ports.as_slice() {
        [] => line,
        [port] => format!("{line}, via port {port}"),
        ports => {
            let chain: Vec<String> = ports.iter().map(u8::to_string).collect();
            format!("{line}, via ports {}", chain.join(">"))
        }
    }
}

/// One answer heard behind a bridge: the line every transport prints, plus the
/// port it came through.
///
/// An answer that carries a port path says so itself ([`serial_line`]): the
/// bridge it came through appended its own port index. One that carries none can
/// only be attributed by the round it arrived in, and `round_port` is that
/// round's port — the reading a bridge scan had before the path existed.
fn bridge_line(node: &SerialNode, round_port: u8) -> String {
    if node.ports.is_empty() {
        format!("{}, port {round_port}", serial_line(node))
    } else {
        serial_line(node)
    }
}

/// One answer, as Klipper's own tool prints it: twelve hex digits of UUID with
/// their leading zeros, and the application's name.
fn found_line(uuid_hex: impl AsRef<str>, application_name: &str) -> String {
    format!(
        "Found uuid={}, Application: {application_name}",
        uuid_hex.as_ref()
    )
}

/// The UUID as twelve lowercase hex digits with their leading zeros, summed the
/// way Klipper's own tool sums it — the byte after the answer's command is the
/// most significant one.
fn uuid_hex(uuid: [u8; 6]) -> String {
    let number = uuid
        .iter()
        .fold(0u64, |value, byte| (value << 8) | u64::from(*byte));
    format!("{number:012x}")
}

/// The scan's closing line: how many boards answered.
///
/// Upstream's own line is `Total %d uuids found`, but its call site passes
/// `len(found_ids,)`, which raises before printing anything; the count it means
/// is printed here.
fn total_line(total: usize) -> String {
    format!("Total {total} uuids found")
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::interface::devices::canserial::{query_unassigned_frame, CanFrame};

    /// The window the host scan tests listen for: long enough that the firmware
    /// would have answered if it were going to, short enough not to spend the
    /// default two seconds per test.
    const HOST_SCAN_WINDOW: Duration = Duration::from_millis(300);

    /// The arguments a bare `scan` gets: every transport and option at its
    /// default, which is the shape the command line fills in.
    fn default_args() -> ScanArgs {
        ScanArgs {
            interface: None,
            port: None,
            host_library: None,
            bridge: None,
            bridge_uuid: None,
            ports: None,
            host_identity: None,
            timeout: DEFAULT_TIMEOUT,
            baud: None,
            can_bitrate: None,
        }
    }

    /// A **private copy** of the real host library, so a scan test that has to
    /// load it does not share the per-path singleton the other library tests
    /// register (the registry in `interface/devices/host.rs` is per path) and may
    /// run beside them. Not a mux firmware: the firmware's mux device half is not
    /// in this build, so it answers no scan.
    fn private_host_library() -> klipperx_test_support::PrivateCopy {
        klipperx_test_support::PrivateCopy::of(&klipperx_test_support::klipper_host_lib_path())
            .expect("the private library copy is made")
    }

    /// A CAN answer, as the admin socket would report it.
    fn can_node(uuid: [u8; 6], application: Option<u8>) -> CanNode {
        let mut data = vec![0x20];
        data.extend_from_slice(&uuid);
        data.extend(application);
        let frame = CanFrame::new(query_unassigned_frame().id() + 1, &data).expect("an answer");
        CanNode::from_reply(&frame).expect("this is an answer")
    }

    /// A serial answer, as the port stream would report it.
    fn serial_node(uuid: [u8; 6], application: Option<u8>) -> SerialNode {
        SerialNode {
            uuid,
            application,
            ports: Vec::new(),
        }
    }

    /// A serial answer that came up through the bridges `ports` names, innermost
    /// first.
    fn serial_node_via(uuid: [u8; 6], ports: &[u8]) -> SerialNode {
        SerialNode {
            uuid,
            application: None,
            ports: ports.to_vec(),
        }
    }

    #[test]
    fn test_both_transports_print_the_same_line() {
        // The two scans report through different types (one per transport), so
        // the printer is what makes the output one shape: the same twelve
        // digits and application name, whichever bus answered. Upstream still
        // prints `canbus_uuid=%012x` here (`scripts/canbus_query.py`); this
        // crate has renamed the key to `uuid`, so the line is a deliberate
        // deviation.
        for (uuid, application, expected) in [
            (
                [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc],
                Some(0x01),
                "Found uuid=11aa22bb33cc, Application: Klipper",
            ),
            (
                [0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff],
                Some(0x11),
                "Found uuid=aabbccddeeff, Application: CanBoot",
            ),
            // The leading zeros of a small UUID are part of the twelve digits.
            (
                [0, 0, 0, 0, 0, 1],
                None,
                "Found uuid=000000000001, Application: Klipper",
            ),
            (
                [0x02, 0x03, 0x04, 0x05, 0x06, 0x07],
                Some(0x42),
                "Found uuid=020304050607, Application: Unknown",
            ),
        ] {
            assert_eq!(can_line(&can_node(uuid, application)), expected);
            assert_eq!(serial_line(&serial_node(uuid, application)), expected);
        }
    }

    #[test]
    fn test_the_total_line_counts_the_boards() {
        assert_eq!(total_line(0), "Total 0 uuids found");
        assert_eq!(total_line(3), "Total 3 uuids found");
    }

    #[test]
    fn test_an_answer_behind_a_bridge_names_the_port_it_came_through() {
        // The port is the answer's own: the bridge appended it, so the line says
        // it — and a change of depth does not: a directly wired board prints what
        // it always did.
        assert_eq!(
            serial_line(&serial_node_via([0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc], &[0])),
            "Found uuid=11aa22bb33cc, Application: Klipper, via port 0"
        );
        // Two bridges passed this one, innermost first: the chain reads in that
        // order, `>` between the hops.
        assert_eq!(
            serial_line(&serial_node_via(
                [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc],
                &[0, 2]
            )),
            "Found uuid=11aa22bb33cc, Application: Klipper, via ports 0>2"
        );
    }

    #[test]
    fn test_a_bridge_answer_without_a_path_keeps_the_round_port() {
        // A bridge that appends no path (`Attribution::Declaration`'s fallback)
        // leaves the answer attributable only by the round it arrived in, which
        // is the line a bridge scan printed before the path existed — and the
        // path, when there is one, is not printed twice.
        let uuid = [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc];
        assert_eq!(
            bridge_line(&serial_node_via(uuid, &[]), 2),
            "Found uuid=11aa22bb33cc, Application: Klipper, port 2"
        );
        assert_eq!(
            bridge_line(&serial_node_via(uuid, &[0]), 2),
            serial_line(&serial_node_via(uuid, &[0]))
        );
    }

    #[test]
    fn test_a_timeout_that_is_not_a_positive_number_is_rejected() {
        // Rejected before anything is opened — a port or a socket — so this
        // runs without hardware: a zero or negative window would ask for
        // nothing (or for a wait the deadline loop cannot express).
        for bad in ["0", "-1", "nan", "inf"] {
            let args = ScanArgs {
                port: Some("/dev/not-a-port".to_string()),
                timeout: bad.parse().expect("a number"),
                ..default_args()
            };
            let err = run(args).expect_err("a window of nothing is not a scan");
            assert!(err.to_string().contains("--timeout"), "{err}");
        }
    }

    #[test]
    fn test_a_scan_without_a_transport_is_rejected() {
        // No default bus: through `ScanArgs` this is an error (on the command
        // line `arg_required_else_help` never gets this far — a bare `scan`
        // prints the help).
        let err = run(default_args()).expect_err("a scan has to be told what to scan");
        assert!(err.to_string().contains("--interface"), "{err}");
    }

    #[test]
    fn test_a_port_and_an_interface_together_are_rejected() {
        // Both named is a mistake the arguments can name before any I/O: the
        // serial port is not a fallback for the CAN bus, it is another bus.
        let args = ScanArgs {
            interface: Some("can0".to_string()),
            port: Some("/dev/not-a-port".to_string()),
            ..default_args()
        };
        let err = run(args).expect_err("two transports are not one scan");
        assert!(err.to_string().contains("not both"), "{err}");
    }

    #[test]
    fn test_a_baud_without_a_port_is_rejected() {
        // A line speed only means something on a serial port; on a CAN bus it
        // is a mistake rather than an unused option, and is caught before the
        // socket is opened.
        let args = ScanArgs {
            interface: Some("can0".to_string()),
            baud: Some(115_200),
            ..default_args()
        };
        let err = run(args).expect_err("a CAN scan has no line speed");
        assert!(err.to_string().contains("--baud"), "{err}");
    }

    #[test]
    fn test_a_host_identity_without_a_host_library_is_rejected() {
        // An identity belongs to a host library and nothing else here has one:
        // named alone it asks for a device that is not there, and is caught
        // before any library is loaded.
        let args = ScanArgs {
            host_identity: Some("11aa22bb33cc".to_string()),
            ..default_args()
        };
        let err = run(args).expect_err("an identity needs a library to name");
        assert!(err.to_string().contains("--host-identity"), "{err}");
    }

    #[test]
    fn test_a_host_library_and_a_port_together_are_rejected() {
        // A host library is a third transport, not a fallback for the serial
        // port: naming both is a mistake, caught before the port is opened.
        let args = ScanArgs {
            port: Some("/dev/not-a-port".to_string()),
            host_library: Some("/nonexistent/libklipper_host.so".to_string()),
            ..default_args()
        };
        let err = run(args).expect_err("two transports are not one scan");
        assert!(err.to_string().contains("not both"), "{err}");
    }

    #[test]
    fn test_a_baud_with_a_host_library_is_rejected() {
        // The line speed only means something on a serial port; a host library
        // has no line of its own, and is caught before it is loaded.
        let args = ScanArgs {
            host_library: Some("/nonexistent/libklipper_host.so".to_string()),
            baud: Some(115_200),
            ..default_args()
        };
        let err = run(args).expect_err("a host library has no line speed");
        assert!(err.to_string().contains("--baud"), "{err}");
    }

    #[test]
    fn test_the_identity_is_twelve_hex_digits() {
        // The identity goes on the wire as six bytes, most significant first,
        // the order a `[mcu] uuid:` option is written in.
        assert_eq!(
            parse_identity("11aa22bb33cc").expect("twelve hex digits"),
            [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc]
        );
        assert_eq!(
            parse_identity("000000000000").expect("the default identity"),
            [0u8; 6]
        );
        // Anything else is a mistyped identity, not one to guess at — including
        // a string whose bytes are not all ASCII, which cannot be cut into
        // digit pairs at all.
        for bad in [
            "",
            "11aa22bb33c",
            "11aa22bb33ccd",
            "11aa22bb33cg",
            "11aa22bb33日",
        ] {
            let err = parse_identity(bad).expect_err("not twelve hex digits");
            assert!(err.contains("--host-identity"), "{err}");
        }
    }

    #[test]
    fn test_a_host_scan_ends_with_no_boards_against_the_real_library() {
        // The end-to-end shape of a host scan against a real library: the unit
        // goes into the library, the window runs out, and the total is zero.
        // This is the whole answer a **non-mux** firmware can give — the test
        // library has no mux device half — so it pins both that the scan is
        // harmless there (no error, no garbage answer) and that nothing is
        // reported for a device that never answered.
        let copy = private_host_library();
        let library = copy.path().display().to_string();
        let mut heard = Vec::new();
        let total =
            host_query_unassigned(&library, None, HOST_SCAN_WINDOW, |node| heard.push(node))
                .expect("a scan runs against the real library");
        assert_eq!(total, 0);
        assert_eq!(total_line(total), "Total 0 uuids found");
        assert!(heard.is_empty(), "a device that never answered is listed");
    }

    #[test]
    fn test_a_host_scan_with_an_identity_runs_against_the_real_library() {
        // The identity is set before klipper comes up, so a host scan that
        // names the device is the same empty scan — and the library that has
        // `klipper_host_set_uuid` is not refused (an identity is only an error
        // for one built without it).
        let copy = private_host_library();
        let library = copy.path().display().to_string();
        let mut heard = Vec::new();
        let total = host_query_unassigned(
            &library,
            Some(parse_identity("11aa22bb33cc").expect("twelve hex digits")),
            HOST_SCAN_WINDOW,
            |node| heard.push(node),
        )
        .expect("a named host library scans");
        assert_eq!(total_line(total), "Total 0 uuids found");
        assert!(heard.is_empty(), "a device that never answered is listed");
    }

    #[test]
    fn test_a_host_library_scan_prints_its_total_and_succeeds() {
        // The command-line wiring, end to end: `--host-library` selects the
        // scan, an all-zero identity is the default the device already has, and
        // an empty window is a successful scan rather than an error.
        let copy = private_host_library();
        let args = ScanArgs {
            host_library: Some(copy.path().display().to_string()),
            host_identity: Some("000000000000".to_string()),
            timeout: HOST_SCAN_WINDOW.as_secs_f64(),
            ..default_args()
        };
        run(args).expect("an empty host scan is a scan, not an error");
    }

    // =======================================================================
    // Bridge scans
    // =======================================================================

    /// The identity the bridge fixture is started with, and the one its link is
    /// named by.
    const BRIDGE_SCAN_UUID: [u8; 6] = [0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6];

    /// The identity the device behind the bridge port answers with.
    const DOWNSTREAM_UUID: [u8; 6] = [0x0d, 0x1e, 0x2f, 0x3a, 0x4b, 0x5c];

    /// The window one bridge port's answers get in the tests: long enough for a
    /// firmware on a pty, short enough not to sit out the default two seconds.
    const BRIDGE_SCAN_WINDOW: Duration = Duration::from_millis(500);

    /// A bridge with a **mux device** behind its port: the scan enables the port,
    /// broadcasts through it, and reports the device's answer with the port index
    /// it came through.
    ///
    /// Two real firmware processes: the device (`linuxprocess-mux`, given the
    /// identity it answers with) and the bridge (`linuxprocess-bridge`), whose
    /// downstream port is the device's own console pty — the device holds the
    /// master and the bridge opens the slave, exactly as a board wired to the
    /// port would sit there.
    #[test]
    fn test_a_bridge_scan_reports_a_downstream_mux_device_with_its_port() {
        let device = klipperx_test_support::LinuxFirmware::spawn_named(DOWNSTREAM_UUID)
            .expect("the mux firmware process starts");
        let bridge = klipperx_test_support::LinuxFirmware::spawn_bridge_port(
            device.serial_path(),
            BRIDGE_SCAN_UUID,
        )
        .expect("the bridge firmware process starts");

        let mut heard: Vec<(u8, SerialNode)> = Vec::new();
        let total = bridge_query_unassigned(
            bridge.serial_path().to_str().expect("a utf-8 path"),
            Some(BRIDGE_SCAN_UUID),
            DEFAULT_BAUD,
            CAN_DEFAULT_BITRATE,
            &[0],
            BRIDGE_SCAN_WINDOW,
            |index, node| heard.push((index, node)),
        )
        .expect("the bridge scans");

        assert_eq!(
            total, 1,
            "the one device behind the bridge's port 0 answered"
        );
        assert_eq!(heard.len(), 1);
        assert_eq!(
            heard[0].0, 0,
            "the answer is attributed to the port it came through"
        );
        assert_eq!(heard[0].1.uuid, DOWNSTREAM_UUID);
        assert_eq!(
            heard[0].1.ports,
            vec![0],
            "the port is the answer's own, appended by the bridge that heard it \
             (`mux.h` NEED_NODEID): the device sits on port 0 of the bridge this \
             scan talks to"
        );
    }

    /// The frame one bridged answer travels in, byte for byte: appending the
    /// port means building the frame again, so its length byte, its CRC over
    /// `[len][seq][payload]` and its trailer are the new payload's own. Read off
    /// a real bridge's console — the only place that frame exists — with the
    /// bridge's port declared the way a scan declares it.
    ///
    /// This is the firmware half's pin: the host's [`SerialNode::from_unit`]
    /// checks the CRC it reads, so the scan test above already fails on a frame
    /// left with its old CRC — here the bytes are checked directly instead of
    /// through a parser.
    #[test]
    fn test_a_bridged_answer_is_one_frame_rebuilt_around_its_appended_path() {
        use crate::core::klippy::frame::{
            MESSAGE_HEADER_SIZE, MESSAGE_POS_LEN, MESSAGE_POS_SEQ, MESSAGE_SYNC,
            MESSAGE_TRAILER_SIZE,
        };
        use crate::core::klippy::interface::devices::serial::{
            encode_header, ASSIGN_SEQ_BYTE, MAGIC, RESP_NEED_NODEID,
        };

        let device = klipperx_test_support::LinuxFirmware::spawn_named(DOWNSTREAM_UUID)
            .expect("the mux firmware process starts");
        let bridge = klipperx_test_support::LinuxFirmware::spawn_bridge_port(
            device.serial_path(),
            BRIDGE_SCAN_UUID,
        )
        .expect("the bridge firmware process starts");

        let unit = bridge_answer_unit(bridge.serial_path(), BRIDGE_SCAN_UUID);
        assert_eq!(
            &unit[..HEADER_BYTES],
            &encode_header(ADMIN_REPLY)[..],
            "the answer is on the admin reply id"
        );
        let frame = &unit[HEADER_BYTES..];
        let len = frame[MESSAGE_POS_LEN] as usize;
        assert_eq!(
            frame.len(),
            len,
            "the length byte counts the bytes of the rebuilt frame"
        );
        assert_eq!(frame[MESSAGE_POS_SEQ], ASSIGN_SEQ_BYTE);
        assert_eq!(frame[len - 1], MESSAGE_SYNC, "the trailer is its own SYNC");
        let content = &frame[MESSAGE_HEADER_SIZE..len - MESSAGE_TRAILER_SIZE];
        assert_eq!(&content[..MAGIC.len()], &MAGIC[..], "the answer rides KXID");
        let mut expected = vec![RESP_NEED_NODEID];
        expected.extend_from_slice(&DOWNSTREAM_UUID);
        expected.extend_from_slice(&[0x81, 0x00]); // one port, index 0
        assert_eq!(
            &content[MAGIC.len()..],
            &expected[..],
            "the payload is the device's answer with the bridge's port appended"
        );
        let crc = crate::crc16_ccitt!(&frame[..len - MESSAGE_TRAILER_SIZE]);
        assert_eq!(
            ((frame[len - 3] as u16) << 8) | frame[len - 2] as u16,
            crc,
            "the CRC covers the appended port: a frame still carrying the CRC \
             of the payload it arrived with would fail here"
        );
    }

    /// One unit read off a bridge's console, as the bytes that arrived: the
    /// bridge identified, its port 0 declared, one discovery broadcast, and the
    /// first unit to come up on the admin reply id.
    fn bridge_answer_unit(tty: &Path, uuid: [u8; 6]) -> Vec<u8> {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .expect("a runtime for the bridge link");
        let link = runtime
            .block_on(async {
                let mut link =
                    BridgeLink::open_serial(tty, DEFAULT_BAUD, BRIDGE_MCU_ID, Some(uuid))?;
                tokio::time::timeout(IDENTIFY_TIMEOUT, link.identify(IDENTIFY_TIMEOUT))
                    .await
                    .map_err(|_| {
                        format!("the bridge did not identify within {IDENTIFY_TIMEOUT:?}")
                    })??;
                link.reset().await?;
                Ok::<BridgeLink, Box<dyn std::error::Error>>(link)
            })
            .expect("the bridge answers an identify");
        let handle = link
            .shared_handle()
            .expect("a serial bridge is on a shared serial port")
            .clone();
        let (sink, answers) = UnitSink::channel();
        handle.set_unit_sink(ADMIN_REPLY, sink);
        runtime
            .block_on(link.declare_port(
                0,
                BridgePort {
                    baud: DEFAULT_BAUD,
                    bitrate: CAN_DEFAULT_BITRATE,
                    reset: PortReset::ResetPort,
                },
            ))
            .expect("the bridge declares its port");
        handle
            .send_admin(&discovery_query()[HEADER_BYTES..])
            .expect("the query goes in");
        let unit = answers
            .recv_timeout(Duration::from_secs(5))
            .expect("the device behind the port answers");
        handle.clear_unit_sink(ADMIN_REPLY);
        link.close();
        unit
    }

    /// A bridge whose port has nothing behind it answers nothing — and that is a
    /// scan that succeeded, exactly as a quiet bus is: no error, and the total is
    /// zero.
    #[test]
    fn test_a_bridge_scan_with_nothing_behind_the_port_succeeds() {
        let port = crate::core::klippy::interface::pty::Pty::new();
        let bridge =
            klipperx_test_support::LinuxFirmware::spawn_bridge_port(&port.path, BRIDGE_SCAN_UUID)
                .expect("the bridge firmware process starts");

        let mut heard = Vec::new();
        let total = bridge_query_unassigned(
            bridge.serial_path().to_str().expect("a utf-8 path"),
            Some(BRIDGE_SCAN_UUID),
            DEFAULT_BAUD,
            CAN_DEFAULT_BITRATE,
            &[0],
            BRIDGE_SCAN_WINDOW,
            |index, node| heard.push((index, node)),
        )
        .expect("a silent port is not a failed scan");

        assert_eq!(total, 0);
        assert!(heard.is_empty(), "nothing answered, so nothing is reported");
    }

    /// `--ports` is the list of port indexes to enable and scan, in order, and a
    /// repeated index or a non-number is the mistake it looks like.
    #[test]
    fn test_the_bridge_ports_of_a_scan_are_parsed_as_indexes() {
        assert_eq!(parse_ports("0"), Ok(vec![0]));
        assert_eq!(parse_ports("0,1"), Ok(vec![0, 1]));
        assert_eq!(parse_ports(" 1 , 0 "), Ok(vec![1, 0]));
        assert!(parse_ports("0,0").expect_err("twice").contains("twice"));
        assert!(parse_ports("0,x")
            .expect_err("not a number")
            .contains("--ports"));
        assert!(parse_ports("")
            .expect_err("nothing named")
            .contains("--ports"));
    }

    /// `--bridge` is a transport of its own: naming another one beside it, or a
    /// bridge flag without the bridge, is the mistake the message names.
    #[test]
    fn test_the_bridge_scan_arguments_do_not_mix_with_other_transports() {
        let args = ScanArgs {
            bridge: Some("/dev/ttyACM0".to_string()),
            port: Some("/dev/ttyUSB0".to_string()),
            ..default_args()
        };
        let err = run(args).expect_err("two transports");
        assert!(err.to_string().contains("--bridge"), "{err}");

        let args = ScanArgs {
            bridge_uuid: Some("11aa22bb33cc".to_string()),
            ..default_args()
        };
        let err = run(args).expect_err("a uuid names a bridge");
        assert!(err.to_string().contains("--bridge-uuid"), "{err}");

        let args = ScanArgs {
            ports: Some("0,1".to_string()),
            ..default_args()
        };
        let err = run(args).expect_err("ports belong to a bridge");
        assert!(err.to_string().contains("--ports"), "{err}");

        let args = ScanArgs {
            can_bitrate: Some(500_000),
            ..default_args()
        };
        let err = run(args).expect_err("a bridge's CAN slot has a bus rate");
        assert!(err.to_string().contains("--can-bitrate"), "{err}");
        assert!(err.to_string().contains("--bridge"), "{err}");
    }
}
