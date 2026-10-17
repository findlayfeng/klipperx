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
use std::time::Duration;

use crate::core::klippy::interface::devices::canserial::{
    query_unassigned, CanbusAdminSocket, UnassignedNode as CanNode,
};
use crate::core::klippy::interface::devices::host::HostDevice;
use crate::core::klippy::interface::devices::serial::SerialDevice;
use crate::core::klippy::interface::devices::serial::{
    serial_query_unassigned, UnassignedNode as SerialNode,
};

/// How long a scan listens, in seconds: the window Klipper's tool uses.
const DEFAULT_TIMEOUT: f64 = 2.0;

/// The line speed a serial scan opens the port at when `--baud` is not given,
/// in baud.
const DEFAULT_BAUD: u32 = 250_000;

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

    /// Line speed, in baud (serial scans only)
    #[arg(long)]
    pub baud: Option<u32>,
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
            Err("give --interface (CAN), --port (serial) or --host-library".into())
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
/// bytes, so the twelve hex digits are formatted here).
fn serial_line(node: &SerialNode) -> String {
    found_line(uuid_hex(node.uuid), node.application_name())
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
    use crate::core::klippy::interface::devices::host::HOST_LIBRARY_TEST_LOCK;

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
            host_identity: None,
            timeout: DEFAULT_TIMEOUT,
            baud: None,
        }
    }

    /// The real host library the interface tests load: built from the klipper
    /// tree by `test-support`, and **not** a mux firmware (the firmware's mux
    /// device half is still to come), so it answers no scan.
    fn host_library_path() -> String {
        klipperx_test_support::klipper_host_lib_path()
            .display()
            .to_string()
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
        SerialNode { uuid, application }
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
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let mut heard = Vec::new();
        let total = host_query_unassigned(&host_library_path(), None, HOST_SCAN_WINDOW, |node| {
            heard.push(node)
        })
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
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let mut heard = Vec::new();
        let total = host_query_unassigned(
            &host_library_path(),
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
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let args = ScanArgs {
            host_library: Some(host_library_path()),
            host_identity: Some("000000000000".to_string()),
            timeout: HOST_SCAN_WINDOW.as_secs_f64(),
            ..default_args()
        };
        run(args).expect("an empty host scan is a scan, not an error");
    }
}
