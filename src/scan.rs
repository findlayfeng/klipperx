//! `klipperx scan` — list the boards that have no node id yet, on a CAN bus or
//! a serial port.
//!
//! One command covers both transports because the question is the same one:
//! which boards has a `printer.cfg` still to name? The answer is always a UUID
//! (what a `[mcu]` section then writes in `uuid:`) and an application name. The
//! transport is chosen by the arguments, not by a second command, and it must
//! be named: the subcommand has no default bus, so a bare `klipperx scan`
//! prints its help rather than guessing at one.
//!
//! - `--interface <name>` — that CAN bus, as Klipper's
//!   `scripts/canbus_query.py` does: one broadcast of `CMD_QUERY_UNASSIGNED` on
//!   the admin id (`0x3f0`), every board without a node id answering on the
//!   next id (`0x3f1`).
//! - `--port <tty>` — that serial port, where discovery runs **inside** the
//!   byte stream instead: one [`discovery_query`] unit, unseen by the data
//!   plane, and the same answers on the admin reply id.
//!
//! Both scan the same way underneath: a single broadcast, then a **deadline**
//! window (`--timeout`, two seconds by default) measured from the query —
//! whatever has arrived when it runs out is the answer, and a board that
//! answers late does not extend it.
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
//! [`discovery_query`]: crate::core::klippy::interface::devices::shared_serial::discovery_query

use clap::Args;
use std::time::Duration;

use crate::core::klippy::interface::devices::canserial::{
    query_unassigned, CanbusAdminSocket, UnassignedNode as CanNode,
};
use crate::core::klippy::interface::devices::serial::SerialDevice;
use crate::core::klippy::interface::devices::shared_serial::{
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

    match (&args.port, &args.interface) {
        // Two transports named at once is a mistake, not a preference: the
        // serial one is not a fallback for a CAN bus, it is another bus.
        (Some(_), Some(_)) => {
            Err("give either --port (serial) or --interface (CAN), not both".into())
        }
        (Some(port), None) => {
            let baud = args.baud.unwrap_or(DEFAULT_BAUD);
            let mut device = SerialDevice::open(port, baud)?;
            let total = serial_query_unassigned(&mut device, window, |node| {
                println!("{}", serial_line(&node))
            })?;
            println!("{}", total_line(total));
            Ok(())
        }
        (None, Some(interface)) => {
            if args.baud.is_some() {
                return Err("--baud needs --port: a CAN scan has no line speed".into());
            }
            let mut bus = CanbusAdminSocket::open(interface)?;
            let scan = query_unassigned(&mut bus, window, |node| println!("{}", can_line(&node)))?;
            println!("{}", total_line(scan.total()));
            Ok(())
        }
        // No transport named: reachable through `ScanArgs` but not from the
        // command line, where `arg_required_else_help` prints the help instead
        // (see the `scan` variant in `main.rs`).
        (None, None) => Err("give --interface (CAN) or --port (serial)".into()),
    }
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
                interface: None,
                port: Some("/dev/not-a-port".to_string()),
                timeout: bad.parse().expect("a number"),
                baud: None,
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
        let args = ScanArgs {
            interface: None,
            port: None,
            timeout: DEFAULT_TIMEOUT,
            baud: None,
        };
        let err = run(args).expect_err("a scan has to be told what to scan");
        assert!(err.to_string().contains("--interface"), "{err}");
    }

    #[test]
    fn test_a_port_and_an_interface_together_are_rejected() {
        // Both named is a mistake the arguments can name before any I/O: the
        // serial port is not a fallback for the CAN bus, it is another bus.
        let args = ScanArgs {
            interface: Some("can0".to_string()),
            port: Some("/dev/not-a-port".to_string()),
            timeout: DEFAULT_TIMEOUT,
            baud: None,
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
            port: None,
            timeout: DEFAULT_TIMEOUT,
            baud: Some(115_200),
        };
        let err = run(args).expect_err("a CAN scan has no line speed");
        assert!(err.to_string().contains("--baud"), "{err}");
    }
}
