//! `klipperx serial-scan` — list the boards on a serial port that have no node
//! id yet.
//!
//! CAN finds those boards with a broadcast on its own admin id, but a serial
//! port has no side channel: the discovery runs **inside** the byte stream. The
//! port is asked by writing one [`discovery_query`] — the admin listen id as a
//! unit's header, then the `KXID` frame carrying the query — and every board
//! that has not been assigned an MCU id answers on the admin reply id with its
//! UUID and the application it runs. (A board's id is normally handed out by
//! `[mcu_ids]` when the config loads; the handoff stays there — this tool only
//! *lists* the boards a `printer.cfg` still has to name, whose UUIDs go into
//! `[mcu]`'s `uuid`.)
//!
//! This is a bench tool, not part of the host: it needs no config file, and it
//! takes nothing over. The query changes no board's state — assigning a node id
//! does, and that stays where it is, in the config the host reads.
//!
//! The output is the CAN scan's, line for line (both are the shape Klipper's
//! `scripts/canbus_query.py` prints): the twelve hex digits of UUID with their
//! leading zeros, and the application's name.
//!
//! ```text
//! Found uuid=11aa22bb33cc, Application: Klipper
//! Total 1 uuids found
//! ```
//!
//! The window (`--timeout`) is a **deadline**, measured from the query, exactly
//! as the CAN scan's is: whatever has arrived when it runs out is the answer,
//! and a board that answers late does not extend it.
//!
//! [`discovery_query`]: crate::core::klippy::interface::devices::shared_serial::discovery_query

use clap::Args;
use std::time::Duration;

use crate::core::klippy::interface::devices::serial::SerialDevice;
use crate::core::klippy::interface::devices::shared_serial::{
    serial_query_unassigned, UnassignedNode,
};

/// How long a scan listens, in seconds: the window the CAN scan (and Klipper's
/// own tool) uses.
const DEFAULT_TIMEOUT: f64 = 2.0;

/// The line speed a scan opens the port at when `--baud` is not given, in baud.
const DEFAULT_BAUD: u32 = 250_000;

#[derive(Args, Debug)]
pub struct SerialScanArgs {
    /// Serial port to scan, as a device path (`/dev/ttyACM0`)
    #[arg(long)]
    pub port: String,

    /// How long to wait for answers, in seconds (defaults to the CAN scan's
    /// two)
    #[arg(long)]
    pub timeout: Option<f64>,

    /// Line speed, in baud
    #[arg(long)]
    pub baud: Option<u32>,
}

/// Entry point for the `serial-scan` subcommand.
///
/// Each board is printed as it answers, so the answers are readable while the
/// window is still open; the total is the last line.
pub fn run(args: SerialScanArgs) -> Result<(), Box<dyn std::error::Error>> {
    let timeout = args.timeout.unwrap_or(DEFAULT_TIMEOUT);
    if !timeout.is_finite() || timeout <= 0.0 {
        return Err("--timeout must be a positive number of seconds".into());
    }

    let mut device = SerialDevice::open(&args.port, args.baud.unwrap_or(DEFAULT_BAUD))?;
    let total = serial_query_unassigned(&mut device, Duration::from_secs_f64(timeout), |node| {
        println!("{}", found_line(&node))
    })?;
    println!("{}", total_line(total));
    Ok(())
}

/// One answer, as the CAN scan prints it: twelve hex digits of UUID with their
/// leading zeros, and the application's name.
fn found_line(node: &UnassignedNode) -> String {
    format!(
        "Found uuid={:012x}, Application: {}",
        uuid_number(node.uuid),
        node.application_name()
    )
}

/// The UUID as one number, summed the way Klipper's own tool sums it — the byte
/// after the answer's command is the most significant one.
fn uuid_number(uuid: [u8; 6]) -> u64 {
    uuid.iter()
        .fold(0u64, |value, byte| (value << 8) | u64::from(*byte))
}

/// The scan's closing line: how many boards answered.
fn total_line(total: usize) -> String {
    format!("Total {total} uuids found")
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// A node as a scan would collect it.
    fn node(uuid: [u8; 6], application: Option<u8>) -> UnassignedNode {
        UnassignedNode { uuid, application }
    }

    #[test]
    fn test_the_found_line_is_klipper_s_own() {
        // The same 12-digit leading-zeroed UUID and application name the CAN
        // scan prints (and Klipper's own tool prints for the bus).
        assert_eq!(
            found_line(&node([0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc], Some(0x01))),
            "Found uuid=11aa22bb33cc, Application: Klipper"
        );
        assert_eq!(
            found_line(&node([0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff], Some(0x11))),
            "Found uuid=aabbccddeeff, Application: CanBoot"
        );
        // The leading zeros of a small UUID are part of the twelve digits.
        assert_eq!(
            found_line(&node([0, 0, 0, 0, 0, 1], None)),
            "Found uuid=000000000001, Application: Klipper"
        );
    }

    #[test]
    fn test_the_total_line_counts_the_boards() {
        assert_eq!(total_line(0), "Total 0 uuids found");
        assert_eq!(total_line(3), "Total 3 uuids found");
    }

    #[test]
    fn test_a_timeout_that_is_not_a_positive_number_is_rejected() {
        // Rejected before the port is opened, so this runs without a port: a
        // zero or negative window would ask for nothing (or for a wait the
        // deadline loop cannot express).
        for bad in ["0", "-1", "nan", "inf"] {
            let args = SerialScanArgs {
                port: "/dev/not-a-port".to_string(),
                timeout: Some(bad.parse().expect("a number")),
                baud: None,
            };
            let err = run(args).expect_err("a window of nothing is not a scan");
            assert!(err.to_string().contains("--timeout"), "{err}");
        }
    }
}
