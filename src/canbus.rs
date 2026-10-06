//! `klipperx canbus-scan` — list the boards on a CAN bus that have no node id yet.
//!
//! Klipper's own tool for this is `scripts/canbus_query.py`, and what it does is
//! one broadcast: the host sends `CMD_QUERY_UNASSIGNED` on the admin id (`0x3f0`)
//! and every board that has not been assigned a `canbus_nodeid` answers on the
//! next id (`0x3f1`) with its UUID and the application it runs. Boards that
//! already have a node id stay quiet, so a scan lists exactly the boards a
//! `printer.cfg` still has to name — the UUIDs that go into `[mcu]`'s
//! `canbus_uuid`.
//!
//! This is a bench tool, not part of the host: it needs no config file, and it
//! takes nothing over. The query is a broadcast that changes no board's state —
//! assigning a node id does, and that stays where it is, in the config the host
//! reads.
//!
//! The output is upstream's, line for line:
//!
//! ```text
//! Found canbus_uuid=11aa22bb33cc, Application: Klipper
//! Total 1 uuids found
//! ```
//!
//! The window (`--timeout`) is a **deadline**, measured from the query, exactly
//! as upstream's two seconds are: whatever has arrived when it runs out is the
//! answer, and a board that answers late does not extend it.

use clap::Args;
use std::time::Duration;

use crate::core::klippy::interface::devices::canserial::{
    query_unassigned, CanbusAdminSocket, UnassignedNode,
};

/// The interface a scan with no `--interface` reads: Klipper's own default for
/// `canbus_interface` (`klippy/mcu.py`).
const DEFAULT_INTERFACE: &str = "can0";

/// How long a scan listens, in seconds: the window Klipper's tool uses.
const DEFAULT_TIMEOUT: f64 = 2.0;

#[derive(Args, Debug)]
pub struct CanbusScanArgs {
    /// CAN interface to scan, as the network interface is named (`can0`)
    #[arg(long, default_value = DEFAULT_INTERFACE)]
    pub interface: String,

    /// How long to wait for answers, in seconds
    #[arg(long, default_value_t = DEFAULT_TIMEOUT)]
    pub timeout: f64,
}

/// Entry point for the `canbus-scan` subcommand.
///
/// Each board is printed as it answers, so the answers are readable while the
/// window is still open; the total is the last line.
pub fn run(args: CanbusScanArgs) -> Result<(), Box<dyn std::error::Error>> {
    if !args.timeout.is_finite() || args.timeout <= 0.0 {
        return Err("--timeout must be a positive number of seconds".into());
    }

    let mut bus = CanbusAdminSocket::open(&args.interface)?;
    let scan = query_unassigned(&mut bus, Duration::from_secs_f64(args.timeout), |node| {
        println!("{}", found_line(&node));
    })?;
    println!("{}", total_line(scan.total()));
    Ok(())
}

/// One answer, as Klipper's own tool prints it: twelve hex digits of UUID with
/// their leading zeros, and the application's name.
fn found_line(node: &UnassignedNode) -> String {
    format!(
        "Found canbus_uuid={}, Application: {}",
        node.uuid_hex(),
        node.application_name()
    )
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

    /// The board with `uuid`, as an answer frame from the bus.
    fn node(uuid: [u8; 6], application: Option<u8>) -> UnassignedNode {
        let mut data = vec![0x20];
        data.extend_from_slice(&uuid);
        data.extend(application);
        let frame = CanFrame::new(query_unassigned_frame().id() + 1, &data).expect("an answer");
        UnassignedNode::from_reply(&frame).expect("this is an answer")
    }

    #[test]
    fn test_the_found_line_is_klipper_s_own() {
        // `scripts/canbus_query.py`: "Found canbus_uuid=%012x, Application: %s".
        assert_eq!(
            found_line(&node([0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc], Some(0x01))),
            "Found canbus_uuid=11aa22bb33cc, Application: Klipper"
        );
        assert_eq!(
            found_line(&node([0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff], Some(0x11))),
            "Found canbus_uuid=aabbccddeeff, Application: CanBoot"
        );
        // The leading zeros of a small UUID are part of the twelve digits.
        assert_eq!(
            found_line(&node([0, 0, 0, 0, 0, 1], None)),
            "Found canbus_uuid=000000000001, Application: Klipper"
        );
    }

    #[test]
    fn test_the_total_line_counts_the_boards() {
        assert_eq!(total_line(0), "Total 0 uuids found");
        assert_eq!(total_line(3), "Total 3 uuids found");
    }

    #[test]
    fn test_a_timeout_that_is_not_a_positive_number_is_rejected() {
        // Rejected before the socket is opened, so this runs without a CAN
        // interface: a zero or negative window would ask for nothing (or for a
        // wait `poll` cannot express).
        for bad in ["0", "-1", "nan", "inf"] {
            let args = CanbusScanArgs {
                interface: "can0".to_string(),
                timeout: bad.parse().expect("a number"),
            };
            let err = run(args).expect_err("a window of nothing is not a scan");
            assert!(err.to_string().contains("--timeout"), "{err}");
        }
    }
}
