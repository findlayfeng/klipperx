//! Arguments for the standalone `flash` host binary, and the plan they make.
//!
//! [`plan`] is the whole argument grammar as a pure function: it turns the
//! command line into one [`Action`] (query a CAN bus, or flash one board over
//! one direct link) or into an error naming the mistake, before anything is
//! opened or sent. The same grammar `scripts/flashtool.py` speaks, in this
//! repo's spelling: a serial board is addressed by its device path, a CAN
//! board by its CANbus UUID plus the node id the session assigns it.

use std::path::{Path, PathBuf};
use std::time::Duration;

use clap::Parser;

use crate::core::klippy::interface::devices::canserial::{self, CanbusAdminSocket};

use super::session;
use super::transport;

/// The line speed a serial flash opens the port at when `--baud` is not
/// given, in baud — `flashtool.py`'s default.
const DEFAULT_BAUD: u32 = 250_000;

/// The node id a CAN flash assigns the board when `--nodeid` is not given:
/// `CANBUS_NODEID_OFFSET` in `flashtool.py`, the bottom of the range the
/// katapult admin commands accept (`nodeid >= 128`).
const DEFAULT_NODEID: u32 = 128;

/// How long `--query` listens for answers, in seconds — the same kind of
/// deadline window a `flashtool.py` query uses.
const QUERY_WINDOW: Duration = Duration::from_secs(2);

/// The command line of the `flash` binary.
#[derive(Parser, Debug)]
#[command(
    name = "flash",
    version,
    about = "Flash firmware to a board running the katapult bootloader, \
             over a direct serial tty/USB link or a SocketCAN bus",
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

    /// List the unassigned nodes on a CAN bus instead of flashing
    #[arg(short, long)]
    pub query: bool,

    /// Firmware image to flash (`out/klipper.bin`)
    pub firmware: Option<PathBuf>,
}

/// What a command line asks the tool to do: the two entry points the
/// [`crate::flash`] session has, over the transport the arguments named.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Action {
    /// Print every board on `interface` that has no node id yet.
    Query { interface: String, window: Duration },
    /// Flash one board over one direct link.
    Flash { target: Target, firmware: PathBuf },
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

    match (&args.device, &args.interface) {
        (Some(_), Some(_)) => {
            Err("give either --device (serial) or --interface (CAN), not both".into())
        }
        (None, None) => Err("give --device (serial) or --interface (CAN)".into()),
        (Some(device), None) => plan_tty(args, device),
        (None, Some(interface)) => plan_can(args, interface),
    }
}

/// The `--query` line: a bus to list and nothing that flashes.
fn plan_query(args: &FlashArgs) -> Result<Action, String> {
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
    let bytes = text.as_bytes();
    if bytes.len() != 12 || !bytes.iter().all(u8::is_ascii_hexdigit) {
        return Err(format!("--uuid expects twelve hex digits, got '{text}'"));
    }
    let mut uuid = [0u8; 6];
    for (i, pair) in bytes.chunks(2).enumerate() {
        let text = std::str::from_utf8(pair).expect("ascii hex digits");
        uuid[i] = u8::from_str_radix(text, 16).expect("ascii hex digits");
    }
    Ok(uuid)
}

/// Run the action a command line asks for: the binary's whole job after
/// parsing. Errors are the tool's exit path — the binary prints them and
/// exits non-zero.
pub fn run(args: FlashArgs) -> Result<(), Box<dyn std::error::Error>> {
    match plan(&args)? {
        Action::Query { interface, window } => run_query(&interface, window),
        Action::Flash { target, firmware } => run_flash(target, &firmware),
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
}
