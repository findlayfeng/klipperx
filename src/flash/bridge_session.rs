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
//! 1. **declare** every port — `allocate_oids` for the object ids, then a
//!    `config_uart` + `config_bridge_port` pair per port
//!    ([`crate::core::klippy::cmd::uart`]). Both commands take effect as they
//!    arrive: the stream is opened, the port is hung on the router and numbered
//!    by arrival order. There is no `finalize_config` in this flow; these
//!    commands alone are what initializes a bridge for a flash run.
//! 2. **reset** each port's device, the way the plan says ([`PortReset`]) —
//!    written at the line for a device with no admin channel, restarted through
//!    it for one that has.
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
use crate::core::klippy::cmd::uart::{ConfigBridgePort, ConfigUart};
use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::interface::{Interface, InterfaceError};
use crate::core::klippy::mcu::{Mcu, McuError};
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortReset {
    /// The named device's own restart (`CMD_RESTART_DEVICE`); needs a device
    /// that answers the admin channel.
    RestartDevice,
    /// The bridge's line pulse on the port (`CMD_RESET_PORT`); the reset for a
    /// legacy device, for a target sitting in its bootloader, and the fallback
    /// for a port whose device is unknown.
    ResetPort,
}

/// One port of a bridge's downstream plan.
///
/// The plan's **order is the port numbering**: `config_bridge_port` makes the
/// bridge number a port by the order the command arrives, and that number is
/// also the slot `config_uart` carries and the number the host named the port's
/// tty with before the bridge came up (`third_party/klipper/src/uart.c`,
/// `mcu/bridge_ports.rs`). So the i-th entry of the slice [`open_bridge`] is
/// given is port `i` — the index every port-level command addresses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BridgePort {
    /// The line rate the bridge opens this port's stream at — the port
    /// section's `uart_baud:`, or
    /// [`DEFAULT_UART_BAUD`](crate::core::klippy::cmd::uart::DEFAULT_UART_BAUD)
    /// when it wrote none.
    pub baud: u32,
    /// Which reset this port's device gets before the tunnel opens.
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
    if ports.len() > MAX_BRIDGE_PORTS {
        return Err(BridgeError::TooManyPorts { ports: ports.len() });
    }

    // 1. The oid table, then one stream-and-port pair per port, in index order:
    //    the bridge numbers a port by the order `config_bridge_port` reaches it.
    let count = (ports.len() * 2) as u8;
    mcu.send(AllocateOids::NAME, &AllocateOids { count }.args())?;
    for (index, port) in ports.iter().enumerate() {
        let index = index as u8;
        let uart_oid = index * 2;
        let uart = ConfigUart {
            oid: uart_oid,
            baud: port.baud,
            slot: index,
        };
        mcu.send(ConfigUart::NAME, &uart.args())?;
        let bridge_port = ConfigBridgePort {
            oid: uart_oid + 1,
            uart: uart_oid,
        };
        mcu.send(ConfigBridgePort::NAME, &bridge_port.args())?;
    }
    // The declaration has to be on the wire before the first port-level command:
    // the bridge builds its port table from it, and a command that names a port
    // it does not have is dropped in silence.
    mcu.flush(DECLARE_FLUSH_TIMEOUT).await?;

    // 2. Every declared port's device, the way its plan says. After the
    //    declaration above, so each command addresses a port the bridge has.
    for (index, port) in ports.iter().enumerate() {
        let index = index as u8;
        match port.reset {
            PortReset::RestartDevice => link.send_restart_device(index)?,
            PortReset::ResetPort => link.send_reset_port(index)?,
        }
    }

    // 3. The target: its route first (legacy, so the raw uplink is served),
    //    then the tunnel that makes the port carry the katapult stream.
    link.send_set_port_node_id(target.index, target.node_id, LEGACY_PORT_FLAGS)?;
    link.send_set_port_tunnel(target.index)?;

    Ok(BridgeReady {
        ports: ports.to_vec(),
        target,
    })
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
    use crate::core::klippy::interface::devices::host::{HostDevice, HOST_LIBRARY_TEST_LOCK};
    use crate::core::klippy::interface::devices::serial::{
        admin_unit, attach_downstream, decode_header, HeaderScan, ADMIN_LISTEN, CMD_RESET_PORT,
        CMD_RESTART_DEVICE, CMD_SET_PORT_NODEID, CMD_SET_PORT_TUNNEL, HEADER_BYTES,
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

    /// The plan's steps in the order the firmware needs them: the declaration
    /// pairs in index order, then each port's reset as its plan says, then the
    /// target named **legacy** and its tunnel opened.
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
            Step::Admin(admin_payload(&[CMD_RESTART_DEVICE, 1])),
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
    #[tokio::test]
    async fn test_open_bridge_declares_resets_and_opens_the_tunnel_in_order() {
        let pty = Pty::new();
        let link = shared_serial_link(&pty);
        let mcu = bridge_mcu("bridge", &link);

        let ports = [
            BridgePort {
                baud: 250_000,
                reset: PortReset::ResetPort,
            },
            BridgePort {
                baud: 115_200,
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

        // The four admin units are the last things on the wire, so seeing them
        // all means every declaration frame is already in hand.
        let units = read_units(&pty.master, Duration::from_secs(2), |units| {
            units.iter().filter(|(arb, _)| *arb == ADMIN_LISTEN).count() >= 4
        });
        let parser = mux_parser();
        assert_eq!(
            steps(&parser, &units),
            expected_steps(TARGET_NODE_ID),
            "the declaration pairs in index order, then each reset as its plan says, \
             then the target named legacy and its tunnel opened"
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

    /// Block (bounded) until the process-wide library on `path` is unloaded,
    /// then return — and **fail here** rather than warning.
    ///
    /// Releasing [`HOST_LIBRARY_TEST_LOCK`] with the mux library still loaded is
    /// what makes the *next* host-library test meet a live image and fail with
    /// "only one host library per process", so the blame has to land on this
    /// test. The probe is [`HostDevice::load_bytes`] in another output mode: it
    /// fails while the mux library is live and succeeds (dropping the probe
    /// unloads it again) once it is gone — the same test as
    /// `bridge_attach::wait_for_mux_library_unload`, but bounded with a panic so
    /// a stuck teardown cannot smuggle a live library past the lock.
    fn wait_for_library_unload(path: &std::path::Path) {
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
    /// `multi_thread` is not incidental: [`Mcu::close`] only **aborts** the
    /// transport tasks, and on the default current-thread runtime an abort is
    /// processed by the runtime — which is idle while the test thread blocks in
    /// the unload wait below. The aborted tasks would keep their `Interface`
    /// clones (and so the library) until the runtime is dropped *after* the lock
    /// is released, leaving the mux library live for the next test. On worker
    /// threads the aborts land while the wait runs, and the library unloads.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_open_bridge_over_the_real_library() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // A bridge opens a port's tty when the host declares it, so those ttys
        // have to exist when the library comes up, each on its own pty.
        let target_port = Pty::new();
        let named_port = Pty::new();

        let library = klipperx_test_support::klipper_host_mux_lib_path();
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
                reset: PortReset::ResetPort,
            },
            BridgePort {
                baud: 250_000,
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

        // Port 1: exactly one thing on its line — the restart, which the bridge
        // forwards to the port with the unit header stripped. One byte past it
        // is asked for, so the window also shows that nothing else arrived.
        let restart = admin_unit(&[CMD_RESTART_DEVICE, 1]);
        let forwarded = &restart[HEADER_BYTES..];
        let on_the_line = read_pty(&named_port.master, forwarded.len() + 1, quiet);
        assert_eq!(
            on_the_line, forwarded,
            "RESTART_DEVICE reaches a legacy port as the bare KXID frame"
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
        // its drop is not synchronous with `drop(mcu)`: close it, let the
        // blocked device read see the shutdown, then wait the library out — all
        // of it under `_serial`, whose scope is the whole test, so the next
        // library test cannot start before this path is free.
        mcu.close();
        drop(mcu);
        drop(link);
        wait_for_library_unload(&library);
    }
}
