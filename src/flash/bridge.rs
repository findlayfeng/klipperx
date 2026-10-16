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
//! Every step is a command on one link, and every command is one the repo
//! already spells for each wire: the bridge's own frame channel is [`Device`],
//! and the port-level commands are the ones the concrete handles carry
//! ([`BridgeLink`]). Nothing here probes what is behind a port — a bridge link
//! carries no such probe — so which reset a port gets, and which port the target
//! is, are the caller's to say.

use crate::core::klippy::cmd::allocate_oids::AllocateOids;
use crate::core::klippy::cmd::uart::{ConfigBridgePort, ConfigUart};
use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::frame::{Frame, MESSAGE_SEQ_MASK};
use crate::core::klippy::interface::devices::host::HostDevice;
use crate::core::klippy::interface::devices::serial::SharedSerialHandle;
use crate::core::klippy::interface::{Device, InterfaceError};
use crate::core::klippy::msg::parser::Parser;
use crate::core::klippy::msg::proto::ArgValue;
use crate::core::klippy::msg::MsgError;
use std::fmt;

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

/// A link a bridge session can drive.
///
/// Two channels are needed, and the handles the config layer builds already
/// carry both: the bridge's own frame channel, for the initialization commands
/// (that is [`Device::send`]), and its **admin** channel, for the port-level
/// ones. Each handle spells the admin unit for its own wire — a muxed host
/// library, a shared serial port and a CAN node are three different units — so
/// this trait names the capability rather than re-encoding any of it.
///
/// Implemented for the two links a klipper config builds for a bridge:
/// [`HostDevice`] (`[mcu] host_library:`) and [`SharedSerialHandle`] (a
/// `serial:` path more than one `[mcu]` section shares). A CAN bridge has the
/// same four commands on its own device and would be added the same way;
/// nothing in this module depends on which of them it was given.
pub trait BridgeLink: Device {
    /// Tell the bridge which id the device behind port `index` answers to
    /// ([`HostDevice::send_set_port_node_id`], `flags` and all).
    fn set_port_node_id(&self, index: u8, node_id: u8, flags: u8) -> Result<(), InterfaceError>;
    /// Ask the device behind port `index` to restart its firmware
    /// ([`HostDevice::send_restart_device`]).
    fn restart_device(&self, index: u8) -> Result<(), InterfaceError>;
    /// Ask the bridge to pulse port `index`'s lines
    /// ([`HostDevice::send_reset_port`]).
    fn reset_port(&self, index: u8) -> Result<(), InterfaceError>;
    /// Toggle port `index`'s flashing tunnel
    /// ([`HostDevice::send_set_port_tunnel`]).
    fn set_port_tunnel(&self, index: u8) -> Result<(), InterfaceError>;
}

impl BridgeLink for HostDevice {
    fn set_port_node_id(&self, index: u8, node_id: u8, flags: u8) -> Result<(), InterfaceError> {
        self.send_set_port_node_id(index, node_id, flags)
    }

    fn restart_device(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_restart_device(index)
    }

    fn reset_port(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_reset_port(index)
    }

    fn set_port_tunnel(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_set_port_tunnel(index)
    }
}

impl BridgeLink for SharedSerialHandle {
    fn set_port_node_id(&self, index: u8, node_id: u8, flags: u8) -> Result<(), InterfaceError> {
        self.send_set_port_node_id(index, node_id, flags)
    }

    fn restart_device(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_restart_device(index)
    }

    fn reset_port(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_reset_port(index)
    }

    fn set_port_tunnel(&self, index: u8) -> Result<(), InterfaceError> {
        self.send_set_port_tunnel(index)
    }
}

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
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BridgeReady {
    /// The plan as it was carried out: every port declared, in index order.
    pub ports: Vec<BridgePort>,
    /// The target as it was named.
    pub target: BridgeTarget,
    /// The sequence byte the bridge's own frame channel expects next.
    ///
    /// The declaration commands consume one sequence byte each
    /// (`next_sequence` in `third_party/klipper/src/command.c`); the port-level
    /// commands travel the admin channel and consume none. A caller that sends
    /// anything else on that channel starts here.
    pub next_sequence: u8,
}

/// Everything that can stop a bridge session.
#[derive(Debug)]
pub enum BridgeError {
    /// The link refused a write — a closed handle, a failed device write.
    Link(InterfaceError),
    /// The dictionary could not spell a command: it carries no such name, or
    /// the arguments do not match what the firmware declares. The parser is the
    /// caller's, so this is a dictionary mismatch rather than a wire failure.
    Command(MsgError),
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
            BridgeError::Command(e) => write!(f, "bridge command: {e}"),
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
            BridgeError::Command(e) => Some(e),
            BridgeError::TooManyPorts { .. } => None,
        }
    }
}

impl From<InterfaceError> for BridgeError {
    fn from(e: InterfaceError) -> Self {
        BridgeError::Link(e)
    }
}

impl From<MsgError> for BridgeError {
    fn from(e: MsgError) -> Self {
        BridgeError::Command(e)
    }
}

/// Declare `ports` on the bridge, reset their devices, and open the tunnel on
/// `target` — the flashing prerequisites of a bridge-port flash, in one call.
///
/// `parser` is the bridge's data dictionary (an identify's, or the built one):
/// every command is encoded from it, so a dictionary that does not carry one is
/// an error rather than bytes that mean something else.
///
/// The three steps and their order are in the module docs. Nothing is sent
/// until the plan has been checked, and everything is sent in the order the
/// firmware needs, so a session either runs to the end or fails with the bridge
/// left as it was.
///
/// # Errors
/// [`BridgeError::TooManyPorts`] when the plan is wider than the oid byte can
/// number, [`BridgeError::Command`] when the dictionary cannot spell a command,
/// and [`BridgeError::Link`] when the link refuses a write.
pub fn open_bridge<L: BridgeLink>(
    link: &L,
    parser: &Parser,
    ports: &[BridgePort],
    target: BridgeTarget,
) -> Result<BridgeReady, BridgeError> {
    if ports.len() > MAX_BRIDGE_PORTS {
        return Err(BridgeError::TooManyPorts { ports: ports.len() });
    }

    // 1. The oid table, then one stream-and-port pair per port, in index order:
    //    the bridge numbers a port by the order `config_bridge_port` reaches it.
    let count = (ports.len() * 2) as u8;
    let mut sequence = 0u8;
    let allocate = AllocateOids { count };
    send_command(
        link,
        parser,
        &mut sequence,
        AllocateOids::NAME,
        &allocate.args(),
    )?;
    for (index, port) in ports.iter().enumerate() {
        let index = index as u8;
        let uart_oid = index * 2;
        let uart = ConfigUart {
            oid: uart_oid,
            baud: port.baud,
            slot: index,
        };
        send_command(link, parser, &mut sequence, ConfigUart::NAME, &uart.args())?;
        let bridge_port = ConfigBridgePort {
            oid: uart_oid + 1,
            uart: uart_oid,
        };
        send_command(
            link,
            parser,
            &mut sequence,
            ConfigBridgePort::NAME,
            &bridge_port.args(),
        )?;
    }

    // 2. Every declared port's device, the way its plan says. After the
    //    declaration above, so each command addresses a port the bridge has.
    for (index, port) in ports.iter().enumerate() {
        let index = index as u8;
        match port.reset {
            PortReset::RestartDevice => link.restart_device(index)?,
            PortReset::ResetPort => link.reset_port(index)?,
        }
    }

    // 3. The target: its route first (legacy, so the raw uplink is served),
    //    then the tunnel that makes the port carry the katapult stream.
    link.set_port_node_id(target.index, target.node_id, LEGACY_PORT_FLAGS)?;
    link.set_port_tunnel(target.index)?;

    Ok(BridgeReady {
        ports: ports.to_vec(),
        target,
        next_sequence: sequence,
    })
}

/// Encode one command from the dictionary and write it on the bridge's own
/// frame channel, taking the next sequence byte.
///
/// The sequence byte is the firmware's own check (`next_sequence` in
/// `third_party/klipper/src/command.c`): it starts at `MESSAGE_DEST`, so a
/// session's first frame carries sequence `0`, and the firmware discards
/// anything it is not expecting until it is retransmitted. The count therefore
/// lives with the session that sends the frames, and the caller gets where it
/// ended ([`BridgeReady::next_sequence`]).
fn send_command<L: BridgeLink>(
    link: &L,
    parser: &Parser,
    sequence: &mut u8,
    name: &str,
    args: &[ArgValue],
) -> Result<(), BridgeError> {
    let payload = parser.encode(name, args)?;
    link.send(&Frame::new(*sequence, payload.payload().to_vec()))?;
    // The firmware's own sequence is four bits wide (it wraps at
    // `MESSAGE_SEQ_MASK` and re-adds `MESSAGE_DEST`), so the count the caller
    // sees wraps with it.
    *sequence = (*sequence + 1) & MESSAGE_SEQ_MASK;
    Ok(())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::frame::{
        MESSAGE_DEST, MESSAGE_HEADER_SIZE, MESSAGE_POS_LEN, MESSAGE_POS_SEQ, MESSAGE_TRAILER_SIZE,
    };
    use crate::core::klippy::interface::devices::host::HOST_LIBRARY_TEST_LOCK;
    use crate::core::klippy::interface::devices::serial::{
        admin_unit, CMD_RESTART_DEVICE, HEADER_BYTES,
    };
    use crate::core::klippy::interface::pty::Pty;
    use crate::core::klippy::interface::HostMux;
    use crate::core::klippy::mcu::Dictionary;
    use crate::core::klippy::msg::proto::Payload;
    use std::io::Read;
    use std::os::fd::AsRawFd;
    use std::sync::Mutex;
    use std::time::{Duration, Instant};

    /// The id the target device is named with behind its port.
    const TARGET_NODE_ID: u8 = 7;

    /// The mux build's data dictionary, in a parser: the same dictionary an
    /// identify would fetch, read from the file `make` wrote instead.
    fn mux_parser() -> Parser {
        let path = klipperx_test_support::klipper_host_mux_dictionary_path();
        let raw =
            std::fs::read(&path).unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
        let dictionary = Dictionary::from_json(
            serde_json::from_slice(&raw).expect("the dictionary file is JSON"),
        )
        .expect("the mux build's data dictionary");
        let mut parser = Parser::new();
        dictionary
            .install(&mut parser)
            .expect("install the mux build's dictionary");
        parser
    }

    /// One call a session made on a link, in order.
    #[derive(Debug, Clone, PartialEq, Eq)]
    enum Call {
        /// A frame on the bridge's own channel, wire bytes and all.
        Frame(Vec<u8>),
        SetPortNodeId {
            index: u8,
            node_id: u8,
            flags: u8,
        },
        RestartDevice(u8),
        ResetPort(u8),
        SetPortTunnel(u8),
    }

    /// A bridge link that answers nothing and records every call: the shape a
    /// session's own commands, arguments and order are pinned on, with no
    /// firmware and no wire.
    struct RecordingLink {
        calls: Mutex<Vec<Call>>,
    }

    impl RecordingLink {
        fn new() -> Self {
            Self {
                calls: Mutex::new(Vec::new()),
            }
        }

        fn calls(&self) -> Vec<Call> {
            self.calls.lock().unwrap().clone()
        }

        fn record(&self, call: Call) {
            self.calls.lock().unwrap().push(call);
        }
    }

    impl Device for RecordingLink {
        fn send(&self, frame: &Frame) -> Result<(), InterfaceError> {
            self.record(Call::Frame(frame.raw_bytes()));
            Ok(())
        }

        fn receive(&self) -> Option<Frame> {
            None
        }

        fn shutdown(&self) {}
    }

    impl BridgeLink for RecordingLink {
        fn set_port_node_id(
            &self,
            index: u8,
            node_id: u8,
            flags: u8,
        ) -> Result<(), InterfaceError> {
            self.record(Call::SetPortNodeId {
                index,
                node_id,
                flags,
            });
            Ok(())
        }

        fn restart_device(&self, index: u8) -> Result<(), InterfaceError> {
            self.record(Call::RestartDevice(index));
            Ok(())
        }

        fn reset_port(&self, index: u8) -> Result<(), InterfaceError> {
            self.record(Call::ResetPort(index));
            Ok(())
        }

        fn set_port_tunnel(&self, index: u8) -> Result<(), InterfaceError> {
            self.record(Call::SetPortTunnel(index));
            Ok(())
        }
    }

    /// What one recorded frame says: its sequence byte, and the command the
    /// dictionary decodes its payload back into. Asserting on the decoded view
    /// keeps the test about *what* was said rather than a byte layout the
    /// dictionary owns.
    fn frame_command(parser: &Parser, frame: &[u8]) -> (u8, String, Vec<ArgValue>) {
        assert_eq!(
            frame[MESSAGE_POS_LEN] as usize,
            frame.len(),
            "a frame's length byte is the length of the whole frame"
        );
        assert_eq!(
            frame[MESSAGE_POS_SEQ] & !MESSAGE_SEQ_MASK,
            MESSAGE_DEST,
            "a command frame's sequence byte carries the destination flag"
        );
        let payload = Payload::from_raw(
            frame[MESSAGE_HEADER_SIZE..frame.len() - MESSAGE_TRAILER_SIZE].to_vec(),
        );
        let decoded = parser.decode(payload).expect("the payload decodes");
        assert_eq!(decoded.len(), 1, "one command per frame");
        let (msg, args) = decoded.into_iter().next().expect("one command per frame");
        (
            frame[MESSAGE_POS_SEQ] & MESSAGE_SEQ_MASK,
            msg.name.clone(),
            args,
        )
    }

    /// The three steps, in the order the firmware needs them, over a link that
    /// only records: the declaration pairs in index order, then each port's
    /// reset as its plan says, then the target named **legacy** and its tunnel
    /// opened.
    #[test]
    fn test_open_bridge_declares_resets_and_opens_the_tunnel_in_order() {
        let parser = mux_parser();
        let link = RecordingLink::new();
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
        let ready = open_bridge(&link, &parser, &ports, target).expect("the session runs");

        assert_eq!(
            ready.ports,
            ports.to_vec(),
            "the plan as it was carried out"
        );
        assert_eq!(ready.target, target);
        assert_eq!(
            ready.next_sequence, 5,
            "allocate_oids plus two commands per port, one sequence byte each"
        );

        let calls = link.calls();
        assert_eq!(
            calls.len(),
            9,
            "five frames, two resets, the naming, the tunnel"
        );

        // The declaration: the oid table, then a stream-and-port pair per port,
        // in index order — the bridge numbers a port by the order
        // `config_bridge_port` reaches it, and the port's oid pair is one
        // number in `config_uart`'s slot.
        let wanted = [
            ("allocate_oids", vec![ArgValue::UInt8(4)]),
            (
                "config_uart",
                vec![
                    ArgValue::UInt8(0),
                    ArgValue::UInt32(250_000),
                    ArgValue::UInt8(0),
                ],
            ),
            (
                "config_bridge_port",
                vec![ArgValue::UInt8(1), ArgValue::UInt8(0)],
            ),
            (
                "config_uart",
                vec![
                    ArgValue::UInt8(2),
                    ArgValue::UInt32(115_200),
                    ArgValue::UInt8(1),
                ],
            ),
            (
                "config_bridge_port",
                vec![ArgValue::UInt8(3), ArgValue::UInt8(2)],
            ),
        ];
        for (sequence, call) in calls.iter().take(wanted.len()).enumerate() {
            let Call::Frame(frame) = call else {
                panic!("the declaration step sends frames only, got {call:?}");
            };
            let (seq, name, args) = frame_command(&parser, frame);
            assert_eq!(
                seq, sequence as u8,
                "the firmware counts a session's frames from sequence 0"
            );
            let (want_name, want_args) = &wanted[sequence];
            assert_eq!(&name, want_name, "frame {sequence}");
            assert_eq!(&args, want_args, "frame {sequence}");
        }

        assert_eq!(
            &calls[wanted.len()..],
            &[
                Call::ResetPort(0),
                Call::RestartDevice(1),
                Call::SetPortNodeId {
                    index: 0,
                    node_id: TARGET_NODE_ID,
                    flags: 0,
                },
                Call::SetPortTunnel(0),
            ],
            "each port's reset follows its plan, then the target is named with no port \
             flags (a units route would drop the katapult stream) and its tunnel opened"
        );
    }

    /// The sequence count is the firmware's four-bit one, and a plan the oid
    /// byte cannot number is refused before anything is sent.
    #[test]
    fn test_an_impossible_plan_is_refused_and_the_sequence_wraps_with_the_firmware() {
        let parser = mux_parser();
        let link = RecordingLink::new();
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
        let err = open_bridge(&link, &parser, &one_too_many, target)
            .expect_err("one port over the oid byte is refused");
        match err {
            BridgeError::TooManyPorts { ports } => assert_eq!(ports, MAX_BRIDGE_PORTS + 1),
            other => panic!("expected TooManyPorts, got {other:?}"),
        }
        assert!(
            link.calls().is_empty(),
            "a plan that cannot run sends nothing"
        );

        // The widest plan that does fit: two oids per port is the whole byte,
        // and its 255 frames wrap the four-bit sequence count.
        let widest = vec![
            BridgePort {
                baud: 250_000,
                reset: PortReset::ResetPort,
            };
            MAX_BRIDGE_PORTS
        ];
        let ready = open_bridge(&link, &parser, &widest, target)
            .expect("the widest plan the oid byte can number runs");
        assert_eq!(
            ready.next_sequence,
            (2 * MAX_BRIDGE_PORTS + 1) as u8 & MESSAGE_SEQ_MASK
        );
    }

    /// Both links a config builds for a bridge meet the session's bound: the
    /// host-library reference and the shared serial port's handle. A CAN
    /// bridge's own device carries the same four commands and is added the same
    /// way; it is not one this module has a fixture for.
    #[test]
    fn test_both_bridge_links_meet_the_session_bound() {
        fn assert_bridge_link<L: BridgeLink>() {}
        assert_bridge_link::<HostDevice>();
        assert_bridge_link::<SharedSerialHandle>();
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
    /// Every command's effect is pinned where the wire is. The two declaration
    /// commands are the library's (they are what opens a port's stream, and
    /// nothing of them reaches the line), the target's naming and its tunnel are
    /// the bridge's own (consumed, never written to a port, no receipt), and
    /// `RESTART_DEVICE` for port 1 is forwarded to it header-stripped — the bare
    /// `KXID` frame a legacy port's line carries.
    #[test]
    fn test_open_bridge_over_the_real_library() {
        let _serial = HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());

        // A bridge opens a port's tty when the host declares it, so those ttys
        // are named when the library comes up, each on its own pty.
        let target_port = Pty::new();
        let named_port = Pty::new();
        let identity = [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc];
        let bridge = HostDevice::load_mux(
            klipperx_test_support::klipper_host_mux_lib_path(),
            HostMux {
                identity,
                named: identity,
            },
            1,
            &[
                (0, target_port.path.to_string_lossy().into_owned()),
                (1, named_port.path.to_string_lossy().into_owned()),
            ],
        )
        .expect("the mux host library opens with both port ttys");

        let parser = mux_parser();
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
        let ready = open_bridge(&bridge, &parser, &ports, target)
            .expect("the session runs against a real bridge");
        assert_eq!(ready.target, target);
        assert_eq!(ready.next_sequence, 5);

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

        // Dropping the bridge is the whole teardown — the library's drop joins
        // its threads and unmaps before it returns, and the lock above is held
        // until this test ends, so the path is free for the next test.
        drop(bridge);
    }
}
