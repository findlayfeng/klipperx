use super::wrapper::ConfigWrapper;
use crate::core::klippy::error::ConfigError;
use crate::core::klippy::interface::devices::serial::{self, ASSIGN_ATTACH_TIMEOUT};
use crate::core::klippy::interface::usb::UsbPowerMethod;
#[cfg(test)]
use crate::core::klippy::interface::SimulatorDevice;
use crate::core::klippy::interface::{CanWire, HostMux, Interface, SerialDevice};
use crate::core::klippy::mcu::port_grouping::{self, PortKind};
use crate::core::klippy::mcu::{host_libraries, serial_ports, McuRestartMethod};
use std::path::Path;
use tracing::warn;

/// Which port-level reset a device behind a bridge port is given.
///
/// Resolved at config time into [`McuConfig::bridge_restart`]. A bridge-port
/// device has no transport of its own — it is spoken to on the bridge's link —
/// so its reset is a port-level command the bridge receives over its admin
/// channel (`mcu/restart.rs`, `interface/devices::serial::send_restart_device`,
/// `send_reset_port`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BridgeRestart {
    /// The device restarts its own firmware: the host sends `RESTART_DEVICE` and
    /// the bridge forwards it to the port. Only a **named** device (a `uuid:` on
    /// the child, `restart_method: admin`) has the admin channel this needs.
    Firmware,
    /// The bridge pulses the port's lines (`RESET_PORT`): the only reset a
    /// **legacy** (uuid-less) device can get, from the **port section's**
    /// `uart_restart_method` (`mcu/bridge_ports.rs`).
    Port,
}

/// MCU (Microcontroller Unit) configuration parsed from config file.
///
/// Parsing does not touch a device: [`McuConfig::open`] is what opens the
/// transport. The split lets a firmware restart reset the board on its **closed**
/// port, between the two (`mcu/object.rs`, `mcu/restart.rs`).
#[derive(Debug, Clone)]
pub struct McuConfig {
    /// MCU name (from ConfigSection's sub field)
    pub name: String,
    /// MCU restart method.
    ///
    /// Resolved to the effective method: `command` for a non-serial MCU, the
    /// config's own value (default `arduino`) for a serial one, and `admin` for
    /// a serial one or a named bridge-port device ([`McuRestartMethod`]) — on a
    /// serial section `admin` is the bridge restarting **itself** over its mux
    /// admin channel (`mcu/restart.rs`). The reset itself is `mcu/restart.rs`'s,
    /// dispatched on this.
    pub restart_method: McuRestartMethod,
    /// Which port-level reset this device gets, when it hangs behind a bridge
    /// port.
    ///
    /// `None` for every transport that is not a bridge port; for a legacy
    /// (uuid-less) device whose port declares no `uart_restart_method`; and for
    /// a **named** (uuid-bearing) device that writes no `restart_method`, or one
    /// other than `admin` — a named device resets itself only under `admin`, and
    /// has no port-level reset to fall back to. The two bridge-port shapes learn
    /// it differently: a **named** device writes `restart_method: admin` itself
    /// ([`BridgeRestart::Firmware`]), while a **legacy** one has no admin channel
    /// of its own and takes the **port section's** `uart_restart_method`
    /// ([`BridgeRestart::Port`]). The command travels the bridge's link, not a
    /// transport of this device's own.
    pub bridge_restart: Option<BridgeRestart>,
    /// What to open, described but not opened.
    pub transport: Transport,
    /// The id this MCU was assigned, unique across every `[mcu]` whatever its
    /// transport (the mcu-layer allocator, `mcu/ids.rs`).
    ///
    /// For a CAN MCU it is the node id the link is addressed with
    /// (`0x100 + 2 * id`); for other transports nothing consumes it yet. An MCU
    /// must hold one before its transport can be opened — parsing only knows an
    /// explicit `canbus_nodeid`, the rest is the allocator's, so this starts
    /// `None` and [`McuObject::connect`](crate::core::klippy::mcu::object::McuObject::connect)
    /// fills it from the id the object registered with at load.
    pub id: Option<u32>,
    /// The baud this MCU shares its serial `serial:` path at, or `None` for an
    /// exclusive port.
    ///
    /// Set at load from the grouping pass (`mcu/serial_ports.rs`): when two or
    /// more `[mcu]` sections name the same `serial:` path, the path is shared
    /// and [`Transport::open`] takes the shared branch. `None` — the value
    /// parsing leaves it at — means the path belongs to this MCU alone and the
    /// transport opens it exactly as before.
    pub shared_baud: Option<u32>,
    /// What a muxed `host_library:` path attaches with, or `None` for a path the
    /// config does not call muxed.
    ///
    /// Set at load from the path grouping (`mcu/host_libraries.rs`): when the
    /// path is muxed — the section writes a `uuid`, or two or more sections name
    /// it — [`Transport::open`] opens the library through
    /// [`Interface::host_mux`] instead of the bare-frame [`Interface::host`].
    /// `None` — the value parsing leaves it at — is today's single-MCU mode,
    /// bare frames with no identity.
    pub host_mux: Option<HostMux>,
    /// The tty the host must name for each of this MCU's bridge ports before the
    /// firmware comes up, as `(slot, path)`: a port section's `uart_path:`, keyed
    /// by the port index `config_uart` carries.
    ///
    /// Set at load from the bridge's port sections (`mcu/bridge_ports.rs`) and
    /// read back by `connect`, exactly as [`McuConfig::shared_baud`] is. Empty
    /// for every MCU that is not a host-library bridge, and for a bridge whose
    /// ports name no path — those ports open the path the library was built
    /// with.
    pub host_port_paths: Vec<(u8, String)>,
    /// Which mechanism `rpi_usb` uses to switch the port's power.
    ///
    /// Only meaningful with `restart_method: rpi_usb`; see
    /// [`UsbPowerMethod`] and `interface/usb.rs`.
    pub usb_power: UsbPowerMethod,
}

/// The transport a section asks for, described before anything is opened.
///
/// Keeping the description separate from the opened [`Interface`] is what lets
/// the host reset a firmware on its **closed** port — a reset needs the port
/// path, and the device must not be open — between parsing the section and
/// opening the device (see [`McuConfig::open`]).
///
/// `FrameMock` is **not** constructed from config — tests build it directly in
/// code and pass it through `Interface::new()`.  `SimulatorDevice` is
/// constructed from `test: dict=<path>` in the `upstream` harness.
#[derive(Debug, Clone, PartialEq)]
pub enum Transport {
    /// A tty at a line speed.
    ///
    /// `uuid` is the identity the board answers to: on a **shared** port the
    /// connect-time handshake names it before re-confirming the assignment
    /// (see [`Transport::open`]); an exclusive port ignores it.
    Serial {
        path: String,
        baud: u32,
        uuid: Option<[u8; 6]>,
    },
    /// A Klipper can-serial link (a CAN interface and a uuid) on the
    /// **classical** CAN wire — `canbus_fd` unset or `false`, today's link.
    ///
    /// The id that addresses the node is **not** part of the transport: it is
    /// the MCU's id at the `[mcu]` level (`McuConfig::id`), allocated to every
    /// MCU regardless of transport (`mcu/ids.rs`); the CAN arm here just
    /// consumes it when the link is opened.
    Can { interface: String, uuid: [u8; 6] },
    /// The same link on a **CAN FD** wire (`canbus_fd: true`).
    ///
    /// One variant rather than a flag on [`Transport::Can`]: the wire is a
    /// property of the other end's build, not a preference this side may switch
    /// alone — a socket that has not asked the kernel for CAN FD frames hears
    /// nothing on an FD bus — so a link says which wire it is. Sixty-four data
    /// bytes per CAN frame instead of eight ([`CanWire::Fd`]).
    CanFd { interface: String, uuid: [u8; 6] },
    /// Klipper's host library, loaded from a shared object.
    ///
    /// `uuid` is the identity the board answers to when the library path is
    /// reached through a mux (`mcu/host_libraries.rs`): a path one section names
    /// without a `uuid` is addressed with bare frames, today's mode. The
    /// transport holds it; whether the path **is** muxed is the whole config's
    /// call, not this section's.
    Host {
        library: String,
        uuid: Option<[u8; 6]>,
    },
    /// A dictionary-driven fake MCU, in test builds (`test: dict=<path>`).
    #[cfg(test)]
    Simulator(String),
    /// A device on a bridge downstream port — `uart:` or `can:` naming a
    /// `[uart_device <name>]` / `[can_device <name>]` section (M1's port
    /// sections, `mcu/bridge_ports.rs`).
    ///
    /// `port` is the port **section identifier** the device hangs on
    /// (`uart_device uart0` — the spelling the grouping pass and the section
    /// loader both key by), `kind` what the port section is declared as, and
    /// `uuid` the identity the board answers the mux with: mandatory on a CAN
    /// port (rule 6 — the port grouping is the one place that refuses a
    /// member without one, `mcu/port_grouping.rs`), optional on a uart one,
    /// where its absence is rule 5's legacy shape. Whether the port is shared
    /// is likewise the whole config's call, like a shared `serial:` path.
    /// Opening one is not [`Transport::open`]'s path: `McuObject::connect`
    /// routes it to the attach pass (`mcu/bridge_attach.rs`), which reconciles
    /// the bridge's port table and opens the downstream handle on the bridge's
    /// own link.
    BridgePort {
        port: String,
        uuid: Option<[u8; 6]>,
        kind: PortKind,
    },
}

/// Parse a CAN uuid: six bytes as twelve hex digits.
///
/// The canonical key is `uuid:`; `canbus_uuid:` is the older name the option
/// still loads under (a config written for Klipper keeps working).
fn parse_uuid(text: &str) -> Result<[u8; 6], String> {
    let text = text.trim();
    if text.len() != 12 {
        return Err(format!("expected 12 hex digits, got {}", text.len()));
    }
    let mut uuid = [0u8; 6];
    for (i, byte) in uuid.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&text[i * 2..i * 2 + 2], 16)
            .map_err(|_| format!("'{text}' is not hexadecimal"))?;
    }
    Ok(uuid)
}

/// The connection keys a `[mcu]` section may carry, in the order errors list
/// them. Exactly one of them names the transport, the way Klipper's own `[mcu]`
/// works (`serial` or `uuid`): see [`McuConfig::create_interface`].
///
/// `uart` and `can` name a bridge downstream **port section** — a
/// `[uart_device <name>]` / `[can_device <name>]` (M1's port sections) —
/// rather than a device path.
///
/// `uuid` is the canonical CAN key and `canbus_uuid` its backwards-compatible
/// alias (a config written for Klipper keeps loading); both count as a CAN
/// interface key, so neither can trip the unused-option check on its own.
///
/// `test` exists in test builds only — the `upstream` harness uses
/// `test: dict=<path>` to inject a dictionary-driven fake MCU.
fn interface_keys() -> &'static [&'static str] {
    #[cfg(test)]
    return &[
        "host_library",
        "serial",
        "uart",
        "can",
        "uuid",
        "canbus_uuid",
        "test",
    ];
    #[cfg(not(test))]
    return &[
        "host_library",
        "serial",
        "uart",
        "can",
        "uuid",
        "canbus_uuid",
    ];
}

/// The upper bound of an MCU-layer id.
///
/// It comes from the message that carries an assigned id, not from the bus: the
/// CAN admin message puts it in a **single byte**
/// (`can_process_set_klipper_nodeid` decodes `msg->data[7]` via
/// `can_decode_nodeid` — `src/generic/canserial.c:140-143,177`), so the largest
/// id the link can express is `0xff`. CAN is only the transport that constrains
/// the bound; the id itself belongs to every `[mcu]` whatever it is connected
/// by. The `[mcu_ids]` allocator reuses it as the upper end of its one sequence
/// for all transports (`mcu/ids.rs`), so an auto id can never leave the range.
pub(crate) const MAX_MCU_ID: u32 = 0xff;

impl McuConfig {
    /// Parse MCU configuration from a ConfigSection.
    ///
    /// The name comes from the section's sub (`[mcu zboard]` → `zboard`), and the
    /// interface from one of the connection keys (see `interface_keys`):
    ///
    /// ```ini
    /// [mcu]
    /// serial: /dev/ttyACM0
    /// baud: 250000
    ///
    /// [mcu simulated]
    /// host_library: /path/to/libklipper_host.so
    /// ```
    ///
    /// A host library may carry a `uuid` too: it names the board a mux
    /// addresses that library path by, and decides nothing on its own
    /// (`mcu/host_libraries.rs`).
    ///
    /// A CAN-connected MCU names its uuid and the CAN interface it is on
    /// (Klipper's default is `can0`). The id is the MCU-layer id from the
    /// `[mcu_ids]` allocator (`mcu/ids.rs`; the historical `[canbus_ids]`
    /// spelling still loads) unless the section overrides it explicitly:
    ///
    /// ```ini
    /// [mcu]
    /// uuid: 11aa22bb33cc
    /// canbus_interface: can0
    ///
    /// [mcu bed]
    /// uuid: 44dd55ee66ff
    /// canbus_nodeid: 2
    /// ```
    ///
    /// `canbus_nodeid` is klipperx-only (upstream has no such option): it names the
    /// MCU-layer id this `[mcu]` takes instead of the one the allocator would hand
    /// out (the key keeps Klipper's spelling).
    ///
    /// How the link is carried is one further key, orthogonal to the uuid and
    /// the id:
    ///
    /// ```ini
    /// [mcu]
    /// uuid: 11aa22bb33cc
    /// canbus_fd: true          # `false` by default, the classical wire
    /// ```
    ///
    /// `canbus_fd` picks the wire the socket asks the kernel for. It only
    /// applies to the CAN family; written on a section with another transport
    /// it names a second interface, like every other `canbus_*` key.
    ///
    /// `baud` only applies to `serial`, and defaults to Klipper's 250000.
    ///
    /// A device on a bridge downstream port names the port **section**
    /// instead of a device path: `uart: <name>` for a `[uart_device <name>]`,
    /// `can: <name>` for a `[can_device <name>]` (`mcu/bridge_ports.rs`).
    /// The uuid rules follow the port: optional on uart (rule 5's legacy
    /// shape when absent), mandatory on CAN — the port grouping is the one
    /// place that refuses a CAN member without one
    /// (`mcu/port_grouping.rs`). A CAN port's member is a CAN device, so
    /// `canbus_nodeid` overrides its id the way it does on a direct CAN
    /// link; a uart port's member refuses it, as every non-CAN transport
    /// does.
    ///
    /// # Arguments
    /// * `section` — The MCU configuration section from the config file.
    ///
    /// # Returns
    /// `Ok(McuConfig)` with the appropriate interface type, or `Err` when the
    /// section names no interface, names several, or names one that cannot be
    /// brought up.
    pub fn new(section: &ConfigWrapper) -> Result<Self, ConfigError> {
        // Upstream names an MCU by its config section with the `mcu ` prefix
        // stripped (`klippy/mcu.py:1151-1153`): the main `[mcu]` is "mcu", and
        // `[mcu zboard]` is "zboard". Not an empty string for the main one.
        let name = section
            .section()
            .sub
            .clone()
            .unwrap_or_else(|| section.section().id.clone());
        let transport = Self::transport_for(section)?;
        // A CAN MCU's id is the node id it is addressed with. It is normally the
        // allocator's to hand out at load (`mcu/ids.rs`); the only value parsing
        // knows here is an explicit `canbus_nodeid`, read and validated now and
        // handed to the allocator when the MCU registers (`mcu/object.rs`).
        // A CAN port's member is a CAN device the same way, so the override
        // rides it unchanged (the override is orthogonal to how the link is
        // reached). Every other transport has no id yet — `register` assigns
        // one at load.
        let id = match &transport {
            Transport::Can { .. }
            | Transport::CanFd { .. }
            | Transport::BridgePort {
                kind: PortKind::Can,
                ..
            } => match section.get_str("canbus_nodeid") {
                Some(text) => match text.parse::<u32>() {
                    Ok(id) if (1..=MAX_MCU_ID).contains(&id) => Some(id),
                    _ => {
                        return Err(ConfigError::new(format!(
                            "MCU '{}' has an invalid canbus_nodeid: '{text}' \
                             (expected 1..={MAX_MCU_ID})",
                            section.identifier()
                        )))
                    }
                },
                None => None,
            },
            _ => None,
        };
        let serial = matches!(transport, Transport::Serial { .. });
        // A serial reset is line-wide: `arduino` toggles the shared line's DTR,
        // `cheetah` drives its RTS, and `rpi_usb` cuts its power, so any of them
        // would interrupt every MCU on the port. A path two or more `[mcu]`
        // sections name is therefore pinned to `command` (`parse_restart_method`).
        let shared = match &transport {
            Transport::Serial { path, .. } => serial_ports::path_is_shared(section, path),
            _ => false,
        };

        // The option only means something on a serial port or on a bridge-port
        // device, where `admin` is the one value that applies. Say so rather
        // than dropping it silently, then report what the MCU will actually
        // use.
        if !serial
            && !matches!(transport, Transport::BridgePort { .. })
            && section.has("restart_method")
        {
            warn!(
                "MCU '{name}' sets restart_method, which only applies to a serial MCU; \
                 resetting with command"
            );
        }
        let (restart_method, bridge_restart) =
            Self::parse_restart_method(section, &transport, shared)?;
        let usb_power = Self::parse_usb_power(section, &restart_method)?;

        Ok(Self {
            name,
            restart_method,
            bridge_restart,
            transport,
            id,
            shared_baud: None,
            host_mux: None,
            host_port_paths: Vec::new(),
            usb_power,
        })
    }

    /// Resolve the `usb_power` option: which mechanism `rpi_usb` uses.
    ///
    /// Only meaningful with `restart_method: rpi_usb`; a value given for another
    /// method is reported and ignored, the way `restart_method` itself is off a
    /// serial MCU.
    fn parse_usb_power(
        section: &ConfigWrapper,
        restart_method: &McuRestartMethod,
    ) -> Result<UsbPowerMethod, ConfigError> {
        let Some(text) = section.get_str("usb_power") else {
            return Ok(UsbPowerMethod::default());
        };
        let method = UsbPowerMethod::parse(&text).ok_or_else(|| {
            ConfigError::new(format!(
                "MCU '{}' has an invalid usb_power: '{text}' (expected one of {})",
                section.identifier(),
                UsbPowerMethod::CHOICES.join(", ")
            ))
        })?;
        if *restart_method != McuRestartMethod::RpiUsb {
            warn!(
                "MCU '{}' sets usb_power, which only applies to restart_method 'rpi_usb'; \
                 ignored",
                section.identifier()
            );
        }
        Ok(method)
    }

    /// Resolve `canbus_fd`: which CAN wire the link uses.
    ///
    /// Unset is the classical wire (eight data bytes per CAN frame) — the same
    /// link a section with no `canbus_fd` has always parsed to. The option is
    /// read through the wrapper's boolean parse, so a value that is not a
    /// boolean is refused there and the read counts for the undefined-option
    /// check.
    fn parse_can_wire(section: &ConfigWrapper) -> Result<CanWire, ConfigError> {
        let fd = section.get_bool("canbus_fd", Some(false))?;
        Ok(if fd { CanWire::Fd } else { CanWire::Classical })
    }

    /// Open the transport this config describes.
    ///
    /// An exclusive serial port opens the tty directly and sends **no**
    /// assignment frame; a shared one completes the synchronous assignment
    /// handshake inside [`serial::attach`] before it is handed back (see
    /// [`Transport::open`]).
    ///
    /// # Errors
    /// Returns the transport's own message (`serial: …`, `canbus: …`,
    /// `host_library: …`) when it cannot be opened.
    pub fn open(&self) -> Result<Interface, String> {
        // Cheetah boards need RTS deasserted for the **whole** connection, not
        // just while resetting, or the board drops into its bootloader
        // (`klippy/mcu.py:703-705`). Every other method wants the line left as
        // the driver opened it. A shared serial port ignores this: several MCUs
        // one port cannot each drive its modem lines (`Transport::open` warns).
        let rts = self.restart_method != McuRestartMethod::Cheetah;
        self.transport.open(
            self.id,
            rts,
            self.shared_baud,
            self.host_mux,
            &self.host_port_paths,
        )
    }

    /// Resolve `restart_method`, and which port-level reset (if any) a
    /// bridge-port device takes.
    ///
    /// Upstream reads the option **only** when the MCU is on a serial port
    /// (`klippy/mcu.py:668-671`); every other transport resets with `command`.
    /// An unknown value is a config error rather than silently becoming the
    /// default — upstream's `getchoice` refuses it too (`:666-671`).
    ///
    /// A **shared** serial port (`mcu/serial_ports.rs`) is pinned to `command`:
    /// every serial method acts on the whole line, which several MCUs are on,
    /// so the written value is not used. Only a value that was **written** and
    /// asks for another method is reported; an unset `restart_method` (whose
    /// serial default is `arduino`) is a silent downgrade, and an unknown
    /// spelling is still a config error.
    ///
    /// A device behind a bridge port is the third case, resolved by
    /// [`McuConfig::parse_bridge_restart`].
    ///
    /// `admin` is the one value both shapes with an admin channel use, and which
    /// channel it is depends on the transport: a **serial** MCU is the bridge
    /// itself, so the host restarts it over its mux admin channel before the
    /// port opens (`mcu/restart.rs`), while a **bridge port** child that writes
    /// a `uuid:` is restarted by the bridge forwarding the command. A **shared**
    /// serial port keeps `admin` too: the frame travels the admin id, not the
    /// line's modem lines, so it interrupts no other MCU the way a DTR toggle
    /// would — the reason the line-wide methods are downgraded does not apply.
    ///
    /// # Returns
    /// The MCU's own method, and the port-level reset a bridge-port device uses
    /// — `None` unless this section is one ([`BridgeRestart`]).
    ///
    /// `transport` is a parameter rather than a look at an opened interface so
    /// the rules can be exercised without a real port.
    fn parse_restart_method(
        section: &ConfigWrapper,
        transport: &Transport,
        shared: bool,
    ) -> Result<(McuRestartMethod, Option<BridgeRestart>), ConfigError> {
        // Read the option even when the transport is not serial: the read is
        // what marks it valid to the undefined-option check, so a
        // `restart_method` on a non-serial MCU is warned about (in `new`)
        // rather than rejected as an unused option. Upstream reads it only for
        // a serial port; a config whose transport was substituted (the
        // regression harness's `test:` fake) still names it.
        let text = section.get_str("restart_method");

        if let Transport::BridgePort { port, uuid, kind } = transport {
            return Self::parse_bridge_restart(section, port, *kind, uuid.is_some(), text);
        }

        if !matches!(transport, Transport::Serial { .. }) {
            if text
                .as_deref()
                .is_some_and(|text| McuRestartMethod::parse(text) == Some(McuRestartMethod::Admin))
            {
                return Err(Self::admin_without_an_admin_channel(section));
            }
            return Ok((McuRestartMethod::Command, None));
        }
        // The value the option asks for, resolved the way an exclusive port
        // uses it: unset is `arduino`, an unknown spelling is a config error.
        let written = match &text {
            None => McuRestartMethod::Arduino,
            Some(text) => Self::parse_method(section, text)?,
        };
        // The bridge's own restart is an admin unit, not a modem-line sequence:
        // it reaches the mux's admin id, so a shared port keeps it (the
        // line-wide methods below are pinned to `command` for the opposite
        // reason).
        if written == McuRestartMethod::Admin {
            return Ok((written, None));
        }
        if !shared {
            return Ok((written, None));
        }
        // A value that was written and asks for a line-wide reset cannot be
        // honoured: say which one it was, then reset with `command` instead.
        if let Some(text) = &text {
            if written != McuRestartMethod::Command {
                warn!(
                    "MCU '{}' sets restart_method '{text}', which resets the whole line; its \
                     serial port is shared with another MCU, so it resets with 'command'",
                    section.identifier()
                );
            }
        }
        Ok((McuRestartMethod::Command, None))
    }

    /// Resolve a bridge-port device's reset.
    ///
    /// A **named** device (a `uuid:` on the child) answers the bridge's mux
    /// itself, so it has an admin channel to be told to restart its own
    /// firmware: `restart_method: admin` is the one value that means something on
    /// such a section, and it becomes [`BridgeRestart::Firmware`]. `admin` on any
    /// other bridge-port section is a config error — a legacy device has no
    /// channel to forward to.
    ///
    /// A **legacy** device (no `uuid:`) has no channel of its own, so only its
    /// bridge can reset it, by pulsing the port's lines; that is the **port
    /// section's** `uart_restart_method` ([`BridgeRestart::Port`]). The child
    /// writes none of it — the port owns the choice.
    ///
    /// Every other value is reported and ignored, like the option off a serial
    /// port, and leaves the device resetting with `command`.
    fn parse_bridge_restart(
        section: &ConfigWrapper,
        port: &str,
        kind: PortKind,
        named: bool,
        text: Option<String>,
    ) -> Result<(McuRestartMethod, Option<BridgeRestart>), ConfigError> {
        let method = match &text {
            Some(text) if McuRestartMethod::parse(text) == Some(McuRestartMethod::Admin) => {
                if !named {
                    return Err(Self::admin_without_an_admin_channel(section));
                }
                McuRestartMethod::Admin
            }
            None => McuRestartMethod::Command,
            Some(text) => {
                // The spelling is still validated, and a value that is not
                // `admin` does not apply here: only a named device has a
                // channel to reset itself, and the line-level methods act on a
                // port the child does not own. `command` is what applies, so
                // only a value asking for **another** method is reported.
                let written = Self::parse_method(section, text)?;
                if written != McuRestartMethod::Command {
                    warn!(
                        "MCU '{}' sets restart_method '{text}', which only applies to a serial MCU \
                         or a named (uuid-bearing) bridge-port device; resetting with command",
                        section.identifier()
                    );
                }
                McuRestartMethod::Command
            }
        };
        let restart = if method == McuRestartMethod::Admin {
            Some(BridgeRestart::Firmware)
        } else if !named && kind == PortKind::Uart {
            Self::legacy_port_restart(section, port)
        } else {
            None
        };
        Ok((method, restart))
    }

    /// The port-level reset a legacy bridge-port device gets.
    ///
    /// The choice is the **port section's** `uart_restart_method`, read from the
    /// sibling section (raw text: the port's own factory is what records the
    /// option as read). Absent is `None` — today's behaviour, where the child
    /// resets with `command` and nothing acts before its transport opens. A
    /// written line-level method (`arduino`/`cheetah`) becomes
    /// [`BridgeRestart::Port`]. Every other value is the port factory's to
    /// refuse (`mcu/bridge_ports.rs`), so one that slips through here — the
    /// child is parsed before the port section — is simply not a reset this
    /// device can take.
    fn legacy_port_restart(section: &ConfigWrapper, port: &str) -> Option<BridgeRestart> {
        let text = section
            .sibling(port)
            .and_then(|port| port.section().get_text("uart_restart_method"))?;
        match McuRestartMethod::parse(text.trim()) {
            Some(McuRestartMethod::Arduino) | Some(McuRestartMethod::Cheetah) => {
                Some(BridgeRestart::Port)
            }
            _ => None,
        }
    }

    /// Parse one `restart_method` spelling, reporting an unknown one.
    fn parse_method(section: &ConfigWrapper, text: &str) -> Result<McuRestartMethod, ConfigError> {
        McuRestartMethod::parse(text).ok_or_else(|| {
            ConfigError::new(format!(
                "MCU '{}' has an invalid restart_method: '{text}' (expected one of {})",
                section.identifier(),
                McuRestartMethod::CHOICES.join(", ")
            ))
        })
    }

    /// The error for `restart_method: admin` on a section with **no** admin
    /// channel to use.
    ///
    /// `admin` acts on the mux admin channel, and only two shapes have one: a
    /// serial MCU, which is the bridge restarting itself, and a child behind a
    /// bridge port that writes a `uuid:`, whose restart the bridge forwards to
    /// its port. Every other section — a CAN MCU, a host library, a legacy
    /// (uuid-less) bridge-port device — has neither, so the error names the two
    /// shapes rather than silently downgrading a *written* choice that can never
    /// work.
    fn admin_without_an_admin_channel(section: &ConfigWrapper) -> ConfigError {
        ConfigError::new(format!(
            "MCU '{}': restart_method 'admin' needs an admin channel — a 'serial:' link, where \
             the bridge restarts itself, or a bridge port with a 'uuid:', where the bridge \
             forwards the command to the device",
            section.identifier()
        ))
    }

    /// Describe the transport the section asks for, without opening it.
    ///
    /// One connection selector names it: `host_library`, `serial`, `uart`/
    /// `can` (a bridge downstream port section), `test`, or the CAN options
    /// (`canbus_interface`/`canbus_nodeid`). Two of them is a
    /// configuration mistake, not a preference order, so it is reported rather
    /// than resolved silently — except that `uuid`/`canbus_uuid` are the
    /// *identity* the board is addressed by, not a selector: they ride on
    /// whichever transport wins and may sit on a serial, a CAN, a
    /// port-referencing or a host-library section. Every check lives here;
    /// [`Transport::open`] only performs the side effects. What a `uart:` /
    /// `can:` section cannot know alone — is the port shared, is its one
    /// unnamed device legacy, does every CAN member write a uuid — is the
    /// port grouping's call ([`port_grouping::validate`]), run here once per
    /// referencing section exactly as [`host_libraries::validate`] runs for a
    /// host-library one.
    fn transport_for(section: &ConfigWrapper) -> Result<Transport, ConfigError> {
        // The keys that can name or configure a transport, in the order the
        // conflict error lists the present ones. `uuid`/`canbus_uuid` are the
        // identity, not selectors, but they belong in the list so a conflict
        // is reported with every key involved. The rest of the CAN family —
        // `canbus_interface`/`canbus_nodeid` and the option that only shapes
        // that link — sits below the `interface_keys` the conflict counter used
        // to count, and is listed here for the same reason.
        let present = || {
            interface_keys()
                .iter()
                .copied()
                .chain(["canbus_interface", "canbus_nodeid", "canbus_fd"].into_iter())
                .filter(|key| section.has(key))
                .collect::<Vec<_>>()
        };
        let too_many = || {
            ConfigError::new(format!(
                "MCU '{}' sets more than one interface: {}",
                section.identifier(),
                present().join(", ")
            ))
        };
        let both_uuids = || {
            // `uuid` is the canonical key; `canbus_uuid` is its earlier name
            // and still loads (a config written for Klipper keeps working).
            // Writing both names the identity twice, so it is a conflict
            // reported up front with the canonical name first.
            ConfigError::new(format!(
                "MCU '{}' sets both uuid and canbus_uuid",
                section.identifier()
            ))
        };

        if let Some(library) = section.get_str("host_library") {
            // The library is one transport; `serial`, `test` or anything in
            // the CAN family with it names a second. `uuid`/`canbus_uuid` are
            // the *identity* here too, not a selector: a host MCU may carry one
            // (it names the board a mux addresses the library path by).
            let other = [
                "serial",
                "uart",
                "can",
                "test",
                "canbus_interface",
                "canbus_nodeid",
                "canbus_fd",
            ]
            .into_iter()
            .filter(|key| section.has(key))
            .collect::<Vec<_>>();
            if !other.is_empty() {
                return Err(too_many());
            }
            if section.has("uuid") && section.has("canbus_uuid") {
                return Err(both_uuids());
            }
            let uuid = section
                .get_str("uuid")
                .or_else(|| section.get_str("canbus_uuid"))
                .map(|text| {
                    parse_uuid(&text).map_err(|e| {
                        ConfigError::new(format!(
                            "MCU '{}' has an invalid uuid: {e}",
                            section.identifier()
                        ))
                    })
                })
                .transpose()?;
            // Whether this path is muxed is a property of the whole config:
            // what a host library path is addressed by depends on every `[mcu]`
            // that names it. A conflict is a config error reported here, at the
            // section's own parse, exactly as `serial_ports::path_is_shared`
            // answers the shared-port question for a serial section.
            host_libraries::validate(section)?;
            return Ok(Transport::Host { library, uuid });
        }

        if let Some(path) = section.get_str("serial") {
            let other = [
                "host_library",
                "uart",
                "can",
                "test",
                "canbus_interface",
                "canbus_nodeid",
                "canbus_fd",
            ]
            .into_iter()
            .filter(|key| section.has(key))
            .collect::<Vec<_>>();
            if !other.is_empty() {
                return Err(too_many());
            }
            if section.has("uuid") && section.has("canbus_uuid") {
                return Err(both_uuids());
            }
            // One uuid spelling names the board the port talks to; it only
            // changes how the **shared** path's handshake begins (`Transport::open`).
            let uuid = section
                .get_str("uuid")
                .or_else(|| section.get_str("canbus_uuid"))
                .map(|text| {
                    parse_uuid(&text).map_err(|e| {
                        ConfigError::new(format!(
                            "MCU '{}' has an invalid uuid: {e}",
                            section.identifier()
                        ))
                    })
                })
                .transpose()?;
            let baud = match section.get_str("baud") {
                // A rate of zero would ask the kernel to hang the line up.
                Some(text) => match text.parse::<u32>() {
                    Ok(baud) if baud > 0 => baud,
                    _ => {
                        return Err(ConfigError::new(format!(
                            "MCU '{}' has an invalid baud: '{text}'",
                            section.identifier()
                        )))
                    }
                },
                None => crate::core::klippy::interface::devices::serial::DEFAULT_BAUD,
            };
            return Ok(Transport::Serial { path, baud, uuid });
        }

        // A `uart:`/`can:` section references a bridge downstream port
        // **section** (`[uart_device <sub>]` / `[can_device <sub>]`, M1's port
        // sections) rather than a device path, so the port branches sit after
        // the direct ones and before the CAN-uuid fallback below — a `can:`
        // child is a port member, never a direct CAN link.
        for kind in [PortKind::Uart, PortKind::Can] {
            let Some(text) = section.get_str(kind.reference_option()) else {
                continue;
            };
            // Every other selector — the other port kind included — names a
            // second transport. `canbus_nodeid` is an id override rather than
            // a selector: a CAN port's member is a CAN device and takes it
            // the way a direct CAN link does, while no non-CAN transport has
            // ever accepted it (`serial:`/`host_library:` refuse it), so a
            // uart port's member refuses it with the same conflict. The
            // CAN family's shaping option (`canbus_fd`)
            // belongs to the bridge's own link, not to a device hanging on one
            // of its ports, so a port child refuses them the same way.
            let mut other = [
                "host_library",
                "serial",
                "uart",
                "can",
                "test",
                "canbus_interface",
                "canbus_fd",
            ]
            .into_iter()
            .filter(|key| *key != kind.reference_option() && section.has(key))
            .collect::<Vec<_>>();
            if kind == PortKind::Uart && section.has("canbus_nodeid") {
                other.push("canbus_nodeid");
            }
            if !other.is_empty() {
                return Err(too_many());
            }
            if section.has("uuid") && section.has("canbus_uuid") {
                return Err(both_uuids());
            }
            // One uuid spelling names the board the port talks to. Optional on
            // uart (the absence is rule 5's legacy shape), mandatory on CAN —
            // and the CAN refusal is the port grouping's alone
            // (`port_grouping::validate`), so it is not repeated here.
            let uuid = section
                .get_str("uuid")
                .or_else(|| section.get_str("canbus_uuid"))
                .map(|text| {
                    parse_uuid(&text).map_err(|e| {
                        ConfigError::new(format!(
                            "MCU '{}' has an invalid uuid: {e}",
                            section.identifier()
                        ))
                    })
                })
                .transpose()?;
            // The reference must resolve to a port section of the kind the
            // option spells: existence is a config-section check, so it is
            // independent of load order. The message names both the child
            // section and the port section asked for.
            let sub = text.trim();
            let port = format!("{} {sub}", kind.section_type());
            if !section.has_sibling(&port) {
                return Err(ConfigError::new(format!(
                    "MCU '{}' references port '{sub}' via '{}', but the config has no [{port}] \
                     section",
                    section.identifier(),
                    kind.reference_option()
                )));
            }
            // The port's topology — shared or exclusive, legacy or named,
            // every CAN member named — is the whole config's call, checked by
            // the grouping pass once per referencing section (the same
            // precedent `host_libraries::validate` sets above).
            port_grouping::validate(section)?;
            return Ok(Transport::BridgePort { port, uuid, kind });
        }

        #[cfg(test)]
        if section.has("test") {
            // A `test:` simulator and a CAN MCU are two transports.
            if [
                "uuid",
                "canbus_uuid",
                "canbus_interface",
                "canbus_nodeid",
                "canbus_fd",
            ]
            .into_iter()
            .any(|key| section.has(key))
            {
                return Err(too_many());
            }
            if let Some(test_value) = section.get_str("test") {
                if let Some(path) = test_value.trim().strip_prefix("dict=") {
                    return Ok(Transport::Simulator(path.trim().to_string()));
                }
            }
        }

        // The remaining interface is the CAN family: a uuid (either spelling)
        // names a CAN MCU, and `canbus_interface`/`canbus_nodeid` only set its
        // interface and id while `canbus_fd` shapes the link itself — which wire
        // it is.
        if section.has("uuid") && section.has("canbus_uuid") {
            return Err(both_uuids());
        }
        if let Some(text) = section
            .get_str("uuid")
            .or_else(|| section.get_str("canbus_uuid"))
        {
            let uuid = parse_uuid(&text).map_err(|e| {
                ConfigError::new(format!(
                    "MCU '{}' has an invalid uuid: {e}",
                    section.identifier()
                ))
            })?;
            let interface = section
                .get_str("canbus_interface")
                .unwrap_or_else(|| "can0".to_string());
            // The node id is not read here: it is the MCU's id at the
            // `[mcu]` level (see `McuConfig::id`), where parsing keeps only an
            // explicit `canbus_nodeid` and the allocator the rest
            // (`mcu/object.rs`, `mcu/ids.rs`).
            let wire = Self::parse_can_wire(section)?;
            return Ok(match wire {
                CanWire::Classical => Transport::Can { interface, uuid },
                CanWire::Fd => Transport::CanFd { interface, uuid },
            });
        }

        if section.has("canbus_nodeid")
            || section.has("canbus_interface")
            || section.has("canbus_fd")
        {
            return Err(ConfigError::new(format!(
                "MCU '{}' needs a uuid to go with its CAN settings",
                section.identifier()
            )));
        }

        let how = if cfg!(test) {
            "set host_library: <libklipper_host.so>, serial: <tty>, uuid: <hex>, \
             or test: dict=<dictionary>"
        } else {
            "set host_library: <libklipper_host.so>, serial: <tty>, or uuid: <hex>"
        };
        Err(ConfigError::new(format!(
            "MCU '{}' needs an interface: {how}",
            section.identifier()
        )))
    }
}

/// The id a CAN family link addresses its node by, or the error that the
/// allocator has not handed one out yet.
///
/// Every CAN family variant needs it the same way: the id is the MCU's at the
/// `[mcu]` level, and the link cannot be opened before the allocator assigned
/// one (`mcu/ids.rs`, `mcu/object.rs`).
fn can_link_id(id: Option<u32>) -> Result<u32, String> {
    id.ok_or_else(|| "canbus: the MCU id has not been assigned".to_string())
}

impl Transport {
    /// Open the described transport.
    ///
    /// This is where the side effects — opening a tty, a CAN socket, or the host
    /// library — and the errors that name them live. Parsing, and every config
    /// check, happened in [`McuConfig::new`].
    ///
    /// `id` is the id the MCU was assigned at load (`mcu/ids.rs`): the CAN arm
    /// needs it to address the node, and a **shared** serial port needs it as
    /// the source of its arbitration-id header. `shared_baud` is the baud the
    /// path's MCUs share when two or more name this `serial:` path
    /// (`mcu/serial_ports.rs`), or `None` when it is exclusive. The serial arm
    /// splits on it:
    ///
    /// - `Some(baud)` — the path is shared: the port is opened once for every
    ///   MCU on it, and [`serial::attach`] performs the synchronous
    ///   assignment handshake before the shared handle is returned. When the
    ///   section also names a `uuid`, the handshake **starts by writing the
    ///   SET_NODEID unit that names the board** ([`serial::set_node_id_for`])
    ///   and only then re-confirms with the ordinary assignment request —
    ///   mirroring CAN's `CanSerialDevice::open`, which sends its SET_NODEID
    ///   when the link opens.
    /// - `None` — the path is exclusive: a private [`SerialDevice`], **no**
    ///   assignment frame is ever sent, and a `uuid` on the section is
    ///   ignored.
    ///
    /// `host_mux` is what the host-library arm attaches a muxed path with
    /// (`mcu/host_libraries.rs`): `Some` opens the library through
    /// [`Interface::host_mux`], giving it the path's identity and addressing its
    /// frames by `id`, while `None` is the bare-frame mode that sends nothing
    /// extra. Like `shared_baud`, it is decided for the whole config at load,
    /// not derived from this section alone.
    ///
    /// `host_port_paths` is what that same open tells the library before it
    /// comes up: the tty each of the MCU's bridge ports opens, as
    /// `(slot, path)`. It is how `[uart_device <name>] uart_path:` reaches the
    /// firmware at run time, instead of being a path the library was built
    /// with; an empty slice (any MCU that is not a host-library bridge) leaves
    /// every port on the library's own default.
    ///
    /// `rts` is the state an **exclusive** serial port is left in after
    /// opening; a shared one ignores it — and warns when it is `false` —
    /// because several MCUs would otherwise fight over the same modem line.
    /// The other transports ignore it.
    ///
    /// # Errors
    /// Returns a message prefixed with the transport that failed.
    pub fn open(
        &self,
        id: Option<u32>,
        rts: bool,
        shared_baud: Option<u32>,
        host_mux: Option<HostMux>,
        host_port_paths: &[(u8, String)],
    ) -> Result<Interface, String> {
        match self {
            Transport::Serial { path, baud, uuid } if shared_baud == Some(*baud) => {
                // A shared port is addressed by id, so it cannot be opened
                // before the allocator has handed one out (`mcu/ids.rs`).
                let id =
                    id.ok_or_else(|| format!("serial {path}: the MCU id has not been assigned"))?;
                if !rts {
                    warn!(
                        "serial {path}: shared port ignores restart_method 'cheetah'; \
                         RTS is not driven"
                    );
                }
                let handle =
                    serial::attach(Path::new(path), *baud, id, ASSIGN_ATTACH_TIMEOUT, *uuid)
                        .map_err(|e| format!("serial {path}: shared port attach failed: {e}"))?;
                Ok(Interface::shared_serial(handle))
            }
            // An exclusive port ignores its uuid: no SET_NODEID and no
            // assignment frame ever go out on it (see the doc comment above).
            Transport::Serial {
                path,
                baud,
                uuid: _,
            } => {
                let device = SerialDevice::open(path, *baud).map_err(|e| format!("serial: {e}"))?;
                if !rts {
                    device.set_rts(false).map_err(|e| format!("serial: {e}"))?;
                }
                Ok(Interface::from_serial(device))
            }
            Transport::Can { interface, uuid } => {
                // A link cannot be opened until the id is known: the allocator
                // fills it in at connect time (see `mcu/object.rs`), and only an
                // explicit `canbus_nodeid` is set this early.
                let id = can_link_id(id)?;
                Interface::canserial(interface, *uuid, id).map_err(|e| format!("canbus: {e}"))
            }
            // Klipper's can-serial link on a CAN FD wire: the socket is asked
            // for sixty-four-byte frames, and the same admin `SET_NODEID` opens
            // it.
            Transport::CanFd { interface, uuid } => {
                let id = can_link_id(id)?;
                Interface::canserial_with_wire(interface, *uuid, id, CanWire::Fd)
                    .map_err(|e| format!("canbus: {e}"))
            }
            // A muxed path is addressed by id, so it cannot be opened before
            // the allocator has handed one out (`mcu/ids.rs`); a path the config
            // does not call muxed opens bare, today's mode.
            Transport::Host { library, uuid: _ } => match host_mux {
                Some(mux) => {
                    let id = id.ok_or_else(|| {
                        format!("host_library {library}: the MCU id has not been assigned")
                    })?;
                    Interface::host_mux(library, mux, id, host_port_paths)
                        .map_err(|e| format!("host_library: {e}"))
                }
                None => Interface::host(library).map_err(|e| format!("host_library: {e}")),
            },
            // A bridge port's device is reached through the bridge itself —
            // its port table and the port-level `SET_PORT_NODEID` — so it is
            // not opened here: `McuObject::connect` routes this transport to
            // the attach pass (`mcu/bridge_attach.rs`), which opens the
            // downstream handle on the bridge's own link. This arm says so
            // rather than pretending the device came up, in the same place
            // every other open failure is said.
            Transport::BridgePort {
                port,
                kind,
                uuid: _,
            } => Err(format!(
                "{} {port}: a bridge-port device is opened by the attach pass \
                 (mcu/bridge_attach.rs), not by the transport",
                kind.section_type()
            )),
            #[cfg(test)]
            Transport::Simulator(path) => SimulatorDevice::new(path)
                .map(Interface::simulator)
                .map_err(|e| format!("test: {e}")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::assign;
    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::section::ConfigSection;
    use crate::core::klippy::config::value::ConfigValue;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::devices::serial::DEFAULT_BAUD;
    use crate::core::klippy::interface::pty::Pty;
    use crate::core::klippy::printer::Printer;
    use crate::core::klippy::reactor::ManualReactor;
    use std::io::{Read, Write};
    use std::os::fd::AsRawFd;
    use std::sync::{Arc, Mutex};
    use tracing_subscriber::layer::SubscriberExt;

    /// Helper that creates a `[mcu]` section with `serial: /fake/tty`.
    fn make_section(_test_lines: &[&str]) -> ConfigSection {
        let mut section = ConfigSection::new("mcu", None);
        section.parameters.insert(
            "serial".to_string(),
            ConfigValue::Single("/fake/tty".to_string()),
        );
        section
    }

    /// A section with one `key: value` parameter.
    fn section_with(key: &str, value: &str) -> ConfigSection {
        let mut section = ConfigSection::new("mcu", None);
        section
            .parameters
            .insert(key.to_string(), ConfigValue::Single(value.to_string()));
        section
    }

    /// Wrap a hand-built section the way the loader does.
    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    /// A serial transport for the restart-method rules, naming no real port.
    fn serial_transport() -> Transport {
        Transport::Serial {
            path: "/fake/tty".to_string(),
            baud: 250_000,
            uuid: None,
        }
    }

    /// A transport that is neither serial nor a bridge port.
    fn host_transport() -> Transport {
        Transport::Host {
            library: "/does/not/matter".to_string(),
            uuid: None,
        }
    }

    #[test]
    fn test_parse_mcu_config_with_name() {
        let mut section = make_section(&["01 02"]);
        section.sub = Some("mcu0".to_string());
        let result = McuConfig::new(&wrap(&section));
        assert!(result.is_ok());
        assert_eq!(result.unwrap().name, "mcu0");
    }

    #[test]
    fn test_parse_mcu_config_with_restart_method() {
        let mut section = make_section(&["01 02"]);
        section.parameters.insert(
            "restart_method".to_string(),
            ConfigValue::Single("rpi_usb".to_string()),
        );
        // A serial MCU reads the option; the value is taken as written.
        let (method, restart) =
            McuConfig::parse_restart_method(&wrap(&section), &serial_transport(), false).unwrap();
        assert_eq!(method, McuRestartMethod::RpiUsb);
        assert_eq!(restart, None);
    }

    #[test]
    fn test_a_serial_mcu_defaults_to_arduino() {
        let section = make_section(&["01 02"]);
        let (method, restart) =
            McuConfig::parse_restart_method(&wrap(&section), &serial_transport(), false).unwrap();
        assert_eq!(method, McuRestartMethod::Arduino);
        assert_eq!(restart, None);
    }

    #[test]
    fn test_an_unknown_restart_method_is_a_config_error() {
        let mut section = make_section(&["01 02"]);
        section.parameters.insert(
            "restart_method".to_string(),
            ConfigValue::Single("bogus".to_string()),
        );
        let err = McuConfig::parse_restart_method(&wrap(&section), &serial_transport(), false)
            .unwrap_err();
        assert!(err.to_string().contains("restart_method"), "{err}");
        assert!(err.to_string().contains("bogus"), "{err}");
        // The error names the valid choices, so the config can be fixed.
        assert!(err.to_string().contains("cheetah"), "{err}");
    }

    #[test]
    fn test_restart_method_spellings_round_trip() {
        // Every advertised choice parses back to a method that spells itself the
        // same way the config option did.
        for spelling in McuRestartMethod::CHOICES {
            let method = McuRestartMethod::parse(spelling).unwrap();
            assert_eq!(method.as_str(), *spelling);
        }
    }

    #[test]
    fn test_a_non_serial_mcu_resets_with_command_and_ignores_the_option() {
        let mut section = make_section(&["01 02"]);
        section.parameters.insert(
            "restart_method".to_string(),
            ConfigValue::Single("cheetah".to_string()),
        );
        // Upstream does not read the option off serial, so a value that is
        // meaningless there is not an error — it simply does not apply.
        let (method, restart) =
            McuConfig::parse_restart_method(&wrap(&section), &host_transport(), false).unwrap();
        assert_eq!(method, McuRestartMethod::Command);
        assert_eq!(restart, None);
    }

    #[test]
    fn test_a_non_serial_restart_method_is_recorded_as_read() {
        // The option is read (and so valid to the undefined-option check) even
        // when the transport is not serial; it is ignored with a warning rather
        // than rejected. A substituted transport (the regression harness's
        // `test:` fake) is the case that needs this.
        let mut section = make_section(&["01 02"]);
        section.parameters.insert(
            "restart_method".to_string(),
            ConfigValue::Single("command".to_string()),
        );
        let access = Arc::new(AccessTracking::new());
        let wrapper = ConfigWrapper::new(&section, Arc::clone(&access));

        let (method, restart) =
            McuConfig::parse_restart_method(&wrapper, &host_transport(), false).unwrap();

        assert_eq!(method, McuRestartMethod::Command);
        assert_eq!(restart, None);
        assert!(access.contains("mcu", "restart_method"));
    }

    #[test]
    fn test_usb_power_defaults_to_auto_and_is_validated() {
        // Unset: `auto`.
        let section = make_section(&["01 02"]);
        assert_eq!(
            McuConfig::parse_usb_power(&wrap(&section), &McuRestartMethod::RpiUsb).unwrap(),
            UsbPowerMethod::Auto
        );

        // A valid value is taken as written.
        let with = |value: &str| {
            let mut section = make_section(&["01 02"]);
            section.parameters.insert(
                "usb_power".to_string(),
                ConfigValue::Single(value.to_string()),
            );
            section
        };
        assert_eq!(
            McuConfig::parse_usb_power(&wrap(&with("sysfs")), &McuRestartMethod::RpiUsb).unwrap(),
            UsbPowerMethod::Sysfs
        );

        // A typo is a config error, not a silent default.
        let err = McuConfig::parse_usb_power(&wrap(&with("bogus")), &McuRestartMethod::RpiUsb)
            .unwrap_err();
        assert!(err.to_string().contains("usb_power"), "{err}");
        assert!(err.to_string().contains("bogus"), "{err}");

        // With another restart method the option has no effect; it is reported
        // and ignored rather than refused.
        assert_eq!(
            McuConfig::parse_usb_power(&wrap(&with("sysfs")), &McuRestartMethod::Arduino).unwrap(),
            UsbPowerMethod::Sysfs
        );
    }

    #[test]
    fn test_parse_mcu_config_no_interface() {
        let section = ConfigSection::new("mcu", None);
        let err = McuConfig::new(&wrap(&section)).unwrap_err();
        // The error has to say how to fix the section, not just that it is wrong.
        assert!(err.to_string().contains("needs an interface"), "{err}");
        assert!(err.to_string().contains("host_library"), "{err}");
    }

    #[test]
    fn test_host_library_key_becomes_the_host_transport() {
        // Parsing routes it to the host transport; opening is what touches the
        // library, so the failure is the library's, not "needs an interface".
        let section = section_with("host_library", "/nonexistent/libklipper_host.so");
        let config = McuConfig::new(&wrap(&section)).unwrap();
        assert!(matches!(&config.transport, Transport::Host { .. }));

        let err = config.open().unwrap_err();
        assert!(err.starts_with("host_library: "), "{err}");
        assert!(
            err.to_string().contains("/nonexistent/libklipper_host.so"),
            "{err}"
        );
    }

    #[test]
    fn test_two_interface_keys_are_rejected() {
        let mut section = section_with("host_library", "/nonexistent/libklipper_host.so");
        section.parameters.insert(
            "serial".to_string(),
            ConfigValue::Single("/dev/ttyACM0".to_string()),
        );

        let err = McuConfig::new(&wrap(&section)).unwrap_err();
        assert!(err.to_string().contains("more than one interface"), "{err}");
        assert!(err.to_string().contains("host_library, serial"), "{err}");
    }

    #[test]
    fn test_serial_key_becomes_the_serial_transport() {
        let section = section_with("serial", "/dev/not-a-serial-port");
        let config = McuConfig::new(&wrap(&section)).unwrap();
        assert!(matches!(
            config.transport,
            Transport::Serial { ref path, .. } if path == "/dev/not-a-serial-port"
        ));

        // A port that cannot be opened proves the routing: the error names the
        // port, which only the serial device's own error does.
        let err = config.open().unwrap_err();
        assert!(err.starts_with("serial: "), "{err}");
        assert!(err.to_string().contains("/dev/not-a-serial-port"), "{err}");
    }

    /// `serial:` with a `uuid:` is one transport carrying the board's identity:
    /// it parses (a shared-path handshake names the board with it) instead of
    /// being a two-interface conflict.
    #[test]
    fn test_a_serial_with_uuid_carries_the_identity() {
        let mut section = section_with("serial", "/dev/not-a-serial-port");
        section.parameters.insert(
            "uuid".to_string(),
            ConfigValue::Single("11aa22bb33cc".to_string()),
        );
        let config = McuConfig::new(&wrap(&section)).unwrap();
        assert!(matches!(
            &config.transport,
            Transport::Serial {
                path,
                uuid: Some(u),
                ..
            } if path == "/dev/not-a-serial-port"
                && u == &[0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc]
        ));
        // Still a serial transport — opening goes through the serial device.
        let err = config.open().unwrap_err();
        assert!(err.starts_with("serial: "), "{err}");
        assert!(err.to_string().contains("/dev/not-a-serial-port"), "{err}");
    }

    /// The old spelling loads on a serial MCU the same way it does on a CAN
    /// one: aliased to the same identity.
    #[test]
    fn test_a_serial_with_aliased_canbus_uuid_carries_the_identity() {
        let mut section = section_with("serial", "/dev/not-a-serial-port");
        section.parameters.insert(
            "canbus_uuid".to_string(),
            ConfigValue::Single("44dd55ee66ff".to_string()),
        );
        let config = McuConfig::new(&wrap(&section)).unwrap();
        assert!(matches!(
            &config.transport,
            Transport::Serial {
                uuid: Some(u), ..
            } if u == &[0x44, 0xdd, 0x55, 0xee, 0x66, 0xff]
        ));
    }

    /// Both spellings on a serial MCU are the same identity written twice, so
    /// the canonical-key-first error is the CAN one's.
    #[test]
    fn test_a_serial_with_both_uuid_spellings_is_rejected() {
        let mut section = section_with("serial", "/dev/ttyACM0");
        section.parameters.insert(
            "uuid".to_string(),
            ConfigValue::Single("11aa22bb33cc".to_string()),
        );
        section.parameters.insert(
            "canbus_uuid".to_string(),
            ConfigValue::Single("44dd55ee66ff".to_string()),
        );
        let err = McuConfig::new(&wrap(&section)).unwrap_err();
        assert!(
            err.to_string().contains("both uuid and canbus_uuid"),
            "{err}"
        );
    }

    /// A malformed uuid on a serial MCU keeps the CAN error's wording.
    #[test]
    fn test_a_serial_with_an_invalid_uuid_is_rejected() {
        for bad in ["bogus", "11aa22bb33cg"] {
            let mut section = section_with("serial", "/dev/ttyACM0");
            section
                .parameters
                .insert("uuid".to_string(), ConfigValue::Single(bad.to_string()));
            let err = McuConfig::new(&wrap(&section)).unwrap_err();
            assert!(err.to_string().contains("invalid uuid"), "{bad}: {err}");
        }
    }

    /// `uuid:` alone names the default CAN interface, unchanged by the serial
    /// identity work.
    #[test]
    fn test_uuid_alone_is_still_can() {
        let section = section_with("uuid", "11aa22bb33cc");
        let mut config = McuConfig::new(&wrap(&section)).unwrap();
        assert!(matches!(
            &config.transport,
            Transport::Can {
                ref interface,
                uuid,
            } if interface == "can0" && *uuid == [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc]
        ));
        // With an id it gets as far as the socket, which is where it fails here.
        config.id = Some(2);
        let err = config.open().unwrap_err();
        assert!(err.contains("no CAN interface named 'can0'"), "{err}");
    }

    /// CAN settings with a serial transport name two transports, not one MCU
    /// with both a tty and a CAN link.
    #[test]
    fn test_serial_plus_can_settings_is_rejected() {
        for can_key in ["canbus_interface", "canbus_nodeid"] {
            let mut section = section_with("serial", "/dev/ttyACM0");
            section
                .parameters
                .insert(can_key.to_string(), ConfigValue::Single("can0".to_string()));
            let err = McuConfig::new(&wrap(&section)).unwrap_err();
            assert!(err.to_string().contains("more than one interface"), "{err}");
        }
    }

    /// `host_library:` with a `uuid:` is one transport carrying the board's
    /// identity — the uuid a mux addresses the library path by
    /// (`mcu/host_libraries.rs`) — not a second interface.
    #[test]
    fn test_host_library_with_uuid_carries_the_identity() {
        let mut section = section_with("host_library", "/nonexistent/libklipper_host.so");
        section.parameters.insert(
            "uuid".to_string(),
            ConfigValue::Single("11aa22bb33cc".to_string()),
        );
        let config = McuConfig::new(&wrap(&section)).unwrap();
        assert!(matches!(
            config.transport,
            Transport::Host {
                uuid: Some(u),
                ..
            } if u == [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc]
        ));
    }

    /// The old spelling loads on a host MCU the same way it does on a serial or
    /// a CAN one: aliased to the same identity.
    #[test]
    fn test_host_library_with_aliased_canbus_uuid_carries_the_identity() {
        let mut section = section_with("host_library", "/nonexistent/libklipper_host.so");
        section.parameters.insert(
            "canbus_uuid".to_string(),
            ConfigValue::Single("44dd55ee66ff".to_string()),
        );
        let config = McuConfig::new(&wrap(&section)).unwrap();
        assert!(matches!(
            config.transport,
            Transport::Host {
                uuid: Some(u),
                ..
            } if u == [0x44, 0xdd, 0x55, 0xee, 0x66, 0xff]
        ));
    }

    /// Both spellings on a host MCU are the same identity written twice, so the
    /// canonical-key-first error is the CAN one's.
    #[test]
    fn test_host_library_with_both_uuid_spellings_is_rejected() {
        let mut section = section_with("host_library", "/nonexistent/libklipper_host.so");
        section.parameters.insert(
            "uuid".to_string(),
            ConfigValue::Single("11aa22bb33cc".to_string()),
        );
        section.parameters.insert(
            "canbus_uuid".to_string(),
            ConfigValue::Single("44dd55ee66ff".to_string()),
        );
        let err = McuConfig::new(&wrap(&section)).unwrap_err();
        assert!(
            err.to_string().contains("both uuid and canbus_uuid"),
            "{err}"
        );
    }

    /// A malformed uuid on a host MCU keeps the CAN error's wording.
    #[test]
    fn test_host_library_with_an_invalid_uuid_is_rejected() {
        for bad in ["bogus", "11aa22bb33cg"] {
            let mut section = section_with("host_library", "/nonexistent/libklipper_host.so");
            section
                .parameters
                .insert("uuid".to_string(), ConfigValue::Single(bad.to_string()));
            let err = McuConfig::new(&wrap(&section)).unwrap_err();
            assert!(err.to_string().contains("invalid uuid"), "{bad}: {err}");
        }
    }

    /// A CAN setting with a host library still names two transports: only
    /// `uuid`/`canbus_uuid` became the host transport's identity.
    #[test]
    fn test_host_library_plus_can_settings_is_rejected() {
        for can_key in ["canbus_interface", "canbus_nodeid"] {
            let mut section = section_with("host_library", "/nonexistent/libklipper_host.so");
            section
                .parameters
                .insert(can_key.to_string(), ConfigValue::Single("can0".to_string()));
            let err = McuConfig::new(&wrap(&section)).unwrap_err();
            assert!(err.to_string().contains("more than one interface"), "{err}");
        }
    }

    /// `test:` and a CAN MCU are two transports even in test builds.
    #[test]
    fn test_the_dict_interface_plus_a_uuid_is_rejected() {
        let mut section = section_with("test", "dict=/fake/path.dict");
        section.parameters.insert(
            "uuid".to_string(),
            ConfigValue::Single("11aa22bb33cc".to_string()),
        );
        let err = McuConfig::new(&wrap(&section)).unwrap_err();
        assert!(err.to_string().contains("more than one interface"), "{err}");
    }

    /// A CAN section, with `overrides` on top of a complete one.
    fn can_section(overrides: &[(&str, &str)]) -> ConfigSection {
        let mut section = section_with("uuid", "11aa22bb33cc");
        section.parameters.insert(
            "canbus_nodeid".to_string(),
            ConfigValue::Single("2".to_string()),
        );
        for (key, value) in overrides {
            section
                .parameters
                .insert(key.to_string(), ConfigValue::Single(value.to_string()));
        }
        section
    }

    #[test]
    fn test_canbus_keys_become_the_can_transport() {
        let section = can_section(&[("canbus_interface", "can99")]);
        let config = McuConfig::new(&wrap(&section)).unwrap();
        assert!(matches!(
            config.transport,
            Transport::Can { ref interface, .. } if interface == "can99"
        ));

        // No CAN interface in the test environment, so opening shows up as the
        // socket's error naming the interface we asked for.
        let err = config.open().unwrap_err();
        assert!(err.starts_with("canbus: "), "{err}");
        assert!(err.to_string().contains("can99"), "{err}");
    }

    #[test]
    fn test_uuid_is_parsed_as_klipper_writes_it() {
        assert_eq!(
            parse_uuid("11aa22bb33cc").unwrap(),
            [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc]
        );
        for bad in ["11aa22bb33c", "11aa22bb33ccdd", "11aa22bb33cg", ""] {
            let section = can_section(&[("uuid", bad)]);
            let err = McuConfig::new(&wrap(&section)).unwrap_err();
            assert!(err.to_string().contains("invalid uuid"), "{bad}: {err}");
        }
    }

    /// The old spelling of the key still loads, aliased to the same transport:
    /// a config written for Klipper's `canbus_uuid` parses to exactly the
    /// `Transport::Can` that `uuid:` would give it.
    #[test]
    fn test_aliased_canbus_uuid_still_becomes_the_can_transport() {
        let mut section = section_with("canbus_uuid", "11aa22bb33cc");
        section.parameters.insert(
            "canbus_interface".to_string(),
            ConfigValue::Single("can98".to_string()),
        );
        let config = McuConfig::new(&wrap(&section)).unwrap();
        assert!(matches!(
            config.transport,
            Transport::Can {
                ref interface,
                uuid,
            } if interface == "can98" && uuid == [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc]
        ));
    }

    /// Both spellings in one section name the uuid twice; the canonical key
    /// leads the error.
    #[test]
    fn test_uuid_and_its_alias_together_are_rejected() {
        let mut section = section_with("uuid", "11aa22bb33cc");
        section.parameters.insert(
            "canbus_uuid".to_string(),
            ConfigValue::Single("44dd55ee66ff".to_string()),
        );
        let err = McuConfig::new(&wrap(&section)).unwrap_err();
        assert!(
            err.to_string().contains("both uuid and canbus_uuid"),
            "{err}"
        );
    }

    #[test]
    fn test_canbus_nodeid_is_validated() {
        for bad in ["0", "fast", "256"] {
            let section = can_section(&[("canbus_nodeid", bad)]);
            let err = McuConfig::new(&wrap(&section)).unwrap_err();
            assert!(
                err.to_string().contains("invalid canbus_nodeid"),
                "{bad}: {err}"
            );
            // Pins the rendered range too: `MAX_MCU_ID` moving (or the message
            // losing its range) turns this red on its own.
            assert!(
                err.to_string().contains("(expected 1..=255)"),
                "{bad}: {err}"
            );
        }

        // 255 (`0xff`) is the largest id the single admin byte can carry, so the
        // upper bound itself is accepted; one past it (above) is not.
        let config = McuConfig::new(&wrap(&can_section(&[("canbus_nodeid", "255")]))).unwrap();
        assert_eq!(config.id, Some(255));

        // A valid node id gets as far as the socket, which is where it fails here.
        let err = McuConfig::new(&wrap(&can_section(&[])))
            .unwrap()
            .open()
            .unwrap_err();
        assert!(
            err.to_string().contains("no CAN interface named 'can0'"),
            "{err}"
        );
    }

    #[test]
    fn test_canbus_nodeid_is_optional_and_left_for_the_allocator() {
        let mut section = section_with("uuid", "11aa22bb33cc");
        // No `canbus_nodeid`: parsing knows no id; the allocator fills one in at
        // load (`mcu/ids.rs`), and `connect` puts it on the config before the
        // link is opened.
        let config = McuConfig::new(&wrap(&section)).unwrap();
        assert!(matches!(config.id, None));
        assert!(matches!(config.transport, Transport::Can { .. }));
        let err = config.open().unwrap_err();
        assert!(err.contains("MCU id has not been assigned"), "{err}");

        // Written explicitly, the option is the id parsing carries.
        section.parameters.insert(
            "canbus_nodeid".to_string(),
            ConfigValue::Single("7".to_string()),
        );
        let config = McuConfig::new(&wrap(&section)).unwrap();
        assert_eq!(config.id, Some(7));
    }

    #[test]
    fn test_canbus_settings_without_a_uuid_are_reported() {
        let section = section_with("canbus_nodeid", "2");
        let err = McuConfig::new(&wrap(&section)).unwrap_err();
        assert!(err.to_string().contains("needs a uuid"), "{err}");

        let section = section_with("canbus_interface", "can0");
        let err = McuConfig::new(&wrap(&section)).unwrap_err();
        assert!(err.to_string().contains("needs a uuid"), "{err}");

        // The shaping option is a CAN setting too: alone it names no link any
        // more than `canbus_nodeid` does.
        for (key, value) in [("canbus_fd", "true")] {
            let section = section_with(key, value);
            let err = McuConfig::new(&wrap(&section)).unwrap_err();
            assert!(err.to_string().contains("needs a uuid"), "{key}: {err}");
        }
    }

    /// The new key unset leaves the transport exactly what it was — a
    /// can-serial link on the classical wire — and the spelling of that
    /// default parses to the very same one.
    #[test]
    fn test_the_can_family_keys_default_to_todays_link() {
        let default = McuConfig::new(&wrap(&can_section(&[]))).unwrap();
        assert_eq!(
            default.transport,
            Transport::Can {
                interface: "can0".to_string(),
                uuid: [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc],
            }
        );
        for (key, value) in [("canbus_fd", "false")] {
            let written = McuConfig::new(&wrap(&can_section(&[(key, value)]))).unwrap();
            assert_eq!(written.transport, default.transport, "{key}: {value}");
        }
    }

    /// `canbus_fd: true` picks the same link on a CAN FD wire: one key, one
    /// transport, with everything else read as before.
    #[test]
    fn test_canbus_fd_selects_the_fd_can_link() {
        let section = can_section(&[("canbus_interface", "can99"), ("canbus_fd", "true")]);
        let config = McuConfig::new(&wrap(&section)).unwrap();
        assert_eq!(
            config.transport,
            Transport::CanFd {
                interface: "can99".to_string(),
                uuid: [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc],
            }
        );
        // The id override still rides the CAN family, and the link is opened the
        // can-serial way — so it fails on the socket the test environment does
        // not have, named the way that link names it.
        assert_eq!(config.id, Some(2));
        let err = config.open().unwrap_err();
        assert!(err.starts_with("canbus: "), "{err}");
        assert!(err.contains("no CAN interface named 'can99'"), "{err}");
    }

    /// `canbus_fd` is parsed strictly: a value that is not a boolean is refused
    /// by the wrapper's own boolean parse, which also records the read.
    #[test]
    fn test_the_can_family_keys_are_validated() {
        for bad in ["yes-please", "2", "fd"] {
            let err = McuConfig::new(&wrap(&can_section(&[("canbus_fd", bad)]))).unwrap_err();
            assert!(err.to_string().contains("canbus_fd"), "{bad}: {err}");
        }
        // Every spelling the wrapper's boolean parse accepts works, on the same
        // terms every other boolean option gets.
        for good in ["1", "yes", "true", "on", "0", "no", "false", "off"] {
            assert!(
                McuConfig::new(&wrap(&can_section(&[("canbus_fd", good)]))).is_ok(),
                "{good}"
            );
        }
    }

    /// The shaping key only shapes a CAN link. On a serial, a host library or a
    /// `test:` section it names a second interface; on a device hanging on a
    /// bridge port it would shape the **bridge's** link rather than the
    /// child's, so the child refuses it the way it refuses
    /// `canbus_interface`.
    #[test]
    fn test_the_can_family_keys_off_a_can_link_are_a_second_interface() {
        for key in ["canbus_fd"] {
            let value = "true";
            for (base, base_value) in [
                ("serial", "/dev/ttyACM0"),
                ("host_library", "/nonexistent/libklipper_host.so"),
                ("test", "dict=/fake/path.dict"),
            ] {
                let mut section = section_with(base, base_value);
                section
                    .parameters
                    .insert(key.to_string(), ConfigValue::Single(value.to_string()));
                let err = McuConfig::new(&wrap(&section)).unwrap_err();
                let text = err.to_string();
                assert!(
                    text.contains("more than one interface"),
                    "{base} + {key}: {text}"
                );
                assert!(text.contains(key), "the conflict names {key}: {text}");
            }
            for reference in ["uart: uart0", "can: can0"] {
                let port = if reference.starts_with("uart") {
                    "[uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n"
                } else {
                    "[can_device can0]\ncan_mcu: bridge\ncan_oid: can0\n"
                };
                let err = parse_section(
                    &format!(
                        "[mcu bridge]\nserial: /dev/a\n\
                         [mcu child]\n{reference}\nuuid: 11aa22bb33cc\n{key}: {value}\n{port}"
                    ),
                    "mcu child",
                )
                .unwrap_err();
                let text = err.to_string();
                assert!(
                    text.contains("more than one interface"),
                    "{reference} + {key}: {text}"
                );
            }
        }
    }

    #[test]
    fn test_invalid_baud_is_rejected_before_opening_the_port() {
        let mut section = section_with("serial", "/dev/not-a-serial-port");
        section
            .parameters
            .insert("baud".to_string(), ConfigValue::Single("fast".to_string()));
        let err = McuConfig::new(&wrap(&section)).unwrap_err();
        assert!(err.to_string().contains("invalid baud"), "{err}");
        assert!(err.to_string().contains("fast"), "{err}");
    }

    #[test]
    fn test_parse_mcu_config_with_dict_interface() {
        let mut section = ConfigSection::new("mcu", None);
        section.parameters.insert(
            "test".to_string(),
            ConfigValue::Single("dict=/fake/path.dict".to_string()),
        );
        let result = McuConfig::new(&wrap(&section));
        assert!(result.is_ok());
        let config = result.unwrap();
        assert_eq!(config.name, "mcu");
        assert!(matches!(config.transport, Transport::Simulator(_)));
        // A `test: dict=` MCU is not serial, so it resets with `command` like
        // every non-serial transport.
        assert_eq!(config.restart_method, McuRestartMethod::Command);
    }

    // -----------------------------------------------------------------------
    // Shared vs exclusive serial ports (`shared_baud`).
    // -----------------------------------------------------------------------

    /// Read exactly `want` bytes from the pty master (blocking).
    fn read_n(master: &std::fs::File, want: usize) -> Vec<u8> {
        let mut master = master;
        let mut out = Vec::with_capacity(want);
        let mut buf = [0u8; 64];
        while out.len() < want {
            let read = master.read(&mut buf).expect("pty master read");
            assert_ne!(read, 0, "the pty closed before {want} bytes arrived");
            out.extend_from_slice(&buf[..read]);
        }
        out
    }

    /// A parsed config naming `pty`'s slave path, exclusive unless a caller
    /// stamps `shared_baud`.
    fn serial_config(pty: &Pty) -> McuConfig {
        let section = section_with("serial", pty.path.to_str().expect("a utf-8 pty path"));
        McuConfig::new(&wrap(&section)).expect("a serial section parses")
    }

    /// Open `config` on a blocking thread while playing the device side of the
    /// assignment handshake from the pty master, and return what `open` did.
    ///
    /// `open` blocks until the device answers, so it cannot run on the test's
    /// own runtime thread; the reply is written here from the master.
    async fn open_answering_assignment(
        config: McuConfig,
        pty: &Pty,
        mcu_id: u32,
    ) -> Result<Interface, String> {
        let handle = tokio::runtime::Handle::current();
        let opener = tokio::task::spawn_blocking(move || {
            let _entered = handle.enter();
            config.open()
        });
        let request = serial::assignment_request(mcu_id).expect("a valid mcu id");
        assert_eq!(
            read_n(&pty.master, request.len()),
            request,
            "the assignment request goes out verbatim"
        );
        (&pty.master)
            .write_all(&serial::assignment_reply(mcu_id).expect("a valid mcu id"))
            .expect("pty master write");
        opener.await.expect("the opener task ran")
    }

    /// Like [`open_answering_assignment`], but the config carries a serial
    /// `uuid`: the device side sees the SET_NODEID unit first, then the
    /// assignment request, and answers the latter.
    async fn open_answering_assignment_with_uuid(
        config: McuConfig,
        pty: &Pty,
        mcu_id: u32,
        uuid: [u8; 6],
    ) -> Result<Interface, String> {
        let handle = tokio::runtime::Handle::current();
        let opener = tokio::task::spawn_blocking(move || {
            let _entered = handle.enter();
            config.open()
        });
        let set = serial::set_node_id_for(uuid, mcu_id).expect("a valid mcu id");
        let request = serial::assignment_request(mcu_id).expect("a valid mcu id");
        let wire = read_n(&pty.master, set.len() + request.len());
        assert_eq!(
            &wire[..set.len()],
            set.as_slice(),
            "the SET_NODEID unit goes out first"
        );
        assert_eq!(
            &wire[set.len()..],
            request.as_slice(),
            "the assignment request follows SET_NODEID"
        );
        (&pty.master)
            .write_all(&serial::assignment_reply(mcu_id).expect("a valid mcu id"))
            .expect("pty master write");
        opener.await.expect("the opener task ran")
    }

    /// A shared port completes the assignment handshake before `open` returns,
    /// and the interface it returns is the shared one — every send carries the
    /// MCU's arbitration header.
    #[tokio::test]
    async fn test_a_shared_serial_open_completes_the_assignment_handshake() {
        let pty = Pty::new();
        let mut config = serial_config(&pty);
        config.id = Some(7);
        config.shared_baud = Some(DEFAULT_BAUD);

        let interface = open_answering_assignment(config, &pty, 7)
            .await
            .expect("a shared attach succeeds once the device answers");

        // The shared device frames each send with the arb header; an exclusive
        // one would write the bare frame.
        let frame = Frame::new(0, vec![1, 2]);
        interface.send(frame.clone()).await.unwrap();
        let header = serial::encode_header(assign::arb_id(7).expect("a valid id"));
        let wire = read_n(&pty.master, header.len() + frame.raw_bytes().len());
        assert_eq!(
            &wire[..header.len()],
            header.as_slice(),
            "a shared send is prefixed with the arb header"
        );
        assert_eq!(&wire[header.len()..], frame.raw_bytes());
    }

    /// A shared port whose config names a `uuid` sends the SET_NODEID unit
    /// that names the board *before* the ordinary assignment request — the
    /// byte order a discovery-driven board expects on the wire.
    #[tokio::test]
    async fn test_a_shared_serial_open_with_uuid_names_the_board_before_the_handshake() {
        let pty = Pty::new();
        let mut section = section_with("serial", pty.path.to_str().unwrap());
        section.parameters.insert(
            "uuid".to_string(),
            ConfigValue::Single("11aa22bb33cc".to_string()),
        );
        let mut config = McuConfig::new(&wrap(&section)).expect("a serial+uuid section parses");
        assert!(matches!(
            config.transport,
            Transport::Serial { uuid: Some(_), .. }
        ));
        config.id = Some(5);
        config.shared_baud = Some(DEFAULT_BAUD);

        let interface = open_answering_assignment_with_uuid(
            config,
            &pty,
            5,
            [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc],
        )
        .await
        .expect("a shared attach with a uuid succeeds");
        // The handshake it completed is the shared path's (SET_NODEID then the
        // assignment request above); a shutdown keeps the handle well-formed.
        interface.shutdown();
    }

    /// A shared port with no answer fails `open`, naming the path and the
    /// attach that did not complete.
    #[test]
    fn test_a_shared_serial_open_fails_when_the_device_does_not_answer() {
        let pty = Pty::new();
        let mut config = serial_config(&pty);
        config.id = Some(7);
        config.shared_baud = Some(DEFAULT_BAUD);

        let err = config.open().unwrap_err();
        assert!(err.contains("shared port attach failed"), "{err}");
        assert!(err.contains(pty.path.to_str().unwrap()), "{err}");
    }

    /// A shared port cannot be opened before the allocator has handed out an
    /// id: the arb header it is addressed by needs one.
    #[test]
    fn test_a_shared_serial_open_without_an_id_is_refused() {
        let pty = Pty::new();
        let mut config = serial_config(&pty);
        config.shared_baud = Some(DEFAULT_BAUD);

        let err = config.open().unwrap_err();
        assert!(err.contains("the MCU id has not been assigned"), "{err}");
        assert!(err.contains(pty.path.to_str().unwrap()), "{err}");
    }

    /// An exclusive port opens exactly as before and writes **nothing** on the
    /// wire: the load-time handshake is the shared path's alone.
    #[tokio::test]
    async fn test_an_exclusive_serial_open_sends_no_assignment_frame() {
        let pty = Pty::new();
        let mut config = serial_config(&pty);
        config.id = Some(7);
        config.shared_baud = None;

        let interface = config.open().expect("an exclusive port opens");

        // Non-blocking master: a read either returns data (the bug this pins)
        // or says so with `WouldBlock` (nothing was ever sent).
        let flags = unsafe { libc::fcntl(pty.master.as_raw_fd(), libc::F_GETFL) };
        assert_eq!(
            unsafe {
                libc::fcntl(
                    pty.master.as_raw_fd(),
                    libc::F_SETFL,
                    flags | libc::O_NONBLOCK,
                )
            },
            0
        );
        let mut buf = [0u8; 32];
        match (&pty.master).read(&mut buf) {
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
            other => panic!("an exclusive open must write nothing, got {other:?}"),
        }
        interface.shutdown();
    }

    /// An exclusive port ignores its uuid: it writes **nothing** on the wire —
    /// not the SET_NODEID unit, not an assignment — whatever identity the
    /// section carries.
    #[tokio::test]
    async fn an_exclusive_serial_open_ignores_its_uuid() {
        let pty = Pty::new();
        let mut section = section_with("serial", pty.path.to_str().unwrap());
        section.parameters.insert(
            "uuid".to_string(),
            ConfigValue::Single("11aa22bb33cc".to_string()),
        );
        let mut config = McuConfig::new(&wrap(&section)).expect("a serial+uuid section parses");
        assert!(matches!(
            config.transport,
            Transport::Serial { uuid: Some(_), .. }
        ));
        config.id = Some(7);
        config.shared_baud = None;

        let interface = config.open().expect("an exclusive port opens");

        let flags = unsafe { libc::fcntl(pty.master.as_raw_fd(), libc::F_GETFL) };
        assert_eq!(
            unsafe {
                libc::fcntl(
                    pty.master.as_raw_fd(),
                    libc::F_SETFL,
                    flags | libc::O_NONBLOCK,
                )
            },
            0
        );
        let mut buf = [0u8; 32];
        match (&pty.master).read(&mut buf) {
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
            other => panic!("an exclusive open with a uuid must write nothing, got {other:?}"),
        }
        interface.shutdown();
    }

    /// On a shared port `restart_method: cheetah` is ignored: `open` succeeds
    /// rather than trying to drive RTS on a line several MCUs share.
    #[tokio::test]
    async fn test_a_shared_serial_ignores_the_cheetah_rts() {
        let pty = Pty::new();
        let mut section = section_with("serial", pty.path.to_str().unwrap());
        section.parameters.insert(
            "restart_method".to_string(),
            ConfigValue::Single("cheetah".to_string()),
        );
        let mut config = McuConfig::new(&wrap(&section)).expect("a serial section parses");
        // `cheetah` asks for RTS deasserted on an exclusive port; a pty rejects
        // the ioctl, so a shared open that called it would fail here.
        assert_eq!(config.restart_method, McuRestartMethod::Cheetah);
        config.id = Some(3);
        config.shared_baud = Some(DEFAULT_BAUD);

        let interface = open_answering_assignment(config, &pty, 3)
            .await
            .expect("a shared attach succeeds and ignores the cheetah RTS");
        interface.shutdown();
    }

    // -----------------------------------------------------------------------
    // A shared `serial:` path pins `restart_method` to `command`.
    // -----------------------------------------------------------------------

    /// Parse `text`'s `[identifier]` section the way the loader does — with the
    /// whole config on the wrapper — and return the result with the warnings
    /// that came with it.
    ///
    /// The loader's wrapper carries the whole config (`load.rs`), which is what
    /// lets the shared-path rule see the sibling `[mcu]` sections; a bare
    /// `wrap` would answer `false` (`mcu/serial_ports.rs`).
    fn parse_shared(text: &str, identifier: &str) -> (Result<McuConfig, ConfigError>, String) {
        let capture = LogCapture::default();
        let subscriber = tracing_subscriber::registry()
            .with(tracing_subscriber::filter::LevelFilter::WARN)
            .with(
                tracing_subscriber::fmt::layer()
                    .with_writer(capture.clone())
                    .with_ansi(false),
            );
        let result = tracing::subscriber::with_default(subscriber, || {
            let (config, _) = Config::from_text(text).expect("the test config parses");
            let section = config
                .get_section(identifier)
                .unwrap_or_else(|| panic!("the config has [{identifier}]"));
            let wrapper =
                ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
            McuConfig::new(&wrapper)
        });
        (result, capture.text())
    }

    /// A byte sink for the fmt layer above. `with_default` keeps the capture off
    /// the process-wide subscriber, the same shape `serial_ports.rs` uses.
    #[derive(Clone, Default)]
    struct LogCapture(Arc<Mutex<Vec<u8>>>);

    impl LogCapture {
        fn text(&self) -> String {
            let bytes = self
                .0
                .lock()
                .expect("the capture lock is not poisoned")
                .clone();
            String::from_utf8(bytes).expect("tracing writes UTF-8")
        }
    }

    impl Write for LogCapture {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.0
                .lock()
                .expect("the capture lock is not poisoned")
                .extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    impl<'a> tracing_subscriber::fmt::MakeWriter<'a> for LogCapture {
        type Writer = LogCapture;

        fn make_writer(&'a self) -> Self::Writer {
            self.clone()
        }
    }

    /// A shared path with `restart_method: arduino` written resets with
    /// `command` and says so: the DTR toggle acts on the whole line.
    #[test]
    fn a_shared_serial_forces_arduino_to_command_with_a_warning() {
        let (config, warnings) = parse_shared(
            "[mcu]\nserial: /dev/ttyACM0\nrestart_method: arduino\n\
             [mcu tool]\nserial: /dev/ttyACM0\n",
            "mcu",
        );
        assert_eq!(config.unwrap().restart_method, McuRestartMethod::Command);
        assert!(
            warnings.contains("MCU 'mcu'"),
            "the MCU is named: {warnings}"
        );
        assert!(
            warnings.contains("arduino"),
            "the written value appears: {warnings}"
        );
        assert!(
            warnings.contains("shared"),
            "the sharing is stated: {warnings}"
        );
        assert!(
            warnings.contains("command"),
            "the fallback appears: {warnings}"
        );
    }

    /// The Cheetah RTS sequence is line-wide too, so a shared path drops it just
    /// the same and reports the written value.
    #[test]
    fn a_shared_serial_forces_cheetah_to_command_with_a_warning() {
        let (config, warnings) = parse_shared(
            "[mcu]\nserial: /dev/ttyACM0\nrestart_method: cheetah\n\
             [mcu tool]\nserial: /dev/ttyACM0\n",
            "mcu",
        );
        assert_eq!(config.unwrap().restart_method, McuRestartMethod::Command);
        assert!(warnings.contains("cheetah"), "{warnings}");
        assert!(warnings.contains("command"), "{warnings}");
    }

    /// `rpi_usb` cuts the whole port's power, so a shared path drops it too; the
    /// `usb_power` option that only means something with `rpi_usb` is still
    /// reported rather than swallowed by the downgrade.
    #[test]
    fn a_shared_serial_forces_rpi_usb_to_command_and_still_reports_usb_power() {
        let (config, warnings) = parse_shared(
            "[mcu]\nserial: /dev/ttyACM0\nrestart_method: rpi_usb\nusb_power: sysfs\n\
             [mcu tool]\nserial: /dev/ttyACM0\n",
            "mcu",
        );
        let config = config.unwrap();
        assert_eq!(config.restart_method, McuRestartMethod::Command);
        assert_eq!(config.usb_power, UsbPowerMethod::Sysfs);
        assert!(warnings.contains("rpi_usb"), "{warnings}");
        assert!(warnings.contains("command"), "{warnings}");
        assert!(
            warnings.contains("usb_power"),
            "the usb_power report must survive the downgrade: {warnings}"
        );
    }

    /// Nothing written is not a disagreement to report: the serial default is
    /// `arduino`, and it is downgraded silently.
    #[test]
    fn a_shared_serial_without_restart_method_is_silently_command() {
        let (config, warnings) = parse_shared(
            "[mcu]\nserial: /dev/ttyACM0\n[mcu tool]\nserial: /dev/ttyACM0\n",
            "mcu",
        );
        assert_eq!(config.unwrap().restart_method, McuRestartMethod::Command);
        assert!(
            warnings.is_empty(),
            "an unset restart_method is not warned about: {warnings}"
        );
    }

    /// A written `command` is already the shared rule, so there is nothing to
    /// warn about.
    #[test]
    fn a_shared_serial_written_command_needs_no_warning() {
        let (config, warnings) = parse_shared(
            "[mcu]\nserial: /dev/ttyACM0\nrestart_method: command\n\
             [mcu tool]\nserial: /dev/ttyACM0\n",
            "mcu",
        );
        assert_eq!(config.unwrap().restart_method, McuRestartMethod::Command);
        assert!(
            warnings.is_empty(),
            "command needs no downgrade warning: {warnings}"
        );
    }

    /// An invalid value is a config error on a shared path too: the rule bends
    /// the method, not the validation.
    #[test]
    fn a_shared_serial_still_rejects_an_invalid_restart_method() {
        let (config, _) = parse_shared(
            "[mcu]\nserial: /dev/ttyACM0\nrestart_method: fast\n\
             [mcu tool]\nserial: /dev/ttyACM0\n",
            "mcu",
        );
        let err = config.unwrap_err();
        assert!(err.to_string().contains("invalid restart_method"), "{err}");
        assert!(err.to_string().contains("fast"), "{err}");
    }

    /// An exclusive serial path is untouched: the written method is the method.
    #[test]
    fn an_exclusive_serial_keeps_its_written_restart_method() {
        let (config, warnings) = parse_shared(
            "[mcu]\nserial: /dev/ttyACM0\nrestart_method: arduino\n",
            "mcu",
        );
        assert_eq!(config.unwrap().restart_method, McuRestartMethod::Arduino);
        assert!(
            warnings.is_empty(),
            "an exclusive path reads the option as written: {warnings}"
        );
    }

    // -----------------------------------------------------------------------
    // A `host_library:` path a mux addresses (`mcu/host_libraries.rs`).
    // -----------------------------------------------------------------------

    /// Two host-library MCUs on one path with no uuid cannot be told apart, so
    /// the section's own parse reports the conflict — the config fails to load.
    #[test]
    fn a_host_library_path_two_mcus_share_without_a_uuid_is_rejected() {
        let (config, _) = parse_shared(
            "[mcu]\nhost_library: /lib/libklipper_host.so\n\
             [mcu tool]\nhost_library: /lib/libklipper_host.so\n",
            "mcu",
        );
        let err = config.unwrap_err();
        assert!(err.to_string().contains("host library"), "{err}");
        assert!(err.to_string().contains("'mcu tool'"), "{err}");
    }

    /// Two host-library MCUs writing the same uuid name one board twice: also a
    /// config error, reported through the same parse.
    #[test]
    fn a_host_library_path_two_mcus_share_with_one_uuid_is_rejected() {
        let (config, _) = parse_shared(
            "[mcu a]\nhost_library: /lib/libklipper_host.so\nuuid: 11aa22bb33cc\n\
             [mcu b]\nhost_library: /lib/libklipper_host.so\nuuid: 11aa22bb33cc\n",
            "mcu a",
        );
        let err = config.unwrap_err();
        assert!(err.to_string().contains("'mcu a'"), "{err}");
        assert!(err.to_string().contains("'mcu b'"), "{err}");
    }

    /// Two host-library sections on one path, one of which omits the uuid, are a
    /// shape a mux can address: the config parses.
    #[test]
    fn a_host_library_path_a_mux_can_address_parses() {
        let (config, _) = parse_shared(
            "[mcu]\nhost_library: /lib/libklipper_host.so\nuuid: 11aa22bb33cc\n\
             [mcu tool]\nhost_library: /lib/libklipper_host.so\n",
            "mcu",
        );
        assert!(config.is_ok(), "{config:?}");
    }

    /// Two `[mcu]` sections naming two **different** `host_library:` paths are a
    /// shape that parses: each path is a separate image, and klipper runs both
    /// in one process.
    #[test]
    fn a_config_with_two_host_library_paths_parses() {
        let (config, _) = parse_shared(
            "[mcu]\nhost_library: /lib/one.so\n\
             [mcu tool]\nhost_library: /lib/two.so\n",
            "mcu",
        );
        assert!(config.is_ok(), "{config:?}");
    }

    // -----------------------------------------------------------------------
    // Bridge downstream ports (`uart:` / `can:`, M1 P1-C).
    // -----------------------------------------------------------------------

    /// A `uart:` section parses to the bridge-port transport, carrying the
    /// port section identifier, the port kind and the uuid when written. A
    /// port child is not a serial MCU, so it resets with `command`.
    #[test]
    fn a_uart_reference_becomes_the_bridge_port_transport() {
        let (config, _) = parse_shared(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\nuart: uart0\nuuid: 11aa22bb33cc\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
            "mcu child",
        );
        let config = config.unwrap();
        assert!(matches!(
            &config.transport,
            Transport::BridgePort {
                port,
                uuid: Some(u),
                kind: PortKind::Uart,
            } if port == "uart_device uart0"
                && u == &[0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc]
        ));
        assert_eq!(config.restart_method, McuRestartMethod::Command);
    }

    /// A `can:` section likewise; its uuid is the identity the CAN port
    /// addresses the board by.
    #[test]
    fn a_can_reference_becomes_the_bridge_port_transport() {
        let (config, _) = parse_shared(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\ncan: can0\nuuid: 44dd55ee66ff\n\
             [can_device can0]\ncan_mcu: bridge\ncan_oid: can0\n",
            "mcu child",
        );
        let config = config.unwrap();
        assert!(matches!(
            &config.transport,
            Transport::BridgePort {
                port,
                uuid: Some(u),
                kind: PortKind::Can,
            } if port == "can_device can0"
                && u == &[0x44, 0xdd, 0x55, 0xee, 0x66, 0xff]
        ));
    }

    /// A uart port child with no `uuid` is rule 5's legacy shape: it parses,
    /// carrying no identity.
    #[test]
    fn a_uart_reference_without_a_uuid_parses_as_legacy() {
        let (config, _) = parse_shared(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\nuart: uart0\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
            "mcu child",
        );
        let config = config.unwrap();
        assert!(matches!(
            &config.transport,
            Transport::BridgePort {
                uuid: None,
                kind: PortKind::Uart,
                ..
            }
        ));
    }

    /// Every other interface key with `uart:` names two transports, listed in
    /// the order the keys are declared — the same refusal every selector has.
    #[test]
    fn a_uart_reference_with_another_interface_is_rejected() {
        let base = "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\nuart: uart0\n{extra}\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n";
        for (extra, listed) in [
            ("serial: /dev/b", "serial, uart"),
            (
                "host_library: /lib/libklipper_host.so",
                "host_library, uart",
            ),
            ("can: other", "uart, can"),
            ("test: dict=/fake.dict", "uart, test"),
        ] {
            let (config, _) = parse_shared(&base.replace("{extra}", extra), "mcu child");
            let err = config.unwrap_err();
            assert!(
                err.to_string().contains("more than one interface"),
                "{extra}: {err}"
            );
            assert!(err.to_string().contains(listed), "{extra}: {err}");
        }
    }

    /// `canbus_nodeid` is an id override, not a selector — but no non-CAN
    /// transport has ever accepted it, so a uart port's member refuses it the
    /// way `serial:` does.
    #[test]
    fn a_uart_reference_with_canbus_nodeid_is_rejected() {
        let (config, _) = parse_shared(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\nuart: uart0\ncanbus_nodeid: 3\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
            "mcu child",
        );
        let err = config.unwrap_err();
        assert!(err.to_string().contains("more than one interface"), "{err}");
        assert!(err.to_string().contains("uart, canbus_nodeid"), "{err}");
    }

    /// A CAN port's member is a CAN device: `canbus_nodeid` overrides its id
    /// the way it does on a direct CAN link (the override is orthogonal to
    /// the interface), and its range is validated the same.
    #[test]
    fn a_can_reference_takes_canbus_nodeid_as_its_id() {
        let (config, _) = parse_shared(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\ncan: can0\nuuid: 44dd55ee66ff\ncanbus_nodeid: 7\n\
             [can_device can0]\ncan_mcu: bridge\ncan_oid: can0\n",
            "mcu child",
        );
        assert_eq!(config.unwrap().id, Some(7));

        let (config, _) = parse_shared(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\ncan: can0\nuuid: 44dd55ee66ff\ncanbus_nodeid: 0\n\
             [can_device can0]\ncan_mcu: bridge\ncan_oid: can0\n",
            "mcu child",
        );
        let err = config.unwrap_err();
        assert!(err.to_string().contains("invalid canbus_nodeid"), "{err}");
    }

    /// A `uart:` naming no `[uart_device]` section cannot open anything: the
    /// error names the child section and the port section it asked for.
    #[test]
    fn a_reference_to_no_port_section_is_refused() {
        let (config, _) = parse_shared(
            "[mcu bridge]\nserial: /dev/a\n[mcu child]\nuart: nowhere\n",
            "mcu child",
        );
        let err = config.unwrap_err();
        assert!(err.to_string().contains("'mcu child'"), "{err}");
        assert!(err.to_string().contains("[uart_device nowhere]"), "{err}");
    }

    /// A `uart:` naming a port declared `[can_device]` is a type mismatch, not
    /// a second port: the error names what the config is missing.
    #[test]
    fn a_reference_to_the_other_port_kind_is_refused() {
        let (config, _) = parse_shared(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\nuart: can0\n\
             [can_device can0]\ncan_mcu: bridge\ncan_oid: can0\n",
            "mcu child",
        );
        let err = config.unwrap_err();
        assert!(err.to_string().contains("'mcu child'"), "{err}");
        assert!(
            err.to_string().contains("no [uart_device can0] section"),
            "{err}"
        );
    }

    /// A CAN port's member with no `uuid` cannot be addressed (rule 6); the
    /// refusal is the port grouping's alone — its wording names the port — so
    /// the transport parse adds no second report.
    #[test]
    fn a_can_reference_without_a_uuid_is_the_grouping_to_refuse() {
        let (config, _) = parse_shared(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\ncan: can0\n\
             [can_device can0]\ncan_mcu: bridge\ncan_oid: can0\n",
            "mcu child",
        );
        let err = config.unwrap_err();
        assert!(err.to_string().contains("can_device can0"), "{err}");
        assert!(err.to_string().contains("'mcu child'"), "{err}");
        assert!(err.to_string().contains("by uuid"), "{err}");
    }

    /// Both spellings of the identity on a port child are one identity written
    /// twice, exactly as on every other transport.
    #[test]
    fn a_port_reference_with_both_uuid_spellings_is_rejected() {
        let (config, _) = parse_shared(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\nuart: uart0\nuuid: 11aa22bb33cc\ncanbus_uuid: 44dd55ee66ff\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
            "mcu child",
        );
        let err = config.unwrap_err();
        assert!(
            err.to_string().contains("both uuid and canbus_uuid"),
            "{err}"
        );
    }

    /// A malformed uuid on a port child keeps the CAN error's wording.
    #[test]
    fn a_port_reference_with_an_invalid_uuid_is_rejected() {
        let (config, _) = parse_shared(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\nuart: uart0\nuuid: bogus\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
            "mcu child",
        );
        let err = config.unwrap_err();
        assert!(err.to_string().contains("invalid uuid"), "{err}");
    }

    /// The transport describes the port, but `Transport::open` is not its path:
    /// the attach through the bridge's port table (`mcu/bridge_attach.rs`)
    /// opens it, and `open` points there rather than pretending the device came
    /// up.
    #[test]
    fn a_port_child_open_points_at_the_bridge_attach() {
        let (config, _) = parse_shared(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\nuart: uart0\nuuid: 11aa22bb33cc\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
            "mcu child",
        );
        let config = config.unwrap();
        let err = config.open().unwrap_err();
        assert!(err.contains("uart_device uart0"), "{err}");
        assert!(err.contains("bridge_attach"), "{err}");
    }

    /// A `restart_method` on a port child is reported and ignored: only a
    /// serial MCU resets by line, and this transport is not one.
    #[test]
    fn a_port_child_resets_with_command_and_reports_the_option() {
        let (config, warnings) = parse_shared(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\nuart: uart0\nrestart_method: arduino\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
            "mcu child",
        );
        assert_eq!(config.unwrap().restart_method, McuRestartMethod::Command);
        assert!(
            warnings.contains("only applies to a serial MCU"),
            "{warnings}"
        );
    }

    /// The end-to-end load contract: a legacy child on a legal `[uart_device]`
    /// loads clean through the whole walk — every option read, nothing left
    /// for `check_unused` — and the port grouping the child's load built is
    /// registered once, reporting the port as legacy.
    #[test]
    fn a_legacy_port_child_loads_clean() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(
            "[mcu bridge]\nserial: /dev/not-opened-yet\n\
             [mcu child]\nuart: uart0\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
        )
        .expect("the test config parses");

        printer.load_config(&config).expect("the config loads");

        let groups = printer
            .lookup_object_as::<port_grouping::PortGroups>(port_grouping::PORT_GROUPS_OBJECT)
            .expect("the port grouping registered");
        assert_eq!(groups.peers("uart_device uart0"), ["mcu child"]);
        assert!(groups.is_legacy("uart_device uart0"));
    }

    /// A shared port whose members cannot be told apart fails the whole load
    /// at the referencing section's parse, naming the port and the uuid-less
    /// member — one report for one error.
    #[test]
    fn a_shared_port_missing_a_uuid_fails_the_load() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu one]\nuart: uart0\nuuid: 11aa22bb33cc\n\
             [mcu two]\nuart: uart0\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n",
        )
        .expect("the test config parses");

        let err = printer.load_config(&config).unwrap_err();
        assert!(err.to_string().contains("uart_device uart0"), "{err}");
        assert!(err.to_string().contains("'mcu two'"), "{err}");
    }

    /// A CAN port's member registers its uuid the way a direct CAN link does:
    /// two sections naming one identity are one board claimed twice, and the
    /// load says so — proof the port child took the `register_can` arm.
    #[test]
    fn a_can_port_child_registers_its_uuid() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu direct]\nuuid: 11aa22bb33cc\n\
             [mcu child]\ncan: can0\nuuid: 11aa22bb33cc\n\
             [can_device can0]\ncan_mcu: bridge\ncan_oid: can0\n",
        )
        .expect("the test config parses");

        let err = printer.load_config(&config).unwrap_err();
        assert!(err.to_string().contains("Duplicate uuid"), "{err}");
    }

    /// The explicit id override rides a CAN port's member the way it does a
    /// direct CAN link: it is reserved at load, and a second claim on it is
    /// refused — proof the override is consumed, consistently with the
    /// conflict decision that allows it on `can:` only.
    #[test]
    fn a_can_port_child_honours_its_explicit_nodeid() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu other]\nuuid: 22bb33cc44dd\ncanbus_nodeid: 3\n\
             [mcu child]\ncan: can0\nuuid: 11aa22bb33cc\ncanbus_nodeid: 3\n\
             [can_device can0]\ncan_mcu: bridge\ncan_oid: can0\n",
        )
        .expect("the test config parses");

        let err = printer.load_config(&config).unwrap_err();
        assert!(err.to_string().contains("canbus_nodeid"), "{err}");
        // The allocator names MCUs by their section sub, not identifier.
        assert!(err.to_string().contains("MCU 'child'"), "{err}");
    }

    // -----------------------------------------------------------------------
    // The bridge-port reset face: `restart_method: admin` for a named device,
    // the port's `uart_restart_method` for a legacy one.
    // -----------------------------------------------------------------------

    /// Parse the `[<identifier>]` section of `text` with the whole config behind
    /// the wrapper, the way the loader hands a factory one — a bridge-port parse
    /// looks its port section up as a sibling.
    fn parse_section(text: &str, identifier: &str) -> Result<McuConfig, ConfigError> {
        let (config, _) = Config::from_text(text).expect("the test config parses");
        let section = config
            .get_section(identifier)
            .unwrap_or_else(|| panic!("the config has [{identifier}]"));
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        McuConfig::new(&wrapper)
    }

    /// A child on a uart port, with `restart_method` and the port's
    /// `uart_restart_method` as written.
    fn bridge_child(child_options: &str, port_options: &str) -> String {
        format!(
            "[mcu bridge]\nserial: /dev/a\n\
             [mcu child]\nuart: uart0\n{child_options}\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\n{port_options}"
        )
    }

    /// A named (uuid-bearing) bridge-port device takes `restart_method: admin`:
    /// it answers the bridge's mux itself, so it has the admin channel the
    /// command needs, and the bridge forwards the restart to its port.
    #[test]
    fn a_named_bridge_port_child_takes_admin() {
        let config = parse_section(
            &bridge_child("uuid: 11aa22bb33cc\nrestart_method: admin\n", ""),
            "mcu child",
        )
        .expect("a named bridge-port child with admin parses");

        assert_eq!(config.restart_method, McuRestartMethod::Admin);
        assert_eq!(config.bridge_restart, Some(BridgeRestart::Firmware));
    }

    /// `admin` on a legacy (uuid-less) child is refused: there is no admin
    /// channel to forward the command to, and the error names the rule.
    #[test]
    fn admin_on_a_legacy_bridge_port_child_is_refused() {
        let err =
            parse_section(&bridge_child("restart_method: admin\n", ""), "mcu child").unwrap_err();

        let text = err.to_string();
        assert!(
            text.contains("restart_method 'admin' needs an admin channel"),
            "{text}"
        );
        assert!(text.contains("or a bridge port with a 'uuid:'"), "{text}");
    }

    /// `admin` on a serial MCU is the bridge restarting **itself**: its mux admin
    /// channel is reached on this very link, so the value is kept rather than
    /// refused — and no port-level reset is asked for, since the device in
    /// question is not behind a port.
    #[test]
    fn admin_on_a_serial_mcu_is_the_bridge_restarting_itself() {
        let config = parse_section("[mcu]\nserial: /dev/a\nrestart_method: admin\n", "mcu")
            .expect("a serial MCU with admin parses");

        assert_eq!(config.restart_method, McuRestartMethod::Admin);
        assert_eq!(config.bridge_restart, None);
    }

    /// A **shared** serial path keeps `admin`: the frame goes to the mux's admin
    /// id, not to the line's modem lines, so no other MCU on the port is
    /// interrupted the way a DTR toggle would interrupt it.
    #[test]
    fn admin_on_a_shared_serial_path_is_kept() {
        let config = parse_section(
            "[mcu bridge]\nserial: /dev/a\nrestart_method: admin\n[mcu board]\nserial: /dev/a\n",
            "mcu bridge",
        )
        .expect("a shared serial path with admin parses");

        assert_eq!(config.restart_method, McuRestartMethod::Admin);
        assert_eq!(config.bridge_restart, None);
    }

    /// `admin` on a non-bridge transport is refused even when the section does
    /// write a `uuid:` — a CAN link has no admin channel and no bridge port to
    /// forward on.
    #[test]
    fn admin_on_a_can_mcu_is_refused() {
        let err =
            parse_section("[mcu]\nuuid: 11aa22bb33cc\nrestart_method: admin\n", "mcu").unwrap_err();

        assert!(
            err.to_string()
                .contains("restart_method 'admin' needs an admin channel"),
            "{err}"
        );
    }

    /// A named child that writes anything but `admin` resets with `command`: the
    /// line-level methods drive a port the child does not own, and `command` is
    /// the one value it already gets. No port-level reset is asked for either.
    #[test]
    fn a_named_child_with_an_inapplicable_method_resets_with_command() {
        for value in ["arduino", "cheetah", "rpi_usb", "command"] {
            let config = parse_section(
                &bridge_child(
                    &format!("uuid: 11aa22bb33cc\nrestart_method: {value}\n"),
                    "",
                ),
                "mcu child",
            )
            .unwrap_or_else(|err| panic!("{value}: {err}"));

            assert_eq!(config.restart_method, McuRestartMethod::Command, "{value}");
            assert_eq!(config.bridge_restart, None, "{value}");
        }
    }

    /// An unknown spelling is a config error on a bridge-port section too, the
    /// way it is on a serial one — an unreadable value is never silently the
    /// default.
    #[test]
    fn an_unknown_restart_method_on_a_bridge_child_is_a_config_error() {
        let err = parse_section(
            &bridge_child("uuid: 11aa22bb33cc\nrestart_method: wibble\n", ""),
            "mcu child",
        )
        .unwrap_err();

        assert!(err.to_string().contains("invalid restart_method"), "{err}");
        assert!(err.to_string().contains("wibble"), "{err}");
    }

    /// A legacy child takes no method of its own: what resets it is the port's
    /// `uart_restart_method`, and a line-level one becomes `BridgeRestart::Port`.
    #[test]
    fn a_legacy_child_takes_its_ports_line_reset() {
        let config = parse_section(
            &bridge_child("", "uart_restart_method: arduino\n"),
            "mcu child",
        )
        .expect("a legacy child on a declared port parses");

        assert_eq!(config.restart_method, McuRestartMethod::Command);
        assert_eq!(config.bridge_restart, Some(BridgeRestart::Port));
    }

    /// A port that declares nothing leaves the legacy child with no port-level
    /// reset: today's behaviour, where nothing acts before its transport opens.
    #[test]
    fn a_legacy_child_on_an_undeclared_port_gets_no_reset() {
        let config = parse_section(&bridge_child("", ""), "mcu child")
            .expect("a legacy child on an undeclared port parses");

        assert_eq!(config.restart_method, McuRestartMethod::Command);
        assert_eq!(config.bridge_restart, None);
    }
}
