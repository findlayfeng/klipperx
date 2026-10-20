pub mod devices;
pub mod error;
#[cfg(test)]
pub(crate) mod pty;
pub mod usb;

pub use devices::canserial::{CanSerialDevice, CanWire};
#[cfg(test)]
pub use devices::frame_mock::{FrameMock, MappingEntry};
pub use devices::host::{HostDevice, HostMux};
pub use devices::serial::SerialDevice;
use devices::serial::SharedSerialHandle;
#[cfg(test)]
pub use devices::simulator::SimulatorDevice;
pub use error::InterfaceError;

use super::frame::{Frame, MESSAGE_HEADER_SIZE, MESSAGE_MAX, MESSAGE_MIN, MESSAGE_TRAILER_SIZE};
use std::path::Path;
use std::sync::Arc;

pub trait Device: Send + Sync {
    fn send(&self, frame: &Frame) -> Result<(), InterfaceError>;
    /// Block until a frame is received, or return `None` once the device has
    /// been shut down (no further frames will ever arrive).
    fn receive(&self) -> Option<Frame>;
    /// Unblock any in-flight [`Device::receive`] and prevent further receives.
    ///
    /// A synchronous device read runs inside `spawn_blocking`, which cannot be
    /// cancelled by aborting the async task that awaits it. Calling this on
    /// shutdown releases that blocked thread so the runtime can exit.
    fn shutdown(&self);
}

/// Interface for communicating with a Klipper device.
///
/// An open transport plus the runtime that transport does its blocking I/O on.
/// The handle is captured when the transport is opened and **stored**, rather
/// than asked for at each spawn (`tokio::task::spawn_blocking` uses the ambient
/// runtime). The device's `send`/`receive` are blocking, so which runtime they
/// land on is part of what an interface *is*. It is the machine runtime; a
/// future split of the machine and API runtimes (TODO A3) changes only where it
/// is captured.
///
/// The transport itself is private: which kind a section asks for is decided by
/// `[mcu]` parsing, so nothing outside this module needs to match on it.
#[derive(Debug, Clone)]
pub struct Interface {
    handle: tokio::runtime::Handle,
    transport: Transport,
}

/// The device behind an [`Interface`], one variant per transport a `[mcu]`
/// section can ask for:
/// - `Serial(SerialDevice)` — a real MCU on a tty (`serial:`)
/// - `SharedSerial(SharedSerialHandle)` — one MCU's handle on a tty two or more
///   MCUs share (`serial:` named by several `[mcu]` sections)
/// - `CanSerial(CanSerialDevice)` — a real MCU reached over CAN, using Klipper's
///   can-serial link (`uuid:` + `canbus_interface:`; `canbus_nodeid:` is
///   the Klipper-named key that explicitly overrides the MCU's layer id)
/// - `Host(HostDevice)` — klipper's host library, loaded from a shared object
///   (`host_library:`)
/// - `FrameMock(FrameMock)` — a frame-level mock for tests (built in code)
/// - `Simulator(SimulatorDevice)` — a dictionary-driven fake MCU, in test builds
///   (`test: dict=`)
///
/// There is no "configured nothing" variant on purpose: a section that names no
/// transport is reported when it is parsed, rather than turned into an interface
/// that fails on every call.
#[derive(Debug, Clone)]
enum Transport {
    Serial(Arc<SerialDevice>),
    /// One MCU's handle on a port several share: a single reader splits the byte
    /// stream by arbitration id, and `shutdown` stops only this handle.
    SharedSerial(Arc<SharedSerialHandle>),
    CanSerial(Arc<CanSerialDevice>),
    Host(Arc<HostDevice>),
    #[cfg(test)]
    FrameMock(Arc<FrameMock>),
    #[cfg(test)]
    Simulator(Arc<SimulatorDevice>),
    /// A recording wire for transport-timing tests (`Interface::recording`).
    #[cfg(test)]
    Recording(Arc<devices::frame_mock::RecordingWire>),
}

impl Interface {
    /// Wrap `transport`, picking up the runtime its I/O will run on.
    ///
    /// This is the one place the machine's runtime enters a transport, and it
    /// is called *after* the device is open — so a transport that fails to open
    /// never needs a runtime at all (the config tests rely on that).
    fn with_transport(transport: Transport) -> Self {
        Self {
            handle: tokio::runtime::Handle::current(),
            transport,
        }
    }

    /// The runtime this interface's blocking I/O runs on.
    ///
    /// [`Mcu`](crate::core::klippy::mcu::Mcu) uses it for its transport tasks,
    /// so both halves of a connection share one runtime.
    pub fn handle(&self) -> &tokio::runtime::Handle {
        &self.handle
    }

    /// Create a new `Interface` wrapping a frame-level mock.
    #[cfg(test)]
    pub fn new(device: FrameMock) -> Self {
        Self::with_transport(Transport::FrameMock(Arc::new(device)))
    }

    /// Create an interface over a dictionary-driven fake MCU.
    #[cfg(test)]
    pub fn simulator(device: SimulatorDevice) -> Self {
        Self::with_transport(Transport::Simulator(Arc::new(device)))
    }

    /// The dictionary-driven fake MCU this interface is wired to, when it is
    /// one (`test: dict=`).
    ///
    /// The test-side handle the other way round: a multi-MCU test needs the
    /// *instance* behind each `[mcu …]` section — to prove each board answers
    /// its own dictionary, and to link instances into one machine — not just
    /// the connection that talks to it.
    #[cfg(test)]
    pub(crate) fn simulator_device(&self) -> Option<Arc<SimulatorDevice>> {
        match &self.transport {
            Transport::Simulator(device) => Some(Arc::clone(device)),
            _ => None,
        }
    }

    /// Create an interface over a recording wire (`RecordingWire`): records
    /// every frame sent, acks, and answers scripted requests — for tests that
    /// assert *when* a message goes out without predicting its derived bytes.
    #[cfg(test)]
    pub fn recording(device: devices::frame_mock::RecordingWire) -> Self {
        Self::with_transport(Transport::Recording(Arc::new(device)))
    }

    /// Create an interface for a real MCU on the serial port `path`.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] if the port cannot be opened or put into raw
    /// mode at `baud`.
    pub fn serial(path: impl AsRef<std::path::Path>, baud: u32) -> Result<Self, InterfaceError> {
        Ok(Self::from_serial(SerialDevice::open(path, baud)?))
    }

    /// Wrap an already-open serial device.
    ///
    /// For a caller that opened the port itself: `[mcu]` leaves RTS in a state
    /// that depends on `restart_method`, so it opens the device before handing
    /// it over (`config/mcu.rs`).
    pub fn from_serial(device: SerialDevice) -> Self {
        Self::with_transport(Transport::Serial(Arc::new(device)))
    }

    /// Wrap an already-attached handle on a shared serial port.
    ///
    /// For the second and later MCU on one `serial:` path: the port is opened
    /// once and the id handshake already ran
    /// (`interface::devices::serial`), so this only installs the handle.
    pub fn shared_serial(handle: Arc<SharedSerialHandle>) -> Self {
        Self::with_transport(Transport::SharedSerial(handle))
    }

    /// Create an interface for the MCU `uuid` on the CAN interface `name`, brought
    /// up as node `nodeid` — the classical CAN link.
    ///
    /// This is [`Interface::canserial_with_wire`] on [`CanWire::Classical`].
    ///
    /// # Errors
    /// Returns [`InterfaceError`] if the interface does not exist or the socket
    /// cannot be set up.
    pub fn canserial(name: &str, uuid: [u8; 6], nodeid: u32) -> Result<Self, InterfaceError> {
        Self::canserial_with_wire(name, uuid, nodeid, CanWire::Classical)
    }

    /// The same on `wire`: a CAN FD link carries sixty-four data bytes per CAN
    /// frame, and its socket reads and writes seventy-two byte datagrams.
    ///
    /// The wire is not a preference but a pairing with the other end — a socket
    /// that has not asked the kernel for CAN FD frames hears nothing on an FD
    /// bus — so a link says which wire it is when it opens.
    ///
    /// # Errors
    /// The same as [`Interface::canserial`], plus a socket that cannot be given
    /// CAN FD frames.
    pub fn canserial_with_wire(
        name: &str,
        uuid: [u8; 6],
        nodeid: u32,
        wire: CanWire,
    ) -> Result<Self, InterfaceError> {
        Ok(Self::with_transport(Transport::CanSerial(Arc::new(
            CanSerialDevice::open_with_wire(name, uuid, nodeid, wire)?,
        ))))
    }

    /// Create an interface running klipper's host library from `path`.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] if the library cannot be loaded, does not
    /// export the expected symbols, or fails to initialize.
    pub fn host(path: impl AsRef<Path>) -> Result<Self, InterfaceError> {
        Ok(Self::with_transport(Transport::Host(Arc::new(
            HostDevice::load(path)?,
        ))))
    }

    /// Wrap an already-loaded host-library reference, for a device **behind a
    /// bridge port**.
    ///
    /// [`Interface::host`] loads the library; this is the case where the bridge
    /// already has it open and the reference is a downstream device
    /// ([`HostDevice::load_downstream`], `mcu/bridge_attach.rs`) rather than the
    /// bridge's own device.
    pub(crate) fn from_host(device: Arc<HostDevice>) -> Self {
        Self::with_transport(Transport::Host(device))
    }

    /// Tell the bridge this interface's link reaches which id the device behind
    /// port `index` answers to.
    ///
    /// The port-level command
    /// ([`set_port_node_id`](devices::serial::set_port_node_id)) travels the
    /// bridge's **admin** channel, so only a link that carries port mux units
    /// can carry it: a shared serial port, a muxed host library, or a
    /// can-serial CAN link (the node behind it is the bridge, and the channel is
    /// the mux unit inside the byte stream). Every other transport has no bridge
    /// to tell, and the message says so rather than writing the unit on a link
    /// nobody listens to.
    ///
    /// # Errors
    /// The device's own error, or [`InterfaceError::Other`] when this interface
    /// is not a bridge link.
    pub(crate) fn send_set_port_node_id(
        &self,
        index: u8,
        mcu_id: u8,
        flags: u8,
    ) -> Result<(), InterfaceError> {
        match &self.transport {
            Transport::SharedSerial(handle) => handle.send_set_port_node_id(index, mcu_id, flags),
            Transport::CanSerial(device) => device.send_set_port_node_id(index, mcu_id, flags),
            Transport::Host(device) => {
                if !device.is_mux() {
                    return Err(InterfaceError::Other(
                        "SET_PORT_NODEID needs a link that carries port mux units (a shared \
                         serial port, a muxed host library, or a can-serial link); this host \
                         library reference has no mux, so no bridge is listening"
                            .to_string(),
                    ));
                }
                device.send_set_port_node_id(index, mcu_id, flags)
            }
            _ => Err(InterfaceError::Other(
                "SET_PORT_NODEID needs a link that carries port mux units (a shared serial port, \
                 a muxed host library, or a can-serial link); this interface is not one"
                    .to_string(),
            )),
        }
    }

    /// Ask the device behind bridge port `index` to restart its firmware
    /// ([`CMD_RESTART_DEVICE`](devices::serial::CMD_RESTART_DEVICE)), the bridge
    /// forwarding the command to that port.
    ///
    /// The restart a **named** bridge-port device gets: it answers the bridge's
    /// mux itself, so it has an admin channel to receive this. Like
    /// `SET_PORT_NODEID` the command travels that channel, so only a link that
    /// carries port mux units does: a shared serial port, a muxed host library,
    /// or a can-serial CAN link.
    ///
    /// # Errors
    /// The device's own error, or [`InterfaceError::Other`] when this interface
    /// is not a bridge link.
    pub(crate) fn send_restart_device(&self, index: u8) -> Result<(), InterfaceError> {
        match &self.transport {
            Transport::SharedSerial(handle) => handle.send_restart_device(index),
            Transport::CanSerial(device) => device.send_restart_device(index),
            Transport::Host(device) => {
                if !device.is_mux() {
                    return Err(InterfaceError::Other(
                        "RESTART_DEVICE needs a link that carries port mux units (a shared \
                         serial port, a muxed host library, or a can-serial link); this host \
                         library reference has no mux, so no bridge is listening"
                            .to_string(),
                    ));
                }
                device.send_restart_device(index)
            }
            _ => Err(InterfaceError::Other(
                "RESTART_DEVICE needs a link that carries port mux units (a shared serial port, \
                 a muxed host library, or a can-serial link); this interface is not one"
                    .to_string(),
            )),
        }
    }

    /// Ask the bridge to pulse the lines of port `index`
    /// ([`CMD_RESET_PORT`](devices::serial::CMD_RESET_PORT)).
    ///
    /// The reset a **legacy** bridge-port device gets: it has no admin channel of
    /// its own, so its bridge's port is the only thing that can reset it. Like
    /// `SET_PORT_NODEID` the command travels the bridge's admin channel, so only
    /// a link that carries port mux units does: a shared serial port, a muxed
    /// host library, or a can-serial CAN link.
    ///
    /// # Errors
    /// The device's own error, or [`InterfaceError::Other`] when this interface
    /// is not a bridge link.
    pub(crate) fn send_reset_port(&self, index: u8) -> Result<(), InterfaceError> {
        match &self.transport {
            Transport::SharedSerial(handle) => handle.send_reset_port(index),
            Transport::CanSerial(device) => device.send_reset_port(index),
            Transport::Host(device) => {
                if !device.is_mux() {
                    return Err(InterfaceError::Other(
                        "RESET_PORT needs a link that carries port mux units (a shared serial \
                         port, a muxed host library, or a can-serial link); this host library \
                         reference has no mux, so no bridge is listening"
                            .to_string(),
                    ));
                }
                device.send_reset_port(index)
            }
            _ => Err(InterfaceError::Other(
                "RESET_PORT needs a link that carries port mux units (a shared serial port, a \
                 muxed host library, or a can-serial link); this interface is not one"
                    .to_string(),
            )),
        }
    }

    /// Ask the device behind bridge port `index` to enter its bootloader
    /// ([`CMD_REQUEST_BOOTLOADER`](devices::serial::CMD_REQUEST_BOOTLOADER)),
    /// the bridge forwarding the command to that port.
    ///
    /// The command a **flashing tool** addresses a device with: the device
    /// writes a bootloader-request marker and resets, so it comes back in
    /// katapult rather than in the firmware that was running. No receipt, and
    /// the device has to re-enumerate/restart before `CONNECT` reaches it, so a
    /// caller that uses this waits for the answer
    /// ([`BridgeLink::flash`](crate::flash::bridge_flash::BridgeLink::flash)).
    /// Like `SET_PORT_NODEID` the command travels the bridge's admin channel,
    /// so only a link that carries port mux units does: a shared serial port, a
    /// muxed host library, or a can-serial CAN link.
    ///
    /// # Errors
    /// The device's own error, or [`InterfaceError::Other`] when this interface
    /// is not a bridge link.
    ///
    /// `uuid` names the board the request is for. A CAN port turns a
    /// uuid-carrying request into the bus's single admin frame
    /// (`0x3f0 [0x02][uuid]`), which the device reads whatever carrier its own
    /// stream uses, and *also* sends the mux
    /// unit for a device that reads units. Without it only the unit travels.
    pub(crate) fn send_request_bootloader(
        &self,
        index: u8,
        uuid: Option<[u8; 6]>,
    ) -> Result<(), InterfaceError> {
        match &self.transport {
            Transport::SharedSerial(handle) => handle.send_request_bootloader(index, uuid),
            Transport::CanSerial(device) => device.send_request_bootloader(index, uuid),
            Transport::Host(device) => {
                if !device.is_mux() {
                    return Err(InterfaceError::Other(
                        "REQUEST_BOOTLOADER needs a link that carries port mux units (a shared \
                         serial port, a muxed host library, or a can-serial link); this host \
                         library reference has no mux, so no bridge is listening"
                            .to_string(),
                    ));
                }
                device.send_request_bootloader(index, uuid)
            }
            _ => Err(InterfaceError::Other(
                "REQUEST_BOOTLOADER needs a link that carries port mux units (a shared serial \
                 port, a muxed host library, or a can-serial link); this interface is not one"
                    .to_string(),
            )),
        }
    }

    /// Toggle the flashing tunnel of bridge port `index`
    /// ([`CMD_SET_PORT_TUNNEL`](devices::serial::CMD_SET_PORT_TUNNEL)).
    ///
    /// What a **flashing tool** turns on for the port a device in its bootloader
    /// sits behind: in tunnel mode the port carries a katapult raw byte stream
    /// rather than units. The bridge flips its own port state and consumes the
    /// command — nothing is forwarded downstream and there is no receipt — so a
    /// repeat flips the mode back and the caller tracks the state it asked for.
    /// Like `SET_PORT_NODEID` the command travels the bridge's admin channel, so
    /// only a link that carries port mux units does: a shared serial port, a
    /// muxed host library, or a can-serial CAN link.
    ///
    /// # Errors
    /// The device's own error, or [`InterfaceError::Other`] when this interface
    /// is not a bridge link.
    pub(crate) fn send_set_port_tunnel(&self, index: u8) -> Result<(), InterfaceError> {
        match &self.transport {
            Transport::SharedSerial(handle) => handle.send_set_port_tunnel(index),
            Transport::CanSerial(device) => device.send_set_port_tunnel(index),
            Transport::Host(device) => {
                if !device.is_mux() {
                    return Err(InterfaceError::Other(
                        "SET_PORT_TUNNEL needs a link that carries port mux units (a shared \
                         serial port, a muxed host library, or a can-serial link); this host \
                         library reference has no mux, so no bridge is listening"
                            .to_string(),
                    ));
                }
                device.send_set_port_tunnel(index)
            }
            _ => Err(InterfaceError::Other(
                "SET_PORT_TUNNEL needs a link that carries port mux units (a shared serial \
                 port, a muxed host library, or a can-serial link); this interface is not one"
                    .to_string(),
            )),
        }
    }

    /// Create an interface running klipper's host library from `path` on a path
    /// the config says is muxed: the library is given `mux`'s identity and the
    /// reference is attached under `mcu_id` ([`HostDevice::load_mux`]), so its
    /// frames carry the mux header from then on.
    ///
    /// `host_port_paths` is the tty each of this MCU's bridge ports opens, as
    /// `(slot, path)` (`mcu/bridge_ports.rs`): it is handed to the library
    /// before klipper comes up, so a port's tty is a run-time fact rather than
    /// one the library was built with. An empty slice — every reference that is
    /// not a bridge — leaves the ports on the library's own default.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] for the reasons [`Interface::host`] does, and
    /// [`InterfaceError::Timeout`] when the device never confirms the
    /// assignment.
    pub fn host_mux(
        path: impl AsRef<Path>,
        mux: HostMux,
        mcu_id: u32,
        host_port_paths: &[(u8, String)],
    ) -> Result<Self, InterfaceError> {
        Ok(Self::with_transport(Transport::Host(Arc::new(
            HostDevice::load_mux(path, mux, mcu_id, host_port_paths)?,
        ))))
    }

    /// Run one blocking device operation on the interface's own runtime.
    ///
    /// The device call blocks, so it goes to the blocking pool of the stored
    /// handle rather than whatever runtime happens to be ambient.
    ///
    /// `None` means that runtime is gone: a queued blocking op is cancelled when
    /// its runtime shuts down, and tokio reports that as a cancelled
    /// `JoinHandle`. That is a closed transport, not a device panic — `send`
    /// refuses it and `receive` ends its read loop.
    async fn off_runtime<T, F>(&self, op: F) -> Option<T>
    where
        T: Send + 'static,
        F: FnOnce() -> T + Send + 'static,
    {
        self.handle.spawn_blocking(op).await.ok()
    }

    pub async fn send(&self, frame: Frame) -> Result<(), InterfaceError> {
        match &self.transport {
            Transport::Serial(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.send(&frame))
                    .await
                    .unwrap_or(Err(InterfaceError::ConnectionLost))
            }
            Transport::SharedSerial(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.send(&frame))
                    .await
                    .unwrap_or(Err(InterfaceError::ConnectionLost))
            }
            Transport::CanSerial(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.send(&frame))
                    .await
                    .unwrap_or(Err(InterfaceError::ConnectionLost))
            }
            Transport::Host(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.send(&frame))
                    .await
                    .unwrap_or(Err(InterfaceError::ConnectionLost))
            }
            #[cfg(test)]
            Transport::FrameMock(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.send(&frame))
                    .await
                    .unwrap_or(Err(InterfaceError::ConnectionLost))
            }
            #[cfg(test)]
            Transport::Simulator(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.send(&frame))
                    .await
                    .unwrap_or(Err(InterfaceError::ConnectionLost))
            }
            #[cfg(test)]
            Transport::Recording(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.send(&frame))
                    .await
                    .unwrap_or(Err(InterfaceError::ConnectionLost))
            }
        }
    }

    pub async fn receive(&self) -> Option<Frame> {
        match &self.transport {
            Transport::Serial(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.receive()).await.flatten()
            }
            Transport::SharedSerial(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.receive()).await.flatten()
            }
            Transport::CanSerial(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.receive()).await.flatten()
            }
            Transport::Host(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.receive()).await.flatten()
            }
            #[cfg(test)]
            Transport::FrameMock(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.receive()).await.flatten()
            }
            #[cfg(test)]
            Transport::Simulator(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.receive()).await.flatten()
            }
            #[cfg(test)]
            Transport::Recording(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.receive()).await.flatten()
            }
        }
    }

    /// Shut down the underlying device, unblocking any pending `receive()`.
    pub fn shutdown(&self) {
        match &self.transport {
            Transport::Serial(device) => device.shutdown(),
            Transport::SharedSerial(device) => device.shutdown(),
            Transport::CanSerial(device) => device.shutdown(),
            Transport::Host(device) => device.shutdown(),
            #[cfg(test)]
            Transport::FrameMock(device) => device.shutdown(),
            #[cfg(test)]
            Transport::Simulator(device) => device.shutdown(),
            #[cfg(test)]
            Transport::Recording(device) => device.shutdown(),
        }
    }
}

// ===========================================================================
// TRACE formatting
// ===========================================================================

/// Bytes as 4-byte runs separated by a space.
///
/// A run shorter than four bytes is written whole, so the last run of a section
/// never trails a space: `01020304 05060708 090a`.
pub(crate) fn hex_runs(bytes: &[u8]) -> String {
    bytes
        .chunks(4)
        .map(|run| {
            run.iter()
                .map(|byte| format!("{byte:02x}"))
                .collect::<String>()
        })
        .collect::<Vec<_>>()
        .join(" ")
}

/// One frame's wire bytes, split into the parts the protocol gives them:
/// header (length, sequence) | payload | trailer (CRC, SYNC).
///
/// A flat dump makes the reader count bytes to find where the payload starts and
/// ends; `0a11 | 01020304 05 | 1a2b7e` does not. Bytes that are not one whole
/// frame — a read that stopped mid-frame, two frames glued together — have no
/// such parts, so they fall back to [`hex_runs`].
pub(crate) fn describe_frame(raw: &[u8]) -> String {
    match frame_parts(raw) {
        Some((header, payload, trailer)) => format!(
            "{} | {} | {}",
            hex_runs(header),
            hex_runs(payload),
            hex_runs(trailer)
        ),
        None => hex_runs(raw),
    }
}

/// Split a whole frame into its header, payload and trailer.
///
/// Only the length byte is trusted, and only when it describes exactly the bytes
/// given: a frame with a bad CRC is still a frame, and seeing where its parts
/// begin is the reason to log it at all.
fn frame_parts(raw: &[u8]) -> Option<(&[u8], &[u8], &[u8])> {
    let length = usize::from(*raw.first()?);
    if raw.len() != length || !(MESSAGE_MIN..=MESSAGE_MAX).contains(&length) {
        return None;
    }
    let payload_end = length - MESSAGE_TRAILER_SIZE;
    Some((
        &raw[..MESSAGE_HEADER_SIZE],
        &raw[MESSAGE_HEADER_SIZE..payload_end],
        &raw[payload_end..],
    ))
}

#[cfg(test)]
mod trace_tests {
    use super::*;

    #[test]
    fn test_hex_runs_groups_four_bytes_with_one_space() {
        assert_eq!(hex_runs(&[]), "");
        assert_eq!(hex_runs(&[0xab]), "ab");
        assert_eq!(hex_runs(&[1, 2, 3, 4]), "01020304");
        assert_eq!(hex_runs(&[1, 2, 3, 4, 5, 6]), "01020304 0506");
        assert_eq!(
            hex_runs(&[1, 2, 3, 4, 5, 6, 7, 8, 9]),
            "01020304 05060708 09"
        );
    }

    #[test]
    fn test_describe_frame_splits_header_payload_and_trailer() {
        let frame = Frame::encode(1, &[1, 2, 3, 4, 5]);
        assert_eq!(frame.len(), 10, "five bytes of overhead plus the payload");

        // length+sequence | payload in 4-byte runs | CRC+SYNC
        let expected = format!("0a11 | 01020304 05 | {:02x}{:02x}7e", frame[7], frame[8]);
        assert_eq!(describe_frame(&frame), expected);
    }

    #[test]
    fn test_describe_frame_leaves_an_empty_payload_empty() {
        let frame = Frame::encode(2, &[]);
        assert_eq!(frame.len(), MESSAGE_MIN);

        // The parts stay three even when the middle one has no bytes.
        let expected = format!("0512 |  | {:02x}{:02x}7e", frame[2], frame[3]);
        assert_eq!(describe_frame(&frame), expected);
    }

    #[test]
    fn test_describe_frame_falls_back_when_the_bytes_are_not_one_frame() {
        let frame = Frame::encode(1, &[9, 9]);

        // A fragment: the length byte promises more than arrived.
        assert_eq!(describe_frame(&frame[..3]), hex_runs(&frame[..3]));

        // Two frames glued together: no single length describes them.
        let pair = [frame.clone(), frame].concat();
        assert_eq!(describe_frame(&pair), hex_runs(&pair));
    }

    #[test]
    fn test_a_can_slice_is_formatted_as_a_serial_frame() {
        // CAN carries the serial byte stream cut every eight bytes, so a frame
        // that fills one CAN frame arrives whole and splits as usual.
        let whole = Frame::encode(1, &[1, 2, 3]);
        assert_eq!(whole.len(), 8, "one CAN frame's worth");
        let expected = format!("0811 | 010203 | {:02x}{:02x}7e", whole[5], whole[6]);
        assert_eq!(describe_frame(&whole), expected);

        // A longer frame is cut, so its first slice is a fragment with no whole
        // frame's parts to show; the same formatter degrades to runs.
        let longer = Frame::encode(1, &[0; 6]);
        assert_eq!(describe_frame(&longer[..8]), hex_runs(&longer[..8]));
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::interface::devices::canserial::read_wire;
    use crate::core::klippy::interface::devices::frame_mock::FrameMock;
    use crate::core::klippy::interface::devices::serial::{
        admin_unit, attach_downstream, set_port_node_id, CMD_RESET_PORT, CMD_RESTART_DEVICE,
        CMD_SET_PORT_TUNNEL, DEFAULT_BAUD, MUX_PORT_UNITS,
    };
    use crate::core::klippy::interface::pty::Pty;
    use std::io::Read;
    use std::os::fd::AsRawFd;
    use std::time::{Duration, Instant};

    fn make_frame(seq: u8, payload: &[u8]) -> Frame {
        Frame::new(seq, payload.to_vec())
    }

    /// Read exactly `want` bytes from the pty master within `window`; a sender
    /// that writes nothing fails on the expired window instead of blocking the
    /// test.
    fn read_master_n(master: &std::fs::File, want: usize, window: Duration) -> Vec<u8> {
        let deadline = Instant::now() + window;
        let mut master = master;
        let mut out = Vec::with_capacity(want);
        let mut buf = [0u8; 64];
        while out.len() < want {
            let now = Instant::now();
            assert!(
                now < deadline,
                "only {} of {want} bytes arrived within {window:?}",
                out.len()
            );
            let mut poll = libc::pollfd {
                fd: master.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            };
            let rc = unsafe {
                libc::poll(
                    &mut poll,
                    1,
                    deadline.saturating_duration_since(now).as_millis() as i32,
                )
            };
            assert!(
                rc >= 0,
                "polling the pty master: {}",
                std::io::Error::last_os_error()
            );
            if rc == 0 {
                continue;
            }
            let read = master.read(&mut buf).expect("pty master read");
            assert_ne!(read, 0, "the pty closed before {want} bytes arrived");
            out.extend_from_slice(&buf[..read]);
        }
        out
    }

    #[tokio::test]
    async fn test_interface_new_and_send_receive() {
        let input = make_frame(1, b"hello");
        let expected_output = make_frame(2, b"world");

        let device = FrameMock::new(vec![MappingEntry {
            input: input.clone(),
            outputs: vec![expected_output.clone()],
        }]);
        let interface = Interface::new(device);

        // Send should succeed
        let result = interface.send(input.clone()).await;
        assert!(result.is_ok());

        // Receive should return the expected output
        let output = interface.receive().await.expect("frame available");
        assert_eq!(output, expected_output);
    }

    #[tokio::test]
    async fn test_interface_multiple_send_receive() {
        let pairs: Vec<(Frame, Frame)> = vec![
            (make_frame(1, b"msg1"), make_frame(2, b"resp1")),
            (make_frame(3, b"msg2"), make_frame(4, b"resp2")),
            (make_frame(5, b"msg3"), make_frame(6, b"resp3")),
        ];

        let mappings: Vec<MappingEntry> = pairs
            .iter()
            .map(|(input, output)| MappingEntry {
                input: input.clone(),
                outputs: vec![output.clone()],
            })
            .collect();

        let device = FrameMock::new(mappings);
        let interface = Interface::new(device);

        for (i, (input, expected_output)) in pairs.iter().enumerate() {
            let idx = i + 1;
            let result = interface.send(input.clone()).await;
            assert!(result.is_ok(), "send #{} failed", idx);

            let output = interface.receive().await.expect("frame available");
            assert_eq!(output, *expected_output, "output #{} mismatch", idx);
        }
    }

    #[tokio::test]
    async fn test_interface_send_fails_on_mismatch() {
        let expected = make_frame(1, b"expected");
        let actual = make_frame(2, b"actual");

        let device = FrameMock::new(vec![MappingEntry {
            input: expected.clone(),
            outputs: vec![make_frame(3, b"response")],
        }]);
        let interface = Interface::new(device);

        let result = interface.send(actual).await;
        assert!(result.is_err());
        match result.unwrap_err() {
            InterfaceError::SendError(msg) => {
                assert!(msg.contains("does not match"));
            }
            other => panic!("Expected SendError, got {:?}", other),
        }
    }

    #[tokio::test]
    async fn test_interface_send_without_mapping() {
        let device = FrameMock::new(vec![]);
        let interface = Interface::new(device);

        let result = interface.send(make_frame(1, b"extra")).await;
        assert!(result.is_err());
        match result.unwrap_err() {
            InterfaceError::SendError(msg) => {
                assert!(msg.contains("No mapping entry"));
            }
            other => panic!("Expected SendError, got {:?}", other),
        }
    }

    #[tokio::test]
    async fn test_interface_clone_shares_device() {
        let input = make_frame(1, b"shared");
        let output1 = make_frame(2, b"reply1");
        let output2 = make_frame(3, b"reply2");

        let device = FrameMock::new(vec![
            MappingEntry {
                input: input.clone(),
                outputs: vec![output1.clone()],
            },
            MappingEntry {
                input: input.clone(),
                outputs: vec![output2.clone()],
            },
        ]);

        let interface = Interface::new(device);
        let cloned = interface.clone();

        // First use
        assert!(interface.send(input.clone()).await.is_ok());
        assert_eq!(interface.receive().await.unwrap(), output1);

        // Second use via clone
        assert!(cloned.send(input).await.is_ok());
        assert_eq!(cloned.receive().await.unwrap(), output2);
    }

    #[tokio::test]
    async fn test_interface_multiple_outputs_per_send() {
        let input = make_frame(1, b"broadcast");
        let outputs = vec![
            make_frame(2, b"reply1"),
            make_frame(3, b"reply2"),
            make_frame(4, b"reply3"),
        ];

        let device = FrameMock::new(vec![MappingEntry {
            input: input.clone(),
            outputs: outputs.clone(),
        }]);
        let interface = Interface::new(device);

        assert!(interface.send(input).await.is_ok());

        for (i, expected) in outputs.iter().enumerate() {
            let received = interface.receive().await.expect("frame available");
            assert_eq!(
                received,
                *expected,
                "output #{} mismatch: expected {:?}, got {:?}",
                i + 1,
                expected,
                received
            );
        }
    }

    #[tokio::test]
    async fn test_interface_frame_payload_preservation() {
        let payload = vec![0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0xFF];
        let input = make_frame(10, &payload);
        let expected_output = make_frame(20, &payload);

        let device = FrameMock::new(vec![MappingEntry {
            input: input.clone(),
            outputs: vec![expected_output.clone()],
        }]);
        let interface = Interface::new(device);

        assert!(interface.send(input).await.is_ok());
        let output = interface.receive().await.expect("frame available");
        assert_eq!(output.payload(), payload.as_slice());
        assert_eq!(output.seq(), 20);
    }

    #[tokio::test]
    async fn test_interface_send_error_preserves_message() {
        let device = FrameMock::new(vec![]);
        let interface = Interface::new(device);

        let result = interface.send(make_frame(1, b"no_mapping")).await;
        assert!(result.is_err());

        let err = result.unwrap_err();
        let err_str = format!("{}", err);
        assert!(
            err_str.contains("No mapping entry"),
            "Error message should contain 'No mapping entry', got: {}",
            err_str
        );
    }

    /// A runtime that is gone is a closed transport, not a device panic.
    ///
    /// [`Interface`] spawns device I/O on the handle captured when it was built.
    /// If that runtime shuts down first, the queued blocking op is cancelled —
    /// tokio hands back a cancelled `JoinHandle` — and reading that as a device
    /// panic killed the caller (in tests it turned a teardown race under load
    /// into a bogus failure). `send` refuses it and `receive` ends its loop.
    #[test]
    fn test_a_shut_down_runtime_is_a_closed_transport_not_a_panic() {
        // The interface captures the runtime it is built on…
        let inner = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .expect("the interface's runtime");
        let interface = inner.block_on(async { Interface::new(FrameMock::new(vec![])) });
        // …and that runtime then goes away, handle included.
        inner.shutdown_timeout(std::time::Duration::from_millis(50));

        // A later call runs on *this* runtime; the spawn still lands on the dead
        // handle, and that must come back as a closed transport.
        let outer = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .expect("the caller's runtime");
        outer.block_on(async {
            let err = interface
                .send(make_frame(1, b"after_shutdown"))
                .await
                .expect_err("a dead runtime cannot carry a send");
            assert_eq!(err, InterfaceError::ConnectionLost);
            assert!(
                interface.receive().await.is_none(),
                "a dead runtime ends the read loop"
            );
        });
        outer.shutdown_timeout(std::time::Duration::from_millis(50));
    }

    /// A CAN link **is** a bridge link: the node behind it is the bridge, and the
    /// port-level commands go out as the serial side's units cut into CAN frames —
    /// the same `0x03`/`0x04`/`0x05` units, not a CAN frame shape of their own.
    /// Measured off the wire the device writes.
    #[tokio::test]
    async fn test_a_can_link_carries_the_bridge_commands() {
        let (device, mut wire) = CanSerialDevice::test_wire(3);
        let interface = Interface::with_transport(Transport::CanSerial(Arc::new(device)));

        interface
            .send_set_port_node_id(1, 2, MUX_PORT_UNITS)
            .unwrap();
        interface.send_restart_device(1).unwrap();
        interface.send_reset_port(1).unwrap();

        let sent: Vec<u8> = read_wire(&mut wire)
            .iter()
            .flat_map(|frame| frame.data().to_vec())
            .collect();
        assert_eq!(
            sent,
            [
                set_port_node_id(1, 2, MUX_PORT_UNITS).expect("id 2 is addressable"),
                admin_unit(&[CMD_RESTART_DEVICE, 1]),
                admin_unit(&[CMD_RESET_PORT, 1]),
            ]
            .concat(),
            "the three units, byte for byte on the CAN byte stream"
        );
    }

    /// The tunnel toggle reaches the device's own sender through the interface:
    /// `send_set_port_tunnel(3)` on a can-serial link writes exactly the bytes
    /// the device-side sender does — the dispatch adds nothing of its own.
    #[tokio::test]
    async fn test_the_tunnel_toggle_dispatches_to_the_device_sender() {
        let (device, mut wire) = CanSerialDevice::test_wire(3);
        let interface = Interface::with_transport(Transport::CanSerial(Arc::new(device)));
        interface.send_set_port_tunnel(3).unwrap();

        let (direct, mut direct_wire) = CanSerialDevice::test_wire(3);
        direct.send_set_port_tunnel(3).unwrap();

        let through_interface: Vec<u8> = read_wire(&mut wire)
            .iter()
            .flat_map(|frame| frame.data().to_vec())
            .collect();
        let from_the_device: Vec<u8> = read_wire(&mut direct_wire)
            .iter()
            .flat_map(|frame| frame.data().to_vec())
            .collect();
        assert_eq!(
            through_interface, from_the_device,
            "the interface dispatch is byte for byte the device sender's"
        );
    }

    /// A shared serial port carries the toggle too: the dispatch writes the
    /// port's admin unit, byte for byte.
    #[tokio::test]
    async fn test_a_shared_port_carries_the_tunnel_toggle() {
        let pty = Pty::new();
        let handle =
            attach_downstream(&pty.path, DEFAULT_BAUD, 3).expect("a downstream attach opens");
        let interface = Interface::shared_serial(handle);
        interface.send_set_port_tunnel(3).unwrap();

        let unit = admin_unit(&[CMD_SET_PORT_TUNNEL, 3]);
        let written = read_master_n(&pty.master, unit.len(), Duration::from_secs(5));
        assert_eq!(written, unit, "the admin unit, byte for byte");
    }

    /// A host library reference that is **not** muxed has no admin channel, so
    /// the Host arm refuses the toggle with its own wording — nothing is written
    /// on the library.
    #[tokio::test]
    async fn test_a_non_mux_host_library_has_no_tunnel_command() {
        // One live host library per process: the same lock every host-library
        // test takes, so this load does not race theirs.
        let _serial = crate::core::klippy::interface::devices::host::HOST_LIBRARY_TEST_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let device = HostDevice::load_bytes(klipperx_test_support::klipper_host_lib_path(), None)
            .expect("the test host library loads");
        let interface = Interface::from_host(Arc::new(device));

        assert_eq!(
            interface.send_set_port_tunnel(3).unwrap_err(),
            InterfaceError::Other(
                "SET_PORT_TUNNEL needs a link that carries port mux units (a shared serial \
                 port, a muxed host library, or a can-serial link); this host library \
                 reference has no mux, so no bridge is listening"
                    .to_string()
            )
        );
    }

    /// The bridge commands travel a bridge's admin channel; an interface with no
    /// such link says so (`_` arm of each `send_*`) instead of writing a unit
    /// nobody listens to — the message is asserted verbatim.
    ///
    /// A can-serial CAN link is one of the links that do carry them (its node is
    /// the bridge, see [`test_a_can_link_carries_the_bridge_commands`]), so this
    /// pins the wording for the transports with no port-level channel at all: it
    /// names what such a link needs instead of calling the interface "none of
    /// them".
    #[tokio::test]
    async fn test_bridge_commands_need_a_bridge_link() {
        let interface = Interface::new(FrameMock::new(vec![]));

        assert_eq!(
            interface.send_set_port_node_id(1, 2, 0).unwrap_err(),
            InterfaceError::Other(
                "SET_PORT_NODEID needs a link that carries port mux units (a shared serial port, \
                 a muxed host library, or a can-serial link); this interface is not one"
                    .to_string()
            )
        );
        assert_eq!(
            interface.send_restart_device(1).unwrap_err(),
            InterfaceError::Other(
                "RESTART_DEVICE needs a link that carries port mux units (a shared serial port, \
                 a muxed host library, or a can-serial link); this interface is not one"
                    .to_string()
            )
        );
        assert_eq!(
            interface.send_reset_port(1).unwrap_err(),
            InterfaceError::Other(
                "RESET_PORT needs a link that carries port mux units (a shared serial port, a \
                 muxed host library, or a can-serial link); this interface is not one"
                    .to_string()
            )
        );
        assert_eq!(
            interface.send_set_port_tunnel(1).unwrap_err(),
            InterfaceError::Other(
                "SET_PORT_TUNNEL needs a link that carries port mux units (a shared serial \
                 port, a muxed host library, or a can-serial link); this interface is not one"
                    .to_string()
            )
        );
    }
}
