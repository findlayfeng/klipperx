pub mod devices;
pub mod error;
#[cfg(test)]
pub(crate) mod pty;
pub mod usb;

pub use devices::canserial::CanSerialDevice;
#[cfg(test)]
pub use devices::frame_mock::{FrameMock, MappingEntry};
pub use devices::host::HostDevice;
pub use devices::serial::SerialDevice;
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
/// - `CanSerial(CanSerialDevice)` — a real MCU reached over CAN, using Klipper's
///   can-serial link (`canbus_uuid:` + `canbus_interface:` + `canbus_nodeid:`)
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

    /// Create an interface for the MCU `uuid` on the CAN interface `name`, brought
    /// up as node `nodeid`.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] if the interface does not exist or the socket
    /// cannot be set up.
    pub fn canserial(name: &str, uuid: [u8; 6], nodeid: u32) -> Result<Self, InterfaceError> {
        Ok(Self::with_transport(Transport::CanSerial(Arc::new(
            CanSerialDevice::open(name, uuid, nodeid)?,
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

    /// Run one blocking device operation on the interface's own runtime.
    ///
    /// The device call blocks, so it goes to the blocking pool of the stored
    /// handle rather than whatever runtime happens to be ambient.
    async fn off_runtime<T, F>(&self, op: F) -> T
    where
        T: Send + 'static,
        F: FnOnce() -> T + Send + 'static,
    {
        self.handle
            .spawn_blocking(op)
            .await
            .expect("Interface device task panicked")
    }

    pub async fn send(&self, frame: Frame) -> Result<(), InterfaceError> {
        match &self.transport {
            Transport::Serial(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.send(&frame)).await
            }
            Transport::CanSerial(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.send(&frame)).await
            }
            Transport::Host(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.send(&frame)).await
            }
            #[cfg(test)]
            Transport::FrameMock(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.send(&frame)).await
            }
            #[cfg(test)]
            Transport::Simulator(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.send(&frame)).await
            }
            #[cfg(test)]
            Transport::Recording(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.send(&frame)).await
            }
        }
    }

    pub async fn receive(&self) -> Option<Frame> {
        match &self.transport {
            Transport::Serial(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.receive()).await
            }
            Transport::CanSerial(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.receive()).await
            }
            Transport::Host(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.receive()).await
            }
            #[cfg(test)]
            Transport::FrameMock(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.receive()).await
            }
            #[cfg(test)]
            Transport::Simulator(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.receive()).await
            }
            #[cfg(test)]
            Transport::Recording(device) => {
                let device = Arc::clone(device);
                self.off_runtime(move || device.receive()).await
            }
        }
    }

    /// Shut down the underlying device, unblocking any pending `receive()`.
    pub fn shutdown(&self) {
        match &self.transport {
            Transport::Serial(device) => device.shutdown(),
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
    use crate::core::klippy::interface::devices::frame_mock::FrameMock;

    fn make_frame(seq: u8, payload: &[u8]) -> Frame {
        Frame::new(seq, payload.to_vec())
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
}
