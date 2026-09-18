pub mod canserial;
pub mod error;
pub mod host;
pub mod serial;
#[cfg(test)]
pub mod test;

pub use canserial::CanSerialDevice;
pub use error::InterfaceError;
pub use host::HostDevice;
pub use serial::SerialDevice;
#[cfg(test)]
pub use test::{MappingEntry, TestDevice};

use super::frame::Frame;
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
/// One variant per transport a `[mcu]` section can ask for:
/// - `Serial(SerialDevice)` — a real MCU on a tty (`serial:`)
/// - `CanSerial(CanSerialDevice)` — a real MCU reached over CAN, using Klipper's
///   can-serial link (`canbus_uuid:` + `canbus_interface:` + `canbus_nodeid:`)
/// - `Host(HostDevice)` — klipper's host library, loaded from a shared object
///   (`host_library:`)
/// - `Test(TestDevice)` — a scripted device, in test builds (`test:`)
///
/// There is no "configured nothing" variant on purpose: a section that names no
/// transport is reported when it is parsed, rather than turned into an interface
/// that fails on every call.
#[derive(Debug, Clone)]
pub enum Interface {
    Serial(Arc<SerialDevice>),
    CanSerial(Arc<CanSerialDevice>),
    Host(Arc<HostDevice>),
    #[cfg(test)]
    Test(Arc<TestDevice>),
}

impl Interface {
    /// Create a new `Interface` wrapping the given device.
    #[cfg(test)]
    pub fn new(device: TestDevice) -> Self {
        Self::Test(Arc::new(device))
    }

    /// Create an interface for a real MCU on the serial port `path`.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] if the port cannot be opened or put into raw
    /// mode at `baud`.
    pub fn serial(path: impl AsRef<std::path::Path>, baud: u32) -> Result<Self, InterfaceError> {
        Ok(Self::Serial(Arc::new(SerialDevice::open(path, baud)?)))
    }

    /// Create an interface for the MCU `uuid` on the CAN interface `name`, brought
    /// up as node `nodeid`.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] if the interface does not exist or the socket
    /// cannot be set up.
    pub fn canserial(name: &str, uuid: [u8; 6], nodeid: u32) -> Result<Self, InterfaceError> {
        Ok(Self::CanSerial(Arc::new(CanSerialDevice::open(
            name, uuid, nodeid,
        )?)))
    }

    /// Create an interface running klipper's host library from `path`.
    ///
    /// # Errors
    /// Returns [`InterfaceError`] if the library cannot be loaded, does not
    /// export the expected symbols, or fails to initialize.
    pub fn host(path: impl AsRef<Path>) -> Result<Self, InterfaceError> {
        Ok(Self::Host(Arc::new(HostDevice::load(path)?)))
    }

    pub async fn send(&self, frame: Frame) -> Result<(), InterfaceError> {
        match self {
            Self::Serial(device) => {
                let device = Arc::clone(device);
                tokio::task::spawn_blocking(move || device.send(&frame))
                    .await
                    .expect("Interface send task panicked")
            }
            Self::CanSerial(device) => {
                let device = Arc::clone(device);
                tokio::task::spawn_blocking(move || device.send(&frame))
                    .await
                    .expect("Interface send task panicked")
            }
            Self::Host(device) => {
                let device = Arc::clone(device);
                tokio::task::spawn_blocking(move || device.send(&frame))
                    .await
                    .expect("Interface send task panicked")
            }
            #[cfg(test)]
            Self::Test(device) => {
                let device = Arc::clone(device);
                tokio::task::spawn_blocking(move || device.send(&frame))
                    .await
                    .expect("Interface send task panicked")
            }
        }
    }

    pub async fn receive(&self) -> Option<Frame> {
        match self {
            Self::Serial(device) => {
                let device = Arc::clone(device);
                tokio::task::spawn_blocking(move || device.receive())
                    .await
                    .expect("Interface receive task panicked")
            }
            Self::CanSerial(device) => {
                let device = Arc::clone(device);
                tokio::task::spawn_blocking(move || device.receive())
                    .await
                    .expect("Interface receive task panicked")
            }
            Self::Host(device) => {
                let device = Arc::clone(device);
                tokio::task::spawn_blocking(move || device.receive())
                    .await
                    .expect("Interface receive task panicked")
            }
            #[cfg(test)]
            Self::Test(device) => {
                let device = Arc::clone(device);
                tokio::task::spawn_blocking(move || device.receive())
                    .await
                    .expect("Interface receive task panicked")
            }
        }
    }

    /// Shut down the underlying device, unblocking any pending `receive()`.
    pub fn shutdown(&self) {
        match self {
            Self::Serial(device) => device.shutdown(),
            Self::CanSerial(device) => device.shutdown(),
            Self::Host(device) => device.shutdown(),
            #[cfg(test)]
            Self::Test(device) => device.shutdown(),
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::interface::test::TestDevice;

    fn make_frame(seq: u8, payload: &[u8]) -> Frame {
        Frame::new(seq, payload.to_vec())
    }

    #[tokio::test]
    async fn test_interface_new_and_send_receive() {
        let input = make_frame(1, b"hello");
        let expected_output = make_frame(2, b"world");

        let device = TestDevice::new(vec![MappingEntry {
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

        let device = TestDevice::new(mappings);
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

        let device = TestDevice::new(vec![MappingEntry {
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
        let device = TestDevice::new(vec![]);
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

        let device = TestDevice::new(vec![
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

        let device = TestDevice::new(vec![MappingEntry {
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

        let device = TestDevice::new(vec![MappingEntry {
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
        let device = TestDevice::new(vec![]);
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
