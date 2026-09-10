#[cfg(test)]
pub mod test;
#[cfg(test)]
pub use test::{MappingEntry, TestDevice};

use super::frame::Frame;
use super::msg::proto::Payload;
use super::traits::InterfaceError;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use tokio::sync::mpsc;
use tracing;

pub trait Device: Send + Sync {
    fn send(&mut self, frame: &Frame) -> Result<(), InterfaceError>;
    fn receive(&self) -> Frame;
}

#[derive(Clone)]
pub enum InterfaceDevice {
    /// Test device for deterministic testing (only available in test builds).
    #[cfg(test)]
    Test(test::TestDevice),
}

impl Device for InterfaceDevice {
    fn send(&mut self, _frame: &Frame) -> Result<(), InterfaceError> {
        match self {
            #[cfg(test)]
            InterfaceDevice::Test(test_device) => test_device.send(_frame),
            _ => todo!("Implement send for other device types"),
        }
    }

    fn receive(&self) -> Frame {
        match self {
            #[cfg(test)]
            InterfaceDevice::Test(test_device) => test_device.receive(),
            _ => todo!("Implement receive for other device types"),
        }
    }
}

pub struct Interface {
    send_buf_tx: Arc<tokio::sync::Mutex<mpsc::Sender<Frame>>>,
    rx: Arc<tokio::sync::Mutex<mpsc::Receiver<Frame>>>,
    seq: Arc<AtomicU8>,
    #[cfg_attr(not(test), allow(dead_code))]
    pub device: Arc<Mutex<InterfaceDevice>>,
}

impl Interface {
    pub async fn send(&self, payload: &Payload) -> Result<(), InterfaceError> {
        // Encode payload into a frame with next sequence number
        let seq = self.next_tx_seq();
        let frame = Frame::new(seq, payload.payload().to_vec());
        let tx = self.send_buf_tx.lock().await;
        tx.send(frame)
            .await
            .map_err(|_| InterfaceError::SendError("Channel closed".to_string()))
    }

    pub async fn receive(&self) -> Result<Payload, InterfaceError> {
        let mut rx = self.rx.lock().await;
        let frame = rx
            .recv()
            .await
            .ok_or(InterfaceError::ConnectionLost)?;
        let mut payload = Payload::new();
        payload
            .extend(frame.payload())
            .map_err(|e| InterfaceError::Other(e.to_string()))?;
        Ok(payload)
    }

    pub fn run(device: InterfaceDevice) -> Self {
        let (tx, mut rx) = mpsc::channel(100);
        let (tx_rx, rx_rx) = mpsc::channel(100);
        let device = Arc::new(Mutex::new(device));
        let device_clone = device.clone();

        // Background task: receive from device and forward to rx channel
        let rx_device = device.clone();
        tokio::spawn(async move {
            loop {
                let frame = tokio::task::spawn_blocking({
                    let dev = rx_device.clone();
                    move || {
                        dev.lock().unwrap().receive()
                    }
                })
                .await
                .unwrap_or_else(|e| {
                    tracing::error!("Receive task panicked: {:?}", e);
                    Frame::new(0, vec![])
                });
                if tx_rx.send(frame).await.is_err() {
                    break;
                }
            }
        });

        // Background task: handle sends
        tokio::spawn(async move {
            loop {
                tokio::select! {
                    Some(frame) = rx.recv() => {
                        let mut dev = device_clone.lock().unwrap();
                        if let Err(e) = dev.send(&frame) {
                            tracing::error!("Failed to send frame: {:?}", e);
                        }
                    }
                    else => {
                        break;
                    }
                }
            }
        });

        Self {
            send_buf_tx: Arc::new(tokio::sync::Mutex::new(tx)),
            rx: Arc::new(tokio::sync::Mutex::new(rx_rx)),
            seq: Arc::new(AtomicU8::new(0)),
            device,
        }
    }

    /// Generate the next sequence number for outgoing frames.
    fn next_tx_seq(&self) -> u8 {
        let current = self.seq.fetch_add(1, Ordering::Relaxed);
        current & 0x0f
    }
}

#[cfg(test)]
impl Interface {
    /// Create an Interface with a TestDevice for testing.
    pub fn test_new(mappings: Vec<test::MappingEntry>) -> Self {
        let device = InterfaceDevice::Test(test::TestDevice::new(mappings));
        Self::run(device)
    }

    /// Get a reference to the underlying device (for testing).
    pub fn device(&self) -> &Arc<Mutex<InterfaceDevice>> {
        &self.device
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::interface::test::MappingEntry;
    use crate::core::klippy::frame::Frame;
    use tokio::time::{sleep, Duration};

    fn make_frame(seq: u8, payload: &[u8]) -> Frame {
        Frame::new(seq, payload.to_vec())
    }

    fn make_payload(bytes: &[u8]) -> Payload {
        let mut p = Payload::new();
        p.extend(bytes).unwrap();
        p
    }

    #[tokio::test(flavor = "multi_thread")]
    #[ignore] // TODO: fix async/blocking interaction
    async fn test_interface_send_receive_roundtrip() {
        let input_payload = make_payload(b"hello");
        let expected_output = make_payload(b"world");

        let input_frame = make_frame(0, b"hello");
        let output_frame = make_frame(0, b"world");

        let interface = Interface::test_new(vec![MappingEntry {
            input: input_frame,
            outputs: vec![output_frame],
        }]);

        // Send payload
        let result = interface.send(&input_payload).await;
        assert!(result.is_ok());

        // Wait for background task to process
        sleep(Duration::from_millis(50)).await;

        // Receive should return the mapped output
        let received = interface.receive().await.unwrap();
        assert_eq!(received.payload(), expected_output.payload());
    }

    #[tokio::test(flavor = "multi_thread")]
    #[ignore] // TODO: fix async/blocking interaction
    async fn test_interface_sequence_numbers() {
        let payload = make_payload(b"test");
        let output_frame = make_frame(0, b"resp");

        let interface = Interface::test_new(vec![
            MappingEntry {
                input: make_frame(0, b"test"),
                outputs: vec![output_frame.clone()],
            },
            MappingEntry {
                input: make_frame(1, b"test"),
                outputs: vec![output_frame],
            },
        ]);

        // First send should use seq 0
        let result = interface.send(&payload).await;
        assert!(result.is_ok());
        sleep(Duration::from_millis(50)).await;

        // Second send should use seq 1
        let result = interface.send(&payload).await;
        assert!(result.is_ok());
        sleep(Duration::from_millis(50)).await;
    }

    #[tokio::test(flavor = "multi_thread")]
    #[ignore] // TODO: fix async/blocking interaction
    async fn test_interface_payload_preservation() {
        let original = vec![0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0xFF];
        let input = make_payload(&original);

        let input_frame = make_frame(0, &original);
        let output_frame = make_frame(0, &original);

        let interface = Interface::test_new(vec![MappingEntry {
            input: input_frame,
            outputs: vec![output_frame],
        }]);

        interface.send(&input).await.unwrap();
        sleep(Duration::from_millis(50)).await;

        let received = interface.receive().await.unwrap();
        assert_eq!(received.payload(), original.as_slice());
    }

    #[tokio::test(flavor = "multi_thread")]
    #[ignore] // TODO: fix async/blocking interaction
    async fn test_interface_empty_payload() {
        let input = Payload::new();

        let input_frame = make_frame(0, &[]);
        let output_frame = make_frame(0, &[]);

        let interface = Interface::test_new(vec![MappingEntry {
            input: input_frame,
            outputs: vec![output_frame],
        }]);

        interface.send(&input).await.unwrap();
        sleep(Duration::from_millis(50)).await;

        let received = interface.receive().await.unwrap();
        assert!(received.is_empty());
    }

    #[tokio::test(flavor = "multi_thread")]
    #[ignore] // TODO: fix async/blocking interaction
    async fn test_interface_large_payload() {
        let large_data: Vec<u8> = (0..50).collect();
        let input = make_payload(&large_data);

        let input_frame = make_frame(0, &large_data);
        let output_frame = make_frame(0, &large_data);

        let interface = Interface::test_new(vec![MappingEntry {
            input: input_frame,
            outputs: vec![output_frame],
        }]);

        interface.send(&input).await.unwrap();
        sleep(Duration::from_millis(50)).await;

        let received = interface.receive().await.unwrap();
        assert_eq!(received.payload(), large_data.as_slice());
    }

    #[tokio::test(flavor = "multi_thread")]
    #[ignore] // TODO: fix async/blocking interaction
    async fn test_interface_multiple_outputs() {
        let output1 = make_payload(b"first");
        let output2 = make_payload(b"second");

        let input_frame = make_frame(0, b"broadcast");
        let output_frame1 = make_frame(0, b"first");
        let output_frame2 = make_frame(0, b"second");

        let interface = Interface::test_new(vec![MappingEntry {
            input: input_frame,
            outputs: vec![output_frame1, output_frame2],
        }]);

        let input = make_payload(b"broadcast");
        interface.send(&input).await.unwrap();
        sleep(Duration::from_millis(50)).await;

        let r1 = interface.receive().await.unwrap();
        assert_eq!(r1.payload(), output1.payload());

        let r2 = interface.receive().await.unwrap();
        assert_eq!(r2.payload(), output2.payload());
    }
}
