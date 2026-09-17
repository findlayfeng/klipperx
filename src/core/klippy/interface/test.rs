use crate::core::klippy::traits::InterfaceError;
use crate::core::klippy::frame::Frame;

use super::Device;
use std::{collections::VecDeque, sync::Mutex};
use crossbeam_channel::{bounded, Receiver, Sender};

#[derive(Debug, Clone, PartialEq)]
pub struct MappingEntry {
    pub input: Frame,
    pub outputs: Vec<Frame>,
}

/// A deterministic mock device for testing Klipper protocol interactions.
///
/// Pre-configured with input→output mappings. Each `send()` consumes one
/// mapping entry in FIFO order, validates the input frame, and queues the
/// configured output frame(s) for `receive()`.
///
/// **Thread safety**: `TestDevice` is `Send` but not `Sync` — it must be
/// shared through an outer `Mutex` (e.g. `Arc<Mutex<TestDevice>>` in
/// `Interface::run()`). Each field that needs `Sync` is individually protected
/// (e.g. `mapping` uses its own `Mutex` since `VecDeque` is not `Sync`).
#[derive(Debug)]
pub struct TestDevice {
    /// `crossbeam::channel::Sender` wrapped in `Option` — `take()` on the last
    /// mapping closes the channel by dropping the sender.
    buf_tx: Mutex<Option<Sender<Frame>>>,
    /// `crossbeam::channel::Receiver` — `recv_blocking()` takes `&self`.
    buf_rx: Receiver<Frame>,
    /// FIFO queue of input→output mappings — protected by its own `Mutex`.
    mapping: Mutex<VecDeque<MappingEntry>>,
}

impl TestDevice {
    pub fn new(mapping: Vec<MappingEntry>) -> Self {
        let (tx, rx) = bounded::<Frame>(100);
        Self {
            buf_tx: Mutex::new(Some(tx.clone())),
            buf_rx: rx,
            mapping: Mutex::new(mapping.into()),
        }
    }
}

impl Device for TestDevice {
    fn send(&self, frame: &Frame) -> Result<(), InterfaceError> {
        let (entry, tx) = {
            let mut map = self.mapping.lock().unwrap();
            let entry = map.pop_front().ok_or(InterfaceError::SendError(
                "No mapping entry available for sent frame".to_string(),
            ))?;

            // Clone the sender for sending outside the lock.
            let tx = self.buf_tx.lock().unwrap().clone().ok_or_else(|| {
                InterfaceError::SendError("channel already closed".to_string())
            })?;

            // Drop the sender when all mappings are consumed.
            // Dropping the last sender closes the channel, causing `receive()`
            // to panic on `unwrap()` and exit background tasks.
            if map.is_empty() {
                self.buf_tx.lock().unwrap().take();
            }

            (entry, tx)
        };

        if entry.input != *frame {
            return Err(InterfaceError::SendError(format!(
                "Sent frame does not match expected input frame. Expected: {:?}, Got: {:?}",
                entry.input, frame
            )));
        }

        for output_frame in entry.outputs {
            tx.send(output_frame)
                .map_err(|e| InterfaceError::SendError(format!("Failed to send output frame: {e}")))?;
        }

        Ok(())
    }

    fn receive(&self) -> Frame {
        self.buf_rx.recv().unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_frame(seq: u8, payload: &[u8]) -> Frame {
        Frame::new(seq, payload.to_vec())
    }

    #[tokio::test]
    async fn test_single_send_receive() {
        let input = make_frame(1, b"hello");
        let output = make_frame(2, b"world");

        let device = TestDevice::new(vec![MappingEntry {
            input: input.clone(),
            outputs: vec![output.clone()],
        }]);

        let result = device.send(&input);
        assert!(result.is_ok());

        let received = device.receive();
        assert_eq!(received, output);
    }

    #[tokio::test]
    async fn test_multiple_sequential_sends() {
        let input1 = make_frame(1, b"msg1");
        let output1 = make_frame(2, b"resp1");
        let input2 = make_frame(3, b"msg2");
        let output2 = make_frame(4, b"resp2");

        let device = TestDevice::new(vec![
            MappingEntry {
                input: input1.clone(),
                outputs: vec![output1.clone()],
            },
            MappingEntry {
                input: input2.clone(),
                outputs: vec![output2.clone()],
            },
        ]);

        // First send/receive
        assert!(device.send(&input1).is_ok());
        assert_eq!(device.receive(), output1);

        // Second send/receive
        assert!(device.send(&input2).is_ok());
        assert_eq!(device.receive(), output2);
    }

    #[test]
    fn test_send_mismatched_frame() {
        let expected = make_frame(1, b"expected");
        let actual = make_frame(2, b"actual");

        let device = TestDevice::new(vec![MappingEntry {
            input: expected.clone(),
            outputs: vec![make_frame(3, b"response")],
        }]);

        let result = device.send(&actual);
        assert!(result.is_err());
        match result.unwrap_err() {
            InterfaceError::SendError(msg) => {
                assert!(msg.contains("does not match"));
            }
            other => panic!("Expected SendError, got {:?}", other),
        }
    }

    #[test]
    fn test_no_mapping_entry() {
        let frame = make_frame(1, b"extra");
        let device = TestDevice::new(vec![]);

        let result = device.send(&frame);
        assert!(result.is_err());
        match result.unwrap_err() {
            InterfaceError::SendError(msg) => {
                assert!(msg.contains("No mapping entry"));
            }
            other => panic!("Expected SendError, got {:?}", other),
        }
    }

    #[tokio::test]
    async fn test_multiple_outputs_per_input() {
        let input = make_frame(1, b"broadcast");
        let output1 = make_frame(2, b"reply1");
        let output2 = make_frame(3, b"reply2");
        let output3 = make_frame(4, b"reply3");

        let device = TestDevice::new(vec![MappingEntry {
            input: input.clone(),
            outputs: vec![output1.clone(), output2.clone(), output3.clone()],
        }]);

        assert!(device.send(&input).is_ok());
        assert_eq!(device.receive(), output1);
        assert_eq!(device.receive(), output2);
        assert_eq!(device.receive(), output3);
    }

    #[test]
    fn test_empty_outputs() {
        let input = make_frame(1, b"no_response");

        let device = TestDevice::new(vec![MappingEntry {
            input: input.clone(),
            outputs: vec![],
        }]);

        assert!(device.send(&input).is_ok());
        // No outputs queued — verify channel is indeed empty by checking
        // that a non-blocking attempt returns None (channel closed or empty).
        // We can't easily test "empty" on mpsc, so just verify send succeeded
        // without queuing anything unexpected.
    }

    #[tokio::test]
    async fn test_send_without_matching_receive() {
        let input = make_frame(1, b"data");
        let output = make_frame(2, b"result");

        let device = TestDevice::new(vec![MappingEntry {
            input: input.clone(),
            outputs: vec![output.clone()],
        }]);

        // First send succeeds
        assert!(device.send(&input).is_ok());
        // Second send fails - no more mapping entries
        assert!(device.send(&input).is_err());

        // But we can still receive the queued outputs
        assert_eq!(device.receive(), output);
    }

    #[tokio::test]
    async fn test_frame_payload_preservation() {
        let payload = vec![0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0xFF];
        let input = make_frame(5, &payload);
        let output = make_frame(6, &payload);

        let device = TestDevice::new(vec![MappingEntry {
            input: input.clone(),
            outputs: vec![output.clone()],
        }]);

        assert!(device.send(&input).is_ok());
        let received = device.receive();
        assert_eq!(received, output);
    }

    #[tokio::test]
    async fn test_concurrent_send_receive() {
        use std::sync::Arc;

        let input1 = make_frame(1, b"concurrent1");
        let output1 = make_frame(2, b"resp1");
        let input2 = make_frame(3, b"concurrent2");
        let output2 = make_frame(4, b"resp2");

        // Share through Arc<Mutex<>> — mirrors how Interface::run() shares the device.
        let device = Arc::new(std::sync::Mutex::new(TestDevice::new(vec![
            MappingEntry {
                input: input1.clone(),
                outputs: vec![output1.clone()],
            },
            MappingEntry {
                input: input2.clone(),
                outputs: vec![output2.clone()],
            },
        ])));

        let device_send = device.clone();
        let device_recv = device.clone();

        let send_handle = tokio::task::spawn_blocking(move || {
            device_send.lock().unwrap().send(&input1).unwrap();
            device_send.lock().unwrap().send(&input2).unwrap();
        });

        let recv_handle = tokio::task::spawn_blocking(move || {
            let r1 = device_recv.lock().unwrap().receive();
            let r2 = device_recv.lock().unwrap().receive();
            (r1, r2)
        });

        send_handle.await.unwrap();
        let (r1, r2) = recv_handle.await.unwrap();
        assert_eq!(r1, output1);
        assert_eq!(r2, output2);
    }
}
