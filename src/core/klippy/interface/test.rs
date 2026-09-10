use crate::core::klippy::traits::InterfaceError;

use super::super::frame::Frame;
use super::Device;
use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
};
use tokio::sync::mpsc;

#[derive(Debug, Clone)]
pub struct MappingEntry {
    pub input: Frame,
    pub outputs: Vec<Frame>,
}

#[derive(Debug)]
pub struct TestDevice {
    buf_rx: Arc<Mutex<mpsc::Receiver<Frame>>>,
    buf_tx: Arc<Mutex<mpsc::Sender<Frame>>>,
    mapping: Arc<Mutex<VecDeque<MappingEntry>>>,
}

impl Clone for TestDevice {
    fn clone(&self) -> Self {
        Self {
            buf_rx: self.buf_rx.clone(),
            buf_tx: self.buf_tx.clone(),
            mapping: self.mapping.clone(),
        }
    }
}

impl TestDevice {
    pub fn new(mapping: Vec<MappingEntry>) -> Self {
        let (tx, rx) = mpsc::channel::<Frame>(100);
        Self {
            buf_rx: Arc::new(Mutex::new(rx)),
            buf_tx: Arc::new(Mutex::new(tx)),
            mapping: Arc::new(Mutex::new(mapping.into())),
        }
    }
}

impl Device for TestDevice {
    fn send(&mut self, frame: &Frame) -> Result<(), InterfaceError> {
        let mut map = self.mapping.lock().unwrap();

        let entry = map.pop_front().ok_or(InterfaceError::SendError(
            "No mapping entry available for sent frame".to_string(),
        ))?;

        if entry.input != *frame {
            return Err(InterfaceError::SendError(format!(
                "Sent frame does not match expected input frame. Expected: {:?}, Got: {:?}",
                entry.input, frame
            )));
        }

        for output_frame in entry.outputs {
            let tx = self.buf_tx.lock().unwrap();
            tx.blocking_send(output_frame)
                .map_err(|e| InterfaceError::SendError(format!("Failed to send output frame: {}", e)))?;
        }

        Ok(())
    }

    fn receive(&self) -> Frame {
        let mut rx = self.buf_rx.lock().unwrap();
        rx.blocking_recv()
            .expect("Failed to receive frame from test device")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_frame(seq: u8, payload: &[u8]) -> Frame {
        Frame::new(seq, payload.to_vec())
    }

    #[test]
    fn test_single_send_receive() {
        let input = make_frame(1, b"hello");
        let output = make_frame(2, b"world");

        let mut device = TestDevice::new(vec![MappingEntry {
            input: input.clone(),
            outputs: vec![output.clone()],
        }]);

        let result = device.send(&input);
        assert!(result.is_ok());

        let received = device.receive();
        assert_eq!(received, output);
    }

    #[test]
    fn test_multiple_sequential_sends() {
        let input1 = make_frame(1, b"msg1");
        let output1 = make_frame(2, b"resp1");
        let input2 = make_frame(3, b"msg2");
        let output2 = make_frame(4, b"resp2");

        let mut device = TestDevice::new(vec![
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

        let mut device = TestDevice::new(vec![MappingEntry {
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
        let mut device = TestDevice::new(vec![]);

        let result = device.send(&frame);
        assert!(result.is_err());
        match result.unwrap_err() {
            InterfaceError::SendError(msg) => {
                assert!(msg.contains("No mapping entry"));
            }
            other => panic!("Expected SendError, got {:?}", other),
        }
    }

    #[test]
    fn test_multiple_outputs_per_input() {
        let input = make_frame(1, b"broadcast");
        let output1 = make_frame(2, b"reply1");
        let output2 = make_frame(3, b"reply2");
        let output3 = make_frame(4, b"reply3");

        let mut device = TestDevice::new(vec![MappingEntry {
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

        let mut device = TestDevice::new(vec![MappingEntry {
            input: input.clone(),
            outputs: vec![],
        }]);

        assert!(device.send(&input).is_ok());
        // No outputs queued, so receive would block forever
        // This tests that empty outputs don't cause an error
    }

    #[test]
    fn test_send_without_matching_receive() {
        let input = make_frame(1, b"data");
        let output = make_frame(2, b"result");

        let mut device = TestDevice::new(vec![MappingEntry {
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

    #[test]
    fn test_frame_payload_preservation() {
        let payload = vec![0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0xFF];
        let input = make_frame(5, &payload);
        let output = make_frame(6, &payload);

        let mut device = TestDevice::new(vec![MappingEntry {
            input: input.clone(),
            outputs: vec![output.clone()],
        }]);

        assert!(device.send(&input).is_ok());
        let received = device.receive();
        assert_eq!(received.payload(), payload.as_slice());
    }
}
