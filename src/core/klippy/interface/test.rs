// Test interface implementation of KlippyInterface
//
// This module provides a test interface that uses an mpsc channel
// for communication. It is only available in test mode
// (enabled by the "klipper" feature or #[cfg(test)]).
//
// Use this interface when you need deterministic, predictable responses
// for testing application logic without any external dependencies.

use super::super::error::KlippyError;
use super::super::frame::{Frame, MESSAGE_SEQ_MASK};
use super::super::msg::proto::Payload;
use super::super::traits::{InterfaceError, KlippyInterface};
use std::sync::{Arc, Mutex};
use tokio::sync::mpsc;
use tracing::info;

/// Fixed channel capacity
const CHANNEL_CAPACITY: usize = 16;

/// Fixed receive timeout (1 second)
const RECEIVE_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(1);

/// A mapping entry: one input frame maps to multiple output frames.
/// When the input frame is received, all output frames are enqueued in order.
#[derive(Debug, Clone)]
pub struct MappingEntry {
    /// The expected input frame (seq + payload).
    pub input: Frame,
    /// The output frames to enqueue when this input is received.
    pub outputs: Vec<Frame>,
}

/// Test interface KlippyInterface implementation
///
/// Uses an `mpsc` channel for receiving `Frame` data. When `receive()` is called,
/// it waits on the channel with a configurable timeout:
/// - If a frame is available in the channel, extracts and returns its payload
/// - If the timeout expires before a frame arrives, returns `Err(InterfaceError::Timeout)`
/// - If the channel is closed, returns `Err(InterfaceError::ConnectionLost)`
///
/// The `send()` method constructs a `Frame` with an auto-incrementing cyclic
/// sequence number (0~15) and enqueues it.
///
/// If a mapping list is provided, send() only accepts data in the order
/// specified by the input patterns. On a match, the corresponding outputs
/// are added to the output queue.
///
/// # Example
/// ```ignore
/// let interface = TestInterface::new();
///
/// let mut payload = Payload::new();
/// payload.extend(b"hello").unwrap();
/// interface.send(&payload).await.unwrap();
/// let result = interface.receive().await.unwrap();
/// assert_eq!(result.payload(), b"hello");
/// ```
pub struct TestInterface {
    tx: Arc<tokio::sync::Mutex<mpsc::Sender<Frame>>>,
    rx: Arc<tokio::sync::Mutex<mpsc::Receiver<Frame>>>,
    seq: Arc<Mutex<u8>>,
    rx_seq: Arc<Mutex<u8>>,
    timeout: std::time::Duration,
    /// Mapping list for expected send order.
    mapping: Arc<Mutex<Vec<MappingEntry>>>,
    /// Current index in the mapping list.
    mapping_idx: Arc<Mutex<usize>>,
}

impl TestInterface {
    /// Create a new TestInterface with a mapping list.
    ///
    /// The mapping list defines the expected order of inputs. When send()
    /// is called, the payload must match the next expected input in the
    /// list. On a match, the corresponding outputs are enqueued.
    ///
    /// # Arguments
    /// * `mapping` — List of input-to-output mappings.
    pub fn new(mapping: Vec<MappingEntry>) -> Self {
        let (tx, rx) = mpsc::channel::<Frame>(CHANNEL_CAPACITY);
        Self {
            tx: Arc::new(tokio::sync::Mutex::new(tx)),
            rx: Arc::new(tokio::sync::Mutex::new(rx)),
            seq: Arc::new(std::sync::Mutex::new(0u8)),
            rx_seq: Arc::new(std::sync::Mutex::new(0u8)),
            timeout: RECEIVE_TIMEOUT,
            mapping: Arc::new(Mutex::new(mapping)),
            mapping_idx: Arc::new(Mutex::new(0)),
        }
    }

    /// Get the next sequence number (cyclic 0~15).
    fn next_seq(&self) -> u8 {
        let mut seq = self.seq.lock().unwrap();
        let current = *seq;
        *seq = (current.wrapping_add(1)) & MESSAGE_SEQ_MASK;
        current
    }

    /// Check if the payload matches the next expected input in the mapping.
    /// If it matches, enqueue all outputs and return Ok(()).
    /// If it doesn't match or the mapping is exhausted, return an error.
    async fn check_and_enqueue(&self, payload: &Payload) -> Result<(), KlippyError> {
        let seq = self.next_seq();
        let incoming = Frame::new(seq, payload.payload().to_vec());

        // Collect outputs in a block scope to release locks before await
        let outputs = {
            let mut mapping = self.mapping.lock().unwrap();
            let mut idx = self.mapping_idx.lock().unwrap();

            if *idx >= mapping.len() {
                info!("[TEST INTERFACE] Mapping exhausted, no more expected inputs");
                return Err(KlippyError::Internal("mapping exhausted".to_string()));
            }

            let expected = &mapping[*idx].input;
            if incoming != *expected {
                info!(
                    "[TEST INTERFACE] Send mismatch: expected seq={} payload={:?}, got seq={} payload={:?}",
                    expected.seq(), expected.payload(),
                    incoming.seq(), incoming.payload()
                );
                return Err(KlippyError::Internal(format!(
                    "send mismatch: expected seq={} payload={:?}, got seq={} payload={:?}",
                    expected.seq(), expected.payload(),
                    incoming.seq(), incoming.payload()
                )));
            }

            // Match found, enqueue outputs
            let outputs = std::mem::take(&mut mapping[*idx].outputs);
            *idx += 1;
            outputs
        };

        let tx = self.tx.clone();
        let guard = tx.lock().await;
        for output in outputs {
            guard
                .send(output)
                .await
                .map_err(|_| KlippyError::Internal("channel closed".to_string()))?;
        }

        Ok(())
    }
}

#[async_trait::async_trait]
impl KlippyInterface for TestInterface {
    async fn send(&self, payload: &Payload) -> Result<(), KlippyError> {
        info!(
            "[TEST INTERFACE] Send payload_len={}",
            payload.len()
        );

        self.check_and_enqueue(payload).await
    }

    async fn receive(&self) -> Result<Payload, InterfaceError> {
        let rx = self.rx.clone();
        let rx_seq = self.rx_seq.clone();
        let timeout = self.timeout;

        match tokio::time::timeout(timeout, async {
            let mut guard = rx.lock().await;
            guard.recv().await
        })
        .await
        {
            Ok(Some(frame)) => {
                let expected_seq = *rx_seq.lock().unwrap();
                if frame.seq() != expected_seq {
                    info!(
                        "[TEST INTERFACE] Receive seq mismatch: expected {} got {}",
                        expected_seq, frame.seq()
                    );
                    return Err(InterfaceError::InvalidData);
                }
                let mut next_rx_seq = rx_seq.lock().unwrap();
                *next_rx_seq = (*next_rx_seq).wrapping_add(1) & MESSAGE_SEQ_MASK;
                drop(next_rx_seq);

                let mut payload = Payload::new();
                payload.extend(frame.payload()).map_err(|_| {
                    InterfaceError::Other("payload too large".to_string())
                })?;
                info!(
                    "[TEST INTERFACE] Receive seq={} payload_len={}",
                    frame.seq(),
                    payload.len()
                );
                Ok(payload)
            }
            Ok(None) => {
                info!("[TEST INTERFACE] Channel closed");
                Err(InterfaceError::ConnectionLost)
            }
            Err(_) => {
                info!("[TEST INTERFACE] Receive timed out");
                Err(InterfaceError::Timeout)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_payload(bytes: &[u8]) -> Payload {
        let mut p = Payload::new();
        p.extend(bytes).unwrap();
        p
    }

    fn assert_payload_eq(actual: &Payload, expected: &[u8]) {
        assert_eq!(actual.payload(), expected);
    }

    #[tokio::test]
    async fn test_send_receive() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, b"hello".to_vec()),
            outputs: vec![Frame::new(0, b"hello".to_vec())],
        }];
        let interface = TestInterface::new(mapping);

        interface.send(&make_payload(b"hello")).await.unwrap();

        let payload = interface.receive().await.unwrap();
        assert_payload_eq(&payload, b"hello");
    }

    #[tokio::test]
    async fn test_receive_timeout() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, b"dummy".to_vec()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);

        let result = interface.receive().await;
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), InterfaceError::Timeout);
    }

    #[tokio::test]
    async fn test_send_ok() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, b"test payload".to_vec()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);

        let result = interface.send(&make_payload(b"test payload")).await;
        assert!(result.is_ok());
    }

    #[tokio::test]
    async fn test_cyclic_seq() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, b"dummy".to_vec()),
            outputs: vec![],
        }];
        let interface = TestInterface::new(mapping);

        // Should cycle 0, 1, 2, ..., 15, 0, 1, ...
        let seqs: Vec<u8> = (0..18).map(|_| interface.next_seq()).collect();
        assert_eq!(seqs, vec![0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0, 1]);
    }

    #[tokio::test]
    async fn test_receive_multiple() {
        let mapping = vec![
            MappingEntry {
                input: Frame::new(0, b"cmd1".to_vec()),
                outputs: vec![Frame::new(0, vec![1, 2, 3])],
            },
            MappingEntry {
                input: Frame::new(1, b"cmd2".to_vec()),
                outputs: vec![Frame::new(1, vec![4, 5, 6])],
            },
        ];
        let interface = TestInterface::new(mapping);

        interface.send(&make_payload(b"cmd1")).await.unwrap();
        interface.send(&make_payload(b"cmd2")).await.unwrap();

        let p1 = interface.receive().await.unwrap();
        assert_payload_eq(&p1, &[1, 2, 3]);

        let p2 = interface.receive().await.unwrap();
        assert_payload_eq(&p2, &[4, 5, 6]);
    }

    #[tokio::test]
    async fn test_frame_seq_in_queue() {
        let mapping = vec![
            MappingEntry {
                input: Frame::new(0, b"first".to_vec()),
                outputs: vec![Frame::new(0, b"first".to_vec())],
            },
            MappingEntry {
                input: Frame::new(1, b"second".to_vec()),
                outputs: vec![Frame::new(1, b"second".to_vec())],
            },
            MappingEntry {
                input: Frame::new(2, b"third".to_vec()),
                outputs: vec![Frame::new(2, b"third".to_vec())],
            },
        ];
        let interface = TestInterface::new(mapping);

        interface.send(&make_payload(b"first")).await.unwrap();
        interface.send(&make_payload(b"second")).await.unwrap();
        interface.send(&make_payload(b"third")).await.unwrap();

        // Verify sequence numbers via receive
        let p1 = interface.receive().await.unwrap();
        let p2 = interface.receive().await.unwrap();
        let p3 = interface.receive().await.unwrap();

        assert_payload_eq(&p1, b"first");
        assert_payload_eq(&p2, b"second");
        assert_payload_eq(&p3, b"third");
    }

    #[tokio::test]
    async fn test_mapping_send_receive() {
        let mapping = vec![
            MappingEntry {
                input: Frame::new(0, b"cmd1".to_vec()),
                outputs: vec![
                    Frame::new(0, b"resp1a".to_vec()),
                    Frame::new(1, b"resp1b".to_vec()),
                ],
            },
            MappingEntry {
                input: Frame::new(1, b"cmd2".to_vec()),
                outputs: vec![Frame::new(2, b"resp2".to_vec())],
            },
        ];
        let interface = TestInterface::new(mapping);

        // Send in correct order
        interface.send(&make_payload(b"cmd1")).await.unwrap();
        interface.send(&make_payload(b"cmd2")).await.unwrap();

        // Receive responses in order
        let r1 = interface.receive().await.unwrap();
        let r2 = interface.receive().await.unwrap();
        let r3 = interface.receive().await.unwrap();

        assert_payload_eq(&r1, b"resp1a");
        assert_payload_eq(&r2, b"resp1b");
        assert_payload_eq(&r3, b"resp2");
    }

    #[tokio::test]
    async fn test_mapping_wrong_order() {
        let mapping = vec![
            MappingEntry {
                input: Frame::new(0, b"cmd1".to_vec()),
                outputs: vec![Frame::new(0, b"resp1".to_vec())],
            },
            MappingEntry {
                input: Frame::new(1, b"cmd2".to_vec()),
                outputs: vec![Frame::new(1, b"resp2".to_vec())],
            },
        ];
        let interface = TestInterface::new(mapping);

        // Send in wrong order
        let result = interface.send(&make_payload(b"cmd2")).await;
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_mapping_exhausted() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, b"cmd1".to_vec()),
            outputs: vec![Frame::new(0, b"resp1".to_vec())],
        }];
        let interface = TestInterface::new(mapping);

        interface.send(&make_payload(b"cmd1")).await.unwrap();

        // No more mappings
        let result = interface.send(&make_payload(b"cmd1")).await;
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_mapping_seq_mismatch() {
        let mapping = vec![MappingEntry {
            input: Frame::new(5, b"cmd1".to_vec()),
            outputs: vec![Frame::new(0, b"resp1".to_vec())],
        }];
        let interface = TestInterface::new(mapping);

        // Send with wrong seq (will get seq=0, expected seq=5)
        let result = interface.send(&make_payload(b"cmd1")).await;
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_receive_seq_mismatch() {
        let mapping = vec![MappingEntry {
            input: Frame::new(0, b"cmd1".to_vec()),
            outputs: vec![Frame::new(99, b"resp1".to_vec())],
        }];
        let interface = TestInterface::new(mapping);

        interface.send(&make_payload(b"cmd1")).await.unwrap();

        // Receive with wrong seq (frame has seq=99, expected seq=0)
        let result = interface.receive().await;
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), InterfaceError::InvalidData);
    }
}
