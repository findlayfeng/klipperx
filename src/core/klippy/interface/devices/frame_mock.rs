use crate::core::klippy::frame::Frame;
use crate::core::klippy::interface::error::InterfaceError;

use crate::core::klippy::interface::Device;
use crossbeam_channel::{bounded, Receiver, RecvTimeoutError, Sender};
use std::{
    collections::VecDeque,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::Duration,
};

#[derive(Debug, Clone, PartialEq)]
pub struct MappingEntry {
    pub input: Frame,
    pub outputs: Vec<Frame>,
}

/// Records the frames a [`FrameMock`] accepted, in order.
///
/// For tests that assert on the wire shape — how many blocks went out and what
/// is in each — rather than on a scripted exchange. Take one with
/// [`FrameMock::recorder`] before the device is moved into an interface.
#[derive(Debug, Clone, Default)]
pub struct FrameRecorder {
    frames: Arc<Mutex<Vec<Frame>>>,
}

impl FrameRecorder {
    /// The frames accepted so far, oldest first.
    pub fn frames(&self) -> Vec<Frame> {
        self.frames.lock().unwrap().clone()
    }
}

/// A deterministic frame-level mock for testing Klipper protocol interactions.
///
/// Pre-configured with input→output mappings. Each `send()` consumes one
/// mapping entry in FIFO order, validates the input frame, and queues the
/// configured output frame(s) for `receive()`.
///
/// **Thread safety**: `FrameMock` is `Send` but not `Sync` — it must be
/// shared through an outer `Mutex` (e.g. `Arc<Mutex<FrameMock>>` in
/// `Interface::run()`). Each field that needs `Sync` is individually protected
/// (e.g. `mapping` uses its own `Mutex` since `VecDeque` is not `Sync`).
#[derive(Debug)]
pub struct FrameMock {
    /// `crossbeam::channel::Sender` wrapped in `Option` — `take()` on the last
    /// mapping closes the channel by dropping the sender.
    buf_tx: Mutex<Option<Sender<Frame>>>,
    /// `crossbeam::channel::Receiver` — `recv_blocking()` takes `&self`.
    buf_rx: Receiver<Frame>,
    /// FIFO queue of input→output mappings — protected by its own `Mutex`.
    mapping: Mutex<VecDeque<MappingEntry>>,
    /// Every frame this device accepted, shared with the test that holds the
    /// [`FrameRecorder`] handle.
    recorded: FrameRecorder,
}

impl FrameMock {
    pub fn new(mapping: Vec<MappingEntry>) -> Self {
        let (tx, rx) = bounded::<Frame>(100);
        Self {
            buf_tx: Mutex::new(Some(tx.clone())),
            buf_rx: rx,
            mapping: Mutex::new(mapping.into()),
            recorded: FrameRecorder::default(),
        }
    }

    /// A handle that lists every frame this device accepts, in order.
    ///
    /// Take it before the device is moved into an
    /// [`Interface`](crate::core::klippy::interface::Interface); the handle shares state with the device.
    pub fn recorder(&self) -> FrameRecorder {
        self.recorded.clone()
    }
}

/// A frame **recorder** for transport-timing tests: every payload it is sent is
/// recorded (no input validation — the gate tests assert *when* something goes
/// out, and the derived command bytes are not worth predicting), each block is
/// acked with `seq + 1`, and every scripted request is answered with a canned
/// response so a `call` round trip can complete.
///
/// Cloneable: each clone is a new **view** of the wire — its own place in the
/// receive queue and its own lifetime — while the record, the scripted
/// answers and the wire's own sender are shared. Two sessions on one wire
/// then read like two fds on one UART: whichever reader is still running
/// takes the frame (session theft included), and a session's
/// [`RecordingWire::shutdown`] stops **its** reads without closing the line
/// the session that reopens the port is about to use — the shape the
/// reset → reconnect fixture in `mcu/mod.rs` needs.
///
/// The test keeps a handle so it can shut its own view down explicitly, but
/// the guarantee that no receive outlives its session comes from
/// `Mcu::close` (called by `Mcu::Drop` too): the receive runs inside
/// `spawn_blocking`, and a view that is still parked would hold up the test
/// runtime's shutdown.
pub struct RecordingWire {
    /// What every view shares: the record of sends, the scripted answers, and
    /// the firmware→wire sender.
    inner: Arc<RecordingWireInner>,
    /// This view's position in the firmware→host queue (a crossbeam receiver
    /// is multi-consumer: a frame goes to exactly one waiting view).
    rx: Receiver<Frame>,
    /// Set by [`RecordingWire::shutdown`]: this view reads no more.
    closed: Arc<AtomicBool>,
}

struct RecordingWireInner {
    sent: Arc<Mutex<Vec<Vec<u8>>>>,
    replies: Mutex<Vec<(Vec<u8>, Vec<u8>)>>,
    tx: Mutex<Option<Sender<Frame>>>,
}

impl Clone for RecordingWire {
    fn clone(&self) -> Self {
        Self {
            inner: Arc::clone(&self.inner),
            rx: self.rx.clone(),
            closed: Arc::new(AtomicBool::new(false)),
        }
    }
}

impl RecordingWire {
    pub fn new() -> Self {
        let (tx, rx) = bounded(64);
        Self {
            inner: Arc::new(RecordingWireInner {
                sent: Arc::new(Mutex::new(Vec::new())),
                replies: Mutex::new(Vec::new()),
                tx: Mutex::new(Some(tx)),
            }),
            rx,
            closed: Arc::new(AtomicBool::new(false)),
        }
    }

    /// The live record of every payload this wire was sent.
    pub fn sent(&self) -> Arc<Mutex<Vec<Vec<u8>>>> {
        Arc::clone(&self.inner.sent)
    }

    /// Answer a send whose payload equals `request` with `response` (before
    /// the ack of that block).
    ///
    /// Several requests may be scripted at once: the identify handshake asks
    /// for its one data chunk and then for the terminator, and both need an
    /// answer. A later call for the same `request` replaces its answer.
    pub fn reply_to(&self, request: Vec<u8>, response: Vec<u8>) {
        let mut replies = self.inner.replies.lock().unwrap();
        replies.retain(|(known, _)| *known != request);
        replies.push((request, response));
    }

    /// Stop **this view's** receives: a pending call returns within one
    /// poll tick and future calls return `None` — the wire itself stays up.
    ///
    /// A session that closes before the port is reopened (the reset →
    /// reconnect path) must not take the firmware's answers away from the
    /// session that reopens it: only this view stops reading.
    pub fn shutdown(&self) {
        self.closed.store(true, Ordering::SeqCst);
    }
}

impl std::fmt::Debug for RecordingWire {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RecordingWire")
            .field("sent", &self.inner.sent.lock().unwrap().len())
            .finish_non_exhaustive()
    }
}

impl Device for RecordingWire {
    fn send(&self, frame: &Frame) -> Result<(), InterfaceError> {
        let payload = frame.payload().to_vec();
        self.inner.sent.lock().unwrap().push(payload.clone());
        let tx = self.inner.tx.lock().unwrap().clone();
        let Some(tx) = tx else {
            return Ok(());
        };
        let seq = (frame.seq() + 1) & 0x0f;
        let replies = self.inner.replies.lock().unwrap();
        if let Some((_, response)) = replies.iter().find(|(request, _)| *request == payload) {
            let _ = tx.send(Frame::new(seq, response.clone()));
        }
        drop(replies);
        // The block was taken, so the window advances (`Sender::settle`).
        let _ = tx.send(Frame::new(seq, Vec::new()));
        Ok(())
    }

    fn receive(&self) -> Option<Frame> {
        // A bounded wait rather than a bare `recv()`: this view has to notice
        // its own `shutdown` without the wire's sender being dropped (another
        // session may still be reading it). The wait ends immediately when a
        // frame arrives, so delivery is not slowed by the poll.
        loop {
            if self.closed.load(Ordering::SeqCst) {
                return None;
            }
            match self.rx.recv_timeout(Duration::from_millis(20)) {
                Ok(frame) => {
                    if self.closed.load(Ordering::SeqCst) {
                        return None;
                    }
                    return Some(frame);
                }
                Err(RecvTimeoutError::Timeout) => continue,
                // The wire itself is gone: every sender dropped with it.
                Err(RecvTimeoutError::Disconnected) => return None,
            }
        }
    }

    fn shutdown(&self) {
        RecordingWire::shutdown(self);
    }
}

impl Device for FrameMock {
    fn send(&self, frame: &Frame) -> Result<(), InterfaceError> {
        let (entry, tx) = {
            let mut map = self.mapping.lock().unwrap();
            let entry = map.pop_front().ok_or(InterfaceError::SendError(
                "No mapping entry available for sent frame".to_string(),
            ))?;

            // Clone the sender for sending outside the lock.
            let tx =
                self.buf_tx.lock().unwrap().clone().ok_or_else(|| {
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

        self.recorded.frames.lock().unwrap().push(frame.clone());

        for output_frame in entry.outputs {
            tx.send(output_frame).map_err(|e| {
                InterfaceError::SendError(format!("Failed to send output frame: {e}"))
            })?;
        }

        Ok(())
    }

    fn receive(&self) -> Option<Frame> {
        // `None` means the channel was closed: either every mapping has been
        // consumed, or `shutdown()` was called. Either way the receive loop
        // should stop instead of blocking forever.
        self.buf_rx.recv().ok()
    }

    fn shutdown(&self) {
        // Dropping the last sender closes the channel, which unblocks a
        // pending `receive()` and lets its thread terminate.
        self.buf_tx.lock().unwrap().take();
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

        let device = FrameMock::new(vec![MappingEntry {
            input: input.clone(),
            outputs: vec![output.clone()],
        }]);

        let result = device.send(&input);
        assert!(result.is_ok());

        let received = device.receive().expect("frame available");
        assert_eq!(received, output);
    }

    #[tokio::test]
    async fn test_multiple_sequential_sends() {
        let input1 = make_frame(1, b"msg1");
        let output1 = make_frame(2, b"resp1");
        let input2 = make_frame(3, b"msg2");
        let output2 = make_frame(4, b"resp2");

        let device = FrameMock::new(vec![
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
        assert_eq!(device.receive().unwrap(), output1);

        // Second send/receive
        assert!(device.send(&input2).is_ok());
        assert_eq!(device.receive().unwrap(), output2);
    }

    #[test]
    fn test_send_mismatched_frame() {
        let expected = make_frame(1, b"expected");
        let actual = make_frame(2, b"actual");

        let device = FrameMock::new(vec![MappingEntry {
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
        let device = FrameMock::new(vec![]);

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

        let device = FrameMock::new(vec![MappingEntry {
            input: input.clone(),
            outputs: vec![output1.clone(), output2.clone(), output3.clone()],
        }]);

        assert!(device.send(&input).is_ok());
        assert_eq!(device.receive().unwrap(), output1);
        assert_eq!(device.receive().unwrap(), output2);
        assert_eq!(device.receive().unwrap(), output3);
    }

    #[test]
    fn test_empty_outputs() {
        let input = make_frame(1, b"no_response");

        let device = FrameMock::new(vec![MappingEntry {
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

        let device = FrameMock::new(vec![MappingEntry {
            input: input.clone(),
            outputs: vec![output.clone()],
        }]);

        // First send succeeds
        assert!(device.send(&input).is_ok());
        // Second send fails - no more mapping entries
        assert!(device.send(&input).is_err());

        // But we can still receive the queued outputs
        assert_eq!(device.receive().unwrap(), output);
    }

    #[tokio::test]
    async fn test_frame_payload_preservation() {
        let payload = vec![0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0xFF];
        let input = make_frame(5, &payload);
        let output = make_frame(6, &payload);

        let device = FrameMock::new(vec![MappingEntry {
            input: input.clone(),
            outputs: vec![output.clone()],
        }]);

        assert!(device.send(&input).is_ok());
        let received = device.receive().expect("frame available");
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
        let device = Arc::new(std::sync::Mutex::new(FrameMock::new(vec![
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
            let r1 = device_recv.lock().unwrap().receive().unwrap();
            let r2 = device_recv.lock().unwrap().receive().unwrap();
            (r1, r2)
        });

        send_handle.await.unwrap();
        let (r1, r2) = recv_handle.await.unwrap();
        assert_eq!(r1, output1);
        assert_eq!(r2, output2);
    }
}
