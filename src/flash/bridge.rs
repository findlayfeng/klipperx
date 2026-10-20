//! The `Read + Write` face of one bridge flashing tunnel.
//!
//! A bridge port in tunnel mode carries a katapult byte stream: the host wraps
//! each whole block in one mux unit ([`tunnel_unit`]) on the way down, and the
//! bridge hands each whole block back up as one unit on the way up. A flash
//! session drives a plain `Read + Write` byte stream and knows nothing of mux
//! framing, so this module is the adapter between the two: it cuts the
//! session's bytes into whole katapult blocks before they go out — a split
//! block would put outer framing inside the inner message, which the inner CRC
//! rejects — and serves the uplink's whole blocks back as bytes.
//!
//! The cut rule is not repeated here: the tag, the word-count length and the
//! `8..=128` bounds are `devices::serial`'s ([`tunnel_frame_len`] and
//! friends), the one place both the link readers and this adapter read them.
//!
//! [`tunnel_unit`]: crate::core::klippy::interface::devices::serial::tunnel_unit

use std::cell::RefCell;
use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::rc::Rc;
use std::sync::mpsc::{Receiver, TryRecvError};
use std::sync::Arc;

use crate::core::klippy::interface::devices::canserial::CanSerialDevice;
use crate::core::klippy::interface::devices::host::HostDevice;
use crate::core::klippy::interface::devices::serial::{
    tunnel_frame_len, SharedSerialHandle, TUNNEL_FRAME_MAX, TUNNEL_FRAME_MIN, TUNNEL_POS_WORDCNT,
    TUNNEL_TAG,
};
use crate::core::klippy::interface::error::InterfaceError;

/// The link half of one tunnel: put one whole katapult block on the wire as a
/// tunnel unit headed with `arb`.
///
/// Every transport that carries a mux unit implements this with its own
/// `send_unit` — the shared serial port, the host library and the CAN link
/// write the same bytes — so a [`TunnelBridge`] is built the same way over any
/// of them.
pub trait TunnelLink {
    /// Send `block` as one tunnel unit headed with `arb`. The caller hands over
    /// a whole katapult message only ([`TunnelWriter`] cuts them), so an
    /// implementation just wraps and writes it.
    fn send_tunnel_block(&self, arb: u32, block: &[u8]) -> io::Result<()>;
}

/// A link failure as the I/O error a `Write` can carry.
fn link_error(error: InterfaceError) -> io::Error {
    io::Error::other(error.to_string())
}

impl TunnelLink for SharedSerialHandle {
    fn send_tunnel_block(&self, arb: u32, block: &[u8]) -> io::Result<()> {
        self.send_unit(arb, block).map_err(link_error)
    }
}

impl TunnelLink for HostDevice {
    fn send_tunnel_block(&self, arb: u32, block: &[u8]) -> io::Result<()> {
        self.send_unit(arb, block).map_err(link_error)
    }
}

impl TunnelLink for CanSerialDevice {
    fn send_tunnel_block(&self, arb: u32, block: &[u8]) -> io::Result<()> {
        self.send_unit(arb, block).map_err(link_error)
    }
}

impl<T: TunnelLink + ?Sized> TunnelLink for Arc<T> {
    fn send_tunnel_block(&self, arb: u32, block: &[u8]) -> io::Result<()> {
        (**self).send_tunnel_block(arb, block)
    }
}

/// One handle on one bridge tunnel: the bridge's katapult byte stream, cut and
/// wrapped on the downlink and unpacked on the uplink.
///
/// `arb` is the arbitration id the downlink tunnel units are headed with — the
/// port's device id as a mux header, `arb_id(mcu_id)` for a legacy bridge port.
/// `frames` is the receiving end of the sink registered for the port's uplink
/// id ([`TunnelSink::channel`]), which yields whole katapult blocks; `link` is
/// the transport they go back down through.
///
/// [`TunnelSink::channel`]: crate::core::klippy::interface::devices::serial::TunnelSink::channel
pub struct TunnelBridge {
    arb: u32,
    state: Rc<RefCell<TunnelState>>,
    link: Rc<dyn TunnelLink>,
}

struct TunnelState {
    /// Whole katapult blocks the bridge handed up, waiting for `read`.
    frames: Receiver<Vec<u8>>,
    /// Bytes of one delivered block not yet handed to the caller.
    current: VecDeque<u8>,
    /// Session bytes not yet one whole katapult block.
    pending: Vec<u8>,
}

impl TunnelBridge {
    /// A bridge tunnel writing under `arb`, reading whole blocks from `frames`,
    /// and sending every whole block down `link`.
    pub fn new(arb: u32, frames: Receiver<Vec<u8>>, link: impl TunnelLink + 'static) -> Self {
        Self {
            arb,
            state: Rc::new(RefCell::new(TunnelState {
                frames,
                current: VecDeque::new(),
                pending: Vec::new(),
            })),
            link: Rc::new(link),
        }
    }

    /// The two halves a session drives — [`super::session::flash`]'s reader and
    /// writer. They share one queue, so the pair may not be used in parallel.
    pub fn split(&self) -> (TunnelReader, TunnelWriter) {
        (
            TunnelReader {
                state: Rc::clone(&self.state),
            },
            TunnelWriter {
                arb: self.arb,
                state: Rc::clone(&self.state),
                link: Rc::clone(&self.link),
            },
        )
    }
}

/// The uplink half: the bridge's whole katapult blocks, served as bytes.
pub struct TunnelReader {
    state: Rc<RefCell<TunnelState>>,
}

impl Read for TunnelReader {
    /// Copy the next bytes of the current block out, pulling the next block
    /// when it runs out. Never blocks: no block waiting answers
    /// [`WouldBlock`](io::ErrorKind::WouldBlock), which a session treats as
    /// "no bytes in this window" until its own deadline. A sink that has been
    /// dropped is a **broken pipe**, not end of stream: the bridge is still up
    /// and the tunnel may come back, and reporting `0` would let the session
    /// keep polling a window that can never fill — the failure has to end the
    /// run instead.
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        let mut state = self.state.borrow_mut();
        if state.current.is_empty() {
            match state.frames.try_recv() {
                Ok(block) => state.current.extend(block),
                Err(TryRecvError::Empty) => {
                    return Err(io::Error::new(
                        io::ErrorKind::WouldBlock,
                        "no tunnel block arrived yet",
                    ))
                }
                Err(TryRecvError::Disconnected) => {
                    return Err(io::Error::new(
                        io::ErrorKind::BrokenPipe,
                        "the bridge's tunnel sink is gone: no block can arrive any more",
                    ))
                }
            }
        }
        let take = buf.len().min(state.current.len());
        for (slot, byte) in buf[..take].iter_mut().zip(state.current.drain(..take)) {
            *slot = byte;
        }
        Ok(take)
    }
}

/// The downlink half: a session's katapult bytes, sent as whole blocks.
pub struct TunnelWriter {
    arb: u32,
    state: Rc<RefCell<TunnelState>>,
    link: Rc<dyn TunnelLink>,
}

impl Write for TunnelWriter {
    /// Append `buf` and send every whole katapult block that completes. A
    /// block split across two writes waits for its tail: the tunnel carries
    /// whole blocks only.
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut state = self.state.borrow_mut();
        state.pending.extend_from_slice(buf);
        while let Some(length) = whole_block(&state.pending)? {
            let block: Vec<u8> = state.pending.drain(..length).collect();
            self.link.send_tunnel_block(self.arb, &block)?;
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// The length of the whole katapult block at the front of `pending`, `None`
/// when the bytes so far do not yet fill one.
///
/// The one tunnel cut rule ([`tunnel_frame_len`], the tag and the `8..=128`
/// bounds). A front that cannot start a block is an error, because the session
/// writes katapult frames and nothing else — bytes that are not one mean the
/// adapter was handed a stream it cannot carry.
fn whole_block(pending: &[u8]) -> io::Result<Option<usize>> {
    if pending.len() <= TUNNEL_POS_WORDCNT {
        return Ok(None); // the word count is still in flight
    }
    if !pending.starts_with(&TUNNEL_TAG) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "tunnel write does not start with a katapult block",
        ));
    }
    let length = tunnel_frame_len(pending[TUNNEL_POS_WORDCNT]);
    if !(TUNNEL_FRAME_MIN..=TUNNEL_FRAME_MAX).contains(&length) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "tunnel write's word count is outside the katapult frame bounds",
        ));
    }
    Ok((pending.len() >= length).then_some(length))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    /// A [`TunnelLink`] that records what it was asked to send.
    #[derive(Default)]
    struct Recording {
        sent: Mutex<Vec<(u32, Vec<u8>)>>,
    }

    impl Recording {
        fn sent(&self) -> Vec<(u32, Vec<u8>)> {
            self.sent.lock().unwrap().clone()
        }
    }

    impl TunnelLink for Recording {
        fn send_tunnel_block(&self, arb: u32, block: &[u8]) -> io::Result<()> {
            self.sent.lock().unwrap().push((arb, block.to_vec()));
            Ok(())
        }
    }

    /// The golden Connect block: one whole katapult message.
    const BLOCK: [u8; 8] = [0x01, 0x88, 0x11, 0x00, 0xf1, 0x7c, 0x99, 0x03];

    #[test]
    fn test_write_sends_whole_katapult_blocks_only() {
        let link = Arc::new(Recording::default());
        let (_tx, frames) = std::sync::mpsc::channel();
        let bridge = TunnelBridge::new(7, frames, Arc::clone(&link));
        let (_reader, mut writer) = bridge.split();

        // One whole block goes out as it came, under the bridge's arb.
        writer.write_all(&BLOCK).unwrap();
        assert_eq!(link.sent(), vec![(7, BLOCK.to_vec())]);

        // A block split across two writes waits for its tail: only whole
        // blocks travel.
        writer.write_all(&BLOCK[..3]).unwrap();
        assert_eq!(link.sent().len(), 1, "half a block is not sent");
        writer.write_all(&BLOCK[3..]).unwrap();
        assert_eq!(link.sent().len(), 2, "the block goes once its tail arrives");
        assert_eq!(link.sent()[1], (7, BLOCK.to_vec()));
    }

    #[test]
    fn test_write_refuses_bytes_that_are_not_one_block() {
        let link = Arc::new(Recording::default());
        let (_tx, frames) = std::sync::mpsc::channel();
        let bridge = TunnelBridge::new(7, frames, Arc::clone(&link));
        let (_reader, mut writer) = bridge.split();

        // Not the tag: the session writes katapult frames, so this is not one.
        let err = writer
            .write_all(&[0x02, 0x88, 0x11, 0x00, 0x00, 0x00, 0x99, 0x03])
            .unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::InvalidData);
        assert!(link.sent().is_empty(), "nothing bad goes on the wire");

        // The right tag, but word count 31 claims 132 bytes: past the limit.
        let over = [0x01, 0x88, 0x12, 0x1f, 0x00, 0x00, 0x00, 0x00];
        let err = writer.write_all(&over).unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::InvalidData);
    }

    #[test]
    fn test_read_serves_whole_blocks_and_would_block() {
        let link = Arc::new(Recording::default());
        let (tx, frames) = std::sync::mpsc::channel();
        let bridge = TunnelBridge::new(7, frames, Arc::clone(&link));
        let (mut reader, _writer) = bridge.split();

        // No block waiting is a non-blocking “no bytes in this window”.
        let mut buf = [0u8; 64];
        let err = reader.read(&mut buf).unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::WouldBlock);

        // A delivered block is served whole, then in pieces across reads.
        tx.send(BLOCK.to_vec()).unwrap();
        assert_eq!(reader.read(&mut buf).unwrap(), BLOCK.len());
        assert_eq!(&buf[..BLOCK.len()], &BLOCK[..]);
        tx.send(vec![0xaa, 0xbb]).unwrap();
        let mut one = [0u8; 1];
        assert_eq!(reader.read(&mut one).unwrap(), 1);
        assert_eq!(one, [0xaa]);
        assert_eq!(reader.read(&mut one).unwrap(), 1);
        assert_eq!(one, [0xbb]);

        // A dropped sink is a broken pipe, not end of stream: nothing more can
        // arrive, and the session must end the run rather than poll on.
        drop(tx);
        let err = reader.read(&mut buf).unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::BrokenPipe);
    }
}
