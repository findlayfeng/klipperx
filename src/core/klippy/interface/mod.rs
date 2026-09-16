#[cfg(test)]
pub mod test;
#[cfg(test)]
pub use test::{MappingEntry, TestDevice};

use super::frame::Frame;
use super::traits::InterfaceError;
use std::sync::Arc;

pub trait Device: Send + Sync {
    fn send(&self, frame: &Frame) -> Result<(), InterfaceError>;
    fn receive(&self) -> Frame;
}

/// Generic interface for communicating with a Klipper device.
///
/// `D` is the underlying device type (e.g. [`TestDevice`](test::TestDevice)
/// in tests, or a real serial/socket device in production).
pub struct Interface<D: Device> {
    device: Arc<D>,
}

impl<D: Device + 'static> Interface<D> {
    pub fn new(device: D) -> Self {
        Self {
            device: Arc::new(device),
        }
    }

    pub async fn send(&self, frame: Frame) -> Result<(), InterfaceError> {
        let device = Arc::clone(&self.device);
        tokio::task::spawn_blocking(move || device.send(&frame))
            .await
            .expect("Interface send task panicked")
    }

    pub async fn receive(&self) -> Frame {
        let device = Arc::clone(&self.device);
        tokio::task::spawn_blocking(move || device.receive())
            .await
            .expect("Interface receive task panicked")
    }
}

impl<D: Device + Clone> Clone for Interface<D> {
    fn clone(&self) -> Self {
        Self {
            device: Arc::clone(&self.device),
        }
    }
}

/// Placeholder device for non-test builds (returns errors).
#[cfg(not(test))]
#[derive(Debug, Clone)]
pub struct StubDevice;

#[cfg(not(test))]
impl Device for StubDevice {
    fn send(&self, _frame: &Frame) -> Result<(), InterfaceError> {
        Err(InterfaceError::Other(
            "StubDevice: no real device configured".to_string(),
        ))
    }
    fn receive(&self) -> Frame {
        Frame::new(0, Vec::new())
    }
}

/// Concrete interface type for test builds.
#[cfg(test)]
pub type TestInterface = Interface<test::TestDevice>;

/// Concrete interface type for non-test builds (uses stub device).
#[cfg(not(test))]
pub type TestInterface = Interface<StubDevice>;
