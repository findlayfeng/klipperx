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

/// Interface for communicating with a Klipper device.
///
/// At runtime, this is either:
/// - `Test(TestDevice)` — when building for tests
/// - `Stub(StubDevice)` — for all other builds
#[derive(Clone)]
pub enum Interface {
    #[cfg(test)]
    Test(Arc<TestDevice>),
    Stub(Arc<StubDevice>),
}

impl Interface {
    /// Create a new `Interface` wrapping the given device.
    #[cfg(test)]
    pub fn new(device: TestDevice) -> Self {
        Self::Test(Arc::new(device))
    }

    /// Create a stub interface (always returns errors).
    #[cfg(not(test))]
    pub fn stub() -> Self {
        Self::Stub(Arc::new(StubDevice))
    }

    pub async fn send(&self, frame: Frame) -> Result<(), InterfaceError> {
        match self {
            #[cfg(test)]
            Self::Test(device) => {
                let device = Arc::clone(device);
                tokio::task::spawn_blocking(move || device.send(&frame))
                    .await
                    .expect("Interface send task panicked")
            }
            Self::Stub(device) => {
                let device = Arc::clone(device);
                tokio::task::spawn_blocking(move || device.send(&frame))
                    .await
                    .expect("Interface send task panicked")
            }
        }
    }

    pub async fn receive(&self) -> Frame {
        match self {
            #[cfg(test)]
            Self::Test(device) => {
                let device = Arc::clone(device);
                tokio::task::spawn_blocking(move || device.receive())
                    .await
                    .expect("Interface receive task panicked")
            }
            Self::Stub(device) => {
                let device = Arc::clone(device);
                tokio::task::spawn_blocking(move || device.receive())
                    .await
                    .expect("Interface receive task panicked")
            }
        }
    }
}

/// Placeholder device for non-test builds (returns errors).
#[derive(Debug, Clone)]
pub struct StubDevice;

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
