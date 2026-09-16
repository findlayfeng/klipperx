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

pub enum InterfaceDevice {
    /// Test device for deterministic testing (only available in test builds).
    #[cfg(test)]
    Test(test::TestDevice),
}

pub struct Interface {
    device: Arc<InterfaceDevice>,
}

impl Interface {
    pub fn new(device: InterfaceDevice) -> Self {
        Self {
            device: Arc::new(device),
        }
    }

    pub async fn send(&self, frame: Frame) -> Result<(), InterfaceError> {
        match self.device.as_ref() {
            #[cfg(test)]
            InterfaceDevice::Test(test_device) => test_device.send(&frame),
            #[cfg(not(test))]
            _ => todo!("Implement send for other device types: {frame:?}"),
        }
    }

    pub async fn receive(&self) -> Frame {
        match self.device.as_ref() {
            #[cfg(test)]
            InterfaceDevice::Test(test_device) => test_device.receive(),
            #[cfg(not(test))]
            _ => todo!("Implement receive for other device types"),
        }
    }
}
