//! Clock synchronisation — reading the firmware's free-running clock.
//!
//! # Protocol
//!
//! The firmware exposes its clock counter through a request/response pair. Both
//! messages come from the data dictionary:
//!
//! | Direction | Format |
//! |---|---|
//! | host → MCU | `get_clock` |
//! | MCU → host | `clock clock=%u` |
//!
//! The value is the low 32 bits of the MCU clock, which ticks at
//! `CLOCK_FREQ` (a firmware constant available through
//! [`Dictionary::constant_f64`](crate::core::klippy::mcu::Dictionary::constant_f64)).
//! It wraps at 2^32 ticks; a 64-bit view needs the `get_uptime`/`uptime` pair,
//! which is not implemented yet.

use crate::core::klippy::mcu::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::{Mcu, McuError};
use crate::core::klippy::msg::proto::ArgValue;
use std::future::Future;
use std::sync::Arc;
use tokio::time::Duration;

/// Default timeout for a clock query.
pub const CLOCK_TIMEOUT: Duration = Duration::from_secs(1);

/// `get_clock` — ask the MCU for its current clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GetClock;

impl McuCommand for GetClock {
    const NAME: &'static str = "get_clock";

    fn args(&self) -> Vec<ArgValue> {
        // No parameters.
        Vec::new()
    }
}

/// `clock clock=%u` — the MCU's clock at the time it handled the request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockState {
    /// Low 32 bits of the MCU clock.
    pub clock: u32,
}

impl McuResponse for ClockState {
    const NAME: &'static str = "clock";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            clock: params.get_u32("clock")?,
        })
    }
}

/// Reading the firmware clock.
///
/// Implemented for a real MCU by [`McuClock`]. Kept as a trait so callers can
/// depend on the capability rather than on the transport, and so tests can
/// substitute a fake without a device.
pub trait ClockSync {
    /// Read the current clock value.
    ///
    /// # Errors
    /// Returns [`McuError`] when the MCU is not identified, does not implement
    /// the message, or fails to answer in time.
    fn get_clock(&self) -> impl Future<Output = Result<ClockState, McuError>> + Send;
}

/// [`ClockSync`] backed by an MCU.
///
/// The handle is shared rather than cloned: the `Mcu` owns the device connection
/// and shutting it down is tied to dropping the last reference.
#[derive(Debug)]
pub struct McuClock {
    mcu: Arc<Mcu>,
    timeout: Duration,
}

impl McuClock {
    /// Create a [`ClockSync`] for `mcu`, using [`CLOCK_TIMEOUT`].
    pub fn new(mcu: Arc<Mcu>) -> Self {
        Self {
            mcu,
            timeout: CLOCK_TIMEOUT,
        }
    }

    /// Override the query timeout.
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// The MCU this module talks to.
    pub fn mcu(&self) -> &Arc<Mcu> {
        &self.mcu
    }
}

impl ClockSync for McuClock {
    fn get_clock(&self) -> impl Future<Output = Result<ClockState, McuError>> + Send {
        let mcu = Arc::clone(&self.mcu);
        let timeout = self.timeout;
        async move {
            mcu.call_msg::<GetClock, ClockState>(&GetClock, timeout)
                .await
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::test::{MappingEntry, TestDevice};
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::Dictionary;
    use crate::core::klippy::msg::proto::Payload;
    use serde_json::json;

    /// Only the messages this module needs, as the firmware would publish them.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {"get_clock": 5},
            "responses": {"clock clock=%u": 18},
            "config": {"CLOCK_FREQ": 20000000}
        }))
        .unwrap()
    }

    /// Payload bytes for a message built from its firmware member order.
    fn payload(values: &[ArgValue]) -> Vec<u8> {
        let mut out = Payload::new();
        for value in values {
            out.push_value(value).unwrap();
        }
        out.into_raw()
    }

    /// An identified MCU whose only exchange is `get_clock` → `clock`.
    fn mcu_answering(mappings: Vec<MappingEntry>) -> Arc<Mcu> {
        let mcu = Mcu::for_test("test_mcu", Interface::new(TestDevice::new(mappings)));
        mcu.install_dictionary(dictionary()).unwrap();
        Arc::new(mcu)
    }

    /// The single mapping for one `get_clock` exchange.
    fn clock_exchange(clock: u32) -> Vec<MappingEntry> {
        vec![MappingEntry {
            input: Frame::new(0, payload(&[ArgValue::UInt8(5)])),
            outputs: vec![Frame::new(
                0,
                payload(&[ArgValue::UInt8(18), ArgValue::UInt32(clock)]),
            )],
        }]
    }

    #[tokio::test]
    async fn test_get_clock_reads_firmware_clock() {
        let mcu = mcu_answering(clock_exchange(0x1234_5678));
        let clock = McuClock::new(Arc::clone(&mcu));

        let state = clock.get_clock().await.unwrap();

        assert_eq!(state, ClockState { clock: 0x1234_5678 });
        assert_eq!(clock.mcu().name(), "test_mcu");
    }

    #[tokio::test]
    async fn test_get_clock_wraps_around_at_32_bits() {
        let mcu = mcu_answering(clock_exchange(0xffff_ffff));
        let clock = McuClock::new(mcu);

        assert_eq!(clock.get_clock().await.unwrap().clock, u32::MAX);
    }

    #[tokio::test]
    async fn test_get_clock_before_identify_fails() {
        let mcu = Mcu::for_test("test_mcu", Interface::new(TestDevice::new(Vec::new())));
        let clock = McuClock::new(Arc::new(mcu));

        let err = clock.get_clock().await.unwrap_err();

        assert!(matches!(err, McuError::NotIdentified), "{err:?}");
    }

    #[tokio::test]
    async fn test_get_clock_times_out_when_mcu_stays_silent() {
        let mcu = Mcu::for_test("test_mcu", Interface::new(TestDevice::new(Vec::new())));
        mcu.install_dictionary(dictionary()).unwrap();
        let clock = McuClock::new(Arc::new(mcu)).with_timeout(Duration::from_millis(50));

        let err = clock.get_clock().await.unwrap_err();

        assert!(matches!(err, McuError::Call(_)), "{err:?}");
    }

    // -----------------------------------------------------------------------
    // The trait is a usable seam
    // -----------------------------------------------------------------------

    /// A `ClockSync` implementation with no MCU behind it, which is the reason
    /// the capability is expressed as a trait.
    struct FixedClock(u32);

    impl ClockSync for FixedClock {
        fn get_clock(&self) -> impl Future<Output = Result<ClockState, McuError>> + Send {
            let clock = self.0;
            async move { Ok(ClockState { clock }) }
        }
    }

    #[tokio::test]
    async fn test_clock_sync_trait_can_be_implemented_without_an_mcu() {
        let clock = FixedClock(42);
        assert_eq!(clock.get_clock().await.unwrap().clock, 42);
    }
}
