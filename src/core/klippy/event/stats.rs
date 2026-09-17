//! `stats` — periodic scheduler timing from the firmware.
//!
//! One event so far, and it comes from the "Timing and load stats" section of
//! the firmware's `basecmd.c`:
//!
//! | Direction | Message |
//! |---|---|
//! | MCU → host | `stats count=%u sum=%u sumsq=%u` |
//!
//! `stats_update` accumulates the time spent in every scheduled task and, once
//! the clock has advanced five seconds (`timer_from_us(5000000)`), sends one
//! report and resets the counters. `count` is the number of samples, `sum` the
//! total time, and `sumsq` the sum of squares scaled by `STATS_SUMSQ_BASE`; the
//! host turns those into an average and a variance.
//!
//! It is sent with `sendf`, so it is an ordinary encoder in the dictionary's
//! `responses` table. It has no request, which is what makes it an event: it is
//! consumed by a callback registered through [`Mcu::bind_event`], never by
//! `call_msg`.
//!
//! The real consumer is a statistics display. Until that exists,
//! [`register_stats_logging`] subscribes a handler that only logs each report,
//! which is enough to exercise the event path end to end without inventing a
//! consumer.

use crate::core::klippy::cmd::Params;
use crate::core::klippy::event::McuEvent;
use crate::core::klippy::mcu::{Mcu, McuError};
use tracing::info;

/// `stats count=%u sum=%u sumsq=%u` — one timing report from `stats_update`.
///
/// The three counters are raw sums; dividing `sum` by `count` gives the mean
/// task duration, and `sumsq` (scaled by the firmware constant
/// `STATS_SUMSQ_BASE`) the variance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Stats {
    /// Number of scheduled-task samples in this report.
    pub count: u32,
    /// Sum of the sample durations, in clock ticks.
    pub sum: u32,
    /// Sum of the squared durations, divided by `STATS_SUMSQ_BASE`.
    pub sumsq: u32,
}

impl McuEvent for Stats {
    const NAME: &'static str = "stats";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            count: params.get_u32("count")?,
            sum: params.get_u32("sum")?,
            sumsq: params.get_u32("sumsq")?,
        })
    }
}

/// Subscribe the placeholder `stats` handler, which logs each report.
///
/// This is deliberately not the finishing move: the message and the delivery
/// path are the point, and a display can replace this with its own
/// [`Mcu::bind_event`] call later. It is kept as a named function so that swap
/// has one obvious place to happen.
///
/// # Errors
/// Returns [`McuError`] when the MCU is not identified, or when its dictionary
/// has no `stats` response.
pub fn register_stats_logging(mcu: &Mcu) -> Result<(), McuError> {
    mcu.bind_event::<Stats, _>(|stats| {
        info!(
            "stats count={} sum={} sumsq={}",
            stats.count, stats.sum, stats.sumsq
        );
    })
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::event::test_support::{dictionary, parser};
    use crate::core::klippy::msg::proto::ArgValue;
    use std::sync::Arc;

    #[test]
    fn test_stats_decodes_its_parameters_by_name() {
        let parser = parser();

        let encoded = parser
            .encode(
                Stats::NAME,
                &[
                    ArgValue::UInt32(5),
                    ArgValue::UInt32(100),
                    ArgValue::UInt32(2500),
                ],
            )
            .unwrap();
        let decoded = parser.decode(encoded).unwrap();
        let (msg, values) = &decoded[0];
        let stats = Stats::decode(&Params::new(Arc::clone(msg), values)).unwrap();

        assert_eq!(
            stats,
            Stats {
                count: 5,
                sum: 100,
                sumsq: 2500
            }
        );
        // The fixture also covers the rest of the event module's tests.
        assert!(dictionary().message(Stats::NAME).is_some());
    }

    #[test]
    fn test_stats_decode_reports_parameter_mismatches() {
        // Same message name, but `count` declared as a string.
        use crate::core::klippy::msg::parser::Parser;
        let mut parser = Parser::new();
        parser
            .register(0, "stats count=%s sum=%u sumsq=%u")
            .unwrap();
        let encoded = parser
            .encode(
                Stats::NAME,
                &[
                    ArgValue::Str("5".into()),
                    ArgValue::UInt32(100),
                    ArgValue::UInt32(2500),
                ],
            )
            .unwrap();
        let decoded = parser.decode(encoded).unwrap();
        let (msg, values) = &decoded[0];
        let err = Stats::decode(&Params::new(Arc::clone(msg), values)).unwrap_err();

        assert!(matches!(err, McuError::Decode(_)), "{err:?}");
    }
}
