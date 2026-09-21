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
use std::sync::{Arc, Mutex};
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

/// The scheduler load a `stats` report turns into.
///
/// Upstream's `MCUStatsHelper._handle_mcu_stats` (`klippy/mcu.py:931-941`),
/// the three numbers its `stats()` collector reports and its `get_status`
/// exposes as `last_stats`:
///
/// - `mcu_tick_avg` — mean time in scheduled tasks, in seconds;
/// - `mcu_tick_stddev` — its standard deviation, in seconds;
/// - `mcu_tick_awake` — total time awake, in seconds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LastStats {
    /// Mean task duration, seconds.
    pub mcu_tick_avg: f64,
    /// Standard deviation of the task duration, seconds.
    pub mcu_tick_stddev: f64,
    /// Total time spent awake, seconds.
    pub mcu_tick_awake: f64,
}

impl LastStats {
    /// Turn one raw report into the three numbers.
    ///
    /// `mcu_freq` is the firmware's `CLOCK_FREQ` and `stats_sumsq_base` its
    /// `STATS_SUMSQ_BASE`; both are firmware constants read at identify. A
    /// report with `count == 0` (or a non-positive frequency) has nothing to
    /// average, so it leaves the previous values in place.
    pub fn from_report(report: &Stats, mcu_freq: f64, stats_sumsq_base: f64) -> Option<Self> {
        if report.count == 0 || mcu_freq <= 0.0 {
            return None;
        }
        let count = f64::from(report.count);
        let sum = f64::from(report.sum);
        let c = 1.0 / (count * mcu_freq);
        let tick_sumsq = f64::from(report.sumsq) * stats_sumsq_base;
        let diff = count * tick_sumsq - sum * sum;
        Some(Self {
            mcu_tick_avg: sum * c,
            mcu_tick_stddev: c * diff.max(0.0).sqrt(),
            mcu_tick_awake: sum / mcu_freq,
        })
    }

    /// The three numbers as `last_stats`.
    pub fn to_json(&self) -> serde_json::Value {
        serde_json::json!({
            "mcu_tick_avg": self.mcu_tick_avg,
            "mcu_tick_stddev": self.mcu_tick_stddev,
            "mcu_tick_awake": self.mcu_tick_awake,
        })
    }
}

/// Subscribe a handler that keeps the latest [`LastStats`] in `slot`.
///
/// This replaces the placeholder that only logged each report: the value is
/// what `get_status` reports as `last_stats`, and the log line stays so the
/// event path is still visible in DEBUG builds.
///
/// # Errors
/// Returns [`McuError`] when the MCU is not identified, or when its dictionary
/// has no `stats` response.
pub fn register_stats(
    mcu: &Mcu,
    mcu_freq: f64,
    stats_sumsq_base: f64,
    slot: Arc<Mutex<Option<LastStats>>>,
) -> Result<(), McuError> {
    mcu.bind_event::<Stats, _>(move |stats| {
        info!(
            "stats count={} sum={} sumsq={}",
            stats.count, stats.sum, stats.sumsq
        );
        if let Some(last) = LastStats::from_report(&stats, mcu_freq, stats_sumsq_base) {
            *slot.lock().unwrap_or_else(|poison| poison.into_inner()) = Some(last);
        }
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

    #[test]
    fn test_last_stats_matches_upstreams_arithmetic() {
        // A report over 100 samples costing 3000 ticks total, with the squares
        // scaled by 4; at a 72 MHz clock. `sumsq` is large enough that the
        // variance is positive (the squares must dominate the square of the
        // sum, as they do for real task timings).
        let report = Stats {
            count: 100,
            sum: 3000,
            sumsq: 250_000,
        };
        let last = LastStats::from_report(&report, 72_000_000.0, 4.0).unwrap();

        // c = 1/(100 * 72e6); avg = sum * c; awake = sum / 72e6.
        assert!((last.mcu_tick_avg - 3000.0 / (100.0 * 72_000_000.0)).abs() < 1e-18);
        assert!((last.mcu_tick_awake - 3000.0 / 72_000_000.0).abs() < 1e-18);
        // stddev = c * sqrt(count*sumsq*base - sum^2).
        let expected =
            (100.0 * 250_000.0 * 4.0 - 3000.0f64.powi(2)).sqrt() / (100.0 * 72_000_000.0);
        assert!((last.mcu_tick_stddev - expected).abs() < 1e-18);
    }

    #[test]
    fn test_last_stats_ignores_an_empty_report() {
        let report = Stats {
            count: 0,
            sum: 0,
            sumsq: 0,
        };
        assert!(LastStats::from_report(&report, 72_000_000.0, 4.0).is_none());
    }
}
