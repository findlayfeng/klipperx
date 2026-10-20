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
//! The consumer is [`register_stats`]: it keeps the latest report as the
//! `last_stats` / `last_stats_time` pair `get_status` reports, so a client can
//! see what the firmware thinks it is doing — and, with two reports, how fast
//! its clock is running.
//!
//! Binding does **not** depend on the dictionary publishing the two conversion
//! constants (`CLOCK_FREQ`, `STATS_SUMSQ_BASE`): a report that arrives is always
//! consumed, because the host's half of it — when it arrived, and what the
//! transport had carried by then — needs no firmware constant. A dictionary
//! without them costs only the tick numbers, which stay zero (see
//! [`tick_constants`]).

use crate::core::klippy::cmd::Params;
use crate::core::klippy::event::McuEvent;
use crate::core::klippy::mcu::{Mcu, McuError, RoundTripTimes};
use crate::core::klippy::printer::Printer;
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;
use tracing::{info, warn};

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

/// The last `stats` report: the scheduler load it turns into, when it arrived,
/// how much traffic the transport had carried by then, and what the line had
/// measured of its own round trip.
///
/// Upstream's `MCUStatsHelper._handle_mcu_stats` (`klippy/mcu.py:931-941`),
/// the numbers its `stats()` collector reports (`klippy/mcu.py:968-975`), and
/// what its `get_status` exposes as `last_stats`:
///
/// - `mcu_awake` — total time awake, in seconds;
/// - `mcu_task_avg` — mean time in scheduled tasks, in seconds;
/// - `mcu_task_stddev` — its standard deviation, in seconds.
///
/// The three are renamed at the wire boundary alone: upstream's own variables
/// are `_mcu_tick_awake` / `_mcu_tick_avg` / `_mcu_tick_stddev`
/// (`klippy/mcu.py:924-926`), but the keys a client reads are the `mcu_task_*`
/// ones above, so that is what this host publishes.
///
/// The rest is this host's, because the report carries only the three numbers
/// above: `eventtime` stamps it with the machine's clock, the byte counts and
/// the round-trip numbers are the transport's, not the firmware's. Upstream
/// fills the same keys from its C serial queue (`serialqueue_get_stats`,
/// `klippy/chelper/serialqueue.c:1040-1050`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LastStats {
    /// Mean task duration, seconds.
    pub mcu_task_avg: f64,
    /// Standard deviation of the task duration, seconds.
    pub mcu_task_stddev: f64,
    /// Total time spent awake, seconds.
    pub mcu_awake: f64,
    /// When the report arrived, in [`Printer::eventtime`] seconds.
    ///
    /// A counter delta means nothing without the interval it was measured
    /// over, so the numbers here are only interpretable next to this.
    pub eventtime: f64,
    /// Bytes the transport had read when the report arrived.
    ///
    /// Cumulative since it was opened, and the link's traffic as the host sees
    /// it: a mux unit's header and a tunnel block's bytes are in it. Upstream's
    /// `bytes_read`.
    pub bytes_read: u64,
    /// Bytes it had written: upstream's `bytes_write`.
    pub bytes_write: u64,
    /// The line's round-trip numbers, `None` until one has been measured.
    ///
    /// All three or none: upstream's `srtt`/`rttvar`/`rto` are one estimator's
    /// output, and before the first sample there is no `srtt` to report
    /// (`serialqueue.c:222`) — so `last_stats` leaves the keys out rather than
    /// publish the `MIN_RTO` floor as a measurement.
    pub round_trip: Option<RoundTripTimes>,
}

/// What the host knows about a `stats` report besides the firmware's own
/// numbers.
///
/// Gathered at dispatch, on the receive task, so the reader inside
/// [`from_report`](LastStats::from_report) needs no clock or transport handle
/// of its own and the numbers below are a single point in time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReportContext {
    /// [`Printer::eventtime`] seconds when the report arrived.
    pub eventtime: f64,
    /// Bytes the transport had read at that moment.
    pub bytes_read: u64,
    /// Bytes it had written at that moment.
    pub bytes_write: u64,
    /// What the line had measured of its round trip at that moment.
    pub round_trip: Option<RoundTripTimes>,
}

impl LastStats {
    /// Turn one raw report into the three numbers.
    ///
    /// `mcu_freq` is the firmware's `CLOCK_FREQ` and `stats_sumsq_base` its
    /// `STATS_SUMSQ_BASE`; both are firmware constants read at identify.
    /// `context` is the host's side of the same moment. A report with
    /// `count == 0` (or a non-positive frequency) has nothing to average, so it
    /// leaves the previous values in place.
    pub fn from_report(
        report: &Stats,
        mcu_freq: f64,
        stats_sumsq_base: f64,
        context: ReportContext,
    ) -> Option<Self> {
        if report.count == 0 || mcu_freq <= 0.0 {
            return None;
        }
        let count = f64::from(report.count);
        let sum = f64::from(report.sum);
        let c = 1.0 / (count * mcu_freq);
        let tick_sumsq = f64::from(report.sumsq) * stats_sumsq_base;
        let diff = count * tick_sumsq - sum * sum;
        Some(Self {
            mcu_task_avg: sum * c,
            mcu_task_stddev: c * diff.max(0.0).sqrt(),
            mcu_awake: sum / mcu_freq,
            eventtime: context.eventtime,
            bytes_read: context.bytes_read,
            bytes_write: context.bytes_write,
            round_trip: context.round_trip,
        })
    }

    /// The report's traffic half alone.
    ///
    /// Used when the dictionary does not publish the two conversion constants:
    /// the three counters cannot be turned into seconds, so the task numbers
    /// are zero, while everything the host measured itself — the arrival time,
    /// the transport's byte counts and the line's round trip — is real. That
    /// is the half a port's utilization is computed from, so it must not be
    /// lost with the ticks.
    pub fn traffic_only(context: ReportContext) -> Self {
        Self {
            mcu_task_avg: 0.0,
            mcu_task_stddev: 0.0,
            mcu_awake: 0.0,
            eventtime: context.eventtime,
            bytes_read: context.bytes_read,
            bytes_write: context.bytes_write,
            round_trip: context.round_trip,
        }
    }

    /// The load, the traffic and the round trip as `last_stats`.
    ///
    /// Every value is a number `serde_json` renders without an exponent and,
    /// for the second-denominated ones, with a decimal point — the shape
    /// upstream's `{float(v) if '.' in v else int(v)}` parse assumes
    /// (`klippy/mcu.py:974`): `srtt=%.3f` there, the same seconds here.
    /// `srtt`/`rttvar`/`rto` are absent until a round trip has been measured,
    /// which is also how upstream's report reads (`serialqueue.c:222`).
    pub fn to_json(&self) -> serde_json::Value {
        let mut stats = serde_json::json!({
            "mcu_task_avg": self.mcu_task_avg,
            "mcu_task_stddev": self.mcu_task_stddev,
            "mcu_awake": self.mcu_awake,
            "bytes_read": self.bytes_read,
            "bytes_write": self.bytes_write,
        });
        if let (Some(round_trip), Some(fields)) = (self.round_trip, stats.as_object_mut()) {
            fields.insert("srtt".to_string(), seconds(round_trip.srtt));
            fields.insert("rttvar".to_string(), seconds(round_trip.rttvar));
            fields.insert("rto".to_string(), seconds(round_trip.rto));
        }
        stats
    }
}

/// A duration as upstream reports it: seconds, as a JSON number
/// (`serialqueue_get_stats` prints `srtt=%.3f rttvar=%.3f rto=%.3f`,
/// `serialqueue.c:1043`).
///
/// The three decimals there are printf's, not the quantity's resolution: the
/// numbers upstream keeps are `double` seconds, and so is this, so the
/// microseconds a `Duration` holds are kept rather than rounded away.
fn seconds(duration: Duration) -> serde_json::Value {
    serde_json::json!(duration.as_secs_f64())
}

/// Subscribe a handler that keeps the latest [`LastStats`] in `slot`.
///
/// The value is what `get_status` reports as `last_stats` and its
/// `eventtime` as `last_stats_time`; the log line stays so the event path is
/// still visible in DEBUG builds. `printer` is only read for its clock —
/// [`Printer::eventtime`] — and a machine that is already gone stamps nothing,
/// which is why the slot then keeps its previous report.
///
/// The conversion constants are read here, from the same dictionary the bind
/// resolves `stats` through, so no caller can gate the bind on them: a bound
/// handler that never fires would leave the firmware's 5-second report logged
/// as an unhandled message and `last_stats` empty. A dictionary that publishes
/// neither constant still gets the handler, with the ticks zero.
///
/// A firmware that does not declare `stats` at all is not an error — there is
/// simply nothing to subscribe to — so the slot stays empty and the reason is
/// logged once.
///
/// # Errors
/// Returns [`McuError`] when the MCU is not identified, or when its dictionary
/// declares a `stats` response that cannot be bound.
pub fn register_stats(
    mcu: &Arc<Mcu>,
    slot: Arc<Mutex<Option<LastStats>>>,
    printer: Weak<Printer>,
) -> Result<(), McuError> {
    if !mcu.has_message(Stats::NAME) {
        warn!(
            "MCU '{}' does not report `stats`; last_stats stays empty",
            mcu.name()
        );
        return Ok(());
    }
    let constants = tick_constants(mcu);
    // A `Weak`, like the printer: the callback table belongs to the session and
    // a handler holding its own `Mcu` strongly would be another
    // `Mcu → events → handler → Mcu` edge for `clear_events` to break.
    let connection = Arc::downgrade(mcu);
    mcu.bind_event::<Stats, _>(move |stats| {
        info!(
            "stats count={} sum={} sumsq={}",
            stats.count, stats.sum, stats.sumsq
        );
        let Some(printer) = printer.upgrade() else {
            return;
        };
        let Some(connection) = connection.upgrade() else {
            return;
        };
        let (bytes_read, bytes_write) = connection.wire_bytes();
        let context = ReportContext {
            eventtime: printer.eventtime(),
            bytes_read,
            bytes_write,
            round_trip: connection.wire_round_trip(),
        };
        let last = match constants {
            Some(constants) => {
                LastStats::from_report(&stats, constants.freq, constants.sumsq_base, context)
            }
            None => Some(LastStats::traffic_only(context)),
        };
        if let Some(last) = last {
            *slot.lock().unwrap_or_else(|poison| poison.into_inner()) = Some(last);
        }
    })
}

/// The two firmware constants a report's counters are converted with.
///
/// Both are dictionary constants read at identify: `CLOCK_FREQ` is the clock the
/// counters are in, `STATS_SUMSQ_BASE` the scale `sumsq` was divided by. Neither
/// is available from anywhere else, so a dictionary that publishes neither (or
/// only one) leaves the counters unreadable.
#[derive(Debug, Clone, Copy)]
struct TickConstants {
    /// `CLOCK_FREQ`, in Hz.
    freq: f64,
    /// `STATS_SUMSQ_BASE`.
    sumsq_base: f64,
}

/// The conversion above, or `None` with one warning naming what is missing.
///
/// Warned here rather than per report: the constants belong to the session's
/// dictionary, so an absence cannot change until the next connect, and a report
/// arrives every five seconds.
fn tick_constants(mcu: &Mcu) -> Option<TickConstants> {
    // Not identified at all: `bind_event` is the one that reports that, and
    // naming missing constants first would blame the wrong thing.
    let dictionary = mcu.dictionary()?;
    let freq = dictionary.constant_f64("CLOCK_FREQ");
    let sumsq_base = dictionary.constant_f64("STATS_SUMSQ_BASE");
    match (freq, sumsq_base) {
        (Some(freq), Some(sumsq_base)) => Some(TickConstants { freq, sumsq_base }),
        _ => {
            let missing: Vec<&str> = [
                ("CLOCK_FREQ", freq.is_none()),
                ("STATS_SUMSQ_BASE", sumsq_base.is_none()),
            ]
            .into_iter()
            .filter_map(|(name, absent)| absent.then_some(name))
            .collect();
            warn!(
                "MCU '{}' dictionary has no {}; `stats` is still consumed, but \
                 mcu_task_avg/mcu_task_stddev/mcu_awake stay 0 — last_stats_time, \
                 bytes_read, bytes_write and srtt/rttvar/rto are the host's own and \
                 stay valid",
                mcu.name(),
                missing.join(" or ")
            );
            None
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::cmd::uptime::GetUptime;
    use crate::core::klippy::event::test_support::{dictionary, frame, mcu, parser};
    use crate::core::klippy::interface::devices::frame_mock::MappingEntry;
    use crate::core::klippy::msg::proto::ArgValue;
    use crate::core::klippy::reactor::ManualReactor;
    use crate::core::klippy::test_util::wait_until;
    use std::io::Write;
    use std::sync::Arc;
    use std::time::Duration;
    use tracing_subscriber::layer::SubscriberExt;

    /// A device answering `get_uptime` with one `stats` report.
    ///
    /// A report is unsolicited on the wire; answering an unrelated command is
    /// only how a test gets one to arrive (`event/mod.rs` does the same). The id
    /// is the firmware's own — `stats count=%u sum=%u sumsq=%u: -1` in a real
    /// dictionary, a signed VLQ byte.
    fn device_answering_with_a_stats_report() -> Vec<MappingEntry> {
        vec![MappingEntry {
            input: frame(0, &[ArgValue::UInt8(4)]),
            outputs: vec![frame(
                0,
                &[
                    ArgValue::Int16(-1),
                    ArgValue::UInt32(5),
                    ArgValue::UInt32(100),
                    ArgValue::UInt32(2500),
                ],
            )],
        }]
    }

    /// Wait (bounded) until the slot holds a report.
    async fn a_report_arrives(slot: &Arc<Mutex<Option<LastStats>>>) -> LastStats {
        tokio::time::timeout(
            Duration::from_secs(5),
            wait_until(
                Duration::from_secs(1),
                Duration::from_millis(5),
                || slot.lock().unwrap_or_else(|p| p.into_inner()).is_some(),
                "a `stats` report must arrive",
            ),
        )
        .await
        .expect("the wait is bounded");
        slot.lock()
            .unwrap_or_else(|p| p.into_inner())
            .expect("just waited for it")
    }

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
        let context = ReportContext {
            eventtime: 12.5,
            bytes_read: 1_024,
            bytes_write: 2_048,
            round_trip: None,
        };
        let last = LastStats::from_report(&report, 72_000_000.0, 4.0, context).unwrap();

        // c = 1/(100 * 72e6); avg = sum * c; awake = sum / 72e6.
        assert_eq!(last.eventtime, 12.5, "stamped with the arrival time");
        assert_eq!((last.bytes_read, last.bytes_write), (1_024, 2_048));
        assert!((last.mcu_task_avg - 3000.0 / (100.0 * 72_000_000.0)).abs() < 1e-18);
        assert!((last.mcu_awake - 3000.0 / 72_000_000.0).abs() < 1e-18);
        // stddev = c * sqrt(count*sumsq*base - sum^2).
        let expected =
            (100.0 * 250_000.0 * 4.0 - 3000.0f64.powi(2)).sqrt() / (100.0 * 72_000_000.0);
        assert!((last.mcu_task_stddev - expected).abs() < 1e-18);
    }

    /// The keys a client reads are upstream's, not the names of the variables
    /// behind them (`klippy/mcu.py:924-926` vs `:969`).
    #[test]
    fn test_last_stats_publishes_upstreams_task_keys_and_not_its_own() {
        let last = LastStats {
            mcu_task_avg: 1.5e-6,
            mcu_task_stddev: 2.5e-7,
            mcu_awake: 0.25,
            eventtime: 12.5,
            bytes_read: 1_024,
            bytes_write: 2_048,
            round_trip: None,
        };
        let json = last.to_json();

        assert_eq!(json["mcu_task_avg"], 1.5e-6);
        assert_eq!(json["mcu_task_stddev"], 2.5e-7);
        assert_eq!(json["mcu_awake"], 0.25);
        for stale in ["mcu_tick_avg", "mcu_tick_stddev", "mcu_tick_awake"] {
            assert!(json.get(stale).is_none(), "{stale} is not a key: {json}");
        }
    }

    /// The round trip is reported in **seconds**, the unit upstream prints
    /// (`serialqueue.c:1042-1043`, with `MIN_RTO 0.025` / `MAX_RTO 5.000` for
    /// scale at `:107-108`) — not in milliseconds or microseconds.
    #[test]
    fn test_last_stats_publishes_the_round_trip_in_seconds() {
        let last = LastStats {
            mcu_task_avg: 0.0,
            mcu_task_stddev: 0.0,
            mcu_awake: 0.0,
            eventtime: 12.5,
            bytes_read: 0,
            bytes_write: 0,
            round_trip: Some(RoundTripTimes {
                srtt: Duration::from_millis(25),
                rttvar: Duration::from_micros(2_500),
                rto: Duration::from_micros(35_000),
            }),
        };
        let json = last.to_json();

        assert_eq!(json["srtt"], 0.025);
        assert_eq!(json["rttvar"], 0.0025);
        assert_eq!(json["rto"], 0.035);
        // A dotted number, which is what upstream's `'.' in v` parse keys off
        // (`klippy/mcu.py:974`).
        for key in ["srtt", "rttvar", "rto"] {
            let text = json[key].to_string();
            assert!(text.contains('.'), "{key} = {text} reads as a float");
        }
    }

    /// No sample yet: the three keys are absent, not zero — the floor a wait
    /// starts at is not a measurement (`serialqueue.c:222`, `srtt == 0`).
    #[test]
    fn test_last_stats_omits_the_round_trip_until_one_is_measured() {
        let report = Stats {
            count: 100,
            sum: 3000,
            sumsq: 250_000,
        };
        let last = LastStats::from_report(
            &report,
            72_000_000.0,
            4.0,
            ReportContext {
                eventtime: 12.5,
                bytes_read: 0,
                bytes_write: 0,
                round_trip: None,
            },
        )
        .unwrap();
        let json = last.to_json();

        for key in ["srtt", "rttvar", "rto"] {
            assert!(json.get(key).is_none(), "{key} before a sample: {json}");
        }
        // The rest of the report is there either way.
        assert!(json["mcu_awake"].as_f64().unwrap() > 0.0);
    }

    #[test]
    fn test_last_stats_ignores_an_empty_report() {
        let report = Stats {
            count: 0,
            sum: 0,
            sumsq: 0,
        };
        assert!(LastStats::from_report(
            &report,
            72_000_000.0,
            4.0,
            ReportContext {
                eventtime: 12.5,
                bytes_read: 0,
                bytes_write: 0,
                round_trip: None,
            }
        )
        .is_none());
    }

    /// A dictionary without the conversion constants still consumes the report.
    ///
    /// Binding must not depend on them: a handler gated on the constants left the
    /// firmware's five-second report unhandled on a real board, with the ticks
    /// unreadable _and_ the traffic counters (which need no firmware constant)
    /// frozen with it.
    #[tokio::test]
    async fn test_a_report_without_the_tick_constants_still_stamps_the_traffic() {
        // The fixture dictionary publishes neither `CLOCK_FREQ` nor
        // `STATS_SUMSQ_BASE`.
        let mcu = mcu(device_answering_with_a_stats_report());
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let slot = Arc::new(Mutex::new(None));
        register_stats(&mcu, Arc::clone(&slot), Arc::downgrade(&printer)).unwrap();

        mcu.send_msg(&GetUptime).unwrap();
        let last = a_report_arrives(&slot).await;

        assert_eq!(
            (last.mcu_task_avg, last.mcu_task_stddev, last.mcu_awake),
            (0.0, 0.0, 0.0),
            "nothing to divide the counters by"
        );
        assert_eq!(last.eventtime, printer.eventtime());
        let (bytes_read, bytes_write) = mcu.wire_bytes();
        assert_eq!(
            (last.bytes_read, last.bytes_write),
            (bytes_read, bytes_write),
            "the transport's own counters are real either way"
        );
    }

    /// The round trip is the host's own measurement, so a dictionary without
    /// the conversion constants loses the task numbers and nothing else.
    #[test]
    fn test_traffic_only_keeps_the_round_trip_it_was_handed() {
        let times = RoundTripTimes {
            srtt: Duration::from_millis(25),
            rttvar: Duration::from_micros(2_500),
            rto: Duration::from_micros(35_000),
        };
        let last = LastStats::traffic_only(ReportContext {
            eventtime: 12.5,
            bytes_read: 1_024,
            bytes_write: 2_048,
            round_trip: Some(times),
        });

        assert_eq!(last.round_trip, Some(times));
        assert_eq!(last.to_json()["rto"], 0.035);
    }

    /// The missing constants are named once, at registration — not on every
    /// report.
    #[tokio::test]
    async fn test_missing_tick_constants_are_reported_once_not_per_report() {
        let mcu = mcu(device_answering_with_a_stats_report());
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let slot = Arc::new(Mutex::new(None));

        let capture = LogCapture::default();
        let subscriber = tracing_subscriber::registry()
            .with(tracing_subscriber::filter::LevelFilter::WARN)
            .with(
                tracing_subscriber::fmt::layer()
                    .with_writer(capture.clone())
                    .with_ansi(false),
            );
        let _guard = tracing::subscriber::set_default(subscriber);

        register_stats(&mcu, Arc::clone(&slot), Arc::downgrade(&printer)).unwrap();
        mcu.send_msg(&GetUptime).unwrap();
        a_report_arrives(&slot).await;
        // Reports repeat every five seconds on a real board; a second one adds
        // nothing to the log.
        let logged = capture.text();
        assert_eq!(logged.matches("CLOCK_FREQ").count(), 1, "{logged}");
        assert_eq!(logged.matches("STATS_SUMSQ_BASE").count(), 1, "{logged}");
        assert!(logged.contains("stay valid"), "{logged}");
    }

    /// A byte sink for the fmt layer above; `set_default` keeps the capture off
    /// the process-wide subscriber, so tests stay independent of each other.
    #[derive(Clone, Default)]
    struct LogCapture(Arc<Mutex<Vec<u8>>>);

    impl LogCapture {
        fn text(&self) -> String {
            let bytes = self
                .0
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .clone();
            String::from_utf8(bytes).expect("tracing writes UTF-8")
        }
    }

    impl Write for LogCapture {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.0
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    impl<'a> tracing_subscriber::fmt::MakeWriter<'a> for LogCapture {
        type Writer = LogCapture;

        fn make_writer(&'a self) -> Self::Writer {
            self.clone()
        }
    }
}
