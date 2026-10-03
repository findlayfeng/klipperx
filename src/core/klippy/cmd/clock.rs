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

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::{Mcu, McuError};
use crate::core::klippy::msg::proto::ArgValue;
use crate::core::klippy::reactor::Reactor;
use std::future::Future;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Instant;
use tokio::time::Duration;

/// Default timeout for a clock query.
pub const CLOCK_TIMEOUT: Duration = Duration::from_secs(1);

/// How fast an old minimum round-trip time is allowed to age, in seconds of
/// "credit" per second since it was seen (`klippy/clocksync.py:8`).
const RTT_AGE: f64 = 0.000010 / (60. * 60.);

/// EWMA weight of each new sample in the clock/time regression
/// (`klippy/clocksync.py:9`).
const DECAY: f64 = 1. / 30.;

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

/// The regression that maps host time to the MCU clock.
///
/// Upstream's `ClockSync` (`klippy/clocksync.py:12-173`) fits the MCU clock
/// against the system time at which each `get_clock` was sent, and keeps the
/// best round-trip time as a lower bound on the latency. This is the part that
/// carries no transport: it is handed the three numbers a sample produces —
/// when it was sent, when it came back, and the clock it reported — and exposes
/// the conversions.
///
/// `print_time` and the MCU clock are the same time base:
/// [`ClockEstimator::print_time_to_clock`] is a plain multiply, exactly as
/// upstream's (`klippy/clocksync.py:137-138`). What the regression supplies is
/// the mapping from the **system** clock (the reactor's [`monotonic`]) to that
/// base, which is how the host learns how much motion the MCU still has
/// buffered.
///
/// [`monotonic`]: Reactor::monotonic
#[derive(Debug, Clone)]
pub struct ClockEstimator {
    mcu_freq: f64,
    /// 32-bit to 64-bit clock extension; updated on every sample.
    last_clock: i64,
    /// EWMA linear regression of the MCU clock against the system sent time.
    time_avg: f64,
    time_variance: f64,
    clock_avg: f64,
    clock_covariance: f64,
    prediction_variance: f64,
    last_prediction_time: f64,
    /// The best (half) round-trip time seen, and when it was seen.
    min_half_rtt: f64,
    min_rtt_time: f64,
    /// The system-time → clock mapping: `clock_at_sample` at
    /// `clock_sample_time`, advancing at `clock_freq` ticks per second.
    clock_sample_time: f64,
    clock_at_sample: f64,
    clock_freq: f64,
    /// Clock queries in flight (`klippy/clocksync.py:22`).
    queries_pending: u32,
}

impl ClockEstimator {
    /// An estimator whose regression has not been seeded yet.
    ///
    /// The initial values mirror upstream's `ClockSync.__init__`
    /// (`klippy/clocksync.py:12-32`): everything zero except the minimum
    /// round-trip time, which starts at "unseen".
    pub fn new(mcu_freq: f64) -> Self {
        Self {
            mcu_freq,
            last_clock: 0,
            time_avg: 0.0,
            time_variance: 0.0,
            clock_avg: 0.0,
            clock_covariance: 0.0,
            prediction_variance: 0.0,
            last_prediction_time: 0.0,
            min_half_rtt: 999_999_999.9,
            min_rtt_time: 0.0,
            clock_sample_time: 0.0,
            clock_at_sample: 0.0,
            clock_freq: 0.0,
            queries_pending: 0,
        }
    }

    /// The firmware clock frequency this estimator converts with.
    pub fn mcu_freq(&self) -> f64 {
        self.mcu_freq
    }

    /// Set the firmware clock frequency, once identify has read the dictionary.
    ///
    /// A value of zero (or less) is ignored so a not-yet-identified MCU cannot
    /// make the conversions divide by zero.
    pub fn set_mcu_freq(&mut self, mcu_freq: f64) {
        if mcu_freq > 0.0 {
            self.mcu_freq = mcu_freq;
        }
    }

    /// The last extended (64-bit) clock value.
    pub fn last_clock(&self) -> i64 {
        self.last_clock
    }

    /// Queries still waiting for a response.
    pub fn queries_pending(&self) -> u32 {
        self.queries_pending
    }

    /// Note that a `get_clock` request went out.
    pub fn note_query_sent(&mut self) {
        self.queries_pending += 1;
    }

    /// Whether the estimate is fresh enough to trust
    /// (`klippy/clocksync.py:156-157`): upstream stops once more than four
    /// queries are outstanding.
    pub fn is_active(&self) -> bool {
        self.queries_pending <= 4
    }

    /// Seed the regression from a `get_uptime` sample, as upstream's `connect`
    /// does (`klippy/clocksync.py:33-51`).
    ///
    /// `get_uptime` reports a 64-bit clock with none of `get_clock`'s
    /// wrap-around, so this fixes `last_clock` *and* gives the regression its
    /// first point to grow from.
    pub fn seed(&mut self, sent_time: f64, clock: i64) {
        self.last_clock = clock;
        self.clock_avg = clock as f64;
        self.time_avg = sent_time;
        self.clock_sample_time = sent_time;
        self.clock_at_sample = clock as f64;
        self.clock_freq = self.mcu_freq;
        self.prediction_variance = (0.001 * self.mcu_freq).powi(2);
        // Upstream sets this just before its first samples so that none of
        // them is mistaken for an outlier (`klippy/clocksync.py:46`).
        self.last_prediction_time = -9999.0;
    }

    /// Fold one `get_clock` sample in.
    ///
    /// Returns whether the sample was used: upstream discards the ones that
    /// look like an outlier rather than letting them drag the regression
    /// (`klippy/clocksync.py:68-100`). `clock32` is the firmware's low 32 bits;
    /// the extension to 64 bits happens here.
    pub fn update(&mut self, sent_time: f64, receive_time: f64, clock32: u32) -> bool {
        self.queries_pending = 0;
        // Extend the clock to 64 bits (`_handle_clock`).
        self.last_clock += (i64::from(clock32) - self.last_clock) & 0xffff_ffff;
        let clock = self.last_clock;
        if !self.update_regression(sent_time, clock) {
            return false;
        }
        let new_freq = if self.time_variance > 0.0 {
            self.clock_covariance / self.time_variance
        } else {
            self.mcu_freq
        };
        self.update_best_rtt(sent_time, receive_time);
        // Upstream also hands the sender a release time here
        // (`serial.set_clock_est(new_freq, time_avg + TRANSMIT_EXTRA,
        // clock_avg - 3*stddev)`); with no serial queue to pace, only the
        // estimate the rest of the host reads is kept.
        self.clock_sample_time = self.time_avg + self.min_half_rtt;
        self.clock_at_sample = self.clock_avg;
        self.clock_freq = new_freq;
        true
    }

    /// The EWMA regression step (`klippy/clocksync.py:68-100`).
    fn update_regression(&mut self, sent_time: f64, clock: i64) -> bool {
        let clock = clock as f64;
        let old_freq = self.clock_freq;
        let exp_clock = (sent_time - self.time_avg) * old_freq + self.clock_avg;
        let clock_diff2 = (clock - exp_clock).powi(2);
        if clock_diff2 > 25.0 * self.prediction_variance
            && clock_diff2 > (0.000_500 * self.mcu_freq).powi(2)
        {
            // A sample far from the prediction is either a real disturbance or
            // a delayed message. A clock *ahead* of the prediction, arriving
            // soon after the last good sample, is the latter.
            if clock > exp_clock && sent_time < self.last_prediction_time + 10.0 {
                return false;
            }
            self.prediction_variance = (0.001 * self.mcu_freq).powi(2);
        } else {
            self.last_prediction_time = sent_time;
            self.prediction_variance =
                (1.0 - DECAY) * (self.prediction_variance + clock_diff2 * DECAY);
        }
        let diff_sent_time = sent_time - self.time_avg;
        self.time_avg += DECAY * diff_sent_time;
        self.time_variance =
            (1.0 - DECAY) * (self.time_variance + diff_sent_time * diff_sent_time * DECAY);
        let diff_clock = clock - self.clock_avg;
        self.clock_avg += DECAY * diff_clock;
        self.clock_covariance =
            (1.0 - DECAY) * (self.clock_covariance + diff_sent_time * diff_clock * DECAY);
        true
    }

    /// Track the smallest round-trip time seen, aging the old one
    /// (`klippy/clocksync.py:101-109`).
    fn update_best_rtt(&mut self, sent_time: f64, receive_time: f64) {
        let half_rtt = 0.5 * (receive_time - sent_time);
        let aged_rtt = (sent_time - self.min_rtt_time) * RTT_AGE;
        if half_rtt < self.min_half_rtt + aged_rtt {
            self.min_half_rtt = half_rtt;
            self.min_rtt_time = sent_time;
        }
    }

    /// Seconds of print time to firmware clock ticks
    /// (`klippy/clocksync.py:137-138`).
    pub fn print_time_to_clock(&self, print_time: f64) -> i64 {
        (print_time * self.mcu_freq) as i64
    }

    /// Firmware clock ticks to seconds of print time
    /// (`klippy/clocksync.py:139-140`).
    pub fn clock_to_print_time(&self, clock: i64) -> f64 {
        clock as f64 / self.mcu_freq
    }

    /// The estimated clock at a system time (`klippy/clocksync.py:142-144`).
    pub fn get_clock(&self, eventtime: f64) -> i64 {
        (self.clock_at_sample + (eventtime - self.clock_sample_time) * self.clock_freq) as i64
    }

    /// The regression's anchor: `(sample_time, clock, freq)`
    /// (`ClockSync.clock_est`), for [`SecondarySync`] to align against.
    pub fn clock_est(&self) -> (f64, f64, f64) {
        (
            self.clock_sample_time,
            self.clock_at_sample,
            self.clock_freq,
        )
    }

    /// The estimated print time at a system time
    /// (`klippy/clocksync.py:148-149`).
    pub fn estimated_print_time(&self, eventtime: f64) -> f64 {
        self.clock_to_print_time(self.get_clock(eventtime))
    }

    /// Extend a 32-bit clock reading into the 64-bit domain
    /// (`klippy/clocksync.py:151-155`).
    pub fn clock32_to_clock64(&self, clock32: u32) -> i64 {
        let mut diff = (i64::from(clock32) - self.last_clock) & 0xffff_ffff;
        // A reading more than 2^31 ahead is really behind (wrap-around).
        diff -= (diff & 0x8000_0000) << 1;
        self.last_clock + diff
    }
}

/// A secondary MCU's clock mapping (upstream's `SecondarySync`,
/// `klippy/clocksync.py:177-231`).
///
/// The primary MCU *defines* print time (`print_time = clock / mcu_freq`), so its
/// own crystal drift only rescales it. A secondary has its own crystal, so it is
/// mapped onto the primary's print time as `print_time = clock / freq + offset`,
/// and that mapping is recalibrated periodically: the two crystals drift apart,
/// and without this a secondary's steps would slide by seconds over a long print.
#[derive(Debug, Clone, Copy)]
pub struct SecondarySync {
    /// The print time this MCU's clock zero corresponds to.
    pub offset: f64,
    /// The frequency this MCU's clock is mapped with (may differ from nominal).
    pub freq: f64,
    /// The print time of the last calibration (spacing the next one).
    pub last_sync_time: f64,
}

impl SecondarySync {
    /// A mapping at `freq` with no offset yet.
    pub fn new(freq: f64) -> Self {
        Self {
            offset: 0.0,
            freq,
            last_sync_time: 0.0,
        }
    }

    /// An absolute print time to this MCU's clock.
    pub fn print_time_to_clock(&self, print_time: f64) -> i64 {
        ((print_time - self.offset) * self.freq) as i64
    }

    /// This MCU's clock back to an absolute print time.
    pub fn clock_to_print_time(&self, clock: i64) -> f64 {
        clock as f64 / self.freq + self.offset
    }

    /// Re-align to the primary (`SecondarySync.calibrate_clock`).
    ///
    /// The new mapping is chosen so the secondary's clock at a future sync time
    /// (`sync2`) matches the primary's print time there; calibrating periodically
    /// keeps the two crystals from drifting apart. `print_time` is the caller's
    /// current print time (upstream passes `last_step_gen_time`).
    pub fn calibrate(
        &mut self,
        primary: &ClockEstimator,
        secondary: &ClockEstimator,
        print_time: f64,
        eventtime: f64,
    ) {
        let (ser_time, ser_clock, ser_freq) = primary.clock_est();
        let main_mcu_freq = primary.mcu_freq();
        let est_main_clock = (eventtime - ser_time) * ser_freq + ser_clock;
        let est_print_time = est_main_clock / main_mcu_freq;
        let sync1_print_time = print_time.max(est_print_time);
        let sync2_print_time = (sync1_print_time + 4.0)
            .max(self.last_sync_time)
            .max(print_time + 2.5 * (print_time - est_print_time));
        // The system time `sync2_print_time` falls at, per the primary.
        let sync2_main_clock = sync2_print_time * main_mcu_freq;
        let sync2_sys_time = ser_time + (sync2_main_clock - ser_clock) / ser_freq;
        let sync1_clock = self.print_time_to_clock(sync1_print_time) as f64;
        let sync2_clock = secondary.get_clock(sync2_sys_time) as f64;
        let adjusted_freq = (sync2_clock - sync1_clock) / (sync2_print_time - sync1_print_time);
        self.freq = adjusted_freq;
        self.offset = sync1_print_time - sync1_clock / adjusted_freq;
        self.last_sync_time = sync2_print_time;
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
/// The handle is shared rather than cloned: the `Mcu` owns the device
/// connection. A reconnect ends the old session explicitly (`Mcu::close`,
/// which `reconnect` calls — reference counting is not what waits on here,
/// since this clock is precisely one of the handles that outlives it), and
/// dropping the last reference still closes the connection as the backstop
/// (`Drop` behind `Mcu::close`). Every query also feeds the
/// [`ClockEstimator`], so this is how the host learns the mapping from the
/// reactor's clock to the firmware's — and, through the same round trip, the
/// `Mcu`'s own clock estimate (`Mcu::record_clock_sample`, behind
/// `Mcu::estimated_clock`).
pub struct McuClock {
    mcu: Arc<Mcu>,
    reactor: Arc<dyn Reactor>,
    timeout: Duration,
    estimator: Arc<Mutex<ClockEstimator>>,
}

impl std::fmt::Debug for McuClock {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("McuClock")
            .field("mcu", &self.mcu.name())
            .field("timeout", &self.timeout)
            .finish_non_exhaustive()
    }
}

impl McuClock {
    /// Create a [`ClockSync`] for `mcu`, using [`CLOCK_TIMEOUT`].
    ///
    /// `reactor` is the host clock each sample is timed against; it is what
    /// upstream's serial queue would stamp on every message.
    pub fn new(mcu: Arc<Mcu>, reactor: Arc<dyn Reactor>) -> Self {
        Self {
            mcu,
            reactor,
            timeout: CLOCK_TIMEOUT,
            estimator: Arc::new(Mutex::new(ClockEstimator::new(1.0))),
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

    /// The clock estimate, for reads and for seeding.
    pub fn estimator(&self) -> MutexGuard<'_, ClockEstimator> {
        self.estimator
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// Seed the estimate from a `get_uptime` sample.
    ///
    /// Upstream does this once in `connect` (`klippy/clocksync.py:33-51`),
    /// before the periodic `get_clock` queries take over.
    pub fn seed(&self, sent_time: f64, clock: i64) {
        let mut estimator = self.estimator();
        // The frequency comes from the firmware's dictionary; without it the
        // estimator would keep its placeholder 1 Hz and map a clock of millions
        // of ticks to millions of seconds.
        if let Ok(freq) = self.mcu.clock_freq() {
            estimator.set_mcu_freq(freq);
        }
        estimator.seed(sent_time, clock);
    }

    /// Seconds of print time to firmware clock ticks.
    pub fn print_time_to_clock(&self, print_time: f64) -> i64 {
        self.estimator().print_time_to_clock(print_time)
    }

    /// Firmware clock ticks to seconds of print time.
    pub fn clock_to_print_time(&self, clock: i64) -> f64 {
        self.estimator().clock_to_print_time(clock)
    }

    /// The estimated print time at a system time.
    pub fn estimated_print_time(&self, eventtime: f64) -> f64 {
        self.estimator().estimated_print_time(eventtime)
    }

    /// Extend a 32-bit clock reading into the 64-bit domain.
    pub fn clock32_to_clock64(&self, clock32: u32) -> i64 {
        self.estimator().clock32_to_clock64(clock32)
    }
}

impl ClockSync for McuClock {
    fn get_clock(&self) -> impl Future<Output = Result<ClockState, McuError>> + Send {
        let mcu = Arc::clone(&self.mcu);
        let reactor = Arc::clone(&self.reactor);
        let estimator = Arc::clone(&self.estimator);
        let timeout = self.timeout;
        async move {
            // The send and receive times bracket the exchange, which is what
            // upstream's serial queue stamps on each message (`#sent_time` /
            // `#receive_time`). The `Instant` pair brackets the same exchange
            // on this machine's clock: it is the sample the `Mcu`'s own
            // estimate is fitted from, and it needs host instants, not the
            // reactor's seconds-since-arbitrary-epoch.
            let sent_at = Instant::now();
            let sent_time = reactor.monotonic();
            let state = mcu
                .call_msg::<GetClock, ClockState>(&GetClock, timeout)
                .await?;
            let receive_time = reactor.monotonic();
            let received_at = Instant::now();
            let mut estimator = estimator
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            if let Ok(freq) = mcu.clock_freq() {
                estimator.set_mcu_freq(freq);
            }
            // Only a sample the regression kept counts as a clock reading:
            // `update` discards the ones it reads as delayed messages
            // (`klippy/clocksync.py:68-100`), and a reading that late is just
            // as wrong for the MCU's own estimate, which has no outlier logic
            // of its own.
            if estimator.update(sent_time, receive_time, state.clock) {
                // The extended 64-bit reading, so the two views of the clock
                // never disagree about which wrap-around it is in.
                mcu.record_clock_sample(sent_at, received_at, estimator.last_clock().max(0) as u64);
            }
            Ok(state)
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
    use crate::core::klippy::interface::devices::frame_mock::{FrameMock, MappingEntry};
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::Dictionary;
    use crate::core::klippy::msg::proto::Payload;
    use crate::core::klippy::reactor::ManualReactor;
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
        let mcu = Mcu::for_test("test_mcu", Interface::new(FrameMock::new(mappings)));
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
        let clock = McuClock::new(Arc::clone(&mcu), ManualReactor::shared());

        let state = clock.get_clock().await.unwrap();

        assert_eq!(state, ClockState { clock: 0x1234_5678 });
        assert_eq!(clock.mcu().name(), "test_mcu");
    }

    #[tokio::test]
    async fn test_get_clock_wraps_around_at_32_bits() {
        let mcu = mcu_answering(clock_exchange(0xffff_ffff));
        let clock = McuClock::new(mcu, ManualReactor::shared());

        assert_eq!(clock.get_clock().await.unwrap().clock, u32::MAX);
    }

    #[tokio::test]
    async fn test_get_clock_before_identify_fails() {
        let mcu = Mcu::for_test("test_mcu", Interface::new(FrameMock::new(Vec::new())));
        let clock = McuClock::new(Arc::new(mcu), ManualReactor::shared());

        let err = clock.get_clock().await.unwrap_err();

        assert!(matches!(err, McuError::NotIdentified), "{err:?}");
    }

    #[tokio::test]
    async fn test_get_clock_times_out_when_mcu_stays_silent() {
        let mcu = Mcu::for_test("test_mcu", Interface::new(FrameMock::new(Vec::new())));
        mcu.install_dictionary(dictionary()).unwrap();
        let clock = McuClock::new(Arc::new(mcu), ManualReactor::shared())
            .with_timeout(Duration::from_millis(50));

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

    // -----------------------------------------------------------------------
    // The clock estimate
    // -----------------------------------------------------------------------

    #[test]
    fn test_print_time_and_clock_round_trip() {
        let estimator = ClockEstimator::new(20_000_000.0);

        assert_eq!(estimator.print_time_to_clock(1.5), 30_000_000);
        assert_eq!(estimator.clock_to_print_time(30_000_000), 1.5);
    }

    #[test]
    fn test_the_regression_converges_on_the_clock_frequency() {
        let freq = 20_000_000.0;
        let mut estimator = ClockEstimator::new(freq);
        estimator.seed(1.0, freq as i64);
        // Samples of a clock running at exactly `freq`, taken every 0.2 s.
        for i in 0..300 {
            let at = 1.0 + i as f64 * 0.2;
            assert!(
                estimator.update(at, at + 0.000_5, (at * freq) as u32),
                "{at}"
            );
        }

        // The estimate at a later system time tracks the real clock.
        let predicted = estimator.estimated_print_time(100.0);
        assert!((predicted - 100.0).abs() < 0.01, "{predicted}");
    }

    #[test]
    fn test_an_outlier_sample_is_discarded() {
        let freq = 20_000_000.0;
        let mut estimator = ClockEstimator::new(freq);
        estimator.seed(1.0, freq as i64);
        for i in 0..10 {
            let at = 1.0 + i as f64 * 0.2;
            assert!(estimator.update(at, at + 0.000_5, (at * freq) as u32));
        }

        // A clock a full second ahead, arriving immediately, is a queued or
        // delayed message, not a real jump (`klippy/clocksync.py:74-83`).
        let at = 1.0 + 10.0 * 0.2;
        let ahead = ((at + 1.0) * freq) as u32;

        assert!(!estimator.update(at, at + 0.000_5, ahead));
    }

    #[test]
    fn test_secondary_sync_compensates_crystal_drift() {
        // The primary defines print time; the secondary's crystal runs 100 ppm
        // fast. Sampling `get_clock` and recalibrating once a second must keep
        // the secondary's clock mapped onto the primary's print time.
        let nominal = 1_000_000.0;
        let drifted = 1_000_100.0;
        let mut primary = ClockEstimator::new(nominal);
        primary.seed(0.0, 0);
        let mut secondary = ClockEstimator::new(nominal);
        secondary.seed(0.0, 0);
        let mut sync = SecondarySync::new(nominal);

        let steps = 36_000;
        for step in 0..steps {
            let t = step as f64 * 0.1;
            secondary.update(t, t + 0.000_005, (t * drifted) as u32);
            if step % 10 == 0 {
                sync.calibrate(&primary, &secondary, t, t);
            }
        }

        let t = (steps - 1) as f64 * 0.1;
        let clock = (t * drifted) as i64;
        let mapped = sync.clock_to_print_time(clock);
        assert!((mapped - t).abs() < 0.01, "mapped {mapped} vs {t}");
        assert!((sync.freq - drifted).abs() < 1.0, "{}", sync.freq);

        // Without recalibration the same run drifts by much more (this is what
        // the one-shot connect-time offset used to give).
        let naive = SecondarySync::new(nominal);
        let naive_mapped = naive.clock_to_print_time(clock);
        assert!(
            (naive_mapped - t).abs() > 0.1,
            "\\n{naive_mapped} should drift"
        );
    }

    #[test]
    fn test_clock32_extends_around_the_wrap() {
        let mut estimator = ClockEstimator::new(1.0);
        estimator.seed(0.0, 0xffff_fff0);

        assert_eq!(estimator.clock32_to_clock64(0x10), 0x1_0000_0010);
    }

    #[test]
    fn test_estimated_print_time_advances_with_the_system_clock() {
        let freq = 20_000_000.0;
        let mut estimator = ClockEstimator::new(freq);
        estimator.seed(10.0, (10.0 * freq) as i64);

        // Before any get_clock sample, the seed's frequency is used.
        assert!((estimator.estimated_print_time(11.0) - 11.0).abs() < 1e-6);
        assert!((estimator.estimated_print_time(12.0) - 12.0).abs() < 1e-6);
    }

    #[tokio::test]
    async fn test_a_query_updates_the_estimate() {
        let mcu = mcu_answering(clock_exchange(1234));
        let clock = McuClock::new(Arc::clone(&mcu), ManualReactor::shared());

        clock.get_clock().await.unwrap();

        // The sample was folded in: the 64-bit clock advanced to the value the
        // firmware reported, and the frequency came from the dictionary.
        let estimator = clock.estimator();
        assert_eq!(estimator.last_clock(), 1234);
        assert_eq!(estimator.mcu_freq(), 20_000_000.0);
    }

    #[tokio::test]
    async fn test_a_query_feeds_the_mcu_clock_estimate() {
        // The seed the MCU object would take at connect, then one round trip:
        // the sample has to pull the estimate off the seed and onto the
        // reading the firmware reported.
        let mcu = mcu_answering(clock_exchange(1234));
        mcu.set_clock_base(0);
        let clock = McuClock::new(Arc::clone(&mcu), ManualReactor::shared());

        clock.get_clock().await.unwrap();

        let got = mcu.estimated_clock().unwrap();
        assert!(got >= 1234, "the sample's reading anchors it: {got}");
        assert!(
            got < 1234 + 20_000_000,
            "only half a round trip of extrapolation, not seconds: {got}"
        );
    }

    #[tokio::test]
    async fn test_seed_takes_the_frequency_from_the_dictionary() {
        // A board that has been up 10 s at 20 MHz, with no query: the seed alone
        // must map its clock back to 10 s of print time.
        let mcu = mcu_answering(Vec::new());
        let clock = McuClock::new(mcu, ManualReactor::shared());

        clock.seed(7.5, 200_000_000);

        assert_eq!(clock.estimator().mcu_freq(), 20_000_000.0);
        assert!((clock.estimated_print_time(7.5) - 10.0).abs() < 1e-9);
        // And it advances with the host clock from there.
        assert!((clock.estimated_print_time(8.5) - 11.0).abs() < 1e-9);
    }

    #[tokio::test]
    async fn test_clock_sync_trait_can_be_implemented_without_an_mcu() {
        let clock = FixedClock(42);
        assert_eq!(clock.get_clock().await.unwrap().clock, 42);
    }
}
