//! The MCU as an ADC chip: `McuAdc`, upstream's `MCU_adc`.
//!
//! An analog input is configured once (`config_analog_in`) and then sampled by
//! the firmware on a periodic query armed at init. Each report is an event, not
//! a response, so the input has to be listening before the query is sent:
//! [`McuAdc`] binds its handler from a **post-init** callback, the moment the
//! firmware has accepted the configuration and is about to run the `init` list.
//!
//! # Two report formats
//!
//! The firmware has answered with `value=%hu` since early on and with
//! `values=%*s` (a batch) more recently. Both are the same `analog_in_state`
//! message, told apart by their format string; the query that is sent has to
//! match. Upstream tries `bytes_per_report` and falls back to the old shape; here
//! the dictionary says which the firmware declares, so the choice is exact.
//!
//! # Routing by oid
//!
//! One MCU can have many inputs, and every report carries the `oid` it belongs
//! to. The parser allows one callback per message, so the per-input handlers are
//! kept here, in [`AdcRegistry`], and one bound callback routes by oid — the same
//! thing upstream's `register_serial_response(..., oid=…)` does.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard};

use super::pin::{pin_number, AdcClock};
use crate::core::klippy::cmd::adc::{
    AnalogInState, ConfigAnalogIn, QueryAnalogIn, QueryAnalogInOld,
};
use crate::core::klippy::cmd::McuResponse;
use crate::core::klippy::mcu::{query_slot, ConfigBuilder, Mcu, McuError};
use crate::core::klippy::msg::proto::ArgValue;
use crate::core::klippy::pins::{Adc, AdcCallback, PinError, PinParams, PrinterPins};
use tracing::warn;

/// The format string of the older one-sample query (no `bytes_per_report`).
///
/// The batched (newer) form is the fallback, so only this one needs to be named
/// to tell which firmware generation is in front of us.
const OLD_QUERY: &str = "query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c \
                         rest_ticks=%u min_value=%hu max_value=%hu range_check_count=%c";

/// The most samples one report may batch (`48 // 2`, `klippy/mcu.py:576`).
const MAX_BATCH_NUM: u32 = 24;

/// Per-oid routing for `analog_in_state`, one per MCU.
///
/// Shared by every [`McuAdc`] on a chip. Every registration (re)binds the
/// parser callback: a message has one callback, and all the closures are the
/// same router over the same map, so rebinding is harmless — and it is what
/// keeps a reconnected `Mcu` (a firmware `reset`) working, since its fresh
/// parser has no callback yet.
#[derive(Default)]
pub struct AdcRegistry {
    /// The handler for each input's oid.
    channels: Mutex<HashMap<u8, Arc<AdcState>>>,
}

impl AdcRegistry {
    /// An empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Route this oid's reports into `state`, (re)binding the callback.
    fn register(
        self: &Arc<Self>,
        mcu: &Mcu,
        oid: u8,
        state: Arc<AdcState>,
    ) -> Result<(), McuError> {
        self.channels
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .insert(oid, Arc::clone(&state));
        let registry = Arc::clone(self);
        mcu.bind_callback(AnalogInState::NAME, move |values| {
            // Every report starts with `oid=%c`; anything else is not ours.
            let oid = match values.first() {
                Some(ArgValue::UInt8(oid)) => *oid,
                _ => return,
            };
            let state = registry
                .channels
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .get(&oid)
                .cloned();
            if let Some(state) = state {
                state.handle(values);
            }
        })
    }
}

/// The state one input's config callback builds and its reports update.
struct AdcState {
    /// The chip's clock facts: what dates a report. A view, not the [`McuChip`]
    /// itself — the chip owns the registry that owns this state, so a chip here
    /// would be a strong cycle (`AdcClock` has the details).
    clock: AdcClock,
    // Sampling parameters, set by `setup_adc_sample`.
    report_time: Mutex<f64>,
    sample_time: Mutex<f64>,
    sample_count: Mutex<u32>,
    batch_num: Mutex<u32>,
    minval: Mutex<f64>,
    maxval: Mutex<f64>,
    range_check_count: Mutex<u32>,
    // Sampling parameters as they reach the wire, filled by `build`.
    sample_ticks: Mutex<u32>,
    min_sample: Mutex<u16>,
    max_sample: Mutex<u16>,
    // Built state.
    oid: Mutex<Option<u8>>,
    /// `1.0 / (sample_count * ADC_MAX)`, the scale from a raw average to
    /// `0.0..=1.0`.
    inv_max_adc: Mutex<f64>,
    /// The report period in clock ticks, needed to date a batch's samples.
    report_clock: Mutex<u32>,
    /// Whether the old `value=%hu` format is in use.
    old_format: Mutex<bool>,
    /// The last `(print time, value)` seen.
    last_value: Mutex<Option<(f64, f64)>>,
    /// Whether an undatable report has already been warned about.
    undated_reported: Mutex<bool>,
    /// The consumer's callback, shared so it can be cloned out before it runs.
    callback: Mutex<Option<Arc<AdcCallback>>>,
}

/// One analog input on an MCU.
pub struct McuAdc {
    state: Arc<AdcState>,
}

impl McuAdc {
    /// Build the resource and register its config callback.
    ///
    /// # Panics
    /// As [`McuDigitalOut::new`](crate::core::klippy::mcu::McuDigitalOut).
    pub(crate) fn new(
        config: Arc<ConfigBuilder>,
        pins: Arc<PrinterPins>,
        chip_name: String,
        registry: Arc<AdcRegistry>,
        pin: PinParams,
        clock: AdcClock,
    ) -> Self {
        let state = Arc::new(AdcState {
            clock,
            report_time: Mutex::new(0.0),
            sample_time: Mutex::new(0.0),
            sample_count: Mutex::new(1),
            batch_num: Mutex::new(1),
            minval: Mutex::new(0.0),
            maxval: Mutex::new(1.0),
            range_check_count: Mutex::new(0),
            sample_ticks: Mutex::new(0),
            min_sample: Mutex::new(0),
            max_sample: Mutex::new(0),
            oid: Mutex::new(None),
            inv_max_adc: Mutex::new(0.0),
            report_clock: Mutex::new(0),
            old_format: Mutex::new(true),
            last_value: Mutex::new(None),
            undated_reported: Mutex::new(false),
            callback: Mutex::new(None),
        });

        let callback_state = Arc::clone(&state);
        // Weak, not Arc: see `McuDigitalOut::new` — a strong handle would cycle
        // through the chip's `ConfigBuilder` and outlive a restart.
        let callback_pins = Arc::downgrade(&pins);
        let callback_registry = Arc::clone(&registry);
        let callback_pin = pin.clone();
        config
            .register_config_callback(Box::new(move |builder, mcu| {
                let pins = callback_pins
                    .upgrade()
                    .expect("the pins registry outlives the resources it built");
                callback_state.build(
                    builder,
                    mcu,
                    &pins,
                    &callback_registry,
                    &chip_name,
                    &callback_pin,
                )
            }))
            .expect("a resource is always built before the configuration is");
        Self { state }
    }
}

impl Adc for McuAdc {
    fn setup_adc_sample(
        &self,
        report_time: f64,
        sample_time: f64,
        sample_count: u32,
        batch_num: u32,
        minval: f64,
        maxval: f64,
        range_check_count: u32,
    ) {
        *self.state.lock(|state| &state.report_time) = report_time;
        *self.state.lock(|state| &state.sample_time) = sample_time;
        *self.state.lock(|state| &state.sample_count) = sample_count;
        *self.state.lock(|state| &state.batch_num) = batch_num.clamp(1, MAX_BATCH_NUM);
        *self.state.lock(|state| &state.minval) = minval;
        *self.state.lock(|state| &state.maxval) = maxval;
        *self.state.lock(|state| &state.range_check_count) = range_check_count;
    }

    fn setup_adc_callback(&self, callback: AdcCallback) {
        *self.state.lock(|state| &state.callback) = Some(Arc::new(callback));
    }

    fn get_last_value(&self) -> Option<(f64, f64)> {
        *self.state.lock(|state| &state.last_value)
    }
}

/// The query form an input arms: the old one-sample message or the batched one.
pub(crate) enum AdcQuery {
    Old(QueryAnalogInOld),
    New(QueryAnalogIn),
}

impl AdcState {
    /// Scale one raw average to `0.0..=1.0`.
    fn scale(&self, raw: u16) -> f64 {
        f64::from(raw) * *self.lock(|state| &state.inv_max_adc)
    }

    /// Handle one `analog_in_state`, updating the last value and calling back.
    ///
    /// The values arrive positionally in the firmware's declaration order:
    /// `oid`, `next_clock`, then `value` (old) or `values` (new).
    ///
    /// `next_clock` is the firmware's **low 32 bits**, so it is extended before
    /// it is used — a report that straddles the 2^32 wrap would otherwise date
    /// its samples backwards — and the extended clock is mapped to print time
    /// (`klippy/mcu.py:636-646`). Dating a report needs the MCU's clock
    /// mapping, which the chip fills at connect; a report that arrives before
    /// that is dropped whole (the extension needs the clock estimate, and a
    /// bare firmware clock is not in the consumer's time domain). The first
    /// such drop is warned about, once: a silently starved sensor looks exactly
    /// like a sensor whose reports never arrive.
    fn handle(&self, values: &[ArgValue]) {
        let next_clock = match values.get(1) {
            Some(ArgValue::UInt32(clock)) => *clock,
            _ => return,
        };
        let Some(mapping) = self.clock.mapping() else {
            self.note_undated_report();
            return;
        };
        // The mapping and the estimate are filled together at connect
        // (`McuChip::set_clock`), so this only closes the type.
        let Some(next_clock) = self.clock.clock32_to_clock64(next_clock) else {
            return;
        };
        let old_format = *self.lock(|state| &state.old_format);
        // `next_clock` is when the **next** report is due, so every sample of
        // this one is dated backwards from it: the last sample of a batch sits
        // one report period before `next_clock`, the first one `count` periods
        // before (`klippy/mcu.py:635,644`).
        let report_clock = i64::from(*self.lock(|state| &state.report_clock));

        let samples: Vec<(f64, f64)> = if old_format {
            let raw = match values.get(2) {
                Some(ArgValue::UInt16(value)) => *value,
                _ => return,
            };
            vec![(
                mapping.print_time(next_clock - report_clock),
                self.scale(raw),
            )]
        } else {
            let bytes = match values.get(2) {
                Some(ArgValue::Bytes(bytes)) => bytes,
                _ => return,
            };
            let count = bytes.len() / 2;
            let mut samples = Vec::with_capacity(count);
            for (index, pair) in bytes.chunks_exact(2).enumerate() {
                let raw = u16::from_le_bytes([pair[0], pair[1]]);
                // The back-off happens in the 64-bit domain, so a report that
                // straddles the wrap cannot fold into a huge difference.
                let clock = next_clock - (count - index) as i64 * report_clock;
                samples.push((mapping.print_time(clock), self.scale(raw)));
            }
            samples
        };

        if let Some(last) = samples.last() {
            *self.lock(|state| &state.last_value) = Some(*last);
        }
        // Cloned out, then the lock is gone before the consumer runs: a std
        // mutex is not reentrant, and a consumer that reads this input back
        // from its callback (`get_last_value`) would deadlock on it.
        let callback = self.lock(|state| &state.callback).clone();
        if let Some(callback) = callback {
            (*callback)(&samples);
        }
    }

    /// Warn once that a report could not be dated.
    fn note_undated_report(&self) {
        let mut reported = self.lock(|state| &state.undated_reported);
        if *reported {
            return;
        }
        *reported = true;
        let oid = *self.lock(|state| &state.oid);
        warn!(
            "ADC input oid={oid:?}: a report arrived before the MCU clock mapping; \
             dropping its samples until the chip has one"
        );
    }

    /// The build-time half: resolve the pin and add the configuration/query.
    ///
    /// Upstream's `MCU_adc._build_config` (`klippy/mcu.py:584-630`).
    fn build(
        self: &Arc<Self>,
        builder: &ConfigBuilder,
        mcu: &Mcu,
        pins: &PrinterPins,
        registry: &Arc<AdcRegistry>,
        chip_name: &str,
        pin: &PinParams,
    ) -> Result<(), McuError> {
        let sample_count = *self.lock(|state| &state.sample_count);
        // Not set up: no oid, no query, exactly as upstream returns early.
        if sample_count == 0 {
            return Ok(());
        }
        let Some(sample_count) = u8::try_from(sample_count).ok() else {
            return Err(PinError::AdcSampleCountTooLarge(sample_count).into());
        };

        let oid = builder.create_oid()?;
        *self.lock(|state| &state.oid) = Some(oid);

        let canonical = pins.resolve_pin(chip_name, &pin.pin)?;
        let number = pin_number(mcu, &canonical, chip_name)?;
        builder.add_config_cmd(&ConfigAnalogIn { oid, pin: number })?;

        let sample_ticks = mcu.seconds_to_clock(*self.lock(|state| &state.sample_time))? as u32;
        *self.lock(|state| &state.sample_ticks) = sample_ticks;
        let report_clock = mcu.seconds_to_clock(*self.lock(|state| &state.report_time))? as u32;
        *self.lock(|state| &state.report_clock) = report_clock;

        let adc_max = mcu
            .dictionary()
            .and_then(|dictionary| dictionary.constant_f64("ADC_MAX"))
            .ok_or_else(|| McuError::Config("dictionary has no ADC_MAX".to_string()))?;
        let max_adc = f64::from(sample_count) * adc_max;
        // The firmware reports the average in 16 bits.
        if max_adc >= 65536.0 {
            return Err(PinError::AdcSampleCountTooLarge(u32::from(sample_count)).into());
        }
        *self.lock(|state| &state.inv_max_adc) = 1.0 / max_adc;

        let min_sample = (*self.lock(|state| &state.minval) * max_adc).clamp(0.0, 65535.0) as u16;
        let max_sample = (*self.lock(|state| &state.maxval) * max_adc)
            .ceil()
            .clamp(0.0, 65535.0) as u16;
        *self.lock(|state| &state.min_sample) = min_sample;
        *self.lock(|state| &state.max_sample) = max_sample;

        // The old one-sample form is used only when the firmware declares it
        // and one sample per report was asked for; otherwise the batched form
        // carries the count (`klippy/mcu.py:626-628`). The two forms share the
        // message name, so the format string is what tells them apart.
        let old_declared = mcu
            .dictionary()
            .map(|dictionary| {
                dictionary
                    .commands()
                    .iter()
                    .any(|message| message.format == OLD_QUERY)
            })
            .unwrap_or(false);
        let use_old = *self.lock(|state| &state.batch_num) == 1 && old_declared;
        *self.lock(|state| &state.old_format) = use_old;

        // The query carries an **absolute** clock. It is sent from the post-init
        // callback — once the firmware has accepted the configuration and on the
        // connection that will report — so the clock is read fresh there rather
        // than baked into the config. That matters when a firmware is reset and
        // re-identified mid-connect: a waketime from before the reboot is tens of
        // seconds off the new clock, and the firmware's signed timer comparison
        // reads it as "in the past" (`sched.c:94`).
        let registry = Arc::clone(registry);
        let state = Arc::clone(self);
        builder.register_post_init_callback(Box::new(move |mcu| {
            if let Err(err) = state.arm_query(mcu, oid) {
                warn!("MCU '{}': could not arm the ADC query: {err}", mcu.name());
                return;
            }
            if let Err(err) = registry.register(mcu, oid, Arc::clone(&state)) {
                warn!("MCU '{}': could not bind ADC response: {err}", mcu.name());
            }
        }))?;
        Ok(())
    }

    /// Send this input's `query_analog_in`, with a clock read from `mcu` now.
    fn arm_query(&self, mcu: &Mcu, oid: u8) -> Result<(), McuError> {
        let clock = query_slot(mcu, oid)?;
        match self.query_for(oid, clock)? {
            AdcQuery::Old(query) => mcu.send_msg(&query),
            AdcQuery::New(query) => mcu.send_msg(&query),
        }
    }

    /// The `query_analog_in` this input arms, for `clock`.
    ///
    /// Split out from [`AdcState::arm_query`] so the message shape can be
    /// checked without a live connection.
    fn query_for(&self, oid: u8, clock: u32) -> Result<AdcQuery, McuError> {
        let sample_count = u8::try_from(*self.lock(|state| &state.sample_count))
            .map_err(|_| McuError::Config("ADC sample_count does not fit a byte".to_string()))?;
        let range_check_count = u8::try_from(*self.lock(|state| &state.range_check_count))
            .map_err(|_| {
                McuError::Config("ADC range_check_count does not fit a byte".to_string())
            })?;
        let sample_ticks = *self.lock(|state| &state.sample_ticks);
        let rest_ticks = *self.lock(|state| &state.report_clock);
        let min_value = *self.lock(|state| &state.min_sample);
        let max_value = *self.lock(|state| &state.max_sample);
        if *self.lock(|state| &state.old_format) {
            Ok(AdcQuery::Old(QueryAnalogInOld {
                oid,
                clock,
                sample_ticks,
                sample_count,
                rest_ticks,
                min_value,
                max_value,
                range_check_count,
            }))
        } else {
            let batch_num = *self.lock(|state| &state.batch_num);
            Ok(AdcQuery::New(QueryAnalogIn {
                oid,
                clock,
                sample_ticks,
                sample_count,
                rest_ticks,
                bytes_per_report: (batch_num * 2) as u8,
                min_value,
                max_value,
                range_check_count,
            }))
        }
    }

    fn lock<T>(&self, field: impl Fn(&Self) -> &Mutex<T>) -> MutexGuard<'_, T> {
        field(self)
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::cmd::clock::McuClock;
    use crate::core::klippy::cmd::McuCommand;
    use crate::core::klippy::interface::devices::frame_mock::FrameMock;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::Dictionary;
    use crate::core::klippy::mcu::McuChip;
    use crate::core::klippy::msg::proto::Payload;
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::json;

    /// A dictionary whose query/response pair matches `batched`.
    fn dictionary(batched: bool) -> Dictionary {
        let (query, response) = if batched {
            (
                "query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u \
                 bytes_per_report=%c min_value=%hu max_value=%hu range_check_count=%c",
                "analog_in_state oid=%c next_clock=%u values=%*s",
            )
        } else {
            (
                "query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u \
                 min_value=%hu max_value=%hu range_check_count=%c",
                "analog_in_state oid=%c next_clock=%u value=%hu",
            )
        };
        Dictionary::from_json(json!({
            "commands": {
                "allocate_oids count=%c": 2,
                "get_config": 7,
                "finalize_config crc=%u": 6,
                "config_analog_in oid=%c pin=%u": 30,
                query: 31
            },
            "responses": {
                "config is_config=%c crc=%u is_shutdown=%c move_count=%hu": 9,
                response: 33
            },
            "enumerations": {"pin": {"PA0": 0, "PA1": 1, "PB2": 2}},
            "config": {"CLOCK_FREQ": 20000000, "ADC_MAX": 4095}
        }))
        .unwrap()
    }

    fn mcu(batched: bool) -> Mcu {
        let mcu = Mcu::for_test("mcu", Interface::new(FrameMock::new(Vec::new())));
        mcu.install_dictionary(dictionary(batched)).unwrap();
        // The query slot needs a clock to place the first report in the future.
        mcu.set_clock_base(1_000_000);
        mcu
    }

    fn chip() -> (McuChip, Arc<PrinterPins>) {
        let pins = Arc::new(PrinterPins::new());
        let chip = McuChip::new(
            "mcu".to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::clone(&pins),
        );
        pins.register_chip("mcu", Arc::new(chip.clone())).unwrap();
        (chip, pins)
    }

    /// The decoded `(name, args)` of one list, from a single build.
    fn decode_list(mcu: &Mcu, payloads: &[Payload]) -> Vec<(String, Vec<ArgValue>)> {
        let mut parser = crate::core::klippy::msg::parser::Parser::new();
        mcu.dictionary().unwrap().install(&mut parser).unwrap();
        payloads
            .iter()
            .map(|payload| {
                let frame = crate::core::klippy::frame::Frame::new(0, payload.payload().to_vec());
                let decoded = parser.decode(frame.into()).unwrap();
                (decoded[0].0.name.clone(), decoded[0].1.clone())
            })
            .collect()
    }

    /// Build the chip, then return the decoded config and init lists.
    fn build_lists(
        chip: &McuChip,
        mcu: &Mcu,
    ) -> (Vec<(String, Vec<ArgValue>)>, Vec<(String, Vec<ArgValue>)>) {
        let built = chip.config().build(mcu).unwrap();
        (
            decode_list(mcu, &built.config),
            decode_list(mcu, &built.init),
        )
    }

    fn configure(pins: &Arc<PrinterPins>, sample_count: u32, batch_num: u32) -> Arc<dyn Adc> {
        let adc = pins.setup_adc("PA1", None).unwrap();
        adc.setup_adc_sample(0.1, 0.01, sample_count, batch_num, 0.0, 1.0, 3);
        adc
    }

    #[tokio::test]
    async fn test_a_batched_adc_builds_the_config_and_defers_the_query() {
        let (chip, pins) = chip();
        configure(&pins, 8, 4);
        let mcu = mcu(true);

        let (config, init) = build_lists(&chip, &mcu);

        assert_eq!(config[1].0, "config_analog_in");
        assert_eq!(config[1].1, vec![ArgValue::UInt8(0), ArgValue::UInt32(1)]);
        // The query carries an absolute clock, so it is sent from the post-init
        // callback rather than baked into `init` (a waketime from before a
        // firmware reset is off the new clock).
        assert!(init.is_empty());
    }

    #[test]
    fn test_the_batched_query_carries_the_sampling_parameters() {
        let state = armed_state(false, 4);
        let query = state.query_for(0, 123).unwrap();
        let AdcQuery::New(query) = query else {
            panic!("the batched form was declared");
        };
        let args = query.args();
        assert_eq!(args[0], ArgValue::UInt8(0));
        assert_eq!(args[1], ArgValue::UInt32(123));
        assert_eq!(args[2], ArgValue::UInt32(200_000));
        assert_eq!(args[3], ArgValue::UInt8(8));
        assert_eq!(args[4], ArgValue::UInt32(2_000_000));
        // `bytes_per_report` = batch_num * 2.
        assert_eq!(args[5], ArgValue::UInt8(8));
        assert_eq!(args[7], ArgValue::UInt16(32760));
    }

    #[test]
    fn test_an_old_firmware_gets_the_old_query() {
        let state = armed_state(true, 1);
        let query = state.query_for(0, 123).unwrap();
        // The old form has no `bytes_per_report`.
        assert!(matches!(query, AdcQuery::Old(_)));
    }

    #[test]
    fn test_a_batch_of_one_on_new_firmware_still_uses_the_batched_query() {
        let state = armed_state(false, 1);
        let query = state.query_for(0, 123).unwrap();
        let AdcQuery::New(query) = query else {
            panic!("the batched form was declared");
        };
        // One sample per report is still two bytes.
        assert_eq!(query.args()[5], ArgValue::UInt8(2));
    }

    #[test]
    fn test_a_range_check_count_that_does_not_fit_a_byte_is_rejected() {
        let state = armed_state(false, 4);
        // The wire field is `%c`. Clamping to 255 would arm a different check
        // than the one asked for, silently; upstream's format packing refuses.
        *state.lock(|s| &s.range_check_count) = 256;

        let Err(err) = state.query_for(0, 123) else {
            panic!("a range_check_count of 256 was clamped into the query");
        };

        assert!(
            err.to_string()
                .contains("range_check_count does not fit a byte"),
            "{err}"
        );
    }

    #[tokio::test]
    async fn test_sample_count_zero_builds_nothing() {
        let (chip, pins) = chip();
        configure(&pins, 0, 1);
        let mcu = mcu(true);

        let (config, init) = build_lists(&chip, &mcu);

        // Only allocate_oids and finalize_config, and no init query.
        assert_eq!(config.len(), 2);
        assert!(init.is_empty());
    }

    #[tokio::test]
    async fn test_a_sample_count_that_overflows_the_average_is_rejected() {
        let (chip, pins) = chip();
        // 20 * 4095 > 2^16.
        configure(&pins, 20, 1);
        let mcu = mcu(true);

        let err = chip.config().build(&mcu).unwrap_err();

        assert!(
            err.to_string().contains("sample_count=20 too large"),
            "{err}"
        );
    }

    #[tokio::test]
    async fn test_the_query_slot_places_the_first_report_in_the_future() {
        let (_chip, _pins) = chip();
        let mcu = mcu(true);
        mcu.set_clock_base(1_000_000);
        let before = mcu.estimated_clock().unwrap();

        let clock = query_slot(&mcu, 0).unwrap();

        let expected = before + 30_000_000; // 1.5 s at 20 MHz
        assert!(u64::from(clock) >= expected, "{clock} < {expected}");
        assert!(
            u64::from(clock) < expected + 2_000_000,
            "{clock} is too far out"
        );
    }

    /// A state with the sampling parameters `build` would have stored.
    fn armed_state(old_format: bool, batch_num: u32) -> Arc<AdcState> {
        let state = state(undated_clock(), old_format, 1.0 / 32760.0, 2_000_000);
        *state.lock(|s| &s.sample_ticks) = 200_000;
        *state.lock(|s| &s.sample_count) = 8;
        *state.lock(|s| &s.report_clock) = 2_000_000;
        *state.lock(|s| &s.min_sample) = 0;
        *state.lock(|s| &s.max_sample) = 32760;
        *state.lock(|s| &s.range_check_count) = 4;
        *state.lock(|s| &s.batch_num) = batch_num;
        state
    }

    // -----------------------------------------------------------------------
    // Response handling
    // -----------------------------------------------------------------------

    /// A clock view with no mapping yet: a report cannot be dated.
    fn undated_clock() -> AdcClock {
        McuChip::new(
            "mcu".to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::new(PrinterPins::new()),
        )
        .adc_clock()
    }

    /// A chip whose clock is seeded at `clock` (20 MHz, from the test
    /// dictionary), and the view an ADC input dates reports with.
    fn dated_clock(clock: i64) -> (McuChip, AdcClock) {
        let chip = McuChip::new(
            "mcu".to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::new(PrinterPins::new()),
        );
        let mcu = Arc::new(mcu(true));
        let estimate = Arc::new(McuClock::new(Arc::clone(&mcu), ManualReactor::shared()));
        estimate.seed(0.0, clock);
        chip.set_clock(estimate, 0.0);
        let view = chip.adc_clock();
        (chip, view)
    }

    fn state(
        clock: AdcClock,
        old_format: bool,
        inv_max_adc: f64,
        report_clock: u32,
    ) -> Arc<AdcState> {
        Arc::new(AdcState {
            clock,
            report_time: Mutex::new(0.0),
            sample_time: Mutex::new(0.0),
            sample_count: Mutex::new(1),
            batch_num: Mutex::new(1),
            minval: Mutex::new(0.0),
            maxval: Mutex::new(1.0),
            range_check_count: Mutex::new(0),
            sample_ticks: Mutex::new(0),
            min_sample: Mutex::new(0),
            max_sample: Mutex::new(0),
            oid: Mutex::new(Some(0)),
            inv_max_adc: Mutex::new(inv_max_adc),
            report_clock: Mutex::new(report_clock),
            old_format: Mutex::new(old_format),
            last_value: Mutex::new(None),
            undated_reported: Mutex::new(false),
            callback: Mutex::new(None),
        })
    }

    #[tokio::test]
    async fn test_the_old_report_dates_the_sample_one_report_back() {
        let (chip, clock) = dated_clock(0);
        // `next_clock` is when the next report is due, so the sample in this one
        // is a report period older (`klippy/mcu.py:635`).
        let expected_time = chip.clock_to_print_time(5_000).unwrap();
        let state = state(clock, true, 1.0 / 4095.0, 2_000_000);
        let seen: Arc<Mutex<Vec<(f64, f64)>>> = Arc::new(Mutex::new(Vec::new()));
        let seen_for_callback = Arc::clone(&seen);
        state
            .lock(|state| &state.callback)
            .replace(Arc::new(Box::new(move |samples| {
                seen_for_callback.lock().unwrap().extend_from_slice(samples);
            })));

        state.handle(&[
            ArgValue::UInt8(0),
            ArgValue::UInt32(2_005_000),
            ArgValue::UInt16(4095),
        ]);

        assert_eq!(
            state.lock(|state| &state.last_value).as_ref().copied(),
            Some((expected_time, 1.0))
        );
        assert_eq!(*seen.lock().unwrap(), vec![(expected_time, 1.0)]);
    }

    #[tokio::test]
    async fn test_a_report_clock_that_wraps_32_bits_still_dates_samples_forwards() {
        // Seeded just below the wrap, so both reports extend forwards from it:
        // the first stays at 0xffff_ff00, the second becomes 0x1_0000_0100.
        let (chip, clock) = dated_clock(0xffff_fe00);
        let first = chip.clock_to_print_time(0xffff_ff00 - 2_000_000).unwrap();
        let second = chip
            .clock_to_print_time(0x0001_0000_0100 - 2_000_000)
            .unwrap();
        let state = state(clock, true, 1.0 / 4095.0, 2_000_000);
        let seen: Arc<Mutex<Vec<(f64, f64)>>> = Arc::new(Mutex::new(Vec::new()));
        let seen_for_callback = Arc::clone(&seen);
        state
            .lock(|state| &state.callback)
            .replace(Arc::new(Box::new(move |samples| {
                seen_for_callback.lock().unwrap().extend_from_slice(samples);
            })));

        // The 32-bit report clock wraps between the two reports.
        state.handle(&[
            ArgValue::UInt8(0),
            ArgValue::UInt32(0xffff_ff00),
            ArgValue::UInt16(1),
        ]);
        state.handle(&[
            ArgValue::UInt8(0),
            ArgValue::UInt32(0x0000_0100),
            ArgValue::UInt16(1),
        ]);

        let seen = seen.lock().unwrap();
        assert_eq!(seen.len(), 2);
        // Running past the wrap while the samples are dated.
        assert!(
            seen[1].0 > seen[0].0,
            "the second sample is dated before the first: {:?}",
            *seen
        );
        // One report period back from each `next_clock`, from the chip's own
        // conversion.
        assert_eq!([seen[0].0, seen[1].0], [first, second]);
    }

    #[tokio::test]
    async fn test_the_batched_report_dates_each_sample_back_from_the_next_one() {
        let (chip, clock) = dated_clock(0);
        // Two samples one report period (1000 ticks) apart, both dated back
        // from the *next* report: 9_000 - 2 * 1000 and 9_000 - 1000.
        let first = chip.clock_to_print_time(7_000).unwrap();
        let last = chip.clock_to_print_time(8_000).unwrap();
        let state = state(clock, false, 1.0 / 4095.0, 1_000);
        let seen: Arc<Mutex<Vec<(f64, f64)>>> = Arc::new(Mutex::new(Vec::new()));
        let seen_for_callback = Arc::clone(&seen);
        state
            .lock(|state| &state.callback)
            .replace(Arc::new(Box::new(move |samples| {
                seen_for_callback.lock().unwrap().extend_from_slice(samples);
            })));
        // Two samples, the report clock 1000 apart: each is dated one period
        // before the report that follows it.
        let bytes = vec![0xff, 0x0f, 0x00, 0x08]; // 4095, 2048
        state.handle(&[
            ArgValue::UInt8(0),
            ArgValue::UInt32(9_000),
            ArgValue::Bytes(bytes),
        ]);

        let (time, value) = state.lock(|state| &state.last_value).unwrap();
        assert_eq!(time, last);
        assert!((value - 2048.0 / 4095.0).abs() < f64::EPSILON);
        assert_eq!(
            *seen.lock().unwrap(),
            vec![(first, 1.0), (last, 2048.0 / 4095.0)]
        );
    }

    #[tokio::test]
    async fn test_a_consumer_is_called_with_the_callback_lock_released() {
        let (_chip, clock) = dated_clock(0);
        let state = state(clock, true, 1.0 / 4095.0, 1_000);
        let probe = Arc::downgrade(&state);
        let reentrant: Arc<Mutex<Option<bool>>> = Arc::new(Mutex::new(None));
        let reentrant_for_callback = Arc::clone(&reentrant);
        state
            .lock(|state| &state.callback)
            .replace(Arc::new(Box::new(move |_samples| {
                let state = probe.upgrade().unwrap();
                // A consumer may re-register itself from its own callback
                // (`setup_adc_callback`), or touch any other field. A std mutex
                // is not reentrant, so the callback lock has to be free here.
                *reentrant_for_callback.lock().unwrap() = Some(state.callback.try_lock().is_ok());
            })));

        state.handle(&[
            ArgValue::UInt8(0),
            ArgValue::UInt32(2_000),
            ArgValue::UInt16(4095),
        ]);

        assert_eq!(*reentrant.lock().unwrap(), Some(true));
    }

    #[test]
    fn test_a_report_before_the_clock_mapping_is_dropped_whole() {
        let state = state(undated_clock(), false, 1.0 / 4095.0, 1_000);
        let seen: Arc<Mutex<Vec<(f64, f64)>>> = Arc::new(Mutex::new(Vec::new()));
        let seen_for_callback = Arc::clone(&seen);
        state
            .lock(|state| &state.callback)
            .replace(Arc::new(Box::new(move |samples| {
                seen_for_callback.lock().unwrap().extend_from_slice(samples);
            })));

        state.handle(&[
            ArgValue::UInt8(0),
            ArgValue::UInt32(9_000),
            ArgValue::Bytes(vec![0xff, 0x0f, 0x00, 0x08]),
        ]);

        assert!(
            seen.lock().unwrap().is_empty(),
            "a batch was dated without a clock mapping"
        );
        assert_eq!(
            state.lock(|state| &state.last_value).as_ref().copied(),
            None
        );
    }
}
