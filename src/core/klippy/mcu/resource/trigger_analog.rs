//! The `trigger_analog` resource: a sensor sample the firmware watches for a
//! stop.
//!
//! Upstream's `MCU_trigger_analog` / `MCU_SosFilter`
//! (`klippy/extras/trigger_analog.py`). Where an endstop watches a GPIO, this
//! watches one sensor's raw samples: each is range-checked, run through an
//! [`SosFilter`], and compared against a trigger; a match fires the
//! [`TriggerDispatch`] and stops the steppers. Failures (raw range, filter
//! overflow, a sensor that went quiet) fire the same dispatch with an error
//! code, and [`McuTriggerAnalog::home_wait`] decodes it into the upstream
//! message (`trigger_analog.py:387-411`).
//!
//! # What is here
//!
//! | item | firmware | upstream |
//! |---|---|---|
//! | [`McuTriggerAnalog::home_start`] | `trigger_analog_home` (arm) | `home_start` |
//! | [`McuTriggerAnalog::home_wait`] | `trigger_analog_home` (disable) + `trigger_analog_query_state` | `home_wait` / `_clear_home` |
//! | [`SosFilter::reset`] | `sos_filter_set_*` with change-only resend | `MCU_SosFilter.reset_filter` |
//!
//! The sensor itself (`ldc1612_attach_trigger_analog`, sample delivery) is not
//! wired here — the caller supplies the sensor's sample rate at construction,
//! and sensor-specific error codes are decoded through
//! [`McuTriggerAnalog::set_sensor_error_lookup`].

use std::sync::{Arc, Mutex};

use super::pin::McuChip;
use super::trsync::{Completion, TriggerDispatch};
use crate::core::klippy::cmd::sos_filter::{
    ConfigSosFilter, SosFilterSetActive, SosFilterSetOffsetScale, SosFilterSetSection,
    SosFilterSetState,
};
use crate::core::klippy::cmd::trigger_analog::{
    ConfigTriggerAnalog, TriggerAnalogHome, TriggerAnalogQueryState, TriggerAnalogSetRawRange,
    TriggerAnalogSetTrigger, TriggerAnalogState, TriggerAnalogType, REASON_TRIGGER_ANALOG,
};
use crate::core::klippy::cmd::trsync::{raw_is_failure, TriggerReason};
use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::extras::toolhead::{EndstopFuture, HomingEndstop, QueryEndstopFuture};
use crate::core::klippy::mcu::{Mcu, McuError};

/// How long a `trigger_analog_query_state` exchange may take.
const QUERY_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(1);

/// How many missed sample windows cancel homing
/// (`MCU_trigger_analog.MONITOR_MAX`, `trigger_analog.py:272`).
pub const MONITOR_MAX: u32 = 3;

/// One `sos_filter` on one MCU: the fixed-point filter description plus the
/// resend caches upstream keeps (`_last_sent_coeffs` / `_last_sent_offset_scale`,
/// `trigger_analog.py:141-142, 264-291`).
///
/// Writing sections/state deactivates the firmware filter, so [`reset`](Self::reset)
/// always re-sends states and the activation; coefficients and offset/scale
/// travel only when they changed.
pub struct SosFilter {
    oid: u8,
    max_sections: u8,
    /// The description to send on the next [`reset`](Self::reset).
    design: Mutex<SosFilterDesign>,
    /// The coefficients last sent, per section.
    last_sent_coeffs: Mutex<Vec<Option<[i32; 5]>>>,
    /// The offset/scale tuple last sent.
    last_sent_offset_scale: Mutex<Option<(i32, i32, u8, bool)>>,
}

/// What [`SosFilter::reset`] sends: sections and states of fixed-point
/// coefficients, plus offset/scale and activation.
///
/// The conversion from a filter design (butterworth coefficients, sample-rate
/// scaling) into these numbers lives with the sensor code that designs
/// filters; this is only the wire-ready form.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct SosFilterDesign {
    /// One section's five coefficients (`b0 b1 b2 a1 a2`) per section.
    pub sections: Vec<[i32; 5]>,
    /// One section's two state words per section; must match `sections`.
    pub states: Vec<[i32; 2]>,
    /// Added to each raw sample before the sections run.
    pub offset: i32,
    /// Scale applied after the offset, in `scale_frac_bits` fixed point.
    pub scale: i32,
    /// Fractional bits of `scale`; the upstream initial state is `scale = 1`
    /// with no fractional bits.
    pub scale_frac_bits: u8,
    /// Whether the first sample replaces `offset`.
    pub auto_offset: bool,
    /// Fractional bits of the section coefficients.
    pub coeff_frac_bits: u8,
}

impl SosFilter {
    /// Build the filter on `chip` and register `config_sos_filter`.
    ///
    /// `max_sections` is the most sections that will ever be written at
    /// runtime; `0` passes samples through (upstream's default when no filter
    /// is attached, `trigger_analog.py:244`).
    ///
    /// # Errors
    /// Returns [`McuError`] if the oid cannot be created or the callback
    /// registered.
    pub fn new(chip: &McuChip, max_sections: u8) -> Result<Arc<Self>, McuError> {
        let builder = chip.config();
        let oid = builder.create_oid()?;
        let filter = Arc::new(Self {
            oid,
            max_sections,
            design: Mutex::new(SosFilterDesign::default()),
            last_sent_coeffs: Mutex::new(vec![None; max_sections as usize]),
            last_sent_offset_scale: Mutex::new(None),
        });
        let registered = Arc::clone(&filter);
        builder.register_config_callback(Box::new(move |builder, _mcu| {
            builder.add_config_cmd(&ConfigSosFilter {
                oid: registered.oid,
                max_sections: registered.max_sections,
            })?;
            Ok(())
        }))?;
        Ok(filter)
    }

    /// The oid the firmware assigned.
    pub fn oid(&self) -> u8 {
        self.oid
    }

    /// The most sections this filter accepts.
    pub fn max_sections(&self) -> u8 {
        self.max_sections
    }

    /// Replace the description the next [`reset`](Self::reset) sends.
    pub fn set_filter_design(&self, design: SosFilterDesign) {
        *self.design.lock().unwrap_or_else(|p| p.into_inner()) = design;
    }

    /// The messages one [`reset`](Self::reset) sends, updating the resend
    /// caches as a send would.
    ///
    /// Section coefficients and offset/scale are omitted when unchanged since
    /// the last send; states and activation are always included, because the
    /// firmware deactivates the filter whenever a section or state is written
    /// (`sos_filter.c:144-160`) and the activation is what re-arms it
    /// (upstream `reset_filter`, `trigger_analog.py:230-263`).
    ///
    /// # Errors
    /// Returns [`McuError`] when there are more sections than `max_sections`,
    /// or the state count does not match the section count — upstream raises
    /// the same as `ValueError` (`trigger_analog.py:230-263`).
    pub fn take_pending_messages(
        &self,
    ) -> Result<Vec<(&'static str, Vec<crate::core::klippy::msg::proto::ArgValue>)>, McuError> {
        let design = self
            .design
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        let num_sections = design.sections.len();
        if num_sections > self.max_sections as usize {
            return Err(McuError::Config(format!(
                "Too many filter sections: {num_sections}, The max is {}",
                self.max_sections
            )));
        }
        if design.states.len() != num_sections {
            return Err(McuError::Config(format!(
                "The number of filter sections ({num_sections}) and state sections ({}) must be equal",
                design.states.len()
            )));
        }

        use crate::core::klippy::msg::proto::ArgValue;
        let mut messages: Vec<(&'static str, Vec<ArgValue>)> = Vec::new();

        let mut last_coeffs = self
            .last_sent_coeffs
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        for (index, section) in design.sections.iter().enumerate() {
            if last_coeffs[index] == Some(*section) {
                continue;
            }
            messages.push((
                SosFilterSetSection::NAME,
                SosFilterSetSection {
                    oid: self.oid,
                    section_idx: index as u8,
                    sos: *section,
                }
                .args(),
            ));
            last_coeffs[index] = Some(*section);
        }
        drop(last_coeffs);

        for (index, state) in design.states.iter().enumerate() {
            messages.push((
                SosFilterSetState::NAME,
                SosFilterSetState {
                    oid: self.oid,
                    section_idx: index as u8,
                    state0: state[0],
                    state1: state[1],
                }
                .args(),
            ));
        }

        let offset_scale = (
            design.offset,
            design.scale,
            design.scale_frac_bits,
            design.auto_offset,
        );
        let mut last_offset_scale = self
            .last_sent_offset_scale
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        if *last_offset_scale != Some(offset_scale) || design.auto_offset {
            messages.push((
                SosFilterSetOffsetScale::NAME,
                SosFilterSetOffsetScale {
                    oid: self.oid,
                    offset: design.offset,
                    scale: design.scale,
                    scale_frac_bits: design.scale_frac_bits,
                    auto_offset: design.auto_offset,
                }
                .args(),
            ));
            *last_offset_scale = Some(offset_scale);
        }
        drop(last_offset_scale);

        if self.max_sections != 0 {
            messages.push((
                SosFilterSetActive::NAME,
                SosFilterSetActive {
                    oid: self.oid,
                    n_sections: num_sections as u8,
                    coeff_frac_bits: design.coeff_frac_bits,
                }
                .args(),
            ));
        }
        Ok(messages)
    }

    /// Send the pending messages to `mcu` (`MCU_SosFilter.reset_filter`).
    ///
    /// # Errors
    /// As [`take_pending_messages`](Self::take_pending_messages), or when a
    /// send fails.
    pub fn reset(&self, mcu: &Mcu) -> Result<(), McuError> {
        for (name, args) in self.take_pending_messages()? {
            mcu.send(name, &args)?;
        }
        Ok(())
    }
}

/// One `trigger_analog` object on one MCU.
pub struct McuTriggerAnalog {
    oid: u8,
    chip: McuChip,
    dispatch: TriggerDispatch,
    sos_filter: Arc<SosFilter>,
    /// The sensor's sample rate, for the monitor window in `home_start`.
    samples_per_second: f64,
    /// The raw range to enforce while homing (`set_raw_range`).
    raw_range: Mutex<(i32, i32)>,
    /// The range last sent; only a change goes out
    /// (`_last_range_args`, `trigger_analog.py:283`).
    last_range_args: Mutex<Option<(i32, i32)>>,
    /// The trigger to enforce while homing (`set_trigger`).
    trigger: Mutex<(TriggerAnalogType, i32)>,
    /// The trigger last sent (`_last_trigger_args`, `trigger_analog.py:287`).
    last_trigger_args: Mutex<Option<(TriggerAnalogType, i32)>>,
    /// The 64-bit clock the last `trigger_analog_home` armed at, so
    /// `home_wait` can map the firmware's 32-bit `homing_clock` onto **this**
    /// move instead of the synchronized estimate — the same reason
    /// [`McuEndstop`](super::endstop::McuEndstop) keeps one.
    arm_clock: Mutex<Option<u64>>,
    /// The print time of the last trigger, as upstream's `_last_trigger_time`.
    last_trigger_time: Mutex<f64>,
    /// Upstream's sensor-side decoder for codes at or above
    /// `SENSOR_SPECIFIC` (`trigger_analog.py:381-384`); `None` until a sensor
    /// installs one.
    sensor_error: Mutex<Option<Arc<dyn Fn(u8) -> String + Send + Sync>>>,
}

impl McuTriggerAnalog {
    /// Build the object on `chip` and register `config_trigger_analog`.
    ///
    /// `samples_per_second` is the sensor's update rate; it sets the monitor
    /// window (`1 / samples_per_second` ticks per expected sample). `sos_filter`
    /// is the filter to arm with the object; `None` creates the pass-through
    /// filter upstream defaults to (`max_sections = 0`).
    ///
    /// # Errors
    /// Returns [`McuError`] when the rate is not positive, the oid cannot be
    /// created, or a callback cannot be registered.
    pub fn new(
        chip: McuChip,
        samples_per_second: f64,
        sos_filter: Option<Arc<SosFilter>>,
    ) -> Result<Self, McuError> {
        if !(samples_per_second > 0.) {
            return Err(McuError::Config(
                "trigger_analog samples_per_second must be positive".to_string(),
            ));
        }
        let builder = chip.config();
        let oid = builder.create_oid()?;
        let sos_filter = match sos_filter {
            Some(filter) => filter,
            None => SosFilter::new(&chip, 0)?,
        };
        let sos_filter_oid = sos_filter.oid();
        builder.register_config_callback(Box::new(move |builder, _mcu| {
            builder.add_config_cmd(&ConfigTriggerAnalog {
                oid,
                sos_filter_oid,
            })?;
            Ok(())
        }))?;
        let dispatch = TriggerDispatch::new(vec![chip.clone()])?;
        Ok(Self {
            oid,
            chip,
            dispatch,
            sos_filter,
            samples_per_second,
            raw_range: Mutex::new((0, 0)),
            last_range_args: Mutex::new(None),
            trigger: Mutex::new((TriggerAnalogType::AbsGe, 0)),
            last_trigger_args: Mutex::new(None),
            arm_clock: Mutex::new(None),
            last_trigger_time: Mutex::new(0.0),
            sensor_error: Mutex::new(None),
        })
    }

    /// The oid the firmware assigned.
    pub fn oid(&self) -> u8 {
        self.oid
    }

    /// The trigger dispatch this object fires.
    pub fn dispatch(&self) -> &TriggerDispatch {
        &self.dispatch
    }

    /// The filter this object runs samples through.
    pub fn sos_filter(&self) -> &Arc<SosFilter> {
        &self.sos_filter
    }

    /// The print time of the last trigger (`get_last_trigger_time`).
    pub fn last_trigger_time(&self) -> f64 {
        *self
            .last_trigger_time
            .lock()
            .unwrap_or_else(|p| p.into_inner())
    }

    /// The raw samples outside which homing fails with `RAW_RANGE`
    /// (`set_raw_range`). Sent on the next arm, only if changed.
    pub fn set_raw_range(&self, raw_min: i32, raw_max: i32) {
        *self.raw_range.lock().unwrap_or_else(|p| p.into_inner()) = (raw_min, raw_max);
    }

    /// The comparison each filtered sample is checked against (`set_trigger`).
    /// Sent on the next arm, only if changed.
    pub fn set_trigger(&self, trigger_type: TriggerAnalogType, trigger_value: i32) {
        *self.trigger.lock().unwrap_or_else(|p| p.into_inner()) = (trigger_type, trigger_value);
    }

    /// Install the sensor's decoder for `SENSOR_SPECIFIC` and above codes
    /// (the value passed is the code below `SENSOR_SPECIFIC`).
    pub fn set_sensor_error_lookup(&self, lookup: impl Fn(u8) -> String + Send + Sync + 'static) {
        *self.sensor_error.lock().unwrap_or_else(|p| p.into_inner()) = Some(Arc::new(lookup));
    }

    /// The messages one arm sends: raw range and trigger when changed since
    /// the last arm, then the SOS filter's pending messages
    /// (upstream `_reset_filter`, `trigger_analog.py:351-363`).
    ///
    /// # Errors
    /// As [`SosFilter::take_pending_messages`].
    pub fn take_pending_filter_messages(
        &self,
    ) -> Result<Vec<(&'static str, Vec<crate::core::klippy::msg::proto::ArgValue>)>, McuError> {
        let mut messages = Vec::new();

        let range = *self.raw_range.lock().unwrap_or_else(|p| p.into_inner());
        let mut last_range = self
            .last_range_args
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        if *last_range != Some(range) {
            messages.push((
                TriggerAnalogSetRawRange::NAME,
                TriggerAnalogSetRawRange {
                    oid: self.oid,
                    raw_min: range.0,
                    raw_max: range.1,
                }
                .args(),
            ));
            *last_range = Some(range);
        }
        drop(last_range);

        let trigger = *self.trigger.lock().unwrap_or_else(|p| p.into_inner());
        let mut last_trigger = self
            .last_trigger_args
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        if *last_trigger != Some(trigger) {
            messages.push((
                TriggerAnalogSetTrigger::NAME,
                TriggerAnalogSetTrigger {
                    oid: self.oid,
                    trigger_analog_type: trigger.0,
                    trigger_value: trigger.1,
                }
                .args(),
            ));
            *last_trigger = Some(trigger);
        }
        drop(last_trigger);

        messages.extend(self.sos_filter.take_pending_messages()?);
        Ok(messages)
    }

    /// Arm the object for a probing move (`MCU_trigger_analog.home_start`).
    ///
    /// The sample-count arguments are upstream's uniform homing interface —
    /// `MCU_trigger_analog` ignores them too (`trigger_analog.py:374-385`);
    /// the sensor's rate, fixed at construction, sets the monitor window.
    ///
    /// # Errors
    /// Returns [`McuError`] if the MCU is not connected, the clock estimate is
    /// missing, or a send fails.
    pub fn home_start(
        &self,
        print_time: f64,
        _sample_time: f64,
        _sample_count: u8,
        _rest_time: f64,
        _triggered: bool,
    ) -> Result<Arc<Completion>, McuError> {
        let mcu = self
            .chip
            .mcu()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))?;
        for (name, args) in self.take_pending_filter_messages()? {
            mcu.send(name, &args)?;
        }
        let completion = self.dispatch.start(print_time)?;
        let clock = self
            .chip
            .print_time_to_clock(print_time)
            .ok_or_else(|| McuError::Config("trigger_analog has no clock estimate".to_string()))?;
        *self.arm_clock.lock().unwrap_or_else(|p| p.into_inner()) = Some(clock);
        let monitor_ticks = mcu.seconds_to_clock(1.0 / self.samples_per_second)? as u32;
        mcu.send_msg(&TriggerAnalogHome {
            oid: self.oid,
            trsync_oid: self.dispatch.get_oid(),
            trigger_reason: TriggerReason::EndstopHit as u8,
            error_reason: REASON_TRIGGER_ANALOG,
            clock: clock as u32,
            monitor_ticks,
            monitor_max: MONITOR_MAX,
        })?;
        Ok(completion)
    }

    /// Map the firmware's 32-bit `homing_clock` onto the 64-bit clock of the
    /// move that was armed — [`McuEndstop::trigger_clock`](super::endstop::McuEndstop)
    /// has the same problem and the same answer.
    fn trigger_clock(&self, clock32: u32) -> Option<i64> {
        let arm = *self.arm_clock.lock().unwrap_or_else(|p| p.into_inner());
        match arm {
            Some(arm) => {
                let diff = i64::from(clock32.wrapping_sub(arm as u32) as i32);
                Some(arm as i64 + diff)
            }
            None => self.chip.clock32_to_clock64(clock32),
        }
    }

    /// Stop homing and read back the trigger clock (upstream `_clear_home`,
    /// `trigger_analog.py:365-369`): an all-zero `trigger_analog_home`
    /// disables the check, and `trigger_analog_state` carries the arm or
    /// trigger clock as a print time.
    async fn clear_home(&self) -> Result<f64, McuError> {
        let mcu = self
            .chip
            .mcu()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))?;
        mcu.send_msg(&TriggerAnalogHome::disable(self.oid))?;
        let state = mcu
            .call_msg::<TriggerAnalogQueryState, TriggerAnalogState>(
                &TriggerAnalogQueryState { oid: self.oid },
                QUERY_TIMEOUT,
            )
            .await?;
        let clock = self.trigger_clock(state.homing_clock).unwrap_or(0);
        Ok(self.chip.clock_to_print_time(clock).unwrap_or(0.0))
    }

    /// The upstream error message for a `trigger_analog_error:` code
    /// (`trigger_analog.py:377-393`).
    ///
    /// Codes at or above `SENSOR_SPECIFIC` belong to the sensor and go through
    /// the installed lookup; without one they keep the dictionary's own name.
    /// The names come from the dictionary enumeration, exactly as upstream
    /// loads `self._error_map` from it.
    fn error_text(&self, mcu: &Mcu, error_code: u8) -> String {
        let dictionary = mcu.dictionary();
        let enumeration = dictionary
            .as_ref()
            .and_then(|dict| dict.enumeration("trigger_analog_error:"));
        let sensor_specific = enumeration
            .and_then(|entries| entries.value("SENSOR_SPECIFIC"))
            .unwrap_or(0);
        if i64::from(error_code) >= sensor_specific {
            if let Some(lookup) = self
                .sensor_error
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .as_ref()
            {
                return lookup(error_code.wrapping_sub(sensor_specific.clamp(0, 255) as u8));
            }
        }
        match enumeration.and_then(|entries| entries.name(i64::from(error_code))) {
            Some(name) => name.to_string(),
            None => format!("Unknown code {error_code}"),
        }
    }

    /// Wait for the probing move's end (`MCU_trigger_analog.home_wait`).
    ///
    /// Returns the print time the trigger fired at, or `0.0` when the move
    /// ended without one. A firmware error — raw range, filter overflow, a
    /// quiet sensor — is raised as the upstream message.
    ///
    /// # Errors
    /// Returns [`McuError`] on a communication timeout, a `trigger_analog`
    /// error code, or a missing answer.
    pub async fn home_wait(&self, home_end_time: f64) -> Result<f64, McuError> {
        let mcu = self
            .chip
            .mcu()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))?;
        self.dispatch.wait_end(home_end_time);
        let raw = self.dispatch.completion().wait_raw().await;
        // Clear the homing state before judging the reason: the trigger clock
        // is read back here, and upstream runs `_clear_home` first too
        // (`trigger_analog.py:365-369`).
        let trigger_time = self.clear_home().await?;
        self.dispatch.stop();
        if raw_is_failure(raw) {
            if raw == TriggerReason::CommsTimeout as u8 {
                return Err(McuError::Config(
                    "Communication timeout during homing".to_string(),
                ));
            }
            let error_code = raw - REASON_TRIGGER_ANALOG;
            return Err(McuError::Config(format!(
                "Trigger analog error: {}",
                self.error_text(&mcu, error_code)
            )));
        }
        if raw != TriggerReason::EndstopHit as u8 {
            return Ok(0.0);
        }
        *self
            .last_trigger_time
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = trigger_time;
        Ok(trigger_time)
    }
}

/// The second implementation of the homing channel: upstream drives
/// `MCU_trigger_analog` through the same `homing.probing_move` endstop
/// interface as `MCU_endstop` (`probe_eddy_current.py:752, 902`), so an
/// eddy-class probe's endstop reaches the very same probing move a `[probe]`
/// switch does.
impl HomingEndstop for McuTriggerAnalog {
    fn home_start(
        &self,
        print_time: f64,
        sample_time: f64,
        sample_count: u8,
        rest_time: f64,
        triggered: bool,
    ) -> Result<Arc<Completion>, McuError> {
        McuTriggerAnalog::home_start(
            self,
            print_time,
            sample_time,
            sample_count,
            rest_time,
            triggered,
        )
    }

    fn home_wait(&self, home_end_time: f64) -> EndstopFuture<'_> {
        Box::pin(McuTriggerAnalog::home_wait(self, home_end_time))
    }

    fn dispatch(&self) -> Option<&TriggerDispatch> {
        Some(McuTriggerAnalog::dispatch(self))
    }

    /// Whether the probe reads triggered now: an analog trigger has no pin to
    /// sample, so — exactly as upstream's virtual probe helper without a
    /// query callback (`probe.py:HomingViaProbeHelper.query_endstop` →
    /// `False`) — it reports "open".
    fn query_endstop(&self, _print_time: f64) -> QueryEndstopFuture<'_> {
        Box::pin(async { Ok(false) })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::core::klippy::cmd::clock::McuClock;
    use crate::core::klippy::interface::devices::frame_mock::FrameMock;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::{BuiltConfig, ConfigBuilder, Dictionary, Mcu};
    use crate::core::klippy::msg::proto::ArgValue;
    use crate::core::klippy::pins::PrinterPins;
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::json;

    /// A dictionary with the trigger_analog/sos_filter/trsync messages and the
    /// firmware's error enumeration.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "allocate_oids count=%c": 2,
                "get_config": 7,
                "finalize_config crc=%u": 6,
                "config_sos_filter oid=%c max_sections=%c": 40,
                "sos_filter_set_section oid=%c section_idx=%c sos0=%i sos1=%i sos2=%i sos3=%i sos4=%i": 41,
                "sos_filter_set_state oid=%c section_idx=%c state0=%i state1=%i": 42,
                "sos_filter_set_offset_scale oid=%c offset=%i scale=%i scale_frac_bits=%c auto_offset=%c": 43,
                "sos_filter_set_active oid=%c n_sections=%c coeff_frac_bits=%c": 44,
                "config_trigger_analog oid=%c sos_filter_oid=%c": 45,
                "trigger_analog_set_raw_range oid=%c raw_min=%i raw_max=%i": 46,
                "trigger_analog_set_trigger oid=%c trigger_analog_type=%c trigger_value=%i": 47,
                "trigger_analog_home oid=%c trsync_oid=%c trigger_reason=%c error_reason=%c clock=%u monitor_ticks=%u monitor_max=%u": 48,
                "trigger_analog_query_state oid=%c": 49,
                "config_trsync oid=%c": 30,
                "trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c": 31,
                "trsync_set_timeout oid=%c clock=%u": 32,
                "trsync_trigger oid=%c reason=%c": 33,
                "stepper_stop_on_trigger oid=%c trsync_oid=%c": 34
            },
            "responses": {
                "trigger_analog_state oid=%c homing=%c homing_clock=%u": 50,
                "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u": 35,
                "config is_config=%c crc=%u is_shutdown=%c move_count=%hu": 9
            },
            "enumerations": {
                "trigger_analog_error:": {"RAW_RANGE": 0, "OVERFLOW": 1, "MONITOR": 2, "SENSOR_SPECIFIC": 3},
                "trigger_analog_type": {"abs_ge": 0, "gt": 1, "diff_peak_gt": 2}
            },
            "config": {"CLOCK_FREQ": 1_000_000}
        }))
        .unwrap()
    }

    /// A chip over an identified test MCU with a 1 MHz clock.
    fn chip() -> (McuChip, Arc<Mcu>) {
        let mcu = Arc::new(Mcu::for_test(
            "mcu",
            Interface::new(FrameMock::new(Vec::new())),
        ));
        mcu.install_dictionary(dictionary()).unwrap();
        let chip = McuChip::new(
            "mcu".to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::new(PrinterPins::new()),
        );
        let clock = Arc::new(McuClock::new(Arc::clone(&mcu), ManualReactor::shared()));
        clock.seed(0.0, 0);
        chip.set_clock(clock, 0.0);
        chip.attach(Arc::clone(&mcu));
        (chip, mcu)
    }

    /// The `(name, arguments)` of every config command the build produced.
    fn built_commands(mcu: &Mcu, built: &BuiltConfig) -> Vec<(String, Vec<ArgValue>)> {
        let mut parser = crate::core::klippy::msg::parser::Parser::new();
        mcu.dictionary().unwrap().install(&mut parser).unwrap();
        built
            .config
            .iter()
            .map(|payload| {
                let frame = crate::core::klippy::frame::Frame::new(0, payload.payload().to_vec());
                let decoded = parser.decode(frame.into()).unwrap();
                (decoded[0].0.name.clone(), decoded[0].1.clone())
            })
            .collect()
    }

    /// The byte argument at `index`, for assertions over built commands.
    fn u8_arg(args: &[ArgValue], index: usize) -> u8 {
        match args[index] {
            ArgValue::UInt8(value) => value,
            ref other => panic!("argument {index} is not a byte: {other:?}"),
        }
    }

    #[tokio::test]
    async fn test_two_trigger_analogs_get_distinct_oids_and_one_config_each() {
        let (chip, mcu) = chip();
        let filter = SosFilter::new(&chip, 4).unwrap();
        let a = McuTriggerAnalog::new(chip.clone(), 400.0, Some(Arc::clone(&filter))).unwrap();
        let b = McuTriggerAnalog::new(chip.clone(), 400.0, None).unwrap();
        assert_ne!(a.oid(), b.oid(), "two objects must not share an oid");
        assert_ne!(a.oid(), filter.oid());

        let built = chip.config().build(&mcu).unwrap();
        let commands = built_commands(&mcu, &built);

        let config_sos: Vec<_> = commands
            .iter()
            .filter(|(name, _)| name == "config_sos_filter")
            .collect();
        // `a` shares the explicitly built filter; `b` made its own pass-through
        // one — exactly one config per SosFilter, no duplicates.
        assert_eq!(config_sos.len(), 2, "{commands:?}");

        let config_ta: Vec<_> = commands
            .iter()
            .filter(|(name, _)| name == "config_trigger_analog")
            .collect();
        assert_eq!(config_ta.len(), 2, "{commands:?}");
        // Each object names its own filter's oid.
        let binds: Vec<(u8, u8)> = config_ta
            .iter()
            .map(|(_, args)| (u8_arg(args, 0), u8_arg(args, 1)))
            .collect();
        assert_eq!(
            binds,
            vec![(a.oid(), filter.oid()), (b.oid(), b.sos_filter().oid())]
        );
        assert_eq!(config_sos.len(), 2);
        // And each config_sos_filter names a distinct oid with the right size.
        let sos_oids: Vec<(u8, u8)> = config_sos
            .iter()
            .map(|(_, args)| (u8_arg(args, 0), u8_arg(args, 1)))
            .collect();
        assert!(sos_oids.contains(&(filter.oid(), 4)), "{sos_oids:?}");
        assert!(
            sos_oids.contains(&(b.sos_filter().oid(), 0)),
            "{sos_oids:?}"
        );
    }

    #[tokio::test]
    async fn test_error_codes_decode_to_the_four_dictionary_names() {
        let (chip, mcu) = chip();
        let ta = McuTriggerAnalog::new(chip, 400.0, None).unwrap();
        // Reasons 5-8 carry codes 0-3 (`REASON_TRIGGER_ANALOG` + the enum).
        assert_eq!(ta.error_text(&mcu, 0), "RAW_RANGE");
        assert_eq!(ta.error_text(&mcu, 1), "OVERFLOW");
        assert_eq!(ta.error_text(&mcu, 2), "MONITOR");
        assert_eq!(ta.error_text(&mcu, 3), "SENSOR_SPECIFIC");
        // A code the dictionary does not name renders like upstream's default.
        assert_eq!(ta.error_text(&mcu, 9), "Unknown code 9");
    }

    #[tokio::test]
    async fn test_sensor_specific_codes_go_through_the_sensor_lookup() {
        let (chip, mcu) = chip();
        let ta = McuTriggerAnalog::new(chip, 400.0, None).unwrap();
        let seen = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&seen);
        ta.set_sensor_error_lookup(move |code| {
            sink.lock().unwrap().push(code);
            format!("ldc1612_error {code}")
        });
        // SENSOR_SPECIFIC is 3: code 3+ decodes as the sensor's own code below
        // it, codes below it still use the dictionary.
        assert_eq!(ta.error_text(&mcu, 3), "ldc1612_error 0");
        assert_eq!(ta.error_text(&mcu, 5), "ldc1612_error 2");
        assert_eq!(ta.error_text(&mcu, 2), "MONITOR");
        assert_eq!(*seen.lock().unwrap(), vec![0, 2]);
    }

    #[tokio::test]
    async fn test_sos_filter_resends_only_what_changed() {
        let (chip, _mcu) = chip();
        let filter = SosFilter::new(&chip, 2).unwrap();
        filter.set_filter_design(SosFilterDesign {
            sections: vec![[100, -200, 300, -400, 500], [1, 2, 3, 4, 5]],
            states: vec![[7, -7], [8, -8]],
            offset: -50,
            scale: 3,
            scale_frac_bits: 2,
            auto_offset: false,
            coeff_frac_bits: 18,
        });

        let first: Vec<&str> = filter
            .take_pending_messages()
            .unwrap()
            .into_iter()
            .map(|(name, _)| name)
            .collect();
        assert_eq!(
            first,
            vec![
                "sos_filter_set_section",
                "sos_filter_set_section",
                "sos_filter_set_state",
                "sos_filter_set_state",
                "sos_filter_set_offset_scale",
                "sos_filter_set_active",
            ]
        );

        // Identical design: coefficients and offset/scale are cached, states
        // and activation always go (the firmware deactivated the filter).
        let second: Vec<&str> = filter
            .take_pending_messages()
            .unwrap()
            .into_iter()
            .map(|(name, _)| name)
            .collect();
        assert_eq!(
            second,
            vec![
                "sos_filter_set_state",
                "sos_filter_set_state",
                "sos_filter_set_active"
            ]
        );

        // One coefficient changes: only that section is resent.
        let mut design = SosFilterDesign {
            sections: vec![[100, -200, 300, -400, 501], [1, 2, 3, 4, 5]],
            states: vec![[7, -7], [8, -8]],
            offset: -50,
            scale: 3,
            scale_frac_bits: 2,
            auto_offset: false,
            coeff_frac_bits: 18,
        };
        filter.set_filter_design(design.clone());
        let third: Vec<&str> = filter
            .take_pending_messages()
            .unwrap()
            .into_iter()
            .map(|(name, _)| name)
            .collect();
        assert_eq!(
            third,
            vec![
                "sos_filter_set_section",
                "sos_filter_set_state",
                "sos_filter_set_state",
                "sos_filter_set_active"
            ]
        );

        // auto_offset always re-sends the offset/scale.
        design.auto_offset = true;
        filter.set_filter_design(design);
        let names: Vec<&str> = filter
            .take_pending_messages()
            .unwrap()
            .into_iter()
            .map(|(name, _)| name)
            .collect();
        assert!(names.contains(&"sos_filter_set_offset_scale"), "{names:?}");
    }

    #[tokio::test]
    async fn test_sos_filter_rejects_too_many_sections_and_state_mismatch() {
        let (chip, _mcu) = chip();
        let filter = SosFilter::new(&chip, 1).unwrap();

        filter.set_filter_design(SosFilterDesign {
            sections: vec![[0; 5], [0; 5]],
            states: vec![[0; 2], [0; 2]],
            ..SosFilterDesign::default()
        });
        let err = filter.take_pending_messages().unwrap_err();
        assert!(
            err.to_string().contains("Too many filter sections"),
            "{err}"
        );

        filter.set_filter_design(SosFilterDesign {
            sections: vec![[0; 5]],
            states: vec![[0; 2], [0; 2]],
            ..SosFilterDesign::default()
        });
        let err = filter.take_pending_messages().unwrap_err();
        assert!(err.to_string().contains("must be equal"), "{err}");
    }

    #[tokio::test]
    async fn test_range_and_trigger_are_sent_once_until_changed() {
        let (chip, _mcu) = chip();
        let ta = McuTriggerAnalog::new(chip, 400.0, None).unwrap();
        ta.set_raw_range(-1_000_000, 1_000_000);
        ta.set_trigger(TriggerAnalogType::Gt, 1500);

        let first: Vec<&str> = ta
            .take_pending_filter_messages()
            .unwrap()
            .into_iter()
            .map(|(name, _)| name)
            .collect();
        assert_eq!(
            first,
            vec![
                "trigger_analog_set_raw_range",
                "trigger_analog_set_trigger",
                "sos_filter_set_offset_scale"
            ]
        );

        // Nothing changed: the next arm sends only the filter's activation
        // state — and with the default pass-through filter (max_sections = 0)
        // nothing at all.
        let second = ta.take_pending_filter_messages().unwrap();
        assert!(second.is_empty(), "{second:?}");

        // Changing the trigger sends just that one message.
        ta.set_trigger(TriggerAnalogType::DiffPeakGt, 42);
        let third: Vec<&str> = ta
            .take_pending_filter_messages()
            .unwrap()
            .into_iter()
            .map(|(name, _)| name)
            .collect();
        assert_eq!(third, vec!["trigger_analog_set_trigger"]);
    }

    #[tokio::test]
    async fn test_a_non_positive_sample_rate_is_refused() {
        let (chip, _mcu) = chip();
        let err = match McuTriggerAnalog::new(chip, 0.0, None) {
            Err(err) => err,
            Ok(_) => panic!("a zero sample rate must be refused"),
        };
        assert!(err.to_string().contains("samples_per_second"), "{err}");
    }

    // -----------------------------------------------------------------------
    // The resource end to end against the fake firmware
    // -----------------------------------------------------------------------

    use crate::core::klippy::cmd::stepper::ResetStepClock;
    use crate::core::klippy::interface::SimulatorDevice;
    use std::time::Duration;

    /// Tears the fake-firmware session down when the test's scope ends.
    ///
    /// Two things have to happen before the runtime is dropped, and both are
    /// easy to forget at the end of a test body — which is why they live in a
    /// `Drop`:
    ///
    /// * **Break the `Mcu → events → resource → Mcu` cycle.**
    ///   `chip.config().configure()` binds `trsync_state`, whose handler owns
    ///   the `TrsyncRegistry` that owns the `McuTrsync` holding this `Mcu`
    ///   through the chip — the same strong cycle production breaks in
    ///   `McuObject::release_cycles`. Without it no handle is the last one,
    ///   `Mcu::Drop` never runs and the session leaks.
    /// * **Release the parked device read.** `SimulatorDevice::receive()`
    ///   runs inside `spawn_blocking` and parks on a condvar until
    ///   `Mcu::close()` shuts the device down. A blocking task that is already
    ///   running cannot be cancelled, so the runtime's blocking pool waits for
    ///   it **forever** when it is alive at shutdown — hanging the whole test
    ///   binary, not just this test. Whether the worker had picked the read up
    ///   yet is a race, so this shows up as a flake: under CPU pressure the
    ///   test thread is preempted, the worker wins the race, and the run hangs.
    ///   Closing in `Drop` also covers a failing assertion: unwinding tears the
    ///   session down instead of turning the failure into a hang.
    ///
    /// Bind it to a **named** variable — `let _ = …` would drop it on the spot.
    /// It comes first in the tuple so that it drops last (locals drop in
    /// reverse declaration order); the order is not load-bearing — running the
    /// teardown while the resource and the chip are still alive closes the
    /// session just the same — it only keeps the teardown reading in the order
    /// production does it.
    struct SessionGuard(Arc<Mcu>);

    impl Drop for SessionGuard {
        fn drop(&mut self) {
            self.0.clear_events();
            self.0.close();
        }
    }

    /// An identified fake MCU with the resource configured on a chip.
    ///
    /// The [`SessionGuard`] returned first must stay bound for the whole test.
    async fn resource_harness(
        samples_per_second: f64,
    ) -> (SessionGuard, Arc<Mcu>, McuChip, McuTriggerAnalog) {
        let device =
            SimulatorDevice::new(klipperx_test_support::test_dicts_dir().join("atmega2560.dict"))
                .expect("the corpus dictionary");
        let mcu = Mcu::connect("mcu", Interface::simulator(device))
            .await
            .expect("identify against the fake firmware");
        let pins = Arc::new(PrinterPins::new());
        let chip = McuChip::new(
            "mcu".to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::clone(&pins),
        );
        let clock = Arc::new(McuClock::new(Arc::clone(&mcu), ManualReactor::shared()));
        clock.seed(0.0, 0);
        chip.set_clock(clock, 0.0);
        chip.attach(Arc::clone(&mcu));
        let ta = McuTriggerAnalog::new(chip.clone(), samples_per_second, None)
            .expect("build the resource");
        chip.config()
            .configure(&mcu)
            .await
            .expect("configure against the fake firmware");
        // Keep the pin registry alive for the test's duration.
        std::mem::forget(pins);
        (SessionGuard(Arc::clone(&mcu)), mcu, chip, ta)
    }

    #[tokio::test]
    async fn test_home_completes_when_the_move_starts() {
        let (_session, mcu, _chip, ta) = resource_harness(400.0).await;
        ta.set_raw_range(-2_000_000, 2_000_000);
        ta.set_trigger(TriggerAnalogType::Gt, 42);

        let completion = ta.home_start(1.0, 0.0, 0, 0.0, true).unwrap();
        // The first move after arming is the trigger.
        mcu.send_msg(&ResetStepClock { oid: 0, clock: 0 }).unwrap();

        let time = tokio::time::timeout(Duration::from_secs(5), ta.home_wait(2.0))
            .await
            .expect("fires within the monitor window")
            .expect("a hit is not an error");
        assert!(time > 0.9 && time < 1.1, "trigger print time: {time}");
        assert_eq!(ta.last_trigger_time(), time);
        assert_eq!(completion.reason(), Some(TriggerReason::EndstopHit));
    }

    #[tokio::test]
    async fn test_monitor_timeout_fails_home_wait() {
        // 400 Hz → a monitor window of 4 x 2.5 ms; no move ever starts, so the
        // fake's monitor expiry must wake `home_wait` with the upstream error.
        let (_session, _mcu, _chip, ta) = resource_harness(400.0).await;
        ta.home_start(0.01, 0.0, 0, 0.0, true).unwrap();

        let err = tokio::time::timeout(Duration::from_secs(5), ta.home_wait(1.0))
            .await
            .expect("the monitor expiry completes the wait")
            .expect_err("a quiet sensor is an error");
        assert!(
            err.to_string().contains("Trigger analog error: MONITOR"),
            "{err}"
        );
    }

    #[tokio::test]
    async fn test_it_arms_and_waits_through_the_homing_endstop_trait() {
        let (_session, mcu, _chip, ta) = resource_harness(400.0).await;
        ta.set_raw_range(-2_000_000, 2_000_000);
        ta.set_trigger(TriggerAnalogType::Gt, 42);

        // The very channel `probing_move` drives `McuEndstop` over — the
        // second implementation now reaches it too.
        let endstop: &dyn HomingEndstop = &ta;
        let completion = endstop.home_start(1.0, 0.0, 0, 0.0, true).unwrap();
        mcu.send_msg(&ResetStepClock { oid: 0, clock: 0 }).unwrap();

        let time = tokio::time::timeout(Duration::from_secs(5), endstop.home_wait(2.0))
            .await
            .expect("fires within the monitor window")
            .expect("a hit is not an error");
        assert!(time > 0.9 && time < 1.1, "trigger print time: {time}");
        assert_eq!(completion.reason(), Some(TriggerReason::EndstopHit));
    }

    /// The guard is what makes the harness's session droppable: `configure()`
    /// leaves `Mcu → events → resource → Mcu` as a strong cycle, so without
    /// `clear_events()` no handle is ever the last one and `Mcu::Drop` — the
    /// backstop that runs `close()` — never fires. Dropping the guard mid-test
    /// must therefore leave no live handle at all.
    #[tokio::test]
    async fn test_the_harness_guard_releases_the_session() {
        let (session, mcu, chip, ta) = resource_harness(400.0).await;
        let weak = Arc::downgrade(&mcu);
        drop(ta);
        drop(chip);
        drop(mcu);
        drop(session);
        assert!(
            weak.upgrade().is_none(),
            "the guard must break the cycle, so the session can drop"
        );
    }

    #[tokio::test]
    async fn test_the_stub_sensor_feeds_a_session_and_decodes_its_error_codes() {
        use crate::core::klippy::extras::probe::SampleDelivery;

        let (chip, mcu) = chip();
        let ta = McuTriggerAnalog::new(chip, 400.0, None).unwrap();

        // The stub stands in for the sensor half of the seam (the ldc1612
        // producer lands with the eddy probe): its samples reach an open
        // probe session…
        struct StubSession {
            samples: Mutex<Vec<(f64, f64)>>,
        }
        impl SampleDelivery for StubSession {
            fn deliver_sample(&self, time: f64, value: f64) {
                self.samples.lock().unwrap().push((time, value));
            }
        }
        let session = Arc::new(StubSession {
            samples: Mutex::new(Vec::new()),
        });
        let delivery: Arc<dyn SampleDelivery> = Arc::clone(&session) as Arc<dyn SampleDelivery>;
        delivery.deliver_sample(1.5, 654_321.0);
        delivery.deliver_sample(1.5025, 654_000.0);
        assert_eq!(
            *session.samples.lock().unwrap(),
            vec![(1.5, 654_321.0), (1.5025, 654_000.0)]
        );

        // …and its error-code decoder is what `set_sensor_error_lookup`
        // hands the firmware's sensor-specific codes (`SENSOR_SPECIFIC` = 3
        // in the dictionary below).
        ta.set_sensor_error_lookup(|code| format!("stub sensor error {code}"));
        assert_eq!(ta.error_text(&mcu, 3), "stub sensor error 0");
        assert_eq!(ta.error_text(&mcu, 5), "stub sensor error 2");
        // Codes below `SENSOR_SPECIFIC` still name the dictionary's own.
        assert_eq!(ta.error_text(&mcu, 2), "MONITOR");
    }
}
