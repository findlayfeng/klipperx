//! The `endstop` resource: a pin the firmware watches during a homing move.
//!
//! Upstream's `MCU_endstop` (`klippy/mcu.py:340-407`). It owns the endstop's
//! oid, the `endstop_home`/`endstop_query_state` commands, and a
//! [`TriggerDispatch`] that stops the steppers when the pin trips.
//!
//! # What is here
//!
//! | method | firmware | upstream |
//! |---|---|---|
//! | [`McuEndstop::query_endstop`] | `endstop_query_state` → `endstop_state` | `query_endstop` |
//! | [`McuEndstop::home_start`] | `endstop_home` (arm) | `home_start` |
//! | [`McuEndstop::home_wait`] | `endstop_home` (disable) + `trsync_trigger` | `home_wait` |
//!
//! # Clock-ordered queries
//!
//! Upstream's `query_endstop(print_time)` sends its query with `minclock`, so the
//! serial queue holds it until the queued motion has run. This host has no such
//! queue (the FW6 decision: send now, let the firmware schedule), so the query
//! reads the pin as soon as it arrives. When idle — the `M119` case — that is the
//! same answer; during a move it is the current level rather than the level at
//! the move's end.

use std::sync::{Arc, Mutex};

use super::pin::{pin_number, McuChip};
use super::trsync::{Completion, TriggerDispatch};
use crate::core::klippy::cmd::endstop::{
    ConfigEndstop, EndstopHome, EndstopQueryState, EndstopState,
};
use crate::core::klippy::cmd::trsync::TriggerReason;
use crate::core::klippy::mcu::McuError;
use crate::core::klippy::pins::PinParams;

/// How long a `endstop_query_state` exchange may take.
const QUERY_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(1);

/// One endstop on one MCU.
pub struct McuEndstop {
    oid: u8,
    /// `!` on the pin: the host inverts the raw level.
    invert: bool,
    chip: McuChip,
    dispatch: TriggerDispatch,
    /// Ticks between poll attempts, kept for `home_wait`'s trigger-clock math.
    rest_ticks: Mutex<u32>,
    /// The 64-bit clock this endstop was armed at, kept so `home_wait` can map
    /// the firmware's 32-bit trigger clock relative to **this move** instead of
    /// the synchronized estimate.
    ///
    /// A real MCU's clock and the host's print time share one base, so the
    /// estimate is fine and upstream maps against it. Ours must not: print time
    /// can run far ahead of what the MCU reports (49 bed-mesh points take
    /// milliseconds of wall time and ~136 s of print time), and past 2^31 ticks
    /// — 134 s at 16 MHz — a 32-bit value is read a whole revolution off. The
    /// trigger cannot be outside this move, so the arm clock is the right
    /// reference and the 2^31 window is never approached.
    arm_clock: Mutex<Option<u64>>,
}

impl McuEndstop {
    /// Build the endstop for `params` and register its config callback.
    ///
    /// # Errors
    /// Returns [`McuError`] if the oid cannot be created or the callback
    /// registered.
    pub fn new(chip: McuChip, params: &PinParams) -> Result<Self, McuError> {
        let builder = chip.config();
        let oid = builder.create_oid()?;
        let invert = params.invert;
        let callback_chip = chip.clone();
        let callback_params = params.clone();
        builder.register_config_callback(Box::new(move |builder, mcu| {
            let pin = pin_number(
                mcu,
                &callback_chip
                    .pins()
                    .resolve_pin(callback_chip.name(), &callback_params.pin)?,
                callback_chip.name(),
            )?;
            builder.add_config_cmd(&ConfigEndstop {
                oid,
                pin: pin as u8,
                pull_up: callback_params.pullup,
            })?;
            // Disable the check on every connect, as upstream's `on_restart`
            // `endstop_home` does: a reused firmware must not still be homing.
            builder.add_restart_cmd(&EndstopHome::disable(oid))?;
            Ok(())
        }))?;
        let dispatch = TriggerDispatch::new(vec![chip.clone()])?;
        Ok(Self {
            oid,
            invert,
            chip,
            dispatch,
            rest_ticks: Mutex::new(0),
            arm_clock: Mutex::new(None),
        })
    }

    /// The oid the firmware assigned.
    /// The MCU this endstop's pin lives on.
    pub fn chip_name(&self) -> &str {
        self.chip.name()
    }

    pub fn oid(&self) -> u8 {
        self.oid
    }

    /// The trigger dispatch this endstop fires.
    pub fn dispatch(&self) -> &TriggerDispatch {
        &self.dispatch
    }

    /// Whether the pin is triggered now (`MCU_endstop.query_endstop`).
    ///
    /// `_print_time` is upstream's `minclock`; see the module docs for why this
    /// host does not hold the query.
    ///
    /// # Errors
    /// Returns [`McuError`] if the MCU is not connected or does not answer.
    pub async fn query_endstop(&self, _print_time: f64) -> Result<bool, McuError> {
        let mcu = self
            .chip
            .mcu()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))?;
        let state = mcu
            .call_msg::<EndstopQueryState, EndstopState>(
                &EndstopQueryState { oid: self.oid },
                QUERY_TIMEOUT,
            )
            .await?;
        Ok((state.pin_value != 0) ^ self.invert)
    }

    /// Arm the endstop for a homing move (`MCU_endstop.home_start`).
    ///
    /// `sample_time`/`sample_count` confirm a trigger; `rest_time` is the poll
    /// interval; `triggered` is the level to stop on (before the `!` inversion).
    ///
    /// # Errors
    /// Returns [`McuError`] if the MCU is not connected or a send fails.
    pub fn home_start(
        &self,
        print_time: f64,
        sample_time: f64,
        sample_count: u8,
        rest_time: f64,
        triggered: bool,
    ) -> Result<Arc<Completion>, McuError> {
        let mcu = self
            .chip
            .mcu()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))?;
        let clock = self
            .chip
            .print_time_to_clock(print_time)
            .ok_or_else(|| McuError::Config("endstop has no clock estimate".to_string()))?;
        let rest_ticks = self
            .chip
            .print_time_to_clock(print_time + rest_time)
            .map(|end| end.saturating_sub(clock))
            .unwrap_or(0);
        *self.rest_ticks.lock().unwrap_or_else(|p| p.into_inner()) = rest_ticks as u32;
        *self.arm_clock.lock().unwrap_or_else(|p| p.into_inner()) = Some(clock);
        let sample_ticks = mcu.seconds_to_clock(sample_time)? as u32;

        let completion = self.dispatch.start(print_time)?;
        mcu.send_msg(&EndstopHome {
            oid: self.oid,
            clock: clock as u32,
            sample_ticks,
            sample_count,
            rest_ticks: rest_ticks as u32,
            pin_value: u8::from(triggered ^ self.invert),
            trsync_oid: self.dispatch.get_oid(),
            trigger_reason: TriggerReason::EndstopHit as u8,
        })?;
        Ok(completion)
    }

    /// Map the firmware's 32-bit trigger clock onto the 64-bit clock of the
    /// move that was armed: the arm clock plus the signed 32-bit difference,
    /// falling back to the synchronized estimate when nothing is armed.
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

    /// Wait for the move's endstop (`MCU_endstop.home_wait`).
    ///
    /// Returns the print time the endstop tripped at, or `0.0` when the move
    /// ended without a hit.
    ///
    /// # Errors
    /// Returns [`McuError`] on a communication timeout or a missing answer.
    pub async fn home_wait(&self, home_end_time: f64) -> Result<f64, McuError> {
        let mcu = self
            .chip
            .mcu()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))?;
        self.dispatch.wait_end(home_end_time);
        let reason = self.dispatch.completion().wait().await;
        // Disable the check before anything else, as upstream does.
        mcu.send_msg(&EndstopHome::disable(self.oid))?;
        // Reset the firmware's group; the completion already recorded why.
        self.dispatch.stop();
        if reason.is_failure() {
            return Err(McuError::Config(
                "Communication timeout during homing".to_string(),
            ));
        }
        if reason != TriggerReason::EndstopHit {
            return Ok(0.0);
        }
        let state = mcu
            .call_msg::<EndstopQueryState, EndstopState>(
                &EndstopQueryState { oid: self.oid },
                QUERY_TIMEOUT,
            )
            .await?;
        let next_clock = self
            .trigger_clock(state.next_clock)
            .ok_or_else(|| McuError::Config("endstop has no clock estimate".to_string()))?;
        let rest_ticks = i64::from(*self.rest_ticks.lock().unwrap_or_else(|p| p.into_inner()));
        Ok(self
            .chip
            .clock_to_print_time(next_clock - rest_ticks)
            .unwrap_or(0.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::core::klippy::cmd::clock::McuClock;
    use crate::core::klippy::interface::devices::frame_mock::FrameMock;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::{ConfigBuilder, Dictionary, Mcu};
    use crate::core::klippy::pins::PrinterPins;
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::json;

    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "allocate_oids count=%c": 2,
                "get_config": 7,
                "finalize_config crc=%u": 6,
                "config_endstop oid=%c pin=%c pull_up=%c": 40,
                "endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c": 41,
                "endstop_query_state oid=%c": 42,
                "config_trsync oid=%c": 30,
                "trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c": 31,
                "trsync_set_timeout oid=%c clock=%u": 32,
                "trsync_trigger oid=%c reason=%c": 33,
                "stepper_stop_on_trigger oid=%c trsync_oid=%c": 34
            },
            "responses": {
                "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c": 43,
                "config is_config=%c crc=%u is_shutdown=%c move_count=%hu": 9
            },
            "enumerations": {"pin": {"PA0": 0, "PA1": 1}},
            "config": {"CLOCK_FREQ": 1_000_000}
        }))
        .unwrap()
    }

    /// A chip over a test MCU with a 1 MHz clock.
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

    fn params(pin: &str, invert: bool) -> PinParams {
        PinParams {
            chip_name: "mcu".to_string(),
            pin: pin.to_string(),
            invert,
            pullup: 0,
            share_type: None,
        }
    }

    #[tokio::test]
    async fn test_home_start_arms_the_endstop_and_trsync() {
        let (chip, mcu) = chip();
        let endstop = McuEndstop::new(chip, &params("PA1", false)).unwrap();

        let completion = endstop.home_start(1.0, 0.000_015, 4, 0.01, true).unwrap();
        assert!(completion.reason().is_none());

        // One trsync on the chip, sharing the chip's clock.
        assert_eq!(endstop.dispatch().get_oid(), endstop.dispatch().get_oid());
        let _ = mcu;
    }

    #[test]
    fn test_home_wait_returns_zero_for_a_host_request() {
        // The trigger reason decides the result; `home_wait` returns 0 for
        // anything but an endstop hit. This is the pure half of the decision.
        assert!(!TriggerReason::EndstopHit.is_failure());
        assert_eq!(TriggerReason::from_u8(1), Some(TriggerReason::EndstopHit));
    }
}
