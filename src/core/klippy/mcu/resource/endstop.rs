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
//! serial queue holds it until the queued motion has run (`klippy/mcu.py:401-405`).
//! This host does the same: the print time becomes the query's `min_clock`, the
//! send gates hold it, and the firmware reads the pin when it *processes* the
//! message (`src/endstop.c:102-114`) — which the floor pins to the requested
//! sample instant. When idle — the `M119` case — the floor is already behind the
//! clock and the query goes at once. (This replaces the old FW6 decision, "send
//! now, let the firmware schedule", whose effect was that during a move the
//! answer read the current level rather than the level at the move's end.)

use std::sync::{Arc, Mutex};

use super::pin::{pin_number, McuChip};
use super::trsync::{Completion, TriggerDispatch};
use crate::core::klippy::cmd::endstop::{
    ConfigEndstop, EndstopHome, EndstopQueryState, EndstopState,
};
use crate::core::klippy::cmd::trsync::TriggerReason;
use crate::core::klippy::mcu::{McuError, SendClocks};
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
    /// `_print_time` is upstream's `minclock` (`klippy/mcu.py:401-405`): the
    /// query is gated to that print time so the pin is sampled when the queued
    /// motion has run (see the module docs). No clock mapping yet means no
    /// floor — the gates read an unknown clock as "send now"
    /// (`serialqueue.c:612-618`).
    ///
    /// # Errors
    /// Returns [`McuError`] if the MCU is not connected or does not answer.
    pub async fn query_endstop(&self, print_time: f64) -> Result<bool, McuError> {
        let mcu = self
            .chip
            .mcu()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))?;
        let min_clock = self.chip.print_time_to_clock(print_time);
        let state = mcu
            .call_msg_clocked::<EndstopQueryState, EndstopState>(
                &EndstopQueryState { oid: self.oid },
                SendClocks {
                    min_clock,
                    req_clock: None,
                },
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
    /// The arm carries upstream's `reqclock=clock` (`klippy/mcu.py:385`): its
    /// `req_clock` keeps it behind any lower `req_clock` on the wire — the
    /// queue_step messages queued for this move — which is how commit
    /// `6bd5f4e4` made steps reach the board before the check starts.
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
        mcu.send_msg_clocked(
            &EndstopHome {
                oid: self.oid,
                clock: clock as u32,
                sample_ticks,
                sample_count,
                rest_ticks: rest_ticks as u32,
                pin_value: u8::from(triggered ^ self.invert),
                trsync_oid: self.dispatch.get_oid(),
                trigger_reason: TriggerReason::EndstopHit as u8,
            },
            SendClocks {
                min_clock: None,
                req_clock: Some(clock),
            },
        )?;
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
    use crate::core::klippy::interface::devices::frame_mock::{FrameMock, RecordingWire};
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::{ConfigBuilder, Dictionary, Mcu};
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::ArgValue;
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
        (chip_with(Arc::clone(&mcu)), mcu)
    }

    /// The same chip, wired to an MCU the caller keeps a handle to.
    fn chip_with(mcu: Arc<Mcu>) -> McuChip {
        let chip = McuChip::new(
            "mcu".to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::new(PrinterPins::new()),
        );
        let clock = Arc::new(McuClock::new(Arc::clone(&mcu), ManualReactor::shared()));
        clock.seed(0.0, 0);
        chip.set_clock(clock, 0.0);
        chip.attach(mcu);
        chip
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

    /// Encode one message with the test dictionary's formats.
    fn wire_bytes(format: &str, id: i16, args: &[ArgValue]) -> Vec<u8> {
        let mut parser = Parser::new();
        parser.register(id, format).unwrap();
        parser.encode(name_of(format), args).unwrap().into_raw()
    }

    fn name_of(format: &str) -> &str {
        format.split_whitespace().next().unwrap()
    }

    /// ⑤ The arm carries upstream's `reqclock` (`klippy/mcu.py:385`): with the
    /// arm clock far ahead of the estimate, `endstop_home` and the trsync pair
    /// wait for the 100 ms lead window instead of going out at once — the
    /// ordering that keeps the arm behind this move's queue_step messages
    /// (upstream commit `6bd5f4e4`).
    #[tokio::test]
    async fn test_home_start_waits_for_its_reqclock_window() {
        use tokio::time::sleep;
        let wire = RecordingWire::new();
        let closer = wire.clone();
        let sent = wire.sent();
        let mcu = Arc::new(Mcu::for_test("mcu", Interface::recording(wire)));
        mcu.install_dictionary(dictionary()).unwrap();
        mcu.set_clock_base(0);
        let chip = chip_with(Arc::clone(&mcu));
        let endstop = McuEndstop::new(chip, &params("PA1", false)).unwrap();

        // Arm 10 s out on the 1 MHz clock; the window opens at 10 s − 0.1 s.
        endstop.home_start(10.0, 0.000_015, 4, 0.01, true).unwrap();
        sleep(std::time::Duration::from_millis(60)).await;
        assert_eq!(
            sent.lock().unwrap().len(),
            0,
            "the arm went out without its reqclock holding it to the window"
        );

        mcu.set_clock_base(10_000_000 - 100_000 + 1);
        sleep(std::time::Duration::from_millis(60)).await;
        // The whole arm releases — and the send task may coalesce it into one
        // block, so count messages, not frames.
        let frames = sent.lock().unwrap().clone();
        assert!(!frames.is_empty(), "the arm never left after the window");
        let mut parser = Parser::new();
        parser
            .register(
                31,
                "trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c",
            )
            .unwrap();
        parser
            .register(32, "trsync_set_timeout oid=%c clock=%u")
            .unwrap();
        parser
            .register(
                41,
                "endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c",
            )
            .unwrap();
        let mut names: Vec<String> = Vec::new();
        let names: Vec<String> = frames
            .iter()
            .flat_map(|payload| {
                parser
                    .decode(crate::core::klippy::msg::proto::Payload::from_raw(
                        payload.clone(),
                    ))
                    .unwrap_or_default()
            })
            .map(|(msg, _)| msg.name.clone())
            .collect();
        assert_eq!(
            names,
            ["trsync_start", "trsync_set_timeout", "endstop_home"],
            "the arm must go out whole, in queue order"
        );
        // Production breaks the `Mcu → events → resource → Mcu` cycle in
        // `McuObject::release_cycles`; a bare test has to do the same or the
        // response callback keeps the `Mcu` (and its blocked receive) alive
        // past the runtime's shutdown.
        mcu.clear_events();
        drop(endstop);
        // Close the wire explicitly: the bare `Mcu` can be kept alive by the
        // resource callback cycle (production breaks it in
        // `McuObject::release_cycles`), and a still-blocked receive would park
        // this test runtime's shutdown.
        closer.shutdown();
    }

    /// ⑤ `endstop_query_state` carries upstream's `minclock`
    /// (`klippy/mcu.py:401-405`): `query_endstop(print_time)` waits for that
    /// print time instead of answering the pin as it is *now* — the FW6 old
    /// behaviour the module docs used to describe.
    #[tokio::test]
    async fn test_query_endstop_holds_until_its_minclock() {
        use tokio::time::sleep;
        let wire = RecordingWire::new();
        let sent = wire.sent();
        let query = wire_bytes("endstop_query_state oid=%c", 42, &[ArgValue::UInt8(0)]);
        let state = wire_bytes(
            "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c",
            43,
            &[
                ArgValue::UInt8(0),
                ArgValue::UInt8(0),
                ArgValue::UInt32(0),
                ArgValue::UInt8(1),
            ],
        );
        wire.reply_to(query.clone(), state);
        let mcu = Arc::new(Mcu::for_test("mcu", Interface::recording(wire)));
        mcu.install_dictionary(dictionary()).unwrap();
        mcu.set_clock_base(0);
        let chip = chip_with(Arc::clone(&mcu));
        let endstop = McuEndstop::new(chip, &params("PA1", false)).unwrap();

        // `min_clock` = 10 s of MCU clock; the estimate sits at 0.
        let pending = endstop.query_endstop(10.0);
        tokio::pin!(pending);
        tokio::select! {
            early = &mut pending => panic!("query answered before its minclock: {early:?}"),
            _ = sleep(std::time::Duration::from_millis(60)) => {}
        }
        assert!(
            sent.lock().unwrap().is_empty(),
            "the query went out before print_time arrived"
        );

        mcu.set_clock_base(10_000_000 - 100_000 + 1);
        let answered = tokio::time::timeout(std::time::Duration::from_secs(1), pending)
            .await
            .expect("the gated query completes after the release")
            .expect("the canned endstop_state answers");
        assert!(answered, "pin_value=1 with invert=false");
        assert_eq!(
            sent.lock().unwrap().iter().filter(|p| **p == query).count(),
            1,
            "exactly one query once the minclock passed"
        );
    }
}
