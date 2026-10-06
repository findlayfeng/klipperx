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
    use crate::core::klippy::cmd::config::Reset;
    use crate::core::klippy::config::{Config, ConfigWrapper};
    use crate::core::klippy::extras::board_pins::BoardPins;
    use crate::core::klippy::interface::devices::frame_mock::{FrameMock, RecordingWire};
    use crate::core::klippy::interface::{Interface, SimulatorDevice};
    use crate::core::klippy::mcu::{ConfigBuilder, Dictionary, Mcu};
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::ArgValue;
    use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
    use crate::core::klippy::printer::{Printer, PrinterObject};
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::json;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::time::Duration;

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
        // Close the session explicitly: the receive runs inside
        // `spawn_blocking`, and `Mcu::close` releases that parked read (its
        // own view of the wire stops reading; the wire itself stays up).
        mcu.close();
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

    // -----------------------------------------------------------------------
    // Real board: R3, the X endstop's level and polarity (M119)
    // -----------------------------------------------------------------------

    /// How long a rebooted board gets before its port is reopened.
    ///
    /// The same order as `mcu/object.rs`'s `RESET_SETTLE` (500 ms): a board
    /// that reboots on `reset` is not ready the moment the command has been
    /// flushed, and reopening its port too early races its startup. Kept local
    /// because that constant is private to the bring-up.
    const REBOOT_SETTLE: Duration = Duration::from_millis(500);

    /// How long `reset` has to leave the send queue before the port is closed.
    ///
    /// `mcu/object.rs`'s own `RESET_FLUSH_TIMEOUT` (1 s): the firmware reboots
    /// on receipt, so the command must be flushed while the transport is still
    /// up. The flush itself often reports the port closing as the board goes
    /// away — the command was still written first, so that is not an error.
    const RESET_FLUSH: Duration = Duration::from_secs(1);

    /// What the R3 real-board case needs from `KLIPPERX_HW_CONFIG`.
    ///
    /// `[stepper_x]` is the section the X endstop belongs to and `endstop_pin`
    /// is the key that says which pin it is; a config that wires no X endstop
    /// (sensorless, or a printer without X) is skipped by
    /// [`crate::hardware_test::acquire`]. Shared with
    /// [`test_the_endstop_case_declares_stepper_x_endstop_pin`], so the
    /// declaration the config is judged against and the one the test runs under
    /// cannot drift apart.
    fn endstop_case() -> crate::hardware_test::Requires {
        crate::hardware_test::Requires::new()
            .mcu()
            .option("stepper_x", "endstop_pin")
    }

    /// A builder whose only content is `oids` oids: two counts make two CRCs,
    /// which is all a test needs to be "a configuration the board does not
    /// carry".
    fn builder_with_oids(oids: u32) -> Arc<ConfigBuilder> {
        let builder = Arc::new(ConfigBuilder::new());
        for _ in 0..oids {
            builder
                .create_oid()
                .expect("the oid fits in allocate_oids' count");
        }
        builder
    }

    /// Leave `mcu` carrying `builder_with_oids(oids)`'s configuration.
    async fn configure_the_fake(mcu: &Arc<Mcu>, oids: u32) {
        let builder = builder_with_oids(oids);
        let mut built = builder.build(mcu).expect("the dictionary encodes it");
        builder
            .handshake(mcu, &mut built, false)
            .await
            .expect("the fake firmware accepts the configuration");
    }

    /// Connect to the board and make it carry `builder`'s configuration,
    /// rebooting it once if it still carries another.
    ///
    /// `Mcu::connect` completes identify; it does not configure the firmware,
    /// and a resource built on a chip sends nothing until `config` /
    /// `finalize_config` have been accepted. A board a running printer host has
    /// already configured — its CRC is not the one this host just computed —
    /// refuses the configuration until it is rebooted, and a firmware whose
    /// only reset is its own `reset` reports `McuError::ResetRequired`
    /// (`mcu/config.rs`). This does what `mcu/object.rs` does for the printer:
    /// send `reset`, flush it, close the session, let the board come back, and
    /// hand the **same** `BuiltConfig` to [`ConfigBuilder::handshake`] on the
    /// reconnected session.
    ///
    /// One retry, then it gives up with a message that names the likely cause.
    /// Each attempt prints what it found, so `--nocapture` shows whether the
    /// board was taken over or came up free.
    ///
    /// Written for the R3 endstop case here; the same take-over is what the
    /// firmware-restart, motion and long-run cases will need, so this is the
    /// piece to lift into `crate::hardware_test` when it grows a bring-up.
    async fn configure_taking_over<'a, F, Fut>(
        name: &str,
        mut mcu: Arc<Mcu>,
        mut reopen: F,
        builder: &ConfigBuilder,
    ) -> Result<Arc<Mcu>, String>
    where
        F: FnMut() -> Fut + 'a,
        Fut: std::future::Future<Output = Result<Arc<Mcu>, String>> + 'a,
    {
        let mut built = builder.build(&mcu).map_err(|err| err.to_string())?;
        let mut reboots = 0;
        loop {
            match builder.handshake(&mcu, &mut built, false).await {
                Ok(_) => {
                    println!("HW-CONFIG: {name}: the firmware accepted the configuration");
                    return Ok(mcu);
                }
                Err(McuError::ResetRequired) if reboots == 0 => {
                    println!(
                        "HW-CONFIG: {name}: the firmware still carries a configuration; \
                         rebooting it with `reset` and reconnecting (attempt 2)"
                    );
                    mcu.send_msg(&Reset).map_err(|err| err.to_string())?;
                    let _ = mcu.flush(RESET_FLUSH).await;
                    mcu.close();
                    tokio::time::sleep(REBOOT_SETTLE).await;
                    mcu = reopen().await?;
                    reboots += 1;
                }
                Err(McuError::ResetRequired) => {
                    return Err(format!(
                        "{name}: the firmware still carries a configuration after `reset`; \
                         a running printer host may still hold the board — stop it (and the \
                         printer) and run this again"
                    ));
                }
                Err(err) => {
                    return Err(format!("{name}: the configuration was refused: {err}"));
                }
            }
        }
    }

    /// The pin registry a config's `endstop_pin` resolves against, and the main
    /// MCU's chip and config builder to build on.
    ///
    /// The printer's loader builds one chip per `[mcu]` section and applies
    /// every `[board_pins]` alias before any resource resolves a pin; a
    /// hardware test that builds one resource reproduces that much of it, so an
    /// `endstop_pin` written as an alias (`^X_STOP`) resolves the way it does at
    /// run time.
    fn pin_world(config: &Config) -> (Arc<PrinterPins>, McuChip, Arc<ConfigBuilder>) {
        let pins = Arc::new(PrinterPins::new());
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let registered: Arc<dyn PrinterObject> = pins.clone();
        printer
            .add_object(PINS_OBJECT, registered)
            .expect("the pin registry is not registered twice");
        let mut main = None;
        for section in config.get_sections_by_id("mcu") {
            let name = section.sub.clone().unwrap_or_else(|| "mcu".to_string());
            let builder = Arc::new(ConfigBuilder::new());
            let chip = McuChip::new(name.clone(), Arc::clone(&builder), Arc::clone(&pins));
            pins.register_chip(&name, Arc::new(chip.clone()))
                .expect("one chip per [mcu] section");
            if name == "mcu" {
                main = Some((chip, builder));
            }
        }
        let (chip, builder) = main.expect("acquire required a reachable [mcu]");
        for section in config.get_sections_by_id("board_pins") {
            BoardPins::new(&ConfigWrapper::untracked(section), &printer)
                .expect("the [board_pins] aliases apply to the registered chips");
        }
        (pins, chip, builder)
    }

    /// One query, printed, so `--nocapture` shows both sides of the flip.
    ///
    /// No clock estimate is installed on the chip, so `print_time_to_clock`
    /// has nothing to map and the query carries no `min_clock`: this is the
    /// idle `M119` case, which goes out at once because there is no queued
    /// motion for it to wait behind (see the module docs' "Clock-ordered
    /// queries").
    async fn read_endstop_level(endstop: &McuEndstop, label: &str) -> bool {
        let level = endstop
            .query_endstop(0.0)
            .await
            .expect("the firmware answers endstop_query_state");
        println!("HW-ENDSTOP: {label}: {}", state_name(level));
        level
    }

    /// The two levels as the API prints them: `open` is the untriggered pin,
    /// `TRIGGERED` the level the endstop stops on (upstream `QueryEndstops`, the
    /// `M119` and `query_endstops/status` wording).
    fn state_name(level: bool) -> &'static str {
        if level {
            "TRIGGERED"
        } else {
            "open"
        }
    }

    /// Wait for the operator to press Enter, without blocking the runtime.
    ///
    /// The blocking read runs on a blocking thread, so the MCU's transport
    /// tasks keep being polled while the operator works. EOF (a redirected
    /// stdin) returns at once rather than hanging.
    async fn wait_for_operator(prompt: &str) {
        use std::io::Write;
        println!("{prompt}");
        let _ = std::io::stdout().flush();
        let _ = tokio::task::spawn_blocking(|| {
            let mut line = String::new();
            let _ = std::io::stdin().read_line(&mut line);
        })
        .await;
    }

    /// R3: the X endstop's level under the two physical states, read twice.
    ///
    /// This is the real-board half of `query_endstop`'s story (the fake-device
    /// half is `test_query_endstop_holds_until_its_minclock`): it configures the
    /// board with **only** the X endstop resource and asks the firmware for the
    /// pin level twice, in between letting the operator change the wiring.
    ///
    /// **Premise**: the config is assumed correct; this case checks this
    /// repository's endstop query path, not your config — a mismatch of config,
    /// wiring or firmware is not a bug here.
    ///
    /// # What it asserts, and what only the operator can confirm
    ///
    /// `TESTING.md`'s R3 (`TESTING.md:43-46`) reads "X endstop read once in each
    /// of 断开 / 手动短接; 判定: `open` ↔ `TRIGGERED` flips with the real level, and
    /// the `!`/`^` polarity and pull-up match the config". Split by what the host
    /// can see:
    ///
    /// - **Asserted here**: both reads return (the `endstop_query_state`
    ///   round trip completed), and `query_endstop` maps the firmware's
    ///   `pin_value` through the pin's `!` exactly as production does. The two
    ///   results are printed as `open` / `TRIGGERED`.
    /// - **The operator's call**: that the level *changed* between the two
    ///   reads, and that it changed in the direction the wiring and the `!`/`^`
    ///   in `endstop_pin` promise. Two identical reads are *reported* with a
    ///   plain-language remind, never failed: with no motion and no queued
    ///   move, a level that does not move means the wiring did not change, not
    ///   that the query is wrong. `^`/`~` are passed to the firmware as
    ///   `config_endstop`'s `pull_up` and never reach the host, so they are
    ///   checked on the board, not here.
    ///
    /// # What it assumes about the wiring
    ///
    /// The X endstop's signal is wired to the pin `[stepper_x] endstop_pin`
    /// names, on the main `[mcu]` board, and the operator can open and short
    /// that pin for the two reads. `^`/`~` (pull-up / pull-down) and `!`
    /// (invert) are taken from the config as written. Nothing else — no
    /// stepper, no heater — has to be wired for this case.
    ///
    /// # What it is safe to run
    ///
    /// **No motion, no heater.** Nothing is armed and no `queue_step` is sent —
    /// the only command that reaches the firmware after configuration is
    /// `endstop_query_state`, and `query_endstop` never touches a stepper or a
    /// heater. It reads the X endstop of the config's own `[stepper_x]`.
    ///
    /// # What it does to the board
    ///
    /// It takes the board over: the printer (or any other host) must not be
    /// running, because a second session on the board is exactly what the
    /// configuration handshake cannot share. The board ends up carrying this
    /// test's configuration (the endstop resource and nothing else) and is
    /// **not** restored — the printer resets it on its next start, or a
    /// power-cycle does. A board that is already configured is rebooted once
    /// over its own `reset` before the test's configuration is sent.
    ///
    /// Run it with the board's X endstop readable, one hand free to move the
    /// jumper:
    ///
    /// ```text
    /// KLIPPERX_HW_CONFIG=~/printer_data/config/printer.cfg \
    ///   cargo test -p klipperx --lib test_endstop_level_reads_open_and_shorted \
    ///   -- --ignored --nocapture
    /// ```
    #[tokio::test]
    #[ignore = "hardware: needs KLIPPERX_HW_CONFIG"]
    async fn test_endstop_level_reads_open_and_shorted_on_a_real_board() {
        let Some(machine) = crate::hardware_test::acquire(
            "test_endstop_level_reads_open_and_shorted_on_a_real_board",
            &endstop_case(),
        ) else {
            return;
        };
        let config = machine.config();
        let pin = config
            .get_section("stepper_x")
            .and_then(|section| section.get_str("endstop_pin"))
            .expect("[stepper_x] endstop_pin, which acquire required")
            .to_string();

        let (pins, chip, builder) = pin_world(config);
        let endstop = pins
            .setup_endstop(&pin, None)
            .expect("endstop_pin names a pin and a chip this config has");
        assert_eq!(
            endstop.chip_name(),
            "mcu",
            "this case declares .mcu(); an X endstop on another board needs its own declaration"
        );

        let mcu = Mcu::connect("mcu", machine.open_mcu().expect("the port opens"))
            .await
            .expect("identify completes");
        let reopen = || async {
            let interface = machine.open_mcu()?;
            Mcu::connect("mcu", interface)
                .await
                .map_err(|err| err.to_string())
        };
        let mcu = configure_taking_over("mcu", mcu, reopen, &builder)
            .await
            .expect("the board carries this configuration");
        chip.attach(mcu);

        println!(
            "HW-ENDSTOP: endstop_pin = {pin}; the host inverts the level: {}",
            endstop.invert
        );
        let first = read_endstop_level(&endstop, "read 1").await;
        wait_for_operator(
            "HW-ENDSTOP: change the X endstop's physical state now \
             (open it if it is shorted, short it if it is open), then press Enter",
        )
        .await;
        let second = read_endstop_level(&endstop, "read 2").await;

        if first == second {
            println!(
                "HW-ENDSTOP: 未检测到电平变化（两次都是 {}）：请确认端停是否已按需断开/短接后重跑",
                state_name(second)
            );
        } else {
            println!(
                "HW-ENDSTOP: 电平翻转确认：{} -> {}",
                state_name(first),
                state_name(second)
            );
        }
    }

    /// The take-over helper against a fake firmware: a board that already
    /// carries a configuration is rebooted once and then accepts this one.
    ///
    /// `atmega2560.dict` declares `reset` and no `config_reset`, which is the
    /// firmware whose only way out of a configuration is to reboot itself —
    /// the one `ConfigBuilder::handshake` answers with `ResetRequired`.
    #[tokio::test]
    async fn test_the_take_over_helper_reboots_a_configured_board_once() {
        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        let mcu = Mcu::connect(
            "mcu",
            Interface::simulator(SimulatorDevice::new(&dict).unwrap()),
        )
        .await
        .expect("identify against the fake firmware");
        // Leave the fake configured, and with a different CRC than the builder
        // below computes — what a running printer host leaves behind.
        configure_the_fake(&mcu, 1).await;
        let builder = builder_with_oids(2);

        let reconnects = Arc::new(AtomicUsize::new(0));
        let counted = Arc::clone(&reconnects);
        let reopened = configure_taking_over(
            "mcu",
            mcu,
            move || {
                let counted = Arc::clone(&counted);
                let dict = dict.clone();
                async move {
                    counted.fetch_add(1, Ordering::SeqCst);
                    let interface = Interface::simulator(SimulatorDevice::new(&dict).unwrap());
                    Mcu::connect("mcu", interface)
                        .await
                        .map_err(|err| err.to_string())
                }
            },
            &builder,
        )
        .await
        .expect("the reboot brings back a board that takes the configuration");

        assert_eq!(
            reconnects.load(Ordering::SeqCst),
            1,
            "the helper must reconnect exactly once"
        );
        assert_eq!(reopened.name(), "mcu");
    }

    /// The same helper when the board never comes back clean: the retry is
    /// bounded, and the failure names the likely cause.
    #[tokio::test]
    async fn test_the_take_over_helper_gives_up_after_one_reboot() {
        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        let mcu = Mcu::connect(
            "mcu",
            Interface::simulator(SimulatorDevice::new(&dict).unwrap()),
        )
        .await
        .expect("identify against the fake firmware");
        configure_the_fake(&mcu, 1).await;
        let builder = builder_with_oids(2);

        let err = configure_taking_over(
            "mcu",
            mcu,
            move || {
                let dict = dict.clone();
                async move {
                    // The board comes back still carrying a configuration, so
                    // the second handshake cannot get past it either.
                    let interface = Interface::simulator(SimulatorDevice::new(&dict).unwrap());
                    let mcu = Mcu::connect("mcu", interface)
                        .await
                        .map_err(|err| err.to_string())?;
                    configure_the_fake(&mcu, 1).await;
                    Ok(mcu)
                }
            },
            &builder,
        )
        .await
        .expect_err("a board that keeps its configuration is reported, not retried forever");

        assert!(
            err.contains("still carries a configuration after `reset`"),
            "{err}"
        );
        assert!(err.contains("printer host"), "{err}");
    }

    /// The declaration the real-board case runs under is the one the config is
    /// judged against: `[stepper_x]` with `endstop_pin` activates it, a
    /// commented-out or missing pair skips it.
    ///
    /// This is the half of a hardware test no board is needed for, and it is
    /// the one that keeps a wrong declaration from turning the case into one
    /// that is *always* skipped.
    #[test]
    fn test_the_endstop_case_declares_stepper_x_endstop_pin() {
        let with_it =
            Config::from_text("[mcu]\nserial: /dev/fake\n[stepper_x]\nendstop_pin: ^PA2\n")
                .expect("the fixture parses")
                .0;
        assert_eq!(
            crate::hardware_test::check(&with_it, &endstop_case()),
            Ok(())
        );

        let commented =
            Config::from_text("# [stepper_x]\n# endstop_pin: ^PA2\n[mcu]\nserial: /dev/fake\n")
                .expect("the fixture parses")
                .0;
        assert_eq!(
            crate::hardware_test::check(&commented, &endstop_case()),
            Err(vec![crate::hardware_test::Missing::Section(
                "stepper_x".to_string()
            )])
        );

        let without_it =
            Config::from_text("[mcu]\nserial: /dev/fake\n[stepper_x]\nstep_pin: PA0\n")
                .expect("the fixture parses")
                .0;
        assert_eq!(
            crate::hardware_test::check(&without_it, &endstop_case()),
            Err(vec![crate::hardware_test::Missing::Option {
                section: "stepper_x".to_string(),
                option: "endstop_pin".to_string(),
            }])
        );
    }
}
