//! The `trsync` resource: a trigger group that stops steppers together.
//!
//! Upstream's `MCU_trsync` / `TriggerDispatch` (`klippy/mcu.py:155-339`) plus the
//! C `chelper/trdispatch.c`. A homing move arms the endstop to fire a `trsync`
//! when the pin trips; the firmware then stops every stepper registered as a
//! signal, and the host keeps the group's deadline extended while the firmware
//! reports it is still alive.
//!
//! # What this port changes
//!
//! Upstream's `trdispatch.c` is threads + a `serialqueue` fastreader, and its
//! cross-MCU "extend using the slowest MCU's acknowledged time" logic is in C.
//! Here it is plain Rust:
//!
//! * [`TriggerGroup`] holds the `McuTrsync`s of one dispatch and the completion;
//! * a per-MCU [`TrsyncRegistry`] routes `trsync_state` by oid, through **one**
//!   handler for the whole MCU (our `Mcu::bind_event` keeps one handler per
//!   message name, so several endstops on one MCU must share it). The handler is
//!   bound per **arm**, not per configuration: an MCU's event table belongs to
//!   one connection, and this host keeps a section connected across a firmware
//!   reset by replacing the session (`McuTrsync::start`, upstream
//!   `MCU_trsync.start`);
//! * on each report the group recomputes the minimum acknowledged print time and
//!   resends `trsync_set_timeout` when the extension is worth it
//!   (`handle_trsync_state`, `chelper/trdispatch.c:70-140`).
//!
//! Multi-MCU is the same code path: one `McuTrsync` per MCU, each with its own
//! `McuChip` clock, and the group takes the slowest.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;

use tokio::sync::Notify;
use tokio::time::Instant;

use super::pin::{McuChip, TrsyncChip};
use super::stepper::McuStepper;
use crate::core::klippy::cmd::stepper::StepperStopOnTrigger;
use crate::core::klippy::cmd::trsync::{
    ConfigTrsync, TriggerReason, TrsyncSetTimeout, TrsyncStart, TrsyncState, TrsyncTrigger,
};
use crate::core::klippy::error::ConfigError;
use crate::core::klippy::mcu::{ConfigBuilder, Mcu, McuError, SendClocks};

/// The deadline for a multi-MCU dispatch (`TRSYNC_TIMEOUT`, `klippy/mcu.py:279`).
pub const TRSYNC_TIMEOUT: f64 = 0.025;

/// The deadline for a single-MCU dispatch (`TRSYNC_SINGLE_MCU_TIMEOUT`,
/// `klippy/mcu.py:260`).
pub const TRSYNC_SINGLE_MCU_TIMEOUT: f64 = 0.250;

/// How long a homing dispatch may go without a `trsync_state` report before its
/// wait fails as a communication timeout.
///
/// Upstream has no host-side bound on this wait: its firmware deadline
/// (`expire_clock`, `TRSYNC_TIMEOUT`/`TRSYNC_SINGLE_MCU_TIMEOUT` above) ends a
/// live dispatch, and the report that follows completes it. What it does bound
/// is the *missing answer* — a retried query gives up after
/// `RetryAsyncCommand.TIMEOUT_TIME` (`klippy/mcu.py:41`), the same 5 s this host
/// uses for `QUERY_TIMEOUT` elsewhere. The report is exactly such an answer, so
/// a dispatch that has been silent that long is treated the same way: the wait
/// fails with [`TriggerReason::CommsTimeout`] (`home_wait` raises upstream's
/// "Communication timeout during homing", `klippy/mcu.py:393`) instead of
/// parking the g-code script that started the homing forever.
///
/// The clock it measures is **silence**, not the length of the move: every
/// report pushes it out (see [`Completion::refresh_deadline`]).
pub const HOME_WAIT_TIMEOUT: Duration = Duration::from_secs(5);

/// A one-shot completion the homing move waits on.
///
/// The `McuTrsync` that sees `can_trigger=0` completes it; `home_wait` awaits.
/// `Notify` plus the stored reason, so a completion that already happened is
/// seen by a waiter that arrives afterwards.
///
/// The stored value is the **raw** wire reason. Most callers read the typed
/// [`wait`](Completion::wait), which maps 1-4 through [`TriggerReason`];
/// consumers of higher codes (`trigger_analog`'s error reasons) read
/// [`wait_raw`](Completion::wait_raw) instead.
#[derive(Debug, Default)]
pub struct Completion {
    raw: Mutex<Option<u8>>,
    notify: Notify,
    /// When a wait gives up, in the reactor's time: [`HOME_WAIT_TIMEOUT`] of
    /// silence. [`TriggerDispatch::start`] arms it and every `trsync_state`
    /// report pushes it out; `None` — a completion no dispatch owns, such as a
    /// test endstop's — waits forever, as before.
    deadline: Mutex<Option<Instant>>,
}

impl Completion {
    /// An incomplete completion.
    pub fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    /// Clear it for a new dispatch.
    pub fn reset(&self) {
        *self.raw.lock().unwrap_or_else(|p| p.into_inner()) = None;
        *self.deadline.lock().unwrap_or_else(|p| p.into_inner()) = None;
    }

    /// Start the silence clock a dispatch's wait is bounded by: unless a report
    /// pushes it out first, the wait fails [`HOME_WAIT_TIMEOUT`] from now.
    pub fn arm_deadline(&self) {
        *self.deadline.lock().unwrap_or_else(|p| p.into_inner()) =
            Some(Instant::now() + HOME_WAIT_TIMEOUT);
    }

    /// Push the clock out: called for every report the firmware makes, so the
    /// bound covers silence rather than the length of the move.
    pub fn refresh_deadline(&self) {
        let mut deadline = self.deadline.lock().unwrap_or_else(|p| p.into_inner());
        if deadline.is_some() {
            *deadline = Some(Instant::now() + HOME_WAIT_TIMEOUT);
        }
    }

    /// How long a wait may still sleep before it gives up.
    fn remaining(&self) -> Option<Duration> {
        self.deadline
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .map(|deadline| deadline.saturating_duration_since(Instant::now()))
    }

    /// Complete it with `reason`; the first reason stands.
    pub fn complete(&self, reason: TriggerReason) {
        self.complete_raw(reason as u8);
    }

    /// Complete it with a raw wire reason; the first reason stands.
    ///
    /// Signals with `notify_waiters()`: only waiters that already registered
    /// are woken and no permit is stored, so [`wait`](Completion::wait_raw)
    /// registers **before** it reads the state (the race is noted there).
    ///
    /// Reasons 5 and above (`trigger_analog` errors) have no typed variant but
    /// still complete the dispatch — dropping them would leave `home_wait`
    /// waiting forever.
    pub fn complete_raw(&self, raw: u8) {
        let mut guard = self.raw.lock().unwrap_or_else(|p| p.into_inner());
        if guard.is_none() {
            *guard = Some(raw);
            drop(guard);
            self.notify.notify_waiters();
        }
    }

    /// The raw reason, once completed.
    pub fn raw(&self) -> Option<u8> {
        *self.raw.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// The reason, once completed and only when it is one this host types.
    ///
    /// A raw reason outside 1-4 reads as `None` here even though the
    /// completion has fired; use [`raw`](Completion::raw) or
    /// [`wait_raw`](Completion::wait_raw) when higher codes are possible.
    pub fn reason(&self) -> Option<TriggerReason> {
        self.raw().and_then(TriggerReason::from_u8)
    }

    /// Wait for the completion and return its raw reason.
    ///
    /// A completion a dispatch owns is bounded by [`HOME_WAIT_TIMEOUT`] of
    /// silence: the wait then fails it with [`TriggerReason::CommsTimeout`],
    /// which is what a caller checking
    /// [`TriggerReason::is_failure`] reads as the upstream "Communication
    /// timeout during homing".
    pub async fn wait_raw(&self) -> u8 {
        loop {
            let notified = self.notify.notified();
            tokio::pin!(notified);
            // Register with the `Notify` **before** reading the state.
            //
            // `complete_raw` signals with `notify_waiters()`, which wakes only
            // the waiters already registered and stores no permit for later
            // ones; a `Notified` future does not register itself until it is
            // enabled or first polled. Reading `raw` first and awaiting after
            // would therefore lose a completion that lands in between: the
            // reason would be recorded, no waiter would be woken, and this
            // loop would sleep forever. `enable()` closes that window — the
            // state read below then either sees the reason (return it) or the
            // completion is guaranteed to wake us.
            notified.as_mut().enable();
            if let Some(raw) = self.raw() {
                return raw;
            }
            let Some(remaining) = self.remaining() else {
                notified.await;
                continue;
            };
            if tokio::time::timeout(remaining, notified).await.is_ok() {
                continue;
            }
            // The firmware has not reported for `HOME_WAIT_TIMEOUT`: the
            // dispatch's trigger report is lost, and waiting for it any longer
            // is what would leave the homing script — and the g-code command
            // that started it — parked for good. Record the failure the way a
            // late report would be recorded, so `reason()` and `stop()` see it
            // too, and let the caller's error path close the dispatch down.
            self.complete_raw(TriggerReason::CommsTimeout as u8);
            return self.raw().unwrap_or(TriggerReason::CommsTimeout as u8);
        }
    }

    /// Wait for the completion and return its typed reason.
    ///
    /// A raw reason outside 1-4 collapses onto [`TriggerReason::CommsTimeout`]
    /// — both mean the attempt failed; callers that must name the failure
    /// (e.g. `trigger_analog`'s four error codes) wait on
    /// [`wait_raw`](Completion::wait_raw) instead.
    pub async fn wait(&self) -> TriggerReason {
        TriggerReason::from_u8(self.wait_raw().await).unwrap_or(TriggerReason::CommsTimeout)
    }
}

/// The shared state of one dispatch: its trsyncs and completion.
pub struct TriggerGroup {
    trsyncs: Mutex<Vec<Arc<McuTrsync>>>,
    completion: Arc<Completion>,
    /// Whether the group can still trigger, as far as the host knows
    /// (`trdispatch.c:21`): set when a dispatch arms, cleared by the report
    /// that closes it, and read to drop reports a closed dispatch can no
    /// longer use (`trdispatch.c:78-79`).
    can_trigger: AtomicBool,
}

impl TriggerGroup {
    /// Close the group on a `can_trigger=0` report: it can no longer trigger,
    /// and every MCU in it is told to trigger.
    ///
    /// Upstream `handle_trsync_state` (`chelper/trdispatch.c:81-89`): the MCU
    /// that reported has already stopped watching — it hit the endstop or its
    /// expire timer fired — while the others may still be driving the move, so
    /// each one gets `trsync_trigger` (the reporting MCU included, as upstream's
    /// loop does) with `HOST_REQUEST`. Then the reason is recorded, after the
    /// triggers go out, so the completion never precedes the stop messages.
    fn close(&self, raw_reason: u8) {
        self.can_trigger.store(false, Ordering::SeqCst);
        let trsyncs: Vec<Arc<McuTrsync>> = self
            .trsyncs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        for trsync in &trsyncs {
            trsync.send_trigger(TriggerReason::HostRequest);
        }
        // Zero is not a reason the firmware reports, so it completes nothing,
        // as before.
        if raw_reason != 0 {
            self.completion.complete_raw(raw_reason);
        }
    }

    /// Recompute the minimum acknowledged time and extend each MCU's deadline
    /// (`handle_trsync_state`).
    ///
    /// Also pushes the host-side silence clock out: a report is the firmware
    /// saying the dispatch is still alive, which is what
    /// [`HOME_WAIT_TIMEOUT`] measures the absence of.
    fn on_report(&self) {
        self.completion.refresh_deadline();
        let trsyncs: Vec<Arc<McuTrsync>> = self
            .trsyncs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        // Each MCU's acknowledged print time; `None` when it has not reported.
        let mut times: Vec<Option<f64>> = Vec::with_capacity(trsyncs.len());
        for trsync in &trsyncs {
            times.push(trsync.acknowledged_print_time());
        }
        let min_time = times
            .iter()
            .flatten()
            .copied()
            .fold(f64::INFINITY, f64::min);
        if !min_time.is_finite() {
            return;
        }
        // The slowest MCU gets the next-slowest time, so it can catch up without
        // the others waiting for it (upstream's `min_tdm`/`next_min_time`).
        let next_min_time = times
            .iter()
            .flatten()
            .copied()
            .filter(|time| *time > min_time)
            .fold(f64::INFINITY, f64::min);
        let next_min_time = if next_min_time.is_finite() {
            next_min_time
        } else {
            min_time
        };
        let slowest = times
            .iter()
            .position(|time| *time == Some(min_time))
            .unwrap_or(0);
        for (index, trsync) in trsyncs.iter().enumerate() {
            let target = if index == slowest {
                next_min_time
            } else {
                min_time
            };
            trsync.extend_timeout(target);
        }
    }
}

/// One MCU's part of a dispatch.
pub struct McuTrsync {
    /// The oid `config_trsync` assigned.
    oid: u8,
    chip: TrsyncChip,
    /// The MCU's shared `trsync_state` router, for (re)binding on the session
    /// that is about to report.
    ///
    /// Strong, while the registry keeps its trsyncs [weak](Weak): that is the
    /// direction that neither leaves the registry without an owner nor makes
    /// `registry ↔ trsync` a cycle (the registry is per MCU and the trsyncs are
    /// what keep it alive; a trsync that reported into it strongly would keep
    /// the connected `Mcu`, and its device read, alive forever).
    registry: Arc<TrsyncRegistry>,
    group: Weak<TriggerGroup>,
    /// The steps that stop when the group fires, with the name their rail uses
    /// (for the multi-MCU shared-axis check).
    steppers: Mutex<Vec<(Weak<McuStepper>, String)>>,
    /// The current deadline and how far it may be extended.
    expire_clock: Mutex<u64>,
    expire_ticks: Mutex<u64>,
    min_extend_ticks: Mutex<u64>,
    /// The last clock the firmware acknowledged.
    last_status_clock: Mutex<Option<i64>>,
    /// When the move is planned to end; a report past it asks for a trigger.
    home_end_clock: Mutex<Option<i64>>,
}

impl McuTrsync {
    /// Build the trsync on `chip` and register `config_trsync` + the router.
    ///
    /// # Errors
    /// Returns [`McuError`] if the oid cannot be created or the callback
    /// registered.
    fn new(chip: McuChip, group: Weak<TriggerGroup>) -> Result<Arc<Self>, McuError> {
        let builder: Arc<ConfigBuilder> = chip.config();
        let oid = builder.create_oid()?;
        let registry = chip.trsync_registry();
        let trsync_chip = chip.trsync_chip();
        let trsync = Arc::new(Self {
            oid,
            chip: trsync_chip,
            registry: Arc::clone(&registry),
            group,
            steppers: Mutex::new(Vec::new()),
            expire_clock: Mutex::new(0),
            expire_ticks: Mutex::new(0),
            min_extend_ticks: Mutex::new(0),
            last_status_clock: Mutex::new(None),
            home_end_clock: Mutex::new(None),
        });
        let registered = Arc::clone(&trsync);
        let router = Arc::downgrade(&registry);
        builder.register_config_callback(Box::new(move |builder, _mcu| {
            builder.add_config_cmd(&ConfigTrsync { oid })?;
            // Disarm the group on **every** connect, as upstream's `on_restart`
            // `trsync_start … report_ticks=0` does (`klippy/mcu.py:189-193`).
            // Only `trsync_trigger` and a new `trsync_start` delete the
            // firmware's periodic report timer (`src/trsync.c:171-186`), so a
            // board that was reset while a homing move had one armed would
            // otherwise keep reporting on the session that replaces it.
            builder.add_restart_cmd(&TrsyncStart {
                oid,
                report_clock: 0,
                report_ticks: 0,
                expire_reason: 0,
            })?;
            // Clear the report timer **a killed previous process left on this
            // oid**: the firmware deletes a group's report and expire timers
            // only in `trsync_trigger` or on a new `trsync_start`
            // (`src/trsync.c:171-186`), so a host killed mid-homing leaves the
            // feed running — and `restart_method: admin` reconnects to that same
            // firmware instead of resetting it, on the same oid (`config_trsync`
            // is re-sent with the same number). A clean exit leaves nothing
            // behind, which is why the symptom only appears after a kill.
            //
            // The `init` list is what makes it deterministic: sent on **every**
            // connect, after `config_trsync` assigned the oid (`mcu/config.rs`,
            // upstream's `_send_config` `init` pass). It is safe on a group that
            // was never started: `trsync_do_trigger` skips the trigger itself
            // (`!(flags & TSF_CAN_TRIGGER)`), while both `sched_del_timer` calls
            // and the one `trsync_state` report still happen
            // (`src/trsync.c:31-53,171-186`) — that report reaches a dispatch
            // that is not armed, and a reason-2 report with `can_trigger=0` only
            // records a reason the next `start` resets.
            builder.add_init_cmd(&TrsyncTrigger {
                oid,
                reason: TriggerReason::HostRequest as u8,
            })?;
            if let Some(router) = router.upgrade() {
                router.register(&registered);
            }
            Ok(())
        }))?;
        Ok(trsync)
    }

    /// The oid the firmware assigned.
    pub fn oid(&self) -> u8 {
        self.oid
    }

    /// Register a stepper to stop when the group fires.
    ///
    /// A stepper already registered here is ignored, as upstream's
    /// `MCU_trsync.add_stepper` does (`klippy/mcu.py:183-186`): a rail registers
    /// all of its motors with its endstop and a virtual endstop's helper (the
    /// probe's `LookupZSteppers`) registers the same motors again, and the
    /// firmware must get one `stepper_stop_on_trigger` per motor, not two.
    ///
    /// The handle is weak: the oid is read when the group is armed, not now,
    /// because this happens at config-load time and the oid is only assigned
    /// when the configuration is built.
    pub fn add_stepper(&self, stepper: Weak<McuStepper>, name: &str) {
        let mut steppers = self.steppers.lock().unwrap_or_else(|p| p.into_inner());
        if steppers
            .iter()
            .any(|(known, _)| Weak::ptr_eq(known, &stepper))
        {
            return;
        }
        steppers.push((stepper, name.to_string()));
    }

    /// The `(name, is the stepper alive)` pairs registered on this trsync.
    fn stepper_names(&self) -> Vec<String> {
        self.steppers
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
            .map(|(_, name)| name.clone())
            .collect()
    }

    /// The last acknowledged print time for this MCU.
    fn acknowledged_print_time(&self) -> Option<f64> {
        let clock = (*self
            .last_status_clock
            .lock()
            .unwrap_or_else(|p| p.into_inner()))?;
        self.chip.clock_to_print_time(clock)
    }

    /// Arm the group (`MCU_trsync.start`).
    fn start(
        &self,
        print_time: f64,
        report_offset: f64,
        expire_timeout: f64,
    ) -> Result<(), McuError> {
        let mcu = self.require_mcu()?;
        // The report handler goes in **before** the arm messages: the firmware
        // may report as soon as it processes `trsync_start`, and the first
        // report is what completes a dispatch.
        self.bind_report_handler(&mcu)?;
        let freq = self.require_clock()?.estimator().mcu_freq();
        let clock = self
            .chip
            .print_time_to_clock(print_time)
            .ok_or_else(|| McuError::Config("trsync has no clock estimate".to_string()))?;
        let expire_ticks = (expire_timeout * freq) as u64;
        let expire_clock = clock + expire_ticks;
        let report_ticks = (expire_timeout * 0.3 * freq) as u64;
        let report_clock = clock + (report_ticks as f64 * report_offset + 0.5) as u64;
        let min_extend_ticks = (report_ticks as f64 * 0.8 + 0.5) as u64;
        *self.expire_clock.lock().unwrap_or_else(|p| p.into_inner()) = expire_clock;
        *self.expire_ticks.lock().unwrap_or_else(|p| p.into_inner()) = expire_ticks;
        *self
            .min_extend_ticks
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = min_extend_ticks;

        // Upstream arms the group with `reqclock=clock` (`klippy/mcu.py:260`):
        // the start waits behind the lower `req_clock`s of the queue_step
        // messages queued for this move (commit `6bd5f4e4`), so the trsync
        // never starts watching before the steps it has to stop exist.
        mcu.send_msg_clocked(
            &TrsyncStart {
                oid: self.oid,
                report_clock: report_clock as u32,
                report_ticks: report_ticks as u32,
                expire_reason: TriggerReason::CommsTimeout as u8,
            },
            SendClocks {
                min_clock: None,
                req_clock: Some(clock),
            },
        )?;
        for (stepper, _) in self
            .steppers
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
        {
            let Some(stepper) = stepper.upgrade() else {
                continue;
            };
            let oid = stepper.oid()?;
            mcu.send_msg(&StepperStopOnTrigger {
                oid,
                trsync_oid: self.oid,
            })?;
        }
        // Upstream sends this with `reqclock=clock` too — the **arm** clock,
        // not the expire one (`klippy/mcu.py:264`); the 25 ms of
        // `TRSYNC_TIMEOUT` only moves the release point inside the same
        // 100 ms lead window, and the arm clock is what keeps the message on
        // the homing priority line. (The C4 design text said "expire clock";
        // aligned to upstream per the close-to-upstream rule, noted here.)
        mcu.send_msg_clocked(
            &TrsyncSetTimeout {
                oid: self.oid,
                clock: expire_clock as u32,
            },
            SendClocks {
                min_clock: None,
                req_clock: Some(clock),
            },
        )?;
        Ok(())
    }

    /// Note when the move is planned to end (`set_home_end_time`).
    fn set_home_end(&self, end_time: f64) {
        *self
            .home_end_clock
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = self
            .chip
            .print_time_to_clock(end_time)
            .map(|clock| clock as i64);
    }

    /// Extend the deadline to `target_print_time` if it is worth a message.
    fn extend_timeout(&self, target_print_time: f64) {
        let Some(expire) = self.chip.print_time_to_clock(target_print_time) else {
            return;
        };
        let expire = expire + *self.expire_ticks.lock().unwrap_or_else(|p| p.into_inner());
        let current = *self.expire_clock.lock().unwrap_or_else(|p| p.into_inner());
        let min_extend = *self
            .min_extend_ticks
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        if (expire as i64 - current as i64) >= min_extend as i64 {
            *self.expire_clock.lock().unwrap_or_else(|p| p.into_inner()) = expire;
            if let Some(mcu) = self.chip.mcu() {
                let _ = mcu.send_msg(&TrsyncSetTimeout {
                    oid: self.oid,
                    clock: expire as u32,
                });
            }
        }
    }

    /// Send `trsync_trigger` for this trsync's MCU (`send_trsync_trigger`,
    /// `chelper/trdispatch.c:40-47`): fire-and-forget, no answer awaited.
    fn send_trigger(&self, reason: TriggerReason) {
        if let Some(mcu) = self.chip.mcu() {
            let _ = mcu.send_msg(&TrsyncTrigger {
                oid: self.oid,
                reason: reason as u8,
            });
        }
    }

    /// Handle one `trsync_state` (`MCU_trsync._handle_trsync_state`).
    fn handle_state(&self, state: TrsyncState) {
        let group = self.group.upgrade();
        // A group that has closed takes no report: what arrives now is the tail
        // of the round that closed it — the answer to the `trsync_trigger`s its
        // close broadcast. Upstream returns immediately here
        // (`chelper/trdispatch.c:78-79`), because taking such a report would
        // extend timeouts, fire `PAST_END_TIME`, or complete a dispatch with
        // the previous round's reason.
        if group
            .as_ref()
            .is_some_and(|group| !group.can_trigger.load(Ordering::SeqCst))
        {
            return;
        }
        if !state.can_trigger {
            // Record every real reason, including codes the typed enum does
            // not name (5+, `trigger_analog`'s errors): a completion that
            // stays silent would hang `home_wait`.
            if let Some(group) = &group {
                group.close(state.trigger_reason);
            }
            return;
        }
        let clock = self.chip.clock32_to_clock64(state.clock).unwrap_or(0);
        *self
            .last_status_clock
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = Some(clock);
        // The move's planned end passed without an endstop: ask for a trigger.
        let home_end = *self
            .home_end_clock
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        if let Some(end) = home_end {
            if clock >= end {
                *self
                    .home_end_clock
                    .lock()
                    .unwrap_or_else(|p| p.into_inner()) = None;
                self.send_trigger(TriggerReason::PastEndTime);
            }
        }
        if let Some(group) = &group {
            group.on_report();
        }
    }

    /// Install the MCU's shared `trsync_state` handler on the session that is
    /// about to report.
    ///
    /// Per arm, not per configuration: `ConfigBuilder::build` runs the config
    /// callbacks **once** per configuration (`mcu/config.rs`), while the event
    /// table a handler lives in belongs to one connection
    /// ([`McuEvents`](crate::core::klippy::mcu::events)) — and a section that is
    /// reset in place keeps its chips but gets a new session (`McuObject`'s
    /// `reconnect`). A binding made at configuration time therefore covers only
    /// the first session; every report on a replacement session was dropped as
    /// unhandled (`mcu/mod.rs`, "Unhandled message") and the dispatch waited
    /// forever. Upstream binds its handler in `MCU_trsync.start` for the same
    /// reason (`klippy/mcu.py:255-259`).
    fn bind_report_handler(&self, mcu: &Arc<Mcu>) -> Result<(), McuError> {
        self.registry.bind(mcu)
    }

    fn require_mcu(&self) -> Result<Arc<Mcu>, McuError> {
        self.chip
            .mcu()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))
    }

    fn require_clock(&self) -> Result<Arc<crate::core::klippy::cmd::clock::McuClock>, McuError> {
        self.chip
            .clock()
            .ok_or_else(|| McuError::Config("MCU has no clock estimate".to_string()))
    }
}

/// Routes `trsync_state` to the `McuTrsync` that owns the oid.
///
/// One per MCU: `Mcu::bind_event` keeps a single handler per message name, so
/// every trsync on the MCU must share it.
#[derive(Default)]
pub struct TrsyncRegistry {
    /// The trsyncs of this MCU, by the oid `config_trsync` assigned.
    ///
    /// Weak: each trsync holds this registry (to bind the report handler), so
    /// a strong entry would be a cycle — and through the trsync's chip slot it
    /// would keep the connected `Mcu` alive past teardown.
    by_oid: Mutex<HashMap<u8, Weak<McuTrsync>>>,
}

impl TrsyncRegistry {
    /// An empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Remember `trsync`, so a report naming its oid can be routed to it.
    pub fn register(&self, trsync: &Arc<McuTrsync>) {
        self.by_oid
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(trsync.oid(), Arc::downgrade(trsync));
    }

    /// Bind the shared `trsync_state` handler on `mcu`, replacing a handler an
    /// earlier session left behind (`Mcu::bind_event` keeps one per message).
    ///
    /// Two callers, two reasons, one binding:
    /// [`McuChip::attach`](super::pin::McuChip::attach) binds on every session
    /// the chip gets — right after `identify`, before the configuration
    /// handshake — so a report that arrives before anything is armed (a killed
    /// previous process's leftover periodic feed) is already routed and dropped
    /// in silence; it is never unbound for that session's lifetime.
    /// [`McuTrsync::start`] binds again before arming, so a homing move on a
    /// firmware that cannot report fails there instead of waiting forever.
    ///
    /// # Errors
    /// Returns [`McuError`] if the message is not in the session's dictionary.
    pub(crate) fn bind(self: &Arc<Self>, mcu: &Mcu) -> Result<(), McuError> {
        // Weak, like the connection's own handlers (`McuObject`'s
        // `watch_connect_shutdown`): the table this closure sits in belongs to
        // the session, and a handler that kept its resource strongly would be
        // another `Mcu → events → resource` edge for `clear_events` to break.
        let registry = Arc::downgrade(self);
        mcu.bind_event::<TrsyncState, _>(move |state| {
            if let Some(registry) = registry.upgrade() {
                registry.route(state);
            }
        })
    }

    /// Route `state` to its trsync; an oid nobody owns is dropped silently, as
    /// upstream's per-oid handlers drop a late report
    /// (`_handle_trsync_state`, `klippy/mcu.py:228-240`).
    fn route(&self, state: TrsyncState) {
        let trsync = self
            .by_oid
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&state.oid)
            .and_then(Weak::upgrade);
        if let Some(trsync) = trsync {
            trsync.handle_state(state);
        }
    }
}

/// One dispatch: a set of `McuTrsync`s and their shared completion.
pub struct TriggerDispatch {
    group: Arc<TriggerGroup>,
}

impl TriggerDispatch {
    /// Create a dispatch over `chips` (one `McuTrsync` each).
    ///
    /// # Errors
    /// Returns [`McuError`] if a trsync cannot be built.
    pub fn new(chips: Vec<McuChip>) -> Result<Self, McuError> {
        let group = Arc::new(TriggerGroup {
            trsyncs: Mutex::new(Vec::new()),
            completion: Completion::new(),
            can_trigger: AtomicBool::new(false),
        });
        for chip in chips {
            let trsync = McuTrsync::new(chip, Arc::downgrade(&group))?;
            group
                .trsyncs
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(trsync);
        }
        Ok(Self { group })
    }

    /// The oid the endstop's `endstop_home` should name.
    pub fn get_oid(&self) -> u8 {
        self.group.trsyncs.lock().unwrap_or_else(|p| p.into_inner())[0].oid()
    }

    /// The shared completion.
    pub fn completion(&self) -> Arc<Completion> {
        Arc::clone(&self.group.completion)
    }

    /// Register a stepper on `mcu_name` to stop when the group fires.
    /// Register a rail's stepper with the dispatch.
    ///
    /// The trsync for the stepper's MCU is created when it is missing, so a rail
    /// whose stepper is on a different MCU than its endstop still stops on a
    /// trigger. The handle is weak: the oid is read when the group is armed,
    /// because this runs at config-load time.
    ///
    /// # Errors
    /// Returns a config error for a multi-MCU shared axis — two steppers of one
    /// axis on different MCUs — which upstream rejects too
    /// (`TriggerDispatch.add_stepper`, `klippy/mcu.py:293-308`).
    pub fn add_stepper(
        &self,
        chip: McuChip,
        stepper: Weak<McuStepper>,
        name: &str,
    ) -> Result<(), ConfigError> {
        let trsyncs: Vec<Arc<McuTrsync>> = self
            .group
            .trsyncs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        let trsync = match trsyncs
            .iter()
            .find(|trsync| trsync.chip.name() == chip.name())
            .cloned()
        {
            Some(trsync) => trsync,
            None => {
                let trsync = McuTrsync::new(chip, Arc::downgrade(&self.group))
                    .map_err(|err| ConfigError::new(err.to_string()))?;
                self.group
                    .trsyncs
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .push(Arc::clone(&trsync));
                trsync
            }
        };
        // A shared axis (several steppers whose names share a prefix) must not
        // span MCUs: the firmware stop is per trsync, so the axis would stop on
        // whichever MCU triggered first.
        if name.starts_with("stepper_") {
            let prefix = &name[..name.len().min(9)];
            for other in &trsyncs {
                if Arc::ptr_eq(other, &trsync) {
                    continue;
                }
                if other
                    .stepper_names()
                    .iter()
                    .any(|other_name| other_name.starts_with(prefix))
                {
                    return Err(ConfigError::new(
                        "Multi-mcu homing not supported on multi-mcu shared axis",
                    ));
                }
            }
        }
        trsync.add_stepper(stepper, name);
        Ok(())
    }

    /// The names of every stepper registered with the dispatch, across all its
    /// trsyncs, in registration order (upstream's `TriggerDispatch.get_steppers`,
    /// `klippy/mcu.py:308-309`, by name).
    #[cfg(test)]
    pub(crate) fn stepper_names(&self) -> Vec<String> {
        self.group
            .trsyncs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
            .flat_map(|trsync| trsync.stepper_names())
            .collect()
    }

    /// Arm every trsync; the completion fires when one of them triggers.
    ///
    /// # Errors
    /// As [`McuTrsync`]'s sends.
    pub fn start(&self, print_time: f64) -> Result<Arc<Completion>, McuError> {
        let trsyncs = self
            .group
            .trsyncs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        self.group.completion.reset();
        self.group.completion.arm_deadline();
        self.group.can_trigger.store(true, Ordering::SeqCst);
        let timeout = if trsyncs.len() == 1 {
            TRSYNC_SINGLE_MCU_TIMEOUT
        } else {
            TRSYNC_TIMEOUT
        };
        let count = trsyncs.len() as f64;
        for (index, trsync) in trsyncs.iter().enumerate() {
            trsync.start(print_time, index as f64 / count, timeout)?;
        }
        Ok(Arc::clone(&self.group.completion))
    }

    /// Note the print time the move is planned to end at.
    pub fn wait_end(&self, end_time: f64) {
        for trsync in self
            .group
            .trsyncs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
        {
            trsync.set_home_end(end_time);
        }
    }

    /// Fire every trsync with `HOST_REQUEST` and return the recorded reason.
    ///
    /// The group is closed here too, so the reports these triggers provoke are
    /// dropped once they arrive (`handle_state`) instead of completing or
    /// extending a dispatch that already ended.
    pub fn stop(&self) -> TriggerReason {
        self.group.can_trigger.store(false, Ordering::SeqCst);
        for trsync in self
            .group
            .trsyncs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
        {
            trsync.send_trigger(TriggerReason::HostRequest);
        }
        self.group
            .completion
            .reason()
            .unwrap_or(TriggerReason::HostRequest)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::core::klippy::interface::devices::frame_mock::{FrameMock, RecordingWire};
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::{Dictionary, Mcu};
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::{ArgValue, Payload};
    use crate::core::klippy::pins::{PinParams, PrinterPins};
    use crate::core::klippy::reactor::ManualReactor;
    use crate::core::klippy::test_util::wait_until;
    use serde_json::json;
    use std::time::Duration;
    use tokio::time::{sleep, timeout};

    /// A dictionary with the trsync/stepper commands.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "allocate_oids count=%c": 2,
                "get_config": 7,
                "finalize_config crc=%u": 6,
                "config_trsync oid=%c": 30,
                "trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c": 31,
                "trsync_set_timeout oid=%c clock=%u": 32,
                "trsync_trigger oid=%c reason=%c": 33,
                "stepper_stop_on_trigger oid=%c trsync_oid=%c": 34
            },
            "responses": {
                "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u": 35,
                "config is_config=%c crc=%u is_shutdown=%c move_count=%hu": 9
            },
            "config": {"CLOCK_FREQ": 1_000_000}
        }))
        .unwrap()
    }

    fn identified_mcu(name: &str) -> Arc<Mcu> {
        let mcu = Mcu::for_test(name, Interface::new(FrameMock::new(Vec::new())));
        mcu.install_dictionary(dictionary()).unwrap();
        Arc::new(mcu)
    }

    /// An identified MCU whose sends a test can read back as raw blocks.
    ///
    /// A `RecordingWire`, not a `FrameMock`: the frame mock accepts only what a
    /// mapping entry scripts, so an unsolicited `trsync_trigger` would never be
    /// recorded.
    fn recording_mcu(name: &str) -> (Arc<Mcu>, Arc<Mutex<Vec<Vec<u8>>>>) {
        let wire = RecordingWire::new();
        let sent = wire.sent();
        let mcu = Arc::new(Mcu::for_test(name, Interface::recording(wire)));
        mcu.install_dictionary(dictionary()).unwrap();
        (mcu, sent)
    }

    /// Closes the fake session when a test ends — on a failed assertion too.
    ///
    /// A `RecordingWire`'s read parks in `spawn_blocking`; only `Mcu::close`
    /// releases it, and a session left open holds up the runtime's shutdown for
    /// the whole test binary (the shape `endstop.rs`'s `SessionGuard` documents).
    struct SessionGuard(Arc<Mcu>);

    impl Drop for SessionGuard {
        fn drop(&mut self) {
            self.0.clear_events();
            self.0.close();
        }
    }

    /// The message names the recorded wire blocks carry, decoded with this test
    /// dictionary's command table.
    fn sent_message_names(blocks: &[Vec<u8>]) -> Vec<String> {
        use crate::core::klippy::msg::parser::Parser;
        use crate::core::klippy::msg::proto::Payload;

        let mut parser = Parser::new();
        for (id, format) in [
            (30, "config_trsync oid=%c"),
            (
                31,
                "trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c",
            ),
            (32, "trsync_set_timeout oid=%c clock=%u"),
            (33, "trsync_trigger oid=%c reason=%c"),
            (34, "stepper_stop_on_trigger oid=%c trsync_oid=%c"),
        ] {
            parser.register(id, format).unwrap();
        }
        blocks
            .iter()
            .flat_map(|block| {
                parser
                    .decode(Payload::from_raw(block.clone()))
                    .unwrap_or_default()
            })
            .map(|(message, _)| message.name.clone())
            .collect()
    }

    fn chip(name: &str, mcu: Arc<Mcu>) -> McuChip {
        let pins = Arc::new(PrinterPins::new());
        let chip = McuChip::new(
            name.to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::clone(&pins),
        );
        // The chip holds the registry weakly (the printer owns it in
        // production); keep it alive for the test process.
        std::mem::forget(pins);
        // Production sets this in `McuObject::connect`; the test builds the
        // chip directly, so seed the clock from the fixture dictionary (1 MHz).
        let clock = Arc::new(crate::core::klippy::cmd::clock::McuClock::new(
            Arc::clone(&mcu),
            ManualReactor::shared(),
        ));
        clock.seed(0.0, 0);
        chip.set_clock(clock, 0.0);
        chip.attach(mcu);
        chip
    }

    /// The first trsync of a dispatch.
    fn first(dispatch: &TriggerDispatch) -> Arc<McuTrsync> {
        dispatch
            .group
            .trsyncs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .first()
            .cloned()
            .unwrap()
    }

    #[tokio::test]
    async fn test_a_state_report_completes_the_group() {
        let dispatch = TriggerDispatch::new(vec![chip("mcu", identified_mcu("mcu"))]).unwrap();
        let completion = dispatch.start(0.0).unwrap();
        let trsync = first(&dispatch);

        // The firmware reports it can no longer trigger, with an endstop hit.
        trsync.handle_state(TrsyncState {
            oid: trsync.oid(),
            can_trigger: false,
            trigger_reason: TriggerReason::EndstopHit as u8,
            clock: 0,
        });

        assert_eq!(completion.wait().await, TriggerReason::EndstopHit);
    }

    #[tokio::test]
    async fn test_a_completion_records_reasons_the_typed_enum_does_not_name() {
        // A `trigger_analog` monitor failure arrives as raw reason 7 — higher
        // than `TriggerReason` covers. It must still complete the dispatch so
        // `home_wait` can wake and decode it, while the typed view collapses
        // it onto the failure reason.
        let dispatch = TriggerDispatch::new(vec![chip("mcu", identified_mcu("mcu"))]).unwrap();
        let completion = dispatch.start(0.0).unwrap();
        let trsync = first(&dispatch);

        trsync.handle_state(TrsyncState {
            oid: trsync.oid(),
            can_trigger: false,
            trigger_reason: 7,
            clock: 0,
        });

        assert_eq!(completion.wait_raw().await, 7);
        assert_eq!(completion.raw(), Some(7));
        assert_eq!(completion.reason(), None);
        assert_eq!(completion.wait().await, TriggerReason::CommsTimeout);
    }

    #[tokio::test]
    async fn test_a_zero_reason_does_not_complete() {
        // The firmware never reports reason 0; a stray zero must not wake a
        // waiter with a meaningless value.
        let dispatch = TriggerDispatch::new(vec![chip("mcu", identified_mcu("mcu"))]).unwrap();
        let completion = dispatch.start(0.0).unwrap();
        let trsync = first(&dispatch);

        trsync.handle_state(TrsyncState {
            oid: trsync.oid(),
            can_trigger: false,
            trigger_reason: 0,
            clock: 0,
        });

        assert_eq!(completion.raw(), None);
        assert_eq!(completion.reason(), None);
    }

    /// A report that arrives after the group closed is dropped: upstream's
    /// `handle_trsync_state` returns at once when the dispatch can no longer
    /// trigger (`trdispatch.c:78-79`).
    ///
    /// The window this covers is the one between the close and the next arm —
    /// `home_wait`'s tail, before the next homing round's `start`. A report
    /// taken there would extend the group's timeouts and fire `PAST_END_TIME`
    /// for a move that already ended, and the reason it carries is the previous
    /// round's, not this one's.
    #[tokio::test]
    async fn test_a_report_after_the_dispatch_closed_completes_nothing() {
        let mcu = identified_mcu("mcu");
        let dispatch = TriggerDispatch::new(vec![chip("mcu", Arc::clone(&mcu))]).unwrap();
        let completion = dispatch.start(0.0).unwrap();
        let trsync = first(&dispatch);

        // The host closes the round itself, before any report resolved it —
        // `McuEndstop::home_wait`'s `stop()`. Its completion stays unresolved.
        dispatch.stop();
        assert_eq!(completion.raw(), None);

        // The firmware's answer to that trigger reaches the host late.
        trsync.handle_state(TrsyncState {
            oid: trsync.oid(),
            can_trigger: false,
            trigger_reason: TriggerReason::EndstopHit as u8,
            clock: 0,
        });
        assert_eq!(completion.raw(), None, "a closed group takes no reason");

        // A report that still says it can trigger must not move the group's
        // clocks either: that is where `on_report` extends timeouts from.
        trsync.handle_state(TrsyncState {
            oid: trsync.oid(),
            can_trigger: true,
            trigger_reason: 0,
            clock: 1_000_000,
        });
        assert!(
            trsync.acknowledged_print_time().is_none(),
            "a closed group takes no status clock"
        );
    }

    /// A dispatch whose trigger report never arrives must not park the wait:
    /// [`HOME_WAIT_TIMEOUT`] of silence fails it as a communication timeout —
    /// what `home_wait` raises as upstream's "Communication timeout during
    /// homing" — and the close-out puts `trsync_trigger` on the wire, so the
    /// firmware stops the group instead of driving on.
    ///
    /// This is the shipped hang: every `trsync_state` report went unhandled, the
    /// completion never fired, and `G28` — with the g-code script parked on it —
    /// never returned.
    ///
    /// Real time, not the paused clock: an `Mcu` in the runtime (its receive task
    /// parks in `spawn_blocking`) stops `start_paused` from advancing the clock,
    /// so a virtual wait would hang instead of timing out.
    #[tokio::test]
    async fn test_a_dispatch_with_no_report_fails_the_wait_after_the_silence_timeout() {
        let (mcu, sent) = recording_mcu("mcu");
        // Declared before the resource, so it drops last.
        let _session = SessionGuard(Arc::clone(&mcu));
        let dispatch = TriggerDispatch::new(vec![chip("mcu", Arc::clone(&mcu))]).unwrap();
        let completion = dispatch.start(0.0).unwrap();

        let started = tokio::time::Instant::now();
        assert_eq!(
            completion.wait().await,
            TriggerReason::CommsTimeout,
            "a lost trigger report must fail the dispatch, not hang it"
        );
        assert!(
            started.elapsed() >= HOME_WAIT_TIMEOUT,
            "the wait is bounded by HOME_WAIT_TIMEOUT: {:?}",
            started.elapsed()
        );
        assert_eq!(completion.raw(), Some(TriggerReason::CommsTimeout as u8));

        // The dispatcher is closed down the way `McuEndstop::home_wait` does it
        // on the failure path, and the firmware is told to fire the group.
        assert_eq!(dispatch.stop(), TriggerReason::CommsTimeout);
        mcu.flush(Duration::from_millis(500)).await.unwrap();
        let blocks = sent.lock().unwrap_or_else(|p| p.into_inner()).clone();
        let names = sent_message_names(&blocks);
        assert!(
            names.iter().any(|name| name == "trsync_trigger"),
            "the firmware must be told to stop the group: {names:?}"
        );
    }

    /// The clock the wait is bounded by measures *silence*, not the length of
    /// the move: a `trsync_state` report pushes it out, so a long homing move
    /// whose firmware keeps reporting is never failed by it.
    #[tokio::test]
    async fn test_a_report_pushes_the_silence_deadline_out() {
        let dispatch = TriggerDispatch::new(vec![chip("mcu", identified_mcu("mcu"))]).unwrap();
        let completion = dispatch.start(0.0).unwrap();
        let trsync = first(&dispatch);

        // Four seconds in — one second inside the arm's silence deadline —
        // the firmware reports it is still armed. The wait is polled on that
        // deadline rather than timed by a fixed 4 s sleep: it is real time, and
        // a loaded machine can stretch the sleep past the 5 s it has to stay
        // inside, which would fail the arm before the report is sent.
        //
        // Real time, not the paused clock: an `Mcu` in the runtime (its receive
        // task parks in `spawn_blocking`) stops `start_paused` from advancing
        // the clock at all, so a virtual sleep here would hang the test.
        wait_until(
            HOME_WAIT_TIMEOUT,
            Duration::from_millis(20),
            || {
                completion
                    .remaining()
                    .is_some_and(|left| left <= Duration::from_secs(1))
            },
            "the report is sent inside the arm's silence deadline",
        )
        .await;
        trsync.handle_state(TrsyncState {
            oid: trsync.oid(),
            can_trigger: true,
            trigger_reason: 0,
            clock: 0,
        });

        // Six seconds after the arm — past the deadline the arm set — the wait
        // still stands, because that report restarted the clock.
        let pending = completion.wait_raw();
        tokio::pin!(pending);
        assert!(
            tokio::time::timeout(Duration::from_secs(2), &mut pending)
                .await
                .is_err(),
            "a live report must restart the silence clock"
        );
        // And it gives up five seconds after that report.
        assert_eq!(
            tokio::time::timeout(Duration::from_secs(4), pending)
                .await
                .expect("the silence clock runs out"),
            TriggerReason::CommsTimeout as u8
        );
    }

    #[tokio::test]
    async fn test_a_secondary_extends_the_slowest_mcus_timeout() {
        // Two MCUs; the secondary's acknowledged time is far behind, so the
        // primary's deadline is extended to the secondary's next report.
        let dispatch = TriggerDispatch::new(vec![
            chip("mcu", identified_mcu("mcu")),
            chip("zboard", identified_mcu("zboard")),
        ])
        .unwrap();
        dispatch.start(0.0).unwrap();
        let trsyncs = dispatch
            .group
            .trsyncs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        let (fast, slow) = (&trsyncs[0], &trsyncs[1]);
        // The fast MCU acknowledges print time 1.0; the slow one only 0.1.
        fast.handle_state(TrsyncState {
            oid: fast.oid,
            can_trigger: true,
            trigger_reason: 0,
            clock: 1_000_000,
        });
        let before = *fast.expire_clock.lock().unwrap_or_else(|p| p.into_inner());
        slow.handle_state(TrsyncState {
            oid: slow.oid,
            can_trigger: true,
            trigger_reason: 0,
            clock: 100_000,
        });

        // The fast MCU's deadline moved forward: it now waits for the slow one
        // (clock 0.1 s plus that MCU's expire_ticks), not just its own report.
        assert!(
            *fast.expire_clock.lock().unwrap_or_else(|p| p.into_inner()) >= before,
            "the group did not take the slowest MCU into account"
        );
    }

    /// Closing a group is a broadcast: upstream `handle_trsync_state` walks
    /// `td->tdm_list` and sends `trsync_trigger` to every MCU in it, so the one
    /// that reported is not the only one told to stop (`trdispatch.c:81-89`).
    ///
    /// The other MCUs may still be driving the move — that is why the report is
    /// propagated instead of only completing this host's dispatch.
    #[tokio::test]
    async fn test_a_trigger_report_stops_every_mcu_in_the_group() {
        let (primary, primary_sent) = recording_mcu("mcu");
        let (secondary, secondary_sent) = recording_mcu("zboard");
        let _primary_session = SessionGuard(Arc::clone(&primary));
        let _secondary_session = SessionGuard(Arc::clone(&secondary));
        let dispatch = TriggerDispatch::new(vec![
            chip("mcu", Arc::clone(&primary)),
            chip("zboard", Arc::clone(&secondary)),
        ])
        .unwrap();
        dispatch.start(0.0).unwrap();
        let trsyncs = dispatch
            .group
            .trsyncs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        let (reporting, other) = (&trsyncs[0], &trsyncs[1]);
        // Let the arm go out as its own blocks: the client batches messages
        // sent together, and the trigger below is compared as one payload.
        primary.flush(Duration::from_millis(500)).await.unwrap();
        secondary.flush(Duration::from_millis(500)).await.unwrap();

        // The MCU that hit its endstop reports it can no longer trigger.
        reporting.handle_state(TrsyncState {
            oid: reporting.oid,
            can_trigger: false,
            trigger_reason: TriggerReason::EndstopHit as u8,
            clock: 0,
        });

        primary.flush(Duration::from_millis(500)).await.unwrap();
        secondary.flush(Duration::from_millis(500)).await.unwrap();
        let trigger = |oid: u8| {
            payload(
                "trsync_trigger",
                &[
                    ArgValue::UInt8(oid),
                    ArgValue::UInt8(TriggerReason::HostRequest as u8),
                ],
            )
        };
        assert!(
            secondary_sent
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .contains(&trigger(other.oid)),
            "the other MCU was not told to trigger: {:?}",
            sent_names(&secondary_sent)
        );
        // Upstream's loop covers the whole list, the reporting MCU included
        // (its `trsync_trigger` is idempotent, `src/trsync.c:171-186`).
        assert!(
            primary_sent
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .contains(&trigger(reporting.oid)),
            "the reporting MCU was not told to trigger: {:?}",
            sent_names(&primary_sent)
        );
        assert_eq!(
            dispatch.completion().reason(),
            Some(TriggerReason::EndstopHit)
        );
    }

    #[tokio::test]
    async fn test_a_registry_routes_by_oid() {
        // A report with an unknown oid is dropped; the registered one reaches
        // its trsync.
        let mcu = identified_mcu("mcu");
        let chip = chip("mcu", Arc::clone(&mcu));
        let dispatch = TriggerDispatch::new(vec![chip.clone()]).unwrap();
        let trsync = first(&dispatch);
        let registry = chip.trsync_registry();
        registry.register(&trsync);
        // Armed, so the group can trigger: a report for a dispatch that never
        // armed is dropped (`handle_state`), which is the case pinned by
        // `test_a_killed_process_leftover_feed_is_silent_and_cleared_on_connect`.
        dispatch.start(0.0).unwrap();

        // An oid nobody owns stops at the registry: the handler is bound to the
        // MCU, so a late report never falls through to the "unhandled message"
        // path (`mcu/mod.rs`) — upstream ignores it the same way
        // (`_handle_trsync_state`, `klippy/mcu.py:228-240`).
        registry.route(TrsyncState {
            oid: trsync.oid.wrapping_add(1),
            can_trigger: false,
            trigger_reason: TriggerReason::EndstopHit as u8,
            clock: 0,
        });
        assert_eq!(dispatch.completion().reason(), None);

        registry.route(TrsyncState {
            oid: trsync.oid,
            can_trigger: false,
            trigger_reason: TriggerReason::HostRequest as u8,
            clock: 0,
        });

        assert_eq!(
            dispatch.completion().reason(),
            Some(TriggerReason::HostRequest)
        );
    }

    #[tokio::test]
    async fn test_two_endstops_on_one_mcu_share_the_registry() {
        // Two rails whose endstops are on the *same* MCU: each gets its own
        // dispatch/trsync (distinct oids) but they share the chip's one bound
        // handler, so the registry must route each report to the right one.
        let mcu = identified_mcu("mcu");
        let chip = chip("mcu", Arc::clone(&mcu));
        let x = TriggerDispatch::new(vec![chip.clone()]).unwrap();
        let y = TriggerDispatch::new(vec![chip.clone()]).unwrap();
        let tx = first(&x);
        let ty = first(&y);
        assert_ne!(tx.oid, ty.oid, "two endstops must not share an oid");
        // Both trsyncs belong to the same MCU, so they share its one registry.
        assert_eq!(tx.chip.name(), ty.chip.name());

        let registry = chip.trsync_registry();
        registry.register(&tx);
        registry.register(&ty);
        // Both groups armed: each is open to reports, so a report reaching the
        // wrong group would complete it.
        x.start(0.0).unwrap();
        y.start(0.0).unwrap();
        registry.route(TrsyncState {
            oid: ty.oid,
            can_trigger: false,
            trigger_reason: TriggerReason::EndstopHit as u8,
            clock: 0,
        });

        // Only Y's group completed.
        assert_eq!(y.completion().reason(), Some(TriggerReason::EndstopHit));
        assert_eq!(x.completion().reason(), None);
    }

    /// An `McuStepper` on `chip`, for the registration tests.
    fn stepper(chip: &McuChip, pin: &str) -> Arc<McuStepper> {
        let params = |pin: &str| PinParams {
            chip_name: chip.name().to_string(),
            pin: pin.to_string(),
            invert: false,
            pullup: 0,
            share_type: None,
        };
        chip.setup_stepper(params(pin), params(&format!("{pin}b")), 0, 0.000_002, false)
    }

    #[tokio::test]
    async fn test_a_trsync_can_stop_several_steppers() {
        // A rail with more than one stepper (a shared axis) registers each of
        // them with the rail's trsync.
        let chip = chip("mcu", identified_mcu("mcu"));
        let dispatch = TriggerDispatch::new(vec![chip.clone()]).unwrap();
        let a = stepper(&chip, "PA1");
        let b = stepper(&chip, "PA2");

        dispatch
            .add_stepper(chip.clone(), Arc::downgrade(&a), "stepper_x")
            .unwrap();
        dispatch
            .add_stepper(chip.clone(), Arc::downgrade(&b), "stepper_x1")
            .unwrap();

        let trsync = first(&dispatch);
        assert_eq!(trsync.stepper_names(), vec!["stepper_x", "stepper_x1"]);
    }

    /// Registering the same stepper twice keeps one entry: a rail registers all
    /// of its motors with its endstop, and a virtual endstop's helper (the
    /// probe's `LookupZSteppers`) registers the same motors again — upstream's
    /// `MCU_trsync.add_stepper` ignores the repeat (`klippy/mcu.py:183-186`),
    /// and the firmware must get one stop message per motor.
    #[tokio::test]
    async fn test_a_stepper_is_registered_once_even_if_added_twice() {
        let chip = chip("mcu", identified_mcu("mcu"));
        let dispatch = TriggerDispatch::new(vec![chip.clone()]).unwrap();
        let z1 = stepper(&chip, "PA2");

        dispatch
            .add_stepper(chip.clone(), Arc::downgrade(&z1), "stepper_z1")
            .unwrap();
        dispatch
            .add_stepper(chip.clone(), Arc::downgrade(&z1), "stepper_z1")
            .unwrap();

        assert_eq!(first(&dispatch).stepper_names(), vec!["stepper_z1"]);
    }

    /// A trigger analog's dispatch is handed the Z rail's steppers
    /// (`stepper_z`, `stepper_z1`…): it must report them by name, and report
    /// nothing before anyone registers.
    ///
    /// This is the dispatch-layer half of the load-cell Z wiring
    /// (`LoadCellProbingMove`'s `LookupZSteppers`, `load_cell_probe.py:320`);
    /// the end-to-end half lives with the probe.
    #[tokio::test]
    async fn test_a_dispatch_reports_the_z_steppers_registered_with_it() {
        let chip = chip("mcu", identified_mcu("mcu"));
        let dispatch = TriggerDispatch::new(vec![chip.clone()]).unwrap();
        assert!(
            first(&dispatch).stepper_names().is_empty(),
            "a dispatch nobody registered with must report no steppers"
        );

        let z = stepper(&chip, "PA1");
        let z1 = stepper(&chip, "PA2");
        dispatch
            .add_stepper(chip.clone(), Arc::downgrade(&z), "stepper_z")
            .unwrap();
        dispatch
            .add_stepper(chip.clone(), Arc::downgrade(&z1), "stepper_z1")
            .unwrap();

        assert_eq!(
            first(&dispatch).stepper_names(),
            vec!["stepper_z", "stepper_z1"]
        );
    }

    #[tokio::test]
    async fn test_a_shared_axis_spanning_mcus_is_refused() {
        // A rail's stepper on a different MCU than its endstop is allowed: the
        // dispatch creates a trsync for that MCU. But two steppers of the same
        // axis on different MCUs is the multi-MCU shared axis upstream rejects.
        let primary = chip("mcu", identified_mcu("mcu"));
        let secondary = chip("zboard", identified_mcu("zboard"));
        let dispatch = TriggerDispatch::new(vec![primary.clone()]).unwrap();
        let x = stepper(&primary, "PA1");
        let x1 = stepper(&secondary, "PA3");
        let y = stepper(&secondary, "PA5");

        dispatch
            .add_stepper(primary.clone(), Arc::downgrade(&x), "stepper_x")
            .unwrap();
        let err = dispatch
            .add_stepper(secondary.clone(), Arc::downgrade(&x1), "stepper_x1")
            .unwrap_err();
        assert!(err.to_string().contains("Multi-mcu homing"), "{err}");

        // A different axis on the secondary is fine, and gets its own trsync.
        dispatch
            .add_stepper(secondary.clone(), Arc::downgrade(&y), "stepper_y")
            .unwrap();
        assert_eq!(
            dispatch
                .group
                .trsyncs
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .len(),
            2
        );
    }

    /// One message's wire payload, as the host encodes it from the fixture
    /// dictionary (`wire_bytes` in `endstop.rs` does the same).
    fn payload(name: &str, args: &[ArgValue]) -> Vec<u8> {
        let mut parser = Parser::new();
        dictionary().install(&mut parser).unwrap();
        parser.encode(name, args).unwrap().into_raw()
    }

    /// The message names in the payloads `wire` was sent, in order (one block
    /// may carry several).
    fn sent_names(sent: &Arc<Mutex<Vec<Vec<u8>>>>) -> Vec<String> {
        let mut parser = Parser::new();
        dictionary().install(&mut parser).unwrap();
        sent.lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
            .flat_map(|payload| {
                parser
                    .decode(Payload::from_raw(payload.clone()))
                    .unwrap_or_default()
            })
            .map(|(msg, _)| msg.name.clone())
            .collect()
    }

    /// Closes a fake session when a test ends — on a failed assertion too.
    ///
    /// A trsync reaches its `Mcu` through the chip's device slot while the
    /// chip's builder holds the config callback, so the dispatch a test holds
    /// keeps the session alive: `Mcu::Drop` never runs and its parked device
    /// read would hold up runtime shutdown (`endstop.rs`'s `SessionGuard` has
    /// the full note).
    /// The `trsync_state` handler must be bound on the session that **arms**,
    /// not only on the one the configuration was built on.
    ///
    /// A section survives a firmware reset by replacing its session
    /// (`McuObject::reconnect`): the chips stay, the event table does not, and
    /// the config callbacks never run again (`ConfigBuilder::build` runs them
    /// once per configuration). Binding in the config callback therefore left a
    /// replaced session with **no** handler: every `trsync_state` it received was
    /// dropped as unhandled (`mcu/mod.rs`, "Unhandled message") and
    /// `home_wait` waited for a completion nothing could deliver.
    #[tokio::test]
    async fn test_a_replaced_session_binds_the_report_handler_when_it_arms() {
        // Session 1: the configuration is built here, which is where the
        // handler used to be installed.
        let first_session = identified_mcu("mcu");
        let chip = chip("mcu", Arc::clone(&first_session));
        let dispatch = TriggerDispatch::new(vec![chip.clone()]).unwrap();
        let trsync = first(&dispatch);
        let _built = chip.config().build(&first_session).unwrap();

        // Session 2 replaces it. Its wire answers `trsync_trigger` with the
        // `trsync_state` the firmware would send back — a plain report, because
        // the trigger went out with `send_msg`, so no pending call owns it.
        let wire = RecordingWire::new();
        let sent = wire.sent();
        wire.reply_to(
            payload(
                "trsync_trigger",
                &[
                    ArgValue::UInt8(trsync.oid()),
                    ArgValue::UInt8(TriggerReason::HostRequest as u8),
                ],
            ),
            payload(
                "trsync_state",
                &[
                    ArgValue::UInt8(trsync.oid()),
                    ArgValue::UInt8(0),
                    ArgValue::UInt8(TriggerReason::HostRequest as u8),
                    ArgValue::UInt32(0),
                ],
            ),
        );
        let second_session = Arc::new(Mcu::for_test("mcu", Interface::recording(wire)));
        second_session.install_dictionary(dictionary()).unwrap();
        second_session.set_clock_base(0);
        let _session = SessionGuard(Arc::clone(&second_session));
        chip.attach(Arc::clone(&second_session));

        // From the session's first moment, before anything is armed: a report
        // that arrives now belongs to no dispatch (`oid` unknown until an arm,
        // or the leftover feed a killed host left running) and must be dropped
        // in silence, not as an unhandled message.
        assert!(
            second_session.has_event_handler::<TrsyncState>(),
            "a replaced session must have the trsync_state router from attach on"
        );

        let completion = dispatch.start(0.0).unwrap();
        // Let the arm flush as its own block, so the trigger below is answered
        // on its own (the wire answers a whole block at a time).
        sleep(Duration::from_millis(120)).await;
        // The trigger goes out while the group is still armed, and the wire
        // answers it with the report the firmware sends back. The trigger comes
        // from the test rather than `TriggerDispatch::stop`, because a closed
        // group drops reports (`handle_state`): what this test pins is a report
        // an **open** dispatch must take, routed through the replaced session's
        // handler.
        second_session
            .send_msg(&TrsyncTrigger {
                oid: trsync.oid(),
                reason: TriggerReason::HostRequest as u8,
            })
            .unwrap();

        let reason = timeout(Duration::from_secs(2), completion.wait())
            .await
            .expect("the report must be routed, not dropped as unhandled");
        assert_eq!(reason, TriggerReason::HostRequest);
        // The close-out `home_wait` runs once the completion has resolved
        // (`McuEndstop::home_wait`): it tells every MCU in the group to trigger.
        dispatch.stop();

        // The arm and the stop both reached the firmware. `trsync_trigger` is
        // what deletes the firmware's report and expire timers
        // (`src/trsync.c:171-186`), i.e. what ends the periodic feed an armed
        // group otherwise keeps producing (`src/trsync.c:57-65`).
        let names = sent_names(&sent);
        assert!(
            names.iter().any(|name| name == "trsync_start"),
            "the group was never armed: {names:?}"
        );
        assert!(
            names.iter().any(|name| name == "trsync_trigger"),
            "the firmware was never told to stop reporting: {names:?}"
        );
    }

    /// The dictionary a real `stm32f446` build publishes declares
    /// `trsync_state`, so binding it cannot fail on the bench board.
    ///
    /// This is the “first suspect” checked off: `McuTrsync::new`'s config
    /// callback runs after the dictionary is installed (`mcu/object.rs` builds
    /// the configuration after `Mcu::connect`), so the old
    /// `bound.swap(true)`-before-`bind_event` ordering never actually poisoned a
    /// real board — the missing router was the **replaced session**
    /// (`test_a_replaced_session_binds_the_report_handler_when_it_arms`), not a
    /// failed bind.
    #[tokio::test]
    async fn test_a_real_board_dictionary_can_bind_the_trsync_state_handler() {
        let json: serde_json::Value = serde_json::from_str(include_str!(
            "../../upstream_generated/fixtures/stm32f446.dict"
        ))
        .unwrap();
        let mcu = Mcu::for_test("mcu", Interface::new(FrameMock::new(Vec::new())));
        mcu.install_dictionary(Dictionary::from_json(json).unwrap())
            .unwrap();

        assert!(mcu.has_message("trsync_state"));
        mcu.bind_event::<TrsyncState, _>(|_| {}).unwrap();
    }

    /// A previous process killed mid-homing leaves the firmware's report timer
    /// running on this oid; the session that reconnects must neither warn about
    /// its reports nor leave the feed alive.
    ///
    /// `restart_method: admin` reconnects to the same firmware instead of
    /// resetting it, and the configuration is reused on the same oid, so the
    /// leftover `trsync_state` feed keeps arriving on a group this session has
    /// not armed. A clean exit leaves no timer behind — which is why the symptom
    /// was intermittent (only after a kill).
    #[tokio::test]
    async fn test_a_killed_process_leftover_feed_is_silent_and_cleared_on_connect() {
        // The previous session: a group was armed and the process died without
        // a `trsync_trigger` (upstream's `MCU_trsync.stop`), so the firmware's
        // report timer is still re-arming itself (`src/trsync.c:57-65`).
        let wire = RecordingWire::new();
        let previous = Arc::new(Mcu::for_test("mcu", Interface::recording(wire)));
        previous.install_dictionary(dictionary()).unwrap();
        previous.set_clock_base(0);
        let previous_guard = SessionGuard(Arc::clone(&previous));
        let chip = chip("mcu", Arc::clone(&previous));
        let dispatch = TriggerDispatch::new(vec![chip.clone()]).unwrap();
        let trsync = first(&dispatch);
        // The configuration a reconnect reuses — its `init` list is what every
        // connect sends *after* `config_trsync` assigned the oid.
        let built = chip.config().build(&previous).unwrap();
        dispatch.start(0.0).unwrap();
        drop(previous_guard);

        // The replacement session, as the next `connect` brings it up.
        let session = identified_mcu("mcu");
        let _guard = SessionGuard(Arc::clone(&session));
        chip.attach(Arc::clone(&session));

        // (a) The leftover feed has a router from the session's first moment,
        // so it is dropped in silence instead of logged as an unhandled
        // message (`mcu/mod.rs`).
        assert!(
            session.has_event_handler::<TrsyncState>(),
            "the reconnected session must route trsync_state before anything is armed"
        );
        let registry = chip.trsync_registry();
        registry.route(TrsyncState {
            oid: trsync.oid(),
            can_trigger: true,
            trigger_reason: 0,
            clock: 1_000,
        });
        assert_eq!(
            dispatch.completion().reason(),
            None,
            "a report for a group this session never armed must not complete anything"
        );

        // (b) The connect tells this oid to stop reporting. The `init` list is
        // sent on every connect after the configuration (`mcu/config.rs`), and
        // the firmware's `trsync_trigger` deletes both timers even for a group
        // that cannot trigger (`src/trsync.c:31-53,171-186`).
        let mut parser = Parser::new();
        dictionary().install(&mut parser).unwrap();
        let cleared: Vec<Vec<ArgValue>> = built
            .init
            .iter()
            .flat_map(|payload| parser.decode(payload.clone()).unwrap_or_default())
            .filter(|(msg, _)| msg.name == "trsync_trigger")
            .map(|(_, args)| args)
            .collect();
        assert!(
            cleared.contains(&vec![
                ArgValue::UInt8(trsync.oid()),
                ArgValue::UInt8(TriggerReason::HostRequest as u8),
            ]),
            "the connect must clear the leftover report timer on this oid: {cleared:?}"
        );
    }

    /// The restart list disarms every group, upstream's `on_restart`
    /// `trsync_start … report_ticks=0` (`klippy/mcu.py:189-193`).
    ///
    /// Only `trsync_trigger` and a new `trsync_start` delete the firmware's
    /// periodic report timer (`src/trsync.c:171-186`), so without this a board
    /// that was reset while a homing move had a group armed keeps reporting on
    /// the session that replaces it.
    #[tokio::test]
    async fn test_the_restart_list_disarms_a_group_left_armed() {
        let mcu = identified_mcu("mcu");
        let chip = chip("mcu", Arc::clone(&mcu));
        let dispatch = TriggerDispatch::new(vec![chip.clone()]).unwrap();
        let oid = first(&dispatch).oid();
        let built = chip.config().build(&mcu).unwrap();

        let mut parser = Parser::new();
        dictionary().install(&mut parser).unwrap();
        let mut disarmed = false;
        for payload in &built.restart {
            for (msg, args) in parser.decode(payload.clone()).unwrap_or_default() {
                if msg.name != "trsync_start" {
                    continue;
                }
                assert_eq!(
                    args,
                    [
                        ArgValue::UInt8(oid),
                        ArgValue::UInt32(0),
                        ArgValue::UInt32(0),
                        ArgValue::UInt8(0),
                    ],
                    "the restart must disarm (report_ticks=0), not arm"
                );
                disarmed = true;
            }
        }
        assert!(
            disarmed,
            "the restart list must disarm the group on every connect"
        );
    }
}
