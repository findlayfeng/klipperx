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
//! * a per-MCU [`TrsyncRegistry`] binds **one** `trsync_state` handler for the
//!   whole MCU and routes by oid (our `Mcu::bind_event` keeps one handler per
//!   message name, so several endstops on one MCU must share it);
//! * on each report the group recomputes the minimum acknowledged print time and
//!   resends `trsync_set_timeout` when the extension is worth it
//!   (`handle_trsync_state`, `chelper/trdispatch.c:70-140`).
//!
//! Multi-MCU is the same code path: one `McuTrsync` per MCU, each with its own
//! `McuChip` clock, and the group takes the slowest.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};

use tokio::sync::Notify;

use super::pin::McuChip;
use crate::core::klippy::cmd::stepper::StepperStopOnTrigger;
use crate::core::klippy::cmd::trsync::{
    ConfigTrsync, TriggerReason, TrsyncSetTimeout, TrsyncStart, TrsyncState, TrsyncTrigger,
};
use crate::core::klippy::mcu::{ConfigBuilder, Mcu, McuError};

/// The deadline for a multi-MCU dispatch (`TRSYNC_TIMEOUT`, `klippy/mcu.py:259`).
pub const TRSYNC_TIMEOUT: f64 = 0.025;

/// The deadline for a single-MCU dispatch (`TRSYNC_SINGLE_MCU_TIMEOUT`,
/// `klippy/mcu.py:260`).
pub const TRSYNC_SINGLE_MCU_TIMEOUT: f64 = 0.250;

/// A one-shot completion the homing move waits on.
///
/// The `McuTrsync` that sees `can_trigger=0` completes it; `home_wait` awaits.
/// `Notify` plus the stored reason, so a completion that already happened is
/// seen by a waiter that arrives afterwards.
#[derive(Debug, Default)]
pub struct Completion {
    reason: Mutex<Option<TriggerReason>>,
    notify: Notify,
}

impl Completion {
    /// An incomplete completion.
    pub fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    /// Clear it for a new dispatch.
    pub fn reset(&self) {
        *self.reason.lock().unwrap_or_else(|p| p.into_inner()) = None;
    }

    /// Complete it with `reason`; the first reason stands.
    pub fn complete(&self, reason: TriggerReason) {
        let mut guard = self.reason.lock().unwrap_or_else(|p| p.into_inner());
        if guard.is_none() {
            *guard = Some(reason);
            drop(guard);
            self.notify.notify_waiters();
        }
    }

    /// The reason, once completed.
    pub fn reason(&self) -> Option<TriggerReason> {
        *self.reason.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Wait for the completion.
    pub async fn wait(&self) -> TriggerReason {
        loop {
            let notified = self.notify.notified();
            if let Some(reason) = self.reason() {
                return reason;
            }
            notified.await;
        }
    }
}

/// The shared state of one dispatch: its trsyncs and completion.
pub struct TriggerGroup {
    trsyncs: Mutex<Vec<Arc<McuTrsync>>>,
    completion: Arc<Completion>,
    /// Whether the group can still trigger, as far as the host knows.
    can_trigger: AtomicBool,
}

impl TriggerGroup {
    /// Recompute the minimum acknowledged time and extend each MCU's deadline
    /// (`handle_trsync_state`).
    fn on_report(&self) {
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
    chip: McuChip,
    group: Weak<TriggerGroup>,
    /// Steps that stop when the group fires.
    stepper_oids: Mutex<Vec<u8>>,
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
        let trsync = Arc::new(Self {
            oid,
            chip: chip.clone(),
            group,
            stepper_oids: Mutex::new(Vec::new()),
            expire_clock: Mutex::new(0),
            expire_ticks: Mutex::new(0),
            min_extend_ticks: Mutex::new(0),
            last_status_clock: Mutex::new(None),
            home_end_clock: Mutex::new(None),
        });
        let registered = Arc::clone(&trsync);
        builder.register_config_callback(Box::new(move |builder, mcu| {
            builder.add_config_cmd(&ConfigTrsync { oid })?;
            let registry = registered.chip.trsync_registry();
            registry.register(Arc::clone(&registered), mcu)
        }))?;
        Ok(trsync)
    }

    /// The oid the firmware assigned.
    pub fn oid(&self) -> u8 {
        self.oid
    }

    /// Register a stepper to stop when the group fires.
    pub fn add_stepper(&self, oid: u8) {
        self.stepper_oids
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push(oid);
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

        mcu.send_msg(&TrsyncStart {
            oid: self.oid,
            report_clock: report_clock as u32,
            report_ticks: report_ticks as u32,
            expire_reason: TriggerReason::CommsTimeout as u8,
        })?;
        for stepper_oid in self
            .stepper_oids
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
        {
            mcu.send_msg(&StepperStopOnTrigger {
                oid: *stepper_oid,
                trsync_oid: self.oid,
            })?;
        }
        mcu.send_msg(&TrsyncSetTimeout {
            oid: self.oid,
            clock: expire_clock as u32,
        })?;
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

    /// Handle one `trsync_state` (`MCU_trsync._handle_trsync_state`).
    fn handle_state(&self, state: TrsyncState) {
        if !state.can_trigger {
            if let Some(reason) = state.reason() {
                if let Some(group) = self.group.upgrade() {
                    group.completion.complete(reason);
                }
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
                if let Some(mcu) = self.chip.mcu() {
                    let _ = mcu.send_msg(&TrsyncTrigger {
                        oid: self.oid,
                        reason: TriggerReason::PastEndTime as u8,
                    });
                }
            }
        }
        if let Some(group) = self.group.upgrade() {
            group.on_report();
        }
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
    by_oid: Mutex<HashMap<u8, Arc<McuTrsync>>>,
    bound: AtomicBool,
}

impl TrsyncRegistry {
    /// An empty registry.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register `trsync` and, on the first call, bind the shared handler.
    ///
    /// # Errors
    /// Returns [`McuError`] if the message is not in the dictionary.
    pub fn register(self: &Arc<Self>, trsync: Arc<McuTrsync>, mcu: &Mcu) -> Result<(), McuError> {
        self.by_oid
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(trsync.oid(), trsync);
        if !self.bound.swap(true, Ordering::SeqCst) {
            let registry = Arc::clone(self);
            mcu.bind_event::<TrsyncState, _>(move |state| registry.route(state))?;
        }
        Ok(())
    }

    fn route(&self, state: TrsyncState) {
        let trsync = self
            .by_oid
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&state.oid)
            .cloned();
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
    pub fn add_stepper(&self, mcu_name: &str, stepper_oid: u8) {
        for trsync in self
            .group
            .trsyncs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
        {
            if trsync.chip.name() == mcu_name {
                trsync.add_stepper(stepper_oid);
                return;
            }
        }
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
    pub fn stop(&self) -> TriggerReason {
        self.group.can_trigger.store(false, Ordering::SeqCst);
        for trsync in self
            .group
            .trsyncs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
        {
            if let Some(mcu) = trsync.chip.mcu() {
                let _ = mcu.send_msg(&TrsyncTrigger {
                    oid: trsync.oid,
                    reason: TriggerReason::HostRequest as u8,
                });
            }
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

    use crate::core::klippy::interface::devices::test::TestDevice;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::{Dictionary, Mcu};
    use crate::core::klippy::pins::PrinterPins;
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::json;

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
        let mcu = Mcu::for_test(name, Interface::new(TestDevice::new(Vec::new())));
        mcu.install_dictionary(dictionary()).unwrap();
        Arc::new(mcu)
    }

    fn chip(name: &str, mcu: Arc<Mcu>) -> McuChip {
        let chip = McuChip::new(
            name.to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::new(PrinterPins::new()),
        );
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

    #[tokio::test]
    async fn test_a_registry_routes_by_oid() {
        // A report with an unknown oid is dropped; the registered one reaches
        // its trsync.
        let mcu = identified_mcu("mcu");
        let dispatch = TriggerDispatch::new(vec![chip("mcu", Arc::clone(&mcu))]).unwrap();
        let trsync = first(&dispatch);
        let registry = trsync.chip.trsync_registry();
        registry.register(Arc::clone(&trsync), &mcu).unwrap();

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
}
