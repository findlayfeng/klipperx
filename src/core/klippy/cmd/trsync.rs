//! `trsync` commands — the trigger dispatch the endstops use.
//!
//! Host view of `src/trsync.c`. A `trsync` is a trigger group: when one of its
//! signals fires (an endstop hit, the host asking, the deadline passing), it
//! dispatches to every registered signal — each stepper stops — and reports the
//! outcome. The host starts it before a homing move, keeps its deadline extended
//! while the firmware keeps reporting, and asks it to stop when the move ends.
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `config_trsync oid=%c` |
//! | host → MCU | `trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c` |
//! | host → MCU | `trsync_set_timeout oid=%c clock=%u` |
//! | host → MCU | `trsync_trigger oid=%c reason=%c` |
//! | MCU → host | `trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u` |
//!
//! `trsync_state` is both a pushed report (every `report_ticks` while the group
//! can still trigger) and the answer to `trsync_trigger`, so [`TrsyncState`]
//! implements [`McuResponse`] and [`McuEvent`](crate::core::klippy::event::McuEvent).

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::mcu::McuError;
use crate::core::klippy::msg::proto::ArgValue;

/// Why a trigger fired.
///
/// Upstream's `MCU_trsync.REASON_*` (`klippy/mcu.py:157-160`); the firmware
/// echoes the number in `trigger_reason`, and the host decides from it whether a
/// homing move succeeded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TriggerReason {
    /// An endstop tripped.
    EndstopHit = 1,
    /// The host asked for a trigger (the move ended).
    HostRequest = 2,
    /// The move's planned end time passed without an endstop.
    PastEndTime = 3,
    /// The firmware did not hear from the host in time.
    CommsTimeout = 4,
}

impl TriggerReason {
    /// The reason for a wire value, or `None` for one this host does not know.
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            1 => Some(Self::EndstopHit),
            2 => Some(Self::HostRequest),
            3 => Some(Self::PastEndTime),
            4 => Some(Self::CommsTimeout),
            _ => None,
        }
    }

    /// Whether this reason means the trigger dispatch failed.
    ///
    /// Upstream treats `>= REASON_COMMS_TIMEOUT` as an error
    /// (`klippy/mcu.py:206`, `:280`) — a hit or a host/past-end trigger is a
    /// normal way for the move to end.
    pub fn is_failure(self) -> bool {
        matches!(self, Self::CommsTimeout)
    }
}

/// Whether a **raw** trsync reason means the homing attempt failed.
///
/// The typed [`TriggerReason`] only covers 1-4; `trigger_analog` failures ride
/// higher codes (`REASON_TRIGGER_ANALOG` and up, see
/// [`trigger_analog`](crate::core::klippy::cmd::trigger_analog)), and upstream
/// classifies by number: `res >= REASON_COMMS_TIMEOUT` is an error
/// (`trigger_analog.py:377`). Codes this host does not know still compare
/// correctly here, which is why callers that may see them use this instead of
/// [`TriggerReason::is_failure`].
pub fn raw_is_failure(raw: u8) -> bool {
    raw >= TriggerReason::CommsTimeout as u8
}

/// `config_trsync oid=%c` — allocate a trigger group.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigTrsync {
    /// The oid the config callback assigned.
    pub oid: u8,
}

impl McuCommand for ConfigTrsync {
    const NAME: &'static str = "config_trsync";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid)]
    }
}

/// `trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c` — arm
/// the group.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrsyncStart {
    /// The trigger group's oid.
    pub oid: u8,
    /// When to send the first `trsync_state` report.
    pub report_clock: u32,
    /// Ticks between reports; `0` disables them.
    pub report_ticks: u32,
    /// The reason to fire with when the deadline passes.
    pub expire_reason: u8,
}

impl McuCommand for TrsyncStart {
    const NAME: &'static str = "trsync_start";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.report_clock),
            ArgValue::UInt32(self.report_ticks),
            ArgValue::UInt8(self.expire_reason),
        ]
    }
}

/// `trsync_set_timeout oid=%c clock=%u` — set the deadline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrsyncSetTimeout {
    /// The trigger group's oid.
    pub oid: u8,
    /// The clock the deadline falls at.
    pub clock: u32,
}

impl McuCommand for TrsyncSetTimeout {
    const NAME: &'static str = "trsync_set_timeout";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt32(self.clock)]
    }
}

/// `trsync_trigger oid=%c reason=%c` — fire the group now.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrsyncTrigger {
    /// The trigger group's oid.
    pub oid: u8,
    /// The reason to record and report.
    pub reason: u8,
}

impl McuCommand for TrsyncTrigger {
    const NAME: &'static str = "trsync_trigger";

    fn args(&self) -> Vec<ArgValue> {
        vec![ArgValue::UInt8(self.oid), ArgValue::UInt8(self.reason)]
    }
}

/// `trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u` — the group's
/// report, pushed and used as the answer to `trsync_trigger`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrsyncState {
    /// The trigger group's oid.
    pub oid: u8,
    /// Whether the group can still trigger.
    pub can_trigger: bool,
    /// The reason recorded when it fired; only meaningful once `can_trigger` is
    /// false.
    pub trigger_reason: u8,
    /// The firmware clock the report was made at.
    pub clock: u32,
}

impl TrsyncState {
    /// The trigger reason, when it is one this host knows.
    pub fn reason(&self) -> Option<TriggerReason> {
        TriggerReason::from_u8(self.trigger_reason)
    }
}

impl McuResponse for TrsyncState {
    const NAME: &'static str = "trsync_state";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            oid: params.get_u8("oid")?,
            can_trigger: params.get_u8("can_trigger")? != 0,
            trigger_reason: params.get_u8("trigger_reason")?,
            clock: params.get_u32("clock")?,
        })
    }
}

impl crate::core::klippy::event::McuEvent for TrsyncState {
    const NAME: &'static str = "trsync_state";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        <Self as McuResponse>::decode(params)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trsync_start_args_follow_the_firmware_order() {
        let cmd = TrsyncStart {
            oid: 2,
            report_clock: 1000,
            report_ticks: 75000,
            expire_reason: TriggerReason::CommsTimeout as u8,
        };
        assert_eq!(
            cmd.args(),
            vec![
                ArgValue::UInt8(2),
                ArgValue::UInt32(1000),
                ArgValue::UInt32(75000),
                ArgValue::UInt8(4),
            ]
        );
    }

    #[test]
    fn trigger_reasons_map_to_the_firmware_numbers() {
        assert_eq!(TriggerReason::from_u8(1), Some(TriggerReason::EndstopHit));
        assert_eq!(TriggerReason::from_u8(2), Some(TriggerReason::HostRequest));
        assert_eq!(TriggerReason::from_u8(3), Some(TriggerReason::PastEndTime));
        assert_eq!(TriggerReason::from_u8(4), Some(TriggerReason::CommsTimeout));
        assert_eq!(TriggerReason::from_u8(9), None);
        assert!(TriggerReason::CommsTimeout.is_failure());
        assert!(!TriggerReason::EndstopHit.is_failure());
        assert!(!TriggerReason::HostRequest.is_failure());
    }

    #[test]
    fn raw_failure_classification_covers_the_typed_and_trigger_analog_codes() {
        // 1-4 agree with the typed view: only a comms timeout is a failure.
        for raw in 1..=3u8 {
            assert!(!raw_is_failure(raw), "{raw}");
        }
        assert!(raw_is_failure(TriggerReason::CommsTimeout as u8));
        // `trigger_analog` error codes 5-8 are failures too, though the typed
        // enum does not name them.
        for raw in 5..=8u8 {
            assert!(raw_is_failure(raw), "{raw}");
            assert!(TriggerReason::from_u8(raw).is_none(), "{raw}");
        }
    }
}
