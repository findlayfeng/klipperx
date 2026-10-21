//! The MCU side of a stepper.
//!
//! Upstream's `MCU_stepper` (`klippy/stepper.py:22-260`): it owns the oid and
//! the step/dir pins, adds `config_stepper` while the config is built, and
//! sends `queue_step`/`set_next_step_dir` at runtime. It also re-anchors the
//! firmware's step chain to 0 whenever a configured firmware is taken over
//! (`stepper.py:117-118`, restart list), which is what keeps a fresh session's
//! `last_step_clock = 0` in step with the chain the firmware still carries.
//! The host-side solver and compressor live in `motion::Stepper`; this is the
//! wire half, and the two are joined when `[stepper_*]` sections land (FW5e).
//!
//! The config build also answers upstream's step-on-both-edges question
//! (`_build_config`, `klippy/stepper.py:84-116`): it is the first place the
//! MCU's compile-time constants exist, so it is where `invert_step` becomes
//! `-1` (the firmware's single-timer path) and, on an older firmware, where
//! `step_pulse_ticks` becomes `0`.

use std::sync::{Arc, Mutex};

use super::pin::{pin_number, McuChip};
use crate::core::klippy::cmd::stepper::{ConfigStepper, QueueStep, ResetStepClock, SetNextStepDir};
use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::mcu::{ConfigBuilder, Dictionary, Mcu, McuError, StepBatchClocks};
use crate::core::klippy::motion::stepcompress::StepCommand;
use crate::core::klippy::pins::{PinParams, PrinterPins};

/// The longest pulse that is still symmetric enough for the step to be driven
/// on both edges (`MIN_BOTH_EDGE_DURATION`, `klippy/stepper.py:17`).
const MIN_BOTH_EDGE_DURATION: f64 = 0.000_000_500;

/// The same limit for a firmware that only knows `STEPPER_BOTH_EDGE`
/// (`MIN_OPTIMIZED_BOTH_EDGE_DURATION`, `klippy/stepper.py:18`).
const MIN_OPTIMIZED_BOTH_EDGE_DURATION: f64 = 0.000_000_150;

/// The width `config_stepper` gets when nobody has decided one
/// (`MCU_stepper._build_config`, `klippy/stepper.py:79-80`).
const DEFAULT_STEP_PULSE_DURATION: f64 = 0.000_002;

/// The step-pulse record a `[stepper_*]` section and this firmware half share.
///
/// Upstream keeps all three fields on `MCU_stepper` (`_step_pulse_duration`,
/// `_req_step_both_edge`, `_step_both_edge`; `klippy/stepper.py:44-45`,
/// `:70-73`, `:87-110`): a driver records its request at load, and the config
/// build — the first place the MCU constants exist — resolves it. The section
/// exposes the record as `PrinterStepper::get_pulse_duration` /
/// `setup_default_pulse_duration`; the resolution happens here.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct StepPulse {
    /// The width in seconds, `None` while nobody has decided one. Only a
    /// section knows "wrote no `step_pulse_duration`", and that is what lets a
    /// driver's own default win later (`:70-73`).
    duration: Option<f64>,
    /// `_req_step_both_edge`: a driver asked for step on both edges.
    requested_both_edge: bool,
    /// `_step_both_edge`: the decision the config build resolved.
    both_edge: bool,
}

impl StepPulse {
    /// The width `config_stepper` is built with: the decided one, or the 2 µs
    /// default (`klippy/stepper.py:79-80`).
    fn seconds(&self) -> f64 {
        self.duration.unwrap_or(DEFAULT_STEP_PULSE_DURATION)
    }
}

/// The three firmware constants the both-edge decision reads
/// (`klippy/stepper.py:84-86`). A constant the firmware does not declare
/// counts as `0`, like upstream's `int(constants.get(name, '0'))`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct StepEdgeConstants {
    /// `STEPPER_STEP_BOTH_EDGE`: this firmware has the both-edge step mode.
    step_both_edge: bool,
    /// `STEPPER_BOTH_EDGE`: the older both-edge mode, which needs
    /// `step_pulse_ticks=0` to arm its single timer.
    both_edge: bool,
    /// `STEPPER_OPTIMIZED_UNSTEP`: the firmware toggles the pin itself, which
    /// beats stepping on both edges.
    optimized_unstep: bool,
}

impl StepEdgeConstants {
    /// Read them off an identified MCU's dictionary (`mcu.get_constants()`,
    /// `klippy/stepper.py:84-86`).
    fn from_dictionary(dictionary: Option<&Dictionary>) -> Self {
        let declared = |name: &str| match dictionary {
            Some(dictionary) => {
                matches!(dictionary.constant_f64(name), Some(value) if value != 0.0)
            }
            None => false,
        };
        Self {
            step_both_edge: declared("STEPPER_STEP_BOTH_EDGE"),
            both_edge: declared("STEPPER_BOTH_EDGE"),
            optimized_unstep: declared("STEPPER_OPTIMIZED_UNSTEP"),
        }
    }
}

/// What `config_stepper` gets once the both-edge question is answered.
#[derive(Debug, Clone, Copy, PartialEq)]
struct StepConfig {
    /// `0`/`1` for a single-edge step, `-1` for the both-edge single timer.
    invert_step: i8,
    /// The pulse width in seconds; `0` lets an older firmware arm its timer.
    step_pulse_duration: f64,
    /// Whether the step is driven on both edges.
    both_edge: bool,
}

/// Upstream's both-edge decision chain (`MCU_stepper._build_config`,
/// `klippy/stepper.py:87-101`), including the order the four vetoes apply in.
///
/// `invert_step` is the step pin's own `0`/`1`; the chain turns it into `-1`
/// (the firmware's single-timer path) only when every veto lets the request
/// through.
fn resolve_step_config(
    invert_step: i8,
    requested_both_edge: bool,
    step_pulse_duration: f64,
    constants: StepEdgeConstants,
) -> StepConfig {
    let mut want_both_edges = requested_both_edge;
    if step_pulse_duration > MIN_BOTH_EDGE_DURATION {
        // A wide pulse's rise and fall times need not be symmetric (`:88-92`).
        want_both_edges = false;
    } else if constants.both_edge && step_pulse_duration > MIN_OPTIMIZED_BOTH_EDGE_DURATION {
        // An older MCU and a pulse wider than its fast path allows (`:93-95`).
        want_both_edges = false;
    } else if !constants.both_edge && !constants.step_both_edge {
        // This firmware has no both-edge step mode at all (`:96-98`).
        want_both_edges = false;
    } else if constants.optimized_unstep {
        // The firmware's own optimized step/unstep is better (`:99-101`).
        want_both_edges = false;
    }
    if want_both_edges {
        StepConfig {
            invert_step: -1,
            // The older mode only engages with a zero pulse (`:108-110`).
            step_pulse_duration: if constants.both_edge {
                0.0
            } else {
                step_pulse_duration
            },
            both_edge: true,
        }
    } else {
        StepConfig {
            invert_step,
            step_pulse_duration,
            both_edge: false,
        }
    }
}

/// The oid the config callback assigned.
#[derive(Debug, Default)]
struct StepperState {
    oid: Mutex<Option<u8>>,
}

/// One stepper's firmware side: its oid, and the commands it sends.
#[derive(Debug)]
pub struct McuStepper {
    state: Arc<StepperState>,
    /// The chip this stepper was built on, so a rail can register it with an
    /// endstop's trigger dispatch (which may be on another MCU).
    chip: McuChip,
    /// `dir_pin` was written with `!`: the direction bit on the wire is the
    /// opposite of the solver's.
    ///
    /// Upstream keeps this in the step compressor (`stepcompress_set_invert_sdir`);
    /// here it is applied where the command is built, which is the same wire
    /// boundary.
    invert_dir: bool,
    /// Shared with the chip, so runtime sends reach the connected device.
    mcu: Arc<Mutex<Option<Arc<Mcu>>>>,
    /// The width and the both-edge bookkeeping, shared with the `[stepper_*]`
    /// section that built this resource ([`StepPulse`]).
    pulse: Arc<Mutex<StepPulse>>,
}

impl McuStepper {
    /// Build the resource and register its config callback.
    ///
    /// `invert_step` is `0` normal, `1` inverted, `-1` single-schedule;
    /// `step_pulse_duration` is the pulse width in seconds, `0` letting the
    /// firmware choose (`config_stepper`). `invert_dir` flips the direction bit
    /// of the wire commands, for a `dir_pin` written with `!`.
    ///
    /// The width starts as this argument — a caller that goes through a
    /// `[stepper_*]` section refines it with
    /// [`McuStepper::set_step_pulse_duration`], because only the section knows
    /// whether it wrote the option at all.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        config: Arc<ConfigBuilder>,
        pins: Arc<PrinterPins>,
        chip_name: String,
        mcu: Arc<Mutex<Option<Arc<Mcu>>>>,
        step_pin: PinParams,
        dir_pin: PinParams,
        invert_step: i8,
        step_pulse_duration: f64,
        invert_dir: bool,
        chip: McuChip,
    ) -> Self {
        let state = Arc::new(StepperState::default());
        let callback_state = Arc::clone(&state);
        let pulse = Arc::new(Mutex::new(StepPulse {
            duration: Some(step_pulse_duration),
            ..Default::default()
        }));
        let callback_pulse = Arc::clone(&pulse);
        // Weak, not Arc: the registry owns the chip, and the chip owns this
        // callback's `ConfigBuilder` (`mcu/resource/pin.rs` explains the cycle).
        let callback_pins = Arc::downgrade(&pins);
        config
            .register_config_callback(Box::new(move |builder, mcu| {
                let pins = callback_pins
                    .upgrade()
                    .expect("the pins registry outlives the resources it built");
                callback_state.build(
                    builder,
                    mcu,
                    &pins,
                    &chip_name,
                    &step_pin,
                    &dir_pin,
                    invert_step,
                    &callback_pulse,
                )
            }))
            .expect("a resource is always built before the configuration is");
        Self {
            state,
            chip,
            invert_dir,
            mcu,
            pulse,
        }
    }

    /// The pulse width and whether the step is driven on both edges
    /// (upstream `MCU_stepper.get_pulse_duration`, `klippy/stepper.py:68-69`).
    ///
    /// The width is the 2 µs default while nobody has decided one; the flag is
    /// the config build's decision, so it is `false` until that build runs.
    pub fn pulse_duration(&self) -> (f64, bool) {
        let pulse = *self.pulse_guard();
        (pulse.seconds(), pulse.both_edge)
    }

    /// Record a driver's default pulse width and its step-on-both-edges
    /// request (upstream `MCU_stepper.setup_default_pulse_duration`,
    /// `klippy/stepper.py:70-73`): the width wins **only** when nobody has
    /// decided one, so a `step_pulse_duration` option always outranks it. The
    /// request is resolved by the config build, not here.
    pub fn setup_default_pulse_duration(&self, pulse_duration: f64, step_both_edge: bool) {
        let mut pulse = self.pulse_guard();
        if pulse.duration.is_none() {
            pulse.duration = Some(pulse_duration);
        }
        pulse.requested_both_edge = step_both_edge;
    }

    /// Record the `step_pulse_duration` the `[stepper_*]` section itself wrote,
    /// `None` when it wrote none.
    ///
    /// This is the half of upstream's `_step_pulse_duration` initialization the
    /// chip path cannot carry: [`McuStepper::new`] only takes a number, so the
    /// record starts as if the option was written and the section corrects it
    /// right after construction. Without that, a driver's default
    /// ([`McuStepper::setup_default_pulse_duration`]) could never win over the
    /// parsed 2 µs default.
    pub fn set_step_pulse_duration(&self, duration: Option<f64>) {
        self.pulse_guard().duration = duration;
    }

    fn pulse_guard(&self) -> std::sync::MutexGuard<'_, StepPulse> {
        self.pulse
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The chip this stepper was built on.
    pub fn chip(&self) -> &McuChip {
        &self.chip
    }

    /// The oid the config callback assigned.
    ///
    /// # Errors
    /// Returns [`McuError::Config`] before the configuration has been built.
    pub fn oid(&self) -> Result<u8, McuError> {
        self.state
            .oid
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .ok_or_else(|| McuError::Config("stepper is not configured yet".to_string()))
    }

    /// The connected device, or `None` before connect.
    ///
    /// The resource shares the chip's connection slot, so this is how a caller
    /// that needs the clock frequency or a response query reaches the MCU.
    pub fn mcu(&self) -> Option<Arc<Mcu>> {
        self.mcu
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }

    /// Whether `dir_pin` was written with `!`.
    pub fn invert_dir(&self) -> bool {
        self.invert_dir
    }

    /// Send a runtime command to the connected device.
    fn send<C: McuCommand>(&self, cmd: &C) -> Result<(), McuError> {
        let mcu = self
            .mcu()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))?;
        mcu.send_msg(cmd)
    }

    /// Queue `count` steps (`queue_step`).
    ///
    /// # Errors
    /// As [`McuStepper::send`].
    pub fn queue_step(&self, interval: u32, count: u16, add: i16) -> Result<(), McuError> {
        self.send(&QueueStep {
            oid: self.oid()?,
            interval,
            count,
            add,
        })
    }

    /// Set the direction of the next queued move (`set_next_step_dir`).
    ///
    /// # Errors
    /// As [`McuStepper::send`].
    pub fn set_next_step_dir(&self, direction: bool) -> Result<(), McuError> {
        self.send(&SetNextStepDir {
            oid: self.oid()?,
            dir: u8::from(direction ^ self.invert_dir),
        })
    }

    /// Reset the stepper's time base (`reset_step_clock`).
    ///
    /// The stepper must be idle. Upstream sends `clock=0` after homing
    /// (`MCU_stepper.note_homing_end`), so the next move's schedule is relative
    /// to a known point.
    ///
    /// # Errors
    /// As [`McuStepper::send`].
    pub fn reset_step_clock(&self, clock: u32) -> Result<(), McuError> {
        self.send(&ResetStepClock {
            oid: self.oid()?,
            clock,
        })
    }

    /// Send the commands a [`motion::Stepper`](crate::core::klippy::motion::Stepper)
    /// produced.
    ///
    /// # Errors
    /// As [`McuStepper::send`].
    pub fn send_steps(&self, commands: &[StepCommand]) -> Result<(), McuError> {
        let oid = self.oid()?;
        let invert_dir = self.invert_dir;
        for command in commands {
            match step_command_to_mcu(oid, invert_dir, command) {
                McuStepCommand::Dir(cmd) => self.send(&cmd)?,
                McuStepCommand::Step(cmd) => self.send(&cmd)?,
            }
        }
        Ok(())
    }

    /// Send the steps a
    /// [`motion::Stepper`](crate::core::klippy::motion::Stepper) produced,
    /// waiting for room in the transport's send queue.
    ///
    /// [`McuStepper::send_steps`] uses the non-blocking send, which is right for
    /// a few commands but gives up on a long move once the send queue stays full
    /// for its bounded wait (`mcu/mod.rs`). The motion flush loop uses this one
    /// instead: a move can be hundreds of `queue_step` commands, and they must
    /// all reach the firmware.
    ///
    /// `clocks` is the window the batch covers (see [`StepBatchClocks`]): every
    /// command goes out through the move pool, with `start` as its
    /// `req_clock` and `completion` as the slot-freeing clock
    /// (`Mcu::send_move_payload`).
    ///
    /// # Errors
    /// As [`McuStepper::send`], plus any transport error from the awaited send.
    pub async fn send_steps_async(
        &self,
        commands: &[StepCommand],
        clocks: StepBatchClocks,
    ) -> Result<(), McuError> {
        let oid = self.oid()?;
        let mcu = self
            .mcu()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))?;
        let invert_dir = self.invert_dir;
        for command in commands {
            let payload = match step_command_to_mcu(oid, invert_dir, command) {
                McuStepCommand::Dir(cmd) => mcu.encode(SetNextStepDir::NAME, &cmd.args())?,
                McuStepCommand::Step(cmd) => mcu.encode(QueueStep::NAME, &cmd.args())?,
            };
            mcu.send_move_payload(payload, clocks.start, clocks.completion)
                .await?;
        }
        Ok(())
    }

    /// Ask the firmware how many steps this stepper has taken.
    ///
    /// Upstream's `MCU_stepper._query_mcu_position` (`klippy/stepper.py:212`):
    /// the signed count of steps since `config_stepper`, which is how the host
    /// aligns its solver with the board after connect or homing. The sign is the
    /// firmware's direction and is flipped here for a `!` direction pin, so the
    /// result reads in the solver's own sign convention.
    ///
    /// # Errors
    /// As [`McuStepper::send`], plus a transport failure or timeout.
    pub async fn query_position(&self, timeout: std::time::Duration) -> Result<i32, McuError> {
        let oid = self.oid()?;
        let mcu = self
            .mcu()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))?;
        let position: crate::core::klippy::cmd::stepper::StepperPosition = mcu
            .call_msg_oid::<
                crate::core::klippy::cmd::stepper::StepperGetPosition,
                crate::core::klippy::cmd::stepper::StepperPosition,
            >(
                &crate::core::klippy::cmd::stepper::StepperGetPosition { oid },
                oid,
                timeout,
            )
            .await?;
        Ok(if self.invert_dir {
            -position.pos
        } else {
            position.pos
        })
    }
}

impl StepperState {
    /// Resolve the pins and add the configuration
    /// (`MCU_stepper._build_config`, `klippy/stepper.py:78-131`).
    #[allow(clippy::too_many_arguments)]
    fn build(
        &self,
        builder: &ConfigBuilder,
        mcu: &Mcu,
        pins: &PrinterPins,
        chip_name: &str,
        step_pin: &PinParams,
        dir_pin: &PinParams,
        invert_step: i8,
        pulse: &Mutex<StepPulse>,
    ) -> Result<(), McuError> {
        let step_number = pin_number(mcu, &pins.resolve_pin(chip_name, &step_pin.pin)?, chip_name)?;
        let dir_number = pin_number(mcu, &pins.resolve_pin(chip_name, &dir_pin.pin)?, chip_name)?;
        // The both-edge decision (`_build_config`, `klippy/stepper.py:84-116`):
        // the constants only exist once the firmware has identified, which is
        // why this — and not the section's construction — is where it happens.
        let constants = StepEdgeConstants::from_dictionary(mcu.dictionary().as_deref());
        let decided = {
            let record = *pulse_guard(pulse);
            resolve_step_config(
                invert_step,
                record.requested_both_edge,
                record.seconds(),
                constants,
            )
        };
        if !constants.step_both_edge {
            // Upstream records this through `configfile.deprecate_mcu_code`
            // (`klippy/stepper.py:103-104`). The config callback has the MCU,
            // not the printer that owns the `configfile` object, so the
            // equivalent warning is logged here instead of landing in the
            // status warnings.
            tracing::warn!(
                "MCU '{chip_name}' has deprecated code (it is missing feature \
                 'STEPPER_STEP_BOTH_EDGE'). Recompiling and flashing is recommended."
            );
        }
        {
            let mut record = pulse_guard(pulse);
            record.duration = Some(decided.step_pulse_duration);
            record.both_edge = decided.both_edge;
        }
        // The pulse width is written in seconds in the config and becomes clock
        // ticks here, where the firmware frequency is known
        // (`MCU.seconds_to_clock`, `klippy/mcu.py:1184-1185`). The clamp is the
        // floor the option already carries — config bounds it to `0.0..=0.001`
        // (`extras/stepper.rs`) — and it is applied **here** because
        // `seconds_to_clock` refuses a negative duration rather than folding it
        // onto `0`. A both-edge step on an older MCU resolves to `0`, which is
        // what arms its single timer.
        let step_pulse_ticks = mcu.seconds_to_clock(decided.step_pulse_duration.max(0.0))? as u32;
        let oid = builder.create_oid()?;
        *self.oid.lock().unwrap_or_else(|poison| poison.into_inner()) = Some(oid);
        builder.add_config_cmd(&ConfigStepper {
            oid,
            step_pin: step_number as u8,
            dir_pin: dir_number as u8,
            invert_step: decided.invert_step,
            step_pulse_ticks,
        })?;
        // Re-anchor the firmware's step chain whenever a **configured**
        // firmware is taken over (upstream `klippy/stepper.py:117-118`: an
        // `on_restart=True` command, which lands in the restart list and is
        // what the reused branch sends — `klippy/mcu.py:1069`; this port maps
        // it to `add_restart_cmd`, as `resource/endstop.rs` does for
        // `endstop_home`).
        //
        // A fresh session's compressor starts at `last_step_clock = 0`, so
        // without this the first `queue_step` interval is added onto the chain
        // the **previous** session left behind (B6 reuses a running firmware):
        // the first deadline lands at `leftover + first` mod 2^32 — seconds
        // late when that wraps ahead (silent), and in the past when it wraps
        // behind → firmware `Timer too close`. C5's sampling measured both
        // (c5c.log +2.4 s late but silent; verify.log −14.9 s in the past →
        // trip), while the estimate stayed aligned throughout (`est − fw`
        // +1..2 ms on every sample).
        builder.add_restart_cmd(&ResetStepClock { oid, clock: 0 })?;
        Ok(())
    }
}

/// The shared record's guard, with a poisoned lock treated as usable.
fn pulse_guard(pulse: &Mutex<StepPulse>) -> std::sync::MutexGuard<'_, StepPulse> {
    pulse.lock().unwrap_or_else(|poison| poison.into_inner())
}

/// The firmware command a [`StepCommand`] becomes for `oid`.
///
/// The step solver works in `u32`/`i32` (its own arithmetic), while the wire
/// messages are `%hu`/`%hi`; this is where the two meet.
pub fn step_command_to_mcu(oid: u8, invert_dir: bool, command: &StepCommand) -> McuStepCommand {
    match *command {
        StepCommand::SetNextStepDir { direction, .. } => McuStepCommand::Dir(SetNextStepDir {
            oid,
            dir: u8::from(direction ^ invert_dir),
        }),
        StepCommand::QueueStep {
            interval,
            count,
            add,
            ..
        } => McuStepCommand::Step(QueueStep {
            oid,
            interval,
            count: count as u16,
            add: add as i16,
        }),
    }
}

/// A step command with its oid resolved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum McuStepCommand {
    /// A direction change.
    Dir(SetNextStepDir),
    /// A run of steps.
    Step(QueueStep),
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::frame::Frame;
    use crate::core::klippy::interface::devices::frame_mock::FrameMock;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::Dictionary;
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::ArgValue;

    #[test]
    fn test_a_direction_change_becomes_set_next_step_dir() {
        let command = StepCommand::SetNextStepDir {
            oid: 99,
            direction: true,
        };

        assert_eq!(
            step_command_to_mcu(3, false, &command),
            McuStepCommand::Dir(SetNextStepDir { oid: 3, dir: 1 })
        );
        // The command's own oid is ignored: the resource owns the real one.
    }

    #[test]
    fn test_a_run_of_steps_becomes_queue_step() {
        let command = StepCommand::QueueStep {
            oid: 99,
            interval: 1000,
            count: 1,
            add: 0,
        };

        assert_eq!(
            step_command_to_mcu(3, false, &command),
            McuStepCommand::Step(QueueStep {
                oid: 3,
                interval: 1000,
                count: 1,
                add: 0,
            })
        );
    }

    #[test]
    fn test_an_inverted_dir_pin_flips_the_wire_direction() {
        let forward = StepCommand::SetNextStepDir {
            oid: 0,
            direction: true,
        };
        let backward = StepCommand::SetNextStepDir {
            oid: 0,
            direction: false,
        };

        assert_eq!(
            step_command_to_mcu(1, true, &forward),
            McuStepCommand::Dir(SetNextStepDir { oid: 1, dir: 0 })
        );
        assert_eq!(
            step_command_to_mcu(1, true, &backward),
            McuStepCommand::Dir(SetNextStepDir { oid: 1, dir: 1 })
        );
    }

    #[test]
    fn test_a_negative_add_survives_the_narrowing() {
        // The full compressor emits accelerating runs; the add is signed.
        let command = StepCommand::QueueStep {
            oid: 0,
            interval: 900,
            count: 40,
            add: -3,
        };

        let McuStepCommand::Step(step) = step_command_to_mcu(0, false, &command) else {
            panic!("a queue_step command becomes a step");
        };
        assert_eq!(step.add, -3);
        assert_eq!(step.count, 40);
    }

    // =====================================================================
    // C5: the firmware's step chain must be re-anchored on a takeover
    // =====================================================================

    /// A dictionary with the stepper's two commands and its two pins.
    fn dictionary() -> Dictionary {
        dictionary_declaring(serde_json::json!({"CLOCK_FREQ": 20000000}))
    }

    /// The same dictionary, with the given firmware compile-time constants.
    fn dictionary_declaring(constants: serde_json::Value) -> Dictionary {
        Dictionary::from_json(serde_json::json!({
            "commands": {
                "allocate_oids count=%c": 2,
                "get_config": 7,
                "finalize_config crc=%u": 6,
                "config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c step_pulse_ticks=%u": 41,
                "reset_step_clock oid=%c clock=%u": 42
            },
            "responses": {
                "config is_config=%c crc=%u is_shutdown=%c move_count=%hu": 9
            },
            "enumerations": {
                "pin": {"PA0": 0, "PA1": 1}
            },
            "config": constants
        }))
        .unwrap()
    }

    /// An identified MCU that sends nowhere, declaring `constants`.
    fn mcu_with(constants: serde_json::Value) -> Mcu {
        let mcu = Mcu::for_test("mcu", Interface::new(FrameMock::new(Vec::new())));
        mcu.install_dictionary(dictionary_declaring(constants))
            .unwrap();
        mcu
    }

    /// An identified MCU that sends nowhere.
    fn mcu() -> Mcu {
        let mcu = Mcu::for_test("mcu", Interface::new(FrameMock::new(Vec::new())));
        mcu.install_dictionary(dictionary()).unwrap();
        mcu
    }

    /// A chip with its pins registry, so a stepper can resolve its pins.
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

    #[tokio::test]
    async fn test_a_stepper_reanchors_the_firmware_chain_on_a_reused_firmware() {
        let (chip, _pins) = chip();
        let params = |pin: &str| PinParams {
            chip_name: "mcu".to_string(),
            pin: pin.to_string(),
            invert: false,
            pullup: 0,
            share_type: None,
        };
        // `_pins` stays bound: the chip holds only a `Weak` to it, and the
        // config callback upgrades it at build time.
        chip.setup_stepper(params("PA0"), params("PA1"), 0, 0.0, false);
        let mcu = mcu();

        let built = chip.config().build(&mcu).unwrap();
        let mut parser = Parser::new();
        mcu.dictionary().unwrap().install(&mut parser).unwrap();
        let name_of = |payload: &[u8]| {
            parser
                .decode(Frame::new(0, payload.to_vec()).into())
                .unwrap()[0]
                .0
                .name
                .clone()
        };

        // Upstream sends `reset_step_clock oid clock=0` whenever a configured
        // firmware is taken over (`klippy/stepper.py:117-118`, an
        // `on_restart=True` command; the reused branch sends the restart list,
        // `klippy/mcu.py:1069`). Without it the fresh session's
        // `last_step_clock = 0` and the firmware's leftover chain disagree,
        // and the first deadline lands at `leftover + first` mod 2^32 — late
        // or in the past (`Timer too close`).
        assert_eq!(
            built.restart.len(),
            1,
            "the stepper re-anchors the firmware chain on takeover"
        );
        let decoded = parser
            .decode(Frame::new(0, built.restart[0].payload().to_vec()).into())
            .unwrap();
        assert_eq!(decoded[0].0.name, "reset_step_clock");
        assert_eq!(
            decoded[0].1,
            vec![ArgValue::UInt8(0), ArgValue::UInt32(0)],
            "oid 0, clock 0 — the chain starts where the compressor does"
        );

        // It stays out of the hashed config: `config_stepper` alone is the
        // CRC's stepper half, and a fresh firmware already boots at 0.
        let config_names: Vec<String> = built
            .config
            .iter()
            .map(|payload| name_of(payload.payload()))
            .collect();
        assert!(
            config_names.iter().any(|name| name == "config_stepper"),
            "{config_names:?}"
        );
        assert!(
            !config_names.iter().any(|name| name == "reset_step_clock"),
            "the re-anchor is a restart command, not a config one: {config_names:?}"
        );
    }

    // =====================================================================
    // both-edge: upstream's decision chain (`klippy/stepper.py:84-116`)
    // =====================================================================

    /// The firmware's three both-edge constants, read the way the config build
    /// reads them — off an identified MCU's dictionary
    /// (`mcu.get_constants()`, `klippy/stepper.py:84-86`).
    fn constants_from_firmware(
        step_both_edge: i64,
        both_edge: i64,
        optimized_unstep: i64,
    ) -> StepEdgeConstants {
        StepEdgeConstants::from_dictionary(Some(&dictionary_declaring(serde_json::json!({
            "CLOCK_FREQ": 20000000,
            "STEPPER_STEP_BOTH_EDGE": step_both_edge,
            "STEPPER_BOTH_EDGE": both_edge,
            "STEPPER_OPTIMIZED_UNSTEP": optimized_unstep,
        }))))
    }

    /// Build the chip's configuration and return the arguments of its
    /// `config_stepper` command.
    fn config_stepper_arguments(chip: &McuChip, mcu: &Mcu) -> Vec<ArgValue> {
        let built = chip.config().build(mcu).unwrap();
        let mut parser = Parser::new();
        mcu.dictionary().unwrap().install(&mut parser).unwrap();
        let decoded = built
            .config
            .iter()
            .map(|payload| {
                parser
                    .decode(Frame::new(0, payload.payload().to_vec()).into())
                    .unwrap()
            })
            .find(|decoded| decoded[0].0.name == "config_stepper")
            .expect("the configuration carries a config_stepper command");
        decoded[0].1.clone()
    }

    /// A constant the firmware does not declare reads as `0`, like upstream's
    /// `int(constants.get(name, '0'))` (`klippy/stepper.py:84-86`).
    #[test]
    fn test_the_both_edge_constants_are_read_off_the_firmware() {
        let missing = StepEdgeConstants::from_dictionary(Some(&dictionary_declaring(
            serde_json::json!({"CLOCK_FREQ": 20000000}),
        )));
        assert_eq!(missing, StepEdgeConstants::default());
        assert_eq!(
            StepEdgeConstants::from_dictionary(None),
            StepEdgeConstants::default()
        );
        assert_eq!(
            constants_from_firmware(0, 0, 0),
            StepEdgeConstants::default()
        );
        assert_eq!(
            constants_from_firmware(1, 1, 1),
            StepEdgeConstants {
                step_both_edge: true,
                both_edge: true,
                optimized_unstep: true,
            }
        );
    }

    /// Rung one: a pulse wider than 500 ns may have asymmetric rise and fall
    /// times, so it vetoes both edges (`klippy/stepper.py:88-92`).
    #[test]
    fn test_a_wide_pulse_duration_vetoes_both_edges() {
        let firmware = constants_from_firmware(1, 0, 0);

        let vetoed = resolve_step_config(1, true, 600e-9, firmware);
        assert!(!vetoed.both_edge);
        assert_eq!(vetoed.invert_step, 1, "the pin's own invert_step survives");
        assert_eq!(vetoed.step_pulse_duration, 600e-9);

        // Exactly 500 ns is not *wider* than the limit and gets through.
        assert!(resolve_step_config(1, true, 500e-9, firmware).both_edge);
    }

    /// Rung two: the older `STEPPER_BOTH_EDGE` mode only runs below 150 ns
    /// (`klippy/stepper.py:93-95`).
    #[test]
    fn test_an_old_firmware_vetoes_both_edges_above_150ns() {
        let firmware = constants_from_firmware(1, 1, 0);

        let vetoed = resolve_step_config(0, true, 200e-9, firmware);
        assert!(!vetoed.both_edge);
        assert_eq!(vetoed.invert_step, 0);
        assert_eq!(vetoed.step_pulse_duration, 200e-9);

        // At the limit it still runs on both edges, and the old mode is armed
        // with a zero pulse (`:108-110`).
        let both = resolve_step_config(0, true, 150e-9, firmware);
        assert!(both.both_edge);
        assert_eq!(both.invert_step, -1);
        assert_eq!(both.step_pulse_duration, 0.0);
    }

    /// Rung three: neither constant means the firmware cannot drive the step on
    /// both edges (`klippy/stepper.py:96-98`). The older constant alone is
    /// enough — that is the single-timer path.
    #[test]
    fn test_a_firmware_without_step_both_edge_vetoes_both_edges() {
        let config = resolve_step_config(0, true, 100e-9, constants_from_firmware(0, 0, 0));
        assert!(!config.both_edge);
        assert_eq!(config.invert_step, 0);
        assert_eq!(config.step_pulse_duration, 100e-9);

        let older = resolve_step_config(0, true, 100e-9, constants_from_firmware(0, 1, 0));
        assert!(older.both_edge);
        assert_eq!(older.invert_step, -1);
        assert_eq!(older.step_pulse_duration, 0.0);
    }

    /// Rung four: the firmware's own optimized step/unstep beats stepping on
    /// both edges (`klippy/stepper.py:99-101`).
    #[test]
    fn test_optimized_unstep_vetoes_both_edges() {
        let config = resolve_step_config(0, true, 100e-9, constants_from_firmware(1, 0, 1));
        assert!(!config.both_edge);
        assert_eq!(config.invert_step, 0);
        assert_eq!(config.step_pulse_duration, 100e-9);
    }

    /// No driver asked: a both-edge-capable firmware still gets the single-edge
    /// configuration this port has always sent.
    #[test]
    fn test_without_a_request_the_single_edge_config_stands() {
        let config = resolve_step_config(0, false, 100e-9, constants_from_firmware(1, 1, 0));
        assert!(!config.both_edge);
        assert_eq!(config.invert_step, 0);
        assert_eq!(config.step_pulse_duration, 100e-9);
    }

    /// A cleared chain on an MCU with both constants: `config_stepper` carries
    /// `invert_step=-1` and `step_pulse_ticks=0` (`klippy/stepper.py:105-116`),
    /// and the record the section reads reports both edges.
    #[tokio::test]
    async fn test_a_both_edge_stepper_is_configured_with_the_single_timer() {
        let (chip, _pins) = chip();
        let params = |pin: &str| PinParams {
            chip_name: "mcu".to_string(),
            pin: pin.to_string(),
            invert: false,
            pullup: 0,
            share_type: None,
        };
        // `extras/stepper.rs` hands the parsed 2 µs default to the chip and then
        // reports that the section wrote no option, so a driver's own default
        // (TMC's 100 ns, with the both-edge request) is what decides.
        let stepper = chip.setup_stepper(params("PA0"), params("PA1"), 0, 0.000_002, false);
        stepper.set_step_pulse_duration(None);
        stepper.setup_default_pulse_duration(100e-9, true);
        let mcu = mcu_with(serde_json::json!({
            "CLOCK_FREQ": 20000000,
            "STEPPER_STEP_BOTH_EDGE": 1,
            "STEPPER_BOTH_EDGE": 1,
        }));

        assert_eq!(
            config_stepper_arguments(&chip, &mcu),
            vec![
                ArgValue::UInt8(0),   // oid
                ArgValue::UInt8(0),   // PA0
                ArgValue::UInt8(1),   // PA1
                ArgValue::UInt8(255), // invert_step -1, a `%c` on the wire
                ArgValue::UInt32(0),  // step_pulse_ticks: arms the single timer
            ]
        );
        assert_eq!(stepper.pulse_duration(), (0.0, true));
    }

    /// Without a request nothing changes: the build keeps the pin's
    /// `invert_step` and the pulse width in ticks, and the record stays
    /// single-edge.
    #[tokio::test]
    async fn test_a_stepper_without_a_both_edge_request_keeps_the_single_edge_config() {
        let (chip, _pins) = chip();
        let params = |pin: &str| PinParams {
            chip_name: "mcu".to_string(),
            pin: pin.to_string(),
            invert: false,
            pullup: 0,
            share_type: None,
        };
        let stepper = chip.setup_stepper(params("PA0"), params("PA1"), 0, 100e-9, false);
        let mcu = mcu_with(serde_json::json!({
            "CLOCK_FREQ": 20000000,
            "STEPPER_STEP_BOTH_EDGE": 1,
            "STEPPER_BOTH_EDGE": 1,
        }));

        assert_eq!(
            config_stepper_arguments(&chip, &mcu),
            vec![
                ArgValue::UInt8(0),
                ArgValue::UInt8(0),
                ArgValue::UInt8(1),
                ArgValue::UInt8(0),
                ArgValue::UInt32(2), // 100 ns at 20 MHz
            ]
        );
        assert_eq!(stepper.pulse_duration(), (100e-9, false));
    }
}
