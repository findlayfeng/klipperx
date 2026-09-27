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

use std::sync::{Arc, Mutex};

use super::pin::{pin_number, McuChip};
use crate::core::klippy::cmd::stepper::{ConfigStepper, QueueStep, ResetStepClock, SetNextStepDir};
use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::mcu::{ConfigBuilder, Mcu, McuError, StepBatchClocks};
use crate::core::klippy::motion::stepcompress::StepCommand;
use crate::core::klippy::pins::{PinParams, PrinterPins};

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
}

impl McuStepper {
    /// Build the resource and register its config callback.
    ///
    /// `invert_step` is `0` normal, `1` inverted, `-1` single-schedule;
    /// `step_pulse_duration` is the pulse width in seconds, `0` letting the
    /// firmware choose (`config_stepper`). `invert_dir` flips the direction bit
    /// of the wire commands, for a `dir_pin` written with `!`.
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
                    step_pulse_duration,
                )
            }))
            .expect("a resource is always built before the configuration is");
        Self {
            state,
            chip,
            invert_dir,
            mcu,
        }
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
            .call_msg::<
                crate::core::klippy::cmd::stepper::StepperGetPosition,
                crate::core::klippy::cmd::stepper::StepperPosition,
            >(
                &crate::core::klippy::cmd::stepper::StepperGetPosition { oid },
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
        step_pulse_duration: f64,
    ) -> Result<(), McuError> {
        let step_number = pin_number(mcu, &pins.resolve_pin(chip_name, &step_pin.pin)?, chip_name)?;
        let dir_number = pin_number(mcu, &pins.resolve_pin(chip_name, &dir_pin.pin)?, chip_name)?;
        // The pulse width is written in seconds in the config and becomes clock
        // ticks here, where the firmware frequency is known
        // (`MCU.seconds_to_clock`, `klippy/mcu.py:1140`).
        let step_pulse_ticks = mcu.seconds_to_clock(step_pulse_duration.max(0.0))? as u32;
        let oid = builder.create_oid()?;
        *self.oid.lock().unwrap_or_else(|poison| poison.into_inner()) = Some(oid);
        builder.add_config_cmd(&ConfigStepper {
            oid,
            step_pin: step_number as u8,
            dir_pin: dir_number as u8,
            invert_step,
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
    use crate::core::klippy::interface::devices::frame_mock::FrameMock;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::Dictionary;

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
            "config": {"CLOCK_FREQ": 20000000}
        }))
        .unwrap()
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
        use crate::core::klippy::frame::Frame;
        use crate::core::klippy::msg::parser::Parser;
        use crate::core::klippy::msg::proto::ArgValue;

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
}
