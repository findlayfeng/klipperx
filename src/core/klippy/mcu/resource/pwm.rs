//! The MCU as a PWM chip: `McuPwm`, upstream's `MCU_pwm`.
//!
//! A PWM output is the duty-cycle sibling of [`McuDigitalOut`]. Two very
//! different firmware mechanisms sit behind one interface:
//!
//! * **hardware PWM** — `config_pwm_out` + `queue_pwm_out`, with the duty in
//!   `0..PWM_MAX` and a period in clock ticks. `PWM_MAX` is a firmware constant.
//! * **software PWM** — a digital output (`config_digital_out`) that the
//!   firmware toggles, with `set_digital_out_pwm_cycle` fixing the period and
//!   `queue_digital_out`'s `on_ticks` carrying the duty. Here the full scale is
//!   the period itself, and a duty change may only land on a cycle boundary.
//!
//! Which one is used is [`McuPwm::setup_cycle_time`]'s `hardware_pwm` flag,
//! decided before the configuration is built. Everything after that is the same
//! `set_pwm(clock, value)`.
//!
//! [`McuDigitalOut`]: crate::core::klippy::mcu::McuDigitalOut

use std::sync::{Arc, Mutex, MutexGuard};

use super::pin::{pin_number, MAX_SCHEDULE_TICKS};
use crate::core::klippy::cmd::gpio::{ConfigDigitalOut, QueueDigitalOut, SetDigitalOutPwmCycle};
use crate::core::klippy::cmd::pwm::{ConfigPwmOut, QueuePwmOut};
use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::mcu::{ConfigBuilder, Mcu, McuError};
use crate::core::klippy::pins::{PinError, PinParams, PrinterPins, PwmOut};

/// The default period of a PWM output, in seconds (`klippy/mcu.py:453`).
pub(crate) const DEFAULT_CYCLE_TIME: f64 = 0.100;

/// The default `max_duration`, in seconds (`klippy/mcu.py:454`).
const DEFAULT_MAX_DURATION: f64 = 2.0;

/// The state a PWM's config callback reads and its runtime methods update.
///
/// Shared with the callback so the resource itself is not captured by it (that
/// would be a reference cycle through the builder).
struct PwmState {
    /// Longest a queued duty may be outstanding, seconds; `0.0` = no limit.
    max_duration: Mutex<f64>,
    /// One PWM period, seconds.
    cycle_time: Mutex<f64>,
    /// Whether [`McuPwm::setup_cycle_time`] asked for the hardware path.
    hardware_pwm: Mutex<bool>,
    /// Duty to drive at startup, already inverted, clamped to `0..=1`.
    start_value: Mutex<f64>,
    /// Duty the firmware falls back to on shutdown, already inverted.
    shutdown_value: Mutex<f64>,
    /// The last duty set, for [`PwmOut::next_aligned_clock`].
    last_value: Mutex<f64>,
    /// The clock of the last queued change.
    last_clock: Mutex<u32>,
    /// The oid the firmware assigned, set by the config callback.
    oid: Mutex<Option<u8>>,
    /// The full-scale duty: `PWM_MAX` (hardware) or the period in ticks
    /// (software); set by the config callback.
    pwm_max: Mutex<f64>,
    /// One period in clock ticks; set by the config callback.
    cycle_ticks: Mutex<u32>,
    /// The firmware frequency, cached so alignment needs no MCU lookup.
    clock_freq: Mutex<f64>,
    /// Which path the config callback took.
    hardware: Mutex<bool>,
}

/// One PWM output on an MCU.
pub struct McuPwm {
    state: Arc<PwmState>,
    /// Shared with the chip, so runtime sends reach the connected device.
    mcu: Arc<Mutex<Option<Arc<Mcu>>>>,
    pin: PinParams,
}

impl McuPwm {
    /// Build the resource and register its config callback.
    ///
    /// # Panics
    /// As [`McuDigitalOut::new`](crate::core::klippy::mcu::McuDigitalOut): a
    /// resource is always built while the config file is loaded, before the
    /// configuration is frozen.
    pub(crate) fn new(
        config: Arc<ConfigBuilder>,
        pins: Arc<PrinterPins>,
        chip_name: String,
        mcu: Arc<Mutex<Option<Arc<Mcu>>>>,
        pin: PinParams,
    ) -> Self {
        let state = Arc::new(PwmState {
            max_duration: Mutex::new(DEFAULT_MAX_DURATION),
            cycle_time: Mutex::new(DEFAULT_CYCLE_TIME),
            hardware_pwm: Mutex::new(false),
            // With an inverting pin "off" is a high level, so the initial duty
            // flips too.
            start_value: Mutex::new(f64::from(pin.invert)),
            shutdown_value: Mutex::new(f64::from(pin.invert)),
            last_value: Mutex::new(f64::from(pin.invert)),
            last_clock: Mutex::new(0),
            oid: Mutex::new(None),
            pwm_max: Mutex::new(0.0),
            cycle_ticks: Mutex::new(0),
            clock_freq: Mutex::new(0.0),
            hardware: Mutex::new(false),
        });

        let callback_state = Arc::clone(&state);
        // Weak, not Arc: see `McuDigitalOut::new` — a strong handle would cycle
        // through the chip's `ConfigBuilder` and outlive a restart.
        let callback_pins = Arc::downgrade(&pins);
        let callback_pin = pin.clone();
        config
            .register_config_callback(Box::new(move |builder, mcu| {
                let pins = callback_pins
                    .upgrade()
                    .expect("the pins registry outlives the resources it built");
                callback_state.build(builder, mcu, &pins, &chip_name, &callback_pin)
            }))
            .expect("a resource is always built before the configuration is");

        Self { state, mcu, pin }
    }

    /// The oid the config callback assigned.
    ///
    /// # Errors
    /// Returns [`McuError::Config`] before the configuration has been built.
    fn oid(&self) -> Result<u8, McuError> {
        self.state
            .oid
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .ok_or_else(|| McuError::Config("PWM is not configured yet".to_string()))
    }

    /// The connected device, or a config error before connect.
    fn connected_mcu(&self) -> Result<Arc<Mcu>, McuError> {
        self.mcu
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))
    }

    /// Send a runtime command to the connected device.
    fn send<C: McuCommand>(&self, cmd: &C) -> Result<(), McuError> {
        self.connected_mcu()?.send_msg(cmd)
    }

    /// Lock one of the shared state's fields.
    fn lock<T>(&self, field: impl Fn(&PwmState) -> &Mutex<T>) -> MutexGuard<'_, T> {
        self.state.lock(field)
    }
}

impl PwmOut for McuPwm {
    fn setup_max_duration(&self, max_duration: f64) {
        *self.lock(|state| &state.max_duration) = max_duration;
    }

    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) {
        *self.lock(|state| &state.cycle_time) = cycle_time;
        *self.lock(|state| &state.hardware_pwm) = hardware_pwm;
    }

    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        let (start_value, shutdown_value) = if self.pin.invert {
            (1.0 - start_value, 1.0 - shutdown_value)
        } else {
            (start_value, shutdown_value)
        };
        let start_value = start_value.clamp(0.0, 1.0);
        let shutdown_value = shutdown_value.clamp(0.0, 1.0);
        *self.lock(|state| &state.start_value) = start_value;
        *self.lock(|state| &state.shutdown_value) = shutdown_value;
        *self.lock(|state| &state.last_value) = start_value;
    }

    fn set_pwm(&self, clock: u32, value: f64) -> Result<(), McuError> {
        let invert = self.pin.invert;
        let value = if invert { 1.0 - value } else { value };
        let pwm_max = *self.lock(|state| &state.pwm_max);
        let scaled = (value.clamp(0.0, 1.0) * pwm_max + 0.5) as u32;
        let oid = self.oid()?;

        let hardware = *self.lock(|state| &state.hardware);
        if hardware {
            self.send(&QueuePwmOut {
                oid,
                clock,
                value: scaled as u16,
            })?;
        } else {
            self.send(&QueueDigitalOut {
                oid,
                clock,
                on_ticks: scaled,
            })?;
        }
        *self.lock(|state| &state.last_clock) = clock;
        *self.lock(|state| &state.last_value) = value;
        Ok(())
    }

    fn update_pwm(&self, value: f64) -> Result<(), McuError> {
        let mcu = self.connected_mcu()?;
        let now = mcu
            .estimated_clock()
            .ok_or_else(|| McuError::Config("the firmware clock is unknown".to_string()))?;
        let clock = self.next_aligned_clock(now as u32, 0.0)?;
        self.set_pwm(clock, value)
    }

    fn next_aligned_clock(&self, clock: u32, allow_early: f64) -> Result<u32, McuError> {
        let hardware = *self.lock(|state| &state.hardware);
        if hardware {
            return Ok(clock);
        }
        // Fully on or fully off has no duty to align.
        let last_value = *self.lock(|state| &state.last_value);
        if last_value == 1.0 || last_value == 0.0 {
            return Ok(clock);
        }
        let cycle_ticks = i64::from(*self.lock(|state| &state.cycle_ticks));
        let cycle_time = *self.lock(|state| &state.cycle_time);
        let freq = *self.lock(|state| &state.clock_freq);
        if cycle_ticks <= 0 {
            return Ok(clock);
        }
        let early_seconds = allow_early.min(0.5 * cycle_time);
        let early_ticks = (early_seconds * freq) as i64;
        let req_clock = i64::from(clock) - early_ticks;
        let last_clock = i64::from(*self.lock(|state| &state.last_clock));
        // Round up to the next cycle boundary after `req_clock`.
        let pulses = (req_clock - last_clock + cycle_ticks - 1).div_euclid(cycle_ticks);
        Ok((last_clock + pulses * cycle_ticks) as u32)
    }
}

impl PwmState {
    /// The build-time half: resolve the pin and add the configuration.
    ///
    /// Upstream's `MCU_pwm._build_config` (`klippy/mcu.py:475-527`).
    fn build(
        &self,
        builder: &ConfigBuilder,
        mcu: &Mcu,
        pins: &PrinterPins,
        chip_name: &str,
        pin: &PinParams,
    ) -> Result<(), McuError> {
        let max_duration = *self.lock(|state| &state.max_duration);
        let cycle_time = *self.lock(|state| &state.cycle_time);
        let start_value = *self.lock(|state| &state.start_value);
        let shutdown_value = *self.lock(|state| &state.shutdown_value);
        let hardware_pwm = *self.lock(|state| &state.hardware_pwm);

        if max_duration != 0.0 && start_value != shutdown_value {
            return Err(PinError::MaxDurationMismatch.into());
        }
        let mdur_ticks = mcu.seconds_to_clock(max_duration)?;
        if mdur_ticks > MAX_SCHEDULE_TICKS {
            return Err(PinError::PwmMaxDurationTooLarge.into());
        }
        let cycle_ticks = mcu.seconds_to_clock(cycle_time)?;

        // The first change goes a little into the future so the firmware's
        // scheduler does not see it in the past: upstream uses `now + 0.200`
        // (`klippy/mcu.py:477-478`). Without a clock estimate (a firmware with no
        // `get_uptime`), `0` means "apply as soon as possible".
        let last_clock = match mcu.estimated_clock() {
            Some(clock) => (clock + mcu.seconds_to_clock(0.200)?) as u32,
            None => 0,
        };
        *self.lock(|state| &state.last_clock) = last_clock;
        *self.lock(|state| &state.clock_freq) = mcu.clock_freq()?;
        *self.lock(|state| &state.cycle_ticks) = cycle_ticks as u32;

        // Aliases and reservations first (no dictionary needed), then the
        // firmware's pin enumeration.
        let canonical = pins.resolve_pin(chip_name, &pin.pin)?;
        let number = pin_number(mcu, &canonical, chip_name)?;

        if hardware_pwm {
            let pwm_max = mcu
                .dictionary()
                .and_then(|dictionary| dictionary.constant_f64("PWM_MAX"))
                .ok_or_else(|| McuError::Config("dictionary has no PWM_MAX".to_string()))?;
            *self.lock(|state| &state.pwm_max) = pwm_max;
            *self.lock(|state| &state.hardware) = true;

            builder.request_move_queue_slot()?;
            let oid = builder.create_oid()?;
            *self.lock(|state| &state.oid) = Some(oid);
            builder.add_config_cmd(&ConfigPwmOut {
                oid,
                pin: number,
                cycle_ticks: cycle_ticks as u32,
                value: (start_value * pwm_max + 0.5) as u16,
                default_value: (shutdown_value * pwm_max + 0.5) as u16,
                max_duration: mdur_ticks as u32,
            })?;
            builder.add_restart_cmd(&QueuePwmOut {
                oid,
                clock: last_clock,
                value: (start_value * pwm_max + 0.5) as u16,
            })?;
            return Ok(());
        }

        // Software PWM: the digital-output firmware does the toggling, so the
        // shutdown level has to be one the firmware can express as a level.
        if shutdown_value != 0.0 && shutdown_value != 1.0 {
            return Err(PinError::SoftPwmShutdown.into());
        }
        if cycle_ticks > MAX_SCHEDULE_TICKS {
            return Err(PinError::PwmCycleTimeTooLarge.into());
        }
        let pwm_max = cycle_ticks as f64;
        *self.lock(|state| &state.pwm_max) = pwm_max;
        *self.lock(|state| &state.hardware) = false;

        builder.request_move_queue_slot()?;
        let oid = builder.create_oid()?;
        *self.lock(|state| &state.oid) = Some(oid);
        builder.add_config_cmd(&ConfigDigitalOut {
            oid,
            pin: number,
            value: u8::from(start_value >= 1.0),
            default_value: u8::from(shutdown_value >= 0.5),
            max_duration: mdur_ticks as u32,
        })?;
        builder.add_config_cmd(&SetDigitalOutPwmCycle {
            oid,
            cycle_ticks: cycle_ticks as u32,
        })?;
        // The start duty is part of the init list, not the config: it is a
        // `queue_digital_out`, which the firmware only accepts once running.
        builder.add_init_cmd(&QueueDigitalOut {
            oid,
            clock: last_clock,
            on_ticks: (start_value * pwm_max + 0.5) as u32,
        })?;
        Ok(())
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
    use crate::core::klippy::interface::devices::test::TestDevice;
    use crate::core::klippy::interface::Interface;
    use crate::core::klippy::mcu::Dictionary;
    use crate::core::klippy::mcu::McuChip;
    use crate::core::klippy::msg::proto::ArgValue;
    use serde_json::json;

    /// A dictionary with the pin enumeration and both PWM paths.
    fn dictionary() -> Dictionary {
        Dictionary::from_json(json!({
            "commands": {
                "allocate_oids count=%c": 2,
                "get_config": 7,
                "finalize_config crc=%u": 6,
                "config_pwm_out oid=%c pin=%u cycle_ticks=%u value=%hu default_value=%hu max_duration=%u": 20,
                "queue_pwm_out oid=%c clock=%u value=%hu": 21,
                "config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u": 10,
                "update_digital_out oid=%c value=%c": 11,
                "queue_digital_out oid=%c clock=%u on_ticks=%u": 12,
                "set_digital_out_pwm_cycle oid=%c cycle_ticks=%u": 13
            },
            "responses": {
                "config is_config=%c crc=%u is_shutdown=%c move_count=%hu": 9
            },
            "enumerations": {"pin": {"PA0": 0, "PA1": 1, "PB2": 2}},
            "config": {"CLOCK_FREQ": 20000000, "PWM_MAX": 4095}
        }))
        .unwrap()
    }

    fn mcu() -> Mcu {
        let mcu = Mcu::for_test("mcu", Interface::new(TestDevice::new(Vec::new())));
        mcu.install_dictionary(dictionary()).unwrap();
        mcu
    }

    /// An MCU that also has a clock estimate, so a first change can be placed in
    /// the future rather than at clock zero.
    fn mcu_with_clock() -> Mcu {
        let mcu = mcu();
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

    /// The decoded `(name, args)` of every command in each of the three lists,
    /// from a single build (a builder can only be built once).
    type Commands = Vec<(String, Vec<ArgValue>)>;
    type Lists = (Commands, Commands, Commands);

    fn build_lists(chip: &McuChip, mcu: &Mcu) -> Lists {
        let built = chip.config().build(mcu).unwrap();
        let mut parser = crate::core::klippy::msg::parser::Parser::new();
        mcu.dictionary().unwrap().install(&mut parser).unwrap();
        let decode = |payloads: &[crate::core::klippy::msg::proto::Payload]| {
            payloads
                .iter()
                .map(|payload| {
                    let frame =
                        crate::core::klippy::frame::Frame::new(0, payload.payload().to_vec());
                    let decoded = parser.decode(frame.into()).unwrap();
                    (decoded[0].0.name.clone(), decoded[0].1.clone())
                })
                .collect()
        };
        (
            decode(&built.config),
            decode(&built.init),
            decode(&built.restart),
        )
    }

    #[tokio::test]
    async fn test_a_hardware_pwm_builds_config_pwm_out() {
        let (chip, pins) = chip();
        let pwm = pins.setup_pwm("PA1", None).unwrap();
        pwm.setup_cycle_time(0.1, true);
        pwm.setup_max_duration(0.0);
        pwm.setup_start_value(0.5, 0.0);
        let mcu = mcu();

        let (commands, _init, _restart) = build_lists(&chip, &mcu);

        assert_eq!(commands.len(), 3);
        assert_eq!(commands[0].0, "allocate_oids");
        assert_eq!(commands[1].0, "config_pwm_out");
        assert_eq!(
            commands[1].1,
            vec![
                ArgValue::UInt8(0),
                ArgValue::UInt32(1),         // PA1
                ArgValue::UInt32(2_000_000), // 0.1 s at 20 MHz
                ArgValue::UInt16(2048),      // 0.5 * 4095, rounded
                ArgValue::UInt16(0),
                ArgValue::UInt32(0),
            ]
        );
        assert_eq!(commands[2].0, "finalize_config");
    }

    #[tokio::test]
    async fn test_a_software_pwm_builds_a_digital_output_and_cycle() {
        let (chip, pins) = chip();
        let pwm = pins.setup_pwm("PB2", None).unwrap();
        pwm.setup_cycle_time(0.05, false);
        pwm.setup_max_duration(0.0);
        pwm.setup_start_value(0.25, 0.0);
        let mcu = mcu();

        let (commands, init, _restart) = build_lists(&chip, &mcu);

        assert_eq!(
            commands
                .iter()
                .map(|(name, _)| name.as_str())
                .collect::<Vec<_>>(),
            [
                "allocate_oids",
                "config_digital_out",
                "set_digital_out_pwm_cycle",
                "finalize_config"
            ]
        );
        // value is "on" only at fully on; default is the shutdown level.
        assert_eq!(
            commands[1].1,
            vec![
                ArgValue::UInt8(0),
                ArgValue::UInt32(2),
                ArgValue::UInt8(0),
                ArgValue::UInt8(0),
                ArgValue::UInt32(0),
            ]
        );
        assert_eq!(
            commands[2].1,
            vec![ArgValue::UInt8(0), ArgValue::UInt32(1_000_000)]
        );
        // The start duty is queued at init: 0.25 * 1_000_000.
        assert_eq!(init[0].0, "queue_digital_out");
        assert_eq!(
            init[0].1,
            vec![
                ArgValue::UInt8(0),
                ArgValue::UInt32(0),
                ArgValue::UInt32(250_000)
            ]
        );
    }

    #[tokio::test]
    async fn test_a_software_pwm_rejects_a_partial_shutdown_value() {
        let (chip, pins) = chip();
        let pwm = pins.setup_pwm("PA1", None).unwrap();
        pwm.setup_cycle_time(0.1, false);
        pwm.setup_max_duration(0.0);
        pwm.setup_start_value(0.5, 0.5);
        let mcu = mcu();

        let err = chip.config().build(&mcu).unwrap_err();

        assert!(
            err.to_string()
                .contains("shutdown value must be 0.0 or 1.0"),
            "{err}"
        );
    }

    #[tokio::test]
    async fn test_a_start_shutdown_mismatch_with_a_max_duration_fails() {
        let (chip, pins) = chip();
        let pwm = pins.setup_pwm("PA1", None).unwrap();
        pwm.setup_cycle_time(0.1, true);
        pwm.setup_start_value(0.5, 0.0);
        // The default max_duration is non-zero, so the two have to agree.
        let mcu = mcu();

        let err = chip.config().build(&mcu).unwrap_err();

        assert!(
            err.to_string().contains("start value equal to shutdown"),
            "{err}"
        );
    }

    #[tokio::test]
    async fn test_an_inverting_pwm_flips_the_duty() {
        let (chip, pins) = chip();
        let pwm = pins.setup_pwm("!PA0", None).unwrap();
        pwm.setup_cycle_time(0.1, true);
        pwm.setup_max_duration(0.0);
        // Logically a quarter duty; inverted, it is three quarters.
        pwm.setup_start_value(0.25, 0.0);
        let mcu = mcu();

        let (commands, _init, _restart) = build_lists(&chip, &mcu);

        assert_eq!(commands[1].1[3], ArgValue::UInt16(3071));
        assert_eq!(commands[1].1[4], ArgValue::UInt16(4095));
    }

    #[tokio::test]
    async fn test_next_aligned_clock_rounds_a_software_pwm_to_a_cycle() {
        let (chip, pins) = chip();
        let pwm = pins.setup_pwm("PA1", None).unwrap();
        pwm.setup_cycle_time(0.1, false);
        pwm.setup_max_duration(0.0);
        pwm.setup_start_value(0.5, 0.0);
        let mcu = Arc::new(mcu_with_clock());

        // The build scheduled the start duty at `last_clock`; a request at that
        // clock or just before it stays there, one just after moves a period on.
        let (_config, init, _restart) = build_lists(&chip, mcu.as_ref());
        chip.attach(Arc::clone(&mcu));
        let last = match init[0].1[1] {
            ArgValue::UInt32(clock) => clock,
            ref other => panic!("unexpected clock {other:?}"),
        };
        assert_eq!(pwm.next_aligned_clock(last, 0.0).unwrap(), last);
        assert_eq!(pwm.next_aligned_clock(last - 1, 0.0).unwrap(), last);
        assert_eq!(
            pwm.next_aligned_clock(last + 1, 0.0).unwrap(),
            last + 2_000_000
        );

        // Fully on: no alignment.
        pwm.set_pwm(last, 1.0).unwrap();
        assert_eq!(pwm.next_aligned_clock(123, 0.0).unwrap(), 123);
    }

    #[tokio::test]
    async fn test_a_hardware_pwm_never_aligns() {
        let (chip, pins) = chip();
        let pwm = pins.setup_pwm("PA1", None).unwrap();
        pwm.setup_cycle_time(0.1, true);
        pwm.setup_max_duration(0.0);
        pwm.setup_start_value(0.5, 0.0);
        let mcu = mcu();
        chip.config().build(&mcu).unwrap();

        assert_eq!(pwm.next_aligned_clock(123, 0.0).unwrap(), 123);
    }

    #[tokio::test]
    async fn test_update_pwm_sends_on_the_estimated_clock() {
        let (chip, pins) = chip();
        let pwm = pins.setup_pwm("PA1", None).unwrap();
        pwm.setup_cycle_time(0.1, true);
        pwm.setup_max_duration(0.0);
        pwm.setup_start_value(0.0, 0.0);
        let mcu = Arc::new(mcu_with_clock());
        chip.config().build(mcu.as_ref()).unwrap();
        chip.attach(Arc::clone(&mcu));

        // Hardware PWM: the command is a `queue_pwm_out` and encodes.
        pwm.update_pwm(0.75).unwrap();
    }

    #[tokio::test]
    async fn test_update_pwm_before_connect_is_reported() {
        let (chip, pins) = chip();
        let pwm = pins.setup_pwm("PA1", None).unwrap();
        let mcu = mcu();
        chip.config().build(&mcu).unwrap();

        let err = pwm.update_pwm(0.5).unwrap_err();

        assert!(err.to_string().contains("not connected"), "{err}");
    }
}
