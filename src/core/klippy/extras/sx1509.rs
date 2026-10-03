//! `[sx1509 <name>]` — the SX1509 I2C GPIO expander as a pin chip (upstream's
//! `klippy/extras/sx1509.py`).
//!
//! The SX1509 is a 16-bit I2C GPIO expander with an LED-driver (PWM) block. A
//! board that carries one uses its pins as fans or plain outputs; a consumer
//! writes `pin: sx1509_<name>:PIN_<n>`, where `<name>` is the section's own sub
//! and `<n>` is the pin number (0..15). Upstream registers the chip under
//! `"sx1509_" + name` (`sx1509.py:24`), so `[sx1509 duex]` answers
//! `sx1509_duex:PIN_12`.
//!
//! | option | meaning |
//! |---|---|
//! | `i2c_address` | the device address, required, 0..127 |
//! | `i2c_mcu` | the MCU to use (default `mcu`) |
//! | `i2c_speed` | clock in Hz (default 400000, minimum 100000) |
//! | `i2c_bus` / `i2c_software_scl_pin` / `i2c_software_sda_pin` | the hardware bus, or a bit-banged one |
//!
//! Supported pin types are **digital output** and **hardware PWM**, exactly the
//! two upstream builds (`SX1509_digital_out`, `SX1509_pwm`). Every other type
//! (and a pin whose name does not start with `PIN_`) is refused with upstream's
//! `Wrong pin or incompatible type: …! ` message (`sx1509.py:60-66`).
//!
//! # Bring-up
//!
//! The register set is written once at `klippy:connect` (`SX1509.handle_connect`):
//! two reset writes, the oscillator and clock-divider writes, then every cached
//! word register (`REG_DIR`, `REG_DATA`, `REG_PULLUP`, `REG_PULLDOWN`,
//! `REG_INPUT_DISABLE`, `REG_ANALOG_DRIVER_ENABLE`) and every per-pin `REG_I_ON`
//! byte register. A consumer that runs before connect (an `[output_pin]` clearing
//! its direction bit, a PWM arming its LED-driver registers) only edits the
//! cache; the connect write then sends the whole cache in dict order, so the
//! bytes and their order match upstream's.
//!
//! Upstream sends through `MCU_I2C.i2c_write_noack`, a clocked non-blocking bus
//! write. This host's [`McuI2c`] exposes only the awaiting `write`, which the
//! synchronous [`DigitalOut`]/[`PwmOut`] methods cannot await, so the requested
//! bytes are recorded in order and one spawned task sends them (the
//! [`mcp4451`](super::mcp4451) seam: `Handle::try_current()` + `spawn`). With no
//! async runtime (unit tests) only the record is kept.
//!
//! # Load phase
//!
//! The section is `phase = early`, for the same reason `[multi_pin]` and
//! `[adc_scaled]` are: this loader walks a phase's **regular** sections before
//! its **prefix** sections (`load.rs:215-236`), while `[sx1509 <name>]` is
//! prefix-only and a consumer like `[output_pin FAN3]` resolves its `pin:` while
//! it loads. In the generic phase the consumer would load first and report
//! `Unknown pin chip name 'sx1509_duex'`; `phase = early` registers the chip
//! before the generic walk. (`[mcu]`, `[multi_pin]`, `[adc_scaled]` and
//! `[thermistor]` are early for the same reason.)
//!
//! # What is not here
//!
//! Upstream's `SX1509_digital_out.set_pwm` (a digital pin driven as a
//! software PWM, `sx1509.py:136-137`) has no host counterpart: the
//! [`DigitalOut`] trait has no `set_pwm`, and a consumer that wants PWM asks for
//! `setup_pwm`, which builds the hardware LED driver.

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::spi_device::mcu_object_name;
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{ConfigBuilder, I2cMode, McuEndstop, McuError, McuI2c, McuObject};
use crate::core::klippy::pins::{
    Adc, DigitalOut, PinChip, PinError, PinParams, PrinterPins, PwmOut, PINS_OBJECT,
};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form (`[sx1509 <name>]`) exists upstream
// (`sx1509.py:180-181`); `phase = early` is load-bearing, see the module docs.
section!(
    "sx1509",
    order = 16,
    phase = early,
    prefix = load_config_prefix
);

/// Word registers (`sx1509.py:10-18`): the reset pair, the oscillator and clock
/// divider, and the six the cache starts with.
const REG_RESET: u8 = 0x7D;
const REG_CLOCK: u8 = 0x1E;
const REG_MISC: u8 = 0x1F;
const REG_DIR: u8 = 0x0E;
const REG_DATA: u8 = 0x10;
const REG_PULLUP: u8 = 0x06;
const REG_PULLDOWN: u8 = 0x08;
const REG_INPUT_DISABLE: u8 = 0x00;
const REG_ANALOG_DRIVER_ENABLE: u8 = 0x20;

/// Per-pin LED-driver registers, one byte each (`sx1509.py:21-23`).
const REG_I_ON: [u8; 16] = [
    0x2A, 0x2D, 0x30, 0x33, 0x36, 0x3B, 0x40, 0x45, 0x4A, 0x4D, 0x50, 0x53, 0x56, 0x5B, 0x5F, 0x65,
];

/// Upstream passes `default_speed=400000` to `MCU_I2C_from_config`
/// (`sx1509.py:28`), unlike the 100000 of most I2C extras.
const DEFAULT_SPEED: i64 = 400_000;

/// Upstream's `i2c_speed` `minval` (`bus.py:307`).
const MIN_SPEED: i64 = 100_000;

/// The default `_max_duration` of both resources (`sx1509.py:110`, `:150`).
const DEFAULT_MAX_DURATION: f64 = 2.0;

/// One `[sx1509 <name>]`: the I2C device, the register cache, and the chip
/// registration.
pub struct Sx1509 {
    /// The section's sub (`duex`); the chip name is `sx1509_` + this
    /// (`sx1509.py:24`).
    name: String,
    /// The chip name every consumer's `chip:pin` uses.
    chip_name: String,
    /// The I2C device the register writes go through.
    i2c: Arc<McuI2c>,
    /// The configuration builder, for the build-time heater checks.
    config: Arc<ConfigBuilder>,
    /// The cached register values, in upstream's dict order.
    state: Arc<Mutex<Registers>>,
    /// This chip, weakly: the resources a `setup_*` builds hold the chip, and
    /// the pins registry owns it, so a strong self-reference would be a cycle
    /// that outlives a restart.
    self_weak: Weak<Sx1509>,
}

/// The register cache (`SX1509.reg_dict` / `reg_i_on_dict`), kept in insertion
/// order so the connect write sends the same bytes in the same sequence.
#[derive(Debug)]
struct Registers {
    /// Word registers, `(register, value)`.
    word: Vec<(u8, u16)>,
    /// Byte LED-driver registers, `(register, value)`.
    i_on: Vec<(u8, u8)>,
    /// Every transfer this chip asked for, in order. Tests assert these
    /// instead of a real bus.
    writes: Vec<Vec<u8>>,
}

impl Registers {
    /// Upstream's default values (`sx1509.py:29-33`).
    fn new() -> Self {
        Self {
            word: vec![
                (REG_DIR, 0xFFFF),
                (REG_DATA, 0),
                (REG_PULLUP, 0),
                (REG_PULLDOWN, 0),
                (REG_INPUT_DISABLE, 0),
                (REG_ANALOG_DRIVER_ENABLE, 0),
            ],
            i_on: REG_I_ON.iter().map(|reg| (*reg, 0)).collect(),
            writes: Vec::new(),
        }
    }

    /// `clear_bits_in_register` (`sx1509.py:71-75`): the register keeps
    /// everything but `bitmask`.
    fn clear_bits(&mut self, reg: u8, bitmask: u32) {
        if let Some((_, value)) = self.word.iter_mut().find(|(r, _)| *r == reg) {
            // `~bitmask` is truncated to the register's 16 bits, the way the
            // send's two value bytes are.
            *value &= !(bitmask as u16);
        } else if let Some((_, value)) = self.i_on.iter_mut().find(|(r, _)| *r == reg) {
            *value &= (!bitmask) as u8;
        }
    }

    /// `set_bits_in_register` (`sx1509.py:76-80`).
    fn set_bits(&mut self, reg: u8, bitmask: u32) {
        if let Some((_, value)) = self.word.iter_mut().find(|(r, _)| *r == reg) {
            *value |= bitmask as u16;
        } else if let Some((_, value)) = self.i_on.iter_mut().find(|(r, _)| *r == reg) {
            *value |= bitmask as u8;
        }
    }

    /// `set_register` (`sx1509.py:81-85`). The LED-driver registers hold one
    /// byte, so a larger value is truncated the way the send masks it.
    fn set_register(&mut self, reg: u8, value: u16) {
        if let Some((_, current)) = self.word.iter_mut().find(|(r, _)| *r == reg) {
            *current = value;
        } else if let Some((_, current)) = self.i_on.iter_mut().find(|(r, _)| *r == reg) {
            *current = (value & 0xFF) as u8;
        }
    }

    /// `send_register`'s payload (`sx1509.py:86-98`): the register byte, then
    /// two value bytes for a word register or one for a byte register.
    fn register_bytes(&self, reg: u8) -> Option<Vec<u8>> {
        if let Some((_, value)) = self.word.iter().find(|(r, _)| *r == reg) {
            Some(vec![
                reg & 0xFF,
                ((value >> 8) & 0xFF) as u8,
                (value & 0xFF) as u8,
            ])
        } else {
            self.i_on
                .iter()
                .find(|(r, _)| *r == reg)
                .map(|(_, value)| vec![reg & 0xFF, *value & 0xFF])
        }
    }
}

impl Sx1509 {
    /// Read the bus options, register the chip, and arm the connect write.
    ///
    /// Upstream's `SX1509.__init__` (`sx1509.py:25-41`): the I2C device first,
    /// the chip registration second, the connect handler last.
    ///
    /// # Errors
    /// A missing or out-of-range `i2c_address`, an `i2c_speed` below 100000, an
    /// unknown `i2c_mcu`, mismatched software bus pins, or a chip name already
    /// taken.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let identifier = config.identifier();
        let name = config.section().sub.clone().ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{identifier}' must be a '[sx1509 <name>]' section"
            ))
        })?;

        // `MCU_I2C_from_config` (`bus.py:302-330`): address, speed, MCU, bus.
        let address = config.get_int_bounded("i2c_address", None, Some(0), Some(127))? as u8;
        let speed =
            config.get_int_bounded("i2c_speed", Some(DEFAULT_SPEED), Some(MIN_SPEED), None)?;
        let speed = speed as u32;

        let mcu_name = config
            .get_str("i2c_mcu")
            .map(|text| text.trim().to_string())
            .unwrap_or_else(|| "mcu".to_string());
        let mcu_object = printer
            .lookup_object_as::<McuObject>(&mcu_object_name(&mcu_name))
            .ok_or_else(|| {
                ConfigError::new(format!("Section '{identifier}': unknown MCU '{mcu_name}'"))
            })?;

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");

        let mode = match (
            config.get_str("i2c_software_scl_pin"),
            config.get_str("i2c_software_sda_pin"),
        ) {
            (Some(scl), Some(sda)) => {
                let scl_params = pins
                    .lookup_pin(&scl, false, false, Some("scl"))
                    .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
                let sda_params = pins
                    .lookup_pin(&sda, false, false, Some("sda"))
                    .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
                if scl_params.chip_name != mcu_name || sda_params.chip_name != mcu_name {
                    return Err(ConfigError::new(format!(
                        "Section '{identifier}': i2c pins must be on the same mcu '{mcu_name}'"
                    )));
                }
                I2cMode::Software {
                    scl_pin: scl_params.pin,
                    sda_pin: sda_params.pin,
                    speed,
                }
            }
            (None, None) => I2cMode::Hardware {
                bus: config.get_str("i2c_bus"),
                speed,
            },
            _ => {
                return Err(ConfigError::new(format!(
                    "Section '{identifier}': both 'i2c_software_scl_pin' and \
                     'i2c_software_sda_pin' must be set"
                )));
            }
        };

        let i2c = mcu_object.setup_i2c(mode, address);
        let chip = Arc::new_cyclic(|self_weak| Self {
            name: name.clone(),
            chip_name: format!("sx1509_{name}"),
            i2c,
            config: mcu_object.config(),
            state: Arc::new(Mutex::new(Registers::new())),
            self_weak: self_weak.clone(),
        });

        pins.register_chip(&chip.chip_name, chip.clone())
            .map_err(|err| ConfigError::new(err.to_string()))?;

        // Upstream registers `handle_connect` for `klippy:connect`
        // (`sx1509.py:40-41`); weak, so the printer's handler list does not keep
        // the chip (and through it the connected device) alive across a restart.
        let weak = Arc::downgrade(&chip);
        printer.register_event_handler(
            KlippyEvent::KlippyConnect,
            Box::new(move |_| {
                if let Some(chip) = weak.upgrade() {
                    chip.handle_connect();
                }
            }),
        );

        Ok(chip)
    }

    /// A strong handle to this chip, for the resources it builds.
    fn arc(&self) -> Result<Arc<Sx1509>, PinError> {
        self.self_weak
            .upgrade()
            .ok_or_else(|| PinError::Message("the sx1509 section is gone".to_string()))
    }

    fn state(&self) -> MutexGuard<'_, Registers> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The transfers this chip has asked for, in order.
    #[cfg(test)]
    fn writes(&self) -> Vec<Vec<u8>> {
        self.state().writes.clone()
    }

    /// Upstream's `clear_bits_in_register`.
    fn clear_bits(&self, reg: u8, bitmask: u32) {
        self.state().clear_bits(reg, bitmask);
    }

    /// Upstream's `set_bits_in_register`.
    fn set_bits(&self, reg: u8, bitmask: u32) {
        self.state().set_bits(reg, bitmask);
    }

    /// Upstream's `set_register`.
    fn set_register(&self, reg: u8, value: u16) {
        self.state().set_register(reg, value);
    }

    /// Upstream's `send_register`: queue the register's current bytes.
    fn send_register(&self, reg: u8) {
        let bytes = self.state().register_bytes(reg);
        if let Some(bytes) = bytes {
            self.write(vec![bytes]);
        }
    }

    /// Upstream's `handle_connect` (`sx1509.py:42-59`).
    ///
    /// The two reset writes and the oscillator/clock-divider writes come first
    /// and are unconditional; then the whole word-register dict, then the whole
    /// LED-driver dict, in insertion order.
    fn handle_connect(&self) {
        let mut batches = vec![
            vec![REG_RESET, 0x12],
            vec![REG_RESET, 0x34],
            vec![REG_CLOCK, 1 << 6],
            vec![REG_MISC, 1 << 4],
        ];
        {
            let state = self.state();
            for (reg, _) in &state.word {
                if let Some(bytes) = state.register_bytes(*reg) {
                    batches.push(bytes);
                }
            }
            for (reg, _) in &state.i_on {
                if let Some(bytes) = state.register_bytes(*reg) {
                    batches.push(bytes);
                }
            }
        }
        self.write(batches);
    }

    /// Record these transfers, then send them (when a runtime is present).
    fn write(&self, batches: Vec<Vec<u8>>) {
        if batches.is_empty() {
            return;
        }
        {
            let mut state = self.state();
            state.writes.extend(batches.iter().cloned());
        }
        let i2c = Arc::clone(&self.i2c);
        let chip_name = self.chip_name.clone();
        match tokio::runtime::Handle::try_current() {
            Ok(handle) => {
                handle.spawn(async move {
                    for bytes in batches {
                        if let Err(err) = i2c.write(&bytes).await {
                            warn!("'{chip_name}': I2C write failed: {err}");
                            return;
                        }
                    }
                });
            }
            Err(_) => warn!("'{chip_name}': I2C register writes need an async runtime"),
        }
    }
}

impl PinChip for Sx1509 {
    fn setup_digital_out(&self, params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
        if !params.pin.starts_with("PIN_") {
            return Err(wrong_type(&params.pin, "digital_out"));
        }
        let sxpin = parse_pin_number(&params.pin, "digital_out")?;
        let bitmask = pin_bitmask(sxpin)?;
        let max_duration = Arc::new(Mutex::new(DEFAULT_MAX_DURATION));
        register_build_checks(&self.config, Arc::clone(&max_duration), None)
            .map_err(|err| PinError::Message(err.to_string()))?;
        // Upstream clears the direction bit as the resource is built
        // (`sx1509.py:113`).
        self.clear_bits(REG_DIR, bitmask);
        Ok(Arc::new(Sx1509DigitalOut {
            chip: self.arc()?,
            bitmask,
            invert: params.invert,
            max_duration,
        }))
    }

    fn setup_pwm(&self, params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
        if !params.pin.starts_with("PIN_") {
            return Err(wrong_type(&params.pin, "pwm"));
        }
        let sxpin = parse_pin_number(&params.pin, "pwm")?;
        let bitmask = pin_bitmask(sxpin)?;
        // `REG_I_ON[self._sxpin]` (`sx1509.py:145`).
        let i_on_reg = REG_I_ON
            .get(sxpin)
            .copied()
            .ok_or_else(|| PinError::Message(format!("SX1509 has no pin {sxpin}")))?;
        let hardware_pwm = Arc::new(Mutex::new(false));
        let max_duration = Arc::new(Mutex::new(DEFAULT_MAX_DURATION));
        register_build_checks(
            &self.config,
            Arc::clone(&max_duration),
            Some(Arc::clone(&hardware_pwm)),
        )
        .map_err(|err| PinError::Message(err.to_string()))?;
        // Upstream's `SX1509_pwm.__init__` register setup (`sx1509.py:140-161`).
        self.set_bits(REG_INPUT_DISABLE, bitmask);
        self.clear_bits(REG_PULLUP, bitmask);
        self.clear_bits(REG_DIR, bitmask);
        self.set_bits(REG_ANALOG_DRIVER_ENABLE, bitmask);
        self.clear_bits(REG_DATA, bitmask);
        Ok(Arc::new(Sx1509Pwm {
            chip: self.arc()?,
            i_on_reg,
            invert: params.invert,
            hardware_pwm,
            max_duration,
        }))
    }

    fn setup_static_digital_out(&self, params: &PinParams) -> Result<(), PinError> {
        Err(wrong_type(&params.pin, "digital_out"))
    }

    fn setup_adc(&self, params: &PinParams) -> Result<Arc<dyn Adc>, PinError> {
        Err(wrong_type(&params.pin, "adc"))
    }

    fn setup_endstop(&self, params: &PinParams) -> Result<Arc<McuEndstop>, PinError> {
        Err(wrong_type(&params.pin, "endstop"))
    }
}

/// Upstream's `_build_config` checks (`sx1509.py:114-116`, `:145-149`), run at
/// build time on the MCU's configuration builder.
///
/// A PWM also refuses a software cycle (`hardware_pwm` off), which upstream
/// checks before the heater duration. The error is an [`McuError::Config`]
/// because a build callback has no config-error return.
fn register_build_checks(
    config: &Arc<ConfigBuilder>,
    max_duration: Arc<Mutex<f64>>,
    hardware_pwm: Option<Arc<Mutex<bool>>>,
) -> Result<(), McuError> {
    config.register_config_callback(Box::new(move |_builder, _mcu| {
        if let Some(hardware_pwm) = &hardware_pwm {
            let enabled = *hardware_pwm
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            if !enabled {
                return Err(McuError::Config(
                    "SX1509_pwm must have hardware_pwm enabled".to_string(),
                ));
            }
        }
        let max_duration = *max_duration
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        if max_duration != 0.0 {
            return Err(McuError::Config(
                "SX1509 pins are not suitable for heaters".to_string(),
            ));
        }
        Ok(())
    }))
}

/// The pin number in `PIN_<n>` (`int(pin.split('_')[1])`, `sx1509.py:104`).
fn parse_pin_number(pin: &str, pin_type: &str) -> Result<usize, PinError> {
    pin.split('_')
        .nth(1)
        .and_then(|text| text.parse::<usize>().ok())
        .ok_or_else(|| wrong_type(pin, pin_type))
}

/// The one-bit mask for `sxpin` (`1 << self._sxpin`, `sx1509.py:105`).
///
/// Upstream builds the mask with Python's unbounded ints; the cache is 16 bits
/// wide, so a pin above 15 shifts out of the word and its mask becomes a no-op
/// (the register keeps its value). Only a shift this host cannot express is
/// refused.
fn pin_bitmask(sxpin: usize) -> Result<u32, PinError> {
    1u32.checked_shl(sxpin as u32)
        .ok_or_else(|| PinError::Message(format!("SX1509 has no pin {sxpin}")))
}

/// Upstream's `setup_pin` refusal (`sx1509.py:60-66`), shared by every type
/// this chip does not build.
fn wrong_type(pin: &str, pin_type: &str) -> PinError {
    let first4: String = pin.chars().take(4).collect();
    PinError::Message(format!(
        "Wrong pin or incompatible type: {first4} with type {pin_type}! "
    ))
}

/// One digital output on the expander (`SX1509_digital_out`).
struct Sx1509DigitalOut {
    chip: Arc<Sx1509>,
    /// The pin's bit in `REG_DIR` / `REG_DATA`.
    bitmask: u32,
    /// The pin's `!`: the level written is inverted.
    invert: bool,
    /// `setup_max_duration`: a non-zero value means a heater, which upstream
    /// refuses at build (`sx1509.py:114-116`).
    max_duration: Arc<Mutex<f64>>,
}

impl Sx1509DigitalOut {
    /// Upstream's `set_digital` (`sx1509.py:130-135`).
    fn set_digital(&self, value: bool) {
        if u8::from(value) ^ u8::from(self.invert) != 0 {
            self.chip.set_bits(REG_DATA, self.bitmask);
        } else {
            self.chip.clear_bits(REG_DATA, self.bitmask);
        }
        self.chip.send_register(REG_DATA);
    }
}

impl DigitalOut for Sx1509DigitalOut {
    fn setup_max_duration(&self, max_duration: f64) {
        *self
            .max_duration
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = max_duration;
    }

    fn setup_start_value(&self, start_value: bool, _shutdown_value: bool) {
        // Upstream seeds the DATA register from the start value
        // (`sx1509.py:118-131`); the shutdown value is stored but not written.
        let start = start_value ^ self.invert;
        if start {
            self.chip.set_bits(REG_DATA, self.bitmask);
        } else {
            self.chip.clear_bits(REG_DATA, self.bitmask);
        }
    }

    fn queue_digital_out(&self, _clock: u32, value: bool) -> Result<(), McuError> {
        self.set_digital(value);
        Ok(())
    }

    fn update_digital_out(&self, value: bool) -> Result<(), McuError> {
        self.set_digital(value);
        Ok(())
    }
}

/// One hardware PWM on the expander's LED driver (`SX1509_pwm`).
struct Sx1509Pwm {
    chip: Arc<Sx1509>,
    /// The pin's `REG_I_ON` register (`REG_I_ON[self._sxpin]`).
    i_on_reg: u8,
    /// The pin's `!`.
    invert: bool,
    /// `setup_cycle_time`'s hardware flag; upstream refuses a software PWM at
    /// build (`sx1509.py:163-164`).
    hardware_pwm: Arc<Mutex<bool>>,
    /// `setup_max_duration`; a non-zero value is a heater and is refused
    /// (`sx1509.py:165-166`).
    max_duration: Arc<Mutex<f64>>,
}

/// The LED-driver byte for a duty: `~int(255 * value)` for a normal pin, or
/// `int(255 * value)` for an inverted one (`sx1509.py:185`, `:187`),
/// both masked to a byte the way the send is.
fn i_on_byte(value: f64, invert: bool) -> u8 {
    let raw = (255.0 * value) as i32;
    if invert {
        (raw & 0xFF) as u8
    } else {
        ((!raw) & 0xFF) as u8
    }
}

impl PwmOut for Sx1509Pwm {
    fn setup_max_duration(&self, max_duration: f64) {
        *self
            .max_duration
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = max_duration;
    }

    fn setup_cycle_time(&self, _cycle_time: f64, hardware_pwm: bool) {
        *self
            .hardware_pwm
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = hardware_pwm;
    }

    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        // Upstream complements both before clamping when inverted and writes
        // the start level (`sx1509.py:174-181`).
        let (start_value, _shutdown_value) = if self.invert {
            (1.0 - start_value, 1.0 - shutdown_value)
        } else {
            (start_value, shutdown_value)
        };
        let start_value = start_value.max(0.0).min(1.0);
        self.chip
            .set_register(self.i_on_reg, u16::from(i_on_byte(start_value, false)));
    }

    fn set_pwm(&self, _clock: u32, value: f64) -> Result<(), McuError> {
        self.chip
            .set_register(self.i_on_reg, u16::from(i_on_byte(value, self.invert)));
        self.chip.send_register(self.i_on_reg);
        Ok(())
    }

    fn update_pwm(&self, value: f64) -> Result<(), McuError> {
        self.set_pwm(0, value)
    }

    /// Upstream's `SX1509_pwm` defines no `next_aligned_print_time`, so there
    /// is no cycle to align to.
    fn next_aligned_clock(&self, clock: u32, _allow_early: f64) -> Result<u32, McuError> {
        Ok(clock)
    }
}

impl PrinterObject for Sx1509 {
    /// Upstream's `SX1509` defines no `get_status`, so it is not
    /// client-visible.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for Sx1509 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Sx1509")
            .field("name", &self.name)
            .field("chip_name", &self.chip_name)
            .finish_non_exhaustive()
    }
}

/// The section factory: one `[sx1509 <name>]` section.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let chip = Sx1509::new(config, printer)?;
    Ok(chip as Arc<dyn PrinterObject>)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::section::ConfigSection;
    use crate::core::klippy::config::value::ConfigValue;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::reactor::ManualReactor;

    /// An `[sx1509 duex]` section's options, as the loader would hand them over.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("sx1509", Some("duex"));
        for (key, value) in options {
            section.parameters.insert(
                key.to_lowercase(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A printer with `pins` and one MCU, as the loader builds them before any
    /// section runs.
    fn printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        let object = Arc::new(
            McuObject::new(ConfigSection::new("mcu", None), &printer)
                .expect("the MCU registers its chip"),
        );
        printer
            .add_object("mcu", object as Arc<dyn PrinterObject>)
            .unwrap();
        printer
    }

    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    /// A chip whose subsection is `duex`.
    fn chip(printer: &Arc<Printer>) -> Arc<Sx1509> {
        Sx1509::new(&wrap(&section(&[("i2c_address", "62")])), printer).expect("the chip loads")
    }

    fn pins(printer: &Arc<Printer>) -> Arc<PrinterPins> {
        printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins`")
    }

    fn params(pin: &str, invert: bool) -> PinParams {
        PinParams {
            chip_name: "sx1509_duex".to_string(),
            pin: pin.to_string(),
            invert,
            pullup: 0,
            share_type: None,
        }
    }

    // -- the chip and its registration -------------------------------------

    #[test]
    fn test_the_chip_is_registered_under_the_section_name() {
        let printer = printer();
        let _chip = chip(&printer);

        assert!(pins(&printer).chips().contains(&"sx1509_duex".to_string()));
    }

    #[test]
    fn test_a_pin_resolves_through_the_chip_name() {
        let printer = printer();
        let chip = chip(&printer);
        let pins = pins(&printer);

        let out = pins
            .setup_digital_out("sx1509_duex:PIN_12", None)
            .expect("PIN_12 resolves on the sx1509_duex chip");
        out.update_digital_out(true).unwrap();

        // The write is the DATA register with bit 12 set: [reg, hi, lo].
        assert_eq!(chip.writes().last().unwrap(), &vec![0x10, 0x10, 0x00]);
    }

    // -- the connect write sequence ----------------------------------------

    /// `generic-duet2-duex.cfg:315-316` (address 62, no consumers): the reset,
    /// oscillator and clock-divider writes, then every cached register in dict
    /// order (`sx1509.py:37-50`).
    #[test]
    fn test_the_connect_write_sends_the_upstream_sequence() {
        let printer = printer();
        let chip = chip(&printer);
        chip.handle_connect();

        let mut expected = vec![
            vec![0x7D, 0x12],
            vec![0x7D, 0x34],
            vec![0x1E, 0x40],
            vec![0x1F, 0x10],
            // REG_DIR, REG_DATA, REG_PULLUP, REG_PULLDOWN, REG_INPUT_DISABLE,
            // REG_ANALOG_DRIVER_ENABLE, all at their defaults.
            vec![0x0E, 0xFF, 0xFF],
            vec![0x10, 0x00, 0x00],
            vec![0x06, 0x00, 0x00],
            vec![0x08, 0x00, 0x00],
            vec![0x00, 0x00, 0x00],
            vec![0x20, 0x00, 0x00],
        ];
        for reg in REG_I_ON {
            expected.push(vec![reg, 0x00]);
        }
        assert_eq!(chip.writes(), expected);
    }

    /// A digital output clears its direction bit before the connect write, so
    /// the whole cache carries the change (`sx1509.py:113`).
    #[test]
    fn test_a_digital_out_clears_its_direction_bit() {
        let printer = printer();
        let chip = chip(&printer);
        let pins = pins(&printer);
        pins.setup_digital_out("sx1509_duex:PIN_12", None).unwrap();
        chip.handle_connect();

        let writes = chip.writes();
        // REG_DIR (0x0E) is the fifth write; bit 12 (0x1000) is cleared.
        assert_eq!(writes[4], vec![0x0E, 0xEF, 0xFF]);
    }

    // -- set_digital -------------------------------------------------------

    #[test]
    fn test_set_digital_updates_the_data_register() {
        let printer = printer();
        let chip = chip(&printer);
        let pins = pins(&printer);
        let out = pins.setup_digital_out("sx1509_duex:PIN_12", None).unwrap();

        out.update_digital_out(true).unwrap();
        assert_eq!(chip.writes().last().unwrap(), &vec![0x10, 0x10, 0x00]);

        out.update_digital_out(false).unwrap();
        assert_eq!(chip.writes().last().unwrap(), &vec![0x10, 0x00, 0x00]);

        // `queue_digital_out` is the clocked twin and writes the same bytes
        // (`set_digital`, `sx1509.py:130-135`).
        out.queue_digital_out(1234, true).unwrap();
        assert_eq!(chip.writes().last().unwrap(), &vec![0x10, 0x10, 0x00]);
        out.queue_digital_out(1235, false).unwrap();
        assert_eq!(chip.writes().last().unwrap(), &vec![0x10, 0x00, 0x00]);
    }

    #[test]
    fn test_an_inverted_pin_writes_the_opposite_level() {
        let printer = printer();
        let chip = chip(&printer);
        let pins = pins(&printer);
        let out = pins.setup_digital_out("!sx1509_duex:PIN_12", None).unwrap();

        out.update_digital_out(true).unwrap();
        assert_eq!(chip.writes().last().unwrap(), &vec![0x10, 0x00, 0x00]);
        out.update_digital_out(false).unwrap();
        assert_eq!(chip.writes().last().unwrap(), &vec![0x10, 0x10, 0x00]);
    }

    #[test]
    fn test_the_start_value_seeds_the_data_register() {
        let printer = printer();
        let chip = chip(&printer);
        let pins = pins(&printer);
        let out = pins.setup_digital_out("sx1509_duex:PIN_7", None).unwrap();

        // `setup_start_value` writes the bit; the connect write then carries it
        // (`sx1509.py:118-131`).
        out.setup_start_value(true, false);
        chip.handle_connect();
        // REG_DATA (0x10) is the sixth write; bit 7 (0x0080) is set.
        assert_eq!(chip.writes()[5], vec![0x10, 0x00, 0x80]);
    }

    // -- PWM ---------------------------------------------------------------

    /// The corpus fan shape (`generic-duet2-duex.cfg:328-359`): `PIN_12`,
    /// `hardware_pwm: true`, value rounded to a PWM duty.
    #[test]
    fn test_the_pwm_arms_the_led_driver_registers() {
        let printer = printer();
        let chip = chip(&printer);
        let pins = pins(&printer);
        let pwm = pins.setup_pwm("sx1509_duex:PIN_12", None).unwrap();
        pwm.setup_cycle_time(0.1, true);
        pwm.setup_max_duration(0.0);
        pwm.setup_start_value(0.5, 0.0);
        chip.handle_connect();

        let writes = chip.writes();
        // The reset/oscillator writes first; then REG_DIR, REG_DATA, REG_PULLUP,
        // REG_PULLDOWN, REG_INPUT_DISABLE, REG_ANALOG_DRIVER_ENABLE.
        assert_eq!(writes[4], vec![0x0E, 0xEF, 0xFF]); // DIR, bit 12 cleared
        assert_eq!(writes[5], vec![0x10, 0x00, 0x00]); // DATA, cleared
        assert_eq!(writes[6], vec![0x06, 0x00, 0x00]); // PULLUP, cleared from 0
        assert_eq!(writes[8], vec![0x00, 0x10, 0x00]); // INPUT_DISABLE
        assert_eq!(writes[9], vec![0x20, 0x10, 0x00]); // ANALOG_DRIVER_ENABLE
                                                       // `REG_I_ON[12]` is 0x56; `~int(255 * .5) & 0xFF` = 0x80.
        assert_eq!(writes[10 + 12], vec![0x56, 0x80]);
    }

    #[test]
    fn test_set_pwm_writes_the_duty_byte() {
        let printer = printer();
        let chip = chip(&printer);
        let pins = pins(&printer);
        let pwm = pins.setup_pwm("sx1509_duex:PIN_5", None).unwrap();
        pwm.setup_cycle_time(0.1, true);
        pwm.setup_max_duration(0.0);
        pwm.setup_start_value(0.0, 0.0);

        // `~int(255 * .25) & 0xFF` = 0xC0, on `REG_I_ON[5]` = 0x3B
        // (`sx1509.py:184-188`).
        pwm.set_pwm(0, 0.25).unwrap();
        assert_eq!(chip.writes().last().unwrap(), &vec![0x3B, 0xC0]);

        pwm.update_pwm(1.0).unwrap();
        assert_eq!(chip.writes().last().unwrap(), &vec![0x3B, 0x00]);
    }

    // -- option reading and refusals ---------------------------------------

    #[test]
    fn test_the_address_is_required() {
        let err = Sx1509::new(&wrap(&section(&[])), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'i2c_address' in section 'sx1509 duex' must be specified"
        );
    }

    #[test]
    fn test_the_address_is_bounded_by_the_bus_range() {
        let err = Sx1509::new(&wrap(&section(&[("i2c_address", "128")])), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'i2c_address' in section 'sx1509 duex' must have maximum of 127"
        );
    }

    #[test]
    fn test_the_speed_defaults_to_400000_and_has_a_minimum() {
        // The default speed is accepted (`default_speed=400000`,
        // `sx1509.py:28`).
        let defaulted = section(&[("i2c_address", "62")]);
        Sx1509::new(&wrap(&defaulted), &printer()).expect("the default speed is accepted");

        let err = Sx1509::new(
            &wrap(&section(&[("i2c_address", "62"), ("i2c_speed", "50000")])),
            &printer(),
        )
        .map(|_| ())
        .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'i2c_speed' in section 'sx1509 duex' must have minimum of 100000"
        );
    }

    #[test]
    fn test_an_unsupported_type_reports_the_upstream_message() {
        let printer = printer();
        let chip = chip(&printer);

        let err = chip
            .setup_digital_out(&params("PA1", false))
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Wrong pin or incompatible type: PA1 with type digital_out! "
        );

        let err = chip
            .setup_adc(&params("PIN_12", false))
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Wrong pin or incompatible type: PIN_ with type adc! "
        );
    }

    // -- the real loader ---------------------------------------------------

    /// The corpus shape: `[sx1509 duex]` and then an `[output_pin FAN3]` whose
    /// `pin:` names the chip it registered. The chip has to exist by the time
    /// the output pin loads, which is why the section is early.
    fn full_config() -> String {
        "[mcu]\nserial: /dev/not-opened-yet\n\
         [sx1509 duex]\ni2c_address: 62\n\
         [output_pin FAN3]\npin: sx1509_duex:PIN_12\npwm: True\nhardware_pwm: True\n"
            .to_string()
    }

    fn load(text: &str) -> (Arc<Printer>, Result<(), ConfigError>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(text).expect("the config parses");
        let result = printer.load_config(&config);
        (printer, result)
    }

    #[test]
    fn test_the_section_loads_from_a_full_config() {
        let (printer, result) = load(&full_config());
        result.expect("the config loads");

        assert!(pins(&printer).chips().contains(&"sx1509_duex".to_string()));
        printer
            .lookup_object_as::<Sx1509>("sx1509 duex")
            .expect("the section is registered");
        // No `get_status` upstream, so `objects/list` leaves it out.
        assert!(!printer
            .queryable_objects()
            .contains(&"sx1509 duex".to_string()));
    }

    #[test]
    fn test_a_pin_without_the_section_is_an_unknown_chip() {
        let (_printer, result) = load(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [output_pin FAN3]\npin: sx1509_duex:PIN_12\n",
        );

        assert_eq!(
            result.unwrap_err().to_string(),
            "output_pin FAN3: Unknown pin chip name 'sx1509_duex'"
        );
    }
}
