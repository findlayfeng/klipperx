//! Pin descriptions and the `pins` printer object.
//!
//! A pin in a config file is written by a human — `PA1`, `!PB3`, `^x_endstop`,
//! `zboard:PA0` — and every resource (a digital output, a PWM, an endstop, an
//! ADC channel) has to turn that description into an MCU and a numeric pin
//! before it can build a `config_*` command. This module is that vocabulary:
//! [`PrinterPins`] parses descriptions, tracks who is already using which pin,
//! and holds one [`PinResolver`] per MCU for aliases (`[board_pins]`) and
//! reservations (`RESERVE_PINS_*`, `BUS_PINS_*`).
//!
//! Upstream is `klippy/pins.py`: `PrinterPins` (`:60`) and `PinResolver`
//! (`:18`). The pieces map one to one:
//!
//! | Upstream | Here |
//! |---|---|
//! | `PrinterPins.parse_pin` | [`PrinterPins::parse_pin`] |
//! | `PrinterPins.lookup_pin` | [`PrinterPins::lookup_pin`] |
//! | `PrinterPins.reset_pin_sharing` | [`PrinterPins::reset_pin_sharing`] |
//! | `PrinterPins.allow_multi_use_pin` | [`PrinterPins::allow_multi_use_pin`] |
//! | `PrinterPins.get_pin_resolver` | [`PrinterPins::reserve_pin`] / [`PrinterPins::alias_pin`] / [`PrinterPins::resolve_pin`] |
//! | `PrinterPins.register_chip` | [`PrinterPins::register_chip`] |
//! | `PrinterPins.setup_pin` | [`PrinterPins::setup_digital_out`] / [`PrinterPins::setup_pwm`] / [`PrinterPins::setup_adc`] |
//! | `PinResolver.reserve_pin` / `alias_pin` | [`PinResolver::reserve_pin`] / [`PinResolver::alias_pin`] |
//! | `PinResolver.update_command` | [`PinResolver::resolve`] |
//!
//! # Where the number comes from
//!
//! Upstream leaves the pin **name** in the command text and lets the message
//! parser resolve it against the firmware's `pin` enumeration at encode time.
//! This host has no command text and its encoder takes [`ArgValue`]s, so the
//! name has to become a number before the command is built — and that can only
//! happen after identify, when the dictionary exists. So a resource keeps the
//! name from [`PrinterPins::lookup_pin`] and resolves it in its *config
//! callback*, which runs at build time with the connected `Mcu`
//! (`mcu/config.rs`). [`PinResolver::resolve`] is the half that does not need
//! the dictionary (aliases and reservations); the name-to-number lookup is
//! `Dictionary::enumeration("pin")`.
//!
//! # The object is not queryable
//!
//! Upstream's `objects/list` keeps only objects with a `get_status` method, and
//! `PrinterPins` has none — so `pins` is a registered object a client can never
//! list. [`PrinterObject::is_queryable`] is that distinction here.
//!
//! # Not here
//!
//! * **`MCU_endstop`**: upstream's `PrinterPins.setup_pin` dispatches on the pin
//!   type to `MCU_digital_out` / `MCU_pwm` / `MCU_adc` / `MCU_endstop`. Digital
//!   output, PWM and ADC exist ([`PrinterPins::setup_digital_out`] /
//!   [`PrinterPins::setup_pwm`] / [`PrinterPins::setup_adc`]); the endstop arrives
//!   with F8/C1, as another method on [`PinChip`].
//! * **`BUS_PINS_<bus>`** reservation is done by
//!   [`PrinterPins::resolve_bus_name`] / [`PrinterPins::resolve_bus_value`]
//!   (`McuChip` delegates to them); the SPI/I2C layer that calls them is F6/F7.
//!   `RESERVE_PINS_*` is done, at MCU connect.
//!
//! `[board_pins]` (the section that calls [`PrinterPins::alias_pin`] and
//! [`PrinterPins::reserve_pin`]) is `extras/board_pins.rs`.

use std::collections::{HashMap, HashSet};
use std::fmt;
use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};

use crate::core::klippy::mcu::{Mcu, McuEndstop, McuError, McuStepper};
use crate::core::klippy::printer::PrinterObject;

/// The name clients and other modules use to find this object.
pub const PINS_OBJECT: &str = "pins";

/// What a pin is being set up as.
///
/// The type decides which decorations a description may carry: an endstop may
/// be inverted and pulled up, a digital output or PWM may be inverted, an ADC
/// input neither.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinType {
    Endstop,
    DigitalOut,
    Pwm,
    Adc,
}

impl PinType {
    /// Whether `!` (invert) is allowed for this type (`klippy/pins.py:114`).
    pub fn can_invert(self) -> bool {
        matches!(self, PinType::Endstop | PinType::DigitalOut | PinType::Pwm)
    }

    /// Whether `^` / `~` (pull-up / pull-down) is allowed (`klippy/pins.py:115`).
    pub fn can_pullup(self) -> bool {
        matches!(self, PinType::Endstop)
    }

    /// The name upstream uses in messages ("digital_out", "pwm", …).
    pub fn as_str(self) -> &'static str {
        match self {
            PinType::Endstop => "endstop",
            PinType::DigitalOut => "digital_out",
            PinType::Pwm => "pwm",
            PinType::Adc => "adc",
        }
    }
}

/// A parsed and validated pin description.
///
/// `pin` is the name **as written**; aliases are resolved later, by
/// [`PinResolver::resolve`], so that a config error about the alias points at
/// what the user typed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PinParams {
    /// The MCU the pin belongs to (`mcu`, or the sub of `[mcu <sub>]`).
    pub chip_name: String,
    /// The pin name as written.
    pub pin: String,
    /// `!` was present: the signal is active low.
    pub invert: bool,
    /// `^` (1, pull-up), `~` (-1, pull-down), or `0` for none.
    pub pullup: i8,
    /// The share type this pin was looked up with, if any. Two users may share
    /// a pin only when both name the same share type.
    pub share_type: Option<String>,
}

impl PinParams {
    /// The key a pin is tracked under: `chip:pin`, with the name as written.
    fn share_name(&self) -> String {
        format!("{}:{}", self.chip_name, self.pin)
    }
}

/// A digital output resource: a pin that is driven high or low.
///
/// The trait is the seam between "which pin" (`PrinterPins`) and "what the
/// firmware does with it" (`mcu/resource/pin.rs`). The clocked methods take an absolute
/// firmware clock; turning wall or print time into one is the clock layer's job.
pub trait DigitalOut: Send + Sync {
    /// The longest a scheduled change may be outstanding, in seconds. `0.0`
    /// removes the firmware's limit.
    fn setup_max_duration(&self, max_duration: f64);

    /// The level to drive at startup and the level the firmware falls back to
    /// on shutdown.
    fn setup_start_value(&self, start_value: bool, shutdown_value: bool);

    /// Change the level at `clock` (`queue_digital_out`).
    ///
    /// # Errors
    /// Returns [`McuError`] if the output is not configured yet or the MCU is
    /// not connected.
    fn queue_digital_out(&self, clock: u32, value: bool) -> Result<(), McuError>;

    /// Change the level now (`update_digital_out`).
    ///
    /// # Errors
    /// As [`DigitalOut::queue_digital_out`].
    fn update_digital_out(&self, value: bool) -> Result<(), McuError>;
}

/// A PWM output resource: a pin whose duty cycle is set as a fraction.
///
/// Upstream's `MCU_pwm` (`klippy/mcu.py:451-553`). Whether the pin is a hardware
/// PWM or a software one is decided at build time (see
/// [`setup_cycle_time`](PwmOut::setup_cycle_time)); either way the duty is
/// `0.0..=1.0` and the resource converts it to the firmware's own scale.
pub trait PwmOut: Send + Sync {
    /// The longest a queued duty may be outstanding, in seconds. `0.0` removes
    /// the firmware's limit.
    fn setup_max_duration(&self, max_duration: f64);

    /// Set the PWM period and whether to use the firmware's hardware PWM.
    ///
    /// Must be called before the configuration is built: it chooses between
    /// `config_pwm_out` and `config_digital_out` +
    /// `set_digital_out_pwm_cycle`.
    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool);

    /// The duty to drive at startup and the duty to fall back to on shutdown,
    /// as fractions in `0.0..=1.0`.
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64);

    /// Change the duty at `clock` (upstream's `set_pwm`).
    ///
    /// # Errors
    /// Returns [`McuError`] if the PWM is not configured yet or the MCU is not
    /// connected.
    fn set_pwm(&self, clock: u32, value: f64) -> Result<(), McuError>;

    /// Change the duty as soon as the firmware can.
    ///
    /// The PWM counterpart of [`DigitalOut::update_digital_out`]: there is no
    /// clockless PWM command, so the resource uses
    /// [`Mcu::estimated_clock`](crate::core::klippy::mcu::Mcu::estimated_clock) and
    /// aligns a software PWM to its cycle. This is what a `SET_PIN` does when
    /// there is no print-time scheduler yet (TODO C1).
    ///
    /// # Errors
    /// As [`PwmOut::set_pwm`], plus when the firmware clock cannot be
    /// estimated.
    fn update_pwm(&self, value: f64) -> Result<(), McuError>;

    /// The earliest clock at or after `clock` a software-PWM change may take
    /// effect at.
    ///
    /// Upstream's `next_aligned_print_time`: a software PWM can only change duty
    /// on a cycle boundary, so a caller must round its requested time up. A
    /// hardware PWM, or one currently fully on/off, needs no alignment and
    /// returns `clock`. `allow_early` is how far before `clock` the change may
    /// land, in seconds.
    ///
    /// # Errors
    /// Returns [`McuError`] if the firmware frequency is unknown.
    fn next_aligned_clock(&self, clock: u32, allow_early: f64) -> Result<u32, McuError>;
}

/// A batch of ADC samples: `(firmware clock, value)` pairs, oldest first.
///
/// Upstream dates samples with print time; this host has no print-time layer
/// yet (TODO C1), so the firmware clock is what comes through. The value is
/// already scaled to `0.0..=1.0`.
pub type AdcCallback = Box<dyn Fn(&[(u64, f64)]) + Send + Sync>;

/// An analog input resource.
///
/// Upstream's `MCU_adc` (`klippy/mcu.py:555-655`). A consumer configures the
/// sampling and installs a callback; the firmware then pushes batches after the
/// query is armed at init.
pub trait Adc: Send + Sync {
    /// Configure the periodic query. A `sample_count` of zero disables the
    /// input, so the resource builds no query.
    fn setup_adc_sample(
        &self,
        report_time: f64,
        sample_time: f64,
        sample_count: u32,
        batch_num: u32,
        minval: f64,
        maxval: f64,
        range_check_count: u32,
    );

    /// Install the callback that receives each batch.
    fn setup_adc_callback(&self, callback: AdcCallback);

    /// The last sample seen, as `(firmware clock, value)`.
    fn get_last_value(&self) -> Option<(u64, f64)>;
}

/// The chip (MCU) side of pin setup.
///
/// Upstream's `MCU.setup_pin` dispatches on the pin type to `MCU_digital_out`,
/// `MCU_pwm`, `MCU_adc` or `MCU_endstop` (`klippy/mcu.py:1111-1116`). Here the
/// dispatch is one method per resource kind, added with the kinds; only
/// `digital_out` exists so far.
pub trait PinChip: Send + Sync {
    /// Build the digital output for an already-validated pin.
    ///
    /// # Errors
    /// Returns a [`PinError`] if the chip cannot build the resource.
    fn setup_digital_out(&self, params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError>;

    /// Add a config-time `set_digital_out` for an already-validated pin.
    ///
    /// Unlike [`PinChip::setup_digital_out`] this allocates no oid and returns
    /// no resource: `[static_digital_output]` just holds a pin at a fixed level
    /// for the session (upstream `static_digital_output.py:12-16`). The default
    /// refuses, like the other optional resource kinds.
    ///
    /// # Errors
    /// Returns a [`PinError`] if the chip cannot add the command.
    fn setup_static_digital_out(&self, _params: &PinParams) -> Result<(), PinError> {
        Err(PinError::Unsupported("digital_out".to_string()))
    }

    /// Build the PWM output for an already-validated pin.
    ///
    /// The default refuses, so a chip that does not implement PWM (a test
    /// double, or a future non-MCU chip) reports the same message a missing
    /// resource would.
    ///
    /// # Errors
    /// Returns a [`PinError`] if the chip cannot build the resource.
    fn setup_pwm(&self, _params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
        Err(PinError::Unsupported(PinType::Pwm.as_str().to_string()))
    }

    /// Build the analog input for an already-validated pin.
    ///
    /// The default refuses, like [`PinChip::setup_pwm`].
    ///
    /// # Errors
    /// Returns a [`PinError`] if the chip cannot build the resource.
    fn setup_adc(&self, _params: &PinParams) -> Result<Arc<dyn Adc>, PinError> {
        Err(PinError::Unsupported(PinType::Adc.as_str().to_string()))
    }

    /// Build a stepper from its already-validated step/dir pins.
    ///
    /// A stepper is not quite a pin *type* (upstream's `PrinterStepper` looks
    /// its two pins up and hands them to `MCU_stepper`), but it is still built
    /// by the chip the step pin names, so it rides on the same dispatch. The
    /// default refuses, like [`PinChip::setup_pwm`].
    ///
    /// # Errors
    /// Returns a [`PinError`] if the chip cannot build the resource.
    fn setup_stepper(
        &self,
        _step_pin: &PinParams,
        _dir_pin: &PinParams,
        _invert_step: i8,
        _step_pulse_duration: f64,
        _invert_dir: bool,
    ) -> Result<Arc<McuStepper>, PinError> {
        Err(PinError::Unsupported("stepper".to_string()))
    }

    /// Build an endstop on this chip.
    ///
    /// The default refuses, like [`PinChip::setup_pwm`].
    ///
    /// # Errors
    /// Returns a [`PinError`] if the chip cannot build the resource.
    fn setup_endstop(&self, _params: &PinParams) -> Result<Arc<McuEndstop>, PinError> {
        Err(PinError::Unsupported("endstop".to_string()))
    }
}

/// A pin description or pin-sharing mistake.
///
/// The messages are upstream's (`klippy/pins.py`), because a user sees them as
/// config errors.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PinError {
    /// No MCU is registered under that chip name.
    UnknownChip(String),
    /// The description is malformed; carries the format hint for its type.
    InvalidDescription {
        /// The description as written.
        description: String,
        /// The accepted form, e.g. `"^! [chip_name:] pin_name"`.
        format: String,
    },
    /// A `[board_pins]` alias value contained pin decorations or whitespace.
    InvalidAlias(String),
    /// An alias was already mapped to a different pin.
    AliasConflict {
        alias: String,
        existing: String,
        requested: String,
    },
    /// A pin was already reserved for a different purpose.
    ReserveConflict {
        pin: String,
        existing: String,
        requested: String,
    },
    /// A chip name was registered twice.
    DuplicateChip(String),
    /// The same pin was used twice without a matching share type.
    UsedMultipleTimes(String),
    /// A shared pin was used with a different polarity.
    PolarityMismatch(String),
    /// The pin is reserved (`RESERVE_PINS_*`, `BUS_PINS_*`, `[board_pins] <>`).
    Reserved { pin: String, reserved_for: String },
    /// The pin was referenced under two different names.
    IsAlias { name: String, canonical: String },
    /// The pin name is not in the firmware's `pin` enumeration.
    InvalidName { pin: String, chip: String },
    /// A bus was left out but the firmware does not name bus 0.
    MustSpecifyBus { param: String, chip: String },
    /// A bus name is not in the firmware's bus enumeration.
    UnknownBus { param: String, bus: String },
    /// The chip does not build this kind of resource (yet).
    Unsupported(String),
    /// A chip's own refusal, shown verbatim.
    ///
    /// Upstream chips raise `pins.error("…")` with arbitrary text (the probe
    /// virtual endstop's two refusals, `multi_pin`, …), so those messages must
    /// not be wrapped in the `Unsupported` sentence.
    Message(String),
    /// A stepper's step and direction pins name different MCUs.
    StepperChipMismatch,
    /// A pin with a maximum duration must start and shut down at the same
    /// level, or the firmware would have nothing to fall back to.
    MaxDurationMismatch,
    /// The maximum duration does not fit the firmware's scheduler.
    MaxDurationTooLarge,
    /// A PWM's maximum duration does not fit the firmware's scheduler.
    PwmMaxDurationTooLarge,
    /// A software PWM's cycle does not fit the firmware's scheduler.
    PwmCycleTimeTooLarge,
    /// A software PWM can only fall back to fully on or fully off.
    SoftPwmShutdown,
    /// `sample_count * ADC_MAX` does not fit the 16-bit average the firmware
    /// reports.
    AdcSampleCountTooLarge(u32),
}

impl fmt::Display for PinError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PinError::UnknownChip(name) => write!(f, "Unknown pin chip name '{name}'"),
            PinError::InvalidDescription {
                description,
                format,
            } => write!(
                f,
                "Invalid pin description '{description}'\nFormat is: {format}[chip_name:] pin_name"
            ),
            PinError::InvalidAlias(alias) => write!(f, "Invalid pin alias '{alias}'"),
            PinError::AliasConflict {
                alias,
                existing,
                requested,
            } => write!(
                f,
                "Alias {alias} mapped to {existing} - can't alias to {requested}"
            ),
            PinError::ReserveConflict {
                pin,
                existing,
                requested,
            } => write!(
                f,
                "Pin {pin} reserved for {existing} - can't reserve for {requested}"
            ),
            PinError::DuplicateChip(name) => write!(f, "Duplicate chip name '{name}'"),
            PinError::UsedMultipleTimes(pin) => {
                write!(f, "pin {pin} used multiple times in config")
            }
            PinError::PolarityMismatch(pin) => {
                write!(f, "Shared pin {pin} must have same polarity")
            }
            PinError::Reserved { pin, reserved_for } => {
                write!(f, "pin {pin} is reserved for {reserved_for}")
            }
            PinError::IsAlias { name, canonical } => {
                write!(f, "pin {name} is an alias for {canonical}")
            }
            PinError::InvalidName { pin, chip } => {
                write!(f, "Pin '{pin}' is not a valid pin name on mcu '{chip}'")
            }
            PinError::MustSpecifyBus { param, chip } => {
                write!(f, "Must specify {param} on mcu '{chip}'")
            }
            PinError::UnknownBus { param, bus } => {
                write!(f, "Unknown {param} '{bus}'")
            }
            PinError::Unsupported(kind) => {
                write!(f, "pin type {kind} not supported on this mcu")
            }
            PinError::Message(message) => write!(f, "{message}"),
            PinError::StepperChipMismatch => {
                write!(f, "Stepper dir pin must be on same mcu as step pin")
            }
            PinError::MaxDurationMismatch => write!(
                f,
                "Pin with max duration must have start value equal to shutdown value"
            ),
            PinError::MaxDurationTooLarge => write!(f, "Digital pin max duration too large"),
            PinError::PwmMaxDurationTooLarge => write!(f, "PWM pin max duration too large"),
            PinError::PwmCycleTimeTooLarge => write!(f, "PWM pin cycle time too large"),
            PinError::SoftPwmShutdown => {
                write!(f, "shutdown value must be 0.0 or 1.0 on soft pwm")
            }
            PinError::AdcSampleCountTooLarge(count) => {
                write!(f, "ADC sample_count={count} too large for MCU")
            }
        }
    }
}

impl std::error::Error for PinError {}

// ===========================================================================
// PinResolver
// ===========================================================================

/// Per-MCU aliases and reservations.
///
/// Upstream's `PinResolver` (`klippy/pins.py:18`). It is the part of pin
/// handling that is per-chip, because a `[board_pins]` section names the MCUs it
/// applies to.
pub struct PinResolver {
    /// Whether referencing one pin by two names is an error. Upstream's
    /// `validate_aliases`, on by default; a test turns it off.
    validate_aliases: bool,
    /// Canonical pin name → what it is reserved for.
    reserved: HashMap<String, String>,
    /// Alias → canonical pin name.
    aliases: HashMap<String, String>,
    /// Canonical pin name → the first name it was resolved under, so that a
    /// second name for the same pin can be reported as an alias.
    active_pins: HashMap<String, String>,
}

impl PinResolver {
    /// A resolver with no aliases and no reservations.
    pub fn new() -> Self {
        Self {
            validate_aliases: true,
            reserved: HashMap::new(),
            aliases: HashMap::new(),
            active_pins: HashMap::new(),
        }
    }

    /// Reserve a pin for something else, so a config that also uses it fails.
    ///
    /// Reserving the same pin for the same purpose twice is fine; a different
    /// purpose is [`PinError::ReserveConflict`].
    ///
    /// # Errors
    /// As described above.
    pub fn reserve_pin(&mut self, pin: &str, reserve_name: &str) -> Result<(), PinError> {
        if let Some(existing) = self.reserved.get(pin) {
            if existing != reserve_name {
                return Err(PinError::ReserveConflict {
                    pin: pin.to_string(),
                    existing: existing.clone(),
                    requested: reserve_name.to_string(),
                });
            }
        }
        self.reserved
            .insert(pin.to_string(), reserve_name.to_string());
        Ok(())
    }

    /// Map an alias to a pin name.
    ///
    /// The target must be a bare pin name — no `!`/`^`/`~`/`:` and no
    /// whitespace. Chained aliases are followed, so aliasing to an alias works.
    ///
    /// # Errors
    /// Returns [`PinError::AliasConflict`] if `alias` already maps elsewhere
    /// and [`PinError::InvalidAlias`] if `pin` is not a bare name.
    pub fn alias_pin(&mut self, alias: &str, pin: &str) -> Result<(), PinError> {
        if let Some(existing) = self.aliases.get(alias) {
            if existing != pin {
                return Err(PinError::AliasConflict {
                    alias: alias.to_string(),
                    existing: existing.clone(),
                    requested: pin.to_string(),
                });
            }
        }
        if pin.contains(['^', '~', '!', ':']) || pin.split_whitespace().count() != 1 {
            return Err(PinError::InvalidAlias(pin.to_string()));
        }
        // Follow a chain: aliasing to a name that is itself an alias points at
        // the eventual pin.
        let resolved = self
            .aliases
            .get(pin)
            .cloned()
            .unwrap_or_else(|| pin.to_string());
        self.aliases.insert(alias.to_string(), resolved.clone());
        // Re-point anything that aliased to `alias`.
        for existing_pin in self.aliases.values_mut() {
            if *existing_pin == alias {
                *existing_pin = resolved.clone();
            }
        }
        Ok(())
    }

    /// Resolve the name a config used to its canonical pin.
    ///
    /// Upstream's `PinResolver.update_command` without the text rewrite: it
    /// follows aliases, reports a pin that appears under two names
    /// ([`PinError::IsAlias`]), and refuses a reserved pin
    /// ([`PinError::Reserved`]). Called when a resource builds its command, so
    /// the error names what the user wrote.
    ///
    /// # Errors
    /// As described above.
    pub fn resolve(&mut self, name: &str) -> Result<String, PinError> {
        let canonical = self
            .aliases
            .get(name)
            .cloned()
            .unwrap_or_else(|| name.to_string());
        if self.validate_aliases {
            match self.active_pins.get(&canonical) {
                Some(first) if first != name => {
                    return Err(PinError::IsAlias {
                        name: name.to_string(),
                        canonical: first.clone(),
                    });
                }
                Some(_) => {}
                None => {
                    self.active_pins.insert(canonical.clone(), name.to_string());
                }
            }
        }
        if let Some(reserved_for) = self.reserved.get(&canonical) {
            return Err(PinError::Reserved {
                pin: name.to_string(),
                reserved_for: reserved_for.clone(),
            });
        }
        Ok(canonical)
    }
}

impl Default for PinResolver {
    fn default() -> Self {
        Self::new()
    }
}

// ===========================================================================
// PrinterPins
// ===========================================================================

struct PinsState {
    /// Known chip (MCU) names, in registration order.
    chips: Vec<String>,
    /// One resolver per chip.
    resolvers: HashMap<String, PinResolver>,
    /// The chip handles that build resources, keyed by name.
    chip_impls: HashMap<String, Arc<dyn PinChip>>,
    /// Pins already handed out, keyed `chip:pin`.
    active_pins: HashMap<String, PinParams>,
    /// Pins that may be used by more than one owner, keyed `chip:pin`.
    allow_multi_use: HashSet<String>,
}

/// The `pins` printer object: the pin vocabulary shared by every MCU.
///
/// Registered by [`Printer::load_config`](crate::core::klippy::printer::Printer::load_config)
/// before any section is loaded, so resources and `[board_pins]` can use it
/// while the config file is being read.
pub struct PrinterPins {
    state: Mutex<PinsState>,
}

impl PrinterPins {
    /// An empty registry.
    pub fn new() -> Self {
        Self {
            state: Mutex::new(PinsState {
                chips: Vec::new(),
                resolvers: HashMap::new(),
                chip_impls: HashMap::new(),
                active_pins: HashMap::new(),
                allow_multi_use: HashSet::new(),
            }),
        }
    }

    /// Declare an MCU: its name becomes a valid chip, it gets a resolver, and
    /// `chip` is what builds resources on its behalf.
    ///
    /// Upstream's `register_chip(chip_name, chip)` stores the MCU object, which
    /// `setup_pin` then dispatches to (`klippy/pins.py:126-130`).
    ///
    /// # Errors
    /// Returns [`PinError::DuplicateChip`] if the name is taken.
    pub fn register_chip(&self, chip_name: &str, chip: Arc<dyn PinChip>) -> Result<(), PinError> {
        let mut state = self.lock();
        if state.resolvers.contains_key(chip_name) {
            return Err(PinError::DuplicateChip(chip_name.to_string()));
        }
        state.chips.push(chip_name.to_string());
        state
            .resolvers
            .insert(chip_name.to_string(), PinResolver::new());
        state.chip_impls.insert(chip_name.to_string(), chip);
        Ok(())
    }

    /// The chip names registered so far, in registration order.
    pub fn chips(&self) -> Vec<String> {
        self.lock().chips.clone()
    }

    /// Resolve a bus name against the firmware's enumeration and reserve the
    /// pins the firmware speaks for (`BUS_PINS_<bus>`).
    ///
    /// Upstream's `resolve_bus_name` (`klippy/extras/bus.py:9-32`). `param` is
    /// the enumeration the caller asked for (`spi_bus`, `i2c_bus`); the firmware
    /// may instead publish a single generic `bus` enumeration. `bus` is what the
    /// config said, or `None` to use the bus the firmware names `0`.
    ///
    /// A firmware with no such enumeration has no bus names to check: the
    /// config's value is returned unchanged, or `"0"` when it left the option
    /// out. This is the SPI/I2C layer's helper for F6/F7 — it lives here because
    /// reservations are the pin layer's business.
    ///
    /// # Errors
    /// Returns [`PinError::MustSpecifyBus`] when the bus was left out but the
    /// firmware does not name bus 0, and [`PinError::UnknownBus`] for a name
    /// the enumeration does not have.
    pub fn resolve_bus_name(
        &self,
        mcu: &Mcu,
        param: &str,
        bus: Option<&str>,
    ) -> Result<String, PinError> {
        let dictionary = mcu.dictionary();
        let enums = dictionary
            .as_ref()
            .and_then(|dictionary| dictionary.enumeration(param))
            .or_else(|| {
                dictionary
                    .as_ref()
                    .and_then(|dictionary| dictionary.enumeration("bus"))
            });
        let Some(enums) = enums else {
            return Ok(bus.unwrap_or("0").to_string());
        };

        let bus = match bus {
            Some(bus) => {
                if enums.value(bus).is_none() {
                    return Err(PinError::UnknownBus {
                        param: param.to_string(),
                        bus: bus.to_string(),
                    });
                }
                bus.to_string()
            }
            None => enums
                .name(0)
                .map(str::to_string)
                .ok_or_else(|| PinError::MustSpecifyBus {
                    param: param.to_string(),
                    chip: mcu.name().to_string(),
                })?,
        };

        // The firmware marks the pins a bus owns; reserve them so a config that
        // also drives one fails instead of silently stealing it.
        if let Some(pins) = dictionary
            .as_ref()
            .and_then(|dictionary| dictionary.constant(&format!("BUS_PINS_{bus}")))
            .and_then(|value| value.as_str())
        {
            for pin in pins.split(',') {
                self.reserve_pin(mcu.name(), pin, &bus)?;
            }
        }
        Ok(bus)
    }

    /// Resolve a bus name to the numeric value the firmware's enumeration
    /// expects, reserving the bus pins exactly as
    /// [`PrinterPins::resolve_bus_name`] does.
    ///
    /// Upstream leaves the bus **name** in the command text and the firmware
    /// resolves it against the `%u` enumeration. This port encodes config
    /// commands instead of sending text, so the number has to be looked up here.
    /// Parsing the value as a number is the fallback for a firmware that
    /// publishes no bus enumeration, where the config already wrote a number.
    ///
    /// # Errors
    /// As [`PrinterPins::resolve_bus_name`], plus [`PinError::UnknownBus`] when
    /// there is no enumeration and the value is not a number.
    pub fn resolve_bus_value(
        &self,
        mcu: &Mcu,
        param: &str,
        bus: Option<&str>,
    ) -> Result<u32, PinError> {
        let name = self.resolve_bus_name(mcu, param, bus)?;
        let dictionary = mcu.dictionary();
        let value = dictionary
            .as_ref()
            .and_then(|dictionary| dictionary.enumeration(param))
            .or_else(|| {
                dictionary
                    .as_ref()
                    .and_then(|dictionary| dictionary.enumeration("bus"))
            })
            .and_then(|enums| enums.value(&name));
        match value {
            Some(value) => Ok(value as u32),
            None => name.parse::<u32>().map_err(|_| PinError::UnknownBus {
                param: param.to_string(),
                bus: name,
            }),
        }
    }

    /// Reserve a pin on `chip` (a `[board_pins]` `<>` entry, or an internal
    /// reservation such as `RESERVE_PINS_*`).
    ///
    /// # Errors
    /// Returns [`PinError::UnknownChip`] if the chip is not registered, or
    /// whatever [`PinResolver::reserve_pin`] reports.
    pub fn reserve_pin(&self, chip: &str, pin: &str, reserve_name: &str) -> Result<(), PinError> {
        let mut state = self.lock();
        let resolver = state
            .resolvers
            .get_mut(chip)
            .ok_or_else(|| PinError::UnknownChip(chip.to_string()))?;
        resolver.reserve_pin(pin, reserve_name)
    }

    /// Map an alias on `chip` (a `[board_pins]` entry).
    ///
    /// # Errors
    /// Returns [`PinError::UnknownChip`] if the chip is not registered, or
    /// whatever [`PinResolver::alias_pin`] reports.
    pub fn alias_pin(&self, chip: &str, alias: &str, pin: &str) -> Result<(), PinError> {
        let mut state = self.lock();
        let resolver = state
            .resolvers
            .get_mut(chip)
            .ok_or_else(|| PinError::UnknownChip(chip.to_string()))?;
        resolver.alias_pin(alias, pin)
    }

    /// Resolve a pin name to its canonical form on `chip`.
    ///
    /// # Errors
    /// Returns [`PinError::UnknownChip`] if the chip is not registered, or
    /// whatever [`PinResolver::resolve`] reports.
    pub fn resolve_pin(&self, chip: &str, name: &str) -> Result<String, PinError> {
        let mut state = self.lock();
        let resolver = state
            .resolvers
            .get_mut(chip)
            .ok_or_else(|| PinError::UnknownChip(chip.to_string()))?;
        resolver.resolve(name)
    }

    /// Parse a description into a chip and a pin name.
    ///
    /// `can_invert` / `can_pullup` say which decorations are valid for the
    /// resource being built ([`PinType`] knows). A missing chip prefix means
    /// the main `mcu`, as upstream does.
    ///
    /// # Errors
    /// Returns [`PinError::UnknownChip`] for an unknown prefix and
    /// [`PinError::InvalidDescription`] for a malformed one.
    pub fn parse_pin(
        &self,
        description: &str,
        can_invert: bool,
        can_pullup: bool,
    ) -> Result<PinParams, PinError> {
        let format = {
            let mut format = String::new();
            if can_pullup {
                format.push_str("[^~] ");
            }
            if can_invert {
                format.push_str("[!] ");
            }
            format
        };
        let invalid = || PinError::InvalidDescription {
            description: description.to_string(),
            format: format.clone(),
        };

        let mut desc = description.trim();
        let mut pullup = 0i8;
        if can_pullup && (desc.starts_with('^') || desc.starts_with('~')) {
            pullup = if desc.starts_with('~') { -1 } else { 1 };
            desc = desc[1..].trim();
        }
        let mut invert = false;
        if can_invert && desc.starts_with('!') {
            invert = true;
            desc = desc[1..].trim();
        }

        let (chip_name, pin) = match desc.split_once(':') {
            Some((chip, pin)) => (chip.trim().to_string(), pin.trim().to_string()),
            None => ("mcu".to_string(), desc.to_string()),
        };
        let state = self.lock();
        if !state.resolvers.contains_key(&chip_name) {
            return Err(PinError::UnknownChip(chip_name));
        }
        // The remaining name must be bare: no decorations and no whitespace
        // inside (a name like "PA 1" is a typo, not a pin).
        if pin.contains(['^', '~', '!', ':']) || pin.split_whitespace().count() != 1 {
            return Err(invalid());
        }
        Ok(PinParams {
            chip_name,
            pin,
            invert,
            pullup,
            share_type: None,
        })
    }

    /// Look up a pin for a resource, enforcing single use.
    ///
    /// A pin may be used more than once only when every user passes the same
    /// `share_type` (and the same polarity), or when it was declared
    /// multi-use. The returned parameters are the ones the first lookup stored,
    /// so every user sees the same chip, name and polarity.
    ///
    /// # Errors
    /// Returns [`PinError::UsedMultipleTimes`] for a second unshared use and
    /// [`PinError::PolarityMismatch`] for a shared pin with different
    /// decorations.
    pub fn lookup_pin(
        &self,
        description: &str,
        can_invert: bool,
        can_pullup: bool,
        share_type: Option<&str>,
    ) -> Result<PinParams, PinError> {
        let mut params = self.parse_pin(description, can_invert, can_pullup)?;
        let share_name = params.share_name();
        let mut state = self.lock();

        if let Some(existing) = state.active_pins.get(&share_name) {
            let multi_use = state.allow_multi_use.contains(&share_name);
            let same_share = share_type.is_some() && existing.share_type.as_deref() == share_type;
            if !multi_use && !same_share {
                return Err(PinError::UsedMultipleTimes(params.pin));
            }
            if !multi_use && (params.invert != existing.invert || params.pullup != existing.pullup)
            {
                return Err(PinError::PolarityMismatch(params.pin));
            }
            return Ok(existing.clone());
        }

        params.share_type = share_type.map(str::to_string);
        state.active_pins.insert(share_name, params.clone());
        Ok(params)
    }

    /// Forget that a pin is in use, so it can be looked up again.
    ///
    /// Upstream's `reset_pin_sharing`: the SPI/I2C helpers use it when a
    /// `cs_pin: None` turned out not to be a real pin.
    pub fn reset_pin_sharing(&self, params: &PinParams) {
        self.lock().active_pins.remove(&params.share_name());
    }

    /// Allow a pin to be used by several owners.
    ///
    /// # Errors
    /// Returns [`PinError::UnknownChip`] / [`PinError::InvalidDescription`]
    /// from parsing `description`.
    pub fn allow_multi_use_pin(&self, description: &str) -> Result<(), PinError> {
        let params = self.parse_pin(description, true, true)?;
        self.lock().allow_multi_use.insert(params.share_name());
        Ok(())
    }

    /// Look up a pin and build a digital output on the chip it names.
    ///
    /// The pin is validated and reserved exactly as [`PrinterPins::lookup_pin`]
    /// does; `share_type` is how a caller that legitimately shares a pin (a
    /// multi-pin device) says so.
    ///
    /// # Errors
    /// Returns whatever validation reports, or the chip's own error.
    pub fn setup_digital_out(
        &self,
        description: &str,
        share_type: Option<&str>,
    ) -> Result<Arc<dyn DigitalOut>, PinError> {
        let pin_type = PinType::DigitalOut;
        let params = self.lookup_pin(
            description,
            pin_type.can_invert(),
            pin_type.can_pullup(),
            share_type,
        )?;
        let chip = self.chip(&params.chip_name)?;
        chip.setup_digital_out(&params)
    }

    /// Reserve a pin and add a config-time `set_digital_out` for it.
    ///
    /// No oid and no resource: the pin is held at a fixed level for the session
    /// (upstream `static_digital_output.py`). `can_invert` is on the way
    /// upstream looks the pin up; the level written is the logical 1, so an
    /// inverted pin gets `0`.
    ///
    /// # Errors
    /// Returns whatever validation reports, or the chip's own error.
    pub fn setup_static_digital_out(&self, description: &str) -> Result<(), PinError> {
        let params = self.lookup_pin(description, true, false, None)?;
        let chip = self.chip(&params.chip_name)?;
        chip.setup_static_digital_out(&params)
    }

    /// Look up a pin and build a PWM output on the chip it names.
    ///
    /// # Errors
    /// Returns whatever validation reports, or the chip's own error.
    pub fn setup_pwm(
        &self,
        description: &str,
        share_type: Option<&str>,
    ) -> Result<Arc<dyn PwmOut>, PinError> {
        let pin_type = PinType::Pwm;
        let params = self.lookup_pin(
            description,
            pin_type.can_invert(),
            pin_type.can_pullup(),
            share_type,
        )?;
        let chip = self.chip(&params.chip_name)?;
        chip.setup_pwm(&params)
    }

    /// Look up a pin and build an analog input on the chip it names.
    ///
    /// # Errors
    /// Returns whatever validation reports, or the chip's own error.
    pub fn setup_adc(
        &self,
        description: &str,
        share_type: Option<&str>,
    ) -> Result<Arc<dyn Adc>, PinError> {
        let pin_type = PinType::Adc;
        let params = self.lookup_pin(
            description,
            pin_type.can_invert(),
            pin_type.can_pullup(),
            share_type,
        )?;
        let chip = self.chip(&params.chip_name)?;
        chip.setup_adc(&params)
    }

    /// Look up a pin and build a stepper's two pins on their chip.
    ///
    /// The step pin's `!` becomes upstream's `invert_step` (`0`/`1`); the
    /// direction pin's `!` is carried to the wire layer. The two pins must be on
    /// the same MCU, which upstream checks in `MCU_stepper.__init__`
    /// (`klippy/stepper.py:41-43`).
    ///
    /// # Errors
    /// Returns whatever validation reports, [`PinError::StepperChipMismatch`]
    /// when the pins name different chips, or the chip's own error.
    pub fn setup_stepper(
        &self,
        step_pin: &str,
        dir_pin: &str,
        step_pulse_duration: f64,
    ) -> Result<Arc<McuStepper>, PinError> {
        // A step or direction pin may carry `!`; neither takes a pull-up.
        let step = self.lookup_pin(step_pin, true, false, None)?;
        let dir = self.lookup_pin(dir_pin, true, false, None)?;
        if step.chip_name != dir.chip_name {
            return Err(PinError::StepperChipMismatch);
        }
        let chip = self.chip(&step.chip_name)?;
        chip.setup_stepper(
            &step,
            &dir,
            i8::from(step.invert),
            step_pulse_duration,
            dir.invert,
        )
    }

    /// Look up an endstop pin and build the resource on its chip.
    ///
    /// The pin may carry `!` (invert) and `^`/`~` (pull-up/pull-down).
    ///
    /// # Errors
    /// Returns whatever validation reports, or the chip's own error.
    pub fn setup_endstop(
        &self,
        description: &str,
        share_type: Option<&str>,
    ) -> Result<Arc<McuEndstop>, PinError> {
        let pin_type = PinType::Endstop;
        let params = self.lookup_pin(
            description,
            pin_type.can_invert(),
            pin_type.can_pullup(),
            share_type,
        )?;
        let chip = self.chip(&params.chip_name)?;
        chip.setup_endstop(&params)
    }

    /// The registered chip under `name`, cloned out so the caller does not hold
    /// the state lock while the chip builds a resource.
    fn chip(&self, name: &str) -> Result<Arc<dyn PinChip>, PinError> {
        self.lock()
            .chip_impls
            .get(name)
            .cloned()
            .ok_or_else(|| PinError::UnknownChip(name.to_string()))
    }

    fn lock(&self) -> MutexGuard<'_, PinsState> {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl Default for PrinterPins {
    fn default() -> Self {
        Self::new()
    }
}

impl PrinterObject for PrinterPins {
    /// Never called through the API: [`PrinterObject::is_queryable`] is false,
    /// so `objects/list` leaves `pins` out and `objects/query` answers `{}` for
    /// it. Returning an empty object matches upstream's
    /// `not hasattr(po, 'get_status')` path.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// A chip that records the pins it was asked to build.
    #[derive(Default)]
    struct TestChip {
        seen: Mutex<Vec<PinParams>>,
        created: Mutex<Vec<Arc<FakeDigitalOut>>>,
    }

    impl PinChip for TestChip {
        fn setup_digital_out(&self, params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            self.seen
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(params.clone());
            let out = Arc::new(FakeDigitalOut::default());
            self.created
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(Arc::clone(&out));
            Ok(out)
        }
    }

    /// A digital output that records what it was told to do.
    #[derive(Default)]
    struct FakeDigitalOut {
        updates: Mutex<Vec<bool>>,
        queued: Mutex<Vec<(u32, bool)>>,
    }

    impl DigitalOut for FakeDigitalOut {
        fn setup_max_duration(&self, _max_duration: f64) {}
        fn setup_start_value(&self, _start_value: bool, _shutdown_value: bool) {}
        fn queue_digital_out(&self, clock: u32, value: bool) -> Result<(), McuError> {
            self.queued
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push((clock, value));
            Ok(())
        }
        fn update_digital_out(&self, value: bool) -> Result<(), McuError> {
            self.updates
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(value);
            Ok(())
        }
    }

    /// A registry with one chip called `mcu` and one called `zboard`.
    fn pins() -> PrinterPins {
        let pins = PrinterPins::new();
        pins.register_chip("mcu", Arc::new(TestChip::default()))
            .unwrap();
        pins.register_chip("zboard", Arc::new(TestChip::default()))
            .unwrap();
        pins
    }

    // -----------------------------------------------------------------------
    // parse_pin
    // -----------------------------------------------------------------------

    #[test]
    fn test_a_bare_name_is_on_the_main_mcu() {
        let params = pins().parse_pin("PA1", false, false).unwrap();

        assert_eq!(
            params,
            PinParams {
                chip_name: "mcu".to_string(),
                pin: "PA1".to_string(),
                invert: false,
                pullup: 0,
                share_type: None,
            }
        );
    }

    #[test]
    fn test_a_chip_prefix_selects_the_mcu() {
        let params = pins().parse_pin("zboard:PA0", false, false).unwrap();

        assert_eq!(params.chip_name, "zboard");
        assert_eq!(params.pin, "PA0");
    }

    #[test]
    fn test_invert_and_pullup_are_read_only_when_allowed() {
        let pins = pins();

        let endstop = pins.parse_pin("^!PA1", true, true).unwrap();
        assert!(endstop.invert);
        assert_eq!(endstop.pullup, 1);

        let pulldown = pins.parse_pin("~PB2", true, true).unwrap();
        assert_eq!(pulldown.pullup, -1);

        // For a plain output the decorations are not part of the grammar, so
        // the name itself is rejected as malformed.
        let err = pins.parse_pin("!PA1", false, false).unwrap_err();
        assert!(matches!(err, PinError::InvalidDescription { .. }), "{err}");
    }

    #[test]
    fn test_an_unknown_chip_is_rejected() {
        let err = pins().parse_pin("nope:PA1", false, false).unwrap_err();

        assert_eq!(err, PinError::UnknownChip("nope".to_string()));
        assert_eq!(err.to_string(), "Unknown pin chip name 'nope'");
    }

    #[test]
    fn test_a_malformed_description_names_the_accepted_format() {
        let err = pins().parse_pin("PA 1", true, true).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Invalid pin description 'PA 1'\nFormat is: [^~] [!] [chip_name:] pin_name"
        );
    }

    #[test]
    fn test_a_duplicate_chip_is_rejected() {
        let pins = pins();
        assert_eq!(
            pins.register_chip("mcu", Arc::new(TestChip::default()))
                .unwrap_err(),
            PinError::DuplicateChip("mcu".to_string())
        );
    }

    #[test]
    fn test_setup_digital_out_validates_then_asks_the_chip() {
        let pins = PrinterPins::new();
        let chip = Arc::new(TestChip::default());
        pins.register_chip("mcu", chip.clone()).unwrap();

        let out = pins.setup_digital_out("!PA1", None).unwrap();
        out.update_digital_out(true).unwrap();

        // The chip was handed the parsed parameters. A digital output may be
        // inverted but not pulled, so `^` would be rejected.
        let seen = chip.seen.lock().unwrap();
        assert_eq!(seen.len(), 1);
        assert_eq!(seen[0].pin, "PA1");
        assert!(seen[0].invert);
        assert_eq!(seen[0].pullup, 0);
        drop(seen);

        // And the resource it returned is the one the caller drives.
        let created = chip.created.lock().unwrap();
        assert_eq!(*created[0].updates.lock().unwrap(), [true]);
    }

    #[test]
    fn test_setup_digital_out_reports_an_unknown_chip() {
        let pins = PrinterPins::new();
        let err = match pins.setup_digital_out("nope:PA1", None) {
            Ok(_) => panic!("an unknown chip was accepted"),
            Err(err) => err,
        };
        assert_eq!(err, PinError::UnknownChip("nope".to_string()));
    }

    // -----------------------------------------------------------------------
    // lookup_pin
    // -----------------------------------------------------------------------

    #[test]
    fn test_a_pin_cannot_be_used_twice() {
        let pins = pins();
        pins.lookup_pin("PA1", false, false, None).unwrap();

        let err = pins.lookup_pin("PA1", false, false, None).unwrap_err();

        assert_eq!(err, PinError::UsedMultipleTimes("PA1".to_string()));
    }

    #[test]
    fn test_a_shared_pin_must_use_the_same_share_type() {
        let pins = pins();
        let first = pins.lookup_pin("PA1", false, false, Some("spi")).unwrap();

        // Same share type: fine, and the first lookup's parameters come back.
        let second = pins.lookup_pin("PA1", false, false, Some("spi")).unwrap();
        assert_eq!(first, second);

        // A different share type, or none at all, is a second owner.
        assert_eq!(
            pins.lookup_pin("PA1", false, false, Some("other"))
                .unwrap_err(),
            PinError::UsedMultipleTimes("PA1".to_string())
        );
        assert_eq!(
            pins.lookup_pin("PA1", false, false, None).unwrap_err(),
            PinError::UsedMultipleTimes("PA1".to_string())
        );
    }

    #[test]
    fn test_a_shared_pin_must_keep_its_polarity() {
        let pins = pins();
        pins.lookup_pin("!PA1", true, false, Some("spi")).unwrap();

        let err = pins
            .lookup_pin("PA1", true, false, Some("spi"))
            .unwrap_err();
        assert_eq!(err, PinError::PolarityMismatch("PA1".to_string()));
    }

    #[test]
    fn test_multi_use_pins_may_be_used_by_many_owners() {
        let pins = pins();
        pins.allow_multi_use_pin("PA1").unwrap();

        pins.lookup_pin("PA1", false, false, None).unwrap();
        pins.lookup_pin("PA1", false, false, None).unwrap();
    }

    #[test]
    fn test_reset_pin_sharing_frees_the_pin() {
        let pins = pins();
        let params = pins.lookup_pin("PA1", false, false, None).unwrap();
        pins.reset_pin_sharing(&params);

        pins.lookup_pin("PA1", false, false, None).unwrap();
    }

    // -----------------------------------------------------------------------
    // PinResolver
    // -----------------------------------------------------------------------

    #[test]
    fn test_an_alias_resolves_to_its_pin() {
        let pins = pins();
        pins.alias_pin("mcu", "x_endstop", "PA1").unwrap();

        assert_eq!(pins.resolve_pin("mcu", "x_endstop").unwrap(), "PA1");
        // Resolving the canonical name *after* the alias is the "used under two
        // names" case, covered below.
    }

    #[test]
    fn test_aliases_chain_to_the_eventual_pin() {
        let pins = pins();
        pins.alias_pin("mcu", "first", "PA1").unwrap();
        // An alias may point at another alias; it resolves through to the pin.
        pins.alias_pin("mcu", "second", "first").unwrap();

        assert_eq!(pins.resolve_pin("mcu", "second").unwrap(), "PA1");
    }

    #[test]
    fn test_an_alias_cannot_be_remapped() {
        let pins = pins();
        pins.alias_pin("mcu", "x", "PA1").unwrap();

        let err = pins.alias_pin("mcu", "x", "PA2").unwrap_err();

        assert_eq!(
            err,
            PinError::AliasConflict {
                alias: "x".to_string(),
                existing: "PA1".to_string(),
                requested: "PA2".to_string(),
            }
        );
    }

    #[test]
    fn test_an_alias_target_must_be_a_bare_name() {
        let pins = pins();

        let err = pins.alias_pin("mcu", "x", "!PA1").unwrap_err();
        assert_eq!(err, PinError::InvalidAlias("!PA1".to_string()));
    }

    #[test]
    fn test_a_reserved_pin_is_refused_when_resolved() {
        let pins = pins();
        pins.reserve_pin("mcu", "PA1", "uart").unwrap();

        let err = pins.resolve_pin("mcu", "PA1").unwrap_err();

        assert_eq!(
            err,
            PinError::Reserved {
                pin: "PA1".to_string(),
                reserved_for: "uart".to_string(),
            }
        );
    }

    #[test]
    fn test_reserving_the_same_pin_twice_for_different_purposes_fails() {
        let pins = pins();
        pins.reserve_pin("mcu", "PA1", "uart").unwrap();
        pins.reserve_pin("mcu", "PA1", "uart").unwrap();

        let err = pins.reserve_pin("mcu", "PA1", "spi").unwrap_err();
        assert!(matches!(err, PinError::ReserveConflict { .. }), "{err}");
    }

    #[test]
    fn test_one_pin_under_two_names_is_an_alias_error() {
        let pins = pins();
        pins.alias_pin("mcu", "x_endstop", "PA1").unwrap();

        // Both names resolve to PA1; using both is a config mistake upstream
        // reports as "an alias for".
        pins.resolve_pin("mcu", "x_endstop").unwrap();
        let err = pins.resolve_pin("mcu", "PA1").unwrap_err();

        assert_eq!(
            err,
            PinError::IsAlias {
                name: "PA1".to_string(),
                canonical: "x_endstop".to_string(),
            }
        );
    }

    #[test]
    fn test_resolvers_are_per_chip() {
        let pins = pins();
        pins.reserve_pin("zboard", "PA1", "uart").unwrap();

        // The same name on another chip is a different pin.
        assert_eq!(pins.resolve_pin("mcu", "PA1").unwrap(), "PA1");
        assert!(pins.resolve_pin("zboard", "PA1").is_err());
        assert_eq!(
            pins.resolve_pin("nope", "PA1").unwrap_err(),
            PinError::UnknownChip("nope".to_string())
        );
    }

    #[test]
    fn test_the_object_is_registered_but_not_queryable() {
        let pins = pins();
        assert_eq!(pins.get_status(0.0), json!({}));
        assert!(!pins.is_queryable());
    }
}
