//! Common helper code for the TMC stepper drivers (upstream's
//! `klippy/extras/tmc.py` plus the current helper it borrows from
//! `tmc2130.py`).
//!
//! | upstream | here |
//! |---|---|
//! | `ffs` | [`ffs`] |
//! | `FieldHelper` | [`FieldHelper`] |
//! | `TMCCommandHelper` + `TMCCurrentHelper` + `TMCErrorCheck` | [`TmcDriver`] |
//! | `TMCVirtualPinHelper` | [`TmcVirtualPin`] (chip) + [`TmcVirtualEndstop`] |
//! | `TMCMicrostepHelper` / `TMCStealthchopHelper` / `TMCVcoolthrsHelper` / `TMCtstepHelper` | the free functions below |
//!
//! # One concrete driver type
//!
//! Upstream's per-chip classes (`TMC2208`, `TMC2209`, …) each end up answering
//! the same handful of methods (`get_phase_offset`, `get_status`, the
//! `SET_TMC_*` commands). Here every driver registers one concrete type,
//! [`TmcDriver`], so a consumer that only knows the upstream protocol — this
//! host's `[endstop_phase]`, which downcasts the `"<driver> <stepper>"` object
//! and calls `get_phase_offset()` — reaches it without naming a chip module.
//!
//! # Deliberate deviation: nothing is sent under `fileoutput`
//!
//! Upstream still emits one `spi_send` per register write in its file-output
//! ("debug") mode, and answers every register read with 0. Here the UART
//! transport ([`crate::core::klippy::extras::tmc_uart`]) answers reads with 0
//! and *drops* writes outright, so no `tmcuart_send` frame is produced at all.
//! That is what keeps a corpus run from blocking: the fake firmware never
//! answers `tmcuart_response` (or `spi_transfer_response`), so any real
//! exchange would hang. The driver-side cache (`fields.registers`) still tracks
//! every write, so `DUMP_TMC` reports what was configured.
//!
//! The UART transport is likewise not wired for a live MCU in this unit: a
//! non-file-output read or write reports
//! [`McuError::Config`](crate::core::klippy::mcu::McuError::Config) rather than
//! silently doing nothing. Only the `tmc2208`/`tmc2209` UART drivers are built
//! here; the SPI (2130/5160/2240) and 2660 transports are left to a later unit.

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::extruder::PrinterExtruder;
use crate::core::klippy::extras::stepper::PrinterStepper;
use crate::core::klippy::extras::toolhead::{
    EndstopFuture, HomingEndstop, QueryEndstopFuture, ToolHeadObject,
};
use crate::core::klippy::gcode::{
    parse_float, sync, CommandError, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::mcu::{Completion, McuError};
use crate::core::klippy::pins::{
    DigitalOut, PinChip, PinError, PinParams, PrinterPins, PINS_OBJECT,
};
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};

/// The toolhead object's registered name (upstream `lookup_object('toolhead')`).
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The maximum driver current (`tmc2130.MAX_CURRENT`), in amps.
pub const MAX_CURRENT: f64 = 2.000;

// ===========================================================================
// Field helpers
// ===========================================================================

/// The position of the first set bit in `mask` (`tmc.ffs`).
///
/// Upstream `(mask & -mask).bit_length() - 1`; a zero mask has no first bit and
/// never occurs (every field carries at least one bit).
pub fn ffs(mask: u32) -> u32 {
    (mask & mask.wrapping_neg()).trailing_zeros()
}

/// `int.bit_length()` for the signed-field sign extension.
fn bit_length(value: i64) -> u32 {
    if value == 0 {
        0
    } else {
        u64::BITS - (value as u64).leading_zeros()
    }
}

/// The write-through register cache, in insertion order (`FieldHelper.registers`).
#[derive(Default)]
struct Registers {
    order: Vec<String>,
    values: HashMap<String, u32>,
}

impl Registers {
    fn insert(&mut self, name: &str, value: u32) {
        if !self.values.contains_key(name) {
            self.order.push(name.to_string());
        }
        self.values.insert(name.to_string(), value);
    }

    fn get(&self, name: &str) -> Option<u32> {
        self.values.get(name).copied()
    }

    fn ordered(&self) -> Vec<(String, u32)> {
        self.order
            .iter()
            .filter_map(|name| self.values.get(name).map(|value| (name.clone(), *value)))
            .collect()
    }
}

/// One chip's register/field layout, and the values written to it
/// (`tmc.FieldHelper`).
///
/// All the layout tables are immutable after construction; the mutated part is
/// the register cache, behind a [`Mutex`] so the helper is `Send + Sync` and can
/// be shared by the driver, the current helper and the virtual-pin helper.
pub struct FieldHelper {
    /// Register name → (`field name` → bit mask).
    all_fields: HashMap<String, HashMap<String, u32>>,
    /// Fields whose value is two's-complement signed in the register.
    signed_fields: HashSet<String>,
    /// Field name → formatter for `DUMP_TMC`/logging.
    field_formatters: HashMap<String, fn(i64) -> String>,
    /// Field name → the register it lives in.
    field_to_register: HashMap<String, String>,
    /// The values written so far (`registers`).
    registers: Mutex<Registers>,
}

impl FieldHelper {
    /// Build the layout from a chip's tables (`FieldHelper.__init__`).
    pub fn new(
        all_fields: HashMap<String, HashMap<String, u32>>,
        signed_fields: &[&str],
        field_formatters: HashMap<String, fn(i64) -> String>,
    ) -> Self {
        let field_to_register = all_fields
            .iter()
            .flat_map(|(register, fields)| {
                fields
                    .keys()
                    .map(move |field| (field.clone(), register.clone()))
            })
            .collect();
        Self {
            all_fields,
            signed_fields: signed_fields.iter().map(|f| (*f).to_string()).collect(),
            field_formatters,
            field_to_register,
            registers: Mutex::new(Registers::default()),
        }
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Registers> {
        self.registers
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The register `field_name` lives in (`lookup_register`).
    pub fn lookup_register(&self, field_name: &str) -> Option<&str> {
        self.field_to_register.get(field_name).map(String::as_str)
    }

    /// The bit mask of `field_name` in `reg_name`, when both are known.
    pub fn field_mask(&self, reg_name: &str, field_name: &str) -> Option<u32> {
        self.all_fields.get(reg_name)?.get(field_name).copied()
    }

    /// Every field of `reg_name` (used by the error check to build its masks).
    pub fn register_fields(&self, reg_name: &str) -> Option<&HashMap<String, u32>> {
        self.all_fields.get(reg_name)
    }

    /// A register's cached value, or `None` when it was never written
    /// (`fields.registers.get`).
    pub fn register_value(&self, reg_name: &str) -> Option<u32> {
        self.lock().get(reg_name)
    }

    /// Every cached register, in insertion order (`fields.registers.items()`).
    pub fn register_values(&self) -> Vec<(String, u32)> {
        self.lock().ordered()
    }

    /// The field's value, read from the cache when `reg_value` is not supplied
    /// (`FieldHelper.get_field`).
    ///
    /// A register that was never written reads as 0, as upstream's
    /// `self.registers.get(reg_name, 0)` does.
    pub fn get_field(
        &self,
        field_name: &str,
        reg_value: Option<u32>,
        reg_name: Option<&str>,
    ) -> i64 {
        let reg_name = match reg_name {
            Some(name) => name.to_string(),
            None => self
                .field_to_register
                .get(field_name)
                .cloned()
                .unwrap_or_else(|| panic!("unknown tmc field '{field_name}'")),
        };
        let reg_value = reg_value.unwrap_or_else(|| self.lock().get(&reg_name).unwrap_or(0));
        let mask = *self
            .all_fields
            .get(&reg_name)
            .and_then(|fields| fields.get(field_name))
            .unwrap_or_else(|| panic!("unknown tmc field '{field_name}' in register '{reg_name}'"));
        let mut field_value = ((reg_value & mask) >> ffs(mask)) as i64;
        if self.signed_fields.contains(field_name) && ((reg_value & mask) as i64) << 1 > mask as i64
        {
            field_value -= 1i64 << bit_length(field_value);
        }
        field_value
    }

    /// Store `field_value` in `field_name`'s bits and return the new register
    /// value (`FieldHelper.set_field`).
    pub fn set_field(
        &self,
        field_name: &str,
        field_value: i64,
        reg_value: Option<u32>,
        reg_name: Option<&str>,
    ) -> u32 {
        let reg_name = match reg_name {
            Some(name) => name.to_string(),
            None => self
                .field_to_register
                .get(field_name)
                .cloned()
                .unwrap_or_else(|| panic!("unknown tmc field '{field_name}'")),
        };
        let reg_value = reg_value.unwrap_or_else(|| self.lock().get(&reg_name).unwrap_or(0));
        let mask = *self
            .all_fields
            .get(&reg_name)
            .and_then(|fields| fields.get(field_name))
            .unwrap_or_else(|| panic!("unknown tmc field '{field_name}' in register '{reg_name}'"));
        let new_value = (reg_value & !mask) | (((field_value as u32) << ffs(mask)) & mask);
        self.lock().insert(&reg_name, new_value);
        new_value
    }

    /// Read `driver_<FIELD>` and store it (`FieldHelper.set_config_field`).
    ///
    /// A one-bit field reads as a boolean; a signed field is bounded
    /// symmetrically around zero; anything else is bounded `0..=mask>>ffs`.
    ///
    /// # Errors
    /// Propagates the option's parse/bound error.
    pub fn set_config_field(
        &self,
        config: &ConfigWrapper,
        field_name: &str,
        default: i64,
    ) -> Result<(), ConfigError> {
        let config_name = format!("driver_{}", field_name.to_uppercase());
        let reg_name = self
            .field_to_register
            .get(field_name)
            .cloned()
            .ok_or_else(|| ConfigError::new(format!("unknown tmc field '{field_name}'")))?;
        let mask = self.all_fields[&reg_name][field_name];
        let maxval = (mask >> ffs(mask)) as i64;
        let value = if maxval == 1 {
            i64::from(config.get_bool(&config_name, Some(default != 0))?)
        } else if self.signed_fields.contains(field_name) {
            config.get_int_bounded(
                &config_name,
                Some(default),
                Some(-(maxval / 2 + 1)),
                Some(maxval / 2),
            )?
        } else {
            config.get_int_bounded(&config_name, Some(default), Some(0), Some(maxval))?
        };
        self.set_field(field_name, value, None, None);
        Ok(())
    }

    /// A human-readable rendering of a register value (`FieldHelper.pretty_format`).
    pub fn pretty_format(&self, reg_name: &str, reg_value: u32) -> String {
        let empty = HashMap::new();
        let reg_fields = self.all_fields.get(reg_name).unwrap_or(&empty);
        let mut reg_fields: Vec<(u32, &String)> = reg_fields
            .iter()
            .map(|(name, mask)| (*mask, name))
            .collect();
        reg_fields.sort();
        let mut out = format!("{:<11} {:08x}", format!("{reg_name}:"), reg_value);
        for (_, field_name) in reg_fields {
            let field_value = self.get_field(field_name, Some(reg_value), Some(reg_name));
            let sval = match self.field_formatters.get(field_name) {
                Some(formatter) => formatter(field_value),
                None => field_value.to_string(),
            };
            if !sval.is_empty() && sval != "0" {
                out.push_str(&format!(" {field_name}={sval}"));
            }
        }
        out
    }
}

// ===========================================================================
// Transport
// ===========================================================================

/// One register read's result (`get_register_raw`).
#[derive(Debug, Clone, Copy)]
pub struct TmcRegister {
    /// The register's value (`data`); 0 under file output.
    pub data: u32,
    /// The print-time the reading was taken (`#receive_time`); 0 for a cache.
    pub receive_time: f64,
}

/// How a driver reaches its chip (upstream's `MCU_TMC_SPI_chain` /
/// `MCU_TMC_uart`).
///
/// The methods map a register *name* to its bus address through the chip's own
/// table; the driver never sees addresses. Under file output a read answers 0
/// and a write is dropped (see the module docs).
pub trait TmcTransport: Send + Sync {
    /// Read a register, with the read time (`get_register_raw`).
    ///
    /// # Errors
    /// The bus error, or the "not wired outside file output" refusal.
    fn get_register_raw(&self, reg_name: &str) -> Result<TmcRegister, McuError>;

    /// Read a register's value (`get_register`).
    ///
    /// # Errors
    /// As [`TmcTransport::get_register_raw`].
    fn get_register(&self, reg_name: &str) -> Result<u32, McuError> {
        Ok(self.get_register_raw(reg_name)?.data)
    }

    /// Write a register (`set_register`).
    ///
    /// # Errors
    /// The bus error, or the "not wired outside file output" refusal.
    fn set_register(
        &self,
        reg_name: &str,
        val: u32,
        print_time: Option<f64>,
    ) -> Result<(), McuError>;

    /// The chip's internal TSTEP frequency, when it has one
    /// (`get_tmc_frequency`).
    fn get_tmc_frequency(&self) -> Option<f64> {
        None
    }

    /// The chip's register-name → address table.
    fn name_to_reg(&self) -> &HashMap<String, u8>;
}

// ===========================================================================
// Current helper (`tmc2130.TMCCurrentHelper`)
// ===========================================================================

/// The TMC current model shared by `tmc2208`/`tmc2209`
/// (`tmc2130.TMCCurrentHelper`).
///
/// `run_current` is required; `hold_current` defaults to [`MAX_CURRENT`] and
/// `sense_resistor` to 0.110 Ω. The constructor seeds `vsense`/`irun`/`ihold`,
/// so the register cache the driver writes at connect already carries a
/// non-zero `ihold` (which the periodic error check needs, see
/// [`TmcErrorCheck`]).
pub struct TmcCurrent {
    fields: Arc<FieldHelper>,
    transport: Arc<dyn TmcTransport>,
    sense_resistor: f64,
    req_hold_current: Mutex<f64>,
}

impl TmcCurrent {
    /// Read the current options and seed the registers.
    ///
    /// # Errors
    /// A missing/invalid `run_current`, `hold_current` or `sense_resistor`.
    pub fn new(
        config: &ConfigWrapper,
        fields: Arc<FieldHelper>,
        transport: Arc<dyn TmcTransport>,
    ) -> Result<Self, ConfigError> {
        let run_current = config.get_float_bounded(
            "run_current",
            None,
            None,
            Some(MAX_CURRENT),
            Some(0.),
            None,
        )?;
        let hold_current = config.get_float_bounded(
            "hold_current",
            Some(MAX_CURRENT),
            None,
            Some(MAX_CURRENT),
            Some(0.),
            None,
        )?;
        let sense_resistor =
            config.get_float_bounded("sense_resistor", Some(0.110), None, None, Some(0.), None)?;
        let helper = Self {
            fields,
            transport,
            sense_resistor,
            req_hold_current: Mutex::new(hold_current),
        };
        let (vsense, irun, ihold) = helper.calc_current(run_current, hold_current);
        helper.fields.set_field("vsense", vsense as i64, None, None);
        helper.fields.set_field("ihold", ihold, None, None);
        helper.fields.set_field("irun", irun, None, None);
        Ok(helper)
    }

    fn calc_current_bits(&self, current: f64, vsense: bool) -> i64 {
        let sense_resistor = self.sense_resistor + 0.020;
        let vref = if vsense { 0.18 } else { 0.32 };
        let cs =
            (32. * sense_resistor * current * std::f64::consts::SQRT_2 / vref + 0.5) as i64 - 1;
        cs.clamp(0, 31)
    }

    fn calc_current_from_bits(&self, cs: i64, vsense: bool) -> f64 {
        let sense_resistor = self.sense_resistor + 0.020;
        let vref = if vsense { 0.18 } else { 0.32 };
        (cs + 1) as f64 * vref / (32. * sense_resistor * std::f64::consts::SQRT_2)
    }

    fn calc_current(&self, run_current: f64, hold_current: f64) -> (bool, i64, i64) {
        let mut vsense = true;
        let mut irun = self.calc_current_bits(run_current, true);
        if irun == 31 {
            let cur = self.calc_current_from_bits(irun, true);
            if cur < run_current {
                let irun2 = self.calc_current_bits(run_current, false);
                let cur2 = self.calc_current_from_bits(irun2, false);
                if (run_current - cur2).abs() < (run_current - cur).abs() {
                    vsense = false;
                    irun = irun2;
                }
            }
        }
        let ihold = self.calc_current_bits(hold_current.min(run_current), vsense);
        (vsense, irun, ihold)
    }

    /// `(run_current, hold_current, requested_hold_current, max_current)`
    /// (`get_current`).
    pub fn get_current(&self) -> (f64, f64, f64, f64) {
        let irun = self.fields.get_field("irun", None, None);
        let ihold = self.fields.get_field("ihold", None, None);
        let vsense = self.fields.get_field("vsense", None, None);
        let run_current = self.calc_current_from_bits(irun, vsense != 0);
        let hold_current = self.calc_current_from_bits(ihold, vsense != 0);
        let req = *self
            .req_hold_current
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        (run_current, hold_current, req, MAX_CURRENT)
    }

    /// Set a new current pair (`set_current`).
    ///
    /// # Errors
    /// The transport write error (never under file output).
    pub fn set_current(
        &self,
        run_current: f64,
        hold_current: f64,
        print_time: Option<f64>,
    ) -> Result<(), McuError> {
        *self
            .req_hold_current
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = hold_current;
        let (vsense, irun, ihold) = self.calc_current(run_current, hold_current);
        if vsense != (self.fields.get_field("vsense", None, None) != 0) {
            let val = self.fields.set_field("vsense", vsense as i64, None, None);
            self.transport.set_register("CHOPCONF", val, print_time)?;
        }
        self.fields.set_field("ihold", ihold, None, None);
        let val = self.fields.set_field("irun", irun, None, None);
        self.transport.set_register("IHOLD_IRUN", val, print_time)
    }
}

// ===========================================================================
// Periodic error checking (`tmc.TMCErrorCheck`)
// ===========================================================================

/// The driver-fault check (upstream `TMCErrorCheck`).
///
/// Only the synchronous part is kept: [`TmcErrorCheck::check_once`] runs the
/// `DRV_STATUS` and `GSTAT` queries the periodic timer would. The timer itself
/// is not started here — nothing calls the enable path that would start it — so
/// a corpus run never leaves a timer behind. Under file output both reads
/// answer 0, so a check never reports a fault (and never shuts the printer down).
pub struct TmcErrorCheck {
    transport: Arc<dyn TmcTransport>,
    fields: Arc<FieldHelper>,
    stepper_name: String,
    drv_reg_name: String,
    drv_mask: u32,
    drv_err_mask: u32,
    gstat_reg: Option<String>,
    last_drv_status: Mutex<Option<u32>>,
    last_gstat: Mutex<Option<u32>>,
}

impl TmcErrorCheck {
    /// Build the register info (`TMCErrorCheck.__init__`).
    pub fn new(
        stepper_name: &str,
        fields: Arc<FieldHelper>,
        transport: Arc<dyn TmcTransport>,
    ) -> Self {
        let gstat_reg = fields
            .lookup_register("drv_err")
            .map(|name| name.to_string());
        let drv_reg_name = "DRV_STATUS".to_string();
        let mut mask = 0u32;
        let mut err_mask = 0u32;
        let err_fields = ["ot", "s2ga", "s2gb", "s2vsa", "s2vsb"];
        let warn_fields = ["otpw", "t120", "t143", "t150", "t157"];
        if let Some(fields_map) = fields.register_fields(&drv_reg_name) {
            for field in err_fields.iter().chain(warn_fields.iter()) {
                if let Some(bits) = fields_map.get(*field) {
                    mask |= bits;
                    if err_fields.contains(field) {
                        err_mask |= bits;
                    }
                }
            }
        }
        Self {
            transport,
            fields,
            stepper_name: stepper_name.to_string(),
            drv_reg_name,
            drv_mask: mask,
            drv_err_mask: err_mask,
            gstat_reg,
            last_drv_status: Mutex::new(None),
            last_gstat: Mutex::new(None),
        }
    }

    fn query_register(
        &self,
        reg_name: &str,
        mask: u32,
        err_mask: u32,
        last: &Mutex<Option<u32>>,
    ) -> Result<(), McuError> {
        let val = self.transport.get_register(reg_name)?;
        let mut last = last.lock().unwrap_or_else(|poison| poison.into_inner());
        if val & mask != last.unwrap_or(0) & mask {
            tracing::info!(
                "TMC '{}' reports {}",
                self.stepper_name,
                self.fields.pretty_format(reg_name, val)
            );
        }
        *last = Some(val);
        if val & err_mask == 0 {
            return Ok(());
        }
        Err(McuError::Config(format!(
            "TMC '{}' reports error: {}",
            self.stepper_name,
            self.fields.pretty_format(reg_name, val)
        )))
    }

    /// Run one full check (`_query_register` on both register infos).
    ///
    /// # Errors
    /// The read error, or a fault reported by the driver.
    pub fn check_once(&self) -> Result<(), McuError> {
        self.query_register(
            &self.drv_reg_name,
            self.drv_mask,
            self.drv_err_mask,
            &self.last_drv_status,
        )?;
        if let Some(gstat) = &self.gstat_reg {
            self.query_register(gstat, 0xffff_ffff, 0xffff_ffff, &self.last_gstat)?;
        }
        Ok(())
    }
}

// ===========================================================================
// Config-reading helpers
// ===========================================================================

/// The stepper/sub name a tmc section works on
/// (`' '.join(config.get_name().split()[1:])`).
pub fn config_stepper_name(config: &ConfigWrapper) -> String {
    let identifier = config.identifier();
    identifier
        .split_whitespace()
        .skip(1)
        .collect::<Vec<_>>()
        .join(" ")
}

/// `microsteps` → the `mres` code (`TMCMicrostepHelper`'s `steps` table).
const MICROSTEP_CHOICES: [(&str, i64); 9] = [
    ("256", 0),
    ("128", 1),
    ("64", 2),
    ("32", 3),
    ("16", 4),
    ("8", 5),
    ("4", 6),
    ("2", 7),
    ("1", 8),
];

/// Store the stepper's `microsteps`/`interpolate` in the driver
/// (`TMCMicrostepHelper`).
///
/// # Errors
/// A missing `[<stepper>]` section, or an unknown `microsteps` choice.
pub fn microstep_helper(config: &ConfigWrapper, fields: &FieldHelper) -> Result<(), ConfigError> {
    let stepper_name = config_stepper_name(config);
    let sibling = config.sibling(&stepper_name).ok_or_else(|| {
        ConfigError::new(format!(
            "Could not find config section '[{stepper_name}]' required by tmc driver"
        ))
    })?;
    let choices: Vec<&str> = MICROSTEP_CHOICES.iter().map(|(name, _)| *name).collect();
    let microsteps = sibling.get_choice("microsteps", &choices, None)?;
    let mres = MICROSTEP_CHOICES
        .iter()
        .find(|(name, _)| *name == microsteps)
        .map(|(_, code)| *code)
        .unwrap_or(0);
    fields.set_field("mres", mres, None, None);
    let intpol = i64::from(config.get_bool("interpolate", Some(true))?);
    fields.set_field("intpol", intpol, None, None);
    Ok(())
}

/// The attached stepper section's step distance
/// (`stepper.parse_step_distance` → `rotation_dist / steps_per_rotation`).
///
/// # Errors
/// A missing section, an unknown `microsteps`, or a malformed
/// `rotation_distance`/`gear_ratio`.
fn stepper_step_dist(config: &ConfigWrapper) -> Result<f64, ConfigError> {
    let stepper_name = config_stepper_name(config);
    let sibling = config.sibling(&stepper_name).ok_or_else(|| {
        ConfigError::new(format!(
            "Could not find config section '[{stepper_name}]' required by tmc driver"
        ))
    })?;
    let microsteps = sibling.get_int("microsteps", None)? as f64;
    let full_steps = sibling.get_int("full_steps_per_rotation", Some(200))? as f64;
    let gear_ratio = sibling
        .get_list("gear_ratio", ',')
        .map(|parts| {
            parts
                .iter()
                .filter_map(|part| part.parse::<f64>().ok())
                .product::<f64>()
        })
        .unwrap_or(1.0);
    let rotation_distance =
        sibling.get_float_bounded("rotation_distance", None, None, None, Some(0.), None)?;
    Ok(rotation_distance / (full_steps * microsteps * gear_ratio))
}

/// A TSTEP threshold for `velocity` (`TMCtstepHelper`), clamped to `0xfffff`.
///
/// The step distance comes from the looked-up stepper; when it is not yet known
/// (`pstepper` is `None`) the attached section is parsed instead.
///
/// # Errors
/// A malformed stepper section, when its distance has to be parsed.
pub fn tstep_helper(
    fields: &FieldHelper,
    transport: &dyn TmcTransport,
    velocity: f64,
    pstepper: Option<&Arc<PrinterStepper>>,
    config: Option<&ConfigWrapper>,
) -> Result<i64, ConfigError> {
    if velocity <= 0. {
        return Ok(0xfffff);
    }
    let step_dist = match pstepper {
        Some(stepper) => stepper.step_dist(),
        None => match config {
            Some(config) => stepper_step_dist(config)?,
            None => return Ok(0xfffff),
        },
    };
    let mres = fields.get_field("mres", None, None);
    let step_dist_256 = step_dist / ((1i64 << mres) as f64);
    let tmc_freq = transport.get_tmc_frequency().unwrap_or(0.);
    let threshold = (tmc_freq * step_dist_256 / velocity + 0.5) as i64;
    Ok(threshold.clamp(0, 0xfffff))
}

/// Store `stealthchop_threshold`'s `tpwmthrs`/`en_spreadcycle`/`en_pwm_mode`
/// (`TMCStealthchopHelper`).
///
/// # Errors
/// A malformed `stealthchop_threshold`, or a malformed stepper section.
pub fn stealthchop_helper(
    config: &ConfigWrapper,
    fields: &FieldHelper,
    transport: &dyn TmcTransport,
) -> Result<(), ConfigError> {
    let velocity = if config.has("stealthchop_threshold") {
        Some(config.get_float_bounded("stealthchop_threshold", None, Some(0.), None, None, None)?)
    } else {
        None
    };
    let mut en_pwm_mode = false;
    let mut tpwmthrs = 0xfffff;
    if let Some(velocity) = velocity {
        en_pwm_mode = true;
        tpwmthrs = tstep_helper(fields, transport, velocity, None, Some(config))?;
    }
    fields.set_field("tpwmthrs", tpwmthrs, None, None);
    if fields.lookup_register("en_pwm_mode").is_some() {
        fields.set_field("en_pwm_mode", i64::from(en_pwm_mode), None, None);
    } else {
        // TMC2208 uses en_spreadCycle.
        fields.set_field("en_spreadcycle", i64::from(!en_pwm_mode), None, None);
    }
    Ok(())
}

/// Store `coolstep_threshold`'s `tcoolthrs` (`TMCVcoolthrsHelper`).
///
/// # Errors
/// A malformed `coolstep_threshold`, or a malformed stepper section.
pub fn vcoolthrs_helper(
    config: &ConfigWrapper,
    fields: &FieldHelper,
    transport: &dyn TmcTransport,
) -> Result<(), ConfigError> {
    let velocity = if config.has("coolstep_threshold") {
        Some(config.get_float_bounded("coolstep_threshold", None, Some(0.), None, None, None)?)
    } else {
        None
    };
    let tcoolthrs = match velocity {
        Some(velocity) => tstep_helper(fields, transport, velocity, None, Some(config))?,
        None => 0,
    };
    fields.set_field("tcoolthrs", tcoolthrs, None, None);
    Ok(())
}

// ===========================================================================
// G-Code command helpers + the driver object
// ===========================================================================

/// Translates a read register into the name its fields are declared under
/// (upstream `read_translate`, e.g. TMC2208's `IOIN` split by `sel_a`).
pub type ReadTranslate = Box<dyn Fn(&str, u32) -> (String, u32) + Send + Sync>;

/// One configured TMC driver (`tmc2208`/`tmc2209`), and its command surface.
///
/// This is the object a `[tmc22xx <stepper>]` section registers, and the
/// concrete type `[endstop_phase]` downcasts to call
/// [`TmcDriver::get_phase_offset`]. It owns the field layout, the transport, the
/// current helper and the error check, and registers `SET_TMC_FIELD`,
/// `INIT_TMC`, `SET_TMC_CURRENT` and `DUMP_TMC`.
pub struct TmcDriver {
    printer: Weak<Printer>,
    /// The stepper name the section names (`stepper_x`, `extruder`).
    name: String,
    /// The full section sub (`stepper_x`), for logging and the error check.
    stepper_name: String,
    fields: Arc<FieldHelper>,
    transport: Arc<dyn TmcTransport>,
    current: Arc<TmcCurrent>,
    echeck: Arc<TmcErrorCheck>,
    read_registers: Vec<String>,
    read_translate: Option<ReadTranslate>,
    mcu_phase_offset: Mutex<Option<i64>>,
    stepper: Mutex<Option<Arc<PrinterStepper>>>,
    /// Previous field values the virtual-endstop homing stashed, for restore.
    virtual_prev_state: Mutex<Vec<(&'static str, i64)>>,
    /// Registers the virtual-endstop homing changed, awaiting a send.
    virtual_dirty: Mutex<Vec<(String, u32)>>,
}

impl TmcDriver {
    /// Assemble a driver around an already-seeded field layout.
    ///
    /// `fields` must already carry the driver's default register values (the
    /// chip module sets them before calling this). This runs the microstep
    /// helper, builds the error check and registers the command surface.
    ///
    /// # Errors
    /// A missing stepper section or a duplicate mux command.
    pub fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        fields: Arc<FieldHelper>,
        transport: Arc<dyn TmcTransport>,
        current: Arc<TmcCurrent>,
        read_registers: Vec<String>,
        read_translate: Option<ReadTranslate>,
    ) -> Result<Arc<Self>, ConfigError> {
        microstep_helper(config, &fields)?;
        let stepper_name = config_stepper_name(config);
        let name = stepper_name
            .split_whitespace()
            .next_back()
            .unwrap_or("")
            .to_string();
        let echeck = Arc::new(TmcErrorCheck::new(
            &stepper_name,
            Arc::clone(&fields),
            Arc::clone(&transport),
        ));
        let driver = Arc::new(Self {
            printer: Arc::downgrade(printer),
            name,
            stepper_name,
            fields,
            transport,
            current,
            echeck,
            read_registers,
            read_translate,
            mcu_phase_offset: Mutex::new(None),
            stepper: Mutex::new(None),
            virtual_prev_state: Mutex::new(Vec::new()),
            virtual_dirty: Mutex::new(Vec::new()),
        });
        driver.clone().register_commands(printer)?;
        Ok(driver)
    }

    fn register_commands(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .ok_or_else(|| ConfigError::new("the g-code dispatcher is not registered"))?;
        let name = self.name.clone();
        let register = |cmd: &str, handler, desc: &str| {
            gcode
                .register_mux_command(cmd, "STEPPER", Some(&name), handler, Some(desc))
                .map_err(|err| ConfigError::new(err))
        };
        let set_field = Arc::clone(self);
        register(
            "SET_TMC_FIELD",
            sync(move |gcmd| set_field.cmd_set_tmc_field(gcmd)),
            "Set a register field of a TMC driver",
        )?;
        let init = Arc::clone(self);
        register(
            "INIT_TMC",
            sync(move |gcmd| init.cmd_init_tmc(gcmd)),
            "Initialize TMC stepper driver registers",
        )?;
        let set_current = Arc::clone(self);
        register(
            "SET_TMC_CURRENT",
            sync(move |gcmd| set_current.cmd_set_tmc_current(gcmd)),
            "Set the current of a TMC driver",
        )?;
        let dump = Arc::clone(self);
        register(
            "DUMP_TMC",
            sync(move |gcmd| dump.cmd_dump_tmc(gcmd)),
            "Read and display TMC stepper driver registers",
        )?;
        Ok(())
    }

    /// The driver's default name (`config.get_name().split()[-1]`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The stepper name (`config.get_name().split()[1:]`).
    pub fn stepper_name(&self) -> &str {
        &self.stepper_name
    }

    /// The transport, for the virtual-pin helper.
    pub fn fields(&self) -> &Arc<FieldHelper> {
        &self.fields
    }

    /// The register cache in insertion order.
    pub fn registers(&self) -> Vec<(String, u32)> {
        self.fields.register_values()
    }

    /// The driver's transport, so a test can pin the file-output short-circuit
    /// and the register-name table.
    pub fn transport(&self) -> &Arc<dyn TmcTransport> {
        &self.transport
    }

    /// Send every cached register (`_init_registers`).
    ///
    /// # Errors
    /// The transport write error.
    pub fn init_registers(&self, print_time: Option<f64>) -> Result<(), McuError> {
        for (reg_name, val) in self.fields.register_values() {
            self.transport.set_register(&reg_name, val, print_time)?;
        }
        Ok(())
    }

    /// The driver's phase count and tracked offset
    /// (`get_phase_offset`): `(mcu_phase_offset, (256 >> mres) * 4)`.
    ///
    /// The offset stays `None` — this host runs no position-sync handler — but
    /// the method exists so `[endstop_phase]` can find it.
    pub fn get_phase_offset(&self) -> (Option<i64>, i64) {
        let offset = *self
            .mcu_phase_offset
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let phases = (256 >> self.fields.get_field("mres", None, None)) * 4;
        (offset, phases)
    }

    /// The error check, for a caller that wants to run it (the enable path
    /// would call this).
    pub fn error_check(&self) -> &TmcErrorCheck {
        &self.echeck
    }

    fn last_move_time(&self) -> f64 {
        self.printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT))
            .map(|toolhead| toolhead.get_last_move_time())
            .unwrap_or(0.)
    }

    fn cmd_init_tmc(&self, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
        tracing::info!("INIT_TMC {}", self.name);
        let print_time = self.last_move_time();
        self.init_registers(Some(print_time))
            .map_err(|err| CommandError::new(err.to_string()))
    }

    fn cmd_set_tmc_field(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let field_name = gcmd.get_str("FIELD")?.to_lowercase();
        let reg_name = self
            .fields
            .lookup_register(&field_name)
            .map(|name| name.to_string())
            .ok_or_else(|| CommandError::new(format!("Unknown field name '{field_name}'")))?;
        let params = gcmd.get_command_parameters();
        let value = if params.contains_key("VALUE") {
            Some(gcmd.get_int("VALUE")?)
        } else {
            None
        };
        let velocity = if params.contains_key("VELOCITY") {
            Some(gcmd.get("VELOCITY", None, parse_float, Some(0.), None, None, None)?)
        } else {
            None
        };
        if value.is_none() == velocity.is_none() {
            return Err(CommandError::new("Specify either VALUE or VELOCITY"));
        }
        let value = match velocity {
            Some(velocity) => {
                if self.transport.get_tmc_frequency().is_none() {
                    return Err(CommandError::new(
                        "VELOCITY parameter not supported by this driver",
                    ));
                }
                let stepper = self
                    .stepper
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner())
                    .clone();
                tstep_helper(
                    &self.fields,
                    self.transport.as_ref(),
                    velocity,
                    stepper.as_ref(),
                    None,
                )
                .map_err(|err| CommandError::new(err.to_string()))?
            }
            None => value.unwrap_or(0),
        };
        let reg_val = self.fields.set_field(&field_name, value, None, None);
        let print_time = self.last_move_time();
        self.transport
            .set_register(&reg_name, reg_val, Some(print_time))
            .map_err(|err| CommandError::new(err.to_string()))
    }

    fn cmd_set_tmc_current(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let (mut prev_cur, prev_hold_cur, mut req_hold_cur, max_cur) = self.current.get_current();
        let params = gcmd.get_command_parameters();
        let run_current = if params.contains_key("CURRENT") {
            Some(gcmd.get(
                "CURRENT",
                None,
                parse_float,
                Some(0.),
                Some(max_cur),
                None,
                None,
            )?)
        } else {
            None
        };
        let hold_current = if params.contains_key("HOLDCURRENT") {
            Some(gcmd.get(
                "HOLDCURRENT",
                None,
                parse_float,
                None,
                Some(max_cur),
                Some(0.),
                None,
            )?)
        } else {
            None
        };
        if run_current.is_some() || hold_current.is_some() {
            let run_current = run_current.unwrap_or(prev_cur);
            let hold_current = hold_current.unwrap_or(req_hold_cur);
            let print_time = self.last_move_time();
            self.current
                .set_current(run_current, hold_current, Some(print_time))
                .map_err(|err| CommandError::new(err.to_string()))?;
            (prev_cur, _, req_hold_cur, _) = self.current.get_current();
        }
        let _ = req_hold_cur;
        gcmd.respond_info(&format!(
            "Run Current: {prev_cur:0.2}A Hold Current: {prev_hold_cur:0.2}A"
        ));
        Ok(())
    }

    fn cmd_dump_tmc(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        tracing::info!("DUMP_TMC {}", self.name);
        let reg_name = gcmd.get_str("REGISTER").ok();
        match reg_name {
            Some(reg_name) => {
                let reg_name = reg_name.to_uppercase();
                let cached = self.fields.register_value(&reg_name);
                if let Some(val) = cached {
                    if !self.read_registers.contains(&reg_name) {
                        gcmd.respond_info(&self.fields.pretty_format(&reg_name, val));
                        return Ok(());
                    }
                }
                if self.read_registers.contains(&reg_name) {
                    let mut val = self
                        .transport
                        .get_register(&reg_name)
                        .map_err(|err| CommandError::new(err.to_string()))?;
                    let mut reg_name = reg_name;
                    if let Some(translate) = &self.read_translate {
                        let (name, translated) = translate(&reg_name, val);
                        reg_name = name;
                        val = translated;
                    }
                    gcmd.respond_info(&self.fields.pretty_format(&reg_name, val));
                } else {
                    return Err(CommandError::new(format!(
                        "Unknown register name '{reg_name}'"
                    )));
                }
            }
            None => {
                gcmd.respond_info("========== Write-only registers ==========");
                for (reg_name, val) in self.fields.register_values() {
                    if !self.read_registers.contains(&reg_name) {
                        gcmd.respond_info(&self.fields.pretty_format(&reg_name, val));
                    }
                }
                gcmd.respond_info("========== Queried registers ==========");
                for reg_name in &self.read_registers {
                    let mut val = self
                        .transport
                        .get_register(reg_name)
                        .map_err(|err| CommandError::new(err.to_string()))?;
                    let mut reg_name = reg_name.clone();
                    if let Some(translate) = &self.read_translate {
                        let (name, translated) = translate(&reg_name, val);
                        reg_name = name;
                        val = translated;
                    }
                    gcmd.respond_info(&self.fields.pretty_format(&reg_name, val));
                }
            }
        }
        Ok(())
    }

    // -- virtual endstop field juggling (`TMCVirtualPinHelper`) -------------

    fn set_virtual_field(&self, field_name: &'static str, value: i64) {
        {
            let prev = self.fields.get_field(field_name, None, None);
            let mut state = self
                .virtual_prev_state
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            state.push((field_name, prev));
        }
        let reg_name = self
            .fields
            .lookup_register(field_name)
            .map(|name| name.to_string())
            .unwrap_or_default();
        let val = self.fields.set_field(field_name, value, None, None);
        self.virtual_dirty
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .push((reg_name, val));
    }

    fn send_virtual_fields(&self) -> Result<(), McuError> {
        let mut dirty = self
            .virtual_dirty
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        for (reg, val) in dirty.drain(..) {
            self.transport.set_register(&reg, val, None)?;
        }
        Ok(())
    }

    /// Apply the stallguard/sg fields a sensorless home needs
    /// (`handle_homing_move_begin`).
    fn virtual_homing_begin(&self, diag_pin_field: Option<&'static str>) -> Result<(), McuError> {
        let sg4_thrs = if self.fields.lookup_register("sg4_thrs").is_some() {
            self.fields.get_field("sg4_thrs", None, None)
        } else {
            0
        };
        if self.fields.lookup_register("en_pwm_mode").is_none() {
            // On "stallguard4" drivers, "stealthchop" must be enabled.
            self.set_virtual_field("tpwmthrs", 0);
            self.set_virtual_field("en_spreadcycle", 0);
        } else if sg4_thrs != 0 {
            self.set_virtual_field("en_pwm_mode", 1);
            self.set_virtual_field("tpwmthrs", 0);
            if let Some(field) = diag_pin_field {
                self.set_virtual_field(field, 1);
            }
        } else {
            self.set_virtual_field("en_pwm_mode", 0);
            if let Some(field) = diag_pin_field {
                self.set_virtual_field(field, 1);
            }
        }
        if self.fields.get_field("tcoolthrs", None, None) == 0 {
            self.set_virtual_field("tcoolthrs", 0xfffff);
        }
        if self.fields.lookup_register("thigh").is_some() {
            self.set_virtual_field("thigh", 0);
        }
        self.send_virtual_fields()
    }

    /// Restore the fields a sensorless home changed (`handle_homing_move_end`).
    fn virtual_homing_end(&self) -> Result<(), McuError> {
        let prev: Vec<(&'static str, i64)> = {
            let mut state = self
                .virtual_prev_state
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            std::mem::take(&mut *state)
        };
        for (field, val) in prev {
            self.set_virtual_field(field, val);
        }
        self.send_virtual_fields()
    }

    /// Look up the stepper this driver configures (`_handle_mcu_identify`).
    ///
    /// A `[stepper_*]` section is loaded `late`, so this cannot run at load;
    /// the `connect` phase ([`TmcDriver::connect`]) is the first point the
    /// object exists. `[extruder]` registers its stepper through the extruder
    /// object instead of a `[stepper_*]` section.
    fn lookup_stepper(&self) -> Option<Arc<PrinterStepper>> {
        let printer = self.printer.upgrade()?;
        if let Some(stepper) = printer.lookup_object_as::<PrinterStepper>(&self.stepper_name) {
            return Some(stepper);
        }
        printer
            .lookup_object_as::<PrinterExtruder>(&self.stepper_name)
            .and_then(|extruder| extruder.printer_stepper().cloned())
    }

    fn handle_connect(&self) {
        if let Some(stepper) = self.lookup_stepper() {
            *self
                .stepper
                .lock()
                .unwrap_or_else(|poison| poison.into_inner()) = Some(stepper);
        }
        if let Err(err) = self.init_registers(None) {
            tracing::info!("TMC {} failed to init: {}", self.name, err);
        }
    }
}

impl PrinterObject for TmcDriver {
    fn get_status(&self, _eventtime: f64) -> Value {
        let (run_current, hold_current, _, _) = self.current.get_current();
        let offset = *self
            .mcu_phase_offset
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        json!({
            "mcu_phase_offset": offset,
            "phase_offset_position": Value::Null,
            "run_current": run_current,
            "hold_current": hold_current,
        })
    }

    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            self.handle_connect();
            Ok(())
        })
    }
}

// ===========================================================================
// TMC virtual pins (`TMCVirtualPinHelper`)
// ===========================================================================

/// The `[tmc22xx <stepper>]` chip that answers `virtual_endstop`
/// (`TMCVirtualPinHelper` + `PinChip`).
///
/// It reads the diag-pin options at load and registers a chip named
/// `<driver>_<stepper>`; when a rail resolves
/// `<driver>_<stepper>:virtual_endstop`, [`TmcVirtualPin::setup_endstop_dyn`]
/// builds the real MCU endstop and wraps it in [`TmcVirtualEndstop`], which
/// applies and restores the stallguard fields around the move.
pub struct TmcVirtualPin {
    printer: Weak<Printer>,
    driver: Arc<TmcDriver>,
    diag_pin: Option<String>,
    diag_pin_field: Option<&'static str>,
}

impl TmcVirtualPin {
    /// Read the diag options and register the chip (`TMCVirtualPinHelper.__init__`).
    ///
    /// On a driver that has `diag0_stall`/`diag1_stall` (the SPI chips) the pin
    /// comes from `diag0_pin`/`diag1_pin`; the UART chips only have `diag_pin`.
    ///
    /// # Errors
    /// A duplicate chip name.
    pub fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        driver: &Arc<TmcDriver>,
    ) -> Result<Arc<Self>, ConfigError> {
        let fields = driver.fields();
        let (diag_pin, diag_pin_field) = if fields.lookup_register("diag0_stall").is_some() {
            if config.get("diag0_pin", None).is_ok() {
                (config.get("diag0_pin", None).ok(), Some("diag0_stall"))
            } else {
                (config.get("diag1_pin", None).ok(), Some("diag1_stall"))
            }
        } else {
            (config.get("diag_pin", None).ok(), None)
        };
        let ppins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .ok_or_else(|| ConfigError::new("the pins object is not registered"))?;
        let chip = Arc::new(Self {
            printer: Arc::downgrade(printer),
            driver: Arc::clone(driver),
            diag_pin,
            diag_pin_field,
        });
        let chip_name = format!("{}_{}", section_driver(config), driver.name());
        ppins
            .register_chip(&chip_name, chip.clone())
            .map_err(|err| ConfigError::new(err.to_string()))?;
        Ok(chip)
    }
}

/// The chip's driver prefix (`[tmc2209 stepper_x]` → `tmc2209`).
fn section_driver(config: &ConfigWrapper) -> String {
    config
        .identifier()
        .split_whitespace()
        .next()
        .unwrap_or("")
        .to_string()
}

impl PinChip for TmcVirtualPin {
    fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
        Err(PinError::Unsupported("digital_out".to_string()))
    }

    fn setup_endstop_dyn(&self, params: &PinParams) -> Result<Arc<dyn HomingEndstop>, PinError> {
        if params.pin != "virtual_endstop" {
            return Err(PinError::Message(
                "tmc virtual endstop only useful as endstop".to_string(),
            ));
        }
        if params.invert || params.pullup != 0 {
            return Err(PinError::Message(
                "Can not pullup/invert tmc virtual pin".to_string(),
            ));
        }
        let Some(diag_pin) = &self.diag_pin else {
            return Err(PinError::Message(
                "tmc virtual endstop requires diag pin config".to_string(),
            ));
        };
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| PinError::Message("printer is gone".to_string()))?;
        let ppins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .ok_or_else(|| PinError::Message("the pins object is not registered".to_string()))?;
        let inner = ppins.setup_endstop_dyn(diag_pin, None)?;
        Ok(Arc::new(TmcVirtualEndstop {
            inner,
            driver: Arc::downgrade(&self.driver),
            diag_pin_field: self.diag_pin_field,
        }))
    }
}

/// A sensorless-homing endstop that arms and restores the driver's stallguard
/// fields around the move (`TMCVirtualPinHelper.handle_homing_move_begin/end`).
///
/// The begin/end are the decorator's own methods rather than the
/// `homing:homing_move_begin/end` events upstream uses, because this host's
/// homing driver calls the endstop it is handed directly; no toolhead or event
/// plumbing is touched.
///
/// The driver is held **weakly**: the rail's `PrinterStepper` owns this
/// endstop, and the driver strongly owns the stepper, so a strong reference
/// back would be a cycle that keeps the MCU (and its receive task) alive past
/// teardown.
pub struct TmcVirtualEndstop {
    inner: Arc<dyn HomingEndstop>,
    driver: Weak<TmcDriver>,
    diag_pin_field: Option<&'static str>,
}

impl HomingEndstop for TmcVirtualEndstop {
    fn home_start(
        &self,
        print_time: f64,
        sample_time: f64,
        sample_count: u8,
        rest_time: f64,
        triggered: bool,
    ) -> Result<Arc<Completion>, McuError> {
        if let Some(driver) = self.driver.upgrade() {
            driver.virtual_homing_begin(self.diag_pin_field)?;
        }
        self.inner
            .home_start(print_time, sample_time, sample_count, rest_time, triggered)
    }

    fn home_wait(&self, home_end_time: f64) -> EndstopFuture<'_> {
        let inner = Arc::clone(&self.inner);
        let driver = self.driver.clone();
        Box::pin(async move {
            let result = inner.home_wait(home_end_time).await;
            if let Some(driver) = driver.upgrade() {
                let _ = driver.virtual_homing_end();
            }
            result
        })
    }

    fn dispatch(&self) -> Option<&crate::core::klippy::mcu::TriggerDispatch> {
        self.inner.dispatch()
    }

    fn query_endstop(&self, print_time: f64) -> QueryEndstopFuture<'_> {
        self.inner.query_endstop(print_time)
    }
}
