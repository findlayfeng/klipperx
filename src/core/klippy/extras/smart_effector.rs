//! `[smart_effector]` — the Duet SmartEffector probe and its two commands.
//!
//! Upstream `klippy/extras/smart_effector.py`. The section *is* a `[probe]`
//! in disguise: upstream builds the probe family's helpers from the same
//! options, then adds the effector's own `control_pin` programming pin, the
//! `probe_accel` / `recovery_time` recovery knobs and `SET_SMART_EFFECTOR` /
//! `RESET_SMART_EFFECTOR`. Here the probe family is [`PrinterProbe`] itself,
//! built from this section's options: it reads the probe option set,
//! registers the `probe` virtual pin chip and lands `QUERY_PROBE` / `PROBE` /
//! `PROBE_ACCURACY` / `PROBE_CALIBRATE` exactly once (reusing them, not
//! registering them again), and `load_config` additionally registers the
//! built probe under the upstream name `probe`
//! (`add_object('probe', smart_effector)`), which is what
//! `ProbePointsHelper` (`BED_MESH_CALIBRATE` and friends) looks up.
//!
//! What rides along with the port's existing probe gaps (`probe.rs`, H9):
//! upstream wraps every probing move in `probe_prepare`/`probe_finish` to
//! apply `probe_accel` (an `M204` round trip) and dwell `recovery_time`; the
//! homing/probe move has no prepare/finish hooks here yet, so the two options
//! are read, stored, reported and reprogrammed by `SET_SMART_EFFECTOR`, but
//! not yet driven around a probing move.

use std::sync::{Arc, Mutex};

use serde_json::Value;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::probe::PrinterProbe;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    parse_float, CommandError, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::McuObject;
use crate::core::klippy::pins::{DigitalOut, PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("smart_effector", order = 30, load = load_config);

/// The object upstream's `load_config` registers the built probe under.
const PROBE_OBJECT: &str = "probe";

/// The toolhead object, as the loader registers `[printer]`.
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The SmartEffector programming rate (`smart_effector.py: BITS_PER_SECOND`).
const BITS_PER_SECOND: f64 = 1000.;

/// The `[smart_effector]` options on top of the probe family's
/// (`smart_effector.py:77-79`); the rest come from [`PrinterProbe`].
#[derive(Debug, Clone, PartialEq)]
pub struct SmartEffectorOptions {
    /// The effector's programming pin; without it `RESET_SMART_EFFECTOR`
    /// stays unregistered and `SENSITIVITY` programming is refused.
    pub control_pin: Option<String>,
    /// Probing acceleration to apply around a probe (`probe_accel`, `minval=0`).
    pub probe_accel: f64,
    /// Dwell after a probe before the next move (`recovery_time`, `minval=0`).
    pub recovery_time: f64,
}

impl SmartEffectorOptions {
    /// Read the effector's own options, so `check_unused` passes.
    ///
    /// # Errors
    /// As the option readers: a `probe_accel` / `recovery_time` that is not a
    /// number or below zero.
    pub fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        Ok(Self {
            control_pin: config.get_str("control_pin"),
            probe_accel: config.get_float_bounded(
                "probe_accel",
                Some(0.),
                Some(0.),
                None,
                None,
                None,
            )?,
            recovery_time: config.get_float_bounded(
                "recovery_time",
                Some(0.4),
                Some(0.),
                None,
                None,
                None,
            )?,
        })
    }
}

/// The programming pin and the clock its bits are timed against
/// (`smart_effector.py:ControlPinHelper`).
struct ControlPinHelper {
    /// The reserved digital output the bit stream is written to.
    out: Arc<dyn DigitalOut>,
    /// The pin's MCU, as its printer-object name (`mcu`, `mcu <name>`), for
    /// turning print times into firmware clocks at write time.
    mcu: String,
    /// The machine, to find the MCU and the toolhead when a command runs.
    printer: WeakPrinter,
}

/// `Weak<Printer>` under a name that says what it is for.
type WeakPrinter = std::sync::Weak<Printer>;

impl ControlPinHelper {
    /// Reserve `description` as the control pin and build its output.
    ///
    /// # Errors
    /// A config error when the description is malformed, names an unknown
    /// chip, or the pin is already in use — the same complaints
    /// `PrinterPins::setup_digital_out` raises for any digital output.
    fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        description: &str,
    ) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let out = pins
            .setup_digital_out(description, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        let params = pins
            .parse_pin(description, true, false)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        // Upstream's `config_digital_out … value=%d default_value=%d
        // max_duration=%d` with `value = default_value = start value` and
        // `max_duration=0`: idle at the logical-off level, no time limit.
        out.setup_start_value(false, false);
        out.setup_max_duration(0.);
        Ok(Self {
            out,
            mcu: mcu_object_name(&params.chip_name),
            printer: Arc::downgrade(printer),
        })
    }

    /// Write `bits` starting at `start_time`, 1 ms per bit
    /// (`ControlPinHelper.write_bits`): a write only where the level changes,
    /// then back to the start level after the last bit.
    ///
    /// # Errors
    /// "Printer is not ready" before the MCU's clock mapping exists, or
    /// whatever the firmware reports for a failed send.
    fn write_bits(&self, start_time: f64, bits: &[bool]) -> Result<f64, CommandError> {
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let mcu = printer
            .lookup_object_as::<McuObject>(&self.mcu)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let clock = |print_time: f64| {
            mcu.print_time_to_clock(print_time)
                .map(|clock| clock as u32)
                .ok_or_else(|| CommandError::new("Printer is not ready"))
        };

        let bit_step = 1. / BITS_PER_SECOND;
        let mut last_value = false;
        let mut bit_time = start_time;
        for &bit in bits {
            if bit != last_value {
                self.out
                    .queue_digital_out(clock(bit_time)?, bit)
                    .map_err(|err| CommandError::new(err.to_string()))?;
                last_value = bit;
            }
            bit_time += bit_step;
        }
        // After the last bit, the signal on the control pin must go back
        // to its start value.
        if last_value {
            self.out
                .queue_digital_out(clock(bit_time)?, false)
                .map_err(|err| CommandError::new(err.to_string()))?;
            bit_time += bit_step;
        }
        Ok(bit_time)
    }
}

/// The printer-object name of an MCU: `mcu`, or `mcu <name>`
/// (`klippy/mcu.py:1251`), which is how the pin's chip name maps back to the
/// object that owns its clock.
fn mcu_object_name(chip_name: &str) -> String {
    if chip_name == "mcu" {
        "mcu".to_string()
    } else {
        format!("mcu {chip_name}")
    }
}

impl std::fmt::Debug for ControlPinHelper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ControlPinHelper")
            .field("mcu", &self.mcu)
            .finish_non_exhaustive()
    }
}

/// The effector's command state: the two knobs `SET_SMART_EFFECTOR`
/// reprograms, and the programming pin.
struct Control {
    /// The current probing acceleration (`probe_accel`).
    probe_accel: f64,
    /// The current recovery dwell (`recovery_time`).
    recovery_time: f64,
    /// `None` without `control_pin` in `[smart_effector]`.
    pin: Option<Arc<ControlPinHelper>>,
}

/// Lock the control state, recovering from a poisoned lock (the values are
/// plain floats and a pin handle, so a poisoned lock carries no torn state
/// this code could not re-establish).
fn lock_control(control: &Mutex<Control>) -> std::sync::MutexGuard<'_, Control> {
    control.lock().unwrap_or_else(|poison| poison.into_inner())
}

/// One byte as the SmartEffector sees it —
/// `[0 0 1 0 b7 b6 b5 b4 !b4 b3 b2 b1 b0 !b0]`
/// (`smart_effector.py:cmd_SET_SMART_EFFECTOR`'s `_send_command`).
fn bit_stream(buf: &[u8]) -> Vec<bool> {
    let mut bits = Vec::with_capacity(buf.len() * 14);
    for &byte in buf {
        bits.extend([false, false, true, false]);
        bits.extend([
            byte & 0x80 != 0,
            byte & 0x40 != 0,
            byte & 0x20 != 0,
            byte & 0x10 != 0,
        ]);
        bits.push((!byte) & 0x10 != 0);
        bits.extend([
            byte & 0x08 != 0,
            byte & 0x04 != 0,
            byte & 0x02 != 0,
            byte & 0x01 != 0,
        ]);
        bits.push((!byte) & 0x01 != 0);
    }
    bits
}

/// `SET_SMART_EFFECTOR`'s parameters: optional `SENSITIVITY`
/// (`minval=0`, `maxval=255`), `ACCEL` and `RECOVERY_TIME` (both `minval=0`,
/// both defaulting to the current value) — `cmd_SET_SMART_EFFECTOR`.
///
/// # Errors
/// When a parameter is present but not a number, or out of range (upstream's
/// `Option '…' must have … of …`, reported with this dispatcher's line
/// context).
fn parse_set(
    gcmd: &GcodeCommand,
    probe_accel: f64,
    recovery_time: f64,
) -> Result<(Option<i64>, f64, f64), CommandError> {
    let sensitivity = if gcmd.get_command_parameters().contains_key("SENSITIVITY") {
        Some(gcmd.get_int_bounded("SENSITIVITY", Some(0), Some(255))?)
    } else {
        None
    };
    let probe_accel = gcmd.get::<f64>(
        "ACCEL",
        Some(probe_accel),
        parse_float,
        Some(0.),
        None,
        None,
        None,
    )?;
    let recovery_time = gcmd.get::<f64>(
        "RECOVERY_TIME",
        Some(recovery_time),
        parse_float,
        Some(0.),
        None,
        None,
        None,
    )?;
    Ok((sensitivity, probe_accel, recovery_time))
}

/// Refuse `SENSITIVITY` without a `control_pin`
/// (`cmd_SET_SMART_EFFECTOR`'s `control_pin must be set …`).
///
/// # Errors
/// The upstream message when sensitivity was asked for on a printer whose
/// `[smart_effector]` has no `control_pin`.
fn check_sensitivity_programming(
    control_pin_present: bool,
    sensitivity: Option<i64>,
) -> Result<(), CommandError> {
    if sensitivity.is_some() && !control_pin_present {
        return Err(CommandError::new(
            "control_pin must be set in [smart_effector] for sensitivity programming",
        ));
    }
    Ok(())
}

/// What `SET_SMART_EFFECTOR` reports, one line per item, prefixed
/// `SmartEffector:` (`cmd_SET_SMART_EFFECTOR`; the `accelartion` typo is
/// upstream's and users see this text verbatim).
fn set_message(sensitivity: Option<i64>, probe_accel: f64, recovery_time: f64) -> String {
    let mut lines = Vec::new();
    if let Some(sensitivity) = sensitivity {
        lines.push(format!("sensitivity: {sensitivity}"));
    }
    if probe_accel != 0. {
        lines.push(format!("probing accelartion: {probe_accel:.3}"));
    } else {
        lines.push("probing acceleration control disabled".to_string());
    }
    if recovery_time != 0. {
        lines.push(format!("probe recovery time: {recovery_time:.3}"));
    } else {
        lines.push("probe recovery time disabled".to_string());
    }
    format!("SmartEffector:\n{}", lines.join("\n"))
}

/// Push `buf` to the control pin between two sync points: wait for the moves
/// queued so far (`M400`), take the flushed print time as the start
/// (upstream's `get_last_move_time`), write the bits, dwell across them
/// (`G4`, which advances the print time past the stream) and wait again
/// (`_send_command`).
///
/// # Errors
/// "Printer is not ready" before connect, or whatever the writes report.
async fn send_command(
    printer: &Printer,
    control: &ControlPinHelper,
    buf: &[u8],
) -> Result<(), CommandError> {
    let gcode = printer
        .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
        .expect("the loader registers `gcode` first");
    let toolhead = printer
        .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)
        .ok_or_else(|| CommandError::new("Printer is not ready"))?;
    // Wait for previous actions to finish.
    gcode.run_script_from_command("M400").await?;
    let start_time = toolhead.print_time();
    // Write generated bits to the control pin.
    let end_time = control.write_bits(start_time, &bit_stream(buf))?;
    // Dwell to make sure no subsequent actions are queued together with the
    // SmartEffector programming, then wait for that to finish too.
    gcode
        .run_script_from_command(&format!("G4 P{:.3}", (end_time - start_time) * 1000.))
        .await?;
    gcode.run_script_from_command("M400").await?;
    Ok(())
}

/// One configured `[smart_effector]` (`smart_effector.py:SmartEffectorProbe`,
/// the object upstream registers both as itself and as `probe`).
pub struct SmartEffectorProbe {
    /// The probe family this section built; its `get_status` is what both
    /// the `smart_effector` and the `probe` object report.
    probe: Arc<PrinterProbe>,
}

impl PrinterObject for SmartEffectorProbe {
    fn get_status(&self, eventtime: f64) -> Value {
        // Upstream: `self.cmd_helper.get_status(eventtime)`.
        self.probe.get_status(eventtime)
    }
}

impl std::fmt::Debug for SmartEffectorProbe {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SmartEffectorProbe")
            .field("probe", &self.probe)
            .finish()
    }
}

/// Register `SET_SMART_EFFECTOR`, plus `RESET_SMART_EFFECTOR` when a
/// `control_pin` gave the section its programming pin
/// (`smart_effector.py:__init__`).
///
/// # Errors
/// When a command name is invalid or already registered — a wiring mistake,
/// reported at config load like upstream's.
fn register_commands(
    printer: &Arc<Printer>,
    control: &Arc<Mutex<Control>>,
    has_control_pin: bool,
) -> Result<(), ConfigError> {
    let gcode = printer
        .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
        .expect("the loader registers `gcode` first");

    let printer_weak = Arc::downgrade(printer);
    let set_control = Arc::clone(control);
    gcode
        .register_command(
            "SET_SMART_EFFECTOR",
            Arc::new(move |gcmd| {
                let printer_weak = printer_weak.clone();
                let control = Arc::clone(&set_control);
                Box::pin(async move {
                    let (sensitivity, probe_accel, recovery_time) = {
                        let state = lock_control(&control);
                        parse_set(gcmd, state.probe_accel, state.recovery_time)?
                    };
                    let pin = { lock_control(&control).pin.clone() };
                    check_sensitivity_programming(pin.is_some(), sensitivity)?;
                    if let Some(sensitivity) = sensitivity {
                        let pin = pin.expect("checked by check_sensitivity_programming");
                        let printer = printer_weak
                            .upgrade()
                            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                        let buf = [105u8, sensitivity as u8, (255 - sensitivity) as u8];
                        send_command(&printer, &pin, &buf).await?;
                    }
                    {
                        let mut state = lock_control(&control);
                        state.probe_accel = probe_accel;
                        state.recovery_time = recovery_time;
                    }
                    gcmd.respond_info(&set_message(sensitivity, probe_accel, recovery_time));
                    Ok(())
                })
            }),
            Some("Set SmartEffector parameters"),
            false,
        )
        .map_err(ConfigError::new)?;

    if has_control_pin {
        let printer_weak = Arc::downgrade(printer);
        let reset_control = Arc::clone(control);
        gcode
            .register_command(
                "RESET_SMART_EFFECTOR",
                Arc::new(move |gcmd| {
                    let printer_weak = printer_weak.clone();
                    let control = Arc::clone(&reset_control);
                    Box::pin(async move {
                        let pin = lock_control(&control).pin.clone().ok_or_else(|| {
                            CommandError::new("control_pin must be set in [smart_effector]")
                        })?;
                        let printer = printer_weak
                            .upgrade()
                            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                        send_command(&printer, &pin, &[131, 131]).await?;
                        gcmd.respond_info("SmartEffector sensitivity was reset");
                        Ok(())
                    })
                }),
                Some("Reset SmartEffector settings (sensitivity)"),
                false,
            )
            .map_err(ConfigError::new)?;
    }
    Ok(())
}

/// Upstream's `load_config` for `[smart_effector]`: build the probe family
/// from this section, register it as `probe`, then the effector's own state
/// and commands.
///
/// # Errors
/// When an option is missing or malformed, the probe pin cannot be built, the
/// `probe` chip or object is already taken (a `[probe]` section alongside
/// this one), the control pin is unusable, or a command is already registered.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let options = SmartEffectorOptions::read(config)?;
    // The probe family: reads the probe options, builds the endstop from
    // `pin`, registers the `probe` pin chip and lands the probe commands —
    // once, for this section.
    let probe = Arc::new(PrinterProbe::new(config, printer)?);
    // Upstream: `config.get_printer().add_object('probe', smart_effector)`.
    printer.add_object(PROBE_OBJECT, Arc::clone(&probe) as Arc<dyn PrinterObject>)?;

    let control_pin = options
        .control_pin
        .as_deref()
        .map(|description| ControlPinHelper::new(config, printer, description).map(Arc::new))
        .transpose()?;
    let control = Arc::new(Mutex::new(Control {
        probe_accel: options.probe_accel,
        recovery_time: options.recovery_time,
        pin: control_pin,
    }));
    register_commands(printer, &control, options.control_pin.is_some())?;

    Ok(Arc::new(SmartEffectorProbe { probe }))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{Config, ConfigSection, ConfigValue};
    use crate::core::klippy::pins::{PinChip, PinError, PinParams, PwmOut};
    use crate::core::klippy::reactor::{ManualReactor, TokioReactor};
    use std::collections::HashMap;
    use std::sync::Mutex as StdMutex;

    /// A `[smart_effector]` section with the given options.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("smart_effector", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    #[test]
    fn options_carry_upstream_defaults() {
        let section = section(&[("pin", "PC7"), ("control_pin", "PC5"), ("z_offset", "1.15")]);
        let config = ConfigWrapper::untracked(&section);

        let options = SmartEffectorOptions::read(&config).unwrap();
        assert_eq!(options.control_pin.as_deref(), Some("PC5"));
        assert_eq!(options.probe_accel, 0.);
        assert_eq!(options.recovery_time, 0.4);

        let probe = crate::core::klippy::extras::probe::ProbeOptions::read(&config).unwrap();
        assert_eq!(probe.pin, "PC7");
        assert_eq!(probe.z_offset, 1.15);
    }

    #[test]
    fn every_option_the_corpus_writes_is_claimed() {
        // The corpus's `[smart_effector]` writes `pin`, `control_pin`,
        // `probe_accel`, `z_offset`; upstream's helpers read the whole probe
        // family from the same section too, so `check_unused` needs every one
        // of these claimed.
        let section = section(&[
            ("pin", "PC7"),
            ("control_pin", "!PC5"),
            ("probe_accel", "50"),
            ("recovery_time", "0.2"),
            ("z_offset", "1.15"),
            ("x_offset", "20.0"),
            ("y_offset", "5.0"),
            ("speed", "2.0"),
            ("lift_speed", "10.0"),
            ("samples", "3"),
            ("sample_retract_dist", "4.0"),
            ("samples_result", "average"),
            ("samples_tolerance", "0.05"),
            ("samples_tolerance_retries", "5"),
            ("deactivate_on_each_sample", "false"),
            ("activate_gcode", "probe_reset"),
            ("deactivate_gcode", "probe_reset"),
        ]);
        let config = ConfigWrapper::untracked(&section);

        let options = SmartEffectorOptions::read(&config).unwrap();
        assert_eq!(options.control_pin.as_deref(), Some("!PC5"));
        assert_eq!(options.probe_accel, 50.);
        assert_eq!(options.recovery_time, 0.2);

        let probe = crate::core::klippy::extras::probe::ProbeOptions::read(&config).unwrap();
        assert_eq!(probe.x_offset, 20.0);
        assert_eq!(probe.samples, 3);
        assert_eq!(probe.samples_result, "average");
    }

    #[test]
    fn a_negative_accel_or_recovery_time_is_refused() {
        let accel = section(&[("pin", "PC7"), ("z_offset", "1.15"), ("probe_accel", "-1")]);
        let err = SmartEffectorOptions::read(&ConfigWrapper::untracked(&accel)).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'probe_accel' in section 'smart_effector' must have minimum of 0"
        );

        let recovery = section(&[
            ("pin", "PC7"),
            ("z_offset", "1.15"),
            ("recovery_time", "-0.1"),
        ]);
        let err = SmartEffectorOptions::read(&ConfigWrapper::untracked(&recovery)).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'recovery_time' in section 'smart_effector' must have minimum of 0"
        );
    }

    /// A `SET_SMART_EFFECTOR` command carrying `params`.
    fn set_command(params: &[(&str, &str)]) -> (Arc<Printer>, GCodeDispatch, GcodeCommand) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let gcode = GCodeDispatch::new(Arc::clone(&printer));
        let line = if params.is_empty() {
            "SET_SMART_EFFECTOR".to_string()
        } else {
            let mut line = String::from("SET_SMART_EFFECTOR");
            for (name, value) in params {
                line.push_str(&format!(" {name}={value}"));
            }
            line
        };
        let params = params
            .iter()
            .map(|(name, value)| (name.to_string(), value.to_string()))
            .collect::<HashMap<_, _>>();
        let gcmd = gcode.create_gcode_command("SET_SMART_EFFECTOR", &line, params);
        (printer, gcode, gcmd)
    }

    #[test]
    fn set_parses_its_parameters_with_the_upstream_defaults() {
        let (_printer, _gcode, gcmd) = set_command(&[]);
        let (sensitivity, accel, recovery) = parse_set(&gcmd, 50., 0.4).unwrap();
        assert_eq!(sensitivity, None);
        // ACCEL and RECOVERY_TIME default to the current values.
        assert_eq!(accel, 50.);
        assert_eq!(recovery, 0.4);

        let (_printer, _gcode, gcmd) = set_command(&[
            ("SENSITIVITY", "99"),
            ("ACCEL", "25"),
            ("RECOVERY_TIME", "0"),
        ]);
        let (sensitivity, accel, recovery) = parse_set(&gcmd, 50., 0.4).unwrap();
        assert_eq!(sensitivity, Some(99));
        assert_eq!(accel, 25.);
        assert_eq!(recovery, 0.);
    }

    #[test]
    fn set_refuses_a_sensitivity_or_accel_out_of_range() {
        let (_printer, _gcode, gcmd) = set_command(&[("SENSITIVITY", "256")]);
        let err = parse_set(&gcmd, 0., 0.4).unwrap_err();
        assert!(
            err.to_string()
                .contains("SENSITIVITY must have maximum of 255"),
            "{err}"
        );

        let (_printer, _gcode, gcmd) = set_command(&[("SENSITIVITY", "-1")]);
        let err = parse_set(&gcmd, 0., 0.4).unwrap_err();
        assert!(
            err.to_string()
                .contains("SENSITIVITY must have minimum of 0"),
            "{err}"
        );

        let (_printer, _gcode, gcmd) = set_command(&[("ACCEL", "-1")]);
        let err = parse_set(&gcmd, 0., 0.4).unwrap_err();
        assert!(
            err.to_string().contains("ACCEL must have minimum of 0"),
            "{err}"
        );

        let (_printer, _gcode, gcmd) = set_command(&[("RECOVERY_TIME", "fast")]);
        assert!(parse_set(&gcmd, 0., 0.4).is_err());
    }

    #[test]
    fn sensitivity_without_a_control_pin_is_refused_with_upstream_wording() {
        let err = check_sensitivity_programming(false, Some(50)).unwrap_err();
        assert_eq!(
            err.to_string(),
            "control_pin must be set in [smart_effector] for sensitivity programming"
        );
        // Without a SENSITIVITY ask, no control pin is needed at all.
        check_sensitivity_programming(false, None).unwrap();
        check_sensitivity_programming(true, Some(50)).unwrap();
    }

    #[test]
    fn the_set_message_follows_upstream_text() {
        assert_eq!(
            set_message(Some(99), 50., 0.4),
            "SmartEffector:\n\
             sensitivity: 99\n\
             probing accelartion: 50.000\n\
             probe recovery time: 0.400"
        );
        assert_eq!(
            set_message(None, 0., 0.),
            "SmartEffector:\n\
             probing acceleration control disabled\n\
             probe recovery time disabled"
        );
    }

    #[test]
    fn every_byte_is_framed_the_upstream_way() {
        // `[0 0 1 0 b7 b6 b5 b4 !b4 b3 b2 b1 b0 !b0]` for 105 = 0b01101001.
        assert_eq!(
            bit_stream(&[105]),
            vec![
                false, false, true, false, //
                false, true, true, false, //
                true,  //
                true, false, false, true, //
                false,
            ]
        );
        // SET's payload is three bytes, RESET's two: 14 bits each.
        assert_eq!(bit_stream(&[105, 99, 156]).len(), 42);
        assert_eq!(bit_stream(&[131, 131]).len(), 28);
        // The frame ends on the complement of b0, never on a bare data bit:
        // for byte 0 that complement is 1.
        let framed = bit_stream(&[0]);
        assert!(framed[13]);
        assert_eq!(framed.len(), 14);
    }

    // --------------------------------------------------------------------
    // The control pin against a fake chip
    // --------------------------------------------------------------------

    /// A digital output that records the writes it was given.
    #[derive(Default)]
    struct FakeDigitalOut {
        writes: StdMutex<Vec<(u32, bool)>>,
    }

    impl DigitalOut for FakeDigitalOut {
        fn setup_max_duration(&self, _max_duration: f64) {}
        fn setup_start_value(&self, _start_value: bool, _shutdown_value: bool) {}
        fn queue_digital_out(
            &self,
            clock: u32,
            value: bool,
        ) -> Result<(), crate::core::klippy::mcu::McuError> {
            self.writes.lock().unwrap().push((clock, value));
            Ok(())
        }
        fn update_digital_out(
            &self,
            value: bool,
        ) -> Result<(), crate::core::klippy::mcu::McuError> {
            self.writes.lock().unwrap().push((0, value));
            Ok(())
        }
    }

    /// A chip that hands out a [`FakeDigitalOut`] per setup.
    #[derive(Default)]
    struct FakeChip {
        digital: StdMutex<Vec<Arc<FakeDigitalOut>>>,
    }

    impl PinChip for FakeChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            let out = Arc::new(FakeDigitalOut::default());
            self.digital.lock().unwrap().push(Arc::clone(&out));
            Ok(out)
        }

        fn setup_pwm(&self, _params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
            Err(PinError::Unsupported("pwm".to_string()))
        }
    }

    /// A printer with `gcode` and `pins` over a fake `mcu` chip.
    fn pins_printer() -> (Arc<Printer>, Arc<FakeChip>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let pins = Arc::new(PrinterPins::new());
        let chip = Arc::new(FakeChip::default());
        pins.register_chip("mcu", chip.clone()).unwrap();
        printer.add_object(PINS_OBJECT, pins).unwrap();
        (printer, chip)
    }

    fn control_section() -> ConfigSection {
        section(&[("pin", "PC7"), ("z_offset", "1.15"), ("control_pin", "PC5")])
    }

    #[test]
    fn the_control_pin_is_reserved_as_a_digital_output() {
        let (printer, chip) = pins_printer();
        let section = control_section();
        let config = ConfigWrapper::untracked(&section);

        let control = ControlPinHelper::new(&config, &printer, "PC5").unwrap();
        assert_eq!(control.mcu, "mcu");
        assert_eq!(chip.digital.lock().unwrap().len(), 1);
    }

    #[test]
    fn a_second_user_of_the_control_pin_is_refused() {
        let (printer, _chip) = pins_printer();
        let section = control_section();
        let config = ConfigWrapper::untracked(&section);

        ControlPinHelper::new(&config, &printer, "PC5").unwrap();
        let err = ControlPinHelper::new(&config, &printer, "PC5").unwrap_err();
        assert!(err.to_string().contains("PC5"), "{err}");
    }

    #[test]
    fn an_unknown_chip_or_a_pullup_on_the_control_pin_is_refused() {
        let (printer, _chip) = pins_printer();
        let section = control_section();
        let config = ConfigWrapper::untracked(&section);

        let err = ControlPinHelper::new(&config, &printer, "nope:PC5").unwrap_err();
        assert_eq!(
            err.to_string(),
            "smart_effector: Unknown pin chip name 'nope'"
        );

        let err = ControlPinHelper::new(&config, &printer, "^PC5").unwrap_err();
        assert!(err.to_string().contains("PC5"), "{err}");
    }

    // --------------------------------------------------------------------
    // The whole section, loaded against the fake firmware
    // --------------------------------------------------------------------

    /// The corpus's `[smart_effector]` shape — a cartesian printer whose
    /// `[stepper_z]` homes through `probe:z_virtual_endstop`.
    fn machine_config(dict: &std::path::Path, control_pin: Option<&str>) -> Config {
        let control = control_pin
            .map(|pin| format!("control_pin: {pin}\n"))
            .unwrap_or_default();
        let text = format!(
            "[mcu]\ntest: dict={}\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n\
             max_z_velocity: 5\nmax_z_accel: 100\n\
             [stepper_x]\nstep_pin: PF0\ndir_pin: PF1\nenable_pin: !PD7\nmicrosteps: 16\n\
             rotation_distance: 40\nendstop_pin: ^PE5\nposition_endstop: 0\nposition_max: 200\nhoming_speed: 50\n\
             [stepper_y]\nstep_pin: PF6\ndir_pin: !PF7\nenable_pin: !PF2\nmicrosteps: 16\n\
             rotation_distance: 40\nendstop_pin: ^PJ1\nposition_endstop: 0\nposition_max: 200\nhoming_speed: 50\n\
             [stepper_z]\nstep_pin: PL3\ndir_pin: PL1\nenable_pin: !PK0\nmicrosteps: 16\n\
             rotation_distance: 8\nendstop_pin: probe:z_virtual_endstop\nposition_max: 200\n\
             [smart_effector]\npin: PC7\n{control}probe_accel: 50\nz_offset: 1.15\n",
            dict.display()
        );
        Config::from_text(&text).expect("the config parses").0
    }

    /// Load the corpus-shaped config and bring the machine up, as
    /// `upstream.rs::run_phases` does for an upstream case. The printer is
    /// returned beside the setup outcome so the caller can tear it down
    /// **before** asserting — a panicking assert with the fake device still
    /// open would hang the test runtime at shutdown.
    async fn up_machine(
        dict: &std::path::Path,
        control_pin: Option<&str>,
    ) -> (Arc<Printer>, Result<(), String>) {
        let config = machine_config(dict, control_pin);
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let printer = Arc::new(Printer::new(reactor));
        let mut start_args =
            crate::core::klippy::api::StartArgs::collect("smart_effector.cfg", None);
        start_args.debug_output = Some("_test_output".to_string());
        printer.set_start_args(Arc::new(start_args));
        let setup = async {
            printer
                .load_config(&config)
                .map_err(|err| err.to_string())?;
            if tokio::time::timeout(std::time::Duration::from_secs(10), printer.bring_up())
                .await
                .is_err()
            {
                return Err("bring_up timed out".to_string());
            }
            let state = printer.get_state_message();
            if state.category != crate::core::klippy::printer::PrinterState::Ready {
                return Err(format!("not ready: {}", state.message));
            }
            Ok(())
        }
        .await;
        (printer, setup)
    }

    fn fake_dictionary() -> Option<std::path::PathBuf> {
        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        dict.is_file().then_some(dict)
    }

    /// Chip registration and both refusals, end to end, then the two
    /// commands against the fake firmware: without the `probe` chip this
    /// section registers, the config above cannot load at all (`Unknown pin
    /// chip name 'probe'`), and once loaded `SET_SMART_EFFECTOR` /
    /// `RESET_SMART_EFFECTOR` program the control pin the way the upstream
    /// corpus case exercises them.
    #[tokio::test(flavor = "multi_thread")]
    async fn loading_registers_the_probe_chip_and_object() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, Some("PC5")).await;

        let run = async {
            setup?;
            // Upstream's `add_object('probe', …)`, beside the section's own
            // name; the load above is also the proof that `[stepper_z]`
            // resolved `probe:z_virtual_endstop` through the new chip.
            if printer.lookup_object(PROBE_OBJECT).is_none() {
                return Err("the probe object is not registered".to_string());
            }
            if printer.lookup_object("smart_effector").is_none() {
                return Err("the section object is not registered".to_string());
            }

            // The chip's refusal for a wrong pin name (`probe.py:setup_pin`);
            // `z_virtual_endstop` itself is already reserved by
            // `[stepper_z]`, and the invert/pull-up refusal on that name is
            // `probe.rs`'s own unit test (`ProbeChip` shares the check).
            let pins = printer
                .lookup_object_as::<PrinterPins>(PINS_OBJECT)
                .expect("the loader registers `pins`");
            let err = pins
                .setup_endstop("probe:z", None)
                .err()
                .expect("a non-endstop pin name on the probe chip is refused");
            if err.to_string() != "Probe virtual endstop only useful as endstop pin" {
                return Err(format!("unexpected refusal: {err}"));
            }

            let gcode = printer
                .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                .expect("the loader registers `gcode`");
            let replies = Arc::new(StdMutex::new(Vec::<String>::new()));
            {
                let replies = Arc::clone(&replies);
                gcode.register_output_handler(Arc::new(move |line: &str| {
                    replies.lock().unwrap().push(line.to_string());
                }));
            }

            // Home and move first, as the corpus case does: the bit writes
            // are timed against the planner's print time.
            gcode
                .run_script("G28\nG1 F6000\nG1 X1")
                .await
                .map_err(|err| err.to_string())?;
            gcode
                .run_script("SET_SMART_EFFECTOR SENSITIVITY=99")
                .await
                .map_err(|err| err.to_string())?;
            gcode
                .run_script("RESET_SMART_EFFECTOR")
                .await
                .map_err(|err| err.to_string())?;
            let captured = replies.lock().unwrap().clone();
            Ok(captured)
        }
        .await;
        printer.teardown();
        let replies = run.expect("the machine loads, homes and runs both commands");

        // SET reports the config's `probe_accel: 50` (no ACCEL asked), the
        // default recovery time, and the programmed sensitivity; RESET
        // reports its upstream confirmation.
        let reply = |needle: &str| {
            assert!(
                replies.iter().any(|line| line.contains(needle)),
                "no reply contains {needle:?}: {replies:?}"
            );
        };
        reply("sensitivity: 99");
        reply("probing accelartion: 50.000");
        reply("probe recovery time: 0.400");
        reply("SmartEffector sensitivity was reset");
    }

    /// Without `control_pin`, `RESET_SMART_EFFECTOR` stays unregistered, as
    /// upstream only registers it inside the `if control_pin:` branch, while
    /// `SET_SMART_EFFECTOR` is always there.
    #[tokio::test(flavor = "multi_thread")]
    async fn reset_stays_unregistered_without_a_control_pin() {
        let Some(dict) = fake_dictionary() else {
            return;
        };
        let (printer, setup) = up_machine(&dict, None).await;

        let run: Result<Vec<String>, String> = async {
            setup?;
            let gcode = printer
                .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                .expect("the loader registers `gcode`");
            let replies = Arc::new(StdMutex::new(Vec::<String>::new()));
            {
                let replies = Arc::clone(&replies);
                gcode.register_output_handler(Arc::new(move |line: &str| {
                    replies.lock().unwrap().push(line.to_string());
                }));
            }

            // An unregistered extended command is answered quietly.
            gcode
                .run_script("RESET_SMART_EFFECTOR")
                .await
                .map_err(|err| err.to_string())?;
            // The SET command is registered either way; without a sensitivity
            // ask it needs neither the pin nor the planner's clock.
            gcode
                .run_script("SET_SMART_EFFECTOR RECOVERY_TIME=0.9")
                .await
                .map_err(|err| err.to_string())?;
            let captured = replies.lock().unwrap().clone();
            Ok(captured)
        }
        .await;
        printer.teardown();
        let replies = run.expect("the machine loads and runs SET");

        assert!(
            replies
                .iter()
                .any(|line| line.contains("Unknown command")
                    && line.contains("RESET_SMART_EFFECTOR")),
            "{replies:?}"
        );
        assert!(
            replies
                .iter()
                .any(|line| line.contains("probe recovery time: 0.900")),
            "{replies:?}"
        );
    }
}
