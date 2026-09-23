//! `[probe]` — the probe's virtual Z endstop.
//!
//! Upstream `klippy/extras/probe.py`. This unit lands the section, its option
//! set and the `probe` virtual pin chip, so that
//! `endstop_pin: probe:z_virtual_endstop` resolves and the Z rail homes on the
//! probe's physical pin (the pin itself is built with `PrinterPins::setup_endstop`
//! and handed back through the chip).
//!
//! What is **not** here yet (tracked in `TODO.md` H9):
//!
//! - the endstop *wrapper*'s overrides — `z_offset` folded into the reported
//!   trigger position, `query_endstop`/`get_position_endstop` semantics,
//!   `multi_probe_begin/end` and the `probe_prepare`/`probe_finish` hooks. The
//!   pin layer's `PinChip::setup_endstop` returns a concrete `Arc<McuEndstop>`,
//!   so a virtual chip cannot hand back a wrapper type yet; the trait has to
//!   become an interface first.
//! - `QUERY_PROBE` / `PROBE` / `PROBE_ACCURACY` and the session sampling logic
//!   (`ProbeSessionHelper`).
//! - `activate_gcode` / `deactivate_gcode` templates: they need the
//!   `[gcode_macro]` template machinery, which this port does not have (H3).
//!   The options are read and recorded; a section that sets them warns, and
//!   they will be rendered (or refused) once H3 lands.

use std::sync::Arc;

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::McuEndstop;
use crate::core::klippy::pins::{
    DigitalOut, PinChip, PinError, PinParams, PrinterPins, PINS_OBJECT,
};
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("probe", order = 30, load = load_config);

/// The chip name the virtual endstop is reached under.
const CHIP_NAME: &str = "probe";

/// The only pin name the chip answers to (`klippy/extras/probe.py:222-229`).
const VIRTUAL_ENDSTOP: &str = "z_virtual_endstop";

/// The `[probe]` options as written (`probe.py:563-600`).
#[derive(Debug, Clone, PartialEq)]
pub struct ProbeOptions {
    /// The physical probe pin.
    pub pin: String,
    /// The probe's trigger offset from the nozzle.
    pub z_offset: f64,
    /// Probe-to-nozzle X offset.
    pub x_offset: f64,
    /// Probe-to-nozzle Y offset.
    pub y_offset: f64,
    /// Probing speed.
    pub speed: f64,
    /// Speed for the retract moves between samples.
    pub lift_speed: Option<f64>,
    /// Samples per probe.
    pub samples: i64,
    /// Retract distance between samples.
    pub sample_retract_dist: f64,
    /// `median` or `average`.
    pub samples_result: String,
    /// How far the samples may spread.
    pub samples_tolerance: f64,
    /// How many times a spread sample set is retried.
    pub samples_tolerance_retries: i64,
    /// Retract the probe between samples.
    pub deactivate_on_each_sample: bool,
    /// G-code template run before probing (needs `[gcode_macro]`, H3).
    pub activate_gcode: Option<String>,
    /// G-code template run after probing (needs `[gcode_macro]`, H3).
    pub deactivate_gcode: Option<String>,
}

impl ProbeOptions {
    /// Read every option the section accepts, so `check_unused` passes.
    ///
    /// # Errors
    /// As the option readers: a missing `pin` or `z_offset`, a `speed` that is
    /// not above zero, an unknown `samples_result`, and so on.
    pub fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        Ok(Self {
            pin: config.get("pin", None)?,
            z_offset: config.get_float("z_offset", None)?,
            x_offset: config.get_float("x_offset", Some(0.0))?,
            y_offset: config.get_float("y_offset", Some(0.0))?,
            speed: config.get_float_bounded("speed", Some(5.0), None, None, Some(0.0), None)?,
            lift_speed: config.get_optional_float("lift_speed")?,
            samples: config.get_int_bounded("samples", Some(1), Some(1), None)?,
            sample_retract_dist: config.get_float_bounded(
                "sample_retract_dist",
                Some(2.0),
                None,
                None,
                Some(0.0),
                None,
            )?,
            samples_result: config.get_choice(
                "samples_result",
                &["median", "average"],
                Some("median"),
            )?,
            samples_tolerance: config.get_float_bounded(
                "samples_tolerance",
                Some(0.100),
                Some(0.0),
                None,
                None,
                None,
            )?,
            samples_tolerance_retries: config.get_int_bounded(
                "samples_tolerance_retries",
                Some(0),
                Some(0),
                None,
            )?,
            deactivate_on_each_sample: config.get_bool("deactivate_on_each_sample", Some(true))?,
            activate_gcode: config.get_str("activate_gcode"),
            deactivate_gcode: config.get_str("deactivate_gcode"),
        })
    }
}

/// The chip behind `endstop_pin: probe:…`.
struct ProbeChip {
    /// The physical probe endstop the virtual name resolves to.
    endstop: Arc<McuEndstop>,
}

impl PinChip for ProbeChip {
    fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
        Err(PinError::Unsupported("digital_out".to_string()))
    }

    fn setup_endstop(&self, params: &PinParams) -> Result<Arc<McuEndstop>, PinError> {
        check_virtual_endstop(params)?;
        Ok(Arc::clone(&self.endstop))
    }
}

/// Upstream's two refusals for the virtual endstop (`probe.py:223-229`).
///
/// Split out so the checks are testable without an MCU.
fn check_virtual_endstop(params: &PinParams) -> Result<(), PinError> {
    if params.pin != VIRTUAL_ENDSTOP {
        return Err(PinError::Message(
            "Probe virtual endstop only useful as endstop pin".to_string(),
        ));
    }
    if params.invert || params.pullup != 0 {
        return Err(PinError::Message(
            "Can not pullup/invert probe virtual endstop".to_string(),
        ));
    }
    Ok(())
}

/// One configured `[probe]` (`probe.py:PrinterProbe`).
pub struct PrinterProbe {
    /// The section's identifier, for logging and `Debug`.
    identifier: String,
    /// The options as read; later units consume them.
    options: ProbeOptions,
    /// The physical probe endstop, also reachable as `probe:z_virtual_endstop`.
    endstop: Arc<McuEndstop>,
}

impl PrinterProbe {
    /// Build the physical endstop and register the `probe` virtual chip.
    ///
    /// # Errors
    /// Returns a config error when an option is missing or malformed, when the
    /// probe pin cannot be built, or when the `probe` chip is already taken.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let options = ProbeOptions::read(config)?;

        if options.activate_gcode.is_some() || options.deactivate_gcode.is_some() {
            warn!(
                "[{identifier}]: activate_gcode/deactivate_gcode need [gcode_macro], \
                 which is not implemented yet; the templates are recorded but not rendered"
            );
        }

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let endstop = pins
            .setup_endstop(&options.pin, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        // Upstream registers the chip while the section loads
        // (`probe.py:HomingViaProbeHelper.__init__`), which is what makes
        // `endstop_pin: probe:z_virtual_endstop` resolvable for the rails.
        pins.register_chip(
            CHIP_NAME,
            Arc::new(ProbeChip {
                endstop: Arc::clone(&endstop),
            }),
        )
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        Ok(Self {
            identifier,
            options,
            endstop,
        })
    }

    /// The section identifier.
    pub fn identifier(&self) -> &str {
        &self.identifier
    }

    /// The options as read.
    pub fn options(&self) -> &ProbeOptions {
        &self.options
    }

    /// The physical probe endstop.
    pub fn endstop(&self) -> &Arc<McuEndstop> {
        &self.endstop
    }
}

impl PrinterObject for PrinterProbe {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    /// Upstream's `PrinterProbe` defines no `get_status`, so `objects/list`
    /// leaves the `probe` object out.
    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for PrinterProbe {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterProbe")
            .field("identifier", &self.identifier)
            .field("pin", &self.options.pin)
            .field("z_offset", &self.options.z_offset)
            .finish()
    }
}

/// Upstream's `load_config` for `[probe]`.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PrinterProbe::new(config, printer)?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{ConfigSection, ConfigValue};

    /// A section with the given options, as the parser would build it.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("probe", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    fn params(pin: &str, invert: bool, pullup: i8) -> PinParams {
        PinParams {
            chip_name: CHIP_NAME.to_string(),
            pin: pin.to_string(),
            invert,
            pullup,
            share_type: None,
        }
    }

    #[test]
    fn options_carry_upstream_defaults() {
        let section = section(&[("pin", "PA0"), ("z_offset", "1.5")]);
        let options = ProbeOptions::read(&ConfigWrapper::untracked(&section)).unwrap();

        assert_eq!(options.pin, "PA0");
        assert_eq!(options.z_offset, 1.5);
        assert_eq!(options.x_offset, 0.0);
        assert_eq!(options.y_offset, 0.0);
        assert_eq!(options.speed, 5.0);
        assert_eq!(options.lift_speed, None);
        assert_eq!(options.samples, 1);
        assert_eq!(options.sample_retract_dist, 2.0);
        assert_eq!(options.samples_result, "median");
        assert_eq!(options.samples_tolerance, 0.100);
        assert_eq!(options.samples_tolerance_retries, 0);
        assert!(options.deactivate_on_each_sample);
        assert_eq!(options.activate_gcode, None);
        assert_eq!(options.deactivate_gcode, None);
    }

    #[test]
    fn every_option_the_corpus_writes_is_claimed() {
        // The option set the upstream corpus exercises (33 `[probe]` sections),
        // so `check_unused` passes on all of them.
        let section = section(&[
            ("pin", "^PA0"),
            ("z_offset", "2.0"),
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
        let options = ProbeOptions::read(&ConfigWrapper::untracked(&section)).unwrap();

        assert_eq!(options.pin, "^PA0");
        assert_eq!(options.samples, 3);
        assert_eq!(options.samples_result, "average");
        assert_eq!(options.samples_tolerance_retries, 5);
        assert!(!options.deactivate_on_each_sample);
        assert_eq!(options.activate_gcode.as_deref(), Some("probe_reset"));
        assert_eq!(options.deactivate_gcode.as_deref(), Some("probe_reset"));
    }

    #[test]
    fn a_bad_samples_result_is_refused() {
        let section = section(&[
            ("pin", "PA0"),
            ("z_offset", "1.0"),
            ("samples_result", "mode"),
        ]);
        let err = ProbeOptions::read(&ConfigWrapper::untracked(&section)).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Choice 'mode' for option 'samples_result' in section 'probe' is not a valid choice"
        );
    }

    #[test]
    fn the_virtual_endstop_pin_name_is_accepted() {
        check_virtual_endstop(&params(VIRTUAL_ENDSTOP, false, 0)).unwrap();
    }

    #[test]
    fn another_pin_name_is_refused_like_upstream() {
        let err = check_virtual_endstop(&params("z", false, 0)).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Probe virtual endstop only useful as endstop pin"
        );
    }

    #[test]
    fn inverting_or_pulling_up_the_virtual_endstop_is_refused() {
        let inverted = check_virtual_endstop(&params(VIRTUAL_ENDSTOP, true, 0)).unwrap_err();
        assert_eq!(
            inverted.to_string(),
            "Can not pullup/invert probe virtual endstop"
        );

        let pulled_up = check_virtual_endstop(&params(VIRTUAL_ENDSTOP, false, 1)).unwrap_err();
        assert_eq!(
            pulled_up.to_string(),
            "Can not pullup/invert probe virtual endstop"
        );
    }
}
