//! `[adc_scaled <name>]` — ADC values rescaled by a measured VREF/VSSA pair.
//!
//! Upstream's `klippy/extras/adc_scaled.py`: the section reads two analog inputs
//! — one measuring the reference voltage (`vref_pin`) and one the negative
//! reference (`vssa_pin`) — smooths both, and registers itself as a **pin chip**
//! under its own name. A consumer's `sensor_pin: vref_scaled:PB0` then resolves
//! `PB0` on that chip, which builds the real ADC on the MCU the two references
//! live on and wraps it so every raw sample comes back as the fraction
//! `(raw - vssa) / (vref - vssa)`.
//!
//! | option | meaning |
//! |---|---|
//! | `vref_pin` | the pin carrying the reference voltage, required |
//! | `vssa_pin` | the pin carrying the negative reference, required |
//! | `smooth_time` | seconds over which both references are smoothed (default 2, must be > 0) |
//!
//! # Load phase
//!
//! The section is `phase = early` for the same reason `[mcu]` is: it is a
//! *provider* of a chip name, and this loader walks a phase's regular sections
//! before its prefix sections. In the generic phase `adc_scaled` would load
//! *after* `[extruder]`/`[heater_bed]`, and those sections' `sensor_pin:
//! vref_scaled:PB0` would still report `Unknown pin chip name 'vref_scaled'`.
//! Upstream achieves the same by loading every prefix section before the generic
//! walk (`klippy/klippy.py:111-121`).
//!
//! # What is not here
//!
//! * **`query_adc`.** Upstream registers both reference inputs and every scaled
//!   input with the `query_adc` object, which answers `QUERY_ADC`. This host has
//!   no `query_adc` module — the same reason `adc_temperature` registers nothing
//!   — and no corpus config asks for it, so the registrations are left out.
//! * **Print-time timestamps.** Upstream smooths on print time; this host's ADC
//!   reports carry the firmware clock (there is no print-time layer yet, TODO
//!   C1), so [`calc_smooth`] applies upstream's formula to whatever time the
//!   batch carries.

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::mcu::{McuEndstop, McuStepper};
use crate::core::klippy::pins::{
    Adc, AdcCallback, DigitalOut, PinChip, PinError, PinParams, PrinterPins, PwmOut, PINS_OBJECT,
};
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form (`[adc_scaled <name>]`) exists upstream
// (`adc_scaled.py:79`); `phase = early` is load-bearing, see the module docs.
section!(
    "adc_scaled",
    order = 20,
    phase = early,
    prefix = load_config_prefix
);

/// The sampling the two reference inputs are configured with
/// (`adc_scaled.py:7-9, 57`). The rest of `setup_adc_sample`'s arguments are
/// upstream's own defaults for a three-argument call.
const SAMPLE_TIME: f64 = 0.001;
const SAMPLE_COUNT: u32 = 8;
const REPORT_TIME: f64 = 0.300;

/// One `[adc_scaled <name>]`: the scaled chip plus the two smoothed references.
pub struct AdcScaled {
    /// The chip name, which is the section's sub (`adc_scaled.py:38`).
    name: String,
    /// The `[mcu]` chip the two references — and every scaled input — live on.
    mcu_chip_name: String,
    /// `1 / smooth_time` (`adc_scaled.py:44-45`).
    inv_smooth_time: f64,
    /// The printer, weakly held: the chip is stored in the printer's own pins
    /// object, so a strong handle would be a cycle that outlives a restart.
    printer: Weak<Printer>,
    /// The last smoothed reference readings, as `(time, value)`. Both start at
    /// `(0., 0.)` as upstream's do.
    last_vref: Arc<Mutex<(f64, f64)>>,
    last_vssa: Arc<Mutex<(f64, f64)>>,
}

impl AdcScaled {
    /// Read the options, build the two reference inputs, and register the chip.
    ///
    /// The order the options are read and the pins are built is upstream's
    /// (`PrinterADCScaled.__init__`): each reference's option and its input come
    /// before the next, `smooth_time` after both, and the same-MCU check last.
    ///
    /// # Errors
    /// A missing `vref_pin`/`vssa_pin`, a `smooth_time` that is not above 0, a
    /// pin that cannot be built, references on two different MCUs, or a chip
    /// name that is already taken.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let name = config.section().sub.clone().ok_or_else(|| {
            ConfigError::new(format!(
                "Section '{}' must be a '[adc_scaled <name>]' section",
                config.identifier()
            ))
        })?;
        let pins = lookup_pins(printer)?;

        // `_config_pin(config, 'vref', …)` then `'vssa'`.
        let vref_pin = config.get("vref_pin", None)?;
        let vref_chip = pin_chip_name(&vref_pin);
        let mcu_vref = setup_reference(&pins, &vref_pin)?;
        let vssa_pin = config.get("vssa_pin", None)?;
        let vssa_chip = pin_chip_name(&vssa_pin);
        let mcu_vssa = setup_reference(&pins, &vssa_pin)?;

        let smooth_time =
            config.get_float_bounded("smooth_time", Some(2.0), None, None, Some(0.0), None)?;
        if vref_chip != vssa_chip {
            return Err(ConfigError::new("vref and vssa must be on same mcu"));
        }

        let scaled = Arc::new(Self {
            name,
            mcu_chip_name: vref_chip,
            inv_smooth_time: 1.0 / smooth_time,
            printer: Arc::downgrade(printer),
            last_vref: Arc::new(Mutex::new((0.0, 0.0))),
            last_vssa: Arc::new(Mutex::new((0.0, 0.0))),
        });
        scaled.watch_references(&mcu_vref, &mcu_vssa);
        pins.register_chip(&scaled.name, scaled.clone())
            .map_err(|err| ConfigError::new(err.to_string()))?;
        Ok(scaled)
    }

    /// Install the smoothing callbacks on the two reference inputs
    /// (`vref_callback` / `vssa_callback`).
    fn watch_references(&self, mcu_vref: &Arc<dyn Adc>, mcu_vssa: &Arc<dyn Adc>) {
        let vref = Arc::clone(&self.last_vref);
        let inv_smooth_time = self.inv_smooth_time;
        mcu_vref.setup_adc_callback(Box::new(move |samples| {
            smooth_into(&vref, samples, inv_smooth_time);
        }));
        let vssa = Arc::clone(&self.last_vssa);
        mcu_vssa.setup_adc_callback(Box::new(move |samples| {
            smooth_into(&vssa, samples, inv_smooth_time);
        }));
    }

    /// The printer's pin registry, for the chip's own setup calls.
    fn pins(&self) -> Result<Arc<PrinterPins>, PinError> {
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| PinError::Message("the printer is gone".to_string()))?;
        lookup_pins(&printer).map_err(|err| PinError::Message(err.to_string()))
    }
}

impl PinChip for AdcScaled {
    /// Build a scaled ADC for `chip:pin` on the MCU the references live on.
    ///
    /// Upstream's `PrinterADCScaled.setup_pin` → `MCU_scaled_adc`: the real ADC
    /// is built with the same pin name on the reference MCU, and each consumer
    /// gets a fresh wrapper over it (one per `sensor_pin`).
    fn setup_adc(&self, params: &PinParams) -> Result<Arc<dyn Adc>, PinError> {
        let pins = self.pins()?;
        let inner = pins.setup_adc(&format!("{}:{}", self.mcu_chip_name, params.pin), None)?;
        Ok(Arc::new(ScaledAdc {
            inner,
            last_vref: Arc::clone(&self.last_vref),
            last_vssa: Arc::clone(&self.last_vssa),
            callback: Arc::new(Mutex::new(None)),
            last_state: Arc::new(Mutex::new((0, 0.0))),
        }))
    }

    // Upstream's `setup_pin` rejects every type but `adc` with one message;
    // here each type is its own method, so each repeats it.

    fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
        Err(only_adc())
    }

    fn setup_static_digital_out(&self, _params: &PinParams) -> Result<(), PinError> {
        Err(only_adc())
    }

    fn setup_pwm(&self, _params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
        Err(only_adc())
    }

    fn setup_stepper(
        &self,
        _step_pin: &PinParams,
        _dir_pin: &PinParams,
        _invert_step: i8,
        _step_pulse_duration: f64,
        _invert_dir: bool,
    ) -> Result<Arc<McuStepper>, PinError> {
        Err(only_adc())
    }

    fn setup_endstop(&self, _params: &PinParams) -> Result<Arc<McuEndstop>, PinError> {
        Err(only_adc())
    }
}

impl PrinterObject for AdcScaled {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    /// Upstream's object defines no `get_status`, so `objects/list` leaves it
    /// out (`adc_scaled.py`); here the trait is the registration, so the filter
    /// is explicit.
    fn is_queryable(&self) -> bool {
        false
    }
}

/// One scaled input: a real ADC wrapped so every sample is rescaled against the
/// section's smoothed references (upstream's `MCU_scaled_adc`).
struct ScaledAdc {
    /// The real ADC on the reference MCU, already resolved to `mcu:PB0`.
    inner: Arc<dyn Adc>,
    /// The section's smoothed references, shared with every other input.
    last_vref: Arc<Mutex<(f64, f64)>>,
    last_vssa: Arc<Mutex<(f64, f64)>>,
    /// The consumer's callback, installed through `setup_adc_callback`.
    callback: Arc<Mutex<Option<AdcCallback>>>,
    /// The last rescaled sample, `(time, value)`; `(0, 0.0)` until one arrives,
    /// as upstream's `_last_state` starts.
    last_state: Arc<Mutex<(u64, f64)>>,
}

impl Adc for ScaledAdc {
    fn setup_adc_sample(
        &self,
        report_time: f64,
        sample_time: f64,
        sample_count: u32,
        batch_num: u32,
        minval: f64,
        maxval: f64,
        range_check_count: u32,
    ) {
        self.inner.setup_adc_sample(
            report_time,
            sample_time,
            sample_count,
            batch_num,
            minval,
            maxval,
            range_check_count,
        );
    }

    /// Store the consumer's callback and subscribe to the real ADC with the
    /// rescaling wrapper (`MCU_scaled_adc.setup_adc_callback`).
    ///
    /// The first reports arrive before the references have ever been read, so
    /// `vref - vssa` is still `0` and the scaled value is a division by zero —
    /// `NaN` for a zero reading, `±inf` for any other. Upstream has the same
    /// hole and no guard; it is reproduced rather than papered over.
    fn setup_adc_callback(&self, callback: AdcCallback) {
        *self.callback.lock().unwrap_or_else(|p| p.into_inner()) = Some(callback);
        let callback = Arc::clone(&self.callback);
        let last_state = Arc::clone(&self.last_state);
        let vref = Arc::clone(&self.last_vref);
        let vssa = Arc::clone(&self.last_vssa);
        self.inner.setup_adc_callback(Box::new(move |samples| {
            let max_adc = vref.lock().unwrap_or_else(|p| p.into_inner()).1;
            let min_adc = vssa.lock().unwrap_or_else(|p| p.into_inner()).1;
            let adjusted: Vec<(u64, f64)> = samples
                .iter()
                .map(|&(time, read_value)| (time, (read_value - min_adc) / (max_adc - min_adc)))
                .collect();
            if let Some(last) = adjusted.last() {
                *last_state.lock().unwrap_or_else(|p| p.into_inner()) = *last;
            }
            if let Some(callback) = callback.lock().unwrap_or_else(|p| p.into_inner()).as_ref() {
                callback(&adjusted);
            }
        }));
    }

    fn get_last_value(&self) -> Option<(u64, f64)> {
        Some(*self.last_state.lock().unwrap_or_else(|p| p.into_inner()))
    }
}

/// Build one reference input and configure its sampling (`_config_pin`).
fn setup_reference(pins: &Arc<PrinterPins>, pin_desc: &str) -> Result<Arc<dyn Adc>, ConfigError> {
    let adc = pins
        .setup_adc(pin_desc, None)
        .map_err(|err| ConfigError::new(err.to_string()))?;
    adc.setup_adc_sample(REPORT_TIME, SAMPLE_TIME, SAMPLE_COUNT, 1, 0.0, 1.0, 0);
    Ok(adc)
}

/// Upstream's `PrinterADCScaled.calc_smooth`: move `last` a `min(Δt /
/// smooth_time, 1)` fraction of the way to the new reading.
fn calc_smooth(
    read_time: f64,
    read_value: f64,
    last: (f64, f64),
    inv_smooth_time: f64,
) -> (f64, f64) {
    let (last_time, last_value) = last;
    let adj_time = ((read_time - last_time) * inv_smooth_time).min(1.0);
    (read_time, last_value + (read_value - last_value) * adj_time)
}

/// Fold one reference report into `state`: the last sample of the batch, as
/// upstream's `vref_callback`/`vssa_callback` do (`samples[-1]`).
fn smooth_into(state: &Mutex<(f64, f64)>, samples: &[(u64, f64)], inv_smooth_time: f64) {
    let (read_time, read_value) = samples[samples.len() - 1];
    let mut last = state.lock().unwrap_or_else(|p| p.into_inner());
    *last = calc_smooth(read_time as f64, read_value, *last, inv_smooth_time);
}

/// The chip a pin description names, as `setup_adc` would resolve it:
/// decorations stripped, `chip:pin` split, a bare name on the main `mcu`.
///
/// Upstream compares the two references' `get_mcu()` objects; the [`Adc`] trait
/// does not carry its chip here, so the names are compared instead — two
/// different chip names cannot name the same MCU (`bltouch.rs` reads a chip name
/// out of a description the same way).
fn pin_chip_name(description: &str) -> String {
    let desc = description.trim().trim_start_matches(['!', '^', '~']);
    match desc.split_once(':') {
        Some((chip, _)) if !chip.trim().is_empty() => chip.trim().to_string(),
        _ => "mcu".to_string(),
    }
}

/// The one refusal upstream's `setup_pin` raises for every non-`adc` type.
fn only_adc() -> PinError {
    PinError::Message("adc_scaled only supports adc pins".to_string())
}

/// The printer's pin registry.
fn lookup_pins(printer: &Arc<Printer>) -> Result<Arc<PrinterPins>, ConfigError> {
    printer
        .lookup_object_as::<PrinterPins>(PINS_OBJECT)
        .ok_or_else(|| ConfigError::new("the pins object is not registered"))
}

/// Build one `[adc_scaled <name>]` section.
///
/// # Errors
/// As [`AdcScaled::new`].
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(AdcScaled::new(config, printer)?)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{Config, ConfigSection, ConfigValue};
    use crate::core::klippy::reactor::ManualReactor;

    /// The sampling parameters one `setup_adc_sample` call stored.
    #[derive(Debug, Clone, Copy, PartialEq)]
    struct Sampling {
        report_time: f64,
        sample_time: f64,
        sample_count: u32,
        batch_num: u32,
        minval: f64,
        maxval: f64,
        range_check_count: u32,
    }

    /// An ADC that records how it was configured and lets the test push sample
    /// batches through the installed callback.
    #[derive(Default)]
    struct FakeAdc {
        sampling: Mutex<Option<Sampling>>,
        callback: Mutex<Option<AdcCallback>>,
        last_value: Mutex<Option<(u64, f64)>>,
    }

    impl FakeAdc {
        /// Push one batch, as a report from the firmware would.
        fn emit(&self, samples: &[(u64, f64)]) {
            *self.last_value.lock().unwrap() = samples.last().copied();
            if let Some(callback) = self.callback.lock().unwrap().as_ref() {
                callback(samples);
            }
        }
    }

    impl Adc for FakeAdc {
        fn setup_adc_sample(
            &self,
            report_time: f64,
            sample_time: f64,
            sample_count: u32,
            batch_num: u32,
            minval: f64,
            maxval: f64,
            range_check_count: u32,
        ) {
            *self.sampling.lock().unwrap() = Some(Sampling {
                report_time,
                sample_time,
                sample_count,
                batch_num,
                minval,
                maxval,
                range_check_count,
            });
        }

        fn setup_adc_callback(&self, callback: AdcCallback) {
            *self.callback.lock().unwrap() = Some(callback);
        }

        fn get_last_value(&self) -> Option<(u64, f64)> {
            *self.last_value.lock().unwrap()
        }
    }

    /// A chip that answers `setup_adc` with a fresh [`FakeAdc`], recording the
    /// pin each one was built for.
    #[derive(Default)]
    struct FakeChip {
        adcs: Mutex<Vec<(String, Arc<FakeAdc>)>>,
    }

    impl PinChip for FakeChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            Err(PinError::Unsupported("digital_out".to_string()))
        }

        fn setup_adc(&self, params: &PinParams) -> Result<Arc<dyn Adc>, PinError> {
            let adc = Arc::new(FakeAdc::default());
            self.adcs
                .lock()
                .unwrap()
                .push((params.pin.clone(), Arc::clone(&adc)));
            Ok(adc)
        }
    }

    /// A printer with `pins` over a fake `mcu` chip.
    fn printer() -> (Arc<Printer>, Arc<FakeChip>, Arc<PrinterPins>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let pins = Arc::new(PrinterPins::new());
        let chip = Arc::new(FakeChip::default());
        pins.register_chip("mcu", chip.clone()).unwrap();
        printer.add_object(PINS_OBJECT, pins.clone()).unwrap();
        (printer, chip, pins)
    }

    /// A `[adc_scaled vref_scaled]` section with `options`.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("adc_scaled", Some("vref_scaled"));
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    /// The error from a call that has to fail: `unwrap_err` needs the success
    /// type to be `Debug`, and a built resource is not.
    fn expect_err<T, E>(result: Result<T, E>) -> E {
        match result {
            Ok(_) => panic!("the call was expected to fail"),
            Err(err) => err,
        }
    }

    /// The ADCs built on `chip` so far, as `(pin, adc)`.
    fn adcs(chip: &FakeChip) -> Vec<(String, Arc<FakeAdc>)> {
        chip.adcs.lock().unwrap().clone()
    }

    /// The `.cfg`-shaped config the corpus uses: a cartesian printer whose
    /// extruder reads `vref_scaled:PB0`, with `scaled` inserted before it.
    fn full_config(scaled: &str) -> String {
        format!(
            "[mcu]\nserial: /dev/not-opened-yet\n{scaled}\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_y]\nstep_pin: PA2\ndir_pin: PA3\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z]\nstep_pin: PA4\ndir_pin: PA5\nrotation_distance: 8\nmicrosteps: 16\nposition_max: 200\n\
             [extruder]\nstep_pin: PA6\ndir_pin: PA7\nrotation_distance: 33.5\nmicrosteps: 16\n\
             nozzle_diameter: 0.4\nfilament_diameter: 1.75\nheater_pin: PB1\n\
             sensor_type: EPCOS 100K B57560G104F\nsensor_pin: vref_scaled:PB0\n\
             control: pid\npid_Kp: 1\npid_Ki: 0.1\npid_Kd: 10\n\
             min_temp: 0\nmax_temp: 250\nmin_extrude_temp: 0\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n"
        )
    }

    const SCALED_SECTION: &str = "[adc_scaled vref_scaled]\nvref_pin: PA17\nvssa_pin: PA19\n";

    fn load(text: &str) -> (Arc<Printer>, Result<(), ConfigError>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text(text).expect("the config parses");
        let result = printer.load_config(&config);
        (printer, result)
    }

    // -----------------------------------------------------------------------
    // The section and its reference inputs
    // -----------------------------------------------------------------------

    #[test]
    fn test_the_section_registers_its_chip_and_samples_the_references() {
        let (printer, chip, pins) = printer();
        AdcScaled::new(
            &wrap(&section(&[("vref_pin", "PA17"), ("vssa_pin", "PA19")])),
            &printer,
        )
        .unwrap();

        assert!(pins.chips().contains(&"vref_scaled".to_string()));
        let built = adcs(&chip);
        assert_eq!(built.len(), 2);
        assert_eq!(built[0].0, "PA17");
        assert_eq!(built[1].0, "PA19");
        // 0.300 s report, 0.001 s between samples, 8 averaged, upstream's other
        // defaults for the three-argument call.
        assert_eq!(
            built[0].1.sampling.lock().unwrap().unwrap(),
            Sampling {
                report_time: 0.300,
                sample_time: 0.001,
                sample_count: 8,
                batch_num: 1,
                minval: 0.0,
                maxval: 1.0,
                range_check_count: 0,
            }
        );
    }

    #[test]
    fn test_smooth_time_defaults_to_two_seconds() {
        let (printer, _chip, _pins) = printer();
        let scaled = AdcScaled::new(
            &wrap(&section(&[("vref_pin", "PA17"), ("vssa_pin", "PA19")])),
            &printer,
        )
        .unwrap();

        assert_eq!(scaled.inv_smooth_time, 0.5);
    }

    #[test]
    fn test_a_smooth_time_that_is_not_above_zero_is_rejected() {
        let (printer, _chip, _pins) = printer();
        let err = expect_err(AdcScaled::new(
            &wrap(&section(&[
                ("vref_pin", "PA17"),
                ("vssa_pin", "PA19"),
                ("smooth_time", "0"),
            ])),
            &printer,
        ));

        assert_eq!(
            err.to_string(),
            "Option 'smooth_time' in section 'adc_scaled vref_scaled' must be above 0"
        );
    }

    #[test]
    fn test_each_reference_pin_is_required() {
        let (printer, _chip, _pins) = printer();
        let err = expect_err(AdcScaled::new(
            &wrap(&section(&[("vssa_pin", "PA19")])),
            &printer,
        ));
        assert_eq!(
            err.to_string(),
            "Option 'vref_pin' in section 'adc_scaled vref_scaled' must be specified"
        );

        let err = expect_err(AdcScaled::new(
            &wrap(&section(&[("vref_pin", "PA17")])),
            &printer,
        ));
        assert_eq!(
            err.to_string(),
            "Option 'vssa_pin' in section 'adc_scaled vref_scaled' must be specified"
        );
    }

    #[test]
    fn test_references_on_different_mcus_are_rejected() {
        let (printer, _chip, pins) = printer();
        pins.register_chip("toolboard", Arc::new(FakeChip::default()))
            .unwrap();

        let err = expect_err(AdcScaled::new(
            &wrap(&section(&[
                ("vref_pin", "PA17"),
                ("vssa_pin", "toolboard:PA6"),
            ])),
            &printer,
        ));

        assert_eq!(err.to_string(), "vref and vssa must be on same mcu");
    }

    #[test]
    fn test_a_bare_pin_is_on_the_main_mcu_like_an_explicit_one() {
        let (printer, chip, _pins) = printer();
        // `PA17` and `mcu:PA19` are the same MCU, so this loads and the scaled
        // inputs are built on `mcu` too.
        let scaled = AdcScaled::new(
            &wrap(&section(&[("vref_pin", "PA17"), ("vssa_pin", "mcu:PA19")])),
            &printer,
        )
        .unwrap();

        assert_eq!(scaled.mcu_chip_name, "mcu");
        let adc = scaled
            .pins()
            .unwrap()
            .setup_adc("vref_scaled:PB0", None)
            .unwrap();
        assert!(adc.get_last_value().is_some());
        assert_eq!(adcs(&chip)[2].0, "PB0");
    }

    #[test]
    fn test_a_secondboard_reference_is_accepted_and_used_for_scaled_inputs() {
        let (printer, chip, pins) = printer();
        let toolboard = Arc::new(FakeChip::default());
        pins.register_chip("toolboard", toolboard.clone()).unwrap();
        let scaled = AdcScaled::new(
            &wrap(&section(&[
                ("vref_pin", "toolboard:PA7"),
                ("vssa_pin", "toolboard:PA6"),
            ])),
            &printer,
        )
        .unwrap();

        assert_eq!(scaled.mcu_chip_name, "toolboard");
        scaled
            .pins()
            .unwrap()
            .setup_adc("vref_scaled:PB9", None)
            .unwrap();

        assert_eq!(adcs(&chip).len(), 0);
        assert_eq!(adcs(&toolboard).len(), 3);
        assert_eq!(adcs(&toolboard)[2].0, "PB9");
    }

    // -----------------------------------------------------------------------
    // The chip side
    // -----------------------------------------------------------------------

    #[test]
    fn test_every_scaled_input_builds_its_own_inner_adc() {
        let (printer, chip, pins) = printer();
        AdcScaled::new(
            &wrap(&section(&[("vref_pin", "PA17"), ("vssa_pin", "PA19")])),
            &printer,
        )
        .unwrap();

        let first = pins.setup_adc("vref_scaled:PB0", None).unwrap();
        let second = pins.setup_adc("vref_scaled:PB1", None).unwrap();

        // The two references, then one real ADC per scaled pin.
        let built = adcs(&chip);
        assert_eq!(built.len(), 4);
        assert_eq!(built[2].0, "PB0");
        assert_eq!(built[3].0, "PB1");
        // The wrapper has not seen a report yet, so its last state is the
        // `(0, 0.0)` upstream starts from.
        assert_eq!(first.get_last_value(), Some((0, 0.0)));
        assert_eq!(second.get_last_value(), Some((0, 0.0)));
    }

    #[test]
    fn test_only_adc_pins_are_supported() {
        let (printer, _chip, pins) = printer();
        AdcScaled::new(
            &wrap(&section(&[("vref_pin", "PA17"), ("vssa_pin", "PA19")])),
            &printer,
        )
        .unwrap();

        // Each call looks the pin up, so each gets its own description.
        assert_eq!(
            expect_err(pins.setup_pwm("vref_scaled:PB0", None)).to_string(),
            "adc_scaled only supports adc pins"
        );
        assert_eq!(
            expect_err(pins.setup_digital_out("vref_scaled:PB1", None)).to_string(),
            "adc_scaled only supports adc pins"
        );
        assert_eq!(
            expect_err(pins.setup_endstop("vref_scaled:PB2", None)).to_string(),
            "adc_scaled only supports adc pins"
        );
    }

    // -----------------------------------------------------------------------
    // The rescaling
    // -----------------------------------------------------------------------

    #[test]
    fn test_a_scaled_input_is_rescaled_against_the_smoothed_references() {
        let (printer, chip, pins) = printer();
        AdcScaled::new(
            &wrap(&section(&[("vref_pin", "PA17"), ("vssa_pin", "PA19")])),
            &printer,
        )
        .unwrap();
        let scaled = pins.setup_adc("vref_scaled:PB0", None).unwrap();

        let built = adcs(&chip);
        // vref reads 0.8, vssa 0.1 — the first report is unsmoothed (Δt from
        // the `(0., 0.)` start is large, so `adj` is capped at 1).
        built[0].1.emit(&[(1_000, 0.8)]);
        built[1].1.emit(&[(1_000, 0.1)]);
        let seen = Arc::new(Mutex::new(Vec::new()));
        let seen_for_callback = Arc::clone(&seen);
        scaled.setup_adc_callback(Box::new(move |samples| {
            seen_for_callback.lock().unwrap().extend_from_slice(samples);
        }));

        built[2].1.emit(&[(2_000, 0.4)]);

        // `(raw - vssa) / (vref - vssa)`, vref being the maximum.
        let expected = (0.4 - 0.1) / (0.8 - 0.1);
        assert_eq!(*seen.lock().unwrap(), vec![(2_000, expected)]);
        assert_eq!(scaled.get_last_value(), Some((2_000, expected)));
    }

    #[test]
    fn test_a_report_before_the_references_arrive_divides_by_zero() {
        let (printer, chip, pins) = printer();
        AdcScaled::new(
            &wrap(&section(&[("vref_pin", "PA17"), ("vssa_pin", "PA19")])),
            &printer,
        )
        .unwrap();
        let scaled = pins.setup_adc("vref_scaled:PB0", None).unwrap();
        let seen = Arc::new(Mutex::new(Vec::new()));
        let seen_for_callback = Arc::clone(&seen);
        scaled.setup_adc_callback(Box::new(move |samples| {
            seen_for_callback.lock().unwrap().extend_from_slice(samples);
        }));

        // No reference has been read yet, so `(raw - vssa) / (vref - vssa)` is
        // a division by zero. Upstream has no guard; both shapes are
        // reproduced.
        let built = adcs(&chip);
        built[2].1.emit(&[(1_000, 0.0)]);
        built[2].1.emit(&[(2_000, 0.4)]);

        let samples = seen.lock().unwrap();
        assert_eq!(samples.len(), 2);
        assert!(samples[0].1.is_nan());
        assert_eq!(samples[1].1, f64::INFINITY);
        assert_eq!(scaled.get_last_value(), Some((2_000, f64::INFINITY)));
    }

    #[test]
    fn test_the_references_are_smoothed_by_min_delta_over_smooth_time() {
        // smooth_time = 2 → `inv_smooth_time` = 0.5.
        let inv = 0.5;
        // No time passed: the reading does not move the state.
        assert_eq!(calc_smooth(0.0, 1.0, (0.0, 0.0), inv), (0.0, 0.0));
        // One second of two: half the way.
        assert_eq!(calc_smooth(1.0, 1.0, (0.0, 0.0), inv), (1.0, 0.5));
        // Two seconds: the whole way, and no further.
        assert_eq!(calc_smooth(3.0, 1.0, (1.0, 0.5), inv), (3.0, 1.0));
        assert_eq!(calc_smooth(100.0, 1.0, (3.0, 0.5), inv), (100.0, 1.0));
    }

    #[test]
    fn test_the_reference_callback_smooths_the_last_sample_of_a_batch() {
        let (printer, chip, _pins) = printer();
        let scaled = AdcScaled::new(
            &wrap(&section(&[("vref_pin", "PA17"), ("vssa_pin", "PA19")])),
            &printer,
        )
        .unwrap();

        // Two reports a second apart: the second moves vref halfway to 1.0.
        let built = adcs(&chip);
        built[0].1.emit(&[(0, 0.0)]);
        built[0].1.emit(&[(1, 1.0)]);
        assert_eq!(*scaled.last_vref.lock().unwrap(), (1.0, 0.5));

        // Only the last sample of a batch counts: vssa moves half of the way
        // from (0, 0) to the batch's last reading (1, 0.5).
        built[1].1.emit(&[(0, 0.25), (1, 0.5)]);
        assert_eq!(*scaled.last_vssa.lock().unwrap(), (1.0, 0.25));
    }

    // -----------------------------------------------------------------------
    // The real loader
    // -----------------------------------------------------------------------

    #[test]
    fn test_the_section_loads_from_a_full_config() {
        // The corpus shape: `[adc_scaled]` and then an `[extruder]` whose
        // `sensor_pin` names the chip it registered. The chip has to exist by
        // the time the extruder loads, which is why the section is early.
        let (printer, result) = load(&full_config(SCALED_SECTION));
        result.expect("the config loads");

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins`");
        assert!(pins.chips().contains(&"vref_scaled".to_string()));
        printer
            .lookup_object_as::<AdcScaled>("adc_scaled vref_scaled")
            .expect("the section is registered");
        // No `get_status` upstream, so `objects/list` leaves it out.
        assert!(!printer
            .queryable_objects()
            .contains(&"adc_scaled vref_scaled".to_string()));
    }

    #[test]
    fn test_a_scaled_pin_without_the_section_is_an_unknown_chip() {
        let (_printer, result) = load(&full_config(""));

        assert_eq!(
            result.unwrap_err().to_string(),
            "sensor_pin: Unknown pin chip name 'vref_scaled'"
        );
    }
}
