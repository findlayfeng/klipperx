//! ADC → temperature converters and the bridge to the heater callback.
//!
//! Upstream's `adc_temperature.py`: linear interpolation, voltage sensors
//! (AD595/AD597/AD849x), resistance sensors (PT100/PT1000), and the
//! `PrinterADCtoTemperature` object that hooks an MCU ADC into the heater
//! callback chain.
//!
//! This module registers the built-in sensor factories with `heaters`
//! (the `[adc_temperature]` section exists only to be loaded).

use std::sync::{Arc, Mutex};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::heaters::{self, PrinterHeaters, Sensor, SensorCallback};
use crate::core::klippy::load::section;
use crate::core::klippy::pins::{Adc, PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// ===========================================================================
// Linear interpolation
// ===========================================================================

/// A piecewise-linear table for forward and reverse lookup.
pub struct LinearInterpolate {
    keys: Vec<f64>,
    slopes: Vec<(f64, f64)>,
}

impl LinearInterpolate {
    pub fn new(samples: &[(f64, f64)]) -> Result<Self, ConfigError> {
        if samples.len() < 2 {
            return Err(ConfigError::new(
                "linear interpolation requires at least two samples",
            ));
        }

        let mut keys: Vec<f64> = Vec::new();
        let mut slopes: Vec<(f64, f64)> = Vec::new();
        let mut last_key = 0.0;
        let mut last_value = 0.0;
        let mut first = true;

        for &(key, value) in samples {
            if !first && key <= last_key {
                return Err(ConfigError::new("linear interpolation: duplicate value"));
            }
            if first {
                last_key = key;
                last_value = value;
                first = false;
                continue;
            }
            let gain = (value - last_value) / (key - last_key);
            let offset = last_value - last_key * gain;
            if let Some(&(g, o)) = slopes.last() {
                if g == gain && o == offset {
                    last_key = key;
                    last_value = value;
                    continue;
                }
            }
            last_key = key;
            last_value = value;
            keys.push(key);
            slopes.push((gain, offset));
        }

        if keys.is_empty() {
            return Err(ConfigError::new(
                "linear interpolation: need at least two samples",
            ));
        }

        keys.push(f64::MAX);
        slopes.push(slopes[slopes.len() - 1]);

        Ok(Self { keys, slopes })
    }

    /// Forward: key → value.
    pub fn interpolate(&self, key: f64) -> f64 {
        let pos = self.keys.partition_point(|&k| k < key);
        let pos = pos.min(self.slopes.len() - 1);
        let (gain, offset) = self.slopes[pos];
        key * gain + offset
    }

    /// Reverse: value → key.
    pub fn reverse_interpolate(&self, value: f64) -> f64 {
        let values: Vec<f64> = self
            .keys
            .iter()
            .zip(self.slopes.iter())
            .map(|(&k, &(g, o))| k * g + o)
            .collect();

        let increasing = values[0] < values[values.len() - 2];
        let valid: Vec<usize> = if increasing {
            self.keys
                .iter()
                .enumerate()
                .filter_map(|(i, _)| if values[i] >= value { Some(i) } else { None })
                .collect()
        } else {
            self.keys
                .iter()
                .enumerate()
                .filter_map(|(i, _)| if values[i] <= value { Some(i) } else { None })
                .collect()
        };

        let pos = valid.first().copied().unwrap_or(self.slopes.len() - 1);
        let (gain, offset) = self.slopes[pos];
        (value - offset) / gain
    }
}

// ===========================================================================
// Linear voltage → temperature
// ===========================================================================

/// ADC voltage sensors calibrated from temperature measurements.
pub struct LinearVoltage {
    table: LinearInterpolate,
}

impl LinearVoltage {
    pub fn new(config: &ConfigWrapper, params: &[(f64, f64)]) -> Result<Self, ConfigError> {
        let adc_voltage =
            config.get_float_bounded("adc_voltage", Some(5.0), Some(0.0), None, None, None)?;
        let voltage_offset =
            config.get_float_bounded("voltage_offset", Some(0.0), None, None, None, None)?;

        let mut samples: Vec<(f64, f64)> = Vec::new();
        for &(temp, volt) in params {
            let adc = (volt - voltage_offset) / adc_voltage;
            if adc < 0.0 || adc > 1.0 {
                continue;
            }
            samples.push((adc, temp));
        }

        let table = LinearInterpolate::new(&samples)?;
        Ok(Self { table })
    }

    pub fn calc_temp(&self, adc: f64) -> f64 {
        self.table.interpolate(adc)
    }

    pub fn calc_adc(&self, temp: f64) -> f64 {
        self.table.reverse_interpolate(temp)
    }
}

// ===========================================================================
// Linear resistance → temperature
// ===========================================================================

/// Linear resistance calibrated from temperature measurements.
pub struct LinearResistance {
    pullup: f64,
    table: LinearInterpolate,
}

impl LinearResistance {
    pub fn new(config: &ConfigWrapper, samples: &[(f64, f64)]) -> Result<Self, ConfigError> {
        let pullup = config.get_float_bounded(
            "pullup_resistor",
            Some(4700.0),
            Some(0.0),
            None,
            None,
            None,
        )?;
        let res_to_temp: Vec<(f64, f64)> = samples.iter().map(|&(t, r)| (r, t)).collect();
        let table = LinearInterpolate::new(&res_to_temp)?;
        Ok(Self { pullup, table })
    }

    pub fn calc_temp(&self, adc: f64) -> f64 {
        let adc = adc.max(0.00001).min(0.99999);
        let r = self.pullup * adc / (1.0 - adc);
        self.table.interpolate(r)
    }

    pub fn calc_adc(&self, temp: f64) -> f64 {
        let r = self.table.reverse_interpolate(temp);
        r / (self.pullup + r)
    }
}

// ===========================================================================
// Default sensor tables
// ===========================================================================

// Voltage sensors: AD595, AD597, AD8494-7, PT100 INA826
// Resistance sensors: PT1000

const AD595: &[(f64, f64)] = &[
    (0., 0.0027),
    (10., 0.101),
    (20., 0.200),
    (25., 0.250),
    (30., 0.300),
    (40., 0.401),
    (50., 0.503),
    (60., 0.605),
    (80., 0.810),
    (100., 1.015),
    (120., 1.219),
    (140., 1.420),
    (160., 1.620),
    (180., 1.817),
    (200., 2.015),
    (220., 2.213),
    (240., 2.413),
    (260., 2.614),
    (280., 2.817),
    (300., 3.022),
    (320., 3.227),
    (340., 3.434),
    (360., 3.641),
    (380., 3.849),
    (400., 4.057),
    (420., 4.266),
    (440., 4.476),
    (460., 4.686),
    (480., 4.896),
];

const AD597: &[(f64, f64)] = &[
    (0., 0.),
    (10., 0.097),
    (20., 0.196),
    (25., 0.245),
    (30., 0.295),
    (40., 0.395),
    (50., 0.496),
    (60., 0.598),
    (80., 0.802),
    (100., 1.005),
    (120., 1.207),
    (140., 1.407),
    (160., 1.605),
    (180., 1.801),
    (200., 1.997),
    (220., 2.194),
    (240., 2.392),
    (260., 2.592),
    (280., 2.794),
    (300., 2.996),
    (320., 3.201),
    (340., 3.406),
    (360., 3.611),
    (380., 3.817),
    (400., 4.024),
    (420., 4.232),
    (440., 4.440),
    (460., 4.649),
    (480., 4.857),
    (500., 5.066),
];

const AD8494: &[(f64, f64)] = &[
    (-180., -0.714),
    (-160., -0.658),
    (-140., -0.594),
    (-120., -0.523),
    (-100., -0.446),
    (-80., -0.365),
    (-60., -0.278),
    (-40., -0.188),
    (-20., -0.095),
    (0., 0.002),
    (20., 0.1),
    (25., 0.125),
    (40., 0.201),
    (60., 0.303),
    (80., 0.406),
    (100., 0.511),
    (120., 0.617),
    (140., 0.723),
    (160., 0.829),
    (180., 0.937),
    (200., 1.044),
    (220., 1.151),
    (240., 1.259),
    (260., 1.366),
    (280., 1.473),
    (300., 1.58),
    (320., 1.687),
    (340., 1.794),
    (360., 1.901),
    (380., 2.008),
    (400., 2.114),
    (420., 2.221),
    (440., 2.328),
    (460., 2.435),
    (480., 2.542),
    (500., 2.65),
    (520., 2.759),
    (540., 2.868),
    (560., 2.979),
    (580., 3.09),
    (600., 3.203),
    (620., 3.316),
    (640., 3.431),
    (660., 3.548),
    (680., 3.666),
    (700., 3.786),
    (720., 3.906),
    (740., 4.029),
    (760., 4.152),
    (780., 4.276),
    (800., 4.401),
    (820., 4.526),
    (840., 4.65),
    (860., 4.774),
    (880., 4.897),
    (900., 5.018),
    (920., 5.138),
    (940., 5.257),
    (960., 5.374),
    (980., 5.49),
    (1000., 5.606),
    (1020., 5.72),
    (1040., 5.833),
    (1060., 5.946),
    (1080., 6.058),
    (1100., 6.17),
    (1120., 6.282),
    (1140., 6.394),
    (1160., 6.505),
    (1180., 6.616),
    (1200., 6.727),
];

const AD8495: &[(f64, f64)] = &[
    (-260., -0.786),
    (-240., -0.774),
    (-220., -0.751),
    (-200., -0.719),
    (-180., -0.677),
    (-160., -0.627),
    (-140., -0.569),
    (-120., -0.504),
    (-100., -0.432),
    (-80., -0.355),
    (-60., -0.272),
    (-40., -0.184),
    (-20., -0.093),
    (0., 0.003),
    (20., 0.1),
    (25., 0.125),
    (40., 0.2),
    (60., 0.301),
    (80., 0.402),
    (100., 0.504),
    (120., 0.605),
    (140., 0.705),
    (160., 0.803),
    (180., 0.901),
    (200., 0.999),
    (220., 1.097),
    (240., 1.196),
    (260., 1.295),
    (280., 1.396),
    (300., 1.497),
    (320., 1.599),
    (340., 1.701),
    (360., 1.803),
    (380., 1.906),
    (400., 2.01),
    (420., 2.113),
    (440., 2.217),
    (460., 2.321),
    (480., 2.425),
    (500., 2.529),
    (520., 2.634),
    (540., 2.738),
    (560., 2.843),
    (580., 2.947),
    (600., 3.051),
    (620., 3.155),
    (640., 3.259),
    (660., 3.362),
    (680., 3.465),
    (700., 3.568),
    (720., 3.67),
    (740., 3.772),
    (760., 3.874),
    (780., 3.975),
    (800., 4.076),
    (820., 4.176),
    (840., 4.275),
    (860., 4.374),
    (880., 4.473),
    (900., 4.571),
    (920., 4.669),
    (940., 4.766),
    (960., 4.863),
    (980., 4.959),
    (1000., 5.055),
    (1020., 5.15),
    (1040., 5.245),
    (1060., 5.339),
    (1080., 5.432),
    (1100., 5.525),
    (1120., 5.617),
    (1140., 5.709),
    (1160., 5.8),
    (1180., 5.891),
    (1200., 5.98),
    (1220., 6.069),
    (1240., 6.158),
    (1260., 6.245),
    (1280., 6.332),
    (1300., 6.418),
    (1320., 6.503),
    (1340., 6.587),
    (1360., 6.671),
    (1380., 6.754),
];

const AD8496: &[(f64, f64)] = &[
    (-180., -0.642),
    (-160., -0.59),
    (-140., -0.53),
    (-120., -0.464),
    (-100., -0.392),
    (-80., -0.315),
    (-60., -0.235),
    (-40., -0.15),
    (-20., -0.063),
    (0., 0.027),
    (20., 0.119),
    (25., 0.142),
    (40., 0.213),
    (60., 0.308),
    (80., 0.405),
    (100., 0.503),
    (120., 0.601),
    (140., 0.701),
    (160., 0.8),
    (180., 0.9),
    (200., 1.001),
    (220., 1.101),
    (240., 1.201),
    (260., 1.302),
    (280., 1.402),
    (300., 1.502),
    (320., 1.602),
    (340., 1.702),
    (360., 1.801),
    (380., 1.901),
    (400., 2.001),
    (420., 2.1),
    (440., 2.2),
    (460., 2.3),
    (480., 2.401),
    (500., 2.502),
    (520., 2.603),
    (540., 2.705),
    (560., 2.808),
    (580., 2.912),
    (600., 3.017),
    (620., 3.124),
    (640., 3.231),
    (660., 3.34),
    (680., 3.451),
    (700., 3.562),
    (720., 3.675),
    (740., 3.789),
    (760., 3.904),
    (780., 4.02),
    (800., 4.137),
    (820., 4.254),
    (840., 4.37),
    (860., 4.486),
    (880., 4.6),
    (900., 4.714),
    (920., 4.826),
    (940., 4.937),
    (960., 5.047),
    (980., 5.155),
    (1000., 5.263),
    (1020., 5.369),
    (1040., 5.475),
    (1060., 5.581),
    (1080., 5.686),
    (1100., 5.79),
    (1120., 5.895),
    (1140., 5.999),
    (1160., 6.103),
    (1180., 6.207),
    (1200., 6.311),
];

const AD8497: &[(f64, f64)] = &[
    (-260., -0.785),
    (-240., -0.773),
    (-220., -0.751),
    (-200., -0.718),
    (-180., -0.676),
    (-160., -0.626),
    (-140., -0.568),
    (-120., -0.503),
    (-100., -0.432),
    (-80., -0.354),
    (-60., -0.271),
    (-40., -0.184),
    (-20., -0.092),
    (0., 0.003),
    (20., 0.101),
    (25., 0.126),
    (40., 0.2),
    (60., 0.301),
    (80., 0.403),
    (100., 0.505),
    (120., 0.605),
    (140., 0.705),
    (160., 0.804),
    (180., 0.902),
    (200., 0.999),
    (220., 1.097),
    (240., 1.196),
    (260., 1.296),
    (280., 1.396),
    (300., 1.498),
    (320., 1.599),
    (340., 1.701),
    (360., 1.804),
    (380., 1.907),
    (400., 2.01),
    (420., 2.114),
    (440., 2.218),
    (460., 2.322),
    (480., 2.426),
    (500., 2.53),
    (520., 2.634),
    (540., 2.739),
    (560., 2.843),
    (580., 2.948),
    (600., 3.052),
    (620., 3.156),
    (640., 3.259),
    (660., 3.363),
    (680., 3.466),
    (700., 3.569),
    (720., 3.671),
    (740., 3.773),
    (760., 3.874),
    (780., 3.976),
    (800., 4.076),
    (820., 4.176),
    (840., 4.276),
    (860., 4.375),
    (880., 4.474),
    (900., 4.572),
    (920., 4.67),
    (940., 4.767),
    (960., 4.863),
    (980., 4.96),
    (1000., 5.055),
    (1020., 5.151),
    (1040., 5.245),
    (1060., 5.339),
    (1080., 5.433),
    (1100., 5.526),
    (1120., 5.618),
    (1140., 5.71),
    (1160., 5.801),
    (1180., 5.891),
    (1200., 5.981),
    (1220., 6.07),
    (1240., 6.158),
    (1260., 6.246),
    (1280., 6.332),
    (1300., 6.418),
    (1320., 6.503),
    (1340., 6.588),
    (1360., 6.671),
    (1380., 6.754),
];

// PT100 resistance using Callendar-Van Dusen formula (base = 100 Ω)
fn calc_pt100(base: f64) -> Vec<(f64, f64)> {
    let a = 3.9083e-3;
    let b = -5.775e-7;
    (0..=500)
        .step_by(10)
        .map(|t| {
            (
                t as f64,
                base * (1.0 + a * t as f64 + b * t as f64 * t as f64),
            )
        })
        .collect()
}

// PT100 with INA826 amplifier: 4400 Ω pullup, 10× gain, 5 V reference
fn calc_ina826_pt100() -> Vec<(f64, f64)> {
    let pt100 = calc_pt100(100.);
    pt100
        .iter()
        .map(|&(t, r)| (t, 10.0 * 5.0 * r / (4400.0 + r)))
        .collect()
}

// PT1000 resistance using Callendar-Van Dusen formula (base = 1000 Ω)
fn calc_pt1000() -> Vec<(f64, f64)> {
    calc_pt100(1000.)
}

// Default voltage-based sensors: (name, calibration_data)
const DEFAULT_VOLTAGE_SENSORS: &[(&str, &[(f64, f64)])] = &[
    ("AD595", AD595),
    ("AD597", AD597),
    ("AD8494", AD8494),
    ("AD8495", AD8495),
    ("AD8496", AD8496),
    ("AD8497", AD8497),
];

// Default resistance-based sensors: PT1000 is computed at runtime.

/// Get the computed PT100 INA826 calibration data.
pub fn get_pt100_ina826_data() -> Vec<(f64, f64)> {
    calc_ina826_pt100()
}

/// Get the computed PT1000 calibration data.
pub fn get_pt1000_data() -> Vec<(f64, f64)> {
    calc_pt1000()
}

// ===========================================================================
// Convert trait and PrinterADCtoTemperature
// ===========================================================================

/// The conversion trait: ADC fraction ↔ temperature.
pub trait Convert: Send + Sync {
    fn calc_temp(&self, adc: f64) -> f64;
    fn calc_adc(&self, temp: f64) -> f64;
}

impl<C: Convert> Convert for Arc<C> {
    fn calc_temp(&self, adc: f64) -> f64 {
        (**self).calc_temp(adc)
    }
    fn calc_adc(&self, temp: f64) -> f64 {
        (**self).calc_adc(temp)
    }
}

const SAMPLE_TIME: f64 = 0.001;
const SAMPLE_COUNT: u32 = 8;
const REPORT_TIME: f64 = 0.300;
const RANGE_CHECK_COUNT: u32 = 4;

/// Build the ADC resource a sensor section names with `sensor_pin`.
fn sensor_adc(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<dyn Adc>, ConfigError> {
    let pin = config.get("sensor_pin", None)?;
    let pins = printer
        .lookup_object_as::<PrinterPins>(PINS_OBJECT)
        .ok_or_else(|| ConfigError::new("sensor_pin: the pins object is not registered"))?;
    pins.setup_adc(&pin, None)
        .map_err(|err| ConfigError::new(format!("sensor_pin: {err}")))
}

/// Bridge between MCU ADC and heater temperature callback.
///
/// Upstream's `PrinterADCtoTemperature` (`klippy/extras/adc_temperature.py:17`):
/// it owns the ADC resource for the section's `sensor_pin`, converts each report
/// with the [`Convert`] it was given, and forwards the temperature to the
/// heater's callback.
pub struct AdcTemperatureBridge<C: Convert> {
    convert: C,
    adc: Arc<dyn Adc>,
    callback: Mutex<Option<SensorCallback>>,
}

impl<C: Convert + 'static> AdcTemperatureBridge<C> {
    /// Create a bridge on `adc` and install its report callback.
    pub fn new(convert: C, adc: Arc<dyn Adc>) -> Arc<Self> {
        let bridge = Arc::new(Self {
            convert,
            adc: Arc::clone(&adc),
            callback: Mutex::new(None),
        });
        // Weak, not Arc: the resource owns the callback and the bridge owns the
        // resource, so a strong handle would close that cycle. The printer object
        // holding the bridge is what keeps it alive.
        let weak = Arc::downgrade(&bridge);
        adc.setup_adc_callback(Box::new(move |samples| {
            if let Some(bridge) = weak.upgrade() {
                bridge.handle_adc_report(samples);
            }
        }));
        bridge
    }

    /// Handle one ADC report and forward the temperature via the stored callback.
    pub fn handle_adc_report(&self, samples: &[(u64, f64)]) {
        let (read_time, read_value) = samples[samples.len() - 1];
        let temp = self.convert.calc_temp(read_value);
        let time = (read_time as f64) + f64::from(SAMPLE_COUNT) * SAMPLE_TIME;
        if let Some(cb) = self
            .callback
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .as_mut()
        {
            cb(time, temp);
        }
    }
}

impl<C: Convert + 'static> Sensor for AdcTemperatureBridge<C> {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) {
        let arange = [
            self.convert.calc_adc(min_temp),
            self.convert.calc_adc(max_temp),
        ];
        let (min_adc, max_adc) = if arange[0] < arange[1] {
            (arange[0], arange[1])
        } else {
            (arange[1], arange[0])
        };
        self.adc.setup_adc_sample(
            REPORT_TIME,
            SAMPLE_TIME,
            SAMPLE_COUNT,
            1,
            min_adc,
            max_adc,
            RANGE_CHECK_COUNT,
        );
    }

    fn setup_callback(&self, callback: SensorCallback) {
        *self.callback.lock().unwrap_or_else(|p| p.into_inner()) = Some(callback);
    }
}

impl<C: Convert> std::fmt::Debug for AdcTemperatureBridge<C> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AdcTemperatureBridge").finish()
    }
}

// ===========================================================================
// Sensor factories for built-in voltage sensors
// ===========================================================================

/// Build a sensor factory for a voltage sensor.
pub fn voltage_sensor_factory(
    params: &[(f64, f64)],
) -> impl Fn(&ConfigWrapper, &Arc<Printer>) -> Result<Arc<dyn Sensor>, ConfigError> {
    let params = params.to_vec();
    move |config: &ConfigWrapper, printer: &Arc<Printer>| {
        let adc = sensor_adc(config, printer)?;
        let lv = LinearVoltage::new(config, &params)?;
        Ok(AdcTemperatureBridge::new(lv, adc) as Arc<dyn Sensor>)
    }
}

/// Build a sensor factory for a resistance sensor.
pub fn resistance_sensor_factory(
    samples: Vec<(f64, f64)>,
) -> impl Fn(&ConfigWrapper, &Arc<Printer>) -> Result<Arc<dyn Sensor>, ConfigError> {
    let samples = samples.clone();
    move |config: &ConfigWrapper, printer: &Arc<Printer>| {
        let adc = sensor_adc(config, printer)?;
        let lr = LinearResistance::new(config, &samples)?;
        Ok(AdcTemperatureBridge::new(lr, adc) as Arc<dyn Sensor>)
    }
}

// ===========================================================================
// Thermistor — Steinhart-Hart / Beta model
// ===========================================================================

/// Thermistor resistance-to-temperature converter.
pub struct Thermistor {
    pullup: f64,
    inline_resistor: f64,
    c1: f64,
    c2: f64,
    c3: f64,
}

const KELVIN_TO_CELSIUS: f64 = -273.15;

impl Thermistor {
    pub fn new(pullup: f64, inline_resistor: f64) -> Self {
        Self {
            pullup,
            inline_resistor,
            c1: 0.0,
            c2: 0.0,
            c3: 0.0,
        }
    }

    pub fn setup_coefficients(&mut self, t1: f64, r1: f64, t2: f64, r2: f64, t3: f64, r3: f64) {
        let inv_t1 = 1.0 / (t1 - KELVIN_TO_CELSIUS);
        let inv_t2 = 1.0 / (t2 - KELVIN_TO_CELSIUS);
        let inv_t3 = 1.0 / (t3 - KELVIN_TO_CELSIUS);
        let ln_r1 = r1.ln();
        let ln_r2 = r2.ln();
        let ln_r3 = r3.ln();
        let ln3_r1 = ln_r1 * ln_r1 * ln_r1;
        let ln3_r2 = ln_r2 * ln_r2 * ln_r2;
        let ln3_r3 = ln_r3 * ln_r3 * ln_r3;

        let inv_t12 = inv_t1 - inv_t2;
        let inv_t13 = inv_t1 - inv_t3;
        let ln_r12 = ln_r1 - ln_r2;
        let ln_r13 = ln_r1 - ln_r3;
        let ln3_r12 = ln3_r1 - ln3_r2;
        let ln3_r13 = ln3_r1 - ln3_r3;

        self.c3 = (inv_t12 - inv_t13 * ln_r12 / ln_r13) / (ln3_r12 - ln3_r13 * ln_r12 / ln_r13);
        if self.c3 <= 0.0 {
            let beta = ln_r13 / inv_t13;
            self.setup_coefficients_beta(t1, r1, beta);
            return;
        }
        self.c2 = (inv_t12 - self.c3 * ln3_r12) / ln_r12;
        self.c1 = inv_t1 - self.c2 * ln_r1 - self.c3 * ln3_r1;
    }

    pub fn setup_coefficients_beta(&mut self, t1: f64, r1: f64, beta: f64) {
        let inv_t1 = 1.0 / (t1 - KELVIN_TO_CELSIUS);
        let ln_r1 = r1.ln();
        self.c3 = 0.0;
        self.c2 = 1.0 / beta;
        self.c1 = inv_t1 - self.c2 * ln_r1;
    }

    pub fn calc_temp(&self, adc: f64) -> f64 {
        let adc = adc.max(0.00001).min(0.99999);
        let r = self.pullup * adc / (1.0 - adc);
        let ln_r = (r - self.inline_resistor).ln();
        let inv_t = self.c1 + self.c2 * ln_r + self.c3 * ln_r * ln_r * ln_r;
        1.0 / inv_t + KELVIN_TO_CELSIUS
    }

    pub fn calc_adc(&self, temp: f64) -> f64 {
        if temp <= KELVIN_TO_CELSIUS {
            return 1.0;
        }
        let inv_t = 1.0 / (temp - KELVIN_TO_CELSIUS);
        let ln_r = if self.c3 != 0.0 {
            let y = (self.c1 - inv_t) / (2.0 * self.c3);
            let x = ((self.c2 / (3.0 * self.c3)).powi(3) + y * y).sqrt();
            (x - y).powf(1.0 / 3.0) - (x + y).powf(1.0 / 3.0)
        } else {
            (inv_t - self.c1) / self.c2
        };
        let r = ln_r.exp() + self.inline_resistor;
        r / (self.pullup + r)
    }
}

impl Convert for Thermistor {
    fn calc_temp(&self, adc: f64) -> f64 {
        self.calc_temp(adc)
    }
    fn calc_adc(&self, temp: f64) -> f64 {
        self.calc_adc(temp)
    }
}

impl Convert for LinearVoltage {
    fn calc_temp(&self, adc: f64) -> f64 {
        self.calc_temp(adc)
    }
    fn calc_adc(&self, temp: f64) -> f64 {
        self.calc_adc(temp)
    }
}

impl Convert for LinearResistance {
    fn calc_temp(&self, adc: f64) -> f64 {
        self.calc_temp(adc)
    }
    fn calc_adc(&self, temp: f64) -> f64 {
        self.calc_adc(temp)
    }
}

/// Create a thermistor sensor from params.
pub fn thermistor_sensor(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
    params: &[(f64, f64, f64)], // (t, r, beta_or_0)
) -> Result<Arc<dyn Sensor>, ConfigError> {
    let adc = sensor_adc(config, printer)?;
    let pullup =
        config.get_float_bounded("pullup_resistor", Some(4700.0), Some(0.0), None, None, None)?;
    let inline_resistor =
        config.get_float_bounded("inline_resistor", Some(0.0), Some(0.0), None, None, None)?;

    let mut thermistor = Thermistor::new(pullup, inline_resistor);

    if params.len() == 1 && params[0].2 > 0.0 {
        // Beta parameter form: (t1, r1, beta)
        thermistor.setup_coefficients_beta(params[0].0, params[0].1, params[0].2);
    } else if params.len() >= 3 {
        // Three-point Steinhart-Hart: (t1, r1), (t2, r2), (t3, r3)
        let (t1, r1, _) = params[0];
        let (t2, r2, _) = params[1];
        let (t3, r3, _) = params[2];
        thermistor.setup_coefficients(t1, r1, t2, r2, t3, r3);
    } else {
        return Err(ConfigError::new(format!(
            "adc_temperature {} in heater {}: need at least two samples",
            config.section().sub.clone().unwrap_or_default(),
            config.identifier()
        )));
    }

    Ok(AdcTemperatureBridge::new(thermistor, adc) as Arc<dyn Sensor>)
}

// ===========================================================================
// Custom thermistor from config
// ===========================================================================

/// Parse a custom thermistor section like `[thermistor MyName]`.
pub struct CustomThermistor {
    /// Sensor name extracted from config section (e.g., "MyName" from `[thermistor MyName]`).
    #[allow(dead_code)]
    name: String,
    params: Vec<(f64, f64, f64)>, // (temp, resistance, beta)
}

impl CustomThermistor {
    /// Read temperature/resistance points from config.
    pub fn new(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        // The sensor name is the section's sub-name (`[thermistor MyName]` →
        // `MyName`), which is what a consumer writes as `sensor_type`.
        let name = config.section().sub.clone().unwrap_or_default();

        let mut params: Vec<(f64, f64, f64)> = Vec::new();
        let mut i = 1;
        loop {
            let temp_key = format!("temperature{}", i);
            let resistance_key = format!("resistance{}", i);

            let temp = match config.get_optional_float(&temp_key)? {
                Some(t) => t,
                None => break,
            };
            let resistance =
                config.get_float_bounded(&resistance_key, None, Some(0.0), None, None, None)?;

            let beta = if i == 1 {
                config.get_optional_float("beta")?.unwrap_or(0.0)
            } else {
                0.0
            };

            params.push((temp, resistance, beta));
            i += 1;
        }

        Ok(Self { name, params })
    }

    /// Create a sensor from this custom thermistor's config.
    pub fn create(
        &self,
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
    ) -> Result<Arc<dyn Sensor>, ConfigError> {
        thermistor_sensor(config, printer, &self.params)
    }
}

// ===========================================================================
// Custom linear voltage/resistance from config
// ===========================================================================

/// Custom linear voltage sensor from config: `[adc_temperature <name>]` with voltage points.
pub struct CustomLinearVoltage {
    /// Sensor name extracted from config section.
    #[allow(dead_code)]
    name: String,
    params: Vec<(f64, f64)>, // (temp, voltage)
}

impl CustomLinearVoltage {
    pub fn new(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let name = config.section().sub.clone().unwrap_or_default();

        let mut params: Vec<(f64, f64)> = Vec::new();
        let mut i = 1;
        loop {
            let temp_key = format!("temperature{}", i);
            let voltage_key = format!("voltage{}", i);

            let temp = match config.get_optional_float(&temp_key)? {
                Some(t) => t,
                None => break,
            };
            let voltage = config.get_float_bounded(&voltage_key, None, None, None, None, None)?;

            params.push((temp, voltage));
            i += 1;
        }

        Ok(Self { name, params })
    }

    pub fn create(
        &self,
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
    ) -> Result<Arc<dyn Sensor>, ConfigError> {
        let adc = sensor_adc(config, printer)?;
        let lv = LinearVoltage::new(config, &self.params)?;
        Ok(AdcTemperatureBridge::new(lv, adc) as Arc<dyn Sensor>)
    }
}

/// Custom linear resistance sensor from config: `[adc_temperature <name>]` with resistance points.
pub struct CustomLinearResistance {
    /// Sensor name extracted from config section.
    #[allow(dead_code)]
    name: String,
    samples: Vec<(f64, f64)>, // (temp, resistance)
}

impl CustomLinearResistance {
    pub fn new(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let name = config.section().sub.clone().unwrap_or_default();

        let mut samples: Vec<(f64, f64)> = Vec::new();
        let mut i = 1;
        loop {
            let temp_key = format!("temperature{}", i);
            let resistance_key = format!("resistance{}", i);

            let temp = match config.get_optional_float(&temp_key)? {
                Some(t) => t,
                None => break,
            };
            let resistance =
                config.get_float_bounded(&resistance_key, None, Some(0.0), None, None, None)?;

            samples.push((temp, resistance));
            i += 1;
        }

        Ok(Self { name, samples })
    }

    pub fn create(
        &self,
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
    ) -> Result<Arc<dyn Sensor>, ConfigError> {
        let adc = sensor_adc(config, printer)?;
        let lr = LinearResistance::new(config, &self.samples)?;
        Ok(AdcTemperatureBridge::new(lr, adc) as Arc<dyn Sensor>)
    }
}

// ===========================================================================
// Register built-in sensors with heaters
// ===========================================================================

/// The named thermistors upstream ships in `temperature_sensors.cfg`.
///
/// Each `[thermistor <name>]` section there registers a sensor factory under its
/// sub-name; we have no config file to read at runtime, so the table is here.
/// The tuples are `(temperature, resistance, beta_or_0)`; a lone entry with a
/// non-zero beta is the beta form, three entries are Steinhart-Hart.
const BUILTIN_THERMISTORS: &[(&str, &[(f64, f64, f64)])] = &[
    (
        "ATC Semitec 104GT-2",
        &[
            (20.0, 126800.0, 0.0),
            (150.0, 1360.0, 0.0),
            (300.0, 80.65, 0.0),
        ],
    ),
    (
        "ATC Semitec 104NT-4-R025H42G",
        &[
            (25.0, 100000.0, 0.0),
            (160.0, 1074.0, 0.0),
            (300.0, 82.78, 0.0),
        ],
    ),
    (
        "EPCOS 100K B57560G104F",
        &[
            (25.0, 100000.0, 0.0),
            (150.0, 1641.9, 0.0),
            (250.0, 226.15, 0.0),
        ],
    ),
    (
        "Generic 3950",
        &[
            (25.0, 100000.0, 0.0),
            (150.0, 1770.0, 0.0),
            (250.0, 230.0, 0.0),
        ],
    ),
    (
        "SliceEngineering 450",
        &[
            (25.0, 500000.0, 0.0),
            (200.0, 3734.0, 0.0),
            (400.0, 240.0, 0.0),
        ],
    ),
    (
        "TDK NTCG104LH104JT1",
        &[
            (25.0, 100000.0, 0.0),
            (50.0, 31230.0, 0.0),
            (125.0, 2066.0, 0.0),
        ],
    ),
    ("Honeywell 100K 135-104LAG-J01", &[(25.0, 100000.0, 3974.0)]),
    ("NTC 100K MGB18-104F39050L32", &[(25.0, 100000.0, 4100.0)]),
];

/// Register all built-in ADC temperature sensors with the heaters registry.
///
/// Upstream's `adc_temperature.load_config`: registers the default voltage and
/// resistance sensors. The named thermistors come from `temperature_sensors.cfg`
/// upstream; here they are [`BUILTIN_THERMISTORS`].
pub fn ensure(heaters: &Arc<PrinterHeaters>) -> Result<(), ConfigError> {
    for &(name, params) in DEFAULT_VOLTAGE_SENSORS {
        heaters.add_sensor_factory(name, Arc::new(voltage_sensor_factory(params)));
    }
    heaters.add_sensor_factory(
        "PT1000",
        Arc::new(resistance_sensor_factory(get_pt1000_data())),
    );
    heaters.add_sensor_factory(
        "PT100 INA826",
        Arc::new(resistance_sensor_factory(get_pt100_ina826_data())),
    );
    for &(name, params) in BUILTIN_THERMISTORS {
        let params = params.to_vec();
        let factory = Arc::new(move |config: &ConfigWrapper, printer: &Arc<Printer>| {
            thermistor_sensor(config, printer, &params)
        });
        heaters.add_sensor_factory(name, factory);
    }
    Ok(())
}

/// A sensor-definition section's printer object.
///
/// `[thermistor <name>]` and `[adc_temperature <name>]` produce a **sensor
/// factory**, not a queryable object (upstream's `load_config_prefix` returns
/// `None`). This placeholder claims the section so the loader is satisfied and
/// keeps it out of `objects/list`.
struct SensorSection;

impl PrinterObject for SensorSection {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

/// `[thermistor <name>]`: register a sensor factory under the sub-name.
pub fn load_thermistor_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let heaters = heaters::ensure(printer)?;
    let custom = CustomThermistor::new(config)?;
    let name = custom.name.clone();
    let factory = Arc::new(move |config: &ConfigWrapper, printer: &Arc<Printer>| {
        custom.create(config, printer)
    });
    heaters.add_sensor_factory(&name, factory);
    Ok(Arc::new(SensorSection))
}

/// Bare `[adc_temperature]`: upstream loads the module's defaults here.
pub fn load_config(
    _config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    heaters::ensure(printer)?;
    Ok(Arc::new(SensorSection))
}

/// `[adc_temperature <name>]`: a custom linear voltage or resistance sensor.
pub fn load_adc_temperature_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let heaters = heaters::ensure(printer)?;
    // Upstream chooses by whether `resistance1` is present.
    if config.get_optional_float("resistance1")?.is_some() {
        let custom = CustomLinearResistance::new(config)?;
        let name = custom.name.clone();
        let factory = Arc::new(move |config: &ConfigWrapper, printer: &Arc<Printer>| {
            custom.create(config, printer)
        });
        heaters.add_sensor_factory(&name, factory);
    } else {
        let custom = CustomLinearVoltage::new(config)?;
        let name = custom.name.clone();
        let factory = Arc::new(move |config: &ConfigWrapper, printer: &Arc<Printer>| {
            custom.create(config, printer)
        });
        heaters.add_sensor_factory(&name, factory);
    }
    Ok(Arc::new(SensorSection))
}

// The sections the loader must know about. `thermistor` is prefix-only; the bare
// `[adc_temperature]` is upstream's "load the defaults" switch.
section!("thermistor", order = 25, prefix = load_thermistor_prefix);
section!(
    "adc_temperature",
    order = 25,
    load = load_config,
    prefix = load_adc_temperature_prefix
);

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_linear_interpolate_forward() {
        let samples = vec![(0.0, 0.0), (0.5, 50.0), (1.0, 100.0)];
        let li = LinearInterpolate::new(&samples).unwrap();

        assert!((li.interpolate(0.0) - 0.0).abs() < f64::EPSILON);
        assert!((li.interpolate(0.5) - 50.0).abs() < f64::EPSILON);
        assert!((li.interpolate(1.0) - 100.0).abs() < f64::EPSILON);
        assert!((li.interpolate(0.25) - 25.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_linear_interpolate_reverse() {
        let samples = vec![(0.0, 0.0), (0.5, 50.0), (1.0, 100.0)];
        let li = LinearInterpolate::new(&samples).unwrap();

        assert!((li.reverse_interpolate(0.0) - 0.0).abs() < f64::EPSILON);
        assert!((li.reverse_interpolate(50.0) - 0.5).abs() < f64::EPSILON);
        assert!((li.reverse_interpolate(100.0) - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_thermistor_steinhart_hart() {
        let mut t = Thermistor::new(4700.0, 0.0);
        // EPCOS 100K B57560G104F: 25°C=100K, 150°C=1641.9, 250°C=226.15
        t.setup_coefficients(25.0, 100000.0, 150.0, 1641.9, 250.0, 226.15);

        // At ADC=0.5, resistance = 4700 * 0.5 / 0.5 = 4700
        let temp = t.calc_temp(0.5);
        // 4700Ω should be somewhere between 25°C and 150°C
        assert!(temp > 25.0 && temp < 150.0, "temp={}", temp);
    }

    #[test]
    fn test_thermistor_beta_model() {
        let mut t = Thermistor::new(4700.0, 0.0);
        // Honeywell 100K: 25°C=100K, beta=3974
        t.setup_coefficients_beta(25.0, 100000.0, 3974.0);

        let temp = t.calc_temp(0.5);
        assert!(temp > 25.0 && temp < 150.0, "temp={}", temp);
    }
}
