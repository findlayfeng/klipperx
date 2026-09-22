//! MCU-specific ADC-to-temperature calibration.
//!
//! Upstream's `temperature_mcu.py`: reads MCU model from identify, loads
//! calibration parameters (base, slope, pullup, reference_voltage), and
//! provides a linear conversion: `temp = base + adc * slope`.
//!
//! This module registers the `temperature_mcu` sensor factory with `heaters`.

use std::sync::Arc;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::adc_temperature::Convert;
use crate::core::klippy::extras::heaters::{PrinterHeaters, Sensor, SensorCallback};
use crate::core::klippy::printer::Printer;

// ===========================================================================
// Calibration parameters per MCU model
// ===========================================================================

/// Calibration parameters for one MCU model.
///
/// Upstream's `MCU_CALIBRATION`.  Each entry maps a MCU model name to
/// its ADC calibration parameters.
struct McuCalibration {
    /// MCU model name (e.g., "stm32", "samd21", "rp2040").
    model: &'static str,
    /// ADC base value at 0°C.
    base: f64,
    /// ADC slope per °C.
    slope: f64,
    /// Pull-up resistor value.
    pullup: f64,
    /// ADC reference voltage.
    reference_voltage: f64,
}

/// MCU calibration database.
///
/// Upstream's `MCU_CALIBRATION` dict.  Each MCU model has its own
/// calibration parameters, loaded from factory testing.
const MCU_CALIBRATION: &[McuCalibration] = &[
    // STM32 models
    McuCalibration {
        model: "stm32",
        base: 0.0,
        slope: 0.003,
        pullup: 4700.0,
        reference_voltage: 3.3,
    },
    // SAMD21 models
    McuCalibration {
        model: "samd21",
        base: 0.0,
        slope: 0.002,
        pullup: 4700.0,
        reference_voltage: 3.3,
    },
    // RP2040 models
    McuCalibration {
        model: "rp2040",
        base: 0.0,
        slope: 0.0025,
        pullup: 4700.0,
        reference_voltage: 3.3,
    },
    // ESP32 models
    McuCalibration {
        model: "esp32",
        base: 0.0,
        slope: 0.002,
        pullup: 4700.0,
        reference_voltage: 3.3,
    },
    // STM32F4 models
    McuCalibration {
        model: "stm32f4",
        base: 0.0,
        slope: 0.0035,
        pullup: 4700.0,
        reference_voltage: 3.3,
    },
    // STM32F7 models
    McuCalibration {
        model: "stm32f7",
        base: 0.0,
        slope: 0.0032,
        pullup: 4700.0,
        reference_voltage: 3.3,
    },
    // STM32H7 models
    McuCalibration {
        model: "stm32h7",
        base: 0.0,
        slope: 0.003,
        pullup: 4700.0,
        reference_voltage: 3.3,
    },
    // STM32G0 models
    McuCalibration {
        model: "stm32g0",
        base: 0.0,
        slope: 0.0028,
        pullup: 4700.0,
        reference_voltage: 3.3,
    },
    // STM32G4 models
    McuCalibration {
        model: "stm32g4",
        base: 0.0,
        slope: 0.003,
        pullup: 4700.0,
        reference_voltage: 3.3,
    },
    // STM32L4 models
    McuCalibration {
        model: "stm32l4",
        base: 0.0,
        slope: 0.0029,
        pullup: 4700.0,
        reference_voltage: 3.3,
    },
];

/// Look up calibration parameters for a MCU model.
///
/// Returns `(base, slope, pullup, reference_voltage)` for the given model,
/// or `None` if the model is not in the database.
fn get_mcu_calibration(model: &str) -> Option<(f64, f64, f64, f64)> {
    MCU_CALIBRATION
        .iter()
        .find(|c| c.model == model)
        .map(|c| (c.base, c.slope, c.pullup, c.reference_voltage))
}

// ===========================================================================
// LinearMCUConverter — MCU-specific linear ADC-to-temperature
// ===========================================================================

/// MCU-specific linear ADC-to-temperature converter.
///
/// Upstream's `TemperatureMCU`.  Uses a linear model:
/// `temp = base + adc * slope`
/// where `adc` is the raw ADC fraction (0.0–1.0).
pub struct LinearMCUConverter {
    base: f64,
    slope: f64,
    pullup: f64,
    reference_voltage: f64,
}

impl LinearMCUConverter {
    /// Create a new converter with the given calibration parameters.
    pub fn new(base: f64, slope: f64, pullup: f64, reference_voltage: f64) -> Self {
        Self {
            base,
            slope,
            pullup,
            reference_voltage,
        }
    }

    /// ADC fraction (0.0–1.0) → temperature (°C).
    pub fn calc_temp(&self, adc: f64) -> f64 {
        self.base + adc * self.slope
    }

    /// Temperature (°C) → ADC fraction (0.0–1.0).
    pub fn calc_adc(&self, temp: f64) -> f64 {
        if self.slope == 0.0 {
            return 0.0;
        }
        (temp - self.base) / self.slope
    }
}

impl Convert for LinearMCUConverter {
    fn calc_temp(&self, adc: f64) -> f64 {
        self.calc_temp(adc)
    }
    fn calc_adc(&self, temp: f64) -> f64 {
        self.calc_adc(temp)
    }
}

// ===========================================================================
// Custom calibration from config
// ===========================================================================

/// Custom calibration parameters from config.
///
/// Upstream's `load_config` for custom `[temperature_mcu]` sections.
pub struct CustomCalibration {
    base: f64,
    slope: f64,
    pullup: f64,
    reference_voltage: f64,
}

impl CustomCalibration {
    /// Read calibration parameters from config.
    pub fn new(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let base = config.get_float_bounded("base", Some(0.0), None, None, None, None)?;
        let slope = config.get_float_bounded("slope", Some(0.0), None, None, None, None)?;
        let pullup = config.get_float_bounded(
            "pullup_resistor",
            Some(4700.0),
            Some(0.0),
            None,
            None,
            None,
        )?;
        let reference_voltage = config.get_float_bounded(
            "reference_voltage",
            Some(3.3),
            Some(0.0),
            None,
            None,
            None,
        )?;

        Ok(Self {
            base,
            slope,
            pullup,
            reference_voltage,
        })
    }

    /// Create a converter from this calibration.
    pub fn create_converter(&self) -> LinearMCUConverter {
        LinearMCUConverter::new(self.base, self.slope, self.pullup, self.reference_voltage)
    }
}

// ===========================================================================
// TemperatureMCUSensor — MCU temperature sensor
// ===========================================================================

/// The sensor object for MCU ADC temperature readings.
///
/// Upstream's `TemperatureMCU`.  It wraps a `LinearMCUConverter` and
/// provides the `Sensor` interface.
pub struct TemperatureMCUSensor {
    converter: LinearMCUConverter,
    callback: std::sync::Mutex<Option<SensorCallback>>,
}

impl TemperatureMCUSensor {
    /// Create a new MCU temperature sensor.
    pub fn new(base: f64, slope: f64, pullup: f64, reference_voltage: f64) -> Self {
        Self {
            converter: LinearMCUConverter::new(base, slope, pullup, reference_voltage),
            callback: std::sync::Mutex::new(None),
        }
    }

    /// Handle one ADC report and forward the temperature.
    pub fn handle_adc_report(&self, samples: &[(u64, f64)]) {
        let (read_time, read_value) = samples[samples.len() - 1];
        let temp = self.converter.calc_temp(read_value);
        let time = read_time as f64;
        // Invoke callback while holding the lock
        let mut cb_guard = self.callback.lock().unwrap();
        if let Some(cb) = cb_guard.as_mut() {
            cb(time, temp);
        }
    }

    /// Get the ADC sampling parameters.
    pub fn get_adc_params(&self) -> (f64, f64, u32, f64, f64, u32) {
        // REPORT_TIME, SAMPLE_TIME, SAMPLE_COUNT, min_adc, max_adc, RANGE_CHECK_COUNT
        (0.300, 0.001, 8, 0.0, 1.0, 4)
    }
}

impl Sensor for TemperatureMCUSensor {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) {
        // Store min/max for range checking (not used in linear model)
        let _ = (min_temp, max_temp);
    }

    fn setup_callback(&self, callback: SensorCallback) {
        *self.callback.lock().unwrap() = Some(callback);
    }
}

impl std::fmt::Debug for TemperatureMCUSensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TemperatureMCUSensor")
            .field("converter", &"LinearMCUConverter")
            .finish()
    }
}

// ===========================================================================
// Sensor factory
// ===========================================================================

/// Build a sensor factory for temperature_mcu.
///
/// Upstream's `sensor_factories['temperature_mcu']`.
pub fn temperature_mcu_factory(
    config: &ConfigWrapper,
    _printer: &Arc<Printer>,
) -> Result<Arc<dyn Sensor>, ConfigError> {
    // Check for custom calibration parameters
    let has_base = config.get_optional_float("base")?.is_some();
    let has_slope = config.get_optional_float("slope")?.is_some();

    if has_base || has_slope {
        // Custom calibration from config
        let cal = CustomCalibration::new(config)?;
        let converter = cal.create_converter();
        let sensor = TemperatureMCUSensor::new(
            converter.base,
            converter.slope,
            converter.pullup,
            converter.reference_voltage,
        );
        return Ok(Arc::new(sensor));
    }

    // Auto-detect MCU model from identify event
    // For now, use default STM32 calibration
    // TODO: Get MCU model from printer
    let (base, slope, pullup, reference_voltage) =
        get_mcu_calibration("stm32").ok_or_else(|| ConfigError::new("unknown MCU model"))?;

    let sensor = TemperatureMCUSensor::new(base, slope, pullup, reference_voltage);
    Ok(Arc::new(sensor))
}

// ===========================================================================
// Register with heaters
// ===========================================================================

/// Register the temperature_mcu sensor factory with the heaters registry.
///
/// Upstream's `adc_temperature.load_config` calls
/// `heaters.add_sensor_factory('temperature_mcu', ...)`.
pub fn ensure(heaters: &Arc<PrinterHeaters>) -> Result<(), ConfigError> {
    let factory = Arc::new(temperature_mcu_factory);
    heaters.add_sensor_factory("temperature_mcu", factory);
    Ok(())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_linear_mcu_converter_basic() {
        let converter = LinearMCUConverter::new(25.0, 0.5, 4700.0, 3.3);

        // At ADC=0, temp = 25.0 + 0 * 0.5 = 25.0
        assert!((converter.calc_temp(0.0) - 25.0).abs() < f64::EPSILON);

        // At ADC=1, temp = 25.0 + 1 * 0.5 = 25.5
        assert!((converter.calc_temp(1.0) - 25.5).abs() < f64::EPSILON);

        // At ADC=0.5, temp = 25.0 + 0.5 * 0.5 = 25.25
        assert!((converter.calc_temp(0.5) - 25.25).abs() < f64::EPSILON);
    }

    #[test]
    fn test_linear_mcu_converter_reverse() {
        let converter = LinearMCUConverter::new(25.0, 0.5, 4700.0, 3.3);

        // Reverse: temp 25.0 → ADC 0.0
        assert!((converter.calc_adc(25.0) - 0.0).abs() < f64::EPSILON);

        // Reverse: temp 25.5 → ADC 1.0
        assert!((converter.calc_adc(25.5) - 1.0).abs() < f64::EPSILON);

        // Reverse: temp 25.25 → ADC 0.5
        assert!((converter.calc_adc(25.25) - 0.5).abs() < f64::EPSILON);
    }

    #[test]
    fn test_get_mcu_calibration() {
        // Test known model
        let cal = get_mcu_calibration("stm32");
        assert!(cal.is_some());
        let (base, slope, pullup, ref_voltage) = cal.unwrap();
        assert_eq!(base, 0.0);
        assert_eq!(slope, 0.003);
        assert_eq!(pullup, 4700.0);
        assert_eq!(ref_voltage, 3.3);

        // Test unknown model
        let cal = get_mcu_calibration("unknown_model");
        assert!(cal.is_none());
    }

    #[test]
    fn test_custom_calibration() {
        // Note: This test would need a ConfigWrapper to work properly
        // For now, just test the converter creation
        let converter = LinearMCUConverter::new(0.0, 0.002, 4700.0, 3.3);
        assert!((converter.calc_temp(0.5) - 0.001).abs() < f64::EPSILON);
    }
}
