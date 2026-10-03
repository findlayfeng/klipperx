//! `temperature_mcu` — the MCU's own ADC temperature channel.
//!
//! Upstream's `temperature_mcu.py`: it sets up an ADC on the MCU's internal
//! temperature channel (`<mcu>:ADC_TEMPERATURE`), computes the chip-specific
//! linear calibration once the dictionary is known, and reports
//! `temp = base + adc * slope`.
//!
//! # Why the calibration is read in a pre-build callback
//!
//! Some MCUs (STM32F4/F0/G0/H7, SAMD) do not have a fixed calibration: their
//! factory values live in chip registers and are read over `debug_read`, a
//! request/response command (upstream reads them in `handle_mcu_identify`,
//! `klippy/extras/temperature_mcu.py:58-89`). The read has to happen **after**
//! identify (the dictionary must be installed) and **before** the configuration
//! is built (the ADC's sampling range depends on the calibration). That window
//! is exactly [`PreBuildCallback`](crate::core::klippy::mcu::ConfigBuilder), and
//! this module is its first user.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::core::klippy::cmd::debug::{DebugRead, DebugResult};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::heaters::{PrinterHeaters, Sensor, SensorCallback};
use crate::core::klippy::mcu::{ConfigBuilder, Mcu, McuError, McuObject};
use crate::core::klippy::pins::{Adc, PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::Printer;

/// Upstream's `temperature_mcu.py` constants.
const SAMPLE_TIME: f64 = 0.001;
const SAMPLE_COUNT: u32 = 8;
const REPORT_TIME: f64 = 0.300;
const RANGE_CHECK_COUNT: u32 = 4;
const KELVIN_TO_CELSIUS: f64 = -273.15;

/// How long one `debug_read` may take while calibrating.
const DEBUG_READ_TIMEOUT: Duration = Duration::from_secs(5);

/// ADC fraction ↔ temperature, as a line.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct McuCalibration {
    /// The temperature at ADC 0, from `temp - adc * slope`.
    pub base: f64,
    /// Degrees per ADC fraction.
    pub slope: f64,
}

impl McuCalibration {
    /// `temp = base + adc * slope`.
    pub fn calc_temp(&self, adc: f64) -> f64 {
        self.base + adc * self.slope
    }

    /// `adc = (temp - base) / slope`.
    pub fn calc_adc(&self, temp: f64) -> f64 {
        (temp - self.base) / self.slope
    }

    /// The line through `(adc, temp)` with the given slope.
    pub fn through(temp: f64, adc: f64, slope: f64) -> Self {
        Self {
            base: temp - adc * slope,
            slope,
        }
    }
}

// ===========================================================================
// Debug-memory reads
// ===========================================================================

/// One `debug_read` round-trip (`order`: 1 = 16-bit, 2 = 32-bit).
async fn debug_read(mcu: &Mcu, order: u8, addr: u32) -> Result<u32, McuError> {
    let result = mcu
        .call_msg::<DebugRead, DebugResult>(&DebugRead { order, addr }, DEBUG_READ_TIMEOUT)
        .await?;
    Ok(result.val)
}

/// Upstream's `read16` (`klippy/extras/temperature_mcu.py:169-171`).
async fn read16(mcu: &Mcu, addr: u32) -> Result<f64, McuError> {
    Ok(f64::from(debug_read(mcu, 1, addr).await?))
}

/// Upstream's `read32` (`klippy/extras/temperature_mcu.py:172-174`).
async fn read32(mcu: &Mcu, addr: u32) -> Result<u32, McuError> {
    debug_read(mcu, 2, addr).await
}

// ===========================================================================
// Per-model calibration
// ===========================================================================

/// The calibration for `model` (the dictionary's `MCU` constant).
///
/// Ports upstream's `config_*` helpers (`klippy/extras/temperature_mcu.py:92-167`).
/// The model prefixes are tested in upstream's order, so e.g. `stm32f103xe`
/// matches `stm32f1` and not a shorter key.
///
/// # Errors
/// [`McuError::Config`] for a model with no known calibration, and whatever
/// `debug_read` reports for a model that needs one.
pub async fn calibration_for(model: &str, mcu: &Mcu) -> Result<McuCalibration, McuError> {
    let unknown = || McuError::Config(format!("MCU temperature not supported on {model}"));
    let model = model.to_lowercase();
    if model.starts_with("rp2") {
        // RP2040/RP2350.
        let slope = 3.3 / -0.001721;
        Ok(McuCalibration::through(27.0, 0.706 / 3.3, slope))
    } else if model.starts_with("sam3") {
        let slope = 3.3 / 0.002650;
        Ok(McuCalibration::through(27.0, 0.8 / 3.3, slope))
    } else if model.starts_with("sam4") {
        let slope = 3.3 / 0.004700;
        Ok(McuCalibration::through(27.0, 1.44 / 3.3, slope))
    } else if model.starts_with("same70") {
        let slope = 3.3 / 0.002330;
        Ok(McuCalibration::through(25.0, 0.72 / 3.3, slope))
    } else if model.starts_with("samd21") {
        samd_calibration(mcu, 0x0080_6030).await
    } else if model.starts_with("samd51") || model.starts_with("same5") {
        samd_calibration(mcu, 0x0080_0100).await
    } else if model.starts_with("stm32f1") {
        let slope = 3.3 / -0.004300;
        Ok(McuCalibration::through(25.0, 1.43 / 3.3, slope))
    } else if model.starts_with("stm32f2") {
        let slope = 3.3 / 0.002500;
        Ok(McuCalibration::through(25.0, 0.76 / 3.3, slope))
    } else if model.starts_with("stm32f4") {
        two_point(mcu, 0x1FFF_7A2C, 0x1FFF_7A2E, 4095.0).await
    } else if model.starts_with("stm32f042") || model.starts_with("stm32f072") {
        two_point(mcu, 0x1FFF_F7B8, 0x1FFF_F7C2, 4095.0).await
    } else if model.starts_with("stm32f070") {
        // Fixed slope with a single factory point.
        let slope = 3.3 / -0.004300;
        let adc_30 = read16(mcu, 0x1FFF_F7B8).await? / 4095.0;
        Ok(McuCalibration::through(30.0, adc_30, slope))
    } else if model.starts_with("stm32g0")
        || model.starts_with("stm32g4")
        || model.starts_with("stm32l4")
    {
        // Two factory points; the 3.0/3.3 factor is in the datasheet.
        let scale = 3.0 / (3.3 * 4095.0);
        let adc_30 = read16(mcu, 0x1FFF_75A8).await? * scale;
        let adc_130 = read16(mcu, 0x1FFF_75CA).await? * scale;
        Ok(two_point_calibration(30.0, adc_30, 130.0, adc_130))
    } else if model.starts_with("stm32h723") {
        two_point(mcu, 0x1FF1_E820, 0x1FF1_E840, 4095.0).await
    } else if model.starts_with("stm32h7") {
        two_point(mcu, 0x1FF1_E820, 0x1FF1_E840, 65535.0).await
    } else {
        Err(unknown())
    }
}

/// The STM32F4/F0/H7 form: two 16-bit factory points at 30 °C and 110 °C.
async fn two_point(
    mcu: &Mcu,
    addr30: u32,
    addr110: u32,
    full_scale: f64,
) -> Result<McuCalibration, McuError> {
    let adc_30 = read16(mcu, addr30).await? / full_scale;
    let adc_110 = read16(mcu, addr110).await? / full_scale;
    Ok(two_point_calibration(30.0, adc_30, 110.0, adc_110))
}

/// The two-point line, shared by the STM32 forms.
fn two_point_calibration(t_low: f64, adc_low: f64, t_high: f64, adc_high: f64) -> McuCalibration {
    let slope = (t_high - t_low) / (adc_high - adc_low);
    McuCalibration::through(t_low, adc_low, slope)
}

/// Upstream's `config_samd21` (`klippy/extras/temperature_mcu.py:120-133`).
async fn samd_calibration(mcu: &Mcu, addr: u32) -> Result<McuCalibration, McuError> {
    let cal1 = read32(mcu, addr).await?;
    let cal2 = read32(mcu, addr + 4).await?;
    let get1v = |val: u32| -> f64 {
        let mut val = (val & 0xff) as i32;
        if val & 0x80 != 0 {
            val = 0x100 - val;
        }
        1.0 - f64::from(val) / 1000.0
    };
    let bits = |word: u32, shift: u32, mask: u32| ((word >> shift) & mask) as f64;
    let room_temp = bits(cal1, 0, 0xff) + bits(cal1, 8, 0xf) / 10.0;
    let hot_temp = bits(cal1, 12, 0xff) + bits(cal1, 20, 0xf) / 10.0;
    let room_1v = get1v(cal1 >> 24);
    let hot_1v = get1v(cal2);
    let room_adc = bits(cal2, 8, 0xfff) * room_1v / (3.3 * 4095.0);
    let hot_adc = bits(cal2, 20, 0xfff) * hot_1v / (3.3 * 4095.0);
    Ok(two_point_calibration(
        room_temp, room_adc, hot_temp, hot_adc,
    ))
}

// ===========================================================================
// The sensor
// ===========================================================================

/// The `temperature_mcu` sensor.
pub struct TemperatureMCUSensor {
    /// Set by the pre-build callback (or at construction for a fixed slope).
    calibration: Arc<Mutex<Option<McuCalibration>>>,
    /// `(min, max)` from `setup_minmax`, read by the pre-build callback.
    min_max: Arc<Mutex<(f64, f64)>>,
    callback: Arc<Mutex<Option<SensorCallback>>>,
}

impl TemperatureMCUSensor {
    /// Forward one ADC report as a temperature.
    fn handle_adc_report(&self, samples: &[(u64, f64)]) {
        let (read_time, read_value) = samples[samples.len() - 1];
        let Some(calibration) = *self.calibration.lock().unwrap() else {
            return;
        };
        let temp = calibration.calc_temp(read_value);
        let time = read_time as f64 + f64::from(SAMPLE_COUNT) * SAMPLE_TIME;
        if let Some(cb) = self.callback.lock().unwrap().as_mut() {
            cb(time, temp);
        }
    }
}

impl Sensor for TemperatureMCUSensor {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) {
        *self.min_max.lock().unwrap() = (min_temp, max_temp);
    }

    fn setup_callback(&self, callback: SensorCallback) {
        *self.callback.lock().unwrap() = Some(callback);
    }
}

impl std::fmt::Debug for TemperatureMCUSensor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TemperatureMCUSensor").finish()
    }
}

// ===========================================================================
// Sensor factory
// ===========================================================================

/// Upstream's manual calibration override
/// (`sensor_temperature1` / `sensor_adc1` / `sensor_temperature2` / `sensor_adc2`).
///
/// `None` means use the chip's own calibration; `OnePoint` keeps the chip's
/// slope and only fixes the offset, exactly as upstream's `handle_mcu_identify`
/// does when only `sensor_temperature1` is given.
#[derive(Clone, Copy, Debug)]
enum ManualCalibration {
    None,
    OnePoint { temp: f64, adc: f64 },
    Full(McuCalibration),
}

fn manual_calibration(config: &ConfigWrapper) -> Result<ManualCalibration, ConfigError> {
    let Some(temp1) = config.get_optional_float("sensor_temperature1")? else {
        return Ok(ManualCalibration::None);
    };
    let adc1 = config.get_float_bounded("sensor_adc1", None, Some(0.0), Some(1.0), None, None)?;
    let Some(temp2) = config.get_optional_float("sensor_temperature2")? else {
        return Ok(ManualCalibration::OnePoint {
            temp: temp1,
            adc: adc1,
        });
    };
    let adc2 = config.get_float_bounded("sensor_adc2", None, Some(0.0), Some(1.0), None, None)?;
    let slope = (temp2 - temp1) / (adc2 - adc1);
    Ok(ManualCalibration::Full(McuCalibration::through(
        temp1, adc1, slope,
    )))
}

/// Build the `temperature_mcu` sensor for one `[temperature_sensor]`.
///
/// Upstream's `sensor_factories['temperature_mcu']`.
pub fn temperature_mcu_factory(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn Sensor>, ConfigError> {
    let sensor_mcu = config
        .get_str("sensor_mcu")
        .unwrap_or_else(|| "mcu".to_string());
    let pins = printer
        .lookup_object_as::<PrinterPins>(PINS_OBJECT)
        .ok_or_else(|| ConfigError::new("temperature_mcu: the pins object is not registered"))?;
    let mcu_object = printer
        .lookup_object_as::<McuObject>(&sensor_mcu)
        .ok_or_else(|| ConfigError::new(format!("temperature_mcu: no MCU named '{sensor_mcu}'")))?;
    let builder: Arc<ConfigBuilder> = mcu_object.config();

    let manual = manual_calibration(config)?;
    let calibration: Arc<Mutex<Option<McuCalibration>>> = Arc::new(Mutex::new(match manual {
        ManualCalibration::Full(cal) => Some(cal),
        _ => None,
    }));
    let min_max: Arc<Mutex<(f64, f64)>> = Arc::new(Mutex::new((KELVIN_TO_CELSIUS, 99_999_999.9)));
    let callback: Arc<Mutex<Option<SensorCallback>>> = Arc::new(Mutex::new(None));
    let adc_slot: Arc<Mutex<Option<Arc<dyn Adc>>>> = Arc::new(Mutex::new(None));

    // Registered **before** `setup_adc`, so it runs before the ADC's own config
    // callback: whatever range it computes here is what the query is built with.
    {
        let calibration = Arc::clone(&calibration);
        let min_max = Arc::clone(&min_max);
        let adc_slot = Arc::clone(&adc_slot);
        let mcu_name = sensor_mcu.clone();
        builder
            .register_pre_build_callback(Box::new(move |mcu: Arc<Mcu>| {
                let calibration = Arc::clone(&calibration);
                let min_max = Arc::clone(&min_max);
                let adc_slot = Arc::clone(&adc_slot);
                let mcu_name = mcu_name.clone();
                Box::pin(async move {
                    calibrate(&mcu, &mcu_name, manual, &calibration).await?;
                    let calibration = calibration.lock().unwrap().expect("calibrate set it above");
                    let (min_temp, max_temp) = *min_max.lock().unwrap();
                    let adc_min = calibration.calc_adc(min_temp);
                    let adc_max = calibration.calc_adc(max_temp);
                    let (low, high) = if adc_min < adc_max {
                        (adc_min, adc_max)
                    } else {
                        (adc_max, adc_min)
                    };
                    if let Some(adc) = adc_slot.lock().unwrap().clone() {
                        adc.setup_adc_sample(
                            REPORT_TIME,
                            SAMPLE_TIME,
                            SAMPLE_COUNT,
                            1,
                            low,
                            high,
                            RANGE_CHECK_COUNT,
                        );
                    }
                    Ok(())
                })
            }))
            .map_err(|err| ConfigError::new(format!("temperature_mcu: {err}")))?;
    }

    let pin = format!("{sensor_mcu}:ADC_TEMPERATURE");
    let adc = pins
        .setup_adc(&pin, None)
        .map_err(|err| ConfigError::new(format!("temperature_mcu: {err}")))?;
    *adc_slot.lock().unwrap() = Some(Arc::clone(&adc));

    let sensor = Arc::new(TemperatureMCUSensor {
        calibration,
        min_max,
        callback,
    });
    let weak = Arc::downgrade(&sensor);
    adc.setup_adc_callback(Box::new(move |samples| {
        if let Some(sensor) = weak.upgrade() {
            sensor.handle_adc_report(samples);
        }
    }));
    Ok(sensor)
}

/// Fill in the calibration unless the config pinned it.
async fn calibrate(
    mcu: &Mcu,
    sensor_mcu: &str,
    manual: ManualCalibration,
    calibration: &Arc<Mutex<Option<McuCalibration>>>,
) -> Result<(), McuError> {
    let model = match mcu.dictionary() {
        Some(dictionary) => dictionary
            .constant("MCU")
            .and_then(|value| value.as_str())
            .unwrap_or("")
            .to_string(),
        None => String::new(),
    };
    let cal = match manual {
        // A full manual two-point calibration needs nothing from the chip.
        ManualCalibration::Full(cal) => cal,
        // One manual point: keep the chip's slope, fix the offset.
        ManualCalibration::OnePoint { temp, adc } => {
            let chip = calibration_for(&model, mcu).await?;
            McuCalibration::through(temp, adc, chip.slope)
        }
        ManualCalibration::None => calibration_for(&model, mcu).await?,
    };
    *calibration.lock().unwrap() = Some(cal);
    info_calibration(sensor_mcu, &model, cal);
    Ok(())
}

/// Upstream logs the resolved line (`klippy/extras/temperature_mcu.py:79`).
fn info_calibration(sensor_mcu: &str, model: &str, cal: McuCalibration) {
    tracing::info!(
        "mcu_temperature '{sensor_mcu}' ({model}) base={:.6} slope={:.6}",
        cal.base,
        cal.slope
    );
}

// ===========================================================================
// Register with heaters
// ===========================================================================

/// Register the `temperature_mcu` sensor factory with the heaters registry.
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
    fn test_a_line_through_a_point() {
        // STM32F1: 1.43 V at 25 °C, -4.3 mV/°C.
        let slope = 3.3 / -0.004300;
        let cal = McuCalibration::through(25.0, 1.43 / 3.3, slope);
        assert!((cal.calc_temp(1.43 / 3.3) - 25.0).abs() < 1e-9);
        // Warmer means a lower voltage.
        assert!(cal.calc_temp(1.0 / 3.3) > 25.0);
        // Round trip.
        assert!((cal.calc_adc(cal.calc_temp(0.4)) - 0.4).abs() < 1e-12);
    }

    #[test]
    fn test_two_point_calibration() {
        let cal = two_point_calibration(30.0, 0.4, 110.0, 0.2);
        assert!((cal.calc_temp(0.4) - 30.0).abs() < 1e-9);
        assert!((cal.calc_temp(0.2) - 110.0).abs() < 1e-9);
    }

    #[test]
    fn test_manual_calibration_reads_upstream_options() {
        use crate::core::klippy::config::{Config, ConfigWrapper};
        let config = Config::from_text(
            "[temperature_sensor mcu]\n\
             sensor_type: temperature_mcu\n\
             sensor_temperature1: 25\n\
             sensor_adc1: 0.5\n\
             sensor_temperature2: 100\n\
             sensor_adc2: 0.25\n",
        )
        .expect("parses")
        .0;
        let section = config
            .get_sections_by_id("temperature_sensor")
            .into_iter()
            .next()
            .expect("the section")
            .clone();
        let wrapper = ConfigWrapper::untracked(&section);
        let cal = match manual_calibration(&wrapper).unwrap() {
            ManualCalibration::Full(cal) => cal,
            other => panic!("expected a full two-point calibration, got {other:?}"),
        };
        // (100 - 25) / (0.25 - 0.5) = -300 °C per ADC.
        assert!((cal.slope + 300.0).abs() < 1e-9);
        assert!((cal.calc_temp(0.5) - 25.0).abs() < 1e-9);
    }
}
