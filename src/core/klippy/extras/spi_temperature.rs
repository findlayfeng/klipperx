//! SPI thermocouple and RTD temperature sensors.
//!
//! Upstream's `spi_temperature.py`: `MAX6675`, `MAX31855`, `MAX31856`
//! (thermocouples) and `MAX31865` (an RTD). Each is a `[temperature_sensor]`
//! whose `sensor_pin` is the chip select; the firmware polls the chip and pushes
//! raw register values, and the chip class here turns them into degrees.
//!
//! # Timing
//!
//! The periodic query carries an absolute clock, so — like the ADC and DS18B20
//! paths — it is armed from a **post-init** callback with a clock read on the
//! connection that will report, not baked into the (reset-surviving) config.
//! The SPI init registers some chips need are written at the same moment.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock, Weak};

use tracing::warn;

use crate::core::klippy::cmd::thermocouple::{
    ConfigThermocouple, QueryThermocouple, ThermocoupleResult, ThermocoupleType,
};
use crate::core::klippy::cmd::{McuResponse, Params};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::heaters::{PrinterHeaters, Sensor, SensorCallback};
use crate::core::klippy::extras::spi_device::{mcu_object_name, mcu_spi_from_config};
use crate::core::klippy::mcu::{query_slot, Mcu, McuError, McuObject, McuSpi};
use crate::core::klippy::msg::proto::ArgValue;
use crate::core::klippy::msg::Msg;
use crate::core::klippy::printer::Printer;

/// How often a chip is read (`REPORT_TIME`).
const REPORT_TIME: f64 = 0.300;
/// Bad readings the firmware tolerates before shutting down (`MAX_INVALID_COUNT`).
const MAX_INVALID_COUNT: u8 = 3;
/// Default SPI clock for these chips (upstream's `default_speed`).
const DEFAULT_SPEED: u32 = 4_000_000;

// ===========================================================================
// The chip classes
// ===========================================================================

/// One SPI temperature chip: how to read it and what its raw value means.
trait Chip: Send + Sync {
    /// The firmware's `thermocouple_type` number.
    fn kind(&self) -> ThermocoupleType;
    /// The SPI mode used when the section does not set `spi_mode`.
    fn default_spi_mode(&self) -> u8;
    /// Raw register value → degrees Celsius.
    fn calc_temp(&self, adc: u32) -> f64;
    /// Degrees Celsius → the raw value, for the firmware's range check.
    fn calc_adc(&self, temp: f64) -> u32;
    /// Report (and possibly clear) a fault.
    fn handle_fault(&self, spi: &McuSpi, adc: u32, fault: u8);
    /// Registers to write before the first read, if any.
    fn init_bytes(&self, _spi: &McuSpi) -> Vec<Vec<u8>> {
        Vec::new()
    }
}

fn report_fault(msg: String) {
    warn!("{msg}");
}

// ---------------------------------------------------------------------------
// MAX6675
// ---------------------------------------------------------------------------

const MAX6675_SCALE: u32 = 3;
const MAX6675_MULT: f64 = 0.25;

struct Max6675;

impl Chip for Max6675 {
    fn kind(&self) -> ThermocoupleType {
        ThermocoupleType::Max6675
    }
    fn default_spi_mode(&self) -> u8 {
        0
    }
    fn calc_temp(&self, adc: u32) -> f64 {
        let mut adc = i64::from(adc >> MAX6675_SCALE);
        // Fix the sign bit.
        if adc & 0x2000 != 0 {
            adc = -(((adc & 0x1FFF) + 1) as i64);
        }
        MAX6675_MULT * adc as f64
    }
    fn calc_adc(&self, temp: f64) -> u32 {
        let adc = (temp / MAX6675_MULT + 0.5) as i64;
        (adc.clamp(0, 0x1FFF) as u32) << MAX6675_SCALE
    }
    fn handle_fault(&self, _spi: &McuSpi, _adc: u32, fault: u8) {
        if fault & 0x02 != 0 {
            report_fault("Max6675 : Device ID error".to_string());
        }
        if fault & 0x04 != 0 {
            report_fault("Max6675 : Thermocouple Open Fault".to_string());
        }
    }
}

// ---------------------------------------------------------------------------
// MAX31855
// ---------------------------------------------------------------------------

const MAX31855_SCALE: u32 = 18;
const MAX31855_MULT: f64 = 0.25;

struct Max31855;

impl Chip for Max31855 {
    fn kind(&self) -> ThermocoupleType {
        ThermocoupleType::Max31855
    }
    fn default_spi_mode(&self) -> u8 {
        0
    }
    fn calc_temp(&self, adc: u32) -> f64 {
        let mut adc = i64::from(adc >> MAX31855_SCALE);
        if adc & 0x2000 != 0 {
            adc = -(((adc & 0x1FFF) + 1) as i64);
        }
        MAX31855_MULT * adc as f64
    }
    fn calc_adc(&self, temp: f64) -> u32 {
        let adc = (temp / MAX31855_MULT + 0.5) as i64;
        (adc.clamp(0, 0x1FFF) as u32) << MAX31855_SCALE
    }
    fn handle_fault(&self, _spi: &McuSpi, _adc: u32, fault: u8) {
        if fault & 0x1 != 0 {
            report_fault("MAX31855 : Open Circuit".to_string());
        }
        if fault & 0x2 != 0 {
            report_fault("MAX31855 : Short to GND".to_string());
        }
        if fault & 0x4 != 0 {
            report_fault("MAX31855 : Short to Vcc".to_string());
        }
    }
}

// ---------------------------------------------------------------------------
// MAX31856
// ---------------------------------------------------------------------------

const MAX31856_CR0_REG: u8 = 0x00;
const MAX31856_CR0_AUTOCONVERT: u8 = 0x80;
const MAX31856_CR0_FILT50HZ: u8 = 0x01;
const MAX31856_CR1_AVGSEL1: u8 = 0x00;
const MAX31856_CR1_AVGSEL2: u8 = 0x10;
const MAX31856_CR1_AVGSEL4: u8 = 0x20;
const MAX31856_CR1_AVGSEL8: u8 = 0x30;
const MAX31856_CR1_AVGSEL16: u8 = 0x70;
const MAX31856_MASK_VOLTAGE_UNDER_OVER_FAULT: u8 = 0x02;
const MAX31856_MASK_THERMOCOUPLE_OPEN_FAULT: u8 = 0x01;
const MAX31856_SCALE: u32 = 5;
const MAX31856_MULT: f64 = 0.0078125;

struct Max31856 {
    init: Vec<u8>,
}

impl Max31856 {
    fn new(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let mut cr0 = MAX31856_CR0_AUTOCONVERT;
        if config.get_bool("tc_use_50Hz_filter", Some(false))? {
            cr0 |= MAX31856_CR0_FILT50HZ;
        }
        let tc_type = config.get_choice(
            "tc_type",
            &["B", "E", "J", "K", "N", "R", "S", "T"],
            Some("K"),
        )?;
        let cr1_type: u8 = match tc_type.as_str() {
            "B" => 0b0000,
            "E" => 0b0001,
            "J" => 0b0010,
            "K" => 0b0011,
            "N" => 0b0100,
            "R" => 0b0101,
            "S" => 0b0110,
            "T" => 0b0111,
            _ => unreachable!("get_choice validated the value"),
        };
        let averaging = config.get_int("tc_averaging_count", Some(1))?;
        let cr1 = cr1_type
            | match averaging {
                1 => MAX31856_CR1_AVGSEL1,
                2 => MAX31856_CR1_AVGSEL2,
                4 => MAX31856_CR1_AVGSEL4,
                8 => MAX31856_CR1_AVGSEL8,
                16 => MAX31856_CR1_AVGSEL16,
                _ => {
                    return Err(ConfigError::new(format!(
                        "Option 'tc_averaging_count' in section '{}' must be one of \
                         1, 2, 4, 8, 16",
                        config.identifier()
                    )))
                }
            };
        let mask = MAX31856_MASK_VOLTAGE_UNDER_OVER_FAULT | MAX31856_MASK_THERMOCOUPLE_OPEN_FAULT;
        Ok(Self {
            init: vec![0x80 + MAX31856_CR0_REG, cr0, cr1, mask],
        })
    }

    fn fault_name(fault: u8) -> Option<&'static str> {
        match fault {
            0x80 => Some("Max31856: Cold Junction Range Fault"),
            0x40 => Some("Max31856: Thermocouple Range Fault"),
            0x20 => Some("Max31856: Cold Junction High Fault"),
            0x10 => Some("Max31856: Cold Junction Low Fault"),
            0x08 => Some("Max31856: Thermocouple High Fault"),
            0x04 => Some("Max31856: Thermocouple Low Fault"),
            0x02 => Some("Max31856: Over/Under Voltage Fault"),
            0x01 => Some("Max31856: Thermocouple Open Fault"),
            _ => None,
        }
    }
}

impl Chip for Max31856 {
    fn kind(&self) -> ThermocoupleType {
        ThermocoupleType::Max31856
    }
    fn default_spi_mode(&self) -> u8 {
        1
    }
    fn calc_temp(&self, adc: u32) -> f64 {
        let mut adc = i64::from(adc >> MAX31856_SCALE);
        // Fix the sign bit (bit 18).
        if adc & 0x40000 != 0 {
            adc = -(((adc & 0x3FFFF) + 1) as i64);
        }
        MAX31856_MULT * adc as f64
    }
    fn calc_adc(&self, temp: f64) -> u32 {
        let adc = (temp / MAX31856_MULT + 0.5) as i64;
        (adc.clamp(0, 0x3FFFF) as u32) << MAX31856_SCALE
    }
    fn handle_fault(&self, _spi: &McuSpi, _adc: u32, fault: u8) {
        for bit in [0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01] {
            if fault & bit != 0 {
                if let Some(name) = Self::fault_name(bit) {
                    report_fault(name.to_string());
                }
            }
        }
    }
    fn init_bytes(&self, _spi: &McuSpi) -> Vec<Vec<u8>> {
        vec![self.init.clone()]
    }
}

// ---------------------------------------------------------------------------
// MAX31865 (RTD)
// ---------------------------------------------------------------------------

const MAX31865_CONFIG_REG: u8 = 0x00;
const MAX31865_CONFIG_BIAS: u8 = 0x80;
const MAX31865_CONFIG_MODEAUTO: u8 = 0x40;
const MAX31865_CONFIG_3WIRE: u8 = 0x10;
const MAX31865_CONFIG_FAULTCLEAR: u8 = 0x02;
const MAX31865_CONFIG_FILT50HZ: u8 = 0x01;
const MAX31865_ADC_MAX: u32 = 1 << 15;
const CVD_A: f64 = 3.9083e-3;
const CVD_B: f64 = -5.775e-7;

struct Max31865 {
    /// `rtd_reference_r / MAX31865_ADC_MAX / rtd_nominal_r`.
    adc_to_resist_div_nominal: f64,
    config_reg: Vec<u8>,
}

impl Max31865 {
    fn new(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let rtd_nominal_r =
            config.get_float_bounded("rtd_nominal_r", Some(100.0), Some(0.0), None, None, None)?;
        let rtd_reference_r = config.get_float_bounded(
            "rtd_reference_r",
            Some(430.0),
            Some(0.0),
            None,
            None,
            None,
        )?;
        let adc_to_resist = rtd_reference_r / f64::from(MAX31865_ADC_MAX);
        let adc_to_resist_div_nominal = adc_to_resist / rtd_nominal_r;

        let mut value =
            MAX31865_CONFIG_BIAS | MAX31865_CONFIG_MODEAUTO | MAX31865_CONFIG_FAULTCLEAR;
        if config.get_bool("rtd_use_50Hz_filter", Some(false))? {
            value |= MAX31865_CONFIG_FILT50HZ;
        }
        if config.get_int("rtd_num_of_wires", Some(2))? == 3 {
            value |= MAX31865_CONFIG_3WIRE;
        }
        Ok(Self {
            adc_to_resist_div_nominal,
            config_reg: vec![0x80 + MAX31865_CONFIG_REG, value],
        })
    }
}

impl Chip for Max31865 {
    fn kind(&self) -> ThermocoupleType {
        ThermocoupleType::Max31865
    }
    fn default_spi_mode(&self) -> u8 {
        1
    }
    fn calc_temp(&self, adc: u32) -> f64 {
        let adc = adc >> 1; // drop the fault bit
        let r_div_nominal = f64::from(adc) * self.adc_to_resist_div_nominal;
        // Solve `R_div_nominal = 1 + A*t + B*t^2` for `t`.
        let discriminant = (CVD_A * CVD_A - 4.0 * CVD_B * (1.0 - r_div_nominal)).sqrt();
        (-CVD_A + discriminant) / (2.0 * CVD_B)
    }
    fn calc_adc(&self, temp: f64) -> u32 {
        let temp = temp.min(1768.3); // melting point of platinum
        let r_div_nominal = 1.0 + CVD_A * temp + CVD_B * temp * temp;
        let adc = (r_div_nominal / self.adc_to_resist_div_nominal + 0.5) as i64;
        (adc.clamp(0, i64::from(MAX31865_ADC_MAX - 1)) as u32) << 1
    }
    fn handle_fault(&self, spi: &McuSpi, _adc: u32, fault: u8) {
        if fault & 0x80 != 0 {
            report_fault("Max31865 RTD input is disconnected".to_string());
        }
        if fault & 0x40 != 0 {
            report_fault("Max31865 RTD input is shorted".to_string());
        }
        if fault & 0x20 != 0 {
            report_fault("Max31865 VREF- is greater than 0.85 * VBIAS, FORCE- open".to_string());
        }
        if fault & 0x10 != 0 {
            report_fault("Max31865 VREF- is less than 0.85 * VBIAS, FORCE- open".to_string());
        }
        if fault & 0x08 != 0 {
            report_fault("Max31865 VRTD- is less than 0.85 * VBIAS, FORCE- open".to_string());
        }
        if fault & 0x04 != 0 {
            report_fault("Max31865 Overvoltage or undervoltage fault".to_string());
        }
        if fault & 0xfc == 0 {
            report_fault("Max31865 Unspecified error".to_string());
        }
        // Attempt to clear the fault, as upstream does.
        let _ = spi.send(&self.config_reg);
    }
    fn init_bytes(&self, spi: &McuSpi) -> Vec<Vec<u8>> {
        let _ = spi;
        vec![self.config_reg.clone()]
    }
}

// ===========================================================================
// The sensor
// ===========================================================================

/// One `[temperature_sensor]` backed by an SPI chip.
pub struct SpiTemperature {
    chip: Arc<dyn Chip>,
    spi: Arc<McuSpi>,
    mcu: Arc<McuObject>,
    /// Set by the config callback (the oid must be allocated at build time).
    oid: Mutex<Option<u8>>,
    /// Ticks between readings, from the build callback.
    report_clock: Mutex<u32>,
    /// Raw range from `setup_minmax`, used by the query.
    min_sample: Mutex<u32>,
    max_sample: Mutex<u32>,
    callback: Mutex<Option<SensorCallback>>,
}

impl SpiTemperature {
    fn oid(&self) -> Option<u8> {
        *self.oid.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Send this chip's periodic query, with a clock read from `mcu` now.
    fn arm_query(&self, mcu: &Mcu) -> Result<(), McuError> {
        let oid = self
            .oid()
            .ok_or_else(|| McuError::Config("thermocouple oid is not allocated".to_string()))?;
        let clock = query_slot(mcu, oid)?;
        mcu.send_msg(&QueryThermocouple {
            oid,
            clock,
            rest_ticks: *self.report_clock.lock().unwrap_or_else(|p| p.into_inner()),
            min_value: *self.min_sample.lock().unwrap_or_else(|p| p.into_inner()),
            max_value: *self.max_sample.lock().unwrap_or_else(|p| p.into_inner()),
            max_invalid_count: MAX_INVALID_COUNT,
        })
    }

    fn handle_result(&self, result: ThermocoupleResult) {
        if result.fault != 0 {
            self.chip
                .handle_fault(&self.spi, result.value, result.fault);
            return;
        }
        let temp = self.chip.calc_temp(result.value);
        let Some(next) = self.mcu.clock32_to_clock64(result.next_clock) else {
            return;
        };
        let Some(clock) = self.mcu.clock() else {
            return;
        };
        let report_clock = i64::from(*self.report_clock.lock().unwrap_or_else(|p| p.into_inner()));
        let read_time = clock.clock_to_print_time(next - report_clock);
        if let Some(cb) = self
            .callback
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .as_mut()
        {
            cb(read_time, temp);
        }
    }
}

impl Sensor for SpiTemperature {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) {
        let low = self.chip.calc_adc(min_temp);
        let high = self.chip.calc_adc(max_temp);
        let (min, max) = if low < high { (low, high) } else { (high, low) };
        *self.min_sample.lock().unwrap_or_else(|p| p.into_inner()) = min;
        *self.max_sample.lock().unwrap_or_else(|p| p.into_inner()) = max;
    }

    fn setup_callback(&self, callback: SensorCallback) {
        *self.callback.lock().unwrap_or_else(|p| p.into_inner()) = Some(callback);
    }
}

impl std::fmt::Debug for SpiTemperature {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SpiTemperature").finish()
    }
}

/// Build the sensor for one `[temperature_sensor]`.
fn spi_sensor(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
    chip: Arc<dyn Chip>,
) -> Result<Arc<dyn Sensor>, ConfigError> {
    let setup = mcu_spi_from_config(
        config,
        printer,
        chip.default_spi_mode(),
        "sensor_pin",
        DEFAULT_SPEED,
    )?;
    let spi = setup.device;

    let mcu_name = config
        .get_str("spi_mcu")
        .unwrap_or_else(|| "mcu".to_string());
    let mcu_object = printer
        .lookup_object_as::<McuObject>(&mcu_object_name(&mcu_name))
        .ok_or_else(|| ConfigError::new(format!("Unknown mcu '{mcu_name}'")))?;
    let builder = mcu_object.config();

    let sensor = Arc::new(SpiTemperature {
        chip: Arc::clone(&chip),
        spi: Arc::clone(&spi),
        mcu: Arc::clone(&mcu_object),
        oid: Mutex::new(None),
        report_clock: Mutex::new(0),
        min_sample: Mutex::new(0),
        max_sample: Mutex::new(0),
        callback: Mutex::new(None),
    });

    let build_sensor = Arc::downgrade(&sensor);
    let build_spi = Arc::clone(&spi);
    let build_chip = Arc::clone(&chip);
    builder
        .register_config_callback(Box::new(move |builder, mcu| {
            let spi_oid = build_spi.oid()?;
            let oid = builder.create_oid()?;
            builder.add_config_cmd(&ConfigThermocouple {
                oid,
                spi_oid,
                thermocouple_type: build_chip.kind(),
            })?;
            let rest_ticks = mcu.seconds_to_clock(REPORT_TIME)? as u32;
            if let Some(sensor) = build_sensor.upgrade() {
                *sensor
                    .report_clock
                    .lock()
                    .unwrap_or_else(|p| p.into_inner()) = rest_ticks;
                *sensor.oid.lock().unwrap_or_else(|p| p.into_inner()) = Some(oid);
            }
            Ok(())
        }))
        .map_err(|err| ConfigError::new(err.to_string()))?;

    let post_sensor = Arc::downgrade(&sensor);
    let post_spi = Arc::clone(&spi);
    let post_chip = Arc::clone(&chip);
    builder
        .register_post_init_callback(Box::new(move |mcu| {
            let Some(sensor) = post_sensor.upgrade() else {
                return;
            };
            // The chip's init registers first, then the query that reads it.
            for bytes in post_chip.init_bytes(&post_spi) {
                if let Err(err) = post_spi.send(&bytes) {
                    warn!(
                        "MCU '{}': could not configure the thermocouple: {err}",
                        mcu.name()
                    );
                    return;
                }
            }
            if let Err(err) = sensor.arm_query(mcu) {
                warn!(
                    "MCU '{}': could not arm the thermocouple query: {err}",
                    mcu.name()
                );
                return;
            }
            let registry = registry_for(mcu.name());
            let Some(oid) = sensor.oid() else {
                return;
            };
            if let Err(err) = registry.bind(mcu, oid, Arc::downgrade(&sensor)) {
                warn!(
                    "MCU '{}': could not bind thermocouple response: {err}",
                    mcu.name()
                );
            }
        }))
        .map_err(|err| ConfigError::new(err.to_string()))?;

    Ok(sensor as Arc<dyn Sensor>)
}

// ===========================================================================
// Per-oid routing
// ===========================================================================

/// Per-oid routing for `thermocouple_result`, one registry per MCU.
#[derive(Default)]
struct ThermocoupleRegistry {
    sensors: Mutex<HashMap<u8, Weak<SpiTemperature>>>,
}

impl ThermocoupleRegistry {
    fn bind(
        self: &Arc<Self>,
        mcu: &Mcu,
        oid: u8,
        sensor: Weak<SpiTemperature>,
    ) -> Result<(), McuError> {
        self.sensors
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(oid, sensor);

        let msg = mcu.require_message(ThermocoupleResult::NAME)?;
        // A standalone declaration, so the callback does not keep the registry
        // entry alive (`bind_event` does the same).
        let declaration = Arc::new(Msg::new(msg.id, msg.name.clone(), msg.params.clone()));
        let dictionary = mcu.dictionary();
        let registry = Arc::clone(self);
        mcu.bind_callback(ThermocoupleResult::NAME, move |values| {
            let oid = match values.first() {
                Some(ArgValue::UInt8(oid)) => *oid,
                _ => return,
            };
            let sensor = registry
                .sensors
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .get(&oid)
                .and_then(Weak::upgrade);
            let Some(sensor) = sensor else {
                return;
            };
            let params = Params::new(Arc::clone(&declaration), values);
            let params = match &dictionary {
                Some(dictionary) => params.with_dictionary(Arc::clone(dictionary)),
                None => params,
            };
            match ThermocoupleResult::decode(&params) {
                Ok(result) => sensor.handle_result(result),
                Err(err) => warn!("Failed to decode thermocouple_result: {err}"),
            }
        })
    }
}

fn registry_for(name: &str) -> Arc<ThermocoupleRegistry> {
    static REGISTRIES: OnceLock<Mutex<HashMap<String, Arc<ThermocoupleRegistry>>>> =
        OnceLock::new();
    let registries = REGISTRIES.get_or_init(|| Mutex::new(HashMap::new()));
    let mut registries = registries.lock().unwrap_or_else(|p| p.into_inner());
    Arc::clone(
        registries
            .entry(name.to_string())
            .or_insert_with(|| Arc::new(ThermocoupleRegistry::default())),
    )
}

// ===========================================================================
// Registration
// ===========================================================================

/// Register the four SPI temperature chips with the heaters registry.
pub fn ensure(heaters: &Arc<PrinterHeaters>) -> Result<(), ConfigError> {
    heaters.add_sensor_factory(
        "MAX6675",
        Arc::new(|config: &ConfigWrapper, printer: &Arc<Printer>| {
            spi_sensor(config, printer, Arc::new(Max6675))
        }),
    );
    heaters.add_sensor_factory(
        "MAX31855",
        Arc::new(|config: &ConfigWrapper, printer: &Arc<Printer>| {
            spi_sensor(config, printer, Arc::new(Max31855))
        }),
    );
    heaters.add_sensor_factory(
        "MAX31856",
        Arc::new(|config: &ConfigWrapper, printer: &Arc<Printer>| {
            let chip = Arc::new(Max31856::new(config)?);
            spi_sensor(config, printer, chip)
        }),
    );
    heaters.add_sensor_factory(
        "MAX31865",
        Arc::new(|config: &ConfigWrapper, printer: &Arc<Printer>| {
            let chip = Arc::new(Max31865::new(config)?);
            spi_sensor(config, printer, chip)
        }),
    );
    Ok(())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_max6675_and_max31855_conversions() {
        // 25 °C is raw (25 / 0.25) << scale.
        let chip = Max6675;
        let raw = ((25.0 / MAX6675_MULT) as u32) << MAX6675_SCALE;
        assert!((chip.calc_temp(raw) - 25.0).abs() < 1e-9);
        assert_eq!(chip.calc_adc(25.0), raw);

        let chip = Max31855;
        let raw = ((25.0 / MAX31855_MULT) as u32) << MAX31855_SCALE;
        assert!((chip.calc_temp(raw) - 25.0).abs() < 1e-9);
        assert_eq!(chip.calc_adc(25.0), raw);
    }

    #[test]
    fn test_the_sign_bit_produces_a_negative_temperature() {
        // The sign bit alone is one negative count (-0.25 °C).
        let chip = Max6675;
        let raw = 0x2000u32 << MAX6675_SCALE;
        assert!((chip.calc_temp(raw) + 0.25).abs() < 1e-9);

        let chip = Max31855;
        let raw = 0x2000u32 << MAX31855_SCALE;
        assert!((chip.calc_temp(raw) + 0.25).abs() < 1e-9);
    }

    #[test]
    fn test_max31856_conversions() {
        let chip = Max31856 { init: Vec::new() };
        let raw = ((25.0 / MAX31856_MULT) as u32) << MAX31856_SCALE;
        assert!((chip.calc_temp(raw) - 25.0).abs() < 1e-9);
        assert_eq!(chip.calc_adc(25.0), raw);
    }

    #[test]
    fn test_max31865_conversions() {
        // 100 Ω nominal, 430 Ω reference: 100 Ω at 0 °C.
        let chip = Max31865 {
            adc_to_resist_div_nominal: 430.0 / f64::from(MAX31865_ADC_MAX) / 100.0,
            config_reg: Vec::new(),
        };
        let raw = chip.calc_adc(0.0);
        assert!(
            (chip.calc_temp(raw) - 0.0).abs() < 0.1,
            "{}",
            chip.calc_temp(raw)
        );
        let raw = chip.calc_adc(100.0);
        assert!(
            (chip.calc_temp(raw) - 100.0).abs() < 0.1,
            "{}",
            chip.calc_temp(raw)
        );
    }
}
