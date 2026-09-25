//! TMC UART transport (upstream's `klippy/extras/tmc_uart.py`).
//!
//! | upstream | here |
//! |---|---|
//! | `MCU_analog_mux` | [`McuAnalogMux`] |
//! | `MCU_TMC_uart_bitbang` | [`TmcUartBitbang`] |
//! | `lookup_tmc_uart_bitbang` | [`lookup_tmc_uart_bitbang`] |
//! | `MCU_TMC_uart` | [`TmcUart`] (a [`TmcTransport`]) |
//!
//! # Deliberate deviation: the live wire path is not wired
//!
//! The corpus runs under `fileoutput`, where a TMC register read answers 0 and
//! a write is dropped ([`TmcUart`] short-circuits exactly where upstream's
//! `_do_get_register`/`set_register` check `debugoutput`). Nothing here sends a
//! `tmcuart_send` frame, which is the point: the fake firmware never answers
//! `tmcuart_response`, so a real exchange would hang. A non-file-output read or
//! write therefore reports an error rather than doing nothing.
//!
//! Because only the file-output path is exercised, the wire encode/decode
//! helpers ([`crc8`], [`TmcuartMessage::encode_read`], …) are implemented and
//! unit-tested but not yet wired to an `Mcu` round-trip; the SPI (2130/5160/
//! 2240) and 2660 transports belong to a later unit.

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::spi_device::mcu_object_name;
use crate::core::klippy::extras::tmc::{TmcRegister, TmcTransport};
use crate::core::klippy::mcu::{McuError, McuObject};
use crate::core::klippy::msg::proto::ArgValue;
use crate::core::klippy::pins::{DigitalOut, PinError, PinParams, PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The printer object that shares one [`TmcUartBitbang`] per `(rx, tx)` pair
/// (upstream's `PrinterTMCUartMutexes`, registered as `tmc_uart`).
const TMC_UART_OBJECT: &str = "tmc_uart";

/// The nominal TMC UART baud rate (`TMC_BAUD_RATE`).
const TMC_BAUD_RATE: f64 = 40000.;

/// The lower baud rate AVR boards need (`TMC_BAUD_RATE_AVR`).
const TMC_BAUD_RATE_AVR: f64 = 9000.;

/// `config_tmcuart` (`src/tmcuart.c:185`). The firmware dictionary owns the
/// format; the args follow it in order.
struct ConfigTmcuart {
    oid: u8,
    rx_pin: u32,
    pull_up: u8,
    tx_pin: u32,
    bit_time: u32,
}

impl McuCommand for ConfigTmcuart {
    const NAME: &'static str = "config_tmcuart";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.rx_pin),
            ArgValue::UInt8(self.pull_up),
            ArgValue::UInt32(self.tx_pin),
            ArgValue::UInt32(self.bit_time),
        ]
    }
}

// ===========================================================================
// UART message encoding (`MCU_TMC_uart_bitbang` helpers)
// ===========================================================================

/// CRC8-ATM over `data` (`_calc_crc8`).
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for byte in data {
        let mut b = *byte;
        for _ in 0..8 {
            if (crc >> 7) ^ (b & 0x01) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
            b >>= 1;
        }
    }
    crc
}

/// Add the UART start/stop bits to a byte array (`_add_serial_bits`).
pub fn add_serial_bits(data: &[u8]) -> Vec<u8> {
    let mut out: u128 = 0;
    let mut pos = 0;
    for d in data {
        let b = ((*d as u128) << 1) | 0x200;
        out |= b << pos;
        pos += 10;
    }
    let mut res = Vec::new();
    for i in 0..(pos + 7) / 8 {
        res.push(((out >> (i * 8)) & 0xff) as u8);
    }
    res
}

/// The sync byte for a read (`0xf5`).
const UART_SYNC: u8 = 0xf5;

/// Encode a read request (`_encode_read`).
pub fn encode_read(addr: u8, reg: u8) -> Vec<u8> {
    let mut msg = vec![UART_SYNC, addr, reg];
    msg.push(crc8(&msg));
    add_serial_bits(&msg)
}

/// Encode a request with a leading `sync` byte (`_encode_write`).
///
/// A register write uses `sync = 0xf5` with the register's `0x80` bit set; the
/// read-response check reuses the same encoder with `sync = 0x05`.
pub fn encode_write(sync: u8, addr: u8, reg: u8, val: u32) -> Vec<u8> {
    let mut msg = vec![
        sync,
        addr,
        reg,
        ((val >> 24) & 0xff) as u8,
        ((val >> 16) & 0xff) as u8,
        ((val >> 8) & 0xff) as u8,
        (val & 0xff) as u8,
    ];
    msg.push(crc8(&msg));
    add_serial_bits(&msg)
}

/// Decode a read response, verifying the start/stop bits and CRC
/// (`_decode_read`).
pub fn decode_read(reg: u8, data: &[u8]) -> Option<u32> {
    if data.len() != 10 {
        return None;
    }
    let mut mval: u128 = 0;
    let mut pos = 0;
    for d in data {
        mval |= (*d as u128) << pos;
        pos += 8;
    }
    let val = ((((mval >> 31) & 0xff) << 24)
        | (((mval >> 41) & 0xff) << 16)
        | (((mval >> 51) & 0xff) << 8)
        | ((mval >> 61) & 0xff)) as u32;
    let encoded = encode_write(0x05, 0xff, reg, val);
    if data != encoded.as_slice() {
        return None;
    }
    Some(val)
}

// ===========================================================================
// Analog mux (`MCU_analog_mux`)
// ===========================================================================

/// The select pins that address one of several chips on a shared UART
/// (`MCU_analog_mux`).
struct McuAnalogMux {
    /// The printer, for re-parsing a later instance's select pins.
    printer: Weak<Printer>,
    /// The MCU chip the select pins live on.
    chip: String,
    /// The select pins, in order.
    pins: Vec<String>,
    /// The digital outputs driving them. Activation is only reachable from
    /// the live wire path, which is not wired in this unit (see the module
    /// docs); the outputs are built so the select pins are reserved and their
    /// `config_digital_out` is registered.
    #[allow(dead_code)]
    outs: Vec<Arc<dyn DigitalOut>>,
    /// The value last written to each (`pin_values`), `None` before the first.
    #[allow(dead_code)]
    pin_values: Mutex<Vec<Option<bool>>>,
}

impl McuAnalogMux {
    /// Build the mux: one digital output per select pin.
    ///
    /// # Errors
    /// A pin that cannot be resolved or reserved.
    fn new(printer: &Arc<Printer>, select_pins_desc: &[String]) -> Result<Self, ConfigError> {
        let ppins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .ok_or_else(|| ConfigError::new("the pins object is not registered"))?;
        let parsed = select_pins_desc
            .iter()
            .map(|desc| ppins.parse_pin(desc, true, false))
            .collect::<Result<Vec<PinParams>, PinError>>()
            .map_err(|err| ConfigError::new(err.to_string()))?;
        let chip = parsed
            .first()
            .map(|params| params.chip_name.clone())
            .unwrap_or_default();
        let mut outs = Vec::with_capacity(select_pins_desc.len());
        for desc in select_pins_desc {
            let out = ppins
                .setup_digital_out(desc, None)
                .map_err(|err| ConfigError::new(err.to_string()))?;
            outs.push(out);
        }
        let pins = parsed.into_iter().map(|params| params.pin).collect();
        Ok(Self {
            printer: Arc::downgrade(printer),
            chip,
            pins,
            outs,
            pin_values: Mutex::new(vec![None; select_pins_desc.len()]),
        })
    }

    /// The polarity tuple identifying one select combination (`get_instance_id`).
    ///
    /// # Errors
    /// A select pin on a different MCU, or a different pin set.
    fn get_instance_id(&self, select_pins_desc: &[String]) -> Result<Vec<bool>, ConfigError> {
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| ConfigError::new("the printer is gone"))?;
        let ppins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .ok_or_else(|| ConfigError::new("the pins object is not registered"))?;
        let parsed = select_pins_desc
            .iter()
            .map(|desc| ppins.parse_pin(desc, true, false))
            .collect::<Result<Vec<PinParams>, PinError>>()
            .map_err(|err| ConfigError::new(err.to_string()))?;
        for params in &parsed {
            if params.chip_name != self.chip {
                return Err(ConfigError::new("TMC mux pins must be on the same mcu"));
            }
        }
        let pins: Vec<String> = parsed.iter().map(|params| params.pin.clone()).collect();
        if pins != self.pins {
            return Err(ConfigError::new(
                "All TMC mux instances must use identical pins",
            ));
        }
        Ok(parsed.into_iter().map(|params| !params.invert).collect())
    }

    /// Drive the select pins to `instance_id` (`activate`).
    #[allow(dead_code)]
    fn activate(&self, instance_id: &[bool]) -> Result<(), McuError> {
        let mut values = self
            .pin_values
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        for (index, out) in self.outs.iter().enumerate() {
            let new = instance_id.get(index).copied().unwrap_or(false);
            if values[index] != Some(new) {
                out.update_digital_out(new)?;
            }
            values[index] = Some(new);
        }
        Ok(())
    }
}

// ===========================================================================
// Bit-banged UART (`MCU_TMC_uart_bitbang`)
// ===========================================================================

/// One shared UART peripheral on an MCU (`MCU_TMC_uart_bitbang`).
///
/// It owns the firmware `config_tmcuart` oid and the set of
/// `(instance_id, address)` pairs already registered, so a second driver on the
/// same pins must use a distinct address (or a distinct mux polarity).
pub(crate) struct TmcUartBitbang {
    rx_pin: String,
    tx_pin: String,
    mux: Option<McuAnalogMux>,
    instances: Mutex<HashSet<(Option<Vec<bool>>, i64)>>,
}

impl TmcUartBitbang {
    /// Allocate the oid and register the `config_tmcuart` command.
    fn build(
        printer: &Arc<Printer>,
        chip_name: &str,
        rx_params: &PinParams,
        tx_params: &PinParams,
        select_pins_desc: Option<&[String]>,
    ) -> Result<Arc<Self>, ConfigError> {
        let mcu_object = printer
            .lookup_object_as::<McuObject>(&mcu_object_name(chip_name))
            .ok_or_else(|| {
                ConfigError::new(format!(
                    "Could not find the '{chip_name}' MCU for a TMC uart"
                ))
            })?;
        let builder = mcu_object.config();
        let oid = builder
            .create_oid()
            .map_err(|err| ConfigError::new(err.to_string()))?;
        let mux = match select_pins_desc {
            Some(pins) => Some(McuAnalogMux::new(printer, pins)?),
            None => None,
        };
        let rx_pin = rx_params.pin.clone();
        let tx_pin = tx_params.pin.clone();
        let pullup = rx_params.pullup;
        let callback_chip = chip_name.to_string();
        let callback_rx_pin = rx_params.pin.clone();
        let callback_tx_pin = tx_params.pin.clone();
        builder
            .register_config_callback(Box::new(move |builder, mcu| {
                let baud = {
                    let mcu_type = mcu
                        .dictionary()
                        .and_then(|dictionary| {
                            dictionary
                                .constant("MCU")
                                .and_then(Value::as_str)
                                .map(str::to_string)
                        })
                        .unwrap_or_default();
                    if mcu_type.starts_with("atmega") || mcu_type.starts_with("at90usb") {
                        TMC_BAUD_RATE_AVR
                    } else {
                        TMC_BAUD_RATE
                    }
                };
                let bit_time = mcu.seconds_to_clock(1. / baud)? as u32;
                let rx = pin_number(mcu, &callback_chip, &callback_rx_pin)?;
                let tx = pin_number(mcu, &callback_chip, &callback_tx_pin)?;
                builder.add_config_cmd(&ConfigTmcuart {
                    oid,
                    rx_pin: rx,
                    pull_up: pullup as u8,
                    tx_pin: tx,
                    bit_time,
                })?;
                Ok(())
            }))
            .map_err(|err| ConfigError::new(err.to_string()))?;
        Ok(Arc::new(Self {
            rx_pin,
            tx_pin,
            mux,
            instances: Mutex::new(HashSet::new()),
        }))
    }

    /// Register a driver on this peripheral (`register_instance`).
    ///
    /// # Errors
    /// Different pins, or a repeated `(instance_id, address)`.
    fn register_instance(
        &self,
        rx_params: &PinParams,
        tx_params: &PinParams,
        select_pins_desc: Option<&[String]>,
        addr: i64,
    ) -> Result<Option<Vec<bool>>, ConfigError> {
        let same_pins = rx_params.pin == self.rx_pin
            && tx_params.pin == self.tx_pin
            && select_pins_desc.is_some() == self.mux.is_some();
        if !same_pins {
            return Err(ConfigError::new("Shared TMC uarts must use the same pins"));
        }
        let instance_id = match (&self.mux, select_pins_desc) {
            (Some(mux), Some(pins)) => Some(mux.get_instance_id(pins)?),
            _ => None,
        };
        let mut instances = self
            .instances
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        if !instances.insert((instance_id.clone(), addr)) {
            return Err(ConfigError::new(
                "Shared TMC uarts need unique address or select_pins polarity",
            ));
        }
        Ok(instance_id)
    }
}

/// Resolve a pin name to its firmware number
/// (`pin_number`, `mcu/resource/pin.rs`).
fn pin_number(
    mcu: &crate::core::klippy::mcu::Mcu,
    chip_name: &str,
    name: &str,
) -> Result<u32, McuError> {
    let dictionary = mcu
        .dictionary()
        .ok_or_else(|| McuError::Config("the MCU has no dictionary".to_string()))?;
    let number = dictionary
        .enumeration("pin")
        .and_then(|enumeration| enumeration.value(name))
        .ok_or_else(|| McuError::Config(format!("pin '{name}' is not on MCU '{chip_name}'")))?;
    u32::try_from(number)
        .map_err(|_| McuError::Config(format!("pin '{name}' does not fit a 32-bit pin number")))
}

// ===========================================================================
// Shared-uart registry (printer object `tmc_uart`)
// ===========================================================================

/// The printer object that hands out one [`TmcUartBitbang`] per
/// `(chip, rx, tx)`.
struct TmcUartRegistry {
    bitbangs: Mutex<HashMap<(String, String, String), Arc<TmcUartBitbang>>>,
}

impl PrinterObject for TmcUartRegistry {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

/// Look up (or build) the shared UART for a TMC section
/// (`lookup_tmc_uart_bitbang`).
///
/// # Errors
/// A missing/invalid `uart_pin`/`tx_pin`/`uart_address`, pins on different
/// MCUs, or a duplicate `(instance_id, address)`.
pub(crate) fn lookup_tmc_uart_bitbang(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
    max_addr: i64,
) -> Result<(Option<Vec<bool>>, i64, Arc<TmcUartBitbang>), ConfigError> {
    let ppins = printer
        .lookup_object_as::<PrinterPins>(PINS_OBJECT)
        .ok_or_else(|| ConfigError::new("the pins object is not registered"))?;
    let uart_pin = config.get("uart_pin", None)?;
    let rx_params = ppins
        .lookup_pin(&uart_pin, false, true, Some("tmc_uart_rx"))
        .map_err(|err| ConfigError::new(err.to_string()))?;
    let tx_params = match config.get("tx_pin", None) {
        Ok(tx_desc) => ppins
            .lookup_pin(&tx_desc, false, false, Some("tmc_uart_tx"))
            .map_err(|err| ConfigError::new(err.to_string()))?,
        Err(_) => rx_params.clone(),
    };
    if rx_params.chip_name != tx_params.chip_name {
        return Err(ConfigError::new(
            "TMC uart rx and tx pins must be on the same mcu",
        ));
    }
    let select_pins_desc = config.get_list("select_pins", ',');
    let addr = config.get_int_bounded("uart_address", Some(0), Some(0), Some(max_addr))?;

    let registry = match printer.lookup_object(TMC_UART_OBJECT) {
        Some(object) => {
            let any: Arc<dyn std::any::Any + Send + Sync> = object;
            match any.downcast::<TmcUartRegistry>() {
                Ok(registry) => registry,
                Err(_) => return Err(ConfigError::new("'tmc_uart' is not a TMC uart registry")),
            }
        }
        None => {
            let registry = Arc::new(TmcUartRegistry {
                bitbangs: Mutex::new(HashMap::new()),
            });
            printer
                .add_object(
                    TMC_UART_OBJECT,
                    Arc::clone(&registry) as Arc<dyn PrinterObject>,
                )
                .map_err(|err| ConfigError::new(err.to_string()))?;
            registry
        }
    };

    let key = (
        rx_params.chip_name.clone(),
        rx_params.pin.clone(),
        tx_params.pin.clone(),
    );
    let bitbang = {
        let mut bitbangs = registry
            .bitbangs
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        match bitbangs.get(&key) {
            Some(bitbang) => Arc::clone(bitbang),
            None => {
                let bitbang = TmcUartBitbang::build(
                    printer,
                    &rx_params.chip_name,
                    &rx_params,
                    &tx_params,
                    select_pins_desc.as_deref(),
                )?;
                bitbangs.insert(key, Arc::clone(&bitbang));
                bitbang
            }
        }
    };
    let instance_id =
        bitbang.register_instance(&rx_params, &tx_params, select_pins_desc.as_deref(), addr)?;
    Ok((instance_id, addr, bitbang))
}

// ===========================================================================
// The transport (`MCU_TMC_uart`)
// ===========================================================================

/// A TMC driver's UART link (`MCU_TMC_uart`), as a [`TmcTransport`].
///
/// The bitbang registration ([`lookup_tmc_uart_bitbang`]) happens in
/// [`TmcUart::new`], so a duplicate address or mismatched pins is a config
/// error at load; the live wire path itself is not wired (see the module docs).
pub struct TmcUart {
    printer: Weak<Printer>,
    name_to_reg: HashMap<String, u8>,
    tmc_frequency: f64,
}

impl TmcUart {
    /// Build the transport for a `[tmc22xx <stepper>]` section.
    ///
    /// # Errors
    /// Whatever [`lookup_tmc_uart_bitbang`] reports.
    pub fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        name_to_reg: HashMap<String, u8>,
        max_addr: i64,
        tmc_frequency: f64,
    ) -> Result<Self, ConfigError> {
        lookup_tmc_uart_bitbang(config, printer, max_addr)?;
        Ok(Self {
            printer: Arc::downgrade(printer),
            name_to_reg,
            tmc_frequency,
        })
    }

    /// Whether this run writes its MCU output to a file (`is_fileoutput`).
    fn fileoutput(&self) -> bool {
        self.printer
            .upgrade()
            .map(|printer| printer.is_fileoutput())
            .unwrap_or(false)
    }
}

impl TmcTransport for TmcUart {
    fn get_register_raw(&self, _reg_name: &str) -> Result<TmcRegister, McuError> {
        if self.fileoutput() {
            // Upstream `_do_get_register` returns data 0 under `debugoutput`.
            return Ok(TmcRegister {
                data: 0,
                receive_time: 0.,
            });
        }
        Err(McuError::Config(
            "TMC UART reads are only implemented for file-output (test) runs".to_string(),
        ))
    }

    fn set_register(
        &self,
        _reg_name: &str,
        _val: u32,
        _print_time: Option<f64>,
    ) -> Result<(), McuError> {
        if self.fileoutput() {
            // Upstream `set_register` returns before touching the bus.
            return Ok(());
        }
        Err(McuError::Config(
            "TMC UART writes are only implemented for file-output (test) runs".to_string(),
        ))
    }

    fn get_tmc_frequency(&self) -> Option<f64> {
        Some(self.tmc_frequency)
    }

    fn name_to_reg(&self) -> &HashMap<String, u8> {
        &self.name_to_reg
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_crc8_matches_a_known_read_frame() {
        // The read request `[0xf5, addr, reg]` with its CRC appended, before the
        // serial-bit framing: the CRC of the three header bytes.
        assert_eq!(crc8(&[0xf5, 0x00, 0x00]), 0x0f);
    }

    #[test]
    fn serial_framing_is_ten_bits_per_byte() {
        // One byte -> 10 bits -> 2 bytes (last byte padded).
        assert_eq!(add_serial_bits(&[0x00]).len(), 2);
        // Four bytes -> 40 bits -> exactly 5 bytes.
        assert_eq!(add_serial_bits(&[0, 0, 0, 0]).len(), 5);
    }

    #[test]
    fn a_read_request_round_trips_through_the_encoder() {
        // `_decode_read` verifies a read response against the write encoding of
        // the same register; a response encoded that way must decode.
        let reg = 0x6f;
        let val = 0x0000_0000;
        let encoded = encode_write(0x05, 0xff, reg, val);
        assert_eq!(decode_read(reg, &encoded), Some(val));
    }

    #[test]
    fn a_short_response_does_not_decode() {
        assert_eq!(decode_read(0x6f, &[0, 1, 2]), None);
    }
}
