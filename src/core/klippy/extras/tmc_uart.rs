//! TMC UART transport (upstream's `klippy/extras/tmc_uart.py`).
//!
//! | upstream | here |
//! |---|---|
//! | `MCU_analog_mux` | [`McuAnalogMux`] |
//! | `MCU_TMC_uart_bitbang` | [`TmcUartBitbang`] |
//! | `lookup_tmc_uart_bitbang` | [`lookup_tmc_uart_bitbang`] |
//! | `MCU_TMC_uart` | [`TmcUart`] (a [`TmcTransport`]) |
//!
//! | firmware | here |
//! |---|---|
//! | `config_tmcuart` | [`ConfigTmcuart`] |
//! | `tmcuart_send` | [`TmcuartSend`] |
//! | `tmcuart_response` | [`TmcuartResponse`] |
//!
//! # The wire path
//!
//! One exchange is `tmcuart_send oid=%c write=%*s read=%c` followed by
//! `tmcuart_response oid=%c read=%*s` (`src/tmcuart.c`). A read encodes the
//! request with [`encode_read`], sends it with `read = `[`UART_READ_LEN`] and
//! decodes the answer with [`decode_read`]; a write encodes `reg | 0x80` with
//! [`encode_write`] and sends it with `read = 0`, which makes the firmware
//! answer **nothing** — so the write path goes out through
//! [`Mcu::send_msg_clocked`](crate::core::klippy::mcu::Mcu::send_msg_clocked)
//! and is never waited on.
//!
//! Several chips share one bit-banged UART, so every answer carries the
//! bitbang's `oid` and the read path resolves it with
//! [`Mcu::call_msg_oid`](crate::core::klippy::mcu::Mcu::call_msg_oid): a
//! `tmcuart_response` belonging to another chip does not answer this caller.
//!
//! ## Deviations from upstream (`tmc_uart.py`)
//!
//! * **`receive_time`.** Upstream fills it from the response's
//!   `#receive_time`, the clock-synced print time stamped when the answer
//!   arrived. This port has no `#receive_time` channel, so an exchange dates
//!   its reading with a reactor monotonic sample taken **after** the call
//!   returns, converted through the MCU's estimate
//!   ([`McuObject::estimated_print_time`]): a live reading can be up to one
//!   round trip older than the stamp says.
//! * **Timeout.** Upstream's `lookup_query_command(..., is_async=True)` waits
//!   forever for a `tmcuart_response`; here one exchange gives up after
//!   [`UART_TIMEOUT`] so a silent chip reports an error instead of parking the
//!   reactor. Decode failures (a frame whose start/stop bits or CRC do not
//!   check out) are still retried upstream's five times.
//! * **The bus lock** is per [`TmcUartBitbang`]; upstream shares one lock per
//!   MCU (`lookup_tmc_uart_mutex`) to keep a slow board from being flooded with
//!   concurrent UART traffic. Which answer belongs to whom does not rest on the
//!   lock either way — the `oid` filter in
//!   [`Mcu::call_msg_oid`](crate::core::klippy::mcu::Mcu::call_msg_oid) does
//!   that.
//! * **Writing `IFCNT`.** Upstream sends it and then reads the counter back to
//!   compare against itself, which can only fail; here a write to `IFCNT` is
//!   sent and accepted without the IFCNT handshake.

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex, MutexGuard, Weak};
use std::time::Duration;

use serde_json::{json, Value};
use tracing::debug;

use crate::core::klippy::cmd::{McuCommand, McuResponse, Params};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::spi_device::mcu_object_name;
use crate::core::klippy::extras::tmc::{TmcRegister, TmcTransport};
use crate::core::klippy::extras::z_tilt::block_in_command;
use crate::core::klippy::mcu::{McuCallError, McuError, McuObject, SendClocks};
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

/// `tmcuart_send oid=%c write=%*s read=%c` — bit-bang `write` out of the tx
/// pin and clock `read` bytes back in on the rx pin.
///
/// `read = 0` is the write path: the firmware then sends **no**
/// `tmcuart_response` at all (`src/tmcuart.c`), so that frame must not be
/// waited on
/// ([`Mcu::send_msg_clocked`](crate::core::klippy::mcu::Mcu::send_msg_clocked)).
/// [`UART_READ_LEN`] is the read path's length — one full response frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TmcuartSend {
    /// The bitbang device oid (`config_tmcuart`'s).
    pub oid: u8,
    /// The serial-bit framed request bytes.
    pub write: Vec<u8>,
    /// How many bytes to clock in (0 for a write).
    pub read: u8,
}

impl McuCommand for TmcuartSend {
    const NAME: &'static str = "tmcuart_send";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::Bytes(self.write.clone()),
            ArgValue::UInt8(self.read),
        ]
    }
}

/// `tmcuart_response oid=%c read=%*s` — the bytes clocked in on the rx pin.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TmcuartResponse {
    /// The bitbang device oid this answer belongs to.
    pub oid: u8,
    /// The bytes clocked in.
    pub read: Vec<u8>,
}

impl McuResponse for TmcuartResponse {
    const NAME: &'static str = "tmcuart_response";

    fn decode(params: &Params<'_>) -> Result<Self, McuError> {
        Ok(Self {
            oid: params.get_u8("oid")?,
            read: params.get_bytes("read")?,
        })
    }
}

/// The bytes a read asks the firmware to clock in: one response frame, which
/// is 7 data bytes plus CRC at 10 bits per byte (`_encode_write` of one 32-bit
/// word).
const UART_READ_LEN: u8 = 10;

/// How long one wire exchange may take before it is given up.
///
/// Upstream waits forever (`tmc_uart.py:240`); a board with nothing on the UART
/// would park the reactor here.
const UART_TIMEOUT: Duration = Duration::from_secs(5);

/// How many times a read is retried before it is reported as failed
/// (`tmc_uart.py:231`, `for retry in range(5)`).
const UART_ATTEMPTS: u32 = 5;

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

/// The request that reads `reg` from the chip at `addr` (`reg_read`): a framed
/// read request, answered with [`UART_READ_LEN`] bytes.
fn read_command(oid: u8, addr: u8, reg: u8) -> TmcuartSend {
    TmcuartSend {
        oid,
        write: encode_read(addr, reg),
        read: UART_READ_LEN,
    }
}

/// The request that writes `val` to `reg` at `addr` (`reg_write`): a framed
/// write with the register's `0x80` bit set, and `read = 0` — the firmware
/// answers nothing, so this frame is sent and never waited on.
fn write_command(oid: u8, addr: u8, reg: u8, val: u32) -> TmcuartSend {
    TmcuartSend {
        oid,
        write: encode_write(UART_SYNC, addr, reg | 0x80, val),
        read: 0,
    }
}

/// Slack added on top of a frame's clocking time before the bus is considered
/// free again.
const FRAME_DRAIN_MARGIN: Duration = Duration::from_millis(1);

/// How long `bytes` bytes take on a `baud` UART: ten bits per byte (start, eight
/// data, stop) plus [`FRAME_DRAIN_MARGIN`].
///
/// The firmware drops a `tmcuart_send` that arrives while the UART is busy
/// (`src/tmcuart.c`, "host should retransmit"), and a write asks for no answer,
/// so the host has no signal that the frame is out — it waits this long instead
/// (see [`TmcUartBitbang::drain_time`]).
fn frame_time(bytes: usize, baud: f64) -> Duration {
    Duration::from_secs_f64(bytes as f64 * 10.0 / baud) + FRAME_DRAIN_MARGIN
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
    /// The digital outputs driving them, in the same order.
    outs: Vec<Arc<dyn DigitalOut>>,
    /// The value last written to each (`pin_values`), `None` before the first.
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
    ///
    /// Only the pins whose value changes are written, which is upstream's
    /// `if old != new` guard.
    ///
    /// # Errors
    /// A select pin that cannot be driven (the MCU is not connected yet).
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
    /// The printer, for the reactor clock a live reading is dated with.
    printer: Weak<Printer>,
    /// The MCU this peripheral sits on, resolved through its name at build
    /// time (the `[mcu]` object exists before any section is loaded).
    chip_name: String,
    mcu_object: Weak<McuObject>,
    /// The `config_tmcuart` oid.
    ///
    /// It is allocated in [`TmcUartBitbang::build`], i.e. while the section is
    /// still being parsed — before [`Printer`] connects — so it is an ordinary
    /// field and not something the config callback has to hand back.
    oid: u8,
    /// One live exchange at a time on this pair of pins (upstream's
    /// `lookup_tmc_uart_mutex`, scoped narrower — see the module docs).
    ///
    /// A plain [`Mutex`], because the holder runs the exchange through
    /// [`block_in_command`]: a `tokio` mutex could not be held across that
    /// call, and the lock guards nothing on the reactor side, so it cannot
    /// deadlock with the reactor itself. One [`TmcUart`] never recurses into
    /// its own read/write path, which is what would make this reentrant.
    bus: Mutex<()>,
    /// The baud the firmware clocks this UART at, resolved in the config
    /// callback (the MCU constant decides AVR's 9 kbaud). The write path uses
    /// it to wait out a frame — see [`TmcUartBitbang::drain_time`].
    baud: Arc<Mutex<f64>>,
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
        let baud_cell = Arc::new(Mutex::new(TMC_BAUD_RATE));
        let callback_baud = Arc::clone(&baud_cell);
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
                *callback_baud
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner()) = baud;
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
            printer: Arc::downgrade(printer),
            chip_name: chip_name.to_string(),
            mcu_object: Arc::downgrade(&mcu_object),
            oid,
            bus: Mutex::new(()),
            baud: baud_cell,
        }))
    }

    /// How long the firmware needs to clock `bytes` bytes out of this UART: the
    /// frame at 10 bits per byte, plus a millisecond of margin.
    ///
    /// It is the write path's pacing. `command_tmcuart_send` **silently drops**
    /// a request while the UART is busy (`src/tmcuart.c:107-109`, "host should
    /// retransmit"), so a second command sent before the first has finished
    /// clocking out is lost — that is why upstream's blocking transport never
    /// needed this and a fire-and-forget write does. Measured on the bench: the
    /// immediate `IFCNT` read-back of the very next write saw a stale value
    /// while the write itself did land, and back-to-back writes went missing.
    fn drain_time(&self, bytes: usize) -> Duration {
        let baud = *self
            .baud
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        frame_time(bytes, baud)
    }

    /// Hold the bus for one read/write sequence.
    fn bus_lock(&self) -> MutexGuard<'_, ()> {
        self.bus.lock().unwrap_or_else(|poison| poison.into_inner())
    }

    /// The `[mcu]` object this bitbang registered its `config_tmcuart` with.
    ///
    /// # Errors
    /// The object is gone, which only happens while the printer is being torn
    /// down.
    fn mcu_object(&self) -> Result<Arc<McuObject>, McuError> {
        self.mcu_object.upgrade().ok_or_else(|| {
            McuError::Config(format!(
                "the MCU '{}' of this TMC uart is gone",
                self.chip_name
            ))
        })
    }

    /// The print time an answer just received is dated with.
    ///
    /// Upstream takes the response's `#receive_time`; this port has no such
    /// channel, so the reading is dated with a reactor monotonic sample taken
    /// after the exchange returned, mapped onto the MCU's print-time estimate
    /// (module docs). `0.0` when there is nothing to map with yet, which is
    /// upstream's value for a cache.
    fn receive_time(&self, mcu_object: &McuObject) -> f64 {
        let Some(printer) = self.printer.upgrade() else {
            return 0.;
        };
        let eventtime = printer.reactor().monotonic();
        mcu_object.estimated_print_time(eventtime).unwrap_or(0.)
    }

    /// Read `reg` from the chip at `addr` (`reg_read`).
    ///
    /// `Ok(None)` is upstream's `data is None`: the bytes clocked in were not a
    /// well-formed frame (start/stop bits or CRC), which
    /// [`TmcUart::do_get_register`] retries.
    ///
    /// # Errors
    /// The MCU is not connected, the send failed, the answer did not arrive
    /// within [`UART_TIMEOUT`], or a select pin could not be driven.
    fn reg_read(
        &self,
        instance_id: Option<&[bool]>,
        addr: u8,
        reg: u8,
    ) -> Result<Option<TmcRegister>, McuError> {
        let mcu_object = self.mcu_object()?;
        let mcu = mcu_object.mcu().ok_or(McuError::NotIdentified)?;
        if let Some(mux) = &self.mux {
            mux.activate(instance_id.unwrap_or(&[]))?;
        }
        let command = read_command(self.oid, addr, reg);
        // `call_msg_oid` already carries `UART_TIMEOUT`; the `tokio` timeout is
        // the outer net the rest of the tree puts around a call that must not
        // park a command forever.
        let call =
            mcu.call_msg_oid::<TmcuartSend, TmcuartResponse>(&command, self.oid, UART_TIMEOUT);
        let response =
            match block_in_command(async { tokio::time::timeout(UART_TIMEOUT, call).await }) {
                Ok(Ok(response)) => response,
                Ok(Err(err)) => return Err(err),
                Err(_elapsed) => {
                    return Err(McuError::Call(McuCallError::Timeout(format!(
                        "no tmcuart_response for oid {} within {UART_TIMEOUT:?}",
                        self.oid
                    ))))
                }
            };
        // The answer is in: this is the closest this port gets to the moment
        // it arrived (module docs).
        let receive_time = self.receive_time(&mcu_object);
        Ok(decode_read(reg, &response.read).map(|data| TmcRegister { data, receive_time }))
    }

    /// Write `val` to `reg` at `addr` (`reg_write`).
    ///
    /// The frame asks for no response (`read = 0`), so this only sends: waiting
    /// for a `tmcuart_response` the firmware will not send would park for
    /// [`UART_TIMEOUT`]. `print_time` is upstream's `minclock` gate — the write
    /// may not reach the board before that print time.
    ///
    /// # Errors
    /// The MCU is not connected, the message could not be queued, or a select
    /// pin could not be driven.
    fn reg_write(
        &self,
        instance_id: Option<&[bool]>,
        addr: u8,
        reg: u8,
        val: u32,
        print_time: Option<f64>,
    ) -> Result<(), McuError> {
        let mcu_object = self.mcu_object()?;
        let mcu = mcu_object.mcu().ok_or(McuError::NotIdentified)?;
        // Upstream sends `minclock=0` for a write with no print time, which is
        // the same as no gate at all (`SendClocks::default`).
        let min_clock =
            print_time.and_then(|print_time| mcu_object.print_time_to_clock(print_time));
        if let Some(mux) = &self.mux {
            mux.activate(instance_id.unwrap_or(&[]))?;
        }
        let command = write_command(self.oid, addr, reg, val);
        let bytes = command.write.len();
        mcu.send_msg_clocked(
            &command,
            SendClocks {
                min_clock,
                req_clock: None,
            },
        )?;
        // Wait the frame out before anyone may queue the next command on this
        // UART: the firmware drops one that arrives while it is still clocking
        // (see [`TmcUartBitbang::drain_time`]).
        std::thread::sleep(self.drain_time(bytes));
        Ok(())
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
/// error at load. The live wire path is described in the module docs.
pub struct TmcUart {
    printer: Weak<Printer>,
    /// Upstream's `self.name`: the last word of the section name
    /// (`[tmc2209 stepper_x]` → `stepper_x`), used in the error messages.
    name: String,
    name_to_reg: HashMap<String, u8>,
    tmc_frequency: f64,
    /// The mux polarity this section selected (`None` without a mux).
    instance_id: Option<Vec<bool>>,
    /// The chip's UART address on the shared bus.
    addr: u8,
    /// The shared UART this section drives.
    bitbang: Arc<TmcUartBitbang>,
    /// The frame counter the last verified write left behind (`self.ifcnt`):
    /// the baseline the next write must land exactly one past.
    ifcnt: Mutex<Option<u32>>,
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
        let name = config
            .identifier()
            .split_whitespace()
            .next_back()
            .unwrap_or_default()
            .to_string();
        let (instance_id, addr, bitbang) = lookup_tmc_uart_bitbang(config, printer, max_addr)?;
        Ok(Self {
            printer: Arc::downgrade(printer),
            name,
            name_to_reg,
            tmc_frequency,
            instance_id,
            addr: addr as u8,
            bitbang,
            ifcnt: Mutex::new(None),
        })
    }

    /// Whether this run writes its MCU output to a file (`is_fileoutput`).
    fn fileoutput(&self) -> bool {
        self.printer
            .upgrade()
            .map(|printer| printer.is_fileoutput())
            .unwrap_or(false)
    }

    /// The UART address of `reg_name`.
    ///
    /// # Errors
    /// A register the chip's table does not name.
    fn reg_address(&self, reg_name: &str) -> Result<u8, McuError> {
        self.name_to_reg.get(reg_name).copied().ok_or_else(|| {
            McuError::Config(format!(
                "TMC uart '{}': unknown register name '{reg_name}'",
                self.name
            ))
        })
    }

    /// One register read, retried (`_do_get_register`).
    ///
    /// # Errors
    /// A register the table does not name, a bus error, or all
    /// [`UART_ATTEMPTS`] attempts failing to decode.
    fn do_get_register(&self, reg_name: &str) -> Result<TmcRegister, McuError> {
        let reg = self.reg_address(reg_name)?;
        read_register_with_retries(&self.name, reg_name, || {
            self.bitbang
                .reg_read(self.instance_id.as_deref(), self.addr, reg)
        })
    }
}

impl TmcTransport for TmcUart {
    fn get_register_raw(&self, reg_name: &str) -> Result<TmcRegister, McuError> {
        if self.fileoutput() {
            // Upstream `_do_get_register` returns data 0 under `debugoutput`,
            // before it touches the bus.
            return Ok(TmcRegister {
                data: 0,
                receive_time: 0.,
            });
        }
        let _bus = self.bitbang.bus_lock();
        self.do_get_register(reg_name)
    }

    fn set_register(
        &self,
        reg_name: &str,
        val: u32,
        print_time: Option<f64>,
    ) -> Result<(), McuError> {
        if self.fileoutput() {
            // Upstream `set_register` returns before touching the bus.
            return Ok(());
        }
        let reg = self.reg_address(reg_name)?;
        let _bus = self.bitbang.bus_lock();
        if self.name_to_reg.get("IFCNT") == Some(&reg) {
            // The write check below reads IFCNT back and requires it to be the
            // baseline's successor — writing IFCNT itself would compare against
            // the value just sent. Send and accept (module docs).
            return self.bitbang.reg_write(
                self.instance_id.as_deref(),
                self.addr,
                reg,
                val,
                print_time,
            );
        }
        let mut ifcnt = self
            .ifcnt
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        write_register_verified(
            &self.name,
            reg_name,
            val,
            &mut ifcnt,
            || {
                self.bitbang
                    .reg_write(self.instance_id.as_deref(), self.addr, reg, val, print_time)
            },
            || Ok(self.do_get_register("IFCNT")?.data),
            || Ok(self.do_get_register(reg_name)?.data),
        )
    }

    fn get_tmc_frequency(&self) -> Option<f64> {
        Some(self.tmc_frequency)
    }

    fn name_to_reg(&self) -> &HashMap<String, u8> {
        &self.name_to_reg
    }
}

/// [`UART_ATTEMPTS`] attempts at one read, retried while the answer is not a
/// well-formed frame (`_do_get_register`).
///
/// `read` is the whole attempt, so a decode failure and a bus error are told
/// apart by the caller: only the former is retried. The loop is a free function
/// so the retry count and the failure message are testable without a board.
///
/// # Errors
/// A bus error from any attempt, or the "Unable to read" report once every
/// attempt has failed to decode.
fn read_register_with_retries(
    name: &str,
    reg_name: &str,
    mut read: impl FnMut() -> Result<Option<TmcRegister>, McuError>,
) -> Result<TmcRegister, McuError> {
    for attempt in 1..=UART_ATTEMPTS {
        match read()? {
            Some(register) => return Ok(register),
            None => debug!(
                "tmc uart '{name}': register {reg_name} is not a valid frame \
                 (attempt {attempt}/{UART_ATTEMPTS})"
            ),
        }
    }
    Err(McuError::Config(format!(
        "Unable to read tmc uart '{name}' register {reg_name}"
    )))
}

/// [`UART_ATTEMPTS`] attempts at one write, each verified through `IFCNT`
/// (`set_register`) **or** a read-back of the register itself.
///
/// `ifcnt` is the cached counter (upstream's `self.ifcnt`): it is read once,
/// before the first write, and then kept at whatever the last attempt read
/// back. An attempt is accepted when either
///
/// * the counter came back as `(baseline + 1) & 0xff` — the chip saw exactly
///   one more frame (upstream's rule), or
/// * the register reads back the value that was just written.
///
/// The second arm exists because a TMC2209 on a single-wire UART really does
/// drop frames: on the bench a write changed `CHOPCONF` while `IFCNT` stayed
/// put, and back-to-back writes advanced it by one instead of five
/// (`command_tmcuart_send` drops a request that arrives while the line is
/// busy, `src/tmcuart.c`). Upstream never notices because its blocking
/// transport paces itself; here the register value is the authoritative
/// evidence, which is also what the SPI transports verify with.
///
/// # Errors
/// A bus error from `write`, `read_ifcnt` or `read_back`, or the "Unable to
/// write" report once every attempt has failed both checks.
fn write_register_verified(
    name: &str,
    reg_name: &str,
    val: u32,
    ifcnt: &mut Option<u32>,
    mut write: impl FnMut() -> Result<(), McuError>,
    mut read_ifcnt: impl FnMut() -> Result<u32, McuError>,
    mut read_back: impl FnMut() -> Result<u32, McuError>,
) -> Result<(), McuError> {
    for attempt in 1..=UART_ATTEMPTS {
        let baseline = match *ifcnt {
            Some(baseline) => baseline,
            None => {
                let read = read_ifcnt()?;
                *ifcnt = Some(read);
                read
            }
        };
        write()?;
        let after = read_ifcnt()?;
        *ifcnt = Some(after);
        if after == (baseline + 1) & 0xff {
            return Ok(());
        }
        debug!(
            "tmc uart '{name}': register {reg_name} write left IFCNT at {after} \
             (expected {}, attempt {attempt}/{UART_ATTEMPTS})",
            (baseline + 1) & 0xff
        );
        // The counter not moving does not prove the write was lost — check the
        // register itself before spending another attempt (see the docs).
        if read_back()? == val {
            debug!(
                "tmc uart '{name}': register {reg_name} holds {val:#x} although \
                 IFCNT stayed at {after}"
            );
            return Ok(());
        }
    }
    Err(McuError::Config(format!(
        "Unable to write tmc uart '{name}' register {reg_name}"
    )))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mcu::Dictionary;
    use crate::core::klippy::msg::parser::Parser;
    use serde_json::json;

    #[test]
    fn the_crc8_matches_a_known_read_frame() {
        // The read request `[0xf5, addr, reg]` with its CRC appended, before the
        // serial-bit framing: the CRC of the three header bytes.
        assert_eq!(crc8(&[0xf5, 0x00, 0x00]), 0x0f);
    }

    /// A write frame is eight bytes, ten bits each: 2.0 ms at the nominal
    /// 40 kbaud, 8.9 ms on AVR's 9 kbaud. The pacing that keeps the firmware
    /// from dropping the next command is that plus the margin.
    #[test]
    fn a_write_frame_takes_its_ten_bits_per_byte_to_drain() {
        assert_eq!(
            frame_time(8, TMC_BAUD_RATE),
            Duration::from_micros(2_000) + FRAME_DRAIN_MARGIN
        );
        assert_eq!(
            frame_time(8, TMC_BAUD_RATE_AVR),
            Duration::from_secs_f64(80.0 / TMC_BAUD_RATE_AVR) + FRAME_DRAIN_MARGIN
        );
        // The margin is what makes it a pacing rather than an estimate.
        assert!(frame_time(8, TMC_BAUD_RATE) > Duration::from_micros(2_000));
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

    // -----------------------------------------------------------------------
    // The wire commands
    // -----------------------------------------------------------------------

    /// A parser carrying the two `tmcuart` messages as `src/tmcuart.c` declares
    /// them, so a frame is checked against the firmware's own format string.
    fn parser() -> Parser {
        let dictionary = Dictionary::from_json(json!({
            "commands": {"tmcuart_send oid=%c write=%*s read=%c": 60},
            "responses": {"tmcuart_response oid=%c read=%*s": 61}
        }))
        .unwrap();
        let mut parser = Parser::new();
        dictionary.install(&mut parser).unwrap();
        parser
    }

    #[test]
    fn the_send_command_is_oid_bytes_and_a_read_length() {
        let command = read_command(3, 0x00, 0x6f);

        assert_eq!(TmcuartSend::NAME, "tmcuart_send");
        assert_eq!(
            command.args(),
            vec![
                ArgValue::UInt8(3),
                ArgValue::Bytes(encode_read(0x00, 0x6f)),
                ArgValue::UInt8(UART_READ_LEN),
            ]
        );
    }

    #[test]
    fn the_send_command_round_trips_through_the_firmware_format() {
        let parser = parser();
        let command = read_command(3, 0x00, 0x6f);

        let encoded = parser.encode(TmcuartSend::NAME, &command.args()).unwrap();
        let decoded = parser.decode(encoded).unwrap();

        assert_eq!(decoded[0].0.name, TmcuartSend::NAME);
        assert_eq!(decoded[0].1, command.args());
    }

    #[test]
    fn the_response_decodes_its_oid_and_bytes() {
        let parser = parser();
        let encoded = parser
            .encode(
                TmcuartResponse::NAME,
                &[ArgValue::UInt8(3), ArgValue::Bytes(vec![1, 2, 3])],
            )
            .unwrap();
        let decoded = parser.decode(encoded).unwrap();

        let response =
            TmcuartResponse::decode(&Params::new(Arc::clone(&decoded[0].0), &decoded[0].1))
                .unwrap();

        assert_eq!(TmcuartResponse::NAME, "tmcuart_response");
        assert_eq!(
            response,
            TmcuartResponse {
                oid: 3,
                read: vec![1, 2, 3]
            }
        );
    }

    #[test]
    fn a_requested_read_length_is_one_full_response_frame() {
        // `read = 10` is what the firmware clocks in for one register; the
        // decoder refuses anything else, so the two must agree.
        let frame = encode_write(0x05, 0xff, 0x6f, 0);
        assert_eq!(frame.len(), usize::from(UART_READ_LEN));
        assert_eq!(UART_READ_LEN, 10);
    }

    #[test]
    fn a_write_frame_asks_for_no_response() {
        // `read = 0` makes the firmware skip `tmcuart_response` entirely
        // (`src/tmcuart.c`), so the write path sends this frame through
        // `Mcu::send_msg_clocked` and never waits for an answer — a `call`
        // here would park until the timeout, every time.
        let command = write_command(3, 0x00, 0x6f, 0x1234_5678);

        assert_eq!(
            command,
            TmcuartSend {
                oid: 3,
                write: encode_write(UART_SYNC, 0x00, 0x6f | 0x80, 0x1234_5678),
                read: 0,
            }
        );
        assert_eq!(command.args()[2], ArgValue::UInt8(0));
    }

    // -----------------------------------------------------------------------
    // The retry loops
    // -----------------------------------------------------------------------

    #[test]
    fn a_write_is_accepted_once_the_frame_counter_advances() {
        // The first two frames never reach the chip (the counter stands still);
        // the third one does.
        let counter = std::cell::Cell::new(10u8);
        let mut writes = 0;
        let mut ifcnt = None;

        write_register_verified(
            "stepper_x",
            "GCONF",
            0x1234_5678,
            &mut ifcnt,
            || {
                writes += 1;
                if writes == 3 {
                    counter.set(counter.get().wrapping_add(1));
                }
                Ok(())
            },
            || Ok(u32::from(counter.get())),
            || Ok(0),
        )
        .unwrap();

        assert_eq!(writes, 3);
        assert_eq!(ifcnt, Some(11));
    }

    /// The bench's case: `CHOPCONF` took the new value while `IFCNT` stood
    /// still — a single-wire TMC2209 drops frames, so the register itself is
    /// what decides whether a write landed.
    #[test]
    fn a_write_is_accepted_when_the_register_reads_back_although_ifcnt_stood_still() {
        let mut writes = 0;
        let mut ifcnt = Some(4);

        write_register_verified(
            "stepper_x",
            "CHOPCONF",
            0x1403_0055,
            &mut ifcnt,
            || {
                writes += 1;
                Ok(())
            },
            || Ok(4),
            || Ok(0x1403_0055),
        )
        .unwrap();

        assert_eq!(writes, 1, "one write, then read the register back");
        assert_eq!(ifcnt, Some(4));
    }

    #[test]
    fn a_cached_frame_counter_saves_the_baseline_read() {
        let counter = std::cell::Cell::new(5u8);
        let mut reads = 0;
        let mut ifcnt = Some(5);

        write_register_verified(
            "stepper_x",
            "GCONF",
            0,
            &mut ifcnt,
            || {
                counter.set(counter.get().wrapping_add(1));
                Ok(())
            },
            || {
                reads += 1;
                Ok(u32::from(counter.get()))
            },
            || Ok(0),
        )
        .unwrap();

        assert_eq!(reads, 1, "only the read-back, not a fresh baseline");
        assert_eq!(ifcnt, Some(6));
    }

    #[test]
    fn a_write_whose_frame_counter_never_advances_gives_up_after_five() {
        let mut writes = 0;
        let mut reads = 0;
        let mut ifcnt = None;

        let err = write_register_verified(
            "stepper_x",
            "GCONF",
            0x42,
            &mut ifcnt,
            || {
                writes += 1;
                Ok(())
            },
            || {
                reads += 1;
                Ok(7)
            },
            || Ok(0),
        )
        .unwrap_err();

        assert_eq!(writes, UART_ATTEMPTS as usize);
        assert_eq!(
            reads,
            UART_ATTEMPTS as usize + 1,
            "one baseline read plus a read-back per attempt"
        );
        match err {
            McuError::Config(msg) => {
                assert_eq!(msg, "Unable to write tmc uart 'stepper_x' register GCONF")
            }
            other => panic!("expected the write report, got {other:?}"),
        }
    }

    #[test]
    fn a_bad_read_frame_is_retried_and_a_later_one_is_used() {
        let mut attempts = 0;

        let register = read_register_with_retries("stepper_x", "GCONF", || {
            attempts += 1;
            if attempts < 3 {
                Ok(None)
            } else {
                Ok(Some(TmcRegister {
                    data: 7,
                    receive_time: 1.5,
                }))
            }
        })
        .unwrap();

        assert_eq!(attempts, 3);
        assert_eq!(register.data, 7);
        assert_eq!(register.receive_time, 1.5);
    }

    #[test]
    fn five_bad_read_frames_report_upstream_s_message() {
        let mut attempts = 0;

        let err = read_register_with_retries("stepper_x", "GCONF", || {
            attempts += 1;
            Ok(None)
        })
        .unwrap_err();

        assert_eq!(attempts, UART_ATTEMPTS as usize);
        match err {
            McuError::Config(msg) => {
                assert_eq!(msg, "Unable to read tmc uart 'stepper_x' register GCONF")
            }
            other => panic!("expected the read report, got {other:?}"),
        }
    }

    #[test]
    fn a_bus_error_is_reported_without_a_retry() {
        let mut attempts = 0;

        let err = read_register_with_retries("stepper_x", "GCONF", || {
            attempts += 1;
            Err(McuError::Call(McuCallError::Timeout(
                "no tmcuart_response".to_string(),
            )))
        })
        .unwrap_err();

        assert_eq!(attempts, 1, "a timeout is not a frame error");
        assert!(
            matches!(err, McuError::Call(McuCallError::Timeout(_))),
            "{err:?}"
        );
    }
}
