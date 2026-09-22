//! The MCU as an SPI bus: `McuSpi`, upstream's `MCU_SPI` (`klippy/extras/bus.py:42`).
//!
//! A device is created with `config_spi` (which also installs the chip-select
//! pin the firmware drives) or `config_spi_without_cs`, then configured as
//! either a **hardware bus** (`spi_set_bus`, using the MCU's SPI peripheral) or
//! a **software bus** (`spi_set_sw_bus`, bit-banging on GPIO pins). After that,
//! bytes move with `spi_send` (write only) or `spi_transfer` (full duplex).
//!
//! # Chip select
//!
//! The firmware owns the CS pin: every transfer asserts it, shifts the bytes,
//! and releases it. That is why the resource does not drive it itself, and why
//! a read that needs the device held across several commands is expressed as one
//! `spi_transfer` (a command byte plus the dummy bytes it clocks out).
//!
//! # Lifecycle
//!
//! 1. **Build time** (`McuSpi::new`): register the config callback. The mode
//!    (hardware vs software) and CS pin are decided here.
//! 2. **Config callback**: resolve the CS and bus/pin numbers, add
//!    `config_spi` / `spi_set_bus` / `spi_set_sw_bus`.
//! 3. **Runtime**: `transfer()` shifts bytes and returns what came back;
//!    `send()` shifts without waiting.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::core::klippy::cmd::spi::{
    add_software_bus, ConfigSpi, ConfigSpiWithoutCs, SoftwareSpiBus, SpiSend, SpiSetBus,
    SpiTransfer, SpiTransferResponse,
};
use crate::core::klippy::mcu::{ConfigBuilder, Mcu, McuError};
use crate::core::klippy::pins::{PinParams, PrinterPins};

use super::pin::pin_number;

/// How long a `spi_transfer` may take before it is reported as a timeout.
const TRANSFER_TIMEOUT: Duration = Duration::from_secs(1);

/// The mode an SPI device uses to talk to the bus.
#[derive(Debug, Clone)]
pub enum SpiMode {
    /// Hardware SPI peripheral: `spi_set_bus` with a bus name.
    Hardware {
        /// Bus enumeration name (e.g. `"spi1a"`), or `None` for the bus named
        /// `0`.
        bus: Option<String>,
        /// Clock frequency in Hz.
        speed: u32,
        /// SPI mode (CPOL/CPHA), 0..=3.
        mode: u8,
    },
    /// Software (bit-banged) SPI on GPIO pins: `spi_set_sw_bus`.
    ///
    /// The pins are **names** as written in the config (aliases already
    /// resolved by the section); they become numbers in the config callback,
    /// when the firmware dictionary can be asked.
    Software {
        /// MISO pin name.
        miso_pin: String,
        /// MOSI pin name.
        mosi_pin: String,
        /// SCLK pin name.
        sclk_pin: String,
        /// Clock frequency in Hz (used to compute `pulse_ticks`).
        speed: u32,
        /// SPI mode (CPOL/CPHA), 0..=3.
        mode: u8,
    },
}

/// The state a config callback captures and runtime methods read.
struct SpiState {
    /// The oid the firmware assigned.
    oid: Mutex<Option<u8>>,
    /// Which mode was configured (hardware vs software).
    mode: Mutex<SpiMode>,
    /// The chip-select pin, or `None` for `config_spi_without_cs`.
    cs_pin: Mutex<Option<PinParams>>,
    /// Whether the CS pin is active high.
    cs_active_high: Mutex<bool>,
}

/// One SPI device on an MCU.
pub struct McuSpi {
    state: Arc<SpiState>,
    /// Shared with the chip, so runtime sends reach the connected device.
    mcu: Arc<Mutex<Option<Arc<Mcu>>>>,
}

impl McuSpi {
    /// Build the resource and register its config callback.
    ///
    /// # Panics
    /// A resource is always built while the config file is loaded, before the
    /// configuration is frozen. The config callback runs later, at build time.
    pub fn new(
        config: Arc<ConfigBuilder>,
        pins: Arc<PrinterPins>,
        chip_name: &str,
        mcu: Arc<Mutex<Option<Arc<Mcu>>>>,
        mode: SpiMode,
        cs_pin: Option<PinParams>,
        cs_active_high: bool,
    ) -> Self {
        let state = Arc::new(SpiState {
            oid: Mutex::new(None),
            mode: Mutex::new(mode),
            cs_pin: Mutex::new(cs_pin),
            cs_active_high: Mutex::new(cs_active_high),
        });

        let callback_state = Arc::clone(&state);
        let callback_pins = Arc::downgrade(&pins);
        let callback_chip_name = chip_name.to_string();

        config
            .register_config_callback(Box::new(move |builder, mcu| {
                let pins = callback_pins
                    .upgrade()
                    .expect("the pins registry outlives the resources it built");
                callback_state.build(builder, mcu, &pins, &callback_chip_name)
            }))
            .expect("a resource is always built before the configuration is");

        Self { state, mcu }
    }

    /// The oid the firmware allocated.
    ///
    /// # Errors
    /// Returns [`McuError::Config`] before the configuration has been built.
    pub(crate) fn oid(&self) -> Result<u8, McuError> {
        self.state
            .oid
            .lock()
            .expect("oid lock poisoned")
            .ok_or_else(|| McuError::Config("SPI device is not configured yet".to_string()))
    }

    /// The connected MCU, or an error if not connected.
    pub(crate) fn connected_mcu(&self) -> Result<Arc<Mcu>, McuError> {
        self.mcu
            .lock()
            .expect("mcu lock poisoned")
            .clone()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))
    }

    /// Shift `data` out and return the bytes clocked in, the same length.
    ///
    /// A single call is one chip-select pulse, so a device protocol that needs
    /// the select held across a command and its data is expressed here as one
    /// buffer (e.g. `9f 00 00 00` for a JEDEC-ID read).
    ///
    /// # Errors
    /// Returns [`McuError::Call`] on a send failure or timeout, and
    /// [`McuError::Decode`] when the response cannot be decoded.
    pub async fn transfer(&self, data: &[u8]) -> Result<Vec<u8>, McuError> {
        let mcu = self.connected_mcu()?;
        let oid = self.oid()?;

        let command = SpiTransfer {
            oid,
            data: data.to_vec(),
        };
        let response: SpiTransferResponse =
            tokio::time::timeout(TRANSFER_TIMEOUT, mcu.call_msg(&command, TRANSFER_TIMEOUT))
                .await
                .map_err(|_| {
                    McuError::Call(crate::core::klippy::mcu::McuCallError::Timeout(
                        "spi_transfer timed out".to_string(),
                    ))
                })??;
        Ok(response.response)
    }

    /// Shift `data` out, ignoring what comes back.
    ///
    /// The firmware sends no response, so this only queues the command. It is
    /// the `spi_send` fast path (a write that needs no read).
    ///
    /// # Errors
    /// Returns [`McuError`] if the MCU is not connected or the send buffer is
    /// full.
    pub fn send(&self, data: &[u8]) -> Result<(), McuError> {
        let mcu = self.connected_mcu()?;
        let oid = self.oid()?;
        mcu.send_msg(&SpiSend {
            oid,
            data: data.to_vec(),
        })
    }

    /// The chip-select pin, or `None` when the device has none.
    pub fn cs_pin(&self) -> Option<PinParams> {
        self.state
            .cs_pin
            .lock()
            .expect("cs pin lock poisoned")
            .clone()
    }
}

impl SpiState {
    /// Build the device's config commands.
    ///
    /// This runs in the config callback, the first moment the dictionary exists.
    fn build(
        &self,
        builder: &ConfigBuilder,
        mcu: &Mcu,
        pins: &PrinterPins,
        chip_name: &str,
    ) -> Result<(), McuError> {
        // Allocate an oid — the firmware assigns it.
        let oid = builder.create_oid()?;
        *self.oid.lock().expect("oid lock poisoned") = Some(oid);

        // Chip select: the firmware drives it around every transfer.
        let cs_pin = self.cs_pin.lock().expect("cs pin lock poisoned").clone();
        let cs_active_high = *self.cs_active_high.lock().expect("cs lock poisoned");
        match cs_pin {
            Some(params) => {
                let pin = pins
                    .resolve_pin(chip_name, &params.pin)
                    .map_err(|e| McuError::Config(format!("spi cs pin: {e}")))?;
                let pin = pin_number(mcu, &pin, chip_name)
                    .map_err(|e| McuError::Config(format!("spi cs pin: {e}")))?;
                builder
                    .add_config_cmd(&ConfigSpi {
                        oid,
                        pin,
                        cs_active_high,
                    })
                    .map_err(|e| McuError::Config(format!("config_spi: {e}")))?;
            }
            None => {
                builder
                    .add_config_cmd(&ConfigSpiWithoutCs { oid })
                    .map_err(|e| McuError::Config(format!("config_spi_without_cs: {e}")))?;
            }
        }

        // Add the bus config command based on mode.
        let mode = self.mode.lock().expect("mode lock poisoned").clone();
        match mode {
            SpiMode::Hardware {
                ref bus,
                speed,
                mode,
            } => {
                // The firmware declares `spi_bus=%u`, so the name the config
                // wrote has to become the enumeration's value here: this port
                // encodes config commands instead of sending them as text, so
                // the firmware never sees the name to resolve.
                let bus = pins
                    .resolve_bus_value(mcu, "spi_bus", bus.as_deref())
                    .map_err(|e| McuError::Config(format!("spi bus: {e}")))?;
                builder
                    .add_config_cmd(&SpiSetBus {
                        oid,
                        bus,
                        mode,
                        rate: speed,
                    })
                    .map_err(|e| McuError::Config(format!("spi_set_bus: {e}")))?;
            }
            SpiMode::Software {
                ref miso_pin,
                ref mosi_pin,
                ref sclk_pin,
                speed,
                mode,
            } => {
                // The pins arrive as names; the firmware wants numbers, which
                // only exist once the dictionary does. Aliases resolve first,
                // exactly as for a digital output.
                let miso = pins
                    .resolve_pin(chip_name, miso_pin)
                    .map_err(|e| McuError::Config(format!("spi miso pin: {e}")))?;
                let miso = pin_number(mcu, &miso, chip_name)
                    .map_err(|e| McuError::Config(format!("spi miso pin: {e}")))?;
                let mosi = pins
                    .resolve_pin(chip_name, mosi_pin)
                    .map_err(|e| McuError::Config(format!("spi mosi pin: {e}")))?;
                let mosi = pin_number(mcu, &mosi, chip_name)
                    .map_err(|e| McuError::Config(format!("spi mosi pin: {e}")))?;
                let sclk = pins
                    .resolve_pin(chip_name, sclk_pin)
                    .map_err(|e| McuError::Config(format!("spi sclk pin: {e}")))?;
                let sclk = pin_number(mcu, &sclk, chip_name)
                    .map_err(|e| McuError::Config(format!("spi sclk pin: {e}")))?;

                // Which command that is — and whether the tick count or the
                // raw rate goes out — is the command layer's business: newer
                // firmware gets `spi_set_sw_bus`, older gets
                // `spi_set_software_bus`.
                add_software_bus(
                    builder,
                    mcu,
                    &SoftwareSpiBus {
                        oid,
                        miso_pin: miso,
                        mosi_pin: mosi,
                        sclk_pin: sclk,
                        mode,
                        speed,
                    },
                )?;
            }
        }

        Ok(())
    }
}
