//! The MCU as an I2C bus: `McuI2c`, upstream's `MCU_I2C` (`klippy/extras/bus.py:161`).
//!
//! An I2C device is created with `config_i2c`, then configured as either a
//! **hardware bus** (`i2c_set_bus`, using the MCU's I2C peripheral) or a
//! **software bus** (`i2c_set_sw_bus`, bit-banging on GPIO pins). After
//! configuration, data moves with `i2c_transfer` (legacy) or `i2c_read` /
//! `i2c_write` (new firmware: a read carries the register bytes it writes
//! first, a write carries only data).
//!
//! # Lifecycle
//!
//! 1. **Build time** (`McuI2c::new`): create oid, add `config_i2c`, register
//!    the config callback. Mode (hardware vs software) is decided here.
//! 2. **Config callback**: resolve bus name or pin numbers, pick the transfer
//!    style (`i2c_transfer` vs `i2c_write`/`i2c_read`), add the bus config
//!    command.
//! 3. **Runtime**: `transfer()` sends data and returns the response.

use std::sync::{Arc, Mutex};

use crate::core::klippy::cmd::i2c::{
    add_software_bus, ConfigI2c, I2cBusStatus, I2cRead, I2cSetBus, I2cTransfer, I2cWrite,
    SoftwareI2cBus,
};
use crate::core::klippy::mcu::{ConfigBuilder, Mcu, McuError};
use crate::core::klippy::pins::PrinterPins;

use super::pin::pin_number;

/// Default I2C clock speed in Hz (`klippy/extras/bus.py:220`).
pub const DEFAULT_SPEED: u32 = 100_000;

/// The mode an I2C device uses to talk to the bus.
#[derive(Debug, Clone)]
pub enum I2cMode {
    /// Hardware I2C peripheral: `i2c_set_bus` with a bus name.
    Hardware {
        /// Bus enumeration name (e.g. `"i2c_1"`), or `None` for the bus named
        /// `0`.
        bus: Option<String>,
        /// Clock frequency in Hz.
        speed: u32,
    },
    /// Software (bit-banged) I2C on GPIO pins: `i2c_set_sw_bus`.
    ///
    /// The pins are **names** as written in the config (aliases already
    /// resolved by the section); they become numbers in the config callback,
    /// when the firmware dictionary can be asked.
    Software {
        /// SCL pin name.
        scl_pin: String,
        /// SDA pin name.
        sda_pin: String,
        /// Clock frequency in Hz (used to compute `pulse_ticks`).
        speed: u32,
    },
}

/// The state a config callback captures and runtime methods read.
struct I2cState {
    /// The oid the firmware assigned.
    oid: Mutex<Option<u8>>,
    /// Which mode was configured (hardware vs software).
    mode: Mutex<I2cMode>,
    /// 7-bit device address (0..=127).
    address: Mutex<u8>,
    /// Whether the legacy `i2c_transfer` / `i2c_response` style is available.
    legacy_transfer: Mutex<bool>,
    /// Whether the new `i2c_write` / `i2c_read` style is available.
    new_transfer: Mutex<bool>,
}

/// One I2C device on an MCU.
///
/// Wraps the oid, mode, and address; dispatches transfers to the correct
/// firmware command pair based on what the dictionary declares at runtime.
pub struct McuI2c {
    state: Arc<I2cState>,
    /// Shared with the chip, so runtime sends reach the connected device.
    mcu: Arc<Mutex<Option<Arc<Mcu>>>>,
}

impl McuI2c {
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
        mode: I2cMode,
        address: u8,
    ) -> Self {
        let state = Arc::new(I2cState {
            oid: Mutex::new(None),
            mode: Mutex::new(mode),
            address: Mutex::new(address),
            legacy_transfer: Mutex::new(false),
            new_transfer: Mutex::new(false),
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
    fn oid(&self) -> Result<u8, McuError> {
        self.state
            .oid
            .lock()
            .expect("oid lock poisoned")
            .ok_or_else(|| McuError::Config("I2C device is not configured yet".to_string()))
    }

    /// The connected MCU, or an error if not connected.
    fn connected_mcu(&self) -> Result<Arc<Mcu>, McuError> {
        self.mcu
            .lock()
            .expect("mcu lock poisoned")
            .clone()
            .ok_or_else(|| McuError::Config("MCU is not connected".to_string()))
    }

    /// Send a read-write transaction.
    ///
    /// Writes `write_data` then reads `read_len` bytes. Uses the firmware's
    /// preferred transfer style (new split commands if available, legacy
    /// combined otherwise).
    ///
    /// # Errors
    /// Returns [`McuError::I2cBus`] if the transfer fails or the response
    /// indicates a bus error (NACK, timeout, etc.).
    pub async fn transfer(&self, write_data: &[u8], read_len: u32) -> Result<Vec<u8>, McuError> {
        let mcu = self.connected_mcu()?;
        let oid = self.oid()?;

        // Prefer the new split style: the firmware's `i2c_read` writes the
        // register bytes and then reads, so a combined transfer is one command.
        // (`i2c_write` is for a write with no read; sending it here too would
        // put the register bytes on the bus twice.)
        if *self.state.new_transfer.lock().expect("lock poisoned") {
            let read_cmd = I2cRead {
                oid,
                reg: write_data.to_vec(),
                read_len,
            };
            let response: crate::core::klippy::cmd::i2c::I2cReadResponse = tokio::time::timeout(
                std::time::Duration::from_secs(1),
                mcu.call_msg(&read_cmd, std::time::Duration::from_secs(1)),
            )
            .await
            .map_err(|_| {
                McuError::Call(crate::core::klippy::mcu::McuCallError::Timeout(
                    "i2c_read timed out".to_string(),
                ))
            })??;

            Ok(response.response)
        } else if *self.state.legacy_transfer.lock().expect("lock poisoned") {
            // Legacy combined transfer.
            let cmd = I2cTransfer {
                oid,
                write_data: write_data.to_vec(),
                read_len,
            };
            let response: crate::core::klippy::cmd::i2c::I2cResponse = tokio::time::timeout(
                std::time::Duration::from_secs(1),
                mcu.call_msg(&cmd, std::time::Duration::from_secs(1)),
            )
            .await
            .map_err(|_| {
                McuError::Call(crate::core::klippy::mcu::McuCallError::Timeout(
                    "i2c_transfer timed out".to_string(),
                ))
            })??;

            if !response.bus_status.is_ok() {
                return Err(McuError::I2cBus {
                    oid,
                    status: response.bus_status,
                });
            }

            Ok(response.response)
        } else {
            Err(McuError::I2cBus {
                oid,
                status: I2cBusStatus::Unknown(0),
            })
        }
    }

    /// Send a write-only transaction (no read data wanted).
    ///
    /// Uses `i2c_write` when the firmware has it. Otherwise the legacy
    /// `i2c_transfer` with `read_len=0` is used — which the firmware still
    /// answers with an `i2c_response`, so the reply is awaited and its status
    /// checked rather than left dangling in the receiver.
    ///
    /// # Errors
    /// As [`McuI2c::transfer`].
    pub async fn write(&self, data: &[u8]) -> Result<(), McuError> {
        let mcu = self.connected_mcu()?;
        let oid = self.oid()?;

        if *self.state.new_transfer.lock().expect("lock poisoned") {
            return mcu.send_msg(&I2cWrite {
                oid,
                data: data.to_vec(),
            });
        }

        // Legacy: a zero-length read still produces an `i2c_response`.
        let cmd = I2cTransfer {
            oid,
            write_data: data.to_vec(),
            read_len: 0,
        };
        let response: crate::core::klippy::cmd::i2c::I2cResponse = tokio::time::timeout(
            std::time::Duration::from_secs(1),
            mcu.call_msg(&cmd, std::time::Duration::from_secs(1)),
        )
        .await
        .map_err(|_| {
            McuError::Call(crate::core::klippy::mcu::McuCallError::Timeout(
                "i2c_transfer timed out".to_string(),
            ))
        })??;

        if !response.bus_status.is_ok() {
            return Err(McuError::I2cBus {
                oid,
                status: response.bus_status,
            });
        }
        Ok(())
    }

    /// The device address.
    pub fn address(&self) -> u8 {
        *self.state.address.lock().expect("address lock poisoned")
    }
}

impl I2cState {
    /// Build the bus config command and detect available transfer commands.
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

        // Add `config_i2c` to the config builder.
        builder
            .add_config_cmd(&ConfigI2c { oid })
            .map_err(|e| McuError::Config(format!("config_i2c: {e}")))?;

        // Add the bus config command based on mode.
        let mode = self.mode.lock().expect("mode lock poisoned").clone();
        let address = *self.address.lock().expect("address lock poisoned");
        match mode {
            I2cMode::Hardware { ref bus, speed } => {
                // The firmware declares `i2c_bus=%u`, so the name the config
                // wrote has to become the enumeration's value here: this port
                // encodes config commands instead of sending them as text, so
                // the firmware never sees the name to resolve.
                let bus = pins
                    .resolve_bus_value(mcu, "i2c_bus", bus.as_deref())
                    .map_err(|e| McuError::Config(format!("i2c bus: {e}")))?;
                builder
                    .add_config_cmd(&I2cSetBus {
                        oid,
                        bus,
                        rate: speed,
                        address,
                    })
                    .map_err(|e| McuError::Config(format!("i2c_set_bus: {e}")))?;
            }
            I2cMode::Software {
                ref scl_pin,
                ref sda_pin,
                speed,
            } => {
                // The pins arrive as names; the firmware wants numbers, which
                // only exist once the dictionary does. Aliases resolve first,
                // exactly as for a digital output.
                let scl = pins
                    .resolve_pin(chip_name, scl_pin)
                    .map_err(|e| McuError::Config(format!("i2c scl pin: {e}")))?;
                let scl_pin = pin_number(mcu, &scl, chip_name)
                    .map_err(|e| McuError::Config(format!("i2c scl pin: {e}")))?;
                let sda = pins
                    .resolve_pin(chip_name, sda_pin)
                    .map_err(|e| McuError::Config(format!("i2c sda pin: {e}")))?;
                let sda_pin = pin_number(mcu, &sda, chip_name)
                    .map_err(|e| McuError::Config(format!("i2c sda pin: {e}")))?;

                // Which command that is — and whether the tick count or the
                // raw rate goes out — is the command layer's business: newer
                // firmware gets `i2c_set_sw_bus`, older gets
                // `i2c_set_software_bus`.
                add_software_bus(
                    builder,
                    mcu,
                    &SoftwareI2cBus {
                        oid,
                        scl_pin,
                        sda_pin,
                        speed,
                        address,
                    },
                )?;
            }
        }

        // Detect available transfer commands.
        let has_legacy = mcu.try_lookup_command("i2c_transfer oid=%c write=%*s read_len=%u");
        let has_new = mcu
            .try_lookup_command("i2c_write oid=%c data=%*s")
            .is_some()
            && mcu
                .try_lookup_command("i2c_read oid=%c reg=%*s read_len=%u")
                .is_some();

        *self.legacy_transfer.lock().expect("lock poisoned") = has_legacy.is_some();
        *self.new_transfer.lock().expect("lock poisoned") = has_new;

        Ok(())
    }
}
