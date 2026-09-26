//! The SPI transports for the TMC stepper drivers (upstream's
//! `tmc2130.MCU_TMC_SPI_chain` and `tmc2660.MCU_TMC2660_SPI`).
//!
//! A driver section hands its register table to one of these and reaches the
//! chip through [`TmcTransport`], so the chip modules never see bus addresses:
//!
//! | chip | transport | frame |
//! |---|---|---|
//! | `tmc2130` / `tmc5160` / `tmc2240` | [`TmcSpiChain`] | 5 bytes, `reg \| 0x80` marks a write |
//! | `tmc2660` | [`Tmc2660Spi`] | 3 bytes, reads rewrite the `rdsel` field first |
//!
//! Both build their bus with `mcu_spi_from_config`: the chain in SPI mode 3 at
//! 4 MHz (`tmc2130.py:191-192`), the 2660 in mode 0 at 4 MHz
//! (`tmc2660.py:196-197`). The chain pads every frame for its position on a
//! daisy chain (`tmc2130.py:196-232`), and both `chain_length` and
//! `chain_position` are read from the section (`tmc2130.py:239-259`).
//!
//! # Deliberate deviations from upstream
//!
//! **Nothing reaches the bus.** [`TmcTransport`] is synchronous while
//! [`McuSpi::transfer`] is async with a one-second timeout the fake firmware
//! never answers, so the transports implement file-output semantics only — the
//! same treatment [`crate::core::klippy::extras::tmc_uart`] gives UART. Under
//! file output a read answers 0 and a write is **dropped**; outside file output
//! both report `TMC SPI reads/writes are only implemented for file-output
//! (test) runs`. Upstream instead still queues each write with `spi_send` in
//! its `debugoutput` mode (`tmc2130.py:222-224`, `tmc2660.py:233-238`) and
//! verifies a chain write by reading it back, retrying five times and raising
//! `Unable to write tmc spi '<name>' register <reg>` (`tmc2130.py:286-293`);
//! that read-back verification has no place to run until the live path exists,
//! so a live write simply reports the refusal above.
//!
//! **One bus per section, not one per chip select.** Upstream hangs a single
//! `MCU_TMC_SPI_chain` off the shared `cs_pin` (a `share_type="tmc_spi_cs"`
//! lookup, `tmc2130.py:243-255`) so every section of a daisy chain reuses one
//! `spi_set_bus`. That share is not ported: [`mcu_spi_from_config`] always
//! shares the chip-select pin under the `"cs"` role, and each section builds
//! its own [`McuSpi`], so a chain issues one `config_spi`/`spi_set_bus` per
//! section. The chain's *bookkeeping* — the length the first section set and
//! the positions already claimed — is reproduced in [`TmcSpiChains`], so the
//! two chain errors (`TMC SPI chain must have same length`,
//! `TMC SPI chain can not have duplicate position`) fire as upstream's do when
//! the sections disagree.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::spi_device::mcu_spi_from_config;
use crate::core::klippy::extras::tmc::{FieldHelper, TmcRegister, TmcTransport};
use crate::core::klippy::mcu::{McuError, McuSpi};
use crate::core::klippy::printer::{Printer, PrinterObject};

/// The chain bus's SPI mode (`MCU_SPI_from_config(config, 3, ...)`).
const CHAIN_SPI_MODE: u8 = 3;

/// The 2660 bus's SPI mode (`MCU_SPI_from_config(config, 0, ...)`).
const TMC2660_SPI_MODE: u8 = 0;

/// The default clock both transports use (`default_speed=4000000`).
const DEFAULT_SPI_SPEED: u32 = 4_000_000;

/// The chain's frame width in bytes (`_build_cmd` pads by `* 5`).
const CHAIN_FRAME_LEN: usize = 5;

/// The 2660's frame width in bytes.
const TMC2660_FRAME_LEN: usize = 3;

/// The name the chain bookkeeping is registered under.
const CHAIN_REGISTRY_OBJECT: &str = "tmc_spi_chain";

/// Upstream's error when two sections on one chain disagree on its length
/// (`tmc2130.py:249`).
const CHAIN_LENGTH_MISMATCH: &str = "TMC SPI chain must have same length";

/// Upstream's error when a chain position is claimed twice
/// (`tmc2130.py:255`).
const CHAIN_DUPLICATE_POSITION: &str = "TMC SPI chain can not have duplicate position";

/// What both transports report for a read or write outside file output.
const FILEOUTPUT_ONLY: &str =
    "TMC SPI reads/writes are only implemented for file-output (test) runs";

// ===========================================================================
// The chain registry
// ===========================================================================

/// The bookkeeping upstream keeps on the shared chip-select pin
/// (`lookup_tmc_spi_chain`, `tmc2130.py:239-259`).
///
/// Upstream stores the shared `MCU_TMC_SPI_chain` itself on the pin's
/// `pin_params` and hangs `chain_len`/`taken_chain_positions` off it. This host
/// has no per-pin slot, and the bus object is deliberately not shared (module
/// docs), so the same bookkeeping lives in one hidden printer object keyed by
/// the `cs_pin` description.
#[derive(Default)]
struct TmcSpiChains {
    chains: Mutex<HashMap<String, ChainState>>,
}

/// One chain's negotiated length and the positions already claimed on it.
struct ChainState {
    chain_length: i64,
    taken_positions: Vec<i64>,
}

impl PrinterObject for TmcSpiChains {
    /// Upstream has no such object; it is registered to carry state, not to be
    /// queried.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

/// The chain registry for `printer`, creating it on first use.
///
/// # Errors
/// A duplicate object name (another part registered as `tmc_spi_chain`).
fn chain_registry(printer: &Arc<Printer>) -> Result<Arc<TmcSpiChains>, ConfigError> {
    if let Some(existing) = printer.lookup_object_as::<TmcSpiChains>(CHAIN_REGISTRY_OBJECT) {
        return Ok(existing);
    }
    let registry = Arc::new(TmcSpiChains::default());
    let object: Arc<dyn PrinterObject> = registry.clone();
    printer.add_object(CHAIN_REGISTRY_OBJECT, object)?;
    Ok(registry)
}

/// A section's place on its SPI chain: a length (1 when it is not on a chain)
/// and a position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ChainPlacement {
    length: i64,
    position: i64,
}

/// Resolve `chain_length`/`chain_position` for one section
/// (`lookup_tmc_spi_chain`, `tmc2130.py:239-259`).
///
/// A section with no `chain_length` is a plain single-device bus, at position
/// 1 of a length-1 chain. A section on a chain is checked against the first
/// section that claimed the same `cs_pin`: its length must match, and its
/// position must not already be taken.
///
/// # Errors
/// A `chain_length` below 2, a missing/out-of-range `chain_position`, or one of
/// the two chain errors ([`CHAIN_LENGTH_MISMATCH`],
/// [`CHAIN_DUPLICATE_POSITION`]).
fn lookup_tmc_spi_chain(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<ChainPlacement, ConfigError> {
    // `chain_length` is read with `minval=2` and no default; absent means the
    // section is not on a chain.
    let chain_length = if config.has("chain_length") {
        Some(config.get_int_bounded("chain_length", Some(0), Some(2), None)?)
    } else {
        None
    };
    let Some(chain_length) = chain_length else {
        return Ok(ChainPlacement {
            length: 1,
            position: 1,
        });
    };
    // Upstream reads `cs_pin` here (it is required on a chain) and reuses the
    // bus registered under it; that object is not shared here, only its
    // bookkeeping is.
    let cs_pin = config.get("cs_pin", None)?;
    let registry = chain_registry(printer)?;
    let mut chains = registry
        .chains
        .lock()
        .unwrap_or_else(|poison| poison.into_inner());
    let state = chains.entry(cs_pin).or_insert_with(|| ChainState {
        chain_length,
        taken_positions: Vec::new(),
    });
    if state.chain_length != chain_length {
        return Err(ConfigError::new(CHAIN_LENGTH_MISMATCH));
    }
    // Upstream reads the position only once the lengths agree, so a length
    // mismatch is reported even when the position is also wrong.
    let position = config.get_int_bounded("chain_position", None, Some(1), Some(chain_length))?;
    if state.taken_positions.contains(&position) {
        return Err(ConfigError::new(CHAIN_DUPLICATE_POSITION));
    }
    state.taken_positions.push(position);
    Ok(ChainPlacement {
        length: chain_length,
        position,
    })
}

// ===========================================================================
// The chain transport (`MCU_TMC_SPI_chain`)
// ===========================================================================

/// The 5-byte-frame SPI transport used by `tmc2130`, `tmc5160` and `tmc2240`
/// (`MCU_TMC_SPI_chain`, `tmc2130.py:185-294`).
///
/// On a daisy chain every frame is flanked by zero padding, so the bytes meant
/// for one chip sit at its position and the others see zeros. The write path
/// marks itself with `reg | 0x80`; the read path sends the bare register.
pub struct TmcSpiChain {
    printer: Weak<Printer>,
    name_to_reg: HashMap<String, u8>,
    tmc_frequency: f64,
    chain_length: usize,
    chain_position: usize,
    /// The bus this section built. Kept so the `config_spi` it added stays
    /// reachable; no frame is sent through it here (module docs).
    device: Arc<McuSpi>,
}

impl TmcSpiChain {
    /// Build the transport for one `[tmc2130 <stepper>]`-style section.
    ///
    /// `tmc_frequency` is the chip's TSTEP frequency (13.2 MHz for the 2130,
    /// 12 MHz for the 5160, 12.5 MHz for the 2240).
    ///
    /// # Errors
    /// The chain resolution ([`lookup_tmc_spi_chain`]) or a bad `spi_*` option
    /// or chip-select pin.
    pub fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        name_to_reg: HashMap<String, u8>,
        tmc_frequency: f64,
    ) -> Result<Self, ConfigError> {
        let placement = lookup_tmc_spi_chain(config, printer)?;
        let setup =
            mcu_spi_from_config(config, printer, CHAIN_SPI_MODE, "cs_pin", DEFAULT_SPI_SPEED)?;
        Ok(Self {
            printer: Arc::downgrade(printer),
            name_to_reg,
            tmc_frequency,
            chain_length: placement.length as usize,
            chain_position: placement.position as usize,
            device: setup.device,
        })
    }

    /// The bus this section built.
    pub fn device(&self) -> &Arc<McuSpi> {
        &self.device
    }

    /// Whether this run writes its MCU output to a file (`is_fileoutput`).
    fn fileoutput(&self) -> bool {
        self.printer
            .upgrade()
            .map(|printer| printer.is_fileoutput())
            .unwrap_or(false)
    }

    /// The bus address of `reg_name`.
    ///
    /// # Errors
    /// A register the driver's table does not name.
    fn reg_address(&self, reg_name: &str) -> Result<u8, McuError> {
        self.name_to_reg
            .get(reg_name)
            .copied()
            .ok_or_else(|| McuError::Config(format!("TMC SPI: unknown register name '{reg_name}'")))
    }

    /// The five bytes a read shifts out (`[reg, 0x00, 0x00, 0x00, 0x00]`).
    fn read_bytes(reg: u8) -> [u8; CHAIN_FRAME_LEN] {
        [reg, 0, 0, 0, 0]
    }

    /// The five bytes a write shifts out
    /// (`[(reg | 0x80) & 0xff, v>>24, v>>16, v>>8, v]`).
    fn write_bytes(reg: u8, val: u32) -> [u8; CHAIN_FRAME_LEN] {
        [
            reg | 0x80,
            (val >> 24) as u8,
            (val >> 16) as u8,
            (val >> 8) as u8,
            val as u8,
        ]
    }

    /// `_build_cmd`: `data` flanked by the chain's zero padding
    /// (`(chain_length - chain_position) * 5` before, `(chain_position - 1) * 5`
    /// after).
    fn pad_frame(&self, data: &[u8]) -> Vec<u8> {
        let prefix = (self.chain_length - self.chain_position) * CHAIN_FRAME_LEN;
        let suffix = (self.chain_position - 1) * CHAIN_FRAME_LEN;
        let mut frame = vec![0u8; prefix];
        frame.extend_from_slice(data);
        frame.resize(frame.len() + suffix, 0);
        frame
    }

    /// The bytes a read of `reg_name` shifts out, padded for this position.
    ///
    /// # Errors
    /// An unknown register name.
    pub fn read_command(&self, reg_name: &str) -> Result<Vec<u8>, McuError> {
        Ok(self.pad_frame(&Self::read_bytes(self.reg_address(reg_name)?)))
    }

    /// The bytes a write of `val` to `reg_name` shifts out, padded for this
    /// position.
    ///
    /// # Errors
    /// An unknown register name.
    pub fn write_command(&self, reg_name: &str, val: u32) -> Result<Vec<u8>, McuError> {
        Ok(self.pad_frame(&Self::write_bytes(self.reg_address(reg_name)?, val)))
    }

    /// The five response bytes this position owns
    /// (`pr[(chain_length - chain_position) * 5 : ... + 5]`).
    fn response_frame<'a>(&self, response: &'a [u8]) -> Option<&'a [u8]> {
        let start = (self.chain_length - self.chain_position) * CHAIN_FRAME_LEN;
        response.get(start..start + CHAIN_FRAME_LEN)
    }

    /// The status byte leading this position's response window (`spi_status`).
    pub fn decode_spi_status(&self, response: &[u8]) -> Option<u8> {
        Some(self.response_frame(response)?[0])
    }

    /// The data word in this position's response window
    /// (`pr[1] << 24 | pr[2] << 16 | pr[3] << 8 | pr[4]`).
    pub fn decode_data(&self, response: &[u8]) -> Option<u32> {
        let frame = self.response_frame(response)?;
        Some(u32::from_be_bytes([frame[1], frame[2], frame[3], frame[4]]))
    }
}

impl TmcTransport for TmcSpiChain {
    fn get_register_raw(&self, reg_name: &str) -> Result<TmcRegister, McuError> {
        // Upstream resolves the address before its debug-output branch
        // (`tmc2130.py:275-277`).
        let _reg = self.reg_address(reg_name)?;
        if self.fileoutput() {
            // Upstream answers 0 under `debugoutput` (`tmc2130.py:207-212`,
            // `MCU_TMC_SPI.get_register_raw`).
            return Ok(TmcRegister {
                data: 0,
                receive_time: 0.,
            });
        }
        Err(McuError::Config(FILEOUTPUT_ONLY.to_string()))
    }

    fn set_register(
        &self,
        reg_name: &str,
        _val: u32,
        _print_time: Option<f64>,
    ) -> Result<(), McuError> {
        let _reg = self.reg_address(reg_name)?;
        if self.fileoutput() {
            // Upstream queues the frame with `spi_send` here; this transport
            // sends nothing (module docs).
            return Ok(());
        }
        Err(McuError::Config(FILEOUTPUT_ONLY.to_string()))
    }

    fn get_tmc_frequency(&self) -> Option<f64> {
        Some(self.tmc_frequency)
    }

    fn name_to_reg(&self) -> &HashMap<String, u8> {
        &self.name_to_reg
    }
}

// ===========================================================================
// The 2660 transport (`MCU_TMC2660_SPI`)
// ===========================================================================

/// The 3-byte-frame SPI transport used by `tmc2660` (`MCU_TMC2660_SPI`,
/// `tmc2660.py:192-238`).
///
/// A write is `[((val >> 16) | reg) & 0xff, val >> 8, val]`. A read carries no
/// register of its own: the chip answers whichever register `rdsel` selects, so
/// reading one of `read_registers` first rewrites the `DRVCONF.rdsel` field
/// (queueing that value change before the read when it actually changes).
pub struct Tmc2660Spi {
    printer: Weak<Printer>,
    name_to_reg: HashMap<String, u8>,
    read_registers: Vec<String>,
    fields: Arc<FieldHelper>,
    /// The bus this section built. Kept so the `config_spi` it added stays
    /// reachable; no frame is sent through it here (module docs).
    device: Arc<McuSpi>,
}

impl Tmc2660Spi {
    /// Build the transport for a `[tmc2660 <stepper>]` section.
    ///
    /// `read_registers` is the chip's readable-register list
    /// (`READRSP@RDSEL0`..`2`), which fixes each register's `rdsel` index.
    ///
    /// # Errors
    /// A bad `spi_*` option or chip-select pin.
    pub fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        name_to_reg: HashMap<String, u8>,
        read_registers: Vec<String>,
        fields: Arc<FieldHelper>,
    ) -> Result<Self, ConfigError> {
        let setup = mcu_spi_from_config(
            config,
            printer,
            TMC2660_SPI_MODE,
            "cs_pin",
            DEFAULT_SPI_SPEED,
        )?;
        Ok(Self {
            printer: Arc::downgrade(printer),
            name_to_reg,
            read_registers,
            fields,
            device: setup.device,
        })
    }

    /// The bus this section built.
    pub fn device(&self) -> &Arc<McuSpi> {
        &self.device
    }

    /// Whether this run writes its MCU output to a file (`is_fileoutput`).
    fn fileoutput(&self) -> bool {
        self.printer
            .upgrade()
            .map(|printer| printer.is_fileoutput())
            .unwrap_or(false)
    }

    /// The bus address of `reg_name`.
    ///
    /// # Errors
    /// A register the driver's table does not name.
    fn reg_address(&self, reg_name: &str) -> Result<u8, McuError> {
        self.name_to_reg
            .get(reg_name)
            .copied()
            .ok_or_else(|| McuError::Config(format!("TMC SPI: unknown register name '{reg_name}'")))
    }

    /// The index of a readable register (`ReadRegisters.index(reg_name)`).
    ///
    /// # Errors
    /// A register the chip does not answer through `rdsel`.
    fn rdsel_index(&self, reg_name: &str) -> Result<i64, McuError> {
        self.read_registers
            .iter()
            .position(|name| name == reg_name)
            .map(|index| index as i64)
            .ok_or_else(|| {
                McuError::Config(format!("TMC2660: '{reg_name}' is not a readable register"))
            })
    }

    /// The three bytes a write shifts out
    /// (`[((val >> 16) | reg) & 0xff, val >> 8, val]`).
    fn write_bytes(reg: u8, val: u32) -> [u8; TMC2660_FRAME_LEN] {
        [
            (((val >> 16) | u32::from(reg)) & 0xff) as u8,
            ((val >> 8) & 0xff) as u8,
            (val & 0xff) as u8,
        ]
    }

    /// The bytes one write of `val` to `reg_name` shifts out.
    ///
    /// # Errors
    /// An unknown register name.
    pub fn write_command(
        &self,
        reg_name: &str,
        val: u32,
    ) -> Result<[u8; TMC2660_FRAME_LEN], McuError> {
        Ok(Self::write_bytes(self.reg_address(reg_name)?, val))
    }

    /// Prepare one read: rewrite `rdsel` and return the `DRVCONF` frame plus
    /// whether upstream would queue the value change before it.
    ///
    /// This mutates the `rdsel` field exactly as upstream's `get_register_raw`
    /// does (`tmc2660.py:213-227`), so a second read of the same register needs
    /// no setup frame. The response is three big-endian bytes
    /// ([`Tmc2660Spi::decode_data`]).
    ///
    /// # Errors
    /// An unknown `DRVCONF` address, or a register that is not readable.
    pub fn read_command(
        &self,
        reg_name: &str,
    ) -> Result<(bool, [u8; TMC2660_FRAME_LEN]), McuError> {
        let new_rdsel = self.rdsel_index(reg_name)?;
        let reg = self.reg_address("DRVCONF")?;
        let old_rdsel = self.fields.get_field("rdsel", None, None);
        let val = self.fields.set_field("rdsel", new_rdsel, None, None);
        Ok((new_rdsel != old_rdsel, Self::write_bytes(reg, val)))
    }

    /// The data word from a read response (`pr[0] << 16 | pr[1] << 8 | pr[2]`).
    pub fn decode_data(response: &[u8]) -> Option<u32> {
        let frame = response.get(..TMC2660_FRAME_LEN)?;
        Some((u32::from(frame[0]) << 16) | (u32::from(frame[1]) << 8) | u32::from(frame[2]))
    }
}

impl TmcTransport for Tmc2660Spi {
    fn get_register_raw(&self, reg_name: &str) -> Result<TmcRegister, McuError> {
        // Upstream indexes the read register before its debug-output branch
        // (`tmc2660.py:205-210`).
        let _rdsel = self.rdsel_index(reg_name)?;
        if self.fileoutput() {
            return Ok(TmcRegister {
                data: 0,
                receive_time: 0.,
            });
        }
        Err(McuError::Config(FILEOUTPUT_ONLY.to_string()))
    }

    fn set_register(
        &self,
        reg_name: &str,
        _val: u32,
        _print_time: Option<f64>,
    ) -> Result<(), McuError> {
        // No write verification: upstream's `set_register` only sends
        // (`tmc2660.py:233-238`).
        let _reg = self.reg_address(reg_name)?;
        if self.fileoutput() {
            return Ok(());
        }
        Err(McuError::Config(FILEOUTPUT_ONLY.to_string()))
    }

    fn get_tmc_frequency(&self) -> Option<f64> {
        // The 2660 has no TSTEP-based frequency (`tmc2660.py:236-237`).
        None
    }

    fn name_to_reg(&self) -> &HashMap<String, u8> {
        &self.name_to_reg
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::StartArgs;
    use crate::core::klippy::config::section::ConfigSection;
    use crate::core::klippy::config::value::ConfigValue;
    use crate::core::klippy::mcu::McuObject;
    use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[tmc2130 stepper_x]`-shaped section's options, as the loader would
    /// hand them over.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("tmc2130", Some("stepper_x"));
        for (key, value) in options {
            section.parameters.insert(
                key.to_lowercase(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    /// A printer with `pins` and one MCU, shaped like a file-output test run.
    fn printer(fileoutput: bool) -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        let object = Arc::new(
            McuObject::new(ConfigSection::new("mcu", None), &printer)
                .expect("the MCU registers its chip"),
        );
        printer
            .add_object("mcu", object as Arc<dyn PrinterObject>)
            .unwrap();
        if fileoutput {
            let mut start_args = StartArgs::collect("tmc_spi.cfg", None);
            start_args.debug_output = Some("_test_output".to_string());
            printer.set_start_args(Arc::new(start_args));
        }
        printer
    }

    /// A chain section's options: one chip select, and a place on the chain.
    fn chain_section(cs_pin: &str, chain_length: &str, chain_position: &str) -> ConfigSection {
        section(&[
            ("cs_pin", cs_pin),
            ("chain_length", chain_length),
            ("chain_position", chain_position),
        ])
    }

    fn chain_registers() -> HashMap<String, u8> {
        HashMap::from([
            ("GCONF".to_string(), 0x00),
            ("DRV_STATUS".to_string(), 0x6F),
        ])
    }

    fn tmc2660_registers() -> HashMap<String, u8> {
        HashMap::from([
            ("DRVCONF".to_string(), 0x0E),
            ("CHOPCONF".to_string(), 0x08),
            ("SGCSCONF".to_string(), 0x0C),
        ])
    }

    fn tmc2660_read_registers() -> Vec<String> {
        vec![
            "READRSP@RDSEL0".to_string(),
            "READRSP@RDSEL1".to_string(),
            "READRSP@RDSEL2".to_string(),
        ]
    }

    fn tmc2660_fields() -> Arc<FieldHelper> {
        Arc::new(FieldHelper::new(
            HashMap::from([(
                "DRVCONF".to_string(),
                HashMap::from([("rdsel".to_string(), 0x03 << 4)]),
            )]),
            &[],
            HashMap::new(),
        ))
    }

    // -- chain frames ------------------------------------------------------

    #[test]
    fn test_the_chain_frames_pad_by_position() {
        let printer = printer(true);
        let chains: Vec<TmcSpiChain> = (1..=3)
            .map(|position| {
                let position = position.to_string();
                TmcSpiChain::new(
                    &wrap(&chain_section("PD7", "3", &position)),
                    &printer,
                    chain_registers(),
                    13.2e6,
                )
                .expect("the chain section loads")
            })
            .collect();

        // Position 1 leads with `(3 - 1) * 5` zero bytes; its own frame is last.
        let read = chains[0]
            .read_command("DRV_STATUS")
            .expect("a known register");
        assert_eq!(read.len(), 15);
        assert_eq!(&read[..10], &[0u8; 10]);
        assert_eq!(&read[10..], &[0x6F, 0x00, 0x00, 0x00, 0x00]);

        // Position 2 has five zero bytes on each side.
        let write = chains[1]
            .write_command("GCONF", 0x0102_0304)
            .expect("a known register");
        assert_eq!(
            write,
            vec![0, 0, 0, 0, 0, 0x80, 0x01, 0x02, 0x03, 0x04, 0, 0, 0, 0, 0]
        );

        // Position 3 has no lead padding, so the frame is first.
        let write = chains[2]
            .write_command("GCONF", 0xDEAD_BEEF)
            .expect("a known register");
        assert_eq!(&write[..5], &[0x80, 0xDE, 0xAD, 0xBE, 0xEF]);
        assert_eq!(&write[5..], &[0u8; 10]);
    }

    #[test]
    fn test_a_chain_read_decodes_its_own_window() {
        let printer = printer(true);
        let chain = TmcSpiChain::new(
            &wrap(&chain_section("PD7", "3", "2")),
            &printer,
            chain_registers(),
            12e6,
        )
        .expect("the chain section loads");

        // Position 2 of a 3-long chain owns bytes 5..10.
        let mut response = vec![0u8; 15];
        response[5] = 0x42; // spi_status
        response[6..10].copy_from_slice(&[0x01, 0x02, 0x03, 0x04]);
        assert_eq!(chain.decode_spi_status(&response), Some(0x42));
        assert_eq!(chain.decode_data(&response), Some(0x0102_0304));
        // A short response has no window at this position.
        assert_eq!(chain.decode_data(&[0u8; 4]), None);
    }

    // -- chain options -----------------------------------------------------

    #[test]
    fn test_the_chain_options_are_read() {
        let printer = printer(true);
        // A single-device section is position 1 of a length-1 chain.
        let single = TmcSpiChain::new(
            &wrap(&section(&[("cs_pin", "PD7")])),
            &printer,
            chain_registers(),
            12.5e6,
        )
        .expect("a plain section loads");
        let read = single.read_command("GCONF").expect("a known register");
        assert_eq!(read, vec![0x00, 0, 0, 0, 0]);
    }

    #[test]
    fn test_a_chain_rejects_a_mismatched_length() {
        let printer = printer(true);
        TmcSpiChain::new(
            &wrap(&chain_section("PD7", "3", "1")),
            &printer,
            chain_registers(),
            13.2e6,
        )
        .expect("the first chain section loads");
        let err = TmcSpiChain::new(
            &wrap(&chain_section("PD7", "4", "2")),
            &printer,
            chain_registers(),
            13.2e6,
        )
        .err()
        .expect("a different length is refused");
        assert_eq!(err.to_string(), CHAIN_LENGTH_MISMATCH);
    }

    #[test]
    fn test_a_chain_rejects_a_duplicate_position() {
        let printer = printer(true);
        TmcSpiChain::new(
            &wrap(&chain_section("PD7", "3", "1")),
            &printer,
            chain_registers(),
            13.2e6,
        )
        .expect("the first chain section loads");
        let err = TmcSpiChain::new(
            &wrap(&chain_section("PD7", "3", "1")),
            &printer,
            chain_registers(),
            13.2e6,
        )
        .err()
        .expect("a repeated position is refused");
        assert_eq!(err.to_string(), CHAIN_DUPLICATE_POSITION);
    }

    #[test]
    fn test_an_out_of_range_chain_position_is_refused() {
        let printer = printer(true);
        let err = TmcSpiChain::new(
            &wrap(&chain_section("PD7", "3", "4")),
            &printer,
            chain_registers(),
            13.2e6,
        )
        .err()
        .expect("a position past the chain is refused");
        assert_eq!(
            err.to_string(),
            "Option 'chain_position' in section 'tmc2130 stepper_x' must have maximum of 3"
        );
    }

    // -- frequency ---------------------------------------------------------

    #[test]
    fn test_the_chain_reports_its_drivers_frequency() {
        let printer = printer(true);
        for frequency in [13.2e6, 12e6, 12.5e6] {
            let chain = TmcSpiChain::new(
                &wrap(&section(&[("cs_pin", "PD7")])),
                &printer,
                chain_registers(),
                frequency,
            )
            .expect("a plain section loads");
            assert_eq!(chain.get_tmc_frequency(), Some(frequency));
        }
    }

    // -- file output -------------------------------------------------------

    #[test]
    fn test_file_output_answers_zero_and_drops_writes() {
        let printer = printer(true);
        let chain = TmcSpiChain::new(
            &wrap(&section(&[("cs_pin", "PD7")])),
            &printer,
            chain_registers(),
            13.2e6,
        )
        .expect("a plain section loads");
        assert_eq!(chain.get_register("DRV_STATUS").unwrap(), 0);
        chain
            .set_register("GCONF", 0x1234, None)
            .expect("a write is dropped, not sent");
        assert!(chain.name_to_reg().contains_key("DRV_STATUS"));

        let spi = Tmc2660Spi::new(
            &wrap(&section(&[("cs_pin", "PD7")])),
            &printer,
            tmc2660_registers(),
            tmc2660_read_registers(),
            tmc2660_fields(),
        )
        .expect("a 2660 section loads");
        assert_eq!(spi.get_register("READRSP@RDSEL2").unwrap(), 0);
        spi.set_register("CHOPCONF", 0x1234, None)
            .expect("a write is dropped, not sent");
        assert_eq!(spi.get_tmc_frequency(), None);
        assert!(spi.name_to_reg().contains_key("DRVCONF"));
    }

    #[test]
    fn test_a_live_bus_is_refused() {
        let printer = printer(false);
        let chain = TmcSpiChain::new(
            &wrap(&section(&[("cs_pin", "PD7")])),
            &printer,
            chain_registers(),
            13.2e6,
        )
        .expect("a plain section loads");
        assert_eq!(
            refusal(chain.get_register_raw("GCONF")),
            FILEOUTPUT_ONLY,
            "a live read reports the refusal"
        );
        assert_eq!(
            refusal(chain.set_register("GCONF", 0, None)),
            FILEOUTPUT_ONLY,
            "a live write reports the refusal"
        );

        let spi = Tmc2660Spi::new(
            &wrap(&section(&[("cs_pin", "PD7")])),
            &printer,
            tmc2660_registers(),
            tmc2660_read_registers(),
            tmc2660_fields(),
        )
        .expect("a 2660 section loads");
        assert_eq!(
            refusal(spi.get_register_raw("READRSP@RDSEL2")),
            FILEOUTPUT_ONLY
        );
        assert_eq!(
            refusal(spi.set_register("CHOPCONF", 0, None)),
            FILEOUTPUT_ONLY
        );
    }

    /// The message of a refusal (a [`McuError::Config`]).
    fn refusal(result: Result<impl std::fmt::Debug, McuError>) -> String {
        match result {
            Err(McuError::Config(message)) => message,
            other => panic!("expected a config refusal, got {other:?}"),
        }
    }

    // -- 2660 frames -------------------------------------------------------

    #[test]
    fn test_a_2660_write_shifts_three_bytes() {
        let printer = printer(true);
        let spi = Tmc2660Spi::new(
            &wrap(&section(&[("cs_pin", "PD7")])),
            &printer,
            tmc2660_registers(),
            tmc2660_read_registers(),
            tmc2660_fields(),
        )
        .expect("a 2660 section loads");
        // `[((val >> 16) | reg) & 0xff, val >> 8, val]`.
        assert_eq!(
            spi.write_command("CHOPCONF", 0x0001_2345).unwrap(),
            [0x09, 0x23, 0x45]
        );
        assert_eq!(
            spi.write_command("SGCSCONF", 0x0000_001D).unwrap(),
            [0x0C, 0x00, 0x1D]
        );
    }

    #[test]
    fn test_a_2660_read_rewrites_rdsel_first() {
        let printer = printer(true);
        let spi = Tmc2660Spi::new(
            &wrap(&section(&[("cs_pin", "PD7")])),
            &printer,
            tmc2660_registers(),
            tmc2660_read_registers(),
            tmc2660_fields(),
        )
        .expect("a 2660 section loads");
        // RDSEL0 is index 0, which `rdsel` already is: no setup send.
        assert_eq!(
            spi.read_command("READRSP@RDSEL0").unwrap(),
            (false, [0x0E, 0x00, 0x00])
        );
        // Moving to RDSEL1 changes the field, so upstream queues it first.
        assert_eq!(
            spi.read_command("READRSP@RDSEL1").unwrap(),
            (true, [0x0E, 0x00, 0x10])
        );
        // Reading RDSEL1 again leaves `rdsel` alone.
        assert_eq!(
            spi.read_command("READRSP@RDSEL1").unwrap(),
            (false, [0x0E, 0x00, 0x10])
        );
        // The response is three bytes big-endian.
        assert_eq!(
            Tmc2660Spi::decode_data(&[0xAB, 0xCD, 0xEF]),
            Some(0x00AB_CDEF)
        );
        assert_eq!(Tmc2660Spi::decode_data(&[0x00, 0x01]), None);
    }
}
