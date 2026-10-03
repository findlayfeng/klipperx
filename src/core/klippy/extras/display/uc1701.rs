//! `uc1701` — the UC1701 (128x64 graphics) panel driver, and the IO wrappers
//! and reset helper the SSD1306 shares with it (upstream keeps all of it in one
//! file, `klippy/extras/display/uc1701.py`, because `display.py:18-19` maps
//! `lcd_type: uc1701`, `ssd1306` and `sh1106` to classes defined there).
//!
//! A `[display]` section with `lcd_type: uc1701` builds a [`Uc1701`]. The panel
//! sits behind a "4 wire" SPI bus: the firmware owns `cs_pin` (every byte is one
//! chip-select pulse) and the host's extra `a0_pin` line says whether the bytes
//! are commands or RAM data (`uc1701.py:118-127`).
//!
//! | framework call | here | upstream |
//! |---|---|---|
//! | `init()` | reset, the 17-command power-up list, `0xA5`/`0xA4`, flush | `uc1701.py:174-196` |
//! | `clear()` | blank the eight page framebuffers | `uc1701.py:110-113` |
//! | `flush()` | batch the framebuffer differences and send them | `uc1701.py:28-52` |
//! | `set_glyphs()` | — (see below) | `uc1701.py:64-70` |
//!
//! # What is not here
//!
//! * **Text and glyph drawing.** [`super::display`] never draws, so
//!   `write_text`/`write_glyph`/`write_graphics` — and with them
//!   `_swizzle_bits` and the font — are not ported; `set_glyphs` therefore has
//!   nothing to store, because upstream only caches the swizzled icons there
//!   for `write_glyph` to draw later.
//! * **`BACKGROUND_PRIORITY_CLOCK`** (`uc1701.py:11`): upstream sends every
//!   byte with `reqclock=0x7fffffff00000000`, which lets the panel writes
//!   overtake queued g-code. This host's [`McuCommand`](crate::core::klippy::cmd::McuCommand) carries no clock, so
//!   commands go out in call order.
//! * **The reset helper's queue stall** (`uc1701.py:163-165`): upstream orders
//!   the three reset writes by `minclock` *and* notes that the last one
//!   "force[s] a delay to any subsequent commands on the command queue", which
//!   is what holds the panel's power-up bytes back until reset is released.
//!   This host schedules the three writes on the MCU clock
//!   ([`ResetHelper`]) but has no way to stall the queue, so a panel that needs
//!   `rst_pin` gets its init bytes in call order rather than after `+0.300s`.
//! * **`SH1106`** (`uc1701.py:238-241`): the 132-column sibling is a separate
//!   `lcd_type` and is still refused by [`super::display`].
//! * **The I2C status reply.** Upstream's SSD1306 sends `i2c_transfer …
//!   read_len=0` as an ordinary command and only *listens* for the
//!   `i2c_response` status (`bus.py:256-259`); this host queues the same bytes
//!   without waiting (see [`PanelIo::send`]).

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::cmd::i2c::I2cTransfer;
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::spi_device::{mcu_object_name, mcu_spi_from_config};
use crate::core::klippy::mcu::{I2cMode, McuError, McuI2c, McuObject, McuSpi};
use crate::core::klippy::pins::{DigitalOut, PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};
use crate::core::klippy::reactor::Reactor;

use super::display::{Glyph, LcdChip, SentMessage};

/// Upstream's `MCU_SPI_from_config(config, 0, default_speed=10000000)`
/// (`uc1701.py:120`): SPI mode 0 at 10 MHz.
pub(crate) const DEFAULT_SPI_SPEED: u32 = 10_000_000;

/// Upstream's default contrast (`uc1701.py:172`).
pub(crate) const DEFAULT_UC1701_CONTRAST: i64 = 40;

/// The panel's framebuffer: one page of `columns` bytes, plus the copy the
/// firmware has already been told about (`uc1701.py:21-23`).
#[derive(Debug)]
pub(crate) struct Page {
    /// What the screen should show.
    data: Vec<u8>,
    /// What the firmware's copy holds; the difference is what a flush sends.
    synced: Vec<u8>,
}

/// Upstream's `DisplayBase` framebuffers (`uc1701.py:15-27`): eight pages of
/// `columns` bytes. The page buffer starts blank and its "already sent" copy is
/// `~`, so the first flush sends the whole screen.
#[derive(Debug)]
pub(crate) struct DisplayBase {
    /// Bytes per page — one page is eight pixel rows of `columns` columns.
    columns: usize,
    /// The eight pages, in row order.
    pages: Vec<Page>,
}

impl DisplayBase {
    /// Eight blank pages, each with an unsent copy (`uc1701.py:21-23`).
    pub(crate) fn new(columns: usize) -> Self {
        Self {
            columns,
            pages: (0..8)
                .map(|_| Page {
                    data: vec![0; columns],
                    synced: vec![b'~'; columns],
                })
                .collect(),
        }
    }

    /// Upstream's `DisplayBase.clear` (`uc1701.py:110-113`): every page goes
    /// back to zeros.
    pub(crate) fn clear(&mut self) {
        for page in &mut self.pages {
            page.data = vec![0; self.columns];
        }
    }

    /// Upstream's `DisplayBase.flush` (`uc1701.py:28-52`): for every page that
    /// changed, send the changed runs — set the page and column registers, then
    /// shift the bytes in as data.
    ///
    /// # Errors
    /// The first send that fails stops the flush with that error, and the page
    /// keeps its old "sent" copy, so the next flush tries again.
    pub(crate) fn flush(&self, io: &PanelIo) -> Result<(), McuError> {
        for (index, page) in self.pages.iter().enumerate() {
            if page.data == page.synced {
                continue;
            }
            for (pos, count) in changed_runs(&page.data, &page.synced) {
                // Page start (`0xb0 | page`), then the column address as its
                // high and low nibbles (`uc1701.py:46-49`).
                let page_register = 0xb0 | ((index as u8) & 0x0f);
                let column_msb = 0x10 | (((pos >> 4) as u8) & 0x0f);
                let column_lsb = (pos & 0x0f) as u8;
                io.send(&[page_register, column_msb, column_lsb], false)?;
                io.send(&page.data[pos..pos + count], true)?;
            }
        }
        Ok(())
    }
}

/// Upstream's flush batching (`uc1701.py:34-42`): the positions of every
/// changed byte as `(position, length)` runs, with runs closer than five bytes
/// joined while the run ahead is shorter than sixteen.
pub(crate) fn changed_runs(data: &[u8], synced: &[u8]) -> Vec<(usize, usize)> {
    let mut diffs: Vec<(usize, usize)> = data
        .iter()
        .zip(synced.iter())
        .enumerate()
        .filter(|(_, (new, old))| new != old)
        .map(|(i, _)| (i, 1))
        .collect();
    for i in (0..diffs.len().saturating_sub(1)).rev() {
        let (pos, _count) = diffs[i];
        let (next_pos, next_count) = diffs[i + 1];
        if pos + 5 >= next_pos && next_count < 16 {
            diffs[i] = (pos, next_count + (next_pos - pos));
            diffs.remove(i + 1);
        }
    }
    diffs
}

// ===========================================================================
// The IO wrappers
// ===========================================================================

/// Where the panel's bytes go: upstream's `SPI4wire` (`uc1701.py:118-127`) and
/// `I2C` (`uc1701.py:130-142`) wrappers behind one type, because a chip holds
/// one of them.
enum Transport {
    /// A "4 wire" SPI bus: the firmware drives `cs_pin` around each write and
    /// the host's data/-control line says what the bytes are.
    Spi {
        /// The SPI resource (`cs_pin` and the bus).
        spi: Arc<McuSpi>,
        /// The `a0_pin` (UC1701) / `dc_pin` (SPI SSD1306) line.
        dc: Arc<dyn DigitalOut>,
    },
    /// The I2C bus (SSD1306 without a `cs_pin`).
    I2c {
        /// The I2C resource.
        i2c: Arc<McuI2c>,
    },
}

/// One panel's IO, with every message it handed to the firmware (tests and
/// diagnostics).
pub(crate) struct PanelIo {
    /// Where the bytes go.
    transport: Transport,
    /// The MCU the bus and the pins live on.
    mcu: Arc<McuObject>,
    /// Every message handed to the firmware, in order.
    sent: Mutex<Vec<SentMessage>>,
}

impl PanelIo {
    /// Upstream's `SPI4wire(config, data_pin_name)` (`uc1701.py:119-123`): the
    /// SPI bus from `cs_pin` plus one extra data/control line.
    ///
    /// # Errors
    /// A missing `cs_pin` (upstream's `config.get(pin_option)`, `bus.py:129`),
    /// a missing `data_pin_option`, an unresolvable or already-used pin, a pin
    /// that is not on the panel's MCU (`Pin <desc> must be on mcu <mcu>`,
    /// `bus.py:343-345`), or an unknown `spi_mcu`.
    pub(crate) fn spi4wire(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        data_pin_option: &str,
    ) -> Result<(Self, Arc<McuObject>), ConfigError> {
        // `cs_pin` is required: unlike a temperature sensor, the panel has no
        // "no chip select" mode.
        config.get("cs_pin", None)?;
        let setup = mcu_spi_from_config(config, printer.as_ref(), 0, "cs_pin", DEFAULT_SPI_SPEED)?;
        let mcu = panel_mcu(config, printer, "spi_mcu")?;
        let description = config.get(data_pin_option, None)?;
        let dc = digital_out(config, printer, &description, mcu.name())?;
        let io = Self {
            transport: Transport::Spi {
                spi: setup.device,
                dc,
            },
            mcu: Arc::clone(&mcu),
            sent: Mutex::new(Vec::new()),
        };
        Ok((io, mcu))
    }

    /// Upstream's `I2C(config, default_addr)` (`uc1701.py:131-134`): the I2C
    /// bus at the SSD1306's default address of 60.
    ///
    /// # Errors
    /// An out-of-range `i2c_address`, a too-slow `i2c_speed`, one software pin
    /// without the other, or an unknown `i2c_mcu`.
    pub(crate) fn i2c(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
    ) -> Result<(Self, Arc<McuObject>), ConfigError> {
        let identifier = config.identifier();
        let mcu = panel_mcu(config, printer, "i2c_mcu")?;

        // `MCU_I2C_from_config(config, default_addr=60, default_speed=400000)`
        // (`uc1701.py:131-134`).
        let address = config.get_int("i2c_address", Some(60))?;
        if !(0..=127).contains(&address) {
            return Err(ConfigError::new(format!(
                "Option 'i2c_address' in section '{identifier}' must be between 0 and 127"
            )));
        }
        let speed = config.get_int("i2c_speed", Some(400_000))?;
        if speed < 100_000 {
            return Err(ConfigError::new(format!(
                "Option 'i2c_speed' in section '{identifier}' must be at least 100000"
            )));
        }
        let speed = speed as u32;

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let mode = match (
            config.get_str("i2c_software_scl_pin"),
            config.get_str("i2c_software_sda_pin"),
        ) {
            (Some(scl), Some(sda)) => {
                let scl_params = pins
                    .lookup_pin(&scl, false, false, Some("scl"))
                    .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
                let sda_params = pins
                    .lookup_pin(&sda, false, false, Some("sda"))
                    .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
                if scl_params.chip_name != mcu.name() || sda_params.chip_name != mcu.name() {
                    return Err(ConfigError::new(format!(
                        "Section '{identifier}': i2c pins must be on the same mcu '{}'",
                        mcu.name()
                    )));
                }
                I2cMode::Software {
                    scl_pin: scl_params.pin,
                    sda_pin: sda_params.pin,
                    speed,
                }
            }
            (None, None) => I2cMode::Hardware {
                bus: config.get_str("i2c_bus"),
                speed,
            },
            _ => {
                return Err(ConfigError::new(format!(
                    "Section '{identifier}': both 'i2c_software_scl_pin' and \
                     'i2c_software_sda_pin' must be set"
                )));
            }
        };
        let i2c = mcu.setup_i2c(mode, address as u8);
        let io = Self {
            transport: Transport::I2c { i2c },
            mcu: Arc::clone(&mcu),
            sent: Mutex::new(Vec::new()),
        };
        Ok((io, mcu))
    }

    /// Send one message to the panel: `is_data` selects the data/control line
    /// (`SPI4wire.send`, `uc1701.py:124-127`) or the I2C control byte
    /// (`I2C.send`, `uc1701.py:135-142`).
    ///
    /// The I2C half is upstream's `async_write_only` path: `i2c_transfer` with
    /// `read_len=0` sent as an ordinary command (`bus.py:256-259`), because
    /// upstream only listens for the status reply and never waits for it. This
    /// host does not wait either — [`McuI2c::write`] would need the reply, and
    /// the refresh path is synchronous — so the bytes are queued and the reply,
    /// if the firmware sends one, is logged as unhandled by the MCU receive
    /// task.
    ///
    /// # Errors
    /// The SPI half reports an unconnected or not-yet-configured MCU; the I2C
    /// half reports that, plus a firmware without `i2c_transfer`.
    pub(crate) fn send(&self, cmds: &[u8], is_data: bool) -> Result<(), McuError> {
        match &self.transport {
            Transport::Spi { spi, dc } => {
                dc.update_digital_out(is_data)?;
                spi.send(cmds)?;
            }
            Transport::I2c { i2c } => {
                let oid = i2c.oid()?;
                let mcu = self
                    .mcu
                    .mcu()
                    .ok_or_else(|| McuError::Config("the MCU is not connected".to_string()))?;
                let mut data = Vec::with_capacity(cmds.len() + 1);
                data.push(if is_data { 0x40 } else { 0x00 });
                data.extend_from_slice(cmds);
                mcu.send_msg(&I2cTransfer {
                    oid,
                    write_data: data,
                    read_len: 0,
                })?;
            }
        }
        self.sent
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .push(SentMessage {
                is_data,
                bytes: cmds.to_vec(),
            });
        Ok(())
    }

    /// Every message handed to the firmware, in order.
    pub(crate) fn sent(&self) -> Vec<SentMessage> {
        self.sent
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }

    /// Whether this panel writes over I2C rather than SPI.
    #[cfg(test)]
    pub(crate) fn is_i2c(&self) -> bool {
        matches!(self.transport, Transport::I2c { .. })
    }
}

/// The MCU a panel's bus lives on: `spi_mcu` / `i2c_mcu`, defaulting to the
/// main one, the way `MCU_SPI_from_config` takes it from the chip-select pin
/// (`bus.py:136`) and `MCU_I2C_from_config` from `i2c_mcu` (`bus.py:306`).
fn panel_mcu(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
    option: &str,
) -> Result<Arc<McuObject>, ConfigError> {
    let identifier = config.identifier();
    let mcu_name = config
        .get_str(option)
        .map(|text| text.trim().to_string())
        .unwrap_or_else(|| "mcu".to_string());
    printer
        .lookup_object_as::<McuObject>(&mcu_object_name(&mcu_name))
        .ok_or_else(|| {
            ConfigError::new(format!("Section '{identifier}': unknown MCU '{mcu_name}'"))
        })
}

/// One of the panel's own GPIO lines built as an output: upstream's
/// `MCU_bus_digital_out(io_bus.get_mcu(), pin_desc, …)` (`uc1701.py:122-123`,
/// `:150-151`).
///
/// The pin is looked up without inversion or a pull-up (`pins.lookup_pin`'s
/// defaults, `pins.py:96-97`) and must sit on the MCU the bus is on
/// (`bus.py:343-345`).
///
/// # Errors
/// A malformed pin description, a pin on another MCU, or a pin already used.
fn digital_out(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
    description: &str,
    mcu_name: &str,
) -> Result<Arc<dyn DigitalOut>, ConfigError> {
    let identifier = config.identifier();
    let pins = printer
        .lookup_object_as::<PrinterPins>(PINS_OBJECT)
        .expect("the loader registers `pins` before any section");
    let params = pins
        .parse_pin(description, false, false)
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
    if params.chip_name != mcu_name {
        return Err(ConfigError::new(format!(
            "Pin {description} must be on mcu {mcu_name}"
        )));
    }
    pins.setup_digital_out(description, None)
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))
}

// ===========================================================================
// The reset helper
// ===========================================================================

/// Upstream's `ResetHelper` (`uc1701.py:145-165`): an optional pin that is
/// pulled low, then high, then held high 100/200/300 ms into startup.
pub(crate) struct ResetHelper {
    /// The reset line, or `None` when the config writes no `rst_pin`/
    /// `reset_pin` (`uc1701.py:148-149`).
    pub(crate) out: Option<Arc<dyn DigitalOut>>,
    /// The MCU the pin is on, for its clock.
    mcu: Arc<McuObject>,
    /// The machine's clock, to place the toggles.
    reactor: Arc<dyn Reactor>,
}

impl ResetHelper {
    /// Read the reset pin option (`rst_pin` for the UC1701, `reset_pin` for
    /// the SSD1306) and build the line when it is written.
    ///
    /// # Errors
    /// As [`digital_out`].
    pub(crate) fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        option: &str,
        mcu: &Arc<McuObject>,
    ) -> Result<Self, ConfigError> {
        let out = match config.get_str(option) {
            None => None,
            Some(description) => Some(digital_out(config, printer, &description, mcu.name())?),
        };
        Ok(Self {
            out,
            mcu: Arc::clone(mcu),
            reactor: printer.reactor(),
        })
    }

    /// Upstream's `ResetHelper.init` (`uc1701.py:152-165`): low at `+0.100s`,
    /// high at `+0.200s`, and high again at `+0.300s` (which upstream uses to
    /// hold the command queue back — module docs).
    ///
    /// Nothing happens when the config writes no pin, or when the MCU is not
    /// connected yet: a panel without a reset line is the common case, and a
    /// refresh that runs before the machine is up only logs.
    pub(crate) fn init(&self) {
        let Some(out) = &self.out else {
            return;
        };
        let eventtime = self.reactor.monotonic();
        let Some(print_time) = self.mcu.estimated_print_time(eventtime) else {
            warn!("the panel's reset line cannot be timed: the MCU is not connected");
            return;
        };
        for (delay, level) in [(0.100, false), (0.200, true), (0.300, true)] {
            let Some(clock) = self.mcu.print_time_to_clock(print_time + delay) else {
                warn!("the panel's reset line cannot be timed: the MCU clock is unknown");
                return;
            };
            if let Err(err) = out.queue_digital_out(clock as u32, level) {
                warn!("the panel could not drive its reset line: {err}");
                return;
            }
        }
    }
}

// ===========================================================================
// The UC1701 panel
// ===========================================================================

/// The 17 power-up commands (`uc1701.py:176-192`), with `contrast` in the
/// electronic-volume slot.
pub(crate) fn uc1701_init_commands(contrast: i64) -> Vec<u8> {
    vec![
        0xE2,           // System reset
        0x40,           // Set display to start at line 0
        0xA0,           // Set SEG direction
        0xC8,           // Set COM Direction
        0xA2,           // Set Bias = 1/9
        0x2C,           // Boost ON
        0x2E,           // Voltage regulator on
        0x2F,           // Voltage follower on
        0xF8,           // Set booster ratio
        0x00,           // Booster ratio value (4x)
        0x23,           // Set resistor ratio (3)
        0x81,           // Set Electronic Volume
        contrast as u8, // Electronic Volume value
        0xAC,           // Set static indicator off
        0x00,           // NOP
        0xA6,           // Disable Inverse
        0xAF,           // Set display enable
    ]
}

/// One `lcd_type: uc1701` panel (upstream's `UC1701`, `uc1701.py:168-196`).
pub struct Uc1701 {
    /// The eight page framebuffers.
    base: Mutex<DisplayBase>,
    /// Where the bytes go.
    io: PanelIo,
    /// The optional reset line.
    reset: ResetHelper,
    /// `contrast`, bounded `0..=63` (`uc1701.py:172`).
    contrast: i64,
}

impl Uc1701 {
    /// Read the options and build the panel.
    ///
    /// Upstream's `UC1701.__init__` (`uc1701.py:169-173`): the SPI bus and the
    /// `a0_pin`, then `contrast`, then the optional `rst_pin`.
    ///
    /// # Errors
    /// As [`PanelIo::spi4wire`] and [`ResetHelper::new`], plus a `contrast`
    /// outside `0..=63`.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let (io, mcu) = PanelIo::spi4wire(config, printer, "a0_pin")?;
        let contrast =
            config.get_int_bounded("contrast", Some(DEFAULT_UC1701_CONTRAST), Some(0), Some(63))?;
        let reset = ResetHelper::new(config, printer, "rst_pin", &mcu)?;
        Ok(Self {
            base: Mutex::new(DisplayBase::new(128)),
            io,
            reset,
            contrast,
        })
    }

    /// The framebuffer.
    fn base(&self) -> std::sync::MutexGuard<'_, DisplayBase> {
        self.base
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl LcdChip for Uc1701 {
    /// Upstream's `UC1701.init` (`uc1701.py:174-196`): the reset toggle, the
    /// 17 power-up commands, "display all", "normal display", then the first
    /// flush.
    fn init(&self) {
        self.reset.init();
        let commands = uc1701_init_commands(self.contrast);
        if let Err(err) = self.io.send(&commands, false) {
            warn!("uc1701: could not initialise the panel: {err}");
            return;
        }
        if let Err(err) = self.io.send(&[0xA5], false) {
            warn!("uc1701: could not initialise the panel: {err}");
            return;
        }
        if let Err(err) = self.io.send(&[0xA4], false) {
            warn!("uc1701: could not initialise the panel: {err}");
            return;
        }
        self.flush();
    }

    /// Upstream's `DisplayBase.clear` (`uc1701.py:110-113`).
    fn clear(&self) {
        self.base().clear();
    }

    /// Upstream's `DisplayBase.flush` (`uc1701.py:28-52`).
    fn flush(&self) {
        if let Err(err) = self.base().flush(&self.io) {
            warn!("uc1701: could not flush the panel: {err}");
        }
    }

    /// The panel's size in characters (`uc1701.py:114-115`).
    fn get_dimensions(&self) -> (usize, usize) {
        (16, 4)
    }

    /// Upstream's `DisplayBase.set_glyphs` stores the icons so `write_glyph`
    /// can draw them (`uc1701.py:64-70`); nothing draws here (module docs), so
    /// there is nothing to keep.
    fn set_glyphs(&self, _glyphs: &BTreeMap<String, Glyph>) {}

    /// Every message this panel has handed to the firmware, in order.
    fn sent_messages(&self) -> Vec<SentMessage> {
        self.io.sent()
    }
}

impl PrinterObject for Uc1701 {
    /// Upstream's `UC1701` has no `get_status`, so it is not client-visible.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for Uc1701 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Uc1701")
            .field("contrast", &self.contrast)
            .finish_non_exhaustive()
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::section::ConfigSection;
    use crate::core::klippy::config::value::ConfigValue;
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[display]` section's options, as the loader would hand them over.
    fn display_section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("display", None);
        for (key, value) in options {
            section.parameters.insert(
                key.to_lowercase(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A printer with `pins` and two MCUs, as the loader builds them before any
    /// section runs: `mcu`, and `board2` for the "must be on one MCU" checks.
    fn printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(PINS_OBJECT, Arc::new(PrinterPins::new()))
            .unwrap();
        for (name, sub) in [("mcu", None), ("mcu board2", Some("board2"))] {
            let object = Arc::new(
                McuObject::new(ConfigSection::new("mcu", sub), &printer)
                    .expect("the MCU registers its chip"),
            );
            printer
                .add_object(name, object as Arc<dyn PrinterObject>)
                .unwrap();
        }
        printer
    }

    /// The corpus's `lcd_type: uc1701` shape (`printer-creality-cr20-2018.cfg`).
    const UC1701_SECTION: &[(&str, &str)] = &[
        ("lcd_type", "uc1701"),
        ("cs_pin", "PA3"),
        ("a0_pin", "PA5"),
        ("encoder_pins", "^PC4, ^PC6"),
        ("click_pin", "^!PC2"),
    ];

    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    // -- options -----------------------------------------------------------

    #[test]
    fn test_the_section_loads_and_reads_every_option() {
        let printer = printer();
        let section = display_section(UC1701_SECTION);
        let chip = Uc1701::new(&wrap(&section), &printer).expect("the panel loads");

        assert_eq!(chip.contrast, DEFAULT_UC1701_CONTRAST);
        assert_eq!(chip.get_dimensions(), (16, 4));
        assert_eq!(chip.sent_message_count(), 0);
        // No `rst_pin` in the corpus's shape, so no reset line is built.
        assert!(chip.reset.out.is_none());
    }

    #[test]
    fn test_contrast_is_bounded_the_way_upstream_bounds_it() {
        // `config.getint('contrast', 40, minval=0, maxval=63)`
        // (`uc1701.py:172`). A fresh machine each time: the pins a panel
        // reserves are refused to the next one.
        let mut options = UC1701_SECTION.to_vec();
        options.push(("contrast", "55"));
        let section = display_section(&options);
        let chip = Uc1701::new(&wrap(&section), &printer()).unwrap();
        assert_eq!(chip.contrast, 55);

        let mut options = UC1701_SECTION.to_vec();
        options.push(("contrast", "64"));
        let section = display_section(&options);
        let err = Uc1701::new(&wrap(&section), &printer())
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'contrast' in section 'display' must have maximum of 63"
        );
    }

    // -- pin and bus validation -------------------------------------------

    #[test]
    fn test_the_chip_select_and_data_pins_are_required() {
        let printer = printer();

        let section = display_section(&[("a0_pin", "PA5")]);
        let err = Uc1701::new(&wrap(&section), &printer)
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'cs_pin' in section 'display' must be specified"
        );

        let section = display_section(&[("cs_pin", "PA3")]);
        let err = Uc1701::new(&wrap(&section), &printer)
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'a0_pin' in section 'display' must be specified"
        );
    }

    #[test]
    fn test_the_data_pin_must_be_on_the_panels_mcu() {
        // Upstream's `MCU_bus_digital_out` refuses a pin on another MCU
        // (`bus.py:343-345`).
        let printer = printer();
        let section = display_section(&[("cs_pin", "PA3"), ("a0_pin", "board2:PA1")]);

        let err = Uc1701::new(&wrap(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(err.to_string(), "Pin board2:PA1 must be on mcu mcu");
    }

    #[test]
    fn test_a_partly_written_software_bus_is_refused() {
        // The SPI bus has to be either hardware or all three software pins
        // (`spi_device::mcu_spi_from_config`).
        let printer = printer();
        let mut options = UC1701_SECTION.to_vec();
        options.push(("spi_software_sclk_pin", "PB0"));
        let section = display_section(&options);

        let err = Uc1701::new(&wrap(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert!(
            err.to_string()
                .contains("all three of 'spi_software_miso_pin'"),
            "{err}"
        );
        assert!(
            err.to_string()
                .contains("'spi_software_sclk_pin' must be set"),
            "{err}"
        );
    }

    // -- the power-up sequence --------------------------------------------

    #[test]
    fn test_the_init_sequence_is_upstreams_power_up_list() {
        let commands = uc1701_init_commands(40);
        assert_eq!(
            commands,
            vec![
                0xE2, 0x40, 0xA0, 0xC8, 0xA2, 0x2C, 0x2E, 0x2F, 0xF8, 0x00, 0x23, 0x81, 40, 0xAC,
                0x00, 0xA6, 0xAF,
            ]
        );
        // The contrast lands in the electronic-volume slot (`uc1701.py:187-188`).
        assert_eq!(uc1701_init_commands(55)[12], 55);
        assert_eq!(uc1701_init_commands(63)[12], 63);
    }

    #[test]
    fn test_the_reset_line_is_only_built_when_the_config_writes_it() {
        let printer = printer();
        let mut options = UC1701_SECTION.to_vec();
        options.push(("rst_pin", "PC1"));
        let section = display_section(&options);

        let chip = Uc1701::new(&wrap(&section), &printer).expect("the panel loads");
        assert!(chip.reset.out.is_some());
        // Not connected, so there is no clock to place the toggles on; the
        // helper says so instead of failing the printer.
        chip.reset.init();
    }

    // -- the framebuffers --------------------------------------------------

    #[test]
    fn test_the_framebuffers_start_unsent() {
        // Eight blank pages, each with `~` as its "already sent" copy, so the
        // first flush sends the whole screen (`uc1701.py:21-23`).
        let base = DisplayBase::new(128);
        assert_eq!(base.columns, 128);
        assert_eq!(base.pages.len(), 8);
        for page in &base.pages {
            assert_eq!(page.data, vec![0; 128]);
            assert_eq!(page.synced, vec![b'~'; 128]);
        }
    }

    #[test]
    fn test_clear_blanks_every_page() {
        let mut base = DisplayBase::new(128);
        for page in &mut base.pages {
            page.data = vec![0xff; 128];
        }
        base.clear();
        for page in &base.pages {
            assert_eq!(page.data, vec![0; 128]);
        }
    }

    #[test]
    fn test_close_changes_are_batched_into_one_run() {
        // The upstream rule: a change runs up to five bytes ahead while the run
        // ahead is shorter than sixteen (`uc1701.py:34-42`).
        let synced = vec![0; 8];
        let mut data = vec![0; 8];
        data[0] = 1;
        data[3] = 1;
        assert_eq!(changed_runs(&data, &synced), vec![(0, 4)]);

        // Far-apart changes stay separate.
        let synced = vec![0; 32];
        let mut data = vec![0; 32];
        data[0] = 1;
        data[20] = 1;
        assert_eq!(changed_runs(&data, &synced), vec![(0, 1), (20, 1)]);

        // A twenty-byte run stops growing once the run ahead is sixteen bytes,
        // and the four bytes in front of it are batched separately
        // (`uc1701.py:34-42`).
        let mut data = vec![0; 32];
        for byte in data.iter_mut().take(20) {
            *byte = 1;
        }
        assert_eq!(changed_runs(&data, &synced), vec![(0, 4), (4, 16)]);

        // A gap of five bytes still joins (`pos + 5 >= nextpos`).
        let mut data = vec![0; 32];
        data[0] = 1;
        data[5] = 1;
        assert_eq!(changed_runs(&data, &synced), vec![(0, 6)]);
    }

    // -- against the fake firmware -----------------------------------------

    /// The AVR dictionary the corpus's UC1701 boards use (`atmega2560.dict`),
    /// when this build produced it.
    fn fake_dictionary() -> Option<std::path::PathBuf> {
        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        dict.is_file().then_some(dict)
    }

    /// The panel really talks to the firmware: `config_spi`/`spi_set_bus` at
    /// build time, then the power-up list as `spi_send` with the data/controls
    /// line low, then the first flush as data with it high.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_the_panel_is_written_to_against_the_fake_firmware() {
        use crate::core::klippy::api::StartArgs;
        use crate::core::klippy::printer::PrinterState;
        use crate::core::klippy::reactor::TokioReactor;

        let Some(dict) = fake_dictionary() else {
            return;
        };
        let text = format!(
            "[mcu]\ntest: dict={}\n[display]\nlcd_type: uc1701\ncs_pin: PA3\na0_pin: PA5\n",
            dict.display()
        );
        let config = crate::core::klippy::config::Config::from_text(&text)
            .expect("the config parses")
            .0;
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let printer = Arc::new(Printer::new(reactor));
        let mut start_args = StartArgs::collect("display.cfg", None);
        start_args.debug_output = Some("_test_output".to_string());
        printer.set_start_args(Arc::new(start_args));

        let outcome = async {
            printer
                .load_config(&config)
                .map_err(|err| err.to_string())?;
            if tokio::time::timeout(std::time::Duration::from_secs(10), printer.bring_up())
                .await
                .is_err()
            {
                return Err("bring_up timed out".to_string());
            }
            if printer.get_state_message().category != PrinterState::Ready {
                return Err(format!(
                    "not ready: {}",
                    printer.get_state_message().message
                ));
            }
            let display = printer
                .lookup_object_as::<super::super::display::PrinterLCD>("display")
                .expect("the display sits under its section id");
            Ok::<_, String>(display.sent_messages())
        }
        .await;

        printer.teardown();
        let messages = outcome.expect("the panel comes up");

        assert_eq!(
            messages[0],
            SentMessage {
                is_data: false,
                bytes: uc1701_init_commands(40),
            }
        );
        assert_eq!(
            messages[1],
            SentMessage {
                is_data: false,
                bytes: vec![0xA5],
            }
        );
        assert_eq!(
            messages[2],
            SentMessage {
                is_data: false,
                bytes: vec![0xA4],
            }
        );
        // The first flush writes the blank screen as data.
        assert!(
            messages.iter().any(|message| message.is_data),
            "the flush sends its framebuffer contents: {messages:?}"
        );
        // Page 0's registers come before its bytes.
        let first_data = messages
            .iter()
            .position(|message| message.is_data)
            .expect("a data message");
        assert_eq!(
            messages[first_data - 1],
            SentMessage {
                is_data: false,
                bytes: vec![0xb0, 0x10, 0x00],
            }
        );
    }
}
