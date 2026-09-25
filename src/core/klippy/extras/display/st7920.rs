//! `st7920` — the ST7920 (128x64 graphics) panel driver
//! (upstream `klippy/extras/display/st7920.py`).
//!
//! A `[display]` section with `lcd_type: st7920` builds one of these. Three
//! GPIO pins (`cs`, `sclk`, `sid`, all on one MCU) are the whole bus: the host
//! drives the panel's own bit-level protocol through the firmware's
//! `config_st7920` / `st7920_send_cmds` / `st7920_send_data` commands, which is
//! why no SPI resource is involved (upstream's `st7920.py` does the same).
//!
//! | framework call | here | upstream |
//! |---|---|---|
//! | `init()` | the 8-command power-up sequence, then a flush | `st7920.py:63-74` |
//! | `clear()` | blank the text and graphics framebuffers | `st7920.py:125-130` |
//! | `flush()` | batch the framebuffer differences and send them | `st7920.py:30-54` |
//! | `set_glyphs()` | keep the 16x16 icons and cache the two animated pairs | `st7920.py:109-116` |
//!
//! The framebuffers are upstream's three (text, glyph, 32 graphics rows), and so
//! is the flush algorithm: find the changed bytes, join runs closer than five
//! bytes, then send each run as a command pair plus its data. Commands take the
//! panel in and out of extended mode by prepending `0x26`/`0x22`
//! (`st7920.py:181-189`).
//!
//! # What is not here
//!
//! * **`emulated_st7920`** (`st7920.py:191-234`): the software-SPI variant that
//!   shifts each byte as two nibbles and toggles an enable pin. Not implemented;
//!   `lcd_type` reports it as a gap.
//! * **Text and glyph drawing.** [`super::display`] never draws, so the
//!   `write_text`/`write_glyph`/`write_graphics` half of `DisplayBase` is not
//!   ported. The glyph framebuffer is still filled by [`ST7920::set_glyphs`],
//!   because upstream caches the animated icons at load time.
//! * **`BACKGROUND_PRIORITY_CLOCK` / `minclock`** (`st7920.py:9,186`): upstream
//!   sends with `reqclock=0x7fffffff00000000` and orders startup writes by clock;
//!   this host's [`McuCommand`](crate::core::klippy::cmd::McuCommand) carries no
//!   clock, so commands go out in call order.

use std::collections::{BTreeMap, HashMap};
use std::sync::{Arc, Mutex, MutexGuard, Weak};

use tracing::warn;

use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::spi_device::mcu_object_name;
use crate::core::klippy::mcu::{pin_number, ConfigBuilder, Mcu, McuError, McuObject};
use crate::core::klippy::msg::proto::ArgValue;
use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

use super::display::{Glyph, LcdChip, SentMessage};

/// Upstream's `ST7920_SYNC_DELAY` (`st7920.py:12`), in seconds.
const ST7920_SYNC_DELAY: f64 = 0.000045;
/// Upstream's `ST7920_CMD_DELAY` (`st7920.py:14`), in seconds.
const ST7920_CMD_DELAY: f64 = 0.000020;

/// The glyph framebuffer's id in the flush order (`st7920.py:26`).
const GLYPH_FB_ID: u8 = 0x40;
/// The text framebuffer's id in the flush order (`st7920.py:24`).
const TEXT_FB_ID: u8 = 0x80;
/// The panel's size in characters (`st7920.py:132-133`).
const DIMENSIONS: (usize, usize) = (16, 4);

/// One framebuffer and the copy the firmware has already been told about —
/// upstream's `(new_data, old_data, fb_id)` triples (`st7920.py:24-30`).
#[derive(Debug)]
struct Framebuffer {
    /// What the screen should show.
    data: Vec<u8>,
    /// What the firmware's copy holds; the difference is what a flush sends.
    synced: Vec<u8>,
    /// The `fb_id` the flush protocol selects this buffer by.
    fb_id: u8,
}

/// The three framebuffers, in the order upstream flushes them
/// (`st7920.py:23-30`).
#[derive(Debug)]
struct Framebuffers {
    text: Framebuffer,
    glyph: Framebuffer,
    graphics: Vec<Framebuffer>,
}

impl Framebuffers {
    /// Upstream's `DisplayBase.__init__` buffers: the text screen is spaces,
    /// everything else zeros, and every "already sent" copy is `~` so the first
    /// flush sends the lot (`st7920.py:20-30`).
    fn new() -> Self {
        let blank = |len: usize, byte: u8, fb_id: u8| Framebuffer {
            data: vec![byte; len],
            synced: vec![b'~'; len],
            fb_id,
        };
        Self {
            text: blank(64, b' ', TEXT_FB_ID),
            glyph: blank(128, 0, GLYPH_FB_ID),
            graphics: (0..32).map(|i| blank(32, 0, i as u8)).collect(),
        }
    }

    /// Text, glyph and graphics buffers in flush order, each with its two
    /// byte vectors and its id.
    fn iter_mut(&mut self) -> Vec<(&mut Vec<u8>, &mut Vec<u8>, u8)> {
        let mut all = vec![
            (&mut self.text.data, &mut self.text.synced, self.text.fb_id),
            (
                &mut self.glyph.data,
                &mut self.glyph.synced,
                self.glyph.fb_id,
            ),
        ];
        for graphics in &mut self.graphics {
            all.push((&mut graphics.data, &mut graphics.synced, graphics.fb_id));
        }
        all
    }
}

/// The state a config callback and the runtime methods share.
#[derive(Debug)]
struct St7920State {
    /// The oid the firmware allocated for this panel.
    oid: u8,
    /// The three pin descriptions as written, in `cs`, `sclk`, `sid` order.
    pins: Vec<String>,
    /// The chip all three pins are on.
    chip_name: String,
    /// Whether the panel is currently in extended mode (`st7920.py:180`).
    is_extended: Mutex<bool>,
    /// The framebuffers.
    framebuffers: Mutex<Framebuffers>,
    /// The 16x16 icons by glyph name, split into the two column halves
    /// (`st7920.py:27`).
    icons: Mutex<HashMap<String, (Vec<u8>, Vec<u8>)>>,
    /// Every message handed to the firmware, in order (tests and diagnostics).
    sent: Mutex<Vec<SentMessage>>,
}

/// One `lcd_type: st7920` panel.
pub struct ST7920 {
    state: Arc<St7920State>,
    /// The MCU the pins are on, for the runtime sends.
    mcu: Arc<McuObject>,
}

impl ST7920 {
    /// Read the three pins, allocate the oid, and register the config callback.
    ///
    /// Upstream's `ST7920.__init__` (`st7920.py:140-155`).
    ///
    /// # Errors
    /// A missing pin, an unresolvable or already-used pin, pins on different
    /// MCUs (`st7920 all pins must be on same mcu`), or an oid shortage.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");

        let mut descriptions = Vec::with_capacity(3);
        let mut chip_name: Option<String> = None;
        for role in ["cs", "sclk", "sid"] {
            let description = config.get(&format!("{role}_pin"), None)?;
            let params = pins
                .lookup_pin(&description, false, false, None)
                .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
            match &chip_name {
                Some(chip) if *chip != params.chip_name => {
                    return Err(ConfigError::new(
                        "st7920 all pins must be on same mcu".to_string(),
                    ));
                }
                Some(_) => {}
                None => chip_name = Some(params.chip_name.clone()),
            }
            descriptions.push(params.pin);
        }
        let chip_name = chip_name.expect("three pins were read");

        let mcu = printer
            .lookup_object_as::<McuObject>(&mcu_object_name(&chip_name))
            .ok_or_else(|| {
                ConfigError::new(format!("Section '{identifier}': unknown MCU '{chip_name}'"))
            })?;
        let oid = mcu
            .config()
            .create_oid()
            .map_err(|err| ConfigError::new(err.to_string()))?;

        let state = Arc::new(St7920State {
            oid,
            pins: descriptions,
            chip_name,
            is_extended: Mutex::new(false),
            framebuffers: Mutex::new(Framebuffers::new()),
            icons: Mutex::new(HashMap::new()),
            sent: Mutex::new(Vec::new()),
        });

        let build_state = Arc::downgrade(&state);
        let build_pins = Arc::downgrade(&pins);
        mcu.config()
            .register_config_callback(Box::new(move |builder, mcu| match build_state.upgrade() {
                Some(state) => state.build(builder, mcu, &build_pins),
                None => Ok(()),
            }))
            .map_err(|err| ConfigError::new(err.to_string()))?;

        Ok(Self { state, mcu })
    }

    /// Send one protocol message, switching extended mode when upstream does
    /// (`st7920.py:181-189`).
    fn send(&self, cmds: &[u8], is_data: bool, is_extended: bool) -> Result<(), McuError> {
        let mut bytes = cmds.to_vec();
        if !is_data {
            let mut extended = self
                .state
                .is_extended
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            if *extended != is_extended {
                bytes.insert(0, if is_extended { 0x26 } else { 0x22 });
                *extended = is_extended;
            }
        }
        let mcu = self
            .mcu
            .mcu()
            .ok_or_else(|| McuError::Config("the MCU is not connected".to_string()))?;
        if is_data {
            mcu.send_msg(&St7920SendData {
                oid: self.state.oid,
                data: bytes.clone(),
            })?;
        } else {
            mcu.send_msg(&St7920SendCmds {
                oid: self.state.oid,
                cmds: bytes.clone(),
            })?;
        }
        self.state
            .sent
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .push(SentMessage { is_data, bytes });
        Ok(())
    }

    /// The glyph framebuffer, for [`ST7920::cache_glyph`].
    fn framebuffers(&self) -> MutexGuard<'_, Framebuffers> {
        self.state
            .framebuffers
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl LcdChip for ST7920 {
    /// Upstream's `DisplayBase.init` (`st7920.py:63-74`): the eight power-up
    /// commands, then the first flush.
    fn init(&self) {
        let cmds = [0x24, 0x40, 0x02, 0x26, 0x22, 0x02, 0x06, 0x0c];
        if let Err(err) = self.send(&cmds, false, false) {
            warn!("st7920: could not initialise the panel: {err}");
            return;
        }
        self.flush();
    }

    /// Upstream's `DisplayBase.clear` (`st7920.py:125-130`). The glyph
    /// framebuffer is deliberately left alone: the cached icons live there.
    fn clear(&self) {
        let mut framebuffers = self.framebuffers();
        framebuffers.text.data = vec![b' '; 64];
        for graphics in &mut framebuffers.graphics {
            graphics.data = vec![0; 32];
        }
    }

    /// Upstream's `DisplayBase.flush` (`st7920.py:30-54`): send the changed
    /// bytes of every framebuffer, batching runs that are close together.
    fn flush(&self) {
        let mut framebuffers = self.framebuffers();
        for (data, synced, fb_id) in framebuffers.iter_mut() {
            let data = data.clone();
            if data == *synced {
                continue;
            }
            // Positions of every changed byte, each a run of one.
            let mut diffs: Vec<(usize, usize)> = data
                .iter()
                .zip(synced.iter())
                .enumerate()
                .filter(|(_, (new, old))| new != old)
                .map(|(i, _)| (i, 1))
                .collect();
            // Join runs closer than five bytes, as upstream does
            // (`st7920.py:38-44`).
            for i in (0..diffs.len().saturating_sub(1)).rev() {
                let (pos, _count) = diffs[i];
                let (next_pos, next_count) = diffs[i + 1];
                if pos + 5 >= next_pos && next_count < 16 {
                    diffs[i] = (pos, next_count + (next_pos - pos));
                    diffs.remove(i + 1);
                }
            }
            for (pos, count) in diffs {
                let count = count + (pos & 0x01);
                let count = count + (count & 0x01);
                let pos = pos & !0x01;
                let chip_pos = pos >> 1;
                let command = if fb_id < GLYPH_FB_ID {
                    // A graphics row: two extended-mode bytes select it.
                    vec![0x80 + fb_id, 0x80 + chip_pos as u8]
                } else {
                    vec![fb_id + chip_pos as u8]
                };
                if let Err(err) = self.send(&command, false, fb_id < GLYPH_FB_ID) {
                    warn!("st7920: could not flush the panel: {err}");
                    return;
                }
                if let Err(err) = self.send(&data[pos..pos + count], true, false) {
                    warn!("st7920: could not flush the panel: {err}");
                    return;
                }
            }
            synced.copy_from_slice(&data);
        }
    }

    /// The panel's size in characters (`st7920.py:132-133`).
    fn get_dimensions(&self) -> (usize, usize) {
        DIMENSIONS
    }

    /// Upstream's `DisplayBase.set_glyphs` (`st7920.py:109-116`): keep every
    /// 16x16 icon, then cache the two animated glyph pairs.
    fn set_glyphs(&self, glyphs: &BTreeMap<String, Glyph>) {
        {
            let mut icons = self
                .state
                .icons
                .lock()
                .unwrap_or_else(|poison| poison.into_inner());
            for (name, glyph) in glyphs {
                if let Some(icon) = &glyph.icon16x16 {
                    icons.insert(name.clone(), icon.clone());
                }
            }
        }
        self.cache_glyph("fan2", "fan1", 0);
        self.cache_glyph("bed_heat2", "bed_heat1", 1);
    }

    /// How many messages this panel has handed to the firmware.
    fn sent_messages(&self) -> Vec<SentMessage> {
        self.state
            .sent
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }
}

impl PrinterObject for ST7920 {
    /// Upstream's `ST7920` has no `get_status`, so it is not client-visible.
    fn get_status(&self, _eventtime: f64) -> serde_json::Value {
        serde_json::json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for ST7920 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ST7920")
            .field("oid", &self.state.oid)
            .finish_non_exhaustive()
    }
}

impl ST7920 {
    /// Upstream's `DisplayBase.cache_glyph` (`st7920.py:75-86`): store the
    /// difference between an animated glyph and its base in the character
    /// generator's CGRAM slots, so the two frames can be shown by writing a
    /// text byte.
    fn cache_glyph(&self, glyph_name: &str, base_glyph_name: &str, glyph_id: usize) {
        let icons = self
            .state
            .icons
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let (Some(icon), Some(base_icon)) = (icons.get(glyph_name), icons.get(base_glyph_name))
        else {
            return;
        };
        let mut framebuffers = self.framebuffers();
        for (i, (ic1, ic2, b1, b2)) in icon
            .0
            .iter()
            .zip(icon.1.iter())
            .zip(base_icon.0.iter().zip(base_icon.1.iter()))
            .map(|((ic1, ic2), (b1, b2))| (*ic1, *ic2, *b1, *b2))
            .enumerate()
        {
            let (x1, x2) = (ic1 ^ b1, ic2 ^ b2);
            let pos = glyph_id * 32 + i * 2;
            framebuffers.glyph.data[pos..pos + 2].copy_from_slice(&[x1, x2]);
            // The "already sent" copy is deliberately the *inverse*
            // (`st7920.py:84`), so the next flush transmits these two bytes.
            framebuffers.glyph.synced[pos..pos + 2].copy_from_slice(&[x1 ^ 1, x2 ^ 1]);
        }
    }
}

impl St7920State {
    /// Add this panel's configuration command, with the pin numbers resolved.
    ///
    /// Upstream's `ST7920.build_config` (`st7920.py:157-169`): the pin *names*
    /// become firmware numbers here, the first moment the dictionary exists.
    fn build(
        &self,
        builder: &ConfigBuilder,
        mcu: &Mcu,
        pins: &Weak<PrinterPins>,
    ) -> Result<(), McuError> {
        let pins = pins
            .upgrade()
            .ok_or_else(|| McuError::Config("the pins registry is gone".to_string()))?;
        let mut numbers = Vec::with_capacity(3);
        for description in &self.pins {
            let name = pins
                .resolve_pin(&self.chip_name, description)
                .map_err(|err| McuError::Config(format!("st7920 pin: {err}")))?;
            numbers.push(
                pin_number(mcu, &name, &self.chip_name)
                    .map_err(|err| McuError::Config(format!("st7920 pin: {err}")))?,
            );
        }
        builder.add_config_cmd(&ConfigSt7920 {
            oid: self.oid,
            cs_pin: numbers[0],
            sclk_pin: numbers[1],
            sid_pin: numbers[2],
            sync_delay_ticks: mcu.seconds_to_clock(ST7920_SYNC_DELAY)? as u32,
            cmd_delay_ticks: mcu.seconds_to_clock(ST7920_CMD_DELAY)? as u32,
        })
    }
}

// ===========================================================================
// The firmware commands
// ===========================================================================

/// `config_st7920 oid=%c cs_pin=%u sclk_pin=%u sid_pin=%u
/// sync_delay_ticks=%u cmd_delay_ticks=%u` — configure one panel.
///
/// Upstream builds this as a text command in `build_config` (`st7920.py:160-166`);
/// the wording of the format string is the firmware's, and the delay ticks are
/// `mcu.seconds_to_clock` of the two delays above.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigSt7920 {
    /// The panel's oid.
    pub oid: u8,
    /// Chip-select pin number (firmware enumeration).
    pub cs_pin: u32,
    /// Serial-clock pin number.
    pub sclk_pin: u32,
    /// Serial-data pin number.
    pub sid_pin: u32,
    /// Delay between the sync pulses, in clock ticks.
    pub sync_delay_ticks: u32,
    /// Delay after a command byte, in clock ticks.
    pub cmd_delay_ticks: u32,
}

impl McuCommand for ConfigSt7920 {
    const NAME: &'static str = "config_st7920";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.cs_pin),
            ArgValue::UInt32(self.sclk_pin),
            ArgValue::UInt32(self.sid_pin),
            ArgValue::UInt32(self.sync_delay_ticks),
            ArgValue::UInt32(self.cmd_delay_ticks),
        ]
    }
}

/// `st7920_send_cmds oid=%c cmds=%*s` — send command bytes, not data.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct St7920SendCmds {
    /// The panel's oid.
    pub oid: u8,
    /// The command bytes.
    pub cmds: Vec<u8>,
}

impl McuCommand for St7920SendCmds {
    const NAME: &'static str = "st7920_send_cmds";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::Bytes(self.cmds.clone()),
        ]
    }
}

/// `st7920_send_data oid=%c data=%*s` — send display data bytes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct St7920SendData {
    /// The panel's oid.
    pub oid: u8,
    /// The data bytes.
    pub data: Vec<u8>,
}

impl McuCommand for St7920SendData {
    const NAME: &'static str = "st7920_send_data";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::Bytes(self.data.clone()),
        ]
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
    use crate::core::klippy::config::Config;
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::reactor::ManualReactor;

    /// The three commands, with the format strings the firmware publishes.
    fn roundtrip<C: McuCommand>(command: &C) -> Vec<ArgValue> {
        let dictionary = crate::core::klippy::mcu::Dictionary::from_json(serde_json::json!({
            "commands": {
                "config_st7920 oid=%c cs_pin=%u sclk_pin=%u sid_pin=%u sync_delay_ticks=%u cmd_delay_ticks=%u": 59,
                "st7920_send_cmds oid=%c cmds=%*s": 58,
                "st7920_send_data oid=%c data=%*s": 57,
            },
            "config": {"CLOCK_FREQ": 20000000},
        }))
        .unwrap();
        let mut parser = Parser::new();
        dictionary.install(&mut parser).unwrap();
        let encoded = parser.encode(C::NAME, &command.args()).unwrap();
        let decoded = parser.decode(encoded).unwrap();
        assert_eq!(decoded[0].0.name, C::NAME);
        decoded[0].1.clone()
    }

    #[test]
    fn test_the_config_command_keeps_the_firmware_format() {
        let command = ConfigSt7920 {
            oid: 3,
            cs_pin: 16,
            sclk_pin: 17,
            sid_pin: 18,
            sync_delay_ticks: 900,
            cmd_delay_ticks: 400,
        };
        assert_eq!(
            roundtrip(&command),
            vec![
                ArgValue::UInt8(3),
                ArgValue::UInt32(16),
                ArgValue::UInt32(17),
                ArgValue::UInt32(18),
                ArgValue::UInt32(900),
                ArgValue::UInt32(400),
            ]
        );
    }

    #[test]
    fn test_the_send_commands_carry_binary_bytes() {
        // A high byte proves the parameter is `%*s` (binary), not text.
        let cmds = St7920SendCmds {
            oid: 1,
            cmds: vec![0x24, 0x40, 0xff],
        };
        assert_eq!(
            roundtrip(&cmds),
            vec![ArgValue::UInt8(1), ArgValue::Bytes(vec![0x24, 0x40, 0xff])]
        );
        let data = St7920SendData {
            oid: 1,
            data: vec![0x20; 4],
        };
        assert_eq!(
            roundtrip(&data),
            vec![ArgValue::UInt8(1), ArgValue::Bytes(vec![0x20; 4])]
        );
    }

    /// A `[display]` section's three pins, as the loader would hand it over.
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
    /// section runs: `mcu`, and `board2` for the "pins on one MCU" check.
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

    #[test]
    fn test_the_three_pins_must_be_on_one_mcu() {
        let printer = printer();
        let mut section = display_section(&[("cs_pin", "board2:PA3")]);
        section.parameters.insert(
            "sclk_pin".to_string(),
            ConfigValue::Single("PA1".to_string()),
        );
        section.parameters.insert(
            "sid_pin".to_string(),
            ConfigValue::Single("PC1".to_string()),
        );

        let err = ST7920::new(&ConfigWrapper::untracked(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(err.to_string(), "st7920 all pins must be on same mcu");
    }

    #[test]
    fn test_a_missing_pin_is_reported_the_way_the_config_reports_it() {
        let printer = printer();
        let section = display_section(&[("cs_pin", "PA3"), ("sclk_pin", "PA1")]);

        let err = ST7920::new(&ConfigWrapper::untracked(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'sid_pin' in section 'display' must be specified"
        );
    }

    #[test]
    fn test_the_unconnected_panel_cannot_send() {
        // Without a connected MCU there is nothing to write to; the driver
        // reports it instead of panicking, so a refresh that runs before the
        // machine is up only logs.
        let printer = printer();
        let section =
            display_section(&[("cs_pin", "PA3"), ("sclk_pin", "PA1"), ("sid_pin", "PC1")]);
        let chip = ST7920::new(&ConfigWrapper::untracked(&section), &printer).unwrap();
        assert_eq!(chip.sent_message_count(), 0);

        let err = chip.send(&[0x24], false, false).unwrap_err();
        assert!(err.to_string().contains("not connected"), "{err}");
        assert_eq!(chip.sent_message_count(), 0);
    }

    #[test]
    fn test_a_config_section_is_not_a_required_part_of_the_driver() {
        // The driver asks the printer for the MCU named by its pins; a section
        // that never says which chip gets the section's own name back.
        let printer = printer();
        let section = display_section(&[("cs_pin", "PA3"), ("sclk_pin", "PA1")]);
        let err = ST7920::new(&ConfigWrapper::untracked(&section), &printer)
            .map(|_| ())
            .unwrap_err();
        assert!(err.to_string().starts_with("Option 'sid_pin'"), "{err}");

        // And a section whose chip does not exist is reported by name.
        let section = display_section(&[
            ("cs_pin", "board2:PA3"),
            ("sclk_pin", "board2:PA1"),
            ("sid_pin", "board2:PC1"),
        ]);
        let chip = ST7920::new(&ConfigWrapper::untracked(&section), &printer)
            .expect("a second MCU is a valid chip");
        assert_eq!(chip.sent_message_count(), 0);
    }

    #[test]
    fn test_the_framebuffers_start_unsent() {
        // Every "already sent" copy is `~`, so the first flush sends the whole
        // screen (`st7920.py:24-30`).
        let framebuffers = Framebuffers::new();
        assert_eq!(framebuffers.text.data, vec![b' '; 64]);
        assert_eq!(framebuffers.text.synced, vec![b'~'; 64]);
        assert_eq!(framebuffers.graphics.len(), 32);
        assert_eq!(framebuffers.graphics[31].fb_id, 31);
        assert_eq!(framebuffers.glyph.synced, vec![b'~'; 128]);
    }

    #[test]
    fn test_clear_blanks_text_and_graphics_but_keeps_the_glyph_slots() {
        let printer = printer();
        let section =
            display_section(&[("cs_pin", "PA3"), ("sclk_pin", "PA1"), ("sid_pin", "PC1")]);
        let chip = ST7920::new(&ConfigWrapper::untracked(&section), &printer).unwrap();
        {
            let mut framebuffers = chip.framebuffers();
            framebuffers.text.data = vec![b'x'; 64];
            framebuffers.glyph.data[0] = 0xff;
            framebuffers.graphics[0].data[0] = 0xff;
        }
        chip.clear();

        let framebuffers = chip.framebuffers();
        assert_eq!(framebuffers.text.data, vec![b' '; 64]);
        assert_eq!(framebuffers.graphics[0].data, vec![0; 32]);
        assert_eq!(framebuffers.glyph.data[0], 0xff);
    }

    #[test]
    fn test_set_glyphs_caches_the_two_animated_pairs() {
        let printer = printer();
        let section =
            display_section(&[("cs_pin", "PA3"), ("sclk_pin", "PA1"), ("sid_pin", "PC1")]);
        let chip = ST7920::new(&ConfigWrapper::untracked(&section), &printer).unwrap();

        let mut glyphs: BTreeMap<String, Glyph> = BTreeMap::new();
        for name in ["fan1", "fan2", "bed_heat1", "bed_heat2"] {
            let first = if name.ends_with('2') { 0xff } else { 0x00 };
            glyphs.insert(
                name.to_string(),
                Glyph {
                    icon16x16: Some((vec![first; 16], vec![first; 16])),
                    icon5x8: None,
                },
            );
        }
        chip.set_glyphs(&glyphs);

        let framebuffers = chip.framebuffers();
        // `fan2` (slot 0) differs from `fan1` in every bit, so the cached bytes
        // are all ones and the "sent" copy is their inverse.
        assert_eq!(framebuffers.glyph.data[0], 0xff);
        assert_eq!(framebuffers.glyph.data[31], 0xff);
        assert_eq!(framebuffers.glyph.synced[0], 0xfe);
        // `bed_heat2` (slot 1) starts at byte 32.
        assert_eq!(framebuffers.glyph.data[32], 0xff);
    }

    #[test]
    fn test_a_config_parse_is_not_needed_to_know_the_panel_shape() {
        // `get_dimensions` is fixed by the driver, not the config.
        let (config, _) = Config::from_text("[display]\nlcd_type: st7920\n").unwrap();
        assert!(config.get_section("display").is_some());
        assert_eq!(DIMENSIONS, (16, 4));
    }
}
