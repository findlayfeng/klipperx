//! `hd44780` — the HD44780 (20x4 text) panel driver
//! (upstream `klippy/extras/display/hd44780.py`).
//!
//! A `[display]` section with `lcd_type: hd44780` builds one of these. Six GPIO
//! pins (`rs`, `e`, `d4`…`d7`, all on one MCU) are a 4-bit parallel bus: the
//! host shifts each byte out through the firmware's `hd44780_send_cmds` /
//! `hd44780_send_data` commands, after `config_hd44780` has wired the pins and
//! been told how long one nibble takes (`HD44780_DELAY`, `hd44780.py:15`).
//!
//! | framework call | here | upstream |
//! |---|---|---|
//! | `init()` | the power-up command groups, then a flush | `hd44780.py:90-103` |
//! | `clear()` | blank both text framebuffers | `hd44780.py:129-132` |
//! | `flush()` | batch the framebuffer differences and send them | `hd44780.py:69-89` |
//! | `set_glyphs()` | keep the 5x8 icons for the glyphs the layout names | `hd44780.py:109-113` |
//!
//! The framebuffers are upstream's three: two text buffers of `2*line_length`
//! bytes (`hd44780.py:40-50`), because each of the panel's four rows is driven
//! as one half of one buffer, and the 64-byte character generator buffer. As
//! upstream, every "already sent" copy starts as `~`, so the first flush writes
//! the whole screen. `line_length` is 16 or 20 (`hd44780.py:10-11,26-27`) and
//! decides the panel's width, and with it the default `display_group`.
//!
//! # What is not here
//!
//! * **Text and glyph drawing.** [`super::display`] never draws, so
//!   `write_text`/`write_glyph`/`write_graphics` (`hd44780.py:104-128`) are not
//!   ported; [`Hd44780::set_glyphs`] keeps the 5x8 icons because upstream
//!   collects them there (`hd44780.py:109-113`).
//! * **`hd44780_spi`**: the SPI backpack variant is a different `lcd_type`, and
//!   `lcd_type` reports it as a gap.
//! * **`BACKGROUND_PRIORITY_CLOCK` / `minclock`** (`hd44780.py:9,92-102`):
//!   upstream sends each init group with a clock deadline 100ms apart and
//!   stamps its writes with `reqclock=0x7fffffff00000000`; this host's
//!   [`McuCommand`](crate::core::klippy::cmd::McuCommand) carries neither, so
//!   commands go out in call order.

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

/// The time one nibble needs, in seconds (`hd44780.py:15`).
const HD44780_DELAY: f64 = 0.000040;
/// The panel's width when `line_length` is not written (`hd44780.py:10`).
const LINE_LENGTH_DEFAULT: &str = "20";
/// The widths `line_length` may choose between (`hd44780.py:11`).
const LINE_LENGTH_OPTIONS: &[&str] = &["16", "20"];

/// One framebuffer and the copy the firmware has already been told about —
/// upstream's `(new_data, old_data, fb_id)` triples (`hd44780.py:43-50`).
#[derive(Debug)]
struct Framebuffer {
    /// What the screen should show.
    data: Vec<u8>,
    /// What the firmware's copy holds; the difference is what a flush sends.
    synced: Vec<u8>,
    /// The `fb_id` the flush protocol selects this buffer by.
    fb_id: u8,
}

/// The two text framebuffers and the character generator's buffer, in the order
/// upstream flushes them (`hd44780.py:43-50`).
#[derive(Debug)]
struct Framebuffers {
    /// One buffer per pair of panel rows (`hd44780.py:40-41`).
    text: [Framebuffer; 2],
    /// The 64-byte CGRAM buffer (`hd44780.py:42`).
    glyph: Framebuffer,
}

impl Framebuffers {
    /// Upstream's `__init__` buffers: the text screens are spaces, the glyph
    /// buffer is zeros, and every "already sent" copy is `~` so the first flush
    /// sends the lot (`hd44780.py:40-50`).
    fn new(line_length: usize) -> Self {
        let half = 2 * line_length;
        let blank = |len: usize, byte: u8, fb_id: u8| Framebuffer {
            data: vec![byte; len],
            synced: vec![b'~'; len],
            fb_id,
        };
        Self {
            text: [blank(half, b' ', 0x80), blank(half, b' ', 0xc0)],
            glyph: blank(64, 0, 0x40),
        }
    }

    /// The three buffers in flush order, each with its two byte vectors and its
    /// id.
    fn iter_mut(&mut self) -> [(&mut Vec<u8>, &mut Vec<u8>, u8); 3] {
        let [first, second] = &mut self.text;
        [
            (&mut first.data, &mut first.synced, first.fb_id),
            (&mut second.data, &mut second.synced, second.fb_id),
            (
                &mut self.glyph.data,
                &mut self.glyph.synced,
                self.glyph.fb_id,
            ),
        ]
    }
}

/// The state a config callback and the runtime methods share.
#[derive(Debug)]
struct Hd44780State {
    /// The oid the firmware allocated for this panel.
    oid: u8,
    /// The six pin descriptions as written, in `rs`, `e`, `d4`…`d7` order
    /// (`hd44780.py:20-23`).
    pins: Vec<String>,
    /// The chip all six pins are on.
    chip_name: String,
    /// The panel's width in characters (`hd44780.py:26-27`).
    line_length: usize,
    /// Whether the power-up sequence programs the 4-bit protocol
    /// (`hd44780.py:24-25`).
    hd44780_protocol_init: bool,
    /// The framebuffers.
    framebuffers: Mutex<Framebuffers>,
    /// The 5x8 icons by glyph name (`hd44780.py:109-113`).
    icons: Mutex<HashMap<String, (u8, Vec<u8>)>>,
    /// Every message handed to the firmware, in order (tests and diagnostics).
    sent: Mutex<Vec<SentMessage>>,
}

/// One `lcd_type: hd44780` panel.
pub struct Hd44780 {
    state: Arc<Hd44780State>,
    /// The MCU the pins are on, for the runtime sends.
    mcu: Arc<McuObject>,
}

impl Hd44780 {
    /// Read the six pins and the two options, allocate the oid, and register
    /// the config callback.
    ///
    /// Upstream's `HD44780.__init__` (`hd44780.py:18-50`): all six pins are
    /// read first, then `hd44780_protocol_init` and `line_length`, and only
    /// then is the "all pins on one MCU" rule applied — so a missing or
    /// invalid option is reported before a pin conflict.
    ///
    /// # Errors
    /// A missing pin, an unresolvable or already-used pin, pins on different
    /// MCUs (`hd44780 all pins must be on same mcu`), a `line_length` outside
    /// [`LINE_LENGTH_OPTIONS`], or an oid shortage.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");

        let mut descriptions = Vec::with_capacity(6);
        let mut chip_names = Vec::with_capacity(6);
        for role in ["rs", "e", "d4", "d5", "d6", "d7"] {
            let description = config.get(&format!("{role}_pin"), None)?;
            let params = pins
                .lookup_pin(&description, false, false, None)
                .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
            chip_names.push(params.chip_name);
            descriptions.push(params.pin);
        }

        let hd44780_protocol_init = config.get_bool("hd44780_protocol_init", Some(true))?;
        let line_length = config
            .get_choice(
                "line_length",
                LINE_LENGTH_OPTIONS,
                Some(LINE_LENGTH_DEFAULT),
            )?
            .parse::<usize>()
            .expect("the choice is one of LINE_LENGTH_OPTIONS");

        if chip_names.iter().any(|chip| *chip != chip_names[0]) {
            return Err(ConfigError::new(
                "hd44780 all pins must be on same mcu".to_string(),
            ));
        }
        let chip_name = chip_names[0].clone();

        let mcu = printer
            .lookup_object_as::<McuObject>(&mcu_object_name(&chip_name))
            .ok_or_else(|| {
                ConfigError::new(format!("Section '{identifier}': unknown MCU '{chip_name}'"))
            })?;
        let oid = mcu
            .config()
            .create_oid()
            .map_err(|err| ConfigError::new(err.to_string()))?;

        let state = Arc::new(Hd44780State {
            oid,
            pins: descriptions,
            chip_name,
            line_length,
            hd44780_protocol_init,
            framebuffers: Mutex::new(Framebuffers::new(line_length)),
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

    /// Send one protocol message on the command or the data channel.
    fn send(&self, cmds: &[u8], is_data: bool) -> Result<(), McuError> {
        let mcu = self
            .mcu
            .mcu()
            .ok_or_else(|| McuError::Config("the MCU is not connected".to_string()))?;
        let bytes = cmds.to_vec();
        if is_data {
            mcu.send_msg(&Hd44780SendData {
                oid: self.state.oid,
                data: bytes.clone(),
            })?;
        } else {
            mcu.send_msg(&Hd44780SendCmds {
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

    /// The framebuffers, for the trait methods.
    fn framebuffers(&self) -> MutexGuard<'_, Framebuffers> {
        self.state
            .framebuffers
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl LcdChip for Hd44780 {
    /// Upstream's `HD44780.init` (`hd44780.py:90-103`): the power-up command
    /// groups, then the first flush.
    fn init(&self) {
        for cmds in init_commands(self.state.hd44780_protocol_init) {
            if let Err(err) = self.send(&cmds, false) {
                warn!("hd44780: could not initialise the panel: {err}");
                return;
            }
        }
        self.flush();
    }

    /// Upstream's `HD44780.clear` (`hd44780.py:129-132`): only the text
    /// framebuffers are blanked; the glyph buffer keeps the icons.
    fn clear(&self) {
        let mut framebuffers = self.framebuffers();
        for text in &mut framebuffers.text {
            text.data.fill(b' ');
        }
    }

    /// Upstream's `HD44780.flush` (`hd44780.py:69-89`): send the changed bytes
    /// of every framebuffer, batching changes that are close together.
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
            // Join runs within four bytes of each other, as upstream does
            // (`hd44780.py:78-84`).
            for i in (0..diffs.len().saturating_sub(1)).rev() {
                let (pos, _count) = diffs[i];
                let (next_pos, next_count) = diffs[i + 1];
                if pos + 4 >= next_pos && next_count < 16 {
                    diffs[i] = (pos, next_count + (next_pos - pos));
                    diffs.remove(i + 1);
                }
            }
            for (pos, count) in diffs {
                if let Err(err) = self.send(&[fb_id + pos as u8], false) {
                    warn!("hd44780: could not flush the panel: {err}");
                    return;
                }
                if let Err(err) = self.send(&data[pos..pos + count], true) {
                    warn!("hd44780: could not flush the panel: {err}");
                    return;
                }
            }
            synced.copy_from_slice(&data);
        }
    }

    /// The panel's size in characters (`hd44780.py:133-134`).
    fn get_dimensions(&self) -> (usize, usize) {
        (self.state.line_length, 4)
    }

    /// Upstream's `HD44780.set_glyphs` (`hd44780.py:109-113`): keep every 5x8
    /// icon by glyph name.
    fn set_glyphs(&self, glyphs: &BTreeMap<String, Glyph>) {
        let mut icons = self
            .state
            .icons
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        for (name, glyph) in glyphs {
            if let Some((slot, bits)) = &glyph.icon5x8 {
                icons.insert(name.clone(), (*slot, bits.clone()));
            }
        }
    }

    /// Every message this panel has handed to the firmware, in order.
    fn sent_messages(&self) -> Vec<SentMessage> {
        self.state
            .sent
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }
}

impl PrinterObject for Hd44780 {
    /// Upstream's `HD44780` has no `get_status`, so it is not client-visible.
    fn get_status(&self, _eventtime: f64) -> serde_json::Value {
        serde_json::json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for Hd44780 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Hd44780")
            .field("oid", &self.state.oid)
            .finish_non_exhaustive()
    }
}

impl Hd44780State {
    /// Add this panel's configuration command, with the pin numbers resolved.
    ///
    /// Upstream's `HD44780.build_config` (`hd44780.py:51-62`): the pin
    /// *names* become firmware numbers here, and the delay is
    /// `mcu.seconds_to_clock(HD44780_DELAY)`.
    fn build(
        &self,
        builder: &ConfigBuilder,
        mcu: &Mcu,
        pins: &Weak<PrinterPins>,
    ) -> Result<(), McuError> {
        let pins = pins
            .upgrade()
            .ok_or_else(|| McuError::Config("the pins registry is gone".to_string()))?;
        let mut numbers = Vec::with_capacity(6);
        for description in &self.pins {
            let name = pins
                .resolve_pin(&self.chip_name, description)
                .map_err(|err| McuError::Config(format!("hd44780 pin: {err}")))?;
            numbers.push(
                pin_number(mcu, &name, &self.chip_name)
                    .map_err(|err| McuError::Config(format!("hd44780 pin: {err}")))?,
            );
        }
        builder.add_config_cmd(&ConfigHd44780 {
            oid: self.oid,
            rs_pin: numbers[0],
            e_pin: numbers[1],
            d4_pin: numbers[2],
            d5_pin: numbers[3],
            d6_pin: numbers[4],
            d7_pin: numbers[5],
            delay_ticks: mcu.seconds_to_clock(HD44780_DELAY)? as u32,
        })
    }
}

/// The power-up command groups, one message each (`hd44780.py:94-101`).
///
/// With `hd44780_protocol_init` (the default) the panel is put into 4-bit,
/// 2-line mode first and then homed; without it only the home command is sent.
/// Either way the last group sets the update direction and enables the display
/// with the cursor hidden.
fn init_commands(hd44780_protocol_init: bool) -> Vec<Vec<u8>> {
    let mut groups: Vec<Vec<u8>> = if hd44780_protocol_init {
        vec![vec![0x33], vec![0x33], vec![0x32], vec![0x28, 0x28, 0x02]]
    } else {
        vec![vec![0x02]]
    };
    groups.push(vec![0x06, 0x0c]);
    groups
}

// ===========================================================================
// The firmware commands
// ===========================================================================

/// `config_hd44780 oid=%c rs_pin=%u e_pin=%u d4_pin=%u d5_pin=%u d6_pin=%u
/// d7_pin=%u delay_ticks=%u` — configure one panel.
///
/// Upstream builds this as a text command in `build_config`
/// (`hd44780.py:52-57`); the wording of the format string is the firmware's,
/// and `delay_ticks` is `mcu.seconds_to_clock` of [`HD44780_DELAY`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigHd44780 {
    /// The panel's oid.
    pub oid: u8,
    /// Register-select pin number (firmware enumeration).
    pub rs_pin: u32,
    /// Enable pin number.
    pub e_pin: u32,
    /// Data pin 4.
    pub d4_pin: u32,
    /// Data pin 5.
    pub d5_pin: u32,
    /// Data pin 6.
    pub d6_pin: u32,
    /// Data pin 7.
    pub d7_pin: u32,
    /// Delay between nibbles, in clock ticks.
    pub delay_ticks: u32,
}

impl McuCommand for ConfigHd44780 {
    const NAME: &'static str = "config_hd44780";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt32(self.rs_pin),
            ArgValue::UInt32(self.e_pin),
            ArgValue::UInt32(self.d4_pin),
            ArgValue::UInt32(self.d5_pin),
            ArgValue::UInt32(self.d6_pin),
            ArgValue::UInt32(self.d7_pin),
            ArgValue::UInt32(self.delay_ticks),
        ]
    }
}

/// `hd44780_send_cmds oid=%c cmds=%*s` — send command bytes, not data.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Hd44780SendCmds {
    /// The panel's oid.
    pub oid: u8,
    /// The command bytes.
    pub cmds: Vec<u8>,
}

impl McuCommand for Hd44780SendCmds {
    const NAME: &'static str = "hd44780_send_cmds";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::Bytes(self.cmds.clone()),
        ]
    }
}

/// `hd44780_send_data oid=%c data=%*s` — send display data bytes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Hd44780SendData {
    /// The panel's oid.
    pub oid: u8,
    /// The data bytes.
    pub data: Vec<u8>,
}

impl McuCommand for Hd44780SendData {
    const NAME: &'static str = "hd44780_send_data";

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
    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::section::ConfigSection;
    use crate::core::klippy::config::value::ConfigValue;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::reactor::ManualReactor;

    /// The three commands, with the format strings the firmware publishes
    /// (`src/lcd_hd44780.c:118-140`).
    fn roundtrip<C: McuCommand>(command: &C) -> Vec<ArgValue> {
        let dictionary = crate::core::klippy::mcu::Dictionary::from_json(serde_json::json!({
            "commands": {
                "config_hd44780 oid=%c rs_pin=%u e_pin=%u d4_pin=%u d5_pin=%u d6_pin=%u d7_pin=%u delay_ticks=%u": 60,
                "hd44780_send_cmds oid=%c cmds=%*s": 61,
                "hd44780_send_data oid=%c data=%*s": 62,
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

    /// A `[display]` section's pins and options, as the loader would hand it
    /// over.
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

    /// The six pins of the corpus's RAMPS boards (`printer-bq-hephestos-2014.cfg`),
    /// all on the default MCU.
    const SIX_PINS: &[(&str, &str)] = &[
        ("rs_pin", "PH1"),
        ("e_pin", "PH0"),
        ("d4_pin", "PA1"),
        ("d5_pin", "PA3"),
        ("d6_pin", "PA5"),
        ("d7_pin", "PA7"),
    ];

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

    // -- the firmware commands ---------------------------------------------

    #[test]
    fn test_the_config_command_keeps_the_firmware_format() {
        let command = ConfigHd44780 {
            oid: 3,
            rs_pin: 16,
            e_pin: 17,
            d4_pin: 18,
            d5_pin: 19,
            d6_pin: 20,
            d7_pin: 21,
            delay_ticks: 800,
        };
        assert_eq!(
            roundtrip(&command),
            vec![
                ArgValue::UInt8(3),
                ArgValue::UInt32(16),
                ArgValue::UInt32(17),
                ArgValue::UInt32(18),
                ArgValue::UInt32(19),
                ArgValue::UInt32(20),
                ArgValue::UInt32(21),
                ArgValue::UInt32(800),
            ]
        );
    }

    #[test]
    fn test_the_send_commands_carry_binary_bytes() {
        // A high byte proves the parameter is `%*s` (binary), not text.
        let cmds = Hd44780SendCmds {
            oid: 1,
            cmds: vec![0x28, 0x28, 0x02],
        };
        assert_eq!(
            roundtrip(&cmds),
            vec![ArgValue::UInt8(1), ArgValue::Bytes(vec![0x28, 0x28, 0x02])]
        );
        let data = Hd44780SendData {
            oid: 1,
            data: vec![0xff; 4],
        };
        assert_eq!(
            roundtrip(&data),
            vec![ArgValue::UInt8(1), ArgValue::Bytes(vec![0xff; 4])]
        );
    }

    // -- options ------------------------------------------------------------

    #[test]
    fn test_the_options_are_read_with_upstream_defaults() {
        // `hd44780_protocol_init` defaults to True and `line_length` to 20
        // (`hd44780.py:10,24-27`), and both are recorded as read — `check_unused`
        // requires a reader for every option a config writes.
        let printer = printer();
        let section = display_section(SIX_PINS);
        let access = AccessTracking::shared();
        let chip = Hd44780::new(&ConfigWrapper::new(&section, Arc::clone(&access)), &printer)
            .expect("the section loads");

        assert!(chip.state.hd44780_protocol_init);
        assert_eq!(chip.state.line_length, 20);
        assert_eq!(chip.get_dimensions(), (20, 4));
        for option in [
            "rs_pin",
            "e_pin",
            "d4_pin",
            "d5_pin",
            "d6_pin",
            "d7_pin",
            "hd44780_protocol_init",
            "line_length",
        ] {
            assert!(access.contains("display", option), "option {option}");
        }
    }

    #[test]
    fn test_line_length_and_protocol_init_change_the_panel() {
        let printer = printer();
        let mut options = SIX_PINS.to_vec();
        options.push(("line_length", "16"));
        options.push(("hd44780_protocol_init", "False"));
        let section = display_section(&options);

        let chip =
            Hd44780::new(&ConfigWrapper::untracked(&section), &printer).expect("the section loads");

        assert!(!chip.state.hd44780_protocol_init);
        assert_eq!(chip.state.line_length, 16);
        assert_eq!(chip.get_dimensions(), (16, 4));
        // The framebuffers follow the width: two half-lines of 16 characters
        // (`hd44780.py:40-41`).
        assert_eq!(chip.framebuffers().text[0].data.len(), 32);
    }

    #[test]
    fn test_an_unknown_line_length_is_a_choice_error() {
        let printer = printer();
        let mut options = SIX_PINS.to_vec();
        options.push(("line_length", "18"));
        let section = display_section(&options);

        let err = Hd44780::new(&ConfigWrapper::untracked(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Choice '18' for option 'line_length' in section 'display' is not a valid choice"
        );
    }

    // -- pin validation -----------------------------------------------------

    #[test]
    fn test_a_missing_pin_is_reported_the_way_the_config_reports_it() {
        let printer = printer();
        let section = display_section(&[("rs_pin", "PH1")]);

        let err = Hd44780::new(&ConfigWrapper::untracked(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'e_pin' in section 'display' must be specified"
        );
    }

    #[test]
    fn test_the_last_pin_is_read_after_the_five_before_it() {
        let printer = printer();
        let section = display_section(&SIX_PINS[..5]);

        let err = Hd44780::new(&ConfigWrapper::untracked(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'd7_pin' in section 'display' must be specified"
        );
    }

    #[test]
    fn test_the_six_pins_must_be_on_one_mcu() {
        let printer = printer();
        let mut options = SIX_PINS.to_vec();
        options[0] = ("rs_pin", "board2:PA3");
        let section = display_section(&options);

        let err = Hd44780::new(&ConfigWrapper::untracked(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(err.to_string(), "hd44780 all pins must be on same mcu");
    }

    #[test]
    fn test_the_options_are_read_before_the_pin_conflict() {
        // Upstream reads the two options between the pins and the "one MCU"
        // rule (`hd44780.py:19-32`), so a bad `line_length` is the error the
        // user sees, not the conflict.
        let printer = printer();
        let mut options = SIX_PINS.to_vec();
        options[0] = ("rs_pin", "board2:PA3");
        options.push(("line_length", "18"));
        let section = display_section(&options);

        let err = Hd44780::new(&ConfigWrapper::untracked(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Choice '18' for option 'line_length' in section 'display' is not a valid choice"
        );
    }

    // -- the panel -----------------------------------------------------------

    #[test]
    fn test_the_init_sequence_matches_upstream() {
        // `hd44780.py:94-101`: the 4-bit protocol handshake, the home, and the
        // shared "positive direction, display on, cursor hidden" group.
        assert_eq!(
            init_commands(true),
            vec![
                vec![0x33],
                vec![0x33],
                vec![0x32],
                vec![0x28, 0x28, 0x02],
                vec![0x06, 0x0c],
            ]
        );
        // Without the handshake only the home command is sent first.
        assert_eq!(init_commands(false), vec![vec![0x02], vec![0x06, 0x0c]]);
    }

    #[test]
    fn test_the_framebuffers_start_unsent() {
        // Every "already sent" copy is `~`, so the first flush writes the whole
        // screen (`hd44780.py:40-50`).
        let framebuffers = Framebuffers::new(20);
        assert_eq!(framebuffers.text[0].data, vec![b' '; 40]);
        assert_eq!(framebuffers.text[0].synced, vec![b'~'; 40]);
        assert_eq!(framebuffers.text[0].fb_id, 0x80);
        assert_eq!(framebuffers.text[1].data, vec![b' '; 40]);
        assert_eq!(framebuffers.text[1].fb_id, 0xc0);
        assert_eq!(framebuffers.glyph.data, vec![0; 64]);
        assert_eq!(framebuffers.glyph.synced, vec![b'~'; 64]);
        assert_eq!(framebuffers.glyph.fb_id, 0x40);
    }

    #[test]
    fn test_clear_blanks_the_text_but_keeps_the_glyph_buffer() {
        let printer = printer();
        let section = display_section(SIX_PINS);
        let chip =
            Hd44780::new(&ConfigWrapper::untracked(&section), &printer).expect("the section loads");
        {
            let mut framebuffers = chip.framebuffers();
            framebuffers.text[0].data[0] = b'x';
            framebuffers.text[1].data[0] = b'y';
            framebuffers.glyph.data[0] = 0xff;
        }

        chip.clear();

        let framebuffers = chip.framebuffers();
        assert_eq!(framebuffers.text[0].data, vec![b' '; 40]);
        assert_eq!(framebuffers.text[1].data, vec![b' '; 40]);
        assert_eq!(framebuffers.glyph.data[0], 0xff);
    }

    #[test]
    fn test_set_glyphs_keeps_the_5x8_icons() {
        let printer = printer();
        let section = display_section(SIX_PINS);
        let chip =
            Hd44780::new(&ConfigWrapper::untracked(&section), &printer).expect("the section loads");

        let mut glyphs: BTreeMap<String, Glyph> = BTreeMap::new();
        glyphs.insert(
            "thermometer".to_string(),
            Glyph {
                icon16x16: Some((vec![0xff; 16], vec![0xff; 16])),
                icon5x8: Some((3, vec![0x0e; 8])),
            },
        );
        glyphs.insert(
            "fan1".to_string(),
            Glyph {
                icon16x16: Some((vec![0xff; 16], vec![0xff; 16])),
                icon5x8: None,
            },
        );
        chip.set_glyphs(&glyphs);

        // Only the HD44780-sized glyphs are kept (`hd44780.py:109-113`).
        assert_eq!(
            *chip.state.icons.lock().unwrap(),
            HashMap::from([("thermometer".to_string(), (3u8, vec![0x0e; 8]))])
        );
    }

    #[test]
    fn test_the_unconnected_panel_cannot_send() {
        // Without a connected MCU there is nothing to write to; the driver
        // reports it instead of panicking, so an `init` that runs before the
        // machine is up only logs.
        let printer = printer();
        let section = display_section(SIX_PINS);
        let chip =
            Hd44780::new(&ConfigWrapper::untracked(&section), &printer).expect("the section loads");
        assert_eq!(chip.sent_message_count(), 0);

        let err = chip.send(&[0x33], false).unwrap_err();
        assert!(err.to_string().contains("not connected"), "{err}");
        assert_eq!(chip.sent_message_count(), 0);

        // `init` swallows that error: it logs and stops instead of failing the
        // printer.
        chip.init();
        assert_eq!(chip.sent_message_count(), 0);
    }

    // -- against the fake firmware ------------------------------------------

    /// The whole panel against the dictionary-driven fake firmware: the init
    /// groups arrive as commands, then the first flush writes both text
    /// framebuffers and the character generator buffer.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_the_panel_is_written_to_the_fake_firmware() {
        use crate::core::klippy::api::StartArgs;
        use crate::core::klippy::extras::display::display::PrinterLCD;
        use crate::core::klippy::printer::PrinterState;
        use crate::core::klippy::reactor::TokioReactor;

        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        if !dict.is_file() {
            return;
        }
        let text = format!(
            "[mcu]\ntest: dict={}\n[display]\nlcd_type: hd44780\n{}",
            dict.display(),
            SIX_PINS
                .iter()
                .map(|(key, value)| format!("{key}: {value}\n"))
                .collect::<String>()
        );
        let config = Config::from_text(&text).expect("the config parses").0;
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
                .lookup_object_as::<PrinterLCD>("display")
                .expect("the display sits under its section id");
            Ok::<_, String>(display.sent_messages())
        }
        .await;

        printer.teardown();
        let messages = outcome.expect("the display comes up");

        // The init sequence, one `hd44780_send_cmds` per group
        // (`hd44780.py:94-101`).
        let init: Vec<SentMessage> = init_commands(true)
            .into_iter()
            .map(|bytes| SentMessage {
                is_data: false,
                bytes,
            })
            .collect();
        assert_eq!(messages[..init.len()], init[..]);

        // And the first flush wrote the blank screen. Upstream's batching
        // (`hd44780.py:78-84`) splits each 40-byte text buffer into runs of 8,
        // 16 and 16 bytes, each as a position command (0x80 + position) and its
        // data; the second text buffer starts at 0xc0 and the character
        // generator buffer at 0x40.
        let first = &messages[init.len()..];
        assert_eq!(
            first[..4].to_vec(),
            vec![
                SentMessage {
                    is_data: false,
                    bytes: vec![0x80]
                },
                SentMessage {
                    is_data: true,
                    bytes: vec![b' '; 8]
                },
                SentMessage {
                    is_data: false,
                    bytes: vec![0x88]
                },
                SentMessage {
                    is_data: true,
                    bytes: vec![b' '; 16]
                },
            ]
        );
        for fb_id in [0x80u8, 0xc0, 0x40] {
            assert!(
                first
                    .iter()
                    .any(|message| !message.is_data && message.bytes == vec![fb_id]),
                "the flush addresses framebuffer {fb_id:#04x}: {messages:?}"
            );
        }
    }
}
