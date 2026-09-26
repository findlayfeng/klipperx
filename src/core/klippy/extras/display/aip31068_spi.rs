//! `aip31068_spi` — the AIP31068 (20x4 text) panel driver on an SPI backpack
//! (upstream `klippy/extras/display/aip31068_spi.py`).
//!
//! A `[display]` section with `lcd_type: aip31068_spi` builds one of these. The
//! controller is HD44780-compatible but speaks a different transport: every
//! command or data word is nine bits — a register-select bit in front of eight
//! bits — and the panel sits behind an SPI bus whose chip select is
//! `latch_pin` (`aip31068_spi.py:81-83`). Upstream keeps the SW_SPI driver at
//! whole bytes by sending eight of those nine-bit words as nine bytes, so one
//! `spi_send` carries a whole group (`aip31068_spi.py:112-136`).
//!
//! | framework call | here | upstream |
//! |---|---|---|
//! | `init()` | the five power-up commands, then a flush | `aip31068_spi.py:159-168` |
//! | `clear()` | blank both text framebuffers | `aip31068_spi.py:190-193` |
//! | `flush()` | batch the framebuffer differences and send them | `aip31068_spi.py:137-157` |
//! | `set_glyphs()` | keep the 5x8 icons for the glyphs the layout names | `aip31068_spi.py:169-174` |
//!
//! As `hd44780`, the framebuffers are the two text buffers of `2*line_length`
//! bytes — each of the panel's four rows is one half of one buffer — plus the
//! 64-byte character generator buffer, and every "already sent" copy starts as
//! `~`, so the first flush writes the whole screen (`aip31068_spi.py:88-111`).
//! `line_length` is 16 or 20 (`aip31068_spi.py:20-21,86-87`) and decides the
//! panel's width, and with it the default `display_group`.
//!
//! # What is not here
//!
//! * **Text and glyph drawing.** [`super::display`] never draws, so
//!   `write_text`/`write_glyph`/`write_graphics` (`aip31068_spi.py:176-189`) are
//!   not ported; [`Aip31068Spi::set_glyphs`] keeps the 5x8 icons because
//!   upstream collects them there (`aip31068_spi.py:169-174`).
//! * **`minclock`** (`aip31068_spi.py:159-168`): upstream stamps each init group
//!   with a clock deadline 100ms apart; this host's
//!   [`McuCommand`](crate::core::klippy::cmd::McuCommand) carries no clock, so
//!   commands go out in call order.

use std::collections::{BTreeMap, HashMap};
use std::sync::{Arc, Mutex, MutexGuard};

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::spi_device::mcu_spi_from_config;
use crate::core::klippy::mcu::{McuError, McuSpi};
use crate::core::klippy::printer::{Printer, PrinterObject};

use super::display::{Glyph, LcdChip, SentMessage};

/// The panel's width when `line_length` is not written (`aip31068_spi.py:20`).
const LINE_LENGTH_DEFAULT: &str = "20";
/// The widths `line_length` may choose between (`aip31068_spi.py:21`).
const LINE_LENGTH_OPTIONS: &[&str] = &["16", "20"];

/// Upstream's `MCU_SPI_from_config(config, 0x00, pin_option="latch_pin")`
/// (`aip31068_spi.py:82-83`): SPI mode 0 with the default 100 kHz clock
/// (`bus.py:124`).
const SPI_SPEED: u32 = 100_000;

// Upstream's `CMND` opcodes (`aip31068_spi.py:29-38`) and the flag bits the
// driver uses (`aip31068_spi.py:41-70`).

/// `CMND.HOME` — move the cursor home (`aip31068_spi.py:31`).
const CMND_HOME: u16 = 2;
/// `CMND.ENTERY_MODE` — entry-mode set (`aip31068_spi.py:32`).
const CMND_ENTERY_MODE: u16 = 1 << 2;
/// `flg_ENTERY_MODE.INC` — increment the cursor (`aip31068_spi.py:42`).
const FLG_ENTERY_MODE_INC: u16 = 1 << 1;
/// `CMND.DISPLAY` — display on/off control (`aip31068_spi.py:33`).
const CMND_DISPLAY: u16 = 1 << 3;
/// `flg_DISPLAY.ON` (`aip31068_spi.py:47`).
const FLG_DISPLAY_ON: u16 = 1 << 2;
/// `CMND.SHIFT` — cursor or display shift (`aip31068_spi.py:34`).
const CMND_SHIFT: u16 = 1 << 4;
/// `flg_SHIFT.RIGHT` (`aip31068_spi.py:52`).
const FLG_SHIFT_RIGHT: u16 = 1 << 2;
/// `CMND.FUNCTION` — function set (`aip31068_spi.py:35`).
const CMND_FUNCTION: u16 = 1 << 5;
/// `flg_FUNCTION.TWO_LINES` (`aip31068_spi.py:57`).
const FLG_FUNCTION_TWO_LINES: u16 = 1 << 3;
/// `CMND.CGRAM` — character generator RAM (`aip31068_spi.py:36`).
const CMND_CGRAM: u16 = 1 << 6;
/// `CMND.DDRAM` — display data RAM (`aip31068_spi.py:37`).
const CMND_DDRAM: u16 = 1 << 7;
/// `flg_DDRAM.MASK` (`aip31068_spi.py:64`).
const FLG_DDRAM_MASK: u16 = 0b0111_1111;
/// `CMND.WRITE_RAM` (`aip31068_spi.py:38`).
const CMND_WRITE_RAM: u16 = 1 << 8;

/// The number of bits in one command/data word (`aip31068_spi.py:40-46`).
const COMMAND_BITS: u32 = 9;
/// A group of this many words encodes to exactly nine bytes
/// (`aip31068_spi.py:112-136`).
const GROUP_WORDS: usize = 8;
/// The pad for a short group: the fast entry-mode command, whose 39us is far
/// shorter than a clear (`aip31068_spi.py:129-131`).
const SEND_PAD: u16 = CMND_ENTERY_MODE | FLG_ENTERY_MODE_INC;

/// Upstream's `DISPLAY_INIT_CMNDS` (`aip31068_spi.py:71-79`): home, positive
/// entry direction, display on with the cursor and blink off, right shift, and
/// the 2-line 5x8 function set. No clear — the first flush rewrites the screen.
const DISPLAY_INIT_CMNDS: &[u16] = &[
    CMND_HOME,
    CMND_ENTERY_MODE | FLG_ENTERY_MODE_INC,
    CMND_DISPLAY | FLG_DISPLAY_ON,
    CMND_SHIFT | FLG_SHIFT_RIGHT,
    CMND_FUNCTION | FLG_FUNCTION_TWO_LINES,
];

/// One framebuffer and the copy the firmware has already been told about —
/// upstream's `(new_data, old_data, fb_cmnd)` triples
/// (`aip31068_spi.py:100-110`).
#[derive(Debug)]
struct Framebuffer {
    /// What the screen should show.
    data: Vec<u8>,
    /// What the firmware's copy holds; the difference is what a flush sends.
    synced: Vec<u8>,
    /// The base RAM address command the flush resolves positions against.
    fb_cmnd: u8,
}

/// The two text framebuffers and the character generator's buffer, in the order
/// upstream flushes them (`aip31068_spi.py:100-110`).
#[derive(Debug)]
struct Framebuffers {
    /// One buffer per pair of panel rows (`aip31068_spi.py:89-90`).
    text: [Framebuffer; 2],
    /// The 64-byte CGRAM buffer (`aip31068_spi.py:91`).
    glyph: Framebuffer,
}

impl Framebuffers {
    /// Upstream's `__init__` buffers: the text screens are spaces, the glyph
    /// buffer is zeros, and every "already sent" copy is `~` so the first flush
    /// sends the lot (`aip31068_spi.py:88-111`).
    fn new(line_length: usize) -> Self {
        let half = 2 * line_length;
        let blank = |len: usize, byte: u8, fb_cmnd: u8| Framebuffer {
            data: vec![byte; len],
            synced: vec![b'~'; len],
            fb_cmnd,
        };
        Self {
            text: [
                // The first text buffer starts at RAM 0:
                // `CMND.DDRAM | (flg_DDRAM.MASK & 0x00)` (`aip31068_spi.py:102-103`).
                blank(half, b' ', CMND_DDRAM as u8),
                // The second starts half-way through the RAM:
                // `CMND.DDRAM | (flg_DDRAM.MASK & 0x40)` (`aip31068_spi.py:104-105`).
                blank(half, b' ', (CMND_DDRAM | (FLG_DDRAM_MASK & 0x40)) as u8),
            ],
            // The glyph buffer sits at RAM 0 of the character generator:
            // `CMND.CGRAM | (flg_CGRAM.MASK & 0x00)` (`aip31068_spi.py:106-108`).
            glyph: blank(64, 0, CMND_CGRAM as u8),
        }
    }

    /// The three buffers in flush order, each with its two byte vectors and its
    /// address command.
    fn iter_mut(&mut self) -> [(&mut Vec<u8>, &mut Vec<u8>, u8); 3] {
        let [first, second] = &mut self.text;
        [
            (&mut first.data, &mut first.synced, first.fb_cmnd),
            (&mut second.data, &mut second.synced, second.fb_cmnd),
            (
                &mut self.glyph.data,
                &mut self.glyph.synced,
                self.glyph.fb_cmnd,
            ),
        ]
    }
}

/// One `lcd_type: aip31068_spi` panel.
pub struct Aip31068Spi {
    /// The SPI bus `latch_pin` addressed (`aip31068_spi.py:81-83`).
    spi: Arc<McuSpi>,
    /// The panel's width in characters (`aip31068_spi.py:86-87`).
    line_length: usize,
    /// The framebuffers.
    framebuffers: Mutex<Framebuffers>,
    /// The 5x8 icons by glyph name (`aip31068_spi.py:169-174`).
    icons: Mutex<HashMap<String, (u8, Vec<u8>)>>,
    /// Every message handed to the firmware, in order (tests and diagnostics).
    sent: Mutex<Vec<SentMessage>>,
}

impl Aip31068Spi {
    /// Read the SPI bus and `line_length`.
    ///
    /// Upstream's `aip31068_spi.__init__` (`aip31068_spi.py:80-87`): the bus
    /// options first — `latch_pin` among them — and only then `line_length`.
    ///
    /// # Errors
    /// A missing `latch_pin` (upstream's `config.get(pin_option)`), an
    /// unresolvable or already-used pin, a chip-select pin on another MCU, a
    /// malformed `spi_*` option, a `line_length` outside
    /// [`LINE_LENGTH_OPTIONS`], or an unknown `spi_mcu`.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        // `latch_pin` is required: unlike a temperature sensor, the panel has
        // no "no chip select" mode (`aip31068_spi.py:82-83`).
        config.get("latch_pin", None)?;
        let setup = mcu_spi_from_config(config, printer.as_ref(), 0, "latch_pin", SPI_SPEED)?;
        let line_length = config
            .get_choice(
                "line_length",
                LINE_LENGTH_OPTIONS,
                Some(LINE_LENGTH_DEFAULT),
            )?
            .parse::<usize>()
            .expect("the choice is one of LINE_LENGTH_OPTIONS");

        Ok(Self {
            spi: setup.device,
            line_length,
            framebuffers: Mutex::new(Framebuffers::new(line_length)),
            icons: Mutex::new(HashMap::new()),
            sent: Mutex::new(Vec::new()),
        })
    }

    /// Send one list of nine-bit words: split into groups of eight, pad the
    /// last group, encode each as nine bytes, and hand them to the bus
    /// (`aip31068_spi.py:128-136`).
    ///
    /// # Errors
    /// The first `spi_send` that fails stops the send with that error; the
    /// groups already sent stay recorded.
    fn send(&self, data: &[u16]) -> Result<(), McuError> {
        let mut sent = self
            .sent
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        for group in encoded_groups(data) {
            self.spi.send(&group)?;
            sent.push(SentMessage {
                is_data: false,
                bytes: group,
            });
        }
        Ok(())
    }

    /// The framebuffers, for the trait methods.
    fn framebuffers(&self) -> MutexGuard<'_, Framebuffers> {
        self.framebuffers
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl LcdChip for Aip31068Spi {
    /// Upstream's `aip31068_spi.init` (`aip31068_spi.py:159-168`): the five
    /// power-up commands, one message each, then the first flush.
    fn init(&self) {
        for command in DISPLAY_INIT_CMNDS {
            if let Err(err) = self.send(&[*command]) {
                warn!("aip31068_spi: could not initialise the panel: {err}");
                return;
            }
        }
        self.flush();
    }

    /// Upstream's `aip31068_spi.clear` (`aip31068_spi.py:190-193`): only the
    /// text framebuffers are blanked; the glyph buffer keeps the icons.
    fn clear(&self) {
        let mut framebuffers = self.framebuffers();
        for text in &mut framebuffers.text {
            text.data.fill(b' ');
        }
    }

    /// Upstream's `aip31068_spi.flush` (`aip31068_spi.py:137-157`): send the
    /// changed bytes of every framebuffer, batching changes that are close
    /// together.
    fn flush(&self) {
        let mut framebuffers = self.framebuffers();
        for (data, synced, fb_cmnd) in framebuffers.iter_mut() {
            let data = data.clone();
            if data == *synced {
                continue;
            }
            for message in flush_messages(&data, synced, fb_cmnd) {
                if let Err(err) = self.send(&message) {
                    warn!("aip31068_spi: could not flush the panel: {err}");
                    return;
                }
            }
            synced.copy_from_slice(&data);
        }
    }

    /// The panel's size in characters (`aip31068_spi.py:195-196`).
    fn get_dimensions(&self) -> (usize, usize) {
        (self.line_length, 4)
    }

    /// Upstream's `aip31068_spi.set_glyphs` (`aip31068_spi.py:169-174`): keep
    /// every 5x8 icon by glyph name.
    fn set_glyphs(&self, glyphs: &BTreeMap<String, Glyph>) {
        let mut icons = self
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
    ///
    /// The SPI transport has one channel — the register-select bit sits inside
    /// the nine-bit word — so `is_data` is always false here.
    fn sent_messages(&self) -> Vec<SentMessage> {
        self.sent
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }
}

impl PrinterObject for Aip31068Spi {
    /// Upstream's `aip31068_spi` has no `get_status`, so it is not
    /// client-visible.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for Aip31068Spi {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Aip31068Spi")
            .field("line_length", &self.line_length)
            .finish_non_exhaustive()
    }
}

/// Upstream's `aip31068_spi.encode` (`aip31068_spi.py:112-136`): pack each
/// `width`-bit word of `data` into a big-endian bit stream, emitting a byte
/// whenever eight bits have accumulated and right-padding the last byte with
/// zero bits.
///
/// Every word the driver sends fits in [`COMMAND_BITS`] — the largest is
/// `CMND.WRITE_RAM | byte` — which the debug assertion states.
fn encode(data: &[u16], width: u32) -> Vec<u8> {
    let mut encoded = Vec::new();
    let mut accumulator: u64 = 0;
    let mut acc_bits: u32 = 0;
    for &num in data {
        debug_assert!(
            u32::from(num) < (1 << width),
            "number {num} does not fit in {width} bits"
        );
        accumulator = (accumulator << width) | u64::from(num);
        acc_bits += width;
        while acc_bits >= 8 {
            acc_bits -= 8;
            encoded.push(((accumulator >> acc_bits) & 0xff) as u8);
            accumulator &= (1 << acc_bits) - 1;
        }
    }
    if acc_bits > 0 {
        encoded.push((accumulator << (8 - acc_bits)) as u8);
    }
    encoded
}

/// Upstream's `aip31068_spi.send` (`aip31068_spi.py:128-136`): the nine-bit
/// words of `data` as groups of eight, the last group padded with
/// [`SEND_PAD`], each group encoded to nine bytes.
fn encoded_groups(data: &[u16]) -> Vec<Vec<u8>> {
    let mut groups = Vec::new();
    for group in data.chunks(GROUP_WORDS) {
        let mut words = group.to_vec();
        words.resize(GROUP_WORDS, SEND_PAD);
        groups.push(encode(&words, COMMAND_BITS));
    }
    groups
}

/// Upstream's flush batching (`aip31068_spi.py:146-153`): the positions of
/// every changed byte as `(position, length)` runs, with runs closer than five
/// bytes joined while the run ahead is shorter than sixteen.
fn changed_runs(data: &[u8], synced: &[u8]) -> Vec<(usize, usize)> {
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
        if pos + 4 >= next_pos && next_count < 16 {
            diffs[i] = (pos, next_count + (next_pos - pos));
            diffs.remove(i + 1);
        }
    }
    diffs
}

/// The send arguments `flush` produces for one framebuffer
/// (`aip31068_spi.py:154-156`): for every batched run, the position command
/// (`fb_cmnd + pos`) and then the run's bytes as `CMND.WRITE_RAM | byte`.
fn flush_messages(data: &[u8], synced: &[u8], fb_cmnd: u8) -> Vec<Vec<u16>> {
    let mut messages = Vec::new();
    for (pos, count) in changed_runs(data, synced) {
        messages.push(vec![u16::from(fb_cmnd) + pos as u16]);
        messages.push(
            data[pos..pos + count]
                .iter()
                .map(|byte| CMND_WRITE_RAM | u16::from(*byte))
                .collect(),
        );
    }
    messages
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
    use crate::core::klippy::mcu::McuObject;
    use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[display]` section's options, as the loader would hand it over.
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
    /// section runs: `mcu`, and `board2` for the "must be on the panel's MCU"
    /// checks.
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

    /// The AIP31068 corpus shape (`printer-geeetech-A10T-A20T-2021.cfg`): one
    /// `latch_pin` on the default MCU.
    const AIP31068_SECTION: &[(&str, &str)] = &[
        ("lcd_type", "aip31068_spi"),
        ("latch_pin", "PA1"),
        ("spi_speed", "4000000"),
    ];

    // -- options ------------------------------------------------------------

    #[test]
    fn test_the_options_are_read_with_upstream_defaults() {
        // Without `line_length` the panel is 20 wide (`aip31068_spi.py:20,86-87`),
        // and reading it records the option — `check_unused` requires a reader
        // for every option a config writes.
        let printer = printer();
        let section = display_section(&[("latch_pin", "PA1")]);
        let access = AccessTracking::shared();
        let chip = Aip31068Spi::new(&ConfigWrapper::new(&section, Arc::clone(&access)), &printer)
            .expect("the section loads");

        assert_eq!(chip.line_length, 20);
        assert_eq!(chip.get_dimensions(), (20, 4));
        for option in ["latch_pin", "line_length"] {
            assert!(access.contains("display", option), "option {option}");
        }
    }

    #[test]
    fn test_line_length_16_makes_a_narrower_panel() {
        let printer = printer();
        let mut options = AIP31068_SECTION.to_vec();
        options.push(("line_length", "16"));
        let section = display_section(&options);

        let chip = Aip31068Spi::new(&ConfigWrapper::untracked(&section), &printer)
            .expect("the section loads");

        assert_eq!(chip.line_length, 16);
        assert_eq!(chip.get_dimensions(), (16, 4));
        // The framebuffers follow the width: two half-lines of 16 characters
        // (`aip31068_spi.py:89-90`).
        assert_eq!(chip.framebuffers().text[0].data.len(), 32);
    }

    #[test]
    fn test_an_unknown_line_length_is_a_choice_error() {
        let printer = printer();
        let mut options = AIP31068_SECTION.to_vec();
        options.push(("line_length", "18"));
        let section = display_section(&options);

        let err = Aip31068Spi::new(&ConfigWrapper::untracked(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Choice '18' for option 'line_length' in section 'display' is not a valid choice"
        );
    }

    // -- pin and bus validation ---------------------------------------------

    #[test]
    fn test_the_latch_pin_is_required() {
        let printer = printer();
        let section = display_section(&[("spi_speed", "4000000")]);

        let err = Aip31068Spi::new(&ConfigWrapper::untracked(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'latch_pin' in section 'display' must be specified"
        );
    }

    #[test]
    fn test_the_latch_pin_must_be_on_the_spi_mcu() {
        // Upstream's `MCU_SPI_from_config` refuses a chip-select pin on another
        // MCU (`bus.py:129-130`).
        let printer = printer();
        let section = display_section(&[("latch_pin", "board2:PA1")]);

        let err = Aip31068Spi::new(&ConfigWrapper::untracked(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Section 'display': latch_pin must be on mcu 'mcu'"
        );
    }

    #[test]
    fn test_a_partly_written_software_bus_is_refused() {
        let printer = printer();
        let mut options = AIP31068_SECTION.to_vec();
        options.push(("spi_software_sclk_pin", "PB0"));
        let section = display_section(&options);

        let err = Aip31068Spi::new(&ConfigWrapper::untracked(&section), &printer)
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

    #[test]
    fn test_a_too_slow_spi_clock_is_refused() {
        let printer = printer();
        let section = display_section(&[("latch_pin", "PA1"), ("spi_speed", "1000")]);

        let err = Aip31068Spi::new(&ConfigWrapper::untracked(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'spi_speed' in section 'display' must be at least 100000"
        );
    }

    // -- the encoding --------------------------------------------------------

    #[test]
    fn test_the_init_sequence_is_upstreams_five_commands() {
        // `aip31068_spi.py:71-79`: home, entry mode with INC, display off's
        // other half plus ON, right shift, and the 2-line 5x8 function set.
        assert_eq!(DISPLAY_INIT_CMNDS, &[0x02, 0x06, 0x0c, 0x14, 0x28]);
    }

    #[test]
    fn test_eight_nine_bit_words_are_packed_into_nine_bytes() {
        // The pad word's own bytes are the same tail for every group
        // (`aip31068_spi.py:112-136`): 8x9 = 72 bits = nine bytes.
        let pad = SEND_PAD;
        // A single word is nine bits, so it fills one byte and one bit of the
        // next, which is right-padded with zeros.
        assert_eq!(encode(&[0x02], COMMAND_BITS), vec![0x01, 0x00]);
        assert_eq!(
            encoded_groups(&[CMND_HOME])[0],
            vec![0x01, 0x01, 0x80, 0xc0, 0x60, 0x30, 0x18, 0x0c, 0x06]
        );
        assert_eq!(
            encoded_groups(&[CMND_ENTERY_MODE | FLG_ENTERY_MODE_INC])[0],
            vec![0x03, 0x01, 0x80, 0xc0, 0x60, 0x30, 0x18, 0x0c, 0x06]
        );
        assert_eq!(
            encoded_groups(&[CMND_DISPLAY | FLG_DISPLAY_ON])[0],
            vec![0x06, 0x01, 0x80, 0xc0, 0x60, 0x30, 0x18, 0x0c, 0x06]
        );
        assert_eq!(
            encoded_groups(&[CMND_SHIFT | FLG_SHIFT_RIGHT])[0],
            vec![0x0a, 0x01, 0x80, 0xc0, 0x60, 0x30, 0x18, 0x0c, 0x06]
        );
        assert_eq!(
            encoded_groups(&[CMND_FUNCTION | FLG_FUNCTION_TWO_LINES])[0],
            vec![0x14, 0x01, 0x80, 0xc0, 0x60, 0x30, 0x18, 0x0c, 0x06]
        );
        // `pad` is the value the short groups are padded with.
        assert_eq!(pad, 0x06);

        // A full group of eight data words encodes past the byte boundary: the
        // nine-bit `WRITE_RAM | ' '` (0x120) words are 0b1_0010_0000 each.
        assert_eq!(
            encoded_groups(&[CMND_WRITE_RAM | 0x20; 8])[0],
            vec![0x90, 0x48, 0x24, 0x12, 0x09, 0x04, 0x82, 0x41, 0x20]
        );
    }

    #[test]
    fn test_a_long_message_becomes_one_nine_byte_group_per_eight_words() {
        // Sixteen words is two `spi_send` calls (`aip31068_spi.py:130-136`).
        let words = vec![CMND_WRITE_RAM | 0x20; 16];
        let groups = encoded_groups(&words);
        assert_eq!(groups.len(), 2);
        assert_eq!(groups[0], groups[1]);
        assert_eq!(groups[0].len(), 9);
    }

    // -- the framebuffers ----------------------------------------------------

    #[test]
    fn test_the_framebuffers_start_unsent() {
        // Every "already sent" copy is `~`, so the first flush writes the whole
        // screen (`aip31068_spi.py:88-111`).
        let framebuffers = Framebuffers::new(20);
        assert_eq!(framebuffers.text[0].data, vec![b' '; 40]);
        assert_eq!(framebuffers.text[0].synced, vec![b'~'; 40]);
        assert_eq!(framebuffers.text[0].fb_cmnd, 0x80);
        assert_eq!(framebuffers.text[1].data, vec![b' '; 40]);
        assert_eq!(framebuffers.text[1].fb_cmnd, 0xc0);
        assert_eq!(framebuffers.glyph.data, vec![0; 64]);
        assert_eq!(framebuffers.glyph.synced, vec![b'~'; 64]);
        assert_eq!(framebuffers.glyph.fb_cmnd, 0x40);
    }

    #[test]
    fn test_clear_blanks_the_text_but_keeps_the_glyph_buffer() {
        let printer = printer();
        let section = display_section(AIP31068_SECTION);
        let chip = Aip31068Spi::new(&ConfigWrapper::untracked(&section), &printer)
            .expect("the section loads");
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
        let section = display_section(AIP31068_SECTION);
        let chip = Aip31068Spi::new(&ConfigWrapper::untracked(&section), &printer)
            .expect("the section loads");

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

        // Only the HD44780-sized glyphs are kept (`aip31068_spi.py:169-174`).
        assert_eq!(
            *chip.icons.lock().unwrap(),
            HashMap::from([("thermometer".to_string(), (3u8, vec![0x0e; 8]))])
        );
    }

    // -- the flush plan ------------------------------------------------------

    #[test]
    fn test_close_changes_are_batched_into_one_run() {
        // The upstream rule: a change runs up to four bytes ahead while the run
        // ahead is shorter than sixteen (`aip31068_spi.py:146-153`).
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

        // A gap of four bytes still joins (`pos + 4 >= next_pos`); a gap of
        // five does not, and a sixteen-byte run ahead stops the merge.
        let mut data = vec![0; 6];
        data[0] = 1;
        data[4] = 1;
        assert_eq!(changed_runs(&data, &[0; 6]), vec![(0, 5)]);
        let mut data = vec![0; 7];
        data[0] = 1;
        data[5] = 1;
        assert_eq!(changed_runs(&data, &[0; 7]), vec![(0, 1), (5, 1)]);
    }

    #[test]
    fn test_the_first_flush_writes_the_blank_screen() {
        // The 40-byte text buffer splits into runs of 8, 16 and 16
        // (`aip31068_spi.py:146-153`), each as a position command and its data.
        let synced = vec![b'~'; 40];
        let data = vec![b' '; 40];
        let messages = flush_messages(&data, &synced, 0x80);

        // Six messages: (0x80, 8 bytes), (0x88, 16 bytes), (0x98, 16 bytes).
        let positions: Vec<u16> = messages
            .iter()
            .step_by(2)
            .map(|message| message[0])
            .collect();
        assert_eq!(positions, vec![0x80, 0x88, 0x98]);
        assert_eq!(messages[1].len(), 8);
        assert_eq!(messages[3].len(), 16);
        assert_eq!(messages[5].len(), 16);
        // Every data word is `CMND.WRITE_RAM | ' '`.
        assert!(messages[1].iter().all(|word| *word == 0x120));
    }

    #[test]
    fn test_a_narrow_panel_splits_into_two_sixteen_byte_runs() {
        let synced = vec![b'~'; 32];
        let data = vec![b' '; 32];
        let messages = flush_messages(&data, &synced, 0x80);

        let positions: Vec<u16> = messages
            .iter()
            .step_by(2)
            .map(|message| message[0])
            .collect();
        assert_eq!(positions, vec![0x80, 0x90]);
        assert_eq!(messages[1].len(), 16);
        assert_eq!(messages[3].len(), 16);
    }

    #[test]
    fn test_the_position_command_rides_the_framebuffers_base_address() {
        // The glyph buffer's base is `CMND.CGRAM` (0x40) and the second text
        // buffer's is 0xc0 (`aip31068_spi.py:102-108`).
        let mut synced = vec![0u8; 4];
        let mut data = vec![0u8; 4];
        data[2] = 0x0e;
        assert_eq!(flush_messages(&data, &synced, 0x40)[0], vec![0x42]);

        synced[2] = 0xff;
        data[2] = 0x00;
        assert_eq!(flush_messages(&data, &synced, 0xc0)[0], vec![0xc2]);
    }

    #[test]
    fn test_the_unconnected_panel_cannot_send() {
        // Without a connected MCU there is nothing to write to; the driver
        // reports it instead of panicking, so an `init` that runs before the
        // machine is up only logs.
        let printer = printer();
        let section = display_section(AIP31068_SECTION);
        let chip = Aip31068Spi::new(&ConfigWrapper::untracked(&section), &printer)
            .expect("the section loads");
        assert_eq!(chip.sent_message_count(), 0);

        let err = chip.send(&[CMND_HOME]).unwrap_err();
        assert!(err.to_string().contains("not connected"), "{err}");
        assert_eq!(chip.sent_message_count(), 0);

        // `init` swallows that error: it logs and stops instead of failing the
        // printer.
        chip.init();
        assert_eq!(chip.sent_message_count(), 0);
    }

    // -- against the fake firmware ------------------------------------------

    /// The AVR dictionary the corpus boards use (`atmega2560.dict`), when this
    /// build produced it.
    fn fake_dictionary() -> Option<std::path::PathBuf> {
        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        dict.is_file().then_some(dict)
    }

    /// The whole panel against the dictionary-driven fake firmware: the init
    /// groups arrive as `spi_send`, then the first flush writes both text
    /// framebuffers and the character generator buffer.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_the_panel_is_written_to_the_fake_firmware() {
        use crate::core::klippy::api::StartArgs;
        use crate::core::klippy::config::Config;
        use crate::core::klippy::extras::display::display::PrinterLCD;
        use crate::core::klippy::printer::PrinterState;
        use crate::core::klippy::reactor::TokioReactor;

        let Some(dict) = fake_dictionary() else {
            return;
        };
        let text = format!(
            "[mcu]\ntest: dict={}\n[display]\nlcd_type: aip31068_spi\nlatch_pin: PA1\n",
            dict.display()
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

        // The five init groups, one `spi_send` each, encoded to nine bytes
        // (`aip31068_spi.py:159-166`).
        let init: Vec<SentMessage> = DISPLAY_INIT_CMNDS
            .iter()
            .map(|command| SentMessage {
                is_data: false,
                bytes: encoded_groups(&[*command])[0].clone(),
            })
            .collect();
        assert_eq!(messages[..init.len()], init[..]);

        // And the first flush wrote the blank screen. Upstream's batching
        // (`aip31068_spi.py:146-153`) splits each 40-byte text buffer into runs
        // of 8, 16 and 16 bytes, each as a position command (`0x80` + position)
        // and its data; the second text buffer starts at 0xc0 and the character
        // generator buffer at 0x40.
        let first = &messages[init.len()..];
        assert_eq!(
            first[0],
            SentMessage {
                is_data: false,
                bytes: encoded_groups(&[0x80])[0].clone(),
            }
        );
        assert_eq!(
            first[1],
            SentMessage {
                is_data: false,
                bytes: encoded_groups(&[CMND_WRITE_RAM | 0x20; 8])[0].clone(),
            }
        );
        for fb_id in [0x80u16, 0xc0, 0x40] {
            assert!(
                first
                    .iter()
                    .any(|message| message.bytes == encoded_groups(&[fb_id])[0]),
                "the flush addresses framebuffer {fb_id:#04x}: {messages:?}"
            );
        }
    }
}
