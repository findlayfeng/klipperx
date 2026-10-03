//! `hd44780_spi` — the HD44780 (20x4 text) panel behind a shift-register
//! backpack (upstream `klippy/extras/display/hd44780_spi.py`).
//!
//! A `[display]` section with `lcd_type: hd44780_spi` builds one of these. The
//! panel is an ordinary HD44780 whose pins hang off a 74HC595 shift register
//! instead of the MCU: `latch_pin` is the register's latch clock, and upstream
//! hands it to `MCU_SPI_from_config` as the chip-select option
//! (`hd44780_spi.py:24-25`), so the firmware pulses it around every byte. The
//! bits themselves go out over the same SPI bus the three `spi_software_*` pins
//! (or `spi_bus`) configure.
//!
//! Every protocol byte therefore leaves as two nibbles, each as three
//! `spi_send`s — data with the enable line low, then the enable pulse, then low
//! again (`hd44780_spi.py:48-55`):
//!
//! | bit | meaning |
//! |---|---|
//! | 4..7 | the nibble (the backpack's D4..D7) |
//! | 1 | `data_mask`, the register-select line (high for data, low for commands) |
//! | 3 | `enable_mask`, the HD44780's E line |
//!
//! | framework call | here | upstream |
//! |---|---|---|
//! | `init()` | the power-up command groups, then a flush | `hd44780_spi.py:81-94` |
//! | `clear()` | blank both text framebuffers | `hd44780_spi.py:120-123` |
//! | `flush()` | batch the framebuffer differences and send them | `hd44780_spi.py:60-80` |
//! | `set_glyphs()` | keep the 5x8 icons for the glyphs the layout names | `hd44780_spi.py:100-104` |
//!
//! The framebuffers, the flush algorithm and the power-up groups are the plain
//! HD44780's ([`super::hd44780`]): two text buffers of `2*line_length` bytes,
//! because each of the panel's four rows is one half of one buffer, and the
//! 64-byte character generator buffer. As upstream, every "already sent" copy
//! starts as `~`, so the first flush writes the whole screen. `line_length` is
//! 16 or 20 (`hd44780_spi.py:11-12,33-34`) and decides the panel's width, and
//! with it the default `display_group`.
//!
//! # What is not here
//!
//! * **Text and glyph drawing.** [`super::display`] never draws, so
//!   `write_text`/`write_glyph`/`write_graphics` (`hd44780_spi.py:95-119`) —
//!   and with them the `TextGlyphs` table that maps `right_arrow` to `0x7e`
//!   (`hd44780_spi.py:14`) — are not ported; [`Hd44780Spi::set_glyphs`] keeps
//!   the 5x8 icons because upstream collects them there
//!   (`hd44780_spi.py:100-104`).
//! * **`minclock`** (`hd44780_spi.py:82-83,92`): upstream orders the init groups
//!   by a clock deadline 100ms apart, computed from the MCU's estimated print
//!   time. This host's [`McuCommand`](crate::core::klippy::cmd::McuCommand)
//!   carries no clock, so the groups go out in call order.

use std::collections::{BTreeMap, HashMap};
use std::sync::{Arc, Mutex, MutexGuard};

use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::spi_device::mcu_spi_from_config;
use crate::core::klippy::mcu::{McuError, McuSpi};
use crate::core::klippy::printer::{Printer, PrinterObject};

use super::display::{Glyph, LcdChip, SentMessage};

/// Upstream's `MCU_SPI_from_config(config, 0x00, pin_option="latch_pin")` uses
/// the default clock (`hd44780_spi.py:24-25`, `bus.py:124`), 100 kHz.
const DEFAULT_SPI_SPEED: u32 = 100_000;
/// The SPI mode upstream passes as the second argument
/// (`hd44780_spi.py:25`): mode 0.
const SPI_MODE: u8 = 0;

/// The register-select bit in the shifted byte (`hd44780_spi.py:28`): high for
/// data, low for commands.
const DATA_MASK: u8 = 1 << 1;
/// The command mask (`hd44780_spi.py:29`): commands leave the RS bit low.
const COMMAND_MASK: u8 = 0;
/// The HD44780 enable line (`hd44780_spi.py:30`): the nibble is latched on the
/// falling edge of the middle byte.
const ENABLE_MASK: u8 = 1 << 3;

/// The panel's width when `line_length` is not written (`hd44780_spi.py:11`).
const LINE_LENGTH_DEFAULT: &str = "20";
/// The widths `line_length` may choose between (`hd44780_spi.py:12`).
const LINE_LENGTH_OPTIONS: &[&str] = &["16", "20"];

/// One framebuffer and the copy the firmware has already been told about —
/// upstream's `(new_data, old_data, fb_id)` triples (`hd44780_spi.py:40-47`).
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
/// upstream flushes them (`hd44780_spi.py:40-47`).
#[derive(Debug)]
struct Framebuffers {
    /// One buffer per pair of panel rows (`hd44780_spi.py:37-38`).
    text: [Framebuffer; 2],
    /// The 64-byte CGRAM buffer (`hd44780_spi.py:39`).
    glyph: Framebuffer,
}

impl Framebuffers {
    /// Upstream's `__init__` buffers: the text screens are spaces, the glyph
    /// buffer is zeros, and every "already sent" copy is `~` so the first flush
    /// sends the lot (`hd44780_spi.py:37-47`).
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

/// One `lcd_type: hd44780_spi` panel.
pub struct Hd44780Spi {
    /// The SPI bus the shift register hangs on; its "chip select" is
    /// `latch_pin` (upstream's `self.spi`, `hd44780_spi.py:24-25`).
    spi: Arc<McuSpi>,
    /// The panel's width in characters (`hd44780_spi.py:33-34`).
    line_length: usize,
    /// Whether the power-up sequence programs the 4-bit protocol
    /// (`hd44780_spi.py:21-22`).
    hd44780_protocol_init: bool,
    /// The framebuffers.
    framebuffers: Mutex<Framebuffers>,
    /// The 5x8 icons by glyph name (`hd44780_spi.py:32,100-104`).
    icons: Mutex<HashMap<String, (u8, Vec<u8>)>>,
    /// Every message handed to the firmware, in order (tests and diagnostics).
    sent: Mutex<Vec<SentMessage>>,
}

impl Hd44780Spi {
    /// Read the two options and the SPI bus, and build the framebuffers.
    ///
    /// Upstream's `hd44780_spi.__init__` (`hd44780_spi.py:19-47`):
    /// `hd44780_protocol_init` first, then the SPI bus off `latch_pin`, then
    /// `line_length`.
    ///
    /// # Errors
    /// A missing `latch_pin` (the `MCU_SPI_from_config` chip-select option), a
    /// missing or unresolvable software-SPI pin, an unknown `spi_mcu`, a
    /// `line_length` outside [`LINE_LENGTH_OPTIONS`], or whatever the SPI
    /// options refuse.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let hd44780_protocol_init = config.get_bool("hd44780_protocol_init", Some(true))?;
        // The latch pin is required: `MCU_SPI_from_config` reports a missing
        // chip select instead of a "no chip select" device (`bus.py:129-134`).
        config.get("latch_pin", None)?;
        let setup = mcu_spi_from_config(
            config,
            printer.as_ref(),
            SPI_MODE,
            "latch_pin",
            DEFAULT_SPI_SPEED,
        )?;
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
            hd44780_protocol_init,
            framebuffers: Mutex::new(Framebuffers::new(line_length)),
            icons: Mutex::new(HashMap::new()),
            sent: Mutex::new(Vec::new()),
        })
    }

    /// Send one nibble: the three shifted bytes of
    /// `hd44780_spi.send_4_bits` (`hd44780_spi.py:48-55`).
    ///
    /// # Errors
    /// The first `spi_send` that fails (an unconnected MCU, a firmware without
    /// `spi_send`, or a full send buffer) stops the nibble with that error.
    fn send_4_bits(&self, nibble: u8, is_data: bool) -> Result<(), McuError> {
        for byte in nibble_bytes(nibble, is_data) {
            self.spi.send(&[byte])?;
            self.sent
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .push(SentMessage {
                    is_data,
                    bytes: vec![byte],
                });
        }
        Ok(())
    }

    /// Send whole protocol bytes, each as its high nibble then its low one
    /// (`hd44780_spi.send`, `hd44780_spi.py:56-59`).
    ///
    /// Upstream's `data<<4` keeps only the high nibble in the end because
    /// `send_4_bits` masks with `0xF0`: the low nibble moves to bits 4..7.
    ///
    /// # Errors
    /// As [`Hd44780Spi::send_4_bits`].
    fn send(&self, cmds: &[u8], is_data: bool) -> Result<(), McuError> {
        for &data in cmds {
            self.send_4_bits(data & 0xf0, is_data)?;
            self.send_4_bits((data & 0x0f) << 4, is_data)?;
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

/// The three bytes one nibble becomes (`hd44780_spi.py:48-55`): the nibble in
/// bits 4..7, the register-select bit, and the enable pulse (`data`, `data|E`,
/// `data`).
///
/// Upstream masks with `cmd & 0xF0`, so a caller passes the nibble already
/// positioned there.
fn nibble_bytes(nibble: u8, is_data: bool) -> [u8; 3] {
    let mask = if is_data { DATA_MASK } else { COMMAND_MASK };
    [nibble | mask, nibble | mask | ENABLE_MASK, nibble | mask]
}

impl LcdChip for Hd44780Spi {
    /// Upstream's `hd44780_spi.init` (`hd44780_spi.py:81-94`): the power-up
    /// command groups, then the first flush.
    fn init(&self) {
        for cmds in init_commands(self.hd44780_protocol_init) {
            if let Err(err) = self.send(&cmds, false) {
                warn!("hd44780_spi: could not initialise the panel: {err}");
                return;
            }
        }
        self.flush();
    }

    /// Upstream's `hd44780_spi.clear` (`hd44780_spi.py:120-123`): only the text
    /// framebuffers are blanked; the glyph buffer keeps the icons.
    fn clear(&self) {
        let mut framebuffers = self.framebuffers();
        for text in &mut framebuffers.text {
            text.data.fill(b' ');
        }
    }

    /// Upstream's `hd44780_spi.flush` (`hd44780_spi.py:60-80`): send the
    /// changed bytes of every framebuffer, batching changes that are close
    /// together.
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
            // (`hd44780_spi.py:68-74`).
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
                    warn!("hd44780_spi: could not flush the panel: {err}");
                    return;
                }
                if let Err(err) = self.send(&data[pos..pos + count], true) {
                    warn!("hd44780_spi: could not flush the panel: {err}");
                    return;
                }
            }
            synced.copy_from_slice(&data);
        }
    }

    /// The panel's size in characters (`hd44780_spi.py:124-125`).
    fn get_dimensions(&self) -> (usize, usize) {
        (self.line_length, 4)
    }

    /// Upstream's `hd44780_spi.set_glyphs` (`hd44780_spi.py:100-104`): keep
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
    fn sent_messages(&self) -> Vec<SentMessage> {
        self.sent
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }
}

impl PrinterObject for Hd44780Spi {
    /// Upstream's `hd44780_spi` has no `get_status`, so it is not
    /// client-visible.
    fn get_status(&self, _eventtime: f64) -> serde_json::Value {
        serde_json::json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for Hd44780Spi {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Hd44780Spi")
            .field("line_length", &self.line_length)
            .finish_non_exhaustive()
    }
}

/// The power-up command groups, one message each (`hd44780_spi.py:85-90`).
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
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::section::ConfigSection;
    use crate::core::klippy::config::value::ConfigValue;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::mcu::McuObject;
    use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
    use crate::core::klippy::printer::Printer;
    use crate::core::klippy::reactor::ManualReactor;

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

    /// The corpus's software-SPI backpack wiring
    /// (`generic-mightyboard.cfg`, `printer-flashforge-creator-pro-2018.cfg`):
    /// three software SPI pins and the shift register's latch line.
    const SPI_SECTION: &[(&str, &str)] = &[
        ("spi_software_mosi_pin", "PC3"),
        ("spi_software_sclk_pin", "PC2"),
        ("spi_software_miso_pin", "PJ1"),
        ("latch_pin", "PC4"),
    ];

    /// A printer with `pins` and two MCUs, as the loader builds them before any
    /// section runs: `mcu`, and `board2` for the "must be on the named MCU"
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

    // -- the shift-register bytes ------------------------------------------

    #[test]
    fn test_the_nibble_leaves_as_upstreams_three_bytes() {
        // A command: RS low, the nibble in bits 4..7, and the enable pulse in
        // between (`hd44780_spi.py:48-55`).
        assert_eq!(nibble_bytes(0x30, false), [0x30, 0x38, 0x30]);
        assert_eq!(nibble_bytes(0x00, false), [0x00, 0x08, 0x00]);
        assert_eq!(nibble_bytes(0x80, false), [0x80, 0x88, 0x80]);
        // Data: bit 1 (the register-select line) is set.
        assert_eq!(nibble_bytes(0x20, true), [0x22, 0x2a, 0x22]);
        assert_eq!(nibble_bytes(0x00, true), [0x02, 0x0a, 0x02]);
    }

    // -- options ------------------------------------------------------------

    #[test]
    fn test_the_options_are_read_with_upstream_defaults() {
        // `hd44780_protocol_init` defaults to True and `line_length` to 20
        // (`hd44780_spi.py:11,21-22,33-34`), and both are recorded as read —
        // `check_unused` requires a reader for every option a config writes.
        let printer = printer();
        let section = display_section(SPI_SECTION);
        let access = AccessTracking::shared();
        let chip = Hd44780Spi::new(&ConfigWrapper::new(&section, Arc::clone(&access)), &printer)
            .expect("the section loads");

        assert!(chip.hd44780_protocol_init);
        assert_eq!(chip.line_length, 20);
        assert_eq!(chip.get_dimensions(), (20, 4));
        for option in [
            "latch_pin",
            "spi_software_mosi_pin",
            "spi_software_sclk_pin",
            "spi_software_miso_pin",
            "hd44780_protocol_init",
            "line_length",
        ] {
            assert!(access.contains("display", option), "option {option}");
        }
    }

    #[test]
    fn test_line_length_and_protocol_init_change_the_panel() {
        let printer = printer();
        let mut options = SPI_SECTION.to_vec();
        options.push(("line_length", "16"));
        options.push(("hd44780_protocol_init", "False"));
        let section = display_section(&options);

        let chip = Hd44780Spi::new(&ConfigWrapper::untracked(&section), &printer)
            .expect("the section loads");

        assert!(!chip.hd44780_protocol_init);
        assert_eq!(chip.line_length, 16);
        assert_eq!(chip.get_dimensions(), (16, 4));
        // The framebuffers follow the width: two half-lines of 16 characters
        // (`hd44780_spi.py:37-38`).
        assert_eq!(chip.framebuffers().text[0].data.len(), 32);
    }

    #[test]
    fn test_an_unknown_line_length_is_a_choice_error() {
        let printer = printer();
        let mut options = SPI_SECTION.to_vec();
        options.push(("line_length", "18"));
        let section = display_section(&options);

        let err = Hd44780Spi::new(&ConfigWrapper::untracked(&section), &printer)
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
        let section = display_section(&[("spi_software_mosi_pin", "PC3")]);

        let err = Hd44780Spi::new(&ConfigWrapper::untracked(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'latch_pin' in section 'display' must be specified"
        );
    }

    #[test]
    fn test_the_latch_pin_is_read_before_line_length() {
        // Upstream reads the bus (`latch_pin` and friends) before `line_length`
        // (`hd44780_spi.py:24-34`), so the missing pin is the error the user
        // sees, not the bad width.
        let printer = printer();
        let section = display_section(&[("line_length", "18")]);

        let err = Hd44780Spi::new(&ConfigWrapper::untracked(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'latch_pin' in section 'display' must be specified"
        );
    }

    #[test]
    fn test_a_partly_written_software_bus_is_refused() {
        // The SPI bus has to be either hardware or all three software pins
        // (`spi_device::mcu_spi_from_config`, upstream `bus.py:138-151`).
        let printer = printer();
        let section = display_section(&[("latch_pin", "PC4"), ("spi_software_sclk_pin", "PC2")]);

        let err = Hd44780Spi::new(&ConfigWrapper::untracked(&section), &printer)
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
    fn test_the_software_pins_must_be_on_the_named_mcu() {
        let printer = printer();
        let mut options = SPI_SECTION.to_vec();
        options[2] = ("spi_software_miso_pin", "board2:PJ1");
        options.push(("spi_mcu", "mcu"));
        let section = display_section(&options);

        let err = Hd44780Spi::new(&ConfigWrapper::untracked(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Section 'display': spi_software_miso_pin must be on mcu 'mcu'"
        );
    }

    #[test]
    fn test_an_unknown_spi_mcu_names_the_section() {
        let printer = printer();
        let mut options = SPI_SECTION.to_vec();
        options.push(("spi_mcu", "zboard"));
        let section = display_section(&options);

        let err = Hd44780Spi::new(&ConfigWrapper::untracked(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(err.to_string(), "Section 'display': unknown MCU 'zboard'");
    }

    // -- the panel ----------------------------------------------------------

    #[test]
    fn test_the_init_sequence_matches_upstream() {
        // `hd44780_spi.py:85-90`: the 4-bit protocol handshake, the home, and
        // the shared "positive direction, display on, cursor hidden" group.
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
        // screen (`hd44780_spi.py:37-47`).
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
        let section = display_section(SPI_SECTION);
        let chip = Hd44780Spi::new(&ConfigWrapper::untracked(&section), &printer)
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
        let section = display_section(SPI_SECTION);
        let chip = Hd44780Spi::new(&ConfigWrapper::untracked(&section), &printer)
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

        // Only the HD44780-sized glyphs are kept (`hd44780_spi.py:100-104`).
        assert_eq!(
            *chip.icons.lock().unwrap(),
            HashMap::from([("thermometer".to_string(), (3u8, vec![0x0e; 8]))])
        );
    }

    #[test]
    fn test_the_unconnected_panel_cannot_send() {
        // Without a connected MCU there is nothing to shift into the register;
        // the driver reports it instead of panicking, so an `init` that runs
        // before the machine is up only logs.
        let printer = printer();
        let section = display_section(SPI_SECTION);
        let chip = Hd44780Spi::new(&ConfigWrapper::untracked(&section), &printer)
            .expect("the section loads");
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

    /// An AVR dictionary with the SPI commands, when this build produced it.
    fn fake_dictionary() -> Option<std::path::PathBuf> {
        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        dict.is_file().then_some(dict)
    }

    /// The 48 shift-register bytes the default init sequence shifts out, one
    /// `spi_send` each: `[0x33], [0x33], [0x32], [0x28, 0x28, 0x02],
    /// [0x06, 0x0c]`, each byte as its high nibble then its low one with the
    /// enable pulse between (`hd44780_spi.py:48-59,85-90`).
    const INIT_BYTES: [u8; 48] = [
        0x30, 0x38, 0x30, 0x30, 0x38, 0x30, // 0x33
        0x30, 0x38, 0x30, 0x30, 0x38, 0x30, // 0x33
        0x30, 0x38, 0x30, 0x20, 0x28, 0x20, // 0x32
        0x20, 0x28, 0x20, 0x80, 0x88, 0x80, // 0x28
        0x20, 0x28, 0x20, 0x80, 0x88, 0x80, // 0x28
        0x00, 0x08, 0x00, 0x20, 0x28, 0x20, // 0x02
        0x00, 0x08, 0x00, 0x60, 0x68, 0x60, // 0x06
        0x00, 0x08, 0x00, 0xc0, 0xc8, 0xc0, // 0x0c
    ];

    /// A `SentMessage` for one shifted byte.
    fn shifted(byte: u8, is_data: bool) -> SentMessage {
        SentMessage {
            is_data,
            bytes: vec![byte],
        }
    }

    /// The whole panel against the dictionary-driven fake firmware: the init
    /// groups arrive as `spi_send`s, then the first flush addresses the text
    /// framebuffer and shifts the blank screen out as data.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_the_panel_is_written_to_the_fake_firmware() {
        use crate::core::klippy::api::StartArgs;
        use crate::core::klippy::extras::display::display::PrinterLCD;
        use crate::core::klippy::printer::PrinterState;
        use crate::core::klippy::reactor::TokioReactor;

        let Some(dict) = fake_dictionary() else {
            return;
        };
        let text = format!(
            "[mcu]\ntest: dict={}\n[display]\nlcd_type: hd44780_spi\n{}",
            dict.display(),
            SPI_SECTION
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

        // The init sequence, one `spi_send` per shifted byte
        // (`hd44780_spi.py:48-59,85-90`).
        let init: Vec<SentMessage> = INIT_BYTES
            .iter()
            .map(|byte| shifted(*byte, false))
            .collect();
        assert_eq!(messages[..init.len()], init[..]);

        // And the first flush addressed the first text framebuffer (0x80) and
        // took the panel out of data mode for the position command.
        let flush = &messages[init.len()..];
        for (index, byte) in [0x80, 0x88, 0x80, 0x00, 0x08, 0x00].into_iter().enumerate() {
            assert_eq!(flush[index], shifted(byte, false), "flush byte {index}");
        }
        // Then the blank screen as data: a space is 0x20, so the register-select
        // bit (0x02) rides along on every byte.
        for (index, byte) in [0x22, 0x2a, 0x22, 0x02, 0x0a, 0x02].into_iter().enumerate() {
            assert_eq!(
                flush[6 + index],
                shifted(byte, true),
                "first data byte, shifted byte {index}"
            );
        }
    }
}
