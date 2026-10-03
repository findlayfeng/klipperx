//! `ssd1306` — the SSD1306 (128x64 OLED) panel driver (upstream defines it in
//! `klippy/extras/display/uc1701.py:199-235`, because it is the same
//! framebuffer and flush as the UC1701; `display.py:19` maps `lcd_type:
//! ssd1306` to `uc1701.SSD1306`).
//!
//! The panel picks its bus from the config, exactly as upstream does
//! (`uc1701.py:201-207`): a `cs_pin` makes it a "4 wire" SPI panel that also
//! needs a `dc_pin`, and without one it is an I2C panel at address 60 — which
//! is what the corpus's `printer-wanhao-duplicator-6-2016.cfg` is.
//!
//! | framework call | here | upstream |
//! |---|---|---|
//! | `init()` | reset, the 25-command power-up list, flush | `uc1701.py:213-235` |
//! | `clear()` | blank the eight page framebuffers | `uc1701.py:110-113` |
//! | `flush()` | batch the framebuffer differences and send them | `uc1701.py:28-52` |
//!
//! The framebuffers, the flush, the IO wrappers and the reset helper are
//! [`super::uc1701`]'s — upstream imports them from the same module.
//!
//! # What is not here
//!
//! * **`SH1106`** (`uc1701.py:238-241`): the 132-column variant adds an
//!   `x_offset` option and is a separate `lcd_type`, still refused by
//!   [`super::display`]; the `columns`/`x_offset` parameters of upstream's
//!   `SSD1306.__init__` exist only for that subclass, so this driver takes the
//!   defaults (128 columns, no offset) and reads no option for them.
//! * **Text and glyph drawing**, the reset queue stall and
//!   `BACKGROUND_PRIORITY_CLOCK`: see [`super::uc1701`]'s module docs.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::printer::{Printer, PrinterObject};

use super::display::{Glyph, LcdChip, SentMessage};
use super::uc1701::{DisplayBase, PanelIo, ResetHelper};

/// Upstream's default contrast (`uc1701.py:210`).
const DEFAULT_CONTRAST: i64 = 239;

/// Upstream's default `vcomh` (`uc1701.py:211`).
const DEFAULT_VCOMH: i64 = 0;

/// The power-up commands (`uc1701.py:215-233`), with `contrast`, `vcomh` and
/// the `invert` choice in their slots.
fn init_commands(contrast: i64, vcomh: i64, invert: bool) -> Vec<u8> {
    vec![
        0xAE, // Display off
        0xD5,
        0x80, // Set oscillator frequency
        0xA8,
        0x3f, // Set multiplex ratio
        0xD3,
        0x00, // Set display offset
        0x40, // Set display start line
        0x8D,
        0x14, // Charge pump setting
        0x20,
        0x02, // Set Memory addressing mode
        0xA1, // Set Segment re-map
        0xC8, // Set COM output scan direction
        0xDA,
        0x12, // Set COM pins hardware configuration
        0x81,
        contrast as u8, // Set contrast control
        0xD9,
        0xA1, // Set pre-charge period
        0xDB,
        vcomh as u8,                      // Set VCOMH deselect level
        0x2E,                             // Deactivate scroll
        0xA4,                             // Output ram to display
        if invert { 0xA7 } else { 0xA6 }, // Set normal/invert
        0xAF,                             // Display on
    ]
}

/// One `lcd_type: ssd1306` panel (upstream's `SSD1306`, `uc1701.py:199-235`).
pub struct Ssd1306 {
    /// The eight page framebuffers.
    base: Mutex<DisplayBase>,
    /// Where the bytes go: SPI with a `cs_pin`, I2C without one.
    io: PanelIo,
    /// The optional reset line.
    reset: ResetHelper,
    /// `contrast`, bounded `0..=255` (`uc1701.py:210`).
    contrast: i64,
    /// `vcomh`, bounded `0..=63` (`uc1701.py:211`).
    vcomh: i64,
    /// `invert` (`uc1701.py:212`).
    invert: bool,
}

impl Ssd1306 {
    /// Read the options and build the panel.
    ///
    /// Upstream's `SSD1306.__init__` (`uc1701.py:200-212`): the bus from
    /// `cs_pin`, the optional `reset_pin`, then `contrast`, `vcomh` and
    /// `invert`.
    ///
    /// # Errors
    /// As [`PanelIo::spi4wire`] (including a missing `dc_pin`) or
    /// [`PanelIo::i2c`], a `reset_pin` that is not on the bus's MCU, a
    /// `contrast` outside `0..=255`, or a `vcomh` outside `0..=63`.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let (io, mcu) = if config.get_str("cs_pin").is_some() {
            // `SPI4wire(config, "dc_pin")` (`uc1701.py:205-207`).
            PanelIo::spi4wire(config, printer, "dc_pin")?
        } else {
            // `I2C(config, 60)` (`uc1701.py:203`).
            PanelIo::i2c(config, printer)?
        };
        let reset = ResetHelper::new(config, printer, "reset_pin", &mcu)?;
        let contrast =
            config.get_int_bounded("contrast", Some(DEFAULT_CONTRAST), Some(0), Some(255))?;
        let vcomh = config.get_int_bounded("vcomh", Some(DEFAULT_VCOMH), Some(0), Some(63))?;
        let invert = config.get_bool("invert", Some(false))?;
        Ok(Self {
            base: Mutex::new(DisplayBase::new(128)),
            io,
            reset,
            contrast,
            vcomh,
            invert,
        })
    }

    /// The framebuffer.
    fn base(&self) -> std::sync::MutexGuard<'_, DisplayBase> {
        self.base
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }
}

impl LcdChip for Ssd1306 {
    /// Upstream's `SSD1306.init` (`uc1701.py:213-235`): the reset toggle, the
    /// power-up list, then the first flush.
    fn init(&self) {
        self.reset.init();
        let commands = init_commands(self.contrast, self.vcomh, self.invert);
        if let Err(err) = self.io.send(&commands, false) {
            warn!("ssd1306: could not initialise the panel: {err}");
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
            warn!("ssd1306: could not flush the panel: {err}");
        }
    }

    /// The panel's size in characters (`uc1701.py:114-115`).
    fn get_dimensions(&self) -> (usize, usize) {
        (16, 4)
    }

    /// Nothing draws here, so the icons upstream caches for `write_glyph` have
    /// nowhere to go (see [`super::uc1701`]).
    fn set_glyphs(&self, _glyphs: &BTreeMap<String, Glyph>) {}

    /// Every message this panel has handed to the firmware, in order.
    fn sent_messages(&self) -> Vec<SentMessage> {
        self.io.sent()
    }
}

impl PrinterObject for Ssd1306 {
    /// Upstream's `SSD1306` has no `get_status`, so it is not client-visible.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for Ssd1306 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Ssd1306")
            .field("contrast", &self.contrast)
            .field("vcomh", &self.vcomh)
            .field("invert", &self.invert)
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
    use crate::core::klippy::mcu::McuObject;
    use crate::core::klippy::pins::{PrinterPins, PINS_OBJECT};
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

    /// A printer with `pins` and one MCU, as the loader builds them.
    fn printer() -> Arc<Printer> {
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
        printer
    }

    fn wrap(section: &ConfigSection) -> ConfigWrapper<'_> {
        ConfigWrapper::untracked(section)
    }

    // -- the two buses -----------------------------------------------------

    #[test]
    fn test_without_a_chip_select_the_panel_is_an_i2c_one() {
        // The corpus's shape: `printer-wanhao-duplicator-6-2016.cfg`.
        let printer = printer();
        let section = display_section(&[
            ("lcd_type", "ssd1306"),
            ("reset_pin", "PE3"),
            ("encoder_pins", "^PG1, ^PG0"),
            ("click_pin", "^!PD2"),
        ]);

        let chip = Ssd1306::new(&wrap(&section), &printer).expect("the panel loads");

        assert!(chip.io.is_i2c());
        assert_eq!(chip.contrast, DEFAULT_CONTRAST);
        assert_eq!(chip.vcomh, DEFAULT_VCOMH);
        assert!(!chip.invert);
        assert!(chip.reset.out.is_some());
        assert_eq!(chip.get_dimensions(), (16, 4));
        assert_eq!(chip.sent_message_count(), 0);
    }

    #[test]
    fn test_a_chip_select_makes_the_panel_spi_and_demands_a_dc_pin() {
        let printer = printer();

        let section = display_section(&[("cs_pin", "PA3"), ("dc_pin", "PA5")]);
        let chip = Ssd1306::new(&wrap(&section), &printer).expect("the SPI panel loads");
        assert!(!chip.io.is_i2c());

        // Upstream's `SPI4wire(config, "dc_pin")` reads the option with no
        // default (`uc1701.py:121`).
        let section = display_section(&[("cs_pin", "PA3")]);
        let err = Ssd1306::new(&wrap(&section), &printer)
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'dc_pin' in section 'display' must be specified"
        );
    }

    #[test]
    fn test_an_i2c_address_is_checked_the_way_upstream_checks_it() {
        // `MCU_I2C_from_config(config, default_addr=60, …, minval=0, maxval=127)`
        // (`uc1701.py:131-133`, `bus.py:303-308`).
        let printer = printer();
        let section = display_section(&[("i2c_address", "128")]);

        let err = Ssd1306::new(&wrap(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'i2c_address' in section 'display' must be between 0 and 127"
        );
    }

    #[test]
    fn test_one_software_i2c_pin_without_the_other_is_refused() {
        let printer = printer();
        let section = display_section(&[("i2c_software_scl_pin", "PA0")]);

        let err = Ssd1306::new(&wrap(&section), &printer)
            .map(|_| ())
            .unwrap_err();

        assert!(
            err.to_string()
                .contains("both 'i2c_software_scl_pin' and 'i2c_software_sda_pin' must be set"),
            "{err}"
        );
    }

    // -- the options -------------------------------------------------------

    #[test]
    fn test_contrast_and_vcomh_keep_upstreams_bounds() {
        let printer = printer();
        let mut options = vec![
            ("lcd_type", "ssd1306"),
            ("contrast", "128"),
            ("vcomh", "30"),
            ("invert", "True"),
        ];
        let section = display_section(&options);
        let chip = Ssd1306::new(&wrap(&section), &printer).unwrap();
        assert_eq!(chip.contrast, 128);
        assert_eq!(chip.vcomh, 30);
        assert!(chip.invert);

        options[1] = ("contrast", "256");
        let section = display_section(&options);
        let err = Ssd1306::new(&wrap(&section), &printer)
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'contrast' in section 'display' must have maximum of 255"
        );

        options[1] = ("contrast", "128");
        options[2] = ("vcomh", "64");
        let section = display_section(&options);
        let err = Ssd1306::new(&wrap(&section), &printer)
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'vcomh' in section 'display' must have maximum of 63"
        );
    }

    // -- the power-up sequence --------------------------------------------

    #[test]
    fn test_the_init_sequence_is_upstreams_power_up_list() {
        let commands = init_commands(239, 0, false);
        assert_eq!(
            commands,
            vec![
                0xAE, 0xD5, 0x80, 0xA8, 0x3f, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC8,
                0xDA, 0x12, 0x81, 239, 0xD9, 0xA1, 0xDB, 0x00, 0x2E, 0xA4, 0xA6, 0xAF,
            ]
        );
        // The three options land in their own slots, and `invert` picks the
        // inverse command (`uc1701.py:226-232`).
        assert_eq!(init_commands(42, 63, false)[17], 42);
        assert_eq!(init_commands(42, 63, false)[21], 63);
        assert_eq!(*init_commands(42, 63, true).last().unwrap(), 0xAF);
        assert_eq!(init_commands(42, 63, true)[24], 0xA7);
        assert_eq!(init_commands(42, 63, false)[24], 0xA6);
    }

    // -- against the fake firmware -----------------------------------------

    /// The AVR dictionary, when this build produced it.
    fn fake_dictionary() -> Option<std::path::PathBuf> {
        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        dict.is_file().then_some(dict)
    }

    /// The I2C panel against the dictionary-driven fake firmware: the config
    /// carries `config_i2c`/`i2c_set_bus`, and `init` queues the power-up list
    /// as an `i2c_transfer` write.
    ///
    /// The write is fire-and-forget (see [`PanelIo::send`]), so this test says
    /// the panel reaches the firmware, not that the firmware answered: the
    /// status reply it sends back is logged as unhandled.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_the_panel_is_written_to_against_the_fake_firmware() {
        use crate::core::klippy::api::StartArgs;
        use crate::core::klippy::config::Config;
        use crate::core::klippy::printer::PrinterState;
        use crate::core::klippy::reactor::TokioReactor;

        let Some(dict) = fake_dictionary() else {
            return;
        };
        let text = format!(
            "[mcu]\ntest: dict={}\n[display]\nlcd_type: ssd1306\nreset_pin: PC1\n",
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
                bytes: init_commands(DEFAULT_CONTRAST, DEFAULT_VCOMH, false),
            }
        );
        assert!(
            messages.iter().any(|message| message.is_data),
            "the flush sends its framebuffer contents: {messages:?}"
        );
    }
}
