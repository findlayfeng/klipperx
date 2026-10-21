//! `[display]` — the LCD framework: which panel, which layout, and the timer
//! that keeps the panel written (upstream `klippy/extras/display/display.py`).
//!
//! One `[display]` section builds one [`PrinterLCD`]:
//!
//! | step | here | upstream |
//! |---|---|---|
//! | the panel | `lcd_type` picks a [`LcdChip`] | `display.py:16-22`, `:181` |
//! | `[display_status]` | created on demand, whether or not the config names it | `display.py:188` |
//! | the menu | [`MenuManager`](super::menu::MenuManager), on the primary display only | `display.py:180-190` |
//! | the menu's frame | while it is open, each of its rows is painted as one line from column 0 | `menu.py:601-660`, `:842-853` |
//! | the menu keys | registered as buttons by the manager | `menu_keys.py:12-43` |
//! | templates | the shipped `display.cfg` is merged into the `display_template` registry | `display.py:115-166` |
//! | layouts | `display_data` groups | `display.py:56-87` |
//! | glyphs | `display_glyph` sections | `display.py:103-114` |
//! | the group | `display_group`, defaulted from the panel's width | `display.py:194-200` |
//! | refresh | `klippy:ready` initialises the panel and starts the timer | `display.py:217-241` |
//! | drawing | each `display_data` item is rendered and drawn, `~glyph~` runs included | `display.py:78-87`, `:247-264` |
//! | g-code | `SET_DISPLAY_GROUP` | `display.py:207-214`, `:266-271` |
//!
//! # The shipped layout
//!
//! Upstream reads `display.cfg` from beside the module and merges it with the
//! main config's own `[display_template]`/`[display_data]`/`[display_glyph]`
//! sections, where the main config's sections win by name
//! (`display.py:115-166`). The file is vendored here and included in the binary
//! ([`DISPLAY_CFG`]), because a shipped binary has no `klippy/extras` directory
//! to read from; `test_the_vendored_layout_has_not_drifted` keeps the copy
//! byte-for-byte equal to the submodule's.
//!
//! Merging keeps upstream's rule (`display.py:126-128`): a section whose
//! identifier the main config already defines is not loaded from the shipped
//! file, so the config's own item replaces the shipped one instead of joining
//! it. A config's own `[display_data]`/`[display_glyph]` sections have
//! factories of their own ([`load_display_data`], [`load_display_glyph`]): the
//! factory claims the section and parses it, and `[display]` assembles the
//! groups and glyphs from every section plus this file.
//!
//! # What is not here
//!
//! * **The other panels' drawing.** [`LcdChip::write_text`],
//!   [`LcdChip::write_glyph`] and [`LcdChip::write_graphics`] have a
//!   draw-nothing default, and only `hd44780` implements them: `st7920`,
//!   `uc1701`, `ssd1306`, `hd44780_spi` and `aip31068_spi` need their own font
//!   tables (`font8x14.VGA_FONT`, `st7920.CharGlyphs`, `uc1701.CharGlyphs`),
//!   which are a later unit. Those panels therefore render into nothing.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::section::ConfigSection;
use crate::core::klippy::config::{Config, ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::display_status::ensure as ensure_display_status;
use crate::core::klippy::extras::led::{
    load_display_template, DisplayTemplates, DISPLAY_TEMPLATES_OBJECT,
};
use crate::core::klippy::extras::template::{
    Builtin, Context, HostCallable, PrinterView, Rt, Template, TemplateError,
};
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};
use crate::core::klippy::reactor::{Reactor, TimerHandle};

use super::aip31068_spi::Aip31068Spi;
use super::hd44780::Hd44780;
use super::hd44780_spi::Hd44780Spi;
use super::menu::MenuManager;
use super::ssd1306::Ssd1306;
use super::st7920::ST7920;
use super::uc1701::Uc1701;

// The framework reads `[display_status]` (created on demand) and the
// `[display_template]` registry sits at order 40; `[spi_device]`'s resources
// come at 50 and a panel driver only needs plain pins, so 45 puts `[display]`
// after everything it reads and before the bus consumers.
section!("display", order = 45, load = load_config);

// The layout sections a config may write for itself. Upstream reads them out of
// the config without a factory of their own (`display.py:132-166`); here the
// factory table is the loader's schema, so each needs one to be a section this
// host knows. They sit with `[display]` (45): a main section is loaded before
// every prefix section in its phase, so `[display]` has already read these
// sections by the time their factories run.
section!("display_data", order = 45, prefix = load_display_data);
section!("display_glyph", order = 45, prefix = load_display_glyph);

/// Normal time between each screen redraw (`display.py:12`).
const REDRAW_TIME: f64 = 0.500;
/// Minimum time between screen redraws (`display.py:14`).
const REDRAW_MIN_TIME: f64 = 0.100;

/// What builds one value of `lcd_type`: a panel driver's constructor, behind
/// the framework's trait object.
type PanelConstructor = fn(&ConfigWrapper, &Arc<Printer>) -> Result<Arc<dyn LcdChip>, ConfigError>;

/// The panels `lcd_type` accepts, each with the constructor that builds it —
/// upstream's `LCD_chips` table (`display.py:16-22`).
///
/// One list rather than a name list plus a match: every accepted name has a
/// driver here, so a name cannot be accepted without a panel behind it (the
/// choice error is still upstream's wording, from
/// [`ConfigWrapper::get_choice`]).
const LCD_CHIPS: &[(&str, PanelConstructor)] = &[
    ("st7920", |config, printer| {
        Ok(Arc::new(ST7920::new(config, printer)?))
    }),
    ("emulated_st7920", |config, printer| {
        Ok(Arc::new(ST7920::new_emulated(config, printer)?))
    }),
    ("hd44780", |config, printer| {
        Ok(Arc::new(Hd44780::new(config, printer)?))
    }),
    ("uc1701", |config, printer| {
        Ok(Arc::new(Uc1701::new(config, printer)?))
    }),
    ("ssd1306", |config, printer| {
        Ok(Arc::new(Ssd1306::new(config, printer)?))
    }),
    ("sh1106", |config, printer| {
        Ok(Arc::new(Ssd1306::new_sh1106(config, printer)?))
    }),
    ("hd44780_spi", |config, printer| {
        Ok(Arc::new(Hd44780Spi::new(config, printer)?))
    }),
    ("aip31068_spi", |config, printer| {
        Ok(Arc::new(Aip31068Spi::new(config, printer)?))
    }),
];

/// The names [`LCD_CHIPS`] offers, in the order upstream's table lists them:
/// what `lcd_type` is validated against.
fn lcd_type_names() -> Vec<&'static str> {
    LCD_CHIPS.iter().map(|(name, _)| *name).collect()
}

/// The shipped layout (`klippy/extras/display/display.cfg`).
///
/// Vendored verbatim from the upstream submodule; the drift guard test keeps it
/// that way.
const DISPLAY_CFG: &str = include_str!("display.cfg");

/// The name the shipped layout is reported under. There is no path at run time,
/// so errors name the file the way a reader knows it.
const DISPLAY_CFG_NAME: &str = "display.cfg";

/// The default group for a 16-character panel (`display.py:194`).
const DEFAULT_GROUP_16X4: &str = "_default_16x4";
/// The default group for a 20-character panel (`display.py:196`).
const DEFAULT_GROUP_20X4: &str = "_default_20x4";

// ===========================================================================
// The panel interface
// ===========================================================================

/// The panel behind a `[display]` section — one value of `lcd_type`
/// (upstream's `LCD_chips`, `display.py:16-22`).
///
/// The framework's half of the contract is small: bring the panel up, blank it
/// and flush it on every redraw, say how wide it is (the default `display_group`
/// depends on that), take the glyphs it may draw, and accept the three drawing
/// calls a rendered layout makes. Panels that have no font table here keep the
/// draw-nothing default of the three `write_*` methods (module docs).
pub trait LcdChip: Send + Sync {
    /// Bring the panel up (`display.py:218`).
    fn init(&self);
    /// Blank the framebuffer (`display.py:226`).
    fn clear(&self);
    /// Send the framebuffer differences (`display.py:238`).
    fn flush(&self);
    /// The panel's size in characters (`display.py:195`, `:215`).
    fn get_dimensions(&self) -> (usize, usize);
    /// Take the glyphs the layout may name (`display.py:193`).
    fn set_glyphs(&self, glyphs: &BTreeMap<String, Glyph>);
    /// Write text into the panel's character cells, starting at column `x`
    /// (`display.py:251`).
    fn write_text(&self, _x: usize, _row: usize, _text: &[u8]) {}
    /// Draw the glyph named `name` at (`x`, `row`), returning how many character
    /// cells it took (`display.py:254`).
    fn write_glyph(&self, _x: usize, _row: usize, _name: &str) -> usize {
        0
    }
    /// Write one 16-byte graphics column at (`x`, `row`) — a pixel column,
    /// which only a panel with a graphics mode has (`display.py:263`).
    fn write_graphics(&self, _x: usize, _row: usize, _data: &[u8]) {}
    /// Every message this panel has handed to the firmware, in order (tests and
    /// diagnostics): a panel that loaded but never wrote to the firmware is a
    /// blank screen, not a working one.
    fn sent_messages(&self) -> Vec<SentMessage>;
    /// How many messages this panel has handed to the firmware.
    fn sent_message_count(&self) -> usize {
        self.sent_messages().len()
    }
}

/// One message a panel handed to the firmware, with the extension-mode switch
/// already applied (`st7920.py:174-184`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SentMessage {
    /// Whether it went as `st7920_send_data` (the other is
    /// `st7920_send_cmds`).
    pub is_data: bool,
    /// The bytes the firmware was given.
    pub bytes: Vec<u8>,
}

/// One parsed `[display_glyph <name>]` section (`display.py:103-114`).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Glyph {
    /// The 16x16 icon, split into its two byte-per-row halves
    /// (`display.py:159-161`).
    pub icon16x16: Option<(Vec<u8>, Vec<u8>)>,
    /// The HD44780 CGRAM slot and the 5x8 icon (`display.py:164-166`).
    pub icon5x8: Option<(u8, Vec<u8>)>,
}

// ===========================================================================
// Layouts
// ===========================================================================

/// One `[display_data <group> <item>]` row (`display.py:57-77`).
#[derive(Debug)]
pub struct DisplayDataItem {
    /// The screen row.
    pub row: usize,
    /// The screen column.
    pub col: usize,
    /// The section identifier, e.g. `display_data _default_16x4 extruder`.
    pub identifier: String,
    /// The item's `text` option as written (its template source).
    pub text: String,
    /// That `text` compiled, the way upstream compiles it while building the
    /// group (`gcode_macro.load_template(c, 'text')`, `display.py:70`).
    template: Template,
}

/// One `display_data` group: its items, ordered by screen position
/// (`display.py:68-77`).
#[derive(Debug, Default)]
pub struct DisplayGroup {
    items: Vec<DisplayDataItem>,
}

impl DisplayGroup {
    /// The items, ordered by `(row, col, identifier)` as upstream sorts them
    /// (`display.py:73`).
    pub fn items(&self) -> &[DisplayDataItem] {
        &self.items
    }
}

// ===========================================================================
// PrinterLCD
// ===========================================================================

/// The `display` printer object: one panel plus the layout and templates it
/// draws (`display.py:176`).
pub struct PrinterLCD {
    /// The panel.
    lcd_chip: Arc<dyn LcdChip>,
    /// Every `display_data` group by name (`display.py:192`).
    display_data_groups: BTreeMap<String, DisplayGroup>,
    /// The name of the group currently shown (`display.py:198`).
    show_data_group: Mutex<String>,
    /// The template registry (`display.py:191`).
    display_templates: Arc<DisplayTemplates>,
    /// The menu, on the primary display only (`display.py:184-188`).
    menu: Mutex<Option<Arc<MenuManager>>>,
    /// The printer a layout's bodies read statuses through.
    printer: Arc<Printer>,
    /// The reactor the refresh timer lives on.
    reactor: Arc<dyn Reactor>,
    /// The refresh timer, registered at ready.
    timer: Mutex<Option<TimerHandle>>,
    /// Whether something asked for a redraw sooner than the next period
    /// (`display.py:205`).
    redraw_request_pending: AtomicBool,
    /// The time the pending redraw is due at (`display.py:206`).
    redraw_time: Mutex<f64>,
    /// A weak handle to this object, for the event handler and the timer.
    self_ref: Weak<PrinterLCD>,
}

impl PrinterLCD {
    /// Build the display: panel, layouts, templates, glyphs, group and
    /// g-code.
    ///
    /// Upstream's `PrinterLCD.__init__` (`display.py:177-214`), with the menu
    /// left out (module docs).
    ///
    /// # Errors
    /// A missing or unknown `lcd_type`, a missing option in any of the
    /// sections the display reads, or an unknown `display_group`.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        // The panel first: upstream builds it before anything else
        // (`display.py:181`), and its width decides the default group.
        let lcd_type = config.get_choice("lcd_type", &lcd_type_names(), None)?;
        let (_, build) = LCD_CHIPS
            .iter()
            .find(|(name, _)| *name == lcd_type)
            .expect("`get_choice` only accepts a name from `LCD_CHIPS`");
        let lcd_chip = build(config, printer)?;

        // The `[display]` section is the only section that registers a display
        // (`display.py:185-187`); the menu is therefore always built, and its
        // key options are always read.
        let menu_options = read_menu_options(config)?;

        ensure_display_status(printer)?;

        let display_templates = load_display_templates(config, printer)?;
        let glyphs = load_display_glyphs(config)?;
        lcd_chip.set_glyphs(&glyphs);

        let mut default_group = DEFAULT_GROUP_16X4;
        if lcd_chip.get_dimensions().0 == 20 {
            default_group = DEFAULT_GROUP_20X4;
        }
        let dgroup = config.get("display_group", Some(default_group))?;
        let display_data_groups = load_display_groups(config)?;
        if !display_data_groups.contains_key(&dgroup) {
            return Err(ConfigError::new(format!(
                "Unknown display_data group '{dgroup}'"
            )));
        }

        let display = Arc::new_cyclic(|weak| Self {
            lcd_chip,
            display_data_groups,
            show_data_group: Mutex::new(dgroup),
            display_templates,
            menu: Mutex::new(None),
            printer: Arc::clone(printer),
            reactor: printer.reactor(),
            timer: Mutex::new(None),
            redraw_request_pending: AtomicBool::new(false),
            redraw_time: Mutex::new(0.0),
            self_ref: weak.clone(),
        });

        // The menu is built after the panel, because it reads the panel's
        // dimensions (`menu.py:704`).
        let menu = MenuManager::new(config, printer, &menu_options, &display)?;
        *display
            .menu
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = Some(menu);

        let ready = Arc::downgrade(&display);
        printer.register_event_handler(
            KlippyEvent::KlippyReady,
            Box::new(move |_| {
                if let Some(display) = ready.upgrade() {
                    display.handle_ready();
                }
            }),
        );
        display.register_command(printer)?;
        Ok(display)
    }

    /// Register `SET_DISPLAY_GROUP` (`display.py:207-214`).
    fn register_command(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let handler: CommandHandler = {
            let display = Arc::clone(self);
            sync(move |gcmd| display.cmd_set_display_group(gcmd))
        };
        gcode
            .register_mux_command_with_params(
                "SET_DISPLAY_GROUP",
                "DISPLAY",
                Some("display"),
                Arc::clone(&handler),
                Some(SET_DISPLAY_GROUP_HELP),
                &["GROUP"],
            )
            .and_then(|()| {
                // The primary display also answers the unqualified command
                // (`display.py:212-214`).
                gcode.register_mux_command_with_params(
                    "SET_DISPLAY_GROUP",
                    "DISPLAY",
                    None,
                    handler,
                    Some(SET_DISPLAY_GROUP_HELP),
                    &["GROUP"],
                )
            })
            .map_err(ConfigError::new)
    }

    /// `klippy:ready` (`display.py:217-220`): bring the panel up, then start the
    /// refresh timer now.
    fn handle_ready(&self) {
        self.lcd_chip.init();
        // Upstream updates the (already registered) timer to `NOW`; this host
        // registers it here instead, because a timer is created and cancelled
        // rather than re-armed.
        let reactor = Arc::clone(&self.reactor);
        let weak = self.self_ref.clone();
        let handle = reactor.register_timer_named(
            "display",
            Box::new(move |eventtime| {
                let display = weak.upgrade()?;
                display.screen_update_event(eventtime)
            }),
            reactor.monotonic(),
        );
        *self.timer.lock().unwrap_or_else(|p| p.into_inner()) = Some(handle);
    }

    /// One refresh (`display.py:222-241`), into `chip`.
    ///
    /// The panel is an argument rather than `self.lcd_chip` so that a frame can
    /// be drawn into another [`LcdChip`] — that is how a test reads one back.
    /// The rest is upstream's order: the menu is asked first, and while it is
    /// open it owns the screen and the group is not drawn (`display.py:227-231`).
    /// Otherwise the frame is the active layout redrawn from scratch — clear,
    /// render and draw the group's items, flush — and what the render raises is
    /// swallowed and logged (`display.py:234-237`), so a broken layout shows the
    /// part it managed to draw instead of stopping the printer.
    fn draw_screen(&self, chip: &Arc<dyn LcdChip>, eventtime: f64) -> Option<f64> {
        if self.redraw_request_pending.swap(false, Ordering::SeqCst) {
            *self.redraw_time.lock().unwrap_or_else(|p| p.into_inner()) =
                eventtime + REDRAW_MIN_TIME;
        }
        chip.clear();
        // `display.py:227-231`: the menu redraws the screen and asks for the
        // regular period.
        if let Some(menu) = self.menu() {
            if menu.screen_update_event(chip, eventtime) {
                chip.flush();
                return Some(eventtime + REDRAW_TIME);
            }
        }
        // With the menu closed the group owns the frame; whatever a layout
        // raises is swallowed here (`display.py:234-237`).
        let group = self.show_data_group();
        if let Some(group) = self.display_data_groups.get(&group) {
            if let Err(err) = show_group(chip, group, &self.display_templates, &self.printer) {
                warn!("Error during display screen update: {err}");
            }
        }
        chip.flush();
        if self.redraw_request_pending.load(Ordering::SeqCst) {
            return Some(
                *self
                    .redraw_time
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner()),
            );
        }
        Some(eventtime + REDRAW_TIME)
    }

    /// One refresh of this display's own panel (`display.py:222-241`).
    fn screen_update_event(&self, eventtime: f64) -> Option<f64> {
        self.draw_screen(&self.lcd_chip, eventtime)
    }

    /// `SET_DISPLAY_GROUP` (`display.py:266-271`).
    fn cmd_set_display_group(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let group = gcmd.get_str("GROUP")?;
        if !self.display_data_groups.contains_key(&group) {
            return Err(CommandError::new(format!(
                "Unknown display_data group '{group}'"
            )));
        }
        *self
            .show_data_group
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = group;
        Ok(())
    }

    /// The group currently shown.
    pub fn show_data_group(&self) -> String {
        self.show_data_group
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }

    /// Every layout group by name, as the shipped file and the config define
    /// them.
    pub fn display_data_groups(&self) -> &BTreeMap<String, DisplayGroup> {
        &self.display_data_groups
    }

    /// The template registry this display reads.
    pub fn display_templates(&self) -> &Arc<DisplayTemplates> {
        &self.display_templates
    }

    /// The menu this display owns, if it was built.
    pub fn menu(&self) -> Option<Arc<MenuManager>> {
        self.menu
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }

    /// The panel's size in characters (`display.py:215-216`).
    pub fn get_dimensions(&self) -> (usize, usize) {
        self.lcd_chip.get_dimensions()
    }

    /// Ask for a redraw sooner than the next period (`request_redraw`,
    /// `display.py:242-246`).
    pub fn request_redraw(&self) {
        if self.redraw_request_pending.load(Ordering::SeqCst) {
            return;
        }
        self.redraw_request_pending.store(true, Ordering::SeqCst);
    }

    /// Every message the panel has handed to the firmware.
    pub fn sent_messages(&self) -> Vec<SentMessage> {
        self.lcd_chip.sent_messages()
    }

    /// How many messages the panel has handed to the firmware.
    pub fn sent_message_count(&self) -> usize {
        self.lcd_chip.sent_message_count()
    }
}

impl PrinterObject for PrinterLCD {
    /// Upstream's `PrinterLCD` defines no `get_status`, so it is not
    /// client-visible (`display.py:176`); the trait needs one.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for PrinterLCD {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterLCD")
            .field("groups", &self.display_data_groups.keys().count())
            .field("group", &self.show_data_group())
            .field("sent_messages", &self.lcd_chip.sent_message_count())
            .finish_non_exhaustive()
    }
}

/// `SET_DISPLAY_GROUP`'s help text (`display.py:265`).
const SET_DISPLAY_GROUP_HELP: &str = "Set the active display group";

/// Upstream's `load_config` (`display.py:273-274`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(PrinterLCD::new(config, printer)?)
}

// ===========================================================================
// The menu options
// ===========================================================================

/// What the menu reads off the `[display]` section (`menu.py:689-722`).
#[derive(Debug, Clone, PartialEq)]
pub struct MenuOptions {
    /// `menu_timeout` (`menu.py:703`).
    pub timeout: i64,
    /// `menu_reverse_navigation` (`menu.py:705-707`).
    pub reverse_navigation: bool,
    /// `menu_root`: the container the menu opens at (`menu.py:701`).
    pub root: String,
    /// The key options `menu_keys.py` reads.
    pub keys: MenuKeysOptions,
}

/// The key options (`MenuKeys.__init__`, `menu_keys.py:13-43`).
#[derive(Debug, Clone, PartialEq)]
pub struct MenuKeysOptions {
    /// `encoder_pins`, as the two pin names written (`menu_keys.py:20-31`).
    pub encoder_pins: Option<(String, String)>,
    /// `encoder_steps_per_detent`: 2 or 4 (`menu_keys.py:21-22`).
    pub encoder_steps_per_detent: u8,
    /// `encoder_fast_rate`: the gap under which two ticks are a fast turn
    /// (`menu_keys.py:32`).
    pub encoder_fast_rate: f64,
    /// `click_pin` (`menu_keys.py:38`).
    pub click_pin: Option<String>,
    /// `back_pin` (`menu_keys.py:40`).
    pub back_pin: Option<String>,
    /// `up_pin` (`menu_keys.py:41`).
    pub up_pin: Option<String>,
    /// `down_pin` (`menu_keys.py:42`).
    pub down_pin: Option<String>,
    /// `kill_pin` (`menu_keys.py:43`).
    pub kill_pin: Option<String>,
}

impl MenuKeysOptions {
    /// The pin an option name holds, when it is set.
    pub fn pin(&self, option: &str) -> Option<String> {
        match option {
            "click_pin" => self.click_pin.clone(),
            "back_pin" => self.back_pin.clone(),
            "up_pin" => self.up_pin.clone(),
            "down_pin" => self.down_pin.clone(),
            "kill_pin" => self.kill_pin.clone(),
            _ => None,
        }
    }
}

/// Read the options the menu consumes.
///
/// Upstream reads them in `MenuManager.__init__` (`menu.py:689-722`) and
/// `MenuKeys.__init__` (`menu_keys.py:13-43`); the manager here takes this
/// struct, so the section is read once.
///
/// The pins are read but not resolved: a pin string is handed to the buttons
/// registry as written, as upstream hands it to `buttons.register_button*`.
///
/// # Errors
/// Upstream's wording: an unparseable `encoder_pins` (`Unable to parse
/// encoder_pins`, `menu_keys.py:22-26`), a bad `encoder_steps_per_detent`
/// choice, a non-positive `encoder_fast_rate`, or a malformed
/// `analog_range_*` pair.
pub fn read_menu_options(config: &ConfigWrapper) -> Result<MenuOptions, ConfigError> {
    let identifier = config.identifier();
    // `MenuManager.__init__` (`menu.py:689-722`).
    let root = config.get("menu_root", Some("__main"))?;
    let timeout = config.get_int("menu_timeout", Some(0))?;
    let reverse_navigation = config.get_bool("menu_reverse_navigation", Some(false))?;

    // `MenuKeys.__init__` (`menu_keys.py:13-43`).
    let encoder_pins = match config.get_str("encoder_pins") {
        None => None,
        Some(pins) => {
            let mut parts = pins.split(',');
            let first = parts.next().unwrap_or("").trim().to_string();
            match (parts.next().map(str::trim), parts.next()) {
                (Some(second), None) => Some((first, second.to_string())),
                _ => return Err(ConfigError::new("Unable to parse encoder_pins".to_string())),
            }
        }
    };
    let detent = config.get_choice("encoder_steps_per_detent", &["2", "4"], Some("4"))?;
    let encoder_steps_per_detent = detent.parse::<u8>().unwrap_or(4);
    let encoder_fast_rate = config.get_float_bounded(
        "encoder_fast_rate",
        Some(0.030),
        None,
        None,
        Some(0.0),
        None,
    )?;

    // `MenuKeys.register_button` (`menu_keys.py:45-61`): the button pins, each
    // optionally paired with an analog range and a pull-up resistor.
    let click_pin = config.get_str("click_pin");
    let back_pin = config.get_str("back_pin");
    let up_pin = config.get_str("up_pin");
    let down_pin = config.get_str("down_pin");
    let kill_pin = config.get_str("kill_pin");
    let mut analog = false;
    for (option, pin) in [
        ("click_pin", &click_pin),
        ("back_pin", &back_pin),
        ("up_pin", &up_pin),
        ("down_pin", &down_pin),
        ("kill_pin", &kill_pin),
    ] {
        if pin.is_none() {
            continue;
        }
        let range = format!("analog_range_{option}");
        if config.get_str(&range).is_some() {
            parse_float_list(config, &range, identifier.as_str(), 2)?;
            analog = true;
        }
    }
    if analog {
        config.get_float_bounded(
            "analog_pullup_resistor",
            Some(4700.0),
            None,
            None,
            Some(0.0),
            None,
        )?;
    }
    Ok(MenuOptions {
        timeout,
        reverse_navigation,
        root,
        keys: MenuKeysOptions {
            encoder_pins,
            encoder_steps_per_detent,
            encoder_fast_rate,
            click_pin,
            back_pin,
            up_pin,
            down_pin,
            kill_pin,
        },
    })
}

/// Parse an option as a comma-separated float list of exactly `count` values.
///
/// The wrapper has no float-list getter; this is `getfloatlist(count=2)`
/// (`klippy/configfile.py:115`) with its two error wordings.
fn parse_float_list(
    config: &ConfigWrapper,
    option: &str,
    identifier: &str,
    count: usize,
) -> Result<Vec<f64>, ConfigError> {
    let text = config.get(option, None)?;
    let mut values = Vec::new();
    for part in text.split(',') {
        let part = part.trim();
        if part.is_empty() {
            continue;
        }
        let value = part.parse::<f64>().map_err(|_| {
            ConfigError::new(format!(
                "Unable to parse option '{option}' in section '{identifier}'"
            ))
        })?;
        values.push(value);
    }
    if values.len() != count {
        return Err(ConfigError::new(format!(
            "Option '{option}' in section '{identifier}' must have {count} elements"
        )));
    }
    Ok(values)
}

// ===========================================================================
// The shipped layout
// ===========================================================================

/// Parse the shipped layout.
///
/// Upstream `pconfig.read_config(<display.cfg>)` (`display.py:118-120`); a
/// failure there is `Cannot load config '<path>'`.
fn published_config() -> Result<Config, ConfigError> {
    Config::from_text(DISPLAY_CFG)
        .map(|(config, _)| config)
        .map_err(|err| ConfigError::new(format!("Cannot load config '{DISPLAY_CFG_NAME}': {err}")))
}

/// The template registry, with the shipped layout merged in.
///
/// Upstream's `lookup_display_templates` (`display.py:124-131`, `:168-174`)
/// loads the main config's `[display_template]` sections — which the loader
/// already did, through their factory in [`super::led`] — and then every shipped
/// one whose name the main config does not define, so the main config wins.
///
/// # Errors
/// A missing `text` or an invalid `param_*` literal in a shipped template.
fn load_display_templates(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<DisplayTemplates>, ConfigError> {
    let main: BTreeSet<String> = config
        .sibling_prefix_sections("display_template")
        .iter()
        .map(ConfigWrapper::identifier)
        .collect();
    let published = published_config()?;
    for section in published.get_sections_by_id("display_template") {
        if skip_published(section, &main) {
            continue;
        }
        let wrapper = ConfigWrapper::untracked(section);
        load_display_template(&wrapper, printer)?;
    }
    printer
        .lookup_object_as::<DisplayTemplates>(DISPLAY_TEMPLATES_OBJECT)
        .ok_or_else(|| {
            ConfigError::new("the display_template registry was not created".to_string())
        })
}

/// The `display_data` groups, main config first, then the shipped layout.
///
/// # Errors
/// Upstream's wording: a section name that is not `<id> <group> <item>`, or a
/// `position` that is not `row, col` (`display.py:61-66`).
fn load_display_groups(
    config: &ConfigWrapper,
) -> Result<BTreeMap<String, DisplayGroup>, ConfigError> {
    let mut seen = BTreeSet::new();
    let mut collected: BTreeMap<String, Vec<DisplayDataItem>> = BTreeMap::new();
    for wrapper in config.sibling_prefix_sections("display_data") {
        let (group, item) = parse_data_item(&wrapper)?;
        seen.insert(wrapper.identifier());
        collected.entry(group).or_default().push(item);
    }
    let published = published_config()?;
    for section in published.get_sections_by_id("display_data") {
        if skip_published(section, &seen) {
            continue;
        }
        let wrapper = ConfigWrapper::untracked(section);
        let (group, item) = parse_data_item(&wrapper)?;
        collected.entry(group).or_default().push(item);
    }
    Ok(collected
        .into_iter()
        .map(|(group, mut items)| {
            items.sort_by(|a, b| (a.row, a.col, &a.identifier).cmp(&(b.row, b.col, &b.identifier)));
            (group, DisplayGroup { items })
        })
        .collect())
}

/// One `[display_data <group> <item>]` section (`display.py:57-77`).
///
/// # Errors
/// Upstream's wording for a malformed section name or `position`.
fn parse_data_item(config: &ConfigWrapper) -> Result<(String, DisplayDataItem), ConfigError> {
    let identifier = config.identifier();
    let name_parts: Vec<&str> = identifier.split(' ').collect();
    if name_parts.len() != 3 {
        return Err(ConfigError::new(format!(
            "Section name '{identifier}' is not valid"
        )));
    }
    let group = name_parts[1].to_string();
    let position = config.get("position", None)?;
    let (row, col) = parse_position(&position).ok_or_else(|| {
        ConfigError::new(format!(
            "Unable to parse 'position' in section '{identifier}'"
        ))
    })?;
    let text = config.get("text", None)?;
    let template = Template::parse(&format!("{identifier}:text"), &text)
        .map_err(|error| ConfigError::new(error.to_string()))?;
    Ok((
        group,
        DisplayDataItem {
            row,
            col,
            identifier,
            text,
            template,
        },
    ))
}

/// Upstream's `row, col = [int(v.strip()) for v in pos.split(',')]`: exactly two
/// integers (`display.py:62-63`).
fn parse_position(position: &str) -> Option<(usize, usize)> {
    let parts: Vec<&str> = position.split(',').collect();
    if parts.len() != 2 {
        return None;
    }
    let row = parts[0].trim().parse::<usize>().ok()?;
    let col = parts[1].trim().parse::<usize>().ok()?;
    Some((row, col))
}

/// The glyphs, main config first, then the shipped layout.
///
/// # Errors
/// Upstream's `Invalid glyph line in <name>` / `Glyph <name> incorrect lines`
/// (`display.py:109-113`), and a missing or out-of-range `hd44780_slot`.
fn load_display_glyphs(config: &ConfigWrapper) -> Result<BTreeMap<String, Glyph>, ConfigError> {
    let mut glyphs = BTreeMap::new();
    let main: BTreeSet<String> = config
        .sibling_prefix_sections("display_glyph")
        .iter()
        .map(ConfigWrapper::identifier)
        .collect();
    for wrapper in config.sibling_prefix_sections("display_glyph") {
        let (name, glyph) = parse_glyph_section(&wrapper)?;
        glyphs.insert(name, glyph);
    }
    let published = published_config()?;
    for section in published.get_sections_by_id("display_glyph") {
        if skip_published(section, &main) {
            continue;
        }
        let wrapper = ConfigWrapper::untracked(section);
        let (name, glyph) = parse_glyph_section(&wrapper)?;
        glyphs.insert(name, glyph);
    }
    Ok(glyphs)
}

/// One `[display_glyph <name>]` section (`display.py:147-166`).
///
/// # Errors
/// Upstream's glyph wording, and `hd44780_slot`'s bounds.
fn parse_glyph_section(config: &ConfigWrapper) -> Result<(String, Glyph), ConfigError> {
    let identifier = config.identifier();
    let name = config
        .section()
        .sub
        .clone()
        .ok_or_else(|| ConfigError::new(format!("Section name '{identifier}' is not valid")))?;
    let mut glyph = Glyph::default();
    if let Some(data) = config.get_str("data") {
        let rows = parse_glyph(&name, &data, 16, 16)?;
        let icon1 = rows.iter().map(|bits| (bits >> 8) as u8).collect();
        let icon2 = rows.iter().map(|bits| (bits & 0xff) as u8).collect();
        glyph.icon16x16 = Some((icon1, icon2));
    }
    if let Some(data) = config.get_str("hd44780_data") {
        let slot = config.get_int_bounded("hd44780_slot", None, Some(0), Some(7))? as u8;
        let rows = parse_glyph(&name, &data, 5, 8)?;
        glyph.icon5x8 = Some((slot, rows.into_iter().map(|bits| bits as u8).collect()));
    }
    Ok((name, glyph))
}

/// Upstream's `PrinterDisplayTemplate._parse_glyph` (`display.py:103-114`).
///
/// The two error wordings are the user's: a line that is not `width` columns of
/// `.`/`*`, and a glyph whose line count is not `height`.
///
/// # Errors
/// As described above.
fn parse_glyph(
    name: &str,
    data: &str,
    width: usize,
    height: usize,
) -> Result<Vec<u16>, ConfigError> {
    let mut rows = Vec::new();
    for line in data.split('\n') {
        let line = line.trim().replace('.', "0").replace('*', "1");
        if line.is_empty() {
            continue;
        }
        if line.len() != width || line.chars().any(|c| c != '0' && c != '1') {
            return Err(ConfigError::new(format!("Invalid glyph line in {name}")));
        }
        rows.push(
            u16::from_str_radix(&line, 2)
                .map_err(|_| ConfigError::new(format!("Invalid glyph line in {name}")))?,
        );
    }
    if rows.len() != height {
        return Err(ConfigError::new(format!("Glyph {name} incorrect lines")));
    }
    Ok(rows)
}

// ===========================================================================
// The layout sections' factories
// ===========================================================================

/// Claim one `[display_data <group> <item>]` section of the config.
///
/// Upstream reads these sections straight out of the config
/// (`display.py:132-145`); here a section is valid exactly when a factory
/// claims it, so this is the claim. The section is then handed to the same
/// parse path the shipped layout uses ([`parse_data_item`]), which is what
/// reports a malformed section under its own name and records its options for
/// the undefined-option check.
///
/// The parsed item is not kept: a group is assembled from every `display_data`
/// section at once and merged with the shipped `display.cfg`, which only
/// `[display]` can do ([`load_display_groups`]).
///
/// # Errors
/// Upstream's wording for a malformed section name or `position`.
pub fn load_display_data(
    config: &ConfigWrapper,
    _printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    parse_data_item(config)?;
    Ok(Arc::new(LayoutSection))
}

/// Claim one `[display_glyph <name>]` section of the config.
///
/// The counterpart of [`load_display_data`]: [`parse_glyph_section`] reads the
/// two icon encodings and the CGRAM slot, and the merged set is the one the
/// panel is given ([`load_display_glyphs`]).
///
/// # Errors
/// Upstream's glyph wording, and `hd44780_slot`'s bounds.
pub fn load_display_glyph(
    config: &ConfigWrapper,
    _printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    parse_glyph_section(config)?;
    Ok(Arc::new(LayoutSection))
}

/// What a claimed `[display_data]`/`[display_glyph]` section registers.
///
/// It carries nothing — see [`load_display_data`] — and is not a printer part,
/// so it does not reach `objects/list`.
struct LayoutSection;

impl PrinterObject for LayoutSection {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

/// Whether a shipped section is skipped because the main config defines a
/// section with the same identifier (`display.py:126-128`).
fn skip_published(section: &ConfigSection, main: &BTreeSet<String>) -> bool {
    if section.sub.is_none() {
        return true;
    }
    main.contains(&section.identifier())
}

/// Check one `render("name", k=…)` call against a template's declared
/// parameters.
///
/// Upstream's `DisplayTemplate.render` (`display/display.py:45-53`) copies the
/// template's `param_*` values and then updates them with the call's keywords:
/// the sizes only match while every keyword the call passes is one the template
/// already declares. That rule is user-visible text, so it is kept here —
/// [`render_callable`] is the caller, the body's `render(...)`.
///
/// # Errors
/// `Invalid parameter to display_template <name>` for a keyword the template
/// does not declare.
pub fn check_render_params(
    params: &[String],
    template_name: &str,
    kwargs: &[&str],
) -> Result<(), ConfigError> {
    if kwargs.iter().all(|kwarg| params.iter().any(|p| p == kwarg)) {
        return Ok(());
    }
    Err(ConfigError::new(format!(
        "Invalid parameter to display_template {template_name}"
    )))
}

// ===========================================================================
// The render step
// ===========================================================================

/// The context a layout's bodies render against, including the two callables
/// this step owns.
///
/// Upstream's `DisplayGroup.show` builds one context from the first item's
/// `create_template_context` and then adds `render` and `draw_progress_bar`
/// (`display.py:78-83`); the globals are the ones `create_template_context`
/// binds (`gcode_macro.py:106-113`).
type ContextFactory = Arc<dyn Fn() -> Context + Send + Sync>;

/// Build that context. `chip` is what `draw_progress_bar` paints into; a
/// nested `render()` builds its own context through the same factory, which is
/// what upstream's `dict(context)` copy gives it (`display.py:50`).
pub(crate) fn layout_context(
    chip: &Arc<dyn LcdChip>,
    templates: &Arc<DisplayTemplates>,
    printer: &Arc<Printer>,
) -> Context {
    let factory: ContextFactory = {
        let chip = Arc::clone(chip);
        let templates = Arc::clone(templates);
        let printer = Arc::clone(printer);
        Arc::new(move || layout_context(&chip, &templates, &printer))
    };

    let mut context = Context::new();
    context.insert(
        "printer",
        Rt::Printer(PrinterView::new(Arc::clone(printer))),
    );
    context.insert(
        "action_respond_info",
        Rt::Builtin(Builtin::RespondInfo(Arc::clone(printer))),
    );
    context.insert("action_raise_error", Rt::Builtin(Builtin::RaiseError));
    context.insert("range", Rt::Builtin(Builtin::Range));
    context.set_callable("render", render_callable(Arc::clone(templates), factory));
    context.set_callable("draw_progress_bar", progress_bar_callable(Arc::clone(chip)));
    context
}

/// `context['render']` — a body's `render("<template>", k=v)`
/// (`display.py:80-82`).
///
/// The keyword arguments are the call's own: one the template does not declare
/// is upstream's `Invalid parameter to display_template <name>`
/// (`display.py:43-47`, [`check_render_params`]), and an unknown template name
/// is the `KeyError` its `templates[name]` lookup raises.
fn render_callable(templates: Arc<DisplayTemplates>, factory: ContextFactory) -> HostCallable {
    Arc::new(move |args, kwargs| {
        let name = args.first().and_then(Value::as_str).ok_or_else(|| {
            "TypeError: render() missing 1 required positional argument: 'name'".to_string()
        })?;
        let declared = templates
            .param_names(name)
            .ok_or_else(|| format!("KeyError: '{name}'"))?;
        let params: Vec<(String, Value)> = kwargs.to_vec();
        let written: Vec<&str> = params.iter().map(|(key, _)| key.as_str()).collect();
        check_render_params(&declared, name, &written).map_err(|error| error.to_string())?;
        let text = templates
            .render(name, &mut factory(), &params)
            .ok_or_else(|| format!("KeyError: '{name}'"))?;
        text.map(Value::String).map_err(|error| error.to_string())
    })
}

/// `context['draw_progress_bar']` — a body's
/// `draw_progress_bar(row, col, width, value)` (`display.py:83`, `:259-264`).
///
/// It paints into the panel and returns `""`, which is what upstream's
/// `PrinterLCD.draw_progress_bar` returns: the bar is graphics, not text.
fn progress_bar_callable(chip: Arc<dyn LcdChip>) -> HostCallable {
    Arc::new(move |args, _kwargs| {
        let row = cell_argument(args, 0, "row")?;
        let col = cell_argument(args, 1, "col")?;
        let width = cell_argument(args, 2, "width")?;
        let value = args.get(3).and_then(Value::as_f64).ok_or_else(|| {
            "TypeError: draw_progress_bar() argument 'value' must be a number".to_string()
        })?;
        draw_progress_bar(&*chip, row, col, width, value);
        Ok(Value::String(String::new()))
    })
}

/// One `draw_progress_bar` cell argument, or the wording Python's own missing
/// argument would get.
fn cell_argument(args: &[Value], at: usize, name: &str) -> Result<usize, String> {
    args.get(at)
        .and_then(Value::as_u64)
        .map(|value| value as usize)
        .ok_or_else(|| {
            format!("TypeError: draw_progress_bar() argument '{name}' must be an integer")
        })
}

/// Draw one layout: upstream's `DisplayGroup.show` (`display.py:78-87`).
///
/// Every item's `text` is rendered in the group's context and drawn at the
/// item's own `position` — with its newlines dropped, as upstream drops them
/// before drawing (`display.py:86`).
///
/// # Errors
/// The first body that fails to render, worded by the engine or by the
/// callable it called: the items already drawn stay on the panel, which is
/// upstream's behaviour too (the exception ends the group's frame,
/// `display.py:234-237`).
pub(crate) fn show_group(
    chip: &Arc<dyn LcdChip>,
    group: &DisplayGroup,
    templates: &Arc<DisplayTemplates>,
    printer: &Arc<Printer>,
) -> Result<(), TemplateError> {
    let mut context = layout_context(chip, templates, printer);
    for item in &group.items {
        let text = item.template.render(&mut context)?;
        draw_text(&**chip, item.row, item.col, &text.replace('\n', ""));
    }
    Ok(())
}

/// Write one item's `mixed_text` at (`row`, `col`): upstream's
/// `PrinterLCD.draw_text` (`display.py:247-258`).
///
/// The text is split on `~`: the even runs are text and cost their length in
/// cells, the odd ones are glyph names and cost what the panel's
/// [`LcdChip::write_glyph`] says it drew. Returns the column the write ended
/// at — upstream returns it for the menu, which this host does not have.
pub fn draw_text(chip: &dyn LcdChip, row: usize, col: usize, mixed_text: &str) -> usize {
    let mut pos = col;
    for (index, text) in mixed_text.split('~').enumerate() {
        if index % 2 == 0 {
            chip.write_text(pos, row, text.as_bytes());
            pos += text.len();
        } else {
            pos += chip.write_glyph(pos, row, text);
        }
    }
    pos
}

/// Paint a progress bar into the panel: upstream's
/// `PrinterLCD.draw_progress_bar` (`display.py:259-264`).
///
/// The bar is a bitmap over `row`, `width` 8-pixel columns wide, sent as one
/// 16-byte graphics column per cell — `0xff` above and below, the middle 14
/// bytes the bar. Bit `k` of that bitmap is set when `k` is at or past
/// `int(width * 8 * (1. - value) + .5)` (the empty part of the bar), or when it
/// is the first or the last bit (the frame's two edges); column `i` (0 = the
/// rightmost) sends bits `i * 8` … `i * 8 + 7`.
///
/// The bits are computed one at a time rather than as Python's
/// `-1 << n | …` on an arbitrary-precision int, so a bar wider than 128 bits
/// still works; a `value` above 1. is past full and draws a full bar, where
/// Python's negative shift would raise and abandon the frame.
pub fn draw_progress_bar(chip: &dyn LcdChip, row: usize, col: usize, width: usize, value: f64) {
    let bits = width * 8;
    let filled = (bits as f64 * (1.0 - value) + 0.5).trunc() as i64;
    for column in 0..width {
        let mut byte = 0u8;
        for bit_index in 0..8 {
            let bit = column * 8 + bit_index;
            if bit as i64 >= filled || bit == 0 || bit + 1 == bits {
                byte |= 1 << bit_index;
            }
        }
        // Above and below the bar's 14 pixel rows the column is solid.
        let mut data = Vec::with_capacity(16);
        data.push(0xff);
        data.extend(std::iter::repeat(byte).take(14));
        data.push(0xff);
        chip.write_graphics(col + width - 1 - column, row, &data);
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::value::ConfigValue;
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[display]` section with `options`, as the parser stores them.
    fn cfg_section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("display", None);
        for (key, value) in options {
            section.parameters.insert(
                key.to_lowercase(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    fn wrap<'a>(section: &'a ConfigSection, access: &Arc<AccessTracking>) -> ConfigWrapper<'a> {
        ConfigWrapper::new(section, Arc::clone(access))
    }

    /// A machine with nothing but a printer in it, for the loader tests: the
    /// loader itself registers `gcode`, `configfile` and `pins`.
    fn machine() -> Arc<Printer> {
        Arc::new(Printer::new(ManualReactor::shared()))
    }

    /// A config with one MCU and one display, as the corpus writes them.
    fn display_config(display: &str) -> Config {
        let text = format!("[mcu]\nserial: /dev/a\n[display]\n{display}");
        Config::from_text(&text).expect("the config parses").0
    }

    // -- the shipped layout -------------------------------------------------

    #[test]
    fn test_the_vendored_layout_has_not_drifted() {
        // The shipped `display.cfg` is a copy; this is the guard that says so.
        // A worktree without the submodule checked out has nothing to compare
        // against and skips — point `KLIPPERX_KLIPPER_DIR` at the main checkout
        // to run it.
        let upstream =
            klipperx_test_support::klipper_dir().join("klippy/extras/display/display.cfg");
        let Ok(text) = std::fs::read_to_string(&upstream) else {
            return;
        };
        assert_eq!(
            DISPLAY_CFG,
            text,
            "{} is out of date with the shipped display.cfg",
            upstream.display()
        );
    }

    #[test]
    fn test_the_shipped_layout_parses_and_keeps_upstreams_shape() {
        let published = published_config().expect("the shipped layout parses");

        // 4 templates, 24 data items (three groups of eight), and 12 glyphs:
        // the five names the 20x4 block repeats are merged into the 16x4
        // sections, exactly as upstream's `configparser` merges them.
        assert_eq!(published.get_sections_by_id("display_template").len(), 4);
        assert_eq!(published.get_sections_by_id("display_data").len(), 24);
        assert_eq!(published.get_sections_by_id("display_glyph").len(), 12);
    }

    // -- positions and glyphs ----------------------------------------------

    #[test]
    fn test_a_position_is_two_integers() {
        assert_eq!(parse_position("0, 10"), Some((0, 10)));
        assert_eq!(parse_position("3,0"), Some((3, 0)));
        assert_eq!(parse_position("0"), None);
        assert_eq!(parse_position("0, 1, 2"), None);
        assert_eq!(parse_position("row, col"), None);
    }

    #[test]
    fn test_a_glyph_line_must_be_the_full_width() {
        // A line that is not five columns, and one with a character that is
        // neither `.` nor `*` (`display.py:109-110`).
        let err = parse_glyph("bed", ".....\n....", 5, 8).unwrap_err();
        assert_eq!(err.to_string(), "Invalid glyph line in bed");

        let err = parse_glyph("bed", ".....\n..*.x\n", 5, 8).unwrap_err();
        assert_eq!(err.to_string(), "Invalid glyph line in bed");
    }

    #[test]
    fn test_a_glyph_must_have_the_full_line_count() {
        let err = parse_glyph("bed", ".....\n.....", 5, 8).unwrap_err();
        assert_eq!(err.to_string(), "Glyph bed incorrect lines");
    }

    #[test]
    fn test_a_glyph_is_read_as_binary_rows() {
        // `.` is a clear bit and `*` a set one (`display.py:106`).
        let rows = parse_glyph("x", ".*.*.\n*****", 5, 2).unwrap();
        assert_eq!(rows, vec![0b01010, 0b11111]);
    }

    // -- the render parameter rule -----------------------------------------

    #[test]
    fn test_a_render_call_may_only_pass_declared_parameters() {
        let params = vec!["param_heater_name".to_string()];

        assert!(
            check_render_params(&params, "_heater_temperature", &["param_heater_name"]).is_ok()
        );
        assert!(check_render_params(&params, "_heater_temperature", &[]).is_ok());

        let err =
            check_render_params(&params, "_heater_temperature", &["param_other"]).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Invalid parameter to display_template _heater_temperature"
        );
    }

    // -- the menu options --------------------------------------------------

    #[test]
    fn test_the_menu_options_are_read() {
        let access = AccessTracking::shared();
        let section = cfg_section(&[
            ("menu_timeout", "40"),
            ("menu_reverse_navigation", "True"),
            ("encoder_pins", "^PC4, ^PC6"),
            ("encoder_steps_per_detent", "2"),
            ("up_pin", "^PA0"),
            ("click_pin", "^!PC2"),
            ("kill_pin", "^!PG0"),
            ("analog_range_up_pin", "100, 1000"),
            ("analog_pullup_resistor", "4700"),
        ]);

        let options = read_menu_options(&wrap(&section, &access)).unwrap();

        assert_eq!(
            options,
            MenuOptions {
                timeout: 40,
                reverse_navigation: true,
                root: "__main".to_string(),
                keys: MenuKeysOptions {
                    encoder_pins: Some(("^PC4".to_string(), "^PC6".to_string())),
                    encoder_steps_per_detent: 2,
                    encoder_fast_rate: 0.030,
                    click_pin: Some("^!PC2".to_string()),
                    back_pin: None,
                    up_pin: Some("^PA0".to_string()),
                    down_pin: None,
                    kill_pin: Some("^!PG0".to_string()),
                },
            }
        );
        for option in [
            "menu_root",
            "menu_timeout",
            "menu_reverse_navigation",
            "encoder_pins",
            "encoder_steps_per_detent",
            "encoder_fast_rate",
            "up_pin",
            "click_pin",
            "kill_pin",
            "analog_range_up_pin",
            "analog_pullup_resistor",
        ] {
            assert!(access.contains("display", option), "option {option}");
        }
    }

    #[test]
    fn test_an_unparseable_encoder_pins_is_reported() {
        let section = cfg_section(&[("encoder_pins", "^PC4")]);

        let err = read_menu_options(&wrap(&section, &AccessTracking::shared())).unwrap_err();

        assert_eq!(err.to_string(), "Unable to parse encoder_pins");
    }

    #[test]
    fn test_an_unknown_encoder_detent_is_a_choice_error() {
        let section = cfg_section(&[("encoder_steps_per_detent", "3")]);

        let err = read_menu_options(&wrap(&section, &AccessTracking::shared())).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Choice '3' for option 'encoder_steps_per_detent' in section 'display' is not a valid choice"
        );
    }

    #[test]
    fn test_an_analog_range_must_hold_two_values() {
        let section = cfg_section(&[("up_pin", "^PA0"), ("analog_range_up_pin", "100, 1000, 0")]);

        let err = read_menu_options(&wrap(&section, &AccessTracking::shared())).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'analog_range_up_pin' in section 'display' must have 2 elements"
        );
    }

    // -- the whole section -------------------------------------------------

    /// The corpus's `lcd_type: st7920` shape: three pins plus the menu keys the
    /// Creality boards and the Voron write.
    const ST7920_SECTION: &str = "lcd_type: st7920\n\
                                  cs_pin: PA3\n\
                                  sclk_pin: PA1\n\
                                  sid_pin: PC1\n\
                                  encoder_pins: ^PD2, ^PD3\n\
                                  click_pin: ^!PC0\n\
                                  kill_pin: ^!PG0\n\
                                  menu_timeout: 40\n";

    #[test]
    fn test_the_section_loads_and_leaves_no_option_unread() {
        // `Printer::load_config` ends in the undefined-option check, so a load
        // that returns `Ok` is the proof that every option written here was
        // read.
        let printer = machine();
        printer
            .load_config(&display_config(ST7920_SECTION))
            .expect("the section loads and reads every option");

        // `display_status` comes with the display, whether the config names it
        // or not (`display.py:188`), and it is registered before the display.
        assert_eq!(
            printer.objects(),
            [
                "gcode",
                "configfile",
                "pins",
                "mcu_ids",
                "error_mcu",
                "serial_ports",
                "mcu",
                "display_status",
                // The shipped layout's `display_template` sections bring the
                // registry up (`display/display.py:168-174`), before the
                // display itself is registered.
                "display_template",
                // The menu loads `gcode_macro`, then its key buttons, and
                // registers itself (`menu.py:713-721`) — all before the display
                // it belongs to.
                "gcode_macro",
                "buttons",
                "menu",
                "display",
                "idle_timeout",
                "printer_stats"
            ]
        );
        let display = printer
            .lookup_object_as::<PrinterLCD>("display")
            .expect("the display is registered under its section id");
        assert!(!display.is_queryable());

        // The shipped groups are all there, and the 16x4 one is the default.
        assert!(display.display_data_groups().contains_key("_default_16x4"));
        assert!(display.display_data_groups().contains_key("_default_20x4"));
        assert!(display
            .display_data_groups()
            .contains_key("_multiextruder_16x4"));
        assert_eq!(display.show_data_group(), "_default_16x4");
        assert_eq!(
            display.display_data_groups()["_default_16x4"].items().len(),
            8
        );

        // The shipped templates went into the registry, with their parameters.
        assert_eq!(
            display
                .display_templates()
                .param_names("_heater_temperature"),
            Some(vec!["param_heater_name".to_string()])
        );
        assert!(display
            .display_templates()
            .param_names("_printing_time")
            .is_some());
    }

    #[test]
    fn test_a_group_is_selected_by_display_group() {
        let printer = machine();
        printer
            .load_config(&display_config(&format!(
                "{ST7920_SECTION}display_group: _default_20x4\n"
            )))
            .expect("the section loads");
        let display = printer
            .lookup_object_as::<PrinterLCD>("display")
            .expect("the display");

        assert_eq!(display.show_data_group(), "_default_20x4");
    }

    #[test]
    fn test_a_missing_lcd_type_is_reported() {
        let err = machine()
            .load_config(&display_config("cs_pin: PA3\n"))
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'lcd_type' in section 'display' must be specified"
        );
    }

    #[test]
    fn test_an_unknown_lcd_type_is_a_choice_error() {
        let err = machine()
            .load_config(&display_config("lcd_type: made_up\n"))
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Choice 'made_up' for option 'lcd_type' in section 'display' is not a valid choice"
        );
    }

    #[test]
    fn test_a_panel_reports_its_missing_required_option() {
        // Every panel has a driver, so a section that omits an option the
        // driver needs stops in option validation rather than in the old "not
        // implemented" sentence.
        let err = machine()
            .load_config(&display_config(
                "lcd_type: uc1701\ncs_pin: PA3\nsclk_pin: PA1\nsid_pin: PC1\n",
            ))
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'a0_pin' in section 'display' must be specified"
        );

        // `hd44780_spi`'s required option is the shift register's latch line.
        let err = machine()
            .load_config(&display_config("lcd_type: hd44780_spi\n"))
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'latch_pin' in section 'display' must be specified"
        );

        // `emulated_st7920`'s are its software SPI bus and its enable line
        // (`st7920.py:211-222`).
        let err = machine()
            .load_config(&display_config("lcd_type: emulated_st7920\n"))
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'spi_software_miso_pin' in section 'display' must be specified"
        );
        let err = machine()
            .load_config(&display_config(
                "lcd_type: emulated_st7920\n\
                 spi_software_miso_pin: PB4\n\
                 spi_software_mosi_pin: PB5\n\
                 spi_software_sclk_pin: PB3\n",
            ))
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'en_pin' in section 'display' must be specified"
        );
    }

    /// The corpus's `lcd_type: uc1701` shape (`printer-creality-cr20-2018.cfg`),
    /// for the load that says every option of the section was read.
    const UC1701_SECTION: &str = "lcd_type: uc1701\n\
                                 cs_pin: PA3\n\
                                 a0_pin: PA5\n\
                                 encoder_pins: ^PC4, ^PC6\n\
                                 click_pin: ^!PC2\n";

    /// The corpus's `lcd_type: ssd1306` shape
    /// (`printer-wanhao-duplicator-6-2016.cfg`): the I2C panel.
    const SSD1306_SECTION: &str = "lcd_type: ssd1306\n\
                                  reset_pin: PE3\n\
                                  encoder_pins: ^PG1, ^PG0\n\
                                  click_pin: ^!PD2\n";

    /// The `lcd_type: sh1106` shape: the SSD1306's 132-column sibling, whose
    /// own option is `x_offset` (`uc1701.py:238-241`).
    const SH1106_SECTION: &str = "lcd_type: sh1106\n\
                                 x_offset: 2\n\
                                 contrast: 128\n\
                                 vcomh: 30\n\
                                 invert: True\n\
                                 encoder_pins: ^PG1, ^PG0\n\
                                 click_pin: ^!PD2\n";

    /// The `lcd_type: emulated_st7920` shape: a software SPI bus, the enable
    /// line, and the clock (`st7920.py:211-227`).
    const EMULATED_ST7920_SECTION: &str = "lcd_type: emulated_st7920\n\
                                          spi_software_miso_pin: PB4\n\
                                          spi_software_mosi_pin: PB5\n\
                                          spi_software_sclk_pin: PB3\n\
                                          spi_speed: 500000\n\
                                          en_pin: PC4\n\
                                          encoder_pins: ^PD2, ^PD3\n\
                                          click_pin: ^!PC0\n";

    #[test]
    fn test_each_panel_shape_loads_and_leaves_no_option_unread() {
        for section in [
            UC1701_SECTION,
            SSD1306_SECTION,
            SH1106_SECTION,
            EMULATED_ST7920_SECTION,
        ] {
            let printer = machine();
            printer
                .load_config(&display_config(section))
                .expect("the section loads and reads every option");
            let display = printer
                .lookup_object_as::<PrinterLCD>("display")
                .expect("the display is registered under its section id");
            assert_eq!(display.show_data_group(), "_default_16x4");
            assert_eq!(display.sent_message_count(), 0);
        }
    }

    #[test]
    fn test_an_unknown_display_group_is_reported() {
        let err = machine()
            .load_config(&display_config(&format!(
                "{ST7920_SECTION}display_group: _nope\n"
            )))
            .unwrap_err();

        assert_eq!(err.to_string(), "Unknown display_data group '_nope'");
    }

    #[test]
    fn test_a_bad_data_section_name_is_reported() {
        let section = ConfigSection::new("display_data", Some("_default_16x4"));
        let err = parse_data_item(&ConfigWrapper::untracked(&section))
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Section name 'display_data _default_16x4' is not valid"
        );
    }

    #[test]
    fn test_a_bad_position_is_reported() {
        let mut section = ConfigSection::new("display_data", Some("_default_16x4 extruder"));
        section
            .parameters
            .insert("position".to_string(), ConfigValue::Single("0".to_string()));
        let err = parse_data_item(&ConfigWrapper::untracked(&section))
            .map(|_| ())
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Unable to parse 'position' in section 'display_data _default_16x4 extruder'"
        );
    }

    /// `SET_DISPLAY_GROUP` both by name and, on the primary display, without a
    /// `DISPLAY` parameter (`display.py:207-214`).
    #[tokio::test(flavor = "multi_thread")]
    async fn test_set_display_group_switches_the_group() {
        let printer = machine();
        printer
            .load_config(&display_config(ST7920_SECTION))
            .expect("the section loads");
        printer.send_event(&KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the dispatcher");
        let display = printer
            .lookup_object_as::<PrinterLCD>("display")
            .expect("the display");

        gcode
            .run_script("SET_DISPLAY_GROUP DISPLAY=display GROUP=_default_20x4")
            .await
            .unwrap();
        assert_eq!(display.show_data_group(), "_default_20x4");

        // The unqualified form is the primary display's own registration.
        gcode
            .run_script("SET_DISPLAY_GROUP GROUP=_default_16x4")
            .await
            .unwrap();
        assert_eq!(display.show_data_group(), "_default_16x4");

        let err = gcode
            .run_script("SET_DISPLAY_GROUP GROUP=_nope")
            .await
            .unwrap_err();
        assert_eq!(err.to_string(), "Unknown display_data group '_nope'");
        assert_eq!(display.show_data_group(), "_default_16x4");
    }

    /// The ready handler runs on `klippy:ready`, so a load-only test never
    /// reaches the panel; this one drives the event by hand.
    #[test]
    fn test_ready_initialises_the_panel_and_starts_the_timer() {
        let printer = machine();
        printer
            .load_config(&display_config(ST7920_SECTION))
            .expect("the section loads");
        let display = printer
            .lookup_object_as::<PrinterLCD>("display")
            .expect("the display");
        // No MCU is connected, so the panel cannot write: the handler reports
        // it and stays quiet instead of failing the printer.
        assert_eq!(display.sent_message_count(), 0);

        printer.send_event(&KlippyEvent::KlippyReady);

        assert_eq!(display.sent_message_count(), 0);
        assert!(display.timer.lock().unwrap().is_some());
    }

    // -- against the fake firmware -----------------------------------------

    /// The AVR dictionary the corpus's ST7920 boards use (`atmega2560.dict`),
    /// when this build produced it.
    fn fake_dictionary() -> Option<std::path::PathBuf> {
        let dict = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        dict.is_file().then_some(dict)
    }

    /// The whole display against the dictionary-driven fake firmware.
    ///
    /// This is the test that says the panel really talks to the firmware — a
    /// display that loads and then writes nothing is a blank screen, not a
    /// working display — and that the three `display_status` commands exist
    /// even though this config has no `[display_status]` section.
    #[tokio::test(flavor = "multi_thread")]
    async fn test_the_panel_is_written_to_and_the_commands_work() {
        use crate::core::klippy::api::StartArgs;
        use crate::core::klippy::extras::display_status::DisplayStatus;
        use crate::core::klippy::printer::PrinterState;
        use crate::core::klippy::reactor::TokioReactor;

        let Some(dict) = fake_dictionary() else {
            return;
        };
        let text = format!(
            "[mcu]\ntest: dict={}\n[display]\n{ST7920_SECTION}",
            dict.display()
        );
        let config = Config::from_text(&text).expect("the config parses").0;
        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let printer = Arc::new(Printer::new(reactor));
        let mut start_args = StartArgs::collect(
            "display.cfg",
            None,
            crate::core::klippy::api::endpoints::info::NO_KLIPPER_PATH,
        );
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
            let gcode = printer
                .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                .expect("the dispatcher");
            let status = printer
                .lookup_object_as::<DisplayStatus>("display_status")
                .expect("the display created `display_status` on demand");

            // The panel was brought up at `klippy:ready`: the eight power-up
            // commands, then the first flush of the blank screen.
            let messages = display.sent_messages();
            gcode
                .run_script("M117 Printing")
                .await
                .map_err(|err| err.to_string())?;
            let message = status.get_status(0.0)["message"].clone();
            gcode
                .run_script("M73 P25")
                .await
                .map_err(|err| err.to_string())?;
            let progress = status.get_status(0.0)["progress"].clone();
            gcode
                .run_script("SET_DISPLAY_GROUP GROUP=_default_20x4")
                .await
                .map_err(|err| err.to_string())?;
            let group = display.show_data_group();
            Ok::<_, String>((messages, message, progress, group))
        }
        .await;

        printer.teardown();
        let (messages, message, progress, group) = outcome.expect("the display comes up");

        // The init sequence, as one `st7920_send_cmds` (`st7920.py:63-73`).
        assert_eq!(
            messages[0],
            SentMessage {
                is_data: false,
                bytes: vec![0x24, 0x40, 0x02, 0x26, 0x22, 0x02, 0x06, 0x0c],
            }
        );
        // And the first flush sent display data as well.
        assert!(
            messages.iter().any(|message| message.is_data),
            "the flush sends its framebuffer contents: {messages:?}"
        );

        assert_eq!(message, json!("Printing"));
        assert_eq!(progress, json!(0.25));
        assert_eq!(group, "_default_20x4");
    }

    // -- the render step ---------------------------------------------------

    /// A 20x4 text panel with a graphics layer: the render step's writes land
    /// in character cells the test can read back, the way a real panel's
    /// framebuffers hold them. A glyph name is one cell, marked `#` (a real
    /// panel draws the icon there).
    struct FakePanel {
        state: Mutex<FakePanelState>,
    }

    #[derive(Default)]
    struct FakePanelState {
        rows: Vec<Vec<u8>>,
        graphics: BTreeMap<(usize, usize), Vec<u8>>,
    }

    impl FakePanel {
        fn new() -> Arc<Self> {
            Arc::new(Self {
                state: Mutex::new(FakePanelState {
                    rows: vec![vec![b' '; 20]; 4],
                    ..FakePanelState::default()
                }),
            })
        }

        /// One row as the characters it holds.
        fn row(&self, row: usize) -> String {
            String::from_utf8(self.state.lock().unwrap().rows[row].clone())
                .expect("the frame is ASCII")
        }

        /// One graphics column as it was written.
        fn graphics(&self, row: usize, x: usize) -> Option<Vec<u8>> {
            self.state.lock().unwrap().graphics.get(&(row, x)).cloned()
        }
    }

    impl LcdChip for FakePanel {
        fn init(&self) {}

        fn clear(&self) {
            let mut state = self.state.lock().unwrap();
            for row in state.rows.iter_mut() {
                row.fill(b' ');
            }
            // A panel's `clear()` blanks its graphics too (`st7920.py:129-134`).
            state.graphics.clear();
        }

        fn flush(&self) {}

        fn get_dimensions(&self) -> (usize, usize) {
            (20, 4)
        }

        fn set_glyphs(&self, _glyphs: &BTreeMap<String, Glyph>) {}

        fn write_text(&self, x: usize, row: usize, text: &[u8]) {
            let mut state = self.state.lock().unwrap();
            let line = &mut state.rows[row];
            for (offset, byte) in text.iter().enumerate() {
                if let Some(cell) = line.get_mut(x + offset) {
                    *cell = *byte;
                }
            }
        }

        fn write_glyph(&self, x: usize, row: usize, _name: &str) -> usize {
            self.write_text(x, row, b"#");
            1
        }

        fn write_graphics(&self, x: usize, row: usize, data: &[u8]) {
            self.state
                .lock()
                .unwrap()
                .graphics
                .insert((row, x), data.to_vec());
        }

        fn sent_messages(&self) -> Vec<SentMessage> {
            Vec::new()
        }
    }

    /// A `display_data` item as the loader builds one.
    fn item(row: usize, col: usize, text: &str) -> DisplayDataItem {
        let identifier = format!("display_data _test r{row}c{col}");
        DisplayDataItem {
            row,
            col,
            identifier: identifier.clone(),
            text: text.to_string(),
            template: Template::parse(&format!("{identifier}:text"), text)
                .expect("the body parses"),
        }
    }

    fn group(items: Vec<DisplayDataItem>) -> DisplayGroup {
        DisplayGroup { items }
    }

    /// A machine with a `[display]` section and the two statuses the shipped
    /// layouts read unguarded: `gcode_move.speed_factor` and
    /// `toolhead.extruder` (the guarded ones — `extruder`, `fan`,
    /// `virtual_sdcard` — stay absent, so their items render empty).
    fn layout_machine() -> Arc<Printer> {
        layout_machine_with("")
    }

    /// [`layout_machine`] with `extra` appended to its config text, for a
    /// config that writes its own layout sections.
    fn layout_machine_with(extra: &str) -> Arc<Printer> {
        struct Stub(Value);
        impl PrinterObject for Stub {
            fn get_status(&self, _eventtime: f64) -> Value {
                self.0.clone()
            }
        }
        let printer = machine();
        let text = format!("[mcu]\nserial: /dev/a\n[display]\n{ST7920_SECTION}{extra}");
        printer
            .load_config(&Config::from_text(&text).expect("the config parses").0)
            .expect("the section loads");
        printer
            .add_object("gcode_move", Arc::new(Stub(json!({ "speed_factor": 1.0 }))))
            .expect("the name is free");
        printer
            .add_object(
                "toolhead",
                Arc::new(Stub(json!({
                    "extruder": "extruder",
                    "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
                    "estimated_print_time": 0.0,
                }))),
            )
            .expect("the name is free");
        printer
    }

    /// The layout machine's display and the panel a frame is drawn into.
    fn layout_panel(printer: &Arc<Printer>) -> (Arc<DisplayTemplates>, Arc<FakePanel>) {
        let display = printer
            .lookup_object_as::<PrinterLCD>("display")
            .expect("the display");
        (Arc::clone(display.display_templates()), FakePanel::new())
    }

    #[test]
    fn test_a_group_is_rendered_into_the_panel_at_its_positions() {
        // The render step end to end: each item's `text` is rendered (with its
        // `{% set %}`), its `~glyph~` runs become cells, its newlines are
        // dropped, and a write past the edge is cut there.
        let printer = machine();
        let templates = Arc::new(DisplayTemplates::default());
        let panel = FakePanel::new();
        let chip: Arc<dyn LcdChip> = Arc::clone(&panel) as Arc<dyn LcdChip>;
        let group = group(vec![
            item(0, 0, "{% set who = \"world\" %}hi { who }"),
            item(1, 2, "~right_arrow~ { 2 + 3 }"),
            item(2, 0, "one\ntwo"),
            item(3, 19, "abc"),
        ]);

        show_group(&chip, &group, &templates, &printer).expect("the group renders");

        assert_eq!(panel.row(0), "hi world            ");
        assert_eq!(panel.row(1), "  # 5               ");
        assert_eq!(panel.row(2), "onetwo              ");
        assert_eq!(panel.row(3), "                   a");
    }

    #[test]
    fn test_a_group_reads_its_statuses_at_render_time() {
        // The frame follows the data: the same group drawn twice around a
        // changed status is two different frames.
        struct Shared(Arc<Mutex<Value>>);
        impl PrinterObject for Shared {
            fn get_status(&self, _eventtime: f64) -> Value {
                self.0.lock().unwrap().clone()
            }
        }
        let printer = machine();
        let speed = Arc::new(Mutex::new(json!({ "speed_factor": 1.0 })));
        printer
            .add_object("gcode_move", Arc::new(Shared(Arc::clone(&speed))))
            .expect("the name is free");
        let templates = Arc::new(DisplayTemplates::default());
        let panel = FakePanel::new();
        let chip: Arc<dyn LcdChip> = Arc::clone(&panel) as Arc<dyn LcdChip>;
        let group = group(vec![item(
            1,
            0,
            "~feedrate~\n  { \"{:^4.0%}\".format(printer.gcode_move.speed_factor) }",
        )]);

        show_group(&chip, &group, &templates, &printer).expect("the group renders");
        assert_eq!(panel.row(1), "#  100%             ");

        *speed.lock().unwrap() = json!({ "speed_factor": 0.5 });
        panel.clear();
        show_group(&chip, &group, &templates, &printer).expect("the group renders again");
        assert_eq!(panel.row(1), "#  50%              ");
    }

    #[test]
    fn test_a_failing_item_keeps_the_frame_drawn_so_far() {
        let printer = machine();
        let templates = Arc::new(DisplayTemplates::default());
        let panel = FakePanel::new();
        let chip: Arc<dyn LcdChip> = Arc::clone(&panel) as Arc<dyn LcdChip>;
        let group = group(vec![
            item(0, 0, "first"),
            item(1, 0, "{ nope }"),
            item(2, 0, "third"),
        ]);

        let error = show_group(&chip, &group, &templates, &printer).expect_err("the item fails");

        // The frame stops at the failing item — nothing after it is drawn — and
        // what came before stays, which is what upstream's `except` does
        // (`display.py:234-237`). The error names the item's own template.
        assert!(error.to_string().contains("undefined value"), "{error}");
        assert!(
            error.to_string().contains("display_data _test r1c0:text"),
            "{error}"
        );
        assert_eq!(panel.row(0), "first               ");
        assert_eq!(panel.row(1), "                    ");
        assert_eq!(panel.row(2), "                    ");
    }

    #[test]
    fn test_a_render_call_is_checked_against_the_templates_parameters() {
        // `render()` is the display's own callable: an undeclared keyword is
        // upstream's `Invalid parameter` and an unknown name is the `KeyError`
        // its lookup raises.
        let printer = layout_machine();
        let (templates, panel) = layout_panel(&printer);
        let chip: Arc<dyn LcdChip> = Arc::clone(&panel) as Arc<dyn LcdChip>;

        let refused = group(vec![item(
            0,
            0,
            "{ render('_heater_temperature', nope=1) }",
        )]);
        let error = show_group(&chip, &refused, &templates, &printer).expect_err("the call fails");
        assert!(
            error
                .to_string()
                .contains("Invalid parameter to display_template _heater_temperature"),
            "{error}"
        );

        let missing = group(vec![item(0, 0, "{ render('_nope') }")]);
        let error = show_group(&chip, &missing, &templates, &printer).expect_err("the call fails");
        assert!(error.to_string().contains("KeyError: '_nope'"), "{error}");
    }

    #[test]
    fn test_draw_progress_bar_paints_eight_bits_per_cell() {
        let panel = FakePanel::new();
        let chip: Arc<dyn LcdChip> = Arc::clone(&panel) as Arc<dyn LcdChip>;

        // Half full over four columns is 32 bits, `int(32 * .5 + .5)` = 16 of
        // them empty: the two rightmost columns are solid, the two leftmost
        // hold only the bar's frame bits (the bottom one).
        draw_progress_bar(&*chip, 2, 1, 4, 0.5);

        assert_eq!(panel.graphics(2, 4), Some(column(0x01)));
        assert_eq!(panel.graphics(2, 3), Some(column(0x00)));
        assert_eq!(panel.graphics(2, 2), Some(column(0xff)));
        assert_eq!(panel.graphics(2, 1), Some(column(0xff)));
        assert_eq!(panel.graphics(2, 0), None);
    }

    /// One graphics column: a solid row, the bar's fourteen pixel rows, and a
    /// solid row (`display.py:263`).
    fn column(byte: u8) -> Vec<u8> {
        let mut data = vec![0xff];
        data.extend(std::iter::repeat(byte).take(14));
        data.push(0xff);
        data
    }

    // -- a config's own layout sections -------------------------------------

    #[test]
    fn test_a_main_config_data_item_joins_its_group_and_renders() {
        // A config's own `[display_data <group> <item>]` section is a layout
        // source: the item joins the group the section names, it is ordered by
        // its position with the shipped items, and the frame shows it.
        let printer = layout_machine_with(
            "\n\
             [display_data _default_16x4 site]\n\
             position: 0, 0\n\
             text: hello\n",
        );
        let display = printer
            .lookup_object_as::<PrinterLCD>("display")
            .expect("the display");
        let group = &display.display_data_groups()["_default_16x4"];
        assert_eq!(group.items().len(), 9);
        // The shipped `[display_data _default_16x4 extruder]` is also at (0, 0),
        // and the tie is broken by the section identifier.
        assert_eq!(
            group.items()[1].identifier,
            "display_data _default_16x4 site"
        );

        let (templates, panel) = layout_panel(&printer);
        let chip: Arc<dyn LcdChip> = Arc::clone(&panel) as Arc<dyn LcdChip>;
        show_group(&chip, group, &templates, &printer).expect("the group renders");
        assert_eq!(panel.row(0), "hello               ");
    }

    #[test]
    fn test_a_main_config_data_section_creates_its_group() {
        // A group the shipped file does not have is created from the config's
        // sections (`groups.setdefault(...)`, `display.py:139-145`), and
        // `display_group` may select it.
        let printer = layout_machine_with(
            "display_group: _site\n\
             [display_data _site bottom]\n\
             position: 3, 2\n\
             text: B\n\
             [display_data _site top]\n\
             position: 1, 0\n\
             text: A\n",
        );
        let display = printer
            .lookup_object_as::<PrinterLCD>("display")
            .expect("the display");
        assert_eq!(display.show_data_group(), "_site");
        let group = &display.display_data_groups()["_site"];
        assert_eq!(
            group
                .items()
                .iter()
                .map(|item| (item.row, item.col, item.text.as_str()))
                .collect::<Vec<_>>(),
            [(1, 0, "A"), (3, 2, "B")]
        );

        let (templates, panel) = layout_panel(&printer);
        let chip: Arc<dyn LcdChip> = Arc::clone(&panel) as Arc<dyn LcdChip>;
        show_group(&chip, group, &templates, &printer).expect("the group renders");
        assert_eq!(
            [panel.row(0), panel.row(1), panel.row(2), panel.row(3)],
            [
                "                    ",
                "A                   ",
                "                    ",
                "  B                 ",
            ]
        );
    }

    #[test]
    fn test_a_main_config_data_item_replaces_the_shipped_one() {
        // The same section identifier as a shipped item: upstream merges by
        // name (`display.py:126-128`), so the config's item replaces it — it
        // neither joins the group beside it nor adds a second one.
        let printer = layout_machine_with(
            "\n\
             [display_data _default_16x4 fan]\n\
             position: 0, 10\n\
             text: cfgfan\n",
        );
        let display = printer
            .lookup_object_as::<PrinterLCD>("display")
            .expect("the display");
        let group = &display.display_data_groups()["_default_16x4"];
        assert_eq!(group.items().len(), 8);
        assert_eq!(
            group.items()[1].identifier,
            "display_data _default_16x4 fan"
        );
        assert_eq!(group.items()[1].text, "cfgfan");

        let (templates, panel) = layout_panel(&printer);
        let chip: Arc<dyn LcdChip> = Arc::clone(&panel) as Arc<dyn LcdChip>;
        show_group(&chip, group, &templates, &printer).expect("the group renders");
        assert_eq!(panel.row(0), "          cfgfan    ");
    }

    /// `value`'s lines, indented as a config value's continuation lines.
    fn indented(value: &str) -> String {
        value
            .lines()
            .map(|line| format!("  {line}"))
            .collect::<Vec<_>>()
            .join("\n")
    }

    #[test]
    fn test_a_main_config_glyph_section_is_parsed() {
        // `data` is the 16x16 icon, split into its high and low bytes, and
        // `hd44780_data` + `hd44780_slot` the 5x8 one (`display.py:147-166`);
        // a config's own `[display_glyph <name>]` section may write both.
        let wide = vec![".............*.."; 16].join("\n");
        let narrow = vec!["**..."; 8].join("\n");
        let text = format!(
            "[mcu]\nserial: /dev/a\n[display]\n{ST7920_SECTION}\
\
             [display_glyph site]\ndata:\n{}\nhd44780_data:\n{}\nhd44780_slot: 6\n",
            indented(&wide),
            indented(&narrow),
        );
        let config = Config::from_text(&text).expect("the config parses").0;
        let section = config.get_section("display").expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::with_config(section, access, None, &config);

        let glyphs = load_display_glyphs(&wrapper).expect("the glyphs load");
        // Bit 2 is set in every row of the wide glyph, bits 4 and 3 of the
        // narrow one.
        assert_eq!(
            glyphs["site"].icon16x16,
            Some((vec![0x00; 16], vec![0x04; 16]))
        );
        assert_eq!(glyphs["site"].icon5x8, Some((6, vec![0x18; 8])));
        // The shipped glyphs are still there, beside the config's own one.
        assert!(glyphs.contains_key("extruder"));
        assert!(glyphs.contains_key("bed"));
    }

    #[test]
    fn test_the_layout_sections_are_claimed_by_factories() {
        // The factory table is the loader's schema: a `[display_data]` or
        // `[display_glyph]` section is a section this host knows exactly when
        // its own factory claims it, and the factory reads the section (the
        // load ends in the undefined-option check, so `Ok` is the proof).
        assert!(crate::core::klippy::load::known_section_ids().contains(&"display_data"));
        assert!(crate::core::klippy::load::known_section_ids().contains(&"display_glyph"));

        let printer = machine();
        let text = format!(
            "[mcu]\nserial: /dev/a\n[display]\n{ST7920_SECTION}\
\
             [display_data _default_16x4 site]\nposition: 0, 0\ntext: hello\n\
             [display_glyph site]\ndata:\n{}\n",
            indented(&vec![".............*.."; 16].join("\n")),
        );
        printer
            .load_config(&Config::from_text(&text).expect("the config parses").0)
            .expect("the layout sections load");

        // Each section is claimed under its own identifier, but a layout piece
        // is not a printer part: it stays out of `objects/list`.
        for identifier in ["display_data _default_16x4 site", "display_glyph site"] {
            let claimed = printer
                .lookup_object(identifier)
                .expect("the section is claimed");
            assert!(!claimed.is_queryable());
        }
    }

    #[test]
    fn test_the_shipped_layouts_render_a_frame() {
        // The three `display_data` groups the shipped `display.cfg` defines,
        // each rendered into a panel with `gcode_move.speed_factor` at 1.0 and
        // nothing else running (`display_status` at 0%, `idle_timeout` at
        // `00:00`, `Ready`), and with the two guarded items — `extruder`/`fan`
        // — absent. Each row is the characters the panel would show, in order.
        let printer = layout_machine();
        let (templates, panel) = layout_panel(&printer);
        let display = printer
            .lookup_object_as::<PrinterLCD>("display")
            .expect("the display");
        let chip: Arc<dyn LcdChip> = Arc::clone(&panel) as Arc<dyn LcdChip>;

        // `_default_16x4`: the feedrate glyph and its speed on row 1, the
        // progress text and the clock on row 2, the status line on row 3. The
        // progress bar is ten graphics columns on row 2, not text.
        panel.clear();
        let group = &display.display_data_groups()["_default_16x4"];
        show_group(&chip, group, &templates, &printer).expect("_default_16x4");
        assert_eq!(
            [panel.row(0), panel.row(1), panel.row(2), panel.row(3)],
            [
                "                    ",
                "          #100%     ",
                "    0%     00:00    ",
                "Ready               ",
            ]
        );
        assert_eq!(graphics_columns(&panel), row_columns(2, 0, 10));

        // `_multiextruder_16x4`: the same rows with the second extruder and a
        // four-character progress field, and its bar six columns wide on row 1.
        panel.clear();
        let group = &display.display_data_groups()["_multiextruder_16x4"];
        show_group(&chip, group, &templates, &printer).expect("_multiextruder_16x4");
        assert_eq!(
            [panel.row(0), panel.row(1), panel.row(2), panel.row(3)],
            [
                "                    ",
                "            0%      ",
                "           00:00    ",
                "Ready               ",
            ]
        );
        assert_eq!(graphics_columns(&panel), row_columns(1, 10, 6));

        // `_default_20x4`: four rows of items at two columns each — the
        // feedrate, the progress and the clock all carry their glyph — and no
        // progress bar, so no graphics at all.
        panel.clear();
        let group = &display.display_data_groups()["_default_20x4"];
        show_group(&chip, group, &templates, &printer).expect("_default_20x4");
        assert_eq!(
            [panel.row(0), panel.row(1), panel.row(2), panel.row(3)],
            [
                "                    ",
                "                    ",
                "#100%   # 0%  #00:00",
                "Ready               ",
            ]
        );
        assert_eq!(graphics_columns(&panel), Vec::new());
    }

    /// Every graphics column a panel was written, in `(row, x)` order.
    fn graphics_columns(panel: &FakePanel) -> Vec<(usize, usize)> {
        let mut columns: Vec<(usize, usize)> = panel
            .state
            .lock()
            .unwrap()
            .graphics
            .keys()
            .copied()
            .collect();
        columns.sort();
        columns
    }

    /// The `width` columns starting at (`row`, `col`), as [`graphics_columns`]
    /// reports them.
    fn row_columns(row: usize, col: usize, width: usize) -> Vec<(usize, usize)> {
        (col..col + width).map(|x| (row, x)).collect()
    }

    // -- the menu's frame ---------------------------------------------------

    /// The layout machine plus a small menu the panel can paint: a list whose
    /// first row is its `..`, then a command and a container.
    fn menu_machine() -> Arc<Printer> {
        struct Stub(Value);
        impl PrinterObject for Stub {
            fn get_status(&self, _eventtime: f64) -> Value {
                self.0.clone()
            }
        }
        let text = format!(
            "[mcu]\nserial: /dev/a\n[display]\n{ST7920_SECTION}menu_root: __test\n\
             [menu __test]\n\
             type: list\n\
             name: Test\n\
             [menu __test __a]\n\
             type: command\n\
             name: A\n\
             gcode: M117 a\n\
             [menu __test __sub]\n\
             type: list\n\
             name: Sub\n"
        );
        let config = Config::from_text(&text).expect("the config parses").0;
        let printer = machine();
        printer.load_config(&config).expect("the config loads");
        printer
            .add_object("gcode_move", Arc::new(Stub(json!({ "speed_factor": 1.0 }))))
            .expect("the name is free");
        printer
            .add_object(
                "toolhead",
                Arc::new(Stub(json!({
                    "extruder": "extruder",
                    "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
                    "estimated_print_time": 0.0,
                }))),
            )
            .expect("the name is free");
        printer
    }

    /// One refresh of `display` into `panel`, as the four rows it holds after.
    fn frame(display: &Arc<PrinterLCD>, panel: &Arc<FakePanel>, eventtime: f64) -> Vec<String> {
        let chip: Arc<dyn LcdChip> = Arc::clone(panel) as Arc<dyn LcdChip>;
        display
            .draw_screen(&chip, eventtime)
            .expect("a refresh always asks for the next frame");
        (0..4).map(|row| panel.row(row)).collect()
    }

    /// A frame's rows as hex, two digits a byte, so they are pinned byte for
    /// byte.
    fn hex_frame(frame: &[String]) -> Vec<String> {
        frame
            .iter()
            .map(|row| row.bytes().map(|byte| format!("{byte:02x}")).collect())
            .collect()
    }

    /// The open menu's frame: every row of the container is painted from
    /// column 0 — the `..` row first with the cursor on it, the names padded to
    /// the panel, the container's `>` in the last cell — and a `down` moves the
    /// cursor, the next frame following it.
    #[test]
    fn test_the_open_menu_paints_its_rows_and_moves_the_cursor() {
        let printer = menu_machine();
        let display = printer
            .lookup_object_as::<PrinterLCD>("display")
            .expect("the display");
        let menu = display.menu().expect("the primary display owns the menu");
        let panel = FakePanel::new();

        menu.begin(0.0);
        let open = hex_frame(&frame(&display, &panel, 1.0));
        assert_eq!(
            open,
            [
                // ">..                 " — cursor on the `..` row
                "3e2e2e2020202020202020202020202020202020".to_string(),
                // " A                  "
                "2041202020202020202020202020202020202020".to_string(),
                // " Sub           >    " — the container's marker, last
                // cell of the 16-column row
                "2053756220202020202020202020203e20202020".to_string(),
                // the row past the container is blank
                "2020202020202020202020202020202020202020".to_string(),
            ]
        );

        // `down` selects `A`: the row above loses the cursor, the row below
        // gains it, and nothing else changes.
        menu.down(false);
        let moved = hex_frame(&frame(&display, &panel, 2.0));
        assert_eq!(
            moved,
            [
                // " ..                 "
                "202e2e2020202020202020202020202020202020".to_string(),
                // ">A                  "
                "3e41202020202020202020202020202020202020".to_string(),
                // " Sub           >    "
                "2053756220202020202020202020203e20202020".to_string(),
                "2020202020202020202020202020202020202020".to_string(),
            ]
        );
    }

    /// Closing the menu hands the frame back to the active `display_data`
    /// group: the next refresh is the `_default_16x4` layout, not the menu's
    /// rows.
    #[test]
    fn test_closing_the_menu_returns_the_panel_to_the_display_group() {
        let printer = menu_machine();
        let display = printer
            .lookup_object_as::<PrinterLCD>("display")
            .expect("the display");
        let menu = display.menu().expect("the primary display owns the menu");
        let panel = FakePanel::new();

        menu.begin(0.0);
        let open = frame(&display, &panel, 1.0);
        menu.exit(true);
        let closed = hex_frame(&frame(&display, &panel, 2.0));

        assert_ne!(closed, hex_frame(&open), "the menu's frame is gone");
        assert_eq!(
            closed,
            [
                "2020202020202020202020202020202020202020".to_string(),
                // "          #100%     " — the feedrate glyph and its speed
                "2020202020202020202023313030252020202020".to_string(),
                // "    0%     00:00    " — the progress and the clock
                "202020203025202020202030303a303020202020".to_string(),
                // "Ready               "
                "5265616479202020202020202020202020202020".to_string(),
            ]
        );
    }
}
