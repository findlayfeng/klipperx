//! `[display]` — the LCD framework: which panel, which layout, and the timer
//! that keeps the panel written (upstream `klippy/extras/display/display.py`).
//!
//! One `[display]` section builds one [`PrinterLCD`]:
//!
//! | step | here | upstream |
//! |---|---|---|
//! | the panel | `lcd_type` picks a [`LcdChip`] | `display.py:16-22`, `:181` |
//! | `[display_status]` | created on demand, whether or not the config names it | `display.py:188` |
//! | the menu keys | the options are read, no menu is built | `menu.py:688-722`, `menu_keys.py:12-43` |
//! | templates | the shipped `display.cfg` is merged into the `display_template` registry | `display.py:115-166` |
//! | layouts | `display_data` groups | `display.py:56-88` |
//! | glyphs | `display_glyph` sections | `display.py:103-114` |
//! | the group | `display_group`, defaulted from the panel's width | `display.py:194-200` |
//! | refresh | `klippy:ready` initialises the panel and starts the timer | `display.py:217-241` |
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
//! Merging keeps upstream's rule, including the part that only matters once the
//! main config can name these sections: a section whose identifier the main
//! config already defines is not loaded from the shipped file.
//!
//! # What is not here
//!
//! * **Rendering and the menu.** See the parent module's docs; the panel is
//!   cleared and flushed every `REDRAW_TIME`, and nothing is drawn.
//! * **`draw_text` / `draw_progress_bar`** (`display.py:247-264`): they are the
//!   drawing half of a render step that does not run.
//! * **The `display_template` parameter check.** Upstream raises `Invalid
//!   parameter to display_template <name>` from inside `DisplayTemplate.render`
//!   (`display.py:45-53`); the rule is [`check_render_params`], and nothing
//!   calls it yet because nothing renders.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::section::ConfigSection;
use crate::core::klippy::config::{Config, ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::display_status::ensure as ensure_display_status;
use crate::core::klippy::extras::led::{
    load_display_template, DisplayTemplates, DISPLAY_TEMPLATES_OBJECT,
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
use super::ssd1306::Ssd1306;
use super::st7920::ST7920;
use super::uc1701::Uc1701;

// The framework reads `[display_status]` (created on demand) and the
// `[display_template]` registry sits at order 40; `[spi_device]`'s resources
// come at 50 and a panel driver only needs plain pins, so 45 puts `[display]`
// after everything it reads and before the bus consumers.
section!("display", order = 45, load = load_config);

/// Normal time between each screen redraw (`display.py:12`).
const REDRAW_TIME: f64 = 0.500;
/// Minimum time between screen redraws (`display.py:14`).
const REDRAW_MIN_TIME: f64 = 0.100;

/// The panel names `lcd_type` accepts — upstream's `LCD_chips` keys
/// (`display.py:16-22`).
///
/// The whole list is accepted even though only `st7920`, `hd44780`,
/// `hd44780_spi`, `uc1701`, `ssd1306` and `aip31068_spi` have drivers here, so
/// the choice error is upstream's wording and the names are not silently
/// rejected.
const LCD_TYPES: &[&str] = &[
    "st7920",
    "emulated_st7920",
    "hd44780",
    "uc1701",
    "ssd1306",
    "sh1106",
    "hd44780_spi",
    "aip31068_spi",
];

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
/// depends on that), and take the glyphs it may draw.
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
    /// (`display.py:105-107`).
    pub icon16x16: Option<(Vec<u8>, Vec<u8>)>,
    /// The HD44780 CGRAM slot and the 5x8 icon (`display.py:109-113`).
    pub icon5x8: Option<(u8, Vec<u8>)>,
}

// ===========================================================================
// Layouts
// ===========================================================================

/// One `[display_data <group> <item>]` row (`display.py:57-77`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DisplayDataItem {
    /// The screen row.
    pub row: usize,
    /// The screen column.
    pub col: usize,
    /// The section identifier, e.g. `display_data _default_16x4 extruder`.
    pub identifier: String,
    /// The item's `text` option as written (its template source).
    pub text: String,
}

/// One `display_data` group: its items, ordered by screen position
/// (`display.py:68-77`).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
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
    /// A missing or unknown `lcd_type`, an unimplemented panel, a missing
    /// option in any of the sections the display reads, or an unknown
    /// `display_group`.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        // The panel first: upstream builds it before anything else
        // (`display.py:181`), and its width decides the default group.
        let lcd_type = config.get_choice("lcd_type", LCD_TYPES, None)?;
        let lcd_chip: Arc<dyn LcdChip> = match lcd_type.as_str() {
            "st7920" => Arc::new(ST7920::new(config, printer)?),
            "hd44780" => Arc::new(Hd44780::new(config, printer)?),
            "hd44780_spi" => Arc::new(Hd44780Spi::new(config, printer)?),
            "uc1701" => Arc::new(Uc1701::new(config, printer)?),
            "ssd1306" => Arc::new(Ssd1306::new(config, printer)?),
            "aip31068_spi" => Arc::new(Aip31068Spi::new(config, printer)?),
            other => {
                return Err(ConfigError::new(format!(
                    "lcd_type '{other}' is not implemented in this host"
                )))
            }
        };

        // The `[display]` section is the only section that registers a display
        // (`display.py:185-187`); the menu-key options below are therefore
        // always read.
        let _menu = read_menu_options(config)?;

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
            reactor: printer.reactor(),
            timer: Mutex::new(None),
            redraw_request_pending: AtomicBool::new(false),
            redraw_time: Mutex::new(0.0),
            self_ref: weak.clone(),
        });

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

    /// One refresh (`display.py:222-241`).
    ///
    /// Upstream renders the active group here and swallows whatever the render
    /// raises; this host has no renderer, so the panel is blanked and flushed
    /// and the group's items are left in [`PrinterLCD::display_data_groups`].
    fn screen_update_event(&self, eventtime: f64) -> Option<f64> {
        if self.redraw_request_pending.swap(false, Ordering::SeqCst) {
            *self.redraw_time.lock().unwrap_or_else(|p| p.into_inner()) =
                eventtime + REDRAW_MIN_TIME;
        }
        self.lcd_chip.clear();
        self.lcd_chip.flush();
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

/// What the menu options were read into. Nothing consumes them: the menu is not
/// built (module docs), so the values only prove the section was read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MenuOptions {
    /// `menu_timeout` (`menu.py:703`).
    pub timeout: i64,
    /// `menu_reverse_navigation` (`menu.py:705-707`).
    pub reverse_navigation: bool,
}

/// Read the options the menu would consume.
///
/// Upstream reads them in `MenuManager.__init__` (`menu.py:689-722`) and
/// `MenuKeys.__init__` (`menu_keys.py:13-43`); with no menu here, they are read
/// because `check_unused` requires a reader for every option a config writes —
/// `menu_timeout` and `menu_reverse_navigation` are in the corpus.
///
/// The pins are read but never resolved: the buttons registry they would be
/// registered with takes callbacks for a menu that does not exist, and a menu
/// key does nothing on its own.
///
/// # Errors
/// Upstream's wording: an unparseable `encoder_pins` (`Unable to parse
/// encoder_pins`, `menu_keys.py:22-26`), a bad `encoder_steps_per_detent`
/// choice, a non-positive `encoder_fast_rate`, or a malformed
/// `analog_range_*` pair.
pub fn read_menu_options(config: &ConfigWrapper) -> Result<MenuOptions, ConfigError> {
    let identifier = config.identifier();
    // `MenuManager.__init__` (`menu.py:689-722`).
    let _menu_root = config.get("menu_root", Some("__main"))?;
    let timeout = config.get_int("menu_timeout", Some(0))?;
    let reverse_navigation = config.get_bool("menu_reverse_navigation", Some(false))?;

    // `MenuKeys.__init__` (`menu_keys.py:13-43`).
    if let Some(pins) = config.get_str("encoder_pins") {
        if pins.split(',').count() != 2 {
            return Err(ConfigError::new("Unable to parse encoder_pins".to_string()));
        }
    }
    config.get_choice("encoder_steps_per_detent", &["2", "4"], Some("4"))?;
    config.get_float_bounded(
        "encoder_fast_rate",
        Some(0.030),
        None,
        None,
        Some(0.0),
        None,
    )?;

    // `MenuKeys.register_button` (`menu_keys.py:45-61`): the button pins, each
    // optionally paired with an analog range and a pull-up resistor.
    let mut analog = false;
    for option in ["click_pin", "back_pin", "up_pin", "down_pin", "kill_pin"] {
        if config.get_str(option).is_none() {
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
    Ok((
        group,
        DisplayDataItem {
            row,
            col,
            identifier,
            text,
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
/// already declares. That rule is user-visible text, so it is kept here even
/// though nothing renders yet — the caller is the render step that lands with
/// the template engine.
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
                reverse_navigation: true
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
                "error_mcu",
                "mcu",
                "display_status",
                // The shipped layout's `display_template` sections bring the
                // registry up (`display/display.py:168-173`), before the
                // display itself is registered.
                "display_template",
                "display"
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
        // The panels that have a driver stop in option validation when a
        // required option is absent, instead of the old "not implemented"
        // sentence — that wording now covers only the siblings below.
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
    }

    #[test]
    fn test_the_sibling_panels_that_still_have_no_driver_are_reported() {
        // `sh1106` is the SSD1306's own variant (`uc1701.py:238-241`): the
        // name is accepted by `lcd_type` and refused with the same sentence
        // as every other gap. `aip31068_spi` left this list when its driver
        // landed; it is now built like the others.
        for lcd_type in ["sh1106", "emulated_st7920"] {
            let err = machine()
                .load_config(&display_config(&format!("lcd_type: {lcd_type}\n")))
                .unwrap_err();
            assert_eq!(
                err.to_string(),
                format!("lcd_type '{lcd_type}' is not implemented in this host")
            );
        }
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

    #[test]
    fn test_the_two_new_panels_load_and_leave_no_option_unread() {
        for section in [UC1701_SECTION, SSD1306_SECTION] {
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
}
