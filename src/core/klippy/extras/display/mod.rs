//! The `[display]` family: the LCD framework and the panel drivers it drives
//! (upstream `klippy/extras/display/`).
//!
//! Upstream keeps this feature in a package because the pieces are separate
//! files there — `display.py` (the framework), one module per panel
//! (`st7920.py`, `hd44780.py`, `uc1701.py`, …), `menu.py`/`menu_keys.py` (the
//! on-screen menu), and two shipped layouts (`display.cfg`, `menu.cfg`). Here
//! the same split is kept: [`display`] is the framework, [`st7920`],
//! [`hd44780`], [`hd44780_spi`], [`uc1701`], [`ssd1306`] and [`aip31068_spi`]
//! are the panels that
//! have a driver here, and the shipped layout is [`display::DISPLAY_CFG`],
//! vendored beside them.
//!
//! | section | upstream | notes |
//! |---|---|---|
//! | `[display]` | `display/display.py:176` `load_config` | builds the one panel and draws the layout; the panel is chosen by `lcd_type` |
//! | `[display_template <name>]` | `display/display.py:168` (no file of its own) | lives in [`led`](super::led) |
//! | `[display_data <group> <item>]` | `display/display.py:56` | claimed by a factory of its own; the groups are assembled by `[display]` from the config's items and the shipped `display.cfg`, the config's winning by name |
//! | `[display_glyph <name>]` | `display/display.py:103` | claimed by a factory of its own; merged with the shipped glyphs the same way |
//! | `[menu <name>]` | `display/menu.py:1035` | read by [`menu::MenuManager`]; shipped `menu.cfg` first, then the main config |
//!
//! # What is not here
//!
//! * **The menu's rows are computed, not painted.** [`menu`] ports `menu.py` +
//!   `menu_keys.py` and reads the shipped `menu.cfg`; a config's `[menu <name>]`
//!   sections are read in place (no section factory of their own — the manager
//!   reads its siblings, as upstream's `get_prefix_sections` does). The rows it
//!   lays out are handed back by `MenuManager::screen_rows`, not drawn — the
//!   framework's [`display::draw_text`]/[`display::draw_progress_bar`] is what a
//!   later unit paints them with; the key buttons register through `buttons`,
//!   whose registry has no rotary encoder here.
//! * **The other panels' drawing.** `st7920`, `hd44780`, `hd44780_spi`,
//!   `uc1701`, `ssd1306` and `aip31068_spi` have drivers; `sh1106` and
//!   `emulated_st7920` are accepted by `lcd_type` (so its error wording stays
//!   upstream's) and refused with a message naming the gap. Of the six, only
//!   [`hd44780`] draws: the font tables the others write glyphs from
//!   (`font8x14.VGA_FONT`, `st7920.CharGlyphs`, `uc1701.CharGlyphs`) are a
//!   later unit, so their [`display::LcdChip::write_text`] and friends are the
//!   draw-nothing defaults and their panels stay blank while the layout renders.

pub mod aip31068_spi;
pub mod display;
pub mod hd44780;
pub mod hd44780_spi;
pub mod menu;
pub mod ssd1306;
pub mod st7920;
pub mod uc1701;
