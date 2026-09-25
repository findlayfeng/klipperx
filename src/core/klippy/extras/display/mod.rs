//! The `[display]` family: the LCD framework and the panel drivers it drives
//! (upstream `klippy/extras/display/`).
//!
//! Upstream keeps this feature in a package because the pieces are separate
//! files there — `display.py` (the framework), one module per panel
//! (`st7920.py`, `hd44780.py`, `uc1701.py`, …), `menu.py`/`menu_keys.py` (the
//! on-screen menu), and two shipped layouts (`display.cfg`, `menu.cfg`). Here
//! the same split is kept: [`display`] is the framework, [`st7920`] and
//! [`hd44780`] are the panels that have a driver here, and the shipped layout
//! is [`display::DISPLAY_CFG`], vendored beside them.
//!
//! | section | upstream | notes |
//! |---|---|---|
//! | `[display]` | `display/display.py:176` `load_config` | the only section; the panel is chosen by `lcd_type` |
//! | `[display_template <name>]` | `display/display.py:168` (no file of its own) | lives in [`led`](super::led) |
//! | `[display_data <group> <item>]` | `display/display.py:56` | shipped `display.cfg` only |
//! | `[display_glyph <name>]` | `display/display.py:93` | shipped `display.cfg` only |
//!
//! # What is not here
//!
//! * **Rendering.** Templates, layouts and glyphs are parsed and stored, but no
//!   template is evaluated and nothing is drawn onto a panel: `display.cfg`
//!   needs `{% set %}` (supported since template batch #9), plus `|abs`,
//!   `"%3.0f" % …` and `.format`, which the template engine
//!   (`super::template`) does not have yet. A refresh clears the panel and
//!   flushes the framebuffer differences, exactly as upstream does around its
//!   own (swallowed) render step (`display/display.py:236-240`), so the screen
//!   stays blank rather than the printer failing to come up.
//! * **The menu.** `menu.cfg` is not loaded: no `menu` object, no `menu:*`
//!   events, no menu drawing. The menu options a config writes are still read
//!   (see [`display::PrinterLCD`]), because `check_unused` requires a reader.
//! * **The other panels.** Only `st7920` and `hd44780` have drivers; `uc1701`,
//!   `ssd1306`, `sh1106`, `hd44780_spi`, `aip31068_spi` and `emulated_st7920`
//!   are accepted by `lcd_type` (so its error wording stays upstream's) and
//!   refused with a message naming the gap.
//! * **`[display_data]`/`[display_glyph]` in the main config.** Only the shipped
//!   `display.cfg` is read; a main config's own layouts and glyphs are still
//!   rejected as unknown sections, because the loader claims a section through
//!   its factory table and neither has a factory here.

pub mod display;
pub mod hd44780;
pub mod st7920;
