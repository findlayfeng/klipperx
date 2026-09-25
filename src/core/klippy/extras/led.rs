//! The LED sections: `[led]`, `[neopixel]`, `[dotstar]`, `[pca9533]`,
//! `[pca9632]`, and the `[display_template]` sections they consume.
//!
//! Six sections live in one file because they are one feature: upstream splits
//! them only for Python's module layout (`led.py` holds `LEDHelper`, the four
//! driver files each wrap it, `display/display.py` holds the template registry)
//! while here they share [`LEDHelper`] and [`lookup_display_templates`] as one
//! seam. `display_template` lands next to its consumers for the same reason
//! upstream's lookup crosses `led.py` → `output_pin.py` → `display/display.py`:
//! nothing else in this host creates or reads a template.
//!
//! | section | upstream | options read |
//! |---|---|---|
//! | `[led <name>]` | `extras/led.py:150` `load_config_prefix` | `cycle_time`, `hardware_pwm`, `red_pin`/`green_pin`/`blue_pin`/`white_pin`, `initial_RED`… |
//! | `[neopixel <name>]` | `extras/neopixel.py:106` | `pin`, `chain_count`, `color_order`, `initial_RED`… |
//! | `[dotstar <name>]` | `extras/dotstar.py:55` | `data_pin`, `clock_pin`, `chain_count`, `initial_RED`… |
//! | `[pca9533 <name>]` | `extras/pca9533.py:37` | the `i2c_*` options of `bus.py:302` `MCU_I2C_from_config`, `initial_RED`… |
//! | `[pca9632 <name>]` | `extras/pca9632.py:69` | the `i2c_*` options, `color_order`, `initial_RED`… |
//! | `[display_template <name>]` | `display/display.py:168` `lookup_display_templates` (no upstream file of its own) | every `param_*` option, `text` |
//!
//! `initial_RED`/`GREEN`/`BLUE`/`WHITE` and the chain options are read by
//! [`LEDHelper`] / each factory exactly as upstream does, so every option a
//! shipped config writes is recorded and the undefined-option check passes.
//!
//! # What is not here
//!
//! * **`SET_LED` / `SET_LED_TEMPLATE`** (`led.py:34/100`): not registered, so
//!   the corpus lines using them run through the unknown-command path and are
//!   silently accepted. The colour state exists and reports through
//!   `get_status`; driving it arrives with the command layer (H3/H8).
//! * **Transmit paths.** Upstream pushes the initial colour through the MCU at
//!   connect (`PrinterPWMLED` sets start values — that part *is* here — while
//!   neopixel/dotstar/pca register `klippy:connect` writers). This port creates
//!   no firmware resource for those drivers: the pins are validated and
//!   reserved, the `[led]` PWMs are configured, and nothing else reaches the
//!   firmware until a command needs it to.
//! * **Template rendering.** `text` and `param_*` are parsed and validated with
//!   upstream's error wording, but there is no Jinja engine (`gcode_macro` is
//!   U-A7), so nothing renders a template yet.
//! * **`cycle_time`'s `maxval`** is upstream's `mcu.max_nominal_duration()`
//!   (`led.py:114`); this host has no nominal-duration figure to bound it by,
//!   so only `above=0` is enforced.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::load::section;
use crate::core::klippy::pins::{PrinterPins, PwmOut, PINS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};

// All six are prefix sections upstream (`load_config_prefix` each); order 40
// loads them after `[board_pins]` (30), so a pin alias may name an LED pin,
// alongside the other bus consumer `[i2c_device]`.
section!(
    "display_template",
    order = 40,
    prefix = load_display_template
);
section!("dotstar", order = 40, prefix = load_dotstar);
section!("led", order = 40, prefix = load_led);
section!("neopixel", order = 40, prefix = load_neopixel);
section!("pca9533", order = 40, prefix = load_pca9533);
section!("pca9632", order = 40, prefix = load_pca9632);

/// The printer object name upstream's lazy lookup registers
/// (`display/display.py:172`).
///
/// Public because the `display` module reads the registry back by name: it is
/// the one consumer that merges the shipped `display.cfg` templates in.
pub const DISPLAY_TEMPLATES_OBJECT: &str = "display_template";

/// Firmware limit on one neopixel chain's colour channels
/// (`neopixel.py:24`, `MAX_MCU_SIZE`).
const NEOPIXEL_MAX_CHANNELS: usize = 500;

// ===========================================================================
// LEDHelper — the shared colour state
// ===========================================================================

/// The initial colour and current state of one LED (chain).
///
/// Upstream's `led.py` `LEDHelper`: reads `initial_RED`/`initial_GREEN`/
/// `initial_BLUE`/`initial_WHITE` bounded to `0..1` and holds one
/// `[red, green, blue, white]` row per chain link. Each of the five driver
/// sections returns this as its printer object — upstream's classes differ only
/// in their transmit path, which this port does not have yet (module docs), and
/// `get_status` is `LEDHelper.get_status` on all five upstream
/// (`color_data`).
#[derive(Debug)]
pub struct LEDHelper {
    /// One `[r, g, b, w]` row per link, updated by `SET_LED` once it exists.
    state: Mutex<Vec<[f64; 4]>>,
}

impl LEDHelper {
    /// Read the shared options and ask for the template registry, as upstream's
    /// constructor does (`led.py:14-26`).
    ///
    /// # Errors
    /// A colour out of `0..=1`, or a config error from the template lookup.
    fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        led_count: usize,
    ) -> Result<Self, ConfigError> {
        let red =
            config.get_float_bounded("initial_RED", Some(0.0), Some(0.0), Some(1.0), None, None)?;
        let green = config.get_float_bounded(
            "initial_GREEN",
            Some(0.0),
            Some(0.0),
            Some(1.0),
            None,
            None,
        )?;
        let blue = config.get_float_bounded(
            "initial_BLUE",
            Some(0.0),
            Some(0.0),
            Some(1.0),
            None,
            None,
        )?;
        let white = config.get_float_bounded(
            "initial_WHITE",
            Some(0.0),
            Some(0.0),
            Some(1.0),
            None,
            None,
        )?;

        // Upstream reaches the registry through `output_pin.lookup_template_eval`
        // (`led.py:26`); this port has no evaluator yet, so the LED asks for the
        // registry directly — which is what creates it when a config has LEDs
        // but no `[display_template]` section of its own.
        lookup_display_templates(printer)?;

        Ok(Self {
            state: Mutex::new(vec![[red, green, blue, white]; led_count]),
        })
    }

    /// One link's colour, for a driver's start value.
    fn color(&self, index: usize) -> [f64; 4] {
        self.state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())[index]
    }
}

impl PrinterObject for LEDHelper {
    /// The colour data, as upstream's `LEDHelper.get_status` (`led.py:37-38`).
    fn get_status(&self, _eventtime: f64) -> Value {
        let state = self
            .state
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        json!({ "color_data": *state })
    }
}

// ===========================================================================
// `[display_template <name>]` — the lazy registry
// ===========================================================================

/// One parsed `[display_template <name>]` section: its `text` and `param_*`
/// values, as upstream's `DisplayTemplate` holds them
/// (`display/display.py:25-42`).
#[derive(Debug)]
struct DisplayTemplate {
    /// The template source (`text`), stored unrendered (no engine yet).
    text: String,
    /// `param_*` options keyed by their full option name, parsed as a Python
    /// literal the way `ast.literal_eval` accepts them (`display.py:36`).
    params: BTreeMap<String, Value>,
}

/// The `display_template` printer object: every template by section name.
///
/// Upstream creates exactly one of these, and only when a consumer asks
/// (`display/display.py:168-173`); [`lookup_display_templates`] is that lookup
/// here. It is registered but not client-visible: upstream's
/// `PrinterDisplayTemplate` defines no `get_status`, so it never appears in
/// `objects/list`.
#[derive(Debug, Default)]
pub struct DisplayTemplates {
    templates: Mutex<BTreeMap<String, DisplayTemplate>>,
}

impl DisplayTemplates {
    /// File one parsed section under its name (upstream `display.py:130-131`).
    fn insert(&self, name: String, text: String, params: BTreeMap<String, Value>) {
        self.templates
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .insert(name, DisplayTemplate { text, params });
    }

    /// A template's declared `param_*` option names, or `None` when there is no
    /// such template.
    ///
    /// What the `display` module needs from the registry: upstream's
    /// `DisplayTemplate.render` compares the call's keyword arguments against
    /// the template's own parameters by count (`display/display.py:43-46`), and
    /// the check lives in
    /// [`display::check_render_params`](super::display::check_render_params).
    pub fn param_names(&self, name: &str) -> Option<Vec<String>> {
        let templates = self
            .templates
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        templates
            .get(name)
            .map(|template| template.params.keys().cloned().collect())
    }
}

impl PrinterObject for DisplayTemplates {
    /// Parsed templates by name. Upstream's object has no status at all; the
    /// trait needs one, and this is the data it would report.
    fn get_status(&self, _eventtime: f64) -> Value {
        let templates = self
            .templates
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let map: serde_json::Map<String, Value> = templates
            .iter()
            .map(|(name, template)| {
                (
                    name.clone(),
                    json!({ "text": template.text, "params": template.params }),
                )
            })
            .collect();
        json!({ "templates": map })
    }

    /// Not client-visible, matching upstream's `get_status`-less object
    /// (`display/display.py:90`).
    fn is_queryable(&self) -> bool {
        false
    }
}

/// Look up the shared `display_template` object, creating it on first use.
///
/// Upstream's `lookup_display_templates` (`display/display.py:168`): the object
/// exists only once a consumer — an LED section, `output_pin`, or `display` —
/// asks for it, and every consumer then shares it.
///
/// # Errors
/// Only if the object name is already taken, which a duplicate lazy creation
/// would mean.
fn lookup_display_templates(printer: &Arc<Printer>) -> Result<Arc<DisplayTemplates>, ConfigError> {
    if let Some(existing) = printer.lookup_object_as::<DisplayTemplates>(DISPLAY_TEMPLATES_OBJECT) {
        return Ok(existing);
    }
    let created = Arc::new(DisplayTemplates::default());
    printer.add_object(
        DISPLAY_TEMPLATES_OBJECT,
        Arc::clone(&created) as Arc<dyn PrinterObject>,
    )?;
    Ok(created)
}

/// Parse one `param_*` value the way upstream's `ast.literal_eval` accepts it.
///
/// Accepts Python's `True`/`False`/`None`, numbers, strings, lists, dicts and
/// tuples-in-JSON-spelling; anything else is upstream's "not a valid literal"
/// error at the call site. `true`/`false`/`null` in lowercase are *rejected*:
/// Python reads those as names, not literals (`display/display.py:36-40`).
fn parse_literal(text: &str) -> Option<Value> {
    let trimmed = text.trim();
    match trimmed {
        "True" => return Some(json!(true)),
        "False" => return Some(json!(false)),
        "None" => return Some(Value::Null),
        "true" | "false" | "null" => return None,
        _ => {}
    }
    if let Ok(value) = serde_json::from_str::<Value>(trimmed) {
        return Some(value);
    }
    // Python also spells strings with single quotes, which JSON has no form
    // for; take the content between the quotes as written.
    if trimmed.len() >= 2 && trimmed.starts_with('\'') && trimmed.ends_with('\'') {
        return Some(json!(&trimmed[1..trimmed.len() - 1]));
    }
    None
}

/// Upstream's `load_config_prefix` for `[display_template <name>]`.
///
/// Upstream has no module for this section: the generic walk's
/// `load_object(section, None)` finds no `display_template.py` and skips it
/// (`klippy.py:99-102`), and the object is built later, all at once, from the
/// consumer side. This host's loader has no "skip" path — a section is valid
/// when a factory claims it or something reads it (`config/validate.rs`) — so
/// the section lands here, next to its consumers, and reads itself the way
/// `DisplayTemplate.__init__` does: every `param_*` option first
/// (`display.py:34-40`), then the required `text` (`display.py:41-42`).
pub fn load_display_template(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let identifier = config.identifier();
    let name = config.section().sub.clone().ok_or_else(|| {
        ConfigError::new(format!(
            "Section '{identifier}' must be a '[display_template <name>]' section"
        ))
    })?;

    let mut params = BTreeMap::new();
    for option in config.prefix_options("param_") {
        let raw = config.get(&option, None)?;
        let value = parse_literal(&raw).ok_or_else(|| {
            ConfigError::new(format!(
                "Option '{option}' in section '{identifier}' is not a valid literal"
            ))
        })?;
        params.insert(option, value);
    }
    let text = config.get("text", None)?;

    let templates = lookup_display_templates(printer)?;
    templates.insert(name, text, params);
    Ok(templates as Arc<dyn PrinterObject>)
}

// ===========================================================================
// `[led <name>]` — PWM-driven LEDs
// ===========================================================================

/// Upstream's `load_config_prefix` for `[led <name>]` (`led.py:150`).
///
/// Reads `cycle_time`/`hardware_pwm`, then each colour pin that is present,
/// configures a PWM for it (start value = the initial colour, as
/// `led.py:131-135`), and reports the shared initial colour through
/// [`LEDHelper`].
///
/// # Errors
/// Upstream's wording: a non-positive `cycle_time`, an unresolvable pin, or
/// `No LED pin definitions found in '<section>'` when no colour pin is set
/// (`led.py:121-123`).
pub fn load_led(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let identifier = config.identifier();
    // Read in upstream's order, before the pins (`led.py:113-115`).
    let cycle_time =
        config.get_float_bounded("cycle_time", Some(0.010), None, None, Some(0.0), None)?;
    let hardware_pwm = config.get_bool("hardware_pwm", Some(false))?;

    let pins = printer
        .lookup_object_as::<PrinterPins>(PINS_OBJECT)
        .expect("the loader registers `pins` before any section");

    let mut outputs: Vec<(usize, Arc<dyn PwmOut>)> = Vec::new();
    for (index, channel) in ["red", "green", "blue", "white"].iter().enumerate() {
        let Some(description) = config.get_str(&format!("{channel}_pin")) else {
            continue;
        };
        let pwm = pins
            .setup_pwm(&description, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
        pwm.setup_max_duration(0.0);
        pwm.setup_cycle_time(cycle_time, hardware_pwm);
        outputs.push((index, pwm));
    }
    if outputs.is_empty() {
        return Err(ConfigError::new(format!(
            "No LED pin definitions found in '{identifier}'"
        )));
    }

    let helper = LEDHelper::new(config, printer, 1)?;
    for (index, pwm) in &outputs {
        pwm.setup_start_value(helper.color(0)[*index], 0.0);
    }
    Ok(Arc::new(helper) as Arc<dyn PrinterObject>)
}

// ===========================================================================
// `[neopixel <name>]`
// ===========================================================================

/// Upstream's `load_config_prefix` for `[neopixel <name>]` (`neopixel.py:106`).
///
/// Reads `pin` (validating and reserving it), `chain_count`, and `color_order`,
/// with upstream's three chain checks in upstream's order
/// (`neopixel.py:24-47`): a single-entry `color_order` repeats across the
/// chain, the resulting length must equal `chain_count`, each entry must be a
/// permutation of `RGB` or `RGBW`, and the total channel count may not exceed
/// [`NEOPIXEL_MAX_CHANNELS`].
///
/// # Errors
/// Upstream's wording, including `color_order does not match chain_count`,
/// `Invalid color_order '<x>'` and `neopixel chain too long`.
pub fn load_neopixel(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let identifier = config.identifier();

    let description = config.get("pin", None)?;
    let pins = printer
        .lookup_object_as::<PrinterPins>(PINS_OBJECT)
        .expect("the loader registers `pins` before any section");
    pins.lookup_pin(&description, false, false, None)
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

    let chain_count = config.get_int_bounded("chain_count", Some(1), Some(1), None)? as usize;
    let mut color_order = config
        .get_list("color_order", ',')
        .unwrap_or_else(|| vec!["GRB".to_string()]);
    if color_order.len() == 1 {
        let only = color_order.remove(0);
        color_order = vec![only; chain_count];
    }
    if color_order.len() != chain_count {
        return Err(ConfigError::new(
            "color_order does not match chain_count".to_string(),
        ));
    }

    let mut channels = 0usize;
    for order in &color_order {
        if !is_color_order(order) {
            return Err(ConfigError::new(format!("Invalid color_order '{order}'")));
        }
        channels += order.len();
    }
    if channels > NEOPIXEL_MAX_CHANNELS {
        return Err(ConfigError::new("neopixel chain too long".to_string()));
    }

    let helper = LEDHelper::new(config, printer, chain_count)?;
    Ok(Arc::new(helper) as Arc<dyn PrinterObject>)
}

/// Whether `order` is a permutation of `RGB` or `RGBW`
/// (`neopixel.py:39-40`: `sorted(co) in (sorted("RGB"), sorted("RGBW"))`).
fn is_color_order(order: &str) -> bool {
    let mut chars: Vec<char> = order.chars().collect();
    chars.sort_unstable();
    let mut rgb: Vec<char> = "RGB".chars().collect();
    rgb.sort_unstable();
    let mut rgbw: Vec<char> = "RGBW".chars().collect();
    rgbw.sort_unstable();
    chars == rgb || chars == rgbw
}

// ===========================================================================
// `[dotstar <name>]`
// ===========================================================================

/// Upstream's `load_config_prefix` for `[dotstar <name>]` (`dotstar.py:55`).
///
/// Reads `data_pin` and `clock_pin` (validating and reserving both), rejects
/// pins on different MCUs (`dotstar.py:20-21`), then `chain_count` and the
/// shared initial colour. Upstream builds a software SPI here; that resource
/// belongs to the transmit path this port does not have yet (module docs), so
/// only the pins are set up.
///
/// # Errors
/// Upstream's wording, including `Dotstar pins must be on same mcu`.
pub fn load_dotstar(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let identifier = config.identifier();

    let data_description = config.get("data_pin", None)?;
    let clock_description = config.get("clock_pin", None)?;
    let pins = printer
        .lookup_object_as::<PrinterPins>(PINS_OBJECT)
        .expect("the loader registers `pins` before any section");
    let data = pins
        .lookup_pin(&data_description, false, false, None)
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
    let clock = pins
        .lookup_pin(&clock_description, false, false, None)
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
    if data.chip_name != clock.chip_name {
        return Err(ConfigError::new("Dotstar pins must be on same mcu"));
    }

    let chain_count = config.get_int_bounded("chain_count", Some(1), Some(1), None)? as usize;
    let helper = LEDHelper::new(config, printer, chain_count)?;
    Ok(Arc::new(helper) as Arc<dyn PrinterObject>)
}

// ===========================================================================
// `[pca9533 <name>]` / `[pca9632 <name>]` — I²C LED drivers
// ===========================================================================

/// Upstream's `load_config_prefix` for `[pca9533 <name>]` (`pca9533.py:37`):
/// the I²C bus options of `MCU_I2C_from_config` (`bus.py:302`) and the shared
/// initial colour. The chip's connect-time register writes belong to the
/// transmit path this port does not have yet (module docs).
pub fn load_pca9533(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    read_i2c_options(config, printer)?;
    let helper = LEDHelper::new(config, printer, 1)?;
    Ok(Arc::new(helper) as Arc<dyn PrinterObject>)
}

/// Upstream's `load_config_prefix` for `[pca9632 <name>]` (`pca9632.py:69`):
/// the I²C bus options, then `color_order` — which must be a permutation of
/// `RGBW` (`pca9632.py:34-36`) — then the shared initial colour
/// (`pca9632.py:36-39`).
pub fn load_pca9632(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    read_i2c_options(config, printer)?;

    let color_order = config.get("color_order", Some("RGBW"))?;
    if !is_color_order(&color_order) || color_order.chars().count() != 4 {
        return Err(ConfigError::new(format!(
            "Invalid color_order '{color_order}'"
        )));
    }

    let helper = LEDHelper::new(config, printer, 1)?;
    Ok(Arc::new(helper) as Arc<dyn PrinterObject>)
}

/// Read the `i2c_*` options the drivers share, as upstream's
/// `MCU_I2C_from_config` does (`bus.py:302-329`): `i2c_mcu` (default `mcu`),
/// `i2c_speed` (default and minimum 100000), `i2c_address` (default 98,
/// `0..=127`), then either both software-pin options or `i2c_bus`.
///
/// The pins are validated and reserved but no bus resource is created: the
/// drivers' register writes happen at connect upstream, and this port's
/// transmit path is not here yet (module docs).
///
/// # Errors
/// Upstream's bound wording, `must be specified` for a lone software pin, and
/// `<section>: i2c pins must be on same mcu` (`bus.py:321-322`).
fn read_i2c_options(config: &ConfigWrapper, printer: &Printer) -> Result<(), ConfigError> {
    let identifier = config.identifier();
    let mcu = config.get("i2c_mcu", Some("mcu"))?.trim().to_string();
    config.get_int_bounded("i2c_speed", Some(100_000), Some(100_000), None)?;
    config.get_int_bounded("i2c_address", Some(98), Some(0), Some(127))?;

    let Some(scl) = config.get_str("i2c_software_scl_pin") else {
        // No software pins: the option left is the hardware bus name, which
        // upstream reads as an optional value (`bus.py:326`).
        let _ = config.get_str("i2c_bus");
        return Ok(());
    };
    let sda = config.get("i2c_software_sda_pin", None)?;

    let pins = printer
        .lookup_object_as::<PrinterPins>(PINS_OBJECT)
        .expect("the loader registers `pins` before any section");
    let scl = pins
        .lookup_pin(&scl, false, false, Some("i2c_software_scl_pin"))
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
    let sda = pins
        .lookup_pin(&sda, false, false, Some("i2c_software_sda_pin"))
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;
    if scl.chip_name != mcu || sda.chip_name != mcu {
        return Err(ConfigError::new(format!(
            "{identifier}: i2c pins must be on same mcu"
        )));
    }
    Ok(())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, ConfigSection, ConfigValue};
    use crate::core::klippy::mcu::McuError;
    use crate::core::klippy::pins::{DigitalOut, PinChip, PinError, PinParams};
    use crate::core::klippy::reactor::ManualReactor;

    /// A PWM that records how it was configured.
    #[derive(Default)]
    struct FakePwm {
        max_duration: Mutex<f64>,
        cycle_time: Mutex<(f64, bool)>,
        start_value: Mutex<(f64, f64)>,
    }

    impl PwmOut for FakePwm {
        fn setup_max_duration(&self, max_duration: f64) {
            *self.max_duration.lock().unwrap() = max_duration;
        }
        fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) {
            *self.cycle_time.lock().unwrap() = (cycle_time, hardware_pwm);
        }
        fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
            *self.start_value.lock().unwrap() = (start_value, shutdown_value);
        }
        fn set_pwm(&self, _clock: u32, _value: f64) -> Result<(), McuError> {
            Ok(())
        }
        fn update_pwm(&self, _value: f64) -> Result<(), McuError> {
            Ok(())
        }
        fn next_aligned_clock(&self, clock: u32, _allow_early: f64) -> Result<u32, McuError> {
            Ok(clock)
        }
    }

    /// A chip that hands out a [`FakePwm`] per setup. The LED sections never
    /// build a plain digital output, so that arm of the trait is not
    /// implemented.
    #[derive(Default)]
    struct FakeChip {
        pwms: Mutex<Vec<Arc<FakePwm>>>,
    }

    impl PinChip for FakeChip {
        fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
            panic!("the LED sections drive only PWM outputs")
        }

        fn setup_pwm(&self, _params: &PinParams) -> Result<Arc<dyn PwmOut>, PinError> {
            let pwm = Arc::new(FakePwm::default());
            self.pwms.lock().unwrap().push(Arc::clone(&pwm));
            Ok(pwm)
        }
    }

    /// A printer with `pins` over an `mcu` chip, as the loader builds it
    /// before any section runs.
    fn printer() -> (Arc<Printer>, Arc<FakeChip>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let pins = Arc::new(PrinterPins::new());
        let chip = Arc::new(FakeChip::default());
        pins.register_chip("mcu", chip.clone()).unwrap();
        printer.add_object(PINS_OBJECT, pins).unwrap();
        (printer, chip)
    }

    /// A second chip, for a section whose pins must share one MCU.
    fn second_chip(printer: &Printer) -> Arc<FakeChip> {
        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("pins");
        let chip = Arc::new(FakeChip::default());
        pins.register_chip("board2", chip.clone()).unwrap();
        chip
    }

    /// A section with `options`, keyed the way the parser stores them
    /// (lowercased, as upstream's `optionxform` does).
    fn cfg_section(id: &str, sub: Option<&str>, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new(id, sub);
        for (key, value) in options {
            section.parameters.insert(
                key.to_lowercase(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// Wrap a section the way the loader does, recording into `access`.
    fn wrap<'a>(section: &'a ConfigSection, access: &Arc<AccessTracking>) -> ConfigWrapper<'a> {
        ConfigWrapper::new(section, Arc::clone(access))
    }

    // -- [led] --------------------------------------------------------------

    #[test]
    fn test_a_pwm_led_reads_its_pins_and_initial_color() {
        let (printer, chip) = printer();
        let access = AccessTracking::shared();
        let section = cfg_section(
            "led",
            Some("lled"),
            &[
                ("red_pin", "PA2"),
                ("green_pin", "PA3"),
                ("cycle_time", "0.05"),
                ("hardware_pwm", "true"),
                ("initial_RED", "0.2"),
            ],
        );

        let led = load_led(&wrap(&section, &access), &printer).unwrap();

        let pwms = chip.pwms.lock().unwrap();
        assert_eq!(pwms.len(), 2);
        assert_eq!(*pwms[0].max_duration.lock().unwrap(), 0.0);
        assert_eq!(*pwms[0].cycle_time.lock().unwrap(), (0.05, true));
        assert_eq!(*pwms[0].start_value.lock().unwrap(), (0.2, 0.0));
        assert_eq!(*pwms[1].start_value.lock().unwrap(), (0.0, 0.0));
        drop(pwms);

        assert_eq!(
            led.get_status(0.0)["color_data"],
            json!([[0.2, 0.0, 0.0, 0.0]])
        );
        for option in [
            "red_pin",
            "green_pin",
            "cycle_time",
            "hardware_pwm",
            "initial_red",
        ] {
            assert!(access.contains("led lled", option), "option {option}");
        }
    }

    #[test]
    fn test_a_pwm_led_defaults_to_a_software_cycle() {
        let (printer, chip) = printer();
        load_led(
            &wrap(
                &cfg_section("led", Some("lled"), &[("red_pin", "PA2")]),
                &AccessTracking::shared(),
            ),
            &printer,
        )
        .unwrap();

        let pwm = chip.pwms.lock().unwrap()[0].clone();
        assert_eq!(*pwm.cycle_time.lock().unwrap(), (0.010, false));
        assert_eq!(*pwm.start_value.lock().unwrap(), (0.0, 0.0));
    }

    #[test]
    fn test_a_led_without_any_pin_is_reported() {
        let (printer, _chip) = printer();
        let section = cfg_section("led", Some("lled"), &[("initial_RED", "0.2")]);

        let err = load_led(&wrap(&section, &AccessTracking::shared()), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "No LED pin definitions found in 'led lled'"
        );
    }

    #[test]
    fn test_an_initial_color_above_one_is_reported() {
        let (printer, _chip) = printer();
        let section = cfg_section(
            "led",
            Some("lled"),
            &[("red_pin", "PA2"), ("initial_GREEN", "1.5")],
        );

        let err = load_led(&wrap(&section, &AccessTracking::shared()), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'initial_GREEN' in section 'led lled' must have maximum of 1"
        );
    }

    // -- [neopixel] ---------------------------------------------------------

    #[test]
    fn test_a_neopixel_reads_its_chain_and_initial_colors() {
        let (printer, _chip) = printer();
        let access = AccessTracking::shared();
        let section = cfg_section(
            "neopixel",
            Some("nled"),
            &[
                ("pin", "PA3"),
                ("chain_count", "4"),
                ("initial_RED", "0.2"),
                ("initial_GREEN", "0.3"),
                ("initial_BLUE", "0.4"),
            ],
        );

        let led = load_neopixel(&wrap(&section, &access), &printer).unwrap();

        assert_eq!(
            led.get_status(0.0)["color_data"],
            json!([
                [0.2, 0.3, 0.4, 0.0],
                [0.2, 0.3, 0.4, 0.0],
                [0.2, 0.3, 0.4, 0.0],
                [0.2, 0.3, 0.4, 0.0]
            ])
        );
        for option in [
            "pin",
            "chain_count",
            "initial_red",
            "initial_green",
            "initial_blue",
        ] {
            assert!(access.contains("neopixel nled", option), "option {option}");
        }
    }

    #[test]
    fn test_a_neopixel_defaults_to_one_led_and_grb() {
        let (printer, _chip) = printer();
        let access = AccessTracking::shared();
        let section = cfg_section("neopixel", Some("nled"), &[("pin", "PA3")]);

        let led = load_neopixel(&wrap(&section, &access), &printer).unwrap();

        assert_eq!(
            led.get_status(0.0)["color_data"],
            json!([[0.0, 0.0, 0.0, 0.0]])
        );
        assert!(access.contains("neopixel nled", "chain_count"));
        assert!(access.contains("neopixel nled", "initial_red"));
    }

    #[test]
    fn test_a_neopixel_color_order_must_match_the_chain() {
        // A single-entry `color_order` repeats across the chain upstream
        // (`neopixel.py:30-31`); only a list that neither repeats nor matches
        // `chain_count` is an error.
        let (printer, _chip) = printer();
        let section = cfg_section(
            "neopixel",
            Some("nled"),
            &[
                ("pin", "PA3"),
                ("chain_count", "4"),
                ("color_order", "GRB,BGR"),
            ],
        );

        let err = load_neopixel(&wrap(&section, &AccessTracking::shared()), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(err.to_string(), "color_order does not match chain_count");
    }

    #[test]
    fn test_an_invalid_neopixel_color_order_is_reported() {
        let (printer, _chip) = printer();
        let section = cfg_section(
            "neopixel",
            Some("nled"),
            &[("pin", "PA3"), ("color_order", "XYZ")],
        );

        let err = load_neopixel(&wrap(&section, &AccessTracking::shared()), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(err.to_string(), "Invalid color_order 'XYZ'");
    }

    #[test]
    fn test_a_neopixel_chain_beyond_the_firmware_limit_is_reported() {
        let (printer, _chip) = printer();
        let section = cfg_section(
            "neopixel",
            Some("nled"),
            &[("pin", "PA3"), ("chain_count", "501")],
        );

        let err = load_neopixel(&wrap(&section, &AccessTracking::shared()), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(err.to_string(), "neopixel chain too long");
    }

    // -- [dotstar] ----------------------------------------------------------

    #[test]
    fn test_a_dotstar_reads_both_pins_and_the_chain() {
        let (printer, _chip) = printer();
        let access = AccessTracking::shared();
        let section = cfg_section(
            "dotstar",
            Some("dled"),
            &[
                ("data_pin", "PA4"),
                ("clock_pin", "PA5"),
                ("chain_count", "2"),
                ("initial_RED", "0.4"),
            ],
        );

        let led = load_dotstar(&wrap(&section, &access), &printer).unwrap();

        assert_eq!(
            led.get_status(0.0)["color_data"],
            json!([[0.4, 0.0, 0.0, 0.0], [0.4, 0.0, 0.0, 0.0]])
        );
        for option in ["data_pin", "clock_pin", "chain_count", "initial_red"] {
            assert!(access.contains("dotstar dled", option), "option {option}");
        }
    }

    #[test]
    fn test_dotstar_pins_must_be_on_the_same_mcu() {
        let (printer, _chip) = printer();
        second_chip(&printer);
        let section = cfg_section(
            "dotstar",
            Some("dled"),
            &[("data_pin", "board2:PA4"), ("clock_pin", "PA5")],
        );

        let err = load_dotstar(&wrap(&section, &AccessTracking::shared()), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(err.to_string(), "Dotstar pins must be on same mcu");
    }

    // -- [pca9533] / [pca9632] ---------------------------------------------

    #[test]
    fn test_a_pca9533_reads_the_i2c_options_with_upstreams_defaults() {
        let (printer, _chip) = printer();
        let access = AccessTracking::shared();
        let section = cfg_section(
            "pca9533",
            Some("p5led"),
            &[("initial_RED", "0.1"), ("initial_BLUE", "0.3")],
        );

        let led = load_pca9533(&wrap(&section, &access), &printer).unwrap();

        assert_eq!(
            led.get_status(0.0)["color_data"],
            json!([[0.1, 0.0, 0.3, 0.0]])
        );
        for option in [
            "i2c_mcu",
            "i2c_speed",
            "i2c_address",
            "initial_red",
            "initial_blue",
        ] {
            assert!(access.contains("pca9533 p5led", option), "option {option}");
        }
    }

    #[test]
    fn test_an_i2c_address_outside_the_seven_bit_range_is_reported() {
        let (printer, _chip) = printer();
        let section = cfg_section("pca9533", Some("p5led"), &[("i2c_address", "200")]);

        let err = load_pca9533(&wrap(&section, &AccessTracking::shared()), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'i2c_address' in section 'pca9533 p5led' must have maximum of 127"
        );
    }

    #[test]
    fn test_a_pca9632_reads_its_software_i2c_pins_and_color_order() {
        let (printer, _chip) = printer();
        let access = AccessTracking::shared();
        let section = cfg_section(
            "pca9632",
            Some("p6led"),
            &[
                ("i2c_software_scl_pin", "PB1"),
                ("i2c_software_sda_pin", "PB2"),
                ("initial_RED", "0.4"),
            ],
        );

        let led = load_pca9632(&wrap(&section, &access), &printer).unwrap();

        assert_eq!(
            led.get_status(0.0)["color_data"],
            json!([[0.4, 0.0, 0.0, 0.0]])
        );
        for option in [
            "i2c_software_scl_pin",
            "i2c_software_sda_pin",
            "color_order",
            "initial_red",
        ] {
            assert!(access.contains("pca9632 p6led", option), "option {option}");
        }
    }

    #[test]
    fn test_a_lone_i2c_software_pin_is_reported() {
        let (printer, _chip) = printer();
        let section = cfg_section("pca9632", Some("p6led"), &[("i2c_software_scl_pin", "PB1")]);

        let err = load_pca9632(&wrap(&section, &AccessTracking::shared()), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'i2c_software_sda_pin' in section 'pca9632 p6led' must be specified"
        );
    }

    #[test]
    fn test_a_pca9632_color_order_must_be_a_rgbw_permutation() {
        let (printer, _chip) = printer();
        let section = cfg_section("pca9632", Some("p6led"), &[("color_order", "RGB")]);

        let err = load_pca9632(&wrap(&section, &AccessTracking::shared()), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(err.to_string(), "Invalid color_order 'RGB'");
    }

    // -- [display_template] -------------------------------------------------

    #[test]
    fn test_a_display_template_reads_text_and_params() {
        let (printer, _printer_chip) = printer();
        let access = AccessTracking::shared();
        let section = cfg_section(
            "display_template",
            Some("dtest"),
            &[("param_myvar", "1.2"), ("text", "{ param_myvar }, 0.0")],
        );

        let templates = load_display_template(&wrap(&section, &access), &printer).unwrap();

        let dtest = &templates.get_status(0.0)["templates"]["dtest"];
        assert_eq!(dtest["params"]["param_myvar"], json!(1.2));
        assert_eq!(dtest["text"], json!("{ param_myvar }, 0.0"));
        assert!(access.contains("display_template dtest", "param_myvar"));
        assert!(access.contains("display_template dtest", "text"));
        // Registered under the upstream object name as well (`display.py:172`).
        assert!(printer.lookup_object(DISPLAY_TEMPLATES_OBJECT).is_some());
        // And not client-visible: upstream's object has no `get_status`.
        assert!(!templates.is_queryable());
    }

    #[test]
    fn test_the_display_template_object_is_created_lazily_and_shared() {
        let (printer, _chip) = printer();

        assert!(printer.lookup_object(DISPLAY_TEMPLATES_OBJECT).is_none());

        // An LED section brings the registry up through its constructor, the
        // way upstream's LEDHelper walks to `lookup_display_templates`.
        let section = cfg_section("led", Some("lled"), &[("red_pin", "PA2")]);
        load_led(&wrap(&section, &AccessTracking::shared()), &printer).unwrap();
        assert!(printer.lookup_object(DISPLAY_TEMPLATES_OBJECT).is_some());

        let first = lookup_display_templates(&printer).unwrap();
        let second = lookup_display_templates(&printer).unwrap();
        assert!(Arc::ptr_eq(&first, &second));
    }

    #[test]
    fn test_a_display_template_requires_text() {
        let (printer, _chip) = printer();
        let section = cfg_section("display_template", Some("dtest"), &[("param_myvar", "1.2")]);

        let err = load_display_template(&wrap(&section, &AccessTracking::shared()), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'text' in section 'display_template dtest' must be specified"
        );
    }

    #[test]
    fn test_a_display_template_param_must_be_a_literal() {
        let (printer, _chip) = printer();
        let section = cfg_section(
            "display_template",
            Some("dtest"),
            &[("param_myvar", "notaliteral"), ("text", "0.0")],
        );

        let err = load_display_template(&wrap(&section, &AccessTracking::shared()), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'param_myvar' in section 'display_template dtest' is not a valid literal"
        );
    }

    #[test]
    fn test_a_python_false_is_not_a_json_true() {
        // `ast.literal_eval` reads lowercase `true` as a name, not a literal
        // (`display/display.py:36`), so it must fail here too.
        let (printer, _chip) = printer();
        let section = cfg_section(
            "display_template",
            Some("dtest"),
            &[("param_flag", "true"), ("text", "0.0")],
        );

        let err = load_display_template(&wrap(&section, &AccessTracking::shared()), &printer)
            .map(|_| ())
            .unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'param_flag' in section 'display_template dtest' is not a valid literal"
        );
    }
}
