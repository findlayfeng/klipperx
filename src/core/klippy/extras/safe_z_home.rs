//! `[safe_z_home]` — home Z at a chosen XY position
//! (upstream `klippy/extras/safe_z_home.py:7 def load_config`).
//!
//! | option | default | role |
//! |---|---|---|
//! | `home_xy_position` | — (required) | the `x, y` the toolhead moves to before homing Z (`:11`) |
//! | `z_hop` | `0.0` | how far to lift Z before the homing move (`:13`) |
//! | `z_hop_speed` | `15.0` mm/s | the hop's speed, above 0 (`:14`) |
//! | `speed` | `50.0` mm/s | the speed of the safe XY move and the move back, above 0 (`:19`) |
//! | `move_to_previous` | `false` | return XY to where the statement found them (`:20`) |
//!
//! The section takes `G28` away from the toolhead and installs its own handler
//! in front of it (`safe_z_home.py:21-24`): a plain `G28` first homes X and Y
//! with an explicit `X0 Y0` statement, moves to `home_xy_position`, then homes
//! Z there; other `G28` statements pass their axes straight through. The
//! previous handler is kept — the toolhead's, which is where `G28` lives
//! (`toolhead.rs:603-606`) — and every statement is handed to it, so the real
//! homing still runs.
//!
//! **Load order is load-bearing**: the section is `phase = late, order = 70`,
//! after the toolhead's `printer` section (`order = 60`, the same phase), which
//! is what registers `G28`. Loaded any earlier, the handler would be installed
//! before the toolhead's and then silently overwritten by it.
//!
//! Deliberate deviations, all narrower than upstream:
//!
//! - `max_z` is read exactly as upstream reads it (`:18`, a required
//!   `position_max` on the Z endstop section) and then unused, as upstream
//!   never consults it either.
//! - the Z endstop section is found the way
//!   `manual_probe.lookup_z_endstop_config` finds it (`manual_probe.py:25-33`):
//!   `[stepper_z]` when present, else the `[carriage <name>]` whose `axis` is
//!   `z`. The carriage form is unclaimed here, since this port has no factory
//!   for `[carriage …]` sections.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::Coord;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("safe_z_home", order = 70, phase = late, load = load_config);

/// The toolhead object, as the loader registers `[printer]`.
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The axis indices, as [`Coord`] numbers them.
const X_AXIS: usize = 0;
const Y_AXIS: usize = 1;
const Z_AXIS: usize = 2;

/// The section's option values (`safe_z_home.py:11-20`).
#[derive(Debug, PartialEq, Clone, Copy)]
struct SafeZHomeOptions {
    /// `home_xy_position`, as `[x, y]` (`:11`).
    home_xy_position: [f64; 2],
    /// `z_hop` (`:13`); `0.0` disables the hop.
    z_hop: f64,
    /// `z_hop_speed` (`:14`), above 0.
    z_hop_speed: f64,
    /// `speed` (`:19`), above 0.
    speed: f64,
    /// `move_to_previous` (`:20`).
    move_to_previous: bool,
    /// The Z endstop section's `position_max` (`:18`) — read to prove the
    /// section has one, then unused, as upstream never consults it.
    #[allow(dead_code)]
    max_z: f64,
}

impl SafeZHomeOptions {
    /// Read the section in upstream's order (`safe_z_home.py:11-20`).
    ///
    /// # Errors
    /// A malformed or missing `home_xy_position` (`getfloatlist(..., count=2)`),
    /// a bound violated by `speed`/`z_hop_speed`, a missing Z endstop section
    /// (`'Missing Z endstop config for safe_z_homing'`), or a missing
    /// `position_max` on it (`getfloat` with no default, `:18`).
    fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let home_xy_position = home_xy_position(config)?;
        let z_hop = config.get_float("z_hop", Some(0.0))?;
        let z_hop_speed =
            config.get_float_bounded("z_hop_speed", Some(15.0), None, None, Some(0.0), None)?;
        let zconfig = lookup_z_endstop_config(config)
            .ok_or_else(|| ConfigError::new("Missing Z endstop config for safe_z_homing"))?;
        let max_z = zconfig.get_float("position_max", None)?;
        let speed = config.get_float_bounded("speed", Some(50.0), None, None, Some(0.0), None)?;
        let move_to_previous = config.get_bool("move_to_previous", Some(false))?;
        Ok(Self {
            home_xy_position,
            z_hop,
            z_hop_speed,
            speed,
            move_to_previous,
            max_z,
        })
    }
}

/// The `home_xy_position` option — upstream's `getfloatlist(..., count=2)`
/// (`safe_z_home.py:11`), whose parse and count failures carry the option and
/// section names (`klippy/configfile.py:99-101`).
///
/// # Errors
/// `Option 'home_xy_position' in section '<id>' must be specified` when the
/// option is absent, `… must have 2 elements` when it does not hold two, and
/// `Unable to parse option 'home_xy_position' in section '<id>'` when an
/// element is not a number.
fn home_xy_position(config: &ConfigWrapper) -> Result<[f64; 2], ConfigError> {
    let identifier = config.identifier();
    let Some(items) = config.get_list("home_xy_position", ',') else {
        return Err(ConfigError::new(format!(
            "Option 'home_xy_position' in section '{identifier}' must be specified"
        )));
    };
    if items.len() != 2 {
        return Err(ConfigError::new(format!(
            "Option 'home_xy_position' in section '{identifier}' must have 2 elements"
        )));
    }
    let mut out = [0.0f64; 2];
    for (slot, item) in out.iter_mut().zip(&items) {
        *slot = item.trim().parse::<f64>().map_err(|_| {
            ConfigError::new(format!(
                "Unable to parse option 'home_xy_position' in section '{identifier}'"
            ))
        })?;
    }
    Ok(out)
}

/// The Z endstop section — upstream's `manual_probe.lookup_z_endstop_config`
/// (`manual_probe.py:25-33`): `[stepper_z]` when it exists, else the
/// `[carriage <name>]` whose `axis` option (defaulting to the name) is `z`.
///
/// Unlike upstream's, the `axis` read is recorded: this port has no
/// `note_valid=False`.
fn lookup_z_endstop_config<'a>(config: &ConfigWrapper<'a>) -> Option<ConfigWrapper<'a>> {
    if let Some(stepper_z) = config.sibling("stepper_z") {
        return Some(stepper_z);
    }
    for carriage in config.sibling_prefix_sections("carriage ") {
        let name = carriage.section().sub.clone().unwrap_or_default();
        let axis = carriage.get_str("axis").unwrap_or(name);
        if axis == "z" {
            return Some(carriage);
        }
    }
    None
}

/// The toolhead effects the `G28` handler performs, so the homing sequence is
/// testable without a machine (the `z_tilt.rs` `Adjust` / `probe.rs` `RoundOps`
/// pattern).
trait HomeOps {
    /// `kinematics.get_status()['homed_axes']`, the lower-case axis letters.
    fn homed_axes(&self) -> String;
    /// `toolhead.get_position()`.
    ///
    /// # Errors
    /// "Printer is not ready" before connect.
    fn position(&self) -> Result<Coord, CommandError>;
    /// `toolhead.manual_move` — queue the move and fire `toolhead:manual_move`.
    ///
    /// # Errors
    /// "Printer is not ready" before connect, or whatever the planner refuses.
    fn manual_move(&self, position: Coord, speed: f64) -> Result<(), CommandError>;
    /// `toolhead.set_position(position, homing_axes)`.
    ///
    /// # Errors
    /// "Printer is not ready" before connect, or a failed step flush.
    async fn set_position(
        &self,
        position: Coord,
        homing_axes: &[usize],
    ) -> Result<(), CommandError>;
    /// `kinematics.clear_homing_state(axes)`.
    fn clear_homing_state(&self, axes: &[usize]);
}

/// The [`HomeOps`] wiring against the real toolhead (`safe_z_home.py:35-90`).
struct LiveHome {
    toolhead: Arc<ToolHeadObject>,
    printer: Weak<Printer>,
}

impl HomeOps for LiveHome {
    fn homed_axes(&self) -> String {
        self.toolhead.get_status(0.0)["homed_axes"]
            .as_str()
            .unwrap_or("")
            .to_string()
    }

    fn position(&self) -> Result<Coord, CommandError> {
        self.toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    fn manual_move(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        self.toolhead.move_to(position, speed)?;
        // Upstream's `manual_move` fires this (`toolhead.py:416`); `gcode_move`
        // re-anchors its `last_position` on it.
        if let Some(printer) = self.printer.upgrade() {
            printer.send_event(&KlippyEvent::ToolheadManualMove);
        }
        Ok(())
    }

    async fn set_position(
        &self,
        position: Coord,
        homing_axes: &[usize],
    ) -> Result<(), CommandError> {
        self.toolhead.set_position(position, homing_axes).await
    }

    fn clear_homing_state(&self, axes: &[usize]) {
        self.toolhead.clear_homing_state(axes);
    }
}

/// Upstream's `ToolHead.manual_move` (`toolhead.py:410-416`): the non-`None`
/// entries of `coord` replace the commanded position's, then the move is
/// queued.
///
/// # Errors
/// As [`HomeOps::position`] and [`HomeOps::manual_move`].
fn manual_move<O: HomeOps + ?Sized>(
    ops: &O,
    coord: [Option<f64>; 3],
    speed: f64,
) -> Result<(), CommandError> {
    let mut curpos = ops.position()?;
    for (axis, value) in coord.iter().enumerate() {
        if let Some(value) = value {
            curpos.set_axis(axis, *value);
        }
    }
    ops.manual_move(curpos, speed)
}

/// The `G28` replacement itself (`safe_z_home.py:31-90`).
///
/// `gcode` is only used to synthesise the statements the previous handler is
/// handed (`create_gcode_command`, `safe_z_home.py:59-84`).
///
/// # Errors
/// `'Must home X and Y axes first'` for a Z home with X or Y unhomed
/// (`:70-72`), whatever the previous handler reports, and whatever the
/// toolhead refuses.
async fn run_g28<O: HomeOps + ?Sized>(
    options: &SafeZHomeOptions,
    gcode: &GCodeDispatch,
    ops: &O,
    prev_g28: &CommandHandler,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    // Perform the Z hop if necessary (`safe_z_home.py:33-50`).
    if options.z_hop != 0.0 {
        let homed = ops.homed_axes();
        let mut pos = ops.position()?;
        if !homed.contains('z') {
            // Always perform the z_hop if the Z axis is not homed: pretend Z
            // is at 0, hop, then forget that homing state again.
            pos.set_axis(Z_AXIS, 0.0);
            ops.set_position(pos, &[Z_AXIS]).await?;
            manual_move(ops, [None, None, Some(options.z_hop)], options.z_hop_speed)?;
            ops.clear_homing_state(&[Z_AXIS]);
        } else if pos.z() < options.z_hop {
            // If the Z axis is homed, and below z_hop, lift it to z_hop.
            manual_move(ops, [None, None, Some(options.z_hop)], options.z_hop_speed)?;
        }
    }

    // Determine which axes we need to home (`safe_z_home.py:52-56`): no axis
    // named means all three.
    let given = gcmd.get_command_parameters();
    let mut need = [false; 3];
    for (axis, name) in [(X_AXIS, "X"), (Y_AXIS, "Y"), (Z_AXIS, "Z")] {
        need[axis] = given.contains_key(name);
    }
    if !need.iter().any(|needed| *needed) {
        need = [true; 3];
    }

    // Home the XY axes if necessary (`safe_z_home.py:58-66`).
    let mut new_params = HashMap::new();
    if need[X_AXIS] {
        new_params.insert("X".to_string(), "0".to_string());
    }
    if need[Y_AXIS] {
        new_params.insert("Y".to_string(), "0".to_string());
    }
    if !new_params.is_empty() {
        let g28_gcmd = gcode.create_gcode_command("G28", "G28", new_params);
        prev_g28(&g28_gcmd).await?;
    }

    // Home the Z axis if necessary (`safe_z_home.py:68-90`).
    if need[Z_AXIS] {
        // X and Y must be homed before moving to the safe position.
        let homed = ops.homed_axes();
        if !homed.contains('x') || !homed.contains('y') {
            return Err(CommandError::new("Must home X and Y axes first"));
        }
        let prevpos = ops.position()?;
        manual_move(
            ops,
            [
                Some(options.home_xy_position[0]),
                Some(options.home_xy_position[1]),
                None,
            ],
            options.speed,
        )?;
        let g28_gcmd = gcode.create_gcode_command("G28", "G28", z_only());
        prev_g28(&g28_gcmd).await?;
        // Perform the Z hop again, for pressure-based probes
        // (`safe_z_home.py:80-84`).
        if options.z_hop != 0.0 && ops.position()?.z() < options.z_hop {
            manual_move(ops, [None, None, Some(options.z_hop)], options.z_hop_speed)?;
        }
        // Move XY back to where the statement found them.
        if options.move_to_previous {
            manual_move(
                ops,
                [Some(prevpos.x()), Some(prevpos.y()), None],
                options.speed,
            )?;
        }
    }
    Ok(())
}

/// The `{'Z': '0'}` parameters of the Z homing statement
/// (`safe_z_home.py:83`).
fn z_only() -> HashMap<String, String> {
    HashMap::from([("Z".to_string(), "0".to_string())])
}

/// The `[safe_z_home]` section: the `G28` wrapper's registration
/// (`safe_z_home.py:7-29`).
///
/// The option values and the toolhead are not kept: the installed handler
/// captures them, and it outlives any read of this object.
pub struct SafeZHoming {
    /// The handler this section replaced — the toolhead's. The handler clones
    /// the cell, so the two watch the same slot; nothing consumes the value
    /// after registration, hence the field-level `allow` (the tests read it to
    /// pin the load order).
    #[allow(dead_code)]
    prev_g28: Arc<Mutex<Option<CommandHandler>>>,
}

impl SafeZHoming {
    /// Read the section and install the `G28` wrapper
    /// (`safe_z_home.py:10-29`).
    ///
    /// # Errors
    /// As [`SafeZHomeOptions::read`], upstream's
    /// `'homing_override and safe_z_homing cannot be used simultaneously'`
    /// (`:26-28`), a `G28` this section cannot chain to (loaded before the
    /// toolhead), or a name already taken by another module.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let options = SafeZHomeOptions::read(config)?;
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");

        // Take the previous `G28` so this module can chain to it
        // (`safe_z_home.py:21-24`). `section!` loads this after the toolhead,
        // which is where `G28` is registered (`toolhead.rs:603-606`); without
        // it there would be nothing to chain the homing to, so a missing one
        // is a wiring error rather than a silently dropped `G28`.
        let prev = gcode.unregister_command("G28").ok_or_else(|| {
            ConfigError::new("safe_z_home must be loaded after `G28` is registered")
        })?;

        let prev_g28 = Arc::new(Mutex::new(Some(prev)));
        let handler: CommandHandler = {
            let options = options;
            let gcode = Arc::clone(&gcode);
            let prev_g28 = Arc::clone(&prev_g28);
            let printer = Arc::downgrade(printer);
            Arc::new(move |gcmd: &GcodeCommand| {
                let gcode = Arc::clone(&gcode);
                let prev_g28 = Arc::clone(&prev_g28);
                let printer = printer.clone();
                Box::pin(async move {
                    let printer = printer
                        .upgrade()
                        .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                    let toolhead = printer
                        .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)
                        .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                    let prev = prev_g28
                        .lock()
                        .unwrap_or_else(|poison| poison.into_inner())
                        .clone()
                        .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                    let ops = LiveHome {
                        toolhead,
                        printer: Arc::downgrade(&printer),
                    };
                    run_g28(&options, &gcode, &ops, &prev, gcmd).await
                })
            })
        };
        // No help text: upstream leaves the previous one in place
        // (`gcode.rs:575-583`).
        gcode
            .register_command("G28", handler, None, false)
            .map_err(ConfigError::new)?;

        // The override owns `G28`, so the two cannot share it
        // (`safe_z_home.py:26-28`).
        if config.has_sibling("homing_override") {
            return Err(ConfigError::new(
                "homing_override and safe_z_homing cannot be used simultaneously",
            ));
        }

        Ok(Self { prev_g28 })
    }
}

impl PrinterObject for SafeZHoming {
    /// Upstream's `SafeZHoming` defines no `get_status`; like the sections it
    /// wraps, the port reports an empty object.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }
}

/// The factory `section!` names (`safe_z_home.py:91-92 def load_config`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(SafeZHoming::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::{check_unused, Config};
    use crate::core::klippy::reactor::ManualReactor;

    /// A tracked wrapper around `identifier` in `config`, the way the loader
    /// hands a factory its section (`sibling` needs the whole config).
    fn wrapper<'a>(
        config: &'a Config,
        identifier: &str,
    ) -> (ConfigWrapper<'a>, Arc<AccessTracking>) {
        let section = config.get_section(identifier).expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::with_config(section, Arc::clone(&access), None, config);
        (wrapper, access)
    }

    fn parse(text: &str) -> Config {
        Config::from_text(text).expect("the test config parses").0
    }

    /// A `[stepper_z]` with just the `position_max` the endstop lookup reads.
    const Z_ENDSTOP: &str = "[stepper_z]\nposition_max: 200\n";

    // -----------------------------------------------------------------------
    // Reading the section
    // -----------------------------------------------------------------------

    /// `home_xy_position` is the only required option; the rest fall back to
    /// upstream's defaults (`safe_z_home.py:11-20`), and the `position_max` of
    /// the Z endstop section is read to prove it has one (`:18`).
    #[test]
    fn test_the_defaults_are_upstreams() {
        let text = format!("{Z_ENDSTOP}[safe_z_home]\nhome_xy_position: 10, 20\n");
        let config = parse(&text);
        let (wrapper, access) = wrapper(&config, "safe_z_home");

        let options = SafeZHomeOptions::read(&wrapper).expect("the section reads");
        assert_eq!(options.home_xy_position, [10.0, 20.0]);
        assert_eq!(options.z_hop, 0.0);
        assert_eq!(options.z_hop_speed, 15.0);
        assert_eq!(options.speed, 50.0);
        assert!(!options.move_to_previous);
        assert_eq!(options.max_z, 200.0);

        // The defaults are recorded as used, as upstream's `getfloat` does
        // (`klippy/configfile.py:33-36`).
        assert!(access.contains("safe_z_home", "z_hop"));
        assert!(access.contains("safe_z_home", "speed"));
        assert!(!access.contains("safe_z_home", "move_to_previous"));
        assert!(access.contains("stepper_z", "position_max"));
    }

    /// The corpus's own option set (`config/printer-creality-ender3-v2-neo-2022.cfg`)
    /// is read back through the tracker, so `check_unused` accepts the section.
    #[test]
    fn test_every_option_the_corpus_writes_is_recorded_as_read() {
        let text = format!(
            "{Z_ENDSTOP}[safe_z_home]\n\
             home_xy_position: 160,120\n\
             speed: 150\n\
             z_hop: 10\n\
             z_hop_speed: 10\n"
        );
        let config = parse(&text);
        let (wrapper, access) = wrapper(&config, "safe_z_home");

        let options = SafeZHomeOptions::read(&wrapper).expect("the section reads");
        check_unused(&config, &access, &[]).expect("no option is left unread");

        assert_eq!(options.home_xy_position, [160.0, 120.0]);
        assert_eq!(options.speed, 150.0);
        assert_eq!(options.z_hop, 10.0);
        assert_eq!(options.z_hop_speed, 10.0);
    }

    /// A missing `home_xy_position` and one with a single value are both
    /// `getfloatlist(..., count=2)` failures (`safe_z_home.py:11`,
    /// `klippy/configfile.py:99-101`).
    #[test]
    fn test_a_missing_or_single_home_xy_position_is_refused() {
        let text = format!("{Z_ENDSTOP}[safe_z_home]\nspeed: 150\n");
        let config = parse(&text);
        let (wrapper, _) = wrapper(&config, "safe_z_home");
        assert_eq!(
            SafeZHomeOptions::read(&wrapper).unwrap_err().to_string(),
            "Option 'home_xy_position' in section 'safe_z_home' must be specified"
        );

        let text = format!("{Z_ENDSTOP}[safe_z_home]\nhome_xy_position: 160\n");
        let config = parse(&text);
        let (wrapper, _) = wrapper(&config, "safe_z_home");
        assert_eq!(
            SafeZHomeOptions::read(&wrapper).unwrap_err().to_string(),
            "Option 'home_xy_position' in section 'safe_z_home' must have 2 elements"
        );

        let text = format!("{Z_ENDSTOP}[safe_z_home]\nhome_xy_position: 160,abc\n");
        let config = parse(&text);
        let (wrapper, _) = wrapper(&config, "safe_z_home");
        assert_eq!(
            SafeZHomeOptions::read(&wrapper).unwrap_err().to_string(),
            "Unable to parse option 'home_xy_position' in section 'safe_z_home'"
        );
    }

    /// No `[stepper_z]` and no `[carriage z]` is upstream's missing-endstop
    /// error (`safe_z_home.py:15-17`).
    #[test]
    fn test_a_missing_z_endstop_section_is_refused() {
        let config = parse("[safe_z_home]\nhome_xy_position: 160,120\n");
        let (wrapper, _) = wrapper(&config, "safe_z_home");
        assert_eq!(
            SafeZHomeOptions::read(&wrapper).unwrap_err().to_string(),
            "Missing Z endstop config for safe_z_homing"
        );
    }

    /// `manual_probe.lookup_z_endstop_config`'s second half
    /// (`manual_probe.py:28-33`): without `[stepper_z]`, a `[carriage <name>]`
    /// whose `axis` is `z` supplies the endstop section; a carriage on another
    /// axis does not.
    #[test]
    fn test_a_carriage_on_z_supplies_the_endstop_section() {
        let text = "[carriage carriage_x]\naxis: x\nposition_max: 300\n\
                    [carriage carriage_z]\naxis: z\nposition_max: 100\n\
                    [safe_z_home]\nhome_xy_position: 160,120\n";
        let config = parse(text);
        let (wrapper, _) = wrapper(&config, "safe_z_home");
        let options = SafeZHomeOptions::read(&wrapper).expect("the carriage supplies the endstop");
        assert_eq!(options.max_z, 100.0);

        // The `axis` option defaults to the carriage's name (`:31`), so
        // `[carriage z]` is a Z endstop section on its own.
        let text = "[carriage z]\nposition_max: 100\n\
                    [safe_z_home]\nhome_xy_position: 160,120\n";
        let config = parse(text);
        let (wrapper, _) = wrapper(&config, "safe_z_home");
        let options = SafeZHomeOptions::read(&wrapper).expect("the name supplies the axis");
        assert_eq!(options.max_z, 100.0);
    }

    // -----------------------------------------------------------------------
    // The `G28` handler
    // -----------------------------------------------------------------------

    /// A dispatcher to build statements with (the printer behind it is only
    /// needed to construct one).
    fn dispatch() -> GCodeDispatch {
        GCodeDispatch::new(Arc::new(Printer::new(ManualReactor::shared())))
    }

    fn statement(gcode: &GCodeDispatch, params: &[(&str, &str)]) -> GcodeCommand {
        let params = params
            .iter()
            .map(|(name, value)| (name.to_string(), value.to_string()))
            .collect();
        gcode.create_gcode_command("G28", "G28", params)
    }

    /// The toolhead side of `run_g28`, recording every call in order.
    #[derive(Default)]
    struct RecordingHome {
        homed: Mutex<[bool; 3]>,
        position: Mutex<Coord>,
        log: Mutex<Vec<String>>,
    }

    impl RecordingHome {
        fn new(position: Coord, homed: [bool; 3]) -> Arc<Self> {
            Arc::new(Self {
                homed: Mutex::new(homed),
                position: Mutex::new(position),
                log: Mutex::new(Vec::new()),
            })
        }

        fn push(&self, entry: String) {
            self.log
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(entry);
        }

        fn log(&self) -> Vec<String> {
            self.log.lock().unwrap_or_else(|p| p.into_inner()).clone()
        }

        /// Mark the axes a homing statement named as homed, the way the real
        /// toolhead would.
        fn home_named(&self, params: &HashMap<String, String>) {
            let mut homed = self.homed.lock().unwrap_or_else(|p| p.into_inner());
            for (axis, name) in [(X_AXIS, "X"), (Y_AXIS, "Y"), (Z_AXIS, "Z")] {
                if params.contains_key(name) {
                    homed[axis] = true;
                }
            }
        }
    }

    impl HomeOps for RecordingHome {
        fn homed_axes(&self) -> String {
            let homed = self.homed.lock().unwrap_or_else(|p| p.into_inner());
            ["x", "y", "z"]
                .iter()
                .zip(homed.iter())
                .filter(|(_, homed)| **homed)
                .map(|(name, _)| *name)
                .collect()
        }

        fn position(&self) -> Result<Coord, CommandError> {
            Ok(*self.position.lock().unwrap_or_else(|p| p.into_inner()))
        }

        fn manual_move(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
            self.push(format!(
                "move x={:.3} y={:.3} z={:.3} f={:.3}",
                position.x(),
                position.y(),
                position.z(),
                speed
            ));
            *self.position.lock().unwrap_or_else(|p| p.into_inner()) = position;
            Ok(())
        }

        async fn set_position(
            &self,
            position: Coord,
            homing_axes: &[usize],
        ) -> Result<(), CommandError> {
            self.push(format!(
                "set_position z={:.3} homing={homing_axes:?}",
                position.z()
            ));
            *self.position.lock().unwrap_or_else(|p| p.into_inner()) = position;
            let mut homed = self.homed.lock().unwrap_or_else(|p| p.into_inner());
            for axis in homing_axes {
                homed[*axis] = true;
            }
            Ok(())
        }

        fn clear_homing_state(&self, axes: &[usize]) {
            self.push(format!("clear_homing_state {axes:?}"));
            let mut homed = self.homed.lock().unwrap_or_else(|p| p.into_inner());
            for axis in axes {
                homed[*axis] = false;
            }
        }
    }

    /// What the previous `G28` was handed, in order. The axes it named are
    /// marked homed, as the real handler's homing would leave them.
    #[derive(Default)]
    struct Recorded {
        commands: Mutex<Vec<HashMap<String, String>>>,
    }

    fn recording_prev(recorded: Arc<Recorded>, ops: Arc<RecordingHome>) -> CommandHandler {
        Arc::new(move |gcmd: &GcodeCommand| {
            let params = gcmd.get_command_parameters().clone();
            recorded
                .commands
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(params.clone());
            ops.home_named(&params);
            Box::pin(async { Ok::<(), CommandError>(()) })
        })
    }

    fn options(z_hop: f64, move_to_previous: bool) -> SafeZHomeOptions {
        SafeZHomeOptions {
            home_xy_position: [160.0, 120.0],
            z_hop,
            z_hop_speed: 10.0,
            speed: 150.0,
            move_to_previous,
            max_z: 200.0,
        }
    }

    fn recorded(recorded: &Recorded) -> Vec<HashMap<String, String>> {
        recorded
            .commands
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }

    /// A bare `G28` homes XY first — with the synthesised `X0 Y0`
    /// (`safe_z_home.py:59-66`) — then moves to the safe position and homes Z
    /// with a `Z0` statement (`:68-84`).
    #[tokio::test]
    async fn test_g28_without_axes_homes_xy_then_z() {
        let options = options(0.0, false);
        let gcode = dispatch();
        let ops = RecordingHome::new(Coord::new(0.0, 0.0, 0.0, 0.0), [false; 3]);
        let recorded = Arc::new(Recorded::default());
        let prev = recording_prev(Arc::clone(&recorded), Arc::clone(&ops));

        run_g28(
            &options,
            &gcode,
            ops.as_ref(),
            &prev,
            &statement(&gcode, &[]),
        )
        .await
        .expect("the homing runs");

        assert_eq!(
            recorded(&recorded),
            [
                HashMap::from([
                    ("X".to_string(), "0".to_string()),
                    ("Y".to_string(), "0".to_string())
                ]),
                HashMap::from([("Z".to_string(), "0".to_string())]),
            ]
        );
        // The safe move happens between the two statements, at `speed`.
        assert_eq!(
            ops.log(),
            ["move x=160.000 y=120.000 z=0.000 f=150.000".to_string(),]
        );
    }

    /// A `G28 Z` statement skips the XY homing entirely and still refuses to
    /// home Z while X or Y is unhomed (`safe_z_home.py:70-72`).
    #[tokio::test]
    async fn test_g28_z_requires_x_and_y_homed() {
        let options = options(0.0, false);
        let gcode = dispatch();
        let ops = RecordingHome::new(Coord::new(0.0, 0.0, 0.0, 0.0), [false; 3]);
        let recorded = Arc::new(Recorded::default());
        let prev = recording_prev(Arc::clone(&recorded), Arc::clone(&ops));

        let err = run_g28(
            &options,
            &gcode,
            ops.as_ref(),
            &prev,
            &statement(&gcode, &[("Z", "0")]),
        )
        .await
        .unwrap_err();

        assert_eq!(err.to_string(), "Must home X and Y axes first");
        assert!(recorded(&recorded).is_empty(), "no statement was passed on");
        assert!(ops.log().is_empty(), "no move was made");
    }

    /// The z-hop of an unhomed Z pretends the axis is at 0, hops, and then
    /// forgets that homing state again (`safe_z_home.py:39-46`) — the order is
    /// what matters: without the `clear_homing_state` the axis would keep
    /// looking homed after the hop.
    #[tokio::test]
    async fn test_the_unhomed_z_hop_is_undone_after_the_move() {
        let options = options(10.0, false);
        let gcode = dispatch();
        let ops = RecordingHome::new(Coord::new(5.0, 6.0, 3.0, 0.0), [false; 3]);
        let recorded = Arc::new(Recorded::default());
        let prev = recording_prev(Arc::clone(&recorded), Arc::clone(&ops));

        // `G28 Z` on a machine with XY already homed, so the Z branch runs.
        {
            let mut homed = ops.homed.lock().unwrap();
            homed[X_AXIS] = true;
            homed[Y_AXIS] = true;
        }
        run_g28(
            &options,
            &gcode,
            ops.as_ref(),
            &prev,
            &statement(&gcode, &[("Z", "0")]),
        )
        .await
        .expect("the homing runs");

        let log = ops.log();
        assert_eq!(
            log,
            [
                "set_position z=0.000 homing=[2]".to_string(),
                "move x=5.000 y=6.000 z=10.000 f=10.000".to_string(),
                "clear_homing_state [2]".to_string(),
                "move x=160.000 y=120.000 z=10.000 f=150.000".to_string(),
            ]
        );
        // The hop is from the forced z=0, keeping the current XY, at
        // `z_hop_speed`.
        assert!(
            log.iter()
                .position(|entry| entry.starts_with("set_position"))
                < log
                    .iter()
                    .position(|entry| entry.starts_with("clear_homing_state")),
            "the forced homing state must be cleared after the hop"
        );
    }

    /// A homed Z below `z_hop` is lifted to it before the homing move, with no
    /// fake homing state (`safe_z_home.py:47-50`).
    #[tokio::test]
    async fn test_a_homed_z_below_the_hop_is_lifted() {
        let options = options(10.0, false);
        let gcode = dispatch();
        let ops = RecordingHome::new(Coord::new(5.0, 6.0, 3.0, 0.0), [true; 3]);
        let recorded = Arc::new(Recorded::default());
        let prev = recording_prev(Arc::clone(&recorded), Arc::clone(&ops));

        run_g28(
            &options,
            &gcode,
            ops.as_ref(),
            &prev,
            &statement(&gcode, &[("Z", "0")]),
        )
        .await
        .expect("the homing runs");

        assert_eq!(
            ops.log(),
            [
                "move x=5.000 y=6.000 z=10.000 f=10.000".to_string(),
                "move x=160.000 y=120.000 z=10.000 f=150.000".to_string(),
            ]
        );
    }

    /// A homed Z above `z_hop` stays where it is; only the safe XY move
    /// happens (`safe_z_home.py:33-50`'s `else if`).
    #[tokio::test]
    async fn test_a_homed_z_above_the_hop_does_not_move() {
        let options = options(10.0, false);
        let gcode = dispatch();
        let ops = RecordingHome::new(Coord::new(5.0, 6.0, 30.0, 0.0), [true; 3]);
        let recorded = Arc::new(Recorded::default());
        let prev = recording_prev(Arc::clone(&recorded), Arc::clone(&ops));

        run_g28(
            &options,
            &gcode,
            ops.as_ref(),
            &prev,
            &statement(&gcode, &[("Z", "0")]),
        )
        .await
        .expect("the homing runs");

        assert_eq!(
            ops.log(),
            ["move x=160.000 y=120.000 z=30.000 f=150.000".to_string()]
        );
    }

    /// `z_hop == 0.0` skips the whole hop block: no forced position, no move,
    /// no cleared homing state (`safe_z_home.py:33`).
    #[tokio::test]
    async fn test_z_hop_zero_skips_the_hop() {
        let options = options(0.0, false);
        let gcode = dispatch();
        let ops = RecordingHome::new(Coord::new(5.0, 6.0, 3.0, 0.0), [false, false, false]);
        let recorded = Arc::new(Recorded::default());
        let prev = recording_prev(Arc::clone(&recorded), Arc::clone(&ops));

        run_g28(
            &options,
            &gcode,
            ops.as_ref(),
            &prev,
            &statement(&gcode, &[]),
        )
        .await
        .expect("the homing runs");

        let log = ops.log();
        assert!(
            !log.iter().any(|entry| entry.starts_with("set_position")),
            "an unhomed Z is not forced anywhere: {log:?}"
        );
        assert!(
            !log.iter()
                .any(|entry| entry.starts_with("clear_homing_state")),
            "nothing was forced, so nothing is cleared: {log:?}"
        );
        assert_eq!(
            log,
            ["move x=160.000 y=120.000 z=3.000 f=150.000".to_string()],
            "only the safe XY move is left"
        );
    }

    /// After homing Z at the safe position, Z is hopped again when it ended up
    /// below `z_hop` (`safe_z_home.py:80-84`).
    #[tokio::test]
    async fn test_z_is_hopped_again_after_homing() {
        let options = options(10.0, false);
        let gcode = dispatch();
        let ops = RecordingHome::new(Coord::new(5.0, 6.0, 3.0, 0.0), [true; 3]);
        let recorded = Arc::new(Recorded::default());
        // A pressure probe homes Z down to 0, where the second hop picks it up.
        let prev: CommandHandler = {
            let ops = Arc::clone(&ops);
            Arc::new(move |_gcmd: &GcodeCommand| {
                let ops = Arc::clone(&ops);
                Box::pin(async move {
                    let mut position = ops.position.lock().unwrap();
                    position.set_axis(Z_AXIS, 0.0);
                    Ok(())
                })
            })
        };

        run_g28(
            &options,
            &gcode,
            ops.as_ref(),
            &prev,
            &statement(&gcode, &[("Z", "0")]),
        )
        .await
        .expect("the homing runs");

        assert_eq!(
            ops.log().last(),
            Some(&"move x=160.000 y=120.000 z=10.000 f=10.000".to_string())
        );
    }

    /// `move_to_previous` sends XY back to where the statement found them,
    /// at `speed`, after the Z homing (`safe_z_home.py:86-88`).
    #[tokio::test]
    async fn test_move_to_previous_returns_xy() {
        let options = options(0.0, true);
        let gcode = dispatch();
        let ops = RecordingHome::new(Coord::new(11.0, 22.0, 30.0, 0.0), [true; 3]);
        let recorded = Arc::new(Recorded::default());
        let prev = recording_prev(Arc::clone(&recorded), Arc::clone(&ops));

        run_g28(
            &options,
            &gcode,
            ops.as_ref(),
            &prev,
            &statement(&gcode, &[("Z", "0")]),
        )
        .await
        .expect("the homing runs");

        assert_eq!(
            ops.log(),
            [
                "move x=160.000 y=120.000 z=30.000 f=150.000".to_string(),
                "move x=11.000 y=22.000 z=30.000 f=150.000".to_string(),
            ]
        );
    }

    /// A statement that names only X passes that through: no safe move, no Z
    /// homing (`safe_z_home.py:52-66`).
    #[tokio::test]
    async fn test_g28_x_only_passes_through() {
        let options = options(10.0, true);
        let gcode = dispatch();
        let ops = RecordingHome::new(Coord::new(11.0, 22.0, 30.0, 0.0), [true; 3]);
        let recorded = Arc::new(Recorded::default());
        let prev = recording_prev(Arc::clone(&recorded), Arc::clone(&ops));

        run_g28(
            &options,
            &gcode,
            ops.as_ref(),
            &prev,
            &statement(&gcode, &[("X", "0")]),
        )
        .await
        .expect("the statement runs");

        assert_eq!(
            recorded(&recorded),
            [HashMap::from([("X".to_string(), "0".to_string())])]
        );
        assert!(ops.log().is_empty());
    }

    // -----------------------------------------------------------------------
    // Loading
    // -----------------------------------------------------------------------

    /// A minimal cartesian printer the loader can build a toolhead from, with
    /// `extra` appended (the same sections `load.rs`'s cartesian test uses).
    fn loadable(extra: &str) -> Config {
        parse(&format!(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_y]\nstep_pin: PA2\ndir_pin: PA3\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z]\nstep_pin: PA4\ndir_pin: PA5\nrotation_distance: 8\nmicrosteps: 16\nposition_max: 200\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n\
             {extra}"
        ))
    }

    /// The load order is load-bearing: the toolhead registers `G28` when it
    /// loads (`toolhead.rs:603-606`), so the section must load after it
    /// (`order = 70` against the toolhead's `order = 60`, both `phase = late`)
    /// to take that handler away rather than have its own overwritten.
    #[test]
    fn test_g28_is_replaced_after_the_toolhead_registers_it() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = loadable("[safe_z_home]\nhome_xy_position: 160,120\n");
        printer.load_config(&config).expect("the fixture loads");

        let safe_z_home = printer
            .lookup_object_as::<SafeZHoming>("safe_z_home")
            .expect("the loader registers the section");
        assert!(
            safe_z_home
                .prev_g28
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .is_some(),
            "the toolhead's G28 must have been taken, not left as None"
        );
    }

    /// `[homing_override]` owns `G28` too, so the pair is refused
    /// (`safe_z_home.py:26-28`).
    #[test]
    fn test_homing_override_and_safe_z_homing_cannot_share_g28() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let config = loadable(
            "[homing_override]\ngcode: G28\n\
             [safe_z_home]\nhome_xy_position: 160,120\n",
        );
        let err = printer.load_config(&config).unwrap_err();
        assert_eq!(
            err.to_string(),
            "homing_override and safe_z_homing cannot be used simultaneously"
        );
    }
}
