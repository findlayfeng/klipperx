//! `[homing_override]` — run a user script in place of a normal `G28`
//! (upstream `klippy/extras/homing_override.py:65 def load_config`).
//!
//! | option | default | role |
//! |---|---|---|
//! | `axes` | `XYZ` | upper-cased; the axes whose `G28` triggers the override (`:16`) |
//! | `set_position_x/y/z` | none | forced position applied before the script (`:14-15`) |
//! | `gcode` | — (required) | the homing script, compiled as a macro template (`:18`) |
//!
//! The section takes `G28` away from the toolhead and installs its own handler
//! (`:18-20`): a `G28` naming no axis, or one naming an axis `axes` selects,
//! runs the `gcode` template instead of homing (`:33-46`), after the configured
//! `set_position_*` axes have been forced and marked homed (`:54-60`); every
//! other statement goes to the previous handler unchanged.
//!
//! **Load order is load-bearing** (`phase = late, order = 70`): the toolhead's
//! `[printer]` section (`order = 60`, same phase) is what registers `G28`
//! (`toolhead.rs:603-606`); loaded earlier, this handler would be overwritten.
//!
//! The body renders against the `G28` statement's `params`/`rawparams` and the
//! usual macro globals (`:57-58`). A `G28` *inside* the script reaches the
//! previous handler — upstream's `in_script` flag (`:22-24`, `:61-63`).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::gcode_macro::PrinterGCodeMacro;
use crate::core::klippy::extras::template::{Builtin, Context, PrinterView, Rt, Template};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::Coord;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!(
    "homing_override",
    order = 70,
    phase = late,
    load = load_config
);

/// The toolhead object, as the loader registers `[printer]`.
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The `G28` axis parameters and the axis index each names (`:33-46`).
const AXES: [(usize, &str); 3] = [(0, "X"), (1, "Y"), (2, "Z")];

/// The option values (`homing_override.py:14-16`).
#[derive(Debug, PartialEq, Clone)]
struct HomingOverrideOptions {
    /// `axes`, upper-cased (`:16`).
    axes: String,
    /// `set_position_x`/`_y`/`_z` in axis order; `None` where unset
    /// (`:14-15`).
    start_pos: [Option<f64>; 3],
}

impl HomingOverrideOptions {
    /// Read the section in upstream's order (`:14-16`); `gcode` is the
    /// template's, read by [`HomingOverride::new`] (`:18`).
    ///
    /// # Errors
    /// A `set_position_*` that is not a number (upstream's `getfloat`).
    fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let mut start_pos = [None; 3];
        for (index, axis) in "xyz".char_indices() {
            let option = format!("set_position_{axis}");
            start_pos[index] = match config.get_str(&option) {
                Some(raw) => Some(parse_position(&option, &config.identifier(), &raw)?),
                None => None,
            };
        }
        let axes = config.get("axes", Some("XYZ"))?.to_uppercase();
        Ok(Self { axes, start_pos })
    }

    /// The trigger axes as a mask (`:12` + `cmd_G28`'s `for axis in
    /// self.axes`); letters outside `x`/`y`/`z` select nothing.
    fn axis_mask(&self) -> [bool; 3] {
        let mut mask = [false; 3];
        for letter in self.axes.chars() {
            match letter {
                'X' => mask[0] = true,
                'Y' => mask[1] = true,
                'Z' => mask[2] = true,
                _ => {}
            }
        }
        mask
    }

    /// Whether a `G28` with the given parameter axes runs the override
    /// (`:33-46`): no axis named means the whole override, otherwise any named
    /// axis that `axes` selects.
    fn overrides(&self, requested: &[bool; 3]) -> bool {
        if requested.iter().all(|requested| !requested) {
            return true;
        }
        let mask = self.axis_mask();
        mask.iter()
            .zip(requested)
            .any(|(selected, asked)| *selected && *asked)
    }

    /// The forced start position (`:54-60`): every axis with a
    /// `set_position_*` takes that value, and the axes so set are the ones the
    /// toolhead is told are homed.
    fn start_position(&self, mut position: Coord) -> (Coord, Vec<usize>) {
        let mut homing_axes = Vec::new();
        for (axis, location) in self.start_pos.iter().enumerate() {
            if let Some(location) = location {
                position.set_axis(axis, *location);
                homing_axes.push(axis);
            }
        }
        (position, homing_axes)
    }
}

/// One `set_position_*` value — upstream's `getfloat` (`configfile.py`).
///
/// # Errors
/// `Unable to parse option '<option>' in section '<identifier>'`.
fn parse_position(option: &str, identifier: &str, raw: &str) -> Result<f64, ConfigError> {
    raw.trim().parse::<f64>().map_err(|_| {
        ConfigError::new(format!(
            "Unable to parse option '{option}' in section '{identifier}'"
        ))
    })
}

/// What the installed handler needs: the option values, the compiled script,
/// and the recursion flag (`:22-24`).
struct OverrideCore {
    options: HomingOverrideOptions,
    template: Template,
    in_script: AtomicBool,
}

impl OverrideCore {
    fn in_script(&self) -> bool {
        self.in_script.load(Ordering::SeqCst)
    }

    /// Claim the run, releasing the flag however it ends — upstream's
    /// `try/finally` (`:61-63`).
    fn enter(&self) -> ScriptRun<'_> {
        self.in_script.store(true, Ordering::SeqCst);
        ScriptRun(&self.in_script)
    }
}

/// A script run in progress: clears `in_script` when dropped.
struct ScriptRun<'a>(&'a AtomicBool);

impl Drop for ScriptRun<'_> {
    fn drop(&mut self) {
        self.0.store(false, Ordering::SeqCst);
    }
}

/// The toolhead effects the `G28` handler performs, so the override is testable
/// without a machine (the `safe_z_home.rs` `HomeOps` pattern).
trait HomingEffects {
    /// `toolhead.get_position()`; "Printer is not ready" before connect.
    ///
    /// # Errors
    /// As the toolhead reports it.
    fn position(&self) -> Result<Coord, CommandError>;
    /// `toolhead.set_position(position, homing_axes)`.
    ///
    /// # Errors
    /// "Printer is not ready" before connect, or a failed step flush.
    async fn set_position(
        &self,
        position: Coord,
        homing_axes: &[usize],
    ) -> Result<(), CommandError>;
}

/// The [`HomingEffects`] wiring against the real toolhead.
struct LiveHoming {
    toolhead: Arc<ToolHeadObject>,
}

impl HomingEffects for LiveHoming {
    fn position(&self) -> Result<Coord, CommandError> {
        self.toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    async fn set_position(
        &self,
        position: Coord,
        homing_axes: &[usize],
    ) -> Result<(), CommandError> {
        self.toolhead.set_position(position, homing_axes).await
    }
}

/// The `G28` replacement itself (`:21-63`).
///
/// # Errors
/// Whatever the previous handler reports, whatever the toolhead refuses, or a
/// template that does not render or a script that does not run.
async fn run_g28<O: HomingEffects + ?Sized>(
    core: &OverrideCore,
    gcode: &GCodeDispatch,
    printer: &Arc<Printer>,
    ops: &O,
    prev_g28: &CommandHandler,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    // Was called recursively - invoke the real G28 command (`:22-24`).
    if core.in_script() {
        return prev_g28(gcmd).await;
    }

    // No axis named means the override; otherwise any axis `axes` selects
    // (`:26-46`).
    let mut requested = [false; 3];
    for (axis, name) in AXES {
        requested[axis] = gcmd.get_command_parameters().contains_key(name);
    }
    if !core.options.overrides(&requested) {
        return prev_g28(gcmd).await;
    }

    // Calculate forced position (if configured) (`:48-60`).
    let (position, homing_axes) = core.options.start_position(ops.position()?);
    ops.set_position(position, &homing_axes).await?;

    // Perform homing (`:52-60`).
    let mut context = template_context(printer, gcmd);
    let script = core
        .template
        .render(&mut context)
        .map_err(|error| CommandError::new(error.to_string()))?;
    let _run = core.enter();
    gcode.run_script_from_command(&script).await
}

/// The context `run_gcode_from_command` gets (`:56-58`): the globals a macro
/// body sees plus the statement's `params` / `rawparams`.
fn template_context(printer: &Arc<Printer>, gcmd: &GcodeCommand) -> Context {
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
    let params: serde_json::Map<String, Value> = gcmd
        .get_command_parameters()
        .iter()
        .map(|(key, value)| (key.clone(), json!(value)))
        .collect();
    context.insert("params", Rt::Json(Value::Object(params)));
    context.insert(
        "rawparams",
        Rt::Json(Value::String(gcmd.get_raw_command_parameters())),
    );
    context
}

/// The `[homing_override]` section: the `G28` wrapper's registration.
///
/// The options and the compiled script are not kept here — the installed
/// handler captures them, and it outlives any read of this object.
pub struct HomingOverride {
    /// The handler this section replaced — the toolhead's. Nothing consumes the
    /// value after registration, hence the field-level `allow`; the tests read
    /// it to pin the load order.
    #[allow(dead_code)]
    prev_g28: Arc<Mutex<Option<CommandHandler>>>,
}

impl HomingOverride {
    /// Read the section, compile the script and install the `G28` wrapper
    /// (`:10-20`).
    ///
    /// # Errors
    /// As [`HomingOverrideOptions::read`], a missing or unparsable `gcode`, a
    /// `G28` this section cannot chain to (loaded before the toolhead), or a
    /// name already taken.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let options = HomingOverrideOptions::read(config)?;
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");
        // `gcode_macro.load_template(config, 'gcode')` (`:17-18`).
        let template = PrinterGCodeMacro::ensure(printer)?.load_template(config, "gcode", None)?;
        let core = Arc::new(OverrideCore {
            options,
            template,
            in_script: AtomicBool::new(false),
        });

        // Take the previous `G28` so this section can chain to it (`:19-20`).
        // `section!` loads this after the toolhead (`toolhead.rs:603-606`);
        // without it there would be nothing to home through, so a missing one
        // is a wiring error rather than a silently dropped `G28`.
        let prev = gcode.unregister_command("G28").ok_or_else(|| {
            ConfigError::new("homing_override must be loaded after `G28` is registered")
        })?;
        let prev_g28 = Arc::new(Mutex::new(Some(prev)));

        let handler: CommandHandler = {
            let core = Arc::clone(&core);
            let prev_g28 = Arc::clone(&prev_g28);
            // Weak: this handler lives in the dispatcher's command table, so a
            // strong handle would be a reference cycle (`safe_z_home.rs`).
            let gcode = Arc::downgrade(&gcode);
            let printer = Arc::downgrade(printer);
            Arc::new(move |gcmd: &GcodeCommand| {
                let core = Arc::clone(&core);
                let prev_g28 = Arc::clone(&prev_g28);
                let (gcode, printer) = (gcode.clone(), printer.clone());
                Box::pin(async move {
                    let not_ready = || CommandError::new("Printer is not ready");
                    let gcode = gcode.upgrade().ok_or_else(not_ready)?;
                    let printer = printer.upgrade().ok_or_else(not_ready)?;
                    let toolhead = printer
                        .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)
                        .ok_or_else(not_ready)?;
                    let prev = prev_g28
                        .lock()
                        .unwrap_or_else(|poison| poison.into_inner())
                        .clone()
                        .ok_or_else(not_ready)?;
                    let ops = LiveHoming { toolhead };
                    run_g28(&core, &gcode, &printer, &ops, &prev, gcmd).await
                })
            })
        };
        // No help text: upstream leaves the previous one in place. The axes are
        // the parameters `run_g28` reads, so declaring them keeps the list the
        // toolhead's `G28` put on the re-registration.
        gcode
            .register_command_with_params("G28", handler, None, &["X", "Y", "Z"], false)
            .map_err(ConfigError::new)?;

        Ok(Self { prev_g28 })
    }
}

impl PrinterObject for HomingOverride {
    /// Upstream's `HomingOverride` defines no `get_status`; like the sections it
    /// wraps, the port reports an empty object.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }
}

/// The factory `section!` names (`homing_override.py:65 def load_config`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(HomingOverride::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::{check_unused, Config, ConfigSection, ConfigValue};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::gcode::sync;
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[homing_override]` section with the given options.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("homing_override", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    fn printer() -> Arc<Printer> {
        Arc::new(Printer::new(ManualReactor::shared()))
    }

    /// The `gcode` option, loaded through the shared template holder the way
    /// [`HomingOverride::new`] loads it.
    fn load_template(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
    ) -> Result<Template, ConfigError> {
        PrinterGCodeMacro::ensure(printer)?.load_template(config, "gcode", None)
    }

    /// The corpus option set (`test/klippy/sdcard_loop.cfg:9-14`, and
    /// `kit-zav3d-2019.cfg:141`) is read back through the tracker, so
    /// `check_unused` accepts the section.
    #[test]
    fn every_option_the_section_writes_is_recorded_as_read() {
        let text = "[homing_override]
axes: xyz
set_position_x: 0
set_position_y: 0
set_position_z: 0
gcode:
  G92 X0 Y0 Z0
";
        let (config, _) = Config::from_text(text).expect("the section parses");
        let sect = config.get_section("homing_override").expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(sect, Arc::clone(&access));

        let parsed = HomingOverrideOptions::read(&wrapper).expect("the section reads");
        let template = load_template(&wrapper, &printer()).expect("the body compiles");
        check_unused(&config, &access, &[]).expect("no option is left unread");

        assert_eq!(parsed.axes, "XYZ"); // upper-cased (`:16`)
        assert_eq!(parsed.start_pos, [Some(0.), Some(0.), Some(0.)]);
        // Named after its section and option (`gcode_macro.py:47-52`).
        assert_eq!(template.name(), "homing_override:gcode");
    }

    /// With only the mandatory script, `axes` falls back to upstream's `XYZ`
    /// and no position is forced (`:14-16`); the default is recorded as used
    /// (`config/wrapper.rs:179-190`).
    #[test]
    fn the_defaults_are_upstreams() {
        let text = "[homing_override]\ngcode:\n  G28\n";
        let (config, _) = Config::from_text(text).expect("the section parses");
        let sect = config.get_section("homing_override").expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(sect, Arc::clone(&access));

        let parsed = HomingOverrideOptions::read(&wrapper).expect("the section reads");
        let template = load_template(&wrapper, &printer()).expect("the body compiles");
        check_unused(&config, &access, &[]).expect("no option is left unread");

        assert_eq!(parsed.axes, "XYZ");
        assert_eq!(parsed.start_pos, [None, None, None]);
        assert!(access.contains("homing_override", "axes"));
        assert!(!access.contains("homing_override", "set_position_x"));

        // The body is one plain statement, so no context is needed.
        let mut context = Context::new();
        assert_eq!(
            template
                .render(&mut context)
                .expect("the body renders")
                .trim(),
            "G28"
        );
    }

    /// A `set_position_*` that is not a number is refused with upstream's
    /// `getfloat` wording, and a missing `gcode` with the loader's
    /// required-option wording (`:14,18`).
    #[test]
    fn malformed_options_are_refused_with_upstream_wording() {
        let sect = section(&[("gcode", "G28"), ("set_position_x", "abc")]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            HomingOverrideOptions::read(&config)
                .unwrap_err()
                .to_string(),
            "Unable to parse option 'set_position_x' in section 'homing_override'"
        );

        let sect = section(&[("axes", "XYZ")]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            load_template(&config, &printer()).unwrap_err().to_string(),
            "Option 'gcode' in section 'homing_override' must be specified"
        );
    }

    /// The `G28` statement decides against the `axes` mask exactly as
    /// upstream's `cmd_G28` (`:33-46`).
    #[test]
    fn a_g28_statement_decides_against_the_axes_mask() {
        let override_xyz = HomingOverrideOptions {
            axes: "XYZ".to_string(),
            start_pos: [Some(0.); 3],
        };
        let requested = |x: bool, y: bool, z: bool| [x, y, z];

        // No axis named → the override runs (`no_axis` branch).
        assert!(override_xyz.overrides(&requested(false, false, false)));
        // Any requested axis that `axes` selects → override.
        assert!(override_xyz.overrides(&requested(true, false, false)));
        assert!(override_xyz.overrides(&requested(false, true, true)));
        // `axes: Z` only overrides a Z statement.
        let override_z = HomingOverrideOptions {
            axes: "Z".to_string(),
            start_pos: [None; 3],
        };
        assert!(!override_z.overrides(&requested(true, false, false)));
        assert!(override_z.overrides(&requested(false, false, true)));
        // The mask comes from the upper-cased option; unknown letters in it
        // select nothing.
        assert_eq!(override_xyz.axis_mask(), [true, true, true]);
        let odd = HomingOverrideOptions {
            axes: "XW".to_string(),
            start_pos: [None; 3],
        };
        assert_eq!(odd.axis_mask(), [true, false, false]);
    }

    // -----------------------------------------------------------------------
    // The `G28` handler
    // -----------------------------------------------------------------------

    /// `[mcu]`, three axes and `[printer]` — what a `G28` needs to exist — with
    /// `extra` appended.
    fn loadable(extra: &str) -> Config {
        Config::from_text(&format!(
            "[mcu]\nserial: /dev/not-opened-yet\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_y]\nstep_pin: PA2\ndir_pin: PA3\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z]\nstep_pin: PA4\ndir_pin: PA5\nrotation_distance: 8\nmicrosteps: 16\nposition_max: 200\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n\
             {extra}"
        ))
        .expect("the fixture parses")
        .0
    }

    /// The load order is load-bearing: the section must load after the toolhead
    /// (`order = 70` against `60`, both `phase = late`) to take its `G28` away
    /// rather than have its own overwritten.
    #[test]
    fn g28_is_replaced_after_the_toolhead_registers_it() {
        let printer = printer();
        let config = loadable("[homing_override]\ngcode:\n  G92 X0 Y0 Z0\n");
        printer.load_config(&config).expect("the fixture loads");

        let homing_override = printer
            .lookup_object_as::<HomingOverride>("homing_override")
            .expect("the loader registers the section");
        assert!(
            homing_override
                .prev_g28
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .is_some(),
            "the toolhead's G28 must have been taken, not left as None"
        );
        assert_eq!(homing_override.get_status(0.0), json!({}));
    }

    /// `[safe_z_home]` owns `G28` too, so the pair is refused
    /// (`safe_z_home.py:26-28`).
    #[test]
    fn homing_override_and_safe_z_homing_cannot_share_g28() {
        let printer = printer();
        let config = loadable(
            "[homing_override]\ngcode: G28\n\
             [safe_z_home]\nhome_xy_position: 160,120\n",
        );
        assert_eq!(
            printer.load_config(&config).unwrap_err().to_string(),
            "homing_override and safe_z_homing cannot be used simultaneously"
        );
    }

    /// The toolhead side of `run_g28`, plus the dispatcher the script runs on.
    /// Both write into one log, so the forced position and the script's effects
    /// are ordered.
    struct Rig {
        log: Arc<Mutex<Vec<String>>>,
        position: Mutex<Coord>,
        gcode: Arc<GCodeDispatch>,
        printer: Arc<Printer>,
    }

    impl Rig {
        /// A ready dispatcher over a fresh printer, with `MARK` recording the
        /// parameters the script renders into the log.
        fn new() -> Arc<Self> {
            let printer = printer();
            let gcode = Arc::new(GCodeDispatch::new(Arc::clone(&printer)));
            printer.send_event(&KlippyEvent::KlippyReady);
            let log: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
            {
                let log = Arc::clone(&log);
                gcode
                    .register_command(
                        "MARK",
                        sync(move |gcmd: &GcodeCommand| {
                            let mut pairs: Vec<String> = gcmd
                                .get_command_parameters()
                                .iter()
                                .map(|(key, value)| format!("{key}={value}"))
                                .collect();
                            // A `HashMap` has no order; the log has to be stable.
                            pairs.sort();
                            log.lock()
                                .unwrap_or_else(|p| p.into_inner())
                                .push(format!("script {}", pairs.join(" ")));
                            Ok(())
                        }),
                        None,
                        false,
                    )
                    .expect("MARK registers");
            }
            Arc::new(Self {
                log,
                position: Mutex::new(Coord::new(1.0, 2.0, 3.0, 0.0)),
                gcode,
                printer,
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

        fn statement(&self, params: &[(&str, &str)]) -> GcodeCommand {
            let params = params
                .iter()
                .map(|(name, value)| ((*name).to_string(), (*value).to_string()))
                .collect();
            self.gcode.create_gcode_command("G28", "G28 X0", params)
        }

        fn position(&self) -> Coord {
            *self.position.lock().unwrap_or_else(|p| p.into_inner())
        }

        /// Install the override's handler for a nested `G28`: the real one's
        /// body without the toolhead look-up, which needs a connected machine.
        fn install(self: &Arc<Self>, core: &Arc<OverrideCore>, prev: &CommandHandler) {
            let (core, rig, prev) = (Arc::clone(core), Arc::clone(self), Arc::clone(prev));
            let handler: CommandHandler = Arc::new(move |gcmd: &GcodeCommand| {
                let (core, rig, prev) = (Arc::clone(&core), Arc::clone(&rig), Arc::clone(&prev));
                Box::pin(async move {
                    run_g28(&core, &rig.gcode, &rig.printer, rig.as_ref(), &prev, gcmd).await
                })
            });
            self.gcode
                .register_command_with_params("G28", handler, None, &["X", "Y", "Z"], false)
                .expect("G28 registers");
        }
    }

    impl HomingEffects for Rig {
        fn position(&self) -> Result<Coord, CommandError> {
            Ok(*self.position.lock().unwrap_or_else(|p| p.into_inner()))
        }

        async fn set_position(
            &self,
            position: Coord,
            homing_axes: &[usize],
        ) -> Result<(), CommandError> {
            self.push(format!(
                "set_position x={:.3} homing={homing_axes:?}",
                position.x()
            ));
            *self.position.lock().unwrap_or_else(|p| p.into_inner()) = position;
            Ok(())
        }
    }

    /// A previous `G28` that records the statement it was handed.
    fn recording_prev(log: &Arc<Mutex<Vec<String>>>) -> CommandHandler {
        let log = Arc::clone(log);
        Arc::new(move |gcmd: &GcodeCommand| {
            let entry = format!("prev {}", gcmd.commandline());
            log.lock().unwrap_or_else(|p| p.into_inner()).push(entry);
            Box::pin(async { Ok::<(), CommandError>(()) })
        })
    }

    fn core(axes: &str, start_pos: [Option<f64>; 3], body: &str) -> Arc<OverrideCore> {
        Arc::new(OverrideCore {
            options: HomingOverrideOptions {
                axes: axes.to_string(),
                start_pos,
            },
            template: Template::parse("homing_override:gcode", body).expect("the body compiles"),
            in_script: AtomicBool::new(false),
        })
    }

    /// A `G28` the `axes` mask selects renders the script — against the
    /// statement's own `params`/`rawparams` (`:57-58`) — and runs it after the
    /// forced start position (`:48-60`).
    #[tokio::test]
    async fn the_script_runs_after_the_forced_start_position() {
        let rig = Rig::new();
        let core = core(
            "XYZ",
            [Some(10.0), None, None],
            "MARK VALUE={params.X} RAW={rawparams}",
        );
        let prev = recording_prev(&rig.log);

        run_g28(
            &core,
            &rig.gcode,
            &rig.printer,
            rig.as_ref(),
            &prev,
            &rig.statement(&[("X", "0")]),
        )
        .await
        .expect("the override runs");

        assert_eq!(
            rig.log(),
            [
                "set_position x=10.000 homing=[0]".to_string(),
                "script RAW=X0 VALUE=0".to_string(),
            ]
        );
    }

    /// A `G28` naming no axis of `axes` is homed by the previous handler:
    /// `axes: Z` leaves `G28 X` alone (`:39-46`).
    #[tokio::test]
    async fn an_axis_outside_axes_is_homed_by_the_previous_handler() {
        let rig = Rig::new();
        let core = core("Z", [Some(10.0), None, None], "MARK");
        let prev = recording_prev(&rig.log);

        run_g28(
            &core,
            &rig.gcode,
            &rig.printer,
            rig.as_ref(),
            &prev,
            &rig.statement(&[("X", "0")]),
        )
        .await
        .expect("the statement passes through");

        // Nothing of the override ran: no forced position, no script.
        assert_eq!(rig.log(), ["prev G28 X0".to_string()]);
        assert_eq!(rig.position().z(), 3.0);
    }

    /// A `G28` inside the script reaches the previous handler instead of
    /// re-entering the override, and the flag is cleared once it ends
    /// (`:22-24`, `:61-63`).
    #[tokio::test]
    async fn a_g28_inside_the_script_reaches_the_previous_handler() {
        let rig = Rig::new();
        let core = core("XYZ", [None; 3], "G28 Y0");
        let prev = recording_prev(&rig.log);
        rig.install(&core, &prev);

        rig.gcode
            .run_script_from_command("G28")
            .await
            .expect("the override runs");

        // One forced position (the outer statement's) and one homing through
        // the previous handler: the inner `G28 Y0` did not re-enter.
        assert_eq!(
            rig.log(),
            [
                "set_position x=1.000 homing=[]".to_string(),
                "prev G28 Y0".to_string(),
            ]
        );
        assert!(!core.in_script(), "the run releases the recursion flag");
    }
}
