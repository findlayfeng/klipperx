//! `[input_shaper]` — the per-axis shaper parameters and `SET_INPUT_SHAPER`
//! (upstream `klippy/extras/input_shaper.py`).
//!
//! | option | default | role |
//! |---|---|---|
//! | `shaper_type` | `mzv` | the shaper type every axis defaults to |
//! | `shaper_type_<axis>` | `shaper_type` | one axis's shaper type |
//! | `shaper_freq_<axis>` | `0` | one axis's shaper frequency; `0` = no shaping |
//! | `damping_ratio_<axis>` | `0.1` | one axis's damping ratio |
//!
//! The section carries no options when an axis takes the defaults, and a shaper
//! type may carry arguments (`mzv(5,0.6)`, `ei(v_tol=0.02)`) — see
//! [`shaper_defs`] for the coefficients and the argument text.
//!
//! With `dual_carriage(s)` in the config, upstream refuses an `[input_shaper]`
//! that configures an axis and leaves the shaping to the `dual_carriage`
//! module; `SET_INPUT_SHAPER` at run time is not refused there, which is how the
//! corpus's `hybrid_corexy_dual_carriage.test` turns shaping on.
//!
//! # Differences from upstream
//!
//! The parameters are validated, reported, and *not applied*: upstream swaps the
//! stepper kinematics of every toolhead stepper for an input-shaper one
//! (`chelper`'s `input_shaper_alloc` / `input_shaper_set_sk` /
//! `input_shaper_set_shaper_params`, `input_shaper.py:137-165`) and re-points
//! them when a dual carriage changes the kinematics. Neither that layer nor the
//! toolhead/stepper plumbing it needs is ported here, so
//! [`InputShaper::update_input_shaping`] is the seam where they would be
//! reached, and the motion queue's scan-window accounting it ends with is
//! [`recompute_scan_windows`] — a documented no-op this host has no mechanism
//! for. `SET_INPUT_SHAPER` therefore changes the reported parameters only.
//!
//! The same layer is what upstream re-points when a dual carriage changes the
//! kinematics (`_update_kinematics`, `input_shaper.py:153-168`, on
//! `dual_carriage:update_kinematics`); `idex_modes` fires no such event here
//! (`KlippyEvent::DualCarriageUpdateKinematics` is declared, but nothing sends
//! it), so no handler is registered and there is nothing to re-point.

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::shaper_defs;
use crate::core::klippy::gcode::{
    parse_float, sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("input_shaper", order = 30, load = load_config);

/// Upstream's complaint about an `[input_shaper]` that configures an axis while
/// dual carriage(s) are enabled (`input_shaper.py:128-134`).
const DUAL_CARRIAGE_CONFIG_ERROR: &str = "Input shaper parameters cannot be \
configured via [input_shaper] section with dual_carriage(s) enabled. Refer to \
Klipper documentation on how to configure input shaper for dual_carriage(s).";

/// One axis's parameters (`input_shaper.InputShaperParams`, `:11-63`).
struct InputShaperParams {
    /// The lower-case axis letter, `x`, `y` or `z` (`input_shaper.py:13`).
    axis: char,
    /// The shaper type, as the config or the command wrote it — a name's
    /// arguments included (`mzv(5,0.6)`).
    shaper_type: String,
    /// The damping ratio, [`shaper_defs::DEFAULT_DAMPING_RATIO`] by default.
    damping_ratio: f64,
    /// The shaper frequency; `0` means this axis is not shaped.
    shaper_freq: f64,
}

impl InputShaperParams {
    /// Read one axis's parameters (`input_shaper.py:13-25`).
    ///
    /// # Errors
    /// [`ConfigError`] for a shaper type no shaper carries, for an option
    /// outside its bounds, and for a shaper the parameters cannot build.
    fn new(axis: char, config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let shaper_type = config.get("shaper_type", Some("mzv"))?;
        let shaper_type = config.get(&format!("shaper_type_{axis}"), Some(&shaper_type))?;
        let sconfig = shaper_defs::get_shaper_cfg(&shaper_type)
            .ok_or_else(|| ConfigError::new(format!("Unsupported shaper type: {shaper_type}")))?;
        let damping_ratio = config.get_float_bounded(
            &format!("damping_ratio_{axis}"),
            Some(shaper_defs::DEFAULT_DAMPING_RATIO),
            Some(0.),
            Some(sconfig.max_damping_ratio),
            None,
            None,
        )?;
        let shaper_freq = config.get_float_bounded(
            &format!("shaper_freq_{axis}"),
            Some(0.),
            Some(0.),
            None,
            None,
            None,
        )?;
        let params = Self {
            axis,
            shaper_type,
            damping_ratio,
            shaper_freq,
        };
        // Validate the input shaper, as upstream's `self.get_shaper()` in
        // `__init__` does (with the config writer's error).
        params
            .shaper_n(
                &params.shaper_type,
                params.shaper_freq,
                params.damping_ratio,
            )
            .map_err(|err| ConfigError::new(err.to_string()))?;
        Ok(params)
    }

    /// Update this axis from a `SET_INPUT_SHAPER` line
    /// (`input_shaper.py:27-48`).
    ///
    /// # Errors
    /// [`CommandError`] for a shaper type no shaper carries, for a damping
    /// ratio above the shaper's maximum, for a parameter outside its bound, or
    /// for a shaper the parameters cannot build.
    fn update(&mut self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let axis = self.axis.to_ascii_uppercase();
        // `SHAPER_TYPE` applies to whichever axis this is; a command that sets
        // one axis names it as `SHAPER_TYPE_<axis>`.
        let shaper_type = match gcmd.get_str("SHAPER_TYPE") {
            Ok(shaper_type) => shaper_type,
            Err(_) => gcmd.get_str_default(&format!("SHAPER_TYPE_{axis}"), &self.shaper_type),
        };
        let shaper_name = shaper_type.to_lowercase();
        let sconfig = shaper_defs::get_shaper_cfg(&shaper_name)
            .ok_or_else(|| CommandError::new(format!("Unsupported shaper type: {shaper_type}")))?;
        let damping_ratio = gcmd.get(
            &format!("DAMPING_RATIO_{axis}"),
            Some(self.damping_ratio),
            parse_float,
            Some(0.),
            None,
            None,
            None,
        )?;
        if damping_ratio > sconfig.max_damping_ratio {
            return Err(CommandError::new(format!(
                "Too high value of damping_ratio={damping_ratio:.3} for shaper {shaper_type} \
on axis {axis}"
            )));
        }
        let shaper_freq = gcmd.get(
            &format!("SHAPER_FREQ_{axis}"),
            Some(self.shaper_freq),
            parse_float,
            Some(0.),
            None,
            None,
            None,
        )?;
        // Validate before committing, as upstream's `get_shaper` does.
        self.shaper_n(&shaper_name, shaper_freq, damping_ratio)
            .map_err(|err| CommandError::new(err.to_string()))?;
        self.damping_ratio = damping_ratio;
        self.shaper_type = shaper_name;
        self.shaper_freq = shaper_freq;
        Ok(())
    }

    /// Upstream's `get_shaper` (`input_shaper.py:49-58`): the number of impulses
    /// of the shaper these parameters describe — `0` for an axis with no
    /// `shaper_freq`, more for every shaper otherwise.
    ///
    /// The coefficients are built here, so that a shaper the parameters cannot
    /// build is reported (upstream passes them on to the kinematics layer,
    /// which this port has none of — see the module docs); the message is
    /// upstream's, the "Failed to initialize shaper: …" prefix included.
    fn shaper_n(
        &self,
        shaper_type: &str,
        shaper_freq: f64,
        damping_ratio: f64,
    ) -> Result<usize, shaper_defs::ShaperError> {
        if shaper_freq == 0. {
            let (a, _t) = shaper_defs::get_none_shaper();
            return Ok(a.len());
        }
        let Some((a, _t)) = shaper_defs::init_shaper(shaper_type, shaper_freq, damping_ratio)
            .map_err(shaper_defs::init_failed)?
        else {
            // Unreachable: both callers looked the name up in the same table
            // (`get_shaper_cfg`) a moment earlier.
            return Err(shaper_defs::ShaperError::new(format!(
                "Unsupported shaper type: {shaper_type}"
            )));
        };
        Ok(a.len())
    }

    /// Upstream's `get_status` (`input_shaper.py:59-63`): the report's fields,
    /// in upstream's order, with upstream's `%.3f` and `%.6f` formats.
    fn get_status(&self) -> [(&'static str, String); 3] {
        [
            ("shaper_type", self.shaper_type.clone()),
            ("shaper_freq", format!("{:.3}", self.shaper_freq)),
            ("damping_ratio", format!("{:.6}", self.damping_ratio)),
        ]
    }
}

/// One axis's shaper (`input_shaper.AxisInputShaper`, `:65-103`).
struct AxisInputShaper {
    /// The lower-case axis letter.
    axis: char,
    params: InputShaperParams,
    /// Upstream's `self.n`: how many impulses the shaper has, `0` when the axis
    /// is not shaped. Upstream keeps the coefficients beside it for the
    /// kinematics layer; this port has none (module docs), so the count stands
    /// for the shaper.
    n: usize,
}

impl AxisInputShaper {
    /// Read one axis's shaper (`input_shaper.py:66-70`).
    fn new(axis: char, config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let params = InputShaperParams::new(axis, config)?;
        let n = params
            .shaper_n(
                &params.shaper_type,
                params.shaper_freq,
                params.damping_ratio,
            )
            .map_err(|err| ConfigError::new(err.to_string()))?;
        Ok(Self { axis, params, n })
    }

    /// Upstream's `is_enabled` (`input_shaper.py:87-88`).
    fn is_enabled(&self) -> bool {
        self.n > 0
    }

    /// Upstream's `AxisInputShaper.update` (`input_shaper.py:75-77`): the
    /// parameters, then the shaper they now describe.
    fn update(&mut self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        self.params.update(gcmd)?;
        self.n = self
            .params
            .shaper_n(
                &self.params.shaper_type,
                self.params.shaper_freq,
                self.params.damping_ratio,
            )
            .map_err(|err| CommandError::new(err.to_string()))?;
        Ok(())
    }

    /// Upstream's `report` (`input_shaper.py:100-103`).
    fn report(&self, gcmd: &GcodeCommand) {
        let info = self
            .params
            .get_status()
            .iter()
            .map(|(key, value)| format!("{key}_{}:{value}", self.axis))
            .collect::<Vec<String>>()
            .join(" ");
        gcmd.respond_info(&info);
    }
}

/// `[input_shaper]`:
///
/// ```text
/// [input_shaper]
/// shaper_type_x: mzv(5,0.6)
/// shaper_freq_x: 33.2
/// ```
pub struct InputShaper {
    printer: Weak<Printer>,
    /// The three axes, in upstream's order (`input_shaper.py:112-114`). One
    /// lock, so an update of all three and the report that follows it see one
    /// set of parameters.
    shapers: Mutex<Vec<AxisInputShaper>>,
}

impl InputShaper {
    /// Read the section (`input_shaper.py:106-121`).
    ///
    /// # Errors
    /// As [`InputShaperParams::new`], for the first axis that cannot be built.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        Ok(Self {
            printer: Arc::downgrade(printer),
            shapers: Mutex::new(vec![
                AxisInputShaper::new('x', config)?,
                AxisInputShaper::new('y', config)?,
                AxisInputShaper::new('z', config)?,
            ]),
        })
    }

    fn lock(&self) -> MutexGuard<'_, Vec<AxisInputShaper>> {
        self.shapers
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// Upstream's `connect` (`input_shaper.py:124-137`).
    ///
    /// With dual carriage(s), the shaping belongs to the `dual_carriage` module:
    /// a section that configures an axis is a config error. Upstream's check
    /// reads the parameters only, and only here — a `SET_INPUT_SHAPER` that
    /// enables shaping at run time is allowed (`:200-207`).
    fn connect(&self) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        if printer.lookup_object("dual_carriage").is_some() {
            if self.lock().iter().any(AxisInputShaper::is_enabled) {
                printer.set_error_state(DUAL_CARRIAGE_CONFIG_ERROR);
            }
            return;
        }
        self.update_input_shaping();
    }

    /// Upstream's `_update_input_shaping` (`input_shaper.py:169-190`): put the
    /// shapers' coefficients into the toolhead's steppers.
    ///
    /// This port has no shaper kinematics layer to put them into (module docs),
    /// and everything the upstream path validates is validated before this point
    /// ([`InputShaperParams::new`], [`InputShaperParams::update`]); the last
    /// step left, the motion queue's accounting, is [`recompute_scan_windows`].
    fn update_input_shaping(&self) {
        recompute_scan_windows();
    }

    /// Upstream's `cmd_SET_INPUT_SHAPER` (`input_shaper.py:200-207`): apply the
    /// line's parameters to the three axes, then report them — the x and y axes
    /// always, the z axis when it is shaped.
    fn cmd_set_input_shaper(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let mut shapers = self.lock();
        if !gcmd.get_command_parameters().is_empty() {
            for shaper in shapers.iter_mut() {
                shaper.update(gcmd)?;
            }
            self.update_input_shaping();
        }
        for (index, shaper) in shapers.iter().enumerate() {
            if index < 2 || shaper.is_enabled() {
                shaper.report(gcmd);
            }
        }
        Ok(())
    }

    /// The events and commands upstream's `__init__` sets up
    /// (`input_shaper.py:110-121`).
    fn register_handlers(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        printer.register_event_handler(
            KlippyEvent::KlippyConnect,
            Box::new({
                let object = Arc::clone(self);
                move |_| object.connect()
            }),
        );
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        let handler: CommandHandler = {
            let object = Arc::clone(self);
            sync(move |gcmd| object.cmd_set_input_shaper(gcmd))
        };
        gcode
            .register_command_with_params(
                "SET_INPUT_SHAPER",
                handler,
                Some("Set cartesian parameters for input shaper"),
                SET_INPUT_SHAPER_PARAMS,
                false,
            )
            .map_err(ConfigError::new)
    }
}

impl PrinterObject for InputShaper {
    fn get_status(&self, _eventtime: f64) -> Value {
        // Upstream's `InputShaper` has no `get_status` — the shaper parameters
        // are reported by `SET_INPUT_SHAPER` — and `is_queryable` keeps the
        // object out of `objects/list` either way.
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

/// The words `SET_INPUT_SHAPER` reads, in read order: `InputShaperParams::
/// update` reads the plain `SHAPER_TYPE` first and then that axis's own
/// spellings, and the handler runs it for x, y and z in turn
/// (`input_shaper.py:27-48,200-207`).
const SET_INPUT_SHAPER_PARAMS: &[&str] = &[
    "SHAPER_TYPE",
    "SHAPER_TYPE_X",
    "DAMPING_RATIO_X",
    "SHAPER_FREQ_X",
    "SHAPER_TYPE_Y",
    "DAMPING_RATIO_Y",
    "SHAPER_FREQ_Y",
    "SHAPER_TYPE_Z",
    "DAMPING_RATIO_Z",
    "SHAPER_FREQ_Z",
];

/// Upstream's `motion_queuing.check_step_generation_scan_windows()`, which
/// `_update_input_shaping` ends with (`input_shaper.py:185-186`): the step
/// generation windows a shaper widens have to be recomputed when it changes.
///
/// This host has no such mechanism — `motion_queuing` is not ported and nothing
/// under `src/core/klippy/motion/` mentions a scan window — so there is nothing
/// to recompute. The call is kept, named and made rather than dropped, so the
/// seam stays visible and cannot be lost silently when the mechanism lands.
fn recompute_scan_windows() {}

/// The factory `section!` names (`input_shaper.py:209 def load_config`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = Arc::new(InputShaper::new(config, printer)?);
    object.register_handlers(printer)?;
    Ok(object)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{
        access::AccessTracking, check_unused, Config, ConfigSection, ConfigValue,
    };
    use crate::core::klippy::printer::PrinterState;
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[input_shaper]` section with `options`.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("input_shaper", None);
        for (key, value) in options {
            section.parameters.insert(
                (*key).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// The options the corpus's `test/klippy/input_shaper.cfg` carries.
    const CORPUS_OPTIONS: &[(&str, &str)] = &[
        ("shaper_type_x", "mzv"),
        ("shaper_freq_x", "33.2"),
        ("shaper_type_y", "ei(v_tol=0.02)"),
        ("shaper_freq_y", "39.3"),
        ("damping_ratio_y", "0.4"),
        ("shaper_freq_z", "42"),
    ];

    /// A printer with a g-code dispatcher and the `[input_shaper]` object
    /// loaded into it.
    fn printer(options: &[(&str, &str)]) -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let section = section(options);
        let object =
            load_config(&ConfigWrapper::untracked(&section), &printer).expect("the section loads");
        printer.add_object("input_shaper", object).unwrap();
        // `SET_INPUT_SHAPER` is not a base command: the dispatcher answers it
        // only once the printer is ready.
        printer.send_event(&KlippyEvent::KlippyReady);
        printer
    }

    /// Run `script` and return the error, if any, together with the lines the
    /// dispatcher emitted (`// ` prefixed, as `respond_info` writes them).
    fn run(printer: &Arc<Printer>, script: &str) -> (Result<(), CommandError>, Vec<String>) {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the dispatcher");
        let lines = Arc::new(Mutex::new(Vec::new()));
        {
            let lines = Arc::clone(&lines);
            gcode.register_output_handler(Arc::new(move |line: &str| {
                lines
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner())
                    .push(line.to_string())
            }));
        }
        let result = gcode.run_script_sync(script);
        let lines = lines
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone();
        (result, lines)
    }

    /// The corpus's section loads and every one of its options is read.
    #[test]
    fn the_corpus_section_reads_every_option_it_has() {
        let mut text = String::from("[input_shaper]\n");
        for (key, value) in CORPUS_OPTIONS {
            text.push_str(&format!("{key}: {value}\n"));
        }
        let (config, _) = Config::from_text(&text).expect("the section parses");
        let sect = config.get_section("input_shaper").expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(sect, Arc::clone(&access));

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        load_config(&wrapper, &printer).expect("the section loads");

        check_unused(&config, &access, &["input_shaper".to_string()])
            .expect("no option is left unread");
    }

    /// A `[input_shaper]` with no options at all — the corpus's
    /// `hybrid_corexy_dual_carriage.cfg` — loads, and reports only the two axes
    /// upstream always reports.
    #[test]
    fn the_bare_section_loads_and_reports_the_unshaped_axes() {
        let printer = printer(&[]);

        let (result, lines) = run(&printer, "SET_INPUT_SHAPER");

        result.expect("the bare command reports");
        assert_eq!(
            lines,
            [
                "// shaper_type_x:mzv shaper_freq_x:0.000 damping_ratio_x:0.100000",
                "// shaper_type_y:mzv shaper_freq_y:0.000 damping_ratio_y:0.100000",
            ]
        );
    }

    /// Both `SET_INPUT_SHAPER` lines of the corpus's `input_shaper.test`, whose
    /// reports cover the three axes (the x and y axes always, the z axis because
    /// the config gives it a frequency).
    #[test]
    fn set_input_shaper_reports_each_axis_with_upstreams_numbers() {
        let printer = printer(CORPUS_OPTIONS);

        let (result, lines) = run(
            &printer,
            "SET_INPUT_SHAPER SHAPER_FREQ_X=22.2 DAMPING_RATIO_X=.1 SHAPER_TYPE_X='mzv(5,0.6)'\n\
             SET_INPUT_SHAPER SHAPER_FREQ_Y=33.3 DAMPING_RATIO_Y=.11 SHAPER_TYPE_Y=2hump_ei",
        );

        result.expect("the two lines apply");
        assert_eq!(
            lines,
            [
                // The first line sets the x axis; y and z are still the config's.
                "// shaper_type_x:mzv(5,0.6) shaper_freq_x:22.200 damping_ratio_x:0.100000",
                "// shaper_type_y:ei(v_tol=0.02) shaper_freq_y:39.300 damping_ratio_y:0.400000",
                "// shaper_type_z:mzv shaper_freq_z:42.000 damping_ratio_z:0.100000",
                // The second line sets the y axis.
                "// shaper_type_x:mzv(5,0.6) shaper_freq_x:22.200 damping_ratio_x:0.100000",
                "// shaper_type_y:2hump_ei shaper_freq_y:33.300 damping_ratio_y:0.110000",
                "// shaper_type_z:mzv shaper_freq_z:42.000 damping_ratio_z:0.100000",
            ]
        );
    }

    /// The three ways a `SET_INPUT_SHAPER` line is rejected, with upstream's
    /// messages.
    #[test]
    fn set_input_shaper_rejects_what_it_cannot_apply() {
        // A shaper type no shaper carries.
        let (result, lines) = {
            let printer = printer(&[]);
            run(&printer, "SET_INPUT_SHAPER SHAPER_TYPE_X=no_such")
        };
        assert_eq!(
            result.unwrap_err().message(),
            "Unsupported shaper type: no_such"
        );
        assert!(
            lines.iter().any(|line| line.ends_with("no_such")),
            "{lines:?}"
        );

        // A damping ratio above the shaper's maximum: `3hump_ei` takes 0.2.
        let (result, _) = {
            let printer = printer(&[]);
            run(
                &printer,
                "SET_INPUT_SHAPER SHAPER_TYPE_Y=3hump_ei DAMPING_RATIO_Y=0.5",
            )
        };
        assert_eq!(
            result.unwrap_err().message(),
            "Too high value of damping_ratio=0.500 for shaper 3hump_ei on axis Y"
        );

        // A shaper the parameters cannot build, quoted as the corpus writes it.
        let (result, _) = {
            let printer = printer(&[]);
            run(
                &printer,
                "SET_INPUT_SHAPER SHAPER_FREQ_X=22.2 SHAPER_TYPE_X='mzv(2,0.5)'",
            )
        };
        assert_eq!(
            result.unwrap_err().message(),
            "Failed to initialize shaper: Too small n=2, must be at least 3"
        );
    }

    /// A config error rejects a shaper type no shaper carries, as upstream's
    /// `__init__` does.
    #[test]
    fn the_section_rejects_an_unknown_shaper_type() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let section = section(&[("shaper_type_x", "bogus")]);

        let err = load_config(&ConfigWrapper::untracked(&section), &printer)
            .err()
            .expect("the section is rejected");

        assert_eq!(err.message(), "Unsupported shaper type: bogus");
    }

    /// The object the check looks up: any registered `dual_carriage` will do.
    struct Carriage;

    impl PrinterObject for Carriage {
        fn get_status(&self, _eventtime: f64) -> Value {
            json!({})
        }
    }

    /// With dual carriage(s), a `[input_shaper]` that configures an axis is a
    /// config error (`input_shaper.py:124-137`).
    #[test]
    fn a_configured_axis_with_dual_carriage_is_a_config_error() {
        let printer = printer(&[("shaper_freq_x", "30")]);
        printer
            .add_object("dual_carriage", Arc::new(Carriage))
            .unwrap();

        printer.send_event(&KlippyEvent::KlippyConnect);

        let state = printer.get_state_message();
        assert_eq!(state.category, PrinterState::Error);
        assert_eq!(state.message, DUAL_CARRIAGE_CONFIG_ERROR);
    }

    /// …but the same printer accepts a `SET_INPUT_SHAPER` that turns shaping
    /// on, as the corpus's `hybrid_corexy_dual_carriage.test` does: the check is
    /// a connect-time one only.
    #[test]
    fn a_running_set_input_shaper_is_allowed_with_dual_carriage() {
        let printer = printer(&[]);
        printer
            .add_object("dual_carriage", Arc::new(Carriage))
            .unwrap();

        let (result, lines) = run(
            &printer,
            "SET_INPUT_SHAPER SHAPER_TYPE_X=MZV SHAPER_FREQ_X=70\n\
             SET_INPUT_SHAPER SHAPER_TYPE_Y=2HUMP_EI SHAPER_FREQ_Y=50",
        );

        result.expect("the two lines the hybrid case runs apply");
        assert_eq!(
            lines.last().map(String::as_str),
            Some("// shaper_type_y:2hump_ei shaper_freq_y:50.000 damping_ratio_y:0.100000")
        );
    }
}
