//! `[delta_calibrate]` — `DELTA_CALIBRATE` and `DELTA_ANALYZE`
//! (upstream `klippy/extras/delta_calibrate.py`).
//!
//! The section reads `radius` (the default probe points orbit it), restores
//! the saved stable positions (`height%d`, `manual_height%d`, `distance%d`
//! with their `…_pos` triples), and registers the two commands. Both drive the
//! same fit: turn measured heights/distances into stable positions, run
//! upstream's coordinate descent over `radius`, the two free tower angles, the
//! three endstops (and the arms, when distances are in play), then store the
//! result for `SAVE_CONFIG`.
//!
//! | command | what it does |
//! |---|---|
//! | `DELTA_CALIBRATE [METHOD=manual]` | probe the default points ([`ProbePointsHelper`]; a printer without `[probe]` — or `METHOD=manual` — walks them with the manual helper) and fit |
//! | `DELTA_ANALYZE …` | add `MANUAL_HEIGHT` measurements, or record the `*_DISTS`/`*_PILLAR_WIDTHS`/`SCALE` of the calibration object and fit with `CALIBRATE=extended` |
//!
//! The math is upstream's: stable positions and trilateration come from
//! [`DeltaCalibration`](crate::core::klippy::motion::delta::DeltaCalibration),
//! the measurement geometry from `measurements_to_distances`
//! (`delta_calibrate.py:32-66`), the fit from `mathutil.coordinate_descent`
//! (already in [`coordinate_descent`](crate::core::klippy::mathutil)).
//!
//! What is deliberately not here: running the fit in a background process
//! (upstream `background_coordinate_descent`); this port runs it inline, like
//! its `z_tilt`/`bed_tilt` fits.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock, Weak};

use serde_json::{json, Value};
use tracing::{info, warn};

use crate::core::klippy::config::object::{PrinterConfig, CONFIGFILE_OBJECT};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::error::KlippyError;
use crate::core::klippy::extras::probe::{ProbeOffsets, ProbePointsFinalize, ProbePointsHelper};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{coordinate_descent, Coord};
use crate::core::klippy::motion::delta::DeltaCalibration;
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};

section!("delta_calibrate", order = 30, load = load_config);

/// The object the toolhead is looked up under.
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The two commands this section registers (`delta_calibrate.py:121-123`).
const COMMAND_CALIBRATE: &str = "DELTA_CALIBRATE";
const COMMAND_ANALYZE: &str = "DELTA_ANALYZE";

/// How much to prefer a distance measurement over a height measurement
/// (`MEASURE_WEIGHT`).
const MEASURE_WEIGHT: f64 = 0.5;

/// The error a fit cannot use: upstream returns this from its error function
/// when `trilateration` raises (`delta_calibrate.py:176-177`).
const FIT_IMPOSSIBLE: f64 = 9999999999999.9;

/// The calibration object's geometry (`calibrate_size.stl`,
/// `delta_calibrate.py:23-27`).
const MEASURE_ANGLES: [f64; 6] = [210., 270., 330., 30., 90., 150.];
const MEASURE_OUTER_RADIUS: f64 = 65.;
const MEASURE_RIDGE_RADIUS: f64 = 5.0 - 0.5;

/// A stable position: steps taken since each tower hit its endstop
/// (`delta_calibrate.py:7-11`).
type Stable = [f64; 3];
/// A measured height paired with the stable position it was taken at.
type HeightPosition = (f64, Stable);
/// A measured distance and the two stable positions it spans.
type Distance = (f64, Stable, Stable);

/// `[delta_calibrate]`: the probe helper, the saved measurements, and the
/// running `DELTA_ANALYZE` entry (`DeltaCalibrate`, `delta_calibrate.py:69-160`).
pub struct DeltaCalibrate {
    /// The machine, for the commands, the toolhead and the configfile object.
    printer: Weak<Printer>,
    /// The points helper behind `DELTA_CALIBRATE`; wired after construction
    /// (the callback needs the `Weak` first — the `ZTilt` pattern).
    probe_helper: OnceLock<Arc<ProbePointsHelper>>,
    /// The `height%d` stable positions loaded from the config (or the last
    /// basic round's, once `DELTA_ANALYZE` has anything to extend).
    last_probe_positions: Mutex<Vec<HeightPosition>>,
    /// The `manual_height%d` entries `DELTA_ANALYZE MANUAL_HEIGHT=` recorded.
    manual_heights: Mutex<Vec<HeightPosition>>,
    /// The `distance%d` entries loaded from the config.
    last_distances: Mutex<Vec<Distance>>,
    /// The measurements this `DELTA_ANALYZE` command line is building;
    /// starts with upstream's implicit `SCALE = 1`.
    delta_analyze_entry: Mutex<HashMap<&'static str, Vec<f64>>>,
    /// The first finalize error, for the command to report — an automatic
    /// round's callback cannot raise into the probing loop itself.
    last_error: Mutex<Option<CommandError>>,
}

impl DeltaCalibrate {
    /// Read the section, wire the probe helper, restore the saved
    /// measurements, and register both commands
    /// (`DeltaCalibrate.__init__`, `delta_calibrate.py:73-123`).
    ///
    /// # Errors
    /// A malformed option, fewer than three probe points, or a command name
    /// that is taken.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        // The default probe points: the centre plus six points on a scattered
        // ring of `radius` (`delta_calibrate.py:76-82`).
        let radius = config.get_float_bounded("radius", None, None, None, Some(0.0), None)?;
        const SCATTER: [f64; 6] = [0.95, 0.90, 0.85, 0.70, 0.75, 0.80];
        let mut points = vec![(0., 0.)];
        for (index, scatter) in SCATTER.iter().enumerate() {
            let degrees = 90. + 60. * index as f64;
            let dist = radius * scatter;
            points.push((
                degrees.to_radians().cos() * dist,
                degrees.to_radians().sin() * dist,
            ));
        }

        let this = Arc::new(Self {
            printer: Arc::downgrade(printer),
            probe_helper: OnceLock::new(),
            last_probe_positions: Mutex::new(Vec::new()),
            manual_heights: Mutex::new(Vec::new()),
            last_distances: Mutex::new(Vec::new()),
            delta_analyze_entry: Mutex::new(HashMap::from([("SCALE", vec![1.])])),
            last_error: Mutex::new(None),
        });

        let finalize: ProbePointsFinalize = Arc::new({
            let weak = Arc::downgrade(&this);
            move |offsets, positions| {
                let Some(this) = weak.upgrade() else {
                    warn!("DELTA_CALIBRATE finalize: the delta_calibrate object is gone");
                    return None;
                };
                match this.probe_finalize(offsets, positions) {
                    Ok(()) => None,
                    Err(error) => {
                        warn!("DELTA_CALIBRATE: {error}");
                        let mut slot = this.last_error.lock().unwrap_or_else(|p| p.into_inner());
                        if slot.is_none() {
                            *slot = Some(error);
                        }
                        None
                    }
                }
            }
        });
        let probe_helper =
            ProbePointsHelper::with_default_points(config, printer, finalize, Some(points))?;
        probe_helper.minimum_points(3)?;
        this.probe_helper
            .set(probe_helper)
            .unwrap_or_else(|_| unreachable!("the probe helper is wired once"));

        // Restore the saved stable positions and measurements
        // (`delta_calibrate.py:85-120`).
        *this
            .last_probe_positions
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = read_heights(config, "height")?;
        *this
            .manual_heights
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = read_heights(config, "manual_height")?;
        *this
            .last_distances
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = read_distances(config)?;

        // Register the commands (`delta_calibrate.py:121-123`).
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");
        {
            let weak = Arc::downgrade(&this);
            let handler: CommandHandler = Arc::new(move |gcmd: &GcodeCommand| {
                let weak = weak.clone();
                Box::pin(async move {
                    let this = weak
                        .upgrade()
                        .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                    this.cmd_delta_calibrate(gcmd).await
                })
            });
            gcode
                .register_command(
                    COMMAND_CALIBRATE,
                    handler,
                    Some("Delta calibration script"),
                    false,
                )
                .map_err(ConfigError::new)?;
        }
        {
            let weak = Arc::downgrade(&this);
            let handler: CommandHandler = sync(move |gcmd| {
                let this = weak
                    .upgrade()
                    .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                this.cmd_delta_analyze(gcmd)
            });
            gcode
                .register_command(
                    COMMAND_ANALYZE,
                    handler,
                    Some("Extended delta calibration tool"),
                    false,
                )
                .map_err(ConfigError::new)?;
        }
        Ok(this)
    }

    /// `DELTA_CALIBRATE`: probe the default points and fit
    /// (`cmd_DELTA_CALIBRATE`).
    async fn cmd_delta_calibrate(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let helper = self
            .probe_helper
            .get()
            .expect("the probe helper is wired in `new`");
        helper.start_probe(gcmd).await?;
        // A round whose finalize failed reports the first error here (the
        // callback itself cannot raise into the probing loop).
        self.take_last_error()
    }

    /// The stored finalize error, if one happened (`screws_tilt_adjust`'s
    /// `last_error` pattern).
    fn take_last_error(&self) -> Result<(), CommandError> {
        let mut slot = self.last_error.lock().unwrap_or_else(|p| p.into_inner());
        match slot.take() {
            Some(error) => Err(error),
            None => Ok(()),
        }
    }

    /// `probe_finalize`: turn the round's positions into `(height, stable)`
    /// pairs and fit (`probe_finalize`, `delta_calibrate.py:151-163`).
    ///
    /// The port's probe results are already the **test** position (nozzle
    /// coordinates; the `z_offset`-into-`bed_z` fold is the H9 gap recorded in
    /// `extras/probe.rs`), so upstream's `test_z - bed_z` height is the
    /// probe's `z_offset` — zero for a manual round, whose offsets are zeros.
    fn probe_finalize(
        &self,
        offsets: ProbeOffsets,
        positions: &[Coord],
    ) -> Result<(), CommandError> {
        let toolhead = self.toolhead()?;
        let calibration = self.calibration(&toolhead)?;
        let probe_positions: Vec<HeightPosition> = positions
            .iter()
            .map(|position| {
                let test = [position.x(), position.y(), position.z()];
                (offsets.z, calibration.calc_stable_position(test))
            })
            .collect();
        let distances = self
            .last_distances
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        self.calculate_params(&probe_positions, &distances)
    }

    /// The toolhead, when the machine is up.
    fn toolhead(&self) -> Result<Arc<ToolHeadObject>, CommandError> {
        self.printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT))
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    /// The kinematics' calibration parameters (`kin.get_calibration()`),
    /// refused when the machine is not a delta (`delta_calibrate.py:127-131`).
    fn calibration(
        &self,
        toolhead: &Arc<ToolHeadObject>,
    ) -> Result<DeltaCalibration, CommandError> {
        toolhead
            .delta_calibration()
            .ok_or_else(|| CommandError::new("Delta calibrate is only for delta printers"))
    }

    /// The fit itself (`calculate_params`, `delta_calibrate.py:165-220`):
    /// coordinate descent over upstream's adjustable set, then store the
    /// result and report the `SAVE_CONFIG` notice.
    fn calculate_params(
        &self,
        probe_positions: &[HeightPosition],
        distances: &[Distance],
    ) -> Result<(), CommandError> {
        let toolhead = self.toolhead()?;
        let original = self.calibration(&toolhead)?;
        let extended = !distances.is_empty();

        let mut height_positions = self
            .manual_heights
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        height_positions.extend_from_slice(probe_positions);

        // Upstream's weights: heights alone weigh 1; with distances in play
        // the distances' sum is scaled to their count against half the probe
        // points (`MEASURE_WEIGHT * len(probe_positions)`).
        let z_weight = if distances.is_empty() {
            1.
        } else {
            distances.len() as f64 / (MEASURE_WEIGHT * probe_positions.len() as f64)
        };

        let mut params = original.descent_params(extended);
        info!(
            "Calculating delta_calibrate with:\n{height_positions:?}\n{distances:?}\n\
             Initial delta_calibrate parameters: {params:?}"
        );
        coordinate_descent(&mut params, |values| {
            let trial = DeltaCalibration::from_descent_params(&original, values, extended);
            let mut total_error = 0.0;
            for (z_offset, stable) in &height_positions {
                let Some(position) = trial.get_position_from_stable(*stable) else {
                    return FIT_IMPOSSIBLE;
                };
                total_error += (position[2] - z_offset).powi(2);
            }
            total_error *= z_weight;
            for (distance, first, second) in distances {
                let (Some(a), Some(b)) = (
                    trial.get_position_from_stable(*first),
                    trial.get_position_from_stable(*second),
                ) else {
                    return FIT_IMPOSSIBLE;
                };
                let measured =
                    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
                total_error += (measured - distance).powi(2);
            }
            // Upstream's `ValueError` branch: an unusable geometry scores as
            // an enormous error rather than aborting the search.
            if total_error.is_finite() {
                total_error
            } else {
                FIT_IMPOSSIBLE
            }
        });
        let new_calibration = DeltaCalibration::from_descent_params(&original, &params, extended);
        info!("Calculated delta_calibrate parameters: {params:?}");

        self.save_state(probe_positions, distances, &new_calibration)?;
        self.respond(
            "The SAVE_CONFIG command will update the printer config file\n\
             with these parameters and restart the printer.",
        );
        Ok(())
    }

    /// Store the fit for `SAVE_CONFIG` (`save_state`, `delta_calibrate.py:135-160`,
    /// plus `DeltaCalibration.save_state`, `delta.py:223-240`).
    fn save_state(
        &self,
        probe_positions: &[HeightPosition],
        distances: &[Distance],
        calibration: &DeltaCalibration,
    ) -> Result<(), CommandError> {
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let configfile = printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;

        // The delta parameters (`DeltaCalibration.save_state`).
        configfile.set(
            "printer",
            "delta_radius",
            &format!("{:.6}", calibration.radius),
        );
        for (index, axis) in ['a', 'b', 'c'].into_iter().enumerate() {
            let section = format!("stepper_{axis}");
            configfile.set(
                &section,
                "angle",
                &format!("{:.6}", calibration.angles[index]),
            );
            configfile.set(
                &section,
                "arm_length",
                &format!("{:.6}", calibration.arms[index]),
            );
            configfile.set(
                &section,
                "position_endstop",
                &format!("{:.6}", calibration.endstops[index]),
            );
        }
        self.respond(&format!(
            "stepper_a: position_endstop: {:.6} angle: {:.6} arm_length: {:.6}\n\
             stepper_b: position_endstop: {:.6} angle: {:.6} arm_length: {:.6}\n\
             stepper_c: position_endstop: {:.6} angle: {:.6} arm_length: {:.6}\n\
             delta_radius: {:.6}",
            calibration.endstops[0],
            calibration.angles[0],
            calibration.arms[0],
            calibration.endstops[1],
            calibration.angles[1],
            calibration.arms[1],
            calibration.endstops[2],
            calibration.angles[2],
            calibration.arms[2],
            calibration.radius,
        ));

        // The measurements the next run restores (`save_state`).
        let section = "delta_calibrate";
        configfile.remove_section(section);
        for (index, (height, stable)) in probe_positions.iter().enumerate() {
            configfile.set(
                section,
                &format!("height{index}"),
                &format_stable_height(*height),
            );
            configfile.set(
                section,
                &format!("height{index}_pos"),
                &format_stable(stable),
            );
        }
        for (index, (height, stable)) in self
            .manual_heights
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
            .enumerate()
        {
            configfile.set(
                section,
                &format!("manual_height{index}"),
                &format_stable_height(*height),
            );
            configfile.set(
                section,
                &format!("manual_height{index}_pos"),
                &format_stable(stable),
            );
        }
        for (index, (distance, first, second)) in distances.iter().enumerate() {
            configfile.set(section, &format!("distance{index}"), &format!("{distance}"));
            configfile.set(
                section,
                &format!("distance{index}_pos1"),
                &format_stable(first),
            );
            configfile.set(
                section,
                &format!("distance{index}_pos2"),
                &format_stable(second),
            );
        }
        Ok(())
    }

    /// `DELTA_ANALYZE`: record measurements, or fit them
    /// (`cmd_DELTA_ANALYZE`, `delta_calibrate.py:257-284`).
    fn cmd_delta_analyze(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        // `MANUAL_HEIGHT` records where the nozzle is and returns.
        if gcmd.get_command_parameters().contains_key("MANUAL_HEIGHT") {
            let height = gcmd.get_float("MANUAL_HEIGHT")?;
            return self.add_manual_height(height);
        }
        // Parse the measurement parameters, each with its fixed count.
        const ARGS: [(&str, usize); 5] = [
            ("CENTER_DISTS", 6),
            ("CENTER_PILLAR_WIDTHS", 3),
            ("OUTER_DISTS", 6),
            ("OUTER_PILLAR_WIDTHS", 6),
            ("SCALE", 1),
        ];
        let mut entry = self
            .delta_analyze_entry
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        for (name, count) in ARGS {
            if !gcmd.get_command_parameters().contains_key(name) {
                continue;
            }
            let raw = gcmd.get_str_default(name, "");
            let parsed: Result<Vec<f64>, _> = raw
                .split(',')
                .map(|part| part.trim().parse::<f64>())
                .collect();
            let values = parsed
                .map_err(|_| CommandError::new(format!("Unable to parse parameter '{name}'")))?;
            if values.len() != count {
                return Err(CommandError::new(format!(
                    "Parameter '{name}' must have {count} values"
                )));
            }
            info!("DELTA_ANALYZE {name} = {values:?}");
            entry.insert(name, values);
        }
        drop(entry);

        if let Some(action) = gcmd_parameter(gcmd, "CALIBRATE") {
            if action != "extended" {
                return Err(CommandError::new("Unknown calibrate action"));
            }
            return self.do_extended_calibration();
        }
        Ok(())
    }

    /// `DELTA_ANALYZE`'s fit over the recorded distances
    /// (`do_extended_calibration`, `delta_calibrate.py:241-255`).
    fn do_extended_calibration(&self) -> Result<(), CommandError> {
        let entry = self
            .delta_analyze_entry
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        let distances: Vec<Distance> = if entry.len() <= 1 {
            self.last_distances
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .clone()
        } else if entry.len() < 5 {
            return Err(CommandError::new("Not all measurements provided"));
        } else {
            let toolhead = self.toolhead()?;
            let calibration = self.calibration(&toolhead)?;
            measurements_to_distances(&entry, &calibration)?
        };
        let last_probe_positions = self
            .last_probe_positions
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        if last_probe_positions.is_empty() {
            return Err(CommandError::new(
                "Must run basic calibration with DELTA_CALIBRATE first",
            ));
        }
        self.calculate_params(&last_probe_positions, &distances)
    }

    /// `DELTA_ANALYZE MANUAL_HEIGHT=`: record the height the nozzle is at,
    /// as a stable position (`add_manual_height`, `delta_calibrate.py:222-239`).
    ///
    /// The port reads the toolhead's commanded position instead of
    /// re-deriving it from the steppers after a generation flush: the two are
    /// the same number once the planner has drained, which is the state this
    /// host's manual paths hand back.
    fn add_manual_height(&self, height: f64) -> Result<(), CommandError> {
        let toolhead = self.toolhead()?;
        let position = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let calibration = self.calibration(&toolhead)?;
        let stable = calibration.calc_stable_position([position.x(), position.y(), position.z()]);
        self.manual_heights
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push((height, stable));
        self.respond(&format!(
            "Adding manual height: {:.3},{:.3},{:.3} is actually z={:.3}",
            position.x(),
            position.y(),
            position.z(),
            height
        ));
        Ok(())
    }

    /// `gcode.respond_info`, for the tool's report lines.
    fn respond(&self, message: &str) {
        if let Some(printer) = self.printer.upgrade() {
            if let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) {
                gcode.respond_info(message, true);
            }
        }
    }
}

impl PrinterObject for DeltaCalibrate {
    /// Upstream's object has no `get_status`, so it is not client-visible.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }

    /// `klippy:connect`'s handler: refuse to sit on a non-delta printer
    /// (`handle_connect`, `delta_calibrate.py:125-131`).
    ///
    /// The check runs against the toolhead's **load-time** kinematics kind,
    /// because upstream loads `toolhead` last and this object's connect comes
    /// first — the kinematics object itself does not exist yet.
    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            let toolhead = self
                .printer
                .upgrade()
                .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT));
            match toolhead {
                Some(toolhead) if !toolhead.has_delta_calibration() => Err(KlippyError::Config(
                    ConfigError::new("Delta calibrate is only for delta printers"),
                )),
                _ => Ok(()),
            }
        })
    }
}

/// The factory `section!` names (`load_config`, `delta_calibrate.py:286-287`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = DeltaCalibrate::new(config, printer)?;
    Ok(object as Arc<dyn PrinterObject>)
}

// ===========================================================================
// Config readers
// ===========================================================================

/// Load a stable position from a config entry (`load_config_stable`,
/// `delta_calibrate.py:14-15`): three floats on one line.
///
/// # Errors
/// When the option is missing (the entry it belongs to is incomplete) or a
/// value is not a number.
fn load_config_stable(config: &ConfigWrapper, option: &str) -> Result<Stable, ConfigError> {
    let groups = config.get_list_of_lists(option, '\n', ',', 3)?;
    let identifier = config.identifier();
    if groups.len() != 1 {
        return Err(ConfigError::new(format!(
            "Option '{option}' in section '{identifier}' is required"
        )));
    }
    let mut stable = [0.0; 3];
    for (index, item) in groups[0].iter().enumerate() {
        stable[index] = item.trim().parse::<f64>().map_err(|_| {
            ConfigError::new(format!(
                "Unable to parse option '{option}' in section '{identifier}'"
            ))
        })?;
    }
    Ok(stable)
}

/// Restore every `height%d`/`height%d_pos` pair, stopping at the first
/// missing index (`delta_calibrate.py:97-102`).
///
/// # Errors
/// As [`load_config_stable`].
fn read_heights(config: &ConfigWrapper, prefix: &str) -> Result<Vec<HeightPosition>, ConfigError> {
    let mut heights = Vec::new();
    for index in 0..999 {
        let Some(height) = config.get_optional_float(&format!("{prefix}{index}"))? else {
            break;
        };
        let stable = load_config_stable(config, &format!("{prefix}{index}_pos"))?;
        heights.push((height, stable));
    }
    Ok(heights)
}

/// Restore every `distance%d`/`distance%d_pos1`/`…_pos2` triple, stopping at
/// the first missing index (`delta_calibrate.py:113-120`).
///
/// # Errors
/// As [`load_config_stable`].
fn read_distances(config: &ConfigWrapper) -> Result<Vec<Distance>, ConfigError> {
    let mut distances = Vec::new();
    for index in 0..999 {
        let Some(distance) = config.get_optional_float(&format!("distance{index}"))? else {
            break;
        };
        let first = load_config_stable(config, &format!("distance{index}_pos1"))?;
        let second = load_config_stable(config, &format!("distance{index}_pos2"))?;
        distances.push((distance, first, second));
    }
    Ok(distances)
}

/// A command parameter, or `None` when absent (`gcmd.get(name, None)`).
fn gcmd_parameter(gcmd: &GcodeCommand, name: &str) -> Option<String> {
    gcmd.get_command_parameters().get(name).cloned()
}

/// A stable position as `%.3f,%.3f,%.3f` (`save_state`'s format).
fn format_stable(stable: &Stable) -> String {
    format!("{:.3},{:.3},{:.3}", stable[0], stable[1], stable[2])
}

/// A height as the config stores it (upstream writes the raw float).
fn format_stable_height(height: f64) -> String {
    format!("{height}")
}

// ===========================================================================
// measurements_to_distances (delta_calibrate.py:32-66)
// ===========================================================================

/// Convert distance measurements made on the calibration object into
/// `(distance, stable_position1, stable_position2)` triples, using the
/// calibration object's current geometry.
///
/// # Errors
/// When a measurement the geometry needs is missing from the entry (upstream
/// would raise `KeyError`; the caller already checked the count, so this only
/// fires for a partial set).
fn measurements_to_distances(
    measured: &HashMap<&'static str, Vec<f64>>,
    delta_params: &DeltaCalibration,
) -> Result<Vec<Distance>, CommandError> {
    let required = [
        "CENTER_DISTS",
        "CENTER_PILLAR_WIDTHS",
        "OUTER_DISTS",
        "OUTER_PILLAR_WIDTHS",
    ];
    for name in required {
        if !measured.contains_key(name) {
            return Err(CommandError::new("Not all measurements provided"));
        }
    }
    let scale = measured.get("SCALE").map(|values| values[0]).unwrap_or(1.);

    let cpw = &measured["CENTER_PILLAR_WIDTHS"];
    let center_widths = [cpw[0], cpw[2], cpw[1], cpw[0], cpw[2], cpw[1]];
    let center_dists: Vec<f64> = measured["CENTER_DISTS"]
        .iter()
        .zip(center_widths)
        .map(|(distance, width)| distance - width)
        .collect();
    let outer_dists: Vec<f64> = measured["OUTER_DISTS"]
        .iter()
        .zip(&measured["OUTER_PILLAR_WIDTHS"])
        .map(|(distance, width)| distance - width)
        .collect();

    // The six measurement angles as XY multipliers.
    let xy_angles: Vec<(f64, f64)> = MEASURE_ANGLES
        .iter()
        .map(|angle| {
            let radians = angle.to_radians();
            (radians.cos(), radians.sin())
        })
        .collect();

    // Stable positions for the centre measurements: the inner ridge to the
    // outer ridge along each measurement angle.
    let inner_ridge = MEASURE_RIDGE_RADIUS * scale;
    let inner_pos: Vec<Stable> = xy_angles
        .iter()
        .map(|(ax, ay)| [ax * inner_ridge, ay * inner_ridge, 0.])
        .collect();
    let outer_ridge = (MEASURE_OUTER_RADIUS + MEASURE_RIDGE_RADIUS) * scale;
    let outer_pos: Vec<Stable> = xy_angles
        .iter()
        .map(|(ax, ay)| [ax * outer_ridge, ay * outer_ridge, 0.])
        .collect();
    let mut out: Vec<Distance> = center_dists
        .iter()
        .zip(inner_pos)
        .zip(outer_pos)
        .map(|((distance, inner), outer)| {
            (
                *distance,
                delta_params.calc_stable_position(inner),
                delta_params.calc_stable_position(outer),
            )
        })
        .collect();

    // Stable positions for the outer measurements: the ridge pair shifted by
    // the pillar's own angle around the outer-circle start.
    let outer_center = MEASURE_OUTER_RADIUS * scale;
    let start_pos: Vec<(f64, f64)> = xy_angles
        .iter()
        .map(|(ax, ay)| (ax * outer_center, ay * outer_center))
        .collect();
    let shifted: Vec<(f64, f64)> = xy_angles[2..]
        .iter()
        .chain(&xy_angles[..2])
        .copied()
        .collect();
    let first_pos: Vec<Stable> = shifted
        .iter()
        .zip(&start_pos)
        .map(|((ax, ay), (spx, spy))| [ax * inner_ridge + spx, ay * inner_ridge + spy, 0.])
        .collect();
    let second_pos: Vec<Stable> = shifted
        .iter()
        .zip(&start_pos)
        .map(|((ax, ay), (spx, spy))| [ax * outer_ridge + spx, ay * outer_ridge + spy, 0.])
        .collect();
    out.extend(outer_dists.iter().zip(first_pos).zip(second_pos).map(
        |((distance, first), second)| {
            (
                *distance,
                delta_params.calc_stable_position(first),
                delta_params.calc_stable_position(second),
            )
        },
    ));
    Ok(out)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::motion::delta::DeltaConfig;

    /// `config/example-delta.cfg`'s machine, as its calibration parameters.
    fn example_calibration() -> DeltaCalibration {
        use crate::core::klippy::motion::delta::DeltaKinematics;
        DeltaKinematics::new(DeltaConfig {
            radius: 174.75,
            print_radius: 174.75,
            minimum_z_position: 0.0,
            angles: [210., 330., 90.],
            arm_lengths: [333., 333., 333.],
            endstops: [297.05, 297.05, 297.05],
            step_dists: [0.0125, 0.0125, 0.0125],
            max_velocity: 300.,
            max_accel: 3000.,
            max_z_velocity: 150.,
            max_z_accel: 1500.,
        })
        .expect("the example geometry computes")
        .calibration()
    }

    /// The section's default points: the centre plus the scattered ring, in
    /// upstream's order (`delta_calibrate.py:76-82`).
    fn default_points(radius: f64) -> Vec<(f64, f64)> {
        const SCATTER: [f64; 6] = [0.95, 0.90, 0.85, 0.70, 0.75, 0.80];
        let mut points = vec![(0., 0.)];
        for (index, scatter) in SCATTER.iter().enumerate() {
            let radians = (90. + 60. * index as f64).to_radians();
            let dist = radius * scatter;
            points.push((radians.cos() * dist, radians.sin() * dist));
        }
        points
    }

    #[test]
    fn test_default_points_match_the_upstream_scatter() {
        let points = default_points(50.);
        assert_eq!(points.len(), 7);
        assert_eq!(points[0], (0., 0.));
        // 90°, radius 50 · .95 → (0, 47.5).
        assert!((points[1].0).abs() < 1e-12);
        assert!((points[1].1 - 47.5).abs() < 1e-12);
        // 150°, radius 50 · .90 → (45·cos150°, 45·sin150°).
        assert!((points[2].0 + 38.97114317029974).abs() < 1e-9);
        assert!((points[2].1 - 22.5).abs() < 1e-9);
        // 390° (= 30°), radius 50 · .80 → dist 40.
        assert!((points[6].0 - 40.0f64 * 30.0f64.to_radians().cos()).abs() < 1e-9);
        assert!((points[6].1 - 40.0f64 * 30.0f64.to_radians().sin()).abs() < 1e-9);
    }

    #[test]
    fn test_analyze_parameter_counts_and_errors_match_upstream() {
        // "Parameter 'X' must have N values".
        let err = CommandError::new("Parameter 'CENTER_DISTS' must have 6 values");
        assert_eq!(
            err.to_string(),
            "Parameter 'CENTER_DISTS' must have 6 values"
        );
        // "Unable to parse parameter 'X'".
        let err = CommandError::new("Unable to parse parameter 'SCALE'");
        assert_eq!(err.to_string(), "Unable to parse parameter 'SCALE'");
        // The other two fixed messages.
        assert_eq!(
            CommandError::new("Not all measurements provided").to_string(),
            "Not all measurements provided"
        );
        assert_eq!(
            CommandError::new("Must run basic calibration with DELTA_CALIBRATE first").to_string(),
            "Must run basic calibration with DELTA_CALIBRATE first"
        );
        assert_eq!(
            CommandError::new("Unknown calibrate action").to_string(),
            "Unknown calibrate action"
        );
        assert_eq!(
            CommandError::new("Delta calibrate is only for delta printers").to_string(),
            "Delta calibrate is only for delta printers"
        );
    }

    #[test]
    fn test_the_save_config_report_matches_the_upstream_wording() {
        // `DeltaCalibration.save_state`'s report lines, exactly as upstream
        // formats them (`delta.py:231-240`).
        let calibration = example_calibration();
        let line = format!(
            "stepper_a: position_endstop: {:.6} angle: {:.6} arm_length: {:.6}\n\
             stepper_b: position_endstop: {:.6} angle: {:.6} arm_length: {:.6}\n\
             stepper_c: position_endstop: {:.6} angle: {:.6} arm_length: {:.6}\n\
             delta_radius: {:.6}",
            calibration.endstops[0],
            calibration.angles[0],
            calibration.arms[0],
            calibration.endstops[1],
            calibration.angles[1],
            calibration.arms[1],
            calibration.endstops[2],
            calibration.angles[2],
            calibration.arms[2],
            calibration.radius,
        );
        let expected =
            "stepper_a: position_endstop: 297.050000 angle: 210.000000 arm_length: 333.000000\n\
             stepper_b: position_endstop: 297.050000 angle: 330.000000 arm_length: 333.000000\n\
             stepper_c: position_endstop: 297.050000 angle: 90.000000 arm_length: 333.000000\n\
             delta_radius: 174.750000";
        assert_eq!(line, expected);
        assert_eq!(
            "The SAVE_CONFIG command will update the printer config file\nwith these parameters and restart the printer.",
            "The SAVE_CONFIG command will update the printer config file\nwith these parameters and restart the printer."
        );
    }

    #[test]
    fn test_measurements_to_distances_builds_twelve_triples() {
        let calibration = example_calibration();
        let mut measured: HashMap<&'static str, Vec<f64>> = HashMap::new();
        measured.insert("SCALE", vec![1.]);
        measured.insert("CENTER_DISTS", vec![74.0; 6]);
        measured.insert("CENTER_PILLAR_WIDTHS", vec![9.0; 3]);
        measured.insert("OUTER_DISTS", vec![74.0; 6]);
        measured.insert("OUTER_PILLAR_WIDTHS", vec![9.0; 6]);
        let distances = measurements_to_distances(&measured, &calibration)
            .expect("a complete measurement set converts");
        assert_eq!(distances.len(), 12);
        // The centre distances shrink by the pillar widths in upstream's
        // reordered pattern ([0], [2], [1], …).
        assert!((distances[0].0 - (74. - 9.)).abs() < 1e-12);
        // And each pair carries distinct stable positions on either side of
        // the ridge (`calc_stable_position` over the inner/outer points).
        assert_ne!(distances[0].1, distances[0].2);
        // A partial set is refused with upstream's message.
        let mut incomplete = measured.clone();
        incomplete.remove("OUTER_PILLAR_WIDTHS");
        let err = measurements_to_distances(&incomplete, &calibration).unwrap_err();
        assert_eq!(err.to_string(), "Not all measurements provided");
    }

    #[test]
    fn test_a_saved_stable_position_round_trips_through_the_config_reader() {
        // A `height0_pos` line as the SAVE_CONFIG block writes it.
        let (config, _) = crate::core::klippy::config::Config::from_text(
            "[delta_calibrate]\nradius: 50\nheight0: 0.0\nheight0_pos: 2970499.999,2970499.999,2970499.999\n",
        )
        .expect("the config parses");
        let section = config.get_section("delta_calibrate").expect("the section");
        let wrapper = ConfigWrapper::untracked(section);
        let heights = read_heights(&wrapper, "height").expect("heights restore");
        assert_eq!(heights.len(), 1);
        assert_eq!(heights[0].0, 0.0);
        assert_eq!(heights[0].1, [2970499.999, 2970499.999, 2970499.999]);
    }
}
