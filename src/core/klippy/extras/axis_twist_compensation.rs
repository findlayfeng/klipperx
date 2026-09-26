//! `[axis_twist_compensation]` — compensates a gantry's X (and optional Y)
//! twist by nudging the probed Z (`klippy/extras/axis_twist_compensation.py`).
//!
//! The section reads the calibration travel (`calibrate_start_x` /
//! `calibrate_end_x` / `calibrate_y`, and the Y-axis mirror) and the stored
//! compensation curves (`z_compensations` / `zy_compensations` with their
//! `compensation_start_*` / `compensation_end_*` extents). It registers one
//! command — `AXIS_TWIST_COMPENSATION_CALIBRATE` — that probes a line of points
//! and records the offsets between the hardware probe and a manual (nozzle)
//! probe, then folds the average out so the curve is independent of `z_offset`.
//!
//! The compensation itself rides the probe's result event: after each probing
//! move the probe fires `probe:update_results` with the result it is about to
//! report, this section adds the interpolated `z_compensations` / `zy_*` value
//! for the probed X (and Y), and the probe reports the adjusted Z
//! (`axis_twist_compensation.py:57-86`, `probe.py:329`).
//!
//! # Port scope
//!
//! Every option is read (so `check_unused` accepts the section), the
//! interpolation and the event handler are complete, and the calibration
//! command runs the full wizard: it validates `SAMPLE_COUNT`/`AXIS` and the
//! required `calibrate_*` options, probes each point, hands the nozzle to the
//! interactive manual probe, and on the last point writes the curve back
//! through `configfile.set` (the `SAVE_CONFIG` write-back). The interactive
//! step needs a running reactor to continue to the next point, so a host
//! without one stops after the first point, as upstream would fail without a
//! reactor to drive its greenlets.
//!
//! Upstream's object defines no `get_status`; neither does this one
//! ([`is_queryable`](PrinterObject::is_queryable) is `false`).

use std::collections::HashMap;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper, PrinterConfig};
use crate::core::klippy::error::KlippyError;
use crate::core::klippy::event::printer_bus::ProbeResultsHandle;
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::manual_probe::{
    FinalizeCallback, ManualProbe, MANUAL_PROBE_OBJECT,
};
use crate::core::klippy::extras::probe::{lookup_probe_session, ProbeSession};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{CommandError, GCodeDispatch, GcodeCommand, GCODE_OBJECT};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{Coord, X_AXIS, Y_AXIS, Z_AXIS};
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};

section!("axis_twist_compensation", order = 30, load = load_config);

/// The toolhead object, as the loader registers `[printer]`.
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The `configfile` object, for the `SAVE_CONFIG` write-back.
const CONFIGFILE_OBJECT: &str = "configfile";

/// Default number of points the calibration probes
/// (`axis_twist_compensation.py:DEFAULT_SAMPLE_COUNT`).
const DEFAULT_SAMPLE_COUNT: i64 = 3;

/// Default speed for the calibration's travels
/// (`axis_twist_compensation.py:DEFAULT_SPEED`).
const DEFAULT_SPEED: f64 = 50.;

/// Default `horizontal_move_z` (`axis_twist_compensation.py:DEFAULT_HORIZONTAL_MOVE_Z`).
const DEFAULT_HORIZONTAL_MOVE_Z: f64 = 5.;

/// The command's help, byte for byte (`cmd_AXIS_TWIST_COMPENSATION_CALIBRATE_help`,
/// `axis_twist_compensation.py:133-137`).
const CALIBRATE_HELP: &str = "\n    Performs the x twist calibration wizard\n    Measure z probe offset at n points along the x axis,\n    and calculate x twist compensation\n    ";

/// The `[axis_twist_compensation]` options as written
/// (`axis_twist_compensation.py:22-46`).
#[derive(Debug, Clone, PartialEq)]
pub struct AxisTwistOptions {
    /// The lift between points, default `5.`.
    pub horizontal_move_z: f64,
    /// The calibration travel speed, default `50.`.
    pub speed: f64,
    /// X-axis calibration start X, or none.
    pub calibrate_start_x: Option<f64>,
    /// X-axis calibration end X, or none.
    pub calibrate_end_x: Option<f64>,
    /// The X-axis calibration's fixed Y.
    pub calibrate_y: Option<f64>,
    /// The stored X compensation curve.
    pub z_compensations: Vec<f64>,
    /// Where `z_compensations[0]` applies.
    pub compensation_start_x: Option<f64>,
    /// Where `z_compensations[-1]` applies.
    pub compensation_end_x: Option<f64>,
    /// Y-axis calibration start Y, or none.
    pub calibrate_start_y: Option<f64>,
    /// Y-axis calibration end Y, or none.
    pub calibrate_end_y: Option<f64>,
    /// The Y-axis calibration's fixed X.
    pub calibrate_x: Option<f64>,
    /// The stored Y compensation curve.
    pub zy_compensations: Vec<f64>,
    /// Where `zy_compensations[0]` applies.
    pub compensation_start_y: Option<f64>,
    /// Where `zy_compensations[-1]` applies.
    pub compensation_end_y: Option<f64>,
}

impl AxisTwistOptions {
    /// Read every option the section accepts, so `check_unused` passes.
    ///
    /// # Errors
    /// As the option readers: an unparseable float, or an element of
    /// `z_compensations` / `zy_compensations` that is not a number.
    pub fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        Ok(Self {
            horizontal_move_z: config
                .get_float("horizontal_move_z", Some(DEFAULT_HORIZONTAL_MOVE_Z))?,
            speed: config.get_float("speed", Some(DEFAULT_SPEED))?,
            calibrate_start_x: config.get_optional_float("calibrate_start_x")?,
            calibrate_end_x: config.get_optional_float("calibrate_end_x")?,
            calibrate_y: config.get_optional_float("calibrate_y")?,
            z_compensations: read_float_list(config, "z_compensations")?,
            compensation_start_x: config.get_optional_float("compensation_start_x")?,
            compensation_end_x: config.get_optional_float("compensation_end_x")?,
            calibrate_start_y: config.get_optional_float("calibrate_start_y")?,
            calibrate_end_y: config.get_optional_float("calibrate_end_y")?,
            calibrate_x: config.get_optional_float("calibrate_x")?,
            zy_compensations: read_float_list(config, "zy_compensations")?,
            compensation_start_y: config.get_optional_float("compensation_start_y")?,
            compensation_end_y: config.get_optional_float("compensation_end_y")?,
        })
    }
}

/// Upstream's `getlists(..., parser=float)` (`configfile.py`): a comma-separated
/// list, absent meaning empty.
///
/// # Errors
/// An element that is not a number keeps the parser's `Unable to parse option`
/// wording.
fn read_float_list(config: &ConfigWrapper, option: &str) -> Result<Vec<f64>, ConfigError> {
    let Some(items) = config.get_list(option, ',') else {
        return Ok(Vec::new());
    };
    items
        .iter()
        .map(|item| {
            item.trim().parse::<f64>().map_err(|_| {
                ConfigError::new(format!(
                    "Unable to parse option '{option}' in section '{}'",
                    config.identifier()
                ))
            })
        })
        .collect()
}

/// The live compensation curves, updated by the calibration and read by the
/// probe result handler (`AxisTwistCompensation`'s mutable fields).
#[derive(Debug, Clone, Default, PartialEq)]
struct Compensations {
    /// The X curve.
    z: Vec<f64>,
    /// Where `z[0]` applies.
    start_x: Option<f64>,
    /// Where `z[-1]` applies.
    end_x: Option<f64>,
    /// The Y curve.
    zy: Vec<f64>,
    /// Where `zy[0]` applies.
    start_y: Option<f64>,
    /// Where `zy[-1]` applies.
    end_y: Option<f64>,
}

impl Compensations {
    /// The curves the config starts with (`__init__`, `axis_twist_compensation.py:22-46`).
    fn from_options(options: &AxisTwistOptions) -> Self {
        Self {
            z: options.z_compensations.clone(),
            start_x: options.compensation_start_x,
            end_x: options.compensation_end_x,
            zy: options.zy_compensations.clone(),
            start_y: options.compensation_start_y,
            end_y: options.compensation_end_y,
        }
    }
}

/// Upstream's `bed_mesh.constrain` (`bed_mesh.py:30-31`).
fn constrain(value: f64, min: f64, max: f64) -> f64 {
    max.min(min.max(value))
}

/// Upstream's `bed_mesh.lerp` (`bed_mesh.py:34-35`).
fn lerp(t: f64, v0: f64, v1: f64) -> f64 {
    (1. - t) * v0 + t * v1
}

/// The compensation at `coord` along one curve
/// (`_get_interpolated_z_compensation`, `axis_twist_compensation.py:88-105`).
///
/// The curve is a piecewise-linear function over
/// `[comp_start, comp_end]`; a coordinate outside the range clamps to the end
/// segments (upstream's `constrain`). A one-element curve has no interval to
/// interpolate over — upstream would divide by zero — so it answers its sole
/// value.
fn interpolated_z_compensation(
    coord: f64,
    z_compensations: &[f64],
    comp_start: f64,
    comp_end: f64,
) -> f64 {
    let sample_count = z_compensations.len();
    if sample_count < 2 {
        return z_compensations.first().copied().unwrap_or(0.);
    }
    let spacing = (comp_end - comp_start) / (sample_count as f64 - 1.);
    let mut interpolate_t = (coord - comp_start) / spacing;
    let mut interpolate_i = interpolate_t.floor();
    interpolate_i = constrain(interpolate_i, 0., sample_count as f64 - 2.);
    interpolate_t -= interpolate_i;
    let index = interpolate_i as usize;
    lerp(
        interpolate_t,
        z_compensations[index],
        z_compensations[index + 1],
    )
}

/// Add the interpolated compensation to every probed Z
/// (`_update_z_compensation_value`, `axis_twist_compensation.py:57-86`).
///
/// The X curve reads the probed X, the Y curve the probed Y; a curve without
/// its `compensation_start_*` / `compensation_end_*` extent is skipped, since
/// there is nowhere to place it.
fn compensate_positions(state: &Compensations, positions: &mut [Coord]) {
    for pos in positions.iter_mut() {
        let mut zo = 0.;
        if !state.z.is_empty() {
            if let (Some(start), Some(end)) = (state.start_x, state.end_x) {
                zo += interpolated_z_compensation(pos.x(), &state.z, start, end);
            }
        }
        if !state.zy.is_empty() {
            if let (Some(start), Some(end)) = (state.start_y, state.end_y) {
                zo += interpolated_z_compensation(pos.y(), &state.zy, start, end);
            }
        }
        if zo != 0. {
            pos.set_axis(Z_AXIS, pos.z() + zo);
        }
    }
}

/// One probe measurement through the probe session (`run_single_probe`,
/// `probe.py:531-537`).
async fn run_single_probe(
    probe: &Arc<dyn ProbeSession>,
    gcmd: &GcodeCommand,
) -> Result<Coord, CommandError> {
    probe.start_probe_session(gcmd)?;
    probe.run_probe(gcmd).await?;
    let pos = probe
        .pull_probed_results()
        .into_iter()
        .next()
        .ok_or_else(|| CommandError::new("Internal probe error - no probe result"))?;
    probe.end_probe_session()?;
    Ok(pos)
}

/// Move to a Z, leaving X/Y (`_move_helper` with only Z set).
fn move_to_z(toolhead: &ToolHeadObject, z: f64, speed: f64) -> Result<(), CommandError> {
    let mut pos = toolhead
        .position()
        .ok_or_else(|| CommandError::new("Printer is not ready"))?;
    pos.set_axis(Z_AXIS, z);
    toolhead.move_to(pos, speed)
}

/// Move to an X/Y, leaving Z (`_move_helper` with X/Y set).
fn move_to_xy(toolhead: &ToolHeadObject, x: f64, y: f64, speed: f64) -> Result<(), CommandError> {
    let mut pos = toolhead
        .position()
        .ok_or_else(|| CommandError::new("Printer is not ready"))?;
    pos.set_axis(X_AXIS, x);
    pos.set_axis(Y_AXIS, y);
    toolhead.move_to(pos, speed)
}

/// The calibration wizard (`axis_twist_compensation.py:Calibrater`).
///
/// Shared behind an `Arc`: the manual-probe callback that ends one point starts
/// the next, so the state outlives the command that began the run.
struct Calibrater {
    /// The machine, for the toolhead and the sibling objects.
    printer: Weak<Printer>,
    /// The live compensation curves, updated on `_finalize_calibration`.
    state: Arc<Mutex<Compensations>>,
    /// The section id `_finalize_calibration` writes back to.
    configname: String,
    /// The travel speed (`speed`).
    speed: f64,
    /// The lift between points (`horizontal_move_z`).
    horizontal_move_z: f64,
    /// `(calibrate_start_x, calibrate_y)`.
    x_start_point: (Option<f64>, Option<f64>),
    /// `(calibrate_end_x, calibrate_y)`.
    x_end_point: (Option<f64>, Option<f64>),
    /// `(calibrate_x, calibrate_start_y)`.
    y_start_point: (Option<f64>, Option<f64>),
    /// `(calibrate_x, calibrate_end_y)`.
    y_end_point: (Option<f64>, Option<f64>),
    /// The z-offsets measured so far in the running calibration.
    results: Mutex<Vec<f64>>,
    /// Which point is being probed.
    current_point_index: AtomicUsize,
    /// The hardware probe's Z at the current point.
    current_measured_z: Mutex<f64>,
    /// The axis being calibrated (`X` or `Y`).
    current_axis: Mutex<String>,
}

impl Calibrater {
    /// Reset the running calibration (upstream's `cmd_…` preamble).
    fn begin(&self, axis: &str) -> Result<(), CommandError> {
        self.clear_compensations(Some(axis.as_ref()));
        *self.current_axis.lock().unwrap_or_else(|p| p.into_inner()) = axis.to_string();
        self.results
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clear();
        self.current_point_index.store(0, Ordering::SeqCst);
        Ok(())
    }

    /// `clear_compensations` (`axis_twist_compensation.py:107-113`).
    fn clear_compensations(&self, axis: Option<&str>) {
        let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        match axis {
            None => {
                state.z.clear();
                state.zy.clear();
            }
            Some("X") => state.z.clear(),
            Some("Y") => state.zy.clear(),
            _ => {}
        }
    }

    /// `AXIS_TWIST_COMPENSATION_CALIBRATE` (`axis_twist_compensation.py:139-233`).
    async fn cmd_calibrate(self: Arc<Self>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let probe = lookup_probe_session(&printer).ok_or_else(|| {
            CommandError::new("AXIS_TWIST_COMPENSATION requires [probe] to be defined")
        })?;
        let offsets = probe.offsets();
        let sample_count = gcmd.get_int_default("SAMPLE_COUNT", DEFAULT_SAMPLE_COUNT)?;
        let axis = gcmd.get_str_default("AXIS", "X");

        if sample_count < 2 {
            return Err(CommandError::new(
                "SAMPLE_COUNT to probe must be at least 2",
            ));
        }

        let mut nozzle_points: Vec<(f64, f64)> = Vec::new();
        if axis == "X" {
            self.clear_compensations(Some("X"));
            let (start_x, end_x, y) = (
                self.x_start_point.0,
                self.x_end_point.0,
                self.x_start_point.1,
            );
            let (Some(start_x), Some(end_x), Some(y)) = (start_x, end_x, y) else {
                return Err(CommandError::new(
                    "AXIS_TWIST_COMPENSATION for X axis requires\n                    calibrate_start_x, calibrate_end_x and calibrate_y\n                    to be defined\n                    ",
                ));
            };
            let interval_dist = (end_x - start_x) / (sample_count - 1) as f64;
            for i in 0..sample_count {
                nozzle_points.push((start_x + i as f64 * interval_dist, y));
            }
        } else if axis == "Y" {
            self.clear_compensations(Some("Y"));
            let (start_y, end_y, x) = (
                self.y_start_point.1,
                self.y_end_point.1,
                self.y_start_point.0,
            );
            let (Some(start_y), Some(end_y), Some(x)) = (start_y, end_y, x) else {
                return Err(CommandError::new(
                    "AXIS_TWIST_COMPENSATION for Y axis requires\n                    calibrate_start_y, calibrate_end_y and calibrate_x\n                    to be defined\n                    ",
                ));
            };
            let interval_dist = (end_y - start_y) / (sample_count - 1) as f64;
            for i in 0..sample_count {
                nozzle_points.push((x, start_y + i as f64 * interval_dist));
            }
        } else {
            return Err(CommandError::new(
                "AXIS_TWIST_COMPENSATION_CALIBRATE: Invalid axis.",
            ));
        }

        // `_calculate_probe_points`: net the nozzle positions of the offsets.
        let probe_points: Vec<(f64, f64)> = nozzle_points
            .iter()
            .map(|(x, y)| (x - offsets.x, y - offsets.y))
            .collect();

        let manual_probe = printer
            .lookup_object_as::<ManualProbe>(MANUAL_PROBE_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        manual_probe.verify_no_manual_probe(&printer)?;

        self.begin(&axis)?;
        self.probe_point(gcmd, 0, &probe_points, &nozzle_points)
            .await
    }

    /// Probe one point, then hand over to the manual probe
    /// (`_calibration`, `axis_twist_compensation.py:236-283`).
    async fn probe_point(
        self: &Arc<Self>,
        gcmd: &GcodeCommand,
        index: usize,
        probe_points: &[(f64, f64)],
        nozzle_points: &[(f64, f64)],
    ) -> Result<(), CommandError> {
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");
        let toolhead = printer
            .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let probe = lookup_probe_session(&printer).ok_or_else(|| {
            CommandError::new("AXIS_TWIST_COMPENSATION requires [probe] to be defined")
        })?;
        let manual_probe = printer
            .lookup_object_as::<ManualProbe>(MANUAL_PROBE_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let lift_speed = probe.probe_params(gcmd)?.lift_speed;

        gcode.respond_info(
            &format!(
                "AXIS_TWIST_COMPENSATION_CALIBRATE: Probing point {} of {}",
                index + 1,
                probe_points.len()
            ),
            true,
        );
        move_to_z(&toolhead, self.horizontal_move_z, lift_speed)?;
        move_to_xy(
            &toolhead,
            probe_points[index].0,
            probe_points[index].1,
            self.speed,
        )?;
        let pos = run_single_probe(&probe, gcmd).await?;
        *self
            .current_measured_z
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = pos.z();
        move_to_z(&toolhead, self.horizontal_move_z, lift_speed)?;
        move_to_xy(
            &toolhead,
            nozzle_points[index].0,
            nozzle_points[index].1,
            self.speed,
        )?;

        // `_manual_probe_callback_factory`: record this point's offset, and
        // either finish or continue on the next one.
        let is_end = index == probe_points.len() - 1;
        let this = Arc::clone(self);
        let probe_points = probe_points.to_vec();
        let nozzle_points = nozzle_points.to_vec();
        let callback: FinalizeCallback = Arc::new(move |mpresult: Option<Coord>| {
            let Some(mpresult) = mpresult else {
                this.report(
                    "AXIS_TWIST_COMPENSATION_CALIBRATE: Probe cancelled, calibration aborted",
                );
                return;
            };
            let measured = *this
                .current_measured_z
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            let z_offset = measured - mpresult.z();
            this.results
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(z_offset);
            if is_end {
                this.finalize();
            } else {
                this.current_point_index.store(index + 1, Ordering::SeqCst);
                let next = Arc::clone(&this);
                let probe_points = probe_points.clone();
                let nozzle_points = nozzle_points.clone();
                next.continue_at(index + 1, probe_points, nozzle_points);
            }
        });
        manual_probe.start_helper(&printer, gcmd, callback)?;
        Ok(())
    }

    /// Start the next point from the manual-probe callback, which is not
    /// itself async: the continuation runs as a spawned task when a reactor is
    /// driving the host.
    fn continue_at(
        self: Arc<Self>,
        index: usize,
        probe_points: Vec<(f64, f64)>,
        nozzle_points: Vec<(f64, f64)>,
    ) {
        let Ok(handle) = tokio::runtime::Handle::try_current() else {
            return;
        };
        handle.spawn(async move {
            let Some(printer) = self.printer.upgrade() else {
                return;
            };
            let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) else {
                return;
            };
            let gcmd =
                gcode.create_gcode_command("AXIS_TWIST_COMPENSATION_CALIBRATE", "", HashMap::new());
            if let Err(err) = self
                .probe_point(&gcmd, index, &probe_points, &nozzle_points)
                .await
            {
                gcode.respond_info(&format!("AXIS_TWIST_COMPENSATION_CALIBRATE: {err}"), true);
            }
        });
    }

    /// `_finalize_calibration` (`axis_twist_compensation.py:284-345`): average
    /// the offsets, fold the average out, write the curve back and store it.
    fn finalize(self: &Arc<Self>) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) else {
            return;
        };
        let results = self
            .results
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        if results.is_empty() {
            return;
        }
        let avg = results.iter().sum::<f64>() / results.len() as f64;
        // Subtract the average so the curve is independent of `z_offset`.
        let adjusted: Vec<f64> = results.iter().map(|x| avg - x).collect();
        let values_as_str = adjusted
            .iter()
            .map(|x| format!("{x:.6}"))
            .collect::<Vec<_>>()
            .join(", ");
        let axis = self
            .current_axis
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        let configfile = printer.lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT);
        {
            let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
            match axis.as_str() {
                "X" => {
                    if let Some(configfile) = &configfile {
                        configfile.set(&self.configname, "z_compensations", &values_as_str);
                    }
                    if let Some(start) = self.x_start_point.0 {
                        if let Some(configfile) = &configfile {
                            configfile.set(
                                &self.configname,
                                "compensation_start_x",
                                &start.to_string(),
                            );
                        }
                    }
                    if let Some(end) = self.x_end_point.0 {
                        if let Some(configfile) = &configfile {
                            configfile.set(
                                &self.configname,
                                "compensation_end_x",
                                &end.to_string(),
                            );
                        }
                    }
                    state.z = adjusted.clone();
                    state.start_x = self.x_start_point.0;
                    state.end_x = self.x_end_point.0;
                }
                "Y" => {
                    if let Some(configfile) = &configfile {
                        configfile.set(&self.configname, "zy_compensations", &values_as_str);
                    }
                    if let Some(start) = self.y_start_point.1 {
                        if let Some(configfile) = &configfile {
                            configfile.set(
                                &self.configname,
                                "compensation_start_y",
                                &start.to_string(),
                            );
                        }
                    }
                    if let Some(end) = self.y_end_point.1 {
                        if let Some(configfile) = &configfile {
                            configfile.set(
                                &self.configname,
                                "compensation_end_y",
                                &end.to_string(),
                            );
                        }
                    }
                    state.zy = adjusted.clone();
                    state.start_y = self.y_start_point.1;
                    state.end_y = self.y_end_point.1;
                }
                _ => {}
            }
        }
        gcode.respond_info(
            "AXIS_TWIST_COMPENSATION state has been saved for the current session.  \
             The SAVE_CONFIG command will update the printer config file and restart the printer.",
            true,
        );
        let offsets_repr = format!(
            "[{}]",
            adjusted
                .iter()
                .map(|x| x.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        );
        gcode.respond_info(
            &format!(
                "AXIS_TWIST_COMPENSATION_CALIBRATE: Calibration complete, offsets: {offsets_repr}, mean z_offset: {avg:.6}"
            ),
            true,
        );
    }

    /// Emit an informational line from a callback that has no command in hand.
    fn report(&self, message: &str) {
        if let Some(printer) = self.printer.upgrade() {
            if let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) {
                gcode.respond_info(message, true);
            }
        }
    }
}

/// The `[axis_twist_compensation]` object (`AxisTwistCompensation`).
pub struct AxisTwistCompensation {
    /// The machine, for the connect check.
    printer: Weak<Printer>,
    /// The options as read.
    options: AxisTwistOptions,
}

impl AxisTwistCompensation {
    /// Read the options, register the result handler and the calibration
    /// command.
    ///
    /// # Errors
    /// As [`AxisTwistOptions::read`], or when the command name is taken.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let options = AxisTwistOptions::read(config)?;
        let state = Arc::new(Mutex::new(Compensations::from_options(&options)));

        let calibrater = Arc::new(Calibrater {
            printer: Arc::downgrade(printer),
            state: Arc::clone(&state),
            configname: config.identifier().to_string(),
            speed: options.speed,
            horizontal_move_z: options.horizontal_move_z,
            x_start_point: (options.calibrate_start_x, options.calibrate_y),
            x_end_point: (options.calibrate_end_x, options.calibrate_y),
            y_start_point: (options.calibrate_x, options.calibrate_start_y),
            y_end_point: (options.calibrate_x, options.calibrate_end_y),
            results: Mutex::new(Vec::new()),
            current_point_index: AtomicUsize::new(0),
            current_measured_z: Mutex::new(0.),
            current_axis: Mutex::new("X".to_string()),
        });

        // The probe result handler: it edits the reported Z in place.
        {
            let state = Arc::clone(&state);
            printer.register_event_handler(
                KlippyEvent::ProbeUpdateResults {
                    results: ProbeResultsHandle::new(Vec::new()),
                },
                Box::new(move |event| {
                    if let KlippyEvent::ProbeUpdateResults { results } = event {
                        let state = state.lock().unwrap_or_else(|p| p.into_inner());
                        let shared = results.shared();
                        let mut positions = shared.lock().unwrap_or_else(|p| p.into_inner());
                        compensate_positions(&state, &mut positions);
                    }
                }),
            );
        }

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");
        {
            let calibrater = Arc::clone(&calibrater);
            gcode
                .register_command(
                    "AXIS_TWIST_COMPENSATION_CALIBRATE",
                    Arc::new(move |gcmd| {
                        let calibrater = Arc::clone(&calibrater);
                        Box::pin(async move { calibrater.cmd_calibrate(gcmd).await })
                    }),
                    Some(CALIBRATE_HELP),
                    false,
                )
                .map_err(ConfigError::new)?;
        }

        Ok(Self {
            printer: Arc::downgrade(printer),
            options,
        })
    }

    /// The options as read.
    pub fn options(&self) -> &AxisTwistOptions {
        &self.options
    }
}

impl PrinterObject for AxisTwistCompensation {
    /// Upstream's object defines no `get_status`; this answers the empty
    /// object the API path falls back to for a part without one.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    /// Not client-visible: upstream defines no `get_status` (`objects/list`
    /// keeps only parts that do, `klippy/klippy.py`).
    fn is_queryable(&self) -> bool {
        false
    }

    /// `Calibrater._handle_connect` (`axis_twist_compensation.py:116-122`):
    /// a `[probe]` section is required.
    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            let Some(printer) = self.printer.upgrade() else {
                return Ok(());
            };
            if lookup_probe_session(&printer).is_none() {
                return Err(KlippyError::Config(ConfigError::new(
                    "AXIS_TWIST_COMPENSATION requires [probe] to be defined",
                )));
            }
            Ok(())
        })
    }
}

impl std::fmt::Debug for AxisTwistCompensation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("AxisTwistCompensation")
            .field("options", &self.options)
            .finish()
    }
}

/// Upstream's `load_config` for `[axis_twist_compensation]`.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(AxisTwistCompensation::new(config, printer)?))
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::access::AccessTracking;
    use crate::core::klippy::config::{check_unused, Config, ConfigSection, ConfigValue};
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[axis_twist_compensation]` section with the given options, as the
    /// parser would build it.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("axis_twist_compensation", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    // --- Config parsing ---------------------------------------------------

    /// The bare section reads upstream's defaults: the travels and an empty
    /// curve, with every calibration extent unset
    /// (`axis_twist_compensation.py:22-46`).
    #[test]
    fn the_bare_section_answers_upstreams_defaults() {
        let options = AxisTwistOptions::read(&ConfigWrapper::untracked(&section(&[])))
            .expect("the bare section reads");

        assert_eq!(options.horizontal_move_z, 5.);
        assert_eq!(options.speed, 50.);
        assert_eq!(options.calibrate_start_x, None);
        assert_eq!(options.calibrate_end_x, None);
        assert_eq!(options.calibrate_y, None);
        assert!(options.z_compensations.is_empty());
        assert_eq!(options.compensation_start_x, None);
        assert_eq!(options.compensation_end_x, None);
        assert_eq!(options.calibrate_start_y, None);
        assert_eq!(options.calibrate_end_y, None);
        assert_eq!(options.calibrate_x, None);
        assert!(options.zy_compensations.is_empty());
        assert_eq!(options.compensation_start_y, None);
        assert_eq!(options.compensation_end_y, None);
    }

    /// Every option the corpus writes is read back through the tracker, so
    /// `check_unused` accepts the section — the option-level half of the
    /// loader's validation (`config/validate.rs:48`).
    #[test]
    fn every_option_the_section_writes_is_recorded_as_read() {
        let text = "\
[axis_twist_compensation]
horizontal_move_z: 10
speed: 200
calibrate_start_x: 3
calibrate_end_x: 207
calibrate_y: 110
z_compensations: 0.0001, -0.02, 0.03
compensation_start_x: 3
compensation_end_x: 207
calibrate_start_y: 5
calibrate_end_y: 195
calibrate_x: 110
zy_compensations: 0.01, 0.02
compensation_start_y: 5
compensation_end_y: 195
";
        let (config, _) = Config::from_text(text).expect("the section parses");
        let sect = config
            .get_section("axis_twist_compensation")
            .expect("the section");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::new(sect, Arc::clone(&access));

        let parsed = AxisTwistOptions::read(&wrapper).expect("the section reads");
        check_unused(&config, &access, &[]).expect("no option is left unread");

        assert_eq!(parsed.horizontal_move_z, 10.);
        assert_eq!(parsed.speed, 200.);
        assert_eq!(parsed.calibrate_start_x, Some(3.));
        assert_eq!(parsed.calibrate_end_x, Some(207.));
        assert_eq!(parsed.calibrate_y, Some(110.));
        assert_eq!(parsed.z_compensations, vec![0.0001, -0.02, 0.03]);
        assert_eq!(parsed.compensation_start_x, Some(3.));
        assert_eq!(parsed.compensation_end_x, Some(207.));
        assert_eq!(parsed.calibrate_start_y, Some(5.));
        assert_eq!(parsed.calibrate_end_y, Some(195.));
        assert_eq!(parsed.calibrate_x, Some(110.));
        assert_eq!(parsed.zy_compensations, vec![0.01, 0.02]);
        assert_eq!(parsed.compensation_start_y, Some(5.));
        assert_eq!(parsed.compensation_end_y, Some(195.));
    }

    /// A compensation curve element that is not a number keeps the parser's
    /// wording, and the `None`-default extents stay `None` (`getlists` with
    /// `parser=float`, `configfile.py`).
    #[test]
    fn a_non_numeric_curve_keeps_the_parser_wording() {
        let sect = section(&[("z_compensations", "0.1, x")]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            AxisTwistOptions::read(&config).unwrap_err().to_string(),
            "Unable to parse option 'z_compensations' in section 'axis_twist_compensation'"
        );

        let sect = section(&[("speed", "fast")]);
        let config = ConfigWrapper::untracked(&sect);
        assert_eq!(
            AxisTwistOptions::read(&config).unwrap_err().to_string(),
            "Unable to parse option 'speed' in section 'axis_twist_compensation'"
        );
    }

    /// The command's help is the upstream string byte for byte
    /// (`axis_twist_compensation.py:133-137`).
    #[test]
    fn the_calibrate_help_is_upstreams_string_verbatim() {
        assert_eq!(
            CALIBRATE_HELP,
            "\n    Performs the x twist calibration wizard\n    \
             Measure z probe offset at n points along the x axis,\n    \
             and calculate x twist compensation\n    "
        );
    }

    // --- Interpolation ----------------------------------------------------

    /// A two-point curve is a straight line through its endpoints, and a
    /// coordinate outside the extent clamps to the near segment
    /// (`axis_twist_compensation.py:88-105`, upstream's `bed_mesh.constrain`).
    #[test]
    fn a_two_point_curve_interpolates_and_clamps() {
        let curve = [0.0, 0.4];
        assert_eq!(interpolated_z_compensation(0., &curve, 0., 100.), 0.);
        assert_eq!(interpolated_z_compensation(100., &curve, 0., 100.), 0.4);
        assert_eq!(interpolated_z_compensation(50., &curve, 0., 100.), 0.2);
        // Beyond either end upstream clamps the *segment index* but keeps the
        // residual `t`, so the end segment extrapolates
        // (`axis_twist_compensation.py:92-105`).
        assert!((interpolated_z_compensation(-10., &curve, 0., 100.) + 0.04).abs() < 1e-12);
        assert_eq!(interpolated_z_compensation(200., &curve, 0., 100.), 0.8);
    }

    /// A three-point curve hits its middle sample and interpolates between the
    /// samples around a coordinate (`axis_twist_compensation.py:92-105`).
    #[test]
    fn a_three_point_curve_hits_its_samples() {
        let curve = [1.0, 3.0, 2.0];
        assert_eq!(interpolated_z_compensation(0., &curve, 0., 100.), 1.0);
        assert_eq!(interpolated_z_compensation(50., &curve, 0., 100.), 3.0);
        assert_eq!(interpolated_z_compensation(100., &curve, 0., 100.), 2.0);
        // Halfway into the first segment.
        assert_eq!(interpolated_z_compensation(25., &curve, 0., 100.), 2.0);
    }

    /// A one-element curve has no interval; upstream would divide by zero, so
    /// the port answers its sole value (`axis_twist_compensation.py:92-105`).
    #[test]
    fn a_one_point_curve_answers_its_sole_value() {
        assert_eq!(interpolated_z_compensation(42., &[0.7], 0., 100.), 0.7);
        assert_eq!(interpolated_z_compensation(42., &[], 0., 100.), 0.);
    }

    // --- Compensation -----------------------------------------------------

    /// The X curve reads the probed X and the Y curve the probed Y; both add to
    /// the probed Z (`_update_z_compensation_value`,
    /// `axis_twist_compensation.py:57-86`).
    #[test]
    fn the_curves_add_to_the_probed_z() {
        let state = Compensations {
            z: vec![0.0, 1.0],
            start_x: Some(0.),
            end_x: Some(100.),
            zy: vec![0.0, 0.5],
            start_y: Some(0.),
            end_y: Some(100.),
            ..Default::default()
        };
        let mut positions = vec![
            Coord::new(50., 50., 10., 0.),
            Coord::new(0., 0., 10., 0.),
            Coord::new(100., 100., 10., 0.),
        ];
        compensate_positions(&state, &mut positions);

        assert_eq!(positions[0].z(), 10. + 0.5 + 0.25);
        assert_eq!(positions[1].z(), 10.);
        assert_eq!(positions[2].z(), 10. + 1.0 + 0.5);
    }

    /// A curve without its extent is skipped rather than applied at a guessed
    /// origin (`axis_twist_compensation.py:60-85`).
    #[test]
    fn a_curve_without_its_extent_is_skipped() {
        let state = Compensations {
            z: vec![0.0, 1.0],
            ..Default::default()
        };
        let mut positions = vec![Coord::new(50., 50., 10., 0.)];
        compensate_positions(&state, &mut positions);
        assert_eq!(positions[0].z(), 10.);
    }

    // --- Loader integration ----------------------------------------------

    /// The loader builds the section, registers the calibration command with
    /// upstream's help, and the part is not client-visible (upstream defines no
    /// `get_status`).
    #[test]
    fn the_section_registers_its_command_and_hides_from_objects_list() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = Config::from_text("[axis_twist_compensation]\n").expect("parses");
        printer.load_config(&config).expect("the section loads");

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        assert_eq!(
            gcode
                .command_help()
                .get("AXIS_TWIST_COMPENSATION_CALIBRATE"),
            Some(&CALIBRATE_HELP.to_string())
        );

        let part = printer
            .lookup_object_as::<AxisTwistCompensation>("axis_twist_compensation")
            .expect("the section is registered");
        assert!(!part.is_queryable());
        assert_eq!(part.get_status(0.), json!({}));
    }

    /// The probe result event carries a shared result the section edits in
    /// place — the seam `probe.py:329` uses (`axis_twist_compensation.py:57-86`).
    #[test]
    fn the_result_handler_edits_the_shared_result_in_place() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .expect("gcode registers");
        let (config, _) = Config::from_text(
            "[axis_twist_compensation]\n\
             z_compensations: 0, 1\n\
             compensation_start_x: 0\n\
             compensation_end_x: 100\n",
        )
        .expect("parses");
        let sect = config
            .get_section("axis_twist_compensation")
            .expect("the section");
        let wrapper = ConfigWrapper::untracked(sect);
        AxisTwistCompensation::new(&wrapper, &printer).expect("the section builds");

        let results = ProbeResultsHandle::new(vec![Coord::new(50., 10., 7.0, 0.)]);
        printer.send_event(&KlippyEvent::ProbeUpdateResults {
            results: results.clone(),
        });

        let positions = results.to_vec();
        assert_eq!(positions[0].z(), 7.0 + 0.5);
        assert_eq!(positions[0].x(), 50.);
    }
}
