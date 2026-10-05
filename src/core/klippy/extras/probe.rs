//! `[probe]` — the probe's virtual Z endstop, its session and its commands.
//!
//! Upstream `klippy/extras/probe.py`. This module lands the `[probe]` section,
//! its option set, the `probe` virtual pin chip, the probe session (sampling
//! with tolerance retries) and `QUERY_PROBE` / `PROBE` / `PROBE_ACCURACY` /
//! `PROBE_CALIBRATE` (the last one hands over to `manual_probe` and writes
//! `z_offset` back through `configfile.set()`).
//!
//! What is **not** here yet (tracked in `TODO.md` H9):
//!
//! - `Z_OFFSET_APPLY_PROBE` (the same command exists for `probe_eddy_current`,
//!   `probe_eddy_current.rs`), and the *stow* half of upstream's wrapper:
//!   `ProbeEndstopWrapper` drives `activate_gcode` / `deactivate_gcode` around
//!   each sample and keeps the `OFF`/`FIRST`/`ON` multi-probe state
//!   (`probe.py:545-605`). Here those two options are read and recorded but not
//!   rendered, and there is no multi-probe state; a section that sets them warns
//!   (`probe.rs:665`).
//! - The **position** half of that wrapper is done: the pin chip's
//!   `virtual_endstop_position` returns `z_offset` and the rail takes
//!   `position_endstop` from it (upstream's `get_position_endstop`,
//!   `probe.py:235`) — `probe.rs:306`, `extras/stepper.rs:374-406`.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper, PrinterConfig};
use crate::core::klippy::event::printer_bus::ProbeResultsHandle;
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::manual_probe::{
    FinalizeCallback, ManualProbe, MANUAL_PROBE_OBJECT,
};
use crate::core::klippy::extras::probe_eddy_current;
use crate::core::klippy::extras::toolhead::{HomingEndstop, ToolHeadObject};
use crate::core::klippy::gcode::{
    CommandError, CommandFuture, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::Coord;
use crate::core::klippy::mcu::McuEndstop;
use crate::core::klippy::pins::{
    DigitalOut, PinChip, PinError, PinParams, PrinterPins, PINS_OBJECT,
};
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("probe", order = 30, load = load_config);

/// The chip name the virtual endstop is reached under.
const CHIP_NAME: &str = "probe";

/// The only pin name the chip answers to (`klippy/extras/probe.py:238-243`).
const VIRTUAL_ENDSTOP: &str = "z_virtual_endstop";

/// The toolhead object, as the loader registers `[printer]`.
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The `configfile` object, for the calibration write-back.
const CONFIGFILE_OBJECT: &str = "configfile";

/// The Z axis index, as [`Coord`] numbers them.
const Z_AXIS: usize = 2;

/// The `[probe]` options as written (`probe.py:563-600`).
#[derive(Debug, Clone, PartialEq)]
pub struct ProbeOptions {
    /// The physical probe pin.
    pub pin: String,
    /// The probe's trigger offset from the nozzle.
    pub z_offset: f64,
    /// Probe-to-nozzle X offset.
    pub x_offset: f64,
    /// Probe-to-nozzle Y offset.
    pub y_offset: f64,
    /// Probing speed.
    pub speed: f64,
    /// Speed for the retract moves between samples.
    pub lift_speed: Option<f64>,
    /// Samples per probe.
    pub samples: i64,
    /// Retract distance between samples.
    pub sample_retract_dist: f64,
    /// `median` or `average`.
    pub samples_result: String,
    /// How far the samples may spread.
    pub samples_tolerance: f64,
    /// How many times a spread sample set is retried.
    pub samples_tolerance_retries: i64,
    /// Retract the probe between samples.
    pub deactivate_on_each_sample: bool,
    /// G-code template run before probing (needs `[gcode_macro]`, H3).
    pub activate_gcode: Option<String>,
    /// G-code template run after probing (needs `[gcode_macro]`, H3).
    pub deactivate_gcode: Option<String>,
}

impl ProbeOptions {
    /// Read every option the section accepts, so `check_unused` passes.
    ///
    /// # Errors
    /// As the option readers: a missing `pin` or `z_offset`, a `speed` that is
    /// not above zero, an unknown `samples_result`, and so on.
    pub fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        Ok(Self {
            pin: config.get("pin", None)?,
            z_offset: config.get_float("z_offset", None)?,
            x_offset: config.get_float("x_offset", Some(0.0))?,
            y_offset: config.get_float("y_offset", Some(0.0))?,
            speed: config.get_float_bounded("speed", Some(5.0), None, None, Some(0.0), None)?,
            lift_speed: config.get_optional_float("lift_speed")?,
            samples: config.get_int_bounded("samples", Some(1), Some(1), None)?,
            sample_retract_dist: config.get_float_bounded(
                "sample_retract_dist",
                Some(2.0),
                None,
                None,
                Some(0.0),
                None,
            )?,
            samples_result: config.get_choice(
                "samples_result",
                &["median", "average"],
                Some("median"),
            )?,
            samples_tolerance: config.get_float_bounded(
                "samples_tolerance",
                Some(0.100),
                Some(0.0),
                None,
                None,
                None,
            )?,
            samples_tolerance_retries: config.get_int_bounded(
                "samples_tolerance_retries",
                Some(0),
                Some(0),
                None,
            )?,
            deactivate_on_each_sample: config.get_bool("deactivate_on_each_sample", Some(true))?,
            activate_gcode: config.get_str("activate_gcode"),
            deactivate_gcode: config.get_str("deactivate_gcode"),
        })
    }
}

/// The XYZ offsets, as the probe family reads them (`ProbeOffsetsHelper`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProbeOffsets {
    /// Probe-to-nozzle X offset.
    pub x: f64,
    /// Probe-to-nozzle Y offset.
    pub y: f64,
    /// The probe's trigger offset from the nozzle.
    pub z: f64,
}

/// One probe session's parameters: the section's defaults, overridable per
/// command (`ProbeSessionHelper.get_probe_params`).
#[derive(Debug, Clone, PartialEq)]
pub struct ProbeParams {
    /// Probing speed.
    pub probe_speed: f64,
    /// Retract speed between samples.
    pub lift_speed: f64,
    /// Samples per probe.
    pub samples: i64,
    /// Retract distance between samples.
    pub sample_retract_dist: f64,
    /// How far the samples may spread.
    pub samples_tolerance: f64,
    /// How many times a spread sample set is retried.
    pub samples_tolerance_retries: i64,
    /// `median` or `average`.
    pub samples_result: String,
}

/// The `KEY` names [`ProbeParams::from_command`] reads, in read order: every
/// command that runs a probe session accepts them.
pub(crate) const PROBE_PARAMS: &[&str] = &[
    "PROBE_SPEED",
    "LIFT_SPEED",
    "SAMPLES",
    "SAMPLE_RETRACT_DIST",
    "SAMPLES_TOLERANCE",
    "SAMPLES_TOLERANCE_RETRIES",
    "SAMPLES_RESULT",
];

/// The `KEY` names [`ProbePointsHelper::start_probe`] reads before the probe
/// parameters, in read order (`probe.py:start_probe`).
pub(crate) const PROBE_POINTS_PARAMS: &[&str] = &["METHOD", "HORIZONTAL_MOVE_Z"];

/// The declared parameters of a command that drives a full
/// [`ProbePointsHelper::start_probe`] round: the method and move height the
/// round reads first, then the probe parameters it hands on to the session.
pub(crate) fn probe_points_params() -> Vec<&'static str> {
    let mut params = PROBE_POINTS_PARAMS.to_vec();
    params.extend_from_slice(PROBE_PARAMS);
    params
}

/// The declared parameters of `PROBE_CALIBRATE`: the probe parameters, then
/// the `SPEED` the interactive manual probe helper reads
/// (`manual_probe.py:ManualProbeHelper.__init__`).
fn probe_calibrate_params() -> Vec<&'static str> {
    let mut params = PROBE_PARAMS.to_vec();
    params.push("SPEED");
    params
}

impl ProbeParams {
    /// The section's defaults: `lift_speed` falls back to `speed`, as upstream
    /// does (`probe.py:250`).
    pub(crate) fn from_options(options: &ProbeOptions) -> Self {
        Self {
            probe_speed: options.speed,
            lift_speed: options.lift_speed.unwrap_or(options.speed),
            samples: options.samples,
            sample_retract_dist: options.sample_retract_dist,
            samples_tolerance: options.samples_tolerance,
            samples_tolerance_retries: options.samples_tolerance_retries,
            samples_result: options.samples_result.clone(),
        }
    }

    /// The parameters a command asks for: its own parameters override the
    /// section's (`probe.py:291-311`).
    ///
    /// # Errors
    /// When a parameter is present but not a number, or is out of the range
    /// upstream enforces (`above=0.` / `minval=1` / `minval=0`).
    pub(crate) fn from_command(&self, gcmd: &GcodeCommand) -> Result<Self, CommandError> {
        let probe_speed = gcmd.get_float_default("PROBE_SPEED", self.probe_speed)?;
        let lift_speed = gcmd.get_float_default("LIFT_SPEED", self.lift_speed)?;
        let samples = gcmd.get_int_default("SAMPLES", self.samples)?;
        let sample_retract_dist =
            gcmd.get_float_default("SAMPLE_RETRACT_DIST", self.sample_retract_dist)?;
        let samples_tolerance =
            gcmd.get_float_default("SAMPLES_TOLERANCE", self.samples_tolerance)?;
        let samples_tolerance_retries =
            gcmd.get_int_default("SAMPLES_TOLERANCE_RETRIES", self.samples_tolerance_retries)?;
        let samples_result = gcmd.get_str_default("SAMPLES_RESULT", &self.samples_result);

        if probe_speed <= 0.0 {
            return Err(CommandError::new("Option 'PROBE_SPEED' must be above 0.0"));
        }
        if lift_speed <= 0.0 {
            return Err(CommandError::new("Option 'LIFT_SPEED' must be above 0.0"));
        }
        if samples < 1 {
            return Err(CommandError::new("Option 'SAMPLES' must have minimum of 1"));
        }
        if sample_retract_dist <= 0.0 {
            return Err(CommandError::new(
                "Option 'SAMPLE_RETRACT_DIST' must be above 0.0",
            ));
        }
        if samples_tolerance < 0.0 {
            return Err(CommandError::new(
                "Option 'SAMPLES_TOLERANCE' must have minimum of 0",
            ));
        }
        if samples_tolerance_retries < 0 {
            return Err(CommandError::new(
                "Option 'SAMPLES_TOLERANCE_RETRIES' must have minimum of 0",
            ));
        }

        Ok(Self {
            probe_speed,
            lift_speed,
            samples,
            sample_retract_dist,
            samples_tolerance,
            samples_tolerance_retries,
            samples_result,
        })
    }
}

/// The chip behind `endstop_pin: probe:…`.
///
/// Shared with `[bltouch]`, which registers the same virtual name for its
/// sensor endstop (`bltouch.py:HomingViaProbeHelper`).
pub(crate) struct ProbeChip {
    /// The physical probe endstop the virtual name resolves to.
    pub(crate) endstop: Arc<McuEndstop>,
    /// The probe's trigger offset: what a `probe:z_virtual_endstop` rail uses
    /// as its `position_endstop` (`ProbeEndstopWrapper.get_position_endstop`,
    /// `probe.py:235-236`).
    pub(crate) z_offset: f64,
}

impl PinChip for ProbeChip {
    fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
        Err(PinError::Unsupported("digital_out".to_string()))
    }

    fn setup_endstop(&self, params: &PinParams) -> Result<Arc<McuEndstop>, PinError> {
        check_virtual_endstop(params)?;
        Ok(Arc::clone(&self.endstop))
    }

    fn virtual_endstop_position(&self, params: &PinParams) -> Option<f64> {
        check_virtual_endstop(params).ok()?;
        Some(self.z_offset)
    }
}

/// Upstream's two refusals for the virtual endstop (`probe.py:223-229`).
///
/// Split out so the checks are testable without an MCU.
pub(crate) fn check_virtual_endstop(params: &PinParams) -> Result<(), PinError> {
    if params.pin != VIRTUAL_ENDSTOP {
        return Err(PinError::Message(
            "Probe virtual endstop only useful as endstop pin".to_string(),
        ));
    }
    if params.invert || params.pullup != 0 {
        return Err(PinError::Message(
            "Can not pullup/invert probe virtual endstop".to_string(),
        ));
    }
    Ok(())
}

/// Upstream's `calc_probe_z_average` (`probe.py:17-32`).
///
/// `average` averages every axis; `median` sorts by Z and takes the middle
/// sample (the mean of the two middle ones for an even count).
pub(crate) fn calc_probe_z_average(positions: &[Coord], method: &str) -> Coord {
    if method != "median" {
        let count = positions.len() as f64;
        let mut out = Coord::new(0.0, 0.0, 0.0, 0.0);
        for axis in 0..3 {
            let sum: f64 = positions.iter().map(|pos| pos.axis(axis)).sum();
            out.set_axis(axis, sum / count);
        }
        return out;
    }
    let mut sorted = positions.to_vec();
    sorted.sort_by(|a, b| a.z().total_cmp(&b.z()));
    let middle = sorted.len() / 2;
    if sorted.len() % 2 == 1 {
        sorted[middle]
    } else {
        calc_probe_z_average(&sorted[middle - 1..middle + 1], "average")
    }
}

/// The state `PROBE`/`QUERY_PROBE` report (`ProbeCommandHelper.get_status`).
///
/// Shared with `[bltouch]`, whose section registers both the `bltouch` and the
/// `probe` object and reports this state from either (`bltouch.py:load_config`).
#[derive(Debug, Default)]
pub(crate) struct ProbeCommandState {
    /// The last `QUERY_PROBE` result.
    pub(crate) last_query: AtomicBool,
    /// The last `PROBE` result.
    pub(crate) last_z_result: Mutex<f64>,
}

/// What `get_status` reports for either probe section
/// (`ProbeCommandHelper.get_status`), keyed by the section's identifier.
pub(crate) fn command_status(name: &str, state: &ProbeCommandState) -> Value {
    json!({
        "name": name,
        "last_query": state.last_query.load(Ordering::SeqCst),
        "last_z_result": *state
            .last_z_result
            .lock()
            .unwrap_or_else(|p| p.into_inner()),
    })
}

/// The callbacks a probe with its own hardware runs around one probing move
/// (`bltouch.py:BLTouchProbe.start_probe_session`, `_probe_prepare`,
/// `_probe_finish`, `end_probe_session`). `[probe]` passes no hooks.
pub(crate) struct ProbeHooks {
    /// A session opened (`BLTouchProbe.start_probe_session`).
    pub(crate) start: Arc<dyn Fn() -> Result<(), CommandError> + Send + Sync>,
    /// A session closed (`BLTouchProbe.end_probe_session`).
    pub(crate) end: Arc<dyn Fn() -> Result<(), CommandError> + Send + Sync>,
    /// Before the probing move: lower the probe (`_probe_prepare`).
    pub(crate) prepare: Arc<dyn Fn() -> CommandFuture<'static> + Send + Sync>,
    /// After it, success **or** failure (`_probe_finish`).
    pub(crate) finish: Arc<dyn Fn() -> CommandFuture<'static> + Send + Sync>,
}

/// Tracks a series of probe attempts within one command
/// (`probe.py:ProbeSessionHelper`).
pub(crate) struct ProbeSessionHelper {
    /// The machine, for the toolhead and the results event.
    printer: Weak<Printer>,
    /// What every probing move drives: the physical endstop for `[probe]`, the
    /// BLTouch wrapper (which raises the pin once the sensor trips) for
    /// `[bltouch]` (`BLTouchProbe.home_start`).
    endstop: Arc<dyn HomingEndstop>,
    /// The endstop `QUERY_PROBE` reads. Upstream asks the probe's own
    /// `query_endstop`, which for a BLTouch is the bare sensor endstop
    /// (`bltouch.py:BLTouchProbe.query_endstop`).
    query_endstop: Arc<McuEndstop>,
    /// The Z to move down to while probing: `[stepper_z] position_min`, or
    /// `[printer] minimum_z_position` when there is no Z stepper
    /// (`probe.py:188-193`).
    z_position: f64,
    /// The section's parameters; a command may override them.
    defaults: ProbeParams,
    /// Whether a session is open.
    pending: AtomicBool,
    /// The sample sets run in this session.
    results: Mutex<Vec<Coord>>,
    /// The hardware's session callbacks, when the probe has its own (`[bltouch]`).
    hooks: Option<ProbeHooks>,
}

impl ProbeSessionHelper {
    /// Read the session's defaults and the Z position to probe to.
    ///
    /// # Errors
    /// When `[stepper_z] position_min` / `[printer] minimum_z_position` is
    /// present but not a number.
    pub(crate) fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        endstop: Arc<dyn HomingEndstop>,
        query_endstop: Arc<McuEndstop>,
        options: &ProbeOptions,
        hooks: Option<ProbeHooks>,
    ) -> Result<Self, ConfigError> {
        // Upstream reads this with `note_valid=False`, so the option stays
        // "unused" for the undefined-option check; reading it through the
        // sibling records it instead, which is harmless (it *is* read).
        let z_position = match config.sibling("stepper_z") {
            Some(sibling) => sibling.get_float("position_min", Some(0.0))?,
            None => match config.sibling("printer") {
                Some(sibling) => sibling.get_float("minimum_z_position", Some(0.0))?,
                None => 0.0,
            },
        };
        Ok(Self {
            printer: Arc::downgrade(printer),
            endstop,
            query_endstop,
            z_position,
            defaults: ProbeParams::from_options(options),
            pending: AtomicBool::new(false),
            results: Mutex::new(Vec::new()),
            hooks,
        })
    }

    /// The toolhead, or "not ready".
    fn toolhead(&self) -> Result<Arc<ToolHeadObject>, CommandError> {
        self.printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT))
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    /// Upstream's `_probe_state_error`.
    fn state_error() -> CommandError {
        CommandError::new("Internal probe error - start/end probe session mismatch")
    }

    /// Open a session (`probe.py:start_probe_session`).
    ///
    /// # Errors
    /// When a session is already open.
    fn start(&self) -> Result<(), CommandError> {
        if self.pending.swap(true, Ordering::SeqCst) {
            return Err(Self::state_error());
        }
        self.results
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clear();
        if let Some(hooks) = &self.hooks {
            (hooks.start)()?;
        }
        Ok(())
    }

    /// Close a session (`probe.py:end_probe_session`).
    ///
    /// # Errors
    /// When no session is open, or the hardware's own close step fails
    /// (`BLTouchProbe.end_probe_session`).
    fn end(&self) -> Result<(), CommandError> {
        if !self.pending.swap(false, Ordering::SeqCst) {
            return Err(Self::state_error());
        }
        self.results
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clear();
        if let Some(hooks) = &self.hooks {
            (hooks.end)()?;
        }
        Ok(())
    }

    /// Whether a session is open.
    fn is_pending(&self) -> bool {
        self.pending.load(Ordering::SeqCst)
    }

    /// One probing move down to `z_position` (`probe.py:_probe`).
    ///
    /// # Errors
    /// "Must home before probe" when Z is not homed, and whatever the probing
    /// move reports (including "no trigger").
    async fn probe_once(&self, gcmd: &GcodeCommand, speed: f64) -> Result<Coord, CommandError> {
        let toolhead = self.toolhead()?;
        let homed = toolhead.get_status(0.0)["homed_axes"]
            .as_str()
            .unwrap_or("")
            .to_string();
        if !homed.contains('z') {
            return Err(CommandError::new("Must home before probe"));
        }
        let mut target = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        target.set_axis(Z_AXIS, self.z_position);

        // The probe's own prepare step runs before the move
        // (`BLTouchProbe.run_probe` lowers the pin first).
        if let Some(hooks) = &self.hooks {
            (hooks.prepare)().await?;
        }
        let moved = toolhead.probing_move(&*self.endstop, target, speed).await;
        // …and its finish step runs whether or not the move got its trigger
        // (`BLTouchProbe.run_probe`'s `except: _probe_finish(); raise`).
        let epos = match moved {
            Ok(epos) => {
                if let Some(hooks) = &self.hooks {
                    (hooks.finish)().await?;
                }
                epos
            }
            Err(err) => {
                if let Some(hooks) = &self.hooks {
                    (hooks.finish)().await?;
                }
                return Err(err);
            }
        };
        // A consumer (`axis_twist_compensation`) edits the result in place; the
        // reported value is what the handlers left (`probe.py:364-367`, where
        // `_probe` reads `results[0]` back after `send_event`).
        let results = ProbeResultsHandle::new(vec![epos]);
        if let Some(printer) = self.printer.upgrade() {
            printer.send_event(&KlippyEvent::ProbeUpdateResults {
                results: results.clone(),
            });
        }
        let epos = results.to_vec().first().copied().unwrap_or(epos);
        gcmd.respond_info(&format!(
            "probe at {:.3},{:.3} is z={:.6}",
            epos.x(),
            epos.y(),
            epos.z()
        ));
        Ok(epos)
    }

    /// Run one sample set with tolerance retries (`probe.py:336-378`).
    ///
    /// # Errors
    /// When no session is open, a parameter is bad, or the samples keep
    /// spreading beyond `samples_tolerance` after the allowed retries.
    async fn run_with(
        &self,
        gcmd: &GcodeCommand,
        params: &ProbeParams,
    ) -> Result<(), CommandError> {
        if !self.is_pending() {
            return Err(Self::state_error());
        }
        let toolhead = self.toolhead()?;
        let probexy = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let mut retries = 0i64;
        let mut positions: Vec<Coord> = Vec::new();
        while (positions.len() as i64) < params.samples {
            let pos = self.probe_once(gcmd, params.probe_speed).await?;
            positions.push(pos);
            let spread = {
                let max = positions
                    .iter()
                    .map(Coord::z)
                    .fold(f64::NEG_INFINITY, f64::max);
                let min = positions.iter().map(Coord::z).fold(f64::INFINITY, f64::min);
                max - min
            };
            if spread > params.samples_tolerance {
                if retries >= params.samples_tolerance_retries {
                    return Err(CommandError::new("Probe samples exceed samples_tolerance"));
                }
                gcmd.respond_info("Probe samples exceed tolerance. Retrying...");
                retries += 1;
                positions.clear();
            }
            if (positions.len() as i64) < params.samples {
                let mut lift = probexy;
                lift.set_axis(Z_AXIS, pos.z() + params.sample_retract_dist);
                toolhead.move_to(lift, params.lift_speed)?;
            }
        }
        let epos = calc_probe_z_average(&positions, &params.samples_result);
        self.results
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push(epos);
        Ok(())
    }

    /// `run_probe`: a sample set with the command's parameters.
    ///
    /// # Errors
    /// As [`ProbeSessionHelper::run_with`].
    async fn run(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let params = self.defaults.from_command(gcmd)?;
        self.run_with(gcmd, &params).await
    }

    /// Take the session's completed sample sets (`pull_probed_results`).
    fn pull_results(&self) -> Vec<Coord> {
        std::mem::take(&mut *self.results.lock().unwrap_or_else(|p| p.into_inner()))
    }
}

/// One configured `[probe]` (`probe.py:PrinterProbe`).
pub struct PrinterProbe {
    /// The section's identifier, for logging and `Debug`.
    identifier: String,
    /// The options as read.
    options: ProbeOptions,
    /// The physical probe endstop, also reachable as `probe:z_virtual_endstop`.
    endstop: Arc<McuEndstop>,
    /// The session `PROBE`/`PROBE_ACCURACY` drive.
    session: Arc<ProbeSessionHelper>,
    /// What `get_status` reports.
    state: Arc<ProbeCommandState>,
}

impl PrinterProbe {
    /// Build the physical endstop, register the `probe` chip and the commands.
    ///
    /// # Errors
    /// Returns a config error when an option is missing or malformed, when the
    /// probe pin cannot be built, or when the `probe` chip or a command is
    /// already taken.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let options = ProbeOptions::read(config)?;

        if options.activate_gcode.is_some() || options.deactivate_gcode.is_some() {
            warn!(
                "[{identifier}]: activate_gcode/deactivate_gcode need [gcode_macro], \
                 which is not implemented yet; the templates are recorded but not rendered"
            );
        }

        let pins = printer
            .lookup_object_as::<PrinterPins>(PINS_OBJECT)
            .expect("the loader registers `pins` before any section");
        let endstop = pins
            .setup_endstop(&options.pin, None)
            .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        // Upstream registers the chip while the section loads
        // (`probe.py:HomingViaProbeHelper.__init__`), which is what makes
        // `endstop_pin: probe:z_virtual_endstop` resolvable for the rails.
        pins.register_chip(
            CHIP_NAME,
            Arc::new(ProbeChip {
                endstop: Arc::clone(&endstop),
                z_offset: options.z_offset,
            }),
        )
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        let session = Arc::new(ProbeSessionHelper::new(
            config,
            printer,
            Arc::clone(&endstop) as Arc<dyn HomingEndstop>,
            Arc::clone(&endstop),
            &options,
            None,
        )?);
        let state = Arc::new(ProbeCommandState::default());
        let offsets = ProbeOffsets {
            x: options.x_offset,
            y: options.y_offset,
            z: options.z_offset,
        };
        register_commands(printer, &identifier, &session, &state, offsets)?;

        Ok(Self {
            identifier,
            options,
            endstop,
            session,
            state,
        })
    }

    /// Assemble the section from parts a sibling probe section already built
    /// (`[bltouch]`, which owns the hardware but reports through this type,
    /// `bltouch.py:PrinterBLTouch`).
    pub(crate) fn from_parts(
        identifier: String,
        options: ProbeOptions,
        endstop: Arc<McuEndstop>,
        session: Arc<ProbeSessionHelper>,
        state: Arc<ProbeCommandState>,
    ) -> Self {
        Self {
            identifier,
            options,
            endstop,
            session,
            state,
        }
    }

    /// The section identifier.
    pub fn identifier(&self) -> &str {
        &self.identifier
    }

    /// The options as read.
    pub fn options(&self) -> &ProbeOptions {
        &self.options
    }

    /// The physical probe endstop.
    pub fn endstop(&self) -> &Arc<McuEndstop> {
        &self.endstop
    }

    /// The probe offsets (`get_offsets`).
    pub fn offsets(&self) -> ProbeOffsets {
        ProbeOffsets {
            x: self.options.x_offset,
            y: self.options.y_offset,
            z: self.options.z_offset,
        }
    }
}

/// What every consumer of the `probe` object drives: the session surface of
/// one configured probe section (`probe.py:PrinterProbe`'s
/// `start_probe_session` / `run_probe` / `pull_probed_results` /
/// `end_probe_session` / `get_probe_params` / `get_offsets`).
///
/// The points round dispatches through this trait ([`lookup_probe_session`]),
/// so a second probe section registering the same `probe` object plugs into
/// the same round. `PrinterProbe` — `[probe]`, and through it `[bltouch]` /
/// `[smart_effector]` — is the first implementation; the round's tests drive
/// a second one.
pub trait ProbeSession: Send + Sync {
    /// Open a session (`start_probe_session`). The command that opens it
    /// comes along so a probe whose methods differ per command can dispatch
    /// on `METHOD` (upstream passes `gcmd` here too).
    fn start_probe_session(&self, gcmd: &GcodeCommand) -> Result<(), CommandError>;
    /// Run one sample set in the open session (`run_probe`).
    fn run_probe<'a>(&'a self, gcmd: &'a GcodeCommand) -> CommandFuture<'a>;
    /// The parameters a command asks for (`ProbeSessionHelper.get_probe_params`).
    fn probe_params(&self, gcmd: &GcodeCommand) -> Result<ProbeParams, CommandError>;
    /// Take the completed sample sets (`pull_probed_results`).
    fn pull_probed_results(&self) -> Vec<Coord>;
    /// Close the session (`end_probe_session`).
    fn end_probe_session(&self) -> Result<(), CommandError>;
    /// The probe's offsets (`get_offsets`).
    fn offsets(&self) -> ProbeOffsets;
}

impl ProbeSession for PrinterProbe {
    fn start_probe_session(&self, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
        self.session.start()
    }

    fn run_probe<'a>(&'a self, gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
        Box::pin(self.session.run(gcmd))
    }

    fn probe_params(&self, gcmd: &GcodeCommand) -> Result<ProbeParams, CommandError> {
        self.session.defaults.from_command(gcmd)
    }

    fn pull_probed_results(&self) -> Vec<Coord> {
        self.session.pull_results()
    }

    fn end_probe_session(&self) -> Result<(), CommandError> {
        self.session.end()
    }

    fn offsets(&self) -> ProbeOffsets {
        // The same answer as the inherent accessor, which stays for the
        // callers outside this module (bed_mesh drives it concretely today).
        ProbeOffsets {
            x: self.options.x_offset,
            y: self.options.y_offset,
            z: self.options.z_offset,
        }
    }
}

/// How sensor samples reach an open probe session — the delivery seam this
/// port fixes ahead of the eddy probe. Upstream, the sensor's client stream
/// feeds the probing session while it runs (`probe_eddy_current.py`'s
/// gather/scan loops read batches and turn them into results); the real
/// ldc1612 producer lands with that probe, and its tests stub it here — a
/// producer holds an `Arc<dyn SampleDelivery>` pointing into the open
/// session.
pub trait SampleDelivery: Send + Sync {
    /// Deliver one sample read at `time` (print seconds) whose sensor value
    /// is `value`.
    fn deliver_sample(&self, time: f64, value: f64);
}

/// The registered `probe` object as its session trait: the lookup every
/// points round starts from (`probe.py:start_probe_session`'s object lookup
/// by name).
///
/// The object is built by the probe family — `[probe]`, `[bltouch]`,
/// `[smart_effector]` (all `PrinterProbe`), and the eddy probe
/// (`probe_eddy_current::PrinterEddyProbe`), which registers the same `probe`
/// name and answers this downcast in turn.
pub(crate) fn lookup_probe_session(printer: &Printer) -> Option<Arc<dyn ProbeSession>> {
    if let Some(probe) = printer.lookup_object_as::<PrinterProbe>(PROBE_OBJECT) {
        return Some(probe as Arc<dyn ProbeSession>);
    }
    if let Some(probe) =
        printer.lookup_object_as::<probe_eddy_current::PrinterEddyProbe>(PROBE_OBJECT)
    {
        return Some(probe as Arc<dyn ProbeSession>);
    }
    // `[load_cell_probe]` registers the same `probe` object
    // (`LoadCellPrinterProbe`).
    printer
        .lookup_object_as::<crate::core::klippy::extras::load_cell_probe::LoadCellProbe>(
            PROBE_OBJECT,
        )
        .map(|probe| probe as Arc<dyn ProbeSession>)
}

/// Register `QUERY_PROBE`, `PROBE` and `PROBE_ACCURACY`
/// (`probe.py:ProbeCommandHelper`), plus the session cleanup on a command
/// error.
pub(crate) fn register_commands(
    printer: &Arc<Printer>,
    identifier: &str,
    session: &Arc<ProbeSessionHelper>,
    state: &Arc<ProbeCommandState>,
    offsets: ProbeOffsets,
) -> Result<(), ConfigError> {
    let gcode = printer
        .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
        .expect("the loader registers `gcode` first");
    let name = identifier.to_string();

    // A failing command must not leave the session open
    // (`probe.py:_handle_command_error`).
    {
        let session = Arc::clone(session);
        printer.register_event_handler(
            KlippyEvent::GcodeCommandError,
            Box::new(move |_event| {
                if session.is_pending() {
                    if let Err(err) = session.end() {
                        warn!("Multi-probe end failed: {err}");
                    }
                }
            }),
        );
    }

    // QUERY_PROBE: the probe pin's level now.
    {
        let session = Arc::clone(session);
        let state = Arc::clone(state);
        gcode
            .register_command(
                "QUERY_PROBE",
                Arc::new(move |gcmd| {
                    let session = Arc::clone(&session);
                    let state = Arc::clone(&state);
                    Box::pin(async move {
                        let toolhead = session.toolhead()?;
                        let print_time = toolhead.print_time();
                        let triggered = session
                            .query_endstop
                            .query_endstop(print_time)
                            .await
                            .map_err(|err| CommandError::new(err.to_string()))?;
                        state.last_query.store(triggered, Ordering::SeqCst);
                        gcmd.respond_info(&format!(
                            "probe: {}",
                            if triggered { "TRIGGERED" } else { "open" }
                        ));
                        Ok(())
                    })
                }),
                Some("Return the status of the z-probe"),
                false,
            )
            .map_err(ConfigError::new)?;
    }

    // PROBE: one sample set at the current XY position.
    {
        let session = Arc::clone(session);
        let state = Arc::clone(state);
        gcode
            .register_command_with_params(
                "PROBE",
                Arc::new(move |gcmd| {
                    let session = Arc::clone(&session);
                    let state = Arc::clone(&state);
                    Box::pin(async move {
                        session.start()?;
                        session.run(gcmd).await?;
                        let pos = session
                            .pull_results()
                            .into_iter()
                            .next()
                            .ok_or_else(ProbeSessionHelper::state_error)?;
                        session.end()?;
                        gcmd.respond_info(&format!("Result is z={:.6}", pos.z()));
                        *state
                            .last_z_result
                            .lock()
                            .unwrap_or_else(|p| p.into_inner()) = pos.z();
                        Ok(())
                    })
                }),
                Some("Probe Z-height at current XY position"),
                PROBE_PARAMS,
                false,
            )
            .map_err(ConfigError::new)?;
    }

    // PROBE_ACCURACY: `SAMPLES` single-sample probes, then the spread
    // (`probe.py:cmd_PROBE_ACCURACY`).
    {
        let session = Arc::clone(session);
        gcode
            .register_command_with_params(
                "PROBE_ACCURACY",
                Arc::new(move |gcmd| {
                    let session = Arc::clone(&session);
                    Box::pin(async move {
                        let params = session.defaults.from_command(gcmd)?;
                        let sample_count = gcmd.get_int_default("SAMPLES", 10)?;
                        if sample_count < 1 {
                            return Err(CommandError::new(
                                "Option 'SAMPLES' must have minimum of 1",
                            ));
                        }
                        let toolhead = session.toolhead()?;
                        let pos = toolhead
                            .position()
                            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                        gcmd.respond_info(&format!(
                            "PROBE_ACCURACY at X:{:.3} Y:{:.3} Z:{:.3} (samples={} retract={:.3} speed={:.1} lift_speed={:.1})",
                            pos.x(), pos.y(), pos.z(), sample_count,
                            params.sample_retract_dist, params.probe_speed, params.lift_speed,
                        ));
                        // The accuracy loop probes one sample at a time, with a
                        // lift between them (`fo_params['SAMPLES'] = '1'`).
                        let single = ProbeParams {
                            samples: 1,
                            ..params.clone()
                        };
                        session.start()?;
                        for _ in 0..sample_count {
                            session.run_with(gcmd, &single).await?;
                            let pos = toolhead
                                .position()
                                .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                            let mut lift = pos;
                            lift.set_axis(Z_AXIS, pos.z() + params.sample_retract_dist);
                            toolhead.move_to(lift, params.lift_speed)?;
                        }
                        let positions = session.pull_results();
                        session.end()?;

                        let max_value = positions
                            .iter()
                            .map(Coord::z)
                            .fold(f64::NEG_INFINITY, f64::max);
                        let min_value = positions.iter().map(Coord::z).fold(f64::INFINITY, f64::min);
                        let range_value = max_value - min_value;
                        let avg_value = calc_probe_z_average(&positions, "average").z();
                        let median = calc_probe_z_average(&positions, "median").z();
                        let deviation_sum: f64 = positions
                            .iter()
                            .map(|p| (p.z() - avg_value).powi(2))
                            .sum();
                        let sigma = (deviation_sum / positions.len() as f64).sqrt();
                        gcmd.respond_info(&format!(
                            "probe accuracy results: maximum {max_value:.6}, minimum {min_value:.6}, range {range_value:.6}, average {avg_value:.6}, median {median:.6}, standard deviation {sigma:.6}"
                        ));
                        Ok(())
                    })
                }),
                Some("Probe Z-height accuracy at current XY position"),
                PROBE_PARAMS,
                false,
            )
            .map_err(ConfigError::new)?;
    }

    // PROBE_CALIBRATE: probe once, move the nozzle over the probe, then hand
    // over to the interactive manual probe (`probe.py:cmd_PROBE_CALIBRATE`).
    {
        let session = Arc::clone(session);
        let name = name.clone();
        let calibrate_z = Arc::new(Mutex::new(0.0f64));
        let printer_weak = Arc::downgrade(printer);
        gcode
            .register_command_with_params(
                "PROBE_CALIBRATE",
                Arc::new(move |gcmd| {
                    let session = Arc::clone(&session);
                    let name = name.clone();
                    let calibrate_z = Arc::clone(&calibrate_z);
                    let printer_weak = printer_weak.clone();
                    Box::pin(async move {
                        let printer = printer_weak
                            .upgrade()
                            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                        let manual_probe = printer
                            .lookup_object_as::<ManualProbe>(MANUAL_PROBE_OBJECT)
                            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                        manual_probe.verify_no_manual_probe(&printer)?;

                        // Initial probe at the current position.
                        let params = session.defaults.from_command(gcmd)?;
                        session.start()?;
                        session.run(gcmd).await?;
                        let pos = session
                            .pull_results()
                            .into_iter()
                            .next()
                            .ok_or_else(ProbeSessionHelper::state_error)?;
                        session.end()?;

                        // Move away from the bed, then over the probe point.
                        let toolhead = session.toolhead()?;
                        let mut curpos = pos;
                        *calibrate_z.lock().unwrap_or_else(|p| p.into_inner()) = curpos.z();
                        curpos.set_axis(Z_AXIS, curpos.z() + 5.0);
                        toolhead.move_to(curpos, params.lift_speed)?;
                        curpos.set_axis(0, curpos.x() + offsets.x);
                        curpos.set_axis(1, curpos.y() + offsets.y);
                        toolhead.move_to(curpos, params.probe_speed)?;

                        // Interactive part; ACCEPT reports the new z_offset.
                        let cb_printer = Arc::clone(&printer);
                        let callback: FinalizeCallback =
                            Arc::new(move |kin_pos: Option<Coord>| {
                                let Some(kin) = kin_pos else { return };
                                let z_offset = *calibrate_z.lock().unwrap_or_else(|p| p.into_inner())
                                    - kin.z();
                                if let Some(gcode) =
                                    cb_printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                                {
                                    gcode.respond_info(
                                        &format!(
                                            "{name}: z_offset: {z_offset:.3}\n\
                                             The SAVE_CONFIG command will update the printer config file\n\
                                             with the above and restart the printer."
                                        ),
                                        true,
                                    );
                                }
                                if let Some(configfile) =
                                    cb_printer.lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
                                {
                                    configfile.set(&name, "z_offset", &format!("{z_offset:.3}"));
                                }
                            });
                        manual_probe.start_helper(&printer, gcmd, callback)?;
                        Ok(())
                    })
                }),
                Some("Calibrate the probe's z_offset"),
                &probe_calibrate_params(),
                false,
            )
            .map_err(ConfigError::new)?;
    }

    Ok(())
}

impl PrinterObject for PrinterProbe {
    fn get_status(&self, _eventtime: f64) -> Value {
        command_status(&self.identifier, &self.state)
    }
}

impl std::fmt::Debug for PrinterProbe {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterProbe")
            .field("identifier", &self.identifier)
            .field("pin", &self.options.pin)
            .field("z_offset", &self.options.z_offset)
            .finish()
    }
}

/// Upstream's `load_config` for `[probe]`.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PrinterProbe::new(config, printer)?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{ConfigSection, ConfigValue};

    /// A section with the given options, as the parser would build it.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("probe", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    fn params(pin: &str, invert: bool, pullup: i8) -> PinParams {
        PinParams {
            chip_name: CHIP_NAME.to_string(),
            pin: pin.to_string(),
            invert,
            pullup,
            share_type: None,
        }
    }

    #[test]
    fn options_carry_upstream_defaults() {
        let section = section(&[("pin", "PA0"), ("z_offset", "1.5")]);
        let options = ProbeOptions::read(&ConfigWrapper::untracked(&section)).unwrap();

        assert_eq!(options.pin, "PA0");
        assert_eq!(options.z_offset, 1.5);
        assert_eq!(options.x_offset, 0.0);
        assert_eq!(options.y_offset, 0.0);
        assert_eq!(options.speed, 5.0);
        assert_eq!(options.lift_speed, None);
        assert_eq!(options.samples, 1);
        assert_eq!(options.sample_retract_dist, 2.0);
        assert_eq!(options.samples_result, "median");
        assert_eq!(options.samples_tolerance, 0.100);
        assert_eq!(options.samples_tolerance_retries, 0);
        assert!(options.deactivate_on_each_sample);
        assert_eq!(options.activate_gcode, None);
        assert_eq!(options.deactivate_gcode, None);
    }

    #[test]
    fn every_option_the_corpus_writes_is_claimed() {
        // The option set the upstream corpus exercises (33 `[probe]` sections),
        // so `check_unused` passes on all of them.
        let section = section(&[
            ("pin", "^PA0"),
            ("z_offset", "2.0"),
            ("x_offset", "20.0"),
            ("y_offset", "5.0"),
            ("speed", "2.0"),
            ("lift_speed", "10.0"),
            ("samples", "3"),
            ("sample_retract_dist", "4.0"),
            ("samples_result", "average"),
            ("samples_tolerance", "0.05"),
            ("samples_tolerance_retries", "5"),
            ("deactivate_on_each_sample", "false"),
            ("activate_gcode", "probe_reset"),
            ("deactivate_gcode", "probe_reset"),
        ]);
        let options = ProbeOptions::read(&ConfigWrapper::untracked(&section)).unwrap();

        assert_eq!(options.pin, "^PA0");
        assert_eq!(options.samples, 3);
        assert_eq!(options.samples_result, "average");
        assert_eq!(options.samples_tolerance_retries, 5);
        assert!(!options.deactivate_on_each_sample);
        assert_eq!(options.activate_gcode.as_deref(), Some("probe_reset"));
        assert_eq!(options.deactivate_gcode.as_deref(), Some("probe_reset"));
    }

    #[test]
    fn a_bad_samples_result_is_refused() {
        let section = section(&[
            ("pin", "PA0"),
            ("z_offset", "1.0"),
            ("samples_result", "mode"),
        ]);
        let err = ProbeOptions::read(&ConfigWrapper::untracked(&section)).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Choice 'mode' for option 'samples_result' in section 'probe' is not a valid choice"
        );
    }

    #[test]
    fn the_virtual_endstop_pin_name_is_accepted() {
        check_virtual_endstop(&params(VIRTUAL_ENDSTOP, false, 0)).unwrap();
    }

    #[test]
    fn another_pin_name_is_refused_like_upstream() {
        let err = check_virtual_endstop(&params("z", false, 0)).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Probe virtual endstop only useful as endstop pin"
        );
    }

    #[test]
    fn inverting_or_pulling_up_the_virtual_endstop_is_refused() {
        let inverted = check_virtual_endstop(&params(VIRTUAL_ENDSTOP, true, 0)).unwrap_err();
        assert_eq!(
            inverted.to_string(),
            "Can not pullup/invert probe virtual endstop"
        );

        let pulled_up = check_virtual_endstop(&params(VIRTUAL_ENDSTOP, false, 1)).unwrap_err();
        assert_eq!(
            pulled_up.to_string(),
            "Can not pullup/invert probe virtual endstop"
        );
    }

    #[test]
    fn the_lift_speed_defaults_to_the_probe_speed() {
        let section = section(&[("pin", "PA0"), ("z_offset", "1.0"), ("speed", "3.0")]);
        let options = ProbeOptions::read(&ConfigWrapper::untracked(&section)).unwrap();
        let params = ProbeParams::from_options(&options);

        assert_eq!(params.probe_speed, 3.0);
        assert_eq!(params.lift_speed, 3.0);
    }

    #[test]
    fn averaging_takes_every_axis_and_median_takes_the_middle_z() {
        let positions = [
            Coord::new(0.0, 0.0, 1.0, 0.0),
            Coord::new(2.0, 4.0, 3.0, 0.0),
            Coord::new(4.0, 8.0, 5.0, 0.0),
        ];

        let average = calc_probe_z_average(&positions, "average");
        assert_eq!(average.x(), 2.0);
        assert_eq!(average.y(), 4.0);
        assert_eq!(average.z(), 3.0);

        let median = calc_probe_z_average(&positions, "median");
        assert_eq!(median.z(), 3.0);
    }

    #[test]
    fn an_even_sample_count_takes_the_mean_of_the_two_middle_z() {
        let positions = [
            Coord::new(0.0, 0.0, 1.0, 0.0),
            Coord::new(0.0, 0.0, 3.0, 0.0),
            Coord::new(0.0, 0.0, 5.0, 0.0),
            Coord::new(0.0, 0.0, 7.0, 0.0),
        ];
        let median = calc_probe_z_average(&positions, "median");

        assert_eq!(median.z(), 4.0);
    }
}

// ===========================================================================
// ProbePointsHelper (upstream klippy/extras/probe.py:425-529)
// ===========================================================================

/// The object the `probe` session is looked up under (the `[probe]` section
/// registers itself under this name).
const PROBE_OBJECT: &str = "probe";

/// Upstream's `ProbePointsHelper.finalize_callback` result: it receives the
/// probe offsets and the position probed at each point, and answers
/// `Some(`[`RETRY`]`)` to start the whole round over (`res != "retry"`),
/// anything else to end it.
pub type ProbePointsFinalize =
    Arc<dyn Fn(ProbeOffsets, &[Coord]) -> Option<&'static str> + Send + Sync>;

/// The callback answer that restarts a probing round (upstream `"retry"`).
pub const RETRY: &str = "retry";

/// Helper code that can probe a series of points and report the position at
/// each point (`probe.py:ProbePointsHelper`).
pub struct ProbePointsHelper {
    /// The machine, to find `toolhead`/`probe`/`manual_probe` at run time.
    printer: Weak<Printer>,
    /// The consumer section's name, for the `minimum_points` error.
    name: String,
    /// What does the work once every point has a position.
    finalize: ProbePointsFinalize,
    /// The configured `points` rows (`x, y`).
    probe_points: Mutex<Vec<(f64, f64)>>,
    /// `horizontal_move_z` as configured (a command may override it).
    default_horizontal_move_z: f64,
    /// The travel speed between points (`speed`, `above=0.`).
    speed: f64,
    /// Whether moves subtract the probe's XY offsets (`use_xy_offsets`).
    use_offsets: AtomicBool,
    /// The Z retract speed: the probe's `lift_speed` when automatic, `speed`
    /// when manual (`get_lift_speed`).
    lift_speed: Mutex<f64>,
    /// The Z the toolhead travels between points; the command's
    /// `HORIZONTAL_MOVE_Z` wins over the configured value.
    horizontal_move_z: Mutex<f64>,
    /// The probe's offsets for the running round (zeros when manual).
    probe_offsets: Mutex<ProbeOffsets>,
}

/// The `points` option: rows of `x, y` split on newlines, each row on commas
/// (`probe.py:437-439`, `getlists('points', seps=(',', '\n'), parser=float,
/// count=2)` — newlines split first, every row must hold exactly two values).
///
/// # Errors
/// "must have 2 elements" for a malformed row, "Unable to parse" for a value
/// that is not a number.
fn read_points(config: &ConfigWrapper) -> Result<Vec<(f64, f64)>, ConfigError> {
    let groups = config.get_list_of_lists("points", '\n', ',', 2)?;
    let identifier = config.identifier();
    groups
        .into_iter()
        .map(|pair| {
            let mut parsed = pair.iter().map(|item| {
                item.trim().parse::<f64>().map_err(|_| {
                    ConfigError::new(format!(
                        "Unable to parse option 'points' in section '{identifier}'"
                    ))
                })
            });
            let x = parsed.next().expect("get_list_of_lists checked count=2")?;
            let y = parsed.next().expect("get_list_of_lists checked count=2")?;
            Ok((x, y))
        })
        .collect()
}

impl ProbePointsHelper {
    /// Read the consumer section and its `points` (`probe.py:430-447`).
    ///
    /// # Errors
    /// When `points` is missing and no default points were handed in, or an
    /// option is malformed or out of range.
    pub fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        finalize: ProbePointsFinalize,
    ) -> Result<Arc<Self>, ConfigError> {
        Self::with_default_points(config, printer, finalize, None)
    }

    /// [`ProbePointsHelper::new`] with fallback points, as the consumers that
    /// synthesise their grid pass (`bed_mesh` passes `[]` upstream).
    ///
    /// # Errors
    /// As [`ProbePointsHelper::new`].
    pub fn with_default_points(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        finalize: ProbePointsFinalize,
        default_points: Option<Vec<(f64, f64)>>,
    ) -> Result<Arc<Self>, ConfigError> {
        let name = config.identifier();
        // Configured points win; otherwise the caller's defaults apply; with
        // neither, upstream's `getlists` refuses the missing option.
        let probe_points = if config.has("points") {
            read_points(config)?
        } else if let Some(points) = default_points {
            points
        } else {
            return Err(ConfigError::new(format!(
                "Option 'points' in section '{name}' is not defined"
            )));
        };
        let default_horizontal_move_z = config.get_float("horizontal_move_z", Some(5.0))?;
        let speed = config.get_float_bounded("speed", Some(50.0), None, None, Some(0.0), None)?;
        Ok(Arc::new(Self {
            printer: Arc::downgrade(printer),
            name,
            finalize,
            probe_points: Mutex::new(probe_points),
            default_horizontal_move_z,
            speed,
            use_offsets: AtomicBool::new(false),
            lift_speed: Mutex::new(speed),
            horizontal_move_z: Mutex::new(default_horizontal_move_z),
            probe_offsets: Mutex::new(ProbeOffsets {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
        }))
    }

    /// Refuse fewer points than the consumer needs
    /// (`probe.py:minimum_points`).
    ///
    /// # Errors
    /// "Need at least \<n\> probe points for \<section\>".
    pub fn minimum_points(&self, n: usize) -> Result<(), ConfigError> {
        let count = self
            .probe_points
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .len();
        if count < n {
            return Err(ConfigError::new(format!(
                "Need at least {n} probe points for {}",
                self.name
            )));
        }
        Ok(())
    }

    /// Replace the points and re-check the minimum (`probe.py:update_probe_points`).
    ///
    /// # Errors
    /// As [`ProbePointsHelper::minimum_points`].
    pub fn update_probe_points(
        &self,
        points: Vec<(f64, f64)>,
        min_points: usize,
    ) -> Result<(), ConfigError> {
        *self.probe_points.lock().unwrap_or_else(|p| p.into_inner()) = points;
        self.minimum_points(min_points)
    }

    /// Subtract the probe's XY offsets from every move target
    /// (`probe.py:use_xy_offsets`).
    pub fn use_xy_offsets(&self, use_offsets: bool) {
        self.use_offsets.store(use_offsets, Ordering::SeqCst);
    }

    /// The Z retract speed of the next round (`probe.py:get_lift_speed`).
    pub fn get_lift_speed(&self) -> f64 {
        *self.lift_speed.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// The XY to move to for point `probe_num`, net of the probe offsets when
    /// `use_xy_offsets` is on (`probe.py:_move_next`).
    fn move_target(&self, probe_num: usize) -> Result<(f64, f64), CommandError> {
        let point = self
            .probe_points
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(probe_num)
            .copied()
            .ok_or_else(|| {
                CommandError::new(format!(
                    "Internal probe error - no probe point {probe_num} for {}",
                    self.name
                ))
            })?;
        if !self.use_offsets.load(Ordering::SeqCst) {
            return Ok(point);
        }
        let offsets = *self.probe_offsets.lock().unwrap_or_else(|p| p.into_inner());
        Ok((point.0 - offsets.x, point.1 - offsets.y))
    }

    /// Probe every point and hand `(offsets, positions)` to the finalize
    /// callback (`probe.py:start_probe`).
    ///
    /// `METHOD=manual` (or a printer with no `probe` object) drives the
    /// interactive manual-probe helper point by point instead.
    ///
    /// # Errors
    /// "Already in a manual Z probe…" when one is running, a bad command
    /// parameter, or whatever the toolhead/probe report mid-round.
    pub async fn start_probe(self: &Arc<Self>, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let manual = printer
            .lookup_object_as::<ManualProbe>(MANUAL_PROBE_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        manual.verify_no_manual_probe(&printer)?;

        let method = gcmd.get_str_default("METHOD", "automatic").to_lowercase();
        let def_move_z = self.default_horizontal_move_z;
        let horizontal_move_z = gcmd.get_float_default("HORIZONTAL_MOVE_Z", def_move_z)?;
        *self
            .horizontal_move_z
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = horizontal_move_z;

        let probe = lookup_probe_session(&printer);
        if method == "manual" || probe.is_none() {
            // Manual probing: no offsets, the travel speed is the lift speed,
            // and each point waits for the user's `ACCEPT`
            // (`probe.py:start_probe`'s manual branch).
            *self.lift_speed.lock().unwrap_or_else(|p| p.into_inner()) = self.speed;
            *self.probe_offsets.lock().unwrap_or_else(|p| p.into_inner()) = ProbeOffsets {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            };
            let round = Arc::new(ManualRound {
                ops: Arc::new(LiveRound {
                    helper: Arc::clone(self),
                    printer: Arc::clone(&printer),
                    probe: None,
                }),
                results: Mutex::new(Vec::new()),
            });
            return round.start();
        }
        let Some(probe) = probe else {
            return Err(CommandError::new("Printer is not ready"));
        };

        // Automatic probing through the `probe` object's session.
        let params = probe.probe_params(gcmd)?;
        *self.lift_speed.lock().unwrap_or_else(|p| p.into_inner()) = params.lift_speed;
        let offsets = probe.offsets();
        *self.probe_offsets.lock().unwrap_or_else(|p| p.into_inner()) = offsets;
        if horizontal_move_z < offsets.z {
            return Err(CommandError::new(
                "horizontal_move_z can't be less than probe's z_offset",
            ));
        }
        probe.start_probe_session(gcmd)?;
        let ops = LiveRound {
            helper: Arc::clone(self),
            printer,
            probe: Some(probe),
        };
        automatic_round(&ops, gcmd).await
    }
}

impl std::fmt::Debug for ProbePointsHelper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ProbePointsHelper")
            .field("name", &self.name)
            .field("speed", &self.speed)
            .finish_non_exhaustive()
    }
}

/// The side effects one probing round drives: the toolhead moves and the
/// probe session. Split out so the round's order is testable without a
/// machine — the tests drive a recording fake.
trait RoundOps: Send + Sync {
    /// How many points the round walks (`probe_points.len()`).
    fn point_count(&self) -> usize;
    /// Move Z up to `horizontal_move_z` (`probe.py:_raise_tool`).
    fn raise_tool(&self, is_first: bool) -> Result<(), CommandError>;
    /// Move to point `probe_num` (`probe.py:_move_next`).
    fn move_next(&self, probe_num: usize) -> Result<(), CommandError>;
    /// One sample set at the current position (`run_probe`).
    fn run_probe<'a>(&'a self, gcmd: &'a GcodeCommand) -> CommandFuture<'a>;
    /// Take the session's completed sample sets (`pull_probed_results`).
    fn pull_results(&self) -> Vec<Coord>;
    /// Flush the lookahead queue and invoke the finalize callback; `false`
    /// means "retry" — the round starts over (`probe.py:_invoke_callback`).
    fn invoke_callback(&self, results: &[Coord]) -> Result<bool, CommandError>;
    /// Close the probe session (`end_probe_session`).
    fn end_session(&self) -> Result<(), CommandError>;
    /// Start one interactive manual point (`ManualProbe::start_helper`).
    fn start_manual_helper(&self, callback: FinalizeCallback) -> Result<(), CommandError>;
}

/// The real [`RoundOps`]: a helper, the machine, and (when automatic) the
/// open probe session.
struct LiveRound {
    helper: Arc<ProbePointsHelper>,
    printer: Arc<Printer>,
    /// `None` in manual mode.
    probe: Option<Arc<dyn ProbeSession>>,
}

impl LiveRound {
    /// The toolhead the moves go through.
    fn toolhead(&self) -> Result<Arc<ToolHeadObject>, CommandError> {
        self.printer
            .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }
}

impl RoundOps for LiveRound {
    fn point_count(&self) -> usize {
        self.helper
            .probe_points
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .len()
    }

    fn raise_tool(&self, is_first: bool) -> Result<(), CommandError> {
        // The first raise runs at full travel speed, the rest at the lift
        // speed (`probe.py:_raise_tool`).
        let speed = if is_first {
            self.helper.speed
        } else {
            *self
                .helper
                .lift_speed
                .lock()
                .unwrap_or_else(|p| p.into_inner())
        };
        let toolhead = self.toolhead()?;
        let mut target = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        target.set_axis(
            Z_AXIS,
            *self
                .helper
                .horizontal_move_z
                .lock()
                .unwrap_or_else(|p| p.into_inner()),
        );
        toolhead.move_to(target, speed)
    }

    fn move_next(&self, probe_num: usize) -> Result<(), CommandError> {
        let (x, y) = self.helper.move_target(probe_num)?;
        let toolhead = self.toolhead()?;
        let mut target = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        target.set_axis(0, x);
        target.set_axis(1, y);
        toolhead.move_to(target, self.helper.speed)
    }

    fn run_probe<'a>(&'a self, gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
        let Some(probe) = self.probe.as_ref() else {
            return Box::pin(std::future::ready(Err(CommandError::new(
                "Internal probe error - no probe session",
            ))));
        };
        probe.run_probe(gcmd)
    }

    fn pull_results(&self) -> Vec<Coord> {
        self.probe
            .as_ref()
            .map(|probe| probe.pull_probed_results())
            .unwrap_or_default()
    }

    fn invoke_callback(&self, results: &[Coord]) -> Result<bool, CommandError> {
        // Flush the lookahead queue: upstream asks `get_last_move_time`,
        // which both flushes and returns the time; reading it here is this
        // port's equivalent sync point before the callback runs.
        let toolhead = self.toolhead()?;
        let _ = toolhead.print_time();
        let offsets = *self
            .helper
            .probe_offsets
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let answer = (self.helper.finalize)(offsets, results);
        // Upstream: `res != "retry"`.
        Ok(answer != Some(RETRY))
    }

    fn end_session(&self) -> Result<(), CommandError> {
        match self.probe.as_ref() {
            Some(probe) => probe.end_probe_session(),
            None => Ok(()),
        }
    }

    fn start_manual_helper(&self, callback: FinalizeCallback) -> Result<(), CommandError> {
        let manual = self
            .printer
            .lookup_object_as::<ManualProbe>(MANUAL_PROBE_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let gcode = self
            .printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");
        // The helper gets its own empty command so the outer command's
        // parameters (SPEED) do not leak into it, as upstream's
        // `gcode.create_gcode_command("", "", {})` does.
        let gcmd = gcode.create_gcode_command("", "", std::collections::HashMap::new());
        manual.start_helper(&self.printer, &gcmd, callback)
    }
}

/// One automatic probing round (`probe.py:start_probe`'s `while 1` loop):
/// raise, walk every point, pull the results into the callback, restart on
/// "retry", close the session.
async fn automatic_round<R: RoundOps + ?Sized>(
    ops: &R,
    gcmd: &GcodeCommand,
) -> Result<(), CommandError> {
    let mut probe_num = 0usize;
    loop {
        ops.raise_tool(probe_num == 0)?;
        if probe_num >= ops.point_count() {
            let results = ops.pull_results();
            if ops.invoke_callback(&results)? {
                break;
            }
            // The caller wants a "retry" — restart probing.
            probe_num = 0;
        }
        ops.move_next(probe_num)?;
        ops.run_probe(gcmd).await?;
        probe_num += 1;
    }
    ops.end_session()
}

/// One manual probing round (`probe.py:_manual_probe_start` +
/// `_manual_probe_finalize`): every point waits for the user's `G1`+`ACCEPT`,
/// and a finished list goes to the finalize callback like the automatic one.
struct ManualRound<R: RoundOps> {
    ops: Arc<R>,
    /// The kinematics positions the user accepted, in point order.
    results: Mutex<Vec<Coord>>,
}

impl<R: RoundOps + 'static> ManualRound<R> {
    /// Raise, finish-or-clear the list, move to the next point and start its
    /// helper (`probe.py:_manual_probe_start`).
    fn start(self: &Arc<Self>) -> Result<(), CommandError> {
        let is_first = self
            .results
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_empty();
        self.ops.raise_tool(is_first)?;
        {
            let results = self.results.lock().unwrap_or_else(|p| p.into_inner());
            if results.len() >= self.ops.point_count() {
                let snapshot = results.clone();
                drop(results);
                if self.ops.invoke_callback(&snapshot)? {
                    return Ok(());
                }
                // The caller wants a "retry" — clear results and restart.
                *self.results.lock().unwrap_or_else(|p| p.into_inner()) = Vec::new();
            }
        }
        let next = self.results.lock().unwrap_or_else(|p| p.into_inner()).len();
        self.ops.move_next(next)?;
        let this = Arc::clone(self);
        self.ops.start_manual_helper(Arc::new(move |kin_pos| {
            if let Err(err) = this.manual_finalize(kin_pos) {
                warn!("manual probe point failed: {err}");
            }
        }))
    }

    /// `_manual_probe_finalize`: keep an accepted point and drive the next,
    /// or stop when the user aborted.
    fn manual_finalize(self: &Arc<Self>, kin_pos: Option<Coord>) -> Result<(), CommandError> {
        let Some(pos) = kin_pos else {
            return Ok(());
        };
        self.results
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push(pos);
        self.start()
    }
}

// ===========================================================================
// ProbePointsHelper tests
// ===========================================================================

#[cfg(test)]
mod probe_points_tests {
    use super::*;
    use crate::core::klippy::config::{ConfigSection, ConfigValue};
    use crate::core::klippy::reactor::ManualReactor;
    use std::sync::atomic::AtomicUsize;

    /// A `[z_tilt]`-style section with the given `points` value.
    fn section(points: Option<ConfigValue>) -> ConfigSection {
        let mut section = ConfigSection::new("z_tilt", None);
        if let Some(value) = points {
            section.parameters.insert("points".to_string(), value);
        }
        section
    }

    /// A helper over a throwaway printer; the machine is only needed once
    /// [`ProbePointsHelper::start_probe`] runs, which these tests never do.
    fn helper(points: Option<ConfigValue>) -> Result<Arc<ProbePointsHelper>, ConfigError> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let section = section(points);
        let config = ConfigWrapper::untracked(&section);
        ProbePointsHelper::new(&config, &printer, Arc::new(|_, _| None))
    }

    fn rows(rows: &[&str]) -> ConfigValue {
        ConfigValue::Multi(rows.iter().map(|row| row.to_string()).collect())
    }

    /// One recorded step of a fake round.
    #[derive(Debug, Clone, PartialEq)]
    enum Step {
        Raise(bool),
        MoveNext(usize),
        Probe,
        Pull,
        Callback(usize),
        ManualStart,
        End,
    }

    /// A recording [`RoundOps`]: no machine, just the call order.
    struct FakeRound {
        points: usize,
        /// How many callbacks still answer "retry".
        retries: AtomicUsize,
        log: Mutex<Vec<Step>>,
        manual_callback: Mutex<Option<FinalizeCallback>>,
    }

    impl FakeRound {
        fn new(points: usize, retries: usize) -> Self {
            Self {
                points,
                retries: AtomicUsize::new(retries),
                log: Mutex::new(Vec::new()),
                manual_callback: Mutex::new(None),
            }
        }

        fn record(&self, step: Step) {
            self.log
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(step);
        }

        fn steps(&self) -> Vec<Step> {
            self.log.lock().unwrap_or_else(|p| p.into_inner()).clone()
        }

        /// Take the helper callback the round registered for the next point.
        fn take_manual_callback(&self) -> FinalizeCallback {
            self.manual_callback
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .take()
                .expect("a manual helper was started")
        }
    }

    impl RoundOps for FakeRound {
        fn point_count(&self) -> usize {
            self.points
        }

        fn raise_tool(&self, is_first: bool) -> Result<(), CommandError> {
            self.record(Step::Raise(is_first));
            Ok(())
        }

        fn move_next(&self, probe_num: usize) -> Result<(), CommandError> {
            self.record(Step::MoveNext(probe_num));
            Ok(())
        }

        fn run_probe<'a>(&'a self, _gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
            self.record(Step::Probe);
            Box::pin(std::future::ready(Ok(())))
        }

        fn pull_results(&self) -> Vec<Coord> {
            self.record(Step::Pull);
            Vec::new()
        }

        fn invoke_callback(&self, results: &[Coord]) -> Result<bool, CommandError> {
            self.record(Step::Callback(results.len()));
            if self.retries.load(Ordering::SeqCst) > 0 {
                self.retries.fetch_sub(1, Ordering::SeqCst);
                return Ok(false);
            }
            Ok(true)
        }

        fn end_session(&self) -> Result<(), CommandError> {
            self.record(Step::End);
            Ok(())
        }

        fn start_manual_helper(&self, callback: FinalizeCallback) -> Result<(), CommandError> {
            self.record(Step::ManualStart);
            *self
                .manual_callback
                .lock()
                .unwrap_or_else(|p| p.into_inner()) = Some(callback);
            Ok(())
        }
    }

    fn block_on<F: std::future::Future>(future: F) -> F::Output {
        tokio::runtime::Builder::new_current_thread()
            .build()
            .expect("a current-thread runtime")
            .block_on(future)
    }

    fn dummy_gcmd() -> (Arc<Printer>, GCodeDispatch, GcodeCommand) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let gcode = GCodeDispatch::new(Arc::clone(&printer));
        let gcmd = gcode.create_gcode_command("TEST", "", std::collections::HashMap::new());
        (printer, gcode, gcmd)
    }

    #[test]
    fn points_rows_parse_from_newlines_and_commas() {
        // The corpus format: one `x,y` row per line.
        let grid = helper(Some(rows(&["50,50", "50,195", "195,195"]))).unwrap();
        let points = grid
            .probe_points
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        assert_eq!(points, vec![(50.0, 50.0), (50.0, 195.0), (195.0, 195.0)]);

        // A single-line value works the same way.
        let single = helper(Some(ConfigValue::Single("10, 20".to_string()))).unwrap();
        let points = single
            .probe_points
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        assert_eq!(points, vec![(10.0, 20.0)]);
    }

    #[test]
    fn a_row_without_two_elements_is_refused() {
        // An odd point count leaves a row with one value: upstream's
        // `getlists(..., count=2)` error.
        let err = helper(Some(rows(&["50,50", "195"]))).unwrap_err();

        assert!(err.to_string().contains("must have 2 elements"), "{err}");
    }

    #[test]
    fn missing_points_reports_upstream_wording() {
        let err = helper(None).unwrap_err();

        assert_eq!(
            err.to_string(),
            "Option 'points' in section 'z_tilt' is not defined"
        );
    }

    #[test]
    fn minimum_points_and_update_probe_points_follow_upstream_wording() {
        let helper = helper(Some(rows(&["50,50", "50,195"]))).unwrap();

        helper.minimum_points(2).unwrap();
        let err = helper.minimum_points(3).unwrap_err();
        assert_eq!(err.to_string(), "Need at least 3 probe points for z_tilt");

        // update_probe_points swaps the list and re-checks.
        helper
            .update_probe_points(vec![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)], 3)
            .unwrap();
        helper.minimum_points(3).unwrap();
        let err = helper.update_probe_points(vec![(0.0, 0.0)], 3).unwrap_err();
        assert_eq!(err.to_string(), "Need at least 3 probe points for z_tilt");
    }

    #[test]
    fn the_options_carry_upstream_defaults() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let mut section = ConfigSection::new("z_tilt", None);
        section
            .parameters
            .insert("points".to_string(), ConfigValue::Single("50,50".into()));
        let config = ConfigWrapper::untracked(&section);
        let helper = ProbePointsHelper::new(&config, &printer, Arc::new(|_, _| None)).unwrap();

        assert_eq!(helper.default_horizontal_move_z, 5.0);
        assert_eq!(helper.speed, 50.0);
        assert!(!helper.use_offsets.load(Ordering::SeqCst));
        assert_eq!(helper.get_lift_speed(), 50.0);

        // `speed` is `above=0.` (`probe.py:438`).
        section
            .parameters
            .insert("speed".to_string(), ConfigValue::Single("0".into()));
        let config = ConfigWrapper::untracked(&section);
        let err = ProbePointsHelper::new(&config, &printer, Arc::new(|_, _| None)).unwrap_err();
        assert!(err.to_string().contains("must be above 0"), "{err}");
    }

    #[test]
    fn move_target_subtracts_offsets_only_when_asked() {
        let helper = helper(Some(rows(&["50,50", "195,100"]))).unwrap();
        *helper
            .probe_offsets
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = ProbeOffsets {
            x: 23.0,
            y: 5.0,
            z: 2.0,
        };

        // Without `use_xy_offsets` the raw point is the target.
        assert_eq!(helper.move_target(0).unwrap(), (50.0, 50.0));

        helper.use_xy_offsets(true);
        assert_eq!(helper.move_target(0).unwrap(), (50.0 - 23.0, 50.0 - 5.0));
        assert_eq!(helper.move_target(1).unwrap(), (195.0 - 23.0, 100.0 - 5.0));

        // One past the end is an internal error, not a panic.
        assert!(helper.move_target(2).is_err());
    }

    #[test]
    fn the_automatic_round_walks_every_point_then_finishes() {
        let (_printer, _gcode, gcmd) = dummy_gcmd();
        let fake = FakeRound::new(2, 0);

        block_on(automatic_round(&fake, &gcmd)).unwrap();

        assert_eq!(
            fake.steps(),
            vec![
                // Raise at full speed first, then move/probe point 0.
                Step::Raise(true),
                Step::MoveNext(0),
                Step::Probe,
                Step::Raise(false),
                Step::MoveNext(1),
                Step::Probe,
                // A last raise, then the results go to the callback.
                Step::Raise(false),
                Step::Pull,
                Step::Callback(0),
                Step::End,
            ]
        );
    }

    #[test]
    fn a_retry_answer_restarts_the_walk() {
        let (_printer, _gcode, gcmd) = dummy_gcmd();
        let fake = FakeRound::new(2, 1);

        block_on(automatic_round(&fake, &gcmd)).unwrap();

        let steps = fake.steps();
        // First attempt, the retry (`Pull`, `Callback` without `End`), then
        // the second attempt from point 0 again.
        assert_eq!(
            steps,
            vec![
                Step::Raise(true),
                Step::MoveNext(0),
                Step::Probe,
                Step::Raise(false),
                Step::MoveNext(1),
                Step::Probe,
                Step::Raise(false),
                Step::Pull,
                Step::Callback(0),
                Step::MoveNext(0),
                Step::Probe,
                Step::Raise(false),
                Step::MoveNext(1),
                Step::Probe,
                Step::Raise(false),
                Step::Pull,
                Step::Callback(0),
                Step::End,
            ]
        );
    }

    #[test]
    fn the_manual_round_starts_one_helper_per_point() {
        let fake = Arc::new(FakeRound::new(2, 0));
        let round = Arc::new(ManualRound {
            ops: Arc::clone(&fake),
            results: Mutex::new(Vec::new()),
        });

        round.start().unwrap();
        assert_eq!(
            fake.steps(),
            vec![Step::Raise(true), Step::MoveNext(0), Step::ManualStart]
        );

        // The user accepts point 0: raise, move to point 1, next helper.
        fake.take_manual_callback()(Some(Coord::new(0.0, 0.0, 1.0, 0.0)));
        assert_eq!(
            fake.steps(),
            vec![
                Step::Raise(true),
                Step::MoveNext(0),
                Step::ManualStart,
                Step::Raise(false),
                Step::MoveNext(1),
                Step::ManualStart,
            ]
        );

        // The last point finishes the round: the callback gets every result
        // and no further helper starts.
        fake.take_manual_callback()(Some(Coord::new(1.0, 1.0, 2.0, 0.0)));
        let steps = fake.steps();
        assert_eq!(
            steps[steps.len() - 2..],
            [Step::Raise(false), Step::Callback(2)]
        );
        assert_eq!(
            steps
                .iter()
                .filter(|step| **step == Step::ManualStart)
                .count(),
            2
        );
    }

    #[test]
    fn an_aborted_manual_point_stops_the_round() {
        let fake = Arc::new(FakeRound::new(2, 0));
        let round = Arc::new(ManualRound {
            ops: Arc::clone(&fake),
            results: Mutex::new(Vec::new()),
        });

        round.start().unwrap();
        let before = fake.steps().len();
        // ABORT reports `None`: nothing further runs.
        fake.take_manual_callback()(None);
        assert_eq!(fake.steps().len(), before);
    }

    #[test]
    fn a_manual_retry_clears_the_results_and_walks_again() {
        let fake = Arc::new(FakeRound::new(1, 1));
        let round = Arc::new(ManualRound {
            ops: Arc::clone(&fake),
            results: Mutex::new(Vec::new()),
        });

        round.start().unwrap();
        // One point, one accept — but the callback answers "retry", so the
        // list clears and point 0 is probed again.
        fake.take_manual_callback()(Some(Coord::new(0.0, 0.0, 1.0, 0.0)));
        let steps = fake.steps();
        assert_eq!(
            steps,
            vec![
                Step::Raise(true),
                Step::MoveNext(0),
                Step::ManualStart,
                Step::Raise(false),
                Step::Callback(1),
                Step::MoveNext(0),
                Step::ManualStart,
            ]
        );
    }

    // -----------------------------------------------------------------------
    // The probe session trait seam (M5b): one dispatch, two implementations,
    // plus the sample delivery seam
    // -----------------------------------------------------------------------

    /// A second probe section's session — the shape an eddy probe lands
    /// later: not a `ProbeSessionHelper`, but the same trait surface. It also
    /// receives samples through [`SampleDelivery`].
    struct StubProbe {
        /// What the round drove, in order.
        log: Mutex<Vec<&'static str>>,
        /// The results `pull_probed_results` hands out (taken, as the real
        /// session takes them).
        results: Mutex<Vec<Coord>>,
        /// The samples delivered through the seam.
        samples: Mutex<Vec<(f64, f64)>>,
    }

    impl StubProbe {
        fn new(results: Vec<Coord>) -> Self {
            Self {
                log: Mutex::new(Vec::new()),
                results: Mutex::new(results),
                samples: Mutex::new(Vec::new()),
            }
        }

        fn record(&self, call: &'static str) {
            self.log
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(call);
        }

        fn calls(&self) -> Vec<&'static str> {
            self.log.lock().unwrap_or_else(|p| p.into_inner()).clone()
        }

        fn delivered(&self) -> Vec<(f64, f64)> {
            self.samples
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .clone()
        }
    }

    impl ProbeSession for StubProbe {
        fn start_probe_session(&self, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
            self.record("start");
            Ok(())
        }

        fn run_probe<'a>(&'a self, _gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
            self.record("run");
            Box::pin(std::future::ready(Ok(())))
        }

        fn probe_params(&self, _gcmd: &GcodeCommand) -> Result<ProbeParams, CommandError> {
            self.record("params");
            Ok(ProbeParams {
                probe_speed: 5.0,
                lift_speed: 5.0,
                samples: 1,
                sample_retract_dist: 2.0,
                samples_tolerance: 0.100,
                samples_tolerance_retries: 0,
                samples_result: "median".to_string(),
            })
        }

        fn pull_probed_results(&self) -> Vec<Coord> {
            self.record("pull");
            std::mem::take(&mut *self.results.lock().unwrap_or_else(|p| p.into_inner()))
        }

        fn end_probe_session(&self) -> Result<(), CommandError> {
            self.record("end");
            Ok(())
        }

        fn offsets(&self) -> ProbeOffsets {
            self.record("offsets");
            ProbeOffsets {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            }
        }
    }

    impl SampleDelivery for StubProbe {
        fn deliver_sample(&self, time: f64, value: f64) {
            self.samples
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push((time, value));
        }
    }

    /// The real z implementation, assembled from its parts the way
    /// `PrinterProbe::new` does once the pin layer built the endstop — no
    /// machine needed for the session's command surface.
    fn real_z_probe(printer: &Arc<Printer>, probe_options: &[(&str, &str)]) -> Arc<PrinterProbe> {
        use crate::core::klippy::mcu::{ConfigBuilder, McuChip};

        let mut section = ConfigSection::new("probe", None);
        for (option, value) in probe_options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        let config = ConfigWrapper::untracked(&section);
        let options = ProbeOptions::read(&config).unwrap();
        let chip = McuChip::new(
            "mcu".to_string(),
            Arc::new(ConfigBuilder::new()),
            Arc::new(PrinterPins::new()),
        );
        let params = PinParams {
            chip_name: "mcu".to_string(),
            pin: "PA0".to_string(),
            invert: false,
            pullup: 0,
            share_type: None,
        };
        let endstop = Arc::new(McuEndstop::new(chip, &params).unwrap());
        let session = Arc::new(
            ProbeSessionHelper::new(
                &config,
                printer,
                Arc::clone(&endstop) as Arc<dyn HomingEndstop>,
                Arc::clone(&endstop),
                &options,
                None,
            )
            .unwrap(),
        );
        Arc::new(PrinterProbe::from_parts(
            "probe".to_string(),
            options,
            Arc::clone(&endstop),
            session,
            Arc::new(ProbeCommandState::default()),
        ))
    }

    #[test]
    fn the_round_drives_both_probe_implementations_through_one_dispatch() {
        let (_printer, _gcode, gcmd) = dummy_gcmd();
        let helper = helper(Some(rows(&["50,50"]))).unwrap();
        let printer = Arc::new(Printer::new(ManualReactor::shared()));

        // The lookup the round starts from finds nothing without a `probe`
        // object (the round's manual branch).
        assert!(lookup_probe_session(&printer).is_none());

        // First implementation: the real z probe, registered as `probe`.
        // Every call the round makes runs its actual session logic.
        let z = real_z_probe(&printer, &[("pin", "PA0"), ("z_offset", "1.5")]);
        printer
            .add_object(PROBE_OBJECT, Arc::clone(&z) as Arc<dyn PrinterObject>)
            .unwrap();
        let session: Arc<dyn ProbeSession> =
            lookup_probe_session(&printer).expect("the registered probe answers the lookup");
        session.start_probe_session(&gcmd).unwrap();
        // A second open is still the session-mismatch refusal.
        let err = session.start_probe_session(&gcmd).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Internal probe error - start/end probe session mismatch"
        );

        // Through the round's dispatch point, with no machine behind it, the
        // real path stops at the toolhead lookup — its pre-trait error.
        let round = LiveRound {
            helper: Arc::clone(&helper),
            printer: Arc::clone(&printer),
            probe: Some(Arc::clone(&session)),
        };
        let err = block_on(round.run_probe(&gcmd)).unwrap_err();
        assert_eq!(err.to_string(), "Printer is not ready");
        assert!(round.pull_results().is_empty());
        round.end_session().unwrap();
        assert_eq!(
            session.offsets(),
            ProbeOffsets {
                x: 0.0,
                y: 0.0,
                z: 1.5
            }
        );
        assert_eq!(session.probe_params(&gcmd).unwrap().probe_speed, 5.0);

        // Second implementation: the same dispatch, another session — and
        // the stub saw nothing of the z half above.
        let stub = Arc::new(StubProbe::new(vec![Coord::new(0.0, 0.0, 1.0, 0.0)]));
        assert!(stub.calls().is_empty());
        let stub_session: Arc<dyn ProbeSession> = Arc::clone(&stub) as Arc<dyn ProbeSession>;
        let stub_round = LiveRound {
            helper: Arc::clone(&helper),
            printer: Arc::clone(&printer),
            probe: Some(stub_session),
        };
        stub_round
            .probe
            .as_ref()
            .expect("the stub round carries a session")
            .start_probe_session(&gcmd)
            .unwrap();
        block_on(stub_round.run_probe(&gcmd)).unwrap();
        assert_eq!(
            stub_round.pull_results(),
            vec![Coord::new(0.0, 0.0, 1.0, 0.0)]
        );
        stub_round.end_session().unwrap();
        assert_eq!(stub.calls(), vec!["start", "run", "pull", "end"]);
        assert_eq!(stub_round.point_count(), 1);
    }

    #[test]
    fn samples_delivered_through_the_seam_reach_the_session() {
        let stub = Arc::new(StubProbe::new(Vec::new()));
        let delivery: Arc<dyn SampleDelivery> = Arc::clone(&stub) as Arc<dyn SampleDelivery>;

        delivery.deliver_sample(12.5, 654_321.0);
        delivery.deliver_sample(12.5025, 654_000.0);

        assert_eq!(
            stub.delivered(),
            vec![(12.5, 654_321.0), (12.5025, 654_000.0)]
        );
    }
}
