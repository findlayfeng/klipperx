//! `[probe]` — the probe's virtual Z endstop, its session and its commands.
//!
//! Upstream `klippy/extras/probe.py`. This module lands the `[probe]` section,
//! its option set, the `probe` virtual pin chip, the probe session (sampling
//! with tolerance retries) and `QUERY_PROBE` / `PROBE` / `PROBE_ACCURACY`.
//!
//! What is **not** here yet (tracked in `TODO.md` H9):
//!
//! - the endstop *wrapper*'s overrides — `z_offset` folded into the reported
//!   trigger position (`get_position_endstop`) and `query_endstop` /
//!   `multi_probe_begin/end` / `probe_prepare` / `probe_finish`. The pin layer's
//!   `PinChip::setup_endstop` returns a concrete `Arc<McuEndstop>`, so a virtual
//!   chip cannot hand back a wrapper type yet; the trait has to become an
//!   interface first. Until then the chip returns the physical endstop, which is
//!   what makes `endstop_pin: probe:z_virtual_endstop` resolvable, and the rail's
//!   `position_endstop` keeps coming from the config (upstream takes it from the
//!   endstop, `z_offset`).
//! - `PROBE_CALIBRATE` and `Z_OFFSET_APPLY_PROBE`: they need `manual_probe` and
//!   `configfile.set()` (SAVE_CONFIG write-back), both later units.
//! - `activate_gcode` / `deactivate_gcode` templates: they need the
//!   `[gcode_macro]` template machinery, which this port does not have (H3).
//!   The options are read and recorded; a section that sets them warns.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{CommandError, GCodeDispatch, GcodeCommand, GCODE_OBJECT};
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

/// The only pin name the chip answers to (`klippy/extras/probe.py:222-229`).
const VIRTUAL_ENDSTOP: &str = "z_virtual_endstop";

/// The toolhead object, as the loader registers `[printer]`.
const TOOLHEAD_OBJECT: &str = "toolhead";

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

impl ProbeParams {
    /// The section's defaults: `lift_speed` falls back to `speed`, as upstream
    /// does (`probe.py:250`).
    fn from_options(options: &ProbeOptions) -> Self {
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
    fn from_command(&self, gcmd: &GcodeCommand) -> Result<Self, CommandError> {
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
struct ProbeChip {
    /// The physical probe endstop the virtual name resolves to.
    endstop: Arc<McuEndstop>,
}

impl PinChip for ProbeChip {
    fn setup_digital_out(&self, _params: &PinParams) -> Result<Arc<dyn DigitalOut>, PinError> {
        Err(PinError::Unsupported("digital_out".to_string()))
    }

    fn setup_endstop(&self, params: &PinParams) -> Result<Arc<McuEndstop>, PinError> {
        check_virtual_endstop(params)?;
        Ok(Arc::clone(&self.endstop))
    }
}

/// Upstream's two refusals for the virtual endstop (`probe.py:223-229`).
///
/// Split out so the checks are testable without an MCU.
fn check_virtual_endstop(params: &PinParams) -> Result<(), PinError> {
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
fn calc_probe_z_average(positions: &[Coord], method: &str) -> Coord {
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
#[derive(Debug, Default)]
struct ProbeCommandState {
    /// The last `QUERY_PROBE` result.
    last_query: AtomicBool,
    /// The last `PROBE` result.
    last_z_result: Mutex<f64>,
}

/// Tracks a series of probe attempts within one command
/// (`probe.py:ProbeSessionHelper`).
struct ProbeSessionHelper {
    /// The machine, for the toolhead and the results event.
    printer: Weak<Printer>,
    /// The physical probe endstop every probing move drives.
    endstop: Arc<McuEndstop>,
    /// The Z to move down to while probing: `[stepper_z] position_min`, or
    /// `[printer] minimum_z_position` when there is no Z stepper
    /// (`probe.py:238-245`).
    z_position: f64,
    /// The section's parameters; a command may override them.
    defaults: ProbeParams,
    /// Whether a session is open.
    pending: AtomicBool,
    /// The sample sets run in this session.
    results: Mutex<Vec<Coord>>,
}

impl ProbeSessionHelper {
    /// Read the session's defaults and the Z position to probe to.
    ///
    /// # Errors
    /// When `[stepper_z] position_min` / `[printer] minimum_z_position` is
    /// present but not a number.
    fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        endstop: Arc<McuEndstop>,
        options: &ProbeOptions,
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
            z_position,
            defaults: ProbeParams::from_options(options),
            pending: AtomicBool::new(false),
            results: Mutex::new(Vec::new()),
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
        Ok(())
    }

    /// Close a session (`probe.py:end_probe_session`).
    ///
    /// # Errors
    /// When no session is open.
    fn end(&self) -> Result<(), CommandError> {
        if !self.pending.swap(false, Ordering::SeqCst) {
            return Err(Self::state_error());
        }
        self.results
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clear();
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
        let epos = toolhead.probing_move(&*self.endstop, target, speed).await?;
        // `axis_twist_compensation` updates its results from this event
        // (`probe.py:329`); this port's event carries no payload yet.
        if let Some(printer) = self.printer.upgrade() {
            printer.send_event(&KlippyEvent::ProbeUpdateResults);
        }
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
            }),
        )
        .map_err(|err| ConfigError::new(format!("{identifier}: {err}")))?;

        let session = Arc::new(ProbeSessionHelper::new(
            config,
            printer,
            Arc::clone(&endstop),
            &options,
        )?);
        let state = Arc::new(ProbeCommandState::default());
        register_commands(printer, &identifier, &session, &state)?;

        Ok(Self {
            identifier,
            options,
            endstop,
            session,
            state,
        })
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

    /// The session's parameters for `gcmd` (`get_probe_params`).
    #[allow(dead_code)] // `bed_mesh` (U5) and `z_tilt` (M6) consume these.
    pub(crate) fn probe_params(&self, gcmd: &GcodeCommand) -> Result<ProbeParams, CommandError> {
        self.session.defaults.from_command(gcmd)
    }

    /// Open a probe session (`start_probe_session`).
    #[allow(dead_code)] // consumed by the point-probing helpers (U5/M6).
    pub(crate) fn start_probe_session(&self) -> Result<(), CommandError> {
        self.session.start()
    }

    /// Run one sample set in the open session (`run_probe`).
    #[allow(dead_code)] // consumed by the point-probing helpers (U5/M6).
    pub(crate) async fn run_probe(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        self.session.run(gcmd).await
    }

    /// Take the completed sample sets (`pull_probed_results`).
    #[allow(dead_code)] // consumed by the point-probing helpers (U5/M6).
    pub(crate) fn pull_probed_results(&self) -> Vec<Coord> {
        self.session.pull_results()
    }

    /// Close the session (`end_probe_session`).
    #[allow(dead_code)] // consumed by the point-probing helpers (U5/M6).
    pub(crate) fn end_probe_session(&self) -> Result<(), CommandError> {
        self.session.end()
    }
}

/// Register `QUERY_PROBE`, `PROBE` and `PROBE_ACCURACY`
/// (`probe.py:ProbeCommandHelper`), plus the session cleanup on a command
/// error.
fn register_commands(
    printer: &Arc<Printer>,
    identifier: &str,
    session: &Arc<ProbeSessionHelper>,
    state: &Arc<ProbeCommandState>,
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
                            .endstop
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
            .register_command(
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
                false,
            )
            .map_err(ConfigError::new)?;
    }

    // PROBE_ACCURACY: `SAMPLES` single-sample probes, then the spread
    // (`probe.py:cmd_PROBE_ACCURACY`).
    {
        let session = Arc::clone(session);
        gcode
            .register_command(
                "PROBE_ACCURACY",
                Arc::new(move |gcmd| {
                    let session = Arc::clone(&session);
                    let name = name.clone();
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
                        let _ = name;
                        Ok(())
                    })
                }),
                Some("Probe Z-height accuracy at current XY position"),
                false,
            )
            .map_err(ConfigError::new)?;
    }

    Ok(())
}

impl PrinterObject for PrinterProbe {
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({
            "name": self.identifier,
            "last_query": self.state.last_query.load(Ordering::SeqCst),
            "last_z_result": *self
                .state
                .last_z_result
                .lock()
                .unwrap_or_else(|p| p.into_inner()),
        })
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
