//! `[z_tilt]` — mechanical bed tilt calibration with multiple Z steppers
//! (upstream `klippy/extras/z_tilt.py`).
//!
//! Upstream keeps three helpers in `z_tilt.py` and imports them from there for
//! `[quad_gantry_level]` (`quad_gantry_level.py:from . import z_tilt`); this
//! module does the same: [`RetryHelper`], [`ZAdjustStatus`] and
//! [`ZAdjustHelper`] are public and reused by
//! [`quad_gantry_level`](super::quad_gantry_level).
//!
//! The command flow: `Z_TILT_ADJUST` resets the applied flag, arms the retry
//! helper and hands the configured probe points to
//! [`ProbePointsHelper`], which drives automatic or manual probing. Once every
//! point has a position, [`ZTilt::probe_finalize`] fits the bed plane by
//! coordinate descent, moves each Z motor by its share of the plane
//! ([`ZAdjustHelper::adjust_steppers`]) and answers whether the round should
//! retry.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock, Weak};

use serde_json::{json, Value};
use tracing::warn;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::error::KlippyError;
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::probe::{
    probe_points_params, ProbeOffsets, ProbePointsFinalize, ProbePointsHelper, RETRY,
};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    parse_float, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{coordinate_descent, Coord, Z_AXIS};
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};

section!("z_tilt", order = 30, load = load_config);

/// The toolhead object, as the loader registers `[printer]`.
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The object this section is registered under (the finalize callback looks
/// itself up through the registry to reach its own probe helper).
const Z_TILT_OBJECT: &str = "z_tilt";

/// Drive a finalize callback's awaits from the callback's synchronous
/// context.
///
/// `ProbePointsHelper`'s finalize callback is a plain `Fn` — upstream's
/// `probe_finalize` runs synchronously there too — while this port's toolhead
/// flushes ([`ToolHeadObject::flush_step_generation`],
/// [`ToolHeadObject::set_position`]) are async. The callback always runs from
/// inside a command future, so `block_in_place` parks the (multi-threaded)
/// runtime worker while the flush runs on this thread. Never call this from a
/// current-thread runtime or outside a runtime: `Handle::current()` would
/// panic.
pub(crate) fn block_in_command<F: std::future::Future>(future: F) -> F::Output {
    let handle = tokio::runtime::Handle::current();
    tokio::task::block_in_place(|| handle.block_on(future))
}

/// One `getlists(option, seps=(',', '\n'), parser=float, count=2)` option as
/// parsed `(x, y)` rows — upstream's `z_positions` and `gantry_corners` read.
///
/// Upstream's `getlists` refuses a missing option with
/// "Option '<name>' in section '<section>' is not defined"; this port's
/// `get_list_of_lists` returns an empty list instead, so the absence is
/// checked here to keep upstream's wording.
///
/// # Errors
/// The missing-option message above, "must have 2 elements" for a malformed
/// row, or "Unable to parse option …" for a value that is not a number.
pub(crate) fn read_xy_option(
    config: &ConfigWrapper,
    option: &str,
) -> Result<Vec<(f64, f64)>, ConfigError> {
    if !config.has(option) {
        return Err(ConfigError::new(format!(
            "Option '{option}' in section '{}' is not defined",
            config.identifier()
        )));
    }
    let groups = config.get_list_of_lists(option, '\n', ',', 2)?;
    let identifier = config.identifier();
    groups
        .into_iter()
        .map(|pair| {
            let mut parsed = pair.iter().map(|item| {
                item.trim().parse::<f64>().map_err(|_| {
                    ConfigError::new(format!(
                        "Unable to parse option '{option}' in section '{identifier}'"
                    ))
                })
            });
            let x = parsed.next().expect("get_list_of_lists checked count=2")?;
            let y = parsed.next().expect("get_list_of_lists checked count=2")?;
            Ok((x, y))
        })
        .collect()
}

// ===========================================================================
// RetryHelper (z_tilt.py:85-125, shared with quad_gantry_level)
// ===========================================================================

/// The retry arming and range checking of upstream's `RetryHelper`: how many
/// rounds `retries` allows, how close `retry_tolerance` demands the probed
/// points to be, and the "Probed points range" reports between rounds.
pub struct RetryHelper {
    /// The printer, to reach `gcode` for reports and errors.
    printer: Weak<Printer>,
    default_max_retries: i64,
    default_retry_tolerance: f64,
    /// Upstream's `value_label` — every range message names it.
    value_label: &'static str,
    /// Upstream's `error_msg_extra`, appended to the increase-abort message.
    error_msg_extra: String,
    /// What `start` armed for the running command.
    state: Mutex<RetryState>,
}

/// One `start`'s worth of retry bookkeeping (`z_tilt.py:93-101`).
#[derive(Default)]
struct RetryState {
    max_retries: i64,
    retry_tolerance: f64,
    current_retry: i64,
    previous: Option<f64>,
    increasing: i32,
}

impl RetryHelper {
    /// Read `retries` and `retry_tolerance` (`z_tilt.py:88-90`).
    ///
    /// # Errors
    /// When a bound is violated: `retries` has `minval=0`,
    /// `retry_tolerance` `above=0.`.
    pub fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        error_msg_extra: &str,
    ) -> Result<Self, ConfigError> {
        let default_max_retries = config.get_int_bounded("retries", Some(0), Some(0), None)?;
        // Upstream's `above=0.` bounds the configured value only — and the
        // default `0.` sits exactly on that bound, so a bounds check that
        // also sees defaults would reject every section that does not set
        // `retry_tolerance` (upstream's `_get_wrapper` returns the default
        // before the parser's bounds run).
        let default_retry_tolerance = if config.has("retry_tolerance") {
            config.get_float_bounded("retry_tolerance", Some(0.), None, None, Some(0.), None)?
        } else {
            0.
        };
        Ok(Self {
            printer: Arc::downgrade(printer),
            default_max_retries,
            default_retry_tolerance,
            value_label: "Probed points range",
            error_msg_extra: error_msg_extra.to_string(),
            state: Mutex::new(RetryState::default()),
        })
    }

    /// Arm the retry helper for a command round (`z_tilt.py:94-103`):
    /// `RETRIES` (`minval=0, maxval=30`) and `RETRY_TOLERANCE`
    /// (`minval=0., maxval=1.`) override the configured defaults, and the
    /// counters start over.
    ///
    /// # Errors
    /// As [`GcodeCommand::get`], with upstream's bounds.
    pub fn start(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let max_retries = gcmd.get(
            "RETRIES",
            Some(self.default_max_retries),
            |value| value.parse::<i64>().ok(),
            Some(0),
            Some(30),
            None,
            None,
        )?;
        let retry_tolerance = gcmd.get(
            "RETRY_TOLERANCE",
            Some(self.default_retry_tolerance),
            parse_float,
            Some(0.0),
            Some(1.0),
            None,
            None,
        )?;
        *self.state.lock().unwrap_or_else(|p| p.into_inner()) = RetryState {
            max_retries,
            retry_tolerance,
            current_retry: 0,
            previous: None,
            increasing: 0,
        };
        Ok(())
    }

    /// Compare the round's range against the tolerance
    /// (`z_tilt.py:113-125`): `"done"` when it fits or no retries are
    /// allowed, `"retry"` for another round, an error when the range keeps
    /// rising or the retries run out.
    ///
    /// # Errors
    /// "Retries aborting: … is increasing. …" on a rising range (reported
    /// once it rose twice in a row without relief) or "Too many retries"
    /// past `RETRIES`.
    pub fn check_retry(&self, z_positions: &[f64]) -> Result<&'static str, CommandError> {
        let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        if state.max_retries == 0 {
            return Ok("done");
        }
        let error = (z_positions
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max)
            - z_positions.iter().copied().fold(f64::INFINITY, f64::min))
            * 1e6;
        let error = error.round() / 1e6;
        if let Some(printer) = self.printer.upgrade() {
            if let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) {
                gcode.respond_info(
                    &format!(
                        "Retries: {}/{} {}: {:.6} tolerance: {:.6}",
                        state.current_retry,
                        state.max_retries,
                        self.value_label,
                        error,
                        state.retry_tolerance
                    ),
                    true,
                );
            }
        }
        if state.check_increase(error) {
            return Err(CommandError::new(format!(
                "Retries aborting: {} is increasing. {}",
                self.value_label, self.error_msg_extra
            )));
        }
        if error <= state.retry_tolerance {
            return Ok("done");
        }
        state.current_retry += 1;
        if state.current_retry > state.max_retries {
            return Err(CommandError::new("Too many retries"));
        }
        Ok("retry")
    }
}

/// The `KEY` names [`RetryHelper::start`] reads, in read order
/// (`z_tilt.py:93-101`).
pub(crate) const RETRY_PARAMS: &[&str] = &["RETRIES", "RETRY_TOLERANCE"];

/// The declared parameters of a retry-driven probe command
/// (`Z_TILT_ADJUST`, `QUAD_GANTRY_LEVEL`): the retries the helper reads
/// first, then the probe round's.
pub(crate) fn retry_probe_params() -> Vec<&'static str> {
    let mut params = RETRY_PARAMS.to_vec();
    params.extend(probe_points_params());
    params
}

impl RetryState {
    /// Track a rising range (`z_tilt.py:104-112`): two increases without a
    /// relief in between abort the retries. Upstream tests `self.previous`
    /// for truth, so a stored `0.0` counts as "no previous round" — the same
    /// here.
    fn check_increase(&mut self, error: f64) -> bool {
        let rose = matches!(self.previous, Some(previous) if previous != 0.0)
            && error > self.previous.unwrap_or_default() + 0.0000001;
        if rose {
            self.increasing += 1;
        } else if self.increasing > 0 {
            self.increasing -= 1;
        }
        self.previous = Some(error);
        self.increasing > 1
    }
}

// ===========================================================================
// ZAdjustStatus (z_tilt.py:69-83)
// ===========================================================================

/// The `applied` flag of `get_status`: whether the last round finished and
/// its adjustments were applied. Reset by the command before a new round and
/// whenever the Z motors are switched off (`stepper_enable:motor_off`), which
/// makes the applied adjustments stale.
pub struct ZAdjustStatus {
    applied: AtomicBool,
}

impl ZAdjustStatus {
    /// The status, wired to the motor-off event (`z_tilt.py:72`).
    pub fn new(printer: &Arc<Printer>) -> Arc<Self> {
        let status = Arc::new(Self {
            applied: AtomicBool::new(false),
        });
        let handler = Arc::clone(&status);
        printer.register_event_handler(
            KlippyEvent::StepperEnableMotorOff,
            Box::new(move |_| handler.reset()),
        );
        status
    }

    /// Clear `applied` (`z_tilt.py:78-79`).
    pub fn reset(&self) {
        self.applied.store(false, Ordering::SeqCst);
    }

    /// Record a finished round: `"done"` sets `applied`
    /// (`z_tilt.py:74-77`); the result passes through unchanged.
    pub fn check_retry_result(&self, retry_result: &'static str) -> &'static str {
        if retry_result == "done" {
            self.applied.store(true, Ordering::SeqCst);
        }
        retry_result
    }

    /// The status dict (`z_tilt.py:80-81`).
    pub fn get_status(&self, _eventtime: f64) -> Value {
        json!({ "applied": self.applied.load(Ordering::SeqCst) })
    }
}

// ===========================================================================
// ZAdjustHelper (z_tilt.py:10-67)
// ===========================================================================

/// The Z motors a round adjusts: their count is checked against the machine
/// at connect, and [`ZAdjustHelper::adjust_steppers`] moves them one at a
/// time by taking each off the trapq, walking the toolhead through the
/// sorted offsets, and putting the motors back.
///
/// The toolhead side effects are driven through [`Adjust`] so the walk's
/// order is testable without a machine (the `probe.rs` `RoundOps` pattern).

/// The toolhead/g-code effects `adjust_steppers` performs, one per upstream
/// step (`z_tilt.py:29-67`).
trait Adjust {
    fn position(&self) -> Result<Coord, CommandError>;
    fn respond_info(&self, message: &str);
    async fn flush(&self) -> Result<(), CommandError>;
    fn detach(&self, stepper: &str) -> Result<(), CommandError>;
    fn attach(&self, stepper: &str) -> Result<(), CommandError>;
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError>;
    async fn set_position(&self, position: Coord) -> Result<(), CommandError>;
}

/// The count validation of `ZAdjustHelper.handle_connect`
/// (`z_tilt.py:19-27`).
///
/// # Errors
/// "\<name\> z_positions needs exactly \<n\> items" when `z_positions` lists a
/// different number of motors than the machine has Z steppers, and
/// "\<name\> requires multiple z steppers" for a single motor.
fn check_z_steppers(name: &str, z_count: usize, z_steppers: &[String]) -> Result<(), ConfigError> {
    if z_steppers.len() != z_count {
        return Err(ConfigError::new(format!(
            "{name} z_positions needs exactly {} items",
            z_steppers.len()
        )));
    }
    if z_steppers.len() < 2 {
        return Err(ConfigError::new(format!(
            "{name} requires multiple z steppers"
        )));
    }
    Ok(())
}

/// Upstream's `ZAdjustHelper`: the section's Z motor count, checked at
/// connect, and the one-motor-at-a-time adjustment walk.
pub struct ZAdjustHelper {
    printer: Weak<Printer>,
    /// The section name, for the connect-time errors.
    name: String,
    /// How many rows `z_positions` (or QGL's fixed 4) promised.
    z_count: usize,
    /// The Z steppers in config order, filled at connect
    /// (`z_stepper_names`, this port's `is_active_axis('z')` list).
    z_steppers: Mutex<Vec<String>>,
}

impl ZAdjustHelper {
    /// The helper for `z_count` motors (`z_tilt.py:14-18`).
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>, z_count: usize) -> Self {
        Self {
            printer: Arc::downgrade(printer),
            name: config.identifier(),
            z_count,
            z_steppers: Mutex::new(Vec::new()),
        }
    }

    /// Count the machine's Z steppers and check the promised count
    /// (`z_tilt.py:19-27`, upstream's `klippy:connect` handler).
    ///
    /// # Errors
    /// As [`check_z_steppers`]; "Printer is not ready" should not occur —
    /// the toolhead is registered before any object connects.
    pub fn handle_connect(&self) -> Result<(), KlippyError> {
        let config_error = |message: String| KlippyError::Config(ConfigError::new(message));
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| config_error("Printer is not ready".to_string()))?;
        let toolhead = printer
            .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)
            .ok_or_else(|| config_error("toolhead is not registered".to_string()))?;
        let z_steppers = toolhead.z_stepper_names();
        check_z_steppers(&self.name, self.z_count, &z_steppers).map_err(KlippyError::Config)?;
        *self.z_steppers.lock().unwrap_or_else(|p| p.into_inner()) = z_steppers;
        Ok(())
    }

    /// Move each Z motor by its adjustment (`z_tilt.py:29-67`).
    ///
    /// # Errors
    /// "Printer is not ready" before connect or from a lookup, and whatever
    /// the toolhead reports mid-walk (after which every Z motor is reattached
    /// so the machine is not left with detached motors).
    pub async fn adjust_steppers(
        &self,
        adjustments: &[f64],
        speed: f64,
    ) -> Result<(), CommandError> {
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let toolhead = printer
            .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let z_steppers = self
            .z_steppers
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        let ops = LiveAdjust { toolhead, gcode };
        run_adjust(&ops, &z_steppers, adjustments, speed).await
    }
}

/// The [`Adjust`] wiring against the real toolhead (`z_tilt.py:29-67`).
struct LiveAdjust {
    toolhead: Arc<ToolHeadObject>,
    gcode: Arc<GCodeDispatch>,
}

impl Adjust for LiveAdjust {
    fn position(&self) -> Result<Coord, CommandError> {
        self.toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    fn respond_info(&self, message: &str) {
        self.gcode.respond_info(message, true);
    }

    async fn flush(&self) -> Result<(), CommandError> {
        self.toolhead.flush_step_generation().await
    }

    fn detach(&self, stepper: &str) -> Result<(), CommandError> {
        self.toolhead.set_stepper_trapq(stepper, None)
    }

    fn attach(&self, stepper: &str) -> Result<(), CommandError> {
        let trapq = self
            .toolhead
            .main_trapq()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        self.toolhead.set_stepper_trapq(stepper, Some(trapq))
    }

    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        self.toolhead.move_to(position, speed)
    }

    async fn set_position(&self, position: Coord) -> Result<(), CommandError> {
        self.toolhead.set_position(position, &[]).await
    }
}

/// The adjustment walk itself (`z_tilt.py:29-67`), against any [`Adjust`]:
/// report the moves, take every Z motor off the trapq, then reattach them in
/// adjustment order — sorted by `-adjustment`, so the lowest target first —
/// walking the toolhead's Z between the sorted offsets and finishing with
/// every motor reattached and the toolhead at `z_low + first offset` plus
/// that first offset again.
async fn run_adjust<O: Adjust + ?Sized>(
    ops: &O,
    z_steppers: &[String],
    adjustments: &[f64],
    speed: f64,
) -> Result<(), CommandError> {
    let mut curpos = ops.position()?;

    // Report on movements (`z_tilt.py:32-35`).
    let stepstrs: Vec<String> = z_steppers
        .iter()
        .zip(adjustments)
        .map(|(name, adjustment)| format!("{name} = {adjustment:.6}"))
        .collect();
    ops.respond_info(&format!(
        "Making the following Z adjustments:\n{}",
        stepstrs.join("\n")
    ));

    // Disable Z stepper movements (`z_tilt.py:37-39`).
    ops.flush().await?;
    for stepper in z_steppers {
        ops.detach(stepper)?;
    }

    // Move each z stepper (sorted from lowest to highest) until they match
    // (`z_tilt.py:41-46`): the pairs sort by `-adjustment`, upstream's
    // `sorted(..., key=lambda k: k[0])` on `(-a, stepper)` — a stable sort,
    // which ties keep in stepper order.
    let mut positions: Vec<(f64, usize)> = adjustments
        .iter()
        .enumerate()
        .map(|(index, adjustment)| (-adjustment, index))
        .collect();
    positions.sort_by(|left, right| left.0.total_cmp(&right.0));
    let first_stepper_offset = positions[0].0;
    let z_low = curpos.z() - first_stepper_offset;
    for window in positions.windows(2) {
        let (_, stepper) = window[0];
        let (next_stepper_offset, _) = window[1];
        let stepper = &z_steppers[stepper];
        ops.flush().await?;
        ops.attach(stepper)?;
        curpos.set_axis(Z_AXIS, z_low + next_stepper_offset);
        let result = async {
            ops.move_to(curpos, speed)?;
            ops.set_position(curpos).await
        }
        .await;
        if let Err(err) = result {
            // `z_tilt.py:54-61`: flush and put every Z motor back before
            // re-raising, so a failed walk never leaves a detached motor.
            let _ = ops.flush().await;
            for stepper in z_steppers {
                let _ = ops.attach(stepper);
            }
            return Err(err);
        }
    }

    // Z should now be level — do final cleanup (`z_tilt.py:63-67`).
    let last_stepper = &z_steppers[positions[positions.len() - 1].1];
    ops.flush().await?;
    ops.attach(last_stepper)?;
    curpos.set_axis(Z_AXIS, curpos.z() + first_stepper_offset);
    ops.set_position(curpos).await?;
    Ok(())
}

// ===========================================================================
// ZTilt
// ===========================================================================

/// The `[z_tilt]` section: probe points, the plane fit behind
/// `Z_TILT_ADJUST`, and its status (`z_tilt.py:127-170`).
pub struct ZTilt {
    /// The configured motor positions, `z_positions` rows.
    z_positions: Vec<(f64, f64)>,
    retry_helper: RetryHelper,
    /// Wired after construction: the probe helper's finalize callback looks
    /// the section object up through the registry to reach this helper (for
    /// `get_lift_speed`), so the helper cannot be a field built before the
    /// callback exists.
    probe_helper: OnceLock<Arc<ProbePointsHelper>>,
    z_status: Arc<ZAdjustStatus>,
    z_helper: ZAdjustHelper,
    /// The first finalize error, for the command to report (`None` in the
    /// manual round, whose errors can only be logged — see [`ZTilt::probe_finalize`]).
    last_error: Mutex<Option<CommandError>>,
}

impl ZTilt {
    /// Read the section, wire the probe helper's callback, and register
    /// `Z_TILT_ADJUST` (`z_tilt.py:128-144`).
    ///
    /// # Errors
    /// A malformed option, fewer than two probe points
    /// (`probe_helper.minimum_points(2)`), or a command name that is taken.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        let z_positions = read_xy_option(config, "z_positions")?;
        let retry_helper = RetryHelper::new(config, printer, "")?;
        let z_status = ZAdjustStatus::new(printer);
        let z_helper = ZAdjustHelper::new(config, printer, z_positions.len());

        let z_tilt = Arc::new(Self {
            z_positions,
            retry_helper,
            probe_helper: OnceLock::new(),
            z_status,
            z_helper,
            last_error: Mutex::new(None),
        });

        // The callback reaches the section through the registry (it is
        // registered the moment this factory returns, before any command can
        // run), which is what breaks the helper-constructs-callback circle.
        let printer_weak = Arc::downgrade(printer);
        let finalize: ProbePointsFinalize = Arc::new(move |offsets, positions| {
            let Some(printer) = printer_weak.upgrade() else {
                return None;
            };
            let Some(z_tilt) = printer.lookup_object_as::<ZTilt>(Z_TILT_OBJECT) else {
                warn!("Z_TILT_ADJUST finalize: the z_tilt object is gone");
                return None;
            };
            z_tilt.probe_finalize(offsets, positions)
        });
        let probe_helper = ProbePointsHelper::new(config, printer, finalize)?;
        probe_helper.minimum_points(2)?;
        z_tilt
            .probe_helper
            .set(Arc::clone(&probe_helper))
            .unwrap_or_else(|_| unreachable!("the probe helper is wired once"));

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");
        let this = Arc::downgrade(&z_tilt);
        let handler: CommandHandler = Arc::new(move |gcmd| {
            let this = this.clone();
            Box::pin(async move {
                let this = this
                    .upgrade()
                    .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                this.cmd_z_tilt_adjust(gcmd).await
            })
        });
        gcode
            .register_command_with_params(
                "Z_TILT_ADJUST",
                handler,
                Some("Adjust the Z tilt"),
                &retry_probe_params(),
                false,
            )
            .map_err(ConfigError::new)?;

        Ok(z_tilt)
    }

    /// `Z_TILT_ADJUST` (`z_tilt.py:139-144`): clear the applied flag, arm the
    /// retries, probe every point.
    ///
    /// # Errors
    /// A bad command parameter, whatever the round reports — including the
    /// first error the finalize callback recorded (an automatic round's
    /// callback cannot raise into the loop itself; see [`ZTilt::probe_finalize`]).
    async fn cmd_z_tilt_adjust(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        self.z_status.reset();
        self.retry_helper.start(gcmd)?;
        *self.last_error.lock().unwrap_or_else(|p| p.into_inner()) = None;
        let probe_helper = self
            .probe_helper
            .get()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        probe_helper.start_probe(gcmd).await?;
        if let Some(error) = self
            .last_error
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take()
        {
            return Err(error);
        }
        Ok(())
    }

    /// The probe helper's finalize callback: on error, record the first one
    /// for [`ZTilt::cmd_z_tilt_adjust`] to report and end the round
    /// (`"done"`), because this port's callback signature cannot raise into
    /// the probing loop the way upstream's exception does. The manual round
    /// finishes outside any `Z_TILT_ADJUST` invocation, so its error is only
    /// logged here — a divergence worth remembering.
    fn probe_finalize(&self, offsets: ProbeOffsets, positions: &[Coord]) -> Option<&'static str> {
        match self.run_finalize(offsets, positions) {
            Ok(result) => (result == RETRY).then_some(RETRY),
            Err(error) => {
                warn!("Z_TILT_ADJUST: {error}");
                let mut slot = self.last_error.lock().unwrap_or_else(|p| p.into_inner());
                if slot.is_none() {
                    *slot = Some(error);
                }
                None
            }
        }
    }

    /// The finalize work (`z_tilt.py:146-173`): fit the bed plane, move the
    /// motors, then run the retry check over the raw probed Z — unlike QGL,
    /// upstream compares the positions as probed here.
    fn run_finalize(
        &self,
        offsets: ProbeOffsets,
        positions: &[Coord],
    ) -> Result<&'static str, CommandError> {
        let probe_helper = self
            .probe_helper
            .get()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let adjustments = compute_adjustments(&offsets, positions, &self.z_positions);
        let speed = probe_helper.get_lift_speed();
        block_in_command(self.z_helper.adjust_steppers(&adjustments, speed))?;
        let z_range: Vec<f64> = positions.iter().map(Coord::z).collect();
        let result = self.retry_helper.check_retry(&z_range)?;
        Ok(self.z_status.check_retry_result(result))
    }
}

/// The motor adjustments for a probed plane (`z_tilt.py:146-173`): fit
/// `z = x*x_adjust + y*y_adjust + z_adjust` to the probed positions by
/// coordinate descent (starting at `x = y = 0`, `z = offsets.z`), take the
/// plane's height at each motor's `z_positions` coordinate, and net the
/// probe's own offsets out of the intercept.
fn compute_adjustments(
    offsets: &ProbeOffsets,
    positions: &[Coord],
    z_positions: &[(f64, f64)],
) -> Vec<f64> {
    let z_offset = offsets.z;
    let mut params = [0.0, 0.0, z_offset];
    coordinate_descent(&mut params, |params| {
        let [x_adjust, y_adjust, z_adjust] = *params else {
            return 0.0;
        };
        positions
            .iter()
            .map(|position| {
                let height =
                    position.z() - position.x() * x_adjust - position.y() * y_adjust - z_adjust;
                height * height
            })
            .sum()
    });
    let [x_adjust, y_adjust, z_adjust] = params;
    let z_adjust = z_adjust - z_offset - x_adjust * offsets.x - y_adjust * offsets.y;
    z_positions
        .iter()
        .map(|(x, y)| x * x_adjust + y * y_adjust + z_adjust)
        .collect()
}

impl PrinterObject for ZTilt {
    fn get_status(&self, eventtime: f64) -> Value {
        self.z_status.get_status(eventtime)
    }

    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move { self.z_helper.handle_connect() })
    }
}

/// The factory `section!` names (`z_tilt.py:load_config`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = ZTilt::new(config, printer)?;
    Ok(object as Arc<dyn PrinterObject>)
}

// ===========================================================================
// z_tilt tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{ConfigSection, ConfigValue};
    use crate::core::klippy::reactor::ManualReactor;
    use std::collections::BTreeSet;

    /// Stepper names as the helper holds them.
    fn names(items: &[&str]) -> Vec<String> {
        items.iter().map(|name| name.to_string()).collect()
    }

    /// A section with the given options, as the parser would build it.
    fn section(id: &str, options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new(id, None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A printer with `gcode` registered, so `RetryHelper` can report.
    fn gcode_printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .expect("gcode registers");
        printer
    }

    /// Everything `gcode` reported through `respond_info`, one entry per
    /// line (`// …` prefixes included, as a client sees them).
    fn captured_lines(printer: &Arc<Printer>) -> Arc<Mutex<Vec<String>>> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        let lines = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&lines);
        gcode.register_output_handler(Arc::new(move |line: &str| {
            sink.lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(line.to_string());
        }));
        lines
    }

    fn retry_helper(
        printer: &Arc<Printer>,
        id: &str,
        options: &[(&str, &str)],
        error_msg_extra: &str,
    ) -> RetryHelper {
        let section = section(id, options);
        let config = ConfigWrapper::untracked(&section);
        RetryHelper::new(&config, printer, error_msg_extra).expect("the options read")
    }

    fn gcmd(printer: &Arc<Printer>) -> GcodeCommand {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("gcode is registered");
        gcode.create_gcode_command(
            "Z_TILT_ADJUST",
            "Z_TILT_ADJUST",
            std::collections::HashMap::new(),
        )
    }

    /// `z_positions` item count vs. the machine (`z_tilt.py:22-27`): the
    /// message names the section and the *machine's* stepper count, and a
    /// single stepper is refused even when the counts match.
    #[test]
    fn z_positions_item_count_messages_match_upstream() {
        let three = ["stepper_z", "stepper_z1", "stepper_z2"];
        let three: Vec<String> = three.iter().map(|name| name.to_string()).collect();
        let err = check_z_steppers("z_tilt", 2, &three).unwrap_err();
        assert_eq!(err.to_string(), "z_tilt z_positions needs exactly 3 items");
        let err = check_z_steppers("quad_gantry_level", 4, &three).unwrap_err();
        assert_eq!(
            err.to_string(),
            "quad_gantry_level z_positions needs exactly 3 items"
        );

        let one = ["stepper_z".to_string()];
        let err = check_z_steppers("z_tilt", 1, &one).unwrap_err();
        assert_eq!(err.to_string(), "z_tilt requires multiple z steppers");

        assert!(check_z_steppers("z_tilt", 3, &three).is_ok());
    }

    /// A missing `z_positions` is reported the way upstream's `getlists`
    /// reports a missing option, and a row that is not two numbers keeps
    /// `get_list_of_lists`' wording.
    #[test]
    fn missing_or_malformed_z_positions_reports_upstream_wording() {
        let bare = section("z_tilt", &[]);
        let config = ConfigWrapper::untracked(&bare);
        let err = read_xy_option(&config, "z_positions").unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'z_positions' in section 'z_tilt' is not defined"
        );

        let bad = section("z_tilt", &[("z_positions", "1, 2, 3")]);
        let config = ConfigWrapper::untracked(&bad);
        let err = read_xy_option(&config, "z_positions").unwrap_err();
        assert_eq!(
            err.to_string(),
            "Option 'z_positions' in section 'z_tilt' must have 2 elements"
        );
    }

    /// `ZTilt::new` refuses a section with fewer than two probe points,
    /// before any command is registered (`z_tilt.py:134`).
    #[test]
    fn z_tilt_needs_at_least_two_probe_points() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let section = section(
            "z_tilt",
            &[("z_positions", "-55,-7\n305,320"), ("points", "50,50")],
        );
        let config = ConfigWrapper::untracked(&section);
        let Err(err) = ZTilt::new(&config, &printer) else {
            panic!("one probe point must not load");
        };
        assert_eq!(err.to_string(), "Need at least 2 probe points for z_tilt");
    }

    /// `check_retry` reports the round's range before judging it, and a
    /// range inside the tolerance ends the retries (`z_tilt.py:113-121`).
    #[test]
    fn retry_helper_reports_the_range_and_finishes_within_tolerance() {
        let printer = gcode_printer();
        let lines = captured_lines(&printer);
        let retry = retry_helper(
            &printer,
            "z_tilt",
            &[("retries", "3"), ("retry_tolerance", "0.01")],
            "",
        );
        let gcmd = gcmd(&printer);
        retry.start(&gcmd).expect("the defaults arm");

        let result = retry
            .check_retry(&[1.0, 1.02, 1.01])
            .expect("a range this wide retries");
        assert_eq!(result, "retry");
        let lines = lines.lock().unwrap_or_else(|p| p.into_inner());
        assert!(
            lines.iter().any(|line| line
                .contains("Retries: 0/3 Probed points range: 0.020000 tolerance: 0.010000")),
            "the range report is missing: {lines:?}"
        );
        drop(lines);

        let result = retry
            .check_retry(&[1.0, 1.005, 1.004])
            .expect("a range this narrow is done");
        assert_eq!(result, "done");
    }

    /// One round past `RETRIES` aborts with upstream's message
    /// (`z_tilt.py:123-124`).
    #[test]
    fn retry_helper_stops_after_the_retry_limit() {
        let printer = gcode_printer();
        let retry = retry_helper(&printer, "z_tilt", &[("retries", "1")], "");
        let gcmd = gcmd(&printer);
        retry.start(&gcmd).expect("the defaults arm");

        assert_eq!(
            retry.check_retry(&[0.0, 0.02]).expect("first round"),
            "retry"
        );
        let err = retry.check_retry(&[0.0, 0.02]).unwrap_err();
        assert_eq!(err.to_string(), "Too many retries");
    }

    /// A range that rises twice without relief aborts with upstream's
    /// message (`z_tilt.py:117-119`) — trailing space included,
    /// because upstream's `error_msg_extra` for `z_tilt` is empty.
    #[test]
    fn retry_helper_aborts_a_rising_range() {
        let printer = gcode_printer();
        let retry = retry_helper(
            &printer,
            "z_tilt",
            &[("retries", "5"), ("retry_tolerance", "0.001")],
            "",
        );
        let gcmd = gcmd(&printer);
        retry.start(&gcmd).expect("the defaults arm");

        assert_eq!(
            retry.check_retry(&[0.0, 0.01]).expect("first round"),
            "retry"
        );
        assert_eq!(
            retry.check_retry(&[0.0, 0.02]).expect("second round"),
            "retry"
        );
        let err = retry.check_retry(&[0.0, 0.03]).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Retries aborting: Probed points range is increasing. "
        );
    }

    /// The same abort from QGL's helper appends its `error_msg_extra`
    /// (`quad_gantry_level.py:30-31`).
    #[test]
    fn retry_helper_appends_the_error_msg_extra() {
        let printer = gcode_printer();
        let retry = retry_helper(
            &printer,
            "quad_gantry_level",
            &[("retries", "5")],
            "Possibly Z motor numbering is wrong",
        );
        let gcmd = gcmd(&printer);
        retry.start(&gcmd).expect("the defaults arm");

        assert_eq!(
            retry.check_retry(&[0.0, 0.01]).expect("first round"),
            "retry"
        );
        assert_eq!(
            retry.check_retry(&[0.0, 0.02]).expect("second round"),
            "retry"
        );
        let err = retry.check_retry(&[0.0, 0.03]).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Retries aborting: Probed points range is increasing. \
             Possibly Z motor numbering is wrong"
        );
    }

    /// `retries: 0` answers `"done"` without a report and without touching
    /// the counters (`z_tilt.py:113-115`).
    #[test]
    fn retry_helper_without_retries_is_silent() {
        let printer = gcode_printer();
        let lines = captured_lines(&printer);
        let retry = retry_helper(&printer, "z_tilt", &[], "");
        let gcmd = gcmd(&printer);
        retry.start(&gcmd).expect("the defaults arm");

        assert_eq!(
            retry
                .check_retry(&[0.0, 10.0])
                .expect("no retries means done"),
            "done"
        );
        assert!(lines.lock().unwrap_or_else(|p| p.into_inner()).is_empty());
    }

    /// `applied` follows the round and clears when the motors go off
    /// (`z_tilt.py:69-83`).
    #[test]
    fn z_adjust_status_tracks_applied_and_motor_off() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let status = ZAdjustStatus::new(&printer);
        assert_eq!(status.get_status(0.0), json!({ "applied": false }));

        assert_eq!(status.check_retry_result("done"), "done");
        assert_eq!(status.get_status(0.0), json!({ "applied": true }));
        assert_eq!(status.check_retry_result("retry"), "retry");
        assert_eq!(status.get_status(0.0), json!({ "applied": true }));

        printer.send_event(&KlippyEvent::StepperEnableMotorOff);
        assert_eq!(status.get_status(0.0), json!({ "applied": false }));
    }

    /// The plane fit recovers a known plane from probed points and maps it
    /// onto the motors (`z_tilt.py:146-173`): the probe's own offsets net
    /// out of the intercept, each motor gets the plane's height at its
    /// `z_positions` coordinate, shifted by the probe z_offset.
    #[test]
    fn compute_adjustments_recovers_the_known_plane() {
        let (a, b, c) = (0.01, -0.02, 2.0);
        let offsets = ProbeOffsets {
            x: 0.5,
            y: -0.25,
            z: 1.5,
        };
        // Probed points as the toolhead reports them: its position at the
        // trigger, with Z the bed height under the probe (nozzle + offsets).
        let nozzle = [
            (0.0, 0.0),
            (0.0, 20.0),
            (20.0, 0.0),
            (20.0, 20.0),
            (10.0, 10.0),
        ];
        let positions: Vec<Coord> = nozzle
            .iter()
            .map(|(x, y)| Coord::new(*x, *y, c + a * (x + offsets.x) + b * (y + offsets.y), 0.0))
            .collect();
        let z_positions = [(0.0, 0.0), (0.0, 20.0), (20.0, 0.0), (20.0, 20.0)];

        let adjustments = compute_adjustments(&offsets, &positions, &z_positions);

        for (motor, adjustment) in z_positions.iter().zip(&adjustments) {
            // The fit absorbs the probe offsets into its intercept;
            // netting them back out (z_tilt.py:165-168) leaves the plane's
            // height at the motor, shifted by the probe's z_offset.
            let expected = c + a * motor.0 + b * motor.1 - offsets.z;
            assert!(
                (adjustment - expected).abs() < 1e-4,
                "motor at {motor:?}: {adjustment} vs {expected}"
            );
        }
    }

    /// A recording [`Adjust`] fake: every effect lands in `log`, and
    /// `attached` is the set of motors currently on a trapq.
    #[derive(Default)]
    struct RecordingAdjust {
        position: Mutex<Coord>,
        attached: Mutex<BTreeSet<String>>,
        log: Mutex<Vec<String>>,
        /// Fail the first `move_to`, to walk the recovery path.
        fail_move: std::sync::atomic::AtomicBool,
    }

    impl RecordingAdjust {
        fn new(steppers: &[&str], z: f64) -> Self {
            Self {
                position: Mutex::new(Coord::new(10.0, 10.0, z, 0.0)),
                attached: Mutex::new(
                    steppers
                        .iter()
                        .map(|name| name.to_string())
                        .collect::<BTreeSet<_>>(),
                ),
                log: Mutex::new(Vec::new()),
                fail_move: std::sync::atomic::AtomicBool::new(false),
            }
        }

        fn log(&self) -> Vec<String> {
            self.log.lock().unwrap_or_else(|p| p.into_inner()).clone()
        }

        fn attached(&self) -> Vec<String> {
            self.attached
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .iter()
                .cloned()
                .collect()
        }

        fn push(&self, entry: String) {
            self.log
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(entry);
        }
    }

    impl Adjust for RecordingAdjust {
        fn position(&self) -> Result<Coord, CommandError> {
            Ok(*self.position.lock().unwrap_or_else(|p| p.into_inner()))
        }

        fn respond_info(&self, message: &str) {
            self.push(format!("respond {message}"));
        }

        async fn flush(&self) -> Result<(), CommandError> {
            self.push("flush".to_string());
            Ok(())
        }

        fn detach(&self, stepper: &str) -> Result<(), CommandError> {
            self.push(format!("detach {stepper}"));
            self.attached
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .remove(stepper);
            Ok(())
        }

        fn attach(&self, stepper: &str) -> Result<(), CommandError> {
            self.push(format!("attach {stepper}"));
            self.attached
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .insert(stepper.to_string());
            Ok(())
        }

        fn move_to(&self, position: Coord, _speed: f64) -> Result<(), CommandError> {
            if self.fail_move.load(Ordering::SeqCst) {
                return Err(CommandError::new("move refused"));
            }
            self.push(format!("move z={:.6}", position.z()));
            *self.position.lock().unwrap_or_else(|p| p.into_inner()) = position;
            Ok(())
        }

        async fn set_position(&self, position: Coord) -> Result<(), CommandError> {
            self.push(format!("set z={:.6}", position.z()));
            *self.position.lock().unwrap_or_else(|p| p.into_inner()) = position;
            Ok(())
        }
    }

    /// The walk reports the moves, takes every motor off, then reattaches
    /// them in `-adjustment` order — flushing before each step — and ends
    /// with every motor back on the trapq (`z_tilt.py:29-67`).
    #[tokio::test]
    async fn adjust_steppers_walks_the_sorted_offsets_and_reattaches() {
        let steppers = ["stepper_z", "stepper_z1", "stepper_z2"];
        let ops = RecordingAdjust::new(&steppers, 1.0);
        // -adjustments sort to: stepper_z2 (-0.06), stepper_z (-0.04),
        // stepper_z1 (0.02); first offset -0.06, so z_low = 1.06.
        let adjustments = [0.04, -0.02, 0.06];

        run_adjust(&ops, &names(&steppers), &adjustments, 50.0)
            .await
            .expect("the walk succeeds");

        let expected: Vec<String> = [
            "respond Making the following Z adjustments:\n\
             stepper_z = 0.040000\n\
             stepper_z1 = -0.020000\n\
             stepper_z2 = 0.060000",
            "flush",
            "detach stepper_z",
            "detach stepper_z1",
            "detach stepper_z2",
            "flush",
            "attach stepper_z2",
            "move z=1.020000",
            "set z=1.020000",
            "flush",
            "attach stepper_z",
            "move z=1.080000",
            "set z=1.080000",
            "flush",
            "attach stepper_z1",
            "set z=1.020000",
        ]
        .iter()
        .map(|entry| entry.to_string())
        .collect();
        assert_eq!(ops.log(), expected);

        // Every motor is back on the trapq, and the toolhead sits at
        // z_low + last offset + first offset.
        assert_eq!(
            ops.attached(),
            names(&["stepper_z", "stepper_z1", "stepper_z2"])
        );
        let position = ops.position().expect("a position");
        assert!((position.z() - 1.02).abs() < 1e-9, "z = {}", position.z());
    }

    /// A failed move reattaches every motor before the error escapes
    /// (`z_tilt.py:54-61`).
    #[tokio::test]
    async fn adjust_steppers_reattaches_every_motor_after_a_failure() {
        let steppers = ["stepper_z", "stepper_z1"];
        let ops = RecordingAdjust::new(&steppers, 1.0);
        ops.fail_move.store(true, Ordering::SeqCst);

        let err = run_adjust(&ops, &names(&steppers), &[0.02, 0.01], 50.0)
            .await
            .unwrap_err();
        assert_eq!(err.to_string(), "move refused");
        assert_eq!(ops.attached(), names(&["stepper_z", "stepper_z1"]));
    }
}
