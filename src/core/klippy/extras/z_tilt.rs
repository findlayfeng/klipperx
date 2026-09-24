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
    ProbeOffsets, ProbePointsFinalize, ProbePointsHelper, RETRY,
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

/// One `start`'s worth of retry bookkeeping (`z_tilt.py:96-103`).
#[derive(Default)]
struct RetryState {
    max_retries: i64,
    retry_tolerance: f64,
    current_retry: i64,
    previous: Option<f64>,
    increasing: i32,
}

impl RetryHelper {
    /// Read `retries` and `retry_tolerance` (`z_tilt.py:88-93`).
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
        let default_retry_tolerance =
            config.get_float_bounded("retry_tolerance", Some(0.), None, None, Some(0.), None)?;
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

    /// Clear `applied` (`z_tilt.py:79-80`).
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

    /// The status dict (`z_tilt.py:81-82`).
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
/// `Z_TILT_ADJUST`, and its status (`z_tilt.py:126-176`).
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
            .register_command("Z_TILT_ADJUST", handler, Some("Adjust the Z tilt"), false)
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
