//! `manual_probe` — the interactive Z-height probe helper.
//!
//! Upstream `klippy/extras/manual_probe.py`: `MANUAL_PROBE` /
//! `Z_ENDSTOP_CALIBRATE` start a helper that registers `ACCEPT` / `NEXT` /
//! `ABORT` / `TESTZ` for the duration, reports the current and nearest tested Z
//! positions, and calls a finalizer when the user accepts or aborts.
//!
//! Deliberate simplifications, all listed in `TODO.md` H9:
//!
//! - the reported "kinematics position" is the toolhead's commanded position.
//!   Upstream asks the kinematics (`kin.calc_position(kin_spos)`) so linear
//!   deltas can differ from the commanded tower positions; that matters once
//!   `delta` lands (T5), and `position()` is the same number for cartesian.
//! - `Z_OFFSET_APPLY_ENDSTOP` / `Z_OFFSET_APPLY_DELTA_ENDSTOPS` are not
//!   implemented yet: they adjust `position_endstop` from the `gcode_move`
//!   Z offset, which needs the delta tower sections (T5) and belongs with the
//!   endstop-calibration group.

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::Coord;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("manual_probe", order = 30, load = load_config);

/// The toolhead object, as the loader registers `[printer]`.
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The `configfile` object, for the autosave write-back.
const CONFIGFILE_OBJECT: &str = "configfile";

/// The Z axis index, as [`Coord`] numbers them.
const Z_AXIS: usize = 2;

/// How far above the requested position the nozzle bobs before moving down
/// (`manual_probe.py:126`).
const Z_BOB_MINIMUM: f64 = 0.500;

/// The longest single `TESTZ` step (`manual_probe.py:127`).
const BISECT_MAX: f64 = 0.200;

/// What a finished manual probe reports back (`finalize_callback`).
pub(crate) type FinalizeCallback = Arc<dyn Fn(Option<Coord>) + Send + Sync>;

/// Where `bisect_left` would insert `value` in a sorted list.
fn bisect_left(values: &[f64], value: f64) -> usize {
    let mut lo = 0;
    let mut hi = values.len();
    while lo < hi {
        let mid = (lo + hi) / 2;
        if values[mid] < value {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    lo
}

/// The `[manual_probe]` section (`manual_probe.py:ManualProbe`).
pub struct ManualProbe {
    /// The machine, for the helper's toolhead and the autosave write-back.
    printer: Weak<Printer>,
    /// `[stepper_z] position_endstop`, when there is one: it is what
    /// `Z_ENDSTOP_CALIBRATE` writes back, and its absence is why that command
    /// is not registered on a printer without a Z endstop.
    z_position_endstop: Option<f64>,
    /// The status `get_status` reports and the helper updates.
    status: Arc<Mutex<Value>>,
}

impl ManualProbe {
    /// Register the commands the printer supports.
    ///
    /// # Errors
    /// When a command name is already taken.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let z_position_endstop = config.sibling("stepper_z").and_then(|sibling| {
            sibling
                .get_optional_float("position_endstop")
                .ok()
                .flatten()
        });
        let status = Arc::new(Mutex::new(reset_status()));

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");

        // MANUAL_PROBE: the plain helper, reporting the accepted Z.
        {
            let printer = Arc::downgrade(printer);
            let status = Arc::clone(&status);
            gcode
                .register_command(
                    "MANUAL_PROBE",
                    Arc::new(move |gcmd| {
                        let printer = printer.clone();
                        let status = Arc::clone(&status);
                        Box::pin(async move {
                            let printer = printer
                                .upgrade()
                                .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                            let report_printer = Arc::clone(&printer);
                            let callback: FinalizeCallback =
                                Arc::new(move |kin_pos: Option<Coord>| {
                                    if let Some(pos) = kin_pos {
                                        report(
                                            &report_printer,
                                            &format!("Z position is {:.3}", pos.z()),
                                        );
                                    }
                                });
                            ManualProbeHelper::start(&printer, gcmd, callback, status)?;
                            Ok(())
                        })
                    }),
                    Some("Start manual probe helper script"),
                    false,
                )
                .map_err(ConfigError::new)?;
        }

        // Z_ENDSTOP_CALIBRATE: same helper, writing `position_endstop` back.
        if let Some(z_position_endstop) = z_position_endstop {
            let printer = Arc::downgrade(printer);
            let status = Arc::clone(&status);
            gcode
                .register_command(
                    "Z_ENDSTOP_CALIBRATE",
                    Arc::new(move |gcmd| {
                        let printer = printer.clone();
                        let status = Arc::clone(&status);
                        Box::pin(async move {
                            let printer = printer
                                .upgrade()
                                .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                            let cb_printer = Arc::clone(&printer);
                            let callback: FinalizeCallback =
                                Arc::new(move |kin_pos: Option<Coord>| {
                                    let Some(pos) = kin_pos else { return };
                                    let z_pos = z_position_endstop - pos.z();
                                    report(
                                        &cb_printer,
                                        &format!(
                                            "stepper_z: position_endstop: {z_pos:.3}\n\
                                             The SAVE_CONFIG command will update the printer config file\n\
                                             with the above and restart the printer."
                                        ),
                                    );
                                    if let Some(configfile) = cb_printer
                                        .lookup_object_as::<crate::core::klippy::config::PrinterConfig>(
                                            CONFIGFILE_OBJECT,
                                        )
                                    {
                                        configfile.set(
                                            "stepper_z",
                                            "position_endstop",
                                            &format!("{z_pos:.3}"),
                                        );
                                    }
                                });
                            ManualProbeHelper::start(&printer, gcmd, callback, status)?;
                            Ok(())
                        })
                    }),
                    Some("Calibrate a Z endstop"),
                    false,
                )
                .map_err(ConfigError::new)?;
        }

        let _ = identifier;
        Ok(Self {
            printer: Arc::downgrade(printer),
            z_position_endstop,
            status,
        })
    }

    /// The `[stepper_z] position_endstop` this printer calibrates, if any.
    pub fn z_position_endstop(&self) -> Option<f64> {
        self.z_position_endstop
    }

    /// Start the interactive helper with a caller's finalizer
    /// (`probe.py:cmd_PROBE_CALIBRATE` ends its probe through this).
    ///
    /// # Errors
    /// "Already in a manual Z probe" when one is running, or a command name is
    /// taken by something else.
    pub(crate) fn start_helper(
        &self,
        printer: &Arc<Printer>,
        gcmd: &GcodeCommand,
        callback: FinalizeCallback,
    ) -> Result<(), CommandError> {
        ManualProbeHelper::start(printer, gcmd, callback, Arc::clone(&self.status))?;
        Ok(())
    }

    /// Refuse a second manual probe (`manual_probe.verify_no_manual_probe`).
    ///
    /// # Errors
    /// "Already in a manual Z probe. Use ABORT to abort it."
    pub(crate) fn verify_no_manual_probe(
        &self,
        printer: &Arc<Printer>,
    ) -> Result<(), CommandError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");
        verify_no_manual_probe(printer, &gcode)
    }
}

/// Emit an informational line from a callback that has no command in hand.
fn report(printer: &Arc<Printer>, message: &str) {
    if let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) {
        gcode.respond_info(message, true);
    }
}

/// The idle status (`manual_probe.py:53-61`).
fn reset_status() -> Value {
    json!({
        "is_active": false,
        "z_position": Value::Null,
        "z_position_lower": Value::Null,
        "z_position_upper": Value::Null,
    })
}

/// Verify that no manual probe is in progress (`manual_probe.py:117-127`).
///
/// Upstream probes by trying to register `ACCEPT`; this port has
/// [`GCodeDispatch::unregister_command`], so the probe is exact.
///
/// # Errors
/// "Already in a manual Z probe. Use ABORT to abort it."
pub(crate) fn verify_no_manual_probe(
    printer: &Arc<Printer>,
    gcode: &Arc<GCodeDispatch>,
) -> Result<(), CommandError> {
    let dummy: CommandHandler = Arc::new(|_gcmd| Box::pin(async { Ok(()) }));
    match gcode.register_command("ACCEPT", dummy, None, false) {
        Ok(()) => {
            gcode.unregister_command("ACCEPT");
            Ok(())
        }
        Err(_) => {
            let _ = printer;
            Err(CommandError::new(
                "Already in a manual Z probe. Use ABORT to abort it.",
            ))
        }
    }
}

/// The helper one manual probe runs (`manual_probe.py:ManualProbeHelper`).
struct ManualProbeHelper {
    /// The machine, for the toolhead.
    printer: Weak<Printer>,
    /// The `manual_probe` object's status, updated as the user moves.
    status: Arc<Mutex<Value>>,
    /// Called once when the probe ends (`ACCEPT` or `ABORT`).
    finalize_callback: FinalizeCallback,
    /// The `TESTZ` move speed (`SPEED`).
    speed: f64,
    /// Where the probe started, for `ACCEPT`'s sanity check.
    start_position: Coord,
    /// The Z positions already visited, kept sorted (`past_positions`).
    past_positions: Mutex<Vec<f64>>,
}

impl ManualProbeHelper {
    /// Register the helper's commands and report the starting position.
    ///
    /// # Errors
    /// "Already in a manual Z probe" when one is running, or a command name is
    /// taken by something else.
    fn start(
        printer: &Arc<Printer>,
        gcmd: &GcodeCommand,
        finalize_callback: FinalizeCallback,
        status: Arc<Mutex<Value>>,
    ) -> Result<Arc<Self>, CommandError> {
        let speed = gcmd.get_float_default("SPEED", 5.0)?;
        let toolhead = printer
            .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");
        verify_no_manual_probe(printer, &gcode)?;

        let start_position = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let helper = Arc::new(Self {
            printer: Arc::downgrade(printer),
            status,
            finalize_callback,
            speed,
            start_position,
            past_positions: Mutex::new(Vec::new()),
        });

        // ACCEPT / NEXT finish the probe, ABORT drops it, TESTZ moves.
        {
            let helper = Arc::clone(&helper);
            gcode
                .register_command(
                    "ACCEPT",
                    Arc::new(move |gcmd| {
                        let helper = Arc::clone(&helper);
                        Box::pin(async move { helper.cmd_accept(gcmd) })
                    }),
                    Some("Accept the current Z position"),
                    false,
                )
                .map_err(CommandError::new)?;
        }
        {
            let helper = Arc::clone(&helper);
            gcode
                .register_command(
                    "NEXT",
                    Arc::new(move |gcmd| {
                        let helper = Arc::clone(&helper);
                        Box::pin(async move { helper.cmd_accept(gcmd) })
                    }),
                    None,
                    false,
                )
                .map_err(CommandError::new)?;
        }
        {
            let helper = Arc::clone(&helper);
            gcode
                .register_command(
                    "ABORT",
                    Arc::new(move |gcmd| {
                        let helper = Arc::clone(&helper);
                        Box::pin(async move { helper.cmd_abort(gcmd) })
                    }),
                    Some("Abort manual Z probing tool"),
                    false,
                )
                .map_err(CommandError::new)?;
        }
        {
            let helper = Arc::clone(&helper);
            gcode
                .register_command(
                    "TESTZ",
                    Arc::new(move |gcmd| {
                        let helper = Arc::clone(&helper);
                        Box::pin(async move { helper.cmd_testz(gcmd) })
                    }),
                    Some("Move to new Z height"),
                    false,
                )
                .map_err(CommandError::new)?;
        }

        gcode.respond_info(
            "Starting manual Z probe. Use TESTZ to adjust position.\n\
             Finish with ACCEPT or ABORT command.",
            true,
        );
        helper.report_z_status(false, None);
        Ok(helper)
    }

    /// The toolhead.
    fn toolhead(&self) -> Result<Arc<ToolHeadObject>, CommandError> {
        self.printer
            .upgrade()
            .and_then(|printer| printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT))
            .ok_or_else(|| CommandError::new("Printer is not ready"))
    }

    /// The position the probe reports: the commanded toolhead position
    /// (upstream asks the kinematics; see the module note).
    fn kinematics_pos(&self) -> Coord {
        self.toolhead()
            .ok()
            .and_then(|toolhead| toolhead.position())
            .unwrap_or(self.start_position)
    }

    /// Move Z, bobbing above the target first (`manual_probe.py:166-174`).
    fn move_z(&self, z_pos: f64) -> Result<(), CommandError> {
        let toolhead = self.toolhead()?;
        let current = toolhead
            .position()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?;
        let bob_pos = z_pos + Z_BOB_MINIMUM;
        if current.z() < bob_pos {
            let mut bob = current;
            bob.set_axis(Z_AXIS, bob_pos);
            toolhead.move_to(bob, self.speed)?;
        }
        let mut target = current;
        target.set_axis(Z_AXIS, z_pos);
        match toolhead.move_to(target, self.speed) {
            Ok(()) => Ok(()),
            Err(err) => {
                self.finalize(false);
                Err(err)
            }
        }
    }

    /// Report the current and nearest untested Z positions
    /// (`manual_probe.py:176-205`).
    fn report_z_status(&self, warn_no_change: bool, prev_pos: Option<f64>) {
        let kin_pos = self.kinematics_pos();
        let z_pos = kin_pos.z();
        if warn_no_change && prev_pos == Some(z_pos) {
            if let Some(printer) = self.printer.upgrade() {
                report(
                    &printer,
                    "WARNING: No change in position (reached stepper resolution)",
                );
            }
        }
        let past = self
            .past_positions
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let next_index = bisect_left(&past, z_pos);
        let mut next_pos = next_index;
        if next_pos < past.len() && past[next_pos] == z_pos {
            next_pos += 1;
        }
        let previous_value = next_index.checked_sub(1).map(|index| past[index]);
        let next_value = past.get(next_pos).copied();
        let (previous_str, next_str) = match (previous_value, next_value) {
            (Some(previous), Some(next)) => (format!("{previous:.3}"), format!("{next:.3}")),
            (Some(previous), None) => (format!("{previous:.3}"), "??????".to_string()),
            (None, Some(next)) => ("??????".to_string(), format!("{next:.3}")),
            (None, None) => ("??????".to_string(), "??????".to_string()),
        };
        drop(past);

        {
            let mut status = self.status.lock().unwrap_or_else(|p| p.into_inner());
            *status = json!({
                "is_active": true,
                "z_position": z_pos,
                "z_position_lower": previous_value,
                "z_position_upper": next_value,
            });
        }
        if let Some(printer) = self.printer.upgrade() {
            report(
                &printer,
                &format!("Z position: {previous_str} --> {z_pos:.3} <-- {next_str}"),
            );
        }
    }

    /// `ACCEPT`: the nozzle must have moved down since the probe started.
    fn cmd_accept(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let pos = self.kinematics_pos();
        if pos.x() != self.start_position.x()
            || pos.y() != self.start_position.y()
            || pos.z() >= self.start_position.z()
        {
            gcmd.respond_info(
                "Manual probe failed! Use TESTZ commands to position the\n\
                 nozzle prior to running ACCEPT.",
            );
            self.finalize(false);
            return Ok(());
        }
        self.finalize(true);
        Ok(())
    }

    /// `ABORT`: drop the probe without a result.
    fn cmd_abort(&self, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
        self.finalize(false);
        Ok(())
    }

    /// `TESTZ`: move to the requested Z (a bisecting `+`/`-` or a number).
    fn cmd_testz(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let kin_pos = self.kinematics_pos();
        let z_pos = kin_pos.z();
        {
            let mut past = self
                .past_positions
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            let insert_at = bisect_left(&past, z_pos);
            if insert_at >= past.len() || past[insert_at] != z_pos {
                past.insert(insert_at, z_pos);
            }
        }
        let request = gcmd.get_str("Z")?;
        let next_z_pos = match request.as_str() {
            "+" | "++" => {
                let past = self
                    .past_positions
                    .lock()
                    .unwrap_or_else(|p| p.into_inner());
                let insert_at = bisect_left(&past, z_pos);
                let mut check_z = 9999999999999.9;
                if insert_at + 1 < past.len() {
                    check_z = past[insert_at + 1];
                }
                if request == "+" {
                    check_z = (check_z + z_pos) / 2.0;
                }
                (check_z).min(z_pos + BISECT_MAX)
            }
            "-" | "--" => {
                let past = self
                    .past_positions
                    .lock()
                    .unwrap_or_else(|p| p.into_inner());
                let insert_at = bisect_left(&past, z_pos);
                let mut check_z = -9999999999999.9;
                if insert_at > 0 {
                    check_z = past[insert_at - 1];
                }
                if request == "-" {
                    check_z = (check_z + z_pos) / 2.0;
                }
                (check_z).max(z_pos - BISECT_MAX)
            }
            _ => z_pos + gcmd.get_float("Z")?,
        };
        self.move_z(next_z_pos)?;
        self.report_z_status(next_z_pos != z_pos, Some(z_pos));
        Ok(())
    }

    /// End the probe: drop the commands, reset the status, report (`finalize`).
    fn finalize(&self, success: bool) {
        if let Some(printer) = self.printer.upgrade() {
            if let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) {
                gcode.unregister_command("ACCEPT");
                gcode.unregister_command("NEXT");
                gcode.unregister_command("ABORT");
                gcode.unregister_command("TESTZ");
            }
        }
        {
            let mut status = self.status.lock().unwrap_or_else(|p| p.into_inner());
            *status = reset_status();
        }
        let kin_pos = if success {
            Some(self.kinematics_pos())
        } else {
            None
        };
        (self.finalize_callback)(kin_pos);
    }
}

impl PrinterObject for ManualProbe {
    fn get_status(&self, _eventtime: f64) -> Value {
        self.status
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }
}

impl std::fmt::Debug for ManualProbe {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ManualProbe")
            .field("z_position_endstop", &self.z_position_endstop)
            .finish()
    }
}

/// Upstream's `load_config` for `[manual_probe]`.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(ManualProbe::new(config, printer)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bisect_left_finds_the_insertion_point() {
        let values = [1.0, 3.0, 5.0];

        assert_eq!(bisect_left(&values, 0.0), 0);
        assert_eq!(bisect_left(&values, 1.0), 0);
        assert_eq!(bisect_left(&values, 2.0), 1);
        assert_eq!(bisect_left(&values, 3.0), 1);
        assert_eq!(bisect_left(&values, 9.0), 3);
    }

    #[test]
    fn the_status_starts_inactive() {
        let status = reset_status();

        assert_eq!(status["is_active"], json!(false));
        assert_eq!(status["z_position"], Value::Null);
        assert_eq!(status["z_position_lower"], Value::Null);
        assert_eq!(status["z_position_upper"], Value::Null);
    }
}
