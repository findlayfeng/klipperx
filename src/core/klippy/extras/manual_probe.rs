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

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper, PrinterConfig};
use crate::core::klippy::extras::gcode_move::{GCodeMove, GCODE_MOVE_OBJECT};
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

/// The object name the probe family looks the manual probe up under.
pub(crate) const MANUAL_PROBE_OBJECT: &str = "manual_probe";

/// The Z axis index, as [`Coord`] numbers them.
const Z_AXIS: usize = 2;

/// How far above the requested position the nozzle bobs before moving down
/// (`manual_probe.py:126`).
const Z_BOB_MINIMUM: f64 = 0.500;

/// The longest single `TESTZ` step (`manual_probe.py:127`).
const BISECT_MAX: f64 = 0.200;

/// What a finished manual probe reports back (`finalize_callback`).
pub(crate) type FinalizeCallback = Arc<dyn Fn(Option<Coord>) + Send + Sync>;

/// The answer the Z-offset apply commands give at a zero offset
/// (`manual_probe.py:114`, `probe.py:174`).
const NOTHING_TO_DO: &str = "Nothing to do: Z Offset is 0";

/// Upstream's `lookup_z_endstop_config` (`manual_probe.py:20-29`): `[stepper_z]`
/// when the printer has one, else the `[carriage <name>]` whose `axis` is `z`
/// (a `kinematics: generic_cartesian` printer describes its rails that way, and
/// a scripted carriage falls back to its own name as the axis).
///
/// # Errors
/// When a carriage's `axis` option cannot be read.
fn lookup_z_endstop_config<'a>(
    config: &ConfigWrapper<'a>,
) -> Result<Option<ConfigWrapper<'a>>, ConfigError> {
    if let Some(stepper_z) = config.sibling("stepper_z") {
        return Ok(Some(stepper_z));
    }
    for carriage in config.sibling_prefix_sections("carriage ") {
        let carriage_name = carriage
            .identifier()
            .rsplit(' ')
            .next()
            .unwrap_or_default()
            .trim()
            .to_string();
        if carriage.get("axis", Some(&carriage_name))? == "z" {
            return Ok(Some(carriage));
        }
    }
    Ok(None)
}

/// The Z of `gcode_move`'s `homing_origin` — the anchor a
/// `SET_GCODE_OFFSET Z=` left (`manual_probe.py:112`, `probe.py:172`).
///
/// # Errors
/// When the printer has no `gcode_move` object.
pub(crate) fn homing_origin_z(printer: &Arc<Printer>) -> Result<f64, CommandError> {
    let gcode_move = printer
        .lookup_object_as::<GCodeMove>(GCODE_MOVE_OBJECT)
        .ok_or_else(|| CommandError::new("gcode_move is not available"))?;
    Ok(gcode_move
        .status()
        .get("homing_origin")
        .and_then(|origin| origin.as_array())
        .and_then(|axes| axes.get(Z_AXIS))
        .and_then(Value::as_f64)
        .unwrap_or(0.0))
}

/// Queue one `position_endstop` for the next `SAVE_CONFIG`
/// (`configfile.set(section, 'position_endstop', "%.3f" % value)`).
fn set_position_endstop(printer: &Arc<Printer>, section: &str, value: f64) {
    if let Some(configfile) = printer.lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT) {
        configfile.set(section, "position_endstop", &format!("{value:.3}"));
    }
}

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
    /// `[stepper_z] position_endstop` (or the Z carriage's), when there is one:
    /// it is what `Z_ENDSTOP_CALIBRATE` writes back, and its absence is why
    /// that command is not registered on a printer without a Z endstop.
    z_position_endstop: Option<f64>,
    /// The section that endstop came from (`z_endstop_config_name`), what the
    /// write-back names.
    z_endstop_config_name: Option<String>,
    /// The A/B/C tower endstops of a linear delta (`a/b/c_position_endstop`),
    /// present only when all three towers declare one — `Z_OFFSET_APPLY_ENDSTOP`
    /// shifts all three.
    delta_position_endstops: Option<(f64, f64, f64)>,
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
        // The Z endstop and the section it is written back to
        // (`manual_probe.py:36-43`).
        let (z_position_endstop, z_endstop_config_name) = match lookup_z_endstop_config(config)? {
            Some(zconfig) => (
                zconfig.get_optional_float("position_endstop")?,
                Some(zconfig.identifier()),
            ),
            None => (None, None),
        };
        // Endstop values for linear delta printers with vertical A,B,C towers
        // (`manual_probe.py:44-53`): each tower's own `position_endstop`.
        let tower_endstop = |name: &str| -> Result<Option<f64>, ConfigError> {
            match config.sibling(name) {
                Some(section) => section.get_optional_float("position_endstop"),
                None => Ok(None),
            }
        };
        let delta_position_endstops = match (
            tower_endstop("stepper_a")?,
            tower_endstop("stepper_b")?,
            tower_endstop("stepper_c")?,
        ) {
            (Some(a), Some(b), Some(c)) => Some((a, b, c)),
            _ => None,
        };
        let is_delta = config
            .sibling("printer")
            .and_then(|printer| printer.get_str("kinematics"))
            .is_some_and(|name| name == "delta");
        let status = Arc::new(Mutex::new(reset_status()));

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");

        // MANUAL_PROBE: the plain helper, reporting the accepted Z.
        {
            let printer = Arc::downgrade(printer);
            let status = Arc::clone(&status);
            gcode
                .register_command_with_params(
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
                    MANUAL_PROBE_START_PARAMS,
                    false,
                )
                .map_err(ConfigError::new)?;
        }

        // Z_ENDSTOP_CALIBRATE: same helper, writing `position_endstop` back.
        if let (Some(z_position_endstop), Some(z_endstop_config_name)) =
            (z_position_endstop, z_endstop_config_name.clone())
        {
            let printer = Arc::downgrade(printer);
            let status = Arc::clone(&status);
            gcode
                .register_command_with_params(
                    "Z_ENDSTOP_CALIBRATE",
                    Arc::new(move |gcmd| {
                        let printer = printer.clone();
                        let status = Arc::clone(&status);
                        let z_endstop_config_name = z_endstop_config_name.clone();
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
                                            "{z_endstop_config_name}: position_endstop: {z_pos:.3}\n\
                                             The SAVE_CONFIG command will update the printer config file\n\
                                             with the above and restart the printer."
                                        ),
                                    );
                                    set_position_endstop(
                                        &cb_printer,
                                        &z_endstop_config_name,
                                        z_pos,
                                    );
                                });
                            ManualProbeHelper::start(&printer, gcmd, callback, status)?;
                            Ok(())
                        })
                    }),
                    Some("Calibrate a Z endstop"),
                    MANUAL_PROBE_START_PARAMS,
                    false,
                )
                .map_err(ConfigError::new)?;
        }

        // Z_OFFSET_APPLY_ENDSTOP: fold the `gcode_move` Z offset into the Z
        // endstop's `position_endstop` (`manual_probe.py:111-124`). Upstream
        // registers this handler and then, on a delta printer, re-registers
        // the name with the tower handler (`:74-79`) — the delta one wins.
        // `register_command` refuses a second registration for one name, so
        // the same outcome is an either/or here.
        if is_delta {
            if let Some((a_position_endstop, b_position_endstop, c_position_endstop)) =
                delta_position_endstops
            {
                let printer = Arc::downgrade(printer);
                gcode
                    .register_command(
                        "Z_OFFSET_APPLY_ENDSTOP",
                        Arc::new(move |gcmd| {
                            let printer = printer.clone();
                            Box::pin(async move {
                                let printer = printer
                                    .upgrade()
                                    .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                                let offset = homing_origin_z(&printer)?;
                                if offset == 0.0 {
                                    gcmd.respond_info(NOTHING_TO_DO);
                                    return Ok(());
                                }
                                let new_a_calibrate = a_position_endstop - offset;
                                let new_b_calibrate = b_position_endstop - offset;
                                let new_c_calibrate = c_position_endstop - offset;
                                gcmd.respond_info(&format!(
                                    "stepper_a: position_endstop: {new_a_calibrate:.3}\n\
                                     stepper_b: position_endstop: {new_b_calibrate:.3}\n\
                                     stepper_c: position_endstop: {new_c_calibrate:.3}\n\
                                     The SAVE_CONFIG command will update the printer config file\n\
                                     with the above and restart the printer."
                                ));
                                set_position_endstop(&printer, "stepper_a", new_a_calibrate);
                                set_position_endstop(&printer, "stepper_b", new_b_calibrate);
                                set_position_endstop(&printer, "stepper_c", new_c_calibrate);
                                Ok(())
                            })
                        }),
                        Some("Adjust the z endstop_position"),
                        false,
                    )
                    .map_err(ConfigError::new)?;
            }
        } else if let (Some(z_position_endstop), Some(z_endstop_config_name)) =
            (z_position_endstop, z_endstop_config_name.clone())
        {
            let printer = Arc::downgrade(printer);
            gcode
                .register_command(
                    "Z_OFFSET_APPLY_ENDSTOP",
                    Arc::new(move |gcmd| {
                        let printer = printer.clone();
                        let z_endstop_config_name = z_endstop_config_name.clone();
                        Box::pin(async move {
                            let printer = printer
                                .upgrade()
                                .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                            let offset = homing_origin_z(&printer)?;
                            if offset == 0.0 {
                                gcmd.respond_info(NOTHING_TO_DO);
                                return Ok(());
                            }
                            let new_calibrate = z_position_endstop - offset;
                            gcmd.respond_info(&format!(
                                "{z_endstop_config_name}: position_endstop: {new_calibrate:.3}\n\
                                 The SAVE_CONFIG command will update the printer config file\n\
                                 with the above and restart the printer."
                            ));
                            set_position_endstop(&printer, &z_endstop_config_name, new_calibrate);
                            Ok(())
                        })
                    }),
                    Some("Adjust the z endstop_position"),
                    false,
                )
                .map_err(ConfigError::new)?;
        }

        let _ = identifier;
        Ok(Self {
            z_position_endstop,
            z_endstop_config_name,
            delta_position_endstops,
            status,
        })
    }

    /// The `[stepper_z] position_endstop` this printer calibrates, if any.
    pub fn z_position_endstop(&self) -> Option<f64> {
        self.z_position_endstop
    }

    /// Create the object when the config never wrote `[manual_probe]`.
    ///
    /// Upstream's toolhead loads `manual_probe` unconditionally
    /// (`klippy/toolhead.py:611`), which is why `PROBE_CALIBRATE` and
    /// `Z_ENDSTOP_CALIBRATE` work without the section; `config` is the
    /// `[printer]` wrapper the toolhead was built from, and the target's
    /// `position_endstop` is still reachable as its sibling.
    ///
    /// # Errors
    /// As [`ManualProbe::new`], or when the object name is taken.
    pub fn ensure(
        printer: &Arc<Printer>,
        config: &ConfigWrapper,
    ) -> Result<Arc<ManualProbe>, ConfigError> {
        if let Some(existing) = printer.lookup_object_as::<ManualProbe>(MANUAL_PROBE_OBJECT) {
            return Ok(existing);
        }
        let object = Arc::new(ManualProbe::new(config, printer)?);
        printer.add_object(MANUAL_PROBE_OBJECT, object.clone())?;
        Ok(object)
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

/// The word the manual probe helper reads when it starts, `SPEED`
/// (`manual_probe.py:24-25`): `MANUAL_PROBE` and `Z_ENDSTOP_CALIBRATE` both
/// start the same helper.
const MANUAL_PROBE_START_PARAMS: &[&str] = &["SPEED"];

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
                .register_command_with_params(
                    "ACCEPT",
                    Arc::new(move |gcmd| {
                        let helper = Arc::clone(&helper);
                        Box::pin(async move { helper.cmd_accept(gcmd) })
                    }),
                    Some("Accept the current Z position"),
                    // `cmd_accept` reads no word.
                    &[],
                    false,
                )
                .map_err(CommandError::new)?;
        }
        {
            let helper = Arc::clone(&helper);
            gcode
                .register_command_with_params(
                    "NEXT",
                    Arc::new(move |gcmd| {
                        let helper = Arc::clone(&helper);
                        Box::pin(async move { helper.cmd_accept(gcmd) })
                    }),
                    None,
                    // `NEXT` is `ACCEPT`: the same handler.
                    &[],
                    false,
                )
                .map_err(CommandError::new)?;
        }
        {
            let helper = Arc::clone(&helper);
            gcode
                .register_command_with_params(
                    "ABORT",
                    Arc::new(move |gcmd| {
                        let helper = Arc::clone(&helper);
                        Box::pin(async move { helper.cmd_abort(gcmd) })
                    }),
                    Some("Abort manual Z probing tool"),
                    // `cmd_abort` reads no word.
                    &[],
                    false,
                )
                .map_err(CommandError::new)?;
        }
        {
            let helper = Arc::clone(&helper);
            gcode
                .register_command_with_params(
                    "TESTZ",
                    Arc::new(move |gcmd| {
                        let helper = Arc::clone(&helper);
                        Box::pin(async move { helper.cmd_testz(gcmd) })
                    }),
                    Some("Move to new Z height"),
                    // `cmd_testz` reads the requested height (`manual_probe.py`
                    // `cmd_TESTZ`).
                    &["Z"],
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
            .field("z_endstop_config_name", &self.z_endstop_config_name)
            .field("delta_position_endstops", &self.delta_position_endstops)
            .finish()
    }
}

/// Load the object when the config has no `[manual_probe]` section (the
/// toolhead does this unconditionally, see [`ManualProbe::ensure`]).
///
/// # Errors
/// As [`ManualProbe::ensure`].
pub(crate) fn ensure(
    printer: &Arc<Printer>,
    config: &ConfigWrapper,
) -> Result<Arc<ManualProbe>, ConfigError> {
    ManualProbe::ensure(printer, config)
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
    use crate::core::klippy::config::{AccessTracking, Config};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::extras::gcode_move;
    use crate::core::klippy::reactor::ManualReactor;

    /// A printer with `gcode`, `configfile` and `gcode_move` — the objects the
    /// Z-offset apply commands reach at run time.
    fn machine() -> (Arc<Printer>, Arc<GCodeDispatch>, Arc<PrinterConfig>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap();
        let configfile = Arc::new(PrinterConfig::new(
            AccessTracking::shared(),
            serde_json::Map::new(),
        ));
        printer
            .add_object(
                CONFIGFILE_OBJECT,
                Arc::clone(&configfile) as Arc<dyn PrinterObject>,
            )
            .unwrap();
        gcode_move::ensure(&printer).unwrap();
        // The dispatcher only offers non-built-in commands once the printer is
        // ready (`gcode.rs`, `Commands::active`).
        printer.send_event(&KlippyEvent::KlippyReady);
        (printer, gcode, configfile)
    }

    /// Load a `[manual_probe]` over `text`, read through the `[printer]`
    /// wrapper the toolhead's `ensure` hands it.
    fn load(text: &str) -> (Arc<Printer>, Arc<GCodeDispatch>, Arc<PrinterConfig>) {
        let (printer, gcode, configfile) = machine();
        let (config, _) = Config::from_text(text).expect("the test config parses");
        let section = config.get_section("printer").expect("a [printer] section");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        ManualProbe::new(&wrapper, &printer).expect("the section loads");
        (printer, gcode, configfile)
    }

    /// Capture every line the dispatcher emits.
    fn capture(gcode: &Arc<GCodeDispatch>) -> Arc<Mutex<Vec<String>>> {
        let log = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&log);
        gcode.register_output_handler(Arc::new(move |line: &str| {
            sink.lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(line.to_string());
        }));
        log
    }

    fn lines(log: &Arc<Mutex<Vec<String>>>) -> Vec<String> {
        log.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }

    /// The pending autosave items `SAVE_CONFIG` would write.
    fn pending(configfile: &Arc<PrinterConfig>) -> Value {
        configfile.get_status(0.0)["save_config_pending_items"].clone()
    }

    const CARTESIAN: &str = "[printer]\nkinematics: cartesian\n\
                            [stepper_z]\nposition_endstop: 1.0\n";

    #[test]
    fn applying_the_endstop_at_a_zero_offset_only_reports() {
        let (_printer, gcode, configfile) = load(CARTESIAN);
        let log = capture(&gcode);

        gcode.run_script_sync("Z_OFFSET_APPLY_ENDSTOP").unwrap();

        assert_eq!(lines(&log), ["// Nothing to do: Z Offset is 0"]);
        assert_eq!(
            configfile.get_status(0.0)["save_config_pending"],
            json!(false)
        );
    }

    #[test]
    fn applying_the_endstop_writes_the_shifted_position_endstop() {
        let (_printer, gcode, configfile) = load(CARTESIAN);
        let log = capture(&gcode);

        gcode.run_script_sync("SET_GCODE_OFFSET Z=0.25").unwrap();
        gcode.run_script_sync("Z_OFFSET_APPLY_ENDSTOP").unwrap();

        assert_eq!(
            lines(&log),
            ["// stepper_z: position_endstop: 0.750\n\
              // The SAVE_CONFIG command will update the printer config file\n\
              // with the above and restart the printer."]
        );
        assert_eq!(
            pending(&configfile)["stepper_z"]["position_endstop"],
            json!("0.750")
        );
    }

    #[test]
    fn a_carriage_described_z_endstop_is_the_one_written_back() {
        // A `kinematics: generic_cartesian` printer has no `[stepper_z]`: its Z
        // endstop lives on the `[carriage <name>]` whose `axis` is `z`
        // (`manual_probe.py:lookup_z_endstop_config`). The `[carriage x]` in
        // front of it is skipped.
        let (_printer, gcode, configfile) = load(
            "[printer]\nkinematics: generic_cartesian\n\
             [carriage x]\naxis: x\nposition_endstop: 9.0\n\
             [carriage z]\naxis: z\nposition_endstop: 5.0\n",
        );
        let log = capture(&gcode);

        gcode.run_script_sync("SET_GCODE_OFFSET Z=1.0").unwrap();
        gcode.run_script_sync("Z_OFFSET_APPLY_ENDSTOP").unwrap();

        assert_eq!(
            lines(&log),
            ["// carriage z: position_endstop: 4.000\n\
              // The SAVE_CONFIG command will update the printer config file\n\
              // with the above and restart the printer."]
        );
        assert_eq!(
            pending(&configfile)["carriage z"]["position_endstop"],
            json!("4.000")
        );
    }

    #[test]
    fn the_delta_variant_takes_over_the_same_command() {
        let (_printer, gcode, configfile) = load(
            "[printer]\nkinematics: delta\n\
             [stepper_a]\nposition_endstop: 1.0\n\
             [stepper_b]\nposition_endstop: 2.0\n\
             [stepper_c]\nposition_endstop: 3.0\n",
        );
        let log = capture(&gcode);

        gcode.run_script_sync("SET_GCODE_OFFSET Z=0.5").unwrap();
        gcode.run_script_sync("Z_OFFSET_APPLY_ENDSTOP").unwrap();

        assert_eq!(
            lines(&log),
            ["// stepper_a: position_endstop: 0.500\n\
              // stepper_b: position_endstop: 1.500\n\
              // stepper_c: position_endstop: 2.500\n\
              // The SAVE_CONFIG command will update the printer config file\n\
              // with the above and restart the printer."]
        );
        let pending = pending(&configfile);
        assert_eq!(pending["stepper_a"]["position_endstop"], json!("0.500"));
        assert_eq!(pending["stepper_b"]["position_endstop"], json!("1.500"));
        assert_eq!(pending["stepper_c"]["position_endstop"], json!("2.500"));
    }

    #[test]
    fn the_delta_variant_also_reports_at_a_zero_offset() {
        let (_printer, gcode, configfile) = load(
            "[printer]\nkinematics: delta\n\
             [stepper_a]\nposition_endstop: 1.0\n\
             [stepper_b]\nposition_endstop: 2.0\n\
             [stepper_c]\nposition_endstop: 3.0\n",
        );
        let log = capture(&gcode);

        gcode.run_script_sync("Z_OFFSET_APPLY_ENDSTOP").unwrap();

        assert_eq!(lines(&log), ["// Nothing to do: Z Offset is 0"]);
        assert_eq!(
            configfile.get_status(0.0)["save_config_pending"],
            json!(false)
        );
    }

    #[test]
    fn the_command_needs_an_endstop_to_write_back_to() {
        // No `[stepper_z]`, no Z carriage: there is nothing to shift, so the
        // command is not registered. The A/B/C towers alone are not enough on
        // a cartesian printer.
        let (_printer, gcode, _configfile) = load(
            "[printer]\nkinematics: cartesian\n\
             [stepper_a]\nposition_endstop: 1.0\n\
             [stepper_b]\nposition_endstop: 2.0\n\
             [stepper_c]\nposition_endstop: 3.0\n",
        );

        assert!(!gcode.command_exists("Z_OFFSET_APPLY_ENDSTOP"));
        assert!(!gcode.command_exists("Z_ENDSTOP_CALIBRATE"));
    }

    #[test]
    fn a_delta_without_tower_endstops_registers_no_apply_command() {
        // The delta handler shifts all three tower endstops; a tower without
        // one of its own leaves it unregistered rather than crashing on the
        // missing value (upstream registers it and fails on `None`).
        let (_printer, gcode, _configfile) = load(
            "[printer]\nkinematics: delta\n\
             [stepper_a]\nposition_endstop: 1.0\n\
             [stepper_b]\nposition_endstop: 2.0\n",
        );

        assert!(!gcode.command_exists("Z_OFFSET_APPLY_ENDSTOP"));
    }

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
