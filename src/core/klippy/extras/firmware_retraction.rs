//! `[firmware_retraction]` — Marlin/Reprap-style firmware retraction through
//! `G10`/`G11` (upstream `klippy/extras/firmware_retraction.py`).
//!
//! | option | default | role |
//! |---|---|---|
//! | `retract_length` | 0 | the mm `G10` pulls the extruder back (`minval=0`) |
//! | `retract_speed` | 20 | the `G10` move's speed in mm/s (`minval=1`) |
//! | `unretract_extra_length` | 0 | mm added to the `G11` move (`minval=0`) |
//! | `unretract_speed` | 10 | the `G11` move's speed in mm/s (`minval=1`) |
//!
//! `unretract_length` is derived (`retract_length + unretract_extra_length`,
//! `firmware_retraction.py:15-16`) and not an option.
//!
//! | command | meaning |
//! |---|---|
//! | `G10` | pull the extruder back once; a second `G10` while retracted does nothing |
//! | `G11` | undo the retraction; a `G11` while not retracted does nothing |
//! | `SET_RETRACTION` | set any of the four parameters (each optional, its own `minval`) |
//! | `GET_RETRACTION` | report the four parameters |
//!
//! `G10`/`G11` do not move anything themselves: they run a script through the
//! dispatcher (`run_script_from_command`) that parks the g-code state, goes
//! relative, moves the extruder, and restores the state — so the move is an
//! ordinary `G1` and takes the active coordinate system and factors with it
//! (`firmware_retraction.py:55-70`). The `is_retracted` guard keeps a repeated
//! `G10` from pulling back twice; `SET_RETRACTION` clears it, as upstream does.
//!
//! The dispatcher is held as a `Weak`: its command table owns this object
//! (the handlers capture it), so a strong handle back would be a reference
//! cycle (the `endstop_phase.rs` convention).

use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::gcode::{
    parse_float, CommandError, CommandFuture, CommandHandler, GCodeDispatch, GcodeCommand,
    GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("firmware_retraction", order = 30, load = load_config);

/// The parameters `G10`/`G11` read and `SET_RETRACTION`/`GET_RETRACTION`
/// change, kept behind one lock so a status query and a command cannot see a
/// half-updated set.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Settings {
    /// `retract_length` (`firmware_retraction.py:10`).
    retract_length: f64,
    /// `retract_speed`, mm/s (`:11`).
    retract_speed: f64,
    /// `unretract_extra_length` (`:12-13`).
    unretract_extra_length: f64,
    /// `unretract_speed`, mm/s (`:14`).
    unretract_speed: f64,
    /// `retract_length + unretract_extra_length` (`:15-16`).
    unretract_length: f64,
    /// `is_retracted`: whether a `G10` still owes a `G11` (`:17`).
    is_retracted: bool,
}

/// The `[firmware_retraction]` module object (upstream's `FirmwareRetraction`).
pub struct FirmwareRetraction {
    /// The four parameters and the retracted flag.
    settings: Mutex<Settings>,
    /// The dispatcher `G10`/`G11` run their script through. Weak: the table
    /// holds this object, see the module docs.
    gcode: Weak<GCodeDispatch>,
}

impl FirmwareRetraction {
    /// Read the section (`firmware_retraction.py:10-18`).
    ///
    /// # Errors
    /// A value below its `minval`: `retract_length`/`unretract_extra_length`
    /// at 0, `retract_speed`/`unretract_speed` at 1.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let retract_length =
            config.get_float_bounded("retract_length", Some(0.), Some(0.), None, None, None)?;
        let retract_speed =
            config.get_float_bounded("retract_speed", Some(20.), Some(1.), None, None, None)?;
        let unretract_extra_length = config.get_float_bounded(
            "unretract_extra_length",
            Some(0.),
            Some(0.),
            None,
            None,
            None,
        )?;
        let unretract_speed =
            config.get_float_bounded("unretract_speed", Some(10.), Some(1.), None, None, None)?;
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        Ok(Self {
            settings: Mutex::new(Settings {
                retract_length,
                retract_speed,
                unretract_extra_length,
                unretract_speed,
                unretract_length: retract_length + unretract_extra_length,
                is_retracted: false,
            }),
            gcode: Arc::downgrade(&gcode),
        })
    }

    /// The settings lock, poisoning treated as continued unwinding
    /// (`gcode_move.rs` convention).
    fn lock(&self) -> MutexGuard<'_, Settings> {
        self.settings
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The dispatcher, when it is still up.
    fn gcode(&self) -> Option<Arc<GCodeDispatch>> {
        self.gcode.upgrade()
    }

    /// `is_retracted` (`firmware_retraction.py:17`).
    fn is_retracted(&self) -> bool {
        self.lock().is_retracted
    }

    /// Set `is_retracted` (`firmware_retraction.py:61,71`).
    fn set_retracted(&self, value: bool) {
        self.lock().is_retracted = value;
    }

    /// The script `G10` runs (`firmware_retraction.py:55-60`): park the state,
    /// go relative, pull the extruder back, restore the state.
    fn retract_script(&self) -> String {
        let settings = self.lock();
        format!(
            "SAVE_GCODE_STATE NAME=_retract_state\n\
             G91\n\
             G1 E-{:.5} F{}\n\
             RESTORE_GCODE_STATE NAME=_retract_state",
            settings.retract_length,
            speed_argument(settings.retract_speed),
        )
    }

    /// The script `G11` runs (`firmware_retraction.py:65-70`), with
    /// `unretract_length` and `unretract_speed`.
    fn unretract_script(&self) -> String {
        let settings = self.lock();
        format!(
            "SAVE_GCODE_STATE NAME=_retract_state\n\
             G91\n\
             G1 E{:.5} F{}\n\
             RESTORE_GCODE_STATE NAME=_retract_state",
            settings.unretract_length,
            speed_argument(settings.unretract_speed),
        )
    }

    /// `SET_RETRACTION` (`firmware_retraction.py:34-45`): override any of the
    /// four parameters, re-derive `unretract_length`, and clear
    /// `is_retracted`.
    ///
    /// # Errors
    /// As the parameter readers: a value below its `minval`.
    fn cmd_set_retraction(&self, gcmd: &GcodeCommand) -> Result<(), CommandError> {
        let mut settings = self.lock();
        let current = *settings;
        settings.retract_length = gcmd.get(
            "RETRACT_LENGTH",
            Some(current.retract_length),
            parse_float,
            Some(0.),
            None,
            None,
            None,
        )?;
        settings.retract_speed = gcmd.get(
            "RETRACT_SPEED",
            Some(current.retract_speed),
            parse_float,
            Some(1.),
            None,
            None,
            None,
        )?;
        settings.unretract_extra_length = gcmd.get(
            "UNRETRACT_EXTRA_LENGTH",
            Some(current.unretract_extra_length),
            parse_float,
            Some(0.),
            None,
            None,
            None,
        )?;
        settings.unretract_speed = gcmd.get(
            "UNRETRACT_SPEED",
            Some(current.unretract_speed),
            parse_float,
            Some(1.),
            None,
            None,
            None,
        )?;
        settings.unretract_length = settings.retract_length + settings.unretract_extra_length;
        settings.is_retracted = false;
        Ok(())
    }

    /// `GET_RETRACTION` (`firmware_retraction.py:47-51`): report the four
    /// parameters, and nothing else.
    fn cmd_get_retraction(&self, gcmd: &GcodeCommand) {
        let settings = *self.lock();
        gcmd.respond_info(&format!(
            concat!(
                "RETRACT_LENGTH={:.5} RETRACT_SPEED={:.5}",
                " UNRETRACT_EXTRA_LENGTH={:.5} UNRETRACT_SPEED={:.5}"
            ),
            settings.retract_length,
            settings.retract_speed,
            settings.unretract_extra_length,
            settings.unretract_speed,
        ));
    }

    /// Register the four commands, capturing a handle to this object
    /// (`firmware_retraction.py:19-24`).
    fn register_commands(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        type Command =
            for<'a> fn(&'a Arc<FirmwareRetraction>, &'a GcodeCommand) -> CommandFuture<'a>;
        const COMMANDS: &[(&str, Command, Option<&str>, &[&str])] = &[
            (
                "SET_RETRACTION",
                cmd_set_retraction,
                Some(SET_RETRACTION_HELP),
                SET_RETRACTION_PARAMS,
            ),
            (
                "GET_RETRACTION",
                cmd_get_retraction,
                Some(GET_RETRACTION_HELP),
                &[],
            ),
            ("G10", cmd_g10, None, &[]),
            ("G11", cmd_g11, None, &[]),
        ];
        for &(name, command, help, params) in COMMANDS {
            let handler: CommandHandler = {
                let object = Arc::clone(self);
                Arc::new(move |gcmd| {
                    let object = Arc::clone(&object);
                    Box::pin(async move { command(&object, gcmd).await })
                })
            };
            gcode
                .register_command_with_params(name, handler, help, params, false)
                .map_err(ConfigError::new)?;
        }
        Ok(())
    }
}

/// The four words `SET_RETRACTION` reads, in source order
/// (`firmware_retraction.py:34-45`). Listed once for the `COMMANDS` table.
const SET_RETRACTION_PARAMS: &[&str] = &[
    "RETRACT_LENGTH",
    "RETRACT_SPEED",
    "UNRETRACT_EXTRA_LENGTH",
    "UNRETRACT_SPEED",
];

/// Upstream's `cmd_SET_RETRACTION_help` (`firmware_retraction.py:33`).
const SET_RETRACTION_HELP: &str = "Set firmware retraction parameters";

/// Upstream's `cmd_GET_RETRACTION_help` (`firmware_retraction.py:46`).
const GET_RETRACTION_HELP: &str = "Report firmware retraction parameters";

/// The `F` word of a retraction move: upstream's `%d % (speed*60)`
/// (`firmware_retraction.py:58`), a feedrate in mm/min.
///
/// Python's `%d` formats a float by truncating toward zero; `as i64` does the
/// same for the values a speed can take.
fn speed_argument(speed: f64) -> i64 {
    (speed * 60.) as i64
}

/// `G10` (`firmware_retraction.py:55-60`): pull the extruder back, once. An
/// already-retracted extruder is left alone.
fn cmd_g10<'a>(object: &'a Arc<FirmwareRetraction>, _gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
    Box::pin(async move {
        if object.is_retracted() {
            return Ok(());
        }
        let script = object.retract_script();
        object
            .gcode()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?
            .run_script_from_command(&script)
            .await?;
        object.set_retracted(true);
        Ok(())
    })
}

/// `G11` (`firmware_retraction.py:65-70`): undo the retraction. An extruder
/// that is not retracted is left alone.
fn cmd_g11<'a>(object: &'a Arc<FirmwareRetraction>, _gcmd: &'a GcodeCommand) -> CommandFuture<'a> {
    Box::pin(async move {
        if !object.is_retracted() {
            return Ok(());
        }
        let script = object.unretract_script();
        object
            .gcode()
            .ok_or_else(|| CommandError::new("Printer is not ready"))?
            .run_script_from_command(&script)
            .await?;
        object.set_retracted(false);
        Ok(())
    })
}

/// `SET_RETRACTION` (`firmware_retraction.py:34-45`).
fn cmd_set_retraction<'a>(
    object: &'a Arc<FirmwareRetraction>,
    gcmd: &'a GcodeCommand,
) -> CommandFuture<'a> {
    Box::pin(async move { object.cmd_set_retraction(gcmd) })
}

/// `GET_RETRACTION` (`firmware_retraction.py:47-51`).
fn cmd_get_retraction<'a>(
    object: &'a Arc<FirmwareRetraction>,
    gcmd: &'a GcodeCommand,
) -> CommandFuture<'a> {
    Box::pin(async move {
        object.cmd_get_retraction(gcmd);
        Ok(())
    })
}

impl PrinterObject for FirmwareRetraction {
    /// Upstream's `FirmwareRetraction.get_status` (`firmware_retraction.py:26-32`).
    fn get_status(&self, _eventtime: f64) -> Value {
        let settings = *self.lock();
        json!({
            "retract_length": settings.retract_length,
            "retract_speed": settings.retract_speed,
            "unretract_extra_length": settings.unretract_extra_length,
            "unretract_speed": settings.unretract_speed,
        })
    }
}

/// The factory `section!` names (`firmware_retraction.py:73 def load_config`).
///
/// # Errors
/// A value below its `minval`, or a command name this dispatcher refuses.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = Arc::new(FirmwareRetraction::new(config, printer)?);
    object.register_commands(printer)?;
    Ok(object as Arc<dyn PrinterObject>)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{Config, ConfigSection, ConfigValue};
    use crate::core::klippy::event::KlippyEvent;
    use crate::core::klippy::extras::gcode_move::{self, MoveTarget};
    use crate::core::klippy::mathutil::{Coord, E_AXIS};
    use crate::core::klippy::reactor::ManualReactor;

    /// A section with the given options, as the parser would build it.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("firmware_retraction", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// Load one `[firmware_retraction]` onto a printer that already has
    /// `gcode` and `gcode_move`.
    fn load(
        printer: &Arc<Printer>,
        options: &[(&str, &str)],
    ) -> Result<Arc<dyn PrinterObject>, ConfigError> {
        load_config(&ConfigWrapper::untracked(&section(options)), printer)
    }

    /// A move target that records what it was asked and stands where the last
    /// move left it — the extruder, as far as the script cares.
    struct RecordingTarget {
        position: Mutex<Coord>,
        moves: Mutex<Vec<(Coord, f64)>>,
    }

    impl RecordingTarget {
        fn new() -> Self {
            Self {
                position: Mutex::new(Coord::default()),
                moves: Mutex::new(Vec::new()),
            }
        }

        fn moves(&self) -> Vec<(Coord, f64)> {
            self.moves
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .clone()
        }
    }

    impl MoveTarget for RecordingTarget {
        fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
            *self
                .position
                .lock()
                .unwrap_or_else(|poison| poison.into_inner()) = position;
            self.moves
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .push((position, speed));
            Ok(())
        }

        fn position(&self) -> Coord {
            *self
                .position
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
        }
    }

    /// A ready printer with `gcode`, `gcode_move` and a recording move target,
    /// plus the output the dispatcher emits.
    fn machine() -> (
        Arc<Printer>,
        Arc<GCodeDispatch>,
        Arc<RecordingTarget>,
        Arc<Mutex<Vec<String>>>,
    ) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let gcode_move = gcode_move::ensure(&printer).unwrap();
        let target = Arc::new(RecordingTarget::new());
        gcode_move
            .set_move_transform(Arc::clone(&target) as Arc<dyn MoveTarget>, true)
            .unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        printer.send_event(&KlippyEvent::ToolheadSetPosition);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap();
        let output = Arc::new(Mutex::new(Vec::new()));
        {
            let output = Arc::clone(&output);
            gcode.register_output_handler(Arc::new(move |line: &str| {
                output
                    .lock()
                    .unwrap_or_else(|poison| poison.into_inner())
                    .push(line.to_string());
            }));
        }
        (printer, gcode, target, output)
    }

    fn emitted(output: &Arc<Mutex<Vec<String>>>) -> Vec<String> {
        output
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }

    /// The defaults are upstream's, and `get_status` reports them
    /// (`firmware_retraction.py:10-14,26-32`).
    #[test]
    fn test_defaults_and_status() {
        let (printer, _gcode, _target, _output) = machine();
        let object = load(&printer, &[]).unwrap();

        assert_eq!(
            object.get_status(0.0),
            json!({
                "retract_length": 0.0,
                "retract_speed": 20.0,
                "unretract_extra_length": 0.0,
                "unretract_speed": 10.0,
            })
        );
    }

    /// A `retract_length` below 0 and a `retract_speed` below 1 are refused
    /// with upstream's wording (`configfile.py:49-50`).
    #[test]
    fn test_a_negative_retract_length_is_refused() {
        let (printer, _gcode, _target, _output) = machine();

        let err = load(&printer, &[("retract_length", "-1")])
            .err()
            .expect("refused");
        assert_eq!(
            err.to_string(),
            "Option 'retract_length' in section 'firmware_retraction' must have minimum of 0"
        );
    }

    /// `retract_speed` has `minval=1`, not 0.
    #[test]
    fn test_a_zero_retract_speed_is_refused() {
        let (printer, _gcode, _target, _output) = machine();

        let err = load(&printer, &[("retract_speed", "0")])
            .err()
            .expect("refused");
        assert_eq!(
            err.to_string(),
            "Option 'retract_speed' in section 'firmware_retraction' must have minimum of 1"
        );
    }

    /// `unretract_speed` has `minval=1` too.
    #[test]
    fn test_a_zero_unretract_speed_is_refused() {
        let (printer, _gcode, _target, _output) = machine();

        let err = load(&printer, &[("unretract_speed", "0.5")])
            .err()
            .expect("refused");
        assert_eq!(
            err.to_string(),
            "Option 'unretract_speed' in section 'firmware_retraction' must have minimum of 1"
        );
    }

    /// `G10` pulls the extruder back by `retract_length` at `retract_speed`,
    /// and says so on the move target (`firmware_retraction.py:55-60`).
    #[test]
    fn test_g10_retracts_by_length_at_speed() {
        let (printer, gcode, target, _output) = machine();
        load(
            &printer,
            &[
                ("retract_length", "2"),
                ("retract_speed", "200"),
                ("unretract_speed", "100"),
            ],
        )
        .unwrap();

        gcode.run_script_sync("G10").unwrap();

        let moves = target.moves();
        assert_eq!(moves.len(), 1);
        // `G1 E-2.00000 F12000`, and `gcode_move` turns the feedrate into mm/s.
        assert_eq!(moves[0], (Coord::new(0.0, 0.0, 0.0, -2.0), 200.0));
    }

    /// `G11` moves the extruder back by `retract_length +
    /// unretract_extra_length` at `unretract_speed` (`firmware_retraction.py:65-70`).
    #[test]
    fn test_g11_unretracts_by_the_derived_length() {
        let (printer, gcode, target, _output) = machine();
        load(
            &printer,
            &[
                ("retract_length", "2"),
                ("unretract_extra_length", "0.5"),
                ("retract_speed", "200"),
                ("unretract_speed", "50"),
            ],
        )
        .unwrap();

        gcode.run_script_sync("G10").unwrap();
        gcode.run_script_sync("G11").unwrap();

        let moves = target.moves();
        assert_eq!(moves.len(), 2);
        // The second move is the undoing one: `E` back to 0 from -2, at 50 mm/s.
        let (before, after) = (moves[0].0.axis(E_AXIS), moves[1].0.axis(E_AXIS));
        assert_eq!(after - before, 2.5, "2 + 0.5 mm back");
        assert_eq!(moves[1].1, 50.0);
    }

    /// A second `G10` while retracted does not move again; `SET_RETRACTION`
    /// clears the flag, so a `G10` after it does (`firmware_retraction.py:55-60,48`).
    #[test]
    fn test_a_repeated_g10_is_a_no_op() {
        let (printer, gcode, target, _output) = machine();
        load(&printer, &[("retract_length", "2")]).unwrap();

        gcode.run_script_sync("G10").unwrap();
        gcode.run_script_sync("G10").unwrap();
        assert_eq!(target.moves().len(), 1, "the second G10 is a no-op");

        gcode.run_script_sync("SET_RETRACTION").unwrap();
        gcode.run_script_sync("G10").unwrap();
        assert_eq!(target.moves().len(), 2, "SET_RETRACTION cleared the flag");
    }

    /// A `G11` while not retracted does nothing (`firmware_retraction.py:64`).
    #[test]
    fn test_a_g11_while_not_retracted_is_a_no_op() {
        let (printer, gcode, target, _output) = machine();
        load(&printer, &[("retract_length", "2")]).unwrap();

        gcode.run_script_sync("G11").unwrap();

        assert!(target.moves().is_empty());
    }

    /// With `retract_length = 0` the script still runs — upstream has no
    /// special case — but the move is zero (`firmware_retraction.py:58`).
    #[test]
    fn test_a_zero_retract_length_moves_zero() {
        let (printer, gcode, target, _output) = machine();
        load(&printer, &[]).unwrap();

        gcode.run_script_sync("G10").unwrap();

        let moves = target.moves();
        assert_eq!(moves.len(), 1);
        assert_eq!(moves[0].0, Coord::default(), "E-0.00000 is no motion");
    }

    /// `SET_RETRACTION` overrides all four parameters and `GET_RETRACTION`
    /// reports them back with upstream's line (`firmware_retraction.py:34-51`).
    #[test]
    fn test_set_retraction_overrides_and_get_retraction_reports() {
        let (printer, gcode, _target, output) = machine();
        let object = load(&printer, &[]).unwrap();

        gcode
            .run_script_sync(
                "SET_RETRACTION RETRACT_LENGTH=5 RETRACT_SPEED=60 \
                 UNRETRACT_EXTRA_LENGTH=1.5 UNRETRACT_SPEED=30",
            )
            .unwrap();
        assert_eq!(
            object.get_status(0.0),
            json!({
                "retract_length": 5.0,
                "retract_speed": 60.0,
                "unretract_extra_length": 1.5,
                "unretract_speed": 30.0,
            })
        );

        gcode.run_script_sync("GET_RETRACTION").unwrap();
        assert!(
            emitted(&output).contains(
                &"// RETRACT_LENGTH=5.00000 RETRACT_SPEED=60.00000 \
                  UNRETRACT_EXTRA_LENGTH=1.50000 UNRETRACT_SPEED=30.00000"
                    .to_string()
            ),
            "{:?}",
            emitted(&output)
        );
    }

    /// A `SET_RETRACTION` parameter leaves the unset ones alone, and a value
    /// below its `minval` is refused with upstream's wording
    /// (`firmware_retraction.py:35-42`).
    #[test]
    fn test_set_retraction_keeps_unset_parameters_and_bounds_each() {
        let (printer, gcode, _target, _output) = machine();
        let object = load(&printer, &[("retract_length", "3")]).unwrap();

        gcode
            .run_script_sync("SET_RETRACTION RETRACT_SPEED=80")
            .unwrap();
        assert_eq!(object.get_status(0.0)["retract_length"], json!(3.0));
        assert_eq!(object.get_status(0.0)["retract_speed"], json!(80.0));

        let err = gcode
            .run_script_sync("SET_RETRACTION RETRACT_SPEED=0")
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "Error on 'SET_RETRACTION RETRACT_SPEED=0': RETRACT_SPEED must have minimum of 1"
        );
    }

    /// The commands reach the dispatcher's help table with upstream's text.
    #[test]
    fn test_the_commands_are_registered() {
        let (printer, gcode, _target, _output) = machine();
        load(&printer, &[]).unwrap();

        let status = gcode.get_status(0.0);
        let commands = status["commands"].as_object().unwrap();
        assert_eq!(
            commands["SET_RETRACTION"]["help"],
            json!("Set firmware retraction parameters")
        );
        assert_eq!(
            commands["GET_RETRACTION"]["help"],
            json!("Report firmware retraction parameters")
        );
        assert!(commands.contains_key("G10"));
        assert!(commands.contains_key("G11"));
    }

    /// An option-less `[firmware_retraction]` is a valid section — the
    /// config-load contract.
    #[test]
    fn test_an_empty_section_loads() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) =
            Config::from_text("[mcu]\nserial: /dev/not-opened-yet\n[firmware_retraction]\n")
                .expect("the config parses");

        printer
            .load_config(&config)
            .expect("an option-less [firmware_retraction] loads");
        assert!(printer.lookup_object("firmware_retraction").is_some());
    }
}
