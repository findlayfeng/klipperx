//! `bed_tilt` — level a tilted bed in the G-Code coordinate system.
//!
//! Upstream's `klippy/extras/bed_tilt.py`. Where `z_tilt` moves Z steppers,
//! this module never touches them: it claims the `gcode_move` move-transform
//! slot ([`MoveTarget`]) and shifts **Z** by a plane — `x_adjust`/`y_adjust`
//! the slopes, `z_adjust` the intercept — so every `G1`/`M114` speaks a
//! coordinate system that follows the bed's tilt:
//!
//! - [`BedTilt::position`] (upstream's `get_position`) **subtracts** the
//!   plane from the toolhead's Z;
//! - [`BedTilt::move_to`] (upstream's `move`) **adds** it back before the
//!   toolhead queues the move.
//!
//! The plane is data, not motion: no stepper turns, only the transform
//! shifts Z. [`BedTilt::update_adjust`] stores a new plane, re-anchors
//! `gcode_move`'s `last_position` and queues the three values for
//! `SAVE_CONFIG`; [`BedTiltCalibrate`] fits that plane from probed `points`
//! — with `points` absent the section compensates only and the command
//! never exists. `BED_TILT_CALIBRATE` itself takes `METHOD` and
//! `HORIZONTAL_MOVE_Z` plus the probe's own parameters, all passed straight
//! to [`ProbePointsHelper::start_probe`] (no `RETRIES`, unlike
//! `z_tilt`/`quad_gantry_level`).
//!
//! | option | default | role |
//! |---|---|---|
//! | `x_adjust` / `y_adjust` / `z_adjust` | `0.` | the plane (the `SAVE_CONFIG` items) |
//! | `points` | — | probe points; **their presence** registers `BED_TILT_CALIBRATE` (`bed_tilt.py:18-19`) |
//! | `horizontal_move_z` / `speed` | `5.0` / `50.` | the calibration's travels (read by [`ProbePointsHelper`]) |
//!
//! [`ProbePointsHelper`]: crate::core::klippy::extras::probe::ProbePointsHelper

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};
use tracing::info;

use crate::core::klippy::config::object::{PrinterConfig, CONFIGFILE_OBJECT};
use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::gcode_move::{self, GCodeMove, MoveTarget, GCODE_MOVE_OBJECT};
use crate::core::klippy::extras::probe::{
    probe_points_params, ProbeOffsets, ProbePointsFinalize, ProbePointsHelper,
};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{CommandError, GCodeDispatch, GCODE_OBJECT};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{coordinate_descent, Coord, Z_AXIS};
use crate::core::klippy::printer::{Printer, PrinterObject};
use crate::logging::set_rollover_info;

section!("bed_tilt", order = 30, load = load_config);

/// The toolhead's object name (`[printer]` is registered as `toolhead`).
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The section's name — upstream hardcodes `'bed_tilt'` in the `SAVE_CONFIG`
/// write-back (`bed_tilt.py:42-44`); the section has no prefix form.
const SECTION: &str = "bed_tilt";

/// The plane the transform applies: upstream's `x_adjust`/`y_adjust`/
/// `z_adjust` trio, copied as one value (`bed_tilt.py:36-38`).
#[derive(Debug, Clone, Copy, PartialEq)]
struct Adjust {
    /// The Z slope along X.
    x_adjust: f64,
    /// The Z slope along Y.
    y_adjust: f64,
    /// The plane's intercept (probe `z_offset` and XY offsets removed).
    z_adjust: f64,
}

/// One `[bed_tilt]` section: the plane and the move transform
/// (`bed_tilt.py:10-44`).
pub struct BedTilt {
    /// The machine, to find `toolhead`/`gcode_move`/`configfile` at run time.
    printer: Weak<Printer>,
    /// The compensation plane.
    adjust: Mutex<Adjust>,
    /// The toolhead the transform wraps — set at `klippy:connect`
    /// (upstream's `handle_connect`), the fake a test injects before that.
    target: Mutex<Option<Arc<dyn MoveTarget>>>,
}

impl BedTilt {
    /// Read the section's plane (`bed_tilt.py:15-17`).
    ///
    /// # Errors
    /// When an adjust option is malformed.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        Ok(Self {
            printer: Arc::downgrade(printer),
            adjust: Mutex::new(Adjust {
                x_adjust: config.get_float("x_adjust", Some(0.0))?,
                y_adjust: config.get_float("y_adjust", Some(0.0))?,
                z_adjust: config.get_float("z_adjust", Some(0.0))?,
            }),
            target: Mutex::new(None),
        })
    }

    /// The events upstream's `__init__` subscribes to (`bed_tilt.py:13-14`).
    fn register_handlers(self: &Arc<Self>, printer: &Arc<Printer>) {
        printer.register_event_handler(
            KlippyEvent::KlippyConnect,
            Box::new({
                let object = Arc::clone(self);
                move |_| object.handle_connect()
            }),
        );
    }

    /// Upstream's `handle_connect`: the toolhead exists by `klippy:connect`.
    fn handle_connect(&self) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        if let Some(toolhead) = printer.lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT) {
            *self.target.lock().unwrap_or_else(|p| p.into_inner()) =
                Some(Arc::new(ToolheadMove(toolhead)));
        }
    }

    /// The plane, copied out of its lock.
    fn adjust(&self) -> Adjust {
        *self.adjust.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Store the new plane, re-anchor `gcode_move`'s `last_position` and
    /// queue the values for `SAVE_CONFIG` (`bed_tilt.py:35-44`).
    pub fn update_adjust(&self, x_adjust: f64, y_adjust: f64, z_adjust: f64) {
        *self.adjust.lock().unwrap_or_else(|p| p.into_inner()) = Adjust {
            x_adjust,
            y_adjust,
            z_adjust,
        };
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        // The g-code position was anchored to the old plane; re-read it
        // through the transform (`gcode_move.reset_last_position`).
        if let Some(gcode_move) = printer.lookup_object_as::<GCodeMove>(GCODE_MOVE_OBJECT) {
            gcode_move.reset_last_position();
        }
        // The write-back `SAVE_CONFIG` will flush (`ConfigAutoSave.set`).
        if let Some(configfile) = printer.lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT) {
            configfile.set(SECTION, "x_adjust", &format!("{x_adjust:.6}"));
            configfile.set(SECTION, "y_adjust", &format!("{y_adjust:.6}"));
            configfile.set(SECTION, "z_adjust", &format!("{z_adjust:.6}"));
        }
    }

    /// The transform's underlying target, when there is one to ask.
    fn target(&self) -> Option<Arc<dyn MoveTarget>> {
        self.target
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone()
    }
}

impl MoveTarget for BedTilt {
    /// Upstream's `move` (`bed_tilt.py:31-34`): add the plane back — the
    /// caller speaks the tilted bed's coordinates, the toolhead the real ones.
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        let adjust = self.adjust();
        let mut toolhead_position = position;
        toolhead_position.set_axis(
            Z_AXIS,
            position.z()
                + position.x() * adjust.x_adjust
                + position.y() * adjust.y_adjust
                + adjust.z_adjust,
        );
        match self.target() {
            Some(target) => target.move_to(toolhead_position, speed),
            // No toolhead yet (`klippy:connect` has not run): upstream would
            // crash on `self.toolhead = None`; here the move is refused.
            None => Err(CommandError::new("Printer is not ready")),
        }
    }

    /// Upstream's `get_position` (`bed_tilt.py:26-30`): the toolhead's
    /// position with the plane **subtracted**, so the g-code space reads flat.
    fn position(&self) -> Coord {
        let Some(target) = self.target() else {
            // As above: before connect there is nothing to read.
            return Coord::default();
        };
        let toolhead_position = target.position();
        let adjust = self.adjust();
        let mut position = toolhead_position;
        position.set_axis(
            Z_AXIS,
            toolhead_position.z()
                - toolhead_position.x() * adjust.x_adjust
                - toolhead_position.y() * adjust.y_adjust
                - adjust.z_adjust,
        );
        position
    }
}

impl PrinterObject for BedTilt {
    /// Upstream's `BedTilt` defines no `get_status`.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    /// Kept out of `objects/list`, as an object without `get_status`
    /// upstream.
    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for BedTilt {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let adjust = self.adjust();
        f.debug_struct("BedTilt")
            .field("x_adjust", &adjust.x_adjust)
            .field("y_adjust", &adjust.y_adjust)
            .field("z_adjust", &adjust.z_adjust)
            .finish_non_exhaustive()
    }
}

/// The toolhead behind the transform: what `position` reads and `move_to`
/// feeds (upstream passes `self.toolhead` around directly).
struct ToolheadMove(Arc<ToolHeadObject>);

impl MoveTarget for ToolheadMove {
    fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
        self.0.move_to(position, speed)
    }

    fn position(&self) -> Coord {
        self.0.position().unwrap_or_default()
    }
}

/// The `BED_TILT_CALIBRATE` half of the section — upstream's embedded
/// `BedTiltCalibrate` class (`bed_tilt.py:47-84`). Built only when the
/// section writes `points`; the registered command holds the long-lived
/// reference.
pub struct BedTiltCalibrate {
    /// Drives every round; reads the section's `points`,
    /// `horizontal_move_z` and `speed`.
    probe_helper: Arc<ProbePointsHelper>,
}

impl BedTiltCalibrate {
    /// Wire the helper and register `BED_TILT_CALIBRATE`, which just runs a
    /// round (`bed_tilt.py:50-57`).
    ///
    /// # Errors
    /// When `points` is missing or malformed, holds fewer than three points
    /// (`probe.py:minimum_points`) or the command name is taken.
    fn new(
        config: &ConfigWrapper,
        printer: &Arc<Printer>,
        bedtilt: &Arc<BedTilt>,
    ) -> Result<Arc<Self>, ConfigError> {
        let finalize: ProbePointsFinalize = Arc::new({
            let bedtilt = Arc::clone(bedtilt);
            let printer = Arc::downgrade(printer);
            move |offsets, positions| {
                let printer = printer.upgrade();
                probe_finalize(&bedtilt, printer.as_ref(), offsets, positions);
                None
            }
        });
        let probe_helper = ProbePointsHelper::new(config, printer, finalize)?;
        probe_helper.minimum_points(3)?;
        let calibrate = Arc::new(Self { probe_helper });

        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");
        gcode
            .register_command_with_params(
                "BED_TILT_CALIBRATE",
                {
                    let calibrate = Arc::clone(&calibrate);
                    Arc::new(move |gcmd| {
                        let calibrate = Arc::clone(&calibrate);
                        Box::pin(async move { calibrate.probe_helper.start_probe(gcmd).await })
                    })
                },
                Some("Bed tilt calibration script"),
                &probe_points_params(),
                false,
            )
            .map_err(ConfigError::new)?;
        Ok(calibrate)
    }
}

/// One calibration round: fit the plane through the probed positions, apply
/// it and report it (`bed_tilt.py:61-84`).
///
/// `coordinate_descent` minimises the squared height left over when every
/// point is pushed through `z - x*x_adjust - y*y_adjust - z_adjust`,
/// starting from the current slopes and the probe's `z_offset` as the
/// intercept (upstream's initial `params`). The fitted intercept is still in
/// the **toolhead** frame; dropping `z_offset` and the probe's XY offsets
/// turns it into the bed frame `get_position` subtracts
/// (`bed_tilt.py:82-84`).
fn probe_finalize(
    bedtilt: &BedTilt,
    printer: Option<&Arc<Printer>>,
    offsets: ProbeOffsets,
    positions: &[Coord],
) {
    let z_offset = offsets.z;
    info!("Calculating bed_tilt with: {positions:?}");
    let current = bedtilt.adjust();
    let mut params = [current.x_adjust, current.y_adjust, z_offset];
    info!(
        "Initial bed_tilt parameters: x_adjust: {} y_adjust: {} z_adjust: {}",
        params[0], params[1], params[2]
    );
    coordinate_descent(&mut params, |p| {
        positions
            .iter()
            .map(|pos| {
                let adjusted = pos.z() - pos.x() * p[0] - pos.y() * p[1] - p[2];
                adjusted * adjusted
            })
            .sum()
    });
    let x_adjust = params[0];
    let y_adjust = params[1];
    let z_adjust = params[2] - z_offset - x_adjust * offsets.x - y_adjust * offsets.y;
    info!(
        "Calculated bed_tilt parameters: x_adjust: {x_adjust} y_adjust: {y_adjust} \
         z_adjust: {z_adjust}"
    );
    bedtilt.update_adjust(x_adjust, y_adjust, z_adjust);

    // Log, tell the log rollover, and answer the command
    // (`bed_tilt.py:78-84`).
    let msg = format!("x_adjust: {x_adjust:.6} y_adjust: {y_adjust:.6} z_adjust: {z_adjust:.6}");
    set_rollover_info(SECTION, Some(&format!("bed_tilt: {msg}")));
    if let Some(printer) = printer {
        if let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) {
            gcode.respond_info(
                &format!(
                    "{msg}\nThe above parameters have been applied to the current\n\
                     session. The SAVE_CONFIG command will update the printer\n\
                     config file and restart the printer."
                ),
                true,
            );
        }
    }
}

/// Upstream's `load_config` for `[bed_tilt]` (`bed_tilt.py:86-87`).
///
/// `gcode_move::ensure` stands in for upstream's
/// `load_object(config, 'gcode_move')`: the transform takes the slot before
/// ready, so `gcode_move._handle_ready` leaves it alone
/// (`gcode_move.py:51-58`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let bedtilt = Arc::new(BedTilt::new(config, printer)?);
    bedtilt.register_handlers(printer);
    // Upstream's `if config.get('points', None) is not None`: only a section
    // that can probe gets the command (`bed_tilt.py:18-19`).
    if config.has("points") {
        BedTiltCalibrate::new(config, printer, &bedtilt)?;
    }
    let gcode_move = gcode_move::ensure(printer)?;
    gcode_move.set_move_transform(Arc::clone(&bedtilt) as Arc<dyn MoveTarget>, false)?;
    Ok(bedtilt)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, Config, ConfigSection, ConfigValue};
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[bed_tilt]` section with the given options, as the parser builds it.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("bed_tilt", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A machine with `gcode` and `configfile` registered — the objects
    /// `bed_tilt` reaches for at run time. The loader registers both itself,
    /// so the load-based tests build a bare one instead.
    fn printer() -> Arc<Printer> {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let parsed = Config::from_text("[bed_tilt]\n").unwrap().0;
        printer
            .add_object(
                CONFIGFILE_OBJECT,
                Arc::new(PrinterConfig::new(
                    AccessTracking::shared(),
                    PrinterConfig::raw_config(&parsed),
                )),
            )
            .unwrap();
        printer
    }

    /// A move target that records what it was asked and stands where the last
    /// move left it — the toolhead, as far as the coordinate math cares.
    struct FakeTarget {
        position: Mutex<Coord>,
        moves: Mutex<Vec<(Coord, f64)>>,
    }

    impl FakeTarget {
        fn new(position: Coord) -> Self {
            Self {
                position: Mutex::new(position),
                moves: Mutex::new(Vec::new()),
            }
        }

        fn moves(&self) -> Vec<(Coord, f64)> {
            self.moves.lock().unwrap_or_else(|p| p.into_inner()).clone()
        }
    }

    impl MoveTarget for FakeTarget {
        fn move_to(&self, position: Coord, speed: f64) -> Result<(), CommandError> {
            *self.position.lock().unwrap_or_else(|p| p.into_inner()) = position;
            self.moves
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .push((position, speed));
            Ok(())
        }

        fn position(&self) -> Coord {
            *self.position.lock().unwrap_or_else(|p| p.into_inner())
        }
    }

    /// Point the transform's slot at a fake standing at `position`.
    fn transform_at(bedtilt: &BedTilt, position: Coord) -> Arc<FakeTarget> {
        let fake = Arc::new(FakeTarget::new(position));
        *bedtilt.target.lock().unwrap_or_else(|p| p.into_inner()) =
            Some(Arc::clone(&fake) as Arc<dyn MoveTarget>);
        fake
    }

    /// Capture everything the dispatcher sends to a client.
    fn capture_output(gcode: &Arc<GCodeDispatch>) -> Arc<Mutex<Vec<String>>> {
        let output = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&output);
        gcode.register_output_handler(Arc::new(move |line: &str| {
            sink.lock()
                .unwrap_or_else(|p| p.into_inner())
                .push(line.to_string());
        }));
        output
    }

    fn emitted(output: &Arc<Mutex<Vec<String>>>) -> String {
        output.lock().unwrap_or_else(|p| p.into_inner()).join("\n")
    }

    #[test]
    fn test_the_adjust_options_default_to_zero_and_read_the_configured_values() {
        let printer = printer();

        let defaults = BedTilt::new(&ConfigWrapper::untracked(&section(&[])), &printer).unwrap();
        assert_eq!(
            defaults.adjust(),
            Adjust {
                x_adjust: 0.0,
                y_adjust: 0.0,
                z_adjust: 0.0,
            }
        );

        let configured = BedTilt::new(
            &ConfigWrapper::untracked(&section(&[
                ("x_adjust", "1.5"),
                ("y_adjust", "-2.25"),
                ("z_adjust", "0.125"),
            ])),
            &printer,
        )
        .unwrap();
        assert_eq!(
            configured.adjust(),
            Adjust {
                x_adjust: 1.5,
                y_adjust: -2.25,
                z_adjust: 0.125,
            }
        );
    }

    #[test]
    fn test_position_subtracts_the_tilt_and_move_adds_it_back() {
        let printer = printer();
        let section = section(&[
            ("x_adjust", "0.01"),
            ("y_adjust", "-0.02"),
            ("z_adjust", "0.5"),
        ]);
        let config = ConfigWrapper::untracked(&section);
        let bedtilt = Arc::new(BedTilt::new(&config, &printer).unwrap());
        let fake = transform_at(&bedtilt, Coord::new(10.0, 20.0, 5.0, 0.0));

        let position = bedtilt.position();
        assert_eq!(position.x(), 10.0);
        assert_eq!(position.y(), 20.0);
        assert!(
            (position.z() - 4.8).abs() < 1e-9,
            "5 - 10*0.01 - 20*(-0.02) - 0.5 = 4.8, got {}",
            position.z()
        );

        // Reading and moving back must land the toolhead where it stood.
        bedtilt.move_to(position, 30.0).unwrap();
        let moves = fake.moves();
        assert_eq!(moves.len(), 1);
        assert_eq!(moves[0].1, 30.0);
        assert!(
            (moves[0].0.z() - 5.0).abs() < 1e-9,
            "the plane comes back on the way down: got {}",
            moves[0].0.z()
        );

        // A g-code move names a flat-bed Z; the toolhead gets the plane added.
        bedtilt
            .move_to(Coord::new(10.0, 20.0, 0.0, 1.0), 25.0)
            .unwrap();
        let moves = fake.moves();
        assert!(
            (moves[1].0.z() - 0.2).abs() < 1e-9,
            "0 + 10*0.01 + 20*(-0.02) + 0.5 = 0.2, got {}",
            moves[1].0.z()
        );
    }

    #[test]
    fn test_update_adjust_reanchors_the_g_code_position_and_records_save_config() {
        let printer = printer();
        let gcode_move = gcode_move::ensure(&printer).unwrap();
        let bedtilt =
            Arc::new(BedTilt::new(&ConfigWrapper::untracked(&section(&[])), &printer).unwrap());
        gcode_move
            .set_move_transform(Arc::clone(&bedtilt) as Arc<dyn MoveTarget>, false)
            .unwrap();
        transform_at(&bedtilt, Coord::new(0.0, 0.0, 1.0, 0.0));
        // Anchors `last_position` through the transform: g-code Z reads 1.0.
        printer.send_event(&KlippyEvent::ToolheadSetPosition);

        bedtilt.update_adjust(0.1, -0.2, 0.5);

        assert_eq!(
            bedtilt.adjust(),
            Adjust {
                x_adjust: 0.1,
                y_adjust: -0.2,
                z_adjust: 0.5,
            }
        );
        // Re-read through the new plane: toolhead 1.0 - z_adjust 0.5.
        assert_eq!(gcode_move.status()["position"], json!([0.0, 0.0, 0.5, 0.0]));

        let configfile = printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .unwrap();
        let status = configfile.get_status(0.0);
        assert_eq!(status["save_config_pending"], json!(true));
        assert_eq!(
            status["save_config_pending_items"]["bed_tilt"],
            json!({
                "x_adjust": "0.100000",
                "y_adjust": "-0.200000",
                "z_adjust": "0.500000",
            })
        );
    }

    #[test]
    fn test_without_points_bed_tilt_calibrate_is_never_registered() {
        // A bare loader run: the printer registers `gcode`/`configfile`/`pins`
        // itself, then the section takes the move-transform slot.
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let parsed = Config::from_text("[bed_tilt]\nx_adjust: 0.1\n").unwrap().0;
        printer.load_config(&parsed).unwrap();
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        let output = capture_output(&gcode);

        // An unknown command answers `Ok` with a remark — the remark is the
        // assertion (`gcode.py`'s unknown-command path).
        gcode.run_script_sync("BED_TILT_CALIBRATE").unwrap();

        assert!(
            emitted(&output).contains("Unknown command:\"BED_TILT_CALIBRATE\""),
            "{}",
            emitted(&output)
        );
    }

    #[test]
    fn test_points_register_the_command_and_must_be_at_least_three() {
        // Fewer than three points: the section refuses to load
        // (`probe.py:minimum_points`).
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let parsed = Config::from_text("[bed_tilt]\npoints:\n    0,0\n    10,0\n")
            .unwrap()
            .0;
        let err = printer.load_config(&parsed).unwrap_err();
        assert!(
            err.to_string()
                .contains("Need at least 3 probe points for bed_tilt"),
            "{err}"
        );

        // With three points a handler runs — it stops at this bare machine's
        // missing `manual_probe`, which is a different answer than "unknown".
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let parsed = Config::from_text("[bed_tilt]\npoints:\n    0,0\n    10,0\n    0,10\n")
            .unwrap()
            .0;
        printer.load_config(&parsed).unwrap();
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap();
        printer.send_event(&KlippyEvent::KlippyReady);
        let output = capture_output(&gcode);

        let result = gcode.run_script_sync("BED_TILT_CALIBRATE");

        assert!(
            result.is_err(),
            "the probe helper runs and stops: {result:?}"
        );
        assert!(
            !emitted(&output).contains("Unknown command"),
            "{}",
            emitted(&output)
        );
    }

    #[test]
    fn test_the_calibration_recovers_a_known_plane() {
        let printer = printer();
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap();
        let output = capture_output(&gcode);
        let bedtilt =
            Arc::new(BedTilt::new(&ConfigWrapper::untracked(&section(&[])), &printer).unwrap());

        // Points on one known plane, in toolhead coordinates:
        // z = 1.2 + 0.004x - 0.003y.
        let (x_slope, y_slope, intercept) = (0.004, -0.003, 1.2);
        let positions: Vec<Coord> = [
            (0.0, 0.0),
            (60.0, 0.0),
            (0.0, 50.0),
            (60.0, 50.0),
            (30.0, 25.0),
        ]
        .iter()
        .map(|&(x, y)| Coord::new(x, y, intercept + x_slope * x + y_slope * y, 0.0))
        .collect();
        let offsets = ProbeOffsets {
            x: 1.0,
            y: 2.0,
            z: 0.25,
        };

        probe_finalize(&bedtilt, Some(&printer), offsets, &positions);

        let adjust = bedtilt.adjust();
        assert!((adjust.x_adjust - x_slope).abs() < 1e-4, "{adjust:?}");
        assert!((adjust.y_adjust - y_slope).abs() < 1e-4, "{adjust:?}");
        let expected_z = intercept - offsets.z - x_slope * offsets.x - y_slope * offsets.y;
        assert!(
            (adjust.z_adjust - expected_z).abs() < 1e-4,
            "z_adjust {} vs {expected_z} (probe offsets removed)",
            adjust.z_adjust
        );

        // All three values queued for `SAVE_CONFIG`, as `%.6f`.
        let configfile = printer
            .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
            .unwrap();
        let status = configfile.get_status(0.0);
        assert_eq!(status["save_config_pending"], json!(true));
        assert_eq!(
            status["save_config_pending_items"]["bed_tilt"],
            json!({
                "x_adjust": format!("{:.6}", adjust.x_adjust),
                "y_adjust": format!("{:.6}", adjust.y_adjust),
                "z_adjust": format!("{:.6}", adjust.z_adjust),
            })
        );

        // The report says what was applied and what `SAVE_CONFIG` will do
        // (`bed_tilt.py:81-84`).
        let lines = emitted(&output);
        assert!(
            lines.contains("The above parameters have been applied to the current"),
            "{lines}"
        );
        assert!(
            lines.contains(&format!("x_adjust: {:.6}", adjust.x_adjust)),
            "{lines}"
        );
    }
}
