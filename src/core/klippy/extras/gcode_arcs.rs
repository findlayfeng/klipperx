//! `[gcode_arcs]` — arc support for the `G2`/`G3` commands
//! (upstream `klippy/extras/gcode_arcs.py`).
//!
//! Upstream's `ArcSupport` reads one option, registers five commands and keeps
//! one piece of state:
//!
//! | option | default | role |
//! |---|---|---|
//! | `resolution` | `1.` | millimetres per arc segment (`above=0.0`) |
//!
//! | command | meaning |
//! |---|---|
//! | `G2` | clockwise arc (`_cmd_inner(gcmd, True)`) |
//! | `G3` | counter-clockwise arc (`_cmd_inner(gcmd, False)`) |
//! | `G17` / `G18` / `G19` | select the X/Y, X/Z or Y/Z plane |
//!
//! `G2`/`G3` are *not* moves of their own: they approximate the arc with many
//! short linear segments and hand **each one to `gcode_move`'s `G1` handler**
//! (`gcode_arcs.py:170-173`), so the active transform (`bed_mesh`, …), the
//! `G92` anchor and the extrude/speed factors all apply as they would to a
//! hand-written `G1`. The command words are exactly the ones `_cmd_inner` reads
//! — `X`, `Y`, `Z`, `R`, `I`, `J`, `K`, `E`, `F` — and the words an arc has no
//! use for are *not* invented here: `R` is refused with upstream's own message
//! (`gcode_arcs.py:63-64`) because upstream's planner is I/J/K-only, and `P` is
//! not read at all.
//!
//! The plane only picks which pair the arc turns in; the third axis is helical
//! (linear travel), which is what `axes` below spells out
//! (`gcode_arcs.py:70-77`):
//!
//! | plane | plane axes (`alpha`, `beta`) | helical |
//! |---|---|---|
//! | `G17` (default) | `I`→X, `J`→Y | Z |
//! | `G18` | `I`→X, `K`→Z | Y |
//! | `G19` | `J`→Y, `K`→Z | X |

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::gcode_move::{self, GCodeMove};
use crate::core::klippy::gcode::{
    parse_float, sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::{E_AXIS, X_AXIS, Y_AXIS, Z_AXIS};
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("gcode_arcs", order = 30, load = load_config);

/// Upstream's plane enum (`gcode_arcs.py:14-16`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Plane {
    /// `ARC_PLANE_X_Y` — `G17`, the default (`gcode_arcs.py:40`).
    Xy,
    /// `ARC_PLANE_X_Z` — `G18`.
    Xz,
    /// `ARC_PLANE_Y_Z` — `G19`.
    Yz,
}

impl Plane {
    /// The plane's two axes and the helical one, as upstream's `axes` tuple
    /// (`gcode_arcs.py:71-77`).
    const fn axes(self) -> (usize, usize, usize) {
        match self {
            Plane::Xy => (X_AXIS, Y_AXIS, Z_AXIS),
            Plane::Xz => (X_AXIS, Z_AXIS, Y_AXIS),
            Plane::Yz => (Y_AXIS, Z_AXIS, X_AXIS),
        }
    }
}

/// The `[gcode_arcs]` section (`gcode_arcs.py:ArcSupport`).
#[derive(Debug)]
pub struct GCodeArcs {
    /// `resolution`: millimetres per arc segment — upstream's
    /// `mm_per_arc_segment` (`gcode_arcs.py:31`).
    mm_per_arc_segment: f64,
    /// Upstream's `self.plane` (`gcode_arcs.py:40`): the plane `G17`/`G18`/
    /// `G19` write and `G2`/`G3` read.
    plane: Mutex<Plane>,
    /// Upstream's `self.gcode_move`, resolved at load (`gcode_arcs.py:34`); the
    /// segments go through its `G1` handler.
    gcode_move: Arc<GCodeMove>,
    /// Upstream's `self.gcode` (`gcode_arcs.py:35`), which builds each
    /// segment's command. Weak: the dispatcher's table holds this object.
    gcode: Weak<GCodeDispatch>,
}

impl GCodeArcs {
    /// Read the section.
    ///
    /// # Errors
    /// An unparseable `resolution`, or one at or below `0.` — upstream's
    /// `config.getfloat('resolution', 1., above=0.0)`
    /// (`gcode_arcs.py:31`) with the shared wording — and a `gcode_move` that
    /// cannot be created.
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let mm_per_arc_segment =
            config.get_float_bounded("resolution", Some(1.), None, None, Some(0.0), None)?;
        // Upstream's `printer.load_object(config, 'gcode_move')`
        // (`gcode_arcs.py:34`).
        let gcode_move = gcode_move::ensure(printer)?;
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        Ok(Self {
            mm_per_arc_segment,
            plane: Mutex::new(Plane::Xy),
            gcode_move,
            gcode: Arc::downgrade(&gcode),
        })
    }

    /// The configured millimetres per arc segment.
    pub fn resolution(&self) -> f64 {
        self.mm_per_arc_segment
    }

    /// The plane lock, poisoning treated as continued unwinding
    /// (`gcode_move.rs` convention).
    fn lock(&self) -> MutexGuard<'_, Plane> {
        self.plane
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    /// The selected plane (`gcode_arcs.py:40`).
    fn plane(&self) -> Plane {
        *self.lock()
    }

    /// `G17`/`G18`/`G19` (`gcode_arcs.py:52-58`).
    fn set_plane(&self, plane: Plane) {
        *self.lock() = plane;
    }

    /// Upstream's `_cmd_inner` (`gcode_arcs.py:60-94`): read the words, pick the
    /// plane's axes and hand the geometry to [`Self::plan_arc`].
    ///
    /// # Errors
    /// An unparseable word; a relative-move mode (`G91`) or an `R` word, both
    /// of which upstream refuses with its own wording; and an arc with no I/J/K
    /// offset at all.
    fn cmd_inner(&self, gcmd: &GcodeCommand, clockwise: bool) -> Result<(), CommandError> {
        let status = self.gcode_move.status();
        // Upstream's `gcodestatus['absolute_coordinates']`
        // (`gcode_arcs.py:56-58`): the planned segments are absolute g-code
        // coordinates, so a relative mode cannot be carried through.
        if !flag(&status, "absolute_coordinates") {
            return Err(CommandError::new(
                "G2/G3 does not support relative move mode",
            ));
        }
        let absolute_extrude = flag(&status, "absolute_extrude");
        let current_pos = [
            gcode_axis(&status, X_AXIS),
            gcode_axis(&status, Y_AXIS),
            gcode_axis(&status, Z_AXIS),
            gcode_axis(&status, E_AXIS),
        ];

        // Parse parameters (`gcode_arcs.py:61-66`). An unnamed axis keeps its
        // g-code position, so an arc that only leaves the plane moves helically.
        let target_pos = [
            gcmd.get_float_default("X", current_pos[X_AXIS])?,
            gcmd.get_float_default("Y", current_pos[Y_AXIS])?,
            gcmd.get_float_default("Z", current_pos[Z_AXIS])?,
        ];

        if optional_float(gcmd, "R")?.is_some() {
            return Err(CommandError::new("G2/G3 does not support R moves"));
        }

        // Determine the plane coordinates and the helical axis
        // (`gcode_arcs.py:69-77`). `I`/`J` are read in every plane and `K` only
        // outside XY, exactly as upstream reads them; the plane then picks which
        // pair is the offset.
        let i = gcmd.get_float_default("I", 0.)?;
        let j = gcmd.get_float_default("J", 0.)?;
        let plane = self.plane();
        let (offset, axes) = match plane {
            Plane::Xy => ((i, j), plane.axes()),
            Plane::Xz => ((i, gcmd.get_float_default("K", 0.)?), plane.axes()),
            Plane::Yz => ((j, gcmd.get_float_default("K", 0.)?), plane.axes()),
        };

        // `if not (asPlanar[0] or asPlanar[1])` (`gcode_arcs.py:79-80`).
        if offset.0 == 0. && offset.1 == 0. {
            return Err(CommandError::new("G2/G3 requires IJ, IK or JK parameters"));
        }

        self.plan_arc(
            &current_pos,
            target_pos,
            offset,
            clockwise,
            gcmd,
            absolute_extrude,
            axes,
        )
    }

    /// Upstream's `planArc` (`gcode_arcs.py:104-175`, itself Marlin's
    /// `plan_arc`): turn the arc into `max(1, floor(mm_of_travel /
    /// mm_per_arc_segment))` linear `G1` moves.
    ///
    /// `current_pos` is the g-code position (`X`, `Y`, `Z`, `E`), `target_pos`
    /// the arc's end (`X`, `Y`, `Z`), `offset` the `(alpha, beta)` centre offset
    /// in words, and `axes` the plane's `(alpha, beta, helical)` position
    /// indices.
    ///
    /// # Errors
    /// An unparseable `E`/`F`, and whatever [`GCodeMove::cmd_g1`] reports for a
    /// segment.
    // The parameters are upstream's, one for one, so the port can be read
    // beside `planArc`'s signature.
    #[allow(clippy::too_many_arguments)]
    fn plan_arc(
        &self,
        current_pos: &[f64; 4],
        target_pos: [f64; 3],
        offset: (f64, f64),
        clockwise: bool,
        gcmd: &GcodeCommand,
        absolute_extrude: bool,
        axes: (usize, usize, usize),
    ) -> Result<(), CommandError> {
        let (alpha_axis, beta_axis, helical_axis) = axes;

        // Radius vector from center to current location
        let mut r_p = -offset.0;
        let mut r_q = -offset.1;

        // Determine angular travel
        let center_p = current_pos[alpha_axis] - r_p;
        let center_q = current_pos[beta_axis] - r_q;
        let rt_alpha = target_pos[alpha_axis] - center_p;
        let rt_beta = target_pos[beta_axis] - center_q;
        let mut angular_travel =
            (r_p * rt_beta - r_q * rt_alpha).atan2(r_p * rt_alpha + r_q * rt_beta);
        if angular_travel < 0. {
            angular_travel += 2. * std::f64::consts::PI;
        }
        if clockwise {
            angular_travel -= 2. * std::f64::consts::PI;
        }

        if angular_travel == 0.
            && current_pos[alpha_axis] == target_pos[alpha_axis]
            && current_pos[beta_axis] == target_pos[beta_axis]
        {
            // Make a circle if the angular rotation is 0 and the target is
            // current position (`gcode_arcs.py:114-120`).
            angular_travel = 2. * std::f64::consts::PI;
        }

        // Determine number of segments (`gcode_arcs.py:122-131`). An arc
        // shorter than one segment is a single line.
        let linear_travel = target_pos[helical_axis] - current_pos[helical_axis];
        let radius = r_p.hypot(r_q);
        let flat_mm = radius * angular_travel;
        let mm_of_travel = if linear_travel != 0. {
            flat_mm.hypot(linear_travel)
        } else {
            flat_mm.abs()
        };
        let segments = (mm_of_travel / self.mm_per_arc_segment).floor().max(1.);

        // Generate coordinates (`gcode_arcs.py:133-134`).
        let theta_per_segment = angular_travel / segments;
        let linear_per_segment = linear_travel / segments;

        // `asE` and `asF` (`gcode_arcs.py:136-137`), read after the geometry as
        // upstream reads them.
        let as_e = optional_float(gcmd, "E")?;
        let as_f = optional_float(gcmd, "F")?;

        let mut e_base = 0.;
        let e_per_move = match as_e {
            Some(as_e) => {
                if absolute_extrude {
                    e_base = current_pos[E_AXIS];
                }
                (as_e - e_base) / segments
            }
            None => 0.,
        };

        let gcode = self
            .gcode
            .upgrade()
            .ok_or_else(|| CommandError::new("gcode is not available"))?;

        for i in 1..=segments as i64 {
            let dist_helical = i as f64 * linear_per_segment;
            let c_theta = i as f64 * theta_per_segment;
            let cos_ti = c_theta.cos();
            let sin_ti = c_theta.sin();
            r_p = -offset.0 * cos_ti + offset.1 * sin_ti;
            r_q = -offset.0 * sin_ti - offset.1 * cos_ti;

            let mut c = [0.; 3];
            c[alpha_axis] = center_p + r_p;
            c[beta_axis] = center_q + r_q;
            c[helical_axis] = current_pos[helical_axis] + dist_helical;

            // The last segment lands exactly on the target, so float error does
            // not accumulate into the commanded end point (`gcode_arcs.py:155-156`).
            if i == segments as i64 {
                c = target_pos;
            }

            // Convert coordinates into a `G1` command (`gcode_arcs.py:157-173`).
            let mut g1_params = HashMap::new();
            g1_params.insert("X".to_string(), number(c[X_AXIS]));
            g1_params.insert("Y".to_string(), number(c[Y_AXIS]));
            g1_params.insert("Z".to_string(), number(c[Z_AXIS]));
            if e_per_move != 0. {
                g1_params.insert("E".to_string(), number(e_base + e_per_move));
                if absolute_extrude {
                    e_base += e_per_move;
                }
            }
            if let Some(as_f) = as_f {
                g1_params.insert("F".to_string(), number(as_f));
            }
            let g1_gcmd = gcode.create_gcode_command("G1", "G1", g1_params);
            self.gcode_move.cmd_g1(&g1_gcmd)?;
        }
        Ok(())
    }

    /// Register `G2`/`G3` and the plane selectors, capturing a handle to this
    /// object (`gcode_arcs.py:36-42`).
    fn register_commands(self: &Arc<Self>, printer: &Arc<Printer>) -> Result<(), ConfigError> {
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` before any section");
        type Command = fn(&GCodeArcs, &GcodeCommand) -> Result<(), CommandError>;
        /// The words `_cmd_inner` reads, in upstream's order
        /// (`gcode_arcs.py:61-89`): the target axes, then `R`, the offsets, and
        /// the extruder/feed words.
        const ARC_PARAMS: &[&str] = &["X", "Y", "Z", "R", "I", "J", "K", "E", "F"];
        const COMMANDS: &[(&str, Command, &[&str])] = &[
            ("G2", cmd_g2, ARC_PARAMS),
            ("G3", cmd_g3, ARC_PARAMS),
            ("G17", cmd_g17, &[]),
            ("G18", cmd_g18, &[]),
            ("G19", cmd_g19, &[]),
        ];
        for (name, command, params) in COMMANDS {
            let handler: CommandHandler = {
                let object = Arc::clone(self);
                sync(move |gcmd| command(&object, gcmd))
            };
            gcode
                .register_command_with_params(name, handler, None, params, false)
                .map_err(ConfigError::new)?;
        }
        Ok(())
    }
}

impl PrinterObject for GCodeArcs {
    /// Upstream's `ArcSupport` defines no `get_status`.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }
}

/// A `true`/`false` field of `gcode_move`'s status, with upstream's Python
/// default when the key is somehow missing.
fn flag(status: &Value, key: &str) -> bool {
    status.get(key).and_then(Value::as_bool).unwrap_or(true)
}

/// One axis of `gcode_move`'s `gcode_position`
/// (`gcodestatus['gcode_position'][axis]`, `gcode_arcs.py:59-62`).
fn gcode_axis(status: &Value, axis: usize) -> f64 {
    status
        .get("gcode_position")
        .and_then(Value::as_array)
        .and_then(|axes| axes.get(axis))
        .and_then(Value::as_f64)
        .unwrap_or(0.)
}

/// Upstream's `gcmd.get_float(name, None)`: an absent word is `None`, one that
/// does not parse is an error (`klippy/gcode.py:65-90`).
fn optional_float(gcmd: &GcodeCommand, name: &str) -> Result<Option<f64>, CommandError> {
    match gcmd.get_command_parameters().get(name) {
        None => Ok(None),
        Some(raw) => parse_float(raw).map(Some).ok_or_else(|| {
            CommandError::new(format!(
                "Error on '{}': unable to parse {}",
                gcmd.commandline(),
                raw
            ))
        }),
    }
}

/// A coordinate word's text.
///
/// Upstream hands the planned float straight to `gcode_move.cmd_G1`
/// (`gcode_arcs.py:167-172`); this port builds the command through
/// [`GCodeDispatch::create_gcode_command`], so the text must read back as the
/// same `f64` — which the shortest round-trip `Display` of `f64` guarantees.
fn number(value: f64) -> String {
    format!("{value}")
}

/// `G2` (`gcode_arcs.py:44-45`): a clockwise arc.
fn cmd_g2(object: &GCodeArcs, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    object.cmd_inner(gcmd, true)
}

/// `G3` (`gcode_arcs.py:47-48`): a counter-clockwise arc.
fn cmd_g3(object: &GCodeArcs, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    object.cmd_inner(gcmd, false)
}

/// `G17` (`gcode_arcs.py:50-51`): the X/Y plane.
fn cmd_g17(object: &GCodeArcs, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
    object.set_plane(Plane::Xy);
    Ok(())
}

/// `G18` (`gcode_arcs.py:53-54`): the X/Z plane.
fn cmd_g18(object: &GCodeArcs, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
    object.set_plane(Plane::Xz);
    Ok(())
}

/// `G19` (`gcode_arcs.py:56-57`): the Y/Z plane.
fn cmd_g19(object: &GCodeArcs, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
    object.set_plane(Plane::Yz);
    Ok(())
}

/// The factory `section!` names (`gcode_arcs.py:load_config`).
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let arcs = Arc::new(GCodeArcs::new(config, printer)?);
    arcs.register_commands(printer)?;
    Ok(arcs)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{AccessTracking, ConfigSection, ConfigValue};
    use crate::core::klippy::extras::gcode_move::MoveTarget;
    use crate::core::klippy::mathutil::Coord;
    use crate::core::klippy::reactor::ManualReactor;

    /// A `[gcode_arcs]` section with the given options, as the parser would
    /// build it.
    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("gcode_arcs", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    /// A wrapper that records into `access`, as the loader builds it.
    fn wrapper<'a>(section: &'a ConfigSection, access: &Arc<AccessTracking>) -> ConfigWrapper<'a> {
        ConfigWrapper::new(section, Arc::clone(access))
    }

    /// Load one `[gcode_arcs]` onto a printer that already has `gcode`.
    fn load(
        printer: &Arc<Printer>,
        options: &[(&str, &str)],
    ) -> Result<Arc<dyn PrinterObject>, ConfigError> {
        load_config(&ConfigWrapper::untracked(&section(options)), printer)
    }

    /// A move target that records what it was asked and stands where the last
    /// move left it — the toolhead, as far as the arc cares.
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

        fn clear(&self) {
            self.moves
                .lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .clear();
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
    /// which is what `G2`/`G3` plan through.
    fn machine() -> (Arc<Printer>, Arc<GCodeDispatch>, Arc<RecordingTarget>) {
        use crate::core::klippy::event::KlippyEvent;

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
        (printer, gcode, target)
    }

    /// Move to `gcode`'s starting point and forget that move, so a test sees
    /// only the arc's segments.
    fn start_at(gcode: &Arc<GCodeDispatch>, target: &Arc<RecordingTarget>, line: &str) {
        gcode
            .run_script_sync(&format!("G90\nG1 {line}"))
            .expect("the setup move runs");
        target.clear();
    }

    /// The default `1.` is used and recorded when `resolution` is absent, so
    /// the access check accepts a bare `[gcode_arcs]` (`config/validate.rs`)
    /// — the corpus's own `gcode_arcs.cfg` writes no options at all
    /// (`gcode_arcs.py:31`).
    #[test]
    fn a_missing_resolution_defaults_to_one_and_is_recorded() {
        let sect = section(&[]);
        let access = AccessTracking::shared();
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let arcs =
            GCodeArcs::new(&wrapper(&sect, &access), &printer).expect("the empty section loads");
        assert_eq!(arcs.resolution(), 1.0);
        assert!(access.contains("gcode_arcs", "resolution"));
    }

    /// A written `resolution` is parsed and recorded as the number
    /// (`gcode_arcs.py:31`).
    #[test]
    fn a_written_resolution_is_parsed_and_recorded() {
        let sect = section(&[("resolution", "0.5")]);
        let access = AccessTracking::shared();
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let arcs = GCodeArcs::new(&wrapper(&sect, &access), &printer).expect("0.5 parses");
        assert_eq!(arcs.resolution(), 0.5);
        assert!(access.contains("gcode_arcs", "resolution"));
    }

    /// `above=0.0` refuses `0.` and negatives with the shared bound wording,
    /// and a non-number keeps the parser's wording (`gcode_arcs.py:31`,
    /// `configfile.py:44/55`).
    #[test]
    fn a_resolution_at_or_below_zero_and_a_non_number_are_refused() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));

        let sect = section(&[("resolution", "0")]);
        let access = AccessTracking::shared();
        assert_eq!(
            GCodeArcs::new(&wrapper(&sect, &access), &printer)
                .unwrap_err()
                .to_string(),
            "Option 'resolution' in section 'gcode_arcs' must be above 0"
        );

        let sect = section(&[("resolution", "-1")]);
        let access = AccessTracking::shared();
        assert_eq!(
            GCodeArcs::new(&wrapper(&sect, &access), &printer)
                .unwrap_err()
                .to_string(),
            "Option 'resolution' in section 'gcode_arcs' must be above 0"
        );

        let sect = section(&[("resolution", "abc")]);
        let access = AccessTracking::shared();
        assert_eq!(
            GCodeArcs::new(&wrapper(&sect, &access), &printer)
                .unwrap_err()
                .to_string(),
            "Unable to parse option 'resolution' in section 'gcode_arcs'"
        );
    }

    /// The section loads through the ordinary loader: it is claimed by its
    /// factory, the default `resolution` is recorded, the object lands under
    /// the section's own name, and `gcode_move` is created for it (upstream's
    /// `load_object`, `gcode_arcs.py:34`).
    #[test]
    fn the_section_loads_through_the_loader() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let (config, _) = crate::core::klippy::config::Config::from_text("[gcode_arcs]\n")
            .expect("the section parses");
        printer.load_config(&config).expect("the section loads");

        assert_eq!(
            printer.objects(),
            [
                "gcode",
                "configfile",
                "pins",
                "gcode_move",
                "gcode_arcs",
                "gcode_macro",
                "idle_timeout"
            ]
        );
        let object = printer
            .lookup_object("gcode_arcs")
            .expect("the object is registered");
        assert_eq!(object.get_status(0.0), json!({}));
    }

    /// The five commands are registered with the words `_cmd_inner` reads
    /// (`gcode_arcs.py:36-42`, `:61-89`) — and with no word it does not read
    /// (`P`, say).
    #[test]
    fn the_five_commands_are_registered_with_their_words() {
        let (printer, gcode, _target) = machine();
        load(&printer, &[]).unwrap();

        let commands = gcode.get_status(0.0)["commands"].clone();
        assert_eq!(
            commands["G2"]["parameters"],
            json!(["X", "Y", "Z", "R", "I", "J", "K", "E", "F"])
        );
        assert_eq!(commands["G3"]["parameters"], commands["G2"]["parameters"]);
        for plane_selector in ["G17", "G18", "G19"] {
            assert_eq!(commands[plane_selector]["parameters"], json!([]));
        }
    }

    /// A start that equals the target with a non-zero offset closes the circle
    /// (`gcode_arcs.py:114-120`): `2π` of travel, `floor(2π·r / 1.) = 62`
    /// segments, every point on the circle, and the last one exactly on the
    /// target.
    #[test]
    fn a_start_equal_to_the_target_is_a_full_circle() {
        let (printer, gcode, target) = machine();
        load(&printer, &[]).unwrap();
        start_at(&gcode, &target, "X20 Y20");

        gcode.run_script_sync("G2 X20 Y20 I10 J0").unwrap();

        let moves = target.moves();
        assert_eq!(moves.len(), 62, "floor(2π·10 / 1) segments");
        // The centre is the start plus the offset: (20+10, 20).
        for (position, _speed) in &moves {
            let radius = (position.x() - 30.).hypot(position.y() - 20.);
            assert!(
                (radius - 10.).abs() < 1e-9,
                "segment at {position:?} is not on the circle"
            );
            assert_eq!(position.z(), 0., "the helical axis does not move");
        }
        let last = moves.last().expect("at least one segment").0;
        assert_eq!((last.x(), last.y()), (20., 20.));
    }

    /// `G3` turns the other way: the same start and offset sweep the other
    /// half of the circle's sign (`gcode_arcs.py:106-112`).
    #[test]
    fn g3_sweeps_the_opposite_way() {
        let (printer, gcode, target) = machine();
        load(&printer, &[]).unwrap();
        start_at(&gcode, &target, "X30 Y10");

        gcode.run_script_sync("G3 X30 Y30 I-10 J0").unwrap();
        let clockwise_first = {
            gcode.run_script_sync("G2 X30 Y10 I-10 J0").unwrap();
            target.moves()
        };

        // A half circle each way over the same centre and radius: the segment
        // count is `floor(π·10)`.
        assert_eq!(clockwise_first.len(), 31);
        assert_eq!(clockwise_first.last().unwrap().0.x(), 30.);
        assert_eq!(clockwise_first.last().unwrap().0.y(), 10.);
        // Counter-clockwise from the bottom of the circle goes through X > 30
        // first; clockwise goes through X < 30 first.
        assert!(clockwise_first[0].0.x() < 30.);
    }

    /// `R` is refused with upstream's own message (`gcode_arcs.py:63-64`):
    /// upstream's planner is I/J/K-only, so no segment is queued.
    #[test]
    fn an_r_word_is_refused() {
        let (printer, gcode, target) = machine();
        load(&printer, &[]).unwrap();
        start_at(&gcode, &target, "X20 Y20");

        let err = gcode.run_script_sync("G2 X20 Y20 I10 J0 R5").unwrap_err();

        assert_eq!(err.message(), "G2/G3 does not support R moves");
        assert!(target.moves().is_empty(), "no move was queued");
    }

    /// An arc with no plane offset at all is refused (`gcode_arcs.py:79-80`).
    #[test]
    fn an_arc_without_an_offset_is_refused() {
        let (printer, gcode, target) = machine();
        load(&printer, &[]).unwrap();
        start_at(&gcode, &target, "X20 Y20");

        let err = gcode.run_script_sync("G2 X40 Y20").unwrap_err();

        assert_eq!(err.message(), "G2/G3 requires IJ, IK or JK parameters");
        assert!(target.moves().is_empty());
    }

    /// A relative XYZ mode cannot be planned (`gcode_arcs.py:56-58`).
    #[test]
    fn a_relative_move_mode_is_refused() {
        let (printer, gcode, target) = machine();
        load(&printer, &[]).unwrap();
        gcode.run_script_sync("G90\nG1 X20 Y20\nG91").unwrap();
        target.clear();

        let err = gcode.run_script_sync("G2 X10 Y10 I10 J0").unwrap_err();

        assert_eq!(err.message(), "G2/G3 does not support relative move mode");
        assert!(target.moves().is_empty());
    }

    /// An unnamed `I`/`J` is `0.` (`gcode_arcs.py:71-73`), so `G2 X20 Y20 J10`
    /// plans the same arc as the explicit `I0`.
    #[test]
    fn an_unnamed_offset_word_is_zero() {
        let (printer, gcode, target) = machine();
        load(&printer, &[]).unwrap();
        start_at(&gcode, &target, "X20 Y20");

        gcode.run_script_sync("G2 X20 Y20 J10").unwrap();
        let without_i = target.moves();
        target.clear();
        gcode.run_script_sync("G90\nG1 X20 Y20").unwrap();
        target.clear();
        gcode.run_script_sync("G2 X20 Y20 I0 J10").unwrap();

        assert_eq!(without_i.len(), 62);
        assert_eq!(without_i, target.moves());
    }

    /// The segment count is upstream's
    /// `max(1, floor(hypot(flat, helical) / resolution))`
    /// (`gcode_arcs.py:122-131`): a helical full circle has longer travel than
    /// the flat one, and a finer `resolution` splits it further.
    #[test]
    fn the_segment_count_follows_the_travel_formula() {
        let (printer, gcode, target) = machine();
        load(&printer, &[]).unwrap();
        start_at(&gcode, &target, "X20 Y20 Z0");

        // Full circle of radius 10 with 10 mm of Z behind it:
        // hypot(2π·10, 10) = 63.62 → 63 segments.
        gcode.run_script_sync("G2 X20 Y20 Z10 I10 J0").unwrap();
        assert_eq!(target.moves().len(), 63);
        assert_eq!(target.moves().last().unwrap().0.z(), 10.);

        // The same circle at a finer resolution: 2π·10 / 0.5 = 125.66 → 125.
        let (printer, gcode, target) = machine();
        load(&printer, &[("resolution", "0.5")]).unwrap();
        start_at(&gcode, &target, "X20 Y20");
        gcode.run_script_sync("G2 X20 Y20 I10 J0").unwrap();
        assert_eq!(target.moves().len(), 125);
    }

    /// `G18` turns in X/Z and leaves Y helical (`gcode_arcs.py:74-76`): the
    /// arc bulges in Z, which an XY arc of the same words cannot do.
    #[test]
    fn g18_turns_in_the_x_z_plane() {
        let (printer, gcode, target) = machine();
        load(&printer, &[]).unwrap();
        start_at(&gcode, &target, "X0 Y7 Z0");

        gcode.run_script_sync("G18\nG2 X20 Z0 I10 K0").unwrap();

        let moves = target.moves();
        assert!(!moves.is_empty());
        for (position, _speed) in &moves {
            assert_eq!(position.y(), 7., "Y is the helical axis and does not turn");
            assert!(position.z() >= 0., "the arc bulges in Z");
        }
        let last = moves.last().unwrap().0;
        assert_eq!((last.x(), last.z()), (20., 0.));
        assert!(moves.iter().any(|(position, _)| position.z() > 1.));
    }

    /// `G19` turns in Y/Z and leaves X helical (`gcode_arcs.py:76-77`).
    #[test]
    fn g19_turns_in_the_y_z_plane() {
        let (printer, gcode, target) = machine();
        load(&printer, &[]).unwrap();
        start_at(&gcode, &target, "X7 Y0 Z0");

        gcode.run_script_sync("G19\nG2 Y20 Z0 J10 K0").unwrap();

        let moves = target.moves();
        assert!(!moves.is_empty());
        for (position, _speed) in &moves {
            assert_eq!(position.x(), 7., "X is the helical axis and does not turn");
            assert!(position.z() >= 0., "the arc bulges in Z");
        }
        let last = moves.last().unwrap().0;
        assert_eq!((last.y(), last.z()), (20., 0.));
        assert!(moves.iter().any(|(position, _)| position.z() > 1.));
    }

    /// `G17` selects XY again after `G18` (`gcode_arcs.py:50-58`), and a plane
    /// selector itself moves nothing.
    #[test]
    fn g17_selects_the_x_y_plane_again() {
        let (printer, gcode, target) = machine();
        load(&printer, &[]).unwrap();
        start_at(&gcode, &target, "X0 Y0 Z0");

        gcode.run_script_sync("G18\nG17").unwrap();
        assert!(target.moves().is_empty(), "a plane selector does not move");

        gcode.run_script_sync("G2 X20 Y0 I10 J0").unwrap();
        let moves = target.moves();
        assert!(!moves.is_empty());
        for (position, _speed) in &moves {
            assert_eq!(position.z(), 0., "Z is the helical axis again");
        }
        assert!(moves.iter().any(|(position, _)| position.y() > 1.));
    }

    /// With absolute extrusion (the default `M82`) the first segment carries
    /// the arc's share of the E word and the last lands on it
    /// (`gcode_arcs.py:139-145,158-161`).
    #[test]
    fn an_absolute_e_word_is_split_across_the_segments() {
        let (printer, gcode, target) = machine();
        load(&printer, &[]).unwrap();
        start_at(&gcode, &target, "X20 Y20");

        gcode.run_script_sync("G2 X20 Y20 I10 J0 E1").unwrap();

        let moves = target.moves();
        assert_eq!(moves.len(), 62);
        assert!((moves[0].0.e() - 1.0 / 62.0).abs() < 1e-12);
        assert!((moves.last().unwrap().0.e() - 1.0).abs() < 1e-12);
    }

    /// With relative extrusion (`M83`) every segment carries its own share
    /// (`gcode_arcs.py:138-145`).
    #[test]
    fn a_relative_e_word_is_per_segment() {
        let (printer, gcode, target) = machine();
        load(&printer, &[]).unwrap();
        start_at(&gcode, &target, "X20 Y20");
        gcode.run_script_sync("M83").unwrap();

        gcode.run_script_sync("G2 X20 Y20 I10 J0 E1").unwrap();

        let moves = target.moves();
        assert_eq!(moves.len(), 62);
        for (position, _speed) in &moves {
            assert!((position.e() - 1.0 / 62.0).abs() < 1e-12);
        }
    }

    /// `F` reaches every segment (`gcode_arcs.py:164-165`) and so sets the
    /// move speed `gcode_move` remembers.
    #[test]
    fn an_f_word_sets_every_segments_speed() {
        let (printer, gcode, target) = machine();
        load(&printer, &[]).unwrap();
        start_at(&gcode, &target, "X20 Y20");

        gcode.run_script_sync("G2 X20 Y20 I10 J0 F600").unwrap();

        let moves = target.moves();
        assert_eq!(moves.len(), 62);
        for (_position, speed) in &moves {
            assert_eq!(*speed, 10., "600 mm/min is 10 mm/s");
        }
    }
}
