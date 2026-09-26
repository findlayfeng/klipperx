//! `kinematics: winch` — cable-winch robots.
//!
//! Upstream keeps this in two places, which is why this file carries both:
//! the position solver is `winch_stepper_alloc` (`chelper/kin_winch.c`), and the
//! kinematics is `WinchKinematics` (`klippy/kinematics/winch.py`). Each winch
//! anchors its cable at a fixed point (`anchor_x` / `anchor_y` / `anchor_z` on
//! the cable's `[stepper_*]` section) and drives the cable length to that
//! anchor; going the other way, the carriage is where the first three cables'
//! spheres intersect (upstream's `mathutil.trilateration`, shared with delta).
//!
//! **Placement.** The kinematics lives here rather than in
//! [`super::kinematics`] so the cartesian family's code stays untouched. The
//! glue that *does* touch shared code is kept minimal: the section declarations
//! (`extras::stepper`), the family name and the solver install
//! (`extras::toolhead`), and the [`Kinematics`] impl below.
//!
//! # What is deliberately absent
//!
//! Upstream's winch is a minimal, experimental kinematics and this port copies
//! that as-is: homing is not implemented (see [`WinchKinematics::home`]), there
//! are no boundary checks or speed limits ([`WinchKinematics::check_move`]
//! accepts every move), and the status always reports `xyz` homed.
//! `config/example-winch.cfg` says the same thing to the operator: jog to
//! `0, 0, 0` by hand and then issue `G28`.

use std::collections::HashMap;

use serde_json::{json, Value};

use super::delta::trilateration;
use super::itersolve::{AxisFlags, PositionFn};
use super::kinematics::{HomingState, Kinematics, MoveContext};
use super::trapq::MoveSegment;
use crate::core::klippy::gcode::CommandError;
use crate::core::klippy::mathutil::Coord;

// ===========================================================================
// winch_stepper_alloc (chelper/kin_winch.c)
// ===========================================================================

/// `winch_stepper_alloc(anchor_x, anchor_y, anchor_z)`
/// (`chelper/kin_winch.c:32-42`): the cable length from the carriage to the
/// anchor, `sqrt(dx² + dy² + dz²)`.
///
/// The anchor rides the bound [`PositionFn`] as `[anchor_x, anchor_y,
/// anchor_z, 0, 0, 0]`, the way `setup_itersolve('winch_stepper_alloc', *a)`
/// passes it (`kinematics/winch.py:20`); the parameter block is six wide
/// (rotary delta's shoulders use the extra slots), so the unused tail is zero.
pub fn winch_position_fn(anchor: [f64; 3]) -> PositionFn {
    PositionFn::bind(
        winch_calc_position,
        [anchor[0], anchor[1], anchor[2], 0., 0., 0.],
    )
}

/// The solver body (`winch_stepper_calc_position`, `kin_winch.c:17-26`).
fn winch_calc_position(segment: &MoveSegment, move_time: f64, anchor: &[f64; 6]) -> f64 {
    let coord = segment.coord(move_time);
    let dx = anchor[0] - coord.x();
    let dy = anchor[1] - coord.y();
    let dz = anchor[2] - coord.z();
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// A cable follows the toolhead on every axis (`AF_X | AF_Y | AF_Z`,
/// `kin_winch.c:39`).
pub fn winch_active_flags() -> AxisFlags {
    AxisFlags::X.union(AxisFlags::Y).union(AxisFlags::Z)
}

// ===========================================================================
// WinchKinematics (kinematics/winch.py)
// ===========================================================================

/// The cable-winch kinematics (`WinchKinematics`, `kinematics/winch.py:9-45`).
///
/// Reverse mapping only uses the first three cables (`calc_position`,
/// `winch.py:21-24`); any further cables still drive their own steppers, so the
/// section list is kept whole.
#[derive(Debug, Clone)]
pub struct WinchKinematics {
    /// Each cable's stepper name, in section order (`stepper_a` first).
    names: Vec<String>,
    /// Each cable's anchor, in the same order.
    anchors: Vec<[f64; 3]>,
    /// The anchor-coordinate bounding box, for `get_status` (`winch.py:19-20`).
    axes_min: Coord,
    axes_max: Coord,
}

impl WinchKinematics {
    /// Build the kinematics from the cables' names and anchors
    /// (`WinchKinematics.__init__`, `kinematics/winch.py:11-20`).
    ///
    /// `cables` is each cable's `(stepper name, anchor)` pair, in section
    /// order. The bounding box is the `min`/`max` of each anchor coordinate.
    ///
    /// # Panics
    /// When fewer than three cables are given. Upstream always reads
    /// `stepper_a/b/c` (`with a for a in 'abc'` are never skipped,
    /// `winch.py:15-17`), and the reverse mapping needs three spheres to
    /// intersect.
    pub fn new(cables: Vec<(String, [f64; 3])>) -> Self {
        assert!(
            cables.len() >= 3,
            "the winch kinematics needs at least stepper_a/b/c"
        );
        let (names, anchors): (Vec<String>, Vec<[f64; 3]>) = cables.into_iter().unzip();
        let mut axes_min = [f64::INFINITY; 3];
        let mut axes_max = [f64::NEG_INFINITY; 3];
        for anchor in &anchors {
            for (axis, value) in anchor.iter().enumerate() {
                axes_min[axis] = axes_min[axis].min(*value);
                axes_max[axis] = axes_max[axis].max(*value);
            }
        }
        let (min_x, min_y, min_z) = (axes_min[0], axes_min[1], axes_min[2]);
        let (max_x, max_y, max_z) = (axes_max[0], axes_max[1], axes_max[2]);
        Self {
            names,
            anchors,
            axes_min: Coord::new(min_x, min_y, min_z, 0.0),
            axes_max: Coord::new(max_x, max_y, max_z, 0.0),
        }
    }
}

impl Kinematics for WinchKinematics {
    fn calc_position(&self, stepper_positions: &HashMap<String, f64>) -> [Option<f64>; 3] {
        // `calc_position` of `kinematics/winch.py:21-24`: only the first three
        // cables give the carriage position, and their sphere radii are the
        // squared cable lengths. A missing cable leaves the whole position
        // unknown (upstream would raise a `KeyError`).
        let mut lengths = [0.0; 3];
        for (slot, name) in self.names.iter().take(3).enumerate() {
            match stepper_positions.get(name) {
                Some(length) => lengths[slot] = *length,
                None => return [None; 3],
            }
        }
        let radius2 = [
            lengths[0] * lengths[0],
            lengths[1] * lengths[1],
            lengths[2] * lengths[2],
        ];
        let anchors = [self.anchors[0], self.anchors[1], self.anchors[2]];
        match trilateration(anchors, radius2) {
            Some([x, y, z]) => [Some(x), Some(y), Some(z)],
            // A geometry whose spheres do not intersect: the axis is
            // undeterminable (`[None; 3]`, the port's shape).
            None => [None; 3],
        }
    }

    fn check_move(&self, _ctx: &mut MoveContext<'_>) -> Result<(), CommandError> {
        // `check_move` of `kinematics/winch.py:38-40`: boundary checks and
        // speed limits are not implemented, so every move is accepted.
        Ok(())
    }

    fn set_position(&mut self, _newpos: Coord, _homing_axes: &[usize]) {
        // Upstream sets each stepper's position here (`winch.py:25-27`); the
        // toolhead already does that for every stepper it drives
        // (`ToolHead.set_position`), so the kinematics keeps no limits.
    }

    fn update_limits(&mut self, _axis: usize, _range: Option<(f64, f64)>) {
        // Upstream's winch kinematics defines no `update_limits`; the
        // cartesian dual-carriage swap is its only caller and winch has no
        // dual carriage.
    }

    fn clear_homing_state(&mut self, _axes: &[usize]) {
        // `clear_homing_state` of `kinematics/winch.py:28-30`: "XXX - homing
        // not implemented", a no-op.
    }

    fn get_status(&self) -> Value {
        // `get_status` of `kinematics/winch.py:41-47`: the axes read as homed
        // unconditionally and the "limits" are the anchor bounding box.
        json!({
            "homed_axes": "xyz",
            "axis_minimum": self.axes_min.as_array(),
            "axis_maximum": self.axes_max.as_array(),
        })
    }

    fn home(&mut self, _homing: &mut dyn HomingState) {
        // `home` of `kinematics/winch.py:31-35` ("XXX - homing not
        // implemented") marks all three axes and forces the position to
        // `0, 0, 0`; both are done by the driver's winch branch, which fires no
        // endstop move and no `homing:home_rails_end`. The trait's `HomingState`
        // has no `set_axes`/`set_homed_position`, so nothing is expressible
        // here.
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mathutil::Xyz;

    #[test]
    fn test_winch_solver_measures_the_cable_length() {
        // The toolhead at (30, 40, 5) is sqrt(30² + 40² + 5²) from the origin
        // anchor (`winch_stepper_calc_position`, `kin_winch.c:17-26`). A
        // standing segment evaluates to its start anywhere.
        let segment = MoveSegment {
            print_time: 0.0,
            move_t: 1.0,
            start_v: 0.0,
            half_accel: 0.0,
            start_pos: Xyz::new(30.0, 40.0, 5.0),
            axes_r: Xyz::default(),
        };
        let from_origin = winch_position_fn([0.0, 0.0, 0.0]);
        assert!((from_origin.call(&segment, 0.5) - 2525.0_f64.sqrt()).abs() < 1e-12);
        // Off-origin anchor: the distance to (0, 40, 5) is 30 mm.
        let from_anchor = winch_position_fn([0.0, 40.0, 5.0]);
        assert!((from_anchor.call(&segment, 0.5) - 30.0).abs() < 1e-12);
        // Every axis moves the cable (`AF_X | AF_Y | AF_Z`, `kin_winch.c:39`).
        assert_eq!(
            winch_active_flags(),
            AxisFlags::X.union(AxisFlags::Y).union(AxisFlags::Z)
        );
    }

    /// Example-winch anchors: a (0, -2000, -100), b (2000, 1000, -100),
    /// c (-2000, 1000, -100), d (0, 0, 3000).
    fn example_winch() -> WinchKinematics {
        WinchKinematics::new(vec![
            ("stepper_a".to_string(), [0.0, -2000.0, -100.0]),
            ("stepper_b".to_string(), [2000.0, 1000.0, -100.0]),
            ("stepper_c".to_string(), [-2000.0, 1000.0, -100.0]),
            ("stepper_d".to_string(), [0.0, 0.0, 3000.0]),
        ])
    }

    #[test]
    fn test_calc_position_trilaterates_the_first_three_cables() {
        // A hand-computed trilateration: spheres centred at (0,0,0), (10,0,0)
        // and (0,10,0) with squared radii 29, 89 and 69 intersect at
        // (2, 3, -4), the branch below the centres. The cable lengths are the
        // square roots of those radii.
        let kin = WinchKinematics::new(vec![
            ("stepper_a".to_string(), [0.0, 0.0, 0.0]),
            ("stepper_b".to_string(), [10.0, 0.0, 0.0]),
            ("stepper_c".to_string(), [0.0, 10.0, 0.0]),
        ]);
        let positions = HashMap::from([
            ("stepper_a".to_string(), 29.0_f64.sqrt()),
            ("stepper_b".to_string(), 89.0_f64.sqrt()),
            ("stepper_c".to_string(), 69.0_f64.sqrt()),
        ]);

        let [x, y, z] = kin.calc_position(&positions);
        assert!((x.unwrap() - 2.0).abs() < 1e-9, "{x:?}");
        assert!((y.unwrap() - 3.0).abs() < 1e-9, "{y:?}");
        assert!((z.unwrap() + 4.0).abs() < 1e-9, "{z:?}");
    }

    #[test]
    fn test_calc_position_needs_all_three_cables() {
        let kin = example_winch();
        // Only two cables: the carriage cannot be located, so each axis is
        // `None`.
        let positions = HashMap::from([
            ("stepper_a".to_string(), 100.0),
            ("stepper_b".to_string(), 200.0),
        ]);
        assert_eq!(kin.calc_position(&positions), [None, None, None]);
    }

    #[test]
    fn test_the_status_reports_the_anchor_bounding_box() {
        let kin = example_winch();
        let status = kin.get_status();
        // The axes always read as homed (`winch.py:43`).
        assert_eq!(status["homed_axes"], "xyz");
        // min/max over every anchor coordinate (`winch.py:19-20`).
        assert_eq!(
            status["axis_minimum"],
            json!([-2000.0, -2000.0, -100.0, 0.0])
        );
        assert_eq!(status["axis_maximum"], json!([2000.0, 1000.0, 3000.0, 0.0]));
    }

    #[test]
    fn test_check_move_accepts_every_move() {
        use crate::core::klippy::motion::plan::{Move, MoveLimits};

        let kin = example_winch();
        let limits = MoveLimits {
            max_velocity: 300.0,
            max_accel: 3000.0,
            junction_deviation: 0.01,
            mcr_pseudo_accel: 1500.0,
        };
        // A move far outside the anchor box is still accepted
        // (`winch.py:38-40`).
        let mut move_ = Move::new(
            Coord::default(),
            Coord::new(1e6, -1e6, 1e6, 0.0),
            100.0,
            &limits,
        );
        let mut ctx = MoveContext::new(&mut move_);
        assert!(kin.check_move(&mut ctx).is_ok());
    }
}
