//! `force_move` — the timing math a move that bypasses the planner needs.
//!
//! Upstream's `klippy/extras/force_move.py` is a utility module for manually
//! driven moves: `FORCE_MOVE` / `SET_KINEMATIC_POSITION` on a normal axis, and
//! [`calc_move_time`] itself, which the manual stepper uses to turn a
//! distance/speed/acceleration into a trapezoid (`manual_stepper.py:8,66`).
//!
//! # What is here
//!
//! Only [`calc_move_time`] is ported so far. Upstream's `ForceMove` printer
//! object and its `FORCE_MOVE` / `SET_KINEMATIC_POSITION` commands are not
//! registered here; the `SET_KINEMATIC_POSITION` this port has belongs to
//! [`toolhead`](crate::core::klippy::extras::toolhead), which is where upstream
//! resolves it through `force_move` for the low-level position.

/// Calculate a move's `(axis_r, accel_t, cruise_t, cruise_v)` trapezoid
/// (`force_move.calc_move_time`, `force_move.py:15-28`).
///
/// `dist` is signed: a negative distance flips `axis_r` and the magnitude is
/// used for the profile. With no acceleration, or no distance, the move is a
/// constant-speed cruise at the requested `speed` (`accel_t` is zero). The
/// capped case slows `cruise_v` to the speed the distance can reach under
/// `accel`, so the trapezoid degenerates to a triangle with no cruise phase.
pub fn calc_move_time(dist: f64, speed: f64, accel: f64) -> (f64, f64, f64, f64) {
    let mut axis_r = 1.0;
    let mut dist = dist;
    if dist < 0.0 {
        axis_r = -1.0;
        dist = -dist;
    }
    if accel == 0.0 || dist == 0.0 {
        return (axis_r, 0.0, dist / speed, speed);
    }
    let mut speed = speed;
    let max_cruise_v2 = dist * accel;
    if max_cruise_v2 < speed.powf(2.0) {
        speed = max_cruise_v2.sqrt();
    }
    let accel_t = speed / accel;
    let accel_decel_d = accel_t * speed;
    let cruise_t = (dist - accel_decel_d) / speed;
    (axis_r, accel_t, cruise_t, speed)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// The two profiles the upstream function is documented on: a negative
    /// distance flips `axis_r` and a distance too short to reach `speed` slows
    /// the cruise (the triangle case).
    #[test]
    fn test_calc_move_time_matches_the_upstream_formula() {
        // `calc_move_time(-2., 10., 100.)` in CPython returns
        // `(-1.0, 0.1, 0.1, 10.0)`.
        assert_eq!(calc_move_time(-2.0, 10.0, 100.0), (-1.0, 0.1, 0.1, 10.0));
    }

    /// A distance that cannot reach `speed` slows the cruise to the speed
    /// `sqrt(dist * accel)` and leaves no cruise phase.
    #[test]
    fn test_a_short_move_slows_the_cruise_to_the_reachable_speed() {
        let (axis_r, accel_t, cruise_t, cruise_v) = calc_move_time(2.0, 100.0, 100.0);
        assert_eq!(axis_r, 1.0);
        let expected = 200.0f64.sqrt();
        assert!(
            (cruise_v - expected).abs() < 1e-12,
            "{cruise_v} vs {expected}"
        );
        assert!((accel_t - expected / 100.0).abs() < 1e-12, "{accel_t}");
        assert!(cruise_t.abs() < 1e-12, "{cruise_t}");
    }

    /// No acceleration is a constant-speed cruise; no distance is a no-op.
    #[test]
    fn test_no_accel_or_no_distance_is_a_plain_cruise() {
        assert_eq!(calc_move_time(5.0, 10.0, 0.0), (1.0, 0.0, 0.5, 10.0));
        assert_eq!(calc_move_time(0.0, 10.0, 100.0), (1.0, 0.0, 0.0, 10.0));
    }

    /// The number the manual-stepper corpus move uses (`MOVE=300 SPEED=10
    /// ACCEL=2000`): the distance is long enough to reach `speed`, so the
    /// cruise is 10 mm/s for the bulk of the move.
    #[test]
    fn test_the_long_manual_move_is_a_full_trapezoid() {
        let (axis_r, accel_t, cruise_t, cruise_v) = calc_move_time(300.0, 10.0, 2000.0);
        assert_eq!(axis_r, 1.0);
        assert_eq!(cruise_v, 10.0);
        assert_eq!(accel_t, 0.005);
        // (300 - accel_decel_d) / speed, as CPython computes it.
        assert!(
            (cruise_t - 29.994_999_999_999_997).abs() < 1e-12,
            "{cruise_t}"
        );
    }
}
