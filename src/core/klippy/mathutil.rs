//! Numeric helpers shared by the motion stack.
//!
//! Upstream keeps `Coord` in `klippy/gcode.py:12-17` — because a G-code
//! parameter is one — and the geometry in `klippy/mathutil.py`. They live
//! together here: the toolhead, the kinematics and G-code all need a
//! coordinate, and none of them should reach through the G-code dispatcher for
//! it. The geometry routines (`trilateration`, `gaussian_solve`, …) arrive with
//! the kinematics that use them.

use std::ops::Index;

/// The four axes, in the order upstream uses.
pub const X_AXIS: usize = 0;
/// See [`X_AXIS`].
pub const Y_AXIS: usize = 1;
/// See [`X_AXIS`].
pub const Z_AXIS: usize = 2;
/// See [`X_AXIS`].
pub const E_AXIS: usize = 3;

/// How many axes a [`Coord`] carries.
pub const AXES: usize = 4;

/// A toolhead position: `x`, `y`, `z`, `e`.
///
/// Upstream's `Coord` is a `tuple` subclass whose constructor pads a shorter
/// sequence with zeros (`klippy/gcode.py:12-17`). The axes are a fixed
/// `[f64; 4]` here so the type stays `Copy` and indexable in the motion hot
/// paths (`Move`, `trapq`); the named accessors keep the call sites readable.
///
/// It deliberately holds no `Option`: a position is always four numbers. The
/// one place a coordinate can be *unknown* is the reverse mapping in
/// `Kinematics::calc_position`, which returns `[Option<f64>; 3]` instead (a
/// delta that cannot be solved, a rail that is not homed).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Coord([f64; AXES]);

impl Coord {
    /// A coordinate from its four axes.
    pub const fn new(x: f64, y: f64, z: f64, e: f64) -> Self {
        Self([x, y, z, e])
    }

    /// A coordinate from a shorter or equal sequence, padding with zeros.
    ///
    /// This is upstream's constructor semantics (`klippy/gcode.py:14-16`):
    /// `Coord::from_axes([1., 2.])` is `(1, 2, 0, 0)`. Extra values are
    /// ignored, matching the tuple slice upstream would build.
    pub fn from_axes(axes: impl IntoIterator<Item = f64>) -> Self {
        let mut coord = [0.0; AXES];
        for (slot, value) in coord.iter_mut().zip(axes) {
            *slot = value;
        }
        Self(coord)
    }

    /// The X axis.
    pub const fn x(&self) -> f64 {
        self.0[X_AXIS]
    }

    /// The Y axis.
    pub const fn y(&self) -> f64 {
        self.0[Y_AXIS]
    }

    /// The Z axis.
    pub const fn z(&self) -> f64 {
        self.0[Z_AXIS]
    }

    /// The extruder axis.
    pub const fn e(&self) -> f64 {
        self.0[E_AXIS]
    }

    /// An axis by index ([`X_AXIS`] … [`E_AXIS`]).
    ///
    /// # Panics
    /// When `axis` is not one of the four constants. A coordinate has a fixed
    /// number of axes, so an out-of-range index is a bug in the caller.
    pub const fn axis(&self, axis: usize) -> f64 {
        self.0[axis]
    }

    /// Set an axis by index.
    ///
    /// # Panics
    /// As [`Coord::axis`].
    pub fn set_axis(&mut self, axis: usize, value: f64) {
        self.0[axis] = value;
    }

    /// The four axes as an array.
    pub const fn as_array(&self) -> &[f64; AXES] {
        &self.0
    }

    /// The axes as an iterator.
    pub fn axes(&self) -> impl Iterator<Item = f64> + '_ {
        self.0.iter().copied()
    }
}

impl Index<usize> for Coord {
    type Output = f64;

    fn index(&self, axis: usize) -> &f64 {
        &self.0[axis]
    }
}

impl From<[f64; AXES]> for Coord {
    fn from(axes: [f64; AXES]) -> Self {
        Self(axes)
    }
}

impl From<Coord> for [f64; AXES] {
    fn from(coord: Coord) -> Self {
        coord.0
    }
}

/// A three-axis position `(x, y, z)`.
///
/// Upstream's `struct coord` (`chelper/trapq.h:5-12`) has exactly these three:
/// the extruder runs its own trapq, so the fourth axis never appears in the
/// motion queue or in a stepper's position solver. [`Coord`] is the four-axis
/// type the toolhead and G-code speak in.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Xyz(pub [f64; 3]);

impl Xyz {
    /// A position from its three axes.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self([x, y, z])
    }

    /// The X axis.
    pub const fn x(&self) -> f64 {
        self.0[0]
    }

    /// The Y axis.
    pub const fn y(&self) -> f64 {
        self.0[1]
    }

    /// The Z axis.
    pub const fn z(&self) -> f64 {
        self.0[2]
    }

    /// The three axes as an array.
    pub const fn as_array(&self) -> &[f64; 3] {
        &self.0
    }
}

impl From<[f64; 3]> for Xyz {
    fn from(axes: [f64; 3]) -> Self {
        Self(axes)
    }
}

impl From<Xyz> for [f64; 3] {
    fn from(xyz: Xyz) -> Self {
        xyz.0
    }
}

impl From<Coord> for Xyz {
    fn from(coord: Coord) -> Self {
        Self([coord.x(), coord.y(), coord.z()])
    }
}

impl From<Xyz> for Coord {
    fn from(xyz: Xyz) -> Self {
        Coord::new(xyz.x(), xyz.y(), xyz.z(), 0.0)
    }
}

// ===========================================================================
// Coordinate descent
// ===========================================================================

/// Minimize an error function over `params` by coordinate descent — upstream's
/// `coordinate_descent(adj_params, params, error_func)`
/// (`klippy/mathutil.py:16-49`).
///
/// Upstream takes the adjustable parameters by name (`adj_params`) out of a
/// `params` dict; here the adjustable set *is* `params`, adjusted in index
/// order. That is the shape the H9 leveling family wants: `z_tilt` and
/// `bed_tilt` fit the plane `z = c + a*x + b*y` by adjusting all three
/// coefficients, so every entry is adjustable and the caller's slice order is
/// upstream's `adj_params` order. Indices avoid `f64` map keys entirely.
///
/// The search mirrors upstream step for step: every parameter starts with a
/// step `dp` of 1.0; each round tries `+dp` (on improvement keep the value and
/// grow `dp` by `* 1.1`), else `-dp` (same rules), else reverts and shrinks
/// `dp` by `* 0.9`. It stops once the steps sum to at most `1e-5` or after
/// 10 000 rounds — the same threshold and cap as upstream's
/// `while sum(dp.values()) > threshold and rounds < 10000`.
///
/// The best parameters are left in `params` (the output is in-place, since
/// `f64` slices cannot be hash-keyed the way upstream returns a dict).
pub fn coordinate_descent(params: &mut [f64], mut error: impl FnMut(&[f64]) -> f64) {
    let mut dp = vec![1.0_f64; params.len()];
    let mut best_err = error(params);
    let mut rounds = 0usize;

    while dp.iter().sum::<f64>() > 1e-5 && rounds < 10_000 {
        rounds += 1;
        for i in 0..params.len() {
            let orig = params[i];
            params[i] = orig + dp[i];
            let err = error(params);
            if err < best_err {
                best_err = err;
                dp[i] *= 1.1;
                continue;
            }
            params[i] = orig - dp[i];
            let err = error(params);
            if err < best_err {
                best_err = err;
                dp[i] *= 1.1;
                continue;
            }
            params[i] = orig;
            dp[i] *= 0.9;
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_named_accessors_and_indexing_agree() {
        let coord = Coord::new(1.0, 2.0, 3.0, 4.0);

        assert_eq!(coord.x(), 1.0);
        assert_eq!(coord.y(), 2.0);
        assert_eq!(coord.z(), 3.0);
        assert_eq!(coord.e(), 4.0);
        // Indexing is the same thing, for the loops that walk all four axes.
        for axis in 0..AXES {
            assert_eq!(coord[axis], coord.axis(axis));
        }
    }

    #[test]
    fn test_a_short_sequence_is_padded_with_zeros() {
        // Upstream's constructor pads to four (`klippy/gcode.py:14-16`).
        assert_eq!(Coord::from_axes([1.0, 2.0]), Coord::new(1.0, 2.0, 0.0, 0.0));
        assert_eq!(Coord::from_axes([]), Coord::new(0.0, 0.0, 0.0, 0.0));
        assert_eq!(
            Coord::from_axes([1.0, 2.0, 3.0, 4.0, 5.0]),
            Coord::new(1.0, 2.0, 3.0, 4.0)
        );
    }

    #[test]
    fn test_axes_round_trip_through_the_array() {
        let coord = Coord::new(-1.0, 0.5, 0.0, 12.25);

        let axes: [f64; AXES] = coord.into();
        assert_eq!(Coord::from(axes), coord);
        assert_eq!(coord.as_array(), &axes);
        assert_eq!(coord.axes().collect::<Vec<_>>(), axes.to_vec());
    }

    #[test]
    fn test_set_axis_writes_one_axis() {
        let mut coord = Coord::new(1.0, 2.0, 3.0, 4.0);

        coord.set_axis(Z_AXIS, 9.0);

        assert_eq!(coord, Coord::new(1.0, 2.0, 9.0, 4.0));
    }

    #[test]
    fn test_coordinate_descent_converges_on_a_quadratic() {
        // f(a, b) = (a - 1)^2 + (b + 2)^2 has its analytic minimum at
        // (1, -2); coordinate descent must walk there from the origin.
        let mut params = [0.0, 0.0];

        coordinate_descent(&mut params, |p| (p[0] - 1.0).powi(2) + (p[1] + 2.0).powi(2));

        assert!((params[0] - 1.0).abs() < 1e-4, "a = {}", params[0]);
        assert!((params[1] + 2.0).abs() < 1e-4, "b = {}", params[1]);
    }

    #[test]
    fn test_xyz_converts_to_and_from_coord() {
        let xyz = Xyz::new(1.0, 2.0, 3.0);

        assert_eq!(xyz.x(), 1.0);
        assert_eq!(xyz.y(), 2.0);
        assert_eq!(xyz.z(), 3.0);
        assert_eq!(xyz.as_array(), &[1.0, 2.0, 3.0]);

        // A four-axis position built from one has a zero extruder axis.
        let coord: Coord = xyz.into();
        assert_eq!(coord, Coord::new(1.0, 2.0, 3.0, 0.0));
        // And the fourth axis is dropped going back.
        let back: Xyz = Coord::new(1.0, 2.0, 3.0, 9.0).into();
        assert_eq!(back, xyz);
    }
}
