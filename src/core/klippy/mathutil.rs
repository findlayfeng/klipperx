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
// NxM matrix helpers (upstream `klippy/mathutil.py` — `solve_linear_equations`
// and friends, which the eddy probe's `PROBE_EDDY_CURRENT_TAP_CALIBRATE` info
// path runs on the main calibration points)
// ===========================================================================

/// Transpose a matrix (`mathutil.mat_transp`).
///
/// # Panics
/// Panics on an empty matrix (upstream indexes `a[0]` the same way).
pub fn mat_transp(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
    (0..a[0].len())
        .map(|i| a.iter().map(|row| row[i]).collect())
        .collect()
}

/// Matrix product (`mathutil.mat_mat_mul`); `None` when the shapes do not
/// line up (upstream returns `None` for `len(a[0]) != len(b)`).
pub fn mat_mat_mul(a: &[Vec<f64>], b: &[Vec<f64>]) -> Option<Vec<Vec<f64>>> {
    if a[0].len() != b.len() {
        return None;
    }
    let bt = mat_transp(b);
    Some(
        a.iter()
            .map(|a_i| {
                bt.iter()
                    .map(|bt_j| a_i.iter().zip(bt_j).map(|(x, y)| x * y).sum())
                    .collect()
            })
            .collect(),
    )
}

/// `mat_mat_mul(a, mat_transp(a))` computed the cheap symmetric way
/// (`mathutil.mat_mul_transp`).
///
/// # Panics
/// Panics on an empty matrix, as upstream's indexing does.
pub fn mat_mul_transp(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
    // Resulting matrix is symmetric - compute lower-left.
    let mut res: Vec<Vec<f64>> = a
        .iter()
        .enumerate()
        .map(|(i, a_i)| {
            a[..=i]
                .iter()
                .map(|a_j| a_i.iter().zip(a_j).map(|(x, y)| x * y).sum())
                .collect()
        })
        .collect();
    // Fill in upper right of matrix.
    for i in 0..res.len() {
        let tail: Vec<f64> = res[i + 1..].iter().map(|res_j| res_j[i]).collect();
        res[i].extend(tail);
    }
    res
}

/// Solve `a · x = rhs` by Gaussian elimination with partial pivoting
/// (`mathutil.gaussian_solve`). `rhs` rows may carry several columns.
///
/// `None` when a pivot is (near) zero, unless `allow_underdetermined` takes
/// the degenerate pivot as a zero reciprocal — upstream's answer for a
/// rank-deficient system.
pub fn gaussian_solve(
    a: &[Vec<f64>],
    rhs: &[Vec<f64>],
    allow_underdetermined: bool,
) -> Option<Vec<Vec<f64>>> {
    let mut res = rhs.to_vec();
    let mut m = a.to_vec();
    let rows_m = m.len();
    // Perform the LU-decomposition through Gaussian elimination, bottom row up.
    for i in (0..rows_m).rev() {
        // Find a pivot and swap the corresponding rows (first max wins, as
        // upstream's `list.index(max(...))` does).
        let mut j = 0;
        let mut best = m[0][i].abs();
        for row in 1..=i {
            let mag = m[row][i].abs();
            if mag > best {
                best = mag;
                j = row;
            }
        }
        if i != j {
            m.swap(i, j);
            res.swap(i, j);
        }

        // Scale the i-th row (and drop its pivot column).
        let pivot = m[i][i];
        let recipr = if pivot.abs() < 1e-10 {
            if !allow_underdetermined {
                return None;
            }
            0.0
        } else {
            1.0 / pivot
        };
        let m_i: Vec<f64> = m[i].iter().take(i).map(|v| v * recipr).collect();
        let res_i: Vec<f64> = res[i].iter().map(|v| v * recipr).collect();
        m[i] = m_i.clone();
        res[i] = res_i.clone();

        // Zero out the pivot column in the rows above it, keeping the
        // multiplier in place (the compact-L form the back pass reads).
        for j in 0..i {
            let c = m[j][i];
            m[j] = m[j]
                .iter()
                .zip(m_i.iter())
                .map(|(m_j_k, m_i_k)| m_j_k - c * m_i_k)
                .collect();
            res[j] = res[j]
                .iter()
                .zip(res_i.iter())
                .map(|(res_j_k, res_i_k)| res_j_k - c * res_i_k)
                .collect();
        }
    }

    // Forward substitution against the unit-lower-triangular factor.
    let mut rest = mat_transp(&res);
    if rest.is_empty() {
        return Some(res);
    }
    for rest_k in &mut rest {
        for i in 1..rows_m {
            let sub: f64 = m[i].iter().zip(rest_k.iter()).map(|(x, y)| x * y).sum();
            rest_k[i] -= sub;
        }
    }
    Some(mat_transp(&rest))
}

/// Least-squares solve of an over-determined system
/// (`mathutil.solve_linear_equations`): the normal equations
/// `(AᵀA) x = Aᵀ·ans` through [`gaussian_solve`].
///
/// # Panics
/// Panics on empty input, as upstream's transposes do.
pub fn solve_linear_equations(eqs: &[Vec<f64>], ans: &[Vec<f64>]) -> Option<Vec<Vec<f64>>> {
    let eqst = mat_transp(eqs);
    let eqst_eqs = mat_mul_transp(&eqst);
    let eqst_ans = mat_mat_mul(&eqst, ans)?;
    gaussian_solve(&eqst_eqs, &eqst_ans, false)
}

/// The pseudo inverse upstream's shaper math solves its impulse equations with
/// (`mathutil.pseudo_inverse`): the normal-equation form `(AᵀA)⁻¹Aᵀ`.
///
/// Upstream calls `gaussian_solve(mtm, mt)` with `allow_underdetermined` at its
/// `False` default (`mathutil.py:210-213`), so a rank-deficient `AᵀA` is `None`
/// — which `shaper_defs.get_mzv_coeffs` turns into its "Ill-formed shaper"
/// error.
///
/// # Panics
/// Panics on an empty matrix, as upstream's transposes do.
pub fn pseudo_inverse(m: &[Vec<f64>]) -> Option<Vec<Vec<f64>>> {
    let mt = mat_transp(m);
    let mtm = mat_mul_transp(&mt);
    gaussian_solve(&mtm, &mt, false)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;

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
    fn test_coordinate_descent_recovers_a_plane_from_coupled_residuals() {
        // Plane fit as z_tilt / bed_tilt use it: minimize
        // sum((z_i - x_i*a - y_i*b - c)^2) over a grid whose points were
        // built from a known plane. The grid is off-center, so the slope
        // and intercept columns are genuinely coupled.
        let (true_a, true_b, true_c) = (0.5, -0.3, 2.0);
        let mut samples = Vec::new();
        for &x in &[10.0, 20.0, 30.0] {
            for &y in &[5.0, 15.0, 25.0] {
                samples.push((x, y, true_a * x + true_b * y + true_c));
            }
        }
        let mut params = [0.0, 0.0, 0.0];

        coordinate_descent(&mut params, |p| {
            let (a, b, c) = (p[0], p[1], p[2]);
            samples
                .iter()
                .map(|&(x, y, z)| (z - x * a - y * b - c).powi(2))
                .sum::<f64>()
        });

        assert!((params[0] - true_a).abs() < 1e-3, "a = {}", params[0]);
        assert!((params[1] - true_b).abs() < 1e-3, "b = {}", params[1]);
        assert!((params[2] - true_c).abs() < 1e-3, "c = {}", params[2]);
    }

    #[test]
    fn test_coordinate_descent_stops_when_the_error_never_improves() {
        // A flat error: every +dp and -dp trial fails, so a round only ever
        // shrinks the steps (dp *= 0.9). The sum(dp) <= 1e-5 condition must
        // end the search instead of looping forever, leaving params reverted.
        let calls = Cell::new(0usize);
        let mut params = [0.5, -1.5];

        coordinate_descent(&mut params, |_p| {
            calls.set(calls.get() + 1);
            42.0
        });

        assert_eq!(params, [0.5, -1.5]);
        // One evaluation up front, then two per parameter per round. Exiting
        // via the shrinking steps takes far fewer calls than the ceiling of
        // 1 + 10000 rounds * 2 trials * 2 parameters.
        assert!(
            calls.get() >= 1 && calls.get() < 10_000,
            "calls = {}",
            calls.get()
        );
        assert!(calls.get() <= 1 + 10_000 * 2 * 2, "calls = {}", calls.get());
    }

    #[test]
    fn test_coordinate_descent_stops_at_the_round_cap() {
        // An error that improves on *every* evaluation: the +dp trial always
        // wins, the steps only grow (dp *= 1.1), and sum(dp) never reaches
        // the threshold — only the 10000-round cap can end the search. One
        // evaluation per parameter per round makes the cap observable.
        let calls = Cell::new(0usize);
        let mut params = [0.5, -1.5];

        coordinate_descent(&mut params, |_| {
            let seen = calls.get();
            calls.set(seen + 1);
            -(seen as f64)
        });

        // 1 initial evaluation + 10000 rounds * 2 parameters, one +dp trial
        // each; without the cap this loop would never terminate.
        assert_eq!(calls.get(), 1 + 10_000 * 2);
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

#[cfg(test)]
mod linalg_tests {
    use super::*;

    /// A square system solved exactly, then put back through `a · x`.
    #[test]
    fn gaussian_solve_recovers_known_values() {
        // 2x + y = 5; x + 3y = 10  →  x = 1, y = 3.
        let a = vec![vec![2.0, 1.0], vec![1.0, 3.0]];
        let rhs = vec![vec![5.0], vec![10.0]];
        let x = gaussian_solve(&a, &rhs, false).expect("a unique solution");
        assert!((x[0][0] - 1.0).abs() < 1e-9, "{x:?}");
        assert!((x[1][0] - 3.0).abs() < 1e-9, "{x:?}");
        // Substitution reproduces the right-hand side.
        let back = mat_mat_mul(&a, &x).expect("shapes line up");
        for (row, want) in back.iter().zip(&rhs) {
            assert!((row[0] - want[0]).abs() < 1e-9);
        }
    }

    /// A singular system has no answer (upstream returns `None`).
    #[test]
    fn gaussian_solve_refuses_a_singular_system() {
        let a = vec![vec![1.0, 2.0], vec![2.0, 4.0]];
        let rhs = vec![vec![1.0], vec![2.0]];
        assert!(gaussian_solve(&a, &rhs, false).is_none());
    }

    /// The over-determined fit the eddy `PROBE_EDDY_CURRENT_TAP_CALIBRATE`
    /// info path runs (`_analyze_main_calibration`): a quadratic through four
    /// (z, freq) pairs, put back through the fitted curve.
    #[test]
    fn solve_linear_equations_fits_a_quadratic_and_substitutes_back() {
        // Exact on f(z) = 3_400_000 - 500_000 z + 100_000 z².
        let points = [(0.05_f64,), (0.15,), (0.40,), (0.70,)];
        let f = |z: f64| 3_400_000.0 - 500_000.0 * z + 100_000.0 * z * z;
        let eqs: Vec<Vec<f64>> = points.iter().map(|&(z,)| vec![1.0, z, z * z]).collect();
        let ans: Vec<Vec<f64>> = points.iter().map(|&(z,)| vec![f(z)]).collect();
        let coeffs = solve_linear_equations(&eqs, &ans).expect("full-rank fit");
        assert!((coeffs[0][0] - 3_400_000.0).abs() < 1e-3, "{coeffs:?}");
        assert!((coeffs[1][0] + 500_000.0).abs() < 1e-3, "{coeffs:?}");
        assert!((coeffs[2][0] - 100_000.0).abs() < 1e-3, "{coeffs:?}");
        // 回代: every point sits on the fitted curve.
        let back = mat_mat_mul(&eqs, &coeffs).expect("shapes line up");
        for (row, &(z,)) in back.iter().zip(points.iter()) {
            assert!((row[0] - f(z)).abs() < 1e-3);
        }
    }

    /// For a square, well-conditioned matrix the pseudo inverse is the ordinary
    /// inverse: `A · pinv(A)` is the identity.
    #[test]
    fn pseudo_inverse_inverts_a_square_matrix() {
        let a = vec![vec![2.0, 1.0], vec![1.0, 3.0]];
        let pinv = pseudo_inverse(&a).expect("a full-rank matrix");
        let product = mat_mat_mul(&a, &pinv).expect("shapes line up");
        for (i, row) in product.iter().enumerate() {
            for (j, value) in row.iter().enumerate() {
                let want = if i == j { 1.0 } else { 0.0 };
                assert!((value - want).abs() < 1e-9, "{product:?}");
            }
        }
    }

    /// `mat_mul_transp` agrees with the straightforward product.
    #[test]
    fn mat_mul_transp_matches_the_reference_product() {
        let a = vec![vec![1.0, 2.0, 3.0], vec![4.0, 5.0, 6.0]];
        // `mat_mul_transp(a)` is `a · aᵀ` (upstream dots each pair of rows), so
        // the reference product is `mat_mat_mul(a, mat_transp(a))`, not `aᵀ · a`.
        let fast = mat_mul_transp(&a);
        let at = mat_transp(&a);
        let slow = mat_mat_mul(&a, &at).expect("shapes line up");
        assert_eq!(fast.len(), slow.len());
        for (row_f, row_s) in fast.iter().zip(&slow) {
            for (f, s) in row_f.iter().zip(row_s) {
                assert!((f - s).abs() < 1e-12);
            }
        }
    }
}
