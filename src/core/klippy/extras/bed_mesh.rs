//! `bed_mesh` — bed mesh calibration.
//!
//! Upstream `klippy/extras/bed_mesh.py` (1765 lines, mesh interpolation,
//! faulty-region substitution, adaptive meshes, rapid scan, fade and the move
//! splitter). This unit lands the part the upstream corpus exercises:
//!
//! - the `[bed_mesh]` section with **every** option the corpus writes (so the
//!   config loads), including the `faulty_region_<N>_min` / `_max` pairs;
//! - the algorithm check (`_verify_algorithm`, `bed_mesh.py:416-456`): only
//!   `lagrange` and `bicubic` are writable, `mesh_pps: 0` on both axes turns
//!   the mesh into `direct`, and the probe counts the two interpolators can
//!   carry are enforced;
//! - `BED_MESH_CALIBRATE`: generate the probe points, move to each (net of the
//!   probe offsets), probe it through the `probe` object's session, store the
//!   probed grid and sample the interpolation grid from it ([`ZMesh`],
//!   `mesh_pps`);
//! - `BED_MESH_CLEAR`, the `bed_mesh/dump_mesh` endpoint (which reads the
//!   stored grid through [`BedMesh::loaded_mesh`]) and the `get_status` shape
//!   clients read;
//! - [`ZMesh::calc_z`]: the bilinear lookup over the mesh, which is the mesh's
//!   own answer to "how far off is the bed here".
//!
//! **Not implemented yet** (tracked in `TODO.md` H9, next units):
//!
//! - the mesh offsets (`BED_MESH_OFFSET`) and the zero reference
//!   (`ZERO_REFERENCE`) — both are profile-command state, so `calc_z` looks up
//!   the coordinate as given;
//! - faulty-region substitution (`_process_faulty_regions`): the regions are
//!   parsed and probed like any other point;
//! - fade (`fade_start` / `fade_end` / `fade_target`) and the z-adjustment the
//!   mesh applies to moves (`MoveSplitter`) — so a calibration currently does
//!   **not** affect subsequent moves;
//! - the profile commands (`BED_MESH_PROFILE`, `BED_MESH_OUTPUT`, `BED_MESH_MAP`,
//!   `BED_MESH_OFFSET`): a calibration answers as the default profile and
//!   nothing is saved across a restart, so `profiles` stays empty.
//!
//! The load side is complete: every option below is claimed, which is what the
//! corpus needs, and the gaps above are capability gaps, not silent shortcuts —
//! they are listed in the manual and the task list.

use std::sync::{Arc, Mutex};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::probe::{lookup_probe_session, PROBE_PARAMS};
use crate::core::klippy::extras::toolhead::ToolHeadObject;
use crate::core::klippy::gcode::{CommandError, GCodeDispatch, GCODE_OBJECT};
use crate::core::klippy::load::section;
use crate::core::klippy::mathutil::Coord;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("bed_mesh", order = 30, load = load_config);

/// The toolhead object, as the loader registers `[printer]`.
const TOOLHEAD_OBJECT: &str = "toolhead";

/// The probe object the calibration drives.
/// What `get_status` reports (`ProbeCommandHelper.get_status`) — the eddy
/// probe reports through probe.rs's own helper with this section id.

/// The Z axis index, as [`Coord`] numbers them.
const Z_AXIS: usize = 2;

/// The object the loader registers `[bed_mesh]` under — the name
/// `bed_mesh/dump_mesh` looks the stored grid up by.
pub const BED_MESH_OBJECT: &str = "bed_mesh";

/// The profile a calibration is stored under: upstream's default for
/// `BED_MESH_CALIBRATE PROFILE=` (`bed_mesh.py:645`), which this unit does not
/// read yet.
const DEFAULT_PROFILE: &str = "default";

/// One `faulty_region_<N>` rectangle: `(min_x, min_y, max_x, max_y)`.
type FaultyRegion = (f64, f64, f64, f64);

/// The `[bed_mesh]` options this unit reads (`bed_mesh.py:88-133`).
#[derive(Debug, Clone, PartialEq)]
pub struct BedMeshOptions {
    /// The mesh area's lower corner — `(-radius, -radius)` on a round bed,
    /// which never reads `mesh_min` (`bed_mesh.py:393-396`).
    pub mesh_min: [f64; 2],
    /// The mesh area's upper corner — `(radius, radius)` on a round bed.
    pub mesh_max: [f64; 2],
    /// Probing points along X and Y (`probe_count`).
    pub probe_count: [i64; 2],
    /// The speed probing moves run at.
    pub speed: f64,
    /// The Z the toolhead travels between points at.
    pub horizontal_move_z: f64,
    /// The verified interpolation algorithm: `lagrange`, `bicubic`, or the
    /// `direct` [`BedMeshOptions::verify_algorithm`] substitutes when
    /// interpolation is off.
    pub algorithm: String,
    /// Where the mesh's fade begins.
    pub fade_start: f64,
    /// Where it ends; `0` disables fading upstream.
    pub fade_end: f64,
    /// The Z the faded region targets.
    pub fade_target: Option<f64>,
    /// Interpolated points between probed points, per axis.
    pub mesh_pps: [i64; 2],
    /// The bicubic tension.
    pub bicubic_tension: f64,
    /// Points along a round bed's diameter (`probe_count` for round meshes).
    pub round_probe_count: i64,
    /// A round bed's radius; `None` for rectangular beds.
    pub mesh_radius: Option<f64>,
    /// A round bed's centre.
    pub mesh_origin: [f64; 2],
    /// How far moves may go before a segmented move is checked.
    pub move_check_distance: f64,
    /// The `faulty_region_<N>` rectangles.
    pub faulty_regions: Vec<FaultyRegion>,
}

/// Two floats from a `x, y` option — upstream's `getfloatlist(…, count=2)`
/// (`configfile.py:87-106`): every value is parsed *before* the count is
/// checked, a missing option is `must be specified` (`configfile.py:37-38`),
/// a wrong count `must have 2 elements` (`configfile.py:100-101`), and an
/// unparseable value keeps the parser's wording (`configfile.py:44-45`).
fn two_floats(
    config: &ConfigWrapper,
    option: &str,
    default: Option<[f64; 2]>,
) -> Result<[f64; 2], ConfigError> {
    let Some(items) = config.get_list(option, ',') else {
        return default.ok_or_else(|| {
            ConfigError::new(format!(
                "Option '{option}' in section '{}' must be specified",
                config.identifier()
            ))
        });
    };
    let mut out = Vec::with_capacity(items.len());
    for item in &items {
        out.push(item.trim().parse::<f64>().map_err(|_| {
            ConfigError::new(format!(
                "Unable to parse option '{option}' in section '{}'",
                config.identifier()
            ))
        })?);
    }
    if out.len() != 2 {
        return Err(ConfigError::new(format!(
            "Option '{option}' in section '{}' must have 2 elements",
            config.identifier()
        )));
    }
    Ok([out[0], out[1]])
}

/// Upstream's `parse_config_pair` (`bed_mesh.py:38-56`): one value makes a
/// square grid `(n, n)`, two give the counts per axis, and any other length is
/// refused with upstream's `malformed` wording; `minval` then bounds both
/// counts — `probe_count` passes `minval=3` (`bed_mesh.py:399`), `mesh_pps`
/// `minval=0` (`bed_mesh.py:409`).
///
/// # Errors
/// An unparseable value keeps upstream's `Unable to parse` wording
/// (`configfile.py:44-45`), a wrong length its `malformed` wording
/// (`bed_mesh.py:42-43`), and a count below `minval` the `minimum of` wording
/// (`bed_mesh.py:47-49`, whose section name carries no quotes, as upstream
/// writes it).
fn parse_config_pair(
    config: &ConfigWrapper,
    option: &str,
    default: i64,
    minval: i64,
) -> Result<[i64; 2], ConfigError> {
    let Some(items) = config.get_list(option, ',') else {
        return Ok([default, default]);
    };
    // `getintlist` parses every value before the length is examined
    // (`configfile.py:98-101`).
    let mut parsed = Vec::with_capacity(items.len());
    for item in &items {
        parsed.push(item.trim().parse::<i64>().map_err(|_| {
            ConfigError::new(format!(
                "Unable to parse option '{option}' in section '{}'",
                config.identifier()
            ))
        })?);
    }
    if parsed.len() != 2 {
        if parsed.len() != 1 {
            return Err(ConfigError::new(format!(
                "bed_mesh: malformed '{option}' value: {}",
                config.get_str(option).unwrap_or_default()
            )));
        }
        parsed.push(parsed[0]);
    }
    if parsed[0] < minval || parsed[1] < minval {
        return Err(ConfigError::new(format!(
            "Option '{option}' in section bed_mesh must have a minimum of {minval}"
        )));
    }
    Ok([parsed[0], parsed[1]])
}

impl BedMeshOptions {
    /// Read every option the corpus writes.
    ///
    /// # Errors
    /// As the option readers: a missing `mesh_min`/`mesh_max` on a
    /// rectangular bed, an inverted min/max pair, a malformed or too-small
    /// `probe_count`, and whatever
    /// [`verify_algorithm`](BedMeshOptions::verify_algorithm) refuses.
    pub fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let mesh_radius = config.get_optional_float("mesh_radius")?;
        let round_probe_count =
            config.get_int_bounded("round_probe_count", Some(5), Some(3), None)?;
        // A round bed counts from `round_probe_count` and needs an odd number
        // of points along each axis, so that the row it samples across the
        // diameter has a middle point (`bed_mesh.py:386-390`).
        if mesh_radius.is_some() && round_probe_count % 2 == 0 {
            return Err(ConfigError::new(
                "bed_mesh: probe_count must be odd for round beds",
            ));
        }
        // Upstream reads `probe_count` only for rectangular beds; a round bed
        // counts from `round_probe_count` (`bed_mesh.py:386`, `bed_mesh.py:399`).
        let probe_count = if mesh_radius.is_some() {
            [round_probe_count, round_probe_count]
        } else {
            parse_config_pair(config, "probe_count", 3, 3)?
        };
        // A round bed takes its bounds from the radius — floored to 0.1mm as
        // upstream derives them — and never reads `mesh_min`/`mesh_max`
        // (`bed_mesh.py:382-396`); a rectangular bed requires both pairs and
        // refuses an inverted one (`bed_mesh.py:400-403`).
        let (mesh_min, mesh_max) = if let Some(radius) = mesh_radius {
            let radius = (radius * 10.0).floor() / 10.0;
            ([-radius, -radius], [radius, radius])
        } else {
            let min = two_floats(config, "mesh_min", None)?;
            let max = two_floats(config, "mesh_max", None)?;
            if max[0] <= min[0] || max[1] <= min[1] {
                return Err(ConfigError::new(
                    "bed_mesh: invalid min/max points".to_string(),
                ));
            }
            (min, max)
        };
        let mesh_pps = parse_config_pair(config, "mesh_pps", 2, 0)?;
        let faulty_regions = read_faulty_regions(config)?;
        // Upstream reads the name raw and lowercases it
        // (`bed_mesh.py:412-413`); membership is checked below.
        let algorithm = config
            .get("algorithm", Some("lagrange"))?
            .trim()
            .to_lowercase();

        let mut options = Self {
            mesh_min,
            mesh_max,
            probe_count,
            speed: config.get_float("speed", Some(50.0))?,
            horizontal_move_z: config.get_float("horizontal_move_z", Some(5.0))?,
            algorithm,
            fade_start: config.get_float("fade_start", Some(1.0))?,
            fade_end: config.get_float("fade_end", Some(0.0))?,
            fade_target: config.get_optional_float("fade_target")?,
            mesh_pps,
            bicubic_tension: config.get_float_bounded(
                "bicubic_tension",
                Some(0.2),
                Some(0.),
                Some(2.),
                None,
                None,
            )?,
            round_probe_count,
            mesh_radius,
            mesh_origin: two_floats(config, "mesh_origin", Some([0.0, 0.0]))?,
            move_check_distance: config.get_float("move_check_distance", Some(5.0))?,
            faulty_regions,
        };
        options.verify_algorithm()?;
        Ok(options)
    }

    /// Upstream's `BedMeshCalibrate._verify_algorithm`
    /// (`bed_mesh.py:416-456`), which load runs as it reads the section
    /// (`bed_mesh.py:418`) and a `BED_MESH_CALIBRATE` override runs again.
    ///
    /// Three rules decide what the sampler gets: interpolation off (`mesh_pps`
    /// zero on both axes) means `direct`; `lagrange` oscillates with more than
    /// six probed points per axis, so that is refused; `bicubic` needs four per
    /// axis and falls back to `lagrange` when the counts are merely small.
    ///
    /// # Errors
    /// An `algorithm` that is neither `lagrange` nor `bicubic` — upstream's
    /// `ALGOS` (`bed_mesh.py:325`), which does **not** include `direct`, so a
    /// config cannot ask for it — and the two probe-count refusals above, each
    /// in upstream's wording.
    fn verify_algorithm(&mut self) -> Result<(), ConfigError> {
        if !matches!(self.algorithm.as_str(), "lagrange" | "bicubic") {
            return Err(ConfigError::new(format!(
                "bed_mesh: Unknown algorithm <{}>",
                self.algorithm
            )));
        }
        let [x_count, y_count] = self.counts();
        let max_probe_count = x_count.max(y_count);
        let min_probe_count = x_count.min(y_count);
        if self.mesh_pps[0].max(self.mesh_pps[1]) == 0 {
            // Interpolation disabled (`bed_mesh.py:426-428`).
            self.algorithm = "direct".to_string();
        } else if self.algorithm == "lagrange" && max_probe_count > 6 {
            return Err(ConfigError::new(format!(
                "bed_mesh: cannot exceed a probe_count of 6 when using lagrange \
                 interpolation. Configured Probe Count: {x_count}, {y_count}"
            )));
        } else if self.algorithm == "bicubic" && min_probe_count < 4 {
            if max_probe_count > 6 {
                return Err(ConfigError::new(format!(
                    "bed_mesh: invalid probe_count option when using bicubic \
                     interpolation.  Combination of 3 points on one axis with more \
                     than 6 on another is not permitted. Configured Probe Count: \
                     {x_count}, {y_count}"
                )));
            }
            // Too few points for the spline, so upstream falls back to
            // Lagrange (`bed_mesh.py:449-455`).
            self.algorithm = "lagrange".to_string();
        }
        Ok(())
    }

    /// The probing grid along each axis: `probe_count` for a rectangular bed,
    /// `round_probe_count` for a round one (`bed_mesh.py:379-415`).
    pub fn counts(&self) -> [i64; 2] {
        if self.mesh_radius.is_some() {
            [self.round_probe_count, self.round_probe_count]
        } else {
            self.probe_count
        }
    }
}

/// The `faulty_region_<N>_min` / `_max` pairs (`bed_mesh.py:827-864`).
fn read_faulty_regions(config: &ConfigWrapper) -> Result<Vec<FaultyRegion>, ConfigError> {
    let mut regions = Vec::new();
    for option in config.prefix_options("faulty_region_") {
        let Some(rest) = option.strip_prefix("faulty_region_") else {
            continue;
        };
        let Some((index, kind)) = rest.rsplit_once('_') else {
            continue;
        };
        if index.is_empty() || index.parse::<u32>().is_err() {
            continue;
        }
        if kind != "min" && kind != "max" {
            continue;
        }
        let corner = two_floats(config, &option, None)?;
        match regions.get_mut(index.parse::<usize>().unwrap_or(0).saturating_sub(1)) {
            Some(slot) => {
                let region: &mut FaultyRegion = slot;
                if kind == "min" {
                    region.0 = corner[0];
                    region.1 = corner[1];
                } else {
                    region.2 = corner[0];
                    region.3 = corner[1];
                }
            }
            None => {
                let mut region: FaultyRegion = (0.0, 0.0, 0.0, 0.0);
                if kind == "min" {
                    region.0 = corner[0];
                    region.1 = corner[1];
                } else {
                    region.2 = corner[0];
                    region.3 = corner[1];
                }
                regions.push(region);
            }
        }
    }
    Ok(regions)
}

/// Generate the probe points (`bed_mesh.py:ProbeManager.generate_points`).
///
/// Rows zigzag: even rows run left to right, odd rows right to left. The
/// distances are floored to hundredths, as upstream does, so the grid stays on
/// whole hundredths.
///
/// # Errors
/// "bed_mesh: min/max points too close together" when the spacing is under
/// 1mm.
pub fn generate_points(options: &BedMeshOptions) -> Result<Vec<(f64, f64)>, CommandError> {
    let [x_count, y_count] = options.counts();
    if x_count < 2 || y_count < 2 {
        return Err(CommandError::new(
            "bed_mesh: min/max points too close together",
        ));
    }
    let mut x_dist = (options.mesh_max[0] - options.mesh_min[0]) / (x_count as f64 - 1.0);
    let mut y_dist = (options.mesh_max[1] - options.mesh_min[1]) / (y_count as f64 - 1.0);
    x_dist = (x_dist * 100.0).floor() / 100.0;
    y_dist = (y_dist * 100.0).floor() / 100.0;
    if x_dist < 1.0 || y_dist < 1.0 {
        return Err(CommandError::new(
            "bed_mesh: min/max points too close together",
        ));
    }

    let round = options.mesh_radius.is_some();
    let (min_x, min_y, max_x);
    if round {
        y_dist = x_dist;
        let new_radius = (x_count / 2) as f64 * x_dist;
        min_x = -new_radius;
        min_y = -new_radius;
        max_x = new_radius;
    } else {
        min_x = options.mesh_min[0];
        min_y = options.mesh_min[1];
        max_x = min_x + x_dist * (x_count as f64 - 1.0);
    }

    let mut points = Vec::new();
    let mut pos_y = min_y;
    for row in 0..y_count {
        for column in 0..x_count {
            let pos_x = if row % 2 == 0 {
                min_x + column as f64 * x_dist
            } else {
                max_x - column as f64 * x_dist
            };
            if let Some(radius) = options.mesh_radius {
                // Round bed: only points inside the radius take part.
                if (pos_x * pos_x + pos_y * pos_y).sqrt() <= radius {
                    points.push((
                        options.mesh_origin[0] + pos_x,
                        options.mesh_origin[1] + pos_y,
                    ));
                }
            } else {
                points.push((pos_x, pos_y));
            }
        }
        pos_y += y_dist;
    }
    if points.is_empty() {
        return Err(CommandError::new("bed_mesh: No valid points found"));
    }
    Ok(points)
}

/// The interpolation algorithm a mesh is sampled with
/// (`bed_mesh.py:1343-1348`).
///
/// A config names [`MeshAlgo::Lagrange`] or [`MeshAlgo::Bicubic`] only — those
/// are upstream's `ALGOS` (`bed_mesh.py:325`) — and [`MeshAlgo::Direct`] is what
/// [`BedMeshOptions::verify_algorithm`] substitutes when interpolation is off,
/// so no config can ask for it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeshAlgo {
    /// A Lagrange polynomial through every probed point of the axis.
    Lagrange,
    /// A cardinal spline over the four probed points nearest the point.
    Bicubic,
    /// No interpolation: the probed grid is the mesh.
    Direct,
}

impl MeshAlgo {
    /// The algorithm one of the three names a loaded section can carry stands
    /// for.
    pub fn from_name(name: &str) -> Option<Self> {
        match name {
            "lagrange" => Some(Self::Lagrange),
            "bicubic" => Some(Self::Bicubic),
            "direct" => Some(Self::Direct),
            _ => None,
        }
    }
}

/// The parameters one mesh is sampled from — upstream's `mesh_params`
/// (`bed_mesh.py:1326-1343`): the keys of `PROFILE_OPTIONS`
/// (`bed_mesh.py:10-13`) plus the bounds `probe_finalize` reads off the
/// generated points (`bed_mesh.py:668-671`).
#[derive(Debug, Clone, PartialEq)]
pub struct MeshParams {
    /// The smallest X a probed point sits at.
    pub min_x: f64,
    /// The largest X.
    pub max_x: f64,
    /// The smallest Y.
    pub min_y: f64,
    /// The largest Y.
    pub max_y: f64,
    /// Probed points along X.
    pub x_count: usize,
    /// Probed points along Y.
    pub y_count: usize,
    /// Points interpolated between two probed points, per X segment.
    pub mesh_x_pps: usize,
    /// The same per Y segment.
    pub mesh_y_pps: usize,
    /// The verified interpolation algorithm.
    pub algo: MeshAlgo,
    /// The bicubic tension (`bicubic_tension`).
    pub tension: f64,
}

impl MeshParams {
    /// A calibration's parameters: the options' counts and interpolation
    /// settings, and the bounds of `points` — the generated points, from which
    /// upstream takes `min_x`…`max_y` (`bed_mesh.py:668-671`).
    ///
    /// # Panics
    /// When `points` is empty: `generate_points` refuses an empty grid
    /// (`bed_mesh: No valid points found`), so a calibration has points.
    pub fn from_options(options: &BedMeshOptions, points: &[(f64, f64)]) -> Self {
        assert!(
            !points.is_empty(),
            "a calibration probes at least one point"
        );
        let [x_count, y_count] = options.counts();
        let mut bounds = [
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        ];
        for (x, y) in points {
            bounds[0] = bounds[0].min(*x);
            bounds[1] = bounds[1].max(*x);
            bounds[2] = bounds[2].min(*y);
            bounds[3] = bounds[3].max(*y);
        }
        Self {
            min_x: bounds[0],
            max_x: bounds[1],
            min_y: bounds[2],
            max_y: bounds[3],
            x_count: x_count as usize,
            y_count: y_count as usize,
            mesh_x_pps: options.mesh_pps[0] as usize,
            mesh_y_pps: options.mesh_pps[1] as usize,
            algo: MeshAlgo::from_name(&options.algorithm)
                .expect("`BedMeshOptions::read` stores a verified algorithm name"),
            tension: options.bicubic_tension,
        }
    }
}

/// Where a [`ZMesh`]'s grid sits: the interpolated dimensions and the spacing
/// that places them (`bed_mesh.py:1346-1355`).
#[derive(Debug, Clone, Copy, PartialEq)]
struct MeshGeometry {
    /// The X the first mesh column sits at.
    min_x: f64,
    /// The Y the first mesh row sits at.
    min_y: f64,
    /// The distance between two mesh columns.
    x_dist: f64,
    /// The distance between two mesh rows.
    y_dist: f64,
    /// Mesh columns.
    mesh_x_count: usize,
    /// Mesh rows.
    mesh_y_count: usize,
    /// Probed points along X (`mesh_params['x_count']`).
    x_count: usize,
    /// Probed points along Y.
    y_count: usize,
    /// How many mesh columns one probed segment spans — `mesh_x_pps + 1`.
    x_mult: usize,
    /// The same for rows.
    y_mult: usize,
}

impl MeshGeometry {
    /// The geometry `mesh_x_pps` / `mesh_y_pps` interpolated points per probed
    /// segment produce.
    ///
    /// A probed axis is at least 3 points long — the config refuses a smaller
    /// `probe_count` (`parse_config_pair(…, 3)`) and a round bed a smaller
    /// `round_probe_count` — which is what keeps the spacing's divisor and the
    /// spline's `last_point - x_mult` off the edge.
    fn new(params: &MeshParams) -> Self {
        // `(px_cnt - 1) * mesh_x_pps + px_cnt` (`bed_mesh.py:1350`), written as
        // the same count with both endpoints kept in the multiplier.
        let mesh_x_count = (params.x_count - 1) * (params.mesh_x_pps + 1) + 1;
        let mesh_y_count = (params.y_count - 1) * (params.mesh_y_pps + 1) + 1;
        Self {
            min_x: params.min_x,
            min_y: params.min_y,
            x_dist: (params.max_x - params.min_x) / (mesh_x_count - 1) as f64,
            y_dist: (params.max_y - params.min_y) / (mesh_y_count - 1) as f64,
            mesh_x_count,
            mesh_y_count,
            x_count: params.x_count,
            y_count: params.y_count,
            x_mult: params.mesh_x_pps + 1,
            y_mult: params.mesh_y_pps + 1,
        }
    }

    /// The X mesh column `index` sits at (`get_x_coordinate`,
    /// `bed_mesh.py:1424-1425`).
    fn x_coordinate(&self, index: usize) -> f64 {
        self.min_x + self.x_dist * index as f64
    }

    /// The Y mesh row `index` sits at (`get_y_coordinate`,
    /// `bed_mesh.py:1426-1427`).
    fn y_coordinate(&self, index: usize) -> f64 {
        self.min_y + self.y_dist * index as f64
    }
}

/// Which axis of the mesh a lookup walks (`bed_mesh.py:1466-1475`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MeshAxis {
    /// The mesh's columns.
    X,
    /// Its rows.
    Y,
}

/// A calibration's interpolation grid — upstream's `ZMesh`
/// (`bed_mesh.py:1321`).
///
/// The probed grid stays on [`LoadedMesh::rows`], where `bed_mesh/dump_mesh`
/// reads it; upstream keeps both matrices on the `ZMesh` and *hands* the probed
/// one to the sampler (`build_mesh`, `bed_mesh.py:1405-1408`), which is how
/// [`ZMesh::build`] takes it here.
#[derive(Debug, Clone, PartialEq)]
pub struct ZMesh {
    /// `mesh_y_count` rows of `mesh_x_count` sampled values.
    mesh_matrix: Vec<Vec<f64>>,
    /// The grid the values sit on.
    geometry: MeshGeometry,
}

impl ZMesh {
    /// Sample the mesh from a probed grid — `ZMesh.__init__` plus
    /// `build_mesh` (`bed_mesh.py:1321-1355`, `bed_mesh.py:1405-1408`).
    ///
    /// # Panics
    /// When `probed` is not `params.y_count` rows of `params.x_count` values,
    /// or when either axis is shorter than 3 points. The samplers index the
    /// probed grid by the mesh's own geometry (`bed_mesh.py:1482-1488`), so a
    /// grid that does not line up has no meaning; every caller hands over a full
    /// grid — a rectangular bed generates one point per cell, a round one is
    /// padded to that (`bed_mesh.py:749-771`) — and the config refuses counts
    /// below 3, so a caller's mistake fails loudly here instead of silently
    /// reading the wrong cell.
    pub fn build(probed: &[Vec<f64>], params: &MeshParams) -> Self {
        assert_eq!(
            probed.len(),
            params.y_count,
            "a probed grid holds one row per Y probe point"
        );
        assert!(
            probed.iter().all(|row| row.len() == params.x_count),
            "every probed row holds one value per X probe point"
        );
        assert!(
            params.x_count >= 3 && params.y_count >= 3,
            "a mesh needs 3 probe points per axis"
        );
        let geometry = MeshGeometry::new(params);
        let mesh_matrix = match params.algo {
            MeshAlgo::Direct => probed.to_vec(),
            MeshAlgo::Lagrange => sample_lagrange(probed, &geometry),
            MeshAlgo::Bicubic => sample_bicubic(probed, &geometry, params.tension),
        };
        Self {
            mesh_matrix,
            geometry,
        }
    }

    /// The sampled grid: `mesh_y_count` rows of `mesh_x_count` values.
    pub fn mesh_matrix(&self) -> &[Vec<f64>] {
        &self.mesh_matrix
    }

    /// The grid the way clients read it — upstream's `get_mesh_matrix` rounds
    /// every value to 6 decimals (`bed_mesh.py:1360-1364`).
    pub fn get_mesh_matrix(&self) -> Vec<Vec<f64>> {
        self.mesh_matrix
            .iter()
            .map(|row| row.iter().map(|z| round_six(*z)).collect())
            .collect()
    }

    /// The Z the mesh holds at `(x, y)`, bilinear over the four grid points
    /// around it — `calc_z` (`bed_mesh.py:1428-1439`).
    ///
    /// A coordinate outside the mesh clamps to its nearest edge, which is what
    /// `_get_linear_index`'s `constrain` does. Upstream adds `mesh_offsets` to
    /// the coordinate first; nothing sets those yet (`BED_MESH_OFFSET`), so the
    /// lookup uses the coordinate as given.
    pub fn calc_z(&self, x: f64, y: f64) -> f64 {
        let (t_x, x_idx) = self.linear_index(x, MeshAxis::X);
        let (t_y, y_idx) = self.linear_index(y, MeshAxis::Y);
        let table = &self.mesh_matrix;
        let z0 = lerp(t_x, table[y_idx][x_idx], table[y_idx][x_idx + 1]);
        let z1 = lerp(t_x, table[y_idx + 1][x_idx], table[y_idx + 1][x_idx + 1]);
        lerp(t_y, z0, z1)
    }

    /// The mesh segment `coord` falls in and where it falls inside it —
    /// `_get_linear_index` (`bed_mesh.py:1465-1477`): the segment index clamped
    /// onto the mesh, and the position within it clamped to `[0, 1]`.
    fn linear_index(&self, coord: f64, axis: MeshAxis) -> (f64, usize) {
        let (min, count, dist) = match axis {
            MeshAxis::X => (
                self.geometry.min_x,
                self.geometry.mesh_x_count,
                self.geometry.x_dist,
            ),
            MeshAxis::Y => (
                self.geometry.min_y,
                self.geometry.mesh_y_count,
                self.geometry.y_dist,
            ),
        };
        let segment = ((coord - min) / dist).floor() as i64;
        let segment = segment.clamp(0, count as i64 - 2) as usize;
        let segment_start = match axis {
            MeshAxis::X => self.geometry.x_coordinate(segment),
            MeshAxis::Y => self.geometry.y_coordinate(segment),
        };
        (constrain((coord - segment_start) / dist, 0., 1.), segment)
    }
}

/// Upstream's `bed_mesh.constrain` (`bed_mesh.py:30-31`).
fn constrain(value: f64, min: f64, max: f64) -> f64 {
    max.min(min.max(value))
}

/// Upstream's `bed_mesh.lerp` (`bed_mesh.py:34-35`).
fn lerp(t: f64, v0: f64, v1: f64) -> f64 {
    (1. - t) * v0 + t * v1
}

/// Python's `round(value, 6)`, which is how `get_mesh_matrix` reports a mesh
/// (`bed_mesh.py:1361-1363`).
///
/// Scaling, rounding and scaling back agrees with Python's decimal round except
/// on an exact tie at the sixth decimal, which Python resolves to even; the
/// values are probed readings, orders of magnitude coarser than that.
fn round_six(value: f64) -> f64 {
    (value * 1e6).round() / 1e6
}

/// The seed both samplers start from: every probed value at its own grid
/// position `(i * x_mult, j * y_mult)`, zero everywhere else
/// (`bed_mesh.py:1482-1488`, `bed_mesh.py:1535-1540`).
fn seed_matrix(probed: &[Vec<f64>], geometry: &MeshGeometry) -> Vec<Vec<f64>> {
    let mut mesh = vec![vec![0.; geometry.mesh_x_count]; geometry.mesh_y_count];
    for (row_index, row) in mesh.iter_mut().enumerate() {
        if row_index % geometry.y_mult != 0 {
            continue;
        }
        for (column, value) in row.iter_mut().enumerate() {
            if column % geometry.x_mult != 0 {
                continue;
            }
            *value = probed[row_index / geometry.y_mult][column / geometry.x_mult];
        }
    }
    mesh
}

/// The mesh a Lagrange polynomial through every probed point produces —
/// `_sample_lagrange` (`bed_mesh.py:1480-1511`).
///
/// Each row that holds probed values is interpolated along X first, then each
/// column along Y from those rows, which is the order upstream uses to keep the
/// control points of every pass final before it runs.
fn sample_lagrange(probed: &[Vec<f64>], geometry: &MeshGeometry) -> Vec<Vec<f64>> {
    let mut mesh = seed_matrix(probed, geometry);
    let (x_points, y_points) = lagrange_coords(geometry);
    for row in 0..geometry.mesh_y_count {
        if row % geometry.y_mult != 0 {
            continue;
        }
        for column in 0..geometry.mesh_x_count {
            if column % geometry.x_mult == 0 {
                continue;
            }
            let x = geometry.x_coordinate(column);
            let value = calc_lagrange(&x_points, x, &mesh, row, MeshAxis::X, geometry);
            mesh[row][column] = value;
        }
    }
    for column in 0..geometry.mesh_x_count {
        for row in 0..geometry.mesh_y_count {
            if row % geometry.y_mult == 0 {
                continue;
            }
            let y = geometry.y_coordinate(row);
            let value = calc_lagrange(&y_points, y, &mesh, column, MeshAxis::Y, geometry);
            mesh[row][column] = value;
        }
    }
    mesh
}

/// The X and Y the probed points sit at, in mesh columns —
/// `_get_lagrange_coords` (`bed_mesh.py:1512-1520`).
fn lagrange_coords(geometry: &MeshGeometry) -> (Vec<f64>, Vec<f64>) {
    let x_points = (0..geometry.x_count)
        .map(|i| geometry.x_coordinate(i * geometry.x_mult))
        .collect();
    let y_points = (0..geometry.y_count)
        .map(|j| geometry.y_coordinate(j * geometry.y_mult))
        .collect();
    (x_points, y_points)
}

/// The Lagrange polynomial through `points`, evaluated at `c` —
/// `_calc_lagrange` (`bed_mesh.py:1521-1539`).
///
/// `index` names the row (along X) or column (along Y) being interpolated: an X
/// pass reads the probed values of that row (`mesh_matrix[row][i * x_mult]`), a
/// Y pass the same column of every probed row (`mesh_matrix[i * y_mult][col]`).
fn calc_lagrange(
    points: &[f64],
    c: f64,
    mesh: &[Vec<f64>],
    index: usize,
    axis: MeshAxis,
    geometry: &MeshGeometry,
) -> f64 {
    let mut total = 0.;
    for (i, point) in points.iter().enumerate() {
        let mut numerator = 1.;
        let mut denominator = 1.;
        for (j, other) in points.iter().enumerate() {
            if i == j {
                continue;
            }
            numerator *= c - other;
            denominator *= point - other;
        }
        let z = match axis {
            MeshAxis::X => mesh[index][i * geometry.x_mult],
            MeshAxis::Y => mesh[i * geometry.y_mult][index],
        };
        total += z * numerator / denominator;
    }
    total
}

/// The mesh a cardinal spline through the four probed points nearest each point
/// produces — `_sample_bicubic` (`bed_mesh.py:1540-1562`).
///
/// Same order as `_sample_lagrange`: rows along X, then columns along Y.
fn sample_bicubic(probed: &[Vec<f64>], geometry: &MeshGeometry, tension: f64) -> Vec<Vec<f64>> {
    let mut mesh = seed_matrix(probed, geometry);
    for row in 0..geometry.mesh_y_count {
        if row % geometry.y_mult != 0 {
            continue;
        }
        for column in 0..geometry.mesh_x_count {
            if column % geometry.x_mult == 0 {
                continue;
            }
            let control = x_control_points(&mesh, geometry, column, row);
            mesh[row][column] = cardinal_spline(&control, tension);
        }
    }
    for column in 0..geometry.mesh_x_count {
        for row in 0..geometry.mesh_y_count {
            if row % geometry.y_mult == 0 {
                continue;
            }
            let control = y_control_points(&mesh, geometry, column, row);
            mesh[row][column] = cardinal_spline(&control, tension);
        }
    }
    mesh
}

/// The four probed values around mesh column `column` of row `row` and the
/// position between the middle two — `_get_x_ctl_pts` (`bed_mesh.py:1563-1594`).
///
/// A point in the first or last segment has no neighbour on one side, so the
/// nearest probed value stands in for it (`p0 = p1`, `p2 = p3`).
/// `last_point` is the last probed column that still has a segment after it
/// (`bed_mesh.py:1567`); in between, the probed column below `column` starts its
/// segment, which is the `i` upstream's search loop stops at.
fn x_control_points(
    mesh: &[Vec<f64>],
    geometry: &MeshGeometry,
    column: usize,
    row: usize,
) -> [f64; 5] {
    let x_mult = geometry.x_mult;
    let last_point = geometry.mesh_x_count - 1 - x_mult;
    let values = &mesh[row];
    if column < x_mult {
        [
            values[0],
            values[0],
            values[x_mult],
            values[2 * x_mult],
            column as f64 / x_mult as f64,
        ]
    } else if column > last_point {
        [
            values[last_point - x_mult],
            values[last_point],
            values[last_point + x_mult],
            values[last_point + x_mult],
            (column - last_point) as f64 / x_mult as f64,
        ]
    } else {
        let start = (column / x_mult) * x_mult;
        [
            values[start - x_mult],
            values[start],
            values[start + x_mult],
            values[start + 2 * x_mult],
            (column - start) as f64 / x_mult as f64,
        ]
    }
}

/// The Y twin of [`x_control_points`] — `_get_y_ctl_pts`
/// (`bed_mesh.py:1595-1626`): the same four values read down column `column`.
fn y_control_points(
    mesh: &[Vec<f64>],
    geometry: &MeshGeometry,
    column: usize,
    row: usize,
) -> [f64; 5] {
    let y_mult = geometry.y_mult;
    let last_point = geometry.mesh_y_count - 1 - y_mult;
    if row < y_mult {
        [
            mesh[0][column],
            mesh[0][column],
            mesh[y_mult][column],
            mesh[2 * y_mult][column],
            row as f64 / y_mult as f64,
        ]
    } else if row > last_point {
        [
            mesh[last_point - y_mult][column],
            mesh[last_point][column],
            mesh[last_point + y_mult][column],
            mesh[last_point + y_mult][column],
            (row - last_point) as f64 / y_mult as f64,
        ]
    } else {
        let start = (row / y_mult) * y_mult;
        [
            mesh[start - y_mult][column],
            mesh[start][column],
            mesh[start + y_mult][column],
            mesh[start + 2 * y_mult][column],
            (row - start) as f64 / y_mult as f64,
        ]
    }
}

/// The cardinal spline through `p0`…`p3` at `t`, with the control values as
/// upstream packs them — `_cardinal_spline` (`bed_mesh.py:1613-1621`).
fn cardinal_spline(p: &[f64; 5], tension: f64) -> f64 {
    let t = p[4];
    let t2 = t * t;
    let t3 = t2 * t;
    let m1 = tension * (p[2] - p[0]);
    let m2 = tension * (p[3] - p[1]);
    let a = p[1] * (2. * t3 - 3. * t2 + 1.);
    let b = p[2] * (-2. * t3 + 3. * t2);
    let c = m1 * (t3 - 2. * t2 + t);
    let d = m2 * (t3 - t2);
    a + b + c + d
}

/// One loaded mesh: which profile it answers as, the grid that was probed and
/// the interpolation grid sampled from it.
///
/// The data half of a `bed_mesh/dump_mesh` reply — upstream reads the same
/// things off the loaded `z_mesh` (`bed_mesh.py:296-302`).
#[derive(Debug, Clone, PartialEq)]
pub struct LoadedMesh {
    /// The profile the grid answers as (`z_mesh.get_profile_name()`).
    pub name: String,
    /// The probed Z values, one row per Y line, each row X-ascending
    /// (`z_mesh.get_probed_matrix()`).
    pub rows: Vec<Vec<f64>>,
    /// The interpolation grid sampled from `rows` (`z_mesh`).
    pub z_mesh: ZMesh,
}

/// Group one calibration's results into the grid upstream stores
/// (`bed_mesh.py:713-741`): a new row whenever Y moves by more than upstream's
/// `abs_tol=.1`, each row X-ascending — the generated points zigzag, so the
/// order they are probed in is not the order they sit in the grid.
fn rows_by_y(points: &[(f64, f64)], values: &[f64]) -> Vec<Vec<f64>> {
    debug_assert_eq!(points.len(), values.len());
    let mut rows: Vec<Vec<f64>> = Vec::new();
    let mut row: Vec<(f64, f64)> = Vec::new();
    let mut row_y: Option<f64> = None;
    for (&(x, y), &z) in points.iter().zip(values) {
        let same_row = row_y.is_some_and(|prev| (y - prev).abs() <= 0.1);
        if !same_row {
            if !row.is_empty() {
                rows.push(take_row(&mut row));
            }
            row_y = Some(y);
        }
        row.push((x, z));
    }
    if !row.is_empty() {
        rows.push(take_row(&mut row));
    }
    rows
}

/// One row, sorted X-ascending and taken out of `row`.
fn take_row(row: &mut Vec<(f64, f64)>) -> Vec<f64> {
    row.sort_by(|left, right| {
        left.0
            .partial_cmp(&right.0)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    row.drain(..).map(|(_, z)| z).collect()
}

/// Pad a round bed's rows out to the full square grid — the last step of
/// upstream's `probe_finalize` (`bed_mesh.py:749-771`).
///
/// A round bed only probes the points inside its radius, so a row holds fewer
/// values than `x_count`; upstream repeats each row's outermost value on both
/// sides, which is the square probed grid the interpolation meshes are built
/// from. Rows at the top and bottom of the circle end up entirely flat.
///
/// # Errors
/// `bed_mesh: incorrect number of points sampled on X` for a row holding an
/// even number of values. Upstream refuses it the same way
/// (`bed_mesh.py:753-759`): a row across a round bed is symmetric about the
/// centre, so it can only hold an odd count. (Upstream's message goes on to
/// print the probed table; this keeps its first line.)
fn pad_round_rows(rows: Vec<Vec<f64>>, x_count: usize) -> Result<Vec<Vec<f64>>, CommandError> {
    let mut padded = Vec::with_capacity(rows.len());
    for mut row in rows {
        let width = row.len();
        if width % 2 == 0 {
            return Err(CommandError::new(
                "bed_mesh: incorrect number of points sampled on X",
            ));
        }
        // A row that is already full is left alone (`bed_mesh.py:760-761`).
        let buffer = x_count.saturating_sub(width) / 2;
        if buffer > 0 {
            let left = vec![row[0]; buffer];
            let right = vec![row[width - 1]; buffer];
            row.splice(0..0, left);
            row.extend(right);
        }
        padded.push(row);
    }
    Ok(padded)
}

/// One configured `[bed_mesh]` (`bed_mesh.py:BedMesh` + `BedMeshCalibrate`).
pub struct BedMesh {
    /// The options as read (shared with the command handlers).
    options: Arc<Mutex<BedMeshOptions>>,
    /// The last calibration's grid, if any (shared likewise).
    mesh: Arc<Mutex<Option<LoadedMesh>>>,
}

impl BedMesh {
    /// Read the section's options and register the commands.
    ///
    /// # Errors
    /// As [`BedMeshOptions::read`], or when a command name is taken.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let options = BedMeshOptions::read(config)?;
        let identifier = config.identifier().to_string();
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the loader registers `gcode` first");

        let mesh = Arc::new(Mutex::new(None));
        let options = Arc::new(Mutex::new(options));
        let printer_weak = Arc::downgrade(printer);

        // BED_MESH_CALIBRATE
        {
            let options = Arc::clone(&options);
            let mesh = Arc::clone(&mesh);
            let printer_weak = printer_weak.clone();
            gcode
                .register_command_with_params(
                    "BED_MESH_CALIBRATE",
                    Arc::new(move |gcmd| {
                        let options = Arc::clone(&options);
                        let mesh = Arc::clone(&mesh);
                        let printer_weak = printer_weak.clone();
                        Box::pin(async move {
                            let printer = printer_weak
                                .upgrade()
                                .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                            let toolhead = printer
                                .lookup_object_as::<ToolHeadObject>(TOOLHEAD_OBJECT)
                                .ok_or_else(|| CommandError::new("Printer is not ready"))?;
                            let probe = lookup_probe_session(&printer).ok_or_else(|| {
                                CommandError::new("bed_mesh: a [probe] section is required")
                            })?;

                            // Every axis the mesh moves along must be homed
                            // (`bed_mesh.py:update_config`).
                            let homed = toolhead.get_status(0.0)["homed_axes"]
                                .as_str()
                                .unwrap_or("")
                                .to_string();
                            if !homed.contains('x') || !homed.contains('y') || !homed.contains('z')
                            {
                                return Err(CommandError::new(
                                    "Must home all axes prior to bed mesh calibration",
                                ));
                            }

                            *mesh.lock().unwrap_or_else(|p| p.into_inner()) = None;

                            let (points, speed, move_z, algorithm) = {
                                let options = options.lock().unwrap_or_else(|p| p.into_inner());
                                (
                                    generate_points(&options)?,
                                    options.speed,
                                    options.horizontal_move_z,
                                    options.algorithm.clone(),
                                )
                            };
                            let offsets = probe.offsets();

                            // Probe every point in one session
                            // (`bed_mesh.py:ProbeManager.start_probe`).
                            probe.start_probe_session(gcmd)?;
                            let mut probed = Vec::with_capacity(points.len());
                            for (x, y) in &points {
                                let mut target =
                                    Coord::new(x - offsets.x, y - offsets.y, move_z, 0.0);
                                target.set_axis(Z_AXIS, move_z);
                                toolhead.move_to(target, speed)?;
                                probe.run_probe(gcmd).await?;
                                let results = probe.pull_probed_results();
                                let Some(result) = results.into_iter().next() else {
                                    return Err(CommandError::new(
                                        "bed_mesh: no probe result for a mesh point",
                                    ));
                                };
                                probed.push(result.z());
                            }
                            probe.end_probe_session()?;

                            // Store the grid: grouped into rows by Y, each
                            // row X-ascending (`bed_mesh.py:713-741`).
                            let rows = rows_by_y(&points, &probed);
                            let (rows, z_mesh) = {
                                let options = options.lock().unwrap_or_else(|p| p.into_inner());
                                // A round bed pads its short rows into the
                                // square grid the mesh needs
                                // (`bed_mesh.py:749-771`).
                                let rows = if options.mesh_radius.is_some() {
                                    pad_round_rows(rows, options.counts()[0] as usize)?
                                } else {
                                    rows
                                };
                                let params = MeshParams::from_options(&options, &points);
                                let z_mesh = ZMesh::build(&rows, &params);
                                (rows, z_mesh)
                            };
                            *mesh.lock().unwrap_or_else(|p| p.into_inner()) = Some(LoadedMesh {
                                name: DEFAULT_PROFILE.to_string(),
                                rows,
                                z_mesh,
                            });
                            gcmd.respond_info(&format!(
                                "Mesh Bed Leveling Complete ({algorithm} mesh stored, {} points)",
                                points.len()
                            ));
                            Ok(())
                        })
                    }),
                    Some("Calibrate the bed mesh"),
                    PROBE_PARAMS,
                    false,
                )
                .map_err(ConfigError::new)?;
        }

        // BED_MESH_CLEAR
        {
            let mesh = Arc::clone(&mesh);
            gcode
                .register_command(
                    "BED_MESH_CLEAR",
                    Arc::new(move |gcmd| {
                        let mesh = Arc::clone(&mesh);
                        Box::pin(async move {
                            *mesh.lock().unwrap_or_else(|p| p.into_inner()) = None;
                            gcmd.respond_info("Bed mesh cleared");
                            Ok(())
                        })
                    }),
                    Some("Clear the currently loaded bed mesh"),
                    false,
                )
                .map_err(ConfigError::new)?;
        }

        let _ = identifier;
        Ok(Self { options, mesh })
    }

    /// The options as read.
    pub fn options(&self) -> BedMeshOptions {
        self.options
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }

    /// The loaded mesh — the profile it answers as and its probed grid — or
    /// `None` while the bed holds no calibration (`bed_mesh.py:296`, which
    /// then answers `{}`).
    ///
    /// This is what `bed_mesh/dump_mesh` and `get_status` both read, so the
    /// two cannot disagree about what is loaded.
    pub fn loaded_mesh(&self) -> Option<LoadedMesh> {
        self.mesh.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }

    /// Put a grid in place without running a calibration — the tests' stand-in
    /// for a completed `BED_MESH_CALIBRATE`.
    ///
    /// It samples the interpolation grid the same way the command does, from
    /// the points the configured section generates; `rows` must therefore be a
    /// full `[y_count, x_count]` probed grid (which is what a calibration
    /// stores).
    #[cfg(test)]
    pub(crate) fn store_mesh_for_test(&self, name: &str, rows: Vec<Vec<f64>>) {
        let options = self.options.lock().unwrap_or_else(|p| p.into_inner());
        let points = generate_points(&options).expect("the test section probes a grid");
        let params = MeshParams::from_options(&options, &points);
        let z_mesh = ZMesh::build(&rows, &params);
        drop(options);
        *self.mesh.lock().unwrap_or_else(|p| p.into_inner()) = Some(LoadedMesh {
            name: name.to_string(),
            rows,
            z_mesh,
        });
    }
}

impl PrinterObject for BedMesh {
    fn get_status(&self, _eventtime: f64) -> Value {
        let options = self.options.lock().unwrap_or_else(|p| p.into_inner());
        let loaded = self.loaded_mesh();
        let profile_name = loaded
            .as_ref()
            .map(|mesh| mesh.name.clone())
            .unwrap_or_default();
        let (probed, mesh) = match loaded {
            Some(mesh) => (mesh.rows, mesh.z_mesh.get_mesh_matrix()),
            None => (Vec::new(), Vec::new()),
        };
        json!({
            "profile_name": profile_name,
            "mesh_min": options.mesh_min,
            "mesh_max": options.mesh_max,
            "probed_matrix": probed,
            "mesh_matrix": mesh,
            "profiles": {},
        })
    }
}

impl std::fmt::Debug for BedMesh {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BedMesh")
            .field(
                "options",
                &*self.options.lock().unwrap_or_else(|p| p.into_inner()),
            )
            .finish()
    }
}

/// Upstream's `load_config` for `[bed_mesh]`.
pub fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(BedMesh::new(config, printer)?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{ConfigSection, ConfigValue};

    fn section(options: &[(&str, &str)]) -> ConfigSection {
        let mut section = ConfigSection::new("bed_mesh", None);
        for (option, value) in options {
            section.parameters.insert(
                (*option).to_string(),
                ConfigValue::Single((*value).to_string()),
            );
        }
        section
    }

    fn options(options: &[(&str, &str)]) -> BedMeshOptions {
        BedMeshOptions::read(&ConfigWrapper::untracked(&section(options))).unwrap()
    }

    /// The `BedMeshOptions::read` error for one `probe_count` value, as the
    /// wording tests below ask for.
    fn probe_count_error(value: &str) -> String {
        BedMeshOptions::read(&ConfigWrapper::untracked(&section(&[
            ("mesh_min", "10,10"),
            ("mesh_max", "180,180"),
            ("probe_count", value),
        ])))
        .unwrap_err()
        .to_string()
    }

    /// One value is a square grid: upstream `parse_config_pair` duplicates it
    /// (`bed_mesh.py:43-44`), which is what sovol's `probe_count: 5` relies on.
    #[test]
    fn a_single_probe_count_value_means_a_square_grid() {
        let read = options(&[
            ("mesh_min", "28, 20"),
            ("mesh_max", "270, 270"),
            ("probe_count", "5"),
        ]);

        assert_eq!(read.probe_count, [5, 5]);
        assert_eq!(generate_points(&read).unwrap().len(), 25);
    }

    /// Two values keep each axis, either order (`bed_mesh.py:40-41`).
    #[test]
    fn a_probe_count_pair_keeps_each_axis() {
        let read = options(&[
            ("mesh_min", "10,10"),
            ("mesh_max", "180,180"),
            ("probe_count", "5, 3"),
        ]);

        assert_eq!(read.probe_count, [5, 3]);
    }

    /// Illegal `probe_count` values keep upstream's wording verbatim: three
    /// values are `malformed` with the raw text (`bed_mesh.py:42-43`), a
    /// non-number the parser's (`configfile.py:44-45`), and a count below
    /// `minval=3` upstream's `minimum of` line, whose section name carries no
    /// quotes (`bed_mesh.py:47-49`).
    #[test]
    fn illegal_probe_counts_keep_upstream_wording() {
        assert_eq!(
            probe_count_error("5, 5, 5"),
            "bed_mesh: malformed 'probe_count' value: 5, 5, 5"
        );
        assert_eq!(
            probe_count_error("left,30"),
            "Unable to parse option 'probe_count' in section 'bed_mesh'"
        );
        assert_eq!(
            probe_count_error("2, 2"),
            "Option 'probe_count' in section bed_mesh must have a minimum of 3"
        );
    }

    #[test]
    fn the_corpus_option_set_is_claimed() {
        let read = options(&[
            ("mesh_min", "10,10"),
            ("mesh_max", "180,180"),
            ("probe_count", "7, 7"),
            ("speed", "120"),
            ("algorithm", "bicubic"),
            ("horizontal_move_z", "10"),
            ("fade_start", "1.0"),
            ("fade_end", "10.0"),
            ("fade_target", "0"),
            ("mesh_pps", "2, 2"),
            ("bicubic_tension", "0.2"),
            ("round_probe_count", "5"),
            ("mesh_origin", "0, 0"),
            ("move_check_distance", "5.0"),
            ("faulty_region_1_min", "21.422, 87.126"),
            ("faulty_region_1_max", "42.922, 129.126"),
            ("faulty_region_2_min", "54.172, 97.376"),
            ("faulty_region_2_max", "100.172, 150.876"),
        ]);

        assert_eq!(read.mesh_min, [10.0, 10.0]);
        assert_eq!(read.mesh_max, [180.0, 180.0]);
        assert_eq!(read.probe_count, [7, 7]);
        assert_eq!(read.algorithm, "bicubic");
        assert_eq!(read.mesh_pps, [2, 2]);
        assert_eq!(
            read.faulty_regions,
            vec![
                (21.422, 87.126, 42.922, 129.126),
                (54.172, 97.376, 100.172, 150.876),
            ]
        );
    }

    #[test]
    fn a_rectangular_grid_zigzags_row_by_row() {
        let read = options(&[
            ("mesh_min", "0,0"),
            ("mesh_max", "20,20"),
            ("probe_count", "3, 3"),
        ]);
        let points = generate_points(&read).unwrap();

        assert_eq!(
            points,
            vec![
                (0.0, 0.0),
                (10.0, 0.0),
                (20.0, 0.0),
                (20.0, 10.0),
                (10.0, 10.0),
                (0.0, 10.0),
                (0.0, 20.0),
                (10.0, 20.0),
                (20.0, 20.0),
            ]
        );
    }

    #[test]
    fn the_spacing_is_floored_to_hundredths() {
        // 180 - 10 = 170 over 6 gaps is 28.33mm; upstream floors to 28.33.
        // A 7×7 grid needs `bicubic`, since `lagrange` refuses more than 6
        // probe points per axis (`_verify_algorithm`).
        let read = options(&[
            ("mesh_min", "10,10"),
            ("mesh_max", "180,180"),
            ("probe_count", "7, 7"),
            ("algorithm", "bicubic"),
        ]);
        let points = generate_points(&read).unwrap();

        assert_eq!(points[1], (38.33, 10.0));
        let (last_x, last_y) = *points.last().unwrap();
        assert!(
            (last_x - (10.0 + 28.33 * 6.0)).abs() < 1e-9
                && (last_y - (10.0 + 28.33 * 6.0)).abs() < 1e-9,
            "{last_x}, {last_y}"
        );
    }

    #[test]
    fn a_round_bed_keeps_only_points_inside_the_radius() {
        // A round bed never reads `mesh_min`/`mesh_max` — the bounds come from
        // `mesh_radius` (`bed_mesh.py:393-396`).
        let mut read = options(&[
            ("mesh_radius", "42"),
            ("mesh_origin", "0, 0"),
            ("probe_count", "5, 5"),
        ]);
        assert_eq!(read.mesh_min, [-42.0, -42.0]);
        assert_eq!(read.mesh_max, [42.0, 42.0]);
        read.round_probe_count = 5;
        let points = generate_points(&read).unwrap();

        // 5×5 grid, 21mm spacing (84mm over 4 gaps): the centre column/row
        // and the ±42 axis points are inside 42mm, the corners are not.
        assert_eq!(points.len(), 13);
        for (x, y) in points {
            assert!((x * x + y * y).sqrt() <= 42.0);
        }
    }

    #[test]
    fn points_too_close_together_are_refused() {
        // 5mm over 6 gaps is 0.83mm < 1mm (7 points need `bicubic`).
        let read = options(&[
            ("mesh_min", "0,0"),
            ("mesh_max", "5,5"),
            ("probe_count", "7, 7"),
            ("algorithm", "bicubic"),
        ]);
        let err = generate_points(&read).unwrap_err();

        assert_eq!(
            err.to_string(),
            "bed_mesh: min/max points too close together"
        );
    }

    #[test]
    fn a_round_mesh_uses_the_round_probe_count() {
        // 7 points like the corpus's round beds, which pair it with `bicubic`
        // (`config/printer-velleman-k8800-2017.cfg`).
        let read = options(&[
            ("probe_count", "3, 3"),
            ("round_probe_count", "7"),
            ("mesh_radius", "45"),
            ("algorithm", "bicubic"),
        ]);

        assert_eq!(read.counts(), [7, 7]);
    }

    /// One `mesh_pps` value fills both axes, two keep each order
    /// (`bed_mesh.py:409` → `bed_mesh.py:38-56`); `minval=0` bounds both.
    #[test]
    fn a_single_mesh_pps_value_fills_both_axes() {
        let rectangular = options(&[
            ("mesh_min", "10,10"),
            ("mesh_max", "180,180"),
            ("mesh_pps", "3"),
        ]);
        assert_eq!(rectangular.mesh_pps, [3, 3]);

        // The tronxy configs ship `mesh_pps: 0` (interpolation off).
        let off = options(&[
            ("mesh_min", "10,10"),
            ("mesh_max", "180,180"),
            ("mesh_pps", "0"),
        ]);
        assert_eq!(off.mesh_pps, [0, 0]);

        let pair = options(&[
            ("mesh_min", "10,10"),
            ("mesh_max", "180,180"),
            ("mesh_pps", "1, 4"),
        ]);
        assert_eq!(pair.mesh_pps, [1, 4]);
    }

    /// Illegal `mesh_pps` keeps upstream's wording verbatim: a wrong length is
    /// `malformed` with the raw text (`bed_mesh.py:42-43`), a non-number the
    /// parser's (`configfile.py:44-45`), and a negative count `minval=0`'s
    /// line, whose section name carries no quotes (`bed_mesh.py:47-49`).
    #[test]
    fn illegal_mesh_pps_keeps_upstream_wording() {
        let error = |value: &str| {
            BedMeshOptions::read(&ConfigWrapper::untracked(&section(&[
                ("mesh_min", "10,10"),
                ("mesh_max", "180,180"),
                ("mesh_pps", value),
            ])))
            .unwrap_err()
            .to_string()
        };

        assert_eq!(
            error("1, 2, 3"),
            "bed_mesh: malformed 'mesh_pps' value: 1, 2, 3"
        );
        assert_eq!(
            error("left,30"),
            "Unable to parse option 'mesh_pps' in section 'bed_mesh'"
        );
        assert_eq!(
            error("-1"),
            "Option 'mesh_pps' in section bed_mesh must have a minimum of 0"
        );
    }

    /// `mesh_min`/`mesh_max` are read only for a rectangular bed, with
    /// upstream's `getfloatlist(count=2)` wording: missing is `must be
    /// specified` (`configfile.py:37-38`), a wrong count `must have 2
    /// elements` (`configfile.py:100-101`), a bad number the parser's, and
    /// values are parsed *before* the count is checked (`configfile.py:98-101`).
    /// An inverted pair is refused outright (`bed_mesh.py:402-403`).
    #[test]
    fn rectangular_mesh_min_keeps_upstream_wording() {
        let error = |options: &[(&str, &str)]| {
            BedMeshOptions::read(&ConfigWrapper::untracked(&section(options)))
                .unwrap_err()
                .to_string()
        };

        assert_eq!(
            error(&[("mesh_max", "180,180")]),
            "Option 'mesh_min' in section 'bed_mesh' must be specified"
        );
        assert_eq!(
            error(&[("mesh_min", "10"), ("mesh_max", "180,180")]),
            "Option 'mesh_min' in section 'bed_mesh' must have 2 elements"
        );
        assert_eq!(
            error(&[("mesh_min", "left,10"), ("mesh_max", "180,180")]),
            "Unable to parse option 'mesh_min' in section 'bed_mesh'"
        );
        assert_eq!(
            error(&[("mesh_min", "180,180"), ("mesh_max", "10,10")]),
            "bed_mesh: invalid min/max points"
        );
    }

    /// A round bed needs no `mesh_min` at all — its bounds derive from
    /// `mesh_radius`, floored to 0.1mm as upstream floors the radius before
    /// it takes `min = -radius`, `max = radius` (`bed_mesh.py:392-396`).
    #[test]
    fn a_round_bed_takes_its_bounds_from_the_radius() {
        let read = options(&[
            ("mesh_radius", "65"),
            ("mesh_origin", "0, 0"),
            ("round_probe_count", "7"),
            ("algorithm", "bicubic"),
        ]);
        assert_eq!(read.mesh_min, [-65.0, -65.0]);
        assert_eq!(read.mesh_max, [65.0, 65.0]);

        let floored = options(&[("mesh_radius", "65.55")]);
        assert_eq!(floored.mesh_min, [-65.5, -65.5]);
        assert_eq!(floored.mesh_max, [65.5, 65.5]);
    }

    /// The probe order is not the grid order: rows zigzag, so the stored
    /// values are grouped by Y and each row sorted X-ascending
    /// (`bed_mesh.py:713-741`).
    #[test]
    fn probed_values_are_grouped_into_rows_by_y() {
        let read = options(&[
            ("mesh_min", "0,0"),
            ("mesh_max", "40,40"),
            ("probe_count", "3,3"),
        ]);
        let points = generate_points(&read).unwrap();
        let values: Vec<f64> = (0..points.len()).map(|index| index as f64 / 10.).collect();

        // Row 1 is probed right to left, so its three values come back
        // reversed once the row is sorted by X.
        assert_eq!(
            rows_by_y(&points, &values),
            vec![
                vec![0.0, 0.1, 0.2],
                vec![0.5, 0.4, 0.3],
                vec![0.6, 0.7, 0.8],
            ]
        );
    }

    /// `get_status` reports the interpolation grid while `probed_matrix` keeps
    /// the grid as probed, so a client can tell the two apart: a 3×3 probe with
    /// the default `mesh_pps: 2` becomes a 7×7 mesh whose probed positions
    /// still hold their probed values (`bed_mesh.py:230-247`).
    #[tokio::test]
    async fn get_status_reports_the_loaded_profile_and_grid() {
        use crate::core::klippy::reactor::ManualReactor;

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let bed = BedMesh::new(
            &ConfigWrapper::untracked(&section(&[
                ("mesh_min", "0,0"),
                ("mesh_max", "100,100"),
                ("probe_count", "3,3"),
            ])),
            &printer,
        )
        .unwrap();

        assert_eq!(bed.get_status(0.0)["profile_name"], "");
        bed.store_mesh_for_test(
            "default",
            vec![
                vec![0.1, 0.2, 0.3],
                vec![0.4, 0.5, 0.6],
                vec![0.7, 0.8, 0.9],
            ],
        );

        let status = bed.get_status(0.0);
        assert_eq!(status["profile_name"], "default");
        assert_eq!(
            status["probed_matrix"],
            serde_json::json!([[0.1, 0.2, 0.3], [0.4, 0.5, 0.6], [0.7, 0.8, 0.9]])
        );
        let mesh = status["mesh_matrix"].as_array().unwrap();
        assert_eq!(mesh.len(), 7);
        assert!(mesh.iter().all(|row| row.as_array().unwrap().len() == 7));
        // The probed positions keep their probed values …
        assert_eq!(mesh[0][0], 0.1);
        assert_eq!(mesh[0][3], 0.2);
        assert_eq!(mesh[0][6], 0.3);
        assert_eq!(mesh[6][0], 0.7);
        assert_eq!(mesh[6][6], 0.9);
        // … and the points between them are interpolated, so the two matrices
        // are no longer the same grid.
        assert_ne!(status["mesh_matrix"], status["probed_matrix"]);
        assert!(mesh[1][1].as_f64().unwrap() > 0.1);
    }

    // -----------------------------------------------------------------------
    // The interpolation grid
    // -----------------------------------------------------------------------

    /// A quadratic surface. A Lagrange polynomial through an axis' probe points
    /// reproduces any polynomial up to degree 4, so a correct interpolation
    /// matches this surface everywhere, not only at the probed points.
    fn surface(x: f64, y: f64) -> f64 {
        0.0001 * (x * x + y * y)
    }

    /// The probed grid `options` describes, with `surface` sampled at every
    /// point — the two things a calibration hands to the mesh.
    fn probed_grid(
        options: &BedMeshOptions,
        surface: impl Fn(f64, f64) -> f64,
    ) -> (MeshParams, Vec<Vec<f64>>) {
        let points = generate_points(options).unwrap();
        let params = MeshParams::from_options(options, &points);
        let values: Vec<f64> = points.iter().map(|&(x, y)| surface(x, y)).collect();
        let rows = rows_by_y(&points, &values);
        let [x_count, y_count] = options.counts();
        assert_eq!(rows.len(), y_count as usize);
        assert!(rows.iter().all(|row| row.len() == x_count as usize));
        (params, rows)
    }

    /// The mesh holds `(x_count - 1) * (mesh_x_pps + 1) + 1` columns (and the
    /// same number of rows), so `mesh_pps` is how many points the interpolation
    /// fills per segment (`bed_mesh.py:1350-1351`).
    #[test]
    fn the_mesh_grid_grows_by_mesh_pps() {
        for (pps, expected) in [("1", 9), ("2", 13), ("3", 17)] {
            let read = options(&[
                ("mesh_min", "0,0"),
                ("mesh_max", "100,100"),
                ("probe_count", "5,5"),
                ("mesh_pps", pps),
            ]);
            let (params, rows) = probed_grid(&read, surface);
            let mesh = ZMesh::build(&rows, &params);

            assert_eq!(mesh.mesh_matrix().len(), expected);
            assert!(mesh.mesh_matrix().iter().all(|row| row.len() == expected));
        }
    }

    /// `mesh_pps: 0` on both axes turns interpolation off: `_verify_algorithm`
    /// rewrites the algorithm to `direct` (`bed_mesh.py:426-428`) and the mesh
    /// is the probed grid, cell for cell (`bed_mesh.py:1478-1479`).
    #[test]
    fn no_mesh_pps_leaves_the_probed_grid_as_the_mesh() {
        let read = options(&[
            ("mesh_min", "0,0"),
            ("mesh_max", "100,100"),
            ("probe_count", "3,3"),
            ("mesh_pps", "0"),
        ]);

        assert_eq!(read.algorithm, "direct");
        let (params, rows) = probed_grid(&read, surface);
        assert_eq!(params.algo, MeshAlgo::Direct);
        let mesh = ZMesh::build(&rows, &params);
        assert_eq!(mesh.mesh_matrix(), rows.as_slice());
    }

    /// A Lagrange mesh passes through every probed point exactly: the sampler
    /// copies each probed value to its own grid position and fills only the
    /// points between (`bed_mesh.py:1482-1495`).
    #[test]
    fn a_lagrange_mesh_holds_the_probed_values_at_the_probed_points() {
        for (pps, stride) in [("1", 2), ("2", 3)] {
            let read = options(&[
                ("mesh_min", "0,0"),
                ("mesh_max", "100,100"),
                ("probe_count", "5,5"),
                ("mesh_pps", pps),
            ]);
            let (params, rows) = probed_grid(&read, surface);
            let mesh = ZMesh::build(&rows, &params);

            for (row, probed_row) in rows.iter().enumerate() {
                for (column, probed) in probed_row.iter().enumerate() {
                    assert_eq!(mesh.mesh_matrix()[row * stride][column * stride], *probed);
                }
            }
        }
    }

    /// …and between them it reproduces the quadratic, because the polynomial
    /// through the five probe points of an axis is exact up to degree 4
    /// (`bed_mesh.py:1521-1539`).
    ///
    /// The tolerance is floating-point only: the Lagrange denominators are
    /// products of four ~100mm distances (~1e7) against numerators of the same
    /// size, carrying ~1e-15 relative error on values of order 1 — 1e-9 leaves
    /// four orders of margin over that.
    #[test]
    fn a_lagrange_mesh_reproduces_a_quadratic_between_the_probed_points() {
        let read = options(&[
            ("mesh_min", "0,0"),
            ("mesh_max", "100,100"),
            ("probe_count", "5,5"),
            ("mesh_pps", "2"),
        ]);
        let (params, rows) = probed_grid(&read, surface);
        let mesh = ZMesh::build(&rows, &params);
        let width = mesh.mesh_matrix()[0].len();
        let x_dist = (params.max_x - params.min_x) / (width - 1) as f64;
        let y_dist = (params.max_y - params.min_y) / (width - 1) as f64;

        for (row, values) in mesh.mesh_matrix().iter().enumerate() {
            for (column, value) in values.iter().enumerate() {
                let x = params.min_x + x_dist * column as f64;
                let y = params.min_y + y_dist * row as f64;
                assert!(
                    (value - surface(x, y)).abs() < 1e-9,
                    "({x}, {y}) interpolates to {value}, surface is {}",
                    surface(x, y)
                );
            }
        }
    }

    /// A bicubic mesh passes through the probed points too (same seed), and
    /// `bicubic_tension` decides how the points between them bend
    /// (`bed_mesh.py:1540-1562`, `bed_mesh.py:1613-1621`).
    #[test]
    fn a_bicubic_mesh_holds_the_probed_values_and_follows_the_tension() {
        let build = |tension: &str| {
            let read = options(&[
                ("mesh_min", "0,0"),
                ("mesh_max", "90,90"),
                ("probe_count", "4,4"),
                ("mesh_pps", "1"),
                ("algorithm", "bicubic"),
                ("bicubic_tension", tension),
            ]);
            let (params, rows) = probed_grid(&read, surface);
            (ZMesh::build(&rows, &params), rows)
        };
        let (untensioned, rows) = build("0.0");
        let (tensioned, _) = build("0.2");

        // 4 probe points, one point between each: 7 columns.
        assert_eq!(untensioned.mesh_matrix().len(), 7);
        for (row, probed_row) in rows.iter().enumerate() {
            for (column, probed) in probed_row.iter().enumerate() {
                assert_eq!(untensioned.mesh_matrix()[row * 2][column * 2], *probed);
            }
        }
        // At tension 0 both spline tangents are zero, so the point in the
        // middle of a segment (t = 0.5) weighs its neighbours `2t³-3t²+1 =
        // -2t³+3t² = 0.5` each (`bed_mesh.py:1613-1621`) — the average of the
        // two probed values the segment spans. Mesh column 1 is halfway between
        // probed columns 0 and 1, and reads its control values from mesh
        // columns 0 and 2 (`bed_mesh.py:1568-1572`).
        let middle = (rows[0][0] + rows[0][1]) / 2.;
        assert!((untensioned.mesh_matrix()[0][1] - middle).abs() < 1e-12);
        // Tension pulls that point off the average.
        assert_ne!(
            tensioned.mesh_matrix()[0][1],
            untensioned.mesh_matrix()[0][1]
        );
    }

    /// `calc_z` is the lookup the mesh answers with: a grid point answers its
    /// own value, a point between four of them the bilinear mix, and a point
    /// off the mesh the nearest edge (`bed_mesh.py:1428-1439`), with the
    /// position within the segment clamped to `[0, 1]`
    /// (`bed_mesh.py:1465-1477`).
    #[test]
    fn calc_z_reads_the_mesh_and_clamps_outside_it() {
        let read = options(&[
            ("mesh_min", "0,0"),
            ("mesh_max", "100,100"),
            ("probe_count", "3,3"),
            ("mesh_pps", "1"),
        ]);
        let (params, rows) = probed_grid(&read, surface);
        let mesh = ZMesh::build(&rows, &params);
        let values = mesh.mesh_matrix();
        let (last_row, last_column) = (values.len() - 1, values[0].len() - 1);

        // A grid point answers its own value: the mesh is uniform, so the
        // corners and the middle sit exactly on grid positions.
        assert_eq!(mesh.calc_z(0., 0.), values[0][0]);
        assert_eq!(mesh.calc_z(50., 50.), values[2][2]);
        assert_eq!(mesh.calc_z(100., 100.), values[last_row][last_column]);
        // Halfway between columns 0 and 1 (the grid steps 25mm), the answer is
        // the mix of the four values around it.
        let mixed = (values[0][0] + values[0][1] + values[1][0] + values[1][1]) / 4.;
        assert!((mesh.calc_z(12.5, 12.5) - mixed).abs() < 1e-12);
        // Outside the mesh the edge stands in, on the low and the high side.
        assert_eq!(mesh.calc_z(-50., -50.), values[0][0]);
        assert_eq!(mesh.calc_z(500., 500.), values[last_row][last_column]);
        for (coord, axis) in [
            (-50., MeshAxis::X),
            (500., MeshAxis::X),
            (-50., MeshAxis::Y),
            (500., MeshAxis::Y),
        ] {
            let (t, segment) = mesh.linear_index(coord, axis);
            assert!((0. ..=1.).contains(&t), "{coord} gives t = {t}");
            assert!(segment < values.len() - 1);
        }
    }

    /// `get_mesh_matrix` reports every value rounded to 6 decimals, which is
    /// the grid a client reads as `mesh_matrix` (`bed_mesh.py:1360-1364`).
    #[test]
    fn the_reported_mesh_is_rounded_to_six_decimals() {
        let read = options(&[
            ("mesh_min", "0,0"),
            ("mesh_max", "20,20"),
            ("probe_count", "3,3"),
            ("mesh_pps", "0"),
        ]);
        let (params, _) = probed_grid(&read, surface);
        let rows = vec![vec![0.123456789, -2.0000004, 1.234567891234]; 3];
        let mesh = ZMesh::build(&rows, &params);
        let reported = mesh.get_mesh_matrix();

        assert_eq!(reported.len(), 3);
        for (value, expected) in reported[0].iter().zip([0.123457, -2.0, 1.234568]) {
            assert!((value - expected).abs() < 1e-12, "{value} != {expected}");
        }
    }

    /// `algorithm` is `lagrange` or `bicubic` — upstream's `ALGOS`, which does
    /// not include the `direct` its verification substitutes
    /// (`bed_mesh.py:325`, `bed_mesh.py:421-424`) — read raw, stripped and
    /// lowercased like upstream reads it (`bed_mesh.py:412-413`).
    #[test]
    fn an_unknown_algorithm_is_refused() {
        let error = |value: &str| {
            BedMeshOptions::read(&ConfigWrapper::untracked(&section(&[
                ("mesh_min", "10,10"),
                ("mesh_max", "180,180"),
                ("algorithm", value),
            ])))
            .unwrap_err()
            .to_string()
        };

        assert_eq!(error("bilinear"), "bed_mesh: Unknown algorithm <bilinear>");
        assert_eq!(error("direct"), "bed_mesh: Unknown algorithm <direct>");

        // A name is stripped and lowercased, as upstream reads the option
        // (`bed_mesh.py:412-413`); the probe count keeps `bicubic` from falling
        // back to `lagrange`.
        let read = options(&[
            ("mesh_min", "10,10"),
            ("mesh_max", "180,180"),
            ("probe_count", "5,5"),
            ("algorithm", " BiCubic "),
        ]);
        assert_eq!(read.algorithm, "bicubic");
    }

    /// The probe counts each interpolator can carry (`_verify_algorithm`,
    /// `bed_mesh.py:429-455`): `lagrange` refuses more than 6 points per axis
    /// (the polynomial oscillates), `bicubic` needs 4 per axis and falls back
    /// to `lagrange` when the grid is merely small, and a grid that is short on
    /// one axis while long on the other is refused outright.
    #[test]
    fn the_probe_counts_are_checked_against_the_algorithm() {
        let error = |extra: &[(&str, &str)]| {
            let mut written = vec![("mesh_min", "10,10"), ("mesh_max", "180,180")];
            written.extend_from_slice(extra);
            BedMeshOptions::read(&ConfigWrapper::untracked(&section(&written)))
                .unwrap_err()
                .to_string()
        };

        assert_eq!(
            error(&[("probe_count", "7,7")]),
            "bed_mesh: cannot exceed a probe_count of 6 when using lagrange \
             interpolation. Configured Probe Count: 7, 7"
        );
        assert_eq!(
            error(&[("probe_count", "3,7"), ("algorithm", "bicubic")]),
            "bed_mesh: invalid probe_count option when using bicubic interpolation.  \
             Combination of 3 points on one axis with more than 6 on another is not \
             permitted. Configured Probe Count: 3, 7"
        );

        // 6 points per axis is the most lagrange takes …
        assert_eq!(
            options(&[
                ("mesh_min", "0,0"),
                ("mesh_max", "200,200"),
                ("probe_count", "6,6"),
            ])
            .algorithm,
            "lagrange"
        );
        // … and a small grid with `bicubic` falls back to it rather than fail
        // (`bed_mesh.py:449-455`).
        assert_eq!(
            options(&[
                ("mesh_min", "0,0"),
                ("mesh_max", "200,200"),
                ("probe_count", "3,3"),
                ("algorithm", "bicubic"),
            ])
            .algorithm,
            "lagrange"
        );
    }

    /// A round bed counts from `round_probe_count`, which the config bounds at
    /// 3 and requires to be odd (`bed_mesh.py:386-390`): the row across the
    /// diameter needs a middle point.
    #[test]
    fn a_round_bed_refuses_a_small_or_even_probe_count() {
        let error = |round_probe_count: &str| {
            BedMeshOptions::read(&ConfigWrapper::untracked(&section(&[
                ("mesh_radius", "50"),
                ("round_probe_count", round_probe_count),
            ])))
            .unwrap_err()
            .to_string()
        };

        assert_eq!(
            error("4"),
            "bed_mesh: probe_count must be odd for round beds"
        );
        assert_eq!(
            error("2"),
            "Option 'round_probe_count' in section 'bed_mesh' must have minimum of 3"
        );
    }

    /// A round bed only probes the points inside its radius, so the rows it
    /// stores are shorter than `x_count`; upstream repeats each row's outermost
    /// values on both sides to square the grid up
    /// (`bed_mesh.py:749-771`), and refuses a row holding an even number of
    /// values (`bed_mesh.py:753-759`).
    #[test]
    fn a_round_bed_row_is_padded_out_to_the_full_grid() {
        let rows = vec![
            vec![3.0],
            vec![0.1, 0.2, 0.3],
            vec![0.1, 0.2, 0.3, 0.4, 0.5],
        ];

        assert_eq!(
            pad_round_rows(rows, 5).unwrap(),
            vec![
                vec![3.0; 5],
                vec![0.1, 0.1, 0.2, 0.3, 0.3],
                vec![0.1, 0.2, 0.3, 0.4, 0.5],
            ]
        );
        assert_eq!(
            pad_round_rows(vec![vec![0.1, 0.2]], 5)
                .unwrap_err()
                .to_string(),
            "bed_mesh: incorrect number of points sampled on X"
        );
    }

    /// A round bed's calibration therefore meshes from a square grid: the
    /// points outside the radius repeat the value at the rim, and the mesh is
    /// as square as a rectangular bed's.
    #[test]
    fn a_round_bed_meshes_from_padded_rows() {
        let read = options(&[
            ("mesh_radius", "40"),
            ("mesh_origin", "0,0"),
            ("round_probe_count", "5"),
        ]);
        let points = generate_points(&read).unwrap();
        let values: Vec<f64> = points.iter().map(|&(x, y)| surface(x, y)).collect();
        let rows = pad_round_rows(rows_by_y(&points, &values), 5).unwrap();
        let params = MeshParams::from_options(&read, &points);
        let mesh = ZMesh::build(&rows, &params);

        // Five probe points, 2 interpolated per segment: 13×13, and every row
        // a full one — the outermost rows sit at the rim and are flat.
        assert_eq!(rows.iter().map(Vec::len).collect::<Vec<_>>(), vec![5; 5]);
        assert!(rows[0].iter().all(|z| *z == rows[0][0]));
        assert_eq!(mesh.mesh_matrix().len(), 13);
        assert!(mesh.mesh_matrix().iter().all(|row| row.len() == 13));
        // The mesh still holds the probed values at the probed points.
        assert_eq!(mesh.mesh_matrix()[6][6], rows[2][2]);
    }
}
