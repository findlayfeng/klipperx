//! `bed_mesh` — bed mesh calibration.
//!
//! Upstream `klippy/extras/bed_mesh.py` (1765 lines, mesh interpolation,
//! faulty-region substitution, adaptive meshes, rapid scan, fade and the move
//! splitter). This unit lands the part the upstream corpus exercises:
//!
//! - the `[bed_mesh]` section with **every** option the corpus writes (so the
//!   config loads), including the `faulty_region_<N>_min` / `_max` pairs;
//! - `BED_MESH_CALIBRATE`: generate the probe points, move to each (net of the
//!   probe offsets), probe it through the `probe` object's session, store the
//!   probed grid;
//! - `BED_MESH_CLEAR` and the `get_status` shape clients read.
//!
//! **Not implemented yet** (tracked in `TODO.md` H9, next unit):
//!
//! - the interpolation meshes (`LagrangeMesh` / `BicubicMesh`, `mesh_pps`) — the
//!   grid is stored as probed, and no interpolated `mesh_matrix` is produced;
//! - faulty-region substitution (`_process_faulty_regions`): the regions are
//!   parsed and probed like any other point;
//! - fade (`fade_start` / `fade_end` / `fade_target`) and the z-adjustment the
//!   mesh applies to moves (`MoveSplitter`) — so a calibration currently does
//!   **not** affect subsequent moves;
//! - the profile commands (`BED_MESH_PROFILE`, `BED_MESH_OUTPUT`, `BED_MESH_MAP`,
//!   `BED_MESH_OFFSET`) and the `bed_mesh/dump_mesh` endpoint.
//!
//! The load side is complete: every option below is claimed, which is what the
//! corpus needs, and the gaps above are capability gaps, not silent shortcuts —
//! they are listed in the manual and the task list.

use std::sync::{Arc, Mutex};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::probe::lookup_probe_session;
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
    /// `lagrange` or `bicubic` (accepted; the interpolation itself is the next
    /// unit's job).
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
/// (`configfile.py:87-107`): every value is parsed *before* the count is
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
    /// `probe_count`, an unknown `algorithm`.
    pub fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let mesh_radius = config.get_optional_float("mesh_radius")?;
        let round_probe_count = config.get_int("round_probe_count", Some(5))?;
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

        Ok(Self {
            mesh_min,
            mesh_max,
            probe_count,
            speed: config.get_float("speed", Some(50.0))?,
            horizontal_move_z: config.get_float("horizontal_move_z", Some(5.0))?,
            algorithm: config.get_choice(
                "algorithm",
                &["lagrange", "bicubic"],
                Some("lagrange"),
            )?,
            fade_start: config.get_float("fade_start", Some(1.0))?,
            fade_end: config.get_float("fade_end", Some(0.0))?,
            fade_target: config.get_optional_float("fade_target")?,
            mesh_pps,
            bicubic_tension: config.get_float("bicubic_tension", Some(0.2))?,
            round_probe_count,
            mesh_radius,
            mesh_origin: two_floats(config, "mesh_origin", Some([0.0, 0.0]))?,
            move_check_distance: config.get_float("move_check_distance", Some(5.0))?,
            faulty_regions,
        })
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

/// One configured `[bed_mesh]` (`bed_mesh.py:BedMesh` + `BedMeshCalibrate`).
pub struct BedMesh {
    /// The options as read (shared with the command handlers).
    options: Arc<Mutex<BedMeshOptions>>,
    /// The last calibration's probed Z values, row-major (shared likewise).
    mesh: Arc<Mutex<Option<Vec<f64>>>>,
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
                .register_command(
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

                            *mesh.lock().unwrap_or_else(|p| p.into_inner()) = Some(probed);
                            gcmd.respond_info(&format!(
                                "Mesh Bed Leveling Complete ({algorithm} mesh stored, {} points)",
                                points.len()
                            ));
                            Ok(())
                        })
                    }),
                    Some("Calibrate the bed mesh"),
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

    /// The last calibration's probed Z values, if any.
    pub fn probed_matrix(&self) -> Option<Vec<f64>> {
        self.mesh.lock().unwrap_or_else(|p| p.into_inner()).clone()
    }
}

impl PrinterObject for BedMesh {
    fn get_status(&self, _eventtime: f64) -> Value {
        let options = self.options.lock().unwrap_or_else(|p| p.into_inner());
        let probed = self.mesh.lock().unwrap_or_else(|p| p.into_inner()).clone();
        json!({
            "profile_name": "",
            "mesh_min": options.mesh_min,
            "mesh_max": options.mesh_max,
            "probed_matrix": probed.clone().unwrap_or_default(),
            // The interpolated matrix is the next unit's job; clients that
            // expect a mesh get the probed grid for now.
            "mesh_matrix": probed.unwrap_or_default(),
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
        let read = options(&[
            ("mesh_min", "10,10"),
            ("mesh_max", "180,180"),
            ("probe_count", "7, 7"),
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
        // 5mm over 6 gaps is 0.83mm < 1mm.
        let read = options(&[
            ("mesh_min", "0,0"),
            ("mesh_max", "5,5"),
            ("probe_count", "7, 7"),
        ]);
        let err = generate_points(&read).unwrap_err();

        assert_eq!(
            err.to_string(),
            "bed_mesh: min/max points too close together"
        );
    }

    #[test]
    fn a_round_mesh_uses_the_round_probe_count() {
        let read = options(&[
            ("probe_count", "3, 3"),
            ("round_probe_count", "7"),
            ("mesh_radius", "45"),
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
}
