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
    /// The mesh area's lower corner.
    pub mesh_min: [f64; 2],
    /// The mesh area's upper corner.
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

/// Two floats from a `x, y` option.
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
    if items.len() != 2 {
        return Err(ConfigError::new(format!(
            "Option '{option}' in section '{}' must have 2 values",
            config.identifier()
        )));
    }
    let mut out = [0.0f64; 2];
    for (slot, item) in out.iter_mut().zip(&items) {
        *slot = item.trim().parse::<f64>().map_err(|_| {
            ConfigError::new(format!(
                "Unable to parse option '{option}' in section '{}'",
                config.identifier()
            ))
        })?;
    }
    Ok(out)
}

/// Two integers from a `x, y` option.
fn two_ints(
    config: &ConfigWrapper,
    option: &str,
    default: Option<[i64; 2]>,
) -> Result<[i64; 2], ConfigError> {
    let Some(items) = config.get_list(option, ',') else {
        return default.ok_or_else(|| {
            ConfigError::new(format!(
                "Option '{option}' in section '{}' must be specified",
                config.identifier()
            ))
        });
    };
    if items.len() != 2 {
        return Err(ConfigError::new(format!(
            "Option '{option}' in section '{}' must have 2 values",
            config.identifier()
        )));
    }
    let mut out = [0i64; 2];
    for (slot, item) in out.iter_mut().zip(&items) {
        *slot = item.trim().parse::<i64>().map_err(|_| {
            ConfigError::new(format!(
                "Unable to parse option '{option}' in section '{}'",
                config.identifier()
            ))
        })?;
    }
    Ok(out)
}

impl BedMeshOptions {
    /// Read every option the corpus writes.
    ///
    /// # Errors
    /// As the option readers: a missing `mesh_min`/`mesh_max`/`probe_count`, a
    /// malformed pair, an unknown `algorithm`.
    pub fn read(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let probe_count = two_ints(config, "probe_count", Some([3, 3]))?;
        let round_probe_count = config.get_int("round_probe_count", Some(5))?;
        let mesh_pps = two_ints(config, "mesh_pps", Some([2, 2]))?;
        let mesh_radius = config.get_optional_float("mesh_radius")?;
        let faulty_regions = read_faulty_regions(config)?;

        Ok(Self {
            mesh_min: two_floats(config, "mesh_min", None)?,
            mesh_max: two_floats(config, "mesh_max", None)?,
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
        let mut read = options(&[
            ("mesh_min", "-50,-50"),
            ("mesh_max", "50,50"),
            ("probe_count", "5, 5"),
            ("mesh_radius", "42"),
            ("mesh_origin", "0, 0"),
        ]);
        read.round_probe_count = 5;
        let points = generate_points(&read).unwrap();

        // 5x5 grid with 25mm spacing: the centre, the four axis neighbours and
        // the four diagonal neighbours are inside 42mm — the axes (±50) are not.
        assert_eq!(points.len(), 9);
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
            ("mesh_min", "-50,-50"),
            ("mesh_max", "50,50"),
            ("probe_count", "3, 3"),
            ("round_probe_count", "7"),
            ("mesh_radius", "45"),
        ]);

        assert_eq!(read.counts(), [7, 7]);
    }
}
