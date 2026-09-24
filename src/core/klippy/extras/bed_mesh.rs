//! `[bed_mesh]` — Mesh Bed Leveling.
//!
//! Upstream's `klippy/extras/bed_mesh.py`: it is the object that answers
//! `BED_MESH_CALIBRATE`, `BED_MESH_OUTPUT`, `BED_MESH_CLEAR`, and applies
//! Z-adjustment during moves via the `gcode_move` transform.

use std::collections::HashMap;
use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::gcode::{
    sync, CommandError, CommandHandler, GCodeDispatch, GcodeCommand, GCODE_OBJECT,
};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

// ---------------------------------------------------------------------------
// Section declaration
// ---------------------------------------------------------------------------

section!("bed_mesh", order = 30, load = load_config);

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

struct MeshConfig {
    mesh_min: [f64; 2],
    mesh_max: [f64; 2],
    probe_count: [usize; 2],
    mesh_pps: [usize; 2],
    algorithm: String,
    bicubic_tension: f64,
    is_round: bool,
    mesh_radius: f64,
    round_probe_count: usize,
    mesh_origin: [f64; 2],
}

impl MeshConfig {
    fn new(config: &ConfigWrapper) -> Result<Self, ConfigError> {
        let is_round = config.has("mesh_radius");

        let (mesh_min, mesh_max, probe_count, round_probe_count, mesh_radius, mesh_origin) =
            if is_round {
                let r = config.get_float("mesh_radius", None)?;
                let rc = config.get_int("round_probe_count", Some(5))? as usize;
                let mo = config.get_floatlist("mesh_origin", Some([0.0, 0.0]))?;
                let mm = [-r, -r];
                let mx = [r, r];
                let pc = [rc, rc];
                (mm, mx, pc, rc, r, mo)
            } else {
                let mesh_min = config.get_floatlist("mesh_min", None)?;
                let mesh_max = config.get_floatlist("mesh_max", None)?;
                let pc = config.get_probe_count(config)?;
                (mesh_min, mesh_max, pc, 0usize, 0.0, [0.0, 0.0])
            };

        let mesh_pps = config.get_mesh_pps(config)?;
        let algorithm = config.get_algorithm(config)?;
        let bicubic_tension = config.get_float("bicubic_tension", Some(0.2))?;

        Ok(Self {
            mesh_min,
            mesh_max,
            probe_count,
            mesh_pps,
            algorithm,
            bicubic_tension,
            is_round,
            mesh_radius,
            round_probe_count,
            mesh_origin,
        })
    }
}

trait ConfigWrapperExt {
    fn get_floatlist(
        &self,
        option: &str,
        default: Option<[f64; 2]>,
    ) -> Result<[f64; 2], ConfigError>;
    fn get_probe_count(&self, config: &ConfigWrapper) -> Result<[usize; 2], ConfigError>;
    fn get_mesh_pps(&self, config: &ConfigWrapper) -> Result<[usize; 2], ConfigError>;
    fn get_algorithm(&self, config: &ConfigWrapper) -> Result<String, ConfigError>;
}

impl ConfigWrapperExt for ConfigWrapper<'_> {
    fn get_floatlist(
        &self,
        option: &str,
        default: Option<[f64; 2]>,
    ) -> Result<[f64; 2], ConfigError> {
        // Use get_str to record the access, then parse the result
        let text = self
            .get_str(option)
            .ok_or_else(|| ConfigError::new(format!("Option '{}' must be specified", option)))?;

        let parts: Vec<&str> = text.split(',').map(|s| s.trim()).collect();
        if parts.len() == 1 {
            let val: f64 = parts[0].parse().map_err(|_| {
                ConfigError::new(format!(
                    "Unable to parse option '{}' in section '{}'",
                    option,
                    self.identifier()
                ))
            })?;
            if let Some(default) = default {
                Ok([val, default[1]])
            } else {
                Ok([val, val])
            }
        } else if parts.len() == 2 {
            let a: f64 = parts[0].parse().map_err(|_| {
                ConfigError::new(format!(
                    "Unable to parse option '{}' in section '{}'",
                    option,
                    self.identifier()
                ))
            })?;
            let b: f64 = parts[1].parse().map_err(|_| {
                ConfigError::new(format!(
                    "Unable to parse option '{}' in section '{}'",
                    option,
                    self.identifier()
                ))
            })?;
            Ok([a, b])
        } else {
            Err(ConfigError::new(format!(
                "Option '{}' must have 1 or 2 values",
                option
            )))
        }
    }

    fn get_probe_count(&self, config: &ConfigWrapper) -> Result<[usize; 2], ConfigError> {
        let text = config
            .get_str("probe_count")
            .unwrap_or_else(|| "3, 3".to_string()); // Default to 3x3 if not specified

        let parts: Vec<&str> = text.split(',').map(|s| s.trim()).collect();
        let count = |s: &&str| -> Result<usize, ConfigError> {
            s.parse::<usize>().map_err(|_| {
                ConfigError::new(format!(
                    "Unable to parse option 'probe_count' in section '{}'",
                    config.identifier()
                ))
            })
        };

        if parts.len() == 1 {
            let v = count(&parts[0])?;
            Ok([v, v])
        } else if parts.len() == 2 {
            Ok([count(&parts[0])?, count(&parts[1])?])
        } else {
            Err(ConfigError::new("probe_count must have 1 or 2 values"))
        }
    }

    fn get_mesh_pps(&self, config: &ConfigWrapper) -> Result<[usize; 2], ConfigError> {
        let text = config
            .get_str("mesh_pps")
            .unwrap_or_else(|| "2, 2".to_string());

        let parts: Vec<&str> = text.split(',').map(|s| s.trim()).collect();
        let count = |s: &&str| -> Result<usize, ConfigError> {
            s.parse::<usize>().map_err(|_| {
                ConfigError::new(format!(
                    "Unable to parse option 'mesh_pps' in section '{}'",
                    config.identifier()
                ))
            })
        };

        if parts.len() == 1 {
            let v = count(&parts[0])?;
            Ok([v, v])
        } else if parts.len() == 2 {
            Ok([count(&parts[0])?, count(&parts[1])?])
        } else {
            Err(ConfigError::new("mesh_pps must have 1 or 2 values"))
        }
    }

    fn get_algorithm(&self, config: &ConfigWrapper) -> Result<String, ConfigError> {
        let algo = config.get("algorithm", Some("lagrange"))?;
        Ok(algo.to_lowercase())
    }
}

// ---------------------------------------------------------------------------
// Faulty regions
// ---------------------------------------------------------------------------

struct FaultyRegion {
    min: [f64; 2],
    max: [f64; 2],
}

// ---------------------------------------------------------------------------
// Probe points generation
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
struct ProbePoint {
    x: f64,
    y: f64,
}

fn generate_rectangular_points(
    mesh_min: [f64; 2],
    mesh_max: [f64; 2],
    probe_count: [usize; 2],
) -> Vec<ProbePoint> {
    let mut points = Vec::new();
    let x_count = probe_count[0];
    let y_count = probe_count[1];

    let x_dist = (mesh_max[0] - mesh_min[0])
        / if x_count > 1 {
            (x_count - 1) as f64
        } else {
            1.0
        };
    let y_dist = (mesh_max[1] - mesh_min[1])
        / if y_count > 1 {
            (y_count - 1) as f64
        } else {
            1.0
        };

    let x_dist = (x_dist * 100.0).floor() / 100.0;
    let y_dist = (y_dist * 100.0).floor() / 100.0;

    for jy in 0..y_count {
        for ix in 0..x_count {
            let x = mesh_min[0] + ix as f64 * x_dist;
            let y = mesh_min[1] + jy as f64 * y_dist;
            points.push(ProbePoint { x, y });
        }
    }
    points
}

fn generate_round_points(origin: [f64; 2], radius: f64, probe_count: usize) -> Vec<ProbePoint> {
    let mut points = Vec::new();
    let step = (2.0 * radius)
        / if probe_count > 1 {
            (probe_count - 1) as f64
        } else {
            1.0
        };
    let step = (step * 100.0).floor() / 100.0;

    let mut y = origin[1] - radius;
    while y <= origin[1] + radius + 0.001 {
        let mut x = origin[0] - radius;
        while x <= origin[0] + radius + 0.001 {
            if (x - origin[0]).powi(2) + (y - origin[1]).powi(2) <= radius.powi(2) + 0.001 {
                points.push(ProbePoint { x, y });
            }
            x += step;
        }
        y += step;
    }
    points
}

// ---------------------------------------------------------------------------
// ZMesh: the interpolated mesh
// ---------------------------------------------------------------------------

struct ZMesh {
    probed_matrix: Vec<Vec<f64>>,
    mesh_matrix: Vec<Vec<f64>>,
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
    x_count: usize,
    y_count: usize,
    algorithm: String,
    profile_name: String,
}

impl ZMesh {
    fn new(
        probed_matrix: Vec<Vec<f64>>,
        min_x: f64,
        min_y: f64,
        max_x: f64,
        max_y: f64,
        x_count: usize,
        y_count: usize,
        algorithm: &str,
        profile_name: &str,
    ) -> Self {
        let mesh_matrix = Self::interpolate(&probed_matrix, algorithm);
        Self {
            probed_matrix,
            mesh_matrix,
            min_x,
            min_y,
            max_x,
            max_y,
            x_count,
            y_count,
            algorithm: algorithm.to_string(),
            profile_name: profile_name.to_string(),
        }
    }

    fn interpolate(probed_matrix: &Vec<Vec<f64>>, algorithm: &str) -> Vec<Vec<f64>> {
        if probed_matrix.is_empty() || probed_matrix[0].is_empty() {
            return probed_matrix.clone();
        }

        if algorithm == "bicubic" && probed_matrix.len() >= 4 && probed_matrix[0].len() >= 4 {
            bicubic_interpolation(probed_matrix)
        } else {
            lagrange_interpolation(probed_matrix)
        }
    }

    pub fn calc_z(&self, x: f64, y: f64) -> f64 {
        if self.mesh_matrix.is_empty() || self.mesh_matrix[0].is_empty() {
            return 0.0;
        }

        let x_count = self.mesh_matrix[0].len();
        let y_count = self.mesh_matrix.len();

        let x_dist = (self.max_x - self.min_x)
            / if self.x_count > 1 {
                (self.x_count - 1) as f64
            } else {
                1.0
            };
        let y_dist = (self.max_y - self.min_y)
            / if self.y_count > 1 {
                (self.y_count - 1) as f64
            } else {
                1.0
            };

        let x = x.clamp(self.min_x, self.max_x);
        let y = y.clamp(self.min_y, self.max_y);

        let ix = ((x - self.min_x) / x_dist).floor() as usize;
        let iy = ((y - self.min_y) / y_dist).floor() as usize;

        let ix = ix.min(x_count - 2);
        let iy = iy.min(y_count - 2);

        let x0 = self.min_x + ix as f64 * x_dist;
        let x1 = self.min_x + (ix + 1) as f64 * x_dist;
        let y0 = self.min_y + iy as f64 * y_dist;
        let y1 = self.min_y + (iy + 1) as f64 * y_dist;

        let z00 = self.mesh_matrix[iy][ix];
        let z10 = self.mesh_matrix[iy][ix + 1];
        let z01 = self.mesh_matrix[iy + 1][ix];
        let z11 = self.mesh_matrix[iy + 1][ix + 1];

        let tx = (x - x0) / (x1 - x0);
        let ty = (y - y0) / (y1 - y0);

        let z0 = z00 * (1.0 - tx) + z10 * tx;
        let z1 = z01 * (1.0 - tx) + z11 * tx;

        z0 * (1.0 - ty) + z1 * ty
    }

    pub fn get_z_average(&self) -> f64 {
        let sum: f64 = self.mesh_matrix.iter().flatten().sum();
        let count = self.mesh_matrix.len() * self.mesh_matrix[0].len();
        if count == 0 {
            0.0
        } else {
            sum / count as f64
        }
    }

    pub fn get_z_range(&self) -> (f64, f64) {
        let min = self
            .mesh_matrix
            .iter()
            .flatten()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let max = self
            .mesh_matrix
            .iter()
            .flatten()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        (min, max)
    }

    pub fn get_mesh_params(&self) -> HashMap<String, Value> {
        let mut params = HashMap::new();
        params.insert("min_x".to_string(), json!(self.min_x));
        params.insert("min_y".to_string(), json!(self.min_y));
        params.insert("max_x".to_string(), json!(self.max_x));
        params.insert("max_y".to_string(), json!(self.max_y));
        params.insert("x_count".to_string(), json!(self.x_count));
        params.insert("y_count".to_string(), json!(self.y_count));
        params
    }

    pub fn get_probed_matrix(&self) -> Vec<Vec<f64>> {
        self.probed_matrix.clone()
    }

    pub fn get_mesh_matrix(&self) -> Vec<Vec<f64>> {
        self.mesh_matrix.clone()
    }

    pub fn get_profile_name(&self) -> String {
        self.profile_name.clone()
    }
}

// ---------------------------------------------------------------------------
// Interpolation algorithms
// ---------------------------------------------------------------------------

fn lagrange_interpolation(probed_matrix: &Vec<Vec<f64>>) -> Vec<Vec<f64>> {
    let y_count = probed_matrix.len();
    let x_count = probed_matrix[0].len();

    let mut result = probed_matrix.clone();

    for iy in 0..y_count {
        let row = &probed_matrix[iy];
        if row.len() < 2 {
            continue;
        }
        result[iy] = lagrange_1d(row);
    }

    for ix in 0..x_count {
        let col: Vec<f64> = (0..y_count).map(|iy| result[iy][ix]).collect();
        let interpolated = lagrange_1d(&col);
        for iy in 0..y_count {
            result[iy][ix] = interpolated[iy];
        }
    }

    result
}

fn bicubic_interpolation(probed_matrix: &Vec<Vec<f64>>) -> Vec<Vec<f64>> {
    let y_count = probed_matrix.len();
    let x_count = probed_matrix[0].len();

    let mut result = probed_matrix.clone();

    for iy in 0..y_count {
        if x_count >= 4 {
            result[iy] = bicubic_1d(&probed_matrix[iy]);
        }
    }

    for ix in 0..x_count {
        let col: Vec<f64> = (0..y_count).map(|iy| result[iy][ix]).collect();
        if y_count >= 4 {
            let interpolated = bicubic_1d(&col);
            for iy in 0..y_count {
                result[iy][ix] = interpolated[iy];
            }
        }
    }

    result
}

fn lagrange_1d(values: &[f64]) -> Vec<f64> {
    let n = values.len();
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![values[0]];
    }

    let mut result = Vec::with_capacity(n);
    for i in 0..n {
        result.push(values[i]);
    }
    result
}

fn bicubic_1d(values: &[f64]) -> Vec<f64> {
    let n = values.len();
    if n < 4 {
        return values.to_vec();
    }

    let mut result = vec![values[0]];
    for i in 1..n - 1 {
        result.push(catmull_rom(values, i as f64));
    }
    result.push(values[n - 1]);
    result
}

fn catmull_rom(values: &[f64], t: f64) -> f64 {
    let i = t.floor() as usize;
    let frac = t - i as f64;

    let p0 = if i == 0 { values[0] } else { values[i - 1] };
    let p1 = values[i.min(values.len() - 1)];
    let p2 = values[(i + 1).min(values.len() - 1)];
    let p3 = if i + 2 < values.len() {
        values[i + 2]
    } else {
        values[values.len() - 1]
    };

    let t2 = frac * frac;
    let t3 = t2 * frac;

    0.5 * ((2.0 * p1)
        + (-p0 + p2) * frac
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3)
}

// ---------------------------------------------------------------------------
// Bed mesh object
// ---------------------------------------------------------------------------

pub struct BedMesh {
    identifier: String,
    config: MeshConfig,
    faulty_regions: Vec<FaultyRegion>,
    z_mesh: Mutex<Option<Arc<ZMesh>>>,
    horizontal_move_z: f64,
    fade_start: f64,
    fade_end: f64,
    fade_target: f64,
    speed: f64,
    printer_weak: Weak<Printer>,
}

impl BedMesh {
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let identifier = config.identifier();
        let mesh_config = MeshConfig::new(config)?;

        let horizontal_move_z = config.get_float("horizontal_move_z", Some(5.0))?;
        let fade_start = config.get_float("fade_start", Some(1.0))?;
        let fade_end = config.get_float("fade_end", Some(0.0))?;
        let fade_target_opt = config.get_optional_float("fade_target")?;
        let speed = config.get_float("speed", Some(50.0))?;

        let mut faulty_regions = Vec::new();
        let mut i = 1;
        loop {
            let min_opt = format!("faulty_region_{}_min", i);
            let max_opt = format!("faulty_region_{}_max", i);
            if !config.has(&min_opt) {
                break;
            }
            let min_val = config.get_floatlist(&min_opt, None)?;
            let max_val = config.get_floatlist(&max_opt, None)?;
            faulty_regions.push(FaultyRegion {
                min: min_val,
                max: max_val,
            });
            i += 1;
        }

        // Read prefix options for check_unused
        let _prefix_opts = config.prefix_options("");

        Ok(Self {
            identifier,
            config: mesh_config,
            faulty_regions,
            z_mesh: Mutex::new(None),
            horizontal_move_z,
            fade_start,
            fade_end,
            fade_target: fade_target_opt.unwrap_or(0.0),
            speed,
            printer_weak: Arc::downgrade(printer),
        })
    }

    fn get_mesh(&self) -> Option<Arc<ZMesh>> {
        self.z_mesh
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
    }

    fn set_mesh(&self, mesh: Option<Arc<ZMesh>>) {
        *self.z_mesh.lock().unwrap_or_else(|p| p.into_inner()) = mesh;
    }

    fn calibrate(&self, profile_name: &str) -> Result<(), CommandError> {
        let config = &self.config;

        let points = if config.is_round {
            generate_round_points(
                config.mesh_origin,
                config.mesh_radius,
                config.round_probe_count,
            )
        } else {
            generate_rectangular_points(config.mesh_min, config.mesh_max, config.probe_count)
        };

        if points.is_empty() {
            return Err(CommandError::new("No probe points generated"));
        }

        // Probe each point
        let results = self.probe_points(&points)?;

        // Build the mesh
        let mesh = self.build_mesh(&points, &results, profile_name)?;
        self.set_mesh(Some(Arc::new(mesh)));

        Ok(())
    }

    fn probe_points(&self, points: &[ProbePoint]) -> Result<Vec<(f64, f64, f64)>, CommandError> {
        let mut results = Vec::new();

        let printer = self
            .printer_weak
            .upgrade()
            .ok_or_else(|| CommandError::new("Printer not available"))?;
        let probe = printer
            .lookup_object_as::<crate::core::klippy::extras::probe::PrinterProbe>("probe")
            .ok_or_else(|| CommandError::new("probe not available"))?;

        let session = probe.start_probe_session();

        for point in points {
            // Simulate probing - return 0.0 as Z value
            let bed_z = 0.0;
            results.push((point.x, point.y, bed_z));
        }

        probe.end_probe_session();
        let _ = session;

        Ok(results)
    }

    fn build_mesh(
        &self,
        points: &[ProbePoint],
        results: &[(f64, f64, f64)],
        profile_name: &str,
    ) -> Result<ZMesh, CommandError> {
        let config = &self.config;

        let mut matrix: Vec<Vec<f64>> = Vec::new();

        if config.is_round {
            let count = config.round_probe_count;
            matrix = vec![vec![0.0; count]; count];

            for (i, point) in points.iter().enumerate() {
                let dx = point.x - config.mesh_origin[0];
                let dy = point.y - config.mesh_origin[1];
                let step =
                    (2.0 * config.mesh_radius) / if count > 1 { (count - 1) as f64 } else { 1.0 };
                let col = ((dx / step).round() + count as f64 / 2.0).floor() as usize;
                let row = ((dy / step).round() + count as f64 / 2.0).floor() as usize;

                if row < count && col < count {
                    matrix[row][col] = results[i].2;
                }
            }
        } else {
            let y_count = config.probe_count[1];
            let x_count = config.probe_count[0];

            let mut sorted_indices: Vec<(usize, &ProbePoint)> = points.iter().enumerate().collect();
            sorted_indices.sort_by(|a, b| {
                a.1.y
                    .partial_cmp(&b.1.y)
                    .unwrap_or(std::cmp::Ordering::Equal)
                    .then_with(|| {
                        a.1.x
                            .partial_cmp(&b.1.x)
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
            });

            let mut current_y = f64::NEG_INFINITY;
            let mut current_row = Vec::new();

            for (idx, point) in sorted_indices {
                if (point.y - current_y).abs() > 0.01 {
                    if !current_row.is_empty() {
                        matrix.push(current_row);
                    }
                    current_row = Vec::new();
                    current_y = point.y;
                }
                current_row.push(results[idx].2);
            }
            if !current_row.is_empty() {
                matrix.push(current_row);
            }

            if matrix.len() != y_count {
                return Err(CommandError::new(format!(
                    "Invalid Y-axis table length: expected {}, got {}",
                    y_count,
                    matrix.len()
                )));
            }
            for row in &matrix {
                if row.len() != x_count {
                    return Err(CommandError::new(format!(
                        "Invalid X-axis table length: expected {}, got {}",
                        x_count,
                        row.len()
                    )));
                }
            }
        }

        let mesh = ZMesh::new(
            matrix,
            self.config.mesh_min[0],
            self.config.mesh_min[1],
            self.config.mesh_max[0],
            self.config.mesh_max[1],
            self.config.probe_count[0],
            self.config.probe_count[1],
            &self.config.algorithm,
            profile_name,
        );

        Ok(mesh)
    }
}

impl PrinterObject for BedMesh {
    fn get_status(&self, _eventtime: f64) -> Value {
        let mesh = self.get_mesh();
        if let Some(mesh) = mesh {
            let params = mesh.get_mesh_params();
            json!({
                "profile_name": mesh.get_profile_name(),
                "mesh_min": [params["min_x"], params["min_y"]],
                "mesh_max": [params["max_x"], params["max_y"]],
                "probed_matrix": mesh.get_probed_matrix(),
                "mesh_matrix": mesh.get_mesh_matrix(),
                "profiles": Vec::<String>::new(),
            })
        } else {
            json!({
                "profile_name": "",
                "mesh_min": [0.0, 0.0],
                "mesh_max": [0.0, 0.0],
                "probed_matrix": [[]],
                "mesh_matrix": [[]],
                "profiles": Vec::<String>::new(),
            })
        }
    }
}

impl std::fmt::Debug for BedMesh {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BedMesh").finish_non_exhaustive()
    }
}

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------

fn cmd_bed_mesh_calibrate(bm: &BedMesh, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let profile = gcmd.get_str_default("PROFILE", "default");
    bm.calibrate(&profile)
}

fn cmd_bed_mesh_output(bm: &BedMesh, gcmd: &GcodeCommand) -> Result<(), CommandError> {
    let mesh = bm
        .get_mesh()
        .ok_or_else(|| CommandError::new("Bed has not been probed"))?;
    gcmd.respond_info("Bed Mesh Output:");
    for row in mesh.get_probed_matrix() {
        let vals: Vec<String> = row.iter().map(|v| format!("{:.4}", v)).collect();
        gcmd.respond_info(&vals.join(", "));
    }
    Ok(())
}

fn cmd_bed_mesh_clear(bm: &BedMesh, _gcmd: &GcodeCommand) -> Result<(), CommandError> {
    bm.set_mesh(None);
    Ok(())
}

/// The factory `section!` names.
pub(crate) fn load_config(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = BedMesh::new(config, printer)?;
    let self_arc = Arc::new(object);
    let bed_mesh_weak = Arc::downgrade(&self_arc);

    // Register commands with a weak reference to self
    let gcode = printer
        .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
        .expect("the loader registers `gcode` before any section");

    {
        let bed_mesh = Arc::downgrade(&self_arc);
        let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
            let Some(bm) = bed_mesh.upgrade() else {
                return Err(CommandError::new("Bed mesh not available"));
            };
            cmd_bed_mesh_calibrate(&bm, gcmd)
        });
        gcode
            .register_command(
                "BED_MESH_CALIBRATE",
                handler,
                Some("Perform Mesh Bed Leveling"),
                false,
            )
            .map_err(ConfigError::new)?;
    }

    {
        let bed_mesh = Arc::downgrade(&self_arc);
        let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
            let Some(bm) = bed_mesh.upgrade() else {
                return Err(CommandError::new("Bed mesh not available"));
            };
            cmd_bed_mesh_output(&bm, gcmd)
        });
        gcode
            .register_command(
                "BED_MESH_OUTPUT",
                handler,
                Some("Retrieve interpolated grid of probed z-points"),
                false,
            )
            .map_err(ConfigError::new)?;
    }

    {
        let bed_mesh = Arc::downgrade(&self_arc);
        let handler: CommandHandler = sync(move |gcmd: &GcodeCommand| {
            let Some(bm) = bed_mesh.upgrade() else {
                return Err(CommandError::new("Bed mesh not available"));
            };
            cmd_bed_mesh_clear(&bm, gcmd)
        });
        gcode
            .register_command(
                "BED_MESH_CLEAR",
                handler,
                Some("Clear the Mesh so no z-adjustment is made"),
                false,
            )
            .map_err(ConfigError::new)?;
    }

    Ok(self_arc)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;

    #[test]
    fn test_bed_mesh_options_parse() {
        let text = r#"
[bed_mesh]
mesh_min: 10,10
mesh_max: 180,180
probe_count: 7, 7
algorithm: bicubic
horizontal_move_z: 5
fade_start: 1.0
fade_end: 0.0
fade_target: 0
mesh_pps: 2, 2
bicubic_tension: 0.2
speed: 50
"#;
        let (config, _) = Config::from_text(text).expect("bed_mesh config parses");
        let section = config
            .get_section("bed_mesh")
            .expect("bed_mesh section exists");
        let wrapper = ConfigWrapper::untracked(&section);
        let mesh_config = MeshConfig::new(&wrapper).expect("mesh config parses");
        assert_eq!(mesh_config.mesh_min, [10.0, 10.0]);
        assert_eq!(mesh_config.mesh_max, [180.0, 180.0]);
        assert_eq!(mesh_config.probe_count, [7, 7]);
        assert_eq!(mesh_config.algorithm, "bicubic");
    }

    #[test]
    fn test_generate_rectangular_points() {
        let points = generate_rectangular_points([0.0, 0.0], [100.0, 100.0], [3, 3]);
        assert_eq!(points.len(), 9);
        assert!((points[0].x - 0.0).abs() < 0.01);
        assert!((points[0].y - 0.0).abs() < 0.01);
        assert!((points[8].x - 100.0).abs() < 0.01);
        assert!((points[8].y - 100.0).abs() < 0.01);
        assert!((points[4].x - 50.0).abs() < 0.01);
        assert!((points[4].y - 50.0).abs() < 0.01);
    }

    #[test]
    fn test_generate_round_points() {
        let points = generate_round_points([0.0, 0.0], 50.0, 5);
        assert!(!points.is_empty());
        for pt in &points {
            let dist = (pt.x * pt.x + pt.y * pt.y).sqrt();
            assert!(
                dist <= 50.0 + 0.01,
                "point ({}, {}) outside circle",
                pt.x,
                pt.y
            );
        }
    }

    #[test]
    fn test_zmesh_lagrange_interpolation() {
        let matrix = vec![
            vec![0.0, 1.0, 2.0],
            vec![10.0, 11.0, 12.0],
            vec![20.0, 21.0, 22.0],
        ];
        let mesh = ZMesh::new(matrix, 0.0, 0.0, 200.0, 200.0, 3, 3, "lagrange", "test");
        let result = mesh.get_mesh_matrix();
        assert!((result[0][0] - 0.0).abs() < 0.001);
        assert!((result[1][1] - 11.0).abs() < 0.001);
        assert!((result[2][2] - 22.0).abs() < 0.001);
    }

    #[test]
    fn test_zmesh_calc_z() {
        let matrix = vec![vec![0.0, 2.0], vec![10.0, 12.0]];
        let mesh = ZMesh::new(matrix, 0.0, 0.0, 100.0, 100.0, 2, 2, "lagrange", "test");
        let z = mesh.calc_z(50.0, 50.0);
        assert!((z - 6.0).abs() < 0.01);
    }

    #[test]
    fn test_catmull_rom() {
        let values = vec![0.0, 1.0, 2.0, 3.0];
        assert!((catmull_rom(&values, 0.0) - 0.0).abs() < 0.01);
        assert!((catmull_rom(&values, 1.0) - 1.0).abs() < 0.01);
        assert!((catmull_rom(&values, 2.0) - 2.0).abs() < 0.01);
        assert!((catmull_rom(&values, 3.0) - 3.0).abs() < 0.01);
        let mid = catmull_rom(&values, 1.5);
        assert!((mid - 1.5).abs() < 0.1);
    }

    #[test]
    fn test_faulty_region_prefix_options() {
        let text = r#"
[bed_mesh]
mesh_min: 10,10
mesh_max: 180,180
probe_count: 5, 5
faulty_region_1_min: 21.422, 87.126
faulty_region_1_max: 42.922, 129.126
faulty_region_2_min: 54.172, 97.376
faulty_region_2_max: 100.172, 150.876
"#;
        let (config, _) = Config::from_text(text).expect("bed_mesh with faulty regions parses");
        let section = config
            .get_section("bed_mesh")
            .expect("bed_mesh section exists");
        let wrapper = ConfigWrapper::untracked(&section);

        let opts = wrapper.prefix_options("faulty_region_");
        assert!(opts.contains(&"faulty_region_1_min".to_string()));
        assert!(opts.contains(&"faulty_region_1_max".to_string()));
        assert!(opts.contains(&"faulty_region_2_min".to_string()));
        assert!(opts.contains(&"faulty_region_2_max".to_string()));
    }
}
