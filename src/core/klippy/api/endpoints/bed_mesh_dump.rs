//! `bed_mesh/dump_mesh` — the loaded grid, the saved profiles, and on request
//! the calibration behind them.
//!
//! Upstream's `BedMesh._handle_dump_request`
//! (`klippy/extras/bed_mesh.py:291-310`): `current_mesh` is the loaded mesh
//! (empty while the bed holds none), `profiles` is what the profile manager has
//! saved, and `mesh_args` is the one parameter — passed, it adds `calibration`.
//!
//! ```json
//! {"id": 1, "method": "bed_mesh/dump_mesh", "params": {"mesh_args": {}}}
//! ```
//!
//! Three gaps between that upstream handler and this one are deliberate, and
//! each is a capability this unit does not have yet rather than a shortcut:
//!
//! * **`mesh_matrix` is the probed grid** — interpolation (`lagrange` /
//!   `bicubic`) is not written yet, so the grid moves would follow is the grid
//!   as probed, the same stand-in `bed_mesh`'s `get_status` makes;
//! * **`mesh_args` is accepted but not applied** — upstream feeds its keys back
//!   into the mesh configuration (`bed_mesh.py:631-633`); the per-command
//!   overrides that reads are not written yet, so `calibration` describes the
//!   configuration the printer is running;
//! * **`profiles` is always `{}`** — nothing is saved across a restart until
//!   `BED_MESH_PROFILE` lands, and upstream's `probe_path` / `rapid_path`
//!   (the probe scheduler's walk) are absent with it.
//!
//! A machine whose config has no `[bed_mesh]` section answers
//! `webhooks: No registered callback for path 'bed_mesh/dump_mesh'`: upstream
//! registers the path from the extras module, so the path exists exactly where
//! the section does (`bed_mesh.py:126-128`), and this is the text a client sees
//! there. Answering [`ApiError::Internal`] instead would take the whole printer
//! down over a request no client could know was doomed.
//!
//! # Status
//!
//! Written, tested and registered by [`register`](super::super::register).

use std::sync::Arc;

use serde_json::{json, Map, Value};

use crate::core::klippy::api::protocol::{ApiError, Request};
use crate::core::klippy::api::registry::{Endpoint, EndpointContext, EndpointFuture};
use crate::core::klippy::api::{Api, ApiWiring, RegistrationError};
use crate::core::klippy::extras::bed_mesh::{
    generate_points, BedMesh, BedMeshOptions, BED_MESH_OBJECT,
};
use crate::core::klippy::printer::Printer;

/// The path, and the name a machine without `[bed_mesh]` reports as missing.
const PATH: &str = "bed_mesh/dump_mesh";

endpoint!(install);

/// Install the `bed_mesh/dump_mesh` endpoint.
pub(crate) fn install(api: &mut Api, wiring: &ApiWiring<'_>) -> Result<(), RegistrationError> {
    api.register(BedMeshDump::new(Arc::clone(wiring.printer)))
        .map_err(RegistrationError::Endpoint)
}

/// The `bed_mesh/dump_mesh` endpoint.
pub struct BedMeshDump {
    printer: Arc<Printer>,
}

impl BedMeshDump {
    /// Build the endpoint over the machine it reads.
    pub fn new(printer: Arc<Printer>) -> Self {
        Self { printer }
    }
}

impl Endpoint for BedMeshDump {
    fn path(&self) -> &'static str {
        PATH
    }

    fn handle<'a>(
        &'a self,
        request: &'a Request,
        _context: &'a EndpointContext<'a>,
    ) -> EndpointFuture<'a> {
        Box::pin(async move {
            let with_calibration = mesh_args(request)?;
            let bed = self
                .printer
                .lookup_object_as::<BedMesh>(BED_MESH_OBJECT)
                .ok_or_else(|| ApiError::UnknownEndpoint(PATH.to_string()))?;

            let mut result = Map::new();
            result.insert("current_mesh".to_string(), current_mesh(&bed)?);
            // No profile manager yet: nothing has been saved (`bed_mesh.py:1665`).
            result.insert("profiles".to_string(), json!({}));
            if with_calibration {
                result.insert("calibration".to_string(), calibration(&bed)?);
            }
            Ok(Value::Object(result))
        })
    }
}

impl std::fmt::Debug for BedMeshDump {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("BedMeshDump").finish_non_exhaustive()
    }
}

// ===========================================================================
// Parameters
// ===========================================================================

/// Whether the request passed `mesh_args`, the endpoint's only parameter.
///
/// Upstream reads it as a dict with `{}` as the default and branches on the
/// dict's truthiness (`bed_mesh.py:304-307`); here *passing* it is what adds
/// `calibration`, and its keys are not applied yet (see the module docs).
///
/// # Errors
/// Returns [`ApiError::InvalidArgumentType`] when the key is present but not an
/// object — upstream's `get_dict` refuses anything but a dict
/// (`klippy/webhooks.py:89-90`).
fn mesh_args(request: &Request) -> Result<bool, ApiError> {
    match request.params().get_opt("mesh_args") {
        None => Ok(false),
        Some(Value::Object(_)) => Ok(true),
        Some(_) => Err(ApiError::InvalidArgumentType("mesh_args".to_string())),
    }
}

// ===========================================================================
// Response
// ===========================================================================

/// The `current_mesh` half: the loaded grid, or `{}` while there is none
/// (`bed_mesh.py:296-302`).
fn current_mesh(bed: &BedMesh) -> Result<Value, ApiError> {
    let Some(mesh) = bed.loaded_mesh() else {
        return Ok(json!({}));
    };
    let options = bed.options();
    let points = probe_points(&options)?;
    let probed = json!(mesh.rows);
    let interpolated = probed.clone();
    Ok(json!({
        "name": mesh.name,
        "probed_matrix": probed,
        // Upstream's is the interpolated grid; see the module docs.
        "mesh_matrix": interpolated,
        "mesh_params": mesh_params(&options, &points),
    }))
}

/// The `calibration` half: the probe points and the configuration they came
/// from (`bed_mesh.py:629-641`, minus the paths of a scheduler not written
/// yet).
fn calibration(bed: &BedMesh) -> Result<Value, ApiError> {
    let options = bed.options();
    let points = probe_points(&options)?;
    Ok(json!({
        "points": points,
        "config": mesh_config(&options),
    }))
}

/// The generated probe points.
///
/// # Errors
/// [`ApiError::CommandError`] when the configuration cannot produce a grid —
/// the same refusal `BED_MESH_CALIBRATE` would hit, answered without taking
/// the printer down.
fn probe_points(options: &BedMeshOptions) -> Result<Vec<(f64, f64)>, ApiError> {
    generate_points(options).map_err(|err| ApiError::CommandError(err.to_string()))
}

/// The `mesh_params` of a loaded grid: upstream's `mesh_config` plus the
/// bounds of the probed points (`bed_mesh.py:676-686`).
fn mesh_params(options: &BedMeshOptions, points: &[(f64, f64)]) -> Value {
    let [x_count, y_count] = options.counts();
    let (min_x, max_x, min_y, max_y) = bounds(points);
    json!({
        "min_x": min_x,
        "max_x": max_x,
        "min_y": min_y,
        "max_y": max_y,
        "x_count": x_count,
        "y_count": y_count,
        "mesh_x_pps": options.mesh_pps[0],
        "mesh_y_pps": options.mesh_pps[1],
        "algo": options.algorithm,
        "tension": options.bicubic_tension,
    })
}

/// The `calibration.config` half: upstream's `mesh_config` with the bounds
/// added, and the radius and origin a round bed derives them from
/// (`bed_mesh.py:629-641`). Both stay `null` on a rectangular bed, which
/// upstream never gives an origin or radius (`bed_mesh.py:383-397`).
fn mesh_config(options: &BedMeshOptions) -> Value {
    let [x_count, y_count] = options.counts();
    let origin = if options.mesh_radius.is_some() {
        json!(options.mesh_origin)
    } else {
        Value::Null
    };
    json!({
        "x_count": x_count,
        "y_count": y_count,
        "mesh_x_pps": options.mesh_pps[0],
        "mesh_y_pps": options.mesh_pps[1],
        "algo": options.algorithm,
        "tension": options.bicubic_tension,
        "mesh_min": options.mesh_min,
        "mesh_max": options.mesh_max,
        "origin": origin,
        "radius": options.mesh_radius,
    })
}

/// The bounds of the probe points — upstream takes `min_x`…`max_y` from the
/// generated points too (`bed_mesh.py:676-680`).
fn bounds(points: &[(f64, f64)]) -> (f64, f64, f64, f64) {
    let mut bounds = (
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
    );
    for (x, y) in points {
        bounds.0 = bounds.0.min(*x);
        bounds.1 = bounds.1.max(*x);
        bounds.2 = bounds.2.min(*y);
        bounds.3 = bounds.3.max(*y);
    }
    bounds
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::test_support::{context, silent_target};
    use crate::core::klippy::api::StartArgs;
    use crate::core::klippy::config::{ConfigSection, ConfigValue, ConfigWrapper};
    use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
    use crate::core::klippy::reactor::ManualReactor;

    fn request(body: &str) -> Request {
        Request::parse(body.as_bytes()).expect("test body is a valid request")
    }

    /// The `[bed_mesh]` section the tests calibrate over: a 0–100mm bed with a
    /// 3×3 grid, so the probe points are the 50mm-spaced corners and centre.
    fn section() -> ConfigSection {
        let mut section = ConfigSection::new("bed_mesh", None);
        for (option, value) in [
            ("mesh_min", "0,0"),
            ("mesh_max", "100,100"),
            ("probe_count", "3,3"),
        ] {
            section
                .parameters
                .insert(option.to_string(), ConfigValue::Single(value.to_string()));
        }
        section
    }

    /// A printer with `gcode` and a configured `[bed_mesh]`, but no
    /// calibration yet — `bed` is a handle to store one on.
    fn printer() -> (Arc<Printer>, Arc<BedMesh>) {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let bed = Arc::new(
            BedMesh::new(&ConfigWrapper::untracked(&section()), &printer)
                .expect("the test section is valid"),
        );
        printer.add_object(BED_MESH_OBJECT, bed.clone()).unwrap();
        (printer, bed)
    }

    /// The endpoint's answer to one request body.
    async fn dump(printer: Arc<Printer>, body: &str) -> Result<Value, ApiError> {
        let endpoint = BedMeshDump::new(printer);
        let api = Api::new();
        endpoint
            .handle(&request(body), &context(&api, silent_target()))
            .await
    }

    #[tokio::test]
    async fn test_the_path_is_the_documented_one() {
        let (printer, _bed) = printer();
        assert_eq!(BedMeshDump::new(printer).path(), "bed_mesh/dump_mesh");
    }

    #[tokio::test]
    async fn test_the_endpoint_is_registered_and_listed_by_list_endpoints() {
        let (printer, _bed) = printer();
        let mut api = Api::new();
        crate::core::klippy::api::register(
            &mut api,
            &printer,
            StartArgs::collect("/tmp/printer.cfg", None),
        )
        .unwrap();

        let listed = api
            .dispatch(&request(r#"{"method":"list_endpoints"}"#), silent_target())
            .await
            .unwrap();

        assert!(listed["endpoints"]
            .as_array()
            .unwrap()
            .iter()
            .any(|path| path == "bed_mesh/dump_mesh"));
        // …and reachable through the same table.
        let answer = api
            .dispatch(
                &request(r#"{"method":"bed_mesh/dump_mesh"}"#),
                silent_target(),
            )
            .await
            .unwrap();
        assert_eq!(answer["current_mesh"], json!({}));
    }

    #[tokio::test]
    async fn test_a_bed_that_has_not_been_probed_reports_no_current_mesh() {
        let (printer, _bed) = printer();

        let answer = dump(printer, r#"{"method":"bed_mesh/dump_mesh"}"#)
            .await
            .unwrap();

        assert_eq!(answer, json!({"current_mesh": {}, "profiles": {}}));
        assert!(answer.get("calibration").is_none());
    }

    #[tokio::test]
    async fn test_a_loaded_mesh_comes_back_with_its_profile_grid_and_params() {
        let (printer, bed) = printer();
        bed.store_mesh_for_test(
            "default",
            vec![
                vec![0.1, 0.2, 0.3],
                vec![0.4, 0.5, 0.6],
                vec![0.7, 0.8, 0.9],
            ],
        );

        let answer = dump(printer, r#"{"method":"bed_mesh/dump_mesh"}"#)
            .await
            .unwrap();

        assert_eq!(
            answer,
            json!({
                "current_mesh": {
                    "name": "default",
                    "probed_matrix": [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6], [0.7, 0.8, 0.9]],
                    // No interpolation yet: the probed grid stands in for it.
                    "mesh_matrix": [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6], [0.7, 0.8, 0.9]],
                    "mesh_params": {
                        "min_x": 0.0, "max_x": 100.0,
                        "min_y": 0.0, "max_y": 100.0,
                        "x_count": 3, "y_count": 3,
                        "mesh_x_pps": 2, "mesh_y_pps": 2,
                        "algo": "lagrange", "tension": 0.2
                    }
                },
                "profiles": {}
            })
        );
    }

    #[tokio::test]
    async fn test_mesh_args_adds_the_calibration_block() {
        let (printer, _bed) = printer();

        let answer = dump(
            printer,
            r#"{"method":"bed_mesh/dump_mesh","params":{"mesh_args":{}}}"#,
        )
        .await
        .unwrap();

        assert_eq!(
            answer["calibration"],
            json!({
                // The 3×3 grid zigzags: even rows left to right, odd ones back.
                "points": [
                    [0.0, 0.0], [50.0, 0.0], [100.0, 0.0],
                    [100.0, 50.0], [50.0, 50.0], [0.0, 50.0],
                    [0.0, 100.0], [50.0, 100.0], [100.0, 100.0],
                ],
                "config": {
                    "x_count": 3, "y_count": 3,
                    "mesh_x_pps": 2, "mesh_y_pps": 2,
                    "algo": "lagrange", "tension": 0.2,
                    "mesh_min": [0.0, 0.0], "mesh_max": [100.0, 100.0],
                    "origin": null, "radius": null
                }
            })
        );
    }

    #[tokio::test]
    async fn test_mesh_args_must_be_an_object() {
        let (printer, _bed) = printer();

        let error = dump(
            printer,
            r#"{"method":"bed_mesh/dump_mesh","params":{"mesh_args":"MESH_MIN=1,1"}}"#,
        )
        .await
        .unwrap_err();

        assert_eq!(
            error,
            ApiError::InvalidArgumentType("mesh_args".to_string())
        );
        assert_eq!(error.to_string(), "Invalid Argument Type [mesh_args]");
    }

    #[tokio::test]
    async fn test_a_machine_without_bed_mesh_gets_the_no_callback_answer() {
        let printer = Arc::new(Printer::new(ManualReactor::shared()));

        let error = dump(printer, r#"{"method":"bed_mesh/dump_mesh"}"#)
            .await
            .unwrap_err();

        assert_eq!(error, ApiError::UnknownEndpoint(PATH.to_string()));
        assert_eq!(
            error.to_string(),
            "webhooks: No registered callback for path 'bed_mesh/dump_mesh'"
        );
    }
}
