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
//! Two gaps between that upstream handler and this one are deliberate, and
//! each is a capability this unit does not have yet rather than a shortcut:
//!
//! * **`mesh_args` is accepted but not applied** — upstream feeds its keys back
//!   into the mesh configuration (`bed_mesh.py:631-633`); the per-command
//!   overrides that reads are not written yet, so `calibration` describes the
//!   configuration the printer is running;
//! * **`probe_path` / `rapid_path` are absent** — those are upstream's probe
//!   scheduler walk (`bed_mesh.py:291-310`), which this host does not model yet.
//!
//! `profiles` is what the profile manager holds (the same map `bed_mesh`'s
//! `get_status` reports, `[bed_mesh <name>]` sections included), so a client's
//! profile list is real from the moment `BED_MESH_PROFILE SAVE=` (or
//! `BED_MESH_CALIBRATE PROFILE=`) has run.
//!
//! `current_mesh` carries the two grids upstream reports: `probed_matrix` is the
//! grid as probed, `mesh_matrix` the interpolation grid sampled from it
//! (`ZMesh`, `mesh_pps`) — the same two answers `bed_mesh`'s `get_status`
//! gives, read off the same loaded mesh.
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
    generate_points, BedMesh, BedMeshOptions, MeshParams, BED_MESH_OBJECT,
};
use crate::core::klippy::printer::{Printer, PrinterObject};

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
            // The same profiles `get_status` reports (`bed_mesh.py:1665`): empty
            // until a `BED_MESH_PROFILE SAVE=` / `BED_MESH_CALIBRATE PROFILE=`
            // has stored one.
            result.insert(
                "profiles".to_string(),
                bed.get_status(0.0)
                    .get("profiles")
                    .cloned()
                    .unwrap_or_else(|| json!({})),
            );
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

/// The `current_mesh` half: the loaded grids, or `{}` while there is none
/// (`bed_mesh.py:296-302`).
fn current_mesh(bed: &BedMesh) -> Result<Value, ApiError> {
    let Some(mesh) = bed.loaded_mesh() else {
        return Ok(json!({}));
    };
    Ok(json!({
        "name": mesh.name,
        "probed_matrix": mesh.rows,
        // The interpolation grid, reported to 6 decimals as upstream's
        // `get_mesh_matrix` does (`bed_mesh.py:1360-1364`).
        "mesh_matrix": mesh.z_mesh.get_mesh_matrix(),
        // The loaded grid's own parameters (`z_mesh.get_mesh_params()`,
        // `bed_mesh.py:296-302`), not the running config's: a `LOAD=` may
        // install a grid of another size.
        "mesh_params": mesh_params(&mesh.params),
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

/// The `mesh_params` of the loaded grid — `z_mesh.get_mesh_params()`
/// (`bed_mesh.py:296-302`): the parameters the grid was sampled with, whose
/// `min_x`…`max_y` are the bounds of its probe points (`bed_mesh.py:668-671`).
fn mesh_params(params: &MeshParams) -> Value {
    json!({
        "min_x": params.min_x,
        "max_x": params.max_x,
        "min_y": params.min_y,
        "max_y": params.max_y,
        "x_count": params.x_count,
        "y_count": params.y_count,
        "mesh_x_pps": params.mesh_x_pps,
        "mesh_y_pps": params.mesh_y_pps,
        "algo": params.algo.name(),
        "tension": params.tension,
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
            StartArgs::collect(
                "/tmp/printer.cfg",
                None,
                crate::core::klippy::api::endpoints::info::NO_KLIPPER_PATH,
            ),
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

        let mesh = &answer["current_mesh"];
        assert_eq!(mesh["name"], "default");
        // `probed_matrix` is the grid as probed …
        assert_eq!(
            mesh["probed_matrix"],
            json!([[0.1, 0.2, 0.3], [0.4, 0.5, 0.6], [0.7, 0.8, 0.9]])
        );
        // … and `mesh_matrix` the interpolation grid sampled from it: a 3×3
        // probe with the default `mesh_pps: 2` becomes a 7×7 mesh, holding the
        // probed values at the probed positions and interpolated ones between
        // them.
        let interpolated = mesh["mesh_matrix"].as_array().unwrap();
        assert_eq!(interpolated.len(), 7);
        assert!(interpolated
            .iter()
            .all(|row| row.as_array().unwrap().len() == 7));
        assert_eq!(interpolated[0][0], 0.1);
        assert_eq!(interpolated[0][3], 0.2);
        assert_eq!(interpolated[0][6], 0.3);
        assert_eq!(interpolated[3][0], 0.4);
        assert_eq!(interpolated[3][3], 0.5);
        assert_eq!(interpolated[6][6], 0.9);
        // Neither grid is the other one: mesh row 1 sits a third of the way
        // from probed row 0 to probed row 1, and that column rises linearly
        // from 0.2 to 0.5 over the 50mm between them, so it holds 0.3.
        assert!((interpolated[1][3].as_f64().unwrap() - 0.3).abs() < 1e-9);
        assert_eq!(
            mesh["mesh_params"],
            json!({
                "min_x": 0.0, "max_x": 100.0,
                "min_y": 0.0, "max_y": 100.0,
                "x_count": 3, "y_count": 3,
                "mesh_x_pps": 2, "mesh_y_pps": 2,
                "algo": "lagrange", "tension": 0.2
            })
        );
        assert_eq!(answer["profiles"], json!({}));
    }

    /// A machine whose config carries a `[bed_mesh <name>]` profile of another
    /// scale, read at load — the grid a `LOAD=` installs instead of the
    /// running configuration's.
    fn printer_with_stored_profile(text: &str) -> (Arc<Printer>, Arc<GCodeDispatch>) {
        use crate::core::klippy::config::object::{PrinterConfig, CONFIGFILE_OBJECT};
        use crate::core::klippy::config::{AccessTracking, Config};

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                GCODE_OBJECT,
                Arc::new(GCodeDispatch::new(Arc::clone(&printer))),
            )
            .unwrap();
        let configfile = Arc::new(PrinterConfig::new(
            AccessTracking::shared(),
            serde_json::Map::new(),
        ));
        printer
            .add_object(
                CONFIGFILE_OBJECT,
                Arc::clone(&configfile) as Arc<dyn PrinterObject>,
            )
            .unwrap();
        let (config, _) = Config::from_text(text).expect("the test config parses");
        let bed_mesh = config
            .get_section("bed_mesh")
            .expect("a [bed_mesh] section");
        let wrapper = ConfigWrapper::with_config(
            bed_mesh,
            AccessTracking::shared(),
            Some(configfile),
            &config,
        );
        let bed = Arc::new(BedMesh::new(&wrapper, &printer).expect("the test section is valid"));
        printer.add_object(BED_MESH_OBJECT, bed).unwrap();
        printer.send_event(&crate::core::klippy::event::KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .unwrap();
        (printer, gcode)
    }

    /// `mesh_params` is the **loaded** grid's parameters, not the running
    /// configuration's: a `4×4` profile over `10…250`/`20…200` reports its own
    /// bounds after `LOAD=`, while `calibration.config` keeps the configured
    /// `mesh_min`/`mesh_max` (`bed_mesh.py:296-302`, `bed_mesh.py:629-641`).
    #[tokio::test]
    async fn test_mesh_params_follow_the_loaded_grid_and_the_config_keeps_its_bounds() {
        let (printer, gcode) = printer_with_stored_profile(
            "[bed_mesh]\nmesh_min: 0,0\nmesh_max: 100,100\nprobe_count: 3,3\n\
             [bed_mesh shop]\nversion: 1\npoints:\n\
             \x20 0.100000, 0.200000, 0.300000, 0.400000\n\
             \x20 0.500000, 0.600000, 0.700000, 0.800000\n\
             \x20 0.900000, 1.000000, 1.100000, 1.200000\n\
             \x20 1.300000, 1.400000, 1.500000, 1.600000\n\
             min_x: 10.0\nmax_x: 250.0\nmin_y: 20.0\nmax_y: 200.0\n\
             x_count: 4\ny_count: 4\nmesh_x_pps: 1\nmesh_y_pps: 1\n\
             algo: direct\ntension: 0.2\n",
        );

        gcode
            .run_script("BED_MESH_PROFILE LOAD=shop")
            .await
            .expect("the stored profile loads");

        let answer = dump(
            Arc::clone(&printer),
            r#"{"method":"bed_mesh/dump_mesh","params":{"mesh_args":{}}}"#,
        )
        .await
        .unwrap();

        // The loaded grid's own bounding box, not the running config's.
        assert_eq!(
            answer["current_mesh"]["mesh_params"],
            json!({
                "min_x": 10.0, "max_x": 250.0,
                "min_y": 20.0, "max_y": 200.0,
                "x_count": 4, "y_count": 4,
                "mesh_x_pps": 1, "mesh_y_pps": 1,
                "algo": "direct", "tension": 0.2
            })
        );
        // `calibration` still describes the configuration being run.
        assert_eq!(
            answer["calibration"]["config"]["mesh_min"],
            json!([0.0, 0.0])
        );
        assert_eq!(
            answer["calibration"]["config"]["mesh_max"],
            json!([100.0, 100.0])
        );
        assert_eq!(answer["calibration"]["config"]["x_count"], 3);
        assert_eq!(answer["calibration"]["config"]["algo"], "lagrange");
    }

    /// The profile manager's own answer, not a hard-coded `{}`: a saved profile
    /// shows up in `profiles`, which is what a client's profile list reads.
    #[tokio::test]
    async fn test_a_saved_profile_comes_back_in_profiles() {
        let (printer, bed) = printer();
        bed.store_mesh_for_test(
            "shop",
            vec![
                vec![0.1, 0.2, 0.3],
                vec![0.4, 0.5, 0.6],
                vec![0.7, 0.8, 0.9],
            ],
        );

        // `SAVE=` is the profile manager's own path: the endpoint must report
        // what it stored, not a constant. Commands need the machine ready.
        printer.send_event(&crate::core::klippy::event::KlippyEvent::KlippyReady);
        let gcode = printer
            .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
            .expect("the fixture registers gcode");
        gcode
            .run_script("BED_MESH_PROFILE SAVE=shop")
            .await
            .expect("the loaded mesh saves as a profile");

        let answer = dump(printer, r#"{"method":"bed_mesh/dump_mesh"}"#)
            .await
            .unwrap();

        let profiles = answer["profiles"]
            .as_object()
            .expect("profiles is an object");
        let shop = profiles
            .get("shop")
            .expect("the stored profile is reported by name");
        assert_eq!(
            shop["points"],
            json!([[0.1, 0.2, 0.3], [0.4, 0.5, 0.6], [0.7, 0.8, 0.9]])
        );
        assert_eq!(shop["mesh_params"]["x_count"], 3);
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
