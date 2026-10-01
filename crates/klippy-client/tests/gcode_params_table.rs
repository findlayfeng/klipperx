//! The checked-in built-in parameter table (`gcode_params::BUILTIN`) must agree
//! with a fresh scan of the host's registration points.
//!
//! Two things are checked, in the two directions that can drift:
//!
//! * **Freshness**: `BUILTIN` is exactly what scanning `src/core/klippy` today
//!   produces, so a changed declaration is caught at test time rather than
//!   shipped stale.
//! * **Completeness**: every `register_*_with_params` call site is either
//!   resolved or listed as unresolved. Nothing is skipped silently, and the
//!   known unresolved sites are pinned so a new one cannot appear unnoticed.

use std::collections::BTreeSet;

use klippy_client::gcode_params::BUILTIN;
use klippy_client::gcode_params_scan;

/// The call sites the scanner cannot resolve today, by file. Each is a dynamic
/// name (the host's own forwarding helpers, a registration closure) or a
/// parameter list built by a helper function rather than a literal or a
/// same-file constant. If this changes, the scan changed.
const KNOWN_UNRESOLVED_FILES: &[&str] = &[
    "extras/axis_twist_compensation.rs",
    "extras/bed_mesh.rs",
    "extras/bed_tilt.rs",
    "extras/delta_calibrate.rs",
    "extras/load_cell_probe.rs",
    "extras/probe.rs",
    "extras/quad_gantry_level.rs",
    "extras/screws_tilt_adjust.rs",
    "extras/tmc.rs",
    "extras/z_tilt.rs",
    "gcode.rs",
];

#[test]
fn test_builtin_table_is_what_a_fresh_scan_produces() {
    let scan = gcode_params_scan::scan().expect("scanning src/core/klippy");
    let scanned = scan.table();
    let checked_in: Vec<(String, Vec<String>)> = BUILTIN
        .iter()
        .map(|(name, params)| {
            (
                (*name).to_string(),
                params.iter().map(|param| (*param).to_string()).collect(),
            )
        })
        .collect();

    assert_eq!(
        scanned, checked_in,
        "src/gcode_params.rs is stale: regenerate it with \
         `cargo run -p klippy-client --bin gen-gcode-params` and commit the result"
    );
}

#[test]
fn test_every_registration_call_site_is_resolved_or_recorded() {
    let scan = gcode_params_scan::scan().expect("scanning src/core/klippy");

    assert_eq!(
        scan.resolved_call_sites + scan.unresolved.len(),
        scan.call_sites,
        "a registration call site was neither resolved nor recorded"
    );
    // Pinning the total is what keeps that sum honest: were the scanner to stop
    // recognising a call site entirely, this drops and the check goes red.
    assert_eq!(
        scan.call_sites, 78,
        "the number of registration call sites the scanner sees changed"
    );

    // The known unresolved sites are pinned: their count and the set of files
    // they live in. A new unresolved site — or one that resolves today and does
    // not tomorrow — changes one of these.
    let files: BTreeSet<&str> = scan
        .unresolved
        .iter()
        .map(|site| site.file.as_str())
        .collect();
    let known: BTreeSet<&str> = KNOWN_UNRESOLVED_FILES.iter().copied().collect();
    assert_eq!(
        scan.unresolved.len(),
        13,
        "the number of unresolved registration call sites changed: {:#?}",
        scan.unresolved
    );
    assert_eq!(
        files, known,
        "the set of files with unresolved registration call sites changed: {:#?}",
        scan.unresolved
    );
}
