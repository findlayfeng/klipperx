// Hand-written upstream-style tests — not @generated.
//
// This file is `include!`d by the generated `mod.rs` as `mod extra`, so every
// `#[test]` here runs as part of the `upstream` test suite. Unlike the
// generated `cases/<stem>.rs` (rewritten on every `cargo test`), this file is
// tracked in git and never overwritten by `test-support/build.rs`.
//
// Use it for tests that exercise the same harness
// (`super::run_generated_upstream_case`) but are not part of upstream's
// `test/klippy/*.test` corpus. Name them `extra_*` to distinguish them from
// the generated `upstream_*` cases.
//
// Dictionaries and configs
// ------------------------
// Generated cases have their `.cfg` and `.dict` copied into `fixtures/` by the
// build script. Hand-written `extra_*` tests share that same `fixtures/`
// directory, so they reference configs and dictionaries the same way:
//
//     concat!(env!("CARGO_MANIFEST_DIR"),
//             "/src/core/klippy/upstream_generated/fixtures/<name>.cfg")
//
// If an `extra_*` test needs a dictionary that upstream's `test/configs/` does
// not build, drop a `.config` file into `extra_configs/` (sibling of this
// file). `test-support/build.rs` scans that directory and builds each
// `<name>.config` into `fixtures/<name>.dict` with the same `make` path the
// upstream dictionaries use, under the same `KLIPPERX_ARCHES` filter. A config
// whose architecture is not enabled is skipped (its `.dict` will not exist in
// `fixtures/`); a test that names it should carry `#[ignore]` or be gated
// accordingly.
//
// Example
// -------
// A custom config at `extra_configs/my_target.config` builds into
// `fixtures/my_target.dict`. A test referencing a corpus config
// (`fixtures/example-cartesian.cfg`) plus that custom dict:
//
//     #[test]
//     fn extra_g1_beyond_position_max_is_rejected() {
//         super::run_generated_upstream_case(
//             concat!(env!("CARGO_MANIFEST_DIR"),
//                     "/src/core/klippy/upstream_generated/fixtures/example-cartesian.cfg"),
//             &[(None, concat!(env!("CARGO_MANIFEST_DIR"),
//                      "/src/core/klippy/upstream_generated/fixtures/atmega2560.dict"))],
//             r#"
//             G28
//             G1 Y9999
//             "#,
//             true, // SHOULD_FAIL
//         );
//     }
