// KlipperX Test Support Library
//
// This crate provides test utilities for KlipperX.
// Its build.rs builds the klipper host shared library (libklipper_host.so)
// so that tests can load it at runtime.

/// Absolute path to the built klipper host shared library.
///
/// This is guaranteed to exist when running `cargo test`, because this
/// crate's build.rs builds it as part of the test build.
pub fn klipper_host_lib_path() -> std::path::PathBuf {
    let manifest_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .expect("test-support must live at crates/test-support");
    workspace_root.join("third_party/klipper/out/libklipper_host.so")
}
