// KlipperX Test Support Library
//
// This crate provides test utilities for KlipperX. Its build.rs builds the
// klipper host shared library (libklipper_host.so) so that tests can load it at
// runtime.

/// Absolute path to the built klipper host shared library.
///
/// The path is decided by `build.rs`, which builds the library into this crate's
/// `OUT_DIR` with a configuration of its own — the klipper submodule's `.config`
/// and `out/` belong to whatever else the developer is building. See the comments
/// in `build.rs` for why the configuration has to be written out explicitly.
///
/// The library is guaranteed to exist when running `cargo test`, because
/// `build.rs` builds it as part of the test build and fails if it is missing.
pub fn klipper_host_lib_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("KLIPPER_HOST_LIB"))
}
