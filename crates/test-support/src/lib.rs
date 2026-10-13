// KlipperX Test Support Library
//
// This crate provides test utilities for KlipperX. Its build.rs builds the
// klipper host shared library (libklipper_host.so) the interface tests load, and
// one data dictionary per enabled MCU target.

/// Absolute path to the klipper checkout the tests read: the `.test` cases,
/// their configs, and the kconfig fragments.
///
/// `KLIPPERX_KLIPPER_DIR` overrides the in-tree submodule location. A git
/// worktree does not populate submodules, so a worktree points this at the main
/// checkout's `third_party/klipper` instead of copying it (a copy would also
/// leave a submodule whose relative gitfile does not resolve, which makes every
/// `git status` in that worktree fail). `build.rs` resolves the same path the
/// same way, and it only ever writes into its own `OUT_DIR`.
///
/// The path is read per call rather than cached, so a test that wants to point
/// at another checkout only has to set the variable before the call.
pub fn klipper_dir() -> std::path::PathBuf {
    let override_dir = std::env::var_os("KLIPPERX_KLIPPER_DIR").filter(|dir| !dir.is_empty());
    match override_dir {
        Some(dir) => std::path::PathBuf::from(dir),
        None => workspace_root().join("third_party/klipper"),
    }
}

/// The repository root, from this crate's own location.
fn workspace_root() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(std::path::Path::parent)
        .expect("test-support lives at crates/test-support")
        .to_path_buf()
}

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

/// Absolute path to the mux-flavoured klipper host shared library.
///
/// A second host build of the same target, with the firmware's serial-mux
/// device half compiled in (`CONFIG_HOST_BRIDGE_SERIAL`, which also selects
/// `CONFIG_MUX` / `CONFIG_MUX_SERIAL`) and the frame API on. It has its own
/// output directory, so the library [`klipper_host_lib_path`] names — which has
/// neither — is untouched.
pub fn klipper_host_mux_lib_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("KLIPPER_HOST_MUX_LIB"))
}

/// The tty path the mux host library's bridge opens in `klipper_host_init()`.
///
/// A path inside the build's own `OUT_DIR`, not a real device: the bridge fails
/// init when it cannot open its port, so a test creates a pty there before
/// loading the library.
pub fn klipper_host_mux_serial_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("KLIPPER_HOST_MUX_SERIAL_PATH"))
}

/// The data dictionary of the mux host build, for the message ids its firmware
/// answers (`get_clock`, `clock`, …).
///
/// The same `<name>.dict` JSON shape as [`test_dicts_dir`], at a path of its
/// own because the host target is not one of `test/configs`.
pub fn klipper_host_mux_dictionary_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("KLIPPER_HOST_MUX_DICT"))
}

/// Absolute path to the directory of built data dictionaries.
///
/// `build.rs` builds one `test/configs/*.config` per architecture named in
/// `KLIPPERX_ARCHES` (default `linux`) and collects each as `<name>.dict`, the
/// same name as the config it came from. A target that cannot be built fails the
/// build, so every file here exists and is complete.
pub fn test_dicts_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("KLIPPERX_TEST_DICTS"))
}
