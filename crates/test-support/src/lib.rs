// KlipperX Test Support Library
//
// This crate provides test utilities for KlipperX. Its build.rs builds the
// klipper host shared library (libklipper_host.so) the interface tests load,
// one data dictionary per enabled MCU target, and the linux process firmware
// (`klipper.elf`) the `linux_firmware` fixture runs.

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
    checkout_dir("KLIPPERX_KLIPPER_DIR", "third_party/klipper")
}

/// Absolute path to the katapult checkout: the bootloader protocol
/// (`protocol.md`), its firmware sources, and `scripts/flashtool.py`.
///
/// `KLIPPERX_KATAPULT_DIR` overrides the in-tree submodule location with the
/// same contract as `KLIPPERX_KLIPPER_DIR`: a git worktree carries no
/// submodules, so it points this at the main checkout's `third_party/katapult`
/// instead of copying one. The path is read per call rather than cached.
pub fn katapult_dir() -> std::path::PathBuf {
    checkout_dir("KLIPPERX_KATAPULT_DIR", "third_party/katapult")
}

/// One submodule checkout, resolved the way both [`klipper_dir`] and
/// [`katapult_dir`] resolve it: the override in `env_var` when it is set to a
/// non-empty value, otherwise `relative` under the repository root.
fn checkout_dir(env_var: &str, relative: &str) -> std::path::PathBuf {
    resolve_checkout(std::env::var_os(env_var), relative)
}

/// [`checkout_dir`] with the environment read already done, so the resolution
/// itself is testable without mutating process-wide state.
fn resolve_checkout(
    override_dir: Option<std::ffi::OsString>,
    relative: &str,
) -> std::path::PathBuf {
    match override_dir.filter(|dir| !dir.is_empty()) {
        Some(dir) => std::path::PathBuf::from(dir),
        None => workspace_root().join(relative),
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
/// `CONFIG_MUX_SERIAL` / `CONFIG_BRIDGE_PORT_SERIAL`; the routing bridge comes
/// with the mux half) and the frame API on. It has its own output directory, so
/// the library [`klipper_host_lib_path`] names — which has neither — is
/// untouched.
pub fn klipper_host_mux_lib_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("KLIPPER_HOST_MUX_LIB"))
}

/// The tty path the mux host library's bridge opens for the host's UART
/// objects.
///
/// A path inside the build's own `OUT_DIR`, not a real device: the stream is
/// opened when the host configures a UART object (`config_uart`), and a tty
/// that cannot be opened fails that configuration, so a test creates a pty
/// there before it configures a port.
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

/// Absolute path to the built linux process firmware (`klipper.elf`).
///
/// The `test/configs/linuxprocess.config` target (`CONFIG_MACH_LINUX=y`),
/// built into this crate's `OUT_DIR` like the dictionaries, and the binary
/// [`linux_firmware::LinuxFirmware`] runs as a standalone process.
///
/// `build.rs` asserts the file exists when the default architecture set builds
/// it; with a `KLIPPERX_ARCHES` that leaves `linux` out the path still resolves
/// but the file is absent, and spawning then reports the missing file.
pub fn linux_firmware_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("KLIPPER_LINUX_FIRMWARE"))
}

/// Absolute path to the built linux process firmware with the serial mux
/// (`test/configs/linuxprocess-mux.config`: `CONFIG_MACH_LINUX=y` +
/// `CONFIG_MUX_SERIAL=y`) — the binary [`linux_firmware::LinuxFirmware::spawn_named`]
/// runs with an identity.
///
/// `build.rs` treats it exactly like [`linux_firmware_path`]: it asserts the
/// file when the default architecture set builds it, and with a `KLIPPERX_ARCHES`
/// that leaves `linux` out the path still resolves but the file is absent.
pub fn linux_mux_firmware_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("KLIPPER_LINUX_MUX_FIRMWARE"))
}

/// Absolute path to the built linux process firmware with the bridge port
/// backend (`test/configs/linuxprocess-bridge.config`: `CONFIG_MACH_LINUX=y` +
/// `CONFIG_MUX_SERIAL=y` + `CONFIG_LINUX_BRIDGE_SERIAL=y`) — the binary
/// [`linux_firmware::LinuxFirmware::spawn_bridge_port`] runs as a **bridge**,
/// with a tty of its own as the downstream port.
///
/// `build.rs` treats it exactly like [`linux_firmware_path`]: it asserts the
/// file when the default architecture set builds it, and with a `KLIPPERX_ARCHES`
/// that leaves `linux` out the path still resolves but the file is absent.
pub fn linux_bridge_firmware_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("KLIPPER_LINUX_BRIDGE_FIRMWARE"))
}

/// A private copy of a build artefact, so a test that wants its own instance of
/// it does not share the one the other tests load.
///
/// The host library the interface tests load is registered **per path**: every
/// reference on one `.so` path shares one loaded library, its runtime and its
/// reader (the registry in `src/core/klippy/interface/devices/host.rs`). So a test
/// that has to own its library — because it runs beside the others rather than
/// under their serialising lock — makes a copy on a path of its own.
///
/// The copy lands in a directory of its own under [`std::env::temp_dir`], named
/// from the process id, the creating thread's id and a per-process counter, so
/// two copies in one process never land on the same path. The directory is
/// removed when this is dropped.
pub struct PrivateCopy {
    dir: std::path::PathBuf,
    path: std::path::PathBuf,
}

impl PrivateCopy {
    /// Copy `source` next to itself, under a path no other copy uses.
    pub fn of(source: &std::path::Path) -> std::io::Result<Self> {
        use std::sync::atomic::{AtomicUsize, Ordering};

        static NEXT_COPY: AtomicUsize = AtomicUsize::new(0);

        let name = source
            .file_name()
            .ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("{} has no file name to copy under", source.display()),
                )
            })?
            .to_os_string();
        let dir = std::env::temp_dir().join(format!(
            "klipperx-private-copy-{}-{:?}-{}",
            std::process::id(),
            std::thread::current().id(),
            NEXT_COPY.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::create_dir_all(&dir)?;
        let path = dir.join(name);
        std::fs::copy(source, &path)?;
        Ok(Self { dir, path })
    }

    /// The copy's own path — the one to load, and the one to hand to anything
    /// that waits for that library to unload.
    pub fn path(&self) -> &std::path::Path {
        &self.path
    }
}

impl Drop for PrivateCopy {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

/// The linux firmware process fixture (spawn / reap, see [`LinuxFirmware`]).
pub mod linux_firmware;
pub use linux_firmware::LinuxFirmware;

#[cfg(test)]
mod tests {
    use super::*;

    /// A private copy holds the same bytes on a path of its own, and leaves
    /// nothing behind: the two properties a test that loads it relies on (the
    /// registry is per path, and a leftover `.so` would pile up in the temp
    /// directory).
    #[test]
    fn a_private_copy_copies_the_bytes_to_a_path_of_its_own_and_cleans_up() {
        let source = std::env::temp_dir().join(format!(
            "klipperx-private-copy-source-{}-{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        std::fs::write(&source, b"library bytes").expect("the source file is written");

        let first = PrivateCopy::of(&source).expect("the first copy is made");
        let second = PrivateCopy::of(&source).expect("the second copy is made");
        assert_ne!(first.path(), source, "the copy is not the source path");
        assert_ne!(first.path(), second.path(), "two copies never share a path");
        assert_eq!(
            std::fs::read(first.path()).expect("the copy is readable"),
            std::fs::read(&source).expect("the source is readable"),
            "the copy carries the source's bytes"
        );

        let dir = first
            .path()
            .parent()
            .expect("the copy lives in a directory")
            .to_path_buf();
        drop(first);
        assert!(!dir.exists(), "dropping the copy removes its directory");

        let dir = second
            .path()
            .parent()
            .expect("the copy lives in a directory")
            .to_path_buf();
        drop(second);
        assert!(!dir.exists(), "dropping the copy removes its directory");
        let _ = std::fs::remove_file(&source);
    }

    /// The resolution contract both submodule checkouts share: a non-empty
    /// override wins, anything else falls back to the in-tree submodule — the
    /// default a git worktree (submodules not populated) starts from before it
    /// points the variable at the main checkout.
    #[test]
    fn checkout_override_wins_and_the_in_tree_submodule_is_the_default() {
        let default = workspace_root().join("third_party/katapult");
        assert_eq!(resolve_checkout(None, "third_party/katapult"), default);
        assert_eq!(
            resolve_checkout(Some(std::ffi::OsString::new()), "third_party/katapult"),
            default
        );
        assert_eq!(
            resolve_checkout(
                Some(std::ffi::OsString::from(
                    "/tmp/main-checkout/third_party/katapult"
                )),
                "third_party/katapult"
            ),
            std::path::PathBuf::from("/tmp/main-checkout/third_party/katapult")
        );
    }
}
