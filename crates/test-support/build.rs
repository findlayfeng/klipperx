// crates/test-support/build.rs
//
// Build the Klipper host shared library for tests.
// This crate is only used during testing (via [dev-dependencies]),
// so the library is only built when running `cargo test`.

use std::path::{Path, PathBuf};
use std::process::Command;

fn main() {
    // This build.rs runs with current_dir = crates/test-support.
    // The klipper sources live at the workspace root: third_party/klipper
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .expect("test-support must live at crates/test-support")
        .to_path_buf();
    let klipper_dir = workspace_root.join("third_party/klipper");
    let lib_path = klipper_dir.join("out/libklipper_host.so");

    if lib_path.exists() && is_up_to_date(&klipper_dir, &lib_path) {
        println!("cargo:rerun-if-changed={}", lib_path.display());
    } else {
        println!("cargo:warning=KlipperX test-support: Building klipper host shared library...");
        build_klipper_shared_lib(&klipper_dir);
    }

    // No compile-time linking is needed here: host.rs loads the library
    // at runtime via libloading (dlopen), so the linker never sees it.
}

fn build_klipper_shared_lib(klipper_dir: &Path) {
    // Ensure shared library config (unset CONFIG_HOST_AR_LIBRARY)
    let config_path = klipper_dir.join(".config");
    if config_path.exists() {
        let config = std::fs::read_to_string(&config_path)
            .expect("Failed to read .config");
        let updated = if config.contains("CONFIG_HOST_AR_LIBRARY=y") {
            config.replace("CONFIG_HOST_AR_LIBRARY=y", "# CONFIG_HOST_AR_LIBRARY is not set")
        } else {
            config
        };
        std::fs::write(&config_path, updated)
            .expect("Failed to write .config");
    }

    // Configure for shared library build
    let status = Command::new("make")
        .current_dir(klipper_dir)
        .args(&["olddefconfig"])
        .status()
        .expect("Failed to run make olddefconfig");
    assert!(status.success(), "make olddefconfig failed");

    // Build the shared library
    let status = Command::new("make")
        .current_dir(klipper_dir)
        .status()
        .expect("Failed to run make");
    assert!(status.success(), "make failed");
}

fn is_up_to_date(klipper_dir: &Path, lib_path: &Path) -> bool {
    let sources = [
        "src/Kconfig",
        "src/host/Kconfig",
        "src/host/Makefile",
        "src/host/main.c",
        "src/host/gpio.c",
        "src/host/timer.c",
        "src/host/serial.c",
    ];

    let lib_mtime = std::fs::metadata(lib_path)
        .ok()
        .and_then(|m| m.modified().ok());

    if let Some(lib_mtime) = lib_mtime {
        for source in &sources {
            let source_path = klipper_dir.join(source);
            if let Ok(metadata) = std::fs::metadata(&source_path) {
                if let Ok(modified) = metadata.modified() {
                    if modified > lib_mtime {
                        return false;
                    }
                }
            }
        }
    }
    true
}
