// crates/test-support/build.rs
//
// Build the Klipper host shared library (libklipper_host.so) that the host
// interface tests load at runtime.
//
// The klipper tree is a submodule that a developer may also be building for real
// hardware, so it has a `.config` and an `out/` of its own. This script uses
// neither: it keeps a configuration and an output directory of its own inside the
// build script's OUT_DIR, and passes both to make, so the test build is
// reproducible from a clean checkout and leaves the developer's build alone.
//
// The configuration is written rather than derived, because kconfig's defaults do
// not describe what these tests need (see HOST_CONFIG below).

use std::path::{Path, PathBuf};
use std::process::Command;

/// What the tests need, stated explicitly.
///
/// kconfig has no default for the machine choice, so an empty configuration
/// selects the first entry (AVR) and builds firmware into a `klipper.elf` the
/// tests cannot load. `CONFIG_HOST_AR_LIBRARY` defaults to the static archive,
/// which produces `libklipper_host.a` instead of the `.so`. Both have to be said
/// out loud; `make olddefconfig` fills in everything else.
const HOST_CONFIG: &str = "\
# Written by klipperx-test-support/build.rs -- edit build.rs, not this file.
CONFIG_MACH_HOST=y
# CONFIG_HOST_AR_LIBRARY is not set
";

fn main() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_root = manifest_dir
        .parent()
        .and_then(Path::parent)
        .expect("test-support must live at crates/test-support")
        .to_path_buf();
    let klipper_dir = workspace_root.join("third_party/klipper");

    // OUT_DIR is stable across runs of the same build, which is what keeps the
    // klipper build incremental: only the first `cargo test` compiles it.
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").expect("cargo sets OUT_DIR"));
    let build_dir = out_dir.join("klipper-host");
    let config = build_dir.join("host.config");
    let klipper_out = build_dir.join("out");

    std::fs::create_dir_all(&build_dir).expect("failed to create the klipper build directory");
    if std::fs::read_to_string(&config).ok().as_deref() != Some(HOST_CONFIG) {
        std::fs::write(&config, HOST_CONFIG).expect("failed to write the host configuration");
    }

    // Watch the sources this build depends on, so a change in the submodule
    // triggers a rebuild instead of a stale library.
    println!(
        "cargo:rerun-if-changed={}",
        klipper_dir.join("src").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        klipper_dir.join("Makefile").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        klipper_dir.join("scripts/buildcommands.py").display()
    );

    println!(
        "cargo:warning=KlipperX test-support: building {}...",
        library(&klipper_out).display()
    );
    // Complete the fragment with every default the current klipper defines, then
    // build. Both are run every time; make decides what is actually stale.
    make(&klipper_dir, &config, &klipper_out, &["olddefconfig"]);
    make(&klipper_dir, &config, &klipper_out, &[]);

    let library = library(&klipper_out);
    assert!(
        library.exists(),
        "make did not produce {}: check that {HOST_CONFIG:?} still selects the shared host library",
        library.display()
    );

    // The tests ask for this path at runtime and cannot derive it: OUT_DIR
    // contains a hash. Hand it over as a compile-time environment variable.
    println!("cargo:rustc-env=KLIPPER_HOST_LIB={}", library.display());
}

fn library(klipper_out: &Path) -> PathBuf {
    klipper_out.join("libklipper_host.so")
}

fn make(klipper_dir: &Path, config: &Path, klipper_out: &Path, targets: &[&str]) {
    let status = Command::new("make")
        .current_dir(klipper_dir)
        // KCONFIG_CONFIG and OUT are assigned inside klipper's Makefile, so only
        // a command line value overrides them.
        .arg(format!("KCONFIG_CONFIG={}", config.display()))
        .arg(format!("OUT={}/", klipper_out.display()))
        .args(targets)
        .status()
        .unwrap_or_else(|e| {
            panic!(
                "failed to run make in {}: {e}\n\
                 the host interface tests need make and a C toolchain",
                klipper_dir.display()
            )
        });
    assert!(
        status.success(),
        "make {} failed in {}",
        targets.join(" "),
        klipper_dir.display()
    );
}
