// crates/test-support/build.rs
//
// Build what the tests need, in output directories of this build script's own
// inside OUT_DIR:
//
//   * the host shared library (`libklipper_host.so`) its interface tests load;
//   * a data dictionary for every `test/configs/*.config` whose architecture is
//     enabled, named after that config (`<name>.config` -> `<name>.dict`).
//
// The architectures to build come from `KLIPPERX_ARCHES` (comma separated), or
// `KLIPPERX_ALL_ARCHES` for every target. The default is the set with widely
// available toolchains — `linux` (host), `avr` (gcc-avr) and every ARM family
// (arm-none-eabi); `pru`, `ar100` and `simu` are not built unless asked for.
// Filtering happens here rather than at test time so that a target that cannot
// be built fails the build with make's own error, instead of turning into a
// silent skip. Setting any of these variables triggers a rebuild
// (`rerun-if-env-changed`).
//
// The klipper tree is a submodule that a developer may also be building for real
// hardware, so it has a `.config` and an `out/` of its own. This script uses
// neither: it copies each configuration into its own output directory and passes
// that to make, so the test build is reproducible from a clean checkout, leaves
// the developer's build alone, and never writes into the submodule.

use std::path::{Path, PathBuf};
use std::process::Command;

/// The host target, which the interface tests load as a shared library.
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
    // klipper builds incremental: only the first `cargo test` compiles them.
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").expect("cargo sets OUT_DIR"));
    let build_dir = out_dir.join("klipper-targets");

    // Watch what these builds depend on, so a change in the submodule or in the
    // selected architectures triggers a rebuild instead of a stale artifact.
    for changed in [
        "src",
        "Makefile",
        "scripts/buildcommands.py",
        "test/configs",
    ] {
        println!(
            "cargo:rerun-if-changed={}",
            klipper_dir.join(changed).display()
        );
    }
    println!("cargo:rerun-if-env-changed=KLIPPERX_ARCHES");
    println!("cargo:rerun-if-env-changed=KLIPPERX_ALL_ARCHES");

    // The host target: a shared library the interface tests load.
    let host_out = build(&klipper_dir, &build_dir.join("host"), HOST_CONFIG);
    let library = host_out.join("libklipper_host.so");
    assert!(
        library.exists(),
        "make did not produce {}: check that {HOST_CONFIG:?} still selects the shared host library",
        library.display()
    );
    println!("cargo:rustc-env=KLIPPER_HOST_LIB={}", library.display());

    // One dictionary per enabled target, named after the config it was built
    // from. A target that fails to build fails the build here.
    let all_architectures = std::env::var_os("KLIPPERX_ALL_ARCHES").is_some();
    let architectures = enabled_architectures();
    let dicts = build_dir.join("dicts");
    std::fs::create_dir_all(&dicts).expect("failed to create the dictionary directory");
    let mut built = 0usize;
    for config in
        std::fs::read_dir(klipper_dir.join("test/configs")).expect("test/configs is readable")
    {
        let config = config.expect("a directory entry").path();
        if config.extension().and_then(|e| e.to_str()) != Some("config") {
            continue;
        }
        let Some(name) = config.file_stem().map(|s| s.to_string_lossy().to_string()) else {
            continue;
        };
        let text = std::fs::read_to_string(&config).expect("a readable config");
        let Some(architecture) = architecture(&text) else {
            continue;
        };
        if !all_architectures && !architectures.contains(&architecture) {
            continue;
        }

        let target_out = build(&klipper_dir, &build_dir.join("mcu").join(&name), &text);
        let dictionary = require_dict(&target_out);
        std::fs::copy(&dictionary, dicts.join(format!("{name}.dict")))
            .unwrap_or_else(|e| panic!("failed to collect {}: {e}", dictionary.display()));
        built += 1;
    }
    assert!(
        built > 0,
        "no test/configs target matched KLIPPERX_ARCHES={architectures:?} \
         (KLIPPERX_ALL_ARCHES builds every target)"
    );
    println!("cargo:rustc-env=KLIPPERX_TEST_DICTS={}", dicts.display());
}

/// The architectures built by default: the ones whose toolchains are easy to
/// come by.
///
/// `linux` builds with the host compiler and `avr` with `avr-gcc`; the rest are
/// the ARM families, all of which build with `arm-none-eabi-gcc`. The remaining
/// families are left out: `pru` needs a PRU toolchain and `ar100` an or1k one
/// (both less common), and no `.test` uses `simu` at all.
const DEFAULT_ARCHITECTURES: &[&str] = &[
    "linux", "avr", "stm32", "atsam", "atsamd", "lpc176x", "rpxxxx", "hc32f460",
];

/// The architectures to build, from `KLIPPERX_ARCHES`; [`DEFAULT_ARCHITECTURES`]
/// when it is not set.
///
/// `KLIPPERX_ALL_ARCHES` (any value) overrides this and builds every target.
fn enabled_architectures() -> Vec<String> {
    match std::env::var("KLIPPERX_ARCHES") {
        Ok(value) => value
            .split(',')
            .map(|entry| entry.trim().to_ascii_lowercase())
            .filter(|entry| !entry.is_empty())
            .collect(),
        Err(_) => DEFAULT_ARCHITECTURES
            .iter()
            .map(|entry| (*entry).to_string())
            .collect(),
    }
}

/// The architecture a kconfig fragment selects.
///
/// Klipper names the family with an all-uppercase `CONFIG_MACH_<FAMILY>` (`AVR`,
/// `STM32`, `LINUX`, …); the board key has lowercase letters. A fragment without
/// one is not a machine choice and is skipped.
fn architecture(config: &str) -> Option<String> {
    for line in config.lines() {
        let Some(key) = line.trim().strip_prefix("CONFIG_MACH_") else {
            continue;
        };
        let Some(family) = key.strip_suffix("=y") else {
            continue;
        };
        if family.chars().all(|c| !c.is_ascii_lowercase()) {
            return Some(family.to_ascii_lowercase());
        }
    }
    None
}

/// Write `config_text` into `target_dir` and build it, returning the output
/// directory.
fn build(klipper_dir: &Path, target_dir: &Path, config_text: &str) -> PathBuf {
    let config = target_dir.join("klipper.config");
    let klipper_out = target_dir.join("out");

    std::fs::create_dir_all(target_dir).expect("failed to create a klipper build directory");
    if std::fs::read_to_string(&config).ok().as_deref() != Some(config_text) {
        std::fs::write(&config, config_text).expect("failed to write a klipper configuration");
    }

    // Complete the fragment with every default the current klipper defines, then
    // build. Both are run every time; make decides what is actually stale.
    make(klipper_dir, &config, &klipper_out, &["olddefconfig"]);
    make(klipper_dir, &config, &klipper_out, &[]);
    klipper_out
}

/// The data dictionary a build produced, which every target emits.
fn require_dict(klipper_out: &Path) -> PathBuf {
    let dictionary = klipper_out.join("klipper.dict");
    assert!(
        dictionary.exists(),
        "make did not produce {}",
        dictionary.display()
    );
    dictionary
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
                 the tests need make and a C toolchain",
                klipper_dir.display()
            )
        });
    assert!(
        status.success(),
        "make {} failed in {}\n\
         (a target that needs a cross compiler must have that toolchain, or its \
         architecture must not be in KLIPPERX_ARCHES)",
        targets.join(" "),
        klipper_dir.display()
    );
}
