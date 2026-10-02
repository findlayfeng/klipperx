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

use std::fs;
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
    // `KLIPPERX_KLIPPER_DIR` points at a klipper checkout outside this worktree.
    // A git worktree does not populate submodules, so pointing at the main
    // checkout's `third_party/klipper` is how a worktree gets a corpus without
    // copying it — which also avoids the dangling relative gitfile a copied
    // submodule leaves behind (`git status` then fails inside the worktree).
    // Sharing one checkout is safe: every build here writes only into this
    // build script's `OUT_DIR` (`KCONFIG_CONFIG` and `OUT` are set explicitly in
    // `make()`), so concurrent worktrees never touch the submodule's own files.
    let klipper_dir = std::env::var_os("KLIPPERX_KLIPPER_DIR")
        .filter(|dir| !dir.is_empty())
        .map(PathBuf::from)
        .unwrap_or_else(|| workspace_root.join("third_party/klipper"));

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
    println!("cargo:rerun-if-env-changed=KLIPPERX_KLIPPER_DIR");
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

    // Upstream's `test/klippy/*.test` corpus, turned into individual
    // `#[test]`s with every fixture (config path, dictionary paths, g-code)
    // frozen in as literals. See `generate_upstream_tests` for the format and
    // the ignore rules.
    println!(
        "cargo:rerun-if-changed={}",
        klipper_dir.join("test/klippy").display()
    );
    // The generated cases are written into the source tree under
    // `src/core/klippy/upstream_generated/` (gitignored) and pulled into the
    // `#[cfg(test)] mod upstream` with a plain relative `include!`. Writing to
    // the source tree sidesteps cargo's `DEP_*`/`OUT_DIR` visibility rules:
    // dev-dependency `links` metadata is *not* exposed to the main crate's
    // build.rs or to `env!`/`option_env!` at compile time, and `OUT_DIR` of a
    // dev-dependency is not reachable from the main crate either. A fixed
    // source-relative path needs none of that.
    let upstream_dir = workspace_root.join("src/core/klippy");
    generate_upstream_tests(&klipper_dir, &dicts, &upstream_dir);
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

// ===========================================================================
// Upstream `.test` corpus → generated integration tests
// ===========================================================================
//
// Upstream ships `test/klippy/*.test`, each a script of `CONFIG` / `DICTIONARY`
// / `GCODE` / `SHOULD_FAIL` directives plus inline g-code lines (see upstream's
// `scripts/test_klippy.py:32-64`). `upstream_test_cases_run` used to scan and
// drive every case at test time, in one giant `#[test]`; this generator turns
// the same corpus into one integration-test module per `.test` file and one
// `#[test]` per `CONFIG`, so `cargo test` runs them as ordinary individual
// tests — no runtime scan, no single point of failure, and `--ignored` / named
// filtering work the way every other test does.
//
// What is frozen in at generation time (so a test run never reads the `.test`
// file, and never reads a `GCODE` file the way `GCODE <path>` used to):
//
//   * the config path, resolved to an absolute path (the config file itself is
//     still read at runtime — `injected_config` has to rewrite its `[mcu]`
//     transport, so the bytes must come through `Config::from_text`);
//   * every dictionary path, resolved against the directory this build just
//     built, so a case names exactly the dictionary `make` produced;
//   * the g-code, whether inline lines or a `GCODE <file>` reference — the
//     file is read here and frozen in as a string literal;
//   * the `SHOULD_FAIL` flag.
//
// Two kinds of case are emitted as `#[ignore]` rather than dropped, so the test
// inventory stays stable across `KLIPPERX_ARCHES` changes and the ignore list
// stays honest:
//
//   * a case whose dictionary was not built under the current
//     `KLIPPERX_ARCHES` — `#[ignore = "dictionary <name> not built"]`;
//   * a case whose config file name is in `IGNORED` (carried over from the old
//     `upstream_test_cases_run`) — `#[ignore = "upstream IGNORED"]`.
//
// The output lands in `OUT_DIR/upstream-tests/`:
//
//   * `<stem>.rs` — one per `.test` file, holding that file's `#[test]`s;
//   * `upstream_root.rs` — `mod <stem> { include!(...) }` for every file, the
//     single root `tests/upstream.rs` includes.
//
// Both are `@generated`; they are never committed (they live under `OUT_DIR`).

/// Cases this host cannot pass yet, identified by their generated test
/// function name (`upstream_<test_stem>_config_<idx>_<cfg_stem>`). A case
/// listed here is emitted with `#[ignore = "upstream IGNORED"]` instead of
/// being skipped at run time, so `cargo test --ignored` still runs it and a
/// passing one shows up as a stale entry to remove.
///
/// Matching by the full generated name (rather than by config file name, as
/// the old `upstream_test_cases_run` did) scopes the ignore to a single case:
/// `example-cartesian.cfg` is referenced by `out_of_bounds.test`,
/// `commands.test` and `printers.test`, but only the `out_of_bounds` case
/// needs to be ignored here.
const IGNORED: &[&str] = &[
    // `out_of_bounds.test` expects `G1 Y9999` to fail a move-bounds check this
    // host has not implemented yet; the move succeeds, so `SHOULD_FAIL` is not
    // satisfied. Remove when move-bounds validation lands.
    "upstream_out_of_bounds_config_0_example_cartesian",
];

/// One MCU dictionary a case names.
#[derive(Clone)]
struct GenDict {
    mcu: Option<String>,
    file: String,
}

/// One run, as upstream's `test_klippy.py` would launch it.
struct GenRun {
    config: PathBuf,
    dictionaries: Vec<GenDict>,
    gcode: String,
    should_fail: bool,
}

/// Parse one `.test` file into its runs, mirroring `upstream.rs`'s
/// `parse_test_file` (which mirrors `scripts/test_klippy.py:32-64`).
///
/// A run is launched at the **next** `CONFIG`, or at end of file; a
/// `DICTIONARY` written after a `CONFIG` belongs to that pending run, one
/// written before applies to the runs that follow. `GCODE <path>` points at a
/// file read at generation time; any other non-directive line is inline g-code.
fn parse_test_file(path: &Path) -> Result<Vec<GenRun>, String> {
    let text =
        fs::read_to_string(path).map_err(|e| format!("failed to read {}: {e}", path.display()))?;
    let dir = path.parent().unwrap_or(Path::new("."));

    let mut runs = Vec::new();
    let mut dictionaries: Vec<GenDict> = Vec::new();
    let mut gcode_file: Option<PathBuf> = None;
    let mut gcode_lines: Vec<String> = Vec::new();
    let mut should_fail = false;
    // `(config, snapshot)` waiting for the next `CONFIG` or EOF; `None` in the
    // snapshot means "no dictionary yet, a following `DICTIONARY` may claim me".
    let mut pending: Option<(PathBuf, Option<Vec<GenDict>>)> = None;

    let flush = |runs: &mut Vec<GenRun>,
                 pending: &mut Option<(PathBuf, Option<Vec<GenDict>>)>,
                 dictionaries: &[GenDict],
                 gcode_file: &Option<PathBuf>,
                 gcode_lines: &[String],
                 should_fail: bool| {
        if let Some((config, snapshot)) = pending.take() {
            let gcode = match gcode_file {
                Some(path) => fs::read_to_string(path)
                    .map_err(|e| format!("{}: {e}", path.display()))
                    .unwrap_or_else(|e| panic!("upstream fixture: {e}")),
                None => gcode_lines.join("\n"),
            };
            runs.push(GenRun {
                config,
                dictionaries: snapshot.unwrap_or_else(|| dictionaries.to_vec()),
                gcode,
                should_fail,
            });
        }
    };

    for raw in text.lines() {
        let line = match raw.find('#') {
            Some(pos) => &raw[..pos],
            None => raw,
        };
        let parts: Vec<&str> = line.split_whitespace().collect();
        let Some((directive, args)) = parts.split_first() else {
            continue;
        };
        match *directive {
            "CONFIG" => {
                flush(
                    &mut runs,
                    &mut pending,
                    &dictionaries,
                    &gcode_file,
                    &gcode_lines,
                    should_fail,
                );
                let arg = args
                    .first()
                    .unwrap_or_else(|| panic!("{}: CONFIG needs a path", path.display()));
                let snapshot = (!dictionaries.is_empty()).then(|| dictionaries.clone());
                pending = Some((dir.join(arg), snapshot));
            }
            "DICTIONARY" => {
                let main = args
                    .first()
                    .unwrap_or_else(|| panic!("{}: DICTIONARY needs a file", path.display()));
                let mut entries = vec![GenDict {
                    mcu: None,
                    file: (*main).to_string(),
                }];
                for spec in &args[1..] {
                    let (mcu, file) = spec.split_once('=').unwrap_or_else(|| {
                        panic!(
                            "{}: secondary DICTIONARY '{spec}' is not name=file",
                            path.display()
                        )
                    });
                    entries.push(GenDict {
                        mcu: Some(mcu.to_string()),
                        file: file.to_string(),
                    });
                }
                dictionaries = entries;
                if let Some((_, snapshot)) = pending.as_mut() {
                    if snapshot.is_none() {
                        *snapshot = Some(dictionaries.clone());
                    }
                }
            }
            "GCODE" => {
                let arg = args
                    .first()
                    .unwrap_or_else(|| panic!("{}: GCODE needs a path", path.display()));
                gcode_file = Some(dir.join(arg));
            }
            "SHOULD_FAIL" => should_fail = true,
            _ => gcode_lines.push(line.trim().to_string()),
        }
    }
    flush(
        &mut runs,
        &mut pending,
        &dictionaries,
        &gcode_file,
        &gcode_lines,
        should_fail,
    );
    Ok(runs)
}

/// Escape `s` into a Rust string literal (with surrounding quotes).
fn rust_str_literal(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if c.is_control() => out.push_str(&format!("\\u{{{:x}}}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}
/// Render `s` as a Rust raw string literal (`r#"..."#`), using as many
/// surrounding `#`s as needed so a `"#` sequence inside `s` cannot close the
/// literal prematurely. Raw strings keep multi-line config / g-code / JSON
/// readable in the generated source instead of escaping every newline.
fn rust_raw_string(s: &str) -> String {
    // Count the longest run of `#`s that immediately follows any `"` in the
    // content; the closing delimiter needs one more than that to be
    // unambiguous.
    let bytes = s.as_bytes();
    let mut max_hashes = 0usize;
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'"' {
            i += 1;
            let mut run = 0usize;
            while i < bytes.len() && bytes[i] == b'#' {
                run += 1;
                i += 1;
            }
            max_hashes = max_hashes.max(run);
        } else {
            i += 1;
        }
    }
    let hashes = "#".repeat(max_hashes + 1);
    format!("r{hashes}\"{s}\"{hashes}")
}

/// Turn a file stem into a valid Rust ident (letters/digits/`_`, not starting
/// with a digit).
fn sanitize_ident(stem: &str) -> String {
    let mut out = String::new();
    for c in stem.chars() {
        if c.is_ascii_alphanumeric() || c == '_' {
            out.push(c.to_ascii_lowercase());
        } else {
            out.push('_');
        }
    }
    if out.chars().next().is_some_and(|c| c.is_ascii_digit()) {
        out.insert(0, '_');
    }
    if out.is_empty() {
        out.push_str("case");
    }
    out
}

/// The `.test` files, sorted (stable module order across platforms).
fn test_files(klipper_dir: &Path) -> Vec<PathBuf> {
    let mut files = Vec::new();
    for entry in
        fs::read_dir(klipper_dir.join("test/klippy")).expect("upstream test/klippy is readable")
    {
        let path = entry.expect("a directory entry").path();
        if path.extension().is_some_and(|e| e == "test") {
            files.push(path);
        }
    }
    files.sort();
    files
}

fn generate_upstream_tests(klipper_dir: &Path, dicts_dir: &Path, out_dir: &Path) {
    // Output layout (all under `src/core/klippy/upstream_generated/`, gitignored):
    //   * `mod.rs` — `mod <stem> { include!(...) }` for every `.test` file;
    //   * `cases/<stem>.rs` — one `#[test]` per `CONFIG` directive in that
    //     `.test`, with the config path and dictionary paths pointing at
    //     copies under `fixtures/` (relative paths, so the generated tree is
    //     self-contained and relocatable) and the g-code inlined as a raw
    //     string literal (it is the `.test`'s own inline content, not a file).
    //   * `fixtures/<cfg_name>.cfg` — every config the corpus references,
    //     copied here so a case never reads `third_party/klipper` at run time;
    //   * `fixtures/<dict_name>.dict` — every built dictionary the corpus
    //     references, copied here alongside the configs.
    let gen_dir = out_dir.join("upstream_generated");
    let _ = std::fs::remove_dir_all(&gen_dir);
    let cases_dir = gen_dir.join("cases");
    let fixtures_dir = gen_dir.join("fixtures");
    std::fs::create_dir_all(&cases_dir).expect("create upstream_generated/cases");
    std::fs::create_dir_all(&fixtures_dir).expect("create upstream_generated/fixtures");

    let mut root = String::new();
    root.push_str(
        "// @generated by test-support/build.rs from upstream test/klippy/*.test.\n\
         // Do not edit — re-run `cargo test` to regenerate.\n\
         // One `mod` per upstream `.test` file; each `#[test]` inside is one\n\
         // `CONFIG` directive. Config and dictionary files are copied under\n\
         // `fixtures/` and referenced by relative path; g-code is inlined as a\n\
         // raw string. Ignored cases carry `#[ignore]` with a reason\n\
         // (`upstream IGNORED` or `dictionary <name> not built`).\n",
    );

    for test_path in test_files(klipper_dir) {
        let stem = test_path
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| "case".to_string());
        let stem_ident = sanitize_ident(&stem);
        let runs = parse_test_file(&test_path).unwrap_or_else(|e| panic!("upstream fixture {e}"));

        let mut body = String::new();
        body.push_str(&format!(
            "// @generated from upstream test/klippy/{stem}.test — do not edit.\n\
             // Each `#[test]` below is one `CONFIG` directive. Config and dictionary\n\
             // paths are relative to this file's `fixtures/` sibling directory.\n",
        ));

        for (idx, run) in runs.iter().enumerate() {
            let cfg_stem = run
                .config
                .file_stem()
                .map(|s| s.to_string_lossy().to_string())
                .unwrap_or_else(|| "cfg".to_string());
            let fn_name = format!(
                "upstream_{stem_ident}_config_{idx}_{}",
                sanitize_ident(&cfg_stem)
            );

            // Copy the config file into `fixtures/`, named by its file name
            // (e.g. `bed_mesh.cfg`). A config that cannot be read is a build
            // error — the corpus is incomplete. The generated test references
            // it by relative path: `../fixtures/<name>`.
            let cfg_name = run
                .config
                .file_name()
                .map(|n| n.to_string_lossy().to_string())
                .unwrap_or_else(|| "config.cfg".to_string());
            let cfg_dest = fixtures_dir.join(&cfg_name);
            if !cfg_dest.exists() {
                std::fs::copy(&run.config, &cfg_dest).unwrap_or_else(|e| {
                    panic!("failed to copy config {}: {e}", run.config.display())
                });
            }
            let cfg_path_code = format!(
                "concat!(env!(\"CARGO_MANIFEST_DIR\"), \"/src/core/klippy/upstream_generated/fixtures/{cfg_name}\")"
            );

            // A case whose any dictionary was not built is ignored — running it
            // with a different target's dictionary would not be the run upstream
            // wrote. `#[ignore]` keeps it visible under `--ignored`. Built
            // dictionaries are copied into `fixtures/` and referenced by
            // relative path, so the case never reads `OUT_DIR` at run time.
            let mut missing_dict = None;
            let mut dict_args = String::new();
            dict_args.push('[');
            for (i, d) in run.dictionaries.iter().enumerate() {
                let dict_src = dicts_dir.join(&d.file);
                let dict_path_code = if dict_src.is_file() {
                    let dict_dest = fixtures_dir.join(&d.file);
                    if !dict_dest.exists() {
                        std::fs::copy(&dict_src, &dict_dest).unwrap_or_else(|e| {
                            panic!("failed to copy dictionary {}: {e}", dict_src.display())
                        });
                    }
                    format!(
                        "concat!(env!(\"CARGO_MANIFEST_DIR\"), \"/src/core/klippy/upstream_generated/fixtures/{}\")",
                        d.file
                    )
                } else {
                    missing_dict = Some(d.file.clone());
                    "\"\"".to_string()
                };
                let mcu_lit = match &d.mcu {
                    None => "None".to_string(),
                    Some(m) => format!("Some({})", rust_str_literal(m)),
                };
                if i > 0 {
                    dict_args.push_str(", ");
                }
                dict_args.push_str(&format!("({mcu_lit}, {dict_path_code})"));
            }
            dict_args.push(']');

            let ignore_attr = if let Some(name) = missing_dict {
                format!("#[ignore = \"dictionary {name} not built\"]\n")
            } else if IGNORED.contains(&fn_name.as_str()) {
                "#[ignore = \"upstream IGNORED\"]\n".to_string()
            } else {
                String::new()
            };

            body.push_str(&format!(
                "#[test]\n{ignore_attr}fn {fn_name}() {{\n    super::run_generated_upstream_case(\n        {cfg},\n        &{dicts},\n        {gcode},\n        {should_fail},\n    );\n}}\n\n",
                cfg = cfg_path_code,
                dicts = dict_args,
                gcode = rust_raw_string(
                    &format!(
                        "\n{}\n        ",
                        run.gcode
                            .split('\n')
                            .map(|l| format!("        {l}"))
                            .collect::<Vec<_>>()
                            .join("\n")
                    ),
                ),
                should_fail = run.should_fail,
            ));
        }

        let case_file = cases_dir.join(format!("{stem_ident}.rs"));
        std::fs::write(&case_file, body)
            .unwrap_or_else(|e| panic!("failed to write {}: {e}", case_file.display()));
        root.push_str(&format!(
            "mod {stem_ident} {{\n    include!(\"cases/{stem_ident}.rs\");\n}}\n",
        ));
    }

    let mod_file = gen_dir.join("mod.rs");
    std::fs::write(&mod_file, root)
        .unwrap_or_else(|e| panic!("failed to write {}: {e}", mod_file.display()));
    println!("cargo:rerun-if-changed={}", gen_dir.display());
}
