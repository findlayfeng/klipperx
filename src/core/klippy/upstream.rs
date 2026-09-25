//! Upstream Klipper's host test corpus, used as fixtures.
//!
//! Upstream's own host tests are `test/klippy/*.test`, run by
//! `scripts/test_klippy.py`: each case names a config, an MCU data dictionary
//! and some g-code, and passes if `klippy.py` runs it without error. The
//! dictionary goes to `-d` and a file to `-o`, so the case needs no hardware —
//! klippy injects the dictionary and tolerates a firmware that never answers
//! (`klippy/mcu.py:841`, `klippy/serialhdl.py:207`).
//!
//! This module is the shared harness: it knows where upstream keeps its cases
//! and how one is read, so the tests below (and any later test that wants to
//! reuse the corpus) do not each re-implement that. The corpus is reused in
//! stages:
//!
//! | stage | needs | state |
//! |---|---|---|
//! | the corpus is well formed and its inputs resolve | nothing | `upstream_test_cases_are_well_formed`, `upstream_test_inputs_resolve` |
//! | every shipped `.cfg` parses with our config parser | nothing | `every_upstream_printer_config_parses` |
//! | the inline g-code parses | the config sections the cases use | `#[ignore]` |
//! | a case runs end to end | the config sections the cases use | `upstream_test_cases_run`, with an ignore list |
//!
//! An end-to-end run talks to a real answerer, not to a host-only short circuit:
//! [`SimulatorDevice`](crate::core::klippy::interface::devices::simulator::SimulatorDevice)
//! is a dictionary-driven fake MCU. The harness replaces every `[mcu]` transport
//! with `test: dict=<dictionary>`, and the fake firmware serves identify, answers
//! the configuration handshake and clock reads, and acknowledges blocks.
//!
//! The tests follow the pipeline the corpus implies:
//!
//! 1. **Build the dictionaries.** `build.rs` reads `KLIPPERX_ARCHES` (comma
//!    separated; by default `linux`, `avr` and every ARM family — the toolchains
//!    that are easy to come by) — or every target when `KLIPPERX_ALL_ARCHES` is
//!    set — builds each matching `test/configs/*.config`, and collects it as the
//!    same-named `<name>.dict`. A target that cannot be built fails the build.
//! 2. **Drop cases without a `DICTIONARY`.** Upstream refuses to start one
//!    (`scripts/test_klippy.py:88`).
//! 3. **Drop cases whose dictionaries were not built.** There is no substitute:
//!    the case was written for that target's command set.
//! 4. **Drop the `IGNORED` cases** — the ones that cannot pass yet (missing
//!    config sections, kinematics, pin chips).
//!
//! What remains runs against the fake firmware, each `[mcu]` served its own
//! target's dictionary. `KLIPPERX_UPSTREAM_ALL=1` bypasses step 4 (the ignore
//! list), so the runs that have their dictionaries report their failures.
//!
//! The inline-g-code stage stays an `#[ignore]` test for the same reason: driving
//! a case's g-code needs the sections it names.
//!
//! One thing is deliberate here: the corpus is read-only. The submodule is
//! whatever the developer checked out, and nothing is written. The `.cfg`
//! parsing test holds the parser to upstream's `configparser` across the
//! shipped configs; it is what first exposed the multiline, `=`, and
//! section-header-comment gaps.

// A harness keeps a full API for the tests that will use it, not only the ones
// that exist today; an unused helper here is not dead code.
#![allow(dead_code)]

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::{Config, ConfigValue};
    use std::sync::Arc;

    /// Render `path` relative to the klipper submodule, for a readable failure.
    fn relative(path: &std::path::Path) -> String {
        path.strip_prefix(klipper_dir())
            .unwrap_or(path)
            .display()
            .to_string()
    }

    /// Every case must say what to run and against what.
    ///
    /// A `CONFIG` and a `DICTIONARY` are what `test_klippy.py` requires
    /// (`scripts/test_klippy.py:76-81`); the g-code is either a file or inline
    /// lines. This checks the corpus itself, so a malformed fixture is reported
    /// as such instead of as a mystery failure in a later stage.
    #[test]
    fn upstream_test_cases_are_well_formed() {
        let files = test_files();
        assert!(
            !files.is_empty(),
            "no .test files found under third_party/klipper"
        );

        let mut problems = Vec::new();
        for file in &files {
            let name = relative(file);
            let runs = parse_test_file(file).unwrap_or_else(|e| panic!("upstream fixture: {e}"));
            if runs.is_empty() {
                problems.push(format!("{name}: no CONFIG"));
                continue;
            }
            for run in &runs {
                if run.dictionaries.is_empty() {
                    problems.push(format!(
                        "{name}: {} has no DICTIONARY",
                        relative(&run.config)
                    ));
                }
                if run.gcode_file.is_none() && run.gcode_lines.is_empty() {
                    problems.push(format!(
                        "{name}: {} has no GCODE file and no inline g-code",
                        relative(&run.config)
                    ));
                }
            }
        }
        assert!(
            problems.is_empty(),
            "malformed upstream test runs:\n  {}",
            problems.join("\n  ")
        );
    }

    /// Every path a run names must exist in the tree, and every dictionary must
    /// have the kconfig fragment upstream's CI builds it from.
    ///
    /// The dictionaries themselves (`*.dict`) are build products, not sources,
    /// so the fragment in `test/configs/<name>.config` is what has to be there.
    #[test]
    fn upstream_test_inputs_resolve() {
        let mut problems = Vec::new();

        for run in &all_runs() {
            let name = relative(&run.path);
            if !run.config.is_file() {
                problems.push(format!(
                    "{name}: CONFIG {} does not exist",
                    relative(&run.config)
                ));
            }
            if let Some(gcode) = &run.gcode_file {
                if !gcode.is_file() {
                    problems.push(format!("{name}: GCODE {} does not exist", relative(gcode)));
                }
            }
            for dictionary in &run.dictionaries {
                if dictionary_source(dictionary).is_none() {
                    problems.push(format!(
                        "{name}: DICTIONARY {} has no test/configs fragment",
                        dictionary.file
                    ));
                }
            }
        }

        assert!(
            problems.is_empty(),
            "upstream test inputs that cannot be resolved:\n  {}",
            problems.join("\n  ")
        );
    }

    /// Every `.cfg` upstream ships — `config/*.cfg` and `test/klippy/*.cfg`, plus
    /// the configs the cases reference — must parse with our config parser.
    ///
    /// This is the corpus the parser has to read in the wild: multiline values,
    /// `=` and `:`, trailing comments on section headers, `;` inline comments,
    /// and so on. It is also the stage that exposes where we still differ from
    /// upstream's `configparser`; those diffs are listed in the failure message.
    #[test]
    fn every_upstream_printer_config_parses() {
        let mut files = printer_config_files();
        for run in &all_runs() {
            files.push(run.config.clone());
        }
        files.sort();
        // A case under `test/klippy` refers to the shipped configs as
        // `../../config/x.cfg`, so the same file arrives under two spellings.
        // Deduplicate by resolved path, keeping the first (sorted) spelling.
        let mut seen = std::collections::HashSet::new();
        let files: Vec<_> = files
            .into_iter()
            .filter(|path| {
                let key = std::fs::canonicalize(path).unwrap_or_else(|_| path.clone());
                seen.insert(key)
            })
            .collect();

        let mut failures = Vec::new();
        for path in &files {
            if let Err(error) = Config::from_file(path) {
                failures.push(format!("{}: {error}", relative(path)));
            }
        }

        assert!(
            failures.is_empty(),
            "{} of {} upstream configs do not parse:\n  {}",
            failures.len(),
            files.len(),
            failures.join("\n  ")
        );
    }

    /// Every run's **full** gap list, not just its first failure.
    ///
    /// `load_config` stops at the first unknown section, so
    /// [`upstream_test_cases_run`] only ever reports the first gap. This scans
    /// each run's config for *all* section ids this host does not know and all
    /// `kinematics:` values it does not implement, so the "one gap away" cases
    /// and the common prefixes are visible. It is a **report**: it prints the
    /// matrix and fails only if the corpus is empty. Run it with `--nocapture`:
    ///
    /// ```text
    /// cargo test -p klipperx --lib upstream_gap_report -- --nocapture
    /// ```
    #[test]
    fn upstream_gap_report() {
        use std::collections::{BTreeMap, BTreeSet};

        let known: BTreeSet<String> = crate::core::klippy::load::known_section_ids()
            .into_iter()
            .map(str::to_string)
            .collect();
        let supported_kinematics = [
            "none",
            "cartesian",
            "corexy",
            "corexz",
            "hybrid_corexy",
            "hybrid_corexz",
            "polar",
            "delta",
        ];

        let mut gap_frequency: BTreeMap<String, usize> = BTreeMap::new();
        let mut one_gap: Vec<String> = Vec::new();
        let mut scanned = 0usize;
        for run in all_runs() {
            let Ok((config, _)) = Config::from_file(&run.config) else {
                continue;
            };
            scanned += 1;
            let mut gaps: BTreeSet<String> = BTreeSet::new();
            for section in config.sections() {
                if section.id == "include" {
                    continue;
                }
                if !known.contains(&section.id) {
                    // A numbered sibling (`[stepper_z1]`) is read by its base
                    // section's owner, not by a factory of its own
                    // (`LookupMultiRail`); treat `<known><digits>` as known.
                    let base = section.id.trim_end_matches(|c: char| c.is_ascii_digit());
                    if base == section.id || !known.contains(base) {
                        gaps.insert(section.identifier());
                    }
                }
                if section.id == "printer" {
                    if let Some(kinematics) = section.get_text("kinematics") {
                        let kinematics = kinematics.trim();
                        if !supported_kinematics.contains(&kinematics) {
                            gaps.insert(format!("kinematics: {kinematics}"));
                        }
                    }
                }
            }
            for gap in &gaps {
                *gap_frequency.entry(gap.clone()).or_default() += 1;
            }
            if gaps.len() == 1 {
                let gap = gaps.iter().next().expect("one gap");
                one_gap.push(format!(
                    "  {} ({}): {gap}",
                    run.path
                        .file_name()
                        .map(|n| n.to_string_lossy().to_string())
                        .unwrap_or_default(),
                    relative(&run.config)
                ));
            }
        }
        assert!(scanned > 0, "no upstream runs were scanned");

        let mut common: Vec<(String, usize)> = gap_frequency.into_iter().collect();
        common.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.cmp(&b.0)));
        one_gap.sort();

        eprintln!("upstream gap report: {scanned} run(s) scanned");
        eprintln!("common gaps (gap: runs referencing it):");
        for (gap, count) in common.iter().take(20) {
            eprintln!("  {count:>4}  {gap}");
        }
        eprintln!("runs one static gap away ({}):", one_gap.len());
        for line in &one_gap {
            eprintln!("{line}");
        }
    }

    /// Guard: an `IGNORED` case that now passes must be removed from the list.
    ///
    /// The default run skips `IGNORED` before it tries anything, so a gap that
    /// gets fixed leaves a stale entry (and a green run that the suite hides).
    /// This runs every ignored case — its dictionaries permitting — and fails
    /// when **all** of a file's runs pass, which is the point at which the entry
    /// should be dropped.
    ///
    /// **Opt-in**: set `KLIPPERX_UPSTREAM_GUARD=1`. It is **load-only** (no
    /// connect, no g-code): a fake MCU never trips a homing endstop, so running
    /// a case's `G28` would block the synchronous dispatcher and leak the
    /// machine; and a case that loads may still fail at run time. It therefore
    /// reports candidates to verify, not proven-green cases.
    #[test]
    fn ignored_cases_still_fail() {
        use crate::core::klippy::printer::Printer;
        use crate::core::klippy::reactor::ManualReactor;

        if std::env::var_os("KLIPPERX_UPSTREAM_GUARD").is_none() {
            return;
        }
        let mut by_file: std::collections::BTreeMap<String, Vec<UpstreamRun>> =
            std::collections::BTreeMap::new();
        for run in all_runs() {
            let file = run
                .path
                .file_name()
                .map(|n| n.to_string_lossy().to_string())
                .unwrap_or_default();
            by_file.entry(file).or_default().push(run);
        }

        let mut stale = Vec::new();
        for (file, runs) in &by_file {
            if !IGNORED.contains(&file.as_str()) {
                continue;
            }
            let mut any_runnable = false;
            let mut all_load = true;
            for run in runs {
                let dictionaries = run_dictionaries(run);
                if dictionaries.iter().any(|(_, path)| path.is_none()) {
                    // A run without its dictionary cannot be judged either way.
                    all_load = false;
                    continue;
                }
                any_runnable = true;
                let mut resolved = Vec::with_capacity(dictionaries.len());
                for (mcu, path) in &dictionaries {
                    if let Some(path) = path {
                        resolved.push((mcu.clone(), path.clone()));
                    }
                }
                let loaded = match injected_config(&run.config, &resolved) {
                    Ok(config) => {
                        // `ManualReactor` runs no tasks: `load_config` alone
                        // opens nothing and spawns nothing.
                        let printer = Arc::new(Printer::new(ManualReactor::shared()));
                        printer.load_config(&config).is_ok()
                    }
                    Err(_) => false,
                };
                if !loaded {
                    all_load = false;
                    break;
                }
            }
            if any_runnable && all_load {
                stale.push(file.clone());
            }
        }

        assert!(
            stale.is_empty(),
            "{} IGNORED case(s) now load; verify their g-code and remove them from IGNORED: {stale:?}",
            stale.len()
        );
    }

    /// Parse the inline g-code of every case with our own g-code parser.
    ///
    /// Pending: a case's g-code is written for the config it names, and most
    /// names sections we do not implement (`[printer]`, kinematics, the extras),
    /// so the parser cannot be driven through the dispatcher until those exist.
    /// The argument list is kept here so the test is a next step, not lost work.
    #[test]
    #[ignore = "needs the config sections the cases use, to build a dispatcher"]
    fn upstream_inline_gcode_parses() {
        let runs: Vec<_> = all_runs()
            .into_iter()
            .filter(|run| !run.gcode_lines.is_empty())
            .collect();
        assert!(
            !runs.is_empty(),
            "no upstream run carries inline g-code to parse"
        );
        panic!(
            "not implemented: {} runs carry inline g-code, but parsing it needs the \
             config sections they name; see the module docs for the stages",
            runs.len()
        );
    }

    // -----------------------------------------------------------------------
    // Running cases against the dictionary-driven fake MCU
    // -----------------------------------------------------------------------

    /// Cases that cannot pass yet, skipped unless `KLIPPERX_UPSTREAM_ALL` is set.
    ///
    /// Nearly every upstream config names sections this host does not implement
    /// (`gcode_macro`, `probe`, `tmc*`, `display`, …), so `load_config`
    /// rejects them before any g-code runs. The list shrinks as those sections
    /// land.
    ///
    /// `KLIPPERX_UPSTREAM_ALL=1` runs every case and reports every failure, so
    /// the list stays honest rather than hiding regressions.
    const IGNORED: &[&str] = &[
        "generic_cartesian_iqex.test",
        "generic_cartesian_itex.test",
        "load_cell.test",
        "printers.test",
        "rotary_delta_calibrate.test",
        "tmc.test",
    ];

    // -----------------------------------------------------------------------
    // Which architectures and dictionaries to run
    // -----------------------------------------------------------------------

    /// The directory of dictionaries `build.rs` built.
    fn dict_dir() -> PathBuf {
        klipperx_test_support::test_dicts_dir()
    }

    /// The dictionary a case names, when it was built.
    ///
    /// `build.rs` builds one `test/configs/*.config` per architecture in
    /// `KLIPPERX_ARCHES` and collects each as the same-named `<name>.dict`; a
    /// name outside that set has no dictionary, and the case cannot run.
    fn dictionary_path(name: &str) -> Option<PathBuf> {
        let stem = name.strip_suffix(".dict").unwrap_or(name);
        let candidate = dict_dir().join(format!("{stem}.dict"));
        candidate.is_file().then_some(candidate)
    }

    /// A parsed config with every `[mcu]` transport replaced by a fake firmware.
    ///
    /// `dictionaries` gives the data dictionary for each named MCU (`None` for
    /// the bare `[mcu]`); the caller resolves all of them for the case before
    /// this runs, so the first entry is always the fallback.
    fn injected(config: &Config, dictionaries: &[(Option<String>, PathBuf)]) -> Config {
        let transport_keys = [
            "serial",
            "baud",
            "canbus_uuid",
            "canbus_interface",
            "canbus_nodeid",
            "host_library",
            "test",
        ];
        let fallback = &dictionaries[0].1;
        let mut out = Config::new();
        for section in config.sections_vec() {
            let mut section = section.clone();
            if section.id == "mcu" {
                let dictionary = dictionaries
                    .iter()
                    .find(|(mcu, _)| mcu.as_deref() == section.sub.as_deref())
                    .map(|(_, path)| path)
                    .unwrap_or(fallback);
                for key in transport_keys {
                    section.parameters.remove(key);
                }
                section.parameters.insert(
                    "test".to_string(),
                    ConfigValue::Single(format!("dict={}", dictionary.display())),
                );
            }
            out.add_section(section);
        }
        out
    }

    /// Parse the config at `path` and inject the fake firmware's transport.
    fn injected_config(
        path: &Path,
        dictionaries: &[(Option<String>, PathBuf)],
    ) -> Result<Config, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let (config, _) =
            Config::from_text(&text).map_err(|e| format!("{}: {e}", path.display()))?;
        Ok(injected(&config, dictionaries))
    }

    /// Load `config`, bring the machine up, and run `script` through the
    /// ordinary g-code dispatcher.
    ///
    /// `config_file` names the config in the start arguments the case gets —
    /// upstream's `klippy.py` argument, which is what `info` would report.
    ///
    /// The two phases are reported separately, because a `SHOULD_FAIL` run may
    /// only be satisfied by the second: an outer `Err` means the machine could
    /// not be brought up at all — a gap in this host (a missing section, say),
    /// never the failure the run is about — while `Ok(Err(..))` means it ran and
    /// the g-code failed, which is what such a run expects.
    ///
    /// The printer is torn down before returning: the config's parts hold the
    /// device open, and a receive task parked on it would keep the test runtime
    /// from shutting down.
    async fn run_phases(
        config: &Config,
        config_file: &str,
        script: &str,
    ) -> Result<Result<(), String>, String> {
        use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
        use crate::core::klippy::printer::{Printer, PrinterState};
        use crate::core::klippy::reactor::TokioReactor;

        let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
        let printer = Arc::new(Printer::new(reactor));

        // Upstream runs every case as `klippy.py -i <gcode> -o <output> -d
        // <dict>` (`scripts/test_klippy.py:100-104`). The `-o` lands in
        // `start_args['debugoutput']`, and `heaters.py:38-39` reads it: a case
        // never answers its temperature queries, so `can_extrude` starts true
        // and the `G1 E…` lines of a case's g-code are not rejected as cold.
        // The host builds the same dictionary at startup (`src/klippy.rs`);
        // this is that step for a case run, done before the config is loaded,
        // as the host does.
        let mut start_args = crate::core::klippy::api::StartArgs::collect(config_file, None);
        start_args.debug_output = Some("_test_output".to_string());
        printer.set_start_args(Arc::new(start_args));

        let setup = async {
            printer.load_config(config).map_err(|e| e.to_string())?;
            // A fake firmware answers at once, so a wait here means the
            // exchange is stuck; give up instead of hanging the test run.
            if tokio::time::timeout(std::time::Duration::from_secs(10), printer.bring_up())
                .await
                .is_err()
            {
                return Err("bring_up timed out".to_string());
            }
            let state = printer.get_state_message();
            if state.category != PrinterState::Ready {
                return Err(format!("not ready: {}", state.message));
            }
            Ok(())
        }
        .await;

        let gcode = if setup.is_ok() {
            match printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) {
                Some(dispatcher) => match dispatcher.run_script(script).await {
                    Ok(()) => {
                        let state = printer.get_state_message();
                        if state.category == PrinterState::Ready {
                            Ok(())
                        } else {
                            Err(format!("left ready: {}", state.message))
                        }
                    }
                    Err(e) => Err(e.to_string()),
                },
                None => Err("the g-code dispatcher is not registered".to_string()),
            }
        } else {
            // Unused: `setup` decides the result below.
            Ok(())
        };

        printer.teardown();
        match setup {
            Err(e) => Err(e),
            Ok(()) => Ok(gcode),
        }
    }

    /// Both phases have to succeed; for the minimal case, which asserts a clean
    /// run rather than an inverted expectation.
    async fn run_script_on(config: &Config, config_file: &str, script: &str) -> Result<(), String> {
        run_phases(config, config_file, script).await?
    }

    /// Every dictionary a run names, and whether it was built.
    fn run_dictionaries(run: &UpstreamRun) -> Vec<(Option<String>, Option<PathBuf>)> {
        run.dictionaries
            .iter()
            .map(|dictionary| (dictionary.mcu.clone(), dictionary_path(&dictionary.file)))
            .collect()
    }

    /// Run one upstream run, as `test_klippy.py` does.
    ///
    /// `dictionaries` pairs each named MCU with the dictionary to serve it; a
    /// missing one is an error — running the run with a different target's
    /// dictionary would not be the run upstream wrote.
    ///
    /// `SHOULD_FAIL` inverts the result of the **g-code phase only**: a config
    /// that cannot be loaded is a gap in this host, so it is reported as a
    /// failure rather than quietly satisfying the expectation. Upstream can
    /// treat every non-zero exit as success because it implements everything.
    async fn run_case(
        run: &UpstreamRun,
        dictionaries: &[(Option<String>, Option<PathBuf>)],
    ) -> Result<(), String> {
        let mut resolved = Vec::with_capacity(dictionaries.len());
        for (mcu, path) in dictionaries {
            let Some(path) = path else {
                return Err(format!(
                    "dictionary for '{}' was not built",
                    mcu.as_deref().unwrap_or("mcu")
                ));
            };
            resolved.push((mcu.clone(), path.clone()));
        }

        let script = match &run.gcode_file {
            Some(path) => {
                std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?
            }
            None => run.gcode_lines.join("\n"),
        };

        let parsed = injected_config(&run.config, &resolved)?;
        let case_file = run.config.display().to_string();
        let gcode = match run_phases(&parsed, &case_file, &script).await {
            Err(setup) => return Err(format!("{}: {setup}", relative(&run.config))),
            Ok(gcode) => gcode,
        };

        match (run.should_fail, gcode) {
            (false, outcome) => outcome,
            (true, Err(_)) => Ok(()),
            (true, Ok(())) => Err("the run was expected to fail".to_string()),
        }
    }

    /// A name resolves exactly when its dictionary was built; nothing resolves
    /// by borrowing another target's dictionary.
    ///
    /// Which names exist depends on `KLIPPERX_ARCHES`, so the test reads the
    /// directory rather than naming a target.
    #[test]
    fn a_dictionary_resolves_exactly_when_it_was_built() {
        let entry = std::fs::read_dir(dict_dir())
            .expect("the dictionary directory")
            .next()
            .expect("at least one dictionary is built")
            .expect("a directory entry");
        let name = entry.file_name().to_string_lossy().to_string();

        assert_eq!(dictionary_path(&name), Some(dict_dir().join(&name)));
        assert_eq!(dictionary_path("definitely-not-a-built-target.dict"), None);
    }

    /// A `SHOULD_FAIL` run is only satisfied by the g-code phase: a config this
    /// host cannot load is a gap in the host, reported as a failure rather than
    /// as the expected one (see [`run_case`]).
    #[tokio::test(flavor = "multi_thread")]
    async fn a_should_fail_run_is_not_satisfied_by_a_config_that_does_not_load() {
        let run = UpstreamRun {
            path: klippy_test_dir().join("linuxtest.test"),
            config: klippy_test_dir().join("linuxtest.cfg"),
            dictionaries: vec![Dictionary {
                mcu: None,
                file: "linuxprocess.dict".to_string(),
            }],
            gcode_file: None,
            gcode_lines: vec!["G4 P1000".to_string()],
            should_fail: true,
        };

        let result = run_case(&run, &run_dictionaries(&run)).await;
        assert!(
            result.is_err(),
            "a config that does not load must not satisfy SHOULD_FAIL"
        );
    }

    /// The minimal case that needs only `[mcu]`: it exercises identify, the
    /// configuration handshake, and the g-code dispatcher without any section
    /// this host has not implemented.
    #[tokio::test(flavor = "multi_thread")]
    async fn a_minimal_config_runs_against_the_fake_firmware() {
        let dictionary = dict_dir().join("linuxprocess.dict");
        let text = format!("[mcu]\ntest: dict={}\n", dictionary.display());
        let (config, _) = Config::from_text(&text).expect("the minimal config parses");

        run_script_on(&config, "<case>", "M115")
            .await
            .expect("a minimal case runs against the fake firmware");
    }

    /// An extruder move runs end to end: the E axis has its own trapq and its
    /// stepper, and a `G1` with an `E` word drives both the kinematic axes and
    /// the extruder.
    #[tokio::test(flavor = "multi_thread")]
    async fn an_extruder_move_runs_against_the_fake_firmware() {
        let dictionary = dict_dir().join("atmega2560.dict");
        let text = format!(
            "[mcu]\ntest: dict={}\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_y]\nstep_pin: PA2\ndir_pin: PA3\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\n\
             [stepper_z]\nstep_pin: PA4\ndir_pin: PA5\nrotation_distance: 8\nmicrosteps: 16\nposition_max: 200\n\
             [extruder]\nstep_pin: PA6\ndir_pin: PA7\nrotation_distance: 33.5\nmicrosteps: 16\n\
             nozzle_diameter: 0.4\nfilament_diameter: 1.75\nheater_pin: PB0\n\
             sensor_type: EPCOS 100K B57560G104F\nsensor_pin: PK5\ncontrol: pid\npid_Kp: 1\npid_Ki: 0.1\npid_Kd: 10\n\
             min_temp: 0\nmax_temp: 250\nmin_extrude_temp: 0\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n",
            dictionary.display()
        );
        let (config, _) = Config::from_text(&text).expect("the extruder config parses");

        run_script_on(
            &config,
            "<case>",
            "SET_KINEMATIC_POSITION X=0 Y=0 Z=0\nG1 X10 Y10 F600\nG1 E1 F300\nM400",
        )
        .await
        .expect("the extruder move runs against the fake firmware");
    }

    /// A homing move runs end to end and the process exits cleanly.
    ///
    /// Regression for the `Mcu → events → resource → Mcu` strong cycle: before
    /// `McuObject` cleared its callbacks on drop, `Mcu::Drop` never ran, its
    /// blocking device read parked, and the test runtime hung at shutdown.
    #[tokio::test(flavor = "multi_thread")]
    async fn a_homing_move_runs_against_the_fake_firmware() {
        let dictionary = dict_dir().join("atmega2560.dict");
        let text = format!(
            "[mcu]\ntest: dict={}\n\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\nendstop_pin: ^PA2\n\
             [stepper_y]\nstep_pin: PA3\ndir_pin: PA4\nrotation_distance: 40\nmicrosteps: 16\nposition_max: 200\nendstop_pin: ^PA5\n\
             [stepper_z]\nstep_pin: PA6\ndir_pin: PA7\nrotation_distance: 8\nmicrosteps: 16\nposition_max: 200\nendstop_pin: ^PB0\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n",
            dictionary.display()
        );
        let (config, _) = Config::from_text(&text).expect("the homing config parses");

        run_script_on(&config, "<case>", "G28")
            .await
            .expect("G28 runs against the fake firmware");
    }

    /// Run the upstream runs that can be run: those whose dictionaries were all
    /// built and that are not on the ignore list.
    /// Which dictionaries exist is decided at build time by `KLIPPERX_ARCHES`
    /// (default `linux`). `KLIPPERX_UPSTREAM_ALL=1` bypasses the ignore list, so
    /// every run with its dictionaries reports its failures.
    #[tokio::test(flavor = "multi_thread")]
    async fn upstream_test_cases_run() {
        let all = std::env::var_os("KLIPPERX_UPSTREAM_ALL").is_some();

        let mut ran = 0usize;
        let mut no_dictionary = 0usize;
        let mut unbuilt = Vec::new();
        let mut ignored = 0usize;
        let mut failures = Vec::new();
        for run in all_runs() {
            let file = run
                .path
                .file_name()
                .map(|n| n.to_string_lossy().to_string())
                .unwrap_or_default();
            let name = format!("{file} ({})", relative(&run.config));

            // A run without a dictionary cannot be started the way upstream
            // starts one; upstream itself refuses to.
            if run.dictionaries.is_empty() {
                no_dictionary += 1;
                continue;
            }

            let dictionaries = run_dictionaries(&run);
            let missing: Vec<&str> = dictionaries
                .iter()
                .filter(|(_, path)| path.is_none())
                .map(|(mcu, _)| mcu.as_deref().unwrap_or("mcu"))
                .collect();
            // A run without its dictionaries cannot be started at all, so this
            // is not something `KLIPPERX_UPSTREAM_ALL` bypasses — enabling more
            // architectures at build time is what makes these runs possible.
            if !missing.is_empty() {
                unbuilt.push(format!("{name} needs {}", missing.join(", ")));
                continue;
            }
            if !all && IGNORED.contains(&file.as_str()) {
                ignored += 1;
                continue;
            }

            match run_case(&run, &dictionaries).await {
                Ok(()) => ran += 1,
                Err(e) => failures.push(format!("{name}: {e}")),
            }
        }

        assert!(
            failures.is_empty(),
            "{} upstream run(s) failed ({} ran, {} without a dictionary, {} with \
             an unbuilt dictionary, {} ignored; KLIPPERX_UPSTREAM_ALL=1 drops the \
             ignore list, KLIPPERX_ARCHES/KLIPPERX_ALL_ARCHES build more \
             dictionaries):\n  {}",
            failures.len(),
            ran,
            no_dictionary,
            unbuilt.len(),
            ignored,
            failures.join("\n  ")
        );
    }

    // ---------------------------------------------------------------------
    // Focused end-to-end checks: does an endstop actually reach a homing move
    // on the fake MCU? The corpus never exercises this on its own (every config
    // that homes fails earlier on a missing section), so `G28` is pinned here —
    // first on a plain MCU endstop, then through `probe:z_virtual_endstop`.
    // ---------------------------------------------------------------------

    /// A minimal cartesian printer whose `[stepper_z]` uses `z_endstop_pin`.
    fn homing_config(dict: &Path, z_endstop_pin: &str, probe: bool, extra: &str) -> Config {
        let probe_section = if probe {
            "[probe]\npin: ^PC3\nz_offset: 1.0\n"
        } else {
            ""
        };
        let text = format!(
            "[mcu]\ntest: dict={dict}\n\
             [printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n\
             max_z_velocity: 15\nmax_z_accel: 100\n\
             {probe_section}\
             [stepper_x]\nstep_pin: PA0\ndir_pin: PA1\nrotation_distance: 40\nmicrosteps: 16\n\
             endstop_pin: ^PA2\nposition_endstop: 0\nposition_min: 0\nposition_max: 200\nhoming_speed: 50\n\
             [stepper_y]\nstep_pin: PB0\ndir_pin: PB1\nrotation_distance: 40\nmicrosteps: 16\n\
             endstop_pin: ^PB2\nposition_endstop: 0\nposition_min: 0\nposition_max: 200\nhoming_speed: 50\n\
             [stepper_z]\nstep_pin: PC0\ndir_pin: PC1\nrotation_distance: 40\nmicrosteps: 16\n\
             endstop_pin: {z_endstop_pin}\nposition_endstop: 0.5\nposition_min: 0\nposition_max: 200\nhoming_speed: 10\n\
             {extra}",
            dict = dict.display(),
            extra = extra,
        );
        Config::from_text(&text)
            .expect("the focused config parses")
            .0
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn a_plain_endstop_reaches_the_homing_move() {
        let Some(dict) = dictionary_path("atmega2560.dict") else {
            return;
        };
        let config = homing_config(&dict, "^PC2", false, "");

        let gcode = run_phases(&config, "focused-plain-home.cfg", "G28 Z\n")
            .await
            .expect("the machine comes up");
        assert!(gcode.is_ok(), "{gcode:?}");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn probe_calibrate_and_accept_complete_on_the_fake_mcu() {
        let Some(dict) = dictionary_path("atmega2560.dict") else {
            return;
        };
        let config = homing_config(&dict, "probe:z_virtual_endstop", true, "");

        let gcode = run_phases(
            &config,
            "focused-probe-calibrate.cfg",
            "G28\nPROBE_CALIBRATE\nTESTZ Z=-1\nACCEPT\n",
        )
        .await
        .expect("the machine comes up");
        assert!(gcode.is_ok(), "{gcode:?}");
    }

    /// The corpus's own `bed_mesh.cfg` end to end: 49 probe points, which is
    /// where the 32-bit trigger clock used to be mapped a whole revolution
    /// away (the arm clock is print time, far ahead of the fake MCU's wall
    /// clock, and the mapping's reference had to be this move's arming time).
    /// The corpus's `z_virtual_endstop.cfg` end to end, for bisecting the
    /// "probe triggered prior to movement" it reported.
    #[tokio::test(flavor = "multi_thread")]
    async fn the_corpus_z_virtual_endstop_config_runs() {
        let Some(dict) = dictionary_path("atmega2560.dict") else {
            return;
        };
        let config = injected_config(
            &klippy_test_dir().join("z_virtual_endstop.cfg"),
            &[(None, dict)],
        )
        .expect("the corpus config parses");

        let gcode = run_phases(
            &config,
            "focused-corpus-z-virtual.cfg",
            "G28\nBED_MESH_CALIBRATE\nG1 Z5 X0 Y0\nPROBE\n",
        )
        .await
        .expect("the machine comes up");
        assert!(gcode.is_ok(), "{gcode:?}");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_corpus_bed_mesh_config_calibrates() {
        let Some(dict) = dictionary_path("atmega2560.dict") else {
            return;
        };
        let config = injected_config(&klippy_test_dir().join("bed_mesh.cfg"), &[(None, dict)])
            .expect("the corpus config parses");

        let gcode = run_phases(
            &config,
            "focused-corpus-bed-mesh.cfg",
            "G28\nG1 F6000\nG1 X60 Y60 Z10\nBED_MESH_CALIBRATE\nG1 Z10\n",
        )
        .await
        .expect("the machine comes up");
        assert!(gcode.is_ok(), "{gcode:?}");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn a_bed_mesh_calibration_completes_on_the_fake_mcu() {
        let Some(dict) = dictionary_path("atmega2560.dict") else {
            return;
        };
        let config = homing_config(
            &dict,
            "probe:z_virtual_endstop",
            true,
            "[bed_mesh]\nmesh_min: 10,10\nmesh_max: 60,60\nprobe_count: 3,3\n",
        );

        let gcode = run_phases(&config, "focused-bed-mesh.cfg", "G28\nBED_MESH_CALIBRATE\n")
            .await
            .expect("the machine comes up");
        assert!(gcode.is_ok(), "{gcode:?}");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn a_probe_command_completes_on_the_fake_mcu() {
        let Some(dict) = dictionary_path("atmega2560.dict") else {
            return;
        };
        let config = homing_config(&dict, "probe:z_virtual_endstop", true, "");

        let gcode = run_phases(&config, "focused-probe-command.cfg", "G28\nPROBE\n")
            .await
            .expect("the machine comes up");
        assert!(gcode.is_ok(), "{gcode:?}");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn the_probe_virtual_endstop_reaches_the_homing_move() {
        let Some(dict) = dictionary_path("atmega2560.dict") else {
            return;
        };
        let config = homing_config(&dict, "probe:z_virtual_endstop", true, "");

        let gcode = run_phases(&config, "focused-probe-home.cfg", "G28 Z\n")
            .await
            .expect("the machine comes up");
        assert!(gcode.is_ok(), "{gcode:?}");
    }

    /// The corpus `corexyuv.cfg` homing, bounded: `G28` on the
    /// generic-cartesian printer must arm and **fire** every axis' trsync and
    /// the move after it must run (U-GC-3). Before the fix the Z homing move
    /// was built with a zero-length profile — the kinematics was constructed
    /// with `max_z_velocity = 0` — so no step was queued, the fake firmware
    /// never fired the armed trsync, and the host waited forever while
    /// retransmitting the arm block (the "infinite arm, never fire" log).
    ///
    /// Only the homing part of `corexyuv.test`'s script runs here; the whole
    /// case, dual-carriage and extruder segments included, runs in
    /// [`the_corexyuv_case_runs_every_gcode_line`].
    #[tokio::test(flavor = "multi_thread")]
    async fn the_corexyuv_config_homes_against_the_fake_firmware() {
        let Some(dict) = dictionary_path("atmega2560.dict") else {
            return;
        };
        let run = all_runs()
            .into_iter()
            .find(|run| {
                run.path
                    .file_name()
                    .map(|name| name == "corexyuv.test")
                    .unwrap_or(false)
            })
            .expect("the corpus carries corexyuv.test");
        let config = injected_config(&run.config, &[(None, dict)]).expect("corexyuv.cfg parses");

        let outcome = tokio::time::timeout(
            std::time::Duration::from_secs(60),
            run_phases(&config, "corexyuv.cfg", "G90\nG28\nG1 X10 Y20 F6000\n"),
        )
        .await;
        let gcode = outcome.expect(
            "generic-cartesian homing finishes instead of waiting on a trsync that never fires",
        );
        assert!(gcode.is_ok(), "{gcode:?}");
    }

    /// The same chain on the smallest printer that can show it: three
    /// `[carriage]` sections, one motor each, and a `G28 Z` — no corpus
    /// section other than the Z rail is involved.
    #[tokio::test(flavor = "multi_thread")]
    async fn a_generic_cartesian_z_home_fires_the_trsync() {
        let Some(dict) = dictionary_path("atmega2560.dict") else {
            return;
        };
        let text = format!(
            "[mcu]\ntest: dict={}\n\
             [printer]\nkinematics: generic_cartesian\nmax_velocity: 300\nmax_accel: 3000\n\
             max_z_velocity: 5\nmax_z_accel: 100\n\
             [carriage carriage_x]\naxis: x\nposition_endstop: 0\nposition_max: 300\n\
             homing_speed: 50\nendstop_pin: ^PE5\n\
             [carriage carriage_y]\naxis: y\nposition_endstop: 0\nposition_max: 200\n\
             homing_speed: 50\nendstop_pin: ^PJ1\n\
             [carriage carriage_z]\naxis: z\nposition_endstop: 0.5\nposition_max: 100\n\
             homing_speed: 5\nendstop_pin: ^PD3\n\
             [stepper a]\ncarriages: carriage_x\nstep_pin: PF0\ndir_pin: PF1\n\
             enable_pin: !PD7\nmicrosteps: 16\nrotation_distance: 40\n\
             [stepper b]\ncarriages: carriage_y\nstep_pin: PH1\ndir_pin: PH0\n\
             enable_pin: !PA1\nmicrosteps: 16\nrotation_distance: 40\n\
             [stepper z]\ncarriages: carriage_z\nstep_pin: PL3\ndir_pin: PL1\n\
             enable_pin: !PK0\nmicrosteps: 16\nrotation_distance: 8\n",
            dict.display()
        );
        let (config, _) = Config::from_text(&text).expect("the generic-cartesian config parses");

        let outcome = tokio::time::timeout(
            std::time::Duration::from_secs(60),
            run_phases(&config, "focused-gc-z.cfg", "G28 Z\n"),
        )
        .await;
        let gcode = outcome.expect(
            "the Z homing move fires its trsync instead of waiting on one that never fires",
        );
        assert!(gcode.is_ok(), "{gcode:?}");
    }

    /// The whole `corexyuv.test` script, line by line on one live machine,
    /// bounded — the case `IGNORED` still lists only because the guard stays
    /// load-only.
    ///
    /// Every line runs through the ordinary dispatcher against the fake
    /// firmware and is reported, so a regression names its command rather
    /// than the case: the U-GC-4 failure stopped at `G91` + `G1 X-10 E.2`
    /// with `Move out of range: -10.000 …`, because a dual carriage's frame
    /// never learned where homing left it (`idex_modes::Shared::homed`) and
    /// the switch onto `carriage_u` re-anchored the toolhead at X=0.
    #[tokio::test(flavor = "multi_thread")]
    async fn the_corexyuv_case_runs_every_gcode_line() {
        use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
        use crate::core::klippy::printer::{Printer, PrinterState};
        use crate::core::klippy::reactor::TokioReactor;

        let Some(dict) = dictionary_path("atmega2560.dict") else {
            return;
        };
        let run = all_runs()
            .into_iter()
            .find(|run| {
                run.path
                    .file_name()
                    .map(|name| name == "corexyuv.test")
                    .unwrap_or(false)
            })
            .expect("the corpus carries corexyuv.test");
        let config = injected_config(&run.config, &[(None, dict)]).expect("corexyuv.cfg parses");

        let outcome = tokio::time::timeout(std::time::Duration::from_secs(60), async {
            let reactor = Arc::new(TokioReactor::new(tokio::runtime::Handle::current()));
            let printer = Arc::new(Printer::new(reactor));
            let mut start_args = crate::core::klippy::api::StartArgs::collect("corexyuv.cfg", None);
            start_args.debug_output = Some("_test_output".to_string());
            printer.set_start_args(Arc::new(start_args));
            printer.load_config(&config).expect("corexyuv.cfg loads");
            tokio::time::timeout(std::time::Duration::from_secs(30), printer.bring_up())
                .await
                .expect("bring_up finishes");
            let state = printer.get_state_message();
            assert_eq!(state.category, PrinterState::Ready, "{state:?}");
            let dispatcher = printer
                .lookup_object_as::<GCodeDispatch>(GCODE_OBJECT)
                .expect("the g-code dispatcher is registered");
            let mut failed = None;
            for line in &run.gcode_lines {
                match dispatcher.run_script(line).await {
                    Ok(()) => eprintln!("OK   | {line}"),
                    Err(e) => {
                        eprintln!("FAIL | {line} -> {e}");
                        failed = Some(format!("{line} -> {e}"));
                        break;
                    }
                }
            }
            printer.teardown();
            failed
        })
        .await;
        let failed = outcome.expect("the full script finishes inside the bound");
        assert!(
            failed.is_none(),
            "corexyuv.test failed at: {}",
            failed.unwrap()
        );
    }
}

use std::fs;
use std::path::{Path, PathBuf};

/// The repository root (the package this crate is).
fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// The klipper checkout the corpus is read from.
///
/// [`klipperx_test_support::klipper_dir`] resolves it once for the whole
/// test-support crate (and `build.rs`), including the `KLIPPERX_KLIPPER_DIR`
/// override that lets a git worktree point at the main checkout instead of
/// carrying its own copy of the submodule.
fn klipper_dir() -> PathBuf {
    klipperx_test_support::klipper_dir()
}

/// Upstream's `<klipper>/test/klippy` — the `.test` cases and their configs.
fn klippy_test_dir() -> PathBuf {
    klipper_dir().join("test/klippy")
}

/// Upstream's `<klipper>/config` — the configs shipped to users.
fn printer_config_dir() -> PathBuf {
    klipper_dir().join("config")
}

/// Upstream's `<klipper>/test/configs` — the kconfig fragments each MCU
/// dictionary is built from.
fn mcu_config_dir() -> PathBuf {
    klipper_dir().join("test/configs")
}

/// One `DICTIONARY` entry: the firmware dictionary a case runs against.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Dictionary {
    /// The MCU it describes. `None` is the main MCU; `Some("zboard")` came from
    /// a `zboard=z.dict` entry for a secondary one.
    mcu: Option<String>,
    /// The file name as the case writes it (`atmega2560.dict`).
    file: String,
}

/// One run: a `CONFIG` block, as `test_klippy.py` launches it.
///
/// A `.test` file is a *sequence* of runs — one per `CONFIG` block, 239 across
/// the corpus (`printers.test` alone opens 203). The `DICTIONARY` line in effect
/// switches at the directive and stays until the next one, so it groups the
/// following runs by MCU target; the inline g-code, a `GCODE` file and
/// `SHOULD_FAIL` are shared across the whole file, as upstream's parser keeps
/// them in variables that only ever grow.
#[derive(Debug, Clone)]
struct UpstreamRun {
    /// The `.test` file this run came from.
    path: PathBuf,
    /// The config this run loads.
    config: PathBuf,
    /// The dictionaries in effect when the run is launched.
    dictionaries: Vec<Dictionary>,
    /// The `GCODE` file, when the file names one instead of inline lines.
    gcode_file: Option<PathBuf>,
    /// The inline g-code accumulated so far.
    gcode_lines: Vec<String>,
    /// Whether `SHOULD_FAIL` had been seen when the run is launched.
    should_fail: bool,
}

/// Every `*.test` file, sorted.
fn test_files() -> Vec<PathBuf> {
    let mut files = Vec::new();
    for entry in fs::read_dir(klippy_test_dir()).expect("upstream test/klippy is readable") {
        let path = entry.expect("a directory entry").path();
        if path.extension().is_some_and(|e| e == "test") {
            files.push(path);
        }
    }
    files.sort();
    files
}

/// Read one `.test` file into its runs.
///
/// The grammar is upstream's (`scripts/test_klippy.py:32-64`): strip `#`
/// comments, split on whitespace, and dispatch on the first word. `CONFIG` and
/// `GCODE` paths are relative to the `.test` file; `DICTIONARY` names a file
/// (qualified `mcu=file` for a secondary MCU), not a path here — see
/// [`dictionary_source`].
///
/// A run is launched at the **next** `CONFIG`, or at the end of the file — which
/// is upstream's rule, and why a file may put `CONFIG` before its `DICTIONARY`
/// (24 of them do): the dictionary is read by the time the run starts.
///
/// # Errors
/// A line count and message for a directive that is missing its argument, or a
/// secondary `DICTIONARY` entry without `=`.
fn parse_test_file(path: &Path) -> Result<Vec<UpstreamRun>, String> {
    let text =
        fs::read_to_string(path).map_err(|e| format!("failed to read {}: {e}", path.display()))?;
    let dir = path.parent().unwrap_or(Path::new("."));

    let mut runs: Vec<UpstreamRun> = Vec::new();
    let mut dictionaries: Vec<Dictionary> = Vec::new();
    let mut gcode_file = None;
    let mut gcode_lines: Vec<String> = Vec::new();
    let mut should_fail = false;
    // The `CONFIG` seen most recently with the dictionary state it owns, waiting
    // for the next `CONFIG` (or the end of the file) to be pushed. Fixtures write
    // the directive both ways: `printers.test` puts `DICTIONARY` before its
    // group, while `bed_mesh.test` and friends put `CONFIG` first — a
    // `DICTIONARY` written after a `CONFIG` belongs to that pending run, one
    // written before applies to the runs that follow. `None` means "no dictionary
    // yet, a following `DICTIONARY` may claim this run".
    let mut pending: Option<(PathBuf, Option<Vec<Dictionary>>)> = None;

    for (index, raw) in text.lines().enumerate() {
        let line = match raw.find('#') {
            Some(pos) => &raw[..pos],
            None => raw,
        };
        let parts: Vec<&str> = line.split_whitespace().collect();
        let Some((directive, args)) = parts.split_first() else {
            continue;
        };
        let line_no = index + 1;
        match *directive {
            "CONFIG" => {
                let arg = args
                    .first()
                    .ok_or_else(|| format!("{}:{line_no}: CONFIG needs a path", path.display()))?;
                if let Some((config, snapshot)) = pending.take() {
                    runs.push(UpstreamRun {
                        path: path.to_path_buf(),
                        config,
                        dictionaries: snapshot.unwrap_or_else(|| dictionaries.clone()),
                        gcode_file: gcode_file.clone(),
                        gcode_lines: gcode_lines.clone(),
                        should_fail,
                    });
                }
                let snapshot = (!dictionaries.is_empty()).then(|| dictionaries.clone());
                pending = Some((dir.join(arg), snapshot));
            }
            "DICTIONARY" => {
                let main = args.first().ok_or_else(|| {
                    format!("{}:{line_no}: DICTIONARY needs a file", path.display())
                })?;
                let mut entries = vec![Dictionary {
                    mcu: None,
                    file: (*main).to_string(),
                }];
                for spec in &args[1..] {
                    let (mcu, file) = spec.split_once('=').ok_or_else(|| {
                        format!(
                            "{}:{line_no}: secondary DICTIONARY '{spec}' is not name=file",
                            path.display()
                        )
                    })?;
                    entries.push(Dictionary {
                        mcu: Some(mcu.to_string()),
                        file: file.to_string(),
                    });
                }
                dictionaries = entries;
                // A `DICTIONARY` written after a `CONFIG` belongs to that pending
                // run; one written before it belongs to the following runs only.
                if let Some((_, snapshot)) = pending.as_mut() {
                    if snapshot.is_none() {
                        *snapshot = Some(dictionaries.clone());
                    }
                }
            }
            "GCODE" => {
                let arg = args
                    .first()
                    .ok_or_else(|| format!("{}:{line_no}: GCODE needs a path", path.display()))?;
                gcode_file = Some(dir.join(arg));
            }
            "SHOULD_FAIL" => should_fail = true,
            _ => gcode_lines.push(line.trim().to_string()),
        }
    }

    if let Some((config, snapshot)) = pending {
        runs.push(UpstreamRun {
            path: path.to_path_buf(),
            config,
            dictionaries: snapshot.unwrap_or(dictionaries),
            gcode_file,
            gcode_lines,
            should_fail,
        });
    }
    Ok(runs)
}

/// Every run in the corpus. Panics on a malformed file, naming it.
fn all_runs() -> Vec<UpstreamRun> {
    test_files()
        .into_iter()
        .flat_map(|path| parse_test_file(&path).unwrap_or_else(|e| panic!("upstream fixture: {e}")))
        .collect()
}

/// The kconfig fragment a dictionary is built from, when upstream ships one.
///
/// Upstream's CI compiles `test/configs/*.config` into `out/klipper.dict` and
/// copies it to the dictionary directory; only the fragment is in the tree, so
/// that is what can be resolved here.
fn dictionary_source(dictionary: &Dictionary) -> Option<PathBuf> {
    let stem = dictionary.file.strip_suffix(".dict")?;
    let config = mcu_config_dir().join(format!("{stem}.config"));
    config.is_file().then_some(config)
}

/// Every `.cfg` upstream ships as a fixture: `config/*.cfg` plus
/// `test/klippy/*.cfg`, deduplicated and sorted.
fn printer_config_files() -> Vec<PathBuf> {
    let mut files = Vec::new();
    for dir in [printer_config_dir(), klippy_test_dir()] {
        for entry in fs::read_dir(&dir).expect("an upstream config directory is readable") {
            let path = entry.expect("a directory entry").path();
            if path.extension().is_some_and(|e| e == "cfg") {
                files.push(path);
            }
        }
    }
    files.sort();
    files.dedup();
    files
}
