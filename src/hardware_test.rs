//! Configuration-driven real-hardware tests.
//!
//! One input — the user's own printer config, named by [`HW_CONFIG_ENV`] — and
//! per-test declarations of what they need. A module's test says what it wants
//! and then asks for a [`Machine`]:
//!
//! ```ignore
//! #[tokio::test]
//! #[ignore = "hardware: needs KLIPPERX_HW_CONFIG"]
//! async fn test_endstop_trigger_on_a_real_board() {
//!     let Some(machine) = crate::hardware_test::acquire(
//!         "test_endstop_trigger_on_a_real_board",
//!         &crate::hardware_test::Requires::new()
//!             .mcu()
//!             .option("stepper_x", "endstop_pin"),
//!     ) else {
//!         return; // reported as HW-IGNORED; the test passes without a board
//!     };
//!     let mcu = Mcu::connect("mcu", machine.open_mcu().expect("the port opens"))
//!         .await
//!         .expect("identify completes");
//!     // … the assertions, against the real board …
//! }
//! ```
//!
//! The `#[ignore]` marker must be written out as a literal with exactly that
//! wording — an attribute cannot name a constant. Run one like this:
//!
//! ```text
//! KLIPPERX_HW_CONFIG=~/printer_data/config/printer.cfg \
//!   cargo test -p klipperx --lib test_endstop_trigger_on_a_real_board \
//!   -- --ignored --nocapture
//! ```
//!
//! # Everything comes from the config file
//!
//! [`HW_CONFIG_ENV`] names the same config the printer reads, and nothing else is
//! asked of the environment. The interface is derived from it (`[mcu]`'s
//! `serial:` with its `baud:`, or `canbus_uuid:`), and so is every section and
//! option a test names. What counts as "present" is decided by the real config
//! parser ([`Config::from_file`]), so a commented-out `[stepper_x]` is simply
//! absent, and `[include …]`s are resolved the way the printer resolves them.
//! [`plan`] prints what the configured printer provides without opening a port.
//!
//! # Several boards
//!
//! A printer with more than one board declares each extra one as
//! `[mcu <name>]` — `[mcu zboard]`, say — and a test asks for that board by its
//! **name**, not by its section header:
//!
//! ```ignore
//! let Some(machine) = crate::hardware_test::acquire(
//!     "test_z_probe_uses_both_boards",
//!     &crate::hardware_test::Requires::new()
//!         .mcu()                 // [mcu] must be reachable
//!         .mcu_named("zboard"),  // and so must [mcu zboard]
//! ) else {
//!     return; // reported as HW-IGNORED; the test passes without the boards
//! };
//! let main = Mcu::connect("mcu", machine.open_mcu().expect("the port opens"))
//!     .await
//!     .expect("identify completes");
//! let z = Mcu::connect("zboard", machine.open_mcu_named("zboard").expect("its port opens"))
//!     .await
//!     .expect("identify completes");
//! ```
//!
//! Two things the framework deliberately leaves out. It hands a test the
//! **parsed config and the transports**, never a running machine: the printer's
//! own `bring_up` walks the `[mcu]` sections in order and gives each board its
//! clock, so a test that needs the boards to move together must start a
//! `Printer` itself (`load_config` + `bring_up`) instead of opening two
//! transports by hand. And the name a test passes to
//! [`Machine::open_mcu_named`] **must be the same name** it passes to
//! `Mcu::connect` — `"mcu"` for `[mcu]`, `"zboard"` for `[mcu zboard]` —
//! because that name is how the printer tells the boards apart.
//!
//! # Ignored is reported, not failed
//!
//! `cargo test` and `cargo test --workspace` never touch the config: every test
//! here is `#[ignore]`d. Asking for the ignored ones on a machine with no
//! `KLIPPERX_HW_CONFIG` prints `HW-IGNORED: <test>: <why>` and **passes** —
//! "this machine has no board for that" is a skip, and a green run must say so
//! rather than test nothing quietly. A config that is there and names a device
//! which then refuses to open is a real failure: it was there to run.
//!
//! # Serialised on the board
//!
//! A board carries one session, and a test that takes over a session depends on
//! the state its own previous connection left behind, so hardware tests must not
//! overlap. [`acquire`] therefore takes an exclusive `flock` on the config file
//! itself **after** the requirements are met and before the test runs, and holds
//! it on the [`Machine`] until the test ends — `Drop`, the panic path included.
//! A skipped test takes no lock and touches nothing.
//!
//! `flock` also excludes a second open description of the same file, whether or
//! not it is in this process, so one lock covers both "another test thread of
//! this binary" (which `cargo test` runs in parallel by default) and "a second
//! `cargo test` run elsewhere". A test that has to wait prints
//! `HW-WAIT: <test>: …` once and then blocks; the wait has no timeout, because
//! what it waits for is the other test's session on that board, and a session
//! that never ends is a hung test rather than a lock problem. `--test-threads=1`
//! is therefore **not** needed (and passing it anyway is harmless).

use std::fmt;
use std::fs::File;
use std::os::fd::AsRawFd;
use std::path::{Path, PathBuf};

use crate::core::klippy::config::mcu::McuConfig;
use crate::core::klippy::config::{Config, ConfigSection, ConfigWrapper};
use crate::core::klippy::interface::Interface;

/// The environment variable that names the printer config to test against.
///
/// It is the only variable this mode reads; there is no way to name an
/// interface directly, because the config already does.
pub const HW_CONFIG_ENV: &str = "KLIPPERX_HW_CONFIG";

/// What one test needs from the configured printer.
///
/// Built before the config is known, and asked of the parsed config by
/// [`check`].
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct Requires {
    /// MCUs that must be reachable, as their section identifiers — `"mcu"`
    /// for `[mcu]`, `"mcu zboard"` for `[mcu zboard]`.
    mcus: Vec<String>,
    /// Sections that must exist, as written in the config (`"stepper_x"`,
    /// `"mcu zboard"`).
    sections: Vec<String>,
    /// `(section, option)` pairs that must be written in the config.
    options: Vec<(String, String)>,
}

impl Requires {
    /// No condition beyond the config file itself.
    ///
    /// The file is still required: this says "the test has no further
    /// requirements", not "the test runs without [`HW_CONFIG_ENV`]".
    pub fn new() -> Self {
        Self::default()
    }

    /// The main MCU, reachable: `[mcu]` carries `serial:` or `canbus_uuid:`.
    ///
    /// A `[mcu <name>]` does not count — that is another board, and opening it
    /// is not what [`Machine::open_mcu`] does. Ask for it with
    /// [`Requires::mcu_named`].
    pub fn mcu(self) -> Self {
        self.mcu_named("mcu")
    }

    /// The MCU named `name`, reachable: `[mcu <name>]` carries `serial:` or
    /// `canbus_uuid:`. The name `"mcu"` is `[mcu]`, the same as
    /// [`Requires::mcu`].
    ///
    /// The name is the one `Mcu::connect` takes, not the section header:
    /// `"zboard"` for `[mcu zboard]`. A section that merely exists is not
    /// enough — unlike [`Requires::section`], which asks no more than that.
    pub fn mcu_named(mut self, name: impl Into<String>) -> Self {
        let section = mcu_section_id(&name.into());
        if !self.mcus.contains(&section) {
            self.mcus.push(section);
        }
        self
    }

    /// The section `section` must exist, by its full identifier.
    pub fn section(mut self, section: impl Into<String>) -> Self {
        self.sections.push(section.into());
        self
    }

    /// The option must be written in `section`, which must therefore exist.
    pub fn option(mut self, section: impl Into<String>, option: impl Into<String>) -> Self {
        self.options.push((section.into(), option.into()));
        self
    }
}

/// One thing a config does not provide.
///
/// Its [`Display`](fmt::Display) is written to be read in a report
/// (`HW-IGNORED: …: missing endstop_pin in [stepper_x]`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Missing {
    /// The MCU in `[<section>]` is absent, or carries neither `serial:` nor
    /// `canbus_uuid:`. The section identifier is `"mcu"` or `"mcu zboard"`.
    Mcu(String),
    /// A section the test named is not in the config.
    Section(String),
    /// The section exists, but the option is not written in it.
    Option { section: String, option: String },
}

impl fmt::Display for Missing {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Missing::Mcu(section) => {
                write!(f, "missing [{section}] with serial: or canbus_uuid:")
            }
            Missing::Section(section) => write!(f, "missing [{section}]"),
            Missing::Option { section, option } => {
                write!(f, "missing {option} in [{section}]")
            }
        }
    }
}

/// Which of `requires` the config does not provide, in declaration order.
///
/// Pure: it reads nothing but `config` — no environment, no files — so a test
/// can ask it about a config written in a string literal.
pub fn check(config: &Config, requires: &Requires) -> Result<(), Vec<Missing>> {
    let mut missing = Vec::new();
    for section in &requires.mcus {
        if !has_mcu(config, section) {
            push_unique(&mut missing, Missing::Mcu(section.clone()));
        }
    }
    for section in &requires.sections {
        if !config.has_section(section) {
            push_unique(&mut missing, Missing::Section(section.clone()));
        }
    }
    for (section, option) in &requires.options {
        match config.get_section(section) {
            // Reporting the section, not the option, is what tells the user to
            // add `[stepper_x]` rather than one line to an existing section.
            None => push_unique(&mut missing, Missing::Section(section.clone())),
            Some(found) if !found.has(option) => push_unique(
                &mut missing,
                Missing::Option {
                    section: section.clone(),
                    option: option.clone(),
                },
            ),
            Some(_) => {}
        }
    }
    if missing.is_empty() {
        Ok(())
    } else {
        Err(missing)
    }
}

/// The section identifier the config gives the MCU named `name`: `[mcu]` for
/// `"mcu"`, `[mcu zboard]` for `"zboard"`. [`McuConfig::new`] reads the same
/// names back out of the section, so the two spellings stay in step.
fn mcu_section_id(name: &str) -> String {
    if name == "mcu" {
        "mcu".to_string()
    } else {
        format!("mcu {name}")
    }
}

/// Whether the config describes a reachable MCU in `section`.
fn has_mcu(config: &Config, section: &str) -> bool {
    config
        .get_section(section)
        .is_some_and(|found| found.has("serial") || found.has("canbus_uuid"))
}

/// Append `item` unless that exact thing is already in the list, so a test that
/// asks for a section and an option of it reports the section once.
fn push_unique(missing: &mut Vec<Missing>, item: Missing) {
    if !missing.contains(&item) {
        missing.push(item);
    }
}

/// What [`acquire`] makes of the configured file, before anything is printed.
///
/// The environment read is [`acquire`]'s alone; this is the part that takes the
/// path as an argument so the branches can be exercised without setting a
/// variable (which no test can do without racing the other tests).
#[derive(Debug)]
enum Decision {
    /// No `KLIPPERX_HW_CONFIG`.
    NotSet,
    /// The path cannot be used; the message says why (missing file, or the
    /// parser's own error).
    Unreadable(String),
    /// The config parsed, but does not provide everything `Requires` asked for.
    Missing(Vec<Missing>),
    /// The config parsed and provides everything.
    Ready { path: PathBuf, config: Config },
}

/// Decide what the file at `path` offers, without reading the environment.
fn decide(path: Option<&Path>, requires: &Requires) -> Decision {
    let Some(path) = path else {
        return Decision::NotSet;
    };
    let config = match load(path) {
        Ok(config) => config,
        Err(reason) => return Decision::Unreadable(reason),
    };
    match check(&config, requires) {
        Ok(()) => Decision::Ready {
            path: path.to_path_buf(),
            config,
        },
        Err(missing) => Decision::Missing(missing),
    }
}

/// Read and parse the config at `path`, or say why not.
///
/// The parse is `Config::from_file`, the same entry the printer uses, so
/// comments, `[include …]`s and option folding all behave as they do at run
/// time.
fn load(path: &Path) -> Result<Config, String> {
    if !path.exists() {
        return Err(format!("{} does not exist", path.display()));
    }
    Config::from_file(path).map(|(config, _sources)| config)
}

/// A config that satisfies a test's [`Requires`], with the board locked.
pub struct Machine {
    /// The parsed printer config.
    config: Config,
    /// The exclusive lock on the board, held until this machine is dropped.
    _board: BoardLock,
}

impl Machine {
    /// The whole parsed printer config.
    pub fn config(&self) -> &Config {
        &self.config
    }

    /// Open `[mcu]`'s transport, as the printer would.
    ///
    /// The shortcut for [`Machine::open_mcu_named`] with the name `"mcu"`, and
    /// the pair of `Mcu::connect("mcu", …)`.
    pub fn open_mcu(&self) -> Result<Interface, String> {
        self.open_mcu_named("mcu")
    }

    /// Open the transport of the MCU named `name`, as the printer would.
    ///
    /// The name is the one `Mcu::connect` takes, not the section header:
    /// `"mcu"` for `[mcu]`, `"zboard"` for `[mcu zboard]` — the section each
    /// name reads is the one the config gives it. Hand the result to
    /// `Mcu::connect(name, …)` with the **same** name.
    ///
    /// The section is parsed by [`McuConfig`], so `serial:` with its `baud:`,
    /// `canbus_uuid:`, `host_library:` and `restart_method:` all behave as they
    /// do at run time.
    ///
    /// # Errors
    /// A failure here is a **real** failure, not a skip: the config said the
    /// board was there, so a port that will not open is a broken test
    /// environment. The message is the parser's or the transport's own, except
    /// that a name with no section says which section is missing.
    pub fn open_mcu_named(&self, name: &str) -> Result<Interface, String> {
        let section_id = mcu_section_id(name);
        let section = self
            .config
            .get_section(&section_id)
            .ok_or_else(|| format!("the config has no [{section_id}] section"))?;
        let config =
            McuConfig::new(&ConfigWrapper::untracked(section)).map_err(|e| e.to_string())?;
        config.open()
    }
}

/// The exclusive right to talk to one board, held for the life of a test.
///
/// An open file description on the config file with `flock(LOCK_EX)`, released
/// when the guard is dropped — the panic path included, where a test's own
/// cleanup never runs.
struct BoardLock {
    /// The config file, open read-only. `flock` needs no write access; the
    /// lock lives on this descriptor.
    file: File,
}

impl Drop for BoardLock {
    /// The lock is held for exactly as long as the guard lives: release it here,
    /// and the descriptor's close below is only the last word on it.
    fn drop(&mut self) {
        // SAFETY: `file` is still open, and the lock being released is the one
        // this guard took on its descriptor.
        unsafe { libc::flock(self.file.as_raw_fd(), libc::LOCK_UN) };
    }
}

/// Take the board lock for the config at `config`, announcing a wait once.
///
/// `test_name` is only used in the message. Blocking, and no other lock is held
/// while blocking here.
fn lock_board(config: &Path, test_name: &str) -> Result<BoardLock, String> {
    let file = File::open(config).map_err(|error| format!("{}: {error}", config.display()))?;
    let fd = file.as_raw_fd();

    // SAFETY: `fd` is open for the lifetime of `file`, which is either returned
    // in the guard or dropped here.
    if unsafe { libc::flock(fd, libc::LOCK_EX | libc::LOCK_NB) } == 0 {
        // Nobody holds the board: nothing to announce, nothing to wait for.
        return Ok(BoardLock { file });
    }
    let error = std::io::Error::last_os_error();
    if error.raw_os_error() != Some(libc::EWOULDBLOCK) {
        return Err(format!("{}: flock: {error}", config.display()));
    }
    println!("HW-WAIT: {test_name}: waiting for the board (another hardware test is running)");

    loop {
        // SAFETY: as above; `fd` is owned by `file`, which is still alive.
        if unsafe { libc::flock(fd, libc::LOCK_EX) } == 0 {
            return Ok(BoardLock { file });
        }
        let error = std::io::Error::last_os_error();
        if error.kind() != std::io::ErrorKind::Interrupted {
            return Err(format!("{}: flock: {error}", config.display()));
        }
    }
}

/// Hand `test_name` a machine for the configured board, or report it as skipped.
///
/// The only reader of [`HW_CONFIG_ENV`]. Returns `None` — and prints one
/// `HW-IGNORED: <test>: <why>` line — when the variable is unset, the file is
/// missing or unparsable, or the config does not provide `requires`; a skipped
/// test touches no device and takes no lock.
///
/// Returns `Some(machine)` with the board locked exclusively when everything is
/// in place, printing `HW-RUN: <test>` first. The lock is held until the machine
/// is dropped, so the test body and its cleanup are one session on the board.
pub fn acquire(test_name: &str, requires: &Requires) -> Option<Machine> {
    let path = std::env::var(HW_CONFIG_ENV).ok();
    match decide(path.as_deref().map(Path::new), requires) {
        Decision::NotSet => {
            println!("HW-IGNORED: {test_name}: {HW_CONFIG_ENV} is not set");
            None
        }
        Decision::Unreadable(reason) => {
            println!("HW-IGNORED: {test_name}: {reason}");
            None
        }
        Decision::Missing(missing) => {
            let reasons: Vec<String> = missing.iter().map(ToString::to_string).collect();
            println!("HW-IGNORED: {test_name}: {}", reasons.join(", "));
            None
        }
        Decision::Ready { path, config } => {
            // Take the lock **before** the test runs: the board can only carry
            // one session, so a test that is about to run must own it. A config
            // that parsed a moment ago but cannot be opened for locking is the
            // one thing here that is neither a skip nor the test's own failure,
            // so say what went wrong and stop.
            let _board = lock_board(&path, test_name).unwrap_or_else(|reason| {
                panic!("{test_name}: cannot serialise on the board's config file: {reason}")
            });
            println!("HW-RUN: {test_name}");
            Some(Machine { config, _board })
        }
    }
}

/// Print what the configured printer provides, without touching a board.
///
/// The point is to see which tests a config would activate before running any
/// of them: the section name list, the options each section carries (the
/// universe [`Requires::section`] and [`Requires::option`] ask about), and
/// one line per MCU section saying what [`Requires::mcu`] and
/// [`Requires::mcu_named`] would make of it.
#[test]
#[ignore = "hardware: prints what KLIPPERX_HW_CONFIG provides"]
fn plan() {
    let Some(path) = std::env::var(HW_CONFIG_ENV).ok() else {
        println!("HW-CONFIG: {HW_CONFIG_ENV} is not set — no hardware test can run.");
        println!("HW-CONFIG: point it at your printer config and re-run this test, e.g.");
        println!(
            "HW-CONFIG:   {HW_CONFIG_ENV}=/path/to/printer.cfg \
             cargo test -p klipperx --lib plan -- --ignored --nocapture"
        );
        return;
    };
    match load(Path::new(&path)) {
        Ok(config) => {
            println!("HW-CONFIG: {path}");
            print_plan(&config);
        }
        Err(reason) => println!("HW-CONFIG: {reason}"),
    }
}

/// The body of [`plan`]: the sections, their options, and the MCU verdicts.
fn print_plan(config: &Config) {
    let sections = config.sections_vec();
    println!("HW-SECTIONS: {}", sections.len());
    for section in sections {
        let options: Vec<&str> = section.parameters.keys().map(String::as_str).collect();
        println!(
            "HW-SECTION: [{}] {}",
            section.identifier(),
            options.join(", ")
        );
    }
    for line in mcu_plan_lines(config) {
        println!("{line}");
    }
}

/// One `HW-MCU:` line per MCU section, in config order — the MCU verdicts
/// [`plan`] prints, as the lines themselves.
///
/// Each line names the section and what a test would ask of it: `.mcu()` for
/// `[mcu]`, `.mcu_named("zboard")` for `[mcu zboard]`, and whether that
/// requirement passes (`serial:` or `canbus_uuid:`) or skips. A config with no
/// MCU section at all is one line saying so.
fn mcu_plan_lines(config: &Config) -> Vec<String> {
    let sections = config.get_sections_by_id("mcu");
    if sections.is_empty() {
        return vec!["HW-MCU: no [mcu] section — `.mcu()` tests are ignored".to_string()];
    }
    sections
        .iter()
        .map(|section| {
            let requirement = mcu_requirement(section);
            let (interface, verdict) = if section.has("serial") {
                (
                    format!(
                        "serial: {} (baud: {})",
                        section.get_str("serial").unwrap_or_default(),
                        section.get_str("baud").unwrap_or("250000 (default)"),
                    ),
                    format!("{requirement} tests run"),
                )
            } else if section.has("canbus_uuid") {
                (
                    format!(
                        "canbus_uuid: {} (interface: {}, nodeid: {})",
                        section.get_str("canbus_uuid").unwrap_or_default(),
                        section.get_str("canbus_interface").unwrap_or("can0"),
                        section
                            .get_str("canbus_nodeid")
                            .unwrap_or("unset (klipperx needs one)"),
                    ),
                    format!("{requirement} tests run"),
                )
            } else {
                (
                    "no serial:/canbus_uuid:".to_string(),
                    format!("{requirement} tests skip"),
                )
            };
            format!("HW-MCU: [{}] {interface} — {verdict}", section.identifier())
        })
        .collect()
}

/// How a test asks [`check`] for the MCU in `section`: `.mcu()` for `[mcu]`,
/// `.mcu_named("zboard")` for `[mcu zboard]`.
fn mcu_requirement(section: &ConfigSection) -> String {
    match &section.sub {
        None => ".mcu()".to_string(),
        Some(name) => format!(".mcu_named(\"{name}\")"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    /// Parse fixture text with the real parser.
    fn config(text: &str) -> Config {
        Config::from_text(text).expect("the fixture parses").0
    }

    /// A fixture config file, removed when the test ends.
    struct TempConfig(PathBuf);

    impl TempConfig {
        fn new(text: &str) -> Self {
            use std::sync::atomic::{AtomicU32, Ordering};
            static NEXT: AtomicU32 = AtomicU32::new(0);
            let path = std::env::temp_dir().join(format!(
                "klipperx-hwtest-{}-{}.cfg",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed),
            ));
            std::fs::write(&path, text).expect("the fixture writes");
            Self(path)
        }

        fn path(&self) -> &Path {
            &self.0
        }
    }

    impl Drop for TempConfig {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }

    // -----------------------------------------------------------------------
    // check() — sections, options, comments, the MCU
    // -----------------------------------------------------------------------

    #[test]
    fn test_a_present_section_satisfies_the_requirement() {
        let config = config("[stepper_x]\nendstop_pin: PA0\n");

        assert_eq!(
            check(&config, &Requires::new().section("stepper_x")),
            Ok(())
        );
        assert_eq!(
            check(&config, &Requires::new().section("stepper_y")),
            Err(vec![Missing::Section("stepper_y".to_string())])
        );
    }

    #[test]
    fn test_a_present_option_satisfies_the_requirement() {
        let config = config("[stepper_x]\nendstop_pin: PA0\n");

        assert_eq!(
            check(&config, &Requires::new().option("stepper_x", "endstop_pin")),
            Ok(())
        );
        // The section is there, only the option is not.
        assert_eq!(
            check(
                &config,
                &Requires::new().option("stepper_x", "position_endstop")
            ),
            Err(vec![Missing::Option {
                section: "stepper_x".to_string(),
                option: "position_endstop".to_string(),
            }])
        );
        // No such section at all: the section is what is missing.
        assert_eq!(
            check(
                &config,
                &Requires::new().option("extruder", "nozzle_diameter")
            ),
            Err(vec![Missing::Section("extruder".to_string())])
        );
    }

    /// The case the whole design turns on: "commented out" is decided by the
    /// real parser, so a commented-out section or option is simply not there.
    #[test]
    fn test_a_commented_out_section_or_option_is_absent() {
        let config = config(
            "# [stepper_x]\n\
             # endstop_pin: PA0\n\
             [mcu]\n\
             serial: /dev/ttyACM0\n\
             # baud: 250000\n",
        );

        assert_eq!(
            check(&config, &Requires::new().section("stepper_x")),
            Err(vec![Missing::Section("stepper_x".to_string())])
        );
        assert_eq!(
            check(&config, &Requires::new().option("mcu", "endstop_pin")),
            Err(vec![Missing::Option {
                section: "mcu".to_string(),
                option: "endstop_pin".to_string(),
            }])
        );
        assert_eq!(
            check(&config, &Requires::new().option("mcu", "baud")),
            Err(vec![Missing::Option {
                section: "mcu".to_string(),
                option: "baud".to_string(),
            }])
        );
    }

    #[test]
    fn test_a_comment_inside_a_present_section_leaves_the_option_absent() {
        let config = config(
            "[stepper_x]\n\
             # endstop_pin: PA0\n\
             position_endstop: 0\n",
        );

        assert_eq!(
            check(
                &config,
                &Requires::new().option("stepper_x", "position_endstop")
            ),
            Ok(())
        );
        assert_eq!(
            check(&config, &Requires::new().option("stepper_x", "endstop_pin")),
            Err(vec![Missing::Option {
                section: "stepper_x".to_string(),
                option: "endstop_pin".to_string(),
            }])
        );
    }

    #[test]
    fn test_the_mcu_requirement_needs_an_interface_key() {
        // No `[mcu]` at all.
        let none = config("[printer]\nkinematics: none\n");
        assert_eq!(
            check(&none, &Requires::new().mcu()),
            Err(vec![Missing::Mcu("mcu".to_string())])
        );

        // `[mcu]` with neither `serial:` nor `canbus_uuid:`.
        let bare = config("[mcu]\nrestart_method: arduino\n");
        assert_eq!(
            check(&bare, &Requires::new().mcu()),
            Err(vec![Missing::Mcu("mcu".to_string())])
        );

        // A serial interface.
        let serial = config("[mcu]\nserial: /dev/ttyACM0\nbaud: 250000\n");
        assert_eq!(check(&serial, &Requires::new().mcu()), Ok(()));

        // A CAN interface.
        let can = config("[mcu]\ncanbus_uuid: 11aa22bb33cc\ncanbus_nodeid: 2\n");
        assert_eq!(check(&can, &Requires::new().mcu()), Ok(()));
    }

    /// `mcu_named` is `.mcu()` with the section swapped, so it asks the same
    /// question of `[mcu <name>]` — presence of the section is not enough, and
    /// the name `"mcu"` means `[mcu]` rather than `[mcu mcu]`.
    #[test]
    fn test_the_named_mcu_requirement_needs_its_own_interface_key() {
        // No `[mcu zboard]` at all: the main MCU does not stand in for it.
        let none = config("[mcu]\nserial: /dev/ttyACM0\n");
        assert_eq!(
            check(&none, &Requires::new().mcu_named("zboard")),
            Err(vec![Missing::Mcu("mcu zboard".to_string())])
        );

        // `[mcu zboard]` with neither `serial:` nor `canbus_uuid:` — the case
        // `Requires::section("mcu zboard")` would wave through.
        let bare = config("[mcu zboard]\nrestart_method: arduino\n");
        assert_eq!(check(&bare, &Requires::new().section("mcu zboard")), Ok(()));
        assert_eq!(
            check(&bare, &Requires::new().mcu_named("zboard")),
            Err(vec![Missing::Mcu("mcu zboard".to_string())])
        );

        // A serial interface.
        let serial = config("[mcu zboard]\nserial: /dev/ttyACM1\n");
        assert_eq!(check(&serial, &Requires::new().mcu_named("zboard")), Ok(()));

        // A CAN interface.
        let can = config("[mcu zboard]\ncanbus_uuid: 11aa22bb33cc\n");
        assert_eq!(check(&can, &Requires::new().mcu_named("zboard")), Ok(()));

        // The name `"mcu"` is the main section, not `[mcu mcu]`.
        let main = config("[mcu]\nserial: /dev/ttyACM0\n");
        assert_eq!(check(&main, &Requires::new().mcu_named("mcu")), Ok(()));
    }

    /// The boundary the requirement has: the main MCU is `[mcu]`, so a config
    /// whose only MCU is a named one does not satisfy `.mcu()`.
    #[test]
    fn test_a_named_mcu_is_not_the_main_mcu() {
        let config = config("[mcu zboard]\nserial: /dev/ttyACM1\n");

        assert_eq!(
            check(&config, &Requires::new().mcu()),
            Err(vec![Missing::Mcu("mcu".to_string())])
        );
        // …though a test can still ask for that section by name.
        assert_eq!(
            check(&config, &Requires::new().section("mcu zboard")),
            Ok(())
        );
    }

    #[test]
    fn test_every_missing_item_is_listed_once() {
        let config = config("[mcu]\nserial: /dev/ttyACM0\n[extruder]\nstep_pin: PA0\n");
        let requires = Requires::new()
            .mcu()
            .section("stepper_x")
            // The same absent section asked for twice — by name and through an
            // option of it — is reported once.
            .option("stepper_x", "endstop_pin")
            .option("extruder", "nozzle_diameter");

        assert_eq!(
            check(&config, &requires),
            Err(vec![
                Missing::Section("stepper_x".to_string()),
                Missing::Option {
                    section: "extruder".to_string(),
                    option: "nozzle_diameter".to_string(),
                },
            ])
        );
    }

    #[test]
    fn test_an_empty_requires_asks_only_for_the_config_file() {
        // Nothing is asked of the config itself …
        assert_eq!(check(&config(""), &Requires::new()), Ok(()));
        assert_eq!(check(&config("[mcu]\n"), &Requires::new()), Ok(()));
        // … but the file is still needed: an unset variable is still a skip.
        assert!(matches!(decide(None, &Requires::new()), Decision::NotSet));
    }

    #[test]
    fn test_a_missing_item_reads_as_what_is_missing() {
        assert_eq!(
            Missing::Mcu("mcu".to_string()).to_string(),
            "missing [mcu] with serial: or canbus_uuid:"
        );
        assert_eq!(
            Missing::Mcu("mcu zboard".to_string()).to_string(),
            "missing [mcu zboard] with serial: or canbus_uuid:"
        );
        assert_eq!(
            Missing::Section("stepper_x".to_string()).to_string(),
            "missing [stepper_x]"
        );
        assert_eq!(
            Missing::Option {
                section: "stepper_x".to_string(),
                option: "endstop_pin".to_string(),
            }
            .to_string(),
            "missing endstop_pin in [stepper_x]"
        );
    }

    // -----------------------------------------------------------------------
    // decide() — the branches that read a path instead of the environment
    // -----------------------------------------------------------------------

    #[test]
    fn test_an_unmet_requirement_is_reported_as_missing() {
        let fixture = TempConfig::new("[printer]\nkinematics: none\n");

        match decide(Some(fixture.path()), &Requires::new().section("stepper_x")) {
            Decision::Missing(missing) => {
                assert_eq!(missing, vec![Missing::Section("stepper_x".to_string())]);
            }
            other => panic!("expected a missing requirement, got {other:?}"),
        }
    }

    #[test]
    fn test_a_path_that_does_not_exist_is_reported_with_its_path() {
        let absent = std::env::temp_dir().join("klipperx-hwtest-does-not-exist/printer.cfg");

        match decide(Some(&absent), &Requires::new()) {
            Decision::Unreadable(reason) => {
                assert!(reason.ends_with("does not exist"), "{reason}");
                assert!(reason.contains("printer.cfg"), "{reason}");
            }
            other => panic!("expected an unreadable config, got {other:?}"),
        }
    }

    #[test]
    fn test_a_config_that_does_not_parse_reports_the_parsers_own_error() {
        // An option before any section: the parser's own wording, not ours.
        let fixture = TempConfig::new("serial: /dev/ttyACM0\n");

        match decide(Some(fixture.path()), &Requires::new()) {
            Decision::Unreadable(reason) => {
                assert!(reason.contains("Parameter outside of section"), "{reason}");
            }
            other => panic!("expected a parse failure, got {other:?}"),
        }
    }

    #[test]
    fn test_a_satisfied_config_is_handed_back_parsed() {
        let fixture = TempConfig::new("[mcu]\nserial: /dev/ttyACM0\n");

        match decide(Some(fixture.path()), &Requires::new().mcu()) {
            Decision::Ready { path, config } => {
                assert_eq!(path, fixture.path());
                assert_eq!(
                    config.get_section("mcu").and_then(|s| s.get_str("serial")),
                    Some("/dev/ttyACM0")
                );
            }
            other => panic!("expected a parsed config, got {other:?}"),
        }
    }

    /// The machine hands back the config it was built from, and opening the MCU
    /// goes through the real transport: a port the config names but which does
    /// not exist is an error, not a skip.
    #[test]
    fn test_the_machine_exposes_its_config_and_opens_the_named_transport() {
        let fixture = TempConfig::new("[mcu]\nserial: /dev/klipperx-hwtest-no-such-port\n");
        let config = load(fixture.path()).expect("the fixture parses");
        let board = lock_board(fixture.path(), "self-test").expect("the fixture locks");
        let machine = Machine {
            config,
            _board: board,
        };

        assert_eq!(
            machine
                .config()
                .get_section("mcu")
                .and_then(|section| section.get_str("serial")),
            Some("/dev/klipperx-hwtest-no-such-port")
        );

        // The message is the transport's own, and it names the device the
        // config asked for — the failure a present-but-broken board produces.
        let error = machine.open_mcu().unwrap_err();
        assert!(error.starts_with("serial: "), "{error}");
        assert!(
            error.contains("/dev/klipperx-hwtest-no-such-port"),
            "{error}"
        );
    }

    /// `open_mcu_named` opens the section the name names: two boards with two
    /// imaginary ports tell the two apart without touching a device, because the
    /// transport's error carries the path the *chosen* section asked for.
    #[test]
    fn test_open_mcu_named_opens_the_section_it_names() {
        let fixture = TempConfig::new(
            "[mcu]\nserial: /dev/does-not-exist-a\n\
             [mcu zboard]\nserial: /dev/does-not-exist-b\n",
        );
        let config = load(fixture.path()).expect("the fixture parses");
        let board = lock_board(fixture.path(), "self-test").expect("the fixture locks");
        let machine = Machine {
            config,
            _board: board,
        };

        let main = machine.open_mcu().unwrap_err();
        assert!(main.contains("/dev/does-not-exist-a"), "{main}");
        let named = machine.open_mcu_named("zboard").unwrap_err();
        assert!(named.contains("/dev/does-not-exist-b"), "{named}");
        assert!(!named.contains("/dev/does-not-exist-a"), "{named}");

        // A name with no section says which section is missing.
        assert_eq!(
            machine.open_mcu_named("nonexistent").unwrap_err(),
            "the config has no [mcu nonexistent] section"
        );
    }

    /// [`plan`] says one line per MCU section, each naming the section and the
    /// requirement a test would use to ask for it.
    #[test]
    fn test_the_plan_prints_one_line_per_mcu_section() {
        let multi = config(
            "[mcu]\nserial: /dev/ttyACM0\n\
             [mcu zboard]\nserial: /dev/ttyACM1\n\
             [mcu toolhead]\nrestart_method: arduino\n",
        );

        assert_eq!(
            mcu_plan_lines(&multi),
            vec![
                "HW-MCU: [mcu] serial: /dev/ttyACM0 (baud: 250000 (default)) — .mcu() tests run"
                    .to_string(),
                "HW-MCU: [mcu zboard] serial: /dev/ttyACM1 (baud: 250000 (default)) \
                 — .mcu_named(\"zboard\") tests run"
                    .to_string(),
                "HW-MCU: [mcu toolhead] no serial:/canbus_uuid: \
                 — .mcu_named(\"toolhead\") tests skip"
                    .to_string(),
            ]
        );

        // A CAN board prints its transport instead of a serial device.
        let can = config("[mcu zboard]\ncanbus_uuid: 11aa22bb33cc\ncanbus_nodeid: 2\n");
        assert_eq!(
            mcu_plan_lines(&can),
            vec![
                "HW-MCU: [mcu zboard] canbus_uuid: 11aa22bb33cc (interface: can0, nodeid: 2) \
                 — .mcu_named(\"zboard\") tests run"
                    .to_string()
            ]
        );

        // No MCU section at all is one line saying so.
        assert_eq!(
            mcu_plan_lines(&config("[printer]\nkinematics: none\n")),
            vec!["HW-MCU: no [mcu] section — `.mcu()` tests are ignored".to_string()]
        );
    }

    // -----------------------------------------------------------------------
    // The board lock — one session at a time
    // -----------------------------------------------------------------------

    #[test]
    fn test_the_lock_is_released_when_the_guard_is_dropped() {
        let fixture = TempConfig::new("[mcu]\nserial: /dev/ttyACM0\n");

        let held = lock_board(fixture.path(), "self-test").expect("the fixture locks");
        // The lock is the config file itself: no side-car file is made, so the
        // same config is the same lock in every process.
        assert!(!fixture.path().with_extension("lock").exists());
        drop(held);
        // A released lock can be taken again — the same test running twice, or
        // the next test in the binary.
        let _again = lock_board(fixture.path(), "self-test").expect("a released lock is free");
    }

    /// Two threads contend for the same config's lock and their critical
    /// sections must not overlap. A barrier makes them contend for real instead
    /// of hoping the scheduler interleaves them.
    #[test]
    fn test_hardware_tests_serialise_on_the_config_file() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        use std::sync::{Arc, Barrier};
        use std::thread;

        let fixture = TempConfig::new("[mcu]\nserial: /dev/ttyACM0\n");
        let barrier = Arc::new(Barrier::new(3));
        let inside = Arc::new(AtomicUsize::new(0));
        let most_inside = Arc::new(AtomicUsize::new(0));

        let workers: Vec<_> = (0..2)
            .map(|_| {
                let path = fixture.path().to_path_buf();
                let barrier = Arc::clone(&barrier);
                let inside = Arc::clone(&inside);
                let most_inside = Arc::clone(&most_inside);
                thread::spawn(move || {
                    barrier.wait();
                    let _held = lock_board(&path, "self-test").expect("the fixture locks");
                    let now_inside = inside.fetch_add(1, Ordering::SeqCst) + 1;
                    most_inside.fetch_max(now_inside, Ordering::SeqCst);
                    thread::sleep(std::time::Duration::from_millis(50));
                    inside.fetch_sub(1, Ordering::SeqCst);
                })
            })
            .collect();

        barrier.wait();
        for worker in workers {
            worker.join().expect("the worker finishes");
        }

        assert_eq!(
            most_inside.load(Ordering::SeqCst),
            1,
            "two holders inside the lock at once"
        );
    }
}
