//! Configuration-driven real-hardware tests.
//!
//! One input — the user's own printer config, named by [`HW_CONFIG_ENV`] — and
//! per-test declarations of what they need. A module's test says what it wants
//! and hands [`case`] (or [`async_case`]) the body to run with a [`Machine`]:
//!
//! ```ignore
//! #[tokio::test]
//! #[ignore = "hardware: needs KLIPPERX_HW_CONFIG"]
//! async fn test_endstop_trigger_on_a_real_board() {
//!     crate::hardware_test::async_case(
//!         "test_endstop_trigger_on_a_real_board",
//!         &crate::hardware_test::Requires::new()
//!             .mcu()
//!             .option("stepper_x", "endstop_pin"),
//!         |machine| async move {
//!             let mcu = Mcu::connect("mcu", machine.open_mcu().expect("the port opens"))
//!                 .await
//!                 .expect("identify completes");
//!             // … the assertions, against the real board …
//!         },
//!     );
//! }
//! ```
//!
//! That case is the **first layer**, and it is `async`: it opens the board
//! itself with `Mcu::connect(…).await` and owns no runtime, so `#[tokio::test]`
//! is the right frame. A case over the **second** layer is the opposite and
//! must be a **plain `#[test]`** with [`case`] — see [the second
//! layer](#the-second-layer-a-machine-that-is-up).
//!
//! Both runners give the case a **total budget** ([`case`], [`CASE_TIMEOUT`]):
//! the body runs on a thread of its own, and a case that hangs is reported as
//! `HW-TIMEOUT: <case>: still at "<phase>" …` rather than parking the run. A
//! config that does not provide what the case declared **fails** the case
//! unless strict mode is turned off ([`HW_STRICT_ENV`]), and the body never
//! runs either way.
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
//! # This tests the module, not your config
//!
//! The premise of the whole mode, and it holds for **every** case here: what is
//! under test is this repository's module, not the config you handed it.
//!
//! The config is the *input*. A case reads it to learn which section, which
//! option and which board it has to work with, and then **takes it on trust**:
//! an accepted config is one upstream Klipper would also accept, wired the way
//! the config says. Nothing here validates a config, and no case is a check of
//! one.
//!
//! So a result reads this way:
//!
//! * A failure caused by the config, the wiring, a firmware that does not match
//!   it, or an environment that is not what the config describes is **not** a bug
//!   in the module the case is about.
//! * A pass does not mean the config is right either: it means the module
//!   behaved on a board whose config was, by assumption, correct.
//!
//! To check a config itself, use the host (`objects/list`, `status`, the log) or
//! upstream Klipper; this mode is not the tool for it.
//!
//! # Everything comes from the config file
//!
//! [`HW_CONFIG_ENV`] names the same config the printer reads, and the only other
//! thing asked of the environment is [`HW_STRICT_ENV`] — a mode, not an input.
//! The interface is derived from the config (`[mcu]`'s
//! `serial:` with its `baud:`, or `uuid:`), and so is every section and
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
//! crate::hardware_test::async_case(
//!     "test_z_probe_uses_both_boards",
//!     &crate::hardware_test::Requires::new()
//!         .mcu()                 // [mcu] must be reachable
//!         .mcu_named("zboard"),  // and so must [mcu zboard]
//!     |machine| async move {
//!         let main = Mcu::connect("mcu", machine.open_mcu().expect("the port opens"))
//!             .await
//!             .expect("identify completes");
//!         let z = Mcu::connect("zboard", machine.open_mcu_named("zboard").expect("its port opens"))
//!             .await
//!             .expect("identify completes");
//!         // …
//!     },
//! );
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
//! # The second layer: a machine that is up
//!
//! [`Machine::open_mcu`]/[`Machine::open_mcu_named`] are the first layer — a
//! parsed config, the board lock, and one transport a test opens itself. That is
//! what a test about a *wire* wants (identify, a frame sequence, a clock read).
//! A test about the **machine** asks [`Machine::bring_up`] for a
//! [`StartedMachine`] instead, and gets the printer the host would have: every
//! part loaded and connected, `toolhead`, `gcode` and the extras registered.
//!
//! ```ignore
//! #[test]
//! #[ignore = "hardware: needs KLIPPERX_HW_CONFIG"]
//! fn test_homing_reaches_the_endstop_on_a_real_board() {
//!     crate::hardware_test::case(
//!         "test_homing_reaches_the_endstop_on_a_real_board",
//!         &crate::hardware_test::Requires::new()
//!             .mcu()
//!             .option("stepper_x", "endstop_pin"),
//!         |machine| {
//!             let started = machine.bring_up().expect("the machine comes up");
//!             let gcode = started
//!                 .printer()
//!                 .lookup_object_as::<GCodeDispatch>(crate::core::klippy::gcode::GCODE_OBJECT)
//!                 .expect("the g-code dispatcher is registered");
//!             gcode.run_script_sync("G28 X").expect("the homing move runs");
//!             // … the assertions, against the running machine …
//!             // `started` is dropped here, and that is what takes the machine
//!             // down (`Drop`, the panic path included) — keep it alive to the
//!             // end of the body, or put it in a scope that ends where the
//!             // session should.
//!         },
//!     );
//! }
//! ```
//!
//! **`#[test]`, never `#[tokio::test]`.** [`Machine::bring_up`] builds the
//! multi-threaded runtime the machine runs on and takes it down with a bounded
//! blocking `shutdown_timeout`, and neither is legal from inside another
//! runtime's context: tokio panics with `Cannot start a runtime from within a
//! runtime` at the build, and `Cannot drop a runtime in a context where blocking
//! is not allowed` at the shutdown — which is on `bring_up`'s failure path too,
//! and in [`StartedMachine`]'s `Drop`. So a second-layer case runs its g-code
//! through the async-free `GCodeDispatch::run_script_sync` /
//! `run_script_from_command_sync` (`#[cfg(test)]`) instead of `.await`, and the
//! module's own bring-up self-tests are plain `#[test]`s for the same reason.
//! Calling `bring_up` from inside a runtime is refused with an `Err` naming the
//! mistake rather than left to tokio's panic.
//!
//! Four things [`Machine::bring_up`] does and does not do:
//!
//! * **It starts what the config describes, not one board.** `bring_up` walks
//!   *every* `[mcu …]` section, so a printer with `[mcu zboard]` gets both
//!   boards connected, identified and clocked against each other — the one thing
//!   the first layer cannot give a test. `open_mcu_named` stays the way to reach
//!   a single board's transport, and the two are complementary rather than
//!   alternatives: a test that wants both boards' *wires* opens two transports,
//!   and a test that wants the machine brings it up.
//! * **The lock is unchanged.** [`acquire`] still takes the board lock on the
//!   config file and [`Machine`] still holds it until the test's `Machine` is
//!   dropped; `bring_up` neither takes a second lock nor releases the first.
//! * **It owns its runtime.** The machine runs on a multi-threaded runtime this
//!   layer builds and keeps, and the shutdown of that runtime is bounded — see
//!   [`SHUTDOWN_TIMEOUT`] and [`StartedMachine`]'s `Drop`. A machine that is
//!   leaked therefore reports as a failure in bounded time instead of hanging
//!   the test run: this is the live shape, where a device's blocking read parks
//!   and a runtime that merely waited for it would wait forever
//!   (`core::klippy::upstream`'s live-case note). Owning the runtime is also why
//!   a case over this layer must be a plain `#[test]`: see the guard at the top
//!   of [`Machine::bring_up`].
//! * **Its failures are failures.** A config that names a board which cannot be
//!   opened, or a bring-up that does not reach `Ready`, is an `Err` — the config
//!   said that hardware was there, so it was there to run. A [`HW_CONFIG_ENV`]
//!   that is set but names a config that cannot be read or parsed is a failure
//!   too ([`acquire`] panics): the variable said there was a config to test
//!   against. An unmet [`Requires`] is a failure for the same reason, unless
//!   [`HW_STRICT_ENV`] turns strict mode off; only an unset [`HW_CONFIG_ENV`] is
//!   a skip in both modes, and `bring_up` is never reached in either case.
//!
//! The strictness is deliberately the default: a machine whose config cannot run
//! a case is not a machine that has nothing to check, and a green `--ignored`
//! run must not be readable as "this config was tested" when the case never
//! ran. `set KLIPPERX_HW_STRICT=0` is the escape hatch for a config shared by
//! several machines, where the cases it cannot run should be reported and
//! passed over.
//!
//! # The live shape, not the corpus' file-output shape
//!
//! Every corpus case and the harness that generates them run in upstream's
//! *file-output* mode (`-o`, `debug_output` set), where nothing answers the
//! machine. A real board is the opposite: the *live* shape, `debug_output`
//! unset, which is what [`Machine::bring_up`] builds. The differences a test
//! has to know about are all of that one kind — a branch that short-circuits
//! under `-o` really runs here:
//!
//! | | corpus (`-o`) | live ([`Machine::bring_up`]) |
//! |---|---|---|
//! | `can_pause` | false | true, so `M400` really waits for the move (`extras/toolhead.rs:3547-3552`) |
//! | `can_extrude` | forced true | `min_extrude_temp <= 0.0` or a real reading above it; `G1 E…` is refused below it (`extras/heaters.rs:868`) |
//! | `TEMPERATURE_WAIT`, `SET_HEATER_TEMPERATURE … WAIT` | return at once | wait for the board's readings (`extras/heaters.rs:563`, `:1044`) |
//! | `verify_heater` | never checks | checks run from `klippy:connect` and shut a heater down when it is not heating (`extras/verify_heater.rs:214-250`) |
//! | temperature readings | never arrive | come from the board, so a move that depends on one waits for the hardware |
//!
//! A case that extrudes without heating therefore *passes* under `-o` and is
//! refused here, and a move that waits is real wall time. Both are the point of
//! testing against a board; neither is a bug in the test harness.
//!
//! # The shutdown bound is a real check again
//!
//! [`StartedMachine`]'s `Drop` tears the machine down and then bounds its
//! runtime's shutdown, so a leaked part is a **failure** rather than a hang.
//! That bound used to be tripped by a known ring, a bug of the machine rather
//! than of this layer: with a live shape and a config that has a heater section
//! (`[extruder]`, `[heater_bed]`, `[heater_generic]` — anything that builds a
//! `Heater`), `verify_heater`'s 1 s reactor timer callback held a **strong**
//! `Arc<Heater>` while `Printer::teardown` never cancels reactor timers, so the
//! reactor's own timer heap kept the last `Arc<Mcu>` alive — `Mcu::Drop` never
//! ran, `Interface::shutdown` was never called, and the board's device read
//! stayed parked for the whole [`SHUTDOWN_TIMEOUT`].
//!
//! That ring is **fixed** (`bd2f32a`): the callback now holds a
//! `Weak<Heater>` (`extras/verify_heater.rs`) and `HeaterCheck::Drop` cancels
//! the timer, so the parts go and the device closes like any other config's.
//! The bound is therefore live rather than a known cost — a heater config now
//! shuts the session down promptly, and a case that trips the bound again is a
//! **regression**, not an accepted leak. The history and the measured numbers
//! are in `docs/klippy/developer-manual/testing.md` (「两层能力」) and in
//! `TODO.md`'s Q11.
//!
//! One ring of that shape remains, and it is neither this layer's nor a
//! heater's: `api/endpoints/objects_subscribe.rs`'s timer callback holds an
//! `Arc<Inner>`, and `Inner` holds the `printer` and the `reactor`, so
//! `reactor → heap → Inner → reactor` keeps a machine alive after an in-place
//! restart (`TODO.md`'s S5). Until that is broken, a restart case with an API
//! subscription can trip the bound; the cycle itself is fixed where it lives,
//! never papered over with `Mcu::close` here.
//!
//! # The machine is turned off before it is torn down
//!
//! A case over the second layer can command hardware: `M104`/`M140` set a
//! target, `SET_PIN` drives a fan or a PWM output. A case that commands one and
//! then fails would leave the board doing it — on a real printer, heating — for
//! as long as it stays powered. So [`StartedMachine`]'s `Drop` turns the machine
//! off **before** it takes the parts down: `TURN_OFF_HEATERS` (every heater's
//! target to 0) and `SET_PIN PIN=<name> VALUE=0` for every `[output_pin]` that is
//! not already at zero. It runs on both paths, the pass and the panic, because
//! it is in `Drop` rather than in the case body, and it reports what it put back
//! as `HW-TEARDOWN:` lines (best effort: a machine that is already shut down
//! refuses commands, which is another off state).
//!
//! # Ignored is reported, not failed
//!
//! `cargo test` and `cargo test --workspace` never touch the config: every test
//! here is `#[ignore]`d. Asking for the ignored ones on a machine with no
//! `KLIPPERX_HW_CONFIG` prints `HW-IGNORED: <test>: <why>` and **passes** —
//! "this machine has no board for that" is a skip, and a green run must say so
//! rather than test nothing quietly. A config that **is** named and does not
//! provide what the test declared it needs is not a skip: it is a failure, the
//! same as a config that is named but unreadable or unparsable, because the
//! variable promised a config and the test was meant to run against it.
//! `KLIPPERX_HW_STRICT=0` asks for the old skip (`HW-IGNORED: …`) instead — see
//! [`HW_STRICT_ENV`]. Strict mode also fails a config that names an object this
//! repository does not implement ([`NOT_IMPLEMENTED_SECTIONS`], the deferred
//! `upstream-deviations.md` §1.2 modules), because no case here could run
//! against it. A config that is there and names a device which then
//! refuses to open is a real failure too: it was there to run.
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
//! `HW-WAIT: <test>: …` once and then polls for the lock; the wait is **bounded**
//! by [`LOCK_TIMEOUT`], because a session that never ends is exactly the hung
//! case a run must hear about rather than sit in — so a wait that runs out says
//! another hardware test still holds the board instead of blocking forever.
//! `--test-threads=1` is therefore **not** needed (and passing it anyway is
//! harmless).

use std::fmt;
use std::fs::File;
use std::os::fd::AsRawFd;
use std::panic::AssertUnwindSafe;
use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use tokio::runtime::Runtime;

use crate::core::klippy::api::StartArgs;
use crate::core::klippy::config::mcu::McuConfig;
use crate::core::klippy::config::{Config, ConfigSection, ConfigWrapper};
use crate::core::klippy::extras::heaters::HEATERS_OBJECT;
use crate::core::klippy::extras::output_pin::OutputPin;
use crate::core::klippy::gcode::{GCodeDispatch, GCODE_OBJECT};
use crate::core::klippy::interface::Interface;
use crate::core::klippy::mcu::ids::NODEID_FIRST;
use crate::core::klippy::printer::{Printer, PrinterState};
use crate::core::klippy::reactor::TokioReactor;

/// The environment variable that names the printer config to test against.
///
/// There is no way to name an interface directly, because the config already
/// does. The one other variable this mode reads is [`HW_STRICT_ENV`], which
/// decides what an unmet [`Requires`] means.
pub const HW_CONFIG_ENV: &str = "KLIPPERX_HW_CONFIG";

/// The environment variable that turns strict mode off: `0`, `false` or `no`
/// (any case, surrounding blanks ignored).
///
/// Strict is the default and the point of the mode: a named config that does
/// not provide what a test declared it needs is a **failure**, so a run cannot
/// report "nothing to test" for hardware the user asked for. The escape hatch
/// is for the case where one config is used across several machines and the
/// tests it cannot run should merely be reported — `HW-IGNORED: …` and pass.
pub const HW_STRICT_ENV: &str = "KLIPPERX_HW_STRICT";

/// The config sections whose upstream objects this repository does not
/// implement.
///
/// The three deferred modules of
/// `docs/klippy/developer-manual/upstream-deviations.md` §1.2 (decided
/// 2026-10-22): each is bound to specific upstream hardware or a specific
/// runtime, so nothing here can be verified against a board and the code is not
/// written. A config that names one is not a config this mode can test — the
/// host would refuse it at start-up — so strict mode fails the case rather than
/// run against a config whose objects are missing.
const NOT_IMPLEMENTED_SECTIONS: &[&str] = &["samd_sercom", "replicape", "palette2"];

/// How long [`Machine::bring_up`] gives the machine to connect every part.
///
/// The host itself has no such bound — a board that is slow to answer identify
/// is a slow board, not a broken run — so this is a harness budget, in the same
/// spirit as the corpus' case bound: without it a machine that never answers
/// parks the test instead of failing it.
const BRING_UP_TIMEOUT: Duration = Duration::from_secs(10);

/// How long one hardware case may take in total, from the board lock to its
/// last assertion.
///
/// Deliberately wide, and the numbers are why. A board's identify handshake is
/// about a second; a `G28` on a big printer is tens of seconds of real motion;
/// and a temperature wait is minutes — a 40 W hotend climbing from room
/// temperature to 200 °C takes several minutes, and a heated bed can take ten.
/// The budget has to clear the slowest legitimate case by a lot, because a
/// false timeout fails a case that would have passed on a slow board, while a
/// budget that is too wide only costs time on a case that is already hung. Five
/// minutes is far above what the cases here do (the longest is a homing case at
/// tens of seconds) and still short enough that a hung case ends a run rather
/// than parking it.
const CASE_TIMEOUT: Duration = Duration::from_secs(300);

/// How often a case waiting for the board looks again.
const LOCK_POLL: Duration = Duration::from_millis(100);

/// How long a case waits for the board lock before it reports the board busy.
///
/// Longer than [`CASE_TIMEOUT`] on purpose: the lock is held for one whole
/// case, so a wait shorter than a case would report "busy" for a predecessor
/// that is merely slow — the false failure this bound is not allowed to make.
/// Twice a case is the point where the predecessor is not slow but stuck, and
/// the wait then says so instead of blocking forever.
const LOCK_TIMEOUT: Duration = Duration::from_secs(600);

/// The phase a case is in before [`acquire`] hands out the machine.
const PHASE_ACQUIRING: &str = "acquiring the machine (the config and the board lock)";

/// The phase a case is in while its own body runs.
const PHASE_BODY: &str = "the case body";

/// The phase [`Machine::bring_up`] is in.
const PHASE_BRING_UP: &str = "bring_up (connecting every part of the config)";

/// The phase [`Machine::open_mcu_named`] is in.
const PHASE_OPEN: &str = "opening the transport of one [mcu] section";

/// How long a [`StartedMachine`] gives its runtime to shut down, once the
/// machine's parts are dropped.
///
/// The same bound the corpus puts on a case's runtime
/// (`core::klippy::upstream`), for the same reason: a device's blocking read is
/// not cancellable by aborting a task, so the only way to tell "the parts went"
/// from "a part was leaked" is to give the runtime a budget and check that it
/// finished inside it.
const SHUTDOWN_TIMEOUT: Duration = Duration::from_secs(5);

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

    /// The main MCU, reachable: `[mcu]` carries `serial:` or `uuid:`.
    ///
    /// A `[mcu <name>]` does not count — that is another board, and opening it
    /// is not what [`Machine::open_mcu`] does. Ask for it with
    /// [`Requires::mcu_named`].
    pub fn mcu(self) -> Self {
        self.mcu_named("mcu")
    }

    /// The MCU named `name`, reachable: `[mcu <name>]` carries `serial:` or
    /// `uuid:`. The name `"mcu"` is `[mcu]`, the same as
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
    /// `uuid:`. The section identifier is `"mcu"` or `"mcu zboard"`.
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
                write!(f, "missing [{section}] with serial: or uuid:")
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
        .is_some_and(|found| found.has("serial") || found.has("uuid") || found.has("canbus_uuid"))
}

/// Append `item` unless that exact thing is already in the list, so a test that
/// asks for a section and an option of it reports the section once.
fn push_unique(missing: &mut Vec<Missing>, item: Missing) {
    if !missing.contains(&item) {
        missing.push(item);
    }
}

/// The sections of `config` whose objects this repository does not implement,
/// in the config's own order.
///
/// Read off the parsed section table rather than by grepping the file: a
/// commented-out header is absent, and an `[include …]`d one is present, exactly
/// as for every other question this module asks of a config.
fn not_implemented_sections(config: &Config) -> Vec<String> {
    config
        .sections()
        .map(ConfigSection::identifier)
        .filter(|identifier| NOT_IMPLEMENTED_SECTIONS.contains(&identifier.as_str()))
        .collect()
}

/// How a strict-mode panic names the config it is about: `[`HW_CONFIG_ENV`]
/// with the path it holds.
fn named_config(path: Option<&Path>) -> String {
    path.map_or_else(
        || HW_CONFIG_ENV.to_string(),
        |path| format!("{HW_CONFIG_ENV} ({})", path.display()),
    )
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

/// Refuse to start a machine from inside a runtime.
///
/// [`Machine::bring_up`] builds and owns the runtime the machine runs on, and
/// takes it down with a bounded blocking `shutdown_timeout`; both are illegal
/// from inside another runtime's context (`Cannot start a runtime from within a
/// runtime`, then `Cannot drop a runtime in a context where blocking is not
/// allowed`). A test written as `#[tokio::test]` is a mistake in the test, not a
/// failure of the board, so it gets a readable `Err` here rather than tokio's
/// panic. Called as the first thing [`Machine::bring_up`] does, before any
/// device is touched, so the refusal can be exercised without a board.
fn ensure_not_in_runtime() -> Result<(), String> {
    if tokio::runtime::Handle::try_current().is_ok() {
        return Err(
            "Machine::bring_up owns a runtime and shuts it down on drop, so it \
                    cannot run inside one: write the case as a plain `#[test]` (L2 cases \
                    are plain tests; see the module docs)"
                .to_string(),
        );
    }
    Ok(())
}

/// Take `runtime` down, bounded by `timeout`, and say whether it finished inside
/// the bound.
///
/// One measure for both callers — [`Machine::bring_up`]'s failure path and
/// [`StartedMachine`]'s `Drop` — so "the parts went" and "a part was leaked" are
/// told apart by the same rule: a device's blocking read cannot be cancelled, so
/// a runtime that is still alive when the bound runs out is holding one. `Err`
/// carries the elapsed time, for the message.
fn shut_down(runtime: Runtime, timeout: Duration) -> Result<(), Duration> {
    let started = Instant::now();
    runtime.shutdown_timeout(timeout);
    let elapsed = started.elapsed();
    if elapsed < timeout {
        Ok(())
    } else {
        Err(elapsed)
    }
}

/// The `Err` a failed [`Machine::bring_up`] returns, once its machine is down.
///
/// `reason` is the failure that stopped the bring-up and is always kept as the
/// `Err`'s beginning; the shutdown's own verdict is appended, never substituted,
/// because the two are different faults. A runtime that does not close inside
/// `timeout` is a leaked part, and saying so is the point: the failure path has
/// no assertion to make — the failure is already being reported — so it reports
/// this one in the message instead of swallowing it.
///
/// `timeout` is a parameter so the bound can be exercised without waiting
/// [`SHUTDOWN_TIMEOUT`] out; [`Machine::bring_up`] passes exactly that.
fn bring_up_failure(reason: String, runtime: Runtime, timeout: Duration) -> String {
    match shut_down(runtime, timeout) {
        Ok(()) => reason,
        Err(elapsed) => format!(
            "{reason}; and the failed machine's runtime did not shut down within \
             {timeout:?} ({elapsed:?}): a part was leaked, so its blocking device read \
             is still parked and the next test would trip over it. This is \
             `StartedMachine`'s own measure, which fails a case for it."
        ),
    }
}

/// Where a case has got to, as the case timeout names it.
///
/// The phase is shared with the runner, which is what makes a timeout report
/// "stuck at bring_up" instead of only "stuck": the case body runs on its own
/// thread, so the thread watching the budget cannot ask it anything — the
/// framework's own seams stamp the phase as the case passes through them.
struct CaseProgress {
    /// The phase's name and the instant it was entered.
    current: Mutex<(&'static str, Instant)>,
}

impl CaseProgress {
    /// Start the record in `phase`.
    fn new(phase: &'static str) -> Arc<Self> {
        Arc::new(Self {
            current: Mutex::new((phase, Instant::now())),
        })
    }

    /// Record that the case has reached `phase`.
    fn enter(&self, phase: &'static str) {
        *self
            .current
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = (phase, Instant::now());
    }

    /// The phase the case is in, and how long it has been in it.
    fn current(&self) -> (&'static str, Duration) {
        let guarded = self
            .current
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        (guarded.0, guarded.1.elapsed())
    }
}

/// A config that satisfies a test's [`Requires`], with the board locked.
pub struct Machine {
    /// The parsed printer config.
    config: Config,
    /// The config file the parser read, as the host's start arguments name it.
    path: PathBuf,
    /// Where this case has got to, for the case timeout's report.
    progress: Arc<CaseProgress>,
    /// The exclusive lock on the board, held until this machine is dropped.
    _board: BoardLock,
}

/// Set the transport facts the load path would have put on `mcu` for `section`.
///
/// The load path decides "this `serial:` path is addressed by id" over the
/// **whole** config (`SerialPorts::classify`, driven from `mcu/object.rs`) and
/// stamps the answer on the MCU object; a test opens one section on its own, so
/// it has to ask the same question of the same pass. Without this a bridge's mux
/// firmware is opened with bare frames, answers nothing, and the session times
/// out in identify — which is how this lane used to fail on a muxed board.
///
/// The implementation is [`crate::stress::apply_id_addressed_transport`]'s: this
/// module is `#[cfg(test)]`, that one is not, and the release `klipperx stress`
/// asks exactly the same question and must get the same answer.
pub(crate) use crate::stress::apply_id_addressed_transport;

impl Machine {
    /// The whole parsed printer config.
    pub fn config(&self) -> &Config {
        &self.config
    }

    /// Record this case's progress in the runner's `progress` from here on.
    ///
    /// [`acquire`] builds a machine with a phase record of its own; the runner
    /// swaps in the one its timeout report watches, so the phases
    /// [`Machine::bring_up`] and [`Machine::open_mcu_named`] stamp land where
    /// that report reads them.
    fn traced_by(&mut self, progress: &Arc<CaseProgress>) {
        self.progress = Arc::clone(progress);
    }

    /// Start the machine the config describes, as the host does.
    ///
    /// The whole second layer: the parts the config names are loaded and
    /// connected, `Ready` is reached, and every `[mcu …]` section is up (not
    /// just the one [`Machine::open_mcu`] would open). The config is the one
    /// [`acquire`] already parsed; the file is never read again.
    ///
    /// The board lock is *not* touched: it stays held by this [`Machine`] until
    /// the test's `Machine` is dropped, so the session `bring_up` starts is the
    /// one the lock covers.
    ///
    /// # Errors
    /// A real failure, never a skip: the config named this hardware, so a
    /// config that will not load, a board that will not open, a bring-up that
    /// does not reach `Ready` inside [`BRING_UP_TIMEOUT`], or a runtime that
    /// cannot be built is an `Err` with the reason. So is a call made from
    /// inside a runtime — a case written as `#[tokio::test]` rather than a plain
    /// `#[test]` — which is refused before any device is touched. A config that
    /// does not satisfy [`Requires`] never gets here — [`acquire`] reports that
    /// as `HW-IGNORED` and hands out no [`Machine`] at all.
    ///
    /// A failed machine is taken down before the `Err` is returned, on the bound
    /// [`StartedMachine`]'s `Drop` uses; a runtime that does not close inside it
    /// is a leaked part, and the `Err` says so after the original reason.
    pub fn bring_up(&self) -> Result<StartedMachine, String> {
        self.progress.enter(PHASE_BRING_UP);
        // First, before any device or config work: a bring-up from inside a
        // runtime cannot work, and a plain `Err` beats tokio's panic (see the
        // module doc's rule for second-layer cases).
        ensure_not_in_runtime()?;
        // The machine gets a runtime of its own, as a corpus case does: this is
        // the live shape, so a part that is leaked leaves a blocking device read
        // parked, and a `#[tokio::test]` runtime drops by waiting for exactly
        // that thread forever. Owning the runtime is what lets `Drop` bound the
        // wait instead.
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .map_err(|error| format!("the machine's runtime: {error}"))?;
        let printer = Arc::new(Printer::new(Arc::new(TokioReactor::new(
            runtime.handle().clone(),
        ))));
        // The host's own start arguments, minus file output: `debug_output`
        // stays unset, which is what makes this the live shape rather than the
        // corpus' `-o` one (see the module doc's table).
        printer.set_start_args(Arc::new(StartArgs::collect(
            self.path.display().to_string(),
            None,
            crate::core::klippy::api::endpoints::info::NO_KLIPPER_PATH,
        )));

        let setup = runtime.block_on(async {
            printer
                .load_config(&self.config)
                .map_err(|e| e.to_string())?;
            if tokio::time::timeout(BRING_UP_TIMEOUT, printer.bring_up())
                .await
                .is_err()
            {
                return Err(format!(
                    "bring_up did not finish within {BRING_UP_TIMEOUT:?}: a part never \
                     connected"
                ));
            }
            let state = printer.get_state_message();
            if state.category != PrinterState::Ready {
                return Err(format!("the machine did not come up: {}", state.message));
            }
            Ok(())
        });

        match setup {
            Ok(()) => Ok(StartedMachine {
                printer: Some(printer),
                runtime: Some(runtime),
            }),
            Err(reason) => {
                // Take the failed machine down here, exactly as a
                // `StartedMachine` does: the parts go while the runtime is
                // still alive (their `Drop` is what shuts a transport down),
                // then the runtime is given the same bound. A failure must not
                // leave the next test a parked read to trip over.
                printer.teardown();
                drop(printer);
                Err(bring_up_failure(reason, runtime, SHUTDOWN_TIMEOUT))
            }
        }
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
    /// `uuid:`, `host_library:` and `restart_method:` all behave as they do at
    /// run time.
    ///
    /// # Errors
    /// A failure here is a **real** failure, not a skip: the config said the
    /// board was there, so a port that will not open is a broken test
    /// environment. The message is the parser's or the transport's own, except
    /// that a name with no section says which section is missing.
    pub fn open_mcu_named(&self, name: &str) -> Result<Interface, String> {
        self.progress.enter(PHASE_OPEN);
        let section_id = mcu_section_id(name);
        let section = self
            .config
            .get_section(&section_id)
            .ok_or_else(|| format!("the config has no [{section_id}] section"))?;
        let mut config =
            McuConfig::new(&ConfigWrapper::untracked(section)).map_err(|e| e.to_string())?;
        // Whether this `serial:` path is addressed by id is the **whole
        // config's** call at load time (`mcu/object.rs`, `serial_ports`), and a
        // hardware test opens one section on its own. Asking the same question
        // here is what lets the lane reach a bridge's mux firmware at all: a
        // muxed console answers bare frames with silence, so the open would
        // otherwise time out in identify.
        apply_id_addressed_transport(&self.config, section, &mut config);
        config.open()
    }
}

/// A machine [`Machine::bring_up`] has started, with the runtime it runs on.
///
/// The live half of the framework: the printer is past `Ready`, so a test can
/// run g-code through it ([`StartedMachine::printer`] hands out the `Printer`,
/// whose `gcode`, `toolhead` and extras are the host's), and the platform — the
/// transport tasks and the blocking device reads they park — is this layer's to
/// take down.
///
/// A `StartedMachine` must be **dropped at the end of the test** (or in a scope
/// that ends where the session should): that is what tears the machine down,
/// on the ordinary path and on the panic path alike. It is deliberately not
/// `Clone` — there is one session on the board, and one owner of it.
pub struct StartedMachine {
    /// The running machine. Taken out first in `Drop`, so it is released while
    /// the runtime is still alive.
    printer: Option<Arc<Printer>>,
    /// The runtime this machine's transports and device I/O run on. Its
    /// shutdown is bounded, so a leaked part fails instead of hanging.
    runtime: Option<Runtime>,
}

impl StartedMachine {
    /// The running machine: its `gcode`, `toolhead` and extras are registered
    /// and connected, exactly as the host has them.
    ///
    /// The handle is valid until this `StartedMachine` is dropped: `Drop`
    /// tears the machine's parts down, so an `Arc<Printer>` kept past that
    /// point names a machine that has been taken apart.
    pub fn printer(&self) -> &Arc<Printer> {
        self.printer
            .as_ref()
            .expect("the machine is alive until its `StartedMachine` is dropped")
    }
}

impl Drop for StartedMachine {
    /// Turn the machine off, take it down, then bound the runtime's shutdown.
    ///
    /// **The machine is turned off first** ([`turn_everything_off`]): every
    /// heater's target goes to zero and every `[output_pin]` that is not
    /// already at zero is driven back to zero, through the same g-code the
    /// host's own client would send (`TURN_OFF_HEATERS`, `SET_PIN … VALUE=0`).
    /// This runs on **both** paths — a case that passes and a case that panics —
    /// because it is here, in `Drop`, and not in the case body: a case that set
    /// a target and then failed on the next assertion would otherwise leave the
    /// board heating, which is the one thing a test lane must never do. The
    /// commands are best effort and report as `HW-TEARDOWN:` lines; a machine
    /// that is already shut down refuses them, which is another off state.
    ///
    /// Then, in this order, and for this reason: the parts the config loaded
    /// are what shut a device down when they drop (`Mcu::Drop` →
    /// `Interface::shutdown`, which releases the blocking read the session
    /// parked), so they have to go while the runtime is still alive to carry
    /// that — and only then does the runtime get its bound. Dropping the machine
    /// as well is what makes the bound mean something: whatever is still holding
    /// a device afterwards was leaked by a part, not kept alive by this
    /// framework.
    ///
    /// A runtime that does not finish inside [`SHUTDOWN_TIMEOUT`] is a **panic**
    /// naming the leak: the live shape's blocked read cannot be cancelled, so
    /// waiting longer would only hide it, and a hang would hide it entirely.
    /// The panic is skipped while unwinding from the test body's own panic —
    /// the failure is already reported, and a second panic from `Drop` would
    /// abort the process instead of reporting either.
    fn drop(&mut self) {
        if let Some(printer) = self.printer.take() {
            turn_everything_off(&printer);
            printer.teardown();
            drop(printer);
        }
        let Some(runtime) = self.runtime.take() else {
            return;
        };
        let leaked = shut_down(runtime, SHUTDOWN_TIMEOUT).err();
        if std::thread::panicking() {
            return;
        }
        let Some(elapsed) = leaked else {
            return;
        };
        panic!(
            "the machine's runtime did not shut down within {SHUTDOWN_TIMEOUT:?} \
             ({elapsed:?}): a part was leaked, so its blocking device read is still parked \
             and the board's session was never closed. The heater ring this used to name \
             (`verify_heater`'s 1 s timer holding a strong `Arc<Heater>`) is fixed, so a \
             regression there is a bug rather than a known cost. The one ring of that shape \
             known to remain is the API subscription timer: `api/endpoints/objects_subscribe.rs`'s \
             callback holds an `Arc<Inner>`, and `Inner` holds the `printer` and the \
             `reactor`, so `reactor → heap → Inner → reactor` keeps the old `Mcu` alive \
             after an in-place restart (`TODO.md`'s S5). Do not paper over this with \
             `Mcu::close`; the cycle itself is what has to go. See \
             `docs/klippy/developer-manual/testing.md`."
        );
    }
}

/// Turn the machine off before it is torn down: every heater's target to zero,
/// every output that is on back to zero.
///
/// What it fixes: a case that set `M104 S200`, or drove a fan, and then failed
/// on the next assertion used to leave the board doing exactly that for as long
/// as the printer stayed powered. `Drop` is where this belongs, because it is
/// the one piece of a case that runs on both paths — the ordinary return and the
/// panic — and it runs before the parts go.
///
/// The two commands are the host's own, not a private back door:
/// `TURN_OFF_HEATERS` is what upstream's `cmd_TURN_OFF_HEATERS` runs
/// (`heaters.py:283-285`, giving every registered heater a target of 0), and
/// `SET_PIN PIN=<name> VALUE=0` is upstream's `output_pin` command
/// (`output_pin.py:196-269`). Both are read off the machine's own status: the
/// heater registry's presence, and the value each `[output_pin]` last drove —
/// only pins that are not already at zero are sent.
///
/// Best effort, never a panic: this runs while unwinding as often as not, and a
/// machine that is already shut down refuses commands. Each line it sends, and
/// each refusal, is printed as an `HW-TEARDOWN:` line so a run says what it put
/// back.
fn turn_everything_off(printer: &Arc<Printer>) {
    let heater_registry = printer.lookup_object(HEATERS_OBJECT).is_some();
    let outputs: Vec<(String, f64)> = printer
        .lookup_objects_as::<OutputPin>(Some("output_pin"))
        .into_iter()
        .map(|(registered, pin)| {
            let value = printer
                .status_of(&registered, printer.eventtime())
                .and_then(|status| status.get("value").and_then(serde_json::Value::as_f64))
                .unwrap_or(0.0);
            (pin.name().to_string(), value)
        })
        .collect();
    let lines = turn_off_script(heater_registry, &outputs);
    if lines.is_empty() {
        return;
    }
    let Some(gcode) = printer.lookup_object_as::<GCodeDispatch>(GCODE_OBJECT) else {
        println!("HW-TEARDOWN: no g-code dispatcher, so nothing can be turned off");
        return;
    };
    drive_teardown(lines, |line| {
        gcode
            .run_script_sync(line)
            .map_err(|error| error.to_string())
    });
}

/// The g-code [`turn_everything_off`] sends, from what the machine reports.
///
/// `heater_registry` is whether the config built a `heaters` registry at all
/// (`[extruder]`'s `heaters::ensure` does, so a config with no heater section
/// has nothing to turn off); `outputs` is every `[output_pin]`'s name and the
/// value it last drove. A pin already at zero is left alone — the command would
/// be a no-op, and a run's `HW-TEARDOWN:` lines should say what changed.
fn turn_off_script(heater_registry: bool, outputs: &[(String, f64)]) -> Vec<String> {
    let mut lines = Vec::new();
    if heater_registry {
        lines.push("TURN_OFF_HEATERS".to_string());
    }
    for (name, value) in outputs {
        if *value != 0.0 {
            lines.push(format!("SET_PIN PIN={name} VALUE=0"));
        }
    }
    lines
}

/// Run `lines` through `run`, reporting each one rather than panicking.
///
/// `run` is a parameter so the driving can be exercised without a machine: a
/// test records what was sent, and sees a refusal reported rather than raised.
fn drive_teardown<F>(lines: Vec<String>, mut run: F)
where
    F: FnMut(&str) -> Result<(), String>,
{
    for line in lines {
        match run(&line) {
            Ok(()) => println!("HW-TEARDOWN: {line}"),
            Err(error) => println!("HW-TEARDOWN: {line} failed: {error}"),
        }
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
/// `test_name` is only used in the message. Blocking, but not unboundedly: a
/// board held for longer than [`LOCK_TIMEOUT`] is a session that never ended,
/// and the `Err` says so instead of parking this case forever. No other lock is
/// held while waiting here.
fn lock_board(config: &Path, test_name: &str) -> Result<BoardLock, String> {
    lock_board_within(config, test_name, LOCK_TIMEOUT)
}

/// [`lock_board`] with the bound as a parameter, so the wait can be exercised
/// without waiting [`LOCK_TIMEOUT`] out.
fn lock_board_within(
    config: &Path,
    test_name: &str,
    timeout: Duration,
) -> Result<BoardLock, String> {
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

    let deadline = Instant::now() + timeout;
    loop {
        // A polled non-blocking take rather than a blocking `flock`, because a
        // blocking one cannot be given a deadline: the thread would sit inside
        // the syscall with no way to look at the clock.
        //
        // SAFETY: as above; `fd` is owned by `file`, which is still alive.
        if unsafe { libc::flock(fd, libc::LOCK_EX | libc::LOCK_NB) } == 0 {
            return Ok(BoardLock { file });
        }
        let error = std::io::Error::last_os_error();
        if error.raw_os_error() != Some(libc::EWOULDBLOCK) {
            return Err(format!("{}: flock: {error}", config.display()));
        }
        if Instant::now() >= deadline {
            return Err(format!(
                "another hardware test still holds the board: waited {timeout:?} for {} and its \
                 session never ended. Free it (a hung `cargo test` left behind?) and run again",
                config.display()
            ));
        }
        std::thread::sleep(LOCK_POLL);
    }
}

/// The `HW-RUN` line [`acquire_at`] prints once a test is about to run.
///
/// One line, greppable by its `HW-` prefix. It names the test and carries the
/// premise of the whole mode with it — see this module's "This tests the module,
/// not your config": the config is taken as correct, so the run checks the host
/// module rather than the user's config.
fn run_line(test_name: &str) -> String {
    format!(
        "HW-RUN: {test_name}  \
         (the config is assumed correct; this checks the host module, not your config)"
    )
}

/// What [`HW_STRICT_ENV`] asks for: strict unless it says otherwise.
///
/// Unset, empty or anything else is strict (the default); only an explicit `0`,
/// `false` or `no` is loose, so a typo in the escape hatch asks for the strict
/// run rather than turning it off by accident. Pure: [`acquire`] reads the
/// variable and hands the answer down.
fn strict_from(value: Option<&str>) -> bool {
    match value {
        None => true,
        Some(value) => !matches!(
            value.trim().to_ascii_lowercase().as_str(),
            "0" | "false" | "no"
        ),
    }
}

/// Hand `test_name` a machine for the configured board, or report it as skipped.
///
/// The only reader of [`HW_CONFIG_ENV`] and [`HW_STRICT_ENV`]: the first names
/// the config, the second decides what an unmet [`Requires`] means, and
/// [`acquire_at`] decides what the config offers. Returns `None` — and prints
/// one `HW-IGNORED: <test>: <why>` line — when the variable is unset, or when
/// the config does not provide `requires` and [`HW_STRICT_ENV`] turns strict
/// mode off. A skipped test touches no device and takes no lock.
///
/// Returns `Some(machine)` with the board locked exclusively when everything is
/// in place, printing the [`run_line`] first. The lock is held until the machine
/// is dropped, so the test body and its cleanup are one session on the board.
///
/// # Panics
/// When the variable is set but names a config that cannot be used (missing
/// file, parser error), when it names a config that does not provide `requires`
/// **or** that names an object this repository does not implement
/// ([`NOT_IMPLEMENTED_SECTIONS`]) while strict mode is on (the default; see
/// [`HW_STRICT_ENV`] for the escape hatch), or when the board cannot be locked:
/// the variable said the machine had a config to test against, so "there was
/// nothing to test" is a failure, not a skip.
pub fn acquire(test_name: &str, requires: &Requires) -> Option<Machine> {
    let path = std::env::var(HW_CONFIG_ENV).ok();
    let strict = strict_from(std::env::var(HW_STRICT_ENV).ok().as_deref());
    acquire_at(path.as_deref().map(Path::new), test_name, requires, strict)
}

/// [`acquire`], on a path and a strictness instead of the environment.
///
/// The environment reads are `acquire`'s alone; this is the rest of it, taking
/// the path and the strictness as arguments so the branches can be exercised
/// without setting a variable (which no test can do without racing the others).
///
/// Under `strict`, two things fail that would otherwise run or skip: a config
/// that does not provide `requires`, and a config that names an object this
/// repository does not implement ([`NOT_IMPLEMENTED_SECTIONS`]). Both are the
/// same premise — the config was named, so it is the machine this test was meant
/// to run against — and both point at [`HW_STRICT_ENV`] for the way out.
fn acquire_at(
    path: Option<&Path>,
    test_name: &str,
    requires: &Requires,
    strict: bool,
) -> Option<Machine> {
    match decide(path, requires) {
        Decision::NotSet => {
            // Unset is a skip in both modes: an ordinary `cargo test` runs these
            // `#[ignore]`d tests on a machine that has no board, and what that
            // must report is "no config", not a failure.
            println!(
                "HW-IGNORED: {test_name}: {HW_CONFIG_ENV} is not set \
                 (set it to a printer config to run the hardware tests)"
            );
            None
        }
        Decision::Unreadable(reason) => {
            // The variable named a config, so this machine was supposed to have
            // one. Only an unset variable and an unmet `Requires` are skips; a
            // path that does not exist or a file that does not parse is a
            // failure — a green run must not report "nothing to test" for a
            // config the user told it to use.
            panic!("{test_name}: {HW_CONFIG_ENV} is set but its config cannot be used: {reason}")
        }
        Decision::Missing(missing) => {
            let reasons: Vec<String> = missing.iter().map(ToString::to_string).collect();
            let reasons = reasons.join(", ");
            if strict {
                // Same premise as the unreadable case, one step further in: the
                // config was named and it parsed, so this test was meant to run
                // against this machine and the config is simply short of what
                // it declared. Skipping would hide that — a green run would say
                // the hardware was tested when the test never ran.
                let named = named_config(path);
                panic!(
                    "{test_name}: {named} does not provide what this test needs: {reasons}. \
                     The config was named, so this is a failure rather than a skip; \
                     set {HW_STRICT_ENV}=0 to skip instead"
                );
            }
            println!("HW-IGNORED: {test_name}: {reasons}");
            None
        }
        Decision::Ready { path, config } => {
            // The config provides what this test asked for — but an object the
            // repository does not implement makes it a config this mode cannot
            // honestly test against: the host would refuse it at start-up, so
            // the case would fail deep in `bring_up` for a reason that has
            // nothing to do with the module under test. Say so up front, under
            // the same strictness as an unmet `Requires`.
            if strict {
                let unimplemented = not_implemented_sections(&config);
                if !unimplemented.is_empty() {
                    panic!(
                        "{test_name}: {} names objects this repository does not implement: {}. \
                         These are the modules `upstream-deviations.md` §1.2 defers, so no case \
                         here can run against this config; set {HW_STRICT_ENV}=0 to skip instead",
                        named_config(Some(&path)),
                        unimplemented.join(", "),
                    );
                }
            }
            // Take the lock **before** the test runs: the board can only carry
            // one session, so a test that is about to run must own it. A config
            // that parsed a moment ago but cannot be opened for locking is the
            // one thing here that is neither a skip nor the test's own failure,
            // so say what went wrong and stop.
            let _board = lock_board(&path, test_name).unwrap_or_else(|reason| {
                panic!("{test_name}: cannot serialise on the board's config file: {reason}")
            });
            println!("{}", run_line(test_name));
            let progress = CaseProgress::new(PHASE_BODY);
            Some(Machine {
                config,
                path,
                progress,
                _board,
            })
        }
    }
}

/// Run `body` as one hardware case: the board locked, and a total budget.
///
/// This is the frame a hardware case is written in, and it replaces the
/// `let Some(machine) = acquire(…) else { return }; …` shape:
///
/// ```ignore
/// #[test]
/// #[ignore = "hardware: needs KLIPPERX_HW_CONFIG"]
/// fn test_homing_reaches_the_endstop_on_a_real_board() {
///     crate::hardware_test::case(
///         "test_homing_reaches_the_endstop_on_a_real_board",
///         &crate::hardware_test::Requires::new().mcu(),
///         move |machine| {
///             let started = machine.bring_up().expect("the machine comes up");
///             // … the case …
///         },
///     );
/// }
/// ```
///
/// `body` runs on a thread of its own, and this one waits for it for at most
/// [`CASE_TIMEOUT`]: a case that parks in a device read or in a temperature
/// wait is then **reported** — `HW-TIMEOUT: <case>: still at "<phase>" …` — and
/// the run goes on. The thread that is still blocked cannot be stopped (a
/// blocking read is not cancellable), so it is abandoned: it keeps the board
/// lock until it ends, which is why the next case's wait for the board is
/// bounded too ([`LOCK_TIMEOUT`]) and says the board is still held.
///
/// A case whose [`Requires`] the config does not provide fails, exactly as
/// [`acquire`] alone reports it — a panic with the missing items in it, unless
/// [`HW_STRICT_ENV`] turns strict mode off, in which case it is `HW-IGNORED`
/// and this returns.
///
/// A panic in `body` — `expect` failing, an assertion — is re-raised here with
/// the same message, so the case fails the ordinary way. The machine is dropped
/// (and its session torn down) inside the case thread on the panic path as
/// well, before that message travels.
///
/// `body` takes the `Machine` by value; it lives on the case thread for exactly
/// as long as `body` runs (and is dropped there, teardown and all), so anything
/// the case opens has to be closed inside it — that is what makes the teardown
/// the machine's `Drop` covers complete.
///
/// # Panics
/// When the case times out, when `body` panics, and when the machine cannot be
/// built (a set [`HW_CONFIG_ENV`] naming a config that cannot be used, a config
/// that does not provide `requires` under strict mode, or a board that cannot be
/// locked).
pub fn case<F>(test_name: &str, requires: &Requires, body: F)
where
    F: FnOnce(Machine) + Send + 'static,
{
    run_case(None, false, test_name, requires, CASE_TIMEOUT, body);
}

/// [`case`] for a case whose body is `async` — an L1 case, which opens a
/// transport with `Mcu::connect(…).await`.
///
/// The body is polled on a current-thread runtime built **inside** the case
/// thread, which is the same runtime a `#[tokio::test]` would have given it:
/// same IO and timers enabled, same single thread. Nothing about the body
/// changes; only its frame does. A case that calls [`Machine::bring_up`] is not
/// one of these — that method owns a runtime and refuses to run inside one, so
/// such a case is a plain [`case`].
pub fn async_case<F, Fut>(test_name: &str, requires: &Requires, body: F)
where
    F: FnOnce(Machine) -> Fut + Send + 'static,
    Fut: std::future::Future<Output = ()>,
{
    run_case(
        None,
        false,
        test_name,
        requires,
        CASE_TIMEOUT,
        move |machine| {
            let runtime = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .expect("the case's runtime builds");
            runtime.block_on(body(machine));
        },
    );
}

/// The end of a case body, as the case thread reports it.
#[derive(Debug)]
enum CaseOutcome {
    /// The body ran to its end (or the case was skipped).
    Finished,
    /// The body panicked; the message is what the case failed with.
    Panicked(String),
}

/// [`case`]'s body: run `body` on its own thread under `budget`.
///
/// `path` is the config to run against; `None` reads [`HW_CONFIG_ENV`] inside
/// the case thread, the way [`acquire`] does. The parameter is what lets a test
/// exercise this without setting a variable. `strict` is the caller's choice on
/// that `Some` branch; on the `None` branch it is unused, because the strictness
/// then comes from [`HW_STRICT_ENV`] inside [`acquire`] — the environment reads
/// belong there.
fn run_case<F>(
    path: Option<PathBuf>,
    strict: bool,
    test_name: &str,
    requires: &Requires,
    budget: Duration,
    body: F,
) where
    F: FnOnce(Machine) + Send + 'static,
{
    let name = test_name.to_string();
    let requires = requires.clone();
    let progress = CaseProgress::new(PHASE_ACQUIRING);
    let case_progress = Arc::clone(&progress);
    let (sender, receiver) = mpsc::channel::<CaseOutcome>();

    let spawned = std::thread::Builder::new()
        .name(format!("hw-case {test_name}"))
        .spawn(move || {
            // The machine lives inside this closure: its `Drop` — the board
            // lock and, for a machine that came up, the session's teardown —
            // runs here, before the outcome travels, on the panic path too.
            let outcome = std::panic::catch_unwind(AssertUnwindSafe(|| {
                let machine = match path {
                    Some(path) => acquire_at(Some(&path), &name, &requires, strict),
                    None => acquire(&name, &requires),
                };
                if let Some(mut machine) = machine {
                    // The machine's phase record becomes the runner's from here
                    // on: the acquiring phase was already the runner's own, and
                    // the body's phases are what it has to report.
                    case_progress.enter(PHASE_BODY);
                    machine.traced_by(&case_progress);
                    body(machine);
                }
            }));
            let outcome = match outcome {
                Ok(()) => CaseOutcome::Finished,
                Err(payload) => CaseOutcome::Panicked(panic_text(payload.as_ref())),
            };
            // A closed receiver means the budget already ran out and nobody is
            // listening; the case thread ends here either way.
            let _ = sender.send(outcome);
        });
    let case_thread = match spawned {
        Ok(case_thread) => case_thread,
        Err(error) => panic!("{test_name}: cannot start the case's thread: {error}"),
    };

    match receiver.recv_timeout(budget) {
        Ok(CaseOutcome::Finished) => {
            // The body is done; the thread has nothing left but its return.
            let _ = case_thread.join();
        }
        Ok(CaseOutcome::Panicked(message)) => {
            let _ = case_thread.join();
            // The case's own failure, re-raised here so it is this test's
            // failure rather than a silent thread's.
            panic!("{message}");
        }
        Err(RecvTimeoutError::Timeout) => {
            let (phase, in_phase) = progress.current();
            let message = format!(
                "HW-TIMEOUT: {test_name}: still at \"{phase}\" after {in_phase:?} in that phase \
                 ({budget:?} of case budget). The case thread is abandoned and still holds the \
                 board, so the next case will report the board as held rather than run: the \
                 case itself hung, which is what this budget is for — look at the phase above \
                 for where"
            );
            println!("{message}");
            panic!("{message}");
        }
        Err(RecvTimeoutError::Disconnected) => {
            panic!("{test_name}: the case thread died without reporting an outcome")
        }
    }
}

/// The message a caught panic carried, for the case [`run_case`] re-raises it
/// with.
fn panic_text(payload: &(dyn std::any::Any + Send)) -> String {
    payload
        .downcast_ref::<String>()
        .cloned()
        .or_else(|| payload.downcast_ref::<&str>().map(|text| text.to_string()))
        .unwrap_or_else(|| "a panic with no message".to_string())
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
            println!(
                "HW-CONFIG: these cases test the host module, not this config \
                 — a config they read is assumed correct"
            );
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
/// requirement passes (`serial:` or `uuid:`) or skips. A config with no
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
            } else if section.has("uuid") || section.has("canbus_uuid") {
                // `canbus_uuid` is the old spelling of the key; both name the
                // same uuid.
                let uuid = section
                    .get_str("uuid")
                    .or_else(|| section.get_str("canbus_uuid"));
                (
                    format!(
                        "uuid: {} (interface: {}, mcu id: {})",
                        uuid.unwrap_or_default(),
                        section.get_str("canbus_interface").unwrap_or("can0"),
                        section
                            .get_str("canbus_nodeid")
                            .unwrap_or("unset (assigned at load by [mcu_ids])"),
                    ),
                    format!("{requirement} tests run"),
                )
            } else {
                (
                    "no serial:/uuid:".to_string(),
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
    use std::sync::{Mutex, Weak};

    use crate::core::klippy::interface::devices::host::HostMux;
    use crate::core::klippy::interface::devices::responder_mcu::ResponderMcu;
    use crate::core::klippy::mcu::host_libraries::HostLibraries;
    use crate::core::klippy::mcu::{Mcu, McuObject};

    /// Parse fixture text with the real parser.
    fn config(text: &str) -> Config {
        Config::from_text(text).expect("the fixture parses").0
    }

    /// The transport facts `Machine::open_mcu_named` stamps on a section.
    fn transport_facts(text: &str) -> (Option<u32>, Option<u32>) {
        transport_facts_of(&config(text), "mcu")
    }

    /// The same for the board called `name`, in the config `parsed`.
    fn transport_facts_of(parsed: &Config, name: &str) -> (Option<u32>, Option<u32>) {
        let identifier = mcu_section_id(name);
        let section = parsed
            .get_section(&identifier)
            .unwrap_or_else(|| panic!("the fixture has [{identifier}]"));
        let mut mcu = McuConfig::new(&ConfigWrapper::with_config(
            section,
            crate::core::klippy::config::AccessTracking::shared(),
            None,
            parsed,
        ))
        .expect("the section parses");
        apply_id_addressed_transport(parsed, section, &mut mcu);
        (mcu.shared_baud, mcu.id)
    }

    /// A path one `[mcu]` names, with no port section: opened exclusively, no
    /// id — exactly what the load path decides.
    #[test]
    fn test_a_plain_serial_path_keeps_the_exclusive_open() {
        assert_eq!(
            transport_facts("[mcu]\nserial: /dev/ttyACM0\n"),
            (None, None)
        );
    }

    /// A path a bridge's port section names is addressed by id, so the lane must
    /// open it the way the load path does — otherwise a mux firmware answers
    /// nothing and every case times out in identify.
    #[test]
    fn test_a_bridge_path_is_opened_by_id() {
        let facts = transport_facts(
            "[mcu]\nserial: /dev/ttyACM0\n\
             [uart_device uart0]\nuart_mcu: mcu\nuart_oid: uart0\n",
        );
        assert_eq!(facts, (Some(250_000), Some(NODEID_FIRST)));
    }

    /// Two boards on one shared port are addressed by **different** ids: the
    /// load path hands every `[mcu]` its own number in declaration order, so
    /// stamping both with the sequence's first id put two sessions on one id —
    /// the second board's identify answered for the first.
    #[test]
    fn test_two_boards_on_one_shared_port_get_the_ids_the_load_path_gives_them() {
        let parsed = config(
            "[mcu]\nserial: /dev/ttyACM0\n\
             [mcu tool]\nserial: /dev/ttyACM0\n",
        );

        assert_eq!(transport_facts_of(&parsed, "mcu").1, Some(NODEID_FIRST));
        assert_eq!(
            transport_facts_of(&parsed, "tool").1,
            Some(NODEID_FIRST + 1),
            "the second section of the sequence, not the first id again"
        );
    }

    /// The number is the **allocator's**, not the section's index: an explicit
    /// `canbus_nodeid` anywhere in the config is reserved before the first auto
    /// id, so the shared port's boards step over it exactly as they do at load.
    #[test]
    fn test_an_explicit_nodeid_is_stepped_over_when_stamping_a_shared_port() {
        let parsed = config(
            "[mcu]\nserial: /dev/ttyACM0\n\
             [mcu tool]\nserial: /dev/ttyACM0\n\
             [mcu can]\nuuid: 11aa22bb33cc\ncanbus_nodeid: 4\n",
        );

        assert_eq!(
            transport_facts_of(&parsed, "mcu").1,
            Some(NODEID_FIRST + 1),
            "4 is claimed explicitly, so the sequence starts at 5"
        );
        assert_eq!(
            transport_facts_of(&parsed, "tool").1,
            Some(NODEID_FIRST + 2)
        );
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

    /// A [`Machine`] over `fixture`, the way [`acquire`] builds one — the board
    /// locked by this test itself, since no test can set the environment
    /// variable without racing the others.
    fn machine_for(fixture: &TempConfig) -> Machine {
        let config = load(fixture.path()).expect("the fixture parses");
        let board = lock_board(fixture.path(), "self-test").expect("the fixture locks");
        Machine {
            config,
            path: fixture.path().to_path_buf(),
            progress: CaseProgress::new(PHASE_BODY),
            _board: board,
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

        // `[mcu]` with neither `serial:` nor `uuid:`.
        let bare = config("[mcu]\nrestart_method: arduino\n");
        assert_eq!(
            check(&bare, &Requires::new().mcu()),
            Err(vec![Missing::Mcu("mcu".to_string())])
        );

        // A serial interface.
        let serial = config("[mcu]\nserial: /dev/ttyACM0\nbaud: 250000\n");
        assert_eq!(check(&serial, &Requires::new().mcu()), Ok(()));

        // A CAN interface.
        let can = config("[mcu]\nuuid: 11aa22bb33cc\ncanbus_nodeid: 2\n");
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

        // `[mcu zboard]` with neither `serial:` nor `uuid:` — the case
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
        let can = config("[mcu zboard]\nuuid: 11aa22bb33cc\n");
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
            "missing [mcu] with serial: or uuid:"
        );
        assert_eq!(
            Missing::Mcu("mcu zboard".to_string()).to_string(),
            "missing [mcu zboard] with serial: or uuid:"
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
        let machine = machine_for(&fixture);

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
        let machine = machine_for(&fixture);

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
                "HW-MCU: [mcu toolhead] no serial:/uuid: \
                 — .mcu_named(\"toolhead\") tests skip"
                    .to_string(),
            ]
        );

        // A CAN board prints its transport instead of a serial device.
        let can = config("[mcu zboard]\nuuid: 11aa22bb33cc\ncanbus_nodeid: 2\n");
        assert_eq!(
            mcu_plan_lines(&can),
            vec![
                "HW-MCU: [mcu zboard] uuid: 11aa22bb33cc (interface: can0, mcu id: 2) \
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

    /// The `HW-RUN` line a test sees when it runs: one line, prefixed `HW-`,
    /// naming the test and carrying the mode's premise with it — the config is
    /// assumed correct, so the run checks the host module, not the user's
    /// config. It is the line [`acquire_at`]'s success path prints, which the
    /// fake config fixture below reaches.
    #[test]
    fn test_the_run_line_names_the_test_and_the_premise() {
        let name = "test_r5_single_axis_move_matches_the_firmware_step_count";
        let line = run_line(name);

        assert!(
            line.starts_with(&format!("HW-RUN: {name}")),
            "the prefix and the test's name come first: {line}"
        );
        assert!(
            line.contains("the config is assumed correct"),
            "the line carries the mode's premise: {line}"
        );
        assert!(
            line.contains("not your config"),
            "and says what the run is not checking: {line}"
        );
        assert_eq!(
            line.lines().count(),
            1,
            "the notice stays one line rather than a multi-line block: {line}"
        );

        // The success path is what prints it: a config that provides what the
        // test asked for hands back a machine, so `acquire_at` reached the run.
        let fixture = TempConfig::new("[mcu]\nserial: /dev/ttyACM0\n");
        assert!(
            acquire_at(Some(fixture.path()), name, &Requires::new().mcu(), true).is_some(),
            "a satisfied config reaches the run, whose line is the one checked above"
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

    // -----------------------------------------------------------------------
    // bring_up() — the second layer, against the dictionary-driven fake firmware
    // -----------------------------------------------------------------------
    //
    // These fixtures name the same `test: dict=` responder the corpus and the
    // multi-MCU tests use, so a machine can be brought up with no board attached
    // at all. `ResponderMcu::new` answers `None` when `build.rs` did not build
    // that dictionary (`KLIPPERX_ARCHES`), and the test skips — a build product
    // it never got, not a failure.

    /// A config with the fake boards `boards` and nothing else: no heater, so
    /// the known live-shape ring (see the module doc) has nothing to hold.
    fn fake_machine_config(boards: &[ResponderMcu]) -> TempConfig {
        TempConfig::new(&format!(
            "{}[printer]\nkinematics: none\nmax_velocity: 300\nmax_accel: 3000\n",
            ResponderMcu::sections(boards),
        ))
    }

    /// The connected session of the fake board `board`, or a panic naming it.
    fn session_of(started: &StartedMachine, board: &ResponderMcu) -> Arc<Mcu> {
        board
            .mcu(started.printer())
            .unwrap_or_else(|| panic!("[mcu {}] is not connected", board.name()))
    }

    /// The guard, from inside a runtime: `bring_up` owns a runtime and cannot be
    /// called from one.
    ///
    /// tokio reports both halves as a panic — building a runtime inside a
    /// runtime, and dropping one where blocking is not allowed — so the
    /// framework checks first and hands back a readable `Err`. The check is the
    /// first thing `bring_up` does, so the call below never opens the port its
    /// config names: without the guard, this test would die in the runtime build
    /// before any device work, not reach the `Err`.
    #[tokio::test]
    async fn test_bring_up_refuses_to_run_inside_a_runtime() {
        let reason = ensure_not_in_runtime().expect_err("inside a runtime the guard refuses");
        assert!(
            reason.contains("plain `#[test]`"),
            "the reason says how to write the case instead: {reason}"
        );

        let fixture = TempConfig::new(
            "[mcu]\nserial: /dev/klipperx-hwtest-no-such-port\n\
             [printer]\nkinematics: none\nmax_velocity: 300\nmax_accel: 3000\n",
        );
        let machine = machine_for(&fixture);
        let reason = match machine.bring_up() {
            Ok(_) => panic!("bring_up refuses inside a runtime, before it touches the device"),
            Err(reason) => reason,
        };
        assert!(
            reason.contains("plain `#[test]`"),
            "bring_up reports the guard's own reason: {reason}"
        );
    }

    /// The same guard outside a runtime: a plain `#[test]` is the frame
    /// `bring_up` is written for.
    #[test]
    fn test_bring_up_is_allowed_outside_a_runtime() {
        assert_eq!(ensure_not_in_runtime(), Ok(()));
    }

    /// Bring up a machine over the fake firmware and reach `Ready`.
    ///
    /// Two boards, because `bring_up` being the layer that starts *every*
    /// `[mcu …]` section is the capability this layer adds over
    /// [`Machine::open_mcu_named`]: both sections identify against their own
    /// dictionary, which a single-transport layer cannot do.
    #[test]
    fn test_bring_up_connects_every_board_and_reaches_ready() {
        let (Some(primary), Some(aux)) = (
            ResponderMcu::new("mcu", "atmega2560.dict"),
            ResponderMcu::new("aux", "stm32f103.dict"),
        ) else {
            return; // a dictionary this fixture needs was not built
        };
        let boards = [primary, aux];
        let fixture = fake_machine_config(&boards);
        let machine = machine_for(&fixture);

        let started = machine.bring_up().expect("the fake boards come up");

        let state = started.printer().get_state_message();
        assert_eq!(
            state.category,
            PrinterState::Ready,
            "the machine is ready: {}",
            state.message
        );
        for board in &boards {
            let mcu = session_of(&started, board);
            assert!(
                mcu.is_identified(),
                "[mcu {}] ran its own identify handshake",
                board.name()
            );
            assert!(
                board.device(started.printer()).is_some(),
                "[mcu {}] answers through its own responder",
                board.name()
            );
        }
    }

    /// Dropping the machine takes its session down: `Drop` tears the parts out
    /// (which is what runs `Mcu::Drop` and closes the device), then bounds the
    /// runtime's shutdown, and the assertion inside `Drop` is what proves the
    /// device read was released rather than parked.
    #[test]
    fn test_dropping_the_started_machine_tears_its_session_down() {
        let Some(board) = ResponderMcu::new("mcu", "atmega2560.dict") else {
            return;
        };
        let fixture = fake_machine_config(std::slice::from_ref(&board));
        let machine = machine_for(&fixture);
        let started = machine.bring_up().expect("the fake board comes up");

        let device = started
            .printer()
            .lookup_object_as::<McuObject>("mcu")
            .and_then(|object| object.mcu())
            .and_then(|mcu| mcu.simulator_device())
            .expect("the fake board's responder");
        // Weak handles: what the machine held must be gone once it is dropped,
        // and no handle of this test's may stand in for it.
        let session = Arc::downgrade(&session_of(&started, &board));
        let responder = Arc::downgrade(&device);
        drop(device);

        drop(started);

        assert!(
            session.upgrade().is_none(),
            "the machine released its session: `Mcu::Drop` ran, which shuts the device down"
        );
        assert!(
            responder.upgrade().is_none(),
            "the fake device went with the session, blocking read and all"
        );
    }

    /// The panic path: a body that panics with the machine up must still tear it
    /// down, because `Drop` runs while unwinding and does its work before the
    /// test body's failure is reported.
    #[test]
    fn test_a_panic_in_the_body_still_tears_the_session_down() {
        let Some(board) = ResponderMcu::new("mcu", "atmega2560.dict") else {
            return;
        };
        let fixture = fake_machine_config(std::slice::from_ref(&board));
        let machine = machine_for(&fixture);

        // The session handle the body leaves behind: the body owns the machine,
        // so the only way out of the closure is a `Weak` it stored first.
        let seen: Mutex<Option<Weak<Mcu>>> = Mutex::new(None);
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let started = machine.bring_up().expect("the fake board comes up");
            *seen.lock().unwrap_or_else(|poison| poison.into_inner()) =
                Some(Arc::downgrade(&session_of(&started, &board)));
            panic!("the body panics with the machine up");
        }));

        assert!(outcome.is_err(), "the body panicked, as intended");
        let session = seen
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .take()
            .expect("the body stored its session handle before panicking");
        assert!(
            session.upgrade().is_none(),
            "unwinding dropped the `StartedMachine`, and its `Drop` took the session down"
        );
    }

    /// A config that names a board which cannot be opened is a **real** failure
    /// of `bring_up`, not a skip: the config said the hardware was there.
    #[test]
    fn test_a_board_that_cannot_open_is_a_failure_of_bring_up() {
        let fixture = TempConfig::new(
            "[mcu]\nserial: /dev/klipperx-hwtest-no-such-port\n\
             [printer]\nkinematics: none\nmax_velocity: 300\nmax_accel: 3000\n",
        );
        let machine = machine_for(&fixture);

        let reason = match machine.bring_up() {
            Ok(_) => panic!("a config that names a device is a real failure, not a skip"),
            Err(reason) => reason,
        };
        assert!(
            reason.contains("/dev/klipperx-hwtest-no-such-port"),
            "the reason names the device the config asked for: {reason}"
        );
    }

    /// The failure path's shutdown is checked on the same measure
    /// [`StartedMachine`]'s `Drop` uses: a runtime that is still alive when the
    /// bound runs out is a leaked part, and the `Err` carries that after the
    /// bring-up's own reason instead of hiding it.
    ///
    /// The parked blocking read is the leak's shape (a device read cannot be
    /// cancelled), and the short bound keeps the check off the real
    /// [`SHUTDOWN_TIMEOUT`].
    #[test]
    fn test_a_failed_machine_that_does_not_shut_down_says_so_in_the_err() {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .expect("a runtime for the failed machine");
        let (release, held) = std::sync::mpsc::channel::<()>();
        runtime.spawn_blocking(move || {
            let _ = held.recv();
        });

        let reason = bring_up_failure(
            "the machine did not come up: a part never connected".to_string(),
            runtime,
            Duration::from_millis(100),
        );

        assert!(
            reason.starts_with("the machine did not come up: a part never connected"),
            "the original reason is kept: {reason}"
        );
        assert!(reason.contains("did not shut down"), "{reason}");
        assert!(reason.contains("leaked"), "{reason}");
        drop(release);
    }

    /// The other half: a failed machine whose parts did go hands back the
    /// bring-up's reason unchanged — the check adds a verdict only when there is
    /// one.
    #[test]
    fn test_a_failed_machine_that_shuts_down_keeps_its_own_reason() {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .expect("a runtime for the failed machine");

        let reason = bring_up_failure(
            "the machine did not come up: a part never connected".to_string(),
            runtime,
            Duration::from_millis(100),
        );

        assert_eq!(
            reason,
            "the machine did not come up: a part never connected"
        );
    }

    /// The loose skip path, unchanged: with strict mode off (`KLIPPERX_HW_STRICT=0`,
    /// here the `false` argument), an unmet [`Requires`] hands back `None` — the
    /// caller sees `HW-IGNORED` and returns — and an unset variable does the same
    /// in **both** modes. No machine is built and the board is not locked, so the
    /// next test can have it.
    #[test]
    fn test_an_unmet_requirement_skips_without_a_machine() {
        let fixture = TempConfig::new("[mcu]\nserial: /dev/ttyACM0\n");
        let unmet = Requires::new().section("stepper_x");

        assert!(
            acquire_at(Some(fixture.path()), "self-test", &unmet, false).is_none(),
            "strict off: a config that does not provide what the test asked for is a skip"
        );
        assert!(
            acquire_at(None, "self-test", &Requires::new(), true).is_none(),
            "strict on: no config at all is still the skip, or `cargo test` would go red"
        );
        assert!(
            acquire_at(None, "self-test", &Requires::new(), false).is_none(),
            "strict off: the same skip"
        );
        // A skipped test took no lock: the board is free for whoever comes next.
        let _board = lock_board(fixture.path(), "self-test").expect("a skip takes no lock");
    }

    /// The change: a named config that does not provide what the test declared
    /// is a **failure** under strict mode (the default), not a skip — a green
    /// `--ignored` run must not be readable as "this config was tested".
    #[test]
    #[should_panic(expected = "missing [stepper_x]")]
    fn test_an_unmet_requirement_fails_the_case_under_strict_mode() {
        let fixture = TempConfig::new("[mcu]\nserial: /dev/ttyACM0\n");
        let unmet = Requires::new()
            .section("stepper_x")
            .option("extruder", "nozzle_diameter");

        let _ = acquire_at(Some(fixture.path()), "self-test", &unmet, true);
    }

    /// The strict failure carries everything needed to act on it: the test, the
    /// config, **every** missing item, and the escape hatch.
    #[test]
    fn test_a_strict_failure_names_the_test_the_config_the_missing_items_and_the_way_out() {
        let fixture = TempConfig::new("[mcu]\nserial: /dev/ttyACM0\n");
        let unmet = Requires::new().section("stepper_x").section("heater_bed");

        let payload = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            acquire_at(Some(fixture.path()), "self-test", &unmet, true)
        })) {
            Ok(machine) => panic!(
                "strict mode must fail an unmet requirement, not hand out {:?}",
                machine.is_some()
            ),
            Err(payload) => payload,
        };

        let message = panic_text(payload.as_ref());
        assert!(message.contains("self-test"), "{message}");
        assert!(
            message.contains(&fixture.path().display().to_string()),
            "the message names the config: {message}"
        );
        assert!(
            message.contains("missing [stepper_x], missing [heater_bed]"),
            "every missing item travels, in declaration order: {message}"
        );
        assert!(
            message.contains("set KLIPPERX_HW_STRICT=0 to skip instead"),
            "the escape hatch is spelled out: {message}"
        );
    }

    /// Strict mode is the default, and only an explicit `0`/`false`/`no` — in
    /// any case, surrounding blanks ignored — turns it off. A typo therefore
    /// asks for the strict run rather than silently skipping.
    #[test]
    fn test_strict_is_the_default_and_only_zero_false_no_turn_it_off() {
        for value in [
            None,
            Some(""),
            Some("1"),
            Some("true"),
            Some("yes"),
            Some("strict"),
        ] {
            assert!(strict_from(value), "{value:?} asks for the strict run");
        }
        for value in [
            Some("0"),
            Some("false"),
            Some("no"),
            Some(" FALSE "),
            Some("No"),
        ] {
            assert!(!strict_from(value), "{value:?} turns strict mode off");
        }
    }

    /// A config that names an object the repository does not implement is a
    /// **failure** under strict mode: no case here can run against it (the host
    /// would refuse it at start-up), and the run must say so instead of failing
    /// later inside a case for an unrelated-looking reason.
    #[test]
    #[should_panic(expected = "palette2")]
    fn test_a_not_implemented_section_fails_the_case_under_strict_mode() {
        let fixture = TempConfig::new("[palette2]\nuart: /dev/ttyAMA0\n");

        let _ = acquire_at(Some(fixture.path()), "self-test", &Requires::new(), true);
    }

    /// And it carries what the reader needs to act: the test, the config, the
    /// sections it found, what they are, and the escape hatch.
    #[test]
    fn test_a_not_implemented_section_is_named_with_its_config_and_the_way_out() {
        let fixture =
            TempConfig::new("[mcu]\nserial: /dev/ttyACM0\n[replicape]\n[palette2]\nha: 5\n");

        let payload = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            acquire_at(Some(fixture.path()), "self-test", &Requires::new(), true)
        })) {
            Ok(machine) => panic!(
                "strict mode must fail a not-implemented section, not hand out {:?}",
                machine.is_some()
            ),
            Err(payload) => payload,
        };

        let message = panic_text(payload.as_ref());
        assert!(message.contains("self-test"), "{message}");
        assert!(
            message.contains(&fixture.path().display().to_string()),
            "the message names the config: {message}"
        );
        assert!(
            message.contains("replicape, palette2"),
            "every section found is listed, in config order: {message}"
        );
        assert!(
            message.contains("§1.2 defers"),
            "the message says what these are: {message}"
        );
        assert!(
            message.contains("set KLIPPERX_HW_STRICT=0 to skip instead"),
            "the escape hatch is spelled out: {message}"
        );
    }

    /// The other half: strict off adds nothing for these sections. Such a config
    /// is handed out as before, and an unmet [`Requires`] is still `HW-IGNORED`
    /// — the check is strict mode's, not a new rule of its own.
    #[test]
    fn test_a_not_implemented_section_is_not_a_failure_when_strict_is_off() {
        let fixture = TempConfig::new("[mcu]\nserial: /dev/ttyACM0\n[palette2]\nha: 5\n");

        assert!(
            acquire_at(Some(fixture.path()), "self-test", &Requires::new(), false).is_some(),
            "strict off: the section alone does not stop the test"
        );
        assert!(
            acquire_at(
                Some(fixture.path()),
                "self-test",
                &Requires::new().section("stepper_x"),
                false,
            )
            .is_none(),
            "strict off: an unmet requirement is still the skip it was"
        );
    }

    /// The deferred list is pinned: it is the three modules of
    /// `upstream-deviations.md` §1.2 and nothing else, so a silent addition or
    /// removal cannot ride along with another change.
    #[test]
    fn test_the_not_implemented_list_is_the_three_deferred_modules() {
        assert_eq!(
            NOT_IMPLEMENTED_SECTIONS,
            &["samd_sercom", "replicape", "palette2"],
            "the list is §1.2's three deferred modules"
        );
    }

    /// `KLIPPERX_HW_CONFIG` set to a config that does not parse is a **failure**,
    /// not a skip: the variable said the machine had a config to test against.
    /// The strictness does not enter into it — only an unset variable is a skip.
    #[test]
    fn test_a_named_config_that_does_not_parse_fails_instead_of_skipping() {
        let fixture = TempConfig::new("serial: /dev/ttyACM0\n");

        let payload = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            acquire_at(Some(fixture.path()), "self-test", &Requires::new(), false)
        })) {
            Ok(_) => panic!("a config that does not parse is a failure, not a skip"),
            Err(payload) => payload,
        };

        let message = panic_text(payload.as_ref());
        assert!(message.contains("cannot be used"), "{message}");
        // The parser's own reason travels with it.
        assert!(
            message.contains("Parameter outside of section"),
            "{message}"
        );
    }

    /// The same for a path that does not exist: a set variable is a promise.
    #[test]
    fn test_a_named_config_that_does_not_exist_fails_instead_of_skipping() {
        let absent = std::env::temp_dir().join("klipperx-hwtest-does-not-exist/printer.cfg");

        let payload = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            acquire_at(Some(&absent), "self-test", &Requires::new(), false)
        })) {
            Ok(_) => panic!("a config path that does not exist is a failure, not a skip"),
            Err(payload) => payload,
        };

        let message = panic_text(payload.as_ref());
        assert!(message.contains("cannot be used"), "{message}");
        assert!(message.contains("printer.cfg"), "{message}");
    }

    /// The host-library transport facts `Machine::open_mcu_named` stamps on the
    /// board called `name`: the mux attach and the bridge port table.
    fn host_facts(parsed: &Config, name: &str) -> (Option<HostMux>, Vec<(u8, String)>) {
        let identifier = mcu_section_id(name);
        let section = parsed
            .get_section(&identifier)
            .unwrap_or_else(|| panic!("the fixture has [{identifier}]"));
        let mut mcu = McuConfig::new(&ConfigWrapper::with_config(
            section,
            crate::core::klippy::config::AccessTracking::shared(),
            None,
            parsed,
        ))
        .expect("the section parses");
        apply_id_addressed_transport(parsed, section, &mut mcu);
        (mcu.host_mux, mcu.host_port_paths)
    }

    /// A `host_library:` path the config does not call muxed opens with bare
    /// frames, exactly as the load path leaves it.
    #[test]
    fn test_a_plain_host_library_path_stays_unmuxed() {
        let parsed = config("[mcu]\nhost_library: /opt/libklipper_host.so\n");

        assert_eq!(host_facts(&parsed, "mcu"), (None, Vec::new()));
    }

    /// A path the config **does** call muxed (here: the section writes a
    /// `uuid`, which names the board the mux routes to) is opened through
    /// [`Interface::host_mux`] with the identity pair the load path gives it —
    /// without this the library would be opened with bare frames and address
    /// nobody.
    #[test]
    fn test_a_muxed_host_library_path_attaches_with_its_identity_pair() {
        let parsed = config("[mcu]\nhost_library: /opt/libklipper_host.so\nuuid: 11aa22bb33cc\n");

        let (mux, paths) = host_facts(&parsed, "mcu");
        let mux = mux.expect("a section writing a uuid makes the path muxed");
        assert_eq!(
            mux.named,
            [0x11, 0xaa, 0x22, 0xbb, 0x33, 0xcc],
            "the attach names the uuid the section wrote"
        );
        // The identity is the path's own (seeded from its text, so the same
        // config always gets the same one), and it is what the load path's own
        // grouping assigns: replaying that call is the point of the stamp.
        let section = parsed.get_section("mcu").expect("the fixture has [mcu]");
        let libraries = HostLibraries::classify(&ConfigWrapper::with_config(
            section,
            crate::core::klippy::config::AccessTracking::shared(),
            None,
            &parsed,
        ))
        .expect("the fixture classifies");
        assert_eq!(
            Some(mux.identity),
            libraries.identity("/opt/libklipper_host.so"),
            "the identity is the load path's, not a number invented here"
        );
        assert!(paths.is_empty(), "this path names no uart port: {paths:?}");
    }

    /// A bridge is muxed even when it writes no `uuid`: the downstream device on
    /// its port is what makes the path addressed by id (`path_is_mux`'s
    /// `has_bridge`/`downstream` rule), and the attach then names the bridge's own
    /// identity.
    #[test]
    fn test_a_host_library_bridge_is_muxed_and_names_its_own_identity() {
        let parsed = config(
            "[mcu bridge]\nhost_library: /opt/libklipper_host.so\n\
             [uart_device uart0]\nuart_mcu: bridge\nuart_oid: uart0\nuart_path: /dev/bridge-a\n",
        );

        let (mux, paths) = host_facts(&parsed, "bridge");
        let mux = mux.expect("a bridge with a downstream device is muxed");
        assert_eq!(
            mux.named, mux.identity,
            "without a uuid the attach names the bridge's own identity"
        );
        assert_eq!(
            paths,
            vec![(0u8, "/dev/bridge-a".to_string())],
            "the port's own tty is recorded under its uart slot"
        );
    }

    /// The bridge port table is the load path's, slot for slot: only a
    /// `[uart_device]` takes a UART slot, a `[can_device]` does not, and a port
    /// without `uart_path:` records no tty but still takes its slot.
    #[test]
    fn test_the_bridge_port_table_counts_uart_slots_the_way_the_load_does() {
        let parsed = config(
            "[mcu bridge]\nhost_library: /opt/libklipper_host.so\n\
             [uart_device first]\nuart_mcu: bridge\nuart_oid: uart0\nuart_path: /dev/a\n\
             [can_device between]\ncan_mcu: bridge\ncan_oid: can0\n\
             [uart_device second]\nuart_mcu: bridge\nuart_oid: uart1\n\
             [uart_device third]\nuart_mcu: bridge\nuart_oid: uart2\nuart_path: /dev/c\n",
        );

        assert_eq!(
            host_facts(&parsed, "bridge").1,
            vec![(0u8, "/dev/a".to_string()), (2u8, "/dev/c".to_string()),],
            "the CAN port takes no slot, and the uart port without a tty still takes one"
        );

        // A serial board has no port table at all.
        assert!(host_facts(&config("[mcu]\nserial: /dev/ttyACM0\n"), "mcu")
            .1
            .is_empty());
    }

    // -----------------------------------------------------------------------
    // The teardown — the machine turned off on both paths
    // -----------------------------------------------------------------------

    /// What the teardown sends: every heater off, and only the outputs that are
    /// actually on.
    ///
    /// `outputs` is the recording fake of the machine's own status — the pair
    /// `turn_everything_off` reads for each `[output_pin]` — so the decision is
    /// checked without a board.
    #[test]
    fn test_the_teardown_turns_heaters_off_and_only_the_outputs_that_are_on() {
        let outputs = vec![
            ("fan".to_string(), 0.6),
            ("led".to_string(), 0.0),
            ("pwm".to_string(), 1.0),
        ];

        assert_eq!(
            turn_off_script(true, &outputs),
            vec![
                "TURN_OFF_HEATERS".to_string(),
                "SET_PIN PIN=fan VALUE=0".to_string(),
                "SET_PIN PIN=pwm VALUE=0".to_string(),
            ],
            "the heaters go off, and a pin already at zero is left alone"
        );

        // A config with no heater section has no registry to command, and a
        // machine with nothing on has nothing to send at all.
        assert_eq!(
            turn_off_script(
                false,
                &["led".to_string()]
                    .iter()
                    .map(|name| (name.clone(), 0.0))
                    .collect::<Vec<_>>()
            ),
            Vec::<String>::new()
        );
    }

    /// The driving: every line is sent, in order, and a refusal is reported
    /// rather than raised — this runs while unwinding as often as not.
    #[test]
    fn test_the_teardown_sends_every_line_and_reports_a_refusal() {
        let mut sent = Vec::new();
        drive_teardown(
            vec![
                "TURN_OFF_HEATERS".to_string(),
                "SET_PIN PIN=fan VALUE=0".to_string(),
            ],
            |line| {
                sent.push(line.to_string());
                if line.contains("SET_PIN") {
                    return Err("the machine is shut down".to_string());
                }
                Ok(())
            },
        );

        assert_eq!(
            sent,
            vec![
                "TURN_OFF_HEATERS".to_string(),
                "SET_PIN PIN=fan VALUE=0".to_string(),
            ],
            "both lines are sent, and the refusal of one does not stop the other"
        );
    }

    // -----------------------------------------------------------------------
    // The case runner — a bound on the whole case, and its attribution
    // -----------------------------------------------------------------------

    /// A case that hangs is **reported** and the runner returns: the body is
    /// abandoned at the budget, the message names the case and the phase it was
    /// stuck in, and the caller fails with that message rather than waiting.
    ///
    /// The budget is milliseconds here so the test stays bounded; production
    /// passes [`CASE_TIMEOUT`].
    #[test]
    fn test_a_case_that_hangs_reports_its_phase_and_the_runner_returns() {
        let fixture = TempConfig::new("[mcu]\nserial: /dev/ttyACM0\n");
        let started = Instant::now();
        let payload = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            run_case(
                Some(fixture.path().to_path_buf()),
                true,
                "test_a_case_that_hangs",
                &Requires::new().mcu(),
                Duration::from_millis(100),
                |_machine| std::thread::sleep(Duration::from_secs(30)),
            )
        })) {
            Ok(()) => panic!("the runner must fail the case, not return as if it passed"),
            Err(payload) => payload,
        };

        // Bounded: the abandoned body sleeps for 30 s, the runner returned long
        // before it.
        assert!(
            started.elapsed() < Duration::from_secs(10),
            "the runner returned inside its budget: {:?}",
            started.elapsed()
        );
        let message = panic_text(payload.as_ref());
        assert!(
            message.contains("HW-TIMEOUT: test_a_case_that_hangs"),
            "{message}"
        );
        assert!(
            message.contains(PHASE_BODY),
            "the message names the phase the case was stuck in: {message}"
        );
    }

    /// The other half: a case that finishes inside its budget passes, and the
    /// body really ran with a machine (which is what the board lock is for).
    #[test]
    fn test_a_case_that_finishes_inside_its_budget_passes() {
        let fixture = TempConfig::new("[mcu]\nserial: /dev/ttyACM0\n");
        let ran = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let seen = Arc::clone(&ran);

        run_case(
            Some(fixture.path().to_path_buf()),
            true,
            "test_a_case_that_finishes",
            &Requires::new().mcu(),
            Duration::from_secs(30),
            move |machine| {
                assert_eq!(
                    machine
                        .config()
                        .get_section("mcu")
                        .and_then(|section| section.get_str("serial")),
                    Some("/dev/ttyACM0")
                );
                seen.store(true, std::sync::atomic::Ordering::SeqCst);
            },
        );

        assert!(
            ran.load(std::sync::atomic::Ordering::SeqCst),
            "the body ran with the machine the config provides"
        );
    }

    /// A body that panics fails the case with its own message: the failure is
    /// this test's, not a silent thread's.
    #[test]
    fn test_a_panic_in_the_body_fails_the_case_with_its_message() {
        let fixture = TempConfig::new("[mcu]\nserial: /dev/ttyACM0\n");

        let payload = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            run_case(
                Some(fixture.path().to_path_buf()),
                true,
                "test_a_case_that_panics",
                &Requires::new().mcu(),
                Duration::from_secs(30),
                |_machine| panic!("the board answered 0 where 1 was expected"),
            )
        })) {
            Ok(()) => panic!("a panicking body fails the case"),
            Err(payload) => payload,
        };

        assert_eq!(
            panic_text(payload.as_ref()),
            "the board answered 0 where 1 was expected"
        );
    }

    /// The loose skip path travels through the runner: with strict mode off
    /// (`KLIPPERX_HW_STRICT=0`, here the `false` argument) a config that does
    /// not provide what the case asked for is `HW-IGNORED`, and the body never
    /// runs. Under strict it is a failure instead, which is
    /// [`test_an_unmet_requirement_fails_the_case_under_strict_mode`]'s case at
    /// the `acquire_at` level.
    #[test]
    fn test_a_case_the_config_does_not_provide_is_skipped_by_the_runner() {
        let fixture = TempConfig::new("[mcu]\nserial: /dev/ttyACM0\n");
        let ran = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let seen = Arc::clone(&ran);

        run_case(
            Some(fixture.path().to_path_buf()),
            false,
            "test_a_case_that_is_skipped",
            &Requires::new().section("stepper_x"),
            Duration::from_secs(30),
            move |_machine| seen.store(true, std::sync::atomic::Ordering::SeqCst),
        );

        assert!(
            !ran.load(std::sync::atomic::Ordering::SeqCst),
            "a skipped case runs no body: there is no machine to hand it"
        );
    }

    /// The board lock's wait is bounded: a board another session holds is
    /// reported as held, not waited on forever. The short bound is a parameter
    /// so the check does not wait [`LOCK_TIMEOUT`] out.
    #[test]
    fn test_waiting_for_a_held_board_ends_with_the_board_reported_as_held() {
        let fixture = TempConfig::new("[mcu]\nserial: /dev/ttyACM0\n");
        // Held for the whole check, as a hung case would hold it.
        let _held = lock_board(fixture.path(), "self-test").expect("the fixture locks");

        let started = Instant::now();
        let reason = lock_board_within(fixture.path(), "self-test", Duration::from_millis(150))
            .err()
            .expect("a held board is not handed out");

        assert!(
            reason.contains("another hardware test still holds the board"),
            "the reason says the board is held: {reason}"
        );
        assert!(
            started.elapsed() < Duration::from_secs(10),
            "the wait ended at its bound: {:?}",
            started.elapsed()
        );
    }
}
