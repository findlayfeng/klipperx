//! `klipperx moonraker` — run Moonraker, keeping its sources and its Python
//! environment here.
//!
//! Moonraker is a Python application with a dozen pinned dependencies, and the
//! usual way to run it is to remember which checkout, which virtualenv and which
//! install it was left in. This subcommand removes all three questions.
//!
//! **Where the sources come from** — the first that applies wins, and the
//! startup line says which:
//!
//! 1. `--source <DIR>` — a checkout to run as it is.
//! 2. The **bundled copy**, `$XDG_DATA_HOME/klipperx/moonraker`
//!    (`~/.local/share/klipperx/moonraker`), copied there from the checkout on
//!    first run. This is the layer packaging owns: put Moonraker's sources here
//!    and this program runs them, no checkout and no code change.
//! 3. The **checkout**, `<repo>/third_party/moonraker` (or
//!    `KLIPPERX_MOONRAKER_DIR`).
//!
//! **Where the dependencies come from**: a virtualenv of this program's own
//! (`~/.local/share/klipperx/moonraker-venv` by default), created on first run,
//! with `scripts/moonraker-requirements.txt` installed into it. "Installed" is
//! not a guess: the requirements file's SHA-256 is written into the virtualenv as
//! [`MARKER_FILE`]; a run whose recorded digest matches does nothing at all — no
//! `pip`, no network, no wait — and a run whose digest differs reinstalls. A
//! failed install leaves the old marker, so the next run retries rather than
//! believing a broken environment is ready. `--offline` installs from a
//! directory of wheels with `pip --no-index`, and refuses to run at all when
//! that directory is missing, because pip would otherwise fall back to the
//! network after a warning nobody reads.
//!
//! Moonraker needs a directory tree around it as much as an interpreter:
//! `config/`, `logs/`, `gcodes/` and friends under its data path, and a
//! `moonraker.conf` inside `config/`. Both are created here when absent, the
//! configuration as a minimal file that points Moonraker at the host's socket
//! (`/tmp/klippy_uds`, the `-a` the `klippy` subcommand defaults to) and trusts
//! the local networks — a starting point to edit, not a considered
//! configuration.

use std::path::{Path, PathBuf};

use clap::Args;

use crate::ui::command::{self, CommandRunner, Invocation, SystemRunner};
use crate::ui::sha256::sha256_hex;
use crate::ui::{bundled_moonraker, expand_home, moonraker_checkout};

/// Where Moonraker's files live by default: the same `~/printer_data` Klipper's
/// own installation scripts use.
pub const DEFAULT_DATA_PATH: &str = "~/printer_data";

/// Where the virtualenv lives by default: this program's own data directory,
/// so it never collides with a Moonraker the operator installed by hand.
pub const DEFAULT_VENV: &str = "~/.local/share/klipperx/moonraker-venv";

/// The subdirectories Moonraker expects under its data path.
const DATA_SUBDIRS: [&str; 6] = ["config", "logs", "gcodes", "comms", "data", "backup"];

/// The file inside the virtualenv recording which requirements are installed.
pub const MARKER_FILE: &str = "klipperx-requirements.sha256";

/// The interpreter the virtualenv is created with.
const PYTHON3: &str = "python3";

/// The entry point, relative to a Moonraker source directory.
const ENTRY_POINT: &str = "moonraker/moonraker.py";

/// The source order, spelled out in `--help`, so an operator can tell which
/// checkout — or which copy — a run will execute.
const SOURCE_HELP: &str = "\
MOONRAKER SOURCE (the first that applies is used):
  1. --source <DIR>                 a checkout to run as it is
  2. the bundled copy               $XDG_DATA_HOME/klipperx/moonraker
                                    (~/.local/share/klipperx/moonraker), copied
                                    there from the checkout on first run
  3. <repo>/third_party/moonraker   the development checkout
                                    (KLIPPERX_MOONRAKER_DIR moves this layer)

The bundled copy is made once and then kept: a newer checkout is not picked up
by itself — pass --source for a checkout being edited. --offline installs from
--wheels (default <source>/python_wheels) with `pip --no-index`, and a missing
wheels directory is an error. The source, the entry point and the command line
are printed at startup, and --dry-run also prints which layer answered.";

/// The configuration written when there is none.
///
/// It is the smallest file Moonraker starts from: a server on 7125 whose Klipper
/// socket is the `klippy` subcommand's default `-a /tmp/klippy_uds`, no machine
/// provider (the host is this repository's, not Moonraker's), the local networks
/// trusted for authorization, and the stores a UI reads its history from.
const DEFAULT_CONFIG: &str = r#"# Moonraker configuration written by `klipperx moonraker`.
# Edit it to match this printer; it is only a starting point.
[server]
host: 0.0.0.0
port: 7125
klippy_uds_address: /tmp/klippy_uds

[machine]
provider: none

[authorization]
trusted_clients:
    127.0.0.1
    ::1
    10.0.0.0/8
    172.16.0.0/12
    192.168.0.0/16

[file_manager]
enable_object_processing: False

[data_store]
temperature_store_size: 600
gcode_store_size: 1000

[history]
"#;

/// Run Moonraker as a local service.
#[derive(Args, Debug, Clone)]
#[command(after_help = SOURCE_HELP)]
pub struct MoonrakerArgs {
    /// Moonraker's data directory (config/, logs/, gcodes/, …)
    #[arg(long, value_name = "DIR", default_value = DEFAULT_DATA_PATH)]
    pub data_path: PathBuf,

    /// The configuration file (default: <data-path>/config/moonraker.conf)
    #[arg(long, value_name = "FILE")]
    pub config: Option<PathBuf>,

    /// The virtualenv to keep the dependencies in
    #[arg(long, value_name = "DIR", default_value = DEFAULT_VENV)]
    pub venv: PathBuf,

    /// The Moonraker sources to run (default: the bundled copy, else the checkout)
    #[arg(long, value_name = "DIR")]
    pub source: Option<PathBuf>,

    /// The requirements file to install (default: <source>/scripts/moonraker-requirements.txt)
    #[arg(long, value_name = "FILE")]
    pub requirements: Option<PathBuf>,

    /// Install from a local wheels directory instead of the network
    #[arg(long)]
    pub offline: bool,

    /// The wheels directory for --offline (default: <source>/python_wheels)
    #[arg(long, value_name = "DIR", requires = "offline")]
    pub wheels: Option<PathBuf>,

    /// Prepare the environment and print the command; do not start Moonraker
    #[arg(long)]
    pub dry_run: bool,
}

/// Run `klipperx moonraker`: resolve and prepare the environment, then hand over
/// to Moonraker and relay its exit code.
pub fn run(args: MoonrakerArgs) -> Result<i32, Box<dyn std::error::Error>> {
    run_with(&args, &SystemRunner).map_err(Into::into)
}

/// [`run`] with the program runner given, for tests.
fn run_with(args: &MoonrakerArgs, runner: &dyn CommandRunner) -> Result<i32, String> {
    let resolution = resolve(args)?;
    // One line, always: which sources this run actually executes.
    println!("{}", resolution.source_line());
    if args.dry_run {
        // The whole order, so `--dry-run` answers "which layer would answer?".
        for line in resolution.layer_lines() {
            println!("{line}");
        }
    }

    let launch = resolution.service().prepare(runner)?;
    println!("moonraker: {}", launch.command_line());
    if args.dry_run {
        println!("moonraker: --dry-run, not starting");
        return Ok(0);
    }
    launch.run_to_completion()
}

/// Where the sources may come from, in the order they are tried.
#[derive(Clone, Debug)]
struct Candidates {
    /// `--source`, when it was given.
    given: Option<PathBuf>,
    /// The copy kept beside the venv.
    bundled: PathBuf,
    /// The development checkout.
    checkout: PathBuf,
}

impl Candidates {
    /// The candidates a run is given: the arguments plus the two fixed places.
    fn from_args(args: &MoonrakerArgs) -> Result<Candidates, String> {
        Ok(Candidates {
            given: match &args.source {
                Some(dir) => Some(expand_home(dir)?),
                None => None,
            },
            bundled: bundled_moonraker()?,
            checkout: moonraker_checkout(),
        })
    }
}

/// Which layer the sources came from.
#[derive(Clone, Debug, PartialEq, Eq)]
enum SourceChoice {
    /// `--source`.
    Given(PathBuf),
    /// The bundled copy, already there.
    Bundled(PathBuf),
    /// The bundled copy, just made from the checkout.
    Copied(PathBuf),
    /// The checkout itself.
    Checkout(PathBuf),
}

impl SourceChoice {
    /// The directory this layer runs.
    fn dir(&self) -> &Path {
        match self {
            SourceChoice::Given(dir)
            | SourceChoice::Bundled(dir)
            | SourceChoice::Copied(dir)
            | SourceChoice::Checkout(dir) => dir,
        }
    }

    /// Which layer of the three answered: 0 `--source`, 1 the bundled copy, 2
    /// the checkout.
    fn layer(&self) -> usize {
        match self {
            SourceChoice::Given(_) => 0,
            SourceChoice::Bundled(_) | SourceChoice::Copied(_) => 1,
            SourceChoice::Checkout(_) => 2,
        }
    }

    /// How to say which layer answered.
    fn describe(&self) -> String {
        match self {
            SourceChoice::Given(dir) => format!("--source {}", dir.display()),
            SourceChoice::Bundled(dir) => format!("the bundled copy at {}", dir.display()),
            SourceChoice::Copied(dir) => {
                format!(
                    "the bundled copy at {} (copied from the checkout now)",
                    dir.display()
                )
            }
            SourceChoice::Checkout(dir) => format!("the checkout at {}", dir.display()),
        }
    }
}

/// Whether `dir` is a Moonraker source tree: the entry point is what is run, so
/// its presence is the test.
fn is_source(dir: &Path) -> bool {
    dir.join("moonraker").join("moonraker.py").is_file()
}

/// The source resolution: `--source`, then the bundled copy, then the checkout.
///
/// The bundled copy is made here on a first run, because that is the only moment
/// it can be: once it exists it is what runs, which is what lets a package place
/// its own sources there instead.
fn choose_source(candidates: &Candidates) -> Result<SourceChoice, String> {
    if let Some(given) = &candidates.given {
        if !is_source(given) {
            return Err(format!(
                "--source {} is not a Moonraker source tree: {} does not exist",
                given.display(),
                given.join(ENTRY_POINT).display()
            ));
        }
        return Ok(SourceChoice::Given(given.clone()));
    }

    if is_source(&candidates.bundled) {
        return Ok(SourceChoice::Bundled(candidates.bundled.clone()));
    }

    if !candidates.bundled.exists() && is_source(&candidates.checkout) {
        copy_tree(&candidates.checkout, &candidates.bundled).map_err(|error| {
            format!(
                "cannot make the bundled copy in {}: {error}",
                candidates.bundled.display()
            )
        })?;
        if is_source(&candidates.bundled) {
            return Ok(SourceChoice::Copied(candidates.bundled.clone()));
        }
    }

    if is_source(&candidates.checkout) {
        if candidates.bundled.exists() {
            println!(
                "moonraker: warning: {} is not a usable copy ({ENTRY_POINT} is missing); \
                 using the checkout",
                candidates.bundled.display()
            );
        }
        return Ok(SourceChoice::Checkout(candidates.checkout.clone()));
    }

    Err(format!(
        "no Moonraker sources: --source was not given, {} is not a copy, and {} is \
         not a source tree. An empty submodule is not a checkout — initialize it in \
         the main one (`git submodule update --init third_party/moonraker`) or pass \
         --source",
        candidates.bundled.display(),
        candidates.checkout.display()
    ))
}

/// Copy a source tree, leaving out what a copy does not need: git's metadata and
/// Python's caches.
fn copy_tree(from: &Path, to: &Path) -> std::io::Result<()> {
    std::fs::create_dir_all(to)?;
    for entry in std::fs::read_dir(from)? {
        let entry = entry?;
        let name = entry.file_name();
        if name == ".git" || name == "__pycache__" {
            continue;
        }
        let path = entry.path();
        let destination = to.join(&name);
        if path.is_dir() {
            copy_tree(&path, &destination)?;
        } else {
            std::fs::copy(&path, &destination)?;
        }
    }
    Ok(())
}

/// One Moonraker installation, resolved to absolute paths.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Service {
    /// The source tree that is run.
    pub source: PathBuf,
    /// `moonraker/moonraker.py` inside it.
    pub entry_point: PathBuf,
    /// Moonraker's data directory.
    pub data_path: PathBuf,
    /// Its configuration file.
    pub config: PathBuf,
    /// The virtualenv its dependencies live in.
    pub venv: PathBuf,
    /// The requirements file those dependencies are read from.
    pub requirements: PathBuf,
    /// The wheels directory `--offline` installs from.
    pub wheels: PathBuf,
    /// Whether the dependencies must come from `wheels`, never the network.
    pub offline: bool,
}

/// One run's resolution: the installation, the layer that answered, and the
/// candidates that were considered.
#[derive(Clone, Debug)]
pub struct Resolution {
    service: Service,
    choice: SourceChoice,
    candidates: Candidates,
}

impl Resolution {
    /// The installation this run uses.
    pub fn service(&self) -> &Service {
        &self.service
    }

    /// The one line naming where the sources came from.
    fn source_line(&self) -> String {
        format!("moonraker: source: {}", self.choice.describe())
    }

    /// All three layers with their verdicts, for `--dry-run`.
    fn layer_lines(&self) -> Vec<String> {
        layer_lines(&self.candidates, &self.choice)
    }
}

/// The candidates, one line each, marking the one that answered.
///
/// The mark is per layer, not per path: `--source <the checkout>` and the
/// checkout layer are the same directory but different answers, and the report
/// is about the answers.
fn layer_lines(candidates: &Candidates, choice: &SourceChoice) -> Vec<String> {
    let answered = choice.layer();
    let mark = |layer: usize| {
        if layer == answered {
            "used"
        } else {
            "not used"
        }
    };
    vec![
        format!(
            "moonraker:   --source: {}",
            match &candidates.given {
                Some(dir) => format!("{} ({})", dir.display(), mark(0)),
                None => "not given".to_string(),
            }
        ),
        format!(
            "moonraker:   bundled copy: {} ({})",
            candidates.bundled.display(),
            mark(1)
        ),
        format!(
            "moonraker:   checkout: {} ({})",
            candidates.checkout.display(),
            mark(2)
        ),
    ]
}

/// Resolve the arguments into one installation.
fn resolve(args: &MoonrakerArgs) -> Result<Resolution, String> {
    let candidates = Candidates::from_args(args)?;
    let choice = choose_source(&candidates)?;
    let source = choice.dir().to_path_buf();

    let data_path = expand_home(&args.data_path)?;
    let config = match &args.config {
        Some(path) => expand_home(path)?,
        None => data_path.join("config").join("moonraker.conf"),
    };
    let venv = expand_home(&args.venv)?;
    let requirements = match &args.requirements {
        Some(path) => expand_home(path)?,
        None => source.join("scripts").join("moonraker-requirements.txt"),
    };
    let wheels = match &args.wheels {
        Some(dir) => expand_home(dir)?,
        None => source.join("python_wheels"),
    };

    let service = Service {
        entry_point: source.join(ENTRY_POINT),
        source,
        data_path,
        config,
        venv,
        requirements,
        wheels,
        offline: args.offline,
    };
    Ok(Resolution {
        service,
        choice,
        candidates,
    })
}

impl Service {
    /// Prepare everything a run needs, and describe the command to run.
    ///
    /// The order matters: the data tree first (Moonraker refuses to start
    /// without it), then the configuration (so a failure to install leaves the
    /// operator a file to look at), then the virtualenv.
    fn prepare(&self, runner: &dyn CommandRunner) -> Result<Launch, String> {
        self.create_data_tree()?;
        self.write_config()?;
        self.ensure_venv(runner)?;
        Ok(Launch {
            program: self.venv.join("bin").join("python"),
            args: vec![
                "-d".to_string(),
                self.data_path.display().to_string(),
                "-c".to_string(),
                self.config.display().to_string(),
            ],
            entry_point: self.entry_point.clone(),
        })
    }

    /// Create the directories Moonraker expects, if they are not there.
    fn create_data_tree(&self) -> Result<(), String> {
        std::fs::create_dir_all(&self.data_path)
            .map_err(|error| format!("cannot create {}: {error}", self.data_path.display()))?;
        for subdir in DATA_SUBDIRS {
            let directory = self.data_path.join(subdir);
            std::fs::create_dir_all(&directory)
                .map_err(|error| format!("cannot create {}: {error}", directory.display()))?;
        }
        Ok(())
    }

    /// Write the starting configuration, if there is none.
    fn write_config(&self) -> Result<(), String> {
        if self.config.is_file() {
            return Ok(());
        }
        if let Some(parent) = self.config.parent() {
            std::fs::create_dir_all(parent)
                .map_err(|error| format!("cannot create {}: {error}", parent.display()))?;
        }
        std::fs::write(&self.config, DEFAULT_CONFIG)
            .map_err(|error| format!("cannot write {}: {error}", self.config.display()))?;
        println!(
            "moonraker: wrote {} — edit it to match this printer",
            self.config.display()
        );
        Ok(())
    }

    /// Create the virtualenv if needed and make its packages match the
    /// requirements file.
    pub fn ensure_venv(&self, runner: &dyn CommandRunner) -> Result<VenvState, String> {
        let python = self.venv.join("bin").join("python");
        let mut created = false;
        if !python.exists() {
            let venv = self.venv.display().to_string();
            let invocation = Invocation::new(PYTHON3, ["-m", "venv", &venv]);
            let completed = runner
                .run(&invocation)
                .map_err(|error| command::start_error(&invocation, &error))?;
            if !completed.success() {
                return Err(command::failure_error(&invocation, &completed));
            }
            created = true;
            println!(
                "moonraker: created the virtualenv in {}",
                self.venv.display()
            );
        }

        let wanted = self.requirements_digest()?;
        let marker = self.venv.join(MARKER_FILE);
        let recorded = std::fs::read_to_string(&marker)
            .ok()
            .map(|text| text.trim().to_string());
        if recorded.as_deref() == Some(wanted.as_str()) {
            return Ok(if created {
                VenvState::Created
            } else {
                VenvState::Ready
            });
        }

        // Only an install reads the wheels directory, so the check is here: a
        // ready environment needs nothing from it.
        self.check_wheels()?;
        let invocation = self.pip_install();
        let completed = runner
            .run(&invocation)
            .map_err(|error| command::start_error(&invocation, &error))?;
        if !completed.success() {
            // The marker is written only after a successful install, so a
            // failure leaves the previous one in place and the next run retries.
            return Err(command::failure_error(&invocation, &completed));
        }
        std::fs::write(&marker, format!("{wanted}\n"))
            .map_err(|error| format!("cannot write {}: {error}", marker.display()))?;
        println!(
            "moonraker: installed the dependencies from {}{}",
            self.requirements.display(),
            if self.offline {
                " (offline, from wheels)"
            } else {
                ""
            }
        );
        Ok(VenvState::Installed)
    }

    /// The `pip install` this environment needs: from the wheel directory when
    /// `--offline` asked for it, from the index otherwise.
    fn pip_install(&self) -> Invocation {
        let pip = self.venv.join("bin").join("pip");
        let mut arguments: Vec<String> = vec!["install".to_string()];
        if self.offline {
            arguments.push("--no-index".to_string());
            arguments.push("--find-links".to_string());
            arguments.push(self.wheels.display().to_string());
        }
        arguments.push("-r".to_string());
        arguments.push(self.requirements.display().to_string());
        Invocation::new(&pip.display().to_string(), arguments)
    }

    /// Refuse an offline install without a wheels directory.
    ///
    /// pip treats a missing `--find-links` directory as a warning and then goes
    /// to the network, which is the one thing `--offline` promises not to do.
    fn check_wheels(&self) -> Result<(), String> {
        if !self.offline || self.wheels.is_dir() {
            return Ok(());
        }
        Err(format!(
            "--offline needs a wheels directory, and {} does not exist: pip only \
             warns about a missing --find-links and would download from the network \
             instead. Point --wheels at the directory holding the wheels.",
            self.wheels.display()
        ))
    }

    /// The digest of the requirements file, as the marker records it.
    fn requirements_digest(&self) -> Result<String, String> {
        let bytes = std::fs::read(&self.requirements).map_err(|error| {
            format!(
                "cannot read the requirements file {}: {error} — initialize the \
                 checkout in the main one (`git submodule update --init \
                 third_party/moonraker`) or name a file with --requirements",
                self.requirements.display()
            )
        })?;
        Ok(sha256_hex(&bytes))
    }
}

/// What [`Service::ensure_venv`] did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VenvState {
    /// The virtualenv was created; the dependencies already matched.
    Created,
    /// The dependencies were installed.
    Installed,
    /// The virtualenv was there and its dependencies matched.
    Ready,
}

/// The command that starts Moonraker, ready to run or to print.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Launch {
    /// The interpreter: the virtualenv's own.
    pub program: PathBuf,
    /// Its arguments: `-d <data-path> -c <config>`.
    pub args: Vec<String>,
    /// The script it runs, kept out of `args` so the two are checkable apart.
    pub entry_point: PathBuf,
}

impl Launch {
    /// The command line as an operator would type it.
    pub fn command_line(&self) -> String {
        let mut line = command::quote(&self.program.display().to_string());
        for part in std::iter::once(self.entry_point.display().to_string()).chain(self.args.clone())
        {
            line.push(' ');
            line.push_str(&command::quote(&part));
        }
        line
    }

    /// Run it, relaying its output and its exit code.
    fn run_to_completion(&self) -> Result<i32, String> {
        let invocation = Invocation::new(
            &self.program.display().to_string(),
            std::iter::once(self.entry_point.display().to_string()).chain(self.args.clone()),
        );
        let runtime = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .map_err(|error| format!("cannot start a runtime: {error}"))?;
        runtime.block_on(async {
            let mut child = tokio::process::Command::new(&self.program)
                .arg(&self.entry_point)
                .args(&self.args)
                .spawn()
                .map_err(|error| command::start_error(&invocation, &error))?;
            let child_pid = child.id();
            tokio::select! {
                status = child.wait() => status
                    .map(|status| exit_code(&status))
                    .map_err(|error| format!("cannot wait for Moonraker: {error}")),
                _ = tokio::signal::ctrl_c() => {
                    // The terminal delivers Ctrl-C to the whole foreground group,
                    // so the child usually has it already; forwarding it also
                    // covers a signal aimed at this process alone.
                    if let Some(pid) = child_pid {
                        unsafe {
                            libc::kill(pid as libc::pid_t, libc::SIGINT);
                        }
                    }
                    let status = child
                        .wait()
                        .await
                        .map_err(|error| format!("cannot wait for Moonraker: {error}"))?;
                    Ok(exit_code(&status))
                }
            }
        })
    }
}

/// The exit code to report for a finished child.
///
/// A child that exited has a code of its own and that is what this program
/// returns: `klipperx moonraker` is a wrapper, not a judge. A child killed by a
/// signal has no code, so it reports the shell's convention instead.
#[cfg(unix)]
fn exit_code(status: &std::process::ExitStatus) -> i32 {
    use std::os::unix::process::ExitStatusExt;
    status
        .code()
        .unwrap_or_else(|| 128 + status.signal().unwrap_or(1))
}

/// The exit code to report for a finished child.
#[cfg(not(unix))]
fn exit_code(status: &std::process::ExitStatus) -> i32 {
    status.code().unwrap_or(1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ui::command::Completed;
    use clap::FromArgMatches;
    use std::cell::RefCell;
    use std::path::Path;
    use std::sync::atomic::{AtomicU32, Ordering};

    /// A runner that records what would have run, answers with scripted output,
    /// and can act like the real program would (creating a virtualenv).
    struct FakeRunner {
        invocations: RefCell<Vec<Invocation>>,
        status: i32,
        stderr: String,
        side_effect: Option<Box<dyn Fn(&Invocation)>>,
    }

    impl FakeRunner {
        fn ok() -> FakeRunner {
            FakeRunner {
                invocations: RefCell::new(Vec::new()),
                status: 0,
                stderr: String::new(),
                side_effect: None,
            }
        }

        fn failing(stderr: &str) -> FakeRunner {
            FakeRunner {
                status: 1,
                stderr: stderr.to_string(),
                ..FakeRunner::ok()
            }
        }

        fn with_side_effect(mut self, effect: impl Fn(&Invocation) + 'static) -> FakeRunner {
            self.side_effect = Some(Box::new(effect));
            self
        }

        fn calls(&self) -> Vec<String> {
            self.invocations
                .borrow()
                .iter()
                .map(Invocation::command_line)
                .collect()
        }
    }

    impl CommandRunner for FakeRunner {
        fn run(&self, invocation: &Invocation) -> std::io::Result<Completed> {
            self.invocations.borrow_mut().push(invocation.clone());
            if let Some(effect) = &self.side_effect {
                effect(invocation);
            }
            Ok(Completed {
                status: self.status,
                stdout: String::new(),
                stderr: self.stderr.clone(),
            })
        }
    }

    static SCRATCH: AtomicU32 = AtomicU32::new(0);

    fn scratch(name: &str) -> PathBuf {
        let directory = std::env::temp_dir().join(format!(
            "klipperx-ui-moonraker-{}-{}-{name}",
            std::process::id(),
            SCRATCH.fetch_add(1, Ordering::Relaxed)
        ));
        let _ = std::fs::remove_dir_all(&directory);
        std::fs::create_dir_all(&directory).expect("a scratch directory");
        directory
    }

    /// A Moonraker-shaped source tree under `directory/name`.
    fn source_tree(directory: &Path, name: &str, requirements: &str) -> PathBuf {
        let source = directory.join(name);
        std::fs::create_dir_all(source.join("moonraker")).expect("moonraker/");
        std::fs::write(source.join(ENTRY_POINT), "print('moonraker')\n").expect("the entry point");
        std::fs::create_dir_all(source.join("scripts")).expect("scripts/");
        std::fs::write(
            source.join("scripts").join("moonraker-requirements.txt"),
            requirements,
        )
        .expect("the requirements file");
        source
    }

    /// A service whose source, data path, venv and requirements are all under
    /// `directory`.
    fn service(directory: &Path, requirements: &str) -> Service {
        let source = source_tree(directory, "source", requirements);
        let data_path = directory.join("printer_data");
        Service {
            entry_point: source.join(ENTRY_POINT),
            requirements: source.join("scripts").join("moonraker-requirements.txt"),
            wheels: source.join("python_wheels"),
            source: source.clone(),
            offline: false,
            config: data_path.join("config").join("moonraker.conf"),
            venv: directory.join("venv"),
            data_path,
        }
    }

    /// A runner that behaves like `python3 -m venv`: it creates the virtualenv
    /// it was asked for, so the next run finds one.
    fn venv_creating_runner() -> FakeRunner {
        FakeRunner::ok().with_side_effect(|invocation| {
            if invocation.program == PYTHON3 {
                let venv = PathBuf::from(invocation.args.last().expect("the venv path"));
                std::fs::create_dir_all(venv.join("bin")).expect("bin/");
                std::fs::write(venv.join("bin").join("python"), "#!/bin/sh\n").expect("python");
            }
        })
    }

    fn candidates(directory: &Path) -> Candidates {
        Candidates {
            given: None,
            bundled: directory.join("bundled"),
            checkout: directory.join("checkout"),
        }
    }

    #[test]
    fn the_source_layers_are_tried_in_order() {
        let directory = scratch("layers");
        let given = source_tree(&directory, "given", "a\n");
        let mut candidates = candidates(&directory);
        let checkout = source_tree(&directory, "checkout", "b\n");
        candidates.checkout = checkout.clone();

        // 1. `--source`, whatever else exists.
        candidates.given = Some(given.clone());
        assert_eq!(
            choose_source(&candidates).expect("the given tree"),
            SourceChoice::Given(given.clone())
        );
        candidates.given = Some(directory.join("absent"));
        let error = choose_source(&candidates).expect_err("not a source tree");
        assert!(error.contains("--source"), "{error}");
        assert!(error.contains("moonraker/moonraker.py"), "{error}");
        candidates.given = None;

        // 2. The bundled copy, made from the checkout on the first run …
        assert_eq!(
            choose_source(&candidates).expect("a copy is made"),
            SourceChoice::Copied(candidates.bundled.clone())
        );
        assert!(is_source(&candidates.bundled), "the copy is a source tree");
        assert!(
            candidates
                .bundled
                .join("scripts")
                .join("moonraker-requirements.txt")
                .is_file(),
            "the copy carries the requirements file"
        );

        // … and used as it is afterwards.
        std::fs::write(candidates.bundled.join("marker"), "edited\n").expect("a marker");
        assert_eq!(
            choose_source(&candidates).expect("the copy is kept"),
            SourceChoice::Bundled(candidates.bundled.clone())
        );
        assert_eq!(
            std::fs::read_to_string(candidates.bundled.join("marker")).expect("the marker"),
            "edited\n"
        );

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn a_broken_copy_falls_back_to_the_checkout() {
        let directory = scratch("broken");
        let checkout = source_tree(&directory, "checkout", "b\n");
        let mut candidates = candidates(&directory);
        candidates.checkout = checkout.clone();
        // A copy that exists but has no entry point is not a copy this program
        // made; the checkout still works.
        std::fs::create_dir_all(&candidates.bundled).expect("an empty bundled dir");

        assert_eq!(
            choose_source(&candidates).expect("the checkout"),
            SourceChoice::Checkout(checkout)
        );

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn a_copy_leaves_git_and_caches_behind() {
        let directory = scratch("copy");
        let checkout = source_tree(&directory, "checkout", "b\n");
        std::fs::write(checkout.join(".git"), "gitdir: elsewhere\n").expect(".git");
        std::fs::create_dir_all(checkout.join("moonraker").join("__pycache__")).expect("cache");
        std::fs::write(
            checkout.join("moonraker").join("__pycache__").join("x.pyc"),
            "bytecode",
        )
        .expect("a cache file");

        let bundled = directory.join("bundled");
        copy_tree(&checkout, &bundled).expect("the copy");

        assert!(is_source(&bundled));
        assert!(!bundled.join(".git").exists());
        assert!(!bundled.join("moonraker").join("__pycache__").exists());

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn no_sources_anywhere_says_what_to_do() {
        let directory = scratch("none");
        let candidates = candidates(&directory);
        let error = choose_source(&candidates).expect_err("no sources");
        assert!(error.contains("--source"), "{error}");
        assert!(error.contains("bundled"), "{error}");
        assert!(error.contains("git submodule update --init"), "{error}");

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn the_layer_report_marks_the_one_that_answered() {
        let directory = scratch("lines");
        let mut candidates = candidates(&directory);
        candidates.given = Some(directory.join("given"));

        let lines = layer_lines(
            &candidates,
            &SourceChoice::Bundled(candidates.bundled.clone()),
        );
        assert_eq!(lines.len(), 3, "{lines:?}");
        assert!(lines[0].contains("--source"), "{}", lines[0]);
        assert!(lines[0].ends_with("(not used)"), "{}", lines[0]);
        assert!(lines[1].contains("bundled copy"), "{}", lines[1]);
        assert!(lines[1].ends_with("(used)"), "{}", lines[1]);
        assert!(lines[2].contains("checkout"), "{}", lines[2]);
        assert!(lines[2].ends_with("(not used)"), "{}", lines[2]);

        // `--source` naming the checkout is still the first layer answering,
        // even though both lines name the same directory.
        let same = layer_lines(
            &Candidates {
                given: Some(candidates.checkout.clone()),
                ..candidates.clone()
            },
            &SourceChoice::Given(candidates.checkout.clone()),
        );
        assert!(same[0].ends_with("(used)"), "{}", same[0]);
        assert!(same[2].ends_with("(not used)"), "{}", same[2]);

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn the_default_candidates_name_the_bundled_copy_and_the_checkout() {
        let defaults = MoonrakerArgs {
            data_path: PathBuf::from(DEFAULT_DATA_PATH),
            config: None,
            venv: PathBuf::from(DEFAULT_VENV),
            source: None,
            requirements: None,
            offline: false,
            wheels: None,
            dry_run: false,
        };
        let candidates = Candidates::from_args(&defaults).expect("the defaults resolve");

        assert!(candidates.given.is_none());
        assert_eq!(
            candidates.bundled,
            bundled_moonraker().expect("a data directory")
        );
        assert_eq!(candidates.checkout, moonraker_checkout());
        assert!(
            candidates.checkout.ends_with("third_party/moonraker"),
            "{}",
            candidates.checkout.display()
        );

        let given = Candidates::from_args(&MoonrakerArgs {
            source: Some(PathBuf::from("/srv/moonraker")),
            ..defaults
        })
        .expect("--source resolves");
        assert_eq!(given.given, Some(PathBuf::from("/srv/moonraker")));
    }

    #[test]
    fn the_arguments_parse_with_their_documented_defaults() {
        let command = <MoonrakerArgs as clap::Args>::augment_args(clap::Command::new("moonraker"));
        let parse = |arguments: &[&str]| {
            let matches = command.clone().try_get_matches_from(
                std::iter::once("moonraker").chain(arguments.iter().copied()),
            )?;
            Ok::<MoonrakerArgs, clap::Error>(
                MoonrakerArgs::from_arg_matches(&matches).expect("the arguments parse"),
            )
        };

        let defaults = parse(&[]).expect("every argument has a default");
        assert_eq!(defaults.data_path, PathBuf::from(DEFAULT_DATA_PATH));
        assert_eq!(defaults.venv, PathBuf::from(DEFAULT_VENV));
        assert!(defaults.source.is_none() && defaults.config.is_none());
        assert!(defaults.requirements.is_none() && defaults.wheels.is_none());
        assert!(!defaults.offline && !defaults.dry_run);

        assert!(
            parse(&["--wheels", "/tmp/wheels"]).is_err(),
            "--wheels is only meaningful with --offline"
        );
        let offline = parse(&["--offline", "--wheels", "/tmp/wheels", "--dry-run"])
            .expect("offline with wheels");
        assert!(offline.offline && offline.dry_run);
        assert_eq!(offline.wheels, Some(PathBuf::from("/tmp/wheels")));
    }

    #[test]
    fn the_first_run_creates_the_venv_and_installs() {
        let directory = scratch("first");
        let service = service(&directory, "tornado>=6.2.0\npyserial==3.4\n");
        let runner = venv_creating_runner();

        let state = service.ensure_venv(&runner).expect("the venv is prepared");
        assert_eq!(state, VenvState::Installed);

        let venv = service.venv.display().to_string();
        let requirements = service.requirements.display().to_string();
        assert_eq!(
            runner.calls(),
            [
                format!("python3 -m venv {venv}"),
                format!("{venv}/bin/pip install -r {requirements}"),
            ]
        );

        let marker = std::fs::read_to_string(service.venv.join(MARKER_FILE)).expect("the marker");
        assert_eq!(
            marker.trim(),
            sha256_hex(&std::fs::read(&service.requirements).expect("the requirements file"))
        );

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn a_prepared_venv_runs_nothing_at_all() {
        let directory = scratch("ready");
        let service = service(&directory, "tornado>=6.2.0\n");
        service
            .ensure_venv(&venv_creating_runner())
            .expect("the venv is prepared");

        let second = venv_creating_runner();
        let state = service.ensure_venv(&second).expect("nothing to do");
        assert_eq!(state, VenvState::Ready);
        assert!(
            second.calls().is_empty(),
            "a ready venv runs nothing: {:?}",
            second.calls()
        );

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn a_changed_requirements_file_reinstalls() {
        let directory = scratch("changed");
        let service = service(&directory, "tornado>=6.2.0\n");
        service
            .ensure_venv(&venv_creating_runner())
            .expect("the venv is prepared");

        std::fs::write(&service.requirements, "tornado>=6.2.0\npaho-mqtt==2.1.0\n")
            .expect("a new requirements file");
        let second = venv_creating_runner();
        let state = service
            .ensure_venv(&second)
            .expect("the change is installed");
        assert_eq!(state, VenvState::Installed);
        assert_eq!(
            second.calls(),
            [format!(
                "{} install -r {}",
                service.venv.join("bin").join("pip").display(),
                service.requirements.display()
            )]
        );

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn a_failed_install_keeps_the_old_marker() {
        let directory = scratch("failed");
        let service = service(&directory, "tornado>=6.2.0\n");
        service
            .ensure_venv(&venv_creating_runner())
            .expect("the venv is prepared");
        let marker_path = service.venv.join(MARKER_FILE);
        let before = std::fs::read_to_string(&marker_path).expect("the marker");

        std::fs::write(&service.requirements, "tornado>=6.2.0\nnope==1\n")
            .expect("a new requirements file");
        let failing = FakeRunner::failing(
            "Collecting nope==1\nERROR: Could not find a version that satisfies nope==1",
        );
        let error = service
            .ensure_venv(&failing)
            .expect_err("the install fails");
        assert!(error.contains("Could not find a version"), "{error}");
        assert_eq!(
            std::fs::read_to_string(&marker_path).expect("the marker"),
            before,
            "a failed install must not record success"
        );

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn offline_installs_from_the_wheels_directory() {
        let directory = scratch("offline");
        let mut service = service(&directory, "tornado>=6.2.0\n");
        service.offline = true;
        std::fs::create_dir_all(&service.wheels).expect("a wheels directory");
        std::fs::write(service.wheels.join("tornado-6.5-py3-none-any.whl"), "wheel")
            .expect("a wheel");

        let runner = venv_creating_runner();
        service.ensure_venv(&runner).expect("the offline install");

        // The virtualenv is created first; the install is the call that carries
        // the offline flags.
        let calls = runner.calls();
        assert_eq!(calls.len(), 2, "{calls:?}");
        assert_eq!(
            calls[1],
            format!(
                "{} install --no-index --find-links {} -r {}",
                service.venv.join("bin").join("pip").display(),
                service.wheels.display(),
                service.requirements.display()
            )
        );

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn offline_without_wheels_refuses_instead_of_downloading() {
        let directory = scratch("nowheels");
        let mut service = service(&directory, "tornado>=6.2.0\n");
        service.offline = true;
        assert!(!service.wheels.exists(), "the fixture has no wheels");

        let error = service
            .ensure_venv(&venv_creating_runner())
            .expect_err("offline without wheels");
        assert!(error.contains("--offline"), "{error}");
        assert!(error.contains("--wheels"), "{error}");
        assert!(error.contains("download from the network"), "{error}");
        assert!(
            !service.venv.join(MARKER_FILE).exists(),
            "a refused install records nothing"
        );

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn the_data_tree_and_a_starting_configuration_are_created() {
        let directory = scratch("tree");
        let service = service(&directory, "tornado>=6.2.0\n");

        let launch = service
            .prepare(&venv_creating_runner())
            .expect("the service prepares");

        for subdir in DATA_SUBDIRS {
            let path = service.data_path.join(subdir);
            assert!(path.is_dir(), "{} is missing", path.display());
        }
        let config = std::fs::read_to_string(&launch.args[3]).expect("the configuration");
        for expected in [
            "[server]",
            "klippy_uds_address: /tmp/klippy_uds",
            "[machine]",
            "provider: none",
            "[authorization]",
            "127.0.0.1",
            "192.168.0.0/16",
            "enable_object_processing: False",
            "[data_store]",
            "[history]",
        ] {
            assert!(
                config.contains(expected),
                "{expected} is missing:\n{config}"
            );
        }

        // An operator's own configuration is never overwritten.
        std::fs::write(&service.config, "[server]\nport: 7126\n").expect("own configuration");
        service
            .prepare(&venv_creating_runner())
            .expect("the service prepares");
        assert_eq!(
            std::fs::read_to_string(&service.config).expect("the configuration"),
            "[server]\nport: 7126\n"
        );

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn the_launch_command_names_the_venv_python_and_the_paths() {
        let directory = scratch("launch");
        let service = service(&directory, "tornado>=6.2.0\n");

        let launch = service
            .prepare(&venv_creating_runner())
            .expect("the service prepares");

        let python = service.venv.join("bin").join("python");
        assert_eq!(launch.program, python);
        assert!(python.is_file(), "the fake venv made the interpreter");
        assert_eq!(launch.entry_point, service.entry_point);
        assert_eq!(
            launch.args,
            [
                "-d".to_string(),
                service.data_path.display().to_string(),
                "-c".to_string(),
                service.config.display().to_string(),
            ]
        );
        let line = launch.command_line();
        assert!(line.starts_with(&python.display().to_string()), "{line}");
        assert!(
            line.contains(&service.entry_point.display().to_string()),
            "{line}"
        );
        assert!(
            line.contains(&format!("-d {}", service.data_path.display())),
            "{line}"
        );
        assert!(
            line.contains(&format!("-c {}", service.config.display())),
            "{line}"
        );

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn a_dry_run_stops_after_preparing() {
        let directory = scratch("dryrun");
        let service = service(&directory, "tornado>=6.2.0\n");
        let args = MoonrakerArgs {
            data_path: service.data_path.clone(),
            config: Some(service.config.clone()),
            venv: service.venv.clone(),
            source: Some(service.source.clone()),
            requirements: Some(service.requirements.clone()),
            offline: false,
            wheels: None,
            dry_run: true,
        };

        // `run_with` is what the subcommand does, so this covers `--dry-run` end
        // to end: the environment is prepared — the venv exists afterwards — and
        // the command is printed instead of being run.
        let code = run_with(&args, &venv_creating_runner()).expect("the dry run succeeds");
        assert_eq!(code, 0);
        assert!(service.venv.join("bin").join("python").is_file());

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn a_missing_requirements_file_names_the_checkout() {
        let directory = scratch("norequirements");
        let mut service = service(&directory, "tornado>=6.2.0\n");
        service.requirements = directory.join("absent.txt");

        let error = service
            .ensure_venv(&venv_creating_runner())
            .expect_err("nothing to install from");
        assert!(error.contains("git submodule update --init"), "{error}");
        assert!(error.contains("--requirements"), "{error}");

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }
}
