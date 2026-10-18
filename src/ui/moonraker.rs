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
//! 1. `--source <DIR>` — a source tree to run as it is.
//! 2. `<data-path>/klipperx/moonraker` — a copy kept beside the data, made from
//!    the checkout on the first run and then used as it is.
//! 3. `<repo>/third_party/moonraker` — the development checkout.
//!
//! **Where the dependencies come from**: a virtualenv of this program's own
//! (`~/.local/share/klipperx/moonraker-venv` by default), created on first run,
//! with `scripts/moonraker-requirements.txt` installed into it. "Installed" is
//! not a guess: the requirements file's SHA-256 is written into the virtualenv as
//! [`MARKER_FILE`]; a run whose recorded digest matches does nothing at all — no
//! `pip`, no network, no wait — and a run whose digest differs reinstalls. A
//! failed install leaves the old marker, so the next run retries rather than
//! believing a broken environment is ready.
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
use crate::ui::{expand_home, repo_root};

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
  1. --source <DIR>                 a source tree to run as it is
  2. <data-path>/klipperx/moonraker copied there from the checkout on first run,
                                    and used as it is afterwards
  3. <repo>/third_party/moonraker   the development checkout

The copy named in 2 is made once: a newer checkout is not picked up by itself —
pass --source for a checkout being edited. --dry-run prepares the environment,
prints the sources, the virtualenv and the command line, and does not start
Moonraker.";

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

    /// The Moonraker sources to run (default: <data-path>/klipperx/moonraker, else the checkout)
    #[arg(long, value_name = "DIR")]
    pub source: Option<PathBuf>,

    /// The requirements file to install (default: <source>/scripts/moonraker-requirements.txt)
    #[arg(long, value_name = "FILE")]
    pub requirements: Option<PathBuf>,

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
    let service = Service::resolve(args)?;
    // One line, always: which sources this run actually executes.
    println!(
        "moonraker: source: {} ({})",
        service.source.display(),
        service.layer.describe()
    );
    let launch = service.prepare(runner)?;
    if args.dry_run {
        println!("moonraker: venv: {}", service.venv.display());
    }
    println!("moonraker: {}", launch.command_line());
    if args.dry_run {
        println!("moonraker: --dry-run, not starting");
        return Ok(0);
    }
    launch.run_to_completion()
}

/// Which of the three places the sources came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Source {
    /// `--source`.
    Given,
    /// The copy under the data path, already there.
    Bundled,
    /// The copy under the data path, just made from the checkout.
    Copied,
    /// The checkout itself.
    Checkout,
}

impl Source {
    /// How to say which layer answered.
    fn describe(self) -> &'static str {
        match self {
            Source::Given => "from --source",
            Source::Bundled => "the bundled copy",
            Source::Copied => "the bundled copy, just copied from the checkout",
            Source::Checkout => "the checkout",
        }
    }
}

/// The Moonraker checkout: the Python sources `moonraker/moonraker.py` lives in.
fn checkout() -> PathBuf {
    repo_root().join("third_party").join("moonraker")
}

/// Where the sources are kept when they are not taken from a checkout:
/// `<data-path>/klipperx/moonraker`, beside the data Moonraker itself writes.
fn bundled_dir(data_path: &Path) -> PathBuf {
    data_path.join("klipperx").join("moonraker")
}

/// Whether `dir` is a Moonraker source tree: the entry point is what is run, so
/// its presence is the test.
fn is_source(dir: &Path) -> bool {
    dir.join("moonraker").join("moonraker.py").is_file()
}

/// The source resolution: `--source`, then the bundled copy, then the checkout.
///
/// The bundled copy is made here on a first run, because that is the only moment
/// it can be: once it exists it is what runs.
fn choose_source(
    given: Option<&Path>,
    bundled: &Path,
    checkout: &Path,
) -> Result<(PathBuf, Source), String> {
    if let Some(given) = given {
        if !is_source(given) {
            return Err(format!(
                "--source {} is not a Moonraker source tree: {} does not exist",
                given.display(),
                given.join(ENTRY_POINT).display()
            ));
        }
        return Ok((given.to_path_buf(), Source::Given));
    }

    if is_source(bundled) {
        return Ok((bundled.to_path_buf(), Source::Bundled));
    }

    if !bundled.exists() && is_source(checkout) {
        copy_tree(checkout, bundled).map_err(|error| {
            format!(
                "cannot make the bundled copy in {}: {error}",
                bundled.display()
            )
        })?;
        if is_source(bundled) {
            return Ok((bundled.to_path_buf(), Source::Copied));
        }
    }

    if is_source(checkout) {
        if bundled.exists() {
            println!(
                "moonraker: warning: {} is not a usable copy ({ENTRY_POINT} is missing); \
                 using the checkout",
                bundled.display()
            );
        }
        return Ok((checkout.to_path_buf(), Source::Checkout));
    }

    Err(format!(
        "no Moonraker sources: --source was not given, {} is not a copy, and {} is \
         not a source tree. An empty submodule is not a checkout — initialize it in \
         the main one (`git submodule update --init third_party/moonraker`) or pass \
         --source",
        bundled.display(),
        checkout.display()
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
    /// Which layer that source came from.
    layer: Source,
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
}

impl Service {
    /// Resolve the arguments into one installation, making the bundled copy on a
    /// first run.
    fn resolve(args: &MoonrakerArgs) -> Result<Service, String> {
        let data_path = expand_home(&args.data_path)?;
        let config = match &args.config {
            Some(path) => expand_home(path)?,
            None => data_path.join("config").join("moonraker.conf"),
        };
        let venv = expand_home(&args.venv)?;
        let given = match &args.source {
            Some(dir) => Some(expand_home(dir)?),
            None => None,
        };
        let (source, layer) =
            choose_source(given.as_deref(), &bundled_dir(&data_path), &checkout())?;
        let requirements = match &args.requirements {
            Some(path) => expand_home(path)?,
            None => source.join("scripts").join("moonraker-requirements.txt"),
        };
        Ok(Service {
            entry_point: source.join(ENTRY_POINT),
            source,
            layer,
            data_path,
            config,
            venv,
            requirements,
        })
    }

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

        let pip = self.venv.join("bin").join("pip").display().to_string();
        let invocation = Invocation::new(
            &pip,
            [
                "install".to_string(),
                "-r".to_string(),
                self.requirements.display().to_string(),
            ],
        );
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
            "moonraker: installed the dependencies from {}",
            self.requirements.display()
        );
        Ok(VenvState::Installed)
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
        fn run(&self, invocation: &Invocation) -> Result<Completed, std::io::Error> {
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
            source: source.clone(),
            layer: Source::Given,
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

    #[test]
    fn the_source_layers_are_tried_in_order() {
        let directory = scratch("layers");
        let given = source_tree(&directory, "given", "a\n");
        let checkout = source_tree(&directory, "checkout", "b\n");
        let bundled = directory.join("data").join("klipperx").join("moonraker");
        assert_eq!(
            bundled,
            bundled_dir(&directory.join("data")),
            "the copy lives under the data path"
        );

        // 1. `--source`, whatever else exists.
        assert_eq!(
            choose_source(Some(&given), &bundled, &checkout).expect("the given tree"),
            (given.clone(), Source::Given)
        );
        let absent = directory.join("absent");
        let error =
            choose_source(Some(&absent), &bundled, &checkout).expect_err("not a source tree");
        assert!(error.contains("--source"), "{error}");
        assert!(error.contains("moonraker/moonraker.py"), "{error}");

        // 2. The bundled copy, made from the checkout on the first run …
        assert_eq!(
            choose_source(None, &bundled, &checkout).expect("a copy is made"),
            (bundled.clone(), Source::Copied)
        );
        assert!(is_source(&bundled), "the copy is a source tree");
        assert!(
            bundled
                .join("scripts")
                .join("moonraker-requirements.txt")
                .is_file(),
            "the copy carries the requirements file"
        );

        // … and used as it is afterwards.
        std::fs::write(bundled.join("marker"), "edited\n").expect("a marker");
        assert_eq!(
            choose_source(None, &bundled, &checkout).expect("the copy is kept"),
            (bundled.clone(), Source::Bundled)
        );
        assert_eq!(
            std::fs::read_to_string(bundled.join("marker")).expect("the marker"),
            "edited\n"
        );

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn a_broken_copy_falls_back_to_the_checkout() {
        let directory = scratch("broken");
        let checkout = source_tree(&directory, "checkout", "b\n");
        let bundled = directory.join("bundled");
        // A copy that exists but has no entry point is not a copy this program
        // made; the checkout still works.
        std::fs::create_dir_all(&bundled).expect("an empty bundled dir");

        assert_eq!(
            choose_source(None, &bundled, &checkout).expect("the checkout"),
            (checkout.clone(), Source::Checkout)
        );
        // And with nothing anywhere, the error says what to do.
        let error =
            choose_source(None, &bundled, &directory.join("absent")).expect_err("no sources");
        assert!(error.contains("--source"), "{error}");
        assert!(error.contains("git submodule update --init"), "{error}");

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

        let bundled = bundled_dir(&directory);
        copy_tree(&checkout, &bundled).expect("the copy");

        assert!(is_source(&bundled));
        assert!(!bundled.join(".git").exists());
        assert!(!bundled.join("moonraker").join("__pycache__").exists());

        std::fs::remove_dir_all(&directory).expect("cleanup");
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
        assert!(defaults.requirements.is_none());
        assert!(!defaults.dry_run);

        let given = parse(&["--source", "/srv/moonraker", "--dry-run"]).expect("parsed");
        assert_eq!(given.source, Some(PathBuf::from("/srv/moonraker")));
        assert!(given.dry_run);
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
