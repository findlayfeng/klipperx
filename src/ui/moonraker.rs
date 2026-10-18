//! `klipperx moonraker` — run Moonraker, keeping its Python environment itself.
//!
//! Moonraker is a Python application with a dozen pinned dependencies, and the
//! usual way to run it is to remember which virtualenv it lives in and when it
//! was last installed into. This subcommand removes both: it owns a virtualenv of
//! its own (`~/.local/share/klipperx/moonraker-venv` by default), creates it on
//! first run, installs `scripts/moonraker-requirements.txt` into it, and starts
//! `moonraker/moonraker.py` with the interpreter it just prepared.
//!
//! "Installed" is not a guess. The requirements file's SHA-256 is written into
//! the virtualenv as [`MARKER_FILE`]; a run whose recorded digest matches does
//! nothing at all — no `pip`, no network, no wait — and a run whose digest
//! differs (the checkout moved, or someone edited the file) reinstalls. A
//! failed install leaves the old marker in place, so the next run tries again
//! rather than believing a broken environment is ready.
//!
//! Moonraker needs a directory tree around it as much as it needs an
//! interpreter: `config/`, `logs/`, `gcodes/` and friends under its data path,
//! and a `moonraker.conf` inside `config/`. Both are created here when absent,
//! the configuration as a minimal file that points Moonraker at the host's
//! socket (`/tmp/klippy_uds`, the `-a` the `klippy` subcommand defaults to) and
//! trusts the local networks — a starting point to edit, not a considered
//! configuration.

use std::path::PathBuf;

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

    /// The requirements file to install (default: the moonraker checkout's)
    #[arg(long, value_name = "FILE")]
    pub requirements: Option<PathBuf>,

    /// Prepare the environment and print the command; do not start Moonraker
    #[arg(long)]
    pub dry_run: bool,
}

/// Run `klipperx moonraker`: prepare the environment, then hand over to
/// Moonraker and relay its exit code.
pub fn run(args: MoonrakerArgs) -> Result<i32, Box<dyn std::error::Error>> {
    run_with(&args, &SystemRunner).map_err(Into::into)
}

/// [`run`] with the program runner given, for tests.
fn run_with(args: &MoonrakerArgs, runner: &dyn CommandRunner) -> Result<i32, String> {
    let service = Service::resolve(args)?;
    let launch = service.prepare(runner)?;
    println!("moonraker: {}", launch.command_line());
    if args.dry_run {
        println!("moonraker: --dry-run, not starting");
        return Ok(0);
    }
    launch.run_to_completion()
}

/// One Moonraker installation, resolved to absolute paths.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Service {
    /// Moonraker's data directory.
    pub data_path: PathBuf,
    /// Its configuration file.
    pub config: PathBuf,
    /// The virtualenv its dependencies live in.
    pub venv: PathBuf,
    /// The requirements file those dependencies are read from.
    pub requirements: PathBuf,
    /// `moonraker/moonraker.py` in the checkout.
    pub entry_point: PathBuf,
}

impl Service {
    /// Resolve the arguments against the checkout and the home directory.
    pub fn resolve(args: &MoonrakerArgs) -> Result<Service, String> {
        let root = repo_root();
        let data_path = expand_home(&args.data_path)?;
        let config = match &args.config {
            Some(path) => expand_home(path)?,
            None => data_path.join("config").join("moonraker.conf"),
        };
        let venv = expand_home(&args.venv)?;
        let requirements = match &args.requirements {
            Some(path) => expand_home(path)?,
            None => root
                .join("third_party")
                .join("moonraker")
                .join("scripts")
                .join("moonraker-requirements.txt"),
        };
        Ok(Service {
            data_path,
            config,
            venv,
            requirements,
            entry_point: root
                .join("third_party")
                .join("moonraker")
                .join("moonraker")
                .join("moonraker.py"),
        })
    }

    /// Prepare everything a run needs, and describe the command to run.
    ///
    /// The order matters: the data tree first (Moonraker refuses to start
    /// without it), then the configuration (so a failure to install leaves the
    /// operator a file to look at), then the virtualenv.
    pub fn prepare(&self, runner: &dyn CommandRunner) -> Result<Launch, String> {
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
        let requirements = self.requirements.display().to_string();
        let invocation = Invocation::new(&pip, ["install", "-r", &requirements]);
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

    /// A service whose data path, venv and requirements all live in `directory`.
    fn service(directory: &Path, requirements: &str) -> Service {
        let requirements_path = directory.join("moonraker-requirements.txt");
        std::fs::write(&requirements_path, requirements).expect("the requirements file");
        let data_path = directory.join("printer_data");
        Service {
            config: data_path.join("config").join("moonraker.conf"),
            venv: directory.join("venv"),
            requirements: requirements_path,
            entry_point: directory.join("moonraker.py"),
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

    fn args_for(service: &Service, dry_run: bool) -> MoonrakerArgs {
        MoonrakerArgs {
            data_path: service.data_path.clone(),
            config: Some(service.config.clone()),
            venv: service.venv.clone(),
            requirements: Some(service.requirements.clone()),
            dry_run,
        }
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
        let config = std::fs::read_to_string(&launch.args[3].clone()).expect("the configuration");
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
        let args = args_for(&service, true);

        // The environment is prepared — the venv exists afterwards — and the
        // command is printed instead of being run. `run_with` is what the
        // subcommand does, so this covers `--dry-run` end to end.
        let code = run_with(&args, &venv_creating_runner()).expect("the dry run succeeds");
        assert_eq!(code, 0);
        assert!(service.venv.join("bin").join("python").is_file());

        std::fs::remove_dir_all(&directory).expect("cleanup");
    }

    #[test]
    fn the_defaults_point_at_the_moonraker_checkout() {
        let service = Service::resolve(&MoonrakerArgs {
            data_path: PathBuf::from(DEFAULT_DATA_PATH),
            config: None,
            venv: PathBuf::from(DEFAULT_VENV),
            requirements: None,
            dry_run: false,
        })
        .expect("the defaults resolve");

        let home = PathBuf::from(std::env::var("HOME").expect("HOME is set for tests"));
        assert_eq!(service.data_path, home.join("printer_data"));
        assert_eq!(
            service.config,
            home.join("printer_data")
                .join("config")
                .join("moonraker.conf")
        );
        assert_eq!(
            service.venv,
            home.join(".local")
                .join("share")
                .join("klipperx")
                .join("moonraker-venv")
        );
        assert!(
            service
                .requirements
                .ends_with("third_party/moonraker/scripts/moonraker-requirements.txt"),
            "{}",
            service.requirements.display()
        );
        assert!(
            service
                .entry_point
                .ends_with("third_party/moonraker/moonraker/moonraker.py"),
            "{}",
            service.entry_point.display()
        );
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
