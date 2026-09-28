use clap::Parser;
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use tracing::{debug, info, warn};

use crate::core::klippy::api::{
    self, AddressError, Api, ApiTarget, RegistrationError, Server, StartArgs, WebhooksStatus,
    WEBHOOKS_OBJECT,
};
use crate::core::klippy::config::Config;
use crate::core::klippy::printer::Printer;
use crate::core::klippy::reactor::TokioReactor;
use crate::logging;

/// What the host is, in one line.
///
/// The `klippy` binary and the `klippy` subcommand of `klipperx` are the same
/// program spelled two ways, so they say the same thing about themselves — and
/// say it once.
pub const ABOUT: &str = "Run the host: load the config and serve the API";

/// Klippy CLI application
#[derive(Parser, Debug)]
#[command(name = "klippy", version, about)]
pub struct AppArgs {
    /// API server listen target: a socket path, or `tcp:<host:port>`
    ///
    /// Defaults to the shared API path, so that a client started without
    /// arguments finds this host. Give an empty value to serve nothing at all —
    /// the API has no authentication, and a host that does not want one should
    /// not have to guess at a path that disables it.
    #[arg(
        short,
        long,
        value_name = "ADDR",
        default_value = klippy_api::address::DEFAULT_API_SERVER
    )]
    pub api_server: String,

    /// Config file path
    ///
    /// Optional to the parser, required to [`run`]: a subcommand can be given
    /// instead of a config file, and clap cannot express "required unless one of
    /// my subcommands is used" for an argument that arrives through a `flatten`.
    /// The check below therefore says the same thing, with the same wording.
    pub config_file: Option<String>,

    /// Write the log to this file as well as to stdout
    ///
    /// Upstream's `--logfile` (`klippy/klippy.py:294-345`): the same lines that
    /// go to the terminal, kept in a file. `info` reports the path as
    /// `log_file`; leaving the option out reports `null`, which is upstream's
    /// answer too.
    #[arg(long = "logfile", value_name = "PATH")]
    pub log_file: Option<String>,
}

/// How long to wait before a rebuilt printer is brought up again.
///
/// It gives a device that was just closed (a serial port, a CAN socket) time to
/// come back, and rate-limits a restart that returns straight away — a config
/// the loader rejects, an MCU that is gone.
const RESTART_DELAY: std::time::Duration = std::time::Duration::from_millis(1000);

/// How long a reactor dispatch round may run before it is reported.
///
/// Upstream's garbage collector uses the same 50 ms threshold
/// (`klippy/extras/garbage_collection.py`, `THRESHOLD`). The subscription tick
/// is the only periodic callback and normally takes well under a millisecond,
/// so a round this long means something blocked the reactor.
const LATENCY_WARNING: f64 = 0.05;

/// Whether a run result asks for the printer to be built again.
///
/// `"exit"` / `"error_exit"` end the process; these two rebuild the machine
/// (upstream's `klippy/klippy.py:355-370`).
fn is_restart(result: &str) -> bool {
    matches!(result, "restart" | "firmware_restart")
}

/// The `versions` rollover block: what a bug report needs first.
///
/// Upstream builds the same block from its args and version
/// (`klippy/klippy.py:345-357`); this host has no git checkout or interpreter to
/// name, so it reports the two facts that identify the run.
fn versions_block(config_file: &str) -> String {
    format!(
        "Versions: klipperx {}\nConfig: {config_file}",
        env!("CARGO_PKG_VERSION")
    )
}

/// The process exit code a finished run asks for.
///
/// `error_exit` is upstream's `sys.exit(-1)` (`klippy/klippy.py:375`); every
/// other end of the loop — `exit`, or an attachment that finished — exits 0.
fn exit_code(result: &str) -> i32 {
    if result == "error_exit" {
        -1
    } else {
        0
    }
}

/// Log a reactor that ran a dispatch round past [`LATENCY_WARNING`].
///
/// Upstream's `_analyze_callback` (`extras/garbage_collection.py:20`) spells the
/// same thing; here the names come from registration rather than introspection.
fn report_reactor_latency(report: crate::core::klippy::LatencyReport) {
    warn!(
        "Reactor busy for {:.3}s with {} callback(s):",
        report.busy,
        report.callbacks.len()
    );
    for callback in &report.callbacks {
        warn!(
            "  {} ran {:.3}s late, took {:.3}s",
            callback.name, callback.lateness, callback.duration
        );
    }
}

/// Run one printer until something asks it to stop, rebuilding it on a restart.
///
/// This future runs **on the machine runtime**: the reactor's dispatcher, the
/// MCU transport tasks, the device's blocking I/O and the blocking `run()` loop
/// below all belong to that runtime, and this is the future its driver thread
/// `block_on`s. The interrupt that asks the printer to exit is a host concern
/// and lives on the API runtime instead (see [`run`]); `request_exit` is a
/// condition variable, so crossing runtimes is fine.
///
/// The machine is built, its config loaded and the API server serving it by the
/// time this is called. The run loop blocks, so it gets a blocking thread of its
/// own.
///
/// A restart reads the config file again, as upstream's `_connect` →
/// `_read_config` does (`klippy/klippy.py:128`, `:114-121`): an edit made while
/// the printer was running takes effect at the next `RESTART`, and a file that
/// no longer parses leaves the printer in its `error` state, which a later
/// `RESTART` can fix without the process going away. The machine is reset and
/// reloaded **in place**: the same `Arc<Printer>` keeps serving, so the
/// endpoints and any attached window survive a restart (this is the answer to
/// Q7 — no printer slot to swap, because the printer is rebuilt under its one
/// handle).
async fn klippy_process(printer: Arc<Printer>, config_file: String) -> String {
    let result = loop {
        printer.bring_up().await;

        let result = {
            let printer = Arc::clone(&printer);
            tokio::task::spawn_blocking(move || printer.run()).await
        };

        let result = match result {
            Ok(result) => result,
            Err(err) => {
                warn!("the run loop did not finish: {err}");
                break "error_exit".to_string();
            }
        };

        if !is_restart(&result) {
            debug!("printer stopped: {result}");
            break result;
        }

        info!("Restarting the printer ({result})");
        // A firmware restart resets the firmware on the live connection before
        // the parts come down (`PrinterObject::before_firmware_restart`); a plain
        // restart only rebuilds them.
        if result == "firmware_restart" {
            printer.prepare_firmware_restart().await;
        }
        printer.reset_for_restart(&result);
        // Read from disk again: upstream builds a fresh `Printer` per restart
        // and its `_connect` reads the config then (`klippy/klippy.py:353-360`),
        // so a file edited while the printer was running is what comes up.
        match Config::from_file(&config_file) {
            Ok((config, _sources)) => {
                if let Err(err) = printer.load_config(&config) {
                    // A config the reload rejects is an `error`, not a shutdown:
                    // upstream's `_connect` sets the state and lets a `RESTART`
                    // fix it.
                    printer.set_error_state(&format!("{err}"));
                }
            }
            // A file that cannot be read or parsed is the same story: the
            // printer reports why and waits for the next `RESTART`; the run
            // loop keeps turning, so a client can still read the state.
            Err(err) => printer.set_error_state(&err),
        }
        tokio::time::sleep(RESTART_DELAY).await;
        // A restart is this host's log rollover: mark the seam again.
        logging::write_rollover();
    };

    // The run loop has ended for good, so the machine comes down here — while
    // the runtime that built it is still up. A device's transport can have a
    // blocking read parked on a worker thread (an MCU's receive task runs one),
    // and only dropping the part releases it; waiting for the `Printer` to drop
    // would leave that read parked, because the API endpoints keep the printer
    // alive past the run loop, and the runtime then hangs on shutdown.
    printer.teardown();
    result
}

/// Something to run alongside the host, attached to its own API.
///
/// `klipperx --tui` is the only implementation: a client talking to this host
/// over an in-process pipe. It is a trait so that the host does not have to know
/// that a terminal, a client or a TUI library exists — and so that the `klippy`
/// binary, which never runs a window, does not link any of it.
pub trait Attachment {
    /// Run until the attachment is finished, with the endpoint table the host is
    /// serving.
    ///
    /// Returning stops the host: whatever is attached is the user interface of
    /// the invocation that asked for it. The endpoint table is the host's, so an
    /// attachment that wants to talk to it can serve one end of its own pipe
    /// with it (`klippy_api::server::serve`).
    fn run<'a>(
        &'a mut self,
        api: Arc<Api>,
    ) -> Pin<Box<dyn Future<Output = Result<(), String>> + 'a>>;
}

/// Main run function for klippy subcommand.
///
/// `attachment` is an optional client to run alongside the host, on the host's
/// own API; see [`Attachment`].
///
/// Returns the process exit code the run asks for: `0` for a normal end, `-1`
/// when the run ended because the printer asked for `error_exit` — upstream's
/// `sys.exit(-1)` after `printer.run()` (`klippy/klippy.py:375`). The caller
/// exits with it.
pub fn run(
    args: AppArgs,
    attachment: Option<Box<dyn Attachment>>,
) -> Result<i32, Box<dyn std::error::Error>> {
    let config_file = args
        .config_file
        .ok_or("the following required argument was not provided: <CONFIG_FILE>")?;
    debug!("Config file: {config_file}");

    // The log's rollover information goes at the top of the file, so a log a
    // user attaches already says which build and config produced it (upstream's
    // `versions` block, `klippy/klippy.py:345-357`).
    logging::clear_rollover_info();
    logging::set_rollover_info("versions", Some(&versions_block(&config_file)));
    logging::write_rollover();

    // Parse the config file.
    //
    // This parse feeds the start-up load only: the run loop re-reads the file on
    // every restart (see `klippy_process`), so it is handed the path rather than
    // this value.
    let (config, sources) = Config::from_file(&config_file)?;
    debug!("Config sources: {:?}", sources);
    info!(
        "Successfully parsed config with {} sections",
        config.sections_vec().len()
    );
    for section in config.sections() {
        debug!("Section: {}", section.identifier());
    }

    // Resolving the target before the runtime starts means a typo is reported
    // like any other bad option, not after the printer has begun to come up.
    let target = if args.api_server.trim() == klippy_api::address::NO_API_SERVER {
        None
    } else {
        match args.api_server.parse::<ApiTarget>() {
            Ok(target) => Some(target),
            Err(err) => {
                // A bad address is an option error, so it is reported the way
                // clap reports one.
                let err: AddressError = err;
                return Err(format!("{err}\n\nFor more information, try '--help'.").into());
            }
        }
    };

    // The machine gets its own runtime and its own driver thread: its timers,
    // MCU transport tasks and device I/O must not queue behind client traffic,
    // and a slow endpoint must not be able to delay a machine callback. The API
    // keeps the process's multi-threaded runtime. This is TODO A3 — two
    // runtimes, split along the boundary that matters for timing.
    //
    // Two workers for the machine, not more: the device's blocking reads run on
    // the blocking pool (see `Interface`), so the workers only carry the
    // reactor dispatcher, the MCU send and receive tasks, and bring-up/restart.
    let machine_runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .thread_name("klippy-mcu")
        .enable_all()
        .build()?;
    let machine_handle = machine_runtime.handle().clone();

    // The reactor is built over the machine runtime and handed to the printer
    // explicitly, so the machine never has to ask for the ambient runtime. The
    // dispatcher task is spawned here, on that handle, before the runtime is
    // driven — it runs as soon as the driver thread starts.
    let printer = Arc::new(Printer::new(Arc::new(TokioReactor::new(
        machine_handle.clone(),
    ))));

    // A diagnostic for the machine's own timing: if a dispatch round runs
    // longer than this, say so, naming the callbacks that ran in it. Upstream
    // wires the same notifier from its garbage collector
    // (`extras/garbage_collection.py`); we have none, so the warning is the
    // whole point (TODO A1b).
    printer
        .reactor()
        .set_latency_notifier(LATENCY_WARNING, Arc::new(report_reactor_latency));

    // The API runtime: the accept loop, one task per connection, and whatever is
    // attached to the host. It is multi-threaded because endpoint handlers are
    // independent work and one slow client must not hold up another.
    let api_runtime = tokio::runtime::Builder::new_multi_thread()
        .thread_name("klippy-api")
        .enable_all()
        .build()?;

    api_runtime.block_on(async move {
        // The server's own object (`webhooks`) is registered before the config
        // is read, because that is how an extras module reaches the server
        // while it loads: `[adxl345]` registers `adxl345/dump_adxl345` on it
        // (upstream adds the object in `Printer.__init__`,
        // `klippy/klippy.py:36-40`). The endpoint table itself is built after
        // the read, so the mux endpoints it collected can go into it.
        api::webhooks::install(&printer).map_err(RegistrationError::Status)?;

        // The start arguments are host data: the printer keeps a copy (upstream
        // `printer.start_args`, `klippy/klippy.py:30`) so that modules such as
        // `M115` and `error_mcu` can read them, and the API reports the same
        // dictionary.
        let mut start_args = StartArgs::collect(config_file.clone(), args.log_file.clone());
        start_args.apiserver = Some(args.api_server.clone());
        printer.set_start_args(Arc::new(start_args.clone()));

        debug!(
            "Klippy process started with {} sections",
            config.sections_vec().len()
        );

        // Config-driven objects, run under the machine runtime's context so any
        // machine-side handle the loader picks up is the machine's. A config the
        // loader rejects does not end the process: the printer reports an `error`
        // state, clients can still connect and read why, and a `RESTART` retries
        // (upstream's `_read_config` does the same — it sets the state and lets
        // the reactor keep running).
        {
            let _machine = machine_handle.enter();
            if let Err(err) = printer.load_config(&config) {
                printer.set_error_state(&format!("{err}"));
            }
        }

        // The API table comes after the config, so that the mux endpoints the
        // modules registered on `webhooks` are part of it before any client can
        // send a request. Everything is registered before the listener is
        // bound.
        let mut api = Api::new();
        api::register(&mut api, &printer, start_args)?;
        let api = Arc::new(api);
        // Hand the table to the `webhooks` object, which is how a module pushes
        // to a connection that registered a remote method
        // (`call_remote_method`).
        if let Some(webhooks) = printer.lookup_object_as::<WebhooksStatus>(WEBHOOKS_OBJECT) {
            webhooks.set_api(Arc::clone(&api));
        }

        let server = match target {
            None => {
                info!("Empty --api-server: not starting the API server");
                None
            }
            Some(target) => {
                let server = Server::bind(target, Arc::clone(&api)).await?;
                // `target` is resolved by the bind, so a `tcp:…:0` port is
                // reported as the one the kernel chose.
                info!("API server listening on {}", server.target());
                Some(tokio::spawn(server.run()))
            }
        };

        // The operator's interrupt is a host concern, so it runs on the API
        // runtime; it only asks the printer to exit, and `request_exit` is a
        // condition variable, so it crosses runtimes freely. A task rather than
        // a `select!`, so the blocking run loop below is awaited exactly once,
        // and so an exit requested from somewhere else (an attached window
        // closing) ends it without waiting for an interrupt. It is a loop
        // because a restart clears the exit request, so the listener has to be
        // armed again for the next run.
        //
        // Both SIGINT (Ctrl-C) and SIGTERM (`systemctl stop`, `docker stop`) ask
        // for a clean exit. That matters beyond tidiness: killing the process
        // while an MCU handshake is outstanding stops the host reading the
        // port, and the firmware's USB endpoint stays full — the board then
        // answers nothing until it resets. Asking the run loop to exit lets
        // bring-up finish (it keeps reading) before the process goes.
        let interrupt = {
            let printer = Arc::clone(&printer);
            tokio::spawn(async move {
                let mut terminate =
                    match tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
                    {
                        Ok(signal) => signal,
                        Err(err) => {
                            warn!("cannot listen for SIGTERM: {err}");
                            return;
                        }
                    };
                loop {
                    tokio::select! {
                        result = tokio::signal::ctrl_c() => {
                            if let Err(err) = result {
                                warn!("cannot listen for an interrupt: {err}");
                                return;
                            }
                        }
                        _ = terminate.recv() => {}
                    }
                    printer.request_exit("exit");
                }
            })
        };

        // The machine runs on its own runtime, driven from a thread of its own:
        // `block_on` cannot be nested, so the API runtime cannot drive it. Its
        // driver thread parks in `block_on` for the whole run, which is fine —
        // the machine's tasks run on that runtime's workers, not on this thread.
        let machine_printer = Arc::clone(&printer);
        // The run loop re-reads this file on every restart, so it is handed the
        // path rather than the parse above.
        let machine_config_file = config_file.clone();
        let machine_thread = std::thread::Builder::new()
            .name("klippy-machine".into())
            .spawn(move || {
                machine_runtime.block_on(klippy_process(machine_printer, machine_config_file))
            })?;

        let exit_code = match attachment {
            // Whatever is attached to this host is the user interface of the
            // invocation that asked for it, so when it is done the host is too
            // — and the host's own shutdown conditions end the attachment
            // instead. Asking the printer to exit is what lets its run loop
            // return in order, rather than dropping a loop that still holds the
            // machine.
            Some(mut attachment) => {
                let outcome = attachment.run(Arc::clone(&api)).await;
                printer.request_exit("exit");
                let result = machine_thread
                    .join()
                    .unwrap_or_else(|_| "error_exit".to_string());
                outcome.map_err(|err| -> Box<dyn std::error::Error> { err.into() })?;
                exit_code(&result)
            }
            None => {
                let result = machine_thread
                    .join()
                    .unwrap_or_else(|_| "error_exit".to_string());
                exit_code(&result)
            }
        };

        interrupt.abort();
        let _ = interrupt.await;

        // The printer has stopped, so the API server goes with it. Aborting is
        // enough: dropping the listener removes the socket file.
        if let Some(handle) = server {
            handle.abort();
            let _ = handle.await;
        }

        Ok::<i32, Box<dyn std::error::Error>>(exit_code)
    })
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::object::{PrinterConfig, CONFIGFILE_OBJECT};
    use crate::core::klippy::config::Config;
    use crate::core::klippy::printer::{ConnectFuture, PrinterObject, PrinterState};
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::Value;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::{Mutex, Weak};

    /// Write a config file for one test and return its path.
    ///
    /// The name carries the test's own tag, so parallel tests in this module do
    /// not share a file, and `std::process::id()` keeps concurrent `cargo test`
    /// processes apart — the pattern the other temp-file tests here use.
    fn write_temp_config(tag: &str, content: &str) -> String {
        let path = std::env::temp_dir().join(format!(
            "klipperx-klippy-restart-{tag}-{}.cfg",
            std::process::id()
        ));
        std::fs::write(&path, content).expect("the config file is written");
        path.to_str().expect("the temp path is utf-8").to_string()
    }

    #[test]
    fn test_only_a_restart_result_rebuilds_the_printer() {
        // The two spellings `gcode/restart` and `gcode/firmware_restart` exit
        // with; everything else stops the process.
        assert!(is_restart("restart"));
        assert!(is_restart("firmware_restart"));
        assert!(!is_restart("exit"));
        assert!(!is_restart("error_exit"));
    }

    #[test]
    fn test_error_exit_is_the_only_non_zero_exit_code() {
        // Upstream `sys.exit(-1)` after `printer.run()`
        // (`klippy/klippy.py:375`); every other result is a normal end.
        assert_eq!(exit_code("error_exit"), -1);
        assert_eq!(exit_code("exit"), 0);
        assert_eq!(exit_code("restart"), 0);
    }

    /// A host part that asks for a restart the first time it connects, and for
    /// an exit the second — enough to drive one turn of the loop. It is a *host*
    /// part (registered before `mark_host_objects`), so a reset keeps it and it
    /// connects again.
    struct RestartOnce {
        connects: Arc<AtomicUsize>,
        printer: Weak<Printer>,
    }

    impl PrinterObject for RestartOnce {
        fn get_status(&self, _eventtime: f64) -> Value {
            serde_json::json!({})
        }

        fn connect<'a>(&'a self) -> ConnectFuture<'a> {
            let result = if self.connects.fetch_add(1, Ordering::SeqCst) == 0 {
                "restart"
            } else {
                "exit"
            };
            if let Some(printer) = self.printer.upgrade() {
                printer.request_exit(result);
            }
            Box::pin(async { Ok(()) })
        }
    }

    #[tokio::test]
    async fn test_a_restart_rebuilds_the_printer_before_the_next_run() {
        // The restart re-reads the file, so the run loop needs one on disk: an
        // empty config, which loads with no sections.
        let config_file = write_temp_config("rebuild", "");

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let connects = Arc::new(AtomicUsize::new(0));
        // Registered before `mark_host_objects`, so it is a host part and a
        // reset keeps it.
        printer
            .add_object(
                "stub",
                Arc::new(RestartOnce {
                    connects: Arc::clone(&connects),
                    printer: Arc::downgrade(&printer),
                }),
            )
            .unwrap();
        printer.mark_host_objects();

        klippy_process(Arc::clone(&printer), config_file.clone()).await;
        let _ = std::fs::remove_file(&config_file);

        // `restart` rebuilt the printer (config reloaded, brought up again)
        // instead of ending the loop; `exit` then ended it.
        assert_eq!(connects.load(Ordering::SeqCst), 2);
        assert_eq!(printer.get_state_message().category, PrinterState::Ready);
    }

    /// A host part that edits the config on disk and asks for a restart the
    /// first time it connects, and records what `configfile` reports before
    /// asking for an exit the second.
    ///
    /// It is a *host* part (registered before `mark_host_objects`), so a reset
    /// keeps it and it connects again — that second connect is what makes the
    /// re-read observable from inside the run loop.
    struct ReloadingConfigFile {
        config_file: String,
        edited_content: String,
        observed: Arc<Mutex<Vec<Option<Value>>>>,
        printer: Weak<Printer>,
    }

    impl PrinterObject for ReloadingConfigFile {
        fn get_status(&self, _eventtime: f64) -> Value {
            serde_json::json!({})
        }

        fn connect<'a>(&'a self) -> ConnectFuture<'a> {
            if let Some(printer) = self.printer.upgrade() {
                // What `configfile` reports at connect time: the start-up parse
                // on the first round, the file re-read at the restart on the
                // second.
                let seen = printer
                    .lookup_object_as::<PrinterConfig>(CONFIGFILE_OBJECT)
                    .map(|configfile| configfile.get_status(0.0)["config"].clone());
                let first = {
                    let mut observed = self.observed.lock().unwrap_or_else(|p| p.into_inner());
                    observed.push(seen);
                    observed.len() == 1
                };
                if first {
                    // The operator's edit, made while the printer was running.
                    std::fs::write(&self.config_file, &self.edited_content)
                        .expect("the edited config is written");
                    printer.request_exit("restart");
                } else {
                    printer.request_exit("exit");
                }
            }
            Box::pin(async { Ok(()) })
        }
    }

    #[tokio::test]
    async fn test_a_restart_rereads_the_config_file_from_disk() {
        // What `run` does at start-up: parse the file, load it, then hand the
        // *path* to the run loop. The file changes on disk afterwards, and the
        // restart has to pick the new content up — upstream's `_connect` reads
        // the config again for every restart (`klippy/klippy.py:114-121`).
        let config_file = write_temp_config("reread", "[pause_resume]\nrecover_velocity: 50\n");
        let (startup_config, _) = Config::from_file(&config_file).expect("the config parses");

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        let observed = Arc::new(Mutex::new(Vec::new()));
        printer
            .add_object(
                "stub",
                Arc::new(ReloadingConfigFile {
                    config_file: config_file.clone(),
                    edited_content: "[pause_resume]\nrecover_velocity: 120\n".to_string(),
                    observed: Arc::clone(&observed),
                    printer: Arc::downgrade(&printer),
                }),
            )
            .unwrap();
        printer.mark_host_objects();
        printer
            .load_config(&startup_config)
            .expect("the startup config loads");

        klippy_process(Arc::clone(&printer), config_file.clone()).await;
        let _ = std::fs::remove_file(&config_file);

        let observed = observed.lock().unwrap_or_else(|p| p.into_inner());
        assert_eq!(observed.len(), 2, "the host part connected once per run");
        // The first run saw the start-up parse …
        let first = observed[0]
            .clone()
            .expect("`configfile` is registered before the first connect");
        assert_eq!(
            first["pause_resume"]["recover_velocity"],
            serde_json::json!("50")
        );
        // … and the second saw the file as it stood when the restart was
        // served, which is the re-read rather than a reused parse.
        let second = observed[1]
            .clone()
            .expect("`configfile` is registered again by the reload");
        assert_eq!(
            second["pause_resume"]["recover_velocity"],
            serde_json::json!("120")
        );
        assert_eq!(printer.get_state_message().category, PrinterState::Ready);
    }

    /// A host part that breaks the config on disk the first time it connects and
    /// asks for a restart.
    ///
    /// The reload can then only fail, and nothing connects on that round (a
    /// printer in `error` has nothing to bring up), so the exit has to come from
    /// outside the printer's own parts.
    struct BreakingConfigFile {
        config_file: String,
        printer: Weak<Printer>,
    }

    impl PrinterObject for BreakingConfigFile {
        fn get_status(&self, _eventtime: f64) -> Value {
            serde_json::json!({})
        }

        fn connect<'a>(&'a self) -> ConnectFuture<'a> {
            if let Some(printer) = self.printer.upgrade() {
                // A file that no longer parses: the restart must report the
                // parse failure rather than come up with a broken machine.
                std::fs::write(&self.config_file, "this is not a config file\n")
                    .expect("the broken config is written");
                printer.request_exit("restart");
            }
            Box::pin(async { Ok(()) })
        }
    }

    #[tokio::test]
    async fn test_a_restart_whose_config_no_longer_parses_reports_error() {
        let config_file = write_temp_config("broken", "[pause_resume]\nrecover_velocity: 50\n");
        let (startup_config, _) = Config::from_file(&config_file).expect("the config parses");

        let printer = Arc::new(Printer::new(ManualReactor::shared()));
        printer
            .add_object(
                "stub",
                Arc::new(BreakingConfigFile {
                    config_file: config_file.clone(),
                    printer: Arc::downgrade(&printer),
                }),
            )
            .unwrap();
        printer.mark_host_objects();
        printer
            .load_config(&startup_config)
            .expect("the startup config loads");

        // The exit cannot come from the printer's parts on the failed round, so
        // watch the state and ask for it once the reload has been rejected.
        let watcher = {
            let printer = Arc::clone(&printer);
            tokio::spawn(async move {
                while printer.get_state_message().category != PrinterState::Error {
                    tokio::time::sleep(std::time::Duration::from_millis(5)).await;
                }
                printer.request_exit("exit");
            })
        };

        let result = klippy_process(Arc::clone(&printer), config_file.clone()).await;
        watcher.await.unwrap();
        let _ = std::fs::remove_file(&config_file);

        // The loop kept turning — the restart delay, then another run — until it
        // was asked to stop, rather than ending the process on the bad config.
        assert_eq!(result, "exit");
        let state = printer.get_state_message();
        assert_eq!(state.category, PrinterState::Error);
        assert!(
            state.message.contains("Parameter outside of section"),
            "the state message is the parse failure: {}",
            state.message
        );
    }

    #[test]
    fn test_a_transport_captures_the_machine_runtime_not_the_ambient_one() {
        // A3's mechanism: `Interface` stores whatever runtime is current when it
        // is opened, and the loader opens transports under the *machine* handle
        // (`run` enters it around `load_config`). Opened inside that context, a
        // `test:` interface must carry the machine runtime, not the API one.
        use crate::core::klippy::interface::devices::frame_mock::FrameMock;
        use crate::core::klippy::interface::Interface;

        let machine = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(1)
            .enable_all()
            .build()
            .unwrap();
        let api = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(1)
            .enable_all()
            .build()
            .unwrap();
        let machine_handle = machine.handle().clone();

        api.block_on(async {
            let interface = {
                let _machine = machine_handle.enter();
                Interface::new(FrameMock::new(vec![]))
            };
            assert_eq!(interface.handle().id(), machine_handle.id());
            assert_ne!(
                interface.handle().id(),
                tokio::runtime::Handle::current().id(),
                "the transport must not carry the ambient (API) runtime"
            );
        });
    }
}
