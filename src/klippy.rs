use clap::Parser;
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use tracing::{debug, info, warn};

use crate::core::klippy::api::{self, AddressError, Api, ApiTarget, Server, StartArgs};
use crate::core::klippy::config::Config;
use crate::core::klippy::printer::Printer;
use crate::core::klippy::reactor::TokioReactor;

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
}

/// How long to wait before a rebuilt printer is brought up again.
///
/// It gives a device that was just closed (a serial port, a CAN socket) time to
/// come back, and rate-limits a restart that returns straight away — a config
/// the loader rejects, an MCU that is gone.
const RESTART_DELAY: std::time::Duration = std::time::Duration::from_millis(1000);

/// Whether a run result asks for the printer to be built again.
///
/// `"exit"` / `"error_exit"` end the process; these two rebuild the machine
/// (upstream's `klippy/klippy.py:355-370`).
fn is_restart(result: &str) -> bool {
    matches!(result, "restart" | "firmware_restart")
}

/// Run one printer until something asks it to stop, rebuilding it on a restart.
///
/// The machine is built, its config loaded and the API server serving it by the
/// time this is called. The run loop blocks, so it gets a blocking thread of its
/// own; the interrupt means "ask the printer to exit", not "kill the process",
/// so the printer still goes down in order.
///
/// A restart reloads the same parsed config rather than re-reading the file, so
/// an edit on disk takes effect at the next *start*, like upstream. The machine
/// is reset and reloaded **in place**: the same `Arc<Printer>` keeps serving, so
/// the endpoints and any attached window survive a restart (this is the answer
/// to Q7 — no printer slot to swap, because the printer is rebuilt under its
/// one handle).
async fn klippy_process(printer: Arc<Printer>, config: Arc<Config>) {
    // The printer's own shutdown conditions (an MCU going away, a client's
    // emergency stop) end the loop on their own; until they exist, the
    // operator's interrupt is the only one. A task rather than a `select!`, so
    // that the blocking loop below is awaited exactly once — and so that an
    // exit requested from somewhere else (an attached window closing) ends it
    // without waiting for an interrupt. It is a loop because a restart clears
    // the exit request, so the listener has to be armed again for the next run.
    let interrupt_printer = Arc::clone(&printer);
    let interrupt = tokio::spawn(async move {
        loop {
            if let Err(err) = tokio::signal::ctrl_c().await {
                warn!("cannot listen for an interrupt: {err}");
                return;
            }
            interrupt_printer.request_exit("exit");
        }
    });

    loop {
        printer.bring_up().await;

        let result = {
            let printer = Arc::clone(&printer);
            tokio::task::spawn_blocking(move || printer.run()).await
        };

        let result = match result {
            Ok(result) => result,
            Err(err) => {
                warn!("the run loop did not finish: {err}");
                break;
            }
        };

        if !is_restart(&result) {
            debug!("printer stopped: {result}");
            break;
        }

        info!("Restarting the printer ({result})");
        printer.reset_for_restart();
        if let Err(err) = printer.load_config(&config) {
            printer.invoke_shutdown(&format!("{err}"));
        }
        tokio::time::sleep(RESTART_DELAY).await;
    }

    interrupt.abort();
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
pub fn run(
    args: AppArgs,
    attachment: Option<Box<dyn Attachment>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let config_file = args
        .config_file
        .ok_or("the following required argument was not provided: <CONFIG_FILE>")?;
    debug!("Config file: {config_file}");

    // Parse the config file
    let (config, sources) = Config::from_file(&config_file)?;
    debug!("Config sources: {:?}", sources);
    info!(
        "Successfully parsed config with {} sections",
        config.sections_vec().len()
    );
    for section in config.sections() {
        debug!("Section: {}", section.identifier());
    }

    // Shared with the run loop: a restart reloads this same parsed config.
    let config = Arc::new(config);

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

    // One runtime for the whole process, and the only place one is created:
    // every async task in klippy (the MCU send and receive tasks, the API
    // server's accept loop and its per-connection tasks) runs here. It is
    // multi-threaded because endpoint handlers, subscription timers and MCU
    // traffic are independent work: on a single thread one slow endpoint would
    // hold up MCU responses and every other client with them.
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;

    runtime.block_on(async move {
        // The machine, built before anything is served: the endpoint table is
        // registered and then the config's objects are loaded, so that a client
        // never observes a half-built table or a half-built machine. Its reactor
        // is this runtime — timers are tokio tasks, and the clock is tokio's, so
        // nothing here builds a second event loop.
        let printer = Arc::new(Printer::new(Arc::new(TokioReactor::new(
            tokio::runtime::Handle::current(),
        ))));

        // The server's own object (`webhooks`) and the endpoints come first, so
        // that `objects/list` starts with `webhooks` as upstream's does
        // (`klippy/klippy.py:36-40`) and no path is half-built when a request
        // arrives. Everything is registered before the listener is bound.
        let mut api = Api::new();
        api::register(&mut api, &printer, StartArgs::collect(config_file.clone()))?;
        let api = Arc::new(api);

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

        debug!(
            "Klippy process started with {} sections",
            config.sections_vec().len()
        );

        // Config-driven objects. A config the loader rejects halts the printer
        // rather than ending the process: clients can still connect and read
        // why (upstream's `_read_config` does the same — it sets the state and
        // lets the reactor keep running).
        if let Err(err) = printer.load_config(&config) {
            printer.invoke_shutdown(&format!("{err}"));
        }

        match attachment {
            // Whatever is attached to this host is the user interface of the
            // invocation that asked for it, so when it is done the host is too
            // — and the host's own shutdown conditions end the attachment
            // instead. Asking the printer to exit is what lets its run loop
            // return in order, rather than dropping a loop that still holds the
            // machine.
            Some(mut attachment) => {
                let host = tokio::spawn(klippy_process(Arc::clone(&printer), Arc::clone(&config)));
                let outcome = attachment.run(Arc::clone(&api)).await;
                printer.request_exit("exit");
                let _ = host.await;
                outcome.map_err(|err| -> Box<dyn std::error::Error> { err.into() })?;
            }
            None => klippy_process(printer, config).await,
        }

        // The printer has stopped, so the API server goes with it. Aborting is
        // enough: dropping the listener removes the socket file.
        if let Some(handle) = server {
            handle.abort();
            let _ = handle.await;
        }

        Ok::<(), Box<dyn std::error::Error>>(())
    })
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::config::Config;
    use crate::core::klippy::printer::{ConnectFuture, PrinterObject, PrinterState};
    use crate::core::klippy::reactor::ManualReactor;
    use serde_json::Value;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Weak;

    #[test]
    fn test_only_a_restart_result_rebuilds_the_printer() {
        // The two spellings `gcode/restart` and `gcode/firmware_restart` exit
        // with; everything else stops the process.
        assert!(is_restart("restart"));
        assert!(is_restart("firmware_restart"));
        assert!(!is_restart("exit"));
        assert!(!is_restart("error_exit"));
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

        klippy_process(Arc::clone(&printer), Arc::new(Config::new())).await;

        // `restart` rebuilt the printer (config reloaded, brought up again)
        // instead of ending the loop; `exit` then ended it.
        assert_eq!(connects.load(Ordering::SeqCst), 2);
        assert_eq!(printer.get_state_message().category, PrinterState::Ready);
    }
}
