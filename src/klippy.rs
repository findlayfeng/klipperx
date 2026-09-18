use clap::Parser;
use std::sync::Arc;
use tracing::{debug, info, warn};

use crate::core::klippy::api::{AddressError, Api, ListenTarget, Server};
use crate::core::klippy::config::Config;

/// Klippy CLI application
#[derive(Parser, Debug)]
#[command(name = "klippy", version, about)]
pub struct AppArgs {
    /// API server listen target: a socket path (default), or `tcp:<host:port>`
    ///
    /// Omitted means no API server is started, as in klipper. The socket is
    /// this host's whole external interface and has no authentication, so a TCP
    /// target belongs on a trusted network only.
    #[arg(short, long, value_name = "ADDR")]
    pub api_server: Option<String>,

    /// Input TTY device
    #[arg(long)]
    pub input_tty: Option<String>,

    /// Config file path
    pub config_file: String,
}

/// Klippy process that receives the parsed config
///
/// Until the printer main loop exists this only keeps the process — and with it
/// the API server — alive.
async fn klippy_process(config: Config) {
    // TODO: turn the config into printer objects, connect the MCUs, fire
    // `klippy:ready`, and run until a shutdown is requested.
    debug!(
        "Klippy process started with {} sections",
        config.sections_vec().len()
    );

    // The printer's own shutdown conditions (a config error, an MCU going away)
    // will end this loop; for now the operator's interrupt is the only one.
    if let Err(err) = tokio::signal::ctrl_c().await {
        warn!("cannot listen for an interrupt: {err}");
    }
}

/// Main run function for klippy subcommand
pub fn run(args: AppArgs) -> Result<(), Box<dyn std::error::Error>> {
    if let Some(ref tty) = args.input_tty {
        debug!("Input TTY: {}", tty);
    }
    debug!("Config file: {}", args.config_file);

    // Parse the config file
    let (config, sources) = Config::from_file(&args.config_file)?;
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
    let target = match args.api_server.as_deref() {
        None => None,
        Some(address) => match address.parse::<ListenTarget>() {
            Ok(target) => Some(target),
            Err(err) => {
                // A bad address is an option error, so it is reported the way
                // clap reports one.
                let err: AddressError = err;
                return Err(format!("{err}\n\nFor more information, try '--help'.").into());
            }
        },
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
        let server = match target {
            None => {
                info!("No --api-server given: not starting the API server");
                None
            }
            Some(target) => {
                // Endpoints are registered before the listener is bound, so a
                // client can never observe a half-built table.
                let api = Api::new();
                // TODO: register the remaining endpoints as they are written.
                // `Info` is defined but stays unregistered until its handler
                // exists: a `todo!()` would panic the connection instead of
                // answering it.
                let server = Server::bind(target, Arc::new(api)).await?;
                // `target` is resolved by the bind, so a `tcp:…:0` port is
                // reported as the one the kernel chose.
                info!("API server listening on {}", server.target());
                Some(tokio::spawn(server.run()))
            }
        };

        klippy_process(config).await;

        // The printer has stopped, so the API server goes with it. Aborting is
        // enough: dropping the listener removes the socket file.
        if let Some(handle) = server {
            handle.abort();
            let _ = handle.await;
        }

        Ok::<(), Box<dyn std::error::Error>>(())
    })
}
