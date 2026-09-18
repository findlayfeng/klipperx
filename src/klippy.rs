use clap::Parser;
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use tracing::{debug, info, warn};

use crate::core::klippy::api::{AddressError, Api, ApiTarget, Server};
use crate::core::klippy::config::Config;

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
        let server = match target {
            None => {
                info!("Empty --api-server: not starting the API server");
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

        match attachment {
            // Whatever is attached to this host is the user interface of the
            // invocation that asked for it, so when it is done the host is too —
            // and the host's own shutdown conditions (a signal, a config error)
            // end the attachment instead. Waiting on one and then stopping the
            // other is what keeps those two from disagreeing.
            Some(mut attachment) => {
                let api = Arc::new(Api::new());
                let host = tokio::spawn(klippy_process(config));
                let outcome = attachment.run(api).await;
                host.abort();
                outcome.map_err(|err| -> Box<dyn std::error::Error> { err.into() })?;
            }
            None => klippy_process(config).await,
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
