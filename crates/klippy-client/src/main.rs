//! The standalone API client.
//!
//! The same two commands the `klipperx` binary carries — `api` for one request
//! and `console` for a session — but as a binary of their own, for the machine
//! that only needs to talk to a printer: no config file, no MCU, no host.
//!
//! It is named after the host it talks to, `klippy`, rather than after the crate
//! it happens to live in, and its help text is spelled out rather than taken
//! from the package description.
//!
//! ```console
//! $ klippy-client api -a tcp:printer.local:7125 info
//! $ klippy-client console -a /run/klipper/api
//! ```

use clap::Parser;
use tracing::{debug, error};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};

use klippy_client::{ApiArgs, ConsoleArgs};

/// Klipper API client
#[derive(Parser, Debug)]
#[command(name = "klippy-client", version)]
struct Cli {
    /// Enable verbose output
    #[arg(short, long, global = true)]
    verbose: bool,

    #[command(subcommand)]
    command: Command,
}

#[derive(clap::Subcommand, Debug)]
enum Command {
    /// Send one API request and print the reply
    Api(ApiArgs),

    /// Connect to the API and enter an interactive session
    Console(ConsoleArgs),
}

/// Install the global tracing subscriber.
///
/// The host has the same twenty lines in `klipperx`'s `logging` module, and they
/// are not shared: the whole point of this crate is that it does not depend on
/// the host, and a crate of its own just for a log filter would be worse than
/// the duplication.
fn init_logging(verbose: bool) {
    let filter = if verbose {
        EnvFilter::try_new("debug").expect("'debug' is a valid filter")
    } else {
        EnvFilter::try_from_default_env()
            .or_else(|_| EnvFilter::try_new("info"))
            .expect("'info' is a valid filter")
    };

    tracing_subscriber::registry()
        .with(filter)
        .with(tracing_subscriber::fmt::layer().with_target(false))
        .try_init()
        .ok();

    debug!("Debug mode enabled");
}

fn main() {
    let cli = Cli::parse();
    init_logging(cli.verbose);

    let result = match cli.command {
        Command::Api(args) => klippy_client::run_api(args),
        Command::Console(args) => klippy_client::run_console(args),
    };
    if let Err(err) = result {
        error!("Error: {err}");
        std::process::exit(1);
    }
}
