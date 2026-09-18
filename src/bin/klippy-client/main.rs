//! The standalone API client.
//!
//! The same two commands the `klipperx` binary carries — `api` for one request
//! and `console` for a session — but as a binary of their own, for the machine
//! that only needs to talk to a printer: no config file, no MCU, no host.
//!
//! It is named after the host it talks to ([`klippy`](../klippy/main.rs)) rather
//! than after the package, and its help text is spelled out rather than taken
//! from the package description, which describes that host.
//!
//! ```console
//! $ klippy-client api -a tcp:printer.local:7125 info
//! $ klippy-client console -a /run/klipper/api
//! ```
//!
//! This mirrors `klippy`, which is the same arrangement for the host: a
//! standalone binary that wraps one subcommand of `klipperx`. What it does not
//! change is what gets built — a `[[bin]]` target in this package still links
//! this package's library, so the host's dependencies are compiled and
//! available; only the command line is client-only. A client that did not drag
//! the host along would have to live in a crate of its own, with the API's
//! protocol and address layers extracted to a shared one.

use clap::Parser;
use tracing::error;

use klipperx::client;

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
    Api(client::ApiArgs),

    /// Connect to the API and enter an interactive session
    Console(client::ConsoleArgs),
}

fn main() {
    let cli = Cli::parse();
    klipperx::logging::init(cli.verbose);

    let result = match cli.command {
        Command::Api(args) => client::run_api(args),
        Command::Console(args) => client::run_console(args),
    };
    if let Err(err) = result {
        error!("Error: {err}");
        std::process::exit(1);
    }
}
