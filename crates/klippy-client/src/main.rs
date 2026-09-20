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
use tracing_subscriber::filter::LevelFilter;
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

/// The level `--verbose` turns on.
const VERBOSE_LEVEL: &str = "debug";

/// The level used when neither source asks for one.
const DEFAULT_LEVEL: &str = "info";

/// The filter the client runs with.
///
/// Two sources can ask for a level: `--verbose` and `RUST_LOG`. Neither is
/// ranked above the other, so the **more detailed** of the two wins:
/// `RUST_LOG=trace` survives `--verbose`, and `--verbose` survives
/// `RUST_LOG=warn`. With the flag absent, `RUST_LOG` is used as it is; with
/// neither, the client is `info`.
///
/// A `RUST_LOG` that does not parse is ignored rather than fatal.
fn filter_for(verbose: bool, rust_log: Option<&str>) -> EnvFilter {
    let rust_log = rust_log.and_then(|spec| EnvFilter::try_new(spec).ok());
    match (verbose, rust_log) {
        (false, Some(env)) => env,
        (false, None) => EnvFilter::new(DEFAULT_LEVEL),
        (true, env) => {
            let verbose = EnvFilter::new(VERBOSE_LEVEL);
            match env {
                Some(env) if detail(&env) > detail(&verbose) => env,
                _ => verbose,
            }
        }
    }
}

/// How detailed a filter is: the loudest level it lets through.
fn detail(filter: &EnvFilter) -> LevelFilter {
    filter.max_level_hint().unwrap_or(LevelFilter::TRACE)
}

/// Install the global tracing subscriber.
///
/// The level is the more detailed of what `--verbose` asks for and what
/// `RUST_LOG` asks for; see [`filter_for`].
///
/// The host has the same lines in `klipperx`'s `logging` module, and they are not
/// shared: the whole point of this crate is that it does not depend on the host,
/// and a crate of its own just for a log filter would be worse than the
/// duplication.
fn init_logging(verbose: bool) {
    let filter = filter_for(verbose, std::env::var("RUST_LOG").ok().as_deref());

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

#[cfg(test)]
mod tests {
    use super::*;

    /// `--verbose` and `RUST_LOG` are two ways to ask for a level; neither is
    /// ranked above the other, and the more detailed wins.
    #[test]
    fn test_the_more_detailed_of_flag_and_environment_wins() {
        // `--verbose` is a floor, not a ceiling: a louder RUST_LOG is kept...
        assert_eq!(detail(&filter_for(true, Some("trace"))), LevelFilter::TRACE);
        // ...and a quieter one does not silence the flag.
        assert_eq!(detail(&filter_for(true, Some("warn"))), LevelFilter::DEBUG);
        assert_eq!(detail(&filter_for(true, None)), LevelFilter::DEBUG);
        // Without the flag, RUST_LOG is used as it is — including to quiet down.
        assert_eq!(
            detail(&filter_for(false, Some("trace"))),
            LevelFilter::TRACE
        );
        assert_eq!(detail(&filter_for(false, Some("warn"))), LevelFilter::WARN);
        assert_eq!(detail(&filter_for(false, None)), LevelFilter::INFO);
        // A RUST_LOG that does not parse is ignored, not fatal.
        assert_eq!(
            detail(&filter_for(false, Some("foo=notalevel"))),
            LevelFilter::INFO
        );
    }
}
