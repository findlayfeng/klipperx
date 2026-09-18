use klipperx::{client, klippy};

use clap::Parser;
use tracing::{debug, error};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};

#[derive(Parser, Debug)]
#[command(name = "klipperx", version, about, long_about = None)]
struct Cli {
    /// Enable verbose output
    #[arg(short, long, global = true)]
    verbose: bool,

    #[command(subcommand)]
    command: Option<Commands>,
}

#[derive(clap::Subcommand, Debug)]
enum Commands {
    /// Run the host: load the config and serve the API
    #[command(name = "klippy")]
    Klippy(klipperx::klippy::AppArgs),

    /// Send one API request and print the reply
    Api(client::ApiArgs),

    /// Connect to the API and enter an interactive session
    Console(client::ConsoleArgs),
}

fn main() {
    let cli: Cli = Cli::parse();

    // Build initial EnvFilter: prefer RUST_LOG env var, fall back to "info",
    // but allow --verbose to override to "debug".
    let initial_filter = if cli.verbose {
        EnvFilter::try_new("debug").expect("Failed to create debug filter")
    } else {
        EnvFilter::try_from_default_env()
            .or_else(|_| EnvFilter::try_new("info"))
            .expect("Failed to create log filter")
    };

    tracing_subscriber::registry()
        .with(initial_filter)
        .with(tracing_subscriber::fmt::layer().with_target(false))
        .try_init()
        .ok();

    debug!("Debug mode enabled");

    let result = match cli.command {
        Some(Commands::Klippy(args)) => klippy::run(args),
        Some(Commands::Api(args)) => client::run_api(args),
        Some(Commands::Console(args)) => client::run_console(args),
        None => Ok(()),
    };
    if let Err(e) = result {
        error!("Error: {}", e);
        std::process::exit(1);
    }
}
