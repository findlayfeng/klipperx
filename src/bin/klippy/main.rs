use klipperx::klippy;

use clap::Parser;
use tracing::{debug, error};
use tracing_subscriber::{
    layer::SubscriberExt, util::SubscriberInitExt, EnvFilter,
};

#[derive(Parser, Debug)]
#[command(name = "klippy", version, about)]
struct Cli {
    /// Enable verbose output
    #[arg(short, long)]
    verbose: bool,

    #[command(flatten)]
    args: klipperx::klippy::AppArgs,
}

fn main() {
    let cli = Cli::parse();

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

    if let Err(e) = klippy::run(cli.args) {
        error!("Error: {}", e);
        std::process::exit(1);
    }
}
