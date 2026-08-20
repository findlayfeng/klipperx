use klipperx::klippy;

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
    /// Klippy subcommand
    #[command(name = "klippy")]
    Klippy(klipperx::klippy::AppArgs),
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

    if let Some(Commands::Klippy(args)) = cli.command {
        if let Err(e) = klippy::run(args) {
            error!("Error: {}", e);
            std::process::exit(1);
        }
    }
}
