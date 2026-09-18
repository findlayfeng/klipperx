use klipperx::klippy;
use klippy_client as client;

use clap::Parser;
use tracing::error;

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

    klipperx::logging::init(cli.verbose);

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
