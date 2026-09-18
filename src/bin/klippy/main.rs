use klipperx::klippy;

use clap::Parser;
use tracing::error;

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

    klipperx::logging::init(cli.verbose);

    if let Err(e) = klippy::run(cli.args) {
        error!("Error: {}", e);
        std::process::exit(1);
    }
}
