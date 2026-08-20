use clap::Parser;
use tracing::{debug, info};

use crate::core::klippy::config::Config;

/// Klippy CLI application
#[derive(Parser, Debug)]
#[command(name = "klippy", version, about)]
pub struct AppArgs {
    /// Klipper API server address
    #[arg(short, long, default_value = "http://127.0.0.1:7125")]
    pub apiserver: String,

    /// Input TTY device
    #[arg(long)]
    pub input_tty: Option<String>,

    /// Config file path
    pub config_file: String,
}

/// Klippy process that receives the parsed config
fn klippy_process(config: Config) {
    // TODO: Implement klippy process logic here
    debug!(
        "Klippy process started with {} sections",
        config.sections_vec().len()
    );
    // Wait for shutdown signal...
}

/// Main run function for klippy subcommand
pub fn run(args: AppArgs) -> Result<(), Box<dyn std::error::Error>> {
    info!("Klipper API server at: {}", args.apiserver);

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

    // Spawn klippy process with the decoded config
    let handle: std::thread::JoinHandle<()> = std::thread::spawn(move || {
        klippy_process(config);
    });

    // Wait for all child processes to finish
    handle.join().map_err(|_| "klippy process panicked")?;

    Ok(())
}
