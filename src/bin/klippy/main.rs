use klipperx::klippy;

use clap::Parser;
use tracing::error;

/// The host, as a binary of its own: the same program as `klipperx klippy`,
/// which is why it says the same thing about itself and behaves the same way.
#[derive(Parser, Debug)]
#[command(
    name = "klippy",
    version,
    about = klipperx::klippy::ABOUT,
    arg_required_else_help = true
)]
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

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory as _;

    /// Parse a command line the way the shell would hand it over.
    fn parse(args: &[&str]) -> Result<Cli, clap::Error> {
        Cli::try_parse_from(std::iter::once("klippy").chain(args.iter().copied()))
    }

    #[test]
    fn test_the_host_alone_prints_its_help() {
        // The same as `klipperx klippy`: whoever types this is asking what the
        // options are.
        let error = parse(&[]).expect_err("nothing was asked for");
        assert_eq!(
            error.kind(),
            clap::error::ErrorKind::DisplayHelpOnMissingArgumentOrSubcommand
        );
        assert!(error.to_string().contains("--api-server"), "{error}");
    }

    #[test]
    fn test_a_flag_without_a_config_file_is_an_error() {
        // Then they were trying to run something, and the missing file is worth
        // saying.
        let error = parse(&["--tui"]).expect_err("no config file");
        assert_eq!(
            error.kind(),
            clap::error::ErrorKind::MissingRequiredArgument
        );
        assert!(error.to_string().contains("CONFIG_FILE"), "{error}");
    }

    #[test]
    fn test_the_arguments_are_the_hosts_own() {
        let cli = parse(&["printer.cfg", "--tui", "-a", "/tmp/x"]).unwrap();
        assert!(cli.args.tui);
        assert_eq!(cli.args.config_file, "printer.cfg");
        assert_eq!(cli.args.api_server.as_deref(), Some("/tmp/x"));
    }

    #[test]
    fn test_it_describes_itself_the_way_the_subcommand_does() {
        // Two spellings of one program must not describe themselves differently,
        // which is what `klippy::ABOUT` is for.
        assert_eq!(
            Cli::command().get_about().map(|about| about.to_string()),
            Some(klipperx::klippy::ABOUT.to_string())
        );
    }
}
