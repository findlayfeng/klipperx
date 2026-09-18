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

    if let Err(e) = klippy::run(cli.args, None) {
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
    fn test_a_flag_without_a_config_file_parses_and_is_caught_later() {
        // Clap cannot demand the config file, since the same arguments are the
        // optional half of `klipperx`'s command line; the host says so instead.
        let cli = parse(&["-v"]).expect("a flag on its own parses");
        assert!(cli.args.config_file.is_none());
    }

    #[test]
    fn test_the_window_is_not_this_binary_s_business() {
        // `--tui` belongs to `klipperx`: a host that only serves the API has no
        // use for a window, and not offering it keeps the terminal library out
        // of this binary.
        let error = parse(&["printer.cfg", "--tui"]).expect_err("unknown option");
        assert_eq!(error.kind(), clap::error::ErrorKind::UnknownArgument);
    }

    #[test]
    fn test_the_arguments_are_the_hosts_own() {
        let cli = parse(&["printer.cfg", "-a", "/tmp/x"]).unwrap();
        assert_eq!(cli.args.config_file.as_deref(), Some("printer.cfg"));
        assert_eq!(cli.args.api_server, "/tmp/x");
    }

    #[test]
    fn test_the_api_path_defaults_to_the_shared_one() {
        // The standalone host serves where a client looks by default, so a
        // client needs no arguments either.
        let cli = parse(&["printer.cfg"]).unwrap();
        assert_eq!(cli.args.api_server, klippy_api::address::DEFAULT_API_SERVER);
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
