use klipperx::klippy;
use klippy_client as client;

use clap::Parser;
use tracing::error;

/// The command line.
///
/// Running the host is the default: `klipperx printer.cfg` means the same thing
/// as `klipperx klippy printer.cfg`, because that is what this program is for and
/// what an operator types. The host's own arguments are therefore declared here
/// as well as under `klippy`, and `args_conflicts_with_subcommands` keeps the two
/// spellings from being mixed: either the subcommand, or the arguments, never
/// both.
///
/// The flattened group is an `Option` on purpose. Clap only stops demanding the
/// group's required arguments — the config file — when the group itself is
/// optional; without that, `klipperx api …` would insist on a config file it has
/// no use for. `arg_required_else_help` then covers the bare `klipperx`, which
/// otherwise parses as "nothing at all was asked for".
#[derive(Parser, Debug)]
#[command(
    name = "klipperx",
    version,
    about,
    long_about = None,
    args_conflicts_with_subcommands = true,
    arg_required_else_help = true
)]
struct Cli {
    /// Enable verbose output
    #[arg(short, long, global = true)]
    verbose: bool,

    #[command(subcommand)]
    command: Option<Commands>,

    /// The host, when no subcommand is given
    #[command(flatten)]
    host: Option<klipperx::klippy::AppArgs>,
}

#[derive(clap::Subcommand, Debug)]
enum Commands {
    /// Run the host: load the config and serve the API
    //
    // `arg_required_else_help` makes `klipperx klippy` on its own print this
    // help instead of complaining about the config file it was not given:
    // someone who types that is asking what the options are. Any other argument
    // — `klipperx klippy -a /tmp/x` — still gets the complaint, because then they
    // *were* trying to run something.
    #[command(name = "klippy", arg_required_else_help = true)]
    Klippy(klipperx::klippy::AppArgs),

    /// Send one API request and print the reply
    Api(client::ApiArgs),

    /// Connect to the API and enter an interactive session
    Console(client::ConsoleArgs),
}

fn main() {
    let cli: Cli = Cli::parse();

    klipperx::logging::init(cli.verbose);

    let result = match (cli.command, cli.host) {
        (Some(Commands::Klippy(args)), _) => klippy::run(args),
        (Some(Commands::Api(args)), _) => client::run_api(args),
        (Some(Commands::Console(args)), _) => client::run_console(args),
        // No subcommand: the arguments were the host's all along.
        (None, Some(host)) => klippy::run(host),
        // Unreachable as things stand — clap rejects a bare `klipperx` (help)
        // and any flag without a config file — but the match has to say
        // something, and "a config file is required" is what is true.
        (None, None) => Err("a config file is required: try 'klipperx <CONFIG_FILE>'".into()),
    };
    if let Err(e) = result {
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

    /// Parse a command line the way the shell would hand it over.
    fn parse(args: &[&str]) -> Result<Cli, clap::Error> {
        Cli::try_parse_from(std::iter::once("klipperx").chain(args.iter().copied()))
    }

    /// The host arguments of a command line that named no subcommand.
    fn host_of(cli: Cli) -> klipperx::klippy::AppArgs {
        assert!(cli.command.is_none(), "no subcommand was named");
        cli.host.expect("the host arguments were given")
    }

    #[test]
    fn test_the_host_is_what_runs_when_no_subcommand_is_given() {
        let host = host_of(parse(&["printer.cfg"]).expect("a config file is enough"));
        assert_eq!(host.config_file, "printer.cfg");
        assert!(!host.tui);
    }

    #[test]
    fn test_options_travel_with_the_default_subcommand() {
        // The host's own options are the top level's too, in any order, and the
        // global flag still comes first.
        let cli = parse(&["-v", "printer.cfg", "--tui", "-a", "/tmp/x"]).unwrap();
        assert!(cli.verbose);
        let host = host_of(cli);
        assert!(host.tui);
        assert_eq!(host.config_file, "printer.cfg");
        assert_eq!(host.api_server.as_deref(), Some("/tmp/x"));
    }

    #[test]
    fn test_the_explicit_spelling_still_works() {
        // Both spellings have to keep working: the documented one is in every
        // example, and the short one is what an operator types.
        let cli = parse(&["klippy", "printer.cfg", "-a", "/tmp/x"]).unwrap();
        match cli.command {
            Some(Commands::Klippy(args)) => {
                assert_eq!(args.config_file, "printer.cfg");
                assert_eq!(args.api_server.as_deref(), Some("/tmp/x"));
            }
            other => panic!("expected the host, got {other:?}"),
        }
    }

    #[test]
    fn test_the_client_subcommands_are_unaffected() {
        let cli = parse(&["api", "-a", "/tmp/x", "list_endpoints"]).unwrap();
        match cli.command {
            Some(Commands::Api(args)) => {
                assert_eq!(args.method, "list_endpoints");
                assert_eq!(args.server.api_server, "/tmp/x");
            }
            other => panic!("expected `api`, got {other:?}"),
        }

        let cli = parse(&["console", "-a", "/tmp/x", "--plain"]).unwrap();
        match cli.command {
            Some(Commands::Console(args)) => {
                assert!(args.plain);
                assert_eq!(args.server.api_server, "/tmp/x");
            }
            other => panic!("expected `console`, got {other:?}"),
        }
    }

    #[test]
    fn test_a_bare_invocation_asks_for_a_config_file() {
        // Not a silent no-op: clap prints the help, whose usage line names what
        // the default subcommand needs.
        let error = parse(&[]).expect_err("nothing at all was asked for");
        let message = error.to_string();
        assert!(message.contains("CONFIG_FILE"), "{message}");

        // A flag and nothing else counts as using the host's arguments, so clap
        // demands the config file for that too.
        let error = parse(&["-v"]).expect_err("a flag is not a config file");
        assert!(error.to_string().contains("CONFIG_FILE"), "{error}");
    }

    #[test]
    fn test_the_host_subcommand_on_its_own_prints_its_help() {
        // Not the missing-argument error: whoever types this is asking what the
        // options are, so answering with the help is the answer.
        let error = parse(&["klippy"]).expect_err("nothing was asked for");
        assert_eq!(
            error.kind(),
            clap::error::ErrorKind::DisplayHelpOnMissingArgumentOrSubcommand
        );
        let help = error.to_string();
        assert!(help.contains("Usage: klipperx klippy"), "{help}");
        assert!(help.contains("--api-server"), "{help}");

        // With an argument it is a real attempt at running the host, so a
        // missing config file is worth complaining about.
        let error = parse(&["klippy", "--tui"]).expect_err("no config file");
        assert_eq!(
            error.kind(),
            clap::error::ErrorKind::MissingRequiredArgument
        );
        assert!(error.to_string().contains("CONFIG_FILE"), "{error}");
    }

    #[test]
    fn test_the_host_args_cannot_be_mixed_with_a_subcommand() {
        // `klipperx printer.cfg api …` is a mistake worth naming rather than
        // guessing at.
        assert!(parse(&["printer.cfg", "api", "-a", "/tmp/x"]).is_err());
    }
}
