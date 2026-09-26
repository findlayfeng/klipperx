use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;

use clap::{Args, Parser};
use tracing::{error, warn};

use klipperx::{klippy, logging};
use klippy_client as client;
use klippy_client::session::{Entry, LogLevel};

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
    //
    // Flattened without an `Option`, unlike the usual "default subcommand"
    // recipe: the host's arguments cannot be a group of their own, because they
    // arrive through two levels of flatten and clap cannot see the inner ones
    // when it decides whether such a group was used. Nothing needs to be
    // optional for that to work here — the config file is checked when the host
    // starts, not when the command line is parsed.
    #[command(flatten)]
    host: HostArgs,
}

/// The host, plus the one option that is `klipperx`'s own.
//
// The window is not the host's: it is a client, and it drags in a terminal
// library the `klippy` binary — which only ever serves the API — has no use for.
// So it is declared here, where the window can be, rather than in
// `klippy::AppArgs`, where the host's own options live.
#[derive(Args, Debug)]
struct HostArgs {
    #[command(flatten)]
    host: klippy::AppArgs,

    /// Open a local client window on this host's own API
    ///
    /// Runs a `klippy-client` window against this process, over an in-process
    /// pipe rather than the socket — so it needs no `-a`, and the host's own log
    /// lines appear in the window alongside the API traffic that produced them.
    /// Leaving the window stops the host. Without a terminal this is a warning
    /// and the host runs headless.
    #[arg(long)]
    tui: bool,
}

#[derive(clap::Subcommand, Debug)]
enum Commands {
    // The about comes from `klippy::ABOUT` rather than a doc comment, so that
    // this subcommand and the `klippy` binary cannot describe themselves
    // differently.
    //
    // `arg_required_else_help` makes `klipperx klippy` on its own print this
    // help instead of complaining about the config file it was not given:
    // someone who types that is asking what the options are. Any other argument
    // — `klipperx klippy -a /tmp/x` — still gets the complaint, because then they
    // *were* trying to run something.
    #[command(
        name = "klippy",
        about = klipperx::klippy::ABOUT,
        arg_required_else_help = true
    )]
    Klippy(HostArgs),

    /// Send one API request and print the reply
    //
    // `arg_required_else_help`: naming a subcommand and saying nothing else is a
    // question about it, so it is answered with its help — the same courtesy
    // `klippy` gets, and the same as `klipperx` on its own.
    #[command(arg_required_else_help = true)]
    Api(client::ApiArgs),

    /// Connect to the API and enter an interactive session
    //
    // No `arg_required_else_help`, unlike `api`: this subcommand's arguments all
    // have defaults, so `klipperx console` on its own is not a question but an
    // instruction — connect to the shared API path and open a window. `api`
    // keeps the attribute because it is genuinely missing its method.
    Console(client::ConsoleArgs),

    /// Stress-test one MCU until it errors: step generation or command load
    //
    // A bench tool, not part of the host: it takes the named MCU over and ramps
    // `queue_step` load (or `get_clock` request rate) until the firmware shuts
    // down (or the link gives out). `arg_required_else_help` because the
    // config file is required; the MCU names default to the bare `[mcu]`.
    #[command(arg_required_else_help = true)]
    Stress(klipperx::stress::StressArgs),
}

fn main() {
    let cli: Cli = Cli::parse();

    klipperx::logging::init(cli.verbose, cli.log_file());

    let result: Result<i32, Box<dyn std::error::Error>> = match (cli.command, cli.host) {
        (Some(Commands::Klippy(args)), _) => run_host(args),
        (Some(Commands::Api(args)), _) => client::run_api(args).map(|()| 0),
        (Some(Commands::Console(args)), _) => client::run_console(args).map(|()| 0),
        (Some(Commands::Stress(args)), _) => klipperx::stress::run(args).map(|()| 0),
        // No subcommand: the arguments were the host's all along.
        (None, host) => run_host(host),
    };
    match result {
        // The host decides its own exit code (`error_exit` is non-zero).
        Ok(code) => std::process::exit(code),
        Err(e) => {
            error!("Error: {}", e);
            std::process::exit(1);
        }
    }
}

impl Cli {
    /// The `--logfile` this invocation was given, whichever spelling of the
    /// host was used.
    fn log_file(&self) -> Option<&std::path::Path> {
        let path = match &self.command {
            Some(Commands::Klippy(args)) => args.host.log_file.as_deref(),
            _ => self.host.host.log_file.as_deref(),
        };
        path.map(std::path::Path::new)
    }
}

/// Run the host, with a window on it if one was asked for.
fn run_host(args: HostArgs) -> Result<i32, Box<dyn std::error::Error>> {
    let windowed = args.tui && client::tui::is_available();
    if args.tui && !windowed {
        warn!("--tui needs a terminal on stdin and stdout; running without a window");
    }
    if !windowed {
        return klippy::run(args.host, None);
    }

    // The window shows the host's own log lines, so they are copied into it
    // rather than written underneath it. Deciding that here, before the host
    // starts, is what gets the host's first lines into the window too; the guard
    // that keeps the redirect lives in the window and falls with it.
    let (records, logs) = tokio::sync::mpsc::unbounded_channel();
    let window = Window {
        _redirect: logging::to_window(records),
        logs: Some(logs),
    };
    klippy::run(args.host, Some(Box::new(window)))
}

/// A `klippy-client` window on the host's own API.
struct Window {
    /// Keeps the host's records going to the window rather than to stdout.
    _redirect: logging::WindowGuard,
    /// The host's records, until the window takes them.
    logs: Option<tokio::sync::mpsc::UnboundedReceiver<logging::Record>>,
}

impl klippy::Attachment for Window {
    fn run<'a>(
        &'a mut self,
        api: Arc<klippy_api::Api>,
    ) -> Pin<Box<dyn Future<Output = Result<(), String>> + 'a>> {
        Box::pin(async move {
            // The host serves one end of an in-process pipe and the client talks
            // on the other: no socket, no `-a`, and the protocol in between is
            // the same as it would be over either.
            let (host_side, client_side) = tokio::io::duplex(64 * 1024);
            tokio::spawn(klippy_api::server::serve(
                klippy_api::ClientConnection::new(api),
                Box::new(host_side),
            ));
            let session =
                client::Session::from_transport(Box::new(client_side), "this host (in-process)");
            let logs = self.logs.take().map(host_log);
            client::tui::run_session(session, logs)
                .await
                .map_err(|err| err.to_string())
        })
    }
}

/// Join the host's records to the window's entries.
///
/// The two sides have a level type each — the price of the host not depending on
/// a client — so they meet here, where both are in scope. Whatever the host has
/// already logged is moved across first and without waiting: it happened before
/// the window existed, and the window shows it first.
fn host_log(
    mut records: tokio::sync::mpsc::UnboundedReceiver<logging::Record>,
) -> tokio::sync::mpsc::UnboundedReceiver<Entry> {
    let (entries, window) = tokio::sync::mpsc::unbounded_channel();
    while let Ok(record) = records.try_recv() {
        let _ = entries.send(to_entry(record));
    }
    tokio::spawn(async move {
        while let Some(record) = records.recv().await {
            if entries.send(to_entry(record)).is_err() {
                break; // the window is gone
            }
        }
    });
    window
}

/// One host record, as the window's vocabulary has it.
fn to_entry((level, text): logging::Record) -> Entry {
    let level = match level {
        logging::Level::Trace => LogLevel::Trace,
        logging::Level::Debug => LogLevel::Debug,
        logging::Level::Info => LogLevel::Info,
        logging::Level::Warn => LogLevel::Warn,
        logging::Level::Error => LogLevel::Error,
    };
    Entry::Log { level, text }
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
    fn host_of(cli: Cli) -> HostArgs {
        assert!(cli.command.is_none(), "no subcommand was named");
        cli.host
    }

    #[test]
    fn test_the_host_is_what_runs_when_no_subcommand_is_given() {
        let host = host_of(parse(&["printer.cfg"]).expect("a config file is enough"));
        assert_eq!(host.host.config_file.as_deref(), Some("printer.cfg"));
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
        assert_eq!(host.host.config_file.as_deref(), Some("printer.cfg"));
        assert_eq!(host.host.api_server, "/tmp/x");
    }

    #[test]
    fn test_the_explicit_spelling_still_works() {
        // Both spellings have to keep working: the documented one is in every
        // example, and the short one is what an operator types.
        let cli = parse(&["klippy", "printer.cfg", "-a", "/tmp/x"]).unwrap();
        match cli.command {
            Some(Commands::Klippy(args)) => {
                assert_eq!(args.host.config_file.as_deref(), Some("printer.cfg"));
                assert_eq!(args.host.api_server, "/tmp/x");
                assert!(!args.tui, "--tui was not asked for");
            }
            other => panic!("expected the host, got {other:?}"),
        }
    }

    #[test]
    fn test_a_subcommand_alone_is_answered_with_its_help() {
        // `klippy` and `api` are missing something without arguments — a config
        // file, a method — so naming them and saying nothing else is a question,
        // and the answer is the help.
        for name in ["klippy", "api"] {
            let error = parse(&[name]).expect_err("nothing was asked for");
            assert_eq!(
                error.kind(),
                clap::error::ErrorKind::DisplayHelpOnMissingArgumentOrSubcommand,
                "{name}"
            );
            let help = error.to_string();
            assert!(help.contains(&format!("klipperx {name}")), "{help}");
        }

        // `console` is not: every one of its arguments has a default, so on its
        // own it means "connect to the shared API path and open a window".
        let cli = parse(&["console"]).expect("console needs no arguments");
        match cli.command {
            Some(Commands::Console(args)) => assert!(
                !args.plain && args.server.api_server == klippy_api::address::DEFAULT_API_SERVER
            ),
            other => panic!("expected `console`, got {other:?}"),
        }
    }

    #[test]
    fn test_the_api_path_has_one_default_for_everyone() {
        // No `-a` anywhere: the host serves on the shared path, and a client
        // connects to it. That is the whole point of the default.
        let host = host_of(parse(&["printer.cfg"]).unwrap());
        assert_eq!(
            host.host.api_server,
            klippy_api::address::DEFAULT_API_SERVER
        );

        let client = parse(&["api", "list_endpoints"]).unwrap();
        match client.command {
            Some(Commands::Api(args)) => {
                assert_eq!(
                    args.server.api_server,
                    klippy_api::address::DEFAULT_API_SERVER
                );
            }
            other => panic!("expected `api`, got {other:?}"),
        }

        let console = parse(&["console"]).unwrap();
        match console.command {
            Some(Commands::Console(args)) => {
                assert_eq!(
                    args.server.api_server,
                    klippy_api::address::DEFAULT_API_SERVER
                );
            }
            other => panic!("expected `console`, got {other:?}"),
        }
    }

    #[test]
    fn test_an_empty_api_path_is_how_a_host_declines_to_serve() {
        // A host may say "no API"; the empty value is how, and it survives to
        // the host, which is the only thing that can tell which side it is.
        let host = host_of(parse(&["printer.cfg", "-a", ""]).unwrap());
        assert_eq!(host.host.api_server, "");
    }

    #[test]
    fn test_the_window_option_is_the_cli_s_own() {
        // `--tui` is not part of the host: the `klippy` binary does not offer it.
        // Here it is accepted by both spellings of the host.
        let named = parse(&["klippy", "printer.cfg", "--tui"]).unwrap();
        match named.command {
            Some(Commands::Klippy(args)) => assert!(args.tui),
            other => panic!("expected the host, got {other:?}"),
        }
        let default = host_of(parse(&["printer.cfg", "--tui"]).unwrap());
        assert!(default.tui);
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
    fn test_the_log_file_option_is_the_hosts_own_in_both_spellings() {
        // `--logfile` belongs to the host, and the CLI has to find it whichever
        // way the host was spelled: the log is opened before the host starts.
        let host = parse(&["printer.cfg", "--logfile", "/tmp/k.log"]).expect("parses");
        assert_eq!(host.log_file(), Some(std::path::Path::new("/tmp/k.log")));

        let named = parse(&["klippy", "printer.cfg", "--logfile", "/tmp/k.log"]).expect("parses");
        assert_eq!(named.log_file(), Some(std::path::Path::new("/tmp/k.log")));

        // Without the option there is no file, which `info` reports as null.
        assert_eq!(parse(&["printer.cfg"]).unwrap().log_file(), None);
    }

    #[test]
    fn test_a_bare_invocation_prints_the_help() {
        let error = parse(&[]).expect_err("nothing at all was asked for");
        assert_eq!(
            error.kind(),
            clap::error::ErrorKind::DisplayHelpOnMissingArgumentOrSubcommand
        );
        assert!(error.to_string().contains("CONFIG_FILE"), "{error}");
    }

    #[test]
    fn test_a_flag_without_a_config_file_parses_and_is_caught_later() {
        // Clap cannot demand the config file — it would demand it of
        // `klipperx api …` too — so the host is the one that says so.
        let host = host_of(parse(&["-v"]).expect("a flag on its own parses"));
        assert!(host.host.config_file.is_none());
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
        // missing config file is worth complaining about — by the host, which is
        // the only place that can tell a host invocation from a client one.
        let cli = parse(&["klippy", "--tui"]).expect("a flag on its own parses");
        match cli.command {
            Some(Commands::Klippy(args)) => {
                assert!(args.tui);
                assert!(args.host.config_file.is_none());
            }
            other => panic!("expected the host, got {other:?}"),
        }
    }

    #[test]
    fn test_the_host_args_cannot_be_mixed_with_a_subcommand() {
        // `klipperx printer.cfg api …` is a mistake worth naming rather than
        // guessing at.
        assert!(parse(&["printer.cfg", "api", "-a", "/tmp/x"]).is_err());
    }
}
