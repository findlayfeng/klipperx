use clap::Parser;
use klipperx::flash::cli::{self, FlashArgs};

/// The flashing tool, as a binary of its own: one direct link, one firmware
/// file, one katapult session. The link is the arguments' business — a
/// serial tty/USB path or a SocketCAN bus — and a bare `flash` is answered
/// with the help, the same way `klippy` answers it.
fn main() {
    let args = FlashArgs::parse();
    if let Err(e) = cli::run(args) {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}
