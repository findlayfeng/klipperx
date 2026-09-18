//! Logging setup, shared by the binaries.
//!
//! Three binaries — `klipperx`, `klippy` and `klipperx-client` — all want the
//! same subscriber: `RUST_LOG` when it is set, `info` otherwise, and `debug`
//! when `--verbose` was typed. Keeping that in one place is why this module
//! exists; it is an application concern that lives in the library because the
//! library is already where this crate's application entry points live.

use tracing::debug;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};

/// Install the global tracing subscriber.
///
/// `--verbose` wins over `RUST_LOG`, because it is the more explicit of the two.
///
/// Installing twice is not an error: the second attempt is ignored, which is
/// what a user of the library as a library will do.
pub fn init(verbose: bool) {
    let filter = if verbose {
        EnvFilter::try_new("debug").expect("'debug' is a valid filter")
    } else {
        EnvFilter::try_from_default_env()
            .or_else(|_| EnvFilter::try_new("info"))
            .expect("'info' is a valid filter")
    };

    tracing_subscriber::registry()
        .with(filter)
        .with(tracing_subscriber::fmt::layer().with_target(false))
        .try_init()
        .ok();

    debug!("Debug mode enabled");
}
