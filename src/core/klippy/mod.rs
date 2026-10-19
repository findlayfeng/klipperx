// Klippy module - abstract interface for Klipper device communication
//
// This module provides:
// - Interface enum: abstract interface for printer communication
// - SerialDevice: a real MCU on a serial port (see interface/serial.rs)
// - HostDevice: real implementation running klipper's host library, loaded at
//   runtime (see interface/host.rs)

pub mod assign;
pub mod cmd;
pub mod config;
pub mod error;
pub mod event;
pub mod extras;
pub mod frame;
pub mod gcode;
pub mod identify;
pub mod load;
pub mod mathutil;
pub mod mcu;
pub mod motion;
pub mod msg;
pub mod pins;
pub mod printer;
pub mod reactor;
// pub mod toolhead;
#[cfg(test)]
pub(crate) mod test_util;

// Upstream's host test corpus, used as fixtures. Test-only: it reads
// third_party/klipper and exists to check our behavior against upstream's
// shipped configs and test cases.
#[cfg(test)]
mod upstream;

// Client-facing API over the Unix Domain Socket
pub mod api;

// Dynamic library loading interface
pub mod interface;

// Re-export common types for convenience
pub use error::KlippyError;
pub use event::KlippyEvent;
pub use gcode::{
    CommandError, CommandHandler, GCodeDispatch, GcodeCommand, OutputHandler, GCODE_OBJECT,
};
pub use interface::{HostDevice, Interface, SerialDevice};
pub use msg::proto::Payload;
pub use msg::Msg;
pub use pins::{PinError, PinParams, PinResolver, PinType, PrinterPins, PINS_OBJECT};
pub use printer::{Printer, PrinterState, StateMessage};
pub use reactor::{
    CallbackRun, LatencyCallback, LatencyReport, ManualReactor, Reactor, TimerCallback,
    TimerHandle, TokioReactor,
};
// pub use interface::canbus::CanbusInterface;
// pub use mcu::{MCU, McuError, McuPin, PinParams};
// pub use mcu::{add_printer_objects, get_printer_mcu};
// pub use msg::proto::{ArgType, MsgParams, MsgValue, crc32, msg_params};

/// Whether opt-in diagnostic tracing is enabled (`KLIPPERX_TRACE=1`).
///
/// The simulator and MCU teardown print trace lines (`SIM-DIAG: …`,
/// `MCU DROP …`) that are useful when chasing a hang but are pure noise in an
/// ordinary test run, so they are off unless the environment opts in.
pub(crate) fn trace_enabled() -> bool {
    static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ENABLED.get_or_init(|| {
        std::env::var("KLIPPERX_TRACE")
            .map(|value| value != "0")
            .unwrap_or(false)
    })
}

/// Wait `delay` without holding the `tokio` worker the caller runs on.
///
/// The pausing paths that use this are synchronous by contract (upstream pauses
/// a greenlet there, and these are plain `Fn` callbacks and blocking helpers), so
/// the wait cannot become an `.await` without turning whole call chains `async`.
/// Instead, on a multi-threaded runtime the caller's worker hands its core off to
/// a new thread for the duration of the sleep ([`tokio::task::block_in_place`]),
/// so the reactor keeps dispatching while the wait runs; off a runtime, or on a
/// current-thread one, this is the plain [`std::thread::sleep`].
///
/// [`tokio::task::block_in_place`] **panics** on a current-thread runtime (there
/// is no other worker to hand off to), hence the flavor check: the fallback
/// parks the only thread, which is what that runtime asked for.
pub(crate) fn sleep_off_worker(delay: std::time::Duration) {
    let on_multi_thread = tokio::runtime::Handle::try_current()
        .is_ok_and(|handle| handle.runtime_flavor() == tokio::runtime::RuntimeFlavor::MultiThread);
    if on_multi_thread {
        tokio::task::block_in_place(|| std::thread::sleep(delay));
    } else {
        std::thread::sleep(delay);
    }
}

#[cfg(test)]
mod sleep_off_worker_tests {
    use super::sleep_off_worker;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;
    use std::time::Duration;

    /// A wait made through [`sleep_off_worker`] must leave the runtime's other
    /// workers free: a task spawned before it runs while the wait is still
    /// going on.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn sleep_off_worker_yields_the_worker() {
        tokio::time::timeout(Duration::from_secs(5), async {
            let ran = Arc::new(AtomicBool::new(false));
            let setter = {
                let ran = Arc::clone(&ran);
                tokio::spawn(async move { ran.store(true, Ordering::SeqCst) })
            };
            sleep_off_worker(Duration::from_millis(50));
            // The wait below is slack for that task's own scheduling only, so a
            // loaded machine cannot turn this into a fixed-sleep flake; the
            // window the flag has to be set in is the sleep above.
            let deadline = tokio::time::Instant::now() + Duration::from_millis(500);
            while !ran.load(Ordering::SeqCst) && tokio::time::Instant::now() < deadline {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
            assert!(
                ran.load(Ordering::SeqCst),
                "the task spawned before sleep_off_worker did not run during it"
            );
            setter.await.expect("the flag-setting task panicked");
        })
        .await
        .expect("sleep_off_worker must return well within the test's own bound");
    }
}
