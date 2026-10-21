//! `printer_stats` — the once-a-second `Stats` line (upstream's `PrinterStats`,
//! `klippy/extras/statistics.py:52-72`).
//!
//! Every part that answers [`PrinterObject::stats`] is collected once, when the
//! machine becomes ready, and one line per second is logged from those answers:
//! `Stats <eventtime>: <lines>`. A part's flag says whether it has something new
//! to report, and the line is printed only when one of them does; a part that
//! only contributes numbers adds them but never makes a line on its own — which
//! is why a machine with nothing to announce is silent instead of chatty
//! (`statistics.py:63-66`).
//!
//! The numbers themselves belong to the parts:
//! [`system_stats`](crate::core::klippy::extras::system_stats) always has them
//! (`sysload=… cputime=… memavail=…`), `temperature_probe` reports a probe's
//! temperature, and a part that has nothing to say leaves the method at its
//! default `None`. This module is only the clock and the line.
//!
//! # The `debugoutput` gate
//!
//! Upstream arms the timer at ready only when
//! `get_start_args().get('debugoutput') is None` (`statistics.py:61-62`): a run
//! that records or replays the MCU protocol is being watched at the protocol
//! level, so it gets no timed chatter. Here the registration *is* the arming,
//! so a `--debugoutput` run — what [`Printer::is_fileoutput`] reports — never
//! registers a timer.
//!
//! # The timer does not own the object
//!
//! The callback holds a `Weak`, not an `Arc`: the reactor's timer heap would
//! otherwise close `reactor → timer → PrinterStats`, a cycle `Printer::teardown`
//! never breaks — teardown drops the parts, but not the timers they registered —
//! and everything the cycle reaches (the object's own handles included) would
//! outlive the printer. With a `Weak` the parts' registry is the only owner;
//! once it is gone the next tick finds nothing to upgrade and retires the timer.
//! It is the same convention as `objects/subscribe` and `temperature_host`.

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};
use tracing::info;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};
use crate::core::klippy::reactor::TimerHandle;

/// The section id, which is also the name the loader registers the object
/// under.
///
/// Upstream registers this object under its module's name, `statistics`, since
/// `load_object` names the object after the section it was given
/// (`klippy/klippy.py:90-110`). Neither the section nor the object name is
/// client-visible — upstream's `PrinterStats` has no `get_status`, so it is not
/// in `objects/list` — and this host names its modules by what they do.
pub const PRINTER_STATS_OBJECT: &str = "printer_stats";

/// The period upstream returns from `generate_stats` (`statistics.py:72`): one
/// second, counted from the event time the tick was woken for.
const STATS_INTERVAL: f64 = 1.0;

section!("printer_stats", order = 30, load = load_config);

/// The `Stats` line's source: the parts to ask, and the timer that asks them.
pub struct PrinterStats {
    /// The machine, for the object walk. `Weak` so this part adds no cycle of
    /// its own: `PrinterStats → printer → PrinterStats` would keep the whole
    /// machine alive past `teardown`.
    printer: Weak<Printer>,
    /// The parts that answer [`PrinterObject::stats`], collected once at
    /// `klippy:ready` — upstream's `hasattr(o, 'stats')` filter
    /// (`statistics.py:53-54`).
    ///
    /// Held strongly: these are the parts whose numbers this object publishes,
    /// and they are dropped with the printer along with this part. The list is
    /// never grown after ready, which is upstream's behaviour too: a part
    /// registered later is not in the line.
    parts: Mutex<Vec<Arc<dyn PrinterObject>>>,
    /// The 1 Hz timer, once armed. Held so `Drop` can cancel it; the callback
    /// itself holds only a `Weak` (see the module docs).
    timer: Mutex<Option<TimerHandle>>,
}

impl PrinterStats {
    fn new(printer: &Arc<Printer>) -> Self {
        Self {
            printer: Arc::downgrade(printer),
            parts: Mutex::new(Vec::new()),
            timer: Mutex::new(None),
        }
    }

    /// Collect the parts and arm the timer — upstream's `handle_ready`.
    ///
    /// Upstream asks `hasattr(o, 'stats')`, which is a static fact about a
    /// Python object. Here the answer is the call itself, so the walk takes the
    /// parts that answered `Some` at ready and asks them again each second; the
    /// extra call costs one read of the numbers at ready.
    fn start(self: &Arc<Self>) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        let parts: Vec<Arc<dyn PrinterObject>> = printer
            .lookup_objects(None)
            .into_iter()
            .map(|(_name, object)| object)
            .filter(|object| object.stats().is_some())
            .collect();
        *self
            .parts
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = parts;

        // A `debugoutput` run gets no timed line (see the module docs).
        if printer.is_fileoutput() {
            return;
        }
        let reactor = printer.reactor();
        let weak = Arc::downgrade(self);
        let handle = reactor.register_timer_named(
            "printer_stats",
            Box::new(move |eventtime| weak.upgrade().and_then(|stats| stats.tick(eventtime))),
            // Upstream arms with `NOW`; the timer is registered here instead of
            // in `__init__`, so this is the same first tick.
            reactor.monotonic(),
        );
        *self
            .timer
            .lock()
            .unwrap_or_else(|poison| poison.into_inner()) = Some(handle);
    }

    /// One tick: log the line if there is one, and ask to be woken in a second.
    ///
    /// The reactor callback, so it runs on the reactor's thread and must not
    /// block; the parts' `stats` are expected to be cheap reads.
    fn tick(&self, eventtime: f64) -> Option<f64> {
        if let Some(line) = self.line(eventtime) {
            info!("{line}");
        }
        Some(eventtime + STATS_INTERVAL)
    }

    /// The line this tick would log, or `None` when no part has anything to
    /// report — upstream's `if max([s[0] for s in stats])` (`statistics.py:63`).
    fn line(&self, eventtime: f64) -> Option<String> {
        // The handles are cloned out so a part may reach back into the printer
        // (or into this object) while it is being asked.
        let parts = self
            .parts
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
            .clone();
        let stats: Vec<(bool, String)> = parts.iter().filter_map(|part| part.stats()).collect();
        stats_line(eventtime, &stats)
    }
}

impl Drop for PrinterStats {
    fn drop(&mut self) {
        if let Some(handle) = self.timer.lock().unwrap_or_else(|p| p.into_inner()).take() {
            handle.cancel();
        }
    }
}

impl PrinterObject for PrinterStats {
    /// Upstream's `PrinterStats` defines no `get_status`, so the object is not
    /// in `objects/list` (`klippy/klippy.py:114-121` keeps only objects that
    /// have one) — the same filter as every other part that is registered but
    /// invisible to clients.
    fn is_queryable(&self) -> bool {
        false
    }

    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }
}

/// The line upstream logs: `Stats %.1f: …`, or `None` when nothing has anything
/// to say (`statistics.py:64-66`).
///
/// The flag is or'ed over every part, and the printed body is the non-empty
/// lines joined with spaces — so a silent part contributes numbers without
/// causing a line, and a part with an empty string contributes nothing to one.
fn stats_line(eventtime: f64, stats: &[(bool, String)]) -> Option<String> {
    if !stats.iter().any(|(has_content, _line)| *has_content) {
        return None;
    }
    let lines: Vec<&str> = stats
        .iter()
        .map(|(_has_content, line)| line.as_str())
        .filter(|line| !line.is_empty())
        .collect();
    Some(format!("Stats {eventtime:.1}: {}", lines.join(" ")))
}

/// Wire the built object into the printer's `klippy:ready` event.
///
/// Upstream registers that handler inside `__init__`; here the `Arc` exists only
/// after construction, so the handler is attached in `load_config`.
fn on_ready(printer: &Arc<Printer>, stats: &Arc<PrinterStats>) {
    let weak = Arc::downgrade(stats);
    printer.register_event_handler(
        KlippyEvent::KlippyReady,
        Box::new(move |_| {
            if let Some(this) = weak.upgrade() {
                this.start();
            }
        }),
    );
}

/// Upstream's `load_config` for the `PrinterStats` half (`statistics.py:66-68`).
///
/// The section reads no options — upstream reads none either, it only takes the
/// printer and the reactor from the config — so the wrapper is unused. The other
/// half upstream builds here (its `PrinterSysStats`, registered as
/// `system_stats`) is the host's object in this port
/// ([`system_stats`](crate::core::klippy::extras::system_stats)).
///
/// # Errors
/// None: nothing in the section can fail to read.
pub fn load_config(
    _config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let object = Arc::new(PrinterStats::new(printer));
    on_ready(printer, &object);
    Ok(object as Arc<dyn PrinterObject>)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::endpoints::info::NO_KLIPPER_PATH;
    use crate::core::klippy::api::StartArgs;
    use crate::core::klippy::config::ConfigSection;
    use crate::core::klippy::reactor::{ManualReactor, Reactor};

    /// A part that answers `stats` with a fixed line, like a sensor would.
    struct Reporting {
        flag: bool,
        line: String,
    }

    impl PrinterObject for Reporting {
        fn get_status(&self, _eventtime: f64) -> Value {
            json!({})
        }

        fn stats(&self) -> Option<(bool, String)> {
            Some((self.flag, self.line.clone()))
        }
    }

    /// A part with no `stats` at all: the trait default.
    struct Silent;

    impl PrinterObject for Silent {
        fn get_status(&self, _eventtime: f64) -> Value {
            json!({})
        }
    }

    /// A machine over a clock the test drives, with nothing but what the test
    /// registers.
    fn machine() -> (Arc<Printer>, Arc<ManualReactor>) {
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
        (printer, reactor)
    }

    /// The object `[printer_stats]` builds, registered as the loader does it.
    ///
    /// The ready lamp is still dark: the test fires it once the parts it wants
    /// in the walk are registered.
    fn object(printer: &Arc<Printer>) -> Arc<PrinterStats> {
        let section = ConfigSection::new(PRINTER_STATS_OBJECT, None);
        let built =
            load_config(&ConfigWrapper::untracked(&section), printer).expect("the section loads");
        printer
            .add_object(PRINTER_STATS_OBJECT, built)
            .expect("the name is free");
        printer
            .lookup_object_as::<PrinterStats>(PRINTER_STATS_OBJECT)
            .expect("the object is registered")
    }

    /// The object alone — not in the printer's registry — so the test can hold
    /// the only strong handle and watch the timer lose its own.
    fn unregistered(printer: &Arc<Printer>) -> Arc<PrinterStats> {
        let stats = Arc::new(PrinterStats::new(printer));
        on_ready(printer, &stats);
        printer.send_event(&KlippyEvent::KlippyReady);
        stats
    }

    fn add(printer: &Arc<Printer>, name: &str, part: Arc<dyn PrinterObject>) {
        printer.add_object(name, part).expect("the name is free");
    }

    /// The line is made from the parts, and one part's flag is enough to print
    /// it — upstream's `max([s[0] for s in stats])`.
    #[test]
    fn stats_line_prints_when_a_part_has_something_to_report() {
        let (printer, _reactor) = machine();
        add(
            &printer,
            "system_stats",
            Arc::new(Reporting {
                flag: false,
                line: "sysload=0.52 cputime=1.234 memavail=4000000".to_string(),
            }),
        );
        add(
            &printer,
            "temperature_probe probe",
            Arc::new(Reporting {
                flag: true,
                line: "temperature_probe probe: temp=41.0".to_string(),
            }),
        );
        // A part with no `stats` is not in the line.
        add(&printer, "silent", Arc::new(Silent));
        let stats = object(&printer);

        printer.send_event(&KlippyEvent::KlippyReady);

        assert_eq!(
            stats.line(12.34),
            Some(
                "Stats 12.3: sysload=0.52 cputime=1.234 memavail=4000000 \
                 temperature_probe probe: temp=41.0"
                    .to_string()
            )
        );
    }

    /// Nothing to report, no line: upstream's `PrinterSysStats` alone is a
    /// silent machine, and an empty walk is silence too.
    #[test]
    fn stats_line_is_silent_without_a_reporting_part() {
        let (printer, _reactor) = machine();
        add(
            &printer,
            "system_stats",
            Arc::new(Reporting {
                flag: false,
                line: "sysload=0.00 cputime=0.000 memavail=4000000".to_string(),
            }),
        );
        let stats = object(&printer);

        printer.send_event(&KlippyEvent::KlippyReady);

        assert_eq!(stats.line(5.0), None, "a numbers-only part is not news");

        // The same walk with nothing in it at all.
        assert_eq!(stats_line(5.0, &[]), None);
    }

    /// The period is upstream's `return eventtime + 1.` (`statistics.py:72`).
    #[test]
    fn the_timer_asks_to_be_woken_one_second_later() {
        let (printer, _reactor) = machine();
        let stats = object(&printer);
        printer.send_event(&KlippyEvent::KlippyReady);

        assert_eq!(stats.tick(7.5), Some(8.5));
        assert_eq!(stats.tick(0.25), Some(1.25));
    }

    /// The timer never outlives the object it reports for: the callback holds a
    /// `Weak`, so dropping the parts' handle drops the object, and the next tick
    /// retires the timer instead of leaking it.
    #[test]
    fn the_timer_does_not_keep_the_object_alive() {
        let (printer, reactor) = machine();
        let stats = unregistered(&printer);

        // The armed timer fires, from the object's own handle.
        let weak = Arc::downgrade(&stats);
        drop(stats);
        assert!(
            weak.upgrade().is_none(),
            "the timer must not hold a strong reference to the object"
        );
        assert_eq!(reactor.advance(1.0), 1, "the armed timer ran once");
        // It found nothing to upgrade and retired itself.
        assert_eq!(reactor.advance(5.0), 0, "the timer retired with the object");
    }

    /// A `debugoutput` run arms no timer at all — upstream's
    /// `get_start_args().get('debugoutput') is None` gate (`statistics.py:61`).
    #[test]
    fn a_debugoutput_run_arms_no_timer() {
        let (printer, reactor) = machine();
        let mut args = StartArgs::collect("/tmp/printer.cfg", None, NO_KLIPPER_PATH);
        args.debug_output = Some("_test_output".to_string());
        printer.set_start_args(Arc::new(args));
        let stats = object(&printer);

        printer.send_event(&KlippyEvent::KlippyReady);

        assert!(stats.timer.lock().expect("not poisoned").is_none());
        assert_eq!(reactor.advance(5.0), 0, "nothing was armed");
    }

    /// The object is registered but not client-visible, upstream's "no
    /// `get_status`, not in `objects/list`" rule.
    #[test]
    fn the_object_is_registered_but_not_queryable() {
        let (printer, _reactor) = machine();
        object(&printer);

        assert!(printer
            .objects()
            .contains(&PRINTER_STATS_OBJECT.to_string()));
        assert!(!printer
            .queryable_objects()
            .contains(&PRINTER_STATS_OBJECT.to_string()));
    }
}
