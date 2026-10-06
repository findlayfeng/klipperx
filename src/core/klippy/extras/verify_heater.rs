//! `[verify_heater <name>]` — the periodic "is this heater heating" check.
//!
//! Upstream's `klippy/extras/verify_heater.py`: once a second the check reads
//! its heater's `(temperature, target)` and shuts the machine down when the
//! heater is not making progress towards the target. The section is a *prefix*
//! section, and an object exists for **every** heater whether or not the config
//! writes the section: upstream's `Heater.__init__` calls
//! `printer.load_object(config, "verify_heater %s")` (`heaters.py:64`), which
//! reads an absent section as all-defaults (`klippy/klippy.py:90-113`). Here
//! that is [`PrinterHeaters::setup_heater`] reading its sibling section
//! through [`ConfigWrapper::sibling`], so a bare `[verify_heater heater_bed]`
//! with no options is claimed — and valid — exactly like one that sets them.
//!
//! | option | default | bounds |
//! |---|---|---|
//! | `hysteresis` | 5 | `>= 0` |
//! | `max_error` | 120 | `>= 0` |
//! | `heating_gain` | 2 | `> 0` |
//! | `check_gain_time` | 60 for `heater_bed`, 20 otherwise | `>= 1` |
//!
//! The check runs from `klippy:connect` and is cancelled by `klippy:shutdown`.
//! A heater that is not heating at the expected rate shuts the machine down
//! with `Heater <name> not heating at expected rate` plus [`HINT_THERMAL`]
//! (`verify_heater.py:86-90`); that shutdown is the only thing this module ever
//! does to the outside — it registers no g-code command, sends no event, and
//! upstream gives it no `get_status`, so the object stays out of
//! `objects/list`.
//!
//! # What is not here
//!
//! * **A file-output run never checks anything.** Upstream returns from
//!   `handle_connect` when `start_args['debugoutput']` is set
//!   (`verify_heater.py:34-37`), which is [`Printer::is_fileoutput`]: every
//!   upstream test case runs that way, and so does this host's corpus.
//! * **The 7 s staleness quell is not implemented.** Upstream's `Heater.get_temp`
//!   reports `(0., target)` when the newest reading is older than
//!   `QUELL_STALE_TIME` (`heaters.py:18,116-122`), comparing the reading's time
//!   against `estimated_print_time(eventtime)`. Readings have no single clock to
//!   compare here: the ADC sensors pass the raw firmware clock through
//!   (`pins.rs:220-225`, `adc_temperature.rs:710`) while the serial ones map
//!   theirs with `clock_to_print_time` (`ds18b20.rs:250`), and
//!   `estimated_print_time` is print time. [`Heater::get_temp`](crate::core::klippy::extras::heaters::Heater::get_temp)
//!   therefore returns the smoothed temperature as it stands. The difference shows only when a
//!   sensor goes quiet for more than 7 s with a target set: upstream would read
//!   that as `0` and fault, this host keeps checking the last reading.

use std::sync::{Arc, Mutex, Weak};

use serde_json::{json, Value};
use tracing::{error, info};

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::heaters::{PrinterHeaters, HEATERS_OBJECT};
use crate::core::klippy::printer::{Printer, PrinterObject};
use crate::core::klippy::reactor::TimerHandle;

/// Upstream's `HINT_THERMAL` (`verify_heater.py:7-11`), appended to the fault
/// message — the leading and trailing newlines are part of it.
const HINT_THERMAL: &str = "\nSee the 'verify_heater' section in docs/Config_Reference.md\nfor the parameters that control this check.\n";

/// How often the check runs: upstream returns `eventtime + 1.`
/// (`verify_heater.py:57,85`).
const CHECK_PERIOD: f64 = 1.0;

/// The option defaults (`verify_heater.py:22-29`).
const DEFAULT_HYSTERESIS: f64 = 5.0;
const DEFAULT_MAX_ERROR: f64 = 120.0;
const DEFAULT_HEATING_GAIN: f64 = 2.0;
/// `check_gain_time`'s default: a bed is given longer to heat than a hotend.
const DEFAULT_BED_GAIN_TIME: f64 = 60.0;
const DEFAULT_GAIN_TIME: f64 = 20.0;

/// One heater's check (upstream's `HeaterCheck`).
pub struct HeaterCheck {
    /// The heater's short name (`heater_bed`, `extruder`, a generic heater's
    /// sub-name).
    name: String,
    hysteresis: f64,
    max_error: f64,
    heating_gain: f64,
    check_gain_time: f64,
    state: Mutex<CheckState>,
    printer: Weak<Printer>,
    /// The handle `klippy:connect` creates, cancelled on shutdown or drop.
    timer: Mutex<Option<TimerHandle>>,
    self_ref: Weak<HeaterCheck>,
}

/// The running state of one check (upstream's `HeaterCheck` attributes).
struct CheckState {
    approaching_target: bool,
    starting_approach: bool,
    last_target: f64,
    goal_temp: f64,
    error: f64,
    goal_systime: f64,
}

impl Default for CheckState {
    fn default() -> Self {
        Self {
            approaching_target: false,
            starting_approach: false,
            last_target: 0.0,
            goal_temp: 0.0,
            error: 0.0,
            // Upstream starts at the reactor's `NEVER` (`verify_heater.py:32`);
            // a target change sets it before anything reads it.
            goal_systime: f64::INFINITY,
        }
    }
}

impl HeaterCheck {
    /// Read the section and build the check (`HeaterCheck.__init__`).
    ///
    /// `config` is the check's own `[verify_heater <name>]` section, or `None`
    /// when the config has none — which reads as all-defaults, as upstream's
    /// `load_object` does for an absent section. `heater_name` is the short
    /// name whose section this is: upstream splits it out of the section name
    /// (`verify_heater.py:20`), and the caller built the section identifier from
    /// the same name.
    ///
    /// # Errors
    /// An option outside its bounds, with upstream's wording.
    pub(crate) fn new(
        config: Option<&ConfigWrapper>,
        heater_name: &str,
        printer: &Arc<Printer>,
    ) -> Result<Arc<Self>, ConfigError> {
        let default_gain_time = if heater_name == "heater_bed" {
            DEFAULT_BED_GAIN_TIME
        } else {
            DEFAULT_GAIN_TIME
        };
        let (hysteresis, max_error, heating_gain, check_gain_time) = match config {
            Some(config) => (
                config.get_float_bounded(
                    "hysteresis",
                    Some(DEFAULT_HYSTERESIS),
                    Some(0.0),
                    None,
                    None,
                    None,
                )?,
                config.get_float_bounded(
                    "max_error",
                    Some(DEFAULT_MAX_ERROR),
                    Some(0.0),
                    None,
                    None,
                    None,
                )?,
                config.get_float_bounded(
                    "heating_gain",
                    Some(DEFAULT_HEATING_GAIN),
                    None,
                    None,
                    Some(0.0),
                    None,
                )?,
                config.get_float_bounded(
                    "check_gain_time",
                    Some(default_gain_time),
                    Some(1.0),
                    None,
                    None,
                    None,
                )?,
            ),
            None => (
                DEFAULT_HYSTERESIS,
                DEFAULT_MAX_ERROR,
                DEFAULT_HEATING_GAIN,
                default_gain_time,
            ),
        };

        let check = Arc::new_cyclic(|weak| Self {
            name: heater_name.to_string(),
            hysteresis,
            max_error,
            heating_gain,
            check_gain_time,
            state: Mutex::new(CheckState::default()),
            printer: Arc::downgrade(printer),
            timer: Mutex::new(None),
            self_ref: weak.clone(),
        });
        // Upstream registers both handlers in `__init__` (`verify_heater.py:15-19`);
        // the `Arc` exists only from here on, so the weak handles are taken now.
        let weak = Arc::downgrade(&check);
        printer.register_event_handler(
            KlippyEvent::KlippyConnect,
            Box::new(move |_| {
                if let Some(this) = weak.upgrade() {
                    this.handle_connect();
                }
            }),
        );
        let weak = Arc::downgrade(&check);
        printer.register_event_handler(
            KlippyEvent::KlippyShutdown,
            Box::new(move |_| {
                if let Some(this) = weak.upgrade() {
                    this.handle_shutdown();
                }
            }),
        );
        Ok(check)
    }

    /// Start checking the heater (`HeaterCheck.handle_connect`).
    fn handle_connect(&self) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        if printer.is_fileoutput() {
            // Disabled when the MCU protocol goes to a debug file: nothing
            // answers the temperature queries in such a run.
            return;
        }
        let Some(heaters) = printer.lookup_object_as::<PrinterHeaters>(HEATERS_OBJECT) else {
            return;
        };
        let heater = match heaters.lookup_heater(&self.name) {
            Ok(heater) => heater,
            Err(err) => {
                // Upstream raises this (`verify_heater.py:38-39`). The object is
                // only built by `setup_heater`, which registers the heater first,
                // so it cannot happen there; report it rather than abort.
                error!("{err}");
                return;
            }
        };
        info!("Starting heater checks for {}", self.name);
        let reactor = printer.reactor();
        // Both handles are weak: the timer lives in the reactor's heap until it
        // is cancelled, and a strong `Heater` there would close the loop
        // `reactor → timer → heater → its chip's clock → reactor`. That cycle
        // outlives `Printer::teardown` — the parts are dropped but the cycle is
        // not — so the heater, its `Mcu` and the reactor's blocked device read
        // would all leak past a teardown. The `Weak<HeaterCheck>` is the same
        // shape; the heater itself is kept by `PrinterHeaters`.
        let weak = self.self_ref.clone();
        let weak_heater = Arc::downgrade(&heater);
        let handle = reactor.register_timer_named(
            "verify_heater",
            Box::new(move |eventtime| {
                let Some(this) = weak.upgrade() else {
                    return None;
                };
                let Some(heater) = weak_heater.upgrade() else {
                    return None;
                };
                let (temp, target) = heater.get_temp();
                this.check(eventtime, temp, target)
            }),
            reactor.monotonic(),
        );
        *self.timer.lock().unwrap_or_else(|p| p.into_inner()) = Some(handle);
    }

    /// Stop checking (`HeaterCheck.handle_shutdown`).
    ///
    /// Upstream reschedules the timer to the reactor's `NEVER`
    /// (`verify_heater.py:43-46`); the equivalent here is cancelling it.
    fn handle_shutdown(&self) {
        if let Some(handle) = self.timer.lock().unwrap_or_else(|p| p.into_inner()).take() {
            handle.cancel();
        }
    }

    /// One check on one reading (upstream's `check_event`).
    ///
    /// `None` retires the timer: the heater faulted and the machine is shutting
    /// down (upstream returns the reactor's `NEVER`, `verify_heater.py:90`).
    fn check(&self, eventtime: f64, temp: f64, target: f64) -> Option<f64> {
        let mut state = self.state.lock().unwrap_or_else(|p| p.into_inner());
        if temp >= target - self.hysteresis || target <= 0.0 {
            // At the target, or with no target: reset the checks.
            if state.approaching_target && target != 0.0 {
                info!("Heater {} within range of {:.3}", self.name, target);
            }
            state.approaching_target = false;
            state.starting_approach = false;
            if temp <= target + self.hysteresis {
                state.error = 0.0;
            }
            state.last_target = target;
            return Some(eventtime + CHECK_PERIOD);
        }
        state.error += (target - self.hysteresis) - temp;
        if !state.approaching_target {
            if target != state.last_target {
                // A new target: give the heater `check_gain_time` to gain
                // `heating_gain` before giving up on it.
                info!(
                    "Heater {} approaching new target of {:.3}",
                    self.name, target
                );
                state.approaching_target = true;
                state.starting_approach = true;
                state.goal_temp = temp + self.heating_gain;
                state.goal_systime = eventtime + self.check_gain_time;
            } else if state.error >= self.max_error {
                // Cannot maintain the target.
                drop(state);
                return self.fault();
            }
        } else if temp >= state.goal_temp {
            // Still gaining: aim one `heating_gain` further and give it the
            // same window again.
            state.starting_approach = false;
            state.error = 0.0;
            state.goal_temp = temp + self.heating_gain;
            state.goal_systime = eventtime + self.check_gain_time;
        } else if eventtime >= state.goal_systime {
            state.approaching_target = false;
            info!(
                "Heater {} no longer approaching target {:.3}",
                self.name, target
            );
        } else if state.starting_approach {
            state.goal_temp = state.goal_temp.min(temp + self.heating_gain);
        }
        state.last_target = target;
        Some(eventtime + CHECK_PERIOD)
    }

    /// Report the heater as broken (`HeaterCheck.heater_fault`).
    fn fault(&self) -> Option<f64> {
        let msg = format!("Heater {} not heating at expected rate", self.name);
        error!("{msg}");
        if let Some(printer) = self.printer.upgrade() {
            printer.invoke_shutdown(&format!("{msg}{HINT_THERMAL}"));
        }
        None
    }
}

impl PrinterObject for HeaterCheck {
    /// Upstream has no `get_status` (`verify_heater.py`); `is_queryable` keeps
    /// the object out of `objects/list`, and a query still answers `{}`.
    fn get_status(&self, _eventtime: f64) -> Value {
        json!({})
    }

    fn is_queryable(&self) -> bool {
        false
    }
}

impl std::fmt::Debug for HeaterCheck {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("HeaterCheck")
            .field("name", &self.name)
            .finish()
    }
}

impl Drop for HeaterCheck {
    fn drop(&mut self) {
        if let Some(handle) = self.timer.lock().unwrap_or_else(|p| p.into_inner()).take() {
            handle.cancel();
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::api::StartArgs;
    use crate::core::klippy::config::{AccessTracking, Config};
    use crate::core::klippy::extras::heaters;
    use crate::core::klippy::printer::PrinterState;
    use crate::core::klippy::reactor::{ManualReactor, Reactor};

    /// A printer with no config behind it.
    fn printer() -> Arc<Printer> {
        Arc::new(Printer::new(ManualReactor::shared()))
    }

    /// The check `[verify_heater <name>]` builds from `text`.
    fn check_from(
        text: &str,
        identifier: &str,
        name: &str,
        printer: &Arc<Printer>,
    ) -> Result<Arc<HeaterCheck>, ConfigError> {
        let (config, _) = Config::from_text(text).expect("the config parses");
        let section = config.get_section(identifier).expect("the section exists");
        let wrapper = ConfigWrapper::with_config(section, AccessTracking::shared(), None, &config);
        HeaterCheck::new(Some(&wrapper), name, printer)
    }

    /// The state of a check's machine, for the tests that pin a sequence.
    fn state(check: &HeaterCheck) -> std::sync::MutexGuard<'_, CheckState> {
        check.state.lock().unwrap_or_else(|p| p.into_inner())
    }

    #[test]
    fn test_the_defaults_are_upstreams() {
        let printer = printer();
        let bed = check_from(
            "[verify_heater heater_bed]",
            "verify_heater heater_bed",
            "heater_bed",
            &printer,
        )
        .unwrap();
        assert_eq!(bed.hysteresis, 5.0);
        assert_eq!(bed.max_error, 120.0);
        assert_eq!(bed.heating_gain, 2.0);
        // A bed is given 60 s to gain, a hotend 20.
        assert_eq!(bed.check_gain_time, 60.0);

        let hotend = check_from(
            "[verify_heater extruder]",
            "verify_heater extruder",
            "extruder",
            &printer,
        )
        .unwrap();
        assert_eq!(hotend.check_gain_time, 20.0);
    }

    #[test]
    fn test_each_option_overrides_its_default() {
        let printer = printer();
        let text = "[verify_heater heater_bed]\n\
                    hysteresis: 2\n\
                    max_error: 30\n\
                    heating_gain: 1\n\
                    check_gain_time: 120\n";
        let (config, _) = Config::from_text(text).expect("the config parses");
        let section = config
            .get_section("verify_heater heater_bed")
            .expect("the section exists");
        let access = AccessTracking::shared();
        let wrapper = ConfigWrapper::with_config(section, Arc::clone(&access), None, &config);

        let check = HeaterCheck::new(Some(&wrapper), "heater_bed", &printer).unwrap();

        for option in section.parameters.keys() {
            assert!(
                access.contains("verify_heater heater_bed", option),
                "option '{option}' was not read"
            );
        }
        assert_eq!(check.hysteresis, 2.0);
        assert_eq!(check.max_error, 30.0);
        assert_eq!(check.heating_gain, 1.0);
        assert_eq!(check.check_gain_time, 120.0);
    }

    /// Upstream's bounds are `minval=0.`, `minval=0.`, `above=0.` and
    /// `minval=1.` (`verify_heater.py:22-29`).
    #[test]
    fn test_an_option_outside_its_bounds_names_the_option_and_the_section() {
        let printer = printer();
        for (option, value, expected) in [
            (
                "hysteresis",
                "-1",
                "Option 'hysteresis' in section 'verify_heater heater_bed' must have minimum of 0",
            ),
            (
                "max_error",
                "-1",
                "Option 'max_error' in section 'verify_heater heater_bed' must have minimum of 0",
            ),
            (
                "heating_gain",
                "0",
                "Option 'heating_gain' in section 'verify_heater heater_bed' must be above 0",
            ),
            (
                "check_gain_time",
                "0.5",
                "Option 'check_gain_time' in section 'verify_heater heater_bed' must have minimum of 1",
            ),
        ] {
            let text = format!("[verify_heater heater_bed]\n{option}: {value}\n");
            let err = check_from(&text, "verify_heater heater_bed", "heater_bed", &printer)
                .expect_err("the value is out of range");
            assert_eq!(err.to_string(), expected);
        }
    }

    /// A reading at or above the target — or with no target at all — resets
    /// the check and never faults (`verify_heater.py:50-58`).
    #[test]
    fn test_a_reading_at_the_target_or_without_one_never_faults() {
        let printer = printer();
        let check = check_from(
            "[verify_heater heater_bed]",
            "verify_heater heater_bed",
            "heater_bed",
            &printer,
        )
        .unwrap();

        // No target: upstream's `target <= 0.` branch.
        for tick in 0..1_000 {
            assert!(check.check(f64::from(tick), 20.0, 0.0).is_some());
        }
        assert_eq!(state(&check).error, 0.0);

        // Build up an error first, then reset it from within range.
        check.check(0.0, 20.0, 200.0);
        assert!(state(&check).error > 0.0);
        assert!(check.check(1.0, 195.0, 200.0).is_some());
        assert_eq!(state(&check).error, 0.0);
        assert!(!state(&check).approaching_target);
        for tick in 2..1_000 {
            assert!(check.check(f64::from(tick), 195.0, 200.0).is_some());
        }
        assert_eq!(printer.get_state_message().category, PrinterState::Startup);
    }

    /// A heater that never moves towards its target faults once
    /// `check_gain_time` has passed, with upstream's message
    /// (`verify_heater.py:60-90`).
    #[test]
    fn test_a_stalled_heater_faults_with_upstreams_message() {
        let printer = printer();
        let check = check_from(
            "[verify_heater heater_bed]",
            "verify_heater heater_bed",
            "heater_bed",
            &printer,
        )
        .unwrap();

        // The first tick notes the new target and starts the window.
        assert_eq!(check.check(0.0, 20.0, 200.0), Some(1.0));
        assert!(state(&check).approaching_target);
        // The window is `check_gain_time` (60 for a bed) long; nothing is
        // gained, so the approach is abandoned at t=60 …
        for tick in 1..60 {
            assert_eq!(
                check.check(f64::from(tick), 20.0, 200.0),
                Some(f64::from(tick) + 1.0)
            );
        }
        assert_eq!(check.check(60.0, 20.0, 200.0), Some(61.0));
        assert!(!state(&check).approaching_target);
        // … and the accumulated error trips the check on the next tick.
        assert_eq!(check.check(61.0, 20.0, 200.0), None);
        assert_eq!(printer.get_state_message().category, PrinterState::Shutdown);
        assert_eq!(
            printer.get_state_message().message,
            "Heater heater_bed not heating at expected rate\n\
             See the 'verify_heater' section in docs/Config_Reference.md\n\
             for the parameters that control this check.\n"
        );
    }

    /// Gaining `heating_gain` every second keeps the check satisfied and the
    /// error at zero, however long the climb takes (`verify_heater.py:65-72`).
    #[test]
    fn test_gaining_the_heating_gain_each_second_never_faults() {
        let printer = printer();
        let check = check_from(
            "[verify_heater heater_bed]",
            "verify_heater heater_bed",
            "heater_bed",
            &printer,
        )
        .unwrap();

        // 20 °C up to 194 °C, two degrees (the default `heating_gain`) a tick.
        assert_eq!(check.check(0.0, 20.0, 200.0), Some(1.0));
        assert!(state(&check).approaching_target);
        for tick in 1..=87 {
            let temp = 20.0 + 2.0 * f64::from(tick);
            assert!(
                check.check(f64::from(tick), temp, 200.0).is_some(),
                "tick {tick} at {temp} °C"
            );
            assert_eq!(state(&check).error, 0.0, "tick {tick}");
        }
        assert!(state(&check).approaching_target);
        // 196 °C is within `hysteresis` of the 200 °C target: the check resets.
        assert!(check.check(89.0, 196.0, 200.0).is_some());
        assert!(!state(&check).approaching_target);
        assert_eq!(printer.get_state_message().category, PrinterState::Startup);
    }

    /// A run whose MCU protocol goes to a file starts no timer: every upstream
    /// test case runs that way (`verify_heater.py:34-37`).
    #[test]
    fn test_a_file_output_run_starts_no_timer() {
        let reactor = Arc::new(ManualReactor::new());
        let printer = Arc::new(Printer::new(Arc::clone(&reactor) as Arc<dyn Reactor>));
        let mut args = StartArgs::collect("/tmp/printer.cfg", None);
        args.debug_output = Some("_test_output".to_string());
        printer.set_start_args(Arc::new(args));
        let check = check_from(
            "[verify_heater heater_bed]",
            "verify_heater heater_bed",
            "heater_bed",
            &printer,
        )
        .unwrap();

        printer.send_event(&KlippyEvent::KlippyConnect);

        assert!(check.timer.lock().unwrap().is_none());
        assert_eq!(reactor.advance(600.0), 0);
        assert_eq!(printer.get_state_message().category, PrinterState::Startup);
    }

    /// A check whose heater cannot be found does not start either; the branch
    /// is unreachable through the loader, which only builds a check for a
    /// heater it just registered.
    #[test]
    fn test_a_check_without_its_heater_starts_no_timer() {
        let printer = printer();
        heaters::ensure(&printer).unwrap();
        let check = check_from(
            "[verify_heater heater_bed]",
            "verify_heater heater_bed",
            "heater_bed",
            &printer,
        )
        .unwrap();

        printer.send_event(&KlippyEvent::KlippyConnect);

        assert!(check.timer.lock().unwrap().is_none());
    }
}
