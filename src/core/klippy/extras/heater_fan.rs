//! `[heater_fan <name>]` — a fan that runs while a heater is hot or heating.
//!
//! Upstream's `klippy/extras/heater_fan.py`: the section is a
//! [`Fan`](crate::core::klippy::extras::fan) core whose speed is chosen once a
//! second — full `fan_speed` while any named heater has a target *or* its
//! measured temperature is above `heater_temp`, and off otherwise (there is no
//! hysteresis: the check is recomputed from scratch every tick).
//!
//! | option | meaning |
//! |---|---|
//! | `pin` | the fan's PWM pin, required (via the `Fan` core) |
//! | `heater` | heaters that switch the fan on (default: `extruder`) |
//! | `heater_temp` | temperature above which a heater switches the fan on (default 50) |
//! | `fan_speed` | duty while on (default 1, `0 ..= 1`) |
//!
//! The per-second timer starts at `klippy:ready` (`PIN_MIN_TIME` after the
//! monotonic clock, then one tick per second), and the `heater` references are
//! resolved in `connect` — upstream's `handle_ready`, with its error wording.
//! The fan keeps running when klippy dies: `Fan::new` is given a
//! `default_shutdown_speed` of 1 (`heater_fan.py:18`), so a hotend fan does not
//! stop with the host.
//!
//! # What is not here
//!
//! * **Heater names are object names.** Upstream looks heaters up in
//!   `heaters.lookup_heater`, which keys them by their *short* name
//!   (`extruder`, or a `[heater_generic <name>]`'s `<name>`); this port has no
//!   such registry yet (H1) and resolves against the printer object registry
//!   instead, so a `heater: <name>` must name an object. The corpus default
//!   (`extruder`) resolves either way.
//! * **The measured temperature is the status one.** Upstream reads the raw
//!   `heater.get_temp(eventtime)`; this port's `Heater` exposes only
//!   `get_status`, whose `temperature` is the smoothed value rounded to two
//!   decimals. Comparing that round number against `heater_temp` can differ
//!   from upstream only within half a hundredth of a degree.

use std::sync::{Arc, Mutex, Weak};

use serde_json::Value;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::event::KlippyEvent;
use crate::core::klippy::extras::fan::Fan;
use crate::core::klippy::extras::heaters;
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{ConnectFuture, Printer, PrinterObject};
use crate::core::klippy::reactor::TimerHandle;

// Only the prefix form (`[heater_fan <name>]`) exists upstream
// (`heater_fan.py:39`).
section!("heater_fan", order = 30, prefix = load_config_prefix);

/// How long after `klippy:ready` the first check runs (`PIN_MIN_TIME`), and
/// the tick period that follows (upstream's callback returns `eventtime + 1.`).
const PIN_MIN_TIME: f64 = 0.100;
const TICK_PERIOD: f64 = 1.0;

/// One `[heater_fan <name>]`.
pub struct HeaterFan {
    fan: Arc<Fan>,
    /// The `heater` option as configured.
    heater_names: Vec<String>,
    heater_temp: f64,
    /// The objects behind the `heater` option, resolved in `connect`.
    heaters: Mutex<Vec<Arc<dyn PrinterObject>>>,
    fan_speed: f64,
    last_speed: Mutex<f64>,
    printer: WeakPrinter,
    /// The handle the `klippy:ready` handler creates, cancelled on drop.
    timer: Mutex<Option<TimerHandle>>,
    self_ref: Weak<HeaterFan>,
}

/// A weak handle to the printer, without importing `std::sync::Weak` twice.
type WeakPrinter = std::sync::Weak<Printer>;

impl HeaterFan {
    /// Read the section and build the fan (`PrinterHeaterFan.__init__`).
    fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Arc<Self>, ConfigError> {
        // Upstream reads `heater`/`heater_temp` before building the fan
        // (`heater_fan.py:13-18`).
        heaters::ensure(printer)?;
        // Upstream's default for `getlist("heater", ("extruder",))`.
        let heater_names = config
            .get_list("heater", ',')
            .unwrap_or_else(|| vec!["extruder".to_string()]);
        let heater_temp = config.get_float("heater_temp", Some(50.0))?;
        // A hotend fan survives klippy's death: `default_shutdown_speed` is 1.
        let fan = Fan::new(config, printer, 1.0)?;
        let fan_speed =
            config.get_float_bounded("fan_speed", Some(1.0), Some(0.0), Some(1.0), None, None)?;

        Ok(Arc::new_cyclic(|weak| Self {
            fan,
            heater_names,
            heater_temp,
            heaters: Mutex::new(Vec::new()),
            fan_speed,
            last_speed: Mutex::new(0.0),
            printer: Arc::downgrade(printer),
            timer: Mutex::new(None),
            self_ref: weak.clone(),
        }))
    }

    /// Resolve the `heater` names (`PrinterHeaterFan.handle_ready`).
    ///
    /// # Errors
    /// An unknown heater, with upstream's wording.
    fn resolve(&self) -> Result<(), ConfigError> {
        let printer = self
            .printer
            .upgrade()
            .ok_or_else(|| ConfigError::new("the printer is gone".to_string()))?;

        let mut heaters = Vec::with_capacity(self.heater_names.len());
        for name in &self.heater_names {
            let heater = printer.lookup_object(name).ok_or_else(|| {
                // Upstream's `heaters.lookup_heater` (`heaters.py:288-292`).
                ConfigError::new(format!("Unknown heater '{name}'"))
            })?;
            heaters.push(heater);
        }
        *self.heaters.lock().unwrap_or_else(|p| p.into_inner()) = heaters;
        Ok(())
    }

    /// Start the per-second check (`PrinterHeaterFan.handle_ready`).
    fn start_timer(&self) {
        let Some(printer) = self.printer.upgrade() else {
            return;
        };
        let reactor = printer.reactor();
        let weak = self.self_ref.clone();
        let handle = reactor.register_timer_named(
            "heater_fan",
            Box::new(move |eventtime| {
                let Some(this) = weak.upgrade() else {
                    return None;
                };
                Some(this.tick(eventtime))
            }),
            reactor.monotonic() + PIN_MIN_TIME,
        );
        *self.timer.lock().unwrap_or_else(|p| p.into_inner()) = Some(handle);
    }

    /// One check (`PrinterHeaterFan.callback`): choose the speed and return the
    /// next wake time.
    fn tick(&self, eventtime: f64) -> f64 {
        let speed = if self.any_heater_on() {
            self.fan_speed
        } else {
            0.0
        };
        let mut last_speed = self.last_speed.lock().unwrap_or_else(|p| p.into_inner());
        if speed != *last_speed {
            *last_speed = speed;
            let _ = self.fan.set_speed(speed);
        }
        eventtime + TICK_PERIOD
    }

    /// Whether any watched heater is heating or hotter than `heater_temp`
    /// (`if target_temp or current_temp > self.heater_temp`).
    fn any_heater_on(&self) -> bool {
        let heaters = self.heaters.lock().unwrap_or_else(|p| p.into_inner());
        heaters.iter().any(|heater| {
            let status = heater.get_status(0.0);
            let target = status.get("target").and_then(Value::as_f64).unwrap_or(0.0);
            let temperature = status
                .get("temperature")
                .and_then(Value::as_f64)
                .unwrap_or(0.0);
            target != 0.0 || temperature > self.heater_temp
        })
    }
}

impl PrinterObject for HeaterFan {
    /// Upstream's `PrinterHeaterFan.get_status`: the fan's own status.
    fn get_status(&self, eventtime: f64) -> Value {
        self.fan.get_status(eventtime)
    }

    fn connect<'a>(&'a self) -> ConnectFuture<'a> {
        Box::pin(async move {
            self.resolve()
                .map_err(crate::core::klippy::error::KlippyError::Config)?;
            Ok(())
        })
    }
}

impl std::fmt::Debug for HeaterFan {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("HeaterFan")
            .field("heater_names", &self.heater_names)
            .finish_non_exhaustive()
    }
}

impl Drop for HeaterFan {
    fn drop(&mut self) {
        if let Some(handle) = self.timer.lock().unwrap_or_else(|p| p.into_inner()).take() {
            handle.cancel();
        }
    }
}

/// Wire the built object into the printer's `klippy:ready` event.
///
/// Upstream registers that handler inside `__init__`; here the `Arc` exists
/// only after construction, so the handler is attached in `load_config_prefix`.
fn on_ready(printer: &Arc<Printer>, fan: &Arc<HeaterFan>) {
    let weak = Arc::downgrade(fan);
    printer.register_event_handler(
        KlippyEvent::KlippyReady,
        Box::new(move |_| {
            if let Some(this) = weak.upgrade() {
                this.start_timer();
            }
        }),
    );
}

/// Upstream's `load_config_prefix` for `[heater_fan <name>]`
/// (`heater_fan.py:39`).
///
/// # Errors
/// A missing or invalid option, or a pin that cannot be set up.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let fan = HeaterFan::new(config, printer)?;
    on_ready(printer, &fan);
    Ok(fan)
}
