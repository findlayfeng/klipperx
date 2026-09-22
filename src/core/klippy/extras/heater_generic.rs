//! `[heater_generic <name>]` — a named heater with a g-code id.
//!
//! Upstream's `klippy/extras/heater_generic.py`: `setup_heater` under the
//! section's `gcode_id` (or none). The heater itself lives in
//! [`heaters`](crate::core::klippy::extras::heaters).
//!
//! `SET_HEATER_TEMPERATURE HEATER=<name>` is registered by
//! [`heaters::PrinterHeaters::setup_heater`].

use std::sync::Arc;

use serde_json::Value;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::heaters::{self, Heater};
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

// Only the prefix form (`[heater_generic <name>]`) exists upstream.
section!("heater_generic", order = 20, prefix = load_config_prefix);

/// One `[heater_generic <name>]`.
pub struct PrinterHeaterGeneric {
    heater: Arc<Heater>,
}

impl PrinterHeaterGeneric {
    /// Build the heater from its section.
    ///
    /// # Errors
    /// A missing or invalid heater option, or an unknown sensor.
    pub fn new(config: &ConfigWrapper, printer: &Arc<Printer>) -> Result<Self, ConfigError> {
        let gcode_id = config.get_str("gcode_id");
        let heater =
            heaters::ensure(printer)?.setup_heater(config, printer, gcode_id.as_deref())?;
        Ok(Self { heater })
    }

    /// The heater.
    pub fn heater(&self) -> &Arc<Heater> {
        &self.heater
    }
}

impl PrinterObject for PrinterHeaterGeneric {
    fn get_status(&self, _eventtime: f64) -> Value {
        self.heater.get_status()
    }
}

impl std::fmt::Debug for PrinterHeaterGeneric {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterHeaterGeneric")
            .finish_non_exhaustive()
    }
}

/// Upstream's `load_config_prefix` for `[heater_generic <name>]`.
pub(crate) fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    Ok(Arc::new(PrinterHeaterGeneric::new(config, printer)?))
}
