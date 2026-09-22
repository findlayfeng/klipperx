//! Extras: the `[<section>]` modules built on top of the core.
//!
//! Upstream keeps these in `klippy/extras/`; here they are a submodule of
//! `core::klippy` for the same reason: they are *users* of the core (the pin
//! layer, the G-Code dispatcher, the MCU configuration), not part of it. The
//! loader reaches them through its factory table (`load.rs`), so the core never
//! imports them.

pub mod board_pins;
pub(crate) mod bus_debug;
pub mod error_mcu;
pub mod i2c_device;
pub mod output_pin;
pub mod query_endstops;
pub mod spi_device;
pub mod stepper;
pub mod toolhead;
