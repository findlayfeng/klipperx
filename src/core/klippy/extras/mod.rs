//! Extras: the `[<section>]` modules built on top of the core.
//!
//! Upstream keeps these in `klippy/extras/`; here they are a submodule of
//! `core::klippy` for the same reason: they are *users* of the core (the pin
//! layer, the G-Code dispatcher, the MCU configuration), not part of it. The
//! loader reaches them through its factory table (`load.rs`), so the core never
//! imports them.

pub mod adc_temperature;
pub mod bed_mesh;
pub mod bed_tilt;
pub mod bltouch;
pub mod board_pins;
pub mod bulk_sensor;
pub(crate) mod bus_debug;
pub mod ds18b20;
pub mod error_mcu;
pub mod extruder;
pub mod fan;
pub mod gcode_move;
pub mod heater_bed;
pub mod heater_generic;
pub mod heaters;
pub mod i2c_device;
pub mod ldc1612;
pub mod manual_probe;
pub mod output_pin;
pub mod probe;
pub mod quad_gantry_level;
pub mod query_endstops;
pub mod screws_tilt_adjust;
pub mod smart_effector;
pub mod spi_device;
pub mod spi_temperature;
pub mod static_digital_output;
pub mod stepper;
pub mod stepper_enable;
pub mod temperature_combined;
pub mod temperature_mcu;
pub mod temperature_sensor;
pub mod toolhead;
pub mod z_tilt;
