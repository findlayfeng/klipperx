//! The MCU as a **pin chip**: the `config_*` resources built on an MCU.
//!
//! Upstream calls these the "printer objects under the resources" of an MCU:
//! the host takes an **oid** and a **pin description**, accumulates a `config_*`
//! command for it, and sends the whole set once, before `finalize_config`. After
//! that the firmware refers to the resource by oid and the host drives it with
//! `queue_*` / `set_*` / `*_transfer` commands. This module holds those
//! resources; the configuration machinery they accumulate into lives in
//! [`super::config`], and the transports in [`super`].
//!
//! | file | resource | upstream |
//! |---|---|---|
//! | [`pin`] | digital output | `MCU_digital_out` |
//! | [`pwm`] | PWM (hardware / software) | `MCU_pwm` |
//! | [`adc`] | analog input | `MCU_adc` |
//!
//! They share the same shape: a resource is built while the config file is
//! loaded (no device yet), so it keeps the MCU's [`ConfigBuilder`] and a shared
//! slot that `McuChip::attach` fills when the MCU connects. The pin **name**
//! becomes a number in the resource's config callback, the first moment the
//! firmware dictionary exists.
//!
//! [`ConfigBuilder`]: super::ConfigBuilder
//! [`McuChip::attach`]: pin::McuChip::attach

mod adc;
mod endstop;
mod i2c;
mod pin;
mod pwm;
mod spi;
mod stepper;
mod trigger_analog;
mod trsync;

pub use adc::McuAdc;
pub use endstop::McuEndstop;
pub use i2c::{I2cMode, McuI2c, DEFAULT_SPEED};
pub use pin::{McuChip, McuDigitalOut};
pub use pwm::McuPwm;
pub use spi::{McuSpi, SpiMode};
pub use stepper::McuStepper;
pub use trigger_analog::{McuTriggerAnalog, SosFilter, SosFilterDesign, MONITOR_MAX};
pub use trsync::{
    Completion, McuTrsync, TriggerDispatch, TrsyncRegistry, TRSYNC_SINGLE_MCU_TIMEOUT,
    TRSYNC_TIMEOUT,
};
