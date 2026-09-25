//! `[tmc2209 <stepper>]` — the TMC2209 UART driver (upstream's
//! `klippy/extras/tmc2209.py`).
//!
//! It reuses the TMC2208 tables ([`crate::core::klippy::extras::tmc2208`]) the
//! way upstream does, adding the StallGuard/CoolStep registers and — the reason
//! this driver and not `tmc2208` — the virtual-endstop chip
//! ([`TmcVirtualPin`]): `[stepper_x]`'s `endstop_pin:
//! tmc2209_stepper_x:virtual_endstop`.
//!
//! Notable differences from `tmc2208`: `uart_address` may be 0..=3,
//! `senddelay=2` avoids tx errors on a shared uart, and `coolstep_threshold`
//! (→ `TCOOLTHRS`) is read.

use std::collections::HashMap;
use std::sync::Arc;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::tmc::{
    stealthchop_helper, vcoolthrs_helper, FieldHelper, TmcCurrent, TmcDriver, TmcTransport,
    TmcVirtualPin,
};
use crate::core::klippy::extras::tmc2208;
use crate::core::klippy::extras::tmc_uart::TmcUart;
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

section!("tmc2209", order = 40, prefix = load_config_prefix);

/// The chip's internal TSTEP frequency (`TMC_FREQUENCY`).
pub const TMC_FREQUENCY: f64 = tmc2208::TMC_FREQUENCY;

/// The largest `uart_address` the chip answers (`MCU_TMC_uart(..., 3, ...)`).
pub const MAX_ADDR: i64 = 3;

/// Register name → UART address (`Registers`).
pub fn registers() -> HashMap<String, u8> {
    let mut registers = tmc2208::registers();
    for (name, addr) in [
        ("TCOOLTHRS", 0x14),
        ("COOLCONF", 0x42),
        ("SGTHRS", 0x40),
        ("SG_RESULT", 0x41),
    ] {
        registers.insert(name.to_string(), addr);
    }
    registers
}

/// The registers `DUMP_TMC` reads (`ReadRegisters`).
pub fn read_registers() -> Vec<String> {
    let mut read = tmc2208::read_registers();
    read.push("SG_RESULT".to_string());
    read
}

/// The TMC2209's register/field layout (`Fields`, over `tmc2208`'s).
pub fn fields() -> HashMap<String, HashMap<String, u32>> {
    let mut fields = tmc2208::fields();
    fields.insert(
        "COOLCONF".to_string(),
        HashMap::from([
            ("semin".to_string(), 0x0f),
            ("seup".to_string(), 0x03 << 5),
            ("semax".to_string(), 0x0f << 8),
            ("sedn".to_string(), 0x03 << 13),
            ("seimin".to_string(), 0x01 << 15),
        ]),
    );
    fields.insert(
        "IOIN".to_string(),
        HashMap::from([
            ("enn".to_string(), 0x01),
            ("ms1".to_string(), 0x01 << 2),
            ("ms2".to_string(), 0x01 << 3),
            ("diag".to_string(), 0x01 << 4),
            ("pdn_uart".to_string(), 0x01 << 6),
            ("step".to_string(), 0x01 << 7),
            ("spread_en".to_string(), 0x01 << 8),
            ("dir".to_string(), 0x01 << 9),
            ("version".to_string(), 0xff << 24),
        ]),
    );
    fields.insert(
        "SGTHRS".to_string(),
        HashMap::from([("sgthrs".to_string(), 0xff)]),
    );
    fields.insert(
        "SG_RESULT".to_string(),
        HashMap::from([("sg_result".to_string(), 0x3ff)]),
    );
    fields.insert(
        "TCOOLTHRS".to_string(),
        HashMap::from([("tcoolthrs".to_string(), 0xfffff)]),
    );
    fields
}

/// Upstream's `load_config_prefix` for `[tmc2209 <stepper>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let field_helper = Arc::new(FieldHelper::new(
        fields(),
        &tmc2208::SIGNED_FIELDS,
        tmc2208::field_formatters(),
    ));
    let transport: Arc<dyn TmcTransport> = Arc::new(TmcUart::new(
        config,
        printer,
        registers(),
        MAX_ADDR,
        TMC_FREQUENCY,
    )?);
    // Setup fields for UART.
    field_helper.set_field("pdn_disable", 1, None, None);
    // Avoid tx errors on shared uart.
    field_helper.set_field("senddelay", 2, None, None);
    let current = Arc::new(TmcCurrent::new(
        config,
        Arc::clone(&field_helper),
        Arc::clone(&transport),
    )?);
    let driver = TmcDriver::new(
        config,
        printer,
        Arc::clone(&field_helper),
        Arc::clone(&transport),
        current,
        read_registers(),
        None,
    )?;
    // Allow virtual pins to be created.
    TmcVirtualPin::new(config, printer, &driver)?;
    // Setup basic register values.
    field_helper.set_field("mstep_reg_select", 1, None, None);
    stealthchop_helper(config, &field_helper, transport.as_ref())?;
    vcoolthrs_helper(config, &field_helper, transport.as_ref())?;
    let set = |field: &str, default: i64| field_helper.set_config_field(config, field, default);
    // GCONF
    set("multistep_filt", 1)?;
    // CHOPCONF
    set("toff", 3)?;
    set("hstrt", 5)?;
    set("hend", 0)?;
    set("tbl", 2)?;
    // COOLCONF
    set("semin", 0)?;
    set("seup", 0)?;
    set("semax", 0)?;
    set("sedn", 0)?;
    set("seimin", 0)?;
    // IHOLDIRUN
    set("iholddelay", 8)?;
    // PWMCONF
    set("pwm_ofs", 36)?;
    set("pwm_grad", 14)?;
    set("pwm_freq", 1)?;
    set("pwm_autoscale", 1)?;
    set("pwm_autograd", 1)?;
    set("freewheel", 0)?;
    set("pwm_reg", 8)?;
    set("pwm_lim", 12)?;
    // TPOWERDOWN
    set("tpowerdown", 20)?;
    // SGTHRS
    set("sgthrs", 0)?;
    Ok(driver)
}
