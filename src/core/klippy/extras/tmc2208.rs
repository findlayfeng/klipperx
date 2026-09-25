//! `[tmc2208 <stepper>]` — the TMC2208 UART driver (upstream's
//! `klippy/extras/tmc2208.py`).
//!
//! The register/field tables are the chip's; the behaviour (fields, G-Code
//! commands, the current model) lives in [`crate::core::klippy::extras::tmc`],
//! which this module assembles around them. `[tmc2209 <stepper>]` reuses these
//! tables and adds its own ([`crate::core::klippy::extras::tmc2209`]).

use std::collections::HashMap;
use std::sync::Arc;

use crate::core::klippy::config::{ConfigError, ConfigWrapper};
use crate::core::klippy::extras::tmc::{
    stealthchop_helper, FieldHelper, ReadTranslate, TmcCurrent, TmcDriver, TmcTransport,
};
use crate::core::klippy::extras::tmc_uart::TmcUart;
use crate::core::klippy::load::section;
use crate::core::klippy::printer::{Printer, PrinterObject};

// Loaded after the UART resource and the pins it names; alongside the other
// `order = 40` device sections.
section!("tmc2208", order = 40, prefix = load_config_prefix);

/// The chip's internal TSTEP frequency (`TMC_FREQUENCY`).
pub const TMC_FREQUENCY: f64 = 12_000_000.;

/// The largest `uart_address` the chip answers (`MCU_TMC_uart(..., 0, ...)`).
pub const MAX_ADDR: i64 = 0;

/// Register name → UART address (`Registers`).
pub fn registers() -> HashMap<String, u8> {
    [
        ("GCONF", 0x00),
        ("GSTAT", 0x01),
        ("IFCNT", 0x02),
        ("SLAVECONF", 0x03),
        ("OTP_PROG", 0x04),
        ("OTP_READ", 0x05),
        ("IOIN", 0x06),
        ("FACTORY_CONF", 0x07),
        ("IHOLD_IRUN", 0x10),
        ("TPOWERDOWN", 0x11),
        ("TSTEP", 0x12),
        ("TPWMTHRS", 0x13),
        ("VACTUAL", 0x22),
        ("MSCNT", 0x6a),
        ("MSCURACT", 0x6b),
        ("CHOPCONF", 0x6c),
        ("DRV_STATUS", 0x6f),
        ("PWMCONF", 0x70),
        ("PWM_SCALE", 0x71),
        ("PWM_AUTO", 0x72),
    ]
    .into_iter()
    .map(|(name, addr)| (name.to_string(), addr))
    .collect()
}

/// The registers `DUMP_TMC` reads (`ReadRegisters`).
pub fn read_registers() -> Vec<String> {
    [
        "GCONF",
        "GSTAT",
        "IFCNT",
        "OTP_READ",
        "IOIN",
        "FACTORY_CONF",
        "TSTEP",
        "MSCNT",
        "MSCURACT",
        "CHOPCONF",
        "DRV_STATUS",
        "PWMCONF",
        "PWM_SCALE",
        "PWM_AUTO",
    ]
    .into_iter()
    .map(str::to_string)
    .collect()
}

/// The fields whose register value is two's-complement signed (`SignedFields`).
pub const SIGNED_FIELDS: [&str; 3] = ["cur_a", "cur_b", "pwm_scale_auto"];

/// The chip's register/field layout (`Registers` → `Fields`).
pub fn fields() -> HashMap<String, HashMap<String, u32>> {
    let mut fields: HashMap<String, HashMap<String, u32>> = HashMap::new();
    macro_rules! reg {
        ($name:literal, { $($field:literal: $mask:expr),* $(,)? }) => {
            fields.insert(
                $name.to_string(),
                HashMap::from([$(($field.to_string(), $mask as u32)),*]),
            );
        };
    }
    reg!("GCONF", {
        "i_scale_analog": 0x01, "internal_rsense": 0x01 << 1, "en_spreadcycle": 0x01 << 2,
        "shaft": 0x01 << 3, "index_otpw": 0x01 << 4, "index_step": 0x01 << 5,
        "pdn_disable": 0x01 << 6, "mstep_reg_select": 0x01 << 7, "multistep_filt": 0x01 << 8,
        "test_mode": 0x01 << 9
    });
    reg!("GSTAT", { "reset": 0x01, "drv_err": 0x01 << 1, "uv_cp": 0x01 << 2 });
    reg!("IFCNT", { "ifcnt": 0xff });
    reg!("SLAVECONF", { "senddelay": 0x0f << 8 });
    reg!("OTP_PROG", {
        "otpbit": 0x07, "otpbyte": 0x03 << 4, "otpmagic": 0xff << 8
    });
    reg!("OTP_READ", {
        "otp_fclktrim": 0x1f, "otp_ottrim": 0x01 << 5, "otp_internalrsense": 0x01 << 6,
        "otp_tbl": 0x01 << 7, "otp_pwm_grad": 0x0f << 8, "otp_pwm_autograd": 0x01 << 12,
        "otp_tpwmthrs": 0x07 << 13, "otp_pwm_ofs": 0x01 << 16, "otp_pwm_reg": 0x01 << 17,
        "otp_pwm_freq": 0x01 << 18, "otp_iholddelay": 0x03 << 19, "otp_ihold": 0x03 << 21,
        "otp_en_spreadcycle": 0x01 << 23
    });
    // IOIN mapping depends on the driver type (SEL_A field).
    // TMC222x (SEL_A == 0)
    reg!("IOIN@TMC222x", {
        "pdn_uart": 0x01 << 1, "spread": 0x01 << 2, "dir": 0x01 << 3, "enn": 0x01 << 4,
        "step": 0x01 << 5, "ms1": 0x01 << 6, "ms2": 0x01 << 7, "sel_a": 0x01 << 8,
        "version": 0xff << 24
    });
    // TMC220x (SEL_A == 1)
    reg!("IOIN@TMC220x", {
        "enn": 0x01, "ms1": 0x01 << 2, "ms2": 0x01 << 3, "diag": 0x01 << 4,
        "pdn_uart": 0x01 << 6, "step": 0x01 << 7, "sel_a": 0x01 << 8, "dir": 0x01 << 9,
        "version": 0xff << 24
    });
    reg!("FACTORY_CONF", { "fclktrim": 0x1f, "ottrim": 0x03 << 8 });
    reg!("IHOLD_IRUN", {
        "ihold": 0x1f, "irun": 0x1f << 8, "iholddelay": 0x0f << 16
    });
    reg!("TPOWERDOWN", { "tpowerdown": 0xff });
    reg!("TSTEP", { "tstep": 0xfffff });
    reg!("TPWMTHRS", { "tpwmthrs": 0xfffff });
    reg!("VACTUAL", { "vactual": 0xffffff });
    reg!("MSCNT", { "mscnt": 0x3ff });
    reg!("MSCURACT", { "cur_a": 0x1ff, "cur_b": 0x1ff << 16 });
    reg!("CHOPCONF", {
        "toff": 0x0f, "hstrt": 0x07 << 4, "hend": 0x0f << 7, "tbl": 0x03 << 15,
        "vsense": 0x01 << 17, "mres": 0x0f << 24, "intpol": 0x01 << 28, "dedge": 0x01 << 29,
        "diss2g": 0x01 << 30, "diss2vs": 0x01 << 31
    });
    reg!("DRV_STATUS", {
        "otpw": 0x01, "ot": 0x01 << 1, "s2ga": 0x01 << 2, "s2gb": 0x01 << 3,
        "s2vsa": 0x01 << 4, "s2vsb": 0x01 << 5, "ola": 0x01 << 6, "olb": 0x01 << 7,
        "t120": 0x01 << 8, "t143": 0x01 << 9, "t150": 0x01 << 10, "t157": 0x01 << 11,
        "cs_actual": 0x1f << 16, "stealth": 0x01 << 30, "stst": 0x01 << 31
    });
    reg!("PWMCONF", {
        "pwm_ofs": 0xff, "pwm_grad": 0xff << 8, "pwm_freq": 0x03 << 16,
        "pwm_autoscale": 0x01 << 18, "pwm_autograd": 0x01 << 19, "freewheel": 0x03 << 20,
        "pwm_reg": 0xf << 24, "pwm_lim": 0xf << 28
    });
    reg!("PWM_SCALE", { "pwm_scale_sum": 0xff, "pwm_scale_auto": 0x1ff << 16 });
    reg!("PWM_AUTO", { "pwm_ofs_auto": 0xff, "pwm_grad_auto": 0xff << 16 });
    fields
}

/// The `DUMP_TMC` field formatters (`FieldFormatters`, from `tmc2130`).
pub fn field_formatters() -> HashMap<String, fn(i64) -> String> {
    fn extvref(v: i64) -> String {
        if v != 0 {
            "1(ExtVREF)".into()
        } else {
            String::new()
        }
    }
    fn reverse(v: i64) -> String {
        if v != 0 {
            "1(Reverse)".into()
        } else {
            String::new()
        }
    }
    fn reset(v: i64) -> String {
        if v != 0 {
            "1(Reset)".into()
        } else {
            String::new()
        }
    }
    fn drv_err(v: i64) -> String {
        if v != 0 {
            "1(ErrorShutdown!)".into()
        } else {
            String::new()
        }
    }
    fn uv_cp(v: i64) -> String {
        if v != 0 {
            "1(Undervoltage!)".into()
        } else {
            String::new()
        }
    }
    fn version(v: i64) -> String {
        format!("{:#x}", v)
    }
    fn mres(v: i64) -> String {
        format!("{v}({}usteps)", 0x100i64 >> v)
    }
    fn otpw(v: i64) -> String {
        if v != 0 {
            "1(OvertempWarning!)".into()
        } else {
            String::new()
        }
    }
    fn ot(v: i64) -> String {
        if v != 0 {
            "1(OvertempError!)".into()
        } else {
            String::new()
        }
    }
    fn s2ga(v: i64) -> String {
        if v != 0 {
            "1(ShortToGND_A!)".into()
        } else {
            String::new()
        }
    }
    fn s2gb(v: i64) -> String {
        if v != 0 {
            "1(ShortToGND_B!)".into()
        } else {
            String::new()
        }
    }
    fn ola(v: i64) -> String {
        if v != 0 {
            "1(OpenLoad_A!)".into()
        } else {
            String::new()
        }
    }
    fn olb(v: i64) -> String {
        if v != 0 {
            "1(OpenLoad_B!)".into()
        } else {
            String::new()
        }
    }
    fn cs_actual(v: i64) -> String {
        if v != 0 {
            v.to_string()
        } else {
            "0(Reset?)".to_string()
        }
    }
    fn sel_a(v: i64) -> String {
        let names = ["TMC222x", "TMC220x"];
        format!("{v}({})", names.get(v as usize).copied().unwrap_or("?"))
    }
    fn s2vsa(v: i64) -> String {
        if v != 0 {
            "1(ShortToSupply_A!)".into()
        } else {
            String::new()
        }
    }
    fn s2vsb(v: i64) -> String {
        if v != 0 {
            "1(ShortToSupply_B!)".into()
        } else {
            String::new()
        }
    }
    HashMap::from([
        ("i_scale_analog".to_string(), extvref as fn(i64) -> String),
        ("shaft".to_string(), reverse),
        ("reset".to_string(), reset),
        ("drv_err".to_string(), drv_err),
        ("uv_cp".to_string(), uv_cp),
        ("version".to_string(), version),
        ("mres".to_string(), mres),
        ("otpw".to_string(), otpw),
        ("ot".to_string(), ot),
        ("s2ga".to_string(), s2ga),
        ("s2gb".to_string(), s2gb),
        ("ola".to_string(), ola),
        ("olb".to_string(), olb),
        ("cs_actual".to_string(), cs_actual),
        ("sel_a".to_string(), sel_a),
        ("s2vsa".to_string(), s2vsa),
        ("s2vsb".to_string(), s2vsb),
    ])
}

/// The TMC2208's `IOIN` split, chosen by `sel_a` (`read_translate`).
pub fn build_read_translate(fields: Arc<FieldHelper>) -> ReadTranslate {
    Box::new(move |reg_name: &str, val: u32| {
        if reg_name == "IOIN" {
            let drv_type = fields.get_field("sel_a", Some(val), Some("IOIN"));
            let name = if drv_type != 0 {
                "IOIN@TMC220x"
            } else {
                "IOIN@TMC222x"
            };
            (name.to_string(), val)
        } else {
            (reg_name.to_string(), val)
        }
    })
}

/// Upstream's `load_config_prefix` for `[tmc2208 <stepper>]`.
pub fn load_config_prefix(
    config: &ConfigWrapper,
    printer: &Arc<Printer>,
) -> Result<Arc<dyn PrinterObject>, ConfigError> {
    let field_helper = Arc::new(FieldHelper::new(
        fields(),
        &SIGNED_FIELDS,
        field_formatters(),
    ));
    let transport: Arc<dyn TmcTransport> = Arc::new(TmcUart::new(
        config,
        printer,
        registers(),
        MAX_ADDR,
        TMC_FREQUENCY,
    )?);
    field_helper.set_field("pdn_disable", 1, None, None);
    let current = Arc::new(TmcCurrent::new(
        config,
        Arc::clone(&field_helper),
        Arc::clone(&transport),
    )?);
    let read_translate = Some(build_read_translate(Arc::clone(&field_helper)));
    let driver = TmcDriver::new(
        config,
        printer,
        Arc::clone(&field_helper),
        Arc::clone(&transport),
        current,
        read_registers(),
        read_translate,
    )?;
    // Setup basic register values.
    field_helper.set_field("mstep_reg_select", 1, None, None);
    stealthchop_helper(config, &field_helper, transport.as_ref())?;
    let set = |field: &str, default: i64| field_helper.set_config_field(config, field, default);
    // GCONF
    set("multistep_filt", 1)?;
    // CHOPCONF
    set("toff", 3)?;
    set("hstrt", 5)?;
    set("hend", 0)?;
    set("tbl", 2)?;
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
    Ok(driver)
}
