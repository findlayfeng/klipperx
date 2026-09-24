//! `sos_filter` commands — the second-order-section filter `trigger_analog`
//! runs each sample through.
//!
//! Host view of `src/sos_filter.c`. The firmware stores up to `max_sections`
//! sections of five coefficients plus two state words each, applies offset and
//! scale to the raw sample first, and runs the cascade while `n_sections` is
//! set. Writing a section or state deactivates the filter until
//! `sos_filter_set_active` re-arms it.
//!
//! | Direction | Message |
//! |---|---|
//! | host → MCU | `config_sos_filter oid=%c max_sections=%c` |
//! | host → MCU | `sos_filter_set_section oid=%c section_idx=%c sos0=%i sos1=%i sos2=%i sos3=%i sos4=%i` |
//! | host → MCU | `sos_filter_set_state oid=%c section_idx=%c state0=%i state1=%i` |
//! | host → MCU | `sos_filter_set_offset_scale oid=%c offset=%i scale=%i scale_frac_bits=%c auto_offset=%c` |
//! | host → MCU | `sos_filter_set_active oid=%c n_sections=%c coeff_frac_bits=%c` |

use crate::core::klippy::cmd::McuCommand;
use crate::core::klippy::msg::proto::ArgValue;

/// `config_sos_filter oid=%c max_sections=%c` — allocate the filter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigSosFilter {
    /// The oid the config callback assigned.
    pub oid: u8,
    /// The most sections that will ever be written at runtime; `0` means the
    /// filter passes samples through unchanged.
    pub max_sections: u8,
}

impl McuCommand for ConfigSosFilter {
    const NAME: &'static str = "config_sos_filter";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.max_sections),
        ]
    }
}

/// `sos_filter_set_section ...` — store one section's five coefficients
/// (`b0 b1 b2 a1 a2`; the fixed-point `a0` is implicit). Deactivates the
/// filter until the next `sos_filter_set_active`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SosFilterSetSection {
    /// The filter's oid.
    pub oid: u8,
    /// Which section to write; must be below `max_sections`.
    pub section_idx: u8,
    /// The five coefficients, already fixed-point (`Q(31-coeff_frac_bits)`).
    pub sos: [i32; 5],
}

impl McuCommand for SosFilterSetSection {
    const NAME: &'static str = "sos_filter_set_section";

    fn args(&self) -> Vec<ArgValue> {
        let mut args = vec![ArgValue::UInt8(self.oid), ArgValue::UInt8(self.section_idx)];
        args.extend(self.sos.iter().map(|value| ArgValue::Int32(*value)));
        args
    }
}

/// `sos_filter_set_state oid=%c section_idx=%c state0=%i state1=%i` — reset one
/// section's two state words. Deactivates the filter like setting a section.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SosFilterSetState {
    /// The filter's oid.
    pub oid: u8,
    /// Which section's state to write.
    pub section_idx: u8,
    /// The first state word.
    pub state0: i32,
    /// The second state word.
    pub state1: i32,
}

impl McuCommand for SosFilterSetState {
    const NAME: &'static str = "sos_filter_set_state";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.section_idx),
            ArgValue::Int32(self.state0),
            ArgValue::Int32(self.state1),
        ]
    }
}

/// `sos_filter_set_offset_scale ...` — the offset added and scale applied to
/// each raw sample before the sections run (`scale` is `Q(frac_bits)`).
/// `auto_offset` takes the first sample as the offset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SosFilterSetOffsetScale {
    /// The filter's oid.
    pub oid: u8,
    /// Added to each raw sample.
    pub offset: i32,
    /// The scale in fixed point.
    pub scale: i32,
    /// Fractional bits of `scale`.
    pub scale_frac_bits: u8,
    /// Whether the first sample replaces `offset`.
    pub auto_offset: bool,
}

impl McuCommand for SosFilterSetOffsetScale {
    const NAME: &'static str = "sos_filter_set_offset_scale";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::Int32(self.offset),
            ArgValue::Int32(self.scale),
            ArgValue::UInt8(self.scale_frac_bits),
            ArgValue::UInt8(self.auto_offset as u8),
        ]
    }
}

/// `sos_filter_set_active oid=%c n_sections=%c coeff_frac_bits=%c` — activate
/// the first `n_sections` sections; `0` passes samples through (after offset
/// and scale).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SosFilterSetActive {
    /// The filter's oid.
    pub oid: u8,
    /// How many sections to run.
    pub n_sections: u8,
    /// Fractional bits of the section coefficients.
    pub coeff_frac_bits: u8,
}

impl McuCommand for SosFilterSetActive {
    const NAME: &'static str = "sos_filter_set_active";

    fn args(&self) -> Vec<ArgValue> {
        vec![
            ArgValue::UInt8(self.oid),
            ArgValue::UInt8(self.n_sections),
            ArgValue::UInt8(self.coeff_frac_bits),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::klippy::mcu::Dictionary;
    use crate::core::klippy::msg::parser::Parser;
    use crate::core::klippy::msg::proto::ArgType;

    #[test]
    fn test_five_commands_match_the_dictionary() {
        let path = klipperx_test_support::test_dicts_dir().join("atmega2560.dict");
        let raw =
            std::fs::read(&path).unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
        let dict =
            Dictionary::from_json(serde_json::from_slice(&raw).expect("JSON")).expect("dictionary");
        let mut parser = Parser::new();
        dict.install(&mut parser).expect("install the dictionary");

        let cases: Vec<(&str, Box<dyn Fn() -> Vec<ArgValue>>, Vec<(&str, ArgType)>)> = vec![
            (
                ConfigSosFilter::NAME,
                Box::new(|| {
                    ConfigSosFilter {
                        oid: 1,
                        max_sections: 4,
                    }
                    .args()
                }),
                vec![("oid", ArgType::UInt8), ("max_sections", ArgType::UInt8)],
            ),
            (
                SosFilterSetSection::NAME,
                Box::new(|| {
                    SosFilterSetSection {
                        oid: 1,
                        section_idx: 3,
                        sos: [1, -2, 3, -4, 5],
                    }
                    .args()
                }),
                vec![
                    ("oid", ArgType::UInt8),
                    ("section_idx", ArgType::UInt8),
                    ("sos0", ArgType::Int32),
                    ("sos1", ArgType::Int32),
                    ("sos2", ArgType::Int32),
                    ("sos3", ArgType::Int32),
                    ("sos4", ArgType::Int32),
                ],
            ),
            (
                SosFilterSetState::NAME,
                Box::new(|| {
                    SosFilterSetState {
                        oid: 1,
                        section_idx: 0,
                        state0: -100,
                        state1: 200,
                    }
                    .args()
                }),
                vec![
                    ("oid", ArgType::UInt8),
                    ("section_idx", ArgType::UInt8),
                    ("state0", ArgType::Int32),
                    ("state1", ArgType::Int32),
                ],
            ),
            (
                SosFilterSetOffsetScale::NAME,
                Box::new(|| {
                    SosFilterSetOffsetScale {
                        oid: 1,
                        offset: -500_000,
                        scale: 1,
                        scale_frac_bits: 0,
                        auto_offset: true,
                    }
                    .args()
                }),
                vec![
                    ("oid", ArgType::UInt8),
                    ("offset", ArgType::Int32),
                    ("scale", ArgType::Int32),
                    ("scale_frac_bits", ArgType::UInt8),
                    ("auto_offset", ArgType::UInt8),
                ],
            ),
            (
                SosFilterSetActive::NAME,
                Box::new(|| {
                    SosFilterSetActive {
                        oid: 1,
                        n_sections: 2,
                        coeff_frac_bits: 18,
                    }
                    .args()
                }),
                vec![
                    ("oid", ArgType::UInt8),
                    ("n_sections", ArgType::UInt8),
                    ("coeff_frac_bits", ArgType::UInt8),
                ],
            ),
        ];

        for (name, args_fn, params) in cases {
            let msg = parser
                .lookup(name)
                .unwrap_or_else(|| panic!("dictionary has no command '{name}'"));
            let declared: Vec<(&str, ArgType)> = msg
                .params
                .iter()
                .map(|(param, atype)| (param.as_str(), *atype))
                .collect();
            assert_eq!(declared, params, "'{name}' parameters drifted");
            let args = args_fn();
            assert_eq!(args.len(), params.len(), "'{name}' argument count");
            for (index, (value, (_, atype))) in args.iter().zip(params).enumerate() {
                assert_eq!(
                    value.arg_type(),
                    atype,
                    "'{name}' argument {index} type drifted"
                );
            }
        }
    }

    #[test]
    fn test_set_section_args_follow_the_firmware_order() {
        let cmd = SosFilterSetSection {
            oid: 5,
            section_idx: 1,
            sos: [100, -200, 300, -400, 500],
        };
        assert_eq!(
            cmd.args(),
            vec![
                ArgValue::UInt8(5),
                ArgValue::UInt8(1),
                ArgValue::Int32(100),
                ArgValue::Int32(-200),
                ArgValue::Int32(300),
                ArgValue::Int32(-400),
                ArgValue::Int32(500),
            ]
        );
    }
}
