//! The input shaper definitions: every shaper's impulse coefficients and the
//! argument text a shaper name may carry (upstream
//! `klippy/extras/shaper_defs.py`).
//!
//! # What is here
//!
//! | shaper | function | min_freq | max_damping_ratio |
//! |---|---|---|---|
//! | `zv` | [`get_zv_shaper`] | 21 | 0.99 |
//! | `mzv` | [`get_mzv_shaper`] | 23 | 0.99 |
//! | `zvd` | [`get_zvd_shaper`] | 29 | 0.99 |
//! | `ei` | [`get_ei_shaper`] | 29 | 0.4 |
//! | `2hump_ei` | [`get_2hump_ei_shaper`] | 39 | 0.3 |
//! | `3hump_ei` | [`get_3hump_ei_shaper`] | 48 | 0.2 |
//!
//! A shaper name may carry arguments — `mzv(5,0.6)`, `ei(v_tol=0.02)` — which
//! count as the shaper's own optional parameters: `mzv`'s impulse count and
//! duration, `ei`'s vibration tolerance. [`get_shaper_cfg`] compares names with
//! the arguments stripped; [`init_shaper`] also parses them.
//!
//! # Differences from upstream
//!
//! Upstream does the two pieces of text handling with regular expressions. Both
//! are scanned by hand here (`re` is not a dependency of this crate):
//! [`split_shaper_name`] for `(\w+)\s*\((.*)\)$`, and [`numbers_with_names`] for
//! the `(?:(\w+)\s*=\s*)?\s*([\d.]+)` argument scan. They accept the same forms
//! for the texts a shaper name can hold; a name whose arguments neither scanner
//! can read is upstream's `TypeError` out of `init_func`'s call, which no caller
//! catches, and here a [`ShaperError`] instead (see [`unsupported_args`]).
//!
//! Upstream's `init_shaper` takes an `error` callable that re-raises through the
//! caller's error type; the messages are the same, applied by the caller of the
//! function here — [`init_failed`] is that prefix.

use std::f64::consts::PI;
use std::fmt;

use crate::core::klippy::mathutil::{mat_mat_mul, mat_transp, pseudo_inverse};

/// The vibration reduction the default `ei` `v_tol` is picked for
/// (`shaper_defs.py:9`).
pub const SHAPER_VIBRATION_REDUCTION: f64 = 20.;
/// The damping ratio a shaper uses when the config names none
/// (`shaper_defs.py:10`).
pub const DEFAULT_DAMPING_RATIO: f64 = 0.1;

/// A shaper could not be built (`shaper_defs.ShaperError`): one of the checks
/// in [`get_mzv_coeffs`], a mixed argument list, or an argument list the
/// shaper's `init_func` cannot take.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ShaperError {
    message: String,
}

impl ShaperError {
    /// A shaper error with the message the user will see.
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }

    /// The message.
    pub fn message(&self) -> &str {
        &self.message
    }
}

impl fmt::Display for ShaperError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for ShaperError {}

/// The error upstream's `init_shaper` raises through a supplied `error`
/// callable: the shaper's own complaint with upstream's "Failed to initialize
/// shaper: %s" prefix (`shaper_defs.py:193-196`).
pub fn init_failed(error: ShaperError) -> ShaperError {
    ShaperError::new(format!("Failed to initialize shaper: {}", error.message()))
}

// ===========================================================================
// The shapers
// ===========================================================================

/// The coefficients function of one shaper (`InputShaperCfg.init_func`).
///
/// Upstream stores the Python function; the port stores which one it is, so
/// that an [`InputShaperCfg`] looked up by name can still be called through
/// [`init_shaper`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShaperInit {
    /// [`get_zv_shaper`].
    Zv,
    /// [`get_mzv_shaper`].
    Mzv,
    /// [`get_zvd_shaper`].
    Zvd,
    /// [`get_ei_shaper`].
    Ei,
    /// [`get_2hump_ei_shaper`].
    TwoHumpEi,
    /// [`get_3hump_ei_shaper`].
    ThreeHumpEi,
}

/// One shaper's definition
/// (`shaper_defs.InputShaperCfg`, `shaper_defs.py:12-14`).
#[derive(Debug, Clone, Copy)]
pub struct InputShaperCfg {
    /// The name a `shaper_type` option or a `SET_INPUT_SHAPER` argument uses.
    pub name: &'static str,
    /// The function that builds its `(A, T)`.
    pub init_func: ShaperInit,
    /// The lowest frequency the shaper is meant to be configured at.
    pub min_freq: f64,
    /// The highest damping ratio it is meant to be configured for.
    pub max_damping_ratio: f64,
}

/// Every shaper this host knows (`shaper_defs.INPUT_SHAPERS`, `:146-159`).
pub const INPUT_SHAPERS: &[InputShaperCfg] = &[
    InputShaperCfg {
        name: "zv",
        init_func: ShaperInit::Zv,
        min_freq: 21.,
        max_damping_ratio: 0.99,
    },
    InputShaperCfg {
        name: "mzv",
        init_func: ShaperInit::Mzv,
        min_freq: 23.,
        max_damping_ratio: 0.99,
    },
    InputShaperCfg {
        name: "zvd",
        init_func: ShaperInit::Zvd,
        min_freq: 29.,
        max_damping_ratio: 0.99,
    },
    InputShaperCfg {
        name: "ei",
        init_func: ShaperInit::Ei,
        min_freq: 29.,
        max_damping_ratio: 0.4,
    },
    InputShaperCfg {
        name: "2hump_ei",
        init_func: ShaperInit::TwoHumpEi,
        min_freq: 39.,
        max_damping_ratio: 0.3,
    },
    InputShaperCfg {
        name: "3hump_ei",
        init_func: ShaperInit::ThreeHumpEi,
        min_freq: 48.,
        max_damping_ratio: 0.2,
    },
];

/// The shaper of an axis with no shaping: no impulses (`get_none_shaper`,
/// `shaper_defs.py:19-20`).
pub fn get_none_shaper() -> (Vec<f64>, Vec<f64>) {
    (vec![], vec![])
}

/// The two-impulse ZV shaper (`shaper_defs.py:22-28`).
pub fn get_zv_shaper(shaper_freq: f64, damping_ratio: f64) -> (Vec<f64>, Vec<f64>) {
    let df = (1. - damping_ratio * damping_ratio).sqrt();
    let k = (-damping_ratio * PI / df).exp();
    let t_d = 1. / (shaper_freq * df);
    (vec![1., k], vec![0., 0.5 * t_d])
}

/// The three-impulse ZVD shaper (`shaper_defs.py:30-36`).
pub fn get_zvd_shaper(shaper_freq: f64, damping_ratio: f64) -> (Vec<f64>, Vec<f64>) {
    let df = (1. - damping_ratio * damping_ratio).sqrt();
    let k = (-damping_ratio * PI / df).exp();
    let t_d = 1. / (shaper_freq * df);
    (vec![1., 2. * k, k * k], vec![0., 0.5 * t_d, t_d])
}

/// The impulses of the MZV family for an `n` and a duration `t`
/// (`get_mzv_coeffs`, `shaper_defs.py:38-64`).
///
/// The amplitudes come out of the linear system the shaper's zeros imply: one
/// row for `sum(A) = 1`, then a cosine and a sine row per additional impulse.
/// The system is not over-constrained — the extra equations are linearly
/// dependent — so it is solved through [`pseudo_inverse`].
///
/// # Errors
/// [`ShaperError`] when `n` is too small, `t` too large, the system cannot be
/// solved, or an impulse would carry a negative amplitude — upstream's four
/// checks, with upstream's messages.
pub fn get_mzv_coeffs(n: i64, t: f64) -> Result<(Vec<f64>, Vec<f64>), ShaperError> {
    if n < 3 {
        return Err(ShaperError::new(format!(
            "Too small n={n}, must be at least 3"
        )));
    }
    if n as f64 <= 2. * t + 1. + 1e-7 {
        return Err(ShaperError::new(format!(
            "Too large t={:.6} for n={n}, must be less than {:.6}",
            t,
            0.5 * (n as f64 - 1.)
        )));
    }
    // Projected shaper duration with n -> infinity, for the shaper's zeros.
    let tau = t * (n as f64 - 2.) / (n as f64 - 2. * t - 1.);
    let times: Vec<f64> = (0..n).map(|i| i as f64 * t / (n as f64 - 1.)).collect();
    // Build the system of equations for A. The first equation is sum(A) = 1.
    let mut m: Vec<Vec<f64>> = vec![vec![1.; n as usize]];
    let mut f: Vec<f64> = vec![1.];
    // Ensure correct placement of the shaper's zeros.
    for i in 0..n - 1 {
        let w: Vec<f64> = times
            .iter()
            .map(|tj| 2. * PI * (1. + i as f64 / tau) * tj)
            .collect();
        m.push(w.iter().map(|w| w.cos()).collect());
        m.push(w.iter().map(|w| w.sin()).collect());
        f.push(0.);
        f.push(0.);
    }
    let m_inv = pseudo_inverse(&m)
        .ok_or_else(|| ShaperError::new(format!("Ill-formed shaper with n={n}, t={t:.6}")))?;
    // `A = F · (M⁻¹)ᵀ`, as upstream's `mat_mat_mul([F], mat_transp(M_inv))[0]`
    // (whose shapes always line up).
    let a = mat_mat_mul(&[f], &mat_transp(&m_inv))
        .expect("the transposed inverse is as wide as the equation row")[0]
        .clone();
    if a.iter().any(|a| *a < -0.00001) {
        return Err(ShaperError::new(format!(
            "Negative-valued shaper with n={n}, t={t:.6}"
        )));
    }
    Ok((a, times))
}

/// The MZV shaper (`shaper_defs.py:66-83`).
///
/// `n`, `t` and `tau` are the arguments a `mzv(...)` name may carry, with
/// upstream's defaults `3`, `0.` and `0.` applied by [`ShaperInit::call`].
pub fn get_mzv_shaper(
    shaper_freq: f64,
    damping_ratio: f64,
    n: i64,
    t: f64,
    tau: f64,
) -> Result<(Vec<f64>, Vec<f64>), ShaperError> {
    let t = if tau == 0. && t == 0. {
        0.75
    } else if tau != 0. {
        // Infer the total shaper duration from the projected duration with
        // n -> infinity.
        tau * (n as f64 - 1.) / (n as f64 + 2. * tau - 2.)
    } else {
        t
    };
    let (mut a, mut times) = get_mzv_coeffs(n, t)?;
    // Apply damping.
    let df = (1. - damping_ratio * damping_ratio).sqrt();
    let k = (-2. * t * damping_ratio * PI / ((n as f64 - 1.) * df)).exp();
    let t_d = 1. / (shaper_freq * df);
    let mut kp = k;
    for i in 1..n as usize {
        times[i] *= t_d;
        a[i] *= kp;
        kp *= k;
    }
    Ok((a, times))
}

/// The EI shaper (`get_ei_shaper`, `shaper_defs.py:85-103`).
///
/// `v_tol` is the residual vibration the shaper is fitted to; a `ei(...)` name
/// may carry it.
pub fn get_ei_shaper(shaper_freq: f64, damping_ratio: f64, v_tol: f64) -> (Vec<f64>, Vec<f64>) {
    let df = (1. - damping_ratio * damping_ratio).sqrt();
    let t_d = 1. / (shaper_freq * df);
    let dr = damping_ratio;

    let a1 = (0.24968 + 0.24961 * v_tol)
        + ((0.80008 + 1.23328 * v_tol) + (0.49599 + 3.17316 * v_tol) * dr) * dr;
    let a3 = (0.25149 + 0.21474 * v_tol)
        + ((-0.83249 + 1.41498 * v_tol) + (0.85181 - 4.90094 * v_tol) * dr) * dr;
    let a2 = 1. - a1 - a3;

    let t2 = 0.4999
        + (((0.46159 + 8.57843 * v_tol) * v_tol)
            + (((4.26169 - 108.644 * v_tol) * v_tol) + ((1.75601 + 336.989 * v_tol) * v_tol) * dr)
                * dr)
            * dr;

    (vec![a1, a2, a3], vec![0., t2 * t_d, t_d])
}

/// Evaluate a damped shaper from its expansion in the damping ratio
/// (`_get_shaper_from_expansion_coeffs`, `shaper_defs.py:105-119`): one row per
/// impulse, most-significant coefficient first, Horner over `damping_ratio`.
///
/// Both shapers that use it carry four coefficients per impulse.
fn get_shaper_from_expansion_coeffs(
    shaper_freq: f64,
    damping_ratio: f64,
    t: &[[f64; 4]],
    a: &[[f64; 4]],
) -> (Vec<f64>, Vec<f64>) {
    let tau = 1. / shaper_freq;
    let mut times = Vec::with_capacity(a.len());
    let mut amps = Vec::with_capacity(a.len());
    for (t_i, a_i) in t.iter().zip(a) {
        let mut u = t_i[3];
        let mut v = a_i[3];
        for j in 0..3 {
            u = u * damping_ratio + t_i[3 - j - 1];
            v = v * damping_ratio + a_i[3 - j - 1];
        }
        times.push(u * tau);
        amps.push(v);
    }
    (amps, times)
}

/// The expansion coefficients of the 2-hump EI shaper
/// (`shaper_defs.py:122-125`).
const TWO_HUMP_EI_T: [[f64; 4]; 4] = [
    [0., 0., 0., 0.],
    [0.49890, 0.16270, -0.54262, 6.16180],
    [0.99748, 0.18382, -1.58270, 8.17120],
    [1.49920, -0.09297, -0.28338, 1.85710],
];
/// The amplitude expansion coefficients of the 2-hump EI shaper
/// (`shaper_defs.py:126-129`).
const TWO_HUMP_EI_A: [[f64; 4]; 4] = [
    [0.16054, 0.76699, 2.26560, -1.22750],
    [0.33911, 0.45081, -2.58080, 1.73650],
    [0.34089, -0.61533, -0.68765, 0.42261],
    [0.15997, -0.60246, 1.00280, -0.93145],
];

/// The expansion coefficients of the 3-hump EI shaper
/// (`shaper_defs.py:133-137`).
const THREE_HUMP_EI_T: [[f64; 4]; 5] = [
    [0., 0., 0., 0.],
    [0.49974, 0.23834, 0.44559, 12.4720],
    [0.99849, 0.29808, -2.36460, 23.3990],
    [1.49870, 0.10306, -2.01390, 17.0320],
    [1.99960, -0.28231, 0.61536, 5.40450],
];
/// The amplitude expansion coefficients of the 3-hump EI shaper
/// (`shaper_defs.py:138-142`).
const THREE_HUMP_EI_A: [[f64; 4]; 5] = [
    [0.11275, 0.76632, 3.29160, -1.44380],
    [0.23698, 0.61164, -2.57850, 4.85220],
    [0.30008, -0.19062, -2.14560, 0.13744],
    [0.23775, -0.73297, 0.46885, -2.08650],
    [0.11244, -0.45439, 0.96382, -1.46000],
];

/// The 2-hump EI shaper (`get_2hump_ei_shaper`, `shaper_defs.py:121-130`).
pub fn get_2hump_ei_shaper(shaper_freq: f64, damping_ratio: f64) -> (Vec<f64>, Vec<f64>) {
    get_shaper_from_expansion_coeffs(shaper_freq, damping_ratio, &TWO_HUMP_EI_T, &TWO_HUMP_EI_A)
}

/// The 3-hump EI shaper (`get_3hump_ei_shaper`, `shaper_defs.py:132-143`).
pub fn get_3hump_ei_shaper(shaper_freq: f64, damping_ratio: f64) -> (Vec<f64>, Vec<f64>) {
    get_shaper_from_expansion_coeffs(
        shaper_freq,
        damping_ratio,
        &THREE_HUMP_EI_T,
        &THREE_HUMP_EI_A,
    )
}

// ===========================================================================
// Shaper names
// ===========================================================================

/// The config for a shaper name, arguments and all
/// (`shaper_defs.get_shaper_cfg`, `:161-168`); `None` for a name no shaper
/// carries.
pub fn get_shaper_cfg(shaper_name: &str) -> Option<&'static InputShaperCfg> {
    let name = split_shaper_name(shaper_name).0;
    INPUT_SHAPERS.iter().find(|s| s.name == name)
}

/// Split a shaper name into the bare name and its argument text, per
/// upstream's `(\w+)\s*\((.*)\)$` (`shaper_defs.py:162`): the leading word,
/// then what sits between the first `(` and the final `)`.
///
/// A name that does not have that shape comes back whole, with no arguments —
/// upstream's non-match, which then finds no shaper in the table.
fn split_shaper_name(shaper_name: &str) -> (&str, Option<&str>) {
    let name_end = shaper_name
        .char_indices()
        .find(|(_, c)| !is_word_char(*c))
        .map_or(shaper_name.len(), |(i, _)| i);
    if name_end == 0 {
        // `\w+` needs at least one character.
        return (shaper_name, None);
    }
    let (name, rest) = shaper_name.split_at(name_end);
    match rest.trim_start().strip_prefix('(') {
        Some(args) if args.ends_with(')') => (name, Some(&args[..args.len() - 1])),
        _ => (shaper_name, None),
    }
}

/// Whether `c` is part of Python's `\w`: a letter, a digit or `_`.
fn is_word_char(c: char) -> bool {
    c.is_alphanumeric() || c == '_'
}

/// One parsed argument value, typed as upstream's `parse_val` types it: an
/// integer when the text has no `.`, a float when it has one
/// (`shaper_defs.py:180-183`).
#[derive(Debug, Clone, Copy, PartialEq)]
enum ShaperArg {
    Int(i64),
    Float(f64),
}

impl ShaperArg {
    /// The value as a float, whatever it was typed as.
    fn as_float(self) -> f64 {
        match self {
            ShaperArg::Int(value) => value as f64,
            ShaperArg::Float(value) => value,
        }
    }
}

/// A shaper name's argument list, as [`init_shaper`] parses it
/// (`shaper_defs.py:171-183`).
#[derive(Default)]
struct ShaperArgs {
    /// The values written without a name, in order.
    positional: Vec<ShaperArg>,
    /// The values written as `name=value`.
    named: Vec<(String, ShaperArg)>,
}

impl ShaperArgs {
    /// Bind the parsed values against `params` — a call's arguments against the
    /// parameters of the function it calls, the parameters being `(name,
    /// default)` in order. `None` when the values do not fit.
    fn bind(&self, params: &[(&str, ShaperArg)]) -> Option<Vec<ShaperArg>> {
        if self.positional.len() > params.len() {
            return None;
        }
        let mut bound: Vec<ShaperArg> = params.iter().map(|(_, default)| *default).collect();
        for (slot, value) in bound.iter_mut().zip(self.positional.iter()) {
            *slot = *value;
        }
        for (name, value) in &self.named {
            let index = params.iter().position(|(param, _)| param == name)?;
            bound[index] = *value;
        }
        Some(bound)
    }
}

/// The port's complaint about an argument list upstream's `init_func` call
/// cannot take, where Python raises `TypeError` (see the module docs).
fn unsupported_args(shaper: &str) -> ShaperError {
    ShaperError::new(format!("Unsupported arguments for shaper {shaper}"))
}

/// Parse a shaper name's argument text (`shaper_defs.py:171-183`): every number
/// in it, with the name it was assigned to.
///
/// # Errors
/// Upstream's "Mixing named and non-named…" when both forms are used in one
/// name, and [`unsupported_args`] for a number that cannot be read.
fn parse_shaper_args(shaper: &str, text: &str) -> Result<ShaperArgs, ShaperError> {
    let mut positional = Vec::new();
    let mut named = Vec::new();
    for (name, value) in numbers_with_names(text) {
        let value = parse_val(value).ok_or_else(|| unsupported_args(shaper))?;
        match name {
            Some(name) => named.push((name.to_string(), value)),
            None => positional.push(value),
        }
    }
    if !positional.is_empty() && !named.is_empty() {
        return Err(ShaperError::new(
            "Mixing named and non-named shaper parameters is not supported",
        ));
    }
    Ok(ShaperArgs { positional, named })
}

/// Upstream's `parse_val` (`shaper_defs.py:180-183`): a float when the text has
/// a `.`, an integer otherwise; `None` for a text that is neither.
fn parse_val(text: &str) -> Option<ShaperArg> {
    if text.contains('.') {
        text.parse::<f64>().ok().map(ShaperArg::Float)
    } else {
        text.parse::<i64>().ok().map(ShaperArg::Int)
    }
}

/// The `(?:(\w+)\s*=\s*)?\s*([\d.]+)` scan of `shaper_defs.py:179`: every run
/// of digits and dots, paired with the `name=` that directly precedes it, if
/// any. Text that matches neither part of the pattern is skipped, as the
/// regex's scan skips it.
fn numbers_with_names(text: &str) -> Vec<(Option<&str>, &str)> {
    let mut found = Vec::new();
    let mut i = 0;
    while i < text.len() {
        let c = text[i..].chars().next().expect("a char at a char boundary");
        if !(c.is_ascii_digit() || c == '.') {
            i += c.len_utf8();
            continue;
        }
        let start = i;
        while i < text.len() && (text.as_bytes()[i].is_ascii_digit() || text.as_bytes()[i] == b'.')
        {
            i += 1;
        }
        found.push((name_before(text, start), &text[start..i]));
    }
    found
}

/// The `\w+` name the `=` directly before `pos` assigns, if there is one
/// (`(?:(\w+)\s*=\s*)?`).
fn name_before(text: &str, pos: usize) -> Option<&str> {
    let before = text[..pos].trim_end().strip_suffix('=')?;
    let end = before.trim_end().len();
    let before = &before[..end];
    let start = before
        .char_indices()
        .rev()
        .find(|(_, c)| !is_word_char(*c))
        .map_or(0, |(i, c)| i + c.len_utf8());
    (start < end).then(|| &before[start..end])
}

impl ShaperInit {
    /// The parameters this shaper's coefficients function takes beyond the
    /// frequency and the damping ratio, with upstream's Python defaults
    /// (`shaper_defs.py:66 get_mzv_shaper`, `:85 get_ei_shaper`). The other
    /// shapers take nothing.
    fn params(self) -> &'static [(&'static str, ShaperArg)] {
        match self {
            ShaperInit::Mzv => &[
                ("n", ShaperArg::Int(3)),
                ("t", ShaperArg::Float(0.)),
                ("tau", ShaperArg::Float(0.)),
            ],
            ShaperInit::Ei => &[("v_tol", ShaperArg::Float(1. / SHAPER_VIBRATION_REDUCTION))],
            ShaperInit::Zv | ShaperInit::Zvd | ShaperInit::TwoHumpEi | ShaperInit::ThreeHumpEi => {
                &[]
            }
        }
    }

    /// Call the coefficients function the way upstream's
    /// `s.init_func(shaper_freq, damping_ratio, *args_l, **args_kv)` does
    /// (`shaper_defs.py:191-192`).
    fn call(
        self,
        name: &'static str,
        shaper_freq: f64,
        damping_ratio: f64,
        args: &ShaperArgs,
    ) -> Result<(Vec<f64>, Vec<f64>), ShaperError> {
        let bound = args
            .bind(self.params())
            .ok_or_else(|| unsupported_args(name))?;
        match self {
            ShaperInit::Zv => Ok(get_zv_shaper(shaper_freq, damping_ratio)),
            ShaperInit::Zvd => Ok(get_zvd_shaper(shaper_freq, damping_ratio)),
            ShaperInit::Mzv => {
                let n = match bound[0] {
                    ShaperArg::Int(n) => n,
                    // Python's `range(n)` needs an int.
                    ShaperArg::Float(_) => return Err(unsupported_args(name)),
                };
                get_mzv_shaper(
                    shaper_freq,
                    damping_ratio,
                    n,
                    bound[1].as_float(),
                    bound[2].as_float(),
                )
            }
            ShaperInit::Ei => Ok(get_ei_shaper(
                shaper_freq,
                damping_ratio,
                bound[0].as_float(),
            )),
            ShaperInit::TwoHumpEi => Ok(get_2hump_ei_shaper(shaper_freq, damping_ratio)),
            ShaperInit::ThreeHumpEi => Ok(get_3hump_ei_shaper(shaper_freq, damping_ratio)),
        }
    }
}

/// A shaper's `(A, T)` for a frequency and a damping ratio
/// (`shaper_defs.init_shaper`, `:170-197`).
///
/// `Ok(None)` when no shaper carries the name — upstream's fall-through, which
/// only a direct call can reach (its callers look the name up in
/// [`INPUT_SHAPERS`] first).
///
/// # Errors
/// The shaper's own complaint, with the messages upstream raises out of
/// `init_func` — plus [`unsupported_args`] for an argument list Python's call
/// would reject.
pub fn init_shaper(
    shaper_name: &str,
    shaper_freq: f64,
    damping_ratio: f64,
) -> Result<Option<(Vec<f64>, Vec<f64>)>, ShaperError> {
    let (name, args) = split_shaper_name(shaper_name);
    let args = match args {
        Some(text) => parse_shaper_args(name, text)?,
        None => ShaperArgs::default(),
    };
    let Some(cfg) = INPUT_SHAPERS.iter().find(|s| s.name == name) else {
        return Ok(None);
    };
    cfg.init_func
        .call(cfg.name, shaper_freq, damping_ratio, &args)
        .map(Some)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// The golden `(A, T)` values below were produced by running upstream's own
    /// module, `python3` from `third_party/klipper/klippy`:
    ///
    /// ```text
    /// python3 -c "import sys; sys.path.insert(0, 'third_party/klipper/klippy');
    ///            from extras import shaper_defs as s;
    ///            print(s.init_shaper('mzv(5,0.6)', 22.2, 0.1))"
    /// ```
    fn assert_close(got: &[f64], want: &[f64]) {
        assert_eq!(got.len(), want.len(), "{got:?} != {want:?}");
        for (got, want) in got.iter().zip(want) {
            let error = (got - want).abs() / want.abs().max(1.);
            assert!(error < 1e-9, "{got:?} != {want:?}");
        }
    }

    /// The two axes the corpus configures (`test/klippy/input_shaper.cfg`)
    /// plus the shapes the task list pins.
    #[test]
    fn init_shaper_matches_upstreams_coefficients() {
        let (a, t) = init_shaper("mzv(5,0.6)", 22.2, 0.1).unwrap().unwrap();
        assert_close(
            &a,
            &[
                0.3479228981297072,
                0.09833622504441925,
                0.07276446376516911,
                0.08136517121966054,
                0.23819521488817164,
            ],
        );
        assert_close(
            &t,
            &[
                0.,
                0.00679079604904873,
                0.01358159209809746,
                0.02037238814714619,
                0.02716318419619492,
            ],
        );

        let (a, t) = init_shaper("2hump_ei", 33.3, 0.11).unwrap().unwrap();
        assert_close(
            &a,
            &[0.2706888575, 0.3597827015, 0.26544562891, 0.10459352005],
        );
        assert_close(
            &t,
            &[
                0.,
                0.015568548162162165,
                0.030313074990990996,
                0.04468517123423424,
            ],
        );

        // `ei`'s default `v_tol`, and with the corpus's own argument.
        let (a, t) = init_shaper("ei(v_tol=0.02)", 39.3, 0.4).unwrap().unwrap();
        assert_close(&a, &[0.674082952, 0.271201816, 0.054715232]);
        assert_close(&t, &[0., 0.014506883026636108, 0.027763090360813283]);
        let (a, t) = init_shaper("ei", 39.3, 0.4).unwrap().unwrap();
        assert_close(
            &a,
            &[0.7116017800000001, 0.23378553999999996, 0.05461267999999997],
        );
        assert_close(&t, &[0., 0.015766204312011933, 0.027763090360813283]);

        // The default `mzv` (`n=3`, `t=0.75`), at the frequency the corpus's
        // own config uses.
        let (a, t) = init_shaper("mzv", 42., 0.1).unwrap().unwrap();
        assert_close(
            &a,
            &[0.2928932188134524, 0.32687415041000684, 0.18239874354058666],
        );
        assert_close(&t, &[0., 0.00897355192195725, 0.0179471038439145]);

        let (a, t) = init_shaper("zv", 50., 0.1).unwrap().unwrap();
        assert_close(&a, &[1., 0.7292476142876709]);
        assert_close(&t, &[0., 0.01005037815259212]);

        let (a, t) = init_shaper("zvd", 50., 0.1).unwrap().unwrap();
        assert_close(&a, &[1., 1.4584952285753419, 0.5318020829442597]);
        assert_close(&t, &[0., 0.01005037815259212, 0.02010075630518424]);

        let (a, t) = init_shaper("3hump_ei", 48., 0.1).unwrap().unwrap();
        assert_close(&a, &[0.2208542, 0.2772112, 0.25969944, 0.167055, 0.0751792]);
        assert_close(
            &t,
            &[
                0.,
                0.011260456249999998,
                0.021417729166666667,
                0.03137289583333333,
                0.041310981249999996,
            ],
        );
    }

    /// A name that carries no arguments, and one that does, both reach the same
    /// shaper: `get_shaper_cfg` strips the arguments, `min_freq` and
    /// `max_damping_ratio` come along.
    #[test]
    fn get_shaper_cfg_strips_the_arguments() {
        let cfg = get_shaper_cfg("mzv(5,0.6)").expect("mzv");
        assert_eq!(cfg.name, "mzv");
        assert_eq!(cfg.min_freq, 23.);
        assert_eq!(cfg.max_damping_ratio, 0.99);
        assert_eq!(get_shaper_cfg("mzv").unwrap().name, "mzv");
        // A name with an argument the shaper does not take is still found: only
        // `init_shaper` reads the arguments.
        assert_eq!(get_shaper_cfg("mzv(nonsense)").unwrap().name, "mzv");
        assert!(get_shaper_cfg("bogus").is_none());
        assert!(get_shaper_cfg("").is_none());
        assert!(get_shaper_cfg("(5,0.6)").is_none());
        assert!(get_shaper_cfg("mzv(").is_none());
    }

    /// The shaper of an axis with no shaping.
    #[test]
    fn the_none_shaper_has_no_impulses() {
        assert_eq!(get_none_shaper(), (vec![], vec![]));
    }

    /// Upstream's checks, with upstream's messages.
    #[test]
    fn a_shaper_the_impulses_cannot_be_built_for_is_an_error() {
        // `n` below three: `mzv(2,0.5)` (the impulse count is the first
        // argument, so this is upstream's `n=2`).
        let err = init_shaper("mzv(2,0.5)", 22.2, 0.1).unwrap_err();
        assert_eq!(err.message(), "Too small n=2, must be at least 3");
        // The prefix `input_shaper` raises with.
        assert_eq!(
            init_failed(err).message(),
            "Failed to initialize shaper: Too small n=2, must be at least 3"
        );

        let err = init_shaper("mzv(3,1.5)", 22.2, 0.1).unwrap_err();
        assert_eq!(
            err.message(),
            "Too large t=1.500000 for n=3, must be less than 1.000000"
        );

        // An unknown name is `None`, not an error (upstream's fall-through).
        assert!(init_shaper("bogus", 22.2, 0.1).unwrap().is_none());

        // The mixing check runs before the name is looked up, as upstream's
        // does.
        let err = init_shaper("mzv(5,t=0.6)", 22.2, 0.1).unwrap_err();
        assert_eq!(
            err.message(),
            "Mixing named and non-named shaper parameters is not supported"
        );

        // Python raises `TypeError` for these; the port reports the shaper's
        // name instead (module docs).
        for name in ["zv(1)", "mzv(bogus=1)", "mzv(5.5,0.6)"] {
            let err = init_shaper(name, 22.2, 0.1).unwrap_err();
            assert!(err.message().starts_with("Unsupported arguments"), "{name}");
        }
    }

    /// The argument scanner reads the two forms a shaper name uses, and skips
    /// text that matches neither (`shaper_defs.py:179`).
    #[test]
    fn the_argument_scan_reads_positional_and_named_values() {
        assert_eq!(
            numbers_with_names("5,0.6"),
            vec![(None, "5"), (None, "0.6")]
        );
        assert_eq!(
            numbers_with_names("v_tol=0.02"),
            vec![(Some("v_tol"), "0.02")]
        );
        assert_eq!(
            numbers_with_names("5, t = 0.60"),
            vec![(None, "5"), (Some("t"), "0.60")]
        );
        assert_eq!(numbers_with_names("nonsense"), vec![]);
        assert_eq!(numbers_with_names(""), vec![]);
    }

    /// Both shapers of the corpus are in the table, and every entry's `name`
    /// resolves to itself.
    #[test]
    fn the_table_holds_every_shaper_under_its_own_name() {
        assert_eq!(INPUT_SHAPERS.len(), 6);
        for cfg in INPUT_SHAPERS {
            assert_eq!(get_shaper_cfg(cfg.name).unwrap().name, cfg.name);
            assert!(cfg.min_freq > 0.);
            assert!(cfg.max_damping_ratio > 0.);
        }
    }
}
