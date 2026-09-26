//! `trigger_analog`'s host-side filter design: fixed-point conversion, the
//! pre-generated SOS table and [`DigitalFilter`].
//!
//! Upstream keeps all of this in `klippy/extras/trigger_analog.py`
//! (integers/`to_fixed_32`/`calc_frac_bits` L11-42, `GeneratedSOS` L44-51,
//! `DigitalFilter` L73-110) — the *design* half of the `trigger_analog`
//! feature, distinct from the MCU resource (`mcu/resource/trigger_analog.rs`)
//! that sends the results to the firmware. This port keeps the same split:
//! this module computes, the resource transmits.
//!
//! The table only carries filters that would otherwise need SciPy: upstream
//! regenerates a entry with
//! `python -c 'import trigger_analog as m; m.pre_gen_filt("lowpass", 400, 25, 4)'`
//! (`trigger_analog.py:53-70`), and a design key the table misses raises
//! "DigitalFilter require the SciPy module". Two corpus consumers are covered
//! from the table: the eddy probe's tap path (`probe_eddy_current.py:786-792`)
//! asks for `add_lowpass(25.0, 4)` + `add_derivative()` at 400 samples/s, and
//! `[load_cell_probe]` asks for the ADS1220's 660 SPS highpass/lowpass pair
//! (`load_cell_probe.py` `ContinuousTareFilterHelper`) — the lowpass is here
//! because the key is in the table, the highpass likewise.
//!
//! Notches have no table key upstream (`GeneratedSOS` keys only `_butter`), so
//! [`DigitalFilter::add_notch`] carries SciPy's `iirnotch`/`tf2sos` closed
//! form instead.

use crate::core::klippy::config::ConfigError;

/// The largest value a signed 32-bit fixed-point word holds
/// (`trigger_analog.py:11-12`).
const MAX_INT32: f64 = 2_147_483_647.0;
const MIN_INT32: f64 = -2_147_483_648.0;

/// Upstream's `OverflowError("Fixed point Q%d.%d overflow" …)`
/// (`trigger_analog.py:16-18`), raised when [`to_fixed_32`] cannot keep a
/// value inside a signed 32-bit word.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedPointOverflow {
    /// The fractional bits the conversion was attempted at.
    pub frac_bits: u32,
}

impl std::fmt::Display for FixedPointOverflow {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Q(31 - frac_bits).frac_bits, as upstream's `%d` formats it.
        write!(
            f,
            "Fixed point Q{}.{} overflow",
            31i64 - i64::from(self.frac_bits),
            self.frac_bits
        )
    }
}

impl std::error::Error for FixedPointOverflow {}

/// Convert a float to a signed 32-bit fixed-point word with `frac_bits`
/// fractional bits (`trigger_analog.py:24-27`).
///
/// Rounding matches Python's `round()` — ties to even — which the upstream
/// `int(round(value * 2**frac_bits))` inherits; the range check then applies
/// to the rounded word (`assert_is_int32`, L13-19).
///
/// # Errors
/// [`FixedPointOverflow`] with upstream's message when the rounded value
/// leaves `[-2^31, 2^31 - 1]`.
pub fn to_fixed_32(value: f64, frac_bits: u32) -> Result<i32, FixedPointOverflow> {
    // `2**frac_bits` as upstream computes it: an exact power of two through
    // f64's range, then `inf` — `inf`/`NaN` fail the range check below, where
    // Python raises its own overflow while scaling.
    let scale = 2f64.powi(i32::try_from(frac_bits).unwrap_or(i32::MAX));
    let fixed_val = (value * scale).round_ties_even();
    if !(MIN_INT32..=MAX_INT32).contains(&fixed_val) {
        return Err(FixedPointOverflow { frac_bits });
    }
    Ok(fixed_val as i32)
}

/// The most fractional bits a list of values fits in
/// (`trigger_analog.py:29-42`).
///
/// Integer lists need none; otherwise the integer part's width leaves
/// `31 - bit_length` bits, and a value that only overflows *after rounding*
/// gives back one bit (upstream's "rare case" fallback, L37-41).
pub fn calc_frac_bits(values: &[f64]) -> u32 {
    if values.iter().all(|value| *value == value.trunc()) {
        return 0;
    }
    let mv = values
        .iter()
        .map(|value| value.abs())
        .fold(0.0f64, f64::max);
    // `int(mv).bit_length()` (L31): the whole part's width; values beyond
    // `u64` saturate here and land on the same `frac_bits <= 0` answer.
    let whole = mv.trunc();
    let bit_length = if whole <= 0.0 {
        0
    } else {
        64 - (whole as u64).leading_zeros()
    };
    let frac_bits = 31i64 - i64::from(bit_length);
    if frac_bits <= 0 {
        return 0;
    }
    let frac_bits = frac_bits as u32;
    if values
        .iter()
        .any(|value| to_fixed_32(*value, frac_bits).is_err())
    {
        return frac_bits - 1;
    }
    frac_bits
}

/// Pre-generated SOS filters, so the common designs need no SciPy
/// (`GeneratedSOS`, `trigger_analog.py:44-51`).
///
/// The key is upstream's `(btype, float(sps) / frequency, order)` compared
/// with exact float equality, as the dict's keys are.
static GENERATED_SOS: &[(&str, f64, u32, &[[f64; 6]])] = &[
    (
        "lowpass",
        400.0 / 25.0,
        4,
        &[
            [
                0.0009334986129548442,
                0.0018669972259096883,
                0.0009334986129548442,
                1.0,
                -1.3651172372392975,
                0.4775922500725171,
            ],
            [1.0, 2.0, 1.0, 1.0, -1.6117270964574348, 0.7445208382054344],
        ],
    ),
    // The `[load_cell_probe]` corpus design: an ADS1220's turbo 660 SPS with
    // the file's `drift_filter_cutoff_frequency: 0.8` and
    // `buzz_filter_cutoff_frequency: 100.0`. Generated with upstream's own
    // tool:
    // `python -c 'import trigger_analog as m; m.pre_gen_filt("highpass", 660, 0.8, 2)'`
    // (and the same for the lowpass), i.e. `scipy.signal.butter(order, Wn,
    // btype, fs=sps, output='sos')` — the table is the SciPy-free fast path.
    (
        "highpass",
        825.0,
        2,
        &[[
            0.9946291620443124,
            -1.9892583240886248,
            0.9946291620443124,
            1.0,
            -1.989229477980254,
            0.9892871701969954,
        ]],
    ),
    (
        "lowpass",
        6.6,
        2,
        &[[
            0.13323133702531725,
            0.2664626740506345,
            0.13323133702531725,
            1.0,
            -0.736116482202693,
            0.26904183030396217,
        ]],
    ),
];

/// Look a design up in [`GENERATED_SOS`] (upstream's `GeneratedSOS.get`
/// through `_butter`, `trigger_analog.py:96-101`).
///
/// # Errors
/// None: a miss is the caller's signal to reach for SciPy.
pub fn generated_sos(
    btype: &str,
    sps_over_frequency: f64,
    order: u32,
) -> Option<&'static [[f64; 6]]> {
    GENERATED_SOS
        .iter()
        .find(|(key_type, key_ratio, key_order, _)| {
            *key_type == btype && *key_order == order && *key_ratio == sps_over_frequency
        })
        .map(|(_, _, _, sections)| *sections)
}

/// A digital filter design: second-order sections in
/// `[b0 b1 b2 a1 a2 a3]` row order, six coefficients each
/// (upstream's `DigitalFilter`, `trigger_analog.py:73-110`).
///
/// The MCU applies the filter (`mcu/resource/trigger_analog.rs`); this side
/// only designs it and renders the design into fixed-point words.
#[derive(Debug, Clone, PartialEq)]
pub struct DigitalFilter {
    /// The sensor's samples per second (`sps`).
    sample_frequency: f64,
    /// The sections so far, in add order.
    filter_sections: Vec<[f64; 6]>,
}

impl DigitalFilter {
    /// A filter over a sensor sampling at `sample_frequency`
    /// (`DigitalFilter.__init__`).
    pub fn new(sample_frequency: f64) -> Self {
        Self {
            sample_frequency,
            filter_sections: Vec::new(),
        }
    }

    /// Add a Butterworth low-pass (`add_lowpass` → `_butter`).
    ///
    /// # Errors
    /// "DigitalFilter require the SciPy module" when the design is not in
    /// [`GENERATED_SOS`] (`trigger_analog.py:77-80, 96-101`).
    pub fn add_lowpass(&mut self, frequency: f64, order: u32) -> Result<(), ConfigError> {
        self.add_butter("lowpass", frequency, order)
    }

    /// Add a Butterworth high-pass (`add_highpass` → `_butter`).
    ///
    /// # Errors
    /// As [`Self::add_lowpass`].
    pub fn add_highpass(&mut self, frequency: f64, order: u32) -> Result<(), ConfigError> {
        self.add_butter("highpass", frequency, order)
    }

    /// `_butter`: the table first, SciPy (which this host does not have) for
    /// anything else (`trigger_analog.py:96-101`).
    fn add_butter(&mut self, btype: &str, frequency: f64, order: u32) -> Result<(), ConfigError> {
        let sections =
            generated_sos(btype, self.sample_frequency / frequency, order).ok_or_else(|| {
                ConfigError::new("DigitalFilter require the SciPy module".to_string())
            })?;
        self.filter_sections.extend_from_slice(sections);
        Ok(())
    }

    /// Add a second-order IIR notch (`add_notch`, `trigger_analog.py:88-94`).
    ///
    /// Upstream asks SciPy for `iirnotch(notch_freq, Q, fs=sps)` and takes the
    /// single `tf2sos` section; the closed form here is exactly that pair of
    /// formulas (`scipy.signal._filter_design._design_notch_peak_filter`,
    /// Orfanidis eqs. 11.3.4/11.3.6/11.3.7/11.3.21):
    ///
    /// ```text
    /// w0 = 2*pi*f0/sps;  bw = w0/Q;  beta = tan(bw/2);  gain = 1/(1+beta)
    /// b = gain * [1, -2*cos(w0), 1]
    /// a = [1, -2*gain*cos(w0), 2*gain - 1]
    /// ```
    ///
    /// `tf2sos(b, a)` of one second-order section is `[[b0, b1, b2, 1, a1, a2]]`
    /// (a0 is already 1), so no factorization step is needed. Q's default
    /// `2.0` and the caller's range check keep `0 < 2*f0/sps < 1`; a frequency
    /// outside it is upstream's SciPy `ValueError`, which this host never tries
    /// to reproduce (the config readers refuse it first).
    pub fn add_notch(&mut self, frequency: f64, quality: f64) {
        let w0 = 2.0 * std::f64::consts::PI * frequency / self.sample_frequency;
        let bw = w0 / quality;
        let beta = (bw / 2.0).tan();
        let gain = 1.0 / (1.0 + beta);
        let cos_w0 = w0.cos();
        self.filter_sections.push([
            gain,
            -2.0 * gain * cos_w0,
            gain,
            1.0,
            -2.0 * gain * cos_w0,
            2.0 * gain - 1.0,
        ]);
    }

    /// Append the sample-to-sample difference stage (`add_derivative`,
    /// `trigger_analog.py:91-94`).
    pub fn add_derivative(&mut self) {
        self.filter_sections.push([1., -1., 0., 1., 0., 0.]);
    }

    /// The sections designed so far (`get_filter_sections`).
    pub fn get_filter_sections(&self) -> &[[f64; 6]] {
        &self.filter_sections
    }

    /// How many sections the design holds (`get_size`).
    pub fn get_size(&self) -> usize {
        self.filter_sections.len()
    }

    /// The per-section initial state, zeros when none was set
    /// (`get_initial_state`: no design state yet → `[[0., 0.]] * n`).
    pub fn get_initial_state(&self) -> Vec<[f64; 2]> {
        vec![[0.0, 0.0]; self.filter_sections.len()]
    }

    /// The fractional bits every coefficient converts at
    /// (`MCU_SosFilter._calc_coeff_bits`, `trigger_analog.py:164-170`: the
    /// flattened sections, column 3 included).
    pub fn coeff_frac_bits(&self) -> u32 {
        let flattened: Vec<f64> = self.filter_sections.iter().flatten().copied().collect();
        calc_frac_bits(&flattened)
    }

    /// Render the sections as fixed-point `[b0 b1 b2 a1 a2]` words
    /// (`MCU_SosFilter._convert_filter`, `trigger_analog.py:173-191`):
    /// column 3 (the `a0` divider, always `1.0`) is omitted.
    ///
    /// # Errors
    /// Upstream's `ValueError` when column 3 is not `1.0`, or
    /// [`FixedPointOverflow`]'s message when a coefficient does not fit.
    pub fn to_fixed_sections(&self, coeff_frac_bits: u32) -> Result<Vec<[i32; 5]>, ConfigError> {
        let mut out = Vec::with_capacity(self.filter_sections.len());
        for section in &self.filter_sections {
            if section[3] != 1.0 {
                return Err(ConfigError::new(format!(
                    "Coefficient 3 is expected to be 1.0 but was {:.6}",
                    section[3]
                )));
            }
            let mut fixed = [0i32; 5];
            for (col, coeff) in section.iter().enumerate() {
                if col == 3 {
                    continue;
                }
                let index = if col < 3 { col } else { col - 1 };
                fixed[index] = to_fixed_32(*coeff, coeff_frac_bits)
                    .map_err(|err| ConfigError::new(err.to_string()))?;
            }
            out.push(fixed);
        }
        Ok(out)
    }

    /// Render the initial state as fixed-point words
    /// (`MCU_SosFilter._convert_state`, `trigger_analog.py:194-211`): each
    /// state value scaled by `start_value`, converted with no fractional bits.
    ///
    /// # Errors
    /// [`FixedPointOverflow`]'s message when a scaled value does not fit.
    pub fn to_fixed_state(&self, start_value: f64) -> Result<Vec<[i32; 2]>, ConfigError> {
        let mut out = Vec::with_capacity(self.filter_sections.len());
        for section in self.get_initial_state() {
            let mut fixed = [0i32; 2];
            for (col, value) in section.iter().enumerate() {
                fixed[col] = to_fixed_32(value * start_value, 0)
                    .map_err(|err| ConfigError::new(err.to_string()))?;
            }
            out.push(fixed);
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The upstream tap design: 400 sps, `add_lowpass(25.0, 4)` then
    /// `add_derivative()` (`probe_eddy_current.py:786-792`).
    fn tap_design() -> DigitalFilter {
        let mut filter = DigitalFilter::new(400.0);
        filter.add_lowpass(25.0, 4).unwrap();
        filter.add_derivative();
        filter
    }

    #[test]
    fn to_fixed_32_scales_and_rounds_ties_to_even() {
        // Scales…
        assert_eq!(to_fixed_32(1.5, 1).unwrap(), 3);
        assert_eq!(to_fixed_32(-1.5, 1).unwrap(), -3);
        assert_eq!(to_fixed_32(0.0, 0).unwrap(), 0);
        // …rounding ties the way Python's `round()` does (ties to even), so
        // `int(round(0.5)) == 0` and `int(round(2.5)) == 2` as upstream.
        assert_eq!(to_fixed_32(0.5, 0).unwrap(), 0);
        assert_eq!(to_fixed_32(2.5, 0).unwrap(), 2);
        // The exact word boundaries are inclusive.
        assert_eq!(to_fixed_32(2_147_483_647.0, 0).unwrap(), i32::MAX);
        assert_eq!(to_fixed_32(-2_147_483_648.0, 0).unwrap(), i32::MIN);
    }

    #[test]
    fn to_fixed_32_reports_upstream_overflow_wording() {
        let err = to_fixed_32(2.0, 31).unwrap_err();
        assert_eq!(err.to_string(), "Fixed point Q0.31 overflow");

        let err = to_fixed_32(-2.0, 31).unwrap_err();
        assert_eq!(err.to_string(), "Fixed point Q0.31 overflow");

        // One past the top word, rounded up from `i32::MAX + 0.5`.
        let err = to_fixed_32(2_147_483_647.5, 0).unwrap_err();
        assert_eq!(err.to_string(), "Fixed point Q31.0 overflow");

        // Beyond f64's finite scale the check still refuses, never wraps.
        let err = to_fixed_32(1.0, u32::MAX).unwrap_err();
        assert_eq!(err.frac_bits, u32::MAX);
    }

    #[test]
    fn calc_frac_bits_zero_for_integers_and_widest_fit_otherwise() {
        assert_eq!(calc_frac_bits(&[0.0]), 0);
        assert_eq!(calc_frac_bits(&[1.0, 2.0, -3.0]), 0);
        // `31 - int(0.5).bit_length()` → 31 fractional bits, and 0.5 still
        // fits there.
        assert_eq!(calc_frac_bits(&[0.5]), 31);
        assert_eq!(calc_frac_bits(&[-0.75]), 31);
        // The table's widest coefficient is 2.0: `31 - bit_length(2)` → 29.
        let flattened: Vec<f64> = generated_sos("lowpass", 400.0 / 25.0, 4)
            .unwrap()
            .iter()
            .flatten()
            .copied()
            .collect();
        assert_eq!(calc_frac_bits(&flattened), 29);
    }

    #[test]
    fn calc_frac_bits_gives_back_a_bit_when_rounding_would_overflow() {
        // Just under 2.0 rounds *up* at `31 - bit_length(1) == 30` fractional
        // bits to 2^31, one past the word — upstream's "rare case" falls back
        // to one bit less (`trigger_analog.py:37-41`).
        let almost_two = 2.0 - f64::EPSILON;
        assert_eq!(calc_frac_bits(&[almost_two]), 29);
    }

    #[test]
    fn the_generated_table_answers_the_tap_key_only() {
        let sections = generated_sos("lowpass", 400.0 / 25.0, 4).unwrap();
        assert_eq!(sections.len(), 2);
        assert_eq!(sections[0][0], 0.0009334986129548442);
        assert_eq!(sections[1][4], -1.6117270964574348);

        // Other keys miss, as the dict would: another order, another shape,
        // another sample rate.
        assert!(generated_sos("lowpass", 400.0 / 25.0, 2).is_none());
        assert!(generated_sos("highpass", 400.0 / 25.0, 4).is_none());
        assert!(generated_sos("lowpass", 400.0 / 30.0, 4).is_none());
    }

    /// The corpus `[load_cell_probe]` design's butter pair (660 SPS ADS1220,
    /// 0.8 Hz highpass and 100 Hz lowpass, both 2nd order).
    #[test]
    fn the_generated_table_carries_the_load_cell_probe_butter_pair() {
        let highpass = generated_sos("highpass", 660.0 / 0.8, 2).unwrap();
        assert_eq!(highpass.len(), 1);
        assert_eq!(highpass[0][0], 0.9946291620443124);
        assert_eq!(highpass[0][4], -1.989229477980254);

        let lowpass = generated_sos("lowpass", 660.0 / 100.0, 2).unwrap();
        assert_eq!(lowpass.len(), 1);
        assert_eq!(lowpass[0][0], 0.13323133702531725);
        assert_eq!(lowpass[0][5], 0.26904183030396217);

        // `add_highpass` + `add_lowpass` append the table's sections in add
        // order.
        let mut filter = DigitalFilter::new(660.0);
        filter.add_highpass(0.8, 2).unwrap();
        filter.add_lowpass(100.0, 2).unwrap();
        assert_eq!(filter.get_size(), 2);
        assert_eq!(filter.get_filter_sections()[0], highpass[0]);
        assert_eq!(filter.get_filter_sections()[1], lowpass[0]);
    }

    /// A key the table does not carry still reaches the SciPy refusal, and
    /// leaves the design untouched.
    #[test]
    fn add_highpass_misses_the_table_like_add_lowpass() {
        let mut filter = DigitalFilter::new(660.0);
        let err = filter.add_highpass(0.8, 4).unwrap_err();
        assert_eq!(err.to_string(), "DigitalFilter require the SciPy module");
        assert_eq!(filter.get_size(), 0);
    }

    /// The notch section is SciPy's `iirnotch`, bit for bit (values from
    /// `scipy.signal.iirnotch(50, Q=2.0, fs=660)`).
    ///
    /// Upstream then runs `tf2sos(b, a)` over the pair; for a single
    /// second-order section `tf2sos` returns the same numbers up to a couple
    /// of last-bit roundings from its own reconstruction (`a1` reads
    /// `-1.5878233713947392` there against `iirnotch`'s
    /// `-1.587823371394739`). Those bits do not change the filter's behaviour
    /// and cannot be reproduced without reimplementing `tf2sos`'s pairing, so
    /// the reference here is the exact `iirnotch` polynomial the closed form
    /// mirrors.
    #[test]
    fn add_notch_matches_the_scipy_closed_form() {
        let mut filter = DigitalFilter::new(660.0);
        filter.add_notch(50.0, 2.0);
        filter.add_notch(60.0, 2.0);
        assert_eq!(
            filter.get_filter_sections(),
            &[
                [
                    0.8932043460899288,
                    -1.587823371394739,
                    0.8932043460899288,
                    1.0,
                    -1.587823371394739,
                    0.7864086921798576
                ],
                [
                    0.8742953116440193,
                    -1.4710080393165397,
                    0.8742953116440193,
                    1.0,
                    -1.4710080393165397,
                    0.7485906232880386
                ],
            ]
        );
    }

    #[test]
    fn the_tap_design_holds_the_table_sections_then_the_derivative() {
        let filter = tap_design();
        assert_eq!(filter.get_size(), 3);
        let sections = filter.get_filter_sections();
        assert_eq!(
            sections[0],
            generated_sos("lowpass", 400.0 / 25.0, 4).unwrap()[0]
        );
        assert_eq!(
            sections[1],
            generated_sos("lowpass", 400.0 / 25.0, 4).unwrap()[1]
        );
        assert_eq!(sections[2], [1.0, -1.0, 0.0, 1.0, 0.0, 0.0]);

        // No state was set, so every section starts at rest.
        assert_eq!(filter.get_initial_state(), vec![[0.0, 0.0]; 3]);
    }

    #[test]
    fn a_design_the_table_misses_is_the_scipy_error_and_changes_nothing() {
        let mut filter = DigitalFilter::new(400.0);
        let err = filter.add_lowpass(25.0, 2).unwrap_err();
        assert_eq!(err.to_string(), "DigitalFilter require the SciPy module");
        assert_eq!(filter.get_size(), 0);

        // A different sample rate misses the key the same way.
        let mut filter = DigitalFilter::new(480.0);
        let err = filter.add_lowpass(25.0, 4).unwrap_err();
        assert_eq!(err.to_string(), "DigitalFilter require the SciPy module");
    }

    #[test]
    fn the_design_converts_to_fixed_point_coefficients() {
        let filter = tap_design();
        // The widest coefficient (2.0) decides the fractional bits…
        assert_eq!(filter.coeff_frac_bits(), 29);
        // …and every coefficient rounds into its word with column 3 dropped.
        assert_eq!(
            filter.to_fixed_sections(29).unwrap(),
            vec![
                [501_168, 1_002_337, 501_168, -732_891_736, 256_405_387],
                [
                    536_870_912,
                    1_073_741_824,
                    536_870_912,
                    -865_289_396,
                    399_711_581
                ],
                [536_870_912, -536_870_912, 0, 0, 0],
            ]
        );
    }

    #[test]
    fn a_non_unit_divider_or_an_oversized_word_is_refused() {
        let mut filter = tap_design();
        filter.filter_sections[0][3] = 0.5;
        let err = filter.to_fixed_sections(29).unwrap_err();
        assert_eq!(
            err.to_string(),
            "Coefficient 3 is expected to be 1.0 but was 0.500000"
        );

        // 2.0 at 40 fractional bits leaves every word: upstream's
        // `OverflowError` text on the way out.
        let err = tap_design().to_fixed_sections(40).unwrap_err();
        assert!(err.to_string().contains("overflow"), "{err}");
    }

    #[test]
    fn the_state_converts_at_rest_scaled_by_the_start_value() {
        let filter = tap_design();
        // Initial state is zeros, so the start value scales into zeros.
        assert_eq!(
            filter.to_fixed_state(5.0).unwrap(),
            vec![[0, 0], [0, 0], [0, 0]]
        );
        assert_eq!(
            filter.to_fixed_state(0.0).unwrap(),
            vec![[0, 0], [0, 0], [0, 0]]
        );
    }
}
