//! Finite-amplitude steepening over the free run of pipe before a mouth.
//!
//! A loud wave's crests outrun its troughs, so over a run of pipe its fronts sharpen and it gains
//! harmonics; a quiet one travels unchanged. The warp that stands for this must do nothing to a wave
//! it has no run to sharpen over, must grow harmonics with the amplitude, and must never fold a front
//! over into a shock.

use crate::common;

use common::FS;
use engine_sim::radiation::Steepening;
use std::f64::consts::PI;

const AREA: f64 = 0.002;
const TEMP: f64 = 800.0;
/// 48 kHz over 375 Hz: a whole number of samples per period, so a DFT over whole periods is exact.
const F0: f64 = 375.0;

/// The warp's output for a sine of particle velocity `u_peak`, m/s, at `F0`, settled.
fn warped_sine(run: f64, u_peak: f64) -> Vec<f64> {
    let mut s = Steepening::new(FS);
    s.set_duct(run, f64::INFINITY);
    let settle = FS as usize / 2;
    let period = (FS / F0) as usize;
    (0..settle + 64 * period)
        .map(|i| {
            let u = u_peak * (2.0 * PI * F0 * i as f64 / FS).sin();
            s.process(u * AREA, AREA, TEMP)
        })
        .skip(settle)
        .collect()
}

/// Amplitude of harmonic `k` of `F0` in `x`, which spans whole periods.
fn harmonic(x: &[f64], k: f64) -> f64 {
    let (mut re, mut im) = (0.0, 0.0);
    for (i, v) in x.iter().enumerate() {
        let ph = 2.0 * PI * k * F0 * i as f64 / FS;
        re += v * ph.cos();
        im += v * ph.sin();
    }
    2.0 * (re * re + im * im).sqrt() / x.len() as f64
}

/// Energy in harmonics 2 to 8, relative to the fundamental's, dB.
fn harmonic_level_db(x: &[f64]) -> f64 {
    let f1 = harmonic(x, 1.0);
    let rest: f64 = (2..=8).map(|k| harmonic(x, k as f64).powi(2)).sum();
    10.0 * (rest / (f1 * f1)).log10()
}

/// With no run to sharpen over, the flow passes unchanged but for a fixed latency and the band limit
/// every radiator applies just short of Nyquist.
#[test]
fn no_run_passes_the_flow_unchanged_but_for_its_latency() {
    let mut s = Steepening::new(FS);
    s.set_duct(0.0, f64::INFINITY);
    let input: Vec<f64> = (0..4000).map(|i| if i < 100 { 0.0 } else { (2.0 * PI * 200.0 * i as f64 / FS).sin() }).collect();
    let out: Vec<f64> = input.iter().map(|&q| s.process(q, AREA, TEMP)).collect();
    let lag = out.iter().position(|v| *v != 0.0).unwrap() - 100;
    assert!(lag > 0 && (lag as f64) < 0.01 * FS, "latency {lag} samples");
    for i in 1000..out.len() {
        assert!((out[i] - input[i - lag]).abs() < 5e-3, "sample {i}: {} against {}", out[i], input[i - lag]);
    }
}

/// A loud wave gains harmonics over its run; a quiet one barely does.
#[test]
fn a_loud_wave_sharpens_and_a_quiet_one_barely_does() {
    let quiet = harmonic_level_db(&warped_sine(1.0, 0.5));
    let loud = harmonic_level_db(&warped_sine(1.0, 60.0));
    assert!(quiet < -40.0, "quiet {quiet} dB");
    assert!(loud > -20.0, "loud {loud} dB");
}

/// The longer the run, the more the same wave sharpens.
#[test]
fn a_longer_run_sharpens_more() {
    let short = harmonic_level_db(&warped_sine(0.2, 20.0));
    let long = harmonic_level_db(&warped_sine(1.0, 20.0));
    assert!(long > short + 6.0, "short {short} dB, long {long} dB");
}

/// However hard it is driven, a front steepens only so far and never folds over.
#[test]
fn never_folds_a_front_into_a_shock() {
    let u_peak = 400.0;
    let out = warped_sine(3.0, u_peak);
    let q_peak = u_peak * AREA;
    let input_slope = q_peak * 2.0 * PI * F0 / FS;
    let steepest = out.windows(2).map(|w| (w[1] - w[0]).abs()).fold(0.0, f64::max);
    // At most `1 / (1 - 0.7)` times the input's steepest, with room for the interpolation.
    assert!(steepest < 4.0 * input_slope, "steepest step {steepest} against {input_slope}");
    let peak = out.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    assert!(peak < 1.05 * q_peak, "peak {peak} against {q_peak}");
}
