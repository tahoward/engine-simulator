//! Shared helpers for the physics tests: the web app's presets, and a little spectral analysis.
//!
//! Each test file is its own crate and uses only some of these.
#![allow(dead_code)]

use engine_sim::spec::{EngineConfig, EngineSpec, LaunchConfig, PipeSegment};
use serde::Deserialize;
use std::sync::OnceLock;

pub const FS: f64 = 48000.0;

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Presets {
    pub default_engine: EngineSpec,
    pub default_config: EngineConfig,
    pub default_collector: Vec<PipeSegment>,
    pub engine_presets: Vec<EnginePreset>,
    pub pipe_presets: Vec<PipePreset>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct EnginePreset {
    pub name: String,
    /// The fields the preset sets, over the defaults.
    pub engine: serde_json::Value,
    /// The whole config the app loads for it.
    pub config: EngineConfig,
    /// The car and gearing `fitLaunch` sizes for it.
    pub launch: LaunchConfig,
    pub fitted_exhaust: FittedExhaust,
}

#[derive(Deserialize)]
pub struct FittedExhaust {
    pub pipe: Vec<PipeSegment>,
    pub collector: Vec<PipeSegment>,
}

#[derive(Deserialize)]
pub struct PipePreset {
    pub name: String,
    pub description: String,
    pub segments: Vec<PipeSegment>,
}

/// The web app's presets, from `tests/fixtures/presets.json` (`npm run export:presets` in apps/web).
pub fn presets() -> &'static Presets {
    static P: OnceLock<Presets> = OnceLock::new();
    P.get_or_init(|| {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/presets.json");
        serde_json::from_str(&std::fs::read_to_string(path).expect("presets fixture")).expect("presets parse")
    })
}

/// `defaultConfig()`: the default single on the megaphone.
pub fn default_config() -> EngineConfig {
    presets().default_config.clone()
}

/// The engine preset called `name`.
pub fn engine_preset(name: &str) -> &'static EnginePreset {
    presets().engine_presets.iter().find(|p| p.name == name).unwrap_or_else(|| panic!("no engine preset {name:?}"))
}

/// `PIPE_PRESETS[i].build()`.
pub fn pipe_preset(i: usize) -> Vec<PipeSegment> {
    presets().pipe_presets[i].segments.clone()
}

/// `spec` with the fields in `patch`, a JSON object in the web app's shape, replaced.
pub fn with(spec: &EngineSpec, patch: serde_json::Value) -> EngineSpec {
    spec.merged(&patch).expect("patch applies")
}

pub fn rms(buf: &[f32]) -> f64 {
    let s: f64 = buf.iter().map(|&v| v as f64 * v as f64).sum();
    (s / buf.len() as f64).sqrt()
}

/// In-place iterative radix-2 FFT. Lengths must match and be a power of two.
pub fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    assert!(n == im.len() && n.is_power_of_two(), "fft: length must match and be a power of two");
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let ang = (-2.0 * std::f64::consts::PI) / len as f64;
        let (w_re, w_im) = (ang.cos(), ang.sin());
        for i in (0..n).step_by(len) {
            let (mut cur_re, mut cur_im) = (1.0, 0.0);
            for k in 0..len / 2 {
                let (u_re, u_im) = (re[i + k], im[i + k]);
                let (a, b) = (re[i + k + len / 2], im[i + k + len / 2]);
                let v_re = a * cur_re - b * cur_im;
                let v_im = a * cur_im + b * cur_re;
                re[i + k] = u_re + v_re;
                im[i + k] = u_im + v_im;
                re[i + k + len / 2] = u_re - v_re;
                im[i + k + len / 2] = u_im - v_im;
                let next_re = cur_re * w_re - cur_im * w_im;
                cur_im = cur_re * w_im + cur_im * w_re;
                cur_re = next_re;
            }
        }
        len <<= 1;
    }
}

/// Magnitude spectrum of `signal`, zero-padded or truncated to `size`: bins 0..=size/2.
pub fn magnitude_spectrum(signal: &[f32], size: usize) -> Vec<f64> {
    let mut re = vec![0.0; size];
    let mut im = vec![0.0; size];
    for (r, &s) in re.iter_mut().zip(signal) {
        *r = s as f64;
    }
    fft(&mut re, &mut im);
    (0..size / 2 + 1).map(|i| re[i].hypot(im[i])).collect()
}

/// Hann window.
pub fn hann(signal: &[f32]) -> Vec<f32> {
    let n = signal.len();
    signal
        .iter()
        .enumerate()
        .map(|(i, &v)| {
            (v as f64 * 0.5 * (1.0 - ((2.0 * std::f64::consts::PI * i as f64) / (n as f64 - 1.0)).cos())) as f32
        })
        .collect()
}

#[derive(Clone, Copy, Debug)]
pub struct Peak {
    pub hz: f64,
    pub mag: f64,
}

/// Local maxima of a magnitude spectrum between `lo_hz` and `hi_hz`, strongest first, with
/// parabolically interpolated frequencies.
pub fn find_peaks(mag: &[f64], sample_rate: f64, size: usize, lo_hz: f64, hi_hz: f64, rel_threshold: f64) -> Vec<Peak> {
    let bin_hz = sample_rate / size as f64;
    let lo = ((lo_hz / bin_hz).floor() as usize).max(1);
    let hi = ((hi_hz / bin_hz).ceil() as usize).min(mag.len() - 2);
    let mut max_mag = 0.0f64;
    for &m in &mag[lo..=hi] {
        max_mag = max_mag.max(m);
    }
    let mut peaks = Vec::new();
    for i in lo..=hi {
        let m = mag[i];
        if m < max_mag * rel_threshold || m <= mag[i - 1] || m < mag[i + 1] {
            continue;
        }
        let (a, b, c) = (mag[i - 1], m, mag[i + 1]);
        let denom = a - 2.0 * b + c;
        let shift = if denom == 0.0 { 0.0 } else { (0.5 * (a - c)) / denom };
        peaks.push(Peak { hz: (i as f64 + shift) * bin_hz, mag: m });
    }
    peaks.sort_by(|x, y| y.mag.partial_cmp(&x.mag).unwrap());
    peaks
}

/// Energy in a band `width_hz` wide around `hz`.
pub fn band_energy(mag: &[f64], sample_rate: f64, size: usize, hz: f64, width_hz: f64) -> f64 {
    let bin_hz = sample_rate / size as f64;
    let lo = ((hz - width_hz / 2.0) / bin_hz).floor().max(0.0) as usize;
    let hi = (((hz + width_hz / 2.0) / bin_hz).ceil() as usize).min(mag.len() - 1);
    mag[lo..=hi].iter().map(|m| m * m).sum()
}
