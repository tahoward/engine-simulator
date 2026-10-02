//! Far-field radiation from an exhaust mouth.
//!
//! A small mouth is a monopole, radiating in proportion to the rate of change of the volume flow
//! leaving it. That holds only while the mouth is small against a wavelength; above `wc = 2c/a` it
//! radiates like a piston and its efficiency stops climbing. So the transfer is a first-order
//! highpass at `wc`, the bilinear transform of `s / (s + wc)`, scaled by `rho wc / (4 pi)`.

use crate::math::{self, PI};
use crate::spec::{density, gas};

// On cache lines of its own, as each may be stepped on a thread of its own.
#[repr(align(128))]
pub struct FarField {
    sample_rate: f64,
    x1: f64,
    y1: f64,
    b0: f64,
    a1: f64,
    scale: f64,
    /// One pole enforcing the band limit it is given: the plane-wave limit, above which the mouth no
    /// longer radiates a plane wave.
    plane_c: f64,
    plane1: f64,
}

impl FarField {
    pub fn new(sample_rate: f64, cutoff_rad: f64) -> FarField {
        let mut f = FarField { sample_rate, x1: 0.0, y1: 0.0, b0: 0.0, a1: 0.0, scale: 0.0, plane_c: 1.0, plane1: 0.0 };
        f.set_cutoff(cutoff_rad, f64::INFINITY);
        f
    }

    /// Called whenever the pipe is rebuilt and the mouth or duct size changes.
    pub fn set_cutoff(&mut self, cutoff_rad: f64, plane_wave_cutoff_rad: f64) {
        let wc = math::max(cutoff_rad, 100.0);
        let k = 2.0 * self.sample_rate;
        self.b0 = k / (k + wc);
        self.a1 = (k - wc) / (k + wc);
        self.scale = (density(gas::P_AMB, gas::T_AMB) * wc) / (4.0 * PI);

        let wp = math::min(plane_wave_cutoff_rad, PI * self.sample_rate * 0.9);
        self.plane_c = if wp.is_finite() { 1.0 - math::exp(-wp / self.sample_rate) } else { 1.0 };
    }

    /// Volume flow leaving the mouth, m^3/s, to radiated pressure referred to 1 m, Pa.
    #[inline]
    pub fn process(&mut self, q: f64) -> f64 {
        let y = self.b0 * (q - self.x1) + self.a1 * self.y1;
        self.x1 = q;
        self.y1 = y;
        let p = self.scale * y;
        self.plane1 += self.plane_c * (p - self.plane1);
        self.plane1
    }

    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.y1 = 0.0;
        self.plane1 = 0.0;
    }
}

/// Largest slope the warp's displacement may take, samples per sample: under 1, so the warp never
/// folds a front over into a shock, and it can raise a frequency at most `1 / (1 - x)` times.
const MAX_WARP_SLOPE: f64 = 0.7;

/// Largest displacement the warp may give a sample, s.
const MAX_WARP_S: f64 = 0.002;

/// Corner, Hz, of the mean the warp measures the mouth's particle velocity against.
const WARP_MEAN_HZ: f64 = 2.0;

/// Finite-amplitude steepening over the free run of pipe before a mouth.
///
/// A loud wave does not travel at one speed: each part of it moves at `c + beta u`, with
/// `beta = (gamma + 1) / 2`, so over a run `L` its crests gain `beta L u / c^2` on its troughs and its
/// fronts sharpen. The duct's grid carries that only as far up as it resolves, so this carries it on
/// above, as a time warp of the mouth flow: output sample `n` is the input from the instant `s` with
/// `s + D - d(s) = n`, where `d(s)` is the gain of the flow leaving at `s` and `D` is a fixed latency.
/// The mouth flow is held to the grid's band limit first, so the warp only sharpens what the grid
/// solved. Fronts sharpen most where the pipe is loud, cool and long, and barely at all at a quiet one
/// or after a muffler. On cache lines of its own, as each may be stepped on a thread of its own.
#[repr(align(128))]
pub struct Steepening {
    sample_rate: f64,
    flow: Vec<f64>,
    shift: Vec<f64>,
    mask: usize,
    write: usize,
    latency: f64,
    max_shift: f64,
    band_c: f64,
    band_y: f64,
    mean_c: f64,
    mean_u: f64,
    last_shift: f64,
    /// `beta L`, m: the free run scaled by the steepening coefficient.
    beta_run: f64,
}

impl Steepening {
    pub fn new(sample_rate: f64) -> Steepening {
        let max_shift = (MAX_WARP_S * sample_rate).ceil();
        let len = ((2.0 * max_shift) as usize + 8).next_power_of_two();
        Steepening {
            sample_rate,
            flow: vec![0.0; len],
            shift: vec![0.0; len],
            mask: len - 1,
            write: 0,
            latency: max_shift + 2.0,
            max_shift,
            band_c: 1.0,
            band_y: 0.0,
            mean_c: 1.0 - math::exp((-2.0 * PI * WARP_MEAN_HZ) / sample_rate),
            mean_u: 0.0,
            last_shift: 0.0,
            beta_run: 0.0,
        }
    }

    /// Set the free run of pipe before the mouth, m, and the grid's band limit, rad/s.
    pub fn set_duct(&mut self, free_run: f64, band_limit_rad: f64) {
        self.beta_run = ((gas::GAMMA_EXH + 1.0) / 2.0) * math::max(free_run, 0.0);
        let wb = math::min(band_limit_rad, PI * self.sample_rate * 0.9);
        self.band_c = if wb.is_finite() { 1.0 - math::exp(-wb / self.sample_rate) } else { 1.0 };
    }

    /// Volume flow leaving the mouth, m^3/s, through a mouth of `area`, m^2, of gas at `temp`, K, to
    /// the same flow as it leaves the end of a run long enough to sharpen it.
    #[inline]
    pub fn process(&mut self, q: f64, area: f64, temp: f64) -> f64 {
        self.band_y += self.band_c * (q - self.band_y);
        let q = self.band_y;

        let u = q / math::max(area, 1e-6);
        self.mean_u += self.mean_c * (u - self.mean_u);
        let inv_c2 = 1.0 / (gas::GAMMA_EXH * gas::R * math::max(temp, 150.0));
        let target = self.beta_run * (u - self.mean_u) * inv_c2 * self.sample_rate;
        let target = math::clamp(target, -self.max_shift, self.max_shift);
        let shift = math::clamp(target, self.last_shift - MAX_WARP_SLOPE, self.last_shift + MAX_WARP_SLOPE);
        self.last_shift = shift;

        self.write = (self.write + 1) & self.mask;
        self.flow[self.write] = q;
        self.shift[self.write] = shift;

        // The lag `t` back from the newest sample solves `t = D - d(n - t)`, a contraction since the
        // shift's slope is under one.
        let mut lag = self.latency;
        for _ in 0..6 {
            lag = self.latency - self.linear(&self.shift, lag);
        }
        self.hermite(lag)
    }

    #[inline]
    fn linear(&self, buf: &[f64], lag: f64) -> f64 {
        let i = lag.floor();
        let f = lag - i;
        let a = buf[self.write.wrapping_sub(i as usize) & self.mask];
        let b = buf[self.write.wrapping_sub(i as usize + 1) & self.mask];
        a + (b - a) * f
    }

    /// The flow `lag` samples back, by cubic Hermite interpolation.
    #[inline]
    fn hermite(&self, lag: f64) -> f64 {
        let i = lag.floor();
        let f = lag - i;
        let at = |k: isize| self.flow[self.write.wrapping_sub((i as isize + k) as usize) & self.mask];
        let (xm1, x0, x1, x2) = (at(-1), at(0), at(1), at(2));
        let c1 = 0.5 * (x1 - xm1);
        let c2 = xm1 - 2.5 * x0 + 2.0 * x1 - 0.5 * x2;
        let c3 = 0.5 * (x2 - xm1) + 1.5 * (x0 - x1);
        ((c3 * f + c2) * f + c1) * f + x0
    }

    pub fn reset(&mut self) {
        self.flow.fill(0.0);
        self.shift.fill(0.0);
        self.band_y = 0.0;
        self.mean_u = 0.0;
        self.last_shift = 0.0;
    }
}
