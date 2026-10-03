//! Far-field radiation from an exhaust mouth.
//!
//! A small mouth is a monopole, radiating in proportion to the rate of change of the volume flow
//! leaving it. That holds only while the mouth is small against a wavelength; above `wc = 2c/a` it
//! radiates like a piston and its efficiency stops climbing. So the transfer is a first-order
//! highpass at `wc`, the bilinear transform of `s / (s + wc)`, scaled by `rho wc / (4 pi)`.

use crate::dsp::Noise;
use crate::math::{self, PI, clamp};
use crate::spec::{ambient_sound_speed, density, gas};

/// Lighthill's constant: the share of a jet's kinetic power `rho U^8 D^2 / c^5` it radiates as
/// sound. Measured jets give 0.3-1.2e-4.
const LIGHTHILL_K: f64 = 5e-5;

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

/// Sound pressure at 1 m, Pa RMS, of a jet of speed `u` (m/s) from a nozzle of diameter `d` (m), by
/// Lighthill's eighth-power law.
pub fn lighthill_pa(u: f64, d: f64) -> f64 {
    let rho = gas::P_AMB / (gas::R * gas::T_AMB);
    let c = math::sqrt(gas::GAMMA_AIR * gas::R * gas::T_AMB);
    let u2 = u * u;
    let power = LIGHTHILL_K * rho * u2 * u2 * u2 * u2 * d * d / (c * c * c * c * c);
    math::sqrt((power * rho * c) / (4.0 * PI))
}

/// The roar of the jet an exhaust mouth blows into the air: the turbulence of the gas leaving it,
/// which no one-dimensional duct can carry.
///
/// Lighthill's law on the mouth's speed of the moment, so it swells and dies with every pulse: the
/// eighth power puts nearly all of it at the tip of a blowdown, which is the fuzz on each beat of a
/// real exhaust. Lighthill's law is for a jet as dense as the air it meets, and a hot one is lighter;
/// but at the speeds an exhaust leaves at, well below the air's speed of sound, a hot jet is no
/// quieter than a cold one of the same speed. Air drawn in at the mouth makes no jet outside it.
/// The roar of the jet an exhaust mouth blows into the air: the turbulence of the gas leaving it,
/// which no one-dimensional duct can carry.
///
/// Lighthill's law on the mouth's speed of the moment, so it swells and dies with every pulse: the
/// eighth power puts nearly all of it at the tip of a blowdown, which is the fuzz on each beat of a
/// real exhaust. The jet is hot, so lighter than the air it meets, which the law takes as the density
/// ratio to the power SAE ARP876 gives: below about half the air's speed of sound a hot jet is the
/// louder, above it the quieter. Its spectrum is white noise shaped round the Strouhal peak,
/// `0.2 U / D`, falling 12 dB an octave above it as measured jets do. Air drawn in at the mouth makes
/// no jet outside it.
pub struct MouthJet {
    noise: Noise,
    lp1: f64,
    lp2: f64,
    hp: f64,
}

impl MouthJet {
    pub fn new(seed: f64) -> MouthJet {
        MouthJet { noise: Noise::new(seed), lp1: 0.0, lp2: 0.0, hp: 0.0 }
    }

    /// Volume flow leaving a mouth of `area`, m^2, of gas at `temp`, K, to the jet's sound at 1 m, Pa,
    /// at `level` of the real one.
    #[inline]
    pub fn process(&mut self, q: f64, area: f64, temp: f64, level: f64, sample_rate: f64) -> f64 {
        let area = math::max(area, 1e-6);
        let u = if level > 0.0 { q / area } else { 0.0 };
        if u <= 0.0 {
            self.lp1 *= 0.99;
            self.lp2 *= 0.99;
            self.hp *= 0.99;
            return 0.0;
        }
        let d = math::sqrt((4.0 * area) / PI);
        let m = math::pow(u / ambient_sound_speed(), 3.5);
        let omega = (3.0 * m) / (0.6 + m) - 1.0;
        let density_ratio = gas::T_AMB / math::max(temp, 0.5 * gas::T_AMB);
        let pa = level * lighthill_pa(u, d) * math::sqrt(math::pow(density_ratio, omega));

        let peak = clamp((0.2 * u) / d, 100.0, 0.4 * sample_rate);
        let c_lp = 1.0 - math::exp((-2.0 * PI * 2.0 * peak) / sample_rate);
        let c_hp = 1.0 - math::exp((-2.0 * PI * 0.5 * peak) / sample_rate);
        // Uniform noise has an RMS of 1/sqrt(3); two one-pole low-passes of pole `a` keep
        // `c^4 (1 + a^2) / (1 - a^2)^3` of its power.
        let r = (1.0 - c_lp) * (1.0 - c_lp);
        let c2 = c_lp * c_lp;
        let norm = math::sqrt((3.0 * (1.0 - r) * (1.0 - r) * (1.0 - r)) / (c2 * c2 * (1.0 + r)));
        self.lp1 += c_lp * (self.noise.next() - self.lp1);
        self.lp2 += c_lp * (self.lp1 - self.lp2);
        self.hp += c_hp * (self.lp2 - self.hp);
        (self.lp2 - self.hp) * norm * pa
    }
}
