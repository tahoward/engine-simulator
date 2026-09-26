//! Far-field radiation from an exhaust mouth.
//!
//! A small mouth is a monopole, radiating in proportion to the rate of change of the volume flow
//! leaving it. That holds only while the mouth is small against a wavelength; above `wc = 2c/a` it
//! radiates like a piston and its efficiency stops climbing. So the transfer is a first-order
//! highpass at `wc`, the bilinear transform of `s / (s + wc)`, scaled by `rho wc / (4 pi)`.

use crate::math::{self, PI};
use crate::spec::{density, gas};

pub struct FarField {
    sample_rate: f64,
    x1: f64,
    y1: f64,
    b0: f64,
    a1: f64,
    scale: f64,
    /// One pole enforcing the band limit: the lower of the plane-wave and resolution limits.
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
