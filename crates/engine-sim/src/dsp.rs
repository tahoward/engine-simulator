//! Small DSP helpers. Nothing here allocates after construction.

use crate::math::{self, PI};

/// One-pole lowpass, `y += c * (x - y)`.
#[derive(Clone, Debug)]
pub struct OnePole {
    y: f64,
    pub c: f64,
}

impl Default for OnePole {
    fn default() -> Self {
        OnePole { y: 0.0, c: 0.5 }
    }
}

impl OnePole {
    #[inline]
    pub fn process(&mut self, x: f64) -> f64 {
        self.y += self.c * (x - self.y);
        self.y
    }

    /// Set the cutoff in Hz.
    pub fn set_cutoff(&mut self, hz: f64, sample_rate: f64) {
        self.c = 1.0 - math::exp((-2.0 * PI * hz) / sample_rate);
    }

    pub fn reset(&mut self) {
        self.y = 0.0;
    }
}

/// One structural mode: a two-pole band-pass that rings at `hz` with the given decay.
///
/// Band-pass rather than all-pole: a vibrating surface radiates in proportion to its acceleration,
/// so a mode radiates nothing at DC. The zeros at DC and Nyquist, `(1 - z^-2) / 2`, say that.
#[derive(Clone, Debug, Default)]
pub struct Resonator {
    y1: f64,
    y2: f64,
    x1: f64,
    x2: f64,
    a1: f64,
    a2: f64,
}

impl Resonator {
    pub fn new(hz: f64, q: f64, sample_rate: f64) -> Resonator {
        let mut r = Resonator::default();
        r.set(hz, q, sample_rate);
        r
    }

    pub fn set(&mut self, hz: f64, q: f64, sample_rate: f64) {
        let r = math::exp((-PI * hz) / (q * sample_rate));
        let theta = (2.0 * PI * hz) / sample_rate;
        self.a1 = 2.0 * r * math::cos(theta);
        self.a2 = -r * r;
    }

    /// The frequency it rings at, Hz, read back from its poles.
    pub fn frequency(&self, sample_rate: f64) -> f64 {
        let r = math::sqrt(-self.a2);
        (math::acos(self.a1 / (2.0 * r)) * sample_rate) / (2.0 * PI)
    }

    /// Excite with `x`; the peak ring amplitude is approximately `x`.
    #[inline]
    pub fn process(&mut self, x: f64) -> f64 {
        let y = 0.5 * (x - self.x2) + self.a1 * self.y1 + self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }

    pub fn reset(&mut self) {
        self.y1 = 0.0;
        self.y2 = 0.0;
        self.x1 = 0.0;
        self.x2 = 0.0;
    }
}

/// A mechanical impact of finite duration: a raised-cosine force pulse of unit area.
///
/// Nothing in an engine hits anything else instantaneously, and the contact time is what band-limits
/// the noise an impact makes.
#[derive(Clone, Debug)]
pub struct Impact {
    window: Vec<f64>,
    pos: usize,
    amp: f64,
}

#[allow(clippy::should_implement_trait)]
impl Impact {
    pub fn new(seconds: f64, sample_rate: f64) -> Impact {
        let n = math::max(2.0, math::round(seconds * sample_rate)) as usize;
        let mut window = vec![0.0; n];
        let mut sum = 0.0;
        for (i, w) in window.iter_mut().enumerate() {
            *w = 0.5 * (1.0 - math::cos((2.0 * PI * (i as f64 + 0.5)) / n as f64));
            sum += *w;
        }
        for w in window.iter_mut() {
            *w /= sum;
        }
        Impact { window, pos: n, amp: 0.0 }
    }

    /// Start a new impact. Retriggering mid-pulse takes the louder of the two.
    pub fn trigger(&mut self, amp: f64) {
        if self.pos < self.window.len() && amp <= self.amp {
            return;
        }
        self.amp = amp;
        self.pos = 0;
    }

    /// Next sample of the force pulse, zero when idle.
    #[inline]
    pub fn next(&mut self) -> f64 {
        if self.pos >= self.window.len() {
            return 0.0;
        }
        let v = self.amp * self.window[self.pos];
        self.pos += 1;
        v
    }

    pub fn reset(&mut self) {
        self.pos = self.window.len();
        self.amp = 0.0;
    }
}

/// Deterministic white noise: xorshift32.
#[derive(Clone, Debug)]
pub struct Noise {
    s: u32,
}

impl Default for Noise {
    fn default() -> Self {
        Noise::new(0x2f6e2b1 as f64)
    }
}

#[allow(clippy::should_implement_trait)]
impl Noise {
    /// Seeded from a number, truncated and taken modulo 2^32.
    pub fn new(seed: f64) -> Noise {
        Noise { s: math::to_uint32(seed) }
    }

    /// Uniform in [-1, 1).
    #[inline]
    pub fn next(&mut self) -> f64 {
        let mut x = self.s;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.s = x;
        self.s as f64 / 2_147_483_648.0 - 1.0
    }

    /// Normally distributed, mean 0, standard deviation 1: Box-Muller, with a floor keeping `log`
    /// away from zero.
    pub fn gaussian(&mut self) -> f64 {
        let u1 = math::max((self.next() + 1.0) / 2.0, 1e-7);
        let u2 = (self.next() + 1.0) / 2.0;
        math::sqrt(-2.0 * math::log(u1)) * math::cos(2.0 * PI * u2)
    }
}

/// Fractional-delay line with linear interpolation. The buffer holds `f32`.
#[derive(Clone, Debug)]
pub struct Delay {
    buf: Vec<f32>,
    write: usize,
    delay_samples: f64,
}

impl Delay {
    pub fn new(max_samples: f64) -> Delay {
        let len = math::max(2.0, max_samples.ceil()) as usize;
        Delay { buf: vec![0.0; len], write: 0, delay_samples: 0.0 }
    }

    pub fn set_delay(&mut self, samples: f64) {
        self.delay_samples = math::clamp(samples, 0.0, self.buf.len() as f64 - 2.0);
    }

    #[inline]
    pub fn process(&mut self, x: f64) -> f64 {
        let len = self.buf.len();
        self.buf[self.write] = x as f32;
        let read = self.write as f64 - self.delay_samples;
        let i = read.floor();
        let frac = read - i;
        let i = i as i64;
        let n = len as i64;
        let a = self.buf[i.rem_euclid(n) as usize] as f64;
        let b = self.buf[(i + 1).rem_euclid(n) as usize] as f64;
        self.write = (self.write + 1) % len;
        a + (b - a) * frac
    }

    pub fn reset(&mut self) {
        self.buf.fill(0.0);
        self.write = 0;
    }
}

/// Gentle saturation on the master output: a cubic knee joining the clamp at 1.5.
#[inline]
pub fn soft_clip(x: f64) -> f64 {
    if x > 1.5 {
        return 1.0;
    }
    if x < -1.5 {
        return -1.0;
    }
    x - (x * x * x) / 6.75
}

/// Wraps a crank angle into [0, 720).
#[inline]
pub fn wrap_cycle(deg: f64) -> f64 {
    let mut d = deg % 720.0;
    if d < 0.0 {
        d += 720.0;
    }
    d
}

/// How far `deg` is past `ref_`, the shorter way round a 720-degree cycle, in (-360, 360].
#[inline]
pub fn cycle_delta(deg: f64, ref_: f64) -> f64 {
    let mut d = (deg - ref_) % 720.0;
    if d < 0.0 {
        d += 720.0;
    }
    if d > 360.0 {
        d -= 720.0;
    }
    d
}

/// Position within an event window that may wrap past 720, as a 0..1 fraction, or -1 outside it.
#[inline]
pub fn window_phase(deg: f64, open: f64, close: f64) -> f64 {
    let mut span = (close - open) % 720.0;
    if span <= 0.0 {
        span += 720.0;
    }
    let mut rel = (deg - open) % 720.0;
    if rel < 0.0 {
        rel += 720.0;
    }
    if rel <= span { rel / span } else { -1.0 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn noise_is_xorshift32() {
        // xorshift32 from the default seed.
        let mut n = Noise::default();
        let first = n.next();
        let mut x: u32 = 0x2f6e2b1;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        assert_eq!(first, x as f64 / 2_147_483_648.0 - 1.0);
    }

    #[test]
    fn resonator_rings_where_it_is_set() {
        let r = Resonator::new(1000.0, 20.0, 48000.0);
        assert!((r.frequency(48000.0) - 1000.0).abs() < 5.0);
    }

    #[test]
    fn delay_delays() {
        let mut d = Delay::new(16.0);
        d.set_delay(3.0);
        let out: Vec<f64> = (0..6).map(|i| d.process(if i == 0 { 1.0 } else { 0.0 })).collect();
        assert_eq!(out, vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0]);
    }
}
