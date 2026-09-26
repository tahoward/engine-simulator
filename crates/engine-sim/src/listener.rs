//! What reaches the listener's ear: the direct path plus the ground reflection, each with air
//! absorption, and the reflection duller than the direct sound.

use crate::dsp::{Delay, OnePole};
use crate::math;
use crate::spec::ambient_sound_speed;

#[derive(Clone, Copy, Debug)]
pub struct ListenerGeometry {
    /// Horizontal distance, m.
    pub distance: f64,
    /// Ear height, m.
    pub mic_height: f64,
    /// Exhaust mouth height, m.
    pub source_height: f64,
    /// Ground reflection coefficient, 0..1.
    pub reflection: f64,
}

pub struct Listener {
    sample_rate: f64,
    delay: Delay,
    ground_loss: OnePole,
    air_direct: OnePole,
    air_ground: OnePole,
    direct_gain: f64,
    ground_gain: f64,
}

impl Listener {
    pub fn new(sample_rate: f64) -> Listener {
        let mut l = Listener {
            sample_rate,
            // 0.35 s of delay line covers any plausible path difference.
            delay: Delay::new((sample_rate * 0.35).ceil()),
            ground_loss: OnePole::default(),
            air_direct: OnePole::default(),
            air_ground: OnePole::default(),
            direct_gain: 1.0,
            ground_gain: 0.0,
        };
        l.set_geometry(ListenerGeometry { distance: 1.5, mic_height: 1.2, source_height: 0.35, reflection: 0.7 });
        l
    }

    pub fn set_geometry(&mut self, g: ListenerGeometry) {
        let c = ambient_sound_speed();
        let d = math::max(g.distance, 0.15);
        let hm = math::max(g.mic_height, 0.02);
        let hs = math::max(g.source_height, 0.02);

        let r_direct = math::hypot(&[d, hm - hs]);
        let r_ground = math::hypot(&[d, hm + hs]);

        self.direct_gain = 1.0 / r_direct;
        self.ground_gain = math::max(g.reflection, 0.0) / r_ground;
        self.delay.set_delay(((r_ground - r_direct) / c) * self.sample_rate);

        self.ground_loss.set_cutoff(2600.0, self.sample_rate);
        self.air_direct.set_cutoff(air_cutoff_hz(r_direct), self.sample_rate);
        self.air_ground.set_cutoff(air_cutoff_hz(r_ground), self.sample_rate);
    }

    /// Radiated pressure referred to 1 m, Pa, to pressure at the ear, Pa.
    #[inline]
    pub fn process(&mut self, source: f64) -> f64 {
        let direct = self.air_direct.process(source) * self.direct_gain;
        let bounced = self.delay.process(source);
        let ground = self.air_ground.process(self.ground_loss.process(bounced)) * self.ground_gain;
        direct + ground
    }

    pub fn reset(&mut self) {
        self.delay.reset();
        self.ground_loss.reset();
        self.air_direct.reset();
        self.air_ground.reset();
    }
}

/// One-pole corner, Hz, approximating atmospheric absorption over `metres`: ISO 9613-1's 0.11 dB/m
/// at 10 kHz, as a corner falling as `1/sqrt(r)`.
fn air_cutoff_hz(metres: f64) -> f64 {
    let corner_at_1m = 1e4 * math::sqrt(4.34 / 0.11);
    corner_at_1m / math::sqrt(math::max(metres, 0.2))
}
