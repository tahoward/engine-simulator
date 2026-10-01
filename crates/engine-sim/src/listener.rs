//! What reaches the listener's ear from each place the engine makes its sound: each tailpipe, the
//! intake, the casing. Each is heard along its own direct path and its own reflection off the ground,
//! both with air absorption, and the reflection duller than the direct sound.
//!
//! Paths are timed against the shortest of them, so only the differences between them delay anything.
//! The ear can move, and when it does every path glides to its new length and level over
//! `GLIDE_S`, as a moving ear hears them, rather than jumping and clicking.

use serde::{Deserialize, Serialize};

use crate::dsp::{Delay, OnePole};
use crate::math;
use crate::spec::ambient_sound_speed;

/// A point in the drawn engine's frame, m: x across the crank, y up, z along it, rearwards.
pub type Vec3 = [f64; 3];

/// Where the engine makes its sound, as the app draws it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SoundSources {
    /// Each tailpipe's outlet, by the duct that ends there.
    #[serde(default)]
    pub mouths: Vec<MouthPlace>,
    /// Where the engine draws its air.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub intake: Option<Vec3>,
    /// The middle of the engine, where its casing radiates from.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub engine: Option<Vec3>,
    /// Where the turbochargers are.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub turbo: Option<Vec3>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MouthPlace {
    pub duct: String,
    pub position: Vec3,
}

/// How long a path takes to glide to a new length and level, s.
const GLIDE_S: f64 = 0.05;

/// Longest difference between two paths, s: 34 m, far more than the listener's room to move gives.
const MAX_DIFFERENCE_S: f64 = 0.1;

/// Closest the ear may come to a source, m. Nearer, its level would run away as 1/r.
const MIN_RANGE: f64 = 0.25;

/// A value moving towards a target as a one-pole lowpass.
#[derive(Clone, Copy, Debug, Default)]
struct Glide {
    value: f64,
    target: f64,
}

impl Glide {
    #[inline]
    fn next(&mut self, c: f64) -> f64 {
        self.value += c * (self.target - self.value);
        self.value
    }
}

/// One source's way to the ear: the direct sound and the ground's reflection, read off one delay line.
struct Path {
    line: Delay,
    direct_delay: Glide,
    ground_delay: Glide,
    direct_gain: Glide,
    ground_gain: Glide,
    ground_loss: OnePole,
    air_direct: OnePole,
    air_ground: OnePole,
}

pub struct Listener {
    sample_rate: f64,
    paths: Vec<Path>,
    glide_c: f64,
}

impl Listener {
    pub fn new(sample_rate: f64) -> Listener {
        Listener { sample_rate, paths: Vec::new(), glide_c: 1.0 - math::exp(-1.0 / (GLIDE_S * sample_rate)) }
    }

    /// Put the ear at `ear` and the sources at `places`, above ground at height `ground` that reflects
    /// `reflection` of what reaches it. With `snap` the paths take their new lengths and levels at once,
    /// as they do when there are not yet as many paths as places; otherwise they glide there.
    pub fn set_geometry(&mut self, ear: Vec3, places: &[Vec3], ground: f64, reflection: f64, snap: bool) {
        let snap = snap || self.paths.len() != places.len();
        if self.paths.len() != places.len() {
            let len = (MAX_DIFFERENCE_S * self.sample_rate).ceil() + 4.0;
            self.paths = places
                .iter()
                .map(|_| Path {
                    line: Delay::new(len),
                    direct_delay: Glide::default(),
                    ground_delay: Glide::default(),
                    direct_gain: Glide::default(),
                    ground_gain: Glide::default(),
                    ground_loss: OnePole::default(),
                    air_direct: OnePole::default(),
                    air_ground: OnePole::default(),
                })
                .collect();
        }
        let c = ambient_sound_speed();
        let ear_height = math::max(ear[1] - ground, 0.02);
        let ranges: Vec<(f64, f64)> = places
            .iter()
            .map(|p| {
                let height = math::max(p[1] - ground, 0.02);
                let (dx, dz) = (ear[0] - p[0], ear[2] - p[2]);
                let direct = math::max(math::hypot(&[dx, ear_height - height, dz]), MIN_RANGE);
                let bounced = math::max(math::hypot(&[dx, ear_height + height, dz]), direct);
                (direct, bounced)
            })
            .collect();
        let nearest = ranges.iter().map(|r| r.0).fold(f64::INFINITY, f64::min);
        let most = MAX_DIFFERENCE_S * self.sample_rate;
        for (path, &(direct, bounced)) in self.paths.iter_mut().zip(&ranges) {
            path.direct_delay.target = math::min(((direct - nearest) / c) * self.sample_rate, most);
            path.ground_delay.target = math::min(((bounced - nearest) / c) * self.sample_rate, most);
            path.direct_gain.target = 1.0 / direct;
            path.ground_gain.target = math::max(reflection, 0.0) / bounced;
            path.ground_loss.set_cutoff(2600.0, self.sample_rate);
            path.air_direct.set_cutoff(air_cutoff_hz(direct), self.sample_rate);
            path.air_ground.set_cutoff(air_cutoff_hz(bounced), self.sample_rate);
            if snap {
                for g in [&mut path.direct_delay, &mut path.ground_delay, &mut path.direct_gain, &mut path.ground_gain]
                {
                    g.value = g.target;
                }
            }
        }
    }

    /// Radiated pressure from source `i`, referred to 1 m, Pa, to its pressure at the ear, Pa.
    #[inline]
    pub fn process(&mut self, i: usize, source: f64) -> f64 {
        let c = self.glide_c;
        let Some(p) = self.paths.get_mut(i) else { return 0.0 };
        p.line.push(source);
        let direct = p.line.tap(p.direct_delay.next(c));
        let bounced = p.line.tap(p.ground_delay.next(c));
        let direct = p.air_direct.process(direct) * p.direct_gain.next(c);
        let ground = p.air_ground.process(p.ground_loss.process(bounced)) * p.ground_gain.next(c);
        direct + ground
    }
}

/// One-pole corner, Hz, approximating atmospheric absorption over `metres`: ISO 9613-1's 0.11 dB/m
/// at 10 kHz, as a corner falling as `1/sqrt(r)`.
fn air_cutoff_hz(metres: f64) -> f64 {
    let corner_at_1m = 1e4 * math::sqrt(4.34 / 0.11);
    corner_at_1m / math::sqrt(math::max(metres, 0.2))
}
