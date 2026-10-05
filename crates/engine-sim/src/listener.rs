//! What reaches the listener's ear from each place the engine makes its sound: each tailpipe, the
//! intake, the casing. Each is heard along its own direct path and its own reflection off the ground,
//! both with air absorption, and the reflection duller than the direct sound. In a room, each is
//! heard off its four walls and its ceiling too, duller again: the first reflections, which arrive
//! before the reverberation (`room`) has built up.
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
    /// Where the engine draws its air: its snorkel's mouth.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub intake: Option<Vec3>,
    /// Where dual plenums' other inlet tract draws its air: its snorkel's mouth.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub second_intake: Option<Vec3>,
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

/// Longest difference between two paths in a room, s: 86 m, so a wall that far gives a reflection.
/// One farther off is too faint to matter.
const MAX_ROOM_DIFFERENCE_S: f64 = 0.25;

/// The walls and ceiling of a room, in the sources' frame, m, and what they do to the sound.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Walls {
    /// Where the side walls stand across the car, and the end walls along it.
    pub x: [f64; 2],
    pub z: [f64; 2],
    pub ceiling: f64,
    /// The share of the pressure arriving at a wall that it sends back.
    pub reflection: f64,
    /// Corner above which a reflection is duller than what reached the wall, Hz.
    pub corner_hz: f64,
}

impl Walls {
    /// `p` brought inside, `margin` from every wall and the ceiling, and above `ground`.
    pub fn inside(&self, p: Vec3, ground: f64, margin: f64) -> Vec3 {
        let within = |v: f64, lo: f64, hi: f64| {
            if lo + margin < hi - margin { math::clamp(v, lo + margin, hi - margin) } else { (lo + hi) / 2.0 }
        };
        [within(p[0], self.x[0], self.x[1]), within(p[1], ground, self.ceiling), within(p[2], self.z[0], self.z[1])]
    }

    /// `p` mirrored in each wall and in the ceiling.
    fn images(&self, p: Vec3) -> [Vec3; WALLS] {
        [
            [2.0 * self.x[0] - p[0], p[1], p[2]],
            [2.0 * self.x[1] - p[0], p[1], p[2]],
            [p[0], p[1], 2.0 * self.z[0] - p[2]],
            [p[0], p[1], 2.0 * self.z[1] - p[2]],
            [p[0], 2.0 * self.ceiling - p[1], p[2]],
        ]
    }
}

/// The four walls and the ceiling. The floor is the ground.
const WALLS: usize = 5;

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
// On cache lines of its own, as each may be stepped on a thread of its own.
#[repr(align(128))]
struct Path {
    line: Delay,
    direct_delay: Glide,
    ground_delay: Glide,
    direct_gain: Glide,
    ground_gain: Glide,
    ground_loss: OnePole,
    air_direct: OnePole,
    air_ground: OnePole,
    /// Each wall's reflection, and what the walls and the air between do to them.
    wall_delay: [Glide; WALLS],
    wall_gain: [Glide; WALLS],
    wall_loss: OnePole,
    air_walls: OnePole,
    /// Whether the walls are heard: from when a room is given until its reflections have faded out.
    walls_on: bool,
    /// What it was given last, until `Listener::take_sources` reads it.
    source: f64,
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
    /// `reflection` of what reaches it, and inside `walls` if there are any. With `snap` the paths take
    /// their new lengths and levels at once, as they do when there are not yet as many paths as places
    /// or when a room first needs them longer; otherwise they glide there.
    pub fn set_geometry(
        &mut self,
        ear: Vec3,
        places: &[Vec3],
        ground: f64,
        reflection: f64,
        walls: Option<&Walls>,
        snap: bool,
    ) {
        // Only ever lengthened, so leaving a room does not cut off what is on its way.
        let longest =
            if walls.is_some() || self.paths.first().is_some_and(|p| p.line.capacity() as f64 > self.line_len(false)) {
                self.line_len(true)
            } else {
                self.line_len(false)
            };
        let resize = self.paths.len() != places.len()
            || self.paths.first().is_some_and(|p| (p.line.capacity() as f64) < longest);
        let snap = snap || resize;
        if resize {
            self.paths = places
                .iter()
                .map(|_| Path {
                    line: Delay::new(longest),
                    direct_delay: Glide::default(),
                    ground_delay: Glide::default(),
                    direct_gain: Glide::default(),
                    ground_gain: Glide::default(),
                    ground_loss: OnePole::default(),
                    air_direct: OnePole::default(),
                    air_ground: OnePole::default(),
                    wall_delay: [Glide::default(); WALLS],
                    wall_gain: [Glide::default(); WALLS],
                    wall_loss: OnePole::default(),
                    air_walls: OnePole::default(),
                    walls_on: false,
                    source: 0.0,
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
        let most_wall = MAX_ROOM_DIFFERENCE_S * self.sample_rate;
        for ((path, &(direct, bounced)), place) in self.paths.iter_mut().zip(&ranges).zip(places) {
            path.direct_delay.target = math::min(((direct - nearest) / c) * self.sample_rate, most);
            path.ground_delay.target = math::min(((bounced - nearest) / c) * self.sample_rate, most);
            path.direct_gain.target = 1.0 / direct;
            path.ground_gain.target = math::max(reflection, 0.0) / bounced;
            path.ground_loss.set_cutoff(2600.0, self.sample_rate);
            path.air_direct.set_cutoff(air_cutoff_hz(direct), self.sample_rate);
            path.air_ground.set_cutoff(air_cutoff_hz(bounced), self.sample_rate);
            match walls {
                Some(w) => {
                    let place = w.inside(*place, ground, 0.0);
                    let mut mean = 0.0;
                    for (k, image) in w.images(place).iter().enumerate() {
                        let range =
                            math::max(math::hypot(&[ear[0] - image[0], ear[1] - image[1], ear[2] - image[2]]), direct);
                        let delay = ((range - nearest) / c) * self.sample_rate;
                        // Beyond what the line holds, a reflection that faint is left out.
                        let heard = delay <= most_wall;
                        path.wall_delay[k].target = math::min(delay, most_wall);
                        path.wall_gain[k].target = if heard { w.reflection / range } else { 0.0 };
                        mean += range / WALLS as f64;
                    }
                    path.wall_loss.set_cutoff(w.corner_hz, self.sample_rate);
                    path.air_walls.set_cutoff(air_cutoff_hz(mean), self.sample_rate);
                    path.walls_on = true;
                }
                None => {
                    for g in path.wall_gain.iter_mut() {
                        g.target = 0.0;
                    }
                }
            }
            if snap {
                for g in [&mut path.direct_delay, &mut path.ground_delay, &mut path.direct_gain, &mut path.ground_gain]
                    .into_iter()
                    .chain(path.wall_delay.iter_mut())
                    .chain(path.wall_gain.iter_mut())
                {
                    g.value = g.target;
                }
            }
        }
    }

    /// Each path's delay line, samples: long enough for a room's walls or not.
    fn line_len(&self, room: bool) -> f64 {
        let most = if room { MAX_ROOM_DIFFERENCE_S } else { MAX_DIFFERENCE_S };
        (most * self.sample_rate).ceil() + 4.0
    }

    /// The sum of what every path has been given since this was last called, each referred to 1 m, Pa:
    /// what the room's reverberation is fed.
    pub fn take_sources(&mut self) -> f64 {
        let mut sum = 0.0;
        for p in self.paths.iter_mut() {
            sum += p.source;
            p.source = 0.0;
        }
        sum
    }

    /// Radiated pressure from source `i`, referred to 1 m, Pa, to its pressure at the ear, Pa.
    #[inline]
    pub fn process(&mut self, i: usize, source: f64) -> f64 {
        match self.paths.get_mut(i) {
            Some(p) => p.process(self.glide_c, source),
            None => 0.0,
        }
    }

    /// Every path, to be processed apart, each with `ListenerPaths::process`.
    pub fn paths(&mut self) -> ListenerPaths<'_> {
        ListenerPaths {
            paths: self.paths.as_mut_ptr(),
            count: self.paths.len(),
            glide_c: self.glide_c,
            _paths: std::marker::PhantomData,
        }
    }
}

/// The listener's paths, from `Listener::paths`, each reached only through its own index.
pub struct ListenerPaths<'a> {
    paths: *mut Path,
    count: usize,
    glide_c: f64,
    _paths: std::marker::PhantomData<&'a mut Listener>,
}

// Each path is reached only through `process(i)`, which each index is given to by one thread.
unsafe impl Sync for ListenerPaths<'_> {}
unsafe impl Send for ListenerPaths<'_> {}

impl ListenerPaths<'_> {
    /// `Listener::process` for path `i`.
    ///
    /// # Safety
    ///
    /// Only one thread may process a given path at a time.
    pub unsafe fn process(&self, i: usize, source: f64) -> f64 {
        if i >= self.count {
            return 0.0;
        }
        unsafe { &mut *self.paths.add(i) }.process(self.glide_c, source)
    }
}

impl Path {
    fn process(&mut self, c: f64, source: f64) -> f64 {
        self.source = source;
        self.line.push(source);
        let direct = self.line.tap(self.direct_delay.next(c));
        let bounced = self.line.tap(self.ground_delay.next(c));
        let direct = self.air_direct.process(direct) * self.direct_gain.next(c);
        let ground = self.air_ground.process(self.ground_loss.process(bounced)) * self.ground_gain.next(c);
        if !self.walls_on {
            return direct + ground;
        }
        let mut walls = 0.0;
        let mut fading = true;
        for (delay, gain) in self.wall_delay.iter_mut().zip(self.wall_gain.iter_mut()) {
            walls += self.line.tap(delay.next(c)) * gain.next(c);
            fading &= gain.target == 0.0 && gain.value < 1e-6;
        }
        // Out of the room, once its reflections have faded.
        if fading {
            self.walls_on = false;
        }
        direct + ground + self.air_walls.process(self.wall_loss.process(walls))
    }
}

/// One-pole corner, Hz, approximating atmospheric absorption over `metres`: ISO 9613-1's 0.11 dB/m
/// at 10 kHz, as a corner falling as `1/sqrt(r)`.
fn air_cutoff_hz(metres: f64) -> f64 {
    let corner_at_1m = 1e4 * math::sqrt(4.34 / 0.11);
    corner_at_1m / math::sqrt(math::max(metres, 0.2))
}
