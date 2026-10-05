//! What reaches the listener's ear from each place the engine makes its sound: each tailpipe, the
//! intake, the casing. Each is heard along its own direct path and its own reflection off the ground,
//! both with air absorption, and the reflection duller than the direct sound. In a room, each is
//! heard off its four walls and its ceiling too, duller again: the first reflections, which arrive
//! before the reverberation (`room`) has built up.
//!
//! In stereo there are two ears, a head apart, each with its own paths, so a source off to one side
//! reaches the nearer ear first and louder; and the head shadows the far ear from it, taking its highs.
//! So it does each reflection, from the way that reflection arrives: off the ground below, off each wall
//! from that wall's side, off the ceiling from above.
//!
//! Paths are timed against the shortest of them, so only the differences between them delay anything.
//! The ear can move, and when it does every path glides to its new length and level over
//! `GLIDE_S`, as a moving ear hears them, rather than jumping and clicking.

use serde::{Deserialize, Serialize};

use crate::dsp::{Delay, OnePole};
use crate::math::{self, PI};
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

/// An average adult head's radius, m: half the distance between the ears.
pub const HEAD_RADIUS: f64 = 0.0875;

/// Brown and Duda's head shadow: the least its zero is put at, as a share of its pole, and the angle
/// from the ear's own side, rad, at which it is, behind which the sound bending round both sides of the
/// head brightens it again.
const SHADOW_LEAST: f64 = 0.1;
const SHADOW_DEEPEST: f64 = 5.0 * PI / 6.0;

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

/// What the head does to sound arriving at one ear: Brown and Duda's spherical head, a one-pole,
/// one-zero filter with its pole at `c / (2 a)`. From the ear's own side its zero sits an octave below
/// its pole, lifting the highs 6 dB as the head's face presses them back; round on the far side it sits
/// three octaves above, and the head's shadow takes 20 dB off them.
#[derive(Clone, Copy, Debug)]
pub(crate) struct HeadShadow {
    b0: f64,
    b1: f64,
    a1: f64,
    x1: f64,
    y1: f64,
}

impl Default for HeadShadow {
    fn default() -> Self {
        HeadShadow { b0: 1.0, b1: 0.0, a1: 0.0, x1: 0.0, y1: 0.0 }
    }
}

impl HeadShadow {
    /// For a sound arriving from `angle` off the ear's own side, rad.
    pub(crate) fn set(&mut self, angle: f64, sample_rate: f64) {
        let alpha = (1.0 + SHADOW_LEAST / 2.0) + (1.0 - SHADOW_LEAST / 2.0) * math::cos((angle / SHADOW_DEEPEST) * PI);
        let w = (2.0 * ambient_sound_speed()) / HEAD_RADIUS;
        let k = 2.0 * sample_rate;
        // `(w + alpha s) / (w + s)`, through the bilinear transform.
        self.b0 = (w + alpha * k) / (w + k);
        self.b1 = (w - alpha * k) / (w + k);
        self.a1 = (w - k) / (w + k);
    }

    #[inline]
    pub(crate) fn process(&mut self, x: f64) -> f64 {
        let y = self.b0 * x + self.b1 * self.x1 - self.a1 * self.y1;
        self.x1 = x;
        self.y1 = y;
        y
    }
}

/// How much later a sound from far off arrives at an ear than at the middle of the head, s, from `angle`
/// off the ear's own side, rad: Woodworth's, round the sphere to the far side. Negative from its own side.
pub(crate) fn ear_delay_s(angle: f64) -> f64 {
    let lead = if angle < PI / 2.0 { -math::cos(angle) } else { angle - PI / 2.0 };
    (HEAD_RADIUS / ambient_sound_speed()) * lead
}

/// The angle off `axis`, a unit vector, that `to` points, rad.
pub(crate) fn angle_off(to: Vec3, axis: Vec3) -> f64 {
    let len = math::max(math::hypot(&to), 1e-9);
    let cos = (to[0] * axis[0] + to[1] * axis[1] + to[2] * axis[2]) / len;
    math::acos(math::clamp(cos, -1.0, 1.0))
}

/// One source's way to one ear: the direct sound, the ground's reflection and the walls', each read off
/// the source's delay line.
#[derive(Clone, Copy, Default)]
struct Ear {
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
    /// The head's shadow over the direct sound, the ground's reflection and each wall's, when there is a
    /// head: with two ears.
    shadow: HeadShadow,
    ground_shadow: HeadShadow,
    wall_shadow: [HeadShadow; WALLS],
    shadowed: bool,
}

/// One source's way to the ears, through one delay line.
// On cache lines of its own, as each may be stepped on a thread of its own.
#[repr(align(128))]
struct Path {
    line: Delay,
    ears: [Ear; 2],
    /// What it was given last, until `Listener::take_sources` reads it.
    source: f64,
}

pub struct Listener {
    sample_rate: f64,
    paths: Vec<Path>,
    glide_c: f64,
    /// One ear, heard alike in both channels, or two a head apart.
    ears: usize,
}

impl Listener {
    pub fn new(sample_rate: f64) -> Listener {
        Listener { sample_rate, paths: Vec::new(), glide_c: 1.0 - math::exp(-1.0 / (GLIDE_S * sample_rate)), ears: 1 }
    }

    /// Put the listener at `ear` and the sources at `places`, above ground at height `ground` that
    /// reflects `reflection` of what reaches it, and inside `walls` if there are any. With `right`, the
    /// way the listener's right is, there are two ears, `HEAD_RADIUS` either side of `ear`, each in the
    /// head's shadow from the other side; without, one, at `ear`. With `snap` the paths take their new
    /// lengths and levels at once, as they do when there are not yet as many paths as places, when a
    /// room first needs them longer, or when the ears are counted afresh; otherwise they glide there.
    #[allow(clippy::too_many_arguments)]
    pub fn set_geometry(
        &mut self,
        ear: Vec3,
        right: Option<Vec3>,
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
        if resize {
            self.paths = places
                .iter()
                .map(|_| Path { line: Delay::new(longest), ears: [Ear::default(); 2], source: 0.0 })
                .collect();
        }
        let right = right.and_then(|r| {
            let len = math::hypot(&r);
            (len > 1e-9 && len.is_finite()).then(|| [r[0] / len, r[1] / len, r[2] / len])
        });
        // Left first.
        let heads: Vec<(Vec3, Option<Vec3>)> = match right {
            Some(r) => [-1.0, 1.0]
                .iter()
                .map(|&side| {
                    let axis = [side * r[0], side * r[1], side * r[2]];
                    (
                        [
                            ear[0] + HEAD_RADIUS * axis[0],
                            ear[1] + HEAD_RADIUS * axis[1],
                            ear[2] + HEAD_RADIUS * axis[2],
                        ],
                        Some(axis),
                    )
                })
                .collect(),
            None => vec![(ear, None)],
        };
        let snap = snap || resize || heads.len() != self.ears;
        self.ears = heads.len();

        let c = ambient_sound_speed();
        let ranges: Vec<Vec<(f64, f64)>> = heads
            .iter()
            .map(|(ear, _)| {
                let ear_height = math::max(ear[1] - ground, 0.02);
                places
                    .iter()
                    .map(|p| {
                        let height = math::max(p[1] - ground, 0.02);
                        let (dx, dz) = (ear[0] - p[0], ear[2] - p[2]);
                        let direct = math::max(math::hypot(&[dx, ear_height - height, dz]), MIN_RANGE);
                        let bounced = math::max(math::hypot(&[dx, ear_height + height, dz]), direct);
                        (direct, bounced)
                    })
                    .collect()
            })
            .collect();
        // Timed against the nearest path to either ear, so the ears keep the time between them.
        let nearest = ranges.iter().flatten().map(|r| r.0).fold(f64::INFINITY, f64::min);
        let most = MAX_DIFFERENCE_S * self.sample_rate;
        let most_wall = MAX_ROOM_DIFFERENCE_S * self.sample_rate;
        let fs = self.sample_rate;
        for (k, ((ear, axis), ranges)) in heads.iter().zip(&ranges).enumerate() {
            for ((path, &(direct, bounced)), place) in self.paths.iter_mut().zip(ranges).zip(places) {
                let e = &mut path.ears[k];
                e.direct_delay.target = math::min(((direct - nearest) / c) * fs, most);
                e.ground_delay.target = math::min(((bounced - nearest) / c) * fs, most);
                e.direct_gain.target = 1.0 / direct;
                e.ground_gain.target = math::max(reflection, 0.0) / bounced;
                e.ground_loss.set_cutoff(2600.0, fs);
                e.air_direct.set_cutoff(air_cutoff_hz(direct), fs);
                e.air_ground.set_cutoff(air_cutoff_hz(bounced), fs);
                e.shadowed = axis.is_some();
                if let Some(axis) = *axis {
                    let to = [place[0] - ear[0], place[1] - ear[1], place[2] - ear[2]];
                    e.shadow.set(angle_off(to, axis), fs);
                    let below = [to[0], 2.0 * ground - place[1] - ear[1], to[2]];
                    e.ground_shadow.set(angle_off(below, axis), fs);
                }
                match walls {
                    Some(w) => {
                        let place = w.inside(*place, ground, 0.0);
                        let mut mean = 0.0;
                        for (n, image) in w.images(place).iter().enumerate() {
                            let range = math::max(
                                math::hypot(&[ear[0] - image[0], ear[1] - image[1], ear[2] - image[2]]),
                                direct,
                            );
                            let delay = ((range - nearest) / c) * fs;
                            // Beyond what the line holds, a reflection that faint is left out.
                            let heard = delay <= most_wall;
                            e.wall_delay[n].target = math::min(delay, most_wall);
                            e.wall_gain[n].target = if heard { w.reflection / range } else { 0.0 };
                            if let Some(axis) = *axis {
                                let to = [image[0] - ear[0], image[1] - ear[1], image[2] - ear[2]];
                                e.wall_shadow[n].set(angle_off(to, axis), fs);
                            }
                            mean += range / WALLS as f64;
                        }
                        e.wall_loss.set_cutoff(w.corner_hz, fs);
                        e.air_walls.set_cutoff(air_cutoff_hz(mean), fs);
                        e.walls_on = true;
                    }
                    None => {
                        for g in e.wall_gain.iter_mut() {
                            g.target = 0.0;
                        }
                    }
                }
                if snap {
                    for g in [&mut e.direct_delay, &mut e.ground_delay, &mut e.direct_gain, &mut e.ground_gain]
                        .into_iter()
                        .chain(e.wall_delay.iter_mut())
                        .chain(e.wall_gain.iter_mut())
                    {
                        g.value = g.target;
                    }
                }
            }
        }
    }

    /// How many ears there are: 1, heard alike in both channels, or 2.
    pub fn ears(&self) -> usize {
        self.ears
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

    /// Radiated pressure from source `i`, referred to 1 m, Pa, to its pressure at the left ear and the
    /// right, Pa: alike with one ear.
    #[inline]
    pub fn process(&mut self, i: usize, source: f64) -> [f64; 2] {
        match self.paths.get_mut(i) {
            Some(p) => p.process(self.glide_c, source, self.ears),
            None => [0.0; 2],
        }
    }

    /// Every path, to be processed apart, each with `ListenerPaths::process`.
    pub fn paths(&mut self) -> ListenerPaths<'_> {
        ListenerPaths {
            paths: self.paths.as_mut_ptr(),
            count: self.paths.len(),
            glide_c: self.glide_c,
            ears: self.ears,
            _paths: std::marker::PhantomData,
        }
    }
}

/// The listener's paths, from `Listener::paths`, each reached only through its own index.
pub struct ListenerPaths<'a> {
    paths: *mut Path,
    count: usize,
    glide_c: f64,
    ears: usize,
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
    pub unsafe fn process(&self, i: usize, source: f64) -> [f64; 2] {
        if i >= self.count {
            return [0.0; 2];
        }
        unsafe { &mut *self.paths.add(i) }.process(self.glide_c, source, self.ears)
    }
}

impl Path {
    #[inline]
    fn process(&mut self, c: f64, source: f64, ears: usize) -> [f64; 2] {
        self.source = source;
        self.line.push(source);
        let left = self.ears[0].process(&self.line, c);
        if ears < 2 {
            return [left, left];
        }
        [left, self.ears[1].process(&self.line, c)]
    }
}

impl Ear {
    #[inline]
    fn process(&mut self, line: &Delay, c: f64) -> f64 {
        let direct = line.tap(self.direct_delay.next(c));
        let bounced = line.tap(self.ground_delay.next(c));
        let mut direct = self.air_direct.process(direct) * self.direct_gain.next(c);
        let mut ground = self.air_ground.process(self.ground_loss.process(bounced)) * self.ground_gain.next(c);
        if self.shadowed {
            direct = self.shadow.process(direct);
            ground = self.ground_shadow.process(ground);
        }
        if !self.walls_on {
            return direct + ground;
        }
        let mut walls = 0.0;
        let mut fading = true;
        for ((delay, gain), shadow) in
            self.wall_delay.iter_mut().zip(self.wall_gain.iter_mut()).zip(&mut self.wall_shadow)
        {
            let heard = line.tap(delay.next(c)) * gain.next(c);
            walls += if self.shadowed { shadow.process(heard) } else { heard };
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
