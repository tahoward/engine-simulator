//! The room the engine runs in, if any: its walls' first reflections, heard along each source's
//! path (`listener`), and the reverberation that builds up after them, here.
//!
//! The reverberation is a feedback delay network: eight delay lines, mixed through an 8 x 8 Hadamard
//! matrix and fed back, each line losing as much per pass as the room's reverberation time says
//! (Jot's design, with a one-pole in each line taking the highs away faster than the lows). It is fed
//! every source referred to 1 m, and gives the diffuse field's level the room's absorption sets,
//! `p^2 = 16 pi p1^2 / R`, wherever the ear stands in it.

use serde::{Deserialize, Serialize};

use crate::dsp::Delay;
use crate::math::{self, PI};

/// Where the engine is listened to.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum Room {
    /// In the open, with only the ground to reflect off.
    #[default]
    Outdoors,
    /// A single concrete garage.
    Garage,
    /// A dyno cell, its walls lined to soak up the sound.
    DynoCell,
    /// A steel-clad workshop.
    Workshop,
    /// An underground car park: wide, long and low, all concrete.
    CarPark,
    /// A road tunnel.
    Tunnel,
}

/// A room's box and what its surfaces do to the sound.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RoomShape {
    /// Across the car, along it and floor to ceiling, m.
    pub width: f64,
    pub length: f64,
    pub height: f64,
    /// The share of the sound energy its surfaces absorb, on average, at low frequencies.
    pub absorption: f64,
    /// Reverberation time at the top of the band as a share of the time at the bottom: the surfaces
    /// and the air both take the highs away faster.
    pub hf_ratio: f64,
    /// Corner above which a reflection off the walls is duller than the sound that reached them, Hz.
    pub wall_corner_hz: f64,
}

impl Room {
    /// Its box, or `None` outdoors.
    pub fn shape(self) -> Option<RoomShape> {
        let shape = |width, length, height, absorption, hf_ratio, wall_corner_hz| RoomShape {
            width,
            length,
            height,
            absorption,
            hf_ratio,
            wall_corner_hz,
        };
        match self {
            Room::Outdoors => None,
            Room::Garage => Some(shape(3.5, 6.5, 2.6, 0.08, 0.35, 5000.0)),
            Room::DynoCell => Some(shape(5.0, 8.0, 3.5, 0.45, 0.5, 1800.0)),
            Room::Workshop => Some(shape(12.0, 18.0, 6.0, 0.15, 0.4, 4000.0)),
            Room::CarPark => Some(shape(32.0, 48.0, 2.8, 0.12, 0.35, 4500.0)),
            Room::Tunnel => Some(shape(9.0, 400.0, 6.5, 0.1, 0.3, 4500.0)),
        }
    }
}

impl RoomShape {
    pub fn volume(&self) -> f64 {
        self.width * self.length * self.height
    }

    pub fn surface(&self) -> f64 {
        2.0 * (self.width * self.length + self.height * (self.width + self.length))
    }

    /// Reverberation time at low frequencies, s: Eyring's, which holds for a dead room as well as a
    /// live one.
    pub fn reverb_time(&self) -> f64 {
        (0.161 * self.volume()) / (-self.surface() * math::log(1.0 - self.absorption))
    }

    /// The room constant `R = S a / (1 - a)`, m^2.
    pub fn room_constant(&self) -> f64 {
        (self.surface() * self.absorption) / (1.0 - self.absorption)
    }

    /// Pressure in the diffuse field over the pressure the same source makes 1 m away in the open:
    /// `sqrt(16 pi / R)`.
    pub fn diffuse_gain(&self) -> f64 {
        math::sqrt((16.0 * PI) / self.room_constant())
    }

    /// How much of the pressure arriving at a wall it sends back.
    pub fn wall_reflection(&self) -> f64 {
        math::sqrt(1.0 - self.absorption)
    }
}

const LINES: usize = 8;

/// Each line's length in a room whose mean free path is `MEAN_FREE_PATH`, s. Spread apart so that no
/// two share a low common multiple, and long enough that the echoes do not flutter.
const LINE_S: [f64; LINES] = [0.0313, 0.0379, 0.0419, 0.0473, 0.0539, 0.0593, 0.0671, 0.0737];
const MEAN_FREE_PATH: f64 = 5.0;
/// Most and least the lines are scaled by with the room's mean free path.
const SCALE_RANGE: (f64, f64) = (0.6, 1.4);

/// How long the old room's tail takes to fade out when the room changes, s.
const FADE_S: f64 = 0.03;

#[derive(Clone, Copy, Debug, PartialEq)]
struct Tuning {
    lengths: [usize; LINES],
    /// Each line's loss per pass at DC, and its one-pole's pole.
    gain: [f64; LINES],
    pole: [f64; LINES],
    input: f64,
    /// Samples the tail takes to fall 90 dB once the input stops.
    ring: usize,
}

pub struct Reverb {
    sample_rate: f64,
    lines: Vec<Delay>,
    damp: [f64; LINES],
    tuning: Option<Tuning>,
    /// The room to change to once the old one's tail has faded out.
    next: Option<Option<Tuning>>,
    shape: Option<RoomShape>,
    fade: f64,
    fade_step: f64,
    /// Samples left until the tail is inaudible after the input stops; 0 when silent.
    ringing: usize,
}

impl Reverb {
    pub fn new(sample_rate: f64) -> Reverb {
        let longest = (LINE_S[LINES - 1] * SCALE_RANGE.1 * sample_rate).ceil() + 4.0;
        Reverb {
            sample_rate,
            lines: (0..LINES).map(|_| Delay::new(longest)).collect(),
            damp: [0.0; LINES],
            tuning: None,
            next: None,
            shape: None,
            fade: 1.0,
            fade_step: 1.0 / (FADE_S * sample_rate),
            ringing: 0,
        }
    }

    /// Reverberate as `shape` does, or with `None` not at all. Into another room, a tail still ringing
    /// fades out first; out of every room, it rings out as it would.
    pub fn set_room(&mut self, shape: Option<RoomShape>) {
        if shape == self.shape {
            return;
        }
        self.shape = shape;
        let tuning = shape.map(|s| self.tune(&s));
        if !self.active() {
            self.clear();
            self.tuning = tuning;
        } else if self.next.is_some() {
            self.next = Some(tuning);
        } else if tuning.is_none() {
            if let Some(t) = self.tuning.as_mut() {
                t.input = 0.0;
            }
        } else {
            self.next = Some(tuning);
        }
    }

    fn tune(&self, shape: &RoomShape) -> Tuning {
        let fs = self.sample_rate;
        let t60 = shape.reverb_time();
        let free_path = (4.0 * shape.volume()) / shape.surface();
        let scale = math::clamp(free_path / MEAN_FREE_PATH, SCALE_RANGE.0, SCALE_RANGE.1);
        let mut lengths = [0; LINES];
        let mut gain = [0.0; LINES];
        let mut pole = [0.0; LINES];
        let mut energy_kept = 0.0;
        for i in 0..LINES {
            let m = next_prime((LINE_S[i] * scale * fs) as usize);
            lengths[i] = m;
            // 60 dB down in `t60`: `g = 10^(-3 m / (fs t60))`.
            let db_per_pass = (-60.0 * m as f64) / (fs * t60);
            gain[i] = math::exp((db_per_pass / 20.0) * std::f64::consts::LN_10);
            // Jot's pole for a one-pole whose loss at Nyquist gives `hf_ratio` of the time.
            let p = ((std::f64::consts::LN_10 / 4.0) * (db_per_pass / 20.0))
                * (1.0 - 1.0 / (shape.hf_ratio * shape.hf_ratio));
            pole[i] = math::clamp(p, 0.0, 0.95);
            energy_kept += gain[i] * gain[i] / LINES as f64;
        }
        // An impulse in puts unit energy into the lines, of which `g^2 / (1 - g^2)` comes out of them
        // in all, a share `1 / LINES` of it in the output: scaled to the diffuse field's energy.
        let input = math::sqrt((LINES as f64 * (1.0 - energy_kept)) / energy_kept) * shape.diffuse_gain();
        Tuning { lengths, gain, pole, input, ring: (1.5 * t60 * fs) as usize }
    }

    fn clear(&mut self) {
        for line in self.lines.iter_mut() {
            line.clear();
        }
        self.damp = [0.0; LINES];
        self.ringing = 0;
    }

    /// Whether there is anything to hear: a room fed, or a tail still ringing.
    #[inline]
    pub fn active(&self) -> bool {
        self.tuning.is_some_and(|t| t.input > 0.0) || self.ringing > 0 || self.next.is_some()
    }

    /// The diffuse field at the ear from `source`, every source summed, each referred to 1 m, Pa.
    pub fn process(&mut self, source: f64) -> f64 {
        let Some(t) = self.tuning else { return 0.0 };
        let mut y = [0.0; LINES];
        for i in 0..LINES {
            let x = self.lines[i].tap(t.lengths[i] as f64 - 1.0);
            self.damp[i] = t.gain[i] * (1.0 - t.pole[i]) * x + t.pole[i] * self.damp[i];
            y[i] = self.damp[i];
        }
        let mut out = 0.0;
        for (i, &v) in y.iter().enumerate() {
            out += if i % 2 == 0 { v } else { -v };
        }
        out *= self.fade / math::sqrt(LINES as f64);
        hadamard(&mut y);
        let feed = (t.input * source) / math::sqrt(LINES as f64);
        for i in 0..LINES {
            let sign = if i % 3 == 0 { -1.0 } else { 1.0 };
            self.lines[i].push(y[i] + sign * feed);
        }

        if t.input > 0.0 && source != 0.0 {
            self.ringing = t.ring;
        } else if self.ringing > 0 {
            self.ringing -= 1;
            if self.ringing == 0 && t.input == 0.0 {
                self.clear();
                self.tuning = None;
            }
        }
        if let Some(next) = self.next {
            self.fade -= self.fade_step;
            if self.fade <= 0.0 {
                self.clear();
                self.tuning = next;
                self.next = None;
                self.fade = 1.0;
            }
        }
        out
    }
}

/// In-place fast Walsh-Hadamard transform, scaled to keep the energy.
fn hadamard(v: &mut [f64; LINES]) {
    let mut h = 1;
    while h < LINES {
        for i in (0..LINES).step_by(2 * h) {
            for j in i..i + h {
                let (a, b) = (v[j], v[j + h]);
                v[j] = a + b;
                v[j + h] = a - b;
            }
        }
        h *= 2;
    }
    let norm = 1.0 / math::sqrt(LINES as f64);
    for x in v.iter_mut() {
        *x *= norm;
    }
}

fn next_prime(n: usize) -> usize {
    let is_prime = |k: usize| k >= 2 && (2..).take_while(|d| d * d <= k).all(|d| !k.is_multiple_of(d));
    (n.max(2)..).find(|&k| is_prime(k)).unwrap()
}
