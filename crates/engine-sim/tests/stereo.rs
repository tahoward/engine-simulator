//! Stereo: two ears a head apart. A source off to one side reaches the nearer ear first, louder and
//! brighter; one dead ahead reaches both alike; out of stereo both channels are the one ear; and a
//! room's reverberation is as loud at each ear as at the one, alike between them in the lows and
//! unalike in the highs.

use crate::common;

use common::FS;
use engine_sim::EngineSim;
use engine_sim::dsp::{Noise, OnePole};
use engine_sim::listener::{MouthPlace, SoundSources};
use engine_sim::room::{Reverb, Room};
use serde_json::json;

const SECOND: usize = FS as usize;

/// The default single, every source of it at `at`, heard in stereo from `ear` facing with its right
/// along `right`, over ground that reflects nothing.
fn heard_from(at: [f64; 3], ear: [f64; 3], right: [f64; 3]) -> EngineSim {
    let mut cfg = common::default_config();
    cfg.engine = common::with(&cfg.engine, json!({ "groundReflection": 0, "combustionVariability": 0 }));
    let mut s = EngineSim::new(FS, &cfg);
    let duct = s.pipe_solver().mouth_duct_id(0).unwrap().to_string();
    s.set_sources(SoundSources {
        mouths: vec![MouthPlace { duct, position: at }],
        intake: Some(at),
        second_intake: None,
        engine: Some(at),
        turbo: Some(at),
        surfaces: Vec::new(),
    });
    s.set_stereo(true);
    s.set_listener_facing(Some(ear), Some(right));
    stereo(&mut s, SECOND);
    s
}

fn stereo(s: &mut EngineSim, n: usize) -> (Vec<f32>, Vec<f32>) {
    let (mut l, mut r) = (vec![0.0; n], vec![0.0; n]);
    s.render_stereo_into(&mut l, &mut r);
    (l, r)
}

/// The lag, samples, at which `b` best matches `a`, within `most` either way: positive where `b` is
/// later.
fn lag(a: &[f32], b: &[f32], most: i64) -> i64 {
    let at = |lag: i64| -> f64 {
        (most as usize..a.len() - most as usize).map(|i| a[i] as f64 * b[(i as i64 + lag) as usize] as f64).sum()
    };
    (-most..=most).max_by(|&x, &y| at(x).total_cmp(&at(y))).unwrap()
}

/// The energy of `x`'s first difference over its own: how much of it is treble.
fn brightness(x: &[f32]) -> f64 {
    let total: f64 = x.iter().map(|&v| (v as f64).powi(2)).sum();
    let high: f64 = x.windows(2).map(|w| ((w[1] - w[0]) as f64).powi(2)).sum();
    high / total
}

/// A source to the listener's right reaches the right ear sooner, by about the time sound takes round
/// the head: Woodworth's `a (theta + sin theta) / c`, 0.66 ms side on, which the lows a real head lags
/// by a little more. And louder and brighter, the left ear being in the head's shadow.
#[test]
fn a_source_to_one_side_reaches_the_nearer_ear_first_louder_and_brighter() {
    let at = [2.0, 0.5, 0.0];
    let mut s = heard_from(at, [0.0, 0.5, 0.0], [1.0, 0.0, 0.0]);
    let (l, r) = stereo(&mut s, SECOND);
    // 0.66 ms is 31.5 samples; the straight line between the ears alone, 0.51 ms, would be 24.
    let late = lag(&r, &l, 48);
    assert!((29..=37).contains(&late), "the left ear {late} samples behind the right");
    let (lr, rr) = (common::rms(&l), common::rms(&r));
    assert!(rr > lr * 1.1, "right {rr} against left {lr}");
    assert!(brightness(&r) > brightness(&l) * 1.5, "right {} against left {}", brightness(&r), brightness(&l));
}

/// Turned round, the ears swap.
#[test]
fn turned_round_the_ears_swap() {
    let at = [2.0, 0.5, 0.0];
    let (l, r) = stereo(&mut heard_from(at, [0.0, 0.5, 0.0], [-1.0, 0.0, 0.0]), SECOND);
    assert!(common::rms(&l) > common::rms(&r) * 1.1, "left {} right {}", common::rms(&l), common::rms(&r));
}

/// A source dead ahead reaches both ears alike.
#[test]
fn a_source_dead_ahead_reaches_both_ears_alike() {
    let at = [0.0, 0.5, -2.0];
    let (l, r) = stereo(&mut heard_from(at, [0.0, 0.5, 0.0], [1.0, 0.0, 0.0]), SECOND);
    let worst = l.iter().zip(&r).map(|(a, b)| (a - b).abs()).fold(0.0f32, f32::max);
    assert!(worst < 1e-6, "the ears differ by up to {worst}");
}

/// Out of stereo both channels are the one ear, and the mix `render_into` gives is that ear as it was.
#[test]
fn out_of_stereo_both_channels_are_the_one_ear() {
    let cfg = common::default_config();
    let mut mono = EngineSim::new(FS, &cfg);
    let mut both = EngineSim::new(FS, &cfg);
    let heard = mono.render(SECOND);
    let (l, r) = stereo(&mut both, SECOND);
    assert_eq!(l, heard);
    assert_eq!(r, heard);
}

/// In stereo the mix keeps the level the one ear hears, near enough.
#[test]
fn the_stereo_mix_is_about_as_loud_as_the_one_ear() {
    let cfg = common::default_config();
    let mut mono = EngineSim::new(FS, &cfg);
    let mut two = EngineSim::new(FS, &cfg);
    two.set_stereo(true);
    mono.render(SECOND);
    two.render(SECOND);
    let (a, b) = (common::rms(&mono.render(SECOND)), common::rms(&two.render(SECOND)));
    let db = 20.0 * (b / a).log10();
    assert!(db.abs() < 1.5, "{db:.2} dB from the one ear");
}

/// Changing to stereo and back mid-sound stays finite and within full scale.
#[test]
fn switching_stereo_stays_finite() {
    let mut cfg = common::default_config();
    cfg.engine = common::with(&cfg.engine, json!({ "room": "garage" }));
    let mut s = EngineSim::new(FS, &cfg);
    for on in [true, false, true] {
        s.set_stereo(on);
        let (l, r) = stereo(&mut s, SECOND / 2);
        assert!(l.iter().chain(&r).all(|v| v.is_finite() && v.abs() <= 1.0));
    }
}

/// A garage's reverberation, as the one ear hears it and as each of a head facing with its right along
/// `right` does, fed `seconds` of white noise.
fn reverberate(room: Room, right: [f64; 3], seconds: usize) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let shape = room.shape().unwrap();
    let mut one = Reverb::new(FS);
    let mut two = Reverb::new(FS);
    one.set_room(Some(shape));
    two.set_room(Some(shape));
    two.set_head(Some(right));
    let mut noise = Noise::default();
    let n = SECOND * seconds;
    let (mut mono, mut l, mut r) = (Vec::with_capacity(n), Vec::with_capacity(n), Vec::with_capacity(n));
    for _ in 0..n {
        let x = noise.next();
        mono.push(one.process(x));
        let [a, b] = two.process_stereo(x);
        l.push(a);
        r.push(b);
    }
    (mono, l, r)
}

const SETTLED: usize = SECOND * 3;

fn power(x: &[f64]) -> f64 {
    x[SETTLED..].iter().map(|v| v * v).sum::<f64>()
}

/// How alike `l` and `r` are below `cutoff` (`low`) or above it, -1..1.
fn coherence(l: &[f64], r: &[f64], cutoff: f64, low: bool) -> f64 {
    let band = |x: &[f64]| {
        let mut lp = OnePole::default();
        lp.set_cutoff(cutoff, FS);
        let mut lp2 = lp;
        x.iter()
            .map(|&v| {
                let y = lp2.process(lp.process(v));
                if low { y } else { v - y }
            })
            .collect::<Vec<f64>>()
    };
    let (a, b) = (band(l), band(r));
    let dot: f64 = a[SETTLED..].iter().zip(&b[SETTLED..]).map(|(x, y)| x * y).sum();
    dot / (power(&a) * power(&b)).sqrt()
}

/// Each ear hears a room's reverberation as loud as the one ear does, to 1 dB; the two alike below a
/// few hundred hertz and unalike above.
#[test]
fn each_ear_hears_the_reverberation_alike_in_the_lows_and_unalike_in_the_highs() {
    let (mono, l, r) = reverberate(Room::Garage, [1.0, 0.0, 0.0], 6);
    for (ear, x) in [("left", &l), ("right", &r)] {
        let db = 10.0 * (power(x) / power(&mono)).log10();
        assert!(db.abs() < 1.0, "{ear} ear {db:.2} dB from the one");
    }
    let low = coherence(&l, &r, 150.0, true);
    let high = coherence(&l, &r, 3000.0, false);
    assert!(low > 0.8, "coherence {low:.2} below 150 Hz");
    assert!(high.abs() < 0.3, "coherence {high:.2} above 3 kHz");
}

/// The reverberation stays where the room is: turned round, the ears hear what each other did.
#[test]
fn turned_round_the_reverberation_swaps_ears() {
    let (_, l, r) = reverberate(Room::Garage, [1.0, 0.0, 0.0], 4);
    let (_, l2, r2) = reverberate(Room::Garage, [-1.0, 0.0, 0.0], 4);
    let worst = l.iter().zip(&r2).chain(r.iter().zip(&l2)).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
    assert!(worst < 1e-9, "the ears differ from each other's by up to {worst}");
}

/// In a tunnel the late sound comes along the bore. Facing along it, that is from ahead and behind,
/// and the ears hear it more alike than facing a wall, when it is from either side. Below 800 Hz,
/// where the little that still comes from the side does not already set the ears apart.
#[test]
fn in_a_tunnel_the_reverberation_comes_along_the_bore() {
    let (_, l, r) = reverberate(Room::Tunnel, [1.0, 0.0, 0.0], 5);
    let (_, l2, r2) = reverberate(Room::Tunnel, [0.0, 0.0, 1.0], 5);
    let along = coherence(&l, &r, 800.0, true);
    let across = coherence(&l2, &r2, 800.0, true);
    assert!(along > across + 0.1, "coherence below 800 Hz {along:.2} facing along against {across:.2} across");
}

/// In a street the late sound comes from the facades either side, what runs along it going out of the
/// ends and what goes up, to the sky. Facing along the street, that is from either side, and the ears
/// hear it less alike than facing a facade, when it is from ahead and behind.
#[test]
fn in_a_street_the_reverberation_comes_from_the_facades() {
    let (_, l, r) = reverberate(Room::Street, [1.0, 0.0, 0.0], 5);
    let (_, l2, r2) = reverberate(Room::Street, [0.0, 0.0, 1.0], 5);
    let along = coherence(&l, &r, 800.0, true);
    let facing = coherence(&l2, &r2, 800.0, true);
    assert!(facing > along + 0.1, "coherence below 800 Hz {facing:.2} facing a facade against {along:.2} along");
}

/// A wall to one side is heard from that side: near the right wall of a garage, the right ear hears
/// more of the treble against the left than it does in the open, the wall's reflection coming from its
/// side and the far ear hearing it in the head's shadow.
#[test]
fn a_wall_to_one_side_is_heard_from_that_side() {
    let treble_right_over_left = |room: &str| {
        let mut cfg = common::default_config();
        cfg.engine =
            common::with(&cfg.engine, json!({ "groundReflection": 0, "combustionVariability": 0, "room": room }));
        let mut s = EngineSim::new(FS, &cfg);
        let duct = s.pipe_solver().mouth_duct_id(0).unwrap().to_string();
        let at = [0.0, 0.5, -1.0];
        s.set_sources(SoundSources {
            mouths: vec![MouthPlace { duct, position: at }],
            intake: Some(at),
            second_intake: None,
            engine: Some(at),
            turbo: Some(at),
            surfaces: Vec::new(),
        });
        s.set_stereo(true);
        // The garage is centred on the sources, 3.5 m wide: this puts the ear 0.4 m from its right wall.
        s.set_listener_facing(Some([1.35, 0.5, 0.0]), Some([1.0, 0.0, 0.0]));
        stereo(&mut s, SECOND);
        let (l, r) = stereo(&mut s, SECOND);
        brightness(&r) / brightness(&l)
    };
    let (open, garage) = (treble_right_over_left("outdoors"), treble_right_over_left("garage"));
    assert!(garage > open * 1.2, "right over left {garage:.2} in the garage against {open:.2} in the open");
}
