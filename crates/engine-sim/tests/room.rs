//! The room the engine runs in: the reverberation reaches the diffuse field's level the room's
//! absorption sets and dies away in its reverberation time; the engine is heard in it louder than in
//! the open; and changing room or leaving it is smooth.

mod common;

use common::FS;
use engine_sim::EngineSim;
use engine_sim::dsp::{Noise, OnePole};
use engine_sim::room::{Reverb, Room};
use serde_json::json;

const ROOMS: [Room; 5] = [Room::Garage, Room::DynoCell, Room::Workshop, Room::CarPark, Room::Tunnel];

/// Noise below a few hundred hertz, as most of an engine's sound is.
fn low_noise(n: usize) -> Vec<f64> {
    let mut noise = Noise::default();
    let mut lp = OnePole::default();
    lp.set_cutoff(300.0, FS);
    (0..n).map(|_| lp.process(noise.next())).collect()
}

/// Fed steadily, the reverberation is as loud as the diffuse field, `16 pi / R` in power, to 2 dB.
#[test]
fn reaches_the_diffuse_field_level() {
    for room in ROOMS {
        let shape = room.shape().unwrap();
        let mut reverb = Reverb::new(FS);
        reverb.set_room(Some(shape));
        let input = low_noise(FS as usize * 12);
        let settle = (shape.reverb_time() * 2.0 * FS) as usize;
        let (mut p_in, mut p_out) = (0.0, 0.0);
        for (i, &x) in input.iter().enumerate() {
            let y = reverb.process(x);
            if i > settle {
                p_in += x * x;
                p_out += y * y;
            }
        }
        let db = 10.0 * (p_out / p_in / shape.diffuse_gain().powi(2)).log10();
        assert!(db.abs() < 2.0, "{room:?}: {db:.2} dB from the diffuse field");
    }
}

/// An impulse's low frequencies die away 60 dB in the room's reverberation time, to 15%, measured
/// over the first 30 dB of the decay.
#[test]
fn dies_away_in_the_reverberation_time() {
    for room in ROOMS {
        let shape = room.shape().unwrap();
        let mut reverb = Reverb::new(FS);
        reverb.set_room(Some(shape));
        let t60 = shape.reverb_time();
        let mut lp = OnePole::default();
        lp.set_cutoff(200.0, FS);
        let ir: Vec<f64> = (0..(t60 * 1.2 * FS) as usize)
            .map(|i| lp.process(reverb.process(if i == 0 { 1.0 } else { 0.0 })))
            .collect();
        // Schroeder's backward integration.
        let mut edc = vec![0.0; ir.len()];
        let mut sum = 0.0;
        for i in (0..ir.len()).rev() {
            sum += ir[i] * ir[i];
            edc[i] = sum;
        }
        let at = |db: f64| edc.iter().position(|&e| 10.0 * (e / edc[0]).log10() < db).unwrap() as f64 / FS;
        let measured = (at(-35.0) - at(-5.0)) * 2.0;
        assert!((measured / t60 - 1.0).abs() < 0.15, "{room:?}: T60 {measured:.2} s against {t60:.2} s");
    }
}

/// The highs die away faster than the lows.
#[test]
fn the_highs_die_away_first() {
    let mut reverb = Reverb::new(FS);
    reverb.set_room(Room::Garage.shape());
    let mut noise = Noise::default();
    let feed = FS as usize * 2;
    let mut tail = Vec::new();
    for i in 0..feed + FS as usize / 4 {
        let y = reverb.process(if i < feed { noise.next() } else { 0.0 });
        if i >= feed + FS as usize / 8 {
            tail.push(y);
        }
    }
    // Energy of the tail's first difference, a highpass, over its own.
    let total: f64 = tail.iter().map(|y| y * y).sum();
    let high: f64 = tail.windows(2).map(|w| (w[1] - w[0]).powi(2)).sum();
    assert!(high / total < 0.2, "a dull tail: {:.3}", high / total);
}

fn sim_in(room: &str) -> EngineSim {
    let mut cfg = common::default_config();
    cfg.engine = common::with(&cfg.engine, json!({ "room": room, "combustionVariability": 0 }));
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(FS as usize);
    sim
}

/// Outdoors is the engine as it has always been heard: unchanged to the bit by the room's machinery.
#[test]
fn outdoors_hears_no_room() {
    let mut cfg = common::default_config();
    cfg.engine = common::with(&cfg.engine, json!({ "combustionVariability": 0 }));
    let mut plain = EngineSim::new(FS, &cfg);
    let mut out = sim_in("outdoors");
    plain.render(FS as usize);
    assert_eq!(plain.render(FS as usize / 2), out.render(FS as usize / 2));
}

/// In a live room the engine is louder than in the open, and in a dead one only a little.
#[test]
fn a_live_room_is_louder_than_the_open() {
    let level = |room: &str| common::rms(&sim_in(room).render(FS as usize));
    let open = level("outdoors");
    let garage = level("garage");
    let dyno = level("dynoCell");
    assert!(garage > open * 1.5, "garage {garage} against {open} outdoors");
    assert!(dyno > open && dyno < garage, "dyno cell {dyno}, between {open} and {garage}");
}

/// Changing room, and leaving it, gives no click: nothing jumps by more than the sound itself does.
#[test]
fn changing_room_is_smooth() {
    let mut sim = sim_in("garage");
    sim.render(FS as usize);
    let largest_step = |s: &[f32]| s.windows(2).fold(0.0f32, |m, w| m.max((w[1] - w[0]).abs()));
    let before = largest_step(&sim.render(FS as usize / 2));
    for room in ["tunnel", "carPark", "outdoors", "workshop"] {
        sim.set_engine_json(&json!({ "room": room })).unwrap();
        let out = sim.render(FS as usize / 2);
        assert!(out.iter().all(|s| s.is_finite()), "{room}: finite");
        let step = largest_step(&out);
        assert!(step < before * 2.0, "{room}: a step of {step} against {before}");
    }
}
