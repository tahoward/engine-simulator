//! The room the engine runs in: the reverberation reaches the diffuse field's level the room's
//! absorption sets and dies away in its reverberation time; the engine is heard in it louder than in
//! the open; and changing room or leaving it is smooth.

use crate::common;

use common::FS;
use engine_sim::EngineSim;
use engine_sim::dsp::{Noise, OnePole};
use engine_sim::room::{Absorption, Reverb, Room, RoomModes, RoomShape};
use serde_json::json;

const ROOMS: [Room; 7] =
    [Room::Garage, Room::DynoCell, Room::Workshop, Room::CarPark, Room::Tunnel, Room::Street, Room::Underpass];

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

/// Where a room has no surface it gives back nothing: a street's sky and ends, an underpass's ends.
#[test]
fn an_open_side_gives_back_nothing() {
    let [left, right, front, rear, ceiling] = Room::Street.shape().unwrap().wall_reflections();
    assert!(left > 0.9 && right > 0.9, "facades {left} {right}");
    assert_eq!([front, rear, ceiling], [0.0; 3]);
    let [left, right, front, rear, ceiling] = Room::Underpass.shape().unwrap().wall_reflections();
    assert!(left > 0.9 && right > 0.9 && ceiling > 0.9, "walls {left} {right}, deck {ceiling}");
    assert_eq!([front, rear], [0.0; 2]);
}

/// An open side takes all that reaches it, so a street rings for less time than it would roofed over
/// and closed at the ends, and its diffuse field is quieter.
#[test]
fn an_open_side_shortens_and_quietens_the_reverberation() {
    let open = Room::Street.shape().unwrap();
    let closed =
        RoomShape { absorption: Absorption { front: 0.06, rear: 0.06, ceiling: 0.06, ..open.absorption }, ..open };
    assert!(
        open.reverb_time() < closed.reverb_time() / 3.0,
        "{} s against {} s",
        open.reverb_time(),
        closed.reverb_time()
    );
    assert!(open.diffuse_gain() < closed.diffuse_gain() / 2.0);
}

// --- the room's modes ---

/// The garage's box, from its corner at the origin, with `shape`'s walls.
fn garage_box(shape: &RoomShape) -> engine_sim::listener::Walls {
    engine_sim::listener::Walls {
        x: [0.0, shape.width],
        z: [0.0, shape.length],
        ceiling: shape.height,
        reflection: shape.wall_reflections(),
        corner_hz: shape.wall_corner_hz,
    }
}

/// A closed room rings at its box's modes: a click in one corner of the garage, heard in the opposite
/// one, rings in peaks at the first along its length, across it and up it, `c / 2 L` and so on, each
/// above the spectrum's middle, which from a corner, where every mode is driven, their skirts fill.
#[test]
fn a_closed_room_rings_at_its_boxs_modes() {
    let shape = Room::Garage.shape().unwrap();
    let walls = garage_box(&shape);
    let mut modes = RoomModes::new(FS);
    modes.set(
        Some(&shape),
        Some(&walls),
        0.0,
        &[[0.05, 0.05, 0.05]],
        [shape.width - 0.05, shape.height - 0.05, shape.length - 0.05],
    );
    assert!(modes.top_hz() > 200.0, "modes up to {} Hz", modes.top_hz());
    let n = 1 << 17;
    let out: Vec<f32> = (0..n).map(|i| modes.process(&[if i == 0 { 1.0 } else { 0.0 }]) as f32).collect();
    let mag = common::magnitude_spectrum(&out, n);
    let c = engine_sim::spec::ambient_sound_speed();
    let bin = |hz: f64| (hz * n as f64 / FS).round() as usize;
    let mut low: Vec<f64> = mag[bin(15.0)..bin(120.0)].to_vec();
    low.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median = low[low.len() / 2];
    for (side, axis) in [(shape.length, "length"), (shape.width, "width"), (shape.height, "height")] {
        let mode = c / (2.0 * side);
        // The strongest bin within 2 Hz of it: there, and far above the spectrum's middle.
        let (lo, hi) = (bin(mode - 2.0), bin(mode + 2.0));
        let peak = (lo..=hi).max_by(|&a, &b| mag[a].partial_cmp(&mag[b]).unwrap()).unwrap();
        let found = peak as f64 * FS / n as f64;
        assert!((found - mode).abs() < 0.6, "{axis}'s first mode at {found:.2} Hz, not {mode:.2}");
        assert!(mag[peak] > 1.5 * median, "{axis}'s first mode {} against the spectrum's middle {median}", mag[peak]);
    }
}

/// A source at the middle of the garage's length sits in a node of every mode that varies along it an
/// odd number of times, and drives none of them; nor is a room open on a side given any modes.
#[test]
fn a_node_drives_nothing_and_an_open_room_has_no_modes() {
    let shape = Room::Garage.shape().unwrap();
    let walls = garage_box(&shape);
    let c = engine_sim::spec::ambient_sound_speed();
    let first = c / (2.0 * shape.length);
    let ring_at = |z: f64| {
        let mut modes = RoomModes::new(FS);
        modes.set(Some(&shape), Some(&walls), 0.0, &[[0.05, 0.05, z]], [0.05, 0.05, 0.05]);
        let n = 1 << 16;
        let out: Vec<f32> = (0..n).map(|i| modes.process(&[if i == 0 { 1.0 } else { 0.0 }]) as f32).collect();
        common::magnitude_spectrum(&out, n)[(first * n as f64 / FS).round() as usize]
    };
    let (end, middle) = (ring_at(0.05), ring_at(shape.length / 2.0));
    assert!(middle < 0.05 * end, "{middle} from the middle against {end} from the end");
    for room in [Room::Tunnel, Room::Street, Room::Underpass] {
        let shape = room.shape().unwrap();
        let mut modes = RoomModes::new(FS);
        modes.set(Some(&shape), Some(&garage_box(&shape)), 0.0, &[[1.0, 1.0, 1.0]], [2.0, 1.0, 2.0]);
        assert_eq!(modes.top_hz(), 0.0, "{room:?}");
    }
}

/// In the garage the room's modes carry the low end, and where a diesel's harmonics land on them they
/// stand out: idling in it, the 6CT's sound has much more of its 50-200 Hz harmonics above the noise
/// between them than its diffuse field alone, all noise down there, would leave. Its cylinders are held
/// alike, as the half orders their differences add between the harmonics would count as that noise, and
/// it idles at 800 rpm, where its 40 Hz firing's harmonics land on the garage's modes.
#[test]
fn in_the_garage_the_modes_carry_the_low_end() {
    let render = |room: &str| {
        let mut cfg = common::engine_preset("Inline six diesel, Cummins 6CT").config.clone();
        cfg.engine = common::with(
            &cfg.engine,
            json!({ "freeRunning": true, "room": room, "cylinderSpread": 0, "rpm": 800, "idleRpm": 800 }),
        );
        let mut sim = EngineSim::new(FS, &cfg);
        sim.set_listener(Some([1.3, 0.9, -0.8]));
        sim.render(3 * FS as usize);
        let n = 1 << 15;
        let x = sim.render(4 * n);
        let mut power = vec![0.0; n / 2 + 1];
        for w in x.chunks(n) {
            for (p, m) in power.iter_mut().zip(common::magnitude_spectrum(&common::hann(w), n)) {
                *p += m * m;
            }
        }
        let hz = |i: usize| i as f64 * FS / n as f64;
        let mut low: Vec<f64> =
            power.iter().enumerate().filter(|(i, _)| (50.0..200.0).contains(&hz(*i))).map(|(_, p)| *p).collect();
        low.sort_by(|a, b| a.partial_cmp(b).unwrap());
        10.0 * (low[low.len() - 1] / low[low.len() / 2]).log10()
    };
    let garage = render("garage");
    println!("50-200 Hz: the strongest harmonic {garage:.1} dB over the noise in the garage");
    // Its diffuse field alone, the modes left out, leaves 35 dB.
    assert!(garage > 40.0, "{garage:.1} dB");
}
