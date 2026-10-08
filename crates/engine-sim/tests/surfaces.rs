//! The casing's surfaces: each block side, head, the oil pan and the front cover heard from where it is,
//! louder the way it faces, each carrying its own parts' sound.

use crate::common;

use engine_sim::engine_sim::EngineSim;
use engine_sim::listener::directivity;
use serde_json::json;
use std::sync::OnceLock;

const FS: f64 = 48000.0;

/// `name` held on full throttle, its tailpipes 30 m off so the casing is what is heard, from `ear`, with its
/// mechanical noise at `mech`: a second of its sound, with the casing's surfaces or, without, from its one
/// place.
fn render(name: &str, surfaces: bool, ear: [f64; 3], mech: f64) -> Vec<f32> {
    let mut cfg = common::engine_preset(name).config.clone();
    let rpm = if name.contains("Cummins") { 1800 } else { 4000 };
    cfg.engine = common::with(
        &cfg.engine,
        json!({ "freeRunning": false, "throttle": 1, "rpm": rpm, "turboNoise": 0, "mechNoise": mech }),
    );
    let sources = cfg.sources.as_mut().unwrap();
    for m in sources.mouths.iter_mut() {
        m.position = [0.0, 0.0, 30.0];
    }
    if !surfaces {
        sources.surfaces.clear();
    }
    let mut sim = EngineSim::new(FS, &cfg);
    sim.set_listener(Some(ear));
    sim.render(FS as usize);
    sim.render(FS as usize)
}

/// The casing's power in each of five bands, 100 Hz to 16 kHz: the sound with its mechanical noise, less
/// the sound without.
fn casing_bands(name: &str, surfaces: bool, ear: [f64; 3]) -> Vec<f64> {
    let bands = |x: &[f32]| -> Vec<f64> {
        let n = 32768;
        let mag = common::magnitude_spectrum(&common::hann(&x[..n]), n);
        let hz = |i: usize| i as f64 * FS / n as f64;
        let edges = [100.0, 400.0, 1000.0, 2500.0, 6000.0, 16000.0];
        edges
            .windows(2)
            .map(|w| mag.iter().enumerate().filter(|(i, _)| hz(*i) >= w[0] && hz(*i) < w[1]).map(|(_, m)| m * m).sum())
            .collect()
    };
    let [on, off] = common::par([0.45, 0.0], |&mech| bands(&render(name, surfaces, ear, mech)));
    on.iter().zip(&off).map(|(a, b)| (a - b).max(1e-30)).collect()
}

const AROUND: [(&str, [f64; 3]); 5] = [
    ("front", [0.0, 0.6, -2.0]),
    ("left", [-2.0, 0.6, 0.0]),
    ("right", [2.0, 0.6, 0.0]),
    ("above", [0.0, 2.0, 0.3]),
    ("rear", [0.0, 0.6, 2.0]),
];
const FRONT: usize = 0;
const LEFT: usize = 1;

const ENGINES: [&str; 2] = ["V8, Chevrolet LT2", "Inline six diesel, Cummins 6CT"];
const LT2: usize = 0;

/// `casing_bands` of `ENGINES[engine]` from `AROUND[ear]`: rendered once, however many tests read it.
fn bands_around(engine: usize, surfaces: bool, ear: usize) -> &'static [f64] {
    static BANDS: [OnceLock<Vec<f64>>; 20] = [const { OnceLock::new() }; 20];
    BANDS[(engine * 2 + surfaces as usize) * AROUND.len() + ear]
        .get_or_init(|| casing_bands(ENGINES[engine], surfaces, AROUND[ear].1))
}

/// A surface is loudest the way it faces and quietest behind, `DIRECTIVITY_FLOOR` of that, and over every
/// direction radiates the power a source radiating alike every way would. A source with no facing is
/// heard alike every way.
#[test]
fn a_surface_is_loudest_the_way_it_faces() {
    let up = Some([0.0, 1.0, 0.0]);
    let ahead = directivity(up, [0.0, 1.0, 0.0]);
    let behind = directivity(up, [0.0, -1.0, 0.0]);
    assert!((behind / ahead - 0.2).abs() < 1e-9, "{behind} behind, {ahead} ahead");
    let n = 2000;
    let power = (0..n)
        .map(|i| {
            let cos = -1.0 + (2.0 * (i as f64 + 0.5)) / n as f64;
            let sin = (1.0 - cos * cos).sqrt();
            directivity(up, [sin, cos, 0.0]).powi(2)
        })
        .sum::<f64>()
        / n as f64;
    assert!((power - 1.0).abs() < 1e-3, "{power}");
    assert_eq!(directivity(None, [0.3, -1.0, 2.0]), 1.0);
}

/// Spread over its surfaces, the casing is about as loud from every side as from its one place: within 5 dB,
/// louder from the way its heads face.
#[test]
fn spread_over_its_surfaces_the_casing_is_about_as_loud() {
    let each_way: Vec<(usize, usize)> =
        (0..ENGINES.len()).flat_map(|e| (0..AROUND.len()).map(move |a| (e, a))).collect();
    common::each(&each_way, |&(engine, ear)| {
        let split: f64 = bands_around(engine, true, ear).iter().sum();
        let one: f64 = bands_around(engine, false, ear).iter().sum();
        let db = 10.0 * (split / one).log10();
        assert!(db.abs() < 5.0, "{} from the {}: {db:+.1} dB", ENGINES[engine], AROUND[ear].0);
    });
}

/// From the side, the casing's block and pistons carry its 1-2.5 kHz band, where from the front the oil pan
/// and the timing drive have more of the sound. From one place there is no such difference to hear.
#[test]
fn from_the_side_the_block_and_pistons_stand_out() {
    let lift = |surfaces: bool| -> f64 {
        let share = |ear: usize| {
            let bands = bands_around(LT2, surfaces, ear);
            10.0 * (bands[2] / bands.iter().sum::<f64>()).log10()
        };
        share(LEFT) - share(FRONT)
    };
    let (split, one) = (lift(true), lift(false));
    println!(
        "1-2.5 kHz from the side against the front: {split:+.1} dB spread over the surfaces, {one:+.1} dB from one place"
    );
    assert!(split > one + 3.0, "{split:+.1} dB against {one:+.1}");
}
