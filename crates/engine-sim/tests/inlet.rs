//! The inlet tract: the air drawn in through the snorkel and airbox to the throttle, its note, and the
//! hiss of the jet past the throttle plate.

use crate::common;

use common::{FS, hann, magnitude_spectrum};
use engine_sim::EngineSim;
use engine_sim::euler_pipe::EulerPipeOptions;
use engine_sim::inlet::InletTract;
use serde_json::{Value, json};

const N: usize = 16384;

/// The F20C as the app loads it, held at `rpm` and `throttle` with the fields in `over`, settled.
fn f20c(rpm: f64, throttle: f64, over: Value) -> EngineSim {
    let preset = common::engine_preset("Inline four, Honda F20C");
    let mut cfg = preset.config.clone();
    cfg.engine = common::with(
        &cfg.engine,
        json!({ "rpm": rpm, "throttle": throttle, "freeRunning": false, "combustionVariability": 0 }),
    );
    cfg.engine = common::with(&cfg.engine, over);
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(2 * FS as usize);
    sim
}

/// The volume flow out of the snorkel's mouth over the next `N` samples, m^3/s.
fn mouth_flow(sim: &mut EngineSim) -> Vec<f32> {
    (0..N)
        .map(|_| {
            sim.tick();
            sim.inlet().expect("an engine without a turbo has an inlet").mouth_flow as f32
        })
        .collect()
}

/// Energy of `flow` between `lo` and `hi` Hz.
fn band(flow: &[f32], lo: f64, hi: f64) -> f64 {
    let mag = magnitude_spectrum(&hann(flow), N);
    let bin = FS / N as f64;
    ((lo / bin) as usize..(hi / bin) as usize).map(|i| mag[i] * mag[i]).sum()
}

/// The engine draws its air in through the snorkel, as much as it burns.
#[test]
fn the_engine_draws_its_air_through_the_snorkel() {
    let mut sim = f20c(6000.0, 1.0, json!({}));
    let flow = mouth_flow(&mut sim);
    let mean = flow.iter().map(|&q| q as f64).sum::<f64>() / N as f64;
    // Half its two litres a revolution at full throttle, drawn in, so out of the mouth is negative.
    let per_rev = -mean / (6000.0 / 60.0);
    assert!(per_rev > 0.0008 && per_rev < 0.0012, "{per_rev} m^3 a revolution");
}

/// The runners' pulses reach the mouth: its flow pulses at the firing frequency, far above where the
/// engine has no order at all. The throttle at the plenum's front hears the cylinders nearest it loudest,
/// so the half orders between the firing order and the next are there too, as they are at a real snorkel.
#[test]
fn the_runners_pulses_reach_the_mouth() {
    let rpm = 6000.0;
    let firing = rpm / 30.0;
    let mut sim = f20c(rpm, 1.0, json!({ "throatNoise": 0 }));
    let flow = mouth_flow(&mut sim);
    let at = band(&flow, firing * 0.95, firing * 1.05);
    // Between the firing order and the half order after it, a quarter of the firing frequency on.
    let between = band(&flow, firing * 1.08, firing * 1.17);
    println!("firing order {:.1} dB over the band between", 10.0 * (at / between).log10());
    assert!(at > between * 10.0, "firing {at} against between {between}");
}

/// A longer snorkel tunes the tract lower.
#[test]
fn a_longer_snorkel_tunes_the_tract_lower() {
    // A puff drawn through the throttle rings the tract at its lowest resonance, the airbox's air
    // bouncing on the column in the snorkel, and the mouth breathes at it.
    let peak = |length: f64| {
        let spec = common::with(
            &common::engine_preset("Inline four, Honda F20C").config.engine,
            json!({ "snorkelLength": length }),
        );
        let opts = EulerPipeOptions { cell_size: Some(0.035), ..Default::default() };
        let mut tract = InletTract::new(&spec, FS, &opts);
        let flow: Vec<f32> = (0..N)
            .map(|i| {
                tract.advance(1.0 / FS, if i < 24 { 0.01 } else { 0.0 }, 0.003, 0.06, 0.0);
                tract.mouth_flow as f32
            })
            .collect();
        let mag = magnitude_spectrum(&hann(&flow), N);
        let bin = FS / N as f64;
        let (lo, hi) = ((20.0 / bin) as usize, (1000.0 / bin) as usize);
        (lo..hi).max_by(|&a, &b| mag[a].total_cmp(&mag[b])).unwrap() as f64 * bin
    };
    let (short, long) = (peak(0.15), peak(0.6));
    println!("tract rings at {short:.0} Hz with a 0.15 m snorkel, {long:.0} Hz with a 0.6 m one");
    assert!(long < short * 0.85, "peak {long} Hz with a long snorkel, {short} Hz with a short one");
}

/// The jet past a nearly shut throttle hisses higher than the air through a wide-open one rushes.
#[test]
fn a_nearly_shut_throttle_hisses_higher_than_a_wide_open_one() {
    let centroid = |throttle: f64| {
        let noise = |level: f64| mouth_flow(&mut f20c(3000.0, throttle, json!({ "throatNoise": level })));
        let (on, off) = (noise(1.0), noise(0.0));
        let diff: Vec<f32> = on.iter().zip(&off).map(|(a, b)| a - b).collect();
        let mag = magnitude_spectrum(&hann(&diff), N);
        let bin = FS / N as f64;
        let (mut num, mut den) = (0.0, 0.0);
        for (i, m) in mag.iter().enumerate().skip(1) {
            num += i as f64 * bin * m * m;
            den += m * m;
        }
        num / den
    };
    let (shut, open) = (centroid(0.05), centroid(1.0));
    println!("throttle noise centred at {shut:.0} Hz nearly shut, {open:.0} Hz wide open");
    assert!(shut > open * 1.5, "noise centred at {shut} Hz nearly shut, {open} Hz wide open");
}

/// A turbocharged engine draws through its compressors instead, and has no tract.
#[test]
fn a_turbocharged_engine_has_no_tract() {
    let preset = common::engine_preset("Inline six, Nissan RB26DETT");
    let sim = EngineSim::new(FS, &preset.config);
    assert!(sim.inlet().is_none());
}
