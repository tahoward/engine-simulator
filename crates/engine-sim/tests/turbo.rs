//! The turbocharger: boost built by the exhaust and held by the wastegate, the lag behind the
//! throttle, what the blow-off valve does when the throttle shuts and what happens without one, and
//! the whine of the compressor.
//!
//! The Nissan RB26DETT preset is the turbocharged engine throughout.

mod common;

use common::FS;
use engine_sim::EngineSim;
use serde_json::{Value, json};

const RB26: &str = "Inline six, Nissan RB26DETT";

fn rb26(over: Value) -> EngineSim {
    let mut cfg = common::engine_preset(RB26).config.clone();
    cfg.engine = common::with(&cfg.engine, json!({ "freeRunning": false, "combustionVariability": 0 }));
    cfg.engine = common::with(&cfg.engine, over);
    EngineSim::new(FS, &cfg)
}

/// Boost, bar gauge.
fn boost(sim: &EngineSim) -> f64 {
    sim.turbo().unwrap().boost() / 1e5
}

/// Mean torque over half a second, N*m, after `settle` seconds at `rpm` on full throttle.
fn torque_at(rpm: f64, settle: f64) -> f64 {
    let mut sim = rb26(json!({ "throttle": 1, "rpm": rpm }));
    sim.render((settle * FS) as usize);
    let n = FS as usize / 2;
    let mut t = 0.0;
    for _ in 0..n {
        sim.render(1);
        t += sim.snapshot().torque;
    }
    t / n as f64
}

/// Held on boost at `rpm`, then the throttle shut: the compressor flow every sample for half a second.
fn lift_off(over: Value) -> (EngineSim, Vec<f64>) {
    let mut o = json!({ "throttle": 1, "rpm": 4000 });
    o.as_object_mut().unwrap().extend(over.as_object().unwrap().clone());
    let mut sim = rb26(o);
    sim.render(3 * FS as usize);
    assert!(boost(&sim) > 0.6, "on boost before the lift: {}", boost(&sim));
    sim.set_controls(0.0, 0.0);
    let mut flow = Vec::new();
    for _ in 0..FS as usize / 2 {
        sim.render(1);
        flow.push(sim.turbo().unwrap().compressor_flow());
    }
    (sim, flow)
}

#[test]
fn a_naturally_aspirated_engine_has_no_turbo() {
    let mut sim = EngineSim::new(FS, &common::engine_preset("Inline four, Honda F20C").config);
    sim.render(FS as usize / 10);
    assert!(sim.turbo().is_none());
    assert!(sim.snapshot().turbo.is_none());
}

#[test]
fn switching_the_turbo_off_leaves_the_engine_breathing_the_atmosphere() {
    let mut sim = rb26(json!({ "throttle": 1, "rpm": 5000 }));
    sim.render(2 * FS as usize);
    assert!(boost(&sim) > 0.5);
    sim.set_engine_json(&json!({ "turbo": false })).unwrap();
    sim.render(FS as usize);
    assert!(sim.turbo().is_none());
    assert!(sim.snapshot().turbo.is_none());
    let manifold = sim.plenum().pressure();
    assert!(manifold < 1.02e5, "no boost left: {manifold}");
}

/// Boost builds with the exhaust flow, and the wastegate holds it at its target.
#[test]
fn builds_boost_with_the_exhaust_and_the_wastegate_holds_it() {
    let at = |rpm: f64| {
        let mut sim = rb26(json!({ "throttle": 1, "rpm": rpm }));
        sim.render(3 * FS as usize);
        (boost(&sim), sim.turbo().unwrap().wastegate())
    };
    let (low, _) = at(1500.0);
    let (mid, wg_mid) = at(4000.0);
    let (high, wg_high) = at(6500.0);
    assert!(low < 0.5, "too little exhaust at 1500 rpm for full boost: {low}");
    assert!((mid - 0.7).abs() < 0.06, "held at the target at 4000 rpm: {mid}");
    assert!((high - 0.7).abs() < 0.06, "held at the target at 6500 rpm: {high}");
    assert!(wg_high > wg_mid && wg_mid > 0.0, "the wastegate opens further with more exhaust: {wg_mid} {wg_high}");
}

/// The shaft takes time to spin up: opened from part throttle, the boost lags behind.
#[test]
fn lags_behind_the_throttle() {
    let mut sim = rb26(json!({ "throttle": 0.12, "rpm": 3500 }));
    sim.render(2 * FS as usize);
    assert!(boost(&sim) < 0.1, "off boost at part throttle: {}", boost(&sim));
    sim.set_controls(1.0, 0.0);
    let mut reached = None;
    for i in 1..=60 {
        sim.render(FS as usize / 20);
        if boost(&sim) > 0.9 * 0.7 {
            reached = Some(i as f64 / 20.0);
            break;
        }
    }
    let t = reached.expect("reaches full boost within 3 s");
    assert!(t > 0.2 && t < 2.5, "90% of the boost after {t} s");
}

/// About the real engine's 368 N*m at 4400 rpm, and its power at 6800 no less than the 280 PS it was
/// rated at nor much more than the 320 or so real ones make.
#[test]
fn makes_about_the_real_engines_torque_and_power() {
    let t4400 = torque_at(4400.0, 3.0);
    assert!((t4400 - 368.0).abs() < 0.1 * 368.0, "{t4400} N*m at 4400 rpm");
    let t6800 = torque_at(6800.0, 3.0);
    let ps = t6800 * 6800.0 * 2.0 * std::f64::consts::PI / 60.0 / 735.5;
    assert!(ps > 280.0 && ps < 350.0, "{ps} PS at 6800 rpm");
}

/// A blow-off valve vents the charge when the throttle shuts, so the compressor never runs backwards.
#[test]
fn a_blow_off_valve_vents_the_charge_when_the_throttle_shuts() {
    let (sim, flow) = lift_off(json!({ "blowOff": "atmospheric" }));
    assert!(flow.iter().all(|&m| m > 0.0), "the compressor never surges");
    assert!(sim.turbo().unwrap().blow_off() > 0.9, "the valve is open on the vacuum");
    assert!(boost(&sim) < 0.2, "the boost is let go: {}", boost(&sim));
}

/// With nowhere for the charge to go, the compressor stalls and recovers over and over: a surge,
/// cycling at a few tens of hertz, which is the flutter.
#[test]
fn without_a_blow_off_valve_the_compressor_surges() {
    let (_, flow) = lift_off(json!({ "blowOff": "none" }));
    let mut reversals = 0;
    for w in flow.windows(2) {
        if w[0] >= 0.0 && w[1] < 0.0 {
            reversals += 1;
        }
    }
    let hz = reversals as f64 / 0.5;
    assert!(hz > 5.0 && hz < 60.0, "{reversals} surge cycles in half a second");
}

/// On boost, the turbo's sound has its strongest tone at the compressor's blade-pass frequency.
#[test]
fn whines_at_the_blade_pass_frequency() {
    let render = |noise: f64| {
        let mut sim = rb26(json!({ "throttle": 1, "rpm": 2500, "turboNoise": noise }));
        sim.render(3 * FS as usize);
        let bpf = sim.turbo().unwrap().blade_pass_hz();
        (sim.render(16384), bpf)
    };
    let (with, bpf) = render(1.0);
    let (without, _) = render(0.0);
    assert!(bpf > 3000.0 && bpf < 12000.0, "blade pass {bpf} Hz");
    let turbo: Vec<f32> = with.iter().zip(&without).map(|(a, b)| a - b).collect();
    let size = 16384;
    let mag = common::magnitude_spectrum(&common::hann(&turbo), size);
    let peaks = common::find_peaks(&mag, FS, size, 1000.0, 16000.0, 0.1);
    let top = peaks[0];
    assert!((top.hz - bpf).abs() < 0.03 * bpf, "strongest tone {} Hz, blade pass {bpf} Hz", top.hz);
}
