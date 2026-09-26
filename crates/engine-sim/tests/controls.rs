//! `set_controls` is the worklet's per-block fast path for throttle and load. It must do exactly what
//! `set_engine` does with the same two values, or the sound would depend on which path a change
//! happened to take.

mod common;

use common::FS;
use engine_sim::EngineSim;
use serde_json::{Value, json};

/// Two identical sims, each a quarter of a second in.
fn pair(engine: Value) -> (EngineSim, EngineSim) {
    let make = || {
        let mut cfg = common::default_config();
        cfg.engine = common::with(&cfg.engine, engine.clone());
        cfg.pipe = common::pipe_preset(1);
        let mut sim = EngineSim::new(FS, &cfg);
        sim.render(FS as usize / 4);
        sim
    };
    (make(), make())
}

fn matches_set_engine_bit_for_bit(free_running: bool) {
    let (mut a, mut b) = pair(json!({ "freeRunning": free_running, "throttle": 0.3, "rpm": 3000, "load": 0.4 }));
    a.set_engine_json(&json!({ "throttle": 0.8, "load": 0.7 })).unwrap();
    b.set_controls(0.8, 0.7);
    let (ra, rb) = (a.render(FS as usize / 2), b.render(FS as usize / 2));
    assert!(ra.iter().zip(&rb).all(|(x, y)| x.to_bits() == y.to_bits()));
}

/// Matches set_engine bit for bit (fixed rpm).
#[test]
fn set_controls_matches_set_engine_bit_for_bit_fixed_rpm() {
    matches_set_engine_bit_for_bit(false);
}

/// Matches set_engine bit for bit (free-running).
#[test]
fn set_controls_matches_set_engine_bit_for_bit_free_running() {
    matches_set_engine_bit_for_bit(true);
}

/// Changes nothing when the values have not moved.
#[test]
fn set_controls_changes_nothing_when_the_values_have_not_moved() {
    let (mut a, mut b) = pair(json!({ "throttle": 0.5, "rpm": 4000, "load": 0.3 }));
    b.set_controls(0.5, 0.3);
    let (ra, rb) = (a.render(FS as usize / 4), b.render(FS as usize / 4));
    assert!(ra.iter().zip(&rb).all(|(x, y)| x.to_bits() == y.to_bits()));
}
