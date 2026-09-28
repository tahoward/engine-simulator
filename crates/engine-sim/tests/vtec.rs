//! Cam profile switching, as VTEC does it: a mild lobe below the switch speed and a wild one above it.

mod common;

use common::FS;
use engine_sim::EngineSim;
use engine_sim::spec::EngineSpec;
use serde_json::{Value, json};

/// `actual` within half a unit of `expected` in the `digits`th decimal place.
#[track_caller]
fn assert_close(actual: f64, expected: f64, digits: f64) {
    let tol = 10f64.powf(-digits) / 2.0;
    assert!((actual - expected).abs() < tol, "expected {actual} to be within {tol} of {expected}");
}

/// The F20C as the app loads it, with its cam switch.
fn f20c() -> &'static EngineSpec {
    &common::engine_preset("Inline four, Honda F20C").config.engine
}

fn build(over: Value) -> EngineSim {
    let mut cfg = common::engine_preset("Inline four, Honda F20C").config.clone();
    cfg.engine = common::with(&cfg.engine, json!({ "freeRunning": false, "throttle": 1, "combustionVariability": 0 }));
    cfg.engine = common::with(&cfg.engine, over);
    EngineSim::new(FS, &cfg)
}

fn torque_at(rpm: f64, over: Value) -> f64 {
    let mut patch = json!({ "rpm": rpm });
    patch.as_object_mut().unwrap().extend(over.as_object().unwrap().clone());
    let mut sim = build(patch);
    // The walls and the waves take a couple of seconds to settle at a new speed.
    sim.render(2 * FS as usize);
    let mut t = 0.0;
    for _ in 0..FS as usize / 2 {
        sim.render(1);
        t += sim.snapshot().torque;
    }
    t / (FS / 2.0)
}

/// The same engine on its high-speed cam alone.
fn high_only() -> Value {
    let e = f20c();
    json!({
        "camSwitchRpm": 0,
        "evo": e.high_evo,
        "evc": e.high_evc,
        "ivo": e.high_ivo,
        "ivc": e.high_ivc,
        "maxLift": e.high_max_lift,
    })
}

/// Has the mild lobe's low end and the wild lobe's top end.
///
/// The mild lobe gives back the low end the wild one costs, and the wild one keeps the top end.
#[test]
fn has_the_mild_lobes_low_end_and_the_wild_lobes_top_end() {
    let low = torque_at(3000.0, json!({}));
    let low_high = torque_at(3000.0, high_only());
    assert!(low > 1.2 * low_high, "switching {low} high only {low_high}");
    assert_close(torque_at(8000.0, json!({})) / torque_at(8000.0, high_only()), 1.0, 2.0);
}

/// Switches at its switch speed and back a little below it.
#[test]
fn switches_at_its_switch_speed_and_back_a_little_below_it() {
    let switch_rpm = f20c().cam_switch_rpm;
    let mut sim = build(json!({ "rpm": switch_rpm - 100.0 }));
    sim.render(FS as usize / 10);
    assert!(!sim.snapshot().high_cam);
    sim.set_engine_json(&json!({ "rpm": switch_rpm + 100.0 })).unwrap();
    sim.render(1);
    assert!(sim.snapshot().high_cam);
    // Inside the hysteresis band it stays on the high lobes.
    sim.set_engine_json(&json!({ "rpm": switch_rpm - 100.0 })).unwrap();
    sim.render(FS as usize / 10);
    assert!(sim.snapshot().high_cam);
    sim.set_engine_json(&json!({ "rpm": switch_rpm - 200.0 })).unwrap();
    sim.render(1);
    assert!(!sim.snapshot().high_cam);
}

/// Opens the valves to the high lobe's lift only on it.
#[test]
fn opens_the_valves_to_the_high_lobes_lift_only_on_it() {
    let switch_rpm = f20c().cam_switch_rpm;
    let peak_lift = |rpm: f64| {
        let mut sim = build(json!({ "rpm": rpm }));
        let mut peak: f64 = 0.0;
        for _ in 0..200 {
            sim.render(FS as usize / 1000);
            peak = peak.max(sim.snapshot().banks[0].in_lift);
        }
        peak
    };
    assert_close(peak_lift(switch_rpm - 1000.0), f20c().max_lift, 3.0);
    assert_close(peak_lift(switch_rpm + 1000.0), f20c().high_max_lift, 3.0);
}
