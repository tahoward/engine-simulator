//! Cam profile switching, as VTEC does it: a mild lobe below the switch speed and a wild one above it.

use crate::common;

use common::{FS, assert_close};
use engine_sim::spec::EngineSpec;
use serde_json::{Value, json};

const F20C: &str = "Inline four, Honda F20C";

/// The F20C as the app loads it, with its cam switch.
fn f20c() -> &'static EngineSpec {
    &common::engine_preset(F20C).config.engine
}

/// The gas's mean torque, N*m, held at `rpm` with `over` on top.
fn torque_at(rpm: f64, over: Value) -> f64 {
    let mut sim = common::held(F20C, rpm, over);
    // The walls and the waves take a couple of seconds to settle at a new speed.
    sim.render(2 * FS as usize);
    common::gas_torque(&mut sim)
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
    let points = [(3000.0, json!({})), (3000.0, high_only()), (8000.0, json!({})), (8000.0, high_only())];
    let [low, low_high, top, top_high] = common::par(points, |(rpm, over)| torque_at(*rpm, over.clone()));
    assert!(low > 1.2 * low_high, "switching {low} high only {low_high}");
    assert_close(top / top_high, 1.0, 2.0);
}

/// Switches at its switch speed and back a little below it.
#[test]
fn switches_at_its_switch_speed_and_back_a_little_below_it() {
    let switch_rpm = f20c().cam_switch_rpm;
    let mut sim = common::held(F20C, switch_rpm - 100.0, json!({}));
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
        let mut sim = common::held(F20C, rpm, json!({}));
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
