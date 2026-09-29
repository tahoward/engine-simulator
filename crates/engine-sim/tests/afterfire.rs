//! Afterfire: unburned fuel lighting in the exhaust. Spark cuts send whole charges down the pipe, and
//! the overrun crackle map sends some on a lift; a steady engine burns its fuel in the cylinder and
//! leaves none to light.

mod common;

use common::FS;
use engine_sim::EngineSim;
use serde_json::{Value, json};

/// `name`'s preset, running free on its own flywheel, with `over` on top.
fn free(name: &str, over: Value) -> EngineSim {
    let mut cfg = common::engine_preset(name).config.clone();
    cfg.engine = common::with(&cfg.engine, json!({ "freeRunning": true }));
    cfg.engine = common::with(&cfg.engine, over);
    EngineSim::new(FS, &cfg)
}

/// Render `seconds`, a snapshot every 10 ms, and say whether any snapshot had the crackle map running.
fn run(sim: &mut EngineSim, seconds: f64) -> bool {
    let mut crackled = false;
    for _ in 0..(seconds * 100.0) as usize {
        sim.render(FS as usize / 100);
        crackled |= sim.snapshot().crackle;
    }
    crackled
}

/// The LT2 revved to about 5000 rpm, short of its limiter, with `over` on top; the pops so far.
fn revved(over: Value) -> (EngineSim, u64) {
    let mut sim = free("V8, Chevrolet LT2", over);
    sim.set_controls(0.2, 0.0);
    run(&mut sim, 2.0);
    assert!(sim.rpm() > 3500.0 && sim.rpm() < 6000.0, "revved to {} rpm", sim.rpm());
    let events = sim.afterfire().events();
    (sim, events)
}

/// Launch control cuts the spark to hold the engine at the launch speed, and the charges it sends out
/// light in the header.
#[test]
fn launch_control_pops_in_the_header() {
    let preset = common::engine_preset("V8, Chevrolet LT6");
    let mut sim = EngineSim::new(FS, &preset.config);
    sim.render(FS as usize / 2);
    sim.start_launch(preset.launch.clone());
    while sim.snapshot().launch.is_some_and(|l| l.phase == "launch") {
        sim.render(FS as usize / 100);
    }
    assert!(sim.afterfire().events() > 0, "no afterfire off the line");
    assert!(sim.afterfire().heat_released() > 0.0);
}

/// Bouncing off the rev limiter pops.
#[test]
fn the_rev_limiter_pops() {
    let mut sim = free("V8, Chevrolet LT2", json!({}));
    sim.set_controls(1.0, 0.0);
    run(&mut sim, 3.0);
    assert!(sim.afterfire().events() > 20, "{} afterfires on the limiter", sim.afterfire().events());
}

/// With the crackle map, a lift pops, while the map runs.
#[test]
fn overrun_crackle_pops_on_lift_off() {
    let (mut sim, before) = revved(json!({ "overrunCrackle": true }));
    assert_eq!(before, 0, "pops before the lift");
    sim.set_controls(0.0, 0.0);
    assert!(run(&mut sim, 1.5), "the crackle map never ran");
    assert!(sim.afterfire().events() > 5, "{} afterfires on the lift", sim.afterfire().events());
}

/// Without it, the fuel is cut on a lift, and the air it pumps has nothing to burn.
#[test]
fn without_the_crackle_map_a_lift_off_is_silent_in_the_pipe() {
    let (mut sim, before) = revved(json!({}));
    sim.set_controls(0.0, 0.0);
    assert!(!run(&mut sim, 0.2));
    assert!(sim.snapshot().fuel_cut);
    assert!(!run(&mut sim, 1.3));
    assert_eq!(sim.afterfire().events(), before);
}

/// A more intense map pops more.
#[test]
fn a_more_intense_crackle_pops_more() {
    let pops = |intensity: f64| {
        let (mut sim, before) = revved(json!({ "overrunCrackle": true, "crackleIntensity": intensity }));
        sim.set_controls(0.0, 0.0);
        run(&mut sim, 1.5);
        sim.afterfire().events() - before
    };
    let (mild, wild) = (pops(0.2), pops(1.0));
    assert!(wild > 2 * mild, "{wild} afterfires at full intensity against {mild} at 0.2");
}

/// The map runs for a few seconds at most after a lift, and stops below its speed, when the fuel cut
/// takes over; opening the throttle again arms it for the next lift.
#[test]
fn the_crackle_map_ends_after_its_window_and_rearms_on_the_throttle() {
    let (mut sim, _) = revved(json!({ "overrunCrackle": true }));
    sim.set_controls(0.0, 0.0);
    assert!(run(&mut sim, 0.5));
    run(&mut sim, 3.0);
    assert!(!sim.snapshot().crackle, "still crackling at {} rpm", sim.rpm());
    assert!(sim.snapshot().fuel_cut || sim.rpm() < 1500.0);
    sim.set_controls(0.2, 0.0);
    run(&mut sim, 2.0);
    sim.set_controls(0.0, 0.0);
    assert!(run(&mut sim, 0.5), "not armed again");
}

/// Every preset, held steady at part and full throttle, burns its fuel in the cylinder and leaves
/// none to light in the pipe; nor does a rich mixture, which leaves fuel but no air to burn it. The
/// first cycles are let go by: a charge drawn from the still-empty manifold can misfire, and pop.
#[test]
fn a_steady_engine_never_afterfires() {
    for preset in &common::presets().engine_presets {
        for (throttle, lambda) in [(0.3, 1.0), (1.0, 1.0), (1.0, 0.8)] {
            let mut cfg = preset.config.clone();
            cfg.engine = common::with(
                &cfg.engine,
                json!({ "freeRunning": false, "rpm": 4200, "throttle": throttle, "lambda": lambda }),
            );
            let mut sim = EngineSim::new(FS, &cfg);
            sim.render(FS as usize / 2);
            let settled = sim.afterfire().events();
            sim.render(FS as usize);
            assert_eq!(
                sim.afterfire().events() - settled,
                0,
                "{} afterfires at throttle {throttle}, lambda {lambda}",
                preset.name
            );
        }
    }
}

/// The crackle map fires the spark well after top dead centre, and skips it on some cycles.
#[test]
fn the_crackle_map_fires_late_and_skips_some_sparks() {
    let (mut sim, _) = revved(json!({ "overrunCrackle": true, "crackleIntensity": 1.0 }));
    sim.set_controls(0.0, 0.0);
    let mut prev: Vec<f64> = sim.cylinders().iter().map(|c| c.angle).collect();
    let (mut cycles, mut unfired) = (0, 0);
    for _ in 0..FS as usize {
        sim.render(1);
        for (b, c) in sim.cylinders().iter().enumerate() {
            if prev[b] < 700.0 && c.angle >= 700.0 {
                assert!(c.spark > 30.0 && c.spark < 60.0, "spark at {}", c.spark);
            }
            if prev[b] < 240.0 && c.angle >= 240.0 {
                cycles += 1;
                unfired += (c.burned < 0.5) as u32;
            }
            prev[b] = c.angle;
        }
    }
    assert!(sim.snapshot().crackle);
    let share = unfired as f64 / cycles as f64;
    assert!(share > 0.2 && share < 0.5, "{unfired} of {cycles} cycles unfired");
}
