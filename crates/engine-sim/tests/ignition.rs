//! Switching the ignition off: the engine coasts to a standstill on its own friction and pumping, the
//! pipes ring down to silence, the manifold's vacuum bleeds away, and switching it back on starts it
//! again.

mod common;

use common::FS;
use engine_sim::EngineSim;
use engine_sim::spec::gas;
use serde_json::json;

fn idling() -> EngineSim {
    let mut cfg = common::default_config();
    cfg.engine = common::with(&cfg.engine, json!({ "freeRunning": true, "throttle": 0.05 }));
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(FS as usize);
    sim
}

fn peak(samples: &[f32]) -> f32 {
    samples.iter().fold(0.0, |m, s| m.max(s.abs()))
}

/// Off, it coasts down to a standstill within a few seconds, and the exhaust falls silent.
#[test]
fn coasts_to_a_stop_and_falls_silent() {
    let mut sim = idling();
    let running = peak(&sim.render(FS as usize / 2));
    sim.set_ignition(false);
    let mut stopped_after = None;
    for tenth in 0..200 {
        let out = sim.render(FS as usize / 10);
        assert!(out.iter().all(|s| s.is_finite()), "finite output while coasting");
        if sim.rpm_instant() < 1.0 {
            stopped_after = Some(tenth as f64 / 10.0);
            break;
        }
    }
    let stopped_after = stopped_after.expect("came to a standstill within 20 s");
    assert!(stopped_after > 0.2, "coasted rather than stopping dead: {stopped_after} s");
    // A second after the crank stops, the pipes have rung down.
    sim.render(FS as usize);
    let quiet = peak(&sim.render(FS as usize / 2));
    assert!(quiet < running * 0.01, "silent at rest: {quiet} against {running} running");
    assert!(sim.rpm_instant() < 1.0, "stayed stopped");
}

/// Off, the idle valve stays open where it was, and the manifold's vacuum bleeds away through it: back
/// within 1.5 kPa of the atmosphere a couple of seconds after the crank stops, where through the throttle
/// plate's clearance alone it takes ten.
#[test]
fn the_manifold_vacuum_bleeds_away_once_it_stops() {
    let preset = common::engine_preset("Inline four, Honda F20C");
    let mut cfg = preset.config.clone();
    cfg.engine = common::with(&cfg.engine, json!({ "freeRunning": true, "throttle": 0 }));
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(2 * FS as usize);
    let idle = sim.plenum().pressure() - gas::P_AMB;
    assert!(idle < -50e3, "deep in vacuum at idle: {idle} Pa");
    sim.set_ignition(false);
    let mut stopped = None;
    for tenth in 0..200 {
        sim.render(FS as usize / 10);
        if stopped.is_none() && sim.rpm_instant() < 1.0 {
            stopped = Some(tenth);
        }
        if let Some(at) = stopped {
            let gauge = sim.plenum().pressure() - gas::P_AMB;
            if gauge.abs() < 1500.0 {
                let after = (tenth - at) as f64 / 10.0;
                println!("vacuum bled away {after} s after the crank stopped");
                assert!(after < 2.5, "bled away {after} s after the crank stopped");
                return;
            }
        }
    }
    panic!("the manifold was still in vacuum 20 s after switching off");
}

/// On again from a standstill, it starts and runs.
#[test]
fn starts_again_after_stopping() {
    let mut sim = idling();
    sim.set_ignition(false);
    sim.render(20 * FS as usize);
    assert!(sim.rpm_instant() < 1.0, "stopped");
    sim.set_ignition(true);
    sim.render(2 * FS as usize);
    assert!(sim.rpm() > 450.0, "running again at {} rpm", sim.rpm());
    assert!(peak(&sim.render(FS as usize / 2)) > 0.0, "and making a sound");
}
