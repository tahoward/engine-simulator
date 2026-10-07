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

/// A turbo diesel's plenum, wide open to its turbo's throttle body, holds what the turbo makes and no
/// more: idling, its air is no colder than the atmosphere it came from and the plenum sits at the charge
/// pressure, with no flow swinging back and forth between the two each sample; switched off, it bleeds
/// back to the atmosphere as the turbo coasts down.
#[test]
fn a_turbo_diesels_plenum_bleeds_away_once_it_stops() {
    let mut cfg = common::engine_preset("Inline six diesel, Cummins 6CT").config.clone();
    cfg.engine = common::with(&cfg.engine, json!({ "freeRunning": true, "throttle": 0 }));
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(3 * FS as usize);
    let turbo = sim.turbo().unwrap();
    assert!(turbo.charge_temp() > gas::T_AMB - 1.0, "charge air at {} K", turbo.charge_temp());
    let across = sim.plenum().throttle_pressure() - turbo.charge_pressure();
    assert!(across.abs() < 1000.0, "{across} Pa across the throttle at idle");
    sim.set_ignition(false);
    sim.render(5 * FS as usize);
    let mut largest = 0.0f64;
    for _ in 0..FS as usize / 10 {
        sim.render(1);
        largest = largest.max(sim.plenum().throttle_flows()[0].abs());
    }
    assert!(largest < 0.01, "{largest} kg/s through the throttle at rest");
    sim.render(15 * FS as usize);
    let gauge = sim.plenum().pressure() - gas::P_AMB;
    assert!(gauge.abs() < 1000.0, "{gauge} Pa in the plenum 20 s after switching off");
}

/// A naturally aspirated diesel draws through a throttle held wide open, between its inlet tract and its
/// plenum: stopped, nothing goes on flowing back and forth through it, so its intake falls quiet as the
/// crank does rather than hissing on.
#[test]
fn a_naturally_aspirated_diesels_intake_falls_quiet_once_it_stops() {
    let mut cfg = common::engine_preset("Inline four, Honda F20C").config.clone();
    cfg.engine = common::with(
        &cfg.engine,
        json!({ "freeRunning": true, "fuel": "diesel", "compressionRatio": 18, "ignition": 708, "maxFuel": 4e-5 }),
    );
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(3 * FS as usize);
    let idling = sim.plenum().throttle_flows()[0].abs();
    sim.set_ignition(false);
    sim.render(FS as usize);
    assert!(sim.rpm_instant() < 1.0, "stopped");
    let (mut flow, mut mouth) = (0.0f64, 0.0f64);
    for _ in 0..FS as usize / 4 {
        sim.render(1);
        flow = flow.max(sim.plenum().throttle_flows()[0].abs());
        mouth = mouth.max(sim.inlet().unwrap().mouth_flow.abs());
    }
    assert!(flow < 1e-3, "{flow} kg/s through the throttle at rest, {idling} idling");
    assert!(mouth < 1e-3, "{mouth} m^3/s at the snorkel's mouth at rest");
}
