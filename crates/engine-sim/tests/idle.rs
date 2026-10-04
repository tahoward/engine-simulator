//! Every engine preset loads idling: in neutral, with the throttle shut and the idle air valve holding
//! `PRESET_IDLE_RPM`. Run free, each has to settle there rather than stall or run away, catch itself
//! there coming down off a lift, come back to it when a load is taken off, and stall under a load the
//! valve cannot hold up.
//!
//! Also a readout, with no assertions, of every preset run free on a shut throttle and a few on a
//! mid-throttle hold: `--nocapture` prints how the speed and manifold pressure evolve.

mod common;

use common::FS;
use engine_sim::EngineSim;
use engine_sim::spec::gas;
use serde_json::json;

/// The speed every preset loads at, rpm.
const PRESET_IDLE_RPM: f64 = 800.0;

/// `name`, run free from its preset config, settles near the idle speed.
fn settles_near_the_idle_speed(name: &str) {
    let mut cfg = common::engine_preset(name).config.clone();
    cfg.engine = common::with(&cfg.engine, json!({ "freeRunning": true }));
    assert_eq!(cfg.engine.rpm, PRESET_IDLE_RPM);
    assert_eq!(cfg.engine.idle_rpm, PRESET_IDLE_RPM);
    assert_eq!(cfg.engine.throttle, 0.0);
    assert_eq!(cfg.engine.load, 0.0);

    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(FS as usize * 3);
    // Averaged, because an idle hunts.
    let mut sum = 0.0;
    let reads = 20;
    for _ in 0..reads {
        sim.render(FS as usize / 10);
        sum += sim.rpm();
    }
    let mean = sum / reads as f64;
    assert!(mean > PRESET_IDLE_RPM - 150.0, "{name}: mean {mean}");
    assert!(mean < PRESET_IDLE_RPM + 200.0, "{name}: mean {mean}");
}

/// Every preset has an idle test below.
const PRESETS: [&str; 14] = [
    "Single, megaphone",
    "45° V-twin, 2-into-1",
    "90° V-twin, 2-into-2",
    "Inline three, Ford 1.5 EcoBoost Dragon",
    "Inline four, Honda F20C",
    "Inline four, Toyota 3S-GTE",
    "Inline five, Audi EA855 EVO",
    "Inline six, Nissan RB26DETT",
    "V6, Toyota 2GR",
    "V8, Chevrolet LT2",
    "V8, Chevrolet LT6",
    "Boxer four",
    "Boxer six",
    "Parallel twin, 360°",
];

/// The idle tests below cover every preset.
#[test]
fn presets_idle_covers_every_preset() {
    let names: Vec<&str> = common::presets().engine_presets.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, PRESETS);
}

macro_rules! idle_tests {
    ($($test:ident => $index:expr),* $(,)?) => {
        $(
            #[test]
            fn $test() {
                settles_near_the_idle_speed(PRESETS[$index]);
            }
        )*
    };
}

// Each preset settles near the idle speed.
idle_tests! {
    presets_idle_single_megaphone => 0,
    presets_idle_45_v_twin_2_into_1 => 1,
    presets_idle_90_v_twin_2_into_2 => 2,
    presets_idle_inline_three_ford_1_5_ecoboost_dragon => 3,
    presets_idle_inline_four_honda_f20c => 4,
    presets_idle_inline_four_toyota_3s_gte => 5,
    presets_idle_inline_five_audi_ea855_evo => 6,
    presets_idle_inline_six_nissan_rb26dett => 7,
    presets_idle_v6_toyota_2gr => 8,
    presets_idle_v8_chevrolet_lt2 => 9,
    presets_idle_v8_chevrolet_lt6 => 10,
    presets_idle_boxer_four => 11,
    presets_idle_boxer_six => 12,
    presets_idle_parallel_twin_360 => 13,
}

// --- the idle air valve ---

fn idling(name: &str, over: serde_json::Value) -> EngineSim {
    let mut cfg = common::engine_preset(name).config.clone();
    cfg.engine = common::with(&cfg.engine, json!({ "freeRunning": true }));
    cfg.engine = common::with(&cfg.engine, over);
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(FS as usize * 3);
    sim
}

/// The slowest and the mean speed over `seconds`, rpm, read every tenth of a second.
fn slowest_and_mean(sim: &mut EngineSim, seconds: usize) -> (f64, f64) {
    let (mut slowest, mut sum) = (f64::INFINITY, 0.0);
    for _ in 0..seconds * 10 {
        sim.render(FS as usize / 10);
        slowest = slowest.min(sim.rpm());
        sum += sim.rpm();
    }
    (slowest, sum / (seconds * 10) as f64)
}

/// Lifting off after a rev, the fuel cut lets the engine fall, and the valve's dashpot catches it at
/// the idle rather than letting it fall through and stall.
#[test]
fn comes_down_off_a_lift_to_the_idle_without_stalling() {
    for name in ["Single, megaphone", "Inline four, Honda F20C", "V8, Chevrolet LT2", "Inline six, Nissan RB26DETT"] {
        let mut sim = idling(name, json!({}));
        sim.set_controls(0.6, 0.0);
        sim.render(FS as usize * 3 / 2);
        sim.set_controls(0.0, 0.0);
        // Ten seconds: the single's heavy flywheel takes most of them to come down.
        let (slowest, _) = slowest_and_mean(&mut sim, 10);
        let (_, mean) = slowest_and_mean(&mut sim, 3);
        assert!(slowest > 550.0, "{name}: fell to {slowest} rpm");
        assert!((mean - PRESET_IDLE_RPM).abs() < 150.0, "{name}: back at {mean} rpm");
    }
}

/// The idle speed is the valve's to set.
#[test]
fn holds_the_idle_speed_it_is_set_to() {
    for name in ["Single, megaphone", "V8, Chevrolet LT2"] {
        let mut sim = idling(name, json!({ "idleRpm": 1100 }));
        sim.render(FS as usize * 3);
        let (_, mean) = slowest_and_mean(&mut sim, 2);
        assert!((mean - 1100.0).abs() < 150.0, "{name}: idles at {mean} rpm");
    }
}

/// The valve opens further to hold the idle against a load; when the load goes, the engine rises past
/// the hold, and the valve must still wind back down to the idle rather than keep the engine up there.
#[test]
fn comes_back_to_the_idle_when_a_load_is_taken_off() {
    for name in ["Inline four, Toyota 3S-GTE", "Inline six, Nissan RB26DETT"] {
        let mut sim = idling(name, json!({}));
        sim.set_controls(0.0, 0.07);
        sim.render(FS as usize * 10);
        sim.set_controls(0.0, 0.0);
        sim.render(FS as usize * 10);
        let (_, mean) = slowest_and_mean(&mut sim, 3);
        assert!((mean - PRESET_IDLE_RPM).abs() < 150.0, "{name}: settled at {mean} rpm");
    }
}

/// A load past what the valve can open against stalls the engine: it comes to a standstill and stays.
#[test]
fn stalls_under_a_load_it_cannot_hold() {
    for name in ["Single, megaphone", "Inline four, Honda F20C", "V8, Chevrolet LT2"] {
        let mut sim = idling(name, json!({}));
        sim.set_controls(0.0, 1.0);
        sim.render(FS as usize * 3);
        let out = sim.render(FS as usize);
        assert!(sim.rpm_instant() < 1.0, "{name}: still turning at {} rpm", sim.rpm_instant());
        assert!(out.iter().all(|s| s.is_finite()), "{name}: finite at a standstill");
    }
}

// --- closed throttle must not run away ---

/// Twelve seconds free, printing the speed at 2, 4, 8 and 12 s.
fn speed_marks(sim: &mut EngineSim) -> String {
    let mut marks = Vec::new();
    for t in [2, 2, 4, 4] {
        sim.render(FS as usize * t);
        marks.push(format!("{:>5.0}", sim.rpm()));
    }
    marks.join(" ")
}

/// Every engine preset, throttle 0, free-running, no load.
#[test]
fn closed_throttle_every_engine_preset_throttle_0_free_running_no_load() {
    println!("\n  preset                              rpm @ 2s   4s     8s    12s   MAP");
    for p in &common::presets().engine_presets {
        let mut cfg = p.config.clone();
        cfg.engine = common::with(&cfg.engine, json!({ "throttle": 0, "freeRunning": true, "load": 0 }));
        let mut sim = EngineSim::new(FS, &cfg);
        let marks = speed_marks(&mut sim);
        let mut sum = 0.0;
        for _ in 0..2000 {
            sim.render(12);
            sum += sim.plenum().pressure();
        }
        println!("  {:<34} {}   {:.2}bar", p.name, marks, sum / 2000.0 / gas::P_AMB);
    }
}

/// And a mid-throttle hold is stable too.
#[test]
fn closed_throttle_and_a_mid_throttle_hold_is_stable_too() {
    println!();
    for name in ["Single", "Inline four", "V8, Chevrolet LT2"] {
        let p = common::presets().engine_presets.iter().find(|x| x.name.starts_with(name)).unwrap();
        let mut cfg = p.config.clone();
        cfg.engine = common::with(&cfg.engine, json!({ "throttle": 0.3, "freeRunning": true, "load": 0.3 }));
        let mut sim = EngineSim::new(FS, &cfg);
        let marks = speed_marks(&mut sim);
        println!("  {:<34} {}  (throttle 0.3, 25 Nm)", p.name, marks);
    }
}

/// Redrawing the exhaust while the engine idles leaves the idle as it was: the intake runners keep their
/// gas, so the cylinders do not get a few full charges from runners refilled at the atmosphere's pressure.
/// Against the same engine left alone, over the second after, run free.
#[test]
fn redrawing_the_exhaust_does_not_rev_the_idle() {
    for name in ["Inline four, Honda F20C", "V8, Chevrolet LT2", "Inline six, Nissan RB26DETT"] {
        let mut cfg = common::engine_preset(name).config.clone();
        cfg.engine = common::with(&cfg.engine, json!({ "freeRunning": true }));
        let top = |sim: &mut EngineSim| {
            let mut hi = 0.0f64;
            for _ in 0..100 {
                sim.render(FS as usize / 100);
                hi = hi.max(sim.rpm());
            }
            hi
        };
        let mut left = EngineSim::new(FS, &cfg);
        let mut redrawn = EngineSim::new(FS, &cfg);
        left.render(FS as usize * 3);
        redrawn.render(FS as usize * 3);
        // A tailpipe 5 mm longer, drawn or compiled.
        match cfg.graph.clone() {
            Some(mut graph) => {
                let mouth = graph.ducts.iter_mut().find(|d| d.vents()).unwrap();
                mouth.segments.last_mut().unwrap().length += 0.005;
                redrawn.set_graph(Some(graph));
            }
            None => {
                let mut pipe = cfg.pipe.clone();
                pipe.last_mut().unwrap().length += 0.005;
                redrawn.set_pipe(&pipe, None);
            }
        }
        let (alone, after) = (top(&mut left), top(&mut redrawn));
        assert!(after < alone + 30.0, "{name}: up to {after} rpm redrawn, {alone} left alone");
    }
}

/// An edit the solver would build the same, as placing a loose pipe is, whose only trace in the graph is
/// the other pipes' headings frozen where they lie, leaves the exhaust running as it was: its gas, and so
/// the idle, untouched.
#[test]
fn an_edit_that_only_moves_the_drawing_leaves_the_exhaust_running() {
    let mut cfg = common::engine_preset("Inline six, Nissan RB26DETT").config.clone();
    cfg.engine = common::with(&cfg.engine, json!({ "freeRunning": true }));
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(FS as usize * 2);
    let mut graph = cfg.graph.clone().unwrap();
    for d in graph.ducts.iter_mut() {
        d.heading_yaw = Some(d.heading_yaw.unwrap_or(0.0) + 0.1);
        d.heading_frame = Some("world".into());
    }
    for t in graph.turbos.iter_mut() {
        t.position = Some([0.1, 0.2, 0.3]);
    }
    let pressure = sim.pipe_solver().primary(0).pressure_at(2);
    sim.set_graph(Some(graph));
    assert_eq!(sim.pipe_solver().primary(0).pressure_at(2), pressure);
}
