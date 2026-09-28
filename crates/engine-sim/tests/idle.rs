//! Every engine preset loads idling: in neutral, on a throttle found for it that holds
//! `PRESET_IDLE_RPM`. Run free, each has to settle there rather than stall or run away.
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
    "Inline four, Honda F20C",
    "Inline four, Toyota 3S-GTE",
    "Inline three",
    "Inline five",
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
    presets_idle_inline_four_honda_f20c => 3,
    presets_idle_inline_four_toyota_3s_gte => 4,
    presets_idle_inline_three => 5,
    presets_idle_inline_five => 6,
    presets_idle_inline_six_nissan_rb26dett => 7,
    presets_idle_v6_toyota_2gr => 8,
    presets_idle_v8_chevrolet_lt2 => 9,
    presets_idle_v8_chevrolet_lt6 => 10,
    presets_idle_boxer_four => 11,
    presets_idle_boxer_six => 12,
    presets_idle_parallel_twin_360 => 13,
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
