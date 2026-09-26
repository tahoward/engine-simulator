//! Real-time cost of the simulation, per engine preset: the fraction of one core it needs to make a
//! second of audio. `cargo run --release -p engine-sim --example bench`.
//!
//! The presets come from the parity fixture, so this measures exactly the engines the web app ships.
//! Each runs held at `BENCH_RPM` (6500 by default) at full throttle, the worst case for the solver,
//! warmed up for half a second, then timed over `BENCH_SECONDS` (3) and reported as the best of
//! `BENCH_REPEATS` (3): the work is deterministic, so anything slower than the fastest run is the
//! rest of the machine.

use std::time::Instant;

use engine_sim::{EngineConfig, EngineSim};
use serde::Deserialize;

#[derive(Deserialize)]
struct Fixture {
    compiled: Vec<Preset>,
}

#[derive(Deserialize)]
struct Preset {
    name: String,
    config: EngineConfig,
}

fn env(name: &str, default: f64) -> f64 {
    std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

fn main() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/scenarios.json");
    let fixture: Fixture = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    let fs = 48000.0;
    let rpm = env("BENCH_RPM", 6500.0);
    let seconds = env("BENCH_SECONDS", 3.0);
    let repeats = env("BENCH_REPEATS", 3.0) as usize;
    let only = std::env::var("BENCH_ONLY").ok();

    let width = fixture.compiled.iter().map(|p| p.name.chars().count()).max().unwrap_or(10);
    println!("{:width$}  cells  substeps  % of one core @ {rpm} rpm  (spread)", "preset");
    let mut worst = (String::new(), 0.0f64);
    for p in &fixture.compiled {
        if only.as_deref().is_some_and(|o| !p.name.contains(o)) {
            continue;
        }
        let mut cfg = p.config.clone();
        cfg.engine.rpm = rpm;
        cfg.engine.throttle = 1.0;
        cfg.engine.free_running = false;
        let mut sim = EngineSim::new(fs, &cfg);
        let mut buf = vec![0.0f32; (fs * 0.5) as usize];
        sim.render_into(&mut buf);
        let mut buf = vec![0.0f32; (fs * seconds) as usize];
        let mut runs = Vec::new();
        for _ in 0..repeats {
            let t0 = Instant::now();
            sim.render_into(&mut buf);
            runs.push(t0.elapsed().as_secs_f64() / seconds);
        }
        let best = runs.iter().copied().fold(f64::INFINITY, f64::min);
        let spread = runs.iter().copied().fold(0.0, f64::max) - best;
        let snap = sim.snapshot();
        println!(
            "{:width$}  {:>5}  {:>8}  {:>10.1}%  {:>8.1}",
            p.name,
            sim.pipe_solver().cells(),
            snap.substeps,
            best * 100.0,
            spread * 100.0
        );
        if best > worst.1 {
            worst = (p.name.clone(), best);
        }
    }
    println!("\nworst: {} at {:.1}%", worst.0, worst.1 * 100.0);
}
