//! A dyno pull: the engine held at the start speed on an absorber at 1:1, then swept up at a steady rate
//! to the end, and the power curve it leaves.

mod common;

use common::FS;
use engine_sim::spec::LaunchSnapshot;
use engine_sim::{EngineSim, LaunchConfig};
use std::f64::consts::PI;

const ENGINE: &str = "Inline four, Honda F20C";

/// The preset's run as a dyno pull from `from` to `to` rpm at `rate` rpm/s.
fn dyno(from: f64, to: f64, rate: f64) -> LaunchConfig {
    LaunchConfig {
        dyno: true,
        ratios: vec![1.0],
        final_drive: 1.0,
        launch_rpm: from,
        shift_rpm: to,
        sweep_rate: rate,
        ..common::engine_preset(ENGINE).launch.clone()
    }
}

/// Every snapshot taken while `config`'s run lasted, one per 20 ms, idling first.
fn run(config: LaunchConfig) -> Vec<LaunchSnapshot> {
    let preset = common::engine_preset(ENGINE);
    let mut sim = EngineSim::new(FS, &preset.config);
    sim.render(FS as usize / 2);
    sim.start_launch(config);
    let mut out = Vec::new();
    let block = (FS / 50.0) as usize;
    for _ in 0..(180 * 50) {
        sim.render(block);
        match sim.snapshot().launch {
            Some(s) => out.push(s),
            None => break,
        }
    }
    out
}

/// Every recorded cycle's rpm and torque, N*m, in order.
fn points(snaps: &[LaunchSnapshot]) -> Vec<(f64, f64)> {
    snaps.iter().flat_map(|s| s.points.chunks(6).map(|p| (p[0] as f64, p[1] as f64))).collect()
}

/// The highest power over the run, kW, from each cycle averaged with its neighbours.
fn peak_kw(pts: &[(f64, f64)]) -> f64 {
    pts.windows(7).map(|w| w.iter().map(|(rpm, nm)| rpm * nm * 2.0 * PI / 60.0).sum::<f64>() / 7e3).fold(0.0, f64::max)
}

/// Holds at the start speed, then sweeps to the end at the rate asked for, going nowhere.
#[test]
fn a_pull_sweeps_at_its_rate() {
    let snaps = run(dyno(3000.0, 8000.0, 500.0));
    assert!(snaps.iter().any(|s| s.phase == "hold"), "holds at the start speed first");
    assert!(snaps.iter().all(|s| s.speed_kmh == 0.0 && s.distance == 0.0 && s.gear == 1.0));
    let pts = points(&snaps);
    let (first, last) = (pts.first().unwrap().0, pts.last().unwrap().0);
    assert!((first - 3000.0).abs() < 150.0, "starts recording at {first} rpm");
    assert!(last > 7700.0, "pulls to {last} rpm");
    assert!(pts.windows(2).all(|w| w[1].0 > w[0].0), "the engine only gains speed through the sweep");
    let sweep = snaps.iter().rfind(|s| !s.finished).unwrap().elapsed;
    let rate = (last - first) / sweep;
    assert!((rate / 500.0 - 1.0).abs() < 0.1, "sweeps at {rate} rpm/s over {sweep} s");
}

/// The dyno measures the same engine the launch does: its peak power within a few percent of the peak
/// through the S2000's gearbox. And a slower sweep reads about the same.
#[test]
fn a_pull_reads_the_launchs_power() {
    let launch = peak_kw(&points(&run(common::engine_preset(ENGINE).launch.clone())));
    let fast = peak_kw(&points(&run(dyno(3000.0, 8500.0, 500.0))));
    let slow = peak_kw(&points(&run(dyno(3000.0, 8500.0, 250.0))));
    println!("peak {launch:.1} kW launching, {fast:.1} kW at 500 rpm/s, {slow:.1} kW at 250 rpm/s");
    assert!((fast / launch - 1.0).abs() < 0.05, "{fast} kW on the dyno against {launch} kW launching");
    assert!((slow / fast - 1.0).abs() < 0.04, "{slow} kW at 250 rpm/s against {fast} kW at 500");
}

/// A pull without a sweep rate starts nothing.
#[test]
fn a_pull_needs_a_sweep_rate() {
    let preset = common::engine_preset(ENGINE);
    let mut sim = EngineSim::new(FS, &preset.config);
    sim.start_launch(dyno(3000.0, 8000.0, 0.0));
    sim.render(FS as usize / 10);
    assert!(sim.snapshot().launch.is_none());
}
