//! A dyno pull: the engine held at the start speed on an absorber at 1:1, then swept up at a steady rate
//! to the end, and the power curve it leaves.

use crate::common;

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
    common::launch(&common::engine_preset(ENGINE).config, config, common::to_the_end)
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
    // The launch, and the pulls at 500 and 250 rpm/s, all at once.
    let [launch, fast, slow] = common::par([None, Some(500.0), Some(250.0)], |rate| match rate {
        None => peak_kw(&points(common::preset_launch(ENGINE))),
        Some(rate) => peak_kw(&points(&run(dyno(3000.0, 8500.0, *rate)))),
    });
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

/// Every cylinder traps a charge every cycle through the LT6's cam phaser sweeping back to rest, from
/// 4550 to 7750 rpm, though the phaser moves the intake closing back and forth as it goes.
#[test]
fn every_cycle_traps_a_charge_as_the_phaser_moves() {
    let preset = common::engine_preset("V8, Chevrolet LT6");
    let config = LaunchConfig {
        dyno: true,
        ratios: vec![1.0],
        final_drive: 1.0,
        launch_rpm: 4500.0,
        shift_rpm: 7800.0,
        sweep_rate: 500.0,
        ..preset.launch.clone()
    };
    let mut sim = EngineSim::new(FS, &preset.config);
    sim.render(FS as usize / 2);
    sim.start_launch(config);
    let n = sim.cylinders().len();
    let mut prev: Vec<f64> = sim.cylinders().iter().map(|c| c.angle).collect();
    let (mut cycles, mut untrapped) = (0, 0);
    // The charge is trapped by 660 degrees at the latest, before any spark: until it is, `burned` is
    // still the last cycle's.
    while sim.snapshot().launch.is_some_and(|l| !l.finished) {
        sim.render(1);
        for (b, cyl) in sim.cylinders().iter().enumerate().take(n) {
            if prev[b] < 660.0 && cyl.angle >= 660.0 && sim.rpm() > 4000.0 {
                cycles += 1;
                if cyl.burned != 0.0 {
                    untrapped += 1;
                }
            }
            prev[b] = cyl.angle;
        }
    }
    assert!(cycles > 1000, "{cycles} cycles");
    assert_eq!(untrapped, 0, "{untrapped} of {cycles} cycles trapped no charge");
}

/// A single on a light flywheel swings its crank speed by more than the hold's band within every cycle,
/// so the hold reads each cycle's mean: it still holds at the start speed and sweeps to the end.
#[test]
fn a_single_holds_and_pulls() {
    let preset = common::engine_preset("Single, Ducati Superquadro Mono");
    let config = LaunchConfig {
        dyno: true,
        ratios: vec![1.0],
        final_drive: 1.0,
        launch_rpm: 2600.0,
        shift_rpm: 9500.0,
        sweep_rate: 500.0,
        ..preset.launch.clone()
    };
    let snaps = common::launch(&preset.config, config, common::to_the_end);
    assert!(snaps.iter().any(|s| s.phase == "pull"), "starts the sweep");
    let last = points(&snaps).last().map_or(0.0, |p| p.0);
    assert!(last > 9200.0, "pulls to {last} rpm");
}
