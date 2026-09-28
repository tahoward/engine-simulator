//! A launch from standstill: the clutch slipped off the line, a pull through every gear, and the
//! timeslip it leaves.

mod common;

use common::FS;
use engine_sim::spec::LaunchSnapshot;
use engine_sim::{EngineSim, LaunchConfig};

/// Run `name`'s fitted car down the strip through `launch`, idling first. Returns every snapshot taken
/// while the run lasted, one per 20 ms.
fn run(name: &str, launch: Option<LaunchConfig>) -> Vec<LaunchSnapshot> {
    let preset = common::engine_preset(name);
    let mut sim = EngineSim::new(FS, &preset.config);
    sim.render(FS as usize / 2);
    sim.start_launch(launch.unwrap_or_else(|| preset.launch.clone()));
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

fn last(snaps: &[LaunchSnapshot]) -> &LaunchSnapshot {
    snaps.iter().rfind(|s| !s.finished).or(snaps.last()).expect("the run took a snapshot")
}

/// Moves off, pulls through all six gears, and times 60 mph, the quarter and the half mile.
#[test]
fn a_launch_leaves_a_timeslip() {
    let snaps = run("Inline four, Honda F20C", None);
    let end = snaps.last().unwrap();
    assert!(snaps.iter().any(|s| s.phase == "launch"), "starts by slipping the clutch");
    assert_eq!(last(&snaps).gear, 6.0, "pulls through every gear");
    let sixty = end.zero_to_sixty.expect("reaches 60 mph");
    let quarter = end.quarter_mile.expect("covers the quarter mile");
    let half = end.half_mile.expect("covers the half mile");
    println!(
        "0-60 {sixty:.2} s, 1/4 {quarter:.2} s @ {:.0} km/h, 1/2 {half:.2} s @ {:.0} km/h",
        end.quarter_mile_kmh.unwrap(),
        end.half_mile_kmh.unwrap()
    );
    // An S2000's figures, give or take: about 5.5 s, 14 s at 160 km/h, and 22 s.
    assert!((4.0..8.0).contains(&sixty), "0-60 in {sixty} s");
    assert!((12.0..17.0).contains(&quarter), "quarter mile in {quarter} s");
    assert!(half > quarter + 5.0 && half < quarter + 11.0, "half mile in {half} s");
    assert!(end.half_mile_kmh.unwrap() > end.quarter_mile_kmh.unwrap());
}

/// The clock starts as the car moves off, not while the engine revs up to the launch speed.
#[test]
fn the_clock_starts_as_the_car_moves() {
    let snaps = run("Inline four, Honda F20C", None);
    let revving = snaps.iter().take_while(|s| s.speed_kmh == 0.0).count();
    assert!(revving > 0, "the engine revs up before the clutch bites");
    assert!(snaps[..revving].iter().all(|s| s.elapsed == 0.0 && s.distance == 0.0));
}

/// Shorter gearing is quicker off the line and runs out of gears sooner; a gearbox of any length is
/// pulled through to its last gear.
#[test]
fn the_gearing_is_the_users() {
    let preset = common::engine_preset("Inline four, Honda F20C");
    let short = LaunchConfig { final_drive: preset.launch.final_drive * 1.3, ..preset.launch.clone() };
    let fitted = run("Inline four, Honda F20C", None);
    let quick = run("Inline four, Honda F20C", Some(short));
    let (a, b) = (fitted.last().unwrap().zero_to_sixty.unwrap(), quick.last().unwrap().zero_to_sixty.unwrap());
    assert!(b < a, "0-60 in {b} s on the short final drive, against {a} s");

    let three = LaunchConfig { ratios: vec![3.0, 1.8, 1.2], ..preset.launch.clone() };
    let snaps = run("Inline four, Honda F20C", Some(three));
    assert_eq!(last(&snaps).gear, 3.0);
    assert!(snaps.iter().all(|s| s.gear <= 3.0));
}

/// A gearbox without gears starts nothing.
#[test]
fn an_empty_gearbox_starts_nothing() {
    let preset = common::engine_preset("Inline four, Honda F20C");
    let mut sim = EngineSim::new(FS, &preset.config);
    sim.start_launch(LaunchConfig { ratios: vec![], ..preset.launch.clone() });
    sim.render(FS as usize / 10);
    assert!(sim.snapshot().launch.is_none());
}

/// Too much torque for the tyres spins them, and the engine flares off the line rather than bogging.
#[test]
fn a_heavy_foot_spins_the_tyres() {
    let preset = common::engine_preset("V8, Chevrolet LT2");
    let light = LaunchConfig { mass: 900.0, ..preset.launch.clone() };
    let snaps = run("V8, Chevrolet LT2", Some(light));
    let end = snaps.last().unwrap();
    let sixty = end.zero_to_sixty.expect("reaches 60 mph");
    // Traction-limited: 0.66 g at best, so no quicker than about 4.1 s whatever the engine.
    assert!(sixty > 3.5, "0-60 in {sixty} s");
}
