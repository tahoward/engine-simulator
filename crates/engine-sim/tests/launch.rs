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

/// Moves off, pulls through all six of the S2000's gears, and times 60 mph, the quarter and the half mile.
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
    // The S2000's own figures, give or take: about 5.5 s, 14 s at 160 km/h, and 22 s.
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

/// The gearing sets the road speed of every shift: a final drive 30% shorter goes into second 30% sooner.
/// A gearbox of any length is pulled through to its last gear.
#[test]
fn the_gearing_is_the_users() {
    let preset = common::engine_preset("Inline four, Honda F20C");
    let short = LaunchConfig { final_drive: preset.launch.final_drive * 1.3, ..preset.launch.clone() };
    let into_second = |snaps: &[LaunchSnapshot]| snaps.iter().find(|s| s.gear == 2.0).unwrap().speed_kmh;
    let (a, b) =
        (into_second(&run("Inline four, Honda F20C", None)), into_second(&run("Inline four, Honda F20C", Some(short))));
    assert!((b * 1.3 / a - 1.0).abs() < 0.05, "into second at {b} km/h on the short final drive, against {a} km/h");

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

/// Far more torque than the tyres can take: spinning them loses grip, so holding them at their peak with
/// traction control is quicker to 60, and no quicker than their grip allows whatever the engine.
#[test]
fn traction_control_beats_spinning_the_tyres() {
    let preset = common::engine_preset("V8, Chevrolet LT2");
    let light = LaunchConfig { mass: 900.0, ..preset.launch.clone() };
    let sixty = |tc: bool| {
        let snaps = run("V8, Chevrolet LT2", Some(LaunchConfig { traction_control: tc, ..light.clone() }));
        snaps.last().unwrap().zero_to_sixty.expect("reaches 60 mph")
    };
    let (held, spun) = (sixty(true), sixty(false));
    assert!(held < spun, "0-60 in {held} s with traction control, against {spun} s spinning the tyres");
    // A grip of 1.31 on 60% of the weight, and more as it moves back: about 1 g, 2.7 s at best.
    assert!(held > 2.4, "0-60 in {held} s");
}

/// The engines from real cars launch through those cars: the Skyline's six-speed to all four wheels, the
/// Corvettes' eight-speed dual clutch, the MR2's five-speed, the RS 3's seven-speed dual clutch to all four
/// wheels, the Fiesta ST's six-speed to the front wheels, the CVO Road Glide's six-speed, sixth direct,
/// through its primary chain and belt, and the RC51's six-speed through its primary gears and chain.
#[test]
fn real_engines_launch_through_their_own_cars() {
    let r34 = &common::engine_preset("Inline six, Nissan RB26DETT").launch;
    assert_eq!(r34.ratios, vec![3.827, 2.36, 1.685, 1.312, 1.0, 0.793]);
    assert_eq!((r34.final_drive, r34.driven_load), (3.545, 1.0));
    let z06 = &common::engine_preset("V8, Chevrolet LT6").launch;
    assert_eq!(z06.ratios.len(), 8);
    assert!(z06.shift_time < 0.2);
    let mr2 = &common::engine_preset("Inline four, Toyota 3S-GTE").launch;
    assert_eq!(mr2.ratios, vec![3.23, 1.913, 1.258, 0.918, 0.731]);
    assert_eq!(mr2.final_drive, 4.285);
    let rs3 = &common::engine_preset("Inline five, Audi EA855 EVO").launch;
    assert_eq!(rs3.ratios.len(), 7);
    assert_eq!((rs3.final_drive, rs3.driven_load), (4.059, 1.0));
    assert!(rs3.dual_clutch);
    let st = &common::engine_preset("Inline three, Ford 1.5 EcoBoost Dragon").launch;
    assert_eq!(st.ratios, vec![3.59, 2.19, 1.52, 1.15, 0.92, 0.79]);
    assert_eq!(st.final_drive, 3.91);
    assert!(st.front_wheel_drive);
    let cvo = &common::engine_preset("45° V-twin, Harley-Davidson Milwaukee-Eight 121").launch;
    assert_eq!((cvo.ratios.len(), cvo.ratios[5], cvo.final_drive), (6, 1.0, 2.875));
    assert!((cvo.ratios[0] * cvo.final_drive - 9.593).abs() < 1e-9);
    let rc51 = &common::engine_preset("90° V-twin, Honda RC51").launch;
    assert_eq!(rc51.ratios, vec![2.461, 1.812, 1.428, 1.24, 1.08, 0.962]);
    assert!((rc51.final_drive - 4.25).abs() < 1e-9);
}

/// The Fiesta ST gets to 60 mph in about the 6.5 s Ford gives it to 62: 200 PS through the front wheels.
#[test]
fn the_fiesta_st_launches_about_as_quick_as_the_real_car() {
    let snaps = run("Inline three, Ford 1.5 EcoBoost Dragon", None);
    let sixty = snaps.last().unwrap().zero_to_sixty.expect("reaches 60 mph");
    assert!((5.7..6.9).contains(&sixty), "0-60 in {sixty} s");
}

/// Driving the front wheels, the weight the car moves back as it pulls away comes off them, so the same
/// car launches slower than it does through the rear.
#[test]
fn front_wheel_drive_launches_softer() {
    let name = "Inline three, Ford 1.5 EcoBoost Dragon";
    let fwd = run(name, None);
    let rwd = run(name, Some(LaunchConfig { front_wheel_drive: false, ..common::engine_preset(name).launch.clone() }));
    let (f, r) = (fwd.last().unwrap().zero_to_sixty.unwrap(), rwd.last().unwrap().zero_to_sixty.unwrap());
    assert!(f > r + 0.1, "0-60 in {f} s through the front wheels, against {r} s through the rear");
}

/// The RS 3 gets to 60 mph in about the 3.6 s road tests time it at: 400 PS through all four wheels.
#[test]
fn the_rs3_launches_about_as_quick_as_the_real_car() {
    let snaps = run("Inline five, Audi EA855 EVO", None);
    let end = snaps.last().unwrap();
    let sixty = end.zero_to_sixty.expect("reaches 60 mph");
    assert!((3.1..4.1).contains(&sixty), "0-60 in {sixty} s");
}

/// All four wheels driven get a car off the line quicker than two.
#[test]
fn all_wheel_drive_launches_harder() {
    let name = "Inline six, Nissan RB26DETT";
    let awd = run(name, None);
    let rwd = run(name, Some(LaunchConfig { driven_load: 0.6, ..common::engine_preset(name).launch.clone() }));
    let (a, r) = (awd.last().unwrap().zero_to_sixty.unwrap(), rwd.last().unwrap().zero_to_sixty.unwrap());
    assert!(a < r - 0.1, "0-60 in {a} s through all four wheels, against {r} s through the rear");
}

