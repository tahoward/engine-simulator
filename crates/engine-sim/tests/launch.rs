//! A launch from standstill: the clutch slipped off the line, a pull through every gear, and the
//! timeslip it leaves.

use crate::common;

use common::FS;
use engine_sim::spec::LaunchSnapshot;
use engine_sim::{EngineSim, LaunchConfig};

/// Run `name`'s fitted car down the strip through `launch`, idling first, until `enough`.
fn run(name: &str, launch: LaunchConfig, enough: impl Fn(&LaunchSnapshot) -> bool) -> Vec<LaunchSnapshot> {
    common::launch(&common::engine_preset(name).config, launch, enough)
}

/// `name`'s car, as `launch` changes it, to 60 mph: the time it took, s.
fn to_sixty(name: &str, launch: LaunchConfig) -> f64 {
    run(name, launch, common::to_sixty).last().unwrap().zero_to_sixty.expect("reaches 60 mph")
}

/// `name`'s own car to 60 mph: the time it took, s.
fn own_to_sixty(name: &str) -> f64 {
    to_sixty(name, common::engine_preset(name).launch.clone())
}

fn last(snaps: &[LaunchSnapshot]) -> &LaunchSnapshot {
    snaps.iter().rfind(|s| !s.finished).or(snaps.last()).expect("the run took a snapshot")
}

/// Moves off, pulls through all six of the S2000's gears, and times 60 mph, the quarter and the half mile.
#[test]
fn a_launch_leaves_a_timeslip() {
    let snaps = common::preset_launch("Inline four, Honda F20C");
    let end = snaps.last().unwrap();
    assert!(snaps.iter().any(|s| s.phase == "launch"), "starts by slipping the clutch");
    assert_eq!(last(snaps).gear, 6.0, "pulls through every gear");
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
    let snaps = common::preset_launch("Inline four, Honda F20C");
    let revving = snaps.iter().take_while(|s| s.speed_kmh == 0.0).count();
    assert!(revving > 0, "the engine revs up before the clutch bites");
    assert!(snaps[..revving].iter().all(|s| s.elapsed == 0.0 && s.distance == 0.0));
}

/// The gearing sets the road speed of every shift: a final drive 30% shorter goes into second 30% sooner.
/// A gearbox of any length is pulled through to its last gear.
#[test]
fn the_gearing_is_the_users() {
    let name = "Inline four, Honda F20C";
    let preset = common::engine_preset(name);
    let into_second = |snaps: &[LaunchSnapshot]| snaps.iter().find(|s| s.gear == 2.0).unwrap().speed_kmh;
    let short = LaunchConfig { final_drive: preset.launch.final_drive * 1.3, ..preset.launch.clone() };
    let three = LaunchConfig { ratios: vec![3.0, 1.8, 1.2], ..preset.launch.clone() };
    // The short final drive only as far as second; the three-speed box to the end.
    let [short, three] = common::par([(short, true), (three, false)], |(launch, to_second)| {
        run(name, launch.clone(), |s| *to_second && s.gear == 2.0)
    });
    let (a, b) = (into_second(common::preset_launch(name)), into_second(&short));
    assert!((b * 1.3 / a - 1.0).abs() < 0.05, "into second at {b} km/h on the short final drive, against {a} km/h");

    assert_eq!(last(&three).gear, 3.0);
    assert!(three.iter().all(|s| s.gear <= 3.0));
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
    let [held, spun] = common::par([true, false], |&tc| {
        to_sixty("V8, Chevrolet LT2", LaunchConfig { traction_control: tc, ..light.clone() })
    });
    assert!(held < spun, "0-60 in {held} s with traction control, against {spun} s spinning the tyres");
    // A grip of 1.31 on 60% of the weight, and more as it moves back: about 1 g, 2.7 s at best.
    assert!(held > 2.4, "0-60 in {held} s");
}

/// The engines from real cars launch through those cars: the Skyline's six-speed to all four wheels, the
/// Corvettes' eight-speed dual clutch, the MR2's five-speed, the RS 3's seven-speed dual clutch to all four
/// wheels, the Fiesta ST's six-speed to the front wheels, the Mazdaspeed MX-5's six-speed, the CVO Road Glide's six-speed, sixth direct,
/// through its primary chain and belt, the RC51's six-speed through its primary gears and chain, and the
/// Hypermotard 698's the same way.
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
    let msm = &common::engine_preset("Inline four, Mazda BPT").launch;
    assert_eq!(msm.ratios, vec![3.76, 2.27, 1.65, 1.26, 1.0, 0.84]);
    assert_eq!(msm.final_drive, 4.1);
    let cvo = &common::engine_preset("45° V-twin, Harley-Davidson Milwaukee-Eight 121").launch;
    assert_eq!((cvo.ratios.len(), cvo.ratios[5], cvo.final_drive), (6, 1.0, 2.875));
    assert!((cvo.ratios[0] * cvo.final_drive - 9.593).abs() < 1e-9);
    let rc51 = &common::engine_preset("90° V-twin, Honda RC51").launch;
    assert_eq!(rc51.ratios, vec![2.461, 1.812, 1.428, 1.24, 1.08, 0.962]);
    assert!((rc51.final_drive - 4.25).abs() < 1e-9);
    let speed_twin = &common::engine_preset("Parallel twin, Triumph 1200 HT").launch;
    assert_eq!(speed_twin.ratios, vec![2.583, 1.842, 1.38, 1.13, 0.966, 0.81]);
    assert!((speed_twin.final_drive - (72.0 / 41.0) * (43.0 / 18.0)).abs() < 1e-9);
    let gt86 = &common::engine_preset("Boxer four, Subaru FA20D").launch;
    assert_eq!(gt86.ratios, vec![3.626, 2.188, 1.541, 1.213, 1.0, 0.767]);
    assert_eq!(gt86.final_drive, 4.1);
    let rs40 = &common::engine_preset("Boxer six, Porsche Mezger 4.0").launch;
    assert_eq!(rs40.ratios, vec![3.82, 2.15, 1.56, 1.21, 0.97, 0.83]);
    assert_eq!(rs40.final_drive, 3.89);
    let mono = &common::engine_preset("Single, Ducati Superquadro Mono").launch;
    assert_eq!(mono.ratios.len(), 6);
    assert!((mono.ratios[0] * mono.final_drive - (36.0 / 13.0) * (61.0 / 31.0) * (43.0 / 15.0)).abs() < 1e-9);
}

/// The Toyota 86 gets to 60 mph in about the 6.2-6.8 s road tests time the manual at: 200 hp through the
/// rear wheels.
#[test]
fn the_toyota_86_launches_about_as_quick_as_the_real_car() {
    let sixty = own_to_sixty("Boxer four, Subaru FA20D");
    assert!((5.8..7.2).contains(&sixty), "0-60 in {sixty} s");
}

/// The GT3 RS 4.0 gets to 60 mph in about the 3.5-4.0 s road tests time it at: 500 PS through the rear
/// wheels, its engine over them.
#[test]
fn the_gt3_rs_4_0_launches_about_as_quick_as_the_real_car() {
    let sixty = own_to_sixty("Boxer six, Porsche Mezger 4.0");
    assert!((3.2..4.4).contains(&sixty), "0-60 in {sixty} s");
}

/// The Fiesta ST gets to 60 mph in about the 6.5 s Ford gives it to 62: 200 PS through the front wheels.
#[test]
fn the_fiesta_st_launches_about_as_quick_as_the_real_car() {
    let sixty = own_to_sixty("Inline three, Ford 1.5 EcoBoost Dragon");
    assert!((5.7..6.9).contains(&sixty), "0-60 in {sixty} s");
}

/// The Mazdaspeed MX-5 gets to 60 mph in about the 6.5-6.9 s road tests time it at: 178 hp through the
/// rear wheels. Its turbo is on boost by 3000 rpm, a little sooner than the real one's, so it runs a
/// few tenths quicker.
#[test]
fn the_mazdaspeed_mx5_launches_about_as_quick_as_the_real_car() {
    let sixty = own_to_sixty("Inline four, Mazda BPT");
    assert!((5.6..7.2).contains(&sixty), "0-60 in {sixty} s");
}

/// Driving the front wheels, the weight the car moves back as it pulls away comes off them, so the same
/// car launches slower than it does through the rear.
#[test]
fn front_wheel_drive_launches_softer() {
    let name = "Inline three, Ford 1.5 EcoBoost Dragon";
    let own = &common::engine_preset(name).launch;
    let [f, r] =
        common::par([true, false], |&front| to_sixty(name, LaunchConfig { front_wheel_drive: front, ..own.clone() }));
    assert!(f > r + 0.1, "0-60 in {f} s through the front wheels, against {r} s through the rear");
}

/// The RS 3 gets to 60 mph in about the 3.6 s road tests time it at: 400 PS through all four wheels.
#[test]
fn the_rs3_launches_about_as_quick_as_the_real_car() {
    let sixty = own_to_sixty("Inline five, Audi EA855 EVO");
    assert!((3.1..4.1).contains(&sixty), "0-60 in {sixty} s");
}

/// All four wheels driven get a car off the line quicker than two.
#[test]
fn all_wheel_drive_launches_harder() {
    let name = "Inline six, Nissan RB26DETT";
    let own = &common::engine_preset(name).launch;
    let [a, r] =
        common::par([own.driven_load, 0.6], |&load| to_sixty(name, LaunchConfig { driven_load: load, ..own.clone() }));
    assert!(a < r - 0.1, "0-60 in {a} s through all four wheels, against {r} s through the rear");
}
