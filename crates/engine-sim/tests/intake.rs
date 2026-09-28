//! The intake runners: a column of air per cylinder, whose momentum rams the charge in and whose length
//! tunes where it does so.

mod common;

use common::FS;
use engine_sim::EngineSim;
use engine_sim::spec::{
    EngineSpec, ExhaustLayout, PipeSegment, SegmentKind, SegmentPartial, collector_groups, displacement,
    exhaust_layout_of, exhaust_port_diameter, gas, intake_runner_of, make_segment,
};
use serde_json::{Value, json};

/// A 6.2 litre pushrod V8 in the proportions of a Chevrolet LT2, with a late-closing cam.
fn v8() -> Value {
    json!({
        "cylinders": 8,
        "vAngle": 90,
        "crankType": "crossplane",
        "exhaustLayout": "perBank",
        "bore": 0.10325,
        "stroke": 0.092,
        "rodLength": 0.1556,
        "compressionRatio": 11.5,
        "exValveDia": 0.0404,
        "inValveDia": 0.054,
        "maxLift": 0.0145,
        "evo": 104,
        "evc": 384,
        "ivo": 338,
        "ivc": 614,
        "revLimit": 6600,
    })
}

fn v8_spec(over: Value) -> EngineSpec {
    common::with(&common::with(&common::presets().default_engine, v8()), over)
}

fn seg(kind: SegmentKind, length: f64, d_in: f64, d_out: f64, yaw: f64) -> PipeSegment {
    make_segment(SegmentPartial {
        kind: Some(kind),
        length: Some(length),
        d_in: Some(d_in),
        d_out: Some(d_out),
        yaw: Some(yaw),
        ..Default::default()
    })
}

/// The web app's `fittedExhaust`: a header runner bored for the valve and, where the layout merges, a
/// constant-velocity collector with a silencer can. `(pipe, collector)`.
fn fitted_exhaust(spec: &EngineSpec) -> (Vec<PipeSegment>, Vec<PipeSegment>) {
    let layout = exhaust_layout_of(spec);
    let groups = collector_groups(spec);
    let collector_count = groups.iter().fold(0, |max, &g| i32::max(max, g + 1));
    let per_collector = if collector_count > 0 { spec.cylinders as f64 / collector_count as f64 } else { 1.0 };

    let d_primary = f64::max(0.85 * exhaust_port_diameter(spec), 0.02);
    let primary_length = if layout == ExhaustLayout::Open { 0.75 } else { 0.45 };
    let mut pipe = vec![seg(SegmentKind::Pipe, primary_length, d_primary, d_primary, 0.0)];
    if layout == ExhaustLayout::Open {
        pipe.push(seg(SegmentKind::Cone, 0.25, d_primary, d_primary * 1.7, 0.0));
        return (pipe, Vec::new());
    }

    let d_collector = d_primary * per_collector.sqrt() * 0.92;
    let served_disp = displacement(spec) * per_collector;
    let can_dia = f64::min(d_collector * 2.5, 0.2);
    let can_area = (std::f64::consts::PI * can_dia * can_dia) / 4.0;
    let can_length = f64::min(f64::max((8.0 * served_disp) / can_area, 0.25), 0.6);
    let run_length = f64::max(2.2 - primary_length - can_length - 0.5, 0.35);
    let collector = vec![
        seg(SegmentKind::Cone, 0.16, d_primary * 1.25, d_collector, 0.0),
        seg(SegmentKind::Pipe, run_length, d_collector, d_collector, 0.0),
        seg(SegmentKind::Chamber, can_length, d_collector, can_dia, 0.0),
        seg(SegmentKind::Pipe, 0.5, d_collector, d_collector, 0.0),
    ];
    (pipe, collector)
}

/// Mean gas torque, N m, over the next `samples` samples.
fn mean_torque(sim: &mut EngineSim, samples: usize) -> f64 {
    let mut t = 0.0;
    for _ in 0..samples {
        sim.render(1);
        t += sim.snapshot().torque;
    }
    t / samples as f64
}

/// Whether the crank crossed `ivc` going from `prev` to `a`, allowing for the wrap at 720.
fn crossed(ivc: f64, prev: f64, a: f64) -> bool {
    if a >= prev { ivc > prev && ivc <= a } else { ivc > prev || ivc <= a }
}

/// Fresh charge trapped in cylinder 0 at each intake closing over `samples` samples, as a fraction
/// of the displacement at ambient density; and the mean torque over the same samples.
fn trapped_charge(sim: &mut EngineSim, spec: &EngineSpec, samples: usize) -> (f64, f64) {
    let mut prev = sim.cylinders()[0].angle;
    let mut trapped = 0.0;
    let mut cycles = 0;
    let mut torque = 0.0;
    for _ in 0..samples {
        sim.render(1);
        torque += sim.snapshot().torque;
        let cyl = &sim.cylinders()[0];
        let a = cyl.angle;
        if crossed(spec.ivc, prev, a) {
            trapped += cyl.mass * (1.0 - cyl.burned_fraction());
            cycles += 1;
        }
        prev = a;
    }
    let full = (gas::P_AMB * displacement(spec)) / (gas::R * gas::T_AMB);
    (trapped / cycles as f64 / full, torque / samples as f64)
}

struct Breath {
    ve: f64,
    torque: f64,
}

/// Volumetric efficiency and mean gas torque at full throttle, held at `rpm`.
fn breathe(rpm: f64, over: Value) -> Breath {
    let spec = v8_spec(over);
    let mut cfg = common::default_config();
    cfg.engine =
        common::with(&spec, json!({ "freeRunning": false, "throttle": 1, "rpm": rpm, "combustionVariability": 0 }));
    let (mut pipe, collector) = fitted_exhaust(&spec);
    // Long-tube headers for this valve, 44 mm primaries, rather than the fitted exhaust's narrower ones.
    for seg in pipe.iter_mut() {
        seg.d_in = 0.044;
        seg.d_out = 0.044;
    }
    cfg.pipe = pipe;
    cfg.collector = collector;
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(FS as usize);
    let (ve, torque) = trapped_charge(&mut sim, &spec, FS as usize / 2);
    Breath { ve, torque }
}

/// The local `fitted_exhaust` is the web app's, as the presets fixture records it.
#[test]
fn fitted_exhaust_matches_the_web_app() {
    let shape = |s: &[PipeSegment]| -> Vec<(SegmentKind, f64, f64, f64, f64)> {
        s.iter().map(|g| (g.kind, g.length, g.d_in, g.d_out, g.yaw)).collect()
    };
    for preset in &common::presets().engine_presets {
        let (pipe, collector) = fitted_exhaust(&preset.config.engine);
        assert_eq!(shape(&pipe), shape(&preset.fitted_exhaust.pipe), "{}", preset.name);
        assert_eq!(shape(&collector), shape(&preset.fitted_exhaust.collector), "{}", preset.name);
    }
}

mod intake_runners {
    use super::*;

    /// are sized from the valves and tuned from the rev range
    #[test]
    fn are_sized_from_the_valves_and_tuned_from_the_rev_range() {
        let r = intake_runner_of(&v8_spec(json!({})));
        assert!((r.diameter - 0.9 * 0.054).abs() < 5e-10, "diameter {}", r.diameter);
        // Quarter-wave at 2.3 times the crank speed at three quarters of 6600 rpm.
        assert!(r.length > 0.43, "length {}", r.length);
        assert!(r.length < 0.47, "length {}", r.length);
        let set = intake_runner_of(&v8_spec(json!({ "intakeRunnerLength": 0.3, "intakeRunnerDia": 0.05 })));
        assert_eq!((set.length, set.diameter), (0.3, 0.05));
    }

    /// ram the charge in, most at the speed they are tuned for
    ///
    /// The ramming: at the speed it is tuned for, three quarters of the rev limit, the runner fills the
    /// cylinder to nearly 100%, about 15 points more than a stub a few centimetres long, which rams
    /// almost nothing; and it fills best there, falling away either side.
    #[test]
    fn ram_the_charge_in_most_at_the_speed_they_are_tuned_for() {
        let tuned = breathe(4950.0, json!({})).ve;
        assert!(tuned > 0.95, "tuned {tuned}");
        let stub = breathe(4950.0, json!({ "intakeRunnerLength": 0.08 })).ve;
        assert!(tuned > stub + 0.1, "tuned {tuned} stub {stub}");
        let low = breathe(4000.0, json!({})).ve;
        assert!(tuned > low, "tuned {tuned} at 4000 {low}");
        let high = breathe(5800.0, json!({})).ve;
        assert!(tuned > high, "tuned {tuned} at 5800 {high}");
    }

    /// move the torque with their length
    ///
    /// A long runner is tuned low and a short one high, so each wins at its own end of the range.
    #[test]
    fn move_the_torque_with_their_length() {
        let long = || json!({ "intakeRunnerLength": 0.8 });
        let short = || json!({ "intakeRunnerLength": 0.25 });
        let (l, s) = (breathe(3500.0, long()).torque, breathe(3500.0, short()).torque);
        assert!(l > s, "at 3500: long {l} short {s}");
        let (l, s) = (breathe(6450.0, long()).torque, breathe(6450.0, short()).torque);
        assert!(s > l, "at 6450: long {l} short {s}");
    }
}

mod headers {
    use super::*;

    /// fill the cylinder more than a manifold at the speed they are tuned for
    ///
    /// Equal-length headers scavenge: the wave each pulse sends back from the merge pulls fresh charge
    /// through the cylinder during the overlap. On the LT6, with 70 degrees of overlap and primaries tuned
    /// for 8400 rpm, that fills the cylinder a couple of points more than a manifold along the ports does.
    #[test]
    fn fill_the_cylinder_more_than_a_manifold_at_the_speed_they_are_tuned_for() {
        let lt6 = common::engine_preset("V8, Chevrolet LT6");
        let fill = |exhaust_headers: bool| {
            let spec = common::with(&lt6.config.engine, json!({ "exhaustHeaders": exhaust_headers }));
            let mut cfg = common::default_config();
            cfg.engine = common::with(
                &spec,
                json!({ "freeRunning": false, "throttle": 1, "rpm": 8400, "combustionVariability": 0 }),
            );
            cfg.pipe = lt6.config.pipe.clone();
            cfg.collector = lt6.config.collector.clone();
            let mut sim = EngineSim::new(FS, &cfg);
            sim.render(FS as usize / 2);
            trapped_charge(&mut sim, &spec, FS as usize / 2).0
        };
        let (headers, manifold) = (fill(true), fill(false));
        assert!(headers > manifold + 0.015, "headers {headers} manifold {manifold}");
    }
}

/// The LT6 as the app loads it, with the fields in `over` on top.
fn lt6(over: Value) -> EngineSim {
    let lt6 = common::engine_preset("V8, Chevrolet LT6");
    let mut cfg = common::default_config();
    cfg.engine = common::with(&lt6.config.engine, over);
    cfg.pipe = lt6.config.pipe.clone();
    cfg.collector = lt6.config.collector.clone();
    EngineSim::new(FS, &cfg)
}

/// `a` with the fields of `b` over it.
fn merged(mut a: Value, b: Value) -> Value {
    for (k, v) in b.as_object().unwrap() {
        a[k] = v.clone();
    }
    a
}

mod variable_valve_timing {
    use super::*;

    fn torque_at(rpm: f64, over: Value) -> f64 {
        let mut sim =
            lt6(merged(json!({ "freeRunning": false, "throttle": 1, "rpm": rpm, "combustionVariability": 0 }), over));
        // The walls and the waves take a couple of seconds to settle at a new speed.
        sim.render(2 * FS as usize);
        mean_torque(&mut sim, FS as usize / 2)
    }

    /// lifts the mid-range of an engine cammed for the top end
    ///
    /// A cam tuned for 8400 rpm gives up the mid-range; advancing it there gives it back.
    #[test]
    fn lifts_the_mid_range_of_an_engine_cammed_for_the_top_end() {
        let fixed = || json!({ "vvtIntakeLow": 0, "vvtExhaustLow": 0 });
        let (phased, still) = (torque_at(4500.0, json!({})), torque_at(4500.0, fixed()));
        assert!(phased > 1.1 * still, "at 4500: phased {phased} fixed {still}");
        // At the top the map has the cams at rest, so it changes nothing there.
        let ratio = torque_at(8400.0, json!({})) / torque_at(8400.0, fixed());
        assert!((ratio - 1.0).abs() < 0.005, "at 8400: ratio {ratio}");
    }

    /// keeps the cams at rest at idle, and moves them under load
    #[test]
    fn keeps_the_cams_at_rest_at_idle_and_moves_them_under_load() {
        let mut idle = lt6(json!({ "freeRunning": true }));
        idle.render(FS as usize * 2);
        let phase = idle.snapshot().intake_cam_advance;
        assert!(phase.abs() < 5e-10, "idle phase {phase}");
        let mut pulling = lt6(json!({ "freeRunning": false, "throttle": 1, "rpm": 3000 }));
        pulling.render(FS as usize / 2);
        let phase = pulling.snapshot().intake_cam_advance;
        assert!(phase > 20.0, "pulling phase {phase}");
    }

    /// brings a cam back to rest when its map is set to nothing
    #[test]
    fn brings_a_cam_back_to_rest_when_its_map_is_set_to_nothing() {
        let mut sim = lt6(json!({ "freeRunning": false, "throttle": 1, "rpm": 3000 }));
        sim.render(FS as usize / 2);
        sim.set_engine_json(&json!({ "vvtIntakeLow": 0, "vvtExhaustLow": 0 })).unwrap();
        sim.render(FS as usize / 2);
        let phase = sim.snapshot().intake_cam_advance;
        assert!(phase.abs() < 5e-10, "phase {phase}");
    }
}

mod two_stage_intake {
    use super::*;

    fn build(over: Value) -> EngineSim {
        lt6(merged(json!({ "freeRunning": false, "throttle": 1, "combustionVariability": 0 }), over))
    }

    fn torque_at(rpm: f64, over: Value) -> f64 {
        let mut sim = build(merged(json!({ "rpm": rpm }), over));
        // The walls and the waves take a couple of seconds to settle at a new speed.
        sim.render(2 * FS as usize);
        mean_torque(&mut sim, FS as usize / 2)
    }

    fn switch_rpm() -> f64 {
        common::engine_preset("V8, Chevrolet LT6").config.engine.intake_switch_rpm
    }

    /// has the long runners’ torque below its switch speed and the short runners’ above it
    ///
    /// Long runners fill it below the switch speed; above it, it has the short ones'.
    #[test]
    fn has_the_long_runners_torque_below_its_switch_speed_and_the_short_runners_above_it() {
        let short_length = common::engine_preset("V8, Chevrolet LT6").config.engine.intake_runner_short_length;
        let short_only = || json!({ "intakeRunnerLength": short_length, "intakeRunnerShortLength": 0 });
        let (both, short) = (torque_at(7800.0, json!({})), torque_at(7800.0, short_only()));
        assert!(both > 1.02 * short, "at 7800: two-stage {both} short only {short}");
        let ratio = torque_at(8400.0, json!({})) / torque_at(8400.0, short_only());
        assert!((ratio - 1.0).abs() < 0.005, "at 8400: ratio {ratio}");
    }

    /// switches at its switch speed, back a little below it, and carries on smoothly
    #[test]
    fn switches_at_its_switch_speed_back_a_little_below_it_and_carries_on_smoothly() {
        let switch_rpm = switch_rpm();
        let mut sim = build(json!({ "rpm": switch_rpm - 100.0 }));
        sim.render(FS as usize);
        assert!(!sim.snapshot().short_runners);
        let before = mean_torque(&mut sim, FS as usize / 4);

        sim.set_engine_json(&json!({ "rpm": switch_rpm + 100.0 })).unwrap();
        sim.render(1);
        assert!(sim.snapshot().short_runners);
        let across = mean_torque(&mut sim, FS as usize / 4);
        // Inside the hysteresis band it stays on the short runners.
        sim.set_engine_json(&json!({ "rpm": switch_rpm - 100.0 })).unwrap();
        sim.render(FS as usize / 10);
        assert!(sim.snapshot().short_runners);
        // Only the set in use is stepped, so the short runners' count is final once they hand over.
        let short_recoveries = sim.intake().recoveries();
        sim.set_engine_json(&json!({ "rpm": switch_rpm - 200.0 })).unwrap();
        sim.render(1);
        assert!(!sim.snapshot().short_runners);
        sim.render(FS as usize / 4);

        assert_eq!(sim.intake().recoveries() + short_recoveries, 0);
        // Near the switch speed the two sets make about the same torque, so the switch is no jolt.
        assert!((across / before - 1.0).abs() < 0.1, "before {before} across {across}");
    }
}

mod torque_curve {
    use super::*;

    /// rises and falls smoothly at full throttle, without a dip or a hump from the runners' resonance
    ///
    /// A runner's waves die away over a few cycles, as a real runner's do, so its resonance does not
    /// build from one cycle to the next and swing the torque by 5-8% every 1500 rpm or so. From 4400 rpm
    /// the LT6's torque falls by under 3% anywhere on its way up to its peak, and rises by under 4%
    /// anywhere on its way down from it.
    #[test]
    fn rises_and_falls_smoothly_at_full_throttle() {
        let rpms: Vec<f64> = (0..=10).map(|i| 4400.0 + 400.0 * i as f64).collect();
        let torque: Vec<f64> = std::thread::scope(|s| {
            let runs: Vec<_> = rpms
                .iter()
                .map(|&rpm| {
                    s.spawn(move || {
                        let mut sim =
                            lt6(json!({ "freeRunning": false, "throttle": 1, "rpm": rpm, "combustionVariability": 0 }));
                        // The walls and the waves take a couple of seconds to settle at a new speed.
                        sim.render(2 * FS as usize);
                        mean_torque(&mut sim, FS as usize / 2)
                    })
                })
                .collect();
            runs.into_iter().map(|r| r.join().unwrap()).collect()
        });
        let peak = (0..torque.len()).max_by(|&a, &b| torque[a].total_cmp(&torque[b])).unwrap();
        for i in 1..torque.len() {
            let change = torque[i] / torque[i - 1] - 1.0;
            let (a, b) = (rpms[i - 1], rpms[i]);
            if i <= peak {
                assert!(change > -0.03, "falls {:.1}% from {a} to {b} rpm: {torque:?}", -100.0 * change);
            } else {
                assert!(change < 0.04, "rises {:.1}% from {a} to {b} rpm: {torque:?}", 100.0 * change);
            }
        }
    }
}
