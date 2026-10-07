//! The intake runners: a column of air per cylinder, whose momentum rams the charge in and whose length
//! tunes where it does so.

use crate::common;

use common::FS;
use engine_sim::EngineSim;
use engine_sim::spec::{EngineSpec, PipeSegment, SegmentKind, displacement, gas, intake_runner_of};
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
    let (mut pipe, collector) = common::fitted_exhaust(&spec);
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

/// `common::fitted_exhaust` is the web app's, as the presets fixture records it.
#[test]
fn fitted_exhaust_matches_the_web_app() {
    let shape = |s: &[PipeSegment]| -> Vec<(SegmentKind, f64, f64, f64, f64)> {
        s.iter().map(|g| (g.kind, g.length, g.d_in, g.d_out, g.yaw)).collect()
    };
    for preset in &common::presets().engine_presets {
        let (pipe, collector) = common::fitted_exhaust(&preset.config.engine);
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
        common::gas_torque(&mut sim)
    }

    /// lifts the mid-range of an engine cammed for the top end
    ///
    /// A cam tuned for 8400 rpm gives up the mid-range; advancing it there gives it back.
    #[test]
    fn lifts_the_mid_range_of_an_engine_cammed_for_the_top_end() {
        let fixed = || json!({ "vvtIntakeLow": 0, "vvtExhaustLow": 0 });
        let [phased, still, top_phased, top_still] = common::par(
            [(4500.0, json!({})), (4500.0, fixed()), (8400.0, json!({})), (8400.0, fixed())],
            |(rpm, over)| torque_at(*rpm, over.clone()),
        );
        assert!(phased > 1.1 * still, "at 4500: phased {phased} fixed {still}");
        // At the top the map takes its high-speed settings, which the low-speed ones leave alone.
        let ratio = top_phased / top_still;
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
        common::gas_torque(&mut sim)
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
        // Just below the switch, where the two sets are nearest each other.
        let below = switch_rpm() - 100.0;
        let [both, short, top_both, top_short] = common::par(
            [(below, json!({})), (below, short_only()), (8400.0, json!({})), (8400.0, short_only())],
            |(rpm, over)| torque_at(*rpm, over.clone()),
        );
        assert!(both > 1.003 * short, "at {below}: two-stage {both} short only {short}");
        let ratio = top_both / top_short;
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

mod plenum {
    use super::*;
    use engine_sim::plenum::{plenum_count_of, plenum_shape_of, plenum_volume_of, runner_span_of, throttle_dia_of};

    /// The F20C at full throttle, settled, and its plenum's front and back zones' gauge pressure over
    /// the next quarter second.
    fn front_and_back(over: Value) -> (Vec<f64>, Vec<f64>) {
        let preset = common::engine_preset("Inline four, Honda F20C");
        let mut cfg = preset.config.clone();
        cfg.engine = common::with(
            &cfg.engine,
            merged(json!({ "rpm": 6000, "throttle": 1, "freeRunning": false, "combustionVariability": 0 }), over),
        );
        let mut sim = EngineSim::new(FS, &cfg);
        sim.render(FS as usize);
        let (mut front, mut back) = (Vec::new(), Vec::new());
        for _ in 0..FS as usize / 4 {
            sim.tick();
            let zones: Vec<f64> = sim.plenum().zone_pressures().collect();
            front.push(zones[0]);
            back.push(*zones.last().unwrap());
        }
        (front, back)
    }

    fn mean(v: &[f64]) -> f64 {
        v.iter().sum::<f64>() / v.len() as f64
    }

    /// holds as much as it is set to, from its size or from its volume
    #[test]
    fn holds_as_much_as_it_is_set_to_from_its_size_or_from_its_volume() {
        let spec = v8_spec(json!({}));
        let auto = plenum_volume_of(&spec);
        let swept = displacement(&spec) * 8.0;
        assert!((auto / swept - 1.5).abs() < 1e-9, "auto {auto} swept {swept}");
        let sized = v8_spec(json!({ "plenumLength": 0.8, "plenumWidth": 0.25, "plenumHeight": 0.2, "plenumTaper": 0.4 }));
        assert!((plenum_volume_of(&sized) - 0.8 * 0.25 * 0.2 * 0.8).abs() < 1e-12);
        // Left to work out its width, it holds the volume asked for at any length.
        let long = v8_spec(json!({ "plenumLength": 0.7, "plenumVolume": 0.004 }));
        assert!((plenum_volume_of(&long) - 0.004).abs() < 1e-12);
        assert_eq!(plenum_shape_of(&long).length, 0.7);
    }

    /// is never set shorter than the row of runners it feeds
    #[test]
    fn is_never_set_shorter_than_the_row_of_runners_it_feeds() {
        let span = runner_span_of(&v8_spec(json!({})));
        let short = v8_spec(json!({ "plenumLength": 0.05 }));
        assert_eq!(plenum_shape_of(&short).length, span);
        assert!(plenum_shape_of(&v8_spec(json!({}))).length >= span);
        let long = v8_spec(json!({ "plenumLength": span + 0.1 }));
        assert_eq!(plenum_shape_of(&long).length, span + 0.1);
    }

    /// is solved along its length: its ends breathe apart, but feed the runners the same air on average
    #[test]
    fn is_solved_along_its_length() {
        let (front, back) = front_and_back(json!({}));
        let swing: f64 = (front.iter().zip(&back).map(|(f, b)| (f - b) * (f - b)).sum::<f64>() / front.len() as f64).sqrt();
        assert!(swing > 500.0, "front and back differ by {swing} Pa rms");
        let (f, b) = (mean(&front), mean(&back));
        assert!((f - b).abs() < 100.0, "front {f} back {b}");
    }

    /// breathes apart along it more the longer it is
    #[test]
    fn breathes_apart_along_it_more_the_longer_it_is() {
        let apart = |length: f64| {
            let (front, back) = front_and_back(json!({ "plenumLength": length }));
            (front.iter().zip(&back).map(|(f, b)| (f - b) * (f - b)).sum::<f64>() / front.len() as f64).sqrt()
        };
        let (short, long) = (apart(0.2), apart(0.8));
        assert!(long > short * 1.5, "short {short} long {long}");
    }

    /// The LT6 at full throttle and `rpm`, settled, with dual plenums whose balance valves open at
    /// `balance` rpm: whether they are open, and the rms of the difference between the two plenums' mean
    /// gauge pressures over the next quarter second, Pa.
    fn dual_apart(rpm: f64, balance: f64) -> (bool, f64) {
        let preset = common::engine_preset("V8, Chevrolet LT6");
        let mut cfg = preset.config.clone();
        cfg.engine = common::with(
            &cfg.engine,
            json!({ "rpm": rpm, "throttle": 1, "freeRunning": false, "combustionVariability": 0,
                    "dualPlenum": true, "plenumBalanceRpm": balance, "plenumBalanceShutRpm": 0 }),
        );
        let mut sim = EngineSim::new(FS, &cfg);
        sim.render(FS as usize);
        let mut sum = 0.0;
        let count = FS as usize / 4;
        for _ in 0..count {
            sim.tick();
            let zones: Vec<f64> = sim.plenum().zone_pressures().collect();
            let (a, b) = zones.split_at(zones.len() / 2);
            let d = mean(a) - mean(b);
            sum += d * d;
        }
        (sim.snapshot().plenum_balanced, (sum / count as f64).sqrt())
    }

    /// dual plenums: one for each bank, holding between them what one would, each with its own throttle
    #[test]
    fn dual_plenums_are_one_for_each_bank_holding_what_one_would() {
        let single = v8_spec(json!({}));
        let dual = v8_spec(json!({ "dualPlenum": true }));
        assert_eq!(plenum_count_of(&single), 1);
        assert_eq!(plenum_count_of(&dual), 2);
        assert!((plenum_volume_of(&dual) / plenum_volume_of(&single) - 1.0).abs() < 1e-9);
        // Each throttle sized for half the air, the two together as one would be.
        assert!((throttle_dia_of(&dual) * 2f64.sqrt() - throttle_dia_of(&single)).abs() < 1e-12);
        // An 87 mm bore given is each one's.
        let lt6 = v8_spec(json!({ "dualPlenum": true, "throttleDia": 0.087 }));
        assert_eq!(throttle_dia_of(&lt6), 0.087);
        // Wide enough for both flanges side by side.
        let narrow = v8_spec(json!({ "dualPlenum": true, "throttleDia": 0.087, "plenumWidth": 0.1 }));
        assert!(plenum_shape_of(&narrow).width > 2.0 * 0.087);
        // An inline engine has only the one.
        let inline = common::with(&common::presets().default_engine, json!({ "cylinders": 4, "vAngle": 0, "dualPlenum": true }));
        assert_eq!(plenum_count_of(&inline), 1);
        let mut cfg = common::engine_preset("V8, Chevrolet LT6").config.clone();
        let zones = |cfg: &engine_sim::spec::EngineConfig| EngineSim::new(FS, cfg).plenum().zone_count();
        cfg.engine = common::with(&cfg.engine, json!({ "dualPlenum": false }));
        let one = zones(&cfg);
        cfg.engine = common::with(&cfg.engine, json!({ "dualPlenum": true }));
        assert_eq!(zones(&cfg), 2 * one);
    }

    /// dual plenums breathe apart, each with its own bank, until the balance valves join them
    #[test]
    fn dual_plenums_breathe_apart_until_the_balance_valves_join_them() {
        let (open, shut) = (dual_apart(6000.0, 5000.0), dual_apart(6000.0, 7000.0));
        assert!(open.0 && !shut.0, "open {open:?} shut {shut:?}");
        assert!(shut.1 > 1.3 * open.1, "apart by {} Pa shut, {} Pa open", shut.1, open.1);
        // Never, at 0.
        assert!(!dual_apart(6000.0, 0.0).0);
    }

    /// dual plenums' balance valves open across their band, and shut above it and below it
    #[test]
    fn dual_plenums_balance_valves_open_across_their_band() {
        let at = |rpm: f64| {
            let preset = common::engine_preset("V8, Chevrolet LT6");
            let mut cfg = preset.config.clone();
            cfg.engine = common::with(
                &cfg.engine,
                json!({ "rpm": rpm, "throttle": 1, "freeRunning": false, "plenumBalanceRpm": 4000, "plenumBalanceShutRpm": 6000 }),
            );
            let mut sim = EngineSim::new(FS, &cfg);
            sim.render(FS as usize / 10);
            sim.snapshot().plenum_balanced
        };
        assert_eq!([at(3000.0), at(5000.0), at(7000.0)], [false, true, false]);
    }
}
