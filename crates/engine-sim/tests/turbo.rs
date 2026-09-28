//! The turbocharger: boost built by the exhaust and held by the wastegate, the lag behind the
//! throttle, what the blow-off valve does when the throttle shuts and what happens without one, and
//! the whine of the compressor; and the turbine running on its map.
//!
//! The Nissan RB26DETT preset is the turbocharged engine throughout.

mod common;

use common::FS;
use engine_sim::EngineSim;
use engine_sim::exhaust_graph::{DuctSink, DuctSource, TurboMount, compile_exhaust};
use engine_sim::turbo;
use serde_json::{Value, json};

const RB26: &str = "Inline six, Nissan RB26DETT";

fn rb26(over: Value) -> EngineSim {
    let mut cfg = common::engine_preset(RB26).config.clone();
    cfg.engine = common::with(&cfg.engine, json!({ "freeRunning": false, "combustionVariability": 0 }));
    cfg.engine = common::with(&cfg.engine, over);
    EngineSim::new(FS, &cfg)
}

/// Boost, bar gauge.
fn boost(sim: &EngineSim) -> f64 {
    sim.turbo().unwrap().boost() / 1e5
}

/// Mean torque at the crank, less friction, over half a second, N*m, after `settle` seconds at `rpm` on
/// full throttle, with the fields in `over` on top.
fn torque_at(rpm: f64, settle: f64, over: Value) -> f64 {
    let mut o = json!({ "throttle": 1, "rpm": rpm });
    o.as_object_mut().unwrap().extend(over.as_object().unwrap().clone());
    let mut sim = rb26(o);
    sim.render((settle * FS) as usize);
    let n = FS as usize / 2;
    let mut t = 0.0;
    for _ in 0..n {
        sim.render(1);
        t += sim.snapshot().torque - sim.friction_torque();
    }
    t / n as f64
}

/// Held on boost at `rpm`, then the throttle shut: the compressor flow every sample for half a second.
fn lift_off(over: Value) -> (EngineSim, Vec<f64>) {
    let mut o = json!({ "throttle": 1, "rpm": 4000 });
    o.as_object_mut().unwrap().extend(over.as_object().unwrap().clone());
    let mut sim = rb26(o);
    sim.render(3 * FS as usize);
    assert!(boost(&sim) > 0.6, "on boost before the lift: {}", boost(&sim));
    sim.set_controls(0.0, 0.0);
    let mut flow = Vec::new();
    for _ in 0..FS as usize / 2 {
        sim.render(1);
        flow.push(sim.turbo().unwrap().compressor_flow());
    }
    (sim, flow)
}

#[test]
fn a_naturally_aspirated_engine_has_no_turbo() {
    let mut sim = EngineSim::new(FS, &common::engine_preset("Inline four, Honda F20C").config);
    sim.render(FS as usize / 10);
    assert!(sim.turbo().is_none());
    assert!(sim.snapshot().turbo.is_none());
}

/// Taking the turbos out of the exhaust leaves the engine breathing the atmosphere again.
#[test]
fn taking_the_turbos_out_leaves_the_engine_breathing_the_atmosphere() {
    let mut sim = rb26(json!({ "throttle": 1, "rpm": 5000 }));
    sim.render(2 * FS as usize);
    assert!(boost(&sim) > 0.5);
    let mut graph = common::engine_preset(RB26).config.graph.clone().unwrap();
    graph.turbos.clear();
    sim.set_graph(Some(graph));
    sim.render(FS as usize);
    assert!(sim.turbo().is_none());
    assert!(sim.snapshot().turbo.is_none());
    let manifold = sim.plenum().pressure();
    assert!(manifold < 1.02e5, "no boost left: {manifold}");
}

/// Boost builds with the exhaust flow, and the wastegate holds it at its target.
#[test]
fn builds_boost_with_the_exhaust_and_the_wastegate_holds_it() {
    let at = |rpm: f64| {
        let mut sim = rb26(json!({ "throttle": 1, "rpm": rpm }));
        sim.render(3 * FS as usize);
        (boost(&sim), sim.turbo().unwrap().wastegate())
    };
    let (low, wg_low) = at(1500.0);
    let (mid, wg_mid) = at(4000.0);
    let (high, wg_high) = at(6500.0);
    assert!(low < 0.5, "too little exhaust at 1500 rpm for full boost: {low}");
    assert!((mid - 0.8).abs() < 0.06, "held at the target at 4000 rpm: {mid}");
    assert!((high - 0.8).abs() < 0.06, "held at the target at 6500 rpm: {high}");
    // Shut below the target, and open once there. How far depends on the pulses the manifold delivers as
    // well as on the flow, and on the drawn exhaust it is furthest open around 5000 rpm.
    assert!(
        wg_mid > wg_low + 0.1 && wg_high > wg_low + 0.1,
        "the wastegate opens to hold the target: {wg_low} {wg_mid} {wg_high}"
    );
}

/// A higher target is reached too, and makes more torque: the turbine's nozzle is sized for it, and
/// the turbo left on auto for the airflow it brings.
#[test]
fn reaches_a_higher_boost_target() {
    let at = |target: f64| {
        let mut sim = rb26(json!({ "throttle": 1, "rpm": 4000, "boostTarget": target * 1e5, "turboSize": 0 }));
        sim.render(4 * FS as usize);
        let n = FS as usize / 2;
        let mut t = 0.0;
        for _ in 0..n {
            sim.render(1);
            t += sim.snapshot().torque;
        }
        (boost(&sim), t / n as f64)
    };
    let (low, t_low) = at(0.7);
    let (high, t_high) = at(2.0);
    assert!((high - 2.0).abs() < 0.1, "held at 2 bar: {high}");
    assert!(t_high > 1.4 * t_low, "more torque on more boost: {t_high} N*m at {high} bar, {t_low} at {low}");
}

/// The shaft takes time to spin up: opened from part throttle, the boost lags behind.
#[test]
fn lags_behind_the_throttle() {
    let mut sim = rb26(json!({ "throttle": 0.12, "rpm": 3500 }));
    sim.render(2 * FS as usize);
    assert!(boost(&sim) < 0.1, "off boost at part throttle: {}", boost(&sim));
    sim.set_controls(1.0, 0.0);
    let mut reached = None;
    for i in 1..=60 {
        sim.render(FS as usize / 20);
        if boost(&sim) > 0.9 * 0.8 {
            reached = Some(i as f64 / 20.0);
            break;
        }
    }
    let t = reached.expect("reaches full boost within 3 s");
    assert!(t > 0.2 && t < 2.5, "90% of the boost after {t} s");
}

/// Floored from idle, the charge air sloshing back through the compressor as the throttle opens slows
/// the shaft but does not stop it, and the engine revs up on boost; again after each lift.
#[test]
fn spools_up_from_idle_every_time() {
    let mut sim = rb26(json!({ "throttle": 0, "rpm": 900, "freeRunning": true }));
    for blip in 0..3 {
        sim.set_controls(0.0, 0.0);
        sim.render(2 * FS as usize);
        sim.set_controls(1.0, 0.0);
        let (mut slowest, mut most) = (f64::INFINITY, f64::NEG_INFINITY);
        for _ in 0..2 * FS as usize {
            sim.render(1);
            slowest = slowest.min(sim.turbo().unwrap().shaft_rpm());
            most = most.max(boost(&sim));
        }
        assert!(slowest > 1000.0, "blip {blip}: the shaft kept turning, down to {slowest} rpm");
        // Once it reaches the rev limiter, the cut fuel starves the turbine and the boost falls back.
        assert!(most > 0.5, "blip {blip}: on boost: {most}");
    }
}

/// About the real engine's 368 N*m at 4400 rpm, and its power at 6800 no less than the 280 PS it was
/// rated at nor much more than the 320 or so real ones make.
#[test]
fn makes_about_the_real_engines_torque_and_power() {
    let t4400 = torque_at(4400.0, 3.0, json!({}));
    assert!((t4400 - 368.0).abs() < 0.1 * 368.0, "{t4400} N*m at 4400 rpm");
    let t6800 = torque_at(6800.0, 3.0, json!({}));
    let ps = t6800 * 6800.0 * 2.0 * std::f64::consts::PI / 60.0 / 735.5;
    assert!(ps > 280.0 && ps < 350.0, "{ps} PS at 6800 rpm");
}

/// Turbos too small for the engine run out of air at the top end: at their choke, spun faster, they
/// pass little more air, so the power falls away rather than holding level to the limit.
#[test]
fn too_small_run_out_of_air_at_the_top_end() {
    let power = |rpm: f64| torque_at(rpm, 3.0, json!({ "turboSize": 0.12 })) * rpm;
    let (at_7000, at_7900) = (power(7000.0), power(7900.0));
    assert!(at_7900 < 0.99 * at_7000, "power at 7900 {at_7900} against 7000 {at_7000}");
}

/// A blow-off valve vents the charge when the throttle shuts, so the compressor never runs backwards.
#[test]
fn a_blow_off_valve_vents_the_charge_when_the_throttle_shuts() {
    let (sim, flow) = lift_off(json!({ "blowOff": "atmospheric" }));
    assert!(flow.iter().all(|&m| m > 0.0), "the compressor never surges");
    assert!(sim.turbo().unwrap().blow_off() > 0.9, "the valve is open on the vacuum");
    assert!(boost(&sim) < 0.2, "the boost is let go: {}", boost(&sim));
}

/// With nowhere for the charge to go, the compressor stalls and recovers over and over: a surge,
/// cycling at a few tens of hertz, which is the flutter.
#[test]
fn without_a_blow_off_valve_the_compressor_surges() {
    let (_, flow) = lift_off(json!({ "blowOff": "none" }));
    let mut reversals = 0;
    for w in flow.windows(2) {
        if w[0] >= 0.0 && w[1] < 0.0 {
            reversals += 1;
        }
    }
    let hz = reversals as f64 / 0.5;
    assert!(hz > 5.0 && hz < 60.0, "{reversals} surge cycles in half a second");
}

/// On boost, what the compressor radiates from its inlet has its strongest tone at the blade-pass
/// frequency.
#[test]
fn whines_at_the_blade_pass_frequency() {
    let mut sim = rb26(json!({ "throttle": 1, "rpm": 2500 }));
    sim.render(3 * FS as usize);
    let bpf = sim.turbo().unwrap().blade_pass_hz();
    assert!(bpf > 3000.0 && bpf < 15000.0, "blade pass {bpf} Hz");
    let size = 16384;
    let mut turbo = Vec::with_capacity(size);
    for _ in 0..size {
        sim.render(1);
        turbo.push(sim.turbo().unwrap().last_sound() as f32);
    }
    let mag = common::magnitude_spectrum(&common::hann(&turbo), size);
    let peaks = common::find_peaks(&mag, FS, size, 1000.0, 16000.0, 0.1);
    let top = peaks[0];
    assert!((top.hz - bpf).abs() < 0.03 * bpf, "strongest tone {} Hz, blade pass {bpf} Hz", top.hz);
}

/// The turbine sits in the exhaust: everything the cylinders push out passes through it, from a
/// higher pressure on its inlet side than on its outlet side.
#[test]
fn the_turbine_passes_the_exhaust_between_two_pressures() {
    let mut sim = rb26(json!({ "throttle": 1, "rpm": 4400 }));
    assert_eq!(sim.pipe_solver().turbine_count(), 2, "two turbines, one for each three cylinders");
    sim.render(3 * FS as usize);
    let n = FS as usize;
    let (mut valves, mut through, mut inlet, mut outlet) = (0.0, 0.0, 0.0, 0.0);
    for _ in 0..n {
        sim.render(1);
        let r = &sim.pipe_solver().result;
        valves += r.valve_mass_flows.iter().sum::<f64>();
        through += r.turbine_flow + r.bypass_flow;
        inlet += r.turbine_inlet;
        outlet += r.turbine_outlet;
    }
    assert!(
        (through - valves).abs() < 0.03 * valves,
        "turbine {} kg/s, valves {} kg/s",
        through / n as f64,
        valves / n as f64
    );
    let rise = (inlet - outlet) / n as f64;
    assert!(rise > 0.2e5, "the exhaust backs up behind the turbine: {rise} Pa");
    assert_eq!(sim.pipe_solver().recoveries(), 0);
}

/// Each exhaust pulse gives up much of itself to the turbine: the pressure past it swings far less
/// than the pressure arriving at it.
#[test]
fn the_turbine_takes_the_edge_off_the_pulses() {
    let mut sim = rb26(json!({ "throttle": 1, "rpm": 4400 }));
    sim.render(3 * FS as usize);
    let n = FS as usize / 2;
    let mut up = Vec::with_capacity(n);
    let mut down = Vec::with_capacity(n);
    for _ in 0..n {
        sim.render(1);
        let r = &sim.pipe_solver().result;
        up.push(r.turbine_inlet);
        down.push(r.turbine_outlet);
    }
    let swing = |v: &[f64]| {
        let mean = v.iter().sum::<f64>() / v.len() as f64;
        (v.iter().map(|x| (x - mean) * (x - mean)).sum::<f64>() / v.len() as f64).sqrt()
    };
    let (su, sd) = (swing(&up), swing(&down));
    assert!(sd < 0.7 * su, "pulses {sd} Pa past the turbine against {su} Pa arriving");
}

/// The turbine's map: at its best blade speed ratio it takes its peak efficiency's share of the
/// isentropic drop, a stalled wheel takes none, and a wheel spinning twice as fast as that brakes.
#[test]
fn the_turbine_map_peaks_at_its_best_blade_speed_ratio() {
    let isentropic: f64 = 150e3;
    let c0 = (2.0 * isentropic).sqrt();
    let eta = |x: f64| turbo::turbine_work(x * c0, isentropic) / isentropic;
    assert!((eta(0.7) - turbo::turbine_efficiency(0.7)).abs() < 1e-12);
    assert!(eta(0.7) > eta(0.5) && eta(0.7) > eta(0.9), "best at 0.7");
    assert!(eta(0.0).abs() < 1e-12, "a stalled wheel does no work");
    assert!(eta(1.4).abs() < 1e-9 && eta(1.6) < 0.0, "a wheel outrunning the gas brakes");
    assert!(turbo::turbine_work(300.0, 0.0) < 0.0, "and churns gas with no drop across it");
}

/// On boost, fed pulses, the turbine runs below its best blade speed ratio, near its best
/// efficiency. Shut the throttle and the wheel outruns what is left of the exhaust, and brakes.
#[test]
fn the_turbine_runs_on_its_map() {
    let mut sim = rb26(json!({ "throttle": 1, "rpm": 4400 }));
    sim.render(3 * FS as usize);
    let t = sim.turbo().unwrap();
    let (eta, bsr) = (t.turbine_efficiency(), t.blade_speed_ratio());
    assert!(bsr > 0.35 && bsr < 0.7, "blade speed ratio on boost: {bsr}");
    assert!(eta > 0.6 && eta < 0.78, "efficiency on boost: {eta}");
    sim.set_controls(0.0, 0.0);
    let mut worst = f64::INFINITY;
    for _ in 0..FS as usize / 2 {
        sim.render(1);
        worst = worst.min(sim.turbo().unwrap().turbine_efficiency());
    }
    assert!(worst < 0.0, "braking once the throttle shuts: {worst}");
}

/// A single, its one pipe drawn into a turbo: a turbine with one pipe in and one out.
fn single_into_a_turbo(connected: bool) -> EngineSim {
    let mut cfg = common::default_config();
    cfg.engine = common::with(
        &cfg.engine,
        json!({ "throttle": 1, "rpm": 6000, "freeRunning": false, "combustionVariability": 0 }),
    );
    let mut graph = compile_exhaust(&cfg.engine, &cfg.pipe, &cfg.collector);
    if connected {
        graph.ducts[0].to = DuctSink::Node { node: "t".into() };
        let mut out = graph.ducts[0].clone();
        out.id = "turbo-out".into();
        out.from = DuctSource::Node { node: "t".into() };
        out.to = DuctSink::Mouth;
        out.segments.truncate(1);
        graph.ducts.push(out);
    }
    graph.turbos.push(TurboMount { id: "turbo1".into(), node: "t".into(), position: None, rotation: None });
    cfg.graph = Some(graph);
    EngineSim::new(FS, &cfg)
}

/// Any engine takes a turbo: a single's one pipe drawn into one makes boost.
#[test]
fn a_single_drawn_into_a_turbo_makes_boost() {
    let mut sim = single_into_a_turbo(true);
    assert_eq!(sim.pipe_solver().turbine_count(), 1);
    sim.render(4 * FS as usize);
    assert!(boost(&sim) > 0.3, "boost {}", boost(&sim));
}

/// A turbo put down with nothing attached to it does nothing.
#[test]
fn a_turbo_with_nothing_attached_does_nothing() {
    let mut sim = single_into_a_turbo(false);
    assert_eq!(sim.pipe_solver().turbine_count(), 0);
    sim.render(FS as usize / 10);
    assert!(sim.turbo().is_none());
}

/// The RB26 with each cylinder's runner drawn into a turbo of its own, the six outlets joining.
fn rb26_six_turbos() -> engine_sim::exhaust_graph::ExhaustGraph {
    let mut graph = common::engine_preset(RB26).config.graph.clone().unwrap();
    let outlet = graph.ducts.iter().find(|d| d.id == "drawn8").unwrap().clone();
    graph.ducts.retain(|d| d.id != "drawn7" && d.id != "drawn8");
    graph.turbos.clear();
    let runners: Vec<usize> =
        (0..graph.ducts.len()).filter(|&i| matches!(graph.ducts[i].from, DuctSource::Valve { .. })).collect();
    for (n, &i) in runners.iter().enumerate() {
        let node = format!("t{n}");
        graph.ducts[i].to = DuctSink::Node { node: node.clone() };
        let mut out = outlet.clone();
        out.id = format!("turbo-out{n}");
        out.from = DuctSource::Node { node: node.clone() };
        graph.ducts.push(out);
        graph.turbos.push(TurboMount { id: format!("turbo{n}"), node, position: None, rotation: None });
    }
    graph
}

/// Six turbos, one on each cylinder, drawn in while the engine runs on two: all six turn, make
/// boost and whine, and going back to two leaves the engine running as before.
#[test]
fn takes_any_number_of_turbos() {
    let mut sim = rb26(json!({ "throttle": 1, "rpm": 5000 }));
    sim.render(FS as usize);
    sim.set_graph(Some(rb26_six_turbos()));
    assert_eq!(sim.pipe_solver().turbine_count(), 6);
    let mut loudest = 0.0f64;
    for _ in 0..3 * FS as usize {
        sim.render(1);
        let sound = sim.turbo().unwrap().last_sound();
        assert!(sound.is_finite());
        loudest = loudest.max(sound.abs());
    }
    assert!(loudest > 0.0, "the turbos sound");
    assert!(boost(&sim) > 0.3, "boost {}", boost(&sim));

    sim.set_graph(common::engine_preset(RB26).config.graph.clone());
    assert_eq!(sim.pipe_solver().turbine_count(), 2);
    sim.render(2 * FS as usize);
    assert!(boost(&sim) > 0.5, "boost {}", boost(&sim));
}
