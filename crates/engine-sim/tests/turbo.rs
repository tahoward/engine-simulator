//! The turbocharger: boost built by the exhaust and held by the wastegate, the lag behind the
//! throttle, what the blow-off valve does when the throttle shuts and what happens without one, and
//! the whine of the compressor; and the turbine running on its map.
//!
//! The Nissan RB26DETT preset is the turbocharged engine throughout, but for the Ford Dragon's, the Toyota
//! 3S-GTE's, the Mazda BPT's and the Audi EA855 EVO's own torque and power: a three, two fours and a five,
//! each on one turbo.

use crate::common;

use common::FS;
use engine_sim::EngineSim;
use engine_sim::exhaust_graph::{DuctSink, DuctSource, TurboMount, TurboSettings, compile_exhaust};
use engine_sim::spec::BlowOff;
use engine_sim::turbo;
use serde_json::{Value, json};

const RB26: &str = "Inline six, Nissan RB26DETT";
/// The preset's boost target, bar gauge.
const TARGET: f64 = 0.7;

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
    // Read as a gauge reads it, over a tenth of a second: the runners and the plenum's waves ride on it.
    let at = |rpm: f64| {
        let mut sim = rb26(json!({ "throttle": 1, "rpm": rpm }));
        sim.render(3 * FS as usize);
        let samples = FS as usize / 10;
        let mean = (0..samples)
            .map(|_| {
                sim.render(1);
                boost(&sim)
            })
            .sum::<f64>()
            / samples as f64;
        (mean, sim.turbo().unwrap().wastegate())
    };
    let (low, wg_low) = at(1500.0);
    let (mid, wg_mid) = at(4000.0);
    let (high, wg_high) = at(6500.0);
    assert!(low < 0.5, "too little exhaust at 1500 rpm for full boost: {low}");
    assert!((mid - TARGET).abs() < 0.06, "held at the target at 4000 rpm: {mid}");
    assert!((high - TARGET).abs() < 0.06, "held at the target at 6500 rpm: {high}");
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
        let (mut t, mut b) = (0.0, 0.0);
        for _ in 0..n {
            sim.render(1);
            t += sim.snapshot().torque;
            b += boost(&sim);
        }
        (b / n as f64, t / n as f64)
    };
    let (low, t_low) = at(0.7);
    let (high, t_high) = at(2.0);
    assert!((high - 2.0).abs() < 0.1, "held at 2 bar: {high}");
    assert!(t_high > 1.4 * t_low, "more torque on more boost: {t_high} N*m at {high} bar, {t_low} at {low}");
}

/// Held on boost, the wastegate settles rather than hunting from shut to wide open: at high boost the
/// turbine has the most power over what it needs, and a controller whose band does not grow with the
/// target swings it, and the boost and the torque with it, a few times a second.
#[test]
fn the_wastegate_settles_on_high_boost_rather_than_hunting() {
    for (name, rpm, target) in [
        ("Inline five, Audi EA855 EVO", 5000.0, None),
        ("Inline three, Ford 1.5 EcoBoost Dragon", 5000.0, None),
        (RB26, 5000.0, Some(2.0e5)),
    ] {
        let mut cfg = common::engine_preset(name).config.clone();
        let mut over = json!({ "freeRunning": false, "throttle": 1, "rpm": rpm });
        if let Some(t) = target {
            over["boostTarget"] = json!(t);
        }
        cfg.engine = common::with(&cfg.engine, over);
        let goal = cfg.engine.boost_target / 1e5;
        let mut sim = EngineSim::new(FS, &cfg);
        sim.render(3 * FS as usize);
        // Sampled off any multiple of the firing interval, so the pulses average out.
        let (mut lo, mut hi, mut sum, mut n) = (f64::MAX, f64::MIN, 0.0, 0.0);
        for _ in 0..1400 {
            sim.render(67);
            let t = sim.turbo().unwrap();
            lo = lo.min(t.wastegate());
            hi = hi.max(t.wastegate());
            sum += boost(&sim);
            n += 1.0;
        }
        assert!(hi - lo < 0.15, "{name} on {goal} bar: wastegate swings {lo:.2}..{hi:.2}");
        let mean = sum / n;
        assert!((mean - goal).abs() < 0.05 * goal, "{name}: held {mean:.2} bar for {goal}");
    }
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
        if boost(&sim) > 0.9 * TARGET {
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
    let [t4400, t6800] = common::brake_torques_at(RB26, 3.0, [4400.0, 6800.0]);
    assert!((t4400 - 368.0).abs() < 0.1 * 368.0, "{t4400} N*m at 4400 rpm");
    let ps = common::ps(t6800, 6800.0);
    assert!(ps > 280.0 && ps < 350.0, "{ps} PS at 6800 rpm");
}

/// The 3S-GTE, one turbo on four cylinders, on its 0.7 bar makes about the Japanese engine's rated
/// 304 N*m once on boost and 225 PS at 6000 rpm. The real one has its peak torque at 3200, where this
/// turbo is still spooling.
#[test]
fn a_four_on_one_turbo_makes_about_the_real_engines_torque_and_power() {
    let [t5000, t6000] = common::brake_torques_at("Inline four, Toyota 3S-GTE", 3.0, [5000.0, 6000.0]);
    assert!((t5000 - 304.0).abs() < 0.1 * 304.0, "{t5000} N*m at 5000 rpm");
    let ps = common::ps(t6000, 6000.0);
    assert!((ps - 225.0).abs() < 0.1 * 225.0, "{ps} PS at 6000 rpm");
}

/// The Mazda BPT, one small turbo on four cylinders, on its 0.5 bar makes about the real engine's rated
/// 166 lb-ft (225 N*m) at 4500 rpm and 178 hp at 6000.
#[test]
fn the_mazda_bpt_makes_about_the_real_engines_torque_and_power() {
    let [t4500, t6000] = common::brake_torques_at("Inline four, Mazda BPT", 3.0, [4500.0, 6000.0]);
    assert!((t4500 - 225.0).abs() < 0.1 * 225.0, "{t4500} N*m at 4500 rpm");
    let hp = common::hp(t6000, 6000.0);
    assert!((hp - 178.0).abs() < 0.1 * 178.0, "{hp} hp at 6000 rpm");
}

/// The 1.5 EcoBoost Dragon, one turbo on three cylinders, on its 1.15 bar makes about the real engine's
/// rated 290 N*m once on boost and 200 PS at 6000 rpm. The real one has its peak torque from 1600, where
/// this turbo is still spooling.
#[test]
fn a_three_on_one_turbo_makes_about_the_real_engines_torque_and_power() {
    let [t3000, t6000] = common::brake_torques_at("Inline three, Ford 1.5 EcoBoost Dragon", 3.0, [3000.0, 6000.0]);
    assert!((t3000 - 290.0).abs() < 0.1 * 290.0, "{t3000} N*m at 3000 rpm");
    let ps = common::ps(t6000, 6000.0);
    assert!((ps - 200.0).abs() < 0.1 * 200.0, "{ps} PS at 6000 rpm");
}

/// The EA855 EVO, one turbo on five cylinders, on its 1.35 bar makes about the real engine's rated
/// 480 N*m through the mid-range and 400 PS from 5850 rpm to 7000.
#[test]
fn a_five_on_one_turbo_makes_about_the_real_engines_torque_and_power() {
    let rpms = [4500.0, 5850.0, 7000.0];
    let [t4500, t5850, t7000] = common::brake_torques_at("Inline five, Audi EA855 EVO", 3.0, rpms);
    assert!((t4500 - 480.0).abs() < 0.1 * 480.0, "{t4500} N*m at 4500 rpm");
    for (rpm, torque) in [(5850.0, t5850), (7000.0, t7000)] {
        let ps = common::ps(torque, rpm);
        assert!((ps - 400.0).abs() < 0.1 * 400.0, "{ps} PS at {rpm} rpm");
    }
}

/// Turbos too small for the engine run out of air at the top end: at their choke, spun faster, they
/// pass little more air, so the power falls away rather than holding level to the limit.
#[test]
fn too_small_run_out_of_air_at_the_top_end() {
    let small = || json!({ "turboSize": 0.12 });
    let [t7000, t7900] = common::brake_torques(RB26, 3.0, [(7000.0, small()), (7900.0, small())]);
    let (at_7000, at_7900) = (t7000 * 7000.0, t7900 * 7900.0);
    assert!(at_7900 < 0.99 * at_7000, "power at 7900 {at_7900} against 7000 {at_7000}");
}

/// Times the compressor's flow turns backwards in `flow`.
fn reversals(flow: &[f64]) -> usize {
    flow.windows(2).filter(|w| w[0] >= 0.0 && w[1] < 0.0).count()
}

/// A blow-off valve vents the charge when the throttle shuts. The pressure wave the throttle sends back
/// up the charge pipe as it slams shut reaches the compressor before the valve has lifted, and pushes
/// the flow back through it the once; from then on the valve lets the charge go, and it never surges.
#[test]
fn a_blow_off_valve_vents_the_charge_when_the_throttle_shuts() {
    let (sim, flow) = lift_off(json!({ "blowOff": "atmospheric" }));
    assert!(reversals(&flow) <= 1, "{} reversals", reversals(&flow));
    let settled = FS as usize / 20;
    assert!(flow[settled..].iter().all(|&m| m > 0.0), "the compressor never surges once the valve is open");
    assert!(sim.turbo().unwrap().blow_off() > 0.9, "the valve is open on the vacuum");
    assert!(boost(&sim) < 0.2, "the boost is let go: {}", boost(&sim));
}

/// With nowhere for the charge to go, the compressor stalls and recovers over and over: a surge,
/// cycling at a few tens of hertz, which is the flutter. Each time the flow falls past the surge line
/// a rotating stall builds in the wheel, and each time it recovers the stall dies away.
#[test]
fn without_a_blow_off_valve_the_compressor_surges() {
    let mut sim = rb26(json!({ "throttle": 1, "rpm": 4000, "blowOff": "none" }));
    sim.render(3 * FS as usize);
    let steady = sim.turbo().unwrap().stall();
    assert!(steady < 0.01, "no stall on boost: {steady}");
    sim.set_controls(0.0, 0.0);
    let (mut flow, mut stall) = (Vec::new(), Vec::new());
    for _ in 0..FS as usize / 2 {
        sim.render(1);
        flow.push(sim.turbo().unwrap().compressor_flow());
        stall.push(sim.turbo().unwrap().stall());
    }
    let hz = reversals(&flow) as f64 / 0.5;
    assert!(hz > 5.0 && hz < 60.0, "{} surge cycles in half a second", reversals(&flow));
    let most = stall.iter().cloned().fold(0.0, f64::max);
    assert!(most > 0.5, "the wheel stalls: {most}");
    let least = stall[FS as usize / 20..].iter().cloned().fold(1.0, f64::min);
    assert!(least < 0.05, "and recovers between surges: {least}");
}

/// Lifted off with no blow-off valve, revving free, the surge dies away as the shaft slows: a slower
/// wheel's speed line is flatter at low flow, and has less to drive one with. Within a second and a
/// half the flow through it barely swings.
#[test]
fn without_a_blow_off_valve_the_surge_dies_away_as_the_shaft_slows() {
    let mut sim = rb26(json!({ "throttle": 0, "rpm": 900, "blowOff": "none", "freeRunning": true }));
    sim.render(2 * FS as usize);
    sim.set_controls(1.0, 0.0);
    sim.render((1.2 * FS) as usize);
    sim.set_controls(0.0, 0.0);
    let flow: Vec<f64> = (0..2 * FS as usize)
        .map(|_| {
            sim.render(1);
            sim.turbo().unwrap().compressor_flow()
        })
        .collect();
    let swing = |v: &[f64]| v.iter().cloned().fold(f64::MIN, f64::max) - v.iter().cloned().fold(f64::MAX, f64::min);
    let (early, late) = (swing(&flow[..FS as usize / 5]), swing(&flow[(1.5 * FS) as usize..]));
    assert!(early > 0.1, "a deep surge on the lift: {early} kg/s");
    assert!(late < 0.15 * early, "gone by 1.5 s: {late} kg/s against {early}");
}

/// On high boost too, the EA855 on its 1.35 bar, the surge starts with the lift and keeps on: the
/// charge trapped behind the shut throttle can never rest against a wheel making no flow, and goes on
/// blowing back through it, cycle after cycle.
#[test]
fn on_high_boost_the_surge_starts_with_the_lift() {
    let mut cfg = common::engine_preset("Inline five, Audi EA855 EVO").config.clone();
    cfg.engine = common::with(
        &cfg.engine,
        json!({ "throttle": 1, "rpm": 4500, "blowOff": "none", "freeRunning": false, "combustionVariability": 0 }),
    );
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(3 * FS as usize);
    sim.set_controls(0.0, 0.0);
    let flow: Vec<f64> = (0..(0.6 * FS) as usize)
        .map(|_| {
            sim.render(1);
            sim.turbo().unwrap().compressor_flow()
        })
        .collect();
    let starts: Vec<usize> = (1..flow.len()).filter(|&i| flow[i - 1] >= 0.0 && flow[i] < 0.0).collect();
    assert!(starts.len() >= 5, "{} surges in 0.6 s", starts.len());
    let longest = starts.windows(2).map(|w| w[1] - w[0]).max().unwrap() as f64 / FS;
    assert!(longest < 0.15, "no lull between surges: the longest {longest} s");
}

/// Switching from a naturally aspirated engine to a turbocharged one leaves the plenum in a deep vacuum
/// behind a throttle body at the atmosphere's pressure, and the blow-off valve wide open: what it lets
/// out as the pulses swing that pressure about the atmosphere's makes no sound to speak of.
#[test]
fn switching_to_a_turbocharged_engine_is_quiet() {
    let mut sim = EngineSim::new(FS, &common::engine_preset("Inline four, Honda F20C").config);
    sim.render(2 * FS as usize);
    let to = &common::engine_preset(RB26).config;
    sim.set_engine_json(&serde_json::to_value(&to.engine).unwrap()).unwrap();
    sim.set_graph(to.graph.clone());
    let mut loudest = 0.0f64;
    for _ in 0..FS as usize / 2 {
        sim.render(1);
        loudest = loudest.max(sim.turbo().unwrap().last_sound().abs());
    }
    assert!(sim.turbo().unwrap().blow_off() > 0.9, "the valve is open on the vacuum");
    assert!(loudest < 0.5, "the turbo stays quiet: {loudest} Pa");
}

/// Starting a launch from idle snaps the throttle open while the blow-off valve is still open on the
/// idle vacuum, venting the little the compressor delivers there. The throttle body empties in a few
/// samples, but the air in the valve's bore cannot stop at once, so the vent dies away without a click.
#[test]
fn starting_a_launch_does_not_click() {
    let preset = common::engine_preset(RB26);
    let mut sim = EngineSim::new(FS, &preset.config);
    sim.render(FS as usize);
    sim.start_launch(preset.launch.clone());
    let mut loudest = 0.0f64;
    for _ in 0..(0.008 * FS) as usize {
        sim.render(1);
        loudest = loudest.max(sim.turbo().unwrap().last_sound().abs());
    }
    assert!(loudest < 1.0, "the turbo is quiet as the throttle opens: {loudest} Pa");
}

/// The throttle shutting is felt at the compressor only once its pressure wave has run back up the
/// charge pipe: for the first few milliseconds the compressor goes on delivering as it would have with
/// the throttle held open, pulses and all.
#[test]
fn the_throttle_shutting_reaches_the_compressor_as_a_wave() {
    let (_, flow) = lift_off(json!({ "blowOff": "none" }));
    let mut held = rb26(json!({ "throttle": 1, "rpm": 4000, "blowOff": "none" }));
    held.render(3 * FS as usize);
    let unaware = (0.007 * FS) as usize;
    let open: Vec<f64> = (0..unaware)
        .map(|_| {
            held.render(1);
            held.turbo().unwrap().compressor_flow()
        })
        .collect();
    let before = open.iter().sum::<f64>() / unaware as f64;
    assert!(
        flow[..unaware].iter().zip(&open).all(|(&m, &o)| (m - o).abs() < 0.03 * before),
        "as if still open for 7 ms: {:?} against {:?}",
        &flow[..unaware],
        open
    );
    let reached = flow.iter().position(|&m| m < 0.5 * before).unwrap() as f64 / FS;
    assert!(reached < 0.02, "the flow falls once the wave arrives, after {reached} s");
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
        for t in &r.turbines {
            through += t.flow + t.bypass_flow;
            inlet += t.inlet / r.turbines.len() as f64;
            outlet += t.outlet / r.turbines.len() as f64;
        }
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
        up.push(r.turbines[0].inlet);
        down.push(r.turbines[0].outlet);
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
    // Over a tenth of a second: each pulse swings them.
    let n = FS as usize / 10;
    let (mut eta, mut bsr) = (0.0, 0.0);
    for _ in 0..n {
        sim.render(1);
        let t = sim.turbo().unwrap();
        eta += t.turbine_efficiency() / n as f64;
        bsr += t.blade_speed_ratio() / n as f64;
    }
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
    graph.turbos.push(TurboMount { id: "turbo1".into(), node: "t".into(), position: None, rotation: None, settings: None });
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
        graph.turbos.push(TurboMount { id: format!("turbo{n}"), node, position: None, rotation: None, settings: None });
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

/// The RB26's own exhaust, its second turbo set on its own.
fn rb26_with_second_turbo(boost_target: f64, turbo_size: f64) -> engine_sim::exhaust_graph::ExhaustGraph {
    rb26_with_second_turbo_as(|s| TurboSettings { boost_target: boost_target * 1e5, turbo_size, ..s })
}

/// The RB26's own exhaust, its second turbo set on its own: the engine's settings, as `set` changes them.
fn rb26_with_second_turbo_as(set: impl Fn(TurboSettings) -> TurboSettings) -> engine_sim::exhaust_graph::ExhaustGraph {
    let cfg = &common::engine_preset(RB26).config;
    let mut graph = cfg.graph.clone().unwrap();
    graph.turbos[1].settings = Some(set(graph.turbos[1].settings_for(&cfg.engine)));
    graph
}

/// The RB26 on full throttle at 5000 rpm with `graph`, settled: each turbo's mean shaft speed and
/// wastegate opening over a second, and how much of it a compressor was surging.
fn run_turbos(graph: Option<engine_sim::exhaust_graph::ExhaustGraph>) -> (Vec<f64>, Vec<f64>, f64) {
    let mut sim = rb26(json!({ "throttle": 1, "rpm": 5000 }));
    if graph.is_some() {
        sim.set_graph(graph);
    }
    sim.render(3 * FS as usize);
    let n = FS as usize;
    let (mut rpm, mut wastegate, mut surging) = (vec![0.0; 2], vec![0.0; 2], 0.0);
    for _ in 0..n {
        sim.render(1);
        let t = sim.snapshot().turbo.unwrap();
        for (i, u) in t.turbos.iter().enumerate() {
            rpm[i] += u.shaft_rpm / n as f64;
            wastegate[i] += u.wastegate / n as f64;
        }
        if t.surging {
            surging += 1.0 / n as f64;
        }
    }
    (rpm, wastegate, surging)
}

/// Left on the engine's settings, the two turbos turn as one.
#[test]
fn turbos_on_the_engines_settings_turn_as_one() {
    let (rpm, wastegate, surging) = run_turbos(None);
    assert_eq!(rpm[0], rpm[1]);
    assert_eq!(wastegate[0], wastegate[1]);
    assert_eq!(surging, 0.0);
}

/// A bigger turbo set on its own, on the same boost, turns slower than the other, both holding the
/// boost without surging.
#[test]
fn a_bigger_turbo_on_its_own_turns_slower() {
    let (rpm, wastegate, surging) = run_turbos(Some(rb26_with_second_turbo(TARGET, 0.22)));
    assert!(rpm[1] < 0.9 * rpm[0], "{} rpm against {} rpm", rpm[1], rpm[0]);
    assert!((wastegate[0] - wastegate[1]).abs() < 0.05, "wastegates {wastegate:?}");
    assert_eq!(surging, 0.0);
}

/// A turbo set to a higher boost than the other holds its wastegate shut while the other's opens.
#[test]
fn a_turbo_on_a_higher_boost_keeps_its_wastegate_shut() {
    let (_, wastegate, _) = run_turbos(Some(rb26_with_second_turbo(TARGET + 0.1, 0.16)));
    assert!(wastegate[0] > wastegate[1] + 0.2, "wastegates {wastegate:?}");
}

/// Setting a turbo on its own resizes it where it is: the gas in the pipes is left as it was, and so is
/// the turbos' spin.
#[test]
fn setting_a_turbo_keeps_the_exhaust_running() {
    let mut sim = rb26(json!({ "throttle": 1, "rpm": 5000 }));
    sim.render(2 * FS as usize);
    let pressure = sim.pipe_solver().primary(0).pressure_at(2);
    let spin = sim.turbo().unwrap().shaft_rpm_of(1);
    sim.set_graph(Some(rb26_with_second_turbo(1.0, 0.2)));
    assert_eq!(sim.pipe_solver().primary(0).pressure_at(2), pressure);
    assert_eq!(sim.turbo().unwrap().shaft_rpm_of(1), spin);
    assert_eq!(sim.pipe_solver().turbine_mounts()[1].settings.unwrap().boost_target, 1.0e5);
}

/// Each turbo has its own blow-off valve: with the second's taken off, lifting off on boost opens the
/// first's while the second's stays shut.
#[test]
fn each_turbo_has_its_own_blow_off_valve() {
    let mut sim = rb26(json!({ "throttle": 1, "rpm": 4000, "blowOff": "atmospheric" }));
    sim.set_graph(Some(rb26_with_second_turbo_as(|s| TurboSettings { blow_off: BlowOff::None, ..s })));
    sim.render(3 * FS as usize);
    sim.set_controls(0.0, 0.0);
    let mut opened = [0.0f64; 2];
    for _ in 0..FS as usize / 10 {
        sim.render(1);
        let t = sim.snapshot().turbo.unwrap();
        for (i, u) in t.turbos.iter().enumerate() {
            opened[i] = opened[i].max(u.blow_off);
        }
    }
    assert!(opened[0] > 0.9, "the first's valve opens: {}", opened[0]);
    assert_eq!(opened[1], 0.0, "the second has none");
}
