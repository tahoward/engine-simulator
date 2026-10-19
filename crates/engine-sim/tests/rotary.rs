//! The Wankel: a rotor's three chambers, each a cylinder on a crank turning at two thirds of the eccentric
//! shaft's speed, breathing through ports its apexes open and shut, two chambers at once while an apex
//! crosses one; and the R26B, which has to come out at about the real engine's torque and power, idle, and
//! sound like a four-rotor, firing every 90 degrees of its shaft.

use crate::common;

use common::{FS, band_energy, hann, magnitude_spectrum};
use engine_sim::EngineSim;
use engine_sim::euler_pipe::{EulerPipe, EulerPipeOptions, SecondChamber, ValveState};
use engine_sim::spec::{
    EngineSpec, EngineType, PipeSegment, ROTARY_PHASE, chamber_offset, chamber_spec, clearance_volume, crank_at,
    CrankGeometry, cylinder_volume, displacement, firing_plan, full_load_torque, valid_layout,
};
use engine_sim::valve::{ROTOR_FACE_LAG, port_area, port_ramp};
use serde_json::json;

const R26B: &str = "Four rotor, Mazda R26B";

/// A rotary of `rotors`, Mazda's 15 x 105 x 80 mm, at 9.7:1.
fn rotary(rotors: u32) -> EngineSpec {
    EngineSpec {
        engine_type: EngineType::Rotary,
        cylinders: rotors,
        v_angle: 0.0,
        compression_ratio: 9.7,
        ..EngineSpec::default()
    }
}

#[test]
fn a_chamber_displaces_three_root_three_e_r_b_and_its_volume_is_sinusoidal_over_three_turns() {
    let spec = rotary(2);
    // Mazda's 654 cc a chamber.
    assert!((displacement(&spec) * 1e6 - 654.6).abs() < 0.5, "{}", displacement(&spec) * 1e6);
    let chambers = chamber_spec(&spec);
    let g = CrankGeometry::of(&chambers);
    let vc = clearance_volume(&spec);
    let vd = displacement(&spec);
    for k in 0..1080 {
        let shaft = k as f64;
        let phase = shaft * ROTARY_PHASE;
        // Least at each top dead centre, 540 degrees of the shaft apart, most at each bottom dead centre.
        let want = vc + 0.5 * vd * (1.0 - (2.0 * std::f64::consts::PI * shaft / 540.0).cos());
        let state = crank_at(&g, phase);
        assert!((state.volume - want).abs() < 1e-12, "{shaft}: {} against {want}", state.volume);
        assert!((cylinder_volume(&chambers, phase) - want).abs() < 1e-12);
        // dV/dphase the slope of what it gives.
        let h = 1e-4;
        let slope = (crank_at(&g, phase + h).volume - crank_at(&g, phase - h).volume) / (2.0 * h.to_radians());
        assert!((state.d_volume - slope).abs() < 1e-9 * vd, "{shaft}");
    }
}

#[test]
fn a_rotarys_angles_are_its_shafts_and_its_chambers_run_at_two_thirds_of_them() {
    let spec = EngineSpec { evo: 195.0, evc: 588.0, ivo: 572.0, ivc: 850.0, ignition: 1065.0, burn_duration: 60.0, ..rotary(2) };
    let c = chamber_spec(&spec);
    for (shaft, chamber) in [(spec.evo, c.evo), (spec.evc, c.evc), (spec.ivo, c.ivo), (spec.ivc, c.ivc), (spec.ignition, c.ignition)] {
        assert!((chamber - shaft * 2.0 / 3.0).abs() < 1e-12);
    }
    assert!((c.burn_duration - 40.0).abs() < 1e-12);
    // A piston engine's is its own.
    assert_eq!(chamber_spec(&EngineSpec::default()), EngineSpec::default());
}

#[test]
fn the_rotors_share_each_turn_of_the_shaft_evenly() {
    for rotors in 1..=4 {
        let spec = rotary(rotors);
        assert!(valid_layout(&spec));
        let plan = firing_plan(&spec);
        assert_eq!(plan.offsets.len(), rotors as usize);
        assert_eq!(plan.bank_count, 1);
        // Every chamber of every rotor, by when it fires over the 1080-degree cycle: evenly spaced.
        let spec = &spec;
        let mut fires: Vec<f64> =
            (0..rotors as usize).flat_map(|r| (0..3).map(move |k| chamber_offset(spec, r, k))).collect();
        fires.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let gap = 360.0 / rotors as f64;
        for (i, f) in fires.iter().enumerate() {
            assert!((f - i as f64 * gap).abs() < 1e-9, "{rotors} rotors: {fires:?}");
        }
    }
    assert!(!valid_layout(&rotary(5)));
    // Two rotors sweep as much every two turns as a four-stroke of twice their displacement does.
    let two = rotary(2);
    let bore = EngineSpec::default().bore;
    let stroke = displacement(&two) / (std::f64::consts::PI * bore * bore / 4.0);
    let piston = EngineSpec { cylinders: 4, v_angle: 0.0, stroke, ..EngineSpec::default() };
    assert!((full_load_torque(&two, false) - full_load_torque(&piston, false)).abs() < 1e-9);
}

#[test]
fn an_apex_crossing_a_port_hands_it_from_one_chamber_to_the_next() {
    // The R26B's exhaust, open 400 degrees of the shaft: 40 of them to two chambers at once.
    let (open, close) = (185.0 * ROTARY_PHASE, 585.0 * ROTARY_PHASE);
    let ramp = port_ramp(open, close);
    assert!((ramp - (close - open - ROTOR_FACE_LAG)).abs() < 1e-9);
    let full = 1e-3;
    for k in 0..720 {
        let deg = k as f64;
        let leading = port_area(deg, open, close, ramp, full);
        let next = port_area(deg - ROTOR_FACE_LAG, open, close, ramp, full);
        assert!(leading <= full && next <= full);
        if leading > 0.0 && next > 0.0 {
            // An apex across the port: between them, the whole of it.
            assert!((leading + next - full).abs() < 1e-12 * full, "{deg}: {leading} + {next}");
        }
    }
    // A port shorter than a turn of the shaft is only ever open to one, and opens over a quarter of it.
    let side = port_ramp(572.0 * ROTARY_PHASE, 850.0 * ROTARY_PHASE);
    assert!((side - 0.25 * (850.0 - 572.0) * ROTARY_PHASE).abs() < 1e-9);
}

/// A port open to two chambers: a chamber blowing down at 4 bar and one at 0.9 bar the port's air flows back
/// into, each through half of 10 cm^2.
#[test]
fn a_port_open_to_two_chambers_takes_each_ones_own_flow() {
    let seg: PipeSegment = common::pipe(0.5, 0.05);
    let opts = EulerPipeOptions { heat_transfer: Some(false), ..Default::default() };
    let mut duct = EulerPipe::new(&[seg], FS, 300.0, &opts);
    let area = 5e-4;
    let valve = ValveState {
        throat_area: area,
        cyl_pressure: 0.9e5,
        cyl_temp: 700.0,
        cyl_gamma: 1.3,
        extra_mass_flow: 0.0,
        second: SecondChamber { throat_area: area, pressure: 4e5, temp: 1200.0, gamma: 1.3 },
    };
    let before = duct.total_mass();
    let dt = 1.0 / FS;
    let (mut into, mut out_of_first, mut out_of_second) = (0.0, 0.0, 0.0);
    for _ in 0..24 {
        duct.begin_step(dt);
        duct.apply_own_boundaries(dt);
        let flow = duct.valve_flux_for(&valve, dt);
        let second = duct.valve_flux_2;
        duct.end_step(dt, flow, &valve);
        into += duct.valve_source_flow() * dt;
        out_of_first += (flow - second) * duct.source_scale * dt;
        out_of_second += second * duct.source_scale * dt;
        into -= duct.face_mass_flow(duct.n) * dt;
    }
    // The blowdown flows out, and the port's gas back into the chamber at less than its pressure.
    assert!(out_of_second > 0.0, "{out_of_second}");
    assert!(out_of_first < 0.0, "{out_of_first}");
    // What the duct gained is what the two chambers gave it, less what left by its mouth.
    let gained = duct.total_mass() - before;
    assert!((gained - into).abs() < 1e-6 * out_of_second.abs(), "{gained} against {into}");
}

/// The R26B makes about the real engine's rated 608 N*m at 6500 rpm and 700 PS (515 kW) at 9000.
#[test]
fn the_r26b_makes_about_the_real_engines_torque_and_power() {
    let [t6500, t9000] = common::brake_torques_at(R26B, 2.0, [6500.0, 9000.0]);
    assert!((t6500 - 608.0).abs() < 0.1 * 608.0, "{t6500} N*m at 6500 rpm");
    let ps = common::ps(t9000, 9000.0);
    assert!((ps - 700.0).abs() < 0.1 * 700.0, "{ps} PS at 9000 rpm");
}

/// Run free from its preset, it settles near its idle speed: high, for the exhaust its ports' overlap lets into
/// its intake.
#[test]
fn the_r26b_idles() {
    let mut cfg = common::engine_preset(R26B).config.clone();
    cfg.engine = common::with(&cfg.engine, json!({ "freeRunning": true }));
    let idle = cfg.engine.idle_rpm;
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(FS as usize * 3);
    let mut sum = 0.0;
    for _ in 0..20 {
        sim.render(FS as usize / 10);
        sum += sim.rpm();
    }
    let mean = sum / 20.0;
    assert!(mean > idle - 150.0 && mean < idle + 200.0, "mean {mean} against {idle}");
}

/// Held at 6500 rpm on a part throttle, its firing is the strongest of its low orders: four rotors each firing
/// once a turn, four times the shaft's speed, against the turn's own order, its half and its others below the
/// firing. Each chamber of a rotor fires every third turn, and alike, so the rotors' turns sound alike too.
#[test]
fn the_r26b_sounds_its_firing_four_times_a_turn() {
    let mut cfg = common::engine_preset(R26B).config.clone();
    cfg.engine = common::with(&cfg.engine, json!({ "freeRunning": false, "rpm": 6500, "throttle": 0.5, "combustionVariability": 0 }));
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(FS as usize);
    let size = 1 << 15;
    let mag = magnitude_spectrum(&hann(&sim.render(size)), size);
    let turn = 6500.0 / 60.0;
    // Each order's band 3% wide: the crank's held speed wanders a little round the speed it is held at.
    let at = |order: f64| band_energy(&mag, FS, size, order * turn, (0.03 * order * turn).max(6.0));
    let firing = at(4.0);
    for order in [1.0 / 3.0, 0.5, 2.0 / 3.0, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5] {
        assert!(firing > 10.0 * at(order), "order {order:.2}: {} against the firing's {firing}", at(order));
    }
    // Nothing went wrong in the pipes.
    let broken: u64 = sim.pipe_solver().ducts.iter().map(|d| d.recoveries).sum();
    assert_eq!(broken, 0);
    assert!(sim.cylinders().iter().all(|c| c.clamp_hits == 0));
}
