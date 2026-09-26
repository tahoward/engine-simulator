//! The cylinder: crank geometry, motored compression, the Wiebe burn, valve flow, and the gas state
//! staying admissible while the cylinder empties.

mod common;

use common::FS;
use engine_sim::EngineSim;
use engine_sim::cylinder::{Cylinder, SpecInstance, wiebe};
use engine_sim::spec::{CrankGeometry, EngineSpec, clearance_volume, crank_at, cylinder_volume, displacement, gas};
use engine_sim::valve::{orifice_mass_flow, valve_flow_area, valve_lift};
use serde_json::json;
use std::f64::consts::PI;

/// The first cylinder's noise seed, as the engine seeds it.
const SEED: f64 = 0x51f3a7 as f64;

fn default_engine() -> EngineSpec {
    common::presets().default_engine.clone()
}

/// `expect(actual).toBeCloseTo(expected, digits)`: within half a unit in the `digits`th decimal place.
#[track_caller]
fn assert_close(actual: f64, expected: f64, digits: f64) {
    let tol = 10f64.powf(-digits) / 2.0;
    assert!((actual - expected).abs() < tol, "expected {actual} to be within {tol} of {expected}");
}

/// dV/dtheta, m^3 per radian: the sim's own, from the consolidated crank evaluation.
fn d_volume_d_theta(spec: &EngineSpec, deg: f64) -> f64 {
    crank_at(&CrankGeometry::of(spec), deg).d_volume
}

/// Motor the cylinder from where it is to `to_deg` with both valves shut.
fn motor(si: &SpecInstance, cyl: &mut Cylinder, to_deg: f64, rpm: f64) {
    let omega = (rpm * 2.0 * PI) / 60.0;
    let dt = 1.0 / FS;
    // Sub-step finely; this test is about the thermodynamics, not the integrator.
    let sub = 8;
    let mut guard = 0;
    while cyl.angle < to_deg && guard < 4_000_000 {
        guard += 1;
        for _ in 0..sub {
            cyl.advance_with(si, dt / sub as f64, omega, 0.0, 0.0, gas::T_AMB, 900.0, 0.0, None);
        }
    }
}

// --- cylinder geometry ---

/// Volume is clearance at TDC and clearance + displacement at BDC.
#[test]
fn geometry_volume_is_clearance_at_tdc_and_clearance_plus_displacement_at_bdc() {
    let spec = default_engine();
    assert_close(cylinder_volume(&spec, 0.0), clearance_volume(&spec), 10.0);
    assert_close(cylinder_volume(&spec, 180.0), clearance_volume(&spec) + displacement(&spec), 10.0);
}

/// Realises the stated compression ratio.
#[test]
fn geometry_realises_the_stated_compression_ratio() {
    let spec = default_engine();
    let ratio = cylinder_volume(&spec, 180.0) / cylinder_volume(&spec, 0.0);
    assert_close(ratio, spec.compression_ratio, 6.0);
}

/// dV/dtheta matches a numerical derivative.
#[test]
fn geometry_dv_dtheta_matches_a_numerical_derivative() {
    // The energy equation leans on this analytically, so a mismatch would quietly
    // corrupt every pressure the model produces.
    let spec = default_engine();
    let h = 1e-4;
    for deg in [10.0, 45.0, 90.0, 135.0, 200.0, 270.0, 350.0] {
        let numeric = ((cylinder_volume(&spec, deg + h) - cylinder_volume(&spec, deg - h)) / (2.0 * h)) * (180.0 / PI);
        assert_close(d_volume_d_theta(&spec, deg), numeric, 8.0);
    }
}

/// Displacement is about 498 cc for the default bore and stroke.
#[test]
fn geometry_displacement_is_about_498_cc_for_the_default_bore_and_stroke() {
    let spec = default_engine();
    assert!(displacement(&spec) * 1e6 > 490.0);
    assert!(displacement(&spec) * 1e6 < 505.0);
}

// --- motored compression ---

/// Start just *after* intake valve closing (576 deg). Crossing IVC is what arms
/// combustion for the cycle, so beginning at 580 guarantees a genuinely motored
/// compression with no heat release.
const START: f64 = 580.0;

fn at_start(spec: &EngineSpec) -> Cylinder {
    let mut cyl = Cylinder::new(spec, START, SEED);
    cyl.set_temp(500.0); // above the 450 K wall, so heat leaves the gas
    cyl.mass = (gas::P_AMB * cylinder_volume(spec, START)) / (gas::R * cyl.temp());
    cyl
}

/// Conserves mass with the valves shut.
#[test]
fn motored_conserves_mass_with_the_valves_shut() {
    let si = SpecInstance::new(default_engine());
    let mut cyl = at_start(&si.spec);
    let m0 = cyl.mass;
    motor(&si, &mut cyl, 719.0, 3000.0);
    assert_close(cyl.mass, m0, 12.0);
}

/// Peak pressure approaches but stays under the isentropic value.
#[test]
fn motored_peak_pressure_approaches_but_stays_under_the_isentropic_value() {
    let si = SpecInstance::new(default_engine());
    let spec = &si.spec;
    let mut cyl = at_start(spec);
    let p1 = cyl.pressure(spec);
    let v1 = cylinder_volume(spec, START);

    motor(&si, &mut cyl, 719.5, 3000.0);
    let p2 = cyl.pressure(spec);
    let v2 = cylinder_volume(spec, cyl.angle);

    let isentropic = p1 * (v1 / v2).powf(gas::GAMMA_CYL);
    let fraction = p2 / isentropic;
    // Woschni wall heat transfer removes a few percent of the compression work.
    // Anything above 1.0 would mean the energy balance is creating energy.
    assert!(fraction > 0.85, "fraction {fraction}");
    assert!(fraction < 1.0, "fraction {fraction}");
}

/// Behaves as a polytropic process with a realistic exponent.
#[test]
fn motored_behaves_as_a_polytropic_process_with_a_realistic_exponent() {
    let si = SpecInstance::new(default_engine());
    let spec = &si.spec;
    let mut cyl = at_start(spec);
    let p1 = cyl.pressure(spec);
    let v1 = cylinder_volume(spec, START);

    motor(&si, &mut cyl, 719.5, 3000.0);
    let p2 = cyl.pressure(spec);
    let v2 = cylinder_volume(spec, cyl.angle);

    let n = (p2 / p1).ln() / (v1 / v2).ln();
    // Real engines measure 1.30-1.35 on the compression line.
    assert!(n > 1.28, "n {n}");
    assert!(n < 1.36, "n {n}");
}

/// Does not arm combustion when the intake valve closing is never crossed.
#[test]
fn motored_does_not_arm_combustion_when_the_intake_valve_closing_is_never_crossed() {
    let si = SpecInstance::new(default_engine());
    let mut cyl = at_start(&si.spec);
    motor(&si, &mut cyl, 719.5, 3000.0);
    assert_eq!(cyl.burned, 0.0);
}

// --- wiebe combustion ---

/// Is monotonic, starts at zero and finishes burnt.
#[test]
fn wiebe_is_monotonic_starts_at_zero_and_finishes_burnt() {
    assert_eq!(wiebe(-5.0, 50.0), 0.0);
    assert_eq!(wiebe(0.0, 50.0), 0.0);
    assert_eq!(wiebe(50.0, 50.0), 1.0);
    assert_eq!(wiebe(200.0, 50.0), 1.0);

    let mut prev = 0.0;
    let mut d = 0.0;
    while d <= 50.0 {
        let x = wiebe(d, 50.0);
        assert!(x >= prev, "wiebe({d}) = {x} < {prev}");
        prev = x;
        d += 0.5;
    }
}

/// Releases most of the heat in the middle of the window.
#[test]
fn wiebe_releases_most_of_the_heat_in_the_middle_of_the_window() {
    // a=5, m=2 gives the familiar S-curve rather than a linear ramp.
    assert!(wiebe(25.0, 50.0) > 0.3);
    assert!(wiebe(25.0, 50.0) < 0.7);
}

// --- valve flow ---

/// Lift is zero outside the window and peaks inside it.
#[test]
fn valve_lift_is_zero_outside_the_window_and_peaks_inside_it() {
    let spec = default_engine();
    // The exhaust window wraps past 720, which is the case most likely to be broken.
    assert_eq!(valve_lift(100.0, spec.evo, spec.evc, spec.max_lift), 0.0);
    assert_eq!(valve_lift(spec.evo - 1.0, spec.evo, spec.evc, spec.max_lift), 0.0);

    let mid = (spec.evo + spec.evc) / 2.0;
    assert_close(valve_lift(mid, spec.evo, spec.evc, spec.max_lift), spec.max_lift, 6.0);

    // Overlap: the exhaust valve is still open a crack just after TDC.
    assert!(valve_lift(370.0, spec.evo, spec.evc, spec.max_lift) > 0.0);
    assert_eq!(valve_lift(380.0, spec.evo, spec.evc, spec.max_lift), 0.0);
}

/// Lift ramps smoothly, with no step at the seat.
#[test]
fn valve_lift_ramps_smoothly_with_no_step_at_the_seat() {
    // A discontinuity here would inject a broadband click every cycle.
    let spec = default_engine();
    let mut prev = 0.0;
    let mut max_jump: f64 = 0.0;
    let mut d = spec.evo - 2.0;
    while d < spec.evc + 2.0 {
        let l = valve_lift(d, spec.evo, spec.evc, spec.max_lift);
        max_jump = max_jump.max((l - prev).abs());
        prev = l;
        d += 0.1;
    }
    assert!(max_jump < spec.max_lift * 0.02, "max jump {max_jump}");
}

/// Flow area saturates at the port throat.
#[test]
fn valve_flow_area_saturates_at_the_port_throat() {
    let dia = 0.034;
    let small = valve_flow_area(0.001, dia);
    let large = valve_flow_area(0.05, dia);
    // Curtain area at 1 mm lift, the area the discharge coefficient is referred to.
    assert_close(small, PI * dia * 0.001, 9.0);
    // Way past the crossover, the throat rules and more lift buys nothing.
    assert_close(large, valve_flow_area(0.1, dia), 12.0);
}

/// Orifice flow chokes and then stops responding to downstream pressure.
#[test]
fn valve_orifice_flow_chokes_and_then_stops_responding_to_downstream_pressure() {
    let area = 5e-4;
    let p_up = 5e5;
    let t = 1200.0;
    let g = gas::GAMMA_CYL;
    let at_critical = orifice_mass_flow(area, 0.72, p_up, t, p_up * 0.53, g);
    let well_below = orifice_mass_flow(area, 0.72, p_up, t, 1e3, g);
    assert_close(well_below, at_critical, 6.0);

    // Unchoked: less pressure drop means less flow.
    let mild = orifice_mass_flow(area, 0.72, p_up, t, p_up * 0.9, g);
    assert!(mild < at_critical);
    assert!(mild > 0.0);

    // No flow uphill or through a shut valve.
    assert_eq!(orifice_mass_flow(area, 0.72, p_up, t, p_up * 1.1, g), 0.0);
    assert_eq!(orifice_mass_flow(0.0, 0.72, p_up, t, 1e3, g), 0.0);
}

/// Peak blowdown empties the cylinder on a plausible timescale.
#[test]
fn valve_peak_blowdown_empties_the_cylinder_on_a_plausible_timescale() {
    // Sanity check on absolute magnitude: at exhaust-valve-opening conditions the
    // charge should dump in a couple of milliseconds, not microseconds or seconds.
    let spec = default_engine();
    let area = valve_flow_area(spec.max_lift * 0.3, spec.ex_valve_dia);
    let mdot = orifice_mass_flow(area, 0.72, 5e5, 1200.0, gas::P_AMB, gas::GAMMA_CYL);
    let trapped = (5e5 * cylinder_volume(&spec, 130.0)) / (gas::R * 1200.0);
    let emptying_time = trapped / mdot;
    assert!(emptying_time > 5e-4, "emptying time {emptying_time}");
    assert!(emptying_time < 2e-2, "emptying time {emptying_time}");
}

// --- the gas state stays admissible while the cylinder empties ---
//
// The exhaust stroke is the hard case for a filling-and-emptying model.
//
// Compression work `-p dV/dt` and outflow enthalpy nearly cancel while gas is being pushed
// out — exactly, at constant pressure, which is why temperature should hold steady. With
// *temperature* as the state variable that near-zero residual gets divided by a mass
// shrinking toward the residual, and it blows up: at part throttle the temperature runs to
// its clamp right at exhaust valve closing, with the cylinder nearly drained. Integrating
// internal energy instead keeps the cancellation between terms of the same size.

/// Emptying the cylinder converges under time refinement.
#[test]
fn emptying_the_cylinder_converges_under_time_refinement() {
    // The integrator tested directly, rather than against an analytic answer — wall heat
    // transfer is always active and legitimately changes the temperature, so a closed-form
    // target would be measuring the physics, not the scheme.
    //
    // Drive the exact quasi-steady constant-pressure outflow (from pV = mRT at fixed p and
    // T, mass tracks p V/(R T), so mdot = -(p/(R T)) dV/dt) and compare coarse against fine
    // steps. A sound integrator gives nearly the same answer; one dividing a near-cancelling
    // residual by a vanishing mass diverges as the cylinder empties.
    let si = SpecInstance::new(default_engine());
    let spec = &si.spec;
    let omega = (3200.0 * 2.0 * PI) / 60.0;

    struct Emptied {
        temp: f64,
        mass_ratio: f64,
        clamps: u64,
    }
    let empty_out = |subdiv: f64| {
        let mut cyl = Cylinder::new(spec, 250.0, SEED); // mid exhaust stroke
        cyl.set_temp(1100.0);
        cyl.mass = (gas::P_AMB * cylinder_volume(spec, 250.0)) / (gas::R * 1100.0);
        let m0 = cyl.mass;
        let dt = 1.0 / (FS * subdiv);
        while cyl.angle < 352.0 {
            let dvdt = d_volume_d_theta(spec, cyl.angle) * omega;
            let ex = -(gas::P_AMB / (gas::R * cyl.temp())) * dvdt;
            cyl.advance_with(&si, dt, omega, ex, 0.0, gas::T_AMB, 900.0, 0.0, None);
        }
        Emptied { temp: cyl.temp(), mass_ratio: cyl.mass / m0, clamps: cyl.clamp_hits }
    };

    let coarse = empty_out(1.0);
    let fine = empty_out(16.0);

    // It really did empty substantially — about half the charge leaves over this stroke,
    // which is the regime a temperature-state integrator breaks in. (Not more: dV/dtheta
    // tapers toward TDC, so the constant-pressure outflow tapers with it.)
    assert!(fine.mass_ratio < 0.6, "mass ratio {}", fine.mass_ratio);
    assert!((coarse.temp - fine.temp).abs() / fine.temp < 0.05, "coarse {} fine {}", coarse.temp, fine.temp);
    assert_eq!(coarse.clamps, 0);
    assert_eq!(fine.clamps, 0);
    assert!(coarse.temp.is_finite());
}

/// Never clamps at any steady operating point.
#[test]
fn emptying_never_clamps_at_any_steady_operating_point() {
    // The clamp truncates energy, so a hit means the integration has left the physics
    // behind. A fault of that kind fires hundreds of times a second below about 0.5 throttle
    // while staying silent at full load, which is why the sweep goes down to 0.1.
    for throttle in [1.0, 0.75, 0.45, 0.3, 0.2, 0.1] {
        let mut cfg = common::default_config();
        cfg.engine = EngineSpec { throttle, rpm: 3200.0, ..cfg.engine };
        cfg.pipe = common::pipe_preset(1);
        let mut sim = EngineSim::new(FS, &cfg);
        sim.render(FS as usize * 2);
        let before = sim.cylinder().clamp_hits;
        sim.render(FS as usize);
        assert_eq!(sim.cylinder().clamp_hits - before, 0, "throttle {throttle}");
    }
}

/// Reverse flow through the exhaust valve arrives at port temperature.
#[test]
fn emptying_reverse_flow_through_the_exhaust_valve_arrives_at_port_temperature() {
    // During overlap the pipe can push gas back into the cylinder. Treating it as arriving
    // at *cylinder* temperature imports heat that was never there.
    let si = SpecInstance::new(default_engine());
    let spec = &si.spec;
    let mut hot = Cylinder::new(spec, 400.0, SEED);
    let mut cold = Cylinder::new(spec, 400.0, SEED);
    for c in [&mut hot, &mut cold] {
        c.set_temp(1200.0);
        c.mass = (gas::P_AMB * cylinder_volume(spec, 400.0)) / (gas::R * 1200.0);
    }
    let omega = (3200.0 * 2.0 * PI) / 60.0;
    // Same reverse flow, different port temperatures.
    for _ in 0..400 {
        hot.advance_with(&si, 1.0 / FS / 8.0, omega, -0.01, 0.0, gas::T_AMB, 1400.0, 0.0, None);
        cold.advance_with(&si, 1.0 / FS / 8.0, omega, -0.01, 0.0, gas::T_AMB, 500.0, 0.0, None);
    }
    assert!(hot.temp() > cold.temp() + 20.0, "hot {} cold {}", hot.temp(), cold.temp());
}

/// Free-running wind-down from full throttle to shut stays clean.
#[test]
fn emptying_free_running_wind_down_from_full_throttle_to_shut_stays_clean() {
    let mut cfg = common::default_config();
    cfg.engine = EngineSpec { free_running: true, throttle: 1.0, load: 0.32, ..cfg.engine };
    cfg.pipe = common::pipe_preset(1);
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(FS as usize * 4);
    sim.set_engine_json(&json!({ "throttle": 0 })).unwrap();

    let hits0 = sim.cylinder().clamp_hits;
    let buf = sim.render(FS as usize * 4);
    for &v in &buf {
        assert!(v.is_finite());
        assert!(v.abs() < 1.0, "sample {v}");
    }
    assert_eq!(sim.cylinder().clamp_hits - hits0, 0);
    assert_eq!(sim.pipe_solver().recoveries(), 0);
    // And it should actually have slowed down.
    assert!(sim.rpm() < 7000.0, "rpm {}", sim.rpm());
}

// --- consolidated crank state ---
//
// `crank_at` computes in one pass what the readable per-quantity forms below compute in four, saving
// five sin/cos/sqrt triples per substep. That makes it a second copy of the same algebra, so it has to
// be pinned to the first copy — otherwise a correction to one of them silently applies to one and
// not the other.

/// d(piston position)/d(crank angle), m per radian, including the rod-obliquity term.
fn d_piston_d_theta(spec: &EngineSpec, deg: f64) -> f64 {
    let a = spec.stroke / 2.0;
    let l = spec.rod_length;
    let th = (deg * PI) / 180.0;
    let (sin, cos) = th.sin_cos();
    let r = (l * l - a * a * sin * sin).max(1e-12).sqrt();
    -a * sin - (a * a * sin * cos) / r
}

/// d2(piston position)/d(crank angle)^2, m per radian^2.
fn d2_piston_d_theta2(spec: &EngineSpec, deg: f64) -> f64 {
    let a = spec.stroke / 2.0;
    let l = spec.rod_length;
    let th = (deg * PI) / 180.0;
    let (sin, cos) = th.sin_cos();
    let r = (l * l - a * a * sin * sin).max(1e-12).sqrt();
    let cos2 = cos * cos - sin * sin;
    -a * cos - a * a * (cos2 / r + (a * a * sin * sin * cos * cos) / (r * r * r))
}

/// Distance from crank centre to the piston pin, m.
fn piston_position(spec: &EngineSpec, deg: f64) -> f64 {
    let a = spec.stroke / 2.0;
    let l = spec.rod_length;
    let th = (deg * PI) / 180.0;
    let sin = th.sin();
    a * th.cos() + (l * l - a * a * sin * sin).max(0.0).sqrt()
}

/// dV/dtheta, m^3 per radian.
fn d_volume_readable(spec: &EngineSpec, deg: f64) -> f64 {
    let a = spec.stroke / 2.0;
    let l = spec.rod_length;
    let area = (PI * spec.bore * spec.bore) / 4.0;
    let th = (deg * PI) / 180.0;
    let (sin, cos) = th.sin_cos();
    let root = (l * l - a * a * sin * sin).max(1e-12).sqrt();
    // d/dth of -(a*cos + root) == a*sin + a^2*sin*cos/root
    area * (a * sin + (a * a * sin * cos) / root)
}

/// Agrees with the individual functions across the whole cycle.
#[test]
fn crank_state_agrees_with_the_individual_functions_across_the_whole_cycle() {
    let spec = default_engine();
    let g = CrankGeometry::of(&spec);
    let mut deg = 0.0;
    while deg < 720.0 {
        let out = crank_at(&g, deg);
        assert_close(out.position, piston_position(&spec, deg), 12.0);
        assert_close(out.position, engine_sim::spec::piston_position(&g, deg), 12.0);
        assert_close(out.d_position, d_piston_d_theta(&spec, deg), 12.0);
        assert_close(out.d2_position, d2_piston_d_theta2(&spec, deg), 10.0);
        assert_close(out.volume, cylinder_volume(&spec, deg), 12.0);
        assert_close(out.d_volume, d_volume_readable(&spec, deg), 12.0);
        deg += 0.5;
    }
}

/// Agrees for extreme geometry too.
#[test]
fn crank_state_agrees_for_extreme_geometry_too() {
    // A very short rod exaggerates the obliquity terms, where a sign slip would hide at
    // ordinary proportions.
    for spec in [
        EngineSpec { stroke: 0.12, rod_length: 0.125, ..default_engine() },
        EngineSpec { stroke: 0.04, rod_length: 0.3, ..default_engine() },
    ] {
        let g = CrankGeometry::of(&spec);
        let mut deg = 0.0;
        while deg < 720.0 {
            let out = crank_at(&g, deg);
            assert_close(out.position, piston_position(&spec, deg), 12.0);
            assert_close(out.d_position, d_piston_d_theta(&spec, deg), 12.0);
            assert_close(out.d2_position, d2_piston_d_theta2(&spec, deg), 9.0);
            assert_close(out.d_volume, d_volume_readable(&spec, deg), 12.0);
            deg += 3.0;
        }
    }
}
