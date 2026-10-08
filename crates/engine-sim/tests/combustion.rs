//! The flame: how long each charge takes to burn, and what the mixture it burns is.
//!
//! The burn duration is predicted from the flame speed of the charge the spark finds, so it has to
//! move the way a real engine's does: a little longer as the engine speeds up, a lot longer at part
//! throttle and lean. The mixture is carried as fuel and air through the manifold and the cylinder,
//! so λ sets how much heat a charge can release, and cutting the fuel leaves nothing to burn.

use crate::common;

use common::{FS, assert_close, default_engine};
use engine_sim::EngineSim;
use engine_sim::cylinder::{burn_angle, laminar_flame_speed, laminar_speed_base};
use engine_sim::spec::EngineSpec;
use serde_json::{Value, json};
use std::f64::consts::PI;

/// Crank speed, rad/s, at which `spec` has mean piston speed `sp` (m/s).
fn omega_at(spec: &EngineSpec, sp: f64) -> f64 {
    (PI * sp) / spec.stroke
}

/// The default single at a fixed speed with no cycle-to-cycle scatter, and `over` on top.
fn single(over: Value) -> EngineSim {
    let mut cfg = common::default_config();
    cfg.engine = common::with(&cfg.engine, json!({ "freeRunning": false, "combustionVariability": 0 }));
    cfg.engine = common::with(&cfg.engine, over);
    cfg.pipe = common::pipe_preset(1);
    EngineSim::new(FS, &cfg)
}

struct Running {
    /// Mean burn duration of the cycles that fired, deg.
    burn: f64,
    /// Fraction of cycles that released heat.
    fired: f64,
    /// Mean gas torque, N*m.
    torque: f64,
}

/// Settle for a second, then watch cylinder 1 for `seconds`.
fn watch(sim: &mut EngineSim, seconds: f64) -> Running {
    sim.render(FS as usize);
    let mut prev = sim.cylinders()[0].angle;
    let mut cycles = 0u32;
    let mut fired = 0u32;
    let mut burn = 0.0;
    let mut torque = 0.0;
    let n = (FS * seconds).round() as usize;
    for _ in 0..n {
        sim.render(1);
        torque += sim.snapshot().torque;
        let cyl = &sim.cylinders()[0];
        let a = cyl.angle;
        // By 90 ATDC the cycle's burn duration is latched and most of its heat released.
        let crossed = if a >= prev { prev < 90.0 && a >= 90.0 } else { prev < 90.0 || a >= 90.0 };
        if crossed {
            cycles += 1;
            if cyl.burned > 0.05 {
                fired += 1;
                burn += cyl.burn_angle;
            }
        }
        prev = a;
    }
    Running {
        burn: if fired > 0 { burn / fired as f64 } else { 0.0 },
        fired: fired as f64 / cycles.max(1) as f64,
        torque: torque / n as f64,
    }
}

// --- laminar flame speed ---

/// Matches the correlation at room conditions and peaks slightly rich.
#[test]
fn laminar_flame_speed_matches_the_correlation_at_room_conditions_and_peaks_slightly_rich() {
    // B_m + B_φ (φ - φ_m)^2 at φ = 1: 0.305 - 0.549 * 0.21^2.
    assert_close(laminar_flame_speed(1.0, 298.0, 101325.0, 0.0), 0.2808, 4.0);
    assert!(laminar_speed_base(1.21) > laminar_speed_base(1.0));
    assert!(laminar_speed_base(1.21) > laminar_speed_base(1.4));
}

/// Rises steeply with temperature and falls gently with pressure.
#[test]
fn laminar_flame_speed_rises_steeply_with_temperature_and_falls_gently_with_pressure() {
    let cold = laminar_flame_speed(1.0, 400.0, 10e5, 0.0);
    let hot = laminar_flame_speed(1.0, 800.0, 10e5, 0.0);
    // T^2.13: doubling the temperature puts it up more than fourfold.
    assert!(hot / cold > 4.0);
    let low = laminar_flame_speed(1.0, 650.0, 2e5, 0.0);
    let high = laminar_flame_speed(1.0, 650.0, 20e5, 0.0);
    assert!(high < low);
    assert!(high / low > 0.5);
}

/// Slows with residual gas, and is gone outside the flammability limits.
#[test]
fn laminar_flame_speed_slows_with_residual_gas_and_is_gone_outside_the_flammability_limits() {
    let clean = laminar_flame_speed(1.0, 650.0, 13e5, 0.0);
    assert!(laminar_flame_speed(1.0, 650.0, 13e5, 0.2) < 0.5 * clean);
    assert_eq!(laminar_flame_speed(0.4, 650.0, 13e5, 0.0), 0.0);
    assert_eq!(laminar_flame_speed(2.1, 650.0, 13e5, 0.0), 0.0);
    assert_eq!(laminar_flame_speed(0.0, 650.0, 13e5, 0.0), 0.0);
}

// --- burn duration ---

/// Is the stated duration at the reference flame state.
#[test]
fn burn_duration_is_the_stated_duration_at_the_reference_flame_state() {
    let spec = default_engine();
    assert_close(burn_angle(&spec, omega_at(&spec, 10.0), 13e5, 650.0, 1.0, 0.04), spec.burn_duration, 9.0);
}

/// Lengthens with rpm, but much less than in proportion.
#[test]
fn burn_duration_lengthens_with_rpm_but_much_less_than_in_proportion() {
    let spec = default_engine();
    let at = |rpm: f64| burn_angle(&spec, (rpm * 2.0 * PI) / 60.0, 13e5, 650.0, 1.0, 0.04);
    let ratio = at(6000.0) / at(1000.0);
    // A burn taking a fixed time would be six times as many degrees.
    assert!(ratio > 1.3, "ratio {ratio}");
    assert!(ratio < 2.2, "ratio {ratio}");
}

/// Comes out longer at part throttle, lean, and with residual, in the running engine.
#[test]
fn burn_duration_comes_out_longer_at_part_throttle_lean_and_with_residual_in_the_running_engine() {
    let wot = watch(&mut single(json!({ "throttle": 1, "rpm": 3200 })), 1.0);
    let part = watch(&mut single(json!({ "throttle": 0.2, "rpm": 3200 })), 1.0);
    let lean = watch(&mut single(json!({ "throttle": 1, "rpm": 3200, "lambda": 1.3 })), 1.0);
    let slow = watch(&mut single(json!({ "throttle": 1, "rpm": 1000 })), 2.0);
    let fast = watch(&mut single(json!({ "throttle": 1, "rpm": 6000 })), 1.0);

    let nominal = default_engine().burn_duration;
    // The default engine at full throttle is near its reference state.
    assert!(wot.burn > 0.85 * nominal, "wot {}", wot.burn);
    assert!(wot.burn < 1.1 * nominal, "wot {}", wot.burn);
    assert!(part.burn > 1.15 * wot.burn, "part {} wot {}", part.burn, wot.burn);
    assert!(lean.burn > 1.1 * wot.burn, "lean {} wot {}", lean.burn, wot.burn);
    assert!(fast.burn > 1.3 * slow.burn, "fast {} slow {}", fast.burn, slow.burn);
}

// --- advance map ---

/// Mean spark timing of cylinder 1 over a second, deg BTDC.
fn advance(over: Value) -> f64 {
    let mut sim = single(over);
    sim.render(FS as usize / 2);
    let mut sum = 0.0;
    for _ in 0..20 {
        sim.render(FS as usize / 20);
        sum += 720.0 - sim.cylinders()[0].spark;
    }
    sum / 20.0
}

/// Retards where the burn is quick and advances where it is slow.
#[test]
fn advance_map_retards_where_the_burn_is_quick_and_advances_where_it_is_slow() {
    let nominal = 720.0 - default_engine().ignition;
    let low = advance(json!({ "throttle": 1, "rpm": 1000 }));
    let high = advance(json!({ "throttle": 1, "rpm": 6000 }));
    let part = advance(json!({ "throttle": 0.2, "rpm": 3200 }));
    assert!(low < nominal - 3.0, "low {low}");
    assert!(high > nominal + 2.0, "high {high}");
    assert!(part > nominal + 2.0, "part {part}");
}

/// Holds the spark where it is set with the map off.
#[test]
fn advance_map_holds_the_spark_where_it_is_set_with_the_map_off() {
    let nominal = 720.0 - default_engine().ignition;
    assert_close(advance(json!({ "throttle": 1, "rpm": 1000, "advanceCurve": false })), nominal, 9.0);
}

// --- mixture ---

/// Lean releases less heat per charge; rich has no more oxygen to release it with.
#[test]
fn mixture_lean_releases_less_heat_per_charge_rich_has_no_more_oxygen_to_release_it_with() {
    let stoich = watch(&mut single(json!({ "throttle": 1, "rpm": 3200 })), 1.0);
    let lean = watch(&mut single(json!({ "throttle": 1, "rpm": 3200, "lambda": 1.3 })), 1.0);
    let rich = watch(&mut single(json!({ "throttle": 1, "rpm": 3200, "lambda": 0.8 })), 1.0);
    assert!(lean.torque < 0.85 * stoich.torque, "lean {} stoich {}", lean.torque, stoich.torque);
    // Oxygen-limited: within a few percent of stoichiometric, and not above it.
    assert!(rich.torque < stoich.torque, "rich {} stoich {}", rich.torque, stoich.torque);
    assert!(rich.torque > 0.95 * stoich.torque, "rich {} stoich {}", rich.torque, stoich.torque);
}

/// Misfires once the excess air dilutes the charge past the limit.
#[test]
fn mixture_misfires_once_the_excess_air_dilutes_the_charge_past_the_limit() {
    assert_eq!(watch(&mut single(json!({ "throttle": 1, "rpm": 3200, "lambda": 1.4 })), 1.0).fired, 1.0);
    let fired = watch(&mut single(json!({ "throttle": 1, "rpm": 3200, "lambda": 2 })), 2.0).fired;
    assert!(fired < 0.9, "fired {fired}");
}

// --- overrun fuel cut ---

/// Leaves nothing to burn with the throttle shut above the cut speed.
#[test]
fn fuel_cut_leaves_nothing_to_burn_with_the_throttle_shut_above_the_cut_speed() {
    let mut sim = single(json!({ "throttle": 0, "rpm": 3200 }));
    let cut = watch(&mut sim, 1.0);
    assert_eq!(cut.fired, 0.0);
    assert!(sim.snapshot().fuel_cut);
    // The injectors are off, so neither the manifold nor the runners hold any fuel worth the name:
    // under 1% of a stoichiometric charge's.
    assert!(sim.plenum().fuel_fraction() < 6e-4, "plenum {}", sim.plenum().fuel_fraction());
    assert!(sim.intake().runners[0].fuel < 6e-4, "runner {}", sim.intake().runners[0].fuel);
}

/// Keeps firing weakly on the throttle leak when it is off.
#[test]
fn fuel_cut_keeps_firing_weakly_on_the_throttle_leak_when_it_is_off() {
    let run = watch(&mut single(json!({ "throttle": 0, "rpm": 3200, "fuelCut": false })), 1.0);
    assert!(run.fired > 0.1, "fired {}", run.fired);
}

/// Is not active below the resume speed or with the throttle open.
#[test]
fn fuel_cut_is_not_active_below_the_resume_speed_or_with_the_throttle_open() {
    assert!(watch(&mut single(json!({ "throttle": 0, "rpm": 1000 })), 1.0).fired > 0.0);
    let mut open = single(json!({ "throttle": 0.05, "rpm": 3200 }));
    watch(&mut open, 0.2);
    assert!(!open.snapshot().fuel_cut);
}

/// Brings the fuel back as soon as the throttle opens.
#[test]
fn fuel_cut_brings_the_fuel_back_as_soon_as_the_throttle_opens() {
    let mut sim = single(json!({ "throttle": 0, "rpm": 3200 }));
    watch(&mut sim, 0.5);
    sim.set_controls(0.5, 0.0);
    assert_eq!(watch(&mut sim, 1.0).fired, 1.0);
}

// --- valves per cylinder ---

/// Mean gas torque, N*m, of the FA20D boxer four at full throttle and `rpm`.
fn boxer_torque(rpm: f64, over: Value) -> f64 {
    let mut sim = common::held("Boxer four, Subaru FA20D", rpm, over);
    sim.render(FS as usize / 2);
    common::gas_torque(&mut sim)
}

/// The FA20D makes about the real engine's rated 205 N*m (151 lb-ft) at 6400-6600 rpm and 200 hp at 7000.
#[test]
fn the_fa20d_makes_about_the_real_engines_torque_and_power() {
    let [t6500, t7000] = common::brake_torques_at("Boxer four, Subaru FA20D", 2.0, [6500.0, 7000.0]);
    assert!((t6500 - 205.0).abs() < 0.1 * 205.0, "{t6500} N*m at 6500 rpm");
    let hp = common::hp(t7000, 7000.0);
    assert!((hp - 200.0).abs() < 0.1 * 200.0, "{hp} hp at 7000 rpm");
}

/// The Mezger 4.0 makes about the real engine's rated 460 N*m (339 lb-ft) at 5750 rpm and 500 PS (368 kW) at
/// 8250.
#[test]
fn the_mezger_4_0_makes_about_the_real_engines_torque_and_power() {
    let [t5750, t8250] = common::brake_torques_at("Boxer six, Porsche Mezger 4.0", 2.0, [5750.0, 8250.0]);
    assert!((t5750 - 460.0).abs() < 0.1 * 460.0, "{t5750} N*m at 5750 rpm");
    let kw = common::kw(t8250, 8250.0);
    assert!((kw - 368.0).abs() < 0.1 * 368.0, "{kw} kW at 8250 rpm");
}

/// Lets a four-valve head breathe at high rpm, where one valve of each chokes.
///
/// A four-valve engine keeps most of its torque to near its rev limit, falling off past the speed its
/// intake runners are tuned for. The same valves, one of each, choke it: the cylinder cannot empty
/// through them, and torque halves long before the limit.
#[test]
fn valves_per_cylinder_let_a_four_valve_head_breathe_at_high_rpm_where_one_valve_of_each_chokes() {
    let one = json!({ "exValveCount": 1, "inValveCount": 1 });
    let points = [(6200.0, json!({})), (3600.0, json!({})), (6200.0, one.clone()), (3600.0, one)];
    let [four_high, four_low, two_high, two_low] = common::par(points, |(rpm, over)| boxer_torque(*rpm, over.clone()));
    let (four, two) = (four_high / four_low, two_high / two_low);
    assert!(four > 0.75, "four {four}");
    assert!(two < four - 0.2, "two {two} four {four}");
}
