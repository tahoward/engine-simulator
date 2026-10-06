//! The diesel: fuel injected into the cylinder near top dead centre and lit by compression, the pedal
//! metering it into an unthrottled intake, and a governor on the speed.

mod common;

use engine_sim::cylinder::{ignition_delay, premixed_share};
use engine_sim::engine_sim::EngineSim;
use engine_sim::spec::{EngineConfig, gas};
use serde_json::{Value, json};

const FS: f64 = 48000.0;
const CUMMINS: &str = "Inline six diesel, Cummins 6CT";
/// The 6CT's start of injection, deg.
const INJECTION: f64 = 708.0;

fn cummins(over: Value) -> EngineConfig {
    let mut cfg = common::engine_preset(CUMMINS).config.clone();
    cfg.engine = common::with(&cfg.engine, over);
    cfg
}

/// The 6CT held at `rpm` on `pedal`, with the fields in `over` on top, settled for `settle` s.
fn held(rpm: f64, pedal: f64, settle: f64, over: Value) -> EngineSim {
    let mut cfg = cummins(json!({ "freeRunning": false, "combustionVariability": 0, "throttle": pedal, "rpm": rpm }));
    cfg.engine = common::with(&cfg.engine, over);
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render((settle * FS) as usize);
    sim
}

/// Mean torque at the crank less friction over half a second, N*m.
fn mean_torque(sim: &mut EngineSim) -> f64 {
    let n = FS as usize / 2;
    let mut t = 0.0;
    for _ in 0..n {
        sim.render(1);
        t += sim.snapshot().torque - sim.friction_torque();
    }
    t / n as f64
}

/// The delay shortens as the air the fuel meets is hotter and denser, and is a few degrees at a running
/// diesel's top dead centre: Heywood quotes 0.4-1 ms for direct injection.
#[test]
fn the_ignition_delay_shortens_in_hotter_denser_air() {
    let typical = ignition_delay(9.0, 80e5, 900.0);
    let ms = typical / (6.0 * 2000.0) * 1e3;
    assert!(ms > 0.3 && ms < 1.2, "{typical} deg, {ms} ms at 2000 rpm");
    assert!(ignition_delay(9.0, 80e5, 1000.0) < typical);
    assert!(ignition_delay(9.0, 120e5, 900.0) < typical);
    // Air too cool to light it waits as long as the correlation allows.
    assert_eq!(ignition_delay(9.0, 15e5, 500.0), 60.0);
}

/// The longer the delay, the more of the fuel mixes before it lights and burns at once; the more fuel,
/// the less of it has mixed.
#[test]
fn the_premixed_share_grows_with_the_delay_and_shrinks_with_the_fuel() {
    assert!(premixed_share(0.3, 1.5e-3) > premixed_share(0.3, 0.5e-3));
    assert!(premixed_share(0.2, 1e-3) > premixed_share(0.7, 1e-3));
    for (phi, delay) in [(0.0, 1e-3), (1.0, 1e-5), (0.01, 1.0)] {
        let share = premixed_share(phi, delay);
        assert!((0.0..=0.9).contains(&share), "{share} at phi {phi}, {delay} s");
    }
}

/// The intake has no throttle and carries no fuel: with the pedal up the plenum sits at the atmosphere,
/// where a petrol engine's shut throttle pulls a deep vacuum.
#[test]
fn the_intake_is_unthrottled_and_carries_no_fuel() {
    let mut sim = held(1500.0, 0.0, 1.0, json!({}));
    let mut sum = 0.0;
    let n = FS as usize / 4;
    for _ in 0..n {
        sim.render(1);
        sum += sim.plenum().pressure();
        assert!(sim.intake().runners.iter().all(|r| r.inflow_fuel == 0.0));
    }
    let plenum = sum / n as f64;
    assert!((plenum - gas::P_AMB).abs() < 0.05 * gas::P_AMB, "plenum {plenum} Pa with the pedal up");

    let mut sim = held(1500.0, 1.0, 1.0, json!({}));
    for _ in 0..n {
        sim.render(1);
        assert!(sim.intake().runners.iter().all(|r| r.inflow_fuel == 0.0));
    }
}

/// Lean as it is on part pedal, far past where a spark would fail, every cycle lights, and lights
/// a few degrees after the injection.
#[test]
fn every_lean_charge_lights_after_its_ignition_delay() {
    let mut sim = held(1500.0, 0.3, 1.0, json!({}));
    let (mut cycles, mut lit, mut prev) = (0, 0, sim.cylinders()[0].angle);
    let (mut peak_burned, mut started) = (0.0f64, false);
    for _ in 0..FS as usize {
        sim.render(1);
        let c = &sim.cylinders()[0];
        peak_burned = peak_burned.max(c.burned);
        if prev < INJECTION - 60.0 && c.angle >= INJECTION - 60.0 {
            // Committed at the intake's closing: well lean, and lighting after the injection.
            let lambda = c.trapped_fresh / (14.5 * c.cycle_fuel());
            assert!(lambda > 2.5, "lambda {lambda}");
            let delay = c.spark - INJECTION;
            assert!(delay > 2.0 && delay < 15.0, "lit {delay} deg after injection");
        }
        // Each cycle's burn is long over by 400 degrees, and the next not begun.
        if prev < 400.0 && c.angle >= 400.0 {
            if started {
                cycles += 1;
                if peak_burned > 0.99 {
                    lit += 1;
                }
            }
            started = true;
            peak_burned = 0.0;
        }
        prev = c.angle;
    }
    assert!(cycles >= 10, "{cycles} cycles");
    assert_eq!(lit, cycles, "{lit} of {cycles} cycles lit");
}

/// Light load lights after a longer delay, with less fuel: much more of it burns premixed, the
/// clatter, than at full load.
#[test]
fn light_load_burns_more_of_its_fuel_premixed() {
    let light = held(1500.0, 0.1, 1.0, json!({})).cylinders()[0].premixed_share;
    let full = held(1500.0, 1.0, 2.0, json!({})).cylinders()[0].premixed_share;
    assert!(light > 0.3, "light {light}");
    assert!(full < 0.2, "full {full}");
}

/// Higher compression heats the air more, and lights the fuel sooner.
#[test]
fn higher_compression_lights_the_fuel_sooner() {
    let delay = |cr: f64| held(1500.0, 0.5, 1.0, json!({ "compressionRatio": cr })).cylinders()[0].spark - INJECTION;
    let (low, high) = (delay(14.0), delay(19.0));
    assert!(high < low, "{high} deg at 19:1, {low} at 14:1");
}

/// Off boost, full pedal is held to the smoke limit; on it, to the pump's full delivery.
#[test]
fn full_pedal_is_never_richer_than_the_smoke_limit() {
    for rpm in [900.0, 1500.0, 2100.0] {
        let sim = held(rpm, 1.0, 2.0, json!({}));
        let c = &sim.cylinders()[0];
        let lambda = c.trapped_fresh / (14.5 * c.cycle_fuel());
        assert!(lambda >= 1.5 * 0.999, "lambda {lambda} at {rpm} rpm");
        assert!(c.cycle_fuel() <= 1.1e-4 * (1.0 + 1e-9), "{} kg at {rpm} rpm", c.cycle_fuel());
    }
    // Off boost at 900 rpm, the smoke limit is what holds it.
    let sim = held(900.0, 1.0, 2.0, json!({}));
    let c = &sim.cylinders()[0];
    let lambda = c.trapped_fresh / (14.5 * c.cycle_fuel());
    assert!((lambda - 1.5).abs() < 0.01, "lambda {lambda} at 900 rpm");
}

/// None of its fuel is in the charge before it burns, so none goes down the pipe to pop, whatever the
/// pedal does.
#[test]
fn it_never_afterfires() {
    let mut cfg = cummins(json!({ "freeRunning": true, "throttle": 1, "load": 0 }));
    cfg.engine = common::with(&cfg.engine, json!({}));
    let mut sim = EngineSim::new(FS, &cfg);
    for pedal in [1.0, 0.0, 1.0, 0.0] {
        sim.set_controls(pedal, 0.0);
        sim.render(FS as usize);
    }
    assert_eq!(sim.afterfire().events(), 0);
}

/// Free-running at full pedal with no load, the governor holds it just under its governed speed, with
/// no spark cut to bounce off.
#[test]
fn the_governor_holds_it_under_its_governed_speed() {
    let mut sim = EngineSim::new(FS, &cummins(json!({ "freeRunning": true, "throttle": 1, "load": 0 })));
    sim.render(4 * FS as usize);
    let (mut lo, mut hi) = (f64::MAX, 0.0f64);
    for _ in 0..FS as usize {
        sim.render(1);
        let rpm = sim.rpm();
        lo = lo.min(rpm);
        hi = hi.max(rpm);
        assert!(!sim.snapshot().limiter);
    }
    assert!(hi < 2500.0, "up to {hi} rpm");
    assert!(lo > 2200.0, "down to {lo} rpm");
    assert!(hi - lo < 60.0, "{lo} to {hi} rpm");
}

/// Above the idle with the pedal up, the governor gives it no fuel at all.
#[test]
fn the_pedal_up_above_the_idle_gives_no_fuel() {
    let mut sim = EngineSim::new(FS, &cummins(json!({ "freeRunning": true, "throttle": 0, "load": 0, "rpm": 2000 })));
    sim.render(FS as usize / 5);
    assert!(sim.rpm() > 1200.0, "{} rpm", sim.rpm());
    assert!(sim.cylinders().iter().all(|c| c.cycle_fuel() == 0.0));
}

/// The 6CT makes about the 920 N*m (680 lb-ft) at 1500 rpm and 250 hp at 2200 its truck ratings give.
#[test]
fn the_6ct_makes_about_the_real_engines_torque_and_power() {
    let t1500 = mean_torque(&mut held(1500.0, 1.0, 2.0, json!({})));
    assert!((t1500 - 920.0).abs() < 0.1 * 920.0, "{t1500} N*m at 1500 rpm");
    let hp = mean_torque(&mut held(2200.0, 1.0, 2.0, json!({}))) * 2200.0 * 2.0 * std::f64::consts::PI / 60.0 / 745.7;
    assert!((hp - 250.0).abs() < 0.1 * 250.0, "{hp} hp at 2200 rpm");
}

