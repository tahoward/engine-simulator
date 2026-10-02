//! Engines with more than two cylinders: the firing plan, the crank it implies, the collectors,
//! and what the running engine does with all of it.
//!
//! The V8 pair is the point of this file. A crossplane and a flatplane V8 fire at exactly the same
//! crank angles — every 90 degrees — so nothing about the *overall* firing distinguishes them.
//! What differs is which bank each firing belongs to, which means the difference only becomes
//! audible once each bank has its own collector. That is a strong claim and it is the one worth
//! nailing down.

mod common;

use common::{FS, band_energy, hann, magnitude_spectrum};
use engine_sim::engine_sim::spread_of;
use engine_sim::exhaust_graph::{compile_collector_layout, radiating_ducts};
use engine_sim::listener::{MouthPlace, SoundSources};
use engine_sim::spec::{
    EngineSpec, ExhaustLayout, PipeSegment, SegmentKind, SegmentPartial, collector_groups, crank_pins,
    exhaust_layout_of, firing_plan, gas, make_segment,
};
use engine_sim::{EngineConfig, EngineSim};
use serde_json::{Value, json};
use std::collections::BTreeSet;

const FS_N: usize = FS as usize;

/// The default engine with `over` applied.
fn spec(over: Value) -> EngineSpec {
    common::with(&common::default_config().engine, over)
}

/// `a` with the fields of `b` over it.
fn merge(a: Value, b: Value) -> Value {
    let mut out = a;
    let obj = out.as_object_mut().expect("object patch");
    for (k, v) in b.as_object().expect("object patch") {
        obj.insert(k.clone(), v.clone());
    }
    out
}

/// The V8 the tests share, with `extra` over it.
fn v8(extra: Value) -> Value {
    merge(json!({ "cylinders": 8, "vAngle": 90, "exhaustLayout": "perBank" }), extra)
}

fn pipe(length: f64, d_in: f64) -> PipeSegment {
    make_segment(SegmentPartial {
        kind: Some(SegmentKind::Pipe),
        length: Some(length),
        d_in: Some(d_in),
        ..Default::default()
    })
}

/// A sim on 0.4 m runners into 0.8 m collectors, with `extra` over `base` over the defaults, run
/// for `seconds`.
fn build_with(base: Value, over: Value, seconds: usize) -> EngineSim {
    let mut sim = EngineSim::new(FS, &config_with(base, over));
    sim.render(FS_N * seconds);
    sim
}

/// `build_with`'s config, before it runs.
fn config_with(base: Value, over: Value) -> EngineConfig {
    let mut cfg = common::default_config();
    cfg.engine = common::with(&common::with(&cfg.engine, base), over);
    cfg.pipe = vec![pipe(0.4, 0.042)];
    cfg.collector = vec![pipe(0.8, 0.065)];
    // The equal-length collector system, explicitly.
    //
    // These tests are about what matched cylinders do when their pulses reach a merge evenly
    // spaced — cancellation, pulse spacing per bank — and that needs every cylinder's path to air
    // the same length. The default compiled exhaust is a manifold along each bank, whose paths
    // differ by design.
    cfg.graph = Some(compile_collector_layout(&cfg.engine, &cfg.pipe, &cfg.collector));
    // Its own mouths, not the default engine's: those are placed by `spaced` where a test needs them.
    cfg.sources = None;
    cfg
}

/// A running engine: full throttle at a pinned 4000 rpm on 35 mm cells.
fn build(over: Value, seconds: usize) -> EngineSim {
    build_with(json!({ "rpm": 4000, "throttle": 1, "freeRunning": false, "pipeCellSize": 0.035 }), over, seconds)
}

/// Crank degrees from each firing to the next, in firing order, over the whole engine.
fn intervals(s: &EngineSpec) -> Vec<f64> {
    let mut fires = firing_plan(s).offsets;
    fires.sort_by(|a, b| a.partial_cmp(b).unwrap());
    gaps(&fires)
}

/// Crank degrees between successive firings of one bank, in firing order.
fn bank_firing_intervals(s: &EngineSpec, bank: u32) -> Vec<f64> {
    let plan = firing_plan(s);
    let mut fires: Vec<f64> =
        plan.offsets.iter().zip(&plan.banks).filter(|(_, b)| **b == bank).map(|(o, _)| *o).collect();
    fires.sort_by(|a, b| a.partial_cmp(b).unwrap());
    gaps(&fires)
}

fn gaps(fires: &[f64]) -> Vec<f64> {
    (0..fires.len()).map(|i| if i + 1 < fires.len() { fires[i + 1] } else { fires[0] + 720.0 } - fires[i]).collect()
}

// --- the three, five, six and V6 ---

fn check_inline(n: u32, every: f64, pins: &[f64]) {
    let s = spec(json!({ "cylinders": n, "vAngle": 0 }));
    assert_eq!(intervals(&s), vec![every; n as usize]);
    assert_eq!(firing_plan(&s).bank_count, 1);
    assert_eq!(crank_pins(&s).iter().map(|p| p.angle_deg).collect::<Vec<_>>(), pins);
}

/// An inline 3 fires every 240 degrees on its crank.
#[test]
fn an_inline_3_fires_every_240_degrees_on_its_crank() {
    check_inline(3, 240.0, &[0.0, 120.0, 240.0]);
}

/// An inline 5 fires every 144 degrees on its crank.
#[test]
fn an_inline_5_fires_every_144_degrees_on_its_crank() {
    check_inline(5, 144.0, &[0.0, 144.0, 216.0, 288.0, 72.0]);
}

/// An inline 6 fires every 120 degrees on its crank.
#[test]
fn an_inline_6_fires_every_120_degrees_on_its_crank() {
    check_inline(6, 120.0, &[0.0, 120.0, 240.0, 240.0, 120.0, 0.0]);
}

/// A 60-degree V6 fires every 120 on three split throws.
///
/// A 60-degree V6 needs a split-pin crank to fire evenly: at that vee a shared pin gives 60-180.
/// Each throw carries one pin per bank, 60 degrees apart round the shaft.
#[test]
fn a_60_degree_v6_fires_every_120_on_three_split_throws() {
    let s = spec(json!({ "cylinders": 6, "vAngle": 60 }));
    assert_eq!(intervals(&s), vec![120.0; 6]);
    assert_eq!(firing_plan(&s).bank_count, 2);
    let pins = crank_pins(&s);
    assert_eq!(pins.len(), 3);
    for pin in &pins {
        assert_eq!(pin.cylinders.len(), 2);
        let split = ((pin.angles[1] - pin.angles[0]) % 360.0 + 360.0) % 360.0;
        assert!((split - 60.0).abs() < 0.5e-9, "split {split}");
    }
    // Each bank hears every other firing: 240 apart.
    assert_eq!(bank_firing_intervals(&s, 0), vec![240.0, 240.0, 240.0]);
}

/// And fires unevenly at any other vee, as that crank would.
#[test]
fn and_fires_unevenly_at_any_other_vee_as_that_crank_would() {
    let s = spec(json!({ "cylinders": 6, "vAngle": 90 }));
    let iv = intervals(&s);
    let distinct: BTreeSet<u64> = iv.iter().map(|v| v.to_bits()).collect();
    assert!(distinct.len() > 1, "{iv:?}");
    assert_eq!(iv.iter().sum::<f64>(), 720.0);
}

/// A boxer: a throw per cylinder, and each opposed pair at top dead centre together, so the two
/// pistons move out and in as one. The firing orders are the Subaru's 1-3-2-4 and the Porsche's
/// 1-6-2-4-3-5, in this model's numbering by throw from the front.
fn check_boxer(n: u32, every: f64, order: &[usize]) {
    let s = spec(json!({ "cylinders": n, "vAngle": 180, "crankType": "boxer", "exhaustLayout": "perBank" }));
    let plan = firing_plan(&s);
    assert_eq!(intervals(&s), vec![every; n as usize]);
    let mut by_offset: Vec<(f64, usize)> = plan.offsets.iter().copied().zip(0..).collect();
    by_offset.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    assert_eq!(by_offset.iter().map(|&(_, i)| i).collect::<Vec<_>>(), order);
    assert_eq!(crank_pins(&s).len(), n as usize);
    for pair in (0..n as usize).step_by(2) {
        let (a, b) = (pair, pair + 1);
        assert_ne!(plan.banks[a], plan.banks[b]);
        // Top dead centre at the same crank angle: fired a revolution apart.
        assert_eq!((plan.offsets[a] - plan.offsets[b]).abs(), 360.0);
    }
    assert_eq!(collector_groups(&s), plan.banks.iter().map(|&b| b as i32).collect::<Vec<_>>());
}

/// A boxer 4 fires every 180 on a throw per cylinder.
#[test]
fn a_boxer_4_fires_every_180_on_a_throw_per_cylinder() {
    check_boxer(4, 180.0, &[0, 2, 1, 3]);
}

/// A boxer 6 fires every 120 on a throw per cylinder.
#[test]
fn a_boxer_6_fires_every_120_on_a_throw_per_cylinder() {
    check_boxer(6, 120.0, &[0, 5, 2, 1, 4, 3]);
}

/// Spreads every cylinder count.
///
/// Every cylinder gets its own place in the spread of breathing, cam timing and head ring.
///
/// The spread's shuffle only permutes when its step shares no factor with the cylinder count, and
/// the steps are 3, 5 and 7 — so taken as they are, a five, or a three or six, would give every
/// cylinder the same place.
#[test]
fn spreads_every_cylinder_count() {
    for n in [2, 3, 4, 5, 6, 8] {
        for (step, offset) in [(5, 2), (3, 1), (7, 3), (3, 2)] {
            let places: BTreeSet<String> = (0..n).map(|b| format!("{:.9}", spread_of(b, n, step, offset))).collect();
            assert_eq!(places.len(), n, "{n} cylinders, step {step}");
        }
    }
}

// --- firing plans ---

/// An inline four fires 1-3-4-2, evenly every 180 degrees on one bank.
#[test]
fn an_inline_four_fires_1_3_4_2_evenly_every_180_degrees_on_one_bank() {
    let s = spec(json!({ "cylinders": 4 }));
    let plan = firing_plan(&s);
    assert_eq!(plan.offsets, vec![0.0, 540.0, 180.0, 360.0]);
    assert_eq!(plan.bank_count, 1);
    assert_eq!(bank_firing_intervals(&s, 0), vec![180.0; 4]);
}

/// Both V8 cranks fire every 90 degrees — overall they are indistinguishable.
#[test]
fn both_v8_cranks_fire_every_90_degrees_overall_they_are_indistinguishable() {
    for crank_type in ["crossplane", "flatplane"] {
        let mut offsets = firing_plan(&spec(v8(json!({ "crankType": crank_type })))).offsets;
        offsets.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(offsets, vec![0.0, 90.0, 180.0, 270.0, 360.0, 450.0, 540.0, 630.0], "{crank_type}");
    }
}

/// But they deal those firings out to the banks quite differently.
#[test]
fn but_they_deal_those_firings_out_to_the_banks_quite_differently() {
    // This is the whole difference between the two engines.
    let cross = spec(v8(json!({ "crankType": "crossplane" })));
    assert_eq!(bank_firing_intervals(&cross, 0), vec![180.0, 90.0, 180.0, 270.0]);
    assert_eq!(bank_firing_intervals(&cross, 1), vec![270.0, 180.0, 90.0, 180.0]);

    let flat = spec(v8(json!({ "crankType": "flatplane" })));
    assert_eq!(bank_firing_intervals(&flat, 0), vec![180.0; 4]);
    assert_eq!(bank_firing_intervals(&flat, 1), vec![180.0; 4]);
}

/// Every cylinder fires exactly once per cycle, at a distinct angle.
#[test]
fn every_cylinder_fires_exactly_once_per_cycle_at_a_distinct_angle() {
    for over in [
        json!({ "cylinders": 1 }),
        json!({ "cylinders": 2 }),
        json!({ "cylinders": 4 }),
        v8(json!({ "crankType": "crossplane" })),
        v8(json!({ "crankType": "flatplane" })),
    ] {
        let s = spec(over);
        let plan = firing_plan(&s);
        assert_eq!(plan.offsets.len(), s.cylinders as usize);
        let distinct: BTreeSet<u64> = plan.offsets.iter().map(|v| v.to_bits()).collect();
        assert_eq!(distinct.len(), plan.offsets.len());
        for &o in &plan.offsets {
            assert!((0.0..720.0).contains(&o), "offset {o}");
        }
    }
}

// --- the crank the plan implies ---

/// Recovers a crossplane crank from the crossplane firing plan.
///
/// `crank_pins` is derived, not stored, so it is a genuine prediction: the pin *angles* it recovers
/// should be the ones the crank is named after. They are, which is a pleasing check on the firing
/// data — a crossplane crank really does come out with its pins at 90-degree intervals, and a
/// flatplane one with all four in a single plane.
#[test]
fn recovers_a_crossplane_crank_from_the_crossplane_firing_plan() {
    let pins = crank_pins(&spec(v8(json!({ "crankType": "crossplane" }))));
    assert_eq!(pins.len(), 4);
    // Along the crank, with the end throws half a turn apart, so the secondary couple cancels.
    let angles: Vec<f64> = pins.iter().map(|p| p.angle_deg).collect();
    assert_eq!(angles, vec![0.0, 270.0, 90.0, 180.0]);
    for p in &pins {
        assert_eq!(p.cylinders.len(), 2);
    }
}

/// And a flatplane crank from the flatplane one.
#[test]
fn and_a_flatplane_crank_from_the_flatplane_one() {
    let pins = crank_pins(&spec(v8(json!({ "crankType": "flatplane" }))));
    assert_eq!(pins.len(), 4);
    // All in one plane, and in an inline four's order along the crank, so the primary couple cancels.
    let angles: Vec<f64> = pins.iter().map(|p| p.angle_deg).collect();
    assert_eq!(angles, vec![0.0, 180.0, 180.0, 0.0]);
    for p in &pins {
        assert_eq!(p.cylinders.len(), 2);
    }
}

/// Gives a V-twin one shared pin and an inline four a pin each.
#[test]
fn gives_a_v_twin_one_shared_pin_and_an_inline_four_a_pin_each() {
    assert_eq!(crank_pins(&spec(json!({ "cylinders": 2, "vAngle": 45, "firingOffset": null }))).len(), 1);
    assert_eq!(crank_pins(&spec(json!({ "cylinders": 4 }))).len(), 4);
}

/// Refuses to pair cylinders no shared pin could carry.
#[test]
fn refuses_to_pair_cylinders_no_shared_pin_could_carry() {
    // 270/450 is exactly what the firing-offset override exists for: a crank with two pins.
    let pins = crank_pins(&spec(json!({ "cylinders": 2, "vAngle": 45, "firingOffset": 270 })));
    assert_eq!(pins.len(), 2);
    for p in &pins {
        assert_eq!(p.cylinders.len(), 1);
    }
}

/// Accounts for every cylinder exactly once, whatever the layout.
#[test]
fn accounts_for_every_cylinder_exactly_once_whatever_the_layout() {
    for over in [
        json!({ "cylinders": 1 }),
        json!({ "cylinders": 2, "firingOffset": 270 }),
        json!({ "cylinders": 4 }),
        v8(json!({ "crankType": "crossplane" })),
        v8(json!({ "crankType": "flatplane" })),
    ] {
        let s = spec(over);
        let mut seen: Vec<usize> = crank_pins(&s).into_iter().flat_map(|p| p.cylinders).collect();
        seen.sort();
        assert_eq!(seen, (0..s.cylinders as usize).collect::<Vec<_>>());
    }
}

// --- plumbing ---

/// Groups cylinders by bank, all together, or not at all.
#[test]
fn groups_cylinders_by_bank_all_together_or_not_at_all() {
    assert_eq!(
        collector_groups(&spec(v8(json!({ "exhaustLayout": "perBank", "crankType": "flatplane" })))),
        vec![0, 1, 0, 1, 0, 1, 0, 1]
    );
    assert_eq!(collector_groups(&spec(v8(json!({ "exhaustLayout": "merged" })))), vec![0; 8]);
    assert_eq!(collector_groups(&spec(v8(json!({ "exhaustLayout": "open" })))), vec![-1; 8]);
}

/// Builds the ducts the grouping asks for.
#[test]
fn builds_the_ducts_the_grouping_asks_for() {
    let per_bank = build(v8(json!({ "exhaustLayout": "perBank" })), 1);
    assert_eq!(per_bank.pipe_solver().primaries().len(), 8);
    assert_eq!(per_bank.pipe_solver().collectors().len(), 2);

    let merged = build(v8(json!({ "exhaustLayout": "merged" })), 1);
    assert_eq!(merged.pipe_solver().collectors().len(), 1);

    let open = build(v8(json!({ "exhaustLayout": "open" })), 1);
    assert_eq!(open.pipe_solver().collectors().len(), 0);
}

/// Accepts the single's and twins' layout names.
#[test]
fn accepts_the_singles_and_twins_layout_names() {
    // The twin presets use them, and so may a saved link.
    assert_eq!(exhaust_layout_of(&spec(json!({ "cylinders": 2, "exhaustLayout": "2into1" }))), ExhaustLayout::Merged);
    assert_eq!(exhaust_layout_of(&spec(json!({ "cylinders": 2, "exhaustLayout": "2into2" }))), ExhaustLayout::Open);
    assert_eq!(exhaust_layout_of(&spec(json!({ "cylinders": 1, "exhaustLayout": "single" }))), ExhaustLayout::Open);
}

/// Collapses per-bank to merged when there is only one bank.
#[test]
fn collapses_per_bank_to_merged_when_there_is_only_one_bank() {
    assert_eq!(exhaust_layout_of(&spec(json!({ "cylinders": 4, "exhaustLayout": "perBank" }))), ExhaustLayout::Merged);
}

// --- a V8 runs ---

/// All eight cylinders fire, at the planned crank angles.
#[test]
fn all_eight_cylinders_fire_at_the_planned_crank_angles() {
    let mut sim = build(v8(json!({ "crankType": "crossplane" })), 1);
    let engine = sim.engine().clone();
    let plan = firing_plan(&engine);
    let mut peak = [0.0f64; 8];
    let mut at = [0.0f64; 8];
    // Two full cycles at 4000 rpm is 0.06 s; take a quarter second to be safe.
    for _ in 0..FS_N / 4 {
        sim.tick();
        let reference = sim.cylinders()[0].angle;
        for b in 0..8 {
            let p = sim.cylinders()[b].pressure(&engine);
            if p > peak[b] {
                peak[b] = p;
                at[b] = reference;
            }
        }
    }
    for p in peak {
        assert!(p > 20e5, "peak pressure {p}");
    }
    // Each cylinder's peak must land where the plan says it fires. Seeding the crank with
    // `+offset` instead of `-offset` runs the order backwards and fails this.
    for b in 0..8 {
        let rel = ((at[b] - at[0]) % 720.0 + 720.0) % 720.0;
        assert!((rel - plan.offsets[b]).abs() < 6.0, "cylinder {}: {rel} vs {}", b + 1, plan.offsets[b]);
    }
}

/// Stays finite and admissible in every layout.
#[test]
fn stays_finite_and_admissible_in_every_layout() {
    for layout in ["open", "perBank", "merged"] {
        let mut sim = build(v8(json!({ "exhaustLayout": layout })), 1);
        let buf = sim.render(FS_N / 2);
        let mut peak = 0.0f32;
        for &v in &buf {
            assert!(v.is_finite(), "{layout}");
            peak = peak.max(v.abs());
        }
        assert!(peak > 1e-3, "{layout} silent");
        assert!(peak < 1.0, "{layout} pinned");
        assert_eq!(sim.pipe_solver().recoveries(), 0, "{layout}");
        for c in sim.cylinders() {
            assert_eq!(c.clamp_hits, 0, "{layout}");
        }
    }
}

/// Runs at one substep, pinned, with no bursts.
#[test]
fn runs_at_one_substep_pinned_with_no_bursts() {
    // 35 mm cells is what makes a V8 affordable: the CFL limit then clears an audio sample
    // outright, so the count is pinned at one instead of two. See DESIGN_WAVE_SPEED.
    let mut sim = build(v8(json!({})), 1);
    let ws = sim.pipe_solver();
    for d in ws.primaries().iter().chain(ws.collectors()) {
        assert_eq!(d.substeps(), 1);
    }
    sim.render(FS_N);
    let ws = sim.pipe_solver();
    let bursts: u64 = ws.primaries().iter().chain(ws.collectors()).map(|d| d.substep_bursts).sum();
    assert_eq!(bursts, 0);
}

/// Fires eight times per cycle, so its firing frequency is four times a single.
#[test]
fn fires_eight_times_per_cycle_so_its_firing_frequency_is_four_times_a_single() {
    let mut sim = build(v8(json!({ "rpm": 3000, "exhaustLayout": "merged" })), 2);
    let n = 65536;
    let mag = magnitude_spectrum(&hann(&sim.render(n)), n);
    let half = 3000.0 / 120.0; // 25 Hz, one firing per cylinder per cycle
    // The eighth order — eight firings per 720 degrees — must dominate the half order.
    let eighth = band_energy(&mag, FS, n, half * 8.0, 4.0);
    assert!(eighth > band_energy(&mag, FS, n, half, 4.0) * 10.0);
}

// --- crossplane against flatplane ---

/// The central claim: the two cranks differ only in bank pattern, so they must differ *at the bank
/// collectors* and be near-identical through one shared collector.
///
/// Measured at each bank's own collector mouth, using the pressure fluctuation the collector sees.
/// An uneven arrival pattern (270-180-90-180) and an even one (180 x 4) cannot produce the same
/// spectrum, and the giveaway is the half order: four evenly spaced pulses per bank put all their
/// energy at multiples of the fourth order, where an uneven pattern spills energy into the lower
/// orders that only repeat once per cycle.
fn bank_order_energy(crank_type: &str, order: f64) -> f64 {
    let rpm = 3000.0;
    let mut sim = build(v8(json!({ "crankType": crank_type, "exhaustLayout": "perBank", "rpm": rpm })), 2);
    let n = 32768;
    // Pressure at bank 0's collector inlet: what that bank's plumbing actually hears.
    let out: Vec<f32> = (0..n)
        .map(|_| {
            sim.tick();
            (sim.pipe_solver().collectors()[0].pressure_at(0) - gas::P_AMB) as f32
        })
        .collect();
    let mag = magnitude_spectrum(&hann(&out), n);
    band_energy(&mag, FS, n, (rpm / 120.0) * order, 4.0)
}

/// A flatplane bank hears four evenly spaced pulses, a crossplane bank does not.
#[test]
fn a_flatplane_bank_hears_four_evenly_spaced_pulses_a_crossplane_bank_does_not() {
    // Order 2 repeats twice per cycle: an evenly firing bank (every 180 crank degrees = order 4)
    // should put very little there, while the uneven crossplane pattern must.
    let cross_second = bank_order_energy("crossplane", 2.0);
    let flat_second = bank_order_energy("flatplane", 2.0);
    assert!(cross_second > flat_second * 5.0, "cross {cross_second} vs flat {flat_second}");
}

/// A flatplane bank concentrates on the fourth order where a crossplane bank spreads.
#[test]
fn a_flatplane_bank_concentrates_on_the_fourth_order_where_a_crossplane_bank_spreads() {
    // Four evenly spaced firings per bank put their energy at multiples of the fourth order and
    // very little between; an uneven pattern repeats only once per cycle, so it fills in the lower
    // orders too. Measured as the fourth order against the third, which is the gap the even
    // pattern should not excite. Note this is *not* a claim that the fourth order dominates the
    // crossplane spectrum — measured, its third order is larger, which is the whole point.
    let flat = bank_order_energy("flatplane", 4.0) / bank_order_energy("flatplane", 3.0);
    let cross = bank_order_energy("crossplane", 4.0) / bank_order_energy("crossplane", 3.0);
    assert!(flat > cross * 20.0, "flat {flat} vs cross {cross}");
}

// --- every preset ---

/// Runs clean and audible.
#[test]
fn every_preset_runs_clean_and_audible() {
    for preset in &common::presets().engine_presets {
        let name = &preset.name;
        let mut cfg = preset.config.clone();
        cfg.engine.throttle = 1.0;
        let mut sim = EngineSim::new(FS, &cfg);
        sim.render(FS_N);
        let buf = sim.render(FS_N / 2);
        let mut peak = 0.0f32;
        for &v in &buf {
            assert!(v.is_finite(), "{name}");
            peak = peak.max(v.abs());
        }
        assert!(peak > 1e-3, "{name} silent");
        assert!(peak < 1.0, "{name} pinned");
        assert_eq!(sim.pipe_solver().recoveries(), 0, "{name}");
        for c in sim.cylinders() {
            assert_eq!(c.clamp_hits, 0, "{name}");
        }
    }
}

// --- why a multi-cylinder engine does not just go up in pitch ---
//
// Two things keep an engine with several tailpipes sounding like one.
//
// Both are about cancellation. Evenly spaced firing cancels every order that is not a multiple of
// the cylinder count, which is correct and is why a multi sounds smooth. But a model can make *two
// further* cancellations perfect when reality does not: separate mouths summed at a single point,
// and cylinders that breathe identically.

const SPECTRUM_N: usize = 32768;

/// The spectrum of a full-throttle engine with `over`, on equal-length runners into collectors,
/// after `seconds` of settling.
fn render(over: Value, seconds: usize) -> Vec<f64> {
    // Equal-length runners into collectors: cancellation is a property of evenly spaced merging
    // pulses.
    let mut sim = build_with(json!({ "throttle": 1, "freeRunning": false, "pipeCellSize": 0.035 }), over, seconds);
    magnitude_spectrum(&hann(&sim.render(SPECTRUM_N)), SPECTRUM_N)
}

/// `render`, with the mouths in a row across the car `spacing` apart, a metre behind the crank.
fn render_spaced(over: Value, spacing: f64, seconds: usize) -> Vec<f64> {
    let mut cfg = config_with(json!({ "throttle": 1, "freeRunning": false, "pipeCellSize": 0.035 }), over);
    let graph = cfg.graph.clone().unwrap();
    let ids: Vec<String> = radiating_ducts(&graph).into_iter().map(|d| graph.ducts[d].id.clone()).collect();
    let middle = (ids.len() as f64 - 1.0) / 2.0;
    let mouths = ids
        .into_iter()
        .enumerate()
        .map(|(k, duct)| MouthPlace { duct, position: [(k as f64 - middle) * spacing, 0.0, 1.0] })
        .collect();
    cfg.sources = Some(SoundSources { mouths, ..Default::default() });
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(FS_N * seconds);
    magnitude_spectrum(&hann(&sim.render(SPECTRUM_N)), SPECTRUM_N)
}

fn flatplane_v8(extra: Value) -> Value {
    merge(
        json!({ "cylinders": 8, "vAngle": 90, "crankType": "flatplane", "exhaustLayout": "perBank", "rpm": 5600 }),
        extra,
    )
}

/// Separate mouths must not annihilate the bank firing order.
///
/// A flatplane V8's two banks fire in exact antiphase, so summing their mouths at one point
/// annihilates the loudest thing in the spectrum — each bank's own firing order — and the engine
/// jumps an octave to the doubled order.
///
/// Real tailpipes are a metre or so apart, which at 187 Hz is most of a wavelength.
#[test]
fn separate_mouths_must_not_annihilate_the_bank_firing_order() {
    let bank_order_hz = (5600.0 / 120.0) * 4.0; // four firings per bank per cycle
    let coincident = render_spaced(flatplane_v8(json!({})), 0.0, 1);
    let spread = render_spaced(flatplane_v8(json!({})), 1.3, 1);
    let at = |mag: &[f64]| band_energy(mag, FS, SPECTRUM_N, bank_order_hz, 4.0);
    // Measures about 85x on this geometry (19 dB) and four orders of magnitude on the shipped
    // preset, the difference being where the bank order falls relative to the pipe's resonances.
    assert!(at(&spread) > at(&coincident) * 20.0, "spread {} vs coincident {}", at(&spread), at(&coincident));
}

/// And the spacing has to be off the mouths' own axis to do anything.
#[test]
fn and_the_spacing_has_to_be_off_the_mouths_own_axis_to_do_anything() {
    // Mouths placed symmetrically about the listener's axis are all the *same* distance away, so
    // the path differences are zero and the sum is as coherent as if they were coincident: the
    // output does not change at all, bit for bit. The listener therefore stands off to one side.
    // This test pins the consequence: sweeping the spacing must actually change the output.
    let bank_order_hz = (5600.0 / 120.0) * 4.0;
    let at = |mag: &[f64]| band_energy(mag, FS, SPECTRUM_N, bank_order_hz, 6.0);
    let near = at(&render_spaced(flatplane_v8(json!({})), 0.3, 1));
    let far = at(&render_spaced(flatplane_v8(json!({})), 1.3, 1));
    // Measured at the bank order rather than broadband, because that is where the path differences
    // do their work; a whole-spectrum metric also moves when anything else changes, and with merge
    // noise in the mix it sits marginally either side of any sensible threshold.
    assert!(near.max(far) / near.min(far) > 3.0, "near {near} vs far {far}");
}

/// Unequal cylinder breathing restores the low orders.
///
/// With identical cylinders the non-multiple orders nearly cancel: what is left of them, about 35 dB
/// down, comes only from the plenum, the cylinders at its far end drawing from air its waves leave
/// slightly different from that by the throttle. Real engines sit 20 to 35 dB down, because no two
/// cylinders breathe alike. Without this an inline four is all but a pure tone on its firing frequency
/// with little rumble underneath, which is the other half of sounding wrong.
#[test]
fn unequal_cylinder_breathing_restores_the_low_orders() {
    let rpm = 3400.0;
    let half = rpm / 120.0;
    let base = json!({ "cylinders": 4, "exhaustLayout": "merged", "rpm": rpm });
    let matched = render(merge(base.clone(), json!({ "cylinderSpread": 0 })), 2);
    let real = render(merge(base, json!({ "cylinderSpread": 1 })), 2);

    let ratio = |mag: &[f64], order: f64| {
        band_energy(mag, FS, SPECTRUM_N, half * order, 3.0) / band_energy(mag, FS, SPECTRUM_N, half * 4.0, 3.0)
    };

    // The third order — 1.5 times per revolution — is the strongest of the cancelled ones.
    assert!(
        ratio(&real, 3.0) > ratio(&matched, 3.0) * 5.0,
        "real {} vs matched {}",
        ratio(&real, 3.0),
        ratio(&matched, 3.0)
    );
    // And it must land in the range real engines occupy, not merely rise: -20 to -35 dB.
    let db = 10.0 * ratio(&real, 3.0).log10();
    assert!(db > -40.0 && db < -12.0, "third order at {db} dB");
}

/// The spread is deterministic, so an engine sounds the same each time it starts.
#[test]
fn the_spread_is_deterministic_so_an_engine_sounds_the_same_each_time_it_starts() {
    let base = json!({ "cylinders": 8, "vAngle": 90, "exhaustLayout": "perBank", "rpm": 3200 });
    let a = render(base.clone(), 1);
    let b = render(base, 1);
    assert_eq!(a, b);
}
