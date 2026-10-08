//! A collector must stay solvable whatever the user draws into it.
//!
//! The hard case is the collector *inlet area*, not anything about what follows it. Four 42 mm
//! primaries merging into a 42 mm inlet ask the junction to pass four pipes' worth of flow
//! through one pipe's area. Solved as drawn, the cell behind it over-expands toward vacuum and
//! the returning wave divided by a collapsed `rho c` becomes an absurd velocity: the duct
//! diverges, a V8 goes silent, the pressure display saturates, and the crank pins at the
//! free-running clamp.
//!
//! Two independent things are asserted, and the second is the one with teeth:
//!
//!   - `recoveries` stays at zero, so nothing diverged;
//!   - `junction_clamps` stays at zero, so the junction never even had to catch a degenerate
//!     end state. A geometry that merely avoids diverging is still not being solved as drawn.
//!     A junction can clamp on a large share of samples while reporting zero recoveries,
//!     which is exactly the failure a recoveries-only test misses.

use crate::common;

use common::{FS, ducts, pipe, preset_collector};
use engine_sim::EngineSim;
use engine_sim::engine_sim::grid_budget_cells;
use engine_sim::euler_pipe::EulerPipe;
use engine_sim::exhaust_graph::{compile_collector_layout, compile_exhaust, node_order};
use engine_sim::spec::{EngineConfig, EngineSpec, PipeSegment, SegmentKind, SegmentPartial, firing_plan, make_segment};
use serde_json::{Value, json};

fn seg(kind: SegmentKind, length: f64, d_in: f64, d_out: f64) -> PipeSegment {
    make_segment(SegmentPartial {
        kind: Some(kind),
        length: Some(length),
        d_in: Some(d_in),
        d_out: Some(d_out),
        ..Default::default()
    })
}

fn cone(length: f64, d_in: f64, d_out: f64) -> PipeSegment {
    seg(SegmentKind::Cone, length, d_in, d_out)
}

fn chamber(length: f64, d_in: f64, d_out: f64) -> PipeSegment {
    seg(SegmentKind::Chamber, length, d_in, d_out)
}

/// The grid budget for an engine as configured, from the graph it builds.
fn budget_of(cfg: &EngineConfig) -> f64 {
    let graph = cfg.graph.clone().unwrap_or_else(|| compile_exhaust(&cfg.engine, &cfg.pipe, &cfg.collector));
    grid_budget_cells(cfg.engine.cylinders as usize, node_order(&graph).len())
}

fn v8() -> Value {
    json!({ "cylinders": 8, "vAngle": 90, "crankType": "flatplane", "exhaustLayout": "perBank" })
}

/// `base` with `a` and then `b` over it.
fn spec_with(base: &EngineSpec, a: Value, b: Value) -> EngineSpec {
    common::with(&common::with(base, a), b)
}

fn recoveries(ducts: &[&EulerPipe]) -> u64 {
    ducts.iter().map(|d| d.recoveries).sum()
}

fn clamps(ducts: &[&EulerPipe]) -> u64 {
    ducts.iter().map(|d| d.junction_clamps).sum()
}

fn max_temperature(ducts: &[&EulerPipe]) -> f64 {
    let mut max_t = 0.0f64;
    for d in ducts {
        for k in 0..d.n {
            max_t = max_t.max(d.temperature_at(k));
        }
    }
    max_t
}

fn cells(ducts: &[&EulerPipe]) -> usize {
    ducts.iter().map(|d| d.n).sum()
}

fn max_substeps(ducts: &[&EulerPipe]) -> usize {
    ducts.iter().map(|d| d.substeps()).max().unwrap()
}

struct Health {
    recoveries: u64,
    clamps: u64,
    finite: bool,
    peak: f64,
}

/// A collector-layout config for `engine` held at `rpm`, full throttle.
fn collector_config(
    engine: &EngineSpec,
    pipe: Vec<PipeSegment>,
    collector: Vec<PipeSegment>,
    rpm: f64,
) -> EngineConfig {
    let mut cfg = common::default_config();
    cfg.engine = common::with(engine, json!({ "rpm": rpm, "throttle": 1, "freeRunning": false }));
    cfg.pipe = pipe;
    cfg.collector = collector;
    // A collector system: these tests are about the junction a set of runners merges at.
    cfg.graph = Some(compile_collector_layout(&cfg.engine, &cfg.pipe, &cfg.collector));
    cfg
}

fn run_health(
    engine: &EngineSpec,
    pipe: Vec<PipeSegment>,
    collector: Vec<PipeSegment>,
    rpm: f64,
    seconds: f64,
) -> Health {
    let cfg = collector_config(engine, pipe, collector, rpm);
    let mut sim = EngineSim::new(FS, &cfg);
    let mut finite = true;
    let mut peak = 0.0f64;
    for _ in 0..(FS * seconds) as usize {
        // Out of stereo, both channels are the one ear.
        let [y, _] = sim.tick();
        if !y.is_finite() {
            finite = false;
        } else if y.abs() > peak {
            peak = y.abs();
        }
    }
    let ducts = ducts(&sim);
    Health { recoveries: recoveries(&ducts), clamps: clamps(&ducts), finite, peak }
}

fn primary() -> Vec<PipeSegment> {
    vec![pipe(0.4, 0.042)]
}

fn v8_spec() -> EngineSpec {
    common::with(&common::default_config().engine, v8())
}

/// Chambers and a cone behind a starved inlet, a pipe-then-chamber and a plain pipe as
/// controls, and two deliberately worse cases. A 42 mm inlet behind four 42 mm primaries is an
/// area ratio of 0.25, the most starved the junction is asked to handle.
mod collector_junctions_stay_solvable {
    use super::*;

    fn v8_collector_is(collector: Vec<PipeSegment>, rpm: f64) {
        let h = run_health(&v8_spec(), primary(), collector, rpm, 1.5);
        assert!(h.finite);
        assert_eq!(h.recoveries, 0);
        assert_eq!(h.clamps, 0);
        // A silent or a saturated result would both pass the checks above.
        assert!(h.peak > 1e-3, "peak {}", h.peak);
        assert!(h.peak < 4.0, "peak {}", h.peak);
    }

    /// V8, collector = chamber straight off the junction
    #[test]
    fn v8_collector_chamber_straight_off_the_junction() {
        v8_collector_is(vec![chamber(0.34, 0.042, 0.13), pipe(0.3, 0.04)], 4000.0);
    }

    /// V8, collector = cone to 130 mm then chamber
    #[test]
    fn v8_collector_cone_to_130_mm_then_chamber() {
        v8_collector_is(vec![cone(0.2, 0.055, 0.13), chamber(0.34, 0.13, 0.13), pipe(0.3, 0.04)], 4000.0);
    }

    /// V8, collector = pipe then chamber
    #[test]
    fn v8_collector_pipe_then_chamber() {
        v8_collector_is(vec![pipe(0.2, 0.055), chamber(0.34, 0.055, 0.13), pipe(0.3, 0.04)], 4000.0);
    }

    /// V8, collector = plain pipe (control)
    #[test]
    fn v8_collector_plain_pipe_control() {
        v8_collector_is(vec![pipe(0.6, 0.055)], 4000.0);
    }

    /// V8, collector = worse: 42 mm inlet to a 150 mm can at 7000 rpm
    #[test]
    fn v8_collector_worse_42_mm_inlet_to_a_150_mm_can_at_7000_rpm() {
        v8_collector_is(vec![chamber(0.3, 0.042, 0.15), pipe(0.25, 0.038)], 7000.0);
    }

    /// V8, collector = worse: straight into a 150 mm can, no tailpipe
    #[test]
    fn v8_collector_worse_straight_into_a_150_mm_can_no_tailpipe() {
        v8_collector_is(vec![chamber(0.4, 0.042, 0.15)], 6000.0);
    }

    /// Widening a starved inlet must not cost the chamber its width, as limiting the area
    /// *gradient* near the junction would — that shrinks a 130 mm can to about 92 mm. The inlet is
    /// raised; nothing downstream of it moves.
    #[test]
    fn raises_a_starved_inlet_without_shrinking_the_chamber_behind_it() {
        let cfg = collector_config(&v8_spec(), primary(), vec![chamber(0.34, 0.042, 0.13), pipe(0.3, 0.04)], 4000.0);
        let sim = EngineSim::new(FS, &cfg);
        let coll = &sim.pipe_solver().collectors()[0];

        let feed_area: f64 = sim.pipe_solver().primaries()[..4].iter().map(|d| d.outlet_area()).sum();
        // Raised to the 0.6 floor rather than left at the drawn 42 mm.
        assert!(coll.face_area(0) > 0.55 * feed_area, "{} vs {}", coll.face_area(0), feed_area);
        // The can still reaches its drawn 130 mm: pi/4 * 0.13^2 = 132.7 cm^2.
        let widest = (0..=coll.n).map(|i| coll.face_area(i)).fold(f64::NEG_INFINITY, f64::max);
        assert!(widest > (0.95 * (std::f64::consts::PI * 0.13 * 0.13)) / 4.0, "widest {widest}");
    }

    /// Every shipped preset must be clean on both counters too.
    #[test]
    fn preset_is_clean() {
        common::sweep(|preset| {
            let Some(collector) = preset_collector(preset) else {
                return;
            };
            let h = run_health(&preset.config.engine, preset.config.pipe.clone(), collector, 6500.0, 1.0);
            assert!(h.finite, "{}", preset.name);
            assert_eq!(h.recoveries, 0, "{}", preset.name);
            assert_eq!(h.clamps, 0, "{}", preset.name);
        });
    }
}

/// The bank angle must reach the sound.
///
/// `firing_plan` derives the offsets from pin angles. The offsets `[0, 90, ... 630]` whatever the
/// vee angle would be the firing pattern of a 90-degree V8 and of no other, and changing the angle
/// would do nothing. So the 90-degree case must give exactly that — the presets depend on it — and
/// every other angle must move.
mod bank_angle_reaches_the_firing_plan {
    use super::*;

    const EVEN: [f64; 8] = [0.0, 90.0, 180.0, 270.0, 360.0, 450.0, 540.0, 630.0];

    fn v8(crank_type: &str, v_angle: f64) -> EngineSpec {
        common::with(
            &common::default_config().engine,
            json!({ "cylinders": 8, "crankType": crank_type, "vAngle": v_angle }),
        )
    }

    /// %s at 90 degrees still fires evenly every 90
    #[test]
    fn at_90_degrees_still_fires_evenly_every_90() {
        for crank in ["crossplane", "flatplane"] {
            let mut offsets = firing_plan(&v8(crank, 90.0)).offsets;
            offsets.sort_by(|a, b| a.partial_cmp(b).unwrap());
            assert_eq!(offsets, EVEN, "{crank}");
        }
    }

    /// %s fires unevenly at 60 degrees
    #[test]
    fn fires_unevenly_at_60_degrees() {
        for crank in ["crossplane", "flatplane"] {
            let offsets = firing_plan(&v8(crank, 60.0)).offsets;
            assert_ne!(offsets, EVEN, "{crank}");
            // Still eight distinct firings inside one cycle, just not evenly spaced.
            let mut sorted = offsets.clone();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
            sorted.dedup();
            assert_eq!(sorted.len(), 8, "{crank}");
            let gaps: Vec<f64> = (0..8)
                .map(|i| if i == 0 { sorted[0] + 720.0 - sorted[7] } else { sorted[i] - sorted[i - 1] })
                .collect();
            let max = gaps.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            let min = gaps.iter().cloned().fold(f64::INFINITY, f64::min);
            assert!(max - min > 1.0, "{crank}: {gaps:?}");
            assert!((gaps.iter().sum::<f64>() - 720.0).abs() < 5e-7, "{crank}");
        }
    }

    /// A V-twin derives its interval from the vee; 45 degrees is the Harley 405/315.
    #[test]
    fn a_v_twin_still_derives_405_315_from_a_45_degree_vee() {
        let spec = common::with(
            &common::default_config().engine,
            json!({ "cylinders": 2, "vAngle": 45, "firingOffset": null }),
        );
        assert_eq!(firing_plan(&spec).offsets, [0.0, 405.0]);
    }

    /// The bank angle must actually change the audio, not merely the plan.
    fn sounds_different_at_a_different_vee(base: Value) {
        let twin = base["cylinders"] == 2;
        let render = |v_angle: f64| {
            let mut cfg = common::default_config();
            cfg.engine = spec_with(
                &cfg.engine,
                base.clone(),
                json!({ "vAngle": v_angle, "rpm": 4000, "throttle": 0.9, "freeRunning": false }),
            );
            cfg.pipe = vec![pipe(0.4, 0.042)];
            cfg.collector = vec![pipe(0.6, 0.06)];
            // A collector system: these tests are about the junction a set of runners merges at.
            cfg.graph = Some(compile_collector_layout(&cfg.engine, &cfg.pipe, &cfg.collector));
            let mut sim = EngineSim::new(FS, &cfg);
            sim.render(FS as usize / 2);
            sim.render(FS as usize / 2)
        };
        let a = render(if twin { 45.0 } else { 90.0 });
        let b = render(60.0);
        let mut diff = 0.0;
        let mut energy = 0.0;
        for (&x, &y) in a.iter().zip(&b) {
            diff += (x as f64 - y as f64).powi(2);
            energy += (x as f64).powi(2);
        }
        // A tenth of the signal's own energy is far more than drift; inert would be exactly zero.
        let rel = (diff / f64::max(energy, 1e-30)).sqrt();
        assert!(rel > 0.1, "relative difference {rel}");
    }

    /// V8 crossplane sounds different at a different vee
    #[test]
    fn v8_crossplane_sounds_different_at_a_different_vee() {
        sounds_different_at_a_different_vee(
            json!({ "cylinders": 8, "exhaustLayout": "perBank", "crankType": "crossplane" }),
        );
    }

    /// V8 flatplane sounds different at a different vee
    #[test]
    fn v8_flatplane_sounds_different_at_a_different_vee() {
        sounds_different_at_a_different_vee(
            json!({ "cylinders": 8, "exhaustLayout": "perBank", "crankType": "flatplane" }),
        );
    }

    /// V-twin sounds different at a different vee
    #[test]
    fn v_twin_sounds_different_at_a_different_vee() {
        sounds_different_at_a_different_vee(json!({ "cylinders": 2, "exhaustLayout": "2into1", "firingOffset": null }));
    }
}

/// Noise must not heat the duct it is stirring.
///
/// The failure guarded against is an energy rectifier in the valve source. A mass exchange at a
/// boundary trades *stagnation enthalpy*, because gas crossing an orifice does flow work on whatever
/// it moves into. A reverse branch that took only `e + u^2/2` back out while the forward branch put
/// `h + u^2/2` in would deposit `R T` per unit mass on every in-and-out pair: about 430 kJ/kg at
/// 1500 K, against 45 kJ/kg of kinetic energy being accounted for. Throat turbulence, valve-seat
/// pulses and collector merge noise are all zero-mean sources that would feed it, and because the
/// source divides by the first cell's volume the smallest ducts would heat fastest.
///
/// Asserted at engine level rather than by driving a bare duct with a synthetic square wave. That
/// looks like the tighter test and is not: the source is capped against the first cell's density
/// and floored for admissibility, so a large synthetic amplitude measures those guards rather than
/// the enthalpy asymmetry, and the two are hard to separate afterwards. What matters is that a real
/// engine with its noise sources at full scale stays at a physical temperature.
mod noise_does_not_pump_energy_into_the_exhaust {
    use super::*;

    fn stays_physical(length: f64) {
        let engine = common::with(&v8_spec(), json!({ "throatNoise": 1, "mechNoise": 1 }));
        let cfg = collector_config(&engine, primary(), vec![cone(length, 0.042, 0.13)], 8500.0);
        let mut sim = EngineSim::new(FS, &cfg);
        sim.render(FS as usize * 2);

        let ducts = ducts(&sim);
        let max_t = max_temperature(&ducts);
        // Exhaust leaves a cylinder near 1200-1600 K and only cools from there. An enthalpy
        // rectifier would drive this geometry past 40,000 K.
        assert!(max_t < 2500.0, "max T {max_t}");
        assert_eq!(recoveries(&ducts), 0);
    }

    /// tiny collector, noise at full scale
    #[test]
    fn tiny_collector_noise_at_full_scale() {
        stays_physical(0.2);
    }

    /// short collector, noise at full scale
    #[test]
    fn short_collector_noise_at_full_scale() {
        stays_physical(0.35);
    }
}

/// A junction must pass what it receives, and the solver must stay inside its cost budget.
///
/// The node solves for a common pressure by linearising the returning wave, then each branch
/// computes its own nonlinear flux from it; nothing makes those agree on their own, and on every
/// layout they can disagree about the mass crossing the node by tens of percent at peak. A Newton
/// step on the *nonlinear* residual, using the closed-form slope the linear model already provides,
/// brings it to a few percent.
mod junctions_conserve_mass_and_the_grid_stays_affordable {
    use super::*;

    fn stays_balanced_and_affordable(collector: Vec<PipeSegment>) {
        let cfg = collector_config(&v8_spec(), primary(), collector, 8500.0);
        let mut sim = EngineSim::new(FS, &cfg);
        sim.render(FS as usize);

        let ducts = ducts(&sim);
        let substeps = max_substeps(&ducts);
        let cells = cells(&ducts);

        // Peak imbalance, not mean: a node has no volume, so a large transient error is still mass
        // from nowhere. The worst of these geometries peaks at 12%: at 8500 rpm the burn runs late
        // enough that some of each charge leaves unburned and afterfires in the primaries, and the
        // blowdown pulses and bangs reaching the node are the hardest the engine makes.
        let residual = sim.pipe_solver().junction_residual;
        assert!(residual < 0.13, "junction residual {residual}");

        // One substep, always.
        //
        // A V8 can afford a grid coarse enough for a single CFL substep at every duct length, and the
        // cost budget must find it. A grid a couple of millimetres finer than the one substep needs
        // pays two substeps for a few percent of resolution, which puts the preset at about a whole
        // core. Above 100% the audio thread cannot deliver at all, so the sound would cut out at high
        // rpm and come back as the revs fell.
        assert_eq!(substeps, 1);
        assert!((cells * substeps) as f64 <= budget_of(&cfg), "{} cells over {}", cells * substeps, budget_of(&cfg));
        // And nothing degenerate along the way.
        assert_eq!(recoveries(&ducts), 0);
        assert_eq!(clamps(&ducts), 0);
    }

    /// V8 with cone 42->130 over 0.2 m stays balanced and affordable
    #[test]
    fn v8_with_cone_42_130_over_0_2_m_stays_balanced_and_affordable() {
        stays_balanced_and_affordable(vec![cone(0.2, 0.042, 0.13)]);
    }

    /// V8 with cone 42->90 over 0.5 m stays balanced and affordable
    #[test]
    fn v8_with_cone_42_90_over_0_5_m_stays_balanced_and_affordable() {
        stays_balanced_and_affordable(vec![cone(0.5, 0.042, 0.09)]);
    }

    /// V8 with cone 50->60 over 1.2 m stays balanced and affordable
    #[test]
    fn v8_with_cone_50_60_over_1_2_m_stays_balanced_and_affordable() {
        stays_balanced_and_affordable(vec![cone(1.2, 0.05, 0.06)]);
    }

    /// V8 with chamber 42->130 + pipe stays balanced and affordable
    #[test]
    fn v8_with_chamber_42_130_plus_pipe_stays_balanced_and_affordable() {
        stays_balanced_and_affordable(vec![chamber(0.34, 0.042, 0.13), pipe(0.3, 0.04)]);
    }

    /// Coarsening for cost must never make the duct *more* expensive than a coarser choice would.
    #[test]
    fn never_picks_a_grid_a_coarser_one_would_beat() {
        for l in [0.15, 0.25, 0.4, 0.6, 0.9, 1.4] {
            let cfg = collector_config(&v8_spec(), primary(), vec![cone(l, 0.042, 0.09)], 6500.0);
            let sim = EngineSim::new(FS, &cfg);
            let ducts = ducts(&sim);
            let cell_steps = cells(&ducts) * max_substeps(&ducts);
            assert!(cell_steps as f64 <= budget_of(&cfg), "length {l}: {cell_steps} over {}", budget_of(&cfg));
        }
    }
}

/// The exhausts every preset compiles stay solvable and affordable: the headers most have, and the
/// manifold the twin-turbo six keeps.
///
/// Each junction on a manifold takes the full blowdown from a stub a few centimetres away, where a
/// collector's half-metre runners would spread it out, so these are the hardest junctions the solver sees.
/// Their peak imbalance is larger than a collector's — it spikes where flow through a junction reverses —
/// so what is held here is what matters for the sound: nothing blows up, nothing is clamped, no duct
/// needs recovering, the gas stays at physical temperatures, and the grid stays inside its budget.
mod presets_stay_solvable_and_affordable {
    use super::*;

    /// Below its rev limit, where the dyno holds it; the limiter, with its hysteresis, starts above.
    const UNDER_LIMIT_RPM: f64 = 200.0;

    /// Each preset at full throttle and `rpm` of its config's own for half a second: its ducts at the
    /// end, and the hottest any of their gas was on the way.
    fn run(preset: &common::EnginePreset, rpm: impl Fn(f64) -> f64) -> (EngineSim, EngineConfig, f64) {
        let mut cfg = common::default_config();
        let held = rpm(preset.config.engine.rev_limit);
        cfg.engine = common::with(&preset.config.engine, json!({ "rpm": held, "throttle": 1, "freeRunning": false }));
        cfg.pipe = preset.config.pipe.clone();
        cfg.collector = preset_collector(preset).unwrap_or_default();
        let mut sim = EngineSim::new(FS, &cfg);
        let mut hottest = 0.0f64;
        for _ in 0..FS as usize / 2 {
            let out = sim.render(1);
            assert!(out.iter().all(|v| v.is_finite()), "{}", preset.name);
            hottest = hottest.max(max_temperature(&ducts(&sim)));
        }
        (sim, cfg, hottest)
    }

    /// %s at full throttle, held at up to 8000 rpm, below its rev limit
    #[test]
    fn at_full_throttle_up_to_8000_rpm() {
        common::sweep(|preset| {
            let name = &preset.name;
            let (sim, cfg, hottest) = run(preset, |limit| f64::min(8000.0, limit - UNDER_LIMIT_RPM));
            // Exhaust leaves a cylinder near 1200-1800 K and only cools from there.
            assert!(hottest < 2500.0, "{name}: max T {hottest}");

            let ducts = ducts(&sim);
            assert_eq!(recoveries(&ducts), 0, "{name}");
            assert_eq!(clamps(&ducts), 0, "{name}");

            // Inside the cost budget. A V8 has only just enough for one substep, and must get it; smaller
            // engines are allowed the finer grid they can afford.
            let substeps = max_substeps(&ducts);
            let cells = cells(&ducts);
            if cfg.engine.cylinders == 8 {
                assert_eq!(substeps, 1, "{name}");
            }
            assert!(
                (cells * substeps) as f64 <= budget_of(&cfg),
                "{name}: {} over {}",
                cells * substeps,
                budget_of(&cfg)
            );
        });
    }

    /// %s at full throttle on its rev limiter
    ///
    /// Asked for more than its limit, the engine runs free on the limiter, and each cut leaves unburnt
    /// charge in the pipe to burn there. A burn heats the gas it is in to at most 2600 K, about the
    /// adiabatic flame temperature, and a pressure wave reaching it squeezes it hotter still. A pocket
    /// that burns in the trough between two waves can be squeezed hardest: the 2GR's burns at 51 kPa
    /// and its next wave brings it to 154 kPa, three times the pressure, which is about 3400 K.
    #[test]
    fn on_the_rev_limiter() {
        common::sweep(|preset| {
            let name = &preset.name;
            let (sim, _, hottest) = run(preset, |limit| limit + 1000.0);
            assert!(hottest < 3500.0, "{name}: max T {hottest}");
            let ducts = ducts(&sim);
            assert_eq!(recoveries(&ducts), 0, "{name}");
            assert_eq!(clamps(&ducts), 0, "{name}");
        });
    }
}

/// A manifold drawn by hand along a three: each cylinder's pipe bent into a pipe of the same bore that
/// carries on past it, three junctions in a row a few centimetres apart, all of one runner's bore, so the
/// flow runs near the speed of sound. At 32 kHz the links between the junctions are a cell long.
fn hand_drawn_three() -> EngineConfig {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/drawn_inline3.json");
    serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap()
}

/// The hand-drawn three at `fs` on full throttle for two seconds: its top rpm, and its note's level from
/// 0.9 s to 1.3 s, as it revs through the top half of its range.
fn rev_hand_drawn_three(fs: f64) -> (EngineSim, f64, f64) {
    let cfg = hand_drawn_three();
    let mut sim = EngineSim::new(fs, &cfg);
    sim.render((fs / 2.0) as usize);
    sim.set_controls(1.0, 0.0);
    let (mut top, mut sq, mut count) = (0.0_f64, 0.0, 0usize);
    let tenth = (fs / 10.0) as usize;
    for step in 0..20 {
        for _ in 0..tenth {
            let v = sim.render(1)[0] as f64;
            if (9..13).contains(&step) {
                sq += v * v;
                count += 1;
            }
        }
        top = top.max(sim.snapshot().rpm);
    }
    (sim, top, (sq / count as f64).sqrt())
}

/// Revved hard, gas in a link comes to race back into a junction faster than sound, from well below the
/// junction's pressure. It meets a shock there: passed through as though a choked end could not feel
/// the junction, it would pour in without end, lock the pipes behind the junction at six atmospheres,
/// pull those past it to a partial vacuum, and kill the note and the pull at 4000 rpm. With the shock
/// the engine revs to its limiter. And the junction fills the pipe after it no faster than sound,
/// choked, rather than at whatever speed the acoustic estimate asks, so it is never clamped at all.
#[test]
fn a_hand_drawn_manifold_revs_to_its_limiter_without_locking_a_junction() {
    let limit = hand_drawn_three().engine.rev_limit;
    let (sim, top, _) = rev_hand_drawn_three(32000.0);
    assert!(top > 0.95 * limit, "revved to {top:.0} rpm of a {limit:.0} rpm limit");
    let ducts = ducts(&sim);
    assert_eq!(recoveries(&ducts), 0);
    assert_eq!(ducts.iter().map(|d| d.junction_clamps).sum::<u64>(), 0, "junction clamps");
    for (k, d) in ducts.iter().enumerate() {
        let mean = (0..d.n).map(|i| d.pressure_at(i)).sum::<f64>() / d.n as f64;
        assert!(mean < 3e5, "duct {k} held at {:.0} kPa", mean / 1000.0);
    }
}

/// And it sounds the same at 32 kHz as at 48: filled choked rather than clamped, the flow out of the last
/// junction is no louder or rougher on the coarser grid.
#[test]
fn a_hand_drawn_manifold_sounds_the_same_at_32_khz_as_at_48() {
    let (_, _, fine) = rev_hand_drawn_three(48000.0);
    let (_, _, coarse) = rev_hand_drawn_three(32000.0);
    assert!((coarse / fine - 1.0).abs() < 0.25, "{coarse:.4} at 32 kHz against {fine:.4} at 48");
}

/// A V6 drawn with each bank's pipes ending in open air a few centimetres past its last junction, one of
/// them only two cells long. Launched, the pulse through that short pipe leaves it faster than sound and
/// expands below the air's pressure: let through as though the air could not push back, the jet would
/// pull the whole bank to a vacuum within a few cycles and the note would stop. Against the shock it
/// meets, the engine launches and keeps its note.
#[test]
fn a_short_open_ended_pipe_does_not_drain_its_bank_on_a_launch() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/drawn_v6_open_ends.json");
    let cfg: EngineConfig = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    let preset = common::engine_preset("V6, Toyota 2GR");
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(FS as usize / 2);
    sim.start_launch(preset.launch.clone());
    let block = (FS / 50.0) as usize;
    for i in 0..(3 * 50) {
        let level = common::rms(&sim.render(block));
        assert!(i < 25 || level > 1e-3, "silent at {:.2} s into the launch", i as f64 / 50.0);
    }
    let ducts = ducts(&sim);
    assert_eq!(recoveries(&ducts), 0);
    assert_eq!(clamps(&ducts), 0, "junction clamps");
}

/// A V6 with all six pipes into one junction and out through a tailpipe 15 cm long, narrowing to 32 mm:
/// four cells, and a nozzle at its end. At idle and revving down with the throttle shut, the cylinders draw exhaust back through
/// their valves, and air in at the tailpipe after it. Let in at the outside air's pressure with its speed
/// on top, each sample would draw in faster than the last, pump the exhaust up to tens of bar, and click
/// at full scale. Drawn in as through a nozzle, from the outside air at rest, it idles and revs down
/// without a click, from idle at a shut throttle or off the limiter once the throttle has shut. (On the
/// limiter itself it pops, with the steep fronts pops have.)
#[test]
fn air_drawn_in_at_a_short_tailpipe_does_not_pump_the_exhaust_up() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/drawn_v6_short_tailpipe.json");
    let cfg: EngineConfig = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    for (name, rev) in [("idle at a shut throttle", false), ("revving down off the limiter", true)] {
        let mut sim = EngineSim::new(FS, &cfg);
        let (mut prev, mut step, mut top) = (0.0_f32, 0.0_f32, 0.0_f64);
        let block = (FS / 50.0) as usize;
        for i in 0..(8 * 50) {
            let t = i as f64 / 50.0;
            sim.set_controls(if rev && (0.5..3.0).contains(&t) { 1.0 } else { 0.0 }, 0.0);
            let shut = !rev || t > 3.0;
            for v in sim.render(block) {
                if shut {
                    step = step.max((v - prev).abs());
                }
                prev = v;
            }
            if shut {
                top = ducts(&sim).iter().flat_map(|d| (0..d.n).map(|k| d.pressure_at(k))).fold(top, f64::max);
            }
        }
        let ducts = ducts(&sim);
        assert_eq!(recoveries(&ducts), 0, "{name}");
        assert!(top < 4e5, "{name}: the exhaust reached {:.0} kPa", top / 1000.0);
        assert!(step < 0.1, "{name}: a step of {step:.3} between samples");
    }
}

/// A tailpipe that narrows faster at its end than its four cells can follow ends in a nozzle of the
/// size drawn. Grid and all, a 32 mm outlet and a 12 mm one would be the same 35 mm opening; through the
/// nozzle, a 12 mm hole on a 3.5 litre V6 chokes at full throttle, holds the exhaust at several bar, and
/// leaves it well under half its torque at 4000 rpm.
#[test]
fn a_tailpipe_narrower_than_its_grid_ends_in_a_nozzle_of_the_size_drawn() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/drawn_v6_short_tailpipe.json");
    let base: EngineConfig = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    let pull = |d_out: f64| {
        let mut cfg = base.clone();
        let graph = cfg.graph.as_mut().unwrap();
        graph.ducts.iter_mut().find(|d| d.id == "collector1").unwrap().segments[0].d_out = d_out;
        cfg.engine = common::with(
            &cfg.engine,
            json!({ "freeRunning": false, "rpm": 4000, "throttle": 1, "combustionVariability": 0 }),
        );
        let mut sim = EngineSim::new(FS, &cfg);
        sim.render(FS as usize);
        let (mut torque, mut inlet) = (0.0, 0.0);
        let n = FS as usize / 4;
        for _ in 0..n {
            sim.render(1);
            torque += sim.snapshot().torque;
            inlet += sim.pipe_solver().collectors()[0].pressure_at(0);
        }
        assert_eq!(sim.pipe_solver().recoveries(), 0, "{d_out} m outlet");
        (torque / n as f64, inlet / n as f64)
    };
    let (open, open_p) = pull(0.047);
    let (medium, _) = pull(0.032);
    let (hole, hole_p) = pull(0.012);
    assert!(medium < open, "{medium:.0} N m through 32 mm, against {open:.0} through 47");
    assert!(hole < 0.5 * open, "{hole:.0} N m through 12 mm, against {open:.0} through 47");
    assert!(hole < medium - 50.0, "{hole:.0} N m through 12 mm, against {medium:.0} through 32");
    assert!(
        hole_p > 4e5 && open_p < 1.5e5,
        "collector at {:.0} kPa through 12 mm, {:.0} through 47",
        hole_p / 1e3,
        open_p / 1e3
    );
}
