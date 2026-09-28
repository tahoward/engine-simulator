//! The exhaust graph: that it reproduces the primaries-and-collectors topology exactly, that it
//! rejects nonsense, and that it can express and solve arrangements that topology cannot.
//!
//! The last part is the point of a graph. "N identical primaries plus M collectors" cannot describe
//! a tri-Y, a branch part way along a duct, or runners of different lengths; a graph can, and the
//! solver has to actually run them rather than merely accept them.
//!
//! The graph editing the UI does stays in the web app and is tested there.

mod common;

use common::{EnginePreset, FS};
use engine_sim::EngineSim;
use engine_sim::euler_pipe::{EulerPipe, EulerPipeOptions};
use engine_sim::exhaust_graph::{
    DuctRole, DuctSink, DuctSource, End, ExhaustDuct, ExhaustGraph, compile_collector_layout, compile_layout, ends_at,
    node_order, path_to_air, radiating_ducts, validate_graph, valve_ducts,
};
use engine_sim::exhaust_system::ExhaustSystem;
use engine_sim::spec::{
    EngineConfig, EngineSpec, ExhaustLayout, PipeSegment, SegmentKind, SegmentPartial, collector_groups, copy_segment,
    displacement, exhaust_layout_of, exhaust_port_diameter, gas, make_segment,
};
use serde_json::{Value, json};

/// `defaultConfig().engine` with the fields in `partial` over it.
fn spec_of(partial: Value) -> EngineSpec {
    common::with(&common::default_config().engine, partial)
}

/// A segment of `length` and inlet diameter `d_in`, either left to its default.
fn seg(length: Option<f64>, d_in: Option<f64>) -> PipeSegment {
    make_segment(SegmentPartial { length, d_in, ..Default::default() })
}

fn pipe(length: f64, d_in: f64) -> PipeSegment {
    seg(Some(length), Some(d_in))
}

fn of_length(length: f64) -> PipeSegment {
    seg(Some(length), None)
}

fn any_segment() -> PipeSegment {
    seg(None, None)
}

fn valve(cylinder: i64) -> DuctSource {
    DuctSource::Valve { cylinder }
}

fn from_node(node: &str) -> DuctSource {
    DuctSource::Node { node: node.into() }
}

fn to_node(node: &str) -> DuctSink {
    DuctSink::Node { node: node.into() }
}

fn duct(id: &str, segments: Vec<PipeSegment>, from: DuctSource, to: DuctSink) -> ExhaustDuct {
    ExhaustDuct {
        id: id.to_string(),
        segments,
        from,
        to,
        heading_yaw: None,
        heading_pitch: None,
        heading_frame: None,
        continues: None,
        role: None,
    }
}

fn ids(graph: &ExhaustGraph, ducts: &[usize]) -> Vec<String> {
    ducts.iter().map(|&i| graph.ducts[i].id.clone()).collect()
}

/// The preset's own collector, or `None` where it has none and the app falls back to the default.
///
/// The fixture's configs fill a missing collector in with `defaultCollector()`, so a collector of
/// exactly that geometry is one the preset did not draw.
fn preset_collector(preset: &EnginePreset) -> Option<Vec<PipeSegment>> {
    let shape = |s: &[PipeSegment]| -> Vec<(SegmentKind, f64, f64, f64)> {
        s.iter().map(|g| (g.kind, g.length, g.d_in, g.d_out)).collect()
    };
    let own = &preset.config.collector;
    if shape(own) == shape(&common::presets().default_collector) { None } else { Some(own.clone()) }
}

/// The web app's `fittedExhaust`: a header runner bored for the valve and, where the layout merges, a
/// constant-velocity collector with a silencer can. `(pipe, collector)`.
fn fitted_exhaust(spec: &EngineSpec) -> (Vec<PipeSegment>, Vec<PipeSegment>) {
    let s = |kind: SegmentKind, length: f64, d_in: f64, d_out: f64, yaw: f64| {
        make_segment(SegmentPartial {
            kind: Some(kind),
            length: Some(length),
            d_in: Some(d_in),
            d_out: Some(d_out),
            yaw: Some(yaw),
            ..Default::default()
        })
    };
    let layout = exhaust_layout_of(spec);
    let groups = collector_groups(spec);
    let collector_count = groups.iter().fold(0, |max, &g| i32::max(max, g + 1));
    let per_collector = if collector_count > 0 { spec.cylinders as f64 / collector_count as f64 } else { 1.0 };

    let d_primary = f64::max(0.85 * exhaust_port_diameter(spec), 0.02);
    let primary_length = if layout == ExhaustLayout::Open { 0.75 } else { 0.45 };
    let mut pipe = vec![s(SegmentKind::Pipe, primary_length, d_primary, d_primary, 0.0)];
    if layout == ExhaustLayout::Open {
        pipe.push(s(SegmentKind::Cone, 0.25, d_primary, d_primary * 1.7, 0.0));
        return (pipe, Vec::new());
    }

    let d_collector = d_primary * per_collector.sqrt() * 0.92;
    let served_disp = displacement(spec) * per_collector;
    let can_dia = f64::min(d_collector * 2.5, 0.2);
    let can_area = (std::f64::consts::PI * can_dia * can_dia) / 4.0;
    let can_length = f64::min(f64::max((8.0 * served_disp) / can_area, 0.25), 0.6);
    let run_length = f64::max(2.2 - primary_length - can_length - 0.5, 0.35);
    let collector = vec![
        s(SegmentKind::Cone, 0.16, d_primary * 1.25, d_collector, 0.0),
        s(SegmentKind::Pipe, run_length, d_collector, d_collector, 0.2),
        s(SegmentKind::Chamber, can_length, d_collector, can_dia, 0.0),
        s(SegmentKind::Pipe, 0.5, d_collector, d_collector, 0.0),
    ];
    (pipe, collector)
}

fn v8_spec() -> EngineSpec {
    spec_of(json!({ "cylinders": 8, "vAngle": 90, "crankType": "crossplane", "exhaustLayout": "perBank" }))
}

/// Primaries then collectors: every duct in the system.
fn ducts(sim: &EngineSim) -> Vec<&EulerPipe> {
    let sys = sim.pipe_solver();
    sys.primaries().iter().chain(sys.collectors()).collect()
}

mod compile_collector_layout_reproduces_the_primaries_and_collectors_layout {
    use super::*;

    /// %s, for every engine preset
    #[test]
    fn every_preset() {
        for preset in &common::presets().engine_presets {
            let name = &preset.name;
            let spec = &preset.config.engine;
            let cylinders = spec.cylinders as usize;
            let graph =
                compile_collector_layout(spec, &preset.config.pipe, &preset_collector(preset).unwrap_or_default());
            let groups = collector_groups(spec);

            assert_eq!(validate_graph(&graph, cylinders), Vec::<String>::new(), "{name}");

            // One duct per cylinder valve, and it goes where the grouping says.
            let valves = valve_ducts(&graph, cylinders);
            assert!(valves.iter().all(|d| d.is_some()), "{name}");
            for (cylinder, &group) in groups.iter().enumerate() {
                let duct = &graph.ducts[valves[cylinder].unwrap()];
                if group < 0 {
                    assert_eq!(duct.to, DuctSink::Mouth, "{name}");
                } else {
                    assert_eq!(duct.to, to_node(&format!("merge{group}")), "{name}");
                }
            }

            // A node per group actually used, and nothing else.
            let mut used: Vec<i32> = groups.iter().copied().filter(|&g| g >= 0).collect();
            used.sort();
            used.dedup();
            let expected: Vec<String> = used.iter().map(|g| format!("merge{g}")).collect();
            assert_eq!(node_order(&graph), expected, "{name}");

            // Each node joins its members plus exactly one duct leaving it.
            for &g in &used {
                let ends = ends_at(&graph, &format!("merge{g}"));
                let upstream = ends.iter().filter(|e| e.1 == End::Outlet).count();
                let downstream = ends.iter().filter(|e| e.1 == End::Inlet).count();
                assert_eq!(upstream, groups.iter().filter(|&&x| x == g).count(), "{name}");
                assert_eq!(downstream, 1, "{name}");
            }
        }
    }

    /// radiates collectors before runners that vent alone
    ///
    /// Mouth order is load-bearing, not cosmetic: `refresh_mouth_paths` lays the mouths out in a line by
    /// index and gives each its own delay and gain, so reordering them changes the sound. Collectors
    /// come before solo runners.
    #[test]
    fn radiates_collectors_before_runners_that_vent_alone() {
        let merged = compile_layout(&v8_spec(), &[of_length(0.4)], &[of_length(0.6)]);
        assert_eq!(ids(&merged, &radiating_ducts(&merged)), ["collector0", "collector1"]);

        let open =
            compile_layout(&spec_of(json!({ "cylinders": 2, "exhaustLayout": "2into2" })), &[any_segment()], &[]);
        assert_eq!(ids(&open, &radiating_ducts(&open)), ["runner0", "runner1"]);
    }

    /// keeps a collector even when nothing was drawn for it
    ///
    /// A group whose collector geometry is empty still gets a duct, or the sound would change.
    #[test]
    fn keeps_a_collector_even_when_nothing_was_drawn_for_it() {
        let spec = spec_of(json!({ "cylinders": 2, "exhaustLayout": "2into1" }));
        let graph = compile_layout(&spec, &[any_segment()], &[]);
        assert!(graph.ducts.iter().any(|d| d.id == "collector0"));
        assert_eq!(validate_graph(&graph, 2), Vec::<String>::new());
    }

    /// gives every duct its own segments
    ///
    /// Copies, not shared arrays — which is what later lets one runner be lengthened alone.
    #[test]
    fn gives_every_duct_its_own_segments() {
        let pipe = [of_length(0.4)];
        let mut graph = compile_layout(&spec_of(json!({ "cylinders": 2, "exhaustLayout": "2into1" })), &pipe, &[]);
        assert_eq!(graph.ducts[0].segments[0].length, graph.ducts[1].segments[0].length);
        graph.ducts[0].segments[0].length = 0.9;
        assert_eq!(graph.ducts[1].segments[0].length, 0.4);
    }
}

mod validate_graph_catches_what_a_half_drawn_route_leaves_behind {
    use super::*;

    fn runner(id: &str, cylinder: i64, to: DuctSink) -> ExhaustDuct {
        duct(id, vec![of_length(0.4)], valve(cylinder), to)
    }

    fn problems(graph: &ExhaustGraph, cylinders: usize) -> String {
        validate_graph(graph, cylinders).join(" ")
    }

    /// a cylinder with no pipe
    #[test]
    fn a_cylinder_with_no_pipe() {
        let graph = ExhaustGraph { ducts: vec![runner("a", 0, DuctSink::Mouth)], ..Default::default() };
        assert!(problems(&graph, 2).contains("cylinder 2 has no exhaust pipe"), "{}", problems(&graph, 2));
    }

    /// two pipes on one port
    #[test]
    fn two_pipes_on_one_port() {
        let graph = ExhaustGraph {
            ducts: vec![runner("a", 0, DuctSink::Mouth), runner("b", 0, DuctSink::Mouth)],
            ..Default::default()
        };
        assert!(problems(&graph, 1).contains("has 2 pipes on its exhaust port"), "{}", problems(&graph, 1));
    }

    /// a junction joining only one pipe
    #[test]
    fn a_junction_joining_only_one_pipe() {
        let graph = ExhaustGraph { ducts: vec![runner("a", 0, to_node("x"))], ..Default::default() };
        let problems = problems(&graph, 1);
        assert!(problems.contains("joins only one pipe"), "{problems}");
        assert!(problems.contains("no pipe leaving it"), "{problems}");
    }

    /// a duct connected to no cylinder
    #[test]
    fn a_duct_connected_to_no_cylinder() {
        let graph = ExhaustGraph {
            ducts: vec![
                runner("a", 0, DuctSink::Mouth),
                duct("orphan", vec![any_segment()], from_node("nowhere"), DuctSink::Mouth),
            ],
            ..Default::default()
        };
        let problems = problems(&graph, 1);
        assert!(problems.contains("\"orphan\" is not connected to any cylinder"), "{problems}");
    }

    /// duplicate ids
    #[test]
    fn duplicate_ids() {
        let graph = ExhaustGraph {
            ducts: vec![runner("same", 0, DuctSink::Mouth), runner("same", 1, DuctSink::Mouth)],
            ..Default::default()
        };
        assert!(problems(&graph, 2).contains("two ducts share the id"), "{}", problems(&graph, 2));
    }

    /// and the solver refuses to build one
    #[test]
    fn and_the_solver_refuses_to_build_one() {
        let graph = ExhaustGraph { ducts: vec![runner("a", 0, to_node("x"))], ..Default::default() };
        match ExhaustSystem::new(&graph, 1, FS, gas::T_AMB, &EulerPipeOptions::default()) {
            Ok(_) => panic!("built an exhaust from a graph that cannot be solved"),
            Err(e) => assert!(e.contains("cannot be solved"), "{e}"),
        }
    }
}

/// Topologies a fixed set of primaries and collectors has no way to describe.
///
/// A tri-Y merges in two stages, so its middle ducts are fed by a junction *and* feed another one —
/// something `primaries`/`collectors` alone cannot express, where a collector is always the last
/// duct. Unequal runners would need a second segment array.
mod arrangements_primaries_and_collectors_cannot_express {
    use super::*;

    /// 4 cylinders, paired into two intermediate ducts, merged again into one tailpipe.
    fn tri_y() -> ExhaustGraph {
        let runner = |i: i64, node: &str, length: f64| {
            duct(&format!("runner{i}"), vec![pipe(length, 0.038)], valve(i), to_node(node))
        };
        ExhaustGraph {
            ducts: vec![
                runner(0, "pairA", 0.4),
                runner(1, "pairB", 0.4),
                runner(2, "pairA", 0.4),
                runner(3, "pairB", 0.4),
                duct("midA", vec![pipe(0.3, 0.05)], from_node("pairA"), to_node("tail")),
                duct("midB", vec![pipe(0.3, 0.05)], from_node("pairB"), to_node("tail")),
                duct("tailpipe", vec![pipe(0.5, 0.06)], from_node("tail"), DuctSink::Mouth),
            ],
            ..Default::default()
        }
    }

    fn four(graph: ExhaustGraph) -> EngineSim {
        let mut cfg = common::default_config();
        cfg.engine = common::with(
            &cfg.engine,
            json!({ "cylinders": 4, "exhaustLayout": "2into1", "rpm": 4000, "throttle": 1, "freeRunning": false }),
        );
        EngineSim::with_options(FS, &cfg, EulerPipeOptions::default(), Some(graph))
    }

    /// a tri-Y validates, and its middle ducts are both fed and feeding
    #[test]
    fn a_tri_y_validates_and_its_middle_ducts_are_both_fed_and_feeding() {
        let graph = tri_y();
        assert_eq!(validate_graph(&graph, 4), Vec::<String>::new());
        assert_eq!(node_order(&graph), ["pairA", "pairB", "tail"]);
        assert_eq!(ids(&graph, &radiating_ducts(&graph)), ["tailpipe"]);
        let mut ends: Vec<&str> = ends_at(&graph, "tail")
            .iter()
            .map(|e| match e.1 {
                End::Inlet => "inlet",
                End::Outlet => "outlet",
            })
            .collect();
        ends.sort();
        assert_eq!(ends, ["inlet", "outlet", "outlet"]);
    }

    /// a tri-Y runs, stays finite and makes a sound
    #[test]
    fn a_tri_y_runs_stays_finite_and_makes_a_sound() {
        let mut sim = four(tri_y());
        sim.render(FS as usize / 2);
        let audio = sim.render(FS as usize / 2);

        let mut peak = 0.0f32;
        for v in &audio {
            assert!(v.is_finite());
            peak = peak.max(v.abs());
        }
        assert!(peak > 1e-3, "peak {peak}");

        let ducts = ducts(&sim);
        assert_eq!(ducts.len(), 7);
        assert_eq!(ducts.iter().map(|d| d.recoveries).sum::<u64>(), 0);
        assert_eq!(ducts.iter().map(|d| d.junction_clamps).sum::<u64>(), 0);
        let mut max_t = 0.0f64;
        for d in &ducts {
            for k in 0..d.n {
                max_t = max_t.max(d.temperature_at(k));
            }
        }
        assert!(max_t < 2500.0, "max T {max_t}");
    }

    /// unequal runners change the sound
    #[test]
    fn unequal_runners_change_the_sound() {
        let render = |lengths: [f64; 4]| {
            let mut graph = tri_y();
            for (i, l) in lengths.into_iter().enumerate() {
                graph.ducts[i].segments[0].length = l;
            }
            let mut sim = four(graph);
            sim.render(FS as usize / 2);
            sim.render(FS as usize / 2)
        };
        let even = render([0.4, 0.4, 0.4, 0.4]);
        let uneven = render([0.3, 0.5, 0.35, 0.45]);

        let mut diff = 0.0;
        let mut energy = 0.0;
        for (&a, &b) in even.iter().zip(&uneven) {
            diff += (a as f64 - b as f64).powi(2);
            energy += (a as f64).powi(2);
        }
        // A tenth of the signal's own energy: far more than drift, and impossible if runners shared one geometry.
        let rel = (diff / f64::max(energy, 1e-30)).sqrt();
        assert!(rel > 0.1, "relative difference {rel}");
    }
}

/// The pieces the panel and the URL rely on.
///
/// `path_to_air` is what "the tuned length" means once the exhaust is a graph — a runner plus whatever it
/// merges into, however many stages that takes. "Primary plus collector" would only describe the layouts
/// that have exactly those two parts.
mod walking_the_graph_for_the_panel_and_the_url {
    use super::*;

    /// a compiled path is the runner plus its collector
    #[test]
    fn a_compiled_path_is_the_runner_plus_its_collector() {
        let spec = spec_of(json!({ "cylinders": 4, "exhaustLayout": "2into1" }));
        let graph = compile_collector_layout(&spec, &[pipe(0.45, 0.038)], &[pipe(0.6, 0.055)]);

        let path = path_to_air(&graph, 0);
        assert_eq!(ids(&graph, &path), ["runner0", "collector0"]);
        let len: f64 = path.iter().map(|&i| graph.ducts[i].segments.iter().map(|s| s.length).sum::<f64>()).sum();
        assert!((len - (0.45 + 0.6)).abs() < 5e-10, "length {len}");
    }

    /// an unmerged runner is the whole path
    #[test]
    fn an_unmerged_runner_is_the_whole_path() {
        let graph =
            compile_layout(&spec_of(json!({ "cylinders": 2, "exhaustLayout": "2into2" })), &[of_length(0.5)], &[]);
        assert_eq!(ids(&graph, &path_to_air(&graph, 1)), ["runner1"]);
    }

    /// a two-stage merge gives a three-duct path
    #[test]
    fn a_two_stage_merge_gives_a_three_duct_path() {
        let graph = ExhaustGraph {
            ducts: vec![
                duct("r0", vec![of_length(0.4)], valve(0), to_node("a")),
                duct("r1", vec![of_length(0.4)], valve(1), to_node("a")),
                duct("mid", vec![of_length(0.3)], from_node("a"), to_node("b")),
                duct("r2", vec![of_length(0.4)], valve(2), to_node("b")),
                duct("tail", vec![of_length(0.5)], from_node("b"), DuctSink::Mouth),
            ],
            ..Default::default()
        };
        assert_eq!(ids(&graph, &path_to_air(&graph, 0)), ["r0", "mid", "tail"]);
        assert_eq!(ids(&graph, &path_to_air(&graph, 2)), ["r2", "tail"]);
    }

    /// does not loop forever on a cycle
    ///
    /// A loop is a drawing mistake; walking it must terminate rather than hang the panel.
    #[test]
    fn does_not_loop_forever_on_a_cycle() {
        let graph = ExhaustGraph {
            ducts: vec![
                duct("r0", vec![any_segment()], valve(0), to_node("a")),
                duct("x", vec![any_segment()], from_node("a"), to_node("b")),
                duct("y", vec![any_segment()], from_node("b"), to_node("a")),
            ],
            ..Default::default()
        };
        assert!(path_to_air(&graph, 0).len() <= 3);
    }

    /// round-trips through JSON and still validates
    ///
    /// A graph survives the URL.
    ///
    /// Saved as base64 JSON in the hash, so it comes back as plain data with no guarantee of being well
    /// formed. Rebuilding each segment through `copy_segment` is what stops a truncated link putting
    /// missing diameters into the solver.
    #[test]
    fn round_trips_through_json_and_still_validates() {
        let mut original = compile_layout(&v8_spec(), &[of_length(0.4)], &[of_length(0.6)]);
        original.ducts[0].heading_yaw = Some(0.3);

        let revived: ExhaustGraph = serde_json::from_str(&serde_json::to_string(&original).unwrap()).unwrap();
        let rebuilt = ExhaustGraph {
            ducts: revived
                .ducts
                .into_iter()
                .map(|d| ExhaustDuct { segments: d.segments.iter().map(copy_segment).collect(), ..d })
                .collect(),
            ..Default::default()
        };

        assert_eq!(validate_graph(&rebuilt, 8), Vec::<String>::new());
        assert!((rebuilt.ducts[0].heading_yaw.unwrap() - 0.3).abs() < 5e-10);
        assert_eq!(node_order(&rebuilt), node_order(&original));
        let ids = |g: &ExhaustGraph| g.ducts.iter().map(|d| d.id.clone()).collect::<Vec<_>>();
        assert_eq!(ids(&rebuilt), ids(&original));
    }

    /// rejects a graph that does not fit the engine
    ///
    /// A graph for the wrong engine is rejected, which is what makes discarding it on load safe.
    #[test]
    fn rejects_a_graph_that_does_not_fit_the_engine() {
        let for_two = compile_layout(
            &spec_of(json!({ "cylinders": 2, "exhaustLayout": "2into1" })),
            &[any_segment()],
            &[any_segment()],
        );
        assert_eq!(validate_graph(&for_two, 2), Vec::<String>::new());
        assert!(!validate_graph(&for_two, 8).is_empty());
    }
}

/// A drawn graph can outlive the engine it was drawn for, and the audio thread must survive that.
///
/// `set_engine` and `set_graph` are separate calls, so switching a V-twin to a V8 rebuilds the exhaust
/// once with the new cylinder count and the *stale* graph before the new one arrives — and that graph has
/// no pipe on cylinders 3 to 8. `ExhaustSystem` rightly refuses to build it, so the rebuild uses the
/// exhaust compiled from the layout instead until a graph that fits arrives.
///
/// Nothing on the audio thread may fail: there is nothing above it to catch anything, and the cost of
/// being wrong is the whole app going quiet.
mod a_stale_graph_does_not_silence_the_engine {
    use super::*;

    fn twin(runner: Vec<PipeSegment>) -> EngineConfig {
        let mut cfg = common::default_config();
        cfg.engine = common::with(
            &cfg.engine,
            json!({
                "cylinders": 2,
                "vAngle": 45,
                "exhaustLayout": "2into1",
                "rpm": 3000,
                "throttle": 0.8,
                "freeRunning": false,
            }),
        );
        cfg.pipe = runner;
        cfg.collector = vec![pipe(0.5, 0.055)];
        cfg.graph = Some(compile_layout(&cfg.engine, &cfg.pipe, &cfg.collector));
        cfg
    }

    fn loudness(audio: &[f32]) -> f32 {
        let mut peak = 0.0f32;
        for v in audio {
            assert!(v.is_finite());
            peak = peak.max(v.abs());
        }
        peak
    }

    fn survives_switching_to_a_v8(runner: Vec<PipeSegment>) {
        let cfg = twin(runner);
        let mut sim = EngineSim::with_options(FS, &cfg, EulerPipeOptions::default(), cfg.graph.clone());
        sim.render(FS as usize / 4);
        // The V-twin itself: a runner with no segments is a short stub, not silence.
        assert!(loudness(&sim.render(FS as usize / 4)) > 1e-4);

        // The order the renderer sends them in: the engine first, the new graph afterwards.
        sim.set_engine(common::with(
            &cfg.engine,
            json!({ "cylinders": 8, "vAngle": 90, "crankType": "crossplane", "exhaustLayout": "perBank" }),
        ));
        assert!(loudness(&sim.render(FS as usize / 4)) > 1e-4);

        // And once the matching graph arrives it is used, with eight runners.
        let graph = compile_layout(sim.engine(), &cfg.pipe, &cfg.collector);
        sim.set_graph(Some(graph));
        assert!(loudness(&sim.render(FS as usize / 4)) > 1e-4);
        assert_eq!(sim.pipe_solver().primaries().len(), 8);
    }

    /// a V-twin with runners drawn makes a sound, and still does after switching to a V8
    #[test]
    fn a_v_twin_with_runners_drawn_makes_a_sound_and_still_does_after_switching_to_a_v8() {
        survives_switching_to_a_v8(vec![pipe(0.4, 0.042)]);
    }

    /// a V-twin with the runners deleted makes a sound, and still does after switching to a V8
    #[test]
    fn a_v_twin_with_the_runners_deleted_makes_a_sound_and_still_does_after_switching_to_a_v8() {
        survives_switching_to_a_v8(Vec::new());
    }

    /// survives shrinking the engine under a larger graph
    ///
    /// Back the other way too: a V8 graph left over on a V-twin.
    #[test]
    fn survives_shrinking_the_engine_under_a_larger_graph() {
        let mut cfg = common::default_config();
        cfg.engine = common::with(
            &cfg.engine,
            json!({
                "cylinders": 8,
                "vAngle": 90,
                "crankType": "crossplane",
                "exhaustLayout": "perBank",
                "rpm": 3000,
                "throttle": 0.8,
                "freeRunning": false,
            }),
        );
        cfg.pipe = vec![pipe(0.4, 0.042)];
        cfg.collector = vec![pipe(0.5, 0.055)];
        let graph = compile_layout(&cfg.engine, &cfg.pipe, &cfg.collector);
        let mut sim = EngineSim::with_options(FS, &cfg, EulerPipeOptions::default(), Some(graph));
        sim.render(FS as usize / 4);
        sim.set_engine(common::with(&cfg.engine, json!({ "cylinders": 2, "vAngle": 45, "exhaustLayout": "2into1" })));
        assert!(loudness(&sim.render(FS as usize / 4)) > 1e-4);
        assert_eq!(sim.pipe_solver().primaries().len(), 2);
    }
}

/// What `compile_layout` builds: a manifold of straight tube along each bank.
///
/// A bank of four is the first cylinder's stub turning onto the manifold, a stub from each of the other
/// three, two further lengths of manifold and the outlet — so four lengths of manifold and outlet in all,
/// and every junction a runner joining the manifold from the side.
mod compile_layout_builds_manifolds {
    use super::*;

    /// a V8 bank is four stubs, two lengths of manifold and an outlet
    #[test]
    fn a_v8_bank_is_four_stubs_two_lengths_of_manifold_and_an_outlet() {
        let graph = compile_layout(&v8_spec(), &[of_length(0.45)], &[of_length(0.6)]);
        assert_eq!(validate_graph(&graph, 8), Vec::<String>::new());
        assert_eq!(graph.ducts.len(), 14);
        for node in node_order(&graph) {
            assert_eq!(ends_at(&graph, &node).len(), 3, "{node}");
        }
        // One runner per bank carries the manifold's first length.
        let carriers: Vec<&ExhaustDuct> = graph
            .ducts
            .iter()
            .filter(|d| {
                graph.ducts.iter().any(|o| o.continues.as_deref() == Some(d.id.as_str()))
                    && d.valve_cylinder().is_some()
            })
            .collect();
        assert_eq!(carriers.len(), 2);
        for c in carriers {
            assert_eq!(c.segments.len(), 2, "{}", c.id);
        }
    }

    /// meets both banks behind the engine for an 8-into-1, keeping the path length
    #[test]
    fn meets_both_banks_behind_the_engine_for_an_8_into_1_keeping_the_path_length() {
        let merged =
            spec_of(json!({ "cylinders": 8, "vAngle": 90, "crankType": "crossplane", "exhaustLayout": "merged" }));
        let collector = [pipe(1.5, 0.07)];
        let graph = compile_layout(&merged, &[of_length(0.45)], &collector);
        assert_eq!(validate_graph(&graph, 8), Vec::<String>::new());
        let downs: Vec<&ExhaustDuct> = graph.ducts.iter().filter(|d| d.id.starts_with("down")).collect();
        assert_eq!(downs.len(), 2);
        // Mirror images but for the stagger between the banks, and the longer came out of the collector.
        let (a, b) = (downs[0].segments[0].length, downs[1].segments[0].length);
        assert!((a - b).abs() < engine_sim::spec::ROD_STAGGER, "{a} {b}");
        let out = graph.ducts.iter().find(|d| d.id == "collector0").unwrap();
        let total = out.segments[0].length + a.max(b);
        assert!((total - 1.5).abs() < 5e-10, "path {total}");
    }
}

/// A manifold widens as it gathers cylinders, and the collector does not pinch it.
///
/// Capped at the collector's inlet — the narrow end of its entry cone — an inline six's manifold would stay
/// at one runner's bore all the way along, and five cylinders' gas would choke through it: the junction at
/// its end clamping on nearly every sample and the gas reaching 2300 K.
mod manifold_sizing {
    use super::*;

    /// widens along the bank and opens into the collector without a step down
    #[test]
    fn widens_along_the_bank_and_opens_into_the_collector_without_a_step_down() {
        let six = spec_of(json!({
            "cylinders": 6,
            "vAngle": 0,
            "exhaustLayout": "merged",
            "bore": 0.082,
            "stroke": 0.0946,
            "exValveDia": 0.031,
        }));
        let (fitted_pipe, fitted_collector) = fitted_exhaust(&six);
        let graph = compile_layout(&six, &fitted_pipe, &fitted_collector);
        let links: Vec<f64> =
            graph.ducts.iter().filter(|d| d.role == Some(DuctRole::Manifold)).map(|d| d.segments[0].d_in).collect();
        for k in 1..links.len() {
            assert!(links[k] > links[k - 1], "{links:?}");
        }
        let last = *links.last().unwrap();
        let collector = graph.ducts.iter().find(|d| d.role == Some(DuctRole::Collector)).unwrap();
        assert!(collector.segments[0].d_in >= last - 1e-12, "{} after {last}", collector.segments[0].d_in);
        // Gathering five of six cylinders: about the bore constant gas speed wants.
        let runner = fitted_pipe[0].d_in;
        assert!((last / runner - 5f64.sqrt() * 0.92).abs() < 5e-7, "{}", last / runner);
    }
}
