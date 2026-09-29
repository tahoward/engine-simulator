//! The exhaust as a graph of ducts joined at nodes, and the layouts compiled into one from the engine.
//!
//! A duct is a list of `PipeSegment`s with something at each end: a cylinder's exhaust valve, a node,
//! or open air. A node is nothing but an id that several duct ends share. A 4-into-1 is four ducts and
//! a collector sharing one node; a tri-Y is two pairs sharing two nodes that feed a third.
//!
//! The graph editing the UI does lives in the web app's `src/model/exhaustGraph.ts`. What is here is
//! what the solver needs: compiling a layout, validating a graph, and walking it.

use serde::{Deserialize, Serialize};

use crate::geometry::{Vec3, distance, exhaust_port_of, sweep_end, turn_between_dirs};
use crate::math;
use crate::spec::{
    BlowOff, EngineSpec, ExhaustLayout, PipeSegment, SegmentKind, SegmentPartial, collector_groups, copy_segment, crank_pins,
    cylinder_spacing, exhaust_layout_of, make_segment, physical_bank, segment_diameter,
};

/// What feeds a duct's inlet.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum DuctSource {
    Valve { cylinder: i64 },
    Node { node: String },
}

/// Where a duct's outlet goes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum DuctSink {
    Node { node: String },
    Mouth,
}

/// What a compiled duct is for.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum DuctRole {
    Runner,
    Stub,
    Manifold,
    Downpipe,
    Collector,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ExhaustDuct {
    pub id: String,
    pub segments: Vec<PipeSegment>,
    pub from: DuctSource,
    pub to: DuctSink,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub heading_yaw: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub heading_pitch: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub heading_frame: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub continues: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub role: Option<DuctRole>,
}

impl ExhaustDuct {
    fn new(id: String, segments: Vec<PipeSegment>, from: DuctSource, to: DuctSink) -> ExhaustDuct {
        ExhaustDuct {
            id,
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

    fn node_from(&self) -> Option<&str> {
        match &self.from {
            DuctSource::Node { node } => Some(node),
            DuctSource::Valve { .. } => None,
        }
    }

    fn node_to(&self) -> Option<&str> {
        match &self.to {
            DuctSink::Node { node } => Some(node),
            DuctSink::Mouth => None,
        }
    }

    pub fn valve_cylinder(&self) -> Option<i64> {
        match self.from {
            DuctSource::Valve { cylinder } => Some(cylinder),
            DuctSource::Node { .. } => None,
        }
    }

    pub fn is_node_fed(&self) -> bool {
        matches!(self.from, DuctSource::Node { .. })
    }

    pub fn vents(&self) -> bool {
        matches!(self.to, DuctSink::Mouth)
    }
}

/// A turbocharger placed in the exhaust: its turbine sits at `node`, so the ducts ending there feed
/// its inlet and the one leaving it is its outlet. Where it is drawn is the web app's business; the
/// solver needs only the node.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TurboMount {
    pub id: String,
    pub node: String,
    #[serde(default)]
    pub position: Option<[f64; 3]>,
    /// How it is turned, as a unit quaternion `[x, y, z, w]`.
    #[serde(default)]
    pub rotation: Option<[f64; 4]>,
    /// Its own settings, or `None` for the engine's.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub settings: Option<TurboSettings>,
}

/// What one turbo can be set to on its own, where the others differ: its wastegate's boost target,
/// gauge, Pa; its size, the compressor's flow at full speed, kg/s, 0 or less sizing it for the engine;
/// its intercooler's effectiveness, 0..1; and its blow-off valve. They are `EngineSpec`'s fields of the
/// same names.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TurboSettings {
    pub boost_target: f64,
    pub turbo_size: f64,
    pub intercooler: f64,
    pub blow_off: BlowOff,
}

impl TurboMount {
    /// Its settings: its own, or the engine's.
    pub fn settings_for(&self, spec: &EngineSpec) -> TurboSettings {
        self.settings.unwrap_or(TurboSettings {
            boost_target: spec.boost_target,
            turbo_size: spec.turbo_size,
            intercooler: spec.intercooler,
            blow_off: spec.blow_off,
        })
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ExhaustGraph {
    pub ducts: Vec<ExhaustDuct>,
    /// Turbochargers, each at one node. A mount whose node no duct names is not connected yet, and does
    /// nothing.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub turbos: Vec<TurboMount>,
}

impl ExhaustGraph {
    /// Whether a turbine sits at `node`.
    pub fn is_turbo_node(&self, node: &str) -> bool {
        self.turbos.iter().any(|t| t.node == node)
    }
}

/// Which side of a duct meets a node.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum End {
    Inlet,
    Outlet,
}

/// Node ids in the order they first appear, which is the order the solver indexes them in.
pub fn node_order(graph: &ExhaustGraph) -> Vec<String> {
    let mut seen: Vec<String> = Vec::new();
    for duct in &graph.ducts {
        for node in [duct.node_from(), duct.node_to()].into_iter().flatten() {
            if !seen.iter().any(|s| s == node) {
                seen.push(node.to_string());
            }
        }
    }
    seen
}

/// Every duct end that meets at `node`: the duct's index, and the side of it that is there.
pub fn ends_at(graph: &ExhaustGraph, node: &str) -> Vec<(usize, End)> {
    let mut ends = Vec::new();
    for (i, duct) in graph.ducts.iter().enumerate() {
        if duct.node_from() == Some(node) {
            ends.push((i, End::Inlet));
        }
        if duct.node_to() == Some(node) {
            ends.push((i, End::Outlet));
        }
    }
    ends
}

/// Ducts that vent to air, downstream-most first: node-fed ducts, then cylinders venting alone.
///
/// The order sets each mouth's place in the line of tailpipes, so it changes the sound.
pub fn radiating_ducts(graph: &ExhaustGraph) -> Vec<usize> {
    let mut mouths: Vec<usize> = (0..graph.ducts.len()).filter(|&i| graph.ducts[i].vents()).collect();
    let rank = |i: usize| if graph.ducts[i].is_node_fed() { 0 } else { 1 };
    mouths.sort_by_key(|&i| rank(i));
    mouths
}

/// The duct each cylinder's valve feeds, or `None` if the graph gives it none.
pub fn valve_ducts(graph: &ExhaustGraph, cylinders: usize) -> Vec<Option<usize>> {
    let mut out = vec![None; cylinders];
    for (i, duct) in graph.ducts.iter().enumerate() {
        let Some(c) = duct.valve_cylinder() else { continue };
        if c < 0 || c >= cylinders as i64 {
            continue;
        }
        if out[c as usize].is_none() {
            out[c as usize] = Some(i);
        }
    }
    out
}

/// The ducts a cylinder's gas passes through on its way to air, in order, following the first duct
/// leaving each junction. Stops if it revisits a duct.
pub fn path_to_air(graph: &ExhaustGraph, cylinder: i64) -> Vec<usize> {
    let mut path = Vec::new();
    let mut duct = graph.ducts.iter().position(|d| d.valve_cylinder() == Some(cylinder));
    let mut seen: Vec<&str> = Vec::new();
    while let Some(i) = duct {
        let d = &graph.ducts[i];
        if seen.contains(&d.id.as_str()) {
            break;
        }
        seen.push(&d.id);
        path.push(i);
        let Some(node) = d.node_to() else { break };
        duct = ends_at(graph, node).into_iter().find(|&(_, e)| e == End::Inlet).map(|(j, _)| j);
    }
    path
}

/// How far a manifold stands off the ports, m: the stub each cylinder feeds it through.
const MANIFOLD_STUB: f64 = 0.1;

fn pipe_segment(length: f64, dia: f64) -> PipeSegment {
    make_segment(SegmentPartial {
        kind: Some(SegmentKind::Pipe),
        length: Some(length),
        d_in: Some(dia),
        d_out: Some(dia),
        ..Default::default()
    })
}

fn copy_all(segments: &[PipeSegment]) -> Vec<PipeSegment> {
    segments.iter().map(copy_segment).collect()
}

/// The exhaust a layout choice describes, built from straight tube that snaps together: a manifold
/// along each bank, and a group across both banks of a V as a manifold per bank meeting behind the
/// engine. Cylinders venting alone keep the given runner geometry.
pub fn compile_layout(spec: &EngineSpec, pipe: &[PipeSegment], collector: &[PipeSegment]) -> ExhaustGraph {
    let groups = collector_groups(spec);
    let mut ducts: Vec<ExhaustDuct> = Vec::new();

    let mut pin_of: Vec<Option<f64>> = vec![None; groups.len()];
    for (i, pin) in crank_pins(spec).iter().enumerate() {
        for &c in &pin.cylinders {
            if c < pin_of.len() {
                pin_of[c] = Some(i as f64);
            }
        }
    }
    let pin = |c: usize| pin_of.get(c).copied().flatten().unwrap_or(c as f64);
    let spacing = cylinder_spacing(spec);
    let runner_dia = match pipe.last() {
        Some(last) => segment_diameter(last, 1.0),
        None => 0.042,
    };
    let mut collector_dia = runner_dia;
    for seg in collector {
        if seg.kind == SegmentKind::Chamber {
            continue;
        }
        collector_dia = math::max(math::max(collector_dia, segment_diameter(seg, 0.0)), segment_diameter(seg, 1.0));
    }
    let gathering = |n: f64| math::min(math::max(runner_dia * math::sqrt(n) * 0.92, runner_dia), collector_dia);
    let collector_after = |manifold_dia: f64| -> Vec<PipeSegment> {
        let mut segs = copy_all(collector);
        if let Some(first) = segs.first() {
            if first.kind != SegmentKind::Chamber && first.d_in < manifold_dia {
                let mut partial = SegmentPartial::from(first);
                partial.d_in = Some(manifold_dia);
                if first.kind == SegmentKind::Pipe {
                    partial.d_out = Some(manifold_dia);
                }
                segs[0] = make_segment(partial);
            }
        }
        segs
    };

    let mut runner_to: Vec<Option<String>> = vec![None; groups.len()];
    let mut runner_segments: Vec<Option<Vec<PipeSegment>>> = vec![None; groups.len()];
    let stub = || vec![pipe_segment(MANIFOLD_STUB, runner_dia)];

    let group_count = groups.iter().fold(0i32, |m, &g| m.max(g + 1));
    let mut tail: Vec<ExhaustDuct> = Vec::new();

    for g in 0..group_count {
        let members: Vec<usize> = (0..groups.len()).filter(|&c| groups[c] == g).collect();
        if members.is_empty() {
            continue;
        }
        let mut banks: Vec<u32> = Vec::new();
        for &c in &members {
            let b = physical_bank(spec, c);
            if !banks.contains(&b) {
                banks.push(b);
            }
        }

        // Chain one bank's cylinders into a manifold ending at `last`, returning the duct carrying the
        // manifold into `last`.
        let chain = |bank_members: &[usize],
                     last: &str,
                     tag: &str,
                     runner_to: &mut Vec<Option<String>>,
                     runner_segments: &mut Vec<Option<Vec<PipeSegment>>>,
                     tail: &mut Vec<ExhaustDuct>|
         -> Option<String> {
            let mut order = bank_members.to_vec();
            order.sort_by(|&a, &b| pin(a).partial_cmp(&pin(b)).unwrap_or(std::cmp::Ordering::Equal));
            if order.len() < 2 {
                if let Some(&c) = order.first() {
                    runner_to[c] = Some(last.to_string());
                }
                return None;
            }
            let node_at = |k: usize| if k == order.len() - 1 { last.to_string() } else { format!("{last}-{k}") };
            let gap_after = |k: usize| (pin(order[k + 1]) - pin(order[k])).abs() * spacing;
            let dia_after = |k: usize| gathering(k as f64 + 1.0);

            let first = exhaust_port_of(spec, order[0]);
            let next = exhaust_port_of(spec, order[1]);
            let s = math::sign(next.position[2] - first.position[2]);
            let along: Vec3 = [0.0, 0.0, if s == 0.0 || s.is_nan() { 1.0 } else { s }];
            let (yaw, pitch) = turn_between_dirs(first.direction, along);

            for (k, &c) in order.iter().enumerate() {
                let segs = if k == 0 {
                    let mut v = stub();
                    v.push(make_segment(SegmentPartial {
                        kind: Some(SegmentKind::Pipe),
                        length: Some(gap_after(0)),
                        d_in: Some(dia_after(0)),
                        d_out: Some(dia_after(0)),
                        yaw: Some(yaw),
                        pitch: Some(pitch),
                        ..Default::default()
                    }));
                    v
                } else {
                    stub()
                };
                runner_segments[c] = Some(segs);
                runner_to[c] = Some(node_at(k.max(1)));
            }

            let mut carrying = format!("runner{}", order[0]);
            for k in 1..order.len() - 1 {
                let id = format!("link{tag}-{k}");
                let mut duct = ExhaustDuct::new(
                    id.clone(),
                    vec![pipe_segment(gap_after(k), dia_after(k))],
                    DuctSource::Node { node: node_at(k) },
                    DuctSink::Node { node: node_at(k + 1) },
                );
                duct.continues = Some(carrying);
                duct.role = Some(DuctRole::Manifold);
                tail.push(duct);
                carrying = id;
            }
            Some(carrying)
        };

        if banks.len() == 1 {
            let merge = format!("merge{g}");
            let last_link = chain(&members, &merge, &format!("{g}"), &mut runner_to, &mut runner_segments, &mut tail);
            let segments = if members.len() > 1 {
                collector_after(gathering(members.len() as f64 - 1.0))
            } else {
                copy_all(collector)
            };
            let mut duct =
                ExhaustDuct::new(format!("collector{g}"), segments, DuctSource::Node { node: merge }, DuctSink::Mouth);
            duct.continues = last_link;
            duct.role = Some(DuctRole::Collector);
            tail.push(duct);
            continue;
        }

        struct BankEnd {
            bank: u32,
            members: Vec<usize>,
            end: Vec3,
        }
        let ends: Vec<BankEnd> = banks
            .iter()
            .map(|&bank| {
                let bank_members: Vec<usize> =
                    members.iter().copied().filter(|&c| physical_bank(spec, c) == bank).collect();
                let mut by_pin = bank_members.clone();
                by_pin.sort_by(|&a, &b| pin(b).partial_cmp(&pin(a)).unwrap_or(std::cmp::Ordering::Equal));
                let port = exhaust_port_of(spec, by_pin[0]);
                let runner = if bank_members.len() > 1 { stub() } else { pipe.to_vec() };
                BankEnd { bank, members: bank_members, end: sweep_end(&runner, port.position, port.direction).0 }
            })
            .collect();
        let count = ends.len() as f64;
        let mut max_z = f64::NEG_INFINITY;
        for e in &ends {
            max_z = math::max(max_z, e.end[2]);
        }
        let mid: Vec3 = [
            ends.iter().fold(0.0, |a, e| a + e.end[0]) / count,
            ends.iter().fold(0.0, |a, e| a + e.end[1]) / count,
            max_z + spacing,
        ];
        let merge = format!("merge{g}");
        let mut downpipe = 0.0;
        let mut downpipe_dia = 0.0;
        for e in &ends {
            if e.members.len() == 1 {
                runner_to[e.members[0]] = Some(merge.clone());
                continue;
            }
            let bank_node = format!("merge{g}-b{}", e.bank);
            let carried = chain(
                &e.members,
                &bank_node,
                &format!("{g}-b{}", e.bank),
                &mut runner_to,
                &mut runner_segments,
                &mut tail,
            );
            let length = distance(e.end, mid);
            downpipe = math::max(downpipe, length);
            let dia = gathering(e.members.len() as f64);
            downpipe_dia = math::max(downpipe_dia, dia);
            let mut duct = ExhaustDuct::new(
                format!("down{g}-b{}", e.bank),
                vec![pipe_segment(length, dia)],
                DuctSource::Node { node: bank_node },
                DuctSink::Node { node: merge.clone() },
            );
            duct.continues = carried;
            duct.role = Some(DuctRole::Downpipe);
            tail.push(duct);
        }
        let base = if downpipe_dia > 0.0 { collector_after(downpipe_dia) } else { copy_all(collector) };
        let mut duct = ExhaustDuct::new(
            format!("collector{g}"),
            shortened(base, downpipe),
            DuctSource::Node { node: merge },
            DuctSink::Mouth,
        );
        duct.role = Some(DuctRole::Collector);
        tail.push(duct);
    }

    for (cylinder, &group) in groups.iter().enumerate() {
        let node = runner_to[cylinder].clone();
        let stubbed = runner_segments[cylinder].take();
        let role = if stubbed.is_some() { DuctRole::Stub } else { DuctRole::Runner };
        let to = match node {
            Some(node) if group >= 0 => DuctSink::Node { node },
            _ => DuctSink::Mouth,
        };
        let mut duct = ExhaustDuct::new(
            format!("runner{cylinder}"),
            stubbed.unwrap_or_else(|| copy_all(pipe)),
            DuctSource::Valve { cylinder: cylinder as i64 },
            to,
        );
        duct.role = Some(role);
        ducts.push(duct);
    }
    ducts.extend(tail);
    ExhaustGraph { ducts, turbos: Vec::new() }
}

/// The exhaust `spec` asks for: equal-length headers into one merge per collector where it has
/// `exhaust_headers` and something to merge, and a manifold along the ports otherwise.
pub fn compile_exhaust(spec: &EngineSpec, pipe: &[PipeSegment], collector: &[PipeSegment]) -> ExhaustGraph {
    if spec.exhaust_headers && exhaust_layout_of(spec) != ExhaustLayout::Open {
        compile_collector_layout(spec, pipe, collector)
    } else {
        compile_layout(spec, pipe, collector)
    }
}

/// The equal-length alternative: every runner of a group into one junction, then its collector.
pub fn compile_collector_layout(spec: &EngineSpec, pipe: &[PipeSegment], collector: &[PipeSegment]) -> ExhaustGraph {
    let groups = collector_groups(spec);
    let mut ducts: Vec<ExhaustDuct> = groups
        .iter()
        .enumerate()
        .map(|(cylinder, &group)| {
            let to = if group < 0 { DuctSink::Mouth } else { DuctSink::Node { node: format!("merge{group}") } };
            let mut d = ExhaustDuct::new(
                format!("runner{cylinder}"),
                copy_all(pipe),
                DuctSource::Valve { cylinder: cylinder as i64 },
                to,
            );
            d.role = Some(DuctRole::Runner);
            d
        })
        .collect();
    let mut unique: Vec<i32> = Vec::new();
    for &g in &groups {
        if g >= 0 && !unique.contains(&g) {
            unique.push(g);
        }
    }
    unique.sort();
    for g in unique {
        let mut d = ExhaustDuct::new(
            format!("collector{g}"),
            copy_all(collector),
            DuctSource::Node { node: format!("merge{g}") },
            DuctSink::Mouth,
        );
        d.role = Some(DuctRole::Collector);
        ducts.push(d);
    }
    ExhaustGraph { ducts, turbos: Vec::new() }
}

/// `segments` with `length` taken out of its longest plain pipe, as far as that pipe can spare.
fn shortened(mut segments: Vec<PipeSegment>, length: f64) -> Vec<PipeSegment> {
    if length <= 0.0 {
        return segments;
    }
    let mut longest: Option<usize> = None;
    for (i, seg) in segments.iter().enumerate() {
        if seg.kind == SegmentKind::Pipe && longest.is_none_or(|l| seg.length > segments[l].length) {
            longest = Some(i);
        }
    }
    if let Some(l) = longest {
        let seg = &mut segments[l];
        seg.length = math::max(seg.length - length, math::min(seg.length, 0.1));
    }
    segments
}

/// Reasons a graph cannot be solved, as readable sentences. Empty means it can.
pub fn validate_graph(graph: &ExhaustGraph, cylinders: usize) -> Vec<String> {
    let mut problems = Vec::new();
    let mut ids: Vec<&str> = Vec::new();
    for duct in &graph.ducts {
        if ids.contains(&duct.id.as_str()) {
            problems.push(format!("two ducts share the id \"{}\"", duct.id));
        }
        ids.push(&duct.id);
    }

    let mut per_valve: Vec<(i64, usize)> = Vec::new();
    for duct in &graph.ducts {
        let Some(c) = duct.valve_cylinder() else { continue };
        match per_valve.iter_mut().find(|(k, _)| *k == c) {
            Some(entry) => entry.1 += 1,
            None => per_valve.push((c, 1)),
        }
    }
    for c in 0..cylinders as i64 {
        let n = per_valve.iter().find(|(k, _)| *k == c).map_or(0, |e| e.1);
        if n == 0 {
            problems.push(format!("cylinder {} has no exhaust pipe", c + 1));
        }
        if n > 1 {
            problems.push(format!("cylinder {} has {n} pipes on its exhaust port", c + 1));
        }
    }
    for &(c, _) in &per_valve {
        if c < 0 || c >= cylinders as i64 {
            problems.push(format!("a pipe is attached to cylinder {}, which does not exist", c + 1));
        }
    }

    for node in node_order(graph) {
        let ends = ends_at(graph, &node);
        let downstream = ends.iter().filter(|e| e.1 == End::Inlet).count();
        let upstream = ends.iter().filter(|e| e.1 == End::Outlet).count();
        if ends.len() < 2 {
            problems.push(format!("junction \"{node}\" joins only one pipe"));
        }
        if upstream == 0 {
            problems.push(format!("junction \"{node}\" has nothing flowing into it"));
        }
        if downstream == 0 {
            problems.push(format!("junction \"{node}\" has no pipe leaving it"));
        }
        if downstream > 1 && graph.is_turbo_node(&node) {
            problems.push(format!("the turbo at \"{node}\" has {downstream} pipes leaving its one outlet"));
        }
    }

    // Every duct must trace back to a valve, or the gas in it came from nowhere.
    let mut reachable: Vec<bool> = vec![false; graph.ducts.len()];
    let mut frontier: Vec<usize> = (0..graph.ducts.len()).filter(|&i| !graph.ducts[i].is_node_fed()).collect();
    for &i in &frontier {
        reachable[i] = true;
    }
    let mut guard = 0;
    while guard < graph.ducts.len() + 1 && !frontier.is_empty() {
        guard += 1;
        let i = frontier.pop().unwrap();
        let Some(node) = graph.ducts[i].node_to() else { continue };
        for (j, end) in ends_at(graph, node) {
            if end != End::Inlet || reachable[j] {
                continue;
            }
            reachable[j] = true;
            frontier.push(j);
        }
    }
    for (i, duct) in graph.ducts.iter().enumerate() {
        if !reachable[i] {
            problems.push(format!("pipe \"{}\" is not connected to any cylinder", duct.id));
        }
    }
    problems
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spec::{CrankType, ExhaustLayoutSpec};

    fn runner() -> Vec<PipeSegment> {
        vec![pipe_segment(0.4, 0.042)]
    }

    #[test]
    fn a_compiled_v8_manifold_validates() {
        let spec = EngineSpec {
            cylinders: 8,
            v_angle: 90.0,
            crank_type: CrankType::Crossplane,
            exhaust_layout: ExhaustLayoutSpec::PerBank,
            ..Default::default()
        };
        let graph = compile_exhaust(&spec, &runner(), &[pipe_segment(0.5, 0.06)]);
        assert!(validate_graph(&graph, 8).is_empty());
        // Three junctions along each bank: where the second, third and fourth cylinders join.
        assert_eq!(node_order(&graph).len(), 6);
    }

    #[test]
    fn a_missing_runner_is_reported() {
        let spec = EngineSpec { cylinders: 2, exhaust_layout: ExhaustLayoutSpec::TwoIntoOne, ..Default::default() };
        let mut graph = compile_exhaust(&spec, &runner(), &[pipe_segment(0.5, 0.06)]);
        graph.ducts.remove(1);
        let problems = validate_graph(&graph, 2);
        assert!(problems.iter().any(|p| p == "cylinder 2 has no exhaust pipe"), "{problems:?}");
    }

    #[test]
    fn graphs_round_trip_through_json() {
        let spec = EngineSpec { cylinders: 4, exhaust_layout: ExhaustLayoutSpec::Merged, ..Default::default() };
        let graph = compile_exhaust(&spec, &runner(), &[pipe_segment(0.5, 0.06)]);
        let json = serde_json::to_string(&graph).unwrap();
        assert!(json.contains("\"kind\":\"valve\""));
        let back: ExhaustGraph = serde_json::from_str(&json).unwrap();
        assert_eq!(back, graph);
    }
}
