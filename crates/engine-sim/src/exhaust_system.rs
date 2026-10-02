//! The exhaust as a whole: an `ExhaustGraph` of ducts joined at junctions, with one duct fed by each
//! cylinder's exhaust valve, all marched in lockstep.
//!
//! A junction is where cylinders hear each other: each blowdown pulse arrives there and partly
//! travels up the other primaries, where it helps scavenge those cylinders or blocks them depending
//! on where the firing interval puts it.

use crate::afterfire::AFTERFIRE_ZONE_LENGTH;
use crate::dsp::Noise;
use crate::euler_pipe::{DuctEnd, EndState, EulerPipe, EulerPipeOptions, InletKind, OutletKind, ValveState};
use crate::exhaust_graph::{
    DuctRole, End, ExhaustGraph, TurboMount, ends_at, node_order, path_to_air, radiating_ducts, validate_graph,
    valve_ducts,
};
use crate::math::{self, PI, clamp};
use crate::pool::{Disjoint, ThreadPool};
use crate::spec::gas;
use crate::turbo;

/// Turbulence intensity of the merge, as a fraction of the mixing mass flow.
const MERGE_TURBULENCE: f64 = 0.14;

/// Specific heat at constant pressure for exhaust gas, J/(kg K).
const CP_EXH: f64 = (gas::GAMMA_EXH * gas::R) / (gas::GAMMA_EXH - 1.0);

/// Relative mass-flux imbalance below which a junction is left alone.
const JUNCTION_BALANCE_TOL: f64 = 0.005;

/// Fewest cells worth handing a thread of their own: below it, handing off costs more than it saves.
const MIN_CELLS_PER_THREAD: usize = 40;

/// Turbulence intensity of the jet through an open wastegate, as a fraction of its mass flow.
const BYPASS_TURBULENCE: f64 = 0.2;

/// A turbine at a junction: what `EngineSim` sets each sample from its turbo's state.
///
/// The flow through it follows Stodola's ellipse law, `m = K sqrt(p_up^2 - p_down^2) / sqrt(T)`, with
/// `K` the turbine's and the open wastegate's together.
#[derive(Clone, Copy, Debug, Default)]
pub struct TurbineSetting {
    /// Flow constants of the turbine and of its wastegate as open as it is now, kg*sqrt(K)/(s*Pa).
    pub k_turbine: f64,
    pub k_wastegate: f64,
    /// Speed of the turbine wheel's tip, m/s: with the drop across it, where it runs on its map.
    pub tip_speed: f64,
    /// Factor on the turbine's flow this sample: its blades' pulsation, 1 on average.
    pub pulsation: f64,
    /// Scale on the wastegate jet's turbulence.
    pub bypass_noise: f64,
}

/// A turbine junction's own state.
struct TurbineNode {
    noise: Noise,
    lp1: f64,
    lp2: f64,
}

/// One junction, with every duct end that meets there. Outlets are always visited before inlets:
/// floating-point addition is not associative, so the visiting order is part of the result.
struct JunctionNode {
    /// Ducts emptying into the node, and ducts leaving it: indices into `ExhaustSystem::ducts`.
    outlets: Vec<usize>,
    inlets: Vec<usize>,
    /// End states, outlets first then inlets.
    states: Vec<EndState>,
    /// Ducts leaving the node, as slots of `fed_by_node`, and the share of the mixing noise each takes.
    downstream: Vec<(usize, f64)>,
    noise: Noise,
    lp1: f64,
    lp2: f64,
}

/// What one turbine did over an `advance`: the power it took from the exhaust and the power of the
/// isentropic drop across it, W, the flow through it and through its wastegate, kg/s, and the mean
/// pressure at its inlet and outlet, Pa.
#[derive(Clone, Copy, Debug, Default)]
pub struct TurbineResult {
    pub power: f64,
    pub isentropic_power: f64,
    pub flow: f64,
    pub bypass_flow: f64,
    pub inlet: f64,
    pub outlet: f64,
}

/// The result of one `advance`. Reused between calls.
#[derive(Clone, Debug, Default)]
pub struct ExhaustResult {
    /// Volume flow out of each radiating mouth, m^3/s.
    pub mouth_flows: Vec<f64>,
    /// Mass flow through each cylinder's exhaust valve, kg/s, positive out of the cylinder.
    pub valve_mass_flows: Vec<f64>,
    pub substeps: usize,
    /// Each turbine's, in the order of `ExhaustSystem::turbine_mounts`, while they are set.
    pub turbines: Vec<TurbineResult>,
    /// Heat each primary's afterfire zone took from `afterfire_heat` this sample, J.
    pub heat_taken: Vec<f64>,
}

pub struct ExhaustSystem {
    /// Every duct: valve-fed first, in cylinder order, then node-fed, in node order.
    pub ducts: Vec<EulerPipe>,
    /// How many of `ducts` are primaries, one per cylinder.
    pub primary_count: usize,
    /// Position in `ducts` of each graph duct, in the graph's order.
    by_graph: Vec<usize>,
    /// Each graph duct's id, in the graph's order.
    pub duct_ids: Vec<String>,
    main_collector: Option<usize>,
    air_path: Vec<usize>,
    radiating: Vec<usize>,
    radiating_index: Vec<i64>,
    pub result: ExhaustResult,
    nodes: Vec<JunctionNode>,
    /// The ducts a junction feeds, with the mass source going into each one's first cell.
    fed_by_node: Vec<(usize, ValveState)>,
    fed_flow: Vec<f64>,
    substep_dt: f64,
    substep_noise_scale: f64,
    /// Worst relative mass-flux imbalance a junction's per-duct solves produced. Diagnostic only.
    pub junction_residual: f64,
    turbulence: f64,
    /// Per node, which of `turbine_mounts` has its turbine there, if one does.
    turbine_slot: Vec<Option<usize>>,
    /// The turbos with a turbine in the exhaust, in node order.
    turbine_mounts: Vec<TurboMount>,
    turbine_nodes: Vec<TurbineNode>,
    /// Each turbine's setting this sample, in the order of `turbine_mounts`; empty for none.
    turbines: Vec<TurbineSetting>,
    /// Heat released by afterfire in each primary's leading cells this sample, W; and how many cells
    /// that is, and their volume, m^3.
    afterfire_heat: Vec<f64>,
    zone_cells: Vec<usize>,
    zone_volume: Vec<f64>,
    /// What each duct's own boundaries and update gave over a substep, gathered in duct order.
    duct_out: Vec<DuctOut>,
    /// What each thread steps, for a pool of `groups_for` threads alongside side work costing
    /// `groups_side`: ducts by their index, side items past them.
    groups: Vec<Vec<usize>>,
    groups_for: usize,
    groups_side: Vec<usize>,
    /// The thread each duct was stepped on in the last `advance_on`, and how many there were.
    owner: Vec<usize>,
    threads_used: usize,
}

/// One duct's part of an `ExhaustResult` over one substep, on a cache line of its own, as each
/// duct's may be written from a thread of its own.
#[derive(Clone, Copy, Debug, Default)]
#[repr(align(128))]
struct DuctOut {
    mouth_flow: f64,
    valve_flow: f64,
    heat: f64,
    /// On the last substep: whether the duct's state had gone inadmissible, and was reset.
    broken: bool,
}

/// What a duct reads to finish a substep, shared by every thread.
struct FinishInputs<'a> {
    h: f64,
    last: bool,
    primaries: usize,
    valves: &'a [ValveState],
    afterfire_heat: &'a [f64],
    zone_cells: &'a [usize],
    zone_volume: &'a [f64],
    fed_by_node: &'a [(usize, ValveState)],
    fed_flow: &'a [f64],
}

impl FinishInputs<'_> {
    /// Duct `i`'s own boundaries, valve or junction source, conservative update and thermal pass,
    /// and on the last substep its recovery if it has broken. It touches no other duct, so the ducts
    /// can be finished in any order, or at once, and each duct's cells stay with the thread that
    /// steps it.
    fn finish(&self, i: usize, duct: &mut EulerPipe) -> DuctOut {
        let h = self.h;
        let mut out = DuctOut { mouth_flow: duct.apply_own_boundaries(h), ..DuctOut::default() };
        if i < self.primaries {
            let valve = &self.valves[i];
            let flow = duct.valve_flux_for(valve);
            duct.set_end_step(h, flow + valve.extra_mass_flow);
            duct.end_step_set(valve);
            out.valve_flow = flow * duct.source_scale;
            let heat = self.afterfire_heat[i];
            if heat > 0.0 {
                out.heat = duct.add_heat(heat * h, self.zone_cells[i], self.zone_volume[i]);
            }
        } else {
            // Every duct past the primaries is node-fed, and `fed_by_node` holds them in duct order.
            let slot = i - self.primaries;
            duct.set_end_step(h, self.fed_flow[slot]);
            duct.end_step_set(&self.fed_by_node[slot].1);
        }
        duct.after_step(h);
        if self.last {
            out.broken = duct.recover_if_broken();
        }
        out
    }
}

/// Work that is independent of the exhaust for a sample, stepped alongside its ducts: `costs[k]` is
/// what item `k` costs, in cells, and `job(k)` steps it. Each item is stepped once per `advance_on`.
pub struct SideWork<'a> {
    pub costs: &'a [usize],
    pub job: &'a (dyn Fn(usize) + Sync),
}

/// Ducts and side items split into `threads` groups of about equal cells, the largest placed first,
/// each onto the group with the fewest so far; each group in order, ducts first.
fn balance(ducts: &[EulerPipe], side: &[usize], threads: usize) -> Vec<Vec<usize>> {
    let cost = |i: usize| if i < ducts.len() { ducts[i].n } else { side[i - ducts.len()] };
    let mut order: Vec<usize> = (0..ducts.len() + side.len()).collect();
    order.sort_by_key(|&i| std::cmp::Reverse(cost(i)));
    let mut groups: Vec<Vec<usize>> = vec![Vec::new(); threads];
    let mut load = vec![0usize; threads];
    for i in order {
        let g = (0..threads).min_by_key(|&g| load[g]).unwrap();
        groups[g].push(i);
        load[g] += cost(i);
    }
    for g in groups.iter_mut() {
        g.sort();
    }
    groups
}

impl ExhaustSystem {
    pub fn new(
        graph: &ExhaustGraph,
        cylinders: usize,
        sample_rate: f64,
        port_gas_temp: f64,
        opts: &EulerPipeOptions,
    ) -> Result<ExhaustSystem, String> {
        let problems = validate_graph(graph, cylinders);
        if !problems.is_empty() {
            return Err(format!("exhaust graph cannot be solved: {}", problems.join("; ")));
        }

        let order = node_order(graph);
        // Built pipes by graph duct index.
        let mut built: Vec<Option<EulerPipe>> = (0..graph.ducts.len()).map(|_| None).collect();
        let mut outlet_area: Vec<f64> = vec![0.0; graph.ducts.len()];
        let mut duct_list: Vec<usize> = Vec::new();

        let valve_fed = valve_ducts(graph, cylinders);
        for di in valve_fed.iter() {
            let di = di.expect("validated: every cylinder has a duct");
            let duct = &graph.ducts[di];
            let o = EulerPipeOptions {
                inlet_kind: Some(InletKind::Valve),
                outlet_kind: Some(if duct.vents() { OutletKind::Mouth } else { OutletKind::Junction }),
                ..opts.clone()
            };
            let pipe = EulerPipe::new(&duct.segments, sample_rate, port_gas_temp, &o);
            outlet_area[di] = pipe.outlet_area();
            built[di] = Some(pipe);
            duct_list.push(di);
        }

        // Node-fed ducts, upstream first, so each knows the area feeding its inlet.
        let mut pending: Vec<usize> = (0..graph.ducts.len()).filter(|&i| graph.ducts[i].is_node_fed()).collect();
        let mut node_fed: Vec<usize> = Vec::new();
        let passes = pending.len() + 1;
        let mut pass = 0;
        while pass < passes && !pending.is_empty() {
            pass += 1;
            let mut made_progress = false;
            let mut i = 0;
            while i < pending.len() {
                let di = pending[i];
                let duct = &graph.ducts[di];
                let crate::exhaust_graph::DuctSource::Node { node } = &duct.from else { unreachable!() };
                let upstream: Vec<usize> =
                    ends_at(graph, node).into_iter().filter(|e| e.1 == End::Outlet).map(|e| e.0).collect();
                if !upstream.iter().all(|&u| built[u].is_some()) {
                    i += 1;
                    continue;
                }
                let mut feed_area = 0.0;
                for &u in &upstream {
                    feed_area += outlet_area[u];
                }
                let o = EulerPipeOptions {
                    inlet_kind: Some(InletKind::Junction),
                    outlet_kind: Some(if duct.vents() { OutletKind::Mouth } else { OutletKind::Junction }),
                    port: None,
                    junction_inlet_area: Some(feed_area),
                    ..opts.clone()
                };
                let pipe = EulerPipe::new(&duct.segments, sample_rate, port_gas_temp, &o);
                outlet_area[di] = pipe.outlet_area();
                built[di] = Some(pipe);
                node_fed.push(di);
                pending.remove(i);
                made_progress = true;
            }
            if !made_progress {
                break;
            }
        }
        for &di in &pending {
            let duct = &graph.ducts[di];
            let o = EulerPipeOptions {
                inlet_kind: Some(InletKind::Junction),
                outlet_kind: Some(if duct.vents() { OutletKind::Mouth } else { OutletKind::Junction }),
                port: None,
                ..opts.clone()
            };
            let pipe = EulerPipe::new(&duct.segments, sample_rate, port_gas_temp, &o);
            outlet_area[di] = pipe.outlet_area();
            built[di] = Some(pipe);
            node_fed.push(di);
        }
        // Back into node order, so the noise seeds and the collector readouts are stable.
        let node_index = |di: usize| -> usize {
            match &graph.ducts[di].from {
                crate::exhaust_graph::DuctSource::Node { node } => order.iter().position(|o| o == node).unwrap_or(0),
                _ => 0,
            }
        };
        node_fed.sort_by_key(|&di| node_index(di));
        duct_list.extend(node_fed.iter().copied());

        // Graph duct index -> position in `ducts`.
        let mut position: Vec<usize> = vec![usize::MAX; graph.ducts.len()];
        for (i, &di) in duct_list.iter().enumerate() {
            position[di] = i;
        }
        let ducts: Vec<EulerPipe> = duct_list.iter().map(|&di| built[di].take().unwrap()).collect();
        let duct_count = ducts.len();

        let collector_duct = node_fed.iter().copied().find(|&di| graph.ducts[di].role == Some(DuctRole::Collector));
        let main_collector = match collector_duct {
            Some(di) => Some(position[di]),
            None => {
                if node_fed.is_empty() {
                    None
                } else {
                    Some(valve_fed.len())
                }
            }
        };
        let air_path: Vec<usize> =
            path_to_air(graph, 0).into_iter().filter(|&di| position[di] != usize::MAX).map(|di| position[di]).collect();

        let mut nodes: Vec<JunctionNode> = Vec::new();
        for (n, id) in order.iter().enumerate() {
            let all = ends_at(graph, id);
            let mut outlet_ends: Vec<usize> =
                all.iter().filter(|e| e.1 == End::Outlet).map(|e| position[e.0]).collect();
            let mut inlet_ends: Vec<usize> = all.iter().filter(|e| e.1 == End::Inlet).map(|e| position[e.0]).collect();
            outlet_ends.sort();
            inlet_ends.sort();
            let mut area_sum = 0.0;
            for &d in &inlet_ends {
                area_sum += ducts[d].inlet_area();
            }
            let downstream: Vec<(usize, f64)> = inlet_ends
                .iter()
                .map(|&d| {
                    let share =
                        if area_sum > 0.0 { ducts[d].inlet_area() / area_sum } else { 1.0 / inlet_ends.len() as f64 };
                    (d, share)
                })
                .collect();
            nodes.push(JunctionNode {
                states: vec![EndState::default(); all.len()],
                outlets: outlet_ends,
                inlets: inlet_ends,
                downstream,
                noise: Noise::new(0x7f4a3b as f64 + n as f64 * 0x9e3779b as f64),
                lp1: 0.0,
                lp2: 0.0,
            });
        }

        let radiating: Vec<usize> = radiating_ducts(graph).into_iter().map(|di| position[di]).collect();
        let radiating_index: Vec<i64> =
            (0..ducts.len()).map(|i| radiating.iter().position(|&r| r == i).map_or(-1, |p| p as i64)).collect();
        let result = ExhaustResult {
            mouth_flows: vec![0.0; radiating.len()],
            valve_mass_flows: vec![0.0; valve_fed.len()],
            heat_taken: vec![0.0; valve_fed.len()],
            substeps: 1,
            ..ExhaustResult::default()
        };

        // Every node-fed duct, in duct order, with the valve state its merge noise enters through.
        let mut fed_by_node: Vec<(usize, ValveState)> = Vec::new();
        let mut fed_slot: Vec<usize> = vec![usize::MAX; ducts.len()];
        for (i, &di) in duct_list.iter().enumerate() {
            if !graph.ducts[di].is_node_fed() {
                continue;
            }
            fed_slot[i] = fed_by_node.len();
            fed_by_node.push((
                i,
                ValveState {
                    throat_area: 0.0,
                    cyl_pressure: gas::P_AMB,
                    cyl_temp: port_gas_temp,
                    cyl_gamma: gas::GAMMA_EXH,
                    extra_mass_flow: 0.0,
                },
            ));
        }
        for node in nodes.iter_mut() {
            for d in node.downstream.iter_mut() {
                d.0 = fed_slot[d.0];
            }
        }
        // `FinishInputs::finish` finds a duct's slot from its position.
        debug_assert!(fed_by_node.iter().enumerate().all(|(k, f)| f.0 == valve_fed.len() + k));
        let fed_flow = vec![0.0; fed_by_node.len()];

        let mut turbine_mounts = Vec::new();
        let turbine_slot: Vec<Option<usize>> = order
            .iter()
            .map(|id| {
                let mount = graph.turbos.iter().find(|t| &t.node == id)?;
                turbine_mounts.push(mount.clone());
                Some(turbine_mounts.len() - 1)
            })
            .collect();
        let turbine_nodes: Vec<TurbineNode> = (0..nodes.len())
            .map(|n| TurbineNode {
                noise: Noise::new(0x51ed_2705 as f64 + n as f64 * 0x9e3779b as f64),
                lp1: 0.0,
                lp2: 0.0,
            })
            .collect();

        let zone_cells: Vec<usize> = ducts[..valve_fed.len()]
            .iter()
            .map(|d| ((AFTERFIRE_ZONE_LENGTH / d.dx).ceil() as usize).clamp(2, d.n.max(2)))
            .collect();
        let zone_volume: Vec<f64> = zone_cells.iter().enumerate().map(|(b, &c)| ducts[b].leading_volume(c)).collect();

        Ok(ExhaustSystem {
            afterfire_heat: vec![0.0; valve_fed.len()],
            zone_cells,
            zone_volume,
            primary_count: valve_fed.len(),
            by_graph: position,
            duct_ids: graph.ducts.iter().map(|d| d.id.clone()).collect(),
            ducts,
            main_collector,
            air_path,
            radiating,
            radiating_index,
            result,
            nodes,
            fed_by_node,
            fed_flow,
            substep_dt: 0.0,
            substep_noise_scale: 1.0,
            junction_residual: 0.0,
            turbulence: 1.0,
            turbine_slot,
            turbine_mounts,
            turbine_nodes,
            turbines: Vec::new(),
            duct_out: vec![DuctOut::default(); duct_count],
            groups: Vec::new(),
            groups_for: 0,
            groups_side: Vec::new(),
            owner: vec![0; duct_count],
            threads_used: 1,
        })
    }

    /// Heat afterfire releases in each primary's leading cells over the next sample, W.
    pub fn afterfire_heat_mut(&mut self) -> &mut [f64] {
        &mut self.afterfire_heat
    }

    /// Cylinder `b`'s afterfire zone: its primary's leading cells, how many, and their volume, m^3.
    pub fn afterfire_zone(&self, b: usize) -> (usize, f64) {
        (self.zone_cells[b], self.zone_volume[b])
    }

    /// Cylinder `b`'s primary.
    #[inline]
    pub fn primary(&self, b: usize) -> &EulerPipe {
        &self.ducts[b]
    }

    /// One per cylinder: the duct its exhaust valve feeds.
    pub fn primaries(&self) -> &[EulerPipe] {
        &self.ducts[..self.primary_count]
    }

    /// Ducts fed by a junction rather than a valve, in node order.
    pub fn collectors(&self) -> &[EulerPipe] {
        &self.ducts[self.primary_count..]
    }

    /// Turbulence scale, shared with the valve throat noise.
    pub fn set_turbulence(&mut self, value: f64) {
        self.turbulence = clamp(value, 0.0, 4.0);
    }

    /// The collector, or `None` if nothing merges: the first duct the graph calls a collector, failing
    /// that the first junction-fed duct.
    pub fn collector(&self) -> Option<&EulerPipe> {
        self.main_collector.map(|i| &self.ducts[i])
    }

    /// How many turbines are in the exhaust: turbos placed at a junction pipes meet at.
    pub fn turbine_count(&self) -> usize {
        self.turbine_mounts.len()
    }

    /// The turbos with a turbine in the exhaust, in the order their settings and results are in.
    pub fn turbine_mounts(&self) -> &[TurboMount] {
        &self.turbine_mounts
    }

    /// Carry each turbo's own settings over from `graph`, which differs from the one this was built
    /// from in nothing else.
    pub fn update_turbo_settings(&mut self, graph: &ExhaustGraph) {
        for m in self.turbine_mounts.iter_mut() {
            m.settings = graph.turbos.iter().find(|t| t.id == m.id).and_then(|t| t.settings);
        }
    }

    /// Each turbine's setting this sample, in the order of `turbine_mounts`, or none at all to take
    /// them out.
    pub fn set_turbines(&mut self, settings: &[TurbineSetting]) {
        self.turbines.clear();
        self.turbines.extend_from_slice(settings);
    }

    /// How many mouths radiate.
    pub fn mouth_count(&self) -> usize {
        self.radiating.len()
    }

    /// Total cells across every duct.
    pub fn cells(&self) -> usize {
        self.ducts.iter().map(|d| d.n).sum()
    }

    pub fn recoveries(&self) -> u64 {
        self.ducts.iter().map(|d| d.recoveries).sum()
    }

    /// Advance every duct by `dt`, on one shared substep count.
    pub fn advance(&mut self, dt: f64, valves: &[ValveState]) -> &ExhaustResult {
        self.advance_on(dt, valves, None, 1, None)
    }

    /// Threads worth giving this exhaust and `side`, out of `available`: enough cells for each to be
    /// worth it.
    pub fn useful_threads(&self, side: &[usize], available: usize) -> usize {
        let cells: usize = self.ducts.iter().map(|d| d.n).sum::<usize>() + side.iter().sum::<usize>();
        available.min(self.ducts.len() + side.len()).min(cells / MIN_CELLS_PER_THREAD).max(1)
    }

    /// `advance`, with the ducts stepped across up to `max_threads` of `pool`'s threads where there are
    /// enough of them to be worth it, and `side` stepped alongside them. Each duct is stepped exactly as on one thread, so
    /// the result is the same to the bit.
    pub fn advance_on(
        &mut self,
        dt: f64,
        valves: &[ValveState],
        pool: Option<&ThreadPool>,
        max_threads: usize,
        side: Option<&SideWork>,
    ) -> &ExhaustResult {
        let side_costs = side.map_or(&[][..], |s| s.costs);
        let threads = pool.map_or(1, |p| self.useful_threads(side_costs, p.threads().min(max_threads)));
        if threads > 1 && (self.groups_for != threads || self.groups_side != side_costs) {
            self.groups = balance(&self.ducts, side_costs, threads);
            self.groups_for = threads;
            self.groups_side = side_costs.to_vec();
            for (w, g) in self.groups.iter().enumerate() {
                for &i in g.iter().filter(|&&i| i < self.ducts.len()) {
                    self.owner[i] = w;
                }
            }
        }
        if threads == 1 {
            self.owner.fill(0);
        }
        self.threads_used = threads;
        let n_ducts = self.ducts.len();
        let n_primaries = self.primary_count;

        let mut substeps = 1;
        for d in self.ducts.iter_mut() {
            let s = d.substeps_for(dt);
            if s > substeps {
                substeps = s;
            }
        }

        let h = dt / substeps as f64;
        self.substep_dt = h;
        self.substep_noise_scale = math::sqrt(substeps as f64);
        self.result.mouth_flows.fill(0.0);
        self.result.valve_mass_flows.fill(0.0);
        self.result.heat_taken.fill(0.0);
        self.result.turbines.clear();
        self.result.turbines.resize(self.turbines.len(), TurbineResult::default());

        for step in 0..substeps {
            // 1. Reconstruct every duct, leaving the boundary faces unset, with the side work on the
            // first substep.
            let side_now = side.filter(|_| step == 0);
            if threads > 1 {
                let ducts = Disjoint::new(&mut self.ducts);
                let groups = &self.groups;
                pool.unwrap().run(threads, &|w| {
                    for &i in &groups[w] {
                        if i < n_ducts {
                            unsafe { ducts.get(i) }.begin_step(h);
                        } else if let Some(side) = side_now {
                            (side.job)(i - n_ducts);
                        }
                    }
                });
            } else {
                for d in self.ducts.iter_mut() {
                    d.begin_step(h);
                }
                if let Some(side) = side_now {
                    for k in 0..side.costs.len() {
                        (side.job)(k);
                    }
                }
            }
            // 2. Solve the junctions while every duct is mid-step.
            if !self.nodes.is_empty() {
                self.solve_junctions();
            }
            // 3. Each duct's own boundaries, its valve or junction source and its update: valve
            // walls and radiating mouths, the valve flux, and everything a junction feeds carrying
            // that junction's share of the mixing noise.
            let inputs = FinishInputs {
                h,
                last: step + 1 == substeps,
                primaries: n_primaries,
                valves,
                afterfire_heat: &self.afterfire_heat,
                zone_cells: &self.zone_cells,
                zone_volume: &self.zone_volume,
                fed_by_node: &self.fed_by_node,
                fed_flow: &self.fed_flow,
            };
            if threads > 1 {
                let ducts = Disjoint::new(&mut self.ducts);
                let out = Disjoint::new(&mut self.duct_out);
                let groups = &self.groups;
                pool.unwrap().run(threads, &|w| {
                    for &i in groups[w].iter().take_while(|&&i| i < n_ducts) {
                        unsafe { *out.get(i) = inputs.finish(i, ducts.get(i)) };
                    }
                });
            } else {
                for (i, d) in self.ducts.iter_mut().enumerate() {
                    self.duct_out[i] = inputs.finish(i, d);
                }
            }
            // 4. What they gave, gathered in duct order.
            for i in 0..n_ducts {
                let out = self.duct_out[i];
                let r_idx = self.radiating_index[i];
                if r_idx >= 0 {
                    self.result.mouth_flows[r_idx as usize] += out.mouth_flow;
                }
                if i < n_primaries {
                    self.result.valve_mass_flows[i] += out.valve_flow;
                    self.result.heat_taken[i] += out.heat;
                }
            }
        }

        let broken = self.duct_out.iter().any(|o| o.broken);

        let inv = if broken { 0.0 } else { 1.0 / substeps as f64 };
        for v in self.result.mouth_flows.iter_mut() {
            *v *= inv;
        }
        for v in self.result.valve_mass_flows.iter_mut() {
            *v *= inv;
        }
        let per = 1.0 / substeps as f64;
        for t in self.result.turbines.iter_mut() {
            t.power *= per;
            t.isentropic_power *= per;
            t.flow *= per;
            t.bypass_flow *= per;
            t.inlet *= per;
            t.outlet *= per;
        }
        self.result.substeps = substeps;
        &self.result
    }

    /// Constant-pressure junctions: the common pressure in closed form from the waves arriving,
    /// held within what the branches can justify, then Newton-corrected toward mass balance.
    fn solve_junctions(&mut self) {
        for ni in 0..self.nodes.len() {
            if let Some(slot) = self.turbine_slot[ni].filter(|&s| s < self.turbines.len()) {
                self.solve_turbine(ni, slot);
                continue;
            }
            let node = &mut self.nodes[ni];
            let n_out = node.outlets.len();
            let n_ends = n_out + node.inlets.len();
            for i in 0..n_out {
                node.states[i] = self.ducts[node.outlets[i]].end_state(DuctEnd::Outlet);
            }
            for i in 0..node.inlets.len() {
                node.states[n_out + i] = self.ducts[node.inlets[i]].end_state(DuctEnd::Inlet);
            }

            let mut num = 0.0;
            let mut den = 0.0;
            let mut p_min = f64::INFINITY;
            let mut p_max = 0.0;
            let mut m_in = 0.0;
            let mut h_in = 0.0;
            let mut scale_guess = 0.0;
            for i in 0..n_ends {
                let st = &node.states[i];
                let w = st.area / st.c;
                num += 2.0 * w * st.toward;
                den += w;
                if st.p < p_min {
                    p_min = st.p;
                }
                if st.p > p_max {
                    p_max = st.p;
                }
                scale_guess += (st.rho * st.area * st.u).abs();
                let into = if i < n_out { st.u } else { -st.u };
                if into > 0.0 {
                    let m = st.rho * st.area * into;
                    m_in += m;
                    h_in += m * (st.p / (st.rho * gas::R) + (into * into) / (2.0 * CP_EXH));
                }
            }

            // The temperature of the gas in the node: the mass-weighted stagnation temperature of
            // whatever is emptying into it.
            let fallback = node.states[node.states.len() - 1];
            let t_junction =
                if m_in > 1e-12 { h_in / m_in } else { fallback.p / (math::max(fallback.rho, 1e-7) * gas::R) };

            let mut gauge =
                clamp(gas::P_AMB + if den > 0.0 { num / den } else { 0.0 }, 0.3 * p_min, 3.0 * p_max) - gas::P_AMB;

            if den > 0.0 {
                let tol = JUNCTION_BALANCE_TOL * math::max(scale_guess, 1e-9);
                for _ in 0..2 {
                    let mut r = 0.0;
                    for i in 0..n_out {
                        let st = node.states[i];
                        r += self.ducts[node.outlets[i]].probe_junction(DuctEnd::Outlet, gauge, t_junction, &st);
                    }
                    for i in 0..node.inlets.len() {
                        let st = node.states[n_out + i];
                        r -= self.ducts[node.inlets[i]].probe_junction(DuctEnd::Inlet, gauge, t_junction, &st);
                    }
                    if !r.is_finite() || r.abs() <= tol {
                        break;
                    }
                    let next = clamp(gas::P_AMB + gauge + r / den, 0.3 * p_min, 3.0 * p_max) - gas::P_AMB;
                    if next == gauge {
                        break;
                    }
                    gauge = next;
                }
            }

            let mut signed = 0.0;
            let mut scale = 0.0;
            for i in 0..n_out {
                let st = node.states[i];
                let f = self.ducts[node.outlets[i]].apply_junction(DuctEnd::Outlet, gauge, t_junction, &st);
                signed += f;
                scale += f.abs();
            }
            for i in 0..node.inlets.len() {
                let st = node.states[n_out + i];
                let f = self.ducts[node.inlets[i]].apply_junction(DuctEnd::Inlet, gauge, t_junction, &st);
                signed -= f;
                scale += f.abs();
            }
            if scale > 1e-9 {
                let rel = signed.abs() / scale;
                if rel > self.junction_residual {
                    self.junction_residual = rel;
                }
            }

            self.update_merge_noise(ni);
        }
    }

    /// A junction with a turbine in it: the ducts emptying into it at one pressure, the ducts it feeds
    /// at another, and between them the turbine and its wastegate passing the flow Stodola's law gives
    /// for the two.
    ///
    /// Each side's flux is linear in its pressure near where it passes nothing, with slope `A/c`, as
    /// the ordinary junction has it. So for a trial flow `m` each side's pressure follows, and the flow
    /// the turbine passes between those two falls as `m` rises: the one `m` where the two agree is
    /// found by Newton's method, kept inside a bisection bracket. Each side's pressure is then corrected against the flux its ducts actually
    /// pass, and the flow solved again.
    fn solve_turbine(&mut self, ni: usize, slot: usize) {
        let setting = self.turbines[slot];
        let k_t = setting.k_turbine * setting.pulsation;
        let k_wg = setting.k_wastegate;
        let k = k_t + k_wg;

        let node = &mut self.nodes[ni];
        let n_out = node.outlets.len();
        let n_in = node.inlets.len();
        for i in 0..n_out {
            node.states[i] = self.ducts[node.outlets[i]].end_state(DuctEnd::Outlet);
        }
        for i in 0..n_in {
            node.states[n_out + i] = self.ducts[node.inlets[i]].end_state(DuctEnd::Inlet);
        }

        // Each side on its own: the pressure it would sit at passing nothing, its slope, its range, and
        // the stagnation temperature of what arrives from it.
        struct Side {
            zero: f64,
            slope: f64,
            lo: f64,
            hi: f64,
            m_in: f64,
            h_in: f64,
        }
        let side = |states: &[EndState], outlet: bool| {
            let (mut num, mut den, mut lo, mut hi, mut m_in, mut h_in) = (0.0, 0.0, f64::INFINITY, 0.0, 0.0, 0.0);
            for st in states {
                let w = st.area / st.c;
                num += 2.0 * w * st.toward;
                den += w;
                lo = math::min(lo, st.p);
                hi = math::max(hi, st.p);
                let into = if outlet { st.u } else { -st.u };
                if into > 0.0 {
                    let m = st.rho * st.area * into;
                    m_in += m;
                    h_in += m * (st.p / (st.rho * gas::R) + (into * into) / (2.0 * CP_EXH));
                }
            }
            Side {
                zero: if den > 0.0 { num / den } else { 0.0 },
                slope: math::max(den, 1e-12),
                lo: 0.3 * lo - gas::P_AMB,
                hi: 3.0 * hi - gas::P_AMB,
                m_in,
                h_in,
            }
        };
        let mut up = side(&node.states[..n_out], true);
        let mut down = side(&node.states[n_out..], false);
        let last = node.states[n_out - 1];
        let t_up = if up.m_in > 1e-12 { up.h_in / up.m_in } else { last.p / (math::max(last.rho, 1e-7) * gas::R) };
        let first = node.states[n_out];
        let t_down_back =
            if down.m_in > 1e-12 { down.h_in / down.m_in } else { first.p / (math::max(first.rho, 1e-7) * gas::R) };
        let sqrt_t = math::sqrt(math::max(t_up, 200.0));

        let stodola = |g_up: f64, g_down: f64| {
            let pu = gas::P_AMB + g_up;
            let pd = gas::P_AMB + g_down;
            let d = pu * pu - pd * pd;
            (k / sqrt_t) * math::sign(d) * math::sqrt(d.abs())
        };
        let pressures = |up: &Side, down: &Side, m: f64| {
            (clamp(up.zero - m / up.slope, up.lo, up.hi), clamp(down.zero + m / down.slope, down.lo, down.hi))
        };
        let solve = |up: &Side, down: &Side| {
            // No flow at `0`; at `m_eq` the two sides are at one pressure and the turbine passes nothing.
            // The root is between, found by Newton's method, falling back to bisection wherever a step
            // would leave the bracket.
            let m_eq = (up.zero - down.zero) / (1.0 / up.slope + 1.0 / down.slope);
            let (mut a, mut b) = if m_eq >= 0.0 { (0.0, m_eq) } else { (m_eq, 0.0) };
            let tol = 1e-4 * m_eq.abs() + 1e-9;
            let mut m = 0.5 * (a + b);
            for _ in 0..12 {
                let (gu, gd) = pressures(up, down, m);
                let h = stodola(gu, gd) - m;
                if h > 0.0 {
                    a = m;
                } else {
                    b = m;
                }
                // dM/dm, through each side's pressure, where it is not held at its limit.
                let pu = gas::P_AMB + gu;
                let pd = gas::P_AMB + gd;
                let d = math::max((pu * pu - pd * pd).abs(), 1e-6);
                let du = if gu > up.lo && gu < up.hi { -1.0 / up.slope } else { 0.0 };
                let dd = if gd > down.lo && gd < down.hi { 1.0 / down.slope } else { 0.0 };
                let slope = (k / sqrt_t) * (pu * du - pd * dd) / math::sqrt(d) - 1.0;
                let next = m - h / slope;
                let next = if slope < 0.0 && next > a && next < b { next } else { 0.5 * (a + b) };
                if (next - m).abs() <= tol {
                    return next;
                }
                m = next;
            }
            m
        };

        let mut m = solve(&up, &down);
        let (mut g_up, mut g_down) = pressures(&up, &down, m);
        let tol = JUNCTION_BALANCE_TOL * math::max(m.abs(), 1e-6);
        for _ in 0..4 {
            let mut f_up = 0.0;
            for i in 0..n_out {
                let st = node.states[i];
                f_up += self.ducts[node.outlets[i]].probe_junction(DuctEnd::Outlet, g_up, t_up, &st);
            }
            let mut f_down = 0.0;
            for i in 0..n_in {
                let st = node.states[n_out + i];
                f_down += self.ducts[node.inlets[i]].probe_junction(DuctEnd::Inlet, g_down, t_down_back, &st);
            }
            if !(f_up.is_finite() && f_down.is_finite()) || ((f_up - m).abs() <= tol && (f_down - m).abs() <= tol) {
                break;
            }
            // Move each side's line through the flux it actually passes, and solve again.
            up.zero = g_up + f_up / up.slope;
            down.zero = g_down - f_down / down.slope;
            m = solve(&up, &down);
            (g_up, g_down) = pressures(&up, &down, m);
        }

        // What leaves for the tailpipe: the turbine's share cooled by the work it did, the wastegate's not.
        let pu = gas::P_AMB + g_up;
        let pd = math::max(gas::P_AMB + g_down, 1e-3);
        let turbine_share = if k > 0.0 { k_t / k } else { 0.0 };
        let (power, isentropic_power, t_leaving) = if m > 0.0 && pu > pd {
            let isentropic = CP_EXH * t_up * (1.0 - math::pow(pd / pu, (gas::GAMMA_EXH - 1.0) / gas::GAMMA_EXH));
            let work = turbo::turbine_work(setting.tip_speed, isentropic);
            let m_t = turbine_share * m;
            (m_t * work, m_t * isentropic, math::max(t_up - (turbine_share * work) / CP_EXH, 200.0))
        } else {
            (0.0, 0.0, t_up)
        };

        let mut signed = 0.0;
        let mut scale = 0.0;
        for i in 0..n_out {
            let st = node.states[i];
            let f = self.ducts[node.outlets[i]].apply_junction(DuctEnd::Outlet, g_up, t_up, &st);
            signed += f;
            scale += f.abs();
        }
        for i in 0..n_in {
            let st = node.states[n_out + i];
            let f = self.ducts[node.inlets[i]].apply_junction(DuctEnd::Inlet, g_down, t_leaving, &st);
            signed -= f;
            scale += f.abs();
        }
        if scale > 1e-9 {
            let rel = signed.abs() / scale;
            if rel > self.junction_residual {
                self.junction_residual = rel;
            }
        }

        let r = &mut self.result.turbines[slot];
        r.power += power;
        r.isentropic_power += isentropic_power;
        r.flow += m * turbine_share;
        r.bypass_flow += m * (1.0 - turbine_share);
        r.inlet += pu;
        r.outlet += pd;

        self.update_merge_noise(ni);
        self.add_bypass_noise(ni, m * (1.0 - turbine_share), pu, t_up, setting.bypass_noise);
    }

    /// The jet through an open wastegate, into the ducts past the turbine: broadband turbulence,
    /// band-limited at the jet's Strouhal frequency, on top of the merge's own.
    fn add_bypass_noise(&mut self, ni: usize, bypass: f64, p: f64, t: f64, level: f64) {
        if bypass <= 0.0 || level <= 0.0 {
            return;
        }
        let substep_dt = self.substep_dt;
        let noise_scale = self.substep_noise_scale;
        let node = &self.nodes[ni];
        let st = node.states[node.outlets.len()];
        let rho = p / (gas::R * math::max(t, 200.0));
        let u = bypass / (rho * math::max(st.area, 1e-6));
        let dia = math::sqrt((4.0 * st.area) / PI);
        let strouhal_hz = (0.2 * u) / math::max(dia, 1e-3);
        let k = clamp(1.0 - math::exp(-2.0 * PI * strouhal_hz * substep_dt), 1e-4, 0.9);
        let sigma = BYPASS_TURBULENCE * self.turbulence * level * bypass;
        let tn = &mut self.turbine_nodes[ni];
        let white = tn.noise.next() * sigma * noise_scale;
        tn.lp1 += k * (white - tn.lp1);
        tn.lp2 += k * (tn.lp1 - tn.lp2);
        for &(slot, share) in &self.nodes[ni].downstream {
            self.fed_flow[slot] += tn.lp2 * share;
        }
    }

    /// Broadband mixing noise for one junction, driven by the shear between its branches and
    /// band-limited at the merge's Strouhal frequency, injected into whatever the node feeds.
    fn update_merge_noise(&mut self, ni: usize) {
        let turbulence = self.turbulence;
        let substep_dt = self.substep_dt;
        let noise_scale = self.substep_noise_scale;
        let node = &mut self.nodes[ni];
        let n_out = node.outlets.len();

        let mut area_sum = 0.0;
        let mut flow_sum = 0.0;
        for st in &node.states[..n_out] {
            area_sum += st.area;
            flow_sum += st.area * st.u;
        }
        let u_bar = if area_sum > 0.0 { flow_sum / area_sum } else { 0.0 };
        let mut shear_sq = 0.0;
        for st in &node.states[..n_out] {
            let d = st.u - u_bar;
            shear_sq += st.area * d * d;
        }
        let shear = if area_sum > 0.0 { math::sqrt(shear_sq / area_sum) } else { 0.0 };
        let u_mix = math::hypot2(shear, 0.2 * u_bar);

        let inlet = if n_out < node.states.len() { node.states[n_out] } else { node.states[node.states.len() - 1] };
        let rho = math::max(inlet.rho, 1e-6);
        let sigma = MERGE_TURBULENCE * turbulence * rho * inlet.area * u_mix;

        let dia = math::sqrt((4.0 * inlet.area) / PI);
        let strouhal_hz = (0.2 * u_mix) / math::max(dia, 1e-3);
        let k = clamp(1.0 - math::exp(-2.0 * PI * strouhal_hz * substep_dt), 1e-4, 0.9);

        let white = node.noise.next() * sigma * noise_scale;
        node.lp1 += k * (white - node.lp1);
        node.lp2 += k * (node.lp1 - node.lp2);

        for &(slot, share) in &node.downstream {
            self.fed_flow[slot] = node.lp2 * share;
            let (d, _) = self.fed_by_node[slot];
            let (p, t, a) = self.ducts[d].read_port();
            let valve = &mut self.fed_by_node[slot].1;
            valve.cyl_temp = t;
            valve.cyl_pressure = p;
            valve.throat_area = a;
        }
    }

    /// The duct the display shows: the collector, or cylinder 0's duct if there is none.
    fn shown(&self) -> &EulerPipe {
        self.collector().unwrap_or(&self.ducts[0])
    }

    /// Gauge pressure along the shown duct, for the display.
    pub fn sample_pressure(&self, out: &mut [f32]) {
        self.shown().sample_pressure(out);
    }

    /// Gauge pressure in every cell of every duct, Pa, into `out`: the ducts in the graph's order, each
    /// port end first. `cells` takes each duct's cell count.
    pub fn sample_duct_pressures(&self, out: &mut Vec<f32>, cells: &mut Vec<u32>) {
        out.clear();
        cells.clear();
        for &d in &self.by_graph {
            let start = out.len();
            self.ducts[d].push_cell_pressures(out);
            cells.push((out.len() - start) as u32);
        }
    }

    pub fn sample_wall_temperature(&self, out: &mut [f32]) {
        self.shown().sample_wall_temperature(out);
    }

    pub fn mean_wall_temp(&self) -> f64 {
        let mut sum = 0.0;
        for d in &self.ducts {
            sum += d.mean_wall_temp();
        }
        sum / self.ducts.len() as f64
    }

    /// First quarter-wave resonance of everything between cylinder 0's valve and open air, Hz.
    pub fn quarter_wave_hz(&self) -> f64 {
        if self.air_path.is_empty() {
            return self.ducts[0].quarter_wave_hz();
        }
        let mut inv_total = 0.0;
        for &d in &self.air_path {
            inv_total += 1.0 / self.ducts[d].quarter_wave_hz();
        }
        1.0 / inv_total
    }

    pub fn export_wall(&self) -> Vec<f64> {
        self.shown().export_wall()
    }

    /// The radiation corner of radiating mouth `m`.
    pub fn mouth_cutoff_rad_of(&self, m: usize) -> f64 {
        self.ducts[self.radiating[m]].mouth_cutoff_rad
    }

    /// The plane-wave limit on mouth `m`'s radiation, rad/s.
    pub fn plane_wave_cutoff_rad_of(&self, m: usize) -> f64 {
        self.ducts[self.radiating[m]].plane_wave_cutoff_rad
    }

    /// The highest frequency the grid resolves at mouth `m`, rad/s.
    pub fn resolution_cutoff_rad_of(&self, m: usize) -> f64 {
        self.ducts[self.radiating[m]].resolution_cutoff_rad
    }

    /// The id of the duct radiating from mouth `m`, as the graph names it.
    pub fn mouth_duct_id(&self, m: usize) -> Option<&str> {
        let d = *self.radiating.get(m)?;
        let g = self.by_graph.iter().position(|&p| p == d)?;
        self.duct_ids.get(g).map(String::as_str)
    }

    /// The duct radiating from mouth `m`.
    /// Threads the last `advance_on` stepped the ducts on, and which of them stepped duct `i`: where
    /// what reads that duct's cells next is best done, as they are in that thread's cache.
    pub fn threads_used(&self) -> usize {
        self.threads_used
    }

    pub fn owner_of(&self, i: usize) -> usize {
        self.owner[i]
    }

    /// Position in `ducts` of radiating mouth `m`'s duct.
    pub fn radiating_duct_index(&self, m: usize) -> usize {
        self.radiating[m]
    }

    pub fn radiating_duct(&self, m: usize) -> &EulerPipe {
        &self.ducts[self.radiating[m]]
    }

    pub fn set_air_speed(&mut self, v: f64) {
        for d in self.ducts.iter_mut() {
            d.set_air_speed(v);
        }
    }
}
