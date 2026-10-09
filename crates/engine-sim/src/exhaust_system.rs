//! The exhaust as a whole: an `ExhaustGraph` of ducts joined at junctions, with one duct fed by each
//! cylinder's exhaust valve, all marched in lockstep.
//!
//! A junction is where cylinders hear each other: each blowdown pulse arrives there and partly
//! travels up the other primaries, where it helps scavenge those cylinders or blocks them depending
//! on where the firing interval puts it.

use std::cmp::Reverse;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

use crate::afterfire::AFTERFIRE_ZONE_LENGTH;
use crate::dsp::Noise;
use crate::euler_pipe::{
    DuctEnd, EndState, EulerPipe, EulerPipeOptions, InletKind, JunctionEnd, JunctionFace, OutletKind, ValveState,
};
use crate::exhaust_graph::{
    DuctRole, End, ExhaustGraph, TurboMount, ends_at, node_order, path_to_air, radiating_ducts, validate_graph,
    valve_ducts,
};
use crate::math::{self, PI, clamp};
use crate::pool::{CachePadded, Disjoint, ThreadPool, wait_until};
use crate::spec::gas;
use crate::turbo;

/// Turbulence intensity of the merge, as a fraction of the mixing mass flow.
const MERGE_TURBULENCE: f64 = 0.14;

/// Specific heat at constant pressure for exhaust gas, J/(kg K).
const CP_EXH: f64 = (gas::GAMMA_EXH * gas::R) / (gas::GAMMA_EXH - 1.0);

/// Relative mass-flux imbalance below which a junction is left alone: a plain junction's, against the
/// flow through it, and a turbine junction's, against the turbine's. A plain junction takes two
/// trials at most, so it is held closer.
const JUNCTION_BALANCE_TOL: f64 = 0.0005;
const TURBINE_BALANCE_TOL: f64 = 0.0005;

/// The range a junction's measured flux slope is trusted over, as a share of the acoustic estimate.
const SLOPE_GAIN_MIN: f64 = 1.0;
const SLOPE_GAIN_MAX: f64 = 5.0;

/// Fewest cells worth handing a thread of their own: below it, handing off costs more than it saves.
const MIN_CELLS_PER_THREAD: usize = 40;

/// The threads are balanced on what each duct, with what goes with it, has been timed to take: one
/// sample in `TIME_EVERY` is timed, each timing folded into a running average by `COST_SMOOTHING`,
/// and every `REBALANCE_EVERY` samples the ducts are dealt out afresh if that would cut the busiest
/// thread's share by `REBALANCE_GAIN`. Moving a duct moves its cells to another core's cache, so it is
/// not done for less.
const TIME_EVERY: u64 = 61;
const COST_SMOOTHING: f64 = 0.005;
const REBALANCE_EVERY: u64 = 4096;
const REBALANCE_GAIN: f64 = 0.1;

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
    /// End states, outlets first then inlets, and each as the boundary takes it.
    states: Vec<EndState>,
    ends: Vec<JunctionEnd>,
    /// Ducts leaving the node, as slots of `fed_by_node`, and the share of the mixing noise each takes.
    downstream: Vec<(usize, f64)>,
    noise: Noise,
    lp1: f64,
    lp2: f64,
    /// How steeply its ducts' flux actually follows its pressure, as a share of the acoustic estimate
    /// `A / c`, as its last solve measured it: on the one pressure, or a turbine's two sides.
    gain: [f64; 2],
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
    /// What each duct's own boundaries and update gave over the sample, gathered in duct order.
    duct_out: Vec<DuctOut>,
    /// Per duct, what it puts ahead of each substep's reconstruction for the junctions at its ends;
    /// and per duct end, by `2 * duct + end`, the junction boundary solved there.
    ahead: Vec<Ahead>,
    junction_faces: Vec<CachePadded<JunctionFace>>,
    /// Per duct, the stamp of the substep it last put its ends ahead for and was last finished for;
    /// per side item, the first substep of the last sample it was stepped for. Each is set by the
    /// thread that does the work, for the threads that read what it leaves.
    ahead_done: Vec<CachePadded<AtomicU64>>,
    duct_done: Vec<CachePadded<AtomicU64>>,
    side_done: Vec<CachePadded<AtomicU64>>,
    /// What each thread steps, for a pool of `groups_for` threads alongside side work costing
    /// `groups_side`: ducts by their index, side items past them.
    groups: Vec<Vec<usize>>,
    groups_for: usize,
    groups_side: Vec<usize>,
    /// Per duct and side item, with what `note_time` adds: what it took in the last timed sample, ns,
    /// and its running average; `costs_known` once there is one.
    unit_time: Vec<CachePadded<f64>>,
    unit_cost: Vec<f64>,
    costs_known: bool,
    /// Samples advanced, whether this one is timed, and samples since the threads were last balanced.
    samples: u64,
    timing: bool,
    since_balance: u64,
    /// Each junction reads what its ducts put ahead and writes only its own boundaries and what it
    /// feeds, so each can be solved on a thread of its own. Per duct, the junction at each end, inlet
    /// then outlet, if any. Per junction, the ducts it reads, those ending at it and those it feeds;
    /// its anchor, the longest duct ending at it, on whose thread it is solved and to whose time its
    /// own is put; its stamp once solved this substep, and its worst imbalance yet. Per thread, the
    /// junctions it solves; and the junctions with a turbine.
    duct_nodes: Vec<[Option<usize>; 2]>,
    node_reads: Vec<Vec<usize>>,
    node_anchor: Vec<usize>,
    node_done: Vec<CachePadded<AtomicU64>>,
    node_residual: Vec<CachePadded<f64>>,
    thread_nodes: Vec<Vec<usize>>,
    turbine_junctions: Vec<usize>,
    /// Substeps advanced, the stamp each part of a substep is marked done with.
    substeps_done: u64,
    /// The thread each duct and side item was stepped on in the last `advance_on`, and how many there
    /// were; and which dealing out of them that was, by `groups_id`.
    owner: Vec<usize>,
    threads_used: usize,
    groups_id: u64,
}

/// What a duct puts ahead of a substep's reconstruction for the junctions at its ends, on cache lines
/// of its own: its end states, inlet then outlet, as the reconstruction will leave them, and its port
/// as `read_port` reads it, which the reconstruction leaves alone.
#[derive(Clone, Copy, Debug, Default)]
#[repr(align(128))]
struct Ahead {
    ends: [EndState; 2],
    port: (f64, f64, f64),
}

impl Ahead {
    /// Ahead of `duct`'s reconstruction over `dt`.
    fn of(duct: &EulerPipe, dt: f64) -> Ahead {
        Ahead {
            ends: [duct.end_state_ahead(DuctEnd::Inlet, dt), duct.end_state_ahead(DuctEnd::Outlet, dt)],
            port: duct.read_port(),
        }
    }

    /// The same, taken from `duct` once it is reconstructed, as one thread does.
    fn after(duct: &EulerPipe) -> Ahead {
        Ahead { ends: [duct.end_state(DuctEnd::Inlet), duct.end_state(DuctEnd::Outlet)], port: duct.read_port() }
    }
}

/// The last `ExhaustSystem::groups_id` given out, by any exhaust.
static GROUPS_DEALT: AtomicU64 = AtomicU64::new(0);

/// One duct's part of an `ExhaustResult`, over one substep or summed over a sample, on a cache line of
/// its own, as each duct's may be written from a thread of its own.
#[derive(Clone, Copy, Debug, Default)]
#[repr(align(128))]
struct DuctOut {
    mouth_flow: f64,
    valve_flow: f64,
    heat: f64,
    /// On the last substep: whether the duct's state had gone inadmissible, and was reset; and the
    /// pressure in its first cell, Pa, as `read_port` gives it, for its valve next sample.
    broken: bool,
    port_pressure: f64,
}

impl DuctOut {
    /// Fold one of a sample's `substeps` into it, the first starting it afresh. On the last, the flows
    /// become their mean over the sample, or nothing from a duct that broke.
    fn add(&mut self, step: DuctOut, first: bool, last: bool, substeps: usize) {
        if first {
            self.mouth_flow = 0.0;
            self.valve_flow = 0.0;
            self.heat = 0.0;
        }
        self.mouth_flow += step.mouth_flow;
        self.valve_flow += step.valve_flow;
        self.heat += step.heat;
        if last {
            self.broken = step.broken;
            self.port_pressure = step.port_pressure;
            let inv = if step.broken { 0.0 } else { 1.0 / substeps as f64 };
            self.mouth_flow *= inv;
            self.valve_flow *= inv;
        }
    }
}

impl TurbineResult {
    /// From its sum over a sample's substeps to its mean, `per` being one over their count.
    fn average(&mut self, per: f64) {
        self.power *= per;
        self.isentropic_power *= per;
        self.flow *= per;
        self.bypass_flow *= per;
        self.inlet *= per;
        self.outlet *= per;
    }
}

/// The exhaust as `advance_on` leaves it, for the work it is given to do once the ducts are stepped:
/// each part read once the thread stepping it is done with it, waiting for that if it is not yet.
pub struct Stepped<'a> {
    ducts: &'a Disjoint<'a, EulerPipe>,
    out: &'a Disjoint<'a, DuctOut>,
    turbines: &'a Disjoint<'a, TurbineResult>,
    radiating: &'a [usize],
    zone_cells: &'a [usize],
    zone_volume: &'a [f64],
    /// On threads, the stamps to wait for; on one, everything is done before the work starts.
    progress: Option<Progress<'a>>,
}

struct Progress<'a> {
    duct_done: &'a [CachePadded<AtomicU64>],
    side_done: &'a [CachePadded<AtomicU64>],
    node_done: &'a [CachePadded<AtomicU64>],
    turbine_junctions: &'a [usize],
    /// The sample's first substep, and its last.
    first: u64,
    last: u64,
}

impl Stepped<'_> {
    fn wait_duct(&self, i: usize) {
        if let Some(p) = &self.progress {
            wait_until(&p.duct_done[i], p.last);
        }
    }

    /// Duct `i`, stepped through the sample.
    pub fn duct(&self, i: usize) -> &EulerPipe {
        self.wait_duct(i);
        // Its thread is done with it until the next sample.
        unsafe { self.ducts.get_ref(i) }
    }

    /// Cylinder `b`'s primary.
    pub fn primary(&self, b: usize) -> &EulerPipe {
        self.duct(b)
    }

    /// The duct radiating from mouth `m`.
    pub fn radiating_duct(&self, m: usize) -> &EulerPipe {
        self.duct(self.radiating[m])
    }

    fn out(&self, i: usize) -> &DuctOut {
        self.wait_duct(i);
        unsafe { self.out.get_ref(i) }
    }

    /// `ExhaustResult::valve_mass_flows[b]`.
    pub fn valve_mass_flow(&self, b: usize) -> f64 {
        self.out(b).valve_flow
    }

    /// `ExhaustResult::heat_taken[b]`.
    pub fn heat_taken(&self, b: usize) -> f64 {
        self.out(b).heat
    }

    /// `ExhaustResult::mouth_flows[m]`.
    pub fn mouth_flow(&self, m: usize) -> f64 {
        self.out(self.radiating[m]).mouth_flow
    }

    /// `ExhaustResult::turbines`.
    pub fn turbines(&self) -> &[TurbineResult] {
        if let Some(p) = &self.progress {
            for &ni in p.turbine_junctions {
                wait_until(&p.node_done[ni], p.last);
            }
        }
        // Each is written only by its junction's thread, which is done with it.
        unsafe { self.turbines.as_slice() }
    }

    /// Wait for side item `k` to be stepped, before reading what it leaves.
    pub fn wait_side(&self, k: usize) {
        if let Some(p) = &self.progress {
            wait_until(&p.side_done[k], p.first);
        }
    }

    /// Cylinder `b`'s afterfire zone: its primary's leading cells, how many, and their volume, m^3.
    pub fn afterfire_zone(&self, b: usize) -> (usize, f64) {
        (self.zone_cells[b], self.zone_volume[b])
    }
}

/// What a duct reads to finish a substep, shared by every thread.
struct FinishInputs<'a> {
    h: f64,
    last: bool,
    primaries: usize,
    valves: &'a (dyn Fn(usize) -> ValveState + Sync),
    afterfire_heat: &'a [f64],
    zone_cells: &'a [usize],
    zone_volume: &'a [f64],
    fed_by_node: &'a Disjoint<'a, (usize, ValveState)>,
    fed_flow: &'a Disjoint<'a, f64>,
    faces: &'a Disjoint<'a, CachePadded<JunctionFace>>,
    duct_nodes: &'a [[Option<usize>; 2]],
}

impl FinishInputs<'_> {
    /// Duct `i`'s junction boundaries, own boundaries, valve or junction source, conservative update and thermal pass,
    /// and on the last substep its recovery if it has broken. It touches no other duct, so the ducts
    /// can be finished in any order, or at once, and each duct's cells stay with the thread that
    /// steps it.
    fn finish(&self, i: usize, duct: &mut EulerPipe) -> DuctOut {
        let h = self.h;
        for end in [DuctEnd::Inlet, DuctEnd::Outlet] {
            if self.duct_nodes[i][end as usize].is_some() {
                // Solved by the junction there, which is done with it.
                duct.take_junction_face(end, unsafe { &mut self.faces.get(2 * i + end as usize).0 });
            }
        }
        let mut out = DuctOut { mouth_flow: duct.apply_own_boundaries(h), ..DuctOut::default() };
        if i < self.primaries {
            let valve = (self.valves)(i);
            let flow = duct.valve_flux_for(&valve, h);
            duct.set_end_step(h, flow + valve.extra_mass_flow);
            duct.end_step_set(&valve);
            out.valve_flow = flow * duct.source_scale;
            let heat = self.afterfire_heat[i];
            if heat > 0.0 {
                out.heat = duct.add_heat(heat * h, self.zone_cells[i], self.zone_volume[i]);
            }
        } else {
            // Every duct past the primaries is node-fed, and `fed_by_node` holds them in duct order.
            let slot = i - self.primaries;
            // Written by the junction feeding it, which has been solved.
            let (flow, valve) = unsafe { (*self.fed_flow.get(slot), self.fed_by_node.get(slot).1) };
            duct.set_end_step(h, flow);
            duct.end_step_set(&valve);
        }
        duct.after_step(h);
        if self.last {
            out.broken = duct.recover_if_broken();
            out.port_pressure = duct.read_port().0;
        }
        out
    }
}

/// What solving the junctions reaches into, each junction's own part handed to the one thread that
/// solves it: its node, what its ducts put ahead, its boundary at each, and the ducts it feeds. No
/// two junctions write any of the same, so they can be solved at once.
struct Junctions<'a> {
    ahead: &'a Disjoint<'a, Ahead>,
    faces: &'a Disjoint<'a, CachePadded<JunctionFace>>,
    nodes: Disjoint<'a, JunctionNode>,
    turbine_nodes: Disjoint<'a, TurbineNode>,
    turbine_results: Disjoint<'a, TurbineResult>,
    fed_by_node: &'a Disjoint<'a, (usize, ValveState)>,
    fed_flow: &'a Disjoint<'a, f64>,
    turbine_slot: &'a [Option<usize>],
    turbines: &'a [TurbineSetting],
    turbulence: f64,
    substep_dt: f64,
    substep_noise_scale: f64,
}

impl Junctions<'_> {
    // Each accessor hands out what only the junction being solved, on the thread solving it, touches,
    // or what its ducts' threads have put ahead for it.

    /// Duct `d`'s end state at `end`.
    fn end(&self, d: usize, end: DuctEnd) -> EndState {
        unsafe { self.ahead.get_ref(d) }.ends[end as usize]
    }

    /// The boundary at duct `d`'s `end`.
    #[allow(clippy::mut_from_ref)]
    fn face(&self, d: usize, end: DuctEnd) -> &mut JunctionFace {
        unsafe { &mut self.faces.get(2 * d + end as usize).0 }
    }

    #[allow(clippy::mut_from_ref)]
    fn node(&self, ni: usize) -> &mut JunctionNode {
        unsafe { self.nodes.get(ni) }
    }

    #[allow(clippy::mut_from_ref)]
    fn fed(&self, slot: usize) -> &mut (usize, ValveState) {
        unsafe { self.fed_by_node.get(slot) }
    }

    #[allow(clippy::mut_from_ref)]
    fn fed_flow(&self, slot: usize) -> &mut f64 {
        unsafe { self.fed_flow.get(slot) }
    }

    /// The end states of `node`'s ducts, outlets then inlets, each as its boundary takes it, with what
    /// the boundaries last probed against the ones before forgotten.
    fn read_ends(&self, node: &mut JunctionNode) {
        let n_out = node.outlets.len();
        for (k, &d) in node.outlets.iter().chain(&node.inlets).enumerate() {
            let end = if k < n_out { DuctEnd::Outlet } else { DuctEnd::Inlet };
            node.states[k] = self.end(d, end);
            node.ends[k] = JunctionEnd::new(&node.states[k]);
            self.face(d, end).forget_probe();
        }
    }

    /// Junction `ni`, at constant pressure: the common pressure in closed form from the waves arriving,
    /// held within what the branches can justify, then Newton-corrected toward mass balance. Returns
    /// the relative mass imbalance it is left with.
    fn solve(&self, ni: usize) -> f64 {
        if let Some(slot) = self.turbine_slot[ni].filter(|&s| s < self.turbines.len()) {
            return self.solve_turbine(ni, slot);
        }
        let node = self.node(ni);
        self.read_ends(node);
        let n_out = node.outlets.len();
        let n_ends = n_out + node.inlets.len();

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
        let t_junction = if m_in > 1e-12 { h_in / m_in } else { fallback.p / (math::max(fallback.rho, 1e-7) * gas::R) };

        let mut gauge =
            clamp(gas::P_AMB + if den > 0.0 { num / den } else { 0.0 }, 0.3 * p_min, 3.0 * p_max) - gas::P_AMB;

        if den > 0.0 {
            let tol = JUNCTION_BALANCE_TOL * math::max(scale_guess, 1e-9);
            // The first step along the slope the last solve measured, each after it along the secant
            // through the last two trials.
            let mut slope = den * node.gain[0];
            let mut last: Option<(f64, f64)> = None;
            for _ in 0..2 {
                let mut r = 0.0;
                for i in 0..n_out {
                    let st = &node.ends[i];
                    r += self.face(node.outlets[i], DuctEnd::Outlet).flux(DuctEnd::Outlet, gauge, t_junction, false, st);
                }
                for i in 0..node.inlets.len() {
                    let st = &node.ends[n_out + i];
                    r -= self.face(node.inlets[i], DuctEnd::Inlet).flux(DuctEnd::Inlet, gauge, t_junction, false, st);
                }
                if !r.is_finite() || r.abs() <= tol {
                    break;
                }
                if let Some((g0, r0)) = last {
                    let measured = (r0 - r) / (gauge - g0);
                    if measured.is_finite() && measured > 0.0 {
                        slope = clamp(measured, SLOPE_GAIN_MIN * den, SLOPE_GAIN_MAX * den);
                        node.gain[0] = slope / den;
                    }
                }
                last = Some((gauge, r));
                let next = clamp(gas::P_AMB + gauge + r / slope, 0.3 * p_min, 3.0 * p_max) - gas::P_AMB;
                if next == gauge {
                    break;
                }
                gauge = next;
            }
        }

        let mut signed = 0.0;
        let mut scale = 0.0;
        for i in 0..n_out {
            let st = &node.ends[i];
            let f = self.face(node.outlets[i], DuctEnd::Outlet).flux(DuctEnd::Outlet, gauge, t_junction, true, st);
            signed += f;
            scale += f.abs();
        }
        for i in 0..node.inlets.len() {
            let st = &node.ends[n_out + i];
            let f = self.face(node.inlets[i], DuctEnd::Inlet).flux(DuctEnd::Inlet, gauge, t_junction, true, st);
            signed -= f;
            scale += f.abs();
        }
        self.update_merge_noise(ni);
        if scale > 1e-9 { signed.abs() / scale } else { 0.0 }
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
    fn solve_turbine(&self, ni: usize, slot: usize) -> f64 {
        let setting = self.turbines[slot];
        let k_t = setting.k_turbine * setting.pulsation;
        let k_wg = setting.k_wastegate;
        let k = k_t + k_wg;

        let node = self.node(ni);
        self.read_ends(node);
        let n_out = node.outlets.len();
        let n_in = node.inlets.len();

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
        // Each side's slope as the last solve measured it, and after that along the secant through its
        // last two trials.
        let acoustic = [up.slope, down.slope];
        up.slope = acoustic[0] * node.gain[0];
        down.slope = acoustic[1] * node.gain[1];
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
        let tol = TURBINE_BALANCE_TOL * math::max(m.abs(), 1e-6);
        let mut last: Option<(f64, f64, f64, f64)> = None;
        for _ in 0..4 {
            let mut f_up = 0.0;
            for i in 0..n_out {
                let st = &node.ends[i];
                f_up += self.face(node.outlets[i], DuctEnd::Outlet).flux(DuctEnd::Outlet, g_up, t_up, false, st);
            }
            let mut f_down = 0.0;
            for i in 0..n_in {
                let st = &node.ends[n_out + i];
                f_down += self.face(node.inlets[i], DuctEnd::Inlet).flux(DuctEnd::Inlet, g_down, t_down_back, false, st);
            }
            if !(f_up.is_finite() && f_down.is_finite()) || ((f_up - m).abs() <= tol && (f_down - m).abs() <= tol) {
                break;
            }
            if let Some((gu0, fu0, gd0, fd0)) = last {
                let su = (fu0 - f_up) / (g_up - gu0);
                if su.is_finite() && su > 0.0 {
                    up.slope = clamp(su, SLOPE_GAIN_MIN * acoustic[0], SLOPE_GAIN_MAX * acoustic[0]);
                    node.gain[0] = up.slope / acoustic[0];
                }
                let sd = (f_down - fd0) / (g_down - gd0);
                if sd.is_finite() && sd > 0.0 {
                    down.slope = clamp(sd, SLOPE_GAIN_MIN * acoustic[1], SLOPE_GAIN_MAX * acoustic[1]);
                    node.gain[1] = down.slope / acoustic[1];
                }
            }
            last = Some((g_up, f_up, g_down, f_down));
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
            let st = &node.ends[i];
            let f = self.face(node.outlets[i], DuctEnd::Outlet).flux(DuctEnd::Outlet, g_up, t_up, true, st);
            signed += f;
            scale += f.abs();
        }
        for i in 0..n_in {
            let st = &node.ends[n_out + i];
            let f = self.face(node.inlets[i], DuctEnd::Inlet).flux(DuctEnd::Inlet, g_down, t_leaving, true, st);
            signed -= f;
            scale += f.abs();
        }
        let residual = if scale > 1e-9 { signed.abs() / scale } else { 0.0 };

        let r = unsafe { self.turbine_results.get(slot) };
        r.power += power;
        r.isentropic_power += isentropic_power;
        r.flow += m * turbine_share;
        r.bypass_flow += m * (1.0 - turbine_share);
        r.inlet += pu;
        r.outlet += pd;

        self.update_merge_noise(ni);
        self.add_bypass_noise(ni, m * (1.0 - turbine_share), pu, t_up, setting.bypass_noise);
        residual
    }

    /// The jet through an open wastegate, into the ducts past the turbine: broadband turbulence,
    /// band-limited at the jet's Strouhal frequency, on top of the merge's own.
    fn add_bypass_noise(&self, ni: usize, bypass: f64, p: f64, t: f64, level: f64) {
        if bypass <= 0.0 || level <= 0.0 {
            return;
        }
        let substep_dt = self.substep_dt;
        let noise_scale = self.substep_noise_scale;
        let node = self.node(ni);
        let st = node.states[node.outlets.len()];
        let rho = p / (gas::R * math::max(t, 200.0));
        let u = bypass / (rho * math::max(st.area, 1e-6));
        let dia = math::sqrt((4.0 * st.area) / PI);
        let strouhal_hz = (0.2 * u) / math::max(dia, 1e-3);
        let k = clamp(1.0 - math::exp(-2.0 * PI * strouhal_hz * substep_dt), 1e-4, 0.9);
        let sigma = BYPASS_TURBULENCE * self.turbulence * level * bypass;
        let tn = unsafe { self.turbine_nodes.get(ni) };
        let white = tn.noise.next() * sigma * noise_scale;
        tn.lp1 += k * (white - tn.lp1);
        tn.lp2 += k * (tn.lp1 - tn.lp2);
        for &(slot, share) in &node.downstream {
            *self.fed_flow(slot) += tn.lp2 * share;
        }
    }

    /// Broadband mixing noise for one junction, driven by the shear between its branches and
    /// band-limited at the merge's Strouhal frequency, injected into whatever the node feeds.
    fn update_merge_noise(&self, ni: usize) {
        let turbulence = self.turbulence;
        let substep_dt = self.substep_dt;
        let noise_scale = self.substep_noise_scale;
        let node = self.node(ni);
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
            *self.fed_flow(slot) = node.lp2 * share;
            let fed = self.fed(slot);
            let (p, t, a) = unsafe { self.ahead.get_ref(fed.0) }.port;
            let valve = &mut fed.1;
            valve.cyl_temp = t;
            valve.cyl_pressure = p;
            valve.throat_area = a;
        }
    }
}

/// Work that is independent of the exhaust for a sample, stepped alongside its ducts: `costs[k]` is
/// what item `k` costs, in cells, and `job(k)` steps it. Each item is stepped once per `advance_on`,
/// in its first substep, item `b` before primary `b`'s valve is read.
pub struct SideWork<'a> {
    pub costs: &'a [usize],
    pub job: &'a (dyn Fn(usize) + Sync),
}

/// Work that reads the exhaust once a sample has stepped it: called once on each thread `advance_on`
/// runs, with that thread's index, after the thread has done its own part of the sample; on one, once
/// the sample is done. What it reads of the exhaust, it reads through `Stepped`, which waits for it.
pub type After<'a> = dyn Fn(usize, &Stepped<'_>) + Sync + 'a;

/// Ducts and side items split into `threads` groups of about equal cost, the costliest placed first,
/// each onto the group with the least so far; each group in order, ducts first.
fn balance(costs: &[f64], threads: usize) -> Vec<Vec<usize>> {
    let mut order: Vec<usize> = (0..costs.len()).collect();
    order.sort_by(|&a, &b| costs[b].total_cmp(&costs[a]));
    let mut groups: Vec<Vec<usize>> = vec![Vec::new(); threads];
    let mut load = vec![0.0f64; threads];
    for i in order {
        let g = (0..threads).min_by(|&a, &b| load[a].total_cmp(&load[b])).unwrap();
        groups[g].push(i);
        load[g] += costs[i];
    }
    for g in groups.iter_mut() {
        g.sort();
    }
    groups
}

/// The busiest group's cost.
fn busiest(groups: &[Vec<usize>], costs: &[f64]) -> f64 {
    groups.iter().map(|g| g.iter().map(|&i| costs[i]).sum::<f64>()).fold(0.0, f64::max)
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
                ends: vec![JunctionEnd::default(); all.len()],
                outlets: outlet_ends,
                inlets: inlet_ends,
                downstream,
                noise: Noise::new(0x7f4a3b as f64 + n as f64 * 0x9e3779b as f64),
                lp1: 0.0,
                lp2: 0.0,
                gain: [1.0; 2],
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
        let mut duct_nodes = vec![[None; 2]; ducts.len()];
        for (ni, node) in nodes.iter().enumerate() {
            for &d in &node.outlets {
                duct_nodes[d][DuctEnd::Outlet as usize] = Some(ni);
            }
            for &d in &node.inlets {
                duct_nodes[d][DuctEnd::Inlet as usize] = Some(ni);
            }
        }
        let node_reads: Vec<Vec<usize>> = nodes
            .iter()
            .map(|node| {
                let fed = node.downstream.iter().map(|&(slot, _)| valve_fed.len() + slot);
                let mut reads: Vec<usize> = node.outlets.iter().chain(&node.inlets).copied().chain(fed).collect();
                reads.sort();
                reads.dedup();
                reads
            })
            .collect();
        let node_anchor: Vec<usize> = nodes
            .iter()
            .map(|node| *node.outlets.iter().chain(&node.inlets).max_by_key(|&&d| (ducts[d].n, Reverse(d))).unwrap())
            .collect();
        let junction_faces = (0..2 * ducts.len())
            .map(|k| {
                let end = if k % 2 == 0 { DuctEnd::Inlet } else { DuctEnd::Outlet };
                CachePadded(JunctionFace::new(ducts[k / 2].end_face_area(end)))
            })
            .collect();

        let mut turbine_mounts = Vec::new();
        let turbine_slot: Vec<Option<usize>> = order
            .iter()
            .map(|id| {
                let mount = graph.turbos.iter().find(|t| &t.node == id)?;
                turbine_mounts.push(mount.clone());
                Some(turbine_mounts.len() - 1)
            })
            .collect();
        let turbine_junctions = (0..nodes.len()).filter(|&ni| turbine_slot[ni].is_some()).collect();
        let node_count = nodes.len();
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

        let duct_out = ducts.iter().map(|d| DuctOut { port_pressure: d.read_port().0, ..DuctOut::default() }).collect();
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
            duct_out,
            groups: Vec::new(),
            groups_for: 0,
            groups_side: Vec::new(),
            ahead: vec![Ahead::default(); duct_count],
            junction_faces,
            ahead_done: (0..duct_count).map(|_| CachePadded(AtomicU64::new(0))).collect(),
            duct_done: (0..duct_count).map(|_| CachePadded(AtomicU64::new(0))).collect(),
            side_done: Vec::new(),
            node_done: (0..node_count).map(|_| CachePadded(AtomicU64::new(0))).collect(),
            node_residual: vec![CachePadded(0.0); node_count],
            thread_nodes: Vec::new(),
            turbine_junctions,
            duct_nodes,
            node_reads,
            node_anchor,
            substeps_done: 0,
            owner: vec![0; duct_count],
            threads_used: 1,
            groups_id: 0,
            unit_time: vec![CachePadded(0.0); duct_count],
            unit_cost: vec![0.0; duct_count],
            costs_known: false,
            samples: 0,
            timing: false,
            since_balance: 0,
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
        self.advance_on(dt, &|b| valves[b], None, 1, None, None)
    }

    /// Threads worth giving this exhaust and `side`, out of `available`: enough cells for each to be
    /// worth it.
    pub fn useful_threads(&self, side: &[usize], available: usize) -> usize {
        let cells: usize = self.ducts.iter().map(|d| d.n).sum::<usize>() + side.iter().sum::<usize>();
        available.min(self.ducts.len() + side.len()).min(cells / MIN_CELLS_PER_THREAD).max(1)
    }

    /// Whether this sample is being timed, for the threads' balance: if it is, the caller times what its
    /// `after` work does with each duct's cells, on that duct's thread, and adds it with `note_time`.
    pub fn timing(&self) -> bool {
        self.timing
    }

    pub fn note_time(&mut self, duct: usize, ns: f64) {
        self.unit_time[duct].0 += ns;
    }

    /// Pressure in primary `b`'s first cell, Pa, as it stands between samples: what `read_port` gives,
    /// kept by the thread that stepped it.
    pub fn port_pressure(&self, b: usize) -> f64 {
        self.duct_out[b].port_pressure
    }

    /// What thread `w` has been timed to take per sample, ns, or failing a timing its cells.
    pub fn thread_load(&self, w: usize) -> f64 {
        let Some(g) = self.groups.get(w) else { return 0.0 };
        g.iter().map(|&i| if self.costs_known { self.unit_cost[i] } else { self.unit_cells(i) }).sum()
    }

    /// Duct or side item `i`'s cells.
    /// Duct or side item `i`'s cells.
    fn unit_cells(&self, i: usize) -> f64 {
        match self.ducts.get(i) {
            Some(d) => d.n as f64,
            None => self.groups_side.get(i - self.ducts.len()).copied().unwrap_or(0) as f64,
        }
    }

    /// Fold the last timed sample into the running costs, and deal the ducts out afresh if they have
    /// drifted out of balance.
    fn rebalance(&mut self, threads: usize) {
        if self.timing {
            for (cost, time) in self.unit_cost.iter_mut().zip(self.unit_time.iter_mut()) {
                *cost = if self.costs_known { *cost + COST_SMOOTHING * (time.0 - *cost) } else { time.0 };
                time.0 = 0.0;
            }
            self.costs_known = true;
        }
        self.since_balance += 1;
        if !self.costs_known || self.since_balance < REBALANCE_EVERY {
            return;
        }
        self.since_balance = 0;
        let groups = balance(&self.unit_cost, threads);
        if busiest(&groups, &self.unit_cost) < busiest(&self.groups, &self.unit_cost) * (1.0 - REBALANCE_GAIN) {
            self.set_groups(groups);
        }
    }

    fn set_groups(&mut self, groups: Vec<Vec<usize>>) {
        for (w, g) in groups.iter().enumerate() {
            for &i in g {
                self.owner[i] = w;
            }
        }
        // Each junction to its anchor's thread, which holds the most of its cells.
        self.thread_nodes = vec![Vec::new(); groups.len()];
        for (ni, &anchor) in self.node_anchor.iter().enumerate() {
            self.thread_nodes[self.owner[anchor]].push(ni);
        }
        self.groups = groups;
        self.groups_id = GROUPS_DEALT.fetch_add(1, Ordering::Relaxed) + 1;
    }

    /// Which dealing out of the ducts and side items to threads the last `plan` chose, as `owner_of`
    /// gives it: the same until they are dealt out afresh, and never the same for
    /// another exhaust's, or for one thread.
    pub fn groups_id(&self) -> u64 {
        if self.threads_used > 1 { self.groups_id } else { 0 }
    }

    /// `advance`, with the ducts stepped across up to `max_threads` of `pool`'s threads where there are
    /// enough of them to be worth it, `side` stepped alongside them, and `after` given what they leave.
    /// Each duct is stepped exactly as on one thread, so the result is the same to the bit.
    pub fn advance_on(
        &mut self,
        dt: f64,
        valves: &(dyn Fn(usize) -> ValveState + Sync),
        pool: Option<&ThreadPool>,
        max_threads: usize,
        side: Option<&SideWork>,
        after: Option<&After>,
    ) -> &ExhaustResult {
        self.plan(pool, max_threads, side.map_or(&[][..], |s| s.costs));
        self.advance_planned(dt, valves, pool, side, after)
    }

    /// Choose the threads the next `advance_planned` steps the ducts and side work costing
    /// `side_costs` on, out of `max_threads` of `pool`'s, and which each duct is stepped on, as
    /// `owner_of` then gives. Returns how many.
    pub fn plan(&mut self, pool: Option<&ThreadPool>, max_threads: usize, side_costs: &[usize]) -> usize {
        let threads = pool.map_or(1, |p| self.useful_threads(side_costs, p.threads().min(max_threads)));
        if threads > 1 && (self.groups_for != threads || self.groups_side != side_costs) {
            // Dealt out by cells until there are timings to go on.
            self.groups_side = side_costs.to_vec();
            let units = self.ducts.len() + side_costs.len();
            let cells: Vec<f64> = (0..units).map(|i| self.unit_cells(i)).collect();
            self.owner.resize(units, 0);
            self.unit_time.resize(units, CachePadded(0.0));
            self.unit_cost.resize(units, 0.0);
            self.side_done.resize_with(side_costs.len(), || CachePadded(AtomicU64::new(0)));
            self.set_groups(balance(&cells, threads));
            self.groups_for = threads;
            self.costs_known = false;
            self.timing = false;
            self.since_balance = 0;
            for t in self.unit_time.iter_mut() {
                t.0 = 0.0;
            }
        }
        if threads == 1 {
            self.owner.fill(0);
            self.timing = false;
        } else {
            self.rebalance(threads);
            self.timing = self.samples.is_multiple_of(TIME_EVERY);
        }
        self.samples += 1;
        self.threads_used = threads;
        threads
    }

    /// `advance_on`, on the threads `plan` chose.
    pub fn advance_planned(
        &mut self,
        dt: f64,
        valves: &(dyn Fn(usize) -> ValveState + Sync),
        pool: Option<&ThreadPool>,
        side: Option<&SideWork>,
        after: Option<&After>,
    ) -> &ExhaustResult {
        let threads = self.threads_used;
        let timing = self.timing;
        let n_ducts = self.ducts.len();
        let n_primaries = self.primary_count;
        let n_nodes = self.nodes.len();
        let n_turbines = self.turbines.len();

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
        self.result.turbines.clear();
        self.result.turbines.resize(self.turbines.len(), TurbineResult::default());
        let per = 1.0 / substeps as f64;
        // Each substep's stamp: the sample's first is `first`.
        let first = self.substeps_done + 1;
        self.substeps_done += substeps as u64;

        let ducts = Disjoint::new(&mut self.ducts);
        let fed_by_node = Disjoint::new(&mut self.fed_by_node);
        let fed_flow = Disjoint::new(&mut self.fed_flow);
        let out = Disjoint::new(&mut self.duct_out);
        let ahead = Disjoint::new(&mut self.ahead);
        let faces = Disjoint::new(&mut self.junction_faces);
        let junctions = Junctions {
            ahead: &ahead,
            faces: &faces,
            nodes: Disjoint::new(&mut self.nodes),
            turbine_nodes: Disjoint::new(&mut self.turbine_nodes),
            turbine_results: Disjoint::new(&mut self.result.turbines),
            fed_by_node: &fed_by_node,
            fed_flow: &fed_flow,
            turbine_slot: &self.turbine_slot,
            turbines: &self.turbines,
            turbulence: self.turbulence,
            substep_dt: self.substep_dt,
            substep_noise_scale: self.substep_noise_scale,
        };
        let inputs = |s: usize| FinishInputs {
            h,
            last: s + 1 == substeps,
            primaries: n_primaries,
            valves,
            afterfire_heat: &self.afterfire_heat,
            zone_cells: &self.zone_cells,
            zone_volume: &self.zone_volume,
            fed_by_node: &fed_by_node,
            fed_flow: &fed_flow,
            faces: &faces,
            duct_nodes: &self.duct_nodes,
        };
        let mut stepped = Stepped {
            ducts: &ducts,
            out: &out,
            turbines: &junctions.turbine_results,
            radiating: &self.radiating,
            zone_cells: &self.zone_cells,
            zone_volume: &self.zone_volume,
            progress: None,
        };
        let mut worst = self.junction_residual;

        if threads > 1 {
            // Each substep on each thread: 1. put its ducts' ends ahead of their reconstruction; 2.
            // solve its junctions, each once every duct it reads has put its ends ahead, as soon as it
            // can, as other threads wait for them; 3. reconstruct its ducts, leaving their boundary
            // faces unset; 4. on the first, step its side items, which need no junction; 5. finish
            // each of its ducts once the junctions at its ends are solved, and a primary once its
            // valve is stepped: its junction and own boundaries, valve or junction source and update,
            // carrying a junction's share of the mixing noise. Then its part of `after`.
            stepped.progress = Some(Progress {
                duct_done: &self.duct_done,
                side_done: &self.side_done,
                node_done: &self.node_done,
                turbine_junctions: &self.turbine_junctions,
                first,
                last: first + substeps as u64 - 1,
            });
            let unit_time = Disjoint::new(&mut self.unit_time);
            let node_residual = Disjoint::new(&mut self.node_residual);
            let (groups, thread_nodes, node_reads) = (&self.groups, &self.thread_nodes, &self.node_reads);
            let (node_anchor, duct_nodes, turbine_slot) = (&self.node_anchor, &self.duct_nodes, &self.turbine_slot);
            let (ahead_done, duct_done, side_done) = (&self.ahead_done, &self.duct_done, &self.side_done);
            let node_done = &self.node_done;
            let stepped = &stepped;
            let time = |i: usize, t0: Option<Instant>| {
                if let Some(t0) = t0 {
                    unsafe { unit_time.get(i) }.0 += t0.elapsed().as_nanos() as f64;
                }
            };
            pool.unwrap().run(threads, &|w| {
                let own = || groups[w].iter().copied().take_while(|&i| i < n_ducts);
                for s in 0..substeps {
                    let stamp = first + s as u64;
                    let last = s + 1 == substeps;
                    for i in own() {
                        // Its junctions are done with what it last put ahead, as it was finished
                        // after them.
                        unsafe { *ahead.get(i) = Ahead::of(ducts.get_ref(i), h) };
                        ahead_done[i].0.store(stamp, Ordering::Release);
                    }
                    for &ni in &thread_nodes[w] {
                        for &d in &node_reads[ni] {
                            wait_until(&ahead_done[d], stamp);
                        }
                        let t0 = timing.then(Instant::now);
                        let residual = unsafe { node_residual.get(ni) };
                        let rel = junctions.solve(ni);
                        if rel > residual.0 {
                            residual.0 = rel;
                        }
                        if let Some(slot) = turbine_slot[ni].filter(|&slot| last && slot < n_turbines) {
                            unsafe { junctions.turbine_results.get(slot) }.average(per);
                        }
                        node_done[ni].0.store(stamp, Ordering::Release);
                        time(node_anchor[ni], t0);
                    }
                    for i in own() {
                        let t0 = timing.then(Instant::now);
                        unsafe { ducts.get(i) }.begin_step(h);
                        time(i, t0);
                    }
                    if let Some(side) = side.filter(|_| s == 0) {
                        for &i in groups[w].iter().skip_while(|&&i| i < n_ducts) {
                            let t0 = timing.then(Instant::now);
                            (side.job)(i - n_ducts);
                            side_done[i - n_ducts].0.store(first, Ordering::Release);
                            time(i, t0);
                        }
                    }
                    let inputs = inputs(s);
                    for i in own() {
                        for ni in duct_nodes[i].into_iter().flatten() {
                            wait_until(&node_done[ni], stamp);
                        }
                        if s == 0 && side.is_some_and(|side| i < side.costs.len()) {
                            wait_until(&side_done[i], first);
                        }
                        let t0 = timing.then(Instant::now);
                        let step = inputs.finish(i, unsafe { ducts.get(i) });
                        unsafe { out.get(i) }.add(step, s == 0, last, substeps);
                        duct_done[i].0.store(stamp, Ordering::Release);
                        time(i, t0);
                    }
                }
                if let Some(after) = after {
                    after(w, stepped);
                }
            });
            for r in &self.node_residual {
                if r.0 > worst {
                    worst = r.0;
                }
            }
        } else {
            for s in 0..substeps {
                for i in 0..n_ducts {
                    unsafe { ducts.get(i) }.begin_step(h);
                    unsafe { *ahead.get(i) = Ahead::after(ducts.get_ref(i)) };
                }
                if let Some(side) = side.filter(|_| s == 0) {
                    for k in 0..side.costs.len() {
                        (side.job)(k);
                    }
                }
                for ni in 0..n_nodes {
                    let rel = junctions.solve(ni);
                    if rel > worst {
                        worst = rel;
                    }
                }
                let inputs = inputs(s);
                for i in 0..n_ducts {
                    let step = inputs.finish(i, unsafe { ducts.get(i) });
                    unsafe { out.get(i) }.add(step, s == 0, s + 1 == substeps, substeps);
                }
            }
            for slot in 0..n_turbines {
                unsafe { junctions.turbine_results.get(slot) }.average(per);
            }
            if let Some(after) = after {
                after(0, &stepped);
            }
        }
        self.junction_residual = worst;

        // What the ducts gave, gathered in duct order.
        for i in 0..n_ducts {
            let out = self.duct_out[i];
            let r_idx = self.radiating_index[i];
            if r_idx >= 0 {
                self.result.mouth_flows[r_idx as usize] = out.mouth_flow;
            }
            if i < n_primaries {
                self.result.valve_mass_flows[i] = out.valve_flow;
                self.result.heat_taken[i] = out.heat;
            }
        }
        self.result.substeps = substeps;
        &self.result
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
