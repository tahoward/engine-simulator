//! The exhaust as a whole: an `ExhaustGraph` of ducts joined at junctions, with one duct fed by each
//! cylinder's exhaust valve, all marched in lockstep.
//!
//! A junction is where cylinders hear each other: each blowdown pulse arrives there and partly
//! travels up the other primaries, where it helps scavenge those cylinders or blocks them depending
//! on where the firing interval puts it.

use crate::dsp::Noise;
use crate::euler_pipe::{
    DESIGN_WAVE_SPEED, DuctEnd, EndState, EulerPipe, EulerPipeOptions, InletKind, OutletKind, ValveState,
};
use crate::exhaust_graph::{
    DuctRole, End, ExhaustGraph, ends_at, node_order, path_to_air, radiating_ducts, validate_graph, valve_ducts,
};
use crate::math::{self, PI, clamp};
use crate::spec::{ambient_sound_speed, gas};

/// Turbulence intensity of the merge, as a fraction of the mixing mass flow.
const MERGE_TURBULENCE: f64 = 0.14;

/// Specific heat at constant pressure for exhaust gas, J/(kg K).
const CP_EXH: f64 = (gas::GAMMA_EXH * gas::R) / (gas::GAMMA_EXH - 1.0);

/// Relative mass-flux imbalance below which a junction is left alone.
const JUNCTION_BALANCE_TOL: f64 = 0.005;

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

/// The result of one `advance`. Reused between calls.
#[derive(Clone, Debug, Default)]
pub struct ExhaustResult {
    /// Volume flow out of each radiating mouth, m^3/s.
    pub mouth_flows: Vec<f64>,
    /// Mass flow through each cylinder's exhaust valve, kg/s, positive out of the cylinder.
    pub valve_mass_flows: Vec<f64>,
    pub substeps: usize,
}

pub struct ExhaustSystem {
    /// Every duct: valve-fed first, in cylinder order, then node-fed, in node order.
    pub ducts: Vec<EulerPipe>,
    /// How many of `ducts` are primaries, one per cylinder.
    pub primary_count: usize,
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
            substeps: 1,
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
        let fed_flow = vec![0.0; fed_by_node.len()];

        Ok(ExhaustSystem {
            primary_count: valve_fed.len(),
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
        })
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

    /// Open every radiating mouth into gas at `p` (Pa) rather than the atmosphere: the inlet of a
    /// turbine, whose back pressure the whole exhaust then works against.
    pub fn set_back_pressure(&mut self, p: f64) {
        let rho = p / (gas::R * gas::T_AMB);
        let c = ambient_sound_speed();
        for &d in &self.radiating {
            self.ducts[d].set_reservoir(p, rho, c);
        }
    }

    /// Open every radiating mouth into the atmosphere again.
    pub fn clear_back_pressure(&mut self) {
        for &d in &self.radiating {
            self.ducts[d].open_to_atmosphere();
        }
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

        for _ in 0..substeps {
            // 1. Reconstruct every duct, leaving the boundary faces unset.
            for d in self.ducts.iter_mut() {
                d.begin_step(h);
            }
            // 2. Solve the junctions while every duct is mid-step.
            if !self.nodes.is_empty() {
                self.solve_junctions();
            }
            // 3. Each duct's own boundaries: valve walls and radiating mouths.
            for i in 0..n_ducts {
                let flow = self.ducts[i].apply_own_boundaries(h);
                let r_idx = self.radiating_index[i];
                if r_idx >= 0 {
                    self.result.mouth_flows[r_idx as usize] += flow;
                }
            }
            // 4. Valve flux and the conservative update.
            for b in 0..n_primaries {
                let valve = &valves[b];
                let primary = &mut self.ducts[b];
                let flow = primary.valve_flux_for(valve);
                primary.set_end_step(h, flow + valve.extra_mass_flow);
                primary.end_step_set(valve);
                self.result.valve_mass_flows[b] += flow * primary.source_scale;
            }
            // 5. Everything a junction feeds, carrying that junction's share of the mixing noise.
            for i in 0..self.fed_by_node.len() {
                let (d, valve) = self.fed_by_node[i];
                let pipe = &mut self.ducts[d];
                pipe.set_end_step(h, self.fed_flow[i]);
                pipe.end_step_set(&valve);
            }
            for d in self.ducts.iter_mut() {
                d.after_step(h);
            }
        }

        let mut broken = false;
        for d in self.ducts.iter_mut() {
            broken = d.recover_if_broken() || broken;
        }

        let inv = if broken { 0.0 } else { 1.0 / substeps as f64 };
        for v in self.result.mouth_flows.iter_mut() {
            *v *= inv;
        }
        for v in self.result.valve_mass_flows.iter_mut() {
            *v *= inv;
        }
        self.result.substeps = substeps;
        &self.result
    }

    /// Constant-pressure junctions: the common pressure in closed form from the waves arriving,
    /// held within what the branches can justify, then Newton-corrected toward mass balance.
    fn solve_junctions(&mut self) {
        for ni in 0..self.nodes.len() {
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

    /// Upper band limit for mouth `m`'s radiation: whichever model limit binds first.
    pub fn band_limit_rad_of(&self, m: usize) -> f64 {
        let d = &self.ducts[self.radiating[m]];
        math::min(d.plane_wave_cutoff_rad, d.resolution_cutoff_rad)
    }

    pub fn set_air_speed(&mut self, v: f64) {
        for d in self.ducts.iter_mut() {
            d.set_air_speed(v);
        }
    }
}

/// The design wave speed, for callers sizing against the junction's velocity ceiling.
pub const JUNCTION_MAX_SPEED: f64 = DESIGN_WAVE_SPEED;
