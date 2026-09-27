//! Quasi-one-dimensional Euler solver for one duct of the exhaust or intake.
//!
//! Real gas dynamics rather than a linear waveguide, because exhaust blowdown is not a small
//! perturbation: pulses reach roughly a bar at Mach 0.3-0.8, where the high-pressure part of a wave
//! travels faster than the low-pressure part and the front steepens toward a shock. That harmonic
//! generation is the brassy crackle of an open pipe.
//!
//! Scheme: MUSCL-Hancock, second order in space and time, with a TVD slope limiter on the primitive
//! variables and an HLLC approximate Riemann solver at each face.
//!
//! ```text
//!     d/dt (A W) + d/dx (A F) = A S + [0, p dA/dx, 0]
//!     W = [rho, rho u, rho E],  F = [rho u, rho u^2 + p, (rho E + p) u]
//! ```

use crate::cross_modes::{ChamberPlacement, CrossModes, PipeOpening};
use crate::math::{self, PI, clamp};
use crate::simd::{F2, select};
use crate::spec::{
    CHAMBER_THROAT, ChamberSection, PIPE_PRESSURE_TAPS, PipeSegment, SegmentKind, ambient_sound_speed, chamber_body,
    chamber_offsets, gas, pipe_temperature, section_area, section_perimeter, segment_diameter, segment_section,
    speed_of_sound_exh,
};
use crate::valve::{VALVE_CD, orifice_solve};

pub const GAMMA: f64 = gas::GAMMA_EXH;
const CV: f64 = gas::R / (GAMMA - 1.0);
pub const CP: f64 = (GAMMA * gas::R) / (GAMMA - 1.0);
/// `1/(gamma-1)`, precomputed: a divide by the constant would not be folded into a multiply.
const INV_GM1: f64 = 1.0 / (GAMMA - 1.0);
const INV_GAMMA: f64 = 1.0 / GAMMA;
/// The static pressure of gas choked at the speed of sound, over the stagnation pressure it came from:
/// `(2 / (gamma + 1))^(gamma / (gamma - 1))`, 0.540 for the exhaust's gamma of 1.33.
const CHOKED_PRESSURE_RATIO: f64 = 0.5404;
const MOUTH_ISENTROPIC_EXP: f64 = (GAMMA - 1.0) / (2.0 * GAMMA);
const TWO_OVER_GM1: f64 = 2.0 / (GAMMA - 1.0);

/// Density of the air outside the pipe, kg/m^3.
const AMBIENT_RHO: f64 = gas::P_AMB / (gas::R * gas::T_AMB);

/// `4 * 0.6133^2`: the mouth's radiation resistance per unit `rho c` of the gas it opens into.
const RESISTANCE_PER_RHO_C: f64 = 4.0 * 0.6133 * 0.6133;

/// Default cell length, m, and cell cap: the solver's own, for a caller that gives none.
pub const DEFAULT_CELL_SIZE: f64 = 0.02;
pub const DEFAULT_MAX_CELLS: usize = 128;

/// Courant number.
pub const DEFAULT_CFL: f64 = 0.85;

/// Design-limit wave speed, m/s, from which the substep count is pinned: 17% over the fastest
/// `|u| + c` measured anywhere across the operating range.
pub const DESIGN_WAVE_SPEED: f64 = 1400.0;

/// Audio samples between refreshes of the thermal coefficients.
const HEAT_INTERVAL: i32 = 16;

/// Internal-energy floor, J/m^3.
const MIN_INTERNAL: f64 = 1e-3 / (GAMMA - 1.0);

/// Physical density and pressure floors for the junction boundary.
pub const MIN_JUNCTION_RHO: f64 = (0.01 * gas::P_AMB) / (gas::R * 2000.0);
pub const MIN_JUNCTION_P: f64 = 0.01 * gas::P_AMB;

/// Ambient temperature to the fourth, for the radiation term.
const AMB4: f64 = gas::T_AMB * gas::T_AMB * gas::T_AMB * gas::T_AMB;

/// Reciprocal time constant of the mean-flow tracker, 1/s.
const MEAN_FLOW_RATE: f64 = 1.0 / 0.8;

/// Steel: density kg/m^3, specific heat J/(kg K).
const WALL_RHO: f64 = 7800.0;
const WALL_CP: f64 = 490.0;

/// Multiplier on the steady-flow Nusselt number, for pulsation.
const PULSATION_NUSSELT: f64 = 3.0;
/// Nusselt floor: fully-developed laminar flow in a round duct.
const NUSSELT_FLOOR: f64 = 3.66;
/// Time constant of the mass-flux average the heat-transfer correlation is evaluated on, s.
const FLUX_AVERAGE_TAU: f64 = 0.05;

/// Open-end length correction coefficient, as a multiple of the mouth radius.
pub const OPEN_END_FACTOR: f64 = 0.6133;

/// Fewest cells a duct is ever discretised into.
const MIN_DUCT_CELLS: f64 = 3.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlopeLimiter {
    Mc,
    Minmod,
    VanLeer,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InletKind {
    Valve,
    Junction,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OutletKind {
    Mouth,
    Junction,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeadPort {
    pub length: f64,
    pub diameter: f64,
}

/// Options for one duct. `None` takes the solver's default.
#[derive(Clone, Debug, Default)]
pub struct EulerPipeOptions {
    /// Hold the duct to one step per audio sample, giving short ducts fewer, longer cells.
    pub single_step: Option<bool>,
    /// Target cell length, m.
    pub cell_size: Option<f64>,
    /// Hard cap on cells.
    pub max_cells: Option<usize>,
    pub limiter: Option<SlopeLimiter>,
    pub cfl: Option<f64>,
    /// Ceiling on substeps per call.
    pub max_substeps: Option<usize>,
    /// The cylinder-head port, prepended as the first length of duct.
    pub port: Option<HeadPort>,
    /// Wall thickness, m.
    pub wall_thickness: Option<f64>,
    /// Air speed past the pipe, m/s.
    pub air_speed: Option<f64>,
    /// Wall temperatures to inherit, resampled by normalised position.
    pub inherit_wall: Option<Vec<f64>>,
    /// Initial wall temperature, K, when there is nothing to inherit.
    pub initial_wall_temp: Option<f64>,
    /// Combined outlet area of the ducts feeding this duct's inlet junction, m^2.
    pub junction_inlet_area: Option<f64>,
    /// Linear momentum damping, 1/s, standing in for viscothermal boundary-layer loss.
    pub linear_damping: Option<f64>,
    /// Darcy friction factor for the mean flow.
    pub darcy_friction: Option<f64>,
    /// False terminates the pipe with a closed wall.
    pub radiate: Option<bool>,
    pub inlet_kind: Option<InletKind>,
    pub outlet_kind: Option<OutletKind>,
    /// False disables wall heat transfer.
    pub heat_transfer: Option<bool>,
    /// Initial gas temperature at the port, K.
    pub initial_port_temp: Option<f64>,
}

impl EulerPipeOptions {
    /// These options with every field `over` sets taken from it instead.
    pub fn overlaid(mut self, over: &EulerPipeOptions) -> EulerPipeOptions {
        macro_rules! take {
            ($($f:ident),*) => { $( if over.$f.is_some() { self.$f = over.$f.clone(); } )* };
        }
        take!(
            single_step,
            cell_size,
            max_cells,
            limiter,
            cfl,
            max_substeps,
            port,
            wall_thickness,
            air_speed,
            inherit_wall,
            initial_wall_temp,
            junction_inlet_area,
            linear_damping,
            darcy_friction,
            radiate,
            inlet_kind,
            outlet_kind,
            heat_transfer,
            initial_port_temp
        );
        self
    }
}

/// Acoustic state at one end of a duct, as a junction needs to see it.
#[derive(Clone, Copy, Debug, Default)]
pub struct EndState {
    /// The wave travelling toward this end, Pa (gauge).
    pub toward: f64,
    /// Specific acoustic impedance there, rho*c.
    pub rho_c: f64,
    /// Cross-sectional area at the end cell, m^2.
    pub area: f64,
    /// Local sound speed, m/s.
    pub c: f64,
    pub rho: f64,
    pub p: f64,
    pub u: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ValveState {
    /// Effective valve flow area, m^2. Zero when shut.
    pub throat_area: f64,
    /// Cylinder absolute pressure, Pa.
    pub cyl_pressure: f64,
    /// Cylinder gas temperature, K.
    pub cyl_temp: f64,
    /// Ratio of specific heats of the cylinder gas.
    pub cyl_gamma: f64,
    /// Extra mass flow into the port, kg/s, on top of the valve's own: a fiction of the acoustic
    /// model the cylinder's mass balance must not see.
    pub extra_mass_flow: f64,
}

impl Default for ValveState {
    fn default() -> Self {
        ValveState {
            throat_area: 0.0,
            cyl_pressure: gas::P_AMB,
            cyl_temp: gas::T_AMB,
            cyl_gamma: GAMMA,
            extra_mass_flow: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AdvanceResult {
    /// Volume flow leaving the mouth, m^3/s, averaged over the substeps taken.
    pub mouth_flow: f64,
    /// Mass flow through the valve, kg/s, positive out of the cylinder.
    pub valve_mass_flow: f64,
    /// Absolute pressure at the valve seat, Pa.
    pub port_pressure: f64,
    pub substeps: usize,
}

/// Which end of a duct.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DuctEnd {
    Inlet,
    Outlet,
}

pub struct EulerPipe {
    pub n: usize,
    pub dx: f64,
    pub total_length: f64,
    /// Leading cells belonging to the head port rather than the user's pipe.
    pub port_cells: usize,
    pub sample_rate: f64,

    rho: Vec<f64>,
    mom: Vec<f64>,
    en: Vec<f64>,

    area_cell: Vec<f64>,
    area_face: Vec<f64>,
    dia_cell: Vec<f64>,
    hyd_dia: Vec<f64>,
    shape_cell: Vec<f64>,
    pub cross_modes: Option<CrossModes>,
    inv_vol: Vec<f64>,
    inv_dia: Vec<f64>,
    u_mean: Vec<f64>,
    contraction_k: Vec<f64>,

    pr: Vec<f64>,
    pu: Vec<f64>,
    sr: Vec<f64>,
    su: Vec<f64>,
    sp: Vec<f64>,
    pp: Vec<f64>,
    lr: Vec<f64>,
    lu: Vec<f64>,
    lp: Vec<f64>,
    rr: Vec<f64>,
    ru: Vec<f64>,
    rp: Vec<f64>,
    f0: Vec<f64>,
    f1: Vec<f64>,
    f2: Vec<f64>,
    fp: Vec<f64>,

    pub limiter: SlopeLimiter,
    pub cfl: f64,
    pub max_substeps: usize,
    wall_t: Vec<f64>,
    wall_heat_capacity: Vec<f64>,
    h_ext: Vec<f64>,
    wall_thickness: f64,
    gas_decay: Vec<f64>,
    flux_avg: Vec<f64>,
    flux_avg_c: f64,
    cell_volume: Vec<f64>,
    outer_area: Vec<f64>,
    q_banked: Vec<f64>,
    inv_wall_heat_capacity: Vec<f64>,
    pub linear_damping: f64,
    pub darcy: f64,
    radiate: bool,
    pub inlet_kind: InletKind,
    pub outlet_kind: OutletKind,
    pub heat_transfer: bool,

    boundary_dt: f64,
    source_flow: f64,
    /// Fraction of the last valve source actually applied to cell 0: 1 unless the cap engaged.
    pub source_scale: f64,
    valve_throat_t: f64,
    mouth_flow_out: f64,
    /// Mass flow leaving the mouth at the last boundary solve, kg/s.
    pub mouth_mass_flow: f64,
    res_p: f64,
    res_rho: f64,
    res_resistance: f64,
    mouth_ref_dt: f64,
    mouth_cut_c: f64,
    mouth_cut_state: f64,
    mouth_phi: f64,
    mouth_radius: f64,
    /// Radiation corner, rad/s.
    pub mouth_cutoff_rad: f64,
    pub plane_wave_cutoff_rad: f64,
    /// Frequency above which this discretisation cannot represent a wave at all, rad/s.
    pub resolution_cutoff_rad: f64,

    tap_index: Vec<usize>,
    last_max_speed: f64,
    pinned_substeps: usize,
    /// Samples that needed more than the pinned count. Should stay zero.
    pub substep_bursts: u64,
    /// What `compute_valve_flux` last found, kg/s, positive out of the cylinder.
    pub valve_flux_out: f64,
    /// Number of times an inadmissible state had to be reset. Should stay zero.
    pub recoveries: u64,
    /// Times the junction boundary had to clamp a degenerate end state. Should stay zero.
    pub junction_clamps: u64,
    /// Faces that were treated as supersonic outflow.
    pub supersonic_faces: u64,
    /// Faces a junction filled at the speed of sound: choked, as a nozzle is, the most it can pass.
    pub choked_faces: u64,
    heat_counter: i32,
    heat_batch: f64,
    /// The last `(p_ghost / p)^(1/gamma)` each end's junction boundary took, keyed by the bits of
    /// its base.
    junction_pow: [(u64, f64); 2],
}

impl EulerPipe {
    pub fn new(pipe: &[PipeSegment], sample_rate: f64, port_gas_temp: f64, opts: &EulerPipeOptions) -> EulerPipe {
        let cell_size = opts.cell_size.unwrap_or(DEFAULT_CELL_SIZE);
        let max_cells = opts.max_cells.unwrap_or(DEFAULT_MAX_CELLS);
        let limiter = opts.limiter.unwrap_or(SlopeLimiter::Mc);
        let cfl = opts.cfl.unwrap_or(DEFAULT_CFL);
        let max_substeps = opts.max_substeps.unwrap_or(16);
        let wall_thickness = clamp(opts.wall_thickness.unwrap_or(0.0012), 2e-4, 0.01);
        let inlet_kind = opts.inlet_kind.unwrap_or(InletKind::Valve);
        let outlet_kind = opts.outlet_kind.unwrap_or(OutletKind::Mouth);

        let min_dx = if opts.single_step.unwrap_or(false) { single_step_dx(sample_rate, cfl) } else { 0.0 };
        let built = build_geometry(
            pipe,
            opts.port,
            cell_size,
            max_cells,
            min_dx,
            inlet_kind,
            opts.junction_inlet_area.unwrap_or(0.0),
        );
        let n = built.count;
        let dx = built.dx;
        let pinned_substeps = pinned_substeps_for(dx, sample_rate, cfl, max_substeps);

        let z = || vec![0.0; n];
        let zf = || vec![0.0; n + 1];

        let mut hyd_dia = z();
        let mut inv_vol = z();
        let mut inv_dia = z();
        for i in 0..n {
            hyd_dia[i] = built.dia_cell[i] / built.shape_cell[i];
        }
        for i in 0..n {
            inv_vol[i] = 1.0 / (built.area_cell[i] * dx);
            inv_dia[i] = 1.0 / hyd_dia[i];
        }

        let mut pipe_ = EulerPipe {
            n,
            dx,
            total_length: built.length,
            port_cells: built.port_cells,
            sample_rate,
            rho: z(),
            mom: z(),
            en: z(),
            area_cell: built.area_cell,
            area_face: built.area_face,
            dia_cell: built.dia_cell,
            hyd_dia,
            shape_cell: built.shape_cell,
            cross_modes: None,
            inv_vol,
            inv_dia,
            u_mean: z(),
            contraction_k: built.contraction_k,
            pr: z(),
            pu: z(),
            sr: z(),
            su: z(),
            sp: z(),
            pp: z(),
            lr: z(),
            lu: z(),
            lp: z(),
            rr: z(),
            ru: z(),
            rp: z(),
            f0: zf(),
            f1: zf(),
            f2: zf(),
            fp: zf(),
            limiter,
            cfl,
            max_substeps,
            wall_t: z(),
            wall_heat_capacity: z(),
            h_ext: z(),
            wall_thickness,
            gas_decay: vec![1.0; n],
            flux_avg: z(),
            flux_avg_c: 0.0,
            cell_volume: z(),
            outer_area: z(),
            q_banked: z(),
            inv_wall_heat_capacity: z(),
            linear_damping: opts.linear_damping.unwrap_or(150.0),
            darcy: opts.darcy_friction.unwrap_or(0.03),
            radiate: opts.radiate.unwrap_or(true),
            inlet_kind,
            outlet_kind,
            heat_transfer: opts.heat_transfer.unwrap_or(true),
            boundary_dt: 0.0,
            source_flow: 0.0,
            source_scale: 1.0,
            valve_throat_t: 1.0,
            mouth_flow_out: 0.0,
            mouth_mass_flow: 0.0,
            res_p: gas::P_AMB,
            res_rho: AMBIENT_RHO,
            res_resistance: RESISTANCE_PER_RHO_C * AMBIENT_RHO * ambient_sound_speed(),
            mouth_ref_dt: -1.0,
            mouth_cut_c: 0.0,
            mouth_cut_state: 0.0,
            mouth_phi: 0.0,
            mouth_radius: 0.0,
            mouth_cutoff_rad: 0.0,
            plane_wave_cutoff_rad: 0.0,
            resolution_cutoff_rad: 0.0,
            tap_index: vec![0; PIPE_PRESSURE_TAPS],
            last_max_speed: 0.0,
            pinned_substeps,
            substep_bursts: 0,
            valve_flux_out: 0.0,
            recoveries: 0,
            junction_clamps: 0,
            supersonic_faces: 0,
            choked_faces: 0,
            heat_counter: 0,
            heat_batch: 0.0,
            junction_pow: [(f64::NAN.to_bits(), f64::NAN); 2],
        };
        let p = &mut pipe_;

        // --- wall thermal state ---
        for i in 0..n {
            p.cell_volume[i] = p.area_cell[i] * dx;
            let d = p.shape_cell[i] * p.dia_cell[i];
            p.outer_area[i] = PI * (d + 2.0 * p.wall_thickness) * dx;
        }
        let init_temp_for_wall = opts.initial_port_temp.unwrap_or(port_gas_temp);
        for i in 0..n {
            let d = p.shape_cell[i] * p.dia_cell[i];
            let mass = WALL_RHO * PI * (d + p.wall_thickness) * p.wall_thickness * dx;
            p.wall_heat_capacity[i] = math::max(mass * WALL_CP, 1e-6);
            p.inv_wall_heat_capacity[i] = 1.0 / p.wall_heat_capacity[i];
            let gas_guess = pipe_temperature(init_temp_for_wall, (i as f64 + 0.5) * dx);
            p.wall_t[i] = gas::T_AMB + 0.62 * (gas_guess - gas::T_AMB);
        }
        if let Some(t) = opts.initial_wall_temp {
            p.wall_t.fill(t);
        }
        if let Some(src) = &opts.inherit_wall {
            if src.len() > 1 {
                for i in 0..n {
                    let u = if n > 1 { i as f64 / (n as f64 - 1.0) } else { 0.0 };
                    let j = math::min(src.len() as f64 - 1.0, math::round(u * (src.len() as f64 - 1.0)));
                    p.wall_t[i] = src[j as usize];
                }
            }
        }
        p.set_air_speed(opts.air_speed.unwrap_or(0.0));

        // Initial condition: still gas at ambient pressure, on the empirical temperature profile.
        let init_temp = opts.initial_port_temp.unwrap_or(port_gas_temp);
        for i in 0..n {
            let x = (i as f64 + 0.5) * dx;
            let t = pipe_temperature(init_temp, x);
            p.set_primitive(i, gas::P_AMB / (gas::R * t), 0.0, gas::P_AMB);
        }

        let cross = CrossModes::new(&built.chambers, n, dx);
        p.cross_modes = if cross.count > 0 { Some(cross) } else { None };

        let mouth_radius = math::max(p.dia_cell[n - 1] / 2.0, 5e-3);
        p.mouth_radius = mouth_radius;
        let mouth_t = pipe_temperature(init_temp, p.total_length);
        p.mouth_cutoff_rad = (2.0 * ambient_sound_speed()) / mouth_radius;
        p.plane_wave_cutoff_rad = (1.8412 * speed_of_sound_exh(mouth_t)) / (mouth_radius * built.launch_radius_ratio);
        p.resolution_cutoff_rad = (2.0 * PI * speed_of_sound_exh(mouth_t)) / (5.0 * dx);
        p.last_max_speed = speed_of_sound_exh(init_temp) * 1.2;

        let first = p.port_cells.min(n.saturating_sub(1));
        let span = (n as i64 - 1 - first as i64).max(0) as f64;
        for k in 0..PIPE_PRESSURE_TAPS {
            let t = first as f64 + math::round((k as f64 / (PIPE_PRESSURE_TAPS as f64 - 1.0)) * span);
            p.tap_index[k] = math::min(n as f64 - 1.0, t) as usize;
        }
        pipe_
    }

    // -------------------------------------------------------------------------
    // State access
    // -------------------------------------------------------------------------

    /// Open the mouth into a reservoir rather than the atmosphere: an intake runner's end in the
    /// manifold. Pressure (Pa), density (kg/m^3) and sound speed (m/s), held until the next call.
    pub fn set_reservoir(&mut self, p: f64, rho: f64, c: f64) {
        self.res_p = p;
        self.res_rho = rho;
        self.res_resistance = RESISTANCE_PER_RHO_C * rho * c;
    }

    /// `valve_flux_for`, into `valve_flux_out`.
    pub fn compute_valve_flux(&mut self, valve: &ValveState) {
        self.valve_flux_out = self.valve_flux(valve);
    }

    pub fn set_primitive(&mut self, i: usize, rho: f64, u: f64, p: f64) {
        self.rho[i] = rho;
        self.mom[i] = rho * u;
        self.en[i] = p / (GAMMA - 1.0) + 0.5 * rho * u * u;
    }

    /// Take on `src`'s gas, laid onto this duct's grid by distance from the inlet.
    pub fn resample_from(&mut self, src: &EulerPipe) {
        let last = src.n as f64 - 1.0;
        for i in 0..self.n {
            let mut f = ((i as f64 + 0.5) * self.dx) / src.dx - 0.5;
            f = if f < 0.0 {
                0.0
            } else if f > last {
                last
            } else {
                f
            };
            let jf = f.floor();
            let j = jf as usize;
            let k = if jf < last { j + 1 } else { j };
            let w = f - jf;
            self.rho[i] = src.rho[j] + (src.rho[k] - src.rho[j]) * w;
            self.mom[i] = src.mom[j] + (src.mom[k] - src.mom[j]) * w;
            self.en[i] = src.en[j] + (src.en[k] - src.en[j]) * w;
            self.u_mean[i] = src.u_mean[j] + (src.u_mean[k] - src.u_mean[j]) * w;
            self.flux_avg[i] = src.flux_avg[j] + (src.flux_avg[k] - src.flux_avg[j]) * w;
        }
        self.mouth_phi = src.mouth_phi;
        self.mouth_cut_state = src.mouth_cut_state;
        self.last_max_speed = src.last_max_speed;
    }

    #[inline]
    pub fn pressure_at(&self, i: usize) -> f64 {
        let r = self.rho[i];
        let u = self.mom[i] / r;
        math::max((GAMMA - 1.0) * (self.en[i] - 0.5 * r * u * u), 1e-3)
    }

    /// Cell cross-sectional area, m^2, after area-gradient limiting.
    pub fn area_of(&self, i: usize) -> f64 {
        self.area_cell[i]
    }

    pub fn density_at(&self, i: usize) -> f64 {
        self.rho[i]
    }

    #[inline]
    pub fn temperature_at(&self, i: usize) -> f64 {
        self.pressure_at(i) / (gas::R * self.rho[i])
    }

    pub fn velocity_at(&self, i: usize) -> f64 {
        self.mom[i] / self.rho[i]
    }

    /// Absolute pressure at the valve seat, Pa.
    pub fn port_pressure(&self) -> f64 {
        self.pressure_at(0)
    }

    /// The inlet cell's pressure, temperature and area.
    #[inline]
    pub fn read_port(&self) -> (f64, f64, f64) {
        (self.pressure_at(0), self.temperature_at(0), self.area_cell[0])
    }

    /// The mouth cell's pressure, temperature and area.
    #[inline]
    pub fn read_mouth(&self) -> (f64, f64, f64) {
        let last = self.n - 1;
        (self.pressure_at(last), self.temperature_at(last), self.area_cell[last])
    }

    pub fn inlet_area(&self) -> f64 {
        self.area_cell[0]
    }

    pub fn mouth_area(&self) -> f64 {
        self.area_cell[self.n - 1]
    }

    /// First quarter-wave resonance of the duct as it is currently filled, Hz, integrated over the
    /// solved temperature field, with the mouth's end correction.
    pub fn quarter_wave_hz(&self) -> f64 {
        let mut travel = 0.0;
        for i in 0..self.n {
            travel += self.dx / math::sqrt(GAMMA * gas::R * self.temperature_at(i));
        }
        if self.outlet_kind == OutletKind::Mouth && self.radiate {
            let last = self.n - 1;
            let r = self.rho[last];
            let delta = (OPEN_END_FACTOR * self.mouth_radius * AMBIENT_RHO) / r;
            travel += delta / math::sqrt(GAMMA * gas::R * self.temperature_at(last));
        }
        if travel > 0.0 { 1.0 / (4.0 * travel) } else { 0.0 }
    }

    /// Total gas mass in the duct, kg.
    pub fn total_mass(&self) -> f64 {
        let mut m = 0.0;
        for i in 0..self.n {
            m += self.rho[i] * self.area_cell[i] * self.dx;
        }
        m
    }

    /// Total energy in the duct, J.
    pub fn total_energy(&self) -> f64 {
        let mut e = 0.0;
        for i in 0..self.n {
            e += self.en[i] * self.area_cell[i] * self.dx;
        }
        e
    }

    /// Gauge pressure along the visible pipe into `out`.
    pub fn sample_pressure(&self, out: &mut [f32]) {
        for (k, o) in out.iter_mut().enumerate().take(self.tap_index.len()) {
            *o = (self.pressure_at(self.tap_index[k]) - gas::P_AMB) as f32;
        }
    }

    /// Gas temperature along the visible pipe, K.
    pub fn sample_temperature(&self, out: &mut [f32]) {
        for (k, o) in out.iter_mut().enumerate().take(self.tap_index.len()) {
            *o = self.temperature_at(self.tap_index[k]) as f32;
        }
    }

    // -------------------------------------------------------------------------
    // Time advance
    // -------------------------------------------------------------------------

    /// Advance the duct by `dt`, substepping to satisfy the CFL condition. The valve flow is
    /// recomputed against the live port pressure on every substep.
    pub fn advance(&mut self, dt: f64, valve: &ValveState) -> AdvanceResult {
        let substeps = self.substeps_for(dt);
        let h = dt / substeps as f64;

        let mut mouth_acc = 0.0;
        let mut valve_acc = 0.0;
        for _ in 0..substeps {
            self.begin_step(h);
            let mouth = self.apply_own_boundaries(h);
            let valve_flow = self.valve_flux(valve);
            self.end_step(h, valve_flow + valve.extra_mass_flow, valve);
            self.after_step(h);
            mouth_acc += mouth;
            valve_acc += valve_flow * self.source_scale;
        }

        let inv = 1.0 / substeps as f64;
        let mut mouth_flow = mouth_acc * inv;
        let mut valve_mass_flow = valve_acc * inv;
        if !mouth_flow.is_finite() || !valve_mass_flow.is_finite() || !self.is_finite() {
            self.reset_to_quiescent();
            mouth_flow = 0.0;
            valve_mass_flow = 0.0;
            self.recoveries += 1;
        }
        AdvanceResult { mouth_flow, valve_mass_flow, port_pressure: self.port_pressure(), substeps }
    }

    /// Mass flow through the valve at this duct's inlet, kg/s, positive out of the cylinder.
    fn valve_flux(&mut self, valve: &ValveState) -> f64 {
        let area = valve.throat_area;
        if area <= 0.0 {
            return 0.0;
        }
        let (p_port, t_port, _) = self.read_port();
        if valve.cyl_pressure > p_port {
            let o = orifice_solve(area, VALVE_CD, valve.cyl_pressure, valve.cyl_temp, p_port, valve.cyl_gamma);
            self.valve_throat_t = if o.mdot > 0.0 { o.throat_t } else { 1.0 };
            return o.mdot;
        }
        // Flowing back into the cylinder, the gas arrives moving: its total pressure and temperature
        // drive it through.
        let r0 = self.rho[0];
        let u0 = self.mom[0] / r0;
        let toward = if u0 < 0.0 { -u0 } else { 0.0 };
        let o = orifice_solve(
            area,
            VALVE_CD,
            p_port + 0.5 * r0 * toward * toward,
            t_port + (toward * toward) / (2.0 * CP),
            valve.cyl_pressure,
            GAMMA,
        );
        -o.mdot
    }

    /// Phase one of a substep: primitives, limited slopes, half-step evolution and every interior
    /// face flux. The two boundary faces are left unset.
    #[inline]
    pub fn begin_step(&mut self, dt: f64) {
        self.reconstruct(dt);
    }

    /// Phase two: the conservative update, valve source and friction.
    pub fn end_step(&mut self, dt: f64, valve_flow: f64, valve: &ValveState) {
        self.set_end_step(dt, valve_flow);
        self.end_step_set(valve);
    }

    #[inline]
    pub fn set_end_step(&mut self, dt: f64, valve_flow: f64) {
        self.boundary_dt = dt;
        self.source_flow = valve_flow;
    }

    #[inline]
    pub fn end_step_set(&mut self, valve: &ValveState) {
        self.update(valve);
    }

    /// Phase three: the thermal pass.
    pub fn after_step(&mut self, dt: f64) {
        if !self.heat_transfer {
            return;
        }
        self.apply_thermal();
        self.heat_batch += dt;
        self.heat_counter -= 1;
        if self.heat_counter <= 0 {
            self.apply_wall_thermal(self.heat_batch);
            self.refresh_thermal_coefficients(dt);
            self.heat_counter = HEAT_INTERVAL;
            self.heat_batch = 0.0;
        }
    }

    /// Whether every cell holds finite state and a positive density. Without early exits, so it
    /// vectorises: a broken duct is the rare case.
    fn is_finite(&self) -> bool {
        let mut ok = true;
        for ((&r, &m), &e) in self.rho.iter().zip(&self.mom).zip(&self.en) {
            ok &= r.is_finite() & m.is_finite() & e.is_finite() & (r > 0.0);
        }
        ok
    }

    /// True if the state has gone inadmissible; resets it if so.
    pub fn recover_if_broken(&mut self) -> bool {
        if self.is_finite() {
            return false;
        }
        self.reset_to_quiescent();
        self.recoveries += 1;
        true
    }

    fn reset_to_quiescent(&mut self) {
        for i in 0..self.n {
            let t = self.wall_t[i];
            self.set_primitive(i, gas::P_AMB / (gas::R * t), 0.0, gas::P_AMB);
            self.flux_avg[i] = 0.0;
        }
        self.mouth_phi = 0.0;
        self.mouth_cut_state = 0.0;
        if let Some(c) = &mut self.cross_modes {
            c.reset();
        }
        self.last_max_speed = speed_of_sound_exh(self.mean_wall_temp()) * 1.5;
    }

    /// Substeps this duct takes to cover `dt`: the pinned count, unless the gas is moving faster than
    /// the design limit.
    #[inline]
    pub fn substeps_for(&mut self, dt: f64) -> usize {
        let limit = (self.cfl * self.dx) / math::max(self.last_max_speed, 1.0);
        let needed = (dt / limit).ceil();
        if needed > self.pinned_substeps as f64 {
            self.substep_bursts += 1;
            return math::min(needed, self.max_substeps as f64) as usize;
        }
        self.pinned_substeps
    }

    /// Substeps this duct always takes, absent a burst.
    pub fn substeps(&self) -> usize {
        self.pinned_substeps
    }

    /// Mass flow through this duct's exhaust valve, kg/s, positive out of the cylinder.
    #[inline]
    pub fn valve_flux_for(&mut self, valve: &ValveState) -> f64 {
        self.valve_flux(valve)
    }

    /// Apply the duct's own boundary conditions, for ends with no junction. Returns the volume flow
    /// out of the mouth, m^3/s.
    #[inline]
    pub fn apply_own_boundaries(&mut self, dt: f64) -> f64 {
        if self.inlet_kind == InletKind::Valve {
            self.inlet_wall();
        }
        if self.outlet_kind != OutletKind::Mouth {
            return 0.0;
        }
        self.boundary_dt = dt;
        self.mouth_boundary();
        self.mouth_flow_out
    }

    /// A solid wall: the reflecting wall's Riemann pressure, written out from HLLC on the mirrored
    /// pair. The valve enters as a source term in cell 0.
    fn inlet_wall(&mut self) {
        let r = self.lr[0];
        let u = self.lu[0];
        let p = self.lp[0];
        let c = math::sqrt((GAMMA * p) / r);
        let s_l = math::min(-u - c, u - c);
        let p_wall = math::max(p + r * (s_l + u) * u, 1e-3);
        self.f0[0] = 0.0;
        self.f1[0] = p_wall;
        self.f2[0] = 0.0;
        self.fp[0] = p_wall;
    }

    /// Primitives, limited slopes, the Hancock half-step and every interior face flux, two cells at a
    /// time. The lanes compute exactly what one cell at a time would; see `simd`.
    fn reconstruct(&mut self, dt: f64) {
        let n = self.n;
        let zero = F2::splat(0.0);
        let half = F2::splat(0.5);
        let one = F2::splat(1.0);
        let two = F2::splat(2.0);
        let gamma = F2::splat(GAMMA);
        let gm1 = F2::splat(GAMMA - 1.0);
        let inv_gm1 = F2::splat(INV_GM1);
        let p_floor = F2::splat(1e-3);
        let r_floor = F2::splat(1e-7);

        // --- primitives, and the wave speed for the next substep ---
        let mut acc = zero;
        let mut i = 0;
        // SAFETY: every load and store here is of cells `i` and `i + 1` with `i + 1 < n`, and every
        // field is at least `n` long.
        unsafe {
            while i + 1 < n {
                let r = F2::load(&self.rho, i);
                let inv_r = one / r;
                let u = F2::load(&self.mom, i) * inv_r;
                let p = (gm1 * (F2::load(&self.en, i) - half * r * u * u)).max_js(p_floor);
                r.store(&mut self.pr, i);
                u.store(&mut self.pu, i);
                p.store(&mut self.pp, i);
                let s = u.abs() + (gamma * p * inv_r).sqrt();
                // `s > max ? s : max`, which skips NaN as the scalar comparison does.
                acc = select(s.gt(acc), s, acc);
                i += 2;
            }
        }
        let mut max_speed = acc.lane0();
        let lane1 = acc.lane1();
        if lane1 > max_speed {
            max_speed = lane1;
        }
        while i < n {
            let r = self.rho[i];
            let inv_r = 1.0 / r;
            let u = self.mom[i] * inv_r;
            let p = math::max((GAMMA - 1.0) * (self.en[i] - 0.5 * r * u * u), 1e-3);
            self.pr[i] = r;
            self.pu[i] = u;
            self.pp[i] = p;
            let s = u.abs() + math::sqrt(GAMMA * p * inv_r);
            if s > max_speed {
                max_speed = s;
            }
            i += 1;
        }
        // A non-finite speed would make the substep size NaN and stall the loop silently.
        self.last_max_speed = if max_speed.is_finite() { math::max(max_speed, 1.0) } else { 1e5 };

        // --- limited slopes on primitives, the end cells peeled off ---
        self.sr[0] = 0.0;
        self.su[0] = 0.0;
        self.sp[0] = 0.0;
        self.sr[n - 1] = 0.0;
        self.su[n - 1] = 0.0;
        self.sp[n - 1] = 0.0;
        let slope_end = n.saturating_sub(1);
        macro_rules! slopes {
            ($vector:expr, $scalar:expr) => {{
                let mut i = 1;
                // SAFETY: cells `i - 1` to `i + 2`, with `i + 1 < n - 1`.
                unsafe {
                    while i + 1 < slope_end {
                        for (src, dst) in [(&self.pr, &mut self.sr), (&self.pu, &mut self.su), (&self.pp, &mut self.sp)]
                        {
                            let c = F2::load(src, i);
                            let a = c - F2::load(src, i - 1);
                            let b = F2::load(src, i + 1) - c;
                            $vector(a, b).store(dst, i);
                        }
                        i += 2;
                    }
                }
                while i < slope_end {
                    self.sr[i] = $scalar(self.pr[i] - self.pr[i - 1], self.pr[i + 1] - self.pr[i]);
                    self.su[i] = $scalar(self.pu[i] - self.pu[i - 1], self.pu[i + 1] - self.pu[i]);
                    self.sp[i] = $scalar(self.pp[i] - self.pp[i - 1], self.pp[i + 1] - self.pp[i]);
                    i += 1;
                }
            }};
        }
        match self.limiter {
            SlopeLimiter::Minmod => {
                slopes!(|a: F2, b: F2| select((a * b).le(zero), zero, select(a.abs().lt(b.abs()), a, b)), minmod)
            }
            SlopeLimiter::VanLeer => {
                slopes!(|a: F2, b: F2| select((a * b).le(zero), zero, (two * a * b) / (a + b)), van_leer)
            }
            SlopeLimiter::Mc => slopes!(
                |a: F2, b: F2| {
                    let c = half * (a + b);
                    let m = c.abs().min_js((two * a.abs()).min_js(two * b.abs()));
                    select((a * b).le(zero), zero, select(c.lt(zero), m.neg(), m))
                },
                mc
            ),
        }

        // --- half-step evolution (Hancock), in the same quasi-1D form as the corrector ---
        let half_dt = 0.5 * dt;
        let half_dt_v = F2::splat(half_dt);
        let mut i = 0;
        // SAFETY: cells `i` and `i + 1` with `i + 1 < n`, and faces up to `i + 2 <= n`, of `n + 1`.
        unsafe {
            while i + 1 < n {
                let sr = F2::load(&self.sr, i);
                let su = F2::load(&self.su, i);
                let sp = F2::load(&self.sp, i);
                let c_r = F2::load(&self.pr, i);
                let c_u = F2::load(&self.pu, i);
                let c_p = F2::load(&self.pp, i);

                let r_l = c_r - half * sr;
                let u_l = c_u - half * su;
                let p_l = c_p - half * sp;
                let r_r = c_r + half * sr;
                let u_r = c_u + half * su;
                let p_r = c_p + half * sp;

                let ul1 = r_l * u_l;
                let ul2 = p_l * inv_gm1 + half * r_l * u_l * u_l;
                let ur1 = r_r * u_r;
                let ur2 = p_r * inv_gm1 + half * r_r * u_r * u_r;

                let a_lh = F2::load(&self.area_face, i);
                let a_rh = F2::load(&self.area_face, i + 1);
                let hk = half_dt_v * F2::load(&self.inv_vol, i);

                let d0 = hk * (a_lh * ul1 - a_rh * ur1);
                let d1 = hk * (a_lh * (ul1 * u_l + p_l) - a_rh * (ur1 * u_r + p_r)) + hk * c_p * (a_rh - a_lh);
                let d2 = hk * (a_lh * (ul2 + p_l) * u_l - a_rh * (ur2 + p_r) * u_r);

                let a0 = r_l + d0;
                let a1 = ul1 + d1;
                let a2 = ul2 + d2;
                let dens = select(a0.gt(r_floor), a0, r_floor);
                let inv_dens = one / dens;
                dens.store(&mut self.lr, i);
                (a1 * inv_dens).store(&mut self.lu, i);
                (gm1 * (a2 - half * a1 * a1 * inv_dens)).max_js(p_floor).store(&mut self.lp, i);

                let a0 = r_r + d0;
                let a1 = ur1 + d1;
                let a2 = ur2 + d2;
                let dens = select(a0.gt(r_floor), a0, r_floor);
                let inv_dens = one / dens;
                dens.store(&mut self.rr, i);
                (a1 * inv_dens).store(&mut self.ru, i);
                (gm1 * (a2 - half * a1 * a1 * inv_dens)).max_js(p_floor).store(&mut self.rp, i);
                i += 2;
            }
        }
        while i < n {
            let (sri, sui, spi) = (self.sr[i], self.su[i], self.sp[i]);
            let (pri, pui, ppi) = (self.pr[i], self.pu[i], self.pp[i]);
            let r_l = pri - 0.5 * sri;
            let u_l = pui - 0.5 * sui;
            let p_l = ppi - 0.5 * spi;
            let r_r = pri + 0.5 * sri;
            let u_r = pui + 0.5 * sui;
            let p_r = ppi + 0.5 * spi;

            let ul1 = r_l * u_l;
            let ul2 = p_l * INV_GM1 + 0.5 * r_l * u_l * u_l;
            let ur1 = r_r * u_r;
            let ur2 = p_r * INV_GM1 + 0.5 * r_r * u_r * u_r;

            let a_lh = self.area_face[i];
            let a_rh = self.area_face[i + 1];
            let hk = half_dt * self.inv_vol[i];

            let d0 = hk * (a_lh * ul1 - a_rh * ur1);
            let d1 = hk * (a_lh * (ul1 * u_l + p_l) - a_rh * (ur1 * u_r + p_r)) + hk * ppi * (a_rh - a_lh);
            let d2 = hk * (a_lh * (ul2 + p_l) * u_l - a_rh * (ur2 + p_r) * u_r);

            let a0 = r_l + d0;
            let a1 = ul1 + d1;
            let a2 = ul2 + d2;
            let dens = if a0 > 1e-7 { a0 } else { 1e-7 };
            let inv_dens = 1.0 / dens;
            self.lr[i] = dens;
            self.lu[i] = a1 * inv_dens;
            self.lp[i] = math::max((GAMMA - 1.0) * (a2 - 0.5 * a1 * a1 * inv_dens), 1e-3);

            let a0 = r_r + d0;
            let a1 = ur1 + d1;
            let a2 = ur2 + d2;
            let dens = if a0 > 1e-7 { a0 } else { 1e-7 };
            let inv_dens = 1.0 / dens;
            self.rr[i] = dens;
            self.ru[i] = a1 * inv_dens;
            self.rp[i] = math::max((GAMMA - 1.0) * (a2 - 0.5 * a1 * a1 * inv_dens), 1e-3);
            i += 1;
        }

        // --- interior faces ---
        let mut f = 1;
        // SAFETY: faces `f` and `f + 1` with `f + 1 < n`, from cells `f - 1` to `f + 1`.
        unsafe {
            while f + 1 < n {
                let (o0, o1, o2, op) = hllc2(
                    F2::load(&self.rr, f - 1),
                    F2::load(&self.ru, f - 1),
                    F2::load(&self.rp, f - 1),
                    F2::load(&self.lr, f),
                    F2::load(&self.lu, f),
                    F2::load(&self.lp, f),
                );
                o0.store(&mut self.f0, f);
                o1.store(&mut self.f1, f);
                o2.store(&mut self.f2, f);
                op.store(&mut self.fp, f);
                f += 2;
            }
        }
        while f < n {
            let flux = hllc(self.rr[f - 1], self.ru[f - 1], self.rp[f - 1], self.lr[f], self.lu[f], self.lp[f]);
            self.set_face(f, flux);
            f += 1;
        }
    }

    /// Conservative update, then the valve source and the chamber modes.
    fn update(&mut self, valve: &ValveState) {
        self.update_cells(self.boundary_dt);
        self.apply_valve_source(valve);
        if let Some(cross) = &mut self.cross_modes {
            cross.step(self.boundary_dt, &self.rho, &mut self.mom, &mut self.en, &self.area_cell, &self.inv_vol);
        }
    }

    /// The conservative update with area weighting and the friction, two cells at a time.
    fn update_cells(&mut self, dt: f64) {
        let n = self.n;
        let k_lin = self.linear_damping;
        let darcy_half = self.darcy * 0.5;
        let track = dt * MEAN_FLOW_RATE;
        let (dt_v, half, one, zero) = (F2::splat(dt), F2::splat(0.5), F2::splat(1.0), F2::splat(0.0));
        let (k_lin_v, darcy_half_v, track_v) = (F2::splat(k_lin), F2::splat(darcy_half), F2::splat(track));
        let (r_floor, min_internal) = (F2::splat(1e-7), F2::splat(MIN_INTERNAL));
        let mut i = 0;
        // SAFETY: cells `i` and `i + 1` with `i + 1 < n`, and faces up to `i + 2 <= n`, of `n + 1`.
        unsafe {
            while i + 1 < n {
                let a_l = F2::load(&self.area_face, i);
                let a_r = F2::load(&self.area_face, i + 1);
                let k = dt_v * F2::load(&self.inv_vol, i);
                let f0l = F2::load(&self.f0, i);
                let f0r = F2::load(&self.f0, i + 1);
                let f1l = F2::load(&self.f1, i);
                let f1r = F2::load(&self.f1, i + 1);
                let f2l = F2::load(&self.f2, i);
                let f2r = F2::load(&self.f2, i + 1);
                let mut nr = F2::load(&self.rho, i) - k * (a_r * f0r - a_l * f0l);
                let p_face = half * (F2::load(&self.fp, i) + F2::load(&self.fp, i + 1));
                let mut nm = F2::load(&self.mom, i) - k * (a_r * f1r - a_l * f1l) + k * p_face * (a_r - a_l);
                let mut ne = F2::load(&self.en, i) - k * (a_r * f2r - a_l * f2l);
                // `nr < 1e-7 ? 1e-7 : nr`
                nr = select(nr.lt(r_floor), r_floor, nr);

                let inv_nr = one / nr;
                let u_old = nm * inv_nr;
                let u_mean = F2::load(&self.u_mean, i);
                let k_quad = u_old.abs() * (darcy_half_v * F2::load(&self.inv_dia, i))
                    + (u_old * F2::load(&self.contraction_k, i)).max_js(zero);
                let u_new = (u_old + k_lin_v * u_mean * dt_v) / (one + (k_lin_v + k_quad) * dt_v);
                nm = nr * u_new;
                (u_mean + (u_new - u_mean) * track_v).store(&mut self.u_mean, i);

                let kinetic = half * nm * nm * inv_nr;
                ne = select((ne - kinetic).lt(min_internal), kinetic + min_internal, ne);

                nr.store(&mut self.rho, i);
                nm.store(&mut self.mom, i);
                ne.store(&mut self.en, i);
                i += 2;
            }
        }
        while i < n {
            let a_l = self.area_face[i];
            let a_r = self.area_face[i + 1];
            let k = dt * self.inv_vol[i];

            let mut nr = self.rho[i] - k * (a_r * self.f0[i + 1] - a_l * self.f0[i]);
            // Well-balanced area source: the mean of the two face pressures.
            let p_face = 0.5 * (self.fp[i] + self.fp[i + 1]);
            let mut nm = self.mom[i] - k * (a_r * self.f1[i + 1] - a_l * self.f1[i]) + k * p_face * (a_r - a_l);
            let mut ne = self.en[i] - k * (a_r * self.f2[i + 1] - a_l * self.f2[i]);

            if nr < 1e-7 {
                nr = 1e-7;
            }

            // Friction: linear damping on the perturbation about the mean flow, quadratic terms on
            // the whole, solved implicitly in u.
            let inv_nr = 1.0 / nr;
            let u_old = nm * inv_nr;
            let u_mean = self.u_mean[i];
            let k_quad = u_old.abs() * (darcy_half * self.inv_dia[i]) + math::max(u_old * self.contraction_k[i], 0.0);
            let u_new = (u_old + k_lin * u_mean * dt) / (1.0 + (k_lin + k_quad) * dt);
            nm = nr * u_new;

            self.u_mean[i] = u_mean + (u_new - u_mean) * track;

            let kinetic = 0.5 * nm * nm * inv_nr;
            if ne - kinetic < MIN_INTERNAL {
                ne = kinetic + MIN_INTERNAL;
            }

            self.rho[i] = nr;
            self.mom[i] = nm;
            self.en[i] = ne;
            i += 1;
        }
    }

    /// Mass, momentum and energy handed to cell 0 by the valve.
    fn apply_valve_source(&mut self, valve: &ValveState) {
        let dt = self.boundary_dt;
        let valve_flow = self.source_flow;
        self.source_scale = 1.0;

        if valve_flow != 0.0 {
            let vol = self.area_cell[0] * self.dx;
            let mut dm = (valve_flow * dt) / vol;
            let cap = 0.25 * self.rho[0];
            let uncapped = dm;
            if dm > cap {
                dm = cap;
            } else if dm < -cap {
                dm = -cap;
            }
            if dm != uncapped {
                self.source_scale = dm / uncapped;
            }

            if valve_flow > 0.0 {
                // Gas arrives carrying its total enthalpy, and part of the jet's momentum.
                let g = valve.cyl_gamma;
                let t0 = valve.cyl_temp;
                let jet_u =
                    math::sqrt(math::max((2.0 * g * gas::R * t0 * (1.0 - self.valve_throat_t)) / (g - 1.0), 0.0));
                self.rho[0] += dm;
                self.mom[0] += dm * jet_u * clamp(valve.throat_area / self.area_cell[0], 0.0, 1.0);
                self.en[0] += dm * CP * valve.cyl_temp;
            } else {
                // Reverse flow: mass leaves carrying the cell's own stagnation enthalpy.
                let u0 = self.mom[0] / self.rho[0];
                let t0 = self.temperature_at(0);
                self.rho[0] = math::max(self.rho[0] + dm, 1e-7);
                self.mom[0] += dm * u0;
                self.en[0] += dm * (CP * t0 + 0.5 * u0 * u0);
            }
        }
    }

    /// Open-end boundary: the outgoing wave stays nonlinear, the returning wave comes from the
    /// mouth's radiation load, a mass and a resistance in parallel.
    fn mouth_boundary(&mut self) {
        let dt = self.boundary_dt;
        let n = self.n;
        let last = n - 1;
        let r = self.rr[last];
        let u = self.ru[last];
        let p = self.rp[last];

        if !self.radiate {
            let flux = hllc(r, u, p, r, -u, p);
            self.set_face(n, flux);
            self.mouth_flow_out = 0.0;
            self.mouth_mass_flow = 0.0;
            return;
        }

        let c = math::sqrt((GAMMA * p) / r);

        // Supersonic outflow: impose nothing.
        if u >= c {
            self.supersonic_faces += 1;
            let flux = hllc(r, u, p, r, u, p);
            self.set_face(n, flux);
            self.mouth_mass_flow = self.f0[n] * self.area_face[n];
            self.mouth_flow_out = self.mouth_mass_flow / r;
            return;
        }

        let zc = r * c;
        let res_p = self.res_p;
        let res_rho = self.res_rho;
        let p_prime = p - res_p;
        let p_plus = 0.5 * (p_prime + zc * u);

        if dt != self.mouth_ref_dt {
            self.mouth_ref_dt = dt;
            self.mouth_cut_c = 1.0 - math::exp(-self.plane_wave_cutoff_rad * dt);
        }
        self.mouth_cut_state += self.mouth_cut_c * (p_plus - self.mouth_cut_state);
        let p_in = self.mouth_cut_state;

        let a = self.mouth_radius;
        let inertance =
            math::max(res_rho * OPEN_END_FACTOR * a - (0.5 * zc) / self.plane_wave_cutoff_rad, 1e-3 * res_rho * a);
        let p_load = ((2.0 * p_in) / zc - self.mouth_phi) / (1.0 / zc + 1.0 / self.res_resistance + dt / inertance);
        self.mouth_phi += (dt * p_load) / inertance;
        let p_minus = p_load - p_in;

        let p_ghost = math::max(res_p + p_plus + p_minus, 1e-3);

        // Ghost velocity from the outgoing Riemann invariant; the entropy from the interior for gas
        // leaving, from the reservoir for gas arriving.
        let mut r_ghost = math::max(r * math::pow(p_ghost / p, INV_GAMMA), 1e-7);
        let c_ghost = c * math::pow(p_ghost / p, MOUTH_ISENTROPIC_EXP);
        let u_ghost = u + TWO_OVER_GM1 * (c - c_ghost);
        if u_ghost < 0.0 {
            r_ghost = math::max(res_rho * math::pow(p_ghost / res_p, INV_GAMMA), 1e-7);
        }

        let flux = hllc(r, u, p, r_ghost, u_ghost, p_ghost);
        self.set_face(n, flux);
        self.mouth_mass_flow = self.f0[n] * self.area_face[n];
        self.mouth_flow_out = self.mouth_mass_flow / r;
    }

    #[inline]
    fn set_face(&mut self, face: usize, flux: (f64, f64, f64, f64)) {
        self.f0[face] = flux.0;
        self.f1[face] = flux.1;
        self.f2[face] = flux.2;
        self.fp[face] = flux.3;
    }

    /// The Nusselt correlation and the resulting per-sample decay factors: the expensive half of the
    /// heat transfer, run occasionally.
    fn refresh_thermal_coefficients(&mut self, dt_sample: f64) {
        const K_GAS: f64 = 0.05;
        const MU: f64 = 3.5e-5;
        const PR_N: f64 = 0.899;

        self.flux_avg_c = 1.0 - math::exp(-dt_sample / FLUX_AVERAGE_TAU);

        for i in 0..self.n {
            let r = self.rho[i];
            let d = self.hyd_dia[i];
            let re = (self.flux_avg[i] * d) / MU;
            let nu = math::max(0.023 * math::pow(re, 0.8) * PR_N * PULSATION_NUSSELT, NUSSELT_FLOOR);
            let htc_vol = ((nu * K_GAS) / d) * (4.0 / d);
            let tau = (r * CV) / math::max(htc_vol, 1e-9);
            self.gas_decay[i] = math::exp(-dt_sample / tau);
        }
    }

    /// Gas-to-wall exchange, every sample, on the cached coefficients.
    fn apply_thermal(&mut self) {
        let n = self.n;
        let (half, cv, c) = (F2::splat(0.5), F2::splat(CV), F2::splat(self.flux_avg_c));
        let mut i = 0;
        // SAFETY: cells `i` and `i + 1` with `i + 1 < n`.
        unsafe {
            while i + 1 < n {
                let r = F2::load(&self.rho, i);
                let m = F2::load(&self.mom, i);
                let kinetic = (half * m * m) / r;
                let e_int = F2::load(&self.en, i) - kinetic;
                let e_wall = r * cv * F2::load(&self.wall_t, i);
                let e_int_new = e_wall + (e_int - e_wall) * F2::load(&self.gas_decay, i);
                (e_int_new + kinetic).store(&mut self.en, i);
                (F2::load(&self.q_banked, i) + (e_int - e_int_new) * F2::load(&self.cell_volume, i))
                    .store(&mut self.q_banked, i);
                let fa = F2::load(&self.flux_avg, i);
                (fa + c * (m.abs() - fa)).store(&mut self.flux_avg, i);
                i += 2;
            }
        }
        while i < n {
            let r = self.rho[i];
            let m = self.mom[i];
            let kinetic = (0.5 * m * m) / r;
            let e_int = self.en[i] - kinetic;
            let e_wall = r * CV * self.wall_t[i];
            let e_int_new = e_wall + (e_int - e_wall) * self.gas_decay[i];
            self.en[i] = e_int_new + kinetic;
            self.q_banked[i] += (e_int - e_int_new) * self.cell_volume[i];
            self.flux_avg[i] += self.flux_avg_c * (m.abs() - self.flux_avg[i]);
            i += 1;
        }
    }

    /// Wall to ambient, by convection and radiation, in batches.
    fn apply_wall_thermal(&mut self, dt: f64) {
        const SIGMA_EPS: f64 = 5.67e-8 * 0.8;
        for i in 0..self.n {
            let tw = self.wall_t[i];
            let q_out =
                (self.h_ext[i] * (tw - gas::T_AMB) + SIGMA_EPS * (tw * tw * tw * tw - AMB4)) * self.outer_area[i] * dt;
            self.wall_t[i] =
                clamp(tw + (self.q_banked[i] - q_out) * self.inv_wall_heat_capacity[i], gas::T_AMB, 1600.0);
            self.q_banked[i] = 0.0;
        }
    }

    /// External heat transfer coefficient, from air moving past the pipe: Zukauskas's cross-flow
    /// correlation, floored at natural convection.
    pub fn set_air_speed(&mut self, air_speed: f64) {
        const K_AIR: f64 = 0.026;
        const NU_AIR: f64 = 1.5e-5;
        const PR_037: f64 = 0.881;
        let v = math::max(air_speed, 0.0);
        for i in 0..self.n {
            let d_out = self.shape_cell[i] * self.dia_cell[i] + 2.0 * self.wall_thickness;
            let re = (v * d_out) / NU_AIR;
            let forced = if re > 1.0 { (K_AIR / d_out) * 0.26 * math::pow(re, 0.6) * PR_037 } else { 0.0 };
            self.h_ext[i] = math::max(9.0, forced);
        }
    }

    /// Acoustic state at one end of the duct, for a junction to solve against. Valid only between
    /// `begin_step` and `end_step`.
    #[inline]
    pub fn end_state(&self, end: DuctEnd) -> EndState {
        let (i, r, u, p) = match end {
            DuctEnd::Inlet => (0, self.lr[0], self.lu[0], self.lp[0]),
            DuctEnd::Outlet => {
                let i = self.n - 1;
                (i, self.rr[i], self.ru[i], self.rp[i])
            }
        };
        let c = math::sqrt((GAMMA * p) / r);
        let rho_c = r * c;
        let toward = match end {
            DuctEnd::Outlet => 0.5 * (p - gas::P_AMB + rho_c * u),
            DuctEnd::Inlet => 0.5 * (p - gas::P_AMB - rho_c * u),
        };
        EndState { toward, rho_c, area: self.area_cell[i], c, rho: r, p, u }
    }

    /// Impose a junction pressure at one end. Returns the mass flux through that face, kg/s, positive
    /// in the duct's +x direction.
    pub fn apply_junction(&mut self, end: DuctEnd, junction_gauge: f64, junction_temp: f64, st: &EndState) -> f64 {
        self.junction_flux(end, junction_gauge, junction_temp, true, st)
    }

    /// Mass flux this duct would pass at a trial junction pressure, committing nothing.
    pub fn probe_junction(&mut self, end: DuctEnd, junction_gauge: f64, junction_temp: f64, st: &EndState) -> f64 {
        self.junction_flux(end, junction_gauge, junction_temp, false, st)
    }

    fn junction_flux(
        &mut self,
        end: DuctEnd,
        junction_gauge: f64,
        junction_temp: f64,
        commit: bool,
        st: &EndState,
    ) -> f64 {
        let face = match end {
            DuctEnd::Inlet => 0,
            DuctEnd::Outlet => self.n,
        };

        // Supersonic outflow into the junction: a choked end cannot be told what pressure to be at, so long
        // as the junction is no higher than it. Into a junction higher than it, the gas meets a shock that
        // runs back up the duct against it, which the Riemann flux below resolves: passed through as if
        // nothing were there, the duct would go on pouring into a junction it cannot fill.
        let c_end = math::sqrt((GAMMA * math::max(st.p, MIN_JUNCTION_P)) / math::max(st.rho, MIN_JUNCTION_RHO));
        let outward = if end == DuctEnd::Outlet { st.u } else { -st.u };
        if outward >= c_end && gas::P_AMB + junction_gauge <= st.p {
            if commit {
                self.supersonic_faces += 1;
            }
            let flux = hllc(st.rho, st.u, st.p, st.rho, st.u, st.p);
            if !commit {
                return flux.0 * self.area_face[face];
            }
            self.set_face(face, flux);
            return self.f0[face] * self.area_face[face];
        }

        let returning = junction_gauge - st.toward;
        let p_ghost = math::max(gas::P_AMB + junction_gauge, 1e-3);
        let ambient_c = ambient_sound_speed();
        if commit && (st.rho < MIN_JUNCTION_RHO || st.rho_c < MIN_JUNCTION_RHO * ambient_c) {
            self.junction_clamps += 1;
        }
        let rho_safe = math::max(st.rho, MIN_JUNCTION_RHO);
        let c_local = math::sqrt((GAMMA * math::max(st.p, MIN_JUNCTION_P)) / rho_safe);
        let u_limit = math::min(5.0 * c_local, DESIGN_WAVE_SPEED);
        let rho_c = math::max(st.rho_c, MIN_JUNCTION_RHO * ambient_c);
        let u_raw =
            if end == DuctEnd::Outlet { (st.toward - returning) / rho_c } else { (returning - st.toward) / rho_c };
        let area = self.area_face[face];

        // Filling the duct, the junction's gas cannot come in faster than sound: from its pressure and
        // temperature it accelerates to the duct's end as through a nozzle, and chokes there, at the sonic
        // state, however far below it the duct's end pressure falls. Asked for more by the acoustic
        // estimate, which knows nothing of choking, the end takes that state instead.
        let filling = if end == DuctEnd::Outlet { u_raw < 0.0 } else { u_raw > 0.0 };
        if filling {
            let t_star = (2.0 * math::max(junction_temp, gas::T_AMB)) / (GAMMA + 1.0);
            let c_star = math::sqrt(GAMMA * gas::R * t_star);
            if u_raw.abs() > c_star {
                if commit {
                    self.choked_faces += 1;
                }
                let p_star = math::max(gas::P_AMB + junction_gauge, 1e-3) * CHOKED_PRESSURE_RATIO;
                let r_star = p_star / (gas::R * t_star);
                let u_star = if end == DuctEnd::Outlet { -c_star } else { c_star };
                let flux = if end == DuctEnd::Inlet {
                    hllc(r_star, u_star, p_star, st.rho, st.u, st.p)
                } else {
                    hllc(st.rho, st.u, st.p, r_star, u_star, p_star)
                };
                if !commit {
                    return flux.0 * area;
                }
                self.set_face(face, flux);
                return self.f0[face] * area;
            }
        }

        if commit && u_raw.abs() > u_limit {
            self.junction_clamps += 1;
        }
        let u_ghost = clamp(u_raw, -u_limit, u_limit);
        let inflow = if end == DuctEnd::Outlet { u_ghost < 0.0 } else { u_ghost > 0.0 };
        let r_ghost = if inflow {
            math::max(
                p_ghost / (gas::R * math::max(junction_temp - (u_ghost * u_ghost) / (2.0 * CP), gas::T_AMB)),
                MIN_JUNCTION_RHO,
            )
        } else {
            let base = p_ghost / st.p;
            let e = end as usize;
            // A junction's last trial pressure is usually the one it commits, so the power it was
            // probed with is kept rather than taken again.
            if base.to_bits() != self.junction_pow[e].0 {
                self.junction_pow[e] = (base.to_bits(), math::pow(base, INV_GAMMA));
            }
            math::max(st.rho * self.junction_pow[e].1, MIN_JUNCTION_RHO)
        };

        let flux = if end == DuctEnd::Inlet {
            hllc(r_ghost, u_ghost, p_ghost, st.rho, st.u, st.p)
        } else {
            hllc(st.rho, st.u, st.p, r_ghost, u_ghost, p_ghost)
        };
        if !commit {
            return flux.0 * area;
        }
        self.set_face(face, flux);
        self.f0[face] * area
    }

    /// Cross-sectional area of the outlet face, m^2.
    pub fn outlet_area(&self) -> f64 {
        self.area_face[self.n]
    }

    /// Cross-sectional area of face `face`, 0..=n, m^2, after area-gradient limiting.
    pub fn face_area(&self, face: usize) -> f64 {
        self.area_face[face]
    }

    /// Mass flux through a face, kg/s, positive in the duct's +x direction.
    pub fn face_mass_flux(&self, face: usize) -> f64 {
        self.f0[face] * self.area_face[face]
    }

    /// Energy flux through a face, W.
    pub fn face_energy_flux(&self, face: usize) -> f64 {
        self.f2[face] * self.area_face[face]
    }

    /// Mean wall temperature, K.
    pub fn mean_wall_temp(&self) -> f64 {
        let mut sum = 0.0;
        for i in 0..self.n {
            sum += self.wall_t[i];
        }
        if self.n > 0 { sum / self.n as f64 } else { gas::T_AMB }
    }

    /// Wall temperature along the visible pipe, K.
    pub fn sample_wall_temperature(&self, out: &mut [f32]) {
        for (k, o) in out.iter_mut().enumerate().take(self.tap_index.len()) {
            *o = self.wall_t[self.tap_index[k]] as f32;
        }
    }

    /// Wall temperatures, for a rebuild to inherit.
    pub fn export_wall(&self) -> Vec<f64> {
        self.wall_t.clone()
    }

    pub fn reset(&mut self) {
        self.mouth_phi = 0.0;
        self.mouth_cut_state = 0.0;
        if let Some(c) = &mut self.cross_modes {
            c.reset();
        }
    }
}

// ---------------------------------------------------------------------------
// Numerics
// ---------------------------------------------------------------------------

#[inline(always)]
fn minmod(a: f64, b: f64) -> f64 {
    if a * b <= 0.0 {
        return 0.0;
    }
    if a.abs() < b.abs() { a } else { b }
}

#[inline(always)]
fn van_leer(a: f64, b: f64) -> f64 {
    if a * b <= 0.0 {
        return 0.0;
    }
    (2.0 * a * b) / (a + b)
}

/// Monotonised central.
#[inline(always)]
fn mc(a: f64, b: f64) -> f64 {
    if a * b <= 0.0 {
        return 0.0;
    }
    let c = 0.5 * (a + b);
    let ac = c.abs();
    let m = math::min(ac, math::min(2.0 * a.abs(), 2.0 * b.abs()));
    if c < 0.0 { -m } else { m }
}

/// HLLC approximate Riemann solver (Toro), with Davis wave-speed estimates. Returns the mass,
/// momentum and energy fluxes and the star-region pressure.
#[inline(always)]
pub fn hllc(r_l: f64, u_l: f64, p_l: f64, r_r: f64, u_r: f64, p_r: f64) -> (f64, f64, f64, f64) {
    let inv_rl = 1.0 / r_l;
    let inv_rr = 1.0 / r_r;
    let c_l = math::sqrt(GAMMA * p_l * inv_rl);
    let c_r = math::sqrt(GAMMA * p_r * inv_rr);
    let e_l = p_l * INV_GM1 + 0.5 * r_l * u_l * u_l;
    let e_r = p_r * INV_GM1 + 0.5 * r_r * u_r * u_r;

    let s_l = math::min(u_l - c_l, u_r - c_r);
    let s_r = math::max(u_l + c_l, u_r + c_r);

    if s_l >= 0.0 {
        return (r_l * u_l, r_l * u_l * u_l + p_l, (e_l + p_l) * u_l, p_l);
    }
    if s_r <= 0.0 {
        return (r_r * u_r, r_r * u_r * u_r + p_r, (e_r + p_r) * u_r, p_r);
    }

    let m_l = r_l * (s_l - u_l);
    let m_r = r_r * (s_r - u_r);
    let denom = m_l - m_r;
    let s_star = if denom.abs() < 1e-12 { 0.0 } else { (p_r - p_l + m_l * u_l - m_r * u_r) / denom };

    let fp = math::max(p_l + m_l * (s_star - u_l), 1e-3);

    if s_star >= 0.0 {
        let f = m_l / (s_l - s_star);
        (
            r_l * u_l + s_l * (f - r_l),
            r_l * u_l * u_l + p_l + s_l * (f * s_star - r_l * u_l),
            (e_l + p_l) * u_l + s_l * (f * (e_l * inv_rl + (s_star - u_l) * (s_star + p_l / m_l)) - e_l),
            fp,
        )
    } else {
        let f = m_r / (s_r - s_star);
        (
            r_r * u_r + s_r * (f - r_r),
            r_r * u_r * u_r + p_r + s_r * (f * s_star - r_r * u_r),
            (e_r + p_r) * u_r + s_r * (f * (e_r * inv_rr + (s_star - u_r) * (s_star + p_r / m_r)) - e_r),
            fp,
        )
    }
}

/// `hllc` at two faces at once. Lanes cannot branch, so all four outcomes are computed and the one the
/// scalar code would take is selected, in the order it tests them.
#[inline(always)]
fn hllc2(r_l: F2, u_l: F2, p_l: F2, r_r: F2, u_r: F2, p_r: F2) -> (F2, F2, F2, F2) {
    let zero = F2::splat(0.0);
    let half = F2::splat(0.5);
    let one = F2::splat(1.0);
    let gamma = F2::splat(GAMMA);
    let inv_gm1 = F2::splat(INV_GM1);

    let inv_rl = one / r_l;
    let inv_rr = one / r_r;
    let c_l = (gamma * p_l * inv_rl).sqrt();
    let c_r = (gamma * p_r * inv_rr).sqrt();
    let e_l = p_l * inv_gm1 + half * r_l * u_l * u_l;
    let e_r = p_r * inv_gm1 + half * r_r * u_r * u_r;

    let s_l = (u_l - c_l).min_js(u_r - c_r);
    let s_r = (u_l + c_l).max_js(u_r + c_r);

    // Plain upwind fluxes, which are the supersonic answers.
    let rlul = r_l * u_l;
    let rrur = r_r * u_r;
    let fl1 = rlul * u_l + p_l;
    let fl2 = (e_l + p_l) * u_l;
    let fr1 = rrur * u_r + p_r;
    let fr2 = (e_r + p_r) * u_r;

    let m_l = r_l * (s_l - u_l);
    let m_r = r_r * (s_r - u_r);
    let denom = m_l - m_r;
    let quot = (p_r - p_l + m_l * u_l - m_r * u_r) / denom;
    let s_star = select(denom.abs().lt(F2::splat(1e-12)), zero, quot);
    let p_star = (p_l + m_l * (s_star - u_l)).max_js(F2::splat(1e-3));

    let f_l = m_l / (s_l - s_star);
    let sl0 = rlul + s_l * (f_l - r_l);
    let sl1 = fl1 + s_l * (f_l * s_star - rlul);
    let sl2 = fl2 + s_l * (f_l * (e_l * inv_rl + (s_star - u_l) * (s_star + p_l / m_l)) - e_l);

    let f_r = m_r / (s_r - s_star);
    let sr0 = rrur + s_r * (f_r - r_r);
    let sr1 = fr1 + s_r * (f_r * s_star - rrur);
    let sr2 = fr2 + s_r * (f_r * (e_r * inv_rr + (s_star - u_r) * (s_star + p_r / m_r)) - e_r);

    let star = s_star.ge(zero);
    let (mut o0, mut o1, mut o2, mut op) =
        (select(star, sl0, sr0), select(star, sl1, sr1), select(star, sl2, sr2), p_star);
    let right = s_r.le(zero);
    o0 = select(right, rrur, o0);
    o1 = select(right, fr1, o1);
    o2 = select(right, fr2, o2);
    op = select(right, p_r, op);
    let left = s_l.ge(zero);
    o0 = select(left, rlul, o0);
    o1 = select(left, fl1, o1);
    o2 = select(left, fl2, o2);
    op = select(left, p_l, op);
    (o0, o1, o2, op)
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// Cells a duct of this discretised length gets at a given target cell size. The cost budget has to
/// predict it exactly, so it is shared.
pub fn duct_cell_count(length: f64, cell_size: f64, max_cells: usize, min_dx: f64) -> usize {
    let mut count = clamp(math::round(length / cell_size), MIN_DUCT_CELLS, max_cells as f64);
    if min_dx > 0.0 {
        count = math::min(count, math::max(1.0, (length / min_dx).floor()));
    }
    // `count | 0`.
    if count.is_finite() { count as usize } else { 0 }
}

/// The smallest cell that still takes one step per audio sample, m.
pub fn single_step_dx(sample_rate: f64, cfl: f64) -> f64 {
    (DESIGN_WAVE_SPEED / (sample_rate * cfl)) * (1.0 + 1e-9)
}

/// Substeps per audio sample a duct with this cell size is pinned to.
pub fn pinned_substeps_for(dx: f64, sample_rate: f64, cfl: f64, max_substeps: usize) -> usize {
    let design_limit = (cfl * dx) / DESIGN_WAVE_SPEED;
    math::min(math::max((1.0 / sample_rate / design_limit).ceil(), 1.0), max_substeps as f64) as usize
}

fn usable(s: &PipeSegment) -> bool {
    s.length > 1e-4 && s.d_in > 1e-3
}

/// Discretised length of a duct, m: its segments and an optional head port.
pub fn duct_grid_length(pipe: &[PipeSegment], port: Option<HeadPort>) -> f64 {
    let segments: Vec<&PipeSegment> = pipe.iter().filter(|s| usable(s)).collect();
    if segments.is_empty() {
        return 0.1;
    }
    let mut total = segments.iter().fold(0.0, |a, s| a + s.length);
    if let Some(port) = port {
        if port.length > 1e-4 && port.diameter > 1e-3 {
            total += port.length;
        }
    }
    total
}

struct BuiltGeometry {
    count: usize,
    dx: f64,
    length: f64,
    port_cells: usize,
    area_cell: Vec<f64>,
    area_face: Vec<f64>,
    dia_cell: Vec<f64>,
    shape_cell: Vec<f64>,
    chambers: Vec<ChamberPlacement>,
    launch_radius_ratio: f64,
    contraction_k: Vec<f64>,
}

/// Loss coefficient of a contraction, referred to the velocity in the narrow pipe: Crane TP-410's
/// reducer formula.
pub fn contraction_loss(half_angle: f64, area_ratio: f64) -> f64 {
    let open = 1.0 - clamp(area_ratio, 0.0, 1.0);
    let s = math::sin(clamp(half_angle, 0.0, PI / 2.0));
    if half_angle <= PI / 8.0 { 0.8 * s * open } else { 0.5 * open * math::sqrt(s) }
}

/// Per-cell contraction loss for a duct, 1/m, signed by the flow direction each applies to.
fn contraction_loss_cells(
    area_face: &[f64],
    area_cell: &[f64],
    dx: f64,
    drawn_diameter: &dyn Fn(f64) -> f64,
    total: f64,
) -> Vec<f64> {
    let count = area_cell.len();
    let mut out = vec![0.0; count];
    let narrows = |i: usize, dir: f64| -> bool {
        if dir > 0.0 {
            area_face[i + 1] < area_face[i] * (1.0 - 1e-9)
        } else {
            area_face[i] < area_face[i + 1] * (1.0 - 1e-9)
        }
    };
    for dir in [1.0f64, -1.0] {
        let mut i = 0;
        while i < count {
            if !narrows(i, dir) {
                i += 1;
                continue;
            }
            let mut j = i;
            while j < count && narrows(j, dir) {
                j += 1;
            }
            let a_wide = if dir > 0.0 { area_face[i] } else { area_face[j] };
            let a_narrow = if dir > 0.0 { area_face[j] } else { area_face[i] };

            let x0 = math::max((i as f64 - 1.0) * dx, 0.0);
            let x1 = math::min((j as f64 + 1.0) * dx, total - 1e-9);
            const SAMPLES: usize = 64;
            let h = (x1 - x0) / SAMPLES as f64;
            let mut fall = 0.0;
            let mut fall_length = 0.0;
            let mut prev = drawn_diameter(if dir > 0.0 { x0 } else { x1 });
            for k in 1..=SAMPLES {
                let d = drawn_diameter(if dir > 0.0 { x0 + k as f64 * h } else { x1 - k as f64 * h });
                if d < prev - 1e-12 {
                    fall += prev - d;
                    fall_length += h;
                }
                prev = d;
            }
            let half_angle = if fall > 0.0 {
                math::atan(fall / (2.0 * math::max(fall_length, 1e-9)))
            } else {
                math::atan((math::sqrt(a_wide) - math::sqrt(a_narrow)) / (math::sqrt(PI) * (j - i) as f64 * dx))
            };
            let k = contraction_loss(half_angle, a_narrow / a_wide);
            let share = k / (j - i) as f64;
            for c in i..j {
                let r = area_cell[c] / a_narrow;
                out[c] = (dir * share * r * r) / (2.0 * dx);
            }
            i = j;
        }
    }
    out
}

/// Half-angle above which a widening cone launches higher modes as a step would.
const HORN_HALF_ANGLE: f64 = (15.0 * PI) / 180.0;

/// Radius, m, that sets where the wave reaching the mouth stops being plane: the largest radius at an
/// abrupt widening in the final widening run, or the throat where that run begins.
pub fn launch_radius(segments: &[PipeSegment]) -> f64 {
    let mut pieces: Vec<(f64, f64, f64)> = Vec::new();
    let mut prev = -1.0;
    for seg in segments {
        let d_in = segment_diameter(seg, 0.0);
        if prev > 0.0 && (prev - d_in).abs() > 1e-9 {
            pieces.push((prev, d_in, 0.0));
        }
        if seg.kind == SegmentKind::Chamber {
            let body = segment_diameter(seg, 0.5);
            let throat = CHAMBER_THROAT * seg.length;
            pieces.push((seg.d_in, seg.d_in, throat));
            pieces.push((seg.d_in, body, 0.0));
            pieces.push((body, body, seg.length - 2.0 * throat));
            pieces.push((body, seg.d_in, 0.0));
            pieces.push((seg.d_in, seg.d_in, throat));
        } else {
            pieces.push((d_in, segment_diameter(seg, 1.0), seg.length));
        }
        prev = segment_diameter(seg, 1.0);
    }

    let mut launch = 0.0;
    let mut throat = f64::INFINITY;
    for &(d_a, d_b, len) in pieces.iter().rev() {
        if d_a > d_b + 1e-9 {
            break;
        }
        throat = math::min(throat, d_a);
        let abrupt = len <= 0.0 || math::atan((d_b - d_a) / (2.0 * len)) > HORN_HALF_ANGLE;
        if d_b > d_a + 1e-9 && abrupt {
            launch = math::max(launch, d_b);
        }
    }
    math::max(launch, if throat.is_finite() { throat } else { 0.0 }) / 2.0
}

fn plain_pipe(id: &str, length: f64, dia: f64) -> PipeSegment {
    PipeSegment {
        id: id.to_string(),
        kind: SegmentKind::Pipe,
        length,
        d_in: dia,
        d_out: dia,
        yaw: 0.0,
        pitch: 0.0,
        section: None,
        height: None,
        offset_in: None,
        offset_out: None,
    }
}

/// Discretise the duct into uniform cells.
fn build_geometry(
    pipe: &[PipeSegment],
    port: Option<HeadPort>,
    cell_size: f64,
    max_cells: usize,
    min_dx: f64,
    inlet: InletKind,
    feed_area: f64,
) -> BuiltGeometry {
    let mut segments: Vec<PipeSegment> = pipe.iter().filter(|s| usable(s)).cloned().collect();
    if segments.is_empty() {
        segments.push(plain_pipe("__stub", 0.1, 0.04));
    }
    let has_port = matches!(port, Some(p) if p.length > 1e-4 && p.diameter > 1e-3);
    if has_port {
        let port = port.unwrap();
        segments.insert(0, plain_pipe("__port", port.length, port.diameter));
    }

    let lengths: Vec<f64> = segments.iter().map(|s| s.length).collect();
    let last_idx = segments.len() - 1;

    let total = lengths.iter().fold(0.0, |a, b| a + b);
    let count = duct_cell_count(total, cell_size, max_cells, min_dx);
    let dx = total / count as f64;

    let mut area_cell = vec![0.0; count];
    let mut dia_cell = vec![0.0; count];
    let mut area_face = vec![0.0; count + 1];

    let mut starts: Vec<f64> = Vec::with_capacity(lengths.len());
    let mut acc = 0.0;
    for l in &lengths {
        starts.push(acc);
        acc += l;
    }

    let locate = |x: f64| -> (usize, f64) {
        let mut si = segments.len() - 1;
        for k in 0..segments.len() {
            if x < starts[k] + lengths[k] {
                si = k;
                break;
            }
        }
        (si, clamp((x - starts[si]) / math::max(lengths[si], 1e-9), 0.0, 1.0))
    };
    let diameter_at = |x: f64| -> f64 {
        let (si, u) = locate(x);
        segment_diameter(&segments[si], u)
    };

    let mut chambers: Vec<ChamberPlacement> = Vec::new();
    for (k, seg) in segments.iter().enumerate() {
        if seg.kind != SegmentKind::Chamber {
            continue;
        }
        let (off_in, off_out) = chamber_offsets(seg);
        chambers.push(ChamberPlacement {
            section: chamber_body(seg),
            x_in: starts[k] + CHAMBER_THROAT * lengths[k],
            x_out: starts[k] + (1.0 - CHAMBER_THROAT) * lengths[k],
            inlet: PipeOpening { offset: off_in, diameter: seg.d_in },
            outlet: PipeOpening { offset: off_out, diameter: seg.d_in },
        });
    }

    // Each face takes the drawn area averaged over the half-cells either side of it.
    const FACE_SAMPLES: usize = 16;
    for f in 0..=count {
        let x0 = math::max((f as f64 - 0.5) * dx, 0.0);
        let x1 = math::min((f as f64 + 0.5) * dx, total - 1e-9);
        let h = (x1 - x0) / FACE_SAMPLES as f64;
        let mut sum = 0.0;
        for k in 0..FACE_SAMPLES {
            let d = diameter_at(x0 + (k as f64 + 0.5) * h);
            sum += (PI * d * d) / 4.0;
        }
        area_face[f] = sum / FACE_SAMPLES as f64;
    }

    const MAX_RATIO: f64 = 1.6;
    // Smallest a junction-fed inlet may be, as a fraction of the area feeding it.
    const JUNCTION_INLET_FRACTION: f64 = 0.6;

    if inlet == InletKind::Junction && feed_area > 0.0 {
        let floor = JUNCTION_INLET_FRACTION * feed_area;
        for f in 0..=count {
            let want = floor / math::pow(MAX_RATIO, f as f64);
            if want <= area_face[f] {
                break;
            }
            area_face[f] = want;
        }
    }

    limit_area_ratio(&mut area_face, MAX_RATIO);

    let mut shape_cell = vec![0.0; count];
    for i in 0..count {
        let a = 0.5 * (area_face[i] + area_face[i + 1]);
        area_cell[i] = a;
        dia_cell[i] = math::sqrt((4.0 * a) / PI);
        let (si, u) = locate(math::min((i as f64 + 0.5) * dx, total - 1e-9));
        let section = segment_section(&segments[si], u);
        shape_cell[i] = if section.section == ChamberSection::Round {
            1.0
        } else {
            section_perimeter(&section) / (PI * math::sqrt((4.0 * section_area(&section)) / PI))
        };
    }

    let port_cells = if has_port { math::min(count as f64 - 1.0, math::round(lengths[0] / dx)) as usize } else { 0 };

    let drawn_mouth = segment_diameter(&segments[last_idx], 1.0) / 2.0;
    let launch = launch_radius(&segments);
    let launch_radius_ratio = if launch < drawn_mouth { launch / drawn_mouth } else { 1.0 };

    let contraction_k = contraction_loss_cells(&area_face, &area_cell, dx, &diameter_at, total);

    BuiltGeometry {
        count,
        dx,
        length: total,
        port_cells,
        area_cell,
        area_face,
        dia_cell,
        shape_cell,
        chambers,
        launch_radius_ratio,
        contraction_k,
    }
}

/// Limit the face-to-face area ratio to `max_ratio`, in place, without changing the duct's volume:
/// each stretch the limit touches is a geometric blend of the two one-sided limits, bisected to hold
/// exactly the volume drawn.
pub fn limit_area_ratio(area_face: &mut [f64], max_ratio: f64) {
    let n = area_face.len();
    if n < 2 {
        return;
    }
    let mut lo = area_face.to_vec();
    let mut hi = area_face.to_vec();
    for f in 1..n {
        lo[f] = math::min(lo[f], lo[f - 1] * max_ratio);
        hi[f] = math::max(hi[f], hi[f - 1] / max_ratio);
    }
    for f in (0..n - 1).rev() {
        lo[f] = math::min(lo[f], lo[f + 1] * max_ratio);
        hi[f] = math::max(hi[f], hi[f + 1] / max_ratio);
    }

    let weight = |f: usize| if f == 0 || f == n - 1 { 0.5 } else { 1.0 };
    let touched = |f: usize| hi[f] > lo[f] * (1.0 + 1e-12);

    let mut f = 0;
    while f < n {
        if !touched(f) {
            f += 1;
            continue;
        }
        let start = f;
        while f < n && touched(f) {
            f += 1;
        }
        let end = f;

        let mut target = 0.0;
        for g in start..end {
            target += weight(g) * area_face[g];
        }
        let volume_at = |t: f64| -> f64 {
            let mut v = 0.0;
            for g in start..end {
                v += weight(g) * lo[g] * math::pow(hi[g] / lo[g], t);
            }
            v
        };

        let mut a = 0.0;
        let mut b = 1.0;
        for _ in 0..40 {
            let m = 0.5 * (a + b);
            if volume_at(m) < target {
                a = m;
            } else {
                b = m;
            }
        }
        let t = 0.5 * (a + b);
        for g in start..end {
            area_face[g] = lo[g] * math::pow(hi[g] / lo[g], t);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spec::{SegmentPartial, make_segment};

    fn pipe(length: f64, d: f64) -> PipeSegment {
        make_segment(SegmentPartial { length: Some(length), d_in: Some(d), ..Default::default() })
    }

    /// The choked pressure ratio is the one the exhaust's gamma gives.
    #[test]
    fn chokes_at_the_pressure_ratio_its_gamma_gives() {
        let ratio = math::pow(2.0 / (GAMMA + 1.0), GAMMA / (GAMMA - 1.0));
        assert!((CHOKED_PRESSURE_RATIO - ratio).abs() < 1e-4, "{ratio}");
    }

    #[test]
    fn a_duct_at_rest_stays_at_rest() {
        let opts =
            EulerPipeOptions { heat_transfer: Some(false), initial_port_temp: Some(gas::T_AMB), ..Default::default() };
        let mut duct = EulerPipe::new(&[pipe(1.0, 0.04)], 48000.0, gas::T_AMB, &opts);
        let valve = ValveState::default();
        for _ in 0..4800 {
            duct.advance(1.0 / 48000.0, &valve);
        }
        for i in 0..duct.n {
            assert!((duct.pressure_at(i) - gas::P_AMB).abs() < 1e-6, "cell {i}");
        }
    }

    #[test]
    fn limit_area_ratio_keeps_volume() {
        let mut a = vec![1.0, 1.0, 1.0, 10.0, 10.0, 10.0, 10.0, 1.0, 1.0, 1.0];
        let before: f64 =
            a.iter().enumerate().map(|(i, v)| if i == 0 || i == a.len() - 1 { 0.5 * v } else { *v }).sum();
        limit_area_ratio(&mut a, 1.6);
        let after: f64 = a.iter().enumerate().map(|(i, v)| if i == 0 || i == a.len() - 1 { 0.5 * v } else { *v }).sum();
        assert!((before - after).abs() < 1e-6 * before);
        for w in a.windows(2) {
            assert!(w[1] / w[0] <= 1.6 * (1.0 + 1e-9) && w[0] / w[1] <= 1.6 * (1.0 + 1e-9));
        }
    }
}
