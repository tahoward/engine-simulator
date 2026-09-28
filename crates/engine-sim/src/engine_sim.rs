//! The simulation itself: cylinders, valves, intake and exhaust gas dynamics and radiation, stepped
//! one audio sample at a time.
//!
//! ```text
//!   crank angle -> valve lift -> valve flow area
//!        |                            |
//!        |                  orifice flow against the port pressure
//!        |                            |
//!   cylinder gas state  <-------------+------> volume flow into the pipe
//!                                                        |
//!              quasi-1D Euler solver, MUSCL-Hancock (the user's exhaust)
//!                                                        |
//!                                    mouth flow -> d/dt -> far-field pressure
//! ```

use crate::cylinder::{AdvanceIo, CylState, Cylinder, SpecInstance};
use crate::drivetrain::{DynoPhase, DynoRun};
use crate::dsp::{Delay, Impact, Noise, Resonator, soft_clip, wrap_cycle};
use crate::euler_pipe::{
    DEFAULT_CFL, DEFAULT_MAX_CELLS, EulerPipeOptions, HeadPort, ValveState, duct_cell_count, duct_grid_length,
    single_step_dx,
};
use crate::exhaust_graph::{ExhaustGraph, compile_exhaust, node_order, validate_graph};
use crate::exhaust_system::ExhaustSystem;
use crate::intake::{IntakeRunners, RunnerIo};
use crate::listener::{Listener, ListenerGeometry};
use crate::math::{self, PI, clamp};
use crate::plenum::IntakePlenum;
use crate::radiation::FarField;
use crate::spec::{
    BankSnapshot, CV_REF, CV_SLOPE, CrankType, DynoConfig, DynoSnapshot, EngineConfig, EngineSnapshot, EngineSpec,
    ExhaustLayout, FUEL_CUT_RPM, FUEL_CUT_THROTTLE, FUEL_RESUME_RPM, PIPE_PRESSURE_TAPS, PipeSegment,
    REV_LIMIT_HYSTERESIS_RPM, RunnerSize, T_REF, TurboSnapshot, ambient_sound_speed, displacement, exhaust_layout_of,
    exhaust_port_diameter, firing_plan, fuel_fraction_at, full_load_torque, gas, intake_runner_of, load_torque_of,
    physical_bank_count,
};
use crate::turbo::{TurbineDrive, Turbo};
use crate::valve::{valve_flow_area, valve_lift};

/// Pressure, Pa, that maps to digital full scale.
const PA_PER_FULLSCALE: f64 = 250.0;

/// Fastest a cam phaser turns, crank degrees per second.
const PHASER_RATE: f64 = 250.0;

/// How far below its switch speed cam profile switching drops back to the low-speed lobes, rev/min.
pub const CAM_SWITCH_HYSTERESIS: f64 = 150.0;

/// How far below its switch speed a two-stage intake switches back to its long runners, rev/min.
const INTAKE_SWITCH_HYSTERESIS: f64 = 150.0;

/// Longest a finished dyno run waits for the engine to wind down to a held speed, s.
const DYNO_WIND_DOWN: f64 = 6.0;

/// Crank degrees per cylinder sub-step.
const MAX_DEG_PER_SUBSTEP: f64 = 0.35;

/// Largest fraction of the trapped mass that may cross the valves in one cylinder sub-step.
const MAX_MASS_FRACTION_PER_SUBSTEP: f64 = 0.05;

/// Ceiling on cylinder sub-steps.
const MAX_CYL_SUBSTEPS: f64 = 64.0;

/// Idle floor, rev/min.
const MIN_RPM: f64 = 450.0;

/// Peak structure-borne levels at 1 m, Pa, at `mech_noise = 1`.
const CLACK_PA_AT_1M: f64 = 6.0;
const SLAP_PA_AT_1M: f64 = 3.5;

/// RMS turbulent fluctuation of the plane-wave volume velocity, as a fraction of the mean valve flow.
const TURBULENCE_INTENSITY: f64 = 0.1;

/// The engine the structure-borne frequencies were set against: the default single.
const REFERENCE_DISPLACEMENT_M3: f64 = 4.977e-4;
const REFERENCE_BORE: f64 = 0.089;
const REFERENCE_EX_VALVE: f64 = 0.034;

/// Valve-seating ring and piston-slap ring on the reference engine, [Hz, Q].
const CLACK_MODE: (f64, f64) = (2700.0, 14.0);
const SLAP_MODE: (f64, f64) = (620.0, 9.0);

/// How far each cylinder's own head and bore ring from the nominal, as a fraction, peak.
const LOCAL_MODE_DETUNE: f64 = 0.06;

/// Cylinder `b` of `n`'s place in an even spread over [-1, 1], shuffled by a fixed permutation.
pub fn spread_of(b: usize, n: usize, step: usize, offset: usize) -> f64 {
    if n <= 1 {
        return 0.0;
    }
    fn gcd(a: usize, c: usize) -> usize {
        if c == 0 { a } else { gcd(c, a % c) }
    }
    let mut k = step;
    while gcd(k, n) != 1 {
        k += 1;
    }
    (((b * k + offset) % n) as f64 / (n as f64 - 1.0)) * 2.0 - 1.0
}

/// How much quieter each valve's clack, or piston's slap, is when its casting is shared.
pub fn clack_share(cylinders_per_head: f64) -> f64 {
    1.0 / math::sqrt(math::max(cylinders_per_head, 1.0))
}

/// Structural modes of the block and head, [Hz, Q].
const STRUCTURAL_MODES: [(f64, f64); 4] = [(780.0, 11.0), (1550.0, 14.0), (2900.0, 17.0), (4700.0, 20.0)];

/// Structural radiation at 1 m per GPa/s of cylinder pressure rise, Pa.
const STRUCTURE_PA_PER_GPA_S: f64 = 0.55;

/// Per-cylinder valve-timing spread at `cylinder_spread = 1`, crank degrees peak.
const CAM_SPREAD_DEG: f64 = 2.2;

/// Time constant, s, of the mean-torque tracker and of the rpm readout's smoothing.
const IRREGULARITY_TAU: f64 = 0.12;

/// The solver's cost budget, and what a cylinder and a junction cost, all in pipe cells.
const CYLINDER_COST_IN_CELLS: f64 = 102.0;
const JUNCTION_COST_IN_CELLS: f64 = 27.0;
const SOLVER_COST_BUDGET: f64 = 1216.0;

/// Cells of pipe the budget leaves for an engine with this many cylinders and junctions.
pub fn grid_budget_cells(cylinders: usize, junctions: usize) -> f64 {
    SOLVER_COST_BUDGET - CYLINDER_COST_IN_CELLS * cylinders as f64 - JUNCTION_COST_IN_CELLS * junctions as f64
}

/// Bandwidth of the combustion pressure-rise drive, Hz, and the band limit on everything
/// structure-borne, Hz.
const DPDT_BANDWIDTH_HZ: f64 = 1500.0;
const STRUCTURE_LIMIT_HZ: f64 = 6000.0;

/// Contact durations of the two impacts, s.
const VALVE_CONTACT_S: f64 = 0.00015;
const SLAP_CONTACT_S: f64 = 0.0004;

/// Everything that forces a full rebuild of cylinders and ducts when it changes.
#[derive(Clone, Copy, PartialEq)]
struct LayoutKey {
    cylinders: u32,
    layout: ExhaustLayout,
    crank: CrankType,
    headers: bool,
}

fn layout_key(spec: &EngineSpec) -> LayoutKey {
    LayoutKey {
        cylinders: spec.cylinders,
        layout: exhaust_layout_of(spec),
        crank: spec.crank_type,
        headers: spec.exhaust_headers,
    }
}

fn same_offsets(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x == y || (x.is_nan() && y.is_nan()))
}

pub struct EngineSim {
    pub sample_rate: f64,
    spec: SpecInstance,
    pipe: Vec<PipeSegment>,
    collector_pipe: Vec<PipeSegment>,
    wg: ExhaustSystem,
    cyls: Vec<Cylinder>,
    far_fields: Vec<FarField>,
    throat_noise: Vec<Noise>,
    clack: Vec<Resonator>,
    slap: Vec<Resonator>,
    clack_impact: Vec<Impact>,
    slap_impact: Vec<Impact>,
    head_share: f64,
    /// Piston slaps triggered since construction.
    pub slap_count: u64,
    /// Each cylinder's exhaust lift, intake lift and exhaust flow area this sample.
    lift_now: Vec<(f64, f64, f64)>,
    cyl_state: Vec<CylState>,
    tdc_pressure: Vec<f64>,
    structure: Vec<Resonator>,
    dpdt_smooth: f64,
    dpdt_smooth_c: f64,
    structure_lp_c: f64,
    structure_lp1: f64,
    structure_lp2: f64,
    listener: Listener,
    mouth_delays: Vec<Delay>,
    mouth_gains: Vec<f64>,
    breathing: Vec<f64>,
    timing: Vec<f64>,
    last_valve_mdot: Vec<f64>,
    valve_states: Vec<ValveState>,
    in_valves: Vec<ValveState>,
    intake_long: IntakeRunners,
    intake_short: Option<IntakeRunners>,
    on_short_runners: bool,
    high_cam_spec: Option<SpecInstance>,
    on_high_cam: bool,
    intake_shift: f64,
    exhaust_shift: f64,
    inject_fraction: f64,
    full_charge_kg: f64,
    torque_last: f64,
    ex_lift: Vec<f64>,
    in_lift: Vec<f64>,
    seating_now: Vec<bool>,
    in_seating_now: Vec<bool>,
    seat_pulse: Vec<Impact>,
    /// CFL substeps the gas solver took last sample.
    pub substeps: usize,
    turb1: Vec<f64>,
    turb2: Vec<f64>,
    omega: f64,
    omega_mean: f64,
    omega_ripple: f64,
    torque_avg: f64,
    omega_display: f64,
    limiter_cut: bool,
    fuel_cut_active: bool,
    dyno: Option<DynoRun>,
    dyno_opening: f64,
    prev_ex_lift: Vec<f64>,
    prev_in_lift: Vec<f64>,
    prev_angle: Vec<f64>,
    rebuild_ramp: f64,
    rebuild_ramp_step: f64,
    peak: f64,
    tap_buffer: Vec<f32>,
    duct_pressure: Vec<f32>,
    duct_cells: Vec<u32>,
    displacement_m3: f64,
    load_torque_nm: f64,
    plenum: IntakePlenum,
    /// The turbocharger, on a turbocharged engine, and the air the throttle draws from: its charge
    /// air, or the atmosphere.
    turbo: Option<Turbo>,
    charge_p: f64,
    charge_t: f64,
    graph: Option<ExhaustGraph>,
    wg_options: EulerPipeOptions,
}

impl EngineSim {
    pub fn new(sample_rate: f64, config: &EngineConfig) -> EngineSim {
        EngineSim::with_options(sample_rate, config, EulerPipeOptions::default(), None)
    }

    /// `wg_options` override what the engine would give its ducts; an explicit `graph` wins over the
    /// config's own.
    pub fn with_options(
        sample_rate: f64,
        config: &EngineConfig,
        wg_options: EulerPipeOptions,
        graph: Option<ExhaustGraph>,
    ) -> EngineSim {
        let graph = graph.or_else(|| config.graph.clone());
        let spec = SpecInstance::new(config.engine.clone());
        let pipe: Vec<PipeSegment> = config.pipe.clone();
        let collector_pipe: Vec<PipeSegment> = config.collector.clone();
        // Replaced by `build_intake` once the cylinders exist.
        let intake_long = IntakeRunners::new(&spec.spec, sample_rate, 0, &EulerPipeOptions::default(), 0.3);

        let wg = build_exhaust_for(&spec.spec, &pipe, &collector_pipe, graph.as_ref(), sample_rate, &wg_options, None);
        let plenum = IntakePlenum::new(&spec.spec);

        let mut sim = EngineSim {
            sample_rate,
            spec,
            pipe,
            collector_pipe,
            wg,
            cyls: Vec::new(),
            far_fields: Vec::new(),
            throat_noise: Vec::new(),
            clack: Vec::new(),
            slap: Vec::new(),
            clack_impact: Vec::new(),
            slap_impact: Vec::new(),
            head_share: 1.0,
            slap_count: 0,
            lift_now: Vec::new(),
            cyl_state: Vec::new(),
            tdc_pressure: Vec::new(),
            structure: Vec::new(),
            dpdt_smooth: 0.0,
            dpdt_smooth_c: 1.0 - math::exp((-2.0 * PI * DPDT_BANDWIDTH_HZ) / sample_rate),
            structure_lp_c: 1.0 - math::exp((-2.0 * PI * STRUCTURE_LIMIT_HZ) / sample_rate),
            structure_lp1: 0.0,
            structure_lp2: 0.0,
            listener: Listener::new(sample_rate),
            mouth_delays: Vec::new(),
            mouth_gains: vec![0.0; 1],
            breathing: Vec::new(),
            timing: Vec::new(),
            last_valve_mdot: Vec::new(),
            valve_states: Vec::new(),
            in_valves: Vec::new(),
            intake_long,
            intake_short: None,
            on_short_runners: false,
            high_cam_spec: None,
            on_high_cam: false,
            intake_shift: 0.0,
            exhaust_shift: 0.0,
            inject_fraction: 0.0,
            full_charge_kg: 1.0,
            torque_last: 0.0,
            ex_lift: Vec::new(),
            in_lift: Vec::new(),
            seating_now: Vec::new(),
            in_seating_now: Vec::new(),
            seat_pulse: Vec::new(),
            substeps: 0,
            turb1: Vec::new(),
            turb2: Vec::new(),
            omega: 0.0,
            omega_mean: 0.0,
            omega_ripple: 0.0,
            torque_avg: 0.0,
            omega_display: 0.0,
            limiter_cut: false,
            fuel_cut_active: false,
            dyno: None,
            dyno_opening: f64::NAN,
            prev_ex_lift: Vec::new(),
            prev_in_lift: Vec::new(),
            prev_angle: Vec::new(),
            rebuild_ramp: 1.0,
            rebuild_ramp_step: 1.0 / (0.008 * sample_rate),
            peak: 0.0,
            tap_buffer: vec![0.0; PIPE_PRESSURE_TAPS],
            duct_pressure: Vec::new(),
            duct_cells: Vec::new(),
            displacement_m3: 0.0,
            load_torque_nm: 0.0,
            plenum,
            turbo: None,
            charge_p: gas::P_AMB,
            charge_t: gas::T_AMB,
            graph,
            wg_options,
        };
        sim.refresh_cam_profiles(false);
        sim.refresh_derived();
        sim.cyls = sim.build_cylinders();
        sim.allocate_per_cylinder();
        sim.build_intake();
        sim.structure = STRUCTURAL_MODES.iter().map(|&(hz, q)| Resonator::new(hz, q, sample_rate)).collect();
        sim.tune_structure();
        sim.set_listener_geometry();
        sim.omega_mean = (math::min(sim.spec.spec.rpm, sim.spec.spec.rev_limit) * 2.0 * PI) / 60.0;
        sim.omega = sim.omega_mean;
        sim.omega_display = sim.omega_mean;
        sim.refresh_far_fields();
        sim.refresh_mouth_paths();
        sim.refresh_turbo();
        sim
    }

    /// Fit, resize or remove the turbocharger to match the exhaust: there is one when a turbo placed in
    /// the exhaust has pipes feeding it.
    fn refresh_turbo(&mut self) {
        let count = self.wg.turbine_count();
        if count == 0 {
            if self.turbo.take().is_some() {
                self.wg.set_turbine(None);
            }
            self.charge_p = gas::P_AMB;
            self.charge_t = gas::T_AMB;
        } else {
            let spec = &self.spec.spec;
            match &mut self.turbo {
                Some(t) => t.configure(spec, count),
                None => self.turbo = Some(Turbo::new(spec, count, self.sample_rate)),
            }
        }
        self.load_torque_nm = load_torque_of(&self.spec.spec, self.turbo.is_some());
    }

    /// The turbocharger, on a turbocharged engine.
    pub fn turbo(&self) -> Option<&Turbo> {
        self.turbo.as_ref()
    }

    fn refresh_derived(&mut self) {
        let spec = &self.spec.spec;
        self.inject_fraction = fuel_fraction_at(spec.lambda);
        self.full_charge_kg = (gas::P_AMB * displacement(spec)) / (gas::R * gas::T_AMB);
        self.displacement_m3 = displacement(spec) * spec.cylinders as f64;
        self.load_torque_nm = load_torque_of(spec, self.turbo.is_some());
    }

    fn set_listener_geometry(&mut self) {
        let spec = &self.spec.spec;
        self.listener.set_geometry(ListenerGeometry {
            distance: spec.mic_distance,
            mic_height: spec.mic_height,
            source_height: spec.exhaust_height,
            reflection: spec.ground_reflection,
        });
    }

    /// One far field per mouth, tuned to that mouth. Keeps existing filters' state.
    fn refresh_far_fields(&mut self) {
        let count = self.wg.mouth_count().max(1);
        while self.far_fields.len() < count {
            let m = self.far_fields.len();
            self.far_fields.push(FarField::new(self.sample_rate, self.wg.mouth_cutoff_rad_of(m)));
        }
        self.far_fields.truncate(count);
        for m in 0..count {
            let (c, b) = (self.wg.mouth_cutoff_rad_of(m), self.wg.band_limit_rad_of(m));
            self.far_fields[m].set_cutoff(c, b);
        }
    }

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    /// The operating point alone: throttle and load. Allocates nothing.
    pub fn set_controls(&mut self, throttle: f64, load: f64) {
        let spec = &mut self.spec.spec;
        if throttle == spec.throttle && load == spec.load {
            return;
        }
        spec.throttle = throttle;
        spec.load = load;
        self.load_torque_nm = load_torque_of(spec, self.turbo.is_some());
        self.plenum.set_geometry(spec);
        self.dyno_opening = f64::NAN;
    }

    /// Start a dyno run through `config`'s gearbox, from the engine's present speed.
    pub fn start_dyno(&mut self, config: DynoConfig) {
        self.dyno =
            Some(DynoRun::new(config, self.omega_mean, full_load_torque(&self.spec.spec, self.turbo.is_some())));
        self.dyno_opening = f64::NAN;
    }

    /// End the dyno run.
    pub fn stop_dyno(&mut self) {
        if let Some(d) = &mut self.dyno {
            d.finish();
        }
    }

    /// Replace the engine with `next`: the whole spec, as the web app's `{ ...spec, ...partial }`.
    pub fn set_engine(&mut self, next: EngineSpec) {
        let prev = &self.spec.spec;
        let prev_temp = prev.port_gas_temp;
        let prev_port_length = prev.port_length;
        let prev_port_dia = exhaust_port_diameter(prev);
        let prev_cell_size = prev.pipe_cell_size;
        let prev_wall_thickness = prev.pipe_wall_thickness;
        let prev_runner: RunnerSize = intake_runner_of(prev);
        let prev_short_runner = prev.intake_runner_short_length;
        let prev_layout = layout_key(prev);
        let prev_phase = firing_plan(prev).offsets;
        let was_high = self.high_cam_spec.is_some() && self.on_high_cam;
        self.spec = SpecInstance::new(next);
        self.refresh_cam_profiles(was_high);
        self.refresh_derived();
        if !self.spec.spec.free_running && !self.integrating_crank() {
            self.omega_mean = (self.spec.spec.rpm * 2.0 * PI) / 60.0;
        }
        self.set_listener_geometry();
        self.refresh_mouth_paths();
        self.wg.set_turbulence(self.spec.spec.throat_noise);
        self.plenum.set_geometry(&self.spec.spec);
        self.refresh_turbo();
        self.dyno_opening = f64::NAN;
        self.make_cylinder_variation(self.spec.spec.cylinders as usize);
        self.tune_structure();
        let spec = &self.spec.spec;
        let runner = intake_runner_of(spec);
        if runner.length != prev_runner.length
            || runner.diameter != prev_runner.diameter
            || spec.intake_runner_short_length != prev_short_runner
            || spec.port_gas_temp != prev_temp
            || spec.port_length != prev_port_length
            || exhaust_port_diameter(spec) != prev_port_dia
            || spec.pipe_cell_size != prev_cell_size
            || spec.pipe_wall_thickness != prev_wall_thickness
            || layout_key(spec) != prev_layout
        {
            if layout_key(spec) != prev_layout {
                self.dyno = None;
                self.allocate_per_cylinder();
                self.cyls = self.build_cylinders();
            }
            self.rebuild_pipe();
        } else if !same_offsets(&firing_plan(spec).offsets, &prev_phase) {
            // Re-phase in place, keeping the gas state.
            let plan = firing_plan(spec);
            let a0 = self.cyls[0].angle;
            for b in 1..self.cyls.len() {
                self.cyls[b].angle = wrap_cycle(a0 - plan.offsets[b]);
            }
        } else {
            self.wg.set_air_speed(spec.air_speed);
        }
    }

    /// `set_engine` with a JSON object of the fields to change, in the web app's shape.
    pub fn set_engine_json(&mut self, patch: &serde_json::Value) -> Result<(), serde_json::Error> {
        let next = self.spec.spec.merged(patch)?;
        self.set_engine(next);
        Ok(())
    }

    /// Replace the whole duct graph, for an exhaust that was drawn rather than chosen.
    pub fn set_graph(&mut self, graph: Option<ExhaustGraph>) {
        self.graph = graph;
        self.rebuild_pipe();
    }

    pub fn set_pipe(&mut self, pipe: &[PipeSegment], collector: Option<&[PipeSegment]>) {
        self.pipe = pipe.to_vec();
        if let Some(c) = collector {
            self.collector_pipe = c.to_vec();
        }
        self.rebuild_pipe();
    }

    fn rebuild_pipe(&mut self) {
        self.wg = self.build_exhaust();
        self.build_intake();
        self.last_valve_mdot.fill(0.0);
        self.refresh_far_fields();
        for f in self.far_fields.iter_mut() {
            f.reset();
        }
        self.refresh_mouth_paths();
        self.rebuild_ramp = 0.0;
        self.refresh_turbo();
    }

    fn requested_cell_size(&self) -> f64 {
        let min_dx = single_step_dx(self.sample_rate, self.wg_options.cfl.unwrap_or(DEFAULT_CFL));
        math::max(math::max(self.spec.spec.pipe_cell_size, min_dx), 1e-4)
    }

    fn build_options(&self, inherit: Option<Vec<f64>>) -> EulerPipeOptions {
        build_options_for(
            &self.spec.spec,
            &self.pipe,
            &self.collector_pipe,
            self.graph.as_ref(),
            self.sample_rate,
            &self.wg_options,
            inherit,
        )
    }

    fn build_intake(&mut self) {
        let opts = EulerPipeOptions {
            cell_size: Some(self.requested_cell_size()),
            ..self.build_options(Some(self.wg.export_wall()))
        };
        let n = self.cyls.len();
        let burned = self.plenum.burned_fraction();
        let sample_rate = self.sample_rate;
        let spec = &self.spec.spec;
        let build = |length: f64| -> IntakeRunners {
            let mut runners = IntakeRunners::new(spec, sample_rate, n, &opts, length);
            runners.prime(burned, 0.0);
            runners
        };
        let long = build(intake_runner_of(spec).length);
        let short =
            if spec.intake_runner_short_length > 0.0 { Some(build(spec.intake_runner_short_length)) } else { None };
        let switch = spec.intake_switch_rpm;
        self.intake_long = long;
        self.intake_short = short;
        let rpm = (self.omega_mean * 60.0) / (2.0 * PI);
        self.on_short_runners = self.intake_short.is_some() && rpm >= switch;
    }

    fn refresh_cam_profiles(&mut self, high: bool) {
        let spec = &self.spec.spec;
        self.high_cam_spec = if spec.cam_switch_rpm > 0.0 {
            Some(self.spec.with_cams(EngineSpec {
                evo: spec.high_evo,
                evc: spec.high_evc,
                ivo: spec.high_ivo,
                ivc: spec.high_ivc,
                max_lift: spec.high_max_lift,
                ..spec.clone()
            }))
        } else {
            None
        };
        self.on_high_cam = high && self.high_cam_spec.is_some();
    }

    fn update_cam_profile(&mut self) {
        let rpm = (self.omega_mean * 60.0) / (2.0 * PI);
        let switch = self.spec.spec.cam_switch_rpm;
        if !self.on_high_cam && rpm >= switch {
            self.on_high_cam = true;
        } else if self.on_high_cam && rpm < switch - CAM_SWITCH_HYSTERESIS {
            self.on_high_cam = false;
        }
    }

    fn update_intake_stage(&mut self) {
        let rpm = (self.omega_mean * 60.0) / (2.0 * PI);
        let switch = self.spec.spec.intake_switch_rpm;
        let Some(short) = &mut self.intake_short else { return };
        if !self.on_short_runners && rpm >= switch {
            short.take_state_from(&self.intake_long);
            self.on_short_runners = true;
        } else if self.on_short_runners && rpm < switch - INTAKE_SWITCH_HYSTERESIS {
            self.intake_long.take_state_from(short);
            self.on_short_runners = false;
        }
    }

    fn update_phasers(&mut self) {
        let spec = &self.spec.spec;
        let dt = 1.0 / self.sample_rate;
        let throttle = match &self.dyno {
            Some(d) => d.throttle,
            None => spec.throttle,
        };
        let load = clamp((throttle - 0.1) / 0.4, 0.0, 1.0);
        let rpm = (self.omega_mean * 60.0) / (2.0 * PI);
        let speed = clamp((rpm - spec.vvt_low_rpm) / math::max(spec.vvt_high_rpm - spec.vvt_low_rpm, 1.0), 0.0, 1.0);
        let advance = load * (spec.vvt_intake_low + (spec.vvt_intake_high - spec.vvt_intake_low) * speed);
        let retard = load * (spec.vvt_exhaust_low + (spec.vvt_exhaust_high - spec.vvt_exhaust_low) * speed);
        let intake_target = -advance;
        let exhaust_target = if spec.vvt_linked { intake_target } else { retard };
        let step = PHASER_RATE * dt;
        self.intake_shift += clamp(intake_target - self.intake_shift, -step, step);
        self.exhaust_shift += clamp(exhaust_target - self.exhaust_shift, -step, step);
    }

    pub fn engine(&self) -> &EngineSpec {
        &self.spec.spec
    }

    pub fn pipe_solver(&self) -> &ExhaustSystem {
        &self.wg
    }

    /// Bank 0's cylinder.
    pub fn cylinder(&self) -> &Cylinder {
        &self.cyls[0]
    }

    pub fn cylinders(&self) -> &[Cylinder] {
        &self.cyls
    }

    /// The finite intake manifold.
    pub fn plenum(&self) -> &IntakePlenum {
        &self.plenum
    }

    /// The intake runners the cylinders breathe through now.
    pub fn intake(&self) -> &IntakeRunners {
        if self.on_short_runners { self.intake_short.as_ref().unwrap() } else { &self.intake_long }
    }

    /// What the structure rings at, Hz: the block's modes, and each cylinder's clack and slap.
    pub fn structural_frequencies(&self) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let hz = |r: &Resonator| r.frequency(self.sample_rate);
        (
            self.structure.iter().map(hz).collect(),
            self.clack.iter().map(hz).collect(),
            self.slap.iter().map(hz).collect(),
        )
    }

    /// Number of banks actually running.
    pub fn bank_count(&self) -> usize {
        self.cyls.len()
    }

    fn allocate_per_cylinder(&mut self) {
        let n = self.spec.spec.cylinders as usize;
        let sr = self.sample_rate;
        self.make_cylinder_variation(n);
        self.last_valve_mdot = vec![0.0; n];
        self.turb1 = vec![0.0; n];
        self.turb2 = vec![0.0; n];
        self.ex_lift = vec![0.0; n];
        self.in_lift = vec![0.0; n];
        self.prev_ex_lift = vec![0.0; n];
        self.prev_in_lift = vec![0.0; n];
        self.prev_angle = vec![0.0; n];
        self.seating_now = vec![false; n];
        self.seat_pulse = (0..n).map(|_| Impact::new(VALVE_CONTACT_S, sr)).collect();
        self.in_seating_now = vec![false; n];
        self.tdc_pressure = vec![-1.0; n];
        self.lift_now = vec![(0.0, 0.0, 0.0); n];
        self.cyl_state = vec![CylState::default(); n];
        self.clack = (0..n).map(|_| Resonator::new(CLACK_MODE.0, CLACK_MODE.1, sr)).collect();
        self.slap = (0..n).map(|_| Resonator::new(SLAP_MODE.0, SLAP_MODE.1, sr)).collect();
        self.clack_impact = (0..n).map(|_| Impact::new(VALVE_CONTACT_S, sr)).collect();
        self.slap_impact = (0..n).map(|_| Impact::new(SLAP_CONTACT_S, sr)).collect();
        if !self.structure.is_empty() {
            self.tune_structure();
        }
        self.valve_states = vec![
            ValveState {
                throat_area: 0.0,
                cyl_pressure: gas::P_AMB,
                cyl_temp: gas::T_AMB,
                cyl_gamma: gas::GAMMA_EXH,
                extra_mass_flow: 0.0,
            };
            n
        ];
        self.in_valves = self.valve_states.clone();
    }

    /// Pitch everything that rings to the size of this engine.
    fn tune_structure(&mut self) {
        let spec = &self.spec.spec;
        let size = clamp(math::cbrt(displacement(spec) / REFERENCE_DISPLACEMENT_M3), 0.5, 3.0);
        for (i, &(hz, q)) in STRUCTURAL_MODES.iter().enumerate() {
            self.structure[i].set(hz / size, q, self.sample_rate);
        }
        self.head_share = clack_share(spec.cylinders as f64 / physical_bank_count(spec) as f64);

        let n = self.clack.len();
        let valve = REFERENCE_EX_VALVE / math::max(spec.ex_valve_dia, 1e-3);
        let bore = REFERENCE_BORE / math::max(spec.bore, 1e-3);
        for b in 0..n {
            let t = spread_of(b, n, 7, 3);
            let u = spread_of(b, n, 3, 2);
            self.clack[b].set(CLACK_MODE.0 * valve * (1.0 + LOCAL_MODE_DETUNE * t), CLACK_MODE.1, self.sample_rate);
            self.slap[b].set(SLAP_MODE.0 * bore * (1.0 + LOCAL_MODE_DETUNE * u), SLAP_MODE.1, self.sample_rate);
        }
    }

    /// Cylinder `b`'s valve lifts and exhaust flow area this sample.
    #[inline]
    fn compute_lifts(&mut self, b: usize) {
        let spec = if self.on_high_cam { &self.high_cam_spec.as_ref().unwrap().spec } else { &self.spec.spec };
        let angle = self.cyls[b].angle;
        let ex_offset = self.timing[b] + self.exhaust_shift;
        let in_offset = self.timing[b] + self.intake_shift;
        let ex_lift = valve_lift(angle, spec.evo + ex_offset, spec.evc + ex_offset, spec.max_lift);
        let in_lift = valve_lift(angle, spec.ivo + in_offset, spec.ivc + in_offset, spec.max_lift);
        self.lift_now[b] = (ex_lift, in_lift, valve_flow_area(ex_lift, spec.ex_valve_dia) * spec.ex_valve_count);
    }

    /// Fixed per-cylinder breathing multipliers and cam timing offsets, spread evenly and shuffled.
    fn make_cylinder_variation(&mut self, n: usize) {
        let spread = clamp(self.spec.spec.cylinder_spread, 0.0, 2.0);
        self.breathing = vec![0.0; n];
        self.timing = vec![0.0; n];
        for b in 0..n {
            let t = spread_of(b, n, 5, 2);
            let u = spread_of(b, n, 3, 1);
            self.breathing[b] = 1.0 + 0.04 * spread * t;
            self.timing[b] = CAM_SPREAD_DEG * spread * u;
        }
    }

    /// Each mouth's path to the ear: the mouths in a line, the listener off to one side, and each
    /// mouth's extra delay and spreading loss relative to the nearest.
    fn refresh_mouth_paths(&mut self) {
        let count = self.wg.mouth_count().max(1);
        let c = ambient_sound_speed();
        let spacing = math::max(self.spec.spec.mouth_spacing, 0.0);
        let distance = math::max(self.spec.spec.mic_distance, 0.15);
        const AZIMUTH: f64 = PI / 4.0;
        let lx = distance * math::cos(AZIMUTH);
        let ly = distance * math::sin(AZIMUTH);

        let mut ranges = vec![0.0; count];
        let mut nearest = f64::INFINITY;
        for m in 0..count {
            let lateral = (m as f64 - (count as f64 - 1.0) / 2.0) * spacing;
            ranges[m] = math::hypot(&[lx - lateral, ly]);
            nearest = math::min(nearest, ranges[m]);
        }
        if self.mouth_delays.len() != count {
            self.mouth_delays = (0..count).map(|_| Delay::new(((4.0 / c) * self.sample_rate).ceil())).collect();
            self.mouth_gains = vec![0.0; count];
        }
        for m in 0..count {
            self.mouth_delays[m].set_delay(((ranges[m] - nearest) / c) * self.sample_rate);
            self.mouth_gains[m] = nearest / ranges[m];
        }
    }

    /// The cylinders, phased on one shared crank, each with its own noise seed.
    fn build_cylinders(&mut self) -> Vec<Cylinder> {
        let spec = &self.spec.spec;
        let plan = firing_plan(spec);
        let mut out = Vec::new();
        for b in 0..spec.cylinders as usize {
            out.push(Cylinder::new(spec, wrap_cycle(-plan.offsets[b]), 0x51f3a7 as f64 + b as f64 * 0x9e3779b as f64));
            if self.throat_noise.len() <= b {
                self.throat_noise.push(Noise::new(0x2c1b3d as f64 + b as f64 * 0x85ebca6b_u32 as f64));
            }
        }
        out
    }

    fn build_exhaust(&self) -> ExhaustSystem {
        build_exhaust_for(
            &self.spec.spec,
            &self.pipe,
            &self.collector_pipe,
            self.graph.as_ref(),
            self.sample_rate,
            &self.wg_options,
            Some(self.wg.export_wall()),
        )
    }

    /// First quarter-wave resonance of the exhaust as it is currently filled, Hz.
    pub fn duct_quarter_wave_hz(&self) -> f64 {
        self.wg.quarter_wave_hz()
    }

    /// Mean crank speed, rev/min.
    pub fn rpm(&self) -> f64 {
        let w = if self.integrating_crank() { self.omega_display } else { self.omega_mean };
        (w * 60.0) / (2.0 * PI)
    }

    fn integrating_crank(&self) -> bool {
        let spec = &self.spec.spec;
        spec.free_running || spec.rpm >= spec.rev_limit || self.dyno.is_some()
    }

    /// Instantaneous crank speed, rev/min, ripple included.
    pub fn rpm_instant(&self) -> f64 {
        (self.omega * 60.0) / (2.0 * PI)
    }

    // -------------------------------------------------------------------------
    // Simulation
    // -------------------------------------------------------------------------

    /// Advance one audio sample. Returns the listener signal, nominally in [-1, 1].
    pub fn tick(&mut self) -> f64 {
        let dt = 1.0 / self.sample_rate;

        // --- Crank speed ---
        let inertia = math::max(self.spec.spec.flywheel_inertia, 1e-3);
        let torque = self.torque_last;

        if self.integrating_crank() {
            let fmep = 0.8e5 + 120.0 * self.omega_mean;
            let friction = (fmep * self.displacement_m3) / (4.0 * PI);
            let load = if let Some(dyno) = &mut self.dyno {
                let mut fresh = 0.0;
                for c in &self.cyls {
                    fresh += c.trapped_fresh;
                }
                dyno.volumetric_efficiency = fresh / self.cyls.len() as f64 / self.full_charge_kg;
                dyno.intake_pressure = self.plenum.pressure();
                dyno.step(dt, self.omega_mean, torque - friction, self.cyls[0].angle)
            } else if self.spec.spec.free_running {
                self.load_torque_nm
            } else {
                0.0
            };
            let net = torque - load - friction;
            self.omega_mean += (net / inertia) * dt;
            let min_omega = (MIN_RPM * 2.0 * PI) / 60.0;
            let max_omega = (12000.0 * 2.0 * PI) / 60.0;
            self.omega_mean = clamp(self.omega_mean, min_omega, max_omega);
            self.omega = self.omega_mean;
            self.omega_display += (self.omega_mean - self.omega_display) * (dt / IRREGULARITY_TAU);
            let spec = &self.spec.spec;
            let limit_omega = (spec.rev_limit * 2.0 * PI) / 60.0;
            if self.omega_mean >= limit_omega {
                self.limiter_cut = true;
            } else if self.omega_mean < ((spec.rev_limit - REV_LIMIT_HYSTERESIS_RPM) * 2.0 * PI) / 60.0 {
                self.limiter_cut = false;
            }
        } else {
            // Holding a mean speed, with the within-cycle ripple left free.
            self.omega_mean = (self.spec.spec.rpm * 2.0 * PI) / 60.0;
            self.omega_display = self.omega_mean;
            self.limiter_cut = false;
            self.torque_avg += (torque - self.torque_avg) * (dt / IRREGULARITY_TAU);
            let leak = math::exp(-dt / (IRREGULARITY_TAU * 4.0));
            self.omega_ripple = self.omega_ripple * leak + ((torque - self.torque_avg) / inertia) * dt;
            self.omega_ripple = clamp(self.omega_ripple, -0.35 * self.omega_mean, 0.35 * self.omega_mean);
            self.omega = math::max(self.omega_mean + self.omega_ripple, 1.0);
        }

        // --- Variable valve timing, intake stage, cam profile ---
        {
            let spec = &self.spec.spec;
            if spec.vvt_intake_low != 0.0
                || spec.vvt_intake_high != 0.0
                || spec.vvt_exhaust_low != 0.0
                || spec.vvt_exhaust_high != 0.0
                || self.intake_shift != 0.0
                || self.exhaust_shift != 0.0
            {
                self.update_phasers();
            }
        }
        if self.intake_short.is_some() {
            self.update_intake_stage();
        }
        if self.high_cam_spec.is_some() {
            self.update_cam_profile();
        }

        // --- Dyno run ---
        let mut throttle = self.spec.spec.throttle;
        if let Some((dyno_throttle, cooldown, phase_time)) =
            self.dyno.as_ref().map(|d| (d.throttle, d.phase == DynoPhase::Cooldown, d.phase_time))
        {
            throttle = dyno_throttle;
            if throttle != self.dyno_opening {
                self.plenum.set_opening(&self.spec.spec, throttle);
                self.dyno_opening = throttle;
            }
            let spec = &self.spec.spec;
            if cooldown
                && (spec.free_running || self.omega_mean <= (spec.rpm * 2.0 * PI) / 60.0 || phase_time > DYNO_WIND_DOWN)
            {
                self.dyno = None;
                self.plenum.set_geometry(&self.spec.spec);
                self.dyno_opening = f64::NAN;
                throttle = self.spec.spec.throttle;
            }
        }

        // --- Overrun fuel cut ---
        if !self.spec.spec.fuel_cut || throttle > FUEL_CUT_THROTTLE {
            self.fuel_cut_active = false;
        } else {
            let rpm_now = (self.omega_mean * 60.0) / (2.0 * PI);
            if rpm_now > FUEL_CUT_RPM {
                self.fuel_cut_active = true;
            } else if rpm_now < FUEL_RESUME_RPM {
                self.fuel_cut_active = false;
            }
        }

        // --- Valves and flows, per bank ---
        let banks = self.cyls.len();
        let mut torque_sum = 0.0;
        let mut dpdt_sum = 0.0;
        let limiter_cut = self.limiter_cut;
        let rpm = self.rpm();
        for b in 0..banks {
            let angle = self.cyls[b].angle;
            {
                let cyl = &mut self.cyls[b];
                cyl.spark_cut = limiter_cut;
                cyl.intake_cam_offset = self.timing[b] + self.intake_shift;
                cyl.exhaust_cam_offset = self.timing[b] + self.exhaust_shift;
            }
            self.compute_lifts(b);
            let (ex_lift, in_lift, ex_area) = self.lift_now[b];
            let state = self.cyls[b].read_state(&self.spec);
            self.cyl_state[b] = state;
            let p_cyl = state.pressure;
            let t_cyl = state.temp;
            let spec = &self.spec.spec;
            let port_abs = self.wg.primary(b).read_port().0;

            // Throat turbulence, scaled by the previous sample's flow through this valve.
            let mut extra_mass_flow = 0.0;
            if ex_area > 0.0 && spec.throat_noise > 0.0 {
                let throat_rho = port_abs / (gas::R * t_cyl);
                let speed = math::min(
                    self.last_valve_mdot[b].abs() / math::max(throat_rho * ex_area, 1e-9),
                    math::sqrt(gas::GAMMA_CYL * gas::R * t_cyl),
                );
                let strouhal_hz = (0.2 * speed) / spec.ex_valve_dia;
                let k = clamp(1.0 - math::exp((-2.0 * PI * strouhal_hz) / self.sample_rate), 0.02, 0.85);
                let white = self.throat_noise[b].next()
                    * self.last_valve_mdot[b].abs()
                    * TURBULENCE_INTENSITY
                    * spec.throat_noise;
                self.turb1[b] += k * (white - self.turb1[b]);
                self.turb2[b] += k * (self.turb1[b] - self.turb2[b]);
                extra_mass_flow += self.turb2[b];
            }

            let seating = self.prev_ex_lift[b] > 0.0 && ex_lift == 0.0;
            if seating {
                self.seat_pulse[b].trigger(spec.mech_noise * 0.02 * (rpm / 3000.0));
            }
            extra_mass_flow += self.seat_pulse[b].next();

            let cyl_gamma = 1.0 + gas::R / (CV_REF + CV_SLOPE * (t_cyl - T_REF));
            self.valve_states[b] =
                ValveState { throat_area: ex_area, cyl_pressure: p_cyl, cyl_temp: t_cyl, cyl_gamma, extra_mass_flow };
            self.in_valves[b] = ValveState {
                throat_area: valve_flow_area(in_lift, spec.in_valve_dia) * spec.in_valve_count,
                cyl_pressure: p_cyl,
                cyl_temp: t_cyl,
                cyl_gamma,
                extra_mass_flow: 0.0,
            };

            self.ex_lift[b] = ex_lift;
            self.in_lift[b] = in_lift;
            self.seating_now[b] = seating;
            self.in_seating_now[b] = self.prev_in_lift[b] > 0.0 && in_lift == 0.0;
            self.tdc_pressure[b] =
                if crossed_angle(self.prev_angle[b], angle, 0.0) || crossed_angle(self.prev_angle[b], angle, 360.0) {
                    p_cyl
                } else {
                    -1.0
                };
            self.prev_angle[b] = angle;
        }

        // --- Exhaust gas dynamics, all ducts in lockstep, with the turbine in them ---
        if let Some(turbo) = &mut self.turbo {
            self.wg.set_turbine(Some(turbo.turbine_setting()));
        }
        self.wg.advance(dt, &self.valve_states);
        self.substeps = self.wg.result.substeps;

        // --- Intake runners, all in lockstep ---
        let p_plenum = self.plenum.pressure();
        let t_plenum = self.plenum.temp();
        let run_io = RunnerIo {
            dt,
            p: p_plenum,
            rho: p_plenum / (gas::R * t_plenum),
            burned: self.plenum.burned_fraction(),
            fuel: self.plenum.fuel_fraction(),
            inject: if self.fuel_cut_active { 0.0 } else { self.inject_fraction },
        };
        let intake = if self.on_short_runners { self.intake_short.as_mut().unwrap() } else { &mut self.intake_long };
        intake.advance(&run_io, &self.in_valves, &self.breathing, &self.cyl_state);

        // --- Cylinder gas state, sub-stepped ---
        let cam_spec = if self.on_high_cam { self.high_cam_spec.as_ref().unwrap() } else { &self.spec };
        for b in 0..banks {
            let ex_mdot = self.wg.result.valve_mass_flows[b];
            self.last_valve_mdot[b] = ex_mdot;
            let in_mdot = -intake.valve_mass_flows[b];
            let port_temp = self.wg.primary(b).read_port().1;
            let cyl = &mut self.cyls[b];

            let deg_per_sample = (self.omega.abs() * dt * 180.0) / PI;
            let mass_flux = (ex_mdot.abs() + in_mdot.abs()) * dt;
            let mass_ratio = mass_flux / math::max(cyl.mass, 1e-12);
            let n_sub = clamp(
                math::max(
                    (deg_per_sample / MAX_DEG_PER_SUBSTEP).ceil(),
                    (mass_ratio / MAX_MASS_FRACTION_PER_SUBSTEP).ceil(),
                ),
                1.0,
                MAX_CYL_SUBSTEPS,
            );
            let sub_dt = dt / n_sub;
            let io = AdvanceIo {
                dt: sub_dt,
                omega: self.omega,
                ex_mdot,
                in_mdot,
                intake_t: intake.port_temps[b],
                port_t: port_temp,
                intake_burned: intake.burned[b],
                intake_fuel: intake.inflow_fuel[b],
            };
            let steps = n_sub as usize;
            for _ in 0..steps {
                cyl.advance(cam_spec, &io);
            }
            torque_sum += cyl.torque + cyl.inertia_torque;
            dpdt_sum += cyl.dpdt;
            self.prev_ex_lift[b] = self.ex_lift[b];
            self.prev_in_lift[b] = self.in_lift[b];
        }

        // --- Plenum ---
        let mut drawn = 0.0;
        let mut back = 0.0;
        let mut back_t = 0.0;
        let mut back_burned = 0.0;
        let mut back_fuel = 0.0;
        for b in 0..banks {
            let f = intake.plenum_flows[b];
            drawn -= f;
            if f > 0.0 {
                back += f;
                back_t += f * intake.plenum_temps[b];
                back_burned += f * intake.burned[b];
                back_fuel += f * intake.fuel[b];
            }
        }
        let throttle_flow = self.plenum.step(
            dt,
            self.charge_p,
            self.charge_t,
            drawn,
            back,
            if back > 0.0 { back_t / back } else { t_plenum },
            if back > 0.0 { back_burned / back } else { 0.0 },
            if back > 0.0 { back_fuel / back } else { 0.0 },
        );

        // --- Turbocharger ---
        let mut turbo_pa = 0.0;
        if let Some(turbo) = &mut self.turbo {
            let r = &self.wg.result;
            let drive = TurbineDrive { power: r.turbine_power, inlet: r.turbine_inlet, outlet: r.turbine_outlet };
            let out = turbo.step(dt, drive, throttle_flow, self.plenum.pressure());
            self.charge_p = out.charge_p;
            self.charge_t = out.charge_t;
            turbo_pa = out.sound;
        }

        // --- Structure-borne noise ---
        let mut direct_pa = 0.0;
        let mech = self.spec.spec.mech_noise;
        let head_share = self.head_share;
        for b in 0..banks {
            let seat = (if self.seating_now[b] { 1.0 } else { 0.0 }) + (if self.in_seating_now[b] { 0.7 } else { 0.0 });
            if seat > 0.0 {
                self.clack_impact[b].trigger(CLACK_PA_AT_1M * mech * seat * (rpm / 3000.0) * head_share);
            }
            let hit = self.clack_impact[b].next();
            direct_pa += self.clack[b].process(hit);

            let p = self.tdc_pressure[b];
            if p >= 0.0 {
                self.slap_count += 1;
                self.slap_impact[b].trigger(SLAP_PA_AT_1M * mech * clamp(p / 3e6, 0.05, 1.6) * head_share);
            }
            let hit = self.slap_impact[b].next();
            direct_pa += self.slap[b].process(hit);
        }

        // Combustion shaking the casing, driven by the summed pressure rise rate.
        if mech > 0.0 {
            self.dpdt_smooth += self.dpdt_smooth_c * (dpdt_sum - self.dpdt_smooth);
            let drive = (self.dpdt_smooth / 1e9) * STRUCTURE_PA_PER_GPA_S * mech;
            for mode in self.structure.iter_mut() {
                direct_pa += mode.process(drive) * 0.25;
            }
        }

        // Band limit for everything structure-borne.
        self.structure_lp1 += self.structure_lp_c * (direct_pa - self.structure_lp1);
        self.structure_lp2 += self.structure_lp_c * (self.structure_lp1 - self.structure_lp2);
        direct_pa = self.structure_lp2;

        self.torque_last = torque_sum;

        // --- Radiate ---
        let flows = &self.wg.result.mouth_flows;
        let mut exhaust_pa = 0.0;
        if flows.len() == 1 {
            exhaust_pa = self.far_fields[0].process(flows[0]);
        } else {
            for m in 0..flows.len() {
                let delayed = self.mouth_delays[m].process(flows[m]);
                exhaust_pa += self.far_fields[m].process(delayed) * self.mouth_gains[m];
            }
        }
        let mut pa = if self.turbo.is_some() {
            self.listener.process(exhaust_pa + direct_pa + turbo_pa)
        } else {
            self.listener.process(exhaust_pa + direct_pa)
        };

        if self.rebuild_ramp < 1.0 {
            self.rebuild_ramp = math::min(1.0, self.rebuild_ramp + self.rebuild_ramp_step);
            pa *= self.rebuild_ramp;
        }

        let mut out = (pa / PA_PER_FULLSCALE) * self.spec.spec.output_gain;
        if !out.is_finite() {
            out = 0.0;
        }
        out = soft_clip(out);

        let mag = out.abs();
        if mag > self.peak {
            self.peak = mag;
        }
        out
    }

    /// A snapshot for the renderer. Resets the peak meter.
    pub fn snapshot(&mut self) -> EngineSnapshot {
        self.wg.sample_pressure(&mut self.tap_buffer);
        self.wg.sample_duct_pressures(&mut self.duct_pressure, &mut self.duct_cells);
        let spec = &self.spec.spec;
        let cams = if self.on_high_cam { &self.high_cam_spec.as_ref().unwrap().spec } else { spec };
        let banks: Vec<BankSnapshot> = self
            .cyls
            .iter()
            .map(|cyl| BankSnapshot {
                crank_angle: wrap_cycle(cyl.angle),
                cyl_pressure: cyl.pressure(spec),
                cyl_temp: cyl.temp(),
                ex_lift: valve_lift(
                    cyl.angle,
                    cams.evo + self.exhaust_shift,
                    cams.evc + self.exhaust_shift,
                    cams.max_lift,
                ),
                in_lift: valve_lift(
                    cyl.angle,
                    cams.ivo + self.intake_shift,
                    cams.ivc + self.intake_shift,
                    cams.max_lift,
                ),
            })
            .collect();
        let first = banks[0].clone();
        let dyno = self.dyno.as_mut().map(|d| DynoSnapshot {
            phase: d.phase.as_str().to_string(),
            gear: (d.gear + 1) as f64,
            speed_kmh: d.speed * 3.6,
            elapsed: d.elapsed,
            finished: d.finished,
            points: d.take_points(),
        });
        let snap = EngineSnapshot {
            crank_angle: first.crank_angle,
            rpm: self.rpm(),
            limiter: self.limiter_cut,
            fuel_cut: self.fuel_cut_active,
            intake_cam_advance: -self.intake_shift,
            exhaust_cam_retard: self.exhaust_shift,
            short_runners: self.on_short_runners,
            high_cam: self.high_cam_spec.is_some() && self.on_high_cam,
            dyno,
            cyl_pressure: first.cyl_pressure,
            cyl_temp: first.cyl_temp,
            ex_lift: first.ex_lift,
            in_lift: first.in_lift,
            torque: self.torque_last,
            pipe_pressure: self.tap_buffer.clone(),
            duct_pressure: self.duct_pressure.clone(),
            duct_cells: self.duct_cells.clone(),
            duct_ids: self.wg.duct_ids.clone(),
            peak: self.peak,
            pipe_cells: (self.wg.cells() + self.intake().cells()) as f64,
            substeps: self.substeps as f64,
            wall_temp: self.wg.mean_wall_temp(),
            turbo: self.turbo.as_ref().map(|t| TurboSnapshot {
                boost: t.boost(),
                manifold: self.plenum.pressure() - gas::P_AMB,
                turbine_inlet: t.back_pressure() - gas::P_AMB,
                shaft_rpm: t.shaft_rpm(),
                wastegate: t.wastegate(),
                blow_off: t.blow_off(),
                surging: t.surging(),
            }),
            banks,
        };
        self.peak = 0.0;
        snap
    }

    /// Render `out.len()` samples into `out`.
    pub fn render_into(&mut self, out: &mut [f32]) {
        for o in out.iter_mut() {
            *o = self.tick() as f32;
        }
    }

    /// Render `n` samples into a new buffer.
    pub fn render(&mut self, n: usize) -> Vec<f32> {
        let mut out = vec![0.0; n];
        self.render_into(&mut out);
        out
    }
}

/// The stored graph if it still describes this engine, otherwise one compiled from the layout.
fn usable_graph(
    spec: &EngineSpec,
    pipe: &[PipeSegment],
    collector: &[PipeSegment],
    stored: Option<&ExhaustGraph>,
) -> ExhaustGraph {
    if let Some(g) = stored {
        if validate_graph(g, spec.cylinders as usize).is_empty() {
            return g.clone();
        }
    }
    compile_exhaust(spec, pipe, collector)
}

/// Cell size that keeps the solver inside its cost budget: the finest at or above the request that
/// fits, or failing that the cheapest.
fn budgeted_cell_size(spec: &EngineSpec, graph: &ExhaustGraph, sample_rate: f64, wg_options: &EulerPipeOptions) -> f64 {
    let cfl = wg_options.cfl.unwrap_or(DEFAULT_CFL);
    let max_cells = wg_options.max_cells.unwrap_or(DEFAULT_MAX_CELLS);
    let min_dx = single_step_dx(sample_rate, cfl);
    let requested = math::max(math::max(spec.pipe_cell_size, min_dx), 1e-4);

    let port = HeadPort { length: spec.port_length, diameter: exhaust_port_diameter(spec) };
    let lengths: Vec<f64> = graph
        .ducts
        .iter()
        .map(|d| duct_grid_length(&d.segments, if d.is_node_fed() { None } else { Some(port) }))
        .collect();
    let runner_cells = spec.cylinders as f64
        * duct_cell_count(
            math::max(intake_runner_of(spec).length, spec.intake_runner_short_length),
            requested,
            max_cells,
            min_dx,
        ) as f64;
    if lengths.is_empty() {
        return requested;
    }
    let cells_at = |cell_size: f64| -> f64 {
        lengths.iter().fold(0.0, |a, &l| a + duct_cell_count(l, cell_size, max_cells, min_dx) as f64)
    };
    let budget_for = grid_budget_cells(spec.cylinders as usize, node_order(graph).len()) - runner_cells;

    let mut best = requested;
    let mut best_cost = cells_at(requested);
    if best_cost <= budget_for {
        return requested;
    }
    let mut dx = requested * 1.01;
    while dx <= 0.15 {
        let c = cells_at(dx);
        if c <= budget_for {
            return dx;
        }
        if c < best_cost {
            best_cost = c;
            best = dx;
        }
        dx *= 1.01;
    }
    best
}

/// Solver options with the cylinder-head port prepended to the user's geometry.
fn build_options_for(
    spec: &EngineSpec,
    pipe: &[PipeSegment],
    collector: &[PipeSegment],
    stored: Option<&ExhaustGraph>,
    sample_rate: f64,
    wg_options: &EulerPipeOptions,
    inherit: Option<Vec<f64>>,
) -> EulerPipeOptions {
    let graph = usable_graph(spec, pipe, collector, stored);
    let base = EulerPipeOptions {
        cell_size: Some(budgeted_cell_size(spec, &graph, sample_rate, wg_options)),
        single_step: Some(true),
        wall_thickness: Some(spec.pipe_wall_thickness),
        air_speed: Some(spec.air_speed),
        inherit_wall: inherit,
        ..Default::default()
    };
    let mut opts = base.overlaid(wg_options);
    opts.port =
        Some(wg_options.port.unwrap_or(HeadPort { length: spec.port_length, diameter: exhaust_port_diameter(spec) }));
    opts
}

fn build_exhaust_for(
    spec: &EngineSpec,
    pipe: &[PipeSegment],
    collector: &[PipeSegment],
    stored: Option<&ExhaustGraph>,
    sample_rate: f64,
    wg_options: &EulerPipeOptions,
    inherit: Option<Vec<f64>>,
) -> ExhaustSystem {
    let graph = usable_graph(spec, pipe, collector, stored);
    let opts = build_options_for(spec, pipe, collector, stored, sample_rate, wg_options, inherit);
    let mut sys = ExhaustSystem::new(&graph, spec.cylinders as usize, sample_rate, spec.port_gas_temp, &opts)
        .expect("a compiled or validated graph is solvable");
    sys.set_turbulence(spec.throat_noise);
    sys
}

/// True if the crank swept past `target` degrees between two samples.
#[inline]
fn crossed_angle(from: f64, to: f64, target: f64) -> bool {
    if to >= from {
        return target > from && target <= to;
    }
    target > from || target <= to
}
