//! The intake runners: one duct per cylinder, from the plenum to the intake valve, solved with the
//! same gas dynamics as the exhaust.
//!
//! A runner is what lets an engine fill past what the plenum's pressure alone could push in: the
//! column of air it holds has momentum, and its pressure waves reflect off the plenum and arrive back
//! at the valve in step at some speeds and out of step at others. That is where an engine's torque
//! peak comes from.
//!
//! Each runner carries a port injector that meters fuel on the net fresh air its cylinder draws.
//! The solver carries no composition, so each runner's is tracked alongside it as one well-mixed
//! fraction of spent gas and one of fuel.

use crate::cylinder::CylState;
use crate::euler_pipe::{EulerPipe, EulerPipeOptions, InletKind, OutletKind, ValveState};
use crate::math::{self, PI};
use crate::spec::{EngineSpec, SegmentKind, SegmentPartial, gas, intake_runner_of, make_segment, speed_of_sound};

/// The plenum's state and the injectors' setting for one `IntakeRunners::advance`.
#[derive(Clone, Copy, Debug, Default)]
pub struct RunnerIo {
    pub dt: f64,
    /// Plenum pressure, Pa, and density, kg/m^3.
    pub p: f64,
    pub rho: f64,
    /// Plenum spent-gas and fuel fractions.
    pub burned: f64,
    pub fuel: f64,
    /// Fuel mass fraction each injector brings the air its cylinder draws to; 0 with the fuel cut.
    pub inject: f64,
}

pub struct IntakeRunners {
    pub runners: Vec<EulerPipe>,
    /// Mass flow through each intake valve, kg/s, positive out of the cylinder into its runner.
    pub valve_mass_flows: Vec<f64>,
    /// Mass flow out of each runner's plenum end, kg/s, positive into the plenum.
    pub plenum_flows: Vec<f64>,
    /// Gas temperature at each runner's plenum end, K.
    pub plenum_temps: Vec<f64>,
    /// Spent-gas and fuel mass fractions of each runner's contents.
    pub burned: Vec<f64>,
    pub fuel: Vec<f64>,
    /// Gas temperature at each runner's valve end, K.
    pub port_temps: Vec<f64>,
    /// Fuel mass fraction of what each cylinder draws in through its valve.
    pub inflow_fuel: Vec<f64>,
    burned_mass: Vec<f64>,
    fuel_mass: Vec<f64>,
    mass: Vec<f64>,
    /// Air each cylinder has pushed back out and not yet drawn back in, kg, as a negative balance.
    air_owed: Vec<f64>,
}

impl IntakeRunners {
    /// `length` is each runner's length, m; a two-stage intake builds a second set at its short one.
    pub fn new(
        spec: &EngineSpec,
        sample_rate: f64,
        count: usize,
        opts: &EulerPipeOptions,
        length: f64,
    ) -> IntakeRunners {
        let diameter = intake_runner_of(spec).diameter;
        let segment = [make_segment(SegmentPartial {
            kind: Some(SegmentKind::Pipe),
            length: Some(length),
            d_in: Some(diameter),
            ..Default::default()
        })];
        let damping = runner_damping(diameter / 2.0, speed_of_sound(gas::T_AMB, gas::GAMMA_AIR) / (4.0 * length));
        let runner_opts = EulerPipeOptions {
            inlet_kind: Some(InletKind::Valve),
            outlet_kind: Some(OutletKind::Mouth),
            heat_transfer: Some(false),
            initial_port_temp: Some(gas::T_AMB),
            linear_damping: Some(damping),
            port: None,
            inherit_wall: None,
            ..opts.clone()
        };
        let runners: Vec<EulerPipe> =
            (0..count).map(|_| EulerPipe::new(&segment, sample_rate, spec.port_gas_temp, &runner_opts)).collect();
        let z = || vec![0.0; count];
        let mut r = IntakeRunners {
            runners,
            valve_mass_flows: z(),
            plenum_flows: z(),
            plenum_temps: z(),
            port_temps: z(),
            burned: z(),
            fuel: z(),
            burned_mass: z(),
            fuel_mass: z(),
            mass: z(),
            air_owed: z(),
            inflow_fuel: z(),
        };
        for b in 0..count {
            r.mass[b] = r.runners[b].total_mass();
            let t = r.runners[b].read_port().1;
            r.port_temps[b] = t;
            r.plenum_temps[b] = t;
        }
        r
    }

    /// Cells across all the runners.
    pub fn cells(&self) -> usize {
        self.runners.iter().map(|r| r.n).sum()
    }

    /// Solver recoveries across all the runners. Should stay zero.
    pub fn recoveries(&self) -> u64 {
        self.runners.iter().map(|r| r.recoveries).sum()
    }

    /// Take over from `src`, the other set of a two-stage intake, as the manifold switches between
    /// them: the gas, its composition and what each injector is owed.
    pub fn take_state_from(&mut self, src: &IntakeRunners) {
        for b in 0..self.runners.len() {
            let r = &mut self.runners[b];
            r.resample_from(&src.runners[b]);
            let m = r.total_mass();
            self.mass[b] = m;
            self.burned[b] = src.burned[b];
            self.fuel[b] = src.fuel[b];
            self.burned_mass[b] = m * src.burned[b];
            self.fuel_mass[b] = m * src.fuel[b];
            self.air_owed[b] = src.air_owed[b];
            self.inflow_fuel[b] = src.inflow_fuel[b];
            self.valve_mass_flows[b] = src.valve_mass_flows[b];
            self.plenum_flows[b] = src.plenum_flows[b];
            self.port_temps[b] = r.read_port().1;
            self.plenum_temps[b] = r.read_mouth().1;
        }
    }

    /// Prime every runner with a mixture, so the first cycles draw a charge.
    pub fn prime(&mut self, burned: f64, fuel: f64) {
        for b in 0..self.runners.len() {
            let m = self.mass[b];
            self.burned_mass[b] = m * burned;
            self.fuel_mass[b] = m * fuel;
            self.burned[b] = burned;
            self.fuel[b] = fuel;
        }
    }

    /// Advance every runner by one sample, in lockstep. `breathing` is each cylinder's multiplier on
    /// the pressure its runner opens onto; `cyl_state` is each cylinder's contents, from which what it
    /// pushes back up its runner takes its composition.
    pub fn advance(&mut self, io: &RunnerIo, valves: &[ValveState], breathing: &[f64], cyl_state: &[CylState]) {
        let dt = io.dt;
        let p_plenum = io.p;
        let rho_plenum = io.rho;
        let plenum_burned = io.burned;
        let plenum_fuel = io.fuel;
        let inject = io.inject;
        let count = self.runners.len();
        let mut substeps = 1;
        for r in self.runners.iter_mut() {
            let s = r.substeps_for(dt);
            if s > substeps {
                substeps = s;
            }
        }
        let h = dt / substeps as f64;
        self.valve_mass_flows.fill(0.0);
        self.plenum_flows.fill(0.0);

        // Breathing scales pressure and density together, so the plenum's sound speed is every runner's.
        let res_c = math::sqrt((gas::GAMMA_EXH * p_plenum) / rho_plenum);
        for b in 0..count {
            let bp = breathing[b];
            self.runners[b].set_reservoir(p_plenum * bp, rho_plenum * bp, res_c);
        }
        for _ in 0..substeps {
            for b in 0..count {
                let r = &mut self.runners[b];
                r.begin_step(h);
                r.apply_own_boundaries(h);
                self.plenum_flows[b] += r.mouth_mass_flow;
                r.compute_valve_flux(&valves[b]);
                let flow = r.valve_flux_out;
                r.set_end_step(h, flow);
            }
            for b in 0..count {
                let r = &mut self.runners[b];
                r.end_step_set(&valves[b]);
                self.valve_mass_flows[b] += r.valve_flux_out * r.source_scale;
                r.after_step(h);
            }
        }

        let inv = 1.0 / substeps as f64;
        for b in 0..count {
            let r = &mut self.runners[b];
            let mut vf = self.valve_mass_flows[b] * inv;
            let mut pf = self.plenum_flows[b] * inv;
            if r.recover_if_broken() {
                vf = 0.0;
                pf = 0.0;
                self.mass[b] = r.total_mass();
            } else {
                self.mass[b] += (vf - pf) * dt;
            }
            self.valve_mass_flows[b] = vf;
            self.plenum_flows[b] = pf;

            let mass = if self.mass[b] > 1e-12 { self.mass[b] } else { 1e-12 };
            let rb = self.burned[b];
            let rf = self.fuel[b];
            let cb = cyl_state[b].burned;
            let cf = cyl_state[b].fuel;
            let mut burned = self.burned_mass[b]
                + ((if vf >= 0.0 { vf * cb } else { vf * rb })
                    - (if pf >= 0.0 { pf * rb } else { pf * plenum_burned }))
                    * dt;
            let air_through = if vf < 0.0 { -vf * (1.0 - rb - rf) } else { -vf * (1.0 - cb - cf) };
            let mut owed = self.air_owed[b] + air_through * dt;
            let mut injected = 0.0;
            if owed > 0.0 && vf < 0.0 {
                injected = owed * inject;
                owed = 0.0;
            }
            self.air_owed[b] = owed;
            let inflow = if vf < 0.0 { -vf * dt } else { 0.0 };
            let sprayed = if inflow > 0.0 { rf + injected / inflow } else { rf };
            self.inflow_fuel[b] = if sprayed > 1.0 - rb { 1.0 - rb } else { sprayed };
            let mut fuel = self.fuel_mass[b]
                + ((if vf >= 0.0 { vf * cf } else { vf * rf }) - (if pf >= 0.0 { pf * rf } else { pf * plenum_fuel }))
                    * dt;
            burned = if burned < 0.0 {
                0.0
            } else if burned > mass {
                mass
            } else {
                burned
            };
            fuel = if fuel < 0.0 {
                0.0
            } else if fuel > mass - burned {
                mass - burned
            } else {
                fuel
            };
            self.burned_mass[b] = burned;
            self.fuel_mass[b] = fuel;
            self.burned[b] = burned / mass;
            self.fuel[b] = fuel / mass;

            self.port_temps[b] = r.read_port().1;
            self.plenum_temps[b] = r.read_mouth().1;
        }
    }
}

/// How many times Kirchhoff's boundary-layer loss a runner's waves lose. His figure is for a small wave
/// in a smooth, straight tube of still air. At full throttle a runner carries large pulses on a
/// turbulent flow, round a bend into the port and past the valve seat. With this factor a wave in the
/// LT6's runners loses more than half its strength over one cycle at 8400 rpm, and dies out within a
/// few, as pressure measured in real runners does. With Kirchhoff's figure alone it takes five cycles
/// to lose as much, so a runner's resonance builds from one cycle to the next, and the full-throttle
/// torque swings by 5-8% between speeds a few hundred rpm apart.
const RUNNER_LOSS_FACTOR: f64 = 5.0;

/// Acoustic damping of a runner, 1/s: Kirchhoff's boundary-layer loss for a wide tube at `hz`, its
/// quarter-wave resonance, as `k = 2 c alpha`, scaled by `RUNNER_LOSS_FACTOR`.
fn runner_damping(radius: f64, hz: f64) -> f64 {
    const AIR_VISCOSITY: f64 = 1.82e-5;
    const AIR_PRANDTL: f64 = 0.71;
    let c = speed_of_sound(gas::T_AMB, gas::GAMMA_AIR);
    let nu = AIR_VISCOSITY / (gas::P_AMB / (gas::R * gas::T_AMB));
    let alpha = (math::sqrt((2.0 * PI * hz * nu) / 2.0) / (math::max(radius, 1e-3) * c))
        * (1.0 + (gas::GAMMA_AIR - 1.0) / math::sqrt(AIR_PRANDTL));
    RUNNER_LOSS_FACTOR * 2.0 * c * alpha
}
