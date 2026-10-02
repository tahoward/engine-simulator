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

/// One runner: its duct, what flowed through it over the last sample, and what it holds. On cache
/// lines of its own, as each runner may be stepped on a thread of its own.
#[repr(align(128))]
pub struct Runner {
    pub pipe: EulerPipe,
    /// Mass flow through the intake valve, kg/s, positive out of the cylinder into the runner.
    pub valve_mass_flow: f64,
    /// Mass flow out of the plenum end, kg/s, positive into the plenum.
    pub plenum_flow: f64,
    /// Gas temperature at the plenum end, K.
    pub plenum_temp: f64,
    /// Spent-gas and fuel mass fractions of the runner's contents.
    pub burned: f64,
    pub fuel: f64,
    /// Gas temperature at the valve end, K.
    pub port_temp: f64,
    /// Fuel mass fraction of what the cylinder draws in through the valve.
    pub inflow_fuel: f64,
    burned_mass: f64,
    fuel_mass: f64,
    mass: f64,
    /// Air the cylinder has pushed back out and not yet drawn back in, kg, as a negative balance.
    air_owed: f64,
}

pub struct IntakeRunners {
    pub runners: Vec<Runner>,
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
            // The runners' ramming is tuned against the plenum's air taken in at its own pressure.
            nozzle_inflow: Some(false),
            linear_damping: Some(damping),
            port: None,
            inherit_wall: None,
            ..opts.clone()
        };
        let runners = (0..count)
            .map(|_| {
                let pipe = EulerPipe::new(&segment, sample_rate, spec.port_gas_temp, &runner_opts);
                let t = pipe.read_port().1;
                Runner {
                    mass: pipe.total_mass(),
                    port_temp: t,
                    plenum_temp: t,
                    pipe,
                    valve_mass_flow: 0.0,
                    plenum_flow: 0.0,
                    burned: 0.0,
                    fuel: 0.0,
                    inflow_fuel: 0.0,
                    burned_mass: 0.0,
                    fuel_mass: 0.0,
                    air_owed: 0.0,
                }
            })
            .collect();
        IntakeRunners { runners }
    }

    /// Cells across all the runners.
    pub fn cells(&self) -> usize {
        self.runners.iter().map(|r| r.pipe.n).sum()
    }

    /// Solver recoveries across all the runners. Should stay zero.
    pub fn recoveries(&self) -> u64 {
        self.runners.iter().map(|r| r.pipe.recoveries).sum()
    }

    /// Take over from `src`, the other set of a two-stage intake, as the manifold switches between
    /// them: the gas, its composition and what each injector is owed.
    pub fn take_state_from(&mut self, src: &IntakeRunners) {
        for (r, s) in self.runners.iter_mut().zip(&src.runners) {
            r.pipe.resample_from(&s.pipe);
            let m = r.pipe.total_mass();
            r.mass = m;
            r.burned = s.burned;
            r.fuel = s.fuel;
            r.burned_mass = m * s.burned;
            r.fuel_mass = m * s.fuel;
            r.air_owed = s.air_owed;
            r.inflow_fuel = s.inflow_fuel;
            r.valve_mass_flow = s.valve_mass_flow;
            r.plenum_flow = s.plenum_flow;
            r.port_temp = r.pipe.read_port().1;
            r.plenum_temp = r.pipe.read_mouth().1;
        }
    }

    /// Prime every runner with a mixture, so the first cycles draw a charge.
    pub fn prime(&mut self, burned: f64, fuel: f64) {
        for r in self.runners.iter_mut() {
            let m = r.mass;
            r.burned_mass = m * burned;
            r.fuel_mass = m * fuel;
            r.burned = burned;
            r.fuel = fuel;
        }
    }

    /// Advance every runner by one sample, in lockstep. `breathing` is each cylinder's multiplier on
    /// the pressure its runner opens onto; `cyl_state` is each cylinder's contents, from which what it
    /// pushes back up its runner takes its composition.
    pub fn advance(&mut self, io: &RunnerIo, valves: &[ValveState], breathing: &[f64], cyl_state: &[CylState]) {
        let count = self.runners.len();
        let step = self.begin(io, breathing);
        for b in 0..count {
            unsafe { step.runner(b, &valves[b], &cyl_state[b]) };
        }
    }

    /// `advance`, set up for its runners to be stepped one at a time, in any order or at once: each
    /// touches only its own `Runner`. The substep count is shared, so it is settled here.
    pub fn begin(&mut self, io: &RunnerIo, breathing: &[f64]) -> IntakeStep<'_> {
        let dt = io.dt;
        let mut substeps = 1;
        for r in self.runners.iter_mut() {
            let s = r.pipe.substeps_for(dt);
            if s > substeps {
                substeps = s;
            }
        }

        // Breathing scales pressure and density together, so the plenum's sound speed is every runner's.
        let res_c = math::sqrt((gas::GAMMA_EXH * io.p) / io.rho);
        for (r, &bp) in self.runners.iter_mut().zip(breathing) {
            r.pipe.set_reservoir(io.p * bp, io.rho * bp, res_c);
        }
        IntakeStep { io: *io, substeps, runners: self.runners.as_mut_ptr(), _runners: std::marker::PhantomData }
    }
}

/// One sample of `IntakeRunners::advance`, from `IntakeRunners::begin`, each runner reached only
/// through its own index.
pub struct IntakeStep<'a> {
    io: RunnerIo,
    substeps: usize,
    runners: *mut Runner,
    _runners: std::marker::PhantomData<&'a mut IntakeRunners>,
}

// Each runner is reached only through `runner(b)`, which each index is given to once.
unsafe impl Sync for IntakeStep<'_> {}
unsafe impl Send for IntakeStep<'_> {}

impl IntakeStep<'_> {
    /// Step runner `b` through the sample, onto its intake `valve` and a cylinder holding `cyl`.
    ///
    /// # Safety
    ///
    /// Each `b` must be stepped once, by one thread: two threads on one runner would race.
    pub unsafe fn runner(&self, b: usize, valve: &ValveState, cyl: &CylState) {
        let dt = self.io.dt;
        let substeps = self.substeps;
        let h = dt / substeps as f64;
        let plenum_burned = self.io.burned;
        let plenum_fuel = self.io.fuel;
        let inject = self.io.inject;
        let run = unsafe { &mut *self.runners.add(b) };
        let r = &mut run.pipe;
        let mut vf = 0.0;
        let mut pf = 0.0;
        for _ in 0..substeps {
            r.begin_step(h);
            r.apply_own_boundaries(h);
            pf += r.mouth_mass_flow;
            r.compute_valve_flux(valve);
            let flow = r.valve_flux_out;
            r.set_end_step(h, flow);
            r.end_step_set(valve);
            vf += r.valve_flux_out * r.source_scale;
            r.after_step(h);
        }

        let inv = 1.0 / substeps as f64;
        vf *= inv;
        pf *= inv;
        if r.recover_if_broken() {
            vf = 0.0;
            pf = 0.0;
            run.mass = r.total_mass();
        } else {
            run.mass += (vf - pf) * dt;
        }
        run.valve_mass_flow = vf;
        run.plenum_flow = pf;

        let mass = if run.mass > 1e-12 { run.mass } else { 1e-12 };
        let rb = run.burned;
        let rf = run.fuel;
        let cb = cyl.burned;
        let cf = cyl.fuel;
        let mut burned = run.burned_mass
            + ((if vf >= 0.0 { vf * cb } else { vf * rb }) - (if pf >= 0.0 { pf * rb } else { pf * plenum_burned }))
                * dt;
        let air_through = if vf < 0.0 { -vf * (1.0 - rb - rf) } else { -vf * (1.0 - cb - cf) };
        let mut owed = run.air_owed + air_through * dt;
        let mut injected = 0.0;
        if owed > 0.0 && vf < 0.0 {
            injected = owed * inject;
            owed = 0.0;
        }
        run.air_owed = owed;
        let inflow = if vf < 0.0 { -vf * dt } else { 0.0 };
        let sprayed = if inflow > 0.0 { rf + injected / inflow } else { rf };
        run.inflow_fuel = if sprayed > 1.0 - rb { 1.0 - rb } else { sprayed };
        let mut fuel = run.fuel_mass
            + ((if vf >= 0.0 { vf * cf } else { vf * rf }) - (if pf >= 0.0 { pf * rf } else { pf * plenum_fuel })) * dt;
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
        run.burned_mass = burned;
        run.fuel_mass = fuel;
        run.burned = burned / mass;
        run.fuel = fuel / mass;

        run.port_temp = r.read_port().1;
        run.plenum_temp = r.read_mouth().1;
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
pub(crate) fn runner_damping(radius: f64, hz: f64) -> f64 {
    const AIR_VISCOSITY: f64 = 1.82e-5;
    const AIR_PRANDTL: f64 = 0.71;
    let c = speed_of_sound(gas::T_AMB, gas::GAMMA_AIR);
    let nu = AIR_VISCOSITY / (gas::P_AMB / (gas::R * gas::T_AMB));
    let alpha = (math::sqrt((2.0 * PI * hz * nu) / 2.0) / (math::max(radius, 1e-3) * c))
        * (1.0 + (gas::GAMMA_AIR - 1.0) / math::sqrt(AIR_PRANDTL));
    RUNNER_LOSS_FACTOR * 2.0 * c * alpha
}
