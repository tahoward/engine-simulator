//! The intake runners: one duct per cylinder, from the plenum to the intake valve, solved with the
//! same gas dynamics as the exhaust.
//!
//! A runner is what lets an engine fill past what the plenum's pressure alone could push in: the
//! column of air it holds has momentum, and its pressure waves reflect off the plenum and arrive back
//! at the valve in step at some speeds and out of step at others. That is where an engine's torque
//! peak comes from.
//!
//! Each runner carries a port injector that meters fuel on the net fresh air its cylinder draws.
//! The solver carries no composition, so each runner's is carried alongside it, cell by cell, by the
//! same mass flows across the same faces: the spent gas and the fuel in each cell. What a cylinder
//! pushes back up its runner at overlap stays by the valve, mixed only as far as the flow carries it,
//! and it is that the cylinder draws back in first: only what is pushed further than the runner is
//! long reaches the plenum.

use crate::cylinder::CylState;
use crate::euler_pipe::{EulerPipe, EulerPipeOptions, InletKind, OutletKind, ValveState};
use crate::math::{self, PI};
use crate::plenum::{IntakePlenum, PlenumFeed};
use crate::spec::{EngineSpec, SegmentKind, SegmentPartial, gas, intake_runner_of, make_segment, speed_of_sound};

/// The injectors' setting for one `IntakeRunners::advance`.
#[derive(Clone, Copy, Debug, Default)]
pub struct RunnerIo {
    pub dt: f64,
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
    /// Spent-gas and fuel mass fractions of the runner's contents, all of it; of what it pushed out of its
    /// plenum end over the last sample; and of what the cylinder drew in through the valve.
    pub burned: f64,
    pub fuel: f64,
    pub mouth_burned: f64,
    pub mouth_fuel: f64,
    pub inflow_burned: f64,
    /// Gas temperature at the valve end, K.
    pub port_temp: f64,
    /// Fuel mass fraction of what the cylinder draws in through the valve.
    pub inflow_fuel: f64,
    /// The spent gas and the fuel in each cell, kg, valve end first, and each cell's fractions of them at
    /// the start of a substep.
    species: Vec<[f64; 2]>,
    fractions: Vec<[f64; 2]>,
    mass: f64,
    /// Air the cylinder has pushed back out and not yet drawn back in, kg, as a negative balance.
    air_owed: f64,
    /// What it draws from the plenum this sample, where it leaves it.
    feed: PlenumFeed,
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
            // The exhaust's material is the exhaust's alone.
            material: None,
            ..opts.clone()
        };
        let runners = (0..count)
            .map(|_| {
                let pipe = EulerPipe::new(&segment, sample_rate, spec.port_gas_temp, &runner_opts);
                let t = pipe.read_port().1;
                let cells = pipe.n;
                Runner {
                    mass: pipe.total_mass(),
                    port_temp: t,
                    plenum_temp: t,
                    pipe,
                    valve_mass_flow: 0.0,
                    plenum_flow: 0.0,
                    burned: 0.0,
                    fuel: 0.0,
                    mouth_burned: 0.0,
                    mouth_fuel: 0.0,
                    inflow_burned: 0.0,
                    inflow_fuel: 0.0,
                    species: vec![[0.0; 2]; cells],
                    fractions: vec![[0.0; 2]; cells],
                    air_owed: 0.0,
                    feed: PlenumFeed::default(),
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
            r.mass = r.pipe.total_mass();
            // Each cell the makeup of the source's cell at the same distance from the valve.
            let (n, src_n) = (r.pipe.n, s.pipe.n);
            for i in 0..n {
                let x = (i as f64 + 0.5) * r.pipe.dx;
                let j = ((x / s.pipe.dx) as usize).min(src_n - 1);
                let m = s.pipe.cell_mass(j);
                let [b, f] = s.species[j];
                let (yb, yf) = if m > 1e-15 { (b / m, f / m) } else { (0.0, 0.0) };
                let mi = r.pipe.cell_mass(i);
                r.species[i] = [mi * yb, mi * yf];
            }
            r.burned = s.burned;
            r.fuel = s.fuel;
            r.mouth_burned = s.mouth_burned;
            r.mouth_fuel = s.mouth_fuel;
            r.inflow_burned = s.inflow_burned;
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
            r.fill(burned, fuel);
            r.mouth_burned = burned;
            r.mouth_fuel = fuel;
            r.inflow_burned = burned;
        }
    }

    /// Advance every runner by one sample, in lockstep, each drawing from `plenum` where it leaves it.
    /// `entry_loss` is how far each runner's entry from the plenum loses more of the air it draws in
    /// than the others', as a share of that air's dynamic head; `cyl_state` is each cylinder's contents,
    /// from which what it pushes back up its runner takes its composition.
    pub fn advance(
        &mut self,
        io: &RunnerIo,
        plenum: &IntakePlenum,
        valves: &[ValveState],
        entry_loss: &[f64],
        cyl_state: &[CylState],
    ) {
        let count = self.runners.len();
        let step = self.begin(io, plenum, entry_loss);
        for b in 0..count {
            unsafe { step.runner(b, &valves[b], &cyl_state[b]) };
        }
    }

    /// `advance`, set up for its runners to be stepped one at a time, in any order or at once: each
    /// touches only its own `Runner`. The substep count is shared, so it is settled here.
    pub fn begin(&mut self, io: &RunnerIo, plenum: &IntakePlenum, entry_loss: &[f64]) -> IntakeStep<'_> {
        let dt = io.dt;
        let mut substeps = 1;
        for r in self.runners.iter_mut() {
            let s = r.pipe.substeps_for(dt);
            if s > substeps {
                substeps = s;
            }
        }

        // Each opens onto its own zone of the plenum. Drawing from it, the air speeds into the runner's end
        // and loses its share of the head that takes there, as it did over the last sample: the zone's
        // pressure less that, the air expanding to it as it goes.
        for (c, (r, &k)) in self.runners.iter_mut().zip(entry_loss).enumerate() {
            let feed = plenum.feed(c);
            let drawn = math::max(-r.plenum_flow, 0.0) / math::max(feed.rho * r.pipe.mouth_area(), 1e-12);
            let p = math::max(feed.p - k * 0.5 * feed.rho * drawn * drawn, 0.5 * feed.p);
            let rho = feed.rho * math::pow(p / feed.p, 1.0 / gas::GAMMA_EXH);
            r.pipe.set_reservoir(p, rho, math::sqrt((gas::GAMMA_EXH * p) / rho));
            r.feed = feed;
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
        let inject = self.io.inject;
        let run = unsafe { &mut *self.runners.add(b) };
        let plenum_in = [run.feed.burned, run.feed.fuel];
        let cyl_out = [cyl.burned, cyl.fuel];
        let r = &mut run.pipe;
        let (species, fractions) = (&mut run.species, &mut run.fractions);
        let n = r.n;
        let mut vf = 0.0;
        let mut pf = 0.0;
        // What crossed the valve into the cylinder, and the plenum end either way, over the sample: mass,
        // kg, and the spent gas and fuel in it.
        let mut drawn = [0.0; 3];
        let mut mouth = [0.0; 3];
        for _ in 0..substeps {
            r.begin_step(h);
            r.apply_own_boundaries(h);
            pf += r.mouth_mass_flow;
            r.compute_valve_flux(valve, h);
            let flow = r.valve_flux_out;
            r.set_end_step(h, flow);
            for (i, y) in fractions.iter_mut().enumerate() {
                let m = r.cell_mass(i);
                *y = if m > 1e-15 { [species[i][0] / m, species[i][1] / m] } else { [0.0; 2] };
            }
            r.end_step_set(valve);
            vf += r.valve_flux_out * r.source_scale;
            carry(r, species, fractions, h, cyl_out, plenum_in, &mut drawn, &mut mouth);
            r.after_step(h);
        }

        let inv = 1.0 / substeps as f64;
        vf *= inv;
        pf *= inv;
        if r.recover_if_broken() {
            vf = 0.0;
            pf = 0.0;
            run.mass = r.total_mass();
            let (b, f) = (run.burned, run.fuel);
            run.fill(b, f);
        } else {
            run.mass += (vf - pf) * dt;
        }
        run.valve_mass_flow = vf;
        run.plenum_flow = pf;

        // Each cell holds no more spent gas and fuel than gas.
        let r = &run.pipe;
        let (mut total, mut burned, mut fuel) = (0.0, 0.0, 0.0);
        for (i, s) in run.species.iter_mut().enumerate() {
            let m = r.cell_mass(i);
            s[0] = s[0].clamp(0.0, m);
            s[1] = s[1].clamp(0.0, m - s[0]);
            total += m;
            burned += s[0];
            fuel += s[1];
        }
        run.burned = if total > 1e-15 { burned / total } else { 0.0 };
        run.fuel = if total > 1e-15 { fuel / total } else { 0.0 };
        let at = |acc: [f64; 3], or: [f64; 2]| if acc[0] > 1e-15 { [acc[1] / acc[0], acc[2] / acc[0]] } else { or };
        let first = run.species[0];
        let m0 = r.cell_mass(0);
        let valve_end = if m0 > 1e-15 { [first[0] / m0, first[1] / m0] } else { [0.0; 2] };
        let [vb, vfu] = at(drawn, valve_end);
        let last = run.species[n - 1];
        let mn = r.cell_mass(n - 1);
        let mouth_end = if mn > 1e-15 { [last[0] / mn, last[1] / mn] } else { [0.0; 2] };
        [run.mouth_burned, run.mouth_fuel] = at(mouth, mouth_end);
        run.inflow_burned = vb;

        // The injector meters on the fresh air the cylinder draws, net of what it pushed back.
        let air_through = if vf < 0.0 { -vf * (1.0 - vb - vfu) } else { -vf * (1.0 - cyl.burned - cyl.fuel) };
        let mut owed = run.air_owed + air_through * dt;
        let mut injected = 0.0;
        if owed > 0.0 && vf < 0.0 {
            injected = owed * inject;
            owed = 0.0;
        }
        run.air_owed = owed;
        let inflow = if vf < 0.0 { -vf * dt } else { 0.0 };
        let sprayed = if inflow > 0.0 { vfu + injected / inflow } else { vfu };
        run.inflow_fuel = if sprayed > 1.0 - vb { 1.0 - vb } else { sprayed };

        let r = &run.pipe;
        run.port_temp = r.read_port().1;
        run.plenum_temp = r.read_mouth().1;
    }
}

impl Runner {
    /// Every cell of the given makeup.
    fn fill(&mut self, burned: f64, fuel: f64) {
        for (i, s) in self.species.iter_mut().enumerate() {
            let m = self.pipe.cell_mass(i);
            *s = [m * burned, m * fuel];
        }
        self.burned = burned;
        self.fuel = fuel;
    }
}

/// Carry the spent gas and fuel in `species` across every face of `r` by the substep's mass flows, `h` s
/// long: each face's from the cell upstream of it, its makeup reconstructed to the face with a limited
/// slope, as the solver reconstructs the gas. `fractions` are each cell's at the substep's start. What
/// comes in at the valve has the cylinder's makeup, `cyl`, and in at the plenum end the plenum's,
/// `plenum`; what crosses the valve into the cylinder, and the plenum end out into the plenum, is added
/// to `drawn` and `mouth`, as mass, spent gas and fuel, kg.
#[allow(clippy::too_many_arguments)]
fn carry(
    r: &EulerPipe,
    species: &mut [[f64; 2]],
    fractions: &[[f64; 2]],
    h: f64,
    cyl: [f64; 2],
    plenum: [f64; 2],
    drawn: &mut [f64; 3],
    mouth: &mut [f64; 3],
) {
    let n = r.n;
    // A cell's makeup at its face towards `towards` (+1 or -1): its own, sloped by the smaller of the
    // differences either side, none at a peak or at the duct's ends.
    let at_face = |i: usize, towards: f64, k: usize| {
        let y = fractions[i][k];
        if i == 0 || i + 1 >= n {
            return y;
        }
        let (back, ahead) = (y - fractions[i - 1][k], fractions[i + 1][k] - y);
        let slope = if back * ahead <= 0.0 { 0.0 } else if back.abs() < ahead.abs() { back } else { ahead };
        (y + 0.5 * towards * slope).clamp(0.0, 1.0)
    };
    for face in 1..n {
        let m = r.face_mass_flow(face) * h;
        let (from, towards) = if m >= 0.0 { (face - 1, 1.0) } else { (face, -1.0) };
        for k in 0..2 {
            let moved = m * at_face(from, towards, k);
            species[face - 1][k] -= moved;
            species[face][k] += moved;
        }
    }
    // The plenum end.
    let m = r.face_mass_flow(n) * h;
    let y = if m >= 0.0 { fractions[n - 1] } else { plenum };
    for k in 0..2 {
        species[n - 1][k] -= m * y[k];
    }
    if m > 0.0 {
        mouth[0] += m;
        mouth[1] += m * y[0];
        mouth[2] += m * y[1];
    }
    // The valve, into cell 0.
    let m = r.valve_source_flow() * h;
    if m >= 0.0 {
        species[0][0] += m * cyl[0];
        species[0][1] += m * cyl[1];
    } else {
        let y = fractions[0];
        species[0][0] += m * y[0];
        species[0][1] += m * y[1];
        drawn[0] -= m;
        drawn[1] -= m * y[0];
        drawn[2] -= m * y[1];
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
