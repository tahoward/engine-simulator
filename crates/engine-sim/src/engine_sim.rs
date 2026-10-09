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

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

use crate::afterfire::Afterfire;
use crate::cylinder::{AdvanceIo, CrackleSpark, CylState, Cylinder, SpecInstance};
use crate::drivetrain::{LaunchPhase, LaunchRun};
use crate::dsp::{Delay, Impact, Noise, Resonator, soft_clip, wrap_cycle};
use crate::euler_pipe::{
    DEFAULT_CFL, DEFAULT_MAX_CELLS, EulerPipeOptions, HeadPort, ValveState, duct_cell_count, duct_grid_length,
    single_step_dx,
};
use crate::exhaust_graph::{ExhaustGraph, compile_exhaust, node_order, validate_graph};
use crate::exhaust_system::{ExhaustSystem, SideWork, Stepped};
use crate::inlet::{InletTract, airbox_volume_of, inlet_count_of, snorkel_dia_of};
use crate::intake::{IntakeRunners, RunnerIo};
use crate::listener::{Listener, SoundSources, SurfaceKind, Vec3, Walls};
use crate::room::{Reverb, RoomModes};
use crate::math::{self, PI, clamp};
use crate::plenum::{IntakePlenum, throttle_dia_of};
use crate::pool::{CachePadded, Disjoint, ThreadPool};
use crate::radiation::{FarField, MouthJet, Steepening};
use crate::shell::ChamberShell;
use crate::spec::{
    BankSnapshot, CV_REF, CV_SLOPE, CrankType, EngineConfig, EngineSnapshot, EngineSpec, ExhaustLayout,
    FUEL_CUT_THROTTLE, Fuel, LaunchConfig, LaunchSnapshot, PIPE_PRESSURE_TAPS, PipeSegment, REV_LIMIT_HYSTERESIS_RPM,
    RunnerSize, T_REF, TurboSnapshot, TurboUnitSnapshot, ambient_sound_speed, crank_pins, cylinder_spacing, density,
    displacement, exhaust_layout_of, exhaust_port_diameter, firing_plan, fuel_cut_rpms, fuel_fraction_at,
    full_load_torque, gas, intake_runner_of, load_torque_of, physical_bank, physical_bank_count,
};
use crate::turbo::{Turbo, TurboOut};
use crate::valve::{valve_flow_area, valve_lift};

/// Pressure, Pa, that maps to digital full scale.
const PA_PER_FULLSCALE: f64 = 300.0;

/// Fastest a cam phaser turns, crank degrees per second.
const PHASER_RATE: f64 = 250.0;

/// How far below its switch speed cam profile switching drops back to the low-speed lobes, rev/min.
pub const CAM_SWITCH_HYSTERESIS: f64 = 150.0;

/// How far below its switch speed a two-stage intake switches back to its long runners, rev/min.
const INTAKE_SWITCH_HYSTERESIS: f64 = 150.0;

/// Longest a finished launch waits for the engine to wind down to a held speed, s.
const LAUNCH_WIND_DOWN: f64 = 6.0;

/// Crank degrees per cylinder sub-step.
const MAX_DEG_PER_SUBSTEP: f64 = 0.35;

/// Largest fraction of the trapped mass that may cross the valves in one cylinder sub-step.
const MAX_MASS_FRACTION_PER_SUBSTEP: f64 = 0.05;

/// Ceiling on cylinder sub-steps.
const MAX_CYL_SUBSTEPS: f64 = 64.0;

/// Below this speed, rev/min, switching the ignition on starts the engine at its starting speed rather
/// than catching it as it turns.
const CATCH_RPM: f64 = 450.0;

/// The idle air valve: the opening it starts from, as more of the throttle plate's, about what an engine
/// needs to idle at `IDLE_VALVE_START_RPM` (`idle_valve_start`), and the most it opens.
const IDLE_VALVE_START: f64 = 0.075;
const IDLE_VALVE_START_RPM: f64 = 800.0;
const IDLE_VALVE_MAX: f64 = 0.25;
/// Its controller's gains on the speed error as a share of the idle speed: proportional, and
/// integral, /s.
const IDLE_KP: f64 = 0.02;
const IDLE_KI: f64 = 0.04;
/// Above this multiple of the idle speed the controller keeps the opening it has learned rather than
/// winding it shut, so an engine coming down from high revs is caught at the idle speed, not stalled.
/// Once the engine has sat there with the throttle shut for `IDLE_SETTLE_TIME`, s, its speed smoothed
/// over `IDLE_TREND_TAU`, s, falling no faster than `IDLE_SETTLED_RATE` idle speeds per second, it
/// winds the opening down again, as slowly as it would at the hold speed, so an opening learned against a load that has gone, or in a dip,
/// cannot hold the engine above its idle. Time spent falling faster counts back down, so a lumpy
/// idle's surges delay the unwinding rather than preventing it.
const IDLE_HOLD_ABOVE: f64 = 1.25;
const IDLE_SETTLED_RATE: f64 = 0.1;
const IDLE_TREND_TAU: f64 = 1.0;
const IDLE_SETTLE_TIME: f64 = 2.0;
/// The dashpot: how much further the valve opens, as more of the plate's, for each idle speed per
/// second the engine is falling at, s, so an engine dropping off a lift has the air to catch itself at
/// the idle rather than falling through it; and as much less for each it is rising at, so the air the
/// plenum takes its time to pass on does not carry the idle up past its speed and back, round and
/// round. And the time constant the rate is smoothed over, s.
const IDLE_DASHPOT: f64 = 0.04;
const IDLE_RATE_TAU: f64 = 0.05;

/// A diesel's idle governor, which meters fuel where a petrol engine's idle air valve meters air: the
/// fuel it starts from and the most it gives, as shares of the full delivery, and its gains on the
/// speed error as a share of the idle speed, proportional and integral, /s, and on how fast the engine
/// is falling, in idle speeds per second, s.
const IDLE_FUEL_START: f64 = 0.06;
const IDLE_FUEL_MAX: f64 = 0.35;
const IDLE_FUEL_KP: f64 = 0.6;
const IDLE_FUEL_KI: f64 = 0.3;
const IDLE_FUEL_DASHPOT: f64 = 0.05;

/// How far below a diesel's governed speed its governor starts taking the fuel away, rev/min: none is
/// left at the governed speed itself.
const GOVERNOR_DROOP: f64 = 300.0;

/// The overrun crackle map: the speed above which a lift starts it, rev/min, and below which it
/// stops; the longest it runs after a lift, s; how late it fires the spark, degrees after top dead
/// centre, and the share of cycles it skips the spark on, at its least and most intense.
const CRACKLE_RPM: f64 = 2500.0;
const CRACKLE_END_RPM: f64 = 2000.0;
const CRACKLE_WINDOW: f64 = 3.0;
const CRACKLE_ATDC_MIN: f64 = 15.0;
const CRACKLE_ATDC_MAX: f64 = 45.0;
const CRACKLE_SKIP_MIN: f64 = 0.1;
const CRACKLE_SKIP_MAX: f64 = 0.35;
/// How far the map holds the throttle open, at its least and most intense, to feed the charge it
/// burns late: as a drive-by-wire throttle is cracked open for it.
const CRACKLE_THROTTLE_MIN: f64 = 0.05;
const CRACKLE_THROTTLE_MAX: f64 = 0.15;

/// A valve's knock on its seat is an impact, and its sound goes as the momentum it brings: the mass that
/// lands times how fast it lands. A cam sets its valves down on their seats on a closing ramp, at a speed
/// it is ground for, `CLOSING_RAMP_M_PER_DEG` m for each degree the crank turns, so they land faster in
/// proportion to the engine's speed; and it lifts them off theirs on an opening ramp the same, as the lash
/// between the follower and the valve closes.
///
/// A valve weighs as `VALVE_MASS_PER_D3` times the cube of its head's diameter, kg: steel, its head and
/// stem growing together, about 43 g for a 34 mm head and 107 g for a 46 mm one. What lands with it on
/// its seat is `SEATING_MASS_SHARE` of that, its retainer, keepers and the moving part of its spring with
/// it. What closes its lash on opening is a rocker and a pushrod, `LASH_MASS_PUSHROD` of the valve, or a
/// bucket over it, `LASH_MASS_BUCKET`.
///
/// What that momentum sounds like at 1 m, Pa per N s, at `mech_noise = 1`, is the one figure set by ear:
/// so the default single's 34 mm exhaust valve knocks at 6 Pa at 3000 rev/min.
const CLOSING_RAMP_M_PER_DEG: f64 = 9e-6;
const VALVE_MASS_PER_D3: f64 = 1100.0;
const SEATING_MASS_SHARE: f64 = 1.5;
const LASH_MASS_PUSHROD: f64 = 1.5;
const LASH_MASS_BUCKET: f64 = 0.5;
const CLACK_PA_PER_NS: f64 = 571.0;

/// Peak structure-borne levels at 1 m, Pa, at `mech_noise = 1`.
const SLAP_PA_AT_1M: f64 = 3.5;
/// And a rod's bearings knocking across their oil clearance, as the force down the rod changes sign
/// at `KNOCK_RATE_REF`, N/s.
const KNOCK_PA_AT_1M: f64 = 2.4;
const KNOCK_RATE_REF: f64 = 2e6;

/// The running noise at 1 m, Pa, at `mech_noise = 1`, before the modes it rings: a piston and its rings
/// rubbing the bore at `SCUFF_SPEED_REF` under `SCUFF_LOAD_REF`; a cam follower carrying a valve at full
/// lift at 3000 rev/min; the timing drive at `TIMING_REF_RPM`, louder as the speed to the 1.5; and the crank's twist per N m it carries, on an engine
/// the reference engine's size.
const SCUFF_PA_AT_1M: f64 = 0.16;
const FOLLOWER_PA_AT_1M: f64 = 0.12;
const TIMING_PA_AT_1M: f64 = 0.15;
const TWIST_PA_PER_NM: f64 = 0.0005;
const SCUFF_SPEED_REF: f64 = 10.0;
const SCUFF_LOAD_REF: f64 = 1000.0;
const TIMING_REF_RPM: f64 = 3000.0;

/// What presses a piston's rings on the bore besides the piston's own side thrust: their tension, N,
/// and the share of the crown's gas load that gets behind the top ring, the ring's face over the bore's
/// area.
const RING_TENSION_N: f64 = 120.0;
const RING_GAS_SHARE: f64 = 0.05;

/// A valve spring's load on its follower as the valve leaves its seat, as a share of its load at full
/// lift.
const SPRING_PRELOAD_SHARE: f64 = 0.4;

/// Teeth on the crank's timing sprocket: the timing drive meshes this many times a turn. And the second
/// harmonic of the mesh, as a share of the first.
const TIMING_TEETH: f64 = 21.0;
const TIMING_SECOND_HARMONIC: f64 = 0.35;
/// How far the timing drive's tension wavers with its chain's slack, RMS as a share of its level, and
/// how fast, Hz.
const TIMING_RATTLE: f64 = 0.5;
const TIMING_RATTLE_HZ: f64 = 40.0;

/// RMS turbulent fluctuation of the plane-wave volume velocity, as a fraction of the mean valve flow.
const TURBULENCE_INTENSITY: f64 = 0.1;

/// Where a mouth the sources do not place goes: in a row across the car, this far apart, this far
/// behind the crank's middle, m.
const UNPLACED_MOUTH_SPACING: f64 = 0.4;
const UNPLACED_MOUTH_REAR: f64 = 1.0;

/// Where the intake draws its air when the sources do not say, m: above the front of the engine.
const UNPLACED_INTAKE: Vec3 = [0.0, 0.3, -0.4];

/// Where the ear is when no listener is given: this far from the middle of the mouths, at 45 degrees
/// off the car's rear axis, and this high above the ground, m.
const DEFAULT_EAR_DISTANCE: f64 = 1.5;
const DEFAULT_EAR_HEIGHT: f64 = 1.2;

/// Closest a room lets the ear come to its walls and ceiling, m.
const EAR_MARGIN: f64 = 0.2;

/// The engine the structure-borne frequencies were set against: the default single.
const REFERENCE_DISPLACEMENT_M3: f64 = 4.977e-4;
const REFERENCE_BORE: f64 = 0.089;
const REFERENCE_EX_VALVE: f64 = 0.034;

/// Valve-seating ring and piston-slap ring on the reference engine, [Hz, Q].
const CLACK_MODE: (f64, f64) = (2700.0, 14.0);
const SLAP_MODE: (f64, f64) = (620.0, 9.0);

/// What the running noise rings on the reference engine, [Hz, Q]: the bore's liner under the rings'
/// rub, the head under the followers, and the crankcase under a bearing's knock. Broad, as rubbing
/// excites a band rather than a ring.
const SCUFF_MODE: (f64, f64) = (1500.0, 2.0);
const FOLLOWER_MODE: (f64, f64) = (1900.0, 3.0);
const KNOCK_MODE: (f64, f64) = (1100.0, 8.0);

/// The crank's first torsional mode with one throw, Hz, and its Q, its damper's: a longer crank twists
/// lower, as the square root of its throws.
const TWIST_MODE: (f64, f64) = (900.0, 12.0);

/// How far each cylinder's own head and bore ring from the nominal, as a fraction, peak.
const LOCAL_MODE_DETUNE: f64 = 0.06;

/// The idle air valve's opening to start from, as more of the throttle plate's: `IDLE_VALVE_START` at an
/// idle of `IDLE_VALVE_START_RPM` or below, and more for a higher one, with the square root of the idle
/// speed. An engine idling
/// higher takes more air; started with too little, it falls through its idle while the controller learns
/// the rest, and with too much it hangs above it.
fn idle_valve_start(spec: &EngineSpec) -> f64 {
    if spec.fuel == Fuel::Diesel {
        return IDLE_FUEL_START;
    }
    IDLE_VALVE_START * math::sqrt(math::max(spec.idle_rpm, IDLE_VALVE_START_RPM) / IDLE_VALVE_START_RPM)
}

/// The frequency a cylinder `bore` m across rings at, Hz, on the chamber mode with Draper's number
/// `alpha`, its gas at `temp` K: `c alpha / (pi bore)`. See `CHAMBER_MODES`.
pub fn chamber_mode_hz(alpha: f64, temp: f64, bore: f64) -> f64 {
    let c = math::sqrt(gas::GAMMA_EXH * gas::R * math::max(temp, 1.0));
    (c * alpha) / (PI * math::max(bore, 1e-3))
}

/// The `i`th of a diesel's block modes on the reference engine, Hz.
fn diesel_block_hz(i: usize) -> f64 {
    let step = math::log(DIESEL_BLOCK_HIGH_HZ / DIESEL_BLOCK_LOW_HZ) / (DIESEL_BLOCK_COUNT - 1) as f64;
    DIESEL_BLOCK_LOW_HZ * math::exp(step * i as f64)
}

/// How much of a diesel's combustion its block passes at `hz` on the reference engine, dB.
fn diesel_block_gain_db(hz: f64) -> f64 {
    let g = &DIESEL_BLOCK_GAIN_DB;
    if hz <= g[0].0 {
        return g[0].1;
    }
    for w in g.windows(2) {
        let ((f0, d0), (f1, d1)) = (w[0], w[1]);
        if hz <= f1 {
            return d0 + (d1 - d0) * (math::log(hz / f0) / math::log(f1 / f0));
        }
    }
    g[g.len() - 1].1
}

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

/// How far each runner's entry from the plenum loses more or less of the air it draws in than the
/// others' do, at `cylinder_spread = 1`, as a share of the air's dynamic head there, peak: no two runners
/// are cast or flared alike, and the loss of a well-radiused entry is a few hundredths of the head where
/// a sharp-edged one's is half of it. It is on top of the loss the runners' ramming is tuned with, so it
/// is as much less on some as more on others.
const ENTRY_LOSS_SPREAD: f64 = 0.3;
/// Per-cylinder valve-timing spread at `cylinder_spread = 1`, crank degrees peak.
const CAM_SPREAD_DEG: f64 = 2.2;
/// A diesel's pump elements and injectors are never alike: at `cylinder_spread = 1` each delivers up to
/// `DELIVERY_SPREAD` more or less fuel than the mean, and starts its injection up to
/// `INJECTION_SPREAD_DEG` crank degrees earlier or later. A pump is calibrated to a few percent and a
/// fraction of a degree at full load, and drifts further apart at idle, where its deliveries are
/// smallest; the cylinders' differences are what the engine's half orders are made of.
const DELIVERY_SPREAD: f64 = 0.7;
const INJECTION_SPREAD_DEG: f64 = 7.0;

/// What the inlet tracts are built from: how many, the throttles' bore, the airbox and the snorkel.
fn inlet_key(spec: &EngineSpec) -> [f64; 5] {
    [
        inlet_count_of(spec) as f64,
        throttle_dia_of(spec),
        airbox_volume_of(spec),
        spec.snorkel_length,
        snorkel_dia_of(spec),
    ]
}

/// The inlet tracts for `spec`, one for each throttle body.
fn build_inlets(spec: &EngineSpec, sample_rate: f64, opts: &EulerPipeOptions) -> Vec<InletTract> {
    (0..inlet_count_of(spec)).map(|k| InletTract::nth(spec, sample_rate, opts, k)).collect()
}

/// The sources heard after the mouths, in the order of their paths to the ear.
#[derive(Clone, Copy)]
enum Source {
    Shells,
    Intake,
    Casing,
    Turbo,
    /// Dual plenums' other snorkel's mouth, heard only where there is one.
    SecondIntake,
}

/// Time constant, s, of the mean-torque tracker and of the rpm readout's smoothing.
const IRREGULARITY_TAU: f64 = 0.12;

/// The solver's cost budget, and what a cylinder and a junction cost, all in pipe cells: what keeps an
/// engine in real time at 48 kHz on one thread of a browser's audio worklet.
const CYLINDER_COST_IN_CELLS: f64 = 102.0;
const JUNCTION_COST_IN_CELLS: f64 = 27.0;
const SOLVER_COST_BUDGET: f64 = 1216.0;

/// Cells of pipe the budget leaves for an engine with this many cylinders and junctions.
pub fn grid_budget_cells(cylinders: usize, junctions: usize) -> f64 {
    scaled_budget_cells(cylinders, junctions, 1.0)
}

/// `grid_budget_cells` with the whole budget `scale` times as large, for a machine that can afford it.
pub fn scaled_budget_cells(cylinders: usize, junctions: usize, scale: f64) -> f64 {
    SOLVER_COST_BUDGET * scale - CYLINDER_COST_IN_CELLS * cylinders as f64 - JUNCTION_COST_IN_CELLS * junctions as f64
}

/// A diesel's block as its combustion shakes it: `DIESEL_BLOCK_COUNT` modes log-spaced from
/// `DIESEL_BLOCK_LOW_HZ` to `DIESEL_BLOCK_HIGH_HZ` on the reference engine, each of Q `DIESEL_BLOCK_Q`,
/// denser than `STRUCTURAL_MODES`. A diesel's pressure rise is ten times a petrol engine's, hard enough
/// to hear how many modes a block has: four would ring as four tones, where a real block's many
/// overlap into one broad ring that carries on between firings.
const DIESEL_BLOCK_COUNT: usize = 168;
const DIESEL_BLOCK_LOW_HZ: f64 = 280.0;
const DIESEL_BLOCK_HIGH_HZ: f64 = 16800.0;
const DIESEL_BLOCK_Q: f64 = 50.0;
/// How much of the combustion each passes, dB, at frequencies on the reference engine, in between
/// by straight lines on a log scale: little at the bottom, where a stiff block radiates poorly, and
/// most around 1.5-3 kHz, where its walls are most mobile. Its structure attenuation, the other way
/// up.
const DIESEL_BLOCK_GAIN_DB: [(f64, f64); 7] =
    [(280.0, -24.0), (560.0, -4.0), (1000.0, 0.0), (1900.0, 0.0), (4200.0, 0.0), (8000.0, -4.0), (16800.0, -10.0)];
/// Below this, on the reference engine, a diesel's block takes its pressure rise ever less to heart,
/// Hz: two poles of it. A stiff casting barely radiates the slow swing of compression and expansion,
/// which is far the largest part of the rise, and the most regular: set so the 6CT's clatter stands as
/// far over its low rumble as a 6CTA's does, idling, heard beside it. 0 for none.
const DIESEL_BLOCK_HIGHPASS_HZ: f64 = 300.0;

/// Over how long after a blow a diesel's block modes are struck, s. A knock is not felt all through the
/// casting at once: bending waves carry it at a few hundred metres a second, slower the lower they are,
/// and the walls that radiate it lie at their own distances from each cylinder. Struck all at once, its
/// many modes would add to one hard spike, far peakier than the clatter of a real block.
const DIESEL_BLOCK_SPREAD_S: f64 = 0.003;

/// How hard a piston's slap drives a diesel's block, against ringing its own mode as on a petrol
/// engine: its knock spreads through the block's modes, as the combustion's does.
const DIESEL_SLAP_DRIVE: f64 = 2.0;

/// Each diesel block mode's share of the drive.
const DIESEL_BLOCK_GAIN: f64 = 0.1;
/// A diesel's combustion pressure-rise drive bandwidth and structure-borne band limit, Hz: its burn
/// is abrupt enough to drive the block well above a petrol engine's.
const DIESEL_DPDT_BANDWIDTH_HZ: f64 = 6000.0;
const DIESEL_STRUCTURE_LIMIT_HZ: f64 = 16000.0;

/// A diesel's chamber ringing. Its premixed burn lights all at once, unevenly, and sets the gas in
/// the chamber ringing at its own acoustic modes: `c alpha / (pi bore)`, with Draper's numbers for the
/// first and second circumferential modes and the first radial, each with its share of the ring.
const CHAMBER_MODES: [(f64, f64); 3] = [(1.841, 1.0), (3.054, 0.5), (3.832, 0.35)];
/// How quickly the ring dies, as the Q of each mode, and the burst that sets it off, s.
const CHAMBER_Q: f64 = 25.0;
const CHAMBER_BURST_S: f64 = 0.0001;
/// How hard the ring swings the cylinder's pressure, as a share of the rise its premixed burn makes:
/// lit in pockets across the bowl, not evenly, the burn rings the chamber's modes by about a bar at
/// idle, as direct-injection diesels are measured to. And how far it varies from one cycle to the next, as a share either way: no two
/// cylinders light alike. The ring is part of the cylinder's pressure, and shakes the block with the
/// rest of it.
const CHAMBER_SHARE: f64 = 0.15;
const CHAMBER_SCATTER: f64 = 0.35;

/// A diesel's injector needle lifting off its seat as the injection starts and slamming back as it
/// ends: its level at 1 m, Pa, at `mech_noise = 1`, the lift's share of it, the injector body's ring
/// [Hz, Q], and the impact's contact time, s.
const NEEDLE_PA_AT_1M: f64 = 4.0;
const NEEDLE_LIFT_SHARE: f64 = 0.4;
const NEEDLE_MODE: (f64, f64) = (4200.0, 12.0);
const NEEDLE_CONTACT_S: f64 = 0.00015;

/// The thin stamped panels of the casing, which ring at their own few low modes as what they are
/// bolted to shakes them: the oil pan's and the cam cover's, [Hz, Q] on a casing `PANEL_REF_LENGTH` m
/// long, lower in proportion on a longer one; and how much each adds to what its surface carries at its
/// own frequency, where it rings most, as a share of what the surface carries there. The block's sides and
/// its front cover are stiff castings, and ring only as the block does.
const OIL_PAN_MODES: [(f64, f64); 3] = [(300.0, 8.0), (650.0, 8.0), (1300.0, 8.0)];
const COVER_MODES: [(f64, f64); 3] = [(450.0, 8.0), (950.0, 8.0), (1800.0, 8.0)];
const PANEL_REF_LENGTH: f64 = 0.5;
const PANEL_SHARE: f64 = 1.0;
/// The share of the block's combustion ring, by power, that its sides radiate: the oil pan, bolted
/// under it, radiates the rest.
const BLOCK_SIDE_SHARE: f64 = 0.75;

/// Bandwidth of the combustion pressure-rise drive, Hz, and the band limit on everything
/// structure-borne, Hz.
const DPDT_BANDWIDTH_HZ: f64 = 1500.0;
const STRUCTURE_LIMIT_HZ: f64 = 6000.0;

/// Contact durations of the impacts, s: a valve on its seat, a piston on its bore, and a rod's bearing
/// across its oil film, which cushions it.
const VALVE_CONTACT_S: f64 = 0.00015;
const SLAP_CONTACT_S: f64 = 0.0004;
const KNOCK_CONTACT_S: f64 = 0.0006;

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
    steepening: Vec<Steepening>,
    jets: Vec<MouthJet>,
    /// Every chamber's shell, ringing with the gas inside it.
    shells: Vec<ChamberShell>,
    /// Each cylinder's valves and what they did this sample.
    banks: Vec<Bank>,
    /// Each cylinder's throat turbulence, kept across rebuilds.
    throat_noise: Vec<CachePadded<Noise>>,
    clack: Vec<Resonator>,
    /// What lands on each seat and closes each lash, kg, every valve of a kind together.
    valve_masses: ValveMasses,
    slap: Vec<Resonator>,
    clack_impact: Vec<Impact>,
    slap_impact: Vec<Impact>,
    head_share: f64,
    /// Piston slaps triggered since construction.
    pub slap_count: u64,
    /// Each cylinder's running noise: its rings rubbing the bore, its cam followers, its rod's bearings,
    /// and the noise the rubbing is drawn from.
    scuff: Vec<Resonator>,
    follower: Vec<Resonator>,
    knock: Vec<Resonator>,
    knock_impact: Vec<Impact>,
    rub_noise: Vec<Noise>,
    /// Bearing knocks triggered since construction.
    pub knock_count: u64,
    /// The timing drive: where its mesh is in a tooth, 0..1; its rattle, drawn from its own noise and
    /// scaled back to that noise's spread by `timing_rattle_gain`; and how many heads' drives it is.
    timing_mesh: f64,
    timing_rattle: f64,
    timing_rattle_c: f64,
    timing_rattle_gain: f64,
    timing_noise: Noise,
    timing_share: f64,
    /// The crank's first torsional mode, and how much it twists per N m against the reference engine's
    /// crank, stiffer as the engine is bigger.
    twist: Resonator,
    twist_share: f64,
    structure: Vec<Resonator>,
    /// A diesel's block, its chamber rings, each three modes, and its injector needles' ticks, each
    /// cylinder's own: see `DIESEL_BLOCK_MODES`, `CHAMBER_MODES` and `NEEDLE_MODE`.
    diesel_block: Vec<Resonator>,
    diesel_block_gain: Vec<f64>,
    /// What drives a diesel's block, kept a while, and how long after it each of its modes is struck,
    /// samples: see `DIESEL_BLOCK_SPREAD_S`.
    diesel_drive: Delay,
    diesel_block_lag: Vec<f64>,
    chamber: Vec<[Resonator; 3]>,
    chamber_burst: Vec<Impact>,
    chamber_last: Vec<f64>,
    needle: Vec<Resonator>,
    needle_impact: Vec<Impact>,
    dpdt_smooth: f64,
    dpdt_smooth_c: f64,
    diesel_dpdt_c: f64,
    /// The high-pass on a diesel's block drive: its coefficient and its two stages' states.
    diesel_hp_c: f64,
    diesel_hp: [f64; 2],
    structure_lp_c: f64,
    diesel_structure_lp_c: f64,
    structure_lp1: f64,
    structure_lp2: f64,
    /// The casing's surfaces, when the sources place them: what each is, its panel modes, its band
    /// limit's state, what it carries this sample and its path's index. And where each part's sound
    /// goes: each cylinder's to its bank's sides and head, and its chamber ring to both; the block's to
    /// the sides and the pan; the bottom end's to the pan; the timing drive's to the front cover. Each
    /// route a list of surfaces and the share of the sound each takes.
    surfaces: Vec<Surface>,
    surface_pa: Vec<f64>,
    surface_path: usize,
    side_route: Vec<Vec<(usize, f64)>>,
    head_route: Vec<Vec<(usize, f64)>>,
    block_route: Vec<(usize, f64)>,
    pan_route: Vec<(usize, f64)>,
    front_route: Vec<(usize, f64)>,
    /// Every source's path to the ear: the mouths in order, then the muffler shells, the intake, the
    /// casing and the turbos (`Source`).
    listener: Listener,
    /// The room's reverberation, fed by every source; and below it, its modes, each source driving them
    /// where it stands, and what each gave its path this sample.
    reverb: Reverb,
    room_modes: RoomModes,
    mode_sources: Vec<f64>,
    sources: SoundSources,
    ear: Option<Vec3>,
    /// The way the listener's right is, m, in the sources' frame.
    right: Option<Vec3>,
    /// Two ears, a head apart, or one.
    stereo: bool,
    /// The air's way in to each throttle, and its snorkel's mouth, on an engine without a turbo.
    inlets: Vec<InletTract>,
    intake_far_fields: [FarField; 2],
    entry_loss: Vec<f64>,
    timing: Vec<f64>,
    /// A diesel's each cylinder's fuel delivery as a share of the mean, and its injection's offset, deg.
    delivery: Vec<f64>,
    injection_offset: Vec<f64>,
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
    /// CFL substeps the gas solver took last sample.
    pub substeps: usize,
    omega: f64,
    omega_mean: f64,
    omega_ripple: f64,
    torque_avg: f64,
    omega_display: f64,
    limiter_cut: bool,
    fuel_cut_active: bool,
    /// Whether the spark and the fuel are on. Off, the engine coasts to a stop on its own friction and
    /// pumping, and the pipes ring down with it.
    ignition: bool,
    /// The idle air valve controller's learned opening: its integral term. A diesel's idle governor
    /// learns its fuel here instead.
    idle_learned: f64,
    /// A diesel's idle governor's fuel this sample, and the fuel its cylinders are given, as shares of
    /// the full delivery, 0..1.
    idle_fuel: f64,
    fuel_demand: f64,
    /// The crank speed it last saw, rev/min, and how fast that is changing, rev/min per s, smoothed.
    idle_last_rpm: f64,
    idle_rate: f64,
    /// The speed's rate of change smoothed over `IDLE_TREND_TAU`, rpm/s, and how long the engine has
    /// sat above the hold with the throttle shut, not falling, s.
    idle_trend: f64,
    idle_settled_for: f64,
    /// The overrun crackle map: running, for how long since the lift, s, and done for this lift.
    crackle_active: bool,
    crackle_time: f64,
    crackle_spent: bool,
    /// Whether the plenum's throttle is held at the crackle map's opening.
    crackle_opened: bool,
    afterfire: Afterfire,
    /// Afterfires counted at the last snapshot.
    afterfires_seen: u64,
    launch: Option<LaunchRun>,
    launch_opening: f64,
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
    /// How many times `SOLVER_COST_BUDGET` the solver may spend: 1 is what a browser can afford.
    budget_scale: f64,
    /// Simulated samples per output sample: 1 is real time, less is slow motion.
    time_scale: f64,
    /// In slow motion, how far the output is from the last simulated sample to the next, 0..1.
    slow_phase: f64,
    slow_prev: [f64; 2],
    slow_next: [f64; 2],
    /// Threads the exhaust's ducts and the intake runners are stepped across; `None` for this thread
    /// alone.
    pool: Option<Arc<ThreadPool>>,
    /// The most of the pool's threads to use.
    max_threads: usize,
    /// Each intake runner's cells, as the exhaust is told them to balance its threads.
    runner_cells: Vec<usize>,
    /// Each primary's port pressure, Pa, as the valves read it.
    port_pressure: Vec<f64>,
    /// What `step_sample` hands each thread once the ducts are stepped, and what it gives back: per cylinder, per mouth, per
    /// shell, and the turbo's.
    close_groups: Vec<Vec<CloseItem>>,
    /// What `close_groups` were dealt out for: the exhaust's threads, by `groups_id`, and how many
    /// cylinders, mouths, shells and inlet tracts there were, and whether there was a turbo.
    close_key: (u64, usize, usize, usize, usize, bool),
    bank_out: Vec<CachePadded<BankOut>>,
    mouth_out: Vec<CachePadded<[f64; 2]>>,
    shell_out: Vec<CachePadded<f64>>,
    turbo_out: CachePadded<Option<TurboOut>>,
    /// What the plenum's step left this sample, and the sample it was left for, by `close_stamp`.
    plenum_out: CachePadded<PlenumOut>,
    plenum_done: CachePadded<AtomicU64>,
    close_stamp: u64,
    /// What each of those parts took in a timed sample, ns: cylinders, mouths, shells.
    close_ns: Vec<CachePadded<f64>>,
}

/// One cylinder's valves: what they did this sample, and what the next needs of the last. On cache
/// lines of its own, as each cylinder's are stepped on its exhaust primary's thread.
#[repr(align(128))]
struct Bank {
    cyl_state: CylState,
    valve_state: ValveState,
    in_valve: ValveState,
    ex_lift: f64,
    in_lift: f64,
    seating_now: bool,
    in_seating_now: bool,
    /// Whether each valve left its seat this sample.
    opening_now: bool,
    in_opening_now: bool,
    /// Cylinder pressure at a top dead centre crossed this sample, Pa, or -1.
    tdc_pressure: f64,
    /// What the cam followers carry: each open valve's spring load, as a share of a valve's at full
    /// lift.
    follower_load: f64,
    /// The piston's speed, m/s, and the load pressing it and its rings on the bore, N.
    piston_speed: f64,
    bore_load: f64,
    /// The force down the rod, N, positive in compression, and how fast it was changing where it
    /// changed sign this sample, N/s, or -1.
    rod_force: f64,
    rod_reversal: f64,
    /// On a diesel: the pressure its premixed burn raises as it lights this sample, Pa, or 0, and the gas
    /// temperature that rings its chamber at, K; and its injector needle lifting (1) or seating (2)
    /// this sample, or 0.
    ring_rise: f64,
    ring_temp: f64,
    needle: u8,
    seat_pulse: Impact,
    /// The throat turbulence's two filter stages.
    turb1: f64,
    turb2: f64,
    last_valve_mdot: f64,
    prev_ex_lift: f64,
    prev_in_lift: f64,
    prev_angle: f64,
}

impl Bank {
    fn new(sample_rate: f64) -> Bank {
        let valve = ValveState {
            throat_area: 0.0,
            cyl_pressure: gas::P_AMB,
            cyl_temp: gas::T_AMB,
            cyl_gamma: gas::GAMMA_EXH,
            extra_mass_flow: 0.0,
        };
        Bank {
            cyl_state: CylState::default(),
            valve_state: valve,
            in_valve: valve,
            ex_lift: 0.0,
            in_lift: 0.0,
            seating_now: false,
            in_seating_now: false,
            opening_now: false,
            in_opening_now: false,
            tdc_pressure: -1.0,
            follower_load: 0.0,
            piston_speed: 0.0,
            bore_load: 0.0,
            rod_force: 0.0,
            rod_reversal: -1.0,
            ring_rise: 0.0,
            ring_temp: 0.0,
            needle: 0,
            seat_pulse: Impact::new(VALVE_CONTACT_S, sample_rate),
            turb1: 0.0,
            turb2: 0.0,
            last_valve_mdot: 0.0,
            prev_ex_lift: 0.0,
            prev_in_lift: 0.0,
            prev_angle: 0.0,
        }
    }
}

/// What lands on a cylinder's seats as its valves close, and what closes their lash as they open, kg:
/// every exhaust valve together, and every intake. See `CLOSING_RAMP_M_PER_DEG`.
#[derive(Clone, Copy, Debug, Default)]
struct ValveMasses {
    seat_ex: f64,
    seat_in: f64,
    lash_ex: f64,
    lash_in: f64,
}

impl ValveMasses {
    fn of(spec: &EngineSpec) -> ValveMasses {
        let valve = |dia: f64| VALVE_MASS_PER_D3 * dia * dia * dia;
        let lash = if spec.pushrods { LASH_MASS_PUSHROD } else { LASH_MASS_BUCKET };
        let ex = valve(spec.ex_valve_dia) * spec.ex_valve_count;
        let inl = valve(spec.in_valve_dia) * spec.in_valve_count;
        ValveMasses {
            seat_ex: SEATING_MASS_SHARE * ex,
            seat_in: SEATING_MASS_SHARE * inl,
            lash_ex: lash * ex,
            lash_in: lash * inl,
        }
    }
}

/// One of the casing's radiating surfaces, as the sources place it.
struct Surface {
    kind: SurfaceKind,
    /// A thin panel's own modes, none on a stiff casting, and each one's drive: `PANEL_SHARE` over what it
    /// gains at its own frequency.
    panel: Vec<Resonator>,
    panel_gain: Vec<f64>,
    /// The two-pole band limit on what it radiates.
    lp1: f64,
    lp2: f64,
}

/// Each sound in `x` that goes along `route`, shared out as it says.
#[inline]
fn emit(acc: &mut [f64], route: &[(usize, f64)], x: f64) {
    for &(i, share) in route {
        acc[i] += share * x;
    }
}

/// What every cylinder's valves read this sample.
struct ValveCtx<'a> {
    spec: &'a SpecInstance,
    /// The cam profile in use, for the lifts.
    lift_spec: &'a EngineSpec,
    timing: &'a [f64],
    delivery: &'a [f64],
    injection_offset: &'a [f64],
    intake_shift: f64,
    exhaust_shift: f64,
    limiter_cut: bool,
    fuel_demand: f64,
    crackle: Option<CrackleSpark>,
    rpm: f64,
    /// Crank speed, rad/s, ripple included.
    omega: f64,
    sample_rate: f64,
}

impl ValveCtx<'_> {
    /// Cylinder `b`'s valves for this sample: their lifts and flow areas, the cylinder's state as they
    /// see it, and the throat turbulence and seating pulse its exhaust valve sheds into a primary whose
    /// port is at `port_abs`, Pa.
    fn step(&self, b: usize, cyl: &mut Cylinder, bank: &mut Bank, noise: &mut Noise, port_abs: f64) {
        let angle = cyl.angle;
        cyl.spark_cut = self.limiter_cut;
        cyl.fuel_demand = self.fuel_demand;
        cyl.delivery = self.delivery.get(b).copied().unwrap_or(1.0);
        cyl.injection_offset = self.injection_offset.get(b).copied().unwrap_or(0.0);
        cyl.crackle = self.crackle;
        cyl.intake_cam_offset = self.timing[b] + self.intake_shift;
        cyl.exhaust_cam_offset = self.timing[b] + self.exhaust_shift;
        let lift_spec = self.lift_spec;
        let ex_offset = self.timing[b] + self.exhaust_shift;
        let in_offset = self.timing[b] + self.intake_shift;
        let ex_lift = valve_lift(angle, lift_spec.evo + ex_offset, lift_spec.evc + ex_offset, lift_spec.max_lift);
        let in_lift = valve_lift(angle, lift_spec.ivo + in_offset, lift_spec.ivc + in_offset, lift_spec.max_lift);
        let ex_area = valve_flow_area(ex_lift, lift_spec.ex_valve_dia) * lift_spec.ex_valve_count;
        let state = cyl.read_state(self.spec);
        bank.cyl_state = state;
        let p_cyl = state.pressure;
        let t_cyl = state.temp;
        let spec = &self.spec.spec;

        // Throat turbulence, scaled by the previous sample's flow through this valve.
        let mut extra_mass_flow = 0.0;
        if ex_area > 0.0 && spec.throat_noise > 0.0 {
            let throat_rho = port_abs / (gas::R * t_cyl);
            let speed = math::min(
                bank.last_valve_mdot.abs() / math::max(throat_rho * ex_area, 1e-9),
                math::sqrt(gas::GAMMA_CYL * gas::R * t_cyl),
            );
            let strouhal_hz = (0.2 * speed) / spec.ex_valve_dia;
            let k = clamp(1.0 - math::exp((-2.0 * PI * strouhal_hz) / self.sample_rate), 0.02, 0.85);
            let white = noise.next() * bank.last_valve_mdot.abs() * TURBULENCE_INTENSITY * spec.throat_noise;
            bank.turb1 += k * (white - bank.turb1);
            bank.turb2 += k * (bank.turb1 - bank.turb2);
            extra_mass_flow += bank.turb2;
        }

        let seating = bank.prev_ex_lift > 0.0 && ex_lift == 0.0;
        if seating {
            bank.seat_pulse.trigger(spec.mech_noise * 0.02 * (self.rpm / 3000.0));
        }
        extra_mass_flow += bank.seat_pulse.next();

        let cyl_gamma = 1.0 + gas::R / (CV_REF + CV_SLOPE * (t_cyl - T_REF));
        bank.valve_state =
            ValveState { throat_area: ex_area, cyl_pressure: p_cyl, cyl_temp: t_cyl, cyl_gamma, extra_mass_flow };
        bank.in_valve = ValveState {
            throat_area: valve_flow_area(in_lift, spec.in_valve_dia) * spec.in_valve_count,
            cyl_pressure: p_cyl,
            cyl_temp: t_cyl,
            cyl_gamma,
            extra_mass_flow: 0.0,
        };

        bank.ex_lift = ex_lift;
        bank.in_lift = in_lift;
        bank.seating_now = seating;
        bank.in_seating_now = bank.prev_in_lift > 0.0 && in_lift == 0.0;
        bank.opening_now = bank.prev_ex_lift == 0.0 && ex_lift > 0.0;
        bank.in_opening_now = bank.prev_in_lift == 0.0 && in_lift > 0.0;
        bank.tdc_pressure =
            if crossed_angle(bank.prev_angle, angle, 0.0) || crossed_angle(bank.prev_angle, angle, 360.0) {
                p_cyl
            } else {
                -1.0
            };
        bank.ring_rise = 0.0;
        bank.needle = 0;
        if spec.fuel == Fuel::Diesel && cyl.cycle_fuel() > 0.0 {
            let prev = bank.prev_angle;
            if cyl.premixed_energy > 0.0 && crossed_angle(prev, angle, cyl.spark) {
                // Lit, its premixed charge burns at once and heats the gas the chamber rings in, raising
                // its pressure as it would at a standstill.
                let heat = cyl.premixed_energy / (math::max(cyl.mass, 1e-9) * CV_REF);
                bank.ring_rise = p_cyl * heat / math::max(t_cyl, 1.0);
                bank.ring_temp = t_cyl + heat;
            }
            if crossed_angle(prev, angle, cyl.injection_start) {
                bank.needle = 1;
            } else if crossed_angle(prev, angle, cyl.injection_end) {
                bank.needle = 2;
            }
        }
        bank.prev_angle = angle;

        // --- What the mechanism carries ---
        // Each valve's spring, from its preload at the seat to its full load at full lift; several
        // valves' noise adds as the square root of their number.
        let spring = |lift: f64| {
            if lift > 0.0 {
                SPRING_PRELOAD_SHARE + (1.0 - SPRING_PRELOAD_SHARE) * (lift / math::max(lift_spec.max_lift, 1e-4))
            } else {
                0.0
            }
        };
        bank.follower_load = spring(ex_lift) * math::sqrt(math::max(lift_spec.ex_valve_count, 1.0))
            + spring(in_lift) * math::sqrt(math::max(lift_spec.in_valve_count, 1.0));

        // Down the rod's line: the gas on the crown, and what it takes to move the piston as it moves.
        // The rod leans, so the piston bears on the bore with the share of that its lean gives.
        let g = &self.spec.crank;
        let k = cyl.crank_now(self.spec);
        let (sin, cos) = math::sincos(angle * (PI / 180.0));
        let lean = (g.a * sin) / math::max(k.position - g.a * cos, 1e-6);
        let axial = (p_cyl - gas::P_AMB) * g.area + spec.recip_mass * k.d2_position * self.omega * self.omega;
        bank.piston_speed = k.d_position * self.omega;
        bank.bore_load =
            RING_TENSION_N + (axial * lean).abs() + RING_GAS_SHARE * math::max(p_cyl - gas::P_AMB, 0.0) * g.area;
        bank.rod_reversal =
            if axial * bank.rod_force < 0.0 { (axial - bank.rod_force).abs() * self.sample_rate } else { -1.0 };
        bank.rod_force = axial;
    }
}

/// One part of what `EngineSim::step_sample` does once the ducts are stepped.
#[derive(Clone, Copy, Debug)]
enum CloseItem {
    Bank(usize),
    /// The plenum, then once it is stepped each inlet tract or the turbo, which draw through it.
    Plenum,
    Inlet(usize),
    Turbo,
    Mouth(usize),
    Shell(usize),
}

/// What the plenum's step leaves for the inlet tracts and the turbo: the flow in through the
/// throttles, kg/s, in all and through each; each throttle body's area, m^2; and the pressure at the
/// throttles, Pa.
#[derive(Clone, Copy, Debug, Default)]
struct PlenumOut {
    throttle_flow: f64,
    flows: [f64; 2],
    area: f64,
    pressure: f64,
}

/// What the plenum left, once its step for sample `stamp` is done.
fn wait_for(done: &CachePadded<AtomicU64>, stamp: u64, out: &Disjoint<PlenumOut>) -> PlenumOut {
    while done.0.load(Ordering::Acquire) != stamp {
        std::hint::spin_loop();
    }
    // Written once, before `done` was marked, and only read after.
    unsafe { out.read(0) }
}

/// What a cylinder gave over a sample: its torque, N m, and its pressure's rate of rise, Pa/s.
#[derive(Clone, Copy, Debug, Default)]
struct BankOut {
    torque: f64,
    dpdt: f64,
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

        let wg =
            build_exhaust_for(&spec.spec, &pipe, &collector_pipe, graph.as_ref(), sample_rate, &wg_options, 1.0, None);
        let plenum = IntakePlenum::new(&spec.spec, sample_rate);
        let rattle_c = 1.0 - math::exp((-2.0 * PI * TIMING_RATTLE_HZ) / sample_rate);

        let mut sim = EngineSim {
            sample_rate,
            spec,
            pipe,
            collector_pipe,
            wg,
            cyls: Vec::new(),
            far_fields: Vec::new(),
            jets: Vec::new(),
            steepening: Vec::new(),
            shells: Vec::new(),
            banks: Vec::new(),
            throat_noise: Vec::new(),
            clack: Vec::new(),
            valve_masses: ValveMasses::default(),
            slap: Vec::new(),
            clack_impact: Vec::new(),
            slap_impact: Vec::new(),
            head_share: 1.0,
            slap_count: 0,
            scuff: Vec::new(),
            follower: Vec::new(),
            knock: Vec::new(),
            knock_impact: Vec::new(),
            rub_noise: Vec::new(),
            knock_count: 0,
            timing_mesh: 0.0,
            timing_rattle: 0.0,
            timing_rattle_c: rattle_c,
            // A one-pole low-pass leaves `c / (2 - c)` of the variance of the uniform noise it smooths,
            // which is a third.
            timing_rattle_gain: math::sqrt((3.0 * (2.0 - rattle_c)) / rattle_c),
            timing_noise: Noise::new(0x6a09e667_u32 as f64),
            timing_share: 1.0,
            twist: Resonator::new(TWIST_MODE.0, TWIST_MODE.1, sample_rate),
            twist_share: 1.0,
            structure: Vec::new(),
            diesel_block: Vec::new(),
            diesel_block_gain: Vec::new(),
            diesel_drive: Delay::new(DIESEL_BLOCK_SPREAD_S * sample_rate + 4.0),
            diesel_block_lag: Vec::new(),
            chamber: Vec::new(),
            chamber_burst: Vec::new(),
            chamber_last: Vec::new(),
            needle: Vec::new(),
            needle_impact: Vec::new(),
            dpdt_smooth: 0.0,
            dpdt_smooth_c: 1.0 - math::exp((-2.0 * PI * DPDT_BANDWIDTH_HZ) / sample_rate),
            diesel_dpdt_c: 1.0 - math::exp((-2.0 * PI * DIESEL_DPDT_BANDWIDTH_HZ) / sample_rate),
            diesel_hp_c: 0.0,
            diesel_hp: [0.0; 2],
            structure_lp_c: 1.0 - math::exp((-2.0 * PI * STRUCTURE_LIMIT_HZ) / sample_rate),
            diesel_structure_lp_c: 1.0 - math::exp((-2.0 * PI * DIESEL_STRUCTURE_LIMIT_HZ) / sample_rate),
            structure_lp1: 0.0,
            structure_lp2: 0.0,
            surfaces: Vec::new(),
            surface_pa: Vec::new(),
            surface_path: 0,
            side_route: Vec::new(),
            head_route: Vec::new(),
            block_route: Vec::new(),
            pan_route: Vec::new(),
            front_route: Vec::new(),
            listener: Listener::new(sample_rate),
            reverb: Reverb::new(sample_rate),
            room_modes: RoomModes::new(sample_rate),
            mode_sources: Vec::new(),
            sources: config.sources.clone().unwrap_or_default(),
            ear: config.listener,
            right: None,
            stereo: false,
            inlets: Vec::new(),
            intake_far_fields: [FarField::new(sample_rate, 0.0), FarField::new(sample_rate, 0.0)],
            entry_loss: Vec::new(),
            timing: Vec::new(),
            delivery: Vec::new(),
            injection_offset: Vec::new(),
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
            substeps: 0,
            omega: 0.0,
            omega_mean: 0.0,
            omega_ripple: 0.0,
            torque_avg: 0.0,
            omega_display: 0.0,
            limiter_cut: false,
            fuel_cut_active: false,
            ignition: true,
            idle_learned: idle_valve_start(&config.engine),
            idle_fuel: 0.0,
            fuel_demand: 0.0,
            idle_last_rpm: 0.0,
            idle_rate: 0.0,
            idle_trend: 0.0,
            idle_settled_for: 0.0,
            crackle_active: false,
            crackle_time: 0.0,
            crackle_spent: false,
            crackle_opened: false,
            afterfire: Afterfire::default(),
            afterfires_seen: 0,
            launch: None,
            launch_opening: f64::NAN,
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
            budget_scale: 1.0,
            time_scale: 1.0,
            slow_phase: 0.0,
            slow_prev: [0.0; 2],
            slow_next: [0.0; 2],
            pool: None,
            max_threads: usize::MAX,
            runner_cells: Vec::new(),
            close_groups: Vec::new(),
            close_key: (0, 0, 0, 0, 0, false),
            bank_out: Vec::new(),
            mouth_out: Vec::new(),
            shell_out: Vec::new(),
            turbo_out: CachePadded(None),
            plenum_out: CachePadded(PlenumOut::default()),
            plenum_done: CachePadded(AtomicU64::new(0)),
            close_stamp: 0,
            close_ns: Vec::new(),
            port_pressure: Vec::new(),
        };
        sim.refresh_cam_profiles(false);
        sim.refresh_derived();
        sim.cyls = sim.build_cylinders();
        sim.allocate_per_cylinder();
        sim.build_intake();
        sim.structure = STRUCTURAL_MODES.iter().map(|&(hz, q)| Resonator::new(hz, q, sample_rate)).collect();
        sim.diesel_block =
            (0..DIESEL_BLOCK_COUNT).map(|_| Resonator::new(1000.0, DIESEL_BLOCK_Q, sample_rate)).collect();
        sim.diesel_block_gain = vec![0.0; DIESEL_BLOCK_COUNT];
        // Each mode struck a share of the spread after the blow, the shares scattered evenly over it by the
        // golden ratio, so no two neighbours in pitch are struck together.
        let golden = (math::sqrt(5.0) - 1.0) / 2.0;
        sim.diesel_block_lag = (0..DIESEL_BLOCK_COUNT)
            .map(|i| {
                let share = (i as f64 * golden).fract();
                share * DIESEL_BLOCK_SPREAD_S * sample_rate
            })
            .collect();
        sim.tune_structure();
        sim.omega_mean = (math::min(sim.spec.spec.rpm, sim.spec.spec.rev_limit) * 2.0 * PI) / 60.0;
        sim.omega = sim.omega_mean;
        sim.omega_display = sim.omega_mean;
        sim.refresh_far_fields();
        sim.refresh_paths(true);
        sim.refresh_turbo();
        sim
    }

    /// Fit, resize or remove the turbochargers to match the exhaust: there is one for each turbo placed
    /// in the exhaust with pipes feeding it, on its own settings or the engine's.
    fn refresh_turbo(&mut self) {
        let spec = &self.spec.spec;
        let settings: Vec<_> = self.wg.turbine_mounts().iter().map(|m| m.settings_for(spec)).collect();
        if settings.is_empty() {
            if self.turbo.take().is_some() {
                self.wg.set_turbines(&[]);
            }
            self.charge_p = gas::P_AMB;
            self.charge_t = gas::T_AMB;
        } else {
            // The charge pipes on the inlet tract's grid, which a turbocharged engine has no use for.
            let opts = EulerPipeOptions { cell_size: Some(self.requested_cell_size()), ..self.build_options(None) };
            let spec = &self.spec.spec;
            match &mut self.turbo {
                Some(t) => t.configure(spec, &settings, &opts),
                None => self.turbo = Some(Turbo::new(spec, &settings, self.sample_rate, &opts)),
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
        self.afterfire.set_charge(self.full_charge_kg, fuel_fraction_at(1.0));
        self.displacement_m3 = displacement(spec) * spec.cylinders as f64;
        self.load_torque_nm = load_torque_of(spec, self.turbo.is_some());
    }

    /// One far field, one steepening run and one jet per mouth, tuned to that mouth, keeping existing
    /// filters' state; and every chamber's shell, afresh.
    fn refresh_far_fields(&mut self) {
        let count = self.wg.mouth_count().max(1);
        while self.far_fields.len() < count {
            let m = self.far_fields.len();
            self.far_fields.push(FarField::new(self.sample_rate, self.wg.mouth_cutoff_rad_of(m)));
            self.steepening.push(Steepening::new(self.sample_rate));
            self.jets.push(MouthJet::new(0x3c6ef372 as f64 + m as f64 * 7919.0));
        }
        self.far_fields.truncate(count);
        self.steepening.truncate(count);
        self.jets.truncate(count);
        for m in 0..count {
            let (c, b) = (self.wg.mouth_cutoff_rad_of(m), self.wg.plane_wave_cutoff_rad_of(m));
            self.far_fields[m].set_cutoff(c, b);
            let run = self.wg.radiating_duct(m).free_run;
            self.steepening[m].set_duct(run, self.wg.resolution_cutoff_rad_of(m));
        }
        self.shells.clear();
        for (d, duct) in self.wg.ducts.iter().enumerate() {
            for c in &duct.chambers {
                self.shells.push(ChamberShell::new(d, c, duct.dx, duct.n, duct.wall_thickness(), self.sample_rate));
            }
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
        self.launch_opening = f64::NAN;
    }

    /// Start a launch from standstill through `config`'s gearbox, or a dyno pull. A gearbox without a
    /// gear, or with one that is not a positive ratio, starts nothing; nor does a dyno pull without a
    /// positive sweep rate.
    pub fn start_launch(&mut self, config: LaunchConfig) {
        if config.ratios.is_empty() || config.ratios.iter().any(|r| !(*r > 0.0)) || !(config.final_drive > 0.0) {
            return;
        }
        if config.dyno && !(config.sweep_rate > 0.0) {
            return;
        }
        self.launch = Some(LaunchRun::new(config, full_load_torque(&self.spec.spec, self.turbo.is_some())));
        self.launch_opening = f64::NAN;
    }

    /// Switch the ignition on or off. Off cuts the spark and the fuel and lets the crank run down to a
    /// standstill; on again from a standstill starts the engine at its starting speed, as a fresh one
    /// does, with the pipes as they were left.
    pub fn set_ignition(&mut self, on: bool) {
        if on == self.ignition {
            return;
        }
        self.ignition = on;
        if !on {
            self.stop_launch();
        } else if self.omega_mean < (CATCH_RPM * 2.0 * PI) / 60.0 {
            let spec = &self.spec.spec;
            self.omega_mean = (math::min(spec.rpm, spec.rev_limit) * 2.0 * PI) / 60.0;
            self.omega = self.omega_mean;
            self.omega_display = self.omega_mean;
            self.omega_ripple = 0.0;
        }
    }

    /// The idle air valve, as an engine management system runs one: opened round the throttle plate by
    /// a PI controller on the crank speed, to hold the idle speed with the throttle shut. Above the idle
    /// speed it closes, so the throttle alone sets the speed; below it, it opens further against a load,
    /// up to its limit, past which the engine stalls. Only for a free-running engine with the ignition on:
    /// one held at a speed has no use for it. `throttle` is the plate's opening this sample.
    ///
    /// With the ignition off it stays where it was, as a stepper motor, or a drive-by-wire throttle's own
    /// motor, does when its power goes: air goes on leaking into the plenum through it, and the manifold's
    /// vacuum bleeds away as the engine coasts to rest, rather than through the plate's clearance alone.
    ///
    /// On a diesel the same controller is its idle governor, and meters fuel, `idle_fuel`, in place of air.
    fn update_idle_valve(&mut self, dt: f64, throttle: f64) {
        let spec = &self.spec.spec;
        if !(spec.free_running && spec.idle_rpm > 0.0) {
            self.plenum.set_bypass(spec, 0.0);
            self.idle_fuel = 0.0;
            return;
        }
        if !self.ignition {
            return;
        }
        let rpm = (self.omega_display * 60.0) / (2.0 * PI);
        let rate = (rpm - self.idle_last_rpm) / dt;
        self.idle_last_rpm = rpm;
        self.idle_rate += (rate - self.idle_rate) * (dt / IDLE_RATE_TAU);
        let error = (spec.idle_rpm - rpm) / spec.idle_rpm;
        let above = rpm >= IDLE_HOLD_ABOVE * spec.idle_rpm;
        let shut = throttle <= FUEL_CUT_THROTTLE && !self.crackle_active;
        self.idle_trend += (self.idle_rate - self.idle_trend) * (dt / IDLE_TREND_TAU);
        self.idle_settled_for = if !(above && shut) {
            0.0
        } else if self.idle_trend > -IDLE_SETTLED_RATE * spec.idle_rpm {
            self.idle_settled_for + dt
        } else {
            math::max(self.idle_settled_for - dt, 0.0)
        };
        let (ki, most) =
            if spec.fuel == Fuel::Diesel { (IDLE_FUEL_KI, IDLE_FUEL_MAX) } else { (IDLE_KI, IDLE_VALVE_MAX) };
        if !above {
            self.idle_learned = clamp(self.idle_learned + ki * error * dt, 0.0, most);
        } else if self.idle_settled_for >= IDLE_SETTLE_TIME {
            let unwind = ki * (1.0 - IDLE_HOLD_ABOVE) * dt;
            self.idle_learned = clamp(self.idle_learned + unwind, 0.0, most);
        }
        // How fast it is falling, in idle speeds per second: negative as it rises.
        let falling = -self.idle_rate / spec.idle_rpm;
        if spec.fuel == Fuel::Diesel {
            self.idle_fuel =
                clamp(self.idle_learned + IDLE_FUEL_KP * error + IDLE_FUEL_DASHPOT * falling, 0.0, IDLE_FUEL_MAX);
            return;
        }
        let opening = clamp(self.idle_learned + IDLE_KP * error + IDLE_DASHPOT * falling, 0.0, IDLE_VALVE_MAX);
        self.plenum.set_bypass(spec, opening);
    }

    /// End the launch.
    pub fn stop_launch(&mut self) {
        if let Some(d) = &mut self.launch {
            d.finish();
        }
    }

    /// Replace the engine with `next`, the whole spec. `set_engine_json` takes only the changes.
    pub fn set_engine(&mut self, next: EngineSpec) {
        let prev = &self.spec.spec;
        let prev_temp = prev.port_gas_temp;
        let prev_port_length = prev.port_length;
        let prev_port_dia = exhaust_port_diameter(prev);
        let prev_cell_size = prev.pipe_cell_size;
        let prev_wall_thickness = prev.pipe_wall_thickness;
        let prev_material = prev.pipe_material;
        let prev_runner: RunnerSize = intake_runner_of(prev);
        let prev_short_runner = prev.intake_runner_short_length;
        let prev_inlet = inlet_key(prev);
        let prev_layout = layout_key(prev);
        let prev_phase = firing_plan(prev).offsets;
        let was_high = self.high_cam_spec.is_some() && self.on_high_cam;
        self.spec = SpecInstance::new(next);
        self.refresh_cam_profiles(was_high);
        self.refresh_derived();
        if !self.spec.spec.free_running && !self.integrating_crank() {
            self.omega_mean = (self.spec.spec.rpm * 2.0 * PI) / 60.0;
        }
        self.refresh_paths(false);
        self.wg.set_turbulence(self.spec.spec.throat_noise);
        self.plenum.set_geometry(&self.spec.spec);
        self.refresh_turbo();
        self.launch_opening = f64::NAN;
        self.make_cylinder_variation(self.spec.spec.cylinders as usize);
        self.tune_structure();
        let spec = &self.spec.spec;
        let runner = intake_runner_of(spec);
        let intake_changed = runner.length != prev_runner.length
            || runner.diameter != prev_runner.diameter
            || spec.intake_runner_short_length != prev_short_runner
            || spec.pipe_cell_size != prev_cell_size
            || layout_key(spec) != prev_layout;
        let exhaust_changed = spec.port_gas_temp != prev_temp
            || spec.port_length != prev_port_length
            || exhaust_port_diameter(spec) != prev_port_dia
            || spec.pipe_wall_thickness != prev_wall_thickness
            || spec.pipe_material != prev_material;
        if !intake_changed && inlet_key(spec) != prev_inlet {
            // The tract alone: the runners and the exhaust keep their gas.
            let opts = EulerPipeOptions {
                cell_size: Some(self.requested_cell_size()),
                ..self.build_options(Some(self.wg.export_wall()))
            };
            self.inlets = build_inlets(&self.spec.spec, self.sample_rate, &opts);
            self.refresh_paths(false);
        }
        let spec = &self.spec.spec;
        if intake_changed || exhaust_changed {
            if layout_key(spec) != prev_layout {
                self.launch = None;
                self.allocate_per_cylinder();
                self.cyls = self.build_cylinders();
            }
            if intake_changed {
                self.rebuild_pipe();
            } else {
                self.rebuild_exhaust();
            }
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

    /// Replace the whole duct graph, for an exhaust that was drawn rather than chosen. One the solver
    /// would build the same, differing only in where things are drawn or in the turbos' own settings,
    /// keeps the gas in the pipes, the turbos resized where they are.
    pub fn set_graph(&mut self, graph: Option<ExhaustGraph>) {
        let same = match (&self.graph, &graph) {
            (Some(old), Some(new)) => solver_view(old) == solver_view(new),
            (None, None) => true,
            _ => false,
        };
        self.graph = graph;
        if !same {
            self.rebuild_exhaust();
        } else if let Some(g) = &self.graph {
            self.wg.update_turbo_settings(g);
            self.refresh_turbo();
        }
    }

    pub fn set_pipe(&mut self, pipe: &[PipeSegment], collector: Option<&[PipeSegment]>) {
        self.pipe = pipe.to_vec();
        if let Some(c) = collector {
            self.collector_pipe = c.to_vec();
        }
        self.rebuild_exhaust();
    }

    /// Rebuild the exhaust and the intake runners, for a change to both.
    fn rebuild_pipe(&mut self) {
        self.build_intake();
        self.rebuild_exhaust();
    }

    /// Rebuild the exhaust alone, for a change to it: the intake runners keep their gas, as the plenum does.
    /// Refilled at the atmosphere's pressure, they would hand a throttled engine a few full charges.
    fn rebuild_exhaust(&mut self) {
        self.wg = self.build_exhaust();
        for bank in self.banks.iter_mut() {
            bank.last_valve_mdot = 0.0;
        }
        self.afterfire.clear();
        self.refresh_far_fields();
        for f in self.far_fields.iter_mut() {
            f.reset();
        }
        for s in self.steepening.iter_mut() {
            s.reset();
        }
        self.refresh_paths(false);
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
            self.budget_scale,
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
        self.inlets = build_inlets(&self.spec.spec, sample_rate, &opts);
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
        let throttle = match &self.launch {
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

    /// Every cylinder's afterfire pocket.
    pub fn afterfire(&self) -> &Afterfire {
        &self.afterfire
    }

    /// Bank 0's cylinder.
    pub fn cylinder(&self) -> &Cylinder {
        &self.cyls[0]
    }

    /// Knocks each cam drive, then a diesel's injection pump's, has made across its slack since the
    /// engine was built.
    pub fn cylinders(&self) -> &[Cylinder] {
        &self.cyls
    }

    /// The finite intake manifold.
    pub fn plenum(&self) -> &IntakePlenum {
        &self.plenum
    }

    /// The intake runners the cylinders breathe through now.
    /// The air's way in to the throttle, on an engine without a turbo: the first's, with dual plenums.
    pub fn inlet(&self) -> Option<&InletTract> {
        self.inlets().first()
    }

    /// The air's way in to each throttle, on an engine without a turbo.
    pub fn inlets(&self) -> &[InletTract] {
        if self.turbo.is_none() { &self.inlets } else { &[] }
    }

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

    /// What the running noise rings at, Hz: each cylinder's bore under its rings, head under its
    /// followers and crankcase under its bearings, and the crank's twist.
    pub fn running_frequencies(&self) -> (Vec<f64>, Vec<f64>, Vec<f64>, f64) {
        let hz = |r: &Resonator| r.frequency(self.sample_rate);
        (
            self.scuff.iter().map(hz).collect(),
            self.follower.iter().map(hz).collect(),
            self.knock.iter().map(hz).collect(),
            hz(&self.twist),
        )
    }

    fn allocate_per_cylinder(&mut self) {
        let n = self.spec.spec.cylinders as usize;
        let sr = self.sample_rate;
        self.make_cylinder_variation(n);
        let full_charge = (gas::P_AMB * displacement(&self.spec.spec)) / (gas::R * gas::T_AMB);
        self.afterfire = Afterfire::new(n, full_charge, fuel_fraction_at(1.0));
        self.banks = (0..n).map(|_| Bank::new(sr)).collect();
        self.clack = (0..n).map(|_| Resonator::new(CLACK_MODE.0, CLACK_MODE.1, sr)).collect();
        self.slap = (0..n).map(|_| Resonator::new(SLAP_MODE.0, SLAP_MODE.1, sr)).collect();
        self.clack_impact = (0..n).map(|_| Impact::new(VALVE_CONTACT_S, sr)).collect();
        self.slap_impact = (0..n).map(|_| Impact::new(SLAP_CONTACT_S, sr)).collect();
        self.scuff = (0..n).map(|_| Resonator::new(SCUFF_MODE.0, SCUFF_MODE.1, sr)).collect();
        self.follower = (0..n).map(|_| Resonator::new(FOLLOWER_MODE.0, FOLLOWER_MODE.1, sr)).collect();
        self.knock = (0..n).map(|_| Resonator::new(KNOCK_MODE.0, KNOCK_MODE.1, sr)).collect();
        self.knock_impact = (0..n).map(|_| Impact::new(KNOCK_CONTACT_S, sr)).collect();
        self.chamber = (0..n).map(|_| std::array::from_fn(|_| Resonator::new(4000.0, CHAMBER_Q, sr))).collect();
        self.chamber_burst = (0..n).map(|_| Impact::new(CHAMBER_BURST_S, sr)).collect();
        self.chamber_last = vec![0.0; n];
        self.needle = (0..n).map(|_| Resonator::new(NEEDLE_MODE.0, NEEDLE_MODE.1, sr)).collect();
        self.needle_impact = (0..n).map(|_| Impact::new(NEEDLE_CONTACT_S, sr)).collect();
        self.rub_noise = (0..n).map(|b| Noise::new(0x3c6ef372 as f64 + b as f64 * 0x9e3779b as f64)).collect();
        if !self.structure.is_empty() {
            self.tune_structure();
        }
    }

    /// Pitch everything that rings to the size of this engine.
    fn tune_structure(&mut self) {
        let spec = &self.spec.spec;
        let size = clamp(math::cbrt(displacement(spec) / REFERENCE_DISPLACEMENT_M3), 0.5, 3.0);
        for (i, &(hz, q)) in STRUCTURAL_MODES.iter().enumerate() {
            self.structure[i].set(hz / size, q, self.sample_rate);
        }
        self.diesel_hp_c = if DIESEL_BLOCK_HIGHPASS_HZ > 0.0 {
            1.0 - math::exp((-2.0 * PI * (DIESEL_BLOCK_HIGHPASS_HZ / size)) / self.sample_rate)
        } else {
            0.0
        };
        for (i, mode) in self.diesel_block.iter_mut().enumerate() {
            let hz = diesel_block_hz(i);
            let top = 0.45 * self.sample_rate;
            mode.set(math::min(hz / size, top), DIESEL_BLOCK_Q, self.sample_rate);
            self.diesel_block_gain[i] = DIESEL_BLOCK_GAIN * math::pow(10.0, diesel_block_gain_db(hz) / 20.0);
        }
        self.head_share = clack_share(spec.cylinders as f64 / physical_bank_count(spec) as f64);
        self.valve_masses = ValveMasses::of(spec);

        let n = self.clack.len();
        let valve = REFERENCE_EX_VALVE / math::max(spec.ex_valve_dia, 1e-3);
        let bore = REFERENCE_BORE / math::max(spec.bore, 1e-3);
        for b in 0..n {
            let t = spread_of(b, n, 7, 3);
            let u = spread_of(b, n, 3, 2);
            self.clack[b].set(CLACK_MODE.0 * valve * (1.0 + LOCAL_MODE_DETUNE * t), CLACK_MODE.1, self.sample_rate);
            self.slap[b].set(SLAP_MODE.0 * bore * (1.0 + LOCAL_MODE_DETUNE * u), SLAP_MODE.1, self.sample_rate);
            let w = spread_of(b, n, 5, 1);
            self.scuff[b].set(SCUFF_MODE.0 * bore * (1.0 + LOCAL_MODE_DETUNE * w), SCUFF_MODE.1, self.sample_rate);
            self.follower[b].set(
                FOLLOWER_MODE.0 * valve * (1.0 + LOCAL_MODE_DETUNE * u),
                FOLLOWER_MODE.1,
                self.sample_rate,
            );
            self.knock[b].set((KNOCK_MODE.0 / size) * (1.0 + LOCAL_MODE_DETUNE * t), KNOCK_MODE.1, self.sample_rate);
            self.needle[b].set(NEEDLE_MODE.0 * (1.0 + LOCAL_MODE_DETUNE * w), NEEDLE_MODE.1, self.sample_rate);
        }

        // One timing drive to each head; the crank twists lower the more throws it has.
        let heads = physical_bank_count(spec) as f64;
        self.timing_share = math::sqrt(heads);

        let throws = crank_pins(spec).len().max(1) as f64;
        self.twist.set(TWIST_MODE.0 / math::sqrt(throws), TWIST_MODE.1, self.sample_rate);
        self.twist_share = REFERENCE_DISPLACEMENT_M3 / math::max(displacement(spec) * spec.cylinders as f64, 1e-6);
        self.route_surfaces();
    }

    /// Set up the casing's surfaces from the sources, and where each part's sound goes among them. With
    /// none, everything is heard from the casing's one place, as it always was.
    fn route_surfaces(&mut self) {
        let placed = &self.sources.surfaces;
        let spec = &self.spec.spec;
        let n = spec.cylinders as usize;
        let length = (crank_pins(spec).len().max(1) as f64 - 1.0) * cylinder_spacing(spec)
            + math::max(cylinder_spacing(spec), 1.3 * spec.bore);
        let scale = clamp(PANEL_REF_LENGTH / math::max(length, 0.05), 0.5, 2.0);
        let sr = self.sample_rate;
        let modes_of = |kind: SurfaceKind| -> &'static [(f64, f64)] {
            match kind {
                SurfaceKind::OilPan => &OIL_PAN_MODES,
                SurfaceKind::Head => &COVER_MODES,
                _ => &[],
            }
        };
        // The same surfaces as before keep ringing as they were, retuned; others start afresh.
        let same =
            self.surfaces.len() == placed.len() && self.surfaces.iter().zip(placed).all(|(s, p)| s.kind == p.kind);
        if !same {
            self.surfaces = placed
                .iter()
                .map(|p| Surface {
                    kind: p.kind,
                    panel: modes_of(p.kind).iter().map(|&(hz, q)| Resonator::new(hz, q, sr)).collect(),
                    panel_gain: vec![0.0; modes_of(p.kind).len()],
                    lp1: 0.0,
                    lp2: 0.0,
                })
                .collect();
            self.surface_pa = vec![0.0; placed.len()];
        }
        for surface in self.surfaces.iter_mut() {
            let modes = surface.panel.iter_mut().zip(surface.panel_gain.iter_mut()).zip(modes_of(surface.kind));
            for ((mode, gain), &(hz, q)) in modes {
                let hz = math::min(hz * scale, 0.45 * sr);
                mode.set(hz, q, sr);
                *gain = PANEL_SHARE / mode.gain_at(hz, sr);
            }
        }

        // The surfaces of `kind`, of `bank` where it is given: failing those, of any bank; failing those,
        // every surface. Shared so the power they carry together is what one would.
        let pick = |kind: SurfaceKind, bank: Option<u32>| -> Vec<(usize, f64)> {
            let of = |bank: Option<u32>| -> Vec<usize> {
                (0..placed.len())
                    .filter(|&i| placed[i].kind == kind && bank.is_none_or(|b| placed[i].bank == b))
                    .collect()
            };
            let mut found = of(bank);
            if found.is_empty() {
                found = of(None);
            }
            if found.is_empty() {
                found = (0..placed.len()).collect();
            }
            let share = 1.0 / math::sqrt(found.len().max(1) as f64);
            found.into_iter().map(|i| (i, share)).collect()
        };
        let scaled = |route: Vec<(usize, f64)>, by: f64| -> Vec<(usize, f64)> {
            route.into_iter().map(|(i, s)| (i, s * by)).collect()
        };
        if placed.is_empty() {
            self.side_route.clear();
            self.head_route.clear();
            self.block_route.clear();
            self.pan_route.clear();
            self.front_route.clear();
            return;
        }
        self.side_route = (0..n).map(|b| pick(SurfaceKind::BlockSide, Some(physical_bank(spec, b)))).collect();
        self.head_route = (0..n).map(|b| pick(SurfaceKind::Head, Some(physical_bank(spec, b)))).collect();
        self.pan_route = pick(SurfaceKind::OilPan, None);
        self.front_route = pick(SurfaceKind::FrontCover, None);
        let mut block = scaled(pick(SurfaceKind::BlockSide, None), math::sqrt(BLOCK_SIDE_SHARE));
        block.extend(scaled(self.pan_route.clone(), math::sqrt(1.0 - BLOCK_SIDE_SHARE)));
        self.block_route = block;
    }

    /// Fixed per-cylinder runner entry losses and cam timing offsets, spread evenly and shuffled.
    fn make_cylinder_variation(&mut self, n: usize) {
        let spread = clamp(self.spec.spec.cylinder_spread, 0.0, 2.0);
        self.entry_loss = vec![0.0; n];
        self.timing = vec![0.0; n];
        self.delivery = vec![1.0; n];
        self.injection_offset = vec![0.0; n];
        for b in 0..n {
            let t = spread_of(b, n, 5, 2);
            let u = spread_of(b, n, 3, 1);
            self.entry_loss[b] = ENTRY_LOSS_SPREAD * spread * t;
            self.timing[b] = CAM_SPREAD_DEG * spread * u;
            self.delivery[b] = 1.0 + DELIVERY_SPREAD * spread * spread_of(b, n, 7, 3);
            self.injection_offset[b] = INJECTION_SPREAD_DEG * spread * spread_of(b, n, 2, 1);
        }
    }

    /// Where the engine makes its sound, as drawn: each tailpipe's outlet, the intake, the casing and
    /// the turbos. A mouth it does not place stands in a row across the car with the others it does not
    /// place, `UNPLACED_MOUTH_SPACING` apart, behind the engine; an intake it does not place is above
    /// the front of the engine, and a casing or turbo at the crank's middle.
    pub fn set_sources(&mut self, sources: SoundSources) {
        self.sources = sources;
        self.refresh_paths(false);
    }

    /// Put the listener's ear at `ear`, m, in the frame the sources are in; with `None`,
    /// `DEFAULT_EAR_DISTANCE` from the middle of the mouths, at 45 degrees off the car's rear axis and
    /// `DEFAULT_EAR_HEIGHT` above the ground. The paths glide to it.
    pub fn set_listener(&mut self, ear: Option<Vec3>) {
        self.set_listener_facing(ear, None);
    }

    /// `set_listener`, with the listener's right the way `right` is, in the same frame, which in stereo
    /// sets which ear is which. With `None`, or where there is no ear given, they face the middle of the
    /// mouths.
    pub fn set_listener_facing(&mut self, ear: Option<Vec3>, right: Option<Vec3>) {
        self.ear = ear;
        self.right = right;
        self.refresh_paths(false);
    }

    /// Hear the engine with two ears, a head apart, or with one. With one, both channels of
    /// `render_stereo_into` are alike.
    pub fn set_stereo(&mut self, on: bool) {
        if on != self.stereo {
            self.stereo = on;
            self.refresh_paths(false);
        }
    }

    /// Every source's path to the ear, from where the sources are and where the ear is. The ground is
    /// `exhaust_height` below the lowest mouth. A room stands on it around the sources, with the ear
    /// kept inside it.
    fn refresh_paths(&mut self, snap: bool) {
        let count = self.wg.mouth_count().max(1);
        let unplaced: Vec<usize> = (0..count)
            .filter(|&m| !self.sources.mouths.iter().any(|p| Some(p.duct.as_str()) == self.wg.mouth_duct_id(m)))
            .collect();
        let mut places: Vec<Vec3> = (0..count)
            .map(|m| {
                let id = self.wg.mouth_duct_id(m);
                match self.sources.mouths.iter().find(|p| Some(p.duct.as_str()) == id) {
                    Some(p) => p.position,
                    None => {
                        let k = unplaced.iter().position(|&u| u == m).unwrap_or(0) as f64;
                        let lateral = (k - (unplaced.len() as f64 - 1.0) / 2.0) * UNPLACED_MOUTH_SPACING;
                        [lateral, 0.0, UNPLACED_MOUTH_REAR]
                    }
                }
            })
            .collect();
        let n = count as f64;
        let middle = [
            places.iter().map(|p| p[0]).sum::<f64>() / n,
            places.iter().map(|p| p[1]).sum::<f64>() / n,
            places.iter().map(|p| p[2]).sum::<f64>() / n,
        ];
        let ground = places.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min) - self.spec.spec.exhaust_height;
        let casing = self.sources.engine.unwrap_or([0.0; 3]);
        // The muffler shells radiate from along the exhaust: from the middle of its mouths.
        places.push(middle);
        places.push(self.sources.intake.unwrap_or(UNPLACED_INTAKE));
        places.push(casing);
        places.push(self.sources.turbo.unwrap_or(casing));
        if self.inlets.len() > 1 {
            let first = self.sources.intake.unwrap_or(UNPLACED_INTAKE);
            places.push(self.sources.second_intake.unwrap_or([-first[0], first[1], first[2]]));
        }
        let ear = self.ear.unwrap_or_else(|| {
            let lean = DEFAULT_EAR_DISTANCE * math::sin(PI / 4.0);
            [middle[0] + lean, ground + DEFAULT_EAR_HEIGHT, middle[2] + lean]
        });
        let room = self.spec.spec.room.shape();
        let walls = room.map(|r| {
            let n = places.len() as f64;
            let (cx, cz) = (places.iter().map(|p| p[0]).sum::<f64>() / n, places.iter().map(|p| p[2]).sum::<f64>() / n);
            Walls {
                x: [cx - r.width / 2.0, cx + r.width / 2.0],
                z: [cz - r.length / 2.0, cz + r.length / 2.0],
                ceiling: ground + r.height,
                reflection: r.wall_reflections(),
                corner_hz: r.wall_corner_hz,
            }
        });
        // The casing's surfaces after everything else, each facing out of the engine.
        let mut facings: Vec<Option<Vec3>> = vec![None; places.len()];
        self.surface_path = places.len();
        for surface in &self.sources.surfaces {
            places.push(surface.position);
            facings.push(Some(surface.facing));
        }
        self.route_surfaces();
        let ear = match &walls {
            Some(w) => w.inside(ear, ground, EAR_MARGIN),
            None => ear,
        };
        // Facing the middle of the mouths, where no way is given: right is forward turned a quarter
        // clockwise, seen from above.
        let right = self.stereo.then(|| {
            self.right.filter(|_| self.ear.is_some()).unwrap_or_else(|| {
                let (fx, fz) = (middle[0] - ear[0], middle[2] - ear[2]);
                [-fz, 0.0, fx]
            })
        });
        let reflection = self.spec.spec.ground_reflection;
        self.listener.set_geometry(ear, right, &places, &facings, ground, reflection, walls.as_ref(), snap);
        self.reverb.set_room(room);
        self.reverb.set_head(right);
        self.room_modes.set(room.as_ref(), walls.as_ref(), ground, &places, ear);
        self.reverb.set_low_cut(self.room_modes.top_hz());

        self.refresh_intake_far_field();
    }

    /// Each snorkel's mouth radiates as a tailpipe's does, up to its tract's own band limit.
    fn refresh_intake_far_field(&mut self) {
        for (k, far) in self.intake_far_fields.iter_mut().enumerate() {
            match self.inlets.get(k) {
                Some(inlet) => {
                    let p = &inlet.pipe;
                    far.set_cutoff(p.mouth_cutoff_rad, math::min(p.plane_wave_cutoff_rad, p.resolution_cutoff_rad));
                }
                None => {
                    let c = ambient_sound_speed();
                    let radius = throttle_dia_of(&self.spec.spec) / 2.0;
                    far.set_cutoff((2.0 * c) / radius, (1.8412 * c) / radius);
                }
            }
        }
    }

    /// The path `source` takes to the ear, after the mouths'.
    fn path_of(&self, source: Source) -> usize {
        self.wg.mouth_count().max(1) + source as usize
    }

    /// The cylinders, phased on one shared crank, each with its own noise seed.
    fn build_cylinders(&mut self) -> Vec<Cylinder> {
        let spec = &self.spec.spec;
        let plan = firing_plan(spec);
        let mut out = Vec::new();
        for b in 0..spec.cylinders as usize {
            out.push(Cylinder::new(spec, wrap_cycle(-plan.offsets[b]), 0x51f3a7 as f64 + b as f64 * 0x9e3779b as f64));
            if self.throat_noise.len() <= b {
                self.throat_noise.push(CachePadded(Noise::new(0x2c1b3d as f64 + b as f64 * 0x85ebca6b_u32 as f64)));
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
            self.budget_scale,
            Some(self.wg.export_wall()),
        )
    }

    /// First quarter-wave resonance of the exhaust as it is currently filled, Hz.
    pub fn duct_quarter_wave_hz(&self) -> f64 {
        self.wg.quarter_wave_hz()
    }

    /// Mean crank speed, rev/min.
    /// The torque friction takes off the crank at its present mean speed, N*m: a friction mean
    /// effective pressure of 0.8 bar plus 85 Pa per rad/s.
    pub fn friction_torque(&self) -> f64 {
        let fmep = 0.8e5 + 85.0 * self.omega_mean;
        (fmep * self.displacement_m3) / (4.0 * PI)
    }

    pub fn rpm(&self) -> f64 {
        let w = if self.integrating_crank() { self.omega_display } else { self.omega_mean };
        (w * 60.0) / (2.0 * PI)
    }

    fn integrating_crank(&self) -> bool {
        let spec = &self.spec.spec;
        !self.ignition || spec.free_running || spec.rpm >= spec.rev_limit || self.launch.is_some()
    }

    /// Instantaneous crank speed, rev/min, ripple included.
    pub fn rpm_instant(&self) -> f64 {
        (self.omega * 60.0) / (2.0 * PI)
    }

    // -------------------------------------------------------------------------
    // Simulation
    // -------------------------------------------------------------------------

    /// Advance one audio sample. Returns the listener signal at the left ear and the right, nominally
    /// in [-1, 1]: alike, out of stereo.
    pub fn tick(&mut self) -> [f64; 2] {
        let dt = 1.0 / self.sample_rate;

        // --- Crank speed ---
        let inertia = math::max(self.spec.spec.flywheel_inertia, 1e-3);
        let torque = self.torque_last;

        if self.integrating_crank() {
            let friction = self.friction_torque();
            let load = if let Some(launch) = &mut self.launch {
                let mut fresh = 0.0;
                for c in &self.cyls {
                    fresh += c.trapped_fresh;
                }
                launch.volumetric_efficiency = fresh / self.cyls.len() as f64 / self.full_charge_kg;
                launch.intake_pressure = self.plenum.pressure();
                launch.step(dt, self.omega_mean, torque - friction, self.cyls[0].angle)
            } else if self.spec.spec.free_running {
                self.load_torque_nm
            } else {
                0.0
            };
            let net = torque - load - friction;
            self.omega_mean += (net / inertia) * dt;
            // Down to a standstill: an engine the idle valve cannot hold up stalls.
            let max_omega = (12000.0 * 2.0 * PI) / 60.0;
            self.omega_mean = clamp(self.omega_mean, 0.0, max_omega);
            self.omega = self.omega_mean;
            self.omega_display += (self.omega_mean - self.omega_display) * (dt / IRREGULARITY_TAU);
            let spec = &self.spec.spec;
            let limit_omega = (spec.rev_limit * 2.0 * PI) / 60.0;
            if spec.fuel == Fuel::Diesel {
                // Its governor takes the fuel away instead: see `fuel_demand`.
                self.limiter_cut = false;
            } else if self.omega_mean >= limit_omega {
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
        self.plenum.update_balance(dt, (self.omega_mean * 60.0) / (2.0 * PI));
        if self.high_cam_spec.is_some() {
            self.update_cam_profile();
        }

        // --- Launch ---
        let mut throttle = self.spec.spec.throttle;
        if let Some((launch_throttle, cooldown, phase_time)) =
            self.launch.as_ref().map(|d| (d.throttle, d.phase == LaunchPhase::Cooldown, d.phase_time))
        {
            throttle = launch_throttle;
            if throttle != self.launch_opening {
                self.plenum.set_opening(&self.spec.spec, throttle);
                self.launch_opening = throttle;
            }
            let spec = &self.spec.spec;
            if cooldown
                && (spec.free_running
                    || self.omega_mean <= (spec.rpm * 2.0 * PI) / 60.0
                    || phase_time > LAUNCH_WIND_DOWN)
            {
                self.launch = None;
                self.plenum.set_geometry(&self.spec.spec);
                self.launch_opening = f64::NAN;
                throttle = self.spec.spec.throttle;
            }
        }

        // --- Idle air valve ---
        self.update_idle_valve(dt, throttle);

        // --- Overrun fuel cut, and the crackle map that holds it off after a lift ---
        let rpm_now = (self.omega_mean * 60.0) / (2.0 * PI);
        let (cut_rpm, resume_rpm) = fuel_cut_rpms(&self.spec.spec);
        let diesel = self.spec.spec.fuel == Fuel::Diesel;
        if diesel || !self.spec.spec.fuel_cut || throttle > FUEL_CUT_THROTTLE {
            self.fuel_cut_active = false;
        } else if rpm_now > cut_rpm {
            self.fuel_cut_active = true;
        } else if rpm_now < resume_rpm {
            self.fuel_cut_active = false;
        }
        if self.spec.spec.overrun_crackle && !diesel && throttle <= FUEL_CUT_THROTTLE && self.ignition {
            if !self.crackle_active && !self.crackle_spent && rpm_now > CRACKLE_RPM {
                self.crackle_active = true;
                self.crackle_time = 0.0;
            }
            if self.crackle_active {
                self.crackle_time += dt;
                if rpm_now < CRACKLE_END_RPM || self.crackle_time > CRACKLE_WINDOW {
                    self.crackle_active = false;
                    self.crackle_spent = true;
                }
            }
        } else {
            self.crackle_active = false;
            self.crackle_spent = false;
        }
        let intensity = clamp(self.spec.spec.crackle_intensity, 0.0, 1.0);
        if self.crackle_active != self.crackle_opened {
            let opening = if self.crackle_active {
                CRACKLE_THROTTLE_MIN + (CRACKLE_THROTTLE_MAX - CRACKLE_THROTTLE_MIN) * intensity
            } else {
                throttle
            };
            self.plenum.set_opening(&self.spec.spec, opening);
            self.crackle_opened = self.crackle_active;
        }
        let crackle = if self.crackle_active {
            Some(CrackleSpark {
                atdc: CRACKLE_ATDC_MIN + (CRACKLE_ATDC_MAX - CRACKLE_ATDC_MIN) * intensity,
                skip: CRACKLE_SKIP_MIN + (CRACKLE_SKIP_MAX - CRACKLE_SKIP_MIN) * intensity,
            })
        } else {
            None
        };

        // --- Valves and flows, per bank ---
        let banks = self.cyls.len();
        let mut torque_sum = 0.0;
        let mut dpdt_sum = 0.0;
        let limiter_cut = self.limiter_cut || !self.ignition || self.launch.as_ref().is_some_and(|l| l.spark_cut);
        // --- A diesel's fuel: the pedal's, or the idle governor's where that asks for more, taken away
        // over the last `GOVERNOR_DROOP` below the governed speed. Launch control cuts it as it would
        // a spark ---
        self.fuel_demand = if !diesel {
            1.0
        } else if limiter_cut {
            0.0
        } else {
            let droop = clamp((self.spec.spec.rev_limit - rpm_now) / GOVERNOR_DROOP, 0.0, 1.0);
            clamp(math::max(throttle, self.idle_fuel), 0.0, 1.0) * droop
        };
        let rpm = self.rpm();
        // --- Exhaust gas dynamics, all ducts in lockstep, with the turbine in them and afterfire ---
        for b in 0..banks {
            self.wg.afterfire_heat_mut()[b] = self.afterfire.heat_rate(b);
        }
        if let Some(turbo) = &mut self.turbo {
            self.wg.set_turbines(turbo.turbine_settings());
        }
        // --- Intake runners, all in lockstep, alongside the exhaust: neither reads the other ---
        let run_io = RunnerIo {
            dt,
            inject: if diesel || (self.fuel_cut_active && !self.crackle_active) || !self.ignition {
                0.0
            } else {
                self.inject_fraction
            },
        };
        // --- Then the cylinders and afterfire, the plenum and its inlet tracts or the turbo, and the radiation ---
        let throttle_flow = self.step_sample(dt, run_io, limiter_cut, crackle, rpm);
        self.substeps = self.wg.result.substeps;
        for out in &self.bank_out {
            torque_sum += out.0.torque;
            dpdt_sum += out.0.dpdt;
        }

        // --- Turbocharger ---
        let mut turbo_pa = 0.0;
        if let Some(out) = self.turbo_out.0.take() {
            self.charge_p = out.charge_p;
            self.charge_t = out.charge_t;
            turbo_pa = out.sound;
        }

        // --- Structure-borne noise ---
        let mut direct_pa = 0.0;
        // Spread over the casing's surfaces when the sources place them, or all from its one place.
        let split = !self.surfaces.is_empty();
        let mech = self.spec.spec.mech_noise;
        let diesel = self.spec.spec.fuel == Fuel::Diesel;
        let head_share = self.head_share;
        let mut follower_load = 0.0;
        let mut slap_knocks = 0.0;
        let mut ring_dpdt = 0.0;
        for b in 0..banks {
            let bank = &self.banks[b];
            // Each valve clacks onto its seat, and ticks as it leaves it and its lash closes: as loud as the
            // momentum each brings, at the ramp's speed.
            let on = |now: bool| if now { 1.0 } else { 0.0 };
            let mass = self.valve_masses;
            let landing = on(bank.seating_now) * mass.seat_ex
                + on(bank.in_seating_now) * mass.seat_in
                + on(bank.opening_now) * mass.lash_ex
                + on(bank.in_opening_now) * mass.lash_in;
            if landing > 0.0 {
                let ramp = CLOSING_RAMP_M_PER_DEG * (math::max(rpm, 0.0) * 6.0);
                self.clack_impact[b].trigger(CLACK_PA_PER_NS * mech * landing * ramp * head_share);
            }
            let hit = self.clack_impact[b].next();
            let x = self.clack[b].process(hit);
            if split { emit(&mut self.surface_pa, &self.head_route[b], x) } else { direct_pa += x }

            let p = bank.tdc_pressure;
            if p >= 0.0 {
                self.slap_count += 1;
                self.slap_impact[b].trigger(SLAP_PA_AT_1M * mech * clamp(p / 3e6, 0.05, 1.6) * head_share);
            }
            let hit = self.slap_impact[b].next();
            if diesel {
                // A diesel's heavy block rings to the knock as it does to the combustion, broadly.
                slap_knocks += hit;
            } else {
                let x = self.slap[b].process(hit);
                if split { emit(&mut self.surface_pa, &self.side_route[b], x) } else { direct_pa += x }
            }

            // The rod's bearings cross their clearance whenever the force down the rod changes sign.
            if bank.rod_reversal >= 0.0 {
                self.knock_count += 1;
                let rate = clamp(bank.rod_reversal / KNOCK_RATE_REF, 0.05, 1.5);
                self.knock_impact[b].trigger(KNOCK_PA_AT_1M * mech * rate * head_share);
            }
            let hit = self.knock_impact[b].next();
            let x = self.knock[b].process(hit);
            if split { emit(&mut self.surface_pa, &self.pan_route, x) } else { direct_pa += x }

            // A diesel's chamber ringing as its premixed charge lights, pitched to the gas it rings in,
            // its pressure swinging with the rest of the cylinder's; and its injector needle ticking as
            // it lifts and seats.
            if diesel {
                if bank.ring_rise > 0.0 {
                    let detune = 1.0 + LOCAL_MODE_DETUNE * spread_of(b, banks, 7, 3);
                    for (mode, &(alpha, _)) in self.chamber[b].iter_mut().zip(CHAMBER_MODES.iter()) {
                        let hz = chamber_mode_hz(alpha, bank.ring_temp, self.spec.spec.bore) * detune;
                        mode.set(math::min(hz, 0.45 * self.sample_rate), CHAMBER_Q, self.sample_rate);
                    }
                    let scatter = 1.0 + CHAMBER_SCATTER * self.rub_noise[b].next();
                    self.chamber_burst[b].trigger(CHAMBER_SHARE * bank.ring_rise * scatter);
                }
                let burst = self.chamber_burst[b].next();
                let mut ring = 0.0;
                for (mode, &(_, share)) in self.chamber[b].iter_mut().zip(CHAMBER_MODES.iter()) {
                    ring += share * mode.process(burst);
                }
                ring_dpdt += (ring - self.chamber_last[b]) / dt;
                self.chamber_last[b] = ring;
                if bank.needle > 0 {
                    let share = if bank.needle == 1 { NEEDLE_LIFT_SHARE } else { 1.0 };
                    self.needle_impact[b].trigger(NEEDLE_PA_AT_1M * mech * share * head_share);
                }
                let hit = self.needle_impact[b].next();
                let x = self.needle[b].process(hit);
                if split { emit(&mut self.surface_pa, &self.head_route[b], x) } else { direct_pa += x }
            }

            // The rings and skirt rubbing the bore, as loud as the speed they slide at and the load on
            // them; the followers rubbing their cam lobes, as loud as what their springs push back with.
            let noise = &mut self.rub_noise[b];
            let rub = (bank.piston_speed.abs() / SCUFF_SPEED_REF) * (bank.bore_load / SCUFF_LOAD_REF);
            let x = self.scuff[b].process(SCUFF_PA_AT_1M * mech * rub * head_share * noise.next());
            if split { emit(&mut self.surface_pa, &self.side_route[b], x) } else { direct_pa += x }
            let cam = bank.follower_load * (rpm / 3000.0);
            let x = self.follower[b].process(FOLLOWER_PA_AT_1M * mech * cam * head_share * noise.next());
            if split { emit(&mut self.surface_pa, &self.head_route[b], x) } else { direct_pa += x }
            follower_load += bank.follower_load;
        }

        // The timing drive meshing, its tension wavering with the valve springs it turns against and with
        // its own slack.
        let mesh_hz = (TIMING_TEETH * self.omega) / (2.0 * PI);
        let mesh = self.timing_mesh + mesh_hz * dt;
        self.timing_mesh = mesh - mesh.floor();
        self.timing_rattle += self.timing_rattle_c * (self.timing_noise.next() - self.timing_rattle);
        if mech > 0.0 {
            let speed = math::max(rpm, 0.0) / TIMING_REF_RPM;
            let load = follower_load / math::max(banks as f64, 1.0);
            let level = TIMING_PA_AT_1M * mech * speed * math::sqrt(speed) * self.timing_share;
            let tension = (0.6 + 0.4 * load) * (1.0 + TIMING_RATTLE * self.timing_rattle_gain * self.timing_rattle);
            let phase = 2.0 * PI * self.timing_mesh;
            let x = level * tension * (math::sin(phase) + TIMING_SECOND_HARMONIC * math::sin(2.0 * phase));
            if split { emit(&mut self.surface_pa, &self.front_route, x) } else { direct_pa += x }
        }

        // The crank twisting under the torque it carries, on its first torsional mode.
        let x = self.twist.process(TWIST_PA_PER_NM * mech * torque_sum * self.twist_share);
        if split { emit(&mut self.surface_pa, &self.pan_route, x) } else { direct_pa += x }

        // Combustion shaking the casing, driven by the summed pressure rise rate: a diesel's through its
        // broader block.
        if mech > 0.0 && diesel {
            self.dpdt_smooth += self.diesel_dpdt_c * (dpdt_sum + ring_dpdt - self.dpdt_smooth);
            let mut fast = self.dpdt_smooth;
            if self.diesel_hp_c > 0.0 {
                for stage in self.diesel_hp.iter_mut() {
                    *stage += self.diesel_hp_c * (fast - *stage);
                    fast -= *stage;
                }
            }
            let drive = (fast / 1e9) * STRUCTURE_PA_PER_GPA_S * mech + DIESEL_SLAP_DRIVE * slap_knocks;
            self.diesel_drive.push(drive);
            let modes =
                self.diesel_block.iter_mut().zip(self.diesel_block_gain.iter()).zip(self.diesel_block_lag.iter());
            for ((mode, &gain), &lag) in modes {
                let x = mode.process(self.diesel_drive.tap(lag)) * gain;
                if split { emit(&mut self.surface_pa, &self.block_route, x) } else { direct_pa += x }
            }
        } else if mech > 0.0 {
            self.dpdt_smooth += self.dpdt_smooth_c * (dpdt_sum - self.dpdt_smooth);
            let drive = (self.dpdt_smooth / 1e9) * STRUCTURE_PA_PER_GPA_S * mech;
            for mode in self.structure.iter_mut() {
                let x = mode.process(drive) * 0.25;
                if split { emit(&mut self.surface_pa, &self.block_route, x) } else { direct_pa += x }
            }
        }

        // Each thin panel ringing with what it carries, then the band limit on everything
        // structure-borne.
        let lp_c = if diesel { self.diesel_structure_lp_c } else { self.structure_lp_c };
        for (surface, carried) in self.surfaces.iter_mut().zip(self.surface_pa.iter_mut()) {
            let mut x = *carried;
            for (mode, &gain) in surface.panel.iter_mut().zip(surface.panel_gain.iter()) {
                x += gain * mode.process(*carried);
            }
            surface.lp1 += lp_c * (x - surface.lp1);
            surface.lp2 += lp_c * (surface.lp1 - surface.lp2);
            *carried = surface.lp2;
        }
        self.structure_lp1 += lp_c * (direct_pa - self.structure_lp1);
        self.structure_lp2 += lp_c * (self.structure_lp1 - self.structure_lp2);
        direct_pa = self.structure_lp2;

        self.torque_last = torque_sum;

        // --- Radiate ---
        let mut pa = [0.0; 2];
        let add = |pa: &mut [f64; 2], heard: [f64; 2]| {
            pa[0] += heard[0];
            pa[1] += heard[1];
        };
        for out in &self.mouth_out {
            add(&mut pa, out.0);
        }
        let mut shells_pa = 0.0;
        for out in &self.shell_out {
            shells_pa += out.0;
        }
        add(&mut pa, self.listener.process(self.path_of(Source::Shells), shells_pa));
        // The throttle's mouth breathes in what the engine draws. A turbocharged engine draws through its
        // compressors instead, whose inlets the turbo radiates itself.
        if self.turbo.is_none() {
            if self.inlets.is_empty() {
                let intake_pa = self.intake_far_fields[0].process(-throttle_flow / density(gas::P_AMB, gas::T_AMB));
                add(&mut pa, self.listener.process(self.path_of(Source::Intake), intake_pa));
            }
            for (k, inlet) in self.inlets.iter().enumerate() {
                let intake_pa = self.intake_far_fields[k].process(inlet.mouth_flow);
                let source = if k == 0 { Source::Intake } else { Source::SecondIntake };
                add(&mut pa, self.listener.process(self.path_of(source), intake_pa));
            }
        }
        add(&mut pa, self.listener.process(self.path_of(Source::Casing), direct_pa));
        for i in 0..self.surface_pa.len() {
            let carried = std::mem::take(&mut self.surface_pa[i]);
            add(&mut pa, self.listener.process(self.surface_path + i, carried));
        }
        if self.turbo.is_some() {
            add(&mut pa, self.listener.process(self.path_of(Source::Turbo), turbo_pa));
        }

        if self.room_modes.top_hz() > 0.0 {
            // The room's modes, below where its field is diffuse, alike at both ears: their
            // wavelengths are metres long.
            self.listener.each_source(&mut self.mode_sources);
            let modal = self.room_modes.process(&self.mode_sources);
            add(&mut pa, [modal, modal]);
        }
        let sources = self.listener.take_sources();
        if self.reverb.active() {
            if self.listener.ears() == 2 {
                add(&mut pa, self.reverb.process_stereo(sources));
            } else {
                let diffuse = self.reverb.process(sources);
                add(&mut pa, [diffuse, diffuse]);
            }
        }

        if self.rebuild_ramp < 1.0 {
            self.rebuild_ramp = math::min(1.0, self.rebuild_ramp + self.rebuild_ramp_step);
            pa[0] *= self.rebuild_ramp;
            pa[1] *= self.rebuild_ramp;
        }

        let channels = if self.listener.ears() == 2 { 2 } else { 1 };
        let mut out = [0.0; 2];
        for ch in 0..channels {
            let mut o = (pa[ch] / PA_PER_FULLSCALE) * self.spec.spec.output_gain;
            if !o.is_finite() {
                o = 0.0;
            }
            o = soft_clip(o);
            let mag = o.abs();
            if mag > self.peak {
                self.peak = mag;
            }
            out[ch] = o;
        }
        if channels == 1 {
            out[1] = out[0];
        }
        out
    }

    /// Gauge pressure along each inlet tract, Pa, throttle first, one tract after the other. The throttle's
    /// own cell shows the swing the throttle draws from, not the depth the solver draws that cell down to
    /// (see `inlet`).
    fn inlet_pressures(&self) -> Vec<f32> {
        let mut out = Vec::new();
        for inlet in self.inlets() {
            let start = out.len();
            inlet.pipe.push_cell_pressures(&mut out);
            if let Some(first) = out.get_mut(start) {
                *first = (inlet.upstream_pressure() - gas::P_AMB) as f32;
            }
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
        let launch = self.launch.as_mut().map(|d| LaunchSnapshot {
            phase: d.phase.as_str().to_string(),
            gear: (d.gear + 1) as f64,
            speed_kmh: d.speed * 3.6,
            elapsed: d.run_time(),
            distance: d.distance,
            finished: d.finished,
            zero_to_sixty: d.sixty,
            quarter_mile: d.quarter.map(|m| m.time),
            quarter_mile_kmh: d.quarter.map(|m| m.speed * 3.6),
            half_mile: d.half.map(|m| m.time),
            half_mile_kmh: d.half.map(|m| m.speed * 3.6),
            throttle: d.throttle,
            points: d.take_points(),
        });
        let (mut runner_pressure, mut runner_cells) = (Vec::new(), Vec::new());
        for r in &self.intake().runners {
            let start = runner_pressure.len();
            r.pipe.push_cell_pressures(&mut runner_pressure);
            runner_cells.push((runner_pressure.len() - start) as u32);
        }
        let snap = EngineSnapshot {
            crank_angle: first.crank_angle,
            rpm: self.rpm(),
            limiter: self.limiter_cut,
            fuel_cut: self.fuel_cut_active && !self.crackle_active,
            afterfires: (self.afterfire.events() - self.afterfires_seen) as u32,
            crackle: self.crackle_active,
            intake_cam_advance: -self.intake_shift,
            exhaust_cam_retard: self.exhaust_shift,
            short_runners: self.on_short_runners,
            high_cam: self.high_cam_spec.is_some() && self.on_high_cam,
            launch,
            cyl_pressure: first.cyl_pressure,
            cyl_temp: first.cyl_temp,
            ex_lift: first.ex_lift,
            in_lift: first.in_lift,
            torque: self.torque_last,
            pipe_pressure: self.tap_buffer.clone(),
            duct_pressure: self.duct_pressure.clone(),
            duct_cells: self.duct_cells.clone(),
            duct_ids: self.wg.duct_ids.clone(),
            inlet_pressure: self.inlet_pressures(),
            inlet_velocity: {
                let mut out = Vec::new();
                for inlet in self.inlets() {
                    inlet.pipe.push_cell_velocities(&mut out);
                }
                out
            },
            plenum_pressure: self.plenum.pressure() - gas::P_AMB,
            plenum_zones: self.plenum.zone_pressures().map(|p| p as f32).collect(),
            plenum_balanced: self.plenum.balanced(),
            runner_pressure,
            runner_cells,
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
                turbos: (0..t.count())
                    .map(|i| TurboUnitSnapshot {
                        id: self.wg.turbine_mounts()[i].id.clone(),
                        shaft_rpm: t.shaft_rpm_of(i),
                        wastegate: t.wastegate_of(i),
                        blow_off: t.blow_off_of(i),
                    })
                    .collect(),
            }),
            banks,
        };
        self.peak = 0.0;
        self.afterfires_seen = self.afterfire.events();
        snap
    }

    /// Render `out.len()` samples into `out`: in stereo, the two ears mixed.
    ///
    /// In slow motion the simulation takes `time_scale` steps per output sample, and the output
    /// is interpolated between them: the sound of a tape played slow, pitched down by as much.
    pub fn render_into(&mut self, out: &mut [f32]) {
        let stereo = self.listener.ears() == 2;
        for o in out.iter_mut() {
            let [l, r] = self.next_frame();
            *o = if stereo { ((l + r) * 0.5) as f32 } else { l as f32 };
        }
    }

    /// Render `left.len()` samples into `left` and `right`, the two ears: alike, out of stereo.
    pub fn render_stereo_into(&mut self, left: &mut [f32], right: &mut [f32]) {
        for (l, r) in left.iter_mut().zip(right.iter_mut()) {
            let frame = self.next_frame();
            *l = frame[0] as f32;
            *r = frame[1] as f32;
        }
    }

    /// The next output sample, a step of the simulation or, in slow motion, between two.
    #[inline]
    fn next_frame(&mut self) -> [f64; 2] {
        if self.time_scale >= 1.0 {
            let frame = self.tick();
            // Where slow motion picks up from, without a step, as it is played.
            self.slow_next = [frame[0] as f32 as f64, frame[1] as f32 as f64];
            return frame;
        }
        self.slow_phase += self.time_scale;
        while self.slow_phase >= 1.0 {
            self.slow_phase -= 1.0;
            self.slow_prev = self.slow_next;
            self.slow_next = self.tick();
        }
        let (a, b, t) = (self.slow_prev, self.slow_next, self.slow_phase);
        [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
    }

    /// One sample of gas dynamics and what follows from it, stepped together across the pool's
    /// threads with one hand-off. Alongside the exhaust's ducts, each cylinder's valves and then its
    /// intake runner, which reads what the valves give it, the primary's port read as it stood at the
    /// end of the last sample. Once the ducts are stepped, the rest in parts that each read only their
    /// own state and the pipes': each cylinder with its afterfire pocket, the plenum, each inlet tract
    /// or the turbo, each mouth's radiation and each chamber shell's, each part on the thread that
    /// stepped the duct it reads, whose cache holds it; the tracts and the turbo draw through the
    /// plenum, so they wait for it. What each gives is left in `bank_out`, `mouth_out`, `shell_out` and
    /// `turbo_out`, to be gathered in order. Returns the flow in through the throttles.
    fn step_sample(
        &mut self,
        dt: f64,
        run_io: RunnerIo,
        limiter_cut: bool,
        crackle: Option<CrackleSpark>,
        rpm: f64,
    ) -> f64 {
        let banks = self.cyls.len();
        let mouths = self.wg.result.mouth_flows.len();
        let shell_count = self.shells.len();
        self.bank_out.resize(banks, CachePadded::default());
        self.mouth_out.resize(mouths, CachePadded::default());
        self.shell_out.resize(shell_count, CachePadded::default());
        self.close_ns.resize(banks + mouths + shell_count, CachePadded::default());

        let intake = if self.on_short_runners { self.intake_short.as_mut().unwrap() } else { &mut self.intake_long };
        self.runner_cells.clear();
        self.runner_cells.extend(intake.runners.iter().map(|r| r.pipe.n));
        let threads = self.wg.plan(self.pool.as_deref(), self.max_threads, &self.runner_cells);
        let close_key = (self.wg.groups_id(), banks, mouths, shell_count, self.inlets.len(), self.turbo.is_some());
        if threads > 1 && close_key != self.close_key {
            self.close_key = close_key;
            if self.close_groups.len() < threads {
                self.close_groups.resize_with(threads, Vec::new);
            }
            for g in self.close_groups.iter_mut() {
                g.clear();
            }
            // Each cylinder on its primary's thread, which it reads as soon as that thread is done with it.
            for b in 0..banks {
                self.close_groups[self.wg.owner_of(b)].push(CloseItem::Bank(b));
            }
            for m in 0..mouths {
                self.close_groups[self.wg.owner_of(self.wg.radiating_duct_index(m))].push(CloseItem::Mouth(m));
            }
            for (k, shell) in self.shells.iter().enumerate() {
                self.close_groups[self.wg.owner_of(shell.duct())].push(CloseItem::Shell(k));
            }
            // The plenum first on the thread with the least else to do, and what waits for it last on
            // the next least busy, so the wait is spent on that thread's own parts.
            let load = |g: &Vec<CloseItem>| -> usize {
                g.iter().map(|item| if matches!(item, CloseItem::Bank(_)) { 4 } else { 1 }).sum()
            };
            let mut by_load: Vec<usize> = (0..threads).collect();
            by_load.sort_by_key(|&w| (load(&self.close_groups[w]), w));
            self.close_groups[by_load[0]].insert(0, CloseItem::Plenum);
            let waiting = (0..if self.turbo.is_none() { self.inlets.len() } else { 0 })
                .map(CloseItem::Inlet)
                .chain(self.turbo.is_some().then_some(CloseItem::Turbo));
            for (n, item) in waiting.enumerate() {
                self.close_groups[by_load[(n + 1) % threads]].push(item);
            }
        }
        self.close_stamp += 1;
        let stamp = self.close_stamp;
        let plenum_done = &self.plenum_done;

        let valves = ValveCtx {
            spec: &self.spec,
            lift_spec: if self.on_high_cam { &self.high_cam_spec.as_ref().unwrap().spec } else { &self.spec.spec },
            timing: &self.timing,
            delivery: &self.delivery,
            injection_offset: &self.injection_offset,
            intake_shift: self.intake_shift,
            exhaust_shift: self.exhaust_shift,
            limiter_cut,
            fuel_demand: self.fuel_demand,
            crackle,
            rpm,
            omega: self.omega,
            sample_rate: self.sample_rate,
        };
        let cyls = Disjoint::new(&mut self.cyls);
        let bank_state = Disjoint::new(&mut self.banks);
        let noise = Disjoint::new(&mut self.throat_noise);
        self.port_pressure.clear();
        self.port_pressure.extend((0..banks).map(|b| self.wg.port_pressure(b)));
        let port_pressure = &self.port_pressure;
        let step = intake.begin(&run_io, &self.plenum, &self.entry_loss);
        // The exhaust steps each side item once, and reads a valve state only after it is done.
        let job = |b: usize| unsafe {
            let bank = bank_state.get(b);
            valves.step(b, cyls.get(b), bank, &mut noise.get(b).0, port_pressure[b]);
            step.runner(b, &bank.in_valve, &bank.cyl_state);
        };
        let side = SideWork { costs: &self.runner_cells, job: &job };
        let valve_of = |b: usize| unsafe { bank_state.get_ref(b) }.valve_state;

        let cam_spec = if self.on_high_cam { self.high_cam_spec.as_ref().unwrap() } else { &self.spec };
        // Timed, each item's time is put to the duct it reads, for the threads' balance.
        let timing = threads > 1 && self.wg.timing();
        let omega = self.omega;
        let bore = throttle_dia_of(&self.spec.spec);
        let throat_noise = self.spec.spec.throat_noise;
        let (charge_p, charge_t) = (self.charge_p, self.charge_t);
        let charge_volume = self.turbo.as_ref().map(|t| t.throttle_body_volume());
        let plenum = Disjoint::new(std::slice::from_mut(&mut self.plenum));
        let plenum_out = Disjoint::new(std::slice::from_mut(&mut self.plenum_out.0));
        let close_ns = Disjoint::new(&mut self.close_ns);
        let (pockets, floor) = self.afterfire.pockets_mut();
        let pockets = Disjoint::new(pockets);
        let steepening = Disjoint::new(&mut self.steepening);
        let far_fields = Disjoint::new(&mut self.far_fields);
        let jets = Disjoint::new(&mut self.jets);
        let jet_noise = self.spec.spec.jet_noise;
        let sample_rate = self.sample_rate;
        let shells = Disjoint::new(&mut self.shells);
        let paths = self.listener.paths();
        let bank_out = Disjoint::new(&mut self.bank_out);
        let mouth_out = Disjoint::new(&mut self.mouth_out);
        let shell_out = Disjoint::new(&mut self.shell_out);
        // The tract is stepped only without a turbo: with one, the engine draws through its compressors.
        let inlets = if self.turbo.is_none() { &mut self.inlets[..] } else { &mut [] };
        let inlet_count = inlets.len();
        let inlets = Disjoint::new(inlets);
        let has_turbo = self.turbo.is_some();
        let turbo = self.turbo.as_mut().map(|t| Disjoint::new(std::slice::from_mut(t)));
        let turbo_out = Disjoint::new(std::slice::from_mut(&mut self.turbo_out));

        // Each item is run once, by the one thread it is grouped to, and touches only its own state.
        let run = |item: CloseItem, wg: &Stepped| unsafe {
            let t0 = timing.then(Instant::now);
            match item {
                CloseItem::Bank(b) => {
                    wg.wait_side(b);
                    let ex_mdot = wg.valve_mass_flow(b);
                    let runner = step.runner_ref(b);
                    let in_mdot = -runner.valve_mass_flow;
                    let primary = wg.primary(b);
                    let port_temp = primary.read_port().1;
                    let cyl = cyls.get(b);

                    let deg_per_sample = (omega.abs() * dt * 180.0) / PI;
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
                    let io = AdvanceIo {
                        dt: dt / n_sub,
                        omega,
                        ex_mdot,
                        in_mdot,
                        intake_t: runner.port_temp,
                        port_t: port_temp,
                        intake_burned: runner.inflow_burned,
                        intake_fuel: runner.inflow_fuel,
                    };
                    for _ in 0..n_sub as usize {
                        cyl.advance(cam_spec, &io);
                    }
                    *bank_out.get(b) = CachePadded(BankOut { torque: cyl.torque + cyl.inertia_torque, dpdt: cyl.dpdt });

                    // What the valve sent out unburned, in the leading cells of the primary.
                    let (fuel, air) = cyl.take_exhausted();
                    let inflow = math::max(ex_mdot, 0.0) * dt;
                    let taken = wg.heat_taken(b);
                    let (cells, volume) = wg.afterfire_zone(b);
                    let zone_mass = primary.density_at(0) * volume;
                    let hottest = || primary.leading_state(cells).1;
                    pockets.get(b).step(dt, fuel, air, inflow, zone_mass, hottest, taken, floor);

                    let bank = bank_state.get(b);
                    bank.last_valve_mdot = ex_mdot;
                    bank.prev_ex_lift = bank.ex_lift;
                    bank.prev_in_lift = bank.in_lift;
                }
                CloseItem::Plenum => {
                    // Each plenum draws from its inlet tract's throttle end, from before the tract is
                    // stepped with its throttle's flow; with a turbo, from its charge air. It reads
                    // every runner.
                    for b in 0..banks {
                        wg.wait_side(b);
                    }
                    let runners = step.runners();
                    let plenum = plenum.get(0);
                    let throttle_flow = if inlet_count > 0 {
                        let mut p_up = [gas::P_AMB; 2];
                        for (k, p) in p_up.iter_mut().enumerate().take(inlet_count) {
                            *p = inlets.get(k).upstream_pressure();
                        }
                        plenum.step(dt, &p_up[..inlet_count], gas::T_AMB, None, runners)
                    } else {
                        plenum.step(dt, &[charge_p], charge_t, charge_volume, runners)
                    };
                    let mut flows = [0.0; 2];
                    flows[..plenum.count()].copy_from_slice(plenum.throttle_flows());
                    *plenum_out.get(0) = PlenumOut {
                        throttle_flow,
                        flows,
                        area: plenum.throttle_area_each(),
                        pressure: plenum.throttle_pressure(),
                    };
                    plenum_done.0.store(stamp, Ordering::Release);
                }
                CloseItem::Inlet(k) => {
                    let p = wait_for(plenum_done, stamp, &plenum_out);
                    inlets.get(k).advance(dt, p.flows[k], p.area, bore, throat_noise);
                }
                CloseItem::Turbo => {
                    let p = wait_for(plenum_done, stamp, &plenum_out);
                    if let Some(turbo) = &turbo {
                        let step = turbo.get(0).step(dt, wg.turbines(), p.throttle_flow, p.pressure);
                        *turbo_out.get(0) = CachePadded(Some(step));
                    }
                }
                CloseItem::Mouth(m) => {
                    let duct = wg.radiating_duct(m);
                    let (_, t, a) = duct.read_mouth();
                    let flow = wg.mouth_flow(m);
                    let q = steepening.get(m).process(flow, a, t);
                    let jet = jets.get(m).process(flow, duct.mouth_area(), t, jet_noise, sample_rate);
                    *mouth_out.get(m) = CachePadded(paths.process(m, far_fields.get(m).process(q) + jet));
                }
                CloseItem::Shell(k) => {
                    let shell = shells.get(k);
                    *shell_out.get(k) = CachePadded(shell.process(wg.duct(shell.duct())));
                }
            }
            if let Some(t0) = t0 {
                let slot = match item {
                    CloseItem::Bank(b) => Some(b),
                    CloseItem::Mouth(m) => Some(banks + m),
                    CloseItem::Shell(k) => Some(banks + mouths + k),
                    CloseItem::Plenum | CloseItem::Inlet(_) | CloseItem::Turbo => None,
                };
                if let Some(slot) = slot {
                    close_ns.get(slot).0 = t0.elapsed().as_nanos() as f64;
                }
            }
        };
        let groups = &self.close_groups;
        let after = |w: usize, wg: &Stepped| {
            if threads > 1 {
                for &item in &groups[w] {
                    run(item, wg);
                }
                return;
            }
            for b in 0..banks {
                run(CloseItem::Bank(b), wg);
            }
            run(CloseItem::Plenum, wg);
            for k in 0..inlet_count {
                run(CloseItem::Inlet(k), wg);
            }
            if has_turbo {
                run(CloseItem::Turbo, wg);
            }
            for m in 0..mouths {
                run(CloseItem::Mouth(m), wg);
            }
            for k in 0..shell_count {
                run(CloseItem::Shell(k), wg);
            }
        };
        self.wg.advance_planned(dt, &valve_of, self.pool.as_deref(), Some(&side), Some(&after));

        if timing {
            for b in 0..banks {
                self.wg.note_time(b, self.close_ns[b].0);
            }
            for m in 0..mouths {
                self.wg.note_time(self.wg.radiating_duct_index(m), self.close_ns[banks + m].0);
            }
            for k in 0..shell_count {
                self.wg.note_time(self.shells[k].duct(), self.close_ns[banks + mouths + k].0);
            }
        }
        self.plenum_out.0.throttle_flow
    }

    /// Step the exhaust's ducts and the intake runners across `pool`'s threads, or with `None` on the
    /// caller's alone. The sound is the same to the bit either way; only how long it takes changes.
    pub fn set_pool(&mut self, pool: Option<Arc<ThreadPool>>) {
        self.pool = pool;
    }

    /// The most of the pool's threads this engine can use: enough cells of pipe for each to be worth
    /// it. More pipe, more threads.
    pub fn useful_threads(&self) -> usize {
        let Some(pool) = &self.pool else { return 1 };
        let runners: Vec<usize> = self.intake().runners.iter().map(|r| r.pipe.n).collect();
        self.wg.useful_threads(&runners, pool.threads())
    }

    /// Let the solver spend `scale` times the budget a browser can afford on finer cells, which
    /// rebuilds the exhaust if that changes them.
    pub fn set_budget_scale(&mut self, scale: f64) {
        let scale = if scale.is_finite() { scale.max(0.1) } else { 1.0 };
        if scale != self.budget_scale {
            self.budget_scale = scale;
            self.rebuild_exhaust();
        }
    }

    /// Use at most `threads` of the pool's threads, the caller's included: fewer can be faster, as each
    /// costs a hand-off per step. The sound is the same whatever the count.
    pub fn set_max_threads(&mut self, threads: usize) {
        self.max_threads = threads.max(1);
    }

    /// Run at `scale` of real time, 0..1: 1 is real time, 0.01 a hundred times slower. The
    /// simulation itself is unchanged; it is only stepped less often.
    pub fn set_time_scale(&mut self, scale: f64) {
        let scale = if scale.is_finite() { scale.clamp(1e-4, 1.0) } else { 1.0 };
        if scale < 1.0 && self.time_scale >= 1.0 {
            // Entering slow motion: hold the last sample played until the next is simulated.
            self.slow_phase = 0.0;
            self.slow_prev = self.slow_next;
        }
        self.time_scale = scale;
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
fn budgeted_cell_size(
    spec: &EngineSpec,
    graph: &ExhaustGraph,
    sample_rate: f64,
    wg_options: &EulerPipeOptions,
    budget_scale: f64,
) -> f64 {
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
    let budget_for = scaled_budget_cells(spec.cylinders as usize, node_order(graph).len(), budget_scale) - runner_cells;

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
#[allow(clippy::too_many_arguments)]
fn build_options_for(
    spec: &EngineSpec,
    pipe: &[PipeSegment],
    collector: &[PipeSegment],
    stored: Option<&ExhaustGraph>,
    sample_rate: f64,
    wg_options: &EulerPipeOptions,
    budget_scale: f64,
    inherit: Option<Vec<f64>>,
) -> EulerPipeOptions {
    let graph = usable_graph(spec, pipe, collector, stored);
    let base = EulerPipeOptions {
        cell_size: Some(budgeted_cell_size(spec, &graph, sample_rate, wg_options, budget_scale)),
        single_step: Some(true),
        wall_thickness: Some(spec.pipe_wall_thickness),
        material: Some(spec.pipe_material.wall()),
        air_speed: Some(spec.air_speed),
        inherit_wall: inherit,
        ..Default::default()
    };
    let mut opts = base.overlaid(wg_options);
    opts.port =
        Some(wg_options.port.unwrap_or(HeadPort { length: spec.port_length, diameter: exhaust_port_diameter(spec) }));
    opts
}

#[allow(clippy::too_many_arguments)]
fn build_exhaust_for(
    spec: &EngineSpec,
    pipe: &[PipeSegment],
    collector: &[PipeSegment],
    stored: Option<&ExhaustGraph>,
    sample_rate: f64,
    wg_options: &EulerPipeOptions,
    budget_scale: f64,
    inherit: Option<Vec<f64>>,
) -> ExhaustSystem {
    let graph = usable_graph(spec, pipe, collector, stored);
    let opts = build_options_for(spec, pipe, collector, stored, sample_rate, wg_options, budget_scale, inherit);
    let mut sys = ExhaustSystem::new(&graph, spec.cylinders as usize, sample_rate, spec.port_gas_temp, &opts)
        .expect("a compiled or validated graph is solvable");
    sys.set_turbulence(spec.throat_noise);
    sys
}

/// `graph` as far as building the solver goes: without where its pipes and turbos are drawn and which
/// way they point, which only the scene reads, nor the turbos' own settings, which resize them in place.
fn solver_view(graph: &ExhaustGraph) -> ExhaustGraph {
    let mut g = graph.clone();
    for d in g.ducts.iter_mut() {
        d.heading_yaw = None;
        d.heading_pitch = None;
        d.heading_frame = None;
        for s in d.segments.iter_mut() {
            s.yaw = 0.0;
            s.pitch = 0.0;
        }
    }
    for t in g.turbos.iter_mut() {
        t.position = None;
        t.rotation = None;
        t.settings = None;
    }
    g
}

/// True if the crank swept past `target` degrees between two samples.
#[inline]
fn crossed_angle(from: f64, to: f64, target: f64) -> bool {
    if to >= from {
        return target > from && target <= to;
    }
    target > from || target <= to
}
