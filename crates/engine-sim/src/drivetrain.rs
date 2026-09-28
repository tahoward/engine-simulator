//! A dyno run: the engine pulling a car on an inertia chassis dyno, at full throttle, through a
//! six-speed gearbox, up to the shift point in every gear.
//!
//! The engine drives the gearbox through a friction clutch whose torque follows the slip speed
//! through a `tanh`, up to its capacity. What the run measures is crank torque averaged over each
//! complete engine cycle, as a real dyno reports it.

use crate::math::{self, PI};
use crate::spec::DynoConfig;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DynoPhase {
    Pull,
    ShiftOut,
    ShiftIn,
    Cooldown,
}

impl DynoPhase {
    pub fn as_str(self) -> &'static str {
        match self {
            DynoPhase::Pull => "pull",
            DynoPhase::ShiftOut => "shiftOut",
            DynoPhase::ShiftIn => "shiftIn",
            DynoPhase::Cooldown => "cooldown",
        }
    }
}

/// Values per recorded point: rpm, crank torque (N*m), road speed (km/h), gear (1-6), volumetric
/// efficiency (a fraction) and intake manifold pressure (bar, absolute).
pub const DYNO_POINT_STRIDE: usize = 6;

/// Points held between snapshots.
const POINT_CAPACITY: usize = 256;

const SHIFT_OUT: f64 = 0.12;
const CLUTCH_OUT: f64 = 0.04;
const SHIFT_IN: f64 = 0.3;
const SETTLE: f64 = 0.1;
const CLUTCH_CAPACITY: f64 = 2.5;
const CLUTCH_SLIP: f64 = 3.0;
const ROLLING_RESISTANCE: f64 = 0.012;
const DRIVELINE_EFFICIENCY: f64 = 0.9;
const G: f64 = 9.81;
const MAX_RUN: f64 = 150.0;
const STALL_TIME: f64 = 4.0;

pub struct DynoRun {
    pub config: DynoConfig,
    pub phase: DynoPhase,
    /// Gear engaged, 0-based.
    pub gear: usize,
    /// Road speed, m/s.
    pub speed: f64,
    /// Throttle opening the run commands, 0..1.
    pub throttle: f64,
    /// Clutch engagement, 0..1.
    pub clutch: f64,
    pub elapsed: f64,
    pub phase_time: f64,
    pub finished: bool,
    /// The engine's volumetric efficiency as of the last intake valve closings, set before each step.
    pub volumetric_efficiency: f64,
    /// Absolute pressure in the intake manifold, Pa, set before each step.
    pub intake_pressure: f64,
    points: Vec<f32>,
    point_count: usize,
    capacity: f64,
    last_angle: f64,
    cycle_torque: f64,
    cycle_omega: f64,
    cycle_time: f64,
    cycle_intake: f64,
    cycle_valid: bool,
    best_speed: f64,
    best_speed_at: f64,
}

impl DynoRun {
    /// `omega` is the engine's speed as the run starts, rad/s; `full_load_torque` sizes the clutch.
    pub fn new(config: DynoConfig, omega: f64, full_load_torque: f64) -> DynoRun {
        let mut run = DynoRun {
            config,
            phase: DynoPhase::Pull,
            gear: 0,
            speed: 0.0,
            throttle: 1.0,
            clutch: 1.0,
            elapsed: 0.0,
            phase_time: 0.0,
            finished: false,
            volumetric_efficiency: 0.0,
            intake_pressure: 0.0,
            points: vec![0.0; POINT_CAPACITY * DYNO_POINT_STRIDE],
            point_count: 0,
            capacity: CLUTCH_CAPACITY * full_load_torque,
            last_angle: -1.0,
            cycle_torque: 0.0,
            cycle_omega: 0.0,
            cycle_time: 0.0,
            cycle_intake: 0.0,
            cycle_valid: false,
            best_speed: 0.0,
            best_speed_at: 0.0,
        };
        run.speed = (omega * run.config.tyre_radius) / run.overall(0);
        run
    }

    fn ratio(&self, g: usize) -> f64 {
        self.config.ratios.get(g).copied().unwrap_or(f64::NAN)
    }

    /// Overall ratio, engine turns per wheel turn, in gear `g`.
    fn overall(&self, g: usize) -> f64 {
        self.ratio(g) * self.config.final_drive
    }

    /// Advance by `dt`. Returns the torque the clutch takes off the crank, N*m.
    pub fn step(&mut self, dt: f64, omega: f64, crank_torque: f64, angle: f64) -> f64 {
        self.elapsed += dt;
        self.phase_time += dt;
        self.sequence(omega);

        let ratio = self.overall(self.gear);
        let input_omega = (self.speed * ratio) / self.config.tyre_radius;
        let clutch_torque = self.capacity * self.clutch * math::tanh((omega - input_omega) / CLUTCH_SLIP);

        let mass = self.config.mass;
        let drive = (clutch_torque * ratio * DRIVELINE_EFFICIENCY) / self.config.tyre_radius;
        let rolling = if self.speed > 0.0 { ROLLING_RESISTANCE * mass * G } else { 0.0 };
        self.speed = math::max(self.speed + ((drive - rolling) / mass) * dt, 0.0);

        self.record(dt, omega, crank_torque, angle);
        clutch_torque
    }

    /// End the run: throttle shut, clutch out. The results so far stand.
    pub fn finish(&mut self) {
        if self.phase == DynoPhase::Cooldown {
            return;
        }
        self.enter(DynoPhase::Cooldown);
        self.throttle = 0.0;
        self.clutch = 0.0;
        self.finished = true;
        self.cycle_valid = false;
    }

    /// Hand over the points recorded since the last call, and forget them.
    pub fn take_points(&mut self) -> Vec<f32> {
        let out = self.points[..self.point_count * DYNO_POINT_STRIDE].to_vec();
        self.point_count = 0;
        out
    }

    fn enter(&mut self, phase: DynoPhase) {
        self.phase = phase;
        self.phase_time = 0.0;
        self.cycle_valid = false;
    }

    fn sequence(&mut self, omega: f64) {
        let rpm = (omega * 60.0) / (2.0 * PI);
        match self.phase {
            DynoPhase::Pull => {
                self.throttle = 1.0;
                self.clutch = 1.0;
                if self.speed > self.best_speed + 0.05 {
                    self.best_speed = self.speed;
                    self.best_speed_at = self.elapsed;
                }
                if rpm >= self.config.shift_rpm {
                    if self.gear + 1 < self.config.ratios.len() {
                        self.enter(DynoPhase::ShiftOut);
                    } else {
                        self.finish();
                    }
                } else if self.elapsed > MAX_RUN || self.elapsed - self.best_speed_at > STALL_TIME {
                    self.finish();
                }
            }
            DynoPhase::ShiftOut => {
                self.throttle = 0.0;
                self.clutch = math::max(1.0 - self.phase_time / CLUTCH_OUT, 0.0);
                if self.phase_time >= SHIFT_OUT {
                    self.gear += 1;
                    self.enter(DynoPhase::ShiftIn);
                }
            }
            DynoPhase::ShiftIn => {
                let u = math::min(self.phase_time / SHIFT_IN, 1.0);
                let smooth = u * u * (3.0 - 2.0 * u);
                self.clutch = smooth;
                self.throttle = smooth;
                if u >= 1.0 {
                    self.enter(DynoPhase::Pull);
                }
            }
            DynoPhase::Cooldown => {
                self.throttle = 0.0;
                self.clutch = 0.0;
            }
        }
    }

    /// Accumulate the engine cycle in progress, and record it once it completes cleanly.
    fn record(&mut self, dt: f64, omega: f64, crank_torque: f64, angle: f64) {
        let wrapped = self.last_angle >= 0.0 && angle < self.last_angle;
        self.last_angle = angle;
        if wrapped {
            if self.cycle_valid && self.cycle_time > 0.0 && self.point_count < POINT_CAPACITY {
                let base = self.point_count * DYNO_POINT_STRIDE;
                let mean_omega = self.cycle_omega / self.cycle_time;
                self.points[base] = ((mean_omega * 60.0) / (2.0 * PI)) as f32;
                self.points[base + 1] = (self.cycle_torque / self.cycle_time) as f32;
                self.points[base + 2] = (self.speed * 3.6) as f32;
                self.points[base + 3] = (self.gear + 1) as f32;
                self.points[base + 4] = self.volumetric_efficiency as f32;
                self.points[base + 5] = (self.cycle_intake / self.cycle_time / 1e5) as f32;
                self.point_count += 1;
            }
            self.cycle_torque = 0.0;
            self.cycle_omega = 0.0;
            self.cycle_time = 0.0;
            self.cycle_intake = 0.0;
            self.cycle_valid = self.phase == DynoPhase::Pull && self.phase_time >= SETTLE;
        }
        self.cycle_torque += crank_torque * dt;
        self.cycle_omega += omega * dt;
        self.cycle_time += dt;
        self.cycle_intake += self.intake_pressure * dt;
    }
}
