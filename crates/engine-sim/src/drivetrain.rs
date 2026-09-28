//! A launch: the engine pulling a car from standstill down a strip, at full throttle, through its
//! gearbox, up to the shift point in every gear.
//!
//! The engine drives the gearbox through a friction clutch whose torque follows the slip speed
//! through a `tanh`, up to its capacity. Off the line the clutch is slipped to hold the engine at the
//! launch speed until the car has caught up with it. The driven wheels are a body of their own, tied
//! to the road by a tyre whose grip also follows its slip through a `tanh`, so an engine with more
//! torque than the tyres can take spins them up. The car works against rolling resistance and air drag.
//!
//! What the run measures is crank torque averaged over each complete engine cycle, as a dyno reports
//! it, and the time from moving off to 60 mph, the quarter mile and the half mile, as a timeslip does.

use crate::math::{self, PI};
use crate::spec::LaunchConfig;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LaunchPhase {
    Launch,
    Pull,
    ShiftOut,
    ShiftIn,
    Cooldown,
}

impl LaunchPhase {
    pub fn as_str(self) -> &'static str {
        match self {
            LaunchPhase::Launch => "launch",
            LaunchPhase::Pull => "pull",
            LaunchPhase::ShiftOut => "shiftOut",
            LaunchPhase::ShiftIn => "shiftIn",
            LaunchPhase::Cooldown => "cooldown",
        }
    }
}

/// Values per recorded point: rpm, crank torque (N*m), road speed (km/h), gear (1-based), volumetric
/// efficiency (a fraction) and intake manifold pressure (bar, absolute).
pub const LAUNCH_POINT_STRIDE: usize = 6;

/// Points held between snapshots.
const POINT_CAPACITY: usize = 256;

const SHIFT_OUT: f64 = 0.12;
const CLUTCH_OUT: f64 = 0.04;
const SHIFT_IN: f64 = 0.3;
const SETTLE: f64 = 0.1;
const CLUTCH_CAPACITY: f64 = 2.5;
const CLUTCH_SLIP: f64 = 3.0;
/// Engine speed above the launch speed at which the clutch is fully in, rev/min: the band the slipping
/// clutch holds the engine in off the line.
const LAUNCH_WINDOW: f64 = 400.0;
/// Longest the clutch may slip off the line before the run gives up, s.
const LAUNCH_TIMEOUT: f64 = 10.0;
const ROLLING_RESISTANCE: f64 = 0.012;
const DRIVELINE_EFFICIENCY: f64 = 0.9;
/// Air density, kg/m^3, and the car's drag area, Cd times frontal area, m^2.
const AIR_DENSITY: f64 = 1.2;
const DRAG_AREA: f64 = 0.6;
/// The driven wheels, tyres and half-shafts, about their axle, kg*m^2.
const WHEEL_INERTIA: f64 = 3.0;
/// Tyre friction coefficient, and the share of the car's weight on the driven wheels as it pulls away.
const GRIP: f64 = 1.1;
const DRIVEN_LOAD: f64 = 0.6;
/// Slip speed over which the tyre builds its grip, m/s: a fixed part, and a part in proportion to speed.
const TYRE_SLIP: f64 = 0.4;
const TYRE_SLIP_RATIO: f64 = 0.06;
/// Drivetrain steps per audio sample: the light wheels on a stiff clutch need a finer step than the crank.
const SUBSTEPS: usize = 4;
const G: f64 = 9.81;
const MAX_RUN: f64 = 150.0;
const STALL_TIME: f64 = 4.0;

/// 60 mph, m/s, and a quarter and a half mile, m.
const SIXTY_MPH: f64 = 26.8224;
const QUARTER_MILE: f64 = 402.336;
const HALF_MILE: f64 = 804.672;

/// When the car reached a distance, s after moving off, and how fast it was going there, m/s.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mark {
    pub time: f64,
    pub speed: f64,
}

pub struct LaunchRun {
    pub config: LaunchConfig,
    pub phase: LaunchPhase,
    /// Gear engaged, 0-based.
    pub gear: usize,
    /// Road speed, m/s.
    pub speed: f64,
    /// Surface speed of the driven tyres, m/s: above `speed` while they spin.
    pub wheel_speed: f64,
    /// Distance covered, m.
    pub distance: f64,
    /// Throttle opening the run commands, 0..1.
    pub throttle: f64,
    /// Clutch engagement, 0..1.
    pub clutch: f64,
    pub elapsed: f64,
    pub phase_time: f64,
    pub finished: bool,
    /// The run's time when the car first moved, s.
    pub moved_at: Option<f64>,
    /// Seconds from moving off to 60 mph.
    pub sixty: Option<f64>,
    pub quarter: Option<Mark>,
    pub half: Option<Mark>,
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

impl LaunchRun {
    /// A car at rest, in first. `full_load_torque` sizes the clutch.
    pub fn new(config: LaunchConfig, full_load_torque: f64) -> LaunchRun {
        LaunchRun {
            config,
            phase: LaunchPhase::Launch,
            gear: 0,
            speed: 0.0,
            wheel_speed: 0.0,
            distance: 0.0,
            throttle: 1.0,
            clutch: 0.0,
            elapsed: 0.0,
            phase_time: 0.0,
            finished: false,
            moved_at: None,
            sixty: None,
            quarter: None,
            half: None,
            volumetric_efficiency: 0.0,
            intake_pressure: 0.0,
            points: vec![0.0; POINT_CAPACITY * LAUNCH_POINT_STRIDE],
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
        }
    }

    fn ratio(&self, g: usize) -> f64 {
        self.config.ratios.get(g).copied().unwrap_or(f64::NAN)
    }

    /// Overall ratio, engine turns per wheel turn, in gear `g`.
    fn overall(&self, g: usize) -> f64 {
        self.ratio(g) * self.config.final_drive
    }

    /// Seconds since the car moved off, or 0 before it has.
    pub fn run_time(&self) -> f64 {
        self.moved_at.map_or(0.0, |t| self.elapsed - t)
    }

    /// Advance by `dt`. Returns the torque the clutch takes off the crank, N*m.
    pub fn step(&mut self, dt: f64, omega: f64, crank_torque: f64, angle: f64) -> f64 {
        self.elapsed += dt;
        self.phase_time += dt;
        self.sequence(omega);

        let ratio = self.overall(self.gear);
        let radius = self.config.tyre_radius;
        let mass = self.config.mass;
        let wheel_mass = WHEEL_INERTIA / (radius * radius);
        let grip = GRIP * DRIVEN_LOAD * mass * G;
        let h = dt / SUBSTEPS as f64;
        let mut clutch_sum = 0.0;
        for _ in 0..SUBSTEPS {
            let input_omega = (self.wheel_speed * ratio) / radius;
            let clutch_torque = self.capacity * self.clutch * math::tanh((omega - input_omega) / CLUTCH_SLIP);
            let drive = (clutch_torque * ratio * DRIVELINE_EFFICIENCY) / radius;
            let slip = self.wheel_speed - self.speed;
            let tyre = grip * math::tanh(slip / (TYRE_SLIP + TYRE_SLIP_RATIO * self.speed));
            let resistance = if self.speed > 0.0 {
                ROLLING_RESISTANCE * mass * G + 0.5 * AIR_DENSITY * DRAG_AREA * self.speed * self.speed
            } else {
                0.0
            };
            self.wheel_speed = math::max(self.wheel_speed + ((drive - tyre) / wheel_mass) * h, 0.0);
            self.speed = math::max(self.speed + ((tyre - resistance) / mass) * h, 0.0);
            self.distance += self.speed * h;
            clutch_sum += clutch_torque;
        }

        self.time_marks();
        self.record(dt, omega, crank_torque, angle);
        clutch_sum / SUBSTEPS as f64
    }

    /// End the run: throttle shut, clutch out. The results so far stand.
    pub fn finish(&mut self) {
        if self.phase == LaunchPhase::Cooldown {
            return;
        }
        self.enter(LaunchPhase::Cooldown);
        self.throttle = 0.0;
        self.clutch = 0.0;
        self.finished = true;
        self.cycle_valid = false;
    }

    /// Hand over the points recorded since the last call, and forget them.
    pub fn take_points(&mut self) -> Vec<f32> {
        let out = self.points[..self.point_count * LAUNCH_POINT_STRIDE].to_vec();
        self.point_count = 0;
        out
    }

    fn enter(&mut self, phase: LaunchPhase) {
        self.phase = phase;
        self.phase_time = 0.0;
        self.cycle_valid = false;
    }

    fn sequence(&mut self, omega: f64) {
        let rpm = (omega * 60.0) / (2.0 * PI);
        match self.phase {
            LaunchPhase::Launch => {
                // Full throttle, the clutch slipped to hold the engine just above the launch speed.
                self.throttle = 1.0;
                self.clutch = math::min(math::max((rpm - self.config.launch_rpm) / LAUNCH_WINDOW, 0.0), 1.0);
                let input_omega = (self.wheel_speed * self.overall(self.gear)) / self.config.tyre_radius;
                if self.speed > 0.0 && omega - input_omega < CLUTCH_SLIP {
                    self.enter(LaunchPhase::Pull);
                } else if self.phase_time > LAUNCH_TIMEOUT {
                    self.finish();
                }
            }
            LaunchPhase::Pull => {
                self.throttle = 1.0;
                self.clutch = 1.0;
                if self.speed > self.best_speed + 0.05 {
                    self.best_speed = self.speed;
                    self.best_speed_at = self.elapsed;
                }
                if rpm >= self.config.shift_rpm {
                    if self.gear + 1 < self.config.ratios.len() {
                        self.enter(LaunchPhase::ShiftOut);
                    } else {
                        self.finish();
                    }
                } else if self.elapsed > MAX_RUN || self.elapsed - self.best_speed_at > STALL_TIME {
                    self.finish();
                }
            }
            LaunchPhase::ShiftOut => {
                self.throttle = 0.0;
                self.clutch = math::max(1.0 - self.phase_time / CLUTCH_OUT, 0.0);
                if self.phase_time >= SHIFT_OUT {
                    self.gear += 1;
                    self.enter(LaunchPhase::ShiftIn);
                }
            }
            LaunchPhase::ShiftIn => {
                let u = math::min(self.phase_time / SHIFT_IN, 1.0);
                let smooth = u * u * (3.0 - 2.0 * u);
                self.clutch = smooth;
                self.throttle = smooth;
                if u >= 1.0 {
                    self.enter(LaunchPhase::Pull);
                }
            }
            LaunchPhase::Cooldown => {
                self.throttle = 0.0;
                self.clutch = 0.0;
            }
        }
    }

    /// Note the moment the car moves off, and when it passes 60 mph, the quarter mile and the half mile.
    fn time_marks(&mut self) {
        if self.moved_at.is_none() && self.speed > 0.0 {
            self.moved_at = Some(self.elapsed);
        }
        if self.moved_at.is_none() || self.finished {
            return;
        }
        let time = self.run_time();
        if self.sixty.is_none() && self.speed >= SIXTY_MPH {
            self.sixty = Some(time);
        }
        if self.quarter.is_none() && self.distance >= QUARTER_MILE {
            self.quarter = Some(Mark { time, speed: self.speed });
        }
        if self.half.is_none() && self.distance >= HALF_MILE {
            self.half = Some(Mark { time, speed: self.speed });
        }
    }

    /// Accumulate the engine cycle in progress, and record it once it completes cleanly.
    fn record(&mut self, dt: f64, omega: f64, crank_torque: f64, angle: f64) {
        let wrapped = self.last_angle >= 0.0 && angle < self.last_angle;
        self.last_angle = angle;
        if wrapped {
            if self.cycle_valid && self.cycle_time > 0.0 && self.point_count < POINT_CAPACITY {
                let base = self.point_count * LAUNCH_POINT_STRIDE;
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
            self.cycle_valid = self.phase == LaunchPhase::Pull && self.phase_time >= SETTLE;
        }
        self.cycle_torque += crank_torque * dt;
        self.cycle_omega += omega * dt;
        self.cycle_time += dt;
        self.cycle_intake += self.intake_pressure * dt;
    }
}
