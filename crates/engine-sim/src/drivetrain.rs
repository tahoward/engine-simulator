//! A launch: the engine pulling a car from standstill down a strip, at full throttle, through its
//! gearbox, up to the shift point in every gear.
//!
//! The engine drives the gearbox through a friction clutch whose torque follows the slip speed
//! through a `tanh`, up to its capacity. Off the line the clutch is slipped to hold the engine at the
//! launch speed until the car has caught up with it. The driven wheels are a body of their own, tied
//! to the road by a tyre whose grip follows its slip on Pacejka's magic formula: it rises to a peak,
//! what the weight on the driven wheels allows, then falls away as the tyre slides, to about four
//! fifths of that spinning freely. That weight grows as the car accelerates and its weight moves back onto the
//! rear wheels. An engine with more torque than the tyres can take spins them up, unless traction control
//! steps in, as a launch control and a dual-clutch gearbox's torque management do. While the clutch slips,
//! off the line and through each shift, it passes no more torque than the tyres can take at their peak;
//! off the line the spark is cut to hold the engine at the launch speed, and through a shift while the
//! engine outruns the gearbox. In gear, the spark is cut while the tyres slip past their peak. The car
//! works against rolling resistance and air drag.
//!
//! A manual shift lifts off and takes the clutch out, then brings both back in the next gear. A
//! dual-clutch gearbox hands the drive from one clutch to the other with no gap, the new gear's clutch
//! slipping the engine down to its speed.
//!
//! What the run measures is crank torque averaged over each complete engine cycle, as a dyno reports
//! it, and the time to 60 mph, the quarter mile and the half mile, as a timeslip does: from the moment
//! the car has rolled a foot, where a drag strip's clock starts and where American road tests start theirs.

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

/// Shares of a shift's time: from lifting off to the next gear going in, of which the clutch takes the
/// first part to come out; and the clutch and throttle coming back in.
const SHIFT_OUT: f64 = 0.3;
const CLUTCH_OUT: f64 = 0.1;
const SHIFT_IN: f64 = 0.7;
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
/// Height of the centre of gravity over the wheelbase: how much of the car's weight moves onto the rear
/// wheels for each g it accelerates at.
const WEIGHT_TRANSFER: f64 = 0.18;
/// Traction control: the share of the tyres' peak grip a slipping clutch passes, a little under it so the
/// tyres hold just short of the peak rather than sliding past it; how far the engine may outrun the
/// gearbox through a shift before the spark is cut, rad/s; and the tyre slip, in units of `TYRE_SLIP`,
/// past which the spark is cut in gear: the magic formula's peak.
const TC_GRIP: f64 = 0.98;
const TC_SYNC: f64 = 10.0;
const TC_SLIP: f64 = 2.0;
/// Slip speed over which the tyre builds its grip, m/s: a fixed part, and a part in proportion to speed.
/// It grips hardest at twice this, about 12% slip at speed.
const TYRE_SLIP: f64 = 0.4;
const TYRE_SLIP_RATIO: f64 = 0.06;
/// The magic formula's shape and stiffness, `sin(C atan(B x))` of the slip in units of `TYRE_SLIP`: a
/// peak at `x = 2`, and `sin(C pi / 2)`, 81% of it, sliding.
const TYRE_SHAPE: f64 = 1.4;
const TYRE_STIFFNESS: f64 = 1.035;
/// Drivetrain steps per audio sample: the light wheels on a stiff clutch need a finer step than the crank.
const SUBSTEPS: usize = 4;
const G: f64 = 9.81;
const MAX_RUN: f64 = 150.0;
const STALL_TIME: f64 = 4.0;

/// How far the car rolls before the clock starts, m: a foot.
const ROLLOUT: f64 = 0.3048;
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
    /// Whether traction control, or the launch control, is cutting the spark.
    pub spark_cut: bool,
    /// The car's acceleration, m/s^2: how much weight it has moved onto the rear wheels.
    pub accel: f64,
    /// Clutch engagement, 0..1.
    pub clutch: f64,
    pub elapsed: f64,
    pub phase_time: f64,
    pub finished: bool,
    /// The run's time when the car had rolled out its first foot, and the clock started, s.
    pub moved_at: Option<f64>,
    /// Seconds from the clock starting to 60 mph.
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
            spark_cut: false,
            accel: 0.0,
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

    /// Seconds since the clock started, or 0 before it has.
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
        let h = dt / SUBSTEPS as f64;
        let traction = self.config.traction_control && !self.finished;
        // A clutch meant to slip, off the line or through a shift, is held to what the tyres can take.
        let managed = traction && matches!(self.phase, LaunchPhase::Launch | LaunchPhase::ShiftIn);
        let mut clutch_sum = 0.0;
        let mut capped = false;
        for _ in 0..SUBSTEPS {
            let load = math::min(self.config.driven_load + (WEIGHT_TRANSFER * math::max(self.accel, 0.0)) / G, 1.0);
            let grip = self.config.tyre_grip * load * mass * G;
            let input_omega = (self.wheel_speed * ratio) / radius;
            let mut clutch_torque = self.capacity * self.clutch * math::tanh((omega - input_omega) / CLUTCH_SLIP);
            if managed {
                // The most torque the tyres can take at the crank, through this gear.
                let limit = (TC_GRIP * grip * radius) / (ratio * DRIVELINE_EFFICIENCY);
                if clutch_torque > limit {
                    clutch_torque = limit;
                    capped = true;
                }
            }
            let drive = (clutch_torque * ratio * DRIVELINE_EFFICIENCY) / radius;
            let slip = self.wheel_speed - self.speed;
            let tyre = grip * tyre_curve(slip / (TYRE_SLIP + TYRE_SLIP_RATIO * self.speed));
            let resistance = if self.speed > 0.0 {
                ROLLING_RESISTANCE * mass * G + 0.5 * AIR_DENSITY * DRAG_AREA * self.speed * self.speed
            } else {
                0.0
            };
            self.wheel_speed = math::max(self.wheel_speed + ((drive - tyre) / wheel_mass) * h, 0.0);
            self.accel = (tyre - resistance) / mass;
            self.speed = math::max(self.speed + self.accel * h, 0.0);
            self.distance += self.speed * h;
            clutch_sum += clutch_torque;
        }

        self.spark_cut = traction && self.cuts_spark(omega, ratio, capped);
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
                    if self.gear + 1 < self.config.ratios.len() && self.config.dual_clutch {
                        // The next gear's clutch takes the drive as the last one lets it go.
                        self.gear += 1;
                        self.enter(LaunchPhase::ShiftIn);
                    } else if self.gear + 1 < self.config.ratios.len() {
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
                let shift = self.config.shift_time;
                self.clutch = math::max(1.0 - self.phase_time / (CLUTCH_OUT * shift), 0.0);
                if self.phase_time >= SHIFT_OUT * shift {
                    self.gear += 1;
                    self.enter(LaunchPhase::ShiftIn);
                }
            }
            LaunchPhase::ShiftIn => {
                let u = math::min(self.phase_time / (SHIFT_IN * self.config.shift_time), 1.0);
                let smooth = u * u * (3.0 - 2.0 * u);
                // A dual clutch keeps the drive through the shift: the new gear's clutch is in from the
                // start, slipping the engine down to its speed, and the throttle stays open.
                let hold = if self.config.dual_clutch { 1.0 } else { smooth };
                self.clutch = hold;
                self.throttle = hold;
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

    /// Whether traction control cuts the spark: off the line, above the launch speed, as a launch
    /// control holds it there; through a shift, while the clutch is at the tyres' limit and the engine
    /// outruns the gearbox, so it comes down to the next gear's speed rather than flaring; and in gear while
    /// the tyres slip past their peak.
    fn cuts_spark(&self, omega: f64, ratio: f64, capped: bool) -> bool {
        match self.phase {
            LaunchPhase::Launch => (omega * 60.0) / (2.0 * PI) > self.config.launch_rpm + LAUNCH_WINDOW,
            LaunchPhase::ShiftIn => capped && omega - (self.wheel_speed * ratio) / self.config.tyre_radius > TC_SYNC,
            LaunchPhase::Pull => self.wheel_speed - self.speed > TC_SLIP * (TYRE_SLIP + TYRE_SLIP_RATIO * self.speed),
            LaunchPhase::ShiftOut | LaunchPhase::Cooldown => false,
        }
    }

    /// Note the moment the car has rolled out a foot, and when it passes 60 mph, the quarter mile and the
    /// half mile.
    fn time_marks(&mut self) {
        if self.moved_at.is_none() && self.distance >= ROLLOUT {
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

/// The tyre's grip at slip `x`, in units of the slip over which it builds, as a share of its peak.
fn tyre_curve(x: f64) -> f64 {
    math::sin(TYRE_SHAPE * math::atan(TYRE_STIFFNESS * x))
}
