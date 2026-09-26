/**
 * A dyno run: the engine pulling a car on an inertia chassis dyno, at full throttle, through a
 * six-speed gearbox, from wherever it is running up to the shift point in every gear.
 *
 * The car sits on rollers, so what the engine drives is the car's mass (plus the rollers', counted
 * in it) and its tyres' rolling resistance, with no air drag. Every gear, sixth included, therefore
 * pulls all the way to the shift point, and a run always finishes.
 *
 * The engine drives the gearbox through a friction clutch that slips smoothly: its torque follows the
 * slip speed through a `tanh`, up to its capacity. Engaged, the slip is a fraction of a radian per
 * second and the engine and car turn together; during a shift the capacity is ramped out and back in,
 * so the engine's speed is pulled down to the next gear's as a real clutch pulls it, with no jolt.
 *
 * What the run measures is crank torque: gas torque less friction, averaged over each complete engine
 * cycle. That is how a real dyno reports it, averaging away the firing pulses, and it is the same
 * torque whichever gear the engine is in.
 */

import type { DynoConfig } from '../../model/spec.js';

export type DynoPhase = 'pull' | 'shiftOut' | 'shiftIn' | 'cooldown';

/**
 * Values per recorded point in `DynoRun.points`: rpm, crank torque (N*m), road speed (km/h), gear (1-6)
 * and volumetric efficiency (a fraction).
 */
export const DYNO_POINT_STRIDE = 5;

/** Points held between snapshots. A snapshot takes them at 60 Hz; a V8 at 8000 rpm makes 67 a second. */
const POINT_CAPACITY = 256;

/** Seconds with the throttle shut and the clutch coming out, before the next gear goes in. */
const SHIFT_OUT = 0.12;
/** Of that, how long the clutch takes to come fully out. */
const CLUTCH_OUT = 0.04;
/** Seconds for the clutch and the throttle to come back in on the next gear. */
const SHIFT_IN = 0.3;
/** Seconds into a pull before cycles are recorded, so a shift's transient is not. */
const SETTLE = 0.1;

/**
 * Clutch capacity, as a multiple of the engine's full-throttle torque. Production clutches carry 1.3
 * to 2 times the engine's torque; more here keeps the engaged slip small against the firing pulses.
 */
const CLUTCH_CAPACITY = 2.5;
/** Slip speed at which the clutch reaches three quarters of its capacity, rad/s. */
const CLUTCH_SLIP = 3;

/** Tyre rolling resistance coefficient on the rollers. */
const ROLLING_RESISTANCE = 0.012;
/** Drivetrain efficiency, gearbox and final drive, from the clutch to the rollers. */
const DRIVELINE_EFFICIENCY = 0.9;
const G = 9.81;

/** Longest a run may last, s: past it the engine has stopped pulling the car and the run ends. */
const MAX_RUN = 150;
/** Seconds without a new top speed in a pull after which the run ends: the engine can pull no further. */
const STALL_TIME = 4;

export class DynoRun {
  phase: DynoPhase = 'pull';
  /** Gear engaged, 0-based. */
  gear = 0;
  /** Road speed, m/s. */
  speed = 0;
  /** Throttle opening the run commands, 0..1. */
  throttle = 1;
  /** Clutch engagement, 0..1, as a fraction of its capacity. */
  clutch = 1;
  /** Seconds since the run started, and since the current phase started. */
  elapsed = 0;
  phaseTime = 0;
  /** Whether the run has finished its last pull, or been stopped. */
  finished = false;
  /**
   * The engine's volumetric efficiency as of the last intake valve closings, set by the engine before
   * each `step`: fresh charge trapped, averaged over the cylinders, as a fraction of a cylinder's swept
   * volume at ambient density. Recorded with each engine cycle.
   */
  volumetricEfficiency = 0;

  /** Recorded points not yet taken by a snapshot, `DYNO_POINT_STRIDE` values each. */
  readonly points = new Float32Array(POINT_CAPACITY * DYNO_POINT_STRIDE);
  pointCount = 0;

  private readonly capacity: number;
  private lastAngle = -1;
  private cycleTorque = 0;
  private cycleOmega = 0;
  private cycleTime = 0;
  private cycleValid = false;
  private bestSpeed = 0;
  private bestSpeedAt = 0;

  /**
   * @param omega The engine's speed as the run starts, rad/s. The car starts rolling in first at the
   *   speed that matches it, so the run pulls from wherever the engine already is.
   * @param fullLoadTorque The engine's nominal full-throttle torque, N*m, which sizes the clutch.
   */
  constructor(
    readonly config: DynoConfig,
    omega: number,
    fullLoadTorque: number,
  ) {
    this.capacity = CLUTCH_CAPACITY * fullLoadTorque;
    this.speed = (omega * config.tyreRadius) / this.overall(0);
  }

  /** Overall ratio, engine turns per wheel turn, in gear `g`. */
  private overall(g: number): number {
    return this.config.ratios[g]! * this.config.finalDrive;
  }

  /**
   * Advance by `dt`.
   *
   * @param omega Engine speed, rad/s.
   * @param crankTorque Gas torque less friction at the crank this sample, N*m.
   * @param angle Crank angle of cylinder 1, deg, which marks out the engine cycles.
   * @returns The torque the clutch takes off the crank, N*m.
   */
  step(dt: number, omega: number, crankTorque: number, angle: number): number {
    this.elapsed += dt;
    this.phaseTime += dt;
    this.sequence(omega);

    const ratio = this.overall(this.gear);
    const inputOmega = (this.speed * ratio) / this.config.tyreRadius;
    const clutchTorque = this.capacity * this.clutch * Math.tanh((omega - inputOmega) / CLUTCH_SLIP);

    const mass = this.config.mass;
    const drive = (clutchTorque * ratio * DRIVELINE_EFFICIENCY) / this.config.tyreRadius;
    const rolling = this.speed > 0 ? ROLLING_RESISTANCE * mass * G : 0;
    this.speed = Math.max(this.speed + ((drive - rolling) / mass) * dt, 0);

    this.record(dt, omega, crankTorque, angle);
    return clutchTorque;
  }

  /** End the run: throttle shut, clutch out. The results so far stand. */
  finish(): void {
    if (this.phase === 'cooldown') return;
    this.enter('cooldown');
    this.throttle = 0;
    this.clutch = 0;
    this.finished = true;
    this.cycleValid = false;
  }

  /** Hand over the points recorded since the last call, and forget them. */
  takePoints(): Float32Array {
    const out = this.points.slice(0, this.pointCount * DYNO_POINT_STRIDE);
    this.pointCount = 0;
    return out;
  }

  private enter(phase: DynoPhase): void {
    this.phase = phase;
    this.phaseTime = 0;
    // A cycle that straddles a change of phase is not a clean pull.
    this.cycleValid = false;
  }

  /** The shift sequence: pull to the shift point, lift and declutch, next gear, back in. */
  private sequence(omega: number): void {
    const rpm = (omega * 60) / (2 * Math.PI);
    switch (this.phase) {
      case 'pull':
        this.throttle = 1;
        this.clutch = 1;
        if (this.speed > this.bestSpeed + 0.05) {
          this.bestSpeed = this.speed;
          this.bestSpeedAt = this.elapsed;
        }
        if (rpm >= this.config.shiftRpm) {
          if (this.gear < this.config.ratios.length - 1) this.enter('shiftOut');
          else this.finish();
        } else if (this.elapsed > MAX_RUN || this.elapsed - this.bestSpeedAt > STALL_TIME) {
          this.finish();
        }
        break;
      case 'shiftOut':
        this.throttle = 0;
        this.clutch = Math.max(1 - this.phaseTime / CLUTCH_OUT, 0);
        if (this.phaseTime >= SHIFT_OUT) {
          this.gear++;
          this.enter('shiftIn');
        }
        break;
      case 'shiftIn': {
        const u = Math.min(this.phaseTime / SHIFT_IN, 1);
        const smooth = u * u * (3 - 2 * u);
        this.clutch = smooth;
        this.throttle = smooth;
        if (u >= 1) this.enter('pull');
        break;
      }
      case 'cooldown':
        this.throttle = 0;
        this.clutch = 0;
        break;
    }
  }

  /** Accumulate the engine cycle in progress, and record it once it completes cleanly. */
  private record(dt: number, omega: number, crankTorque: number, angle: number): void {
    // A cycle ends each time cylinder 1 passes firing top dead centre.
    const wrapped = this.lastAngle >= 0 && angle < this.lastAngle;
    this.lastAngle = angle;
    if (wrapped) {
      if (this.cycleValid && this.cycleTime > 0 && this.pointCount < POINT_CAPACITY) {
        const base = this.pointCount * DYNO_POINT_STRIDE;
        const meanOmega = this.cycleOmega / this.cycleTime;
        this.points[base] = (meanOmega * 60) / (2 * Math.PI);
        this.points[base + 1] = this.cycleTorque / this.cycleTime;
        this.points[base + 2] = this.speed * 3.6;
        this.points[base + 3] = this.gear + 1;
        this.points[base + 4] = this.volumetricEfficiency;
        this.pointCount++;
      }
      this.cycleTorque = 0;
      this.cycleOmega = 0;
      this.cycleTime = 0;
      this.cycleValid = this.phase === 'pull' && this.phaseTime >= SETTLE;
    }
    this.cycleTorque += crankTorque * dt;
    this.cycleOmega += omega * dt;
    this.cycleTime += dt;
  }
}
