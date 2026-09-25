/**
 * In-cylinder thermodynamics: a single control volume whose boundary moves with the
 * piston, filled and emptied through the valves.
 *
 * Mass and temperature are the state; pressure follows from the ideal gas law. This
 * is the standard "filling and emptying" engine model. It is what makes the exhaust
 * pulse the right shape: the pressure at exhaust-valve-opening is a consequence of
 * the compression ratio, the spark timing, the burn rate and how much charge
 * actually got trapped, not a number anyone typed in.
 */

import {
  CV_REF,
  CV_SLOPE,
  GAS,
  T_REF,
  type EngineSpec,
  crankAt,
  cylinderVolume,
  displacement,
  makeCrankState,
} from '../../model/spec.js';
import { Noise, clamp, cycleDelta, windowPhase, wrapCycle } from './dsp.js';

/**
 * `gasEnergy`, `gasEnthalpy` and `gasTemperature` from the spec, written out for the hot path: they
 * are called from functions too large to inline them, where each float argument and result would be
 * boxed into a fresh heap object.
 */
const HALF_SLOPE = 0.5 * CV_SLOPE;
const CV_REF_SQ = CV_REF * CV_REF;
const TWO_SLOPE = 2 * CV_SLOPE;

export class Cylinder {
  /** Crank angle, deg in [0, 720). 0 = TDC firing. */
  angle = 0;
  /**
   * Trapped gas mass, kg.
   *
   * Initialised here, like every numeric field, although the constructor sets it: a field declared
   * without a value starts as `undefined`, V8 then stores it as a generic tagged value, and every later
   * write allocates. `mass` and `energy` are written every substep.
   */
  mass = 0;
  /**
   * Total sensible internal energy of the trapped gas, J: `m * gasEnergy(T)`.
   *
   * This is the state variable, not temperature, and that choice matters. Integrating
   * temperature directly means dividing the energy increment by `m*cv`, and during the
   * exhaust stroke the two largest terms in that increment — compression work `-p dV/dt`
   * and outflow enthalpy `-mdot R T` — very nearly cancel. Analytically they cancel exactly
   * at constant pressure, which is why temperature should hold steady while gas is pushed
   * out. Numerically, the small residual of that cancellation would be divided by a mass shrinking
   * toward the residual, so it would blow up: at part throttle the temperature would run to its
   * clamp several hundred times a second right at exhaust valve closing, with the cylinder
   * drained to 0.5% of a charge.
   *
   * Integrating energy puts the cancellation between two terms of the same size and leaves
   * the division by mass to the point where temperature is *read*, where mass and energy
   * shrink together and the ratio stays well behaved.
   */
  private energy = 0;

  /** 0..1 mass-fraction burned for the current cycle. */
  burned = 0;
  /** Total heat release latched for the current cycle, J. */
  private qCycle = 0;
  /**
   * Unburned charge currently trapped, kg — the part that can still release heat.
   *
   * Tracked rather than inferred. Inferring it as `mass - massAtEvc`, the mass admitted
   * between exhaust valve closing and intake valve closing, silently assumes everything
   * that comes down the intake is clean air. With a finite plenum for an intake that
   * assumption breaks: some of what is re-inducted is the cylinder's own exhaust,
   * pushed up the port during overlap and handed back. Carrying the composition explicitly
   * is what makes residual dilution — and so the load dependence of exhaust temperature —
   * come out of the bookkeeping instead of being assumed away.
   */
  private freshMass = 0;
  /** Fresh charge trapped at intake valve closing, kg. What the flame has to consume. */
  private freshAtIvc = 0;
  /** Whether combustion has been armed for the cycle about to fire. */
  private armed = false;
  /**
   * Set by the rev limiter. Read when the charge is committed at intake valve closing, so a cycle
   * is either fired whole or cut whole; a cut charge goes down the exhaust unburned.
   */
  sparkCut = false;
  /**
   * This cylinder's cam timing offset from nominal, crank degrees. Its valves open and close
   * this much late, so its charge is committed this much late too.
   */
  camOffset = 0;

  /** Instantaneous gas torque at the crank, N*m. Updated by `advance`. */
  torque = 0;
  /** Reciprocating inertia torque at the crank, N*m. Zero-mean over a cycle. */
  inertiaTorque = 0;
  /** Heat lost to the walls this step, W. Exposed for diagnostics. */
  wallHeatFlux = 0;
  /** Times the gas temperature had to be clamped. Should stay zero. */
  clampHits = 0;
  /**
   * Rate of cylinder pressure rise, Pa/s. This is what shakes the block, so it drives
   * the structural noise in `engineSim`.
   */
  dpdt = 0;

  /** Pressure, temperature and burned fraction after the last step, at the `CYL_*` slots. */
  readonly endState = new Float64Array(CYL_STATE_SIZE);

  /** Scratch for `crankAt`, so the per-substep path allocates nothing. */
  private readonly crank = makeCrankState();
  /** Where this substep ends, degrees: handed to the combustion helpers in a field, not an argument. */
  private nextAngle = 0.5;

  /** Per-cycle combustion scatter, redrawn at each intake valve closing. */
  private burnScale = 1;
  private ignitionOffset = 0;
  private readonly noise: Noise;

  constructor(spec: EngineSpec, angle = 0, seed = 0x51f3a7) {
    this.noise = new Noise(seed);
    this.angle = wrapCycle(angle);
    this.mass = (GAS.pAmb * cylinderVolume(spec, this.angle)) / (GAS.R * 700);
    // Start as all burned gas. Seeded fresh instead, the very first cycle would fire an
    // unrealistically large heat release before the bookkeeping settles.
    this.freshMass = 0;
    this.energy = this.mass * energyAt(700);
  }

  /** Fraction of the trapped charge that is spent gas, 0..1. */
  get burnedFraction(): number {
    return clamp(1 - this.freshMass / Math.max(this.mass, MIN_MASS), 0, 1);
  }

  /** Bulk gas temperature, K, derived from energy and mass. */
  get temp(): number {
    const u = this.energy / this.mass;
    return clamp(T_REF + (2 * u) / (CV_REF + Math.sqrt(CV_REF_SQ + TWO_SLOPE * u)), 150, 6000);
  }

  set temp(value: number) {
    this.energy = this.mass * energyAt(value);
  }

  volume(spec: EngineSpec): number {
    return cylinderVolume(spec, this.angle);
  }

  /** Absolute pressure, Pa. */
  pressure(spec: EngineSpec): number {
    return (this.mass * GAS.R * this.temp) / cylinderVolume(spec, this.angle);
  }

  /**
   * `pressure`, `temp` and `burnedFraction`, written to `out` from `base` at the `CYL_*` slots.
   *
   * For the engine loop, which is too big to inline the getters: each would box its result. This
   * gets inlined into it in turn, so it is written out with no floating-point calls of its own; the
   * `clamp`s are spelled as ternaries, and the volume comes from `crankAt`, which returns an object
   * and computes it with the same operations as `cylinderVolume` (its square-root floor is `1e-12`
   * rather than `0`, which no real rod-to-crank ratio comes near).
   */
  readState(spec: EngineSpec, out: Float64Array, base: number): void {
    const b = 1 - this.freshMass / Math.max(this.mass, MIN_MASS);
    const burned = b < 0 ? 0 : b > 1 ? 1 : b;
    const u = this.energy / this.mass;
    const raw = T_REF + (2 * u) / (CV_REF + Math.sqrt(CV_REF_SQ + TWO_SLOPE * u));
    const temp = raw < 150 ? 150 : raw > 6000 ? 6000 : raw;
    this.crank.angle = this.angle;
    const volume = crankAt(spec, this.crank).volume;
    out[base + CYL_PRESSURE] = (this.mass * GAS.R * temp) / volume;
    out[base + CYL_TEMP] = temp;
    out[base + CYL_BURNED] = burned;
  }

  /**
   * Advance the gas state by `dt` seconds at crank speed `omega` (rad/s).
   *
   * @param exMdot kg/s, positive out of the cylinder into the exhaust port.
   * @param inMdot kg/s, positive into the cylinder from the intake plenum.
   * @param intakeTemp K, temperature of the charge arriving through the intake.
   * @param portTemp K, temperature of the gas in the exhaust port, carried in by reverse flow.
   * @param intakeBurned 0..1, burned fraction of what arrives through the intake.
   */
  advance(
    spec: EngineSpec,
    dt: number,
    omega: number,
    exMdot: number,
    inMdot: number,
    intakeTemp: number,
    portTemp: number,
    intakeBurned = 0,
  ): void {
    const io = ADVANCE_IO;
    io[IO_DT] = dt;
    io[IO_OMEGA] = omega;
    io[IO_EX] = exMdot;
    io[IO_IN] = inMdot;
    io[IO_INTAKE_T] = intakeTemp;
    io[IO_PORT_T] = portTemp;
    io[IO_INTAKE_BURNED] = intakeBurned;
    this.advanceIo(spec, io);
  }

  /**
   * `advance`, with its inputs read from `io` at the `IO_*` slots.
   *
   * What the engine's inner loop calls. The step is too large to inline, and a floating-point argument
   * to a call that is not inlined is boxed into a fresh heap object: seven of them per call, once per
   * cylinder per substep. A typed array holds them unboxed, so nothing floating-point crosses the call.
   */
  advanceIo(spec: EngineSpec, io: Float64Array): void {
    const dt = io[IO_DT]!;
    const omega = io[IO_OMEGA]!;
    const exMdot = io[IO_EX]!;
    const inMdot = io[IO_IN]!;
    const intakeTemp = io[IO_INTAKE_T]!;
    const portTemp = io[IO_PORT_T]!;
    const intakeBurned = io[IO_INTAKE_BURNED]!;
    const dTheta = (omega * dt * 180) / Math.PI;
    const nextAngle = wrapCycle(this.angle + dTheta);
    this.nextAngle = nextAngle;

    const tNow = this.temp;
    // One trig evaluation for volume, dV/dtheta and both piston derivatives.
    this.crank.angle = this.angle;
    const k = crankAt(spec, this.crank);
    const v = k.volume;
    const p = (this.mass * GAS.R * tNow) / v;
    const dVdTheta = k.dVolume;
    const dVdt = dVdTheta * omega;
    const pBefore = p;
    const xd = k.dPosition;
    const xdd = k.d2Position;
    this.stepPressure = p;
    this.stepVolume = v;
    this.stepTemp = tNow;

    // --- Combustion ---------------------------------------------------------
    this.updateCombustionLatches(spec);
    const dQcomb = this.heatRelease(spec);

    // --- Wall heat transfer (Woschni) ---------------------------------------
    const dQwall = this.woschni(spec, p, v, omega, tNow) * dt;
    this.wallHeatFlux = dQwall / dt;

    const dQ = dQcomb + dQwall;

    // --- Energy balance -----------------------------------------------------
    // dU/dt = dQ/dt - p dV/dt + (enthalpy in) - (enthalpy out)
    //
    // Each valve is handled by direction, because the gas that comes *back* through an
    // exhaust valve during overlap arrives at port temperature, not at cylinder
    // temperature. Treating it as the latter would quietly import heat that was never there.
    // Every stream carries the enthalpy `gasEnthalpy(T)` of its own temperature.
    const dOwn = tNow - T_REF;
    const hOwn = dOwn * (CV_REF + HALF_SLOPE * dOwn) + GAS.R * tNow;
    const dPort = portTemp - T_REF;
    const dIntake = intakeTemp - T_REF;
    const hEx =
      exMdot >= 0 ? exMdot * hOwn : exMdot * (dPort * (CV_REF + HALF_SLOPE * dPort) + GAS.R * portTemp);
    const hIn =
      inMdot >= 0
        ? inMdot * (dIntake * (CV_REF + HALF_SLOPE * dIntake) + GAS.R * intakeTemp)
        : inMdot * hOwn;
    const dU = dQ + (-p * dVdt - hEx + hIn) * dt;

    const dm = (inMdot - exMdot) * dt;
    const massBefore = this.mass;
    this.mass = Math.max(this.mass + dm, MIN_MASS);
    this.energy = this.energy + dU;

    // --- Composition --------------------------------------------------------
    // Four flows, and each carries a different mixture:
    //   intake in    -> whatever the plenum holds, which is not necessarily clean air
    //   intake out   -> the cylinder's own mixture, going back up the runner
    //   exhaust out  -> the cylinder's own mixture
    //   exhaust in   -> spent gas from the port, so no fresh charge at all
    const freshFrac = this.freshMass / Math.max(massBefore, MIN_MASS);
    let dFresh = 0;
    dFresh += (inMdot >= 0 ? inMdot * (1 - intakeBurned) : inMdot * freshFrac) * dt;
    dFresh -= (exMdot >= 0 ? exMdot * freshFrac : 0) * dt;
    this.freshMass = clamp(this.freshMass + dFresh, 0, this.mass);

    // Floor the internal energy so the derived temperature stays admissible. Counted, not
    // silent: in normal running it must never fire.
    const minEnergy = this.mass * ENERGY_AT_FLOOR;
    if (this.energy < minEnergy) {
      this.energy = minEnergy;
      this.clampHits++;
    }

    // Gas torque: dW = (p - p_crankcase) dV, so torque = dW/dtheta.
    this.torque = (p - GAS.pAmb) * dVdTheta;

    // Reciprocating inertia torque. Power balance gives the torque needed to
    // accelerate the mass as m*a*(dx/dtheta); what the mass exerts on the crank is
    // the negative of that. It integrates to zero over a cycle — energy is stored and
    // returned, not produced — so it cannot change the mean speed, only the evenness.
    this.inertiaTorque = -spec.recipMass * omega * omega * xdd * xd;

    this.angle = nextAngle;

    // Pressure rise rate, evaluated across the step just taken.
    this.crank.angle = this.angle;
    const vAfter = crankAt(spec, this.crank).volume;
    const tAfter = this.temp;
    const pAfter = (this.mass * GAS.R * tAfter) / vAfter;
    this.dpdt = (pAfter - pBefore) / dt;
    // The state `readState` would give now, so the next substep need not recompute the crank.
    const endState = this.endState;
    endState[CYL_PRESSURE] = pAfter;
    endState[CYL_TEMP] = tAfter;
    const bAfter = 1 - this.freshMass / Math.max(this.mass, MIN_MASS);
    endState[CYL_BURNED] = bAfter < 0 ? 0 : bAfter > 1 ? 1 : bAfter;

    // Woschni's motored pressure, compressed isentropically along with the volume from the
    // reference state: dp/p = -gamma dV/V over the step just taken, at the gamma of the motored
    // gas's own temperature `p V / (m R)`.
    const tMotored = (this.motoredPressure * v) / this.refMassR;
    const gMotored = 1 + GAS.R / (CV_REF + CV_SLOPE * (tMotored - T_REF));
    this.motoredPressure *= 1 - (gMotored * (vAfter - v)) / (0.5 * (v + vAfter));
  }

  /**
   * Latch per-cycle quantities at the moments they are physically determined:
   * residual mass when the exhaust valve shuts, and total fuel energy when the
   * intake valve shuts and the charge is committed.
   */
  private updateCombustionLatches(spec: EngineSpec): void {
    const nextAngle = this.nextAngle;
    // Valve events for this cylinder's cam timing, wrapped once rather than every substep.
    if (spec !== this.eventSpec || this.camOffset !== this.eventOffset) {
      this.eventSpec = spec;
      this.eventOffset = this.camOffset;
      this.evoAt = wrapCycle(spec.evo + this.camOffset);
      this.ivcAt = wrapCycle(spec.ivc + this.camOffset);
      this.exchanging = windowPhase(this.angle, spec.evo + this.camOffset, spec.ivc + this.camOffset) >= 0;
    }
    if (crossed(this.angle, nextAngle, this.evoAt)) this.exchanging = true;
    if (crossed(this.angle, nextAngle, this.ivcAt)) {
      this.exchanging = false;
      // Woschni's reference state: the charge as the valve traps it, from which the motored
      // pressure is extrapolated for the rest of the closed period.
      this.refPressure = this.stepPressure;
      this.motoredPressure = this.stepPressure;
      this.refMassR = (this.stepPressure * this.stepVolume) / this.stepTemp;
      // Woschni's `C2 Vd Tr / (pr Vr)`, m/(s Pa), fixed until the next intake valve closing.
      this.combustionVelocity =
        (WOSCHNI_C2 * displacement(spec) * this.stepTemp) / (this.stepPressure * this.stepVolume);

      // Only the fresh charge carries fuel. Residual burned gas does not, so a
      // throttled engine — which traps less fresh air and proportionally more
      // residual — releases less heat. That is the load mechanism, and it falls
      // out of the mass bookkeeping rather than being applied as a fudge factor.
      const fresh = this.freshMass;

      // --- Cycle-to-cycle combustion scatter -----------------------------------
      // Drawn here, at the moment the charge is committed, and held for the whole
      // cycle. Scatter grows as the trapped charge gets thinner: a small, dilute
      // charge burns slowly and is far more sensitive to whatever turbulence happens
      // to sit at the spark gap, which is why an engine idles unevenly and pulls
      // smoothly under load. Calibrated to roughly 2% CoV of indicated work at full
      // load rising past 10% near idle, matching published single-cylinder data.
      //
      // Keyed to fresh charge mass. Residual fraction is the more direct cause, and the
      // model does track it (`burnedFraction`), but fresh fill is the quantity this
      // correlation was calibrated against.
      const fullCharge = (GAS.pAmb * displacement(spec)) / (GAS.R * GAS.tAmb);
      const freshFill = clamp(fresh / Math.max(fullCharge, 1e-12), 0.06, 1.2);
      // Capped. Uncapped this reaches 0.33 at light load, and the Gaussian tails on a
      // spread that wide regularly produce absurd cycles — a burn three times faster than
      // commanded, or ignition 30 crank degrees early — which spike the temperature into
      // its clamp several hundred times a second. The coefficient of variation would be in
      // the right band but arrive through violent outliers rather than ordinary scatter.
      const scatter = Math.min(
        spec.combustionVariability * (0.016 + 0.019 / freshFill),
        MAX_SCATTER,
      );

      // Burn rate and heat release both scatter, and they are correlated: a cycle
      // whose kernel grows slowly also tends to burn less completely.
      const shared = this.noise.gaussian();
      // Bounds tight enough that a three-sigma draw stays a plausible cycle.
      this.burnScale = clamp(1 + shared * scatter, 0.55, 2.0);
      const qScale = clamp(1 + shared * scatter * 0.45 + this.noise.gaussian() * scatter * 0.3, 0.3, 1.25);
      // Ignition delay: the kernel takes a variable number of crank degrees to become
      // a self-sustaining flame front. Because it shifts the whole pulse in *time*
      // rather than just scaling it, it is what keeps the rhythm from being metronomic.
      // Measured ignition-delay scatter is a couple of crank degrees at full load and
      // several near idle; at the default variability this lands around 0.8 deg and
      // 3.5 deg (one sigma) respectively.
      this.ignitionOffset = clamp(this.noise.gaussian() * scatter * 22, -14, 14);

      this.qCycle = fresh * GAS.chargeEnergy * COMBUSTION_EFFICIENCY * qScale;

      // --- Dilution limit ---------------------------------------------------------
      // Spent gas absorbs heat and carries no fuel, so the more of it the charge holds the slower the
      // flame and the weaker the kernel — until, past some limit, the kernel does not survive at all.
      // The burn stretches as the limit is approached and cycles start to misfire outright; that
      // misfire, and the partial burns either side of it, are the lope of a big cam at idle. It
      // regulates itself as a real one does: a misfired charge goes out unburned, so what the next
      // cycle keeps of it is fuel rather than spent gas, and that cycle fires hard.
      //
      // Only past the onset, and only there does it draw a random number, so an engine below it
      // draws exactly the noise sequence it would with no dilution limit at all.
      const residual = 1 - fresh / Math.max(this.mass, MIN_MASS);
      if (residual > DILUTION_ONSET) {
        const x = Math.min((residual - DILUTION_ONSET) / (DILUTION_FULL - DILUTION_ONSET), 1);
        this.burnScale *= 1 + DILUTION_BURN_STRETCH * x;
        const u = (this.noise.next() + 1) / 2;
        if (u < x * x) this.qCycle = 0;
      }

      this.freshAtIvc = fresh;
      this.burned = 0;
      // The scatter is still drawn on a cut cycle, so the limiter does not shift the noise
      // sequence of every cycle after it.
      this.armed = !this.sparkCut;
    }
  }

  /** Wiebe-function heat release for this step, J. */
  private heatRelease(spec: EngineSpec): number {
    const nextAngle = this.nextAngle;
    if (!this.armed || this.qCycle <= 0) return 0;
    const spark = spec.ignition + this.ignitionOffset;
    const duration = Math.max(spec.burnDuration * this.burnScale, 4);
    const from = wiebe(cycleDelta(this.angle, spark), duration);
    const to = wiebe(cycleDelta(nextAngle, spark), duration);
    const d = to - from;
    if (d <= 0) return 0;
    this.burned = to;
    // The flame turns fresh charge into spent gas. Without this the cylinder would still
    // read as full of unburned mixture after combustion, and anything it pushed back up
    // the intake would arrive at the plenum as clean air.
    this.freshMass = Math.max(this.freshMass - d * this.freshAtIvc, 0);
    if (to >= 0.999) this.armed = false;
    return d * this.qCycle;
  }

  /**
   * Woschni correlation for the convective heat transfer coefficient, then
   * multiplied by the instantaneous chamber surface area. Without this the
   * compression peak is too high and the expansion too hot, and the engine sounds
   * hollow and over-bright.
   *
   * The characteristic gas velocity is Woschni's in full:
   *
   *   w = C1 * Sp + C2 * (Vd * Tr / (pr * Vr)) * (p - p_motored)
   *
   * with C1 = 6.18 while the valves exchange gas and 2.28 while they are shut, and the second
   * term, the turbulence the flame itself stirs up, only while they are shut. `r` is the state
   * at intake valve closing and `p_motored` that state compressed isentropically to the
   * present volume.
   *
   * @returns heat flow into the gas, W (negative while the gas is hotter than the wall).
   */
  private woschni(spec: EngineSpec, p: number, v: number, omega: number, temp: number): number {
    /**
     * Two `Math.pow` calls per step, not four.
     *
     * This runs per cylinder per cylinder-substep, so at 8500 rpm on a V8 it is a noticeable share of
     * runtime, and of four calls half would be raising constants to fixed powers. `bore^-0.2` depends
     * only on the spec, and `w^0.8` only on crank speed, which does not change within a sample; both are
     * cached and invalidated on identity, the same trick `crankGeometry` uses. The two that remain
     * genuinely vary with the gas state.
     */
    if (spec !== this.woschniSpec) {
      this.woschniSpec = spec;
      this.woschniBore = Math.pow(spec.bore, -0.2);
      // Force the speed term to be recomputed too: it is scaled by stroke.
      this.woschniOmega = NaN;
    }
    if (omega !== this.woschniOmega) {
      this.woschniOmega = omega;
      this.woschniPistonSpeed = (Math.abs(omega) / (2 * Math.PI)) * 2 * spec.stroke;
      this.woschniSpeedExchange = Math.pow(WOSCHNI_C1_EXCHANGE * this.woschniPistonSpeed, 0.8);
      this.woschniSpeedClosed = Math.pow(WOSCHNI_C1_CLOSED * this.woschniPistonSpeed, 0.8);
    }
    // `(p w)^0.8`: with the speed term cached where it is constant, and raised together with the
    // pressure where the combustion term makes it vary, so it costs one power either way.
    let pw: number;
    const rise = p - this.motoredPressure;
    if (this.exchanging) {
      pw = Math.pow(p / 1000, 0.8) * this.woschniSpeedExchange;
    } else if (this.refPressure > 0 && rise > 0) {
      const w = WOSCHNI_C1_CLOSED * this.woschniPistonSpeed + this.combustionVelocity * rise;
      pw = Math.pow((p / 1000) * w, 0.8);
    } else {
      pw = Math.pow(p / 1000, 0.8) * this.woschniSpeedClosed;
    }
    const h = 3.26 * this.woschniBore * pw * Math.pow(temp, -0.55);

    // Head + piston crown + the exposed liner for the current volume.
    const bore = spec.bore;
    const crossSection = (Math.PI * bore * bore) / 4;
    const height = v / crossSection;
    const area = 2 * crossSection + Math.PI * bore * height;

    return -h * area * (temp - GAS.tWall);
  }

  /** Cached pieces of the Woschni correlation that do not vary with the gas state. */
  private woschniSpec: EngineSpec | null = null;
  private woschniBore = 0;
  private woschniOmega = NaN;
  private woschniPistonSpeed = 0;
  private woschniSpeedExchange = 0;
  private woschniSpeedClosed = 0;
  /** State at the last intake valve closing: Woschni's reference for the motored pressure. */
  private refPressure = 0;
  /** `m R` of the charge as trapped, J/K: `p V / T` at the reference state. */
  private refMassR = 1;
  /** Woschni's combustion-term coefficient from that state, m/(s Pa). */
  private combustionVelocity = 0;
  /** Valve events at this cylinder's cam timing, and the spec and offset they were wrapped for. */
  private eventSpec: EngineSpec | null = null;
  private eventOffset = 0;
  private evoAt = 0;
  private ivcAt = 0;
  /** Whether a valve is exchanging gas: between exhaust valve opening and intake valve closing. */
  private exchanging = false;
  /** The reference state compressed isentropically to the present volume, Pa. */
  private motoredPressure = 0;
  /** The state at the start of the current step, handed to the latches in fields. */
  private stepPressure = 0;
  private stepVolume = 0;
  private stepTemp = 0;
}

/** Woschni's velocity coefficients: m/s per m/s of mean piston speed, and m/(s*K). */
const WOSCHNI_C1_EXCHANGE = 6.18;
const WOSCHNI_C1_CLOSED = 2.28;
const WOSCHNI_C2 = 3.24e-3;

/** Slots of the array `advanceIo` reads its inputs from. */
export const IO_DT = 0;
export const IO_OMEGA = 1;
export const IO_EX = 2;
export const IO_IN = 3;
export const IO_INTAKE_T = 4;
export const IO_PORT_T = 5;
export const IO_INTAKE_BURNED = 6;
export const ADVANCE_IO_SIZE = 7;
/** Scratch for the plain `advance`. */
const ADVANCE_IO = new Float64Array(ADVANCE_IO_SIZE);

/** Slots `readState` writes, from its base. */
export const CYL_PRESSURE = 0;
export const CYL_TEMP = 1;
export const CYL_BURNED = 2;
export const CYL_STATE_SIZE = 3;

const COMBUSTION_EFFICIENCY = 0.92;

/**
 * Floor on trapped mass, kg. Roughly the residual in the clearance volume at a few mbar —
 * small enough never to bind in normal running, large enough to keep `energy/mass` sane.
 */
const MIN_MASS = 2e-7;

/** Ceiling on the per-cycle combustion scatter. See the note where it is applied. */
const MAX_SCATTER = 0.16;

/**
 * Spent-gas fraction of the trapped charge, at intake valve closing, beyond which the flame starts to
 * fail, and the fraction at which it always does. See where they are applied.
 *
 * A spark-ignition engine's combustion goes unstable somewhere past 25-30% total dilution and misfires
 * regularly past about 40%. So the onset is at 0.4: a stock engine idles below it (the crossplane V8 on
 * 25% at 900 rpm, the single on 30%), gets there only as it throttles down toward a stall, and an
 * overcammed one, idling on 60-70%, sits well past it. At 90% nothing lights.
 */
const DILUTION_ONSET = 0.4;
const DILUTION_FULL = 0.9;

/** How much longer the burn takes at a fully diluted charge, as a multiple of the normal duration. */
const DILUTION_BURN_STRETCH = 2;

/**
 * Wiebe mass-fraction-burned. `a = 5`, `m = 2` gives the usual slow-then-fast-then-slow
 * S-curve. Normalised so it reaches exactly 1 at the end of the stated duration: the raw
 * curve only reaches 1 - e^-5, and the remaining 0.7% would otherwise be released in a
 * single step.
 */
export function wiebe(degAfterSpark: number, duration: number): number {
  if (degAfterSpark <= 0) return 0;
  if (degAfterSpark >= duration) return 1;
  const u = degAfterSpark / duration;
  return (1 - Math.exp(-5 * u * u * u)) * WIEBE_NORM;
}

const WIEBE_NORM = 1 / (1 - Math.exp(-5));

/** `gasEnergy`, J/kg. Off the hot path. */
function energyAt(t: number): number {
  const d = t - T_REF;
  return d * (CV_REF + HALF_SLOPE * d);
}

/** Sensible energy at the temperature floor, J/kg. */
const ENERGY_AT_FLOOR = energyAt(150);

/** True if the crank swept past `target` between `from` and `to` (720-deg wrapping). */
function crossed(from: number, to: number, target: number): boolean {
  if (to >= from) return target > from && target <= to;
  // Wrapped through 0/720.
  return target > from || target <= to;
}
