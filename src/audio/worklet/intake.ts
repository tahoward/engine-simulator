/**
 * The intake runners: one duct per cylinder, from the plenum to the intake valve, solved with the
 * same quasi-1D gas dynamics as the exhaust.
 *
 * A runner is what lets an engine fill past what the plenum's pressure alone could push in. The
 * falling piston sets the column of air in the runner moving, and its momentum keeps it flowing
 * into the cylinder after bottom dead centre, against a pressure already above the plenum's; the
 * pressure waves the valve's opening and closing send up the runner reflect off the plenum and arrive
 * back in step with it at some speeds and out of step at others. Without the runner an engine closing
 * its intake valve 70 degrees after bottom dead centre pushes charge back out, and none of the
 * torque peak its runner length is tuned for appears.
 *
 * Each runner's valve end is a valve boundary, as a primary's is, and its other end opens into the
 * plenum: the same open end as an exhaust mouth, onto the plenum's gas rather than the atmosphere's
 * (`EulerPipe.setReservoir`).
 *
 * Each runner carries a port injector. It meters fuel in proportion to the fresh air its cylinder takes
 * in, at the commanded mixture, as an ECU does from the air it measures, and sprays it into the charge on
 * its way through the valve. So the plenum and the runners hold air, apart from whatever the cylinder
 * pushes back up them, and each cylinder's fuel arrives with its own charge.
 *
 * Fresh air means net: what goes in through the valve less what the cylinder pushes back out. Charge
 * pushed back up the runner during overlap is already fuelled, and at idle it is most of what comes back
 * in; metering on everything the valve passes inward would fuel it again, and the idle would run at λ 0.6.
 * Nor is it metered on what the runner draws from the plenum, because the column at the plenum end surges
 * in and out between intake strokes.
 *
 * The gas solver carries no composition, so each runner's is tracked alongside it as one well-mixed
 * fraction of spent gas and one of fuel. Charge the cylinder pushes back up the runner during overlap
 * therefore comes back next cycle, as it does in a real one, though spread through the runner rather
 * than as a slug at the valve.
 */

import { GAS, type EngineSpec, intakeRunnerOf, makeSegment, speedOfSound } from '../../model/spec.js';
import { CYL_BURNED, CYL_FUEL, CYL_STATE_SIZE } from './cylinder.js';
import {
  DEFAULT_CELL_SIZE,
  DEFAULT_CFL,
  DEFAULT_MAX_CELLS,
  EulerPipe,
  RES_C,
  RES_P,
  RES_RHO,
  RES_SIZE,
  createSharedKernel,
  ductCellCount,
  singleStepDx,
  type EulerPipeOptions,
  type ValveState,
} from './eulerPipe.js';
import { IO_DARCY, IO_DT, IO_KLIN, type EulerKernel } from './kernel.js';

/** Slots of the array `IntakeRunners.advance` reads its scalars from. */
export const RUN_DT = 0;
export const RUN_P = 1;
export const RUN_RHO = 2;
export const RUN_BURNED = 3;
export const RUN_FUEL = 4;
/**
 * Fuel mass fraction each injector brings the air its cylinder draws to: `fuelFractionAt` the mixture, 0
 * with the fuel cut. A fraction of the charge rather than a ratio to its air, because the fuel takes the
 * place of some of that air: `f` of it, fuelled, leaves `f / (1 - f)` of fuel to air, which is the ratio.
 */
export const RUN_INJECT = 5;
export const RUN_IO_SIZE = 6;

export class IntakeRunners {
  readonly runners: EulerPipe[];
  /** Mass flow through each intake valve, kg/s, positive out of the cylinder into its runner. */
  readonly valveMassFlows: Float64Array;
  /** Mass flow out of each runner's plenum end, kg/s, positive into the plenum. */
  readonly plenumFlows: Float64Array;
  /** Gas temperature at each runner's plenum end, K: what a flow into the plenum carries. */
  readonly plenumTemps: Float64Array;
  /** Spent-gas and fuel mass fractions of each runner's contents. */
  readonly burned: Float64Array;
  readonly fuel: Float64Array;
  /** Gas temperature at each runner's valve end, K: what the cylinder draws in. */
  readonly portTemps: Float64Array;
  /**
   * Fuel mass fraction of what each cylinder draws in through its valve: the runner's own, plus what the
   * injector sprays into it.
   */
  readonly inflowFuel: Float64Array;

  private readonly burnedMass: Float64Array;
  private readonly fuelMass: Float64Array;
  /** Gas mass in each runner, kg. */
  private readonly mass: Float64Array;
  /**
   * Air each cylinder has pushed back out through its valve and not yet drawn back in, kg, as a
   * negative balance: the injector fuels none of what comes back in until it is repaid.
   */
  private readonly airOwed: Float64Array;
  private readonly port = new Float64Array(3);
  private readonly reservoir = new Float64Array(RES_SIZE);
  /** The kernel every runner shares, or `null` where there is none or they did not fit in one. */
  private readonly kernel: EulerKernel | null;

  constructor(spec: EngineSpec, sampleRate: number, count: number, opts: EulerPipeOptions) {
    const { length, diameter } = intakeRunnerOf(spec);
    const segment = [makeSegment({ kind: 'pipe', length, dIn: diameter })];
    const damping = runnerDamping(diameter / 2, speedOfSound(GAS.tAmb, GAS.gammaAir) / (4 * length));
    // Every runner in one shared kernel, stepped together: see `reconstructBatchIo` in
    // kernel/euler.ts. Each is reserved the cells its grid will have.
    const kernel = opts.useKernel === false ? null : createSharedKernel();
    const minDx = opts.singleStep ? singleStepDx(sampleRate, opts.cfl ?? DEFAULT_CFL) : 0;
    const cells = ductCellCount(length, opts.cellSize ?? DEFAULT_CELL_SIZE, opts.maxCells ?? DEFAULT_MAX_CELLS, minDx);
    const offsets: number[] = [];
    for (let b = 0; kernel !== null && b < count; b++) {
      const off = kernel.addSegment(cells);
      if (off < 0) break;
      offsets.push(off);
    }
    const shared = kernel !== null && offsets.length === count ? kernel : null;
    this.kernel = shared;
    this.runners = Array.from(
      { length: count },
      (_, b) =>
        new EulerPipe(segment, sampleRate, spec.portGasTemp, {
          ...opts,
          kernelSegment: shared ? { kernel: shared, offset: offsets[b]!, index: b } : undefined,
          inletKind: 'valve',
          outletKind: 'mouth',
          // The charge in a runner is near ambient and its walls near the engine's; the exhaust's
          // wall model, steel radiating to the air, does not describe them.
          heatTransfer: false,
          initialPortTemp: GAS.tAmb,
          linearDamping: damping,
          port: undefined,
          inheritWall: undefined,
        }),
    );
    if (shared && this.runners.some((r) => r.n !== cells)) {
      throw new Error('IntakeRunners: a runner has a different grid from the one its segment was sized for');
    }
    const z = () => new Float64Array(count);
    this.valveMassFlows = z();
    this.plenumFlows = z();
    this.plenumTemps = z();
    this.portTemps = z();
    this.burned = z();
    this.fuel = z();
    this.burnedMass = z();
    this.fuelMass = z();
    this.mass = z();
    this.airOwed = z();
    this.inflowFuel = z();
    for (let b = 0; b < count; b++) {
      this.mass[b] = this.runners[b]!.totalMass();
      this.runners[b]!.readPort(this.port);
      this.portTemps[b] = this.port[1]!;
      this.plenumTemps[b] = this.portTemps[b]!;
    }
  }

  /** Cells across all the runners, for the cost budget and the load meter. */
  get cells(): number {
    return this.runners.reduce((a, r) => a + r.n, 0);
  }

  /** Solver recoveries across all the runners. Should stay zero. */
  get recoveries(): number {
    return this.runners.reduce((a, r) => a + r.recoveries, 0);
  }

  /**
   * Prime every runner with a mixture, `burned` spent gas and `fuel` fuel, so the first cycles draw a
   * charge rather than a runner full of nothing but air.
   */
  prime(burned: number, fuel: number): void {
    for (let b = 0; b < this.runners.length; b++) {
      const m = this.mass[b]!;
      this.burnedMass[b] = m * burned;
      this.fuelMass[b] = m * fuel;
      this.burned[b] = burned;
      this.fuel[b] = fuel;
    }
  }

  /**
   * Advance every runner by one sample, in lockstep.
   *
   * @param io The sample's scalars, at the `RUN_*` slots: the step, the plenum's pressure, density,
   *   spent-gas and fuel fractions, and the injectors' fuel fraction. Through an array because this is called from `tick`,
   *   which has no inlining budget left, and a float argument to a call that is not inlined is boxed.
   * @param valves Each cylinder's intake valve: its flow area and the cylinder's state.
   * @param breathing Each cylinder's breathing multiplier on the pressure its runner opens onto. See
   *   `EngineSim.breathing`.
   * @param cylState Each cylinder's `CYL_*` state, from which the spent-gas and fuel fractions of
   *   anything pushed back up its runner are read.
   */
  advance(io: Float64Array, valves: ValveState[], breathing: Float64Array, cylState: Float64Array): void {
    const dt = io[RUN_DT]!;
    const pPlenum = io[RUN_P]!;
    const rhoPlenum = io[RUN_RHO]!;
    const plenumBurned = io[RUN_BURNED]!;
    const plenumFuel = io[RUN_FUEL]!;
    const inject = io[RUN_INJECT]!;
    const runners = this.runners;
    const count = runners.length;
    let substeps = 1;
    for (let b = 0; b < count; b++) {
      const s = runners[b]!.substepsFor(dt);
      if (s > substeps) substeps = s;
    }
    const h = dt / substeps;
    const valveFlows = this.valveMassFlows;
    const plenumFlows = this.plenumFlows;
    valveFlows.fill(0);
    plenumFlows.fill(0);

    // What each runner opens onto. Breathing scales pressure and density together, so the plenum's
    // sound speed is every runner's.
    const res = this.reservoir;
    res[RES_C] = Math.sqrt((GAS.gammaExh * pPlenum) / rhoPlenum);
    for (let b = 0; b < count; b++) {
      const bp = breathing[b]!;
      res[RES_P] = pPlenum * bp;
      res[RES_RHO] = rhoPlenum * bp;
      runners[b]!.setReservoir(res);
    }
    const kernel = this.kernel;
    const first = runners[0]!;
    for (let k = 0; k < substeps; k++) {
      // With a shared kernel, the cell loops for every runner run in one call each, around the
      // boundaries and the valve, which each runner does for itself.
      if (kernel !== null) {
        const kio = kernel.io;
        kio[IO_DT] = h;
        kio[IO_KLIN] = first.linearDamping;
        kio[IO_DARCY] = first.darcy;
        kernel.reconstructBatch(first.limiterCode);
      }
      for (let b = 0; b < count; b++) {
        const r = runners[b]!;
        r.beginStep(h);
        r.applyOwnBoundaries(h);
        plenumFlows[b]! += r.mouthMassFlow;
        r.computeValveFlux(valves[b]!);
        r.setEndStep(h, r.valveFluxOut);
      }
      if (kernel !== null) kernel.updateBatch();
      for (let b = 0; b < count; b++) {
        const r = runners[b]!;
        r.endStepSet(valves[b]!);
        valveFlows[b]! += r.valveFluxOut * r.sourceScale;
        r.afterStep(h);
      }
    }

    const inv = 1 / substeps;
    for (let b = 0; b < count; b++) {
      const r = runners[b]!;
      let vf = valveFlows[b]! * inv;
      let pf = plenumFlows[b]! * inv;
      if (r.recoverIfBroken()) {
        vf = 0;
        pf = 0;
        this.mass[b] = r.totalMass();
      } else {
        // Tracked from the flows rather than summed over the cells: the duct conserves mass, so the
        // two agree, and this costs nothing.
        this.mass[b] = this.mass[b]! + (vf - pf) * dt;
      }
      valveFlows[b] = vf;
      plenumFlows[b] = pf;

      // The runner's composition, well mixed: in from the cylinder at its fractions and from the
      // plenum at the plenum's, out either end at the runner's own.
      const mass = this.mass[b]! > 1e-12 ? this.mass[b]! : 1e-12;
      const rb = this.burned[b]!;
      const rf = this.fuel[b]!;
      const s = b * CYL_STATE_SIZE;
      const cb = cylState[s + CYL_BURNED]!;
      const cf = cylState[s + CYL_FUEL]!;
      let burned = this.burnedMass[b]! + ((vf >= 0 ? vf * cb : vf * rb) - (pf >= 0 ? pf * rb : pf * plenumBurned)) * dt;
      // The injector's fuel, on the net air through the valve: see the class comment. Air pushed back out
      // is owed back before any more is fuelled. The fuel goes into the charge the valve is passing, in
      // place of some of its air rather than adding to its mass, and never into the runner.
      const airThrough = vf < 0 ? -vf * (1 - rb - rf) : -vf * (1 - cb - cf);
      let owed = this.airOwed[b]! + airThrough * dt;
      let injected = 0;
      if (owed > 0 && vf < 0) {
        injected = owed * inject;
        owed = 0;
      }
      this.airOwed[b] = owed;
      const inflow = vf < 0 ? -vf * dt : 0;
      const sprayed = inflow > 0 ? rf + injected / inflow : rf;
      this.inflowFuel[b] = sprayed > 1 - rb ? 1 - rb : sprayed;
      let fuel = this.fuelMass[b]! + ((vf >= 0 ? vf * cf : vf * rf) - (pf >= 0 ? pf * rf : pf * plenumFuel)) * dt;
      burned = burned < 0 ? 0 : burned > mass ? mass : burned;
      fuel = fuel < 0 ? 0 : fuel > mass - burned ? mass - burned : fuel;
      this.burnedMass[b] = burned;
      this.fuelMass[b] = fuel;
      this.burned[b] = burned / mass;
      this.fuel[b] = fuel / mass;

      r.readPort(this.port);
      this.portTemps[b] = this.port[1]!;
      r.readMouth(this.port);
      this.plenumTemps[b] = this.port[1]!;
    }
  }
}

/**
 * Acoustic damping of a runner, 1/s, as `EulerPipe`'s `linearDamping` takes it: the viscous and thermal
 * boundary-layer loss at its walls, from Kirchhoff's result for a wide tube,
 *
 *   alpha = sqrt(omega nu / 2) / (a c) * (1 + (gamma - 1) / sqrt(Pr))    Np/m,
 *
 * at `hz`, its quarter-wave resonance, and `k = 2 c alpha`.
 *
 * Derived rather than borrowed from the exhaust, whose damping is fitted to hot gas in steel pipes: the
 * air in a runner is several times less viscous, and a runner a few centimetres across loses far less
 * at the frequencies its tuning lives at. A 49 mm runner tuned to 250 Hz comes out at 13 /s, against the
 * exhaust's 150, which would damp away the very waves that ram the charge in.
 */
function runnerDamping(radius: number, hz: number): number {
  const c = speedOfSound(GAS.tAmb, GAS.gammaAir);
  const nu = AIR_VISCOSITY / (GAS.pAmb / (GAS.R * GAS.tAmb));
  const alpha =
    (Math.sqrt((2 * Math.PI * hz * nu) / 2) / (Math.max(radius, 1e-3) * c)) *
    (1 + (GAS.gammaAir - 1) / Math.sqrt(AIR_PRANDTL));
  return 2 * c * alpha;
}

/** Dynamic viscosity of air near room temperature, Pa s, and its Prandtl number. */
const AIR_VISCOSITY = 1.82e-5;
const AIR_PRANDTL = 0.71;

