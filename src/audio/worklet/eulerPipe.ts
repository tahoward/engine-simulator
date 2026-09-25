/**
 * Quasi-one-dimensional Euler solver for the exhaust system.
 *
 * Real gas dynamics rather than a linear (Kelly-Lochbaum) waveguide, because exhaust
 * blowdown is not a small perturbation: pulses reach roughly a bar in the header at
 * Mach 0.3-0.8, where the high-pressure part of a wave genuinely travels faster than the
 * low-pressure part and the front steepens toward a shock. A linear model cannot do that
 * at any amplitude — measured on this code, a 1 bar wave generates 63% second harmonic
 * here and exactly 0% in a linear waveguide. That harmonic generation is the brassy
 * crackle of an open pipe.
 *
 * Scheme: MUSCL-Hancock, second order in space and time, with a TVD slope limiter on
 * the primitive variables and an HLLC approximate Riemann solver at each face. The
 * limiter is what makes it usable: without it a second-order scheme rings violently at
 * every steep front, and those oscillations are both unphysical and audible. Verified
 * against the Sod shock tube with zero overshoot.
 *
 *     d/dt (A W) + d/dx (A F) = A S + [0, p dA/dx, 0]
 *     W = [rho, rho u, rho E],  F = [rho u, rho u^2 + p, (rho E + p) u]
 *
 * Solving the full equations brings two things a waveguide can only fake. Gas
 * temperature is a solved field rather than an imposed exponential decay, so the
 * hot charge convects and cools as it travels. And mean flow is real, so the gas the
 * waves ride on is actually moving.
 *
 * What it costs: a finite-volume scheme has numerical dissipation, so unlike a delay
 * line it is not exact at every frequency. At the default 20 mm cells the numerical loss
 * stays under the physical wall loss to roughly 3 kHz and takes over above it. That
 * ceiling sits near where the plane-wave assumption fails anyway, which is the reason
 * this trade is acceptable.
 */

import {
  CHAMBER_THROAT,
  GAS,
  PIPE_PRESSURE_TAPS,
  type PipeSegment,
  ambientSoundSpeed,
  chamberBody,
  chamberOffsets,
  pipeTemperature,
  sectionArea,
  sectionPerimeter,
  segmentDiameter,
  segmentSection,
  speedOfSound,
} from '../../model/spec.js';
import { type ChamberPlacement, CrossModes } from './crossModes.js';
import { clamp } from './dsp.js';
import { EulerKernel, type JunctionField, type KernelField } from './kernel.js';
import { ORIFICE_IO, orificeSolve, VALVE_CD } from './valve.js';

const GAMMA = GAS.gammaExh;
const CV = GAS.R / (GAMMA - 1);
const CP = (GAMMA * GAS.R) / (GAMMA - 1);
/**
 * `1/(gamma-1)`, precomputed.
 *
 * Not pedantry: V8 will not fold `x / (GAMMA - 1)` into a multiply, because for IEEE doubles
 * the two are not bit-identical, so every one of those is a real ~15-cycle divide on the
 * per-cell path. The hot loops convert pressure to internal energy several times per cell per
 * substep, which at 48 kHz is millions of divides a second for a constant.
 */
const INV_GM1 = 1 / (GAMMA - 1);
/** `1/gamma`, for the isentropic density of a ghost state. */
const INV_GAMMA = 1 / GAMMA;
/**
 * Exponents for the nonlinear open-end boundary: `(gamma-1)/(2 gamma)` relates an isentropic
 * pressure ratio to a sound-speed ratio, and `2/(gamma-1)` is the coefficient of `c` in the
 * Riemann invariant `u + 2c/(gamma-1)`. See `mouthBoundary`.
 */
const MOUTH_ISENTROPIC_EXP = (GAMMA - 1) / (2 * GAMMA);
const TWO_OVER_GM1 = 2 / (GAMMA - 1);

/** Density of the air outside the pipe, kg/m^3. The reservoir an open end inhales from. */
const AMBIENT_RHO = GAS.pAmb / (GAS.R * GAS.tAmb);

/**
 * Resistance of the mouth's radiation load, Pa s/m: `4 * 0.6133^2 * rho_amb c_amb`, set so the
 * parallel R-L load has the unflanged mouth's radiation resistance `rho_amb c_amb (ka)^2 / 4` at
 * low ka. See `mouthBoundary`.
 */
const MOUTH_RESISTANCE = 4 * 0.6133 * 0.6133 * AMBIENT_RHO * ambientSoundSpeed();
/**
 * Default cell length, m, and cell cap.
 *
 * Chosen from measured real-time cost, not from accuracy alone. Cost scales roughly as the
 * inverse square — halving the cell both doubles the count and halves the stable timestep —
 * so at 10 mm the longer presets run at 80-95% of one core, which is unusable on an audio
 * thread, where 20 mm puts the worst preset near half of one. The price is bandwidth: the
 * scheme stays faithful to about 3 kHz and is artificially dull above it. That ceiling lands
 * near where the plane-wave assumption fails for a wide mouth anyway, so for most
 * geometries it is not the binding limit. `pipeCellSize` exposes the trade. Run `npm run
 * bench` after changing this.
 *
 * This is the solver's own default, for a caller that gives no `cellSize`. The engine always
 * gives one: `pipeCellSize` (0.035 m by default in spec.ts), passed through its cost budget.
 */
export const DEFAULT_CELL_SIZE = 0.02;
export const DEFAULT_MAX_CELLS = 128;

/**
 * Courant number. MUSCL-Hancock is stable to 1.0 in theory; 0.85 keeps a margin while
 * staying above the threshold where a typical duct needs a second substep per audio
 * sample. At 0.80 the CFL limit falls a hair under the 20.8 microsecond sample period, so
 * every sample would pay for two substeps. The count itself is pinned from geometry; see
 * `DESIGN_WAVE_SPEED`.
 */
export const DEFAULT_CFL = 0.85;

/**
 * Design-limit wave speed, m/s, from which the substep count is pinned.
 *
 * The substep count must be *constant over time*. It sets the shape of the decimation from
 * solver rate to audio rate, and a count that falls to one on quiet samples and rises to two
 * on loud ones modulates broadband noise into the signal. A floor of two would hold it constant
 * only because the CFL condition happens never to ask for three at the default cell size — a
 * coincidence of that geometry, not a guarantee.
 *
 * Pinning it from the geometry makes the constancy explicit, and lets a coarse grid
 * legitimately run at *one* substep rather than paying for two it does not need. What the pin
 * needs is the fastest `|u| + c` the duct will ever see. Measured across 1200-9000 rpm, 0.15 to
 * full throttle, on a megaphone, a long tuned pipe, a 2-into-1 and a 50 mm stub, the worst is
 * 1192 m/s, in a stub at 9000 rpm. 1400 gives 17% over that.
 *
 * Exceeding it is not a stability failure — `substepsFor` still subdivides on the measured
 * speed, so the solver stays correct and only the decimation shape moves for those samples.
 * `substepBursts` counts them, and is expected to stay at zero.
 */
export const DESIGN_WAVE_SPEED = 1400;

/**
 * Audio samples between refreshes of the thermal *coefficients* — not of the transfer
 * itself, which happens every sample. See `refreshThermalCoefficients`.
 */
const HEAT_INTERVAL = 16;

/** Internal-energy floor, J/m^3, keeping the state thermodynamically admissible. */
const MIN_INTERNAL = 1e-3 / (GAMMA - 1);

/**
 * Density and pressure floors for the junction boundary, kg/m^3 and Pa.
 *
 * *Physical* floors, unlike the solver's own `1e-7` admissibility guards, and that
 * distinction is the whole point. A junction imposes a pressure and divides a returning wave
 * by the local `rho c`; guarding that division with a number chosen to be merely nonzero
 * means a degenerate cell still yields a velocity of 10^9 and a pressure of 10^35. One
 * percent of exhaust gas at ambient pressure and 2000 K is well below anything a duct can
 * physically contain and far above where the arithmetic falls apart, so it bounds the damage
 * without touching any state the solver would call healthy — on a working duct both floors
 * are two to three orders of magnitude below the live values and never engage.
 */
export const MIN_JUNCTION_RHO = (0.01 * GAS.pAmb) / (GAS.R * 2000);
export const MIN_JUNCTION_P = 0.01 * GAS.pAmb;

/** Ambient temperature to the fourth, precomputed for the radiation term. */
const AMB4 = GAS.tAmb ** 4;

/**
 * Reciprocal time constant of the mean-flow tracker, 1/s.
 *
 * 1/(0.8 s). Long enough that it extracts something very close to DC: the lowest acoustic
 * component is the firing frequency, 3.75 Hz at idle, and anything the tracker follows is
 * exempt from acoustic damping.
 */
const MEAN_FLOW_RATE = 1 / 0.8;

/** Steel: density kg/m^3, specific heat J/(kg K). */
const WALL_RHO = 7800;
const WALL_CP = 490;

/**
 * Multiplier on the steady-flow Nusselt number, accounting for pulsation.
 *
 * Dittus-Boelter describes steady, fully-developed flow in a long smooth pipe. Exhaust flow
 * is none of those things: it arrives as a pulse train that never lets a thermal boundary
 * layer settle, and every cell is within a few diameters of an entrance or a taper. Measured
 * gas-side coefficients in exhaust manifolds run two to four times the steady-flow value,
 * and engine simulation codes apply an enhancement of this kind rather than the bare
 * correlation.
 */
const PULSATION_NUSSELT = 3;

/**
 * Nusselt floor: fully-developed laminar flow in a round duct.
 *
 * Gas that is momentarily at rest still conducts to the wall, and 3.66 is the closed-form
 * answer for that limit. Flooring the *Nusselt number* rather than the Reynolds number
 * states the intent directly — a Reynolds floor such as `Re >= 600` gives about the same
 * thing, but only for one diameter.
 */
const NUSSELT_FLOOR = 3.66;

/**
 * Time constant of the mass-flux average the heat-transfer correlation is evaluated on, s.
 *
 * The correlation must see the flux the gas *typically* carries, not whatever it happened to
 * be carrying at the instant the coefficients were last refreshed. Cell velocity in an
 * exhaust duct swings from nothing to several hundred metres per second within one cycle, so
 * an instantaneous sample is effectively a random draw — and because the gas-to-wall transfer
 * is exponential in the coefficient, the low-velocity draws dominate and the cycle-average
 * transfer collapses toward the stagnant value. Measured on instantaneous samples, the
 * recovered coefficient jumps between 2 and 80 W/(m^2 K) at one fixed operating point, and
 * averages near the bottom of that range.
 *
 * 50 ms spans a firing period at any normal speed while still following the throttle.
 */
const FLUX_AVERAGE_TAU = 0.05;

export type SlopeLimiter = 'minmod' | 'mc' | 'vanleer';

export interface EulerPipeOptions {
  /**
   * Hold the duct to one step per audio sample, giving short ducts fewer, longer cells if that is what
   * it takes. The engine always sets this; the solver's own tests leave it off to check its numerics on
   * fine grids with several steps.
   */
  singleStep?: boolean;
  /** Target cell length, m. Cell count follows from the pipe length. */
  cellSize?: number;
  /** Hard cap on cells, to bound the cost of an absurdly long pipe. */
  maxCells?: number;
  limiter?: SlopeLimiter;
  /** Courant number. 0.8 is safe for MUSCL-Hancock; lower it if anything rings. */
  cfl?: number;
  /** Ceiling on substeps per call, so one blowdown spike cannot stall the audio thread. */
  maxSubsteps?: number;
  /** The cylinder-head port, prepended as the first length of duct. */
  port?: { length: number; diameter: number };
  /** Wall thickness, m. Sets the wall's thermal mass, so how fast it warms up. */
  wallThickness?: number;
  /** Air speed past the pipe, m/s. 0 is a stationary engine. */
  airSpeed?: number;
  /**
   * Wall temperatures to inherit, so editing the pipe does not discard a thermal state
   * that takes half a minute to rebuild. Resampled by normalised position.
   */
  inheritWall?: Float64Array;
  /** Initial wall temperature, K, when there is nothing to inherit. */
  initialWallTemp?: number;
  /**
   * Combined outlet area of the ducts feeding this duct's inlet junction, m^2.
   *
   * Only meaningful when `inletKind` is `'junction'`. See `JUNCTION_INLET_FRACTION`: a
   * collector inlet much narrower than the pipes emptying into it is not solvable, and the
   * duct cannot work that out for itself because it does not know what feeds it.
   */
  junctionInletArea?: number;
  /**
   * Set false to force the TypeScript hot loops even where the wasm kernel is available.
   *
   * Not a tuning knob — the two paths are bit-identical, so there is nothing to choose
   * between acoustically. It exists so the differential test can run both at once and so the
   * benchmark has something to measure the kernel *against*.
   */
  useKernel?: boolean;
  /**
   * Solve junctions in wasm (`kernel/euler.ts`). Defaults to `useKernel`. Separate because the two are
   * held to different standards: the cell kernel renders bit for bit as the TypeScript does, the junction
   * kernel only to rounding — its `Math.pow` is AssemblyScript's.
   */
  useJunctionKernel?: boolean;
  /**
   * Linear momentum damping, 1/s, standing in for viscothermal boundary-layer loss.
   *
   * Needed *in addition* to Darcy friction, which is quadratic in velocity and so
   * damps small-amplitude waves almost not at all. Real duct acoustic attenuation is
   * linear in velocity, and this term supplies it: a sink of `-k rho u` decays wave
   * amplitude as exp(-k x / 2c), so `k = 2 c alpha`.
   */
  linearDamping?: number;
  /** Darcy friction factor for the mean flow. Exhaust tubing runs 0.02-0.04. */
  darcyFriction?: number;
  /** False terminates the pipe with a closed wall — used by tests. */
  radiate?: boolean;
  /**
   * What sits at each end.
   *
   * `valve` is a solid wall with the cylinder entering as a source term in the first cell;
   * `mouth` radiates; `junction` hands the boundary over to an external solve, so several
   * ducts can be merged into a collector.
   */
  inletKind?: 'valve' | 'junction';
  outletKind?: 'mouth' | 'junction';
  /** False disables wall heat transfer, so the gas keeps its temperature. */
  heatTransfer?: boolean;
  /** Initial gas temperature at the port, K. Defaults to the spec's port temperature. */
  initialPortTemp?: number;
}

/**
 * Acoustic state at one end of a duct, as a junction needs to see it.
 *
 * Valid only between `beginStep` and `endStep`. `endState` fills one preallocated object per
 * end rather than returning a fresh one, so a returned state is overwritten by the next call
 * for the same end of the same duct; see `endState`. The reuse makes no measurable difference
 * to CPU (56.1/56.5% against 56.7/56.7% on interleaved runs); it is there for the garbage,
 * not the speed.
 */
export interface EndState {
  /** The wave travelling toward this end, Pa (gauge). */
  toward: number;
  /** Specific acoustic impedance there, rho*c. */
  rhoC: number;
  /** Cross-sectional area at the end cell, m^2. */
  area: number;
  /** Local sound speed, m/s. */
  c: number;
  rho: number;
  p: number;
  u: number;
}

export interface ValveState {
  /** Effective valve flow area, m^2. Zero when shut. */
  throatArea: number;
  /** Cylinder absolute pressure, Pa. */
  cylPressure: number;
  /** Cylinder gas temperature, K. */
  cylTemp: number;
  /** Ratio of specific heats of the cylinder gas. The exhaust gas's when not given. */
  cylGamma?: number;
  /**
   * Extra mass flow into the port, kg/s, on top of the valve's own.
   *
   * Carries the throat turbulence and the slug displaced by a closing valve. Kept
   * separate from the valve flux because it is a fiction of the acoustic model, not real
   * transport — the cylinder's mass balance must not see it.
   */
  extraMassFlow?: number;
}

export interface AdvanceResult {
  /** Volume flow leaving the mouth, m^3/s, averaged over the substeps taken. */
  mouthFlow: number;
  /** Mass flow through the valve, kg/s, positive out of the cylinder. Substep average. */
  valveMassFlow: number;
  /** Absolute pressure at the valve seat, Pa. */
  portPressure: number;
  /** Substeps actually taken, for the load meter. */
  substeps: number;
}

export class EulerPipe {
  readonly n: number;
  readonly dx: number;
  readonly totalLength: number;
  /** Leading cells belonging to the head port rather than the user's pipe. */
  readonly portCells: number;

  // Conserved state, per unit volume.
  private readonly rho: Float64Array;
  private readonly mom: Float64Array;
  private readonly en: Float64Array;

  // Geometry.
  private readonly areaCell: Float64Array;
  private readonly areaFace: Float64Array;
  private readonly diaCell: Float64Array;
  /**
   * Hydraulic diameter per cell, `4A / P`, m. The same as `diaCell` for a round duct and smaller for
   * a flat one, which has more wall for its area. Friction and heat transfer use this one.
   */
  private readonly hydDia: Float64Array;
  /**
   * Wetted perimeter per cell over that of a circle of the same area: exactly 1 for a round duct,
   * more for a flat one. See `BuiltGeometry.shapeCell`.
   */
  private readonly shapeCell: Float64Array;
  /** The cross-wise modes of any chambers in this duct, or `null` if none has one worth keeping. */
  readonly crossModes: CrossModes | null;
  /** 1/(cell volume), precomputed: the update needs it every cell, every substep. */
  private readonly invVol: Float64Array;
  /** 1/(hydraulic diameter), for the friction term. */
  private readonly invDia: Float64Array;
  /** Slowly tracked mean velocity per cell, m/s, so acoustic damping can skip it. */
  private readonly uMean: Float64Array;
  /**
   * Per-cell loss coefficient for flow separation at a contraction, 1/m, signed by the
   * direction of flow it applies to: positive for flow toward +x, negative toward -x.
   *
   * Sudden *expansion* loss needs no help — the 1D momentum equation reproduces the
   * Borda-Carnot result on its own. Contraction is different: the flow separates and
   * forms a vena contracta, and quasi-1D theory has no way to see that. Engine codes handle
   * this with discrete loss coefficients at area changes; this is the same idea spread
   * over the cells the contraction occupies. See `contractionLoss` for the coefficient.
   */
  private readonly contractionK: Float64Array;

  // Reconstruction scratch.
  private readonly pr: Float64Array;
  private readonly pu: Float64Array;
  /** Limited slopes, one pass before the half-step evolution. See `reconstruct`. */
  private readonly sr: Float64Array;
  private readonly su: Float64Array;
  private readonly sp: Float64Array;
  private readonly pp: Float64Array;
  private readonly lr: Float64Array;
  private readonly lu: Float64Array;
  private readonly lp: Float64Array;
  private readonly rr: Float64Array;
  private readonly ru: Float64Array;
  private readonly rp: Float64Array;
  private readonly f0: Float64Array;
  private readonly f1: Float64Array;
  private readonly f2: Float64Array;
  /** Pressure at each face from the Riemann solution, for the well-balanced area source. */
  private readonly fp: Float64Array;

  /**
   * The wasm SIMD kernel for the two hot loops, or `null` to use the TypeScript ones.
   *
   * Measured on this code: 3.1x on a substep, which is more than two f64 lanes can explain on
   * their own — most of the rest is the typed-array bounds check V8 emits on each of the
   * ~50 array accesses per cell, which a wasm linear-memory load does not pay.
   */
  private readonly kernel: EulerKernel | null;

  // Options.
  private readonly limiterCode: Limiter;
  readonly cfl: number;
  readonly maxSubsteps: number;
  /** Wall temperature per cell, K — a solved field, not a constant. */
  private readonly wallT: Float64Array;
  /** Wall heat capacity per cell, J/K. */
  private readonly wallHeatCapacity: Float64Array;
  /** External heat transfer coefficient per cell, W/(m^2 K). */
  private readonly hExt: Float64Array;
  private readonly wallThickness: number;
  /** Cached per-sample decay of gas internal energy toward the wall. */
  private readonly gasDecay: Float64Array;
  /**
   * Running average of |rho*u| per cell, kg/(m^2 s), for the heat-transfer correlation.
   * See `FLUX_AVERAGE_TAU`.
   */
  private readonly fluxAvg: Float64Array;
  /** One-pole coefficient for `fluxAvg`, set with the substep duration. */
  private fluxAvgC = 0;
  /** Cell gas volume, m^3, and outer wall area, m^2 — both fixed by geometry. */
  private readonly cellVolume: Float64Array;
  private readonly outerArea: Float64Array;
  /** Heat handed to the wall since its last update, J. */
  private readonly qBanked: Float64Array;
  /** 1/(wall heat capacity), precomputed to keep a divide out of the batch. */
  private readonly invWallHeatCapacity: Float64Array;
  private readonly linearDamping: number;
  private readonly darcy: number;
  private readonly radiate: boolean;
  readonly inletKind: 'valve' | 'junction';
  readonly outletKind: 'mouth' | 'junction';
  readonly heatTransfer: boolean;

  // Mouth radiation, applied to the acoustic part of the boundary state.
  /**
   * The substep, valve flow and mouth flow `applyValveSource` and `mouthBoundary` take and give, in
   * fields rather than as arguments and a return: both are too big to inline, and a float crossing
   * a call that is not inlined is boxed into a fresh heap object, several times per substep.
   */
  private boundaryDt = 0;
  private sourceFlow = 0;
  /**
   * Fraction of the last valve source actually applied to cell 0: 1 unless the per-substep cap
   * engaged. The cylinder must be debited only what the duct received, so callers scale the
   * valve flow they report by this.
   */
  sourceScale = 1;
  /** Throat static over cylinder stagnation temperature at the last outflow solve, for the jet. */
  private valveThroatT = 1;
  private mouthFlowOut = 0;
  /**
   * One-pole coefficient at the plane-wave cut-on, applied to the wave arriving at the mouth,
   * recomputed from the *substep* duration rather than the audio sample period, because
   * `mouthBoundary` runs once per CFL substep.
   */
  private mouthRefDt = -1;
  private mouthCutC = 0;
  private mouthCutState = 0;
  /**
   * Velocity through the radiation load's inertance, m/s: the state of the mouth's R-L load.
   * See `mouthBoundary`.
   */
  private mouthPhi = 0;
  /** Mouth radius, m, as discretised. */
  private readonly mouthRadius: number;
  /**
   * Radiation corner, rad/s: `ka = 2`, where a monopole of the mouth's volume velocity would
   * radiate as much power as a piston of its area does once its efficiency has saturated.
   */
  readonly mouthCutoffRad: number;
  readonly planeWaveCutoffRad: number;

  /**
   * Frequency above which this discretisation cannot represent a wave at all, rad/s.
   *
   * Taken as c/(5 dx): with ten cells per wavelength the scheme is already losing a few
   * dB per metre, and by five it has nothing left. Radiating above it would be radiating
   * numerical debris.
   *
   * There is a second reason this matters. Whenever the substep count varies from one audio
   * sample to the next — a burst above `DESIGN_WAVE_SPEED` — the decimation filter changes
   * from sample to sample, which modulates broadband noise into the output. With substeps
   * chosen adaptively throughout, that would put 16 kHz only 18 dB below the peak on a narrow
   * tailpipe, where the duct's own dissipation should put it 60 dB down. Band-limiting the
   * radiated signal removes both problems at once.
   */
  readonly resolutionCutoffRad: number;

  /** One-element scratch for `probeJunction`, so a trial solve touches no real state. */
  private readonly pf0 = new Float64Array(1);
  private readonly pf1 = new Float64Array(1);
  private readonly pf2 = new Float64Array(1);
  private readonly pfp = new Float64Array(1);

  private readonly tapIndex: Int32Array;
  /** Largest wave speed seen last step, m/s, used to size the next substep. */
  private lastMaxSpeed = 0;
  /** Substeps per audio sample, fixed by geometry. See `DESIGN_WAVE_SPEED`. */
  private readonly pinnedSubsteps: number;
  /** Samples that needed more than the pinned count. Should stay zero. */
  substepBursts = 0;

  constructor(
    pipe: PipeSegment[],
    readonly sampleRate: number,
    portGasTemp: number,
    opts: EulerPipeOptions = {},
  ) {
    const cellSize = opts.cellSize ?? DEFAULT_CELL_SIZE;
    const maxCells = opts.maxCells ?? DEFAULT_MAX_CELLS;
    this.limiterCode = limiterCodeOf(opts.limiter ?? 'mc');
    this.cfl = opts.cfl ?? DEFAULT_CFL;
    this.maxSubsteps = opts.maxSubsteps ?? 16;
    this.wallThickness = clamp(opts.wallThickness ?? 0.0012, 2e-4, 0.01);
    this.linearDamping = opts.linearDamping ?? 150;
    this.darcy = opts.darcyFriction ?? 0.03;
    this.radiate = opts.radiate ?? true;
    this.inletKind = opts.inletKind ?? 'valve';
    this.outletKind = opts.outletKind ?? 'mouth';
    this.heatTransfer = opts.heatTransfer ?? true;

    const minDx = opts.singleStep ? singleStepDx(sampleRate, this.cfl) : 0;
    const built = buildGeometry(pipe, opts.port, cellSize, maxCells, minDx, {
      inlet: this.inletKind,
      outlet: this.outletKind,
      feedArea: opts.junctionInletArea ?? 0,
    });
    this.n = built.count;
    this.dx = built.dx;
    this.totalLength = built.length;
    this.portCells = built.portCells;

    // Pinned from geometry alone, so it cannot vary while the pipe is unchanged.
    this.pinnedSubsteps = pinnedSubstepsFor(
      this.dx,
      sampleRate,
      this.cfl,
      opts.maxSubsteps ?? 16,
    );

    const z = () => new Float64Array(this.n);

    /**
     * The two hot loops run in wasm when a kernel is available — over *these very arrays*.
     *
     * Every field the kernel touches is a view into its linear memory rather than a copy, so
     * the TypeScript boundary, junction and thermal code indexes `this.rho[i]` exactly as it
     * does without a kernel and nothing is marshalled per substep. That is what makes a 3x
     * speedup on the inner loops worth having: a copy in and out at 96,000 substeps a second
     * would eat it.
     *
     * `null` when the kernel cannot be built — no wasm, no SIMD, or a duct longer than the
     * capacity it was compiled for. The TypeScript loops below are then used unchanged, and
     * `test/kernel.test.ts` asserts the two paths are bit-identical rather than merely close.
     */
    const kernel =
      opts.useKernel === false ? null : EulerKernel.create(this.n, GAMMA, MEAN_FLOW_RATE);
    this.kernel = kernel;
    const field = (name: KernelField, len: number): Float64Array =>
      kernel !== null ? kernel.fields[name].subarray(0, len) : new Float64Array(len);
    const cell = (name: KernelField) => field(name, this.n);
    const face = (name: KernelField) => field(name, this.n + 1);

    this.rho = cell('rho');
    this.mom = cell('mom');
    this.en = cell('en');
    this.areaCell = built.areaCell;
    // Copied in rather than aliased: `buildGeometry` owns the array it returns, and the
    // kernel needs the values inside its own memory.
    this.areaFace = face('areaFace');
    this.areaFace.set(built.areaFace);
    this.diaCell = built.diaCell;
    this.shapeCell = built.shapeCell;
    this.hydDia = new Float64Array(this.n);
    for (let i = 0; i < this.n; i++) this.hydDia[i] = this.diaCell[i]! / this.shapeCell[i]!;
    this.invVol = cell('invVol');
    this.invDia = cell('invDia');
    this.uMean = cell('uMean');
    this.contractionK = cell('contractionK');
    for (let i = 0; i < this.n; i++) {
      this.invVol[i] = 1 / (this.areaCell[i]! * this.dx);
      this.invDia[i] = 1 / this.hydDia[i]!;
    }
    this.contractionK.set(built.contractionK);
    this.pr = cell('pr');
    this.pu = cell('pu');
    this.sr = cell('sr');
    this.su = cell('su');
    this.sp = cell('sp');
    this.pp = cell('pp');
    this.lr = cell('lr');
    this.lu = cell('lu');
    this.lp = cell('lp');
    this.rr = cell('rr');
    this.ru = cell('ru');
    this.rp = cell('rp');
    this.f0 = face('f0');
    this.f1 = face('f1');
    this.f2 = face('f2');
    this.fp = face('fp');

    // --- wall thermal state ---
    this.wallT = z();
    this.wallHeatCapacity = z();
    this.hExt = z();
    this.gasDecay = z();
    this.fluxAvg = z();
    this.cellVolume = z();
    this.outerArea = z();
    this.qBanked = z();
    this.invWallHeatCapacity = z();
    this.gasDecay.fill(1);
    for (let i = 0; i < this.n; i++) {
      this.cellVolume[i] = this.areaCell[i]! * this.dx;
      // The outer wall is the inner one offset by the thickness, which for any convex section adds
      // the perimeter of a circle of that radius.
      const d = this.shapeCell[i]! * this.diaCell[i]!;
      this.outerArea[i] = Math.PI * (d + 2 * this.wallThickness) * this.dx;
    }
    const initTempForWall = opts.initialPortTemp ?? portGasTemp;
    for (let i = 0; i < this.n; i++) {
      // Thin-wall mass: density * (mid-wall perimeter * thickness) * length, the perimeter being
      // that of the circle with the same wall, `pi * shape * d`.
      const d = this.shapeCell[i]! * this.diaCell[i]!;
      const mass = WALL_RHO * Math.PI * (d + this.wallThickness) * this.wallThickness * this.dx;
      this.wallHeatCapacity[i] = Math.max(mass * WALL_CP, 1e-6);
      this.invWallHeatCapacity[i] = 1 / this.wallHeatCapacity[i]!;
      // Sit the wall part-way between the gas it will see and ambient, so the default
      // configuration sounds right immediately instead of after a minute of warm-up.
      const gasGuess = pipeTemperature(initTempForWall, (i + 0.5) * this.dx);
      this.wallT[i] = GAS.tAmb + 0.62 * (gasGuess - GAS.tAmb);
    }
    if (opts.initialWallTemp !== undefined) this.wallT.fill(opts.initialWallTemp);
    if (opts.inheritWall && opts.inheritWall.length > 1) {
      // Resample the previous wall field by normalised position.
      const src = opts.inheritWall;
      for (let i = 0; i < this.n; i++) {
        const u = this.n > 1 ? i / (this.n - 1) : 0;
        this.wallT[i] = src[Math.min(src.length - 1, Math.round(u * (src.length - 1)))]!;
      }
    }
    this.setAirSpeed(opts.airSpeed ?? 0);


    // Initial condition: still gas at ambient pressure, on the empirical `pipeTemperature`
    // profile. It is only a starting guess — heat transfer and convection take it wherever
    // the physics says within a few cycles.
    const initTemp = opts.initialPortTemp ?? portGasTemp;
    for (let i = 0; i < this.n; i++) {
      const x = (i + 0.5) * this.dx;
      const t = pipeTemperature(initTemp, x);
      this.setPrimitive(i, GAS.pAmb / (GAS.R * t), 0, GAS.pAmb);
    }

    const cross = new CrossModes(
      built.chambers,
      this.n,
      this.dx,
      this.rho,
      this.mom,
      this.en,
      this.areaCell,
      this.invVol,
    );
    this.crossModes = cross.count > 0 ? cross : null;

    const mouthRadius = Math.max(this.diaCell[this.n - 1]! / 2, 5e-3);
    this.mouthRadius = mouthRadius;
    const mouthT = pipeTemperature(initTemp, this.totalLength);
    // The radiation load belongs to the medium the sound leaves *into* — the atmosphere — so
    // the corner is on the ambient sound speed, not the duct gas's.
    this.mouthCutoffRad = (2 * ambientSoundSpeed()) / mouthRadius;
    // The plane-wave cut-on is the opposite case: it is about the wave *inside* the duct,
    // where the gas is hot, so it keeps the duct's own sound speed. First non-axisymmetric
    // mode of a circular duct, at ka = 1.8412 — of the radius where higher modes are
    // actually launched, which a gradual flare keeps below the mouth's. See `launchRadiusRatio`.
    this.planeWaveCutoffRad =
      (1.8412 * speedOfSound(mouthT)) / (mouthRadius * built.launchRadiusRatio);
    this.resolutionCutoffRad = (2 * Math.PI * speedOfSound(mouthT)) / (5 * this.dx);
    this.lastMaxSpeed = speedOfSound(initTemp) * 1.2;

    this.tapIndex = new Int32Array(PIPE_PRESSURE_TAPS);
    const first = Math.min(this.portCells, Math.max(this.n - 1, 0));
    const span = Math.max(this.n - 1 - first, 0);
    for (let k = 0; k < PIPE_PRESSURE_TAPS; k++) {
      this.tapIndex[k] = Math.min(
        this.n - 1,
        first + Math.round((k / (PIPE_PRESSURE_TAPS - 1)) * span),
      );
    }
  }

  // -------------------------------------------------------------------------
  // State access
  // -------------------------------------------------------------------------

  setPrimitive(i: number, rho: number, u: number, p: number): void {
    this.rho[i] = rho;
    this.mom[i] = rho * u;
    this.en[i] = p / (GAMMA - 1) + 0.5 * rho * u * u;
  }

  pressureAt(i: number): number {
    const r = this.rho[i]!;
    const u = this.mom[i]! / r;
    return Math.max((GAMMA - 1) * (this.en[i]! - 0.5 * r * u * u), 1e-3);
  }

  /** Cell cross-sectional area, m^2, after area-gradient limiting. */
  areaOf(i: number): number {
    return this.areaCell[i]!;
  }

  densityAt(i: number): number {
    return this.rho[i]!;
  }

  temperatureAt(i: number): number {
    return this.pressureAt(i) / (GAS.R * this.rho[i]!);
  }

  velocityAt(i: number): number {
    return this.mom[i]! / this.rho[i]!;
  }

  /** Absolute pressure at the valve seat, Pa. */
  get portPressure(): number {
    return this.pressureAt(0);
  }

  /**
   * The inlet cell's pressure, temperature and area, into `out[0..2]`, so none is boxed as a return.
   */
  readPort(out: Float64Array): void {
    out[0] = this.pressureAt(0);
    out[1] = this.temperatureAt(0);
    out[2] = this.areaCell[0]!;
  }

  get inletArea(): number {
    return this.areaCell[0]!;
  }

  get mouthArea(): number {
    return this.areaCell[this.n - 1]!;
  }

  /**
   * First quarter-wave resonance of the duct as it is currently filled, Hz.
   *
   * Integrates dx/c over the real solved temperature field rather than assuming a mean
   * sound speed, so it accounts for the hot gas near the port travelling faster than
   * the cooler gas near the mouth. This is the tuning figure a pipe builder wants, and
   * it is exact for the duct the solver is actually running.
   */
  quarterWaveHz(): number {
    let travel = 0;
    for (let i = 0; i < this.n; i++) {
      travel += this.dx / Math.sqrt(GAMMA * GAS.R * this.temperatureAt(i));
    }
    if (this.outletKind === 'mouth' && this.radiate) {
      // The end correction, as a length of the gas at the mouth: see `mouthBoundary`.
      const last = this.n - 1;
      const r = this.rho[last]!;
      const delta = (OPEN_END_FACTOR * this.mouthRadius * AMBIENT_RHO) / r;
      travel += delta / Math.sqrt(GAMMA * GAS.R * this.temperatureAt(last));
    }
    return travel > 0 ? 1 / (4 * travel) : 0;
  }

  /** Total gas mass in the duct, kg. Conservation check. */
  totalMass(): number {
    let m = 0;
    for (let i = 0; i < this.n; i++) m += this.rho[i]! * this.areaCell[i]! * this.dx;
    return m;
  }

  /** Total energy in the duct, J. */
  totalEnergy(): number {
    let e = 0;
    for (let i = 0; i < this.n; i++) e += this.en[i]! * this.areaCell[i]! * this.dx;
    return e;
  }

  /** Writes gauge pressure along the visible pipe into `out`. */
  samplePressure(out: Float32Array): void {
    for (let k = 0; k < this.tapIndex.length; k++) {
      out[k] = this.pressureAt(this.tapIndex[k]!) - GAS.pAmb;
    }
  }

  /** Gas temperature along the visible pipe, K. */
  sampleTemperature(out: Float32Array): void {
    for (let k = 0; k < this.tapIndex.length; k++) {
      out[k] = this.temperatureAt(this.tapIndex[k]!);
    }
  }

  // -------------------------------------------------------------------------
  // Time advance
  // -------------------------------------------------------------------------

  /**
   * Advance the duct by `dt` seconds, substepping to satisfy the CFL condition.
   *
   * The valve mass flow is recomputed against the live cell-0 pressure on every
   * substep rather than being held for the whole audio sample. When the valve is wide
   * open the coupling is stiff, and holding the flow constant across a 20 microsecond
   * sample overshoots the port pressure and rings.
   */
  advance(dt: number, valve: ValveState): AdvanceResult {
    const substeps = this.substepsFor(dt);
    const h = dt / substeps;

    let mouthAcc = 0;
    let valveAcc = 0;

    for (let k = 0; k < substeps; k++) {
      this.beginStep(h);
      const mouth = this.applyOwnBoundaries(h);
      const valveFlow = this.valveFlux(valve);
      this.endStep(h, valveFlow + (valve.extraMassFlow ?? 0), valve);
      this.afterStep(h);
      mouthAcc += mouth;
      valveAcc += valveFlow * this.sourceScale;
    }

    const inv = 1 / substeps;
    let mouthFlow = mouthAcc * inv;
    let valveMassFlow = valveAcc * inv;

    // Last-resort recovery. On an audio thread a single NaN is unrecoverable — it propagates
    // through the whole array and the app goes permanently silent — so an inadmissible state
    // is reset to quiescent rather than left to poison everything. `recoveries` is exposed so
    // this can never hide silently.
    if (!Number.isFinite(mouthFlow) || !Number.isFinite(valveMassFlow) || !this.isFinite()) {
      this.resetToQuiescent();
      mouthFlow = 0;
      valveMassFlow = 0;
      this.recoveries++;
    }

    return { mouthFlow, valveMassFlow, portPressure: this.portPressure, substeps };
  }

  /**
   * Mass flow through the exhaust valve, kg/s, positive out of the cylinder.
   *
   * Reverse flow is kept: during overlap the pipe can push gas back into the cylinder,
   * which is exactly the mechanism a tuned exhaust exploits.
   */
  private valveFlux(valve: ValveState): number {
    const area = valve.throatArea;
    if (area <= 0) return 0;
    // `orificeMassFlow`, written out through its arrays: this is inlined into the exhaust step, which
    // has no inlining budget left for it, and every float crossing a call that is not inlined is boxed.
    const port = this.portScratch;
    this.readPort(port);
    const pPort = port[0]!;
    const io = ORIFICE_IO;
    io[1] = VALVE_CD;
    io[0] = area;
    if (valve.cylPressure > pPort) {
      // The upstream gas's gamma: the cylinder's mixture going out, exhaust gas coming back.
      io[5] = valve.cylGamma ?? GAMMA;
      io[2] = valve.cylPressure;
      io[3] = valve.cylTemp;
      io[4] = pPort;
      orificeSolve(io);
      this.valveThroatT = io[6]! > 0 ? io[8]! : 1;
      return io[6]!;
    }
    io[5] = GAMMA;
    io[2] = pPort;
    io[3] = port[1]!;
    io[4] = valve.cylPressure;
    orificeSolve(io);
    return -io[6]!;
  }

  /** Scratch for `readPort` inside the solver. */
  private readonly portScratch = new Float64Array(3);

  /**
   * Phase one of a substep: primitives, limited slopes, half-step evolution and every
   * interior face flux. The two boundary faces are deliberately left unset.
   *
   * Split out so an external junction can read `endState`, find the common pressure across
   * several ducts and impose it, all between reconstruction and the conservative update.
   * The reconstruction arrays are only meaningful between this call and `endStep`.
   */
  beginStep(dt: number): void {
    this.reconstruct(dt);
  }

  /** Phase two: the conservative update, valve source and friction. */
  endStep(dt: number, valveFlow: number, valve: ValveState): void {
    this.setEndStep(dt, valveFlow);
    this.endStepSet(valve);
  }

  /**
   * `endStep` in two halves, for a caller that cannot count on inlining it: this one is small enough
   * that V8 always inlines it, so the two floats go into fields unboxed, and `endStepSet` takes none.
   */
  setEndStep(dt: number, valveFlow: number): void {
    this.boundaryDt = dt;
    this.sourceFlow = valveFlow;
  }

  endStepSet(valve: ValveState): void {
    this.update(valve);
  }

  /**
   * Phase three: the thermal pass. Separate from `endStep` because it works on the updated
   * state and is batched on its own schedule.
   */
  afterStep(dt: number): void {
    if (!this.heatTransfer) return;
    this.applyThermal();
    this.heatBatch += dt;
    if (--this.heatCounter <= 0) {
      this.applyWallThermal(this.heatBatch);
      this.refreshThermalCoefficients(dt);
      this.heatCounter = HEAT_INTERVAL;
      this.heatBatch = 0;
    }
  }

  /** Number of times an inadmissible state had to be reset. Should stay zero. */
  recoveries = 0;
  /**
   * Times the junction boundary had to clamp a degenerate end state. Should stay zero.
   *
   * Distinct from `recoveries`, and the distinction matters: a recovery means the duct was
   * already broken and got reset, while a clamp means the junction caught it first. A geometry
   * that clamps steadily is not diverging, but it is not being solved as drawn either — the
   * cells at the boundary are over-expanding and the boundary is papering over it. Zero here
   * is the evidence that a geometry is genuinely well posed rather than merely surviving.
   */
  junctionClamps = 0;
  /**
   * Faces that were treated as supersonic outflow. Not an error, unlike the two counters above.
   *
   * A choked mouth or a choked collector inlet is a real flow state, so this is expected to be
   * non-zero on a short, wide duct driven hard, and expected to be zero on anything ordinary.
   * It is exposed because it changes which boundary condition is in force, and that is worth
   * being able to see when a geometry sounds unlike the drawing.
   */
  supersonicFaces = 0;
  private heatCounter = 0;
  private heatBatch = 0;

  private isFinite(): boolean {
    for (let i = 0; i < this.n; i++) {
      if (!Number.isFinite(this.rho[i]!) || !Number.isFinite(this.mom[i]!)) return false;
      if (!Number.isFinite(this.en[i]!) || this.rho[i]! <= 0) return false;
    }
    return true;
  }

  /** True if the state has gone inadmissible; resets it if so. Checked by the owner. */
  recoverIfBroken(): boolean {
    if (this.isFinite()) return false;
    this.resetToQuiescent();
    this.recoveries++;
    return true;
  }

  private resetToQuiescent(): void {
    // Keep the wall's thermal state: it is slow, and throwing it away would make the
    // tuning jump. Only the gas is reset.
    for (let i = 0; i < this.n; i++) {
      this.setPrimitive(i, GAS.pAmb / (GAS.R * this.wallT[i]!), 0, GAS.pAmb);
      this.fluxAvg[i] = 0;
    }
    this.mouthPhi = 0;
    this.mouthCutState = 0;
    this.crossModes?.reset();
    this.lastMaxSpeed = speedOfSound(this.meanWallTemp()) * 1.5;
  }

  /**
   * Substeps this duct takes to cover `dt`.
   *
   * Normally the pinned count, which depends only on geometry and so never varies while the
   * pipe is unchanged. Rises above it only if the gas is moving faster than the design limit,
   * which is a correctness-over-consistency choice: an under-resolved step is unstable, where a
   * changed decimation shape is merely audible.
   */
  substepsFor(dt: number): number {
    const limit = (this.cfl * this.dx) / Math.max(this.lastMaxSpeed, 1);
    const needed = Math.ceil(dt / limit);
    if (needed > this.pinnedSubsteps) {
      this.substepBursts++;
      return Math.min(needed, this.maxSubsteps);
    }
    return this.pinnedSubsteps;
  }

  /** Substeps this duct always takes, absent a burst. Constant for a given geometry. */
  get substeps(): number {
    return this.pinnedSubsteps;
  }

  /** Mass flow through this duct's exhaust valve, kg/s, positive out of the cylinder. */
  valveFluxFor(valve: ValveState): number {
    return this.valveFlux(valve);
  }

  /** Apply the duct's own built-in boundary conditions, for ends with no junction. */
  applyOwnBoundaries(dt: number): number {
    if (this.inletKind === 'valve') this.inletWall();
    if (this.outletKind !== 'mouth') return 0;
    this.boundaryDt = dt;
    this.mouthBoundary();
    return this.mouthFlowOut;
  }

  private inletWall(): void {
    // A solid wall: no mass or energy crosses, and the pressure on it is the reflecting wall's
    // Riemann pressure — the gas arriving at it is brought to rest, which raises its pressure,
    // and gas leaving it is expanded. The mirrored state gives exactly that, with a zero contact
    // speed. The valve enters as a source term in cell 0 rather than as a flux, which keeps mass
    // and energy conserved by construction and lets the orifice relation handle choking.
    //
    // HLLC on the mirrored pair, written out: its contact speed is exactly zero, so what remains
    // is the star pressure `p + rho (s_L + u) u`, with `s_L = -(|u| + c)`.
    const r = this.lr[0]!;
    const u = this.lu[0]!;
    const p = this.lp[0]!;
    const c = Math.sqrt((GAMMA * p) / r);
    const sL = Math.min(-u - c, u - c);
    const pWall = Math.max(p + r * (sL + u) * u, 1e-3);
    this.f0[0] = 0;
    this.f1[0] = pWall;
    this.f2[0] = 0;
    this.fp[0] = pWall;
  }

  /**
   * Primitives, limited slopes, half-step and interior faces — in wasm if a kernel was built,
   * otherwise in TypeScript below.
   *
   * The finiteness guard on the wave speed lives here rather than in either implementation, so
   * both reach it by the same route. A non-finite speed would make the substep size NaN and
   * stall the loop silently.
   */
  private reconstruct(dt: number): void {
    const kernel = this.kernel;
    if (kernel !== null) {
      const maxSpeed = kernel.reconstruct(this.n, dt, this.limiterCode);
      this.lastMaxSpeed = Number.isFinite(maxSpeed) ? Math.max(maxSpeed, 1) : 1e5;
      return;
    }
    this.reconstructTs(dt);
  }

  private reconstructTs(dt: number): void {
    const { n, rho, mom, en, pr, pu, pp, lr, lu, lp, rr, ru, rp, f0, f1, f2 } = this;
    const { areaFace, invVol } = this;
    const kind = this.limiterCode;

    // --- primitives, and the wave speed for the next substep ---
    let maxSpeed = 0;
    for (let i = 0; i < n; i++) {
      const r = rho[i]!;
      // One reciprocal, used for both the velocity and the sound speed.
      const invR = 1 / r;
      const u = mom[i]! * invR;
      const p = Math.max((GAMMA - 1) * (en[i]! - 0.5 * r * u * u), 1e-3);
      pr[i] = r;
      pu[i] = u;
      pp[i] = p;
      const s = Math.abs(u) + Math.sqrt(GAMMA * p * invR);
      if (s > maxSpeed) maxSpeed = s;
    }
    // A non-finite speed would make the substep size NaN and stall the loop silently.
    this.lastMaxSpeed = Number.isFinite(maxSpeed) ? Math.max(maxSpeed, 1) : 1e5;

    // --- limited slopes on primitives, then half-step evolution (Hancock) ---
    // Slopes first, into a scratch array, with the end cells peeled off.
    //
    // Two things are being kept out of the hot body. An `i === 0 || i === n - 1` test would be
    // a branch per cell for a condition true exactly twice, and a generic `slope` would re-test
    // the limiter code on all three of its calls per cell for a value fixed for the life of the duct.
    // Hoisting the limiter to a switch around three tight loops lets each one inline a single
    // expression instead of calling through a three-way branch 21 million times a second.
    const { sr: srA, su: suA, sp: spA } = this;
    srA[0] = suA[0] = spA[0] = 0;
    srA[n - 1] = suA[n - 1] = spA[n - 1] = 0;
    switch (kind) {
      case Limiter.Minmod:
        for (let i = 1; i < n - 1; i++) {
          srA[i] = minmod(pr[i]! - pr[i - 1]!, pr[i + 1]! - pr[i]!);
          suA[i] = minmod(pu[i]! - pu[i - 1]!, pu[i + 1]! - pu[i]!);
          spA[i] = minmod(pp[i]! - pp[i - 1]!, pp[i + 1]! - pp[i]!);
        }
        break;
      case Limiter.VanLeer:
        for (let i = 1; i < n - 1; i++) {
          srA[i] = vanLeer(pr[i]! - pr[i - 1]!, pr[i + 1]! - pr[i]!);
          suA[i] = vanLeer(pu[i]! - pu[i - 1]!, pu[i + 1]! - pu[i]!);
          spA[i] = vanLeer(pp[i]! - pp[i - 1]!, pp[i + 1]! - pp[i]!);
        }
        break;
      default:
        for (let i = 1; i < n - 1; i++) {
          srA[i] = mc(pr[i]! - pr[i - 1]!, pr[i + 1]! - pr[i]!);
          suA[i] = mc(pu[i]! - pu[i - 1]!, pu[i + 1]! - pu[i]!);
          spA[i] = mc(pp[i]! - pp[i - 1]!, pp[i + 1]! - pp[i]!);
        }
    }

    for (let i = 0; i < n; i++) {
      const sr = srA[i]!;
      const su = suA[i]!;
      const sp = spA[i]!;

      const rL = pr[i]! - 0.5 * sr;
      const uL = pu[i]! - 0.5 * su;
      const pL = pp[i]! - 0.5 * sp;
      const rR = pr[i]! + 0.5 * sr;
      const uR = pu[i]! + 0.5 * su;
      const pR = pp[i]! + 0.5 * sp;

      const UL1 = rL * uL;
      const UL2 = pL * INV_GM1 + 0.5 * rL * uL * uL;
      const UR1 = rR * uR;
      const UR2 = pR * INV_GM1 + 0.5 * rR * uR * uR;

      // Half-step predictor, in the *same* quasi-1D form as the corrector: fluxes
      // weighted by their face areas, plus the p dA momentum source.
      //
      // Both halves of that matter and they must go together. With neither, a cavity
      // between two area changes pumps itself (650x energy growth in a muffler
      // chamber). With the source but constant-area fluxes, the predictor stops
      // being well balanced — at rest it invents momentum wherever the area varies, and
      // the transient that follows is tens of dB too loud. Area-weighting both terms
      // makes the predictor reduce to zero at rest, exactly as the corrector does.
      const aLh = areaFace[i]!;
      const aRh = areaFace[i + 1]!;
      // `invVol` is already 1/(A dx), so the half-step factor is a multiply, not a divide.
      const hk = 0.5 * dt * invVol[i]!;

      const d0 = hk * (aLh * UL1 - aRh * UR1);
      const d1 =
        hk * (aLh * (UL1 * uL + pL) - aRh * (UR1 * uR + pR)) + hk * pp[i]! * (aRh - aLh);
      const d2 = hk * (aLh * (UL2 + pL) * uL - aRh * (UR2 + pR) * uR);

      // One reciprocal per side, shared by the velocity and the kinetic-energy term.
      let a0 = rL + d0;
      let a1 = UL1 + d1;
      let a2 = UL2 + d2;
      let dens = a0 > 1e-7 ? a0 : 1e-7;
      let invDens = 1 / dens;
      lr[i] = dens;
      lu[i] = a1 * invDens;
      lp[i] = Math.max((GAMMA - 1) * (a2 - 0.5 * a1 * a1 * invDens), 1e-3);

      a0 = rR + d0;
      a1 = UR1 + d1;
      a2 = UR2 + d2;
      dens = a0 > 1e-7 ? a0 : 1e-7;
      invDens = 1 / dens;
      rr[i] = dens;
      ru[i] = a1 * invDens;
      rp[i] = Math.max((GAMMA - 1) * (a2 - 0.5 * a1 * a1 * invDens), 1e-3);
    }

    // --- interior faces ---
    //
    // Deliberately a second pass. Fusing this into the loop above looks like an obvious win —
    // an interior cell's left and right states are read by nothing but its own two faces, so
    // they need never reach memory, and carrying the previous cell's right state in three
    // locals removes six stores, six loads and a whole traversal per cell. Measured, it is
    // 2.6 points of realtime *slower* (60.6% against 58.0% on the V-twin preset, best of five
    // runs each). V8 evidently generates better code for two simple uniform loops than for one
    // loop carrying more live state and a boundary branch around the Riemann solve. Left as
    // two passes on the strength of the measurement, against the theory.
    for (let f = 1; f < n; f++) {
      hllc(rr[f - 1]!, ru[f - 1]!, rp[f - 1]!, lr[f]!, lu[f]!, lp[f]!, f, f0, f1, f2, this.fp);
    }
  }

  /**
   * Conservative update, then the valve source.
   *
   * Only the cell loop is vectorised. The valve source is scalar, touches cell 0 alone and
   * needs the cylinder state, so there is nothing there for SIMD to do and a good deal of
   * plumbing to avoid — it stays in TypeScript on both paths.
   */
  private update(valve: ValveState): void {
    const kernel = this.kernel;
    if (kernel !== null) kernel.update(this.n, this.boundaryDt, this.linearDamping, this.darcy);
    else this.updateCellsTs(this.boundaryDt);
    this.applyValveSource(valve);
    const cross = this.crossModes;
    if (cross !== null) {
      cross.dt = this.boundaryDt;
      cross.step();
    }
  }

  private updateCellsTs(dt: number): void {
    const { n, rho, mom, en, f0, f1, f2 } = this;
    const { areaFace } = this;
    // Hoisted for the same reason as the rest: these are read once per cell per substep, and
    // a property load off `this` inside the loop is a load V8 has to prove it can lift.
    const { fp, invVol, uMean: uMeanArr, invDia, contractionK } = this;
    const kLin = this.linearDamping;
    const darcy = this.darcy;

    // --- conservative update with area weighting ---
    for (let i = 0; i < n; i++) {
      const aL = areaFace[i]!;
      const aR = areaFace[i + 1]!;
      const k = dt * invVol[i]!;

      let nr = rho[i]! - k * (aR * f0[i + 1]! - aL * f0[i]!);
      // Well-balanced area source.
      //
      // The pressure here must be the mean of the two *face* pressures the Riemann
      // solver produced, not the cell-centred value. With a cell-centred pressure the
      // flux difference and the source only cancel exactly at rest; once a gradient is
      // present each area change leaks a small, geometry-locked momentum error. A single
      // expansion or contraction merely damps it away, but an expansion followed by a
      // contraction is a resonant cavity that traps and compounds it — measured here as
      // 650x acoustic energy growth in a muffler chamber. Using face pressures makes the
      // two terms telescope for any state, not just a static one.
      const pFace = 0.5 * (fp[i]! + fp[i + 1]!);
      let nm = mom[i]! - k * (aR * f1[i + 1]! - aL * f1[i]!) + k * pFace * (aR - aL);
      let ne = en[i]! - k * (aR * f2[i + 1]! - aL * f2[i]!);

      // Positivity. A violent transient — a wide-open valve on a 6 mm stub, say — can
      // drive a cell's internal energy negative, which then yields an imaginary sound
      // speed and poisons the whole array with NaN within a few steps. Flooring the
      // *internal energy* rather than only the pressure at read time keeps the state
      // itself admissible. This sacrifices strict conservation in exactly the cells
      // where the solution was already meaningless.
      if (nr < 1e-7) nr = 1e-7;

      // --- friction, split by what it physically acts on ---
      //
      // The linear boundary-layer term is *acoustic* damping: it must act on the
      // perturbation about the mean flow, not on the mean flow itself. Applied to the
      // total velocity it also brakes the mean, which is wrong and measurably so —
      // 120 m/s decays to 1 m/s in 50 ms, and at low speed the linear term outweighs
      // Darcy by roughly forty to one. Mean flow should feel only the quadratic terms.
      //
      //   du/dt = -k_lin (u - u_mean) - k_quad u
      //
      // solved implicitly in u so no coefficient can overshoot through zero.
      // One reciprocal of the new density, reused by the velocity and the kinetic floor.
      const invNr = 1 / nr;
      const uOld = nm * invNr;
      const uMean = uMeanArr[i]!;

      // The contraction loss is signed by the direction it acts in: `max(u * k, 0)` is `|u| |k|`
      // when the flow runs into the narrowing and zero when it runs out of it.
      const kQuad =
        Math.abs(uOld) * (darcy * 0.5 * invDia[i]!) + Math.max(uOld * contractionK[i]!, 0);
      const uNew = (uOld + kLin * uMean * dt) / (1 + (kLin + kQuad) * dt);
      nm = nr * uNew;

      // Track the mean with a time constant far longer than any acoustic period in play
      // — the firing frequency is only 3.75 Hz at idle, so a short tracker would mistake
      // the firing pulsation for mean flow and leave it undamped.
      uMeanArr[i] = uMean + (uNew - uMean) * (dt * MEAN_FLOW_RATE);

      const kinetic = 0.5 * nm * nm * invNr;
      if (ne - kinetic < MIN_INTERNAL) ne = kinetic + MIN_INTERNAL;

      rho[i] = nr;
      mom[i] = nm;
      en[i] = ne;
    }
  }

  /** Mass, momentum and energy handed to cell 0 by the exhaust valve. */
  private applyValveSource(valve: ValveState): void {
    const { dx, rho, mom, en, areaCell } = this;
    const dt = this.boundaryDt;
    const valveFlow = this.sourceFlow;
    this.sourceScale = 1;

    if (valveFlow !== 0) {
      const vol = areaCell[0]! * dx;
      let dm = (valveFlow * dt) / vol;
      // A wide valve opening onto a very small first cell can ask to more than double
      // its mass in a single substep, which the explicit update cannot absorb. Capping
      // the change per substep keeps it admissible. What the cap cuts off is not carried
      // over: the flow is recomputed from the pressures every substep. `sourceScale`
      // reports the cut, so the cylinder is debited only what arrived.
      const cap = 0.25 * rho[0]!;
      const uncapped = dm;
      if (dm > cap) dm = cap;
      else if (dm < -cap) dm = -cap;
      if (dm !== uncapped) this.sourceScale = dm / uncapped;

      if (valveFlow > 0) {
        // Gas arrives from the cylinder carrying its total enthalpy, and with the
        // momentum of the jet leaving the valve throat. The jet's speed is the isentropic
        // expansion from the cylinder's stagnation state to the throat's static pressure,
        // which is the port pressure, or the critical pressure once the throat chokes: the
        // throat state of the orifice solve that set this flow.
        const g = valve.cylGamma ?? GAMMA;
        const t0 = valve.cylTemp;
        const jetU = Math.sqrt(Math.max((2 * g * GAS.R * t0 * (1 - this.valveThroatT)) / (g - 1), 0));
        rho[0] = rho[0]! + dm;
        // Only part of the jet's momentum arrives along the port. A poppet valve discharges
        // radially off its head and the flow turns into the port against its walls, which is why
        // 1D valve models (Benson's) recover none of it and let it dissipate as heat at the
        // throat pressure. The fraction of the pipe cross-section the jet occupies carries it
        // here; the rest of the face is a wall.
        mom[0] = mom[0]! + dm * jetU * clamp(valve.throatArea / areaCell[0]!, 0, 1);
        en[0] = en[0]! + dm * CP * valve.cylTemp;
      } else {
        /**
         * Reverse flow: mass leaves carrying the cell's own *stagnation enthalpy*.
         *
         * `CP`, not `CV`, and the difference is a rectifier. Gas crossing an orifice does flow
         * work on whatever it moves into, so a mass exchange has to trade `h + u^2/2` in both
         * directions — which the inflow branch above does. Taking only `e + u^2/2` back out
         * would leave every in-and-out pair of a *zero-mean* mass source depositing `R T` per
         * unit mass in the duct: about 430 kJ/kg at 1500 K, against the 45 kJ/kg of kinetic
         * energy at 300 m/s that it would count.
         *
         * Every oscillating source would feed that: throat turbulence, valve-seat pulses,
         * collector merge noise, and real flow reversal during overlap. Because the source is
         * divided by the first cell's volume, the smaller the duct the faster it would heat — a
         * short collector reaching 40,000 K while long ones looked fine.
         */
        const u0 = mom[0]! / rho[0]!;
        const t0 = this.temperatureAt(0);
        rho[0] = Math.max(rho[0]! + dm, 1e-7);
        mom[0] = mom[0]! + dm * u0;
        en[0] = en[0]! + dm * (CP * t0 + 0.5 * u0 * u0);
      }
    }
  }

  /**
   * Open-end boundary.
   *
   * The outgoing wave stays fully nonlinear; only the small wave coming back from
   * outside is treated acoustically, which is well justified because radiation is weak.
   * The characteristic split about the local mean gives p+ and p-, the mouth's radiation
   * impedance sets the returning p-, and a ghost state carrying that p- is handed to the
   * Riemann solver.
   *
   * The radiation load of an unflanged mouth is a mass and a resistance in parallel, per unit
   * area:
   *
   *   L = rho_amb * 0.6133 a            the air outside moving with the air inside
   *   R = 4 * 0.6133^2 * rho_amb c_amb  so that Re Z = rho_amb c_amb (ka)^2 / 4 at low ka
   *
   * both of the *ambient* air, against the duct gas's own `rho c`. That gives Levine and
   * Schwinger's |R| = 1 - (ka)^2/2 for a cold duct, and for a hot one the right thing too: the
   * end correction, seen as a length of duct gas, grows as `rho_amb / rho`, and more of the wave
   * escapes because hot gas is a poorer impedance match to cold air.
   */
  private mouthBoundary(): void {
    const dt = this.boundaryDt;
    const n = this.n;
    const last = n - 1;
    const r = this.rr[last]!;
    const u = this.ru[last]!;
    const p = this.rp[last]!;

    if (!this.radiate) {
      // Closed wall: mirror the state so no mass or energy crosses.
      hllc(r, u, p, r, -u, p, n, this.f0, this.f1, this.f2, this.fp);
      this.mouthFlowOut = 0;
      return;
    }

    const c = Math.sqrt((GAMMA * p) / r);

    // Supersonic outflow: impose nothing.
    //
    // Everything below splits the boundary state into a wave going out and a small wave coming
    // back, which is a *subsonic* construction — it presumes information can travel upstream
    // into the duct. Once the last cell reaches Mach 1 it cannot: all three characteristics
    // leave the domain, and a reflected wave computed anyway is energy invented at the boundary.
    //
    // A ghost equal to the interior state is exactly the right thing to hand the Riemann
    // solver here: with both sides identical and `u >= c`, HLLC takes its `sL >= 0` branch and
    // returns the interior's own physical flux, which is the supersonic-outflow condition. The
    // reflection filters are left untouched rather than driven, so whatever they held resumes
    // when the flow drops back below Mach 1.
    //
    // A short, wide-mouthed megaphone really can choke, so this is a state the model should be
    // able to represent rather than merely survive.
    if (u >= c) {
      this.supersonicFaces++;
      hllc(r, u, p, r, u, p, n, this.f0, this.f1, this.f2, this.fp);
      this.mouthFlowOut = (this.f0[n]! * this.areaFace[n]!) / r;
      return;
    }

    const zc = r * c;
    const pPrime = p - GAS.pAmb;
    // Split into travelling components about the local state.
    const pPlus = 0.5 * (pPrime + zc * u);

    // Coefficient follows the substep duration. Cached, because it only changes when the
    // substep count does.
    if (dt !== this.mouthRefDt) {
      this.mouthRefDt = dt;
      this.mouthCutC = 1 - Math.exp(-this.planeWaveCutoffRad * dt);
    }
    // Only what lies below the plane-wave cut-on is reflected. Above it a duct no longer
    // carries plane waves at all, so a plane-wave reflection coefficient has no meaning there
    // — that energy goes into higher modes and away. Without this it neither radiates (the
    // output is band-limited) nor leaves, and simply rattles around inside.
    this.mouthCutState += this.mouthCutC * (pPlus - this.mouthCutState);
    const pIn = this.mouthCutState;

    // The radiation load, solved implicitly against the incoming wave:
    //   u' = (2 p_in - p') / zc   (the duct side)
    //   u' = p' / R + phi,  phi' = p' / L   (the load)
    // The cut-on pole delays the reflection by 1/w_pw, which is itself an inertance of
    // zc / (2 w_pw); it is taken out of the load's so the end correction is counted once.
    const a = this.mouthRadius;
    const inertance = Math.max(
      AMBIENT_RHO * OPEN_END_FACTOR * a - (0.5 * zc) / this.planeWaveCutoffRad,
      1e-3 * AMBIENT_RHO * a,
    );
    const pLoad =
      ((2 * pIn) / zc - this.mouthPhi) / (1 / zc + 1 / MOUTH_RESISTANCE + dt / inertance);
    this.mouthPhi += (dt * pLoad) / inertance;
    const pMinus = pLoad - pIn;

    const pGhost = Math.max(GAS.pAmb + pPlus + pMinus, 1e-3);

    /**
     * Ghost velocity from the outgoing Riemann invariant, not from the linear impedance.
     *
     * `(pPlus - pMinus) / zc` is the acoustic relation, and using it here would contradict the
     * claim above that only the weak returning wave is treated acoustically — it applies a
     * linear impedance to the *whole* outgoing wave, which at blowdown is most of a bar. Worse,
     * it is a positive feedback: `zc = rho c` sits in the denominator, so as the mouth pushes
     * mass out and the density falls, the same wave asks for a larger outflow velocity, which
     * pushes more mass out.
     *
     * The invariant `u + 2c/(gamma-1)` is carried out of the domain by the outgoing
     * characteristic, so it is what the interior actually determines. Holding it, taking the
     * pressure from the radiation model and the entropy from the interior closes the boundary
     * exactly and nonlinearly. It reduces to the acoustic relation for small perturbations —
     * `u - dp/(rho c)` — so the two differ only for pulses large enough for the linear relation
     * to be wrong.
     */
    // Outflow first: gas on its way out carries the duct's own entropy, so the ghost is the
    // interior state brought isentropically to the boundary pressure.
    let rGhost = Math.max(r * Math.pow(pGhost / p, INV_GAMMA), 1e-7);
    let cGhost = c * Math.pow(pGhost / p, MOUTH_ISENTROPIC_EXP);
    let uGhost = u + TWO_OVER_GM1 * (c - cGhost);

    /**
     * Inflow carries *ambient* entropy, not the duct's.
     *
     * Scaling the interior state to the boundary pressure is right for gas leaving and wrong for
     * gas arriving: air coming in from outside has never been in the engine, and giving it the
     * duct's entropy hands it the duct's temperature as well. On an ordinary tailpipe that would
     * be a small error, because the end sits near ambient and inflow is brief. On a short duct
     * with a wide mouth it would be a *feedback*: the mouth empties the duct to well below
     * ambient, which draws air in, and each parcel would arrive at
     * `T_interior * (p_amb/p)^((gamma-1)/gamma)` — so a hot duct inhales hotter gas and gets
     * hotter still.
     *
     * With the duct's entropy on inflow, a V8 with a 0.2 m, 42-to-130 mm cone collector at
     * 7000 rpm falls to 0.109 bar and reaches 40,586 K at a four-hundredth of ambient density.
     * Mass leaves while energy does not, which is the signature of gas being replaced by hotter
     * gas rather than of a conservation error; only a closed mouth, which has no inflow, is
     * immune.
     *
     * Physically this is the subsonic-inflow condition: `u + 2c/(gamma-1)` comes out of the duct
     * along the one outgoing characteristic, while pressure and entropy are imposed from the
     * reservoir outside. The two incoming characteristics must carry the reservoir's properties,
     * which is what this imposes.
     *
     * The outside air keeps the exhaust `gamma` rather than `gammaAir`: the solver carries one
     * ratio of specific heats, and a boundary that disagreed with the interior about it would
     * break the Riemann problem for the sake of a few percent in the inflow temperature.
     */
    if (uGhost < 0) {
      /**
       * Density only. The velocity keeps the outgoing-invariant relation above, on purpose.
       *
       * What the interior-entropy ghost gets wrong for arriving gas is its *temperature*, not
       * its velocity. The two obvious ways of also taking the velocity from outside both fail,
       * and instructively:
       *
       *  - recomputing the invariant from an ambient-entropy `cGhost` produces +1600 m/s of
       *    spurious outflow, because ambient air is cool and dense so `cGhost` falls far below
       *    `c` and `2/(gamma-1)` multiplies the difference by six. It turns healthy ducts into
       *    runaways.
       *  - using the exterior acoustic relation `dp / (rho_amb c_amb)` destroys the reflection.
       *    Near an open end `pGhost` is close to ambient by construction, so that expression goes
       *    to zero exactly when the reflection needs a large inflow velocity, and the measured
       *    reflection coefficient at ka = 0.04 falls from 0.99 to 0.732.
       *
       * The velocity has to come from the duct's own outgoing characteristic either way, and in
       * the linear limit the invariant form and the acoustic `(pPlus - pMinus) / zc` agree, which
       * is what keeps `|R|` right. Only the entropy is imposed from outside.
       */
      rGhost = Math.max(AMBIENT_RHO * Math.pow(pGhost / GAS.pAmb, INV_GAMMA), 1e-7);
    }

    hllc(r, u, p, rGhost, uGhost, pGhost, n, this.f0, this.f1, this.f2, this.fp);

    // Volume flow leaving the mouth, from the mass flux the Riemann solver produced.
    this.mouthFlowOut = (this.f0[n]! * this.areaFace[n]!) / r;
  }

  /**
   * Refresh the thermal coefficients: the Nusselt correlation and the resulting per-sample
   * decay factors.
   *
   * This is the expensive half — a `pow` per cell — so it runs occasionally. The cheap half
   * runs every sample, which is the whole point: applying the *transfer* in batches injects
   * a periodic energy perturbation at `sampleRate / interval`, which at 16 samples is
   * exactly 3000 Hz. That artefact would be audible as a whistle in any geometry with a
   * resonance near it, and would take a third of the total energy above 1.5 kHz in the
   * expansion-chamber preset. Note that it is invisible to a grid-refinement test, because
   * the batch interval does not scale with cell size.
   */
  private refreshThermalCoefficients(dtSample: number): void {
    const K_GAS = 0.05; // W/(m K), hot exhaust
    const MU = 3.5e-5; // Pa s
    // Dittus-Boelter's Pr^n takes n = 0.3 for a fluid being cooled, which exhaust gas in a
    // pipe always is.
    const PR_N = 0.899; // Pr^0.3 at Pr = 0.7

    this.fluxAvgC = 1 - Math.exp(-dtSample / FLUX_AVERAGE_TAU);

    for (let i = 0; i < this.n; i++) {
      const r = this.rho[i]!;
      const d = this.hydDia[i]!;
      // On the *averaged* mass flux, not the instantaneous one — see `FLUX_AVERAGE_TAU`.
      const re = (this.fluxAvg[i]! * d) / MU;
      const nu = Math.max(
        0.023 * Math.pow(re, 0.8) * PR_N * PULSATION_NUSSELT,
        NUSSELT_FLOOR,
      );
      const htcVol = ((nu * K_GAS) / d) * (4 / d); // W/(m^3 K)
      const tau = (r * CV) / Math.max(htcVol, 1e-9);
      this.gasDecay[i] = Math.exp(-dtSample / tau);
    }
  }

  /**
   * Gas/wall/ambient exchange, applied every sample using the cached coefficients.
   *
   *   gas  --h_in-->  wall  --convection + radiation-->  ambient
   *
   * The heat leaving the gas is *given to* the wall rather than discarded, so the pair
   * conserves energy instead of relaxing independently toward unrelated setpoints.
   *
   * Radiation is not optional: oxidised steel at 800 K radiates about 18 kW/m^2, equivalent
   * to h = 36 W/(m^2 K), several times natural convection.
   */
  private applyThermal(): void {
    // Gas side only, every sample. Done in internal energy rather than temperature:
    // relaxing T needs three divides per cell, this needs one.
    for (let i = 0; i < this.n; i++) {
      const r = this.rho[i]!;
      const m = this.mom[i]!;
      const kinetic = (0.5 * m * m) / r;
      const eInt = this.en[i]! - kinetic;
      const eWall = r * CV * this.wallT[i]!;
      const eIntNew = eWall + (eInt - eWall) * this.gasDecay[i]!;
      this.en[i] = eIntNew + kinetic;
      // Banked for the wall, which is updated in batches.
      this.qBanked[i] += (eInt - eIntNew) * this.cellVolume[i]!;
      // Mass flux average feeding the correlation. Here rather than in the batched
      // refresh because the point of it is to see every sample, not one in sixteen.
      this.fluxAvg[i] += this.fluxAvgC * (Math.abs(m) - this.fluxAvg[i]!);
    }
  }

  /**
   * Wall side, applied in batches.
   *
   * Unlike the gas relaxation this may be batched safely. The wall moves by only about
   * 0.007 K per sixteen samples against a gas-to-wall difference of hundreds of kelvin, so
   * stepping it contributes a perturbation five orders of magnitude below the one that
   * batching the *gas* transfer would produce. Keeping it out of the per-sample path removes a
   * fourth power and a divide per cell.
   */
  private applyWallThermal(dt: number): void {
    const SIGMA_EPS = 5.67e-8 * 0.8; // Stefan-Boltzmann times emissivity of oxidised steel
    for (let i = 0; i < this.n; i++) {
      const tw = this.wallT[i]!;
      const qOut =
        (this.hExt[i]! * (tw - GAS.tAmb) + SIGMA_EPS * (tw * tw * tw * tw - AMB4)) *
        this.outerArea[i]! *
        dt;
      this.wallT[i] = clamp(
        tw + (this.qBanked[i]! - qOut) * this.invWallHeatCapacity[i]!,
        GAS.tAmb,
        1600,
      );
      this.qBanked[i] = 0;
    }
  }

  /**
   * External heat transfer coefficient, from air moving past the pipe.
   *
   * Natural convection off a hot horizontal cylinder is around 9 W/(m^2 K); forced
   * convection uses Zukauskas's cross-flow correlation for 10^3 < Re < 2x10^5,
   * `Nu = 0.26 Re^0.6 Pr^0.37`, giving about 41 W/(m^2 K) at 5 m/s and 118 at 25 m/s. Recomputed only when the air speed or the geometry changes,
   * so its cost is irrelevant.
   */
  setAirSpeed(airSpeed: number): void {
    const K_AIR = 0.026; // W/(m K) at ambient
    const NU_AIR = 1.5e-5; // m^2/s
    const PR_037 = 0.881; // Pr^0.37 at Pr = 0.71
    const v = Math.max(airSpeed, 0);
    for (let i = 0; i < this.n; i++) {
      // The diameter of a round pipe with the same outer wall, which is what the correlation is for.
      const dOut = this.shapeCell[i]! * this.diaCell[i]! + 2 * this.wallThickness;
      const re = (v * dOut) / NU_AIR;
      const forced = re > 1 ? (K_AIR / dOut) * 0.26 * Math.pow(re, 0.6) * PR_037 : 0;
      this.hExt[i] = Math.max(9, forced);
    }
  }

  /**
   * Acoustic state at one end of the duct, for a junction to solve against.
   *
   * Returns the wave *travelling toward* that end (`toward`), the local specific impedance
   * and area, and the local sound speed. A junction needs nothing else: it finds the common
   * pressure from these, then hands each duct back the wave returning into it.
   *
   * The reconstruction arrays are only valid inside a step, so this must be called between
   * `beginStep` and `endStep`.
   */
  endState(end: 'inlet' | 'outlet'): EndState {
    /**
     * Filled into a preallocated object, one per end.
     *
     * This is called several times per duct per substep — the junction solve needs it for the
     * common-pressure sum, for the pressure bounds, for the mixed node temperature, for each
     * trial Riemann solve and again for the merge noise — so returning a fresh object each time
     * would put a steady stream of short-lived garbage on the audio thread. Two objects per duct, one
     * per end, cannot alias: no caller holds an inlet and an outlet state of the *same* duct at
     * once, and different ducts have different objects.
     *
     * The returned object is only valid until the next call for the same end on the same duct.
     */
    const out = end === 'inlet' ? this.inletState : this.outletState;
    const i = end === 'inlet' ? 0 : this.n - 1;
    const r = end === 'inlet' ? this.lr[i]! : this.rr[i]!;
    const u = end === 'inlet' ? this.lu[i]! : this.ru[i]!;
    const p = end === 'inlet' ? this.lp[i]! : this.rp[i]!;
    const c = Math.sqrt((GAMMA * p) / r);
    const rhoC = r * c;
    // At an outlet the wave heading for the junction travels with +x; at an inlet it
    // travels against it.
    const toward =
      end === 'outlet' ? 0.5 * (p - GAS.pAmb + rhoC * u) : 0.5 * (p - GAS.pAmb - rhoC * u);
    out.toward = toward;
    out.rhoC = rhoC;
    out.area = this.areaCell[i]!;
    out.c = c;
    out.rho = r;
    out.p = p;
    out.u = u;
    return out;
  }

  /**
   * Write this duct's end state into slot `i` of a junction kernel's block, and return it.
   *
   * Done here, reading and writing typed arrays, so no value crosses a call on the way: the junction
   * solve's whole point in wasm is that nothing floating-point is passed as an argument.
   */
  fillJunctionEnd(end: 'inlet' | 'outlet', k: Record<JunctionField, Float64Array>, i: number): EndState {
    const st = this.endState(end);
    k.rho[i] = st.rho;
    k.u[i] = st.u;
    k.p[i] = st.p;
    k.toward[i] = st.toward;
    k.rhoC[i] = st.rhoC;
    k.area[i] = st.area;
    k.c[i] = st.c;
    k.faceArea[i] = this.areaFace[end === 'inlet' ? 0 : this.n]!;
    return st;
  }

  /** Take slot `i`'s committed junction fluxes and diagnostics back into this duct's face. */
  takeJunctionFlux(end: 'inlet' | 'outlet', k: Record<JunctionField, Float64Array>, i: number): void {
    const face = end === 'inlet' ? 0 : this.n;
    this.f0[face] = k.f0[i]!;
    this.f1[face] = k.f1[i]!;
    this.f2[face] = k.f2[i]!;
    this.fp[face] = k.fp[i]!;
    this.junctionClamps += k.clamps[i]! | 0;
    this.supersonicFaces += k.supersonic[i]! | 0;
  }

  private readonly inletState: EndState = { toward: 0, rhoC: 0, area: 0, c: 0, rho: 0, p: 0, u: 0 };
  private readonly outletState: EndState = { toward: 0, rhoC: 0, area: 0, c: 0, rho: 0, p: 0, u: 0 };

  /**
   * Impose a junction pressure at one end: form the ghost state carrying the returning
   * wave and let the Riemann solver produce the flux, exactly as the mouth does.
   *
   * @returns mass flux through that face, kg/s, positive in the duct's +x direction.
   */
  applyJunction(
    end: 'inlet' | 'outlet',
    junctionGauge: number,
    junctionTemp: number,
    pre?: EndState,
  ): number {
    return this.junctionFlux(end, junctionGauge, junctionTemp, true, pre);
  }

  /**
   * Mass flux this duct *would* pass at a trial junction pressure, committing nothing.
   *
   * Lets the node find the pressure at which its branches actually balance, instead of taking the
   * linearised estimate and hoping. The Riemann solve goes into a one-element scratch buffer, so
   * the real flux arrays are untouched and no diagnostic counter moves.
   */
  probeJunction(
    end: 'inlet' | 'outlet',
    junctionGauge: number,
    junctionTemp: number,
    pre?: EndState,
  ): number {
    return this.junctionFlux(end, junctionGauge, junctionTemp, false, pre);
  }

  private junctionFlux(
    end: 'inlet' | 'outlet',
    junctionGauge: number,
    junctionTemp: number,
    commit: boolean,
    pre?: EndState,
  ): number {
    // The reconstruction does not change while a junction is being solved, so a caller that
    // already has this duct's end state can pass it in rather than paying for it per trial.
    const st = pre ?? this.endState(end);
    const face = end === 'inlet' ? 0 : this.n;

    // Supersonic outflow into the junction, for the same reason as the mouth: a choked end
    // cannot be told what pressure to be at. Its flow is fixed by the interior, and imposing a
    // downstream pressure against outgoing characteristics invents energy at the boundary.
    //
    // The junction's own pressure solve still counts this duct's `toward` wave, which is not
    // strictly meaningful once the end is choked, so conservation at the junction is
    // approximate for as long as that lasts. That is the honest trade: a choked duct really is
    // insensitive to downstream pressure, and a slightly inconsistent junction pressure for a
    // few substeps is a far smaller error than inventing energy at the boundary.
    const cEnd = Math.sqrt(
      (GAMMA * Math.max(st.p, MIN_JUNCTION_P)) / Math.max(st.rho, MIN_JUNCTION_RHO),
    );
    const outward = end === 'outlet' ? st.u : -st.u;
    if (outward >= cEnd) {
      if (commit) this.supersonicFaces++;
      if (!commit) {
        hllc(st.rho, st.u, st.p, st.rho, st.u, st.p, 0, this.pf0, this.pf1, this.pf2, this.pfp);
        return this.pf0[0]! * this.areaFace[face]!;
      }
      hllc(st.rho, st.u, st.p, st.rho, st.u, st.p, face, this.f0, this.f1, this.f2, this.fp);
      return this.f0[face]! * this.areaFace[face]!;
    }

    const returning = junctionGauge - st.toward;
    const pGhost = Math.max(GAS.pAmb + junctionGauge, 1e-3);
    /**
     * Ghost velocity, clamped to a few times the local sound speed.
     *
     * Without the clamp this is how the solver dies. A collector much wider than the primaries
     * feeding it reflects a strong expansion back up whichever pipe is blowing down, which can
     * drive the cell at the valve toward vacuum. The density floor then leaves `rhoC` at
     * something like 1e-7, and dividing the returning wave by it produces a velocity of 1e9 —
     * which the Riemann solver turns into Infinity, then NaN, and from there the whole duct is
     * poisoned and the audio thread goes silent. Without the clamp, a four-into-one with a 53 mm
     * collector on 29 mm primaries takes 235,000 recoveries in a quarter of a second.
     *
     * Five times the sound speed is far outside anything physical for duct flow, so the clamp
     * never engages on a valid state; it only stops an already-degenerate one from becoming
     * arithmetic garbage.
     */
    if (commit && (st.rho < MIN_JUNCTION_RHO || st.rhoC < MIN_JUNCTION_RHO * ambientSoundSpeed())) {
      this.junctionClamps++;
    }
    const rhoSafe = Math.max(st.rho, MIN_JUNCTION_RHO);
    const cLocal = Math.sqrt((GAMMA * Math.max(st.p, MIN_JUNCTION_P)) / rhoSafe);
    // Two ceilings, and the second is the one that matters.
    //
    // `5 c` alone would leave a hole. It is computed from the *local* density, so on a
    // degenerate cell it scales with the degeneracy: at an arithmetic 1e-7 floor the local sound
    // speed comes out near 10^6 m/s and a limit of five times that clamps nothing. A V8 with a
    // chamber against its collector inlet would reach u = 1.2e7 m/s and p = 2e35 bar through
    // exactly this gap. `DESIGN_WAVE_SPEED` is a fixed, physical ceiling — already 17% above the fastest
    // `|u| + c` measured anywhere in this solver — so it cannot be inflated by the very state
    // it is supposed to contain.
    const uLimit = Math.min(5 * cLocal, DESIGN_WAVE_SPEED);
    // Likewise physical: the smallest `rho c` real gas in a duct can present, rather than an
    // arithmetic 1e-4 that turns a returning wave into a velocity of 10^9.
    const rhoC = Math.max(st.rhoC, MIN_JUNCTION_RHO * ambientSoundSpeed());
    const uRaw = end === 'outlet' ? (st.toward - returning) / rhoC : (returning - st.toward) / rhoC;
    if (commit && Math.abs(uRaw) > uLimit) this.junctionClamps++;
    const uGhost = clamp(uRaw, -uLimit, uLimit);
    /**
     * Entropy of the ghost: this duct's own when gas is leaving, the junction's when it arrives.
     *
     * The same distinction the open end draws. Scaling the interior state isentropically to the boundary
     * pressure describes gas on its way *out* correctly and gas on its way *in* not at all — it
     * hands the arriving parcel the receiving duct's temperature, so a hot duct inhales hotter
     * gas and climbs. For an inflowing ghost the honest state is the junction's own: the mixed
     * stagnation temperature of the branches emptying into it, at the junction pressure. The
     * ghost moves, so its static temperature is that stagnation temperature less the kinetic
     * part, `T0 - u^2 / 2cp`; taken as static, every parcel would arrive with its kinetic
     * energy counted twice.
     */
    const inflow = end === 'outlet' ? uGhost < 0 : uGhost > 0;
    const rGhost = inflow
      ? Math.max(
          pGhost / (GAS.R * Math.max(junctionTemp - (uGhost * uGhost) / (2 * CP), GAS.tAmb)),
          MIN_JUNCTION_RHO,
        )
      : Math.max(st.rho * Math.pow(pGhost / st.p, INV_GAMMA), MIN_JUNCTION_RHO);

    const area = this.areaFace[face]!;
    if (!commit) {
      // Into scratch, at index 0, so nothing observable changes.
      const { pf0, pf1, pf2, pfp } = this;
      if (end === 'inlet') hllc(rGhost, uGhost, pGhost, st.rho, st.u, st.p, 0, pf0, pf1, pf2, pfp);
      else hllc(st.rho, st.u, st.p, rGhost, uGhost, pGhost, 0, pf0, pf1, pf2, pfp);
      return pf0[0]! * area;
    }
    if (end === 'inlet') {
      hllc(rGhost, uGhost, pGhost, st.rho, st.u, st.p, face, this.f0, this.f1, this.f2, this.fp);
    } else {
      hllc(st.rho, st.u, st.p, rGhost, uGhost, pGhost, face, this.f0, this.f1, this.f2, this.fp);
    }
    return this.f0[face]! * area;
  }

  /** Cross-sectional area of the outlet face, m^2. */
  get outletArea(): number {
    return this.areaFace[this.n]!;
  }

  /** Mass flux through a face, kg/s, positive in the duct's +x direction. */
  faceMassFlux(face: number): number {
    return this.f0[face]! * this.areaFace[face]!;
  }

  /** Energy flux through a face, W, positive in the duct's +x direction. */
  faceEnergyFlux(face: number): number {
    return this.f2[face]! * this.areaFace[face]!;
  }

  /** Mean wall temperature, K. */
  meanWallTemp(): number {
    let sum = 0;
    for (let i = 0; i < this.n; i++) sum += this.wallT[i]!;
    return this.n > 0 ? sum / this.n : GAS.tAmb;
  }

  /** Wall temperature along the visible pipe, K. */
  sampleWallTemperature(out: Float32Array): void {
    for (let k = 0; k < this.tapIndex.length; k++) out[k] = this.wallT[this.tapIndex[k]!]!;
  }

  /** Wall temperatures by normalised position, so a rebuild can inherit the thermal state. */
  exportWall(): Float64Array {
    return this.wallT.slice();
  }

  reset(): void {
    this.mouthPhi = 0;
    this.mouthCutState = 0;
    this.crossModes?.reset();
  }
}

// ---------------------------------------------------------------------------
// Numerics
// ---------------------------------------------------------------------------

/** Limiter as a small integer, so the inner loop compares numbers rather than strings. */
const enum Limiter {
  Mc = 0,
  Minmod = 1,
  VanLeer = 2,
}

function limiterCodeOf(kind: SlopeLimiter): Limiter {
  if (kind === 'minmod') return Limiter.Minmod;
  if (kind === 'vanleer') return Limiter.VanLeer;
  return Limiter.Mc;
}

/**
 * TVD slope limiter. Returns zero across an extremum, which is what suppresses the
 * oscillations an unlimited second-order scheme produces at a steep front.
 *
 * `mc` (monotonized central) is the default: measured over 1.3 m of pipe it loses
 * 4.5 dB at 5 kHz where `minmod` loses 12.4 dB, and both show zero overshoot on the
 * Sod problem — so minmod's extra diffusion buys nothing here.
 *
 * Called three times per cell per substep, hence the integer discriminant: `reconstructTs`
 * switches on it once per step, around three loops that each call one limiter directly.
 */
function minmod(a: number, b: number): number {
  if (a * b <= 0) return 0;
  return Math.abs(a) < Math.abs(b) ? a : b;
}

function vanLeer(a: number, b: number): number {
  if (a * b <= 0) return 0;
  return (2 * a * b) / (a + b);
}

/** Monotonised central. */
function mc(a: number, b: number): number {
  if (a * b <= 0) return 0;
  const c = 0.5 * (a + b);
  const ac = Math.abs(c);
  const m = Math.min(ac, Math.min(2 * Math.abs(a), 2 * Math.abs(b)));
  return c < 0 ? -m : m;
}


/**
 * The six states `hllc` is solving between, passed through here rather than as arguments.
 *
 * A function too large for the engine to inline takes its arguments boxed: every floating-point value
 * handed to it becomes a fresh heap object. With junctions solved in TypeScript (`useJunctionKernel`
 * off), `hllc` is called about fifty times a sample at a V8's junctions alone, and taking boxed
 * arguments would allocate 270 MB a second that way — most of the audio thread's garbage, and the
 * time spent allocating it on top. A typed array holds the values unboxed, so the call that is not
 * inlined takes only integers and arrays, and the one that is — `hllc` itself — is small enough to be.
 */
const HLLC_IN = new Float64Array(6);

/**
 * HLLC approximate Riemann solver (Toro). Resolves the contact discontinuity as well
 * as the two acoustic waves, which matters here because the hot slug leaving the
 * cylinder *is* a contact discontinuity — HLL alone smears it and loses the
 * temperature structure that sets the local speed of sound.
 */
function hllc(
  rL: number,
  uL: number,
  pL: number,
  rR: number,
  uR: number,
  pR: number,
  idx: number,
  f0: Float64Array,
  f1: Float64Array,
  f2: Float64Array,
  fp: Float64Array,
): void {
  HLLC_IN[0] = rL;
  HLLC_IN[1] = uL;
  HLLC_IN[2] = pL;
  HLLC_IN[3] = rR;
  HLLC_IN[4] = uR;
  HLLC_IN[5] = pR;
  hllcSolve(idx, f0, f1, f2, fp);
}

function hllcSolve(
  idx: number,
  f0: Float64Array,
  f1: Float64Array,
  f2: Float64Array,
  fp: Float64Array,
): void {
  const rL = HLLC_IN[0]!;
  const uL = HLLC_IN[1]!;
  const pL = HLLC_IN[2]!;
  const rR = HLLC_IN[3]!;
  const uR = HLLC_IN[4]!;
  const pR = HLLC_IN[5]!;
  // Reciprocal densities, reused below for the star-region energy flux.
  const invRL = 1 / rL;
  const invRR = 1 / rR;
  const cL = Math.sqrt(GAMMA * pL * invRL);
  const cR = Math.sqrt(GAMMA * pR * invRR);
  const eL = pL * INV_GM1 + 0.5 * rL * uL * uL;
  const eR = pR * INV_GM1 + 0.5 * rR * uR * uR;

  // Davis wave-speed estimates: cheap and safe (they bound the true speeds).
  const sL = Math.min(uL - cL, uR - cR);
  const sR = Math.max(uL + cL, uR + cR);

  if (sL >= 0) {
    f0[idx] = rL * uL;
    f1[idx] = rL * uL * uL + pL;
    f2[idx] = (eL + pL) * uL;
    fp[idx] = pL;
    return;
  }
  if (sR <= 0) {
    f0[idx] = rR * uR;
    f1[idx] = rR * uR * uR + pR;
    f2[idx] = (eR + pR) * uR;
    fp[idx] = pR;
    return;
  }

  const mL = rL * (sL - uL);
  const mR = rR * (sR - uR);
  const denom = mL - mR;
  const sStar = Math.abs(denom) < 1e-12 ? 0 : (pR - pL + mL * uL - mR * uR) / denom;

  // Star-region pressure, from Rankine-Hugoniot across the left acoustic wave.
  fp[idx] = Math.max(pL + mL * (sStar - uL), 1e-3);

  if (sStar >= 0) {
    const f = mL / (sL - sStar);
    f0[idx] = rL * uL + sL * (f - rL);
    f1[idx] = rL * uL * uL + pL + sL * (f * sStar - rL * uL);
    f2[idx] = (eL + pL) * uL + sL * (f * (eL * invRL + (sStar - uL) * (sStar + pL / mL)) - eL);
  } else {
    const f = mR / (sR - sStar);
    f0[idx] = rR * uR + sR * (f - rR);
    f1[idx] = rR * uR * uR + pR + sR * (f * sStar - rR * uR);
    f2[idx] = (eR + pR) * uR + sR * (f * (eR * invRR + (sStar - uR) * (sStar + pR / mR)) - eR);
  }
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/** Open-end length correction coefficient, as a multiple of the mouth radius. */
export const OPEN_END_FACTOR = 0.6133;

/**
 * Fewest cells a duct is ever discretised into.
 *
 * Small, and the reason is cost rather than accuracy. Every duct is marched in lockstep, so the
 * finest grid in the system sets the substep count for all of them — and a *short* duct hits this
 * floor first. A 0.24 m collector forced to eight cells would come out at `dx = 30 mm`, under the
 * 34.3 mm the CFL pin needs for one substep, so the whole engine would pay two substeps because one
 * short duct could not be discretised coarsely enough: on a V8 with a 0.2 m cone collector, 96 cells
 * at two substeps and 80-86% of a core, against 53-68% for every longer collector.
 *
 * So few cells is crude, but a 0.24 m duct has correspondingly little to say: its quarter-wave sits
 * above where the plane-wave assumption holds for a wide mouth anyway. The cost budget picks the
 * finest grid that fits regardless, so this floor only binds on ducts short enough that it should.
 *
 * Three rather than four, for the same reason one step further. A manifold of straight tube is
 * built from lengths one pin spacing long — 148 mm on a V8 — and at four cells those would come out
 * at 37 mm, just under the 37.3 mm one substep needs at 44.1 kHz, so on such a machine the whole V8
 * would pay two substeps and the budget would coarsen every other duct to afford them. At three they
 * are 49 mm, which holds one substep to about 60 kHz.
 */
const MIN_DUCT_CELLS = 3;

/**
 * Cells a duct of this discretised length gets at a given target cell size.
 *
 * Exported because the cost budget in `EngineSim` has to predict it *exactly*. The rounding here
 * is what decides the realised `dx`, and `dx` decides the substep count — so a budget that
 * assumed `dx === cellSize` could pick a grid one rounding step below a substep threshold and
 * silently pay double. See `budgetedCellSize`.
 */
export function ductCellCount(
  length: number,
  cellSize: number,
  maxCells: number,
  minDx = 0,
): number {
  let count = clamp(Math.round(length / cellSize), MIN_DUCT_CELLS, maxCells);
  /**
   * Never so many cells that one is too small for a single step per sample.
   *
   * Every duct is marched in lockstep, so one duct with cells under `minDx` would make the whole exhaust
   * take two steps per sample. A short duct therefore gets fewer, longer cells — below the usual floor if
   * it must, down to one — which is crude for that duct and costs nothing anywhere else.
   */
  if (minDx > 0) count = Math.min(count, Math.max(1, Math.floor(length / minDx)));
  return count | 0;
}

/**
 * The smallest cell that still takes one step per audio sample, m.
 *
 * The CFL limit at the design wave speed, with a hair of margin so a cell exactly this long is not tipped
 * into two steps by rounding.
 */
export function singleStepDx(sampleRate: number, cfl: number): number {
  return (DESIGN_WAVE_SPEED / (sampleRate * cfl)) * (1 + 1e-9);
}

/**
 * Substeps per audio sample a duct with this cell size is pinned to.
 *
 * One definition, used by `EulerPipe` to set its own count and by the cost budget to predict it.
 */
export function pinnedSubstepsFor(
  dx: number,
  sampleRate: number,
  cfl: number,
  maxSubsteps: number,
): number {
  const designLimit = (cfl * dx) / DESIGN_WAVE_SPEED;
  return Math.min(Math.max(Math.ceil(1 / sampleRate / designLimit), 1), maxSubsteps);
}

/** Discretised length of a duct, m: its segments and an optional head port. */
export function ductGridLength(
  pipe: PipeSegment[],
  port?: { length: number; diameter: number },
): number {
  const segments = pipe.filter((s) => s.length > 1e-4 && s.dIn > 1e-3);
  if (segments.length === 0) return 0.1;
  let total = segments.reduce((a, s) => a + s.length, 0);
  if (port && port.length > 1e-4 && port.diameter > 1e-3) total += port.length;
  return total;
}

interface BuiltGeometry {
  count: number;
  dx: number;
  length: number;
  portCells: number;
  areaCell: Float64Array;
  areaFace: Float64Array;
  diaCell: Float64Array;
  /**
   * Wetted perimeter per cell over `pi * diaCell`. Kept as a ratio, exactly 1 where the duct is round,
   * so a round duct's wall and friction terms come out bit for bit as they would without it.
   */
  shapeCell: Float64Array;
  chambers: ChamberPlacement[];
  /** `launchRadius` of the drawn duct over its drawn mouth radius: 1 unless the mouth is a gradual flare. */
  launchRadiusRatio: number;
  /** Signed contraction loss per cell, 1/m. See `EulerPipe.contractionK`. */
  contractionK: Float64Array;
}

/**
 * Loss coefficient of a contraction, referred to the velocity in the narrow pipe: Crane TP-410's
 * reducer formula. `halfAngle` is the half-angle of the taper, radians (pi/2 for a step), and
 * `areaRatio` the narrow area over the wide one.
 *
 * A step loses `0.5 (1 - A2/A1)`, the classic sudden-contraction figure. A gentle taper loses far less,
 * since the flow follows the wall instead of separating from it into a vena contracta.
 */
export function contractionLoss(halfAngle: number, areaRatio: number): number {
  const open = 1 - clamp(areaRatio, 0, 1);
  const s = Math.sin(clamp(halfAngle, 0, Math.PI / 2));
  return halfAngle <= Math.PI / 8 ? 0.8 * s * open : 0.5 * open * Math.sqrt(s);
}

/**
 * Per-cell contraction loss for a duct, 1/m, signed by the flow direction each applies to.
 *
 * Each run of cells that narrows in one direction is one contraction. Its coefficient comes from the
 * *drawn* profile over that run, not the grid's: the grid spreads a step over several cells to keep
 * the area ratio per cell bounded, and read off the grid a step would look like a taper. The run's
 * `K` is then shared out so that in steady flow the cells between them drop `K rho u^2 / 2` at the
 * narrow end's velocity: a cell of area `A` carries `(A_narrow / A)` of that velocity, so its share is
 * weighted by `(A / A_narrow)^2`. Applied as `du/dt = -K_i u|u| / (2 dx)`.
 */
function contractionLossCells(
  areaFace: Float64Array,
  areaCell: Float64Array,
  dx: number,
  drawnDiameter: (x: number) => number,
  total: number,
): Float64Array {
  const count = areaCell.length;
  const out = new Float64Array(count);
  const narrows = (i: number, dir: number): boolean =>
    dir > 0
      ? areaFace[i + 1]! < areaFace[i]! * (1 - 1e-9)
      : areaFace[i]! < areaFace[i + 1]! * (1 - 1e-9);
  for (const dir of [1, -1]) {
    let i = 0;
    while (i < count) {
      if (!narrows(i, dir)) {
        i++;
        continue;
      }
      let j = i;
      while (j < count && narrows(j, dir)) j++;
      // Cells i..j-1, faces i..j.
      const aWide = dir > 0 ? areaFace[i]! : areaFace[j]!;
      const aNarrow = dir > 0 ? areaFace[j]! : areaFace[i]!;

      // The drawn taper over the same stretch, a cell either side: total fall in diameter over the
      // length it falls across. A step falls across one sample, which reads as close to a right angle.
      const x0 = Math.max((i - 1) * dx, 0);
      const x1 = Math.min((j + 1) * dx, total - 1e-9);
      const SAMPLES = 64;
      const h = (x1 - x0) / SAMPLES;
      let fall = 0;
      let fallLength = 0;
      let prev = drawnDiameter(dir > 0 ? x0 : x1);
      for (let k = 1; k <= SAMPLES; k++) {
        const d = drawnDiameter(dir > 0 ? x0 + k * h : x1 - k * h);
        if (d < prev - 1e-12) {
          fall += prev - d;
          fallLength += h;
        }
        prev = d;
      }
      const halfAngle =
        fall > 0 ? Math.atan(fall / (2 * Math.max(fallLength, 1e-9))) : Math.atan(
          (Math.sqrt(aWide) - Math.sqrt(aNarrow)) / (Math.sqrt(Math.PI) * (j - i) * dx),
        );
      const k = contractionLoss(halfAngle, aNarrow / aWide);
      const share = k / (j - i);
      for (let c = i; c < j; c++) {
        const r = areaCell[c]! / aNarrow;
        out[c] = (dir * share * r * r) / (2 * dx);
      }
      i = j;
    }
  }
  return out;
}

/**
 * Half-angle, radians, above which a widening cone launches higher modes as a step would.
 *
 * Below it the wall turns slowly enough that a wave entering the cone stays in its lowest mode, a
 * spherical front, all the way to the mouth; above it the flare is short against the wavelengths
 * near cut-on and behaves as an abrupt expansion. The value is a judgement, not a derived figure:
 * every drawn megaphone and collector cone sits under 5 degrees, and a "cone" steeper than 15 is
 * in effect a step in any case.
 */
const HORN_HALF_ANGLE = (15 * Math.PI) / 180;

/**
 * Radius, m, that sets where the wave reaching the mouth stops being plane.
 *
 * Higher modes are launched where the duct widens abruptly: a step, a steep cone, the inlet. A
 * gradual flare does not launch them, so the plane-wave band a megaphone carries out of its mouth is
 * that of its throat, not of its mouth. Only the final widening run matters. Upstream of its
 * narrowest point, any higher mode is cut off by the constriction and only the plane wave gets
 * through, as with the tailpipe after a muffler.
 *
 * Walks the drawn profile back from the mouth: through the run where the duct only widens toward the
 * mouth, taking the largest radius at any abrupt widening, and the throat where the run begins. For
 * a duct ending in straight pipe after a step, or in straight pipe throughout, that is the mouth.
 */
export function launchRadius(segments: PipeSegment[]): number {
  // The profile as pieces from inlet to mouth: `[dA, dB, length]`, a zero length being a step.
  const pieces: Array<[number, number, number]> = [];
  let prev = -1;
  for (const seg of segments) {
    const dIn = segmentDiameter(seg, 0);
    if (prev > 0 && Math.abs(prev - dIn) > 1e-9) pieces.push([prev, dIn, 0]);
    if (seg.kind === 'chamber') {
      const body = segmentDiameter(seg, 0.5);
      const throat = CHAMBER_THROAT * seg.length;
      pieces.push([seg.dIn, seg.dIn, throat], [seg.dIn, body, 0], [body, body, seg.length - 2 * throat]);
      pieces.push([body, seg.dIn, 0], [seg.dIn, seg.dIn, throat]);
    } else {
      pieces.push([dIn, segmentDiameter(seg, 1), seg.length]);
    }
    prev = segmentDiameter(seg, 1);
  }

  let launch = 0;
  let throat = Infinity;
  for (let k = pieces.length - 1; k >= 0; k--) {
    const [dA, dB, len] = pieces[k]!;
    // Upstream is wider: this piece's outlet is the throat of the final run.
    if (dA > dB + 1e-9) break;
    throat = Math.min(throat, dA);
    const abrupt = len <= 0 || Math.atan((dB - dA) / (2 * len)) > HORN_HALF_ANGLE;
    if (dB > dA + 1e-9 && abrupt) launch = Math.max(launch, dB);
  }
  return Math.max(launch, Number.isFinite(throat) ? throat : 0) / 2;
}

/**
 * Discretise the duct into uniform cells.
 *
 * Cell *length* is fixed rather than cell count, so a short pipe costs less than a long
 * one and — more importantly — the CFL timestep does not shrink when the user shortens
 * the pipe. Fixing the count instead would make a 0.5 m system demand a far higher
 * update rate than a 1.5 m one, for no gain.
 */
function buildGeometry(
  pipe: PipeSegment[],
  port: { length: number; diameter: number } | undefined,
  cellSize: number,
  maxCells: number,
  /** Smallest cell that keeps one step per sample. See `ductCellCount`. */
  minDx: number,
  /**
   * Which ends are junctions, and the combined area of whatever feeds an inlet junction, so
   * the discretisation can guarantee the inlet is not a severe contraction.
   */
  ends: {
    inlet: 'valve' | 'junction';
    outlet: 'mouth' | 'junction';
    feedArea: number;
  },
): BuiltGeometry {
  const segments = pipe.filter((s) => s.length > 1e-4 && s.dIn > 1e-3);
  if (segments.length === 0) {
    segments.push({
      id: '__stub',
      kind: 'pipe',
      length: 0.1,
      dIn: 0.04,
      dOut: 0.04,
      yaw: 0,
      pitch: 0,
    });
  }

  const hasPort = !!port && port.length > 1e-4 && port.diameter > 1e-3;
  if (hasPort) {
    segments.unshift({
      id: '__port',
      kind: 'pipe',
      length: port!.length,
      dIn: port!.diameter,
      dOut: port!.diameter,
      yaw: 0,
      pitch: 0,
    });
  }

  // No open-end correction is added to the grid: the air outside the mouth that moves with
  // the air in it is the mouth's radiation inertance, in `mouthBoundary`.
  const lengths = segments.map((s) => s.length);
  const lastIdx = segments.length - 1;

  const total = lengths.reduce((a, b) => a + b, 0);
  const count = ductCellCount(total, cellSize, maxCells, minDx);
  const dx = total / count;

  const areaCell = new Float64Array(count);
  const diaCell = new Float64Array(count);
  const areaFace = new Float64Array(count + 1);

  // Cumulative segment starts, for mapping x to a segment.
  const starts: number[] = [];
  let acc = 0;
  for (const l of lengths) {
    starts.push(acc);
    acc += l;
  }

  // Segment index and normalised position at `x`, written into `at` so nothing is allocated per face.
  const at = { si: 0, u: 0 };
  const locate = (x: number): void => {
    let si = segments.length - 1;
    for (let k = 0; k < segments.length; k++) {
      if (x < starts[k]! + lengths[k]!) {
        si = k;
        break;
      }
    }
    at.si = si;
    at.u = clamp((x - starts[si]!) / Math.max(lengths[si]!, 1e-9), 0, 1);
  };
  const diameterAt = (x: number): number => {
    locate(x);
    return segmentDiameter(segments[at.si]!, at.u);
  };

  const chambers: ChamberPlacement[] = [];
  segments.forEach((seg, k) => {
    if (seg.kind !== 'chamber') return;
    const [offIn, offOut] = chamberOffsets(seg);
    chambers.push({
      section: chamberBody(seg),
      xIn: starts[k]! + CHAMBER_THROAT * lengths[k]!,
      xOut: starts[k]! + (1 - CHAMBER_THROAT) * lengths[k]!,
      inlet: { offset: offIn, diameter: seg.dIn },
      outlet: { offset: offOut, diameter: seg.dIn },
    });
  });

  // Each face takes the drawn area averaged over the half-cells either side of it, not the area at
  // the face itself. Point-sampled, a step lands wholly on one side of whichever face it falls
  // between, so a chamber's volume would come out quantised to whole cells: up to a cell's worth long or
  // short, which on a 35 mm grid is a tenth of a street muffler. Averaged, the face straddling an
  // edge takes the share of it that lies on each side, and the drawn volume is kept.
  const FACE_SAMPLES = 16;
  for (let f = 0; f <= count; f++) {
    const x0 = Math.max((f - 0.5) * dx, 0);
    const x1 = Math.min((f + 0.5) * dx, total - 1e-9);
    const h = (x1 - x0) / FACE_SAMPLES;
    let sum = 0;
    for (let k = 0; k < FACE_SAMPLES; k++) {
      const d = diameterAt(x0 + (k + 0.5) * h);
      sum += (Math.PI * d * d) / 4;
    }
    areaFace[f] = sum / FACE_SAMPLES;
  }

  const MAX_RATIO = 1.6;

  /**
   * Smallest a junction-fed inlet may be, as a fraction of the combined area of the ducts
   * feeding it.
   *
   * This is the variable that decides whether a collector is solvable at all, and it is set by
   * measurement rather than reasoning. A junction imposes a pressure and lets each duct find
   * its own flux; if the collector's inlet is far narrower than the pipes emptying into it,
   * that boundary is being asked to pass several pipes' worth of flow through one pipe's
   * area. The cell inside over-expands toward vacuum, and the returning wave divided by a
   * collapsed `rho c` is where the blow-up comes from.
   *
   * Measured on a V8 without this floor, four 42 mm primaries into one collector, chamber
   * width and everything else held fixed, counting `junctionClamps` over two seconds at
   * 4000 rpm wide open:
   *
   *     inlet    ratio to feeding area    clamps
   *      42 mm          0.25              188,275
   *      48 mm          0.33              171,509
   *      55 mm          0.43              136,517
   *      65 mm          0.60                    0
   *      75 mm          0.80                    0
   *      90 mm          1.15                    0
   *
   * The cliff sits between 0.43 and 0.60, so 0.6 is the floor with nothing to spare and
   * nothing wasted. It is also the physically honest answer: four 42 mm pipes cannot merge
   * into a 42 mm hole without an enormous restriction, so a drawing that says they do is
   * asking for something that does not exist, and rounding it up to something that does is
   * more faithful than solving it as drawn.
   *
   * Limiting the area *gradient* near the junction instead is the wrong variable: six cells
   * of settling cut the clamps only from 188,275 to 104,085 while halving
   * the width of the user's chamber, where widening the inlet by one or two cells removes them
   * entirely and leaves the chamber exactly as drawn.
   */
  const JUNCTION_INLET_FRACTION = 0.6;

  if (ends.inlet === 'junction' && ends.feedArea > 0) {
    const floor = JUNCTION_INLET_FRACTION * ends.feedArea;
    // Decaying back to the drawn area at `MAX_RATIO` per cell, so the result already satisfies
    // the ratio limit below rather than being flattened by it on the next pass.
    for (let f = 0; f <= count; f++) {
      const want = floor / Math.pow(MAX_RATIO, f);
      if (want <= areaFace[f]!) break;
      areaFace[f] = want;
    }
  }

  // Limit how fast area may change from one cell to the next.
  //
  // This is not only a stability measure. Quasi-1D theory assumes the area varies slowly
  // compared with the duct radius; where it does not, the flow is genuinely
  // two-dimensional and a 1D model has no claim on it. Numerically, an unlimited step
  // puts a large `p dA/dx` momentum source in a single cell and can drive the state
  // inadmissible — without the limit, a 34 mm port necking to an 8 mm tailpipe over 8 cells
  // produces NaN within four samples.
  //
  // A chamber's step expansion still reads as a step: spread over three or four cells
  // it remains acoustically abrupt for everything below a few kHz, so mufflers keep
  // reflecting as they should.
  limitAreaRatio(areaFace, MAX_RATIO);

  const shapeCell = new Float64Array(count);
  for (let i = 0; i < count; i++) {
    // Cell area as the mean of its faces, so the cell and face geometry stay
    // consistent after limiting.
    const a = 0.5 * (areaFace[i]! + areaFace[i + 1]!);
    areaCell[i] = a;
    diaCell[i] = Math.sqrt((4 * a) / Math.PI);
    // The drawn shape's perimeter over its equivalent circle's, applied to the limited area: a flat
    // can has more wall per unit of area than a round one, and wall is what friction and heat act on.
    locate(Math.min((i + 0.5) * dx, total - 1e-9));
    const section = segmentSection(segments[at.si]!, at.u);
    shapeCell[i] =
      section.section === 'round'
        ? 1
        : sectionPerimeter(section) / (Math.PI * Math.sqrt((4 * sectionArea(section)) / Math.PI));
  }

  const portCells = hasPort ? Math.min(count - 1, Math.round(lengths[0]! / dx)) : 0;

  // Exactly 1 for any duct that does not end in a gradual flare, so those keep their limit bit for bit.
  const drawnMouth = segmentDiameter(segments[lastIdx]!, 1) / 2;
  const launch = launchRadius(segments);
  const launchRadiusRatio = launch < drawnMouth ? launch / drawnMouth : 1;

  const contractionK = contractionLossCells(areaFace, areaCell, dx, diameterAt, total);

  return {
    count,
    dx,
    length: total,
    portCells,
    areaCell,
    areaFace,
    diaCell,
    shapeCell,
    chambers,
    launchRadiusRatio,
    contractionK,
  };
}

/**
 * Limit the face-to-face area ratio to `maxRatio`, in place, without changing the duct's volume.
 *
 * Shrinking only the larger side of a step is wrong for a chamber. On a 35 mm grid a 42 -> 130 mm
 * muffler needs about five cells to ramp up and five to ramp down, and taking all of them out of the
 * body turns the street muffler's 0.34 m can into a diamond peaking at 108 mm with 39% of its drawn
 * volume. Volume is what sets a chamber's low-frequency attenuation, so it would muffle far less than
 * the drawing says.
 *
 * So take both one-sided limits — `lo`, which only shrinks the large side (the ramp entirely
 * inside the chamber), and `hi`, which only widens the small side (entirely in the pipes around it)
 * — and blend them geometrically, `lo^(1-t) hi^t`. Both satisfy the ratio limit, so any blend does
 * too, being a convex combination in log-area. Each stretch of faces the limit touched gets its own
 * `t`, found by bisection so that stretch holds exactly the volume drawn. The step ends up where
 * the drawing's volume says it is, straddling the drawn edge, rather than wholly on one side of it.
 *
 * Faces the limit did not touch are left exactly as drawn, and they separate the stretches, so
 * each stretch can take its own `t` without breaking the limit where it meets the next.
 */
export function limitAreaRatio(areaFace: Float64Array, maxRatio: number): void {
  const n = areaFace.length;
  if (n < 2) return;
  const lo = Float64Array.from(areaFace);
  const hi = Float64Array.from(areaFace);
  // Two sweeps give the exact envelope: min over g of A[g] R^|f-g|, and max of A[g] / R^|f-g|.
  for (let f = 1; f < n; f++) {
    lo[f] = Math.min(lo[f]!, lo[f - 1]! * maxRatio);
    hi[f] = Math.max(hi[f]!, hi[f - 1]! / maxRatio);
  }
  for (let f = n - 2; f >= 0; f--) {
    lo[f] = Math.min(lo[f]!, lo[f + 1]! * maxRatio);
    hi[f] = Math.max(hi[f]!, hi[f + 1]! / maxRatio);
  }

  // A face's share of the volume, in cells: the end faces bound one cell, the rest two.
  const weight = (f: number) => (f === 0 || f === n - 1 ? 0.5 : 1);
  const touched = (f: number) => hi[f]! > lo[f]! * (1 + 1e-12);

  let f = 0;
  while (f < n) {
    if (!touched(f)) {
      f++;
      continue;
    }
    const start = f;
    while (f < n && touched(f)) f++;
    const end = f; // exclusive

    let target = 0;
    for (let g = start; g < end; g++) target += weight(g) * areaFace[g]!;
    const volumeAt = (t: number): number => {
      let v = 0;
      for (let g = start; g < end; g++) v += weight(g) * lo[g]! * Math.pow(hi[g]! / lo[g]!, t);
      return v;
    };

    // Volume rises monotonically with t, from at most the drawn volume at 0 to at least it at 1.
    let a = 0;
    let b = 1;
    for (let it = 0; it < 40; it++) {
      const m = 0.5 * (a + b);
      if (volumeAt(m) < target) a = m;
      else b = m;
    }
    const t = 0.5 * (a + b);
    for (let g = start; g < end; g++) areaFace[g] = lo[g]! * Math.pow(hi[g]! / lo[g]!, t);
  }
}
