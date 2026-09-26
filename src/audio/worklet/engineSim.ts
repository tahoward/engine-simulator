/**
 * The simulation itself: cylinder + valves + exhaust gas dynamics + radiation, stepped
 * one audio sample at a time.
 *
 * Deliberately free of any AudioWorklet or DOM reference so the whole thing can be
 * driven from Node in tests. `processor.ts` is only a thin shell around this.
 *
 * Signal chain per sample:
 *
 *   crank angle -> valve lift -> valve flow area
 *        |                            |
 *        |                  orifice flow against the port pressure
 *        |                            |
 *   cylinder gas state  <-------------+------> volume flow into the pipe
 *                                                        |
 *              quasi-1D Euler solver, MUSCL-Hancock (the user's exhaust)
 *                                                        |
 *                                    mouth flow -> d/dt -> far-field pressure
 */

import {
  CV_REF,
  CV_SLOPE,
  FUEL_CUT_RPM,
  FUEL_CUT_THROTTLE,
  FUEL_RESUME_RPM,
  GAS,
  T_REF,
  PIPE_PRESSURE_TAPS,
  REV_LIMIT_HYSTERESIS_RPM,
  type EngineConfig,
  type EngineSnapshot,
  type EngineSpec,
  type BankSnapshot,
  type DynoConfig,
  type PipeSegment,
  displacement,
  exhaustLayoutOf,
  fullLoadTorque,
  exhaustPortDiameter,
  fuelFractionAt,
  loadTorqueOf,
  ambientSoundSpeed,
  firingPlan,
  physicalBankCount,
} from '../../model/spec.js';
import { Delay, Impact, Noise, Resonator, clamp, softClip, wrapCycle } from './dsp.js';
import {
  ADVANCE_IO_SIZE,
  CYL_BURNED,
  CYL_FUEL,
  CYL_PRESSURE,
  CYL_STATE_SIZE,
  CYL_TEMP,
  Cylinder,
  IO_DT,
  IO_EX,
  IO_IN,
  IO_INTAKE_BURNED,
  IO_INTAKE_FUEL,
  IO_INTAKE_T,
  IO_OMEGA,
  IO_PORT_T,
} from './cylinder.js';
import { Listener } from './listener.js';
import { FarField } from './radiation.js';
import {
  DEFAULT_CFL,
  DEFAULT_MAX_CELLS,
  ductCellCount,
  ductGridLength,
  singleStepDx,
  type EulerPipeOptions,
  type ValveState,
} from './eulerPipe.js';
import { compileLayout, nodeOrder, validateGraph, type ExhaustGraph } from '../../model/exhaustGraph.js';
import { ExhaustSystem } from './exhaustSystem.js';
import { IntakePlenum } from './plenum.js';
import { DynoRun } from './drivetrain.js';
import { ORIFICE_IO, orificeSolve, VALVE_CD, valveFlowArea, valveLift } from './valve.js';

/** Slots of `EngineSim.intakeIo`. */
const INTAKE_P_PLENUM = 0;
const INTAKE_T_PLENUM = 1;
const INTAKE_GAMMA_PLENUM = 2;
const INTAKE_AREA = 3;
const INTAKE_BREATHING = 4;
const INTAKE_WEIGHT = 5;
const INTAKE_MDOT = 6;
const INTAKE_IO_SIZE = 7;

/**
 * Slots of `EngineSim.intakeAcc`: net valve flow, and the back-flow's mass, mass*T, mass*burned and
 * mass*fuel.
 */
const ACC_FLOW = 0;
const ACC_BACK_MASS = 1;
const ACC_BACK_ENERGY = 2;
const ACC_BACK_BURNED = 3;
const ACC_BACK_FUEL = 4;
const ACC_SIZE = 5;

/**
 * Pressure, in pascals, that maps to digital full scale. An open-piped single at
 * 1.5 m genuinely produces peaks around 40-80 Pa (125-132 dB SPL), so 250 Pa leaves
 * headroom for someone fitting an absurdly short pipe without the output pinning.
 */
const PA_PER_FULLSCALE = 250;

/**
 * Longest a finished dyno run waits for the engine to wind down to a held speed, s, before handing
 * the crank back regardless.
 */
const DYNO_WIND_DOWN = 6;

/** Crank degrees per cylinder sub-step. Keeps the gas integration well resolved at high rpm. */
const MAX_DEG_PER_SUBSTEP = 0.35;

/**
 * Largest fraction of the trapped mass that may cross the valves in one cylinder sub-step.
 *
 * The stiffness guard for the energy equation, which divides by that mass. Without it a
 * nearly-scavenged cylinder integrates unstably.
 */
const MAX_MASS_FRACTION_PER_SUBSTEP = 0.05;

/** Ceiling on cylinder sub-steps, so a pathological instant cannot stall the audio thread. */
const MAX_CYL_SUBSTEPS = 64;

/** Idle floor, rev/min. Below this the free-running model is allowed to stall out to here rather than to zero. */
const MIN_RPM = 450;

/**
 * Peak structure-borne levels at 1 m, in pascals, at `mechNoise = 1`.
 *
 * Together with `STRUCTURE_PA_PER_GPA_S` these are the numbers in the mechanical model
 * chosen by ear rather than derived, so they are stated in physical units and kept
 * together. Calibrated so the whole structure-borne family sits roughly 18 dB below
 * what an open header radiates, matching measured engines — which is why it only becomes
 * audible once a muffler has quietened the exhaust, exactly as on a real bike.
 */
const CLACK_PA_AT_1M = 6;
const SLAP_PA_AT_1M = 3.5;

/**
 * RMS turbulent fluctuation of the *plane-wave* volume velocity, as a fraction of the
 * mean flow through the valve.
 *
 * Much smaller than the ~15% turbulence intensity measured locally in a valve jet,
 * because only the cross-section average couples to a plane wave and incoherent eddies
 * largely cancel in that average.
 */
const TURBULENCE_INTENSITY = 0.1;

/**
 * The engine the structure-borne frequencies below were set by ear against: the default single,
 * 89 mm bore, 34 mm exhaust valve, half a litre in its one cylinder.
 *
 * Everything that rings is scaled from these. A structure's resonances go as the inverse of its size
 * when its shape is kept, so each engine rings at frequencies set by the size of its own parts rather
 * than at the single's.
 */
/** One cylinder's swept volume on the reference engine, m^3. */
const REFERENCE_DISPLACEMENT_M3 = 4.977e-4;
const REFERENCE_BORE = 0.089;
const REFERENCE_EX_VALVE = 0.034;

/** Valve-seating ring and piston-slap ring on the reference engine, [Hz, Q]. */
const CLACK_MODE: readonly [number, number] = [2700, 14];
const SLAP_MODE: readonly [number, number] = [620, 9];

/**
 * How far each cylinder's own head and bore ring from the nominal, as a fraction, peak.
 *
 * A valve seating rings the part of the head over that cylinder, not the engine as a whole, and an end
 * cylinder's surroundings are not a middle one's: different ribs, different bolt bosses, a different path
 * out to the air. With one shared ring for every cylinder, a many-cylinder engine's clacks would sum
 * coherently into a steady tone at the ring's frequency. This is geometry, so it does not follow `cylinderSpread`,
 * which is about manufacturing.
 */
const LOCAL_MODE_DETUNE = 0.06;

/**
 * Cylinder `b` of `n`'s place in an even spread over [-1, 1], shuffled by a fixed permutation.
 *
 * The shuffle is `b * step + offset` modulo `n`, which is only a permutation if `step` shares no factor
 * with `n`. The steps asked for are 3, 5 and 7, which suit 2, 4 and 8 cylinders but not every count: a
 * five with a step of 5, or a three or six with a step of 3, would give every cylinder the *same* place —
 * no spread at all, so identically breathing cylinders, perfect cancellation, and none of the unevenness
 * a real engine has. So the step is moved on to the next one that works; a step that already works is
 * used as it is.
 */
export function spreadOf(b: number, n: number, step: number, offset: number): number {
  if (n <= 1) return 0;
  const gcd = (a: number, c: number): number => (c === 0 ? a : gcd(c, a % c));
  let k = step;
  while (gcd(k, n) !== 1) k++;
  return (((b * k + offset) % n) / (n - 1)) * 2 - 1;
}

/**
 * How much quieter each valve's clack, or piston's slap, is when its casting is shared: the amplitude
 * factor for the number of cylinders sharing it.
 *
 * The clack level was set on a single, whose head is a casting over one cylinder. A bank of a V8 or an
 * inline four is one casting over four: the same seating impulse moves four times the mass a quarter as
 * far, and four times the area radiates that. Radiated power per event therefore goes as one over the
 * cylinders sharing the casting. Without it every valve would seat as loudly as a single's, and a V8's
 * sixteen closings a cycle would put its clack 11 dB above a single's and 15 dB above its own exhaust in
 * the top octaves.
 */
export function clackShare(cylindersPerHead: number): number {
  return 1 / Math.sqrt(Math.max(cylindersPerHead, 1));
}

/**
 * Structural modes of the block and head, [Hz, Q]. A real casting has hundreds; four
 * spread across the range where engine noise actually lives is enough to read as metal
 * rather than as a filtered click.
 */
const STRUCTURAL_MODES: ReadonlyArray<readonly [number, number]> = [
  [780, 11],
  [1550, 14],
  [2900, 17],
  [4700, 20],
];

/**
 * Structural radiation at 1 m per GPa/s of cylinder pressure rise, Pa.
 *
 * Driving this from dp/dt rather than from crank angle is what makes the mechanical
 * character respond correctly: advance the ignition or shorten the burn and the engine
 * audibly hardens, because a steeper pressure rise really does hit the casing harder.
 */
const STRUCTURE_PA_PER_GPA_S = 0.55;

/**
 * Per-cylinder valve-timing spread at `cylinderSpread = 1`, crank degrees peak.
 *
 * Cam lobe machining, valvetrain lash and combustion phasing together put a real engine's
 * cylinders a couple of degrees apart. Conservative for a production engine, and it is the
 * dominant reason the cancelled orders are 20 to 30 dB down in a real engine rather than 45.
 */
const CAM_SPREAD_DEG = 2.2;

/**
 * Time constant, s, of the mean-torque tracker and of the rpm readout's smoothing. The crank-speed
 * ripple integrator leaks over four of them.
 */
const IRREGULARITY_TAU = 0.12;

/**
 * Why a grid budget, and why it is the only lever on the solver's cost.
 *
 * Coarsening the grid is not free. A crossplane V8's ducts total about 9 m, so 150 cells is a 60 mm
 * grid, and `resolutionCutoffRad` puts the solver's faithful limit near 2 kHz — against 3 kHz at
 * 20 mm. A V8 fundamental is 147 Hz with harmonics well past 2 kHz, so this is a real trade and not
 * merely a safety margin.
 *
 * It is also, unfortunately, the only lever there is. Two alternatives look cheaper and are not, for
 * structural reasons:
 *
 *  - Per-duct cell sizes, fine in the collector and coarse in the primaries. Under a tight budget a
 *    uniform grid already leaves the primaries near the hard floor of 8 cells, so there is no slack
 *    to move; a split hits the floor and *adds* cells instead. See `budgetedCellSize`.
 *  - Running the gas solver at half the audio rate and interpolating back up. This cannot
 *    save anything, because the solver's step count is set by CFL, not by the output rate:
 *    the step must be under `0.85 * dx / 1400`, which is 36.7 us on a 60 mm grid, so halving
 *    the rate to a 41.7 us period simply forces two substeps where there was one: 48,000 steps
 *    per second either way, and 96,000 either way on a 20 mm grid.
 *
 * Both of those follow from the same fact. Solver cost is `cells * steps`, and `dx` sets both
 * — which is why the note on `DEFAULT_CELL_SIZE` says cost goes as the inverse *square* of
 * cell size. Grid coarseness, duct length and duct count are the only things that move it.
 */
/**
 * The solver's cost budget, and what a cylinder and a junction cost, all in pipe cells.
 *
 * Measured by timing every preset at 4000 rpm with one step per sample, and fitting cost to what each
 * engine is made of. The fit is good to 1.3 points of a core across all eleven presets:
 *
 *     about 6.6% of a core per cylinder, 1.7% per junction, 0.064% per pipe cell
 *
 * So a cylinder is worth about 102 cells and a junction about 27. The junction term matters: an exhaust
 * of straight tube snapping together is mostly junctions — a V8 has six — and a budget counting only
 * cylinders and cells would take them as free, when a V8's cost more than all its cells together. Cells,
 * meanwhile, are cheap, and a budget that overpriced them would coarsen a V8's grid to save a few points
 * of a core.
 *
 * The total is set so the costliest engine, a V8, sits near 75% of a core on the machine this was
 * measured on, with room left for everything smaller to have its finest grid.
 */
const CYLINDER_COST_IN_CELLS = 102;
const JUNCTION_COST_IN_CELLS = 27;
const SOLVER_COST_BUDGET = 1216;

/** Cells of pipe the budget leaves for an engine with this many cylinders and junctions. */
export function gridBudgetCells(cylinders: number, junctions: number): number {
  return SOLVER_COST_BUDGET - CYLINDER_COST_IN_CELLS * cylinders - JUNCTION_COST_IN_CELLS * junctions;
}

/**
 * Bandwidth of the combustion pressure-rise drive, Hz, and (`STRUCTURE_LIMIT_HZ`, after the
 * contact durations) the band limit on everything structure-borne, Hz.
 *
 * The first is physical: a combustion pressure rise occupies ten to thirty crank degrees,
 * about a millisecond, so 1.5 kHz is generous for it and anything above is the finite
 * difference amplifying integrator jitter.
 *
 * The second sits deliberately above the highest structural mode at 4.7 kHz, so it removes
 * the impulse skirts and the numerical debris without touching the clack, the slap or any
 * mode the model actually has. Two poles, because one sheds too little to matter against a
 * flat-to-Nyquist impulse.
 */
const DPDT_BANDWIDTH_HZ = 1500;

/**
 * Contact durations of the two impacts, seconds.
 *
 * A valve seats fast and hard; a piston crossing its clearance is heavier and slower. Both
 * sit well above the mode they drive (a 0.15 ms raised cosine is 6 dB down at 6.7 kHz, against
 * a 2.7 kHz clack), so they take the Dirac's excess treble off without dulling the impact.
 */
const VALVE_CONTACT_S = 0.00015;
const SLAP_CONTACT_S = 0.0004;
const STRUCTURE_LIMIT_HZ = 6000;

export class EngineSim {
  private spec: EngineSpec;
  private pipe: PipeSegment[];
  private collectorPipe: PipeSegment[];
  private wg: ExhaustSystem;
  /** One per cylinder. All share a crank, so their angles differ by a fixed offset. */
  private cyls: Cylinder[];
  /** One per radiating mouth, each with its own mouth's radiation corner and band limit. */
  private farFields: FarField[] = [];
  /**
   * Throat turbulence, one generator per cylinder.
   *
   * Sharing one generator would be subtly wrong: it is drawn only while that cylinder's
   * exhaust valve is open, so the draw sequence one bank sees would depend on the other's
   * valve timing. Two cylinders' turbulence is physically independent, and a shared
   * generator also makes any A/B comparison between banks impossible to interpret.
   */
  private readonly throatNoise: Noise[] = [];

  /**
   * Structure-borne rings for valve seating and piston slap, one of each per cylinder.
   *
   * Per cylinder because each is local: a valve seats in the head over its own cylinder and a piston
   * slaps its own bore. See `LOCAL_MODE_DETUNE`.
   */
  private clack: Resonator[] = [];
  private slap: Resonator[] = [];
  /** Finite-duration force pulses driving them. See `Impact`. */
  private clackImpact: Impact[] = [];
  private slapImpact: Impact[] = [];
  /** Per-event clack and slap level for how many cylinders share each casting. See `clackShare`. */
  private headShare = 1;
  /** Piston slaps triggered since construction. For tests and diagnostics. */
  slapCount = 0;
  /** Each cylinder's exhaust lift, intake lift and exhaust flow area this sample. See `computeLifts`. */
  private liftNow: Float64Array = new Float64Array(3);
  /** Inputs to `Cylinder.advanceIo`, reused for every cylinder. */
  private readonly cylIo = new Float64Array(ADVANCE_IO_SIZE);
  /** Inputs and output of `intakeFlowNow`, at the `INTAKE_*` slots. */
  private readonly intakeIo = new Float64Array(INTAKE_IO_SIZE);
  /** What the plenum is handed this sample, at the `ACC_*` slots. */
  private readonly intakeAcc = new Float64Array(ACC_SIZE);
  /** Each cylinder's pressure, temperature and burned fraction at the start of this sample. See `Cylinder.readState`. */
  private cylState: Float64Array = new Float64Array(CYL_STATE_SIZE);
  /** The exhaust port's pressure, temperature and area, read through `EulerPipe.readPort`. */
  private readonly portState = new Float64Array(3);
  /** A cylinder's pressure when it crossed TDC this sample, or -1 if it did not. */
  private tdcPressure: Float64Array = new Float64Array(1);
  /**
   * Block and head structural modes, excited by the rate of cylinder pressure rise.
   * A real engine's hard, clattery attack is the casing ringing as combustion hits it,
   * so it is driven by dp/dt rather than triggered off a crank angle.
   */
  private readonly structure: Resonator[];
  /** Smoothed pressure-rise drive, and the two-pole band limit on the structure path. */
  private dpdtSmooth = 0;
  private readonly dpdtSmoothC: number;
  private readonly structureLpC: number;
  private structureLp1 = 0;
  private structureLp2 = 0;
  private readonly listener: Listener;
  /**
   * Per-mouth path delay and spreading loss, so several tailpipes do not sum coincidentally.
   * Sized on construction to the number of radiating mouths; see `refreshMouthPaths`.
   */
  private mouthDelays: Delay[] = [];
  private mouthGains: Float64Array = new Float64Array(1);
  /**
   * Per-cylinder multiplier on intake flow area, fixed for the life of the engine.
   *
   * This is what stops the cancellation being perfect. With identical cylinders an evenly firing
   * engine annihilates every order that is not a multiple of the cylinder count — measured 66 dB
   * down, where real engines sit 20-35 dB down — and the result is a pure tone on the firing
   * frequency with no rumble underneath it. It propagates properly from here, through trapped mass
   * to heat release to pulse strength, rather than being added as noise.
   *
   * Applied as a multiplier on the *runner pressure* each cylinder sees, which is the loss down an
   * unequal-length intake runner. Scaling each cylinder's intake valve *area* instead moves the
   * low orders by 1 dB and no more, because the cylinder equilibrates
   * toward plenum pressure by the time the valve shuts however wide the valve was: area changes
   * the filling rate, not the trapped mass. Pressure changes the trapped mass directly.
   */
  private breathing: Float64Array = new Float64Array(1);
  /**
   * Per-cylinder valve-timing offset, crank degrees, fixed for the life of the engine.
   *
   * The other half of why an engine is never as clean as the arithmetic says. Cancelling an order
   * needs the contributions to match in *both* amplitude and phase, and a couple of degrees of
   * phase error breaks it more effectively than a few percent of amplitude error. Cam lobes are
   * machined to a tolerance, valve lash differs between cylinders, and combustion phasing varies
   * by several degrees, so no two cylinders put their exhaust pulse in quite the same place.
   *
   * On the valve timing rather than on the firing angles, because crank pins are machined to
   * arc-minutes — the crank really is that accurate, and the camshaft and the flame are not.
   */
  private timing: Float64Array = new Float64Array(1);

  /** Previous sample's valve mass flow per bank, kg/s. Scales the turbulence injected next. */
  private lastValveMdot!: Float64Array;
  /** Reused valve-state objects, so the hot path allocates nothing. */
  private valveStates!: ValveState[];
  /** Summed gas + inertia torque from the previous sample, N*m. */
  private torqueLast = 0;
  /** Per-bank scratch, sized to the cylinder count. */
  private exLift!: Float64Array;
  private inLift!: Float64Array;
  private seatingNow!: boolean[];
  private inSeatingNow!: boolean[];
  /** Valve-seating flow pulse into the port, one per cylinder. See `Impact`. */
  private seatPulse!: Impact[];
  /** CFL substeps the gas solver took last sample. Exposed for the load meter. */
  substeps = 0;

  /** Turbulence lowpass state per bank (two cascaded poles). */
  private turb1!: Float64Array;
  private turb2!: Float64Array;
  /** Instantaneous crank speed, rad/s — mean plus the within-cycle ripple. */
  private omega = 0;
  /** Mean crank speed, rad/s. What the rpm readout reports. */
  private omegaMean = 0;
  /** Within-cycle speed ripple, rad/s, from a leaky integration of the net torque. */
  private omegaRipple = 0;
  /** Slowly tracked mean torque, N*m, subtracted so the ripple carries no DC. */
  private torqueAvg = 0;
  /** Smoothed speed for the readout, rad/s. */
  private omegaDisplay = 0;
  /** Whether the rev limiter is cutting the spark. Latched, with hysteresis: see `revLimit`. */
  private limiterCut = false;
  /** Whether the overrun fuel cut has stopped the fuel. Latched, with hysteresis: see `fuelCut`. */
  private fuelCutActive = false;
  /**
   * The dyno run in progress, or `null`. While it runs it drives the throttle and loads the crank
   * through the clutch, and the crank is integrated whatever `freeRunning` says.
   */
  private dyno: DynoRun | null = null;
  /** Throttle opening the plenum was last set to by the dyno run; NaN once the spec's is back. */
  private dynoOpening = NaN;
  private prevExLift!: Float64Array;
  private prevInLift!: Float64Array;
  private prevAngle!: Float64Array;

  /** Short ramp used to mask the discontinuity when the pipe geometry is rebuilt. */
  private rebuildRamp = 1;
  private readonly rebuildRampStep: number;

  private peak = 0;
  private readonly tapBuffer = new Float32Array(PIPE_PRESSURE_TAPS);
  /**
   * Cached swept volume of the whole engine, m^3; only changes with bore, stroke or cylinder count.
   *
   * The whole engine's, because friction is charged against it. One cylinder's would give a V8 an
   * eighth of its friction: it would idle on a fifth of the air a real one needs, and so at a manifold
   * vacuum deep enough that most of what it trapped would be its own exhaust, back-flowed up the intake.
   */
  private displacementM3 = 0;
  /** Cached `loadTorqueOf(spec)`, N*m, so the per-sample path does not call for it. */
  private loadTorqueNm = 0;
  /** The finite intake manifold. See `IntakePlenum` for why it has to be finite. */
  private readonly plenum: IntakePlenum;

  /**
   * An explicit duct graph, or `null` to compile one from the layout spec.
   *
   * Set when the exhaust was *drawn* rather than chosen from the layout dropdown. The solver only ever
   * sees a graph either way; this is just which one.
   */
  private graph: ExhaustGraph | null;

  constructor(
    readonly sampleRate: number,
    config: EngineConfig,
    private readonly wgOptions: EulerPipeOptions = {},
    graph: ExhaustGraph | null = null,
  ) {
    /**
     * An explicit graph wins; otherwise the config's own, if it carried one.
     *
     * The config matters on first load: a shared link can contain a drawn exhaust, and without this the
     * worklet would build the *compiled* layout from `pipe`/`collector` and only pick the drawn one up
     * if something happened to call `setGraph` later.
     */
    this.graph = graph ?? config.graph ?? null;
    this.spec = { ...config.engine };
    this.displacementM3 = displacement(this.spec) * this.spec.cylinders;
    this.loadTorqueNm = loadTorqueOf(this.spec);
    this.pipe = config.pipe.map((s) => ({ ...s }));
    this.collectorPipe = (config.collector ?? []).map((s) => ({ ...s }));
    this.wg = this.buildExhaust();
    this.plenum = new IntakePlenum(this.spec);
    this.cyls = this.buildCylinders();
    this.allocatePerCylinder();
    this.structure = STRUCTURAL_MODES.map(
      ([hz, q]) => new Resonator(hz, q, sampleRate),
    );
    this.tuneStructure();
    this.listener = new Listener(sampleRate);
    this.listener.setGeometry({
      distance: this.spec.micDistance,
      micHeight: this.spec.micHeight,
      sourceHeight: this.spec.exhaustHeight,
      reflection: this.spec.groundReflection,
    });
    this.omegaMean = (Math.min(this.spec.rpm, this.spec.revLimit) * 2 * Math.PI) / 60;
    this.omega = this.omegaMean;
    this.omegaDisplay = this.omegaMean;
    // ~8 ms ramp.
    this.rebuildRampStep = 1 / (0.008 * sampleRate);
    this.dpdtSmoothC = 1 - Math.exp((-2 * Math.PI * DPDT_BANDWIDTH_HZ) / sampleRate);
    this.structureLpC = 1 - Math.exp((-2 * Math.PI * STRUCTURE_LIMIT_HZ) / sampleRate);
    this.refreshFarFields();
    this.refreshMouthPaths();
  }

  /** One far field per mouth, tuned to that mouth. Keeps existing filters' state. */
  private refreshFarFields(): void {
    const count = Math.max(1, this.wg.mouthCount);
    while (this.farFields.length < count) {
      this.farFields.push(new FarField(this.sampleRate, this.wg.mouthCutoffRadOf(this.farFields.length)));
    }
    this.farFields.length = count;
    for (let m = 0; m < count; m++) {
      this.farFields[m]!.setCutoff(this.wg.mouthCutoffRadOf(m), this.wg.bandLimitRadOf(m));
    }
  }

  // -------------------------------------------------------------------------
  // Configuration
  // -------------------------------------------------------------------------

  /**
   * The operating point alone: throttle, commanded speed and load.
   *
   * What the worklet calls every audio block, so it does only what these three reach and allocates
   * nothing. `setEngine` redoes the mouth paths, the structural modes and the firing plan, and makes
   * new arrays — fine for an edit, but not for every frame of a throttle drag on a thread with no time
   * to spare. Same effect as `setEngine({ throttle, rpm, load })`.
   */
  setControls(throttle: number, rpm: number, load: number): void {
    const spec = this.spec;
    if (throttle === spec.throttle && rpm === spec.rpm && load === spec.load) return;
    spec.throttle = throttle;
    spec.rpm = rpm;
    spec.load = load;
    this.loadTorqueNm = loadTorqueOf(spec);
    if (!spec.freeRunning && !this.integratingCrank()) this.omegaMean = (rpm * 2 * Math.PI) / 60;
    this.plenum.setGeometry(spec);
    this.dynoOpening = NaN;
  }

  /**
   * Start a dyno run through `config`'s gearbox, from the engine's present speed. See `DynoRun`.
   * A run already going is replaced.
   */
  startDyno(config: DynoConfig): void {
    this.dyno = new DynoRun(config, this.omegaMean, fullLoadTorque(this.spec));
    this.dynoOpening = NaN;
  }

  /** End the dyno run: the throttle goes back to the spec's once the engine has wound down. */
  stopDyno(): void {
    this.dyno?.finish();
  }

  setEngine(partial: Partial<EngineSpec>): void {
    const prevTemp = this.spec.portGasTemp;
    const prevPortLength = this.spec.portLength;
    const prevPortDia = exhaustPortDiameter(this.spec);
    const prevCellSize = this.spec.pipeCellSize;
    const prevWallThickness = this.spec.pipeWallThickness;
    const prevLayout = this.layoutKey();
    const prevPhase = firingPlan(this.spec).offsets.join(',');
    this.spec = { ...this.spec, ...partial };
    this.displacementM3 = displacement(this.spec) * this.spec.cylinders;
    this.loadTorqueNm = loadTorqueOf(this.spec);
    if (!this.spec.freeRunning) {
      // Fixed-rpm mode follows the slider directly, until it reaches the limiter; from there the crank
      // runs on from wherever it is. See `integratingCrank`.
      if (!this.integratingCrank()) this.omegaMean = (this.spec.rpm * 2 * Math.PI) / 60;
    }
    this.listener.setGeometry({
      distance: this.spec.micDistance,
      micHeight: this.spec.micHeight,
      sourceHeight: this.spec.exhaustHeight,
      reflection: this.spec.groundReflection,
    });
    this.refreshMouthPaths();
    this.wg.setTurbulence(this.spec.throatNoise);
    this.plenum.setGeometry(this.spec);
    this.dynoOpening = NaN;
    // Cheap enough to redo unconditionally, and it must not wait for a pipe rebuild.
    this.makeCylinderVariation(this.spec.cylinders);
    this.tuneStructure();
    // Port temperature sets the speed of sound, and the port itself is the first
    // length of the duct, so any of these changes the discretisation — as do the cell
    // size, the wall and the layout.
    if (
      this.spec.portGasTemp !== prevTemp ||
      this.spec.portLength !== prevPortLength ||
      exhaustPortDiameter(this.spec) !== prevPortDia ||
      this.spec.pipeCellSize !== prevCellSize ||
      this.spec.pipeWallThickness !== prevWallThickness ||
      this.layoutKey() !== prevLayout
    ) {
      if (this.layoutKey() !== prevLayout) {
        // A different engine: a run on the old one means nothing on it.
        this.dyno = null;
        this.allocatePerCylinder();
        this.cyls = this.buildCylinders();
      }
      this.rebuildPipe();
    } else if (firingPlan(this.spec).offsets.join(',') !== prevPhase) {
      // Re-phase in place. Rebuilding the cylinders would discard their gas state and audibly
      // restart the engine every time the slider moved.
      const plan = firingPlan(this.spec);
      for (let b = 1; b < this.cyls.length; b++) {
        this.cyls[b]!.angle = wrapCycle(this.cyls[0]!.angle - plan.offsets[b]!);
      }
    } else {
      // Air speed only changes a coefficient, so it needs no rebuild and keeps the
      // thermal state and the waves in flight.
      this.wg.setAirSpeed(this.spec.airSpeed);
    }
  }

  /** Replace the whole duct graph, for an exhaust that was drawn rather than chosen. */
  setGraph(graph: ExhaustGraph | null): void {
    this.graph = graph;
    this.rebuildPipe();
  }

  setPipe(pipe: PipeSegment[], collector?: PipeSegment[]): void {
    this.pipe = pipe.map((s) => ({ ...s }));
    if (collector) this.collectorPipe = collector.map((s) => ({ ...s }));
    this.rebuildPipe();
  }

  /**
   * Rebuilding allocates and loses the waves currently in flight. That is a
   * one-off cost paid only when the user edits the pipe, and the gain ramp hides
   * the discontinuity.
   */
  private rebuildPipe(): void {
    this.wg = this.buildExhaust();
    this.lastValveMdot.fill(0);
    // A new mouth diameter means both a new radiation corner and a new plane-wave limit.
    this.refreshFarFields();
    for (const f of this.farFields) f.reset();
    // The layout may have changed how many mouths there are.
    this.refreshMouthPaths();
    this.rebuildRamp = 0;
  }

  /**
   * Cell size that keeps the solver inside its cost budget.
   *
   * Every duct takes one step per audio sample (`singleStep`, set in `buildOptions`), so the grid
   * costs its cell count and nothing more, and what it may spend is `gridBudgetCells`: the part of
   * `SOLVER_COST_BUDGET` left after this engine's cylinders and junctions. The requested
   * `pipeCellSize` is floored at `singleStepDx`, 34.3 mm at 48 kHz, and used as it is if it fits.
   * Otherwise the finest coarser size that fits wins; if none does, the cheapest one found is used
   * rather than silently going over.
   *
   * The floor is what makes the cost a plain cell count. Solver cost is `cells * substeps`, and the
   * substep count is pinned by the grid: a duct needs `k` CFL substeps per audio sample unless
   * `dx >= DESIGN_WAVE_SPEED / (fs * cfl * k)`, which at 48 kHz is 34.3 mm for one substep and
   * 17.2 mm for two. Cost as a function of `dx` therefore has a *step* in it, and a budget that
   * counted cells alone would be blind to it.
   *
   * That blindness has a sharp edge. A V8 with 0.4 m primaries and a 0.5 m collector is 4.84 m of
   * duct, so a 150-cell rule would pick `4.84 / 150 = 32.3 mm` — two millimetres below the
   * threshold — and pay two substeps for 300 cell-steps where 34.3 mm buys 141 at one. Twice the
   * work for a 6% finer grid, which puts the preset at 94-101% of one core: it crosses 100%
   * somewhere around 5000-5500 rpm, and above that the audio thread simply cannot deliver, so
   * the sound cuts out and returns when the revs fall.
   *
   * `DEFAULT_MAX_CELLS`, which caps a single duct at 128, bounds nothing on a V8 on its own —
   * eight primaries and two collectors can come to 208 cells and 85% of one core, and the note on `DEFAULT_CELL_SIZE` in eulerPipe.ts calls 80-95% "unusable on an audio
   * thread". It is right: that is where the audio drops out, because 85% is the average and the
   * peaks go over.
   *
   * One size everywhere, and that is a measured conclusion rather than the obvious choice. The
   * tempting refinement is to give the collector a finer grid than the primaries, on the
   * reasoning that the collector is what radiates and what every cylinder's high-frequency
   * content must pass through. It does not pay: at a 150-cell budget a uniform grid already puts a
   * 0.54 m primary at 9 cells, one above the hard floor of 8, so there is no slack to move. Every
   * split ratio from 1.5x to 4x hits that floor and therefore *adds* cells rather than reallocating
   * them — 156 to 186 against a budget of 150 — so the fidelity it buys is bought with CPU, which
   * is the thing being saved. Measured against a 456-cell reference the worst per-octave error goes
   * from 8.4 dB (uniform, 150 cells) to 6.9 dB (2x split, 166 cells):
   * real, but no better than simply raising the budget to 166.
   */
  private budgetedCellSize(): number {
    const cfl = this.wgOptions.cfl ?? DEFAULT_CFL;
    const maxCells = this.wgOptions.maxCells ?? DEFAULT_MAX_CELLS;
    /**
     * One step per audio sample, always, so no cell is ever asked for below what allows it.
     *
     * Finer cells would only buy a higher band limit on the exhaust, and pay for it twice over — more
     * cells, each stepped more often. With every engine held to one step the cost is predictable and the
     * CPU is spent on cylinders and cells rather than on repeating the same cells.
     */
    const minDx = singleStepDx(this.sampleRate, cfl);
    const requested = Math.max(this.spec.pipeCellSize, minDx, 1e-4);

    // The ducts that will actually be built, with the lengths the discretiser will see.
    const port = { length: this.spec.portLength, diameter: exhaustPortDiameter(this.spec) };
    const graph = this.usableGraph();
    const lengths = graph.ducts.map((d) =>
      ductGridLength(d.segments, d.from.kind === 'valve' ? port : undefined),
    );
    if (lengths.length === 0) return requested;

    /** Cells at a candidate size: the whole cost, since every duct takes one step per sample. */
    const cellsAt = (cellSize: number): number =>
      lengths.reduce((a, l) => a + ductCellCount(l, cellSize, maxCells, minDx), 0);

    /** Budget for the grid, after the cylinders and junctions it cannot avoid. */
    const budgetFor = gridBudgetCells(this.spec.cylinders, nodeOrder(graph).length);

    /**
     * Scan coarser sizes and take the finest that fits.
     *
     * A scan rather than a closed form, because each duct rounds to a whole number of cells, so the
     * cost moves in steps as the size does. One percent steps up to 150 mm is a couple of hundred
     * evaluations of integer arithmetic, once per pipe edit.
     */
    let best = requested;
    let bestCost = cellsAt(requested);
    if (bestCost <= budgetFor) return requested;
    for (let dx = requested * 1.01; dx <= 0.15; dx *= 1.01) {
      const c = cellsAt(dx);
      if (c <= budgetFor) return dx;
      // Failing that, the cheapest available, so an extreme geometry degrades rather than
      // quietly running at twice real time.
      if (c < bestCost) {
        bestCost = c;
        best = dx;
      }
    }
    return best;
  }

  /** Solver options with the cylinder-head port prepended to the user's geometry. */
  private buildOptions(): EulerPipeOptions {
    return {
      cellSize: this.budgetedCellSize(),
      // One step per audio sample, for every engine. See `budgetedCellSize`.
      singleStep: true,
      wallThickness: this.spec.pipeWallThickness,
      airSpeed: this.spec.airSpeed,
      // Hand the wall's thermal state to the new duct. It takes tens of seconds to
      // build, so discarding it on every geometry edit would make the tuning jump.
      inheritWall: this.wg?.exportWall(),
      ...this.wgOptions,
      port: this.wgOptions.port ?? {
        length: this.spec.portLength,
        diameter: exhaustPortDiameter(this.spec),
      },
    };
  }

  get engine(): EngineSpec {
    return this.spec;
  }

  get pipeSolver(): ExhaustSystem {
    return this.wg;
  }

  /** Bank 0's cylinder. For the readouts and tests that look at one cylinder. */
  get cylinder(): Cylinder {
    return this.cyls[0]!;
  }

  get cylinders(): Cylinder[] {
    return this.cyls;
  }

  /**
   * What the structure rings at, Hz: the block's modes, and each cylinder's clack and slap.
   * For tests and diagnostics.
   */
  structuralFrequencies(): { block: number[]; clack: number[]; slap: number[] } {
    const hz = (r: Resonator) => r.frequency(this.sampleRate);
    return { block: this.structure.map(hz), clack: this.clack.map(hz), slap: this.slap.map(hz) };
  }

  /** Number of banks actually running. */
  get bankCount(): number {
    return this.cyls.length;
  }

  /**
   * Per-cylinder scratch state, sized to the current cylinder count.
   *
   * Called on construction and whenever the count changes. Everything here is indexed by
   * cylinder and reused every sample, so none of it may be allocated on the audio path.
   */
  private allocatePerCylinder(): void {
    const n = this.spec.cylinders;
    const z = () => new Float64Array(n);
    this.makeCylinderVariation(n);
    this.lastValveMdot = z();
    this.turb1 = z();
    this.turb2 = z();
    this.exLift = z();
    this.inLift = z();
    this.prevExLift = z();
    this.prevInLift = z();
    this.prevAngle = z();
    this.seatingNow = new Array<boolean>(n).fill(false);
    // Same finite contact time as the audible clack, for the same reason: a one-sample mass
    // pulse into a finite-volume cell is a Dirac, and the solver rings on it broadly.
    this.seatPulse = Array.from({ length: n }, () => new Impact(VALVE_CONTACT_S, this.sampleRate));
    this.inSeatingNow = new Array<boolean>(n).fill(false);
    this.tdcPressure = new Float64Array(n).fill(-1);
    this.liftNow = new Float64Array(3 * n);
    this.cylState = new Float64Array(CYL_STATE_SIZE * n);
    this.clack = Array.from({ length: n }, () => new Resonator(CLACK_MODE[0], CLACK_MODE[1], this.sampleRate));
    this.slap = Array.from({ length: n }, () => new Resonator(SLAP_MODE[0], SLAP_MODE[1], this.sampleRate));
    this.clackImpact = Array.from({ length: n }, () => new Impact(VALVE_CONTACT_S, this.sampleRate));
    this.slapImpact = Array.from({ length: n }, () => new Impact(SLAP_CONTACT_S, this.sampleRate));
    // `structure` is created after the first allocation, so tuning waits for it there.
    if (this.structure) this.tuneStructure();
    this.valveStates = Array.from({ length: n }, () => ({
      throatArea: 0,
      cylPressure: GAS.pAmb,
      cylTemp: GAS.tAmb,
      cylGamma: GAS.gammaExh,
      extraMassFlow: 0,
    }));
  }

  /**
   * Pitch everything that rings to the size of this engine.
   *
   * The block's modes scale with a cylinder's linear size, the cube root of its swept volume. The local
   * rings scale with the part that rings — the valve for a
   * seating clack, the bore for piston slap — each detuned a little per cylinder. Retuning keeps each
   * filter's state, so moving the bore slider does not click.
   */
  private tuneStructure(): void {
    const spec = this.spec;
    /**
     * Scaled by the size of one cylinder, not of the whole engine.
     *
     * What radiates from a block is its wall panels between the cylinder bulkheads, and a panel spans a
     * cylinder — so it is the cylinder's size that sets them. The block bending as a whole is far lower
     * but barely radiates: it is a stiff lump small against the wavelength. Scaling by the whole engine
     * instead would put a V8's lowest mode at 350 Hz, on its own second firing harmonic, and the casing
     * would boom 19 dB above the exhaust there — louder than anything a V8 block does.
     */
    const size = clamp(Math.cbrt(displacement(spec) / REFERENCE_DISPLACEMENT_M3), 0.5, 3);
    STRUCTURAL_MODES.forEach(([hz, q], i) => this.structure[i]!.set(hz / size, q, this.sampleRate));

    this.headShare = clackShare(spec.cylinders / physicalBankCount(spec));

    const n = this.clack.length;
    const valve = REFERENCE_EX_VALVE / Math.max(spec.exValveDia, 1e-3);
    const bore = REFERENCE_BORE / Math.max(spec.bore, 1e-3);
    for (let b = 0; b < n; b++) {
      // Evenly spread in [-1, 1] and shuffled, the same way as the breathing spread but on its own
      // permutation, so the cylinder whose head rings highest is not also the one that breathes best.
      const t = spreadOf(b, n, 7, 3);
      const u = spreadOf(b, n, 3, 2);
      this.clack[b]!.set(CLACK_MODE[0] * valve * (1 + LOCAL_MODE_DETUNE * t), CLACK_MODE[1], this.sampleRate);
      this.slap[b]!.set(SLAP_MODE[0] * bore * (1 + LOCAL_MODE_DETUNE * u), SLAP_MODE[1], this.sampleRate);
    }
  }

  /**
   * Intake valve mass flow of cylinder `b`, kg/s, positive into the cylinder, into
   * `intakeIo[INTAKE_MDOT]`, from the plenum state and valve area in `intakeIo` and the cylinder
   * state in `cylState`.
   *
   * The orifice equation takes the upstream gas's gamma: the plenum's charge on the way in, the
   * cylinder's own mixture on the way back out.
   */
  private intakeFlowNow(b: number): void {
    const io = this.intakeIo;
    const s = b * CYL_STATE_SIZE;
    const pRunner = io[INTAKE_P_PLENUM]! * io[INTAKE_BREATHING]!;
    const pCyl = this.cylState[s + CYL_PRESSURE]!;
    const orifice = ORIFICE_IO;
    orifice[0] = io[INTAKE_AREA]!;
    orifice[1] = VALVE_CD;
    if (orifice[0] <= 0 || pRunner === pCyl) {
      io[INTAKE_MDOT] = 0;
      return;
    }
    if (pRunner > pCyl) {
      orifice[2] = pRunner;
      orifice[3] = io[INTAKE_T_PLENUM]!;
      orifice[4] = pCyl;
      orifice[5] = io[INTAKE_GAMMA_PLENUM]!;
      orificeSolve(orifice);
      io[INTAKE_MDOT] = orifice[6]!;
      return;
    }
    orifice[2] = pCyl;
    orifice[3] = this.cylState[s + CYL_TEMP]!;
    orifice[4] = pRunner;
    orifice[5] = 1 + GAS.R / (CV_REF + CV_SLOPE * (orifice[3] - T_REF));
    orificeSolve(orifice);
    io[INTAKE_MDOT] = -orifice[6]!;
  }

  /**
   * Add one substep's intake flow of cylinder `b` to what the plenum is handed, weighted so the
   * sum over the substeps is the sample average.
   */
  private accumulateIntake(b: number): void {
    const io = this.intakeIo;
    const acc = this.intakeAcc;
    const s = b * CYL_STATE_SIZE;
    const w = io[INTAKE_WEIGHT]!;
    const inMdot = io[INTAKE_MDOT]!;
    acc[ACC_FLOW] += inMdot * w;
    if (inMdot < 0) {
      // Back-flow up the runner. Mass-weight its temperature and composition so several
      // cylinders spitting at once are averaged rather than the last one winning.
      const m = -inMdot * w;
      acc[ACC_BACK_MASS] += m;
      acc[ACC_BACK_ENERGY] += m * this.cylState[s + CYL_TEMP]!;
      acc[ACC_BACK_BURNED] += m * this.cylState[s + CYL_BURNED]!;
      acc[ACC_BACK_FUEL] += m * this.cylState[s + CYL_FUEL]!;
    }
  }

  /**
   * Cylinder `b`'s valve lifts and exhaust flow area this sample, into `liftNow[3b..3b+2]`.
   *
   * A method of its own, taking only the index, so that `valveLift` is inlined here. Written inline in
   * `tick` it would not be — `tick` is well past its inlining budget — and each call would box its
   * four floating-point arguments and its result, twice per cylinder per sample.
   */
  private computeLifts(b: number): void {
    const spec = this.spec;
    const angle = this.cyls[b]!.angle;
    // This cylinder's own cam timing, a degree or two from nominal.
    const camOffset = this.timing[b]!;
    const exLift = valveLift(angle, spec.evo + camOffset, spec.evc + camOffset, spec.maxLift);
    const inLift = valveLift(angle, spec.ivo + camOffset, spec.ivc + camOffset, spec.maxLift);
    this.liftNow[b * 3] = exLift;
    this.liftNow[b * 3 + 1] = inLift;
    this.liftNow[b * 3 + 2] = valveFlowArea(exLift, spec.exValveDia) * spec.exValveCount;
  }

  /**
   * Fixed per-cylinder breathing multipliers, spread about 1.
   *
   * Deliberately deterministic rather than random, so an engine sounds the same each time it is
   * started, and spread evenly across the range rather than drawn independently, so a four does
   * not occasionally come out with four nearly identical cylinders and no rumble at all. A few
   * percent is what real engines show; `cylinderSpread` scales it, and zero restores the perfectly
   * matched — and unnaturally pure — behaviour.
   */
  private makeCylinderVariation(n: number): void {
    // Clamped at 2, not 1: 1 is "realistic", but badly matched or worn engines are worse, and
    // clamping at 1 would quietly make the top half of the control do nothing.
    const spread = clamp(this.spec.cylinderSpread, 0, 2);
    this.breathing = new Float64Array(n);
    this.timing = new Float64Array(n);
    for (let b = 0; b < n; b++) {
      // Evenly spaced in [-1, 1], then shuffled by a fixed permutation so neighbouring cylinders
      // are not monotonically ordered — a real engine's weakest cylinder is not always number one.
      const t = spreadOf(b, n, 5, 2);
      // A second, differently shuffled sequence, so the cylinder that breathes best is not also
      // the one whose cam is advanced. Correlating them would make the two effects redundant.
      const u = spreadOf(b, n, 3, 1);
      this.breathing[b] = 1 + 0.04 * spread * t;
      this.timing[b] = CAM_SPREAD_DEG * spread * u;
    }
  }

  /**
   * Work out each mouth's path to the ear.
   *
   * The mouths are laid out in a line across the listener's view, centred on the nominal exhaust
   * position, and each gets the extra delay and spreading loss its own distance implies. Delays
   * are *relative* to the nearest mouth, because the common part of the path is already handled by
   * the `Listener`; only the differences between mouths matter here.
   */
  private refreshMouthPaths(): void {
    const count = Math.max(1, this.wg.mouthCount);
    const c = ambientSoundSpeed();
    const spacing = Math.max(this.spec.mouthSpacing, 0);
    const distance = Math.max(this.spec.micDistance, 0.15);

    // The listener stands off to one side rather than on the mouths' perpendicular bisector.
    //
    // This is the part that actually does the work, and it is easy to get wrong: place the
    // mouths symmetrically about the listener's axis and every one of them is the *same* distance
    // away, so the path differences are zero and the sum is exactly as coherent as if they were
    // coincident. Placed that way, sweeping the spacing from 0 to 2 m changes the output not at
    // all — bit for bit. Nobody listens to an engine from dead ahead of its tailpipes.
    const AZIMUTH = Math.PI / 4;
    const lx = distance * Math.cos(AZIMUTH);
    const ly = distance * Math.sin(AZIMUTH);

    const ranges = new Float64Array(count);
    let nearest = Infinity;
    for (let m = 0; m < count; m++) {
      // Mouths in a line, centred on the nominal exhaust position.
      const lateral = (m - (count - 1) / 2) * spacing;
      ranges[m] = Math.hypot(lx - lateral, ly);
      nearest = Math.min(nearest, ranges[m]!);
    }

    if (this.mouthDelays.length !== count) {
      // A 4 m path difference is longer than any plausible layout, and cheap at this length.
      this.mouthDelays = Array.from({ length: count }, () => new Delay(Math.ceil((4 / c) * this.sampleRate)));
      this.mouthGains = new Float64Array(count);
    }
    for (let m = 0; m < count; m++) {
      this.mouthDelays[m]!.setDelay(((ranges[m]! - nearest) / c) * this.sampleRate);
      // Spreading relative to the nominal distance the Listener already accounts for.
      this.mouthGains[m] = nearest / ranges[m]!;
    }
  }

  /** Everything that forces a full rebuild of cylinders and ducts when it changes. */
  private layoutKey(): string {
    return `${this.spec.cylinders}/${exhaustLayoutOf(this.spec)}/${this.spec.crankType}`;
  }

  /**
   * Build the cylinders, phased on one shared crank.
   *
   * Each `Cylinder` advances its own angle by the same `omega * dt`, so seeding them with the
   * firing offset keeps them in lockstep for ever. Each also gets its own noise seed, or both
   * banks would scatter identically and a twin would sound like one cylinder played twice.
   */
  private buildCylinders(): Cylinder[] {
    const plan = firingPlan(this.spec);
    const out: Cylinder[] = [];
    for (let b = 0; b < this.spec.cylinders; b++) {
      // Note the sign. A cylinder that fires *later* is *behind* on the crank: it has further
      // to go to reach its own firing TDC, so its current angle is lower by the offset, not
      // higher. Seeding `+offset` instead runs the whole firing order backwards, which for a
      // twin is invisible — the pair of intervals is the same either way — but reverses a V8's
      // bank pattern. Seeded this way, `plan.offsets` matches the crank angles at which the
      // cylinders are actually observed to fire.
      out.push(new Cylinder(this.spec, wrapCycle(-plan.offsets[b]!), 0x51f3a7 + b * 0x9e3779b));
      this.throatNoise[b] ??= new Noise(0x2c1b3d + b * 0x85ebca6b);
    }
    return out;
  }

  /**
   * The stored graph if it still describes this engine, otherwise one compiled from the layout.
   *
   * A drawn graph can outlive the engine it was drawn for. `setEngine` and `setGraph` are separate
   * messages, so switching a V-twin to a V8 rebuilds the exhaust *once* with the new cylinder count and
   * the old graph before the new graph arrives — and that graph has no pipe on cylinders 3 to 8.
   *
   * Treated as an error, that would be as bad as it gets. `ExhaustSystem` rightly refuses to build an
   * unsolvable graph, but the throw would land inside the worklet's message handler, leaving a two-duct
   * exhaust attached to an eight-cylinder engine; the next `process` call would read `primaries[2]`,
   * get `undefined`, and the AudioWorkletNode would die for good. Silence, permanently, from switching
   * engine.
   *
   * So a mismatch is treated as staleness rather than as an error. The audio thread has no business
   * throwing: there is nothing above it to catch anything, and the cost of being wrong is the whole
   * app going quiet. The stored graph is kept rather than discarded, because the renderer is about to
   * send a re-seeded one anyway and a graph that does not fit *this* spec may fit the next.
   */
  private usableGraph(): ExhaustGraph {
    const stored = this.graph;
    if (stored) {
      const problems = validateGraph(stored, this.spec.cylinders);
      if (problems.length === 0) return stored;
      console.warn(
        `[engineSim] the drawn exhaust does not fit this engine (${problems[0]}); ` +
          'using the layout until a new one arrives',
      );
    }
    return compileLayout(this.spec, this.pipe, this.collectorPipe);
  }

  private buildExhaust(): ExhaustSystem {
    const sys = new ExhaustSystem(
      this.usableGraph(),
      this.spec.cylinders,
      this.sampleRate,
      this.spec.portGasTemp,
      this.buildOptions(),
    );
    // The merge shares the throat-noise control: both are turbulent flow noise.
    sys.setTurbulence(this.spec.throatNoise);
    return sys;
  }

  /**
   * First quarter-wave resonance of the exhaust duct as it is currently filled, Hz.
   *
   * Integrated over the solved temperature field, so it is not merely `c/4L` with an assumed
   * sound speed — it moves with how hot the duct actually is, which is the point.
   */
  ductQuarterWaveHz(): number {
    return this.wg.quarterWaveHz();
  }

  /** Mean crank speed, rev/min. Excludes the within-cycle ripple so readouts are steady. */
  get rpm(): number {
    const w = this.integratingCrank() ? this.omegaDisplay : this.omegaMean;
    return (w * 60) / (2 * Math.PI);
  }

  /**
   * Whether the crank speed is integrated from the torque rather than held: free-running, or held
   * at a speed the rev limiter will not allow.
   */
  private integratingCrank(): boolean {
    return this.spec.freeRunning || this.spec.rpm >= this.spec.revLimit || this.dyno !== null;
  }

  /** Instantaneous crank speed, rev/min, ripple included. */
  get rpmInstant(): number {
    return (this.omega * 60) / (2 * Math.PI);
  }

  // -------------------------------------------------------------------------
  // Simulation
  // -------------------------------------------------------------------------

  /** Advance one audio sample. Returns the listener signal, nominally in [-1, 1]. */
  tick(): number {
    const spec = this.spec;
    const dt = 1 / this.sampleRate;

    // --- Crank speed --------------------------------------------------------
    const inertia = Math.max(spec.flywheelInertia, 1e-3);
    // Summed over banks, from the previous sample — the cylinders have not advanced yet.
    const torque = this.torqueLast;

    if (this.integratingCrank()) {
      // Gas torque fights the load plus friction, so the pipe's tuning can actually
      // pull the engine up or hold it back.
      //
      // Also how a held speed at or past the rev limiter runs: a crank pinned at the limit would
      // hold it exactly, and what a limiter sounds like is the bounce. So the hold lets go and the
      // engine revs freely into the cut and off it, with no load, as one revved in neutral does.
      //
      // Friction is expressed as an FMEP (friction mean effective pressure) rising
      // with speed and converted to torque the same way indicated work is, which
      // keeps it dimensionally honest across different engine sizes. 0.8 bar at
      // idle rising to ~1.5 bar at 6000 rpm matches published single-cylinder data;
      // a fixed N*m guess does not scale and lets the engine run away.
      const fmep = 0.8e5 + 120 * this.omegaMean;
      const friction = (fmep * this.displacementM3) / (4 * Math.PI);
      const dyno = this.dyno;
      const load = dyno
        ? dyno.step(dt, this.omegaMean, torque - friction, this.cyls[0]!.angle)
        : spec.freeRunning
          ? this.loadTorqueNm
          : 0;
      const net = torque - load - friction;
      this.omegaMean += (net / inertia) * dt;
      const minOmega = (MIN_RPM * 2 * Math.PI) / 60;
      const maxOmega = (12000 * 2 * Math.PI) / 60;
      this.omegaMean = clamp(this.omegaMean, minOmega, maxOmega);
      this.omega = this.omegaMean;
      // The integrated speed already carries the within-cycle ripple, so the readout needs
      // its own smoothing here — otherwise `rpm` reports the instantaneous value in
      // free-running mode and the smoothed one in fixed-rpm mode, and the display jitters.
      this.omegaDisplay += (this.omegaMean - this.omegaDisplay) * (dt / IRREGULARITY_TAU);
      // Hard spark cut. Judged on the unsmoothed speed, as an ECU timing crank teeth does: the
      // readout's smoothing lags by a tenth of a second, which would let the engine sail well past
      // the limit before it noticed.
      const limitOmega = (spec.revLimit * 2 * Math.PI) / 60;
      if (this.omegaMean >= limitOmega) {
        this.limiterCut = true;
      } else if (this.omegaMean < ((spec.revLimit - REV_LIMIT_HYSTERESIS_RPM) * 2 * Math.PI) / 60) {
        this.limiterCut = false;
      }
    } else {
      // Holding a *mean* speed, not a rigid one.
      //
      // A real crank does not turn at a constant rate even on a dynamometer: gas and
      // reciprocating-inertia torque accelerate and decelerate it within every cycle,
      // and on a big single that swing is several percent. Locking the speed exactly
      // removes the frequency modulation that makes a single sound like a single, so
      // the fluctuation is integrated here and superimposed on the commanded mean.
      //
      // Only the *fluctuating* part of the torque may accelerate the crank: whatever
      // holds the speed — a dynamometer, a gearbox, the road — absorbs the mean. So the
      // mean is tracked and subtracted before integrating. Integrating the raw torque
      // instead leaves a DC term that drags the engine off the commanded speed by over
      // a hundred rpm, and by a load-dependent amount at that.
      this.omegaMean = (spec.rpm * 2 * Math.PI) / 60;
      // Kept current, so the readout does not start from a stale speed when the hold lets go.
      this.omegaDisplay = this.omegaMean;
      this.limiterCut = false;
      this.torqueAvg += (torque - this.torqueAvg) * (dt / IRREGULARITY_TAU);
      const leak = Math.exp(-dt / (IRREGULARITY_TAU * 4));
      this.omegaRipple = this.omegaRipple * leak + ((torque - this.torqueAvg) / inertia) * dt;
      // Keep it plausible even if someone dials in a toy flywheel.
      this.omegaRipple = clamp(this.omegaRipple, -0.35 * this.omegaMean, 0.35 * this.omegaMean);
      this.omega = Math.max(this.omegaMean + this.omegaRipple, 1);
    }

    // --- Dyno run -------------------------------------------------------------
    // The run drives the throttle; when it is over, and the engine has wound back down to where a
    // held speed would put it, the throttle and the crank go back to the spec.
    let throttle = spec.throttle;
    const dyno = this.dyno;
    if (dyno) {
      throttle = dyno.throttle;
      if (throttle !== this.dynoOpening) {
        this.plenum.setOpening(spec, throttle);
        this.dynoOpening = throttle;
      }
      if (
        dyno.phase === 'cooldown' &&
        (spec.freeRunning || this.omegaMean <= (spec.rpm * 2 * Math.PI) / 60 || dyno.phaseTime > DYNO_WIND_DOWN)
      ) {
        this.dyno = null;
        this.plenum.setGeometry(spec);
        this.dynoOpening = NaN;
        throttle = spec.throttle;
      }
    }

    // --- Overrun fuel cut ----------------------------------------------------
    // Judged on the mean speed, which in fixed-rpm mode is the commanded one: held on a dynamometer
    // with the throttle shut, the engine is being motored, and the ECU cuts the fuel just the same.
    if (!spec.fuelCut || throttle > FUEL_CUT_THROTTLE) {
      this.fuelCutActive = false;
    } else {
      const rpmNow = (this.omegaMean * 60) / (2 * Math.PI);
      if (rpmNow > FUEL_CUT_RPM) this.fuelCutActive = true;
      else if (rpmNow < FUEL_RESUME_RPM) this.fuelCutActive = false;
    }

    // --- Valves and flows, per bank -----------------------------------------
    const banks = this.cyls.length;
    const valves = this.valveStates;
    let torqueSum = 0;
    let dpdtSum = 0;

    const limiterCut = this.limiterCut;
    for (let b = 0; b < banks; b++) {
      const cyl = this.cyls[b]!;
      const angle = cyl.angle;
      cyl.sparkCut = limiterCut;
      cyl.camOffset = this.timing[b]!;
      // This cylinder's own cam timing, a degree or two from nominal.
      this.computeLifts(b);
      const exLift = this.liftNow[b * 3]!;
      const inLift = this.liftNow[b * 3 + 1]!;
      const exArea = this.liftNow[b * 3 + 2]!;
      // Read into arrays rather than through getters: `tick` is too big for them to inline, and a
      // floating-point return from a call that is not inlined is boxed into a fresh heap object.
      const state = this.cylState;
      const s = b * CYL_STATE_SIZE;
      cyl.readState(spec, state, s);
      const pCyl = state[s + CYL_PRESSURE]!;
      const tCyl = state[s + CYL_TEMP]!;
      this.wg.primaries[b]!.readPort(this.portState);
      const portAbs = this.portState[0]!;

      // Throat turbulence, scaled by the previous sample's flow through *this* valve.
      let extraMassFlow = 0;
      if (exArea > 0 && spec.throatNoise > 0) {
        const throatRho = portAbs / (GAS.R * tCyl);
        const speed = Math.min(
          Math.abs(this.lastValveMdot[b]!) / Math.max(throatRho * exArea, 1e-9),
          // `speedOfSound(tCyl, GAS.gammaCyl)`, written out: a call here would box both arguments.
          Math.sqrt(GAS.gammaCyl * GAS.R * tCyl),
        );
        const strouhalHz = (0.2 * speed) / spec.exValveDia;
        // One-pole coefficient from the corner frequency. `1 - exp(-w/fs)` is the pole
        // mapping; the linear `w/fs` agrees with it only for w << fs, and at a Strouhal
        // frequency of 3.5 kHz would give 0.46 against a correct 0.37 — a filter materially
        // wider than the physics asks for, leaking jet noise into the top two octaves.
        const k = clamp(1 - Math.exp((-2 * Math.PI * strouhalHz) / this.sampleRate), 0.02, 0.85);
        const white =
          this.throatNoise[b]!.next() *
          Math.abs(this.lastValveMdot[b]!) *
          TURBULENCE_INTENSITY *
          spec.throatNoise;
        this.turb1[b] += k * (white - this.turb1[b]!);
        this.turb2[b] += k * (this.turb1[b]! - this.turb2[b]!);
        extraMassFlow += this.turb2[b]!;
      }

      const seating = this.prevExLift[b]! > 0 && exLift === 0;
      if (seating) {
        this.seatPulse[b]!.trigger(spec.mechNoise * 0.02 * (this.rpm / 3000));
      }
      extraMassFlow += this.seatPulse[b]!.next();

      valves[b]!.throatArea = exArea;
      valves[b]!.cylPressure = pCyl;
      valves[b]!.cylTemp = tCyl;
      // `gasGamma`, written out: a call here would box its argument and result.
      valves[b]!.cylGamma = 1 + GAS.R / (CV_REF + CV_SLOPE * (tCyl - T_REF));
      valves[b]!.extraMassFlow = extraMassFlow;

      this.exLift[b] = exLift;
      this.inLift[b] = inLift;
      this.seatingNow[b] = seating;
      this.inSeatingNow[b] = this.prevInLift[b]! > 0 && inLift === 0;
      this.tdcPressure[b] =
        crossedAngle(this.prevAngle[b]!, angle, 0) || crossedAngle(this.prevAngle[b]!, angle, 360)
          ? pCyl
          : -1;
      /**
       * Remembered here, at the angle the check just used, like the lifts are.
       *
       * Saved after the cylinder advances — which is where the *next* sample's `angle` starts — every
       * check would compare an angle with itself, never see a crossing, and piston slap would never
       * sound at all.
       */
      this.prevAngle[b] = angle;
    }

    // --- Exhaust gas dynamics, all ducts in lockstep ------------------------
    const pipeResult = this.wg.advance(dt, valves);
    this.substeps = pipeResult.substeps;

    // --- Cylinder gas state, sub-stepped ------------------------------------
    // Manifold pressure is solved by the plenum rather than mapped from the throttle.
    const pPlenum = this.plenum.pressure;
    const tPlenum = this.plenum.temp;
    const plenumBurned = this.plenum.burnedFraction;
    const plenumFuel = this.plenum.fuelFraction;
    // Accumulated over the cylinders, then handed to the plenum once below.
    const intake = this.intakeAcc;
    intake.fill(0);
    const intakeIo = this.intakeIo;
    intakeIo[INTAKE_P_PLENUM] = pPlenum;
    intakeIo[INTAKE_T_PLENUM] = tPlenum;
    // `gasGamma`, written out: a call here would box its argument and result.
    intakeIo[INTAKE_GAMMA_PLENUM] = 1 + GAS.R / (CV_REF + CV_SLOPE * (tPlenum - T_REF));

    for (let b = 0; b < banks; b++) {
      const cyl = this.cyls[b]!;
      const exMdot = pipeResult.valveMassFlows[b]!;
      this.lastValveMdot[b] = exMdot;
      const inArea = valveFlowArea(this.inLift[b]!, spec.inValveDia) * spec.inValveCount;
      intakeIo[INTAKE_AREA] = inArea;
      // Each cylinder sees a slightly different runner pressure — see `breathing`.
      intakeIo[INTAKE_BREATHING] = this.breathing[b]!;
      // A first estimate from the state the first loop read, only to choose the substep count;
      // the flow actually integrated is recomputed from the cylinder's own state every substep.
      this.intakeFlowNow(b);
      const inMdot = intakeIo[INTAKE_MDOT]!;
      this.wg.primaries[b]!.readPort(this.portState);
      const portTemp = this.portState[1]!;

      const degPerSample = (Math.abs(this.omega) * dt * 180) / Math.PI;
      const massFlux = (Math.abs(exMdot) + Math.abs(inMdot)) * dt;
      const massRatio = massFlux / Math.max(cyl.mass, 1e-12);
      const nSub = clamp(
        Math.max(
          Math.ceil(degPerSample / MAX_DEG_PER_SUBSTEP),
          Math.ceil(massRatio / MAX_MASS_FRACTION_PER_SUBSTEP),
        ),
        1,
        MAX_CYL_SUBSTEPS,
      );
      const subDt = dt / nSub;
      // Through a typed array, not arguments: see `Cylinder.advanceIo`.
      const io = this.cylIo;
      io[IO_DT] = subDt;
      io[IO_OMEGA] = this.omega;
      io[IO_EX] = exMdot;
      io[IO_INTAKE_T] = tPlenum;
      io[IO_PORT_T] = portTemp;
      io[IO_INTAKE_BURNED] = plenumBurned;
      io[IO_INTAKE_FUEL] = plenumFuel;
      intakeIo[INTAKE_WEIGHT] = 1 / nSub;
      if (inArea > 0) {
        for (let k = 0; k < nSub; k++) {
          // The intake valve sees the cylinder pressure as it evolves through the sample. Held at
          // the start-of-sample value it keeps flowing after the pressures have crossed.
          if (k > 0) {
            const end = cyl.endState;
            const st = this.cylState;
            const s = b * CYL_STATE_SIZE;
            st[s + CYL_PRESSURE] = end[CYL_PRESSURE]!;
            st[s + CYL_TEMP] = end[CYL_TEMP]!;
            st[s + CYL_BURNED] = end[CYL_BURNED]!;
            this.intakeFlowNow(b);
          }
          io[IO_IN] = intakeIo[INTAKE_MDOT]!;
          this.accumulateIntake(b);
          cyl.advanceIo(spec, io);
        }
      } else {
        io[IO_IN] = 0;
        for (let k = 0; k < nSub; k++) cyl.advanceIo(spec, io);
      }
      torqueSum += cyl.torque + cyl.inertiaTorque;
      dpdtSum += cyl.dpdt;
      this.prevExLift[b] = this.exLift[b]!;
      this.prevInLift[b] = this.inLift[b]!;
    }

    // The plenum sees the sum of the valve flows, and takes back whatever was spat at it.
    const backflowMass = intake[ACC_BACK_MASS]!;
    this.plenum.step(
      dt,
      intake[ACC_FLOW]!,
      backflowMass,
      backflowMass > 0 ? intake[ACC_BACK_ENERGY]! / backflowMass : tPlenum,
      backflowMass > 0 ? intake[ACC_BACK_BURNED]! / backflowMass : 0,
      backflowMass > 0 ? intake[ACC_BACK_FUEL]! / backflowMass : 0,
      this.fuelCutActive ? 0 : fuelFractionAt(spec.lambda),
    );

    // --- Structure-borne noise ----------------------------------------------
    // The block is one structure for the whole engine: the banks share a crankcase, so combustion in
    // any cylinder shakes the same casing. The valve and piston rings are local, one per cylinder.
    let directPa = 0;
    const mech = spec.mechNoise;
    const headShare = this.headShare;

    for (let b = 0; b < banks; b++) {
      // Valve seating. Both valves land, and the seating velocity follows cam speed, so the clacks
      // get sharper with rpm. Detected on lift reaching exactly zero.
      const seat = (this.seatingNow[b] ? 1 : 0) + (this.inSeatingNow[b] ? 0.7 : 0);
      if (seat > 0) {
        this.clackImpact[b]!.trigger(CLACK_PA_AT_1M * mech * seat * (this.rpm / 3000) * headShare);
      }
      directPa += this.clack[b]!.process(this.clackImpact[b]!.next());

      // Piston slap: the rod side-load reverses across TDC and the piston crosses its clearance to
      // the other side of the bore. The bang scales with how hard the gas in *this* cylinder is
      // pressing at that instant — loud at firing TDC, slight at the exhaust stroke's, nearly absent
      // on overrun. Taking the highest pressure in any cylinder instead would make every
      // exhaust-stroke TDC on a V8 slap as hard as a firing one.
      const p = this.tdcPressure[b]!;
      // The block wall it slaps is shared by its bank the way the head is, so it scales the same way.
      if (p >= 0) {
        this.slapCount++;
        this.slapImpact[b]!.trigger(SLAP_PA_AT_1M * mech * clamp(p / 3e6, 0.05, 1.6) * headShare);
      }
      directPa += this.slap[b]!.process(this.slapImpact[b]!.next());
    }

    // Combustion shaking the casing, driven by the summed pressure rise rate.
    //
    // `dpdt` is a finite difference across one cylinder sub-step, so it carries the
    // integrator's own jitter multiplied by 1/dt — 48,000 or more. Used raw it is part physical
    // pressure rise and part amplified numerical noise, and the noise is white, so it would drive
    // all four resonators continuously and pile up in the top octaves as a buzz.
    //
    // Smoothing it first is not a tone control, it is the correct bandwidth. The pressure
    // rise that shakes a casing spans ten to thirty crank degrees — about a millisecond at
    // 3200 rpm — so there is nothing real in this signal above a few kHz to begin with.
    if (mech > 0) {
      this.dpdtSmooth += this.dpdtSmoothC * (dpdtSum - this.dpdtSmooth);
      const drive = (this.dpdtSmooth / 1e9) * STRUCTURE_PA_PER_GPA_S * mech;
      for (const mode of this.structure) directPa += mode.process(drive) * 0.25;
    }

    // Band limit for everything structure-borne.
    //
    // The exhaust path has one of these — `resolutionCutoffRad`, on the grounds that
    // radiating above where the solver is faithful is radiating numerical debris — and the
    // structure-borne path needs the same argument made of it: the highest mode modelled here
    // is 4.7 kHz, the excitations are short impacts (finite `Impact` pulses, which take off
    // much of an impulse's flat-to-Nyquist spectrum on their own but not all of it), and a
    // two-pole resonator only sheds 12 dB/octave above its peak. So everything above the
    // modal range is either an artefact of impulsing a filter or a mode this model never
    // claims to have, and left in it is audible as a buzz sitting above the engine.
    this.structureLp1 += this.structureLpC * (directPa - this.structureLp1);
    this.structureLp2 += this.structureLpC * (this.structureLp1 - this.structureLp2);
    directPa = this.structureLp2;

    this.torqueLast = torqueSum;

    // --- Radiate -------------------------------------------------------------
    // Exhaust and structure both radiate from roughly the same place, so they take the same route
    // to the ear: spreading, the ground bounce and air absorption.
    //
    // The mouths are summed *with their path differences*, which matters far more than it looks.
    // Treating them as coincident makes the sum exactly coherent, and two banks of a flatplane V8
    // fire in antiphase — so their strongest component, each bank's own firing order, would cancel
    // to 43 dB below where it belongs and the engine would jump an octave. A real pair of tailpipes
    // is a metre or so apart, which at those frequencies is most of a wavelength.
    //
    // The delay, the spreading loss and the far-field transfer are per mouth, the last because its
    // corner and band limit follow the mouth's own size. The ground bounce is shared, a good
    // approximation since all the mouths sit at the same height.
    const flows = pipeResult.mouthFlows;
    const farFields = this.farFields;
    let exhaustPa = 0;
    if (flows.length === 1) {
      exhaustPa = farFields[0]!.process(flows[0]!);
    } else {
      for (let m = 0; m < flows.length; m++) {
        exhaustPa += farFields[m]!.process(this.mouthDelays[m]!.process(flows[m]!)) * this.mouthGains[m]!;
      }
    }
    let pa = this.listener.process(exhaustPa + directPa);

    if (this.rebuildRamp < 1) {
      this.rebuildRamp = Math.min(1, this.rebuildRamp + this.rebuildRampStep);
      pa *= this.rebuildRamp;
    }

    let out = (pa / PA_PER_FULLSCALE) * spec.outputGain;
    if (!Number.isFinite(out)) out = 0;
    out = softClip(out);

    const mag = Math.abs(out);
    if (mag > this.peak) this.peak = mag;
    return out;
  }

  /** Fill a snapshot for the renderer. Resets the peak meter. */
  snapshot(): EngineSnapshot {
    this.wg.samplePressure(this.tapBuffer);
    const spec = this.spec;
    const banks: BankSnapshot[] = this.cyls.map((cyl) => ({
      crankAngle: wrapCycle(cyl.angle),
      cylPressure: cyl.pressure(spec),
      cylTemp: cyl.temp,
      exLift: valveLift(cyl.angle, spec.evo, spec.evc, spec.maxLift),
      inLift: valveLift(cyl.angle, spec.ivo, spec.ivc, spec.maxLift),
    }));
    const first = banks[0]!;

    const snap: EngineSnapshot = {
      banks,
      crankAngle: first.crankAngle,
      rpm: this.rpm,
      limiter: this.limiterCut,
      fuelCut: this.fuelCutActive,
      dyno: this.dyno && {
        phase: this.dyno.phase,
        gear: this.dyno.gear + 1,
        speedKmh: this.dyno.speed * 3.6,
        elapsed: this.dyno.elapsed,
        finished: this.dyno.finished,
        points: this.dyno.takePoints(),
      },
      cylPressure: first.cylPressure,
      cylTemp: first.cylTemp,
      exLift: first.exLift,
      inLift: first.inLift,
      torque: this.torqueLast,
      pipePressure: this.tapBuffer.slice(),
      peak: this.peak,
      pipeCells: this.wg.cells,
      substeps: this.substeps,
      wallTemp: this.wg.meanWallTemp(),
    };
    this.peak = 0;
    return snap;
  }

  /** Render `n` samples into a new array. Test and offline-analysis helper. */
  render(n: number): Float32Array {
    const out = new Float32Array(n);
    for (let i = 0; i < n; i++) out[i] = this.tick();
    return out;
  }
}

/** True if the crank swept past `target` degrees between two samples. */
function crossedAngle(from: number, to: number, target: number): boolean {
  if (to >= from) return target > from && target <= to;
  return target > from || target <= to;
}
