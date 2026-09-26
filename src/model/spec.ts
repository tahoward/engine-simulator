/**
 * Shared data model. Imported by the main thread (three.js scene + UI) and by the
 * AudioWorklet, so it must stay free of any DOM or AudioContext references.
 *
 * Units are SI throughout: metres, kilograms, seconds, kelvin, pascals. The UI
 * converts to millimetres at the edges only — mixing units inside the physics is
 * the classic way to lose an afternoon.
 *
 * Crank-angle convention: 0 deg = TDC at the start of the power stroke, increasing
 * with rotation, cycle = 720 deg.
 *
 *     0-180    power / expansion   (piston down)
 *   180-360    exhaust            (piston up)
 *   360-540    intake             (piston down)
 *   540-720    compression        (piston up)
 */

import type { ExhaustGraph } from './exhaustGraph.js';

/** Shape of one length of exhaust plumbing. */
export type SegmentKind =
  /** Constant-diameter tube. `dOut` is ignored and tracks `dIn`. */
  | 'pipe'
  /** Linear taper from `dIn` to `dOut` — headers, megaphones, reverse cones. */
  | 'cone'
  /** Sudden expansion into a large-diameter volume, then back down: a muffler can. */
  | 'chamber';

/**
 * Cross-section of a chamber's body. The throats at either end are always round.
 *
 * `rect` has rounded corners, of radius `RECT_CORNER` times the shorter side, as a pressed or
 * rolled can does.
 */
export type ChamberSection = 'round' | 'oval' | 'rect';

export const CHAMBER_SECTIONS: ChamberSection[] = ['round', 'oval', 'rect'];

export interface PipeSegment {
  id: string;
  kind: SegmentKind;
  /** Axial length, metres. */
  length: number;
  /** Inlet diameter, metres. */
  dIn: number;
  /**
   * Outlet diameter, metres. Forced equal to `dIn` when `kind === 'pipe'`. For a chamber, the
   * body's diameter if it is round and its width otherwise.
   */
  dOut: number;
  /**
   * Routing only — never affects the 1D acoustics, which care about area vs
   * axial distance. Yaw/pitch are a sharp corner at the segment's start, after
   * which it runs straight, so the user can fold a long pipe into the viewport.
   */
  yaw: number;
  pitch: number;
  /** Chamber only: body cross-section. Round when absent. */
  section?: ChamberSection;
  /** Chamber only: body height, metres, for an oval or rect body. Width is `dOut`. */
  height?: number;
  /**
   * Chamber only: how far the inlet and outlet pipes sit off the body's centreline, metres,
   * along its width. Offset pipes excite the cross-wise modes a centred pipe cannot reach.
   */
  offsetIn?: number;
  offsetOut?: number;
}

/**
 * Cylinder counts with a firing plan defined. Six is an inline six at a zero V angle and a V6 otherwise,
 * the way two is a parallel twin or a V-twin.
 */
export type CylinderCount = 1 | 2 | 3 | 4 | 5 | 6 | 8;

/** Valves on one side of a cylinder: one for a two-valve head, two for a four-valve head. */
export type ValveCount = 1 | 2;

/**
 * Crank arrangement, where an engine has a choice of one.
 *
 * `crossplane` is the American V8 crank, with its pins at 90-degree intervals; `flatplane` is
 * the Ferrari/Voodoo crank, with all four pins in one plane. Both fire every 90 degrees of
 * crank, so on a single collector they sound much the same. The difference is in which bank
 * each firing belongs to — see `firingPlan`.
 *
 * `boxer` is a flat four or flat six: the banks 180 degrees apart, every cylinder on a throw of its
 * own, and each opposed pair's throws half a turn apart so the two pistons move out and in together.
 * See `boxerPlan`.
 *
 * `shared` is the default, and means the engine's only crank: the others are ignored where they do not
 * apply.
 */
export type CrankType = 'shared' | 'flatplane' | 'crossplane' | 'boxer';

export type ExhaustLayout = 'open' | 'perBank' | 'merged';

/**
 * Accepted layout values, including the twin-only names older configs use.
 *
 * `single` and `2into2` both mean one pipe per cylinder; `2into1` means one shared collector.
 * Kept so saved links and older configs still load — `exhaustLayoutOf` normalises them.
 */
export type ExhaustLayoutSpec = ExhaustLayout | 'single' | '2into2' | '2into1';

export interface EngineSpec {
  // --- Layout ---
  /**
   * Number of cylinders: 1, 2, 3, 4, 5, 6 or 8.
   *
   * Restricted to those because the firing plan is real engine data rather than something
   * derived — see `firingPlan`. Any other count would need its own entry, not a formula.
   */
  cylinders: CylinderCount;
  /**
   * Included angle between the cylinders, degrees. 0 is a parallel twin.
   *
   * 45 is the classic Harley, 90 the Ducati L-twin. With a shared crankpin the angle also
   * sets the firing interval, so this is the only control most twins need.
   */
  vAngle: number;
  /**
   * Crank degrees from cylinder 1 firing to cylinder 2 firing, or `null` to derive it.
   *
   * Derived it is `360 + vAngle`, which is what a shared crankpin gives: 405 and 315 for a
   * 45-degree Harley, 450 and 270 for a 90-degree Ducati. Overriding it covers layouts a
   * shared pin cannot make — 360 for a parallel twin with both pistons together, 270 for a
   * modern crossplane twin, or something small for a big-bang layout.
   */
  firingOffset: number | null;
  /**
   * Crank arrangement, for engines with more than two cylinders.
   *
   * On a V8 it is the whole difference between an American V8 and a Ferrari. Both fire every 90
   * degrees; what differs is *which bank* each of those firings belongs to, so it only becomes
   * audible once each bank has its own collector. On a four or a six, `boxer` makes it a flat engine
   * (with `vAngle` 180). Ignored for every other cylinder count, whose plans are fixed.
   */
  crankType: CrankType;
  /**
   * How the exhausts are plumbed.
   *
   * - `open` — one pipe per cylinder, straight out. Two cylinders cannot hear each other.
   * - `perBank` — one collector per bank: a 4-into-1 each side of a V8.
   * - `merged` — every cylinder into one collector. This is `2into1` for a twin.
   *
   * A collector is where cylinders start to interact: each pulse partly travels up the *other*
   * primaries and either helps scavenge those cylinders or blocks them, depending on where the
   * firing interval puts it.
   */
  exhaustLayout: ExhaustLayoutSpec;

  // --- Geometry ---
  /** Cylinder bore, m. */
  bore: number;
  /** Piston stroke, m. Crank radius is half this. */
  stroke: number;
  /** Connecting rod length (centre to centre), m. */
  rodLength: number;
  /** Geometric compression ratio, dimensionless. Sets clearance volume. */
  compressionRatio: number;

  // --- Valves ---
  /** Exhaust valve head diameter, m: each valve's, where there are two. */
  exValveDia: number;
  /**
   * Exhaust valves per cylinder, 1 or 2. A four-valve head has two of each, and two valves open
   * more curtain than one of the same total area: at the same lift, √2 as much. That is why four
   * valves are what lets an engine breathe at high rpm.
   */
  exValveCount: ValveCount;
  /**
   * Length of the exhaust port, from the valve seat to the header flange, m.
   *
   * Modelled as a real length of duct prepended to the user's exhaust rather than as
   * a lumped volume: at 55 mm its own quarter-wave resonance is near 3 kHz, so it is
   * nowhere near acoustically compact in the range that matters and a lumped
   * compliance would wrongly lowpass the source there. As duct it also means the
   * tuned length is measured from the valve, which is where a real one is measured.
   */
  portLength: number;
  /** Intake valve head diameter, m: each valve's, where there are two. */
  inValveDia: number;
  /** Intake valves per cylinder, 1 or 2. See `exValveCount`. */
  inValveCount: ValveCount;
  /** Peak valve lift, m. Applies to both valves. */
  maxLift: number;
  /** Exhaust valve opens, deg ATDC. Typically ~130 (i.e. 50 deg BBDC). */
  evo: number;
  /** Exhaust valve closes, deg ATDC. Typically ~375 (just after TDC overlap). */
  evc: number;
  /** Intake valve opens, deg ATDC. Typically ~345. */
  ivo: number;
  /** Intake valve closes, deg ATDC. Typically ~570. */
  ivc: number;

  // --- Combustion ---
  /**
   * Spark timing, deg ATDC. Negative / >540 means before TDC firing.
   *
   * With `advanceCurve` on, this is the timing for a charge that burns over `burnDuration`; the
   * spark moves from it to keep each cycle's combustion phased the same.
   */
  ignition: number;
  /**
   * Whether the spark follows an advance map. On, it moves with the predicted burn of each charge:
   * later at low rpm, where the burn is quick, earlier at part throttle and high rpm, where it is
   * slow. Off, it fires at `ignition` whatever the charge, as a fixed-timing magneto does.
   */
  advanceCurve: boolean;
  /**
   * Wiebe burn duration at the reference flame state, deg: a stoichiometric charge at 13 bar and
   * 650 K with 4% residual, at 10 m/s mean piston speed. That is roughly any naturally aspirated
   * engine at full throttle.
   *
   * The duration each cycle actually burns over is worked out from this for its own charge, from the
   * flame speed it will have at the spark: longer at part throttle and with residual gas, longer lean, and
   * somewhat longer the faster the engine turns. See `burnAngle` in cylinder.ts.
   */
  burnDuration: number;
  /**
   * Air-fuel equivalence ratio λ: the air-fuel ratio as a multiple of stoichiometric. 1 is
   * stoichiometric, below 1 rich, above 1 lean.
   *
   * The fuel is metered in proportion to the air past the throttle, so this is the mixture the
   * cylinders draw once the manifold has settled. Lean, every kilogram of charge carries less
   * fuel and burns slower; rich, the extra fuel has no oxygen to burn with and goes out unburned.
   */
  lambda: number;
  /**
   * Overrun fuel cut, as a fuel-injected engine's ECU does it: with the throttle shut above
   * `FUEL_CUT_RPM` the fuel stops, and it comes back below `FUEL_RESUME_RPM` or as soon as the
   * throttle opens. The engine is then turned over by its load, pumping air.
   *
   * Off, a closed throttle keeps feeding fuel with the air that leaks past it, as a carburettor
   * does, and the engine keeps firing weakly on the overrun.
   */
  fuelCut: boolean;
  /**
   * Cycle-to-cycle combustion scatter, 0..1 (1 = realistic amount).
   *
   * Flame kernel growth depends on whatever turbulence happens to be at the spark gap
   * when it fires, so no two cycles burn identically. Real engines show 1-3% CoV of
   * peak pressure at full load and 5-15% near idle, where residual gas dilutes the
   * charge and destabilises the flame. Without this every cycle is bit-identical and
   * the result sounds like a looped wavetable rather than an engine.
   */
  combustionVariability: number;
  /**
   * Reciprocating mass: piston, rings, pin and the small end of the rod, kg.
   *
   * Its inertia torque is zero-mean over a cycle, so it does not change how fast the
   * engine runs, but it is comparable in magnitude to the gas torque and oscillates at
   * twice crank frequency — so it dominates how *unevenly* the crank turns, which is
   * most of the character of a big single.
   */
  recipMass: number;
  /**
   * 0..1 butterfly opening. Sets the throttle's flow area; the manifold pressure that
   * results is solved, not mapped.
   *
   * Expect it to feel nonlinear, because a real throttle is: area goes as `1 - cos(angle)`,
   * so most of the flow change happens in the first third of the travel, and past about
   * half open a throttle this size is barely a restriction at moderate rpm.
   */
  throttle: number;
  /**
   * Throttle bore, m. With `throttle` this fixes the flow area into the plenum.
   *
   * Sized for the engine's airflow at peak rpm, which is why a 500 cc single wants about
   * 40 mm and why that same 40 mm is nearly wide open, in flow terms, at a third of travel.
   */
  throttleDia: number;
  /**
   * Intake plenum volume downstream of the throttle, m^3 — manifold plus runners.
   *
   * This is what makes the intake a *finite* reservoir, and that is the whole point of it.
   * With an infinite fixed-pressure plenum, charge that back-flows up the intake during
   * valve overlap simply escapes, so the residual fraction never climbs when the engine is
   * throttled — and residual dilution is the main reason a real engine's exhaust
   * temperature collapses at light load. Give the plenum a volume and the back-flow is
   * retained, re-inducted next stroke, and the dilution appears on its own.
   *
   * Typically one to two times displacement for a single.
   */
  plenumVolume: number;

  // --- Operating point ---
  /**
   * Crank speed the engine starts at, rev/min. With `freeRunning` off, the speed it is held at as well.
   */
  rpm: number;
  /**
   * Rev limiter, rev/min.
   *
   * A hard spark cut: past the limit every cylinder whose charge is committed misses its firing,
   * and sparks return once speed has fallen `REV_LIMIT_HYSTERESIS_RPM` below it. The unburned
   * charge still goes down the pipe, and the engine bounces off the limit in the stuttering way a
   * real one does. With the speed held (`freeRunning` off) at or past the limit, the crank is let
   * go instead, unloaded, so it can bounce too.
   */
  revLimit: number;
  /**
   * When true the crank is integrated from gas torque, reciprocating inertia and load, so the
   * throttle and the pipe's tuning set the speed. See `flywheelInertia` / `load`. The app always runs
   * this way.
   *
   * When false the engine is held at `rpm`, as an engine dynamometer holds it, with only the
   * within-cycle ripple left free. That is a measurement setting: it is how the tests and the
   * benchmark put an engine at an exact operating point, and why it is the default here.
   */
  freeRunning: boolean;
  /** Rotating inertia, kg*m^2. Small single-cylinders are ~0.02-0.2. */
  flywheelInertia: number;
  /**
   * Braking torque at the crank, as a fraction of `fullLoadTorque` — 0 unloaded, 1 about what the
   * engine makes at full throttle.
   *
   * A fraction rather than N*m because a fixed torque means a different thing on every engine: 60 N*m
   * holds a 500 cc single down hard and is nothing to a 5.5 litre V8, which with a light flywheel would
   * run to its limiter in a few hundredths of a second. See `loadTorqueOf`.
   */
  load: number;

  // --- Acoustics / output ---
  /** Port gas temperature, K. Sets the speed of sound at the head of the pipe. */
  portGasTemp: number;
  /**
   * Cell length for the exhaust gas-dynamics solver, m.
   *
   * The trade between fidelity and CPU. Smaller cells resolve higher frequencies — roughly
   * `c / (10 * cellSize)` before numerical dissipation takes over — and cost more cells. The solver
   * takes exactly one step per audio sample, which puts a floor under this: a cell has to be long
   * enough for the fastest wave not to cross it in one sample, about 34 mm at 48 kHz and 37 mm at
   * 44.1 kHz, and anything smaller asked for is raised to that.
   */
  pipeCellSize: number;
  /**
   * Exhaust pipe wall thickness, m.
   *
   * Sets the wall's thermal mass, and so how long the system takes to come up to
   * temperature — tens of seconds for typical 1.2 mm tubing. Because the gas temperature
   * sets the speed of sound, the note genuinely shifts as the pipe warms.
   */
  pipeWallThickness: number;
  /**
   * Air speed past the exhaust, m/s. 0 is a stationary engine, 25 is roughly 90 km/h.
   *
   * Cools the pipe wall, which cools the gas, which slows the wave speed and drops the
   * tuning. Radiation off oxidised steel matters as much as convection here.
   */
  airSpeed: number;
  /** Horizontal distance from the exhaust mouth to the listener, m. */
  micDistance: number;
  /** Listener ear height above the ground, m. */
  micHeight: number;
  /** Height of the exhaust mouth above the ground, m. */
  exhaustHeight: number;
  /**
   * Lateral spacing between adjacent exhaust mouths, m.
   *
   * Only matters with more than one tailpipe, and then it matters a great deal. Mouths at the
   * *same* point sum coherently, and the two banks of a flatplane V8 fire in exact antiphase, so
   * their strongest component — each bank's own firing order — annihilates completely. Measured
   * with the mouths coincident, that component comes out 43 dB below where it belongs and the
   * engine jumps an octave to the doubled order. Real tailpipes sit a metre or more apart, which
   * at 187 Hz is most of a wavelength, so they cannot cancel like that.
   *
   * Typical: 0.15 m for open headers side by side, 0.5 m for twin tailpipes on a bike, 1.2-1.6 m
   * for a V8 exiting each side of a car.
   */
  mouthSpacing: number;
  /**
   * Cylinder-to-cylinder breathing spread, 0..1 (1 = realistic).
   *
   * No two cylinders of a real engine breathe identically: runner lengths differ, valve seats and
   * guides wear differently, fuelling is never perfectly matched. A few percent variation in
   * trapped mass between cylinders is normal, and it is what stops the firing orders cancelling
   * perfectly.
   *
   * It matters more the more cylinders there are. With every cylinder identical, an evenly firing
   * engine cancels every order that is not a multiple of the cylinder count *exactly* — measured
   * 66 dB down, where real engines sit 20-35 dB down. The result is unnaturally pure: a four
   * becomes a buzzer on a single frequency instead of an engine with a rumble under it.
   */
  cylinderSpread: number;
  /**
   * Ground reflection coefficient, 0..1. Asphalt and concrete are near 0.9, grass
   * nearer 0.4.
   *
   * The reflected path is longer than the direct one, so the two comb-filter each
   * other — at 1.5 m that puts the first null near 400 Hz. It is a large, very
   * characteristic colouration, and its absence is much of why synthetic engine audio
   * sounds like it was recorded in a vacuum.
   */
  groundReflection: number;
  /** Master output gain, linear. */
  outputGain: number;
  /** Scales valve-seating clacks and piston/mechanical noise, 0..1. */
  mechNoise: number;
  /** Scales broadband turbulence generated at the valve throat, 0..1. */
  throatNoise: number;
}

export interface EngineConfig {
  engine: EngineSpec;
  /**
   * The duct each cylinder's exhaust valve feeds. For a single this is the whole exhaust; with
   * more cylinders it is the runner every compiled graph starts from.
   *
   * Each compiled runner gets its own copy, so one array seeds them all, and unequal-length
   * runners are made by editing the graph's ducts rather than here. A cylinder that feeds a
   * manifold gets a short stub instead.
   */
  pipe: PipeSegment[];
  /**
   * The shared duct downstream of where a group of cylinders merges, one copy per collector.
   * Unused by the `open` layout.
   */
  collector: PipeSegment[];
  /**
   * The duct graph the exhaust is built from.
   *
   * Authoritative when present: it is what the solver and the renderer both work from. `pipe` and
   * `collector` above then serve only as the *seed* — what a preset or a change of layout compiles a
   * fresh graph out of, since changing the cylinder count or the merge plan invalidates whatever was
   * drawn. Absent means the graph is compiled from them on demand.
   */
  graph?: ExhaustGraph;
}

/** Per-cylinder state, so the renderer can animate each bank. */
export interface BankSnapshot {
  /** Crank angle for this bank, deg in [0, 720). */
  crankAngle: number;
  /** Cylinder pressure, Pa. */
  cylPressure: number;
  /** Cylinder gas temperature, K. */
  cylTemp: number;
  /** Exhaust valve lift, m. */
  exLift: number;
  /** Intake valve lift, m. */
  inLift: number;
}

/** Snapshot pushed from the audio thread to the main thread at ~60 Hz for drawing. */
export interface EngineSnapshot {
  /** One entry per cylinder. */
  banks: BankSnapshot[];
  /** Crank angle, deg in [0, 720). Bank 0, kept flat for the readouts. */
  crankAngle: number;
  rpm: number;
  /** Whether the rev limiter is cutting the spark right now. */
  limiter: boolean;
  /** Whether the overrun fuel cut has stopped the fuel right now. */
  fuelCut: boolean;
  /** Cylinder pressure, Pa. Bank 0. */
  cylPressure: number;
  /** Cylinder gas temperature, K. Bank 0. */
  cylTemp: number;
  /** Exhaust valve lift, m. Bank 0. */
  exLift: number;
  /** Intake valve lift, m. Bank 0. */
  inLift: number;
  /** Instantaneous gas torque at the crank, N*m. */
  torque: number;
  /**
   * Gauge pressure sampled along the whole exhaust, Pa, resampled to a fixed
   * length so the renderer can map it onto pipe geometry without reallocating.
   * Index 0 is the port, last index is the mouth.
   */
  pipePressure: Float32Array;
  /** Peak output sample magnitude since the last snapshot, for a level meter. */
  peak: number;
  /**
   * Work the gas solver is doing: cells in the duct, and CFL substeps per audio sample.
   *
   * Reported instead of a CPU percentage because the audio thread cannot measure its own
   * time — `performance` is not exposed in AudioWorkletGlobalScope, so an in-worklet
   * timer silently reads zero. Cells times substeps is the quantity cost is proportional
   * to, and it is a fact rather than an estimate.
   */
  pipeCells: number;
  substeps: number;
  /** Mean exhaust wall temperature, K. Climbs over tens of seconds from a cold start. */
  wallTemp: number;
  /** The dyno run in progress, or `null` when none is. */
  dyno: DynoSnapshot | null;
}

/** A dyno run's state, sent with each snapshot while it runs. */
export interface DynoSnapshot {
  phase: 'pull' | 'shiftOut' | 'shiftIn' | 'cooldown';
  /** Gear engaged, 1-6. */
  gear: number;
  /** Road speed, km/h. */
  speedKmh: number;
  /** Seconds since the run started. */
  elapsed: number;
  /** Whether the last pull is over, or the run was stopped. The engine may still be winding down. */
  finished: boolean;
  /**
   * The engine cycles recorded since the last snapshot, four values each: rpm, crank torque (N*m),
   * road speed (km/h) and gear (1-6).
   */
  points: Float32Array;
}

/**
 * The car and gearbox a dyno run drives through. See `DynoRun` in drivetrain.ts.
 */
export interface DynoConfig {
  /** Gearbox ratios, first to sixth. */
  ratios: number[];
  /** Final drive ratio. */
  finalDrive: number;
  /** Tyre rolling radius, m. */
  tyreRadius: number;
  /** Mass the engine accelerates, kg: the car's, with the dyno's rollers counted in it. */
  mass: number;
  /** Engine speed each gear is pulled to before the shift, rev/min. In sixth the run ends there. */
  shiftRpm: number;
}

/** A close-ratio six-speed's gears, first to sixth. */
export const DYNO_RATIOS = [3.36, 2.09, 1.47, 1.1, 0.87, 0.71];

/** How far below the rev limiter a dyno run shifts, rev/min, so the pull never touches the cut. */
const DYNO_SHIFT_MARGIN = 150;

/**
 * A car and gearing to suit `spec`, for a dyno run.
 *
 * Sized from a rough peak power: the nominal full-throttle torque at 80% of the rev limit. The car
 * weighs about 9 kg per kW of that, as a quick road car does, held between a light motorcycle and a
 * heavy saloon. Sixth is geared so that the shift point comes at the speed the car could reach on the
 * road, where that power meets its air drag. So the gears come out right for the engine: a 500 cc single
 * tops out near 160 km/h and a 5.5 litre V8 past 300.
 */
export function fitDyno(spec: EngineSpec): DynoConfig {
  const shiftRpm = Math.max(spec.revLimit - DYNO_SHIFT_MARGIN, 1000);
  const power = fullLoadTorque(spec) * ((0.8 * spec.revLimit * 2 * Math.PI) / 60);
  const mass = Math.min(Math.max(power / 110, 180), 1900);
  // Top speed on the road: power against drag, 1/2 rho CdA v^3, for a CdA of 0.6 m^2.
  const topSpeed = Math.min(Math.max(Math.cbrt((2 * power) / (1.2 * 0.6)), 45), 90);
  const tyreRadius = 0.31;
  const top = DYNO_RATIOS[DYNO_RATIOS.length - 1]!;
  const finalDrive = ((shiftRpm * 2 * Math.PI) / 60) * tyreRadius / (top * topSpeed);
  return { ratios: [...DYNO_RATIOS], finalDrive, tyreRadius, mass, shiftRpm };
}

export const PIPE_PRESSURE_TAPS = 128;

/**
 * Brake mean effective pressure that `load = 1` stands for, Pa.
 *
 * Eleven bar is a naturally aspirated petrol engine's full-load figure, give or take a couple. It fixes
 * the load's scale by displacement alone, which is what makes one setting mean the same on a single and
 * a V8; the engine's own torque curve would be more exact, but is only known by running it.
 */
export const FULL_LOAD_BMEP = 11e5;

/**
 * How far below `revLimit` the free-running crank must fall before the spark returns, rev/min.
 * A production hard-cut limiter sits in the 100-300 range; without the gap it would toggle on the
 * crank's own within-cycle ripple rather than on the speed.
 */
export const REV_LIMIT_HYSTERESIS_RPM = 200;

/** Number of crank degrees in one full four-stroke cycle. */
export const CYCLE_DEG = 720;

// ---------------------------------------------------------------------------
// Gas properties
// ---------------------------------------------------------------------------

export const GAS = {
  /** Specific gas constant for air / exhaust, J/(kg*K). */
  R: 287,
  /**
   * Ratio of specific heats of cylinder gas at compression temperatures, around 500 K. Where the
   * cylinder's gamma is needed as a single number; the cylinder itself uses `gasCv`.
   */
  gammaCyl: 1.35,
  /** Ratio of specific heats in the exhaust pipe. */
  gammaExh: 1.33,
  /**
   * Ratio of specific heats in ambient air.
   *
   * Distinct from `gammaExh` because the air *outside* the pipe is not exhaust gas. It
   * matters wherever the exterior medium sets the answer: the radiation load on the mouth,
   * and the path delays to the listener. Using 1.33 there gives 334 m/s instead of 343.
   */
  gammaAir: 1.4,
  /** Ambient pressure, Pa. */
  pAmb: 101325,
  /** Ambient temperature, K. */
  tAmb: 293,
  /** Cylinder wall temperature, K. */
  tWall: 450,
  /** Lower heating value of gasoline, J/kg. */
  fuelLhv: 43.2e6,
  /** Stoichiometric air-fuel ratio of gasoline, by mass. */
  afrStoich: 14.7,
} as const;

/**
 * Throttle opening at or below which the throttle counts as shut for the overrun fuel cut. The pedal
 * is off; the plate sits on its stop with the idle bypass open.
 */
export const FUEL_CUT_THROTTLE = 0.005;

/**
 * Speeds at which the overrun fuel cut acts, rev/min: the fuel stops above the first with the
 * throttle shut, and returns below the second, so the engine settles into its idle rather than
 * stalling. The gap is what a production ECU leaves so the cut does not toggle on the crank's own
 * ripple.
 */
export const FUEL_CUT_RPM = 1500;
export const FUEL_RESUME_RPM = 1200;

/** Fuel mass fraction of a charge mixed at `lambda`, 0..1. */
export function fuelFractionAt(lambda: number): number {
  return 1 / (1 + Math.max(lambda, 0.05) * GAS.afrStoich);
}

/**
 * Specific heat of the gas in the cylinder and the plenum, J/(kg*K), rising linearly with
 * temperature: `cv(T) = CV_REF + CV_SLOPE (T - T_REF)`.
 *
 * Temperature, not composition, is what moves gamma. As a gas heats up its molecules' vibrational
 * modes come into play and take up energy, so cv climbs: burned gas expanding at 1500-2500 K sits
 * near gamma 1.25-1.30, while the same burned gas pushed back in as residual and compressed at
 * 500 K is back near 1.35, like the fresh charge around it. Fixed at the compression value, the
 * burned gas's cv comes out a quarter too small, which overheats the combustion by several hundred
 * kelvin and carries through to the blowdown pressure and the exhaust temperature.
 *
 * Anchored at gamma 1.35 at 500 K (compression) and 1.28 at 1800 K (expansion). That gives 1.36 at
 * room temperature and 1.25 at 2500 K.
 */
const CV_AT_500 = GAS.R / (1.35 - 1);
const CV_AT_1800 = GAS.R / (1.28 - 1);
export const CV_SLOPE = (CV_AT_1800 - CV_AT_500) / (1800 - 500);

/**
 * Datum of the sensible internal energy, K. The heating value is the energy released between
 * reactants and products *at room temperature*, so that is where the energy is zero.
 */
export const T_REF = 298;
export const CV_REF = CV_AT_500 + CV_SLOPE * (T_REF - 500);

/** cv at `t` (K), J/(kg*K). */
export function gasCv(t: number): number {
  return CV_REF + CV_SLOPE * (t - T_REF);
}

/** Ratio of specific heats at `t` (K). */
export function gasGamma(t: number): number {
  return 1 + GAS.R / gasCv(t);
}

/** Sensible internal energy at `t` (K), J/kg: the integral of `gasCv` from `T_REF`. */
export function gasEnergy(t: number): number {
  const d = t - T_REF;
  return d * (CV_REF + 0.5 * CV_SLOPE * d);
}

/**
 * Temperature, K, at sensible internal energy `u` (J/kg): `gasEnergy` inverted, in the form
 * that loses no precision as `CV_SLOPE` goes to zero.
 */
export function gasTemperature(u: number): number {
  return T_REF + (2 * u) / (CV_REF + Math.sqrt(CV_REF * CV_REF + 2 * CV_SLOPE * u));
}

/** Specific enthalpy at `t` (K), J/kg, on the same datum: `u + p/rho`. */
export function gasEnthalpy(t: number): number {
  return gasEnergy(t) + GAS.R * t;
}

/** Speed of sound in exhaust gas at temperature `t` (K), m/s. */
export function speedOfSound(t: number, gamma: number = GAS.gammaExh): number {
  return Math.sqrt(gamma * GAS.R * t);
}

/** Speed of sound in ambient air, m/s. What the radiated wave travels at once it is out. */
export function ambientSoundSpeed(): number {
  return speedOfSound(GAS.tAmb, GAS.gammaAir);
}

/** Gas density at pressure `p` (Pa) and temperature `t` (K), kg/m^3. */
export function density(p: number, t: number): number {
  return p / (GAS.R * t);
}

// ---------------------------------------------------------------------------
// Derived geometry helpers — used by both the physics and the 3D meshes, so the
// piston you see is guaranteed to be at the position the gas law is using.
// ---------------------------------------------------------------------------

/** Swept (displacement) volume, m^3. */
export function displacement(spec: EngineSpec): number {
  return (Math.PI * spec.bore * spec.bore) / 4 * spec.stroke;
}

/** Nominal full-throttle torque of the whole engine, N*m: `FULL_LOAD_BMEP` over its displacement. */
export function fullLoadTorque(spec: EngineSpec): number {
  return (FULL_LOAD_BMEP * displacement(spec) * spec.cylinders) / (4 * Math.PI);
}

/** The braking torque `load` asks for, N*m. */
export function loadTorqueOf(spec: EngineSpec): number {
  return spec.load * fullLoadTorque(spec);
}

/**
 * Diameter of the exhaust port, m: one duct that the exhaust valves share, of the same area as
 * their heads together. Every exhaust starts from it.
 */
export function exhaustPortDiameter(spec: EngineSpec): number {
  return spec.exValveDia * Math.sqrt(spec.exValveCount);
}

/** Clearance (TDC) volume, m^3. */
export function clearanceVolume(spec: EngineSpec): number {
  return displacement(spec) / (spec.compressionRatio - 1);
}

/**
 * Distance from crank centre to the piston pin, m, for crank angle `deg`.
 * Maximum at TDC (deg = 0), minimum at BDC.
 */
export function pistonPosition(spec: EngineSpec, deg: number): number {
  const { a, a2, l2 } = crankGeometry(spec);
  const th = deg * DEG_TO_RAD;
  const sin = Math.sin(th);
  return a * Math.cos(th) + Math.sqrt(Math.max(l2 - a2 * sin * sin, 0));
}

/**
 * d(piston position)/d(crank angle), m per radian.
 *
 * Note this is not simply `-a*sin(theta)`: the second term is the rod-obliquity
 * correction, which is what makes piston motion non-sinusoidal and puts a
 * second-order component in the inertia torque.
 */
export function dPistonDTheta(spec: EngineSpec, deg: number): number {
  const a = spec.stroke / 2;
  const l = spec.rodLength;
  const th = (deg * Math.PI) / 180;
  const sin = Math.sin(th);
  const cos = Math.cos(th);
  const r = Math.sqrt(Math.max(l * l - a * a * sin * sin, 1e-12));
  return -a * sin - (a * a * sin * cos) / r;
}

/** d2(piston position)/d(crank angle)^2, m per radian^2. */
export function d2PistonDTheta2(spec: EngineSpec, deg: number): number {
  const a = spec.stroke / 2;
  const l = spec.rodLength;
  const th = (deg * Math.PI) / 180;
  const sin = Math.sin(th);
  const cos = Math.cos(th);
  const r = Math.sqrt(Math.max(l * l - a * a * sin * sin, 1e-12));
  const cos2 = cos * cos - sin * sin;
  return -a * cos - a * a * (cos2 / r + (a * a * sin * sin * cos * cos) / (r * r * r));
}

/** Cylinder volume, m^3, at crank angle `deg`. */
export function cylinderVolume(spec: EngineSpec, deg: number): number {
  const a = spec.stroke / 2;
  const area = (Math.PI * spec.bore * spec.bore) / 4;
  const top = a + spec.rodLength;
  return clearanceVolume(spec) + area * (top - pistonPosition(spec, deg));
}

/**
 * Everything the cylinder model needs from the crank at one angle, from one `sin`, one `cos`
 * and one `sqrt`.
 *
 * The four functions above are the readable form and are what the renderer and the tests use,
 * but the simulation calls all of them at the *same* angle, several times per audio sample per
 * cylinder — five or six sin/cos/sqrt triples where one will do. Called separately they cost
 * about 6% of total CPU between them.
 *
 * Fills `out` rather than returning a fresh object, because this is on the per-substep path.
 * `crankState` is verified against the individual functions across the whole cycle, so the
 * duplicated algebra cannot drift.
 */
export interface CrankState {
  /**
   * Crank angle to evaluate at, degrees: the input to `crankAt`.
   *
   * Carried in the scratch object rather than passed, because a floating-point argument to a call the
   * engine does not inline is boxed into a fresh heap object — and this is called twice per cylinder per
   * substep.
   */
  angle: number;
  /** Distance from crank centre to the piston pin, m. */
  position: number;
  /** dx/dtheta, m per radian. */
  dPosition: number;
  /** d2x/dtheta^2, m per radian^2. */
  d2Position: number;
  /** Cylinder volume, m^3. */
  volume: number;
  /** dV/dtheta, m^3 per radian. */
  dVolume: number;
}

export function makeCrankState(): CrankState {
  // Fractional starting values, so every field is laid out as a double from the start and a later write
  // updates it in place rather than changing the object's shape.
  return { angle: 0.5, position: 0.5, dPosition: 0.5, d2Position: 0.5, volume: 0.5, dVolume: 0.5 };
}
/**
 * Spec-derived crank geometry, memoised on the spec object's identity.
 *
 * `crankAt` is on the per-substep path — eight cylinders times several
 * cylinder substeps times 48,000 samples a second — and without the memo every call would recompute
 * `stroke / 2`, `rodLength^2`, the bore area and `clearanceVolume` (itself a divide) from scratch.
 * None of those depend on crank angle. Profiled on a V8 at 8500 rpm, recomputing them makes the
 * crank evaluation alone 11.5% of total runtime, which for a pure function of the angle is mostly
 * bookkeeping.
 *
 * Keyed on identity rather than on the field values because `EngineSim` replaces its whole spec
 * object on every `setEngine`, so a stale entry is impossible: a changed spec is a different
 * object. One slot is enough — there is one live spec.
 */
interface CrankGeometry {
  /** Crank throw, m. */
  a: number;
  a2: number;
  l2: number;
  /** Bore cross-section, m^2. */
  area: number;
  /** Clearance volume, m^3. */
  vc: number;
  /** `a + rodLength`, the piston position at TDC. */
  top: number;
}

let crankMemoSpec: EngineSpec | null = null;
let crankMemo: CrankGeometry | null = null;

function crankGeometry(spec: EngineSpec): CrankGeometry {
  if (crankMemoSpec === spec && crankMemo !== null) return crankMemo;
  const a = spec.stroke / 2;
  const l = spec.rodLength;
  const g: CrankGeometry = {
    a,
    a2: a * a,
    l2: l * l,
    area: (Math.PI * spec.bore * spec.bore) / 4,
    vc: clearanceVolume(spec),
    top: a + l,
  };
  crankMemoSpec = spec;
  crankMemo = g;
  return g;
}

const DEG_TO_RAD = Math.PI / 180;


export function crankState(spec: EngineSpec, deg: number, out: CrankState): CrankState {
  out.angle = deg;
  return crankAt(spec, out);
}

/** `crankState` at `out.angle`, for the per-substep path: see `CrankState.angle`. */
export function crankAt(spec: EngineSpec, out: CrankState): CrankState {
  const { a, a2, l2, area, vc, top } = crankGeometry(spec);
  const th = out.angle * DEG_TO_RAD;
  const sin = Math.sin(th);
  const cos = Math.cos(th);
  const root = Math.sqrt(Math.max(l2 - a2 * sin * sin, 1e-12));
  const invRoot = 1 / root;

  const position = a * cos + root;
  // Rod obliquity: the second term is what makes the motion non-sinusoidal.
  const dPosition = -a * sin - a2 * sin * cos * invRoot;
  const cos2 = cos * cos - sin * sin;
  const d2Position =
    -a * cos - a2 * (cos2 * invRoot + a2 * sin * sin * cos * cos * invRoot * invRoot * invRoot);

  out.position = position;
  out.dPosition = dPosition;
  out.d2Position = d2Position;
  out.volume = vc + area * (top - position);
  // V = Vc + A (top - x), so dV/dtheta is just -A dx/dtheta.
  out.dVolume = -area * dPosition;
  return out;
}

/** d(volume)/d(crank angle), m^3 per radian, at crank angle `deg`. */
export function dVolumeDTheta(spec: EngineSpec, deg: number): number {
  const a = spec.stroke / 2;
  const l = spec.rodLength;
  const area = (Math.PI * spec.bore * spec.bore) / 4;
  const th = (deg * Math.PI) / 180;
  const sin = Math.sin(th);
  const cos = Math.cos(th);
  const root = Math.sqrt(Math.max(l * l - a * a * sin * sin, 1e-12));
  // d/dth of -(a*cos + root) == a*sin + a^2*sin*cos/root
  return area * (a * sin + (a * a * sin * cos) / root);
}

// ---------------------------------------------------------------------------
// Pipe helpers
// ---------------------------------------------------------------------------

/** Cross-sectional area, m^2, at normalised position `u` (0..1) within a segment. */
export function segmentArea(seg: PipeSegment, u: number): number {
  const d = segmentDiameter(seg, u);
  return (Math.PI * d * d) / 4;
}

/**
 * Diameter, m, at normalised position `u` (0..1) within a segment.
 *
 * For a non-round chamber body this is the diameter of the circle with the same area, which is
 * all the 1D acoustics needs. `segmentSection` gives the real shape.
 */
export function segmentDiameter(seg: PipeSegment, u: number): number {
  switch (seg.kind) {
    case 'pipe':
      return seg.dIn;
    case 'cone':
      return seg.dIn + (seg.dOut - seg.dIn) * u;
    case 'chamber':
      // Stepped: a short entry throat at dIn, the body at dOut, then back to dIn.
      // The steps are what make a muffler reflect rather than just absorb.
      if (u < CHAMBER_THROAT || u > 1 - CHAMBER_THROAT) return seg.dIn;
      if ((seg.section ?? 'round') === 'round') return seg.dOut;
      return Math.sqrt((4 * sectionArea(chamberBody(seg))) / Math.PI);
  }
}

/** Fraction of a chamber's length taken by each of its throats. */
export const CHAMBER_THROAT = 0.08;

/** Corner radius of a `rect` section, as a fraction of its shorter side. */
export const RECT_CORNER = 0.15;

export interface Section {
  section: ChamberSection;
  width: number;
  height: number;
}

/** The body cross-section of a chamber. */
export function chamberBody(seg: PipeSegment): Section {
  const section = seg.section ?? 'round';
  const width = seg.dOut;
  return { section, width, height: section === 'round' ? width : (seg.height ?? width) };
}

/** Cross-section at normalised position `u` within a segment: round everywhere but a chamber's body. */
export function segmentSection(seg: PipeSegment, u: number): Section {
  if (seg.kind === 'chamber' && u >= CHAMBER_THROAT && u <= 1 - CHAMBER_THROAT) return chamberBody(seg);
  const d = segmentDiameter(seg, u);
  return { section: 'round', width: d, height: d };
}

export function sectionArea(s: Section): number {
  switch (s.section) {
    case 'round':
      return (Math.PI * s.width * s.width) / 4;
    case 'oval':
      return (Math.PI * s.width * s.height) / 4;
    case 'rect': {
      const r = RECT_CORNER * Math.min(s.width, s.height);
      return s.width * s.height - (4 - Math.PI) * r * r;
    }
  }
}

/** Wetted perimeter, m. Ramanujan's second approximation for the ellipse, good to 1e-5 here. */
export function sectionPerimeter(s: Section): number {
  switch (s.section) {
    case 'round':
      return Math.PI * s.width;
    case 'oval': {
      const a = s.width / 2;
      const b = s.height / 2;
      const h = ((a - b) * (a - b)) / ((a + b) * (a + b));
      return Math.PI * (a + b) * (1 + (3 * h) / (10 + Math.sqrt(4 - 3 * h)));
    }
    case 'rect': {
      const r = RECT_CORNER * Math.min(s.width, s.height);
      return 2 * (s.width + s.height) - (8 - 2 * Math.PI) * r;
    }
  }
}

/** Whether the point `(y, z)`, measured from the section's centre along its width and height, is inside it. */
export function insideSection(s: Section, y: number, z: number): boolean {
  const a = s.width / 2;
  const b = s.height / 2;
  switch (s.section) {
    case 'round':
    case 'oval':
      return (y * y) / (a * a) + (z * z) / (b * b) <= 1;
    case 'rect': {
      const r = RECT_CORNER * Math.min(s.width, s.height);
      const dy = Math.abs(y) - (a - r);
      const dz = Math.abs(z) - (b - r);
      if (Math.abs(y) > a || Math.abs(z) > b) return false;
      if (dy <= 0 || dz <= 0) return true;
      return dy * dy + dz * dz <= r * r;
    }
  }
}

/**
 * Where a ray from the section's centre at angle `theta` (from the width axis) meets its wall.
 * Writes the distance and the outward unit normal, as `[r, ny, nz]`, into `out`.
 */
export function sectionBoundary(s: Section, theta: number, out: Float64Array): void {
  const a = s.width / 2;
  const b = s.height / 2;
  const cy = Math.cos(theta);
  const cz = Math.sin(theta);
  const sy = cy < 0 ? -1 : 1;
  const sz = cz < 0 ? -1 : 1;
  const dy = Math.abs(cy);
  const dz = Math.abs(cz);
  if (s.section !== 'rect') {
    const r = 1 / Math.sqrt((dy * dy) / (a * a) + (dz * dz) / (b * b));
    const ny = (r * dy) / (a * a);
    const nz = (r * dz) / (b * b);
    const len = Math.hypot(ny, nz);
    out[0] = r;
    out[1] = (sy * ny) / len;
    out[2] = (sz * nz) / len;
    return;
  }
  const rc = RECT_CORNER * Math.min(s.width, s.height);
  // The flat sides first; a ray that meets neither inside its straight part is in a corner.
  if (dy > 1e-12) {
    const t = a / dy;
    if (t * dz <= b - rc) {
      out[0] = t;
      out[1] = sy;
      out[2] = 0;
      return;
    }
  }
  if (dz > 1e-12) {
    const t = b / dz;
    if (t * dy <= a - rc) {
      out[0] = t;
      out[1] = 0;
      out[2] = sz;
      return;
    }
  }
  // |t d - c| = rc for the corner centre c, taking the far root.
  const ccy = a - rc;
  const ccz = b - rc;
  const half = dy * ccy + dz * ccz;
  const t = half + Math.sqrt(Math.max(half * half - (ccy * ccy + ccz * ccz - rc * rc), 0));
  out[0] = t;
  out[1] = (sy * (t * dy - ccy)) / rc;
  out[2] = (sz * (t * dz - ccz)) / rc;
}

/**
 * A chamber's inlet and outlet offsets, m, held to what fits: a pipe cannot sit further off
 * centre than leaves its whole bore inside the body.
 */
export function chamberOffsets(seg: PipeSegment): [number, number] {
  if (seg.kind !== 'chamber') return [0, 0];
  const room = Math.max(0, (seg.dOut - seg.dIn) / 2);
  const hold = (v: number | undefined) => clampTo(v ?? 0, -room, room);
  return [hold(seg.offsetIn), hold(seg.offsetOut)];
}

function clampTo(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

export function totalPipeLength(pipe: PipeSegment[]): number {
  let l = 0;
  for (const s of pipe) l += s.length;
  return l;
}

/**
 * Gas temperature, K, at distance `x` along the pipe. Exhaust cools as it travels,
 * which raises the impedance and slows the wave speed toward the mouth — a real and
 * audible effect, so it is modelled rather than assumed constant.
 */
export function pipeTemperature(portTemp: number, x: number): number {
  const decayLength = 1.6; // m, empirical
  return GAS.tAmb + (portTemp - GAS.tAmb) * Math.exp(-x / decayLength);
}

// ---------------------------------------------------------------------------
// Defaults and presets
// ---------------------------------------------------------------------------

let idCounter = 0;
export function newSegmentId(): string {
  return `seg${++idCounter}`;
}

export function makeSegment(partial: Partial<PipeSegment> = {}): PipeSegment {
  const kind = partial.kind ?? 'pipe';
  const dIn = partial.dIn ?? 0.042;
  const seg: PipeSegment = {
    id: partial.id ?? newSegmentId(),
    kind,
    length: partial.length ?? 0.3,
    dIn,
    dOut: kind === 'pipe' ? dIn : (partial.dOut ?? dIn),
    yaw: partial.yaw ?? 0,
    pitch: partial.pitch ?? 0,
  };
  if (kind === 'chamber') {
    // Checked rather than copied, since a segment can arrive from a shared link.
    const finite = (v: unknown): v is number => typeof v === 'number' && Number.isFinite(v);
    if (partial.section && partial.section !== 'round' && CHAMBER_SECTIONS.includes(partial.section)) {
      seg.section = partial.section;
      seg.height = finite(partial.height) && partial.height > 1e-3 ? partial.height : seg.dOut;
    }
    if (finite(partial.offsetIn) && partial.offsetIn !== 0) seg.offsetIn = partial.offsetIn;
    if (finite(partial.offsetOut) && partial.offsetOut !== 0) seg.offsetOut = partial.offsetOut;
  }
  return seg;
}

/** A 500cc-ish thumper: 89 mm bore, 80 mm stroke. */
export const DEFAULT_ENGINE: EngineSpec = {
  cylinders: 1,
  vAngle: 45,
  firingOffset: null,
  crankType: 'shared',
  exhaustLayout: 'open',
  bore: 0.089,
  stroke: 0.08,
  rodLength: 0.145,
  compressionRatio: 10.5,

  exValveDia: 0.034,
  exValveCount: 1,
  portLength: 0.055,
  inValveDia: 0.04,
  inValveCount: 1,
  maxLift: 0.0095,
  evo: 128,
  evc: 378,
  ivo: 342,
  ivc: 576,

  ignition: 695, // 25 deg BTDC
  burnDuration: 55,
  advanceCurve: true,
  lambda: 1,
  fuelCut: true,
  combustionVariability: 1,
  recipMass: 0.55,
  throttle: 0.75,
  // Zero means "derive it from the engine" — see `throttleDiaOf` and `plenumVolumeOf`.
  // A fixed figure here would be a single-cylinder's, and wrong for everything else.
  throttleDia: 0,
  plenumVolume: 0,

  rpm: 3200,
  revLimit: 7000,
  freeRunning: false,
  // A bare crank is nearer 0.06; 0.25 represents crank plus clutch and primary
  // drive, which is what a rider actually hears. Lower it for a lumpier idle.
  flywheelInertia: 0.25,
  // About 20 N*m on this engine.
  load: 0.46,

  portGasTemp: 950,
  pipeCellSize: 0.035,
  pipeWallThickness: 0.0012,
  airSpeed: 0,
  micDistance: 1.5,
  micHeight: 1.2,
  exhaustHeight: 0.35,
  mouthSpacing: 0.4,
  cylinderSpread: 1,
  groundReflection: 0.7,
  outputGain: 0.77,
  mechNoise: 0.45,
  throatNoise: 0.5,
};

export interface Preset {
  name: string;
  description: string;
  build: () => PipeSegment[];
}

export const PIPE_PRESETS: Preset[] = [
  {
    name: 'Open header',
    description: 'Short straight pipe, no silencer. Raw and loud, strongly tuned.',
    build: () => [
      makeSegment({ kind: 'pipe', length: 0.42, dIn: 0.042 }),
      makeSegment({ kind: 'pipe', length: 0.18, dIn: 0.048, pitch: -0.12 }),
    ],
  },
  {
    name: 'Megaphone',
    description: 'Header into a long taper. Broad, brassy, race-bike bark.',
    build: () => [
      makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.04 }),
      makeSegment({ kind: 'cone', length: 0.55, dIn: 0.04, dOut: 0.1, yaw: 0.1 }),
    ],
  },
  {
    name: 'Tuned expansion chamber',
    description: 'Diffuser then reverse cone. The classic two-stroke-style pulse tuner.',
    build: () => [
      makeSegment({ kind: 'pipe', length: 0.22, dIn: 0.038 }),
      makeSegment({ kind: 'cone', length: 0.3, dIn: 0.038, dOut: 0.11 }),
      makeSegment({ kind: 'pipe', length: 0.12, dIn: 0.11 }),
      makeSegment({ kind: 'cone', length: 0.24, dIn: 0.11, dOut: 0.026 }),
      makeSegment({ kind: 'pipe', length: 0.16, dIn: 0.026, pitch: -0.15 }),
    ],
  },
  {
    name: 'Street muffler',
    description: 'Long header, expansion can, short tailpipe. Quiet and boomy.',
    build: () => [
      makeSegment({ kind: 'pipe', length: 0.6, dIn: 0.042, yaw: 0.25 }),
      makeSegment({ kind: 'chamber', length: 0.34, dIn: 0.042, dOut: 0.13 }),
      makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.04, pitch: -0.1 }),
    ],
  },
  {
    name: 'Long tuned pipe',
    description: 'Very long primary. Low first resonance, strong low-rpm pull.',
    build: () => [
      makeSegment({ kind: 'pipe', length: 0.9, dIn: 0.04, yaw: 0.4 }),
      makeSegment({ kind: 'cone', length: 0.35, dIn: 0.04, dOut: 0.06, yaw: 0.2 }),
      makeSegment({ kind: 'pipe', length: 0.25, dIn: 0.06 }),
    ],
  },
  {
    name: 'Oval muffler, offset pipes',
    description: 'Flat oval can with its pipes at opposite sides. Rings across the can, rougher in the mids.',
    build: () => [
      makeSegment({ kind: 'pipe', length: 0.6, dIn: 0.042, yaw: 0.25 }),
      makeSegment({
        kind: 'chamber',
        length: 0.4,
        dIn: 0.042,
        dOut: 0.26,
        section: 'oval',
        height: 0.13,
        offsetIn: 0.07,
        offsetOut: -0.07,
      }),
      makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.04, pitch: -0.1 }),
    ],
  },
];

/**
 * Crank degrees from cylinder 1 firing to cylinder 2 firing.
 *
 * With a shared crankpin the second cylinder reaches TDC `vAngle` degrees of crank rotation
 * after the first, and since a four-stroke fires every 720 the camshaft puts that firing a
 * further revolution away: `360 + vAngle`. The two intervals are therefore `360 + vAngle` and
 * `360 - vAngle`, which is the familiar 405/315 of a 45-degree Harley.
 */
export function firingOffsetDeg(spec: EngineSpec): number {
  const derived = 360 + spec.vAngle;
  return ((spec.firingOffset ?? derived) % 720 + 720) % 720;
}

/**
 * Which firings happen when, and which bank each belongs to.
 *
 * `offsets[i]` is the crank angle at which cylinder `i` fires, relative to cylinder 1;
 * `banks[i]` is 0 or 1. Every cylinder is otherwise identical, so this is the whole of an
 * engine's layout as far as the sound is concerned.
 *
 * This is **data, not a derivation**, for everything above two cylinders. Real firing orders
 * are chosen for crankshaft balance and bearing loads, and cannot be recovered from a formula;
 * quoting them from the engines they belong to is both honest and shorter. The twin is the
 * exception, where the shared-crankpin relationship genuinely does derive the interval and is
 * worth keeping explicit, because a rider can hear the V angle in it.
 *
 * The V8 entries are the interesting pair. Both fire every 90 degrees — eight firings over the
 * 720-degree cycle — so through one collector they are near enough the same engine. Split into
 * two banks they are not:
 *
 *   crossplane   L R L L R L R R     banks fire unevenly: 180-90-180-270 apart
 *   flatplane    L R L R L R L R     each bank fires evenly, every 180
 *
 * That is the entire difference between a muscle-car burble and a Ferrari shriek, and here it
 * costs one array. The crossplane bank pattern is the Ford 302 order 1-5-4-2-6-3-7-8 with
 * cylinders 1-4 on the left bank; the flatplane pattern is the alternating order a
 * flat-crank V8 gives.
 */
export interface FiringPlan {
  /** Crank degrees after cylinder 1 at which each cylinder fires, in [0, 720). */
  offsets: number[];
  /** Bank index per cylinder, 0 or 1. */
  banks: number[];
  /** Number of banks actually used. */
  bankCount: number;
  /**
   * Which crank throw each cylinder is on, where that is not simply one per pin or a shared pin.
   *
   * A split-pin crank — a 60-degree V6's — puts two cylinders on one throw but on two separate pins, set
   * apart round the shaft so the engine fires evenly despite a vee too narrow for that on a shared pin.
   * That pairing cannot be recovered from the firing offsets the way a shared pin can, so it is stated.
   */
  throws?: number[];
}

/**
 * The two V8 cranks, stored as pin angles rather than as firing offsets.
 *
 * A firing offset is only meaningful for *one* bank angle. A cylinder on the second bank
 * reaches TDC `vAngle` degrees of crank rotation after the pin partner it shares a throw with,
 * so its firing angle moves when the vee moves. Storing offsets directly would make the bank
 * angle inert on a V8: it would produce even 90-degree firing at every angle, which is true of
 * a 90-degree V8 and of nothing else. A 60-degree vee on a 90-degree crank fires unevenly, and
 * that is most of the reason real 60-degree V8s are rare.
 *
 * `pins` is where each cylinder's throw sits round the shaft, `revs` is which revolution of the
 * two-revolution cycle the camshaft fires it in, and the firing angle is
 *
 *     offset = pin + vAngle * bank + 360 * rev
 *
 * which is exactly the relationship `crankPins` inverts to draw the mechanism. At `vAngle = 90`
 * both cranks fire evenly every 90 degrees, as a 90-degree V8 does.
 *
 * Read off real engines: crossplane is the Ford 302 order with cylinders 1-4 on
 * the left bank, giving pins at 0/90/180/270 — the four-plane crank. Flatplane puts every throw
 * in one plane, so its pins are only ever 0 or 180.
 */
interface V8Crank {
  pins: number[];
  revs: number[];
  banks: number[];
}

const V8_CROSSPLANE: V8Crank = {
  pins: [0, 0, 180, 270, 270, 90, 90, 180],
  revs: [0, 0, 0, 0, 0, 1, 1, 1],
  banks: [0, 1, 0, 0, 1, 0, 1, 1],
};

const V8_FLATPLANE: V8Crank = {
  pins: [0, 0, 180, 180, 0, 0, 180, 180],
  revs: [0, 0, 0, 0, 1, 1, 1, 1],
  banks: [0, 1, 0, 1, 0, 1, 0, 1],
};

/**
 * The 60-degree V6: a split-pin crank, stored as pins as the V8s are.
 *
 * The pins of each throw are 60 degrees apart round the shaft, which at a 60-degree vee gives the even
 * 120-degree firing of a GM or Nissan V6 in the order 1-2-3-4-5-6, cylinders 1-3-5 on one bank. At any
 * other vee the same crank fires unevenly, as it would, because the offset follows
 * `pin + vAngle * bank + 360 * rev`.
 */
const V6_SPLIT_PIN: V8Crank = {
  pins: [0, 60, 240, 300, 120, 180],
  revs: [0, 0, 0, 0, 1, 1],
  banks: [0, 1, 0, 1, 0, 1],
};
const V6_THROWS = [0, 0, 1, 1, 2, 2];

/**
 * Inline engines: every cylinder on its own pin, one bank, firing evenly in the usual order.
 *
 * The offsets are by cylinder, read off the firing order: an inline three fires 1-3-2 every 240 degrees
 * on a 120-degree crank; an inline five 1-2-4-5-3 every 144 on a 72-degree crank, the Audi and Volvo
 * order; an inline six 1-5-3-6-2-4 every 120, which pairs its throws 1-6, 2-5 and 3-4.
 */
/**
 * Flat fours and sixes, stored as pins as the V engines are: a throw per cylinder, banks alternating
 * along the crank, and each opposed pair's pins half a turn apart.
 *
 * With the banks 180 degrees apart, pins half a turn apart put both pistons of a pair at top dead
 * centre at once, which is what a boxer is — they move out and in together, so their inertia forces
 * cancel. The camshaft fires one of the pair and, a revolution later, the other.
 *
 * The four is the Subaru and VW order 1-3-2-4, cylinders 1 and 3 on one bank: pairs at 0 and 180,
 * so a firing every 180 degrees. The six is the Porsche order 1-6-2-4-3-5 with 1-2-3 on one bank and 4
 * opposite 1: pairs at 0, 120 and 240, a firing every 120. Cylinders are indexed by throw from the
 * front, so here index 1 is the Porsche's 4, index 2 its 2, and so on.
 */
const BOXER_4: V8Crank = {
  pins: [0, 180, 180, 0],
  revs: [0, 0, 0, 1],
  banks: [0, 1, 0, 1],
};
const BOXER_6: V8Crank = {
  pins: [0, 180, 240, 60, 120, 300],
  revs: [0, 0, 0, 1, 1, 1],
  banks: [0, 1, 0, 1, 0, 1],
};

/** Whether the spec is a flat four or flat six. */
export function isBoxer(spec: EngineSpec): boolean {
  return spec.crankType === 'boxer' && (spec.cylinders === 4 || spec.cylinders === 6);
}

/** A boxer's plan: offsets from the pins as for a V, and a throw of its own for every cylinder. */
function boxerPlan(spec: EngineSpec): FiringPlan {
  const crank = spec.cylinders === 6 ? BOXER_6 : BOXER_4;
  const offsets = crank.pins.map(
    (pin, i) => (((pin + spec.vAngle * crank.banks[i]! + 360 * crank.revs[i]!) % 720) + 720) % 720,
  );
  return { offsets, banks: [...crank.banks], bankCount: 2, throws: crank.pins.map((_, i) => i) };
}

const INLINE_OFFSETS: Record<number, number[]> = {
  3: [0, 480, 240],
  5: [0, 144, 576, 288, 432],
  6: [0, 480, 240, 600, 120, 360],
};

export function firingPlan(spec: EngineSpec): FiringPlan {
  if (isBoxer(spec)) return boxerPlan(spec);
  switch (spec.cylinders) {
    case 3:
    case 5: {
      const offsets = INLINE_OFFSETS[spec.cylinders]!;
      return { offsets: [...offsets], banks: offsets.map(() => 0), bankCount: 1 };
    }
    case 6: {
      if (!(spec.vAngle > 0)) {
        const offsets = INLINE_OFFSETS[6]!;
        return { offsets: [...offsets], banks: offsets.map(() => 0), bankCount: 1 };
      }
      const crank = V6_SPLIT_PIN;
      const offsets = crank.pins.map(
        (pin, i) =>
          (((pin + spec.vAngle * crank.banks[i]! + 360 * crank.revs[i]!) % 720) + 720) % 720,
      );
      return { offsets, banks: [...crank.banks], bankCount: 2, throws: [...V6_THROWS] };
    }
    case 1:
      return { offsets: [0], banks: [0], bankCount: 1 };
    case 2:
      // Two banks of one: a V-twin's cylinders are each their own bank.
      return { offsets: [0, firingOffsetDeg(spec)], banks: [0, 1], bankCount: 2 };
    case 4:
      // Inline four on a flat crank, pins 0/180/180/0 so the outer pair and the inner pair each
      // share a throw: fires 1-3-4-2, every 180 degrees, one bank.
      return { offsets: [0, 540, 180, 360], banks: [0, 0, 0, 0], bankCount: 1 };
    case 8: {
      const crank = spec.crankType === 'flatplane' ? V8_FLATPLANE : V8_CROSSPLANE;
      const offsets = crank.pins.map(
        (pin, i) =>
          (((pin + spec.vAngle * crank.banks[i]! + 360 * crank.revs[i]!) % 720) + 720) % 720,
      );
      return { offsets, banks: [...crank.banks], bankCount: 2 };
    }
  }
}

/**
 * One crankpin: where it sits round the shaft, and which cylinders hang off it.
 *
 * Recovered from the firing plan rather than stored, because it *is* recoverable: two cylinders
 * share a pin exactly when the second's firing follows the first's by the bank angle, or by the
 * bank angle plus a revolution. Which of the two is a choice the camshaft makes, and having both
 * available is what lets a V8 fire evenly every 90 degrees while still dealing those firings out
 * unevenly between its banks.
 *
 * That freedom is easy to miss. Assuming the pairing is always `vAngle + 360` — as it is for a
 * V-twin — makes an evenly firing 90-degree V8 impossible to construct, which would "prove" that
 * the crossplane burble cannot exist. It does exist; the assumption is what is wrong.
 *
 * Used for drawing the mechanism and for placing the exhaust ports along the crank, whose spacing a
 * compiled manifold's lengths follow. The firing depends on the firing plan alone.
 */
export interface CrankPin {
  /** Angle round the shaft, degrees in [0, 360): the first cylinder's. */
  angleDeg: number;
  /** Cylinder indices on this throw: one per bank, or one alone. */
  cylinders: number[];
  /**
   * Each of those cylinders' own pin angle, in the same order. All equal on a shared pin; different on
   * a split pin, where the throw carries a separate pin for each.
   */
  angles: number[];
}

/**
 * Which side of the engine a cylinder physically sits on: 0 or 1 in a V, 0 for everything else.
 *
 * Not the same as `firingPlan(spec).banks`. A parallel twin is modelled as two banks at a 0-degree vee,
 * because that is how its firing offset is expressed, but both its cylinders are under one head on one side
 * of the engine — so anything physical, where the exhaust comes out or what shares a casting, has to ask
 * this instead.
 */
export function physicalBank(spec: EngineSpec, cylinder: number): number {
  if (!(spec.vAngle > 0)) return 0;
  return firingPlan(spec).banks[cylinder] ?? 0;
}

/** How many physical banks the engine has: 2 for a V, 1 otherwise. */
export function physicalBankCount(spec: EngineSpec): number {
  return spec.vAngle > 0 ? firingPlan(spec).bankCount : 1;
}

/**
 * Centre-to-centre spacing of the crank pins along the crankshaft, m.
 *
 * Shared by the engine's drawing and the exhaust that is compiled for it, because a manifold that chains
 * one runner's end to the next has to be exactly this long to reach.
 */
export function cylinderSpacing(spec: EngineSpec): number {
  // A boxer's throws alternate between the banks, so neighbours along the crank point opposite ways and
  // need not clear each other's bores: half the pitch keeps each bank's own cylinders the usual distance
  // apart, and the opposed pairs just offset, as a real flat engine's are.
  if (isBoxer(spec)) return spec.bore * 0.75;
  return spec.bore * 1.45;
}

export function crankPins(spec: EngineSpec): CrankPin[] {
  const plan = firingPlan(spec);
  const pins: CrankPin[] = [];
  const taken = new Set<number>();

  /**
   * Where a pin must sit for this cylinder to reach TDC when its own crank angle is zero.
   *
   * The `- vAngle * bank` is the part that is easy to lose: a cylinder on the second bank is
   * already rotated away from the first bank's axis, so its pin has to be that much further
   * round to point up its own bore at the right moment. For a *paired* pin the correction
   * cancels — the partner's offset is larger by exactly `vAngle` — which is why leaving it out
   * looks fine on a V-twin and a V8 and goes wrong only on a cylinder that has a pin to itself,
   * such as the second cylinder of a 270-degree parallel twin.
   */
  const pinAngle = (i: number) =>
    ((((plan.offsets[i]! - spec.vAngle * plan.banks[i]!) % 360) + 360) % 360);

  /**
   * A shared pin needs a vee to hang the second bore off.
   *
   * Two cylinders on one crank throw are only distinguishable if the banks are angled apart — that
   * angle is the entire reason the bores do not occupy the same space. With `vAngle` zero the
   * pairing test degenerates: a 360-degree parallel twin fires its cylinders a revolution apart, so
   * the offsets differ by 0 modulo 360 and match a zero bank angle exactly. Paired on that, both
   * cylinders would go on one pin at one Z with no rotation between them, and be drawn precisely on
   * top of each other — along with their exhaust runners, coinciding to the millimetre.
   *
   * A parallel twin really does have two pins side by side, so that is what it gets.
   */
  const vee = (((spec.vAngle % 360) + 360) % 360);
  const canShareAPin = Math.min(vee, 360 - vee) > 1e-6;

  // A crank that says which throw each cylinder is on — a split pin — is taken as it says.
  if (plan.throws) {
    const count = Math.max(...plan.throws) + 1;
    for (let t = 0; t < count; t++) {
      const cylinders = plan.offsets.map((_, i) => i).filter((i) => plan.throws![i] === t);
      if (cylinders.length === 0) continue;
      const angles = cylinders.map(pinAngle);
      pins.push({ angleDeg: angles[0]!, cylinders, angles });
    }
    return pins;
  }

  for (let i = 0; i < plan.offsets.length; i++) {
    if (taken.has(i) || plan.banks[i] !== 0) continue;
    taken.add(i);
    const pin: CrankPin = { angleDeg: pinAngle(i), cylinders: [i], angles: [pinAngle(i)] };
    // The partner on the other bank, if the geometry admits one.
    if (canShareAPin) {
      for (let j = 0; j < plan.offsets.length; j++) {
        if (taken.has(j) || plan.banks[j] === 0) continue;
        const delta = (((plan.offsets[j]! - plan.offsets[i]!) % 360) + 360) % 360;
        if (Math.abs(delta - vee) < 1e-6) {
          pin.cylinders.push(j);
          pin.angles.push(pin.angleDeg);
          taken.add(j);
          break;
        }
      }
    }
    pins.push(pin);
  }
  // Anything unpaired — a firing offset no shared pin can produce — gets its own pin.
  for (let i = 0; i < plan.offsets.length; i++) {
    if (taken.has(i)) continue;
    pins.push({ angleDeg: pinAngle(i), cylinders: [i], angles: [pinAngle(i)] });
    taken.add(i);
  }
  return pins;
}

/** Intervals between successive firings within one bank, crank degrees. */
export function bankFiringIntervals(spec: EngineSpec, bank: number): number[] {
  const plan = firingPlan(spec);
  const fires = plan.offsets
    .filter((_, i) => plan.banks[i] === bank)
    .sort((a, b) => a - b);
  return fires.map((f, i) => {
    const next = i + 1 < fires.length ? fires[i + 1]! : fires[0]! + 720;
    return next - f;
  });
}

/** Normalised exhaust layout, accepting the older twin-only names. */
export function exhaustLayoutOf(spec: EngineSpec): ExhaustLayout {
  const raw = spec.exhaustLayout;
  if (raw === '2into1') return 'merged';
  if (raw === 'single' || raw === '2into2') return 'open';
  // A single cylinder has nothing to merge with, so every layout is one open pipe.
  if (spec.cylinders === 1) return 'open';
  // One bank means per-bank and merged are the same plumbing.
  if (raw === 'perBank' && firingPlan(spec).bankCount === 1) return 'merged';
  return raw;
}

/**
 * Which collector each cylinder feeds, or -1 for a cylinder that vents straight out.
 *
 * The grouping *is* the plumbing: cylinders sharing a number share a collector and can
 * therefore hear each other.
 */
export function collectorGroups(spec: EngineSpec): number[] {
  const layout = exhaustLayoutOf(spec);
  const plan = firingPlan(spec);
  if (layout === 'open') return plan.offsets.map(() => -1);
  if (layout === 'merged') return plan.offsets.map(() => 0);
  return plan.banks.map((b) => b);
}

/**
 * An exhaust system sized for the engine.
 *
 * This exists because exhaust geometry that does not scale with the engine is the single biggest
 * reason a multi-cylinder engine sounds wrong: a V8 breathing through the header of a 500 cc single
 * puts eight times the gas through one cylinder's pipework, which gives a thin, high note, and no
 * amount of adjusting the firing plan fixes it.
 *
 * What it gets right, specifically. With a realistic system the loudest thing in the spectrum is
 * the firing frequency itself, right across the usable rev range — measured at 1.00 to 1.01 times
 * firing from 1000 to 3500 rpm on a 2 litre four. With a short open header the *pipe's* resonance
 * dominates instead, and since that resonance does not care how many cylinders there are, every
 * engine ends up singing at the same high frequency no matter how big it is.
 *
 * The sizing rules are ordinary exhaust practice rather than anything derived:
 *
 * - Primary bore about 0.85x the exhaust port, which is what a header builder uses.
 * - Collector bore scaled as the square root of the number of pipes feeding it, so the gas sees
 *   roughly constant velocity through the merge.
 * - Total path around 2.2 m for a road system. This is the part that matters most: it puts the
 *   first resonance below the firing frequency instead of above it.
 * - Silencer volume about eight times the displacement it serves, which is the usual ballpark
 *   and far larger than a can looks like it needs to be.
 */
export interface ExhaustSizing {
  /** Per-cylinder primary. */
  pipe: PipeSegment[];
  /** Per-collector: merge cone, pipe, silencer, tailpipe. Empty for an open layout. */
  collector: PipeSegment[];
}

export function fittedExhaust(spec: EngineSpec): ExhaustSizing {
  const layout = exhaustLayoutOf(spec);
  const groups = collectorGroups(spec);
  const collectorCount = groups.reduce((max, g) => Math.max(max, g + 1), 0);
  const perCollector = collectorCount > 0 ? spec.cylinders / collectorCount : 1;

  // Primary: bore from the valve it is bolted to, length a typical header runner.
  const dPrimary = Math.max(0.85 * exhaustPortDiameter(spec), 0.02);
  const primaryLength = layout === 'open' ? 0.75 : 0.45;
  const pipe: PipeSegment[] = [
    makeSegment({ kind: 'pipe', length: primaryLength, dIn: dPrimary }),
  ];
  if (layout === 'open') {
    // Nothing downstream, so let it flare — an open pipe of this length alone is a megaphone.
    pipe.push(
      makeSegment({ kind: 'cone', length: 0.25, dIn: dPrimary, dOut: dPrimary * 1.7 }),
    );
    return { pipe, collector: [] };
  }

  // Collector: constant-velocity merge, then a long enough run to put the first resonance low.
  const dCollector = dPrimary * Math.sqrt(perCollector) * 0.92;
  // `displacement` is one cylinder's swept volume, so multiply by the cylinders this collector
  // actually serves. Treating it as the whole engine's would undersize every silencer by that factor.
  const servedDisp = displacement(spec) * perCollector;

  /**
   * Silencer can, sized by volume but with its expansion ratio held back.
   *
   * A real muffler is a 50 mm pipe opening into a 200 mm can — sixteen times the area — and the
   * solver does not survive that at these flow speeds: sized purely by volume, a four-cylinder
   * system diverges continuously (235,000 recoveries in a quarter of a second, which the audio
   * thread reports as silence). Capping the diameter ratio at 2.5 keeps it stable and still
   * silences properly; the missing volume is made up in length. The underlying fragility is a
   * solver limit worth fixing separately, not something to design around for ever.
   */
  const canDia = Math.min(dCollector * 2.5, 0.2);
  const canArea = (Math.PI * canDia * canDia) / 4;
  const canLength = Math.min(Math.max((8 * servedDisp) / canArea, 0.25), 0.6);
  const runLength = Math.max(2.2 - primaryLength - canLength - 0.5, 0.35);

  const collector: PipeSegment[] = [
    makeSegment({ kind: 'cone', length: 0.16, dIn: dPrimary * 1.25, dOut: dCollector }),
    makeSegment({ kind: 'pipe', length: runLength, dIn: dCollector, yaw: 0.2 }),
    makeSegment({ kind: 'chamber', length: canLength, dIn: dCollector, dOut: canDia }),
    // Tailpipe no narrower than the collector: choking it raises back pressure and, with this
    // much flow, is another way to upset the solver.
    makeSegment({ kind: 'pipe', length: 0.5, dIn: dCollector }),
  ];
  return { pipe, collector };
}

/** A modest collector: enough length to matter acoustically without dominating. */
export function defaultCollector(): PipeSegment[] {
  return [
    makeSegment({ kind: 'cone', length: 0.12, dIn: 0.05, dOut: 0.058 }),
    makeSegment({ kind: 'pipe', length: 0.55, dIn: 0.058 }),
  ];
}

/**
 * Complete engine presets, layout included — a V-twin is not just a different pipe.
 */
export interface EnginePreset {
  name: string;
  description: string;
  engine: Partial<EngineSpec>;
  pipe: () => PipeSegment[];
  collector?: () => PipeSegment[];
}

/**
 * Settings that belong to where the engine is being listened to, not to the engine, so loading a
 * preset keeps them.
 */
const PRESET_KEEPS = [
  'freeRunning',
  'micDistance',
  'micHeight',
  'exhaustHeight',
  'groundReflection',
  'airSpeed',
] as const satisfies readonly (keyof EngineSpec)[];

/**
 * The whole engine a preset loads: the preset over the defaults, with the listening setup carried
 * across from `current`.
 *
 * Over the defaults rather than over the current engine, because a preset only states what it changes
 * from them — which is also how its exhaust is sized, in `fullSpec`. Merged over the current engine,
 * every field a preset leaves out would come from whatever was loaded before it: a V-twin after a V8
 * would get the V8's bore, and anything after the overcammed V8 its cam.
 */
export function presetEngine(preset: EnginePreset, current: EngineSpec): EngineSpec {
  const spec: EngineSpec = { ...DEFAULT_ENGINE, ...preset.engine };
  for (const key of PRESET_KEEPS) (spec as unknown as Record<string, unknown>)[key] = current[key];
  return spec;
}

/** A preset's engine filled out with the defaults, for sizing its exhaust. */
function fullSpec(engine: Partial<EngineSpec>): EngineSpec {
  return { ...DEFAULT_ENGINE, ...engine };
}

/**
 * A four-valve head's valves for `bore`, as the presets with overhead cams use.
 *
 * Every engine here with overhead cams has a four-valve head, two intake and two exhaust valves, and is
 * given one. The sizes are a typical four-valve head's for the bore, each intake valve 0.40 of it and
 * each exhaust 0.34, rather than any one engine's published figures. Two of those open about 1.6 times
 * the area one valve of a two-valve head does for the same bore, and without them these engines choke
 * on their own exhaust well before their rev limits.
 *
 * The single, the V-twins and the pushrod V8s keep two-valve heads, as the engines they copy have.
 */
function fourValveHead(bore: number): Pick<EngineSpec, 'exValveDia' | 'exValveCount' | 'inValveDia' | 'inValveCount'> {
  return { exValveDia: 0.34 * bore, exValveCount: 2, inValveDia: 0.4 * bore, inValveCount: 2 };
}

/** Speed every engine preset idles at, rev/min. */
export const PRESET_IDLE_RPM = 800;

/**
 * A preset's operating point: idling in neutral at `PRESET_IDLE_RPM`, on `throttle`.
 *
 * The throttle is each preset's own, found by running it free with no load and adjusting the opening
 * until it settles at the idle speed. Most come out at 7-8%, because every throttle is sized to its
 * engine's airflow (`throttleDiaOf`). The engine starts at the idle speed too, so it does not have
 * to settle there from somewhere else.
 */
function idling(throttle: number): Pick<EngineSpec, 'rpm' | 'load' | 'throttle'> {
  return { rpm: PRESET_IDLE_RPM, load: 0, throttle };
}

/** Engines sized from their real counterparts, with exhausts fitted by `fittedExhaust`. */
const THREE_CYL: Partial<EngineSpec> = {
  cylinders: 3,
  vAngle: 0,
  exhaustLayout: 'merged',
  ...idling(0.075),
  // A Ford 1.0 EcoBoost's.
  revLimit: 6500,
  flywheelInertia: 0.2,
  pipeCellSize: 0.035,
  // A 1.0 litre three.
  bore: 0.072,
  stroke: 0.082,
  rodLength: 0.137,
  compressionRatio: 10,
  ...fourValveHead(0.072),
  maxLift: 0.0085,
  // Level-matched to the inline four, as the other presets are.
  outputGain: 2.89,
};

const FIVE_CYL: Partial<EngineSpec> = {
  cylinders: 5,
  vAngle: 0,
  exhaustLayout: 'merged',
  ...idling(0.075),
  // The Audi 2.5 TFSI's.
  revLimit: 7000,
  flywheelInertia: 0.3,
  pipeCellSize: 0.035,
  // A 2.5 litre five.
  bore: 0.0825,
  stroke: 0.0928,
  rodLength: 0.144,
  compressionRatio: 10,
  ...fourValveHead(0.0825),
  maxLift: 0.0095,
  // Level-matched to the inline four, as the other presets are.
  outputGain: 1.07,
};

const SIX_CYL: Partial<EngineSpec> = {
  cylinders: 6,
  vAngle: 0,
  exhaustLayout: 'merged',
  ...idling(0.075),
  // A BMW 3.0 straight six's.
  revLimit: 7000,
  flywheelInertia: 0.35,
  pipeCellSize: 0.035,
  // A 3.0 litre six.
  bore: 0.082,
  stroke: 0.0946,
  rodLength: 0.145,
  compressionRatio: 10.5,
  ...fourValveHead(0.082),
  maxLift: 0.0095,
  // Level-matched to the inline four, as the other presets are.
  outputGain: 1.32,
};

const V6_60: Partial<EngineSpec> = {
  cylinders: 6,
  vAngle: 60,
  exhaustLayout: 'perBank',
  ...idling(0.075),
  // The GM 3.6's.
  revLimit: 7000,
  flywheelInertia: 0.5,
  mouthSpacing: 1.0,
  pipeCellSize: 0.035,
  // A 3.6 litre 60-degree V6.
  bore: 0.094,
  stroke: 0.0856,
  rodLength: 0.1545,
  compressionRatio: 10.5,
  ...fourValveHead(0.094),
  maxLift: 0.01,
  // Level-matched to the inline four, as the other presets are.
  outputGain: 1.25,
};

/**
 * The two boxers: flat, opposed, a throw per cylinder. See `boxerPlan`.
 *
 * The four is sized as a 2.5 litre Subaru, both banks gathered into one pipe as its header does; the six
 * as a 3.6 litre Porsche, each bank's three into a silencer of its own, as a 911's are.
 */
const BOXER_FOUR: Partial<EngineSpec> = {
  cylinders: 4,
  vAngle: 180,
  crankType: 'boxer',
  exhaustLayout: 'merged',
  ...idling(0.075),
  // A Subaru EJ25's.
  revLimit: 6500,
  flywheelInertia: 0.3,
  pipeCellSize: 0.035,
  bore: 0.0995,
  stroke: 0.079,
  rodLength: 0.1305,
  compressionRatio: 10,
  ...fourValveHead(0.0995),
  maxLift: 0.0105,
  // Level-matched to the inline four, as the other presets are: RMS over two seconds, each at its own rpm.
  outputGain: 1.2,
};

const BOXER_SIX: Partial<EngineSpec> = {
  cylinders: 6,
  vAngle: 180,
  crankType: 'boxer',
  exhaustLayout: 'perBank',
  ...idling(0.076),
  // A 997 Carrera 3.6's.
  revLimit: 7300,
  flywheelInertia: 0.35,
  mouthSpacing: 0.9,
  pipeCellSize: 0.035,
  bore: 0.097,
  stroke: 0.0815,
  rodLength: 0.1275,
  compressionRatio: 11.3,
  ...fourValveHead(0.097),
  maxLift: 0.011,
  // Level-matched to the inline four, as the other presets are.
  outputGain: 0.97,
};

export const ENGINE_PRESETS: EnginePreset[] = [
  {
    name: 'Single, megaphone',
    description: 'The 500 cc thumper this project started as.',
    // A big air-cooled single is out of breath well before 7000.
    engine: { cylinders: 1, exhaustLayout: 'single', revLimit: 7000, ...idling(0.072) },
    pipe: () => PIPE_PRESETS[1]!.build(),
  },
  {
    name: '45\u00b0 V-twin, 2-into-1',
    description:
      'Shared crankpin, so it fires 405/315 — the uneven interval behind the classic lopsided idle. Both primaries merge into one collector.',
    engine: {
      cylinders: 2,
      vAngle: 45,
      firingOffset: null,
      exhaustLayout: '2into1',
      ...idling(0.073),
      // Long-stroke and pushrod: a Harley stops pulling not far past 5500.
      revLimit: 5600,
      flywheelInertia: 0.4,
      // Its own, independent of the default the single uses.
      outputGain: 0.72,
    },
    pipe: () => [
      makeSegment({ kind: 'pipe', length: 0.34, dIn: 0.042 }),
      makeSegment({ kind: 'cone', length: 0.1, dIn: 0.042, dOut: 0.05 }),
    ],
    collector: () => [
      makeSegment({ kind: 'cone', length: 0.12, dIn: 0.05, dOut: 0.06 }),
      makeSegment({ kind: 'pipe', length: 0.5, dIn: 0.06 }),
    ],
  },
  {
    name: '90\u00b0 V-twin, 2-into-2',
    description:
      'An L-twin firing 450/270, with a separate pipe per cylinder so the banks never talk to each other.',
    engine: {
      cylinders: 2,
      vAngle: 90,
      firingOffset: null,
      exhaustLayout: '2into2',
      ...idling(0.075),
      // Desmodromic valves, so no float to guard against: a Ducati twin's 9000.
      revLimit: 9000,
      mouthSpacing: 0.45,
      flywheelInertia: 0.22,
      outputGain: 0.89,
    },
    pipe: () => [
      makeSegment({ kind: 'pipe', length: 0.45, dIn: 0.04 }),
      makeSegment({ kind: 'cone', length: 0.3, dIn: 0.04, dOut: 0.075 }),
    ],
  },
  {
    name: 'Inline four',
    description:
      'Even 180\u00b0 firing on a flat crank, all four gathered by one manifold. Twice the firing frequency of a twin at the same rpm, and no half order at all.',
    engine: {
      cylinders: 4,
      vAngle: 0,
      exhaustLayout: 'merged',
      ...idling(0.079),
      // A 60 mm stroke is a bike engine's, and revs like one.
      revLimit: 10500,
      flywheelInertia: 0.16,
      pipeCellSize: 0.035,
      bore: 0.073,
      stroke: 0.06,
      rodLength: 0.11,
      ...fourValveHead(0.073),
      maxLift: 0.008,
      // A silenced system really is 10-15 dB quieter than an open pipe, which is correct and also
      // makes a preset sound thin next to one. Level-matched to the single instead.
      outputGain: 3.15,
    },
    pipe: () => [makeSegment({ kind: 'pipe', length: 0.42, dIn: 0.034 })],
    // A real exhaust system, not an open header: something over two metres of it, with a
    // silencer. This is where a road engine's body comes from — measured against a 0.57 m open
    // collector, this lifts the bottom two octaves by 15-19 dB.
    collector: () => [
      makeSegment({ kind: 'cone', length: 0.14, dIn: 0.05, dOut: 0.058 }),
      makeSegment({ kind: 'pipe', length: 1.0, dIn: 0.058, yaw: 0.25 }),
      makeSegment({ kind: 'chamber', length: 0.38, dIn: 0.058, dOut: 0.16 }),
      makeSegment({ kind: 'pipe', length: 0.45, dIn: 0.052 }),
    ],
  },
  {
    name: 'Inline three',
    description:
      'Fires every 240\u00b0 on a 120\u00b0 crank, in the order 1-3-2. An odd number of cylinders puts the loudest order at one and a half times the crank speed, which is the offbeat thrum of a three.',
    engine: THREE_CYL,
    pipe: () => fittedExhaust(fullSpec(THREE_CYL)).pipe,
    collector: () => fittedExhaust(fullSpec(THREE_CYL)).collector,
  },
  {
    name: 'Inline five',
    description:
      'Every 144\u00b0 on a 72\u00b0 crank, 1-2-4-5-3 \u2014 the Audi and Volvo five. Two and a half firings per revolution, which is what gives it a warble no even-numbered engine has.',
    engine: FIVE_CYL,
    pipe: () => fittedExhaust(fullSpec(FIVE_CYL)).pipe,
    collector: () => fittedExhaust(fullSpec(FIVE_CYL)).collector,
  },
  {
    name: 'Inline six',
    description:
      'Every 120\u00b0, 1-5-3-6-2-4, its throws paired 1-6, 2-5 and 3-4. Perfectly balanced and evenly fired, so the smooth, silky one.',
    engine: SIX_CYL,
    pipe: () => fittedExhaust(fullSpec(SIX_CYL)).pipe,
    collector: () => fittedExhaust(fullSpec(SIX_CYL)).collector,
  },
  {
    name: 'V6, 60\u00b0, manifold per bank',
    description:
      'A 60\u00b0 V6 on a split-pin crank, which is what lets it fire evenly every 120\u00b0 despite a vee too narrow for that on shared pins. Each bank hears every other firing, 240\u00b0 apart.',
    engine: V6_60,
    pipe: () => fittedExhaust(fullSpec(V6_60)).pipe,
    collector: () => fittedExhaust(fullSpec(V6_60)).collector,
  },
  {
    name: 'V8, crossplane, manifold per bank',
    description:
      'The American V8. Fires every 90\u00b0 overall, but the crossplane crank deals those firings out unevenly between the banks — 180-90-180-270 down each side — and with a collector per bank that uneven arrival pattern is the burble.',
    engine: {
      cylinders: 8,
      vAngle: 90,
      crankType: 'crossplane',
      exhaustLayout: 'perBank',
      ...idling(0.075),
      // Pushrods and a heavy crank: a road V8's 6500.
      revLimit: 6500,
      mouthSpacing: 1.3,
      flywheelInertia: 0.9,
      // The same cells as everything else. At one step per sample, cost goes only as the cell
      // count, so 35 mm costs a V8 a couple of points of a core and gives it its top octave.
      pipeCellSize: 0.035,
      bore: 0.102,
      stroke: 0.084,
      rodLength: 0.145,
      compressionRatio: 10,
      // A 102 mm bore carries far bigger valves than the default 500 cc single.
      exValveDia: 0.041,
      inValveDia: 0.048,
      maxLift: 0.011,
      outputGain: 2.05,
    },
    pipe: () => [makeSegment({ kind: 'pipe', length: 0.5, dIn: 0.044 })],
    // Bank pipe, silencer, tailpipe — the length is most of why a road V8 sounds deep.
    collector: () => [
      makeSegment({ kind: 'cone', length: 0.16, dIn: 0.062, dOut: 0.072 }),
      makeSegment({ kind: 'pipe', length: 1.15, dIn: 0.072, yaw: 0.18 }),
      makeSegment({ kind: 'chamber', length: 0.45, dIn: 0.072, dOut: 0.2 }),
      makeSegment({ kind: 'pipe', length: 0.55, dIn: 0.064 }),
    ],
  },
  {
    name: 'V8, overcammed',
    description:
      'A small-block with far more cam than the street wants: 300\u00b0 of duration on a tight lobe separation, so both valves hang open together for 90\u00b0 around top dead centre. At idle, exhaust is pushed back up the intake and breathed in again, so the charge is mostly spent gas and the manifold has almost no vacuum. About one cycle in four fails to light, and the rest burn late and unevenly: that is the lope.',
    engine: {
      cylinders: 8,
      vAngle: 90,
      crankType: 'crossplane',
      exhaustLayout: 'perBank',
      // Idle is where a big cam is heard. The overlap costs little at wide-open throttle; nearly
      // shut, the manifold is the lowest pressure the exhaust can reach, so it back-flows into the
      // intake. Measured at 800 rpm on this throttle, against the stock crossplane at the same: 0.80
      // bar in the manifold rather than 0.31, and 64% of the trapped charge spent gas rather than
      // 24%. That is past the dilution limit (`DILUTION_ONSET` in cylinder.ts), and a quarter of
      // cycles misfire where the stock engine misfires none. On 6% it is half, an engine about to
      // stall rather than one with a lope; on 12%, almost none. It needs more air to idle than the
      // stock engine's 7.5%: on this throttle the stock one runs up to 950.
      ...idling(0.083),
      // Built to rev, and needs to: a cam this size only starts to pull past 4000.
      revLimit: 7000,
      mouthSpacing: 1.3,
      flywheelInertia: 0.7,
      pipeCellSize: 0.035,
      // A 350: 4.030 x 3.48 in on a 5.7 in rod, with 2.02/1.60 in valves.
      bore: 0.1024,
      stroke: 0.0884,
      rodLength: 0.1448,
      compressionRatio: 11,
      exValveDia: 0.0406,
      inValveDia: 0.0513,
      maxLift: 0.015,
      // 305 and 295 degrees, 45 degrees either side of overlap, on a 105-degree lobe separation.
      evo: 100,
      evc: 405,
      ivo: 315,
      ivc: 610,
      // An idle this dilute burns slowly and wants the spark early to make up for it.
      ignition: 690,
      // The crossplane's gain rather than level-matched at idle, so the two V8s compare directly and
      // opening this one up does not clip.
      outputGain: 2.05,
    },
    pipe: () => [makeSegment({ kind: 'pipe', length: 0.8, dIn: 0.044 })],
    // Long-tube headers into short collectors and a glasspack-sized can: loud, not open.
    collector: () => [
      makeSegment({ kind: 'cone', length: 0.16, dIn: 0.062, dOut: 0.076 }),
      makeSegment({ kind: 'pipe', length: 0.9, dIn: 0.076, yaw: 0.18 }),
      makeSegment({ kind: 'chamber', length: 0.4, dIn: 0.076, dOut: 0.13 }),
      makeSegment({ kind: 'pipe', length: 0.35, dIn: 0.07 }),
    ],
  },
  {
    name: 'V8, flatplane, manifold per bank',
    description:
      'The same engine on a flat crank, so each bank fires evenly every 180\u00b0 and its collector hears four equally spaced pulses. Same firing intervals overall as the crossplane, completely different voice — this is the Ferrari.',
    engine: {
      cylinders: 8,
      vAngle: 90,
      crankType: 'flatplane',
      exhaustLayout: 'perBank',
      // Deliberately the loud one: a flat-crank V8 on short pipes.
      ...idling(0.078),
      // Oversquare, light and flat-cranked, so it revs like the Ferrari it is.
      revLimit: 9000,
      mouthSpacing: 1.3,
      flywheelInertia: 0.5,
      pipeCellSize: 0.035,
      bore: 0.094,
      stroke: 0.067,
      rodLength: 0.132,
      compressionRatio: 12,
      ...fourValveHead(0.094),
      maxLift: 0.0105,
      outputGain: 0.94,
    },
    pipe: () => [makeSegment({ kind: 'pipe', length: 0.44, dIn: 0.042 })],
    collector: () => [
      makeSegment({ kind: 'cone', length: 0.14, dIn: 0.058, dOut: 0.07 }),
      makeSegment({ kind: 'pipe', length: 0.85, dIn: 0.07, yaw: 0.15 }),
      makeSegment({ kind: 'cone', length: 0.25, dIn: 0.07, dOut: 0.09 }),
    ],
  },
  {
    name: 'Boxer four',
    description:
      'Flat, with the pistons of each opposed pair moving out and in together, firing 1-3-2-4 every 180\u00b0 \u2014 the Subaru and the air-cooled VW. Both banks gather into one pipe, so each side\u2019s two runners arrive at different distances from the merge.',
    engine: BOXER_FOUR,
    pipe: () => fittedExhaust(fullSpec(BOXER_FOUR)).pipe,
    collector: () => fittedExhaust(fullSpec(BOXER_FOUR)).collector,
  },
  {
    name: 'Boxer six',
    description:
      'The Porsche flat six: a throw per cylinder, firing 1-6-2-4-3-5 every 120\u00b0. Each bank of three has its own silencer and hears every other firing, 240\u00b0 apart, as a V6\u2019s banks do.',
    engine: BOXER_SIX,
    pipe: () => fittedExhaust(fullSpec(BOXER_SIX)).pipe,
    collector: () => fittedExhaust(fullSpec(BOXER_SIX)).collector,
  },
  {
    name: 'Parallel twin, 360\u00b0',
    description:
      'Both pistons rise together, so it fires evenly every 360\u00b0 and has no half-order thump at all.',
    engine: {
      cylinders: 2,
      vAngle: 0,
      firingOffset: 360,
      ...fourValveHead(DEFAULT_ENGINE.bore),
      exhaustLayout: '2into1',
      ...idling(0.074),
      // A modern 1200 cc parallel twin's.
      revLimit: 7500,
      outputGain: 0.64,
    },
    pipe: () => [makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.04 })],
    collector: () => [
      makeSegment({ kind: 'pipe', length: 0.25, dIn: 0.055 }),
      makeSegment({ kind: 'chamber', length: 0.3, dIn: 0.055, dOut: 0.13 }),
      makeSegment({ kind: 'pipe', length: 0.2, dIn: 0.05 }),
    ],
  },
];

export function defaultConfig(): EngineConfig {
  return {
    engine: { ...DEFAULT_ENGINE },
    pipe: PIPE_PRESETS[1]!.build(),
    collector: defaultCollector(),
  };
}
