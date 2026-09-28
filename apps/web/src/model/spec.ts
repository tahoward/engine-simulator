/**
 * The data model: the engine and exhaust the interface edits and sends to the simulation, the
 * snapshot it gets back, the presets, and the geometry the scene draws. The simulation's own copy of
 * these types, `crates/engine-sim/src/spec.rs`, reads and writes the same JSON, field for field.
 * Free of any DOM or AudioContext reference, so the tests run in Node.
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
import nissanRb26Exhaust from './exhausts/nissan-rb26.json';
import toyota2grExhaust from './exhausts/toyota-2gr.json';

/** Shape of one length of exhaust plumbing. */
export type SegmentKind =
  /**
   * Tube, tapering in a straight line from `dIn` to `dOut` where they differ: a header, a megaphone, a
   * reverse cone, or a pipe the same all the way along.
   */
  | 'pipe'
  /** The same as a tapering `pipe`, under the name some presets and saved exhausts give one. */
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
   * Outlet diameter, metres. For a chamber, the body's diameter if it is round and its width otherwise.
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
  /**
   * A smooth bend instead of a straight run: where the segment ends, m, and the way it is heading there,
   * both in the frame of the way it starts (x along it, y as near up as that allows, z across; see
   * `curveFrame`). `length` is the length along the bend. Routing only, like a corner: the solver hears a
   * pipe of that length.
   */
  curve?: SegmentCurve;
}

export interface SegmentCurve {
  end: [number, number, number];
  dir: [number, number, number];
  /**
   * How long the cubic's handles are, as a share of its chord. Unset, 0.4, which suits a fitted bend of any
   * shape, an S included; a drawn bend is one arc, and sets the share that makes the cubic that arc.
   */
  handle?: number;
}

/** The handle share that makes a cubic follow a circular arc turning through `angle` radians. */
export function arcHandle(angle: number): number {
  const half = Math.sin(angle / 2);
  return half > 1e-9 ? ((4 / 3) * Math.tan(angle / 4)) / (2 * half) : 0.4;
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

/** Which way a header's primaries run to their merge: see `EngineSpec.headerRun`. */
export type HeaderRun = 'outward' | 'lengthways';

export type ExhaustLayout = 'open' | 'perBank' | 'merged';

/**
 * Accepted layout values, including the names a single or a twin is described by.
 *
 * `single` and `2into2` both mean one pipe per cylinder; `2into1` means one shared collector.
 * The twin presets use them, and so may a saved link; `exhaustLayoutOf` normalises them.
 */
export type ExhaustLayoutSpec = ExhaustLayout | 'single' | '2into2' | '2into1';

/**
 * Where a turbocharged engine's blow-off valve vents when the throttle shuts on boost.
 *
 * `atmospheric` vents to the air, with the hiss everyone knows; `recirculating` back into the
 * compressor inlet, as most factory valves do, much more quietly. `none` has no valve: the air trapped
 * between the compressor and the shut throttle has nowhere to go, and the compressor surges.
 */
export type BlowOff = 'atmospheric' | 'recirculating' | 'none';

export const BLOW_OFFS: BlowOff[] = ['atmospheric', 'recirculating', 'none'];

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
  /**
   * Equal-length headers: each cylinder's own primary, `pipe`, all the way to one merge per collector,
   * instead of a manifold along the ports that each cylinder joins through a short stub.
   *
   * A header's primaries are long enough to tune. The pressure wave each exhaust pulse sends back from
   * the merge arrives at the valve as a suction during the overlap, when both valves are open, and pulls
   * fresh charge through the cylinder after the exhaust. Tuned for the speed a high-output engine makes its
   * power at, that is several points of volumetric efficiency a manifold does not give.
   */
  exhaustHeaders: boolean;
  /**
   * Where a header's primaries run to their merge, as drawn.
   *
   * - `outward` — straight out from the middle of the ports, the way they point.
   * - `lengthways` — along the engine, the way the crank runs, to a merge behind the rearmost port.
   *
   * Only where the pipes go, not how long they are, so it makes no difference to the sound.
   */
  headerRun: HeaderRun;

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
  /**
   * Cam profile switching, as Honda's VTEC does it: the engine speed at which each valve switches to a
   * second, high-speed cam lobe, rev/min, or 0 for a single profile. Below it the valves follow the
   * timing and lift above; from it, the `high*` ones. It switches back `CAM_SWITCH_HYSTERESIS` lower.
   *
   * Unlike variable valve timing, which turns the whole cam and so moves a valve's opening and closing
   * together, a second lobe changes how long the valve is open and how far: a mild lobe for low speed
   * and a wild one, with more lift and duration, for the top end.
   */
  camSwitchRpm: number;
  /** The high-speed cam's events, deg ATDC, and its peak lift, m. See `camSwitchRpm`. */
  highEvo: number;
  highEvc: number;
  highIvo: number;
  highIvc: number;
  highMaxLift: number;
  /**
   * Variable valve timing: the ECU's cam map, crank degrees from the timing above, which is the cams'
   * rest position. How far the intake cam is advanced under load at low speed and at high speed, and
   * how far the exhaust cam is retarded. All zero is a fixed cam. See `update_phasers` in `crates/engine-sim/src/engine_sim.rs` for how
   * they are blended with speed and load.
   */
  vvtIntakeLow: number;
  vvtIntakeHigh: number;
  vvtExhaustLow: number;
  vvtExhaustHigh: number;
  /**
   * The engine speeds the map's low-speed and high-speed settings hold at, rev/min. Up to the first the
   * cams take the low-speed settings, from the second the high-speed ones, and between the two a straight
   * line from one to the other.
   */
  vvtLowRpm: number;
  vvtHighRpm: number;
  /**
   * One phaser for both cams, as a pushrod engine's single camshaft has: the whole cam moves by the
   * intake's advance, intake and exhaust lobes together, and the exhaust's map is ignored.
   */
  vvtLinked: boolean;

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
   * somewhat longer the faster the engine turns. See `burn_angle` in `crates/engine-sim/src/cylinder.rs`.
   */
  burnDuration: number;
  /**
   * Air-fuel equivalence ratio λ: the air-fuel ratio as a multiple of stoichiometric. 1 is
   * stoichiometric, below 1 rich, above 1 lean.
   *
   * Each cylinder's port injector meters fuel in proportion to the air its runner draws, so this is
   * the mixture every cylinder traps. Lean, every kilogram of charge carries less fuel and burns
   * slower; rich, the extra fuel has no oxygen to burn with and goes out unburned.
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
  /**
   * Length of each intake runner, from the valve seat to the plenum, m: the port and the manifold
   * runner together. 0 or less sizes it for the engine; see `intakeRunnerOf`.
   *
   * The runner is a column of air with momentum and pressure waves in it, solved by the same gas
   * dynamics as the exhaust. Drawn in by the falling piston, it keeps ramming charge into the cylinder
   * after bottom dead centre, and its pressure waves arrive back at the valve in step with it at one
   * speed and out of step at another. That is where an engine's torque peak comes from, and why a long
   * runner makes low-rpm torque and a short one high-rpm power.
   */
  intakeRunnerLength: number;
  /** Bore of each intake runner, m. 0 or less sizes it from the intake valves; see `intakeRunnerOf`. */
  intakeRunnerDia: number;
  /**
   * A two-stage intake: the length of each runner's short path, m, which the manifold switches to at
   * `intakeSwitchRpm` and above. 0 or less is a single-stage intake, with the one runner length.
   *
   * A long runner's tuning makes torque low down and a short one's power at the top, and one length is
   * a compromise between the two. A two-stage manifold has both: below the switch speed each cylinder
   * breathes through the long runner, `intakeRunnerLength`, and above it a flap opens a shorter path.
   */
  intakeRunnerShortLength: number;
  /**
   * Engine speed the two-stage intake switches to its short runners at, rev/min. It switches back 150
   * rev/min lower, so it does not flap back and forth at the switch speed.
   */
  intakeSwitchRpm: number;

  // --- Turbocharger ---
  //
  // The turbos themselves are placed in the exhaust, as `ExhaustGraph.turbos`, and pipes attached to them.
  // Nothing is taken from a map: each turbine is driven by the exhaust the cylinders push out, so the boost
  // builds with the exhaust flow, and lags behind the throttle while the shaft spins up. These settings are
  // every turbo's. See `crates/engine-sim/src/turbo.rs`.
  /**
   * Boost the wastegate holds, gauge, Pa. It opens a bypass around the turbine as the boost reaches
   * this, so the turbine takes less of the exhaust.
   */
  boostTarget: number;
  /**
   * Size of each turbo: its compressor's flow at full speed, kg/s. 0 or less sizes it for the engine.
   *
   * A small turbo spools early and runs out of breath at the top end, as its compressor nears choke
   * and its turbine chokes the exhaust; a big one lags, and holds its boost to the limiter.
   */
  turboSize: number;
  /** Intercooler effectiveness, 0..1: the share of the compressor's heating it takes back out. */
  intercooler: number;
  /** The blow-off valve. See `BlowOff`. */
  blowOff: BlowOff;
  /**
   * Level of what the turbo itself sounds like, 0..1 (1 = realistic): the whine of the compressor, the
   * blow-off valve, compressor surge and the wastegate's rattle.
   */
  turboNoise: number;

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
   * run to its limiter in a few hundredths of a second. See `load_torque_of` in `crates/engine-sim/src/spec.rs`.
   */
  load: number;

  // --- Acoustics / output ---
  /** Port gas temperature, K. Sets the speed of sound at the head of the pipe. */
  portGasTemp: number;
  /**
   * Cell length for the exhaust gas-dynamics solver, m.
   *
   * The trade between fidelity and CPU. Smaller cells resolve higher frequencies — roughly
   * `c / (10 * pipeCellSize)` before numerical dissipation takes over — and cost more cells. The solver
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
  /** How far the phasers have moved the cams from rest, crank degrees: intake advance, exhaust retard. */
  intakeCamAdvance: number;
  exhaustCamRetard: number;
  /** Whether a two-stage intake is on its short runners right now. */
  shortRunners: boolean;
  /** Whether cam profile switching has the valves on the high-speed lobes right now. */
  highCam: boolean;
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
  /**
   * Gauge pressure in every cell of every exhaust duct, Pa: the ducts in the graph's order, each from
   * its port end, taking `ductCells` values in turn.
   */
  ductPressure: Float32Array;
  /** How many cells of `ductPressure` each duct has. */
  ductCells: number[];
  /**
   * Each duct's id, in the same order. The solver's graph is not quite the one drawn: it leaves out
   * loose pipes and adds a turbo's exit, so the ids say which pipe is which.
   */
  ductIds: string[];
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
  /** The launch in progress, or `null` when none is. */
  launch: LaunchSnapshot | null;
  /** The turbocharger's state, on a turbocharged engine only. */
  turbo?: TurboSnapshot;
}

/** A turbocharger's state, sent with each snapshot. */
export interface TurboSnapshot {
  /** Charge-air pressure, gauge, Pa. */
  boost: number;
  /** Plenum pressure, gauge, Pa: below zero is vacuum. */
  manifold: number;
  /** Pressure at the turbine inlet, gauge, Pa: what the exhaust works against. */
  turbineInlet: number;
  /** Each turbo's shaft speed, rev/min. */
  shaftRpm: number;
  /** Wastegate and blow-off valve openings, 0..1. */
  wastegate: number;
  blowOff: number;
  /** Whether the compressor is surging. */
  surging: boolean;
}

/** A launch's state, sent with each snapshot while it runs. */
export interface LaunchSnapshot {
  phase: 'launch' | 'pull' | 'shiftOut' | 'shiftIn' | 'cooldown';
  /** Gear engaged, 1-based. */
  gear: number;
  /** Road speed, km/h. */
  speedKmh: number;
  /** Seconds since the clock started, once the car had rolled a foot. */
  elapsed: number;
  /** Distance covered, m. */
  distance: number;
  /** Whether the last pull is over, or the run was stopped. The engine may still be winding down. */
  finished: boolean;
  /** Seconds from the clock starting to 60 mph, once the car has reached it. */
  zeroToSixty: number | null;
  /** Seconds to the quarter mile and the speed there, km/h, once the car has covered it. */
  quarterMile: number | null;
  quarterMileKmh: number | null;
  /** The same at the half mile. */
  halfMile: number | null;
  halfMileKmh: number | null;
  /**
   * The engine cycles recorded since the last snapshot, six values each: rpm, crank torque (N*m),
   * road speed (km/h), gear (1-based), volumetric efficiency (a fraction) and intake manifold pressure
   * (bar, absolute).
   */
  points: Float32Array;
}

/**
 * The car and gearbox a launch drives through. See `LaunchRun` in `crates/engine-sim/src/drivetrain.rs`.
 */
export interface LaunchConfig {
  /** Gearbox ratios, first gear first. */
  ratios: number[];
  /** Final drive ratio. */
  finalDrive: number;
  /** Tyre rolling radius, m. */
  tyreRadius: number;
  /** The tyres' friction coefficient at their peak: see `TYRE_GRIP`. */
  tyreGrip: number;
  /** The car's mass, kg. */
  mass: number;
  /** Share of the car's weight on the driven wheels at rest: see `DRIVEN_LOAD`. */
  drivenLoad: number;
  /** Whether traction control eases the throttle to stop the driven wheels spinning. */
  tractionControl: boolean;
  /** Engine speed the clutch is slipped at off the line, rev/min. */
  launchRpm: number;
  /** Engine speed each gear is pulled to before the shift, rev/min. In top gear the run ends there. */
  shiftRpm: number;
  /** How long each shift takes, from lifting off to full throttle in the next gear, s. */
  shiftTime: number;
  /** Whether the gearbox is a dual clutch, which shifts with no gap in the drive. */
  dualClutch: boolean;
}

/**
 * Share of a car's weight on its driven wheels at rest, by the wheels it drives: the rear wheels of a
 * front-engined car carry about half, and all four carry it all. Driving the rear wheels, the launch
 * moves more onto them as the car accelerates.
 */
export const DRIVEN_LOAD = { rwd: 0.5, awd: 1 } as const;

/**
 * How much harder a tyre grips driving in a straight line than cornering, where a skidpad measures it:
 * a tyre's peak grip is typically 5-10% higher along its rolling direction than across it.
 */
export const SKIDPAD_TO_DRIVE = 1.07;

/**
 * Tyres' friction coefficients at their peak, driving: a road tyre's, and the Corvettes', from the 1.22 g
 * the Z06 with the Z07 package pulls on a skidpad on its Pilot Sport Cup 2 R tyres, which the Stingray is
 * given too.
 */
export const TYRE_GRIP = { road: 1.1, corvette: 1.22 * SKIDPAD_TO_DRIVE } as const;

/** A quick shift of a manual gearbox, s: lift, clutch, shift and back on the throttle. */
export const MANUAL_SHIFT_TIME = 0.4;

/** A close-ratio six-speed's gears, first to sixth. */
export const LAUNCH_RATIOS = [3.36, 2.09, 1.47, 1.1, 0.87, 0.71];

/** The fewest and most gears a launch's gearbox may have: the most is what the sheet has colours for. */
export const MIN_GEARS = 1;
export const MAX_GEARS = 8;

/** How far below the rev limiter a launch shifts, rev/min, so the pull never touches the cut. */
const LAUNCH_SHIFT_MARGIN = 150;

/** The launch speed's least margin under the shift point, rev/min. */
export const LAUNCH_RPM_MARGIN = 500;

/**
 * A car and gearing to suit `spec`, for a launch through a gearbox of `ratios`.
 *
 * Sized from a rough peak power: the nominal full-throttle torque at 80% of the rev limit. The car
 * weighs about 9 kg per kW of that, as a quick road car does, held between a light motorcycle and a
 * heavy saloon. Top gear is geared so that the shift point comes at the speed the car could reach on
 * the road, where that power meets its air drag. So the gears come out right for the engine: a 500 cc
 * single tops out near 160 km/h and a 5.5 litre V8 past 300. The clutch is slipped off the line at half
 * the rev limit.
 */
export function fitLaunch(spec: EngineSpec, boosted = false, ratios: number[] = LAUNCH_RATIOS): LaunchConfig {
  const shiftRpm = Math.max(spec.revLimit - LAUNCH_SHIFT_MARGIN, 1000);
  const power = fullLoadTorque(spec, boosted) * ((0.8 * spec.revLimit * 2 * Math.PI) / 60);
  const mass = Math.min(Math.max(power / 110, 180), 1900);
  // Top speed on the road: power against drag, 1/2 rho CdA v^3, for the CdA of 0.6 m^2 the launch
  // runs against (`DRAG_AREA` in drivetrain.rs).
  const topSpeed = Math.min(Math.max(Math.cbrt((2 * power) / (1.2 * 0.6)), 45), 90);
  const tyreRadius = 0.31;
  const top = ratios[ratios.length - 1]!;
  const finalDrive = ((shiftRpm * 2 * Math.PI) / 60) * tyreRadius / (top * topSpeed);
  const launchRpm = Math.max(Math.min(0.5 * spec.revLimit, shiftRpm - LAUNCH_RPM_MARGIN), 1000);
  return {
    ratios: [...ratios],
    finalDrive,
    tyreRadius,
    tyreGrip: TYRE_GRIP.road,
    mass,
    drivenLoad: DRIVEN_LOAD.rwd,
    tractionControl: true,
    launchRpm,
    shiftRpm,
    shiftTime: MANUAL_SHIFT_TIME,
    dualClutch: false,
  };
}

/**
 * Brake mean effective pressure that `load = 1` stands for, Pa.
 *
 * Eleven bar is a naturally aspirated petrol engine's full-load figure, give or take a couple. It fixes
 * the load's scale by displacement alone, which is what makes one setting mean the same on a single and
 * a V8; the engine's own torque curve would be more exact, but is only known by running it.
 */
export const FULL_LOAD_BMEP = 11e5;

// ---------------------------------------------------------------------------
// Gas properties
// ---------------------------------------------------------------------------

export const GAS = {
  /** Specific gas constant for air / exhaust, J/(kg*K). */
  R: 287,
  /**
   * Ratio of specific heats of cylinder gas at compression temperatures, around 500 K, where the
   * cylinder's gamma is needed as a single number. The simulation's cylinder follows a cv that rises
   * with temperature.
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

/** Speed of sound in exhaust gas at temperature `t` (K), m/s. */
export function speedOfSound(t: number, gamma: number = GAS.gammaExh): number {
  return Math.sqrt(gamma * GAS.R * t);
}

// ---------------------------------------------------------------------------
// Derived geometry helpers — the same as the simulation's (`crates/engine-sim/src/spec.rs`), so the
// piston you see is at the position the gas law is using.
// ---------------------------------------------------------------------------

/** Swept (displacement) volume, m^3. */
export function displacement(spec: EngineSpec): number {
  return (Math.PI * spec.bore * spec.bore) / 4 * spec.stroke;
}

/**
 * Nominal full-throttle torque of the whole engine, N*m: `FULL_LOAD_BMEP` over its displacement, and
 * `boosted`, with a turbo in its exhaust, in proportion to the charge pressure the wastegate holds.
 */
export function fullLoadTorque(spec: EngineSpec, boosted = false): number {
  const torque = (FULL_LOAD_BMEP * displacement(spec) * spec.cylinders) / (4 * Math.PI);
  return boosted ? (torque * (GAS.pAmb + spec.boostTarget)) / GAS.pAmb : torque;
}

/**
 * Diameter of the exhaust port, m: one duct that the exhaust valves share, of the same area as
 * their heads together. Every exhaust starts from it.
 */
export function exhaustPortDiameter(spec: EngineSpec): number {
  return spec.exValveDia * Math.sqrt(spec.exValveCount);
}

/**
 * The intake runner `spec` has, length and bore, m: its own where it states them, and otherwise sized
 * for the engine.
 *
 * The bore passes the intake valves' area, a little narrowed, as a port does. The length is tuned: its
 * quarter-wave resonance falls at `RUNNER_TUNE_ORDER` times the crank speed at three quarters of the
 * rev limit, which is where a road engine puts its torque peak. That rule puts the runner of a V8
 * limited at 6600 rpm at 450 mm and of a four limited at 10,500 at 280, and fills each to about 100%
 * there: measured across multiples of 1.6 to 3.5, 2.3 is the best on a 6.2 litre V8 and within a
 * point or two of it on a four and a V6.
 */
export function intakeRunnerOf(spec: EngineSpec): { length: number; diameter: number } {
  const diameter =
    spec.intakeRunnerDia > 0 ? spec.intakeRunnerDia : 0.9 * spec.inValveDia * Math.sqrt(spec.inValveCount);
  if (spec.intakeRunnerLength > 0) return { length: spec.intakeRunnerLength, diameter };
  const tunedHz = RUNNER_TUNE_ORDER * ((0.75 * spec.revLimit) / 60);
  return { length: speedOfSound(GAS.tAmb, GAS.gammaAir) / (4 * tunedHz), diameter };
}

/**
 * Multiple of the crank speed an auto-sized runner's quarter-wave resonance is tuned to. See
 * `intakeRunnerOf`.
 */
const RUNNER_TUNE_ORDER = 2.3;

/**
 * The engine speed `spec`'s intake runners are tuned for, rev/min: where their quarter-wave resonance
 * is `RUNNER_TUNE_ORDER` times the crank speed, the rule an auto-sized runner is cut to.
 */
export function runnerTunedRpm(spec: EngineSpec): number {
  const quarterWaveHz = speedOfSound(GAS.tAmb, GAS.gammaAir) / (4 * intakeRunnerOf(spec).length);
  return (quarterWaveHz / RUNNER_TUNE_ORDER) * 60;
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
  const a = spec.stroke / 2;
  const th = (deg * Math.PI) / 180;
  const sin = Math.sin(th);
  return a * Math.cos(th) + Math.sqrt(Math.max(spec.rodLength * spec.rodLength - a * a * sin * sin, 0));
}

// ---------------------------------------------------------------------------
// Pipe helpers
// ---------------------------------------------------------------------------

/**
 * Diameter, m, at normalised position `u` (0..1) within a segment.
 *
 * For a non-round chamber body this is the diameter of the circle with the same area, which is
 * all the 1D acoustics needs. `segmentSection` gives the real shape.
 */
export function segmentDiameter(seg: PipeSegment, u: number): number {
  switch (seg.kind) {
    case 'pipe':
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
    dOut: partial.dOut ?? dIn,
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
  // Checked for the same reason: a bend from a link must be three numbers each way, the way a unit.
  const c = partial.curve as (SegmentCurve & { handle?: unknown }) | undefined;
  const triple = (v: unknown): v is [number, number, number] =>
    Array.isArray(v) && v.length === 3 && v.every((n) => typeof n === 'number' && Number.isFinite(n));
  if (kind === 'pipe' && c && triple(c.end) && triple(c.dir) && Math.hypot(...c.dir) > 1e-9) {
    const n = Math.hypot(...c.dir);
    seg.curve = {
      end: [...c.end],
      dir: [c.dir[0] / n, c.dir[1] / n, c.dir[2] / n],
      ...(typeof c.handle === 'number' && c.handle > 0 && c.handle < 2 ? { handle: c.handle } : {}),
    };
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
  exhaustHeaders: false,
  headerRun: 'outward',
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
  camSwitchRpm: 0,
  highEvo: 128,
  highEvc: 378,
  highIvo: 342,
  highIvc: 576,
  highMaxLift: 0.0095,
  vvtIntakeLow: 0,
  vvtIntakeHigh: 0,
  vvtExhaustLow: 0,
  vvtExhaustHigh: 0,
  vvtLowRpm: 2000,
  vvtHighRpm: 6000,
  vvtLinked: false,

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
  intakeRunnerLength: 0,
  intakeRunnerDia: 0,
  intakeRunnerShortLength: 0,
  intakeSwitchRpm: 5000,

  boostTarget: 0.7e5,
  turboSize: 0,
  intercooler: 0.7,
  blowOff: 'atmospheric',
  turboNoise: 1,

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
 * Both are indexed by throw from the front, the two cylinders of a throw side by side, first bank
 * then second. Read off real engines: crossplane is the Ford 302 order 1-5-4-2-6-3-7-8, cylinders
 * 1-4 on the first bank, giving pins at 0/270/90/180 along the crank — the four-plane crank, its end
 * throws half a turn apart so the secondary couple cancels. Flatplane puts every throw in one plane,
 * so its pins are only ever 0 or 180, and in the order an inline four's are, 0/180/180/0 along the
 * crank, which is what cancels its primary couple. Each bank then fires 1-3-4-2 by throw, as an
 * inline four does, and each second-bank cylinder fires 90 degrees after its pin partner: the
 * Ferrari order 1-5-3-7-4-8-2-6.
 */
interface V8Crank {
  pins: number[];
  revs: number[];
  banks: number[];
}

const V8_CROSSPLANE: V8Crank = {
  pins: [0, 0, 270, 270, 90, 90, 180, 180],
  revs: [0, 0, 0, 0, 1, 1, 0, 1],
  banks: [0, 1, 0, 1, 0, 1, 0, 1],
};

const V8_FLATPLANE: V8Crank = {
  pins: [0, 0, 180, 180, 180, 180, 0, 0],
  revs: [0, 0, 1, 1, 0, 0, 1, 1],
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

/**
 * How far apart along the crank the cylinders sharing a throw sit, m: a rod's width, so their rods run side
 * by side on a shared pin, or each on its own pin of a split one. It is why one bank of a V sits a little
 * ahead of the other.
 */
export const ROD_STAGGER = 0.016;

/**
 * Where cylinder `cylinder` sits along the crank, m, the engine centred on the origin: at its throw, and on
 * a throw it shares, staggered from the other cylinders on it by `ROD_STAGGER`.
 */
export function cylinderZ(spec: EngineSpec, cylinder: number): number {
  const pins = crankPins(spec);
  const index = Math.max(pins.findIndex((p) => p.cylinders.includes(cylinder)), 0);
  const pin = pins[index];
  const along = (index - (pins.length - 1) / 2) * cylinderSpacing(spec);
  if (!pin || pin.cylinders.length < 2) return along;
  const k = pin.cylinders.indexOf(cylinder);
  return along + (k - (pin.cylinders.length - 1) / 2) * ROD_STAGGER;
}

/**
 * How long the engine is along its crank, m: from its first cylinder to its last, and a cylinder's pitch
 * over, half at each end, which is the length of the block.
 */
export function engineLength(spec: EngineSpec): number {
  const zs = Array.from({ length: spec.cylinders }, (_, c) => cylinderZ(spec, c));
  return Math.max(...zs) - Math.min(...zs) + cylinderSpacing(spec);
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

/**
 * Which gaps between neighbouring throws of `crankPins` carry a main bearing: entry `i` is the gap after
 * throw `i`. There is always one at each end of the crank as well.
 *
 * A main between every throw, as a fully counterweighted inline or V crank has: five for an inline four,
 * seven for a six, five for a V8. A flat engine's opposed pair sit on neighbouring pins half a turn apart,
 * joined by a web, with the mains between the pairs: three for a flat four, four for a six.
 */
export function mainBearingsAfter(spec: EngineSpec): boolean[] {
  const throws = crankPins(spec).length;
  return Array.from({ length: Math.max(throws - 1, 0) }, (_, i) => !isBoxer(spec) || i % 2 === 1);
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

/** Normalised exhaust layout, accepting the single's and twins' names for it. */
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
    makeSegment({ kind: 'pipe', length: runLength, dIn: dCollector }),
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
  /** Turbos its compiled exhaust has: see `compileExhaust`. */
  turbos?: 1 | 2;
  /**
   * An exhaust drawn for it, loaded in place of one compiled from `pipe` and `collector`, which it was
   * drawn from. Read with `graphFromJson`, as a saved one is.
   */
  graph?: () => ExhaustGraph;
  /** The car the engine comes from, for a launch; without one, it gets a car and six-speed fitted to it. */
  car?: Car;
}

/**
 * A real car, as a launch drives it: its gearbox and final drive, its driven tyres and its weight.
 *
 * Each gearbox's overall ratios are the real car's. Where a gearbox has a second reduction, as the
 * S2000's primary gear or the Evora's second final drive, it is folded into the ratios here, since a
 * launch has one final drive.
 */
export interface Car {
  /** The car, as the Launch section names it. */
  name: string;
  /** Gear ratios, first gear first. */
  ratios: number[];
  finalDrive: number;
  /** Rolling radius of the driven tyres, m. */
  tyreRadius: number;
  /** Its tyres' peak friction coefficient: see `TYRE_GRIP`. */
  tyreGrip: number;
  /** Kerb weight with a 75 kg driver, kg. */
  mass: number;
  /** Share of its weight on the driven wheels at rest: 1 for all-wheel drive. */
  drivenLoad: number;
  /** How long a shift takes, s: `MANUAL_SHIFT_TIME` for a manual, much less for a dual clutch. */
  shiftTime: number;
  /** Whether its gearbox is a dual clutch. */
  dualClutch: boolean;
}

/** A driver's weight, kg, added to a car's kerb weight. */
export const DRIVER_MASS = 75;

/**
 * The car and gearing a preset's engine launches through: the real car it comes from where it has one,
 * and `fitLaunch`'s car otherwise. The launch and shift speeds are fitted either way.
 */
export function presetLaunch(spec: EngineSpec, boosted: boolean, car: Car | null | undefined): LaunchConfig {
  const fit = fitLaunch(spec, boosted);
  if (!car) return fit;
  return {
    ...fit,
    ratios: [...car.ratios],
    finalDrive: car.finalDrive,
    tyreRadius: car.tyreRadius,
    tyreGrip: car.tyreGrip,
    mass: car.mass,
    drivenLoad: car.drivenLoad,
    shiftTime: car.shiftTime,
    dualClutch: car.dualClutch,
  };
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
 * would get the V8's bore, and anything after the F20C its high-speed cam.
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
  exhaustHeaders: true,
  headerRun: 'lengthways',
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
  outputGain: 2.21,
};

const FIVE_CYL: Partial<EngineSpec> = {
  cylinders: 5,
  vAngle: 0,
  exhaustLayout: 'merged',
  exhaustHeaders: true,
  headerRun: 'lengthways',
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
  outputGain: 1.02,
};

const NISSAN_RB26: Partial<EngineSpec> = {
  cylinders: 6,
  vAngle: 0,
  exhaustLayout: 'merged',
  ...idling(0.079),
  // Its fuel cut, a little past the 8000 rpm redline.
  revLimit: 8200,
  flywheelInertia: 0.3,
  pipeCellSize: 0.035,
  // The 2.6 litre RB26DETT: 86.0 x 73.7 mm on a 121.5 mm rod, 8.5:1.
  bore: 0.086,
  stroke: 0.0737,
  rodLength: 0.1215,
  compressionRatio: 8.5,
  // 34.5 mm intakes and 30 mm exhausts, two of each.
  exValveDia: 0.03,
  exValveCount: 2,
  inValveDia: 0.0345,
  inValveCount: 2,
  // The stock cams: 240 degrees on the intake and 236 on the exhaust, with little overlap, and about
  // 8.6 mm of lift. No variable timing; that came with the RB25.
  maxLift: 0.0086,
  evo: 124,
  evc: 360,
  ivo: 352,
  ivc: 592,
  // Two Garrett T28s, one for each three cylinders (`turbos` below), on 0.7 bar through an intercooler,
  // the low end of the 0.7-0.8 bar stock cars are quoted at.
  // Their size is an estimate: no map of the standard compressor is published. The R33's N1 turbo, a
  // bigger one, flows up to 0.20 kg/s at 0.8 bar on Mitsubishi's map of it, and a standard pair is
  // reckoned good for 20-22 lb/min each at peak power, 0.15-0.17 kg/s. Sized to that, they reach full
  // speed near the rev limit rather than well below it. Each fed by three cylinders' pulses, they hold
  // full boost from 3000 rpm.
  boostTarget: 0.7e5,
  turboSize: 0.16,
  intercooler: 0.7,
  // The factory valve recirculates; this is the atmospheric one so many are fitted with instead.
  blowOff: 'atmospheric',
  // Level-matched to the inline four, as the other presets are.
  outputGain: 1.23,
};

const TOYOTA_2GR: Partial<EngineSpec> = {
  cylinders: 6,
  vAngle: 60,
  exhaustLayout: 'perBank',
  ...idling(0.078),
  // Its fuel cut.
  revLimit: 6600,
  flywheelInertia: 0.5,
  mouthSpacing: 1.0,
  pipeCellSize: 0.035,
  // The 3.5 litre 2GR-FE: 94.0 x 83.0 mm, 10.8:1.
  bore: 0.094,
  stroke: 0.083,
  // Estimated: its published figures do not include the rod.
  rodLength: 0.155,
  compressionRatio: 10.8,
  // Twin cams and four valves a cylinder, sized as a typical four-valve head's for the bore.
  ...fourValveHead(0.094),
  maxLift: 0.01,
  // Estimated. The intake cam rests late, closing 65 degrees after bottom dead centre, which lets the
  // runners ram the charge in at the top end; below that its phaser advances it up to 40 degrees, so it
  // closes before the charge flows back out. With the cam fixed at either end, torque falls by a tenth
  // to a quarter somewhere in the range: above 5000 rpm with it early, at 3000 and below with it late.
  ivo: 357,
  ivc: 605,
  vvtIntakeLow: 40,
  vvtLowRpm: 2000,
  vvtHighRpm: 5600,
  // Level-matched to the inline four, as the other presets are.
  outputGain: 0.96,
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
  exhaustHeaders: true,
  headerRun: 'lengthways',
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
  outputGain: 0.88,
};

const BOXER_SIX: Partial<EngineSpec> = {
  cylinders: 6,
  vAngle: 180,
  crankType: 'boxer',
  exhaustLayout: 'perBank',
  exhaustHeaders: true,
  headerRun: 'lengthways',
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
  outputGain: 1.03,
};

/** The F20C preset's idle throttle, output gain and high-speed cam. See `idling`. */
const F20C_IDLE_THROTTLE = 0.078;
/** Level-matched to the single at idle, as the other presets are to this one. */
const F20C_GAIN = 1.31;
const F20C_CAM = { evo: 108, evc: 392, ivo: 328, ivc: 625 };
/** The LT2 preset's idle throttle and output gain. See `idling`. */
const LT2_IDLE_THROTTLE = 0.075;
const LT2_GAIN = 2.1;
/** The LT6 preset's idle throttle and output gain. See `idling`. */
const LT6_IDLE_THROTTLE = 0.082;
const LT6_GAIN = 0.93;

export const ENGINE_PRESETS: EnginePreset[] = [
  {
    name: 'Single, megaphone',
    description: 'A 500 cc air-cooled thumper.',
    // A big air-cooled single is out of breath well before 7000.
    engine: { cylinders: 1, exhaustLayout: 'single', revLimit: 7000, ...idling(0.072), outputGain: 0.68 },
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
      exhaustHeaders: true,
      headerRun: 'lengthways',
      ...idling(0.073),
      // Long-stroke and pushrod: a Harley stops pulling not far past 5500.
      revLimit: 5600,
      flywheelInertia: 0.4,
      // Its own, independent of the default the single uses.
      outputGain: 0.55,
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
      outputGain: 0.81,
    },
    pipe: () => [
      makeSegment({ kind: 'pipe', length: 0.45, dIn: 0.04 }),
      makeSegment({ kind: 'cone', length: 0.3, dIn: 0.04, dOut: 0.075 }),
    ],
  },
  {
    name: 'Inline four, Honda F20C',
    car: {
      // Honda's 2001 release: the gears, a 1.160 primary reduction and a 4.100 final drive, 4.756 in all.
      // 225/50R16 rear tyres, 1274 kg.
      name: 'Honda S2000 (AP1)',
      ratios: [3.133, 2.045, 1.481, 1.161, 0.97, 0.81],
      finalDrive: 4.1 * 1.16,
      tyreRadius: 0.308,
      tyreGrip: TYRE_GRIP.road,
      mass: 1274 + DRIVER_MASS,
      // Its engine behind the front axle puts its weight 50:50.
      drivenLoad: 0.5,
      shiftTime: MANUAL_SHIFT_TIME,
      dualClutch: false,
    },
    description:
      'The 2.0 litre four in the Honda S2000: 87 x 84 mm, 11:1, four valves a cylinder and a 9000 rpm redline. Even 180\u00b0 firing on a flat crank, 1-3-4-2, into equal-length headers: twice the firing frequency of a twin at the same rpm, and no half order at all. It makes 195-200 N\u00b7m from 6000 to 8300 rpm and 228 hp at 8300, against the real engine\u2019s 210 N\u00b7m at 7500 and 240 hp at 8300. Its VTEC switches each valve from a mild cam lobe to a wild one at 5500 rpm. Its valves, cams, runners and exhaust are estimates.',
    engine: {
      cylinders: 4,
      vAngle: 0,
      exhaustLayout: 'merged',
      exhaustHeaders: true,
      headerRun: 'lengthways',
      ...idling(F20C_IDLE_THROTTLE),
      // Its fuel cut; the redline is 9000.
      revLimit: 9150,
      flywheelInertia: 0.14,
      pipeCellSize: 0.035,
      bore: 0.087,
      stroke: 0.084,
      rodLength: 0.153,
      // The North American engine's; the Japanese one's is 11.7.
      compressionRatio: 11,
      // Estimated, like the cams and the runners, and tuned with them to hold its torque to 8300 rpm and
      // put its power peak there, where the real engine has it. Valves a little larger than a
      // typical four-valve head's for the bore: with a typical head's, torque falls away above 7000.
      exValveCount: 2,
      exValveDia: 0.032,
      inValveCount: 2,
      inValveDia: 0.0376,
      // VTEC: a mild lobe for low speed, and at 5500 rpm, where the two make about the same torque and
      // within the 5500-6000 the real engine's ECU switches at, a wild one for the top end. On the high
      // cam alone it makes 130-150 N·m below 4000; on the low one alone, 155 at 7000.
      maxLift: 0.009,
      evo: 128,
      evc: 372,
      ivo: 348,
      ivc: 570,
      camSwitchRpm: 5500,
      highMaxLift: 0.013,
      highEvo: F20C_CAM.evo,
      highEvc: F20C_CAM.evc,
      highIvo: F20C_CAM.ivo,
      highIvc: F20C_CAM.ivc,
      intakeRunnerLength: 0.33,
      intakeRunnerDia: 0.04,
      outputGain: F20C_GAIN,
    },
    // Estimated: equal-length headers into one collector, a pipe and a silencer.
    pipe: () => [makeSegment({ kind: 'pipe', length: 0.45, dIn: 0.04 })],
    collector: () => [
      makeSegment({ kind: 'cone', length: 0.14, dIn: 0.05, dOut: 0.06 }),
      makeSegment({ kind: 'pipe', length: 1.0, dIn: 0.06 }),
      makeSegment({ kind: 'chamber', length: 0.38, dIn: 0.06, dOut: 0.16 }),
      makeSegment({ kind: 'pipe', length: 0.45, dIn: 0.055 }),
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
    name: 'Inline six, Nissan RB26DETT',
    car: {
      // Nissan's catalogue for the BNR34 V-Spec: the Getrag 233 six-speed, a 3.545 final drive, 245/40ZR18
      // tyres and 1560 kg, through ATTESA E-TS Pro to all four wheels.
      name: 'Nissan Skyline GT-R V-Spec (R34)',
      ratios: [3.827, 2.36, 1.685, 1.312, 1.0, 0.793],
      finalDrive: 3.545,
      tyreRadius: 0.319,
      tyreGrip: TYRE_GRIP.road,
      mass: 1560 + DRIVER_MASS,
      drivenLoad: DRIVEN_LOAD.awd,
      shiftTime: MANUAL_SHIFT_TIME,
      dualClutch: false,
    },
    description:
      'The 2.6 litre twin-turbo six in the R32, R33 and R34 Skyline GT-R: 86 x 73.7 mm, 8.5:1, four valves a cylinder and an 8000 rpm redline. It fires every 120\u00b0, 1-5-3-6-2-4, its throws paired 1-6, 2-5 and 3-4: perfectly balanced and evenly fired, so the smooth, silky one. Two small turbos on 0.7 bar, one for each three cylinders, spool from 2000 rpm, on full boost by 3000, and whistle as they do, and every exhaust pulse passes through their turbines, which take the edge off the note; lift off on boost and the blow-off valve vents with a hiss, or with it set to none the compressors surge and flutter. It makes 391 N\u00b7m at 4400 rpm and 328 PS at 6800, about 323 hp, against the real engine\u2019s 368 N\u00b7m and a rated 280 PS. It has one throttle into a plenum where the real one has six individual throttle bodies, and its turbo sizes and exhaust are estimates.',
    engine: NISSAN_RB26,
    pipe: () => fittedExhaust(fullSpec(NISSAN_RB26)).pipe,
    collector: () => fittedExhaust(fullSpec(NISSAN_RB26)).collector,
    // Drawn in the editor: each half's three ports into a turbo, their outlets meeting behind them.
    graph: () => structuredClone(nissanRb26Exhaust) as ExhaustGraph,
    turbos: 2,
  },
  {
    name: 'V6, Toyota 2GR',
    car: {
      // Lotus's 2012 specification for the Evora: the Toyota EA60 six-speed with its close-ratio gears,
      // standard from then on, a 3.777 final drive for first to fourth and 3.238 for fifth and sixth, which
      // is folded into those two ratios. 255/35ZR19 rear tyres, 1382 kg unladen with a full tank.
      name: 'Lotus Evora (2012)',
      ratios: [3.538, 1.913, 1.407, 1.091, 0.9697 * (3.238 / 3.777), 0.8611 * (3.238 / 3.777)],
      finalDrive: 3.777,
      tyreRadius: 0.323,
      tyreGrip: TYRE_GRIP.road,
      mass: 1382 + DRIVER_MASS,
      // Mid-engined, 39:61.
      drivenLoad: 0.61,
      shiftTime: MANUAL_SHIFT_TIME,
      dualClutch: false,
    },
    description:
      'The 3.5 litre 60\u00b0 V6 in half of Toyota\u2019s range, from the Camry to the Lotus Evora. A split-pin crank is what lets it fire evenly every 120\u00b0, in the order 1-2-3-4-5-6, despite a vee too narrow for that on shared pins; each bank hears every other firing, 240\u00b0 apart, through a manifold of its own. Its rod, valves, cam and cam map are estimates. Variable intake cam timing keeps its torque curve flat. The real one also has a two-stage intake; here a second set of runners gained it little, so it has one.',
    engine: TOYOTA_2GR,
    pipe: () => fittedExhaust(fullSpec(TOYOTA_2GR)).pipe,
    collector: () => fittedExhaust(fullSpec(TOYOTA_2GR)).collector,
    // Drawn in the editor: a manifold along each bank, the ports' pipes bent into it.
    graph: () => structuredClone(toyota2grExhaust) as ExhaustGraph,
  },
  {
    name: 'V8, Chevrolet LT2',
    car: {
      // GM's figures for the Tremec TR-9080 eight-speed dual clutch, and a 5.56 final drive: the 3.55 ring
      // and pinion and the drop gear before it, as GM lists it for the Z51. 305/30ZR20 Michelin Pilot Sport
      // 4S rear tyres, 1654 kg.
      name: 'Chevrolet Corvette Stingray Z51 (C8)',
      ratios: [2.91, 1.76, 1.22, 0.88, 0.65, 0.51, 0.4, 0.33],
      finalDrive: 5.56,
      tyreRadius: 0.337,
      tyreGrip: TYRE_GRIP.corvette,
      mass: 1654 + DRIVER_MASS,
      // Mid-engined, 40:60; the dual clutch shifts in about a tenth of a second.
      drivenLoad: 0.6,
      shiftTime: 0.1,
      dualClutch: true,
    },
    description:
      'The 6.2 litre small-block in the mid-engine Corvette: pushrods, two big valves a cylinder, 11.5:1 and a cam that closes the intake late, which only pays off because its long intake runners ram the charge in. Tubular headers into a silencer each side. It makes about 640 N·m and 495 hp here, as the real engine makes 637 and 495.',
    engine: {
      cylinders: 8,
      vAngle: 90,
      crankType: 'crossplane',
      exhaustLayout: 'perBank',
      exhaustHeaders: true,
      headerRun: 'lengthways',
      ...idling(LT2_IDLE_THROTTLE),
      revLimit: 6600,
      mouthSpacing: 1.3,
      // The crank, flexplate and dual clutch's input: the gearbox has no flywheel of its own.
      flywheelInertia: 0.4,
      pipeCellSize: 0.035,
      // 4.065 x 3.622 in on a 6.125 in rod.
      bore: 0.10325,
      stroke: 0.092,
      rodLength: 0.1556,
      compressionRatio: 11.5,
      // 2.13 and 1.59 in valves, one of each.
      exValveDia: 0.0404,
      inValveDia: 0.054,
      maxLift: 0.0145,
      // About 275 and 280 degrees on a 116-degree lobe separation.
      evo: 104,
      evc: 384,
      ivo: 338,
      ivc: 614,
      outputGain: LT2_GAIN,
    },
    // Tubular headers, 1-3/4 in primaries. 600 mm is the best of 450 to 900 across the range: its milder
    // cam has little overlap for them to scavenge through, so they mostly help the low end.
    pipe: () => [makeSegment({ kind: 'pipe', length: 0.6, dIn: 0.044 })],
    collector: () => [
      makeSegment({ kind: 'cone', length: 0.16, dIn: 0.066, dOut: 0.076 }),
      makeSegment({ kind: 'pipe', length: 1.0, dIn: 0.076 }),
      makeSegment({ kind: 'chamber', length: 0.45, dIn: 0.076, dOut: 0.2 }),
      makeSegment({ kind: 'pipe', length: 0.5, dIn: 0.07 }),
    ],
  },
  {
    name: 'V8, Chevrolet LT6',
    car: {
      // The Stingray's eight-speed and 5.56 final drive. With the Z07 package, the one GM times at 2.6 s to
      // 60 mph: 345/25ZR21 Michelin Pilot Sport Cup 2 R rear tyres, and 27 kg off the 1663 kg car in carbon.
      name: 'Chevrolet Corvette Z06 Z07 (C8)',
      ratios: [2.91, 1.76, 1.22, 0.88, 0.65, 0.51, 0.4, 0.33],
      finalDrive: 5.56,
      tyreRadius: 0.344,
      tyreGrip: TYRE_GRIP.corvette,
      mass: 1663 - 27 + DRIVER_MASS,
      drivenLoad: 0.6,
      shiftTime: 0.1,
      dualClutch: true,
    },
    description:
      'The 5.5 litre flat-plane V8 in the Corvette Z06: four cams, four valves a cylinder, 12.5:1 and an 8600 rpm limit. The flat crank fires each bank evenly every 180\u00b0, so it shrieks like a Ferrari rather than burbling. Rod length, cam and headers are estimates; the published figures are the bore, stroke, compression, valves and limit. Its cam, short runners and headers are tuned for the top end, where it makes about 665 hp at 8200 rpm against the real engine’s 670 at 8400. Below that its variable cam timing and long runners, also estimates, give back the mid-range: 623 N·m at 6000 against 624 at 6300.',
    engine: {
      cylinders: 8,
      vAngle: 90,
      crankType: 'flatplane',
      exhaustLayout: 'perBank',
      exhaustHeaders: true,
      headerRun: 'lengthways',
      ...idling(LT6_IDLE_THROTTLE),
      revLimit: 8600,
      mouthSpacing: 0.6,
      flywheelInertia: 0.45,
      pipeCellSize: 0.035,
      bore: 0.10425,
      stroke: 0.08,
      // Estimated, from the stroke and the deck of a small-block.
      rodLength: 0.15,
      compressionRatio: 12.5,
      // 42 mm titanium intakes and 35.5 mm exhausts, two of each.
      exValveDia: 0.0355,
      exValveCount: 2,
      inValveDia: 0.042,
      inValveCount: 2,
      // Estimated: a race-bred cam, the intake closing late because the runners and headers are tuned
      // to ram the charge in after bottom dead centre at the top end. Tuned with them and the cam map for
      // power at 8400 and the flattest curve below it.
      maxLift: 0.0135,
      evo: 104,
      evc: 397,
      ivo: 327,
      ivc: 614,
      // Estimated: a two-stage manifold, as the real one has. The short runners are tuned for 8400 rpm; the
      // long ones, two centimetres longer, fill it better from 7200 to 7800, by up to 20 N·m, and fall
      // behind above that. Longer long runners peak higher in the mid-range, 632 N·m at 6300 for 450 mm
      // ones, but fall away sooner above it: these give the flattest curve.
      intakeRunnerLength: 0.365,
      intakeRunnerShortLength: 0.345,
      intakeSwitchRpm: 7900,
      // Estimated, like the cams, and tuned on the dyno at full throttle: the intake advanced 25 degrees up
      // to 4550 rpm, easing back to rest by 7750, which gives back the mid-range a cam tuned for 8400 costs
      // it. The best advance, found point by point, is about 40 degrees at 4000-4500, 20 at 5000-5500, 10
      // at 6000 and little or none from 6500 and below 3500; holding 25 below 4550 rather than 40 costs up
      // to 28 N·m at 4000 and keeps the dip around 3300 shallower. Retarding the exhaust cam 10 degrees as
      // well adds up to 36 N·m at 4000 but costs 15 below 3500; advancing it loses torque at 4000-4500.
      vvtIntakeLow: 25,
      vvtLowRpm: 4550,
      vvtHighRpm: 7750,
      outputGain: LT6_GAIN,
    },
    // Estimated: equal-length headers, their primaries tuned for 8400 rpm.
    pipe: () => [makeSegment({ kind: 'pipe', length: 0.41, dIn: 0.045 })],
    collector: () => [
      makeSegment({ kind: 'cone', length: 0.16, dIn: 0.068, dOut: 0.076 }),
      makeSegment({ kind: 'pipe', length: 0.9, dIn: 0.076 }),
      makeSegment({ kind: 'chamber', length: 0.4, dIn: 0.076, dOut: 0.19 }),
      makeSegment({ kind: 'pipe', length: 0.35, dIn: 0.07 }),
    ],
  },
  {
    name: 'Boxer four',
    description:
      'Flat, with the pistons of each opposed pair moving out and in together, firing 1-3-2-4 every 180\u00b0 \u2014 the Subaru and the air-cooled VW. Both banks gather into one collector through equal-length headers, so each cylinder\u2019s pulse reaches the merge after the same run.',
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
      exhaustHeaders: true,
      headerRun: 'lengthways',
      ...idling(0.074),
      // A modern 1200 cc parallel twin's.
      revLimit: 7500,
      outputGain: 0.54,
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
