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
import chevroletLt6Exhaust from './exhausts/chevrolet-lt6.json';
import harleyM8Exhaust from './exhausts/harley-m8-121.json';
import hondaRc51Exhaust from './exhausts/honda-rc51.json';
import ducatiMonoExhaust from './exhausts/ducati-superquadro-mono.json';
import subaruFa20dExhaust from './exhausts/subaru-fa20d.json';
import porscheMezger40Exhaust from './exhausts/porsche-mezger-40.json';

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
   * Chamber only: how far its body is rolled about the way it runs, radians, from lying flat: its width
   * level and square to it. Routing only, like a corner.
   */
  roll?: number;
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
 * A cylinder count: one bank of 1 to 6, or two banks of 1 to 6 each (`validLayout`). Two banks are an even
 * count at a V angle above zero, so two is a parallel twin or a V-twin, and six an inline six or a V6.
 */
export type CylinderCount = number;

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
 * `boxer` is a flat engine: two banks 180 degrees apart, every cylinder on a throw of its own, and each
 * opposed pair's throws half a turn apart so the two pistons move out and in together. See `boxerPlan`.
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

/**
 * What the engine burns, and so how it lights its charge.
 *
 * - `gasoline`: port-injected and spark-lit. The throttle sets the air, and `lambda` the fuel with it.
 * - `diesel`: injected into the cylinder near top dead centre and lit by the heat of compression after
 *   an ignition delay. The intake is unthrottled: `throttle` is the pedal, and sets the fuel, up to
 *   `smokeLambda` and `maxFuel`. A governor holds the idle at `idleRpm` and takes the fuel away towards
 *   `revLimit`.
 */
export type Fuel = 'gasoline' | 'diesel';

export const BLOW_OFFS: BlowOff[] = ['atmospheric', 'recirculating', 'none'];

/**
 * What the exhaust is made of. It sets the wall's thermal mass, how well its outside radiates heat
 * away, and how rough its bore is: a rough bore drags on the flow and carries more heat to the wall.
 * Cast iron is the roughest by far; stainless and titanium tube are smooth, and titanium is light.
 */
export type PipeMaterial = 'mildSteel' | 'stainless' | 'castIron' | 'titanium';

export const PIPE_MATERIALS: PipeMaterial[] = ['mildSteel', 'stainless', 'castIron', 'titanium'];

/**
 * Where the engine is listened to. Outdoors there is only the ground to reflect off. In a room each
 * source is heard off its four walls and ceiling too, and then in the reverberation that builds up,
 * as loud as the room's absorption lets it get: a bare concrete garage rings for over a second and
 * is about 10 dB louder than the open, a dyno cell's lined walls soak most of it up.
 *
 * Not every room is closed. A street has its facades either side and the road below, with the sky
 * above and the street running on out of both ends; an underpass its walls and the deck overhead,
 * open at both ends, as a tunnel is. An open side gives back nothing, and takes all that reaches it.
 */
export type Room = 'outdoors' | 'garage' | 'dynoCell' | 'workshop' | 'carPark' | 'tunnel' | 'street' | 'underpass';

export const ROOMS: Room[] = ['outdoors', 'street', 'underpass', 'garage', 'dynoCell', 'workshop', 'carPark', 'tunnel'];

export interface EngineSpec {
  // --- Layout ---
  /**
   * Number of cylinders, all banks together: 1 to 6 on one bank, or an even 2 to 12 on two (`validLayout`).
   * Each layout fires as the real engines of its kind do (`defaultFiringPlan`) unless `firingOrder` or
   * `firingIntervals` say otherwise.
   */
  cylinders: CylinderCount;
  /**
   * Included angle between the two banks, degrees, 15 to 180; 0 for one bank. An even cylinder count at
   * an angle is two banks of half of them; 0 is an inline engine or a parallel twin.
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
   * The order the cylinders fire in, by number from 1, or `null` for the layout's own. Cylinders are numbered
   * front to back along the crank, alternating between the banks on two: 1 the first bank's front, 2 the
   * second's. Every cylinder once, or it is ignored.
   */
  firingOrder: number[] | null;
  /**
   * Crank degrees from each firing in `firingOrder` to the next, one per cylinder, summing to 720; `null` for
   * even firing (or, with no order set either, the layout's own). Uneven gaps make an odd-fire or big-bang
   * engine, and a gap of 0 fires two cylinders together.
   */
  firingIntervals: number[] | null;
  /**
   * Crank arrangement, where the layout has a choice.
   *
   * On a V8 it is the whole difference between an American V8 and a Ferrari. Both fire every 90
   * degrees; what differs is *which bank* each of those firings belongs to, so it only becomes
   * audible once each bank has its own collector. On two banks at 180 degrees, `boxer` makes it a flat
   * engine rather than a 180-degree V. Ignored elsewhere.
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
  /**
   * The valvetrain as drawn: one camshaft in the block, in the valley of a V, working the valves through
   * tappets, pushrods and rocker arms, rather than a cam over each row of valves pressing on them through
   * buckets. Only with one intake and one exhaust valve, upright in a row along the head, so both rocker
   * arms reach across to over the cam and their pushrods come straight up the bank, parallel to the
   * cylinders. Heard as well as drawn: a rocker and a pushrod close each valve's lash, heavier than a
   * bucket.
   */
  pushrods: boolean;

  // --- Combustion ---
  /** What the engine burns. See `Fuel`. */
  fuel: Fuel;
  /**
   * Spark timing, deg ATDC. Negative / >540 means before TDC firing. On a diesel, the start of
   * injection: the charge lights after its ignition delay, from the pressure and temperature the
   * compression leaves it at.
   *
   * With `advanceCurve` on, this is the timing for a charge that burns over `burnDuration`; the
   * spark moves from it to keep each cycle's combustion phased the same.
   */
  ignition: number;
  /**
   * Whether the spark follows an advance map. On, it moves with the predicted burn of each charge:
   * later at low rpm, where the burn is quick, earlier at part throttle and high rpm, where it is
   * slow. Off, it fires at `ignition` whatever the charge, as a fixed-timing magneto does. A diesel has
   * no spark to move, and ignores it.
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
   *
   * On a diesel, the duration of the diffusion burn at full fuel and 10 m/s mean piston speed, the part
   * of the burn that follows the premixed spike: shorter with less fuel, longer the faster the engine
   * turns.
   */
  burnDuration: number;
  /**
   * Air-fuel equivalence ratio λ: the air-fuel ratio as a multiple of stoichiometric. 1 is
   * stoichiometric, below 1 rich, above 1 lean.
   *
   * Each cylinder's port injector meters fuel in proportion to the air its runner draws, so this is
   * the mixture every cylinder traps. Lean, every kilogram of charge carries less fuel and burns
   * slower; rich, the extra fuel has no oxygen to burn with and goes out unburned. A diesel's mixture
   * is set by its pedal instead, and it ignores this.
   */
  lambda: number;
  /**
   * Overrun fuel cut, as a fuel-injected engine's ECU does it: with the throttle shut above 1500 rpm the
   * fuel stops, and it comes back below 1200 rpm or as soon as the throttle opens, each higher on an
   * engine that idles above 800 (`fuel_cut_rpms` in `crates/engine-sim/src/spec.rs`). The engine is then turned over by its load, pumping air.
   *
   * Off, a closed throttle keeps feeding fuel with the air that leaks past it, as a carburettor
   * does, and the engine keeps firing weakly on the overrun.
   */
  fuelCut: boolean;
  /**
   * Overrun crackle, as a performance car's "pops and bangs" map does it: for a few seconds after the
   * throttle shuts above 2500 rpm it holds off the fuel cut, cracks the throttle open, fires the spark
   * far after top dead centre and skips it on some cycles. The charges it sends out unburned light in
   * the hot header, and pop.
   */
  overrunCrackle: boolean;
  /** How hard the crackle map works, 0..1: later sparks, more of them skipped, the throttle further open. */
  crackleIntensity: number;
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
   * A diesel's smoke limit: the richest it is fuelled at full pedal, as λ. Past it the fuel finds too
   * little air to burn clean, so the pump's stop, or its boost compensator, holds it there.
   */
  smokeLambda: number;
  /**
   * A diesel's full delivery: the most fuel its pump injects a cylinder each cycle, kg, which sets its
   * torque where the smoke limit does not. 0 for no limit but the smoke limit.
   */
  maxFuel: number;
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
   *
   * On a diesel, the pedal: the share of the full fuel delivery it asks for, with the intake wide open.
   */
  throttle: number;
  /**
   * Throttle bore, m: each throttle body's, with dual plenums. With `throttle` this fixes the flow area
   * into the plenum.
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
   * Typically one to two times displacement for a single. Sets the plenum's width where that is left at
   * 0 (`plenumShapeOf`); with every dimension set, the volume is theirs and this goes unused.
   */
  plenumVolume: number;
  /**
   * The plenum's size, m: its length along the engine, and its width and height at its front, where the
   * throttle body is. 0 or less works each out (`plenumShapeOf`): the length past every runner, the
   * height to take the throttle body's flange, and the width to hold `plenumVolume`.
   *
   * The plenum is solved along its length, not as one volume: a pressure wave takes a millisecond or so
   * to cross it, so the cylinders at its far end draw from air its waves leave different from that by the
   * throttle, and the box rings along its length. A longer plenum rings lower, and its far cylinders
   * breathe further from the near ones.
   */
  plenumLength: number;
  plenumWidth: number;
  plenumHeight: number;
  /**
   * How much of its section the plenum has lost at its back, 0..0.8, narrowing evenly from none at the
   * front: beside an inline head its side away from the head drawn in, on a V or a boxer its top dropped.
   */
  plenumTaper: number;
  /**
   * Dual plenums, as the LT6 has: on a V or a boxer, the plenum's casting divided down its middle, each
   * bank's runners drawing from their own half, each half with its own throttle body of `throttleDia`
   * on its front. The size above is the casting's, both halves together. An engine of one bank, or a V
   * so narrow its banks share a head, has the one plenum whatever this says (`plenumCountOf`).
   *
   * Apart, each half feeds only its own bank, and rings with that bank's pulses alone: on an engine
   * whose banks each fire evenly, at its bank's even spacing, not the engine's. That tunes the intake
   * differently from one shared box, better at some speeds and worse at others, and the balance valves
   * between the halves (`plenumBalanceRpm`) choose between the two.
   */
  dualPlenum: boolean;
  /**
   * With dual plenums, the speed the balance valves through the wall between them open at, rev/min,
   * joining the halves into one box, the speed they shut again at, parting them, and the speed they open
   * again at for the top end; each 0 or less for never, so at 0 they stay shut, and with only the first
   * they stay open above it. Each happens at its speed going up, and 150 rev/min below it coming down, so
   * they do not flap back and forth at any.
   */
  plenumBalanceRpm: number;
  plenumBalanceShutRpm: number;
  plenumBalanceReopenRpm: number;
  /**
   * Volume of the airbox the throttle draws from, m^3. 0 or less sizes it at four times the engine's
   * displacement; see `airboxVolumeOf`.
   *
   * The airbox and the snorkel feeding it are the inlet tract, solved like the exhaust. The runners'
   * pulses travel up it from the throttle, ring in it and leave the snorkel's mouth as the intake's
   * note, and the jet past the throttle plate hisses through it. A turbocharged engine draws through
   * its compressors instead, and has none.
   *
   * Dual plenums' two throttle bodies each have a tract of their own, mirrored either side of the
   * engine: this is their two airboxes' volume together, and the snorkel's length and bore each one's.
   */
  airboxVolume: number;
  /** Length of the snorkel from the airbox to its open mouth, m. Longer tunes the tract lower. */
  snorkelLength: number;
  /** The snorkel's bore, m. 0 or less makes it a little wider than the throttle; see `snorkelDiaOf`. */
  snorkelDia: number;
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
  // every turbo's, but for the boost and size of one given its own (`TurboMount.settings`). See
  // `crates/engine-sim/src/turbo.rs`.
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
   * charge still goes down the pipe, where it can light and pop, and the engine bounces off the
   * limit in the stuttering way a real one does. With the speed held (`freeRunning` off) at or past the limit, the crank is let
   * go instead, unloaded, so it can bounce too.
   *
   * On a diesel, the governed speed: its governor takes the fuel away over the 300 rpm below it, none
   * left at it, so the engine runs up to it smoothly rather than bouncing off it.
   */
  revLimit: number;
  /**
   * Idle speed, rev/min: what the idle air valve holds with the throttle shut, as an engine management
   * system does. The valve bypasses the throttle plate, opened by a controller on the crank speed: shut
   * above the idle speed, so the throttle alone sets the speed there, and opening further below it to
   * hold the idle against a load, up to its limit, past which the engine stalls. 0 for no idle control.
   * Only for a free-running engine (`freeRunning`). On a diesel the governor holds it, with fuel.
   */
  idleRpm: number;
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
   * enough for the fastest wave not to cross it in one sample, about 17 mm at 96 kHz, 34 mm at 48 kHz
   * and 37 mm at 44.1 kHz, and anything smaller asked for is raised to that.
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
  /** What the exhaust is made of. See `PipeMaterial`. */
  pipeMaterial: PipeMaterial;
  /**
   * Air speed past the exhaust, m/s. 0 is a stationary engine, 25 is roughly 90 km/h.
   *
   * Cools the pipe wall, which cools the gas, which slows the wave speed and drops the
   * tuning. Radiation off oxidised steel matters as much as convection here.
   */
  airSpeed: number;
  /** Height of the lowest exhaust mouth above the ground, m: where the ground is, under the scene. */
  exhaustHeight: number;
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
  /** Where the engine is listened to. See `Room`. */
  room: Room;
  /** Master output gain, linear. */
  outputGain: number;
  /** Scales valve-seating clacks and piston/mechanical noise, 0..1. */
  mechNoise: number;
  /** Scales broadband turbulence generated at the valve throat, 0..1. */
  throatNoise: number;
  /**
   * Level of the roar of the jet each tailpipe blows into the air, 0..1 (1 = every instant of each
   * pulse taken as a steady jet of that speed). Loudest from a narrow outlet at high speed.
   */
  jetNoise: number;
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
  /** Where the engine makes its sound, as drawn (`soundSources`). */
  sources?: SoundSources;
  /** Where the listener's ear is, m, in the scene's frame: the camera. Absent, it stands by default. */
  listener?: [number, number, number];
}

/**
 * Where the engine makes its sound, in the scene's frame, m: x across the crank, y up, z along it, rearwards.
 * The simulation hears each from where it is, along its own path to the listener and its own reflection
 * off the ground.
 */
export interface SoundSources {
  /** Each tailpipe's outlet, by the duct the solver radiates it from. */
  mouths: Array<{ duct: string; position: [number, number, number] }>;
  /** Where the engine draws its air: its snorkel's mouth. */
  intake?: [number, number, number];
  /** Where dual plenums' other inlet tract draws its air: its snorkel's mouth. */
  secondIntake?: [number, number, number];
  /** The middle of the engine, where its casing radiates from. */
  engine?: [number, number, number];
  /** Where the turbochargers are. */
  turbo?: [number, number, number];
  /**
   * The casing's surfaces, each heard from where it is and louder the way it faces. Without them the casing
   * radiates from `engine` alike every way.
   */
  surfaces?: CasingSurface[];
}

/**
 * One of the casing's radiating surfaces, and so what it carries: a block side its bank's pistons and the
 * block's combustion ring, a head its valvetrain, the oil pan the bottom end, the front cover the timing drive.
 */
export interface CasingSurface {
  kind: 'blockSide' | 'head' | 'oilPan' | 'frontCover';
  /** The bank whose casting it is, as `physicalBank` counts them: for a block side or a head. */
  bank: number;
  /** Its middle, and the way it faces, a unit vector out of the engine. */
  position: [number, number, number];
  facing: [number, number, number];
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
  /** Afterfires, unburned fuel lighting in the exhaust, since the last snapshot; absent for none. */
  afterfires?: number;
  /** Whether the overrun crackle map is running; absent when it is not. */
  crackle?: boolean;
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
  /**
   * Gauge pressure in every cell of the inlet tract, Pa, from the throttle out to the snorkel's mouth.
   * Empty on an engine with a turbo, which has none.
   */
  inletPressure: Float32Array;
  /** Air speed in every cell of the inlet tract, m/s, the same way: positive out towards the snorkel's mouth. */
  inletVelocity: Float32Array;
  /** Gauge pressure in the plenum, Pa, over its volume: below zero, the manifold's vacuum. */
  plenumPressure: number;
  /**
   * Gauge pressure in each zone along the plenum, Pa, from the throttle at its front to its back: with
   * dual plenums, bank 0's and then bank 1's, as many each.
   */
  plenumZones: Float32Array;
  /** Whether dual plenums' balance valves are open, joining them. */
  plenumBalanced: boolean;
  /**
   * Gauge pressure in every cell of every intake runner, Pa, in cylinder order, each from its valve end,
   * taking `runnerCells` values in turn.
   */
  runnerPressure: Float32Array;
  runnerCells: number[];
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
  /** The turbos' mean shaft speed, rev/min. */
  shaftRpm: number;
  /** The wastegates' and the blow-off valves' mean openings, 0..1. */
  wastegate: number;
  blowOff: number;
  /** Whether a compressor is surging. */
  surging: boolean;
  /** Each turbo's own, in the order of the exhaust's turbines. */
  turbos: TurboUnitSnapshot[];
}

/** One turbo's state, sent with each snapshot: its shaft speed, rev/min, and its wastegate's and blow-off valve's openings, 0..1. */
export interface TurboUnitSnapshot {
  id: string;
  shaftRpm: number;
  wastegate: number;
  blowOff: number;
}

/** A launch's state, sent with each snapshot while it runs. */
export interface LaunchSnapshot {
  phase: 'launch' | 'hold' | 'pull' | 'shiftOut' | 'shiftIn' | 'cooldown';
  /** Gear engaged, 1-based. */
  gear: number;
  /** Road speed, km/h. */
  speedKmh: number;
  /** Seconds since the clock started, once the car had rolled a foot; on the dyno, since the sweep started. */
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
  /** The throttle's opening the run holds, 0..1: open through a pull, shut through a shift that lifts off. */
  throttle: number;
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
  /** Whether the driven wheels are the front ones, which the car's weight moves off as it accelerates. */
  frontWheelDrive: boolean;
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
  /**
   * A dyno pull rather than a launch: the crank drives a dyno's absorber through one gear at 1:1, from
   * `launchRpm` to `shiftRpm`, and the car, tyres and gearbox are not used.
   */
  dyno: boolean;
  /** How fast a dyno pull sweeps the engine up, rev/min per s. */
  sweepRate: number;
}

/** A dyno pull's sweep rate, rev/min per s: an engine dyno's steady sweep. */
export const DYNO_SWEEP_RATE = 500;

/** Where a dyno pull starts on auto, rev/min: a quarter of the rev limiter, and no lower than this. */
export const DYNO_FROM_RPM = 2000;

/**
 * A dyno pull for `spec`: from `from` to `to` rpm, each on auto where `null`, at `rate` rpm/s. It starts
 * at a quarter of the rev limiter, and ends where a launch shifts, just under it.
 */
export function fitDyno(
  spec: EngineSpec,
  from: number | null = null,
  to: number | null = null,
  rate: number | null = null,
): LaunchConfig {
  const fit = fitLaunch(spec, false, [1]);
  const end = Math.min(to ?? fit.shiftRpm, spec.revLimit - 50);
  const start = Math.max(Math.min(from ?? Math.max(0.25 * spec.revLimit, DYNO_FROM_RPM), end - LAUNCH_RPM_MARGIN), 1000);
  return { ...fit, ratios: [1], finalDrive: 1, launchRpm: start, shiftRpm: end, dyno: true, sweepRate: rate ?? DYNO_SWEEP_RATE };
}

/** The wheels a car drives: the rear, the front or all four. */
export type Drive = 'rwd' | 'fwd' | 'awd';
export const DRIVES: Drive[] = ['rwd', 'fwd', 'awd'];

/**
 * Share of a car's weight on its driven wheels at rest, by the wheels it drives: the rear wheels of a
 * front-engined car carry about half, the front wheels of a front-wheel-drive car, its engine over them,
 * about three fifths, and all four carry it all. As the car accelerates its weight moves back: onto
 * driven rear wheels, and off driven front ones.
 */
export const DRIVEN_LOAD: Record<Drive, number> = { rwd: 0.5, fwd: 0.6, awd: 1 };

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
    frontWheelDrive: false,
    tractionControl: true,
    launchRpm,
    shiftRpm,
    shiftTime: MANUAL_SHIFT_TIME,
    dualClutch: false,
    dyno: false,
    sweepRate: DYNO_SWEEP_RATE,
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
    if (finite(partial.roll) && partial.roll !== 0) seg.roll = partial.roll;
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
  firingOrder: null,
  firingIntervals: null,
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
  pushrods: false,

  fuel: 'gasoline',
  ignition: 695, // 25 deg BTDC
  burnDuration: 55,
  advanceCurve: true,
  lambda: 1,
  fuelCut: true,
  overrunCrackle: false,
  crackleIntensity: 0.6,
  combustionVariability: 1,
  smokeLambda: 1.45,
  maxFuel: 0,
  recipMass: 0.55,
  throttle: 0.75,
  // Zero means "derive it from the engine" — see `throttleDiaOf` and `plenumVolumeOf`.
  // A fixed figure here would be a single-cylinder's, and wrong for everything else.
  throttleDia: 0,
  plenumVolume: 0,
  plenumLength: 0,
  plenumWidth: 0,
  plenumHeight: 0,
  plenumTaper: 0.4,
  dualPlenum: false,
  plenumBalanceRpm: 0,
  plenumBalanceShutRpm: 0,
  plenumBalanceReopenRpm: 0,
  airboxVolume: 0,
  snorkelLength: 0.3,
  snorkelDia: 0,
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
  idleRpm: 800,
  freeRunning: false,
  // A bare crank is nearer 0.06; 0.25 represents crank plus clutch and primary
  // drive, which is what a rider actually hears. Lower it for a lumpier idle.
  flywheelInertia: 0.25,
  // About 20 N*m on this engine.
  load: 0.46,

  portGasTemp: 950,
  pipeCellSize: 0.035,
  pipeWallThickness: 0.0012,
  pipeMaterial: 'mildSteel',
  airSpeed: 0,
  exhaustHeight: 0.35,
  cylinderSpread: 0.3,
  groundReflection: 0.7,
  room: 'outdoors',
  outputGain: 0.77,
  mechNoise: 0.45,
  throatNoise: 0.5,
  jetNoise: 0.5,
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
 * This is **data, not a derivation**, wherever a real engine's crank is known: the inline engines, the
 * V4, V6, V8s and the flat four and six. Real firing orders are chosen for crankshaft balance and bearing
 * loads, and cannot be recovered from a formula; quoting them from the engines they belong to is both
 * honest and shorter. The twin is the exception, where the shared-crankpin relationship genuinely does
 * derive the interval and is worth keeping explicit, because a rider can hear the V angle in it; and so are
 * the V10 and V12, two banks of an inline crank (`doubledInline`), and the larger flat engines
 * (`boxerBankPins`), built the way the real ones are. `firingOrder` and `firingIntervals` replace any of
 * them (`firingPlan`).
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
 * A crank stored as pin angles rather than as firing offsets: the V8s', and as they are, the V6's, the V4's
 * and the flat engines'.
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
 * The V8s are indexed by throw from the front, the two cylinders of a throw side by side, first bank
 * then second. Read off real engines: crossplane is the Ford 302 order 1-5-4-2-6-3-7-8, cylinders
 * 1-4 on the first bank, giving pins at 0/270/90/180 along the crank — the four-plane crank, its end
 * throws half a turn apart so the secondary couple cancels. Flatplane puts every throw in one plane,
 * so its pins are only ever 0 or 180, and in the order an inline four's are, 0/180/180/0 along the
 * crank, which is what cancels its primary couple. Each bank then fires 1-3-4-2 by throw, as an
 * inline four does, and each second-bank cylinder fires 90 degrees after its pin partner: the
 * Ferrari order 1-5-3-7-4-8-2-6.
 */
interface PinCrank {
  pins: number[];
  revs: number[];
  banks: number[];
}

const V8_CROSSPLANE: PinCrank = {
  pins: [0, 0, 270, 270, 90, 90, 180, 180],
  revs: [0, 0, 0, 0, 1, 1, 0, 1],
  banks: [0, 1, 0, 1, 0, 1, 0, 1],
};

const V8_FLATPLANE: PinCrank = {
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
const V6_SPLIT_PIN: PinCrank = {
  pins: [0, 60, 240, 300, 120, 180],
  revs: [0, 0, 0, 0, 1, 1],
  banks: [0, 1, 0, 1, 0, 1],
};
const V6_THROWS = [0, 0, 1, 1, 2, 2];

/**
 * The Honda VFR's V4: a 180-degree crank, each throw shared by both banks, firing 180-270-180-90 at a
 * 90-degree vee.
 */
const V4_180: PinCrank = {
  pins: [0, 0, 180, 180],
  revs: [0, 1, 0, 1],
  banks: [0, 1, 0, 1],
};

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
const BOXER_4: PinCrank = {
  pins: [0, 180, 180, 0],
  revs: [0, 0, 0, 1],
  banks: [0, 1, 0, 1],
};
const BOXER_6: PinCrank = {
  pins: [0, 180, 240, 60, 120, 300],
  revs: [0, 0, 0, 1, 1, 1],
  banks: [0, 1, 0, 1, 0, 1],
};

/**
 * The first bank's pins of a flat engine with no crank of its own in the tables: spread evenly round the
 * shaft, so with each opposite pin half a turn from its partner's the engine fires evenly.
 */
function boxerBankPins(perBank: number): number[] {
  if (perBank === 1) return [0];
  if (perBank === 4) return [0, 180, 90, 270];
  if (perBank === 5) return [0, 144, 288, 72, 216];
  return [0, 240, 120, 300, 60, 180];
}

const wrap720 = (deg: number) => ((deg % 720) + 720) % 720;

const pinOffsets = (crank: PinCrank, vAngle: number) =>
  crank.pins.map((pin, i) => (((pin + vAngle * crank.banks[i]! + 360 * crank.revs[i]!) % 720) + 720) % 720);

/** Whether the engine has two banks of cylinders: a V or a flat engine, with an even count and a vee. */
export function hasTwoBanks(spec: EngineSpec): boolean {
  return spec.vAngle > 0 && spec.cylinders >= 2 && spec.cylinders % 2 === 0;
}

/** Cylinders on each bank. */
export function cylindersPerBank(spec: EngineSpec): number {
  return hasTwoBanks(spec) ? spec.cylinders / 2 : spec.cylinders;
}

/** Whether the layout is one the engine can be: one bank of 1 to 6, or two of 1 to 6 each. */
export function validLayout(spec: EngineSpec): boolean {
  const n = cylindersPerBank(spec);
  return Number.isInteger(n) && n >= 1 && n <= 6;
}

/** Whether the spec is a flat engine: two banks laid flat, 180 degrees apart, on a boxer crank. */
export function isBoxer(spec: EngineSpec): boolean {
  return spec.crankType === 'boxer' && hasTwoBanks(spec) && Math.abs(spec.vAngle - 180) < 1e-9;
}

/**
 * A boxer's plan: offsets from the pins as for a V, and a throw of its own for every cylinder. The four and
 * the six are the tables'; any other fires each opposed pair a revolution apart, the first bank's pin then
 * its partner's.
 */
function boxerPlan(spec: EngineSpec): FiringPlan {
  const perBank = cylindersPerBank(spec);
  let offsets: number[];
  if (perBank === 2) offsets = pinOffsets(BOXER_4, spec.vAngle);
  else if (perBank === 3) offsets = pinOffsets(BOXER_6, spec.vAngle);
  else offsets = boxerBankPins(perBank).flatMap((pin) => [wrap720(pin), wrap720(pin + 180 + spec.vAngle)]);
  return { offsets, banks: offsets.map((_, i) => i % 2), bankCount: 2, throws: offsets.map((_, i) => i) };
}

/**
 * Inline engines: every cylinder on its own pin, one bank, firing evenly in the usual order.
 *
 * The offsets are by cylinder, read off the firing order: an inline three fires 1-3-2 every 240 degrees
 * on a 120-degree crank; an inline four 1-3-4-2 every 180 on a flat crank, pins 0/180/180/0, its outer
 * pair and inner pair each sharing a throw's angle; an inline five 1-2-4-5-3 every 144 on a 72-degree
 * crank, the Audi and Volvo order; an inline six 1-5-3-6-2-4 every 120, which pairs its throws 1-6, 2-5
 * and 3-4.
 */
const INLINE_OFFSETS: Record<number, number[]> = {
  1: [0],
  3: [0, 480, 240],
  4: [0, 540, 180, 360],
  5: [0, 144, 576, 288, 432],
  6: [0, 480, 240, 600, 120, 360],
};

/**
 * A V of two banks of an inline engine's crank, each throw shared: the second bank's cylinder fires the bank
 * angle after its partner, in the same revolution. On an inline five's crank that is the even 72-degree
 * firing of a 72-degree V10 and the 54-90 of the Viper's 90; on an inline six's, a 60-degree V12's even 60.
 */
function doubledInline(inline: number[], vAngle: number): FiringPlan {
  const offsets = inline.flatMap((o) => [o, wrap720(o + vAngle)]);
  return { offsets, banks: offsets.map((_, i) => i % 2), bankCount: 2 };
}

/** The layout's own firing plan, from real engines' cranks, before any firing order the spec sets. */
export function defaultFiringPlan(spec: EngineSpec): FiringPlan {
  if (isBoxer(spec)) return boxerPlan(spec);
  const crank = (c: PinCrank, throws?: number[]): FiringPlan => ({
    offsets: pinOffsets(c, spec.vAngle),
    banks: [...c.banks],
    bankCount: 2,
    ...(throws ? { throws } : {}),
  });
  // Two of one is a twin either way: a V-twin's cylinders are each their own bank, and a parallel twin's
  // firing offset is expressed the same way.
  if (spec.cylinders === 2) return { offsets: [0, firingOffsetDeg(spec)], banks: [0, 1], bankCount: 2 };
  if (hasTwoBanks(spec)) {
    if (spec.cylinders === 4) return crank(V4_180);
    if (spec.cylinders === 6) return crank(V6_SPLIT_PIN, [...V6_THROWS]);
    if (spec.cylinders === 8) return crank(spec.crankType === 'flatplane' ? V8_FLATPLANE : V8_CROSSPLANE);
    if (spec.cylinders === 10 || spec.cylinders === 12) {
      return doubledInline(INLINE_OFFSETS[spec.cylinders / 2] ?? [0], spec.vAngle);
    }
  }
  const inline = INLINE_OFFSETS[spec.cylinders];
  if (inline) return { offsets: [...inline], banks: inline.map(() => 0), bankCount: 1 };
  // A layout the engine cannot be: an even firing, one bank, so it still runs.
  const n = Math.max(spec.cylinders, 1);
  return { offsets: Array.from({ length: n }, (_, i) => (i * 720) / n), banks: Array(n).fill(0), bankCount: 1 };
}

/** The order `plan` fires its cylinders in, by number from 1. */
export function firingOrderOf(plan: FiringPlan): number[] {
  return plan.offsets
    .map((_, i) => i)
    .sort((a, b) => plan.offsets[a]! - plan.offsets[b]!)
    .map((i) => i + 1);
}

/** The gaps from each of `plan`'s firings to the next, in firing order, crank degrees. */
export function firingIntervalsOf(plan: FiringPlan): number[] {
  const fires = [...plan.offsets].sort((a, b) => a - b);
  return fires.map((f, i) => (i + 1 < fires.length ? fires[i + 1]! : fires[0]! + 720) - f);
}

/**
 * Why the spec's own firing order or intervals cannot be fired, or `null` where they can or it sets neither:
 * an order must name every cylinder once, and the intervals be one per cylinder, none negative, summing to 720.
 * A gap of 0 fires two cylinders together.
 */
export function firingOrderProblem(spec: EngineSpec): string | null {
  const n = defaultFiringPlan(spec).offsets.length;
  const order = spec.firingOrder;
  if (order) {
    if (order.length !== n) return `The firing order needs all ${n} cylinders.`;
    const seen = new Set<number>();
    for (const c of order) {
      if (!Number.isInteger(c) || c < 1 || c > n) return `There is no cylinder ${c}: they are numbered 1 to ${n}.`;
      if (seen.has(c)) return `Cylinder ${c} is in the firing order twice.`;
      seen.add(c);
    }
  }
  const gaps = spec.firingIntervals;
  if (gaps) {
    if (gaps.length !== n) return `The intervals need one per cylinder, ${n} of them.`;
    if (gaps.some((g) => !Number.isFinite(g) || g < 0)) return 'No interval can be less than 0°.';
    const sum = gaps.reduce((a, b) => a + b, 0);
    if (Math.abs(sum - 720) > 1e-6) return `The intervals add up to ${Math.round(sum)}°, not 720°.`;
  }
  return null;
}

/** The firing offsets the spec's own firing order and intervals give, cylinder 1 at 0, or `null`. */
function customOffsets(spec: EngineSpec, plan: FiringPlan): number[] | null {
  if (!spec.firingOrder && !spec.firingIntervals) return null;
  if (firingOrderProblem(spec)) return null;
  const n = plan.offsets.length;
  const order = (spec.firingOrder ?? firingOrderOf(plan)).map((c) => c - 1);
  const intervals = spec.firingIntervals ?? Array<number>(n).fill(720 / n);
  const offsets = Array<number>(n).fill(0);
  let at = 0;
  order.forEach((c, k) => {
    offsets[c] = at;
    at += intervals[k]!;
  });
  const first = offsets[0]!;
  return offsets.map((o) => wrap720(o - first));
}

/** Which firings happen when: the layout's own plan, or the spec's firing order and intervals on it. */
export function firingPlan(spec: EngineSpec): FiringPlan {
  const plan = defaultFiringPlan(spec);
  const offsets = customOffsets(spec, plan);
  if (!offsets) return plan;
  // Every cylinder of a flat engine keeps its own throw; on any other a pin is shared where the order lets
  // two cylinders share one.
  const { throws, ...rest } = plan;
  return { ...rest, offsets, ...(isBoxer(spec) && throws ? { throws } : {}) };
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
  // Wide enough that a cylinder staggered along its throw (`rodStagger`) clears the next throw's too, and
  // that a throw's pins, as wide as its stagger twice over, leave room between it and the next for a web
  // either side of a main journal.
  const stagger = rodStagger(spec);
  return stagger > ROD_STAGGER ? Math.max(spec.bore * 1.45, 2 * stagger + THROW_ROOM) : spec.bore * 1.45;
}

/**
 * How far apart along the crank the cylinders sharing a throw sit, m: a rod's width, so their rods run side
 * by side on a shared pin, or each on its own pin of a split one. It is why one bank of a V sits a little
 * ahead of the other.
 */
export const ROD_STAGGER = 0.016;

/** What lies between one throw's pins and the next's along the crank, m: a web each side of a main journal. */
const THROW_ROOM = 0.05;

/**
 * How thick the web between a split pin's two offset pins is along the crank, m: it adds to their
 * stagger, so each pin keeps a rod's width of its own.
 */
export const SPLIT_WEB = 0.005;

/** How far apart a V's pistons have to stay, m, side to side or along the crank. */
const PISTON_GAP = 0.004;

const staggers = new Map<string, number>();

/**
 * How far apart along the crank the cylinders sharing a throw sit, m: `ROD_STAGGER`, and `SPLIT_WEB` more
 * on a split pin, for the web between its two pins; or in a V too narrow
 * for the two banks' pistons to pass each other at the bottom of their strokes, as far again as keeps
 * them clear, as a VR engine's banks are staggered.
 *
 * The pistons as drawn, round and from skirt to crown, followed through the cycle: the stagger is the least
 * that keeps every point round one's rims `PISTON_GAP` clear of the other.
 */
export function rodStagger(spec: EngineSpec): number {
  if (!(spec.vAngle > 0) || isBoxer(spec) || physicalBankCount(spec) < 2) return ROD_STAGGER;
  // Each throw's two cylinders: their banks' turns and their pins' angles round the shaft.
  const pairs = crankPins(spec)
    .filter((p) => p.cylinders.length === 2)
    .map((p) =>
      p.cylinders.map((c, k) => ({
        turn: physicalBank(spec, c) === 0 ? 0 : (-spec.vAngle * Math.PI) / 180,
        pin: (p.angles[k]! * Math.PI) / 180,
      })),
    );
  if (pairs.length === 0) return ROD_STAGGER;
  const split = pairs.some(([p, q]) => Math.abs(p!.pin - q!.pin) > 1e-9);
  const least = ROD_STAGGER + (split ? SPLIT_WEB : 0);
  const key = `${spec.bore} ${spec.stroke} ${spec.rodLength} ${spec.vAngle} ${pairs.map((p) => p.map((c) => c.pin).join(',')).join(';')}`;
  const known = staggers.get(key);
  if (known !== undefined) return known;
  const a = spec.stroke / 2;
  const l = spec.rodLength;
  const r = (spec.bore / 2) * 0.985;
  const half = spec.bore * 0.34 * 0.625;
  type V3 = [number, number, number];
  /**
   * A piston for the bank turned by `t` whose pin is at `pin` round the shaft, the crank turned `theta`,
   * `dz` along the crank: its wrist pin up its bore's axis from the pin where the drawn crank has it, and
   * the axis's direction.
   */
  const piston = (t: number, pin: number, theta: number, dz: number): { at: V3; up: V3 } => {
    const [wx, wy] = [a * Math.sin(theta - pin), a * Math.cos(theta - pin)];
    const [px, py] = [wx * Math.cos(t) + wy * Math.sin(t), -wx * Math.sin(t) + wy * Math.cos(t)];
    const h = py + Math.sqrt(l * l - px * px);
    const up: V3 = [-Math.sin(t), Math.cos(t), 0];
    return { at: [up[0] * h, up[1] * h, dz], up };
  };
  /** Whether, `dz` apart along the crank, some point round the rims of one piston comes within the gap of the other. */
  const touch = (dz: number) =>
    pairs.some(([p, q]) => {
      for (let k = 0; k < 72; k++) {
        const theta = (k / 72) * 2 * Math.PI;
        const ends = [piston(p!.turn, p!.pin, theta, 0), piston(q!.turn, q!.pin, theta, dz)];
        for (const [one, other] of [ends, [ends[1]!, ends[0]!]]) {
          const side: V3 = [one!.up[1], -one!.up[0], 0];
          for (const y of [-half, -half / 2, 0, half / 2, half]) {
            for (let m = 0; m < 24; m++) {
              const c = Math.cos((m / 24) * 2 * Math.PI) * r;
              const z = Math.sin((m / 24) * 2 * Math.PI) * r;
              const pt = [0, 1, 2].map((i) => one!.at[i]! + one!.up[i]! * y + side[i]! * c + (i === 2 ? z : 0));
              const d = pt.map((v, i) => v - other!.at[i]!);
              const along = d[0]! * other!.up[0]! + d[1]! * other!.up[1]!;
              if (Math.abs(along) > half + PISTON_GAP) continue;
              const radial = Math.hypot(d[0]! - along * other!.up[0]!, d[1]! - along * other!.up[1]!, d[2]!);
              if (radial < r + PISTON_GAP) return true;
            }
          }
        }
      }
      return false;
    });
  let stagger = least;
  while (stagger < 4 * spec.bore && touch(stagger)) stagger += 0.001;
  staggers.set(key, stagger);
  return stagger;
}

/**
 * Where cylinder `cylinder` sits along the crank, m, the engine centred on the origin: at its throw, and on
 * a throw it shares, staggered from the other cylinders on it by `rodStagger`.
 */
export function cylinderZ(spec: EngineSpec, cylinder: number): number {
  const pins = crankPins(spec);
  const index = Math.max(pins.findIndex((p) => p.cylinders.includes(cylinder)), 0);
  const pin = pins[index];
  const along = (index - (pins.length - 1) / 2) * cylinderSpacing(spec);
  if (!pin || pin.cylinders.length < 2) return along;
  const k = pin.cylinders.indexOf(cylinder);
  return along + (k - (pin.cylinders.length - 1) / 2) * rodStagger(spec);
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
  /** The wheels it drives. */
  drive: Drive;
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
    frontWheelDrive: car.drive === 'fwd',
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
  'exhaustHeight',
  'groundReflection',
  'room',
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

/** Speed the engine presets idle at, rev/min, but for any that sets its own. */
export const PRESET_IDLE_RPM = 800;

/**
 * A preset's operating point: idling in neutral at `PRESET_IDLE_RPM`, with the throttle shut and the
 * idle air valve holding the speed (`idleRpm`). The engine starts at the idle speed too, so it does not
 * have to settle there from somewhere else.
 */
const IDLING: Pick<EngineSpec, 'rpm' | 'idleRpm' | 'load' | 'throttle'> = {
  rpm: PRESET_IDLE_RPM,
  idleRpm: PRESET_IDLE_RPM,
  load: 0,
  throttle: 0,
};

/**
 * The 659 cc Superquadro Mono of the Ducati Hypermotard 698 Mono, a single taken from the 1299 Panigale's
 * twin: 116.0 x 62.4 mm, 13.1:1, desmodromic valves, four of them, and a 10,250 rpm limit, rated at 77.5 hp
 * (57 kW) at 9750 rpm and 63 N*m at 8000.
 */
const DUCATI_SUPERQUADRO_MONO: Partial<EngineSpec> = {
  cylinders: 1,
  exhaustLayout: 'single',
  ...IDLING,
  // Its own idle, as a big single's has to be: at 800 rpm one cylinder has too little to carry it from one
  // firing to the next.
  rpm: 1700,
  idleRpm: 1700,
  // Its limiter, as published. Desmodromic valves cannot float.
  revLimit: 10250,
  // Estimated, and light: a heavier crank holds the flare it starts with longer before it settles to its idle.
  flywheelInertia: 0.1,
  pipeCellSize: 0.035,
  bore: 0.116,
  stroke: 0.0624,
  // Estimated: its published figures do not include the rod.
  rodLength: 0.12,
  compressionRatio: 13.1,
  // The 1299 Panigale's 46.8 mm titanium intakes and 38.2 mm steel exhausts, two of each.
  exValveDia: 0.0382,
  exValveCount: 2,
  inValveDia: 0.0468,
  inValveCount: 2,
  // Estimated, like the runner, the plenum and the exhaust, and tuned with them for the rated torque at
  // 8000 rpm and power at 9750. The cams overlap 27 degrees: more makes more at the top end, but one
  // cylinder at 800 rpm has nothing to carry it through a cycle the exhaust it pushes back dilutes.
  maxLift: 0.0142,
  evo: 114,
  evc: 386,
  ivo: 359,
  ivc: 609,
  intakeRunnerLength: 0.303,
  intakeRunnerDia: 0.058,
  // Its 62 mm throttle body, into a plenum two and a half times the size the app would give it, for the
  // airbox it draws from: with the smaller one it makes a fifth less torque at the top end.
  throttleDia: 0.062,
  plenumVolume: 0.0025,
  recipMass: 0.7,
  // Level-matched to the inline four, as the other presets are.
  outputGain: 0.44,
};

/**
 * The 1977 cc Milwaukee-Eight VVT 121 of the Harley-Davidson CVO Road Glide and Street Glide: 103.5 x
 * 117.5 mm, 11.4:1, a 45-degree V-twin on a shared crankpin, one cam in the block working four valves a
 * cylinder through pushrods, with variable timing on it, rated at 115 hp at 5020 rpm and 189 N*m
 * (139 lb-ft) at 3500.
 */
const HARLEY_M8_121: Partial<EngineSpec> = {
  cylinders: 2,
  vAngle: 45,
  firingOffset: null,
  exhaustLayout: '2into1',
  exhaustHeaders: true,
  headerRun: 'lengthways',
  ...IDLING,
  // Estimated, as a big pushrod twin's: it makes its power at 5000 and is out of breath soon after.
  revLimit: 5600,
  flywheelInertia: 0.4,
  pipeCellSize: 0.035,
  // 4.075 x 4.625 in.
  bore: 0.1035,
  stroke: 0.1175,
  // Estimated: its published figures do not include the rod.
  rodLength: 0.176,
  compressionRatio: 11.4,
  // Four valves a cylinder, sized as a typical four-valve head's for the bore. Its pushrods are not drawn:
  // the drawing has them only for a two-valve head.
  ...fourValveHead(0.1035),
  // Estimated, like the cam map and the runners, and tuned with them and the headers for the rated torque
  // at 3500 rpm and power at 5020. Under load at low speed its one phaser advances the whole cam 15
  // degrees, intake and exhaust lobes together, easing back to rest by 4500 rpm; with the cam fixed at
  // rest it makes a few N*m less below 3000.
  maxLift: 0.0105,
  evo: 125,
  evc: 385,
  ivo: 350,
  ivc: 595,
  vvtLinked: true,
  vvtIntakeLow: 15,
  vvtLowRpm: 2000,
  vvtHighRpm: 4500,
  intakeRunnerLength: 0.6,
  // Estimated: a 64 mm throttle body, and a heavier piston than the default's for the bigger bore.
  throttleDia: 0.064,
  recipMass: 0.8,
  // Level-matched to the other twins.
  outputGain: 0.55,
};

/**
 * The 999 cc twin of the 2000-01 Honda RC51 (SP-1): 100.0 x 63.6 mm, 10.8:1, a 90-degree vee on a shared
 * crankpin, gear-driven twin cams and four valves a cylinder, rated at 133 hp (99 kW) at 9500 rpm and
 * 105 N*m at 8000.
 */
const HONDA_RC51: Partial<EngineSpec> = {
  cylinders: 2,
  vAngle: 90,
  firingOffset: null,
  exhaustLayout: '2into2',
  ...IDLING,
  // Estimated: a little past its 9500 rpm power peak.
  revLimit: 10000,
  flywheelInertia: 0.2,
  pipeCellSize: 0.035,
  bore: 0.1,
  stroke: 0.0636,
  // Estimated: its published figures do not include the rod.
  rodLength: 0.12,
  compressionRatio: 10.8,
  // 40 mm intakes and 34 mm exhausts, two of each.
  ...fourValveHead(0.1),
  // Estimated, like the runners, the plenum and the exhaust, and tuned with them for the rated torque at
  // 8000 rpm and power at 9500. The cams overlap 39 degrees. More would make more power at the top end,
  // but with no variable timing to take it away at idle, the exhaust it pushes back into the intake there
  // dilutes the charge until the idle hunts and stalls.
  maxLift: 0.0139,
  evo: 111,
  evc: 385,
  ivo: 346,
  ivc: 606,
  intakeRunnerLength: 0.38,
  intakeRunnerDia: 0.057,
  // One 62 mm throttle into a plenum a little bigger than the app would give it, which the real one's
  // throttle bodies do without, drawing straight from the airbox. Bigger still, the two cylinders rob each
  // other of less air at the top end, but the plenum answers the idle valve too slowly and the idle hunts.
  throttleDia: 0.062,
  plenumVolume: 0.0025,
  recipMass: 0.65,
  // Level-matched to the other twins.
  outputGain: 0.44,
};

/**
 * The 1200 cc high-torque twin of the Triumph Speed Twin 1200 and Thruxton: 97.6 x 80.0 mm, 12.1:1, a
 * liquid-cooled parallel twin on a 270-degree crank, one cam working four valves a cylinder, rated at
 * 103.5 hp (77.2 kW) at 7750 rpm and 112.5 N*m (83 lb-ft) at 4250.
 */
const TRIUMPH_1200_HT: Partial<EngineSpec> = {
  cylinders: 2,
  vAngle: 0,
  firingOffset: 270,
  exhaustLayout: '2into2',
  ...IDLING,
  // Estimated: a little past its 7750 rpm power peak.
  revLimit: 8000,
  flywheelInertia: 0.25,
  pipeCellSize: 0.035,
  bore: 0.0976,
  stroke: 0.08,
  // Estimated: its published figures do not include the rod.
  rodLength: 0.13,
  compressionRatio: 12.1,
  // Four valves a cylinder, the intakes a little smaller than a typical four-valve head's for the bore.
  exValveDia: 0.0332,
  exValveCount: 2,
  inValveDia: 0.037,
  inValveCount: 2,
  // Estimated, like the runners and the exhaust, and tuned with them for the rated torque at 4250 rpm and
  // power at 7750. The intake closes late, and long runners ram the charge in at the torque peak; there
  // is no variable timing on its one cam.
  maxLift: 0.0095,
  evo: 120,
  evc: 380,
  ivo: 345,
  ivc: 625,
  intakeRunnerLength: 0.54,
  intakeRunnerDia: 0.054,
  // Its 50 mm throttle body.
  throttleDia: 0.05,
  recipMass: 0.7,
  // Level-matched to the other twins.
  outputGain: 0.54,
};

/**
 * The 1.5 litre EcoBoost "Dragon" of the Mk8 Fiesta ST: 84.0 x 90.0 mm, 9.7:1, and one turbo through an
 * air-to-air intercooler, rated at 200 PS (197 hp) at 6000 rpm and 290 N*m (214 lb-ft) from 1600 rpm.
 */
const FORD_DRAGON: Partial<EngineSpec> = {
  cylinders: 3,
  vAngle: 0,
  exhaustLayout: 'merged',
  ...IDLING,
  // Its fuel cut, a little past the 6500 rpm redline.
  revLimit: 6700,
  flywheelInertia: 0.2,
  pipeCellSize: 0.035,
  bore: 0.084,
  stroke: 0.09,
  // Estimated: its published figures do not include the rod.
  rodLength: 0.145,
  compressionRatio: 9.7,
  ...fourValveHead(0.084),
  // Estimated, like its timing: 236 degrees on the exhaust and 242 on the intake, with little overlap, as
  // a turbo engine's are.
  maxLift: 0.009,
  evo: 126,
  evc: 362,
  ivo: 358,
  ivc: 600,
  // Twin independent variable cam timing, its map estimated: at low speed under load the intake advanced
  // 30 degrees and the exhaust retarded 20, easing back to rest by 6000 rpm. With the cams fixed it makes
  // 200 N*m at 2000 rpm and 270 at 3000, where the turbo is still spooling.
  vvtIntakeLow: 30,
  vvtExhaustLow: 20,
  vvtLowRpm: 2000,
  vvtHighRpm: 6000,
  // Its turbo's boost and size are estimates: no map of its compressor is published.
  boostTarget: 1.15e5,
  turboSize: 0.16,
  intercooler: 0.7,
  // The factory valve recirculates.
  blowOff: 'recirculating',
  // Level-matched to the inline four, as the other presets are.
  outputGain: 2.44,
};

/**
 * The 1.8 litre turbocharged BP of the 2004-05 Mazdaspeed MX-5: 83.0 x 85.0 mm on a 133 mm rod, 9.5:1, and
 * an IHI turbo on 0.5-0.6 bar through an air-to-air intercooler, rated at 178 hp at 6000 rpm and 166 lb-ft
 * (225 N*m) at 4500.
 */
const MAZDA_BPT: Partial<EngineSpec> = {
  cylinders: 4,
  vAngle: 0,
  exhaustLayout: 'merged',
  ...IDLING,
  // Its fuel cut, a little past the 6500 rpm redline.
  revLimit: 6700,
  flywheelInertia: 0.2,
  pipeCellSize: 0.035,
  bore: 0.083,
  stroke: 0.085,
  rodLength: 0.133,
  compressionRatio: 9.5,
  ...fourValveHead(0.083),
  // Estimated, like its timing: 245 degrees on the exhaust and 255 on the intake, with little overlap, as a
  // turbo engine's are, and tuned with the boost and the intercooler for the rated torque at 4500 rpm and
  // power at 6000. The late intake close holds its torque to 6000. No variable timing: the turbo engine
  // kept the earlier head without it.
  maxLift: 0.0091,
  evo: 120,
  evc: 365,
  ivo: 355,
  ivc: 610,
  // Its 55 mm throttle body.
  throttleDia: 0.055,
  // On 0.5 bar, the low end of the 7.25-8.5 psi it is quoted at. Its size is an estimate: no map of its
  // compressor is published.
  boostTarget: 0.5e5,
  turboSize: 0.14,
  // Estimated, like the turbo: the stock core is a small one.
  intercooler: 0.4,
  // The factory valve recirculates.
  blowOff: 'recirculating',
  // Level-matched to the inline four, as the other presets are.
  outputGain: 1.6,
};

/**
 * The 2.0 litre F20C of the AP1 Honda S2000: 87.0 x 84.0 mm on a 153 mm rod, 11.0:1, VTEC on both cams, a
 * 9000 rpm redline, rated at 240 hp at 8300 rpm and 210 N*m at 7500.
 */
const HONDA_F20C: Partial<EngineSpec> = {
  cylinders: 4,
  vAngle: 0,
  exhaustLayout: 'merged',
  exhaustHeaders: true,
  headerRun: 'lengthways',
  ...IDLING,
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
  exValveDia: 0.032,
  exValveCount: 2,
  inValveDia: 0.0376,
  inValveCount: 2,
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
  highEvo: 108,
  highEvc: 392,
  highIvo: 328,
  highIvc: 625,
  intakeRunnerLength: 0.33,
  intakeRunnerDia: 0.04,
  // Level-matched to the single at idle, and the other presets to this: RMS over two seconds, each at its
  // own rpm.
  outputGain: 1.31,
};

/**
 * The 2.0 litre 3S-GTE of the SW20 MR2 Turbo, in its second generation, 1990-1993, as Japan had it:
 * 86.0 x 86.0 mm, 8.8:1, and a twin-entry CT26 turbo on 0.7 bar through an air-to-air intercooler,
 * rated at 225 PS at 6000 rpm and 304 N*m at 3200. North America's, on the same boost, was rated at
 * 200 hp and 271 N*m.
 */
const TOYOTA_3SGTE: Partial<EngineSpec> = {
  cylinders: 4,
  vAngle: 0,
  exhaustLayout: 'merged',
  ...IDLING,
  // Its fuel cut, a little past the 7000 rpm redline.
  revLimit: 7200,
  flywheelInertia: 0.25,
  pipeCellSize: 0.035,
  bore: 0.086,
  stroke: 0.086,
  // Estimated: the 3S's rod is quoted at 145-146 mm.
  rodLength: 0.1455,
  compressionRatio: 8.8,
  ...fourValveHead(0.086),
  // 8.2 mm of intake lift, as published. Its timing is estimated: 236 degrees on each cam, with little
  // overlap, as a turbo engine's are.
  maxLift: 0.0082,
  evo: 126,
  evc: 362,
  ivo: 354,
  ivc: 590,
  // The CT26 on 0.7 bar, the low end of the 0.69-0.76 bar it is quoted at. Its size is an estimate: no
  // map of its compressor is published.
  boostTarget: 0.7e5,
  turboSize: 0.3,
  intercooler: 0.7,
  // Estimated: most factory valves recirculate.
  blowOff: 'recirculating',
  // Level-matched to the inline four, as the other presets are.
  outputGain: 1.63,
};

/**
 * The 2.5 litre EA855 EVO of the 8V Audi RS 3, 2017-2020: 82.5 x 92.8 mm, 10.0:1, and one turbo on
 * 1.35 bar through an air-to-air intercooler, rated at 400 PS from 5850 to 7000 rpm and 480 N*m from
 * 1700 to 5850.
 */
const AUDI_EA855_EVO: Partial<EngineSpec> = {
  cylinders: 5,
  vAngle: 0,
  exhaustLayout: 'merged',
  ...IDLING,
  // Its fuel cut, a little past the 7000 rpm redline.
  revLimit: 7200,
  flywheelInertia: 0.3,
  pipeCellSize: 0.035,
  bore: 0.0825,
  stroke: 0.0928,
  rodLength: 0.144,
  compressionRatio: 10,
  ...fourValveHead(0.0825),
  // Estimated, like its timing: 236 degrees on the exhaust and 242 on the intake, with little overlap, as
  // a turbo engine's are. The later intake close holds its power to the limiter.
  maxLift: 0.0105,
  evo: 126,
  evc: 362,
  ivo: 358,
  ivc: 600,
  // Its turbo on 1.35 bar, its compressor passing 0.32 kg/s at full speed.
  boostTarget: 1.35e5,
  turboSize: 0.32,
  intercooler: 0.7,
  // The factory valve recirculates.
  blowOff: 'recirculating',
  // Level-matched to the inline four, as the other presets are.
  outputGain: 1.24,
};

/**
 * The 2.6 litre RB26DETT of the R32, R33 and R34 Skyline GT-R: 86.0 x 73.7 mm on a 121.5 mm rod, 8.5:1, and
 * two turbos on 0.7 bar through an intercooler, rated at 280 PS and 368 N*m.
 */
const NISSAN_RB26: Partial<EngineSpec> = {
  cylinders: 6,
  vAngle: 0,
  exhaustLayout: 'merged',
  ...IDLING,
  // Its fuel cut, a little past the 8000 rpm redline.
  revLimit: 8200,
  flywheelInertia: 0.3,
  pipeCellSize: 0.035,
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
  // Estimated, like the turbos: the stock core is a small one in front of the radiator.
  intercooler: 0.6,
  // The factory valve recirculates; this is the atmospheric one so many are fitted with instead.
  blowOff: 'atmospheric',
  // Level-matched to the inline four, as the other presets are.
  outputGain: 1.23,
};

/**
 * The 8.3 litre Cummins 6CT, the turbocharged C-series of medium-duty trucks, buses and boats: 114 x 135 mm,
 * 16.5:1, two valves a cylinder on pushrods, direct injection from a mechanical pump, and a Holset turbo
 * with no aftercooler, rated in its truck tunes at about 250 hp at 2200 rpm and 920 N*m (680 lb-ft) at 1500.
 */
const CUMMINS_6CT: Partial<EngineSpec> = {
  cylinders: 6,
  vAngle: 0,
  exhaustLayout: 'merged',
  fuel: 'diesel',
  ...IDLING,
  // Its governor's high idle: the fuel is all gone by 2500, full to the rated 2200.
  revLimit: 2500,
  // A truck's flywheel and clutch.
  flywheelInertia: 1.6,
  // Its piston, pin and the small end of its rod: a diesel's, heavy for its compression.
  recipMass: 3.2,
  pipeCellSize: 0.035,
  bore: 0.114,
  stroke: 0.135,
  // Estimated.
  rodLength: 0.216,
  compressionRatio: 16.5,
  // Pushrods, one intake and one exhaust valve a cylinder, about 46 and 41 mm: estimated.
  pushrods: true,
  exValveDia: 0.041,
  inValveDia: 0.046,
  // Estimated: a truck cam, short and with little overlap, for torque low down.
  maxLift: 0.012,
  evo: 128,
  evc: 370,
  ivo: 350,
  ivc: 570,
  // Injection 12 degrees before top dead centre, fixed, as a mechanical pump's static timing is, and a
  // diffusion burn of about 55 degrees at full fuel.
  ignition: 708,
  burnDuration: 55,
  // Fuelled to its rated torque at the pump's full delivery, and never richer than λ 1.5 below that, as
  // its boost compensator holds it until the turbo is up.
  maxFuel: 1.1e-4,
  smokeLambda: 1.5,
  // The realistic amount: its ignition delay and premixed share wander from cycle to cycle, as a real
  // diesel's spray does, so no two firings clatter alike.
  combustionVariability: 1,
  // A Holset H1C on about 1.3 bar, wastegated, with no aftercooler and no blow-off valve: there is no
  // throttle to shut against it. Its size is an estimate.
  boostTarget: 1.3e5,
  turboSize: 0.42,
  intercooler: 0,
  blowOff: 'none',
  // Level-matched to the inline four, as the other presets are.
  outputGain: 1.16,
};

/**
 * The 3.5 litre 2GR-FE of the Lotus Evora and half of Toyota's range: 94.0 x 83.0 mm, 10.8:1, a 60-degree
 * vee on a split-pin crank, and variable timing on the intake cam.
 */
const TOYOTA_2GR: Partial<EngineSpec> = {
  cylinders: 6,
  vAngle: 60,
  exhaustLayout: 'perBank',
  ...IDLING,
  // Its fuel cut.
  revLimit: 6600,
  flywheelInertia: 0.5,
  pipeCellSize: 0.035,
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
 * The 6.2 litre LT2 small-block of the C8 Corvette Stingray: 103.25 x 92.0 mm on a 155.6 mm rod, 11.5:1,
 * pushrods and two valves a cylinder, rated with the Z51 package at 495 hp at 6450 rpm and 637 N*m
 * (470 lb-ft) at 5150.
 */
const CHEVROLET_LT2: Partial<EngineSpec> = {
  cylinders: 8,
  vAngle: 90,
  crankType: 'crossplane',
  exhaustLayout: 'perBank',
  exhaustHeaders: true,
  headerRun: 'lengthways',
  ...IDLING,
  revLimit: 6600,
  // The crank, flexplate and dual clutch's input: the gearbox has no flywheel of its own.
  flywheelInertia: 0.4,
  pipeCellSize: 0.035,
  // 4.065 x 3.622 in on a 6.125 in rod.
  bore: 0.10325,
  stroke: 0.092,
  rodLength: 0.1556,
  compressionRatio: 11.5,
  // Pushrods, and 2.13 and 1.59 in valves, one of each.
  pushrods: true,
  exValveDia: 0.0404,
  inValveDia: 0.054,
  // Estimated: about 290 degrees on the exhaust and 265 on the intake, the intake closing early enough
  // for the torque peak at 5150 rpm, and lift enough to carry the power on to 6450. Tuned with the
  // runners for GM's figures.
  maxLift: 0.0165,
  evo: 104,
  evc: 392,
  ivo: 338,
  ivc: 602,
  // Estimated: tuned for the torque peak at 5150 rpm, and opened out from the valves' 49 mm so the
  // top end breathes.
  intakeRunnerLength: 0.44,
  intakeRunnerDia: 0.056,
  throttleDia: 0.087,
  // Level-matched to the inline four, as the other presets are.
  outputGain: 2.1,
};

/**
 * The 5.5 litre LT6 of the C8 Corvette Z06: 104.25 x 80.0 mm, 12.5:1, a flat-plane crank, four cams and
 * four valves a cylinder, an 8600 rpm limit, rated at 670 hp at 8400 rpm and 624 N*m at 6300.
 */
const CHEVROLET_LT6: Partial<EngineSpec> = {
  cylinders: 8,
  vAngle: 90,
  crankType: 'flatplane',
  exhaustLayout: 'perBank',
  exhaustHeaders: true,
  headerRun: 'lengthways',
  ...IDLING,
  revLimit: 8600,
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
  // to ram the charge in after bottom dead centre at the top end. Tuned with them, the plenums and the
  // cam map for power at 8400 and the flattest curve below it. These are the cams at rest, as they
  // idle, with 15 degrees less overlap than the phasers give them under load (below): at rest with
  // 70 degrees of it, the exhaust it pushes back up the runners at idle dilutes the charge until the
  // idle hunts and stalls.
  maxLift: 0.015,
  evo: 87,
  evc: 372,
  ivo: 342,
  ivc: 635,
  // Estimated: a two-stage manifold. The short runners are tuned for 8400 rpm; the long ones, 80 mm
  // longer, lift the mid-range from 6500 to 7500 rpm and fall behind above that, where it switches.
  intakeRunnerLength: 0.39,
  intakeRunnerShortLength: 0.31,
  intakeSwitchRpm: 7600,
  intakeRunnerDia: 0.06,
  // Dual plenums, one for each bank, as the real manifold has, each with an 87 mm throttle body and an
  // inlet tract of its own, the casting as wide as the two throttle bodies' flanges. Estimated: the
  // balance valves, joined from 3500 to 5700 rpm and again from 7500, where one box fills the cylinders
  // better; from 5700 to 7500 each bank's plenum ringing with its own pulses does, by up to 40 N·m. A
  // bigger plenum makes a few more horsepower at the top, but idles worse: its air answers the idle
  // valve more slowly.
  dualPlenum: true,
  throttleDia: 0.087,
  plenumWidth: 0.266,
  plenumTaper: 0.2,
  plenumBalanceRpm: 3500,
  plenumBalanceShutRpm: 5700,
  plenumBalanceReopenRpm: 7500,
  // Estimated, like the cams: under load the exhaust retarded 25 degrees, and the intake advanced 40 up
  // to 4550 rpm, easing back to 15 by 7750, which gives back the mid-range a cam tuned for 8400 costs it;
  // at idle and light load both at rest.
  vvtIntakeLow: 40,
  vvtIntakeHigh: 15,
  vvtExhaustLow: 25,
  vvtExhaustHigh: 25,
  vvtLowRpm: 4550,
  vvtHighRpm: 7750,
  // Level-matched to the inline four, as the other presets are.
  outputGain: 0.93,
};

/**
 * The 2.0 litre FA20D of the Toyota 86, Scion FR-S and Subaru BRZ, 2012-2020, which Toyota calls the
 * 4U-GSE: 86.0 x 86.0 mm, 12.5:1, a boxer with twin cams and four valves a cylinder, variable timing on
 * all four cams, and a 7400 rpm redline, rated at 200 hp (147 kW) at 7000 rpm and 205 N*m (151 lb-ft) at
 * 6400-6600.
 */
const SUBARU_FA20D: Partial<EngineSpec> = {
  cylinders: 4,
  vAngle: 180,
  crankType: 'boxer',
  exhaustLayout: 'merged',
  exhaustHeaders: true,
  headerRun: 'lengthways',
  ...IDLING,
  // Its fuel cut, a little past the redline.
  revLimit: 7500,
  flywheelInertia: 0.25,
  pipeCellSize: 0.035,
  bore: 0.086,
  stroke: 0.086,
  // Estimated: its published figures do not include the rod.
  rodLength: 0.1305,
  compressionRatio: 12.5,
  ...fourValveHead(0.086),
  // Estimated, like the runners and the cam map, and tuned with them and the exhaust for the rated torque
  // at 6400-6600 rpm and power at 7000.
  maxLift: 0.0105,
  evo: 124,
  evc: 372,
  ivo: 348,
  ivc: 595,
  // Under load at low speed the intake advanced 30 degrees and the exhaust retarded 20, easing back to rest
  // by 6000 rpm; at idle and light load both at rest.
  vvtIntakeLow: 30,
  vvtExhaustLow: 20,
  vvtLowRpm: 2000,
  vvtHighRpm: 6000,
  intakeRunnerLength: 0.37,
  intakeRunnerDia: 0.046,
  // Its 65 mm throttle body.
  throttleDia: 0.065,
  // Level-matched to the inline four, as the other presets are.
  outputGain: 0.88,
};

/**
 * The 4.0 litre Mezger flat six of the 2011 Porsche 911 GT3 RS 4.0, the last of the engines descended from
 * the GT1's: 102.7 x 80.4 mm, the stroke from the GT3 RSR's crank, 12.6:1, dry-sumped, with twin cams and
 * four valves a cylinder on each bank and variable timing on all four cams, an 8500 rpm limit, rated at
 * 500 PS (493 hp, 368 kW) at 8250 rpm and 460 N*m (339 lb-ft) at 5750.
 */
const PORSCHE_MEZGER_40: Partial<EngineSpec> = {
  cylinders: 6,
  vAngle: 180,
  crankType: 'boxer',
  exhaustLayout: 'perBank',
  exhaustHeaders: true,
  headerRun: 'lengthways',
  ...IDLING,
  revLimit: 8500,
  flywheelInertia: 0.3,
  pipeCellSize: 0.035,
  bore: 0.1027,
  stroke: 0.0804,
  // Estimated: its published figures do not include the rod.
  rodLength: 0.127,
  compressionRatio: 12.6,
  // Estimated, like the cams, the runners and the cam map, and tuned with them for the rated torque at
  // 5750 rpm and power at 8250. Valves larger than a typical four-valve head's for the bore: with a
  // typical head's, it makes 478 PS at 8250 and 9 N*m less at 5750.
  exValveDia: 0.037,
  exValveCount: 2,
  inValveDia: 0.044,
  inValveCount: 2,
  maxLift: 0.0135,
  evo: 100,
  evc: 380,
  ivo: 340,
  ivc: 625,
  // Under load up to 5000 rpm the intake advanced 40 degrees, easing back to rest by 7500, which gives back
  // the mid-range a cam closing this late costs it; at idle and light load at rest. The exhaust cam stays
  // at rest: retarded too, the overlap blows fresh charge through into the headers, where it lights.
  vvtIntakeLow: 40,
  vvtLowRpm: 5000,
  vvtHighRpm: 7500,
  // At 62 mm they make 514 PS at 8250 and 4 N*m less at 5750.
  intakeRunnerLength: 0.35,
  intakeRunnerDia: 0.055,
  // Its 82 mm throttle body.
  throttleDia: 0.082,
  // Level-matched to the inline four, as the other presets are.
  outputGain: 1.03,
};

export const ENGINE_PRESETS: EnginePreset[] = [
  {
    name: 'Single, Ducati Superquadro Mono',
    car: {
      // Ducati's figures for the Hypermotard 698 Mono: the six-speed, a 61/31 primary and a 15/43 chain,
      // 5.64 in all. A 160/60ZR17 rear tyre, 160 kg with everything but its fuel.
      name: 'Ducati Hypermotard 698 Mono',
      ratios: [36 / 13, 35 / 17, 32 / 20, 29 / 22, 24 / 21, 26 / 25],
      finalDrive: (61 / 31) * (43 / 15),
      tyreRadius: 0.31,
      tyreGrip: TYRE_GRIP.road,
      mass: 160 + DRIVER_MASS,
      drive: 'rwd',
      drivenLoad: 0.5,
      shiftTime: MANUAL_SHIFT_TIME,
      dualClutch: false,
    },
    description:
      'The 659 cc single in the Ducati Hypermotard 698 Mono, the front cylinder of the 1299 Panigale\u2019s twin made into an engine of its own: a 116 mm bore on a 62.4 mm stroke, 13.1:1, four desmodromic valves and a 10,250 rpm limit. One cylinder fires once every two turns of the crank, so the loudest order is the half order, and the big piston shakes the crank between firings. It makes 54-61 N\u00b7m from 4000 to 10,000 rpm, 58 N\u00b7m at 8000 and 79 hp at 9750, against the real engine\u2019s rated 63 N\u00b7m at 8000 and 77.5 hp at 9750. Its header runs round under the engine into one flared silencer, where the real one splits into two; its rod, cams, runner and plenum are estimates.',
    engine: DUCATI_SUPERQUADRO_MONO,
    // What a change of layout compiles a fresh exhaust from: a header into a silencer.
    pipe: () => [
      makeSegment({ kind: 'pipe', length: 0.55, dIn: 0.053 }),
      makeSegment({ kind: 'cone', length: 0.1, dIn: 0.053, dOut: 0.063 }),
      makeSegment({ kind: 'chamber', length: 0.4, dIn: 0.063, dOut: 0.12 }),
      makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.06 }),
    ],
    // Drawn in the editor: the header down and round under the engine, on through a short pipe into a
    // flared silencer.
    graph: () => structuredClone(ducatiMonoExhaust) as ExhaustGraph,
  },
  {
    name: '45\u00b0 V-twin, Harley-Davidson Milwaukee-Eight 121',
    car: {
      // Harley-Davidson's figures for the CVO Road Glide: the six-speed Cruise Drive, sixth direct, and a
      // 46/34 primary chain and 68/32 belt, 2.875 in all. A 180/55B18 rear tyre, 391 kg in running order.
      name: 'Harley-Davidson CVO Road Glide',
      ratios: [9.593, 6.65, 4.938, 4.0, 3.407, 2.875].map((r) => r / 2.875),
      finalDrive: (46 / 34) * (68 / 32),
      tyreRadius: 0.325,
      tyreGrip: TYRE_GRIP.road,
      mass: 391 + DRIVER_MASS,
      drive: 'rwd',
      drivenLoad: 0.55,
      shiftTime: MANUAL_SHIFT_TIME,
      dualClutch: false,
    },
    description:
      'The 121 cubic inch (1977 cc) twin in the CVO Road Glide and Street Glide: 103.5 x 117.5 mm, 11.4:1, a 45\u00b0 vee on a shared crankpin, so it fires 405/315 like every big Harley, and one cam in the block working four valves a cylinder through pushrods, with variable timing on that cam. Each cylinder\u2019s header runs to a straight pipe of its own, with no silencer. It makes 177-199 N\u00b7m from 1750 to 4500 rpm, 193 N\u00b7m (142 lb\u00b7ft) at 3500 and 117 hp at 5000, against the real engine\u2019s rated 189 N\u00b7m (139 lb\u00b7ft) at 3500 and 115 hp at 5020. Its rod, valves, cam, cam map and runners are estimates.',
    engine: HARLEY_M8_121,
    // What a change of layout compiles a fresh exhaust from: 1-3/4 in headers into one collector and a
    // 4 in silencer.
    pipe: () => [makeSegment({ kind: 'pipe', length: 0.5, dIn: 0.045 })],
    collector: () => [
      makeSegment({ kind: 'cone', length: 0.12, dIn: 0.05, dOut: 0.057 }),
      makeSegment({ kind: 'pipe', length: 0.35, dIn: 0.057 }),
      makeSegment({ kind: 'chamber', length: 0.5, dIn: 0.057, dOut: 0.1 }),
      makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.05 }),
    ],
    // Drawn in the editor: each cylinder's header out to a pipe of its own, open at the back.
    graph: () => structuredClone(harleyM8Exhaust) as ExhaustGraph,
  },
  {
    name: '90\u00b0 V-twin, Honda RC51',
    car: {
      // Honda's figures for the 2000-01 RC51 (SP-1): the six-speed, a 68/40 primary and a 40/16 chain,
      // 4.25 in all. A 190/50ZR17 rear tyre, 223 kg at the kerb.
      name: 'Honda RC51 (SP-1)',
      ratios: [2.461, 1.812, 1.428, 1.24, 1.08, 0.962],
      finalDrive: (68 / 40) * (40 / 16),
      tyreRadius: 0.311,
      tyreGrip: TYRE_GRIP.road,
      mass: 223 + DRIVER_MASS,
      drive: 'rwd',
      drivenLoad: 0.5,
      shiftTime: MANUAL_SHIFT_TIME,
      dualClutch: false,
    },
    description:
      'The 999 cc twin in the Honda RC51, Honda\u2019s World Superbike homologation special: 100 x 63.6 mm, 10.8:1, a 90\u00b0 vee on a shared crankpin, so it fires 450/270 like a Ducati, with gear-driven twin cams, four valves a cylinder and a 10,000 rpm limit. Both headers meet under the engine in one short collector. It makes 93-105 N\u00b7m from 5500 to 9750 rpm, 97 N\u00b7m at 8000 and 127 hp at 9500, against the real engine\u2019s rated 105 N\u00b7m at 8000 and 133 hp at 9500. Its rod, cams, runners and plenum are estimates.',
    engine: HONDA_RC51,
    // What a change of layout compiles a fresh exhaust from: each cylinder's header into a silencer of its
    // own.
    pipe: () => [
      makeSegment({ kind: 'pipe', length: 0.7, dIn: 0.05 }),
      makeSegment({ kind: 'cone', length: 0.1, dIn: 0.05, dOut: 0.058 }),
      makeSegment({ kind: 'chamber', length: 0.4, dIn: 0.058, dOut: 0.12 }),
      makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.058 }),
    ],
    // Drawn in the editor: both headers bent round to meet under the engine, into one short collector.
    graph: () => structuredClone(hondaRc51Exhaust) as ExhaustGraph,
  },
  {
    name: 'Parallel twin, Triumph 1200 HT',
    car: {
      // Triumph's figures for the Speed Twin 1200: the six-speed, a 72/41 primary and a 43/18 chain. A
      // 160/60ZR17 rear tyre, 216 kg wet.
      name: 'Triumph Speed Twin 1200',
      ratios: [2.583, 1.842, 1.38, 1.13, 0.966, 0.81],
      finalDrive: (72 / 41) * (43 / 18),
      tyreRadius: 0.31,
      tyreGrip: TYRE_GRIP.road,
      mass: 216 + DRIVER_MASS,
      drive: 'rwd',
      drivenLoad: 0.5,
      shiftTime: MANUAL_SHIFT_TIME,
      dualClutch: false,
    },
    description:
      'The 1200 cc high-torque twin in the Triumph Speed Twin 1200 and Thruxton: 97.6 x 80 mm, 12.1:1, liquid-cooled, one cam working four valves a cylinder, and a 270\u00b0 crank, so it fires 270/450 like a 90\u00b0 V-twin and has the same lopsided beat. Each cylinder\u2019s header runs back to a silencer of its own. It makes 113 N\u00b7m at 4250 rpm and 104 hp at 7500, against the real engine\u2019s rated 112.5 N\u00b7m (83 lb\u00b7ft) at 4250 and 103.5 hp at 7750. It breathes through its 50 mm throttle body; its rod, valves, cam, runners and exhaust are estimates.',
    engine: TRIUMPH_1200_HT,
    // Estimated: each cylinder's header into a silencer of its own.
    pipe: () => [
      makeSegment({ kind: 'pipe', length: 1.0, dIn: 0.045 }),
      makeSegment({ kind: 'cone', length: 0.1, dIn: 0.045, dOut: 0.055 }),
      makeSegment({ kind: 'chamber', length: 0.45, dIn: 0.055, dOut: 0.1 }),
      makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.05 }),
    ],
  },
  {
    name: 'Inline three, Ford 1.5 EcoBoost Dragon',
    car: {
      // The Mk8 Fiesta ST's six-speed manual and its 3.91 final drive. 205/40R18 tyres, about 1190 kg,
      // front-wheel drive with about 61% of its weight on the front.
      name: 'Ford Fiesta ST (Mk8)',
      ratios: [3.59, 2.19, 1.52, 1.15, 0.92, 0.79],
      finalDrive: 3.91,
      tyreRadius: 0.305,
      tyreGrip: TYRE_GRIP.road,
      mass: 1190 + DRIVER_MASS,
      drive: 'fwd',
      drivenLoad: 0.61,
      shiftTime: MANUAL_SHIFT_TIME,
      dualClutch: false,
    },
    description:
      'The 1.5 litre turbocharged three in the Mk8 Ford Fiesta ST: 84 x 90 mm, 9.7:1, four valves a cylinder and a 6500 rpm redline. It fires every 240\u00b0 on a 120\u00b0 crank, in the order 1-3-2, all three into one turbo on 1.15 bar through an intercooler, with variable timing on both cams. An odd number of cylinders puts the loudest order at one and a half times the crank speed, which is the offbeat thrum of a three. It makes about 290-300 N\u00b7m from 2000 to 4000 rpm and 194 PS at 6000, against the real engine\u2019s rated 290 N\u00b7m from 1600 and 200 PS at 6000; below 2000 its turbo is still spooling. Its rod, cams, cam map, turbo size and exhaust are estimates.',
    engine: FORD_DRAGON,
    pipe: () => fittedExhaust(fullSpec(FORD_DRAGON)).pipe,
    collector: () => fittedExhaust(fullSpec(FORD_DRAGON)).collector,
    turbos: 1,
  },
  {
    name: 'Inline four, Mazda BPT',
    car: {
      // The Mazdaspeed MX-5's six-speed manual and its 4.10 final drive. 205/40R17 tyres, about 1110 kg,
      // with a little under half its weight on the rear.
      name: 'Mazdaspeed MX-5 (NB, 2004-05)',
      ratios: [3.76, 2.27, 1.65, 1.26, 1.0, 0.84],
      finalDrive: 4.1,
      tyreRadius: 0.291,
      tyreGrip: TYRE_GRIP.road,
      mass: 1110 + DRIVER_MASS,
      drive: 'rwd',
      drivenLoad: 0.48,
      shiftTime: MANUAL_SHIFT_TIME,
      dualClutch: false,
    },
    description:
      'The 1.8 litre turbocharged four in the 2004-05 Mazdaspeed MX-5: 83 x 85 mm, 9.5:1, four valves a cylinder and a 6500 rpm redline. It fires every 180\u00b0, 1-3-4-2, like any inline four, all four into one small IHI turbo on 0.5 bar through an intercooler, and draws through a 55 mm throttle body. It makes about 230 N\u00b7m from 3000 to 5000 rpm, 235 N\u00b7m (173 lb\u00b7ft) at 4500 and 180 hp at 6000, against the real engine\u2019s rated 225 N\u00b7m (166 lb\u00b7ft) at 4500 and 178 hp at 6000. Its valves, cams, turbo size, intercooler and exhaust are estimates.',
    engine: MAZDA_BPT,
    pipe: () => fittedExhaust(fullSpec(MAZDA_BPT)).pipe,
    collector: () => fittedExhaust(fullSpec(MAZDA_BPT)).collector,
    turbos: 1,
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
      drive: 'rwd',
      // Its engine behind the front axle puts its weight 50:50.
      drivenLoad: 0.5,
      shiftTime: MANUAL_SHIFT_TIME,
      dualClutch: false,
    },
    description:
      'The 2.0 litre four in the Honda S2000: 87 x 84 mm, 11:1, four valves a cylinder and a 9000 rpm redline. Even 180\u00b0 firing on a flat crank, 1-3-4-2, into equal-length headers: twice the firing frequency of a twin at the same rpm, and no half order at all. It makes 195-200 N\u00b7m from 6000 to 8300 rpm and 228 hp at 8300, against the real engine\u2019s 210 N\u00b7m at 7500 and 240 hp at 8300. Its VTEC switches each valve from a mild cam lobe to a wild one at 5500 rpm. Its valves, cams, runners and exhaust are estimates.',
    engine: HONDA_F20C,
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
    name: 'Inline four, Toyota 3S-GTE',
    car: {
      // The Japanese 1992-93 GT-S hardtop: the E153 five-speed and its 4.285 final drive, 1250 kg,
      // 225/50R15 rear tyres, and grip at the 0.92 g the later cars pull on a skidpad, the middle of the
      // 0.90-0.94 quoted, raised for grip driving rather than cornering. Mid-engined, 44:56.
      name: 'Toyota MR2 GT-S (SW20, 1992-93)',
      ratios: [3.23, 1.913, 1.258, 0.918, 0.731],
      finalDrive: 4.285,
      tyreRadius: 0.295,
      tyreGrip: 0.92 * SKIDPAD_TO_DRIVE,
      mass: 1250 + DRIVER_MASS,
      drive: 'rwd',
      drivenLoad: 0.56,
      shiftTime: MANUAL_SHIFT_TIME,
      dualClutch: false,
    },
    description:
      'The 2.0 litre turbocharged four in the SW20 MR2 Turbo and the ST185 Celica GT-Four: 86 x 86 mm, 8.8:1, four valves a cylinder and a 7000 rpm redline. It fires every 180\u00b0, 1-3-4-2, like any inline four, all four into one twin-entry CT26 turbo through an intercooler, here on 0.5 bar against the 0.7 of the standard car. It makes about 265 N\u00b7m from 2500 to 5000 rpm and 201 hp at 6000, close to the North American engine\u2019s rated 271 N\u00b7m at 3200 and 200 hp at 6000. Its cams, turbo size and exhaust are estimates, and its four runners share one turbine entry where the real manifold pairs them into two.',
    engine: TOYOTA_3SGTE,
    pipe: () => fittedExhaust(fullSpec(TOYOTA_3SGTE)).pipe,
    collector: () => fittedExhaust(fullSpec(TOYOTA_3SGTE)).collector,
    turbos: 1,
  },
  {
    name: 'Inline five, Audi EA855 EVO',
    car: {
      // Audi's figures for the 8V RS 3 Sportback's seven-speed S tronic: a 4.059 final drive for first,
      // fourth and fifth and 3.450 for second, third, sixth and seventh, which is folded into those
      // ratios. 235/35R19 tyres, 1510 kg, through quattro to all four wheels.
      name: 'Audi RS 3 Sportback (8V, 2017-2020)',
      ratios: [3.56, 2.53 * (3.45 / 4.059), 1.68 * (3.45 / 4.059), 1.02, 0.79, 0.76 * (3.45 / 4.059), 0.63 * (3.45 / 4.059)],
      finalDrive: 4.059,
      tyreRadius: 0.323,
      tyreGrip: TYRE_GRIP.road,
      mass: 1510 + DRIVER_MASS,
      drive: 'awd',
      drivenLoad: DRIVEN_LOAD.awd,
      shiftTime: 0.1,
      dualClutch: true,
    },
    description:
      'The 2.5 litre turbocharged five in the 8V Audi RS 3 and the TT RS: 82.5 x 92.8 mm, 10:1, four valves a cylinder and a 7000 rpm redline. It fires every 144\u00b0 on a 72\u00b0 crank, 1-2-4-5-3, all five into one turbo on 1.35 bar through an intercooler: the warble of the Audi five, and a whistle over it as the turbo spools. It makes about 500 N\u00b7m from 3500 to 4500 rpm and 403 PS at 5850, holding 387 to 7000, against the real engine\u2019s rated 480 N\u00b7m from 1700 to 5850 and 400 PS from 5850 to 7000. Its cams and exhaust are estimates.',
    engine: AUDI_EA855_EVO,
    pipe: () => fittedExhaust(fullSpec(AUDI_EA855_EVO)).pipe,
    collector: () => fittedExhaust(fullSpec(AUDI_EA855_EVO)).collector,
    turbos: 1,
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
      drive: 'awd',
      drivenLoad: DRIVEN_LOAD.awd,
      shiftTime: MANUAL_SHIFT_TIME,
      dualClutch: false,
    },
    description:
      'The 2.6 litre twin-turbo six in the R32, R33 and R34 Skyline GT-R: 86 x 73.7 mm, 8.5:1, four valves a cylinder and an 8000 rpm redline. It fires every 120\u00b0, 1-5-3-6-2-4, its throws paired 1-6, 2-5 and 3-4: perfectly balanced and evenly fired, so the smooth, silky one. Two small turbos on 0.7 bar, one for each three cylinders, spool from 2000 rpm, on full boost by 3000, and whistle as they do, and every exhaust pulse passes through their turbines, which take the edge off the note; lift off on boost and the blow-off valve vents with a hiss, or with it set to none the compressors surge and flutter. It makes 395 N\u00b7m at 4400 rpm and 296 PS at 6800, about 292 hp, against the real engine\u2019s 368 N\u00b7m and a rated 280 PS. It has one throttle into a plenum where the real one has six individual throttle bodies, and its turbo sizes and exhaust are estimates.',
    engine: NISSAN_RB26,
    pipe: () => fittedExhaust(fullSpec(NISSAN_RB26)).pipe,
    collector: () => fittedExhaust(fullSpec(NISSAN_RB26)).collector,
    // Drawn in the editor: each half's three ports into a turbo, their outlets meeting behind them.
    graph: () => structuredClone(nissanRb26Exhaust) as ExhaustGraph,
    turbos: 2,
  },
  {
    name: 'Inline six diesel, Cummins 6CT',
    car: {
      // The 6CT never came in a pickup, but it is the swap for one: a first-generation Dodge Ram 2500 with
      // the NV4500 five-speed, 3.54 axle, 235/85R16 tyres and about 2400 kg, a little over half of it on
      // the front.
      name: 'Dodge Ram 2500, 6CT swap (1st gen)',
      ratios: [5.61, 3.04, 1.67, 1.0, 0.75],
      finalDrive: 3.54,
      tyreRadius: 0.385,
      tyreGrip: TYRE_GRIP.road,
      mass: 2400 + DRIVER_MASS,
      drive: 'rwd',
      drivenLoad: 0.45,
      shiftTime: MANUAL_SHIFT_TIME,
      dualClutch: false,
    },
    description:
      'The 8.3 litre turbo diesel six of medium-duty trucks and buses: 114 x 135 mm, 16.5:1, two valves a cylinder on pushrods, and a 2500 rpm governed speed. It fires every 120\u00b0, 1-5-3-6-2-4, all six into one Holset turbo on 1.3 bar with no aftercooler. There is no throttle: the pedal meters the fuel its pump injects 12\u00b0 before top dead centre, and the charge lights by itself once compression has heated it, after an ignition delay. What mixes with the air in that delay burns all at once, which is the diesel\u2019s clatter, loudest at idle; the rest burns as it is injected. It makes about 930 N\u00b7m (685 lb\u00b7ft) at 1500 rpm and 265 hp at 2200, against the truck ratings\u2019 920 N\u00b7m at 1500 and about 250 hp at 2200. Its rod, valves, cams, turbo size and exhaust are estimates.',
    engine: CUMMINS_6CT,
    pipe: () => fittedExhaust(fullSpec(CUMMINS_6CT)).pipe,
    collector: () => fittedExhaust(fullSpec(CUMMINS_6CT)).collector,
    turbos: 1,
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
      drive: 'rwd',
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
      drive: 'rwd',
      // Mid-engined, 40:60; the dual clutch shifts in about a tenth of a second.
      drivenLoad: 0.6,
      shiftTime: 0.1,
      dualClutch: true,
    },
    description:
      'The 6.2 litre small-block in the mid-engine Corvette: pushrods, two big valves a cylinder, 11.5:1 and a cam that closes the intake late, which only pays off because its long intake runners ram the charge in. Tubular headers into a silencer each side, and an 87 mm throttle body. It makes 637 N\u00b7m (470 lb\u00b7ft) at 5150 rpm and 495 hp at 6450, as the real engine does.',
    engine: CHEVROLET_LT2,
    // Tubular headers, 1-3/4 in primaries, run lengthways, each opening out to the collector's bore in its
    // bend as it is drawn. Estimated: 850 mm, tuned with the cam and runners for GM's figures on the dyno as
    // the app draws them; at 600 mm it makes 477 hp.
    pipe: () => [makeSegment({ kind: 'pipe', length: 0.85, dIn: 0.044 })],
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
      drive: 'rwd',
      drivenLoad: 0.6,
      shiftTime: 0.1,
      dualClutch: true,
    },
    description:
      'The 5.5 litre flat-plane V8 in the Corvette Z06: four cams, four valves a cylinder, 12.5:1 and an 8600 rpm limit. The flat crank fires each bank evenly every 180\u00b0, so it shrieks like a Ferrari rather than burbling. Its tailpipes exit together in the middle, as the Z06\u2019s do. Rod length, cam and headers are estimates; the published figures are the bore, stroke, compression, valves and limit. It breathes through dual plenums, one for each bank, each with its own 87 mm throttle body and its own airbox and snorkel, mirrored either side, as the real one does, their balance valves joining them from 3500 to 5700 rpm and again from 7500. Its cam, short runners and headers are tuned for the top end, where it makes about 670 hp at 8350 rpm, as the real engine does at 8400. Below that its variable cam timing, long runners and plenums, also estimates, give back the mid-range: 636 N\u00b7m at 6500 against 624 at 6300.',
    engine: CHEVROLET_LT6,
    // Estimated: equal-length headers, their primaries tuned for 8400 rpm.
    pipe: () => [makeSegment({ kind: 'pipe', length: 0.41, dIn: 0.045 })],
    collector: () => [
      makeSegment({ kind: 'cone', length: 0.16, dIn: 0.068, dOut: 0.076 }),
      makeSegment({ kind: 'pipe', length: 0.9, dIn: 0.076 }),
      makeSegment({ kind: 'chamber', length: 0.4, dIn: 0.076, dOut: 0.19 }),
      makeSegment({ kind: 'pipe', length: 0.35, dIn: 0.07 }),
    ],
    // Drawn from them: each bank's collector turned in under the car to the Z06's centre exit, its
    // tailpipes 0.21 m apart.
    graph: () => structuredClone(chevroletLt6Exhaust) as ExhaustGraph,
  },
  {
    name: 'Boxer four, Subaru FA20D',
    car: {
      // Toyota's figures for the ZN6 86 with the six-speed manual: its gears and a 4.100 final drive.
      // 215/45R17 tyres, 1250 kg, its weight 53:47 front to rear.
      name: 'Toyota 86 (ZN6)',
      ratios: [3.626, 2.188, 1.541, 1.213, 1.0, 0.767],
      finalDrive: 4.1,
      tyreRadius: 0.306,
      tyreGrip: TYRE_GRIP.road,
      mass: 1250 + DRIVER_MASS,
      drive: 'rwd',
      drivenLoad: 0.47,
      shiftTime: MANUAL_SHIFT_TIME,
      dualClutch: false,
    },
    description:
      'The 2.0 litre boxer four in the Toyota 86, Scion FR-S and Subaru BRZ, Subaru\u2019s FA20D, which Toyota calls the 4U-GSE: 86 x 86 mm, 12.5:1, four valves a cylinder, variable timing on all four cams and a 7400 rpm redline. Flat, with the pistons of each opposed pair moving out and in together, it fires 1-3-2-4 every 180\u00b0, each bank\u2019s two headers into a collector of its own, the two meeting under the engine in one pipe. It makes 188-207 N\u00b7m from 2500 to 7000 rpm, 205 N\u00b7m at 6500 and 196 hp at 7000, against the real engine\u2019s rated 205 N\u00b7m at 6400-6600 and 200 hp at 7000. Its rod, valves, cams, cam map, runners and exhaust are estimates.',
    engine: SUBARU_FA20D,
    pipe: () => fittedExhaust(fullSpec(SUBARU_FA20D)).pipe,
    collector: () => fittedExhaust(fullSpec(SUBARU_FA20D)).collector,
    // Drawn in the editor: each bank's two headers into a collector of its own, the two meeting under the
    // engine in one pipe to a silencer.
    graph: () => structuredClone(subaruFa20dExhaust) as ExhaustGraph,
  },
  {
    name: 'Boxer six, Porsche Mezger 4.0',
    car: {
      // The 911 GT3 RS 4.0 (997): its six-speed manual and a 3.89 final drive. 325/30ZR19 Michelin Pilot
      // Sport Cup rear tyres, 1370 kg, rear-engined with about 61% of its weight on the rear wheels.
      name: 'Porsche 911 GT3 RS 4.0 (997)',
      ratios: [3.82, 2.15, 1.56, 1.21, 0.97, 0.83],
      finalDrive: 3.89,
      tyreRadius: 0.332,
      tyreGrip: TYRE_GRIP.road,
      mass: 1370 + DRIVER_MASS,
      drive: 'rwd',
      drivenLoad: 0.61,
      shiftTime: MANUAL_SHIFT_TIME,
      dualClutch: false,
    },
    description:
      'The 4.0 litre flat six in the 2011 Porsche 911 GT3 RS 4.0, the last and largest of the Mezger engines descended from the 911 GT1\u2019s: 102.7 x 80.4 mm, the stroke from the GT3 RSR\u2019s crank, 12.6:1, four valves a cylinder, variable timing on its intake and exhaust cams, here on the intakes only, and an 8500 rpm limit. A throw per cylinder, firing 1-6-2-4-3-5 every 120\u00b0; each bank\u2019s three headers gather under it into a pipe and silencer of its own, and each bank hears every other firing, 240\u00b0 apart, as a V6\u2019s banks do. It makes 403-444 N\u00b7m from 4000 to 8400 rpm, 442 N\u00b7m at 5750 and 493 PS (486 hp) at 8250, against the real engine\u2019s rated 460 N\u00b7m at 5750 and 500 PS (493 hp) at 8250. Its rod, valves, cams, cam map, runners and exhaust are estimates.',
    engine: PORSCHE_MEZGER_40,
    pipe: () => fittedExhaust(fullSpec(PORSCHE_MEZGER_40)).pipe,
    collector: () => fittedExhaust(fullSpec(PORSCHE_MEZGER_40)).collector,
    // Drawn in the editor: each bank's three headers into a collector under it, and from each a pipe of its
    // own back to a silencer. The headers flare to 66 mm where they meet: flared to the collector's 83 mm,
    // the gas goes supersonic in the flare at full throttle, faster than one solver step a sample allows.
    graph: () => structuredClone(porscheMezger40Exhaust) as ExhaustGraph,
  },
];

export function defaultConfig(): EngineConfig {
  return {
    engine: { ...DEFAULT_ENGINE },
    pipe: PIPE_PRESETS[1]!.build(),
    collector: defaultCollector(),
  };
}
