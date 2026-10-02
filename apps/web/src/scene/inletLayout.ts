/**
 * Where the intake's parts are drawn: the plenum on the engine, the throttle body on its front, the tube
 * from it up and over into the front of an airbox across the top of the engine's front, and the snorkel from the airbox forwards
 * to just ahead of the engine. Pure geometry, so what is drawn and where the intake is heard from (`soundSources`) agree.
 *
 * The tube, the airbox and the snorkel are the inlet tract the simulation solves (`inletSegments`): the
 * airbox holds the solver's volume over its length, and the tube and snorkel have the solver's bores.
 * The solver is one-dimensional, so how they are routed changes nothing it hears.
 */

import * as THREE from 'three';

import { engineShell, exhaustPortOf, intakeCamsOf, intakePortOf, sharedHead, valvetrainTop } from '../model/geometry.js';
import { airboxVolumeOf, inletSegments, plenumVolumeOf, snorkelDiaOf, throttleDiaOf } from '../model/intakeSizing.js';
import { cylinderZ, intakeRunnerOf, physicalBankCount, type EngineSpec, type PipeSegment } from '../model/spec.js';

export interface Runner {
  /** Where it leaves the plenum, and where it meets the head. */
  from: THREE.Vector3;
  to: THREE.Vector3;
  /** Which way, and how far, it bends away from each end: the curve's handles. */
  leaving: THREE.Vector3;
  arriving: THREE.Vector3;
  radius: number;
  /** Which of the plenum's faces it leaves by: an upright side, a V's sloping flank, or the underside. */
  exit: 'side' | 'flank' | 'bottom';
}

/** Cylinder `b`'s intake port (`intakePortOf`), as vectors. */
function portOf(spec: EngineSpec, b: number): { position: THREE.Vector3; direction: THREE.Vector3 } {
  const port = intakePortOf(spec, b);
  return { position: new THREE.Vector3(...port.position), direction: new THREE.Vector3(...port.direction).normalize() };
}

export interface InletLayout {
  /**
   * The plenum: its middle and its size along x, y and z, m. On a vee it narrows below the throttle body to
   * `base` of its width, a V's flanks parallel to its banks so it sits down into the valley between the
   * heads; beside an inline head it is a rounded box, its `base` 1. It narrows towards the back, as a cast
   * plenum does (`taper`): beside an inline head, its side away from the head, opposite its runners, drawn in;
   * on a V or a boxer, whose runners leave both sides, its top dropping.
   */
  plenum: { centre: THREE.Vector3; size: THREE.Vector3; base: number; taper: PlenumTaper | null };
  /** One runner a cylinder from the plenum into the head: straight on an inline engine, curved on a vee. */
  runners: Runner[];
  /** The throttle body: its middle, on the plenum's front face, its bore and its length along -z, m. */
  throttle: { centre: THREE.Vector3; bore: number; length: number };
  /** The tube from the throttle body to the airbox, and its bore, m. */
  tube: THREE.CubicBezierCurve3;
  tubeRadius: number;
  /** The airbox: its middle and its size, its length along x, m. */
  airbox: { centre: THREE.Vector3; size: THREE.Vector3 };
  /** Which way across the car the airbox runs from the tube: +1 or -1. */
  side: number;
  /** The snorkel from the airbox to its mouth, its cross-section's area, m^2, and its mouth. */
  snorkel: THREE.CubicBezierCurve3;
  snorkelArea: number;
  mouth: THREE.Vector3;
  /** The solver's tract, throttle first, for the cells each part shows. */
  segments: PipeSegment[];
}

/** How high the plenum is at least, m, and how round its edges are. */
const PLENUM_HEIGHT = 0.1;
export const PLENUM_ROUNDING = 0.012;

/** How thick the plenum's casting is, m. */
export const PLENUM_WALL = 0.005;

/** The throttle body's wall, round its bore, m. */
export const THROTTLE_WALL = 0.006;

/** The `t` in `lo..hi` at which `f(t)` is `target`, `f` rising with `t`: by bisection. */
function fit(f: (t: number) => number, target: number, lo: number, hi: number): number {
  for (let i = 0; i < 40; i++) {
    const mid = (lo + hi) / 2;
    if (f(mid) < target) lo = mid;
    else hi = mid;
  }
  return (lo + hi) / 2;
}

/** How far along the airbox from its end the tube goes into its front, beyond the tube's own radius, m. */
const TUBE_INSET = 0.03;
/** How much further the tube reaches forwards over the top of its U than out of the throttle body. */
const TUBE_TOP_REACH = 1.6;

/** How much wider than high the snorkel is flattened to fit under the bonnet, its area kept. */
export const SNORKEL_ASPECT = 1.8;
/** The snorkel's wall, m, and the gap it is left from the airbox's end as it runs past. */
export const SNORKEL_WALL = 0.004;
const SNORKEL_CLEAR = 0.012;

/**
 * How a plenum narrows towards its back, from none at its front: by its side `side`, +1 for +x or -1 for -x,
 * drawn in `inwards`, and by its top dropping `drop`, m, at the back.
 */
export interface PlenumTaper {
  side: number;
  inwards: number;
  drop: number;
}

/** How much of its taper a plenum `length` long has `z` m back from its middle: none at the front, all at the back. */
export function taperAlong(length: number, z: number): number {
  return Math.min(Math.max(z / length + 0.5, 0), 1);
}

/**
 * How far in an inline plenum's outer side is at its back, as a fraction of its width, and how far a V's or
 * a boxer's top drops there, of its height.
 */
const PLENUM_TAPER = 0.45;
const PLENUM_DROP = 0.4;

export function inletLayout(spec: EngineSpec): InletLayout {
  const shell = engineShell(spec);
  const segments = inletSegments(spec);
  const front = -shell.length / 2;
  // Under a shared head the intake ports are all out of one side of it, as an inline engine's are.
  const vee = physicalBankCount(spec) > 1 && !sharedHead(spec);
  const exhaustSide = Math.sign(exhaustPortOf(spec, 0).direction[0]) || 1;
  const intakeSide = vee ? 1 : -exhaustSide;

  // The plenum: across the valley of a V or a boxer, or along the intake side of an inline head. Its
  // footprint holds the plenum's volume at its height, within what the engine leaves room for, and its
  // front face takes the throttle body, wall and all, inside its rounded edges.
  const bore = throttleDiaOf(spec);
  const face = bore + 2 * THROTTLE_WALL + 2 * PLENUM_ROUNDING + 0.01;
  // Most of the engine's length, and at least past every runner: by its flare where it leaves the plenum, so
  // each opening is on the flat of the side, clear of where the ends round over, and a little more.
  const radius = intakeRunnerOf(spec).diameter / 2;
  const zs = Array.from({ length: spec.cylinders }, (_, b) => cylinderZ(spec, b));
  const lastRunner = Math.max(...zs.map(Math.abs)) + radius * 1.5 + PLENUM_ROUNDING + 0.006;
  const length = Math.max(shell.length * 0.85, 2 * lastRunner);
  const height = Math.max(PLENUM_HEIGHT, face);
  // Narrowing towards the back, it holds about as much as a box half its taper less.
  const footprint = plenumVolumeOf(spec) / (height * length * (1 - (vee ? PLENUM_DROP : PLENUM_TAPER) / 2));
  let width = Math.max(Math.min(Math.max(footprint, 0.07), vee ? shell.width * 0.9 : 0.12), face);
  const runners: Runner[] = [];
  let centre: THREE.Vector3;
  // How wide its underside is against its top: a V's narrows to its banks.
  let base = vee ? 0.55 : 1;
  if (vee) {
    const valley = Math.max(shell.top * Math.cos(shell.straddle) * 0.8, shell.crankcase.radius + 0.02);
    const ports = Array.from({ length: spec.cylinders }, (_, b) => portOf(spec, b));
    if (spec.vAngle < 150) {
      // A V's intake ports are on the valley side of its heads, and its runners leave by the plenum's sloping
      // flanks, already heading down and out to them: the plenum sits low, those flanks just clear above the
      // ports, and narrow enough to stand between the intake cams rather than over them.
      const cams = intakeCamsOf(spec);
      width = Math.max(face, Math.min(width, 2 * (cams.x - cams.reach - 0.008)));
      // The flanks run parallel to the banks, so the underside is the V's own shape: each drops from 0.3 of
      // the way down to the underside, 0.7 of the half-height, coming in by that times the half-angle's tan.
      const drop = 0.7 * (height / 2) * Math.tan(((spec.vAngle / 2) * Math.PI) / 180);
      base = Math.min(Math.max(1 - drop / (width / 2), 0.15), 1);
      const [w, h] = [width / 2, height / 2];
      const [wi, hi] = [w - PLENUM_WALL, h - PLENUM_WALL];
      /** The middle of the flank towards `out`, `wide` and `high` the section's half-width and -height. */
      const flankMiddle = (out: number, wide: number, high: number) =>
        new THREE.Vector2(out * wide * (1 + base) * 0.5, -0.65 * high);
      const flankOutward = (out: number) => {
        const slope = new THREE.Vector2(out * w * (base - 1), -0.7 * h);
        const n = new THREE.Vector2(slope.y, -slope.x).normalize();
        return n.x * out < 0 ? n.negate() : n;
      };
      // Where the runners leave it, from wide Vs to tight ones, the first whose runners can reach their ports
      // square: out of its upright sides, the plenum dropped down into the valley between the ports, where
      // they are far enough out beyond its sides; out of its sloping flanks, the flanks just clear above the
      // ports, where they are far enough out in front of those; and otherwise, in a tight V where the ports
      // are nearly under it, out of its underside, the plenum just high enough above the ports for the
      // runners to turn down into them.
      const highest = Math.max(...ports.map((p) => p.position.y));
      const room = 2.2 * radius;
      const sideY = Math.max(valley + h, highest + 1.2 * radius - 0.1 * h);
      const flankY = Math.max(valley + h, highest + 1.6 * radius + 0.65 * h);
      const reaches = (y: number, at: (out: number) => THREE.Vector2, outward: (out: number) => THREE.Vector2) =>
        ports.every((p) => {
          const out = Math.sign(p.position.x) || 1;
          const there = at(out).add(new THREE.Vector2(0, y));
          return new THREE.Vector2(p.position.x, p.position.y).sub(there).dot(outward(out)) > room;
        });
      const sideAt = (out: number, wide: number, high: number) => new THREE.Vector2(out * wide, 0.1 * high);
      const across = (out: number) => new THREE.Vector2(out, 0);
      const exit: Runner['exit'] = reaches(sideY, (out) => sideAt(out, w, h), across)
        ? 'side'
        : reaches(flankY, (out) => flankMiddle(out, w, h), flankOutward)
          ? 'flank'
          : 'bottom';
      centre = new THREE.Vector3(
        0,
        exit === 'side' ? sideY : exit === 'flank' ? flankY : Math.max(valley + h, highest + 2.4 * radius + h),
        0,
      );
      // How far out along the underside a runner can leave it, its flared mouth clear of the rounded edge.
      const underside = wi * base - Math.max(PLENUM_ROUNDING - PLENUM_WALL, 0.002) - 1.5 * radius - 0.002;
      for (const port of ports) {
        const out = Math.sign(port.position.x) || 1;
        const [start, outward] =
          exit === 'side'
            ? [sideAt(out, wi, hi), across(out)]
            : exit === 'flank'
              ? [flankMiddle(out, wi, hi), flankOutward(out)]
              : [new THREE.Vector2(out * Math.max(Math.min(Math.abs(port.position.x), underside), 0), -hi), new THREE.Vector2(0, -1)];
        const from = new THREE.Vector3(centre.x + start.x, centre.y + start.y, port.position.z);
        const reach = Math.max(from.distanceTo(port.position) * 0.45, 1.6 * radius);
        runners.push({
          from,
          to: port.position,
          leaving: new THREE.Vector3(outward.x, outward.y, 0).multiplyScalar(reach),
          arriving: port.direction.clone().multiplyScalar(-reach),
          radius,
          exit,
        });
      }
    } else {
      // A boxer's sits on top between its heads, a rounded box, square below.
      base = 1;
      centre = new THREE.Vector3(0, valley + height / 2, 0);
      // Each runner leaves the plenum's side towards its bank, square to it, and arches down into its port
      // on top of a boxer's head.
      for (const port of ports) {
        const out = Math.sign(port.position.x) || 1;
        // From the plenum's inside wall, low on its side, its flared mouth just clear of the rounded edge
        // below, so the top has room to drop towards the back over them.
        const low = Math.min(-height / 2 + PLENUM_ROUNDING + 1.5 * radius + 0.004, 0);
        const from = new THREE.Vector3(out * (width / 2 - PLENUM_WALL), centre.y + low, port.position.z);
        const reach = Math.max(from.distanceTo(port.position) * 0.45, 2.2 * radius);
        runners.push({
          from,
          to: port.position,
          // Square out of the plenum's upright side, so its mouth lies flat in the wall, and clear of it
          // before it turns down.
          leaving: new THREE.Vector3(out * reach, 0, 0),
          arriving: port.direction.clone().multiplyScalar(-reach),
          radius,
          exit: 'side',
        });
      }
    }
  } else {
    const x = portOf(spec, 0).position.x + intakeSide * (0.08 + width / 2);
    // Straight and level out of the plenum's side into each intake port, at the height the exhaust leaves
    // the other side of the head: each from the plenum's inside wall, so it meets the wall with no gap and
    // leaves the plenum's cavity clear behind the throttle body.
    const portY = portOf(spec, 0).position.y;
    centre = new THREE.Vector3(x, portY, 0);
    for (let b = 0; b < spec.cylinders; b++) {
      const port = portOf(spec, b).position;
      const from = new THREE.Vector3(x - intakeSide * (width / 2 - PLENUM_WALL), port.y, port.z);
      const third = port.clone().sub(from).divideScalar(3);
      runners.push({ from, to: port, leaving: third, arriving: third, radius, exit: 'side' });
    }
  }
  // Beside an inline head, drawn in on its side opposite the runners, but no further than leaves it room
  // inside at the back. On a V or a boxer, its top dropped, but no further than leaves each runner's flared
  // mouth on the flat of the side it leaves by, below where the top rounds over, nor the sides above a V's
  // flanks too short to be sides.
  let taper: PlenumTaper | null = null;
  if (!vee) {
    const inwards = Math.min(PLENUM_TAPER * width, width - 4 * PLENUM_ROUNDING - 2 * PLENUM_WALL);
    if (inwards > 0) taper = { side: intakeSide, inwards, drop: 0 };
  } else {
    const half = height / 2;
    const sides = base < 1 ? -0.3 * half : -half;
    let drop = Math.min(PLENUM_DROP * height, half - sides - 2 * PLENUM_ROUNDING - 0.006);
    for (const r of runners) {
      if (r.exit !== 'side') continue;
      const need = r.from.y - centre.y + 1.5 * r.radius + PLENUM_ROUNDING + 0.004;
      const along = taperAlong(length, r.from.z - centre.z);
      if (along > 1e-6) drop = Math.min(drop, (half - need) / along);
    }
    if (drop > 0) taper = { side: 0, inwards: 0, drop };
  }
  const plenum = { centre, size: new THREE.Vector3(width, height, length), base, taper };

  // The throttle body on the plenum's front face, looking forwards.
  const throttleLength = 0.05 + bore * 0.3;
  const plenumFront = centre.z - length / 2;
  const throttle = {
    centre: new THREE.Vector3(centre.x, centre.y, plenumFront - throttleLength / 2),
    bore,
    length: throttleLength,
  };

  // The airbox across the top of the engine's front, clear above the plenum and the heads: its length the
  // solver's chamber, its section holding the solver's volume, half again as deep as it is high. It runs
  // from the end the tube enters towards the engine's middle, or across a V from one side.
  const chamber = segments[1]!;
  const area = airboxVolumeOf(spec) / chamber.length;
  const boxHeight = Math.sqrt(area / 1.5);
  const depth = boxHeight * 1.5;
  const start = new THREE.Vector3(centre.x, centre.y, plenumFront - throttleLength);
  const run = vee ? 1 : -intakeSide;
  const side = run;
  const across = new THREE.Vector3(run, 0, 0);
  const heads = Math.max(shell.top, valvetrainTop(spec)) * Math.cos(shell.straddle);
  const boxY = Math.max(centre.y + height / 2, heads) + boxHeight / 2 + 0.03;
  const boxZ = front + depth / 2;
  // On a V it starts just short of the throttle body, so the tube stays short however big the airbox.
  const inletX = vee ? -Math.min(chamber.length / 2, 0.06) : centre.x - run * 0.02;
  const inlet = new THREE.Vector3(inletX, boxY, boxZ);
  const airbox = {
    centre: inlet.clone().addScaledVector(across, chamber.length / 2),
    size: new THREE.Vector3(chamber.length, boxHeight, depth),
  };

  // The tube: forwards out of the throttle body, then up and back over into the front of the airbox, by its
  // end, in one U, as long as the solver's. Into the airbox's end instead, it would have to double back on
  // itself in a bend tighter than its own bore.
  const intoBox = inlet.clone().addScaledVector(across, bore / 2 + TUBE_INSET);
  intoBox.z = boxZ - depth / 2;
  const forwards = new THREE.Vector3(0, 0, -1);
  const tubeAt = (k: number) =>
    new THREE.CubicBezierCurve3(start, start.clone().addScaledVector(forwards, k), intoBox.clone().addScaledVector(forwards, k * TUBE_TOP_REACH), intoBox);
  const tube = tubeAt(fit((k) => tubeAt(k).getLength(), segments[0]!.length, 0, 1));

  // The snorkel: out of the airbox's far end, round to face forwards, its mouth ahead of the engine, as long
  // as the solver's. It runs forwards clear of the airbox's end by its own flattened width, so it does not
  // cut through the airbox's corner on its way past.
  const snorkelArea = (Math.PI * snorkelDiaOf(spec) ** 2) / 4;
  const outlet = inlet.clone().addScaledVector(across, chamber.length);
  const halfWidth = Math.sqrt((snorkelArea / Math.PI) * SNORKEL_ASPECT) + SNORKEL_WALL;
  const aside = halfWidth + SNORKEL_CLEAR;
  const snorkelAt = (ahead: number) => {
    const forward = depth / 2 + ahead;
    const mouth = outlet.clone().addScaledVector(across, aside).add(new THREE.Vector3(0, -0.01, -forward));
    const curve = new THREE.CubicBezierCurve3(
      outlet,
      outlet.clone().addScaledVector(across, aside * 1.15),
      mouth.clone().add(new THREE.Vector3(0, 0, forward * 0.5)),
      mouth,
    );
    return { mouth, curve };
  };
  const ahead = fit((a) => snorkelAt(a).curve.getLength(), segments[2]!.length, 0, 3);
  const { mouth, curve: snorkel } = snorkelAt(ahead);

  return {
    plenum,
    runners,
    throttle,
    tube,
    tubeRadius: bore / 2,
    airbox,
    side,
    snorkel,
    snorkelArea,
    mouth,
    segments,
  };
}
