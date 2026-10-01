/**
 * Where the intake's parts are drawn: the plenum on the engine, the throttle body on its front, the tube
 * from it up into an airbox across the top of the engine's front, and the snorkel from the airbox forwards
 * to just ahead of the engine. Pure geometry, so what is drawn and where the intake is heard from (`soundSources`) agree.
 *
 * The tube, the airbox and the snorkel are the inlet tract the simulation solves (`inletSegments`): the
 * airbox holds the solver's volume over its length, and the tube and snorkel have the solver's bores.
 * The solver is one-dimensional, so how they are routed changes nothing it hears.
 */

import * as THREE from 'three';

import { engineShell, exhaustPortOf, valvetrainTop } from '../model/geometry.js';
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
}

/**
 * Cylinder `b`'s intake port: its exhaust port mirrored across the cylinder, on the other side of the head,
 * opening the other way.
 */
function intakePortOf(spec: EngineSpec, b: number): { position: THREE.Vector3; direction: THREE.Vector3 } {
  const ex = exhaustPortOf(spec, b);
  const d = new THREE.Vector3(...ex.direction).normalize();
  const p = new THREE.Vector3(...ex.position);
  const across = p.x * d.x + p.y * d.y;
  return { position: p.addScaledVector(d, -2 * across), direction: d.negate() };
}

export interface InletLayout {
  /**
   * The plenum: its middle and its size along x, y and z, m. In a V it narrows below the throttle body to
   * `base` of its width, sitting down into the valley between the heads; beside an inline head it is a
   * rounded box, its `base` 1.
   */
  plenum: { centre: THREE.Vector3; size: THREE.Vector3; base: number };
  /** On an inline engine or a boxer, one curved runner a cylinder from the plenum into the head. */
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

export function inletLayout(spec: EngineSpec): InletLayout {
  const shell = engineShell(spec);
  const segments = inletSegments(spec);
  const front = -shell.length / 2;
  const vee = physicalBankCount(spec) > 1;
  const exhaustSide = Math.sign(exhaustPortOf(spec, 0).direction[0]) || 1;
  const intakeSide = vee ? 1 : -exhaustSide;

  // The plenum: across the valley of a V or a boxer, or along the intake side of an inline head. Its
  // footprint holds the plenum's volume at its height, within what the engine leaves room for, and its
  // front face takes the throttle body, wall and all, inside its rounded edges.
  const bore = throttleDiaOf(spec);
  const face = bore + 2 * THROTTLE_WALL + 2 * PLENUM_ROUNDING + 0.01;
  // Most of the engine's length, and at least past every runner, by their bore and a little more.
  const radius = intakeRunnerOf(spec).diameter / 2;
  const zs = Array.from({ length: spec.cylinders }, (_, b) => cylinderZ(spec, b));
  const lastRunner = Math.max(...zs.map(Math.abs)) + radius + 0.02;
  const length = Math.max(shell.length * 0.85, 2 * lastRunner);
  const height = Math.max(PLENUM_HEIGHT, face);
  const footprint = plenumVolumeOf(spec) / (height * length);
  const width = Math.max(Math.min(Math.max(footprint, 0.07), vee ? shell.width * 0.9 : 0.12), face);
  const runners: Runner[] = [];
  let centre: THREE.Vector3;
  if (vee) {
    const valley = Math.max(shell.top * Math.cos(shell.straddle) * 0.8, shell.crankcase.radius + 0.02);
    centre = new THREE.Vector3(0, valley + height / 2, 0);
    // A boxer's heads lie flat either side, their intake ports on top: each runner arches out of the
    // plenum's side and down into its port. A V's ports open into the valley under the plenum.
    if (spec.vAngle >= 150) {
      for (let b = 0; b < spec.cylinders; b++) {
        const port = intakePortOf(spec, b);
        const out = Math.sign(port.position.x) || 1;
        // From the plenum's inside wall, a little above its middle, where its side is still upright.
        const from = new THREE.Vector3(out * (width / 2 - PLENUM_WALL), centre.y + height * 0.1, port.position.z);
        const reach = from.distanceTo(port.position) * 0.45;
        runners.push({
          from,
          to: port.position,
          leaving: new THREE.Vector3(out * reach, reach * 0.4, 0),
          arriving: port.direction.clone().multiplyScalar(-reach),
          radius,
        });
      }
    }
  } else {
    const x = intakeSide * (shell.width / 2 + 0.08 + width / 2);
    // Straight and level out of the plenum's side into each intake port, at the height the exhaust leaves
    // the other side of the head: each from the plenum's inside wall, so it meets the wall with no gap and
    // leaves the plenum's cavity clear behind the throttle body.
    const portY = intakePortOf(spec, 0).position.y;
    centre = new THREE.Vector3(x, portY, 0);
    for (let b = 0; b < spec.cylinders; b++) {
      const port = intakePortOf(spec, b).position;
      const from = new THREE.Vector3(x - intakeSide * (width / 2 - PLENUM_WALL), port.y, port.z);
      const third = port.clone().sub(from).divideScalar(3);
      runners.push({ from, to: port, leaving: third, arriving: third, radius });
    }
  }
  const plenum = { centre, size: new THREE.Vector3(width, height, length), base: vee ? 0.55 : 1 };

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

  // The tube: forwards out of the throttle body, then up and round into the airbox's end, its bends as
  // full as make it the solver's length.
  const tubeAt = (k: number) =>
    new THREE.CubicBezierCurve3(
      start,
      start.clone().add(new THREE.Vector3(0, 0, -k)),
      inlet.clone().addScaledVector(across, -k),
      inlet,
    );
  const tube = tubeAt(fit((k) => tubeAt(k).getLength(), segments[0]!.length, 0.02, 0.5));

  // The snorkel: out of the airbox's far end, round to face forwards, its mouth just ahead of the engine,
  // as long as the solver's.
  const outlet = inlet.clone().addScaledVector(across, chamber.length);
  const snorkelAt = (reach: number) => {
    const mouth = outlet
      .clone()
      .addScaledVector(across, Math.min(reach * 0.25, 0.06))
      .add(new THREE.Vector3(0, -0.01, -(depth / 2 + reach * 0.8)));
    const curve = new THREE.CubicBezierCurve3(
      outlet,
      outlet.clone().addScaledVector(across, Math.min(reach * 0.3, 0.08)),
      mouth.clone().add(new THREE.Vector3(0, 0, reach * 0.3)),
      mouth,
    );
    return { mouth, curve };
  };
  const reach = fit((r) => snorkelAt(r).curve.getLength(), segments[2]!.length, 0, 3);
  const { mouth, curve: snorkel } = snorkelAt(reach);
  const snorkelArea = (Math.PI * snorkelDiaOf(spec) ** 2) / 4;

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
