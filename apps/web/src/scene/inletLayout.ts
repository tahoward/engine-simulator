/**
 * Where the intake's parts are drawn: the plenum on the engine, the throttle body on its front, the tube
 * from it forwards to an airbox across the front of the engine, and the snorkel from the airbox to the
 * grille. Pure geometry, so what is drawn and where the intake is heard from (`soundSources`) agree.
 *
 * The tube, the airbox and the snorkel are the inlet tract the simulation solves (`inletSegments`): the
 * airbox holds the solver's volume over its length, and the tube and snorkel have the solver's bores.
 * The solver is one-dimensional, so how they are routed changes nothing it hears.
 */

import * as THREE from 'three';

import { engineShell, exhaustPortOf } from '../model/geometry.js';
import { airboxVolumeOf, inletSegments, plenumVolumeOf, snorkelDiaOf, throttleDiaOf } from '../model/intakeSizing.js';
import { cylinderZ, intakeRunnerOf, physicalBankCount, type EngineSpec, type PipeSegment } from '../model/spec.js';

export interface Runner {
  /** Where it leaves the plenum, and where it meets the head. */
  from: THREE.Vector3;
  to: THREE.Vector3;
  radius: number;
}

export interface InletLayout {
  /** The plenum: a rounded box, its middle and its size along x, y and z, m. */
  plenum: { centre: THREE.Vector3; size: THREE.Vector3 };
  /** On an inline engine, one curved runner a cylinder from the plenum into the head. */
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

/** How high the plenum is, m. */
const PLENUM_HEIGHT = 0.1;

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
  // footprint holds the plenum's volume at its height, within what the engine leaves room for.
  const length = shell.length * 0.85;
  const footprint = plenumVolumeOf(spec) / (PLENUM_HEIGHT * length);
  const width = Math.min(Math.max(footprint, 0.07), vee ? shell.width * 0.9 : 0.12);
  const runners: Runner[] = [];
  let centre: THREE.Vector3;
  if (vee) {
    const valley = Math.max(shell.top * Math.cos(shell.straddle) * 0.8, shell.crankcase.radius + 0.02);
    centre = new THREE.Vector3(0, valley + PLENUM_HEIGHT / 2, 0);
  } else {
    const x = intakeSide * (shell.width / 2 + 0.08 + width / 2);
    centre = new THREE.Vector3(x, shell.top * 0.7, 0);
    const radius = intakeRunnerOf(spec).diameter / 2;
    for (let b = 0; b < spec.cylinders; b++) {
      const z = cylinderZ(spec, b);
      runners.push({
        from: new THREE.Vector3(x - intakeSide * (width / 2), centre.y + PLENUM_HEIGHT * 0.15, z),
        to: new THREE.Vector3(intakeSide * shell.width * 0.45, shell.top * 0.62, z),
        radius,
      });
    }
  }
  const plenum = { centre, size: new THREE.Vector3(width, PLENUM_HEIGHT, length) };

  // The throttle body on the plenum's front face, looking forwards.
  const bore = throttleDiaOf(spec);
  const throttleLength = 0.05 + bore * 0.3;
  const plenumFront = centre.z - length / 2;
  const throttle = {
    centre: new THREE.Vector3(centre.x, centre.y, plenumFront - throttleLength / 2),
    bore,
    length: throttleLength,
  };

  // The airbox across the front of the engine, a little lower than the throttle, its length the solver's
  // chamber, its section holding the solver's volume, half again as deep as it is high.
  const chamber = segments[1]!;
  const area = airboxVolumeOf(spec) / chamber.length;
  const height = Math.sqrt(area / 1.5);
  const depth = height * 1.5;
  const side = intakeSide;
  const across = new THREE.Vector3(side, 0, 0);
  const start = new THREE.Vector3(centre.x, centre.y, plenumFront - throttleLength);

  // The tube: forwards out of the throttle body, then round into the airbox's end, the airbox set as far
  // ahead as makes the tube the solver's length.
  const tubeAt = (ahead: number) => {
    const inlet = new THREE.Vector3(centre.x + side * 0.1, centre.y - 0.04, Math.min(start.z, front) - ahead);
    const curve = new THREE.CubicBezierCurve3(
      start,
      start.clone().add(new THREE.Vector3(0, 0, -0.16)),
      inlet.clone().addScaledVector(across, -0.14),
      inlet,
    );
    return { inlet, curve };
  };
  const ahead = fit((a) => tubeAt(a).curve.getLength(), segments[0]!.length, depth / 2 + 0.02, 2);
  const { inlet, curve: tube } = tubeAt(ahead);
  const airbox = {
    centre: inlet.clone().addScaledVector(across, chamber.length / 2),
    size: new THREE.Vector3(chamber.length, height, depth),
  };

  // The snorkel: out of the airbox's far end, round to face forwards, its mouth towards the grille, as long
  // as the solver's.
  const outlet = inlet.clone().addScaledVector(across, chamber.length);
  const snorkelAt = (reach: number) => {
    const mouth = outlet
      .clone()
      .addScaledVector(across, reach * 0.3)
      .add(new THREE.Vector3(0, -0.02, -(reach * 0.75 + depth / 2)));
    const curve = new THREE.CubicBezierCurve3(
      outlet,
      outlet.clone().addScaledVector(across, reach * 0.35),
      mouth.clone().add(new THREE.Vector3(0, 0, reach * 0.35)),
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
