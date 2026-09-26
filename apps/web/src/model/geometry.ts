/**
 * Where the engine's exhaust ports are, and where a straight-run pipe from one ends up.
 *
 * Plain arithmetic on the spec, with no scene objects, so the exhaust can be *compiled* to fit the engine
 * as well as drawn on it: a manifold chaining one bank's runners, or two banks' downpipes meeting behind
 * the engine, only snaps together if its pipes are the right lengths, and those lengths are set by where
 * the ports are. The drawn engine asks the same function, so the two cannot disagree.
 */

import {
  type EngineSpec,
  type PipeSegment,
  clearanceVolume,
  crankPins,
  cylinderSpacing,
  firingPlan,
  physicalBank,
  physicalBankCount,
} from './spec.js';

export type Vec3 = [number, number, number];

/**
 * A cylinder's exhaust port: where its pipe attaches, and which way the port points.
 *
 * Out of the head on the exhaust side, a little above the deck, angled slightly up; then turned with the
 * cylinder's bank, and placed along the crank at its pin.
 */
export function exhaustPortOf(spec: EngineSpec, cylinder: number): { position: Vec3; direction: Vec3 } {
  const plan = firingPlan(spec);
  const pins = crankPins(spec);
  const pin = Math.max(pins.findIndex((p) => p.cylinders.includes(cylinder)), 0);
  const z = (pin - (pins.length - 1) / 2) * cylinderSpacing(spec);

  const deg = Math.PI / 180;
  const bankRotation = (plan.banks[cylinder] ?? 0) === 0 ? 0 : -spec.vAngle * deg;
  // Straddle vertical, so a V looks like a V rather than leaning.
  const rot = bankRotation + (plan.bankCount > 1 ? (spec.vAngle / 2) * deg : 0);

  // Deck height so the TDC gap above the crown is exactly the clearance height.
  const crownOffset = spec.bore * 0.34;
  const boreArea = (Math.PI * spec.bore * spec.bore) / 4;
  const deckY = spec.stroke / 2 + spec.rodLength + crownOffset + clearanceVolume(spec) / boreArea;
  const headHeight = spec.bore * 0.52;

  const side = physicalBankCount(spec) > 1 && physicalBank(spec, cylinder) === 0 ? -1 : 1;
  const c = Math.cos(rot);
  const s = Math.sin(rot);
  const px = side * spec.bore * 1.15;
  const py = deckY + headHeight * 0.45;
  const len = Math.hypot(side, 0.18);
  const dx = side / len;
  const dy = 0.18 / len;
  return {
    position: [px * c - py * s, px * s + py * c, z],
    direction: [dx * c - dy * s, dx * s + dy * c, 0],
  };
}

/**
 * `dir` turned by `yaw` about the vertical, then by `pitch` about its new horizontal right axis.
 *
 * The one turn convention every pipe uses. Pitching about a *horizontal* axis changes elevation by exactly
 * `pitch` and leaves the horizontal heading alone.
 */
export function turnDir(dir: Vec3, yaw = 0, pitch = 0): Vec3 {
  let [x, y, z] = normalise(dir);
  if (yaw !== 0) {
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    [x, z] = [x * c + z * s, -x * s + z * c];
  }
  if (pitch !== 0) {
    // Right of the heading: heading x up.
    let [kx, ky, kz] = [-z, 0, x];
    const kl = Math.hypot(kx, ky, kz);
    if (kl < 1e-5) [kx, ky, kz] = [0, 0, 1];
    else [kx, ky, kz] = [kx / kl, ky / kl, kz / kl];
    // Rodrigues, with k . v = 0 since the axis is square to the heading.
    const c = Math.cos(pitch);
    const s = Math.sin(pitch);
    const cx = ky * z - kz * y;
    const cy = kz * x - kx * z;
    const cz = kx * y - ky * x;
    [x, y, z] = [x * c + cx * s, y * c + cy * s, z * c + cz * s];
  }
  return normalise([x, y, z]);
}

/** The yaw and pitch that `turnDir` needs to turn `from` onto `to`: its exact inverse. */
export function turnBetweenDirs(from: Vec3, to: Vec3): { yaw: number; pitch: number } {
  const f = normalise(from);
  const t = normalise(to);
  const fh = Math.hypot(f[0], f[2]);
  const th = Math.hypot(t[0], t[2]);
  let yaw = 0;
  if (fh > 1e-5 && th > 1e-5) {
    // Signed angle about +y from f's horizontal heading to t's.
    const cross = f[2] * t[0] - f[0] * t[2];
    const dot = f[0] * t[0] + f[2] * t[2];
    yaw = Math.atan2(cross / (fh * th), dot / (fh * th));
  }
  const elevation = (v: Vec3) => Math.asin(Math.min(Math.max(v[1], -1), 1));
  return { yaw, pitch: elevation(t) - elevation(f) };
}

/** Where a straight-run pipe starting at `origin` along `dir` ends, and which way it is then going. */
export function sweepEnd(segments: PipeSegment[], origin: Vec3, dir: Vec3): { end: Vec3; dir: Vec3 } {
  let d = normalise(dir);
  const p: Vec3 = [...origin];
  for (const seg of segments) {
    d = turnDir(d, seg.yaw, seg.pitch);
    p[0] += d[0] * seg.length;
    p[1] += d[1] * seg.length;
    p[2] += d[2] * seg.length;
  }
  return { end: p, dir: d };
}

export function distance(a: Vec3, b: Vec3): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function normalise(v: Vec3): Vec3 {
  const l = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / l, v[1] / l, v[2] / l];
}
