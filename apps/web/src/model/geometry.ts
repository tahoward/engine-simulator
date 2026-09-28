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
  cylinderZ,
  firingPlan,
  physicalBank,
  physicalBankCount,
} from './spec.js';

export type Vec3 = [number, number, number];

/**
 * A cylinder's exhaust port: where its pipe attaches, and which way the port points.
 *
 * Out of the head on the exhaust side, a little above the deck, square to the head; then turned with the
 * cylinder's bank, and placed along the crank where the cylinder is (`cylinderZ`).
 */
export function exhaustPortOf(spec: EngineSpec, cylinder: number): { position: Vec3; direction: Vec3 } {
  const plan = firingPlan(spec);
  const z = cylinderZ(spec, cylinder);

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
  // Straight out of the side of the head, square to it.
  return {
    position: [px * c - py * s, px * s + py * c, z],
    direction: [side * c, side * s, 0],
  };
}

/** How far the crank runs on past its end throws, to the nose at one end and the flange at the other, m. */
export const END_JOURNAL = 0.03;

/**
 * The outline of the block and heads: one rounded casting per bank, from the crankcase up past the valve
 * springs, and the crankcase round the crank's sweep, the whole of it along the crank. What the drawn
 * engine's see-through shell is, and what a turbo is put clear of.
 */
export interface EngineShell {
  /** How long the block is along the crank, m, centred on the origin as the engine is. */
  length: number;
  /** Each bank's casting: how far it is turned about the crank from straight up, in the world, radians. */
  banks: number[];
  /** Each casting's width across its bank, and how far up its bank's axis it runs from and to, m. */
  width: number;
  bottom: number;
  top: number;
  /** How round its edges are, m. */
  rounding: number;
  /** The crankcase, a cylinder along the crank: its radius, and its length, m. */
  crankcase: { radius: number; length: number };
  /** How far the whole engine is turned about the crank so a V straddles vertical, radians. */
  straddle: number;
}

export function engineShell(spec: EngineSpec): EngineShell {
  const plan = firingPlan(spec);
  const deg = Math.PI / 180;
  const a = spec.stroke / 2;
  const boreArea = (Math.PI * spec.bore * spec.bore) / 4;
  const deckY = a + spec.rodLength + spec.bore * 0.34 + clearanceVolume(spec) / boreArea;
  const spacing = cylinderSpacing(spec);
  const length = (crankPins(spec).length - 1) * spacing + Math.max(spacing, spec.bore * 1.3);
  const straddle = plan.bankCount > 1 ? (spec.vAngle / 2) * deg : 0;
  const turns = new Set(plan.banks.map((b) => (b === 0 ? 0 : -spec.vAngle * deg)));
  const top = deckY + spec.bore * 0.52;
  const bottom = a * 1.2;
  const width = spec.bore * 2.3;
  return {
    length,
    banks: [...turns].map((t) => t + straddle),
    width,
    bottom,
    top,
    rounding: Math.min(width, top - bottom) * 0.12,
    crankcase: { radius: a * 1.5 + 0.012, length: length + 2 * END_JOURNAL },
    straddle,
  };
}

/** How far `p` is outside the engine's outline (`engineShell`), m: negative inside it, by how deep. */
export function engineShellDistance(shell: EngineShell, p: Vec3): number {
  const dz = Math.abs(p[2]);
  let best = solidCylinder(Math.hypot(p[0], p[1]) - shell.crankcase.radius, dz - shell.crankcase.length / 2);
  const r = shell.rounding;
  const qz = dz - (shell.length / 2 - r);
  for (const turn of shell.banks) {
    // Into the casting's frame: its bank's axis up, across it along x.
    const c = Math.cos(turn);
    const s = Math.sin(turn);
    const x = p[0] * c + p[1] * s;
    const y = -p[0] * s + p[1] * c - (shell.top + shell.bottom) / 2;
    const qx = Math.abs(x) - (shell.width / 2 - r);
    const qy = Math.abs(y) - ((shell.top - shell.bottom) / 2 - r);
    const outside = Math.hypot(Math.max(qx, 0), Math.max(qy, 0), Math.max(qz, 0));
    best = Math.min(best, outside + Math.min(Math.max(qx, qy, qz), 0) - r);
  }
  return best;
}

/**
 * How far outside a solid cylinder a point is, m, from how far it is outside the cylinder's curved side
 * and outside its ends: negative inside, by how deep.
 */
export function solidCylinder(radial: number, along: number): number {
  const outside = Math.hypot(Math.max(radial, 0), Math.max(along, 0));
  return outside > 0 ? outside : Math.max(radial, along);
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
    // Right of the heading: heading x up. Straight up or down there is none, so the yaw says which way
    // the heading goes as it pitches away from vertical: towards (cos yaw, 0, -sin yaw), as a level
    // heading along +x would be turned.
    let [kx, ky, kz] = [-z, 0, x];
    const kl = Math.hypot(kx, ky, kz);
    if (kl < 1e-5) [kx, ky, kz] = [Math.sin(yaw), 0, Math.cos(yaw)];
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
  } else if (th > 1e-5) {
    // From straight up or down, the yaw is the way it pitches away to: see `turnDir`.
    yaw = Math.atan2(-t[2], t[0]);
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

/**
 * How far a bend segment turns, radians, and the radius it turns round, m; `null` for a straight one.
 *
 * In its own frame a bend starts along +x, so its turn is how far its end direction is from +x, and the
 * radius is the one whose arc through that turn spans its chord.
 */
export function bendShape(seg: PipeSegment): { angle: number; radius: number } | null {
  if (!seg.curve) return null;
  const [dx] = normalise(seg.curve.dir);
  const angle = Math.acos(Math.min(Math.max(dx, -1), 1));
  const chord = Math.hypot(...seg.curve.end);
  const half = Math.sin(angle / 2);
  return { angle, radius: half > 1e-9 ? chord / (2 * half) : chord };
}
