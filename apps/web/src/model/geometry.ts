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
  intakeRunnerOf,
  physicalBank,
  physicalBankCount,
} from './spec.js';

export type Vec3 = [number, number, number];

/**
 * Whether a V is so narrow that its banks share one head, as a VR engine's do: where the two banks' intake
 * ports would meet in the valley even on top of the heads (`intakePortLocal`), or where their intake cams
 * leave no room between them for a runner down to a port. Then every cylinder's exhaust valves are on bank 0's outer side and its intake valves on
 * bank 1's, every exhaust port out of the head's outside over bank 0 and every intake port out of its
 * outside over bank 1, as an inline engine's are out of its two sides.
 */
export function sharedHead(spec: EngineSpec): boolean {
  if (physicalBankCount(spec) < 2 || spec.vAngle >= 150) return false;
  if (!valleyPort(spec, 1, true).fits) return true;
  if (spec.pushrods) return false;
  const cams = intakeCamsOf(spec);
  return cams.x - cams.reach < (intakeRunnerOf(spec).diameter / 2) * 1.53;
}

/**
 * Which bank's frame a cylinder's exhaust or intake port is placed in, and which side of that bank's head:
 * its own, exhaust on the outside of the V and intake in the valley; or under a shared head (`sharedHead`)
 * every exhaust port in bank 0's, out of its outside, and every intake port in bank 1's, out of its.
 */
export function portSideOf(spec: EngineSpec, cylinder: number, exhaust: boolean): { bank: number; side: number } {
  if (physicalBankCount(spec) < 2) return { bank: 0, side: exhaust ? 1 : -1 };
  if (sharedHead(spec)) return exhaust ? { bank: 0, side: -1 } : { bank: 1, side: 1 };
  const bank = physicalBank(spec, cylinder);
  return { bank, side: (bank === 0 ? -1 : 1) * (exhaust ? 1 : -1) };
}

/**
 * A cylinder's exhaust port: where its pipe attaches, and which way the port points.
 *
 * Out of the head on the exhaust side, a little above the deck, square to the head; then turned with the
 * cylinder's bank, or the bank whose side of a shared head it comes out of (`portSideOf`), and placed along
 * the crank where the cylinder is (`cylinderZ`).
 */
export function exhaustPortOf(spec: EngineSpec, cylinder: number): { position: Vec3; direction: Vec3 } {
  const plan = firingPlan(spec);
  const z = cylinderZ(spec, cylinder);
  const { bank, side } = portSideOf(spec, cylinder, true);

  const deg = Math.PI / 180;
  const bankRotation = bank === 0 ? 0 : -spec.vAngle * deg;
  // Straddle vertical, so a V looks like a V rather than leaning.
  const rot = bankRotation + (plan.bankCount > 1 ? (spec.vAngle / 2) * deg : 0);

  // Deck height so the TDC gap above the crown is exactly the clearance height.
  const crownOffset = spec.bore * 0.34;
  const boreArea = (Math.PI * spec.bore * spec.bore) / 4;
  const deckY = spec.stroke / 2 + spec.rodLength + crownOffset + clearanceVolume(spec) / boreArea;
  const headHeight = spec.bore * 0.52;

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

/**
 * A cylinder's intake port in the cylinder's own frame, `side` the side of the head it is on (the other
 * from the exhaust): where its runner attaches, and which way the port points, in x and y.
 *
 * Out of the side of the head square to it, as the exhaust is, where there is room. In a narrow V that
 * would put the two heads' ports into each other in the valley, so there each port moves up round the
 * head's inner edge onto its top, turning to face up as it goes, just as far as leaves its flange clear of
 * the valley's middle: in the tightest Vs on top of the head facing straight up.
 */
export function intakePortLocal(spec: EngineSpec, side: number): { position: [number, number]; direction: [number, number] } {
  const { position, direction } = valleyPort(spec, side, physicalBankCount(spec) > 1 && spec.vAngle < 150 && !sharedHead(spec));
  return { position, direction };
}

/**
 * `intakePortLocal`'s port, out of the side of the head, or with `valley` moved round onto its top as far
 * as a V's valley needs, and whether that leaves it clear of the other bank's.
 */
function valleyPort(
  spec: EngineSpec,
  side: number,
  valley: boolean,
): { position: [number, number]; direction: [number, number]; fits: boolean } {
  const crownOffset = spec.bore * 0.34;
  const boreArea = (Math.PI * spec.bore * spec.bore) / 4;
  const deckY = spec.stroke / 2 + spec.rodLength + crownOffset + clearanceVolume(spec) / boreArea;
  const headHeight = spec.bore * 0.52;
  // `u` 0 out of the side, 1 on top facing up.
  const at = (u: number, half: number) => {
    const x = spec.bore * (1.15 - 0.6 * u);
    const y = deckY + headHeight * (0.45 + 0.5 * u);
    const tilt = u * (Math.PI / 2 - half);
    return {
      position: [side * x, y] as [number, number],
      direction: [side * Math.cos(tilt), Math.sin(tilt)] as [number, number],
      fits: true,
    };
  };
  if (!valley) return at(0, 0);
  const half = ((spec.vAngle / 2) * Math.PI) / 180;
  // The flange's inner edge across the valley's middle, for a bank leaning `half` out from upright.
  const flange = intakeRunnerOf(spec).diameter / 2 * 1.53;
  const gap = (u: number) => {
    const { position, direction } = at(u, half);
    const x = -Math.abs(position[0]) * Math.cos(half) + position[1] * Math.sin(half);
    const rise = Math.atan2(direction[1], Math.abs(direction[0])) + half;
    return x - flange * Math.sin(rise);
  };
  const clear = 0.006;
  if (gap(0) >= clear) return at(0, half);
  if (gap(1) < clear) return { ...at(1, half), fits: false };
  let [lo, hi] = [0, 1];
  for (let i = 0; i < 40; i++) {
    const mid = (lo + hi) / 2;
    if (gap(mid) < clear) lo = mid;
    else hi = mid;
  }
  return at(hi, half);
}

/**
 * A cylinder's intake port, as `intakePortLocal` places it, turned with its bank, or the bank whose side of
 * a shared head it comes out of (`portSideOf`), and along the crank.
 */
export function intakePortOf(spec: EngineSpec, cylinder: number): { position: Vec3; direction: Vec3 } {
  const ex = exhaustPortOf(spec, cylinder);
  const plan = firingPlan(spec);
  const deg = Math.PI / 180;
  const { bank, side } = portSideOf(spec, cylinder, false);
  const bankRotation = bank === 0 ? 0 : -spec.vAngle * deg;
  const rot = bankRotation + (plan.bankCount > 1 ? (spec.vAngle / 2) * deg : 0);
  const { position: [px, py], direction: [dx, dy] } = intakePortLocal(spec, side);
  const c = Math.cos(rot);
  const s = Math.sin(rot);
  return {
    position: [px * c - py * s, px * s + py * c, ex.position[2]],
    direction: [dx * c - dy * s, dx * s + dy * c, 0],
  };
}

/** How far the crank runs on past its end throws, to the nose at one end and the flange at the other, m. */
export const END_JOURNAL = 0.03;

/** How far each valve leans from its cylinder's axis, its top out towards its own side of the head, radians. */
export const VALVE_TILT = 0.21;

/** How long a valve's stem is, from its head, as a fraction of the bore. */
export const STEM_LENGTH = 0.85;

/** How tall the bucket on an overhead cam engine's valve is, between the stem and the lobe, m. */
export const BUCKET_HEIGHT = 0.008;

/** The base circle of a cam lobe, m: the round part that leaves the valve shut. */
export function camBaseRadius(spec: EngineSpec): number {
  return Math.max(0.012, spec.bore * 0.14);
}

/** The deck's height above the crank along a bank's axis, m: where the head starts. */
export function deckHeight(spec: EngineSpec): number {
  const boreArea = (Math.PI * spec.bore * spec.bore) / 4;
  return spec.stroke / 2 + spec.rodLength + spec.bore * 0.34 + clearanceVolume(spec) / boreArea;
}

/** Radius of the crank's crankpins, m: about half the bore across, as a real engine's big-end journals are. */
export function crankPinRadius(spec: EngineSpec): number {
  return Math.max(0.24 * spec.bore, 0.011);
}

/** Radius of the crank's main journals, m: a little bigger than its pins. */
export function mainJournalRadius(spec: EngineSpec): number {
  return Math.max(0.28 * spec.bore, 0.013);
}

/** How thick a rod's eye is round its crankpin or wrist pin, m. */
export const ROD_EYE_WALL = 0.007;

/** Radius of a piston's wrist pin, m. */
export function wristPinRadius(spec: EngineSpec): number {
  return Math.max(0.11 * spec.bore, 0.007);
}

/** How much further a pushrod engine's rocker arm moves the valve than its tappet moves. */
export const ROCKER_RATIO = 1.5;

/**
 * How near the point (`x`, `y`), m, in a bank's frame with the crank at the origin and the bank upright,
 * that bank's con rods come over the cycle, to their surface: each a bar from the crankpin's circle to the
 * wrist pin on the bore's axis.
 */
function rodGap(spec: EngineSpec, x: number, y: number): number {
  const a = spec.stroke / 2;
  const l = spec.rodLength;
  let near = Infinity;
  for (let k = 0; k < 72; k++) {
    const phi = (k / 72) * 2 * Math.PI;
    const [px, py] = [a * Math.sin(phi), a * Math.cos(phi)];
    const wy = py + Math.sqrt(l * l - px * px);
    // From the pin (px, py) to the wrist (0, wy).
    const [dx, dy] = [-px, wy - py];
    const t = Math.min(Math.max(((x - px) * dx + (y - py) * dy) / (dx * dx + dy * dy), 0), 1);
    near = Math.min(near, Math.hypot(x - px - t * dx, y - py - t * dy));
  }
  // The rod's big end round its pin, where it is widest.
  return near - crankPinRadius(spec) - ROD_EYE_WALL;
}

/**
 * Where a pushrod engine's camshaft for `bank` runs, m, in the drawn engine's frame before it is turned to
 * straddle the vertical, bank 0 upright.
 *
 * Its pushrods run up the bank straight above it, parallel to the cylinders, so it has to sit where they
 * pass outside every bore and every con rod's swing, and its lobes too. Where one cam can, it runs in the
 * middle of a V's valley, or beside the crank on an inline engine's intake side, as near the crank as
 * that allows. In a narrow V the middle of the valley has no such place, so each bank has a cam of its
 * own outside it, on its exhaust side, beside its bore, as near the crank as clears the rods.
 */
export function blockCamOf(spec: EngineSpec, bank: number): [number, number] {
  const shell = engineShell(spec);
  const near = shell.crankcase.radius + camBaseRadius(spec) + 0.012;
  const lobe = camBaseRadius(spec) + spec.maxLift / ROCKER_RATIO;
  const vee = physicalBankCount(spec) > 1;
  const turns = vee ? [0, (-spec.vAngle * Math.PI) / 180] : [0];
  const r = spec.bore / 2;
  const deck = deckHeight(spec);
  // The lowest a piston's skirt comes, at the bottom of its stroke: the bore runs from there to the deck.
  const skirt = spec.rodLength - spec.stroke / 2 - spec.bore * 0.34 * 0.625;
  const local = (x: number, y: number, t: number) => [x * Math.cos(-t) - y * Math.sin(-t), x * Math.sin(-t) + y * Math.cos(-t)];
  /** Whether something `size` across from (x, y) clears every bore and rod. */
  const clear = (x: number, y: number, size: number) =>
    turns.every((t) => {
      const [lx, ly] = local(x, y, t);
      if (ly < 0) return true;
      if (rodGap(spec, lx, ly) < size + 0.003) return false;
      return ly < skirt - size || Math.abs(lx) >= r + size + 0.003;
    });
  /** Whether a cam at (x, y) for the bank turned by `own` fits, and its pushrods up that bank to its deck. */
  const fits = (x: number, y: number, own: number) => {
    if (!clear(x, y, lobe)) return false;
    const [lx, ly] = local(x, y, own);
    for (let k = 1; k <= 24; k++) {
      const h = ly + ((deck - ly) * k) / 24;
      const [px, py] = [lx * Math.cos(own) - h * Math.sin(own), lx * Math.sin(own) + h * Math.cos(own)];
      if (!clear(px, py, 0.0035)) return false;
    }
    return true;
  };
  const [ux, uy] = vee
    ? [Math.sin(((spec.vAngle / 2) * Math.PI) / 180), Math.cos(((spec.vAngle / 2) * Math.PI) / 180)]
    : [-0.8, 0.6];
  // One cam, out from the crank until it fits.
  for (let d = near; d < deck; d += 0.004) {
    if (turns.every((t) => fits(ux * d, uy * d, t))) return [ux * d, uy * d];
  }
  // A cam a bank, outside it: bank 0's out beside its bore away from the valley, bank 1's mirrored.
  const out = -(r + lobe + 0.004);
  let y = Math.sqrt(Math.max(near * near - out * out, 0));
  while (y < deck && !fits(out, y, 0)) y += 0.004;
  if (bank === 0) return [out, y];
  const t = turns[1]!;
  return [-out * Math.cos(t) - y * Math.sin(t), -out * Math.sin(t) + y * Math.cos(t)];
}

/**
 * Where a vee's intake camshafts run, in the drawn engine's frame: how far either side of the middle their
 * centres are, m, and how far out from them the lobes reach at full lift. A pushrod engine's rocker arms
 * stand in, over the inner valves' tips. What a plenum between the banks has to fit between.
 */
export function intakeCamsOf(spec: EngineSpec): { x: number; y: number; reach: number } {
  const turn = (spec.vAngle / 2) * (Math.PI / 180);
  const base = camBaseRadius(spec);
  const lift = Math.max(spec.maxLift, spec.camSwitchRpm > 0 ? spec.highMaxLift : 0);
  // Bank 0's intake valves lean in towards the valley, their tops, and the cam over them. A pushrod head's
  // valves stand upright over the middle of the cylinder, and their arms reach in from the tips to over
  // the block's cam, where the pushrods come up.
  const along = spec.bore * STEM_LENGTH + (spec.pushrods ? 0.01 : BUCKET_HEIGHT + base);
  const [x, y] = spec.pushrods
    ? [Math.max(blockCamOf(spec, 0)[0], 0.01), deckHeight(spec) + along]
    : [spec.bore * 0.24 + Math.sin(VALVE_TILT) * along, deckHeight(spec) + Math.cos(VALVE_TILT) * along];
  return {
    x: Math.abs(x * Math.cos(turn) - y * Math.sin(turn)),
    y: x * Math.sin(turn) + y * Math.cos(turn),
    reach: spec.pushrods ? 0.03 : base + lift,
  };
}

/**
 * How far the valvetrain reaches along a bank's axis from the crank, m: over the noses of an overhead cam's
 * lobes, or the rocker arms on top of a pushrod engine's heads.
 */
export function valvetrainTop(spec: EngineSpec): number {
  if (spec.pushrods) return deckHeight(spec) + spec.bore * STEM_LENGTH + 0.025;
  const stemTop = deckHeight(spec) + spec.bore * STEM_LENGTH * Math.cos(VALVE_TILT);
  const lift = Math.max(spec.maxLift, spec.camSwitchRpm > 0 ? spec.highMaxLift : 0);
  return stemTop + BUCKET_HEIGHT + 2 * camBaseRadius(spec) + lift;
}

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
