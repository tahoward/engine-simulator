/**
 * Turbochargers placed in the exhaust: where one is, where its turbine's inlet and outlet are, and
 * the graph edits that attach pipes to it.
 *
 * A turbo is a `TurboMount` in the graph, at one node. The pipes ending at that node feed its turbine
 * inlet and the one pipe leaving it is its outlet, so as far as the solver is concerned it is a junction
 * with a turbine in it (`crates/engine-sim/src/exhaust_system.rs`). Unlike a junction, whose place is
 * wherever the pipes meeting at it end, a turbo has a place of its own, set where it was put down, and its
 * outlet pipe starts from its outlet flange.
 *
 * Plain arithmetic, no scene objects, so the layout and its tests can use it as the scene does.
 */

import {
  type DuctDirections,
  type ExhaustDuct,
  type ExhaustGraph,
  type Quat,
  type TurboMount,
  type TurboSettings,
  endsAt,
  junctionRemoval,
  nodeOrder,
  removeJunction,
  splitDuctAt,
} from './exhaustGraph.js';
import { type Vec3, turnBetweenDirs, turnDir } from './geometry.js';
import { type EngineSpec, displacement } from './spec.js';

/** Swept volume one turbo is drawn for, m^3: a size of turbo in the middle of the range. */
const REFERENCE_SWEPT_PER_TURBO = 1.3e-3;

/** How big a turbo is drawn, m. */
export interface TurboSize {
  /** Radius of the turbine's scroll. */
  scroll: number;
  /** Depth of the turbine housing along the shaft. */
  depth: number;
  /** Bore of its outlet, the start of the downpipe, and of its inlet, where the pipes feeding it end. */
  outletDia: number;
  inletDia: number;
}

/** A turbo's size for `spec`, shared out between `count` of them. */
export function turboSize(spec: EngineSpec, count: number): TurboSize {
  const swept = (displacement(spec) * spec.cylinders) / Math.max(count, 1);
  const s = Math.min(Math.max(Math.cbrt(swept / REFERENCE_SWEPT_PER_TURBO), 0.6), 1.6);
  return { scroll: 0.07 * s, depth: 0.06 * s, outletDia: 0.058 * s, inletDia: 0.6 * 0.07 * s };
}

/** Where gas goes into and out of a turbo: each flange's centre, and the way the gas is flowing there. */
export interface TurboPorts {
  inlet: { point: Vec3; dir: Vec3; dia: number };
  outlet: { point: Vec3; dir: Vec3; dia: number };
}

/**
 * Rotations, as unit quaternions `[x, y, z, w]`, the convention three.js's `Quaternion` uses.
 *
 * The turbo is drawn in its own frame, the shaft along +x with the compressor that way, and turned into
 * the world by its mount's `rotation`.
 */
export const IDENTITY: Quat = [0, 0, 0, 1];

/**
 * How a turbo put down on its own is turned: its shaft along the crank, the compressor forwards (the
 * world's -z), and its inlet flange facing up, taking gas arriving downwards.
 */
export const UPRIGHT: Quat = [0.5, 0.5, -0.5, 0.5];

/** `v` turned by `q`. */
export function quatRotate(v: Vec3, q: Quat): Vec3 {
  const [x, y, z, w] = q;
  // t = 2 (q.xyz x v); v' = v + w t + q.xyz x t
  const tx = 2 * (y * v[2] - z * v[1]);
  const ty = 2 * (z * v[0] - x * v[2]);
  const tz = 2 * (x * v[1] - y * v[0]);
  return [
    v[0] + w * tx + (y * tz - z * ty),
    v[1] + w * ty + (z * tx - x * tz),
    v[2] + w * tz + (x * ty - y * tx),
  ];
}

/** `a` after `b`: turning by `b`, then by `a`. */
export function quatMultiply(a: Quat, b: Quat): Quat {
  const [ax, ay, az, aw] = a;
  const [bx, by, bz, bw] = b;
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ];
}

/** A turn of `angle` radians about the unit `axis`. */
export function quatFromAxisAngle(axis: Vec3, angle: number): Quat {
  const s = Math.sin(angle / 2);
  return [axis[0] * s, axis[1] * s, axis[2] * s, Math.cos(angle / 2)];
}

/** A turn of `yaw` radians about world up. */
export function quatFromYaw(yaw: number): Quat {
  return quatFromAxisAngle([0, 1, 0], yaw);
}

/** `q` scaled back to unit length, as repeated turns slowly drift from it. */
export function quatNormalise(q: Quat): Quat {
  const n = Math.hypot(...q);
  return n > 1e-12 ? [q[0] / n, q[1] / n, q[2] / n, q[3] / n] : [...IDENTITY];
}

/**
 * One solid cylinder of the room a turbo takes up, in its own frame: along axis `axis` (0 x, 1 y, 2 z),
 * centred at `centre`, `half` its length each way and of radius `radius`, m.
 */
export interface TurboPart {
  axis: 0 | 1 | 2;
  centre: Vec3;
  half: number;
  radius: number;
}

/**
 * The room a turbo of `size` takes up, in its own frame: solid cylinders round each part `TurboMesh`
 * draws, the turbine's scroll and outlet, the bearing housing, the compressor's scroll, nose and outlet,
 * and the inlet's neck.
 */
export function turboBody(size: TurboSize): TurboPart[] {
  const s = size.scroll;
  const d = size.depth;
  const outlet = 0.5 * d + 0.45 * s;
  const bore = Math.max(size.outletDia / 2 + 0.004, 0.25 * s);
  const bearing = 0.5 * d + 0.35 * s;
  const comp = bearing + 0.75 * s;
  return [
    { axis: 0, centre: [0, 0, 0], half: Math.max(d / 2, 0.36 * s), radius: 0.98 * s },
    { axis: 0, centre: [-(outlet - 0.225 * s), 0, 0], half: 0.225 * s, radius: bore * 1.3 },
    { axis: 0, centre: [bearing, 0, 0], half: 0.35 * s, radius: 0.3 * s },
    { axis: 0, centre: [comp, 0, 0], half: 0.4 * s, radius: s },
    { axis: 0, centre: [comp + 0.6 * s, 0, 0], half: 0.25 * s, radius: 0.42 * s },
    { axis: 1, centre: [comp, 0.9 * s, 0.3 * s], half: 0.3 * s, radius: 0.2 * s },
    { axis: 2, centre: [0, 0, -0.95 * s], half: 0.2 * s, radius: 0.45 * s },
  ];
}

/** The inlet flange in the turbo's own frame: on the side of the scroll, the gas arriving along +z. */
export function localInlet(size: TurboSize): Vec3 {
  return [0, 0, -1.15 * size.scroll];
}

/** The outlet flange in the turbo's own frame: axial, out of the turbine side along -x. */
function localOutlet(size: TurboSize): Vec3 {
  return [-(0.5 * size.depth + 0.45 * size.scroll), 0, 0];
}

/** Where `mount`'s flanges are, once it has been put somewhere. */
export function turboPorts(mount: TurboMount & { position: Vec3 }, size: TurboSize): TurboPorts {
  const at = (local: Vec3): Vec3 => {
    const r = quatRotate(local, mount.rotation);
    return [mount.position[0] + r[0], mount.position[1] + r[1], mount.position[2] + r[2]];
  };
  return {
    inlet: { point: at(localInlet(size)), dir: quatRotate([0, 0, 1], mount.rotation), dia: size.inletDia },
    outlet: { point: at(localOutlet(size)), dir: quatRotate([-1, 0, 0], mount.rotation), dia: size.outletDia },
  };
}

/**
 * The index of `duct`'s fitted bend, if it has one: the bend a drawn pipe finishes in where it joins a
 * turbo or another pipe. Joined at both ends, it is not edited, but fitted again whenever the pipe before
 * it or what it joins moves.
 */
export function fittedBend(duct: ExhaustDuct): number | null {
  if (!duct.fitted || duct.segments.length === 0 || duct.to.kind !== 'node') return null;
  return duct.segments.length - 1;
}

/**
 * Where `duct` stops being edited: its fitted bend, and the swing before it if it takes one, since that
 * was fitted with it, to its length. `null` when all of it is edited.
 */
export function lockedFrom(duct: ExhaustDuct): number | null {
  const from = fittedBend(duct);
  if (from === null) return null;
  return duct.swing && from > 0 ? from - 1 : from;
}

/** Turbos with pipes attached: those whose node a pipe meets. */
export function connectedTurbos(graph: ExhaustGraph): TurboMount[] {
  return (graph.turbos ?? []).filter((t) => endsAt(graph, t.node).length > 0);
}

/** Whether the engine is turbocharged: some turbo has pipes feeding it. */
export function isTurbocharged(graph: ExhaustGraph | undefined): boolean {
  return graph ? connectedTurbos(graph).length > 0 : false;
}

/** The engine's turbo settings: every turbo's while they are in sync. */
export function engineTurboSettings(spec: EngineSpec): TurboSettings {
  return { boostTarget: spec.boostTarget, turboSize: spec.turboSize, intercooler: spec.intercooler, blowOff: spec.blowOff };
}

/** The settings `mount` runs on: its own, or the engine's. */
export function turboSettingsOf(mount: TurboMount | undefined, spec: EngineSpec): TurboSettings {
  return mount?.settings ? { ...mount.settings } : engineTurboSettings(spec);
}

/** Whether the turbos are kept in sync: none has settings of its own, so all run on the engine's. */
export function turbosSynced(graph: ExhaustGraph): boolean {
  return !(graph.turbos ?? []).some((t) => t.settings);
}

/**
 * Keep the turbos in sync, or let each be set on its own. Out of sync, each is given the engine's settings
 * as its own, to change from there. Back in sync, they all go onto the first turbo's, which are returned for
 * the engine to take; `null` when nothing changes.
 */
export function setTurbosSynced(graph: ExhaustGraph, spec: EngineSpec, synced: boolean): Partial<EngineSpec> | null {
  const turbos = graph.turbos ?? [];
  if (synced) {
    const first = turbos.find((t) => t.settings)?.settings;
    for (const t of turbos) delete t.settings;
    return first ? { ...first } : null;
  }
  for (const t of turbos) t.settings ??= engineTurboSettings(spec);
  return null;
}

/** The size every turbo in `graph` is drawn at: they share the engine's airflow between them. */
export function graphTurboSize(graph: ExhaustGraph, spec: EngineSpec): TurboSize {
  return turboSize(spec, Math.max(graph.turbos?.length ?? 0, 1));
}

/** Each placed turbo's flanges, by the node it sits at. Turbos not yet placed are left out. */
export function turboPortsOf(graph: ExhaustGraph, spec: EngineSpec): Map<string, TurboPorts> {
  const size = graphTurboSize(graph, spec);
  const out = new Map<string, TurboPorts>();
  for (const t of graph.turbos ?? []) {
    if (t.position) out.set(t.node, turboPorts(t as TurboMount & { position: Vec3 }, size));
  }
  return out;
}

/** The turn taking the turbo's own x, y and z onto the unit, square `x`, `y` and `z`. */
function quatFromBasis(x: Vec3, y: Vec3, z: Vec3): Quat {
  const trace = x[0] + y[1] + z[2];
  if (trace > 0) {
    const s = 0.5 / Math.sqrt(trace + 1);
    return quatNormalise([(y[2] - z[1]) * s, (z[0] - x[2]) * s, (x[1] - y[0]) * s, 0.25 / s]);
  }
  if (x[0] > y[1] && x[0] > z[2]) {
    const s = 2 * Math.sqrt(1 + x[0] - y[1] - z[2]);
    return quatNormalise([0.25 * s, (y[0] + x[1]) / s, (z[0] + x[2]) / s, (y[2] - z[1]) / s]);
  }
  if (y[1] > z[2]) {
    const s = 2 * Math.sqrt(1 + y[1] - x[0] - z[2]);
    return quatNormalise([(y[0] + x[1]) / s, 0.25 * s, (z[1] + y[2]) / s, (z[0] - x[2]) / s]);
  }
  const s = 2 * Math.sqrt(1 + z[2] - x[0] - y[1]);
  return quatNormalise([(z[0] + x[2]) / s, (z[1] + y[2]) / s, 0.25 * s, (x[1] - y[0]) / s]);
}

const cross = (a: Vec3, b: Vec3): Vec3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a: Vec3, b: Vec3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

/**
 * The level way a turbo's shaft lies with gas arriving at its inlet along the unit `z`: square to it and to
 * the vertical. Gas arriving straight up or down leaves no level way square to it in particular, so then
 * along the crank.
 */
function levelShaft(z: Vec3): Vec3 {
  let x = cross([0, 1, 0], z);
  if (Math.hypot(...x) < 1e-6) {
    const k = -z[2];
    x = [-k * z[0], -k * z[1], -1 - k * z[2]];
  }
  const m = Math.hypot(...x);
  return [x[0] / m, x[1] / m, x[2] / m];
}

/**
 * Put `mount` where its inlet flange is at `point`, turned to take gas arriving along `dir`: the flange
 * square to it, flush with a pipe ending there however that pipe climbs or falls. Tipped only as far as
 * that takes, its shaft lying level (`levelShaft`), then rolled `roll` radians about the way the gas
 * arrives.
 */
export function seatTurbo(mount: TurboMount, point: Vec3, dir: Vec3, size: TurboSize, roll = 0): void {
  const n = Math.hypot(...dir);
  if (n > 1e-9) {
    const z: Vec3 = [dir[0] / n, dir[1] / n, dir[2] / n];
    const x = levelShaft(z);
    const level = quatFromBasis(x, cross(z, x), z);
    mount.rotation = roll === 0 ? level : quatNormalise(quatMultiply(quatFromAxisAngle(z, roll), level));
  }
  const r = quatRotate(localInlet(size), mount.rotation);
  mount.position = [point[0] - r[0], point[1] - r[1], point[2] - r[2]];
}

/** How far a turbo turned by `rotation` is rolled about its inlet's axis from its shaft lying level, radians. */
export function turboRoll(rotation: Quat): number {
  const z = quatRotate([0, 0, 1], rotation);
  const x = quatRotate([1, 0, 0], rotation);
  const level = levelShaft(z);
  return Math.atan2(dot(cross(level, x), z), dot(level, x));
}

/**
 * Whether a turbo can go in at junction `node`, its turbine where the pipes there meet: some pipe runs
 * into it, no more than one leaves it, which becomes the turbo's outlet pipe, and it is neither a turbo
 * already nor an X-pipe's crossing, whose pipes run on through it.
 */
export function turboFitsJunction(graph: ExhaustGraph, node: string): boolean {
  if (graph.turbos?.some((t) => t.node === node)) return false;
  if (graph.junctions?.some((j) => j.node === node && j.through)) return false;
  const ends = endsAt(graph, node);
  return ends.some((e) => e.end === 'outlet') && ends.filter((e) => e.end === 'inlet').length <= 1;
}

/**
 * Put `mount` in at junction `node` (`turboFitsJunction`): the pipes into it feed its inlet, and the pipe
 * leaving it, if one does, runs on from its outlet flange. Into one pipe, it is snapped onto that pipe's
 * end (`TurboMount.snapped`), which takes any bend it was fitted with as drawn; into several, they bend in
 * to meet its flange.
 */
export function placeTurboAtJunction(graph: ExhaustGraph, mount: TurboMount, node: string): void {
  mount.node = node;
  const feeds = endsAt(graph, node).filter((e) => e.end === 'outlet');
  if (feeds.length === 1) {
    const feed = feeds[0]!.duct;
    delete feed.fitted;
    delete feed.swing;
    delete feed.square;
    mount.snapped = true;
  }
  // The turbo has a place of its own, so the junction's goes.
  if (graph.junctions) {
    graph.junctions = graph.junctions.filter((j) => j.node !== node);
    if (graph.junctions.length === 0) delete graph.junctions;
  }
  // Leaving the outlet flange, it carries on from none of the pipes into the inlet.
  for (const d of graph.ducts) if (d.from.kind === 'node' && d.from.node === node) delete d.continues;
  (graph.turbos ??= []).push(mount);
}

/** A turbo not yet in `graph`, with ids nothing is using. */
export function newTurbo(graph: ExhaustGraph, position: Vec3 | null = null, rotation: Quat = IDENTITY): TurboMount {
  const taken = new Set((graph.turbos ?? []).map((t) => t.id));
  let n = 1;
  while (taken.has(`turbo${n}`)) n++;
  // A node id no duct uses, and no other turbo either.
  const nodes = new Set([...nodeOrder(graph), ...(graph.turbos ?? []).map((t) => t.node)]);
  let k = 1;
  while (nodes.has(`turbine${k}`)) k++;
  return { id: `turbo${n}`, node: `turbine${k}`, position, rotation: [...rotation] };
}

/**
 * Add `mount` to the graph, and attach `attach`'s open end to its inlet if given: snapped onto that end,
 * which it then sits on (`TurboMount.snapped`).
 */
export function placeTurbo(graph: ExhaustGraph, mount: TurboMount, attach?: string): void {
  (graph.turbos ??= []).push(mount);
  if (attach && connectToTurbo(graph, attach, mount.id)) mount.snapped = true;
}

/**
 * Put `mount` in where one of pipe `ductId`'s segments meets the next, `x` along it, m: the pipe split there,
 * the turbo snapped onto the end of the part before (`placeTurboAtJunction`) and the part after running on
 * from its outlet. Returns whether it went in.
 */
export function placeTurboAtJoint(graph: ExhaustGraph, mount: TurboMount, ductId: string, x: number): boolean {
  const node = splitDuctAt(graph, ductId, x);
  if (!node) return false;
  placeTurboAtJunction(graph, mount, node);
  return true;
}

/**
 * Attach a duct's far end to a turbo's inlet. Returns whether anything was attached.
 *
 * Nothing is added at its outlet: until a pipe is drawn from it, the turbine exhausts straight to the air
 * at its outlet flange (`solverGraph`).
 */
export function connectToTurbo(graph: ExhaustGraph, ductId: string, turboId: string): boolean {
  const duct = graph.ducts.find((d) => d.id === ductId);
  const mount = graph.turbos?.find((t) => t.id === turboId);
  if (!duct || !mount) return false;
  duct.to = { kind: 'node', node: mount.node };
  for (const d of graph.ducts) if (d.continues === duct.id) delete d.continues;
  return true;
}

/**
 * Take a turbo out. The pipes that fed it end in open air where its inlet was, and its outlet pipe goes,
 * since nothing feeds it any more. Refused, returning `false`, where that pipe has children of its own.
 *
 * A pipe into it that starts at a port or another pipe keeps the bend it was fitted into the inlet with,
 * as drawn: often that bend is most or all of it, as a header's primaries are. A loose pipe, joined to
 * nothing else, gives its bend up and ends where it was drawn to. Given the turbos' `size`, a pipe from its
 * outlet that joins something further on, another pipe or a junction, stays too: a loose pipe from where
 * the outlet flange was, heading as it did, so it is never refused.
 */
export function removeTurbo(graph: ExhaustGraph, turboId: string, dirs?: DuctDirections, size?: TurboSize): boolean {
  const mount = graph.turbos?.find((t) => t.id === turboId);
  if (!mount) return false;
  if (size && mount.position) {
    const outlet = turboPorts(mount as TurboMount & { position: Vec3 }, size).outlet;
    for (const e of endsAt(graph, mount.node)) {
      if (e.end === 'inlet' && e.duct.to.kind === 'node') loosenFrom(e.duct, outlet);
    }
  }
  const inUse = endsAt(graph, mount.node).length > 0;
  // Refused where its outlet pipe carries on into others, as deleting that pipe would take them with it.
  if (inUse && !junctionRemoval(graph, mount.node, true)) return false;
  graph.turbos = graph.turbos!.filter((t) => t !== mount);
  if (graph.turbos.length === 0) delete graph.turbos;
  for (const e of endsAt(graph, mount.node)) {
    if (e.end !== 'outlet' || e.duct.from.kind === 'free') continue;
    delete e.duct.fitted;
    delete e.duct.swing;
    delete e.duct.square;
  }
  if (inUse) removeJunction(graph, mount.node, dirs, true);
  return true;
}

/**
 * Make `duct`, leaving a turbo's `outlet`, a loose pipe starting where the flange is, heading as it did: in
 * the world's terms, which a pipe leaving a turbo stores its heading in or off the flange's way.
 */
function loosenFrom(duct: ExhaustDuct, outlet: TurboPorts['outlet']): void {
  const heading = turnDir(duct.headingFrame === 'world' ? [1, 0, 0] : outlet.dir, duct.headingYaw ?? 0, duct.headingPitch ?? 0);
  const turn = turnBetweenDirs([1, 0, 0], heading);
  duct.from = { kind: 'free', position: [...outlet.point] };
  duct.headingYaw = turn.yaw;
  duct.headingPitch = turn.pitch;
  duct.headingFrame = 'world';
  delete duct.continues;
}
