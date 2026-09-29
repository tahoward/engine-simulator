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
  endsAt,
  junctionRemoval,
  nodeOrder,
  removeJunction,
} from './exhaustGraph.js';
import type { Vec3 } from './geometry.js';
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

/**
 * Put `mount` where its inlet flange is at `point`, turned to take gas arriving along `dir`.
 *
 * Seated level, turned only about the vertical: a pipe arriving from above or below meets the flange at an
 * angle, as it would meet a junction, until the turbo is tipped to meet it.
 */
export function seatTurbo(mount: TurboMount, point: Vec3, dir: Vec3, size: TurboSize): void {
  const flat = Math.hypot(dir[0], dir[2]);
  if (flat > 1e-6) mount.rotation = quatFromYaw(Math.atan2(dir[0], dir[2]));
  const r = quatRotate(localInlet(size), mount.rotation);
  mount.position = [point[0] - r[0], point[1] - r[1], point[2] - r[2]];
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

/** Add `mount` to the graph, and attach `attach`'s open end to its inlet if given. */
export function placeTurbo(graph: ExhaustGraph, mount: TurboMount, attach?: string): void {
  (graph.turbos ??= []).push(mount);
  if (attach) connectToTurbo(graph, attach, mount.id);
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
 */
export function removeTurbo(graph: ExhaustGraph, turboId: string, dirs?: DuctDirections): boolean {
  const mount = graph.turbos?.find((t) => t.id === turboId);
  if (!mount) return false;
  const inUse = endsAt(graph, mount.node).length > 0;
  // Refused where its outlet pipe carries on into others, as deleting that pipe would take them with it.
  if (inUse && !junctionRemoval(graph, mount.node, true)) return false;
  graph.turbos = graph.turbos!.filter((t) => t !== mount);
  if (graph.turbos.length === 0) delete graph.turbos;
  if (inUse) removeJunction(graph, mount.node, dirs, true);
  return true;
}
