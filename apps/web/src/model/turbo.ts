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
  newDuctId,
  nodeOrder,
  removeJunction,
} from './exhaustGraph.js';
import type { Vec3 } from './geometry.js';
import { type EngineSpec, displacement, makeSegment, segmentDiameter } from './spec.js';

/** Swept volume one turbo is drawn for, m^3: a size of turbo in the middle of the range. */
const REFERENCE_SWEPT_PER_TURBO = 1.3e-3;

/** Length of the pipe a turbo is given from its outlet when it is first fed, m. */
const OUTLET_STUB = 0.2;

/** How big a turbo is drawn, m. */
export interface TurboSize {
  /** Radius of the turbine's scroll. */
  scroll: number;
  /** Depth of the turbine housing along the shaft. */
  depth: number;
  /** Bore of its outlet, the start of the downpipe. */
  outletDia: number;
}

/** A turbo's size for `spec`, shared out between `count` of them. */
export function turboSize(spec: EngineSpec, count: number): TurboSize {
  const swept = (displacement(spec) * spec.cylinders) / Math.max(count, 1);
  const s = Math.min(Math.max(Math.cbrt(swept / REFERENCE_SWEPT_PER_TURBO), 0.6), 1.6);
  return { scroll: 0.07 * s, depth: 0.06 * s, outletDia: 0.058 * s };
}

/** Where gas goes into and out of a turbo: each flange's centre, and the way the gas is flowing there. */
export interface TurboPorts {
  inlet: { point: Vec3; dir: Vec3 };
  outlet: { point: Vec3; dir: Vec3 };
}

/**
 * Rotations, as unit quaternions `[x, y, z, w]`, the convention three.js's `Quaternion` uses.
 *
 * The turbo is drawn in its own frame, the shaft along +x with the compressor that way, and turned into
 * the world by its mount's `rotation`.
 */
export const IDENTITY: Quat = [0, 0, 0, 1];

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

/** The inlet flange in the turbo's own frame: on the side of the scroll, the gas arriving along +z. */
function localInlet(size: TurboSize): Vec3 {
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
    inlet: { point: at(localInlet(size)), dir: quatRotate([0, 0, 1], mount.rotation) },
    outlet: { point: at(localOutlet(size)), dir: quatRotate([-1, 0, 0], mount.rotation) },
  };
}

/**
 * The index of `duct`'s fitted bend, if it has one: the bend a drawn pipe finishes in where it joins a
 * turbo or another pipe. Joined at both ends, it is not edited, but fitted again whenever the pipe before
 * it or what it joins moves.
 */
export function fittedBend(_graph: ExhaustGraph, duct: ExhaustDuct): number | null {
  if (!duct.fitted || duct.segments.length === 0 || duct.to.kind !== 'node') return null;
  return duct.segments.length - 1;
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
export function placeTurbo(graph: ExhaustGraph, mount: TurboMount, outletDia: number, attach?: string): void {
  (graph.turbos ??= []).push(mount);
  if (attach) connectToTurbo(graph, attach, mount.id, outletDia);
}

/**
 * Attach a duct's far end to a turbo's inlet.
 *
 * The first pipe into a turbo also gives it an outlet: a short pipe from its outlet flange to the air, of
 * the turbine exit's bore, since a turbine has to exhaust somewhere. Drawing on from the outlet carries
 * that pipe on. Returns whether anything was attached.
 */
export function connectToTurbo(graph: ExhaustGraph, ductId: string, turboId: string, outletDia: number): boolean {
  const duct = graph.ducts.find((d) => d.id === ductId);
  const mount = graph.turbos?.find((t) => t.id === turboId);
  if (!duct || !mount) return false;
  duct.to = { kind: 'node', node: mount.node };
  for (const d of graph.ducts) if (d.continues === duct.id) delete d.continues;
  ensureTurboOutlet(graph, mount.node, outletDia);
  return true;
}

/** Give the turbo at `node` an outlet pipe if it is fed and has none. */
export function ensureTurboOutlet(graph: ExhaustGraph, node: string, outletDia: number): void {
  const ends = endsAt(graph, node);
  if (!ends.some((e) => e.end === 'outlet') || ends.some((e) => e.end === 'inlet')) return;
  const outlet: ExhaustDuct = {
    id: newDuctId(graph, 'turbo-out'),
    segments: [makeSegment({ kind: 'pipe', length: OUTLET_STUB, dIn: outletDia, dOut: outletDia })],
    from: { kind: 'node', node },
    to: { kind: 'mouth' },
  };
  graph.ducts.push(outlet);
}

/** The bore to give a turbo's outlet when nothing else says: a little wider than what feeds it. */
export function outletDiaFor(graph: ExhaustGraph, node: string, fallback: number): number {
  let area = 0;
  for (const e of endsAt(graph, node)) {
    if (e.end !== 'outlet') continue;
    const last = e.duct.segments[e.duct.segments.length - 1];
    if (last) area += (Math.PI * segmentDiameter(last, 1) ** 2) / 4;
  }
  return area > 0 ? Math.max(Math.sqrt((4 * area) / Math.PI), fallback) : fallback;
}

/**
 * Take a turbo out. The pipes that fed it end in open air where its inlet was, and its outlet pipe goes,
 * since nothing feeds it any more.
 */
export function removeTurbo(graph: ExhaustGraph, turboId: string, dirs?: DuctDirections): void {
  const mount = graph.turbos?.find((t) => t.id === turboId);
  if (!mount) return;
  graph.turbos = graph.turbos!.filter((t) => t !== mount);
  if (graph.turbos.length === 0) delete graph.turbos;
  if (endsAt(graph, mount.node).length > 0) removeJunction(graph, mount.node, null, dirs);
}
