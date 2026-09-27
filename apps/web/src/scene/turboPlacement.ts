/**
 * Where turbos go in the scene: seating the ones a compiled layout asked for, and moving one with the
 * pipes that feed it.
 *
 * Geometry on the graph and the layout, no scene objects, so it can be tested as the app uses it.
 */

import * as THREE from 'three';

import { junctionAt, type ExhaustDuct, type ExhaustGraph, type Quat } from '../model/exhaustGraph.js';
import type { Vec3 } from '../model/geometry.js';
import { segmentDiameter, type EngineSpec } from '../model/spec.js';
import { graphTurboSize, seatTurbo, turboPortsOf } from '../model/turbo.js';
import { bendAnchor, fitCurve } from './drawing.js';
import { layoutGraph, type ExhaustPort } from './exhaustLayout.js';
import { layoutPipe } from './PipeMesh.js';

/** Where a turbo is put down unless it is snapped to a pipe: level with the exhaust ports. */
export function turboHeight(ports: ExhaustPort[]): number {
  return ports.length > 0 ? ports.reduce((a, p) => a + p.position.y, 0) / ports.length : 0;
}

/**
 * Put every turbo a compiled layout asked for where that layout's junction is: its inlet flange where the
 * pipes meet, turned to take them.
 */
export function seatTurbos(graph: ExhaustGraph, ports: ExhaustPort[], spec: EngineSpec): void {
  const unseated = (graph.turbos ?? []).filter((t) => !t.position);
  if (unseated.length === 0) return;
  const placement = layoutGraph(ports, graph, turboPortsOf(graph, spec));
  const size = graphTurboSize(graph, spec);
  for (const mount of unseated) {
    const joint = placement.joints.get(mount.node);
    const out = graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === mount.node);
    const leaving = out ? placement.ducts.get(out.id) : undefined;
    const point = joint?.centre ?? leaving?.origin;
    const dir = joint?.axis ?? leaving?.heading;
    if (!point || !dir) {
      mount.position = [0, turboHeight(ports), 0];
      continue;
    }
    seatTurbo(mount, [point.x, point.y, point.z], [dir.x, dir.y, dir.z], size);
  }
}

/**
 * Move a junction to `position`: it has a place of its own from then on, and the pipes into it bend in to
 * meet it there. `axis` is the way it points, which pipes leaving it are turned off, taken where it stood the
 * first time it is moved.
 */
export function moveJunction(
  graph: ExhaustGraph,
  ports: ExhaustPort[],
  spec: EngineSpec,
  node: string,
  position: Vec3,
  axis: Vec3,
): void {
  const pinned = junctionAt(graph, node);
  if (pinned) pinned.position = position;
  else (graph.junctions ??= []).push({ node, position, axis });
  refitBends(graph, ports, spec);
}

/** Move or turn a turbo. The pipes feeding it follow, their bends into its inlet fitted again. */
export function moveTurbo(
  graph: ExhaustGraph,
  ports: ExhaustPort[],
  spec: EngineSpec,
  id: string,
  position: Vec3,
  rotation: Quat,
): void {
  const mount = graph.turbos?.find((t) => t.id === id);
  if (!mount) return;
  mount.position = position;
  mount.rotation = rotation;
  refitBends(graph, ports, spec);
}

/**
 * Fit the bend every joined pipe finishes in: from where the pipe as drawn ends, one smooth bend to what it
 * joins (`bendAnchor`, `fitCurve`). Every pipe into a turbo has one, and so does every pipe drawn to join
 * another.
 *
 * Done on every rebuild, so each bend follows whatever moved: the turbo or the pipe it joins, or the pipe
 * drawn up to it.
 */
export function refitBends(graph: ExhaustGraph, ports: ExhaustPort[], spec: EngineSpec): void {
  const turbos = turboPortsOf(graph, spec);
  if (turbos.size === 0 && !graph.junctions?.length && !graph.ducts.some((d) => d.fitted)) return;
  const placement = layoutGraph(ports, graph, turbos);
  for (const duct of graph.ducts) {
    if (duct.to.kind !== 'node') continue;
    // Every pipe into a turbo, or into a junction that has been moved, bends in to meet it.
    if (!duct.fitted && !turbos.has(duct.to.node) && !junctionAt(graph, duct.to.node)) continue;
    const anchor = bendAnchor(graph, placement, duct.to.node, duct.id);
    const place = placement.ducts.get(duct.id);
    if (anchor && place) fitBend(duct, place.origin, place.heading, anchor);
  }
}

/** Fit `duct`'s end to `anchor`: its drawn part, then a bend arriving there along the anchor's direction. */
export function fitBend(
  duct: ExhaustDuct,
  origin: THREE.Vector3,
  heading: THREE.Vector3,
  anchor: { point: THREE.Vector3; dir: THREE.Vector3 },
): void {
  const drawn = duct.fitted ? duct.segments.slice(0, -1) : duct.segments;
  const swept = layoutPipe(drawn, origin, heading);
  const entry = drawn.length > 0 ? swept.joints.at(-1)! : origin;
  const entryDir = drawn.length > 0 ? swept.jointDirections.at(-1)! : heading;
  const target = anchor.point;
  // Where the drawn pipe already meets it there is nothing to fit: a turbo put down on its end.
  if (entry.distanceTo(target) < 1e-4 && drawn.length > 0) {
    duct.segments = drawn;
    delete duct.fitted;
    return;
  }
  const last = drawn.at(-1) ?? duct.segments.at(-1);
  const dia = last ? segmentDiameter(last, 1) : 0.042;
  const bend = fitCurve(entry, entryDir, target, anchor.dir, { dIn: dia, dOut: dia });
  // The same bend as before keeps its id, so the panel's row for it stays put.
  const old = duct.fitted ? duct.segments.at(-1) : undefined;
  if (old) bend.id = old.id;
  duct.segments = [...drawn, bend];
  duct.fitted = true;
}
