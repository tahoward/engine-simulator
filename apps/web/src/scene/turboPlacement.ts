/**
 * Where turbos go in the scene: seating the ones a compiled layout asked for, and moving one with the
 * pipes that feed it.
 *
 * Geometry on the graph and the layout, no scene objects, so it can be tested as the app uses it.
 */

import * as THREE from 'three';

import { disconnectEnd, type DuctDirections, type ExhaustGraph, type Quat } from '../model/exhaustGraph.js';
import type { Vec3 } from '../model/geometry.js';
import type { EngineSpec } from '../model/spec.js';
import { graphTurboSize, seatTurbo, turboPortsOf } from '../model/turbo.js';
import { MIN_DRAW_LENGTH, fitSegment } from './drawing.js';
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
 * Move or turn a turbo, and bring the pipes feeding it along.
 *
 * Each feed's last segment is fitted again from where it starts to the inlet flange's new place, as it
 * was fitted when the pipe was drawn to it. One whose last segment would be too short to be a pipe comes
 * off, its end left open. The outlet pipe starts from the outlet flange, so it follows by itself.
 */
export function moveTurbo(
  graph: ExhaustGraph,
  ports: ExhaustPort[],
  spec: EngineSpec,
  id: string,
  position: Vec3,
  rotation: Quat,
  dirs?: DuctDirections,
): void {
  const mount = graph.turbos?.find((t) => t.id === id);
  if (!mount) return;
  mount.position = position;
  mount.rotation = rotation;
  const turbos = turboPortsOf(graph, spec);
  const inlet = turbos.get(mount.node)?.inlet.point;
  if (!inlet) return;
  const target = new THREE.Vector3(...inlet);
  const placement = layoutGraph(ports, graph, turbos);
  for (const duct of [...graph.ducts]) {
    if (duct.to.kind !== 'node' || duct.to.node !== mount.node || duct.segments.length === 0) continue;
    const place = placement.ducts.get(duct.id);
    if (!place) continue;
    const swept = layoutPipe(duct.segments, place.origin, place.heading);
    const n = duct.segments.length;
    const entry = n > 1 ? swept.joints[n - 2]! : place.origin;
    const entryDir = n > 1 ? swept.jointDirections[n - 2]! : place.heading;
    if (entry.distanceTo(target) < MIN_DRAW_LENGTH) {
      disconnectEnd(graph, duct.id, dirs);
      continue;
    }
    const last = duct.segments[n - 1]!;
    // Every segment turns where it starts, the first off the duct's heading, so the fit replaces it.
    duct.segments[n - 1] = fitSegment(entry, entryDir, target, { kind: last.kind, dIn: last.dIn, dOut: last.dOut });
  }
}
