/**
 * Turning clicked points into pipe segments, and finding what a click should snap to.
 *
 * Pipes are straight runs that turn at sharp corners where segments meet, so a segment drawn to a
 * clicked point is exact: it turns, where it starts, to face the point, and runs the distance to it.
 * Every segment after the first inherits the previous one's direction and turns off it; the first
 * turns off its port's or junction's direction, which the duct stores as its heading.
 *
 * Nothing here is about the sound. Yaw and pitch are routing only — the 1D solver integrates area against
 * *axial* distance — so a drawn route and an aimed one are acoustically identical if their segment lengths
 * and diameters match.
 */

import * as THREE from 'three';

import { makeSegment, segmentDiameter, type PipeSegment } from '../model/spec.js';
import {
  endsAt,
  nodeOrder,
  type ExhaustDuct,
  type ExhaustGraph,
} from '../model/exhaustGraph.js';
import { layoutPipe, turnBetween } from './PipeMesh.js';
import type { DuctPlacement, ExhaustPlacement, ExhaustPort } from './exhaustLayout.js';

/** Longest segment a click may produce, m. */
const MAX_DRAW_LENGTH = 2;
/** Shortest segment a click may produce, m. Below this a click is a double-click, not a segment. */
export const MIN_DRAW_LENGTH = 0.02;

/**
 * Turn a direction so it lands on the nearest multiple of `stepDeg` away from `reference`.
 *
 * Quantising the *turn* rather than the absolute direction is what makes it useful: a hand-drawn route
 * comes out as a sequence of clean 15- or 45-degree bends relative to the pipe it is leaving, which is
 * how exhaust is actually bent. Quantising in world space instead would snap to the world axes and fight
 * whatever angle the port happens to sit at.
 */
export function quantiseTurn(
  dir: THREE.Vector3,
  reference: THREE.Vector3,
  stepDeg: number,
): THREE.Vector3 {
  if (stepDeg <= 0) return dir.clone().normalize();
  const from = reference.clone().normalize();
  const to = dir.clone().normalize();
  const angle = from.angleTo(to);
  if (angle < 1e-6) return from;
  const step = (stepDeg * Math.PI) / 180;
  const snapped = Math.round(angle / step) * step;
  const axis = new THREE.Vector3().crossVectors(from, to);
  if (axis.lengthSq() < 1e-12) return to;
  axis.normalize();
  return from.applyAxisAngle(axis, snapped).normalize();
}

/**
 * Yaw and pitch that turn `base` onto `dir`, in the convention the sweep uses.
 *
 * What a drawn duct stores for its heading: the direction is carried as a turn off whatever the duct
 * leaves — a port's axis or a junction's — rather than as a world vector that would detach the moment the
 * engine was resized.
 */
export function headingOffsetTo(
  base: THREE.Vector3,
  dir: THREE.Vector3,
): { yaw: number; pitch: number } {
  return turnBetween(base, dir);
}

/** Round a length to a grid, never below the minimum a segment may be. */
export function quantiseLength(length: number, gridM: number): number {
  if (gridM <= 0) return Math.max(length, MIN_DRAW_LENGTH);
  return Math.max(Math.round(length / gridM) * gridM, MIN_DRAW_LENGTH);
}

/**
 * A segment leaving `entry` along `entryDir` whose end lands on `target`.
 *
 * Exact, since a segment is straight: the corner where it starts turns it to face the target, and its
 * length is the distance. This used to be a Newton solve against the sweep, because a turn was spread
 * along the segment as an arc and its end was nowhere near where "point it at the target" put it.
 *
 * `kind` and the diameters come from the caller: geometry here, plumbing there.
 */
export function fitSegment(
  entry: THREE.Vector3,
  entryDir: THREE.Vector3,
  target: THREE.Vector3,
  template: Partial<PipeSegment> = {},
): PipeSegment {
  const chord = target.clone().sub(entry);
  const span = chord.length();
  if (span < 1e-9) return makeSegment({ ...template, id: undefined, length: MIN_DRAW_LENGTH });
  const { yaw, pitch } = turnBetween(entryDir, chord);
  return makeSegment({
    ...template,
    id: undefined,
    length: Math.min(Math.max(span, MIN_DRAW_LENGTH), MAX_DRAW_LENGTH),
    yaw,
    pitch,
  });
}

// ---------------------------------------------------------------------------
// Snapping
// ---------------------------------------------------------------------------

/**
 * Something a route being drawn can end on.
 *
 * `port` starts a runner, `node` joins an existing junction, `ductEnd` joins the far end of a duct — which
 * means making a junction there if it does not already have one — and `ductSurface` is a T, which splits
 * the duct it lands on. As a *start*, the last two continue a pipe from its open end and branch off its
 * side. `free` is the fallback: no target, so the route just carries on to a point in
 * space and the duct ends in open air.
 */
export type SnapTarget =
  | {
      kind: 'port';
      point: THREE.Vector3;
      dir: THREE.Vector3;
      cylinder: number;
      /** The duct already on this port, which drawing from it would replace. */
      occupied?: string;
    }
  | { kind: 'node'; point: THREE.Vector3; node: string }
  | { kind: 'ductEnd'; point: THREE.Vector3; duct: string }
  | {
      kind: 'ductSurface';
      point: THREE.Vector3;
      duct: string;
      x: number;
      /** The pipe's direction there, which a branch drawn *from* the side leaves along. */
      dir?: THREE.Vector3;
    }
  | { kind: 'free'; point: THREE.Vector3 };

/**
 * Every point a route could snap to, other than duct surfaces.
 *
 * Surfaces are found by raycasting the meshes instead, because "the nearest point on a tube" is what a
 * ray already answers and `PipeMesh.stationAt` already turns a hit into an arc distance.
 *
 * Every port is offered, including ones that already have a pipe. Excluding those was a mistake that made
 * draw mode useless: a compiled engine gives every cylinder a runner, so *no* port was ever clickable and
 * a route could only be started from a junction. A cylinder may still only have one pipe — `validateGraph`
 * enforces that — so drawing from an occupied port means *replacing* what is there, which is also the
 * obvious reading of the gesture. `occupied` says which duct would go, so the caller can take it out.
 */
export function collectSnapTargets(
  graph: ExhaustGraph,
  placement: ExhaustPlacement,
  ports: ExhaustPort[],
): SnapTarget[] {
  const targets: SnapTarget[] = [];

  const onPort = new Map<number, string>();
  for (const duct of graph.ducts) {
    if (duct.from.kind === 'valve' && !onPort.has(duct.from.cylinder)) {
      onPort.set(duct.from.cylinder, duct.id);
    }
  }
  ports.forEach((port, cylinder) => {
    const occupied = onPort.get(cylinder);
    targets.push({
      kind: 'port',
      point: port.position.clone(),
      dir: port.direction.clone(),
      cylinder,
      ...(occupied ? { occupied } : {}),
    });
  });

  for (const node of nodeOrder(graph)) {
    const joint = placement.joints.get(node);
    if (joint) {
      targets.push({ kind: 'node', point: joint.centre.clone(), node });
      continue;
    }
    // A junction with fewer than two pipes has no joint, so its position is where the duct leaving it starts.
    const ends = endsAt(graph, node);
    const inlet = ends.find((e) => e.end === 'inlet');
    const place = inlet ? placement.ducts.get(inlet.duct.id) : undefined;
    if (place) targets.push({ kind: 'node', point: place.origin.clone(), node });
  }

  for (const duct of graph.ducts) {
    const place = placement.ducts.get(duct.id);
    if (!place || duct.segments.length === 0) continue;
    const layout = layoutPipe(duct.segments, place.origin, place.heading);
    const end = layout.joints[layout.joints.length - 1];
    // Only a duct that vents to air: one already at a junction has that junction as its target.
    if (end && duct.to.kind === 'mouth') {
      targets.push({ kind: 'ductEnd', point: end.clone(), duct: duct.id });
    }
  }

  return targets;
}

/**
 * The target nearest a point on screen, or `null` if none is within `pixels`.
 *
 * Screen distance rather than world distance, because that is what "near the cursor" means to whoever is
 * pointing at it: a world-space radius makes distant targets impossible to hit and nearby ones grab
 * everything.
 */
export function nearestSnap(
  targets: SnapTarget[],
  pointer: THREE.Vector2,
  camera: THREE.Camera,
  pixels: number,
  viewport: { width: number; height: number },
): SnapTarget | null {
  let best: SnapTarget | null = null;
  let bestDist = Infinity;
  const ndc = new THREE.Vector3();
  for (const target of targets) {
    ndc.copy(target.point).project(camera);
    if (ndc.z < -1 || ndc.z > 1) continue;
    const dx = ((ndc.x - pointer.x) * viewport.width) / 2;
    const dy = ((ndc.y - pointer.y) * viewport.height) / 2;
    const dist = Math.hypot(dx, dy);
    if (dist < bestDist) {
      bestDist = dist;
      best = target;
    }
  }
  return bestDist <= pixels ? best : null;
}

/** Diameter to continue with when a route leaves an existing duct or port. */
export function continuingDiameter(duct: ExhaustDuct | null, fallback: number): number {
  if (!duct || duct.segments.length === 0) return fallback;
  return segmentDiameter(duct.segments[duct.segments.length - 1]!, 1);
}

/** Where a duct being drawn currently ends, and which way it is heading. */
export function routeTip(
  segments: PipeSegment[],
  place: DuctPlacement,
): { point: THREE.Vector3; dir: THREE.Vector3 } {
  if (segments.length === 0) {
    return { point: place.origin.clone(), dir: place.heading.clone().normalize() };
  }
  const layout = layoutPipe(segments, place.origin, place.heading);
  const last = layout.joints[layout.joints.length - 1];
  const dir = layout.jointDirections[layout.jointDirections.length - 1];
  return {
    point: last ? last.clone() : place.origin.clone(),
    dir: dir ? dir.clone().normalize() : place.heading.clone().normalize(),
  };
}
