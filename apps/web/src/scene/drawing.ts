/**
 * Turning clicked points into pipe segments, and finding what a click should snap to.
 *
 * A drawn pipe is straight runs that turn at sharp corners where segments meet, so a segment drawn to a
 * clicked point is exact: it turns, where it starts, to face the point, and runs the distance to it. Where
 * it joins something it finishes in a smooth bend fitted to arrive along it (`fitCurve`).
 * Every segment after the first inherits the previous one's direction and turns off it; the first
 * turns off its port's or junction's direction, which the duct stores as its heading.
 *
 * Nothing here is about the sound. Yaw and pitch are routing only — the 1D solver integrates area against
 * *axial* distance — so a drawn route and an aimed one are acoustically identical if their segment lengths
 * and diameters match.
 */

import * as THREE from 'three';

import { arcHandle, makeSegment, segmentDiameter, type PipeSegment } from '../model/spec.js';
import {
  attachToLooseStart,
  disconnectEnd,
  drawnSegments,
  endsAt,
  fittedCount,
  joinDuctEnd,
  junctionAt,
  newDuctId,
  nodeOrder,
  releaseBend,
  removeDuct,
  splitDuctAt,
  turboAt,
  type DuctDirections,
  type ExhaustDuct,
  type ExhaustGraph,
} from '../model/exhaustGraph.js';
import {
  acrossAxis,
  curveFrame,
  curveInWorld,
  curvePath,
  layoutPipe,
  turnBetween,
  turnHeading,
  widthAxis,
} from './PipeMesh.js';
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
 * how exhaust is actually bent, and it holds whatever angle the port happens to sit at. `snapToEngine` is
 * the other way to tidy a route: square to the engine rather than to the pipe.
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
 * A direction a drawn segment can lock to, in the engine's frame.
 *
 * The engine sits at the origin with its crank along Z and bank 0 standing along +Y, so its frame is the
 * world's: X across the engine, Y up, Z along the crank.
 */
export interface EngineDirection {
  dir: THREE.Vector3;
  /** What to call it, for a readout: "up", "down and across". */
  name: string;
}

function engineDirectionName(x: number, y: number, z: number): string {
  const parts: string[] = [];
  if (y !== 0) parts.push(y > 0 ? 'up' : 'down');
  if (x !== 0) parts.push('across');
  if (z !== 0) parts.push('along the crank');
  return parts.join(' and ');
}

/**
 * The engine's axes and the 45-degree diagonals between each pair: the bends a real system is built from.
 *
 * The cube's corner diagonals are left out, because they sit at 35 degrees off every plane and nobody
 * bends pipe to that.
 */
export const ENGINE_DIRECTIONS: readonly EngineDirection[] = (() => {
  const dirs: EngineDirection[] = [];
  for (const x of [-1, 0, 1]) {
    for (const y of [-1, 0, 1]) {
      for (const z of [-1, 0, 1]) {
        const nonzero = Math.abs(x) + Math.abs(y) + Math.abs(z);
        if (nonzero === 0 || nonzero === 3) continue;
        dirs.push({ dir: new THREE.Vector3(x, y, z).normalize(), name: engineDirectionName(x, y, z) });
      }
    }
  }
  return dirs;
})();

/** Closer to straight back than this, a direction is a pipe folding onto itself and is not offered. */
const MAX_TURN_COS = -0.9;

/**
 * Lock a segment leaving `tip` to whichever engine direction the pointer is nearest, on screen.
 *
 * On screen, because that is the only place the pointer is unambiguous: a point clicked in 3D has no depth,
 * so snapping "the clicked point" would depend on a plane nobody can see. Each direction is drawn as a ray
 * from the tip, the one passing closest to the cursor wins, and the length is where that ray comes closest
 * to the pointer's ray. `ahead` is offered too, so a pipe leaving an angled port can carry straight on.
 *
 * `null` when the tip is behind the camera, where no direction can be drawn.
 */
export function snapToEngine(
  tip: THREE.Vector3,
  ahead: THREE.Vector3,
  ray: THREE.Ray,
  pointer: THREE.Vector2,
  camera: THREE.Camera,
  viewport: { width: number; height: number },
  gridM: number,
): { point: THREE.Vector3; dir: THREE.Vector3; name: string } | null {
  const view = camera.matrixWorldInverse;
  const inFront = (p: THREE.Vector3): boolean => p.clone().applyMatrix4(view).z < 0;
  const toPixels = (p: THREE.Vector3): THREE.Vector2 => {
    const ndc = p.clone().project(camera);
    return new THREE.Vector2((ndc.x * viewport.width) / 2, (ndc.y * viewport.height) / 2);
  };
  if (!inFront(tip)) return null;

  const cursor = new THREE.Vector2((pointer.x * viewport.width) / 2, (pointer.y * viewport.height) / 2);
  const from = toPixels(tip);
  const straight = ahead.clone().normalize();
  const candidates: EngineDirection[] = [...ENGINE_DIRECTIONS, { dir: straight, name: 'straight on' }];

  let best: { dir: THREE.Vector3; name: string; reach: number } | null = null;
  let bestDist = Infinity;
  for (const c of candidates) {
    if (c.dir.dot(straight) < MAX_TURN_COS) continue;
    // Shortened until its far end is in front of the camera, where projecting it means something.
    let reach = MAX_DRAW_LENGTH;
    while (reach > MIN_DRAW_LENGTH && !inFront(tip.clone().addScaledVector(c.dir, reach))) reach /= 2;
    const to = toPixels(tip.clone().addScaledVector(c.dir, reach));
    const span = to.clone().sub(from);
    // Pointing into the screen: no way to aim along it, and its length would be meaningless.
    if (span.lengthSq() < 16) continue;
    const t = THREE.MathUtils.clamp(cursor.clone().sub(from).dot(span) / span.lengthSq(), 0, 1);
    const dist = from.clone().addScaledVector(span, t).distanceTo(cursor);
    // Strictly nearer, so where straight on coincides with an engine direction it goes by the engine's name.
    if (dist < bestDist - 1e-6) {
      bestDist = dist;
      best = { dir: c.dir, name: c.name, reach };
    }
  }
  if (!best) return null;

  const onLine = new THREE.Vector3();
  ray.distanceSqToSegment(tip, tip.clone().addScaledVector(best.dir, best.reach), undefined, onLine);
  const length = quantiseLength(onLine.distanceTo(tip), gridM);
  return {
    point: tip.clone().addScaledVector(best.dir, length),
    dir: best.dir.clone(),
    name: best.name,
  };
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
 * length is the distance.
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

/**
 * Free the pipes carrying on from `ductId`'s end, as it is deleted: where it is the only pipe into its
 * junction, the pipes leaving that junction are left as loose pipes, each where it lies, heading as it
 * does, and what was attached to them stays attached to them. Its end is left open.
 *
 * Not at a turbo: the pipe drawn from its outlet stays on it, unfed until a pipe is drawn into it again.
 */
export function loosenChildren(graph: ExhaustGraph, ductId: string, placement: ExhaustPlacement): void {
  const duct = graph.ducts.find((d) => d.id === ductId);
  if (!duct || duct.to.kind !== 'node' || turboAt(graph, duct.to.node)) return;
  const node = duct.to.node;
  const ends = endsAt(graph, node);
  const others = ends.filter((e) => e.end === 'outlet' && e.duct !== duct);
  if (others.length > 0) return;
  for (const e of ends) {
    if (e.end !== 'inlet') continue;
    const child = e.duct;
    const place = placement.ducts.get(child.id);
    if (!place) continue;
    const turn = turnBetween(new THREE.Vector3(1, 0, 0), place.heading);
    child.from = { kind: 'free', position: [place.origin.x, place.origin.y, place.origin.z] };
    child.headingYaw = turn.yaw;
    child.headingPitch = turn.pitch;
    child.headingFrame = 'world';
    delete child.continues;
  }
  duct.to = { kind: 'mouth' };
  // The bend it was fitted in goes too, and any swing: they were only the way to what it joined.
  releaseBend(duct);
  if (graph.junctions) {
    graph.junctions = graph.junctions.filter((j) => j.node !== node);
    if (graph.junctions.length === 0) delete graph.junctions;
  }
}

/**
 * Delete segment `index` from the middle of a pipe, leaving the segments after it as a loose pipe where
 * they lie. That pipe takes over the far end, and whatever it joined, so the pipes carrying on from it stay
 * joined. The pipe before the gap ends in open air.
 *
 * Returns the loose pipe's id, or `null` when there is nothing after the segment to split off.
 */
export function splitDuct(graph: ExhaustGraph, ductId: string, index: number, place: DuctPlacement): string | null {
  const duct = graph.ducts.find((d) => d.id === ductId);
  if (!duct || index < 0 || index >= duct.segments.length - 1) return null;
  const swept = layoutPipe(duct.segments, place.origin, place.heading);
  const start = swept.joints[index]!;
  const turn = turnBetween(new THREE.Vector3(1, 0, 0), swept.jointDirections[index]!);
  const rest: ExhaustDuct = {
    id: newDuctId(graph, 'pipe'),
    segments: duct.segments.slice(index + 1),
    from: { kind: 'free', position: [start.x, start.y, start.z] },
    to: duct.to,
    headingYaw: turn.yaw,
    headingPitch: turn.pitch,
    headingFrame: 'world',
    ...(duct.fitted ? { fitted: true as const } : {}),
    ...(duct.swing ? { swing: true as const } : {}),
    ...(duct.square ? { square: true as const } : {}),
  };
  for (const d of graph.ducts) if (d.continues === duct.id) d.continues = rest.id;
  duct.segments = duct.segments.slice(0, index);
  duct.to = { kind: 'mouth' };
  delete duct.fitted;
  delete duct.swing;
  delete duct.square;
  graph.ducts.push(rest);
  return rest.id;
}

/**
 * Take `ductId`'s far end off what it joins, with the bend it was fitted in, leaving it ending where it was
 * drawn to. Where it is the only pipe into a junction, the pipes carrying on from it are left loose, where
 * they lie (`loosenChildren`); a turbo keeps the pipe drawn from its outlet. A pipe that was nothing but its
 * bend goes, unless it is a cylinder's. Returns `false`, changing nothing, where it cannot come off.
 */
export function detachDuct(
  graph: ExhaustGraph,
  ductId: string,
  placement: ExhaustPlacement | null,
  dirs?: DuctDirections,
): boolean {
  const duct = graph.ducts.find((d) => d.id === ductId);
  if (!duct || duct.to.kind !== 'node') return false;
  if (placement) loosenChildren(graph, ductId, placement);
  if (duct.to.kind === 'node' && !disconnectEnd(graph, ductId, dirs)) return false;
  if (duct.segments.length === 0 && duct.from.kind !== 'valve') removeDuct(graph, ductId, dirs);
  return true;
}

/**
 * Delete the pipe `ductId` — the stretch of it between its junctions, as the last segment left in it is
 * deleted — and put back what it joined as it was before.
 *
 * A fitted pipe, a bend joining two things, goes by itself: what it split is rejoined, the junctions it made
 * go, and a pipe it joined at its start starts where it did, loose again (`tidyJunctions`). Nothing is
 * turned round, and nothing else moves. A cylinder's own pipe is never deleted, as every cylinder needs one:
 * it comes off what it was fitted to.
 *
 * A straight one goes by itself too, not past its junctions. Each junction it leaves still joining two or more
 * stays where it is, fixed there, and the fitted pipes at it fitted to it as before. One left with only a
 * fitted pipe goes with it, that pipe going too, putting back what it joined at its other end as above (a
 * cylinder's comes off it); one left with only a straight leaves that loose where it lies (`loosenChildren`).
 */
export function removePipe(
  graph: ExhaustGraph,
  ductId: string,
  placement: ExhaustPlacement | null,
  dirs?: DuctDirections,
): void {
  const duct = graph.ducts.find((d) => d.id === ductId);
  if (!duct) return;
  if (duct.fitted) {
    removeFitted(graph, duct, placement, dirs);
    return;
  }
  const alone: ExhaustDuct[] = [];
  let keepsEnd = false;
  for (const node of touchedAt(duct)) {
    if (turboAt(graph, node)) continue;
    const ends = endsAt(graph, node).filter((e) => e.duct !== duct);
    const left = ends.map((e) => e.duct);
    // Left the end of one straight pipe, and fitted pipes: that pipe's end, which they face and follow
    // (`bendAnchor`), as those drawn onto an open end do.
    const straightEnds = ends.filter((e) => !e.duct.fitted);
    const endOfOne = straightEnds.length === 1 && straightEnds[0]!.end === 'outlet';
    if (left.length >= 2 && endOfOne) {
      if (duct.to.kind === 'node' && duct.to.node === node) keepsEnd = true;
    } else if (left.length >= 2) {
      // Still joining two or more: the junction stays where it is, and what is fitted to it with it.
      const joint = placement?.joints.get(node);
      if (joint && !junctionAt(graph, node)) {
        (graph.junctions ??= []).push({ node, position: [joint.centre.x, joint.centre.y, joint.centre.z], axis: [joint.axis.x, joint.axis.y, joint.axis.z] });
      }
      if (duct.to.kind === 'node' && duct.to.node === node) keepsEnd = true;
    } else if (left.length === 1 && left[0]!.fitted) {
      alone.push(left[0]!);
    }
  }
  for (const d of alone) removeFitted(graph, d, placement, dirs);
  // What carries on from its end, where nothing else holds the junction there, is left loose where it lies.
  if (placement && !keepsEnd) loosenChildren(graph, duct.id, placement);
  if (!graph.ducts.includes(duct)) return;
  if (duct.from.kind === 'valve') disconnectEnd(graph, duct.id, dirs);
  else removeDuct(graph, duct.id, dirs);
}

/** `removePipe` for a fitted pipe: a cylinder's comes off what it joined, anything else goes. */
function removeFitted(graph: ExhaustGraph, duct: ExhaustDuct, placement: ExhaustPlacement | null, dirs?: DuctDirections): void {
  if (!graph.ducts.includes(duct)) return;
  // The only pipe into a loose pipe's start: that pipe starts where it did again.
  if (placement) loosenChildren(graph, duct.id, placement);
  if (duct.from.kind === 'valve') disconnectEnd(graph, duct.id, dirs);
  else removeDuct(graph, duct.id, dirs);
}

/** The junctions `duct` starts or ends at. */
function touchedAt(duct: ExhaustDuct): string[] {
  const nodes: string[] = [];
  if (duct.from.kind === 'node') nodes.push(duct.from.node);
  if (duct.to.kind === 'node') nodes.push(duct.to.node);
  return nodes;
}

/**
 * A pipe's shape in the world, segment by segment: the way each sets off, after the corner into it, the way
 * it finishes, and for a bend, where it ends from where it starts. What `swingPipe` turns.
 */
export interface PipeShape {
  starts: THREE.Vector3[];
  ends: THREE.Vector3[];
  /** A bend's chord, start to end; `null` for a straight. */
  chords: (THREE.Vector3 | null)[];
  /** A can's width axis (`widthAxis`), which turns with it; `null` for anything else. */
  widths: (THREE.Vector3 | null)[];
}

/** The shape of `segments` leaving along `heading`. */
export function pipeShape(segments: PipeSegment[], heading: THREE.Vector3): PipeShape {
  const shape: PipeShape = { starts: [], ends: [], chords: [], widths: [] };
  let dir = heading.clone().normalize();
  for (const seg of segments) {
    const start = turnHeading(dir, seg.yaw, seg.pitch);
    if (seg.curve) {
      const bend = curveInWorld(seg.curve, new THREE.Vector3(), start);
      shape.chords.push(bend.end);
      dir = bend.dir;
    } else {
      shape.chords.push(null);
      dir = start.clone();
    }
    shape.starts.push(start);
    shape.ends.push(dir.clone());
    shape.widths.push(seg.kind === 'chamber' ? widthAxis(seg, start) : null);
  }
  return shape;
}

/**
 * Swing a pipe as one piece about where it starts: its drawn segments, whose shape was `shape`, turned
 * `angle` radians about `axis`.
 *
 * The way it sets off goes into its heading, turned off `base` as the layout reads it. Each corner after is
 * worked out again from the turned directions either side of it, and each bend is put back in the frame of
 * the way it now sets off, since that frame is the world's up and not the pipe's own: so every segment keeps
 * its length, and every corner and bend stays as it was.
 *
 * From segment `from` on, it is only the rest of the pipe that swings, about where that segment starts:
 * what comes before it, and the heading, stay put.
 */
export function swingPipe(
  duct: ExhaustDuct,
  base: THREE.Vector3,
  shape: PipeShape,
  axis: THREE.Vector3,
  angle: number,
  from = 0,
): void {
  const turn = (v: THREE.Vector3) => v.clone().applyAxisAngle(axis, angle);
  const starts = shape.starts.map((v, i) => (i >= from ? turn(v) : v.clone()));
  const ends = shape.ends.map((v, i) => (i >= from ? turn(v) : v.clone()));
  if (from === 0) {
    const heading = turnBetween(base, starts[0]!);
    duct.headingYaw = heading.yaw;
    duct.headingPitch = heading.pitch;
  }
  starts.forEach((start, i) => {
    if (i < from) return;
    const seg = duct.segments[i]!;
    const corner = i === 0 ? { yaw: 0, pitch: 0 } : turnBetween(ends[i - 1]!, start);
    seg.yaw = corner.yaw;
    seg.pitch = corner.pitch;
    // A can turns with the rest, rolled round the way it now runs to where its width has turned to.
    const width = shape.widths[i];
    if (seg.kind === 'chamber' && width) {
      const level = acrossAxis(start);
      const want = turn(width);
      const roll = Math.atan2(level.clone().cross(want).dot(start.clone().normalize()), level.dot(want));
      if (Math.abs(roll) > 1e-9) seg.roll = roll;
      else delete seg.roll;
    }
    const chord = shape.chords[i];
    if (seg.curve && chord) {
      const f = curveFrame(start);
      const local = (v: THREE.Vector3): [number, number, number] => [v.dot(f.x), v.dot(f.y), v.dot(f.z)];
      seg.curve = { ...seg.curve, end: local(turn(chord)), dir: local(ends[i]!) };
    }
  });
}

/** A duct's bore `x` m along it. */
export function diameterAt(segments: PipeSegment[], x: number): number {
  let at = 0;
  for (const seg of segments) {
    if (x <= at + seg.length) return segmentDiameter(seg, seg.length > 0 ? (x - at) / seg.length : 0);
    at += seg.length;
  }
  const last = segments.at(-1);
  return last ? segmentDiameter(last, 1) : 0.042;
}

export interface BendAnchor {
  point: THREE.Vector3;
  dir: THREE.Vector3;
  /** The bore there, which the bend ends at so the two match. */
  dia: number;
}

/**
 * Where a pipe ending at `node` bends in to, and the way it arrives there: what it is joining, where that
 * has a place of its own.
 *
 * A turbo's inlet flange, arrived at square. A junction that has been moved, where it was put. Or a
 * junction that carries a pipe on through it (see `ExhaustDuct.continues`): one made at another pipe's open
 * end, or on its side, is where that pipe is, and the pipe joining it arrives along the pipe the gas carries
 * on through, merging into it rather than meeting it at a corner. Turn that pipe where it leaves, and the
 * bend follows it. A junction placed where its pipes' ends average out has no place apart from them, so
 * `null`, as for the pipe that is itself carried on. Into a pipe's side it merges from whichever end of it the
 * pipe as drawn comes down (`sideArrival`). A pipe drawn in square (`ExhaustDuct.square`) arrives
 * across the pipe instead. Either way it ends at the bore of the pipe it joins, and follows it when that
 * changes.
 */
export function bendAnchor(
  graph: ExhaustGraph,
  placement: ExhaustPlacement,
  node: string,
  ductId: string,
): BendAnchor | null {
  const turbo = placement.turbos.get(node);
  if (turbo) {
    const { point, dir, dia } = turbo.inlet;
    return { point: new THREE.Vector3(...point), dir: new THREE.Vector3(...dir), dia };
  }
  const along = junctionAnchor(graph, placement, node, ductId);
  const self = graph.ducts.find((d) => d.id === ductId);
  const place = placement.ducts.get(ductId);
  if (!along || !self || !place) return along;
  const drawn = drawnSegments(self);
  const swept = layoutPipe(drawn, place.origin, place.heading);
  const from = drawn.length > 0 ? swept.joints.at(-1)! : place.origin;
  const fromDir = drawn.length > 0 ? swept.jointDirections.at(-1)! : place.heading;
  // Into a pipe's side, where it runs on through: along it, from whichever end of it the pipe comes in, its end
  // the same circle as the pipe's there, flush; or drawn in square (`ExhaustDuct.square`), across it. Anywhere
  // else — a junction fixed in place, or a pipe's end, as a side becomes once the pipe past it goes — every
  // pipe comes in the one way, the junction's, facing its ring, so however the pipes turn, those meeting there
  // stay together.
  const through = !junctionAt(graph, node) && endsAt(graph, node).some((e) => e.end === 'inlet' && e.duct.continues !== undefined);
  if (!through) return along;
  if (self.square) return { ...along, dir: squareArrival(along.point, along.dir, from, fromDir) };
  return { ...along, dir: sideArrival(along.point, along.dir, from, fromDir) };
}

/**
 * The way a pipe from `from`, heading `fromDir` there, merges into the side of a pipe running along `axis`
 * through `point`: along the pipe, towards the far side from `from`, so it curves in from whichever end of
 * the pipe it comes down. From level with `point`, the way `fromDir` leans along it; failing that, along `axis`.
 */
export function sideArrival(
  point: THREE.Vector3,
  axis: THREE.Vector3,
  from: THREE.Vector3,
  fromDir: THREE.Vector3,
): THREE.Vector3 {
  const a = axis.clone().normalize();
  const EPS = 1e-6;
  const ahead = point.clone().sub(from).dot(a);
  if (Math.abs(ahead) > EPS) return ahead > 0 ? a : a.negate();
  return fromDir.dot(a) < -EPS ? a.negate() : a;
}

/**
 * The way a pipe from `from`, heading `fromDir` there, arrives square into the side of a pipe running along
 * `axis` through `point`: straight across it, from the side `from` is on. From on the pipe's line, across
 * it the way `fromDir` leans; failing that, any way across.
 */
export function squareArrival(
  point: THREE.Vector3,
  axis: THREE.Vector3,
  from: THREE.Vector3,
  fromDir: THREE.Vector3,
): THREE.Vector3 {
  const a = axis.clone().normalize();
  const across = (v: THREE.Vector3) => v.clone().addScaledVector(a, -v.dot(a));
  for (const v of [across(point.clone().sub(from)), across(fromDir)]) {
    if (v.length() > 1e-6) return v.normalize();
  }
  const any = Math.abs(a.y) < 0.9 ? new THREE.Vector3(0, 1, 0) : new THREE.Vector3(1, 0, 0);
  return across(any).normalize();
}

/** `bendAnchor` for a junction, arriving along what it carries on through. */
function junctionAnchor(
  graph: ExhaustGraph,
  placement: ExhaustPlacement,
  node: string,
  ductId: string,
): BendAnchor | null {
  const ends = endsAt(graph, node);
  // A junction that has been moved is where it was put, every pipe into it arriving along the one leaving it
  // that it runs straight on into, or else the first leaving it.
  const pinned = junctionAt(graph, node);
  if (pinned) {
    const onto = pinned.through?.[ductId];
    const out = ends.find((e) => e.end === 'inlet' && e.duct.id === onto)?.duct ?? ends.find((e) => e.end === 'inlet')?.duct;
    const place = out ? placement.ducts.get(out.id) : undefined;
    const dir =
      out && place && out.segments.length > 0
        ? layoutPipe(out.segments, place.origin, place.heading).stations[0]!.direction.clone()
        : new THREE.Vector3(...pinned.axis);
    // Into a header's collector, each primary keeps its own bore, as its gas does. Otherwise the bore of the
    // pipe leaving, or where nothing leaves yet, of the widest pipe already there.
    const self = graph.ducts.find((d) => d.id === ductId);
    const drawn = self ? (self.fitted ? self.segments.slice(0, -1) : self.segments) : [];
    if (pinned.collector && drawn.length > 0) {
      return { point: new THREE.Vector3(...pinned.position), dir, dia: segmentDiameter(drawn.at(-1)!, 1) };
    }
    let dia = out?.segments[0] ? segmentDiameter(out.segments[0], 0) : 0;
    if (!dia) {
      for (const e of ends) {
        const last = e.end === 'outlet' && e.duct.id !== ductId ? e.duct.segments.at(-1) : undefined;
        if (last) dia = Math.max(dia, segmentDiameter(last, 1));
      }
    }
    if (!dia) dia = 0.042;
    return { point: new THREE.Vector3(...pinned.position), dir, dia };
  }
  const onward = ends.find((e) => e.end === 'inlet' && e.duct.continues !== undefined)?.duct;
  if (!onward) {
    // The end of a pipe bends were drawn onto: the one pipe ending here that is not a bend fitted to meet it
    // (`layoutGraph`). A bend comes in there, along it.
    const unbent = ends.filter((e) => e.end === 'outlet' && e.duct.id !== ductId && !e.duct.fitted && e.duct.segments.length > 0);
    const owner = unbent.length === 1 ? unbent[0]!.duct : undefined;
    const at = owner ? placement.ducts.get(owner.id) : undefined;
    if (!owner || !at) return null;
    const swept = layoutPipe(owner.segments, at.origin, at.heading);
    return { point: swept.joints.at(-1)!.clone(), dir: swept.jointDirections.at(-1)!.clone(), dia: segmentDiameter(owner.segments.at(-1)!, 1) };
  }
  const primary = ends.find((e) => e.end === 'outlet' && e.duct.id === onward.continues)?.duct;
  if (!primary || primary.id === ductId) return null;
  const place = placement.ducts.get(primary.id);
  if (!place || primary.segments.length === 0) return null;
  const swept = layoutPipe(primary.segments, place.origin, place.heading);
  // The way the pipe carrying on leaves the junction, its first segment's corner and all.
  const next = placement.ducts.get(onward.id);
  const leaving =
    next && onward.segments.length > 0
      ? layoutPipe(onward.segments, next.origin, next.heading).stations[0]!.direction.clone()
      : swept.jointDirections.at(-1)!.clone();
  return {
    point: swept.joints.at(-1)!.clone(),
    dir: leaving,
    dia: segmentDiameter(primary.segments.at(-1)!, 1),
  };
}

/**
 * The way a branch drawn out of the side of a pipe running along `axis` sets off from `tip`, towards `point`:
 * along the pipe, whichever way along it `point` lies, so it turns off it the least, and its start is the same
 * circle as the pipe's there, flush. Level with `tip`, along `axis`.
 */
export function sideLeaving(axis: THREE.Vector3, tip: THREE.Vector3, point: THREE.Vector3): THREE.Vector3 {
  const a = axis.clone().normalize();
  return point.clone().sub(tip).dot(a) < -1e-6 ? a.negate() : a;
}

/**
 * The one bend from `tip`, setting off along `dir`, to `point`: an arc, so it arrives turned as far again
 * off the line from `tip` to `point` as it set off, the smoothest way there. Straight where `point` is ahead.
 */
export function arcTo(
  tip: THREE.Vector3,
  dir: THREE.Vector3,
  point: THREE.Vector3,
  template: Partial<PipeSegment> = {},
): PipeSegment {
  const d0 = dir.clone().normalize();
  const chord = point.clone().sub(tip);
  const c = chord.lengthSq() > 1e-12 ? chord.clone().normalize() : d0.clone();
  // The arc's tangent at its far end: the way it set off, reflected in the line across it.
  const d1 = c.clone().multiplyScalar(2 * d0.dot(c)).sub(d0);
  if (d1.lengthSq() < 1e-12) d1.copy(c);
  return fitCurve(tip, d0, point, d1.normalize(), template);
}

/** Below this turn and this offset, a pipe runs straight into a port rather than curving: radians, m. */
const STRAIGHT_TURN = (1 * Math.PI) / 180;
const STRAIGHT_OFFSET = 0.001;

/**
 * The one segment running from `entry`, heading `entryDir`, into `target`, arriving along `targetDir`: a
 * port's flange, which a pipe should meet square rather than at whatever angle it happens to come from.
 *
 * Straight where the port is on the pipe's own line. Otherwise a smooth bend (`PipeSegment.curve`), an S
 * where the port is off to one side but facing the same way: leaving the way the pipe was going and
 * arriving square into the flange, as long as the bend is. Bends cost nothing acoustically; see
 * `PipeEditor`.
 */
export function fitCurve(
  entry: THREE.Vector3,
  entryDir: THREE.Vector3,
  target: THREE.Vector3,
  targetDir: THREE.Vector3,
  template: Partial<PipeSegment> = {},
  handle?: number,
): PipeSegment {
  const d0 = entryDir.clone().normalize();
  const d1 = targetDir.clone().normalize();
  const chord = target.clone().sub(entry);
  const span = chord.length();
  const along = chord.dot(d0);
  const offset = chord.clone().addScaledVector(d0, -along).length();
  const base = { ...template, kind: 'pipe' as const, id: undefined, curve: undefined };
  if (span < 1e-3 || (d0.angleTo(d1) < STRAIGHT_TURN && offset < STRAIGHT_OFFSET && along > 0)) {
    // Straight, and as long as it is: short enough to be a nudge, it is not stretched to a drawn pipe's least.
    const { yaw, pitch } = span > 1e-9 ? turnBetween(d0, chord) : { yaw: 0, pitch: 0 };
    return makeSegment({ ...base, length: Math.max(span, 1e-3), yaw, pitch });
  }
  const f = curveFrame(d0);
  const local = (v: THREE.Vector3): [number, number, number] => [v.dot(f.x), v.dot(f.y), v.dot(f.z)];
  const { length } = curvePath(entry, d0, target, d1, 8, handle);
  const curve = { end: local(chord), dir: local(d1), ...(handle !== undefined ? { handle } : {}) };
  return makeSegment({ ...base, length, yaw: 0, pitch: 0, curve });
}

/** The tightest a drawn bend may turn, in pipe bores: about what a mandrel bender makes. */
export const MIN_BEND_BORES = 1.5;

/**
 * A bend, as a tube bender makes one: from `entry` heading `d0`, turning to head `d1` round a radius of
 * `radius`, in the plane the two directions span. Where it ends follows from those. `null` where there is
 * no such bend: `d1` the way the pipe already goes, or straight back the way it came.
 */
export function bendSegment(
  entry: THREE.Vector3,
  d0: THREE.Vector3,
  d1: THREE.Vector3,
  radius: number,
  template: Partial<PipeSegment> = {},
): PipeSegment | null {
  const from = d0.clone().normalize();
  const to = d1.clone().normalize();
  const angle = from.angleTo(to);
  if (angle < STRAIGHT_TURN || angle > Math.PI - STRAIGHT_TURN || radius <= 0) return null;
  const across = to.clone().addScaledVector(from, -Math.cos(angle)).normalize();
  const end = entry
    .clone()
    .addScaledVector(from, radius * Math.sin(angle))
    .addScaledVector(across, radius * (1 - Math.cos(angle)));
  // One arc, not the fitted bend's general cubic, so it turns round the radius it says all the way.
  return fitCurve(entry, from, end, to, template, arcHandle(angle));
}

/**
 * The bend `seg` is, reshaped to turn through `angle` round `radius`, in the plane it already turns in.
 * Its bores and id are kept. `null` where the angle leaves no bend.
 */
export function reshapeBend(seg: PipeSegment, angle: number, radius: number): PipeSegment | null {
  if (!seg.curve) return null;
  const x = new THREE.Vector3(1, 0, 0);
  const dir = new THREE.Vector3(...seg.curve.dir);
  const across = dir.clone().addScaledVector(x, -dir.x);
  if (across.lengthSq() < 1e-12) across.copy(new THREE.Vector3(...seg.curve.end)).addScaledVector(x, -seg.curve.end[0]);
  if (across.lengthSq() < 1e-12) return null;
  across.normalize();
  const to = x.clone().multiplyScalar(Math.cos(angle)).addScaledVector(across, Math.sin(angle));
  const bend = bendSegment(new THREE.Vector3(), x, to, radius, { ...seg });
  if (!bend) return null;
  bend.id = seg.id;
  return bend;
}

/**
 * Bend a whole straight into one arc, as a tube is bent: turning by `angle` radians about `axis`, square to
 * the plane it bends in, and as long as it was, so its radius is its length over the turn. `dir` is the way
 * it runs, in the world, as `axis` is. Where it starts stays put, keeping its corner, its id and its bores.
 *
 * No tighter than `tightest`: a turn that would need it tighter goes only as far as fits, `clamped` saying
 * so.
 */
export function bendWhole(
  seg: PipeSegment,
  dir: THREE.Vector3,
  axis: THREE.Vector3,
  angle: number,
  tightest: number,
): { segment: PipeSegment; clamped: boolean; angle: number; radius: number } {
  const length = seg.length;
  // Short of doubling straight back, where a bend has no plane of its own.
  let turn = Math.min(Math.abs(angle), Math.PI - 2 * STRAIGHT_TURN);
  if (turn < STRAIGHT_TURN || length <= 0) return { segment: seg, clamped: false, angle: 0, radius: Infinity };
  const clamped = length / turn < tightest;
  if (clamped) turn = length / tightest;
  const signed = Math.sign(angle) * turn;
  const from = dir.clone().normalize();
  const to = from.clone().applyAxisAngle(axis.clone().normalize(), signed);
  // The cubic is a hair off the arc it stands for, so its radius is trimmed until it is the length exactly.
  let radius = length / turn;
  let bend = bendSegment(new THREE.Vector3(), from, to, radius, { ...seg })!;
  for (let i = 0; i < 3; i++) {
    radius *= length / bend.length;
    bend = bendSegment(new THREE.Vector3(), from, to, radius, { ...seg })!;
  }
  bend.id = seg.id;
  bend.yaw = seg.yaw;
  bend.pitch = seg.pitch;
  return { segment: bend, clamped, angle: signed, radius };
}

/** Whether `seg` is a plain straight, which a bend beside it can take length from or give it to. */
function straightPipe(seg: PipeSegment | undefined): seg is PipeSegment {
  return !!seg && !seg.curve && seg.kind !== 'chamber';
}

/**
 * Reshape bend `index` of `segments` to `angle` and `radius`, keeping the pipe its length: what the bend
 * gains or loses comes out of the straights either side of it, half each, or all from one where there is
 * only one, so far as they have it. With no straight either side, the bend keeps its own length, and what
 * was not `changed` follows: a new angle tightens or eases its radius, a new radius turns it further or
 * less. Returns whether it was reshaped.
 */
export function reshapeBendKeepingLength(
  segments: PipeSegment[],
  index: number,
  angle: number,
  radius: number,
  changed: 'angle' | 'radius',
): boolean {
  const seg = segments[index];
  if (!seg) return false;
  const sides = [segments[index - 1], segments[index + 1]].filter(straightPipe);
  if (sides.length === 0 && seg.curve) {
    if (changed === 'angle') radius = seg.length / Math.max(angle, 1e-6);
    else angle = Math.min(seg.length / Math.max(radius, 1e-6), Math.PI - 2 * STRAIGHT_TURN);
  }
  const next = reshapeBend(seg, angle, radius);
  if (!next) return false;
  let change = next.length - seg.length;
  const MIN = 1e-3;
  // Each side gives up to what it has, and what one cannot, the other does.
  for (let pass = 0; pass < 2 && sides.length > 0 && Math.abs(change) > 1e-9; pass++) {
    const share = change / sides.length;
    for (const side of sides) {
      const give = Math.min(share, side.length - MIN);
      side.length -= give;
      change -= give;
    }
  }
  segments[index] = next;
  return true;
}

/**
 * Slide bend `index` along its pipe so the straight before it is `before` m long, the straight after it
 * taking up the difference, so the pipe keeps its length. Returns whether it moved.
 */
export function slideBend(segments: PipeSegment[], index: number, before: number): boolean {
  const prev = segments[index - 1];
  const next = segments[index + 1];
  if (!segments[index]?.curve || !straightPipe(prev) || !straightPipe(next)) return false;
  const MIN = 1e-3;
  const span = prev.length + next.length;
  const at = Math.min(Math.max(before, MIN), span - MIN);
  prev.length = at;
  next.length = span - at;
  return true;
}

// ---------------------------------------------------------------------------
// Snapping
// ---------------------------------------------------------------------------

/**
 * Something a route being drawn can end on.
 *
 * `port` starts a runner, `node` joins an existing junction, `ductEnd` joins the far end of a duct — which
 * means making a junction there if it does not already have one — and `ductSurface` is a T, which splits
 * the duct it lands on. As a *start*, `ductEnd` continues a pipe from its open end; a route cannot start
 * from a `ductSurface`, nor from `turboInlet`, a turbo's inlet flange, which it can only end on. `free` is the
 * fallback: no target, so the route just carries on to a point in
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
  | { kind: 'turboInlet'; point: THREE.Vector3; dir: THREE.Vector3; dia: number; turbo: string }
  /** A turbo's outlet flange with no pipe on it yet, which a route can start from. */
  | { kind: 'turboOutlet'; point: THREE.Vector3; dir: THREE.Vector3; dia: number; turbo: string; node: string }
  /** The start of a loose pipe, which a route can end on to attach it. */
  | { kind: 'looseStart'; point: THREE.Vector3; dir: THREE.Vector3; dia: number; duct: string }
  | { kind: 'free'; point: THREE.Vector3 };

/**
 * Every point a route could snap to, other than anywhere along a duct's side: of its side, only where one
 * of its segments meets the next.
 *
 * The rest of a side is found by raycasting the meshes instead, because "the nearest point on a tube" is what
 * a ray already answers and `PipeMesh.stationAt` already turns a hit into an arc distance.
 *
 * Every port is offered, including ones that already have a pipe. Excluding those would make draw mode
 * useless: a compiled engine gives every cylinder a runner, so *no* port would ever be clickable and a
 * route could only be started from a junction. A cylinder may still only have one pipe — `validateGraph`
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

  for (const turbo of graph.turbos ?? []) {
    const ports = placement.turbos.get(turbo.node);
    if (!ports) continue;
    targets.push({
      kind: 'turboInlet',
      point: new THREE.Vector3(...ports.inlet.point),
      dir: new THREE.Vector3(...ports.inlet.dir),
      dia: ports.inlet.dia,
      turbo: turbo.id,
    });
    if (!endsAt(graph, turbo.node).some((e) => e.end === 'inlet')) {
      targets.push({
        kind: 'turboOutlet',
        point: new THREE.Vector3(...ports.outlet.point),
        dir: new THREE.Vector3(...ports.outlet.dir),
        dia: ports.outlet.dia,
        turbo: turbo.id,
        node: turbo.node,
      });
    }
  }

  for (const duct of graph.ducts) {
    if (duct.from.kind !== 'free' || duct.segments.length === 0) continue;
    const place = placement.ducts.get(duct.id);
    if (!place) continue;
    const dir = layoutPipe(duct.segments, place.origin, place.heading).stations[0]!.direction;
    targets.push({
      kind: 'looseStart',
      point: place.origin.clone(),
      dir: dir.clone(),
      dia: segmentDiameter(duct.segments[0]!, 0),
      duct: duct.id,
    });
  }

  for (const node of nodeOrder(graph)) {
    // A turbo's node is reached through its inlet flange, above.
    if (placement.turbos.has(node)) continue;
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
    // Where one of its segments meets the next: a place to join it, or draw from, as on its side, the pipe
    // split there into a junction. Up to where the bend it is fitted to what it joins with begins, but not
    // within that bend, which is fitted, not drawn.
    const drawn = duct.segments.length - fittedCount(duct);
    const last = duct.fitted ? drawn : drawn - 1;
    let x = 0;
    for (let i = 0; i < last; i++) {
      x += duct.segments[i]!.length;
      targets.push({
        kind: 'ductSurface',
        point: layout.joints[i]!.clone(),
        duct: duct.id,
        x,
        dir: layout.jointDirections[i]!.clone(),
      });
    }
  }

  return targets;
}

/**
 * Whether ending the route `ductId` on `target` would close a loop with no way out: leave the pipes it
 * joins with no open end anywhere, so the gas in them has nowhere to go. An open end is a pipe ending in
 * air, a loose pipe's start, or a junction or turbo nothing leaves yet, whose pipes end in air there
 * (`solverGraph`). A loop that still has one somewhere is allowed.
 *
 * Tried on a copy of the graph, joined as the route would join it, so what counts as joined is exactly what
 * connecting does.
 */
export function closesLoop(graph: ExhaustGraph, ductId: string, target: SnapTarget): boolean {
  if (target.kind === 'port' || target.kind === 'turboOutlet' || target.kind === 'free') return false;
  const g = structuredClone(graph);
  const duct = g.ducts.find((d) => d.id === ductId);
  if (!duct) return false;
  let node: string | null = null;
  switch (target.kind) {
    case 'turboInlet':
      node = g.turbos?.find((t) => t.id === target.turbo)?.node ?? null;
      break;
    case 'node':
      node = target.node;
      break;
    case 'looseStart':
      node = attachToLooseStart(g, ductId, target.duct, [1, 0, 0]);
      break;
    case 'ductSurface':
      node = splitDuctAt(g, target.duct, target.x);
      break;
    case 'ductEnd':
      node = joinDuctEnd(g, target.duct);
      break;
    default:
      return false;
  }
  if (!node) return false;
  duct.to = { kind: 'node', node };

  const leaving = new Set(g.ducts.flatMap((d) => (d.from.kind === 'node' ? [d.from.node] : [])));
  const seen = new Set<string>([duct.id]);
  const frontier = [duct];
  while (frontier.length > 0) {
    const d = frontier.pop()!;
    if (d.to.kind === 'mouth' || d.from.kind === 'free') return false;
    for (const end of [d.from, d.to]) {
      if (end.kind !== 'node') continue;
      if (!leaving.has(end.node)) return false;
      for (const e of endsAt(g, end.node)) {
        if (seen.has(e.duct.id)) continue;
        seen.add(e.duct.id);
        frontier.push(e.duct);
      }
    }
  }
  return true;
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
