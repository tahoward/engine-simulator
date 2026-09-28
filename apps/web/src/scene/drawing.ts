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
  disconnectEnd,
  endsAt,
  junctionAt,
  newDuctId,
  nodeOrder,
  releaseBend,
  removeDuct,
  turboAt,
  type DuctDirections,
  type ExhaustDuct,
  type ExhaustGraph,
} from '../model/exhaustGraph.js';
import { curveFrame, curveInWorld, curvePath, layoutPipe, turnBetween, turnHeading } from './PipeMesh.js';
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
  };
  for (const d of graph.ducts) if (d.continues === duct.id) d.continues = rest.id;
  duct.segments = duct.segments.slice(0, index);
  duct.to = { kind: 'mouth' };
  delete duct.fitted;
  delete duct.swing;
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
 * Turn a loose pipe round, end for end, where it lies: its far end becomes where it starts, so a pipe
 * drawn into that end can carry on through it. Every segment keeps its length and its place, its bores
 * swapped end for end and a bend run the other way, so it looks just as it did.
 */
export function flipLoosePipe(duct: ExhaustDuct, place: DuctPlacement): void {
  if (duct.from.kind !== 'free' || duct.segments.length === 0) return;
  const swept = layoutPipe(duct.segments, place.origin, place.heading);
  const points = [place.origin.clone(), ...swept.joints.map((p) => p.clone())];
  // Each segment's direction where it starts and where it ends.
  const starts: THREE.Vector3[] = [];
  duct.segments.forEach((_, i) => {
    const first = swept.stations.find((st) => st.segment === i)!;
    starts.push(first.direction.clone());
  });
  const ends = swept.jointDirections.map((d) => d.clone());

  const flipped: PipeSegment[] = [];
  let prevEnd: THREE.Vector3 | null = null;
  let heading = new THREE.Vector3(1, 0, 0);
  for (let i = duct.segments.length - 1; i >= 0; i--) {
    const seg = duct.segments[i]!;
    const startDir = ends[i]!.clone().negate();
    const endDir = starts[i]!.clone().negate();
    const corner = prevEnd ? turnBetween(prevEnd, startDir) : { yaw: 0, pitch: 0 };
    if (!prevEnd) heading = startDir.clone();
    const next: Partial<PipeSegment> = {
      ...seg,
      id: undefined,
      dIn: seg.kind === 'chamber' ? seg.dIn : seg.dOut,
      dOut: seg.kind === 'chamber' ? seg.dOut : seg.dIn,
      yaw: corner.yaw,
      pitch: corner.pitch,
      offsetIn: seg.offsetOut,
      offsetOut: seg.offsetIn,
      curve: undefined,
    };
    if (seg.curve) {
      const f = curveFrame(startDir);
      const local = (v: THREE.Vector3): [number, number, number] => [v.dot(f.x), v.dot(f.y), v.dot(f.z)];
      next.curve = { end: local(points[i]!.clone().sub(points[i + 1]!)), dir: local(endDir) };
    }
    flipped.push(makeSegment(next));
    prevEnd = endDir;
  }
  const h = turnBetween(new THREE.Vector3(1, 0, 0), heading);
  const start = points[points.length - 1]!;
  duct.from = { kind: 'free', position: [start.x, start.y, start.z] };
  duct.headingYaw = h.yaw;
  duct.headingPitch = h.pitch;
  duct.headingFrame = 'world';
  duct.segments = flipped;
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
}

/** The shape of `segments` leaving along `heading`. */
export function pipeShape(segments: PipeSegment[], heading: THREE.Vector3): PipeShape {
  const shape: PipeShape = { starts: [], ends: [], chords: [] };
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
 * `null`, as for the pipe that is itself carried on.
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
  const ends = endsAt(graph, node);
  // A junction that has been moved is where it was put, every pipe into it arriving along the first
  // leaving it.
  const pinned = junctionAt(graph, node);
  if (pinned) {
    const out = ends.find((e) => e.end === 'inlet')?.duct;
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
  const primary = onward ? ends.find((e) => e.end === 'outlet' && e.duct.id === onward.continues)?.duct : undefined;
  if (!primary || !onward || primary.id === ductId) return null;
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
 * Every point a route could snap to, other than duct surfaces.
 *
 * Surfaces are found by raycasting the meshes instead, because "the nearest point on a tube" is what a
 * ray already answers and `PipeMesh.stationAt` already turns a hit into an arc distance.
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
