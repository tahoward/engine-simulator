/**
 * An X-pipe: two pipes cut and crossed. Each pipe is picked at two points along it, where the X leaves it and
 * where it rejoins it; what ran between goes. From the first point each pipe bends in to where the two cross,
 * runs straight on through it, and bends out to the other pipe's second point, where that pipe carries on as
 * it was. The crossing is one junction of four pipes, which the solver mixes the two banks' gas through.
 *
 * Geometry on the graph and the layout, no scene objects, so it can be tested as the app uses it.
 */

import * as THREE from 'three';

import { junctionAt, newDuctId, newNodeId, splitSegments, type ExhaustDuct, type ExhaustGraph } from '../model/exhaustGraph.js';
import type { Vec3 } from '../model/geometry.js';
import { segmentDiameter, type PipeSegment } from '../model/spec.js';
import { lockedFrom } from '../model/turbo.js';
import { MIN_BEND_BORES, fitCurve, headingOffsetTo } from './drawing.js';
import type { DuctPlacement } from './exhaustLayout.js';
import { bendRadius, layoutPipe, turnHeading } from './PipeMesh.js';

/** Where the ways a pipe leaving a junction in the world's terms are measured from. */
const WORLD_X = new THREE.Vector3(1, 0, 0);

/** Least distance between the two points picked on a pipe, m. */
const MIN_SPAN = 0.02;

/** A point picked on a pipe: the duct, how far along it, where that is, the way the pipe runs there, its bore. */
export interface PipePoint {
  duct: string;
  x: number;
  point: THREE.Vector3;
  dir: THREE.Vector3;
  bore: number;
}

/** One of the two pipes crossed: the point the X leaves it at, and the point it rejoins it at. */
export interface XPair {
  duct: string;
  from: PipePoint;
  to: PipePoint;
}

/** The two pipes an X crosses, and where they cross. */
export interface XPlan {
  pairs: [XPair, XPair];
  cross: THREE.Vector3;
}

/**
 * One pipe of an X as it would be built: `into`, the bend from pair `pair`'s first point into the crossing,
 * and `out`, the bend from the crossing out to its second point. `into` runs on into the *other* pair's `out`.
 */
export interface XLeg {
  pair: number;
  into: PipeSegment;
  out: PipeSegment;
  /** The way `out` leaves the crossing, which the other pipe's `into` arrives along. */
  leaving: THREE.Vector3;
  /** Whether either bend turns tighter than a pipe can be bent (`MIN_BEND_BORES`). */
  tight: boolean;
}

/** The bore of a duct's segments at arc distance `x`, m. */
export function boreAt(segments: PipeSegment[], x: number): number {
  let acc = 0;
  for (const seg of segments) {
    if (x <= acc + seg.length) return segmentDiameter(seg, seg.length > 0 ? (x - acc) / seg.length : 0);
    acc += seg.length;
  }
  const last = segments.at(-1);
  return last ? segmentDiameter(last, 1) : 0;
}

/**
 * The places on a pipe a point snaps to: where it starts, where each of its segments meets the next, and where
 * it ends, each with the way the pipe runs arriving there. Not within a bend fitted to what it joins, which is
 * not cut, and so not its end where it has one.
 */
export function snapPoints(duct: ExhaustDuct, place: DuctPlacement): PipePoint[] {
  if (duct.segments.length === 0) return [];
  const layout = layoutPipe(duct.segments, place.origin, place.heading);
  const locked = lockedFrom(duct);
  const out: PipePoint[] = [
    { duct: duct.id, x: 0, point: place.origin.clone(), dir: place.heading.clone().normalize(), bore: boreAt(duct.segments, 0) },
  ];
  let x = 0;
  duct.segments.forEach((seg, i) => {
    x += seg.length;
    // The end of segment `i`, which a cut there keeps whole: not a fitted one.
    if (locked !== null && i >= locked) return;
    out.push({ duct: duct.id, x, point: layout.joints[i]!.clone(), dir: layout.jointDirections[i]!.clone(), bore: boreAt(duct.segments, x) });
  });
  return out;
}

/** A duct's length, m. */
function lengthOf(segments: PipeSegment[]): number {
  return segments.reduce((a, seg) => a + seg.length, 0);
}

/**
 * The two pipes `points` pick, each at the two points on it in the order it runs, or why they do not: an X
 * needs two points on each of two pipes.
 */
export function xPairs(points: PipePoint[]): { pairs: [XPair, XPair] } | { error: string } {
  const byDuct = new Map<string, PipePoint[]>();
  for (const p of points) byDuct.set(p.duct, [...(byDuct.get(p.duct) ?? []), p]);
  if ([...byDuct.values()].some((ps) => ps.length > 2)) return { error: 'Two points on each pipe, no more' };
  if (points.length < 4 || byDuct.size !== 2) {
    const left = 4 - points.length;
    return { error: left > 0 ? `Click ${left} more point${left === 1 ? '' : 's'}: two on each of two pipes` : 'Two points on each of two pipes' };
  }
  const pairs = [...byDuct.entries()].map(([duct, ps]) => {
    const [a, b] = [...ps].sort((p, q) => p.x - q.x) as [PipePoint, PipePoint];
    return { duct, from: a, to: b };
  });
  if (pairs.some((p) => p.to.x - p.from.x < MIN_SPAN)) return { error: 'The two points on a pipe are too close together' };
  return { pairs: pairs as [XPair, XPair] };
}

/** Where an X crosses before it is moved: amid its four points. */
export function defaultCrossing(pairs: [XPair, XPair]): THREE.Vector3 {
  const c = new THREE.Vector3();
  for (const p of pairs) c.add(p.from.point).add(p.to.point);
  return c.multiplyScalar(1 / 4);
}

/** The four bends of `plan`, a pair of them for each pipe. */
export function xLegs(plan: XPlan): [XLeg, XLeg] {
  const c = plan.cross;
  const leaving = plan.pairs.map((p) => {
    const d = p.to.point.clone().sub(c);
    return d.lengthSq() > 1e-12 ? d.normalize() : p.to.dir.clone();
  });
  return plan.pairs.map((p, i) => {
    const onto = leaving[1 - i]!;
    const into = fitCurve(p.from.point, p.from.dir, c, onto, { dIn: p.from.bore, dOut: p.from.bore });
    const out = fitCurve(c, leaving[i]!, p.to.point, p.to.dir, { dIn: p.to.bore, dOut: p.to.bore });
    const tight =
      bendRadius(p.from.point, p.from.dir, c, onto) < MIN_BEND_BORES * p.from.bore ||
      bendRadius(c, leaving[i]!, p.to.point, p.to.dir) < MIN_BEND_BORES * p.to.bore;
    return { pair: i, into, out, leaving: leaving[i]!.clone(), tight };
  }) as [XLeg, XLeg];
}

/**
 * Build `plan` into the graph. Each pipe keeps its identity up to its first point and bends from there into
 * the crossing, which is fixed where the plan puts it. Past its second point it is a new pipe, leaving the
 * crossing and bending out to that point, then carrying on as the pipe did, to wherever it went. What ran
 * between the two points goes. Returns the crossing's node, or `null` where the plan does not cut.
 */
export function applyXPipe(graph: ExhaustGraph, plan: XPlan): string | null {
  const ducts = plan.pairs.map((p) => graph.ducts.find((d) => d.id === p.duct));
  if (ducts.some((d) => !d)) return null;
  const cuts = plan.pairs.map((p, i) => {
    const segments = ducts[i]!.segments;
    // Rejoining where the pipe ends leaves nothing of it to carry on: the leg ends where it did.
    const tail = p.to.x >= lengthOf(segments) - 1e-6 ? [] : splitSegments(segments, p.to.x)?.[1];
    const head = p.from.x > 1e-6 ? splitSegments(segments, p.from.x)?.[0] : [];
    return head && tail ? { head, tail } : null;
  });
  if (cuts.some((c) => !c)) return null;

  const legs = xLegs(plan);
  const node = newNodeId(graph);
  const leavingIds: string[] = [];
  ducts.forEach((d, i) => {
    const duct = d!;
    const { tail } = cuts[i]!;
    const leg = legs[i]!;
    const turn = headingOffsetTo(WORLD_X, leg.leaving);
    const out: ExhaustDuct = {
      id: newDuctId(graph, `${duct.id}-`),
      segments: [leg.out, ...tail],
      from: { kind: 'node', node },
      to: duct.to,
      headingYaw: turn.yaw,
      headingPitch: turn.pitch,
      headingFrame: 'world',
      // The bend it is fitted to what it joins with is at its far end, so goes with the far part.
      ...(duct.fitted ? { fitted: true as const } : {}),
      ...(duct.swing ? { swing: true as const } : {}),
      ...(duct.square ? { square: true as const } : {}),
    };
    // What carried straight on from the pipe's far end carries on from the part that now ends there.
    for (const on of graph.ducts) if (on.continues === duct.id && on.from.kind === 'node') on.continues = out.id;
    graph.ducts.splice(graph.ducts.indexOf(duct) + 1, 0, out);
    leavingIds.push(out.id);
  });
  ducts.forEach((d, i) => {
    const duct = d!;
    duct.segments = [...cuts[i]!.head, legs[i]!.into];
    duct.to = { kind: 'node', node };
    duct.fitted = true;
    delete duct.swing;
    delete duct.square;
  });

  const c = plan.cross;
  const axis = legs[0].leaving.clone().add(legs[1].leaving);
  if (axis.lengthSq() < 1e-12) axis.copy(legs[0].leaving);
  axis.normalize();
  (graph.junctions ??= []).push({
    node,
    position: [c.x, c.y, c.z],
    axis: [axis.x, axis.y, axis.z],
    // Each pipe runs on across into the other's leg.
    through: { [ducts[0]!.id]: leavingIds[1]!, [ducts[1]!.id]: leavingIds[0]! },
  });
  return node;
}

/** The pipes leaving the X-pipe crossing at `node`, or none where it is not one. */
export function crossingLegs(graph: ExhaustGraph, node: string): ExhaustDuct[] {
  const pin = junctionAt(graph, node);
  if (!pin?.through) return [];
  const ids = new Set(Object.values(pin.through));
  return graph.ducts.filter((d) => ids.has(d.id) && d.from.kind === 'node' && d.from.node === node);
}

/**
 * Move the X-pipe crossing at `node` to `position`. Each pipe leaving it bends out again from there to where
 * it bent out to before, arriving the way it did, so what carries on from it stays put; the pipes into it,
 * fitted, follow it when bends are fitted again. Returns whether `node` is a crossing.
 */
export function moveCrossing(graph: ExhaustGraph, node: string, position: Vec3): boolean {
  const pin = junctionAt(graph, node);
  const legs = crossingLegs(graph, node);
  if (!pin || legs.length === 0) return false;
  const from = new THREE.Vector3(...pin.position);
  const to = new THREE.Vector3(...position);
  const axis = new THREE.Vector3();
  for (const leg of legs) {
    const bend = leg.segments[0];
    if (!bend) continue;
    const frame = leg.headingFrame === 'world' ? WORLD_X : new THREE.Vector3(...pin.axis);
    const swept = layoutPipe([bend], from, turnHeading(frame, leg.headingYaw ?? 0, leg.headingPitch ?? 0));
    const end = swept.joints[0]!;
    const arriving = swept.jointDirections[0]!;
    const out = end.clone().sub(to);
    const leaving = out.lengthSq() > 1e-12 ? out.normalize() : arriving.clone();
    leg.segments[0] = { ...fitCurve(to, leaving, end, arriving, { dIn: bend.dIn, dOut: bend.dOut }), id: bend.id };
    const turn = headingOffsetTo(WORLD_X, leaving);
    leg.headingYaw = turn.yaw;
    leg.headingPitch = turn.pitch;
    leg.headingFrame = 'world';
    axis.add(leaving);
  }
  pin.position = [...position];
  if (axis.lengthSq() > 1e-12) {
    axis.normalize();
    pin.axis = [axis.x, axis.y, axis.z];
  }
  return true;
}

/** Set the bore the pipes meet at in the X-pipe crossing at `node`, m: where each leg sets off from it, which the pipes into it bend in to meet. */
export function setCrossingBore(graph: ExhaustGraph, node: string, bore: number): void {
  for (const leg of crossingLegs(graph, node)) {
    const bend = leg.segments[0];
    if (bend) bend.dIn = bore;
  }
}

/** The bore the pipes meet at in the X-pipe crossing at `node`, m, or 0 where it is not one. */
export function crossingBore(graph: ExhaustGraph, node: string): number {
  const bend = crossingLegs(graph, node)[0]?.segments[0];
  return bend ? segmentDiameter(bend, 0) : 0;
}
