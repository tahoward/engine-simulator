/**
 * An X-pipe: two pipes cut and crossed. Each pipe is picked at two points along it, where the X leaves it and
 * where it rejoins it; what ran between goes. From the first point each pipe bends in to where the two cross,
 * runs straight on through it, and bends out to the other pipe's second point, where that pipe carries on as
 * it was. The crossing is one junction of four pipes, which the solver mixes the two banks' gas through.
 *
 * Geometry on the graph and the layout, no scene objects, so it can be tested as the app uses it.
 */

import * as THREE from 'three';

import { newDuctId, newNodeId, splitSegments, type ExhaustDuct, type ExhaustGraph } from '../model/exhaustGraph.js';
import { segmentDiameter, type PipeSegment } from '../model/spec.js';
import { MIN_BEND_BORES, fitCurve, headingOffsetTo } from './drawing.js';
import { bendRadius } from './PipeMesh.js';

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
    const tail = splitSegments(segments, p.to.x)?.[1];
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
