/**
 * Equal-length pipes: from any openings — exhaust ports, or the open ends of pipes — to one place, every
 * pipe the same length, the nearer ones taking a swing on the way to make theirs up (`fitToLength`), and all
 * merging into a collector there. From a port the pipe bends straight out of it; from a pipe's open end it
 * carries that pipe on. On a V or a boxer the ports on the other bank can be given the mirror image instead,
 * merging at the mirrored place.
 *
 * Geometry on the graph and the layout, no scene objects, so it can be tested as the app uses it.
 */

import * as THREE from 'three';

import {
  endsAt,
  newDuctId,
  newNodeId,
  releaseBend,
  tidyJunctions,
  turboAt,
  type DuctDirections,
  type ExhaustDuct,
  type ExhaustGraph,
} from '../model/exhaustGraph.js';
import { makeSegment, physicalBank, physicalBankCount, segmentDiameter, type EngineSpec, type PipeSegment } from '../model/spec.js';
import { fitCurve } from './drawing.js';
import type { ExhaustPlacement, ExhaustPort } from './exhaustLayout.js';
import { layoutPipe } from './PipeMesh.js';
import { fitToLength } from './turboPlacement.js';

/** How long the collector added out of a new merge is, m. */
const COLLECTOR_STUB = 0.15;

/** Something a pipe can be run from: a cylinder's exhaust port, or the open end of a pipe. */
export type HeaderOpening = { kind: 'port'; cylinder: number } | { kind: 'end'; duct: string };

/** An opening, where it is, which way a pipe from it goes, and its bore. */
export interface OpeningAt {
  opening: HeaderOpening;
  point: THREE.Vector3;
  dir: THREE.Vector3;
  bore: number;
}

/**
 * Pipes from `openings`, `length` m each, widening in their bends to the collector's bore, `collectorBore`,
 * and meeting at `merge`, arriving along `axis`.
 */
export interface HeaderPlan {
  openings: OpeningAt[];
  merge: THREE.Vector3;
  axis: THREE.Vector3;
  length: number;
  collectorBore: number;
}

/** A pipe as a plan would build it: its segments, and whether it swings on its way to make its length. */
export interface HeaderPrimary {
  opening: HeaderOpening;
  segments: PipeSegment[];
  swing: boolean;
}

/** A name for an opening that is the same each time, for keeping which are picked. */
export function openingKey(o: HeaderOpening): string {
  return o.kind === 'port' ? `port:${o.cylinder}` : `end:${o.duct}`;
}

/** The cylinders on `bank`, in order. */
export function bankCylinders(spec: EngineSpec, bank: number): number[] {
  return Array.from({ length: spec.cylinders }, (_, c) => c).filter((c) => physicalBank(spec, c) === bank);
}

/** The bore a cylinder's pipe has now where it leaves the port, or `fallback` where it has none. */
export function runnerBore(graph: ExhaustGraph, cylinder: number, fallback: number): number {
  const runner = graph.ducts.find((d) => d.from.kind === 'valve' && d.from.cylinder === cylinder);
  const first = runner?.segments[0];
  return first ? segmentDiameter(first, 0) : fallback;
}

/**
 * Every opening a pipe could be run from, where it lies in `placement`: each pipe's open end, and each port
 * with nothing on it yet. A port with a pipe on it is used; the opening is where that pipe ends, if it is open.
 */
export function headerOpenings(
  graph: ExhaustGraph,
  placement: ExhaustPlacement,
  ports: ExhaustPort[],
  portBore: number,
): OpeningAt[] {
  const out: OpeningAt[] = [];
  ports.forEach((port, cylinder) => {
    const used = graph.ducts.some((d) => d.from.kind === 'valve' && d.from.cylinder === cylinder && d.segments.length > 0);
    if (used) return;
    out.push({
      opening: { kind: 'port', cylinder },
      point: port.position.clone(),
      dir: port.direction.clone().normalize(),
      bore: runnerBore(graph, cylinder, portBore),
    });
  });
  for (const duct of graph.ducts) {
    if (duct.to.kind !== 'mouth' || duct.segments.length === 0) continue;
    const place = placement.ducts.get(duct.id);
    if (!place) continue;
    const swept = layoutPipe(duct.segments, place.origin, place.heading);
    out.push({
      opening: { kind: 'end', duct: duct.id },
      point: swept.joints.at(-1)!.clone(),
      dir: swept.jointDirections.at(-1)!.clone().normalize(),
      bore: segmentDiameter(duct.segments.at(-1)!, 1),
    });
  }
  return out;
}

/**
 * The junction a set of ports' pipes already merge at by themselves, and the one collector out of it, which
 * a header keeps: not a turbo's, and nothing else feeding it.
 */
export function sharedCollector(graph: ExhaustGraph, openings: HeaderOpening[]): { node: string; collector: ExhaustDuct } | null {
  if (openings.length === 0 || openings.some((o) => o.kind !== 'port')) return null;
  const runners = openings.map((o) =>
    graph.ducts.find((d) => d.from.kind === 'valve' && d.from.cylinder === (o as { cylinder: number }).cylinder),
  );
  const first = runners[0]?.to;
  if (!first || first.kind !== 'node' || turboAt(graph, first.node)) return null;
  if (!runners.every((r) => r?.to.kind === 'node' && r.to.node === first.node)) return null;
  const ends = endsAt(graph, first.node);
  const outs = ends.filter((e) => e.end === 'inlet').map((e) => e.duct);
  const feeds = ends.filter((e) => e.end === 'outlet').length;
  return feeds === runners.length && outs.length === 1 ? { node: first.node, collector: outs[0]! } : null;
}

/** The bore of the collector gathering `n` pipes of `bore`, for roughly the gas speed they have. */
export function collectorBore(bore: number, n: number): number {
  return bore * Math.max(Math.sqrt(n) * 0.92, 1);
}

/** The bore a merge's collector starts at: that of the one it keeps, or one sized to gather its pipes. */
export function headerCollectorBore(graph: ExhaustGraph, openings: OpeningAt[]): number {
  const first = sharedCollector(graph, openings.map((o) => o.opening))?.collector.segments[0];
  if (first) return segmentDiameter(first, 0);
  return collectorBore(Math.max(0, ...openings.map((o) => o.bore)), openings.length);
}

/** Where a merge starts out: straight out from the middle of its openings, the way they point. */
export function defaultMerge(openings: OpeningAt[]): { merge: THREE.Vector3; axis: THREE.Vector3 } {
  const centre = new THREE.Vector3();
  const axis = new THREE.Vector3();
  for (const o of openings) {
    centre.add(o.point);
    axis.add(o.dir);
  }
  centre.multiplyScalar(1 / Math.max(openings.length, 1));
  if (axis.lengthSq() < 1e-8) axis.set(0, -1, 0);
  axis.normalize();
  return { merge: centre.addScaledVector(axis, 0.25), axis };
}

/** The shortest a plan's pipes can all be and still reach its merge: that of the one furthest away, m. */
export function shortestHeader(plan: HeaderPlan): number {
  let longest = 0;
  for (const o of plan.openings) longest = Math.max(longest, fitCurve(o.point, o.dir, plan.merge, plan.axis).length);
  return longest;
}

/** Every pipe of `plan`, fitted to its length (`fitToLength`): bending from the opening itself, with no straight. */
export function headerPrimaries(plan: HeaderPlan): HeaderPrimary[] {
  return plan.openings.map((o) => {
    const duct: ExhaustDuct = { id: '', segments: [], from: { kind: 'free', position: [0, 0, 0] }, to: { kind: 'mouth' } };
    const anchor = { point: plan.merge, dir: plan.axis, dia: plan.collectorBore };
    fitToLength(duct, o.point, o.dir, anchor, plan.length, o.bore);
    return { opening: o.opening, segments: duct.segments, swing: duct.swing === true };
  });
}

/**
 * The plane one bank is the mirror image of the other in: the engine's middle, upright through the crank,
 * which the banks lean apart either side of (`exhaustPortOf`) — a V's, or a boxer's laid flat, whose ports
 * both point down. `null` for an engine with one bank.
 */
export function bankMirror(spec: EngineSpec): { point: THREE.Vector3; normal: THREE.Vector3 } | null {
  if (physicalBankCount(spec) < 2) return null;
  return { point: new THREE.Vector3(), normal: new THREE.Vector3(1, 0, 0) };
}

/** `plan`'s merge in the mirror, for the other bank's `openings`. */
export function mirrorPlan(
  plan: HeaderPlan,
  mirror: { point: THREE.Vector3; normal: THREE.Vector3 },
  openings: OpeningAt[],
): HeaderPlan {
  const n = mirror.normal;
  const merge = plan.merge.clone().addScaledVector(n, -2 * plan.merge.clone().sub(mirror.point).dot(n));
  const axis = plan.axis.clone().addScaledVector(n, -2 * plan.axis.dot(n));
  return { ...plan, openings, merge, axis };
}

/**
 * Build `plan` into the graph: each port's pipe becomes its primary, and each open pipe carries on in its
 * own, all bent into a collector fixed at the merge.
 *
 * Where the ports' pipes already merge by themselves into one collector, that collector is kept, and all
 * that follows it: it just starts from the merge now, along its axis. Otherwise they are taken off what they
 * joined, which is tidied (`tidyJunctions`), and a short collector is added out of the merge.
 */
export function applyHeader(graph: ExhaustGraph, plan: HeaderPlan, primaries: HeaderPrimary[], dirs?: DuctDirections): void {
  if (primaries.length === 0) return;
  const runnerOf = (p: HeaderPrimary) =>
    p.opening.kind === 'port'
      ? graph.ducts.find((d) => d.from.kind === 'valve' && d.from.cylinder === (p.opening as { cylinder: number }).cylinder)
      : undefined;

  let node = sharedCollector(graph, plan.openings.map((o) => o.opening))?.node ?? null;
  if (!node) {
    const left = new Set<string>();
    for (const p of primaries) {
      const r = runnerOf(p);
      if (!r || r.to.kind !== 'node') continue;
      left.add(r.to.node);
      r.to = { kind: 'mouth' };
      releaseBend(r);
      for (const d of graph.ducts) if (d.continues === r.id) delete d.continues;
    }
    tidyJunctions(graph, left, dirs);
    node = newNodeId(graph);
  }

  graph.junctions = (graph.junctions ?? []).filter((j) => j.node !== node);
  const { merge, axis } = plan;
  // Not a header's collector, whose primaries each keep their own bore: these widen to meet it.
  graph.junctions.push({ node, position: [merge.x, merge.y, merge.z], axis: [axis.x, axis.y, axis.z] });

  let joined = 0;
  for (const p of primaries) {
    const added = p.segments.map((sg) => makeSegment(sg));
    let duct: ExhaustDuct | undefined;
    if (p.opening.kind === 'port') {
      duct = runnerOf(p);
      if (!duct) continue;
      duct.segments = added;
      // Straight out of its port: the heading is the port's own.
      duct.headingYaw = 0;
      duct.headingPitch = 0;
      delete duct.headingFrame;
    } else {
      const id = p.opening.duct;
      duct = graph.ducts.find((d) => d.id === id && d.to.kind === 'mouth');
      if (!duct) continue;
      duct.segments = [...duct.segments, ...added];
    }
    duct.to = { kind: 'node', node };
    duct.fitted = true;
    if (p.swing) duct.swing = true;
    else delete duct.swing;
    joined++;
  }
  if (joined === 0) {
    graph.junctions = graph.junctions.filter((j) => j.node !== node);
    return;
  }

  // The collector leaves along the merge's axis, which a pipe from a fixed junction is turned off.
  const collector = graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === node);
  if (collector) {
    collector.headingYaw = 0;
    collector.headingPitch = 0;
    delete collector.headingFrame;
  } else {
    const dia = plan.collectorBore;
    graph.ducts.push({
      id: newDuctId(graph, 'collector'),
      role: 'collector',
      segments: [makeSegment({ kind: 'pipe', length: COLLECTOR_STUB, dIn: dia, dOut: dia })],
      from: { kind: 'node', node },
      to: { kind: 'mouth' },
      headingYaw: 0,
      headingPitch: 0,
    });
  }
}

/** The collector a new merge would add, as a ghost shows it: from the merge along its axis. */
export function collectorGhost(plan: HeaderPlan): PipeSegment[] {
  const dia = plan.collectorBore;
  return [makeSegment({ kind: 'pipe', length: COLLECTOR_STUB, dIn: dia, dOut: dia })];
}
