/**
 * Equal-length headers: every primary on a bank the same length, each straight out of its port and then one
 * smooth bend into a collector put where the user says, the nearer primaries taking a swing on the way to
 * make their length up (`fitToLength`). On a V or a boxer the other bank can be given the mirror image.
 *
 * Geometry on the graph and the ports, no scene objects, so it can be tested as the app uses it.
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
import type { ExhaustPort } from './exhaustLayout.js';
import { fitToLength } from './turboPlacement.js';

/** How long the collector added out of a new header's merge is, m. */
const COLLECTOR_STUB = 0.15;

/**
 * One bank's header: its cylinders' primaries, `length` m each, leaving their ports at `bore` and widening
 * in their bends to the collector's, `collectorBore`, which they meet at `merge`, arriving along `axis`.
 */
export interface HeaderPlan {
  cylinders: number[];
  merge: THREE.Vector3;
  axis: THREE.Vector3;
  length: number;
  bore: number;
  collectorBore: number;
}

/** A primary as a header would build it: its segments, and whether it swings on its way to make its length. */
export interface HeaderPrimary {
  cylinder: number;
  segments: PipeSegment[];
  swing: boolean;
}

/** The cylinders on `bank`, in order. */
export function bankCylinders(spec: EngineSpec, bank: number): number[] {
  return Array.from({ length: spec.cylinders }, (_, c) => c).filter((c) => physicalBank(spec, c) === bank);
}

/** The bore a bank's primaries have now, where it starts at the port, or `fallback` where it has none. */
export function runnerBore(graph: ExhaustGraph, cylinder: number, fallback: number): number {
  const runner = graph.ducts.find((d) => d.from.kind === 'valve' && d.from.cylinder === cylinder);
  const first = runner?.segments[0];
  return first ? segmentDiameter(first, 0) : fallback;
}

/**
 * The junction a bank's primaries already merge at by themselves, and the one collector out of it, which a
 * header keeps: not a turbo's, and nothing else feeding it.
 */
export function sharedCollector(graph: ExhaustGraph, cylinders: number[]): { node: string; collector: ExhaustDuct } | null {
  const runners = cylinders.map((c) => graph.ducts.find((d) => d.from.kind === 'valve' && d.from.cylinder === c));
  const first = runners[0]?.to;
  if (!first || first.kind !== 'node' || turboAt(graph, first.node)) return null;
  if (!runners.every((r) => r?.to.kind === 'node' && r.to.node === first.node)) return null;
  const ends = endsAt(graph, first.node);
  const outs = ends.filter((e) => e.end === 'inlet').map((e) => e.duct);
  const feeds = ends.filter((e) => e.end === 'outlet').length;
  return feeds === runners.length && outs.length === 1 ? { node: first.node, collector: outs[0]! } : null;
}

/** The bore a bank's header collector starts at: that of the one it keeps, or one sized to gather them. */
export function headerCollectorBore(graph: ExhaustGraph, cylinders: number[], bore: number): number {
  const first = sharedCollector(graph, cylinders)?.collector.segments[0];
  return first ? segmentDiameter(first, 0) : collectorBore(bore, cylinders.length);
}

/** Where a bank's collector starts out: straight out from the middle of its ports, the way they point. */
export function defaultMerge(ports: ExhaustPort[], cylinders: number[]): { merge: THREE.Vector3; axis: THREE.Vector3 } {
  const centre = new THREE.Vector3();
  const axis = new THREE.Vector3();
  for (const c of cylinders) {
    centre.add(ports[c]!.position);
    axis.add(ports[c]!.direction);
  }
  centre.multiplyScalar(1 / Math.max(cylinders.length, 1));
  if (axis.lengthSq() < 1e-8) axis.set(1, 0, 0);
  axis.normalize();
  return { merge: centre.addScaledVector(axis, 0.25), axis };
}

/** The shortest a header's primaries can all be and still reach its merge: that of the one furthest away, m. */
export function shortestHeader(ports: ExhaustPort[], plan: HeaderPlan): number {
  let longest = 0;
  for (const c of plan.cylinders) {
    const port = ports[c];
    if (!port) continue;
    longest = Math.max(longest, fitCurve(port.position, port.direction, plan.merge, plan.axis).length);
  }
  return longest;
}

/** Every primary of `plan`, fitted to its length (`fitToLength`): bending from the port itself, with no straight. */
export function headerPrimaries(ports: ExhaustPort[], plan: HeaderPlan): HeaderPrimary[] {
  const primaries: HeaderPrimary[] = [];
  for (const cylinder of plan.cylinders) {
    const port = ports[cylinder];
    if (!port) continue;
    const duct: ExhaustDuct = {
      id: '',
      segments: [],
      from: { kind: 'valve', cylinder },
      to: { kind: 'mouth' },
    };
    const anchor = { point: plan.merge, dir: plan.axis, dia: plan.collectorBore };
    fitToLength(duct, port.position, port.direction, anchor, plan.length, plan.bore);
    primaries.push({ cylinder, segments: duct.segments, swing: duct.swing === true });
  }
  return primaries;
}

/** The bore of the collector gathering `n` primaries of `bore`, for roughly the gas speed they have. */
export function collectorBore(bore: number, n: number): number {
  return bore * Math.max(Math.sqrt(n) * 0.92, 1);
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

/** `plan` in the mirror, for the other bank's `cylinders`. */
export function mirrorPlan(
  plan: HeaderPlan,
  mirror: { point: THREE.Vector3; normal: THREE.Vector3 },
  cylinders: number[],
): HeaderPlan {
  const n = mirror.normal;
  const merge = plan.merge.clone().addScaledVector(n, -2 * plan.merge.clone().sub(mirror.point).dot(n));
  const axis = plan.axis.clone().addScaledVector(n, -2 * plan.axis.dot(n));
  return { ...plan, cylinders, merge, axis };
}

/**
 * Build `plan` into the graph: each of its cylinders' pipes becomes its primary, bent into a collector fixed
 * at the merge.
 *
 * Where the bank's primaries already merge by themselves into one collector, that collector is kept, and
 * all that follows it: it just starts from the merge now, along its axis. Otherwise they are taken off what
 * they joined, which is tidied (`tidyJunctions`), and a short collector is added out of the merge.
 */
export function applyHeader(graph: ExhaustGraph, plan: HeaderPlan, primaries: HeaderPrimary[], dirs?: DuctDirections): void {
  const runners = primaries
    .map((p) => graph.ducts.find((d) => d.from.kind === 'valve' && d.from.cylinder === p.cylinder))
    .filter((d): d is ExhaustDuct => d !== undefined);
  if (runners.length === 0) return;

  let node = sharedCollector(graph, plan.cylinders)?.node ?? null;

  if (!node) {
    const left = new Set<string>();
    for (const r of runners) {
      if (r.to.kind !== 'node') continue;
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

  for (const r of runners) {
    const p = primaries.find((q) => q.cylinder === (r.from as { cylinder: number }).cylinder)!;
    r.segments = p.segments.map((sg) => makeSegment(sg));
    r.to = { kind: 'node', node };
    r.fitted = true;
    if (p.swing) r.swing = true;
    else delete r.swing;
    // Straight out of its port: the heading is the port's own.
    r.headingYaw = 0;
    r.headingPitch = 0;
    delete r.headingFrame;
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

/** The collector a new header would add, as a ghost shows it: from the merge along its axis. */
export function collectorGhost(plan: HeaderPlan): PipeSegment[] {
  const dia = plan.collectorBore;
  return [makeSegment({ kind: 'pipe', length: COLLECTOR_STUB, dIn: dia, dOut: dia })];
}
