/**
 * Equal-length headers, as the header tool builds them: every primary on a bank the same length into one
 * collector where the triad put it, and the other bank the mirror image.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';

import { compileCollectorLayout, compileLayout, junctionAt, validateGraph, type ExhaustGraph } from '../src/model/exhaustGraph.js';
import { defaultConfig, makeSegment, type EngineSpec } from '../src/model/spec.js';
import { EngineMesh } from '../src/scene/EngineMesh.js';
import { layoutGraph, type ExhaustPort } from '../src/scene/exhaustLayout.js';
import {
  applyHeader,
  bankCylinders,
  bankMirror,
  defaultMerge,
  headerCollectorBore,
  headerPrimaries,
  mirrorPlan,
  shortestHeader,
  type HeaderPlan,
} from '../src/scene/headerTool.js';
import { layoutPipe } from '../src/scene/PipeMesh.js';
import { refitBends } from '../src/scene/turboPlacement.js';

const v8 = { ...defaultConfig().engine, cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' } as EngineSpec;
const portsOf = (spec: EngineSpec): ExhaustPort[] => {
  const mesh = new EngineMesh(spec, new THREE.Plane(new THREE.Vector3(0, 0, -1), 0.001));
  return Array.from({ length: mesh.bankCount }, (_, i) => mesh.exhaustPort(i));
};
const total = (graph: ExhaustGraph, cylinder: number) =>
  graph.ducts.find((d) => d.from.kind === 'valve' && d.from.cylinder === cylinder)!.segments.reduce((a, s) => a + s.length, 0);

/** Bank 0's header, its collector below and behind where it starts out, a little longer than it has to be. */
function plan(ports: ExhaustPort[], graph: ExhaustGraph): HeaderPlan {
  const cylinders = bankCylinders(v8, 0);
  const { merge, axis } = defaultMerge(ports, cylinders);
  merge.add(new THREE.Vector3(0, -0.15, 0.1));
  axis.set(axis.x, -1, 0.3).normalize();
  const p: HeaderPlan = { cylinders, merge, axis, length: 0, bore: 0.042, collectorBore: headerCollectorBore(graph, cylinders, 0.042) };
  p.length = shortestHeader(ports, p) + 0.08;
  return p;
}

describe('equal-length headers', () => {
  it('builds every primary on the bank the same length, into one collector where it was put', () => {
    const ports = portsOf(v8);
    const graph = compileCollectorLayout(v8, [makeSegment({ length: 0.4 })], [makeSegment({ length: 0.5, dIn: 0.06 })]);
    const p = plan(ports, graph);
    const collectors = graph.ducts.filter((d) => d.role === 'collector').length;
    applyHeader(graph, p, headerPrimaries(ports, p));

    const nodes = new Set(p.cylinders.map((c) => {
      const to = graph.ducts.find((d) => d.from.kind === 'valve' && d.from.cylinder === c)!.to;
      return to.kind === 'node' ? to.node : '';
    }));
    expect(nodes.size).toBe(1);
    const node = [...nodes][0]!;
    expect(junctionAt(graph, node)!.position).toEqual([p.merge.x, p.merge.y, p.merge.z]);
    // The collector it had is the one it keeps.
    expect(graph.ducts.filter((d) => d.role === 'collector')).toHaveLength(collectors);
    expect(validateGraph(graph, 8)).toEqual([]);

    // Laid out and fitted again as every rebuild does, each is still its length and ends at the collector.
    refitBends(graph, ports, v8);
    const placement = layoutGraph(ports, graph);
    const collector = graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === node)!;
    for (const c of p.cylinders) {
      expect(total(graph, c)).toBeCloseTo(p.length, 2);
      const duct = graph.ducts.find((d) => d.from.kind === 'valve' && d.from.cylinder === c)!;
      // Bent from the port itself, at the port's bore, and meeting the collector at its bore.
      expect(duct.segments.every((sg) => sg.curve)).toBe(true);
      expect(duct.segments[0]!.dIn).toBeCloseTo(0.042, 9);
      expect(duct.segments.at(-1)!.dOut).toBeCloseTo(collector.segments[0]!.dIn, 9);
      const place = placement.ducts.get(duct.id)!;
      const end = layoutPipe(duct.segments, place.origin, place.heading).joints.at(-1)!;
      expect(end.distanceTo(p.merge)).toBeLessThan(1e-3);
    }
  });

  it('gives the other bank the mirror image', () => {
    const ports = portsOf(v8);
    const graph = compileCollectorLayout(v8, [makeSegment({ length: 0.4 })], [makeSegment({ length: 0.5, dIn: 0.06 })]);
    const p = plan(ports, graph);
    const mirror = bankMirror(v8, ports)!;
    expect(mirror).not.toBeNull();
    const other = mirrorPlan(p, mirror, bankCylinders(v8, 1));
    for (const q of [p, other]) applyHeader(graph, q, headerPrimaries(ports, q));
    expect(validateGraph(graph, 8)).toEqual([]);

    // Across the engine's middle, the other way along nothing: the merge and the way it points, mirrored in x.
    expect(other.merge.x).toBeCloseTo(-p.merge.x, 9);
    expect(other.merge.y).toBeCloseTo(p.merge.y, 9);
    expect(other.merge.z).toBeCloseTo(p.merge.z, 9);
    expect(other.axis.x).toBeCloseTo(-p.axis.x, 9);
    for (const c of other.cylinders) expect(total(graph, c)).toBeCloseTo(p.length, 2);
  });

  it('adds a collector where the primaries merged into nothing of their own', () => {
    const twin = { ...defaultConfig().engine, cylinders: 2, vAngle: 0, exhaustLayout: '2into2' } as EngineSpec;
    const ports = portsOf(twin);
    const graph = compileLayout(twin, [makeSegment({ length: 0.4 })], []);
    expect(graph.ducts.every((d) => d.to.kind === 'mouth')).toBe(true);
    const cylinders = bankCylinders(twin, 0);
    const { merge, axis } = defaultMerge(ports, cylinders);
    const p: HeaderPlan = { cylinders, merge, axis, length: 0, bore: 0.042, collectorBore: headerCollectorBore(graph, cylinders, 0.042) };
    p.length = shortestHeader(ports, p) + 0.05;
    applyHeader(graph, p, headerPrimaries(ports, p));
    const collector = graph.ducts.find((d) => d.role === 'collector')!;
    expect(collector.from.kind).toBe('node');
    expect(collector.to).toEqual({ kind: 'mouth' });
    for (const d of graph.ducts) if (d.from.kind === 'valve') expect(d.segments.at(-1)!.dOut).toBeCloseTo(collector.segments[0]!.dIn, 9);
    expect(bankMirror(twin, ports)).toBeNull();
    expect(validateGraph(graph, 2)).toEqual([]);
  });
});
