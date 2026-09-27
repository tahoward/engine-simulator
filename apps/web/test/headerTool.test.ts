/**
 * Equal-length pipes, as the header tool builds them: from any openings, every pipe the same length into
 * one collector where the triad put it, and a bank's ports mirrored onto the other bank's.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';

import {
  compileCollectorLayout,
  compileLayout,
  junctionAt,
  placeLoosePipe,
  validateGraph,
  type ExhaustGraph,
} from '../src/model/exhaustGraph.js';
import { defaultConfig, makeSegment, type EngineSpec } from '../src/model/spec.js';
import { EngineMesh } from '../src/scene/EngineMesh.js';
import { layoutGraph, type ExhaustPort } from '../src/scene/exhaustLayout.js';
import {
  applyHeader,
  bankCylinders,
  bankMirror,
  defaultMerge,
  headerCollectorBore,
  headerOpenings,
  headerPrimaries,
  mirrorPlan,
  shortestHeader,
  type HeaderPlan,
  type OpeningAt,
} from '../src/scene/headerTool.js';
import { layoutPipe } from '../src/scene/PipeMesh.js';
import { refitBends } from '../src/scene/turboPlacement.js';

const v8 = { ...defaultConfig().engine, cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' } as EngineSpec;
const boxer = { ...defaultConfig().engine, cylinders: 4, vAngle: 180, crankType: 'boxer', exhaustLayout: 'perBank' } as EngineSpec;
const portsOf = (spec: EngineSpec): ExhaustPort[] => {
  const mesh = new EngineMesh(spec, new THREE.Plane(new THREE.Vector3(0, 0, -1), 0.001));
  return Array.from({ length: mesh.bankCount }, (_, i) => mesh.exhaustPort(i));
};
const runner = (graph: ExhaustGraph, cylinder: number) =>
  graph.ducts.find((d) => d.from.kind === 'valve' && d.from.cylinder === cylinder)!;
const total = (segments: { length: number }[]) => segments.reduce((a, s) => a + s.length, 0);

/** The openings of `graph` the cylinders' ports give, which have nothing on them yet. */
function portOpenings(graph: ExhaustGraph, ports: ExhaustPort[], cylinders: number[]): OpeningAt[] {
  const all = headerOpenings(graph, layoutGraph(ports, graph), ports, 0.042);
  return cylinders.map((c) => all.find((o) => o.opening.kind === 'port' && o.opening.cylinder === c)!);
}

/** A plan from `openings`, merging below and behind where they start out, a little longer than it has to be. */
function planFrom(graph: ExhaustGraph, openings: OpeningAt[]): HeaderPlan {
  const { merge, axis } = defaultMerge(openings);
  merge.add(new THREE.Vector3(0, -0.15, 0.1));
  axis.set(axis.x, -1, 0.3).normalize();
  const p: HeaderPlan = { openings, merge, axis, length: 0, collectorBore: headerCollectorBore(graph, openings) };
  p.length = shortestHeader(p) + 0.08;
  return p;
}

describe('equal-length pipes', () => {
  it('builds every primary on a bank the same length, into one collector where it was put', () => {
    const ports = portsOf(v8);
    const graph = compileCollectorLayout(v8, [], [makeSegment({ length: 0.5, dIn: 0.06 })]);
    const cylinders = bankCylinders(v8, 0);
    const p = planFrom(graph, portOpenings(graph, ports, cylinders));
    const collectors = graph.ducts.filter((d) => d.role === 'collector').length;
    applyHeader(graph, p, headerPrimaries(p));

    const nodes = new Set(cylinders.map((c) => (runner(graph, c).to as { node: string }).node));
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
    for (const c of cylinders) {
      const duct = runner(graph, c);
      expect(total(duct.segments)).toBeCloseTo(p.length, 2);
      // Bent from the port itself, at the port's bore, and meeting the collector at its bore.
      expect(duct.segments.every((sg) => sg.curve)).toBe(true);
      expect(duct.segments[0]!.dIn).toBeCloseTo(0.042, 9);
      expect(duct.segments.at(-1)!.dOut).toBeCloseTo(collector.segments[0]!.dIn, 9);
      const place = placement.ducts.get(duct.id)!;
      const end = layoutPipe(duct.segments, place.origin, place.heading).joints.at(-1)!;
      expect(end.distanceTo(p.merge)).toBeLessThan(1e-3);
    }
  });

  it('runs every port on both banks to the one place, not mirrored', () => {
    const ports = portsOf(v8);
    const graph = compileCollectorLayout(v8, [], [makeSegment({ length: 0.5, dIn: 0.06 })]);
    const all = Array.from({ length: 8 }, (_, c) => c);
    const openings = portOpenings(graph, ports, all);
    const { merge } = defaultMerge(openings);
    const p: HeaderPlan = {
      openings,
      merge: merge.add(new THREE.Vector3(0, -0.3, 0.2)),
      axis: new THREE.Vector3(0, -0.3, 1).normalize(),
      length: 0,
      collectorBore: headerCollectorBore(graph, openings),
    };
    p.length = shortestHeader(p) + 0.05;
    applyHeader(graph, p, headerPrimaries(p));
    expect(new Set(all.map((c) => (runner(graph, c).to as { node: string }).node)).size).toBe(1);
    for (const c of all) expect(total(runner(graph, c).segments)).toBeCloseTo(p.length, 2);
    expect(validateGraph(graph, 8)).toEqual([]);
  });

  it.each([
    ['a V8', v8],
    ['a boxer four, whose ports both point down', boxer],
  ])('gives the other bank of %s the mirror image', (_n, spec) => {
    const ports = portsOf(spec);
    const graph = compileCollectorLayout(spec, [], [makeSegment({ length: 0.5, dIn: 0.06 })]);
    const p = planFrom(graph, portOpenings(graph, ports, bankCylinders(spec, 0)));
    const mirror = bankMirror(spec)!;
    expect(mirror).not.toBeNull();
    const other = mirrorPlan(p, mirror, portOpenings(graph, ports, bankCylinders(spec, 1)));
    for (const q of [p, other]) applyHeader(graph, q, headerPrimaries(q));
    expect(validateGraph(graph, spec.cylinders)).toEqual([]);

    // Across the engine's middle: the merge and the way it points, mirrored in x.
    expect(other.merge.x).toBeCloseTo(-p.merge.x, 9);
    expect(other.merge.y).toBeCloseTo(p.merge.y, 9);
    expect(other.merge.z).toBeCloseTo(p.merge.z, 9);
    expect(other.axis.x).toBeCloseTo(-p.axis.x, 9);
    for (let c = 0; c < spec.cylinders; c++) expect(total(runner(graph, c).segments)).toBeCloseTo(p.length, 2);
  });

  it('offers a port with a pipe on it only as where that pipe ends, and carries open pipes on', () => {
    const twin = { ...defaultConfig().engine, cylinders: 2, vAngle: 0, exhaustLayout: '2into2' } as EngineSpec;
    const ports = portsOf(twin);
    const graph = compileLayout(twin, [makeSegment({ length: 0.4 })], []);
    const loose = placeLoosePipe(graph, [0.2, 0.1, 0.3], 0.05, 0.2);
    const lengths = (id: string) => graph.ducts.find((d) => d.id === id)!.segments.map((sg) => sg.length);
    const before = { runner0: lengths('runner0'), [loose]: lengths(loose) };
    const all = headerOpenings(graph, layoutGraph(ports, graph), ports, 0.042);
    // Both ports are used: what is open is where their pipes end, and the loose pipe's end.
    expect(all.every((o) => o.opening.kind === 'end')).toBe(true);
    expect(all.map((o) => (o.opening as { duct: string }).duct).sort()).toEqual([loose, 'runner0', 'runner1'].sort());
    const openings = all.filter((o) => (o.opening as { duct: string }).duct !== 'runner1');
    const end = openings.find((o) => (o.opening as { duct: string }).duct === loose)!;
    const p: HeaderPlan = {
      openings,
      merge: end.point.clone().add(new THREE.Vector3(0, -0.2, 0.25)),
      axis: new THREE.Vector3(0, 0, 1),
      length: 0,
      collectorBore: headerCollectorBore(graph, openings),
    };
    p.length = shortestHeader(p) + 0.05;
    applyHeader(graph, p, headerPrimaries(p));

    // Each keeps what it was and carries on to the merge, the same length on from its end.
    const collector = graph.ducts.find((d) => d.role === 'collector')!;
    for (const id of ['runner0', loose]) {
      const pipe = graph.ducts.find((d) => d.id === id)!;
      const was = before[id]!;
      expect(pipe.segments.slice(0, was.length).map((sg) => sg.length)).toEqual(was);
      expect(total(pipe.segments.slice(was.length))).toBeCloseTo(p.length, 2);
      expect(pipe.to).toEqual({ kind: 'node', node: (collector.from as { node: string }).node });
      expect(pipe.segments.at(-1)!.dOut).toBeCloseTo(collector.segments[0]!.dIn, 9);
    }
    // The other cylinder was not picked, and is as it was.
    expect(runner(graph, 1).to).toEqual({ kind: 'mouth' });
    expect(bankMirror(twin)).toBeNull();
    expect(validateGraph(graph, 2)).toEqual([]);
  });
});
