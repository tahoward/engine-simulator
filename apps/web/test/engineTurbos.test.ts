/**
 * Turbos a compiled exhaust seats on the engine: switching a turbocharged engine to another layout gives it a
 * turbo for each bank, laid along the engine halfway along it, out from the bank's ports with each piped
 * straight into it, and clear of the engine, the pipes and the other turbo.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';

import { carriedGeometry, compileExhaust, endsAt, graphFromJson, validateGraph, type ExhaustGraph, type TurboMount } from '../src/model/exhaustGraph.js';
import { engineShell, engineShellDistance, solidCylinder, type Vec3 } from '../src/model/geometry.js';
import { ENGINE_PRESETS, defaultConfig, physicalBank, physicalBankCount, presetEngine, type EngineSpec } from '../src/model/spec.js';
import { graphTurboSize, quatRotate, turboBody, turboPorts, turboPortsOf, type TurboSize } from '../src/model/turbo.js';
import { EngineMesh } from '../src/scene/EngineMesh.js';
import { layoutGraph, pipesMeetAt, type ExhaustPort } from '../src/scene/exhaustLayout.js';
import { seatLengthwaysHeaders } from '../src/scene/headerTool.js';
import { layoutPipe } from '../src/scene/PipeMesh.js';
import { seatEngineTurbos } from '../src/scene/engineTurbos.js';
import { refitBends, seatHeaders, seatManifolds, seatTurbos } from '../src/scene/turboPlacement.js';

const portsOf = (spec: EngineSpec): ExhaustPort[] => {
  const mesh = new EngineMesh(spec);
  return Array.from({ length: mesh.bankCount }, (_, i) => mesh.exhaustPort(i));
};

/** Seated as the view seats a graph when it is rebuilt. */
function rebuild(graph: ExhaustGraph, spec: EngineSpec): ExhaustPort[] {
  const ports = portsOf(spec);
  seatManifolds(graph, ports);
  seatLengthwaysHeaders(graph, ports, spec);
  seatHeaders(graph, ports);
  seatEngineTurbos(graph, ports, spec);
  seatTurbos(graph, ports, spec);
  refitBends(graph, ports, spec);
  return ports;
}

/** Entries of the Cylinders menu, as the panel switches to them. */
const ENGINE_TYPES: Array<[string, Partial<EngineSpec>]> = [
  ['a single', { cylinders: 1, exhaustLayout: 'open', vAngle: 0 }],
  ['a twin', { cylinders: 2, exhaustLayout: 'merged' }],
  ['an inline three', { cylinders: 3, exhaustLayout: 'merged', vAngle: 0 }],
  ['an inline four', { cylinders: 4, exhaustLayout: 'merged', vAngle: 0 }],
  ['an inline five', { cylinders: 5, exhaustLayout: 'merged', vAngle: 0 }],
  ['an inline six', { cylinders: 6, exhaustLayout: 'merged', vAngle: 0 }],
  ['a V6', { cylinders: 6, exhaustLayout: 'perBank', vAngle: 60 }],
  ['a V8', { cylinders: 8, exhaustLayout: 'perBank', vAngle: 90, crankType: 'crossplane' }],
  ['a boxer four', { cylinders: 4, exhaustLayout: 'perBank', vAngle: 180, crankType: 'boxer' }],
  ['a boxer six', { cylinders: 6, exhaustLayout: 'perBank', vAngle: 180, crankType: 'boxer' }],
];

/** The RB26, its exhaust recompiled for `partial` with `turbos` turbos, as switching layouts does. */
function switched(partial: Partial<EngineSpec>, headers: boolean, turbos: number) {
  const preset = ENGINE_PRESETS.find((p) => p.name === 'Inline six, Nissan RB26DETT')!;
  const spec = { ...presetEngine(preset, defaultConfig().engine), ...partial, exhaustHeaders: headers } as EngineSpec;
  const carried = carriedGeometry(graphFromJson(preset.graph!())!);
  const graph = compileExhaust(spec, carried.pipe!, carried.collector!, turbos);
  const ports = rebuild(graph, spec);
  return { spec, graph, ports };
}

/** How far `p` is outside the room `mount` takes up, m: negative inside it. */
function outside(mount: TurboMount, size: TurboSize, p: Vec3): number {
  const q = mount.rotation;
  const local = quatRotate([p[0] - mount.position![0], p[1] - mount.position![1], p[2] - mount.position![2]], [-q[0], -q[1], -q[2], q[3]]);
  return Math.min(
    ...turboBody(size).map((part) => {
      const d = [0, 1, 2].map((i) => local[i]! - part.centre[i]!);
      const radial = Math.hypot(d[(part.axis + 1) % 3]!, d[(part.axis + 2) % 3]!);
      return solidCylinder(radial - part.radius, Math.abs(d[part.axis]!) - part.half);
    }),
  );
}

/** Points filling the room `mount` takes up, on a grid in its own frame. */
function inside(mount: TurboMount, size: TurboSize): Vec3[] {
  const s = size.scroll;
  const out: Vec3[] = [];
  for (let x = -1.5 * s; x <= 3.5 * s; x += 0.2 * s) {
    for (let y = -1.2 * s; y <= 1.5 * s; y += 0.2 * s) {
      for (let z = -1.3 * s; z <= 1.2 * s; z += 0.2 * s) {
        const w = quatRotate([x, y, z], mount.rotation);
        const p: Vec3 = [w[0] + mount.position![0], w[1] + mount.position![1], w[2] + mount.position![2]];
        if (outside(mount, size, p) < 0) out.push(p);
      }
    }
  }
  return out;
}

describe('turbos seated where a switched layout puts them', () => {
  const cases = ENGINE_TYPES.flatMap(([name, partial]) =>
    [false, true].flatMap((headers) => [1, 2].map((turbos) => [`${name}, ${headers ? 'headers' : 'manifolds'}, ${turbos} turbo(s)`, partial, headers, turbos] as const)),
  );

  it.each(cases)('one for each bank, laid along the engine: %s', (_, partial, headers, count) => {
    const { spec, graph, ports } = switched(partial, headers, count);
    expect(validateGraph(graph, spec.cylinders)).toEqual([]);
    const banks = physicalBankCount(spec);
    const turbos = turboPortsOf(graph, spec);
    expect(turbos.size).toBe(banks);
    for (const [node, { inlet, outlet }] of turbos) {
      // The outlet rearwards, and the shaft with it, along the crank.
      expect(outlet.dir[2], node).toBeCloseTo(1, 6);
      // Halfway along the engine, every one of its bank's ports piped straight into it.
      const feeds = graph.ducts.filter((d) => d.to.kind === 'node' && d.to.node === node);
      const cylinders = feeds.map((d) => (d.from.kind === 'valve' ? d.from.cylinder : -1));
      const bankOf = (c: number) => (banks > 1 ? physicalBank(spec, c) : 0);
      expect(cylinders.sort()).toEqual(Array.from({ length: spec.cylinders }, (_, c) => c).filter((c) => bankOf(c) === bankOf(cylinders[0]!)));
      // A flange's length straight out, then into the inlet: straight where it is in line, a bend otherwise.
      for (const d of feeds) {
        expect(d.segments, d.id).toHaveLength(2);
        expect(d.segments[0]!.curve, d.id).toBeUndefined();
      }
      expect(pipesMeetAt(graph, layoutGraph(ports, graph, turbos), node), node).toBe(true);
      const mine = cylinders.map((c) => ports[c]!);
      const out = mine.reduce((v, p) => v.add(p.direction), new THREE.Vector3()).normalize();
      expect(new THREE.Vector3(...inlet.dir).dot(out), node).toBeCloseTo(1, 6);
      expect(inlet.point[2], node).toBeCloseTo(0, 6);
      const beyond = Math.min(...mine.map((p) => new THREE.Vector3(...inlet.point).sub(p.position).dot(out)));
      expect(beyond, node).toBeGreaterThan(0);
    }
  });

  it.each(cases)('are clear of the engine, the pipes and each other: %s', (_, partial, headers, count) => {
    const { spec, graph, ports } = switched(partial, headers, count);
    const size = graphTurboSize(graph, spec);
    const shell = engineShell(spec);
    const placement = layoutGraph(ports, graph, turboPortsOf(graph, spec));
    const turbos = (graph.turbos ?? []).filter((t) => t.position);
    expect(turbos.length).toBe(graph.turbos?.length ?? 0);
    for (const t of turbos) {
      const room = inside(t, size);
      const engine = Math.min(...room.map((p) => engineShellDistance(shell, p)));
      expect(engine, `${t.id} into the engine`).toBeGreaterThan(0);
      for (const other of turbos.filter((o) => o !== t)) {
        expect(Math.min(...room.map((p) => outside(other, size, p))), `${t.id} into ${other.id}`).toBeGreaterThan(0);
      }
      // Its own pipes meet it at its flanges, as they are meant to.
      const { inlet, outlet } = turboPorts(t as TurboMount & { position: Vec3 }, size);
      for (const duct of graph.ducts) {
        const place = placement.ducts.get(duct.id)!;
        const own = endsAt(graph, t.node).some((e) => e.duct === duct);
        for (const st of layoutPipe(duct.segments, place.origin, place.heading).stations) {
          const p: Vec3 = [st.position.x, st.position.y, st.position.z];
          const near = (flange: Vec3, zone: number) => st.position.distanceTo(new THREE.Vector3(...flange)) < zone * size.scroll + st.radius;
          if (own && (near(inlet.point, 0.7) || near(outlet.point, 0.5))) continue;
          expect(outside(t, size, p) - st.radius, `${duct.id} through ${t.id}`).toBeGreaterThan(0);
        }
      }
    }
  });

  it('leaves a turbo that has been put somewhere where it is', () => {
    const preset = ENGINE_PRESETS.find((p) => p.name === 'Inline six, Nissan RB26DETT')!;
    const spec = presetEngine(preset, defaultConfig().engine);
    const graph = graphFromJson(preset.graph!())!;
    const before = JSON.stringify(graph.turbos);
    seatEngineTurbos(graph, portsOf(spec), spec);
    expect(JSON.stringify(graph.turbos)).toBe(before);
  });
});
