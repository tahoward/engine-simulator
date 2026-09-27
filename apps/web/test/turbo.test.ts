/**
 * Turbos placed in the exhaust: compiled into a preset's layout, put down on a pipe, drawn into,
 * moved and taken out, and laid out with their pipes on their flanges.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';

import {
  compileExhaust,
  endsAt,
  graphFromJson,
  pathToAir,
  tidyJunctions,
  validateGraph,
  type ExhaustGraph,
} from '../src/model/exhaustGraph.js';
import { ENGINE_PRESETS, defaultConfig, presetEngine, type EngineSpec } from '../src/model/spec.js';
import {
  graphTurboSize,
  newTurbo,
  placeTurbo,
  quatFromAxisAngle,
  quatMultiply,
  removeTurbo,
  turboPortsOf,
} from '../src/model/turbo.js';
import { collectSnapTargets } from '../src/scene/drawing.js';
import { EngineMesh } from '../src/scene/EngineMesh.js';
import { layoutGraph, pipesMeetAt, type ExhaustPort } from '../src/scene/exhaustLayout.js';
import { moveTurbo, seatTurbos } from '../src/scene/turboPlacement.js';

const portsOf = (spec: EngineSpec): ExhaustPort[] => {
  const mesh = new EngineMesh(spec, new THREE.Plane(new THREE.Vector3(0, 0, -1), 0.001));
  return Array.from({ length: mesh.bankCount }, (_, i) => mesh.exhaustPort(i));
};

/** The default single, with its one pipe open to the air. */
function single(): { spec: EngineSpec; graph: ExhaustGraph; ports: ExhaustPort[] } {
  const cfg = defaultConfig();
  const graph = compileExhaust(cfg.engine, cfg.pipe, cfg.collector);
  return { spec: cfg.engine, graph, ports: portsOf(cfg.engine) };
}

/** The single with a turbo put down on the open end of its pipe, seated there as the view seats one. */
function singleWithTurbo() {
  const { spec, graph, ports } = single();
  const mount = newTurbo(graph);
  const size = graphTurboSize({ ducts: [], turbos: [mount] }, spec);
  placeTurbo(graph, mount, size.outletDia, 'runner0');
  seatTurbos(graph, ports, spec);
  return { spec, graph, ports, mount };
}

describe('a turbo put down on the open end of a pipe', () => {
  it('attaches the pipe, and gives the turbo an outlet to the air', () => {
    const { graph, mount } = singleWithTurbo();
    expect(validateGraph(graph, 1)).toEqual([]);
    const ends = endsAt(graph, mount.node);
    expect(ends.filter((e) => e.end === 'outlet').map((e) => e.duct.id)).toEqual(['runner0']);
    const outs = ends.filter((e) => e.end === 'inlet').map((e) => e.duct);
    expect(outs).toHaveLength(1);
    expect(outs[0]!.to.kind).toBe('mouth');
  });

  it('meets the pipe at its inlet flange, and starts its outlet pipe from its outlet flange', () => {
    const { spec, graph, ports, mount } = singleWithTurbo();
    const turbos = turboPortsOf(graph, spec);
    const placement = layoutGraph(ports, graph, turbos);
    const flanges = turbos.get(mount.node)!;
    expect(pipesMeetAt(graph, placement, mount.node)).toBe(true);
    const outlet = endsAt(graph, mount.node).find((e) => e.end === 'inlet')!.duct;
    const start = placement.ducts.get(outlet.id)!.origin;
    expect(start.distanceTo(new THREE.Vector3(...flanges.outlet.point))).toBeLessThan(1e-9);
    // A turbo is not a junction, so there is no fitting drawn for it.
    expect(placement.joints.has(mount.node)).toBe(false);
  });

  it('is offered as somewhere to draw a pipe to, at its inlet flange', () => {
    const { spec, graph, ports, mount } = singleWithTurbo();
    const turbos = turboPortsOf(graph, spec);
    const targets = collectSnapTargets(graph, layoutGraph(ports, graph, turbos), ports);
    const inlet = targets.find((t) => t.kind === 'turboInlet');
    expect(inlet).toBeDefined();
    expect(inlet!.point.distanceTo(new THREE.Vector3(...turbos.get(mount.node)!.inlet.point))).toBeLessThan(1e-9);
    // Its node is not offered as a junction as well.
    expect(targets.some((t) => t.kind === 'node' && t.node === mount.node)).toBe(false);
  });

  it('keeps its one pipe in and one out, which a junction would be rejoined from', () => {
    const { graph, mount } = singleWithTurbo();
    const before = graph.ducts.length;
    tidyJunctions(graph, [mount.node]);
    expect(graph.ducts).toHaveLength(before);
    expect(validateGraph(graph, 1)).toEqual([]);
  });

  it('comes out leaving its pipe open to the air again', () => {
    const { graph, mount } = singleWithTurbo();
    removeTurbo(graph, mount.id);
    expect(graph.turbos).toBeUndefined();
    expect(graph.ducts.map((d) => d.id)).toEqual(['runner0']);
    expect(graph.ducts[0]!.to.kind).toBe('mouth');
    expect(validateGraph(graph, 1)).toEqual([]);
  });

  it('brings its pipe with it when it is moved', () => {
    const { spec, graph, ports, mount } = singleWithTurbo();
    const [x, y, z] = mount.position!;
    const turned = quatMultiply(quatFromAxisAngle([1, 0, 0], 0.4), mount.rotation);
    moveTurbo(graph, ports, spec, mount.id, [x + 0.06, y - 0.04, z + 0.05], turned);
    const turbos = turboPortsOf(graph, spec);
    expect(graph.ducts.find((d) => d.id === 'runner0')!.to).toEqual({ kind: 'node', node: mount.node });
    expect(pipesMeetAt(graph, layoutGraph(ports, graph, turbos), mount.node)).toBe(true);
  });

  it('turned a quarter turn about an axis, turns its outlet by a quarter turn', () => {
    const { spec, graph, ports, mount } = singleWithTurbo();
    const before = turboPortsOf(graph, spec).get(mount.node)!.outlet.dir;
    const turned = quatMultiply(quatFromAxisAngle([1, 0, 0], Math.PI / 2), mount.rotation);
    moveTurbo(graph, ports, spec, mount.id, mount.position!, turned);
    const after = turboPortsOf(graph, spec).get(mount.node)!.outlet.dir;
    const angle = new THREE.Vector3(...before).angleTo(new THREE.Vector3(...after));
    const alongAxis = Math.abs(before[0]);
    // Only the part of the outlet's direction square to the axis turns.
    expect(Math.cos(angle)).toBeCloseTo(alongAxis * alongAxis, 9);
    // And its pipe still meets it.
    expect(pipesMeetAt(graph, layoutGraph(ports, graph, turboPortsOf(graph, spec)), mount.node)).toBe(true);
  });

  it('survives being saved in a link and read back', () => {
    const { graph } = singleWithTurbo();
    const back = graphFromJson(JSON.parse(JSON.stringify(graph)));
    expect(back).toEqual(graph);
  });

  it('makes the engine boost, heard through the Wasm build', async () => {
    const { Sim } = await import('../src/audio/worklet/sim.js');
    const { spec, graph } = singleWithTurbo();
    const cfg = defaultConfig();
    cfg.engine = { ...spec, throttle: 1, rpm: 6000, freeRunning: false };
    cfg.graph = graph;
    const sim = new Sim(48000, cfg);
    sim.render(48000 * 3);
    const out = sim.render(48000 / 4);
    expect(out.every((v) => Number.isFinite(v))).toBe(true);
    const turbo = sim.snapshot().turbo;
    expect(turbo).toBeDefined();
    expect(turbo!.boost).toBeGreaterThan(0.2e5);
  });
});

describe('the RB26 preset', () => {
  const preset = ENGINE_PRESETS.find((p) => p.name.includes('RB26'))!;
  const spec = presetEngine(preset, defaultConfig().engine);
  const graph = compileExhaust(spec, preset.pipe(), preset.collector!(), preset.turbos);

  it('has two turbos, each fed by three cylinders', () => {
    expect(validateGraph(graph, 6)).toEqual([]);
    expect(graph.turbos).toHaveLength(2);
    for (const turbo of graph.turbos!) {
      const fed = [0, 1, 2, 3, 4, 5].filter((c) =>
        pathToAir(graph, c).some((d) => d.to.kind === 'node' && d.to.node === turbo.node),
      );
      expect(fed).toHaveLength(3);
    }
  });

  it('seats each turbo where its three cylinders’ manifold ends', () => {
    const g = JSON.parse(JSON.stringify(graph)) as ExhaustGraph;
    const ports = portsOf(spec);
    seatTurbos(g, ports, spec);
    expect(g.turbos!.every((t) => t.position !== null)).toBe(true);
    const placement = layoutGraph(ports, g, turboPortsOf(g, spec));
    for (const t of g.turbos!) expect(pipesMeetAt(g, placement, t.node)).toBe(true);
  });
});
