/**
 * Turbos placed in the exhaust: compiled into a preset's layout, put down on a pipe, drawn into,
 * moved and taken out, and laid out with their pipes on their flanges.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';

import {
  compileExhaust,
  disconnectEnd,
  endsAt,
  joinDuctEnd,
  graphFromJson,
  pathToAir,
  tidyJunctions,
  validateGraph,
  type ExhaustGraph,
} from '../src/model/exhaustGraph.js';
import { ENGINE_PRESETS, defaultConfig, makeSegment, presetEngine, segmentDiameter, type EngineSpec } from '../src/model/spec.js';
import {
  graphTurboSize,
  newTurbo,
  placeTurbo,
  quatFromAxisAngle,
  quatMultiply,
  fittedBend,
  removeTurbo,
  turboPortsOf,
} from '../src/model/turbo.js';
import { bendAnchor, collectSnapTargets, fitCurve } from '../src/scene/drawing.js';
import { layoutPipe } from '../src/scene/PipeMesh.js';
import { EngineMesh } from '../src/scene/EngineMesh.js';
import { freezeHeadings, layoutGraph, pipesMeetAt, type ExhaustPort } from '../src/scene/exhaustLayout.js';
import { moveJunction, moveTurbo, refitBends, seatTurbos } from '../src/scene/turboPlacement.js';

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

  it('takes its bend with it when it comes out', () => {
    const { spec, graph, ports, mount } = singleWithTurbo();
    const drawn = graph.ducts.find((d) => d.id === 'runner0')!.segments.length;
    const [px, py, pz] = mount.position!;
    moveTurbo(graph, ports, spec, mount.id, [px + 0.05, py, pz + 0.08], mount.rotation);
    removeTurbo(graph, mount.id);
    const runner = graph.ducts.find((d) => d.id === 'runner0')!;
    expect(runner.segments).toHaveLength(drawn);
    expect(runner.fitted).toBeUndefined();
    expect(runner.segments.every((s) => !s.curve)).toBe(true);
  });

  it('stays in while pipes carry on from its outlet pipe', () => {
    const { graph, mount } = singleWithTurbo();
    const outlet = endsAt(graph, mount.node).find((e) => e.end === 'inlet')!.duct;
    joinDuctEnd(graph, outlet.id, 0);
    const before = JSON.stringify(graph);
    expect(removeTurbo(graph, mount.id)).toBe(false);
    expect(JSON.stringify(graph)).toBe(before);
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

  it('swings its outlet pipe with it when it is turned, once the exhaust has been edited', () => {
    const { spec, graph, ports, mount } = singleWithTurbo();
    const outletOf = () => {
      const turbos = turboPortsOf(graph, spec);
      const out = endsAt(graph, mount.node).find((e) => e.end === 'inlet')!.duct;
      const placement = layoutGraph(ports, graph, turbos);
      return { heading: placement.ducts.get(out.id)!.heading, flange: new THREE.Vector3(...turbos.get(mount.node)!.outlet.dir) };
    };
    // An edit freezes every pipe where it stands, the outlet pipe with it.
    freezeHeadings(graph, layoutGraph(ports, graph, turboPortsOf(graph, spec)), ports);
    const before = outletOf();
    expect(before.heading.angleTo(before.flange)).toBeLessThan(1e-9);
    const turned = quatMultiply(quatFromAxisAngle([0, 1, 0], 1.1), mount.rotation);
    moveTurbo(graph, ports, spec, mount.id, mount.position!, turned);
    const after = outletOf();
    // Still straight out of the flange, which now points somewhere else.
    expect(after.heading.angleTo(after.flange)).toBeLessThan(1e-9);
    expect(after.flange.angleTo(before.flange)).toBeGreaterThan(1);
  });

  it('meets its pipes at its own bores: the inlet, and the outlet', () => {
    const { spec, graph, ports, mount } = singleWithTurbo();
    const [px, py, pz] = mount.position!;
    moveTurbo(graph, ports, spec, mount.id, [px + 0.05, py, pz + 0.08], mount.rotation);
    const size = graphTurboSize(graph, spec);
    const runner = graph.ducts.find((d) => d.id === 'runner0')!;
    expect(runner.segments.at(-1)!.dOut).toBeCloseTo(size.inletDia, 12);
    const out = endsAt(graph, mount.node).find((e) => e.end === 'inlet')!.duct;
    expect(out.segments[0]!.dIn).toBeCloseTo(size.outletDia, 12);
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

describe('a pipe fitted into a turbo’s inlet', () => {
  const entry = new THREE.Vector3(0.1, 0.2, 0);
  const x = new THREE.Vector3(1, 0, 0);
  const swept = (seg: ReturnType<typeof fitCurve>) => layoutPipe([seg], entry, x);

  it('runs straight in when the port is on the pipe’s own line', () => {
    const seg = fitCurve(entry, x, entry.clone().addScaledVector(x, 0.3), x);
    expect(seg.curve).toBeUndefined();
    expect(seg.length).toBeCloseTo(0.3, 9);
  });

  it('is one smooth bend, an S to a port off to one side but facing the same way', () => {
    const target = entry.clone().add(new THREE.Vector3(0.3, 0, 0.12));
    const seg = fitCurve(entry, x, target, x);
    expect(seg.curve).toBeDefined();
    const layout = swept(seg);
    expect(layout.joints).toHaveLength(1);
    expect(layout.joints[0]!.distanceTo(target)).toBeLessThan(1e-9);
    // Leaving the way the pipe was going, and meeting the flange square.
    expect(layout.stations[0]!.direction.angleTo(x)).toBeLessThan(1e-9);
    expect(layout.jointDirections[0]!.angleTo(x)).toBeLessThan(1e-9);
    // Smooth: no two stations along it turn more than a couple of degrees.
    for (let i = 1; i < layout.stations.length; i++) {
      const turn = layout.stations[i]!.direction.angleTo(layout.stations[i - 1]!.direction);
      expect((turn * 180) / Math.PI).toBeLessThan(3);
    }
    // As long as the bend, which is longer than the straight line across it.
    expect(seg.length).toBeCloseTo(layout.totalLength, 9);
    expect(seg.length).toBeGreaterThan(target.distanceTo(entry));
  });

  it('curves round to a port facing another way', () => {
    const target = entry.clone().add(new THREE.Vector3(0.2, 0.15, 0.1));
    const into = new THREE.Vector3(0, 1, 0);
    const layout = swept(fitCurve(entry, x, target, into));
    expect(layout.joints[0]!.distanceTo(target)).toBeLessThan(1e-9);
    expect(layout.jointDirections[0]!.angleTo(into)).toBeLessThan(1e-9);
  });

  it('survives a link, bend and all', () => {
    const seg = fitCurve(entry, x, entry.clone().add(new THREE.Vector3(0.3, 0.05, 0.12)), x);
    const back = graphFromJson(JSON.parse(JSON.stringify({ ducts: [{ id: 'a', segments: [seg], from: { kind: 'valve', cylinder: 0 }, to: { kind: 'mouth' } }] })))!;
    expect(back.ducts[0]!.segments[0]).toEqual(seg);
  });

  it('fits the bend again when the turbo moves, leaving the pipe as drawn, and is not edited', () => {
    const { spec, graph, ports, mount } = singleWithTurbo();
    const drawn = graph.ducts.find((d) => d.id === 'runner0')!.segments.map((s) => ({ ...s }));
    const [px, py, pz] = mount.position!;
    moveTurbo(graph, ports, spec, mount.id, [px + 0.05, py, pz + 0.08], mount.rotation);
    const runner = graph.ducts.find((d) => d.id === 'runner0')!;
    expect(runner.fitted).toBe(true);
    expect(runner.segments).toHaveLength(drawn.length + 1);
    expect(runner.segments.slice(0, drawn.length)).toEqual(drawn);
    expect(fittedBend(graph, runner)).toBe(drawn.length);
    expect(pipesMeetAt(graph, layoutGraph(ports, graph, turboPortsOf(graph, spec)), mount.node)).toBe(true);
    // Back where it was, the pipe runs straight in again as it did.
    moveTurbo(graph, ports, spec, mount.id, [px, py, pz], mount.rotation);
    expect(runner.segments).toEqual(drawn);
    expect(runner.fitted).toBeUndefined();
  });

  it('follows the pipe drawn up to it when that changes', () => {
    const { spec, graph, ports, mount } = singleWithTurbo();
    const [px, py, pz] = mount.position!;
    moveTurbo(graph, ports, spec, mount.id, [px + 0.05, py, pz + 0.08], mount.rotation);
    const runner = graph.ducts.find((d) => d.id === 'runner0')!;
    runner.segments[0]!.length += 0.04;
    refitBends(graph, ports, spec);
    expect(pipesMeetAt(graph, layoutGraph(ports, graph, turboPortsOf(graph, spec)), mount.node)).toBe(true);
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

describe('a pipe drawn to join another', () => {
  // The 90° V-twin: two pipes, each open to the air.
  const preset = ENGINE_PRESETS.find((p) => p.name.startsWith('90'))!;
  const spec = presetEngine(preset, defaultConfig().engine);
  const ports = portsOf(spec);

  /** Pipe 2 joined onto the open end of pipe 1, as drawing from one to the other does. */
  function joined() {
    const graph = compileExhaust(spec, preset.pipe(), preset.collector?.() ?? []);
    const placement = layoutGraph(ports, graph);
    const endOf = (id: string) => {
      const d = graph.ducts.find((x) => x.id === id)!;
      const p = placement.ducts.get(id)!;
      const swept = layoutPipe(d.segments, p.origin, p.heading);
      return { point: swept.joints.at(-1)!, dir: swept.jointDirections.at(-1)! };
    };
    const target = endOf('runner0');
    const tip = endOf('runner1');
    const node = joinDuctEnd(graph, 'runner0', 0)!;
    const runner1 = graph.ducts.find((d) => d.id === 'runner1')!;
    runner1.segments.push(fitCurve(tip.point, tip.dir, target.point, target.dir, { dIn: 0.04, dOut: 0.04 }));
    runner1.fitted = true;
    runner1.to = { kind: 'node', node };
    return { graph, node, runner1 };
  }

  const meets = (graph: ExhaustGraph, node: string) => pipesMeetAt(graph, layoutGraph(ports, graph), node);

  it('bends in to arrive beside the pipe it joins, in one smooth, locked bend', () => {
    const { graph, node, runner1 } = joined();
    expect(validateGraph(graph, 2)).toEqual([]);
    expect(meets(graph, node)).toBe(true);
    expect(runner1.segments.at(-1)!.curve).toBeDefined();
    expect(fittedBend(graph, runner1)).toBe(runner1.segments.length - 1);
    // It arrives along the pipe it joins, so the two merge side by side.
    const placement = layoutGraph(ports, graph);
    const anchor = bendAnchor(graph, placement, node, 'runner1')!;
    const p = placement.ducts.get('runner1')!;
    const arrive = layoutPipe(runner1.segments, p.origin, p.heading).jointDirections.at(-1)!;
    expect(arrive.angleTo(anchor.dir)).toBeLessThan(1e-9);
  });

  it('follows when the pipe it joins, or the pipe drawn up to it, changes', () => {
    const { graph, node, runner1 } = joined();
    graph.ducts.find((d) => d.id === 'runner0')!.segments[0]!.length += 0.05;
    refitBends(graph, ports, spec);
    expect(meets(graph, node)).toBe(true);
    runner1.segments[0]!.length += 0.03;
    refitBends(graph, ports, spec);
    expect(meets(graph, node)).toBe(true);
  });

  it('follows when the pipe it joins is turned', () => {
    const { graph, node, runner1 } = joined();
    const runner0 = graph.ducts.find((d) => d.id === 'runner0')!;
    for (const turn of [0.4, -0.7]) {
      runner0.segments.at(-1)!.yaw += turn;
      runner0.segments.at(-1)!.pitch += turn / 2;
      refitBends(graph, ports, spec);
      expect(meets(graph, node)).toBe(true);
      // Still arriving alongside it, the way it now points.
      const placement = layoutGraph(ports, graph);
      const anchor = bendAnchor(graph, placement, node, 'runner1')!;
      const p = placement.ducts.get('runner1')!;
      const arrive = layoutPipe(runner1.segments, p.origin, p.heading).jointDirections.at(-1)!;
      expect(arrive.angleTo(anchor.dir)).toBeLessThan(1e-9);
    }
  });

  it('follows when the pipe the gas carries on through is pivoted where it leaves', () => {
    const { graph, node, runner1 } = joined();
    const onward = graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === node)!;
    for (const turn of [0.5, -0.3]) {
      onward.segments[0]!.yaw += turn;
      refitBends(graph, ports, spec);
      const placement = layoutGraph(ports, graph);
      expect(pipesMeetAt(graph, placement, node)).toBe(true);
      const o = placement.ducts.get(onward.id)!;
      const leaving = layoutPipe(onward.segments, o.origin, o.heading).stations[0]!.direction;
      const p = placement.ducts.get('runner1')!;
      const arrive = layoutPipe(runner1.segments, p.origin, p.heading).jointDirections.at(-1)!;
      // Merging into the pipe the gas goes on through, the way it now leaves.
      expect(arrive.angleTo(leaving)).toBeLessThan(1e-9);
    }
  });

  it('moves with the junction it starts from, the pipes into that bending in to follow', () => {
    const { graph, node, runner1 } = joined();
    const runner0 = graph.ducts.find((d) => d.id === 'runner0')!;
    const onward = graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === node)!;
    const before = layoutGraph(ports, graph).joints.get(node)!;
    const to: [number, number, number] = [before.centre.x + 0.05, before.centre.y + 0.03, before.centre.z - 0.04];
    moveJunction(graph, ports, spec, node, to, [before.axis.x, before.axis.y, before.axis.z]);
    const placement = layoutGraph(ports, graph);
    const target = new THREE.Vector3(...to);
    // The junction is where it was put, the pipe leaving it starts there, and both pipes in meet it.
    expect(placement.joints.get(node)!.centre.distanceTo(target)).toBeLessThan(1e-9);
    expect(placement.ducts.get(onward.id)!.origin.distanceTo(target)).toBeLessThan(1e-9);
    expect(pipesMeetAt(graph, placement, node)).toBe(true);
    // Each in a fitted bend, arriving along the pipe leaving.
    const leaving = layoutPipe(onward.segments, target, placement.ducts.get(onward.id)!.heading).stations[0]!.direction;
    for (const d of [runner0, runner1]) {
      expect(d.fitted).toBe(true);
      const p = placement.ducts.get(d.id)!;
      expect(layoutPipe(d.segments, p.origin, p.heading).jointDirections.at(-1)!.angleTo(leaving)).toBeLessThan(1e-9);
    }
    expect(validateGraph(graph, 2)).toEqual([]);
    // And it survives a link.
    expect(graphFromJson(JSON.parse(JSON.stringify(graph)))!.junctions).toEqual(graph.junctions);
  });

  it('matches the bore of the pipe before it and of the pipe it joins, tapering between them', () => {
    const { graph, runner1 } = joined();
    const runner0 = graph.ducts.find((d) => d.id === 'runner0')!;
    // Different bores either side, so the bend has something to match.
    runner0.segments.at(-1)!.dOut = 0.05;
    runner1.segments.at(-2)!.dOut = 0.036;
    refitBends(graph, ports, spec);
    const bend = runner1.segments.at(-1)!;
    expect(bend.dIn).toBeCloseTo(0.036, 12);
    expect(bend.dOut).toBeCloseTo(0.05, 12);
    expect(segmentDiameter(bend, 0.5)).toBeCloseTo(0.043, 12);
  });

  it('gives up its bend when it is taken off again', () => {
    const { graph, runner1 } = joined();
    const drawn = runner1.segments.length - 1;
    disconnectEnd(graph, 'runner1');
    expect(runner1.segments).toHaveLength(drawn);
    expect(runner1.fitted).toBeUndefined();
  });
});

describe('a pipe', () => {
  it('tapers in a straight line between its two ends', () => {
    const seg = makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.04, dOut: 0.06 });
    expect(seg.dOut).toBe(0.06);
    expect(segmentDiameter(seg, 0)).toBe(0.04);
    expect(segmentDiameter(seg, 0.25)).toBeCloseTo(0.045, 12);
    expect(segmentDiameter(seg, 1)).toBe(0.06);
    // And one given only its inlet is the same all the way along.
    expect(makeSegment({ kind: 'pipe', dIn: 0.05 }).dOut).toBe(0.05);
  });
});

describe('a loose pipe', () => {
  it('is put down attached to nothing, where it was put, and the solver is not given it', async () => {
    const { placeLoosePipe, solverGraph } = await import('../src/model/exhaustGraph.js');
    const { spec, graph, ports } = single();
    const id = placeLoosePipe(graph, [0.4, 0.2, 0.3], 0.04);
    expect(validateGraph(graph, 1)).toEqual([]);
    const placement = layoutGraph(ports, graph);
    expect(placement.ducts.get(id)!.origin.distanceTo(new THREE.Vector3(0.4, 0.2, 0.3))).toBeLessThan(1e-12);
    expect(solverGraph(graph).ducts.map((d) => d.id)).toEqual(['runner0']);
    // And survives a link.
    expect(graphFromJson(JSON.parse(JSON.stringify(graph)))).toEqual(graph);
    void spec;
  });

  it('is attached by a pipe drawn into its start, and is then fed like any other', async () => {
    const { attachToLooseStart, placeLoosePipe, solverGraph } = await import('../src/model/exhaustGraph.js');
    const { spec, graph, ports } = single();
    const id = placeLoosePipe(graph, [0.5, 0.25, 0.1], 0.04);
    const loose = graph.ducts.find((d) => d.id === id)!;
    const before = layoutGraph(ports, graph).ducts.get(id)!;
    const node = attachToLooseStart(graph, 'runner0', id, [1, 0, 0])!;
    refitBends(graph, ports, spec);
    expect(validateGraph(graph, 1)).toEqual([]);
    // Fed now, so the solver hears it.
    expect(solverGraph(graph).ducts.map((d) => d.id)).toContain(id);
    const placement = layoutGraph(ports, graph);
    // It stayed where it was, and the pipe drawn into it bends in to meet its start.
    const after = placement.ducts.get(id)!;
    expect(after.origin.distanceTo(before.origin)).toBeLessThan(1e-9);
    expect(after.heading.angleTo(before.heading)).toBeLessThan(1e-9);
    expect(pipesMeetAt(graph, placement, node)).toBe(true);
    expect(graph.ducts.find((d) => d.id === 'runner0')!.fitted).toBe(true);
    void loose;
  });
});
