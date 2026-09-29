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
  junctionAt,
  graphFromJson,
  pathToAir,
  solverGraph,
  tidyJunctions,
  validateGraph,
  type ExhaustGraph,
} from '../src/model/exhaustGraph.js';
import {
  ENGINE_PRESETS,
  collectorGroups,
  defaultConfig,
  exhaustLayoutOf,
  makeSegment,
  presetEngine,
  segmentDiameter,
  type EngineSpec,
} from '../src/model/spec.js';
import {
  engineTurboSettings,
  graphTurboSize,
  newTurbo,
  placeTurbo,
  quatFromAxisAngle,
  quatMultiply,
  quatRotate,
  fittedBend,
  lockedFrom,
  removeTurbo,
  setTurbosSynced,
  turboPortsOf,
  turbosSynced,
  UPRIGHT,
} from '../src/model/turbo.js';
import { bendAnchor, collectSnapTargets, fitCurve } from '../src/scene/drawing.js';
import { bendRadius, curveInWorld, layoutPipe } from '../src/scene/PipeMesh.js';
import { EngineMesh } from '../src/scene/EngineMesh.js';
import { freezeHeadings, layoutGraph, pipesMeetAt, type ExhaustPort } from '../src/scene/exhaustLayout.js';
import { seatLengthwaysHeaders } from '../src/scene/headerTool.js';
import { matchLength, moveJunction, moveTurbo, refitBends, seatHeaders, seatManifolds, seatTurbos } from '../src/scene/turboPlacement.js';

const portsOf = (spec: EngineSpec): ExhaustPort[] => {
  const mesh = new EngineMesh(spec);
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
  placeTurbo(graph, mount, 'runner0');
  seatTurbos(graph, ports, spec);
  return { spec, graph, ports, mount };
}

/** As `singleWithTurbo`, with a pipe drawn from the turbo's outlet to the air. */
function singleWithTurboAndOutlet() {
  const t = singleWithTurbo();
  t.graph.ducts.push({
    id: 'downpipe',
    segments: [makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.05 })],
    from: { kind: 'node', node: t.mount.node },
    to: { kind: 'mouth' },
  });
  refitBends(t.graph, t.ports, t.spec);
  return t;
}

describe('a turbo put down on the open end of a pipe', () => {
  it('attaches the pipe, and adds nothing at its outlet', () => {
    const { graph, mount } = singleWithTurbo();
    expect(validateGraph(graph, 1)).toEqual([]);
    const ends = endsAt(graph, mount.node);
    expect(ends.map((e) => [e.duct.id, e.end])).toEqual([['runner0', 'outlet']]);
  });

  it('exhausts to the air at its outlet flange until a pipe is drawn from it', () => {
    const { graph, mount } = singleWithTurbo();
    const solved = solverGraph(graph);
    expect(validateGraph(solved, 1)).toEqual([]);
    const exit = solved.ducts.filter((d) => d.from.kind === 'node' && d.from.node === mount.node);
    expect(exit).toHaveLength(1);
    expect(exit[0]!.to.kind).toBe('mouth');
    // The runner still feeds the turbine, rather than venting past it.
    expect(solved.ducts.find((d) => d.id === 'runner0')!.to).toEqual({ kind: 'node', node: mount.node });
    // Not added to the graph itself.
    expect(graph.ducts).toHaveLength(1);
  });

  it('meets the pipe at its inlet flange, and starts its outlet pipe from its outlet flange', () => {
    const { spec, graph, ports, mount } = singleWithTurboAndOutlet();
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

  it('is offered as somewhere to draw a pipe from, at its outlet flange, until one is drawn', () => {
    const { spec, graph, ports, mount } = singleWithTurbo();
    const turbos = turboPortsOf(graph, spec);
    const outlet = collectSnapTargets(graph, layoutGraph(ports, graph, turbos), ports).find((t) => t.kind === 'turboOutlet');
    expect(outlet).toBeDefined();
    expect(outlet!.point.distanceTo(new THREE.Vector3(...turbos.get(mount.node)!.outlet.point))).toBeLessThan(1e-9);
    const drawn = singleWithTurboAndOutlet();
    const after = collectSnapTargets(drawn.graph, layoutGraph(drawn.ports, drawn.graph, turboPortsOf(drawn.graph, drawn.spec)), drawn.ports);
    expect(after.some((t) => t.kind === 'turboOutlet')).toBe(false);
  });

  it.each([false, true])('keeps its pipes, which a junction would be rejoined from, with an outlet pipe: %s', (outlet) => {
    const { graph, mount } = outlet ? singleWithTurboAndOutlet() : singleWithTurbo();
    const before = JSON.stringify(graph.ducts);
    tidyJunctions(graph, [mount.node]);
    expect(JSON.stringify(graph.ducts)).toBe(before);
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
    const { graph, mount } = singleWithTurboAndOutlet();
    const outlet = endsAt(graph, mount.node).find((e) => e.end === 'inlet')!.duct;
    // A junction at its end, and a pipe drawn on from it.
    const node = joinDuctEnd(graph, outlet.id)!;
    graph.ducts.push({ id: 'onward', segments: [makeSegment({ length: 0.3 })], from: { kind: 'node', node }, to: { kind: 'mouth' } });
    const before = JSON.stringify(graph);
    expect(removeTurbo(graph, mount.id)).toBe(false);
    expect(JSON.stringify(graph)).toBe(before);
  });

  it('keeps its outlet pipe, and what carries on from it, when the pipe into it comes off', () => {
    const { spec, graph, ports, mount } = singleWithTurboAndOutlet();
    const outlet = endsAt(graph, mount.node).find((e) => e.end === 'inlet')!.duct;
    const node = joinDuctEnd(graph, outlet.id)!;
    graph.ducts.push({ id: 'onward', segments: [makeSegment({ length: 0.3 })], from: { kind: 'node', node }, to: { kind: 'mouth' } });
    expect(disconnectEnd(graph, 'runner0')).toBe(true);
    expect(graph.ducts.map((d) => d.id).sort()).toEqual(['downpipe', 'onward', 'runner0']);
    expect(outlet.from).toEqual({ kind: 'node', node: mount.node });
    expect(validateGraph(graph, 1)).toEqual([]);
    // Unfed, it carries no gas: the solver is given the runner alone.
    expect(solverGraph(graph).ducts.map((d) => d.id)).toEqual(['runner0']);
    // Still laid out on its flange, and offered to draw into again.
    const turbos = turboPortsOf(graph, spec);
    const placement = layoutGraph(ports, graph, turbos);
    expect(placement.ducts.get(outlet.id)!.origin.distanceTo(new THREE.Vector3(...turbos.get(mount.node)!.outlet.point))).toBeLessThan(1e-9);
    expect(collectSnapTargets(graph, placement, ports).some((t) => t.kind === 'turboInlet')).toBe(true);
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
    const { spec, graph, ports, mount } = singleWithTurboAndOutlet();
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
    const { spec, graph, ports, mount } = singleWithTurboAndOutlet();
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

  it('keeps its own settings in a link, and drops ones that are not numbers', () => {
    const { graph } = singleWithTurbo();
    graph.turbos![0]!.settings = { boostTarget: 1.1e5, turboSize: 0.2, intercooler: 0.5, blowOff: 'none' };
    expect(graphFromJson(JSON.parse(JSON.stringify(graph)))).toEqual(graph);
    const raw = JSON.parse(JSON.stringify(graph));
    raw.turbos[0].settings = { boostTarget: 'lots', turboSize: 0.2 };
    expect(graphFromJson(raw)!.turbos![0]!.settings).toBeUndefined();
  });

  it('keeps the turbos in sync until told not to, then syncs them onto the first', () => {
    const { spec, graph } = singleWithTurbo();
    const second = newTurbo(graph, [0, 0, 0.3]);
    placeTurbo(graph, second);
    expect(turbosSynced(graph)).toBe(true);
    expect(setTurbosSynced(graph, spec, false)).toBeNull();
    expect(turbosSynced(graph)).toBe(false);
    for (const t of graph.turbos!) expect(t.settings).toEqual(engineTurboSettings(spec));
    graph.turbos![0]!.settings!.boostTarget = 1.2e5;
    graph.turbos![1]!.settings!.blowOff = 'none';
    const patch = setTurbosSynced(graph, spec, true);
    expect(patch).toEqual({ ...engineTurboSettings(spec), boostTarget: 1.2e5 });
    expect(turbosSynced(graph)).toBe(true);
    expect(graph.turbos!.every((t) => t.settings === undefined)).toBe(true);
  });

  it.each([false, true])('makes the engine boost, heard through the Wasm build, with an outlet pipe: %s', async (outlet) => {
    const { Sim } = await import('../src/audio/worklet/sim.js');
    const { spec, graph } = outlet ? singleWithTurboAndOutlet() : singleWithTurbo();
    const cfg = defaultConfig();
    cfg.engine = { ...spec, throttle: 1, rpm: 6000, freeRunning: false };
    // As the audio engine gives it the graph.
    cfg.graph = solverGraph(graph);
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
    expect(fittedBend(runner)).toBe(drawn.length);
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

describe('the RB26 preset, its exhaust compiled', () => {
  const preset = ENGINE_PRESETS.find((p) => p.name.includes('RB26'))!;
  const spec = presetEngine(preset, defaultConfig().engine);
  const graph = compileExhaust(spec, preset.pipe(), preset.collector!(), preset.turbos);

  it('has one turbo, as an engine of one bank does, every cylinder through it', () => {
    expect(validateGraph(graph, 6)).toEqual([]);
    expect(graph.turbos).toHaveLength(1);
    const node = graph.turbos![0]!.node;
    for (let c = 0; c < 6; c++) expect(pathToAir(graph, c).some((d) => d.to.kind === 'node' && d.to.node === node), `cylinder ${c}`).toBe(true);
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
    const t = target.point;
    const d = target.dir;
    const node = joinDuctEnd(graph, 'runner0', { position: [t.x, t.y, t.z], axis: [d.x, d.y, d.z] })!;
    // The pipe carrying the merged flow on, drawn from the junction.
    graph.ducts.push({
      id: 'onward',
      segments: [makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.06 })],
      from: { kind: 'node', node },
      to: { kind: 'mouth' },
    });
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
    expect(fittedBend(runner1)).toBe(runner1.segments.length - 1);
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

  it('matches the bore of the pipe before it and of the pipe it merges into, tapering between them', () => {
    const { graph, runner1 } = joined();
    const onward = graph.ducts.find((d) => d.id === 'onward')!;
    // Different bores either side, so the bend has something to match.
    onward.segments[0]!.dIn = 0.05;
    runner1.segments.at(-2)!.dOut = 0.036;
    refitBends(graph, ports, spec);
    const bend = runner1.segments.at(-1)!;
    expect(bend.dIn).toBeCloseTo(0.036, 12);
    expect(bend.dOut).toBeCloseTo(0.05, 12);
    expect(segmentDiameter(bend, 0.5)).toBeCloseTo(0.043, 12);
  });

  const total = (d: { segments: { length: number }[] }) => d.segments.reduce((a, seg) => a + seg.length, 0);

  it('matches a length: longer, with a swing on its way, locked with its bend', () => {
    const { graph, node, runner1 } = joined();
    refitBends(graph, ports, spec);
    const drawn = runner1.segments.length - 1;
    const want = total(runner1) + 0.25;
    const fit = matchLength(graph, ports, spec, 'runner1', want)!;
    expect(fit.reached).toBe(true);
    expect(total(runner1)).toBeCloseTo(want, 3);
    expect(runner1.swing).toBe(true);
    expect(runner1.segments).toHaveLength(drawn + 2);
    expect(lockedFrom(runner1)).toBe(drawn);
    expect(meets(graph, node)).toBe(true);
    // Refitting on the next rebuild keeps it the length it was matched to.
    refitBends(graph, ports, spec);
    expect(total(runner1)).toBeCloseTo(want, 3);
    // It survives a link, swing and all.
    expect(graphFromJson(JSON.parse(JSON.stringify(graph)))!.ducts.find((d) => d.id === 'runner1')!.swing).toBe(true);
  });

  it('matches a length: shorter, by its last straight, with no swing', () => {
    const { graph, node, runner1 } = joined();
    refitBends(graph, ports, spec);
    const count = runner1.segments.length;
    const want = total(runner1) - 0.02;
    const fit = matchLength(graph, ports, spec, 'runner1', want)!;
    expect(fit.reached).toBe(true);
    expect(total(runner1)).toBeCloseTo(want, 3);
    expect(runner1.swing).toBeUndefined();
    expect(runner1.segments).toHaveLength(count);
    expect(meets(graph, node)).toBe(true);
  });

  it('says so when a length is shorter than it can be made', () => {
    const { graph, runner1 } = joined();
    refitBends(graph, ports, spec);
    const fit = matchLength(graph, ports, spec, 'runner1', 0.05)!;
    expect(fit.reached).toBe(false);
    expect(fit.length).toBeGreaterThan(0.05);
    expect(total(runner1)).toBeCloseTo(fit.length, 9);
  });

  it('takes its swing off with its bend when it comes off', () => {
    const { graph, runner1 } = joined();
    refitBends(graph, ports, spec);
    const drawn = runner1.segments.length - 1;
    matchLength(graph, ports, spec, 'runner1', total(runner1) + 0.25);
    disconnectEnd(graph, 'runner1');
    expect(runner1.segments).toHaveLength(drawn);
    expect(runner1.swing).toBeUndefined();
    expect(runner1.fitted).toBeUndefined();
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

describe('the engine’s length', () => {
  it('is its block: first cylinder to last along the crank, and a pitch over', async () => {
    const { engineLength, cylinderSpacing, cylinderZ } = await import('../src/model/spec.js');
    const four = { ...defaultConfig().engine, cylinders: 4, vAngle: 0 } as EngineSpec;
    expect(engineLength(four)).toBeCloseTo(4 * cylinderSpacing(four), 12);
    const single = { ...defaultConfig().engine, cylinders: 1 } as EngineSpec;
    expect(engineLength(single)).toBeCloseTo(cylinderSpacing(single), 12);
    // A V8's banks are staggered, so its length takes in the stagger too.
    const v8 = { ...defaultConfig().engine, cylinders: 8, vAngle: 90, crankType: 'crossplane' } as EngineSpec;
    const zs = [...Array(8).keys()].map((c) => cylinderZ(v8, c));
    expect(engineLength(v8)).toBeCloseTo(Math.max(...zs) - Math.min(...zs) + cylinderSpacing(v8), 12);
    expect(engineLength(v8)).toBeGreaterThan(4 * cylinderSpacing(v8));
  });
});

describe('a loose pipe', () => {
  it('is put down attached to nothing, where it was put, and the solver is not given it', async () => {
    const { placeLoosePipe, solverGraph } = await import('../src/model/exhaustGraph.js');
    const { graph, ports } = single();
    const id = placeLoosePipe(graph, [0.4, 0.2, 0.3], 0.04, 0.3);
    expect(validateGraph(graph, 1)).toEqual([]);
    const placement = layoutGraph(ports, graph);
    expect(placement.ducts.get(id)!.origin.distanceTo(new THREE.Vector3(0.4, 0.2, 0.3))).toBeLessThan(1e-12);
    // Along the crank, as long as it was given.
    const place = placement.ducts.get(id)!;
    const end = layoutPipe(graph.ducts.find((d) => d.id === id)!.segments, place.origin, place.heading).joints.at(-1)!;
    expect(end.distanceTo(new THREE.Vector3(0.4, 0.2, 0.6))).toBeLessThan(1e-9);
    expect(solverGraph(graph).ducts.map((d) => d.id)).toEqual(['runner0']);
    // And survives a link.
    expect(graphFromJson(JSON.parse(JSON.stringify(graph)))).toEqual(graph);
  });

  it('is attached by a pipe drawn into its start, and is then fed like any other', async () => {
    const { attachToLooseStart, placeLoosePipe, solverGraph } = await import('../src/model/exhaustGraph.js');
    const { spec, graph, ports } = single();
    const id = placeLoosePipe(graph, [0.5, 0.25, 0.1], 0.04, 0.3);
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
  });
});

describe('deleting a pipe in the middle', () => {
  it('leaves the pipes that carried on from it loose, where they lie', async () => {
    const { loosenChildren } = await import('../src/scene/drawing.js');
    const { removeDuct, solverGraph } = await import('../src/model/exhaustGraph.js');
    const { graph, ports } = single();
    // Port → runner → junction → middle pipe → junction → tail, as drawing them would make.
    graph.ducts[0]!.to = { kind: 'node', node: 'a' };
    graph.ducts.push(
      { id: 'middle', segments: [makeSegment({ length: 0.25 })], from: { kind: 'node', node: 'a' }, to: { kind: 'node', node: 'b' } },
      { id: 'tail', segments: [makeSegment({ length: 0.3, yaw: 0.5 })], from: { kind: 'node', node: 'b' }, to: { kind: 'mouth' } },
    );
    const placement = layoutGraph(ports, graph);
    const was = placement.ducts.get('tail')!;
    loosenChildren(graph, 'middle', placement);
    expect(removeDuct(graph, 'middle')).toBe(true);
    const tail = graph.ducts.find((d) => d.id === 'tail')!;
    expect(tail.from.kind).toBe('free');
    const now = layoutGraph(ports, graph).ducts.get('tail')!;
    expect(now.origin.distanceTo(was.origin)).toBeLessThan(1e-9);
    expect(now.heading.angleTo(was.heading)).toBeLessThan(1e-9);
    expect(validateGraph(graph, 1)).toEqual([]);
    expect(solverGraph(graph).ducts.map((d) => d.id)).not.toContain('tail');
  });
});

/** A compiled header's primaries each finish in a bend into their collector, and stay equal-length. */
describe('a compiled header', () => {
  for (const name of ['V8, Chevrolet LT2', 'Inline four, Honda F20C']) {
    it(`bends each primary smoothly into its collector, at its own bore and length: ${name}`, () => {
      const preset = ENGINE_PRESETS.find((p) => p.name === name)!;
      const spec = presetEngine(preset, defaultConfig().engine);
      const graph = compileExhaust(spec, preset.pipe(), preset.collector!());
      const ports = portsOf(spec);
      const length = preset.pipe()[0]!.length;
      const bore = preset.pipe()[0]!.dIn;
      seatHeaders(graph, ports);
      refitBends(graph, ports, spec);
      expect(validateGraph(graph, spec.cylinders)).toEqual([]);
      const placement = layoutGraph(ports, graph, turboPortsOf(graph, spec));
      const runners = graph.ducts.filter((d) => d.role === 'runner');
      const totals = runners.map((r) => r.segments.reduce((a, s) => a + s.length, 0));
      // Equal-length, as they were tuned.
      totals.forEach((t, i) => expect(t, runners[i]!.id).toBeCloseTo(length, 3));
      // Some take a swing to make their length up: a second bend on the way, fitted and locked with the one
      // into the collector, so only the straight out of the port is edited.
      const swinging = runners.filter((r) => r.segments.filter((s) => s.curve).length === 2);
      expect(swinging.length).toBeGreaterThan(0);
      for (const r of swinging) expect(lockedFrom(r)).toBe(1);
      for (const r of runners) {
        expect(r.fitted, r.id).toBe(true);
        for (const s of r.segments) {
          expect(s.dIn).toBeCloseTo(bore, 9);
          expect(s.dOut).toBeCloseTo(bore, 9);
        }
        // Straight out of the port, square to the head.
        const port = ports[(r.from as { cylinder: number }).cylinder]!;
        const place = placement.ducts.get(r.id)!;
        const swept = layoutPipe(r.segments, place.origin, place.heading);
        expect(swept.stations[0]!.direction.angleTo(port.direction)).toBeLessThan(1e-6);
        // No kinks: every bend turns no tighter than one and a half bores, and each carries on the way the
        // one before it finished.
        r.segments.forEach((seg, k) => {
          const entry = k === 0 ? place.origin : swept.joints[k - 1]!;
          const entryDir = k === 0 ? place.heading : swept.jointDirections[k - 1]!;
          if (!seg.curve) return;
          expect(seg.yaw).toBe(0);
          expect(seg.pitch).toBe(0);
          const end = curveInWorld(seg.curve, entry, entryDir);
          expect(bendRadius(entry, entryDir, end.end, end.dir), `${r.id} bend ${k}`).toBeGreaterThan(1.5 * bore - 1e-3);
        });
      }
      for (const node of new Set(runners.map((r) => (r.to as { node: string }).node))) {
        expect(pipesMeetAt(graph, placement, node), node).toBe(true);
      }
      // It survives a link, collector and all, to the last few bits of a renormalised direction.
      const rounded = (g: unknown) => JSON.parse(JSON.stringify(g, (_, v) => (typeof v === 'number' ? Number(v.toFixed(12)) : v)));
      expect(rounded(graphFromJson(JSON.parse(JSON.stringify(graph))))).toEqual(rounded(graph));
      // Done once: seating again changes nothing.
      const before = JSON.stringify(graph);
      seatHeaders(graph, ports);
      expect(JSON.stringify(graph)).toBe(before);
    });
  }

  const lengthways = ENGINE_PRESETS.filter((p) => presetEngine(p, defaultConfig().engine).headerRun === 'lengthways').map((p) => p.name);
  it('builds every preset with a merge with lengthways headers, but those with manifolds', () => {
    const manifolds = ['Inline four, Toyota 3S-GTE', 'Inline five, Audi EA855 EVO', 'Inline six, Nissan RB26DETT', 'V6, Toyota 2GR'];
    const merging = ENGINE_PRESETS.filter((p) => exhaustLayoutOf(presetEngine(p, defaultConfig().engine)) !== 'open');
    expect(lengthways).toEqual(merging.map((p) => p.name).filter((n) => !manifolds.includes(n)));
  });

  it.each(lengthways)('builds lengthways headers with the header tool, back along the engine: %s', (name) => {
    const preset = ENGINE_PRESETS.find((p) => p.name === name)!;
    const spec = presetEngine(preset, defaultConfig().engine);
    expect(spec.headerRun).toBe('lengthways');
    const graph = compileExhaust(spec, preset.pipe(), preset.collector!());
    const ports = portsOf(spec);
    const length = preset.pipe().reduce((a, s) => a + s.length, 0);
    seatLengthwaysHeaders(graph, ports, spec);
    seatHeaders(graph, ports);
    refitBends(graph, ports, spec);
    expect(validateGraph(graph, spec.cylinders)).toEqual([]);
    const placement = layoutGraph(ports, graph);
    const junctions = graph.junctions ?? [];
    expect(junctions).toHaveLength(new Set(collectorGroups(spec).filter((g) => g >= 0)).size);
    for (const j of junctions) {
      // The collector leaves rearwards, along the crank.
      expect(j.axis).toEqual([0, 0, 1]);
      const runners = graph.ducts.filter((d) => d.role === 'runner' && d.to.kind === 'node' && d.to.node === j.node);
      expect(runners.length).toBeGreaterThan(1);
      // Equal-length, at the length they were tuned to.
      for (const r of runners) expect(r.segments.reduce((a, s) => a + s.length, 0), r.id).toBeCloseTo(length, 3);
      const at = runners.map((r) => ports[(r.from as { cylinder: number }).cylinder]!);
      // Out from its ports the way they point, clear of the head, and behind the frontmost of them.
      const out = at.reduce((v, p) => v.add(p.direction), new THREE.Vector3()).normalize();
      const merge = new THREE.Vector3(...j.position);
      for (const p of at) expect(merge.clone().sub(p.position).dot(out)).toBeGreaterThan(0);
      expect(j.position[2]).toBeGreaterThan(Math.min(...at.map((p) => p.position.z)));
      expect(pipesMeetAt(graph, placement, j.node), j.node).toBe(true);
    }
    // Done once: seating again changes nothing.
    const before = JSON.stringify(graph);
    seatLengthwaysHeaders(graph, ports, spec);
    expect(JSON.stringify(graph)).toBe(before);
  });
});

/**
 * A compiled manifold, laid along the engine with each port's stub bent into it, as one is drawn: each
 * preset's engine, its exhaust compiled rather than any drawn for it, and without turbos, which take the
 * ports' pipes straight into them instead.
 */
describe('a compiled manifold', () => {
  const seat = (name: string) => {
    const preset = ENGINE_PRESETS.find((p) => p.name === name)!;
    const spec = presetEngine(preset, defaultConfig().engine);
    const graph = compileExhaust(spec, preset.pipe(), preset.collector!());
    const compiled = JSON.parse(JSON.stringify(graph)) as ExhaustGraph;
    const ports = portsOf(spec);
    seatManifolds(graph, ports);
    refitBends(graph, ports, spec);
    return { spec, graph, compiled, ports };
  };

  it.each(['V6, Toyota 2GR', 'Inline six, Nissan RB26DETT'])('keeps every stub, and each cylinder\'s path to air, its compiled length: %s', (name) => {
    const { spec, graph, compiled } = seat(name);
    expect(validateGraph(graph, spec.cylinders)).toEqual([]);
    const total = (ducts: { segments: { length: number }[] }[]) => ducts.reduce((a, d) => a + d.segments.reduce((b, s) => b + s.length, 0), 0);
    for (const d of graph.ducts.filter((d) => d.role === 'stub')) {
      expect(total([d]), d.id).toBeCloseTo(compiled.ducts.find((c) => c.id === d.id)!.segments[0]!.length, 3);
    }
    const upstream = (g: ExhaustGraph, c: number) => pathToAir(g, c).filter((d) => d.role === 'stub' || d.role === 'manifold');
    for (let c = 0; c < spec.cylinders; c++) expect(total(upstream(graph, c)), `cylinder ${c}`).toBeCloseTo(total(upstream(compiled, c)), 3);
  });

  it.each(['V6, Toyota 2GR', 'Inline six, Nissan RB26DETT'])('fixes only where each pipe along the engine starts: %s', (name) => {
    const { graph } = seat(name);
    const starts = graph.ducts.filter((d) => d.role === 'manifold' && d.from.kind === 'node' && junctionAt(graph, d.from.node));
    const firsts = graph.ducts.filter((d) => d.role === 'stub' && d.to.kind === 'node' && junctionAt(graph, d.to.node));
    expect(starts.length).toBe(firsts.length);
    for (const d of graph.ducts.filter((d) => d.role === 'manifold' && !starts.includes(d))) {
      expect(junctionAt(graph, (d.from as { node: string }).node), d.id).toBeUndefined();
    }
    for (const d of starts) expect(d.headingFrame).toBe('world');
  });

  it.each(['V6, Toyota 2GR', 'Inline six, Nissan RB26DETT'])('bends each stub into a pipe along the engine: %s', (name) => {
    const { spec, graph, ports } = seat(name);
    const placement = layoutGraph(ports, graph, turboPortsOf(graph, spec));
    for (const node of new Set(graph.ducts.flatMap((d) => (d.role === 'stub' && d.to.kind === 'node' ? [d.to.node] : [])))) {
      expect(pipesMeetAt(graph, placement, node) || turboPortsOf(graph, spec).has(node), node).toBe(true);
    }
    for (const j of graph.junctions ?? []) expect(j.axis).toEqual([0, 0, 1]);
    // Each stub arrives along the engine, one smooth bend from its port.
    for (const d of graph.ducts.filter((d) => d.role === 'stub')) {
      const place = placement.ducts.get(d.id)!;
      const bend = d.segments[0]!;
      expect(bend.curve, d.id).toBeDefined();
      const end = curveInWorld(bend.curve!, place.origin, place.heading);
      expect(Math.abs(end.dir.z), d.id).toBeCloseTo(1, 6);
    }
    // Done once: seating again changes nothing.
    const before = JSON.stringify(graph);
    seatManifolds(graph, ports);
    expect(JSON.stringify(graph)).toBe(before);
  });

  it.each(['V6, Toyota 2GR', 'Inline six, Nissan RB26DETT'])('loads the exhaust drawn for it, which fits it and nothing reseats: %s', (name) => {
    const preset = ENGINE_PRESETS.find((p) => p.name === name)!;
    const spec = presetEngine(preset, defaultConfig().engine);
    const graph = graphFromJson(preset.graph!())!;
    expect(validateGraph(graph, spec.cylinders)).toEqual([]);
    const ports = portsOf(spec);
    const before = JSON.stringify(graph);
    seatManifolds(graph, ports);
    seatTurbos(graph, ports, spec);
    seatLengthwaysHeaders(graph, ports, spec);
    seatHeaders(graph, ports);
    expect(JSON.stringify(graph)).toBe(before);
    refitBends(graph, ports, spec);
    const turbos = turboPortsOf(graph, spec);
    expect(turbos.size).toBe(preset.turbos ?? 0);
    const placement = layoutGraph(ports, graph, turbos);
    for (const node of new Set(graph.ducts.flatMap((d) => (d.to.kind === 'node' && !turbos.has(d.to.node) ? [d.to.node] : [])))) {
      expect(pipesMeetAt(graph, placement, node), node).toBe(true);
    }
  });

});


describe('a turbo put down on its own', () => {
  it('lies along the crank with its inlet facing up', () => {
    const near = (v: number[], w: number[]) => v.forEach((x, i) => expect(x).toBeCloseTo(w[i]!, 9));
    // The shaft, compressor forwards.
    near(quatRotate([1, 0, 0], UPRIGHT), [0, 0, -1]);
    // Gas arrives at the inlet downwards, so the flange faces up.
    near(quatRotate([0, 0, 1], UPRIGHT), [0, -1, 0]);
  });
});
