/**
 * The X-pipe: two pipes cut at two points each and crossed, each running on through the crossing into the
 * other's leg, which carries on as the other pipe did.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';

import { Sim } from '../src/audio/worklet/sim.js';
import { compileCollectorLayout, endsAt, graphFromJson, junctionAt, solverGraph, validateGraph, type ExhaustGraph } from '../src/model/exhaustGraph.js';
import { defaultConfig, makeSegment, type EngineSpec } from '../src/model/spec.js';
import { EngineMesh } from '../src/scene/EngineMesh.js';
import { layoutGraph, type ExhaustPlacement, type ExhaustPort } from '../src/scene/exhaustLayout.js';
import { layoutPipe } from '../src/scene/PipeMesh.js';
import { moveJunction, refitBends } from '../src/scene/turboPlacement.js';
import { applyXPipe, boreAt, crossingBore, defaultCrossing, setCrossingBore, snapPoints, xLegs, xPairs, type PipePoint, type XPlan } from '../src/scene/xpipeTool.js';

const v8 = { ...defaultConfig().engine, cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' } as EngineSpec;
const portsOf = (spec: EngineSpec): ExhaustPort[] => {
  const mesh = new EngineMesh(spec);
  return Array.from({ length: mesh.bankCount }, (_, i) => mesh.exhaustPort(i));
};
const total = (segments: { length: number }[]) => segments.reduce((a, s) => a + s.length, 0);

/**
 * A V8 with each bank's merge moved out beside it and a straight 1.5 m pipe from each running rearwards to
 * air, 0.9 m apart: true duals.
 */
function duals(): { graph: ExhaustGraph; ports: ExhaustPort[]; placement: ExhaustPlacement; tails: string[] } {
  const ports = portsOf(v8);
  const graph = compileCollectorLayout(v8, [], [makeSegment({ length: 1.5, dIn: 0.07, dOut: 0.07 })]);
  const collectors = graph.ducts.filter((d) => d.role === 'collector');
  graph.junctions = collectors.map((d, i) => ({
    node: (d.from as { node: string }).node,
    position: [i === 0 ? -0.45 : 0.45, -0.07, 0.3],
    axis: [0, 0, 1],
  }));
  refitBends(graph, ports, v8);
  return { graph, ports, placement: layoutGraph(ports, graph), tails: collectors.map((d) => d.id) };
}

/** The point `x` along duct `id`, as the editor picks one. */
function pointOn(graph: ExhaustGraph, placement: ExhaustPlacement, id: string, x: number): PipePoint {
  const duct = graph.ducts.find((d) => d.id === id)!;
  const place = placement.ducts.get(id)!;
  const layout = layoutPipe(duct.segments, place.origin, place.heading);
  const station = layout.stations.reduce((best, s) => (Math.abs(s.x - x) < Math.abs(best.x - x) ? s : best));
  return { duct: id, x: station.x, point: station.position.clone(), dir: station.direction.clone(), bore: boreAt(duct.segments, station.x) };
}

function planOf(graph: ExhaustGraph, placement: ExhaustPlacement, tails: string[]): XPlan {
  const points = tails.flatMap((id) => [pointOn(graph, placement, id, 1.2), pointOn(graph, placement, id, 0.2)]);
  const picked = xPairs(points);
  if ('error' in picked) throw new Error(picked.error);
  return { pairs: picked.pairs, cross: defaultCrossing(picked.pairs) };
}

describe('the X-pipe', () => {
  it('pairs two points on each of two pipes, in the order each pipe runs', () => {
    const { graph, placement, tails } = duals();
    const [a, b] = tails as [string, string];
    const at = (id: string, x: number) => pointOn(graph, placement, id, x);
    const picked = xPairs([at(a, 1.0), at(b, 0.3), at(a, 0.4), at(b, 0.9)]);
    expect('pairs' in picked).toBe(true);
    if (!('pairs' in picked)) return;
    for (const p of picked.pairs) expect(p.from.x).toBeLessThan(p.to.x);
    expect(picked.pairs.map((p) => p.duct).sort()).toEqual([a, b].sort());

    expect('error' in xPairs([at(a, 0.4), at(a, 1.0), at(b, 0.4)])).toBe(true);
    expect('error' in xPairs([at(a, 0.2), at(a, 0.6), at(a, 1.0), at(a, 1.3)])).toBe(true);
    expect('error' in xPairs([at(a, 0.4), at(a, 0.405), at(b, 0.4), at(b, 1.0)])).toBe(true);
  });

  it('cuts both pipes and crosses them at one junction of four', () => {
    const { graph, ports, tails } = duals();
    const before = new Map(tails.map((id) => [id, total(graph.ducts.find((d) => d.id === id)!.segments)]));
    const plan = planOf(graph, layoutGraph(ports, graph), tails);
    const node = applyXPipe(graph, plan)!;
    expect(node).not.toBeNull();
    expect(validateGraph(graph, 8)).toEqual([]);

    const ends = endsAt(graph, node);
    const feeds = ends.filter((e) => e.end === 'outlet').map((e) => e.duct);
    const legs = ends.filter((e) => e.end === 'inlet').map((e) => e.duct);
    expect(feeds.map((d) => d.id).sort()).toEqual([...tails].sort());
    expect(legs).toHaveLength(2);
    for (const leg of legs) expect(leg.to).toEqual({ kind: 'mouth' });
    const pin = junctionAt(graph, node)!;
    expect(new THREE.Vector3(...pin.position).distanceTo(plan.cross)).toBeLessThan(1e-9);
    // Each crosses into the other's leg.
    const legOf = (id: string) => legs.find((l) => l.id.startsWith(`${id}-`))!.id;
    expect(pin.through).toEqual({ [tails[0]!]: legOf(tails[1]!), [tails[1]!]: legOf(tails[0]!) });

    // What each pipe keeps: its first 0.2 m, then a bend in; and on its leg, a bend out, then its last 0.3 m.
    for (const id of tails) {
      const feed = graph.ducts.find((d) => d.id === id)!;
      const leg = graph.ducts.find((d) => d.id === legOf(id))!;
      expect(total(feed.segments.slice(0, -1))).toBeCloseTo(0.2, 1);
      expect(total(leg.segments.slice(1))).toBeCloseTo(before.get(id)! - 1.2, 1);
      expect(feed.fitted).toBe(true);
    }
  });

  it('keeps the X as every rebuild lays it out: each pipe running straight through into the other leg', () => {
    const { graph, ports, tails } = duals();
    const plan = planOf(graph, layoutGraph(ports, graph), tails);
    const node = applyXPipe(graph, plan)!;
    refitBends(graph, ports, v8);
    const placement = layoutGraph(ports, graph);
    const pin = junctionAt(graph, node)!;
    const sweep = (id: string) => {
      const d = graph.ducts.find((x) => x.id === id)!;
      const at = placement.ducts.get(id)!;
      return layoutPipe(d.segments, at.origin, at.heading);
    };
    for (const pair of plan.pairs) {
      const feed = sweep(pair.duct);
      const onto = sweep(pin.through![pair.duct]!);
      // Into the crossing, along the way the other pipe's leg leaves it.
      expect(feed.joints.at(-1)!.distanceTo(plan.cross)).toBeLessThan(1e-3);
      expect(feed.jointDirections.at(-1)!.angleTo(onto.stations[0]!.direction)).toBeLessThan(0.02);
      // Its own leg leaves the crossing and bends out onto the pipe where it was cut, running along it.
      const own = sweep(Object.values(pin.through!).find((id) => id.startsWith(`${pair.duct}-`))!);
      expect(own.stations[0]!.position.distanceTo(plan.cross)).toBeLessThan(1e-9);
      expect(own.joints[0]!.distanceTo(pair.to.point)).toBeLessThan(1e-3);
      expect(own.jointDirections[0]!.angleTo(pair.to.dir)).toBeLessThan(0.02);
    }
  });

  it('moves its crossing with the legs bending out again to where they went, and what follows them staying put', () => {
    const { graph, ports, tails } = duals();
    const plan = planOf(graph, layoutGraph(ports, graph), tails);
    const node = applyXPipe(graph, plan)!;
    refitBends(graph, ports, v8);
    const pin = junctionAt(graph, node)!;
    const legIds = Object.values(pin.through!);
    const sweep = (id: string, placement: ExhaustPlacement) => {
      const d = graph.ducts.find((x) => x.id === id)!;
      const at = placement.ducts.get(id)!;
      return layoutPipe(d.segments, at.origin, at.heading);
    };
    const before = layoutGraph(ports, graph);
    const ends = new Map(legIds.map((id) => [id, sweep(id, before).joints.map((j) => j.clone())]));

    const to = plan.cross.clone().add(new THREE.Vector3(0.05, 0.1, -0.04));
    moveJunction(graph, ports, v8, node, [to.x, to.y, to.z], pin.axis);
    expect(validateGraph(graph, 8)).toEqual([]);
    const after = layoutGraph(ports, graph);
    for (const id of legIds) {
      const now = sweep(id, after);
      expect(now.stations[0]!.position.distanceTo(to)).toBeLessThan(1e-9);
      // Every joint from the bend's end on is where it was.
      now.joints.forEach((j, k) => expect(j.distanceTo(ends.get(id)![k]!)).toBeLessThan(1e-6));
    }
    for (const pair of plan.pairs) {
      const feed = sweep(pair.duct, after);
      expect(feed.joints.at(-1)!.distanceTo(to)).toBeLessThan(1e-3);
      expect(feed.jointDirections.at(-1)!.angleTo(sweep(pin.through![pair.duct]!, after).stations[0]!.direction)).toBeLessThan(0.02);
    }
  });

  it('sets the bore all four pipes meet at in the crossing', () => {
    const { graph, ports, tails } = duals();
    const node = applyXPipe(graph, planOf(graph, layoutGraph(ports, graph), tails))!;
    refitBends(graph, ports, v8);
    expect(crossingBore(graph, node)).toBeCloseTo(0.07, 9);
    setCrossingBore(graph, node, 0.09);
    refitBends(graph, ports, v8);
    expect(crossingBore(graph, node)).toBeCloseTo(0.09, 9);
    for (const e of endsAt(graph, node)) {
      const seg = e.end === 'inlet' ? e.duct.segments[0]! : e.duct.segments.at(-1)!;
      expect(e.end === 'inlet' ? seg.dIn : seg.dOut).toBeCloseTo(0.09, 9);
    }
  });

  it('keeps which leg each pipe runs into when saved and read back', () => {
    const { graph, ports, tails } = duals();
    const node = applyXPipe(graph, planOf(graph, layoutGraph(ports, graph), tails))!;
    const back = graphFromJson(JSON.parse(JSON.stringify(graph)))!;
    expect(junctionAt(back, node)!.through).toEqual(junctionAt(graph, node)!.through);
  });

  it('snaps to where a pipe starts, where its segments meet, and where it ends, but not into a fitted bend', () => {
    const { graph, placement, tails } = duals();
    const tail = graph.ducts.find((d) => d.id === tails[0])!;
    tail.segments = [makeSegment({ length: 0.5, dIn: 0.07 }), makeSegment({ length: 1.0, dIn: 0.07 })];
    const place = placement.ducts.get(tail.id)!;
    const at = snapPoints(tail, place);
    expect(at.map((p) => p.x)).toEqual([0, 0.5, 1.5]);
    expect(at[0]!.point.distanceTo(place.origin)).toBeLessThan(1e-12);
    const swept = layoutPipe(tail.segments, place.origin, place.heading);
    expect(at[2]!.point.distanceTo(swept.joints.at(-1)!)).toBeLessThan(1e-12);

    // A runner, its last segment the bend fitted into its merge: its start only.
    const runner = graph.ducts.find((d) => d.from.kind === 'valve' && d.fitted)!;
    const offered = snapPoints(runner, placement.ducts.get(runner.id)!);
    const bendStarts = runner.segments.slice(0, -1).reduce((a, sg) => a + sg.length, 0);
    expect(offered.every((p) => p.x <= bendStarts + 1e-9)).toBe(true);
  });

  it('crosses from where the pipes start to where they end', () => {
    const { graph, ports, placement, tails } = duals();
    const points = tails.flatMap((id) => {
      const d = graph.ducts.find((x) => x.id === id)!;
      const snaps = snapPoints(d, placement.ducts.get(id)!);
      return [snaps[0]!, snaps.at(-1)!];
    });
    const picked = xPairs(points);
    if ('error' in picked) throw new Error(picked.error);
    const plan: XPlan = { pairs: picked.pairs, cross: defaultCrossing(picked.pairs) };
    const node = applyXPipe(graph, plan)!;
    expect(node).not.toBeNull();
    expect(validateGraph(graph, 8)).toEqual([]);
    refitBends(graph, ports, v8);
    const after = layoutGraph(ports, graph);
    for (const pair of plan.pairs) {
      // Nothing of the pipe before the bend in, and nothing after the bend out: each leg ends where it did.
      expect(graph.ducts.find((d) => d.id === pair.duct)!.segments).toHaveLength(1);
      const leg = graph.ducts.find((d) => d.id.startsWith(`${pair.duct}-`))!;
      expect(leg.segments).toHaveLength(1);
      expect(leg.to).toEqual({ kind: 'mouth' });
      const at = after.ducts.get(leg.id)!;
      expect(layoutPipe(leg.segments, at.origin, at.heading).joints.at(-1)!.distanceTo(pair.to.point)).toBeLessThan(1e-3);
    }
  });

  it('says when a bend turns tighter than a pipe can be bent', () => {
    const { graph, ports, tails } = duals();
    const plan = planOf(graph, layoutGraph(ports, graph), tails);
    expect(xLegs(plan).some((l) => l.tight)).toBe(false);
    // Crossing right beside one pipe's first point, square off it.
    const a = plan.pairs[0].from;
    const across = new THREE.Vector3(0, 1, 0).cross(a.dir).normalize();
    expect(xLegs({ ...plan, cross: a.point.clone().addScaledVector(across, 0.03) }).some((l) => l.tight)).toBe(true);
  });

  it('runs in the solver', () => {
    const { graph, ports, tails } = duals();
    applyXPipe(graph, planOf(graph, layoutGraph(ports, graph), tails));
    refitBends(graph, ports, v8);
    const cfg = defaultConfig();
    cfg.engine = { ...v8, rpm: 4000, throttle: 1, freeRunning: false };
    cfg.graph = solverGraph(graph);
    const sim = new Sim(48000, cfg);
    const out = sim.render(9600);
    sim.free();
    expect(out.every((v) => Number.isFinite(v))).toBe(true);
    expect(Math.max(...out.map(Math.abs))).toBeGreaterThan(1e-3);
  });
});
