/**
 * Turning clicks into pipe segments.
 *
 * The thing that has to be true and is not obvious: a fitted segment's *swept* end lands where the click
 * was. `PipeSegment` does not describe a straight line — `layoutPipe` turns the heading a little at every
 * station, so any segment with a bend is an arc and "point it at the target" misses by the whole of the
 * bend. That was already the cause of one bug, where runners were aimed at a collector by their inlet
 * direction and then curved away from it.
 *
 * So every case here fits a segment and then checks against the real sweep, not against the arithmetic
 * that produced it.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';

import {
  MIN_DRAW_LENGTH,
  collectSnapTargets,
  continuingDiameter,
  fitSegment,
  headingOffsetTo,
  nearestSnap,
  quantiseLength,
  quantiseTurn,
  routeTip,
} from '../src/scene/drawing.js';
import { layoutGraph, type ExhaustPort } from '../src/scene/exhaustLayout.js';
import { layoutPipe } from '../src/scene/PipeMesh.js';
import {
  compileLayout,
  joinDuctEnd,
  newDuctId,
  newNodeId,
  splitDuctAt,
  splitSegments,
  validateGraph,
  type ExhaustGraph,
} from '../src/model/exhaustGraph.js';
import { EngineMesh } from '../src/scene/EngineMesh.js';
import { defaultConfig, makeSegment, segmentDiameter, type EngineSpec, type PipeSegment } from '../src/model/spec.js';

/** Where a single fitted segment actually finishes, swept the way the renderer sweeps it. */
function sweptEnd(seg: PipeSegment, entry: THREE.Vector3, dir: THREE.Vector3): THREE.Vector3 {
  const layout = layoutPipe([seg], entry, dir);
  return layout.joints[layout.joints.length - 1]!.clone();
}

describe('fitting a segment to a clicked point', () => {
  const entry = new THREE.Vector3(0.2, 0.3, 0);

  it.each([
    ['straight ahead', new THREE.Vector3(1, 0, 0), new THREE.Vector3(0.6, 0.3, 0)],
    ['a gentle left', new THREE.Vector3(1, 0, 0), new THREE.Vector3(0.6, 0.3, 0.15)],
    ['a gentle climb', new THREE.Vector3(1, 0, 0), new THREE.Vector3(0.6, 0.45, 0)],
    ['both at once', new THREE.Vector3(1, 0, 0), new THREE.Vector3(0.55, 0.42, -0.2)],
    ['off an angled entry', new THREE.Vector3(0.7, 0.3, 0.6), new THREE.Vector3(0.5, 0.5, 0.35)],
    ['a short hop', new THREE.Vector3(1, 0, 0), new THREE.Vector3(0.26, 0.31, 0.02)],
  ] as Array<[string, THREE.Vector3, THREE.Vector3]>)(
    'lands on the target: %s',
    (_name, dir, target) => {
      const seg = fitSegment(entry, dir, target, { dIn: 0.042, dOut: 0.042 });
      const end = sweptEnd(seg, entry, dir);
      const miss = end.distanceTo(target);
      expect(
        miss,
        `missed by ${(miss * 1000).toFixed(1)} mm over a ${(target.distanceTo(entry) * 1000).toFixed(0)} mm span`,
      ).toBeLessThan(0.002);
    },
  );

  it('keeps the diameters it was given', () => {
    const seg = fitSegment(entry, new THREE.Vector3(1, 0, 0), new THREE.Vector3(0.6, 0.35, 0.1), {
      kind: 'cone',
      dIn: 0.042,
      dOut: 0.055,
    });
    expect(seg.kind).toBe('cone');
    expect(seg.dIn).toBeCloseTo(0.042, 9);
    expect(seg.dOut).toBeCloseTo(0.055, 9);
  });

  it('never produces a segment shorter than the minimum', () => {
    const seg = fitSegment(entry, new THREE.Vector3(1, 0, 0), entry.clone(), {});
    expect(seg.length).toBeGreaterThanOrEqual(MIN_DRAW_LENGTH);
  });

  /** A turn beyond a right angle is not a segment, it is a doubling back; it must not explode. */
  it('stays finite on an absurd turn', () => {
    const seg = fitSegment(entry, new THREE.Vector3(1, 0, 0), new THREE.Vector3(-0.3, 0.3, 0), {});
    expect(Number.isFinite(seg.length)).toBe(true);
    expect(Number.isFinite(seg.yaw)).toBe(true);
    expect(Number.isFinite(seg.pitch)).toBe(true);
    expect(seg.length).toBeLessThan(5);
  });
});

describe('quantising', () => {
  it('snaps a turn to a multiple, measured from the pipe being left', () => {
    const from = new THREE.Vector3(1, 0, 0);
    // 20 degrees away, snapped to the nearest 15 -> 15.
    const to = from.clone().applyAxisAngle(new THREE.Vector3(0, 1, 0), (20 * Math.PI) / 180);
    const snapped = quantiseTurn(to, from, 15);
    expect((from.angleTo(snapped) * 180) / Math.PI).toBeCloseTo(15, 4);
  });

  it('leaves a direction alone when it is already on a multiple', () => {
    const from = new THREE.Vector3(1, 0, 0);
    const to = from.clone().applyAxisAngle(new THREE.Vector3(0, 0, 1), (45 * Math.PI) / 180);
    const snapped = quantiseTurn(to, from, 15);
    expect(snapped.angleTo(to)).toBeLessThan
      (1e-6);
  });

  it('is a no-op with no step', () => {
    const from = new THREE.Vector3(1, 0, 0);
    const to = new THREE.Vector3(0.4, 0.5, 0.2).normalize();
    expect(quantiseTurn(to, from, 0).angleTo(to)).toBeLessThan(1e-9);
  });

  it('rounds lengths to a grid but never below the minimum', () => {
    expect(quantiseLength(0.237, 0.025)).toBeCloseTo(0.225, 9);
    expect(quantiseLength(0.001, 0.025)).toBeGreaterThanOrEqual(MIN_DRAW_LENGTH);
    expect(quantiseLength(0.237, 0)).toBeCloseTo(0.237, 9);
  });
});

describe('snap targets', () => {
  const spec = { ...defaultConfig().engine, cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' } as EngineSpec;
  const ports = (): ExhaustPort[] => {
    const mesh = new EngineMesh(spec, new THREE.Plane(new THREE.Vector3(0, 0, -1), 0.001));
    return Array.from({ length: mesh.bankCount }, (_, i) => mesh.exhaustPort(i));
  };

  /**
   * Every port is offered, occupied or not.
   *
   * An earlier version left occupied ports out, on the reasoning that a cylinder may only have one pipe.
   * That made draw mode useless: a compiled engine gives every cylinder a runner, so no port was ever
   * clickable and a route could only start from a junction. Drawing from an occupied port replaces what is
   * there, and `occupied` is what says so.
   */
  it('offers every port and every junction', () => {
    const graph = compileLayout(spec, [makeSegment({ length: 0.4, dIn: 0.042 })], [makeSegment({ length: 0.5, dIn: 0.055 })]);
    const p = ports();
    const targets = collectSnapTargets(graph, layoutGraph(p, graph), p);

    const portTargets = targets.filter((t) => t.kind === 'port');
    expect(portTargets).toHaveLength(8);
    // Each one names the runner it would replace.
    expect(portTargets.every((t) => (t as { occupied?: string }).occupied?.startsWith('runner'))).toBe(true);
    // A manifold along each bank: a junction where each of the last three cylinders joins it.
    expect(targets.filter((t) => t.kind === 'node').map((t) => (t as { node: string }).node).sort())
      .toEqual(['merge0', 'merge0-1', 'merge0-2', 'merge1', 'merge1-1', 'merge1-2']);
    // The two collectors vent to air, so their ends are joinable; the runners do not.
    expect(targets.filter((t) => t.kind === 'ductEnd').map((t) => (t as { duct: string }).duct).sort())
      .toEqual(['collector0', 'collector1']);
  });

  it('marks a port with no pipe as free', () => {
    const graph = compileLayout(spec, [makeSegment({ length: 0.4 })], [makeSegment({ length: 0.5 })]);
    const p = ports();
    graph.ducts = graph.ducts.filter((d) => d.id !== 'runner3');
    const targets = collectSnapTargets(graph, layoutGraph(p, graph), p);
    const free = targets.filter(
      (t) => t.kind === 'port' && (t as { occupied?: string }).occupied === undefined,
    );
    expect(free.map((t) => (t as { cylinder: number }).cylinder)).toEqual([3]);
  });

  it('picks the nearest target on screen and gives up beyond the radius', () => {
    const graph = compileLayout(spec, [makeSegment({ length: 0.4 })], [makeSegment({ length: 0.5 })]);
    const p = ports();
    const targets = collectSnapTargets(graph, layoutGraph(p, graph), p);
    const camera = new THREE.PerspectiveCamera(42, 16 / 9, 0.01, 60);
    camera.position.set(0.85, 0.55, 1.15);
    camera.lookAt(0.45, 0.2, 0);
    camera.updateMatrixWorld(true);

    const viewport = { width: 1600, height: 900 };
    const first = targets[0]!;
    const ndc = first.point.clone().project(camera);
    const onIt = nearestSnap(targets, new THREE.Vector2(ndc.x, ndc.y), camera, 12, viewport);
    expect(onIt?.point.distanceTo(first.point)).toBeLessThan(1e-9);

    // Far off in a corner: nothing should be within a dozen pixels.
    const away = nearestSnap(targets, new THREE.Vector2(-0.98, -0.98), camera, 12, viewport);
    expect(away).toBeNull();
  });

  it('continues at the diameter of whatever it is leaving', () => {
    const duct = { id: 'x', segments: [makeSegment({ kind: 'cone', length: 0.2, dIn: 0.04, dOut: 0.06 })], from: { kind: 'valve' as const, cylinder: 0 }, to: { kind: 'mouth' as const } };
    expect(continuingDiameter(duct, 0.042)).toBeCloseTo(0.06, 9);
    expect(continuingDiameter(null, 0.042)).toBeCloseTo(0.042, 9);
  });

  it('reports the tip of a route being drawn', () => {
    const place = { origin: new THREE.Vector3(0.1, 0.2, 0), heading: new THREE.Vector3(1, 0, 0) };
    const empty = routeTip([], place);
    expect(empty.point.distanceTo(place.origin)).toBeLessThan(1e-9);

    const segs = [makeSegment({ length: 0.3, dIn: 0.042 })];
    const tip = routeTip(segs, place);
    expect(tip.point.distanceTo(place.origin)).toBeCloseTo(0.3, 6);
    expect(tip.dir.length()).toBeCloseTo(1, 9);
  });
});

/**
 * A T-branch: dropping a pipe onto the side of another one.
 *
 * The duct it lands on becomes two ducts with a junction between them, which is the only thing the solver
 * needs — it has no notion of a branch part way along a duct, and does not need one.
 */
describe('splitting a duct for a T', () => {
  const pipe = () => [
    makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.04 }),
    makeSegment({ kind: 'cone', length: 0.2, dIn: 0.04, dOut: 0.06 }),
  ];

  it('keeps the total length and the diameter profile', () => {
    const before = pipe();
    const halves = splitSegments(before, 0.4)!;
    expect(halves).not.toBeNull();
    const total = (segs: PipeSegment[]) => segs.reduce((a, s) => a + s.length, 0);
    expect(total(halves[0]) + total(halves[1])).toBeCloseTo(total(before), 9);
    // The cut is inside the cone, so the two new faces have to agree on the diameter there.
    expect(segmentDiameter(halves[0][halves[0].length - 1]!, 1)).toBeCloseTo(
      segmentDiameter(halves[1][0]!, 0),
      9,
    );
    // And the ends are untouched.
    expect(segmentDiameter(halves[0][0]!, 0)).toBeCloseTo(0.04, 9);
    expect(segmentDiameter(halves[1][halves[1].length - 1]!, 1)).toBeCloseTo(0.06, 9);
  });

  /**
   * A chamber cannot be halved. Its profile is a throat, a body, then a throat, so half of one is not a
   * chamber and interpolating its diameter at the cut would quietly turn a muffler into a cone.
   */
  it('moves a cut inside a chamber to the nearer end', () => {
    const segs = [
      makeSegment({ kind: 'pipe', length: 0.2, dIn: 0.04 }),
      makeSegment({ kind: 'chamber', length: 0.3, dIn: 0.04, dOut: 0.12 }),
      makeSegment({ kind: 'pipe', length: 0.2, dIn: 0.04 }),
    ];
    for (const [x, expectedHeadLength] of [[0.25, 0.2], [0.45, 0.5]] as Array<[number, number]>) {
      const halves = splitSegments(segs, x)!;
      expect(halves[0].reduce((a, s) => a + s.length, 0)).toBeCloseTo(expectedHeadLength, 9);
      // Still exactly one chamber, on one side or the other.
      const chambers = [...halves[0], ...halves[1]].filter((s) => s.kind === 'chamber');
      expect(chambers).toHaveLength(1);
      expect(chambers[0]!.length).toBeCloseTo(0.3, 9);
    }
  });

  it('refuses a cut that would leave an empty half', () => {
    expect(splitSegments(pipe(), 0)).toBeNull();
    expect(splitSegments(pipe(), 0.5)).toBeNull();
    expect(splitSegments(pipe(), 99)).toBeNull();
    expect(splitSegments([], 0.1)).toBeNull();
  });

  it('splits a duct in the graph and leaves it solvable', () => {
    const spec = { ...defaultConfig().engine, cylinders: 2, vAngle: 45, exhaustLayout: '2into1' } as EngineSpec;
    const graph = compileLayout(spec, pipe(), [makeSegment({ length: 0.5, dIn: 0.06 })]);
    const node = splitDuctAt(graph, 'runner0', 0.4);

    expect(node).toBeTruthy();
    // The upstream half keeps the duct's identity and now ends at the new junction.
    const upstream = graph.ducts.find((d) => d.id === 'runner0')!;
    expect(upstream.to).toEqual({ kind: 'node', node });
    // The downstream half carries the original outlet and sits right after it in the list.
    const at = graph.ducts.indexOf(upstream);
    const downstream = graph.ducts[at + 1]!;
    expect(downstream.from).toEqual({ kind: 'node', node });
    expect(downstream.to).toEqual({ kind: 'node', node: 'merge0' });
    // A junction of two is still a junction, so the graph stands up.
    expect(validateGraph(graph, 2)).toEqual([]);
  });

  it('hands out ids nothing is using', () => {
    const spec = { ...defaultConfig().engine, cylinders: 2, exhaustLayout: '2into1' } as EngineSpec;
    const graph: ExhaustGraph = compileLayout(spec, pipe(), [makeSegment({})]);
    const d = newDuctId(graph);
    const n = newNodeId(graph);
    expect(graph.ducts.some((x) => x.id === d)).toBe(false);
    expect(graph.ducts.some((x) => x.from.kind === 'node' && x.from.node === n)).toBe(false);
  });
});

/**
 * Merging into a pipe that ended in open air.
 *
 * Unlike a T there is no existing pipe to hand the junction's outlet, so one is invented. That is not a
 * liberty: merging two pipes physically requires a pipe after the merge, and a collector is exactly that.
 */
describe('joining the end of a pipe', () => {
  const spec = { ...defaultConfig().engine, cylinders: 2, vAngle: 45, exhaustLayout: '2into2' } as EngineSpec;

  it('makes a junction and a collector wide enough for both feeds', () => {
    const graph = compileLayout(spec, [makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.04 })], []);
    const area = (d: number) => (Math.PI * d * d) / 4;
    const node = joinDuctEnd(graph, 'runner0', area(0.04))!;
    expect(node).toBeTruthy();

    const runner = graph.ducts.find((d) => d.id === 'runner0')!;
    expect(runner.to).toEqual({ kind: 'node', node });

    const onward = graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === node)!;
    expect(onward.to).toEqual({ kind: 'mouth' });
    // Area of the two feeds added, so the merge does not choke.
    expect(area(onward.segments[0]!.dIn)).toBeCloseTo(2 * area(0.04), 6);

    // Point the other runner at it and the graph stands up as a 2-into-1.
    graph.ducts.find((d) => d.id === 'runner1')!.to = { kind: 'node', node };
    expect(validateGraph(graph, 2)).toEqual([]);
  });

  it('declines a duct that already ends at a junction', () => {
    const merged = { ...defaultConfig().engine, cylinders: 2, exhaustLayout: '2into1' } as EngineSpec;
    const graph = compileLayout(merged, [makeSegment({ length: 0.4 })], [makeSegment({ length: 0.5 })]);
    expect(joinDuctEnd(graph, 'runner0')).toBeNull();
  });

  it('declines a duct with nothing drawn on it', () => {
    const graph = compileLayout(spec, [], []);
    expect(joinDuctEnd(graph, 'runner0')).toBeNull();
  });
});

/**
 * Branching *out of* the side of a pipe.
 *
 * The mirror of a T drawn into a pipe: the pipe is split where the route starts, and the new pipe leaves
 * the junction that makes. So the junction has one pipe in and two out — which the solver, the layout and
 * the joint geometry all have to take, since until now every junction was drawn with pipes going *in*.
 */
describe('branching from the side of a pipe', () => {
  const spec = { ...defaultConfig().engine, cylinders: 1 } as EngineSpec;
  const ports = (): ExhaustPort[] => {
    const mesh = new EngineMesh(spec, new THREE.Plane(new THREE.Vector3(0, 0, -1), 0.001));
    return [mesh.exhaustPort(0)];
  };

  function branched(): { graph: ExhaustGraph; node: string; branch: string } {
    const graph = compileLayout(spec, [makeSegment({ kind: 'pipe', length: 0.8, dIn: 0.042 })], []);
    const node = splitDuctAt(graph, 'runner0', 0.4)!;
    const branch = newDuctId(graph, 'drawn');
    graph.ducts.push({
      id: branch,
      segments: [makeSegment({ kind: 'pipe', length: 0.35, dIn: 0.042 })],
      from: { kind: 'node', node },
      to: { kind: 'mouth' },
      headingYaw: 1.1,
      headingPitch: 0,
    });
    return { graph, node, branch };
  }

  it('makes a graph the solver accepts, with two open ends', () => {
    const { graph } = branched();
    expect(validateGraph(graph, 1)).toEqual([]);
    expect(graph.ducts.filter((d) => d.to.kind === 'mouth')).toHaveLength(2);
  });

  it('starts the branch on the pipe, and leaves the pipe running straight through', () => {
    const { graph, node, branch } = branched();
    const placement = layoutGraph(ports(), graph);
    const up = placement.ducts.get('runner0')!;
    const swept = layoutPipe(graph.ducts[0]!.segments, up.origin, up.heading);
    const split = swept.joints[swept.joints.length - 1]!;
    const onward = graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === node && d.id !== branch)!;

    expect(placement.ducts.get(branch)!.origin.distanceTo(split)).toBeLessThan(1e-9);
    expect(placement.ducts.get(onward.id)!.origin.distanceTo(split)).toBeLessThan(1e-9);
    // The pipe carries on as it was going; the branch turns off it by what was drawn.
    const through = swept.jointDirections[swept.jointDirections.length - 1]!;
    expect(placement.ducts.get(onward.id)!.heading.angleTo(through)).toBeLessThan(1e-6);
    expect(placement.ducts.get(branch)!.heading.angleTo(through)).toBeGreaterThan(0.5);
  });

  it('fits a ball where the branch leaves the pipe', async () => {
    const { buildJointGeometry, hubShape } = await import('../src/scene/jointMesh.js');
    const { graph, node } = branched();
    const joint = layoutGraph(ports(), graph).joints.get(node)!;
    expect(joint.limbs).toHaveLength(3);
    const hub = hubShape(joint);
    // Everything meets at one point, so the fitting is a ball a little wider than the pipe.
    expect(hub.kind).toBe('ball');
    expect(hub.radius).toBeGreaterThan(0.021);
    expect(hub.radius).toBeLessThan(0.021 * 1.3);
    expect(buildJointGeometry(joint)).not.toBeNull();
  });

  it('still makes sound', async () => {
    const { EngineSim } = await import('../src/audio/worklet/engineSim.js');
    const { graph } = branched();
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, ...spec, throttle: 1 };
    cfg.graph = graph;
    const sim = new EngineSim(48000, cfg);
    sim.render(48000 / 2);
    const out = sim.render(48000 / 4);
    let peak = 0;
    for (const v of out) {
      expect(Number.isFinite(v)).toBe(true);
      peak = Math.max(peak, Math.abs(v));
    }
    expect(peak).toBeGreaterThan(1e-3);
  });
});

/**
 * Deleting what is selected.
 *
 * Every case has to leave a graph the solver accepts, since a delete is published straight to the audio.
 */
describe('deleting', () => {
  const twin = { ...defaultConfig().engine, cylinders: 2, vAngle: 45, exhaustLayout: '2into1' } as EngineSpec;
  const v8 = { ...defaultConfig().engine, cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' } as EngineSpec;
  const runner = () => [makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.042 })];
  const collector = () => [makeSegment({ kind: 'pipe', length: 0.5, dIn: 0.06 })];
  const portsOf = (spec: EngineSpec): ExhaustPort[] => {
    const mesh = new EngineMesh(spec, new THREE.Plane(new THREE.Vector3(0, 0, -1), 0.001));
    return Array.from({ length: mesh.bankCount }, (_, i) => mesh.exhaustPort(i));
  };

  it('never deletes a cylinder’s runner', async () => {
    const { removeDuct } = await import('../src/model/exhaustGraph.js');
    const graph = compileLayout(twin, runner(), collector());
    expect(removeDuct(graph, 'runner0')).toBe(false);
    expect(graph.ducts.some((d) => d.id === 'runner0')).toBe(true);
  });

  it('a collector deleted leaves its runners open', async () => {
    const { removeDuct } = await import('../src/model/exhaustGraph.js');
    const graph = compileLayout(twin, runner(), collector());
    expect(removeDuct(graph, 'collector0')).toBe(true);
    expect(graph.ducts.map((d) => d.id).sort()).toEqual(['runner0', 'runner1']);
    expect(graph.ducts.every((d) => d.to.kind === 'mouth')).toBe(true);
    expect(validateGraph(graph, 2)).toEqual([]);
  });

  it('a collector junction deleted: runners open, what came after it goes', async () => {
    const { removeJunction, compileCollectorLayout } = await import('../src/model/exhaustGraph.js');
    const graph = compileCollectorLayout(v8, runner(), collector());
    const joint = layoutGraph(portsOf(v8), graph).joints.get('merge0')!;
    const { throughPipe } = await import('../src/scene/jointMesh.js');
    // Four runners into a collector: nothing runs straight through.
    expect(throughPipe(joint)).toBeNull();
    removeJunction(graph, 'merge0', throughPipe(joint));
    expect(graph.ducts.some((d) => d.id === 'collector0')).toBe(false);
    // The other bank is untouched.
    expect(graph.ducts.some((d) => d.id === 'collector1')).toBe(true);
    expect(graph.ducts.filter((d) => d.from.kind === 'valve')).toHaveLength(8);
    expect(validateGraph(graph, 8)).toEqual([]);
  });

  /**
   * On a manifold, deleting a junction unhooks just that cylinder.
   *
   * The manifold runs straight through each junction along it, so it is rejoined, and only the stub of
   * the cylinder that joined there is left ending in air.
   */
  it('a manifold junction deleted: the manifold rejoins, one stub ends open', async () => {
    const { removeJunction } = await import('../src/model/exhaustGraph.js');
    const { throughPipe } = await import('../src/scene/jointMesh.js');
    const graph = compileLayout(v8, runner(), collector());
    const joint = layoutGraph(portsOf(v8), graph).joints.get('merge0-1')!;
    // The manifold widens here, so only the graph can say it runs through.
    const onward = graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === 'merge0-1')!;
    const stub = joint.limbs.map((l) => l.duct!).find((d) => d !== onward.id && d !== onward.continues)!;
    removeJunction(graph, 'merge0-1', throughPipe(joint));
    expect(graph.ducts.find((d) => d.id === stub)!.to).toEqual({ kind: 'mouth' });
    expect(graph.ducts.some((d) => d.id === 'collector0')).toBe(true);
    expect(validateGraph(graph, 8)).toEqual([]);
  });

  /** A tee deleted takes the branch away and rejoins the pipe it was teed onto. */
  it.each(['into', 'out of'])('a tee branching %s a pipe: deleting it rejoins the pipe', async (way) => {
    const { removeJunction } = await import('../src/model/exhaustGraph.js');
    const { throughPipe } = await import('../src/scene/jointMesh.js');
    const single = { ...defaultConfig().engine, cylinders: way === 'into' ? 2 : 1, vAngle: 45, exhaustLayout: '2into2' } as EngineSpec;
    const graph = compileLayout(single, [makeSegment({ kind: 'pipe', length: 0.8, dIn: 0.042 })], []);
    const before = graph.ducts.find((d) => d.id === 'runner0')!.segments.reduce((a, sg) => a + sg.length, 0);
    const node = splitDuctAt(graph, 'runner0', 0.4)!;
    if (way === 'into') {
      // Drawn onto the split point, as the editor would: straight at it, with the heading stored.
      const p = portsOf(single);
      const up = layoutGraph(p, graph).ducts.get('runner0')!;
      const swept = layoutPipe(graph.ducts[0]!.segments, up.origin, up.heading);
      const at = swept.joints[swept.joints.length - 1]!;
      const dir = at.clone().sub(p[1]!.position);
      const turn = headingOffsetTo(p[1]!.direction, dir.clone().normalize());
      const branch = graph.ducts.find((d) => d.id === 'runner1')!;
      branch.segments = [makeSegment({ kind: 'pipe', length: dir.length(), dIn: 0.042 })];
      branch.headingYaw = turn.yaw;
      branch.headingPitch = turn.pitch;
      branch.to = { kind: 'node', node };
    } else {
      graph.ducts.push({
        id: 'drawn0',
        segments: [makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.042 })],
        from: { kind: 'node', node },
        to: { kind: 'mouth' },
        headingYaw: 1.1,
        headingPitch: 0,
      });
    }
    const joint = layoutGraph(portsOf(single), graph).joints.get(node)!;
    const through = throughPipe(joint);
    expect(through?.[0]).toBe('runner0');

    removeJunction(graph, node, through);
    const pipe = graph.ducts.find((d) => d.id === 'runner0')!;
    expect(pipe.to).toEqual({ kind: 'mouth' });
    expect(pipe.segments.reduce((a, sg) => a + sg.length, 0)).toBeCloseTo(before, 9);
    expect(graph.ducts.some((d) => d.id === 'drawn0')).toBe(false);
    if (way === 'into') expect(graph.ducts.find((d) => d.id === 'runner1')!.to).toEqual({ kind: 'mouth' });
    expect(validateGraph(graph, single.cylinders)).toEqual([]);
  });

  it('deleting a branch leaving a pipe rejoins the pipe', async () => {
    const { removeDuct } = await import('../src/model/exhaustGraph.js');
    const single = { ...defaultConfig().engine, cylinders: 1 } as EngineSpec;
    const graph = compileLayout(single, [makeSegment({ kind: 'pipe', length: 0.8, dIn: 0.042 })], []);
    const node = splitDuctAt(graph, 'runner0', 0.4)!;
    graph.ducts.push({ id: 'drawn0', segments: runner(), from: { kind: 'node', node }, to: { kind: 'mouth' } });
    expect(removeDuct(graph, 'drawn0')).toBe(true);
    expect(graph.ducts.map((d) => d.id)).toEqual(['runner0']);
    expect(graph.ducts[0]!.segments.reduce((a, sg) => a + sg.length, 0)).toBeCloseTo(0.8, 9);
  });

  /** Deleting the middle of a two-stage merge must not leave a junction nothing flows into. */
  it('cascades through a tri-Y', async () => {
    const { removeDuct } = await import('../src/model/exhaustGraph.js');
    const four = { ...defaultConfig().engine, cylinders: 4, vAngle: 0 } as EngineSpec;
    const graph: ExhaustGraph = {
      ducts: [
        ...[0, 1, 2, 3].map((i) => ({
          id: `runner${i}`,
          segments: runner(),
          from: { kind: 'valve' as const, cylinder: i },
          to: { kind: 'node' as const, node: i % 2 ? 'pairB' : 'pairA' },
        })),
        { id: 'midA', segments: runner(), from: { kind: 'node', node: 'pairA' }, to: { kind: 'node', node: 'tail' } },
        { id: 'midB', segments: runner(), from: { kind: 'node', node: 'pairB' }, to: { kind: 'node', node: 'tail' } },
        { id: 'tailpipe', segments: collector(), from: { kind: 'node', node: 'tail' }, to: { kind: 'mouth' } },
      ],
    };
    expect(removeDuct(graph, 'midA')).toBe(true);
    // Runners 0 and 2 lost their way out and now end in air; midB and the tailpipe are one pipe again.
    expect(graph.ducts.find((d) => d.id === 'runner0')!.to).toEqual({ kind: 'mouth' });
    expect(graph.ducts.find((d) => d.id === 'midB')!.to).toEqual({ kind: 'mouth' });
    expect(graph.ducts.some((d) => d.id === 'tailpipe')).toBe(false);
    expect(validateGraph(graph, 4)).toEqual([]);
    void four;
  });
});

/**
 * Attaching a pipe to another leaves the one attached to where it was.
 *
 * It used to swing through 90 degrees: the junction took its direction from the average of its feeds'
 * *starting* directions, which for a pipe attached from the other side of the engine points nowhere
 * useful, and the collar search re-aimed the pipe being attached to as if it were a collector runner.
 */
describe('attaching leaves the pipe attached to alone', () => {
  const spec = { ...defaultConfig().engine, cylinders: 2, vAngle: 45, exhaustLayout: '2into2' } as EngineSpec;
  const ports = (): ExhaustPort[] => {
    const mesh = new EngineMesh(spec, new THREE.Plane(new THREE.Vector3(0, 0, -1), 0.001));
    return [0, 1].map((i) => mesh.exhaustPort(i));
  };
  const pipe = () => [
    makeSegment({ kind: 'pipe', length: 0.5, dIn: 0.042 }),
    makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.042, yaw: 0.4 }),
  ];
  /** The direction runner0 is going `x` along it, before anything is attached. */
  function directionAt(x: number): THREE.Vector3 {
    const graph = compileLayout(spec, pipe(), []);
    const place = layoutGraph(ports(), graph).ducts.get('runner0')!;
    const swept = layoutPipe(graph.ducts[0]!.segments, place.origin, place.heading);
    return swept.stations.find((st) => st.x >= x - 1e-9)!.direction.clone();
  }

  it.each([0.3, 0.65])('into its side, cut at %s m: the pipe carries straight on', (x) => {
    const graph = compileLayout(spec, pipe(), []);
    const node = splitDuctAt(graph, 'runner0', x)!;
    // The other cylinder's runner, unedited, attached into it — which used to set off the collar search.
    graph.ducts.find((d) => d.id === 'runner1')!.to = { kind: 'node', node };
    const placement = layoutGraph(ports(), graph);
    const onward = graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === node)!;
    expect(placement.ducts.get(onward.id)!.heading.angleTo(directionAt(x + 0.01))).toBeLessThan(1e-6);
    // And the pipe it was cut from did not move.
    const before = layoutGraph(ports(), compileLayout(spec, pipe(), [])).ducts.get('runner0')!.heading;
    expect(placement.ducts.get('runner0')!.heading.angleTo(before)).toBeLessThan(1e-9);
  });

  it('onto its end: the new pipe carries on the way it was going', () => {
    const graph = compileLayout(spec, pipe(), []);
    const node = joinDuctEnd(graph, 'runner0', Math.PI * 0.021 ** 2)!;
    graph.ducts.find((d) => d.id === 'runner1')!.to = { kind: 'node', node };
    const placement = layoutGraph(ports(), graph);
    const onward = graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === node)!;
    expect(placement.ducts.get(onward.id)!.heading.angleTo(directionAt(0.8))).toBeLessThan(1e-6);
  });

  it('cut inside a turned segment, the second half does not turn again', () => {
    const halves = splitSegments(pipe(), 0.65)!;
    expect(halves[0][1]!.yaw).toBeCloseTo(0.4, 12);
    expect(halves[1][0]!.yaw).toBe(0);
    expect(halves[1][0]!.pitch).toBe(0);
  });
});

/**
 * No deletion leaves a giant fitting, or moves a pipe it did not touch.
 *
 * Deleting a segment of a pipe that ran into a junction left it short of the junction, and the fitting
 * grew to bridge the gap — up to 36 cm. And pipes the layout aims, a V-twin's runners and an 8-into-1's
 * downpipes, were re-aimed after every deletion, swinging pipes nobody touched by up to two metres. Every
 * single deletion on every preset is tried here, doing what the app does: fix every pipe where it stands,
 * delete, take the edited pipe off its junction if it no longer reaches, and tidy with the old directions.
 */
describe('deleting keeps the exhaust in one piece', async () => {
  const { ENGINE_PRESETS } = await import('../src/model/spec.js');
  const { ductDirections, freezeHeadings, pipesMeetAt } = await import('../src/scene/exhaustLayout.js');
  const { hubShape, throughPipe } = await import('../src/scene/jointMesh.js');
  const { disconnectEnd, nodeOrder, removeDuct, removeJunction } = await import('../src/model/exhaustGraph.js');
  const clone = (g: ExhaustGraph): ExhaustGraph => JSON.parse(JSON.stringify(g));

  const cases = ENGINE_PRESETS.filter((p) => p.collector).flatMap((p) =>
    [undefined, 'merged' as const].map((layout) => [`${p.name}${layout ? ' as 8/2-into-1' : ''}`, p, layout] as const),
  );

  it.each(cases)('%s', (_name, preset, layout) => {
    const spec = { ...defaultConfig().engine, ...preset.engine, ...(layout ? { exhaustLayout: layout } : {}) } as EngineSpec;
    const mesh = new EngineMesh(spec, new THREE.Plane(new THREE.Vector3(0, 0, -1), 0.001));
    const ports = Array.from({ length: mesh.bankCount }, (_, i) => mesh.exhaustPort(i));
    const base = compileLayout(spec, preset.pipe(), preset.collector!());
    const before = layoutGraph(ports, base);
    freezeHeadings(base, before, ports);
    const dirs = ductDirections(base, before);
    const points = (g: ExhaustGraph, pl: ReturnType<typeof layoutGraph>) =>
      g.ducts.map((d) => {
        const place = pl.ducts.get(d.id)!;
        return [d.id, layoutPipe(d.segments, place.origin, place.heading).stations.map((s) => s.position)] as const;
      });
    const was = points(base, before).flatMap(([, pts]) => pts);

    const check = (label: string, g: ExhaustGraph, touched: string) => {
      expect(validateGraph(g, spec.cylinders), label).toEqual([]);
      const after = layoutGraph(ports, g);
      for (const [node, joint] of after.joints) {
        const hub = hubShape(joint);
        expect(hub.kind, `${label}: ${node}`).toBe('ball');
        expect(hub.radius, `${label}: ${node}`).toBeLessThan(0.06);
      }
      for (const [id, pts] of points(g, after)) {
        if (id === touched) continue;
        for (const q of pts) {
          const off = Math.min(...was.map((o) => o.distanceTo(q)));
          expect(off, `${label}: ${id} moved`).toBeLessThan(0.002);
        }
      }
    };

    for (const node of nodeOrder(base)) {
      const g = clone(base);
      const joint = before.joints.get(node);
      removeJunction(g, node, joint ? throughPipe(joint) : null, dirs);
      check(`delete junction ${node}`, g, '');
    }
    for (const d of base.ducts) {
      d.segments.forEach((_, i) => {
        const g = clone(base);
        const duct = g.ducts.find((x) => x.id === d.id)!;
        duct.segments.splice(i, 1);
        if (duct.segments.length === 0 && duct.from.kind !== 'valve') removeDuct(g, duct.id, dirs);
        else if (duct.to.kind === 'node' && !pipesMeetAt(g, layoutGraph(ports, g), duct.to.node)) {
          disconnectEnd(g, duct.id, dirs);
        }
        check(`delete ${d.id} segment ${i + 1}`, g, d.id);
      });
    }
  });
});
