/**
 * Turning clicks into pipe segments.
 *
 * The thing that has to be true and is not obvious: a fitted segment's *swept* end lands where the click
 * was. `PipeSegment` does not describe a straight line — `layoutPipe` turns the heading a little at every
 * station, so any segment with a bend is an arc and "point it at the target" misses by the whole of the
 * bend. Runners aimed at a collector by their inlet direction curve away from it for the same reason.
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
  detachDuct,
  fitSegment,
  headingOffsetTo,
  nearestSnap,
  quantiseLength,
  quantiseTurn,
  routeTip,
  snapToEngine,
  bendSegment,
  bendWhole,
  reshapeBendKeepingLength,
  slideBend,
  reshapeBend,
  splitDuct,
} from '../src/scene/drawing.js';
import { layoutGraph, type ExhaustPort } from '../src/scene/exhaustLayout.js';
import { bendRadius, layoutPipe } from '../src/scene/PipeMesh.js';
import { bendShape } from '../src/model/geometry.js';
import {
  attachToLooseStart,
  compileLayout,
  disconnectEnd,
  joinDuctEnd,
  newDuctId,
  newNodeId,
  placeLoosePipe,
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

describe('locking to the engine', () => {
  const viewport = { width: 1200, height: 800 };
  const camera = new THREE.PerspectiveCamera(40, viewport.width / viewport.height, 0.01, 50);
  camera.position.set(0.85, 0.55, 1.15);
  camera.lookAt(0, 0, 0);
  camera.updateMatrixWorld();
  const tip = new THREE.Vector3(0.1, 0.05, 0.1);
  const ahead = new THREE.Vector3(1, 0, 0);

  /** Aim the pointer at a world point, nudged by some pixels, and snap. */
  function snapAt(world: THREE.Vector3, nudge = new THREE.Vector2()) {
    const ndc = world.clone().project(camera);
    const pointer = new THREE.Vector2(
      ndc.x + (nudge.x * 2) / viewport.width,
      ndc.y + (nudge.y * 2) / viewport.height,
    );
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(pointer, camera);
    return snapToEngine(tip, ahead, raycaster.ray, pointer, camera, viewport, 0.025);
  }

  it.each([
    ['up', new THREE.Vector3(0, 1, 0)],
    ['along the crank', new THREE.Vector3(0, 0, -1)],
    ['down and across', new THREE.Vector3(1, -1, 0).normalize()],
  ] as Array<[string, THREE.Vector3]>)('locks near %s onto it, at the length pointed to', (name, dir) => {
    const snapped = snapAt(tip.clone().addScaledVector(dir, 0.3), new THREE.Vector2(6, -4))!;
    expect(snapped.name).toBe(name);
    expect(snapped.dir.angleTo(dir)).toBeLessThan(1e-9);
    expect(snapped.point.distanceTo(tip)).toBeCloseTo(0.3, 6);
  });

  it('offers straight on, for a pipe leaving at an angle', () => {
    const angled = new THREE.Vector3(1, 0.3, 0.2).normalize();
    const ndc = tip.clone().addScaledVector(angled, 0.4).project(camera);
    const pointer = new THREE.Vector2(ndc.x, ndc.y);
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(pointer, camera);
    const snapped = snapToEngine(tip, angled, raycaster.ray, pointer, camera, viewport, 0.025)!;
    expect(snapped.name).toBe('straight on');
    expect(snapped.point.distanceTo(tip)).toBeCloseTo(0.4, 6);
  });

  it('never folds straight back', () => {
    const snapped = snapAt(tip.clone().addScaledVector(ahead, -0.3))!;
    expect(snapped.dir.dot(ahead)).toBeGreaterThan(-0.9);
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
    const mesh = new EngineMesh(spec);
    return Array.from({ length: mesh.bankCount }, (_, i) => mesh.exhaustPort(i));
  };

  /**
   * Every port is offered, occupied or not.
   *
   * Leaving occupied ports out, on the reasoning that a cylinder may only have one pipe, would make draw
   * mode useless: a compiled engine gives every cylinder a runner, so no port would ever be clickable and a
   * route could only start from a junction. Drawing from an occupied port replaces what is there, and
   * `occupied` is what says so.
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
 * Only the junction is made: the pipe carrying the merged flow on is drawn from it, not invented. Until it
 * is, the pipes meeting there end in open air, which is what the solver is given.
 */
describe('joining the end of a pipe', () => {
  const spec = { ...defaultConfig().engine, cylinders: 2, vAngle: 45, exhaustLayout: '2into2' } as EngineSpec;

  it('makes a junction and nothing after it, fixed where the pipe ends', async () => {
    const { junctionAt, solverGraph } = await import('../src/model/exhaustGraph.js');
    const graph = compileLayout(spec, [makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.04 })], []);
    const count = graph.ducts.length;
    const at = { position: [0.3, 0.2, 0.1] as [number, number, number], axis: [1, 0, 0] as [number, number, number] };
    const node = joinDuctEnd(graph, 'runner0', at)!;
    expect(node).toBeTruthy();
    expect(graph.ducts).toHaveLength(count);
    expect(graph.ducts.find((d) => d.id === 'runner0')!.to).toEqual({ kind: 'node', node });
    expect(junctionAt(graph, node)!.position).toEqual(at.position);

    // Point the other runner at it too: a merge waiting for the pipe after it, which the app accepts.
    graph.ducts.find((d) => d.id === 'runner1')!.to = { kind: 'node', node };
    expect(validateGraph(graph, 2)).toEqual([]);
    // Until that pipe is drawn, both end in open air as far as the solver is concerned.
    expect(solverGraph(graph).ducts.every((d) => d.to.kind === 'mouth')).toBe(true);
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
 * the junction that makes. So the junction has one pipe in and two out, where a merge has several in and
 * one out — and the solver, the layout and the joint geometry all have to take both.
 */
describe('branching from the side of a pipe', () => {
  const spec = { ...defaultConfig().engine, cylinders: 1 } as EngineSpec;
  const ports = (): ExhaustPort[] => {
    const mesh = new EngineMesh(spec);
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
    const { Sim } = await import('../src/audio/worklet/sim.js');
    const { graph } = branched();
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, ...spec, throttle: 1 };
    cfg.graph = graph;
    const sim = new Sim(48000, cfg);
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
    const mesh = new EngineMesh(spec);
    return Array.from({ length: mesh.bankCount }, (_, i) => mesh.exhaustPort(i));
  };

  it('never deletes a cylinder’s runner', async () => {
    const { removeDuct } = await import('../src/model/exhaustGraph.js');
    const graph = compileLayout(twin, runner(), collector());
    expect(removeDuct(graph, 'runner0')).toBe(false);
    expect(graph.ducts.some((d) => d.id === 'runner0')).toBe(true);
  });

  it('a collector deleted leaves its runners merging, open to the air until another is drawn', async () => {
    const { removeDuct, solverGraph } = await import('../src/model/exhaustGraph.js');
    const graph = compileLayout(twin, runner(), collector());
    expect(removeDuct(graph, 'collector0')).toBe(true);
    expect(graph.ducts.map((d) => d.id).sort()).toEqual(['runner0', 'runner1']);
    // Still meeting at their junction, a merge waiting for the pipe after it.
    expect(graph.ducts.every((d) => d.to.kind === 'node')).toBe(true);
    expect(validateGraph(graph, 2)).toEqual([]);
    expect(solverGraph(graph).ducts.every((d) => d.to.kind === 'mouth')).toBe(true);
  });

  it('pipes only bent in to meet the pipe deleted come off, rather than meeting where it was', async () => {
    const { removeDuct, junctionAt, nodeOrder } = await import('../src/model/exhaustGraph.js');
    const bend = () => makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.042 });
    const graph: ExhaustGraph = {
      ducts: [
        ...[0, 1].map((i) => ({
          id: `runner${i}`,
          segments: [...runner(), bend()],
          from: { kind: 'valve' as const, cylinder: i },
          to: { kind: 'node' as const, node: 'join1' },
          fitted: true as const,
        })),
        { id: 'loose1', segments: collector(), from: { kind: 'node', node: 'join1' }, to: { kind: 'mouth' } },
      ],
      junctions: [{ node: 'join1', position: [0.3, 0, 0], axis: [1, 0, 0] }],
    };
    expect(removeDuct(graph, 'loose1')).toBe(true);
    for (const d of graph.ducts) {
      expect(d.to).toEqual({ kind: 'mouth' });
      expect(d.fitted).toBeUndefined();
      expect(d.segments).toHaveLength(runner().length);
    }
    expect(nodeOrder(graph)).toEqual([]);
    expect(junctionAt(graph, 'join1')).toBeUndefined();
    expect(validateGraph(graph, 2)).toEqual([]);
  });

  it('a collector junction is only deleted once the collector after it is: then its runners end open', async () => {
    const { removeDuct, removeJunction, compileCollectorLayout } = await import('../src/model/exhaustGraph.js');
    const graph = compileCollectorLayout(v8, runner(), collector());
    const joint = layoutGraph(portsOf(v8), graph).joints.get('merge0')!;
    const { throughPipe } = await import('../src/scene/jointMesh.js');
    // Four runners into a collector: nothing runs straight through, so the collector leaving it holds it.
    expect(throughPipe(joint)).toBeNull();
    const before = JSON.stringify(graph);
    expect(removeJunction(graph, 'merge0', throughPipe(joint))).toBe(false);
    expect(JSON.stringify(graph)).toBe(before);
    expect(removeDuct(graph, 'collector0')).toBe(true);
    expect(removeJunction(graph, 'merge0', null)).toBe(true);
    expect(graph.ducts.some((d) => d.id === 'collector0')).toBe(false);
    for (const d of graph.ducts) if (d.from.kind === 'valve' && d.to.kind === 'node') expect(d.to.node).not.toBe('merge0');
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

  /** A tee deleted rejoins the pipe it was teed onto, once no branch leaves it. */
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

    // A branch leaving it holds it, until the branch is deleted.
    if (way === 'out of') {
      expect(removeJunction(graph, node, through)).toBe(false);
      graph.ducts = graph.ducts.filter((d) => d.id !== 'drawn0');
    }
    expect(removeJunction(graph, node, through)).toBe(true);
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
  it('deletes a pipe in the middle of a tri-Y, what carries on from it still fed', async () => {
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
    // The tailpipe carries on from midA and midB: midA goes, and with only midB left feeding it, the two
    // rejoin into one pipe that still reaches the air.
    const before = JSON.stringify(graph);
    const copy = JSON.parse(before) as ExhaustGraph;
    expect(removeDuct(copy, 'midA')).toBe(true);
    const midB = copy.ducts.find((d) => d.id === 'midB')!;
    expect(midB.to).toEqual({ kind: 'mouth' });
    expect(midB.segments).toHaveLength(runner().length + collector().length);
    expect(validateGraph(copy, 4)).toEqual([]);
    // Nor is the junction midA leaves, which would delete midA.
    const { removeJunction } = await import('../src/model/exhaustGraph.js');
    expect(removeJunction(graph, 'pairA')).toBe(false);
    expect(JSON.stringify(graph)).toBe(before);
    // The tailpipe has nothing after it, so it goes, and the pipes that fed it still meet, open to the air.
    expect(removeDuct(graph, 'tailpipe')).toBe(true);
    expect(graph.ducts.find((d) => d.id === 'midA')!.to).toEqual({ kind: 'node', node: 'tail' });
    expect(graph.ducts.find((d) => d.id === 'midB')!.to).toEqual({ kind: 'node', node: 'tail' });
    expect(validateGraph(graph, 4)).toEqual([]);
    void four;
  });
});

/**
 * Attaching a pipe to another leaves the one attached to where it was.
 *
 * A junction that took its direction from the average of its feeds' *starting* directions would swing it
 * through 90 degrees, since for a pipe attached from the other side of the engine that average points
 * nowhere useful; and a collar search would re-aim the pipe being attached to as if it were a collector
 * runner.
 */
describe('attaching leaves the pipe attached to alone', () => {
  const spec = { ...defaultConfig().engine, cylinders: 2, vAngle: 45, exhaustLayout: '2into2' } as EngineSpec;
  const ports = (): ExhaustPort[] => {
    const mesh = new EngineMesh(spec);
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
    // The other cylinder's runner, unedited, attached into it — which is what would set off a collar search.
    graph.ducts.find((d) => d.id === 'runner1')!.to = { kind: 'node', node };
    const placement = layoutGraph(ports(), graph);
    const onward = graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === node)!;
    expect(placement.ducts.get(onward.id)!.heading.angleTo(directionAt(x + 0.01))).toBeLessThan(1e-6);
    // And the pipe it was cut from did not move.
    const before = layoutGraph(ports(), compileLayout(spec, pipe(), [])).ducts.get('runner0')!.heading;
    expect(placement.ducts.get('runner0')!.heading.angleTo(before)).toBeLessThan(1e-9);
  });

  it('onto its end: a pipe drawn on from the junction carries on the way it was going', () => {
    const graph = compileLayout(spec, pipe(), []);
    const end = layoutPipe(graph.ducts[0]!.segments, ports()[0]!.position, ports()[0]!.direction);
    const at = end.joints.at(-1)!;
    const dir = end.jointDirections.at(-1)!;
    const node = joinDuctEnd(graph, 'runner0', { position: [at.x, at.y, at.z], axis: [dir.x, dir.y, dir.z] })!;
    graph.ducts.find((d) => d.id === 'runner1')!.to = { kind: 'node', node };
    graph.ducts.push({ id: 'onward', segments: [makeSegment({ length: 0.3 })], from: { kind: 'node', node }, to: { kind: 'mouth' } });
    const placement = layoutGraph(ports(), graph);
    expect(placement.ducts.get('onward')!.heading.angleTo(directionAt(0.8))).toBeLessThan(1e-6);
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
 * Deleting a segment of a pipe that runs into a junction leaves it short of the junction, and a fitting
 * grown to bridge the gap could reach tens of centimetres. And pipes the layout aims, a V-twin's runners
 * and an 8-into-1's downpipes, would be re-aimed after every deletion, swinging pipes nobody touched by
 * metres. Every single deletion on every preset is tried here, doing what the app does: fix every pipe
 * where it stands, delete, take the edited pipe off its junction if it no longer reaches, and tidy with
 * the directions from before the delete.
 */
describe('deleting keeps the exhaust in one piece', async () => {
  const { ENGINE_PRESETS } = await import('../src/model/spec.js');
  const { ductDirections, freezeHeadings, pipesMeetAt } = await import('../src/scene/exhaustLayout.js');
  const { hubShape, throughPipe } = await import('../src/scene/jointMesh.js');
  const { childDucts, disconnectEnd, nodeOrder, removeDuct, removeJunction } = await import('../src/model/exhaustGraph.js');
  const clone = (g: ExhaustGraph): ExhaustGraph => JSON.parse(JSON.stringify(g));

  const cases = ENGINE_PRESETS.filter((p) => p.collector).flatMap((p) =>
    [undefined, 'merged' as const].map((layout) => [`${p.name}${layout ? ' as 8/2-into-1' : ''}`, p, layout] as const),
  );

  it.each(cases)('%s', (_name, preset, layout) => {
    const spec = { ...defaultConfig().engine, ...preset.engine, ...(layout ? { exhaustLayout: layout } : {}) } as EngineSpec;
    const mesh = new EngineMesh(spec);
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
      // Junctions are not drawn, so what matters is that the pipes still meet and the fitting a selected one
      // shows does not balloon.
      for (const [node, joint] of after.joints) {
        expect(hubShape(joint).radius, `${label}: ${node}`).toBeLessThan(0.06);
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
      // Refused where a pipe it would delete has others carrying on from it: then nothing changes.
      if (!removeJunction(g, node, joint ? throughPipe(joint) : null, dirs)) {
        expect(g, `delete junction ${node}`).toEqual(base);
        continue;
      }
      check(`delete junction ${node}`, g, '');
    }
    for (const d of base.ducts) {
      d.segments.forEach((_, i) => {
        const g = clone(base);
        const duct = g.ducts.find((x) => x.id === d.id)!;
        // Only a pipe's last segment is deleted, and not while others carry on from the pipe's end.
        if (i < duct.segments.length - 1) return;
        if (duct.segments.length === 1 && childDucts(g, duct).length > 0) return;
        duct.segments.splice(i, 1);
        if (duct.segments.length === 0 && duct.from.kind !== 'valve') removeDuct(g, duct.id, dirs);
        else if (duct.to.kind === 'node' && !pipesMeetAt(g, layoutGraph(ports, g), duct.to.node)) {
          // The only pipe into a junction stays on it rather than leave the pipes after it unfed.
          if (!disconnectEnd(g, duct.id, dirs)) return;
        }
        check(`delete ${d.id} segment ${i + 1}`, g, d.id);
      });
    }
  });
});

/** Deleting a segment in the middle of a pipe leaves the segments after it loose, where they lie. */
describe('deleting from the middle of a pipe', () => {
  const spec = { ...defaultConfig().engine, cylinders: 2, vAngle: 45, exhaustLayout: '2into2' } as EngineSpec;
  const ports = (): ExhaustPort[] => {
    const mesh = new EngineMesh(spec);
    return [0, 1].map((i) => mesh.exhaustPort(i));
  };
  const pipe = () => [
    makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.042 }),
    makeSegment({ kind: 'pipe', length: 0.2, dIn: 0.042, yaw: 0.4 }),
    makeSegment({ kind: 'pipe', length: 0.25, dIn: 0.042, pitch: -0.3 }),
  ];

  it('keeps the segments after it where they were, as a loose pipe that takes over the far end', () => {
    const graph = compileLayout(spec, pipe(), []);
    const before = layoutGraph(ports(), graph);
    const place = before.ducts.get('runner0')!;
    const swept = layoutPipe(graph.ducts[0]!.segments, place.origin, place.heading);
    const tail = graph.ducts[0]!.segments[2]!.id;
    const loose = splitDuct(graph, 'runner0', 1, place)!;

    expect(graph.ducts.find((d) => d.id === 'runner0')!.segments).toHaveLength(1);
    const rest = graph.ducts.find((d) => d.id === loose)!;
    expect(rest.from.kind).toBe('free');
    expect(rest.segments.map((s) => s.id)).toEqual([tail]);
    expect(rest.to.kind).toBe('mouth');
    expect(validateGraph(graph, 2)).toEqual([]);

    const after = layoutGraph(ports(), graph).ducts.get(loose)!;
    const moved = layoutPipe(rest.segments, after.origin, after.heading);
    expect(moved.joints.at(-1)!.distanceTo(swept.joints.at(-1)!)).toBeLessThan(1e-9);
    expect(moved.jointDirections.at(-1)!.angleTo(swept.jointDirections.at(-1)!)).toBeLessThan(1e-9);
  });

  it('does nothing to the last segment, which is deleted from the end as usual', () => {
    const graph = compileLayout(spec, pipe(), []);
    const place = layoutGraph(ports(), graph).ducts.get('runner0')!;
    expect(splitDuct(graph, 'runner0', 2, place)).toBeNull();
    expect(graph.ducts[0]!.segments).toHaveLength(3);
  });
});

/** Taking a branch's fitted bend off the side of a straight pipe rejoins the pipe it split. */
describe('detaching a branch from the side of a pipe', () => {
  const spec = { ...defaultConfig().engine, cylinders: 2, vAngle: 45, exhaustLayout: '2into2' } as EngineSpec;
  const ports = (): ExhaustPort[] => {
    const mesh = new EngineMesh(spec);
    return [0, 1].map((i) => mesh.exhaustPort(i));
  };
  const pipe = () => [
    makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.042 }),
    makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.042 }),
  ];

  it.each([false, true])('merges the straight pipe back into one, fixed in place: %s', (pinned) => {
    const graph = compileLayout(spec, pipe(), []);
    const before = layoutGraph(ports(), graph).ducts.get('runner0')!;
    const swept = layoutPipe(graph.ducts[0]!.segments, before.origin, before.heading);
    const node = splitDuctAt(graph, 'runner0', 0.5)!;
    if (pinned) graph.junctions = [{ node, position: [0, 0, 0], axis: [1, 0, 0] }];
    const branch = graph.ducts.find((d) => d.id === 'runner1')!;
    branch.segments.push(makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.042 }));
    branch.to = { kind: 'node', node };
    branch.fitted = true;
    const count = branch.segments.length;

    expect(disconnectEnd(graph, 'runner1')).toBe(true);
    const runner = graph.ducts.find((d) => d.id === 'runner0')!;
    expect(runner.to.kind).toBe('mouth');
    expect(runner.fitted).toBeUndefined();
    expect(graph.ducts).toHaveLength(2);
    expect(graph.junctions).toBeUndefined();
    // The segment the branch cut in two is one again.
    expect(runner.segments.map((s) => s.length)).toEqual(pipe().map((s) => s.length));
    // The bend it was fitted in goes with it.
    expect(branch.segments).toHaveLength(count - 1);
    expect(branch.to.kind).toBe('mouth');
    // And the pipe is where it was.
    const place = layoutGraph(ports(), graph).ducts.get('runner0')!;
    const after = layoutPipe(runner.segments, place.origin, place.heading);
    expect(after.joints.at(-1)!.distanceTo(swept.joints.at(-1)!)).toBeLessThan(1e-9);
  });
});

/** A pipe drawn from a port into a placed pipe comes off it again, and the placed pipe is loose again. */
describe('detaching a pipe from a placed pipe', () => {
  const spec = { ...defaultConfig().engine, cylinders: 2, vAngle: 45, exhaustLayout: '2into2' } as EngineSpec;
  const ports = (): ExhaustPort[] => {
    const mesh = new EngineMesh(spec);
    return [0, 1].map((i) => mesh.exhaustPort(i));
  };

  it('takes off the bend, and leaves the placed pipe loose where it lies', () => {
    const graph = compileLayout(spec, [makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.042 })], []);
    const loose = placeLoosePipe(graph, [0.2, -0.3, 0.4], 0.042, 0.3);
    const before = layoutGraph(ports(), graph).ducts.get(loose)!;
    const runner = graph.ducts.find((d) => d.id === 'runner0')!;
    const drawn = runner.segments.length;
    expect(attachToLooseStart(graph, 'runner0', loose, [1, 0, 0])).not.toBeNull();
    runner.segments.push(makeSegment({ kind: 'pipe', length: 0.2, dIn: 0.042 }));
    runner.fitted = true;

    expect(detachDuct(graph, 'runner0', layoutGraph(ports(), graph))).toBe(true);
    expect(runner.to.kind).toBe('mouth');
    expect(runner.fitted).toBeUndefined();
    expect(runner.segments).toHaveLength(drawn);
    const placed = graph.ducts.find((d) => d.id === loose)!;
    expect(placed.from.kind).toBe('free');
    expect(graph.junctions).toBeUndefined();
    expect(validateGraph(graph, 2)).toEqual([]);
    const after = layoutGraph(ports(), graph).ducts.get(loose)!;
    expect(after.origin.distanceTo(before.origin)).toBeLessThan(1e-9);
    expect(after.heading.angleTo(before.heading)).toBeLessThan(1e-9);
  });
});

/** A bend drawn as a tube bender makes one: a turn, round a radius. */
describe('a drawn bend', () => {
  const entry = new THREE.Vector3(0.1, 0.2, -0.05);
  const d0 = new THREE.Vector3(1, 0, 0);

  it.each([
    [new THREE.Vector3(0, -1, 0), 0.08],
    [new THREE.Vector3(1, -1, 0).normalize(), 0.12],
    [new THREE.Vector3(0, 0, 1), 0.1],
    [new THREE.Vector3(-1, 1, 1).normalize(), 0.09],
  ])('turns to head %o round the radius asked for, ending where that arc does', (d1, radius) => {
    const bend = bendSegment(entry, d0, d1, radius, { dIn: 0.042, dOut: 0.042 })!;
    const swept = layoutPipe([bend], entry, d0);
    expect(swept.jointDirections.at(-1)!.angleTo(d1)).toBeLessThan(1e-6);
    const angle = d0.angleTo(d1);
    const across = d1.clone().addScaledVector(d0, -Math.cos(angle)).normalize();
    const end = entry.clone().addScaledVector(d0, radius * Math.sin(angle)).addScaledVector(across, radius * (1 - Math.cos(angle)));
    expect(swept.joints.at(-1)!.distanceTo(end)).toBeLessThan(1e-9);
    // Read back off the segment, as the panel shows it.
    const shape = bendShape(bend)!;
    expect(shape.angle).toBeCloseTo(angle, 9);
    expect(shape.radius).toBeCloseTo(radius, 9);
    // Its tightest turn near the radius it was drawn round: a cubic is not quite an arc.
    // An arc: it turns round that radius all the way, as nearly as sampled points measure it, and is as
    // long as the arc.
    const r = bendRadius(entry, d0, end, d1, bend.curve!.handle);
    expect(r / radius).toBeGreaterThan(0.93);
    expect(r / radius).toBeLessThan(1.07);
    expect(bend.length / (radius * angle)).toBeCloseTo(1, 2);
  });

  it('is no bend at all the way the pipe already goes, or straight back', () => {
    expect(bendSegment(entry, d0, d0, 0.1)).toBeNull();
    expect(bendSegment(entry, d0, d0.clone().negate(), 0.1)).toBeNull();
  });

  it('reshapes to a new turn and radius in the plane it turns in', () => {
    const bend = bendSegment(entry, d0, new THREE.Vector3(0, -1, 0), 0.1, { dIn: 0.042, dOut: 0.042 })!;
    const next = reshapeBend(bend, Math.PI / 4, 0.2)!;
    expect(next.id).toBe(bend.id);
    expect(next.dIn).toBe(0.042);
    const shape = bendShape(next)!;
    expect(shape.angle).toBeCloseTo(Math.PI / 4, 9);
    expect(shape.radius).toBeCloseTo(0.2, 9);
    // Still turning down, the way it did.
    const dir = layoutPipe([next], entry, d0).jointDirections.at(-1)!;
    expect(dir.y).toBeLessThan(0);
    expect(Math.abs(dir.z)).toBeLessThan(1e-9);
  });
});

/** Bending a whole straight into one arc, as a tube is bent: it keeps its length. */
describe('bending a straight', () => {
  const seg = makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.04, dOut: 0.05, yaw: 0.2 });
  const dir = new THREE.Vector3(1, 0, 0);
  const up = new THREE.Vector3(0, 0, 1);

  it('curves all of it about the axis given, as long as it was, its corner, id and bores kept', () => {
    const { segment, clamped, radius } = bendWhole(seg, dir, up, Math.PI / 2, 0.06);
    expect(clamped).toBe(false);
    expect(segment.length).toBeCloseTo(0.3, 6);
    expect(radius).toBeCloseTo(0.3 / (Math.PI / 2), 3);
    expect(segment.id).toBe(seg.id);
    expect(segment.yaw).toBe(0.2);
    expect(segment.dIn).toBe(0.04);
    expect(segment.dOut).toBe(0.05);
    // Heading off the way the turn says, in the plane square to the axis.
    const swept = layoutPipe([{ ...segment, yaw: 0 }], new THREE.Vector3(), dir);
    expect(swept.jointDirections.at(-1)!.angleTo(new THREE.Vector3(0, 1, 0))).toBeLessThan(1e-6);
    expect(Math.abs(swept.joints.at(-1)!.z)).toBeLessThan(1e-9);
  });

  it('turns no tighter than it may, stopping where it would', () => {
    const short = makeSegment({ kind: 'pipe', length: 0.05, dIn: 0.04 });
    const { segment, clamped, angle, radius } = bendWhole(short, dir, up, Math.PI / 2, 0.06);
    expect(clamped).toBe(true);
    expect(radius / 0.06).toBeGreaterThan(0.99);
    expect(Math.abs(angle)).toBeLessThan(Math.PI / 2);
    expect(segment.length).toBeCloseTo(0.05, 6);
  });

  it('reshaped on its own keeps its length: a new angle sets its radius, a new radius its angle', () => {
    const segments = [bendWhole(seg, dir, up, Math.PI / 2, 0.06).segment];
    expect(reshapeBendKeepingLength(segments, 0, Math.PI / 3, 0, 'angle')).toBe(true);
    expect(bendShape(segments[0]!)!.angle).toBeCloseTo(Math.PI / 3, 9);
    expect(segments[0]!.length / 0.3).toBeCloseTo(1, 2);
    expect(reshapeBendKeepingLength(segments, 0, 0, 0.5, 'radius')).toBe(true);
    expect(bendShape(segments[0]!)!.radius).toBeCloseTo(0.5, 9);
    expect(segments[0]!.length / 0.3).toBeCloseTo(1, 2);
  });

  it('between two straights, reshaped or slid along, keeps its length, the straights giving way', () => {
    const bend = bendWhole(makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.04 }), dir, up, Math.PI / 2, 0.06).segment;
    const segments = [makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.04 }), bend, makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.04 })];
    const total = () => segments.reduce((a, s) => a + s.length, 0);
    const was = total();
    expect(reshapeBendKeepingLength(segments, 1, Math.PI / 3, 0.08, 'radius')).toBe(true);
    expect(total()).toBeCloseTo(was, 9);
    expect(bendShape(segments[1]!)!.radius).toBeCloseTo(0.08, 9);
    expect(slideBend(segments, 1, 0.05)).toBe(true);
    expect(segments[0]!.length).toBeCloseTo(0.05, 12);
    expect(total()).toBeCloseTo(was, 9);
  });
});

/**
 * Bending a bend again, as the bend tool does: from the straight it was, turned from how far it turns now,
 * so taking hold of it changes nothing until it is dragged.
 */
describe('bending a bend again', () => {
  it('comes back to the same bend from the straight it was, and to straight at no turn', async () => {
    const { bendWhole } = await import('../src/scene/drawing.js');
    const dir = new THREE.Vector3(1, 0.2, -0.3).normalize();
    const axis = new THREE.Vector3(0, 1, 0).cross(dir).normalize();
    const straight = makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.042 });
    const bend = bendWhole(straight, dir, axis, 0.9, 0.06).segment;
    expect(bend.curve).toBeTruthy();

    // What the tool reads off the bend: the way it sets off, the way it ends, and the turn between.
    const swept = layoutPipe([bend], new THREE.Vector3(), dir);
    const start = swept.stations.find((st) => st.segment === 0)!.direction.clone().normalize();
    const out = swept.jointDirections[0]!.clone().normalize();
    const turnAxis = start.clone().cross(out).normalize();
    const again = bendWhole(makeSegment({ ...bend, curve: undefined }), start, turnAxis, start.angleTo(out), 0.06).segment;
    const redone = layoutPipe([again], new THREE.Vector3(), dir);
    expect(redone.joints[0]!.distanceTo(swept.joints[0]!)).toBeLessThan(1e-6);
    expect(again.length).toBeCloseTo(bend.length, 9);

    // Taken back to no turn, it is the straight it was.
    const flat = bendWhole(makeSegment({ ...bend, curve: undefined }), start, turnAxis, 0, 0.06).segment;
    expect(flat.curve).toBeUndefined();
    expect(flat.length).toBeCloseTo(bend.length, 9);
  });
});
