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
  closesLoop,
  removePipe,
  bendAnchor,
  sideArrival,
  sideLeaving,
  swingPipe,
  pipeShape,
  arcTo,
  collectSnapTargets,
  continuingDiameter,
  detachDuct,
  fitSegment,
  fitCurve,
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
import { ductDirections, layoutGraph, pipesMeetAt, type ExhaustPort } from '../src/scene/exhaustLayout.js';
import { bendRadius, layoutPipe } from '../src/scene/PipeMesh.js';
import { bendShape } from '../src/model/geometry.js';
import {
  attachToLooseStart,
  compileLayout,
  disconnectEnd,
  graphFromJson,
  joinDuctEnd,
  newDuctId,
  newNodeId,
  placeLoosePipe,
  reversedDucts,
  solverGraph,
  splitDuctAt,
  splitSegments,
  validateGraph,
  type ExhaustGraph,
} from '../src/model/exhaustGraph.js';
import { EngineMesh } from '../src/scene/EngineMesh.js';
import { refitBends } from '../src/scene/turboPlacement.js';
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

  it('marks a junction of three pipes where the branch leaves the pipe', async () => {
    const { JointMesh } = await import('../src/scene/jointMesh.js');
    const { graph, node } = branched();
    const joint = layoutGraph(ports(), graph).joints.get(node)!;
    expect(joint.limbs).toHaveLength(3);
    const mark = new JointMesh();
    mark.rebuild(joint);
    expect(mark.pickTarget).not.toBeNull();
    mark.dispose();
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

  it('pipes bent in to meet the pipe deleted stay, still meeting where it was, and fitted to it', async () => {
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
      expect(d.to).toEqual({ kind: 'node', node: 'join1' });
      expect(d.fitted).toBe(true);
      expect(d.segments).toHaveLength(runner().length + 1);
    }
    expect(nodeOrder(graph)).toEqual(['join1']);
    expect(junctionAt(graph, 'join1')).toBeDefined();
    expect(validateGraph(graph, 2)).toEqual([]);
  });

  it('a header’s pipes stay whole when its collector is deleted, fitted to where it was', async () => {
    const { pipesMeetAt } = await import('../src/scene/exhaustLayout.js');
    const { nodeOrder } = await import('../src/model/exhaustGraph.js');
    const ports = portsOf(twin);
    // Both runners bent, with a swing, into a collector fixed where they meet, as the header tool builds.
    const graph: ExhaustGraph = {
      ducts: [
        ...[0, 1].map((i) => ({
          id: `runner${i}`,
          segments: [makeSegment({ kind: 'pipe', length: 0.08, dIn: 0.042 }), makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.042 })],
          from: { kind: 'valve' as const, cylinder: i },
          to: { kind: 'node' as const, node: 'join1' },
          fitted: true as const,
          swing: true as const,
        })),
        { id: 'collector1', segments: collector(), from: { kind: 'node', node: 'join1' }, to: { kind: 'mouth' } },
      ],
      junctions: [{ node: 'join1', position: [0, -0.3, 0.1], axis: [0, 0, 1] }],
    };
    refitBends(graph, ports, twin);
    const before = graph.ducts.filter((d) => d.from.kind === 'valve').map((d) => d.segments.length);
    removePipe(graph, 'collector1', layoutGraph(ports, graph));
    refitBends(graph, ports, twin);
    const runners = graph.ducts.filter((d) => d.from.kind === 'valve');
    expect(runners.map((d) => d.segments.length)).toEqual(before);
    expect(runners.every((d) => d.fitted && d.to.kind === 'node')).toBe(true);
    expect(nodeOrder(graph)).toEqual(['join1']);
    expect(pipesMeetAt(graph, layoutGraph(ports, graph), 'join1')).toBe(true);
    expect(validateGraph(graph, 2)).toEqual([]);
  });

  it('a collector junction is only deleted once the collector after it is: then its runners end open', async () => {
    const { removeDuct, removeJunction, compileCollectorLayout } = await import('../src/model/exhaustGraph.js');
    const graph = compileCollectorLayout(v8, runner(), collector());
    // Four runners into a collector: nothing runs straight through, so the collector leaving it holds it.
    const before = JSON.stringify(graph);
    expect(removeJunction(graph, 'merge0')).toBe(false);
    expect(JSON.stringify(graph)).toBe(before);
    expect(removeDuct(graph, 'collector0')).toBe(true);
    expect(removeJunction(graph, 'merge0')).toBe(true);
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
    const { endsAt, removeJunction } = await import('../src/model/exhaustGraph.js');
    const graph = compileLayout(v8, runner(), collector());
    // The manifold widens here, so the graph records that it runs through.
    const onward = graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === 'merge0-1')!;
    const stub = endsAt(graph, 'merge0-1')
      .map((e) => e.duct.id)
      .find((d) => d !== onward.id && d !== onward.continues)!;
    removeJunction(graph, 'merge0-1');
    expect(graph.ducts.find((d) => d.id === stub)!.to).toEqual({ kind: 'mouth' });
    expect(graph.ducts.some((d) => d.id === 'collector0')).toBe(true);
    expect(validateGraph(graph, 8)).toEqual([]);
  });

  /** A tee deleted rejoins the pipe it was teed onto, once no branch leaves it. */
  it.each(['into', 'out of'])('a tee branching %s a pipe: deleting it rejoins the pipe', async (way) => {
    const { removeJunction } = await import('../src/model/exhaustGraph.js');
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
    // The graph records the pipe running through: the half after the split carries on from runner0.
    expect(graph.ducts.some((d) => d.from.kind === 'node' && d.from.node === node && d.continues === 'runner0')).toBe(true);

    // A branch leaving it holds it, until the branch is deleted.
    if (way === 'out of') {
      expect(removeJunction(graph, node)).toBe(false);
      graph.ducts = graph.ducts.filter((d) => d.id !== 'drawn0');
    }
    expect(removeJunction(graph, node)).toBe(true);
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
 * No deletion moves a pipe it did not touch.
 *
 * Pipes the layout aims, a V-twin's runners and an 8-into-1's downpipes, would be re-aimed after every
 * deletion, swinging pipes nobody touched by metres. Every single deletion on every preset is tried here, doing what the app does: fix every pipe
 * where it stands, delete, take the edited pipe off its junction if it no longer reaches, and tidy with
 * the directions from before the delete.
 */
describe('deleting keeps the exhaust in one piece', async () => {
  const { ENGINE_PRESETS } = await import('../src/model/spec.js');
  const { ductDirections, freezeHeadings, pipesMeetAt } = await import('../src/scene/exhaustLayout.js');
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
      // No pipe the delete did not touch moves.
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
      // Refused where a pipe it would delete has others carrying on from it: then nothing changes.
      if (!removeJunction(g, node, dirs)) {
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

/**
 * A pipe drawn into the side of a placed pipe, open at both ends, makes a T: the gas arriving at the junction
 * goes out of both ends, though the half before it was drawn running into it.
 */
describe('a T into a placed pipe', () => {
  const spec = defaultConfig().engine;
  const tee = () => {
    const graph = compileLayout(spec, [makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.042 })], []);
    const loose = placeLoosePipe(graph, [0.2, -0.3, 0.4], 0.042, 0.6);
    graph.ducts.find((d) => d.id === loose)!.segments[0]!.dOut = 0.05;
    const node = splitDuctAt(graph, loose, 0.2)!;
    graph.ducts.find((d) => d.from.kind === 'valve')!.to = { kind: 'node', node };
    return { graph, loose, node };
  };

  it('gives the solver the half before the junction turned round, out to its open end', () => {
    const { graph, loose, node } = tee();
    expect(reversedDucts(graph)).toEqual(new Set([loose]));
    const solved = solverGraph(graph);
    expect(validateGraph(solved, spec.cylinders)).toEqual([]);
    const back = solved.ducts.find((d) => d.id === loose)!;
    expect(back.from).toEqual({ kind: 'node', node });
    expect(back.to).toEqual({ kind: 'mouth' });
    // Its taper the other way round: it widened towards the junction, so it narrows away from it.
    const drawn = graph.ducts.find((d) => d.id === loose)!.segments[0]!;
    expect(back.segments[0]!.dIn).toBeCloseTo(segmentDiameter(drawn, 1), 12);
    expect(back.segments.at(-1)!.dOut).toBeCloseTo(drawn.dIn, 12);
    expect(solved.ducts.filter((d) => d.to.kind === 'mouth')).toHaveLength(2);
  });

  it('carries the gas out of both ends, heard through the Wasm build', async () => {
    const { Sim } = await import('../src/audio/worklet/sim.js');
    const { graph, loose } = tee();
    const cfg = defaultConfig();
    cfg.engine = { ...spec, throttle: 1, rpm: 4000, freeRunning: false };
    cfg.graph = solverGraph(graph);
    const sim = new Sim(48000, cfg);
    sim.render(48000);
    const s = sim.snapshot();
    const swing = (id: string) => {
      let at = 0;
      for (let k = 0; k < s.ductIds.length; k++) {
        const n = s.ductCells[k]!;
        if (s.ductIds[k] === id) return Math.max(...s.ductPressure.subarray(at, at + n).map(Math.abs));
        at += n;
      }
      return -1;
    };
    const other = graph.ducts.find((d) => d.id !== loose && d.from.kind === 'node')!.id;
    expect(swing(loose)).toBeGreaterThan(0);
    expect(swing(other)).toBeGreaterThan(0);
  });
});

/**
 * A pipe drawn into the side of another with Shift held meets it square, at 90 degrees, rather than curving
 * round into its flow; and stays square as its bend is fitted again. Either way it ends at the bore of the
 * pipe it joins, and follows it when that changes.
 */
describe('a square T', () => {
  const spec = defaultConfig().engine;
  const ports = (): ExhaustPort[] => [new EngineMesh(spec).exhaustPort(0)];

  /** The runner drawn into the side of a loose pipe of a wider bore, square or not, its bend fitted. */
  function joined(square: boolean) {
    const graph = compileLayout(spec, [makeSegment({ kind: 'pipe', length: 0.2, dIn: 0.042 })], []);
    const loose = placeLoosePipe(graph, [0.3, -0.35, 0.3], 0.06, 0.6);
    const node = splitDuctAt(graph, loose, 0.3)!;
    const runner = graph.ducts.find((d) => d.from.kind === 'valve')!;
    runner.to = { kind: 'node', node };
    runner.segments.push(makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.042 }));
    runner.fitted = true;
    if (square) runner.square = true;
    refitBends(graph, ports(), spec);
    const placement = layoutGraph(ports(), graph);
    const place = placement.ducts.get(runner.id)!;
    const swept = layoutPipe(runner.segments, place.origin, place.heading);
    const main = placement.ducts.get(loose)!;
    const mainSwept = layoutPipe(graph.ducts.find((d) => d.id === loose)!.segments, main.origin, main.heading);
    return {
      graph,
      loose,
      runner,
      end: swept.joints.at(-1)!,
      arrives: swept.jointDirections.at(-1)!,
      at: mainSwept.joints.at(-1)!,
      axis: mainSwept.jointDirections.at(-1)!,
    };
  }

  it('arrives straight across the pipe, at its bore', () => {
    const { runner, end, arrives, at, axis } = joined(true);
    expect(end.distanceTo(at)).toBeLessThan(1e-6);
    expect(Math.abs(arrives.dot(axis))).toBeLessThan(1e-6);
    expect(segmentDiameter(runner.segments.at(-1)!, 1)).toBeCloseTo(0.06, 12);
  });

  it('follows the pipe it joins to a new bore, and stays square', () => {
    const { graph, loose, runner } = joined(true);
    for (const d of graph.ducts) {
      if (d.from.kind === 'valve') continue;
      for (const seg of d.segments) {
        seg.dIn = 0.07;
        seg.dOut = 0.07;
      }
    }
    refitBends(graph, ports(), spec);
    expect(segmentDiameter(runner.segments.at(-1)!, 1)).toBeCloseTo(0.07, 12);
    const placement = layoutGraph(ports(), graph);
    const place = placement.ducts.get(runner.id)!;
    const arrives = layoutPipe(runner.segments, place.origin, place.heading).jointDirections.at(-1)!;
    const main = placement.ducts.get(loose)!;
    const axis = layoutPipe(graph.ducts.find((d) => d.id === loose)!.segments, main.origin, main.heading).jointDirections.at(-1)!;
    expect(Math.abs(arrives.dot(axis))).toBeLessThan(1e-6);
  });

  it('otherwise curves round into its flow, at its bore', () => {
    const { runner, arrives, axis } = joined(false);
    expect(arrives.dot(axis)).toBeGreaterThan(1 - 1e-6);
    expect(segmentDiameter(runner.segments.at(-1)!, 1)).toBeCloseTo(0.06, 12);
  });

  it('keeps it in a link', () => {
    const { runner } = joined(true);
    const graph = { ducts: [runner] };
    expect(graphFromJson(JSON.parse(JSON.stringify(graph)))!.ducts[0]!.square).toBe(true);
  });

  it('drawn in from a side, comes in along the pipe, its end the same circle as the pipe there', async () => {
    const { pipesMeetAt } = await import('../src/scene/exhaustLayout.js');
    const { graph, runner, arrives, axis } = joined(false);
    const node = (runner.to as { node: string }).node;
    expect(pipesMeetAt(graph, layoutGraph(ports(), graph), node)).toBe(true);
    expect(Math.abs(arrives.dot(axis))).toBeGreaterThan(1 - 1e-9);
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

describe('closesLoop', () => {
  const pipe = (length = 0.3) => makeSegment({ kind: 'pipe', length, dIn: 0.042, dOut: 0.042 });
  const at = (x: number, y = 0, z = 0) => new THREE.Vector3(x, y, z);

  /** Two runners merging at `j`, and `tail` drawn from their junction, still ending in air. */
  function collector(): ExhaustGraph {
    return {
      ducts: [
        { id: 'a', segments: [pipe()], from: { kind: 'valve', cylinder: 0 }, to: { kind: 'node', node: 'j' } },
        { id: 'b', segments: [pipe()], from: { kind: 'valve', cylinder: 1 }, to: { kind: 'node', node: 'j' } },
        { id: 'tail', segments: [pipe()], from: { kind: 'node', node: 'j' }, to: { kind: 'mouth' } },
      ],
    };
  }

  it('refuses a pipe drawn from a junction back into its own junction', () => {
    expect(closesLoop(collector(), 'tail', { kind: 'node', point: at(0), node: 'j' })).toBe(true);
  });

  it('refuses a pipe drawn from a junction back into the side of a runner feeding it', () => {
    const target = { kind: 'ductSurface' as const, point: at(0), duct: 'a', x: 0.15 };
    expect(closesLoop(collector(), 'tail', target)).toBe(true);
  });

  it('allows a loop where another pipe still reaches the air', () => {
    const graph = collector();
    graph.ducts.push({ id: 'drawn', segments: [pipe()], from: { kind: 'node', node: 'j' }, to: { kind: 'mouth' } });
    const target = { kind: 'ductSurface' as const, point: at(0), duct: 'a', x: 0.15 };
    expect(closesLoop(graph, 'drawn', target)).toBe(false);
  });

  it('allows merging into a pipe that vents to air, which leaves the junction open', () => {
    const graph = collector();
    graph.ducts.push({ id: 'c', segments: [pipe()], from: { kind: 'valve', cylinder: 2 }, to: { kind: 'mouth' } });
    expect(closesLoop(graph, 'c', { kind: 'ductEnd', point: at(0), duct: 'tail' })).toBe(false);
    expect(closesLoop(graph, 'c', { kind: 'ductSurface', point: at(0), duct: 'tail', x: 0.15 })).toBe(false);
  });

  it("refuses a loose pipe carried on into its own start", () => {
    const graph: ExhaustGraph = { ducts: [] };
    const loose = placeLoosePipe(graph, [0, 0, 0], 0.042, 0.3);
    const target = { kind: 'looseStart' as const, point: at(0), dir: at(0, 0, 1), dia: 0.042, duct: loose };
    expect(closesLoop(graph, loose, target)).toBe(true);
  });

  it('does not change the graph it is asked about', () => {
    const graph = collector();
    const before = JSON.stringify(graph);
    closesLoop(graph, 'tail', { kind: 'ductSurface', point: at(0), duct: 'a', x: 0.15 });
    expect(JSON.stringify(graph)).toBe(before);
  });
});

describe('sideArrival', () => {
  const point = new THREE.Vector3(0, 0, 0);
  const axis = new THREE.Vector3(0, 0, 1);

  it('merges along the pipe from whichever end it is drawn down', () => {
    const across = new THREE.Vector3(1, 0, 0);
    expect(sideArrival(point, axis, new THREE.Vector3(0.2, 0, -0.3), across).toArray()).toEqual([0, 0, 1]);
    expect(sideArrival(point, axis, new THREE.Vector3(0.2, 0, 0.3), across).z).toBe(-1);
  });

  it('level with the joint, goes the way the pipe drawn leans along it', () => {
    const from = new THREE.Vector3(0.2, 0, 0);
    expect(sideArrival(point, axis, from, new THREE.Vector3(-1, 0, -0.2)).z).toBe(-1);
    expect(sideArrival(point, axis, from, new THREE.Vector3(-1, 0, 0.2)).z).toBe(1);
    expect(sideArrival(point, axis, from, new THREE.Vector3(-1, 0, 0)).z).toBe(1);
  });
});

describe('drawing out of the side of a pipe', () => {
  const spec = { ...defaultConfig().engine, cylinders: 1 } as EngineSpec;
  const ports = (): ExhaustPort[] => [new EngineMesh(spec).exhaustPort(0)];

  it('sets off along the pipe, whichever way along it the point lies, flush with it', () => {
    const axis = new THREE.Vector3(0, 0, 1);
    const tip = new THREE.Vector3(0, 0, 0);
    expect(sideLeaving(axis, tip, new THREE.Vector3(0.2, 0, 0.3)).toArray()).toEqual([0, 0, 1]);
    expect(sideLeaving(axis, tip, new THREE.Vector3(0, -0.2, -0.3)).z).toBe(-1);
    expect(sideLeaving(axis, tip, new THREE.Vector3(0.2, 0, 0)).z).toBe(1);
  });

  it('bends in one arc from along the pipe to the point', () => {
    const tip = new THREE.Vector3(0, 0, 0);
    const dir = new THREE.Vector3(0, 0, -1);
    const point = new THREE.Vector3(0.2, 0.1, -0.3);
    const seg = arcTo(tip, dir, point, { dIn: 0.042, dOut: 0.042 });
    expect(seg.curve).toBeDefined();
    const swept = layoutPipe([seg], tip, dir);
    expect(swept.stations[0]!.direction.angleTo(dir)).toBeLessThan(1e-6);
    expect(swept.joints.at(-1)!.distanceTo(point)).toBeLessThan(1e-6);
    // As an arc, it arrives turned off the chord by as much as it set off.
    const chord = point.clone().normalize();
    expect(swept.jointDirections.at(-1)!.angleTo(chord)).toBeCloseTo(dir.angleTo(chord), 2);
  });

  it('laid out, a branch set off back along the pipe leaves tangent to it', () => {
    const graph = compileLayout(spec, [makeSegment({ kind: 'pipe', length: 0.8, dIn: 0.042 })], []);
    const before = layoutGraph(ports(), graph);
    const up = before.ducts.get('runner0')!;
    const swept = layoutPipe(graph.ducts[0]!.segments, up.origin, up.heading);
    const node = splitDuctAt(graph, 'runner0', 0.4)!;
    const placement = layoutGraph(ports(), graph);
    const axis = placement.joints.get(node)?.axis ?? swept.stations.find((s) => s.x >= 0.4)!.direction;
    const origin = placement.ducts.get(graph.ducts.find((d) => d.continues === 'runner0')!.id)!.origin;
    const back = axis.clone().negate();
    const point = origin.clone().addScaledVector(back, 0.25).add(new THREE.Vector3(0, 0.15, 0));
    const turn = headingOffsetTo(axis, back);
    graph.ducts.push({
      id: 'branch',
      segments: [arcTo(origin, back, point, { dIn: 0.042, dOut: 0.042 })],
      from: { kind: 'node', node },
      to: { kind: 'mouth' },
      headingYaw: turn.yaw,
      headingPitch: turn.pitch,
    });
    const laid = layoutGraph(ports(), graph).ducts.get('branch')!;
    const branch = layoutPipe(graph.ducts.at(-1)!.segments, laid.origin, laid.heading);
    expect(laid.origin.distanceTo(origin)).toBeLessThan(1e-9);
    expect(branch.stations[0]!.direction.angleTo(back)).toBeLessThan(1e-6);
    expect(branch.joints.at(-1)!.distanceTo(point)).toBeLessThan(1e-6);
    expect(validateGraph(graph, 1)).toEqual([]);
  });
});

describe("drawing onto another pipe's open end", () => {
  const twin = { ...defaultConfig().engine, cylinders: 2, vAngle: 45, exhaustLayout: 'perBank' } as EngineSpec;
  const ports = (): ExhaustPort[] => {
    const mesh = new EngineMesh(twin);
    return [0, 1].map((i) => mesh.exhaustPort(i));
  };

  // Where the ghost's bend goes, as the editor works it out before the click.
  for (const [label, beyond] of [['from behind its end', -0.15], ['from beyond its end', 0.15]] as const) {
    it(`bends in along the pipe, as the ghost showed, ${label}`, () => {
      const graph: ExhaustGraph = {
        ducts: [0, 1].map((i) => ({
          id: `runner${i}`,
          segments: [makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.042 })],
          from: { kind: 'valve' as const, cylinder: i },
          to: { kind: 'mouth' as const },
        })),
      };
      const placement = layoutGraph(ports(), graph);
      const a = placement.ducts.get('runner0')!;
      const sweptA = layoutPipe(graph.ducts[0]!.segments, a.origin, a.heading);
      const end = sweptA.joints.at(-1)!.clone();
      const endDir = sweptA.jointDirections.at(-1)!.clone();
      // runner1 drawn on to a point off to the side of runner0's end, behind or beyond it.
      const b = placement.ducts.get('runner1')!;
      const tipB = layoutPipe(graph.ducts[1]!.segments, b.origin, b.heading);
      const target = end.clone().addScaledVector(endDir, beyond).add(new THREE.Vector3(0, 0.12, 0));
      const runner1 = graph.ducts[1]!;
      runner1.segments.push(fitSegment(tipB.joints.at(-1)!, tipB.jointDirections.at(-1)!, target, { kind: 'pipe', dIn: 0.042, dOut: 0.042 }));
      const drawn = layoutPipe(runner1.segments, b.origin, b.heading);
      const tip = { point: drawn.joints.at(-1)!, dir: drawn.jointDirections.at(-1)! };
      // From either side it comes in along the pipe, so every pipe joining an end comes in together.
      const ghostDir = endDir.clone();
      const ghost = fitCurve(tip.point, tip.dir, end, ghostDir, { dIn: 0.042, dOut: 0.042 });

      // The click: joined as the editor joins it, then fitted again as every rebuild does.
      runner1.segments.push(makeSegment(ghost));
      runner1.fitted = true;
      const node = joinDuctEnd(graph, 'runner0', { position: [end.x, end.y, end.z], axis: [endDir.x, endDir.y, endDir.z] })!;
      runner1.to = { kind: 'node', node };
      refitBends(graph, ports(), twin);
      const after = layoutGraph(ports(), graph).ducts.get('runner1')!;
      const swept = layoutPipe(runner1.segments, after.origin, after.heading);
      expect(swept.joints.at(-1)!.distanceTo(end)).toBeLessThan(1e-6);
      expect(swept.jointDirections.at(-1)!.angleTo(ghostDir)).toBeLessThan(1e-6);
    });
  }
});

describe('turning a pipe with another carrying on from its end', () => {
  const spec = { ...defaultConfig().engine, cylinders: 1 } as EngineSpec;
  const ports = (): ExhaustPort[] => [new EngineMesh(spec).exhaustPort(0)];

  it('swings the pipe carried on round with it, as one piece', () => {
    const straight = makeSegment({ kind: 'pipe', length: 0.2, dIn: 0.042 });
    const graph: ExhaustGraph = {
      ducts: [
        { id: 'runner0', segments: [straight], from: { kind: 'valve', cylinder: 0 }, to: { kind: 'node', node: 'j' }, headingYaw: 0, headingPitch: 0 },
        { id: 'bend', segments: [], from: { kind: 'node', node: 'j' }, to: { kind: 'mouth' } },
      ],
    };
    const before = layoutGraph(ports(), graph);
    const s = before.ducts.get('runner0')!;
    const j = layoutPipe([straight], s.origin, s.heading);
    const tip = j.joints.at(-1)!;
    const dir = j.jointDirections.at(-1)!;
    const bend = graph.ducts[1]!;
    bend.segments = [arcTo(tip, dir, tip.clone().addScaledVector(dir, 0.15).add(new THREE.Vector3(0.1, 0.1, 0)), { dIn: 0.042, dOut: 0.042 })];
    const laid = (g: ExhaustGraph) => {
      const p = layoutGraph(ports(), g).ducts.get('bend')!;
      return layoutPipe(bend.segments, p.origin, p.heading).joints.at(-1)!;
    };
    const end0 = laid(graph);

    // As the triad's ring does, about the port's axis through where the runner starts.
    const axis = s.heading.clone().normalize();
    const angle = 1.1;
    const place = layoutGraph(ports(), graph).ducts.get('bend')!;
    const world = new THREE.Vector3(1, 0, 0);
    const turn = headingOffsetTo(world, place.heading);
    Object.assign(bend, { headingYaw: turn.yaw, headingPitch: turn.pitch, headingFrame: 'world' });
    const bendShape0 = pipeShape(bend.segments, place.heading);
    swingPipe(graph.ducts[0]!, ports()[0]!.direction, pipeShape([straight], s.heading), axis, angle);
    swingPipe(bend, world, bendShape0, axis, angle);

    const expected = end0.clone().sub(s.origin).applyAxisAngle(axis, angle).add(s.origin);
    expect(laid(graph).distanceTo(expected)).toBeLessThan(1e-6);
    expect(laid(graph).distanceTo(end0)).toBeGreaterThan(0.05);
  });
});

describe('pivoting a pipe where it leaves a junction fixed in place', () => {
  const spec = { ...defaultConfig().engine, cylinders: 1 } as EngineSpec;
  const ports = (): ExhaustPort[] => [new EngineMesh(spec).exhaustPort(0)];

  it('the bend into the junction follows it round', async () => {
    const { pipesMeetAt } = await import('../src/scene/exhaustLayout.js');
    // A runner drawn onto the start of a loose pipe: bent in to a junction fixed where the loose pipe began.
    const graph: ExhaustGraph = {
      ducts: [{ id: 'runner0', segments: [makeSegment({ kind: 'pipe', length: 0.2, dIn: 0.042 })], from: { kind: 'valve', cylinder: 0 }, to: { kind: 'mouth' } }],
    };
    const loose = placeLoosePipe(graph, [0.4, 0.3, 0.1], 0.042, 0.5);
    attachToLooseStart(graph, 'runner0', loose, [0, 0, 1]);
    const runner = graph.ducts[0]!;
    runner.fitted = true;
    runner.segments.push(makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.042 }));
    refitBends(graph, ports(), spec);
    const node = (runner.to as { node: string }).node;
    expect(pipesMeetAt(graph, layoutGraph(ports(), graph), node)).toBe(true);

    // Pivoted where it starts: turned off the crank, and up.
    const pipe = graph.ducts.find((d) => d.id === loose)!;
    pipe.headingYaw = -Math.PI / 2 + 0.6;
    pipe.headingPitch = 0.3;
    refitBends(graph, ports(), spec);
    const placement = layoutGraph(ports(), graph);
    expect(pipesMeetAt(graph, placement, node)).toBe(true);
    const out = placement.ducts.get(loose)!;
    const leaving = layoutPipe(pipe.segments, out.origin, out.heading).stations[0]!.direction;
    const r = placement.ducts.get('runner0')!;
    const arriving = layoutPipe(runner.segments, r.origin, r.heading).jointDirections.at(-1)!;
    expect(arriving.angleTo(leaving)).toBeLessThan(1e-6);
  });
});

describe('rolling a pipe with a can in it', () => {
  it('turns the can round with it, its pipes off its middle and all', () => {
    const can = makeSegment({ kind: 'chamber', length: 0.3, dIn: 0.05, dOut: 0.2, section: 'oval', height: 0.1, offsetOut: 0.05 });
    const duct: ExhaustGraph['ducts'][number] = {
      id: 'tail',
      segments: [makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.05 }), can],
      from: { kind: 'free', position: [0, 0, 0] },
      to: { kind: 'mouth' },
      headingYaw: 0,
      headingPitch: 0,
      headingFrame: 'world',
    };
    const origin = new THREE.Vector3();
    const heading = new THREE.Vector3(1, 0, 0);
    const before = layoutPipe(duct.segments, origin, heading);
    const angle = 0.9;
    swingPipe(duct, heading, pipeShape(duct.segments, heading), heading, angle);
    expect(duct.segments[1]!.roll).toBeCloseTo(angle, 9);
    const after = layoutPipe(duct.segments, origin, heading);
    const turn = (v: THREE.Vector3) => v.clone().applyAxisAngle(heading, angle);
    const last = (l: typeof before) => l.stations.at(-1)!;
    expect(last(after).across.angleTo(turn(last(before).across))).toBeLessThan(1e-9);
    expect(after.joints.at(-1)!.distanceTo(turn(before.joints.at(-1)!))).toBeLessThan(1e-9);
  });

  it('keeps its roll through a saved link', () => {
    const can = makeSegment({ kind: 'chamber', length: 0.3, dIn: 0.05, dOut: 0.2, roll: 0.4 });
    expect(makeSegment(JSON.parse(JSON.stringify(can))).roll).toBe(0.4);
  });
});

describe('where one segment of a pipe meets the next', () => {
  const spec = { ...defaultConfig().engine, cylinders: 1 } as EngineSpec;
  const ports = (): ExhaustPort[] => [new EngineMesh(spec).exhaustPort(0)];

  it('is offered to draw from or join, and splits the pipe there into a junction', () => {
    const graph: ExhaustGraph = { ducts: [] };
    const id = placeLoosePipe(graph, [0, 0.3, 0], 0.042, 0.3);
    const pipe = graph.ducts[0]!;
    pipe.segments.push(makeSegment({ kind: 'pipe', length: 0.2, dIn: 0.042 }));
    const placement = layoutGraph(ports(), graph);
    const place = placement.ducts.get(id)!;
    const at = layoutPipe(pipe.segments, place.origin, place.heading).joints[0]!;
    const joints = collectSnapTargets(graph, placement, ports()).filter((t) => t.kind === 'ductSurface');
    expect(joints).toHaveLength(1);
    const joint = joints[0]!;
    expect(joint.kind === 'ductSurface' && joint.x).toBeCloseTo(0.3, 12);
    expect(joint.point.distanceTo(at)).toBeLessThan(1e-9);

    const node = splitDuctAt(graph, id, (joint as { x: number }).x)!;
    expect(pipe.segments).toHaveLength(1);
    expect(pipe.to).toEqual({ kind: 'node', node });
    const rest = graph.ducts.find((d) => d.continues === id)!;
    expect(rest.segments).toHaveLength(1);
    expect(rest.segments[0]!.length).toBeCloseTo(0.2, 12);
  });

  it('is offered where a bend fitted to what the pipe joins begins, and splits the pipe there, the bend its own', () => {
    const graph: ExhaustGraph = { ducts: [] };
    const id = placeLoosePipe(graph, [0, 0.3, 0], 0.042, 0.3);
    const pipe = graph.ducts[0]!;
    pipe.segments.push(makeSegment({ kind: 'pipe', length: 0.2, dIn: 0.042 }));
    pipe.fitted = true;
    const at = collectSnapTargets(graph, layoutGraph(ports(), graph), ports()).filter((t) => t.kind === 'ductSurface');
    expect(at).toHaveLength(1);
    const node = splitDuctAt(graph, id, (at[0] as { x: number }).x)!;
    // The straight before it is a pipe of its own now, and the bend its own, carrying it on and still fitted.
    expect(pipe.fitted).toBeUndefined();
    const bend = graph.ducts.find((d) => d.continues === id)!;
    expect(bend.from).toEqual({ kind: 'node', node });
    expect(bend.fitted).toBe(true);
    expect(bend.segments).toHaveLength(1);
  });

  it('is not offered within a bend fitted to what the pipe joins, swing and all', () => {
    const graph: ExhaustGraph = { ducts: [] };
    placeLoosePipe(graph, [0, 0.3, 0], 0.042, 0.3);
    const pipe = graph.ducts[0]!;
    pipe.segments.push(makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.042 }), makeSegment({ kind: 'pipe', length: 0.2, dIn: 0.042 }));
    pipe.fitted = true;
    pipe.swing = true;
    // Only where the swing begins, after the straight: not between the swing and the bend.
    expect(collectSnapTargets(graph, layoutGraph(ports(), graph), ports()).filter((t) => t.kind === 'ductSurface')).toHaveLength(1);
  });
});

/**
 * Edits one after another, as they come: deleting pipes, moving loose pipes and fixed junctions, and drawing
 * from a port onto a junction or a pipe's open end. However they fall, every junction's pipes still meet at it, and the solver
 * still takes the exhaust.
 */
describe('edits one after another', () => {
  const spec = { ...defaultConfig().engine, cylinders: 6, vAngle: 60, exhaustLayout: 'perBank' } as EngineSpec;
  const ports = (): ExhaustPort[] => {
    const mesh = new EngineMesh(spec);
    return Array.from({ length: mesh.bankCount }, (_, b) => mesh.exhaustPort(b));
  };

  /** Two loose pipes a bend runs between, out of one's side into the other's, and six bare runners. */
  function crossover(): ExhaustGraph {
    const graph: ExhaustGraph = {
      ducts: Array.from({ length: 6 }, (_, c) => ({ id: `runner${c}`, segments: [], from: { kind: 'valve' as const, cylinder: c }, to: { kind: 'mouth' as const } })),
    };
    const a = placeLoosePipe(graph, [0.019, 0.17, 0.535], 0.045, 0.425);
    const b = placeLoosePipe(graph, [0.332, 0.17, 0.512], 0.045, 0.425);
    const ja = splitDuctAt(graph, a, 0.226)!;
    const jb = splitDuctAt(graph, b, 0.205)!;
    graph.ducts.push({
      id: 'bend', segments: [makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.045 })], from: { kind: 'node', node: ja },
      to: { kind: 'node', node: jb }, headingYaw: 2.618, headingPitch: 0, fitted: true,
    });
    return graph;
  }

  it('keeps every junction met however they fall', async () => {
    const { moveJunction } = await import('../src/scene/turboPlacement.js');
    const { pipesMeetAt } = await import('../src/scene/exhaustLayout.js');
    const { nodeOrder } = await import('../src/model/exhaustGraph.js');
    const p = ports();
    let seed = 1;
    const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
    const pick = <T,>(xs: T[]) => xs[Math.floor(rnd() * xs.length)]!;
    for (let run = 0; run < 120; run++) {
      seed = run * 7919 + 13;
      const graph = crossover();
      const lay = () => layoutGraph(p, graph);
      refitBends(graph, p, spec);
      const done: string[] = [];
      for (let step = 0; step < 8; step++) {
        const stable = lay();
        const op = pick(['delete', 'delete', 'move', 'junction', 'draw']);
        const piped = graph.ducts.filter((d) => d.segments.length > 0);
        if (op === 'delete' && piped.length > 0) {
          const d = pick(piped);
          done.push(`delete ${d.id}`);
          removePipe(graph, d.id, stable, ductDirections(graph, stable));
        } else if (op === 'move') {
          const loose = graph.ducts.filter((d) => d.from.kind === 'free' && d.segments.length > 0);
          if (loose.length === 0) continue;
          const d = pick(loose);
          done.push(`move ${d.id}`);
          const [x, y, z] = (d.from as { position: [number, number, number] }).position;
          d.from = { kind: 'free', position: [x + (rnd() - 0.5) * 0.1, y + (rnd() - 0.5) * 0.1, z + (rnd() - 0.5) * 0.1] };
        } else if (op === 'junction') {
          const nodes = nodeOrder(graph).filter((n) => stable.joints.has(n));
          if (nodes.length === 0) continue;
          const n = pick(nodes);
          const j = stable.joints.get(n)!;
          done.push(`move junction ${n}`);
          moveJunction(graph, p, spec, n, [j.centre.x + (rnd() - 0.5) * 0.08, j.centre.y + (rnd() - 0.5) * 0.08, j.centre.z], [j.axis.x, j.axis.y, j.axis.z]);
        } else {
          // Drawn from a port onto a junction or a pipe's open end, as the editor joins them
          // (`PipeEditor.connect`): an open end made a junction at it, which follows it; a junction only
          // where its pipes end fixed in place first; and the route bent in to meet it.
          const nodes = nodeOrder(graph).filter((n) => stable.joints.has(n));
          const open = graph.ducts.filter((d) => d.to.kind === 'mouth' && d.segments.length > 0 && !d.fitted && d.from.kind !== 'valve');
          const bare = graph.ducts.filter((d) => d.from.kind === 'valve' && d.segments.length === 0);
          if (nodes.length + open.length === 0 || bare.length === 0) continue;
          const r = pick(bare);
          const onEnd = open.length > 0 && (nodes.length === 0 || rnd() < 0.5);
          const n = onEnd ? joinDuctEnd(graph, pick(open).id)! : pick(nodes);
          done.push(`draw ${r.id} onto ${onEnd ? 'an end at ' : ''}${n}`);
          r.segments = [makeSegment({ kind: 'pipe', length: 0.12, dIn: 0.042 })];
          let pl = lay();
          if (!bendAnchor(graph, pl, n, r.id) && !graph.junctions?.some((j) => j.node === n)) {
            const j = pl.joints.get(n)!;
            (graph.junctions ??= []).push({ node: n, position: [j.centre.x, j.centre.y, j.centre.z], axis: [j.axis.x, j.axis.y, j.axis.z] });
            pl = lay();
          }
          const anchor = bendAnchor(graph, pl, n, r.id)!;
          const tip = routeTip(r.segments, pl.ducts.get(r.id)!);
          r.segments.push(fitCurve(tip.point, tip.dir, anchor.point, anchor.dir, { dIn: 0.042, dOut: anchor.dia }));
          r.fitted = true;
          r.to = { kind: 'node', node: n };
        }
        refitBends(graph, p, spec);
        const placement = lay();
        const apart = nodeOrder(graph).filter((n) => !pipesMeetAt(graph, placement, n));
        expect({ run, done, apart, problems: validateGraph(graph, spec.cylinders) }).toEqual({ run, done, apart: [], problems: [] });
      }
    }
  });
});

/**
 * Joining and deleting by the simple rules: a fitted pipe is the only thing that joins; nothing it joins is
 * turned round or moved; and deleting it puts back exactly what was there before it was drawn.
 */
describe('fitted pipes join straight ones, and deleting one puts back what was there', () => {
  const spec = { ...defaultConfig().engine, cylinders: 1 } as EngineSpec;
  const ports = (): ExhaustPort[] => [new EngineMesh(spec).exhaustPort(0)];
  const bare = (): ExhaustGraph => ({ ducts: [{ id: 'runner0', segments: [], from: { kind: 'valve', cylinder: 0 }, to: { kind: 'mouth' } }] });

  /** Fit the runner's bend into `node` as the editor does, from a short straight out of its port. */
  function fitRunner(graph: ExhaustGraph, node: string): void {
    const runner = graph.ducts[0]!;
    runner.segments = [makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.042 })];
    const placement = layoutGraph(ports(), graph);
    const anchor = bendAnchor(graph, placement, node, 'runner0')!;
    const tip = routeTip(runner.segments, placement.ducts.get('runner0')!);
    runner.segments.push(fitCurve(tip.point, tip.dir, anchor.point, anchor.dir, { dIn: 0.042, dOut: anchor.dia }));
    runner.fitted = true;
    runner.to = { kind: 'node', node };
    refitBends(graph, ports(), spec);
  }

  /** Where a duct is and what it is, to compare before and after. */
  const shape = (graph: ExhaustGraph, id: string) => {
    const d = graph.ducts.find((x) => x.id === id)!;
    const p = layoutGraph(ports(), graph).ducts.get(id)!;
    const swept = layoutPipe(d.segments, p.origin, p.heading);
    const r = (v: THREE.Vector3) => v.toArray().map((x) => +x.toFixed(9));
    return { from: d.from, to: d.to, lengths: d.segments.map((sg) => +sg.length.toFixed(9)), start: r(p.origin), end: r(swept.joints.at(-1)!) };
  };

  it('onto a loose pipe’s open end: joined at its end, which it follows, and never turned round', async () => {
    const { pipesMeetAt } = await import('../src/scene/exhaustLayout.js');
    const graph = bare();
    const id = placeLoosePipe(graph, [0.4, 0.2, 0.1], 0.042, 0.3);
    const before = shape(graph, id);
    const node = joinDuctEnd(graph, id)!;
    fitRunner(graph, node);
    const loose = graph.ducts.find((d) => d.id === id)!;
    // Not turned round: it starts where it did, and ends at the junction.
    expect(shape(graph, id).start).toEqual(before.start);
    expect(loose.to).toEqual({ kind: 'node', node });
    expect(pipesMeetAt(graph, layoutGraph(ports(), graph), node)).toBe(true);
    // Moved and turned, the runner's bend follows its end.
    loose.from = { kind: 'free', position: [0.45, 0.25, 0.05] };
    loose.headingYaw = (loose.headingYaw ?? 0) + 0.5;
    refitBends(graph, ports(), spec);
    expect(pipesMeetAt(graph, layoutGraph(ports(), graph), node)).toBe(true);
    expect(validateGraph(graph, 1)).toEqual([]);
  });

  it('deleted from a pipe’s side: the pipe one again, exactly as it was', () => {
    const graph = bare();
    const id = placeLoosePipe(graph, [0.4, 0.2, 0.1], 0.042, 0.5);
    const before = shape(graph, id);
    const node = splitDuctAt(graph, id, 0.2)!;
    fitRunner(graph, node);
    removePipe(graph, 'runner0', layoutGraph(ports(), graph));
    expect(graph.ducts.filter((d) => d.id !== 'runner0').map((d) => d.id)).toEqual([id]);
    expect(shape(graph, id)).toEqual(before);
    expect(graph.ducts[0]!.fitted).toBeUndefined();
  });

  it('deleted from a loose pipe’s start: it starts where it did again, loose, the way it pointed', () => {
    const graph = bare();
    const id = placeLoosePipe(graph, [0.4, 0.2, 0.1], 0.042, 0.3);
    const before = shape(graph, id);
    const at = layoutGraph(ports(), graph).ducts.get(id)!;
    const node = attachToLooseStart(graph, 'runner0', id, [at.heading.x, at.heading.y, at.heading.z])!;
    fitRunner(graph, node);
    removePipe(graph, 'runner0', layoutGraph(ports(), graph));
    expect(shape(graph, id)).toEqual(before);
    expect(graph.junctions ?? []).toEqual([]);
  });

  it('drawn out of a loose pipe’s start, and deleted: it starts where it did again', () => {
    const graph = bare();
    const id = placeLoosePipe(graph, [0.4, 0.2, 0.1], 0.042, 0.3);
    const other = placeLoosePipe(graph, [0.1, 0.4, 0.4], 0.042, 0.3);
    const before = shape(graph, id);
    // As drawing from its start does: a junction fixed there, which it and the fitted pipe both leave.
    const at = layoutGraph(ports(), graph).ducts.get(id)!;
    const loose = graph.ducts.find((d) => d.id === id)!;
    const node = 'join9';
    (graph.junctions ??= []).push({ node, position: [0.4, 0.2, 0.1], axis: [at.heading.x, at.heading.y, at.heading.z] });
    loose.from = { kind: 'node', node };
    const out = at.heading.clone().negate();
    const turn = headingOffsetTo(new THREE.Vector3(1, 0, 0), out);
    graph.ducts.push({ id: 'bend', segments: [], from: { kind: 'node', node }, to: { kind: 'mouth' }, headingYaw: turn.yaw, headingPitch: turn.pitch, headingFrame: 'world' });
    const end = attachToLooseStart(graph, 'bend', other, [0, 0, 1])!;
    const bend = graph.ducts.find((d) => d.id === 'bend')!;
    bend.segments = [makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.042 })];
    bend.fitted = true;
    refitBends(graph, ports(), spec);
    expect(end).toBeDefined();
    // Drawn this way, it still makes a graph the solver takes, and that survives being saved.
    expect(validateGraph(graph, 1)).toEqual([]);
    expect(validateGraph(graphFromJson(JSON.parse(JSON.stringify(graph)))!, 1)).toEqual([]);

    removePipe(graph, 'bend', layoutGraph(ports(), graph));
    expect(shape(graph, id)).toEqual(before);
    expect(graph.ducts.find((d) => d.id === other)!.from.kind).toBe('free');
    expect(graph.junctions ?? []).toEqual([]);
  });

  it('deleted from a pipe’s open end: the end open again, the pipe as it was', () => {
    const graph = bare();
    const id = placeLoosePipe(graph, [0.4, 0.2, 0.1], 0.042, 0.3);
    const before = shape(graph, id);
    fitRunner(graph, joinDuctEnd(graph, id)!);
    removePipe(graph, 'runner0', layoutGraph(ports(), graph));
    expect(shape(graph, id)).toEqual(before);
  });

  it('deleting a stretch of straight pipe takes only it, leaving a bend from it that joins something else loose', async () => {
    const { pipesMeetAt } = await import('../src/scene/exhaustLayout.js');
    const graph = bare();
    const a = placeLoosePipe(graph, [0.4, 0.2, 0.1], 0.042, 0.5);
    const b = placeLoosePipe(graph, [0.1, 0.4, 0.4], 0.042, 0.3);
    const bBefore = shape(graph, b);
    // The runner into a's side, and a bend from a's open end into b's start.
    const tee = splitDuctAt(graph, a, 0.2)!;
    fitRunner(graph, tee);
    const far = graph.ducts.find((d) => d.continues === a)!;
    const endNode = joinDuctEnd(graph, far.id)!;
    graph.ducts.push({ id: 'bend', segments: [], from: { kind: 'node', node: endNode }, to: { kind: 'mouth' } });
    attachToLooseStart(graph, 'bend', b, [0, 0, 1]);
    const bend = graph.ducts.find((d) => d.id === 'bend')!;
    bend.segments = [makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.042 })];
    bend.fitted = true;
    refitBends(graph, ports(), spec);
    const aBefore = shape(graph, a);
    expect(validateGraph(graph, 1)).toEqual([]);

    // The far half of a: it goes, and the bend from its end, still joined to b, stays, loose where it lies.
    const bendBefore = shape(graph, 'bend');
    const bendTo = { ...bend.to };
    removePipe(graph, far.id, layoutGraph(ports(), graph));
    refitBends(graph, ports(), spec);
    expect(graph.ducts.map((d) => d.id).sort()).toEqual([a, b, 'bend', 'runner0'].sort());
    expect(bend.from.kind).toBe('free');
    expect(bend.to).toEqual(bendTo);
    expect(bend.fitted).toBe(true);
    expect(shape(graph, 'bend').start).toEqual(bendBefore.start);
    // b stays on the bend, where it was.
    const bNow = shape(graph, b);
    expect(bNow.from).toEqual(bend.to);
    expect([bNow.start, bNow.end, bNow.lengths]).toEqual([bBefore.start, bBefore.end, bBefore.lengths]);
    // The near half stays as it was, and the runner still fitted where they meet, which stays put.
    expect(shape(graph, a).start).toEqual(aBefore.start);
    expect(shape(graph, a).end).toEqual(aBefore.end);
    const runner = graph.ducts.find((d) => d.id === 'runner0')!;
    expect(runner.fitted).toBe(true);
    expect(runner.to).toEqual({ kind: 'node', node: tee });
    expect(pipesMeetAt(graph, layoutGraph(ports(), graph), tee)).toBe(true);
    expect(validateGraph(graph, 1)).toEqual([]);
  });
});

describe('fitted pipes ending at the same junction', () => {
  const spec = { ...defaultConfig().engine, cylinders: 2, vAngle: 45, exhaustLayout: 'perBank' } as EngineSpec;
  const ports = (): ExhaustPort[] => {
    const mesh = new EngineMesh(spec);
    return [0, 1].map((i) => mesh.exhaustPort(i));
  };

  it('stay together, coming in the one way, however the pipe whose end it is turns', async () => {
    const { pipesMeetAt } = await import('../src/scene/exhaustLayout.js');
    const graph: ExhaustGraph = {
      ducts: [0, 1].map((i) => ({ id: `runner${i}`, segments: [makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.042 })], from: { kind: 'valve' as const, cylinder: i }, to: { kind: 'mouth' as const } })),
    };
    const loose = placeLoosePipe(graph, [0, -0.3, 0.2], 0.05, 0.3);
    const node = joinDuctEnd(graph, loose)!;
    for (const id of ['runner0', 'runner1']) {
      const r = graph.ducts.find((d) => d.id === id)!;
      r.segments.push(makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.042 }));
      r.fitted = true;
      r.to = { kind: 'node', node };
    }
    const pipe = graph.ducts.find((d) => d.id === loose)!;
    const yaw0 = pipe.headingYaw ?? 0;
    for (const turn of [0, 0.8, 1.6, Math.PI, -2.2]) {
      pipe.headingYaw = yaw0 + turn;
      refitBends(graph, ports(), spec);
      const placement = layoutGraph(ports(), graph);
      expect(pipesMeetAt(graph, placement, node)).toBe(true);
      const arrives = ['runner0', 'runner1'].map((id) => {
        const d = graph.ducts.find((x) => x.id === id)!;
        const p = placement.ducts.get(id)!;
        return layoutPipe(d.segments, p.origin, p.heading).jointDirections.at(-1)!;
      });
      expect(arrives[0]!.angleTo(arrives[1]!)).toBeLessThan(1e-6);
    }
  });
});

describe('deleting one of two fitted pipes that meet', () => {
  const spec = { ...defaultConfig().engine, cylinders: 2, vAngle: 45, exhaustLayout: 'perBank' } as EngineSpec;
  const ports = (): ExhaustPort[] => {
    const mesh = new EngineMesh(spec);
    return [0, 1].map((i) => mesh.exhaustPort(i));
  };
  /** Both runners bent into a junction fixed between them. */
  function merged(): { graph: ExhaustGraph; node: string } {
    const node = 'j';
    const graph: ExhaustGraph = {
      ducts: [0, 1].map((i) => ({
        id: `runner${i}`,
        segments: [makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.042 }), makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.042 })],
        from: { kind: 'valve' as const, cylinder: i },
        to: { kind: 'node' as const, node },
        fitted: true as const,
      })),
      junctions: [{ node, position: [0, -0.35, 0.25], axis: [0, -1, 0] }],
    };
    refitBends(graph, ports(), spec);
    return { graph, node };
  }

  it('leaves the other, still on its port, its bend kept as drawn', () => {
    const { graph } = merged();
    const other = graph.ducts.find((d) => d.id === 'runner0')!;
    const bent = structuredClone(other.segments);
    removePipe(graph, 'runner1', layoutGraph(ports(), graph));
    refitBends(graph, ports(), spec);
    expect(other.segments).toEqual(bent);
    expect(other.fitted).toBeUndefined();
    expect(other.to.kind).toBe('mouth');
    expect(validateGraph(graph, 2)).toEqual([]);
  });

  it('leaves a fitted pipe from where they met loose where it lies, still joined to what it bends into', () => {
    const { graph, node } = merged();
    const loose = placeLoosePipe(graph, [0.3, -0.5, 0.25], 0.05, 0.3);
    graph.ducts.splice(1, 1);
    graph.ducts.push({ id: 'on', segments: [makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.042 })], from: { kind: 'node', node }, to: { kind: 'mouth' } });
    attachToLooseStart(graph, 'on', loose, [1, 0, 0]);
    const on = graph.ducts.find((d) => d.id === 'on')!;
    on.fitted = true;
    refitBends(graph, ports(), spec);
    const was = layoutGraph(ports(), graph).ducts.get('on')!;
    const to = { ...on.to };
    removePipe(graph, 'runner0', layoutGraph(ports(), graph));
    expect(on.from.kind).toBe('free');
    expect(on.to).toEqual(to);
    expect(layoutGraph(ports(), graph).ducts.get('on')!.origin.distanceTo(was.origin)).toBeLessThan(1e-9);
  });
});

describe('deleting half of a pipe at a tee a fitted pipe comes into', () => {
  const spec = { ...defaultConfig().engine, cylinders: 1 } as EngineSpec;
  const ports = (): ExhaustPort[] => [new EngineMesh(spec).exhaustPort(0)];

  /** A runner bent into a loose pipe's side, square across it or along it; and the tee it made. */
  function tee(square: boolean) {
    const graph: ExhaustGraph = { ducts: [{ id: 'runner0', segments: [], from: { kind: 'valve', cylinder: 0 }, to: { kind: 'mouth' } }] };
    const id = placeLoosePipe(graph, [0.4, 0.2, -0.1], 0.042, 0.5);
    const node = splitDuctAt(graph, id, 0.25)!;
    const runner = graph.ducts[0]!;
    runner.segments = [makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.042 }), makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.042 })];
    runner.fitted = true;
    if (square) runner.square = true;
    runner.to = { kind: 'node', node };
    refitBends(graph, ports(), spec);
    return { graph, id, node, far: graph.ducts.find((d) => d.continues === id)! };
  }

  /** The turn between the runner's end and the way the pipe left at the junction runs there, radians. */
  function facing(graph: ExhaustGraph, node: string, left: string, leftEnds: boolean): number {
    const placement = layoutGraph(ports(), graph);
    expect(pipesMeetAt(graph, placement, node)).toBe(true);
    const swept = (d: string) => {
      const duct = graph.ducts.find((x) => x.id === d)!;
      const p = placement.ducts.get(d)!;
      return layoutPipe(duct.segments, p.origin, p.heading);
    };
    const pipe = leftEnds ? swept(left).jointDirections.at(-1)! : swept(left).stations[0]!.direction;
    return swept('runner0').jointDirections.at(-1)!.angleTo(pipe);
  }

  for (const square of [false, true]) {
    it(`deleting the half past it: the runner faces the ring at the end of what is left, and follows it${square ? ', though drawn in square' : ''}`, () => {
      const { graph, id, node, far } = tee(square);
      removePipe(graph, far.id, layoutGraph(ports(), graph));
      refitBends(graph, ports(), spec);
      expect(graph.junctions ?? []).toEqual([]);
      expect(facing(graph, node, id, true)).toBeLessThan(1e-6);
      // What is left moved and turned: the junction is its end, and the runner follows, facing it still.
      const pipe = graph.ducts.find((d) => d.id === id)!;
      pipe.from = { kind: 'free', position: [0.45, 0.25, -0.05] };
      pipe.headingYaw = (pipe.headingYaw ?? 0) + 0.6;
      refitBends(graph, ports(), spec);
      expect(facing(graph, node, id, true)).toBeLessThan(1e-6);
      expect(validateGraph(graph, 1)).toEqual([]);
    });

    it(`deleting the half before it: the runner faces the ring where what is left starts${square ? ', though drawn in square' : ''}`, () => {
      const { graph, id, node, far } = tee(square);
      removePipe(graph, id, layoutGraph(ports(), graph));
      refitBends(graph, ports(), spec);
      expect(facing(graph, node, far.id, false)).toBeLessThan(1e-6);
      expect(validateGraph(graph, 1)).toEqual([]);
    });
  }
});

describe('rolling a pipe from a segment further along', () => {
  it('rolls that segment and all after it about the way it sets off, leaving the pipe before it', () => {
    const origin = new THREE.Vector3(0.1, 0.2, 0.3);
    const heading = new THREE.Vector3(0, 0, 1);
    const up = new THREE.Vector3(0, 1, 0);
    const bend = bendSegment(new THREE.Vector3(), heading, up, 0.08, { dIn: 0.045, dOut: 0.045 })!;
    const duct: ExhaustGraph['ducts'][number] = {
      id: 'p',
      segments: [makeSegment({ kind: 'pipe', length: 0.2, dIn: 0.045 }), bend, makeSegment({ kind: 'pipe', length: 0.2, dIn: 0.045 })],
      from: { kind: 'free', position: [origin.x, origin.y, origin.z] },
      to: { kind: 'mouth' },
      headingYaw: 0,
      headingPitch: 0,
      headingFrame: 'world',
    };
    const before = layoutPipe(duct.segments, origin, heading);
    const pivot = before.joints[0]!;
    const shape = pipeShape(duct.segments, heading);
    swingPipe(duct, new THREE.Vector3(1, 0, 0), shape, shape.starts[1]!.clone(), Math.PI / 2, 1);
    const after = layoutPipe(duct.segments, origin, heading);
    // The pipe before the bend stays; everything from it on turns about the way the bend sets off.
    expect(after.joints[0]!.distanceTo(pivot)).toBeLessThan(1e-9);
    const turn = (v: THREE.Vector3) => v.clone().sub(pivot).applyAxisAngle(shape.starts[1]!, Math.PI / 2).add(pivot);
    expect(after.joints[1]!.distanceTo(turn(before.joints[1]!))).toBeLessThan(1e-6);
    expect(after.joints[2]!.distanceTo(turn(before.joints[2]!))).toBeLessThan(1e-6);
    expect(after.joints[2]!.distanceTo(before.joints[2]!)).toBeGreaterThan(0.05);
  });
});
