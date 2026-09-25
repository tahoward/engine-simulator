/**
 * The drawn exhaust must be geometrically possible.
 *
 * "The pipes just overlap when there's a collector" is a claim about solid bodies, so it can be checked
 * rather than looked at: sample every runner's centreline and confirm no two of them come closer than
 * the sum of their radii. That was the bug — every runner of a group was aimed at the *same* junction
 * point, and four 42 mm pipes cannot all occupy one place.
 *
 * Interpenetration is allowed in one place only: inside the weld, where a fabricated collector has the
 * pipe walls cut away and the body covers the join. Everywhere else it is a defect.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';

import { EngineMesh } from '../src/scene/EngineMesh.js';
import { freezeHeadings, layoutGraph, type ExhaustPlacement, type ExhaustPort } from '../src/scene/exhaustLayout.js';
import { layoutPipe } from '../src/scene/PipeMesh.js';
import {
  compileCollectorLayout,
  compileLayout,
  graphFromJson,
  splitDuctAt,
  type ExhaustGraph,
} from '../src/model/exhaustGraph.js';
import { buildJointGeometry, hubShape, jointDistance } from '../src/scene/jointMesh.js';
import { headingOffsetTo } from '../src/scene/drawing.js';
import {
  ENGINE_PRESETS,
  collectorGroups,
  defaultConfig,
  makeSegment,
  type EngineSpec,
  type PipeSegment,
} from '../src/model/spec.js';

/**
 * The engine's real exhaust ports.
 *
 * `EngineMesh` is used rather than reproduced. An earlier version of this test synthesised ports with
 * the bank taken as `i % 2`, which is not how the cylinders are arranged: a crossplane V8's banks follow
 * the firing order `[0,1,0,0,1,0,1,1]`, so cylinders 6 and 7 share a collector while `i % 2` put them on
 * opposite sides of the vee. The layout was then asked to drag two runners across the engine to meet,
 * and the test blamed the layout for it. Constructing the real thing needs no canvas.
 */
function makePorts(spec: EngineSpec): ExhaustPort[] {
  const mesh = new EngineMesh(spec, new THREE.Plane(new THREE.Vector3(0, 0, -1), 0.001));
  return Array.from({ length: mesh.bankCount }, (_, i) => mesh.exhaustPort(i));
}

interface Sampled {
  points: THREE.Vector3[];
  radii: number[];
  end: THREE.Vector3;
}

function sample(pipe: PipeSegment[], origin: THREE.Vector3, heading: THREE.Vector3): Sampled {
  const layout = layoutPipe(pipe, origin, heading);
  return {
    points: layout.stations.map((s) => s.position),
    radii: layout.stations.map((s) => s.radius),
    end: layout.joints[layout.joints.length - 1]!.clone(),
  };
}

/** Every duct of the graph, sampled where the layout put it. */
function sampleAll(graph: ExhaustGraph, placement: ExhaustPlacement): Map<string, Sampled> {
  const out = new Map<string, Sampled>();
  for (const duct of graph.ducts) {
    const place = placement.ducts.get(duct.id)!;
    out.set(duct.id, sample(duct.segments, place.origin, place.heading));
  }
  return out;
}

/**
 * Closest approach between two runners *outside the joint*, as a multiple of the clearance they need.
 *
 * Inside the joint they are allowed to meet, and must be: a fabricated collector has the pipe walls cut
 * away where they join and the joint covers it. Demanding clearance along the whole length is what forced
 * the bundle out to 201 mm for 44 mm pipes.
 *
 * "Inside the joint" used to be a slab along the junction's axis, because the body was a surface of
 * revolution and that was all it could mean. Now the joint is the union of the pipes themselves, so it is
 * simply a negative signed distance — the same question the mesh answers, asked of the same field.
 */
function clearanceRatio(
  a: Sampled,
  b: Sampled,
  inside?: (p: THREE.Vector3) => boolean,
): number {
  let worst = Infinity;
  for (let i = 0; i < a.points.length; i++) {
    if (inside?.(a.points[i]!)) continue;
    for (let j = 0; j < b.points.length; j++) {
      if (inside?.(b.points[j]!)) continue;
      const need = a.radii[i]! + b.radii[j]!;
      if (need <= 0) continue;
      const ratio = a.points[i]!.distanceTo(b.points[j]!) / need;
      if (ratio < worst) worst = ratio;
    }
  }
  return worst;
}

/**
 * Whether a point is where pipes are allowed to meet: inside a junction's fitting, or close enough to one
 * that it is a branch entering the side of the pipe it tees into.
 *
 * A tee's branch overlaps the through pipe for up to a pipe radius past its centreline — that overlap is
 * the branch entering the side of the other pipe, which is what a tee is — so the exemption reaches a
 * widest radius beyond the fitting.
 */
function weldOf(placement: ExhaustPlacement): (p: THREE.Vector3) => boolean {
  const joints = [...placement.joints.values()];
  const reach = joints.map((j) => Math.max(...j.limbs.map((l) => l.radius)));
  return (p: THREE.Vector3) => joints.some((j, i) => jointDistance(j, p) < reach[i]!);
}

/**
 * How far the joint's *mesh* reaches, and how far its limbs are spread.
 *
 * Measured off the built geometry rather than a described profile, because a described profile is what
 * went wrong: the old body reported a mouth radius and a length, and the mesh drawn from them was a
 * funnel nobody had asked for. What the renderer puts on screen is the only thing worth asserting on.
 */
function jointExtent(placement: ExhaustPlacement, node: string) {
  const joint = placement.joints.get(node)!;
  const geom = buildJointGeometry(joint)!;
  geom.computeBoundingBox();
  const size = geom.boundingBox!.getSize(new THREE.Vector3());

  let widest = 0;
  let spread = 0;
  for (const limb of joint.limbs) {
    widest = Math.max(widest, limb.radius);
    for (const other of joint.limbs) {
      spread = Math.max(spread, limb.point.distanceTo(other.point));
    }
  }
  const hub = hubShape(joint);
  return { joint, geom, span: Math.max(size.x, size.y, size.z), widest, spread, hub };
}

/** A representative collector, for sizing the body's throat. */
const COLLECTOR: PipeSegment[] = [
  makeSegment({ kind: 'cone', length: 0.18, dIn: 0.055, dOut: 0.065 }),
  makeSegment({ kind: 'pipe', length: 0.5, dIn: 0.065 }),
];

const CASES: Array<{ name: string; engine: Partial<EngineSpec>; pipe: () => PipeSegment[] }> = [
  {
    name: 'V8 per bank, straight runners',
    engine: { cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' },
    pipe: () => [makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.042 })],
  },
  {
    name: 'V8 per bank, bent runners',
    engine: { cylinders: 8, vAngle: 90, crankType: 'flatplane', exhaustLayout: 'perBank' },
    pipe: () => [
      makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.042, yaw: 0.3 }),
      makeSegment({ kind: 'cone', length: 0.15, dIn: 0.042, dOut: 0.05, pitch: -0.2 }),
    ],
  },
  {
    name: 'four into one',
    engine: { cylinders: 4, vAngle: 0, exhaustLayout: '2into1' },
    pipe: () => [makeSegment({ kind: 'pipe', length: 0.45, dIn: 0.038 })],
  },
  {
    name: 'V-twin, 2-into-1',
    engine: { cylinders: 2, vAngle: 45, exhaustLayout: '2into1' },
    pipe: () => [makeSegment({ kind: 'pipe', length: 0.34, dIn: 0.042 })],
  },
  {
    name: 'wide runners that only just fit',
    engine: { cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' },
    pipe: () => [makeSegment({ kind: 'pipe', length: 0.35, dIn: 0.062 })],
  },
];

function layoutOf(engine: Partial<EngineSpec>, segments: PipeSegment[]) {
  const spec = { ...defaultConfig().engine, ...engine } as EngineSpec;
  const groups = collectorGroups(spec);
  const ports = makePorts(spec);
  const graph = compileLayout(spec, segments, COLLECTOR);
  return { spec, groups, ports, graph, placement: layoutGraph(ports, graph) };
}

describe('exhaust layout is geometrically possible', () => {
  it.each(CASES)('$name: runners do not intersect', ({ engine, pipe }) => {
    const segments = pipe();
    const { groups, graph, placement } = layoutOf(engine, segments);
    const sampled = sampleAll(graph, placement);

    for (let i = 0; i < groups.length; i++) {
      for (let j = i + 1; j < groups.length; j++) {
        // Only runners sharing a junction can be aimed into each other.
        if (groups[i] !== groups[j] || groups[i]! < 0) continue;
        const ratio = clearanceRatio(
          sampled.get(`runner${i}`)!,
          sampled.get(`runner${j}`)!,
          weldOf(placement),
        );
        expect(
          ratio,
          `runners ${i} and ${j} approach to ${(ratio * 100).toFixed(0)}% of their combined radii`,
        ).toBeGreaterThan(1);
      }
    }
  });

  /**
   * No gaps, by construction — and the assertion says so.
   *
   * The joint is the union of the pipes that meet at it, each limb reaching back *inside* its own pipe
   * before it starts, so every pipe end is strictly within the solid. That is what makes a gap impossible
   * rather than merely unlikely, and a negative signed distance at each end is the statement of it. The
   * old body could only promise this by being wide enough to enclose everything, which is how a 2-into-1
   * ended up with a balloon on it.
   */
  it.each(CASES)('$name: every pipe end is inside its joint', ({ engine, pipe }) => {
    const segments = pipe();
    const { groups, graph, placement } = layoutOf(engine, segments);
    expect(placement.joints.size).toBeGreaterThan(0);

    for (const node of placement.joints.keys()) {
      const { joint, geom, span, widest, spread } = jointExtent(placement, node);
      expect(geom.getIndex()!.count, `${node} built nothing`).toBeGreaterThan(0);

      for (const [i, limb] of joint.limbs.entries()) {
        const d = jointDistance(joint, limb.point);
        expect(d, `${node} limb ${i} sits ${(d * 1000).toFixed(1)} mm outside the joint`).toBeLessThan(0);
      }

      /**
       * And bounded: a fitting sized to what arrives, not a funnel around how it approached.
       *
       * This is the assertion the tee failed when it was given a collector's body — 391 mm across a
       * joint between two 40 mm pipes.
       */
      const bound = 2.5 * (spread + widest * 2);
      expect(span, `${node} spans ${(span * 1000).toFixed(0)} mm over a ${(spread * 1000).toFixed(0)} mm spread`)
        .toBeLessThan(bound);

      /**
       * The fitting sits where the collector starts, so the collector comes out of it rather than the
       * fitting preceding it. Putting a body in front made the drawn exhaust longer than the one being
       * solved, which merges at a junction with no volume at all.
       */
      const group = Number(node.replace('merge', ''));
      const members = groups.flatMap((g, i) => (g === group ? [i] : []));
      if (members.length < 2) continue;
      const onward = placement.ducts.get(`collector${group}`)!;
      expect(joint.centre.distanceTo(onward.origin)).toBeLessThan(1e-9);
      const collector = graph.ducts.find((d) => d.id === `collector${group}`)!;
      const swept = layoutPipe(collector.segments, onward.origin, onward.heading);
      expect(jointDistance(joint, swept.stations[0]!.position)).toBeLessThan(0);
      expect(jointDistance(joint, swept.stations[swept.stations.length - 1]!.position)).toBeGreaterThan(0);
    }
  });

  /**
   * The editor's handles sit on one duct, placed by sweeping its segments from a frame the editor is
   * told about. That frame has to be the one the *mesh* used: with a collector the runner is aimed at
   * its collar, well away from the bare port axis, and using the port axis put the handles off the pipe
   * entirely. This asserts the gap is real, so the two cannot be casually conflated again.
   */
  it('a collector aims runners well away from the bare port axis', () => {
    const segments = [makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.042 })];
    const spec = { ...defaultConfig().engine, cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' } as EngineSpec;
    const ports = makePorts(spec);
    // The equal-length collector layout, whose runners are aimed at their collar.
    const placement = layoutGraph(ports, compileCollectorLayout(spec, segments, COLLECTOR));

    let worst = 0;
    for (let i = 0; i < ports.length; i++) {
      const place = placement.ducts.get(`runner${i}`)!;
      const alongPort = sample(segments, ports[i]!.position, ports[i]!.direction).end;
      const alongMesh = sample(segments, place.origin, place.heading).end;
      worst = Math.max(worst, alongPort.distanceTo(alongMesh));
    }
    // Tens of millimetres at least: far more than the handle spheres are wide.
    expect(worst).toBeGreaterThan(0.03);
  });

  it.each(ENGINE_PRESETS.map((p) => p.name))('preset lays out cleanly: %s', (name) => {
    const preset = ENGINE_PRESETS.find((p) => p.name === name)!;
    const segments = preset.pipe();
    const spec = { ...defaultConfig().engine, ...preset.engine } as EngineSpec;
    const groups = collectorGroups(spec);
    const ports = makePorts(spec);
    const graph = compileLayout(spec, segments, preset.collector ? preset.collector() : []);
    const placement = layoutGraph(ports, graph);
    const sampled = sampleAll(graph, placement);

    // Every duct got placed.
    for (const duct of graph.ducts) expect(placement.ducts.has(duct.id)).toBe(true);

    for (let i = 0; i < groups.length; i++) {
      for (let j = i + 1; j < groups.length; j++) {
        if (groups[i] !== groups[j] || groups[i]! < 0) continue;
        expect(
          clearanceRatio(
            sampled.get(`runner${i}`)!,
            sampled.get(`runner${j}`)!,
            weldOf(placement),
          ),
        ).toBeGreaterThan(1);
      }
    }
  });
});

/**
 * A merge of runners from opposite banks goes round the side of the engine, down the middle between them.
 *
 * With each bank's exhaust on the outside of the vee, a V-twin's ports face away from each other and their
 * mean heading is only the downward tilt they share — which aimed the collector through the crank. And
 * packed tightly, their ends were read as a tee, which sent the collector off along one runner.
 */
describe('a merge across the vee', () => {
  /**
   * A V-twin's runners meet at one point behind the engine, and one carries straight on into the collector
   * while the other joins its side.
   *
   * They converge at about 50 degrees, so meeting symmetrically they crossed each other for several
   * centimetres before the point, and the fitting big enough to hide that was a big cone. As a tee they
   * meet the way straight pipes do.
   */
  it('meets at a point, as a tee, along the crank rather than down into it', () => {
    const { graph, placement } = layoutOf(
      { cylinders: 2, vAngle: 45, exhaustLayout: '2into1' },
      [makeSegment({ kind: 'pipe', length: 0.34, dIn: 0.042 })],
    );
    const ends = ['runner0', 'runner1'].map((id) => {
      const place = placement.ducts.get(id)!;
      const duct = graph.ducts.find((d) => d.id === id)!;
      return layoutPipe(duct.segments, place.origin, place.heading);
    });
    expect(ends[0]!.joints.at(-1)!.distanceTo(ends[1]!.joints.at(-1)!)).toBeLessThan(1e-3);
    const onward = placement.ducts.get('collector0')!;
    expect(onward.heading.angleTo(ends[0]!.jointDirections.at(-1)!)).toBeLessThan(1e-6);
    expect(Math.abs(onward.heading.z)).toBeGreaterThan(0.8);
    expect(hubShape(placement.joints.get('merge0')!).kind).toBe('ball');
  });
});

/**
 * A bank's manifold: the ports chained by straight tube, each cylinder joining through a short stub.
 */
describe('a manifold along each bank', () => {
  const v8 = { cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' } as Partial<EngineSpec>;

  it('reaches every junction exactly, and fits each with a ball', () => {
    const { graph, placement } = layoutOf(v8, [makeSegment({ kind: 'pipe', length: 0.45, dIn: 0.042 })]);
    for (const d of graph.ducts) {
      if (d.to.kind !== 'node') continue;
      const place = placement.ducts.get(d.id)!;
      const end = layoutPipe(d.segments, place.origin, place.heading).joints.at(-1)!;
      expect(end.distanceTo(placement.joints.get(d.to.node)!.centre), d.id).toBeLessThan(1e-6);
    }
    for (const joint of placement.joints.values()) expect(hubShape(joint).kind).toBe('ball');
  });

  it('runs its outlet on along the bank', () => {
    const { placement } = layoutOf(v8, [makeSegment({ kind: 'pipe', length: 0.45, dIn: 0.042 })]);
    for (const g of [0, 1]) expect(Math.abs(placement.ducts.get(`collector${g}`)!.heading.z)).toBeGreaterThan(0.999);
  });
});

/**
 * A tri-Y merges in two stages, so its middle ducts are fed by a junction *and* feed another one.
 * Placing it requires resolving the nodes in topological order: the final junction sits at the centroid
 * of the middle ducts' ends, so it cannot be placed until they are.
 */
describe('a two-stage merge places in the right order', () => {
  function triY(): ExhaustGraph {
    const runner = (i: number, node: string) => ({
      id: `runner${i}`,
      segments: [makeSegment({ kind: 'pipe' as const, length: 0.4, dIn: 0.038 })],
      from: { kind: 'valve' as const, cylinder: i },
      to: { kind: 'node' as const, node },
    });
    const mid = (id: string, from: string) => ({
      id,
      segments: [makeSegment({ kind: 'pipe' as const, length: 0.3, dIn: 0.05 })],
      from: { kind: 'node' as const, node: from },
      to: { kind: 'node' as const, node: 'tail' },
    });
    return {
      ducts: [
        runner(0, 'pairA'),
        runner(1, 'pairB'),
        runner(2, 'pairA'),
        runner(3, 'pairB'),
        mid('midA', 'pairA'),
        mid('midB', 'pairB'),
        {
          id: 'tailpipe',
          segments: [makeSegment({ kind: 'pipe', length: 0.5, dIn: 0.06 })],
          from: { kind: 'node', node: 'tail' },
          to: { kind: 'mouth' },
        },
      ],
    };
  }

  it('places every duct, with a body at all three junctions', () => {
    const spec = { ...defaultConfig().engine, cylinders: 4, vAngle: 0 } as EngineSpec;
    const graph = triY();
    const placement = layoutGraph(makePorts(spec), graph);

    for (const duct of graph.ducts) expect(placement.ducts.has(duct.id)).toBe(true);
    expect([...placement.joints.keys()].sort()).toEqual(['pairA', 'pairB', 'tail']);

    // The final junction sits where the middle ducts finish, downstream of the first two.
    const tail = placement.joints.get('tail')!;
    const pairA = placement.joints.get('pairA')!;
    expect(tail.centre.dot(tail.axis)).toBeGreaterThan(pairA.centre.dot(tail.axis));
    // The tailpipe starts at the last junction, not before it.
    expect(placement.ducts.get('tailpipe')!.origin.distanceTo(tail.centre)).toBeLessThan(1e-9);
    // Each middle duct starts at its own first-stage junction.
    expect(placement.ducts.get('midA')!.origin.distanceTo(pairA.centre)).toBeLessThan(1e-9);
  });

  /** A drawn duct keeps the heading it was drawn with instead of being aimed. */
  it('honours a stored heading instead of aiming', () => {
    const spec = { ...defaultConfig().engine, cylinders: 4, vAngle: 0 } as EngineSpec;
    const ports = makePorts(spec);
    const aimed = layoutGraph(ports, triY()).ducts.get('runner0')!.heading.clone();

    const drawnGraph = triY();
    drawnGraph.ducts[0]!.headingYaw = 0.6;
    drawnGraph.ducts[0]!.headingPitch = -0.25;
    const drawn = layoutGraph(ports, drawnGraph).ducts.get('runner0')!.heading;

    // Turned off its port by the stored amount, and no longer wherever the collar wanted it.
    expect(drawn.angleTo(aimed)).toBeGreaterThan(0.1);
    expect(drawn.angleTo(ports[0]!.direction)).toBeGreaterThan(0.1);
    expect(drawn.length()).toBeCloseTo(1, 9);
  });
});

/**
 * A duct with no segments at all.
 *
 * Reachable by deleting the last segment of a runner, and with the runners linked that empties every one
 * of them at once. `layoutPipe` sweeps per segment, so an empty list yields no joints — and the obvious
 * `joints[joints.length - 1]!` threw. The non-null assertion is what let it through: `layoutPipe` makes no
 * promise of a joint, `pipeSpan` and `solveHeading` both guard for it, and `sampleRunner` was the one
 * caller that assumed one.
 */
describe('a duct with no segments', () => {
  const v8 = { ...defaultConfig().engine, cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' } as EngineSpec;

  it('lays out without throwing, however much is missing', () => {
    const cases: Array<[string, PipeSegment[], PipeSegment[]]> = [
      ['every runner empty', [], [makeSegment({ length: 0.5, dIn: 0.055 })]],
      ['the collector empty', [makeSegment({ length: 0.4, dIn: 0.042 })], []],
      ['everything empty', [], []],
    ];
    for (const [name, pipe, collector] of cases) {
      const graph = compileLayout(v8, pipe, collector);
      expect(() => layoutGraph(makePorts(v8), graph), name).not.toThrow();
      const placement = layoutGraph(makePorts(v8), graph);
      // Still placed, so the renderer has somewhere to put every mesh.
      for (const duct of graph.ducts) expect(placement.ducts.has(duct.id)).toBe(true);
    }
  });

  it('a runner that spans nothing ends where it starts', () => {
    const graph = compileLayout(v8, [], [makeSegment({ length: 0.5 })]);
    const ports = makePorts(v8);
    const placement = layoutGraph(ports, graph);
    const place = placement.ducts.get('runner0')!;
    expect(place.origin.distanceTo(ports[0]!.position)).toBeLessThan(1e-9);
    expect(layoutPipe([], place.origin, place.heading).joints).toHaveLength(0);
  });
});

/**
 * A joint is the size of the pipes at it, whatever kind of joint it is.
 *
 * There is no longer a case split. A tee and a 4-into-1 are built the same way — the union of the pipes
 * that meet, filleted — so a tee comes out the size of two 40 mm pipes and a collector comes out the size
 * of the bundle it gathers, with nothing deciding which is which. The earlier code had to choose: it drew
 * a surface of revolution sized to *enclose* the feeds, which gave the tee a 391 mm mouth over 720 mm
 * because a steeply-arriving branch crosses the socket zone far off-axis, and the fix was to suppress the
 * body at a tee entirely. Both the funnel and the special case are gone; these tests hold the sizes.
 */
describe('joints are shaped like the joint they are', () => {
  const spec = { ...defaultConfig().engine, cylinders: 2, vAngle: 45, exhaustLayout: '2into2' } as EngineSpec;
  const ports = () => makePorts(spec);
  const RUNNER = () => [makeSegment({ kind: 'pipe', length: 0.6, dIn: 0.04 })];

  /** Branch cylinder 2's runner onto cylinder 1's pipe, landing exactly where it was snapped. */
  function teed(onward?: PipeSegment[]) {
    const graph = compileLayout(spec, RUNNER(), []);
    const p = ports();
    const place = layoutGraph(p, graph).ducts.get('runner0')!;
    const swept = layoutPipe(graph.ducts[0]!.segments, place.origin, place.heading);
    const station = swept.stations.find((st) => st.x >= 0.3)!;

    const node = splitDuctAt(graph, 'runner0', 0.3)!;
    if (onward) {
      graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === node)!.segments = onward;
    }
    const branch = graph.ducts.find((d) => d.id === 'runner1')!;
    const dir = station.position.clone().sub(p[1]!.position);
    const turn = headingOffsetTo(p[1]!.direction, dir.clone().normalize());
    branch.segments = [makeSegment({ kind: 'pipe', length: dir.length(), dIn: 0.04 })];
    branch.to = { kind: 'node', node };
    branch.headingYaw = turn.yaw;
    branch.headingPitch = turn.pitch;

    return { graph, ports: p, node, through: station.direction.clone() };
  }

  it('a tee into a pipe of the same size stays the size of the pipes', () => {
    const { graph, ports: p, node } = teed();
    const placement = layoutGraph(p, graph);
    expect(placement.joints.has(node)).toBe(true);
    const { span, widest } = jointExtent(placement, node);
    // Two 40 mm pipes meeting at a point: tens of millimetres, not the 391 mm the funnel gave.
    expect(span).toBeLessThan(0.12);
    expect(span).toBeGreaterThan(widest * 2);
  });

  it('and the pipe carries on straight through it', () => {
    const { graph, ports: p, node, through } = teed();
    const placement = layoutGraph(p, graph);
    const onward = graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === node)!;
    const heading = placement.ducts.get(onward.id)!.heading;
    // Following the through pipe, not the average of it and its branch.
    expect((heading.angleTo(through) * 180) / Math.PI).toBeLessThan(1);
  });

  it('merging into a wider pipe grows only by the wider pipe', () => {
    // A collector sized for the two feeds' combined area, as `joinDuctEnd` would make.
    const wide = Math.sqrt(2) * 0.04;
    const { graph, ports: p, node } = teed([makeSegment({ kind: 'pipe', length: 0.4, dIn: wide })]);
    const placement = layoutGraph(p, graph);
    const { span, widest, joint } = jointExtent(placement, node);
    expect(widest).toBeCloseTo(wide / 2, 9);
    // Still a collar around the pipes, not a funnel around how they approached.
    expect(span).toBeLessThan(0.15);
    // And the wider outlet is one of the limbs, so the joint follows it out.
    expect(joint.limbs.some((l) => Math.abs(l.radius - wide / 2) < 1e-9)).toBe(true);
  });

  /** A collector spans its bundle: its runners are spread, so the joint has to reach all of them. */
  /** The equal-length layout still gathers four runners into one collector, which must take them all. */
  it('a collector spans the runners it gathers', () => {
    const merged = { ...defaultConfig().engine, cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' } as EngineSpec;
    const graph = compileCollectorLayout(merged, [makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.042 })], COLLECTOR);
    const placement = layoutGraph(makePorts(merged), graph);
    const { span, spread } = jointExtent(placement, 'merge0');
    // Wide enough to bundle four runners, which is the whole point of it.
    expect(span).toBeGreaterThan(0.12);
    expect(span).toBeGreaterThan(spread);
  });
});

/**
 * A pipe teed into another has to run through the joint unbroken.
 *
 * The joint used to sit at the *centroid* of the feed ends, which is right for a collector and wrong here:
 * the branch finishes on the pipe's surface while the pipe's own two halves meet on its centreline, so the
 * average lands half a radius off the axis. Measured as an 11.6 mm step in a pipe of 20 mm radius, showing
 * as a kink exactly where the pipe should be continuous.
 */
describe('a tee runs through unbroken', () => {
  const spec = { ...defaultConfig().engine, cylinders: 2, vAngle: 45, exhaustLayout: '2into2' } as EngineSpec;

  /** Tee cylinder 2's runner into cylinder 1's pipe, aiming `aim` of the way in from the skin. */
  function teeInto(aim: number) {
    const graph = compileLayout(spec, [makeSegment({ kind: 'pipe', length: 0.6, dIn: 0.04 })], []);
    const p = makePorts(spec);
    const place = layoutGraph(p, graph).ducts.get('runner0')!;
    const swept = layoutPipe(graph.ducts[0]!.segments, place.origin, place.heading);
    const station = swept.stations.find((st) => st.x >= 0.3)!;
    // `aim` of 1 is the centreline, 0 the skin.
    const target = station.position
      .clone()
      .add(new THREE.Vector3(0, station.radius * (1 - aim), 0));

    const node = splitDuctAt(graph, 'runner0', 0.3)!;
    const branch = graph.ducts.find((d) => d.id === 'runner1')!;
    const dir = target.clone().sub(p[1]!.position);
    const turn = headingOffsetTo(p[1]!.direction, dir.clone().normalize());
    branch.segments = [makeSegment({ kind: 'pipe', length: dir.length(), dIn: 0.04 })];
    branch.to = { kind: 'node', node };
    branch.headingYaw = turn.yaw;
    branch.headingPitch = turn.pitch;

    const placement = layoutGraph(p, graph);
    const upstream = graph.ducts.find((d) => d.id === 'runner0')!;
    const upPlace = placement.ducts.get('runner0')!;
    const upSwept = layoutPipe(upstream.segments, upPlace.origin, upPlace.heading);
    const onward = graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === node)!;
    return {
      node,
      placement,
      step: upSwept.joints[upSwept.joints.length - 1]!.distanceTo(
        placement.ducts.get(onward.id)!.origin,
      ),
      throughDir: upSwept.jointDirections[upSwept.jointDirections.length - 1]!,
      onwardHeading: placement.ducts.get(onward.id)!.heading,
    };
  }

  it('no step where the two halves meet', () => {
    const { step } = teeInto(1);
    expect(step).toBeLessThan(1e-9);
  });

  it('and the halves point the same way', () => {
    const { throughDir, onwardHeading } = teeInto(1);
    expect((onwardHeading.angleTo(throughDir) * 180) / Math.PI).toBeLessThan(0.5);
  });

  /**
   * Still recognised as a tee when the branch stops on the skin rather than reaching the axis.
   *
   * Draw mode aims at the axis now, but a route built any other way — an older saved link, say — must not
   * turn into a collector because its branch is one radius short.
   */
  it('recognises a tee even when the branch stops on the skin', () => {
    const { node, placement, step } = teeInto(0);
    expect(step).toBeLessThan(1e-9);
    // Still joint-sized rather than collector-sized, so the branch is welded on, not funnelled into.
    expect(jointExtent(placement, node).span).toBeLessThan(0.12);
  });
});

/**
 * What a page refresh does to the exhaust: the config goes out to the URL as JSON and comes back through
 * `graphFromJson`. Anything the loader drops lays the pipes out differently — the bug this guards against
 * dropped each frozen heading's frame, so every pipe after a junction turned on reload.
 */
describe('the exhaust survives a reload', () => {
  const reloaded = (graph: ExhaustGraph) => graphFromJson(JSON.parse(JSON.stringify(graph)))!;

  function expectSameLayout(ports: ExhaustPort[], a: ExhaustGraph, b: ExhaustGraph): void {
    const before = layoutGraph(ports, a);
    const after = layoutGraph(ports, b);
    for (const duct of a.ducts) {
      const p = before.ducts.get(duct.id)!;
      const q = after.ducts.get(duct.id)!;
      expect(q.origin.distanceTo(p.origin), `${duct.id} origin`).toBeLessThan(1e-9);
      expect(q.heading.distanceTo(p.heading), `${duct.id} heading`).toBeLessThan(1e-9);
    }
    for (const [node, joint] of before.joints) {
      expect(after.joints.get(node)!.centre.distanceTo(joint.centre), `${node}`).toBeLessThan(1e-9);
    }
  }

  it.each(ENGINE_PRESETS.map((p) => p.name))('as compiled: %s', (name) => {
    const preset = ENGINE_PRESETS.find((p) => p.name === name)!;
    const spec = { ...defaultConfig().engine, ...preset.engine } as EngineSpec;
    const graph = compileLayout(spec, preset.pipe(), preset.collector ? preset.collector() : []);
    expect(reloaded(graph)).toEqual(graph);
    expectSameLayout(makePorts(spec), graph, reloaded(graph));
  });

  it.each(ENGINE_PRESETS.map((p) => p.name))('after an edit has frozen its headings: %s', (name) => {
    const preset = ENGINE_PRESETS.find((p) => p.name === name)!;
    const spec = { ...defaultConfig().engine, ...preset.engine } as EngineSpec;
    const ports = makePorts(spec);
    const graph = compileLayout(spec, preset.pipe(), preset.collector ? preset.collector() : []);
    // What the app does before any edit: pin every heading where the layout put it.
    freezeHeadings(graph, layoutGraph(ports, graph), ports);
    graph.ducts[0]!.segments[0]!.yaw += 0.3;
    expect(reloaded(graph)).toEqual(graph);
    expectSameLayout(ports, graph, reloaded(graph));
  });
});
