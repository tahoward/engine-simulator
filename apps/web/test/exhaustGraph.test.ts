/**
 * The exhaust graph as the interface edits it: labels, linked runners, and what a change of preset
 * carries across.
 *
 * Compiling, validating and solving a graph are the simulation's, and are tested with it, in
 * `crates/engine-sim/tests/exhaust_graph.rs`. These are the editing functions in
 * `src/model/exhaustGraph.ts` that only the web app has.
 */

import { describe, expect, it } from 'vitest';

import { Sim } from '../src/audio/worklet/sim.js';
import {
  carryBore,
  compileCollectorLayout,
  compileLayout,
  copyToSiblingRunners,
  ductLabel,
  endsAt,
  nodeOrder,
  type ExhaustDuct,
  type ExhaustGraph,
} from '../src/model/exhaustGraph.js';
import { ENGINE_PRESETS, defaultConfig, makeSegment, segmentDiameter, type EngineSpec } from '../src/model/spec.js';

const FS = 48000;
const specOf = (partial: Partial<EngineSpec>): EngineSpec =>
  ({ ...defaultConfig().engine, ...partial }) as EngineSpec;

describe('labelling ducts', () => {
  it('labels ducts by where they start', () => {
    const spec = specOf({ cylinders: 2, exhaustLayout: '2into1' });
    const graph = compileLayout(spec, [makeSegment({})], [makeSegment({})]);
    const labels = graph.ducts.map((d) => ductLabel(graph, d));
    expect(labels).toEqual(['Cylinder 1 primary', 'Cylinder 2 primary', 'After junction 1']);
  });

  it('calls a cylinder’s duct a stub on a manifold and its exhaust when it goes straight out', () => {
    const four = specOf({ cylinders: 4, vAngle: 0, exhaustLayout: 'merged' });
    const manifold = compileLayout(four, [makeSegment({})], [makeSegment({})]);
    const first = manifold.ducts.find((d) => d.from.kind === 'valve' && d.from.cylinder === 0)!;
    expect(ductLabel(manifold, first)).toBe(first.role === 'stub' ? 'Cylinder 1 stub' : 'Cylinder 1 primary');
    expect(manifold.ducts.some((d) => ductLabel(manifold, d).endsWith('stub'))).toBe(true);

    const open = specOf({ cylinders: 1, exhaustLayout: 'open' });
    const single = compileLayout(open, [makeSegment({})], []);
    expect(ductLabel(single, single.ducts[0]!)).toBe('Cylinder 1 exhaust');
  });
});

describe('linking runners', () => {
  const v8 = () =>
    specOf({ cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' });

  it('copies one runner onto the rest and leaves collectors alone', () => {
    const graph = compileCollectorLayout(v8(), [makeSegment({ length: 0.4, dIn: 0.042 })], [makeSegment({ length: 0.6, dIn: 0.055 })]);
    const runner0 = graph.ducts.find((d) => d.id === 'runner0')!;
    runner0.segments.push(makeSegment({ length: 0.2, dIn: 0.05 }));
    runner0.segments[0]!.length = 0.33;

    copyToSiblingRunners(graph, runner0);

    for (const d of graph.ducts) {
      if (d.from.kind !== 'valve') continue;
      expect(d.segments.map((sg) => [sg.length, sg.dIn])).toEqual(
        runner0.segments.map((sg) => [sg.length, sg.dIn]),
      );
      // Copies, not the same objects, or editing one would silently edit them all.
      if (d !== runner0) expect(d.segments[0]).not.toBe(runner0.segments[0]);
    }
    expect(graph.ducts.find((d) => d.id === 'collector0')!.segments[0]!.length).toBeCloseTo(0.6, 9);
  });

  it('copies a pipe drawn from a port onto every cylinder, heading and all', () => {
    const graph = compileCollectorLayout(v8(), [makeSegment({ length: 0.4 })], [makeSegment({ length: 0.6 })]);
    graph.ducts = graph.ducts.filter((d) => d.id !== 'runner0');
    const drawn = { id: 'drawn1', segments: [], from: { kind: 'valve', cylinder: 0 }, to: { kind: 'mouth' } } as ExhaustDuct;
    graph.ducts.push(drawn);

    // Just started, nothing drawn yet: the other cylinders keep their pipes.
    copyToSiblingRunners(graph, drawn);
    expect(graph.ducts.find((d) => d.id === 'runner1')!.segments[0]!.length).toBeCloseTo(0.4, 9);

    drawn.headingYaw = 0;
    drawn.headingPitch = 0.3;
    drawn.segments.push(makeSegment({ length: 0.15 }), makeSegment({ length: 0.25, yaw: 0.5 }));
    copyToSiblingRunners(graph, drawn);
    for (const d of graph.ducts) {
      if (d.from.kind !== 'valve') continue;
      expect(d.segments.map((sg) => [sg.length, sg.yaw])).toEqual([[0.15, 0], [0.25, 0.5]]);
      expect([d.headingYaw, d.headingPitch]).toEqual([0, 0.3]);
    }
  });

  it('empties every runner when a delete empties one', () => {
    const graph = compileCollectorLayout(v8(), [makeSegment({ length: 0.4 })], [makeSegment({ length: 0.6 })]);
    const runner0 = graph.ducts.find((d) => d.id === 'runner0')!;
    runner0.segments = [];
    copyToSiblingRunners(graph, runner0, v8(), true);
    for (const d of graph.ducts) if (d.from.kind === 'valve') expect(d.segments).toEqual([]);
  });

  it.each([
    ['a V8', () => v8()],
    ['a boxer four', () => specOf({ cylinders: 4, vAngle: 180, crankType: 'boxer', exhaustLayout: 'perBank' })],
  ])('gives the other bank of %s the mirror image, bends and all', async (_n, engine) => {
    const THREE = await import('three');
    const { layoutPipe, turnHeading } = await import('../src/scene/PipeMesh.js');
    const { bendWhole } = await import('../src/scene/drawing.js');
    const { exhaustPortOf } = await import('../src/model/geometry.js');
    const { physicalBank } = await import('../src/model/spec.js');
    const spec = engine();
    const graph = compileCollectorLayout(spec, [makeSegment({ length: 0.4 })], [makeSegment({ length: 0.6 })]);
    const source = graph.ducts.find((d) => d.id === 'runner0')!;
    source.headingYaw = 0.4;
    source.headingPitch = -0.2;
    const straight = makeSegment({ length: 0.3 });
    // Bent towards the front and down, off the way it leaves the port.
    const port0 = new THREE.Vector3(...exhaustPortOf(spec, 0).direction);
    const leaving = turnHeading(port0, 0.4, -0.2);
    const bent = bendWhole(straight, leaving, new THREE.Vector3(1, 0.5, 0.3).normalize(), 1.1, 0.06).segment;
    source.segments = [makeSegment({ length: 0.1 }), bent, makeSegment({ length: 0.2, yaw: 0.3, pitch: 0.1 })];
    copyToSiblingRunners(graph, source, spec);

    // Each runner's shape from its own port, as the layout lays it.
    const shape = (d: ExhaustDuct) => {
      const port = exhaustPortOf(spec, (d.from as { cylinder: number }).cylinder);
      const origin = new THREE.Vector3(...port.position);
      const heading = turnHeading(new THREE.Vector3(...port.direction), d.headingYaw, d.headingPitch);
      return layoutPipe(d.segments, origin, heading).joints.map((p) => p.clone().sub(origin));
    };
    const mine = shape(source);
    const runners = graph.ducts.filter((d) => d.from.kind === 'valve');
    let opposite = 0;
    for (const d of runners) {
      const theirs = shape(d);
      const other = physicalBank(spec, (d.from as { cylinder: number }).cylinder) !== physicalBank(spec, 0);
      if (other) opposite++;
      theirs.forEach((p, i) => {
        const want = other ? new THREE.Vector3(-mine[i]!.x, mine[i]!.y, mine[i]!.z) : mine[i]!;
        expect(p.distanceTo(want)).toBeLessThan(1e-9);
      });
    }
    expect(opposite).toBe(spec.cylinders / 2);
  });

  it('leaves each runner its own bore where it meets its junction', () => {
    const graph = compileCollectorLayout(v8(), [makeSegment({ length: 0.4, dIn: 0.042 })], [makeSegment({ length: 0.6, dIn: 0.055 })]);
    const runner0 = graph.ducts.find((d) => d.id === 'runner0')!;
    runner0.segments[0]!.dIn = 0.046;
    runner0.segments[0]!.dOut = 0.05;
    copyToSiblingRunners(graph, runner0, v8());
    for (const d of graph.ducts) {
      if (d.from.kind !== 'valve' || d === runner0) continue;
      // The rest of it is the copy; the end, at its junction, is as it was.
      expect(d.segments[0]!.dIn).toBeCloseTo(0.046, 9);
      expect(d.segments[0]!.dOut).toBeCloseTo(0.042, 9);
    }
  });

  it('does nothing when asked to mirror a collector', () => {
    const graph = compileCollectorLayout(v8(), [makeSegment({ length: 0.4 })], [makeSegment({ length: 0.6 })]);
    const collector = graph.ducts.find((d) => d.id === 'collector0')!;
    collector.segments[0]!.length = 0.9;
    copyToSiblingRunners(graph, collector);
    expect(graph.ducts.find((d) => d.id === 'runner0')!.segments[0]!.length).toBeCloseTo(0.4, 9);
  });

  /**
   * The audible proof: editing one runner with linking on is the same engine as compiling the edited
   * geometry from scratch. If it were not, "apply to every cylinder" would be a different exhaust from
   * one with the same runner on every cylinder.
   */
  it('sounds the same as compiling the edited geometry directly', () => {
    const spec = { ...v8(), rpm: 3800, throttle: 0.85, freeRunning: false } as EngineSpec;
    // Ending at the bore it met the collector at, which is the collector's own and stays each runner's.
    const edited = [makeSegment({ length: 0.33, dIn: 0.044, dOut: 0.042 })];
    const collector = [makeSegment({ length: 0.55, dIn: 0.058 })];

    const render = (graph: ExhaustGraph) => {
      const cfg = defaultConfig();
      cfg.engine = spec;
      cfg.graph = graph;
      const sim = new Sim(FS, cfg);
      sim.render(FS / 4);
      return sim.render(FS / 4);
    };

    const compiled = render(compileCollectorLayout(spec, edited, collector));

    const mirrored = compileCollectorLayout(spec, [makeSegment({ length: 0.4, dIn: 0.042 })], collector);
    const first = mirrored.ducts.find((d) => d.id === 'runner0')!;
    first.segments = edited.map((sg) => makeSegment(sg));
    copyToSiblingRunners(mirrored, first);

    const viaLink = render(mirrored);
    for (let i = 0; i < compiled.length; i++) {
      expect(Object.is(compiled[i], viaLink[i])).toBe(true);
    }
  });
});

describe('compileLayout builds manifolds', () => {
  const v8spec = () => specOf({ cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' });

  /** Linking runners must not copy the manifold's first length onto the stubs, or a stub over it. */
  it('keeps linked runners off the one that carries the manifold', () => {
    const graph = compileLayout(v8spec(), [makeSegment({ length: 0.45 })], [makeSegment({ length: 0.6 })]);
    const stub = graph.ducts.find((d) => d.from.kind === 'valve' && d.segments.length === 1)!;
    stub.segments[0]!.length = 0.12;
    copyToSiblingRunners(graph, stub);
    for (const d of graph.ducts.filter((x) => x.from.kind === 'valve')) {
      if (d.segments.length === 2) expect(d.segments[0]!.length).toBeCloseTo(0.1, 9);
      else expect(d.segments[0]!.length).toBeCloseTo(0.12, 9);
    }
  });
});

describe('a manifold meets what it carries on into', () => {
  /**
   * Along a manifold the gas carries on through each junction into the next length, the downpipe and the
   * collector: each starts at the bore the one before it ends at and widens from there, with no step.
   */
  it.each([
    ['an inline four', { cylinders: 4, exhaustLayout: 'merged', vAngle: 0 }],
    ['an inline six', { cylinders: 6, exhaustLayout: 'merged', vAngle: 0 }],
    ['a V8 per bank', { cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' }],
    ['a V6 merged', { cylinders: 6, vAngle: 60, exhaustLayout: 'merged' }],
  ] as Array<[string, Partial<EngineSpec>]>)('on %s', (_, partial) => {
    // A collector wider than the manifold, as a V6's is after its downpipes.
    const collector = [makeSegment({ length: 0.65, dIn: 0.0612, dOut: 0.0612 })];
    const graph = compileLayout(specOf(partial), [makeSegment({ length: 0.05, dIn: 0.0384, dOut: 0.0384 })], collector);
    const carriedOn = graph.ducts.filter((d) => d.role === 'manifold' || d.role === 'downpipe' || d.role === 'collector');
    for (const node of nodeOrder(graph)) {
      const ends = endsAt(graph, node);
      for (const out of ends.filter((e) => e.end === 'inlet' && carriedOn.includes(e.duct))) {
        const into = out.duct.continues
          ? ends.find((e) => e.end === 'outlet' && e.duct.id === out.duct.continues)
          : ends.find((e) => e.end === 'outlet' && e.duct.role === 'downpipe');
        if (!into) continue;
        const arriving = segmentDiameter(into.duct.segments.at(-1)!, 1);
        expect(segmentDiameter(out.duct.segments[0]!, 0), `${into.duct.id} into ${out.duct.id}`).toBeCloseTo(arriving, 9);
      }
    }
  });
});

describe('switching presets', () => {
  it('carries a manifold’s collector across, and not its stubs or lengths', async () => {
    const { carriedGeometry } = await import('../src/model/exhaustGraph.js');
    const collector = [makeSegment({ kind: 'cone', length: 0.16, dIn: 0.062, dOut: 0.072 })];
    const v8 = specOf({ cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' });
    const graph = compileLayout(v8, [makeSegment({ length: 0.5, dIn: 0.044 })], collector);
    const carried = carriedGeometry(graph);
    expect(carried.pipe).toBeUndefined();
    // The collector itself — opening at the manifold's bore rather than narrower, which compiling it
    // again leaves as it is.
    const [cone] = carried.collector!;
    expect([cone!.kind, cone!.length, cone!.dOut]).toEqual(['cone', 0.16, 0.072]);
    expect(cone!.dIn).toBeGreaterThanOrEqual(0.062);
    expect(carriedGeometry(compileLayout(v8, [makeSegment({ length: 0.5, dIn: 0.044 })], carried.collector!)).collector)
      .toEqual(carried.collector);
  });

  it('carries a runner where there is one', async () => {
    const { carriedGeometry } = await import('../src/model/exhaustGraph.js');
    const graph = compileLayout(specOf({ cylinders: 2, exhaustLayout: '2into2' }), [makeSegment({ length: 0.61 })], []);
    expect(carriedGeometry(graph).pipe![0]!.length).toBeCloseTo(0.61, 12);
  });

  /** The same sequence of calls the panel and `main` make when a preset is picked. */
  it('crossplane, flatplane, crossplane: the same exhaust both times', async () => {
    const { carriedGeometry } = await import('../src/model/exhaustGraph.js');
    const topology = ['cylinders', 'exhaustLayout', 'crankType', 'firingOffset', 'vAngle'];
    const cfg = defaultConfig();
    cfg.graph = compileLayout(cfg.engine, cfg.pipe, cfg.collector);
    const pick = (name: string) => {
      const preset = ENGINE_PRESETS.find((p) => p.name.startsWith(name))!;
      Object.assign(cfg.engine, preset.engine);
      if (Object.keys(preset.engine).some((k) => topology.includes(k))) {
        const carried = carriedGeometry(cfg.graph!);
        if (carried.pipe) cfg.pipe = carried.pipe;
        if (carried.collector) cfg.collector = carried.collector;
        cfg.graph = compileLayout(cfg.engine, cfg.pipe, cfg.collector);
      }
      cfg.pipe = preset.pipe();
      cfg.collector = preset.collector ? preset.collector() : [];
      cfg.graph = compileLayout(cfg.engine, cfg.pipe, cfg.collector);
      return JSON.stringify(cfg.graph.ducts.map((d) => [d.id, d.segments.map((s) => [s.kind, s.length, s.dIn, s.dOut, s.yaw, s.pitch])]));
    };
    const first = pick('V8, cross');
    pick('V8, flat');
    expect(pick('V8, cross')).toBe(first);
    pick('Single');
    pick('Inline');
    expect(pick('V8, cross')).toBe(first);
  });
});

/** Pipes joined at a junction at the same bore stay joined at it when one of them is resized there. */
describe('carrying a bore across a junction', () => {
  function junction(): ExhaustGraph {
    return {
      ducts: [
        { id: 'a', segments: [makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.04, dOut: 0.045 })], from: { kind: 'valve', cylinder: 0 }, to: { kind: 'node', node: 'j' } },
        { id: 'b', segments: [makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.04, dOut: 0.045 })], from: { kind: 'valve', cylinder: 1 }, to: { kind: 'node', node: 'j' } },
        // Arrives narrower: a different bore, which is left as it is.
        { id: 'c', segments: [makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.03 })], from: { kind: 'valve', cylinder: 2 }, to: { kind: 'node', node: 'j' } },
        { id: 'out', segments: [makeSegment({ kind: 'pipe', length: 0.5, dIn: 0.045, dOut: 0.06 })], from: { kind: 'node', node: 'j' }, to: { kind: 'mouth' } },
      ],
    };
  }

  it('the pipe leaving resized where it starts: the pipes arriving at its bore follow', () => {
    const g = junction();
    const out = g.ducts.find((d) => d.id === 'out')!;
    out.segments[0]!.dIn = 0.05;
    carryBore(g, out, 'start', 0.045, 0.05);
    expect(g.ducts.find((d) => d.id === 'a')!.segments[0]!.dOut).toBe(0.05);
    expect(g.ducts.find((d) => d.id === 'b')!.segments[0]!.dOut).toBe(0.05);
    expect(g.ducts.find((d) => d.id === 'c')!.segments[0]!.dOut).toBe(0.03);
    // Only where they meet: where each starts is its own.
    expect(g.ducts.find((d) => d.id === 'a')!.segments[0]!.dIn).toBe(0.04);
  });

  it('a pipe arriving resized where it ends: the rest at its bore follow, the pipe leaving included', () => {
    const g = junction();
    const a = g.ducts.find((d) => d.id === 'a')!;
    a.segments[0]!.dOut = 0.048;
    carryBore(g, a, 'end', 0.045, 0.048);
    expect(g.ducts.find((d) => d.id === 'b')!.segments[0]!.dOut).toBe(0.048);
    expect(g.ducts.find((d) => d.id === 'out')!.segments[0]!.dIn).toBe(0.048);
    expect(g.ducts.find((d) => d.id === 'out')!.segments[0]!.dOut).toBe(0.06);
  });

  it('leaves a turbo’s pipes alone: its flanges are its own size', () => {
    const g = junction();
    g.turbos = [{ id: 'turbo1', node: 'j', position: null, rotation: [0, 0, 0, 1] }];
    const out = g.ducts.find((d) => d.id === 'out')!;
    carryBore(g, out, 'start', 0.045, 0.05);
    expect(g.ducts.find((d) => d.id === 'a')!.segments[0]!.dOut).toBe(0.045);
  });
});

describe('adding at a junction', () => {
  it('puts the segment first on the pipe leaving it', async () => {
    const { addAtJunction } = await import('../src/model/exhaustGraph.js');
    const spec = specOf({ cylinders: 2, exhaustLayout: '2into1' });
    const graph = compileLayout(spec, [makeSegment({ length: 0.4 })], [makeSegment({ length: 0.5, dIn: 0.06 })]);
    const collector = graph.ducts.find((d) => d.from.kind === 'node')!;
    const node = (collector.from as { node: string }).node;
    const before = collector.segments.length;
    expect(addAtJunction(graph, node, 'chamber')).toBe(collector.id);
    expect(collector.segments).toHaveLength(before + 1);
    expect(collector.segments[0]!.kind).toBe('chamber');
    expect(collector.segments[0]!.dIn).toBeCloseTo(0.06, 9);
  });

  it('makes a pipe out of a junction nothing leaves yet', async () => {
    const { addAtJunction, removeDuct } = await import('../src/model/exhaustGraph.js');
    const spec = specOf({ cylinders: 2, exhaustLayout: '2into1' });
    const graph = compileLayout(spec, [makeSegment({ length: 0.4, dIn: 0.04 })], [makeSegment({ length: 0.5 })]);
    const collector = graph.ducts.find((d) => d.from.kind === 'node')!;
    const node = (collector.from as { node: string }).node;
    removeDuct(graph, collector.id);
    const id = addAtJunction(graph, node, 'pipe')!;
    const added = graph.ducts.find((d) => d.id === id)!;
    expect(added.from).toEqual({ kind: 'node', node });
    expect(added.to).toEqual({ kind: 'mouth' });
    expect(added.segments[0]!.dIn).toBeCloseTo(0.04, 9);
  });
});
