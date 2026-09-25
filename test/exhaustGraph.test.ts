/**
 * The exhaust graph: that it reproduces the primaries-and-collectors topology exactly, that it
 * rejects nonsense, and that it can express and solve arrangements that topology cannot.
 *
 * The last part is the point of a graph. "N identical primaries plus M collectors" cannot describe
 * a tri-Y, a branch part way along a duct, or runners of different lengths; a graph can, and the
 * solver has to actually run them rather than merely accept them.
 */

import { describe, expect, it } from 'vitest';

import { EngineSim } from '../src/audio/worklet/engineSim.js';
import { ExhaustSystem } from '../src/audio/worklet/exhaustSystem.js';
import {
  compileCollectorLayout,
  compileLayout,
  copyToSiblingRunners,
  ductLabel,
  endsAt,
  pathToAir,
  nodeOrder,
  radiatingDucts,
  validateGraph,
  valveDucts,
  type ExhaustGraph,
} from '../src/model/exhaustGraph.js';
import {
  ENGINE_PRESETS,
  GAS,
  collectorGroups,
  defaultConfig,
  makeSegment,
  type EngineSpec,
} from '../src/model/spec.js';

const FS = 48000;
const specOf = (partial: Partial<EngineSpec>): EngineSpec =>
  ({ ...defaultConfig().engine, ...partial }) as EngineSpec;

describe('compileCollectorLayout reproduces the primaries-and-collectors layout', () => {
  it.each(ENGINE_PRESETS.map((p) => p.name))('%s', (name) => {
    const preset = ENGINE_PRESETS.find((p) => p.name === name)!;
    const spec = specOf(preset.engine);
    const graph = compileCollectorLayout(spec, preset.pipe(), preset.collector ? preset.collector() : []);
    const groups = collectorGroups(spec);

    expect(validateGraph(graph, spec.cylinders)).toEqual([]);

    // One duct per cylinder valve, and it goes where the grouping says.
    const valves = valveDucts(graph, spec.cylinders);
    expect(valves.every((d) => d !== null)).toBe(true);
    groups.forEach((group, cylinder) => {
      const duct = valves[cylinder]!;
      if (group < 0) expect(duct.to).toEqual({ kind: 'mouth' });
      else expect(duct.to).toEqual({ kind: 'node', node: `merge${group}` });
    });

    // A node per group actually used, and nothing else.
    const used = [...new Set(groups.filter((g) => g >= 0))].sort((a, b) => a - b);
    expect(nodeOrder(graph)).toEqual(used.map((g) => `merge${g}`));

    // Each node joins its members plus exactly one duct leaving it.
    for (const g of used) {
      const ends = endsAt(graph, `merge${g}`);
      const upstream = ends.filter((e) => e.end === 'outlet');
      const downstream = ends.filter((e) => e.end === 'inlet');
      expect(upstream).toHaveLength(groups.filter((x) => x === g).length);
      expect(downstream).toHaveLength(1);
    }
  });

  /**
   * Mouth order is load-bearing, not cosmetic: `refreshMouthPaths` lays the mouths out in a line by
   * index and gives each its own delay and gain, so reordering them changes the sound. Collectors
   * come before solo runners.
   */
  it('radiates collectors before runners that vent alone', () => {
    const spec = specOf({ cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' });
    const merged = compileLayout(spec, [makeSegment({ length: 0.4 })], [makeSegment({ length: 0.6 })]);
    expect(radiatingDucts(merged).map((d) => d.id)).toEqual(['collector0', 'collector1']);

    const open = compileLayout(specOf({ cylinders: 2, exhaustLayout: '2into2' }), [makeSegment({})], []);
    expect(radiatingDucts(open).map((d) => d.id)).toEqual(['runner0', 'runner1']);
  });

  /** A group whose collector geometry is empty still gets a duct, or the sound would change. */
  it('keeps a collector even when nothing was drawn for it', () => {
    const spec = specOf({ cylinders: 2, exhaustLayout: '2into1' });
    const graph = compileLayout(spec, [makeSegment({})], []);
    expect(graph.ducts.map((d) => d.id)).toContain('collector0');
    expect(validateGraph(graph, 2)).toEqual([]);
  });

  /** Copies, not shared arrays — which is what later lets one runner be lengthened alone. */
  it('gives every duct its own segments', () => {
    const pipe = [makeSegment({ length: 0.4 })];
    const graph = compileLayout(specOf({ cylinders: 2, exhaustLayout: '2into1' }), pipe, []);
    const a = graph.ducts[0]!.segments[0]!;
    const b = graph.ducts[1]!.segments[0]!;
    expect(a).not.toBe(b);
    expect(a.length).toBe(b.length);
    a.length = 0.9;
    expect(b.length).toBe(0.4);
  });
});

describe('validateGraph catches what a half-drawn route leaves behind', () => {
  const runner = (id: string, cylinder: number, to: ExhaustGraph['ducts'][0]['to']) => ({
    id,
    segments: [makeSegment({ length: 0.4 })],
    from: { kind: 'valve' as const, cylinder },
    to,
  });

  it('a cylinder with no pipe', () => {
    const graph: ExhaustGraph = { ducts: [runner('a', 0, { kind: 'mouth' })] };
    expect(validateGraph(graph, 2).join(' ')).toContain('cylinder 2 has no exhaust pipe');
  });

  it('two pipes on one port', () => {
    const graph: ExhaustGraph = {
      ducts: [runner('a', 0, { kind: 'mouth' }), runner('b', 0, { kind: 'mouth' })],
    };
    expect(validateGraph(graph, 1).join(' ')).toContain('has 2 pipes on its exhaust port');
  });

  it('a junction joining only one pipe', () => {
    const graph: ExhaustGraph = { ducts: [runner('a', 0, { kind: 'node', node: 'x' })] };
    const problems = validateGraph(graph, 1).join(' ');
    expect(problems).toContain('joins only one pipe');
    expect(problems).toContain('no pipe leaving it');
  });

  it('a duct connected to no cylinder', () => {
    const graph: ExhaustGraph = {
      ducts: [
        runner('a', 0, { kind: 'mouth' }),
        {
          id: 'orphan',
          segments: [makeSegment({})],
          from: { kind: 'node', node: 'nowhere' },
          to: { kind: 'mouth' },
        },
      ],
    };
    expect(validateGraph(graph, 1).join(' ')).toContain('"orphan" is not connected to any cylinder');
  });

  it('duplicate ids', () => {
    const graph: ExhaustGraph = {
      ducts: [runner('same', 0, { kind: 'mouth' }), runner('same', 1, { kind: 'mouth' })],
    };
    expect(validateGraph(graph, 2).join(' ')).toContain('two ducts share the id');
  });

  it('and the solver refuses to build one', () => {
    const graph: ExhaustGraph = { ducts: [runner('a', 0, { kind: 'node', node: 'x' })] };
    expect(() => new ExhaustSystem(graph, 1, FS, GAS.tAmb, {})).toThrow(/cannot be solved/);
  });
});

/**
 * Topologies a fixed set of primaries and collectors has no way to describe.
 *
 * A tri-Y merges in two stages, so its middle ducts are fed by a junction *and* feed another one —
 * something `primaries`/`collectors` alone cannot express, where a collector is always the last
 * duct. Unequal runners would need a second segment array.
 */
describe('arrangements primaries and collectors cannot express', () => {
  /** 4 cylinders, paired into two intermediate ducts, merged again into one tailpipe. */
  function triY(): ExhaustGraph {
    const runner = (i: number, node: string, length: number) => ({
      id: `runner${i}`,
      segments: [makeSegment({ length, dIn: 0.038 })],
      from: { kind: 'valve' as const, cylinder: i },
      to: { kind: 'node' as const, node },
    });
    return {
      ducts: [
        runner(0, 'pairA', 0.4),
        runner(1, 'pairB', 0.4),
        runner(2, 'pairA', 0.4),
        runner(3, 'pairB', 0.4),
        {
          id: 'midA',
          segments: [makeSegment({ length: 0.3, dIn: 0.05 })],
          from: { kind: 'node', node: 'pairA' },
          to: { kind: 'node', node: 'tail' },
        },
        {
          id: 'midB',
          segments: [makeSegment({ length: 0.3, dIn: 0.05 })],
          from: { kind: 'node', node: 'pairB' },
          to: { kind: 'node', node: 'tail' },
        },
        {
          id: 'tailpipe',
          segments: [makeSegment({ length: 0.5, dIn: 0.06 })],
          from: { kind: 'node', node: 'tail' },
          to: { kind: 'mouth' },
        },
      ],
    };
  }

  it('a tri-Y validates, and its middle ducts are both fed and feeding', () => {
    const graph = triY();
    expect(validateGraph(graph, 4)).toEqual([]);
    expect(nodeOrder(graph)).toEqual(['pairA', 'pairB', 'tail']);
    expect(radiatingDucts(graph).map((d) => d.id)).toEqual(['tailpipe']);
    expect(endsAt(graph, 'tail').map((e) => e.end).sort()).toEqual(['inlet', 'outlet', 'outlet']);
  });

  it('a tri-Y runs, stays finite and makes a sound', () => {
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, cylinders: 4, exhaustLayout: '2into1', rpm: 4000, throttle: 1, freeRunning: false } as EngineSpec;
    const sim = new EngineSim(FS, cfg, {}, triY());
    sim.render(FS / 2);
    const audio = sim.render(FS / 2);

    let peak = 0;
    for (let i = 0; i < audio.length; i++) {
      expect(Number.isFinite(audio[i]!)).toBe(true);
      if (Math.abs(audio[i]!) > peak) peak = Math.abs(audio[i]!);
    }
    expect(peak).toBeGreaterThan(1e-3);

    const ducts = [...sim.pipeSolver.primaries, ...sim.pipeSolver.collectors];
    expect(ducts).toHaveLength(7);
    expect(ducts.reduce((a, d) => a + d.recoveries, 0)).toBe(0);
    expect(ducts.reduce((a, d) => a + d.junctionClamps, 0)).toBe(0);
    let maxT = 0;
    for (const d of ducts) for (let k = 0; k < d.n; k++) maxT = Math.max(maxT, d.temperatureAt(k));
    expect(maxT).toBeLessThan(2500);
  });

  it('unequal runners change the sound', () => {
    const render = (lengths: number[]) => {
      const graph = triY();
      lengths.forEach((l, i) => {
        graph.ducts[i]!.segments[0]!.length = l;
      });
      const cfg = defaultConfig();
      cfg.engine = { ...cfg.engine, cylinders: 4, exhaustLayout: '2into1', rpm: 4000, throttle: 1, freeRunning: false } as EngineSpec;
      const sim = new EngineSim(FS, cfg, {}, graph);
      sim.render(FS / 2);
      return sim.render(FS / 2);
    };
    const even = render([0.4, 0.4, 0.4, 0.4]);
    const uneven = render([0.3, 0.5, 0.35, 0.45]);

    let diff = 0;
    let energy = 0;
    for (let i = 0; i < even.length; i++) {
      diff += (even[i]! - uneven[i]!) ** 2;
      energy += even[i]! ** 2;
    }
    // A tenth of the signal's own energy: far more than drift, and impossible if runners shared one geometry.
    expect(Math.sqrt(diff / Math.max(energy, 1e-30))).toBeGreaterThan(0.1);
  });
});

/**
 * The pieces the panel and the URL rely on.
 *
 * `pathToAir` is what "the tuned length" means once the exhaust is a graph — a runner plus whatever it
 * merges into, however many stages that takes. "Primary plus collector" would only describe the layouts
 * that have exactly those two parts.
 */
describe('walking the graph for the panel and the URL', () => {
  it('a compiled path is the runner plus its collector', () => {
    const spec = specOf({ cylinders: 4, exhaustLayout: '2into1' });
    const pipe = [makeSegment({ length: 0.45, dIn: 0.038 })];
    const collector = [makeSegment({ length: 0.6, dIn: 0.055 })];
    const graph = compileCollectorLayout(spec, pipe, collector);

    const path = pathToAir(graph, 0);
    expect(path.map((d) => d.id)).toEqual(['runner0', 'collector0']);
    const len = path.reduce((a, d) => a + d.segments.reduce((b, sg) => b + sg.length, 0), 0);
    expect(len).toBeCloseTo(0.45 + 0.6, 9);
  });

  it('an unmerged runner is the whole path', () => {
    const graph = compileLayout(specOf({ cylinders: 2, exhaustLayout: '2into2' }), [makeSegment({ length: 0.5 })], []);
    expect(pathToAir(graph, 1).map((d) => d.id)).toEqual(['runner1']);
  });

  it('a two-stage merge gives a three-duct path', () => {
    const graph: ExhaustGraph = {
      ducts: [
        { id: 'r0', segments: [makeSegment({ length: 0.4 })], from: { kind: 'valve', cylinder: 0 }, to: { kind: 'node', node: 'a' } },
        { id: 'r1', segments: [makeSegment({ length: 0.4 })], from: { kind: 'valve', cylinder: 1 }, to: { kind: 'node', node: 'a' } },
        { id: 'mid', segments: [makeSegment({ length: 0.3 })], from: { kind: 'node', node: 'a' }, to: { kind: 'node', node: 'b' } },
        { id: 'r2', segments: [makeSegment({ length: 0.4 })], from: { kind: 'valve', cylinder: 2 }, to: { kind: 'node', node: 'b' } },
        { id: 'tail', segments: [makeSegment({ length: 0.5 })], from: { kind: 'node', node: 'b' }, to: { kind: 'mouth' } },
      ],
    };
    expect(pathToAir(graph, 0).map((d) => d.id)).toEqual(['r0', 'mid', 'tail']);
    expect(pathToAir(graph, 2).map((d) => d.id)).toEqual(['r2', 'tail']);
  });

  /** A loop is a drawing mistake; walking it must terminate rather than hang the panel. */
  it('does not loop forever on a cycle', () => {
    const graph: ExhaustGraph = {
      ducts: [
        { id: 'r0', segments: [makeSegment({})], from: { kind: 'valve', cylinder: 0 }, to: { kind: 'node', node: 'a' } },
        { id: 'x', segments: [makeSegment({})], from: { kind: 'node', node: 'a' }, to: { kind: 'node', node: 'b' } },
        { id: 'y', segments: [makeSegment({})], from: { kind: 'node', node: 'b' }, to: { kind: 'node', node: 'a' } },
      ],
    };
    expect(pathToAir(graph, 0).length).toBeLessThanOrEqual(3);
  });

  it('labels ducts by where they start', () => {
    const spec = specOf({ cylinders: 2, exhaustLayout: '2into1' });
    const graph = compileLayout(spec, [makeSegment({})], [makeSegment({})]);
    const labels = graph.ducts.map((d) => ductLabel(graph, d));
    expect(labels).toEqual(['Cylinder 1 runner', 'Cylinder 2 runner', 'After junction 1']);
  });

  /**
   * A graph survives the URL.
   *
   * Saved as base64 JSON in the hash, so it comes back as plain data with no prototypes and no
   * guarantee of being well formed. Rebuilding through `makeSegment` is what stops a truncated link
   * putting `undefined` diameters into the solver.
   */
  it('round-trips through JSON and still validates', () => {
    const spec = specOf({ cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' });
    const original = compileLayout(spec, [makeSegment({ length: 0.4 })], [makeSegment({ length: 0.6 })]);
    original.ducts[0]!.headingYaw = 0.3;

    const revived = JSON.parse(JSON.stringify(original)) as ExhaustGraph;
    const rebuilt: ExhaustGraph = {
      ducts: revived.ducts.map((d) => ({
        ...d,
        segments: d.segments.map((sg) => makeSegment(sg)),
      })),
    };

    expect(validateGraph(rebuilt, 8)).toEqual([]);
    expect(rebuilt.ducts[0]!.headingYaw).toBeCloseTo(0.3, 9);
    expect(nodeOrder(rebuilt)).toEqual(nodeOrder(original));
    expect(rebuilt.ducts.map((d) => d.id)).toEqual(original.ducts.map((d) => d.id));
  });

  /** A graph for the wrong engine is rejected, which is what makes discarding it on load safe. */
  it('rejects a graph that does not fit the engine', () => {
    const forTwo = compileLayout(specOf({ cylinders: 2, exhaustLayout: '2into1' }), [makeSegment({})], [makeSegment({})]);
    expect(validateGraph(forTwo, 2)).toEqual([]);
    expect(validateGraph(forTwo, 8).length).toBeGreaterThan(0);
  });
});

/**
 * A drawn graph can outlive the engine it was drawn for, and the audio thread must survive that.
 *
 * `setEngine` and `setGraph` are separate messages, so switching a V-twin to a V8 rebuilds the exhaust
 * once with the new cylinder count and the *old* graph before the new one arrives — and that graph has
 * no pipe on cylinders 3 to 8. `ExhaustSystem` rightly refuses to build it. A throw landing inside the
 * worklet's message handler would leave a two-duct exhaust attached to an eight-cylinder engine; the
 * next `process` call would read `primaries[2]`, get `undefined`, and the node would die for good, so
 * switching engine would silence the app permanently.
 *
 * Nothing on the audio thread may throw: there is nothing above it to catch anything, and the cost of
 * being wrong is the whole app going quiet.
 */
describe('a stale graph does not silence the engine', () => {
  const twin = (pipe: ReturnType<typeof makeSegment>[]) => {
    const cfg = defaultConfig();
    cfg.engine = {
      ...cfg.engine,
      cylinders: 2,
      vAngle: 45,
      exhaustLayout: '2into1',
      rpm: 3000,
      throttle: 0.8,
      freeRunning: false,
    } as EngineSpec;
    cfg.pipe = pipe;
    cfg.collector = [makeSegment({ length: 0.5, dIn: 0.055 })];
    cfg.graph = compileLayout(cfg.engine, cfg.pipe, cfg.collector);
    return cfg;
  };

  const loudness = (audio: Float32Array) => {
    let peak = 0;
    for (let i = 0; i < audio.length; i++) {
      expect(Number.isFinite(audio[i]!)).toBe(true);
      peak = Math.max(peak, Math.abs(audio[i]!));
    }
    return peak;
  };

  it.each([
    ['with runners drawn', [makeSegment({ length: 0.4, dIn: 0.042 })]],
    ['with the runners deleted', [] as ReturnType<typeof makeSegment>[]],
  ])('a V-twin %s makes a sound, and still does after switching to a V8', (_n, pipe) => {
    const cfg = twin(pipe);
    const sim = new EngineSim(FS, cfg, {}, cfg.graph);
    sim.render(FS / 4);
    // The V-twin itself: a runner with no segments is a short stub, not silence.
    expect(loudness(sim.render(FS / 4))).toBeGreaterThan(1e-4);

    // The order the renderer sends them in: the engine first, the new graph afterwards.
    sim.setEngine({
      ...cfg.engine,
      cylinders: 8,
      vAngle: 90,
      crankType: 'crossplane',
      exhaustLayout: 'perBank',
    } as EngineSpec);
    expect(loudness(sim.render(FS / 4))).toBeGreaterThan(1e-4);

    // And once the matching graph arrives it is used, with eight runners.
    sim.setGraph(compileLayout(sim.engine as EngineSpec, cfg.pipe, cfg.collector));
    expect(loudness(sim.render(FS / 4))).toBeGreaterThan(1e-4);
    expect(sim.pipeSolver.primaries).toHaveLength(8);
  });

  /** Back the other way too: a V8 graph left over on a V-twin. */
  it('survives shrinking the engine under a larger graph', () => {
    const cfg = defaultConfig();
    cfg.engine = {
      ...cfg.engine,
      cylinders: 8,
      vAngle: 90,
      crankType: 'crossplane',
      exhaustLayout: 'perBank',
      rpm: 3000,
      throttle: 0.8,
      freeRunning: false,
    } as EngineSpec;
    cfg.pipe = [makeSegment({ length: 0.4, dIn: 0.042 })];
    cfg.collector = [makeSegment({ length: 0.5, dIn: 0.055 })];
    const sim = new EngineSim(FS, cfg, {}, compileLayout(cfg.engine, cfg.pipe, cfg.collector));
    sim.render(FS / 4);
    sim.setEngine({ ...cfg.engine, cylinders: 2, vAngle: 45, exhaustLayout: '2into1' } as EngineSpec);
    expect(loudness(sim.render(FS / 4))).toBeGreaterThan(1e-4);
    expect(sim.pipeSolver.primaries).toHaveLength(2);
  });
});

/**
 * Linked runners edit every cylinder at once.
 *
 * Per-duct copies are what make unequal headers possible, but they also mean a single edit touches one
 * runner of eight — which looks like nothing happening on a V8 and sounds like almost nothing.
 * `copyToSiblingRunners` is the default that makes editing "the pipe" change all eight runners together.
 */
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
    const edited = [makeSegment({ length: 0.33, dIn: 0.044 })];
    const collector = [makeSegment({ length: 0.55, dIn: 0.058 })];

    const render = (graph: ExhaustGraph) => {
      const cfg = defaultConfig();
      cfg.engine = spec;
      const sim = new EngineSim(FS, cfg, {}, graph);
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

/**
 * What `compileLayout` builds: a manifold of straight tube along each bank.
 *
 * A bank of four is the first cylinder's stub turning onto the manifold, a stub from each of the other
 * three, two further lengths of manifold and the outlet — so four lengths of manifold and outlet in all,
 * and every junction a runner joining the manifold from the side.
 */
describe('compileLayout builds manifolds', () => {
  const v8spec = () => specOf({ cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'perBank' });

  it('a V8 bank is four stubs, two lengths of manifold and an outlet', () => {
    const graph = compileLayout(v8spec(), [makeSegment({ length: 0.45 })], [makeSegment({ length: 0.6 })]);
    expect(validateGraph(graph, 8)).toEqual([]);
    expect(graph.ducts).toHaveLength(14);
    for (const node of nodeOrder(graph)) expect(endsAt(graph, node)).toHaveLength(3);
    // One runner per bank carries the manifold's first length.
    const carriers = graph.ducts.filter((d) => graph.ducts.some((o) => o.continues === d.id) && d.from.kind === 'valve');
    expect(carriers).toHaveLength(2);
    for (const c of carriers) expect(c.segments).toHaveLength(2);
  });

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

  it('meets both banks behind the engine for an 8-into-1, keeping the path length', () => {
    const merged = specOf({ cylinders: 8, vAngle: 90, crankType: 'crossplane', exhaustLayout: 'merged' });
    const collector = [makeSegment({ kind: 'pipe', length: 1.5, dIn: 0.07 })];
    const graph = compileLayout(merged, [makeSegment({ length: 0.45 })], collector);
    expect(validateGraph(graph, 8)).toEqual([]);
    const downs = graph.ducts.filter((d) => d.id.startsWith('down'));
    expect(downs).toHaveLength(2);
    // Mirror images, so the same length, and it came out of the collector.
    expect(downs[0]!.segments[0]!.length).toBeCloseTo(downs[1]!.segments[0]!.length, 9);
    const out = graph.ducts.find((d) => d.id === 'collector0')!;
    expect(out.segments[0]!.length + downs[0]!.segments[0]!.length).toBeCloseTo(1.5, 9);
  });
});

/**
 * Switching presets gives each preset its own exhaust, every time.
 *
 * A change of topology re-seeds the graph and carries the previous runner and collector geometry across.
 * Read off the graph as "the first runner" and "the first pipe after a junction", that would be a 10 cm
 * stub and a manifold length on a manifold; and run after the preset's own geometry has been loaded, the
 * carry-over would overwrite it, so going crossplane to flatplane and back would change the exhaust.
 */
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

/**
 * A manifold widens as it gathers cylinders, and the collector does not pinch it.
 *
 * Capped at the collector's inlet — the narrow end of its entry cone — an inline six's manifold would stay
 * at one runner's bore all the way along, and five cylinders' gas would choke through it: the junction at
 * its end clamping on nearly every sample and the gas reaching 2300 K.
 */
describe('manifold sizing', () => {
  it('widens along the bank and opens into the collector without a step down', async () => {
    const { fittedExhaust } = await import('../src/model/spec.js');
    const six = specOf({ cylinders: 6, vAngle: 0, exhaustLayout: 'merged', bore: 0.082, stroke: 0.0946, exValveDia: 0.031 });
    const fitted = fittedExhaust(six);
    const graph = compileLayout(six, fitted.pipe, fitted.collector);
    const links = graph.ducts.filter((d) => d.role === 'manifold').map((d) => d.segments[0]!.dIn);
    for (let k = 1; k < links.length; k++) expect(links[k]!).toBeGreaterThan(links[k - 1]!);
    const last = links[links.length - 1]!;
    const collector = graph.ducts.find((d) => d.role === 'collector')!;
    expect(collector.segments[0]!.dIn).toBeGreaterThanOrEqual(last - 1e-12);
    // Gathering five of six cylinders: about the bore constant gas speed wants.
    const runner = fitted.pipe[0]!.dIn;
    expect(last / runner).toBeCloseTo(Math.sqrt(5) * 0.92, 6);
  });
});
