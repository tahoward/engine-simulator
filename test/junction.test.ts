/**
 * A collector must stay solvable whatever the user draws into it.
 *
 * The hard case is the collector *inlet area*, not anything about what follows it. Four 42 mm
 * primaries merging into a 42 mm inlet ask the junction to pass four pipes' worth of flow
 * through one pipe's area. Solved as drawn, the cell behind it over-expands toward vacuum and
 * the returning wave divided by a collapsed `rho c` becomes an absurd velocity: the duct
 * diverges, a V8 goes silent, the pressure display saturates, and the crank pins at the
 * free-running clamp.
 *
 * Two independent things are asserted, and the second is the one with teeth:
 *
 *   - `recoveries` stays at zero, so nothing diverged;
 *   - `junctionClamps` stays at zero, so the junction never even had to catch a degenerate
 *     end state. A geometry that merely avoids diverging is still not being solved as drawn.
 *     A junction can clamp on a large share of samples while reporting zero recoveries,
 *     which is exactly the failure a recoveries-only test misses.
 */

import { describe, expect, it } from 'vitest';

import { EngineSim, gridBudgetCells } from '../src/audio/worklet/engineSim.js';
import { compileCollectorLayout, compileLayout, nodeOrder, type ExhaustGraph } from '../src/model/exhaustGraph.js';

/** The grid budget for an engine as configured, from the graph it builds. */
function budgetOf(cfg: { engine: { cylinders: number }; graph?: ExhaustGraph; pipe: PipeSegment[]; collector: PipeSegment[] }): number {
  const graph = cfg.graph ?? compileLayout(cfg.engine as EngineSpec, cfg.pipe, cfg.collector);
  return gridBudgetCells(cfg.engine.cylinders, nodeOrder(graph).length);
}
import {
  ENGINE_PRESETS,
  defaultConfig,
  firingPlan,
  makeSegment,
  type EngineSpec,
  type PipeSegment,
} from '../src/model/spec.js';

const FS = 48000;

const V8: Partial<EngineSpec> = {
  cylinders: 8,
  vAngle: 90,
  crankType: 'flatplane',
  exhaustLayout: 'perBank',
};

interface Health {
  recoveries: number;
  clamps: number;
  finite: boolean;
  peak: number;
}

function runHealth(
  engine: Partial<EngineSpec>,
  pipe: PipeSegment[],
  collector: PipeSegment[],
  rpm: number,
  seconds = 1.5,
): Health {
  const cfg = defaultConfig();
  cfg.engine = { ...cfg.engine, ...engine, rpm, throttle: 1, freeRunning: false };
  cfg.pipe = pipe;
  cfg.collector = collector;

  // A collector system: these tests are about the junction a set of runners merges at.

  cfg.graph = compileCollectorLayout(cfg.engine, cfg.pipe, cfg.collector ?? []);

  const sim = new EngineSim(FS, cfg);
  let finite = true;
  let peak = 0;
  for (let i = 0; i < FS * seconds; i++) {
    const y = sim.tick();
    if (!Number.isFinite(y)) finite = false;
    else if (Math.abs(y) > peak) peak = Math.abs(y);
  }

  const sys = sim.pipeSolver;
  const ducts = [...sys.primaries, ...sys.collectors];
  return {
    recoveries: ducts.reduce((a, d) => a + d.recoveries, 0),
    clamps: ducts.reduce((a, d) => a + d.junctionClamps, 0),
    finite,
    peak,
  };
}

const PRIMARY = () => [makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.042 })];

describe('collector junctions stay solvable', () => {
  /**
   * Chambers and a cone behind a starved inlet, a pipe-then-chamber and a plain pipe as
   * controls, and two deliberately worse cases. A 42 mm inlet behind four 42 mm primaries is an
   * area ratio of 0.25, the most starved the junction is asked to handle.
   */
  const cases: Array<{ name: string; collector: () => PipeSegment[]; rpm: number }> = [
    {
      name: 'chamber straight off the junction',
      collector: () => [
        makeSegment({ kind: 'chamber', length: 0.34, dIn: 0.042, dOut: 0.13 }),
        makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.04 }),
      ],
      rpm: 4000,
    },
    {
      name: 'cone to 130 mm then chamber',
      collector: () => [
        makeSegment({ kind: 'cone', length: 0.2, dIn: 0.055, dOut: 0.13 }),
        makeSegment({ kind: 'chamber', length: 0.34, dIn: 0.13, dOut: 0.13 }),
        makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.04 }),
      ],
      rpm: 4000,
    },
    {
      name: 'pipe then chamber',
      collector: () => [
        makeSegment({ kind: 'pipe', length: 0.2, dIn: 0.055 }),
        makeSegment({ kind: 'chamber', length: 0.34, dIn: 0.055, dOut: 0.13 }),
        makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.04 }),
      ],
      rpm: 4000,
    },
    {
      name: 'plain pipe (control)',
      collector: () => [makeSegment({ kind: 'pipe', length: 0.6, dIn: 0.055 })],
      rpm: 4000,
    },
    {
      name: 'worse: 42 mm inlet to a 150 mm can at 7000 rpm',
      collector: () => [
        makeSegment({ kind: 'chamber', length: 0.3, dIn: 0.042, dOut: 0.15 }),
        makeSegment({ kind: 'pipe', length: 0.25, dIn: 0.038 }),
      ],
      rpm: 7000,
    },
    {
      name: 'worse: straight into a 150 mm can, no tailpipe',
      collector: () => [makeSegment({ kind: 'chamber', length: 0.4, dIn: 0.042, dOut: 0.15 })],
      rpm: 6000,
    },
  ];

  it.each(cases)('V8, collector = $name', ({ collector, rpm }) => {
    const h = runHealth(V8, PRIMARY(), collector(), rpm);
    expect(h.finite).toBe(true);
    expect(h.recoveries).toBe(0);
    expect(h.clamps).toBe(0);
    // A silent or a saturated result would both pass the checks above.
    expect(h.peak).toBeGreaterThan(1e-3);
    expect(h.peak).toBeLessThan(4);
  });

  /**
   * Widening a starved inlet must not cost the chamber its width, as limiting the area
   * *gradient* near the junction would — that shrinks a 130 mm can to about 92 mm. The inlet is
   * raised; nothing downstream of it moves.
   */
  it('raises a starved inlet without shrinking the chamber behind it', () => {
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, ...V8, rpm: 4000, throttle: 1, freeRunning: false };
    cfg.pipe = PRIMARY();
    cfg.collector = [
      makeSegment({ kind: 'chamber', length: 0.34, dIn: 0.042, dOut: 0.13 }),
      makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.04 }),
    ];
    // A collector system: these tests are about the junction a set of runners merges at.
    cfg.graph = compileCollectorLayout(cfg.engine, cfg.pipe, cfg.collector ?? []);
    const sim = new EngineSim(FS, cfg);
    const coll = sim.pipeSolver.collectors[0]!;
    const areaFace = (coll as unknown as { areaFace: Float64Array }).areaFace;

    const feedArea = sim.pipeSolver.primaries
      .slice(0, 4)
      .reduce((a, d) => a + d.outletArea, 0);
    // Raised to the 0.6 floor rather than left at the drawn 42 mm.
    expect(areaFace[0]!).toBeGreaterThan(0.55 * feedArea);
    // The can still reaches its drawn 130 mm: pi/4 * 0.13^2 = 132.7 cm^2.
    const widest = Math.max(...Array.from(areaFace.slice(0, coll.n + 1)));
    expect(widest).toBeGreaterThan(0.95 * (Math.PI * 0.13 * 0.13) / 4);
  });

  /** Every shipped preset must be clean on both counters too. */
  it.each(ENGINE_PRESETS.filter((p) => p.collector).map((p) => p.name))(
    'preset is clean: %s',
    (name) => {
      const preset = ENGINE_PRESETS.find((p) => p.name === name)!;
      const h = runHealth(preset.engine, preset.pipe(), preset.collector!(), 6500, 1);
      expect(h.finite).toBe(true);
      expect(h.recoveries).toBe(0);
      expect(h.clamps).toBe(0);
    },
  );
});

/**
 * The bank angle must reach the sound.
 *
 * `firingPlan` derives the offsets from pin angles. The offsets `[0, 90, ... 630]` whatever the
 * vee angle would be the firing pattern of a 90-degree V8 and of no other, and changing the angle
 * would do nothing. So the 90-degree case must give exactly that — the presets depend on it — and
 * every other angle must move.
 */
describe('bank angle reaches the firing plan', () => {
  const v8 = (crankType: EngineSpec['crankType'], vAngle: number): EngineSpec =>
    ({ ...defaultConfig().engine, cylinders: 8, crankType, vAngle }) as EngineSpec;

  it.each(['crossplane', 'flatplane'] as const)(
    '%s at 90 degrees still fires evenly every 90',
    (crankType) => {
      expect(firingPlan(v8(crankType, 90)).offsets).toEqual([
        0, 90, 180, 270, 360, 450, 540, 630,
      ]);
    },
  );

  it.each(['crossplane', 'flatplane'] as const)('%s fires unevenly at 60 degrees', (crankType) => {
    const offsets = firingPlan(v8(crankType, 60)).offsets;
    expect(offsets).not.toEqual([0, 90, 180, 270, 360, 450, 540, 630]);
    // Still eight distinct firings inside one cycle, just not evenly spaced.
    expect(new Set(offsets).size).toBe(8);
    const sorted = [...offsets].sort((a, b) => a - b);
    const gaps = sorted.map((o, i) => (i === 0 ? o + 720 - sorted[7]! : o - sorted[i - 1]!));
    expect(Math.max(...gaps) - Math.min(...gaps)).toBeGreaterThan(1);
    expect(gaps.reduce((a, b) => a + b, 0)).toBeCloseTo(720, 6);
  });

  /** A V-twin derives its interval from the vee; 45 degrees is the Harley 405/315. */
  it('a V-twin still derives 405/315 from a 45 degree vee', () => {
    const spec = { ...defaultConfig().engine, cylinders: 2, vAngle: 45, firingOffset: null };
    expect(firingPlan(spec as EngineSpec).offsets).toEqual([0, 405]);
  });

  /** The bank angle must actually change the audio, not merely the plan. */
  it.each([
    ['V8 crossplane', { cylinders: 8, exhaustLayout: 'perBank', crankType: 'crossplane' }],
    ['V8 flatplane', { cylinders: 8, exhaustLayout: 'perBank', crankType: 'flatplane' }],
    ['V-twin', { cylinders: 2, exhaustLayout: '2into1', firingOffset: null }],
  ] as Array<[string, Partial<EngineSpec>]>)('%s sounds different at a different vee', (_n, base) => {
    const render = (vAngle: number) => {
      const cfg = defaultConfig();
      cfg.engine = { ...cfg.engine, ...base, vAngle, rpm: 4000, throttle: 0.9, freeRunning: false };
      cfg.pipe = [makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.042 })];
      cfg.collector = [makeSegment({ kind: 'pipe', length: 0.6, dIn: 0.06 })];
      // A collector system: these tests are about the junction a set of runners merges at.
      cfg.graph = compileCollectorLayout(cfg.engine, cfg.pipe, cfg.collector ?? []);
      const sim = new EngineSim(FS, cfg);
      sim.render(FS / 2);
      return sim.render(FS / 2);
    };
    const a = render(base.cylinders === 2 ? 45 : 90);
    const b = render(60);
    let diff = 0;
    let energy = 0;
    for (let i = 0; i < a.length; i++) {
      diff += (a[i]! - b[i]!) ** 2;
      energy += a[i]! ** 2;
    }
    // A tenth of the signal's own energy is far more than drift; inert would be exactly zero.
    expect(Math.sqrt(diff / Math.max(energy, 1e-30))).toBeGreaterThan(0.1);
  });
});

/**
 * Noise must not heat the duct it is stirring.
 *
 * The failure guarded against is an energy rectifier in the valve source. A mass exchange at a
 * boundary trades *stagnation enthalpy*, because gas crossing an orifice does flow work on whatever
 * it moves into. A reverse branch that took only `e + u^2/2` back out while the forward branch put
 * `h + u^2/2` in would deposit `R T` per unit mass on every in-and-out pair: about 430 kJ/kg at
 * 1500 K, against 45 kJ/kg of kinetic energy being accounted for. Throat turbulence, valve-seat
 * pulses and collector merge noise are all zero-mean sources that would feed it, and because the
 * source divides by the first cell's volume the smallest ducts would heat fastest.
 *
 * Asserted at engine level rather than by driving a bare duct with a synthetic square wave. That
 * looks like the tighter test and is not: the source is capped against the first cell's density
 * and floored for admissibility, so a large synthetic amplitude measures those guards rather than
 * the enthalpy asymmetry, and the two are hard to separate afterwards. What matters is that a real
 * engine with its noise sources at full scale stays at a physical temperature.
 */
describe('noise does not pump energy into the exhaust', () => {
  it.each([
    ['tiny collector, noise at full scale', 0.2],
    ['short collector, noise at full scale', 0.35],
  ] as Array<[string, number]>)('%s', (_name, length) => {
    const cfg = defaultConfig();
    cfg.engine = {
      ...cfg.engine,
      ...V8,
      rpm: 8500,
      throttle: 1,
      freeRunning: false,
      throatNoise: 1,
      mechNoise: 1,
    };
    cfg.pipe = PRIMARY();
    cfg.collector = [makeSegment({ kind: 'cone', length, dIn: 0.042, dOut: 0.13 })];
    // A collector system: these tests are about the junction a set of runners merges at.
    cfg.graph = compileCollectorLayout(cfg.engine, cfg.pipe, cfg.collector ?? []);
    const sim = new EngineSim(FS, cfg);
    sim.render(FS * 2);

    const ducts = [...sim.pipeSolver.primaries, ...sim.pipeSolver.collectors];
    let maxT = 0;
    for (const d of ducts) for (let k = 0; k < d.n; k++) maxT = Math.max(maxT, d.temperatureAt(k));
    // Exhaust leaves a cylinder near 1200-1600 K and only cools from there. An enthalpy
    // rectifier would drive this geometry past 40,000 K.
    expect(maxT).toBeLessThan(2500);
    expect(ducts.reduce((a, d) => a + d.recoveries, 0)).toBe(0);
  });
});

/**
 * A junction must pass what it receives, and the solver must stay inside its cost budget.
 *
 * The node solves for a common pressure by linearising the returning wave, then each branch
 * computes its own nonlinear flux from it; nothing makes those agree on their own, and on every
 * layout they can disagree about the mass crossing the node by tens of percent at peak. A Newton
 * step on the *nonlinear* residual, using the closed-form slope the linear model already provides,
 * brings it to a few percent.
 */
describe('junctions conserve mass and the grid stays affordable', () => {
  const geometries: Array<[string, () => PipeSegment[]]> = [
    ['cone 42->130 over 0.2 m', () => [makeSegment({ kind: 'cone', length: 0.2, dIn: 0.042, dOut: 0.13 })]],
    ['cone 42->90 over 0.5 m', () => [makeSegment({ kind: 'cone', length: 0.5, dIn: 0.042, dOut: 0.09 })]],
    ['cone 50->60 over 1.2 m', () => [makeSegment({ kind: 'cone', length: 1.2, dIn: 0.05, dOut: 0.06 })]],
    ['chamber 42->130 + pipe', () => [
      makeSegment({ kind: 'chamber', length: 0.34, dIn: 0.042, dOut: 0.13 }),
      makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.04 }),
    ]],
  ];

  it.each(geometries)('V8 with %s stays balanced and affordable', (_name, build) => {
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, ...V8, rpm: 8500, throttle: 1, freeRunning: false };
    cfg.pipe = PRIMARY();
    cfg.collector = build();
    // A collector system: these tests are about the junction a set of runners merges at.
    cfg.graph = compileCollectorLayout(cfg.engine, cfg.pipe, cfg.collector ?? []);
    const sim = new EngineSim(FS, cfg);
    sim.render(FS);

    const ducts = [...sim.pipeSolver.primaries, ...sim.pipeSolver.collectors];
    const substeps = Math.max(...ducts.map((d) => d.substeps));
    const cells = ducts.reduce((a, d) => a + d.n, 0);

    // Peak imbalance, not mean: a node has no volume, so a large transient error is still mass
    // from nowhere. The worst of these geometries peaks at 5%: at 8500 rpm the burn runs late
    // enough that the blowdown pulses reaching the node are the hardest the engine makes.
    expect(sim.pipeSolver.junctionResidual).toBeLessThan(0.06);

    /**
     * One substep, always.
     *
     * A V8 can afford a grid coarse enough for a single CFL substep at every duct length, and the
     * cost budget must find it. A grid a couple of millimetres finer than the one substep needs
     * pays two substeps for a few percent of resolution, which puts the preset at about a whole
     * core. Above 100% the audio thread cannot deliver at all, so the sound would cut out at high
     * rpm and come back as the revs fell.
     */
    expect(substeps).toBe(1);
    expect(cells * substeps).toBeLessThanOrEqual(budgetOf(cfg));
    // And nothing degenerate along the way.
    expect(ducts.reduce((a, d) => a + d.recoveries, 0)).toBe(0);
    expect(ducts.reduce((a, d) => a + d.junctionClamps, 0)).toBe(0);
  });

  /** Coarsening for cost must never make the duct *more* expensive than a coarser choice would. */
  it('never picks a grid a coarser one would beat', () => {
    for (const L of [0.15, 0.25, 0.4, 0.6, 0.9, 1.4]) {
      const cfg = defaultConfig();
      cfg.engine = { ...cfg.engine, ...V8, rpm: 6500, throttle: 1, freeRunning: false };
      cfg.pipe = PRIMARY();
      cfg.collector = [makeSegment({ kind: 'cone', length: L, dIn: 0.042, dOut: 0.09 })];
      // A collector system: these tests are about the junction a set of runners merges at.
      cfg.graph = compileCollectorLayout(cfg.engine, cfg.pipe, cfg.collector ?? []);
      const sim = new EngineSim(FS, cfg);
      const ducts = [...sim.pipeSolver.primaries, ...sim.pipeSolver.collectors];
      const cellSteps = ducts.reduce((a, d) => a + d.n, 0) * Math.max(...ducts.map((d) => d.substeps));
      expect(cellSteps).toBeLessThanOrEqual(budgetOf(cfg));
    }
  });
});

/**
 * The manifolds every preset compiles to stay solvable and affordable.
 *
 * Each junction on a manifold takes the full blowdown from a stub a few centimetres away, where a
 * collector's half-metre runners would spread it out, so these are the hardest junctions the solver sees.
 * Their peak imbalance is larger than a collector's — it spikes where flow through a junction reverses —
 * so what is held here is what matters for the sound: nothing blows up, nothing is clamped, no duct
 * needs recovering, the gas stays at physical temperatures, and the grid stays inside its budget.
 */
describe('manifolds stay solvable and affordable', () => {
  it.each(ENGINE_PRESETS.map((p) => p.name))('%s at 8000 rpm, full throttle', (name) => {
    const preset = ENGINE_PRESETS.find((p) => p.name === name)!;
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, ...preset.engine, rpm: 8000, throttle: 1, freeRunning: false };
    cfg.pipe = preset.pipe();
    cfg.collector = preset.collector ? preset.collector() : [];
    const sim = new EngineSim(FS, cfg);
    const out = sim.render(FS / 2);
    for (const v of out) expect(Number.isFinite(v)).toBe(true);

    const ducts = [...sim.pipeSolver.primaries, ...sim.pipeSolver.collectors];
    let maxT = 0;
    for (const d of ducts) for (let k = 0; k < d.n; k++) maxT = Math.max(maxT, d.temperatureAt(k));
    expect(maxT).toBeLessThan(2500);
    expect(ducts.reduce((a, d) => a + d.recoveries, 0)).toBe(0);
    expect(ducts.reduce((a, d) => a + d.junctionClamps, 0)).toBe(0);

    // Inside the cost budget. A V8 has only just enough for one substep, and must get it; smaller
    // engines are allowed the finer grid they can afford.
    const substeps = Math.max(...ducts.map((d) => d.substeps));
    const cells = ducts.reduce((a, d) => a + d.n, 0);
    if (cfg.engine.cylinders === 8) expect(substeps).toBe(1);
    expect(cells * substeps).toBeLessThanOrEqual(budgetOf(cfg));
  });
});
