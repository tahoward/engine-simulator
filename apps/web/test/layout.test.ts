/**
 * Banks of one to six cylinders, one bank or two: the firing each layout has of its own, a firing order
 * set over it, and every layout drawn, plumbed and run. The firing plans are the Rust crate's too, and
 * `crates/engine-sim/tests/engine8.rs` pins the same numbers on its side.
 */

import { describe, expect, it } from 'vitest';
import {
  GAS,
  crankPins,
  defaultConfig,
  firingIntervalsOf,
  firingOrderOf,
  firingOrderProblem,
  firingPlan,
  isBoxer,
  makeSegment,
  validLayout,
  type EngineSpec,
} from '../src/model/spec.js';
import { compileExhaust, validateGraph } from '../src/model/exhaustGraph.js';
import { engineFile, engineFileName, readEngineFile } from '../src/model/engineFile.js';
import { EngineMesh } from '../src/scene/EngineMesh.js';
import { inletLayout } from '../src/scene/inletLayout.js';
import { Sim } from '../src/audio/worklet/sim.js';

function spec(over: Partial<EngineSpec> = {}): EngineSpec {
  return { ...defaultConfig().engine, ...over };
}

/** Every layout the engine can be: one bank of 1 to 6, two at 90 degrees, and flat. */
const LAYOUTS: Partial<EngineSpec>[] = [1, 2, 3, 4, 5, 6].flatMap((n) => [
  { cylinders: n, vAngle: 0, crankType: 'shared' as const, exhaustLayout: 'merged' as const },
  { cylinders: 2 * n, vAngle: 90, crankType: 'shared' as const, exhaustLayout: 'perBank' as const },
  { cylinders: 2 * n, vAngle: 180, crankType: 'boxer' as const, exhaustLayout: 'perBank' as const },
]);

const gaps = (s: EngineSpec) => firingIntervalsOf(firingPlan(s));

describe('a layout of any size', () => {
  it('fires each cylinder once a cycle', () => {
    for (const over of LAYOUTS) {
      const s = spec(over);
      expect(validLayout(s), JSON.stringify(over)).toBe(true);
      const plan = firingPlan(s);
      expect(new Set(plan.offsets.map((o) => o.toFixed(3))).size, JSON.stringify(over)).toBe(s.cylinders);
      expect(gaps(s).reduce((a, b) => a + b, 0)).toBeCloseTo(720, 9);
      expect(crankPins(s).reduce((a, p) => a + p.cylinders.length, 0)).toBe(s.cylinders);
    }
    expect(validLayout(spec({ cylinders: 7, vAngle: 0 }))).toBe(false);
    expect(validLayout(spec({ cylinders: 14, vAngle: 90 }))).toBe(false);
  });

  it('fires as the real engines of its kind do', () => {
    expect(gaps(spec({ cylinders: 12, vAngle: 60 }))).toEqual(Array(12).fill(60));
    expect(crankPins(spec({ cylinders: 12, vAngle: 60 }))).toHaveLength(6);
    for (const g of gaps(spec({ cylinders: 10, vAngle: 72 }))) expect(g).toBeCloseTo(72, 9);
    expect(new Set(gaps(spec({ cylinders: 10, vAngle: 90 })).map(Math.round))).toEqual(new Set([54, 90]));
    expect(gaps(spec({ cylinders: 4, vAngle: 90 }))).toEqual([180, 270, 180, 90]);
    for (const n of [2, 8, 10, 12]) {
      const flat = spec({ cylinders: n, vAngle: 180, crankType: 'boxer' });
      expect(isBoxer(flat)).toBe(true);
      for (const g of gaps(flat)) expect(g).toBeCloseTo(720 / n, 9);
      expect(crankPins(flat)).toHaveLength(n);
    }
  });

  it('takes a firing order and intervals of its own, and ignores ones it cannot fire', () => {
    const own = spec({ cylinders: 6, vAngle: 0 });
    expect(firingOrderOf(firingPlan(own))).toEqual([1, 5, 3, 6, 2, 4]);
    expect(firingPlan({ ...own, firingOrder: [1, 4, 2, 6, 3, 5] }).offsets).toEqual([0, 240, 480, 120, 600, 360]);
    const bang = spec({ cylinders: 4, vAngle: 0, firingIntervals: [90, 90, 270, 270] });
    expect(firingPlan(bang).offsets).toEqual([0, 450, 90, 180]);
    for (const bad of [
      { firingOrder: [1, 2, 3] },
      { firingOrder: [1, 2, 2, 4, 5, 6] },
      { firingOrder: [1, 2, 3, 4, 5, 7] },
      { firingIntervals: [120, 120, 120, 120, 120, 100] },
      { firingIntervals: [250, 120, 120, 120, 120, -10] },
    ]) {
      expect(firingOrderProblem({ ...own, ...bad }), JSON.stringify(bad)).not.toBeNull();
      expect(firingPlan({ ...own, ...bad })).toEqual(firingPlan(own));
    }
  });

  it('is drawn with every rod its own length, its inlet laid out and its exhaust compiled', () => {
    const pipe = [makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.042 })];
    const collector = [makeSegment({ kind: 'pipe', length: 0.6, dIn: 0.06 })];
    for (const over of [...LAYOUTS, { cylinders: 6, vAngle: 0, firingOrder: [1, 4, 2, 6, 3, 5] }]) {
      const s = spec(over);
      const name = JSON.stringify(over);
      const mesh = new EngineMesh(s);
      const plan = firingPlan(s);
      for (let a = 0; a < 720; a += 45) {
        mesh.update(
          plan.offsets.map((o) => ({
            crankAngle: (((a - o) % 720) + 720) % 720,
            cylPressure: GAS.pAmb,
            cylTemp: GAS.tAmb,
            exLift: 0,
            inLift: 0,
          })),
          new Array(s.cylinders).fill(0),
        );
        for (let i = 0; i < s.cylinders; i++) expect(mesh.pose(i).rodLength, name).toBeCloseTo(s.rodLength, 9);
      }
      expect(inletLayout(s).runners, name).toHaveLength(s.cylinders);
      expect(validateGraph(compileExhaust(s, pipe, collector), s.cylinders), name).toEqual([]);
    }
  });

  it('is named, saved and read back with its firing order', () => {
    expect(engineFileName(spec({ cylinders: 12, vAngle: 60 }))).toBe('engine-v12');
    expect(engineFileName(spec({ cylinders: 4, vAngle: 90 }))).toBe('engine-v4');
    expect(engineFileName(spec({ cylinders: 12, vAngle: 180, crankType: 'boxer' }))).toBe('engine-flat-12');
    const cfg = defaultConfig();
    cfg.engine = spec({ cylinders: 6, vAngle: 0, firingOrder: [1, 4, 2, 6, 3, 5], firingIntervals: [90, 150, 120, 90, 150, 120] });
    const read = readEngineFile(engineFile(cfg), defaultConfig())!.config.engine;
    expect(read.firingOrder).toEqual([1, 4, 2, 6, 3, 5]);
    expect(read.firingIntervals).toEqual([90, 150, 120, 90, 150, 120]);
    cfg.engine = { ...cfg.engine, firingOrder: [1, 1, 2, 3, 4, 5] };
    expect(readEngineFile(engineFile(cfg), defaultConfig())!.config.engine.firingOrder).toBeNull();
  });

  it('runs a V12 in the Wasm build', () => {
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, cylinders: 12, vAngle: 60, exhaustLayout: 'perBank', throttle: 1, rpm: 4000 };
    cfg.graph = compileExhaust(cfg.engine, cfg.pipe, cfg.collector);
    const sim = new Sim(48000, cfg);
    const out = sim.render(24000);
    let peak = 0;
    for (const v of out) {
      expect(Number.isFinite(v)).toBe(true);
      peak = Math.max(peak, Math.abs(v));
    }
    expect(peak).toBeGreaterThan(1e-3);
  });
});
