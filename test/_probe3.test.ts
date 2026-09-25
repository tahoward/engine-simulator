import { describe, it } from 'vitest';
import { EngineSim } from '../src/audio/worklet/engineSim.js';
import { compileCollectorLayout } from '../src/model/exhaustGraph.js';
import { defaultConfig, makeSegment, type EngineSpec } from '../src/model/spec.js';
declare const process: { stdout: { write(s: string): void } };
const FS = 48000;
const V8: Partial<EngineSpec> = { cylinders: 8, vAngle: 90, crankType: 'flatplane', exhaustLayout: 'perBank' };
describe('probe3', () => {
  it('junction residual', () => {
    const geos = [
      () => [makeSegment({ kind: 'cone', length: 0.2, dIn: 0.042, dOut: 0.13 })],
      () => [makeSegment({ kind: 'cone', length: 0.5, dIn: 0.042, dOut: 0.09 })],
      () => [makeSegment({ kind: 'cone', length: 1.2, dIn: 0.05, dOut: 0.06 })],
      () => [makeSegment({ kind: 'chamber', length: 0.34, dIn: 0.042, dOut: 0.13 }), makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.04 })],
    ];
    for (const g of geos) {
      const cfg = defaultConfig();
      cfg.engine = { ...cfg.engine, ...V8, rpm: 8500, throttle: 1, freeRunning: false } as EngineSpec;
      cfg.pipe = [makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.042 })];
      cfg.collector = g();
      cfg.graph = compileCollectorLayout(cfg.engine, cfg.pipe, cfg.collector);
      const sim = new EngineSim(FS, cfg);
      sim.render(FS);
      process.stdout.write(`\nresidual ${sim.pipeSolver.junctionResidual.toFixed(4)}`);
    }
  });
});
