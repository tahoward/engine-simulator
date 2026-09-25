import { describe, it } from 'vitest';
import { ENGINE_PRESETS, defaultConfig, defaultCollector, type EngineSpec } from '../src/model/spec.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';
declare const process: { stdout: { write(s: string): void } };
const FS = 48000;
describe('probe4', () => {
  it('throttle response, free-running', () => {
    for (const name of ['Inline four', 'Inline three']) {
      for (const load of [0.2, 0.46]) {
        const p = ENGINE_PRESETS.find((x) => x.name.startsWith(name))!;
        const cfg = defaultConfig();
        cfg.engine = { ...cfg.engine, ...p.engine, freeRunning: true, load, throttle: 0.1 } as EngineSpec;
        cfg.pipe = p.pipe();
        cfg.collector = p.collector ? p.collector() : defaultCollector();
        const sim = new EngineSim(FS, cfg);
        const out: string[] = [];
        for (const thr of [0.1, 0.3, 0.6, 1.0, 0.3]) {
          sim.setControls(thr, cfg.engine.rpm, load);
          sim.render(FS * 3);
          out.push(`${thr}:${sim.rpm.toFixed(0)}`);
        }
        process.stdout.write(`\n${name.padEnd(14)} load ${load}  ${out.join('  ')}`);
      }
    }
  });
});
