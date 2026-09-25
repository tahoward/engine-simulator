import { describe, it } from 'vitest';
import { ENGINE_PRESETS, defaultConfig, defaultCollector, type EngineSpec } from '../src/model/spec.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';
declare const process: { stdout: { write(s: string): void } };
const FS = 48000;
describe('probe5', () => {
  it('all presets bog-out', () => {
    for (const p of ENGINE_PRESETS) {
      const out: string[] = [];
      for (const load of [0.3, 0.6]) {
        const cfg = defaultConfig();
        cfg.engine = { ...cfg.engine, ...p.engine, freeRunning: true, load, throttle: 0.1 } as EngineSpec;
        cfg.pipe = p.pipe();
        cfg.collector = p.collector ? p.collector() : defaultCollector();
        const sim = new EngineSim(FS, cfg);
        sim.render(FS * 2);
        const a = sim.rpm;
        sim.setControls(1, cfg.engine.rpm, load);
        sim.render(FS * 3);
        out.push(`load ${load}: ${a.toFixed(0).padStart(5)} -> ${sim.rpm.toFixed(0).padStart(5)}`);
      }
      process.stdout.write(`\n${p.name.padEnd(34)} ${out.join('   ')}`);
    }
  });
});
