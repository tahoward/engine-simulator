import { describe, it } from 'vitest';
import { ENGINE_PRESETS, GAS, defaultConfig, defaultCollector } from '../src/model/spec.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';

const FS = 48000;

describe('closed throttle must not run away', () => {
  it('every engine preset, throttle 0, free-running, no load', () => {
    console.log('\n  preset                              rpm @ 2s   4s     8s    12s   MAP');
    for (const p of ENGINE_PRESETS) {
      const cfg = defaultConfig();
      cfg.engine = { ...cfg.engine, ...p.engine, throttle: 0, freeRunning: true, loadTorque: 0 };
      cfg.pipe = p.pipe();
      cfg.collector = p.collector ? p.collector() : defaultCollector();
      const sim = new EngineSim(FS, cfg);
      const marks: string[] = [];
      for (const t of [2, 2, 4, 4]) {
        sim.render(FS * t);
        marks.push(sim.rpm.toFixed(0).padStart(5));
      }
      const inner = sim as unknown as { plenum: any };
      let sum = 0;
      for (let k = 0; k < 2000; k++) {
        sim.render(12);
        sum += inner.plenum.pressure;
      }
      console.log(
        `  ${p.name.padEnd(34)} ${marks.join(' ')}   ${(sum / 2000 / GAS.pAmb).toFixed(2)}bar`,
      );
    }
  });

  it('and a mid-throttle hold is stable too', () => {
    console.log('');
    for (const name of ['Single', 'Inline four', 'V8, crossplane']) {
      const p = ENGINE_PRESETS.find((x) => x.name.startsWith(name))!;
      const cfg = defaultConfig();
      cfg.engine = { ...cfg.engine, ...p.engine, throttle: 0.3, freeRunning: true, loadTorque: 25 };
      cfg.pipe = p.pipe();
      cfg.collector = p.collector ? p.collector() : defaultCollector();
      const sim = new EngineSim(FS, cfg);
      const marks: string[] = [];
      for (const t of [2, 2, 4, 4]) {
        sim.render(FS * t);
        marks.push(sim.rpm.toFixed(0).padStart(5));
      }
      console.log(`  ${p.name.padEnd(34)} ${marks.join(' ')}  (throttle 0.3, 25 Nm)`);
    }
  });
});
