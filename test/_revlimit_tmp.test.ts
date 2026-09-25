import { it } from 'vitest';
import { EngineSim } from '../src/audio/worklet/engineSim';
import { defaultConfig, ENGINE_PRESETS } from '../src/model/spec';
it('limiter', () => {
  for (const p of ENGINE_PRESETS) {
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, ...p.engine };
    cfg.pipe = p.pipe(); cfg.collector = p.collector ? p.collector() : [];
    const sim = new EngineSim(48000, cfg);
    sim.render(4800);
    sim.setEngine({ rpm: cfg.engine.revLimit + 500 });
    let cuts = 0, prev = false, lo = 1e9, hi = 0;
    for (let i = 0; i < 48000 * 3; i++) {
      sim.tick();
      const s = (sim as any).limiterCut as boolean;
      if (s && !prev) cuts++;
      prev = s;
      if (i > 48000 * 1.5) { lo = Math.min(lo, sim.rpmInstant); hi = Math.max(hi, sim.rpmInstant); }
    }
    console.log(`${p.name.padEnd(36)} limit ${cfg.engine.revLimit} cuts ${cuts} range ${lo.toFixed(0)}-${hi.toFixed(0)} readout ${sim.rpm.toFixed(0)}`);
  }
}, 600000);
