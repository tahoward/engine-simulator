/**
 * Every engine preset loads idling: in neutral, on a throttle found for it that holds
 * `PRESET_IDLE_RPM`. Run free, each has to settle there rather than stall or run away.
 */

import { describe, expect, it } from 'vitest';

import { EngineSim } from '../src/audio/worklet/engineSim.js';
import { ENGINE_PRESETS, PRESET_IDLE_RPM, defaultCollector, defaultConfig } from '../src/model/spec.js';

const FS = 48000;

describe('presets idle', () => {
  it.each(ENGINE_PRESETS.map((p) => p.name))('%s settles near the idle speed', (name) => {
    const preset = ENGINE_PRESETS.find((p) => p.name === name)!;
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, ...preset.engine, freeRunning: true };
    cfg.pipe = preset.pipe();
    cfg.collector = preset.collector ? preset.collector() : defaultCollector();
    expect(cfg.engine.rpm).toBe(PRESET_IDLE_RPM);
    expect(cfg.engine.load).toBe(0);

    const sim = new EngineSim(FS, cfg);
    sim.render(FS * 3);
    // Averaged, because an idle hunts: the overcammed V8 misfires a quarter of its cycles.
    let sum = 0;
    const reads = 20;
    for (let i = 0; i < reads; i++) {
      sim.render(FS / 10);
      sum += sim.rpm;
    }
    const mean = sum / reads;
    expect(mean).toBeGreaterThan(PRESET_IDLE_RPM - 150);
    expect(mean).toBeLessThan(PRESET_IDLE_RPM + 200);
  });
});
