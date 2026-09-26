/**
 * Cam profile switching, as VTEC does it: a mild lobe below the switch speed and a wild one above it.
 */

import { describe, expect, it } from 'vitest';

import { EngineSim } from '../src/audio/worklet/engineSim.js';
import { ENGINE_PRESETS, defaultConfig, type EngineSpec } from '../src/model/spec.js';

const FS = 48000;

describe('cam profile switching', () => {
  const f20c = ENGINE_PRESETS.find((p) => p.name === 'Inline four, Honda F20C')!;
  const switchRpm = f20c.engine.camSwitchRpm!;
  const build = (over: Partial<EngineSpec>) => {
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, ...f20c.engine, freeRunning: false, throttle: 1, combustionVariability: 0, ...over } as EngineSpec;
    cfg.pipe = f20c.pipe();
    cfg.collector = f20c.collector!();
    return new EngineSim(FS, cfg);
  };
  const torqueAt = (rpm: number, over: Partial<EngineSpec>) => {
    const sim = build({ rpm, ...over });
    // The walls and the waves take a couple of seconds to settle at a new speed.
    sim.render(2 * FS);
    const inner = sim as unknown as { torqueLast: number };
    let t = 0;
    for (let i = 0; i < FS / 2; i++) {
      sim.render(1);
      t += inner.torqueLast;
    }
    return t / (FS / 2);
  };
  /** The same engine on its high-speed cam alone. */
  const highOnly = {
    camSwitchRpm: 0,
    evo: f20c.engine.highEvo!,
    evc: f20c.engine.highEvc!,
    ivo: f20c.engine.highIvo!,
    ivc: f20c.engine.highIvc!,
    maxLift: f20c.engine.highMaxLift!,
  };

  /** The mild lobe gives back the low end the wild one costs, and the wild one keeps the top end. */
  it('has the mild lobe’s low end and the wild lobe’s top end', () => {
    expect(torqueAt(3000, {})).toBeGreaterThan(1.2 * torqueAt(3000, highOnly));
    expect(torqueAt(8000, {}) / torqueAt(8000, highOnly)).toBeCloseTo(1, 2);
  });

  it('switches at its switch speed and back a little below it', () => {
    const sim = build({ rpm: switchRpm - 100 });
    sim.render(FS / 10);
    expect(sim.snapshot().highCam).toBe(false);
    sim.setEngine({ rpm: switchRpm + 100 });
    sim.render(1);
    expect(sim.snapshot().highCam).toBe(true);
    // Inside the hysteresis band it stays on the high lobes.
    sim.setEngine({ rpm: switchRpm - 100 });
    sim.render(FS / 10);
    expect(sim.snapshot().highCam).toBe(true);
    sim.setEngine({ rpm: switchRpm - 200 });
    sim.render(1);
    expect(sim.snapshot().highCam).toBe(false);
  });

  it('opens the valves to the high lobe’s lift only on it', () => {
    const peakLift = (rpm: number) => {
      const sim = build({ rpm });
      let peak = 0;
      for (let i = 0; i < 200; i++) {
        sim.render(FS / 1000);
        peak = Math.max(peak, sim.snapshot().banks[0]!.inLift);
      }
      return peak;
    };
    expect(peakLift(switchRpm - 1000)).toBeCloseTo(f20c.engine.maxLift!, 3);
    expect(peakLift(switchRpm + 1000)).toBeCloseTo(f20c.engine.highMaxLift!, 3);
  });
});
