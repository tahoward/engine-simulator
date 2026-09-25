/**
 * `setControls` is the worklet's per-block fast path for throttle, rpm and load. It must do exactly
 * what `setEngine` does with the same three values, or the sound would depend on which path a change
 * happened to take.
 */

import { describe, expect, it } from 'vitest';
import { PIPE_PRESETS, defaultConfig, type EngineSpec } from '../src/model/spec.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';

const FS = 48000;

function pair(engine: Partial<EngineSpec>): [EngineSim, EngineSim] {
  const make = () => {
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, ...engine };
    cfg.pipe = PIPE_PRESETS[1]!.build();
    const sim = new EngineSim(FS, cfg);
    sim.render(FS / 4);
    return sim;
  };
  return [make(), make()];
}

describe('setControls', () => {
  for (const freeRunning of [false, true]) {
    it(`matches setEngine bit for bit (${freeRunning ? 'free-running' : 'fixed rpm'})`, () => {
      const [a, b] = pair({ freeRunning, throttle: 0.3, rpm: 3000, load: 0.4 });
      a.setEngine({ throttle: 0.8, rpm: 5200, load: 0.7 });
      b.setControls(0.8, 5200, 0.7);
      expect(b.render(FS / 2)).toEqual(a.render(FS / 2));
    });
  }

  it('changes nothing when the values have not moved', () => {
    const [a, b] = pair({ throttle: 0.5, rpm: 4000, load: 0.3 });
    b.setControls(0.5, 4000, 0.3);
    expect(b.render(FS / 4)).toEqual(a.render(FS / 4));
  });
});
