import { describe, expect, it } from 'vitest';

import { presetConfig } from '../bench/presetConfig.js';
import { Sim } from '../src/audio/worklet/sim.js';
import { solverGraph, compileExhaust } from '../src/model/exhaustGraph.js';
import { ENGINE_PRESETS, defaultConfig } from '../src/model/spec.js';
import { configSources } from '../src/scene/soundSources.js';

describe('where the engine makes its sound', () => {
  it('places every mouth the solver radiates from, on every preset', () => {
    for (const preset of ENGINE_PRESETS) {
      const cfg = presetConfig(preset);
      const graph = cfg.graph ?? compileExhaust(cfg.engine, cfg.pipe, cfg.collector, 0);
      const mouths = solverGraph(graph).ducts.filter((d) => d.to.kind === 'mouth').map((d) => d.id);
      const sources = configSources(cfg);
      expect(sources.mouths.map((m) => m.duct).sort(), preset.name).toEqual([...mouths].sort());
      for (const m of sources.mouths) expect(m.position.every(Number.isFinite), preset.name).toBe(true);
      expect(sources.intake && sources.engine, preset.name).toBeTruthy();
    }
  });

  it('puts a tailpipe at the end of the pipe that is drawn there', () => {
    // The default single's megaphone runs out from the side of the head.
    const sources = configSources(defaultConfig());
    expect(sources.mouths).toHaveLength(1);
    expect(sources.mouths[0]!.position[0]).toBeGreaterThan(0.5);
  });

  it('brings the Z06’s tailpipes together in the middle', () => {
    const lt6 = ENGINE_PRESETS.find((p) => p.name.includes('LT6'))!;
    const [a, b] = configSources(presetConfig(lt6)).mouths;
    expect(Math.abs(a!.position[0] - b!.position[0])).toBeLessThan(0.25);
    expect(Math.abs(a!.position[0] + b!.position[0])).toBeLessThan(0.02);
  });

  it('is heard from where the listener is: twice as far, about half as loud', () => {
    const cfg = defaultConfig();
    const mouth = configSources(cfg).mouths[0]!;
    const at = mouth.position;
    cfg.engine = { ...cfg.engine, groundReflection: 0, combustionVariability: 0 };
    cfg.sources = { mouths: [mouth], intake: at, engine: at, turbo: at };
    const rms = (distance: number) => {
      const sim = new Sim(48000, { ...cfg, listener: [at[0] + distance, at[1], at[2]] });
      const out = new Float32Array(48000);
      sim.renderInto(out);
      sim.renderInto(out);
      return Math.sqrt(out.reduce((s, v) => s + v * v, 0) / out.length);
    };
    expect(rms(1) / rms(2)).toBeGreaterThan(1.85);
    expect(rms(1) / rms(2)).toBeLessThan(2.15);
  });
});
