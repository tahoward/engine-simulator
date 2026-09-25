import { describe, it } from 'vitest';
import { ENGINE_PRESETS, defaultConfig, defaultCollector, type EngineSpec } from '../src/model/spec.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';

const FS = 48000;
declare const process: { stdout: { write(s: string): void } };

function run(name: string, over: Partial<EngineSpec>, seconds: number) {
  const p = ENGINE_PRESETS.find((x) => x.name.startsWith(name))!;
  const cfg = defaultConfig();
  cfg.engine = { ...cfg.engine, ...p.engine, ...over } as EngineSpec;
  cfg.pipe = p.pipe();
  cfg.collector = p.collector ? p.collector() : defaultCollector();
  const sim = new EngineSim(FS, cfg);
  sim.render(FS * 2);
  const inner = sim as unknown as { cyls: any[]; spec: EngineSpec; plenum: any };
  const spec = inner.spec;
  const prev = inner.cyls.map((c) => c.angle);
  let cycles = 0;
  let misfires = 0;
  let resid = 0;
  let nres = 0;
  let map = 0;
  let rpm = 0;
  const n = FS * seconds;
  for (let i = 0; i < n; i++) {
    sim.render(1);
    map += inner.plenum.pressure;
    rpm += sim.rpm;
    inner.cyls.forEach((c, k) => {
      const a = c.angle;
      const cross = (t: number) => (a >= prev[k] ? t > prev[k] && t <= a : t > prev[k] || t <= a);
      if (cross(90)) {
        cycles++;
        if (c.burned < 0.05) misfires++;
      }
      if (cross(spec.ivc)) {
        resid += c.burnedFraction;
        nres++;
      }
      prev[k] = a;
    });
  }
  process.stdout.write(
    `\n${name.padEnd(16)} ${JSON.stringify(over).padEnd(48)} misfire ${((100 * misfires) / cycles).toFixed(1)}%  resid ${((100 * resid) / nres).toFixed(0)}%  MAP ${(map / n / 1e5).toFixed(2)}  rpm ${(rpm / n).toFixed(0)}`,
  );
}

describe('probe2', () => {
  it('presets', () => {
    run('V8, overcammed', { freeRunning: false }, 6);
    run('V8, overcammed', { freeRunning: false, throttle: 0.06 }, 6);
    run('V8, overcammed', { freeRunning: false, throttle: 0.12 }, 6);
    run('V8, overcammed', { freeRunning: true }, 6);
    run('V8, cross', { freeRunning: false, rpm: 1000, throttle: 0.08 }, 6);
    run('V8, cross', { freeRunning: true, throttle: 0.07, load: 0 }, 6);
    run('Single', { freeRunning: true, throttle: 0.05, load: 0 }, 6);
  });
});
