import { describe, it } from 'vitest';
import { ENGINE_PRESETS, defaultConfig, defaultCollector, type EngineSpec } from '../src/model/spec.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';

const FS = 48000;
declare const process: { stdout: { write(s: string): void } };

function probe(name: string, over: Partial<EngineSpec>, variability = 0) {
  const p = ENGINE_PRESETS.find((x) => x.name.startsWith(name))!;
  const cfg = defaultConfig();
  cfg.engine = { ...cfg.engine, ...p.engine, freeRunning: false, combustionVariability: variability, ...over } as EngineSpec;
  cfg.pipe = p.pipe();
  cfg.collector = p.collector ? p.collector() : defaultCollector();
  const sim = new EngineSim(FS, cfg);
  sim.render(FS);
  const inner = sim as unknown as { cyls: any[]; spec: EngineSpec; plenum: any; torqueLast: number };
  const cyl = inner.cyls[0];
  const spec = inner.spec;
  let prev = cyl.angle;
  const angles: number[] = [];
  let fired = 0;
  let cycles = 0;
  let torque = 0;
  let map = 0;
  const n = FS * 2;
  for (let i = 0; i < n; i++) {
    sim.render(1);
    map += inner.plenum.pressure;
    torque += inner.torqueLast;
    const a = cyl.angle;
    // 90 ATDC: the burn angle for this cycle is latched by now.
    const t = 90;
    const crossed = a >= prev ? t > prev && t <= a : t > prev || t <= a;
    if (crossed) {
      cycles++;
      if (cyl.burnAngle > 0) {
        fired++;
        angles.push(cyl.burnAngle);
      }
    }
    prev = a;
  }
  const mean = angles.reduce((s, v) => s + v, 0) / Math.max(angles.length, 1);
  process.stdout.write(
    `\n${name.padEnd(16)} ${JSON.stringify(over).padEnd(46)} burn ${mean.toFixed(1).padStart(5)}°  fired ${fired}/${cycles}  torque ${(torque / n).toFixed(1).padStart(6)} Nm  MAP ${(map / n / 1e5).toFixed(2)}  rpm ${sim.rpm.toFixed(0)}`,
  );
}

describe('probe', () => {
  it('burn angle', () => {
    for (const rpm of [1000, 2000, 3200, 4500, 6000]) probe('Single', { throttle: 1, rpm });
    for (const throttle of [1, 0.5, 0.3, 0.2, 0.12, 0.08]) probe('Single', { throttle, rpm: 3200 });
    for (const lambda of [0.8, 0.9, 1, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6]) probe('Single', { throttle: 1, rpm: 3200, lambda });
    probe('Single', { throttle: 0, rpm: 3200 });
    probe('Single', { throttle: 0, rpm: 3200, fuelCut: false });
    probe('Inline four', { throttle: 1, rpm: 4000 });
    probe('V8, cross', { throttle: 1, rpm: 4000 });
    probe('V8, cross', { throttle: 0.07, rpm: 900 }, 1);
    probe('V8, overcammed', {}, 1);
  });
});
