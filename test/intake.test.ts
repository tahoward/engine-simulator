/**
 * The intake runners: a column of air per cylinder, whose momentum rams the charge in and whose length
 * tunes where it does so.
 */

import { describe, expect, it } from 'vitest';

import { EngineSim } from '../src/audio/worklet/engineSim.js';
import { DEFAULT_ENGINE, ENGINE_PRESETS, GAS, defaultConfig, displacement, fittedExhaust, intakeRunnerOf, type EngineSpec } from '../src/model/spec.js';

const FS = 48000;

/** A 6.2 litre pushrod V8 in the proportions of a Chevrolet LT2, with a late-closing cam. */
const V8: Partial<EngineSpec> = {
  cylinders: 8,
  vAngle: 90,
  crankType: 'crossplane',
  exhaustLayout: 'perBank',
  bore: 0.10325,
  stroke: 0.092,
  rodLength: 0.1556,
  compressionRatio: 11.5,
  exValveDia: 0.0404,
  inValveDia: 0.054,
  maxLift: 0.0145,
  evo: 104,
  evc: 384,
  ivo: 338,
  ivc: 614,
  revLimit: 6600,
};

/** Volumetric efficiency and mean gas torque at full throttle, held at `rpm`. */
function breathe(rpm: number, over: Partial<EngineSpec> = {}): { ve: number; torque: number } {
  const spec = { ...DEFAULT_ENGINE, ...V8, ...over };
  const cfg = defaultConfig();
  cfg.engine = { ...spec, freeRunning: false, throttle: 1, rpm, combustionVariability: 0 };
  const ex = fittedExhaust(spec);
  // Long-tube headers for this valve, 44 mm primaries, rather than the fitted exhaust's narrower ones.
  for (const seg of ex.pipe) seg.dIn = seg.dOut = 0.044;
  cfg.pipe = ex.pipe;
  cfg.collector = ex.collector;
  const sim = new EngineSim(FS, cfg);
  sim.render(FS);
  const inner = sim as unknown as {
    cyls: { angle: number; mass: number; burnedFraction: number }[];
    torqueLast: number;
  };
  const cyl = inner.cyls[0]!;
  let prev = cyl.angle;
  let trapped = 0;
  let cycles = 0;
  let torque = 0;
  const n = FS / 2;
  for (let i = 0; i < n; i++) {
    sim.render(1);
    torque += inner.torqueLast;
    const a = cyl.angle;
    if (a >= prev ? spec.ivc > prev && spec.ivc <= a : spec.ivc > prev || spec.ivc <= a) {
      trapped += cyl.mass * (1 - cyl.burnedFraction);
      cycles++;
    }
    prev = a;
  }
  const full = (GAS.pAmb * displacement(spec)) / (GAS.R * GAS.tAmb);
  return { ve: trapped / cycles / full, torque: torque / n };
}

describe('intake runners', () => {
  it('are sized from the valves and tuned from the rev range', () => {
    const r = intakeRunnerOf({ ...DEFAULT_ENGINE, ...V8 } as EngineSpec);
    expect(r.diameter).toBeCloseTo(0.9 * 0.054, 9);
    // Quarter-wave at 2.3 times the crank speed at three quarters of 6600 rpm.
    expect(r.length).toBeGreaterThan(0.43);
    expect(r.length).toBeLessThan(0.47);
    const set = intakeRunnerOf({ ...DEFAULT_ENGINE, ...V8, intakeRunnerLength: 0.3, intakeRunnerDia: 0.05 } as EngineSpec);
    expect(set).toEqual({ length: 0.3, diameter: 0.05 });
  });

  /**
   * The ramming: at the speed it is tuned for, three quarters of the rev limit, the runner fills the
   * cylinder to nearly 100%, about 15 points more than a stub a few centimetres long, which rams
   * almost nothing; and it fills best there, falling away either side.
   */
  it('ram the charge in, most at the speed they are tuned for', () => {
    const tuned = breathe(4950).ve;
    expect(tuned).toBeGreaterThan(0.95);
    expect(tuned).toBeGreaterThan(breathe(4950, { intakeRunnerLength: 0.08 }).ve + 0.1);
    expect(tuned).toBeGreaterThan(breathe(4000).ve);
    expect(tuned).toBeGreaterThan(breathe(5800).ve);
  });

  /** A long runner is tuned low and a short one high, so each wins at its own end of the range. */
  it('move the torque with their length', () => {
    const long = { intakeRunnerLength: 0.8 };
    const short = { intakeRunnerLength: 0.25 };
    expect(breathe(3500, long).torque).toBeGreaterThan(breathe(3500, short).torque);
    expect(breathe(6450, short).torque).toBeGreaterThan(breathe(6450, long).torque);
  });
});

describe('headers', () => {
  /**
   * Equal-length headers scavenge: the wave each pulse sends back from the merge pulls fresh charge
   * through the cylinder during the overlap. On the LT6, with 70 degrees of overlap and primaries tuned
   * for 8400 rpm, that fills the cylinder several points more than a manifold along the ports does.
   */
  it('fill the cylinder more than a manifold at the speed they are tuned for', () => {
    const lt6 = ENGINE_PRESETS.find((p) => p.name === 'V8, Chevrolet LT6')!;
    const fill = (exhaustHeaders: boolean) => {
      const spec = { ...DEFAULT_ENGINE, ...lt6.engine, exhaustHeaders } as EngineSpec;
      const cfg = defaultConfig();
      cfg.engine = { ...spec, freeRunning: false, throttle: 1, rpm: 8400, combustionVariability: 0 };
      cfg.pipe = lt6.pipe();
      cfg.collector = lt6.collector!();
      const sim = new EngineSim(FS, cfg);
      sim.render(FS / 2);
      const cyl = (sim as unknown as { cyls: { angle: number; mass: number; burnedFraction: number }[] }).cyls[0]!;
      let prev = cyl.angle;
      let trapped = 0;
      let cycles = 0;
      for (let i = 0; i < FS / 2; i++) {
        sim.render(1);
        const a = cyl.angle;
        if (a >= prev ? spec.ivc > prev && spec.ivc <= a : spec.ivc > prev || spec.ivc <= a) {
          trapped += cyl.mass * (1 - cyl.burnedFraction);
          cycles++;
        }
        prev = a;
      }
      return trapped / cycles / ((GAS.pAmb * displacement(spec)) / (GAS.R * GAS.tAmb));
    };
    expect(fill(true)).toBeGreaterThan(fill(false) + 0.03);
  });
});

describe('runners in one shared kernel', () => {
  /**
   * The cell kernel reproduces the TypeScript loops bit for bit, so runners stepped together in one
   * shared kernel must render exactly as runners on the TypeScript path do: the batching changes how
   * the loops are called, never what they compute.
   */
  it('render bit for bit as the TypeScript path', () => {
    const render = (useKernel: boolean) => {
      const spec = { ...DEFAULT_ENGINE, ...V8 };
      const cfg = defaultConfig();
      cfg.engine = { ...spec, freeRunning: false, throttle: 1, rpm: 5000 };
      const ex = fittedExhaust(spec);
      cfg.pipe = ex.pipe;
      cfg.collector = ex.collector;
      return new EngineSim(FS, cfg, { useKernel, useJunctionKernel: false }).render(FS / 4);
    };
    expect(render(true)).toEqual(render(false));
  });
});

describe('variable valve timing', () => {
  const lt6 = ENGINE_PRESETS.find((p) => p.name === 'V8, Chevrolet LT6')!;
  const build = (over: Partial<EngineSpec>) => {
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, ...lt6.engine, ...over } as EngineSpec;
    cfg.pipe = lt6.pipe();
    cfg.collector = lt6.collector!();
    return new EngineSim(FS, cfg);
  };
  const phase = (sim: EngineSim) => sim.snapshot().intakeCamAdvance;
  const torqueAt = (rpm: number, over: Partial<EngineSpec>) => {
    const sim = build({ freeRunning: false, throttle: 1, rpm, combustionVariability: 0, ...over });
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

  /** A cam tuned for 8400 rpm gives up the mid-range; advancing it there gives it back. */
  it('lifts the mid-range of an engine cammed for the top end', () => {
    const fixed = { vvtIntakeLow: 0, vvtExhaustLow: 0 };
    expect(torqueAt(4500, {})).toBeGreaterThan(1.1 * torqueAt(4500, fixed));
    // At the top the map has the cams at rest, so it changes nothing there.
    expect(torqueAt(8400, {}) / torqueAt(8400, fixed)).toBeCloseTo(1, 2);
  });

  it('keeps the cams at rest at idle, and moves them under load', () => {
    const idle = build({ freeRunning: true });
    idle.render(FS * 2);
    expect(phase(idle)).toBeCloseTo(0, 9);
    const pulling = build({ freeRunning: false, throttle: 1, rpm: 3000 });
    pulling.render(FS / 2);
    expect(phase(pulling)).toBeGreaterThan(30);
  });

  it('brings a cam back to rest when its map is set to nothing', () => {
    const sim = build({ freeRunning: false, throttle: 1, rpm: 3000 });
    sim.render(FS / 2);
    sim.setEngine({ vvtIntakeLow: 0, vvtExhaustLow: 0 });
    sim.render(FS / 2);
    expect(phase(sim)).toBeCloseTo(0, 9);
  });
});

describe('two-stage intake', () => {
  const lt6 = ENGINE_PRESETS.find((p) => p.name === 'V8, Chevrolet LT6')!;
  const build = (over: Partial<EngineSpec>) => {
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, ...lt6.engine, freeRunning: false, throttle: 1, combustionVariability: 0, ...over } as EngineSpec;
    cfg.pipe = lt6.pipe();
    cfg.collector = lt6.collector!();
    return new EngineSim(FS, cfg);
  };
  type Inner = { torqueLast: number; intakeLong: { recoveries: number }; intakeShort: { recoveries: number } };
  const meanTorque = (sim: EngineSim, samples: number) => {
    const inner = sim as unknown as Inner;
    let t = 0;
    for (let i = 0; i < samples; i++) {
      sim.render(1);
      t += inner.torqueLast;
    }
    return t / samples;
  };
  const torqueAt = (rpm: number, over: Partial<EngineSpec>) => {
    const sim = build({ rpm, ...over });
    // The walls and the waves take a couple of seconds to settle at a new speed.
    sim.render(2 * FS);
    return meanTorque(sim, FS / 2);
  };
  const switchRpm = lt6.engine.intakeSwitchRpm!;

  /** Long runners fill it below the switch speed; above it, it has the short ones'. */
  it('has the long runners’ torque below its switch speed and the short runners’ above it', () => {
    const shortOnly = { intakeRunnerLength: 0.33, intakeRunnerShortLength: 0 };
    expect(torqueAt(7800, {})).toBeGreaterThan(1.05 * torqueAt(7800, shortOnly));
    expect(torqueAt(8400, {}) / torqueAt(8400, shortOnly)).toBeCloseTo(1, 2);
  });

  it('switches at its switch speed, back a little below it, and carries on smoothly', () => {
    const sim = build({ rpm: switchRpm - 100 });
    sim.render(FS);
    expect(sim.snapshot().shortRunners).toBe(false);
    const before = meanTorque(sim, FS / 4);

    sim.setEngine({ rpm: switchRpm + 100 });
    sim.render(1);
    expect(sim.snapshot().shortRunners).toBe(true);
    const across = meanTorque(sim, FS / 4);
    // Inside the hysteresis band it stays on the short runners.
    sim.setEngine({ rpm: switchRpm - 100 });
    sim.render(FS / 10);
    expect(sim.snapshot().shortRunners).toBe(true);
    sim.setEngine({ rpm: switchRpm - 200 });
    sim.render(1);
    expect(sim.snapshot().shortRunners).toBe(false);
    sim.render(FS / 4);

    const inner = sim as unknown as Inner;
    expect(inner.intakeLong.recoveries + inner.intakeShort.recoveries).toBe(0);
    // Near the switch speed the two sets make about the same torque, so the switch is no jolt.
    expect(Math.abs(across / before - 1)).toBeLessThan(0.1);
  });
});
