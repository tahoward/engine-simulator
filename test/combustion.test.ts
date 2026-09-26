/**
 * The flame: how long each charge takes to burn, and what the mixture it burns is.
 *
 * The burn duration is predicted from the flame speed of the charge the spark finds, so it has to
 * move the way a real engine's does: a little longer as the engine speeds up, a lot longer at part
 * throttle and lean. The mixture is carried as fuel and air through the manifold and the cylinder,
 * so λ sets how much heat a charge can release, and cutting the fuel leaves nothing to burn.
 */

import { describe, expect, it } from 'vitest';

import { burnAngle, laminarFlameSpeed, laminarSpeedBase } from '../src/audio/worklet/cylinder.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';
import { DEFAULT_ENGINE, ENGINE_PRESETS, PIPE_PRESETS, defaultConfig, type EngineSpec } from '../src/model/spec.js';

const FS = 48000;

/** Crank speed, rad/s, at which `spec` has mean piston speed `sp` (m/s). */
function omegaAt(spec: EngineSpec, sp: number): number {
  return (Math.PI * sp) / spec.stroke;
}

function single(over: Partial<EngineSpec>): EngineSim {
  const cfg = defaultConfig();
  cfg.engine = { ...cfg.engine, freeRunning: false, combustionVariability: 0, ...over };
  cfg.pipe = PIPE_PRESETS[1]!.build();
  return new EngineSim(FS, cfg);
}

interface Running {
  /** Mean burn duration of the cycles that fired, deg. */
  burn: number;
  /** Fraction of cycles that released heat. */
  fired: number;
  /** Mean gas torque, N*m. */
  torque: number;
}

/** Settle for a second, then watch cylinder 1 for `seconds`. */
function watch(sim: EngineSim, seconds: number): Running {
  sim.render(FS);
  const inner = sim as unknown as { cyls: { angle: number; burnAngle: number; burned: number }[]; torqueLast: number };
  const cyl = inner.cyls[0]!;
  let prev = cyl.angle;
  let cycles = 0;
  let fired = 0;
  let burn = 0;
  let torque = 0;
  const n = Math.round(FS * seconds);
  for (let i = 0; i < n; i++) {
    sim.render(1);
    torque += inner.torqueLast;
    const a = cyl.angle;
    // By 90 ATDC the cycle's burn duration is latched and most of its heat released.
    if (a >= prev ? prev < 90 && a >= 90 : prev < 90 || a >= 90) {
      cycles++;
      if (cyl.burned > 0.05) {
        fired++;
        burn += cyl.burnAngle;
      }
    }
    prev = a;
  }
  return { burn: fired > 0 ? burn / fired : 0, fired: fired / Math.max(cycles, 1), torque: torque / n };
}

describe('laminar flame speed', () => {
  it('matches the correlation at room conditions and peaks slightly rich', () => {
    // B_m + B_φ (φ - φ_m)^2 at φ = 1: 0.305 - 0.549 * 0.21^2.
    expect(laminarFlameSpeed(1, 298, 101325, 0)).toBeCloseTo(0.2808, 4);
    expect(laminarSpeedBase(1.21)).toBeGreaterThan(laminarSpeedBase(1));
    expect(laminarSpeedBase(1.21)).toBeGreaterThan(laminarSpeedBase(1.4));
  });

  it('rises steeply with temperature and falls gently with pressure', () => {
    const cold = laminarFlameSpeed(1, 400, 10e5, 0);
    const hot = laminarFlameSpeed(1, 800, 10e5, 0);
    // T^2.13: doubling the temperature puts it up more than fourfold.
    expect(hot / cold).toBeGreaterThan(4);
    const low = laminarFlameSpeed(1, 650, 2e5, 0);
    const high = laminarFlameSpeed(1, 650, 20e5, 0);
    expect(high).toBeLessThan(low);
    expect(high / low).toBeGreaterThan(0.5);
  });

  it('slows with residual gas, and is gone outside the flammability limits', () => {
    const clean = laminarFlameSpeed(1, 650, 13e5, 0);
    expect(laminarFlameSpeed(1, 650, 13e5, 0.2)).toBeLessThan(0.5 * clean);
    expect(laminarFlameSpeed(0.4, 650, 13e5, 0)).toBe(0);
    expect(laminarFlameSpeed(2.1, 650, 13e5, 0)).toBe(0);
    expect(laminarFlameSpeed(0, 650, 13e5, 0)).toBe(0);
  });
});

describe('burn duration', () => {
  const spec = DEFAULT_ENGINE;

  it('is the stated duration at the reference flame state', () => {
    expect(burnAngle(spec, omegaAt(spec, 10), 13e5, 650, 1, 0.04)).toBeCloseTo(spec.burnDuration, 9);
  });

  it('lengthens with rpm, but much less than in proportion', () => {
    const at = (rpm: number) => burnAngle(spec, (rpm * 2 * Math.PI) / 60, 13e5, 650, 1, 0.04);
    const ratio = at(6000) / at(1000);
    // A burn taking a fixed time would be six times as many degrees.
    expect(ratio).toBeGreaterThan(1.3);
    expect(ratio).toBeLessThan(2.2);
  });

  it('comes out longer at part throttle, lean, and with residual, in the running engine', () => {
    const wot = watch(single({ throttle: 1, rpm: 3200 }), 1);
    const part = watch(single({ throttle: 0.2, rpm: 3200 }), 1);
    const lean = watch(single({ throttle: 1, rpm: 3200, lambda: 1.3 }), 1);
    const slow = watch(single({ throttle: 1, rpm: 1000 }), 2);
    const fast = watch(single({ throttle: 1, rpm: 6000 }), 1);

    // The default engine at full throttle is near its reference state.
    expect(wot.burn).toBeGreaterThan(0.85 * DEFAULT_ENGINE.burnDuration);
    expect(wot.burn).toBeLessThan(1.1 * DEFAULT_ENGINE.burnDuration);
    expect(part.burn).toBeGreaterThan(1.15 * wot.burn);
    expect(lean.burn).toBeGreaterThan(1.1 * wot.burn);
    expect(fast.burn).toBeGreaterThan(1.3 * slow.burn);
  });
});

describe('advance map', () => {
  /** Mean spark timing of cylinder 1 over a second, deg BTDC. */
  function advance(over: Partial<EngineSpec>): number {
    const sim = single(over);
    sim.render(FS / 2);
    const cyl = (sim as unknown as { cyls: { spark: number }[] }).cyls[0]!;
    let sum = 0;
    for (let i = 0; i < 20; i++) {
      sim.render(FS / 20);
      sum += 720 - cyl.spark;
    }
    return sum / 20;
  }

  it('retards where the burn is quick and advances where it is slow', () => {
    const nominal = 720 - DEFAULT_ENGINE.ignition;
    const low = advance({ throttle: 1, rpm: 1000 });
    const high = advance({ throttle: 1, rpm: 6000 });
    const part = advance({ throttle: 0.2, rpm: 3200 });
    expect(low).toBeLessThan(nominal - 3);
    expect(high).toBeGreaterThan(nominal + 2);
    expect(part).toBeGreaterThan(nominal + 2);
  });

  it('holds the spark where it is set with the map off', () => {
    const nominal = 720 - DEFAULT_ENGINE.ignition;
    expect(advance({ throttle: 1, rpm: 1000, advanceCurve: false })).toBeCloseTo(nominal, 9);
  });

  /**
   * A four bogged to its speed floor under load has to be able to pull away on full throttle. On fixed
   * timing its burn is so short at 450 rpm that most of the heat is released before top dead centre,
   * and it sits there making no power whatever the throttle does.
   */
  it('lets a bogged engine pull away', () => {
    const four = ENGINE_PRESETS.find((p) => p.name === 'Inline four')!;
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, ...four.engine, freeRunning: true, load: 0.46, throttle: 0.1 };
    cfg.pipe = four.pipe();
    cfg.collector = four.collector!();
    const sim = new EngineSim(FS, cfg);
    sim.render(FS * 2);
    expect(sim.rpm).toBeLessThan(600);
    sim.setControls(1, 0.46);
    sim.render(FS * 3);
    expect(sim.rpm).toBeGreaterThan(3000);
  });
});

describe('mixture', () => {
  it('lean releases less heat per charge; rich has no more oxygen to release it with', () => {
    const stoich = watch(single({ throttle: 1, rpm: 3200 }), 1);
    const lean = watch(single({ throttle: 1, rpm: 3200, lambda: 1.3 }), 1);
    const rich = watch(single({ throttle: 1, rpm: 3200, lambda: 0.8 }), 1);
    expect(lean.torque).toBeLessThan(0.85 * stoich.torque);
    // Oxygen-limited: within a few percent of stoichiometric, and not above it.
    expect(rich.torque).toBeLessThan(stoich.torque);
    expect(rich.torque).toBeGreaterThan(0.95 * stoich.torque);
  });

  it('misfires once the excess air dilutes the charge past the limit', () => {
    expect(watch(single({ throttle: 1, rpm: 3200, lambda: 1.4 }), 1).fired).toBe(1);
    expect(watch(single({ throttle: 1, rpm: 3200, lambda: 2 }), 2).fired).toBeLessThan(0.9);
  });
});

describe('overrun fuel cut', () => {
  it('leaves nothing to burn with the throttle shut above the cut speed', () => {
    const sim = single({ throttle: 0, rpm: 3200 });
    const cut = watch(sim, 1);
    expect(cut.fired).toBe(0);
    expect(sim.snapshot().fuelCut).toBe(true);
    // The injectors are off, so neither the manifold nor the runners hold any fuel worth the name:
    // under 1% of a stoichiometric charge's.
    const inner = sim as unknown as { plenum: { fuelFraction: number }; intake: { fuel: Float64Array } };
    expect(inner.plenum.fuelFraction).toBeLessThan(6e-4);
    expect(inner.intake.fuel[0]!).toBeLessThan(6e-4);
  });

  it('keeps firing weakly on the throttle leak when it is off', () => {
    const run = watch(single({ throttle: 0, rpm: 3200, fuelCut: false }), 1);
    expect(run.fired).toBeGreaterThan(0.1);
  });

  it('is not active below the resume speed or with the throttle open', () => {
    expect(watch(single({ throttle: 0, rpm: 1000 }), 1).fired).toBeGreaterThan(0);
    const open = single({ throttle: 0.05, rpm: 3200 });
    watch(open, 0.2);
    expect(open.snapshot().fuelCut).toBe(false);
  });

  it('brings the fuel back as soon as the throttle opens', () => {
    const sim = single({ throttle: 0, rpm: 3200 });
    watch(sim, 0.5);
    sim.setControls(0.5, 0);
    expect(watch(sim, 1).fired).toBe(1);
  });
});

describe('valves per cylinder', () => {
  /** Mean gas torque, N*m, of the boxer four at full throttle and `rpm`. */
  function boxerTorque(rpm: number, over: Partial<EngineSpec> = {}): number {
    const boxer = ENGINE_PRESETS.find((p) => p.name === 'Boxer four')!;
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, ...boxer.engine, freeRunning: false, throttle: 1, rpm, combustionVariability: 0, ...over };
    cfg.pipe = boxer.pipe();
    cfg.collector = boxer.collector!();
    const sim = new EngineSim(FS, cfg);
    sim.render(FS / 2);
    const inner = sim as unknown as { torqueLast: number };
    let t = 0;
    for (let i = 0; i < FS / 2; i++) {
      sim.render(1);
      t += inner.torqueLast;
    }
    return t / (FS / 2);
  }

  /**
   * A four-valve engine keeps most of its torque to near its rev limit, falling off past the speed its
   * intake runners are tuned for. The same valves, one of each, choke it: the cylinder cannot empty
   * through them, and torque halves long before the limit.
   */
  it('lets a four-valve head breathe at high rpm, where one valve of each chokes', () => {
    const four = boxerTorque(6200) / boxerTorque(3600);
    const two = boxerTorque(6200, { exValveCount: 1, inValveCount: 1 }) / boxerTorque(3600, { exValveCount: 1, inValveCount: 1 });
    expect(four).toBeGreaterThan(0.75);
    expect(two).toBeLessThan(four - 0.2);
  });
});
