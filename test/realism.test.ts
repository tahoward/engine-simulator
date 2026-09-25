/**
 * Tests for the things that separate "physically correct" from "sounds like an engine".
 *
 * The simulation passed every acoustic and thermodynamic test while still sounding
 * synthetic, because it was perfectly periodic: cycle-to-cycle correlation was 0.99,
 * peak-pressure scatter was 0.004% where real engines show 1-3%, the crank turned at a
 * mathematically constant rate, and the listener heard a single anechoic monopole with
 * no ground under it. Those are the properties guarded here.
 */

import { describe, expect, it } from 'vitest';
import {
  DEFAULT_ENGINE,
  GAS,
  PIPE_PRESETS,
  cylinderVolume,
  d2PistonDTheta2,
  dPistonDTheta,
  defaultConfig,
  makeSegment,
  type EngineSpec,
} from '../src/model/spec.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';
import { Cylinder } from '../src/audio/worklet/cylinder.js';
import { Listener } from '../src/audio/worklet/listener.js';
import { magnitudeSpectrum } from './spectrum.js';

const FS = 48000;

function sim(over: Partial<EngineSpec> = {}, preset = 1): EngineSim {
  const cfg = defaultConfig();
  cfg.engine = { ...cfg.engine, rpm: 3200, ...over };
  cfg.pipe = PIPE_PRESETS[preset]!.build();
  const s = new EngineSim(FS, cfg);
  s.render(FS * 2);
  return s;
}

const cycleSamples = (rpm: number) => Math.round((FS * 120) / rpm);

function cov(values: number[]): number {
  const m = values.reduce((a, b) => a + b, 0) / values.length;
  const v = values.reduce((a, b) => a + (b - m) ** 2, 0) / values.length;
  return Math.sqrt(v) / Math.abs(m);
}

/** Indicated work per cycle, J, by integrating (p - ambient) dV. */
function indicatedWork(s: EngineSim, n: number): number[] {
  const out: number[] = [];
  const len = cycleSamples(s.rpm);
  for (let c = 0; c < n; c++) {
    let w = 0;
    let prevV = cylinderVolume(s.engine, s.cylinder.angle);
    for (let i = 0; i < len; i++) {
      s.tick();
      const v = cylinderVolume(s.engine, s.cylinder.angle);
      w += (s.cylinder.pressure(s.engine) - GAS.pAmb) * (v - prevV);
      prevV = v;
    }
    out.push(w);
  }
  return out;
}

/**
 * RMS difference between consecutive cycles, relative to the RMS of a cycle.
 *
 * Correlation is the tempting metric here and it is a bad one: it normalises away
 * amplitude, so a cycle that is 30% louder than the last still correlates at 1.0.
 * Measured against the same signals, correlation moves only 0.991 -> 0.967 when
 * combustion scatter is switched on, while this metric moves 14% -> 29%.
 */
function cycleDifference(s: EngineSim, cycles = 8): number {
  const len = cycleSamples(s.rpm);
  let prev = s.render(len);
  const diffs: number[] = [];
  for (let k = 0; k < cycles; k++) {
    const cur = s.render(len);
    let d = 0;
    let a = 0;
    for (let i = 0; i < len; i++) {
      d += (cur[i]! - prev[i]!) ** 2;
      a += prev[i]! ** 2;
    }
    diffs.push(Math.sqrt(d / Math.max(a, 1e-30)));
    prev = cur;
  }
  return diffs.reduce((x, y) => x + y, 0) / diffs.length;
}

describe('no two cycles are alike', () => {
  it('successive cycles differ materially, and scatter is why', () => {
    const withScatter = cycleDifference(sim());
    const without = cycleDifference(sim({ combustionVariability: 0 }));
    // Measures about 36% with scatter, 30% without.
    //
    // The margin between them narrowed when the Euler solver replaced the waveguide
    // (it was 28% vs 18%): nonlinear propagation and the carried-over gas state make
    // consecutive cycles differ more on their own, so combustion scatter is a smaller
    // share of a larger total. It still has to contribute — with scatter contributing
    // nothing the ratio would be 1.0 and this would fail. The stronger claim about
    // scatter is the load dependence tested below.
    expect(withScatter).toBeGreaterThan(0.24);
    expect(withScatter).toBeGreaterThan(without * 1.1);
  });

  it('combustion scatter can be switched off for a deterministic engine', () => {
    const s = sim({ combustionVariability: 0, throatNoise: 0, mechNoise: 0 });
    const works = indicatedWork(s, 8);
    expect(cov(works)).toBeLessThan(0.002);
  });

  it('scatter grows as the charge thins, as real engines do', () => {
    // Real engines: 1-3% CoV of indicated work at full load, 5-15% near idle.
    const heavy = cov(indicatedWork(sim({ throttle: 1 }), 30));
    const light = cov(indicatedWork(sim({ throttle: 0.12 }), 30));
    expect(heavy).toBeGreaterThan(0.005);
    expect(heavy).toBeLessThan(0.04);
    expect(light).toBeGreaterThan(heavy * 1.8);
    expect(light).toBeLessThan(0.25);
  });
});

describe('the crank does not turn at a constant rate', () => {
  it('ripples within the cycle even at a commanded fixed speed', () => {
    const s = sim();
    let lo = Infinity;
    let hi = -Infinity;
    for (let i = 0; i < cycleSamples(3200) * 4; i++) {
      s.tick();
      lo = Math.min(lo, s.rpmInstant);
      hi = Math.max(hi, s.rpmInstant);
    }
    const irregularity = (hi - lo) / ((hi + lo) / 2);
    // A big single with a modest flywheel swings a few percent.
    expect(irregularity).toBeGreaterThan(0.004);
    expect(irregularity).toBeLessThan(0.15);
  });

  it('still holds the commanded mean speed', () => {
    // The ripple integrator must carry no DC, or the rpm slider stops meaning
    // anything and the error becomes load-dependent.
    for (const throttle of [0.2, 0.6, 1]) {
      const s = sim({ throttle });
      let sum = 0;
      const n = cycleSamples(3200) * 12;
      for (let i = 0; i < n; i++) {
        s.tick();
        sum += s.rpmInstant;
      }
      const mean = sum / n;
      expect(Math.abs(mean - 3200) / 3200, `throttle ${throttle}`).toBeLessThan(0.01);
      // The steady readout should report the commanded mean, not the ripple.
      expect(s.rpm).toBeCloseTo(3200, 0);
    }
  });

  it('a heavier flywheel smooths the ripple', () => {
    const swing = (flywheelInertia: number) => {
      const s = sim({ flywheelInertia });
      let lo = Infinity;
      let hi = -Infinity;
      for (let i = 0; i < cycleSamples(3200) * 4; i++) {
        s.tick();
        lo = Math.min(lo, s.rpmInstant);
        hi = Math.max(hi, s.rpmInstant);
      }
      return hi - lo;
    };
    expect(swing(1.0)).toBeLessThan(swing(0.1));
  });
});

describe('reciprocating inertia', () => {
  it('matches a numerical derivative of the piston motion', () => {
    const spec = DEFAULT_ENGINE;
    const h = 1e-3;
    for (const deg of [15, 60, 110, 190, 265, 340]) {
      const numericFirst =
        ((cylinderVolume(spec, deg + h) - cylinderVolume(spec, deg - h)) / (2 * h)) *
        (180 / Math.PI);
      // dV/dtheta = -A * dx/dtheta, so this cross-checks dPistonDTheta.
      const area = (Math.PI * spec.bore * spec.bore) / 4;
      expect(dPistonDTheta(spec, deg)).toBeCloseTo(-numericFirst / area, 6);

      const numericSecond =
        ((dPistonDTheta(spec, deg + h) - dPistonDTheta(spec, deg - h)) / (2 * h)) *
        (180 / Math.PI);
      expect(d2PistonDTheta2(spec, deg)).toBeCloseTo(numericSecond, 5);
    }
  });

  it('does no net work over a cycle, so it cannot change the mean speed', () => {
    // It only stores and returns energy. If it integrated to something non-zero it
    // would act as a phantom torque and the rpm would depend on the piston mass.
    const spec: EngineSpec = { ...DEFAULT_ENGINE };
    const cyl = new Cylinder(spec, 0);
    const omega = 335;
    let integral = 0;
    const step = 0.05;
    for (let deg = 0; deg < 720; deg += step) {
      cyl.angle = deg;
      cyl.advance(spec, 1e-9, omega, 0, 0, GAS.tAmb, 900);
      integral += cyl.inertiaTorque * ((step * Math.PI) / 180);
    }
    // Normalise against the scale of the torque itself.
    let scale = 0;
    for (let deg = 0; deg < 720; deg += 10) {
      cyl.angle = deg;
      cyl.advance(spec, 1e-9, omega, 0, 0, GAS.tAmb, 900);
      scale = Math.max(scale, Math.abs(cyl.inertiaTorque));
    }
    expect(Math.abs(integral) / (scale * 2 * Math.PI)).toBeLessThan(0.01);
  });
});

describe('the listener is outdoors, not in a vacuum', () => {
  it('the ground reflection combs the spectrum where geometry says it should', () => {
    const listener = new Listener(FS);
    const geom = {
      distance: 1.5,
      micHeight: 1.2,
      sourceHeight: 0.35,
      reflection: 1,
    };
    listener.setGeometry(geom);

    // Impulse in, so the response is the two-path filter itself.
    const n = 8192;
    const ir = new Float32Array(n);
    for (let i = 0; i < n; i++) ir[i] = listener.process(i === 0 ? 1 : 0);
    const mag = magnitudeSpectrum(ir, n);

    const c = Math.sqrt(GAS.gammaExh * GAS.R * GAS.tAmb);
    const rDirect = Math.hypot(geom.distance, geom.micHeight - geom.sourceHeight);
    const rGround = Math.hypot(geom.distance, geom.micHeight + geom.sourceHeight);
    const nullHz = c / (2 * (rGround - rDirect));

    const binHz = FS / n;
    const at = (hz: number) => mag[Math.round(hz / binHz)]!;
    // Destructive at the predicted null, constructive at twice it.
    expect(at(nullHz)).toBeLessThan(at(nullHz * 2) * 0.6);
  });

  it('moving the listener changes the colouration', () => {
    const response = (micHeight: number) => {
      const l = new Listener(FS);
      l.setGeometry({ distance: 1.5, micHeight, sourceHeight: 0.35, reflection: 0.8 });
      const n = 4096;
      const ir = new Float32Array(n);
      for (let i = 0; i < n; i++) ir[i] = l.process(i === 0 ? 1 : 0);
      return magnitudeSpectrum(ir, n);
    };
    const a = response(0.4);
    const b = response(1.8);
    let diff = 0;
    for (let i = 1; i < a.length; i++) diff += Math.abs(a[i]! - b[i]!);
    expect(diff / a.length).toBeGreaterThan(0.01);
  });

  it('a hard surface reflects more than a soft one', () => {
    const energy = (reflection: number) => {
      const l = new Listener(FS);
      l.setGeometry({ distance: 2, micHeight: 1.2, sourceHeight: 0.35, reflection });
      let e = 0;
      for (let i = 0; i < 4096; i++) {
        const y = l.process(i === 0 ? 1 : 0);
        e += y * y;
      }
      return e;
    };
    expect(energy(0.9)).toBeGreaterThan(energy(0.1));
  });

  it('distance still attenuates, and dulls as well as quietens', () => {
    const measure = (distance: number) => {
      const l = new Listener(FS);
      l.setGeometry({ distance, micHeight: 1.2, sourceHeight: 0.35, reflection: 0.7 });
      const n = 4096;
      const ir = new Float32Array(n);
      for (let i = 0; i < n; i++) ir[i] = l.process(i === 0 ? 1 : 0);
      const mag = magnitudeSpectrum(ir, n);
      const binHz = FS / n;
      const band = (lo: number, hi: number) => {
        let e = 0;
        for (let i = Math.floor(lo / binHz); i < hi / binHz; i++) e += mag[i]! * mag[i]!;
        return e;
      };
      return { low: band(100, 400), high: band(6000, 12000) };
    };
    const near = measure(1.5);
    const far = measure(12);
    expect(far.low).toBeLessThan(near.low);
    // Air absorption: the far signal loses proportionally more treble than bass.
    expect(far.high / far.low).toBeLessThan(near.high / near.low);
  });
});

describe('structural noise is driven by combustion, not by a schedule', () => {
  it('a faster burn produces a steeper pressure rise', () => {
    // The mechanism, checked directly rather than through the mix.
    const steepness = (burnDuration: number) => {
      const s = sim({ burnDuration, combustionVariability: 0 });
      let pk = 0;
      for (let i = 0; i < cycleSamples(3200) * 3; i++) {
        s.tick();
        pk = Math.max(pk, Math.abs(s.cylinder.dpdt));
      }
      return pk;
    };
    // Roughly 14 GPa/s at 20 deg falling to 1.6 at 90 deg.
    expect(steepness(20)).toBeGreaterThan(steepness(45) * 1.4);
    expect(steepness(45)).toBeGreaterThan(steepness(90) * 1.4);
  });

  it('and therefore rings the casing harder', () => {
    // Isolated by differencing mechNoise on against off, so the gas path — which is
    // also affected by burn duration, and far louder — cancels out. Measuring the
    // total instead just measures the exhaust.
    const structure = (burnDuration: number) => {
      const base = { burnDuration, combustionVariability: 0, throatNoise: 0 };
      const on = sim({ ...base, mechNoise: 1 }, 3).render(FS / 2);
      const off = sim({ ...base, mechNoise: 0 }, 3).render(FS / 2);
      let d = 0;
      for (let i = 0; i < on.length; i++) d += (on[i]! - off[i]!) ** 2;
      return Math.sqrt(d / on.length);
    };
    expect(structure(20)).toBeGreaterThan(structure(90) * 2);
  });

  it('mechanical noise sits well below an open exhaust', () => {
    const rms = (buf: Float32Array) => {
      let s = 0;
      for (const v of buf) s += v * v;
      return Math.sqrt(s / buf.length);
    };
    const open = rms(sim({ mechNoise: 0, throatNoise: 0 }, 0).render(FS));
    const withMech = rms(sim({ mechNoise: 1, throatNoise: 0 }, 0).render(FS));
    // Turning it to full must barely move an open pipe's level.
    expect(20 * Math.log10(withMech / open)).toBeLessThan(2);
  });

  it('stays finite with degenerate listener geometry', () => {
    const s = sim({
      micDistance: 0.05,
      micHeight: 0,
      exhaustHeight: 0,
      groundReflection: 1,
      recipMass: 5,
      flywheelInertia: 0.02,
    });
    const buf = s.render(FS);
    for (const v of buf) {
      expect(Number.isFinite(v)).toBe(true);
      expect(Math.abs(v)).toBeLessThanOrEqual(1);
    }
  });
});

describe('nothing regressed in the basics', () => {
  it('still produces a stable, audible signal across presets', () => {
    for (let p = 0; p < PIPE_PRESETS.length; p++) {
      const buf = sim({}, p).render(FS / 2);
      let peak = 0;
      for (const v of buf) {
        expect(Number.isFinite(v)).toBe(true);
        peak = Math.max(peak, Math.abs(v));
      }
      expect(peak, `preset ${p} silent`).toBeGreaterThan(1e-3);
      expect(peak, `preset ${p} pinned`).toBeLessThan(1);
    }
  });

  it('an empty pipe with a port still works', () => {
    const cfg = defaultConfig();
    cfg.pipe = [makeSegment({ length: 0.05, dIn: 0.04 })];
    const s = new EngineSim(FS, cfg);
    const buf = s.render(FS);
    for (const v of buf) expect(Number.isFinite(v)).toBe(true);
  });
});
