/**
 * End-to-end tests on the assembled simulation. These check the claims a listener
 * would actually notice: that the note tracks rpm, that it is a four-stroke, that
 * the exhaust the user builds changes what they hear, and that nothing blows up.
 */

import { describe, expect, it } from 'vitest';
import {
  PIPE_PRESETS,
  defaultConfig,
  makeSegment,
  type EngineConfig,
  type EngineSpec,
} from '../src/model/spec.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';
import { bandEnergy, findPeaks, hann, magnitudeSpectrum } from './spectrum.js';

const FS = 48000;
const FFT_SIZE = 65536;

function run(overrides: Partial<EngineSpec> = {}, pipeIndex = 1): EngineSim {
  const cfg: EngineConfig = defaultConfig();
  cfg.engine = { ...cfg.engine, ...overrides };
  cfg.pipe = PIPE_PRESETS[pipeIndex]!.build();
  const sim = new EngineSim(FS, cfg);
  sim.render(FS); // let startup transients wash out of the pipe
  return sim;
}

function spectrumOf(sim: EngineSim): Float64Array {
  return magnitudeSpectrum(hann(sim.render(FFT_SIZE)), FFT_SIZE);
}

function rms(buf: Float32Array): number {
  let s = 0;
  for (const v of buf) s += v * v;
  return Math.sqrt(s / buf.length);
}

describe('firing frequency', () => {
  // A four-stroke single fires once per two revolutions, so f = rpm / 120.
  for (const rpm of [1800, 3200, 4800]) {
    it(`at ${rpm} rpm the harmonic comb sits on multiples of ${(rpm / 120).toFixed(2)} Hz`, () => {
      const f0 = rpm / 120;
      const mag = spectrumOf(run({ rpm }));

      // Compare energy *at* each harmonic with energy *between* harmonics, rather
      // than checking peak positions. Cycle-to-cycle combustion scatter and the
      // within-cycle crank speed ripple put real amplitude and frequency sidebands
      // around every harmonic, so picked peaks wander by a fraction of a percent —
      // correctly. The comb itself is the claim worth testing, and it is unaffected.
      for (let order = 1; order <= 8; order++) {
        const on = bandEnergy(mag, FS, FFT_SIZE, f0 * order, f0 * 0.3);
        const between = bandEnergy(mag, FS, FFT_SIZE, f0 * (order + 0.5), f0 * 0.3);
        expect(
          on,
          `order ${order} (${(f0 * order).toFixed(1)} Hz) is not above the gap beside it`,
        ).toBeGreaterThan(between);
      }

      // The fundamental itself must be present, not merely implied by harmonics.
      const atF0 = bandEnergy(mag, FS, FFT_SIZE, f0, 3);
      const offF0 = bandEnergy(mag, FS, FFT_SIZE, f0 * 1.5, 3);
      expect(atF0).toBeGreaterThan(offF0 * 4);

      // And it must be in the right *place* — within a hair of rpm/120 in absolute
      // terms, which peak-picking still resolves well.
      const near = findPeaks(mag, FS, FFT_SIZE, f0 * 0.5, f0 * 1.5, 0.2)[0]!;
      expect(Math.abs(near.hz - f0)).toBeLessThan(Math.max(1.5, f0 * 0.03));
    });
  }

  it('has no half-order component, confirming a four-stroke cycle', () => {
    const rpm = 3200;
    const mag = spectrumOf(run({ rpm }));
    const firing = bandEnergy(mag, FS, FFT_SIZE, rpm / 120, 3);
    // rpm/60 would be a two-stroke firing every revolution; rpm/240 would be a
    // spurious sub-harmonic from the cycle bookkeeping drifting.
    const half = bandEnergy(mag, FS, FFT_SIZE, rpm / 240, 3);
    expect(firing).toBeGreaterThan(half * 20);
  });

  it('doubling rpm doubles the firing frequency', () => {
    // Locate the fundamental specifically, by searching a window centred on where it
    // is predicted. Taking "the strongest peak in a wide range" instead is fragile:
    // which harmonic happens to win depends on where the pipe resonance falls, so it
    // can silently compare order 1 at one speed against order 2 at the other.
    const fundamental = (rpm: number) => {
      const f0 = rpm / 120;
      const mag = spectrumOf(run({ rpm }));
      return findPeaks(mag, FS, FFT_SIZE, f0 * 0.6, f0 * 1.4, 0.2)[0]!.hz;
    };
    const a = fundamental(2000);
    const b = fundamental(4000);
    expect(b / a).toBeGreaterThan(1.9);
    expect(b / a).toBeLessThan(2.1);
  });
});

describe('the exhaust the user builds changes the sound', () => {
  it('a muffler is quieter than an open header, by about what theory allows', () => {
    const open = rms(run({}, 0).render(FFT_SIZE));
    const muffled = rms(run({}, 3).render(FFT_SIZE));
    const dB = 20 * Math.log10(open / muffled);

    // Around 5 dB, and that is the right order for a *single* expansion chamber. Its
    // transmission loss is 10*log10(1 + 0.25*(m - 1/m)^2 * sin^2(kL)); with an area
    // ratio of 9.6 the peak is about 13 dB near 330 Hz, but it falls to exactly zero at
    // c/2L (~660 Hz) and every multiple, so broadband attenuation is far lower than the
    // peak. Real single-chamber boxes behave the same way, which is why silencers use
    // several chambers of different lengths.
    //
    // The linear waveguide reported 8.2 dB here; the Euler solver's ~5 dB is the more
    // honest figure.
    expect(dB).toBeGreaterThan(3.5);
    expect(dB).toBeLessThan(14);
  });

  // A sharper test — that attenuation peaks in the chamber's tuned band and falls to
  // nothing at c/2L — is deliberately not asserted here. Placing those bands requires
  // the chamber's *body* length and the local gas temperature, and picking them from an
  // approximation would mean tuning the bands until the test passed rather than
  // measuring anything.

  it('every preset produces a distinct spectrum', () => {
    const specs = PIPE_PRESETS.map((_, i) => spectrumOf(run({}, i)));
    // Compare normalised spectra so this measures tonal difference, not just level.
    const norm = specs.map((s) => {
      let e = 0;
      for (const v of s) e += v * v;
      const k = 1 / Math.sqrt(e);
      return s.map((v) => v * k);
    });
    for (let i = 0; i < norm.length; i++) {
      for (let j = i + 1; j < norm.length; j++) {
        let dot = 0;
        for (let k = 0; k < norm[i]!.length; k++) dot += norm[i]![k]! * norm[j]![k]!;
        // Cosine similarity of 1.0 would mean the geometry had no effect at all.
        expect(dot, `presets ${i} and ${j} are spectrally identical`).toBeLessThan(0.995);
      }
    }
  });

  it('a longer primary pipe lowers the exhaust resonance', () => {
    const resonanceOf = (length: number) => {
      const cfg = defaultConfig();
      cfg.engine.rpm = 3200;
      // Deliberately quiet mechanical sources so the measurement sees only the pipe.
      cfg.engine.mechNoise = 0;
      cfg.engine.throatNoise = 0;
      cfg.pipe = [makeSegment({ kind: 'pipe', length, dIn: 0.042 })];
      const sim = new EngineSim(FS, cfg);
      sim.render(FS);
      const mag = magnitudeSpectrum(hann(sim.render(FFT_SIZE)), FFT_SIZE);

      // Strongest radiated component in the band the pipe resonance lives in.
      //
      // This used to be an energy-weighted centroid over 40-1200 Hz, on the reasoning that
      // it was more robust than peak-picking against a harmonic comb. It is not, and the
      // finite intake plenum exposed that: once the low orders carry realistic weight, the
      // centroid is dominated by the firing comb rather than by the resonance, and it went
      // non-monotonic with length (187, 167, 187, 362 Hz at 0.35, 0.7, 1.4, 2.8 m) while the
      // duct's actual quarter-wave frequency tracked c/4L to within 1% (342, 182, 90, 42).
      const binHz = FS / FFT_SIZE;
      let best = 0;
      let bestHz = 0;
      for (let i = Math.floor(40 / binHz); i < Math.floor(1200 / binHz); i++) {
        if (mag[i]! > best) {
          best = mag[i]!;
          bestHz = i * binHz;
        }
      }
      return { peakHz: bestHz, quarterWaveHz: sim.ductQuarterWaveHz() };
    };

    const shortPipe = resonanceOf(0.35);
    const longPipe = resonanceOf(1.4);
    // The duct's own resonance, integrated over the solved temperature field. Quadrupling
    // the length must quarter it, give or take the cell quantisation and the fact that the
    // longer duct runs cooler at its far end.
    expect(longPipe.quarterWaveHz).toBeLessThan(shortPipe.quarterWaveHz * 0.35);
    // And it must reach the output, not just the solver.
    expect(longPipe.peakHz).toBeLessThan(shortPipe.peakHz);
  });

  it('editing the pipe mid-run stays finite and recovers level', () => {
    const sim = run({});
    sim.render(2000);
    sim.setPipe(PIPE_PRESETS[2]!.build());
    const buf = sim.render(FS);
    for (const v of buf) expect(Number.isFinite(v)).toBe(true);
    // The rebuild ramp is ~8 ms, so by the end of a second it must be audible again.
    expect(rms(buf.subarray(FS / 2))).toBeGreaterThan(1e-4);
  });
});

describe('output conditioning', () => {
  it('is audible but never clips, across rpm and every preset', () => {
    for (let p = 0; p < PIPE_PRESETS.length; p++) {
      for (const rpm of [900, 3200, 8000]) {
        const buf = run({ rpm }, p).render(FS / 2);
        let peak = 0;
        for (const v of buf) {
          expect(Number.isFinite(v)).toBe(true);
          peak = Math.max(peak, Math.abs(v));
        }
        expect(peak, `preset ${p} at ${rpm} rpm was silent`).toBeGreaterThan(1e-3);
        expect(peak, `preset ${p} at ${rpm} rpm pinned the output`).toBeLessThan(1.0);
      }
    }
  });

  it('survives extreme and degenerate configurations', () => {
    const nasty: Array<[string, Partial<EngineSpec>, ReturnType<typeof makeSegment>[]]> = [
      ['no pipe at all', {}, []],
      ['a 10 mm stub', {}, [makeSegment({ length: 0.01, dIn: 0.01 })]],
      ['huge chamber', {}, [makeSegment({ kind: 'chamber', length: 2, dIn: 0.02, dOut: 0.4 })]],
      ['closed throttle', { throttle: 0 }, PIPE_PRESETS[0]!.build()],
      ['zero lift', { maxLift: 0 }, PIPE_PRESETS[0]!.build()],
      ['valves never close', { evo: 0, evc: 719, ivo: 0, ivc: 719 }, PIPE_PRESETS[0]!.build()],
      ['12000 rpm', { rpm: 12000 }, PIPE_PRESETS[0]!.build()],
      ['cold exhaust', { portGasTemp: 300 }, PIPE_PRESETS[1]!.build()],
    ];

    for (const [name, spec, pipe] of nasty) {
      const cfg = defaultConfig();
      cfg.engine = { ...cfg.engine, ...spec };
      cfg.pipe = pipe;
      const sim = new EngineSim(FS, cfg);
      const buf = sim.render(FS);
      for (const v of buf) {
        expect(Number.isFinite(v), `${name} produced a non-finite sample`).toBe(true);
        expect(Math.abs(v), `${name} exceeded full scale`).toBeLessThanOrEqual(1);
      }
    }
  });

  it('produces a usable snapshot', () => {
    const sim = run({});
    sim.render(512);
    const s = sim.snapshot();
    expect(s.crankAngle).toBeGreaterThanOrEqual(0);
    expect(s.crankAngle).toBeLessThan(720);
    expect(s.rpm).toBeCloseTo(3200, 0);
    expect(s.cylPressure).toBeGreaterThan(1e4);
    expect(s.cylTemp).toBeGreaterThan(300);
    expect(s.pipePressure.length).toBe(128);
    for (const v of s.pipePressure) expect(Number.isFinite(v)).toBe(true);
    expect(s.pipeCells).toBeGreaterThan(8);
    // One step per audio sample, for every engine.
    expect(s.substeps).toBe(1);
  });
});

describe('free-running crank dynamics', () => {
  function settle(overrides: Partial<EngineSpec>): number {
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, freeRunning: true, ...overrides };
    const sim = new EngineSim(FS, cfg);
    sim.render(FS * 4); // several seconds to find equilibrium
    // Average over a further second to smooth out cyclic irregularity.
    let sum = 0;
    const n = 40;
    for (let i = 0; i < n; i++) {
      sim.render(FS / n);
      sum += sim.rpm;
    }
    return sum / n;
  }

  it('settles to a steady speed instead of running away or stalling', () => {
    const rpm = settle({ throttle: 0.7, load: 0.46 });
    expect(rpm).toBeGreaterThan(600);
    expect(rpm).toBeLessThan(11000);
  });

  it('more load slows it down', () => {
    const light = settle({ throttle: 0.8, load: 0.28 });
    const heavy = settle({ throttle: 0.8, load: 0.78 });
    expect(heavy).toBeLessThan(light * 0.95);
  });

  it('more throttle speeds it up', () => {
    const low = settle({ throttle: 0.3, load: 0.37 });
    const high = settle({ throttle: 1.0, load: 0.37 });
    expect(high).toBeGreaterThan(low * 1.05);
  });
});
