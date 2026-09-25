/**
 * Spectral balance tests.
 *
 * These exist because of a real regression: the valve-throat turbulence was injected as
 * *white* noise, and the radiation derivative then tilted it +6 dB/octave into a rising
 * hiss that dominated everything above 2 kHz by more than 10 dB. The engine measured
 * fine on every other test — firing frequency, resonances, passivity all passed — it just
 * sounded wrong. So the timbre needs its own guards.
 */

import { describe, expect, it } from 'vitest';
import {
  PIPE_PRESETS,
  defaultConfig,
  makeSegment,
  type EngineSpec,
} from '../src/model/spec.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';
import { EulerPipe } from '../src/audio/worklet/eulerPipe.js';
import { hann, magnitudeSpectrum } from './spectrum.js';

const FS = 48000;
const N = 65536;
const BINHZ = FS / N;

function render(over: Partial<EngineSpec> = {}, preset = 1): Float32Array {
  const cfg = defaultConfig();
  cfg.engine = { ...cfg.engine, rpm: 3200, ...over };
  cfg.pipe = PIPE_PRESETS[preset]!.build();
  const sim = new EngineSim(FS, cfg);
  sim.render(FS);
  return sim.render(N);
}

/** Energy in the octave centred on `hz`. */
function octave(mag: Float64Array, hz: number): number {
  let e = 0;
  const lo = Math.max(1, Math.floor(hz / Math.SQRT2 / BINHZ));
  const hi = Math.min(mag.length - 1, Math.ceil((hz * Math.SQRT2) / BINHZ));
  for (let i = lo; i <= hi; i++) e += mag[i]! * mag[i]!;
  return e;
}

const OCTAVES = [31.5, 63, 125, 250, 500, 1000, 2000, 4000, 8000, 16000];

function bands(buf: Float32Array): Map<number, number> {
  const mag = magnitudeSpectrum(hann(buf), N);
  return new Map(OCTAVES.map((f) => [f, octave(mag, f)]));
}

const dB = (a: number, b: number) => 10 * Math.log10(a / b);

describe('the exhaust note is not dominated by broadband hiss', () => {
  it('peaks in the low octaves, where a single-cylinder engine belongs', () => {
    const b = bands(render());
    const loudest = OCTAVES.reduce((a, f) => (b.get(f)! > b.get(a)! ? f : a));
    // At 3200 rpm the firing frequency is 26.7 Hz, so the note's energy sits in the
    // 125-500 Hz octaves. A peak up at 2-4 kHz would mean noise had taken over.
    expect(loudest).toBeGreaterThanOrEqual(125);
    expect(loudest).toBeLessThanOrEqual(500);
  });

  it('rolls off steeply above 2 kHz rather than holding a flat hiss', () => {
    const b = bands(render());
    const peak = Math.max(...OCTAVES.map((f) => b.get(f)!));
    // Measures about -14 dB at 4 kHz and -36 at 8 kHz. The broken version sat at -9.4
    // at 4 kHz, so this still catches it.
    //
    // The threshold moved from -14 to -12 when the linear waveguide was replaced by the
    // Euler solver, and that is expected rather than a regression: nonlinear steepening
    // genuinely creates harmonics the linear model could not produce at any amplitude —
    // the exhaust-only path alone went from -20 to -16 dB at 4 kHz. What matters is that
    // the added content is *harmonic* (brassiness) rather than broadband hiss, and the
    // 'band-limited, not white' test below checks that mechanism directly.
    expect(dB(b.get(4000)!, peak)).toBeLessThan(-12);
    expect(dB(b.get(8000)!, peak)).toBeLessThan(-28);
  });

  it('falls monotonically from 1 kHz upward', () => {
    const b = bands(render());
    for (const [lo, hi] of [
      [1000, 2000],
      [2000, 4000],
      [4000, 8000],
      [8000, 16000],
    ] as const) {
      expect(b.get(hi)!, `${hi} Hz should be quieter than ${lo} Hz`).toBeLessThan(b.get(lo)!);
    }
  });

  it('throat turbulence stays a texture, never the loudest thing in the room', () => {
    const off = bands(render({ throatNoise: 0 }));
    const full = bands(render({ throatNoise: 1 }));
    // Turning it to maximum may colour the upper mids, but it must not transform the
    // spectrum. Measures +6.3 dB at 4 kHz and +2.6 at 8 kHz, against more than 10 dB at
    // 4 kHz at only *half* power before the white-noise fix.
    //
    // The threshold moved from 6 to 8 once the 3 kHz heat-batching artefact was removed:
    // that artefact inflated the no-noise baseline, so the same absolute turbulence used to
    // look like a smaller relative step than it really was.
    expect(dB(full.get(4000)!, off.get(4000)!)).toBeLessThan(8);
    expect(dB(full.get(8000)!, off.get(8000)!)).toBeLessThan(8);
    // And it must not shift the low end at all.
    expect(Math.abs(dB(full.get(250)!, off.get(250)!))).toBeLessThan(1.5);
  });

  it('turbulence is band-limited, not white', () => {
    // Isolate the noise by differencing the spectra with it off and at full power. If
    // the injected noise were white, the radiation derivative would make this
    // difference *grow* with frequency all the way to Nyquist.
    const off = bands(render({ throatNoise: 0, mechNoise: 0 }));
    const on = bands(render({ throatNoise: 1, mechNoise: 0 }));
    const added = (f: number) => Math.max(on.get(f)! - off.get(f)!, 1e-30);
    // At or equal to, because above 8 kHz the noise adds nothing measurable: the two runs differ
    // there by a few percent either way, so both octaves can clamp to the floor together.
    expect(added(8000)).toBeLessThan(added(2000));
    expect(added(16000)).toBeLessThanOrEqual(added(8000));
  });

  it('holds across every preset and speed', () => {
    for (let p = 0; p < PIPE_PRESETS.length; p++) {
      const name = PIPE_PRESETS[p]!.name;

      // No preset is excepted here any more.
      //
      // The expansion chamber used to need a far looser limit, blamed on quasi-1D theory
      // missing flow separation at its reverse cone. That diagnosis was wrong: the peak was
      // a numerical artefact from batching the wall heat transfer every 16 samples, which
      // injects a periodic energy perturbation at exactly 48000/16 = 3000 Hz. It showed up
      // in every preset and was loudest here only because this geometry has a high-Q
      // tailpipe mode near 3 kHz to amplify it.

      for (const rpm of [1500, 3200, 6500]) {
        const b = bands(render({ rpm }, p));
        const peak = Math.max(...OCTAVES.map((f) => b.get(f)!));
        expect(
          dB(b.get(16000)!, peak),
          `${name} at ${rpm} rpm has too much energy at 16 kHz`,
        ).toBeLessThan(-26);
      }
    }
  });
});

describe('the exhaust port is part of the acoustic system', () => {
  it('lengthening the port lowers the resonance, because tuning starts at the valve', () => {
    const first = (portLength: number) => {
      const wg = new EulerPipe([makeSegment({ length: 0.5, dIn: 0.042 })], FS, 293, {
        port: { length: portLength, diameter: 0.034 },
      });
      // Quarter-wave of the whole duct, port included.
      return 343 / (4 * wg.totalLength);
    };
    expect(first(0.15)).toBeLessThan(first(0.01));
  });

  it('reports how much of the duct is port, so the display can skip it', () => {
    const wg = new EulerPipe([makeSegment({ length: 0.5, dIn: 0.042 })], FS, 900, {
      port: { length: 0.055, diameter: 0.034 },
    });
    expect(wg.portCells).toBeGreaterThan(0);
    expect(wg.portCells).toBeLessThan(wg.n);

    const taps = new Float32Array(128);
    wg.samplePressure(taps);
    expect(taps.length).toBe(128);

    const bare = new EulerPipe([makeSegment({ length: 0.5, dIn: 0.042 })], FS, 900);
    expect(bare.portCells).toBe(0);
    // The port really does add duct.
    expect(wg.n).toBeGreaterThan(bare.n);
  });
});

describe('plane-wave validity limit', () => {
  it('a wide mouth cuts on sooner than a narrow one', () => {
    const cutoff = (dia: number) =>
      new EulerPipe([makeSegment({ length: 0.6, dIn: dia })], FS, 900).planeWaveCutoffRad;
    // omega = 1.84 c / a, so a bigger radius means a lower cut-on frequency.
    expect(cutoff(0.12)).toBeLessThan(cutoff(0.03));
  });

  it('sits well above the fundamentals it must not touch', () => {
    const wg = new EulerPipe([makeSegment({ length: 0.6, dIn: 0.1 })], FS, 900);
    const hz = wg.planeWaveCutoffRad / (2 * Math.PI);
    expect(hz).toBeGreaterThan(1500);
    expect(hz).toBeLessThan(12000);
  });
});

describe('no fixed-rate numerical artefacts', () => {
  /**
   * Energy in a narrow band around `hz`, relative to total, for a preset with no
   * stochastic sources at all. Any tone that survives this has to come from the solver.
   */
  function tonePeak(preset: number, hz: number): number {
    const spec = magnitudeSpectrum(
      hann(render({ rpm: 1500, mechNoise: 0, throatNoise: 0, combustionVariability: 0 }, preset)),
      N,
    );
    const at = (f: number) => {
      let e = 0;
      for (let i = Math.floor((f - 40) / BINHZ); i <= Math.ceil((f + 40) / BINHZ); i++) {
        e += spec[i]! ** 2;
      }
      return e;
    };
    // Compare the suspect band with its neighbours: a solver artefact is a narrow spike
    // sitting on top of whatever the engine is doing.
    return at(hz) / Math.max(0.5 * (at(hz * 0.72) + at(hz * 1.38)), 1e-30);
  }

  it('nothing rings at the heat-transfer batch rate', () => {
    // Batching the wall heat transfer every N samples injects a periodic energy
    // perturbation at sampleRate/N — at N = 16 that is exactly 3000 Hz. It appeared in every
    // preset and dominated the expansion chamber, which has a high-Q tailpipe mode there.
    //
    // Deliberately checked on that preset, since it is the one that amplifies it most, and
    // with every stochastic source disabled so nothing can mask it.
    for (const preset of [0, 2, 3]) {
      expect(
        tonePeak(preset, FS / 16),
        `preset ${preset} has a spike at the heat batch rate`,
      ).toBeLessThan(8);
    }
  });

  it('the expansion chamber is no louder in the treble than the others', () => {
    // It needed a 12 dB looser limit while the artefact was there.
    const share = (preset: number) => {
      const b = bands(
        render({ rpm: 1500, mechNoise: 0, throatNoise: 0, combustionVariability: 0 }, preset),
      );
      const total = OCTAVES.reduce((a, f) => a + b.get(f)!, 0);
      return (b.get(2000)! + b.get(4000)! + b.get(8000)! + b.get(16000)!) / total;
    };
    expect(share(2)).toBeLessThan(0.1);
    expect(share(2)).toBeLessThan(share(1) + 0.08);
  });
});
