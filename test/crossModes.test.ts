/**
 * The cross-wise modes of a chamber: the section eigenproblem against shapes with known answers, and
 * the coupled duct against what a wide, offset-pipe can should do to the sound.
 */

import { describe, expect, it } from 'vitest';
import { GAS, makeSegment, type Section } from '../src/model/spec.js';
import { modeCutoffK, sectionModes } from '../src/audio/worklet/crossModes.js';
import { EulerPipe } from '../src/audio/worklet/eulerPipe.js';

const FS = 48000;

describe('section modes', () => {
  it('finds the Bessel roots of a circle', () => {
    const R = 0.1;
    const s: Section = { section: 'round', width: 2 * R, height: 2 * R };
    const modes = sectionModes(s, 6 / R, [{ offset: 0.05, diameter: 0.02 }]);
    // Only modes even about the width axis: cos(m theta) for m = 1, 2, then the first radial mode.
    const want = [1.8412, 3.0542, 3.8317];
    const got = modes.map((m) => m.k * R);
    for (let i = 0; i < want.length; i++) expect(Math.abs(got[i]! - want[i]!) / want[i]!).toBeLessThan(0.005);
  });

  it('finds pi/W across a rounded rectangle, give or take the little the corners change', () => {
    const s: Section = { section: 'rect', width: 0.3, height: 0.12 };
    const modes = sectionModes(s, 40, [{ offset: 0.1, diameter: 0.04 }]);
    const first = modes[0]!.k;
    expect(Math.abs(first - Math.PI / 0.3) / (Math.PI / 0.3)).toBeLessThan(0.03);
  });

  it('puts an ellipse between the circles of its two axes', () => {
    const s: Section = { section: 'oval', width: 0.3, height: 0.15 };
    const k = sectionModes(s, 40, [{ offset: 0.1, diameter: 0.04 }])[0]!.k;
    expect(k).toBeGreaterThan(1.8412 / 0.15);
    expect(k).toBeLessThan(1.8412 / 0.075);
  });

  it('leaves the antisymmetric modes to an offset pipe', () => {
    const s: Section = { section: 'oval', width: 0.3, height: 0.15 };
    const centred = sectionModes(s, 40, [{ offset: 0, diameter: 0.04 }])[0]!;
    const offset = sectionModes(s, 40, [{ offset: 0.1, diameter: 0.04 }])[0]!;
    expect(Math.abs(centred.atPipes[0]!)).toBeLessThan(1e-6);
    expect(Math.abs(offset.atPipes[0]!)).toBeGreaterThan(0.5);
  });
});

describe('a duct with a wide, offset-pipe can', () => {
  const can = (offset: number) => [
    makeSegment({ length: 0.4, dIn: 0.042 }),
    makeSegment({
      kind: 'chamber',
      length: 0.4,
      dIn: 0.042,
      dOut: 0.3,
      section: 'oval',
      height: 0.12,
      offsetIn: offset,
      offsetOut: -offset,
    }),
    makeSegment({ length: 0.4, dIn: 0.042 }),
  ];

  it('keeps no modes for a round can with centred pipes', () => {
    const p = new EulerPipe(
      [makeSegment({ kind: 'chamber', length: 0.34, dIn: 0.042, dOut: 0.13 })],
      FS,
      900,
      { cellSize: 0.035, singleStep: true },
    );
    expect(p.crossModes).toBeNull();
  });

  it('keeps modes for an offset oval, all of them inside the grid band', () => {
    const p = new EulerPipe(can(0.1), FS, 900, { cellSize: 0.035, singleStep: true });
    expect(p.crossModes).not.toBeNull();
    const cap = (modeCutoffK(p.dx) * Math.sqrt(GAS.gammaExh * GAS.R * 1400)) / (2 * Math.PI);
    for (const f of p.crossModes!.frequencies()) expect(f).toBeLessThan(cap);
  });

  /**
   * Ring the duct with one valve pulse and compare the mouth's spectrum with the pipes offset and
   * centred. Below the first cross-wise mode the two are the same muffler, since only the plane wave
   * exists there. From it upwards the offset pipes drive the modes, which put peaks and notches in the
   * transmission, so the spectra part company by many dB.
   */
  it('changes the sound from the first cross-wise mode up, and not below it', () => {
    const ring = (offset: number) => {
      const p = new EulerPipe(can(offset), FS, 900, { cellSize: 0.035, singleStep: true, heatTransfer: false });
      const out = new Float64Array(32768);
      for (let k = 0; k < out.length; k++) {
        const pulse = k < 30;
        out[k] = p.advance(1 / FS, {
          throatArea: pulse ? 3e-4 : 0,
          cylPressure: pulse ? 1.5e5 : GAS.pAmb,
          cylTemp: 900,
        }).mouthFlow;
      }
      return { out, f0: p.crossModes?.frequencies()[0] ?? 0 };
    };
    const offset = ring(0.1);
    const centred = ring(0);
    // c / 2W for a 300 mm can is 800-1000 Hz at these temperatures; the curved wall lifts it a little.
    expect(offset.f0).toBeGreaterThan(800);
    expect(offset.f0).toBeLessThan(1300);

    const meanDiffDb = (lo: number, hi: number) => {
      let sum = 0;
      let n = 0;
      for (let f = lo; f < hi; f += 25) {
        const a = bandEnergy(offset.out, f, f + 25);
        const b = bandEnergy(centred.out, f, f + 25);
        sum += Math.abs(10 * Math.log10(a / b));
        n++;
      }
      return sum / n;
    };
    const below = meanDiffDb(300, offset.f0 * 0.85);
    const above = meanDiffDb(offset.f0 * 0.9, offset.f0 * 1.35);
    // Measured at 1.2 dB below and 3.2 dB above. Below is not zero because a mode under its own
    // frequency still loads the pipe opening a little, as extra mass.
    expect(above).toBeGreaterThan(2.5);
    expect(above).toBeGreaterThan(2 * below);
  });
});

/** Energy of `x` in the DFT bins from `lo` up to `hi` Hz, on a Hann window. */
function bandEnergy(x: Float64Array, lo: number, hi: number): number {
  const N = x.length;
  let e = 0;
  const k0 = Math.max(1, Math.ceil((lo * N) / FS));
  const k1 = Math.max(k0, Math.ceil((hi * N) / FS) - 1);
  for (let k = k0; k <= k1; k++) {
    let re = 0;
    let im = 0;
    for (let n = 0; n < N; n++) {
      const w = 0.5 - 0.5 * Math.cos((2 * Math.PI * n) / (N - 1));
      const t = (2 * Math.PI * k * n) / N;
      re += w * x[n]! * Math.cos(t);
      im -= w * x[n]! * Math.sin(t);
    }
    e += re * re + im * im;
  }
  return e;
}
