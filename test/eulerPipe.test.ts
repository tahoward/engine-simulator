/**
 * Tests for the quasi-1D Euler solver that replaced the linear waveguide.
 *
 * Two families here, and both matter. The first is standard CFD verification: the Sod
 * shock tube against its exact solution, TVD behaviour (no overshoot), and conservation.
 * The second is the acoustics the old waveguide guaranteed *by construction* and this
 * solver has to earn numerically — resonating at c/4L, reflecting off area changes, and
 * staying stable. Swapping an exact linear method for an approximate nonlinear one is
 * only worth it if the linear behaviour survives.
 */

import { describe, expect, it } from 'vitest';
import { GAS, makeSegment, speedOfSound } from '../src/model/spec.js';
import {
  EulerPipe,
  limitAreaRatio,
  type EulerPipeOptions,
  type SlopeLimiter,
} from '../src/audio/worklet/eulerPipe.js';
import { findPeaks, magnitudeSpectrum } from './spectrum.js';

const FS = 48000;
const FFT = 32768;

/** A closed-both-ends uniform duct, for pure gas-dynamics tests. */
function tube(cells: number, length: number, limiter: SlopeLimiter = 'mc'): EulerPipe {
  return new EulerPipe([makeSegment({ length, dIn: 0.05 })], FS, GAS.tAmb, {
    cellSize: length / cells,
    maxCells: 4096,
    radiate: false,
    heatTransfer: false,
    linearDamping: 0,
    darcyFriction: 0,
    limiter,
  });
}

const SHUT = { throatArea: 0, cylPressure: GAS.pAmb, cylTemp: GAS.tAmb };

describe('shock capturing', () => {
  /**
   * Sod's problem, the standard verification case. Exact solution computed here by
   * Newton iteration on the star pressure.
   */
  function sodExact(x: number, t: number, g: number) {
    const rL = 1;
    const pL = 1;
    const rR = 0.125;
    const pR = 0.1;
    const cL = Math.sqrt((g * pL) / rL);
    const cR = Math.sqrt((g * pR) / rR);
    const f = (p: number, rK: number, pK: number, cK: number) => {
      if (p > pK) {
        const A = 2 / ((g + 1) * rK);
        const B = (pK * (g - 1)) / (g + 1);
        return (p - pK) * Math.sqrt(A / (p + B));
      }
      return ((2 * cK) / (g - 1)) * (Math.pow(p / pK, (g - 1) / (2 * g)) - 1);
    };
    const fd = (p: number, rK: number, pK: number, cK: number) => {
      if (p > pK) {
        const A = 2 / ((g + 1) * rK);
        const B = (pK * (g - 1)) / (g + 1);
        const s = Math.sqrt(A / (p + B));
        return s * (1 - (p - pK) / (2 * (p + B)));
      }
      return (1 / (rK * cK)) * Math.pow(p / pK, -(g + 1) / (2 * g));
    };
    let p = 0.5 * (pL + pR);
    for (let k = 0; k < 80; k++) {
      const next = Math.max(p - (f(p, rL, pL, cL) + f(p, rR, pR, cR)) / (fd(p, rL, pL, cL) + fd(p, rR, pR, cR)), 1e-10);
      if (Math.abs(next - p) < 1e-15) {
        p = next;
        break;
      }
      p = next;
    }
    const uStar = 0.5 * (f(p, rR, pR, cR) - f(p, rL, pL, cL));
    const s = t > 0 ? (x - 0.5) / t : 0;
    if (s <= uStar) {
      const cStar = cL * Math.pow(p / pL, (g - 1) / (2 * g));
      if (s < -cL) return rL;
      if (s > uStar - cStar) return rL * Math.pow(p / pL, 1 / g);
      // Inside the left rarefaction fan. The (g-1)/2 factor on s is essential — without
      // it the fan head lands at 1.72*cL instead of cL.
      const c = (2 / (g + 1)) * (cL - ((g - 1) / 2) * s);
      return rL * Math.pow(c / cL, 2 / (g - 1));
    }
    const SR = cR * Math.sqrt(((g + 1) / (2 * g)) * (p / pR) + (g - 1) / (2 * g));
    if (s >= SR) return rR;
    return rR * ((p / pR + (g - 1) / (g + 1)) / (((g - 1) / (g + 1)) * (p / pR) + 1));
  }

  /**
   * Run Sod in solver units. The solver fixes gamma at the exhaust value, so the exact
   * solution is evaluated with the same gamma rather than the textbook 1.4.
   */
  function sod(cells: number, limiter: SlopeLimiter) {
    // Scale the classic initial data up to real pressures so nothing hits the floors.
    const SCALE = 1e5;
    const p = tube(cells, 1, limiter);
    for (let i = 0; i < p.n; i++) {
      const x = (i + 0.5) * p.dx;
      if (x < 0.5) p.setPrimitive(i, 1, 0, SCALE);
      else p.setPrimitive(i, 0.125, 0, 0.1 * SCALE);
    }
    // Time is scaled with the sound speed: c ~ sqrt(p/rho), so t_scaled = t/sqrt(SCALE).
    const tEnd = 0.2 / Math.sqrt(SCALE);
    let t = 0;
    while (t < tEnd) {
      const h = Math.min(2e-5, tEnd - t);
      p.advance(h, SHUT);
      t += h;
    }
    let l1 = 0;
    let maxRho = 0;
    let minRho = Infinity;
    for (let i = 0; i < p.n; i++) {
      const x = (i + 0.5) * p.dx;
      l1 += Math.abs(p.densityAt(i) - sodExact(x, 0.2, GAS.gammaExh)) * p.dx;
      maxRho = Math.max(maxRho, p.densityAt(i));
      minRho = Math.min(minRho, p.densityAt(i));
    }
    return { l1, overshoot: maxRho - 1, undershoot: 0.125 - minRho };
  }

  it('matches the exact Sod solution and converges', () => {
    const coarse = sod(100, 'mc');
    const fine = sod(400, 'mc');
    expect(coarse.l1).toBeLessThan(0.03);
    expect(fine.l1).toBeLessThan(coarse.l1);
    // L1 convergence on a discontinuous problem is capped near first order because the
    // limiter drops to first order at the shock and the contact.
    const order = Math.log(coarse.l1 / fine.l1) / Math.log(4);
    expect(order).toBeGreaterThan(0.5);
  });

  it('is TVD: no limiter overshoots the initial data', () => {
    // This is the whole point of gradient limiting. An unlimited second-order scheme
    // rings at the shock, and those oscillations are audible as well as unphysical.
    for (const limiter of ['minmod', 'mc', 'vanleer'] as SlopeLimiter[]) {
      const r = sod(200, limiter);
      expect(r.overshoot, `${limiter} overshoot`).toBeLessThan(1e-6);
      expect(r.undershoot, `${limiter} undershoot`).toBeLessThan(1e-6);
    }
  });

  it('mc is less diffusive than minmod, as the limiter theory says', () => {
    expect(sod(200, 'mc').l1).toBeLessThan(sod(200, 'minmod').l1);
  });
});

describe('nonlinear steepening — the reason for the whole exercise', () => {
  const LAMBDA = 0.4;

  /**
   * Steepness of a travelling wave after propagating `travel` metres, as a multiple of
   * the steepness the same wave would have if it stayed sinusoidal.
   *
   * A long duct filled with many wavelengths, measured only in the middle half so that
   * nothing reflected from either end can reach the measurement window. Measuring a
   * short burst in a closed tube instead does not work: reflections off the walls
   * generate harmonics of their own, amplitude-independently, which swamps the effect
   * being tested.
   */
  function steepnessRatio(amplitudeBar: number, travel: number): number {
    const length = 4;
    const p = new EulerPipe([makeSegment({ length, dIn: 0.05 })], FS, GAS.tAmb, {
      cellSize: LAMBDA / 100,
      maxCells: 4096,
      radiate: false,
      heatTransfer: false,
      linearDamping: 0,
      darcyFriction: 0,
      maxSubsteps: 4096,
    });
    const rho0 = GAS.pAmb / (GAS.R * GAS.tAmb);
    const c0 = speedOfSound(GAS.tAmb);
    const amp = amplitudeBar * 1e5;
    for (let i = 0; i < p.n; i++) {
      const x = (i + 0.5) * p.dx;
      const dp = amp * Math.sin((2 * Math.PI * x) / LAMBDA);
      // A rightward-travelling simple wave.
      p.setPrimitive(i, rho0 + dp / (c0 * c0), dp / (rho0 * c0), GAS.pAmb + dp);
    }

    const tEnd = travel / c0;
    let t = 0;
    while (t < tEnd) {
      const h = Math.min(2e-5, tEnd - t);
      p.advance(h, SHUT);
      t += h;
    }

    // Steepest gradient and peak amplitude, both from the middle half only.
    const lo = Math.floor(p.n * 0.25);
    const hi = Math.floor(p.n * 0.75);
    let maxGrad = 0;
    let peak = 0;
    for (let i = lo; i < hi; i++) {
      maxGrad = Math.max(maxGrad, Math.abs(p.pressureAt(i + 1) - p.pressureAt(i)) / p.dx);
      peak = Math.max(peak, Math.abs(p.pressureAt(i) - GAS.pAmb));
    }
    // A pure sine of this amplitude and wavelength has max gradient 2*pi*A/lambda.
    return maxGrad / ((2 * Math.PI * peak) / LAMBDA);
  }

  it('a large-amplitude wave steepens; a small one stays sinusoidal', () => {
    const quiet = steepnessRatio(0.002, 0.8);
    const loud = steepnessRatio(0.8, 0.8);
    // The old linear waveguide gave exactly 1.0 here at every amplitude, which is why
    // open pipes never sounded brassy.
    expect(quiet).toBeLessThan(1.15);
    expect(loud).toBeGreaterThan(1.5);
  });

  it('steepening grows with amplitude', () => {
    const a = steepnessRatio(0.1, 0.8);
    const b = steepnessRatio(0.8, 0.8);
    expect(b).toBeGreaterThan(a);
  });

  it('steepening grows with distance travelled', () => {
    expect(steepnessRatio(0.5, 1.0)).toBeGreaterThan(steepnessRatio(0.5, 0.15));
  });
});

describe('conservation', () => {
  it('a sealed duct conserves mass and energy', () => {
    const p = tube(120, 1);
    // Put a pressure bump in the middle so waves slosh around.
    for (let i = 40; i < 60; i++) {
      p.setPrimitive(i, 1.4, 0, 1.6 * GAS.pAmb);
    }
    const m0 = p.totalMass();
    const e0 = p.totalEnergy();
    for (let k = 0; k < 400; k++) p.advance(1 / FS, SHUT);
    expect(Math.abs(p.totalMass() - m0) / m0).toBeLessThan(1e-9);
    expect(Math.abs(p.totalEnergy() - e0) / e0).toBeLessThan(1e-9);
  });

  /**
   * Acoustic energy, i.e. the deviation from a quiescent duct.
   *
   * Total energy is the wrong thing to watch for stability. It is dominated by internal
   * energy — order 1e5 J/m^3 — so an acoustic field can grow by three orders of
   * magnitude while total energy stays conserved to 1e-9. A conservation test alone
   * happily passes a duct that is screaming.
   */
  function acousticEnergy(p: EulerPipe): number {
    let e = 0;
    for (let i = 0; i < p.n; i++) {
      const dp = p.pressureAt(i) - GAS.pAmb;
      const u = p.velocityAt(i);
      e += (dp * dp + p.densityAt(i) * 1e5 * u * u) * p.areaOf(i);
    }
    return e;
  }

  it('a cavity between two area changes does not pump itself', () => {
    // This is a regression guard on two coupled bugs in the Hancock predictor. The area
    // source has to be in the predictor as well as the corrector, *and* the predictor's
    // fluxes have to be area-weighted so it stays well balanced at rest. Omitting the
    // source gave 650x acoustic energy growth here; adding it without area-weighting the
    // fluxes instead invented momentum at rest and made the transient ~56 dB too loud.
    // A single expansion or contraction hides both — only a cavity traps the error.
    for (const segments of [
      [
        makeSegment({ kind: 'pipe', length: 0.35, dIn: 0.04 }),
        makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.14 }),
        makeSegment({ kind: 'pipe', length: 0.35, dIn: 0.04 }),
      ],
      [
        makeSegment({ kind: 'pipe', length: 0.5, dIn: 0.04 }),
        makeSegment({ kind: 'chamber', length: 0.3, dIn: 0.04, dOut: 0.14 }),
        makeSegment({ kind: 'pipe', length: 0.2, dIn: 0.04 }),
      ],
    ]) {
      const p = new EulerPipe(segments, FS, GAS.tAmb, {
        cellSize: 0.008,
        maxCells: 512,
        heatTransfer: false,
        // Production damping. The scheme retains a small second-order inconsistency in a
        // near-lossless sealed cavity, whose growth rate measures about 1.4 /s — two
        // orders of magnitude below this, so it stays comfortably suppressed.
        linearDamping: 150,
        darcyFriction: 0,
        radiate: false,
        maxSubsteps: 64,
      });
      p.setPrimitive(Math.floor(p.n / 2), p.densityAt(0), 0, GAS.pAmb * 1.05);

      let early = 0;
      for (let k = 0; k < FS * 0.35; k++) {
        p.advance(1 / FS, SHUT);
        if (k === Math.floor(FS * 0.03)) early = acousticEnergy(p);
      }
      expect(early).toBeGreaterThan(0);
      expect(acousticEnergy(p) / early).toBeLessThan(1.1);
    }
  });

  it('is well balanced: a duct at rest stays at rest whatever its shape', () => {
    // If the flux difference and the p dA source do not cancel exactly at rest, varying
    // area spontaneously generates flow.
    const p = new EulerPipe(
      [
        makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.035 }),
        makeSegment({ kind: 'cone', length: 0.3, dIn: 0.035, dOut: 0.12 }),
        makeSegment({ kind: 'chamber', length: 0.3, dIn: 0.12, dOut: 0.2 }),
        makeSegment({ kind: 'cone', length: 0.2, dIn: 0.12, dOut: 0.03 }),
      ],
      FS,
      900,
      { heatTransfer: false, radiate: false, maxSubsteps: 64 },
    );
    for (let k = 0; k < 2000; k++) p.advance(1 / FS, SHUT);
    let maxU = 0;
    for (let i = 0; i < p.n; i++) maxU = Math.max(maxU, Math.abs(p.velocityAt(i)));
    expect(maxU).toBeLessThan(1e-6);
  });

  it('a sealed duct does not gain energy, so it cannot run away', () => {
    const p = tube(120, 1, 'mc');
    for (let i = 0; i < p.n; i++) {
      const x = (i + 0.5) * p.dx;
      p.setPrimitive(i, 1.2, 60 * Math.sin(12 * x), GAS.pAmb * (1 + 0.3 * Math.sin(9 * x)));
    }
    const e0 = p.totalEnergy();
    for (let k = 0; k < 3000; k++) p.advance(1 / FS, SHUT);
    expect(p.totalEnergy()).toBeLessThan(e0 * 1.0001);
    for (let i = 0; i < p.n; i++) {
      expect(Number.isFinite(p.pressureAt(i))).toBe(true);
      expect(p.pressureAt(i)).toBeGreaterThan(0);
    }
  });
});

describe('the linear acoustics the waveguide gave for free', () => {
  /** Impulse response at the mouth of a closed-open duct driven by a flow pulse. */
  function impulseResponse(p: EulerPipe, n: number, kick: number): Float32Array {
    const out = new Float32Array(n);
    const rho0 = p.densityAt(0);
    // Inject a short velocity pulse into the first cell.
    p.setPrimitive(0, rho0, kick, GAS.pAmb);
    for (let i = 0; i < n; i++) {
      out[i] = p.advance(1 / FS, SHUT).mouthFlow;
    }
    return out;
  }

  /** Build a test duct with light damping so its resonances are sharp. */
  function duct(segments: ReturnType<typeof makeSegment>[], portTemp: number): EulerPipe {
    return new EulerPipe(segments, FS, portTemp, {
      cellSize: 0.008,
      maxCells: 512,
      // Heat transfer off so the temperature field stays as initialised.
      heatTransfer: false,
      linearDamping: 6,
      darcyFriction: 0,
      maxSubsteps: 64,
    });
  }

  /**
   * Measured fundamental, located by searching a window around the solver's own
   * quarter-wave prediction.
   *
   * Taking "the strongest peak in a wide range" is fragile: which mode dominates depends
   * on the geometry, so it can silently compare mode 1 in one case against mode 3 in
   * another — which is exactly how this test suite first went wrong.
   */
  function fundamentalOf(segments: ReturnType<typeof makeSegment>[], portTemp: number): number {
    const p = duct(segments, portTemp);
    const predicted = p.quarterWaveHz();
    const mag = magnitudeSpectrum(impulseResponse(p, FFT, 2), FFT);
    const peaks = findPeaks(mag, FS, FFT, predicted * 0.55, predicted * 1.6, 0.1);
    expect(peaks.length, `no peak near ${predicted.toFixed(0)} Hz`).toBeGreaterThan(0);
    return peaks.reduce((best, q) =>
      Math.abs(q.hz - predicted) < Math.abs(best.hz - predicted) ? q : best,
    ).hz;
  }

  const fundamental = (length: number) =>
    fundamentalOf([makeSegment({ length, dIn: 0.045 })], GAS.tAmb);

  it('resonates at odd multiples of c/4L', () => {
    for (const length of [0.6, 1.0]) {
      const p = duct([makeSegment({ length, dIn: 0.05 })], GAS.tAmb);
      const ir = impulseResponse(p, FFT, 2);
      const mag = magnitudeSpectrum(ir, FFT);

      // Cross-check the solver's own prediction against the closed form first.
      const f1 = speedOfSound(GAS.tAmb) / (4 * p.totalLength);
      expect(Math.abs(p.quarterWaveHz() - f1) / f1).toBeLessThan(0.02);
      const peaks = findPeaks(mag, FS, FFT, f1 * 0.5, f1 * 4.5, 0.05);
      expect(peaks.length).toBeGreaterThanOrEqual(2);

      for (const mode of [1, 3]) {
        const want = mode * f1;
        const near = peaks.reduce((best, q) =>
          Math.abs(q.hz - want) < Math.abs(best.hz - want) ? q : best,
        );
        // Looser than the waveguide's 6%: a finite-volume scheme has numerical
        // dispersion, where a delay line has none.
        expect(
          Math.abs(near.hz - want) / want,
          `L=${length} mode ${mode}: wanted ~${want.toFixed(0)} Hz, got ${near.hz.toFixed(0)} Hz`,
        ).toBeLessThan(0.1);
      }
    }
  });

  it('halving the length roughly doubles the fundamental', () => {
    const ratio = fundamental(0.5) / fundamental(1.0);
    expect(ratio).toBeGreaterThan(1.75);
    expect(ratio).toBeLessThan(2.25);
  });

  it('a chamber lowers the tuning, so area changes still reflect', () => {
    const straight = fundamentalOf([makeSegment({ length: 1, dIn: 0.04 })], GAS.tAmb);
    const chambered = fundamentalOf(
      [
        makeSegment({ kind: 'pipe', length: 0.5, dIn: 0.04 }),
        makeSegment({ kind: 'chamber', length: 0.3, dIn: 0.04, dOut: 0.14 }),
        makeSegment({ kind: 'pipe', length: 0.2, dIn: 0.04 }),
      ],
      GAS.tAmb,
    );
    // The volume acts as a compliance, so the system tunes below a plain pipe of the
    // same overall length.
    expect(chambered).toBeLessThan(straight * 0.95);
  });

  it('hot gas raises the resonance', () => {
    const pipe = [makeSegment({ length: 1, dIn: 0.045 })];
    // Sound travels faster in hot gas, so the same physical pipe tunes higher — which
    // is why an exhaust note shifts as the engine warms up.
    expect(fundamentalOf(pipe, 950)).toBeGreaterThan(fundamentalOf(pipe, GAS.tAmb) * 1.2);
  });
});

describe('geometry and robustness', () => {
  it('fixes cell length, so a short pipe costs less than a long one', () => {
    const short = new EulerPipe([makeSegment({ length: 0.4, dIn: 0.04 })], FS, 900);
    const long = new EulerPipe([makeSegment({ length: 1.4, dIn: 0.04 })], FS, 900);
    expect(long.n).toBeGreaterThan(short.n * 2);
    // Cell length stays put, which is what keeps the CFL timestep from collapsing when
    // the user shortens the pipe.
    expect(Math.abs(long.dx - short.dx) / short.dx).toBeLessThan(0.25);
  });

  it('reports the port portion so the display can skip it', () => {
    const p = new EulerPipe([makeSegment({ length: 0.5, dIn: 0.042 })], FS, 900, {
      port: { length: 0.055, diameter: 0.034 },
    });
    expect(p.portCells).toBeGreaterThan(0);
    expect(p.portCells).toBeLessThan(p.n);
    const bare = new EulerPipe([makeSegment({ length: 0.5, dIn: 0.042 })], FS, 900);
    expect(bare.portCells).toBe(0);
    expect(p.totalLength).toBeGreaterThan(bare.totalLength);
  });

  it('limits area steps without losing a chamber its volume', () => {
    const face = Float64Array.from([1, 1, 1, 1, 9.6, 9.6, 9.6, 9.6, 9.6, 1, 1, 1, 1, 0.1, 0.1, 0.1]);
    const drawn = [...face];
    limitAreaRatio(face, 1.6);
    for (let f = 0; f + 1 < face.length; f++) {
      expect(Math.max(face[f]! / face[f + 1]!, face[f + 1]! / face[f]!)).toBeLessThanOrEqual(1.6 + 1e-9);
    }
    const vol = (a: ArrayLike<number>) => {
      let v = 0;
      for (let f = 0; f < a.length; f++) v += (f === 0 || f === a.length - 1 ? 0.5 : 1) * a[f]!;
      return v;
    };
    expect(vol(face) / vol(drawn)).toBeCloseTo(1, 6);
    // The step straddles the drawn edge: the pipe beside it widens, the body narrows.
    expect(face[3]!).toBeGreaterThan(1);
    expect(face[4]!).toBeLessThan(9.6);
  });

  it('keeps a drawn muffler can close to its drawn volume and diameter', () => {
    const L = 0.34;
    const p = new EulerPipe(
      [
        makeSegment({ length: 0.5, dIn: 0.042 }),
        makeSegment({ kind: 'chamber', length: L, dIn: 0.042, dOut: 0.13 }),
        makeSegment({ length: 0.5, dIn: 0.042 }),
      ],
      FS,
      900,
      { cellSize: 0.035, singleStep: true },
    );
    let vol = 0;
    let peak = 0;
    for (let i = 0; i < p.n; i++) {
      const x = (i + 0.5) * p.dx;
      if (x < 0.25 || x > 0.75 + L) continue;
      vol += (p.areaOf(i) - (Math.PI / 4) * 0.042 ** 2) * p.dx;
      peak = Math.max(peak, Math.sqrt((4 * p.areaOf(i)) / Math.PI));
    }
    const drawn = (Math.PI / 4) * (0.13 ** 2 - 0.042 ** 2) * L * 0.84;
    expect(vol / drawn).toBeGreaterThan(0.93);
    expect(vol / drawn).toBeLessThan(1.07);
    expect(peak).toBeGreaterThan(0.12);
  });

  it('survives degenerate geometry and a violent valve', () => {
    for (const pipe of [
      [],
      [makeSegment({ length: 0.01, dIn: 0.008 })],
      [makeSegment({ kind: 'chamber', length: 2, dIn: 0.02, dOut: 0.4 })],
    ]) {
      const p = new EulerPipe(pipe, FS, 1100, { port: { length: 0.05, diameter: 0.034 } });
      for (let k = 0; k < 4000; k++) {
        const r = p.advance(1 / FS, {
          throatArea: k % 200 < 60 ? 7e-4 : 0,
          cylPressure: k % 200 < 60 ? 7e6 : GAS.pAmb,
          cylTemp: 1800,
        });
        expect(Number.isFinite(r.mouthFlow)).toBe(true);
        expect(Number.isFinite(r.portPressure)).toBe(true);
        expect(r.portPressure).toBeGreaterThan(0);
      }
    }
  });

  it('keeps the substep count inside its budget', () => {
    const p = new EulerPipe([makeSegment({ length: 1.3, dIn: 0.045 })], FS, 950, {
      port: { length: 0.055, diameter: 0.034 },
    });
    let worst = 0;
    let total = 0;
    const iters = 4000;
    for (let k = 0; k < iters; k++) {
      const r = p.advance(1 / FS, {
        throatArea: k % 300 < 90 ? 7e-4 : 0,
        cylPressure: k % 300 < 90 ? 6e6 : GAS.pAmb,
        cylTemp: 1600,
      });
      worst = Math.max(worst, r.substeps);
      total += r.substeps;
    }
    // Sized for roughly 2 substeps per audio sample; the cap is 8.
    expect(total / iters).toBeLessThan(4);
    expect(worst).toBeLessThanOrEqual(8);
  });
});

describe('wall temperature is solved, not assumed', () => {
  function duct(over: EulerPipeOptions = {}) {
    return new EulerPipe([makeSegment({ length: 1.2, dIn: 0.042 })], FS, 950, {
      port: { length: 0.055, diameter: 0.034 },
      ...over,
    });
  }
  /** Hold the valve open onto a hot, pressurised cylinder, as a running engine would. */
  const HOT = { throatArea: 4e-4, cylPressure: 2.2e5, cylTemp: 1300 };

  it('warms up from cold over tens of seconds', () => {
    const p = duct({ initialWallTemp: GAS.tAmb });
    expect(p.meanWallTemp()).toBeCloseTo(GAS.tAmb, 0);
    const after = (seconds: number) => {
      for (let k = 0; k < FS * seconds; k++) p.advance(1 / FS, HOT);
      return p.meanWallTemp();
    };
    const t2 = after(2);
    const t20 = after(18);
    // Monotonic, and slow: typical 1.2 mm tubing has a time constant of tens of seconds,
    // so two seconds must not get anywhere near equilibrium.
    expect(t2).toBeGreaterThan(GAS.tAmb + 5);
    expect(t20).toBeGreaterThan(t2 + 50);
    expect(t2).toBeLessThan(500);
  });

  it('runs cooler downstream, which a fixed wall temperature could not do', () => {
    const p = duct();
    for (let k = 0; k < FS * 20; k++) p.advance(1 / FS, HOT);
    const wall = new Float32Array(128);
    const gas = new Float32Array(128);
    p.sampleWallTemperature(wall);
    p.sampleTemperature(gas);
    // Both the wall and the gas must fall along the duct.
    expect(wall[0]!).toBeGreaterThan(wall[127]! + 40);
    expect(gas[0]!).toBeGreaterThan(gas[127]!);
    // And the wall must sit between the gas and ambient.
    expect(wall[64]!).toBeLessThan(gas[64]!);
    expect(wall[64]!).toBeGreaterThan(GAS.tAmb);
  });

  it('airflow cools it, and cooler gas lowers the tuning', () => {
    const settle = (airSpeed: number) => {
      const p = duct({ airSpeed });
      for (let k = 0; k < FS * 20; k++) p.advance(1 / FS, HOT);
      return { wall: p.meanWallTemp(), hz: p.quarterWaveHz() };
    };
    const still = settle(0);
    const moving = settle(30);
    expect(moving.wall).toBeLessThan(still.wall - 30);
    // Sound travels slower in cooler gas, so the same pipe tunes lower.
    expect(moving.hz).toBeLessThan(still.hz);
  });

  it('a thicker wall warms more slowly, because thermal mass scales with it', () => {
    const after30 = (wallThickness: number) => {
      const p = duct({ wallThickness, initialWallTemp: GAS.tAmb });
      for (let k = 0; k < FS * 15; k++) p.advance(1 / FS, HOT);
      return p.meanWallTemp();
    };
    expect(after30(0.0006)).toBeGreaterThan(after30(0.003) + 30);
  });

  it('a rebuild inherits the wall state instead of discarding it', () => {
    const p = duct({ initialWallTemp: GAS.tAmb });
    for (let k = 0; k < FS * 15; k++) p.advance(1 / FS, HOT);
    const warm = p.meanWallTemp();
    expect(warm).toBeGreaterThan(GAS.tAmb + 40);

    // A geometry edit must not throw away a thermal state that takes half a minute to
    // rebuild, or the tuning would jump every time the user drags a handle.
    const rebuilt = new EulerPipe([makeSegment({ length: 1.4, dIn: 0.042 })], FS, 950, {
      port: { length: 0.055, diameter: 0.034 },
      inheritWall: p.exportWall(),
    });
    expect(rebuilt.meanWallTemp()).toBeCloseTo(warm, -1);
  });
});

describe('friction acts on what it physically should', () => {
  it('a steady mean flow is barely touched by the acoustic damping term', () => {
    // The linear boundary-layer term is acoustic damping and must skip the mean flow.
    // Applied to the total velocity it braked the mean hard — at low speed it outweighed
    // Darcy roughly forty to one.
    const steady = (linearDamping: number) => {
      const p = new EulerPipe([makeSegment({ length: 1, dIn: 0.04 })], FS, 900, {
        heatTransfer: false,
        linearDamping,
        darcyFriction: 0.03,
      });
      const valve = { throatArea: 6e-4, cylPressure: 2.5e5, cylTemp: 1100 };
      for (let k = 0; k < FS * 0.5; k++) p.advance(1 / FS, valve);
      return p.velocityAt(Math.floor(p.n / 2));
    };
    const none = steady(0);
    const full = steady(150);
    expect(none).toBeGreaterThan(50);
    // Within 15%: the mean is exempt, so only the quadratic terms should bite.
    expect(full).toBeGreaterThan(none * 0.85);
  });

  it('but acoustic waves are still damped, at the calibrated rate', () => {
    const decayPerPass = (linearDamping: number) => {
      const length = 1;
      const p = new EulerPipe([makeSegment({ length, dIn: 0.04 })], FS, 600, {
        heatTransfer: false,
        radiate: false,
        linearDamping,
        darcyFriction: 0,
      });
      const rho0 = GAS.pAmb / (GAS.R * 600);
      const c = speedOfSound(600);
      for (let i = 0; i < p.n; i++) {
        const dp = 200 * Math.sin((2 * Math.PI * (i + 0.5) * p.dx) / 0.5);
        p.setPrimitive(i, rho0 + dp / (c * c), 0, GAS.pAmb + dp);
      }
      const amp = () => {
        let m = 0;
        for (let i = 0; i < p.n; i++) m = Math.max(m, Math.abs(p.pressureAt(i) - GAS.pAmb));
        return m;
      };
      const a0 = amp();
      const seconds = 0.05;
      for (let k = 0; k < FS * seconds; k++) p.advance(1 / FS, SHUT);
      return (20 * Math.log10(amp() / a0)) / ((c * seconds) / length);
    };
    // Calibrated to about 1.5 dB per pass of a 1 m duct; numerical dissipation alone is
    // roughly 0.25, so the physical term has to dominate.
    const physical = decayPerPass(150);
    const numericalOnly = decayPerPass(0);
    expect(physical).toBeLessThan(-1);
    expect(physical).toBeLessThan(numericalOnly * 3);
  });
});

describe('the open end reflects less at high frequency, as a real one does', () => {
  /**
   * Energy decay time of a single closed-open mode, ms, with all wall losses disabled so
   * only the boundary can remove energy.
   */
  function modeDecayMs(order: number, dia: number, cfl?: number): number {
    const p = new EulerPipe([makeSegment({ length: 1, dIn: dia })], FS, GAS.tAmb, {
      cellSize: 0.003,
      maxCells: 4096,
      heatTransfer: false,
      linearDamping: 0,
      darcyFriction: 0,
      maxSubsteps: 64,
      ...(cfl === undefined ? {} : { cfl }),
    });
    const c = speedOfSound(GAS.tAmb);
    const rho0 = GAS.pAmb / (GAS.R * GAS.tAmb);
    // Closed-open mode shape: p ~ cos(kx) with k = (2m-1)pi/2L.
    const k = ((2 * order - 1) * Math.PI) / (2 * p.totalLength);
    for (let i = 0; i < p.n; i++) {
      const dp = 50 * Math.cos(k * (i + 0.5) * p.dx);
      p.setPrimitive(i, rho0 + dp / (c * c), 0, GAS.pAmb + dp);
    }
    const energy = () => {
      let e = 0;
      for (let i = 0; i < p.n; i++) {
        const dp = p.pressureAt(i) - GAS.pAmb;
        e += dp * dp + (rho0 * c) ** 2 * p.velocityAt(i) ** 2;
      }
      return e;
    };
    const e0 = energy();
    for (let s = 0; s < FS * 0.5; s++) {
      p.advance(1 / FS, SHUT);
      if (energy() < e0 / Math.E) return ((s + 1) / FS) * 1000;
    }
    return 500;
  }

  it('high modes die away far faster than low ones', () => {
    // The whole point: a low mode barely radiates and rings for a long time, while a high
    // mode leaves through the mouth almost immediately. Measures 188 ms at 84 Hz against
    // 8 ms at 1.9 kHz.
    const low = modeDecayMs(1, 0.05);
    const mid = modeDecayMs(5, 0.05);
    const high = modeDecayMs(12, 0.05);
    expect(mid).toBeLessThan(low * 0.5);
    expect(high).toBeLessThan(mid * 0.5);
    expect(low).toBeGreaterThan(50);
  });

  it('decay matches the reflection coefficient it should have', () => {
    // Energy falls by |R|^2 each round trip, so the decay time pins |R| down. Compared
    // against the Levine-Schwinger result for an unflanged pipe, which the one-pole at c/a
    // tracks closely.
    const c = speedOfSound(GAS.tAmb);
    const a = 0.025;
    const roundTrip = 2 / c;
    for (const [order, lsRef] of [
      [1, 0.99],
      [5, 0.94],
      [12, 0.77],
    ] as const) {
      const f = ((2 * order - 1) * c) / 4;
      const ka = (2 * Math.PI * f * a) / c;
      const tau = modeDecayMs(order, 0.05) / 1000;
      // tau = roundTrip / (-2 ln|R|)
      const measured = Math.exp(-roundTrip / (2 * tau));
      expect(
        Math.abs(measured - lsRef),
        `ka=${ka.toFixed(2)}: |R| measured ${measured.toFixed(3)}, expected near ${lsRef}`,
      ).toBeLessThan(0.1);
    }
  });

  it('is independent of how many substeps the solver takes', () => {
    // The regression guard. The reflection filter runs once per CFL substep, so its
    // coefficient has to come from the substep duration. Deriving it from the audio sample
    // period instead doubled the corner frequency and made high-frequency standing waves
    // linger about three times too long.
    const twoSubsteps = modeDecayMs(10, 0.05, 0.85);
    const manySubsteps = modeDecayMs(10, 0.05, 0.2);
    expect(manySubsteps).toBeGreaterThan(twoSubsteps * 0.7);
    expect(manySubsteps).toBeLessThan(twoSubsteps * 1.4);
  });

  it('a wider mouth radiates high frequencies away sooner', () => {
    // The corner is c/a, so a bigger radius reflects less at a given frequency.
    expect(modeDecayMs(8, 0.09)).toBeLessThan(modeDecayMs(8, 0.03));
  });
});
