/**
 * Twin-cylinder tests: the firing geometry, and the junction that merges two exhausts.
 *
 * The claims worth checking are the ones a listener would notice. An evenly-firing twin has
 * no half-order component at all, because both cylinders fire 360 degrees apart and the firing
 * frequency simply doubles. An unevenly-firing one does, and more strongly the more uneven it
 * is — that is the whole of the Harley thump. And a shared collector has to actually couple the
 * banks, or it is just two singles playing at once.
 */

import { describe, expect, it } from 'vitest';
import {
  ENGINE_PRESETS,
  GAS,
  defaultConfig,
  firingOffsetDeg,
  makeSegment,
  type EngineSpec,
} from '../src/model/spec.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';
import type { ExhaustGraph } from '../src/model/exhaustGraph.js';
import { ExhaustSystem } from '../src/audio/worklet/exhaustSystem.js';
import { bandEnergy, hann, magnitudeSpectrum } from './spectrum.js';

const FS = 48000;
const N = 65536;

function twin(over: Partial<EngineSpec> = {}, primaryLength = 0.35): EngineSim {
  const cfg = defaultConfig();
  cfg.engine = {
    ...cfg.engine,
    cylinders: 2,
    exhaustLayout: '2into1',
    vAngle: 45,
    firingOffset: null,
    rpm: 3000,
    ...over,
  };
  cfg.pipe = [makeSegment({ kind: 'pipe', length: primaryLength, dIn: 0.04 })];
  const sim = new EngineSim(FS, cfg);
  sim.render(FS * 2);
  return sim;
}

/**
 * Two runners into one collector, as a graph.
 *
 * Built by hand rather than through `compileLayout` so the test states the topology it is testing
 * instead of depending on what a layout name currently means.
 */
function twinGraph(): ExhaustGraph {
  return {
    ducts: [
      {
        id: 'runner0',
        segments: [makeSegment({ length: 0.6, dIn: 0.042 })],
        from: { kind: 'valve', cylinder: 0 },
        to: { kind: 'node', node: 'merge' },
      },
      {
        id: 'runner1',
        segments: [makeSegment({ length: 0.6, dIn: 0.042 })],
        from: { kind: 'valve', cylinder: 1 },
        to: { kind: 'node', node: 'merge' },
      },
      {
        id: 'collector',
        segments: [makeSegment({ length: 0.9, dIn: 0.055 })],
        from: { kind: 'node', node: 'merge' },
        to: { kind: 'mouth' },
      },
    ],
  };
}

describe('firing geometry', () => {
  it('a shared crankpin ties the firing interval to the V angle', () => {
    const base = defaultConfig().engine;
    // 360 + V, so the two intervals are 360+V and 360-V.
    expect(firingOffsetDeg({ ...base, vAngle: 45, firingOffset: null })).toBe(405);
    expect(firingOffsetDeg({ ...base, vAngle: 90, firingOffset: null })).toBe(450);
    expect(firingOffsetDeg({ ...base, vAngle: 0, firingOffset: null })).toBe(360);
  });

  it('an override breaks that relationship, for layouts a shared pin cannot make', () => {
    const base = defaultConfig().engine;
    expect(firingOffsetDeg({ ...base, vAngle: 45, firingOffset: 270 })).toBe(270);
    expect(firingOffsetDeg({ ...base, vAngle: 90, firingOffset: 360 })).toBe(360);
  });

  it('both banks run, a fixed number of degrees apart, and stay there', () => {
    const sim = twin({ vAngle: 45 });
    // Mind the sign: cylinder 2 fires 405 degrees *after* cylinder 1, which means it is 405
    // degrees *behind* on the crank — it has that much further to go to reach its own firing
    // TDC. Asserting `b - a` instead would demand 315 and quietly accept a reversed firing
    // order, which is invisible in a twin and wrong in a V8.
    const gap = () => {
      const [a, b] = sim.cylinders;
      return ((a!.angle - b!.angle + 720) % 720);
    };
    const first = gap();
    expect(first).toBeCloseTo(405, 0);
    // They share one crank, so the offset must not drift over thousands of cycles.
    sim.render(FS * 3);
    expect(gap()).toBeCloseTo(first, 0);
  });

  it('both cylinders actually fire', () => {
    const sim = twin();
    const peaks = [0, 0];
    for (let i = 0; i < FS; i++) {
      sim.tick();
      for (let b = 0; b < 2; b++) {
        peaks[b] = Math.max(peaks[b]!, sim.cylinders[b]!.pressure(sim.engine));
      }
    }
    for (const p of peaks) expect(p).toBeGreaterThan(20e5);
    // Symmetric cylinders, so neither should dominate.
    expect(Math.abs(peaks[0]! - peaks[1]!) / peaks[0]!).toBeLessThan(0.5);
  });
});

describe('firing interval shapes the spectrum', () => {
  const f0 = 3000 / 120; // 25 Hz, one firing per cylinder per two revolutions

  function halfOrderRatio(over: Partial<EngineSpec>): number {
    const mag = magnitudeSpectrum(hann(twin(over).render(N)), N);
    // Energy at the half order against the full order the twin always has.
    return bandEnergy(mag, FS, N, f0, 3) / bandEnergy(mag, FS, N, 2 * f0, 3);
  }

  it('an evenly firing twin has essentially no half order', () => {
    // 360/360 means the two firings are evenly spaced, so the fundamental is 2*f0 and the
    // half order has nothing to excite it. Measures about four orders of magnitude down.
    expect(halfOrderRatio({ firingOffset: 360 })).toBeLessThan(0.01);
  });

  it('an unevenly firing twin does, and more so the more uneven it is', () => {
    const v45 = halfOrderRatio({ vAngle: 45, firingOffset: null }); // 405/315
    const v90 = halfOrderRatio({ vAngle: 90, firingOffset: null }); // 450/270
    expect(v45).toBeGreaterThan(0.005);
    expect(v90).toBeGreaterThan(v45);
  });

  it('a single is not the same as a twin', () => {
    const single = defaultConfig();
    single.engine = { ...single.engine, cylinders: 1, exhaustLayout: 'single', rpm: 3000 };
    single.pipe = [makeSegment({ kind: 'pipe', length: 0.35, dIn: 0.04 })];
    const s = new EngineSim(FS, single);
    s.render(FS * 2);
    const singleMag = magnitudeSpectrum(hann(s.render(N)), N);
    const twinMag = magnitudeSpectrum(hann(twin({ firingOffset: 360 }).render(N)), N);

    // A single fires once per cycle, so it has a strong half order; an evenly firing twin
    // essentially none. Same rpm, same pipe.
    const singleRatio =
      bandEnergy(singleMag, FS, N, f0, 3) / bandEnergy(singleMag, FS, N, 2 * f0, 3);
    const twinRatio = bandEnergy(twinMag, FS, N, f0, 3) / bandEnergy(twinMag, FS, N, 2 * f0, 3);
    expect(singleRatio).toBeGreaterThan(twinRatio * 20);
  });
});

describe('the collector couples the banks', () => {
  /**
   * Bank 0's port pressure over 0.1 s, once warm.
   *
   * The heavy flywheel is what makes this measurement mean anything. There are *two* paths by
   * which one cylinder can reach the other, and the crankshaft is the one that is easy to
   * forget: torque ripple from bank 1 speeds and slows the shared crank, so bank 0's own
   * schedule shifts. Pinning the rpm slider is not enough, because the ripple is applied on
   * top of the mean speed. Only with the inertia turned up does the exhaust become the sole
   * remaining path, which is the one this block is about.
   */
  function bank0Port(layout: EngineSpec['exhaustLayout'], firingOffset: number): Float64Array {
    const sim = twin({
      exhaustLayout: layout,
      firingOffset,
      freeRunning: false,
      flywheelInertia: 1e6,
    });
    const out = new Float64Array(4800);
    for (let i = 0; i < out.length; i++) {
      sim.tick();
      out[i] = sim.pipeSolver.primaries[0]!.portPressure - GAS.pAmb;
    }
    return out;
  }

  /** RMS difference between two traces, as a fraction of the first one's RMS. */
  function relDiff(a: Float64Array, b: Float64Array): number {
    let d = 0;
    let r = 0;
    for (let i = 0; i < a.length; i++) {
      d += (a[i]! - b[i]!) ** 2;
      r += a[i]! ** 2;
    }
    return Math.sqrt(d / Math.max(r, 1e-30));
  }

  /**
   * The real test of coupling. Bank 0's crank schedule is identical in both runs — only
   * *bank 1's* phase moves. Any change bank 0 sees must have arrived through the junction.
   *
   * Comparing 2-into-1 against 2-into-2 directly would not show this: the merged path is also
   * longer, so its port pressure differs for reasons that have nothing to do with the other
   * cylinder. A mutant junction that couples each primary only to the collector fails here,
   * whereas a test comparing fluctuation *magnitude* between the two layouts would pass it
   * happily.
   */
  it('through a collector, bank 0 feels where bank 1 fires', () => {
    // Measures ~1.27, i.e. the change is larger than the signal itself.
    expect(relDiff(bank0Port('2into1', 360), bank0Port('2into1', 450))).toBeGreaterThan(0.3);
  });

  it('with separate pipes the exhaust path is gone, leaving only the intake', () => {
    // Measures ~7e-5, and that is the *intake* path rather than a leak.
    //
    // The intake plenum is finite, a real shared volume: move bank 1's firing and you move
    // when it draws from and spits into the manifold bank 0 breathes out of, so bank 0's
    // trapped mass changes. Engines do this — it is why a twin on one throttle body behaves
    // differently from one with two — so the coupling belongs here. What must stay true is
    // that it is small: three orders of magnitude under the 3e-1 that the collector
    // produces, and tight enough to catch a single turbulence generator shared between the
    // cylinders, which shows up around 4e-2.
    expect(relDiff(bank0Port('2into2', 360), bank0Port('2into2', 450))).toBeLessThan(1e-3);
  });

  it('but the crankshaft is a path of its own', () => {
    // Same separate pipes, ordinary flywheel: torque ripple from bank 1 pushes the shared
    // crank around, so bank 0 does feel it. A twin is never two independent singles.
    const light = (firingOffset: number) => {
      const sim = twin({ exhaustLayout: '2into2', firingOffset, freeRunning: false });
      const out = new Float64Array(4800);
      for (let i = 0; i < out.length; i++) {
        sim.tick();
        out[i] = sim.pipeSolver.primaries[0]!.portPressure - GAS.pAmb;
      }
      return out;
    };
    expect(relDiff(light(360), light(450))).toBeGreaterThan(0.01);
  });

  it('a pulse in one primary reaches the other', () => {
    const sys = new ExhaustSystem(
      twinGraph(),
      2,
      FS,
      GAS.tAmb,
      {
        heatTransfer: false,
        linearDamping: 0,
        darcyFriction: 0,
        radiate: false,
        cellSize: 0.008,
        maxCells: 512,
        maxSubsteps: 64,
      },
    );
    const shut = { throatArea: 0, cylPressure: GAS.pAmb, cylTemp: GAS.tAmb };
    const [p1, p2] = sys.primaries;
    const rho0 = GAS.pAmb / (GAS.R * GAS.tAmb);
    for (let i = 4; i < 12; i++) p1!.setPrimitive(i, rho0 * 1.3, 0, GAS.pAmb * 1.3);

    const peak = (d: typeof p1) => {
      let m = 0;
      for (let i = 0; i < d!.n; i++) m = Math.max(m, Math.abs(d!.pressureAt(i) - GAS.pAmb));
      return m;
    };
    expect(peak(p2)).toBeLessThan(1);
    for (let k = 0; k < FS * 0.004; k++) sys.advance(1 / FS, [shut, shut]);
    expect(peak(p2)).toBeGreaterThan(500);
    expect(peak(sys.collector!)).toBeGreaterThan(500);
  });

  it('nearly conserves mass and energy through the junction', () => {
    const sys = new ExhaustSystem(
      twinGraph(),
      2,
      FS,
      GAS.tAmb,
      {
        heatTransfer: false,
        linearDamping: 0,
        darcyFriction: 0,
        radiate: false,
        cellSize: 0.008,
        maxCells: 512,
        maxSubsteps: 64,
      },
    );
    const shut = { throatArea: 0, cylPressure: GAS.pAmb, cylTemp: GAS.tAmb };
    const all = [sys.primaries[0]!, sys.primaries[1]!, sys.collector!];
    for (let i = 4; i < 12; i++) all[0]!.setPrimitive(i, 1.4, 0, GAS.pAmb * 1.5);
    const mass = () => all.reduce((a, d) => a + d.totalMass(), 0);
    const energy = () => all.reduce((a, d) => a + d.totalEnergy(), 0);
    const m0 = mass();
    const e0 = energy();
    for (let k = 0; k < FS * 0.2; k++) sys.advance(1 / FS, [shut, shut]);

    // Looser than the 1e-9 a single duct manages, and deliberately so: the junction
    // linearises the wave returning into each duct, which is not exactly conservative. About
    // 0.01% over 0.2 s.
    expect(Math.abs(mass() / m0 - 1)).toBeLessThan(1e-3);
    expect(Math.abs(energy() / e0 - 1)).toBeLessThan(1e-3);
    expect(sys.recoveries).toBe(0);
  });
});

describe('robustness', () => {
  it('every engine preset runs clean', () => {
    for (const preset of ENGINE_PRESETS) {
      const cfg = defaultConfig();
      cfg.engine = { ...cfg.engine, ...preset.engine };
      cfg.pipe = preset.pipe();
      if (preset.collector) cfg.collector = preset.collector();
      const sim = new EngineSim(FS, cfg);
      sim.render(FS);
      const buf = sim.render(FS);
      let peak = 0;
      for (const v of buf) {
        expect(Number.isFinite(v), preset.name).toBe(true);
        peak = Math.max(peak, Math.abs(v));
      }
      expect(peak, `${preset.name} silent`).toBeGreaterThan(1e-3);
      expect(peak, `${preset.name} pinned`).toBeLessThan(1);
      expect(sim.pipeSolver.recoveries, preset.name).toBe(0);
      expect(sim.cylinder.clampHits, preset.name).toBe(0);
    }
  });

  it('switching layout and cylinder count mid-run stays finite', () => {
    const sim = twin();
    for (const change of [
      { exhaustLayout: '2into2' as const },
      { cylinders: 1 as const, exhaustLayout: 'single' as const },
      { cylinders: 2 as const, exhaustLayout: '2into1' as const },
      { vAngle: 90 },
      { firingOffset: 270 },
      { firingOffset: null },
    ]) {
      sim.setEngine(change);
      const buf = sim.render(FS / 4);
      for (const v of buf) expect(Number.isFinite(v)).toBe(true);
    }
  });

  it('re-phasing keeps the engine running rather than restarting it', () => {
    // Changing the firing offset must not rebuild the cylinders: that would discard their gas
    // state and audibly restart the engine every time the slider moved.
    const sim = twin();
    const before = sim.cylinders[0]!.mass;
    sim.setEngine({ firingOffset: 300 });
    expect(sim.cylinders[0]!.mass).toBe(before);
    expect(((sim.cylinders[0]!.angle - sim.cylinders[1]!.angle + 720) % 720)).toBeCloseTo(300, 0);
  });

  it('reports one snapshot entry per bank', () => {
    const sim = twin({ vAngle: 90 });
    sim.render(512);
    const snap = sim.snapshot();
    expect(snap.banks).toHaveLength(2);
    expect(((snap.banks[0]!.crankAngle - snap.banks[1]!.crankAngle + 720) % 720)).toBeCloseTo(450, 0);
    // The flat fields mirror bank 0, for the single-bank readouts.
    expect(snap.crankAngle).toBe(snap.banks[0]!.crankAngle);
    expect(snap.cylPressure).toBe(snap.banks[0]!.cylPressure);
  });
});
