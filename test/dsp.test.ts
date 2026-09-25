/**
 * The structural mode filter.
 *
 * The mechanical noise rose in pitch as cylinders were added, because each "mode" was a resonant low-pass
 * that passed everything below it: the combustion drive's firing harmonics, and the valve clacks'
 * repetition rate, both of which scale with the cylinder count. A radiating surface puts out nothing at
 * DC, so a mode must be a band-pass — and must still ring, decay and peak exactly as before, since the
 * levels driving it are calibrated in pascals.
 */

import { describe, expect, it } from 'vitest';

import { Resonator } from '../src/audio/worklet/dsp.js';

const FS = 48000;

/** Steady-state gain at `hz`, measured by driving a sine through and taking the settled amplitude. */
function gainAt(r: Resonator, hz: number): number {
  r.reset();
  let peak = 0;
  const n = FS / 2;
  for (let i = 0; i < n; i++) {
    const y = r.process(Math.sin((2 * Math.PI * hz * i) / FS));
    if (i > n / 2) peak = Math.max(peak, Math.abs(y));
  }
  return peak;
}

describe('a structural mode', () => {
  it.each([[620, 9], [780, 11], [2700, 14], [4700, 20]])('passes nothing at DC: %d Hz', (hz, q) => {
    const r = new Resonator(hz, q, FS);
    let y = 0;
    for (let i = 0; i < FS; i++) y = r.process(1);
    expect(Math.abs(y)).toBeLessThan(1e-9);
  });

  /** Where the firing harmonics of a slow engine sit: well below every mode. */
  it('passes far less below its resonance than at it', () => {
    const r = new Resonator(780, 11, FS);
    const atMode = gainAt(r, 780);
    expect(gainAt(r, 100) / atMode).toBeLessThan(0.02);
    expect(gainAt(r, 200) / atMode).toBeLessThan(0.05);
  });

  it('peaks where it should, at the gain the calibration assumes', () => {
    const r = new Resonator(780, 11, FS);
    const rr = Math.exp((-Math.PI * 780) / (11 * FS));
    // 1 / (2 (1 - r)): unchanged from the all-pole form, so upstream levels mean what they did.
    expect(gainAt(r, 780)).toBeCloseTo(1 / (2 * (1 - rr)), -0.5);
    expect(gainAt(r, 780)).toBeGreaterThan(gainAt(r, 700));
    expect(gainAt(r, 780)).toBeGreaterThan(gainAt(r, 860));
  });

  it('rings to a peak of about the impulse it was struck with', () => {
    const r = new Resonator(2700, 14, FS);
    let peak = 0;
    peak = Math.max(peak, Math.abs(r.process(1)));
    for (let i = 0; i < 400; i++) peak = Math.max(peak, Math.abs(r.process(0)));
    expect(peak).toBeGreaterThan(0.9);
    expect(peak).toBeLessThan(1.1);
  });
});

/**
 * The structure-borne sounds, pitched and levelled for the engine they are in.
 *
 * Every engine used to ring at a single cylinder's frequencies and levels, so a large engine's mechanical
 * layer sat far too high and too loud; and piston slap never sounded at all, because the TDC check
 * compared each angle with itself.
 */
describe('structure-borne sound', async () => {
  const { EngineSim, clackShare } = await import('../src/audio/worklet/engineSim.js');
  const { defaultConfig } = await import('../src/model/spec.js');
  const sim = (engine: Record<string, unknown>) => {
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, ...engine };
    return new EngineSim(FS, cfg);
  };

  it('slaps its pistons, twice a cycle each', () => {
    const s = sim({ cylinders: 1, rpm: 3000, throttle: 0.8 });
    s.render(FS);
    // 3000 rpm is 25 cycles a second, and a piston crosses TDC twice a cycle.
    expect(s.slapCount).toBeGreaterThanOrEqual(48);
    expect(s.slapCount).toBeLessThanOrEqual(52);
  });

  /**
   * By the size of a cylinder, not by how many there are: the block's radiating modes are its wall panels,
   * and a panel spans a cylinder. Scaled by the whole engine, a V8's lowest mode sat on its own firing
   * harmonics and boomed 19 dB over the exhaust.
   */
  it('rings bigger cylinders lower, whatever their number', () => {
    const small = sim({ cylinders: 1 }).structuralFrequencies();
    const big = sim({ cylinders: 8, vAngle: 90, crankType: 'crossplane', bore: 0.102, stroke: 0.084 })
      .structuralFrequencies();
    const ratio = Math.cbrt((0.102 ** 2 * 0.084) / (0.089 ** 2 * 0.08));
    const eight = sim({ cylinders: 8, vAngle: 90, crankType: 'crossplane' }).structuralFrequencies();
    small.block.forEach((hz, i) => expect(eight.block[i]!).toBeCloseTo(hz, 6));
    small.block.forEach((hz, i) => expect(big.block[i]! * ratio).toBeCloseTo(hz, 0));
    // Slap is the bore ringing: a wider bore rings lower.
    const meanSlap = big.slap.reduce((a, v) => a + v, 0) / big.slap.length;
    expect(meanSlap).toBeCloseTo(small.slap[0]! * (0.089 / 0.102), -1);
  });

  it('gives each cylinder its own ring', () => {
    const { clack, slap } = sim({ cylinders: 8, vAngle: 90, crankType: 'crossplane' }).structuralFrequencies();
    expect(new Set(clack.map((hz) => hz.toFixed(1))).size).toBe(8);
    expect(new Set(slap.map((hz) => hz.toFixed(1))).size).toBe(8);
    for (const hz of clack) expect(Math.abs(hz / 2700 - 1)).toBeLessThanOrEqual(0.061);
  });

  it('retunes when the engine changes size', () => {
    const s = sim({ cylinders: 1 });
    const before = s.structuralFrequencies().block[0]!;
    s.setEngine({ bore: 0.1 });
    expect(s.structuralFrequencies().block[0]!).toBeLessThan(before);
  });

  it('makes each event quieter the more cylinders share its casting', () => {
    expect(clackShare(1)).toBe(1);
    // Power per event goes as one over the cylinders sharing: a bank of four is 6 dB down.
    expect(20 * Math.log10(clackShare(4))).toBeCloseTo(-6.02, 2);
  });
});
