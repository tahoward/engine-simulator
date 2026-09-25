/**
 * The wasm SIMD kernel must be *bit-identical* to the TypeScript loops it stands in for.
 *
 * Not "close". Bit-identical, asserted with `Object.is` on every element of every field.
 * That is a reachable bar — JavaScript and wasm both do strictly IEEE-754 double arithmetic
 * with each operation separately rounded, and neither contracts a multiply-add into an FMA
 * (relaxed-simd, which would, is not enabled) — and it is the only bar worth setting. A
 * kernel that agreed to 1e-12 per step would still be a *different solver*: this one runs
 * 96,000 substeps a second inside a feedback loop with the cylinder, so a last-bit
 * disagreement compounds, and every measurement in the README would have to be re-taken
 * against whichever path happened to be compiled in.
 *
 * Bit-exactness also makes the two NaN-handling conventions testable. The solver clamps in
 * two different ways, and they differ on NaN: `Math.max(x, k)` propagates it, `x > k ? x : k`
 * discards it. `recoveries`, `resetToQuiescent` and the silent-audio failure they exist to
 * prevent all depend on which. See the header of `kernel/euler.ts`.
 */

import { describe, expect, it } from 'vitest';

import { EngineSim } from '../src/audio/worklet/engineSim.js';
import { EulerPipe } from '../src/audio/worklet/eulerPipe.js';
import {
  EulerKernel,
  KERNEL_FIELDS,
  kernelAvailable,
  type KernelField,
} from '../src/audio/worklet/kernel.js';
import { ENGINE_PRESETS, GAS, PIPE_PRESETS, defaultConfig } from '../src/model/spec.js';

/** Must match `MEAN_FLOW_RATE` in eulerPipe.ts. Asserted below rather than assumed. */
const MEAN_FLOW_RATE = 1 / 0.8;

/** Limiter codes, matching the `Limiter` enum in eulerPipe.ts. */
const LIMITER_CODE = { mc: 0, minmod: 1, vanleer: 2 } as const;

/** Reach into the solver's private fields. The whole point is to compare them element-wise. */
type Innards = Record<KernelField, Float64Array> & {
  n: number;
  limiterCode: number;
  lastMaxSpeed: number;
  linearDamping: number;
  darcy: number;
  reconstructTs(dt: number): void;
  updateCellsTs(dt: number): void;
};

function innards(pipe: EulerPipe): Innards {
  return pipe as unknown as Innards;
}

/** Deterministic PRNG, so a failure is reproducible from the seed alone. */
function rng(seed: number): () => number {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

/** Copy every field of the solver into the kernel's linear memory. */
function loadKernel(kernel: EulerKernel, pipe: EulerPipe): void {
  const p = innards(pipe);
  for (const name of KERNEL_FIELDS) {
    kernel.fields[name].set(p[name], 0);
  }
}

/** Index of the first element that is not bit-identical, or -1. */
function firstDiff(
  a: Float64Array | Float32Array,
  b: Float64Array | Float32Array,
  len: number,
): number {
  for (let i = 0; i < len; i++) {
    if (!Object.is(a[i], b[i])) return i;
  }
  return -1;
}

/**
 * Fields written by `reconstruct`, with how many slots of each are meaningful. Faces 0 and
 * `n` are excluded from nothing: both paths leave them untouched, so they must still match.
 */
const RECONSTRUCT_OUT: KernelField[] = [
  'pr', 'pu', 'pp',
  'sr', 'su', 'sp',
  'lr', 'lu', 'lp',
  'rr', 'ru', 'rp',
  'f0', 'f1', 'f2', 'fp',
];
const UPDATE_OUT: KernelField[] = ['rho', 'mom', 'en', 'uMean'];

function faceField(name: KernelField): boolean {
  return name === 'f0' || name === 'f1' || name === 'f2' || name === 'fp';
}

function compare(
  kernel: EulerKernel,
  pipe: EulerPipe,
  names: KernelField[],
  label: string,
): void {
  const p = innards(pipe);
  const n = p.n;
  for (const name of names) {
    const len = faceField(name) ? n + 1 : n;
    const i = firstDiff(p[name], kernel.fields[name], len);
    if (i >= 0) {
      throw new Error(
        `${label}: field "${name}" differs at ${i} of ${len}: ` +
          `ts=${p[name][i]} wasm=${kernel.fields[name][i]}`,
      );
    }
  }
}

/** One reconstruct + update on both paths, from a shared starting state. */
function stepBoth(kernel: EulerKernel, pipe: EulerPipe, dt: number): void {
  const p = innards(pipe);
  loadKernel(kernel, pipe);

  const wasmSpeed = kernel.reconstruct(p.n, dt, p.limiterCode);
  // The TypeScript implementations, named directly. `reconstruct` and `update` dispatch to
  // the kernel when the duct has one, so going through them would compare wasm to itself and
  // pass no matter what.
  p.reconstructTs(dt);
  compare(kernel, pipe, RECONSTRUCT_OUT, 'reconstruct');

  // `reconstruct` does not return the wave speed on the TS side; it stores it through the
  // same finiteness guard, which is reproduced here.
  const guarded = Number.isFinite(wasmSpeed) ? Math.max(wasmSpeed, 1) : 1e5;
  expect(guarded).toBe(p.lastMaxSpeed);

  kernel.update(p.n, dt, p.linearDamping, p.darcy);
  // The cell loop only. The scalar valve source into cell 0 stays in TypeScript on both
  // paths, so it is not part of what the kernel has to reproduce.
  p.updateCellsTs(dt);
  compare(kernel, pipe, UPDATE_OUT, 'update');
}

function makeKernel(pipe: EulerPipe): EulerKernel {
  const k = EulerKernel.create(innards(pipe).n, GAS.gammaExh, MEAN_FLOW_RATE);
  if (k === null) throw new Error('kernel unavailable');
  return k;
}

describe('wasm SIMD Euler kernel', () => {
  it('is available in this environment', () => {
    expect(kernelAvailable()).toBe(true);
  });

  it('exposes one field per array the loops touch', () => {
    expect(new Set(KERNEL_FIELDS).size).toBe(KERNEL_FIELDS.length);
    expect(KERNEL_FIELDS.length).toBe(24);
  });

  /**
   * A duct that has been driven by a real engine, which is the state that actually matters:
   * a hot slug at the port, a steepened front partway down, and mean flow everywhere.
   */
  it('matches the TypeScript loops on a running engine', () => {
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, rpm: 6500, throttle: 1, freeRunning: false };
    const sim = new EngineSim(48000, cfg);
    // Long enough for blowdown pulses to be travelling and the walls to have warmed.
    sim.render(24000);

    const pipe = sim.pipeSolver.primaries[0]!;
    const kernel = makeKernel(pipe);
    const dt = 1 / 48000 / 2;

    // Interleaved with real engine samples, so each comparison starts from a different and
    // genuinely reachable state rather than from a synthetic one.
    for (let k = 0; k < 40; k++) {
      stepBoth(kernel, pipe, dt);
      sim.render(97);
    }
  });

  /**
   * A collector, whose ends are junctions rather than a valve and a mouth. Its faces 0 and
   * `n` are written by `applyJunction` between reconstruction and the update, so this checks
   * the kernel really does leave the boundary faces alone.
   */
  it('matches on a duct with junction ends', () => {
    const preset = ENGINE_PRESETS.find((p) => p.name.includes('2-into-1'))!;
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, ...preset.engine, rpm: 6000, throttle: 1, freeRunning: false };
    cfg.pipe = preset.pipe();
    cfg.collector = preset.collector!();
    const sim = new EngineSim(48000, cfg);
    sim.render(24000);

    const collector = sim.pipeSolver.collectors[0];
    expect(collector).toBeDefined();
    const kernel = makeKernel(collector!);
    const dt = 1 / 48000 / 2;
    for (let k = 0; k < 40; k++) {
      stepBoth(kernel, collector!, dt);
      sim.render(97);
    }
  });

  /**
   * Deliberately violent states, to reach the code paths a well-behaved duct never does.
   *
   * The vectorised Riemann solver computes all four of the scalar version's branches and
   * selects, so the supersonic ones only get tested if something is actually supersonic.
   * Mach numbers out to +/-2.5 guarantee faces where `sL >= 0` and where `sR <= 0`, and the
   * pressure and density jumps drive the limiter into its zero-slope guard.
   */
  it.each(['mc', 'minmod', 'vanleer'] as const)('matches with the %s limiter', (limiter) => {
    const segments = PIPE_PRESETS[0]!.build();
    const pipe = new EulerPipe(segments, 48000, 950, { limiter });
    expect(innards(pipe).limiterCode).toBe(LIMITER_CODE[limiter]);

    const kernel = makeKernel(pipe);
    const p = innards(pipe);
    const next = rng(0xc0ffee);
    const dt = 1 / 48000 / 4;

    for (let trial = 0; trial < 12; trial++) {
      for (let i = 0; i < p.n; i++) {
        const rho = 0.1 + 4 * next();
        const u = (next() * 2 - 1) * 900;
        const pr = 2e4 + next() * 6e5;
        pipe.setPrimitive(i, rho, u, pr);
        p.uMean[i] = (next() * 2 - 1) * 120;
      }
      stepBoth(kernel, pipe, dt);
    }
  });

  /** Odd cell counts, where the vector loops hand a leftover element to their scalar tails. */
  it('matches for both parities of cell count', () => {
    const parities = new Set<number>();
    for (const len of [0.31, 0.33, 0.35, 0.37]) {
      const pipe = new EulerPipe(
        [{ kind: 'pipe', length: len, dIn: 0.042, dOut: 0.042 } as never],
        48000,
        950,
      );
      const p = innards(pipe);
      parities.add(p.n % 2);

      const kernel = makeKernel(pipe);
      const next = rng(0xbeef + p.n);
      for (let i = 0; i < p.n; i++) {
        pipe.setPrimitive(i, 0.4 + next(), (next() * 2 - 1) * 400, 8e4 + next() * 3e5);
      }
      for (let k = 0; k < 8; k++) stepBoth(kernel, pipe, 1 / 48000 / 2);
    }
    // The tails are only actually exercised if both parities showed up.
    expect(parities.has(0)).toBe(true);
    expect(parities.has(1)).toBe(true);
  });

  /** A duct longer than the compiled capacity degrades to the TS path rather than breaking. */
  it('declines a duct larger than its capacity', () => {
    expect(EulerKernel.create(10_000, GAS.gammaExh, MEAN_FLOW_RATE)).toBeNull();
  });

  /**
   * The end-to-end assertion: identical audio, sample for sample, with and without the kernel.
   *
   * Everything above compares one loop in isolation from a state that was handed to both
   * paths. This runs the whole engine — cylinders, junctions, thermal batching, radiation, the
   * crank's torque feedback — for a second and compares the output signal. It is the check
   * that the *integration* is right and not just the arithmetic: a field aliased to the wrong
   * offset, or a boundary face the kernel quietly clobbered, would show up here and nowhere
   * else.
   */
  it.each(ENGINE_PRESETS.map((p) => p.name))('renders identical audio with and without the kernel: %s', (name) => {
    const preset = ENGINE_PRESETS.find((p) => p.name === name)!;
    const build = (useKernel: boolean) => {
      const cfg = defaultConfig();
      cfg.engine = { ...cfg.engine, ...preset.engine, throttle: 1, freeRunning: false };
      cfg.pipe = preset.pipe();
      if (preset.collector) cfg.collector = preset.collector();
      // The cell kernel only: the junction kernel agrees to rounding, not bit for bit, and is tested below.
      return new EngineSim(48000, cfg, { useKernel, useJunctionKernel: false });
    };

    const withK = build(true).render(48000);
    const withoutK = build(false).render(48000);

    expect(withK.length).toBe(withoutK.length);
    const i = firstDiff(withK, withoutK, withK.length);
    if (i >= 0) {
      throw new Error(
        `${name}: audio diverges at sample ${i} (${(i / 48000).toFixed(4)} s): ` +
          `kernel=${withK[i]} ts=${withoutK[i]}`,
      );
    }
  });
});

/**
 * The junction solve in wasm renders every preset as the TypeScript does, to rounding and to the ear.
 *
 * It is the TypeScript line for line, but its `Math.pow` is AssemblyScript's, which differs from the
 * JavaScript engine's in the last bit now and then, and in a nonlinear solver a bit grows. So it is held
 * to what that allows and no more: early on, every sample within a millionth of the signal's peak, and
 * over a full second no order of the engine that you could hear moved by more than a twentieth of a dB.
 */
describe('junction kernel', async () => {
  const { EngineSim } = await import('../src/audio/worklet/engineSim.js');
  const { ENGINE_PRESETS, defaultConfig } = await import('../src/model/spec.js');
  const { hann, magnitudeSpectrum } = await import('./spectrum.js');

  it.each(ENGINE_PRESETS.filter((p) => p.collector).map((p) => p.name))('%s', (name) => {
    const preset = ENGINE_PRESETS.find((p) => p.name === name)!;
    const rpm = 4500;
    const render = (useJunctionKernel: boolean) => {
      const cfg = defaultConfig();
      cfg.engine = { ...cfg.engine, ...preset.engine, rpm, throttle: 1, freeRunning: false };
      cfg.pipe = preset.pipe();
      cfg.collector = preset.collector!();
      const sim = new EngineSim(48000, cfg, { useJunctionKernel });
      const early = Float64Array.from(sim.render(4800));
      sim.render(48000 - 4800 - 16384);
      return { early, late: Float32Array.from(sim.render(16384)) };
    };
    const ts = render(false);
    const wasm = render(true);

    let peak = 0;
    let worst = 0;
    for (let i = 0; i < ts.early.length; i++) {
      peak = Math.max(peak, Math.abs(ts.early[i]!));
      worst = Math.max(worst, Math.abs(ts.early[i]! - wasm.early[i]!));
    }
    expect(worst / peak, 'early difference, as a fraction of peak').toBeLessThan(1e-6);

    const N = 16384;
    const orders = (x: Float32Array) => {
      const mag = magnitudeSpectrum(hann(x), N);
      return Array.from({ length: 24 }, (_, k) => {
        const bin = Math.round((rpm / 60) * ((k + 1) / 2) * (N / 48000));
        let v = 0;
        for (let i = bin - 2; i <= bin + 2; i++) v = Math.max(v, mag[i]!);
        return 20 * Math.log10(v + 1e-12);
      });
    };
    const a = orders(ts.late);
    const b = orders(wasm.late);
    const top = Math.max(...a);
    a.forEach((level, k) => {
      if (level > top - 30) expect(Math.abs(level - b[k]!), `order ${(k + 1) / 2}`).toBeLessThan(0.05);
    });
  });
});
