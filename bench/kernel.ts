/**
 * What the wasm SIMD kernel is actually worth, measured against the TypeScript loops it
 * replaces rather than estimated from instruction counts.
 *
 *   BENCH_ENTRY=kernel.ts npm run bench
 *
 * Two loops are timed separately, because they are expected to behave differently.
 * `update` is straight-line arithmetic and should approach the 2x that two f64 lanes allow.
 * `reconstruct` carries the Riemann solve, which cannot branch per lane and so computes all
 * four of the scalar version's outcomes and selects — buying two lanes for roughly 1.7x the
 * flops. Reporting them apart is what makes that prediction checkable.
 *
 * The state is restored from a snapshot every batch. Without that the duct decays toward
 * quiescent over a few thousand unforced substeps, which quietly changes the mix of
 * supersonic and subsonic faces and therefore what is being measured. Both paths get the
 * same treatment and the same starting state.
 */

import { EulerPipe } from '../src/audio/worklet/eulerPipe.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';
import { EulerKernel, KERNEL_FIELDS, type KernelField } from '../src/audio/worklet/kernel.js';
import { ENGINE_PRESETS, GAS, defaultConfig } from '../src/model/spec.js';

const FS = 48000;
const MEAN_FLOW_RATE = 1 / 0.8;
/** Substeps per timed batch, and batches per measurement. */
const BATCH = 256;
const BATCHES = Number(process.env.KERNEL_BATCHES ?? 400);
const REPEATS = Number(process.env.KERNEL_REPEATS ?? 5);

/**
 * Self-time fractions from `node --cpu-prof` on the V-twin preset at 6500 rpm, full
 * throttle, total 56.1% of one core. Used only to project an end-to-end figure from a
 * measured loop speedup; the projection is labelled as such because it assumes the other
 * 27.6% is untouched, which is true of this change but is still an assumption.
 */
const PROFILE = { reconstruct: 0.377, update: 0.193, hllc: 0.154 };
const HOT_FRACTION = PROFILE.reconstruct + PROFILE.update + PROFILE.hllc;

type Innards = Record<KernelField, Float64Array> & {
  n: number;
  limiterCode: number;
  linearDamping: number;
  darcy: number;
  reconstructTs(dt: number): void;
  updateCellsTs(dt: number): void;
};

/** A duct warmed by a real engine, so the gas state is one the solver actually meets. */
function warmDuct(presetName: string): { pipe: EulerPipe; sim: EngineSim } {
  const preset = ENGINE_PRESETS.find((p) => p.name.includes(presetName))!;
  const cfg = defaultConfig();
  cfg.engine = { ...cfg.engine, ...preset.engine, rpm: 6500, throttle: 1, freeRunning: false };
  cfg.pipe = preset.pipe();
  if (preset.collector) cfg.collector = preset.collector();
  const sim = new EngineSim(FS, cfg);
  sim.render(FS / 2);
  return { pipe: sim.pipeSolver.primaries[0]!, sim };
}

interface Snapshot {
  rho: Float64Array;
  mom: Float64Array;
  en: Float64Array;
  uMean: Float64Array;
}

function snap(p: Innards): Snapshot {
  return {
    rho: p.rho.slice(),
    mom: p.mom.slice(),
    en: p.en.slice(),
    uMean: p.uMean.slice(),
  };
}

function restoreTs(p: Innards, s: Snapshot): void {
  p.rho.set(s.rho);
  p.mom.set(s.mom);
  p.en.set(s.en);
  p.uMean.set(s.uMean);
}

function restoreWasm(k: EulerKernel, s: Snapshot): void {
  k.fields.rho.set(s.rho);
  k.fields.mom.set(s.mom);
  k.fields.en.set(s.en);
  k.fields.uMean.set(s.uMean);
}

/** Best of `REPEATS`, in nanoseconds per call. Minimum for the reason `bench.ts` gives. */
function timed(run: () => void, calls: number): number {
  const runs: number[] = [];
  for (let r = 0; r < REPEATS; r++) {
    const t0 = process.hrtime.bigint();
    run();
    runs.push(Number(process.hrtime.bigint() - t0) / calls);
  }
  return Math.min(...runs);
}

interface Row {
  name: string;
  cells: number;
  tsNs: number;
  wasmNs: number;
}

function measure(presetName: string, label: string): Row[] {
  // The duct under test keeps its own kernel; the TS loops are called by name below, so what
  // is being compared is still one implementation against the other.
  const { pipe } = warmDuct(presetName);
  const p = pipe as unknown as Innards;
  const n = p.n;
  const dt = 1 / FS / 2;

  const kernel = EulerKernel.create(n, GAS.gammaExh, MEAN_FLOW_RATE);
  if (kernel === null) throw new Error('kernel unavailable');
  for (const f of KERNEL_FIELDS) kernel.fields[f].set(p[f], 0);

  const s = snap(p);
  const calls = BATCH * BATCHES;

  // Warm both paths so V8 has tiered up the TS loops and wasm is fully compiled.
  for (let i = 0; i < 20000; i++) {
    p.reconstructTs(dt);
    p.updateCellsTs(dt);
    kernel.reconstruct(n, dt, p.limiterCode);
    kernel.update(n, dt, p.linearDamping, p.darcy);
  }
  restoreTs(p, s);
  restoreWasm(kernel, s);

  const rows: Row[] = [];

  // reconstruct alone.
  rows.push({
    name: `${label}: reconstruct`,
    cells: n,
    tsNs: timed(() => {
      for (let b = 0; b < BATCHES; b++) {
        restoreTs(p, s);
        for (let i = 0; i < BATCH; i++) p.reconstructTs(dt);
      }
    }, calls),
    wasmNs: timed(() => {
      for (let b = 0; b < BATCHES; b++) {
        restoreWasm(kernel, s);
        for (let i = 0; i < BATCH; i++) kernel.reconstruct(n, dt, p.limiterCode);
      }
    }, calls),
  });

  // update alone. It needs fluxes present, so reconstruct once per batch before timing the
  // repeated updates — the cost of that single call is amortised over 256 and is identical
  // on both sides.
  rows.push({
    name: `${label}: update`,
    cells: n,
    tsNs: timed(() => {
      for (let b = 0; b < BATCHES; b++) {
        restoreTs(p, s);
        p.reconstructTs(dt);
        for (let i = 0; i < BATCH; i++) p.updateCellsTs(dt);
      }
    }, calls),
    wasmNs: timed(() => {
      for (let b = 0; b < BATCHES; b++) {
        restoreWasm(kernel, s);
        kernel.reconstruct(n, dt, p.limiterCode);
        for (let i = 0; i < BATCH; i++) kernel.update(n, dt, p.linearDamping, p.darcy);
      }
    }, calls),
  });

  // Both, which is what a substep actually does.
  rows.push({
    name: `${label}: substep (both)`,
    cells: n,
    tsNs: timed(() => {
      for (let b = 0; b < BATCHES; b++) {
        restoreTs(p, s);
        for (let i = 0; i < BATCH; i++) {
          p.reconstructTs(dt);
          p.updateCellsTs(dt);
        }
      }
    }, calls),
    wasmNs: timed(() => {
      for (let b = 0; b < BATCHES; b++) {
        restoreWasm(kernel, s);
        for (let i = 0; i < BATCH; i++) {
          kernel.reconstruct(n, dt, p.limiterCode);
          kernel.update(n, dt, p.linearDamping, p.darcy);
        }
      }
    }, calls),
  });

  return rows;
}

const rows = [
  ...measure('Single, megaphone', 'megaphone'),
  ...measure('2-into-1', 'V-twin primary'),
];

console.log('');
console.log('loop                            cells     TS ns    wasm ns   speedup');
for (const r of rows) {
  console.log(
    `${r.name.padEnd(30)}  ${String(r.cells).padStart(4)}  ${r.tsNs.toFixed(0).padStart(8)}  ${r.wasmNs
      .toFixed(0)
      .padStart(9)}   ${(r.tsNs / r.wasmNs).toFixed(2)}x`,
  );
}

const both = rows.filter((r) => r.name.includes('substep'));
const s = both.reduce((acc, r) => acc + r.tsNs / r.wasmNs, 0) / both.length;
const projected = 1 - HOT_FRACTION + HOT_FRACTION / s;
console.log('');
console.log(
  `mean substep speedup ${s.toFixed(2)}x over ${(HOT_FRACTION * 100).toFixed(1)}% of profiled runtime`,
);
console.log(
  `projected end to end: ${(projected * 100).toFixed(0)}% of current cost ` +
    `(${(1 / projected).toFixed(2)}x), so the V-twin's 56.1% -> ~${(56.1 * projected).toFixed(0)}%`,
);
console.log('projection assumes the other 27.6% is unchanged, which this change leaves alone.');
