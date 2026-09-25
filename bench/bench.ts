/**
 * Real-time cost of the simulation, measured the same way for every preset.
 *
 * `npm run bench`. Bundled to plain ESM first (see vite.config.mjs) so it runs under bare
 * node, which means `node --cpu-prof bench/dist/bench.mjs` profiles the same workload.
 *
 * The number that matters is the fraction of one core needed to produce a second of audio.
 * Anything at or above 1.0 cannot run at all; the audio thread also shares the machine with
 * the renderer, so the usable ceiling is well below that.
 */

import { ENGINE_PRESETS, PIPE_PRESETS, defaultConfig, type EngineSpec } from '../src/model/spec.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';

const FS = 48000;
/** Worst case on purpose: full throttle, near the rev limit, where substeps peak. */
const RPM = 6500;
const SECONDS = Number(process.env.BENCH_SECONDS ?? 3);
const ONLY = process.env.BENCH_ONLY;
/**
 * Repeats per preset, reporting the **minimum**.
 *
 * A single timed run is not reproducible enough to optimise against: repeated measurements of
 * identical code spread over about three points (59-62% on this machine), which is larger than
 * most individual improvements. The minimum is the right estimator here — the work is fixed and
 * deterministic, so every run over the fastest one is interference from elsewhere on the
 * machine, not variance in what is being measured.
 */
const REPEATS = Number(process.env.BENCH_REPEATS ?? 3);

interface Row {
  name: string;
  cells: number;
  substeps: number;
  /** Best of `REPEATS`, as a fraction of one core. */
  fraction: number;
  /** Spread between best and worst repeat, in points, so noise stays visible. */
  spread: number;
}

function measure(
  name: string,
  engine: Partial<EngineSpec>,
  pipe: ReturnType<typeof defaultConfig>['pipe'],
  collector?: ReturnType<typeof defaultConfig>['collector'],
): Row {
  const cfg = defaultConfig();
  cfg.engine = { ...cfg.engine, rpm: RPM, ...engine, throttle: 1, freeRunning: false };
  if (engine.rpm === undefined) cfg.engine.rpm = RPM;
  cfg.pipe = pipe;
  if (collector) cfg.collector = collector;

  // `BENCH_NO_KERNEL=1` forces the TypeScript hot loops, for measuring what the wasm SIMD
  // kernel is worth end to end rather than loop by loop.
  const sim = new EngineSim(FS, cfg, { useKernel: process.env.BENCH_NO_KERNEL !== '1' });
  // Warm the pipe walls and let V8 tier the hot loops up before timing anything.
  sim.render(FS / 2);

  const n = FS * SECONDS;
  const runs: number[] = [];
  for (let k = 0; k < REPEATS; k++) {
    const t0 = performance.now();
    sim.render(n);
    runs.push((performance.now() - t0) / 1000 / SECONDS);
  }

  return {
    name,
    cells: sim.pipeSolver.cells,
    substeps: sim.snapshot().substeps,
    fraction: Math.min(...runs),
    spread: Math.max(...runs) - Math.min(...runs),
  };
}

const rows: Row[] = [];
for (const p of PIPE_PRESETS) {
  if (ONLY && !p.name.includes(ONLY)) continue;
  rows.push(measure(`${p.name}, single`, { cylinders: 1, exhaustLayout: 'single' }, p.build()));
}
for (const p of ENGINE_PRESETS) {
  if (ONLY && !p.name.includes(ONLY)) continue;
  if (p.engine.cylinders === 1) continue; // already covered by the pipe presets
  rows.push(measure(p.name, p.engine, p.pipe(), p.collector?.()));
  // Also at the speed it ships at, which for a road V8 is a long way below 6500 and is what a
  // user will actually be paying for.
  if ((p.engine.rpm ?? RPM) < RPM) {
    rows.push(
      measure(`  ...at its own ${p.engine.rpm} rpm`, { ...p.engine, rpm: p.engine.rpm }, p.pipe(), p.collector?.()),
    );
  }
}

const width = Math.max(...rows.map((r) => r.name.length));
console.log(`${'preset'.padEnd(width)}  cells  substeps  % of realtime @ ${RPM} rpm  (spread)`);
for (const r of rows) {
  console.log(
    r.name.padEnd(width),
    String(r.cells).padStart(6),
    String(r.substeps).padStart(9),
    `${(r.fraction * 100).toFixed(1).padStart(10)}%`,
    `${(r.spread * 100).toFixed(1).padStart(8)}`,
  );
}
const worst = rows.reduce((a, b) => (b.fraction > a.fraction ? b : a));
console.log(`\nworst: ${worst.name} at ${(worst.fraction * 100).toFixed(1)}%`);
