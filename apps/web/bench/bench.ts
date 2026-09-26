/**
 * Real-time cost of the simulation the web app ships: the Wasm build, through the same `Sim` the
 * AudioWorklet uses, in Node's V8 (the JavaScript engine in Chrome). `npm run bench`.
 *
 * For each engine preset, the fraction of one core it needs to make a second of audio, held at
 * `BENCH_RPM` (6500 by default) at full throttle, the solver's worst case. Rendered in blocks of 128,
 * as the worklet renders, after half a second of warm-up, and reported as the best of
 * `BENCH_REPEATS` (3): the work is deterministic, so anything slower than the fastest run is the rest
 * of the machine. `cargo run --release -p engine-sim --example bench` measures the native build the
 * same way.
 */

import { ENGINE_PRESETS, defaultConfig } from '../src/model/spec.js';
import { Sim } from '../src/audio/worklet/sim.js';

const FS = 48000;
const RPM = Number(process.env.BENCH_RPM ?? 6500);
const SECONDS = Number(process.env.BENCH_SECONDS ?? 3);
const REPEATS = Number(process.env.BENCH_REPEATS ?? 3);
const ONLY = process.env.BENCH_ONLY;

const width = Math.max(...ENGINE_PRESETS.map((p) => p.name.length));
console.log(`${'preset'.padEnd(width)}  % of one core @ ${RPM} rpm  (spread)`);
let worst = { name: '', fraction: 0 };
for (const p of ENGINE_PRESETS) {
  if (ONLY && !p.name.includes(ONLY)) continue;
  const cfg = defaultConfig();
  cfg.engine = { ...cfg.engine, ...p.engine, rpm: RPM, throttle: 1, freeRunning: false };
  cfg.pipe = p.pipe();
  if (p.collector) cfg.collector = p.collector();
  const sim = new Sim(FS, cfg);
  const block = new Float32Array(128);
  for (let i = 0; i < FS / 2 / 128; i++) sim.renderInto(block);
  const runs: number[] = [];
  for (let k = 0; k < REPEATS; k++) {
    const t0 = performance.now();
    for (let i = 0; i < (FS * SECONDS) / 128; i++) sim.renderInto(block);
    runs.push((performance.now() - t0) / 1000 / SECONDS);
  }
  sim.free();
  const best = Math.min(...runs);
  if (best > worst.fraction) worst = { name: p.name, fraction: best };
  console.log(
    p.name.padEnd(width),
    `${(best * 100).toFixed(1).padStart(10)}%`,
    `${((Math.max(...runs) - best) * 100).toFixed(1).padStart(8)}`,
  );
}
console.log(`\nworst: ${worst.name} at ${(worst.fraction * 100).toFixed(1)}%`);
