/**
 * How cost scales with cell count and with cylinder count, so a bigger engine can be costed
 * before anyone builds it. Not part of `npm run bench`; run it the same way.
 */
import { defaultConfig, makeSegment, type EngineSpec } from '../src/model/spec.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';

const FS = 48000;
const SECONDS = 3;
const REPEATS = 3;

function cost(engine: Partial<EngineSpec>, primary: number, collector: number) {
  const cfg = defaultConfig();
  cfg.engine = { ...cfg.engine, rpm: 6500, throttle: 1, freeRunning: false, ...engine };
  cfg.pipe = [makeSegment({ kind: 'pipe', length: primary, dIn: 0.042 })];
  cfg.collector = [makeSegment({ kind: 'pipe', length: collector, dIn: 0.055 })];
  const sim = new EngineSim(FS, cfg);
  sim.render(FS / 2);
  const runs: number[] = [];
  for (let k = 0; k < REPEATS; k++) {
    const t0 = performance.now();
    sim.render(FS * SECONDS);
    runs.push((performance.now() - t0) / 1000 / SECONDS);
  }
  return {
    cells: sim.pipeSolver.cells,
    substeps: sim.snapshot().substeps,
    pct: Math.min(...runs) * 100,
  };
}

console.log('--- twin 2-into-1, 20 mm cells, varying duct length ---');
console.log('cells  substeps   %core');
const pts: Array<[number, number]> = [];
for (const [p, c] of [
  [0.15, 0.3],
  [0.3, 0.6],
  [0.45, 0.9],
  [0.6, 1.2],
  [0.9, 1.6],
  [1.2, 2.2],
] as Array<[number, number]>) {
  const r = cost({ cylinders: 2, exhaustLayout: '2into1', vAngle: 90 }, p, c);
  pts.push([r.cells, r.pct]);
  console.log(String(r.cells).padStart(5), String(r.substeps).padStart(9), r.pct.toFixed(1).padStart(7));
}
// Least squares on cost = a + b*cells.
const n = pts.length;
const sx = pts.reduce((a, [x]) => a + x, 0);
const sy = pts.reduce((a, [, y]) => a + y, 0);
const sxx = pts.reduce((a, [x]) => a + x * x, 0);
const sxy = pts.reduce((a, [x, y]) => a + x * y, 0);
const b = (n * sxy - sx * sy) / (n * sxx - sx * sx);
const a = (sy - b * sx) / n;
console.log(`\nfit: %core = ${a.toFixed(2)} + ${b.toFixed(4)} per cell   (2 cylinders)`);

console.log('\n--- cost of a cylinder, at matched cell count ---');
// One duct of the same total length, single vs twin: the difference is the second cylinder's
// thermodynamics plus its share of the orchestration.
const single = cost({ cylinders: 1, exhaustLayout: 'single' }, 0.45, 0.9);
const twin2 = cost({ cylinders: 2, exhaustLayout: '2into2', vAngle: 90 }, 0.45, 0.9);
console.log(`single, ${single.cells} cells: ${single.pct.toFixed(1)}%`);
console.log(`2into2, ${twin2.cells} cells: ${twin2.pct.toFixed(1)}%`);
const perCell = b;
const extraFromCells = (twin2.cells - single.cells) * perCell;
console.log(
  `second cylinder costs ${(twin2.pct - single.pct).toFixed(1)} points, of which ` +
    `${extraFromCells.toFixed(1)} is its extra ${twin2.cells - single.cells} cells ` +
    `→ ~${(twin2.pct - single.pct - extraFromCells).toFixed(1)} points of cylinder itself`,
);
