/**
 * Diagnosis for the reported V8-with-a-chamber-collector blow-up.
 *
 *   BENCH_ENTRY=repro.ts npm run bench
 *
 * Established so far:
 *   - the wasm SIMD kernel is not involved; every number is identical with it on and off,
 *   - the 12,000 rpm pin happens for *every* collector including a plain pipe with zero
 *     recoveries, so it is the free-running crank against the chosen load, not a pipe fault,
 *   - the divergence is geometry-specific: a plain 55 mm pipe ahead of the chamber is clean,
 *     while a chamber (or a cone to 130 mm) at the collector inlet produces millions of
 *     recoveries.
 *
 * This run captures the duct state one sample before the first recovery, to identify the
 * mechanism. The prediction from the comment on `applyJunction` is a vacuum collapse: a
 * collector much wider than the primaries reflects a strong expansion back up the pipe that
 * is blowing down, the density floor pins that cell at 1e-7, and the returning wave divided
 * by `rhoC` becomes an absurd velocity.
 */

import type { EulerPipe } from '../src/audio/worklet/eulerPipe.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';
import { GAS, defaultConfig, makeSegment, type EngineSpec, type PipeSegment } from '../src/model/spec.js';

const FS = 48000;

const V8: Partial<EngineSpec> = {
  cylinders: 8,
  vAngle: 90,
  v8Crank: 'flatplane',
  exhaustLayout: 'perBank',
};

interface Probe {
  minRho: number;
  maxAbsU: number;
  minP: number;
  maxP: number;
  atMinRho: number;
}

function probe(d: EulerPipe): Probe {
  const rho = (d as unknown as { rho: Float64Array }).rho;
  const mom = (d as unknown as { mom: Float64Array }).mom;
  let minRho = Infinity;
  let atMinRho = -1;
  let maxAbsU = 0;
  let minP = Infinity;
  let maxP = -Infinity;
  for (let i = 0; i < d.n; i++) {
    const r = rho[i]!;
    if (r < minRho) {
      minRho = r;
      atMinRho = i;
    }
    const u = Math.abs(mom[i]! / r);
    if (u > maxAbsU) maxAbsU = u;
    const p = d.pressureAt(i);
    if (p < minP) minP = p;
    if (p > maxP) maxP = p;
  }
  return { minRho, maxAbsU, minP, maxP, atMinRho };
}

function fmt(p: Probe, n: number): string {
  return (
    `minRho=${p.minRho.toExponential(2)} @cell ${p.atMinRho}/${n}  ` +
    `maxU=${p.maxAbsU.toFixed(0).padStart(6)} m/s  ` +
    `p=[${(p.minP / 1e5).toFixed(3)}, ${(p.maxP / 1e5).toFixed(2)}] bar`
  );
}

function diagnose(name: string, collector: PipeSegment[], rpm = 4000): void {
  const cfg = defaultConfig();
  cfg.engine = {
    ...cfg.engine,
    ...V8,
    freeRunning: false,
    rpm,
    throttle: 1,
    loadTorque: 60,
  };
  cfg.pipe = [makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.042 })];
  cfg.collector = collector;

  const sim = new EngineSim(FS, cfg);
  const sys = sim.pipeSolver;
  const ducts: Array<{ label: string; d: EulerPipe }> = [
    ...sys.primaries.map((d, i) => ({ label: `primary ${i}`, d })),
    ...sys.collectors.map((d, i) => ({ label: `collector ${i}`, d })),
  ];

  console.log('');
  console.log(`--- ${name} ---`);
  for (const { label, d } of ducts.slice(0, 1)) {
    const areaFace = (d as unknown as { areaFace: Float64Array }).areaFace;
    console.log(
      `  ${label}: ${d.n} cells, dx=${(d.dx * 1000).toFixed(1)} mm, ` +
        `inlet area ${(areaFace[0]! * 1e4).toFixed(2)} cm^2`,
    );
  }
  const coll = sys.collectors[0];
  if (coll) {
    const areaFace = (coll as unknown as { areaFace: Float64Array }).areaFace;
    const a0 = areaFace[0]!;
    const aMax = Math.max(...Array.from(areaFace.slice(0, coll.n + 1)));
    const primArea = (sys.primaries[0] as unknown as { areaFace: Float64Array }).areaFace[
      sys.primaries[0]!.n
    ]!;
    const feeding = sys.primaries.length / sys.collectors.length;
    console.log(
      `  collector 0: ${coll.n} cells, inlet ${(a0 * 1e4).toFixed(2)} cm^2, ` +
        `widest ${(aMax * 1e4).toFixed(2)} cm^2; ` +
        `${feeding} primaries of ${(primArea * 1e4).toFixed(2)} cm^2 feed it ` +
        `(area ratio ${(a0 / (feeding * primArea)).toFixed(2)} at the junction)`,
    );
  }

  let prev = ducts.map(({ d }) => probe(d));
  const before = ducts.map(({ d }) => d.recoveries);
  let reported = false;

  for (let i = 0; i < FS * 2 && !reported; i++) {
    sim.tick();
    for (let k = 0; k < ducts.length; k++) {
      const { label, d } = ducts[k]!;
      if (d.recoveries > before[k]!) {
        console.log(`  first recovery at ${(i / FS).toFixed(4)} s in ${label}`);
        console.log(`    one sample before: ${fmt(prev[k]!, d.n)}`);
        reported = true;
        break;
      }
    }
    if (!reported) prev = ducts.map(({ d }) => probe(d));
  }
  if (!reported) console.log('  no recoveries in 2 s');
  const clamps = ducts.reduce((a, { d }) => a + d.junctionClamps, 0);
  const totalRec = ducts.reduce((a, { d }) => a + d.recoveries, 0);
  console.log(`  junctionClamps=${clamps}  recoveries=${totalRec}`);
  console.log(`  ambient density for reference: ${(GAS.pAmb / (GAS.R * 900)).toFixed(3)} kg/m^3 at 900 K`);
}

// Does the *inlet area* explain it, rather than the area gradient?
//
// Four 42 mm primaries feed one collector. In the case that works the collector starts at
// 55 mm; in the cases that fail it starts at 42 mm — the same width as a single primary, so
// the junction is asked to pass four pipes' worth of flow through one pipe's area. This
// sweeps the collector inlet diameter alone, chamber width and everything else held fixed,
// with the geometry guard disabled so only the junction is in play.
for (const dIn of [0.042, 0.048, 0.055, 0.065, 0.075, 0.09]) {
  diagnose(`collector inlet ${(dIn * 1000).toFixed(0)} mm -> 130 mm chamber`, [
    makeSegment({ kind: 'chamber', length: 0.34, dIn, dOut: 0.13 }),
    makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.04 }),
  ]);
}
