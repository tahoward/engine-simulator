/**
 * Engines with more than two cylinders: the firing plan, the crank it implies, the collectors,
 * and the mechanism drawn from all of it.
 *
 * The V8 pair is the point of this file. A crossplane and a flatplane V8 fire at exactly the
 * same crank angles — every 90 degrees — so nothing about the *overall* firing distinguishes
 * them. What differs is which bank each firing belongs to, which means the difference only
 * becomes audible once each bank has its own collector. That is a strong claim and it is the
 * one worth nailing down.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import {
  ENGINE_PRESETS,
  GAS,
  bankFiringIntervals,
  collectorGroups,
  crankPins,
  defaultConfig,
  exhaustLayoutOf,
  firingPlan,
  makeSegment,
  pistonPosition,
  type EngineSpec,
} from '../src/model/spec.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';
import { compileCollectorLayout } from '../src/model/exhaustGraph.js';
import { EngineMesh } from '../src/scene/EngineMesh.js';
import { bandEnergy, hann, magnitudeSpectrum } from './spectrum.js';

const FS = 48000;

function spec(over: Partial<EngineSpec> = {}): EngineSpec {
  return { ...defaultConfig().engine, ...over };
}

function build(over: Partial<EngineSpec>, seconds = 1): EngineSim {
  const cfg = defaultConfig();
  cfg.engine = {
    ...cfg.engine,
    rpm: 4000,
    throttle: 1,
    freeRunning: false,
    pipeCellSize: 0.035,
    ...over,
  };
  cfg.pipe = [makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.042 })];
  cfg.collector = [makeSegment({ kind: 'pipe', length: 0.8, dIn: 0.065 })];
  /**
   * The equal-length collector system, explicitly.
   *
   * These tests are about what matched cylinders do when their pulses reach a merge evenly spaced —
   * cancellation, pulse spacing per bank — and that needs every cylinder's path to air the same length.
   * The default compiled exhaust is a manifold along each bank, whose paths differ by design.
   */
  cfg.graph = compileCollectorLayout(cfg.engine, cfg.pipe, cfg.collector);
  const sim = new EngineSim(FS, cfg);
  sim.render(FS * seconds);
  return sim;
}

const V8 = { cylinders: 8 as const, vAngle: 90, exhaustLayout: 'perBank' as const };

describe('the three, five, six and V6', () => {
  const intervals = (s: EngineSpec) => {
    const fires = [...firingPlan(s).offsets].sort((a, b) => a - b);
    return fires.map((f, i) => (i + 1 < fires.length ? fires[i + 1]! : fires[0]! + 720) - f);
  };

  it.each([
    [3, 240, [0, 120, 240]],
    [5, 144, [0, 144, 216, 288, 72]],
    [6, 120, [0, 120, 240, 240, 120, 0]],
  ] as Array<[3 | 5 | 6, number, number[]]>)('an inline %i fires every %i degrees on its crank', (n, every, pins) => {
    const s = spec({ cylinders: n, vAngle: 0 });
    expect(intervals(s)).toEqual(new Array(n).fill(every));
    expect(firingPlan(s).bankCount).toBe(1);
    expect(crankPins(s).map((p) => p.angleDeg)).toEqual(pins);
  });

  /**
   * A 60-degree V6 needs a split-pin crank to fire evenly: at that vee a shared pin gives 60-180. Each
   * throw carries one pin per bank, 60 degrees apart round the shaft.
   */
  it('a 60-degree V6 fires every 120 on three split throws', () => {
    const s = spec({ cylinders: 6, vAngle: 60 });
    expect(intervals(s)).toEqual([120, 120, 120, 120, 120, 120]);
    expect(firingPlan(s).bankCount).toBe(2);
    const pins = crankPins(s);
    expect(pins).toHaveLength(3);
    for (const pin of pins) {
      expect(pin.cylinders).toHaveLength(2);
      expect((((pin.angles[1]! - pin.angles[0]!) % 360) + 360) % 360).toBeCloseTo(60, 9);
    }
    // Each bank hears every other firing: 240 apart.
    expect(bankFiringIntervals(s, 0)).toEqual([240, 240, 240]);
  });

  it('and fires unevenly at any other vee, as that crank would', () => {
    const s = spec({ cylinders: 6, vAngle: 90 });
    expect(new Set(intervals(s)).size).toBeGreaterThan(1);
    expect(intervals(s).reduce((a, b) => a + b, 0)).toBe(720);
  });

  /**
   * A boxer: a throw per cylinder, and each opposed pair at top dead centre together, so the two pistons
   * move out and in as one. The firing orders are the Subaru's 1-3-2-4 and the Porsche's 1-6-2-4-3-5, in
   * this model's numbering by throw from the front.
   */
  it.each([
    [4, 180, [0, 2, 1, 3]],
    [6, 120, [0, 5, 2, 1, 4, 3]],
  ] as Array<[4 | 6, number, number[]]>)('a boxer %i fires every %i on a throw per cylinder', (n, every, order) => {
    const s = spec({ cylinders: n, vAngle: 180, crankType: 'boxer', exhaustLayout: 'perBank' });
    const plan = firingPlan(s);
    expect(intervals(s)).toEqual(new Array(n).fill(every));
    expect(plan.offsets.map((o, i) => [o, i]).sort((a, b) => a[0]! - b[0]!).map(([, i]) => i)).toEqual(order);
    expect(crankPins(s)).toHaveLength(n);
    for (let pair = 0; pair < n; pair += 2) {
      const [a, b] = [pair, pair + 1];
      expect(plan.banks[a]).not.toBe(plan.banks[b]);
      // Top dead centre at the same crank angle: fired a revolution apart.
      expect(Math.abs(plan.offsets[a]! - plan.offsets[b]!)).toBe(360);
    }
    expect(collectorGroups(s)).toEqual(plan.banks);
  });

  /**
   * Every cylinder gets its own place in the spread of breathing, cam timing and head ring.
   *
   * The spread's shuffle only permutes when its step shares no factor with the cylinder count, and the
   * steps are 3, 5 and 7 — so taken as they are, a five, or a three or six, would give every cylinder the
   * same place.
   */
  it('spreads every cylinder count', async () => {
    const { spreadOf } = await import('../src/audio/worklet/engineSim.js');
    for (const n of [2, 3, 4, 5, 6, 8]) {
      for (const [step, offset] of [[5, 2], [3, 1], [7, 3], [3, 2]] as Array<[number, number]>) {
        const places = Array.from({ length: n }, (_, b) => spreadOf(b, n, step, offset));
        expect(new Set(places.map((v) => v.toFixed(9))).size, `${n} cylinders, step ${step}`).toBe(n);
      }
    }
  });
});

describe('firing plans', () => {
  it('an inline four fires 1-3-4-2, evenly every 180 degrees on one bank', () => {
    const s = spec({ cylinders: 4 });
    const plan = firingPlan(s);
    expect(plan.offsets).toEqual([0, 540, 180, 360]);
    expect(plan.bankCount).toBe(1);
    expect(bankFiringIntervals(s, 0)).toEqual([180, 180, 180, 180]);
  });

  it('both V8 cranks fire every 90 degrees — overall they are indistinguishable', () => {
    for (const crankType of ['crossplane', 'flatplane'] as const) {
      const plan = firingPlan(spec({ ...V8, crankType }));
      expect(plan.offsets, crankType).toEqual([0, 90, 180, 270, 360, 450, 540, 630]);
    }
  });

  it('but they deal those firings out to the banks quite differently', () => {
    // This is the whole difference between the two engines.
    const cross = spec({ ...V8, crankType: 'crossplane' });
    expect(bankFiringIntervals(cross, 0)).toEqual([180, 90, 180, 270]);
    expect(bankFiringIntervals(cross, 1)).toEqual([270, 180, 90, 180]);

    const flat = spec({ ...V8, crankType: 'flatplane' });
    expect(bankFiringIntervals(flat, 0)).toEqual([180, 180, 180, 180]);
    expect(bankFiringIntervals(flat, 1)).toEqual([180, 180, 180, 180]);
  });

  it('every cylinder fires exactly once per cycle, at a distinct angle', () => {
    for (const over of [
      { cylinders: 1 as const },
      { cylinders: 2 as const },
      { cylinders: 4 as const },
      { ...V8, crankType: 'crossplane' as const },
      { ...V8, crankType: 'flatplane' as const },
    ]) {
      const plan = firingPlan(spec(over));
      expect(plan.offsets).toHaveLength(spec(over).cylinders);
      expect(new Set(plan.offsets).size).toBe(plan.offsets.length);
      for (const o of plan.offsets) {
        expect(o).toBeGreaterThanOrEqual(0);
        expect(o).toBeLessThan(720);
      }
    }
  });
});

describe('the crank the plan implies', () => {
  /**
   * `crankPins` is derived, not stored, so it is a genuine prediction: the pin *angles* it
   * recovers should be the ones the crank is named after. They are, which is a pleasing check
   * on the firing data — a crossplane crank really does come out with its pins at 90-degree
   * intervals, and a flatplane one with all four in a single plane.
   */
  it('recovers a crossplane crank from the crossplane firing plan', () => {
    const pins = crankPins(spec({ ...V8, crankType: 'crossplane' }));
    expect(pins).toHaveLength(4);
    expect([...pins.map((p) => p.angleDeg)].sort((a, b) => a - b)).toEqual([0, 90, 180, 270]);
    for (const p of pins) expect(p.cylinders).toHaveLength(2);
  });

  it('and a flatplane crank from the flatplane one', () => {
    const pins = crankPins(spec({ ...V8, crankType: 'flatplane' }));
    expect(pins).toHaveLength(4);
    // All in one plane: only 0 and 180.
    expect(new Set(pins.map((p) => p.angleDeg))).toEqual(new Set([0, 180]));
    for (const p of pins) expect(p.cylinders).toHaveLength(2);
  });

  it('gives a V-twin one shared pin and an inline four a pin each', () => {
    expect(crankPins(spec({ cylinders: 2, vAngle: 45, firingOffset: null }))).toHaveLength(1);
    expect(crankPins(spec({ cylinders: 4 }))).toHaveLength(4);
  });

  it('refuses to pair cylinders no shared pin could carry', () => {
    // 270/450 is exactly what the firing-offset override exists for: a crank with two pins.
    const pins = crankPins(spec({ cylinders: 2, vAngle: 45, firingOffset: 270 }));
    expect(pins).toHaveLength(2);
    for (const p of pins) expect(p.cylinders).toHaveLength(1);
  });

  it('accounts for every cylinder exactly once, whatever the layout', () => {
    for (const over of [
      { cylinders: 1 as const },
      { cylinders: 2 as const, firingOffset: 270 },
      { cylinders: 4 as const },
      { ...V8, crankType: 'crossplane' as const },
      { ...V8, crankType: 'flatplane' as const },
    ]) {
      const s = spec(over);
      const seen = crankPins(s).flatMap((p) => p.cylinders);
      expect(seen.sort((a, b) => a - b)).toEqual([...Array(s.cylinders).keys()]);
    }
  });
});

describe('plumbing', () => {
  it('groups cylinders by bank, all together, or not at all', () => {
    expect(collectorGroups(spec({ ...V8, exhaustLayout: 'perBank', crankType: 'flatplane' })))
      .toEqual([0, 1, 0, 1, 0, 1, 0, 1]);
    expect(collectorGroups(spec({ ...V8, exhaustLayout: 'merged' })))
      .toEqual([0, 0, 0, 0, 0, 0, 0, 0]);
    expect(collectorGroups(spec({ ...V8, exhaustLayout: 'open' })))
      .toEqual([-1, -1, -1, -1, -1, -1, -1, -1]);
  });

  it('builds the ducts the grouping asks for', () => {
    const perBank = build({ ...V8, exhaustLayout: 'perBank' });
    expect(perBank.pipeSolver.primaries).toHaveLength(8);
    expect(perBank.pipeSolver.collectors).toHaveLength(2);

    const merged = build({ ...V8, exhaustLayout: 'merged' });
    expect(merged.pipeSolver.collectors).toHaveLength(1);

    const open = build({ ...V8, exhaustLayout: 'open' });
    expect(open.pipeSolver.collectors).toHaveLength(0);
  });

  it('still honours the old twin-only layout names', () => {
    // Saved links predate `open`/`merged`.
    expect(exhaustLayoutOf(spec({ cylinders: 2, exhaustLayout: '2into1' }))).toBe('merged');
    expect(exhaustLayoutOf(spec({ cylinders: 2, exhaustLayout: '2into2' }))).toBe('open');
    expect(exhaustLayoutOf(spec({ cylinders: 1, exhaustLayout: 'single' }))).toBe('open');
  });

  it('collapses per-bank to merged when there is only one bank', () => {
    expect(exhaustLayoutOf(spec({ cylinders: 4, exhaustLayout: 'perBank' }))).toBe('merged');
  });
});

describe('a V8 runs', () => {
  it('all eight cylinders fire, at the planned crank angles', () => {
    const sim = build({ ...V8, crankType: 'crossplane' }, 1);
    const plan = firingPlan(sim.engine);
    const peak = new Array(8).fill(0);
    const at = new Array(8).fill(0);
    // Two full cycles at 4000 rpm is 0.06 s; take a quarter second to be safe.
    for (let i = 0; i < FS / 4; i++) {
      sim.tick();
      const ref = sim.cylinders[0]!.angle;
      for (let b = 0; b < 8; b++) {
        const p = sim.cylinders[b]!.pressure(sim.engine);
        if (p > peak[b]) {
          peak[b] = p;
          at[b] = ref;
        }
      }
    }
    for (const p of peak) expect(p).toBeGreaterThan(20e5);
    // Each cylinder's peak must land where the plan says it fires. Seeding the crank with
    // `+offset` instead of `-offset` runs the order backwards and fails this.
    for (let b = 0; b < 8; b++) {
      const rel = (((at[b] - at[0]) % 720) + 720) % 720;
      expect(Math.abs(rel - plan.offsets[b]!), `cylinder ${b + 1}`).toBeLessThan(6);
    }
  });

  it('stays finite and admissible in every layout', () => {
    for (const layout of ['open', 'perBank', 'merged'] as const) {
      const sim = build({ ...V8, exhaustLayout: layout });
      const buf = sim.render(FS / 2);
      let peak = 0;
      for (const v of buf) {
        expect(Number.isFinite(v), layout).toBe(true);
        peak = Math.max(peak, Math.abs(v));
      }
      expect(peak, layout).toBeGreaterThan(1e-3);
      expect(peak, layout).toBeLessThan(1);
      expect(sim.pipeSolver.recoveries, layout).toBe(0);
      for (const c of sim.cylinders) expect(c.clampHits, layout).toBe(0);
    }
  });

  it('runs at one substep, pinned, with no bursts', () => {
    // 35 mm cells is what makes a V8 affordable: the CFL limit then clears an audio sample
    // outright, so the count is pinned at one instead of two. See DESIGN_WAVE_SPEED.
    const sim = build({ ...V8 });
    for (const d of [...sim.pipeSolver.primaries, ...sim.pipeSolver.collectors]) {
      expect(d.substeps).toBe(1);
    }
    sim.render(FS);
    let bursts = 0;
    for (const d of [...sim.pipeSolver.primaries, ...sim.pipeSolver.collectors]) {
      bursts += d.substepBursts;
    }
    expect(bursts).toBe(0);
  });

  it('fires eight times per cycle, so its firing frequency is four times a single', () => {
    const sim = build({ ...V8, rpm: 3000, exhaustLayout: 'merged' }, 2);
    const N = 65536;
    const mag = magnitudeSpectrum(hann(sim.render(N)), N);
    const half = 3000 / 120; // 25 Hz, one firing per cylinder per cycle
    // The eighth order — eight firings per 720 degrees — must dominate the half order.
    const eighth = bandEnergy(mag, FS, N, half * 8, 4);
    expect(eighth).toBeGreaterThan(bandEnergy(mag, FS, N, half, 4) * 10);
  });
});

describe('crossplane against flatplane', () => {
  /**
   * The central claim: the two cranks differ only in bank pattern, so they must differ *at the
   * bank collectors* and be near-identical through one shared collector.
   *
   * Measured at each bank's own collector mouth, using the pressure fluctuation the collector
   * sees. An uneven arrival pattern (270-180-90-180) and an even one (180 x 4) cannot produce
   * the same spectrum, and the giveaway is the half order: four evenly spaced pulses per bank
   * put all their energy at multiples of the fourth order, where an uneven pattern spills energy
   * into the lower orders that only repeat once per cycle.
   */
  function bankOrderEnergy(crankType: EngineSpec['crankType'], order: number): number {
    const rpm = 3000;
    const sim = build({ ...V8, crankType, exhaustLayout: 'perBank', rpm }, 2);
    const N = 32768;
    const out = new Float64Array(N);
    // Pressure at bank 0's collector inlet: what that bank's plumbing actually hears.
    for (let i = 0; i < N; i++) {
      sim.tick();
      out[i] = sim.pipeSolver.collectors[0]!.pressureAt(0) - GAS.pAmb;
    }
    const f32 = new Float32Array(out);
    const mag = magnitudeSpectrum(hann(f32), N);
    return bandEnergy(mag, FS, N, (rpm / 120) * order, 4);
  }

  it('a flatplane bank hears four evenly spaced pulses, a crossplane bank does not', () => {
    // Order 2 repeats twice per cycle: an evenly firing bank (every 180 crank degrees = order
    // 4) should put very little there, while the uneven crossplane pattern must.
    const crossSecond = bankOrderEnergy('crossplane', 2);
    const flatSecond = bankOrderEnergy('flatplane', 2);
    expect(crossSecond).toBeGreaterThan(flatSecond * 5);
  });

  it('a flatplane bank concentrates on the fourth order where a crossplane bank spreads', () => {
    // Four evenly spaced firings per bank put their energy at multiples of the fourth order and
    // very little between; an uneven pattern repeats only once per cycle, so it fills in the
    // lower orders too. Measured as the fourth order against the third, which is the gap the
    // even pattern should not excite. Note this is *not* a claim that the fourth order dominates
    // the crossplane spectrum — measured, its third order is larger, which is the whole point.
    const flat = bankOrderEnergy('flatplane', 4) / bankOrderEnergy('flatplane', 3);
    const cross = bankOrderEnergy('crossplane', 4) / bankOrderEnergy('crossplane', 3);
    expect(flat).toBeGreaterThan(cross * 20);
  });
});

describe('the drawn mechanism', () => {
  const clip = new THREE.Plane(new THREE.Vector3(0, 0, -1), 0.001);

  /**
   * Pose the mesh as the app does.
   *
   * Mind the sign on the offset. A cylinder that fires *later* is *behind* on the crank, so its
   * own angle is `theta0 - offset`, which is what the simulation hands the renderer. Getting it
   * backwards is invisible for a single (offset 0) and for an inline four (offsets differ by
   * multiples of 360, and the slider-crank has period 360), so only a V or an odd-fire layout
   * shows it up — which is exactly what the rod-length assertion below catches.
   */
  function poseAll(mesh: EngineMesh, s: EngineSpec, crankAngle: number): void {
    const plan = firingPlan(s);
    const banks = plan.offsets.map((o) => ({
      crankAngle: (((crankAngle - o) % 720) + 720) % 720,
      cylPressure: GAS.pAmb,
      cylTemp: GAS.tAmb,
      exLift: 0,
      inLift: 0,
    }));
    mesh.update(banks, new Array(banks.length).fill(0));
  }

  /**
   * The invariant worth having. A connecting rod is rigid, so however the pin angle, the bank
   * rotation and the Z offset combine, the drawn rod must come out at exactly `rodLength`. Get
   * any of them wrong and it silently stretches to reach instead of looking broken.
   */
  it('draws every rod at exactly its real length, at every crank angle', () => {
    for (const over of [
      { cylinders: 1 as const },
      { cylinders: 2 as const, vAngle: 45, firingOffset: null },
      { cylinders: 2 as const, vAngle: 45, firingOffset: 270 },
      { cylinders: 3 as const, vAngle: 0 },
      { cylinders: 4 as const, vAngle: 0 },
      { cylinders: 5 as const, vAngle: 0 },
      { cylinders: 6 as const, vAngle: 0 },
      // A split-pin crank: each cylinder of a throw on its own pin.
      { cylinders: 6 as const, vAngle: 60, exhaustLayout: 'perBank' as const },
      { ...V8, crankType: 'crossplane' as const },
      { ...V8, crankType: 'flatplane' as const },
    ]) {
      const s = spec(over);
      const mesh = new EngineMesh(s, clip);
      for (let a = 0; a < 720; a += 11) {
        poseAll(mesh, s, a);
        for (let i = 0; i < s.cylinders; i++) {
          expect(mesh.pose(i).rodLength, `${s.cylinders}cyl #${i} at ${a}deg`).toBeCloseTo(
            s.rodLength,
            9,
          );
        }
      }
    }
  });

  it('puts each piston where the physics says it is', () => {
    const s = spec({ ...V8, crankType: 'crossplane' });
    const mesh = new EngineMesh(s, clip);
    const plan = firingPlan(s);
    poseAll(mesh, s, 137);
    for (let i = 0; i < 8; i++) {
      const own = (((137 - plan.offsets[i]!) % 720) + 720) % 720;
      expect(mesh.pose(i).pistonY).toBeCloseTo(pistonPosition(s, own), 12);
    }
  });

  /**
   * The same invariant, driven by the simulation instead of by a hand-built snapshot — so it
   * also checks that the convention the physics publishes is the one the renderer expects. A
   * synthetic snapshot can agree with a wrong renderer; the running engine cannot.
   */
  it('draws the running engine without stretching a rod', () => {
    for (const over of [
      { cylinders: 2 as const, vAngle: 45, firingOffset: null },
      { ...V8, crankType: 'crossplane' as const },
    ]) {
      const sim = build(over, 1);
      const s = sim.engine;
      const mesh = new EngineMesh(s, clip);
      for (let k = 0; k < 40; k++) {
        sim.render(137);
        const snap = sim.snapshot();
        mesh.update(snap.banks, new Array(s.cylinders).fill(0));
        for (let i = 0; i < s.cylinders; i++) {
          expect(mesh.pose(i).rodLength, `${s.cylinders}cyl #${i}`).toBeCloseTo(s.rodLength, 9);
        }
      }
    }
  });

  it('stands cylinders sharing a crankpin in the same plane', () => {
    const s = spec({ ...V8, crankType: 'crossplane' });
    const mesh = new EngineMesh(s, clip);
    for (const pin of crankPins(s)) {
      const zs = pin.cylinders.map((c) => mesh.pose(c).z);
      for (const z of zs) expect(z).toBeCloseTo(zs[0]!, 12);
    }
    // And four distinct planes for four pins.
    expect(new Set([...Array(8).keys()].map((i) => mesh.pose(i).z.toFixed(6))).size).toBe(4);
  });

  it('opens two banks and puts one cylinder per bank on each pin', () => {
    const s = spec({ ...V8, crankType: 'crossplane' });
    const mesh = new EngineMesh(s, clip);
    const rots = new Set([...Array(8).keys()].map((i) => mesh.pose(i).bankRotation.toFixed(6)));
    expect(rots.size).toBe(2);
    for (const pin of crankPins(s)) {
      const banks = pin.cylinders.map((c) => mesh.pose(c).bankRotation);
      expect(banks[0]).not.toBeCloseTo(banks[1]!, 6);
    }
  });

  it('gives every cylinder its own exhaust port', () => {
    const s = spec({ ...V8 });
    const mesh = new EngineMesh(s, clip);
    const seen = new Set<string>();
    for (let i = 0; i < 8; i++) {
      const p = mesh.exhaustPort(i);
      expect(Number.isFinite(p.position.x + p.position.y + p.position.z)).toBe(true);
      seen.add(`${p.position.x.toFixed(4)},${p.position.y.toFixed(4)},${p.position.z.toFixed(4)}`);
    }
    expect(seen.size).toBe(8);
  });

  /**
   * Each bank's exhaust comes out of the outside of the vee.
   *
   * One side for every cylinder would put bank 0's exhaust into the valley and bank 1's outside, so both
   * banks' pipes would leave the same side of the engine. Mirrored, the ports sit either side of the crank
   * and face away from each other.
   */
  it.each([
    ['a V8', { ...V8 }],
    ['a V-twin', { cylinders: 2, vAngle: 45 }],
  ] as Array<[string, Partial<EngineSpec>]>)('puts the exhaust on the outside of each bank of %s', (_n, engine) => {
    const s = spec(engine);
    const mesh = new EngineMesh(s, clip);
    const plan = firingPlan(s);
    for (let i = 0; i < mesh.bankCount; i++) {
      const port = mesh.exhaustPort(i);
      const outward = plan.banks[i] === 0 ? -1 : 1;
      expect(Math.sign(port.position.x), `cylinder ${i} port`).toBe(outward);
      expect(Math.sign(port.direction.x), `cylinder ${i} heading`).toBe(outward);
    }
  });

  /**
   * A valve opens straight down its own stem.
   *
   * With the sideways part of the opening motion the wrong way round, a valve leaning out at the top
   * would slide outward as it dropped — crabbing across its guide instead of travelling along it.
   */
  it.each([
    ['a single', { cylinders: 1 }],
    ['a V8', { ...V8 }],
  ] as Array<[string, Partial<EngineSpec>]>)('opens every valve of %s along its stem', (_n, engine) => {
    const mesh = new EngineMesh(spec(engine), clip) as unknown as {
      cyls: Array<{ exValve: THREE.Group; inValve: THREE.Group; exhaustSide: number }>;
      poseValve: (g: THREE.Group, sign: number, lift: number) => void;
    };
    const lift = 0.009;
    for (const c of mesh.cyls) {
      for (const [valve, sign] of [[c.exValve, c.exhaustSide], [c.inValve, -c.exhaustSide]] as const) {
        mesh.poseValve(valve, sign, 0);
        const shut = valve.position.clone();
        mesh.poseValve(valve, sign, lift);
        const moved = valve.position.clone().sub(shut);
        const stem = new THREE.Vector3(0, 1, 0).applyAxisAngle(new THREE.Vector3(0, 0, 1), valve.rotation.z);
        expect(moved.length()).toBeCloseTo(lift, 9);
        // Down the stem, away from the seat: exactly opposite the stem's upward axis.
        expect(moved.clone().normalize().dot(stem)).toBeCloseTo(-1, 9);
      }
    }
  });

  it('keeps an inline engine on one side', () => {
    const mesh = new EngineMesh(spec({ cylinders: 4, vAngle: 0 }), clip);
    for (let i = 0; i < mesh.bankCount; i++) expect(mesh.exhaustPort(i).direction.x).toBeGreaterThan(0);
  });

  it('rebuilds when the cylinder count changes', () => {
    const mesh = new EngineMesh(spec({ cylinders: 1 }), clip);
    expect(mesh.bankCount).toBe(1);
    mesh.setSpec(spec({ ...V8 }));
    expect(mesh.bankCount).toBe(8);
    mesh.setSpec(spec({ cylinders: 2, vAngle: 45 }));
    expect(mesh.bankCount).toBe(2);
  });
});

describe('every preset', () => {
  it('runs clean and audible', () => {
    for (const preset of ENGINE_PRESETS) {
      const cfg = defaultConfig();
      cfg.engine = { ...cfg.engine, ...preset.engine, throttle: 1 };
      cfg.pipe = preset.pipe();
      if (preset.collector) cfg.collector = preset.collector();
      const sim = new EngineSim(FS, cfg);
      sim.render(FS);
      const buf = sim.render(FS / 2);
      let peak = 0;
      for (const v of buf) {
        expect(Number.isFinite(v), preset.name).toBe(true);
        peak = Math.max(peak, Math.abs(v));
      }
      expect(peak, `${preset.name} silent`).toBeGreaterThan(1e-3);
      expect(peak, `${preset.name} pinned`).toBeLessThan(1);
      expect(sim.pipeSolver.recoveries, preset.name).toBe(0);
      for (const c of sim.cylinders) expect(c.clampHits, preset.name).toBe(0);
    }
  });

  it('draws for every preset without stretching a rod', () => {
    const clip = new THREE.Plane(new THREE.Vector3(0, 0, -1), 0.001);
    for (const preset of ENGINE_PRESETS) {
      const s = spec(preset.engine);
      const mesh = new EngineMesh(s, clip);
      const plan = firingPlan(s);
      for (let a = 0; a < 720; a += 37) {
        mesh.update(
          plan.offsets.map((o) => ({
            crankAngle: (((a - o) % 720) + 720) % 720,
            cylPressure: GAS.pAmb,
            cylTemp: GAS.tAmb,
            exLift: 0,
            inLift: 0,
          })),
          new Array(s.cylinders).fill(0),
        );
        for (let i = 0; i < s.cylinders; i++) {
          expect(mesh.pose(i).rodLength, preset.name).toBeCloseTo(s.rodLength, 9);
        }
      }
    }
  });
});

describe('why a multi-cylinder engine does not just go up in pitch', () => {
  /**
   * Two things keep an engine with several tailpipes sounding like one.
   *
   * Both are about cancellation. Evenly spaced firing cancels every order that is not a multiple
   * of the cylinder count, which is correct and is why a multi sounds smooth. But a model can make
   * *two further* cancellations perfect when reality does not: separate mouths summed at a single
   * point, and cylinders that breathe identically.
   */

  function render(over: Partial<EngineSpec>, seconds = 1) {
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, throttle: 1, freeRunning: false, pipeCellSize: 0.035, ...over };
    cfg.pipe = [makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.042 })];
    cfg.collector = [makeSegment({ kind: 'pipe', length: 0.8, dIn: 0.065 })];
    // Equal-length runners into collectors: cancellation is a property of evenly spaced merging pulses.
    cfg.graph = compileCollectorLayout(cfg.engine, cfg.pipe, cfg.collector);
    const sim = new EngineSim(FS, cfg);
    sim.render(FS * seconds);
    const N = 32768;
    return { mag: magnitudeSpectrum(hann(sim.render(N)), N), N, rpm: cfg.engine.rpm };
  }

  /**
   * A flatplane V8's two banks fire in exact antiphase, so summing their mouths at one point
   * annihilates the loudest thing in the spectrum — each bank's own firing order — and the engine
   * jumps an octave to the doubled order.
   *
   * Real tailpipes are a metre or so apart, which at 187 Hz is most of a wavelength.
   */
  it('separate mouths must not annihilate the bank firing order', () => {
    const rpm = 5600;
    const bankOrderHz = (rpm / 120) * 4; // four firings per bank per cycle
    const base = { cylinders: 8 as const, vAngle: 90, crankType: 'flatplane' as const,
      exhaustLayout: 'perBank' as const, rpm };

    const coincident = render({ ...base, mouthSpacing: 0 });
    const spread = render({ ...base, mouthSpacing: 1.3 });
    const at = (r: ReturnType<typeof render>) => bandEnergy(r.mag, FS, r.N, bankOrderHz, 4);
    // Measures about 85x on this geometry (19 dB) and four orders of magnitude on the shipped
    // preset, the difference being where the bank order falls relative to the pipe's resonances.
    expect(at(spread)).toBeGreaterThan(at(coincident) * 20);
  });

  it('and the spacing has to be off the mouths’ own axis to do anything', () => {
    // Mouths placed symmetrically about the listener's axis are all the *same* distance away, so
    // the path differences are zero and the sum is as coherent as if they were coincident: the
    // output does not change at all, bit for bit. The listener therefore stands off to one side.
    // This test pins the consequence: sweeping the spacing must actually change the output.
    const base = { cylinders: 8 as const, vAngle: 90, crankType: 'flatplane' as const,
      exhaustLayout: 'perBank' as const, rpm: 5600 };
    const bankOrderHz = (5600 / 120) * 4;
    const at = (r: ReturnType<typeof render>) => bandEnergy(r.mag, FS, r.N, bankOrderHz, 6);
    const near = at(render({ ...base, mouthSpacing: 0.3 }));
    const far = at(render({ ...base, mouthSpacing: 1.3 }));
    // Measured at the bank order rather than broadband, because that is where the path
    // differences do their work; a whole-spectrum metric also moves when anything else changes,
    // and with merge noise in the mix it sits marginally either side of any sensible threshold.
    expect(Math.max(near, far) / Math.min(near, far)).toBeGreaterThan(3);
  });

  it('one mouth is unaffected by spacing, as it must be', () => {
    const base = { cylinders: 4 as const, exhaustLayout: 'merged' as const, rpm: 3400 };
    const a = render({ ...base, mouthSpacing: 0 });
    const b = render({ ...base, mouthSpacing: 2 });
    for (let i = 0; i < a.mag.length; i++) expect(b.mag[i]).toBeCloseTo(a.mag[i]!, 12);
  });

  /**
   * With identical cylinders the cancellation of the non-multiple orders is not merely strong but
   * *exact* — measured 70 dB down, where real engines sit 20 to 35 dB down, because no two
   * cylinders breathe alike. Without this an inline four is a pure tone on its firing frequency
   * with no rumble underneath, which is the other half of sounding wrong.
   */
  it('unequal cylinder breathing restores the low orders', () => {
    const rpm = 3400;
    const half = rpm / 120;
    const base = { cylinders: 4 as const, exhaustLayout: 'merged' as const, rpm };
    const matched = render({ ...base, cylinderSpread: 0 }, 2);
    const real = render({ ...base, cylinderSpread: 1 }, 2);

    const ratio = (r: ReturnType<typeof render>, order: number) =>
      bandEnergy(r.mag, FS, r.N, half * order, 3) / bandEnergy(r.mag, FS, r.N, half * 4, 3);

    // The third order — 1.5 times per revolution — is the strongest of the cancelled ones.
    expect(ratio(real, 3)).toBeGreaterThan(ratio(matched, 3) * 10);
    // And it must land in the range real engines occupy, not merely rise: -20 to -35 dB.
    const dB = 10 * Math.log10(ratio(real, 3));
    expect(dB).toBeGreaterThan(-40);
    expect(dB).toBeLessThan(-12);
  });

  it('the spread is deterministic, so an engine sounds the same each time it starts', () => {
    const base = { cylinders: 8 as const, vAngle: 90, exhaustLayout: 'perBank' as const, rpm: 3200 };
    const a = render(base);
    const b = render(base);
    for (let i = 0; i < a.mag.length; i++) expect(b.mag[i]).toBe(a.mag[i]);
  });
});
