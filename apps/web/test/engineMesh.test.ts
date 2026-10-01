/**
 * The mechanism drawn from the engine: the firing plan, the crank it implies and where the ports are,
 * posed from the angles the simulation reports.
 *
 * The drawing and the physics read the same geometry (`src/model/spec.ts`), and these check the two
 * cannot come apart: a rod drawn from a pin placed wrongly would silently stretch to reach its piston
 * rather than look broken.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import {
  ENGINE_PRESETS,
  GAS,
  crankPins,
  mainBearingsAfter,
  ROD_STAGGER,
  defaultConfig,
  firingPlan,
  makeSegment,
  pistonPosition,
  type EngineSpec,
} from '../src/model/spec.js';
import { Sim } from '../src/audio/worklet/sim.js';
import { compileCollectorLayout } from '../src/model/exhaustGraph.js';
import { EngineMesh } from '../src/scene/EngineMesh.js';

const FS = 48000;

function spec(over: Partial<EngineSpec> = {}): EngineSpec {
  return { ...defaultConfig().engine, ...over };
}

/** A running engine on an equal-length collector system, and the spec it runs. */
function build(over: Partial<EngineSpec>, seconds = 1): { sim: Sim; spec: EngineSpec } {
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
  cfg.graph = compileCollectorLayout(cfg.engine, cfg.pipe, cfg.collector);
  const sim = new Sim(FS, cfg);
  sim.render(FS * seconds);
  return { sim, spec: cfg.engine };
}

const V8 = { cylinders: 8 as const, vAngle: 90, exhaustLayout: 'perBank' as const };

describe('the drawn mechanism', () => {
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
      const mesh = new EngineMesh(s);
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

  it('draws each crank web flat round the shaft, its counterweight opposite its pin', () => {
    for (const over of [
      { cylinders: 4 as const, vAngle: 0 },
      { cylinders: 3 as const, vAngle: 0 },
      { cylinders: 6 as const, vAngle: 60, exhaustLayout: 'perBank' as const },
      { ...V8, crankType: 'crossplane' as const },
      { cylinders: 4 as const, vAngle: 180, crankType: 'boxer' as const },
    ]) {
      const s = spec(over);
      const mesh = new EngineMesh(s);
      mesh.group.updateMatrixWorld(true);
      const webs: THREE.Mesh[] = [];
      mesh.group.traverse((o) => {
        if (o instanceof THREE.Mesh && o.name === 'web') webs.push(o);
      });
      // A web either side of every main but the end ones' outer sides.
      const mains = mainBearingsAfter(s).filter(Boolean).length + 2;
      expect(webs).toHaveLength(2 * mains - 2);
      const a = s.stroke / 2;
      for (const web of webs) {
        const phi = ((web.userData as { angle: number }).angle * Math.PI) / 180;
        // Where its pin is, in the crank's own frame.
        const pin = new THREE.Vector3(-Math.sin(phi), Math.cos(phi), 0);
        const pos = web.geometry.getAttribute('position');
        const toCrank = web.matrix;
        let zMin = Infinity;
        let zMax = -Infinity;
        let towards = -Infinity;
        let away = -Infinity;
        for (let i = 0; i < pos.count; i++) {
          const v = new THREE.Vector3().fromBufferAttribute(pos, i).applyMatrix4(toCrank);
          zMin = Math.min(zMin, v.z);
          zMax = Math.max(zMax, v.z);
          const along = v.x * pin.x + v.y * pin.y;
          towards = Math.max(towards, along);
          away = Math.max(away, -along);
          // Square across the pin's direction, it is no wider than its counterweight.
          expect(Math.abs(v.x * pin.y - v.y * pin.x)).toBeLessThan(1.5 * a + 1e-9);
        }
        // Flat: only as thick as a web, not tipped out of the plane the crank turns in.
        expect(zMax - zMin).toBeLessThan(0.012);
        // Round the pin on one side, and reaching further out on the other, opposite it.
        expect(towards).toBeCloseTo(a + 0.022, 6);
        expect(away).toBeCloseTo(1.5 * a, 6);
      }
    }
  });

  it('runs the crank in a main bearing either side of every throw, clear of the webs', () => {
    for (const over of [
      { cylinders: 1 as const },
      { cylinders: 4 as const, vAngle: 0 },
      { cylinders: 6 as const, vAngle: 0 },
      { cylinders: 6 as const, vAngle: 60, exhaustLayout: 'perBank' as const },
      { ...V8, crankType: 'crossplane' as const },
      { cylinders: 4 as const, vAngle: 180, crankType: 'boxer' as const },
      { cylinders: 6 as const, vAngle: 180, crankType: 'boxer' as const },
    ]) {
      const s = spec(over);
      const mesh = new EngineMesh(s);
      mesh.group.updateMatrixWorld(true);
      const named = (name: string) => {
        const found: THREE.Mesh[] = [];
        mesh.group.traverse((o) => {
          if (o instanceof THREE.Mesh && o.name === name) found.push(o);
        });
        return found;
      };
      const extent = (m: THREE.Mesh) => {
        const box = new THREE.Box3().setFromObject(m);
        return [box.min.z, box.max.z] as const;
      };
      const journals = named('main journal');
      const bearings = named('main bearing');
      // One at each end, and one in each gap between throws that has one.
      expect(journals, `${s.cylinders} cylinders`).toHaveLength(mainBearingsAfter(s).filter(Boolean).length + 2);
      expect(bearings).toHaveLength(journals.length);
      const webs = named('web').map(extent);
      for (const [i, journal] of journals.entries()) {
        const [from, to] = extent(journal);
        // Between the webs, into them no more than the seam allowance.
        for (const [wFrom, wTo] of webs) expect(Math.min(to, wTo) - Math.max(from, wFrom)).toBeLessThan(0.0011);
        // Its bearing round it, within its length.
        const [bFrom, bTo] = extent(bearings[i]!);
        expect(bFrom).toBeGreaterThan(from);
        expect(bTo).toBeLessThan(to);
      }
    }
  });

  it('has a main after every throw, but a flat engine only after each opposed pair', () => {
    expect(mainBearingsAfter(spec({ cylinders: 4 as const, vAngle: 0 }))).toEqual([true, true, true]);
    expect(mainBearingsAfter(spec({ ...V8, crankType: 'crossplane' as const }))).toEqual([true, true, true]);
    expect(mainBearingsAfter(spec({ cylinders: 4 as const, vAngle: 180, crankType: 'boxer' as const }))).toEqual([false, true, false]);
    expect(mainBearingsAfter(spec({ cylinders: 6 as const, vAngle: 180, crankType: 'boxer' as const }))).toEqual([
      false, true, false, true, false,
    ]);
  });

  it('joins a flat engine’s opposed pins with one web, round both pins and the shaft', () => {
    const s = spec({ cylinders: 4 as const, vAngle: 180, crankType: 'boxer' as const });
    const mesh = new EngineMesh(s);
    mesh.group.updateMatrixWorld(true);
    const links: THREE.Mesh[] = [];
    mesh.group.traverse((o) => {
      if (o instanceof THREE.Mesh && o.name === 'link web') links.push(o);
    });
    expect(links).toHaveLength(2);
    const a = s.stroke / 2;
    for (const link of links) {
      link.geometry.computeBoundingBox();
      const box = link.geometry.boundingBox!;
      // The pair's pins are half a turn apart, one up and one down: it spans both bosses.
      expect(box.max.y).toBeCloseTo(a + 0.022, 3);
      expect(box.min.y).toBeCloseTo(-(a + 0.022), 3);
      expect(box.max.z - box.min.z).toBeGreaterThan(0.004);
    }
  });

  it('runs the rods on a shared pin side by side, and each exhaust port at its own cylinder', () => {
    for (const over of [
      { cylinders: 2 as const, vAngle: 45, firingOffset: null },
      { cylinders: 6 as const, vAngle: 60, exhaustLayout: 'perBank' as const },
      { ...V8, crankType: 'crossplane' as const },
      { ...V8, crankType: 'flatplane' as const },
      { cylinders: 4 as const, vAngle: 0 },
    ]) {
      const s = spec(over);
      const mesh = new EngineMesh(s);
      poseAll(mesh, s, 37);
      mesh.group.updateMatrixWorld(true);
      const rods: THREE.Box3[] = [];
      mesh.group.traverse((o) => {
        if (o instanceof THREE.Mesh && o.name === 'rod') rods.push(new THREE.Box3().setFromObject(o));
      });
      expect(rods).toHaveLength(s.cylinders);
      for (let i = 0; i < rods.length; i++) {
        for (let j = i + 1; j < rods.length; j++) {
          const overlap = Math.min(rods[i]!.max.z, rods[j]!.max.z) - Math.max(rods[i]!.min.z, rods[j]!.min.z);
          expect(overlap, `${s.cylinders} cylinders, rods ${i} and ${j}`).toBeLessThan(0);
        }
      }
      for (let i = 0; i < s.cylinders; i++) {
        expect(mesh.exhaustPort(i).position.z).toBeCloseTo(mesh.pose(i).z, 12);
      }
    }
  });

  it('puts each piston where the physics says it is', () => {
    const s = spec({ ...V8, crankType: 'crossplane' });
    const mesh = new EngineMesh(s);
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
      const { sim, spec: s } = build(over, 1);
      const mesh = new EngineMesh(s);
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

  it('staggers the cylinders sharing a crankpin by a rod width, one bank a little ahead of the other', () => {
    const s = spec({ ...V8, crankType: 'crossplane' });
    const mesh = new EngineMesh(s);
    const pins = crankPins(s);
    const spacing = mesh.pose(pins[1]!.cylinders[0]!).z - mesh.pose(pins[0]!.cylinders[0]!).z;
    pins.forEach((pin, i) => {
      const [first, second] = pin.cylinders.map((c) => mesh.pose(c).z);
      expect(second! - first!).toBeCloseTo(ROD_STAGGER, 12);
      // Centred on the pin, which is where the pins are spaced along the crank.
      expect((first! + second!) / 2).toBeCloseTo((i - (pins.length - 1) / 2) * spacing, 12);
    });
    // Every bank-0 cylinder the same way off its pin, so each bank's cylinders are evenly spaced.
    const plan = firingPlan(s);
    const offsets = pins.map((pin, i) => mesh.pose(pin.cylinders.find((c) => plan.banks[c] === 0)!).z - i * spacing);
    for (const o of offsets) expect(o).toBeCloseTo(offsets[0]!, 12);
  });

  it('opens two banks and puts one cylinder per bank on each pin', () => {
    const s = spec({ ...V8, crankType: 'crossplane' });
    const mesh = new EngineMesh(s);
    const rots = new Set([...Array(8).keys()].map((i) => mesh.pose(i).bankRotation.toFixed(6)));
    expect(rots.size).toBe(2);
    for (const pin of crankPins(s)) {
      const banks = pin.cylinders.map((c) => mesh.pose(c).bankRotation);
      expect(banks[0]).not.toBeCloseTo(banks[1]!, 6);
    }
  });

  it('gives every cylinder its own exhaust port', () => {
    const s = spec({ ...V8 });
    const mesh = new EngineMesh(s);
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
    const mesh = new EngineMesh(s);
    const plan = firingPlan(s);
    for (let i = 0; i < mesh.bankCount; i++) {
      const port = mesh.exhaustPort(i);
      const outward = plan.banks[i] === 0 ? -1 : 1;
      expect(Math.sign(port.position.x), `cylinder ${i} port`).toBe(outward);
      expect(Math.sign(port.direction.x), `cylinder ${i} heading`).toBe(outward);
    }
  });

  /** Each port's flange is drawn where its pipe starts, facing the way the pipe leaves. */
  it.each([
    ['a single', {}],
    ['a V8', { ...V8 }],
    ['a boxer four', { cylinders: 4, vAngle: 180, crankType: 'boxer' }],
  ] as Array<[string, Partial<EngineSpec>]>)('marks every exhaust port of %s', (_n, engine) => {
    const mesh = new EngineMesh(spec(engine));
    mesh.group.updateMatrixWorld(true);
    const flanges: THREE.Object3D[] = [];
    mesh.group.traverse((o) => {
      if (o.name === 'exhaust port') flanges.push(o);
    });
    expect(flanges).toHaveLength(mesh.bankCount);
    for (let i = 0; i < mesh.bankCount; i++) {
      const port = mesh.exhaustPort(i);
      const at = flanges.map((f) => f.getWorldPosition(new THREE.Vector3()));
      const k = at.findIndex((p) => p.distanceTo(port.position) < 1e-9);
      expect(k, `cylinder ${i}`).toBeGreaterThanOrEqual(0);
      // The torus faces its local z.
      const facing = new THREE.Vector3(0, 0, 1).transformDirection(flanges[k]!.matrixWorld);
      expect(Math.abs(facing.dot(port.direction))).toBeCloseTo(1, 9);
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
    ['a four-valve four', { cylinders: 4, vAngle: 0, exValveCount: 2, inValveCount: 2 }],
  ] as Array<[string, Partial<EngineSpec>]>)('opens every valve of %s along its stem', (_n, engine) => {
    const mesh = new EngineMesh(spec(engine)) as unknown as {
      cyls: Array<{ exValves: THREE.Group[]; inValves: THREE.Group[] }>;
      poseValve: (g: THREE.Group, lift: number) => void;
    };
    const lift = 0.009;
    const count = (engine.exValveCount ?? 1) + (engine.inValveCount ?? 1);
    for (const c of mesh.cyls) {
      const valves = [...c.exValves, ...c.inValves];
      expect(valves.length).toBe(count);
      for (const valve of valves) {
        mesh.poseValve(valve, 0);
        const shut = valve.position.clone();
        mesh.poseValve(valve, lift);
        const moved = valve.position.clone().sub(shut);
        const stem = new THREE.Vector3(0, 1, 0).applyAxisAngle(new THREE.Vector3(0, 0, 1), valve.rotation.z);
        expect(moved.length()).toBeCloseTo(lift, 9);
        // Down the stem, away from the seat: exactly opposite the stem's upward axis.
        expect(moved.clone().normalize().dot(stem)).toBeCloseTo(-1, 9);
      }
    }
  });

  it.each([
    ['a 45° twin', { cylinders: 2, vAngle: 45 }],
    ['a narrow twin', { cylinders: 2, vAngle: 20 }],
    ['an oversquare 60° twin', { cylinders: 2, vAngle: 60, bore: 0.11, stroke: 0.06, rodLength: 0.11 }],
    ['an oversquare 90° V8', { ...V8, bore: 0.11, stroke: 0.06, rodLength: 0.105 }],
    ['a narrow V8', { ...V8, vAngle: 30 }],
    ['a split-pin V6', { cylinders: 6, vAngle: 60, crankType: 'split', bore: 0.1, stroke: 0.07 }],
  ] as Array<[string, Partial<EngineSpec>]>)('keeps the two banks’ pistons apart through the cycle: %s', (_n, over) => {
    const s = spec(over);
    const mesh = new EngineMesh(s);
    const cyls = (mesh as unknown as { cyls: Array<{ piston: THREE.Group; rotation: number; pinAngle: number }> }).cyls;
    const r = (s.bore / 2) * 0.985;
    const half = s.bore * 0.34 * 0.625;
    let deepest = 0;
    for (let deg = 0; deg < 360; deg += 5) {
      // Each cylinder's own crank angle, from where the crank has its pin and which way its bore points.
      mesh.update(
        cyls.map((c) => ({ crankAngle: deg - ((c.pinAngle - c.rotation) * 180) / Math.PI })),
        [],
      );
      mesh.group.updateMatrixWorld(true);
      // Which holds only if those crank angles agree with where the crank puts each pin.
      for (const c of cyls as unknown as Array<{ rod: THREE.Mesh }>) expect(c.rod.scale.y).toBeCloseTo(s.rodLength, 6);
      for (const a of cyls) {
        for (const b of cyls) {
          if (a === b || Math.abs(a.rotation - b.rotation) < 1e-9) continue;
          const into = b.piston.matrixWorld.clone().invert();
          for (const y of [-half, 0, half]) {
            for (let k = 0; k < 24; k++) {
              const t = (k / 24) * 2 * Math.PI;
              const p = new THREE.Vector3(r * Math.cos(t), y, r * Math.sin(t)).applyMatrix4(a.piston.matrixWorld).applyMatrix4(into);
              if (Math.abs(p.y) < half) deepest = Math.max(deepest, r - Math.hypot(p.x, p.z));
            }
          }
        }
      }
    }
    expect(deepest).toBeLessThan(1e-4);
  });

  it('keeps an inline engine on one side', () => {
    const mesh = new EngineMesh(spec({ cylinders: 4, vAngle: 0 }));
    for (let i = 0; i < mesh.bankCount; i++) expect(mesh.exhaustPort(i).direction.x).toBeGreaterThan(0);
  });

  it('rebuilds when the cylinder count changes', () => {
    const mesh = new EngineMesh(spec({ cylinders: 1 }));
    expect(mesh.bankCount).toBe(1);
    mesh.setSpec(spec({ ...V8 }));
    expect(mesh.bankCount).toBe(8);
    mesh.setSpec(spec({ cylinders: 2, vAngle: 45 }));
    expect(mesh.bankCount).toBe(2);
  });
});

describe('every preset', () => {
  it('draws for every preset without stretching a rod', () => {
    for (const preset of ENGINE_PRESETS) {
      const s = spec(preset.engine);
      const mesh = new EngineMesh(s);
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

describe('rebuilding', () => {
  it('releases the geometry and materials it drew before', () => {
    const mesh = new EngineMesh(spec({ cylinders: 8, vAngle: 90 }));
    const drawn = new Set<THREE.BufferGeometry | THREE.Material>();
    mesh.group.traverse((o) => {
      if (!(o instanceof THREE.Mesh)) return;
      drawn.add(o.geometry);
      for (const m of [o.material].flat() as THREE.Material[]) drawn.add(m);
    });
    const released = new Set<unknown>();
    for (const d of drawn) d.addEventListener('dispose', () => released.add(d));
    mesh.setSpec(spec({ cylinders: 4 }));
    expect(released.size).toBe(drawn.size);
  });
});
