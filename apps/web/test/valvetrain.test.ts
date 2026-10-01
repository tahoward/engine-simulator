/**
 * The cams drawn on the engine: each lobe cut to its valve's lift, so through the whole cycle an overhead
 * cam's lobe stays on its bucket, and a pushrod engine's lobe stays under its lifter; and a cam-switching
 * head's finger rockers, locked together only above the switch speed.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';

import { presetConfig } from '../bench/presetConfig.js';
import { BUCKET_HEIGHT, STEM_LENGTH } from '../src/model/geometry.js';
import { ENGINE_PRESETS, type EngineSpec } from '../src/model/spec.js';
import { EngineMesh } from '../src/scene/EngineMesh.js';

interface Lobe {
  mesh: THREE.Mesh;
  exhaust: boolean;
}
interface Cylinder {
  group: THREE.Group;
  exValves: THREE.Group[];
  inValves: THREE.Group[];
  lobes: Lobe[];
  rockers: Array<{ lifter: THREE.Mesh; up: THREE.Vector3; exhaust: boolean; tip: THREE.Vector3; pivot: THREE.Vector3; cup: THREE.Vector3 }>;
  fingers: Array<{ arm: THREE.Group; exhaust: boolean; middle: boolean; pin: THREE.Mesh | null }>;
}

const engine = (name: string): EngineSpec => presetConfig(ENGINE_PRESETS.find((p) => p.name.includes(name))!).engine;

/** How far out from its axis `lobe`'s surface is, m, in the direction `dir` in its parent's frame. */
function surface(lobe: THREE.Mesh, dir: THREE.Vector3): number {
  lobe.updateMatrixWorld(true);
  const parent = lobe.parent!.matrixWorld;
  const centre = lobe.position.clone().applyMatrix4(parent);
  const worldDir = dir.clone().transformDirection(parent);
  const reach = 0.3;
  const ray = new THREE.Raycaster(centre.clone().addScaledVector(worldDir, reach), worldDir.clone().negate());
  const hit = ray.intersectObject(lobe, false)[0];
  if (!hit) throw new Error('the ray missed the lobe');
  return reach - hit.distance;
}

/** Every crank angle round the cycle, in steps. */
const ANGLES = Array.from({ length: 72 }, (_, k) => k * 10);

describe('the cams', () => {
  for (const name of ['F20C', 'LT6', 'Single']) {
    it(`keep every overhead lobe on its bucket through the cycle: ${name}`, () => {
      const spec = engine(name);
      const mesh = new EngineMesh(spec);
      const cyls = (mesh as unknown as { cyls: Cylinder[] }).cyls;
      let worst = 0;
      for (const c of ANGLES) {
        mesh.update(
          cyls.map(() => ({ crankAngle: c })),
          [],
        );
        for (const cyl of cyls) {
          for (const lobe of cyl.lobes) {
            const valves = lobe.exhaust ? cyl.exValves : cyl.inValves;
            const valve = valves.find((v) => Math.abs((v.userData.z as number) - lobe.mesh.position.z) < 1e-9);
            // A high-speed lobe has no valve of its own under it: the valves are on the low ones here.
            if (!valve) continue;
            const stem = new THREE.Vector3(0, 1, 0).applyAxisAngle(new THREE.Vector3(0, 0, 1), valve.rotation.z);
            const top = valve.position.clone().addScaledVector(stem, spec.bore * STEM_LENGTH + BUCKET_HEIGHT);
            const centre = new THREE.Vector3(lobe.mesh.position.x, lobe.mesh.position.y, top.z);
            const gap = top.distanceTo(centre) - surface(lobe.mesh, stem.clone().negate());
            worst = Math.max(worst, Math.abs(gap));
          }
        }
      }
      // Within the lobe's facets, a tenth of a millimetre.
      expect(worst).toBeLessThan(2e-4);
    });
  }

  it('keep every pushrod engine lobe under its lifter through the cycle', () => {
    const spec = engine('LT2');
    expect(spec.pushrods).toBe(true);
    const mesh = new EngineMesh(spec);
    const cyls = (mesh as unknown as { cyls: Cylinder[] }).cyls;
    let worst = 0;
    for (const c of ANGLES) {
      mesh.update(
        cyls.map(() => ({ crankAngle: c })),
        [],
      );
      for (const cyl of cyls) {
        cyl.rockers.forEach((r, k) => {
          const lobe = cyl.lobes[k]!.mesh;
          const centre = new THREE.Vector3(lobe.position.x, lobe.position.y, r.lifter.position.z);
          // The lifter's foot, half its length below its middle.
          const foot = r.lifter.position.clone().addScaledVector(r.up, -0.0125);
          const gap = foot.distanceTo(centre) - surface(lobe, r.up);
          worst = Math.max(worst, Math.abs(gap));
        });
      }
    }
    expect(worst).toBeLessThan(2e-4);
  });

  it('stand a pushrod head’s two valves upright in a row along it, both arms reaching to the cam’s side', () => {
    const cyls = (new EngineMesh(engine('LT2')) as unknown as { cyls: Cylinder[] }).cyls;
    cyls.forEach((cyl, k) => {
      const [ex, inlet] = [cyl.exValves, cyl.inValves];
      expect(ex).toHaveLength(1);
      expect(inlet).toHaveLength(1);
      for (const v of [...ex, ...inlet]) {
        expect(v.rotation.z).toBeCloseTo(0, 12);
        expect(v.position.x).toBeCloseTo(0, 9);
      }
      // Neighbours the other way round, so their like valves sit together.
      expect(Math.sign(ex[0]!.position.z - inlet[0]!.position.z), `cylinder ${k}`).not.toBe(0);
      const [a, b] = cyl.rockers.map((r) => Math.sign(r.pivot.x - r.tip.x));
      expect(a).toBe(b);
    });
  });

  it('run every pushrod up outside its bore, however narrow the V', () => {
    for (const name of ['LT2', '45°']) {
      for (const vAngle of [15, 30, 45, 60, 90, 120]) {
        const spec = { ...engine(name), vAngle };
        const cyls = (new EngineMesh(spec) as unknown as { cyls: Cylinder[] }).cyls;
        for (const cyl of cyls) {
          for (const r of cyl.rockers) expect(Math.abs(r.cup.x), `${name} at ${vAngle}°`).toBeGreaterThan(spec.bore / 2 + 0.0035);
        }
      }
    }
  });

  it('carry a high-speed lobe for each side of a head with cam profile switching', () => {
    const spec = engine('F20C');
    expect(spec.camSwitchRpm).toBeGreaterThan(0);
    const cyls = (new EngineMesh(spec) as unknown as { cyls: Cylinder[] }).cyls;
    // Two valves a side, each on a low-speed lobe, and the high-speed lobe between them.
    expect(cyls[0]!.lobes).toHaveLength(6);
  });

  it('lock the middle finger rocker to the valves only above the switch speed', () => {
    const spec = engine('F20C');
    const mesh = new EngineMesh(spec);
    const cyls = (mesh as unknown as { cyls: Cylinder[] }).cyls;
    // At the intake's peak lift, where the high-speed lobe stands well above the low-speed ones.
    const peak = (spec.ivo + spec.ivc) / 2;
    const pose = (high: boolean) => {
      mesh.highCam = high;
      mesh.update(
        cyls.map(() => ({ crankAngle: peak })),
        [],
      );
      const intake = cyls[0]!.fingers.filter((f) => !f.exhaust);
      const outer = intake.filter((f) => !f.middle).map((f) => f.arm.rotation.z);
      const middle = intake.find((f) => f.middle)!;
      return { outer, middle: middle.arm.rotation.z, pin: middle.pin!.scale.y };
    };
    expect(cyls[0]!.fingers.filter((f) => !f.exhaust)).toHaveLength(3);
    const free = pose(false);
    // Free, the middle one swings further, on its own lobe, and the pin is back inside it.
    expect(Math.abs(free.middle)).toBeGreaterThan(Math.abs(free.outer[0]!) * 1.1);
    expect(free.pin).toBeLessThan(0.02);
    const locked = pose(true);
    // Locked, the three move as one, the pin out through them all.
    for (const o of locked.outer) expect(o).toBeCloseTo(locked.middle, 9);
    expect(locked.pin).toBeGreaterThan(0.03);
  });
});
