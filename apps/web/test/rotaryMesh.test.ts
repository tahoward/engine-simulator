/**
 * The drawn rotary: the rotor stays inside its housing all the way round, rides its apexes on the trochoid,
 * and faces each chamber's flank to the spark plugs at its firing top dead centre and to the ports at the
 * top dead centre between its exhaust and its intake, where the simulation has them.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { ENGINE_PRESETS, defaultConfig, engineTypeChange, presetEngine, type EngineSpec } from '../src/model/spec.js';
import { exhaustPortOf, intakePortOf, trochoidAt } from '../src/model/geometry.js';
import { EngineMesh } from '../src/scene/EngineMesh.js';
import { rotorOutline } from '../src/scene/RotaryMesh.js';

function rotary(cylinders: number): EngineSpec {
  const base = defaultConfig().engine;
  return { ...base, ...engineTypeChange(base, 'rotary'), cylinders };
}

/** The trochoid in the housing's frame, its long axis along x, as the mesh builds it. */
function housing(spec: EngineSpec): THREE.Vector2[] {
  return Array.from({ length: 720 }, (_, k) => {
    const { point } = trochoidAt(spec, k / 2);
    return new THREE.Vector2(point[1], -point[0]);
  });
}

function inside(p: THREE.Vector2, polygon: THREE.Vector2[]): boolean {
  let hit = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const [a, b] = [polygon[i]!, polygon[j]!];
    if (a.y > p.y !== b.y > p.y && p.x < ((b.x - a.x) * (p.y - a.y)) / (b.y - a.y) + a.x) hit = !hit;
  }
  return hit;
}

describe('the rotor', () => {
  it('fits its housing at every angle, apexes all but touching it', () => {
    const spec = rotary(1);
    const outline = rotorOutline(spec);
    const wall = housing(spec);
    for (let deg = 0; deg < 1080; deg += 7) {
      const theta = (deg * Math.PI) / 180;
      const centre = new THREE.Vector2(spec.eccentricity * Math.cos(theta), spec.eccentricity * Math.sin(theta));
      for (const p of outline) {
        const at = p.clone().rotateAround(new THREE.Vector2(), theta / 3).add(centre);
        expect(inside(at, wall), `${deg}°`).toBe(true);
      }
    }
    // Its three apexes reach out to the trochoid, less the clearance.
    for (const k of [0, 60, 120]) expect(outline[k]!.length()).toBeGreaterThan(spec.rotorRadius - 0.002);
  });

  it('faces a chamber to the plugs as it fires, and to the ports between its exhaust and intake', () => {
    for (const rotors of [1, 2, 3, 4]) {
      const spec = rotary(rotors);
      const mesh = new EngineMesh(spec);
      const rotor = mesh.rotors!;
      expect(mesh.bankCount).toBe(rotors);
      /** Which way across the scene rotor 0's first chamber's flank faces, at shaft angle `deg`. */
      const facing = (deg: number) => {
        mesh.update([{ crankAngle: deg }], []);
        // The flank's middle, between apexes 0 and 1, turned out of the housing's frame into the scene's.
        const angle = rotor.pose(0).turn + Math.PI / 3 + Math.PI / 2;
        return new THREE.Vector2(Math.cos(angle), Math.sin(angle));
      };
      // The plugs are on -x, the ports on +x.
      expect(facing(0).x).toBeCloseTo(-1, 9);
      expect(facing(540).x).toBeCloseTo(1, 9);
      expect(Math.sign(exhaustPortOf(spec, 0).position[0])).toBe(1);
      expect(Math.sign(intakePortOf(spec, 0).position[0])).toBe(1);
      // The exhaust port low, the intake high.
      expect(exhaustPortOf(spec, 0).position[1]).toBeLessThan(intakePortOf(spec, 0).position[1]);
    }
  });

  it('draws the rotary preset', () => {
    const preset = ENGINE_PRESETS.find((p) => p.engine.engineType === 'rotary')!;
    const spec = presetEngine(preset, defaultConfig().engine);
    const mesh = new EngineMesh(spec);
    expect(mesh.bankCount).toBe(spec.cylinders);
    let rotors = 0;
    mesh.group.traverse((o) => (rotors += o.name === 'rotor' ? 1 : 0));
    expect(rotors).toBe(spec.cylinders);
  });
});
