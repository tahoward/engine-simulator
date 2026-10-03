/**
 * The intake as drawn: where its parts join, they are open through.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';

import { ENGINE_PRESETS, defaultConfig, presetEngine } from '../src/model/spec.js';
import { InletMesh } from '../src/scene/InletMesh.js';
import { inletLayout } from '../src/scene/inletLayout.js';

describe('the airbox', () => {
  it('is open where the tube and the snorkel join it, so each can be seen through into it', () => {
    for (const preset of ENGINE_PRESETS) {
      const spec = presetEngine(preset, defaultConfig().engine);
      const mesh = new InletMesh();
      mesh.rebuild(spec);
      mesh.group.updateMatrixWorld(true);
      const l = inletLayout(spec);
      const into = (curve: THREE.Curve<THREE.Vector3>, u: number, inwards: number) => {
        const at = curve.getPointAt(u);
        const dir = curve.getTangentAt(u).normalize().multiplyScalar(inwards);
        // From a little way back inside the pipe, along its axis into the box.
        const ray = new THREE.Raycaster(at.clone().addScaledVector(dir, -0.005), dir);
        const hits = ray.intersectObject(mesh.group, true).filter((h) => h.object.visible && h.distance > 1e-4);
        return hits[0]?.distance ?? Infinity;
      };
      // Through to the far side of the box, not stopped at the wall the pipe joins.
      for (const tube of l.tubes) expect(into(tube, 1, 1), preset.name).toBeGreaterThan(l.airbox.size.z * 0.8);
      expect(into(l.snorkel, 0, -1), preset.name).toBeGreaterThan(l.airbox.size.x * 0.8);
    }
  });
});

describe('the plenum', () => {
  it('is drawn the size the simulation solves it at, and holds one and a half times the displacement left to itself', async () => {
    const { plenumShapeOf, plenumVolumeOf } = await import('../src/model/intakeSizing.js');
    const { displacement } = await import('../src/model/spec.js');
    for (const preset of ENGINE_PRESETS) {
      const spec = presetEngine(preset, defaultConfig().engine);
      const l = inletLayout(spec);
      const shape = plenumShapeOf(spec);
      expect(l.plenum.size.z, preset.name).toBeCloseTo(shape.length, 12);
      expect(l.plenum.size.y, preset.name).toBeCloseTo(shape.height, 12);
      const left = { ...spec, plenumWidth: 0, plenumHeight: 0, plenumVolume: 0 };
      expect(plenumVolumeOf(left) / (displacement(spec) * spec.cylinders), preset.name).toBeCloseTo(1.5, 9);
      // Its size set, it is drawn that size.
      const sized = { ...spec, plenumLength: 0.5, plenumWidth: 0.2, plenumHeight: 0.19 };
      const drawn = inletLayout(sized).plenum.size;
      expect([drawn.x, drawn.y, drawn.z], preset.name).toEqual([plenumShapeOf(sized).width, plenumShapeOf(sized).height, 0.5]);
    }
  });
});

describe('dual plenums', () => {
  const lt6 = () => presetEngine(ENGINE_PRESETS.find((p) => p.name.includes('LT6'))!, defaultConfig().engine);

  it('are drawn one each side of a wall, each with its own throttle body and tube, on a V', async () => {
    const { solvedPlenum } = await import('../src/scene/inletLayout.js');
    const { throttleFaceOf } = await import('../src/model/intakeSizing.js');
    const spec = { ...lt6(), dualPlenum: true, throttleDia: 0.087 };
    const l = inletLayout(spec);
    expect([...l.plenum.sides].sort()).toEqual([-1, 1]);
    expect(l.throttles).toHaveLength(2);
    expect(l.tubes).toHaveLength(2);
    expect(l.balances).toHaveLength(2);
    expect(l.plenum.size.x).toBeGreaterThanOrEqual(2 * throttleFaceOf(spec) - 1e-12);
    // Each throttle body on its own side of the wall, on its bank's side.
    l.throttles.forEach((t, k) => expect(Math.sign(t.centre.x - l.plenum.centre.x)).toBe(l.plenum.sides[k]));
    // Each tube as long as the solver's duct, into the airbox apart from the other.
    for (const tube of l.tubes) expect(tube.getLength()).toBeCloseTo(l.segments[0]!.length, 2);
    expect(l.tubes[0]!.getPointAt(1).distanceTo(l.tubes[1]!.getPointAt(1))).toBeGreaterThan(2 * l.tubeRadius);
    expect(solvedPlenum(spec).dualPlenum).toBe(true);
    const mesh = new InletMesh();
    mesh.rebuild(spec);
    mesh.dispose();
  });

  it('are one plenum on an engine of one bank, or with a head shared between its banks', async () => {
    const { solvedPlenum } = await import('../src/scene/inletLayout.js');
    const inline = { ...presetEngine(ENGINE_PRESETS.find((p) => p.name.includes('F20C'))!, defaultConfig().engine), dualPlenum: true };
    const narrow = { ...lt6(), dualPlenum: true, vAngle: 15 };
    for (const spec of [inline, narrow]) {
      const l = inletLayout(spec);
      expect(l.plenum.sides).toEqual([0]);
      expect(l.throttles).toHaveLength(1);
      expect(l.balances).toHaveLength(0);
      expect(solvedPlenum(spec).dualPlenum).toBe(false);
    }
  });
});
