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
      expect(into(l.tube, 1, 1), preset.name).toBeGreaterThan(l.airbox.size.z * 0.8);
      expect(into(l.snorkel, 0, -1), preset.name).toBeGreaterThan(l.airbox.size.x * 0.8);
    }
  });
});
