/**
 * A junction's mark: a small sphere where the pipes meet, there to pick the junction by and shown only
 * while it is selected.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';

import { JointMesh, type JointPlacement } from '../src/scene/jointMesh.js';

/** A pipe teed into another's side: two halves of the through pipe, and the branch. */
function tee(radius = 0.02): JointPlacement {
  return {
    centre: new THREE.Vector3(0.1, 0.2, 0.3),
    axis: new THREE.Vector3(1, 0, 0),
    limbs: [
      { point: new THREE.Vector3(), dir: new THREE.Vector3(1, 0, 0), radius },
      { point: new THREE.Vector3(), dir: new THREE.Vector3(-1, 0, 0), radius: radius * 1.5 },
      { point: new THREE.Vector3(), dir: new THREE.Vector3(0.3, -0.95, 0).normalize(), radius },
    ],
  };
}

describe('a junction mark', () => {
  it('is a sphere at the centre, a little wider than the widest pipe', () => {
    const mark = new JointMesh();
    mark.rebuild(tee());
    const target = mark.pickTarget!;
    expect(target).not.toBeNull();
    expect(target.position.distanceTo(tee().centre)).toBeLessThan(1e-12);
    const radius = (target.geometry as THREE.SphereGeometry).parameters.radius;
    expect(radius).toBeGreaterThan(0.03);
    expect(radius).toBeLessThan(0.03 * 1.5);
    mark.dispose();
  });

  it('is hidden until the junction is selected, and still there to pick', () => {
    const mark = new JointMesh();
    mark.rebuild(tee());
    expect(mark.pickTarget!.visible).toBe(false);
    mark.setSelected(true);
    expect(mark.pickTarget!.visible).toBe(true);
    // Selection survives a rebuild.
    mark.rebuild(tee());
    expect(mark.pickTarget!.visible).toBe(true);
    mark.dispose();
  });

  it('draws nothing for fewer than two pipes', () => {
    const one: JointPlacement = { ...tee(), limbs: tee().limbs.slice(0, 1) };
    const mark = new JointMesh();
    mark.rebuild(one);
    expect(mark.pickTarget).toBeNull();
    expect(mark.boundingBox().isEmpty()).toBe(true);
    mark.dispose();
  });
});
