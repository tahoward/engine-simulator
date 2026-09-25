/**
 * Junction fittings: where straight pipes snap together.
 *
 * A fitting is a plain part sized to the pipes, not a shape derived from them, so what has to hold is
 * simple: every pipe end is inside it, so there is never a gap; a junction where pipes meet at a point gets
 * a small ball, not a funnel; and pipes arriving spread apart get a merge collector that takes them all.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';

import {
  buildJointGeometry,
  hubShape,
  jointDistance,
  throughPipe,
  type JointLimb,
  type JointPlacement,
} from '../src/scene/jointMesh.js';

/** `n` pipes arriving spread around a collar about `+x`, plus an outlet leaving along it. */
function collector(n: number, radius = 0.021, spread = 0.06, outlet = 0.03): JointPlacement {
  const limbs: JointLimb[] = [];
  for (let i = 0; i < n; i++) {
    const theta = (i / n) * Math.PI * 2;
    const offset = new THREE.Vector3(0, Math.cos(theta), Math.sin(theta)).multiplyScalar(spread);
    limbs.push({
      point: offset.clone().addScaledVector(new THREE.Vector3(1, 0, 0), -0.01),
      dir: new THREE.Vector3(1, 0, 0).sub(offset.clone().multiplyScalar(4)).normalize(),
      radius,
      duct: `in${i}`,
    });
  }
  limbs.push({ point: new THREE.Vector3(), dir: new THREE.Vector3(-1, 0, 0), radius: outlet, duct: 'out' });
  return { centre: new THREE.Vector3(), axis: new THREE.Vector3(1, 0, 0), limbs };
}

/** A pipe teed into another's side: two halves of the through pipe, and the branch. */
function tee(radius = 0.02): JointPlacement {
  return {
    centre: new THREE.Vector3(),
    axis: new THREE.Vector3(1, 0, 0),
    limbs: [
      { point: new THREE.Vector3(), dir: new THREE.Vector3(1, 0, 0), radius, duct: 'up' },
      { point: new THREE.Vector3(), dir: new THREE.Vector3(-1, 0, 0), radius, duct: 'down' },
      { point: new THREE.Vector3(), dir: new THREE.Vector3(0.3, -0.95, 0).normalize(), radius, duct: 'branch' },
    ],
  };
}

const CASES: Array<[string, JointPlacement]> = [
  ['a tee', tee()],
  ['a 2-into-1', collector(2)],
  ['a 4-into-1', collector(4)],
  ['an 8-into-1', collector(8, 0.017, 0.085, 0.048)],
];

describe.each(CASES)('%s', (_name, placement) => {
  it('takes every pipe end inside it, so there is no gap to see', () => {
    for (const [i, limb] of placement.limbs.entries()) {
      const d = jointDistance(placement, limb.point);
      expect(d, `limb ${i} sits ${(d * 1000).toFixed(1)} mm outside`).toBeLessThan(0);
    }
  });

  it('builds a finite mesh', () => {
    const geom = buildJointGeometry(placement)!;
    expect(geom).not.toBeNull();
    const pos = geom.getAttribute('position');
    expect(pos.count).toBeGreaterThan(0);
    for (let i = 0; i < pos.count * 3; i++) expect(Number.isFinite(pos.array[i]!)).toBe(true);
  });
});

describe('sizing', () => {
  /** The funnel this whole line of work started from was 391 mm across a joint of two 40 mm pipes. */
  it('fits pipes meeting at a point with a ball a little wider than the widest', () => {
    const hub = hubShape(tee());
    expect(hub.kind).toBe('ball');
    expect(hub.radius).toBeGreaterThan(0.02);
    expect(hub.radius).toBeLessThan(0.02 * 1.3);
  });

  it('fits pipes arriving spread apart with a merge collector that takes them all', () => {
    const hub = hubShape(collector(4));
    expect(hub.kind).toBe('collector');
    if (hub.kind !== 'collector') return;
    // Runners 60 mm off the axis and 21 mm in radius, so a drum of just over 81 mm.
    expect(hub.radius).toBeGreaterThan(0.081);
    expect(hub.radius).toBeLessThan(0.081 * 1.15);
    // Stepping down to a collar just over the outlet.
    expect(hub.outlet).toBeGreaterThan(0.03);
    expect(hub.outlet).toBeLessThan(0.03 * 1.1);
  });

  /** Runners from opposite sides of a V can finish a pipe length or two apart along the axis. */
  it('stretches its drum to take ends that finish at different depths', () => {
    const r = 0.025;
    const staggered: JointPlacement = {
      centre: new THREE.Vector3(),
      axis: new THREE.Vector3(1, 0, 0),
      limbs: [
        { point: new THREE.Vector3(-0.08, 0.03, 0), dir: new THREE.Vector3(1, 0, 0), radius: r },
        { point: new THREE.Vector3(0.08, -0.03, 0), dir: new THREE.Vector3(1, 0, 0), radius: r },
        { point: new THREE.Vector3(), dir: new THREE.Vector3(-1, 0, 0), radius: r },
      ],
    };
    for (const limb of staggered.limbs) expect(jointDistance(staggered, limb.point)).toBeLessThan(0);
  });

  it('declines to build anything for fewer than two limbs', () => {
    const one: JointPlacement = {
      centre: new THREE.Vector3(),
      axis: new THREE.Vector3(1, 0, 0),
      limbs: [{ point: new THREE.Vector3(), dir: new THREE.Vector3(1, 0, 0), radius: 0.02 }],
    };
    expect(buildJointGeometry(one)).toBeNull();
  });
});

describe('the pipe running through', () => {
  it('is found at a tee, as the pipe to rejoin when the tee is deleted', () => {
    expect(throughPipe(tee())).toEqual(['up', 'down']);
  });

  it('is absent at a collector', () => {
    expect(throughPipe(collector(4))).toBeNull();
  });
});
