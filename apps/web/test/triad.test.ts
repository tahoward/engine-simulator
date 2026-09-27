/**
 * The triad's geometry: where the pointer is along an arrow, in a square's plane and round a ring.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';

import {
  MOVE_STEP,
  angleStep,
  axisOffset,
  frameAlong,
  planePoint,
  ringAngle,
  ringFrame,
  snapTo,
  snapTurnToEngine,
} from '../src/scene/Triad.js';

/** A ray from `from` through `to`. */
const ray = (from: THREE.Vector3, to: THREE.Vector3) => new THREE.Ray(from, to.clone().sub(from).normalize());

describe('the triad', () => {
  const origin = new THREE.Vector3(0.1, 0.2, -0.05);

  it('reads how far along an arrow the pointer is, however the axis is seen', () => {
    const x = new THREE.Vector3(1, 0, 0);
    // Rays from two different eyes through the same point on the axis, 37 mm along it.
    const target = origin.clone().addScaledVector(x, 0.037);
    for (const eye of [new THREE.Vector3(0.3, 1, 2), new THREE.Vector3(-1, 0.5, 0.8)]) {
      expect(axisOffset(ray(eye, target), origin, x)).toBeCloseTo(0.037, 9);
    }
    // Looking straight down the axis there is nothing to read.
    expect(axisOffset(ray(new THREE.Vector3(5, 0.2, -0.05), origin), origin, x)).toBeNull();
  });

  it('lands in the plane of a square', () => {
    const up = new THREE.Vector3(0, 1, 0);
    const point = planePoint(ray(new THREE.Vector3(0.4, 1.3, 0.9), new THREE.Vector3(0, 0.2, 0.1)), origin, up)!;
    expect(point.y).toBeCloseTo(origin.y, 12);
  });

  it('reads where the pointer is round a ring, and turns accumulate without wrapping', () => {
    const camera = new THREE.Vector3(0, -1, 0);
    const eye = new THREE.Vector3(0.1, 2, -0.05);
    // Round the vertical ring, whose frame for the y axis is u = z, v = x.
    const at = (deg: number) => {
      const a = (deg * Math.PI) / 180;
      const p = origin.clone().add(new THREE.Vector3(Math.sin(a), 0, Math.cos(a)).multiplyScalar(0.1));
      return ringAngle(ray(eye, p), origin, ringFrame(1), camera)!;
    };
    let turned = 0;
    let last = at(0);
    for (let deg = 20; deg <= 300; deg += 20) {
      const now = at(deg);
      turned += angleStep(last, now);
      last = now;
    }
    expect((turned * 180) / Math.PI).toBeCloseTo(300, 6);
  });

  it('still reads a ring seen edge on, off a plane facing the camera', () => {
    const eye = new THREE.Vector3(2, 0.2, -0.05);
    const p = origin.clone().add(new THREE.Vector3(0, 0, 0.1));
    const angle = ringAngle(ray(eye, p), origin, ringFrame(1), new THREE.Vector3(-1, 0, 0));
    expect(angle).not.toBeNull();
    expect(Number.isFinite(angle!)).toBe(true);
  });

  it('turns with the part: its axes are the part’s own', () => {
    // A part turned a quarter turn about the vertical: its x now runs along the world's -z.
    const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI / 2);
    const ring = ringFrame(0, q);
    expect(ring.axis.distanceTo(new THREE.Vector3(0, 0, -1))).toBeLessThan(1e-12);
    expect(ring.u.dot(ring.axis)).toBeCloseTo(0, 12);
    expect(ring.v.dot(ring.axis)).toBeCloseTo(0, 12);
  });

  it('frames a pipe segment with x along it and y as near up as it can be', () => {
    const dir = new THREE.Vector3(0.6, 0.3, -0.5).normalize();
    const q = frameAlong(dir);
    const x = new THREE.Vector3(1, 0, 0).applyQuaternion(q);
    const y = new THREE.Vector3(0, 1, 0).applyQuaternion(q);
    expect(x.distanceTo(dir)).toBeLessThan(1e-9);
    expect(y.dot(dir)).toBeCloseTo(0, 9);
    expect(y.y).toBeGreaterThan(0);
    // A vertical segment still gets a frame.
    const up = new THREE.Vector3(1, 0, 0).applyQuaternion(frameAlong(new THREE.Vector3(0, 1, 0)));
    expect(up.distanceTo(new THREE.Vector3(0, 1, 0))).toBeLessThan(1e-9);
  });

  it('snaps a turn square to the engine, from a part set at an odd angle', () => {
    const step = (15 * Math.PI) / 180;
    const up = new THREE.Vector3(0, 1, 0);
    // A level pipe 7 degrees off the engine's x, turned about the vertical by about 40 degrees.
    const dir = new THREE.Vector3(1, 0, 0).applyAxisAngle(up, (7 * Math.PI) / 180);
    const turn = snapTurnToEngine(dir, (40 * Math.PI) / 180, up, step);
    const after = dir.clone().applyAxisAngle(up, turn);
    // It lands on 45 degrees off the engine's x, not 7 + 45.
    const angle = (Math.atan2(-after.z, after.x) * 180) / Math.PI;
    expect(angle).toBeCloseTo(45, 9);
    // And turned a little back towards the axis, it squares up onto the axis itself.
    const square = dir.clone().applyAxisAngle(up, snapTurnToEngine(dir, -0.05, up, step));
    expect(square.distanceTo(new THREE.Vector3(1, 0, 0))).toBeLessThan(1e-9);
  });

  it('snaps to 5 mm steps', () => {
    expect(snapTo(0.0123, MOVE_STEP)).toBeCloseTo(0.01, 12);
    expect(snapTo(-0.0128, MOVE_STEP)).toBeCloseTo(-0.015, 12);
  });
});
