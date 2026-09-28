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
  Triad,
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

  it('lines up with the engine in its frame, but for rings that are the part’s own', () => {
    const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI / 2);
    const triad = new Triad();
    triad.setOrientation(q);
    const near = (a: THREE.Vector3, b: THREE.Vector3) => expect(a.distanceTo(b)).toBeLessThan(1e-12);
    near(triad.axisDir(0, 'axis'), new THREE.Vector3(0, 0, -1));
    triad.setEngineFrame(true);
    near(triad.axisDir(0, 'axis'), new THREE.Vector3(1, 0, 0));
    near(triad.ring(0).axis, new THREE.Vector3(1, 0, 0));
    triad.setRingsOwn(true);
    // The arrows stay the engine's; the rings go back to the part's.
    near(triad.axisDir(0, 'plane'), new THREE.Vector3(1, 0, 0));
    near(triad.ring(0).axis, new THREE.Vector3(0, 0, -1));
    triad.setEngineFrame(false);
    near(triad.axisDir(0, 'axis'), new THREE.Vector3(0, 0, -1));
    triad.dispose();
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

describe('swinging a whole pipe about where it starts', () => {
  async function swung(bent: boolean, axis: THREE.Vector3, angle: number, from = 0) {
    const { bendWhole, pipeShape, swingPipe } = await import('../src/scene/drawing.js');
    const { layoutPipe, turnHeading } = await import('../src/scene/PipeMesh.js');
    const { makeSegment } = await import('../src/model/spec.js');
    const origin = new THREE.Vector3(0.2, 0.1, -0.1);
    const base = new THREE.Vector3(1, 0, 0);
    let segments = [
      makeSegment({ length: 0.2, yaw: 0.1 }),
      makeSegment({ length: 0.3, yaw: 0.9, pitch: 0.4 }),
      makeSegment({ length: 0.25, yaw: -0.6, pitch: -0.5 }),
    ];
    const heading = turnHeading(base, 0.3, -0.2);
    if (bent) {
      // Bent with the bend tool: the second straight in one plane, the third in another.
      const at = (i: number) => pipeShape(segments, heading).starts[i]!;
      const up = new THREE.Vector3(0, 1, 0);
      segments[1] = bendWhole(segments[1]!, at(1), up.clone().cross(at(1)).normalize(), 0.8, 0.06).segment;
      segments[2] = bendWhole(segments[2]!, at(2), up, -1.1, 0.06).segment;
    }
    const duct = {
      id: 'a',
      from: { kind: 'free' as const, position: [origin.x, origin.y, origin.z] as [number, number, number] },
      to: { kind: 'mouth' as const },
      headingYaw: 0.3,
      headingPitch: -0.2,
      headingFrame: 'world' as const,
      segments,
    };
    const before = layoutPipe(duct.segments, origin, heading);
    const lengths = duct.segments.map((s) => s.length);
    swingPipe(duct, base, pipeShape(duct.segments, heading), axis, angle, from);
    const after = layoutPipe(duct.segments, origin, turnHeading(base, duct.headingYaw, duct.headingPitch));
    return { before, after, origin, lengths, duct, heading };
  }

  it('held to the face it starts from, turns about its axis: the first straight stays put, the rest swings', async () => {
    const { pipeShape } = await import('../src/scene/drawing.js');
    const { turnHeading } = await import('../src/scene/PipeMesh.js');
    const { makeSegment } = await import('../src/model/spec.js');
    const face = pipeShape([makeSegment({ length: 0.2, yaw: 0.1 })], turnHeading(new THREE.Vector3(1, 0, 0), 0.3, -0.2)).starts[0]!;
    const { before, after, origin } = await swung(true, face, 1.3);
    // The first straight runs out along the face's axis, and still does.
    const firstEnd = before.joints[0]!;
    expect(after.joints[0]!.distanceTo(firstEnd)).toBeLessThan(1e-9);
    // Everything after it turned about that axis, through where the pipe starts.
    before.stations.forEach((st, i) => {
      const expected = st.position.clone().sub(origin).applyAxisAngle(face, 1.3).add(origin);
      expect(after.stations[i]!.position.distanceTo(expected)).toBeLessThan(1e-9);
    });
  });

  it.each([false, true])('turns it as one piece, every segment, corner and bend kept, bent: %s', async (bent) => {
    const axis = new THREE.Vector3(0.3, 0.8, -0.5).normalize();
    const { before, after, origin, lengths, duct } = await swung(bent, axis, 0.7);
    if (bent) expect(duct.segments.filter((s) => s.curve)).toHaveLength(2);
    // Every point along it, bends included, where turning the whole pipe rigidly puts it.
    expect(after.stations).toHaveLength(before.stations.length);
    before.stations.forEach((st, i) => {
      const expected = st.position.clone().sub(origin).applyAxisAngle(axis, 0.7).add(origin);
      expect(after.stations[i]!.position.distanceTo(expected)).toBeLessThan(1e-9);
    });
    duct.segments.forEach((s, i) => expect(s.length).toBe(lengths[i]));
  });

  it('rolls a bend further along about the way it sets off, and the rest of the pipe with it', async () => {
    const { pipeShape } = await import('../src/scene/drawing.js');
    // The shape the bent pipe has, for the way its second bend sets off: `swung` bends the same way each time.
    const first = await swung(true, new THREE.Vector3(1, 0, 0), 0);
    const axis = pipeShape(first.duct.segments, first.heading).starts[2]!;
    const { before, after, duct } = await swung(true, axis, 1.1, 2);
    expect(duct.headingYaw).toBe(0.3);
    const pivot = before.joints[1]!;
    before.stations.forEach((st, i) => {
      const expected = st.segment < 2 ? st.position : st.position.clone().sub(pivot).applyAxisAngle(axis, 1.1).add(pivot);
      expect(after.stations[i]!.position.distanceTo(expected)).toBeLessThan(1e-9);
    });
  });
});
