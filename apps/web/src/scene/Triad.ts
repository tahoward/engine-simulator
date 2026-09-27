/**
 * A triad: the handles something is moved and turned by, one per axis of the part, as a CAD package's move
 * triad has them.
 *
 * An arrow moves along its axis, a square between two arrows moves in their plane, and a ring turns about
 * its axis. The axes are the part's own, turned with it (`setOrientation`), in the colours drawing a pipe
 * uses for x, y and z: red, green and blue. Each handle does one constrained thing, so a move does not
 * depend on which way the camera happens to be looking.
 *
 * A part that can be turned but not moved, such as a pipe held where it starts by its port or junction, shows
 * only the rings (`showMoves`).
 */

import * as THREE from 'three';

/** Colours for x, y and z: red, green and blue. */
export const AXIS_COLOURS = [new THREE.Color(0xff6b6b), new THREE.Color(0x7be07b), new THREE.Color(0x6ba6ff)];
export const ENGINE_AXES = [new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 1)];

/** What a handle of a triad does: move along an axis, move in the plane square to one, or turn about one. */
export type TriadHandle = { kind: 'axis' | 'plane' | 'ring'; axis: 0 | 1 | 2 };

/** What a move snaps to with shift held, m, and a turn, degrees. */
export const MOVE_STEP = 0.005;
export const TURN_STEP_DEG = 15;

/**
 * Below this, the pointer's ray is too close to edge-on to a ring's plane to meet it cleanly, and the
 * angle is read off a camera-facing plane instead.
 */
const MIN_RING_FACING = 0.2;

/** How far along an axis through `origin` the ray passes closest to it, m. */
export function axisOffset(ray: THREE.Ray, origin: THREE.Vector3, axis: THREE.Vector3): number | null {
  // Closest approach of two lines: the axis, origin + t a, and the ray, o + s d.
  const a = axis;
  const d = ray.direction;
  const w = origin.clone().sub(ray.origin);
  const ad = a.dot(d);
  const denom = 1 - ad * ad;
  // Looking straight down the axis there is no telling how far along it the pointer is.
  if (denom < 1e-4) return null;
  return (ad * w.dot(d) - w.dot(a)) / denom;
}

/** Where the ray meets the plane through `origin` square to `normal`, if it does. */
export function planePoint(ray: THREE.Ray, origin: THREE.Vector3, normal: THREE.Vector3): THREE.Vector3 | null {
  const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(normal, origin);
  const point = new THREE.Vector3();
  return ray.intersectPlane(plane, point) ? point : null;
}

/** A ring's axis, and a right-handed pair spanning its plane, so a positive angle is a positive turn about it. */
export interface RingFrame {
  axis: THREE.Vector3;
  u: THREE.Vector3;
  v: THREE.Vector3;
}

/**
 * Where the pointer is round a ring through `centre`, as an angle in its plane, radians.
 *
 * Straight onto the ring's plane where the view allows, so the angle is exactly where the pointer is on
 * the ring. Seen nearly edge-on, that intersection runs off to infinity, so the pointer is taken on a
 * plane facing the camera instead and flattened onto the ring's.
 */
export function ringAngle(
  ray: THREE.Ray,
  centre: THREE.Vector3,
  ring: RingFrame,
  cameraDir: THREE.Vector3,
): number | null {
  const facing = Math.abs(ray.direction.dot(ring.axis));
  const point = planePoint(ray, centre, facing > MIN_RING_FACING ? ring.axis : cameraDir);
  if (!point) return null;
  const { u, v } = ring;
  const rel = point.sub(centre);
  const x = rel.dot(u);
  const y = rel.dot(v);
  if (Math.hypot(x, y) < 1e-6) return null;
  return Math.atan2(y, x);
}

/** The frame of the ring about axis `axis` (0 x, 1 y, 2 z) of a part turned by `orientation`. */
export function ringFrame(axis: number, orientation = new THREE.Quaternion()): RingFrame {
  const a = ENGINE_AXES[axis]!.clone().applyQuaternion(orientation);
  const u = ENGINE_AXES[(axis + 1) % 3]!.clone().applyQuaternion(orientation);
  return { axis: a, u, v: a.clone().cross(u) };
}

/**
 * The orientation of a frame whose x runs along `dir`, with y as near straight up as that allows and z
 * across: a pipe segment's.
 */
export function frameAlong(dir: THREE.Vector3): THREE.Quaternion {
  const x = dir.clone().normalize();
  let z = x.clone().cross(new THREE.Vector3(0, 1, 0));
  if (z.lengthSq() < 1e-8) z = x.clone().cross(new THREE.Vector3(0, 0, 1));
  z.normalize();
  const y = z.clone().cross(x).normalize();
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, z));
}

/** The step from angle `from` to `to`, taken the short way round, so a turn accumulated from them never wraps. */
export function angleStep(from: number, to: number): number {
  let step = to - from;
  if (step > Math.PI) step -= 2 * Math.PI;
  else if (step < -Math.PI) step += 2 * Math.PI;
  return step;
}

/**
 * A turn about `axis`, near `turn`, that lands `dir` on a multiple of `step` round the ring, counted in the
 * engine's frame: from whichever engine axis lies most nearly in the ring's plane.
 *
 * Counted from the engine rather than from where the part started, so a part set at an odd angle squares up
 * to the engine as it snaps, rather than keeping its odd angle in whole steps.
 */
export function snapTurnToEngine(dir: THREE.Vector3, turn: number, axis: THREE.Vector3, step: number): number {
  let ref = new THREE.Vector3();
  let best = -1;
  for (const e of ENGINE_AXES) {
    const inPlane = e.clone().addScaledVector(axis, -e.dot(axis));
    if (inPlane.length() > best + 1e-9) {
      best = inPlane.length();
      ref = inPlane;
    }
  }
  ref.normalize();
  const refV = axis.clone().cross(ref);
  const d = dir.clone().addScaledVector(axis, -dir.dot(axis));
  if (d.lengthSq() < 1e-12) return snapTo(turn, step);
  const from = Math.atan2(d.dot(refV), d.dot(ref));
  return Math.round((from + turn) / step) * step - from;
}

/** `v` rounded to the nearest `step`. */
export function snapTo(v: number, step: number): number {
  return Math.round(v / step) * step;
}

export class Triad {
  readonly group = new THREE.Group();
  private readonly moving = new THREE.Group();
  private readonly turning = new THREE.Group();
  private readonly handles: THREE.Mesh[] = [];
  private readonly materials: THREE.MeshBasicMaterial[] = [];
  private readonly hoverMat = new THREE.MeshBasicMaterial({ color: 0xffd166, depthTest: false });
  private hovered: THREE.Mesh | null = null;
  private readonly orientation = new THREE.Quaternion();

  /** `size` is the arrows' length, m; the rings are sized to match. */
  constructor(size = 0.12) {
    this.group.add(this.moving, this.turning);
    const mat = (axis: number, opacity: number) => {
      const m = new THREE.MeshBasicMaterial({
        color: AXIS_COLOURS[axis]!,
        transparent: true,
        opacity,
        depthTest: false,
        side: THREE.DoubleSide,
      });
      this.materials.push(m);
      return m;
    };
    const add = (group: THREE.Group, mesh: THREE.Mesh, handle: TriadHandle) => {
      mesh.userData = handle;
      mesh.renderOrder = 20;
      this.handles.push(mesh);
      group.add(mesh);
    };
    const up = new THREE.Vector3(0, 1, 0);
    ([0, 1, 2] as const).forEach((axis) => {
      const dir = ENGINE_AXES[axis]!;
      const turn = new THREE.Quaternion().setFromUnitVectors(up, dir);
      // Arrow: a shaft and a head, pointing along the axis from the origin and out past the rings, so it
      // is not lost among them.
      const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, size * 1.2, 12), mat(axis, 0.95));
      shaft.quaternion.copy(turn);
      shaft.position.copy(dir).multiplyScalar(size * 0.6);
      add(this.moving, shaft, { kind: 'axis', axis });
      const head = new THREE.Mesh(new THREE.ConeGeometry(0.018, size * 0.3, 20), mat(axis, 0.95));
      head.quaternion.copy(turn);
      head.position.copy(dir).multiplyScalar(size * 1.35);
      add(this.moving, head, { kind: 'axis', axis });
      // Square: in the plane of the other two axes, a little out from the origin.
      const [p, q] = [ENGINE_AXES[(axis + 1) % 3]!, ENGINE_AXES[(axis + 2) % 3]!];
      const square = new THREE.Mesh(new THREE.PlaneGeometry(size * 0.25, size * 0.25), mat(axis, 0.35));
      square.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), dir);
      square.position.copy(p).add(q).multiplyScalar(size * 0.3);
      add(this.moving, square, { kind: 'plane', axis });
      // Ring: about the axis.
      const ring = new THREE.Mesh(new THREE.TorusGeometry(size * 0.95, 0.004, 8, 96), mat(axis, 0.9));
      ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), dir);
      add(this.turning, ring, { kind: 'ring', axis });
    });
    this.group.visible = false;
  }

  setVisible(on: boolean): void {
    this.group.visible = on;
  }

  /** Turn the triad with the part, so its axes are the part's own. */
  setOrientation(q: THREE.Quaternion): void {
    this.orientation.copy(q);
    this.moving.quaternion.copy(q);
    this.turning.quaternion.copy(q);
  }

  /** Axis `axis` of the part, in the world. */
  axisDir(axis: number): THREE.Vector3 {
    return ENGINE_AXES[axis]!.clone().applyQuaternion(this.orientation);
  }

  /** The frame of the ring about axis `axis`, in the world. */
  ring(axis: number): RingFrame {
    return ringFrame(axis, this.orientation);
  }

  /** Show or hide the arrows and squares, for a part that can be turned but not moved. */
  showMoves(on: boolean): void {
    this.moving.visible = on;
    for (const h of this.handles) {
      if ((h.userData as TriadHandle).kind !== 'ring') h.visible = on;
    }
  }

  /** Hide the ring about `axis`, where turning about it would do nothing: a straight pipe about itself. */
  hideRing(axis: number, hidden: boolean): void {
    for (const h of this.handles) {
      const d = h.userData as TriadHandle;
      if (d.kind === 'ring' && d.axis === axis) h.visible = !hidden;
    }
  }

  get visible(): boolean {
    return this.group.visible;
  }

  /** Where the arrows and squares are: the point they move. */
  setMoveOrigin(point: THREE.Vector3): void {
    this.moving.position.copy(point);
  }

  /** Where the rings are: the point turns are about. */
  setRotateOrigin(point: THREE.Vector3): void {
    this.turning.position.copy(point);
  }

  get moveOrigin(): THREE.Vector3 {
    return this.moving.position.clone();
  }

  get rotateOrigin(): THREE.Vector3 {
    return this.turning.position.clone();
  }

  /** The handle the ray hits, if the triad is showing and it hits one. */
  pick(raycaster: THREE.Raycaster): TriadHandle | null {
    if (!this.group.visible) return null;
    const hit = raycaster.intersectObjects(this.handles.filter((h) => h.visible), false)[0];
    return hit ? (hit.object.userData as TriadHandle) : null;
  }

  /** Highlight the handle under the pointer. Returns whether there is one. */
  hover(raycaster: THREE.Raycaster): boolean {
    const hit = this.group.visible
      ? raycaster.intersectObjects(this.handles.filter((h) => h.visible), false)[0]
      : undefined;
    const next = (hit?.object as THREE.Mesh | undefined) ?? null;
    if (next !== this.hovered) {
      for (const h of this.handles) {
        const { kind, axis } = h.userData as TriadHandle;
        const same = next && (next.userData as TriadHandle).kind === kind && (next.userData as TriadHandle).axis === axis;
        h.material = same ? this.hoverMat : this.materialFor(h);
      }
      this.hovered = next;
    }
    return next !== null;
  }

  private materialFor(h: THREE.Mesh): THREE.Material {
    return this.materials[this.handles.indexOf(h)]!;
  }

  dispose(): void {
    for (const h of this.handles) h.geometry.dispose();
    for (const m of this.materials) m.dispose();
    this.hoverMat.dispose();
  }
}
