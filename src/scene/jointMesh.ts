/**
 * The fitting at a junction: where pipes snap together.
 *
 * Where the pipes meet at a point — a tee, a branch — it is a ball a little wider than the widest of them.
 * Where they arrive spread around a collar, it is a merge collector: a drum the runners plug into, stepping
 * down in a cone to a collar on the pipe that leaves. Either way every pipe end sits inside it, so there is
 * never a gap to see, and it is a plain part sized to the pipes rather than a shape derived from them.
 *
 * This replaced a procedural blend of the pipes — capsules and cones smooth-unioned and polygonised — that
 * made each collector a streamlined, welded-looking casting. That needed a signed distance field, surface
 * nets, a taper per runner, a sleeve along the outlet and a string of special cases for tees, all rebuilt
 * on every drag frame. With straight pipes that snap together, a fitting is what the junction is.
 */

import * as THREE from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

/** One pipe arriving at, or leaving, a joint. */
export interface JointLimb {
  /** Where the pipe's centreline meets the joint. */
  point: THREE.Vector3;
  /** Unit direction *into* the joint. */
  dir: THREE.Vector3;
  radius: number;
  /** The duct this limb is, when the joint was laid out from a graph. */
  duct?: string;
}

export interface JointPlacement {
  /** Where the limbs meet. */
  centre: THREE.Vector3;
  /** The direction flow leaves by. A route drawn from the junction starts off along it. */
  axis: THREE.Vector3;
  limbs: JointLimb[];
}

/**
 * How much wider a fitting is than what it has to take, as a fraction.
 *
 * Enough that a pipe end arriving at an angle, whose rim reaches a little further than its centreline
 * does, is still inside, and enough to read as a fitting rather than as the pipe itself.
 */
const BALL_CLEARANCE = 1.15;
const COLLECTOR_CLEARANCE = 1.06;

/** Half-angle of a merge collector's cone, from the runners' drum down to the pipe leaving it. */
const COLLECTOR_TAPER = (20 * Math.PI) / 180;

/**
 * The fitting's shape.
 *
 * A ball where everything meets at a point. A merge collector where pipes arrive spread around a collar:
 * a drum from `start` along `axis` that the runners plug into, `body` long, then a cone `taper` long down to
 * a collar just over the pipe leaving it. A single shape sized to take every end whatever its spread was a
 * big ball on every collector — 19 cm across on a V8 — which is not what a collector is.
 */
export type HubShape =
  | { kind: 'ball'; centre: THREE.Vector3; radius: number }
  | {
      kind: 'collector';
      start: THREE.Vector3;
      axis: THREE.Vector3;
      radius: number;
      body: number;
      taper: number;
      outlet: number;
    };

/** Size the fitting to what arrives. */
export function hubShape(placement: JointPlacement): HubShape {
  const axis = placement.axis.clone().normalize();
  let widest = 0;
  let furthest = 0;
  let enclose = 0;
  for (const limb of placement.limbs) {
    const d = limb.point.distanceTo(placement.centre);
    widest = Math.max(widest, limb.radius);
    furthest = Math.max(furthest, d);
    enclose = Math.max(enclose, d + limb.radius);
  }
  if (furthest <= widest) {
    return { kind: 'ball', centre: placement.centre.clone(), radius: enclose * BALL_CLEARANCE };
  }

  // The drum spans the runner ends, from a radius before the first to a little past the last, however far
  // apart along the axis they finish — runners from opposite sides of a V can be a pipe length apart.
  const arriving = placement.limbs.filter((l) => l.dir.dot(axis) > 0);
  const leaving = placement.limbs.filter((l) => l.dir.dot(axis) <= 0);
  let first = Infinity;
  let last = -Infinity;
  let reach = 0;
  for (const limb of arriving.length > 0 ? arriving : placement.limbs) {
    const rel = limb.point.clone().sub(placement.centre);
    const along = rel.dot(axis);
    reach = Math.max(reach, rel.addScaledVector(axis, -along).length() + limb.radius);
    first = Math.min(first, along - limb.radius);
    last = Math.max(last, along + limb.radius * 0.5);
  }
  let outlet = 0;
  for (const limb of leaving) outlet = Math.max(outlet, limb.radius);
  if (outlet <= 0) outlet = widest;
  outlet *= COLLECTOR_CLEARANCE;
  const radius = Math.max(reach * COLLECTOR_CLEARANCE, outlet);
  return {
    kind: 'collector',
    start: placement.centre.clone().addScaledVector(axis, first),
    axis,
    radius,
    body: last - first,
    taper: (radius - outlet) / Math.tan(COLLECTOR_TAPER),
    outlet,
  };
}

/** Distance to a joint's fitting, negative inside. Exact for a ball, a bound for a collector. */
export function jointDistance(placement: JointPlacement, p: THREE.Vector3): number {
  const hub = hubShape(placement);
  if (hub.kind === 'ball') return p.distanceTo(hub.centre) - hub.radius;
  const rel = p.clone().sub(hub.start);
  const t = rel.dot(hub.axis);
  const radial = rel.addScaledVector(hub.axis, -t).length();
  const end = hub.body + hub.taper;
  const allowed =
    t <= hub.body ? hub.radius : hub.radius + ((hub.outlet - hub.radius) * (t - hub.body)) / Math.max(hub.taper, 1e-9);
  return Math.max(radial - allowed, -t, t - end);
}

export function buildJointGeometry(placement: JointPlacement): THREE.BufferGeometry | null {
  if (placement.limbs.length < 2) return null;
  const hub = hubShape(placement);
  if (hub.radius <= 0) return null;
  if (hub.kind === 'ball') {
    const geom = new THREE.SphereGeometry(hub.radius, 28, 18);
    geom.translate(hub.centre.x, hub.centre.y, hub.centre.z);
    return geom;
  }

  // Built along +Y from the origin, then turned onto the axis. Separate parts so the edges stay crisp.
  const parts: THREE.BufferGeometry[] = [];
  const face = new THREE.CircleGeometry(hub.radius, 32);
  face.rotateX(Math.PI / 2);
  parts.push(face);
  if (hub.body > 1e-6) {
    const drum = new THREE.CylinderGeometry(hub.radius, hub.radius, hub.body, 32, 1, true);
    drum.translate(0, hub.body / 2, 0);
    parts.push(drum);
  }
  if (hub.taper > 1e-6) {
    const cone = new THREE.CylinderGeometry(hub.outlet, hub.radius, hub.taper, 32, 1, true);
    cone.translate(0, hub.body + hub.taper / 2, 0);
    parts.push(cone);
  }
  const geom = mergeGeometries(parts.map((g) => g.toNonIndexed()));
  for (const g of parts) g.dispose();
  if (!geom) return null;
  geom.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), hub.axis));
  geom.translate(hub.start.x, hub.start.y, hub.start.z);
  return geom;
}

/**
 * The pipe running straight through a joint, as `[arriving duct, leaving duct]`, or `null` if none does.
 *
 * Used to rejoin a tee's through pipe when the tee is deleted, so only its branch goes.
 */
export function throughPipe(placement: JointPlacement): [string, string] | null {
  const limbs = placement.limbs;
  const arriving = (l: JointLimb) => l.dir.dot(placement.axis) >= 0;
  for (let i = 0; i < limbs.length; i++) {
    const g = continuationOf(limbs, i);
    if (g === i) continue;
    const a = limbs[g]!;
    const b = limbs[i]!;
    if (!a.duct || !b.duct) continue;
    // One arriving and one leaving, told apart by which way each points into the joint.
    if (arriving(a) === arriving(b)) continue;
    return arriving(a) ? [a.duct, b.duct] : [b.duct, a.duct];
  }
  return null;
}

/**
 * The earlier limb that limb `i` simply continues, or `i` itself.
 *
 * A continuation is a limb meeting another at the same point, heading the opposite way into the joint,
 * at the same size: the two halves of a pipe that a branch was teed into.
 */
function continuationOf(limbs: JointLimb[], i: number): number {
  const limb = limbs[i]!;
  for (let j = 0; j < i; j++) {
    const other = limbs[j]!;
    if (
      limb.dir.dot(other.dir) < -Math.cos((10 * Math.PI) / 180) &&
      limb.point.distanceTo(other.point) < Math.min(limb.radius, other.radius) * 0.25 &&
      Math.abs(limb.radius - other.radius) < Math.max(limb.radius, other.radius) * 0.05
    ) {
      return continuationOf(limbs, j);
    }
  }
  return i;
}

export class JointMesh {
  readonly group = new THREE.Group();

  private mesh: THREE.Mesh | null = null;
  private geometry: THREE.BufferGeometry | null = null;
  private readonly material: THREE.MeshStandardMaterial;

  constructor() {
    this.material = new THREE.MeshStandardMaterial({
      color: 0x9aa0a8,
      metalness: 0.66,
      roughness: 0.36,
    });
  }

  /** What a pointer ray should be tested against to pick this joint, or `null` if it drew nothing. */
  get pickTarget(): THREE.Mesh | null {
    return this.mesh;
  }

  /** Tint the joint to show it is selected, in the same green as the editor's selected handles. */
  setSelected(on: boolean): void {
    this.material.emissive.setHex(on ? 0x2f6b3a : 0x000000);
  }

  rebuild(placement: JointPlacement | undefined): void {
    this.clear();
    if (!placement) return;
    const geom = buildJointGeometry(placement);
    if (!geom) return;
    this.geometry = geom;
    const mesh = new THREE.Mesh(geom, this.material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    this.mesh = mesh;
    this.group.add(mesh);
  }

  boundingBox(): THREE.Box3 {
    const box = new THREE.Box3();
    if (this.mesh) box.expandByObject(this.mesh);
    return box;
  }

  private clear(): void {
    if (this.mesh) this.group.remove(this.mesh);
    this.geometry?.dispose();
    this.mesh = null;
    this.geometry = null;
  }

  dispose(): void {
    this.clear();
    this.material.dispose();
  }
}
