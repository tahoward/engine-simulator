/**
 * Junctions in the scene: where pipes meet.
 *
 * What the app draws at a junction is a `JointMesh`: a small sphere where the pipes meet, hidden until the
 * junction is selected and there to pick it by. Pipes that join merge in smooth bends of their own, so no
 * fitting is drawn.
 */

import * as THREE from 'three';

/** One pipe arriving at, or leaving, a joint. */
export interface JointLimb {
  /** Where the pipe's centreline meets the joint. */
  point: THREE.Vector3;
  /** Unit direction *into* the joint. */
  dir: THREE.Vector3;
  radius: number;
}

export interface JointPlacement {
  /** Where the limbs meet. */
  centre: THREE.Vector3;
  /** The direction flow leaves by. A route drawn from the junction starts off along it. */
  axis: THREE.Vector3;
  limbs: JointLimb[];
}

/** How much wider than the widest pipe at it a junction's mark is. */
const JOINT_MARK_RATIO = 1.35;

/**
 * A junction's mark, not drawn: pipes that join merge in smooth bends of their own, so the mark is only
 * there to be picked. It shows, see-through and green, while the junction is selected.
 */
export class JointMesh {
  readonly group = new THREE.Group();

  private mesh: THREE.Mesh | null = null;
  private geometry: THREE.BufferGeometry | null = null;
  private readonly material: THREE.MeshStandardMaterial;
  private selected = false;

  constructor() {
    this.material = new THREE.MeshStandardMaterial({
      color: 0x8cff9e,
      emissive: 0x2f6b3a,
      metalness: 0.2,
      roughness: 0.6,
      transparent: true,
      opacity: 0.45,
      depthWrite: false,
    });
  }

  /** What a pointer ray should be tested against to pick this joint, or `null` if it drew nothing. */
  get pickTarget(): THREE.Mesh | null {
    return this.mesh;
  }

  /** Show the joint while it is selected, in the same green as the editor's selected handles. */
  setSelected(on: boolean): void {
    this.selected = on;
    if (this.mesh) this.mesh.visible = on;
  }

  /**
   * A small sphere where the pipes meet, a little wider than the widest of them: all there is to pick a
   * junction by, and what shows when it is selected. Only a sphere at the centre, however spread apart the
   * pipes arrive, so that it does not catch the clicks meant for the pipes around it.
   */
  rebuild(placement: JointPlacement | undefined): void {
    this.clear();
    if (!placement || placement.limbs.length < 2) return;
    let radius = 0;
    for (const limb of placement.limbs) radius = Math.max(radius, limb.radius);
    const geom = new THREE.SphereGeometry(Math.max(radius, 0.01) * JOINT_MARK_RATIO, 24, 16);
    this.geometry = geom;
    const mesh = new THREE.Mesh(geom, this.material);
    mesh.position.copy(placement.centre);
    // Hidden, but still there to pick: a ray tests an object whether or not it is drawn.
    mesh.visible = this.selected;
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
