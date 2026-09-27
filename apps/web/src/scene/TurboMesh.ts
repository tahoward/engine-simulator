/**
 * A turbocharger as it is drawn: the turbine's scroll in cast iron with its inlet flange and axial outlet,
 * the bearing housing, and the compressor's alloy housing on the other end of the shaft.
 *
 * Built in the turbo's own frame, the shaft along +x with the compressor that way, and turned into the
 * world by its mount's rotation, the same frame `turboPorts` puts the flanges in, so the pipes drawn to them
 * meet the mesh.
 */

import * as THREE from 'three';

import type { Quat } from '../model/exhaustGraph.js';
import type { Vec3 } from '../model/geometry.js';
import type { TurboSize } from '../model/turbo.js';

const IRON = { color: 0x5b5d61, metalness: 0.55, roughness: 0.55 };
const ALLOY = { color: 0xb9bdc3, metalness: 0.7, roughness: 0.32 };

export class TurboMesh {
  readonly group = new THREE.Group();

  private readonly iron: THREE.MeshStandardMaterial;
  private readonly alloy: THREE.MeshStandardMaterial;
  private readonly geometries: THREE.BufferGeometry[] = [];
  private readonly body = new THREE.Group();

  /** `ghost` draws it see-through, for one being placed. */
  constructor(ghost = false) {
    const look = ghost ? { transparent: true, opacity: 0.45, depthWrite: false } : {};
    this.iron = new THREE.MeshStandardMaterial({ ...IRON, ...look });
    this.alloy = new THREE.MeshStandardMaterial({ ...ALLOY, ...look });
    this.group.add(this.body);
  }

  /** What a pointer ray should be tested against to pick this turbo, recursively. */
  get pickTarget(): THREE.Object3D {
    return this.body;
  }

  /** Tint it to show it is selected, in the green the editor's selected handles use. */
  setSelected(on: boolean): void {
    const glow = on ? 0x2f6b3a : 0x000000;
    this.iron.emissive.setHex(glow);
    this.alloy.emissive.setHex(glow);
  }

  /** Put it at `position`, turned by `rotation`, a unit quaternion `[x, y, z, w]`. */
  place(position: Vec3, rotation: Quat): void {
    this.group.position.set(...position);
    this.group.quaternion.set(...rotation);
  }

  /** Build its geometry for turbos of `size`. */
  rebuild(size: TurboSize): void {
    this.clear();
    const s = size.scroll;
    const d = size.depth;
    const add = (geom: THREE.BufferGeometry, mat: THREE.Material, at: Vec3, turn?: THREE.Euler) => {
      this.geometries.push(geom);
      const mesh = new THREE.Mesh(geom, mat);
      mesh.position.set(...at);
      if (turn) mesh.rotation.copy(turn);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.body.add(mesh);
    };
    // A cylinder stands along +y; these lay it along the shaft, or along the inlet's +z.
    const alongX = new THREE.Euler(0, 0, Math.PI / 2);
    const alongZ = new THREE.Euler(Math.PI / 2, 0, 0);
    // A torus lies in its xy plane; this stands it round the shaft.
    const roundX = new THREE.Euler(0, Math.PI / 2, 0);

    // Turbine: the scroll round a drum, the inlet neck and flange on its side, the outlet out of its face.
    add(new THREE.TorusGeometry(0.62 * s, 0.36 * s, 16, 40), this.iron, [0, 0, 0], roundX);
    add(new THREE.CylinderGeometry(0.62 * s, 0.62 * s, d, 32), this.iron, [0, 0, 0], alongX);
    add(new THREE.CylinderGeometry(0.3 * s, 0.34 * s, 0.4 * s, 20), this.iron, [0, 0, -0.9 * s], alongZ);
    add(new THREE.BoxGeometry(0.8 * s, 0.8 * s, 0.1 * s), this.iron, [0, 0, -1.1 * s]);
    const outlet = 0.5 * d + 0.45 * s;
    const bore = Math.max(size.outletDia / 2 + 0.004, 0.25 * s);
    add(new THREE.CylinderGeometry(bore, bore, 0.45 * s, 24), this.iron, [-(outlet - 0.225 * s), 0, 0], alongX);
    add(new THREE.CylinderGeometry(bore * 1.3, bore * 1.3, 0.06 * s, 24), this.iron, [-outlet + 0.03 * s, 0, 0], alongX);

    // Bearing housing, then the compressor: a bigger, lighter scroll, and its inlet facing away.
    const centre = 0.5 * d + 0.35 * s;
    add(new THREE.CylinderGeometry(0.3 * s, 0.3 * s, 0.7 * s, 20), this.iron, [centre, 0, 0], alongX);
    const comp = centre + 0.35 * s + 0.4 * s;
    add(new THREE.TorusGeometry(0.66 * s, 0.34 * s, 16, 40), this.alloy, [comp, 0, 0], roundX);
    add(new THREE.CylinderGeometry(0.66 * s, 0.66 * s, 0.8 * s, 32), this.alloy, [comp, 0, 0], alongX);
    add(new THREE.CylinderGeometry(0.42 * s, 0.38 * s, 0.5 * s, 24), this.alloy, [comp + 0.6 * s, 0, 0], alongX);
    // The compressor's outlet, tangential, pointing up.
    add(new THREE.CylinderGeometry(0.2 * s, 0.2 * s, 0.6 * s, 16), this.alloy, [comp, 0.9 * s, 0.3 * s]);
  }

  boundingBox(): THREE.Box3 {
    return new THREE.Box3().expandByObject(this.body);
  }

  private clear(): void {
    this.body.clear();
    for (const g of this.geometries) g.dispose();
    this.geometries.length = 0;
  }

  dispose(): void {
    this.clear();
    this.iron.dispose();
    this.alloy.dispose();
  }
}
