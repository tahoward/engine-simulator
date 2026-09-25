/**
 * The animated cutaway engine, for every cylinder count the spec allows: one cylinder up to a V8.
 *
 * Every moving part is positioned from the *same* functions the physics uses
 * (`pistonPosition`, `valveLift`), driven by each cylinder's own crank angle from the snapshot.
 * So this is not an animation that approximately matches the sound — it is a readout of the
 * simulation state. If a piston looks wrong, the gas law is wrong too.
 *
 * Layout: crank along Z at the origin, rotating in the XY plane. A bank's cylinder axis lies
 * along +Y for bank 0, so its TDC is at crank angle 0 exactly as the physics assumes; bank 1
 * sits `-vAngle` round, which is precisely what makes its own crank angle *lead* by `vAngle`.
 * The whole engine is then rotated by `vAngle/2` so a V straddles vertical.
 *
 * Cylinders are placed along Z by **crankpin**, from `crankPins`, so two cylinders sharing a pin
 * sit in the same plane and are driven by the same throw — which is what they do. That also
 * means the drawn crank shows the real arrangement: pins at 90-degree intervals for a crossplane
 * V8, all in one plane for a flatplane, a single shared pin for a V-twin.
 *
 * The cutaway sections each cylinder at *its own* Z so all of them are open to view, not just
 * the frontmost.
 */

import * as THREE from 'three';
import {
  type BankSnapshot,
  type EngineSpec,
  clearanceVolume,
  crankPins,
  exhaustPortDiameter,
  cylinderSpacing,
  firingPlan,
  physicalBank,
  physicalBankCount,
  pistonPosition,
} from '../model/spec.js';
import { exhaustPortOf } from '../model/geometry.js';
import { valveLift } from '../audio/worklet/valve.js';

const STEEL = { color: 0x8d949e, metalness: 0.92, roughness: 0.34 };
const CAST = { color: 0x5c626c, metalness: 0.55, roughness: 0.66 };
const ALLOY = { color: 0xb9c0c9, metalness: 0.85, roughness: 0.28 };

export interface ExhaustPort {
  /** World position where the pipe begins. */
  position: THREE.Vector3;
  /** Unit direction the pipe leaves in. */
  direction: THREE.Vector3;
}

interface CylinderMesh {
  /** Rotated to the cylinder's bank axis and translated along Z; holds the moving parts. */
  group: THREE.Group;
  piston: THREE.Group;
  exValves: THREE.Group[];
  inValves: THREE.Group[];
  /** Which side of the head, in the cylinder's own frame, the exhaust comes out of: +1 is +X. */
  exhaustSide: number;
  /** Lives outside `group`: it spans from the crankpin to this cylinder's piston. */
  rod: THREE.Mesh;
  flame: THREE.Mesh;
  /** Bank axis rotation about Z, radians. */
  rotation: number;
  /** Position along the crank, m. */
  z: number;
  /** Angle of this cylinder's crankpin round the shaft, radians. */
  pinAngle: number;
  /** Section plane for this cylinder, at its own Z. */
  clip: THREE.Plane;
}

export class EngineMesh {
  readonly group = new THREE.Group();

  private spec: EngineSpec;
  private readonly crank = new THREE.Group();
  private readonly cyls: CylinderMesh[] = [];

  private deckY = 0;
  private crownOffset = 0;
  private readonly valveTilt = 0.21;
  /** Centre-to-centre cylinder spacing along the crank, m. */
  private spacing = 0;

  constructor(
    spec: EngineSpec,
    private readonly clipPlane: THREE.Plane,
  ) {
    this.spec = { ...spec };
    this.group.add(this.crank);
    this.rebuild();
  }

  // -------------------------------------------------------------------------
  // Construction
  // -------------------------------------------------------------------------

  private computeDerived(): void {
    const s = this.spec;
    const a = s.stroke / 2;
    // Real compression heights are roughly a third of the bore.
    this.crownOffset = s.bore * 0.34;
    const boreArea = (Math.PI * s.bore * s.bore) / 4;
    const clearanceHeight = clearanceVolume(s) / boreArea;
    // Deck height so the TDC gap above the crown is exactly the clearance height, which is
    // what makes the drawn compression ratio the real one.
    this.deckY = a + s.rodLength + this.crownOffset + clearanceHeight;
    this.spacing = cylinderSpacing(s);
  }

  private rebuild(): void {
    this.computeDerived();

    for (const c of this.cyls) {
      disposeTree(c.group);
      disposeTree(c.rod);
      this.group.remove(c.group, c.rod);
    }
    this.cyls.length = 0;
    disposeChildren(this.crank);

    const plan = firingPlan(this.spec);
    const pins = crankPins(this.spec);
    // Centre the engine on the origin however many pins there are.
    const zOf = (pin: number) => (pin - (pins.length - 1) / 2) * this.spacing;

    // One throw per pin entry; a split pin puts more than one pin on it, one per angle.
    this.buildCrank(pins.map((p, i) => ({ angles: [...new Set(p.angles)], z: zOf(i) })));

    for (let pin = 0; pin < pins.length; pin++) {
      pins[pin]!.cylinders.forEach((cyl, k) => {
        const bank = plan.banks[cyl]!;
        /**
         * Exhaust on the outside of the vee for both banks, intake in the valley, as a V is built.
         *
         * One side for every cylinder would put bank 0's exhaust into the valley and bank 1's outside, so the
         * two banks' pipes would come off the same side of the engine. Straddling the vertical leans bank 0 towards
         * -X, so its outside is -X; bank 1 leans the other way. An inline engine keeps +X, and so does a
         * parallel twin, whose two firing banks are one physical bank under one head.
         */
        const exhaustSide = physicalBankCount(this.spec) > 1 && physicalBank(this.spec, cyl) === 0 ? -1 : 1;
        this.cyls[cyl] = this.buildCylinder(bank, zOf(pin), pins[pin]!.angles[k]!, exhaustSide);
      });
    }

    // Straddle vertical, so a V looks like a V rather than leaning.
    this.group.rotation.z = ((plan.bankCount > 1 ? this.spec.vAngle / 2 : 0) * Math.PI) / 180;
  }

  private buildCrank(throws: Array<{ angles: number[]; z: number }>): void {
    // A split throw's pins sit side by side along the shaft, each half as wide.
    const pins = throws.flatMap(({ angles, z }) =>
      angles.map((angle, k) => ({
        angle,
        z: z + (angles.length > 1 ? (k - (angles.length - 1) / 2) * 0.016 : 0),
        width: angles.length > 1 ? 0.015 : 0.03,
      })),
    );
    const s = this.spec;
    const a = s.stroke / 2;

    const journal = new THREE.Mesh(
      new THREE.CylinderGeometry(0.019, 0.019, this.spacing * throws.length + 0.06, 18),
      new THREE.MeshStandardMaterial(STEEL),
    );
    journal.rotation.x = Math.PI / 2;
    this.crank.add(journal);

    for (const { angle, z, width } of pins) {
      // Local pin position: rotating the crank by -theta must carry this to
      // a*(sin(theta - angle), cos(theta - angle)), which is TDC when theta == angle.
      const phi = (angle * Math.PI) / 180;
      const x = -a * Math.sin(phi);
      const y = a * Math.cos(phi);

      const web = new THREE.Mesh(
        new THREE.CylinderGeometry(a * 1.5, a * 1.5, 0.022, 26, 1, false, Math.PI * 0.62, Math.PI * 1.76),
        new THREE.MeshStandardMaterial(CAST),
      );
      web.rotation.x = Math.PI / 2;
      web.rotation.z = -phi;
      web.position.z = z;
      web.castShadow = true;
      this.crank.add(web);

      // One big-end journal per pin, shared by the cylinders hanging off it — which is what
      // ties a V-twin's firing interval to its V angle.
      const pin = new THREE.Mesh(
        new THREE.CylinderGeometry(0.0135, 0.0135, width, 16),
        new THREE.MeshStandardMaterial(STEEL),
      );
      pin.rotation.x = Math.PI / 2;
      pin.position.set(x, y, z);
      pin.castShadow = true;
      this.crank.add(pin);
    }
  }

  private buildCylinder(
    bank: number,
    z: number,
    pinAngleDeg: number,
    exhaustSide: number,
  ): CylinderMesh {
    const s = this.spec;
    const rotation = bank === 0 ? 0 : -(s.vAngle * Math.PI) / 180;
    const group = new THREE.Group();
    group.rotation.z = rotation;
    this.group.add(group);

    // Its own section plane, at its own Z, so every cylinder is open to view.
    const clip = new THREE.Plane(this.clipPlane.normal.clone(), this.clipPlane.constant);

    // --- piston ---
    const piston = new THREE.Group();
    const r = (s.bore / 2) * 0.985;
    const body = new THREE.Mesh(
      new THREE.CylinderGeometry(r, r, this.crownOffset * 1.25, 30),
      new THREE.MeshStandardMaterial(ALLOY),
    );
    body.castShadow = true;
    piston.add(body);

    const ringMat = new THREE.MeshStandardMaterial({
      color: 0x2c2f35,
      metalness: 0.7,
      roughness: 0.5,
    });
    for (let i = 0; i < 3; i++) {
      const ring = new THREE.Mesh(new THREE.TorusGeometry(r, 0.0016, 6, 40), ringMat);
      ring.rotation.x = Math.PI / 2;
      ring.position.y = this.crownOffset * 0.42 - i * 0.006;
      piston.add(ring);
    }
    const wrist = new THREE.Mesh(
      new THREE.CylinderGeometry(0.0095, 0.0095, r * 1.7, 14),
      new THREE.MeshStandardMaterial(STEEL),
    );
    wrist.rotation.x = Math.PI / 2;
    wrist.position.y = -this.crownOffset * 0.1;
    piston.add(wrist);
    group.add(piston);

    // --- valves ---
    const exValves = this.buildValves(s.exValveDia, s.exValveCount, exhaustSide, true);
    const inValves = this.buildValves(s.inValveDia, s.inValveCount, -exhaustSide, false);
    group.add(...exValves, ...inValves);

    // --- block, in this cylinder's frame ---
    this.buildCastings(group, clip);

    // --- rod: world space, from the pin to this cylinder's piston ---
    const rod = new THREE.Mesh(
      new THREE.CylinderGeometry(0.009, 0.012, 1, 12),
      new THREE.MeshStandardMaterial(STEEL),
    );
    rod.castShadow = true;
    this.group.add(rod);

    const flame = new THREE.Mesh(
      new THREE.SphereGeometry(1, 20, 14),
      new THREE.MeshBasicMaterial({
        color: 0xffb347,
        transparent: true,
        opacity: 0,
        depthWrite: false,
        blending: THREE.AdditiveBlending,
      }),
    );
    flame.scale.setScalar(s.bore * 0.42);
    group.add(flame);

    // Everything in `group` is in the bank's rotated frame, where the cylinder's own Z offset
    // is still along Z, so the translation can be applied to the group as a whole.
    group.position.z = z;

    return {
      group,
      piston,
      exValves,
      inValves,
      exhaustSide,
      rod,
      flame,
      rotation,
      z,
      pinAngle: (pinAngleDeg * Math.PI) / 180,
      clip,
    };
  }

  /** One side's valves: one on the cylinder's mid-plane, or a pair either side of it along the crank. */
  private buildValves(dia: number, count: number, sign: number, exhaust: boolean): THREE.Group[] {
    const out: THREE.Group[] = [];
    for (let i = 0; i < count; i++) {
      const z = count === 1 ? 0 : (i === 0 ? -1 : 1) * (dia / 2 + 0.001);
      out.push(this.buildValve(dia, sign, exhaust, z));
    }
    return out;
  }

  private buildValve(dia: number, sign: number, exhaust: boolean, z: number): THREE.Group {
    const s = this.spec;
    const group = new THREE.Group();

    const head = new THREE.Mesh(
      new THREE.CylinderGeometry(dia / 2, dia / 2 - 0.0035, 0.007, 24),
      new THREE.MeshStandardMaterial({
        // Exhaust valves run hot enough to discolour; intake valves stay bright.
        color: exhaust ? 0x9b7a63 : 0xc3cad3,
        metalness: 0.9,
        roughness: exhaust ? 0.45 : 0.25,
      }),
    );
    head.castShadow = true;
    group.add(head);

    const stem = new THREE.Mesh(
      new THREE.CylinderGeometry(0.0038, 0.0038, s.bore * 0.85, 12),
      new THREE.MeshStandardMaterial(STEEL),
    );
    stem.position.y = s.bore * 0.425;
    group.add(stem);

    const spring = new THREE.Mesh(
      new THREE.TorusGeometry(0.011, 0.0018, 6, 22),
      new THREE.MeshStandardMaterial({ color: 0x777d86, metalness: 0.8, roughness: 0.4 }),
    );
    spring.rotation.x = Math.PI / 2;
    spring.position.y = s.bore * 0.72;
    group.add(spring);

    group.rotation.z = -sign * this.valveTilt;
    group.position.set(sign * s.bore * 0.24, this.deckY, z);
    group.userData.z = z;
    return group;
  }

  /** Liner, fins and head for one cylinder, in that cylinder's rotated frame. */
  private buildCastings(parent: THREE.Group, clip: THREE.Plane): void {
    const s = this.spec;
    const r = s.bore / 2;
    const linerTop = this.deckY;
    const linerBottom = s.stroke / 2 + s.rodLength - s.stroke - this.crownOffset * 0.2;
    const linerHeight = linerTop - linerBottom;

    // DoubleSide plus the section plane gives a proper cutaway rather than a hole you can
    // see straight through.
    const linerMat = new THREE.MeshStandardMaterial({
      ...CAST,
      side: THREE.DoubleSide,
      clippingPlanes: [clip],
      clipShadows: true,
    });
    const liner = new THREE.Mesh(
      new THREE.CylinderGeometry(r * 1.13, r * 1.13, linerHeight, 40, 1, true),
      linerMat,
    );
    liner.position.y = linerBottom + linerHeight / 2;
    liner.castShadow = true;
    liner.receiveShadow = true;
    parent.add(liner);

    const finMat = new THREE.MeshStandardMaterial({ ...CAST, clippingPlanes: [clip] });
    const finCount = 7;
    for (let i = 0; i < finCount; i++) {
      const t = (i + 0.6) / (finCount + 0.4);
      const fin = new THREE.Mesh(new THREE.CylinderGeometry(r * 1.5, r * 1.5, 0.005, 36), finMat);
      fin.position.y = linerBottom + t * linerHeight;
      fin.castShadow = true;
      parent.add(fin);
    }

    const headHeight = s.bore * 0.52;
    const headMat = new THREE.MeshStandardMaterial({
      color: 0x6b7280,
      metalness: 0.6,
      roughness: 0.55,
      clippingPlanes: [clip],
      clipShadows: true,
    });
    // Slightly narrower than the spacing so adjacent heads read as separate castings.
    const head = new THREE.Mesh(
      new THREE.BoxGeometry(s.bore * 2.3, headHeight, Math.min(s.bore * 1.5, this.spacing * 0.92)),
      headMat,
    );
    head.position.y = this.deckY + headHeight / 2;
    head.castShadow = true;
    head.receiveShadow = true;
    parent.add(head);
  }

  // -------------------------------------------------------------------------
  // Updates
  // -------------------------------------------------------------------------

  /** Rebuild for new geometry. Cheap enough to call on every slider change. */
  setSpec(spec: EngineSpec): void {
    const structural =
      spec.bore !== this.spec.bore ||
      spec.stroke !== this.spec.stroke ||
      spec.rodLength !== this.spec.rodLength ||
      spec.compressionRatio !== this.spec.compressionRatio ||
      spec.exValveDia !== this.spec.exValveDia ||
      spec.inValveDia !== this.spec.inValveDia ||
      spec.exValveCount !== this.spec.exValveCount ||
      spec.inValveCount !== this.spec.inValveCount ||
      spec.cylinders !== this.spec.cylinders ||
      spec.crankType !== this.spec.crankType ||
      spec.firingOffset !== this.spec.firingOffset ||
      spec.vAngle !== this.spec.vAngle;

    this.spec = { ...spec };
    if (structural) this.rebuild();
  }

  /**
   * Pose every moving part from the snapshot.
   *
   * Driven by each cylinder's own crank angle rather than a single global one, so the phase
   * relationship on screen is whatever the physics is actually running.
   */
  update(banks: BankSnapshot[], burnGlow: number[]): void {
    const s = this.spec;
    const a = s.stroke / 2;

    // The crank follows cylinder 0, whose axis is +Y and whose TDC is at angle 0.
    const th0 = ((banks[0]?.crankAngle ?? 0) * Math.PI) / 180;
    this.crank.rotation.z = -th0;
    // Follow the shared cutaway toggle: off, the Viewer pushes its plane out to 100.
    const sectioned = this.clipPlane.constant < 50;

    for (let i = 0; i < this.cyls.length; i++) {
      const mesh = this.cyls[i]!;
      const snap = banks[Math.min(i, banks.length - 1)]!;

      mesh.clip.normal.copy(this.clipPlane.normal);
      mesh.clip.constant = sectioned ? this.clipPlane.constant + mesh.z : this.clipPlane.constant;

      const pinY = pistonPosition(s, snap.crankAngle);
      mesh.piston.position.set(0, pinY, 0);

      // Rod endpoints in world space: this cylinder's pin, and its piston rotated into its own
      // bank axis. Both carry the cylinder's Z.
      const bigEnd = new THREE.Vector3(
        a * Math.sin(th0 - mesh.pinAngle),
        a * Math.cos(th0 - mesh.pinAngle),
        mesh.z,
      );
      const smallEnd = new THREE.Vector3(0, pinY, 0).applyAxisAngle(AXIS_Z, mesh.rotation);
      smallEnd.z = mesh.z;
      const mid = bigEnd.clone().add(smallEnd).multiplyScalar(0.5);
      const axis = smallEnd.clone().sub(bigEnd);
      mesh.rod.position.copy(mid);
      mesh.rod.scale.set(1, axis.length(), 1);
      mesh.rod.quaternion.setFromUnitVectors(AXIS_Y, axis.normalize());

      const exLift = valveLift(snap.crankAngle, s.evo, s.evc, s.maxLift);
      const inLift = valveLift(snap.crankAngle, s.ivo, s.ivc, s.maxLift);
      for (const v of mesh.exValves) this.poseValve(v, mesh.exhaustSide, exLift);
      for (const v of mesh.inValves) this.poseValve(v, -mesh.exhaustSide, inLift);

      const mat = mesh.flame.material as THREE.MeshBasicMaterial;
      mat.opacity = Math.min(burnGlow[i] ?? 0, 1) * 0.75;
      mesh.flame.position.set(0, this.deckY - s.bore * 0.2, 0);
      mesh.flame.visible = mat.opacity > 0.01;
    }
  }

  /**
   * Open a valve by `lift`: down its own stem, away from the seat.
   *
   * Taken from the stem's axis as the valve is actually tilted, rather than from a separate formula for
   * the same angle, so the sideways part cannot disagree with the tilt. With its sign the wrong way round
   * a valve leaning out at the top would slide outward as it opened instead of inward, off its own axis.
   */
  private poseValve(group: THREE.Group, sign: number, lift: number): void {
    const stem = new THREE.Vector3(0, 1, 0).applyAxisAngle(AXIS_Z, group.rotation.z);
    group.position
      .set(sign * this.spec.bore * 0.24, this.deckY, group.userData.z as number)
      .addScaledVector(stem, -lift);
  }

  /** Where cylinder `index`'s exhaust pipe attaches, in world space. */
  exhaustPort(index = 0): ExhaustPort {
    // From the model, so an exhaust compiled to fit the engine fits the engine as drawn.
    const cylinder = Math.max(Math.min(index, this.cyls.length - 1), 0);
    const port = exhaustPortOf(this.spec, cylinder);
    return {
      position: new THREE.Vector3(...port.position),
      direction: new THREE.Vector3(...port.direction),
    };
  }

  get bankCount(): number {
    return this.cyls.length;
  }

  /**
   * How cylinder `i` is currently posed. For tests and diagnostics.
   *
   * `rodLength` is the length the rod is actually *drawn* at, taken from its scale. It has to
   * equal `spec.rodLength` at every crank angle of every cylinder, which is the one assertion
   * that catches a wrong big-end position: get the pin angle, the bank rotation or the Z offset
   * wrong and the rod stretches to reach, rather than visibly breaking.
   */
  pose(i: number): {
    z: number;
    bankRotation: number;
    pistonY: number;
    rodLength: number;
    pinAngleDeg: number;
  } {
    const c = this.cyls[i]!;
    return {
      z: c.z,
      bankRotation: c.rotation,
      pistonY: c.piston.position.y,
      rodLength: c.rod.scale.y,
      pinAngleDeg: (c.pinAngle * 180) / Math.PI,
    };
  }

  /** Diameter the port should hand off to the first pipe segment, m. */
  get portDiameter(): number {
    return exhaustPortDiameter(this.spec) * 0.95;
  }
}

const AXIS_Z = new THREE.Vector3(0, 0, 1);
const AXIS_Y = new THREE.Vector3(0, 1, 0);

/** Empty a group, releasing GPU buffers. Materials are shared, so only geometry. */
function disposeChildren(group: THREE.Group): void {
  for (const child of [...group.children]) {
    disposeTree(child);
    group.remove(child);
  }
}

function disposeTree(node: THREE.Object3D): void {
  node.traverse((o) => {
    if (o instanceof THREE.Mesh) o.geometry.dispose();
  });
}
