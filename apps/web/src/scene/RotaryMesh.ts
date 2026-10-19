/**
 * A Wankel, animated: each rotor turning in its trochoid housing on the eccentric shaft, posed from the
 * shaft's angle the simulation runs each rotor at.
 *
 * Built and turned in the housing's own frame, where the trochoid is `e (cos 3a, sin 3a) + R (cos a, sin a)`
 * and its long axis lies along x, then the whole frame is turned a quarter turn so its long axis stands
 * upright, as `trochoidAt` places the housing. At the shaft's angle `theta` the rotor's centre is `e` out from
 * the shaft along `theta`, and the rotor is turned `theta / 3`: so each apex is always on the trochoid, as
 * it rides round it. Each chamber is at its firing top dead centre, its flank facing the waist on -x, at
 * `theta` 90 degrees in this frame; the simulation's shaft angle is 0 there for the rotor's first chamber.
 */

import * as THREE from 'three';
import { type EngineSpec, cylinderSpacing, cylinderZ, firingPlan } from '../model/spec.js';
import {
  END_HOUSING,
  END_JOURNAL,
  HOUSING_WALL,
  ROTARY_SHAFT_RADIUS,
  engineShell,
  exhaustPortOf,
  intakePortOf,
  trochoidAt,
} from '../model/geometry.js';
import { exhaustPortDiameter } from '../model/spec.js';

const STEEL = { color: 0x8d949e, metalness: 0.92, roughness: 0.34 };
/** The rotor housings' trochoid walls, a little see-through so the rotors show from any side. */
const HOUSING = { color: 0x5c626c, metalness: 0.55, roughness: 0.66, transparent: true, opacity: 0.55, depthWrite: false };
/** The side and end housings: as see-through as a piston engine's outline. */
const SIDE_HOUSING = { color: 0xa9b1bb, metalness: 0.2, roughness: 0.6, transparent: true, opacity: 0.13, depthWrite: false };
const ROTOR = { color: 0xb9c0c9, metalness: 0.85, roughness: 0.28 };
const GEAR = { color: 0xb8894f, metalness: 0.7, roughness: 0.4 };
const PORT = { color: 0xff8c42, emissive: 0x6a2a00, metalness: 0.5, roughness: 0.4 };
const INTAKE_PORT = { color: 0x4aa8ff, emissive: 0x0b2d5c, metalness: 0.5, roughness: 0.4 };

/** How far the rotor's flanks stand in from the housing at their nearest, m, and its sides from the side housings. */
const ROTOR_CLEARANCE = 0.0015;
/** The shaft angle, in the housing's frame, at which a chamber fires. */
const FIRING_THETA = 90;

interface Rotor {
  rotor: THREE.Group;
  lobe: THREE.Mesh;
  flame: THREE.Mesh;
  /** Its lobe's lead round the shaft over the first rotor's, degrees. */
  phase: number;
}

/**
 * The trochoid's points, `steps` of them, in the housing's frame, `out` m out from it along its normal.
 */
function trochoid(spec: EngineSpec, steps: number, out = 0): THREE.Vector2[] {
  const points: THREE.Vector2[] = [];
  for (let k = 0; k < steps; k++) {
    // `trochoidAt` is the housing turned upright; turn it back.
    const { point, normal } = trochoidAt(spec, (k / steps) * 360);
    const [x, y] = [point[0] + normal[0] * out, point[1] + normal[1] * out];
    points.push(new THREE.Vector2(y, -x));
  }
  return points;
}

/**
 * The rotor's outline in its own frame, `steps` points round it, apex 0 along +x: the inner envelope of the
 * trochoid as the rotor sees it turning, so the biggest rotor that never touches the housing, less
 * `ROTOR_CLEARANCE`. The trochoid is followed through a whole turn of the rotor in the rotor's frame, and
 * for each direction out of the rotor's centre the nearest it comes is the most the rotor may reach.
 */
export function rotorOutline(spec: EngineSpec, steps = 180): THREE.Vector2[] {
  const housing = trochoid(spec, 720);
  const e = spec.eccentricity;
  const reach = new Array<number>(steps).fill(Infinity);
  for (let k = 0; k < 1080; k += 2) {
    const theta = (k * Math.PI) / 180;
    const [cx, cy] = [e * Math.cos(theta), e * Math.sin(theta)];
    const turn = -theta / 3;
    const [c, s] = [Math.cos(turn), Math.sin(turn)];
    for (const p of housing) {
      const [x, y] = [p.x - cx, p.y - cy];
      const [rx, ry] = [x * c - y * s, x * s + y * c];
      const angle = (Math.atan2(ry, rx) + 2 * Math.PI) % (2 * Math.PI);
      const bin = Math.round((angle / (2 * Math.PI)) * steps) % steps;
      reach[bin] = Math.min(reach[bin]!, Math.hypot(rx, ry));
    }
  }
  return reach.map((r, i) => {
    const angle = (i / steps) * 2 * Math.PI;
    // The apexes are the outline's corners, where a direction's nearest is the trochoid itself: no
    // direction a little either side of one, taken with it, may cut it short.
    const apex = (i * 3) % steps === 0;
    const d = (apex ? spec.rotorRadius : Math.min(r, spec.rotorRadius)) - ROTOR_CLEARANCE;
    return new THREE.Vector2(d * Math.cos(angle), d * Math.sin(angle));
  });
}

export class RotaryMesh {
  readonly group = new THREE.Group();
  /** The housing's frame, its long axis along x: turned a quarter turn to stand it upright. */
  private readonly frame = new THREE.Group();
  private readonly shaft = new THREE.Group();
  private readonly rotors: Rotor[] = [];

  constructor(private readonly spec: EngineSpec) {
    this.frame.rotation.z = Math.PI / 2;
    this.group.add(this.frame);
    this.frame.add(this.shaft);
    this.build();
  }

  private build(): void {
    const s = this.spec;
    const width = s.rotorWidth;
    const pitch = cylinderSpacing(s);
    const shell = engineShell(s);
    const housingMat = new THREE.MeshStandardMaterial(HOUSING);
    const sideMat = new THREE.MeshStandardMaterial(SIDE_HOUSING);
    const steel = new THREE.MeshStandardMaterial(STEEL);

    // The trochoid wall, the housing's own outline out to its wall's thickness, for each rotor.
    const inner = trochoid(s, 240);
    const outer = trochoid(s, 240, HOUSING_WALL);
    const ring = new THREE.Shape(outer);
    ring.holes.push(new THREE.Path([...inner].reverse()));
    const ringGeometry = new THREE.ExtrudeGeometry(ring, { depth: width, bevelEnabled: false, curveSegments: 1 });
    ringGeometry.translate(0, 0, -width / 2);
    // The side housings between and either side of the rotors: plates the housing's outline, see-through.
    const plate = new THREE.ExtrudeGeometry(new THREE.Shape(outer), { depth: 1, bevelEnabled: false, curveSegments: 1 });

    const rotorShape = new THREE.Shape(rotorOutline(s));
    // The rotor's bore, round its bearing on the eccentric lobe.
    const lobeRadius = this.lobeRadius();
    rotorShape.holes.push(new THREE.Path().absarc(0, 0, lobeRadius + 0.002, 0, Math.PI * 2, true));
    const rotorWidth = width - 2 * ROTOR_CLEARANCE;
    const rotorGeometry = new THREE.ExtrudeGeometry(rotorShape, {
      depth: rotorWidth - 0.002,
      bevelEnabled: true,
      bevelThickness: 0.001,
      bevelSize: 0.001,
      bevelSegments: 1,
      curveSegments: 24,
    });
    rotorGeometry.translate(0, 0, -(rotorWidth - 0.002) / 2);
    // Its internal gear, which meshes with the stationary one round the shaft, three times the eccentricity
    // round, and the stationary gear twice: the ratio that turns the rotor at a third of the shaft's speed.
    const ringGear = new THREE.TorusGeometry(3 * s.eccentricity + 0.004, 0.0035, 8, 48);
    const fixedGear = new THREE.CylinderGeometry(2 * s.eccentricity, 2 * s.eccentricity, width * 0.18, 36);
    fixedGear.rotateX(Math.PI / 2);

    const plan = firingPlan(s);
    for (let r = 0; r < s.cylinders; r++) {
      const z = cylinderZ(s, r);
      const housing = new THREE.Mesh(ringGeometry, housingMat);
      housing.name = 'rotor housing';
      housing.position.z = z;
      housing.renderOrder = 20;
      this.frame.add(housing);

      const rotor = new THREE.Group();
      rotor.name = 'rotor';
      rotor.position.z = z;
      const body = new THREE.Mesh(rotorGeometry, new THREE.MeshStandardMaterial(ROTOR));
      rotor.add(body);
      // An apex seal along each apex, the rotor's width.
      for (let k = 0; k < 3; k++) {
        const angle = (k * 2 * Math.PI) / 3;
        const seal = new THREE.Mesh(new THREE.BoxGeometry(0.004, 0.003, rotorWidth), steel);
        const at = s.rotorRadius - ROTOR_CLEARANCE - 0.0015;
        seal.position.set(at * Math.cos(angle), at * Math.sin(angle), 0);
        seal.rotation.z = angle;
        rotor.add(seal);
      }
      const gear = new THREE.Mesh(ringGear, new THREE.MeshStandardMaterial(GEAR));
      gear.position.z = width * 0.3;
      rotor.add(gear);
      this.frame.add(rotor);

      const fixed = new THREE.Mesh(fixedGear, new THREE.MeshStandardMaterial(GEAR));
      fixed.position.z = z + width * 0.3;
      this.frame.add(fixed);

      const lobe = new THREE.Mesh(new THREE.CylinderGeometry(lobeRadius, lobeRadius, width * 0.9, 36), steel);
      lobe.rotation.x = Math.PI / 2;
      lobe.position.z = z;
      this.frame.add(lobe);

      // The flame, in the chamber across the waist where it fires, and that chamber is long and thin.
      const flame = new THREE.Mesh(
        new THREE.SphereGeometry(0.02, 20, 12),
        new THREE.MeshBasicMaterial({ color: 0xffa040, transparent: true, opacity: 0, depthWrite: false }),
      );
      flame.scale.set(2.8, 0.6, (rotorWidth / 0.04) * 0.9);
      flame.position.set(0, s.rotorRadius - s.eccentricity - 0.011, z);
      flame.visible = false;
      this.frame.add(flame);

      this.rotors.push({ rotor, lobe, flame, phase: plan.offsets[r] ?? 0 });

      this.addPort(exhaustPortOf(s, r), exhaustPortDiameter(s) / 2, PORT);
      this.addPort(intakePortOf(s, r), s.inValveDia / 2, INTAKE_PORT);
    }
    // A side housing either side of each rotor's: the front and rear housings, and the intermediate ones.
    for (let r = 0; r <= s.cylinders; r++) {
      const edge = r === 0 || r === s.cylinders;
      const thick = edge ? END_HOUSING : pitch - width;
      const z = (r - s.cylinders / 2) * pitch + (edge ? (r === 0 ? -1 : 1) * (END_HOUSING / 2 - (pitch - width) / 2) : 0);
      const side = new THREE.Mesh(plate, sideMat);
      side.name = 'shell';
      side.scale.z = thick;
      side.position.z = z - thick / 2;
      side.renderOrder = 20;
      this.frame.add(side);
    }

    // The eccentric shaft, through every rotor and out past the end housings.
    const length = shell.length + 2 * END_JOURNAL;
    const main = new THREE.Mesh(new THREE.CylinderGeometry(ROTARY_SHAFT_RADIUS, ROTARY_SHAFT_RADIUS, length, 32), steel);
    main.rotation.x = Math.PI / 2;
    this.shaft.add(main);
    // A key down its nose, so it is seen to turn.
    const key = new THREE.Mesh(new THREE.BoxGeometry(0.006, 0.006, END_JOURNAL + 0.01), steel);
    key.position.set(ROTARY_SHAFT_RADIUS, 0, -length / 2 + END_JOURNAL / 2);
    this.shaft.add(key);
  }

  /** The eccentric lobe's radius, m: round the shaft, `e` out from it, and the rotor's bearing on it. */
  private lobeRadius(): number {
    return ROTARY_SHAFT_RADIUS + this.spec.eccentricity + 0.01;
  }

  /** A port's flange, where its pipe or runner attaches, in the scene's frame. */
  private addPort(port: { position: number[]; direction: number[] }, r: number, look: object): void {
    const flange = new THREE.Mesh(new THREE.TorusGeometry(r * 1.25, r * 0.28, 10, 32), new THREE.MeshStandardMaterial(look));
    const opening = new THREE.Mesh(
      new THREE.CircleGeometry(r * 1.05, 32),
      new THREE.MeshBasicMaterial({ color: 0x1a0d05, side: THREE.DoubleSide }),
    );
    const group = new THREE.Group();
    group.name = look === PORT ? 'exhaust port' : 'intake port';
    group.add(flange, opening);
    group.position.set(port.position[0]!, port.position[1]!, port.position[2]!);
    group.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), new THREE.Vector3(...(port.direction as [number, number, number])));
    this.group.add(group);
  }

  /**
   * Pose each rotor at its shaft angle, degrees from its first chamber's firing, and its flame as bright as
   * `burnGlow` says, rotor by rotor.
   */
  update(banks: Array<{ crankAngle: number }>, burnGlow: number[]): void {
    const e = this.spec.eccentricity;
    const first = banks[0]?.crankAngle ?? 0;
    this.shaft.rotation.z = ((first + FIRING_THETA) * Math.PI) / 180;
    this.rotors.forEach((r, i) => {
      const theta = (((banks[i]?.crankAngle ?? first - r.phase) + FIRING_THETA) * Math.PI) / 180;
      r.rotor.position.x = e * Math.cos(theta);
      r.rotor.position.y = e * Math.sin(theta);
      r.rotor.rotation.z = theta / 3;
      r.lobe.position.x = r.rotor.position.x;
      r.lobe.position.y = r.rotor.position.y;
      const mat = r.flame.material as THREE.MeshBasicMaterial;
      mat.opacity = Math.min(burnGlow[i] ?? 0, 1) * 0.75;
      r.flame.visible = mat.opacity > 0.01;
    });
  }

  /** How rotor `i` is posed: its centre, in the housing's frame, and its turn, radians. For tests. */
  pose(i: number): { x: number; y: number; turn: number } {
    const r = this.rotors[i]!.rotor;
    return { x: r.position.x, y: r.position.y, turn: r.rotation.z };
  }

  get rotorCount(): number {
    return this.rotors.length;
  }

  dispose(): void {
    this.group.traverse((o) => {
      if (!(o instanceof THREE.Mesh)) return;
      o.geometry.dispose();
      for (const m of [o.material].flat() as THREE.Material[]) m.dispose();
    });
  }
}
