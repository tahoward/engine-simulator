/**
 * The animated engine, for every cylinder count the spec allows: one cylinder up to a V8.
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
 */

import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/examples/jsm/geometries/RoundedBoxGeometry.js';
import {
  type BankSnapshot,
  type EngineSpec,
  clearanceVolume,
  crankPins,
  cylinderZ,
  ROD_STAGGER,
  mainBearingsAfter,
  exhaustPortDiameter,
  cylinderSpacing,
  firingPlan,
  physicalBank,
  physicalBankCount,
  pistonPosition,
} from '../model/spec.js';
import {
  BUCKET_HEIGHT,
  END_JOURNAL,
  STEM_LENGTH,
  VALVE_TILT,
  camBaseRadius,
  engineShell,
  exhaustPortOf,
} from '../model/geometry.js';
import { valveLift } from '../model/cam.js';

const STEEL = { color: 0x8d949e, metalness: 0.92, roughness: 0.34 };
const CAST = { color: 0x5c626c, metalness: 0.55, roughness: 0.66 };
/** The shells the main journals run in: a bronze-coloured lining. */
const BEARING = { color: 0xb8894f, metalness: 0.7, roughness: 0.4 };
const ALLOY = { color: 0xb9c0c9, metalness: 0.85, roughness: 0.28 };
/** The engine's outline: see-through, so everything inside it shows. */
const SHELL = { color: 0xa9b1bb, metalness: 0.2, roughness: 0.6, transparent: true, opacity: 0.13, depthWrite: false };
/** Each exhaust port's flange, bright enough to find. */
const PORT = { color: 0xff8c42, emissive: 0x6a2a00, metalness: 0.5, roughness: 0.4 };

export interface ExhaustPort {
  /** World position where the pipe begins. */
  position: THREE.Vector3;
  /** Unit direction the pipe leaves in. */
  direction: THREE.Vector3;
}

/** A cam lobe, turning at half the crank's speed: the exhaust's or the intake's. */
interface Lobe {
  mesh: THREE.Object3D;
  exhaust: boolean;
}

/**
 * A pushrod engine's rocker arm, in the cylinder's frame, and the tappet on the block's cam that works it,
 * in the engine's: the arm turns on its pivot as the valve opens, and the tappet rides up the lobe. The
 * pushrod between them is not drawn.
 */
interface Rocker {
  arm: THREE.Group;
  exhaust: boolean;
  /** The valve's tip at rest, the pivot, and the way down the stem, in the cylinder's frame. */
  tip: THREE.Vector3;
  pivot: THREE.Vector3;
  stem: THREE.Vector3;
  lifter: THREE.Mesh;
  /** Where the tappet sits at rest, and the way the lobe lifts it, in the engine's frame. */
  lifterAt: THREE.Vector3;
  up: THREE.Vector3;
}

/**
 * One of a cam-switching head's finger rockers, in the cylinder's frame: on the rocker shaft at `pivot`, its
 * pad at `end` under a lobe. The outer ones press their valves; the middle one rides the high-speed lobe,
 * pressing nothing until the pin locks the three together. The pin lives on the middle one.
 */
interface Finger {
  arm: THREE.Group;
  exhaust: boolean;
  middle: boolean;
  pivot: THREE.Vector3;
  end: THREE.Vector3;
  stem: THREE.Vector3;
  pin: THREE.Mesh | null;
}

/** How far a finger rocker's shaft sits from the valve, square to the stem, m. */
const FINGER_REACH = 0.036;

/** How much further a pushrod engine's rocker arm moves the valve than its tappet moves. */
const ROCKER_RATIO = 1.5;

/** How wide each cam lobe is along its shaft, m. */
const LOBE_WIDTH = 0.011;

/**
 * A cam lobe, centred on the origin and `LOBE_WIDTH` thick along z: round its base circle, `base` in radius,
 * it stands out by `lift(deg)` where it meets a follower lying at angle `follower` from it as the crank is at
 * `deg`. Turned by minus half the crank angle, it then lifts the follower by exactly that.
 */
function lobeGeometry(base: number, follower: number, lift: (deg: number) => number): THREE.BufferGeometry {
  const shape = new THREE.Shape();
  const steps = 120;
  for (let k = 0; k < steps; k++) {
    const phi = (k / steps) * Math.PI * 2;
    const r = base + lift(((phi - follower) * 360) / Math.PI);
    const [x, y] = [r * Math.cos(phi), r * Math.sin(phi)];
    if (k === 0) shape.moveTo(x, y);
    else shape.lineTo(x, y);
  }
  shape.closePath();
  const g = new THREE.ExtrudeGeometry(shape, { depth: LOBE_WIDTH, bevelEnabled: false, curveSegments: 1 });
  g.translate(0, 0, -LOBE_WIDTH / 2);
  return g;
}

interface CylinderMesh {
  /** Rotated to the cylinder's bank axis and translated along Z; holds the moving parts. */
  group: THREE.Group;
  piston: THREE.Group;
  exValves: THREE.Group[];
  inValves: THREE.Group[];
  /** Its cam lobes, overhead or on the block's cam, a pushrod engine's rockers and a cam-switching head's. */
  lobes: Lobe[];
  rockers: Rocker[];
  fingers: Finger[];
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
}

export class EngineMesh {
  readonly group = new THREE.Group();

  private spec: EngineSpec;
  private readonly crank = new THREE.Group();
  /** The main bearings, which are the block's and do not turn with the crank. */
  private readonly bearings = new THREE.Group();
  /** The see-through outline of the block and heads. */
  private readonly shell = new THREE.Group();
  /** A pushrod engine's camshaft in the block, and its lobes and tappets. */
  private readonly blockCam = new THREE.Group();
  private readonly cyls: CylinderMesh[] = [];

  private deckY = 0;
  private crownOffset = 0;
  private readonly valveTilt = VALVE_TILT;
  /** Centre-to-centre cylinder spacing along the crank, m. */
  private spacing = 0;

  constructor(spec: EngineSpec) {
    this.spec = { ...spec };
    this.group.add(this.crank, this.bearings, this.shell, this.blockCam);
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
    disposeChildren(this.bearings);
    disposeChildren(this.shell);
    disposeChildren(this.blockCam);

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
        this.cyls[cyl] = this.buildCylinder(bank, cylinderZ(this.spec, cyl), pins[pin]!.angles[k]!, exhaustSide);
      });
    }

    this.buildShell();
    this.buildBlockCamShaft();

    // Straddle vertical, so a V looks like a V rather than leaning.
    this.group.rotation.z = ((plan.bankCount > 1 ? this.spec.vAngle / 2 : 0) * Math.PI) / 180;

    // Posed with cylinder 1 at TDC, so the engine is assembled before the first snapshot turns it.
    this.update(plan.offsets.map((o) => ({ crankAngle: (720 - o) % 720 })), []);
  }

  private buildCrank(throws: Array<{ angles: number[]; z: number }>): void {
    const s = this.spec;
    const a = s.stroke / 2;
    const mainAfter = mainBearingsAfter(s);
    const steel = new THREE.MeshStandardMaterial(STEEL);
    const cast = new THREE.MeshStandardMaterial(CAST);
    const shell = new THREE.MeshStandardMaterial({ ...BEARING, side: THREE.DoubleSide });
    const webShape = crankWebShape(a);

    // Where each pin is round the shaft, with the throw's first cylinder's pin up +y at angle 0: rotating the
    // crank by -theta carries it to a*(sin(theta - angle), cos(theta - angle)), TDC when theta == angle.
    const pinAt = (angle: number) => {
      const phi = (angle * Math.PI) / 180;
      return new THREE.Vector2(-a * Math.sin(phi), a * Math.cos(phi));
    };

    // A split throw's pins sit side by side along the shaft, each half as wide, under their cylinders.
    const throwPins = throws.map(({ angles, z }) =>
      angles.map((angle, k) => ({
        angle,
        z: z + (angles.length > 1 ? (k - (angles.length - 1) / 2) * ROD_STAGGER : 0),
        width: angles.length > 1 ? ROD_STAGGER - 0.001 : PIN_WIDTH,
      })),
    );

    const addWeb = (angle: number, z: number) => {
      const geom = new THREE.ExtrudeGeometry(webShape, { depth: WEB_THICKNESS, bevelEnabled: false, curveSegments: 20 });
      geom.translate(0, 0, -WEB_THICKNESS / 2);
      const web = new THREE.Mesh(geom, cast);
      web.name = 'web';
      web.userData = { angle };
      web.rotation.z = (angle * Math.PI) / 180;
      web.position.z = z;
      web.castShadow = true;
      this.crank.add(web);
    };
    const addMain = (from: number, to: number) => {
      const length = to - from;
      if (length < 0.004) return;
      // Into the webs a little either side, so there is no seam where they meet.
      const journal = new THREE.Mesh(new THREE.CylinderGeometry(MAIN_RADIUS, MAIN_RADIUS, length + 0.002, 20), steel);
      journal.name = 'main journal';
      journal.rotation.x = Math.PI / 2;
      journal.position.z = (from + to) / 2;
      journal.castShadow = true;
      this.crank.add(journal);
      const bearing = new THREE.Mesh(
        new THREE.CylinderGeometry(MAIN_RADIUS + 0.0035, MAIN_RADIUS + 0.0035, Math.min(0.8 * length, 0.024), 24, 1, true),
        shell,
      );
      bearing.name = 'main bearing';
      bearing.rotation.x = Math.PI / 2;
      bearing.position.z = (from + to) / 2;
      this.bearings.add(bearing);
    };

    // Every throw: its pins, and a web with a counterweight on each side that faces a main journal. Between
    // two throws with no main, one web runs straight from the one's pin to the next's instead.
    let mainFrom = -Infinity;
    throwPins.forEach((own, t) => {
      const first = own[0]!;
      const last = own.at(-1)!;
      const lo = first.z - first.width / 2;
      const hi = last.z + last.width / 2;
      const mainBefore = t === 0 || mainAfter[t - 1]!;
      const mainNext = t === throwPins.length - 1 || mainAfter[t]!;
      if (mainBefore) {
        addWeb(first.angle, lo - WEB_THICKNESS / 2);
        addMain(t === 0 ? lo - WEB_THICKNESS - END_JOURNAL : mainFrom, lo - WEB_THICKNESS);
      }
      if (mainNext) {
        addWeb(last.angle, hi + WEB_THICKNESS / 2);
        mainFrom = hi + WEB_THICKNESS;
        if (t === throwPins.length - 1) addMain(mainFrom, mainFrom + END_JOURNAL);
      } else {
        const next = throwPins[t + 1]![0]!;
        const to = next.z - next.width / 2;
        const shape = linkWebShape(pinAt(last.angle), pinAt(next.angle));
        const depth = Math.max(to - hi, 0.004);
        const geom = new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: false, curveSegments: 20 });
        geom.translate(0, 0, hi);
        const link = new THREE.Mesh(geom, cast);
        link.name = 'link web';
        link.castShadow = true;
        this.crank.add(link);
      }

      // One big-end journal per pin, shared by the cylinders hanging off it — which is what ties a V-twin's
      // firing interval to its V angle.
      for (const { angle, z, width } of own) {
        const at = pinAt(angle);
        const pin = new THREE.Mesh(new THREE.CylinderGeometry(PIN_RADIUS, PIN_RADIUS, width, 16), steel);
        pin.rotation.x = Math.PI / 2;
        pin.position.set(at.x, at.y, z);
        pin.castShadow = true;
        this.crank.add(pin);
      }
    });
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

    // --- valves, and what opens them ---
    const exValves = this.buildValves(s.exValveDia, s.exValveCount, exhaustSide, true);
    const inValves = this.buildValves(s.inValveDia, s.inValveCount, -exhaustSide, false);
    group.add(...exValves, ...inValves);
    const lobes: Lobe[] = [];
    const rockers: Rocker[] = [];
    const fingers: Finger[] = [];
    if (s.pushrods) {
      for (const [valves, exhaust] of [
        [exValves, true],
        [inValves, false],
      ] as const) {
        for (const v of valves) rockers.push(this.buildRocker(group, v, rotation, z, exhaust, lobes));
      }
    } else {
      lobes.push(...this.buildOverheadCam(group, exValves, exhaustSide, true, fingers));
      lobes.push(...this.buildOverheadCam(group, inValves, -exhaustSide, false, fingers));
    }

    // --- exhaust port, in this cylinder's frame ---
    group.add(this.buildPort(exhaustSide));

    // --- rod: world space, from the pin to this cylinder's piston ---
    // Slimmer along the crank than across it, as a rod's beam is, so two fit side by side on a shared pin.
    const rod = new THREE.Mesh(
      new THREE.CylinderGeometry(0.009, 0.012, 1, 12),
      new THREE.MeshStandardMaterial(STEEL),
    );
    rod.name = 'rod';
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
      lobes,
      rockers,
      fingers,
      exhaustSide,
      rod,
      flame,
      rotation,
      z,
      pinAngle: (pinAngleDeg * Math.PI) / 180,
    };
  }

  /** Lift at crank angle `deg` of one side's valves on the low-speed lobe, or with `high` the high-speed one. */
  private liftOf(exhaust: boolean, high: boolean): (deg: number) => number {
    const s = this.spec;
    if (exhaust) return high ? (d) => valveLift(d, s.highEvo, s.highEvc, s.highMaxLift) : (d) => valveLift(d, s.evo, s.evc, s.maxLift);
    return high ? (d) => valveLift(d, s.highIvo, s.highIvc, s.highMaxLift) : (d) => valveLift(d, s.ivo, s.ivc, s.maxLift);
  }

  /**
   * The camshaft over one side's valves, in the cylinder's frame: a length of shaft over the cylinder, and a
   * lobe over each valve's bucket, its profile the valve's lift. With cam profile switching, a high-speed lobe
   * beside them, which the valves follow from the switch speed.
   */
  private buildOverheadCam(
    group: THREE.Group,
    valves: THREE.Group[],
    sign: number,
    exhaust: boolean,
    fingers: Finger[],
  ): Lobe[] {
    const s = this.spec;
    const base = camBaseRadius(s);
    const stem = new THREE.Vector3(0, 1, 0).applyAxisAngle(AXIS_Z, -sign * this.valveTilt);
    const centre = new THREE.Vector3(sign * s.bore * 0.24, this.deckY, 0).addScaledVector(
      stem,
      s.bore * STEM_LENGTH + BUCKET_HEIGHT + base,
    );
    const follower = Math.atan2(-stem.y, -stem.x);
    const steel = new THREE.MeshStandardMaterial(STEEL);
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(base * 0.55, base * 0.55, this.spacing, 20), steel);
    shaft.rotation.x = Math.PI / 2;
    shaft.position.copy(centre);
    group.add(shaft);
    const lobes: Lobe[] = [];
    const add = (z: number, high: boolean) => {
      const lobe = new THREE.Mesh(lobeGeometry(base, follower, this.liftOf(exhaust, high)), steel);
      lobe.position.set(centre.x, centre.y, z);
      lobe.castShadow = true;
      group.add(lobe);
      lobes.push({ mesh: lobe, exhaust });
    };
    for (const v of valves) add(v.userData.z as number, false);
    if (s.camSwitchRpm > 0) {
      const middle = valves.length > 1 ? 0 : LOBE_WIDTH * 1.4;
      add(middle, true);
      fingers.push(...this.buildFingers(group, valves, stem, sign, exhaust, middle));
    }
    return lobes;
  }

  /**
   * A cam-switching head's finger rockers for one side's valves, in the cylinder's frame: one under each
   * low-speed lobe, its pad where a bucket would be, pressing its valve, and one under the high-speed lobe at
   * `middle`, between them, all on a rocker shaft `FINGER_REACH` out from the valves, square to the stems
   * at half lift. And the pin through the middle one that locks them together.
   */
  private buildFingers(
    group: THREE.Group,
    valves: THREE.Group[],
    stem: THREE.Vector3,
    sign: number,
    exhaust: boolean,
    middle: number,
  ): Finger[] {
    const s = this.spec;
    const steel = new THREE.MeshStandardMaterial(STEEL);
    const tip = new THREE.Vector3(sign * s.bore * 0.24, this.deckY, 0).addScaledVector(stem, s.bore * STEM_LENGTH);
    const end = tip.clone().addScaledVector(stem, BUCKET_HEIGHT / 2);
    const out = new THREE.Vector3(sign, 0, 0);
    const square = out.addScaledVector(stem, -out.dot(stem)).normalize();
    const pivot = end.clone().addScaledVector(stem, -s.maxLift / 2).addScaledVector(square, FINGER_REACH);
    const span = end.clone().sub(pivot);
    const width = Math.min(LOBE_WIDTH + 0.002, 0.013);

    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.0055, 0.0055, this.spacing, 16), steel);
    shaft.rotation.x = Math.PI / 2;
    shaft.position.copy(pivot);
    group.add(shaft);

    const make = (z: number, isMiddle: boolean): Finger => {
      const arm = new THREE.Group();
      arm.position.set(pivot.x, pivot.y, z);
      const bar = new THREE.Mesh(
        new RoundedBoxGeometry(span.length() + 0.012, BUCKET_HEIGHT, width, 2, 0.0025),
        isMiddle ? new THREE.MeshStandardMaterial({ ...STEEL, color: 0xa7adb6 }) : steel,
      );
      bar.position.copy(span.clone().multiplyScalar(0.5));
      bar.rotation.z = Math.atan2(span.y, span.x);
      arm.add(bar);
      const boss = new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.009, width, 16), steel);
      boss.rotation.x = Math.PI / 2;
      arm.add(boss);
      let pin: THREE.Mesh | null = null;
      if (isMiddle) {
        // Along the shaft through the three, part way out along them, its length set as it locks.
        pin = new THREE.Mesh(
          new THREE.CylinderGeometry(0.0032, 0.0032, 1, 12),
          new THREE.MeshStandardMaterial({ color: 0xc89b4a, metalness: 0.8, roughness: 0.35 }),
        );
        pin.rotation.x = Math.PI / 2;
        pin.position.copy(span.clone().multiplyScalar(0.55));
        arm.add(pin);
      }
      group.add(arm);
      return { arm, exhaust, middle: isMiddle, pivot, end: end.clone().setZ(z), stem, pin };
    };
    const out2 = valves.map((v) => make(v.userData.z as number, false));
    out2.push(make(middle, true));
    return out2;
  }

  /**
   * A pushrod engine's rocker arm over `valve`, in the cylinder's frame, and the tappet and lobe on the
   * block's cam that work it, in the engine's: the arm on a pivot beside the valve's tip, towards the cam
   * for an intake valve and away from it for an exhaust valve, its far end `ROCKER_RATIO` times nearer the
   * pivot, so the valve moves that much more than the tappet does. The tappet sits on the lobe under where the arm's far end would take a pushrod. `rotation` and
   * `z` place the cylinder; its lobe goes into `lobes`.
   */
  private buildRocker(
    group: THREE.Group,
    valve: THREE.Group,
    rotation: number,
    z: number,
    exhaust: boolean,
    lobes: Lobe[],
  ): Rocker {
    const s = this.spec;
    const steel = new THREE.MeshStandardMaterial(STEEL);
    const cam = this.blockCamCentre();
    // The cam as the cylinder sees it, so the arm can reach towards it.
    const camInCylinder = cam.clone().applyAxisAngle(AXIS_Z, -rotation);
    // An intake valve's arm reaches in towards the cam; an exhaust valve's, on the head's outer side, out
    // away from it.
    const inward = Math.sign(camInCylinder.x) || 1;
    const toward = new THREE.Vector3(exhaust ? -inward : inward, 0, 0);
    const stem = new THREE.Vector3(0, 1, 0).applyAxisAngle(AXIS_Z, valve.rotation.z);
    const tip = valve.position.clone().addScaledVector(stem, s.bore * STEM_LENGTH);
    // Square to the stem, level with the tip at half lift: so the arm is at right angles to the stem half
    // way open, and tilts as little either side of it as it can, as a rocker is set up.
    const square = toward.clone().addScaledVector(stem, -toward.dot(stem)).normalize();
    const pivot = tip.clone().addScaledVector(stem, -s.maxLift / 2).addScaledVector(square, 0.024);
    const cup = pivot.clone().add(pivot.clone().sub(tip).divideScalar(ROCKER_RATIO));

    // The arm: a bar from the cup to the tip, turning on the pivot.
    const arm = new THREE.Group();
    arm.position.copy(pivot);
    const span = cup.clone().sub(tip);
    const bar = new THREE.Mesh(new RoundedBoxGeometry(span.length() + 0.012, 0.007, 0.011, 2, 0.002), steel);
    bar.position.copy(cup.clone().add(tip).multiplyScalar(0.5).sub(pivot));
    bar.rotation.z = Math.atan2(span.y, span.x);
    arm.add(bar);
    const stud = new THREE.Mesh(new THREE.CylinderGeometry(0.004, 0.004, 0.016, 10), steel);
    stud.rotation.x = Math.PI / 2;
    arm.add(stud);
    group.add(arm);

    // The tappet, on the block's cam, in line with the arm's far end, in the engine's frame.
    const cupAt = cup.clone().applyAxisAngle(AXIS_Z, rotation);
    cupAt.z = z + (valve.userData.z as number);
    const up = new THREE.Vector3(cupAt.x - cam.x, cupAt.y - cam.y, 0).normalize();
    const base = camBaseRadius(s);
    const lifterAt = new THREE.Vector3(cam.x, cam.y, cupAt.z).addScaledVector(up, base + 0.0125);
    const lifter = new THREE.Mesh(new THREE.CylinderGeometry(0.0065, 0.0065, 0.025, 14), steel);
    lifter.quaternion.setFromUnitVectors(AXIS_Y, up);
    this.blockCam.add(lifter);

    const lift = this.liftOf(exhaust, false);
    const lobe = new THREE.Mesh(lobeGeometry(base, Math.atan2(up.y, up.x), (d) => lift(d) / ROCKER_RATIO), steel);
    lobe.position.set(cam.x, cam.y, cupAt.z);
    this.blockCam.add(lobe);
    lobes.push({ mesh: lobe, exhaust });

    return { arm, exhaust, tip, pivot, stem, lifter, lifterAt, up };
  }

  /**
   * Where a pushrod engine's camshaft runs, in the engine's frame: in the valley of a V, just above the
   * crankcase, or beside the crank on an inline engine's intake side.
   */
  private blockCamCentre(): THREE.Vector3 {
    const s = this.spec;
    const shell = engineShell(s);
    const d = shell.crankcase.radius + camBaseRadius(s) + 0.012;
    if (physicalBankCount(s) > 1) {
      const half = ((s.vAngle / 2) * Math.PI) / 180;
      return new THREE.Vector3(Math.sin(half) * d, Math.cos(half) * d, 0);
    }
    return new THREE.Vector3(-0.8 * d, 0.6 * d, 0);
  }

  /** The block cam's shaft, along the whole engine. */
  private buildBlockCamShaft(): void {
    if (!this.spec.pushrods) return;
    const shell = engineShell(this.spec);
    const centre = this.blockCamCentre();
    const shaft = new THREE.Mesh(
      new THREE.CylinderGeometry(camBaseRadius(this.spec) * 0.55, camBaseRadius(this.spec) * 0.55, shell.length, 20),
      new THREE.MeshStandardMaterial(STEEL),
    );
    shaft.rotation.x = Math.PI / 2;
    shaft.position.copy(centre);
    this.blockCam.add(shaft);
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
      new THREE.CylinderGeometry(0.0038, 0.0038, s.bore * STEM_LENGTH, 12),
      new THREE.MeshStandardMaterial(STEEL),
    );
    stem.position.y = (s.bore * STEM_LENGTH) / 2;
    group.add(stem);
    // Under an overhead cam, the bucket it presses on, capping the stem.
    if (!s.pushrods && s.camSwitchRpm <= 0) {
      const r = Math.min(dia * 0.42, 0.016);
      const bucket = new THREE.Mesh(new THREE.CylinderGeometry(r, r, BUCKET_HEIGHT, 20), new THREE.MeshStandardMaterial(STEEL));
      bucket.position.y = s.bore * STEM_LENGTH + BUCKET_HEIGHT / 2;
      group.add(bucket);
    }

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

  /**
   * The exhaust port, where its pipe starts: a flange round the opening, facing the way the pipe leaves, on
   * the side of the head the exhaust comes out of. In the cylinder's frame, at the place `exhaustPortOf` gives.
   */
  private buildPort(side: number): THREE.Group {
    const s = this.spec;
    const port = new THREE.Group();
    port.name = 'exhaust port';
    const r = exhaustPortDiameter(s) / 2;
    const flange = new THREE.Mesh(new THREE.TorusGeometry(r * 1.25, r * 0.28, 10, 32), new THREE.MeshStandardMaterial(PORT));
    const opening = new THREE.Mesh(
      new THREE.CircleGeometry(r * 1.05, 32),
      new THREE.MeshBasicMaterial({ color: 0x1a0d05, side: THREE.DoubleSide }),
    );
    port.add(flange, opening);
    // Its face square to the pipe: the torus and disc lie in their own XY, facing Z, turned to face X.
    port.rotation.y = Math.PI / 2;
    port.position.set(side * s.bore * 1.15, this.deckY + s.bore * 0.52 * 0.45, 0);
    return port;
  }

  /**
   * The block and heads, see-through: one rounded casting per bank from the crankcase up past the valve
   * springs, and the crankcase round the crank. Only an outline, so the parts moving inside it all show.
   */
  private buildShell(): void {
    const shell = engineShell(this.spec);
    const mat = new THREE.MeshStandardMaterial(SHELL);
    const { width, top, bottom, length } = shell;
    for (const turn of shell.banks) {
      // In the engine's own frame, which the group turns by the straddle.
      const rotation = turn - shell.straddle;
      const bank = new THREE.Mesh(new RoundedBoxGeometry(width, top - bottom, length, 3, shell.rounding), mat);
      bank.name = 'shell';
      bank.position.set(0, (top + bottom) / 2, 0);
      bank.position.applyAxisAngle(AXIS_Z, rotation);
      bank.rotation.z = rotation;
      bank.renderOrder = 20;
      this.shell.add(bank);
    }
    // The crankcase: round the counterweights' sweep, along the whole crank.
    const { radius, length: crankLength } = shell.crankcase;
    const crankcase = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, crankLength, 40), mat);
    crankcase.name = 'shell';
    crankcase.rotation.x = Math.PI / 2;
    crankcase.renderOrder = 20;
    this.shell.add(crankcase);
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
      spec.vAngle !== this.spec.vAngle ||
      // The cam lobes are cut to the valve timing.
      spec.pushrods !== this.spec.pushrods ||
      (['evo', 'evc', 'ivo', 'ivc', 'maxLift', 'camSwitchRpm', 'highEvo', 'highEvc', 'highIvo', 'highIvc', 'highMaxLift'] as const).some(
        (k) => spec[k] !== this.spec[k],
      );

    this.spec = { ...spec };
    if (structural) this.rebuild();
  }

  /**
   * Pose every moving part from the snapshot.
   *
   * Driven by each cylinder's own crank angle rather than a single global one, so the phase
   * relationship on screen is whatever the physics is actually running.
   */
  update(banks: Array<Pick<BankSnapshot, 'crankAngle'>>, burnGlow: number[]): void {
    const s = this.spec;
    const a = s.stroke / 2;

    // The crank follows cylinder 0, whose axis is +Y and whose TDC is at angle 0.
    const th0 = ((banks[0]?.crankAngle ?? 0) * Math.PI) / 180;
    this.crank.rotation.z = -th0;

    for (let i = 0; i < this.cyls.length; i++) {
      const mesh = this.cyls[i]!;
      const snap = banks[Math.min(i, banks.length - 1)]!;

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
      mesh.rod.scale.set(1, axis.length(), ROD_THICKNESS / 0.024);
      mesh.rod.quaternion.setFromUnitVectors(AXIS_Y, axis.normalize());

      const ex = this.exhaustCamRetard;
      const inn = -this.intakeCamAdvance;
      const high = this.highCam;
      const exLift = high
        ? valveLift(snap.crankAngle, s.highEvo + ex, s.highEvc + ex, s.highMaxLift)
        : valveLift(snap.crankAngle, s.evo + ex, s.evc + ex, s.maxLift);
      const inLift = high
        ? valveLift(snap.crankAngle, s.highIvo + inn, s.highIvc + inn, s.highMaxLift)
        : valveLift(snap.crankAngle, s.ivo + inn, s.ivc + inn, s.maxLift);
      for (const v of mesh.exValves) this.poseValve(v, mesh.exhaustSide, exLift);
      for (const v of mesh.inValves) this.poseValve(v, -mesh.exhaustSide, inLift);
      // The cams turn at half the crank's speed, each lobe set round by its cam's phaser.
      for (const lobe of mesh.lobes) {
        lobe.mesh.rotation.z = (-(snap.crankAngle - (lobe.exhaust ? ex : inn)) * Math.PI) / 360;
      }
      for (const r of mesh.rockers) this.poseRocker(r, r.exhaust ? exLift : inLift);
      if (mesh.fingers.length > 0) {
        // The middle finger rides its own lobe until the pin locks it to the others, the valves then on it.
        const exHigh = valveLift(snap.crankAngle, s.highEvo + ex, s.highEvc + ex, s.highMaxLift);
        const inHigh = valveLift(snap.crankAngle, s.highIvo + inn, s.highIvc + inn, s.highMaxLift);
        for (const f of mesh.fingers) {
          const valve = f.exhaust ? exLift : inLift;
          this.poseFinger(f, f.middle && !high ? (f.exhaust ? exHigh : inHigh) : valve, high, mesh);
        }
      }

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

  /**
   * Open a pushrod engine's valve by `lift`: the arm turned on its pivot to push the tip that far down the
   * stem, and the tappet `ROCKER_RATIO` times less far up from the cam.
   */
  private poseRocker(r: Rocker, lift: number): void {
    const rest = r.tip.clone().sub(r.pivot);
    const now = rest.clone().addScaledVector(r.stem, -lift);
    r.arm.rotation.z = Math.atan2(now.y, now.x) - Math.atan2(rest.y, rest.x);
    const rise = lift / ROCKER_RATIO;
    r.lifter.position.copy(r.lifterAt).addScaledVector(r.up, rise);
  }

  /**
   * Swing a finger rocker on its shaft so its pad has moved `lift` down the stem; and on the middle one, the
   * pin out through all three of its side's fingers when they are locked, or back inside it when not.
   */
  private poseFinger(f: Finger, lift: number, locked: boolean, mesh: CylinderMesh): void {
    const rest = f.end.clone().sub(f.pivot).setZ(0);
    const now = rest.clone().addScaledVector(f.stem, -lift);
    f.arm.rotation.z = Math.atan2(now.y, now.x) - Math.atan2(rest.y, rest.x);
    if (!f.pin) return;
    const sideways = mesh.fingers.filter((g) => g.exhaust === f.exhaust).map((g) => g.end.z - f.end.z);
    const across = Math.max(...sideways.map(Math.abs)) * 2 + 0.012;
    f.pin.scale.y = locked ? across : 0.01;
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

  /** Where the cam phasers have the cams, crank degrees, as the last snapshot reported them. */
  intakeCamAdvance = 0;
  exhaustCamRetard = 0;
  /** Whether cam profile switching had the valves on the high-speed lobes at the last snapshot. */
  highCam = false;

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
}

const AXIS_Z = new THREE.Vector3(0, 0, 1);
const AXIS_Y = new THREE.Vector3(0, 1, 0);

/** How thick each crank web is along the shaft, m. */
const WEB_THICKNESS = 0.011;
/** Radius of the crank's main journals, m. */
const MAIN_RADIUS = 0.019;
/** Radius of its crankpins, m. */
const PIN_RADIUS = 0.0135;
/** How wide a crankpin is along the shaft, m: room for two rods side by side on a shared one. */
const PIN_WIDTH = 2 * ROD_STAGGER - 0.002;
/** How thick a rod's big end is along the crank, m: a little less than the stagger between two. */
const ROD_THICKNESS = ROD_STAGGER - 0.003;
/** Radius of the boss a web has round a crankpin, m. */
const PIN_BOSS = 0.022;

/**
 * A crank web's outline, with its pin at `(0, throwRadius)`: a boss round the pin, and the counterweight on
 * the far side of the shaft from it, a broad sector reaching further out than the pin does, so it balances
 * the pin, the big end and its share of the rod.
 */
export function crankWebShape(throwRadius: number): THREE.Shape {
  const a = throwRadius;
  const boss = PIN_BOSS;
  const reach = 1.5 * a;
  const half = (75 * Math.PI) / 180;
  const shape = new THREE.Shape();
  // Round the counterweight, below the shaft, from one side to the other; up to the boss; over it; back down.
  const from = -Math.PI / 2 - half;
  const to = -Math.PI / 2 + half;
  shape.moveTo(reach * Math.cos(from), reach * Math.sin(from));
  shape.absarc(0, 0, reach, from, to, false);
  shape.lineTo(boss, a);
  shape.absarc(0, a, boss, 0, Math.PI, false);
  shape.closePath();
  return shape;
}

/**
 * The web joining two neighbouring pins with no main bearing between them, as an opposed pair's pins in a
 * flat engine are: the outline round both pins' bosses, and round the shaft's axis too, which is what it
 * turns about.
 */
export function linkWebShape(a: THREE.Vector2, b: THREE.Vector2): THREE.Shape {
  const points: THREE.Vector2[] = [];
  const ring = (c: THREE.Vector2, r: number) => {
    for (let i = 0; i < 48; i++) {
      const t = (i / 48) * 2 * Math.PI;
      points.push(new THREE.Vector2(c.x + r * Math.cos(t), c.y + r * Math.sin(t)));
    }
  };
  ring(a, PIN_BOSS);
  ring(b, PIN_BOSS);
  ring(new THREE.Vector2(0, 0), MAIN_RADIUS);
  return new THREE.Shape(convexHull(points));
}

/** The convex hull of `points`, anticlockwise: Andrew's monotone chain. */
function convexHull(points: THREE.Vector2[]): THREE.Vector2[] {
  const sorted = [...points].sort((p, q) => p.x - q.x || p.y - q.y);
  const cross = (o: THREE.Vector2, p: THREE.Vector2, q: THREE.Vector2) =>
    (p.x - o.x) * (q.y - o.y) - (p.y - o.y) * (q.x - o.x);
  const half = (list: THREE.Vector2[]) => {
    const out: THREE.Vector2[] = [];
    for (const p of list) {
      while (out.length >= 2 && cross(out[out.length - 2]!, out[out.length - 1]!, p) <= 0) out.pop();
      out.push(p);
    }
    out.pop();
    return out;
  };
  return [...half(sorted), ...half([...sorted].reverse())];
}

/** Empty a group, releasing its geometry and materials. */
function disposeChildren(group: THREE.Group): void {
  for (const child of [...group.children]) {
    disposeTree(child);
    group.remove(child);
  }
}

function disposeTree(node: THREE.Object3D): void {
  node.traverse((o) => {
    if (!(o instanceof THREE.Mesh)) return;
    o.geometry.dispose();
    for (const m of [o.material].flat() as THREE.Material[]) m.dispose();
  });
}
