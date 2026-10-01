/**
 * The intake as a car has it: a cast plenum on the engine, a throttle body with its butterfly turning with
 * the throttle, a black rubber tube with a bellows coupler and hose clamps, a black plastic airbox with its
 * lid's seam and clips, and a flattened snorkel flaring at its mouth. Where each goes is `inletLayout`.
 *
 * In the pressure view the tube, the airbox and the snorkel take the colour of the cells the simulation
 * solves along them, on the exhaust's scale.
 */

import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/examples/jsm/geometries/RoundedBoxGeometry.js';

import type { EngineSpec } from '../model/spec.js';
import { inletLayout, type InletLayout } from './inletLayout.js';
import { pressureColor } from './PipeMesh.js';

const PLASTIC = { color: 0x2c2f35, metalness: 0.05, roughness: 0.62 };
const RUBBER = { color: 0x1f2125, metalness: 0.0, roughness: 0.85 };
const CAST = { color: 0x9da3ab, metalness: 0.72, roughness: 0.42 };
const CLAMP = { color: 0xc3c7cc, metalness: 0.92, roughness: 0.28 };

/** Stations along a swept tube, and points round its section. */
const ALONG = 48;
const AROUND = 28;

/** How much of the tube, from the throttle body, is the bellows coupler, m, and its ribs' pitch. */
const BELLOWS = 0.09;
const RIB_PITCH = 0.012;

/** Wall thickness of the rubber tube and the snorkel, m: how far they stand out of their bore. */
const WALL = 0.004;

/**
 * A tube swept along `curve`, its section an ellipse `size(u)` returns the half-width (across, level) and
 * half-height of, at each fraction `u` along it. The frame is kept level: across is the curve's direction
 * crossed with up, which suits a tract that runs about level.
 */
function sweep(curve: THREE.Curve<THREE.Vector3>, size: (u: number) => [number, number]): THREE.BufferGeometry {
  const positions: number[] = [];
  const normals: number[] = [];
  const colors: number[] = [];
  const index: number[] = [];
  const up = new THREE.Vector3(0, 1, 0);
  for (let i = 0; i <= ALONG; i++) {
    const u = i / ALONG;
    const at = curve.getPointAt(u);
    const t = curve.getTangentAt(u).normalize();
    const across = new THREE.Vector3().crossVectors(t, up);
    if (across.lengthSq() < 1e-8) across.set(1, 0, 0);
    across.normalize();
    const lift = new THREE.Vector3().crossVectors(across, t).normalize();
    const [a, b] = size(u);
    for (let j = 0; j <= AROUND; j++) {
      const th = (j / AROUND) * Math.PI * 2;
      const c = Math.cos(th);
      const s = Math.sin(th);
      const p = at.clone().addScaledVector(across, a * c).addScaledVector(lift, b * s);
      positions.push(p.x, p.y, p.z);
      // The normal of an ellipse at that point, turned into the frame.
      const n = across.clone().multiplyScalar(c / a).addScaledVector(lift, s / b).normalize();
      normals.push(n.x, n.y, n.z);
      colors.push(1, 1, 1);
    }
  }
  for (let i = 0; i < ALONG; i++) {
    for (let j = 0; j < AROUND; j++) {
      const a = i * (AROUND + 1) + j;
      const b = a + AROUND + 1;
      index.push(a, b, a + 1, b, b + 1, a + 1);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  g.setAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));
  g.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  g.setIndex(index);
  return g;
}

/** A band round a tube at `u` along `curve`: a hose clamp. */
function band(curve: THREE.Curve<THREE.Vector3>, u: number, radius: number, material: THREE.Material): THREE.Mesh {
  const ring = new THREE.Mesh(new THREE.TorusGeometry(radius, 0.0028, 8, 40), material);
  ring.position.copy(curve.getPointAt(u));
  ring.lookAt(ring.position.clone().add(curve.getTangentAt(u)));
  return ring;
}

export class InletMesh {
  readonly group = new THREE.Group();
  /** The parts only an engine without a turbo has: the tube, the airbox and the snorkel. */
  private readonly tract = new THREE.Group();
  private readonly plastic = new THREE.MeshStandardMaterial(PLASTIC);
  /** The airbox's and the snorkel's plastic, which the pressure view colours. */
  private readonly tinted = new THREE.MeshStandardMaterial({ ...PLASTIC, vertexColors: true, side: THREE.DoubleSide });
  private readonly rubber = new THREE.MeshStandardMaterial({ ...RUBBER, vertexColors: true, side: THREE.DoubleSide });
  private readonly cast = new THREE.MeshStandardMaterial({ ...CAST, side: THREE.DoubleSide });
  private readonly clamp = new THREE.MeshStandardMaterial(CLAMP);
  private readonly throat = new THREE.MeshBasicMaterial({ color: 0x08090b, side: THREE.DoubleSide });
  private readonly plate = new THREE.MeshStandardMaterial({ ...CAST, color: 0xc9ced4, side: THREE.DoubleSide });
  private butterfly: THREE.Object3D | null = null;
  private tube: THREE.Mesh | null = null;
  private box: THREE.Mesh | null = null;
  private snorkel: THREE.Mesh | null = null;
  private layout: InletLayout | null = null;
  private showPressure = false;

  constructor() {
    this.group.add(this.tract);
  }

  /** Whether the engine has the tract: one with a turbo draws through its compressors instead. */
  setTractVisible(on: boolean): void {
    this.tract.visible = on;
  }

  setPressureVisible(on: boolean): void {
    this.showPressure = on;
    if (!on) this.paint(null, 1);
  }

  /** Turn the butterfly to the throttle's opening, 0..1: nearly square to the bore shut, edge-on open. */
  setThrottle(opening: number): void {
    if (!this.butterfly) return;
    const open = 1 - Math.cos(Math.max(0, Math.min(1, opening)) * (Math.PI / 2));
    this.butterfly.rotation.x = ((8 + 82 * open) * Math.PI) / 180;
  }

  rebuild(spec: EngineSpec): void {
    this.clear();
    const l = inletLayout(spec);
    this.layout = l;
    this.buildEngineSide(l);
    this.buildTract(l);
    this.setThrottle(spec.throttle);
    this.paint(null, 1);
  }

  /** The plenum, its runners and the throttle body. */
  private buildEngineSide(l: InletLayout): void {
    const { centre, size } = l.plenum;
    const plenum = new THREE.Mesh(new RoundedBoxGeometry(size.x, size.y, size.z, 4, Math.min(size.x, size.y) * 0.3), this.cast);
    plenum.position.copy(centre);
    this.group.add(plenum);

    for (const r of l.runners) {
      const out = new THREE.Vector3(Math.sign(r.to.x - r.from.x), 0, 0);
      const curve = new THREE.CubicBezierCurve3(
        r.from,
        r.from.clone().addScaledVector(out, 0.05).add(new THREE.Vector3(0, 0.04, 0)),
        r.to.clone().addScaledVector(out, -0.05).add(new THREE.Vector3(0, 0.03, 0)),
        r.to,
      );
      this.group.add(new THREE.Mesh(new THREE.TubeGeometry(curve, 24, r.radius, 16, false), this.cast));
    }

    // The throttle body: a cast barrel with a flange each end, a shaft across, and the butterfly on it.
    const { bore, length } = l.throttle;
    const body = new THREE.Group();
    body.position.copy(l.throttle.centre);
    const barrel = new THREE.Mesh(new THREE.CylinderGeometry(bore / 2 + 0.006, bore / 2 + 0.006, length, 32, 1, true), this.cast);
    barrel.rotation.x = Math.PI / 2;
    body.add(barrel);
    for (const end of [-1, 1]) {
      const flange = new THREE.Mesh(new THREE.TorusGeometry(bore / 2 + 0.008, 0.005, 8, 32), this.cast);
      flange.position.z = (end * length) / 2;
      body.add(flange);
    }
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.003, 0.003, bore + 0.04, 8), this.clamp);
    shaft.rotation.z = Math.PI / 2;
    body.add(shaft);
    // The throttle position sensor on the shaft's end.
    const sensor = new THREE.Mesh(new RoundedBoxGeometry(0.012, 0.03, 0.026, 2, 0.003), this.plastic);
    sensor.position.x = bore / 2 + 0.024;
    body.add(sensor);
    const butterfly = new THREE.Object3D();
    const disc = new THREE.Mesh(new THREE.CircleGeometry(bore / 2 - 0.0005, 32), this.plate);
    butterfly.add(disc);
    body.add(butterfly);
    this.butterfly = butterfly;
    this.group.add(body);
  }

  /** The tube, the airbox and the snorkel. */
  private buildTract(l: InletLayout): void {
    // The tube: a bellows coupler off the throttle body, then smooth, a clamp at each end.
    const r = l.tubeRadius + WALL;
    const tubeLength = l.tube.getLength();
    const ribs = BELLOWS / tubeLength;
    this.tube = new THREE.Mesh(
      sweep(l.tube, (u) => {
        const s = u * tubeLength;
        const rib = u < ribs && s > 0.02 ? 0.0045 * Math.max(0, Math.sin((s / RIB_PITCH) * Math.PI * 2)) : 0;
        return [r + rib, r + rib];
      }),
      this.rubber,
    );
    this.tract.add(this.tube);
    this.tract.add(band(l.tube, 0.012, r + 0.0015, this.clamp));
    this.tract.add(band(l.tube, Math.min(ribs + 0.02, 0.5), r + 0.0015, this.clamp));
    this.tract.add(band(l.tube, 0.985, r + 0.0015, this.clamp));
    // The air flow meter on the tube, ahead of the airbox: a small housing with its plug.
    const at = l.tube.getPointAt(0.7);
    const maf = new THREE.Mesh(new RoundedBoxGeometry(0.03, 0.022, 0.04, 2, 0.004), this.plastic);
    maf.position.copy(at).add(new THREE.Vector3(0, r + 0.008, 0));
    maf.lookAt(maf.position.clone().add(l.tube.getTangentAt(0.7)));
    this.tract.add(maf);

    // The airbox: a rounded plastic box, its lid's seam a lip round it, clipped down front and back.
    const { centre, size } = l.airbox;
    const boxGeometry = new RoundedBoxGeometry(size.x, size.y, size.z, 5, Math.min(size.y, size.z) * 0.18);
    const white = new Float32Array(boxGeometry.getAttribute('position').count * 3).fill(1);
    boxGeometry.setAttribute('color', new THREE.BufferAttribute(white, 3));
    this.box = new THREE.Mesh(boxGeometry, this.tinted);
    this.box.position.copy(centre);
    this.tract.add(this.box);
    const seamY = centre.y + size.y * 0.15;
    const lip = new THREE.Mesh(new RoundedBoxGeometry(size.x + 0.01, 0.008, size.z + 0.01, 2, 0.003), this.plastic);
    lip.position.set(centre.x, seamY, centre.z);
    this.tract.add(lip);
    for (const fx of [-0.3, 0.3]) {
      for (const fz of [-1, 1]) {
        const clip = new THREE.Mesh(new RoundedBoxGeometry(0.018, 0.028, 0.008, 2, 0.002), this.clamp);
        clip.position.set(centre.x + fx * size.x, seamY, centre.z + (fz * (size.z + 0.012)) / 2);
        this.tract.add(clip);
      }
    }
    // A spigot each end, where the tube and the snorkel join it.
    for (const [curve, u] of [
      [l.tube, 1],
      [l.snorkel, 0],
    ] as const) {
      const spigot = new THREE.Mesh(new THREE.CylinderGeometry(r + 0.004, r + 0.004, 0.03, 24), this.plastic);
      spigot.position.copy(curve.getPointAt(u));
      spigot.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), curve.getTangentAt(u).normalize());
      this.tract.add(spigot);
    }

    // The snorkel: round where it leaves the airbox, flattening over its first stretch to fit under the
    // bonnet with the solver's area kept, and flaring at its mouth.
    const round = Math.sqrt(l.snorkelArea / Math.PI);
    const aspect = 1.8;
    const snorkelLength = l.snorkel.getLength();
    this.snorkel = new THREE.Mesh(
      sweep(l.snorkel, (u) => {
        const fromMouth = (1 - u) * snorkelLength;
        const flare = fromMouth < 0.04 ? 1 + 0.35 * (1 - fromMouth / 0.04) ** 2 : 1;
        const flat = Math.min(1, (u * snorkelLength) / 0.08);
        const wide = round * (1 + (Math.sqrt(aspect) - 1) * flat);
        const high = (round * round) / wide;
        return [wide * flare + WALL, high * flare + WALL];
      }),
      this.tinted,
    );
    this.tract.add(this.snorkel);
    // A rolled lip round the mouth, the flare's edge.
    const [wide, high] = [round * Math.sqrt(aspect) * 1.35 + WALL, (round / Math.sqrt(aspect)) * 1.35 + WALL];
    const rim = new THREE.Mesh(new THREE.TorusGeometry(1, 0.0035 / Math.min(wide, high), 8, 48), this.plastic);
    rim.scale.set(wide, high, Math.min(wide, high));
    rim.position.copy(l.mouth);
    rim.lookAt(l.mouth.clone().add(l.snorkel.getTangentAt(1)));
    this.tract.add(rim);
    // The dark of the throat, a little way in, so the mouth reads as open rather than lit inside.
    const back = l.snorkel.getTangentAt(0.93);
    const throat = new THREE.Mesh(new THREE.CircleGeometry(1, 32), this.throat);
    throat.scale.set(round * Math.sqrt(aspect) * 1.02, (round / Math.sqrt(aspect)) * 1.02, 1);
    throat.position.copy(l.snorkel.getPointAt(0.93));
    throat.lookAt(throat.position.clone().add(back));
    this.tract.add(throat);
  }

  /**
   * Colour the tract by its cells' gauge pressure, Pa, throttle first, on `scale`; with `null`, plain.
   * Each part takes the cells of the stretch of the solver's tract it is.
   */
  paint(cells: Float32Array | null, scale: number): void {
    if (!this.layout) return;
    const segs = this.layout.segments;
    const total = segs.reduce((s, x) => s + x.length, 0);
    const edges = [0, segs[0]!.length / total, (segs[0]!.length + segs[1]!.length) / total, 1];
    const rgb = new THREE.Color();
    const plain = (mesh: THREE.Mesh | null) => {
      const c = mesh?.geometry.getAttribute('color') as THREE.BufferAttribute | undefined;
      if (!c) return;
      (c.array as Float32Array).fill(1);
      c.needsUpdate = true;
    };
    if (!cells || !this.showPressure || cells.length === 0) {
      plain(this.tube);
      plain(this.box);
      plain(this.snorkel);
      return;
    }
    const cellAt = (f: number) => cells[Math.min(cells.length - 1, Math.max(0, Math.floor(f * cells.length)))]!;
    // Lighter than the ramp, so the dark plastic shows its colour.
    const lift = 1.6;
    const along = (mesh: THREE.Mesh | null, from: number, to: number) => {
      const c = mesh?.geometry.getAttribute('color') as THREE.BufferAttribute | undefined;
      if (!c) return;
      const arr = c.array as Float32Array;
      for (let i = 0; i <= ALONG; i++) {
        pressureColor(cellAt(from + (to - from) * (i / ALONG)) / scale, rgb);
        for (let j = 0; j <= AROUND; j++) {
          const k = (i * (AROUND + 1) + j) * 3;
          arr[k] = rgb.r * lift;
          arr[k + 1] = rgb.g * lift;
          arr[k + 2] = rgb.b * lift;
        }
      }
      c.needsUpdate = true;
    };
    along(this.tube, edges[0]!, edges[1]!);
    along(this.snorkel, edges[2]!, edges[3]!);
    // The airbox as one: the mean of its cells.
    const c = this.box?.geometry.getAttribute('color') as THREE.BufferAttribute | undefined;
    if (c) {
      const first = Math.floor(edges[1]! * cells.length);
      const last = Math.max(first + 1, Math.floor(edges[2]! * cells.length));
      let sum = 0;
      for (let i = first; i < last; i++) sum += cells[i]!;
      pressureColor(sum / (last - first) / scale, rgb);
      const arr = c.array as Float32Array;
      for (let k = 0; k < arr.length; k += 3) {
        arr[k] = rgb.r * lift;
        arr[k + 1] = rgb.g * lift;
        arr[k + 2] = rgb.b * lift;
      }
      c.needsUpdate = true;
    }
  }

  boundingBox(): THREE.Box3 {
    return new THREE.Box3().setFromObject(this.group);
  }

  private clear(): void {
    const drop = (g: THREE.Group) => {
      for (const child of [...g.children]) {
        if (child === this.tract) continue;
        g.remove(child);
        child.traverse((o) => {
          if (o instanceof THREE.Mesh) o.geometry.dispose();
        });
      }
    };
    drop(this.tract);
    drop(this.group);
    this.butterfly = null;
    this.tube = null;
    this.box = null;
    this.snorkel = null;
  }

  dispose(): void {
    this.clear();
    for (const m of [this.plastic, this.tinted, this.rubber, this.cast, this.clamp, this.plate, this.throat]) m.dispose();
  }
}
