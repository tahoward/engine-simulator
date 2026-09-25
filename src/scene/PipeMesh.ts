/**
 * Builds the exhaust geometry from `PipeSegment[]` and paints the travelling
 * pressure waves onto it as colour.
 *
 * The centreline is swept from the port as a polyline: each segment turns by its own yaw and
 * pitch where it starts, then runs straight. Pieces snap together at sharp, mitred corners the
 * way cut and welded tube does, so the user can fold a 1.4 m pipe into the viewport without
 * changing the acoustics — the 1D model only cares about area versus *axial distance*, which
 * turning does not alter.
 *
 * Cross-sections come from `segmentSection`, and areas from `segmentDiameter`, the same functions the
 * duct solver discretises, so what you see is the duct the sound is actually travelling down. A
 * chamber's body can be oval or rectangular, and its pipes can enter and leave off its centreline.
 */

import * as THREE from 'three';
import {
  CHAMBER_THROAT,
  type PipeSegment,
  type Section,
  chamberOffsets,
  sectionBoundary,
  segmentDiameter,
  segmentSection,
} from '../model/spec.js';
import { turnBetweenDirs, turnDir } from '../model/geometry.js';

/** Points round the tube. Enough to turn the corners of a rectangular can. */
const RADIAL = 40;
/** Centreline stations per metre of pipe. Enough to resolve a wave visually. */
const STATIONS_PER_M = 190;
/** Minimum stations in any one segment, so short segments still have a shape. */
const MIN_STATIONS = 4;

export interface Station {
  /** Distance along the centreline from the port, m. */
  x: number;
  position: THREE.Vector3;
  direction: THREE.Vector3;
  /** Radius, m: of the tube where it is round, and of the smallest circle enclosing it where not. */
  radius: number;
  /** Cross-section here. */
  section: Section;
  /** The section's width axis: horizontal and square to the tube, so a flat can lies flat. */
  across: THREE.Vector3;
  /** Index of the authoring segment this station belongs to. */
  segment: number;
  /**
   * At a corner, the normal of the plane the tube is cut in: the bisector of the directions in and out.
   * Both straight runs meet that plane in the same ellipse, which is what makes a mitre close.
   */
  mitre?: THREE.Vector3;
}

export interface PipeLayout {
  stations: Station[];
  totalLength: number;
  /** Centreline point at the end of each segment, for placing drag handles. */
  joints: THREE.Vector3[];
  /** Outward direction at each joint. */
  jointDirections: THREE.Vector3[];
  /** Radius at each joint, m. */
  jointRadii: number[];
}

/**
 * Sweep the centreline. Pure geometry, no three.js scene objects, so the pipe editor
 * can use it for hit-testing and handle placement without touching the mesh.
 */
export function layoutPipe(
  pipe: PipeSegment[],
  origin: THREE.Vector3,
  heading: THREE.Vector3,
): PipeLayout {
  const stations: Station[] = [];
  const joints: THREE.Vector3[] = [];
  const jointDirections: THREE.Vector3[] = [];
  const jointRadii: number[] = [];

  const pos = origin.clone();
  let dir = heading.clone().normalize();
  let x = 0;

  const station = (seg: PipeSegment, si: number, u: number, at: THREE.Vector3): Station => {
    const section = segmentSection(seg, u);
    return {
      x,
      position: at,
      direction: dir.clone(),
      radius: section.section === 'round' ? segmentDiameter(seg, u) / 2 : Math.max(section.width, section.height) / 2,
      section,
      across: acrossAxis(dir),
      segment: si,
    };
  };

  for (let si = 0; si < pipe.length; si++) {
    const seg = pipe[si]!;
    const count = Math.max(MIN_STATIONS, Math.round(seg.length * STATIONS_PER_M));
    const step = seg.length / count;

    // Turn where the segment starts, then run straight.
    const incoming = dir;
    dir = turnHeading(incoming, seg.yaw, seg.pitch);

    if (si === 0) {
      // The first segment's inlet; a turn here is the duct leaving its port or junction at an angle.
      stations.push(station(seg, si, 0, pos.clone()));
    } else if (incoming.angleTo(dir) > 1e-6) {
      // The previous segment's outlet station is this corner: cut it on the bisector.
      stations[stations.length - 1]!.mitre = incoming.clone().add(dir).normalize();
    }

    // An offset pipe enters the can off its centreline, so the body sits to one side of the pipe
    // coming in, and the pipe going out leaves from wherever its own offset puts it.
    const [offIn, offOut] = chamberOffsets(seg);
    const across = acrossAxis(dir);
    const lateral = (u: number) =>
      u < CHAMBER_THROAT ? 0 : u <= 1 - CHAMBER_THROAT ? -offIn : offOut - offIn;

    for (let k = 1; k <= count; k++) {
      pos.addScaledVector(dir, step);
      x += step;
      const u = k / count;
      stations.push(station(seg, si, u, pos.clone().addScaledVector(across, lateral(u))));
    }
    pos.addScaledVector(across, offOut - offIn);

    joints.push(pos.clone());
    jointDirections.push(dir.clone());
    jointRadii.push(segmentDiameter(seg, 1) / 2);
  }

  return { stations, totalLength: x, joints, jointDirections, jointRadii };
}


/**
 * `dir` turned by `yaw` about the vertical, then by `pitch` about its new horizontal right axis.
 *
 * The one convention every turn in the exhaust uses — a segment's corner, a duct leaving its port, a drawn
 * segment's fit — so they all agree. Pitching about a *horizontal* axis changes elevation by exactly
 * `pitch` and leaves the horizontal heading alone, which is what makes `turnBetween` an exact inverse.
 */
/** Horizontal and square to `dir`: the axis a chamber's width lies along. */
export function acrossAxis(dir: THREE.Vector3): THREE.Vector3 {
  const a = new THREE.Vector3(0, 1, 0).cross(dir);
  if (a.lengthSq() < 1e-8) return new THREE.Vector3(1, 0, 0).projectOnPlane(dir).normalize();
  return a.normalize();
}

export function turnHeading(dir: THREE.Vector3, yaw = 0, pitch = 0): THREE.Vector3 {
  return new THREE.Vector3(...turnDir([dir.x, dir.y, dir.z], yaw, pitch));
}

/** The yaw and pitch that `turnHeading` needs to turn `from` onto `to`. */
export function turnBetween(from: THREE.Vector3, to: THREE.Vector3): { yaw: number; pitch: number } {
  return turnBetweenDirs([from.x, from.y, from.z], [to.x, to.y, to.z]);
}

/**
 * Straight-line span from the pipe's inlet to its far end, m.
 *
 * Not the same as the sum of the segment lengths: a pipe with any corners in it doubles back on
 * itself a little, so its centreline is longer than the distance it actually covers. Layout needs the distance covered —
 * using the arc length instead makes every primary overshoot the junction it is aiming at.
 */
export function pipeSpan(pipe: PipeSegment[], heading: THREE.Vector3): number {
  const origin = new THREE.Vector3();
  const layout = layoutPipe(pipe, origin, heading);
  const end = layout.joints[layout.joints.length - 1];
  return end ? end.distanceTo(origin) : 0;
}

/**
 * Heading that puts the pipe's far end on `target`, as nearly as its own corners allow.
 *
 * Corners turn about world up and then the local right axis, so the shape is not a rigid body that
 * can be rotated into place — change the heading and the corners land differently.
 * Aiming the *inlet* at the target, which is what the layout used to do, therefore misses by the
 * whole of the corners: the pipe leaves in the right direction and then turns away.
 *
 * Solved by fixed-point iteration instead. Sweep, see where the end actually went, rotate the
 * heading by the rotation that would carry that point onto the target, repeat. The map is close
 * enough to a rotation that this converges in two or three passes; six is a cheap ceiling, and it
 * only runs when the geometry changes.
 */
export function solveHeading(
  pipe: PipeSegment[],
  origin: THREE.Vector3,
  target: THREE.Vector3,
  seed?: THREE.Vector3,
): THREE.Vector3 {
  const want = target.clone().sub(origin);
  if (want.lengthSq() < 1e-12) return (seed ?? new THREE.Vector3(1, 0, 0)).clone().normalize();
  const wantDir = want.clone().normalize();
  const dir = (seed ?? wantDir).clone().normalize();
  const q = new THREE.Quaternion();

  for (let iter = 0; iter < 6; iter++) {
    const layout = layoutPipe(pipe, origin, dir);
    const end = layout.joints[layout.joints.length - 1];
    if (!end) break;
    const have = end.clone().sub(origin);
    if (have.lengthSq() < 1e-12) break;
    const haveDir = have.normalize();
    if (haveDir.dot(wantDir) > 1 - 1e-12) break;
    q.setFromUnitVectors(haveDir, wantDir);
    dir.applyQuaternion(q).normalize();
  }
  return dir;
}

export class PipeMesh {
  readonly group = new THREE.Group();

  private mesh: THREE.Mesh | null = null;
  private geometry: THREE.BufferGeometry | null = null;
  private layout: PipeLayout | null = null;
  /** Ring offset in the vertex buffer for each station. */
  private stationCount = 0;

  private readonly material: THREE.MeshStandardMaterial;
  private showPressure = true;

  /**
   * Auto-ranging pressure scale, Pa. Decays slowly so the colours stay readable
   * across a muffler (a few kPa) and an open header (tens of kPa) without the user
   * having to adjust anything, but does not flicker frame to frame.
   */
  private pressureScale = 8000;

  constructor() {
    this.material = new THREE.MeshStandardMaterial({
      vertexColors: true,
      metalness: 0.55,
      roughness: 0.38,
      side: THREE.DoubleSide,
    });
  }

  get pipeLayout(): PipeLayout | null {
    return this.layout;
  }

  setPressureVisible(on: boolean): void {
    this.showPressure = on;
    if (!on) this.paintMetal();
  }

  /** Rebuild the tube. Called on every pipe edit, so it disposes as it goes. */
  rebuild(pipe: PipeSegment[], origin: THREE.Vector3, heading: THREE.Vector3): void {
    this.layout = layoutPipe(pipe, origin, heading);
    const stations = this.layout.stations;
    this.stationCount = stations.length;

    if (this.mesh) {
      this.group.remove(this.mesh);
      this.geometry?.dispose();
      this.mesh = null;
      this.geometry = null;
    }
    if (stations.length < 2) return;

    const vertCount = stations.length * (RADIAL + 1);
    const positions = new Float32Array(vertCount * 3);
    const normals = new Float32Array(vertCount * 3);
    const colors = new Float32Array(vertCount * 3);
    const uvs = new Float32Array(vertCount * 2);

    // A parallel-transported frame rather than a Frenet one: Frenet normals flip at
    // inflection points, which would twist the tube visibly.
    let normalRef = pickInitialNormal(stations[0]!.direction);
    let prevTangent = stations[0]!.direction;
    const turn = new THREE.Quaternion();

    for (let i = 0; i < stations.length; i++) {
      const st = stations[i]!;
      const tangent = st.direction;
      /**
       * Carry the normal round by the rotation between the two directions, then re-orthogonalise.
       *
       * Projecting it onto the new cross-section instead is only right for a small turn: at a sharp
       * corner it shortens the in-plane part by the cosine of the angle, the frame twists, and the
       * outgoing ring no longer lines up point for point with the mitre it has to close on.
       */
      turn.setFromUnitVectors(prevTangent, tangent);
      normalRef = normalRef.clone().applyQuaternion(turn).projectOnPlane(tangent).normalize();
      prevTangent = tangent;
      if (!Number.isFinite(normalRef.x) || normalRef.lengthSq() < 0.5) {
        normalRef = pickInitialNormal(tangent);
      }
      const binormal = new THREE.Vector3().crossVectors(tangent, normalRef).normalize();

      /**
       * At a corner, slide each ring point along the tube until it meets the mitre plane.
       *
       * The incoming run meets the bisecting plane in an ellipse, and so does the outgoing one — the
       * same ellipse — so the two runs close on it with no gap and no overlap, as a mitred weld does.
       * The frame then carries on round the corner from the outgoing direction, on the next station.
       */
      const mitre = st.mitre;
      const along = mitre ? tangent.dot(mitre) : 1;
      const shaped = st.section.section !== 'round';
      // The section's own axes, for placing a shaped ring: width across, height square to it.
      const height = shaped ? new THREE.Vector3().crossVectors(tangent, st.across) : null;

      for (let j = 0; j <= RADIAL; j++) {
        const a = (j / RADIAL) * Math.PI * 2;
        const cos = Math.cos(a);
        const sin = Math.sin(a);
        // Ring points keep the transported frame's directions whatever the shape, so each one joins
        // the same point on the next ring and a round throat meets its shaped body without a twist.
        const dx = normalRef.x * cos + binormal.x * sin;
        const dy = normalRef.y * cos + binormal.y * sin;
        const dz = normalRef.z * cos + binormal.z * sin;
        let r = st.radius;
        let nx = dx;
        let ny = dy;
        let nz = dz;
        if (height) {
          const ay = dx * st.across.x + dy * st.across.y + dz * st.across.z;
          const az = dx * height.x + dy * height.y + dz * height.z;
          sectionBoundary(st.section, Math.atan2(az, ay), BOUNDARY);
          r = BOUNDARY[0]!;
          nx = st.across.x * BOUNDARY[1]! + height.x * BOUNDARY[2]!;
          ny = st.across.y * BOUNDARY[1]! + height.y * BOUNDARY[2]!;
          nz = st.across.z * BOUNDARY[1]! + height.z * BOUNDARY[2]!;
        }

        let slide = 0;
        if (mitre && Math.abs(along) > 1e-3) {
          slide = -((dx * mitre.x + dy * mitre.y + dz * mitre.z) * r) / along;
        }

        const vi = (i * (RADIAL + 1) + j) * 3;
        positions[vi] = st.position.x + dx * r + tangent.x * slide;
        positions[vi + 1] = st.position.y + dy * r + tangent.y * slide;
        positions[vi + 2] = st.position.z + dz * r + tangent.z * slide;
        normals[vi] = nx;
        normals[vi + 1] = ny;
        normals[vi + 2] = nz;
        colors[vi] = 0.55;
        colors[vi + 1] = 0.58;
        colors[vi + 2] = 0.62;

        const ui = (i * (RADIAL + 1) + j) * 2;
        uvs[ui] = st.x;
        uvs[ui + 1] = j / RADIAL;
      }
    }

    const indices: number[] = [];
    for (let i = 0; i < stations.length - 1; i++) {
      for (let j = 0; j < RADIAL; j++) {
        const a = i * (RADIAL + 1) + j;
        const b = a + RADIAL + 1;
        indices.push(a, b, a + 1, b, b + 1, a + 1);
      }
    }

    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geom.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
    geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geom.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
    geom.setIndex(indices);
    geom.computeBoundingSphere();

    const mesh = new THREE.Mesh(geom, this.material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    this.geometry = geom;
    this.mesh = mesh;
    this.group.add(mesh);

    if (!this.showPressure) this.paintMetal();
  }

  /** The tube mesh, for raycasting. Null before the first rebuild. */
  get pickTarget(): THREE.Mesh | null {
    return this.mesh;
  }

  /** Axial distance from the port, m, for a point on the tube surface. */
  stationAt(point: THREE.Vector3): { x: number; segment: number } | null {
    if (!this.layout) return null;
    let best = 0;
    let bestD = Infinity;
    for (let i = 0; i < this.layout.stations.length; i++) {
      const d = this.layout.stations[i]!.position.distanceToSquared(point);
      if (d < bestD) {
        bestD = d;
        best = i;
      }
    }
    const st = this.layout.stations[best]!;
    return { x: st.x, segment: st.segment };
  }

  boundingBox(): THREE.Box3 {
    const box = new THREE.Box3();
    if (this.layout) for (const st of this.layout.stations) box.expandByPoint(st.position);
    box.expandByScalar(0.08);
    return box;
  }

  /**
   * Paint the current pressure distribution.
   *
   * @param pressure Gauge pressure along the pipe, Pa, port to mouth.
   */
  update(pressure: Float32Array): void {
    if (!this.geometry || !this.layout || !this.showPressure) return;

    let peak = 0;
    for (const v of pressure) {
      const a = Math.abs(v);
      if (a > peak) peak = a;
    }
    // Rise fast, fall slowly, with a floor so an idling muffled engine still shows
    // something rather than amplifying numerical dust into a light show.
    this.pressureScale =
      peak > this.pressureScale
        ? peak
        : Math.max(1500, this.pressureScale * 0.985 + peak * 0.015);

    const colors = this.geometry.getAttribute('color') as THREE.BufferAttribute;
    const arr = colors.array as Float32Array;
    const taps = pressure.length;
    const rgb = new THREE.Color();

    for (let i = 0; i < this.stationCount; i++) {
      const st = this.layout.stations[i]!;
      // Map station position to a tap by normalised distance along the pipe.
      const u = this.layout.totalLength > 0 ? st.x / this.layout.totalLength : 0;
      const tap = Math.min(taps - 1, Math.max(0, Math.round(u * (taps - 1))));
      pressureColor(pressure[tap]! / this.pressureScale, rgb);

      const base = i * (RADIAL + 1) * 3;
      for (let j = 0; j <= RADIAL; j++) {
        const vi = base + j * 3;
        arr[vi] = rgb.r;
        arr[vi + 1] = rgb.g;
        arr[vi + 2] = rgb.b;
      }
    }
    colors.needsUpdate = true;

  }

  private paintMetal(): void {
    if (!this.geometry) return;
    const colors = this.geometry.getAttribute('color') as THREE.BufferAttribute;
    const arr = colors.array as Float32Array;
    for (let i = 0; i < arr.length; i += 3) {
      arr[i] = 0.55;
      arr[i + 1] = 0.58;
      arr[i + 2] = 0.62;
    }
    colors.needsUpdate = true;
  }

  dispose(): void {
    this.geometry?.dispose();
    this.material.dispose();
  }
}

/**
 * Diverging colour ramp for gauge pressure: compression warm, rarefaction cool.
 * Diverging is the right family here because zero is a meaningful midpoint — it is
 * ambient pressure, not merely the middle of the data range.
 *
 * The midpoint is neutral *metal grey*, not black. Vertex colours multiply the base
 * colour, so a dark midpoint makes an idling pipe look unlit rather than quiet.
 */
function pressureColor(t: number, out: THREE.Color): void {
  const c = Math.max(-1, Math.min(1, t));
  const m = Math.abs(c);
  const base = 0.55;
  if (c >= 0) {
    // grey -> amber -> white-hot
    out.setRGB(base + (1 - base) * m, base - 0.12 * m + 0.32 * m * m, base - 0.45 * m);
  } else {
    // grey -> teal -> pale cyan
    out.setRGB(base - 0.45 * m, base - 0.05 * m + 0.3 * m * m, base + (1 - base) * m);
  }
}

const BOUNDARY = new Float64Array(3);

function pickInitialNormal(tangent: THREE.Vector3): THREE.Vector3 {
  const up = new THREE.Vector3(0, 1, 0);
  const n = up.clone().projectOnPlane(tangent);
  if (n.lengthSq() < 1e-6) return new THREE.Vector3(1, 0, 0).projectOnPlane(tangent).normalize();
  return n.normalize();
}
