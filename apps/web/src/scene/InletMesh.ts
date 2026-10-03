/**
 * The intake as a car has it: a cast plenum on the engine, a throttle body with its butterfly turning with
 * the throttle, a black rubber tube with a bellows coupler and hose clamps, a black plastic airbox with its
 * lid's seam and clips, and a flattened snorkel flaring at its mouth. Where each goes is `inletLayout`.
 *
 * In the pressure view the plenum and an inline engine's runners take the colour of their gauge pressure,
 * on the same scale as the exhaust, so a colour is the same pressure anywhere in the engine: the plenum
 * deep in vacuum at a small throttle. The tube, the airbox and the snorkel stay their own black. And the air itself is shown
 * moving through the tract, as specks carried at the solver's speed in each cell, slowed by `FLOW_SLOWDOWN`
 * so the eye can follow them: drawn in steadily at full throttle, and stopped and sent back by every wave.
 */

import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/examples/jsm/geometries/RoundedBoxGeometry.js';
import { mergeGeometries, toCreasedNormals } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

import type { EngineSnapshot, EngineSpec } from '../model/spec.js';
import { PLENUM_ROUNDING, PLENUM_WALL, taperAlong, type PlenumTaper, SNORKEL_ASPECT, SNORKEL_WALL, THROTTLE_WALL, inletLayout, type InletLayout } from './inletLayout.js';
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
/** How far apart the tube's stations are, m: close enough to round each rib. */
const RIB_STATION = 0.0015;

/** How much wider a runner flares where it meets the plenum, as a fraction of its bore, and over how many
 * of its radii, like a cast fillet outside and a bellmouth in. */
const RUNNER_FLARE = 0.5;
const RUNNER_FLARE_RADII = 1.4;

/** A runner's radius `s` m from where it leaves the plenum's inside wall, flaring by `flare` of its bore. */
function runnerRadius(radius: number, s: number, flare: number): number {
  const reach = RUNNER_FLARE_RADII * radius;
  return s >= reach ? radius : radius * (1 + flare * (1 - s / reach) ** 2);
}

/** Wall thickness of the airbox's moulding, m. */
const AIRBOX_WALL = 0.004;

/** Wall thickness of the rubber tube, m: how far it stands out of its bore. */
const WALL = 0.004;

/** How many specks of air are shown moving through the tract. */
const SPECKS = 220;

/** How many times slower than the air the specks move: at 20 m/s it would cross a metre in 50 ms. */
const FLOW_SLOWDOWN = 25;

/** Give `g` a white vertex colour, for the pressure view to paint over. */
function colourable<T extends THREE.BufferGeometry>(g: T): T {
  const white = new Float32Array(g.getAttribute('position').count * 3).fill(1);
  g.setAttribute('color', new THREE.BufferAttribute(white, 3));
  return g;
}

/** The exhaust pipes' metal, as `PipeMesh` paints it into their vertex colours. */
const PIPE_METAL = new THREE.Color(0.55, 0.58, 0.62);

/** Fill `g`'s vertex colours with `rgb`. */
function fill(g: THREE.BufferGeometry, rgb: THREE.Color): void {
  const c = g.getAttribute('color') as THREE.BufferAttribute | undefined;
  if (!c) return;
  const arr = c.array as Float32Array;
  for (let k = 0; k < arr.length; k += 3) {
    arr[k] = rgb.r;
    arr[k + 1] = rgb.g;
    arr[k + 2] = rgb.b;
  }
  c.needsUpdate = true;
}

/**
 * A tube swept along `curve`, its section an ellipse `size(u)` returns the half-width (across, level) and
 * half-height of, at each fraction `u` along it. The frame is kept level: across is the curve's direction
 * crossed with up, which suits a tract that runs about level.
 */
function sweep(
  curve: THREE.Curve<THREE.Vector3>,
  size: (u: number) => [number, number],
  bend = false,
  along = ALONG,
): THREE.BufferGeometry {
  const positions: number[] = [];
  const normals: number[] = [];
  const index: number[] = [];
  const up = new THREE.Vector3(0, 1, 0);
  // A round tube bending through the vertical, as a runner turning down into its port does, is carried
  // round its bend without twisting: the frame a bent pipe has.
  const frames = bend ? curve.computeFrenetFrames(along, false) : null;
  let was = new THREE.Vector3(1, 0, 0);
  for (let i = 0; i <= along; i++) {
    const u = i / along;
    const at = curve.getPointAt(u);
    const t = curve.getTangentAt(u).normalize();
    const across = frames ? frames.normals[i]!.clone() : new THREE.Vector3().crossVectors(t, up);
    // Running straight up or down, across is kept as it last was, so the tube does not twist there.
    if (across.lengthSq() < 1e-6) across.copy(was);
    across.normalize();
    was = across.clone();
    const lift = frames ? frames.binormals[i]!.clone().normalize() : new THREE.Vector3().crossVectors(across, t).normalize();
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
    }
  }
  for (let i = 0; i < along; i++) {
    for (let j = 0; j < AROUND; j++) {
      const a = i * (AROUND + 1) + j;
      const b = a + AROUND + 1;
      index.push(a, b, a + 1, b, b + 1, a + 1);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  g.setAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));
  g.setIndex(index);
  return g;
}

/**
 * The plenum's section, centred on the origin, `width` by `height`, as points round it anticlockwise:
 * square-shouldered down past the throttle body's centre, then narrowing to `base` of its width at the
 * bottom, its corners rounded by `round`. A `base` of 1 is a rounded rectangle. `inset` draws its +x side in
 * by that much, or its -x side where it is negative, and `drop` lowers its top by that much. `faces` gives, for each of its straight faces a runner
 * can leave by, the two points the face runs between, in the order the points go round: the upright sides,
 * and a tapered section's sloping flanks below them.
 */
function plenumSection(
  width: number,
  height: number,
  base: number,
  round: number,
  inset = 0,
  drop = 0,
): { points: THREE.Vector2[]; faces: Partial<Record<Face, [number, number]>> } {
  // The corners on the outline itself: each is rounded off by cutting `round` back along both its edges.
  const [w, h] = [width / 2, height / 2];
  const [right, left] = [w - Math.max(inset, 0), -w + Math.max(-inset, 0)];
  const top = h - drop;
  const foot = (width / 2) * base;
  const corners: [number, number][] =
    base >= 1
      ? [
          [right, top],
          [left, top],
          [left, -h],
          [right, -h],
        ]
      : [
          [w, top],
          [-w, top],
          [-w, -h * 0.3],
          [-foot, -h],
          [foot, -h],
          [w, -h * 0.3],
        ];
  const points: THREE.Vector2[] = [];
  const first: number[] = [];
  const last: number[] = [];
  corners.forEach(([x, y], i) => {
    const [px, py] = corners[(i + corners.length - 1) % corners.length]!;
    const [nx, ny] = corners[(i + 1) % corners.length]!;
    const c = new THREE.Vector2(x, y);
    const a = c.clone().addScaledVector(new THREE.Vector2(x - px, y - py).normalize(), -round);
    const b = c.clone().addScaledVector(new THREE.Vector2(nx - x, ny - y).normalize(), round);
    first.push(points.length);
    for (let k = 0; k <= 8; k++) {
      const t = k / 8;
      points.push(
        a.clone().multiplyScalar((1 - t) * (1 - t)).addScaledVector(c, 2 * (1 - t) * t).addScaledVector(b, t * t),
      );
    }
    last.push(points.length - 1);
  });
  // The right side runs up from the last corner to the first; the left down from the second to the third;
  // a tapered section's left flank on down to the fourth, its right one up from the fifth, and the
  // underside between them, or a box's along its bottom.
  const faces: Partial<Record<Face, [number, number]>> = {
    right: [last[corners.length - 1]!, first[0]!],
    left: [last[1]!, first[2]!],
  };
  if (base < 1) {
    faces.leftFlank = [last[2]!, first[3]!];
    faces.bottom = [last[3]!, first[4]!];
    faces.rightFlank = [last[4]!, first[5]!];
  } else {
    faces.bottom = [last[2]!, first[3]!];
  }
  return { points, faces };
}

/**
 * The section of a plenum `size` across, high and long, narrowing towards its back by `taper`, `z` m back
 * from its middle, drawn in by `inset` all round, its corners rounded by `round`.
 */
function taperedSection(
  size: THREE.Vector3,
  base: number,
  taper: PlenumTaper | null,
  z: number,
  inset = 0,
  round = Math.max(PLENUM_ROUNDING - inset, 1e-4),
): ReturnType<typeof plenumSection> {
  const f = taper ? taperAlong(size.z, z) : 0;
  const [side, drop] = taper ? [taper.side * taper.inwards * f, taper.drop * f] : [0, 0];
  return plenumSection(size.x - 2 * inset, size.y - 2 * inset, base, round, side, drop);
}

/** A straight face of the plenum a runner can leave by: an upright side, or a tapered section's flank. */
type Face = 'right' | 'left' | 'rightFlank' | 'leftFlank' | 'bottom';

/**
 * A runner's opening in the plenum: the face it is in, and the runner's axis through it, in the plenum's
 * section, from where it starts at the inside face, and how far along the plenum it is.
 */
interface Opening {
  face: Face;
  start: THREE.Vector2;
  direction: THREE.Vector2;
  z: number;
  /** Its radius in the wall's inside face and in its outside one, m: the runner's flare where each is. */
  inside: number;
  outside: number;
}

/** Where along a face, from its first point, the line from `start` along `direction` crosses it, m. */
function alongFace(p: THREE.Vector2, q: THREE.Vector2, start: THREE.Vector2, direction: THREE.Vector2): number {
  const e = q.clone().sub(p);
  // p + e s = start + direction t, for s.
  const det = e.x * -direction.y - e.y * -direction.x;
  const d = start.clone().sub(p);
  return ((d.x * -direction.y - d.y * -direction.x) / det) * e.length();
}

/**
 * The plenum as a hollow casting, centred on the origin, `size` along x, y and z: a shell `PLENUM_WALL`
 * thick round its section, run along z, narrowing towards the back by `taper` (`PlenumTaper`), and rounded
 * over at each end as its sides are. It is closed at the back, and at the front with a throttle body's
 * bore, `bore` across, through it at each of `bores`, how far across from the section's middle each is.
 * Dual plenums' wall down its middle stands between its ends, with a hole through it for each of
 * `balances`, where along it and how high, and its radius. Where runners leave it, the upright side
 * they leave by is a plate of its own, inside and out, with an opening for each, so the runners can be seen
 * into from inside. Its rounded edges and corners shade smoothly, as a pipe's curve does, and its square ones
 * stay sharp.
 */
function plenumGeometry(
  size: THREE.Vector3,
  base: number,
  bore: number,
  bores: number[],
  openings: Opening[],
  taper: PlenumTaper | null,
  balances: { z: number; y: number; radius: number }[] | null,
): THREE.BufferGeometry {
  // Where its sides run straight between its rounded ends, and its section anywhere along there, drawn in by
  // `inset` all round: by its wall, inside.
  const [front, back] = [-size.z / 2 + PLENUM_ROUNDING, size.z / 2 - PLENUM_ROUNDING];
  const insideRound = Math.max(PLENUM_ROUNDING - PLENUM_WALL, 0.002);
  const sectionAt = (z: number, inset: number, round?: number) => taperedSection(size, base, taper, z, inset, round);
  const outer = sectionAt(front, 0);
  const n = outer.points.length;
  const open = (['right', 'left', 'rightFlank', 'leftFlank', 'bottom'] as const).filter((face) =>
    openings.some((o) => o.face === face),
  );
  const parts: THREE.BufferGeometry[] = [];

  /**
   * Rings of points round it, each joined to the next: of the section's points `run` goes through, or round
   * it whole. Wound to face out, or with `inward` in.
   */
  const loft = (rings: { points: THREE.Vector2[]; z: number }[], inward: boolean, run?: number[]) => {
    const order = run ?? [...Array.from({ length: n }, (_, i) => i), 0];
    const positions: number[] = [];
    for (let k = 0; k + 1 < rings.length; k++) {
      const [a, b] = [rings[k]!, rings[k + 1]!];
      // Anticlockwise round the section, so towards +z it faces out as given, and towards -z the other way.
      const flip = b.z > a.z === inward;
      for (let m = 0; m + 1 < order.length; m++) {
        const [i, j] = [order[m]!, order[m + 1]!];
        const corners: [THREE.Vector2, number][] = [
          [a.points[i]!, a.z],
          [a.points[j]!, a.z],
          [b.points[j]!, b.z],
          [b.points[i]!, b.z],
        ];
        for (const c of flip ? [0, 2, 1, 0, 3, 2] : [0, 1, 2, 0, 2, 3]) {
          const [p, z] = corners[c]!;
          positions.push(p.x, p.y, z);
        }
      }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    g.setAttribute('uv', new THREE.Float32BufferAttribute(new Float32Array((positions.length / 3) * 2), 2));
    g.computeVertexNormals();
    parts.push(g);
  };
  const along = (inset: number, round?: number) => [front, back].map((z) => ({ points: sectionAt(z, inset, round).points, z }));

  // The sides: whole, or in pieces between the faces runners leave by.
  if (open.length === 0) {
    loft(along(0), false);
    loft(along(PLENUM_WALL, insideRound), true);
  } else {
    // Each piece runs anticlockwise from where the outline leaves one open face to where it reaches the next.
    const ends = open.map((face) => outer.faces[face]!);
    ends.sort((p, q) => p[0] - q[0]);
    ends.forEach(([, from], k) => {
      const to = ends[(k + 1) % ends.length]![0];
      if (to === from) return;
      const run: number[] = [];
      for (let i = from; ; i = (i + 1) % n) {
        run.push(i);
        if (i === to) break;
      }
      loft(along(0), false, run);
      loft(along(PLENUM_WALL, insideRound), true, run);
    });
    // Each open face, out and in: a plate across it with each runner's opening through it, where the runner's
    // axis crosses it.
    for (const face of open) {
      for (const inset of [0, PLENUM_WALL]) {
        const round = inset > 0 ? insideRound : undefined;
        const ends = (z: number) => {
          const section = sectionAt(z, inset, round);
          return section.faces[face]!.map((i) => section.points[i]!);
        };
        const [pf, qf] = ends(front);
        const [pb, qb] = ends(back);
        // From its end that stays put, along to the one that falls with the top towards the back, if one does.
        const [p, q, qBack] = pf!.distanceTo(pb!) < 1e-9 ? [pf!, qf!, qb!] : [qf!, pf!, pb!];
        const plate = new THREE.Shape([
          new THREE.Vector2(front, 0),
          new THREE.Vector2(back, 0),
          new THREE.Vector2(back, p.distanceTo(qBack)),
          new THREE.Vector2(front, p.distanceTo(q)),
        ]);
        for (const o of openings.filter((o) => o.face === face)) {
          const hole = new THREE.Path();
          const at = alongFace(p, q, o.start, o.direction);
          hole.absarc(o.z, at, inset > 0 ? o.inside : o.outside, 0, Math.PI * 2, true);
          plate.holes.push(hole);
        }
        // Drawn in its own plane, along z and along the face, then laid on the face.
        const e = q.clone().sub(p).normalize();
        const g = new THREE.ShapeGeometry(plate, 24).toNonIndexed();
        g.applyMatrix4(
          new THREE.Matrix4()
            .makeBasis(new THREE.Vector3(0, 0, 1), new THREE.Vector3(e.x, e.y, 0), new THREE.Vector3(e.y, -e.x, 0))
            .setPosition(p.x, p.y, 0),
        );
        parts.push(g);
      }
    }
  }

  // Each end rounded over as its sides are: the section there drawn in a little further at each step round
  // the quarter, to a plate across the end, the front one with the throttle body's bore through it. Inside, a
  // plate closes each end where the sides stop, the bore's rim between the two at the front.
  const bored = (points: THREE.Vector2[], hole: boolean) => {
    const shape = new THREE.Shape(points);
    if (hole) for (const x of bores) shape.holes.push(new THREE.Path().absarc(x, 0, bore / 2, 0, Math.PI * 2, true));
    return new THREE.ShapeGeometry(shape, 24).toNonIndexed();
  };
  for (const [z0, side] of [
    [back, 1],
    [front, -1],
  ] as const) {
    const rings = Array.from({ length: END_ARC + 1 }, (_, k) => {
      const t = (k / END_ARC) * (Math.PI / 2);
      const d = PLENUM_ROUNDING * (1 - Math.cos(t));
      return { points: sectionAt(z0, d).points, z: z0 + side * PLENUM_ROUNDING * Math.sin(t) };
    });
    loft(rings, false);
    const end = bored(rings[END_ARC]!.points, side < 0);
    end.translate(0, 0, side * (size.z / 2));
    parts.push(end);
    const inside = bored(sectionAt(z0, PLENUM_WALL, insideRound).points, side < 0);
    inside.translate(0, 0, z0);
    parts.push(inside);
  }
  for (const x of bores) {
    const rim = new THREE.CylinderGeometry(bore / 2, bore / 2, PLENUM_ROUNDING, 32, 1, true).toNonIndexed();
    rim.rotateX(Math.PI / 2);
    rim.translate(x, 0, front - PLENUM_ROUNDING / 2);
    parts.push(rim);
  }
  // The wall between dual plenums: across the inside from end to end and from the floor to the top, which
  // falls with the taper towards the back, drawn in its own plane, along z and up, then stood in the middle.
  if (balances) {
    const [h, drop] = [size.y / 2 - PLENUM_WALL, taper ? taper.drop : 0];
    const topAt = (z: number) => h - drop * taperAlong(size.z, z);
    const wall = new THREE.Shape([
      new THREE.Vector2(front, -h),
      new THREE.Vector2(back, -h),
      new THREE.Vector2(back, topAt(back)),
      new THREE.Vector2(front, topAt(front)),
    ]);
    for (const b of balances) wall.holes.push(new THREE.Path().absarc(b.z, b.y, b.radius, 0, Math.PI * 2, true));
    const g = new THREE.ShapeGeometry(wall, 24).toNonIndexed();
    g.applyMatrix4(new THREE.Matrix4().makeBasis(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 1, 0), new THREE.Vector3(1, 0, 0)));
    parts.push(g);
  }
  return toCreasedNormals(mergeGeometries(parts), Math.PI / 5);
}

/** How many steps round the quarter the plenum's ends are rounded over in. */
const END_ARC = 6;

/** A round hole through one of a box's faces: the face, by its outward axis, and the hole's middle and radius, m. */
interface BoxHole {
  axis: 0 | 1 | 2;
  sign: 1 | -1;
  centre: THREE.Vector3;
  radius: number;
}

/** Turn each of `g`'s triangles to wind anticlockwise seen from the way its vertex normals point. */
function windOut(g: THREE.BufferGeometry): THREE.BufferGeometry {
  const p = g.getAttribute('position') as THREE.BufferAttribute;
  const n = g.getAttribute('normal') as THREE.BufferAttribute;
  const [a, b, c, e1, e2, m] = [0, 0, 0, 0, 0, 0].map(() => new THREE.Vector3());
  for (let i = 0; i < p.count; i += 3) {
    a.fromBufferAttribute(p, i);
    b.fromBufferAttribute(p, i + 1);
    c.fromBufferAttribute(p, i + 2);
    m.fromBufferAttribute(n, i);
    if (e1.subVectors(b, a).cross(e2.subVectors(c, a)).dot(m) >= 0) continue;
    for (const attr of [p, n]) {
      const [x, y, z] = [attr.getX(i + 1), attr.getY(i + 1), attr.getZ(i + 1)];
      attr.setXYZ(i + 1, attr.getX(i + 2), attr.getY(i + 2), attr.getZ(i + 2));
      attr.setXYZ(i + 2, x, y, z);
    }
  }
  return g;
}

/**
 * The surface of a box centred on the origin, `half` its size each way along x, y and z, every edge and
 * corner rounded by `round`, with `holes` through its flat faces; its normals outwards, or with `inward`
 * in, as the inside of a shell.
 */
function roundedBoxSurface(half: THREE.Vector3, round: number, holes: BoxHole[], inward: boolean): THREE.BufferGeometry {
  const parts: THREE.BufferGeometry[] = [];
  const flip = inward ? -1 : 1;
  const axes = [new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 1)];
  const flat = [half.x - round, half.y - round, half.z - round];
  const ARC = 8;
  // The flat faces, each with any holes through it.
  for (const axis of [0, 1, 2] as const) {
    const [u, v] = [(axis + 1) % 3, (axis + 2) % 3];
    for (const sign of [1, -1] as const) {
      const face = new THREE.Shape([
        new THREE.Vector2(-flat[u]!, -flat[v]!),
        new THREE.Vector2(flat[u]!, -flat[v]!),
        new THREE.Vector2(flat[u]!, flat[v]!),
        new THREE.Vector2(-flat[u]!, flat[v]!),
      ]);
      for (const h of holes.filter((h) => h.axis === axis && h.sign === sign)) {
        const hole = new THREE.Path();
        hole.absarc(h.centre.getComponent(u), h.centre.getComponent(v), h.radius, 0, Math.PI * 2, true);
        face.holes.push(hole);
      }
      const g = new THREE.ShapeGeometry(face, 32).toNonIndexed();
      const pos = g.getAttribute('position') as THREE.BufferAttribute;
      const normal = new Float32Array(pos.count * 3);
      for (let i = 0; i < pos.count; i++) {
        const q = new THREE.Vector3()
          .addScaledVector(axes[u]!, pos.getX(i))
          .addScaledVector(axes[v]!, pos.getY(i))
          .addScaledVector(axes[axis]!, sign * half.getComponent(axis));
        pos.setXYZ(i, q.x, q.y, q.z);
        normal[i * 3 + axis] = sign * flip;
      }
      g.deleteAttribute('uv');
      g.setAttribute('normal', new THREE.BufferAttribute(normal, 3));
      parts.push(g);
    }
  }
  // A quarter round each edge, and an eighth of a ball each corner: the points `round` out from the flat
  // box's edges and corners, every way in between.
  const patch = (rows: THREE.Vector3[][], centreAt: (i: number, j: number) => THREE.Vector3) => {
    const positions: number[] = [];
    const normals: number[] = [];
    const vertex = (i: number, j: number) => {
      const d = rows[i]![j]!.clone().normalize();
      const q = centreAt(i, j).addScaledVector(d, round);
      positions.push(q.x, q.y, q.z);
      normals.push(d.x * flip, d.y * flip, d.z * flip);
    };
    // Between each row and the next, which is as long or one shorter.
    for (let i = 0; i + 1 < rows.length; i++) {
      for (let j = 0; j + 1 < rows[i]!.length; j++) {
        vertex(i, j);
        vertex(i + 1, j);
        vertex(i, j + 1);
        if (j + 1 < rows[i + 1]!.length) {
          vertex(i, j + 1);
          vertex(i + 1, j);
          vertex(i + 1, j + 1);
        }
      }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    g.setAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));
    parts.push(g);
  };
  for (const along of [0, 1, 2] as const) {
    const [a, b] = [(along + 1) % 3, (along + 2) % 3];
    for (const sa of [1, -1]) {
      for (const sb of [1, -1]) {
        // Rows along the edge, each round the quarter from one face to the other.
        const rows = [-1, 1].map(() =>
          Array.from({ length: ARC + 1 }, (_, k) => {
            const t = (k / ARC) * (Math.PI / 2);
            return axes[a]!.clone().multiplyScalar(sa * Math.cos(t)).addScaledVector(axes[b]!, sb * Math.sin(t));
          }),
        );
        patch(rows, (i) =>
          new THREE.Vector3()
            .addScaledVector(axes[a]!, sa * flat[a]!)
            .addScaledVector(axes[b]!, sb * flat[b]!)
            .addScaledVector(axes[along]!, (i === 0 ? -1 : 1) * flat[along]!),
        );
      }
    }
  }
  for (const sx of [1, -1]) {
    for (const sy of [1, -1]) {
      for (const sz of [1, -1]) {
        // Rows from the corner's x-most edge up to its pole on z, each a little shorter than the last.
        const rows = Array.from({ length: ARC + 1 }, (_, i) =>
          Array.from({ length: ARC + 1 - i }, (_, j) => new THREE.Vector3(sx * (ARC - i - j), sy * j, sz * i)),
        );
        const centre = new THREE.Vector3(sx * flat[0]!, sy * flat[1]!, sz * flat[2]!);
        patch(rows, () => centre.clone());
      }
    }
  }
  return windOut(mergeGeometries(parts));
}

/**
 * The airbox as a hollow moulding, centred on the origin, `size` along x, y and z: a shell `wall` thick, its
 * edges and corners rounded by `round`, open through `holes`, each with its rim the wall's thickness.
 */
function airboxGeometry(size: THREE.Vector3, round: number, wall: number, holes: BoxHole[]): THREE.BufferGeometry {
  const half = size.clone().multiplyScalar(0.5);
  const inner = half.clone().subScalar(wall);
  const parts = [
    roundedBoxSurface(half, round, holes, false),
    roundedBoxSurface(inner, Math.max(round - wall, 0.001), holes.map((h) => ({ ...h, centre: h.centre.clone().setComponent(h.axis, h.sign * inner.getComponent(h.axis)) })), true),
  ];
  for (const h of holes) {
    const rim = new THREE.CylinderGeometry(h.radius, h.radius, wall, 32, 1, true).toNonIndexed();
    rim.deleteAttribute('uv');
    // Facing in, towards the hole's axis.
    const n = rim.getAttribute('normal') as THREE.BufferAttribute;
    for (let i = 0; i < n.count; i++) n.setXYZ(i, -n.getX(i), -n.getY(i), -n.getZ(i));
    const axis = new THREE.Vector3().setComponent(h.axis, h.sign);
    rim.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), axis));
    const at = h.centre.clone().setComponent(h.axis, h.sign * (half.getComponent(h.axis) - wall / 2));
    rim.translate(at.x, at.y, at.z);
    parts.push(windOut(rim));
  }
  return mergeGeometries(parts);
}

/** How far the airbox's lid lip stands out of its sides, and how tall it is, m. */
const LIP_OUT = 0.005;
const LIP_HEIGHT = 0.008;

/**
 * The lip round the airbox at its lid's seam, `y` up from its middle: a band standing out of its sides, round
 * its rounded corners, centred on the origin as the airbox is. It stops short either side of any of `holes`
 * it would run across, `radius` there the collar round each.
 */
function airboxLip(size: THREE.Vector3, round: number, y: number, holes: BoxHole[]): THREE.BufferGeometry {
  // Round the outline of the sides, a point every few millimetres, with the way out from the box at each.
  const [x0, z0] = [size.x / 2 - round, size.z / 2 - round];
  const ring: { at: THREE.Vector2; out: THREE.Vector2 }[] = [];
  const corners: [number, number, number][] = [
    [x0, -z0, -Math.PI / 2],
    [x0, z0, 0],
    [-x0, z0, Math.PI / 2],
    [-x0, -z0, Math.PI],
  ];
  const step = 0.003;
  corners.forEach(([cx, cz, from], k) => {
    const arc = Math.max(2, Math.ceil(((Math.PI / 2) * round) / step));
    for (let i = 0; i <= arc; i++) {
      const t = from + (i / arc) * (Math.PI / 2);
      const out = new THREE.Vector2(Math.cos(t), Math.sin(t));
      ring.push({ at: new THREE.Vector2(cx, cz).addScaledVector(out, round), out });
    }
    // Then straight along the side to the next corner.
    const [nx, nz, nfrom] = corners[(k + 1) % 4]!;
    const out = new THREE.Vector2(Math.cos(nfrom), Math.sin(nfrom));
    const a = new THREE.Vector2(cx, cz).addScaledVector(out, round);
    const b = new THREE.Vector2(nx, nz).addScaledVector(out, round);
    const n = Math.ceil(a.distanceTo(b) / step);
    for (let i = 1; i < n; i++) ring.push({ at: a.clone().lerp(b, i / n), out });
  });
  const crosses = (p: THREE.Vector2) =>
    holes.some((h) => {
      const at = new THREE.Vector3(p.x, y, p.y);
      const off = at.sub(h.centre).setComponent(h.axis, 0);
      return off.length() < h.radius;
    });
  const positions: number[] = [];
  const normals: number[] = [];
  const quad = (a: THREE.Vector3, b: THREE.Vector3, c: THREE.Vector3, d: THREE.Vector3, n: THREE.Vector3) => {
    for (const v of [a, b, c, a, c, d]) {
      positions.push(v.x, v.y, v.z);
      normals.push(n.x, n.y, n.z);
    }
  };
  const corner = (p: { at: THREE.Vector2; out: THREE.Vector2 }, outwards: number, up: number) =>
    new THREE.Vector3(p.at.x + p.out.x * outwards, y + up, p.at.y + p.out.y * outwards);
  const [top, bottom] = [LIP_HEIGHT / 2, -LIP_HEIGHT / 2];
  for (let i = 0; i < ring.length; i++) {
    const p = ring[i]!;
    const q = ring[(i + 1) % ring.length]!;
    if (crosses(p.at.clone().add(q.at).multiplyScalar(0.5))) continue;
    const out = new THREE.Vector3(p.out.x + q.out.x, 0, p.out.y + q.out.y).normalize();
    quad(corner(p, LIP_OUT, bottom), corner(q, LIP_OUT, bottom), corner(q, LIP_OUT, top), corner(p, LIP_OUT, top), out);
    quad(corner(p, 0, top), corner(p, LIP_OUT, top), corner(q, LIP_OUT, top), corner(q, 0, top), new THREE.Vector3(0, 1, 0));
    quad(corner(p, 0, bottom), corner(q, 0, bottom), corner(q, LIP_OUT, bottom), corner(p, LIP_OUT, bottom), new THREE.Vector3(0, -1, 0));
    // Its end, where it stops short of a hole.
    const along = new THREE.Vector3(q.at.x - p.at.x, 0, q.at.y - p.at.y).normalize();
    const before = ring[(i + ring.length - 1) % ring.length]!;
    const after = ring[(i + 2) % ring.length]!;
    if (crosses(before.at.clone().add(p.at).multiplyScalar(0.5))) {
      quad(corner(p, 0, bottom), corner(p, LIP_OUT, bottom), corner(p, LIP_OUT, top), corner(p, 0, top), along.clone().negate());
    }
    if (crosses(q.at.clone().add(after.at).multiplyScalar(0.5))) {
      quad(corner(q, 0, bottom), corner(q, LIP_OUT, bottom), corner(q, LIP_OUT, top), corner(q, 0, top), along);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  g.setAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));
  return windOut(g);
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
  private readonly plastic = new THREE.MeshStandardMaterial({ ...PLASTIC, side: THREE.DoubleSide });
  private readonly rubber = new THREE.MeshStandardMaterial({ ...RUBBER, side: THREE.DoubleSide });
  private readonly cast = new THREE.MeshStandardMaterial({ ...CAST, side: THREE.DoubleSide });
  /**
   * The plenum's and the runners' metal, the exhaust pipes', which the pressure view colours. Both are drawn
   * hollow, as castings and tubes with their insides showing.
   */
  private readonly pipeMetal = new THREE.MeshStandardMaterial({
    vertexColors: true,
    metalness: 0.55,
    roughness: 0.38,
    side: THREE.DoubleSide,
  });
  private readonly clamp = new THREE.MeshStandardMaterial(CLAMP);
  private readonly plate = new THREE.MeshStandardMaterial({ ...CAST, color: 0xc9ced4, side: THREE.DoubleSide });
  private butterflies: THREE.Object3D[] = [];
  /** Dual plenums' balance valves' plates, and whether they are shown open. */
  private balanceValves: THREE.Object3D[] = [];
  private balanced = false;
  /** The plenum and the boss on its face, which take its colour. */
  private plenum: THREE.Mesh[] = [];
  private runners: THREE.Mesh[] = [];
  private layout: InletLayout | null = null;
  private showPressure = false;
  /** The specks: where each is along the tract, 0 at the throttle to 1 at the mouth, and across it. */
  private readonly specks = new THREE.Points(
    new THREE.BufferGeometry(),
    new THREE.PointsMaterial({ color: 0xa8e1ff, size: 0.007, transparent: true, opacity: 0.85, depthWrite: false }),
  );
  private readonly along = new Float32Array(SPECKS);
  private readonly offset = new Float32Array(SPECKS * 2);
  private velocity: Float32Array = new Float32Array(0);

  constructor() {
    this.group.add(this.tract);
    for (let i = 0; i < SPECKS; i++) {
      this.along[i] = Math.random();
      const r = Math.sqrt(Math.random()) * 0.75;
      const th = Math.random() * Math.PI * 2;
      this.offset[i * 2] = r * Math.cos(th);
      this.offset[i * 2 + 1] = r * Math.sin(th);
    }
    this.specks.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(SPECKS * 3), 3));
    this.specks.frustumCulled = false;
    this.specks.visible = false;
  }

  /** Whether the engine has the tract: one with a turbo draws through its compressors instead. */
  setTractVisible(on: boolean): void {
    this.tract.visible = on;
  }

  setPressureVisible(on: boolean): void {
    this.showPressure = on;
    this.specks.visible = on;
    this.setFinish();
    if (!on) this.paint(null);
  }

  /**
   * The plenum and the runners in the exhaust pipes' finish (`PipeMesh`): polished metal, or with the pressure
   * shown dull, so a pressure looks the same on them as on a pipe.
   */
  private setFinish(): void {
    this.pipeMetal.metalness = this.showPressure ? 0.15 : 0.55;
    this.pipeMetal.roughness = this.showPressure ? 0.55 : 0.38;
  }

  /** Show a snapshot's pressures, on `scale`, Pa, the exhaust's, and its air speeds, in the pressure view. */
  show(s: EngineSnapshot, scale: number): void {
    this.velocity = s.inletVelocity;
    this.setBalanced(!!s.plenumBalanced);
    if (!this.showPressure) return;
    this.paint(s, scale);
  }

  /** The engine at rest: every part at the atmosphere's pressure, and the air still. */
  settle(): void {
    this.velocity = new Float32Array(0);
    if (!this.showPressure) return;
    const ambient = new THREE.Color();
    pressureColor(0, ambient);
    for (const m of [...this.plenum, ...this.runners]) fill(m.geometry, ambient);
  }

  /**
   * Carry the specks `dt` seconds on, of the simulation's time, at the air's speed where each is. A speck
   * carried out of either end comes back in at the other.
   */
  flow(dt: number): void {
    const l = this.layout;
    if (!l || !this.specks.visible) return;
    const v = this.velocity;
    const total = l.segments.reduce((sum, x) => sum + x.length, 0);
    const pos = this.specks.geometry.getAttribute('position') as THREE.BufferAttribute;
    const arr = pos.array as Float32Array;
    const p = new THREE.Vector3();
    for (let i = 0; i < SPECKS; i++) {
      let s = this.along[i]!;
      if (v.length > 0 && dt > 0) {
        const u = v[Math.min(v.length - 1, Math.max(0, Math.floor(s * v.length)))]!;
        s += (u / total) * (dt / FLOW_SLOWDOWN);
        s -= Math.floor(s);
        this.along[i] = s;
      }
      this.speckAt(l, i % l.tubes.length, s, this.offset[i * 2]!, this.offset[i * 2 + 1]!, p);
      arr[i * 3] = p.x;
      arr[i * 3 + 1] = p.y;
      arr[i * 3 + 2] = p.z;
    }
    pos.needsUpdate = true;
  }

  /**
   * Where a speck `s` along the tract is, through tube `tube` of dual plenums' two, `a` and `b` across it as
   * fractions of its half-width and -height.
   */
  private speckAt(l: InletLayout, tube: number, s: number, a: number, b: number, out: THREE.Vector3): void {
    const segs = l.segments;
    const total = segs.reduce((sum, x) => sum + x.length, 0);
    const e1 = segs[0]!.length / total;
    const e2 = (segs[0]!.length + segs[1]!.length) / total;
    if (s >= e1 && s < e2) {
      const { centre, size } = l.airbox;
      const f = (s - e1) / (e2 - e1) - 0.5;
      out.set(centre.x + l.side * f * size.x, centre.y + b * size.y * 0.45, centre.z + a * size.z * 0.45);
      return;
    }
    const [curve, u, r] =
      s < e1
        ? [l.tubes[tube]!, s / e1, l.tubeRadius]
        : [l.snorkel, (s - e2) / (1 - e2), Math.sqrt(l.snorkelArea / Math.PI)];
    out.copy(curve.getPointAt(u));
    const t = curve.getTangentAt(u);
    const across = new THREE.Vector3(-t.z, 0, t.x);
    if (across.lengthSq() < 1e-8) across.set(1, 0, 0);
    across.normalize();
    const lift = new THREE.Vector3().crossVectors(across, t).normalize();
    out.addScaledVector(across, a * r).addScaledVector(lift, b * r);
  }

  /** Turn the butterflies to the throttle's opening, 0..1: nearly square to the bore shut, edge-on open. */
  setThrottle(opening: number): void {
    const open = 1 - Math.cos(Math.max(0, Math.min(1, opening)) * (Math.PI / 2));
    for (const b of this.butterflies) b.rotation.x = ((8 + 82 * open) * Math.PI) / 180;
  }

  /** Turn dual plenums' balance valves open, edge-on to the wall's holes, or shut across them. */
  private setBalanced(open: boolean): void {
    if (open === this.balanced) return;
    this.balanced = open;
    for (const v of this.balanceValves) v.rotation.y = open ? 0 : Math.PI / 2;
  }

  rebuild(spec: EngineSpec): void {
    this.clear();
    const l = inletLayout(spec);
    this.layout = l;
    this.buildEngineSide(l);
    this.buildTract(l);
    this.tract.add(this.specks);
    this.setThrottle(spec.throttle);
    this.paint(null);
    this.flow(0);
  }

  /** The plenum, its runners and the throttle body. */
  private buildEngineSide(l: InletLayout): void {
    const { centre, size } = l.plenum;
    // Each runner flares by `RUNNER_FLARE`, or as much as the face of the plenum it leaves by has room for
    // where it leaves it.

    const faceOf = (r: (typeof l.runners)[number]): Face => {
      const right = r.from.x >= centre.x;
      if (r.exit === 'bottom') return 'bottom';
      if (r.exit === 'flank') return right ? 'rightFlank' : 'leftFlank';
      return right ? 'right' : 'left';
    };
    const start = (r: (typeof l.runners)[number]) => new THREE.Vector2(r.from.x - centre.x, r.from.y - centre.y);
    const direction = (r: (typeof l.runners)[number]) => new THREE.Vector2(r.leaving.x, r.leaving.y).normalize();
    const flares = l.runners.map((r) => {
      const section = taperedSection(size, l.plenum.base, l.plenum.taper, r.from.z - centre.z);
      const [p, q] = section.faces[faceOf(r)]!.map((i) => section.points[i]!);
      const at = alongFace(p!, q!, start(r), direction(r));
      const room = Math.min(at, p!.distanceTo(q!) - at) - 0.002;
      return Math.max(0, Math.min(RUNNER_FLARE, room / r.radius - 1));
    });
    const openings: Opening[] = l.runners.map((r, k) => ({
      face: faceOf(r),
      start: start(r),
      direction: direction(r),
      z: r.from.z - centre.z,
      inside: runnerRadius(r.radius, 0, flares[k]!),
      outside: runnerRadius(r.radius, PLENUM_WALL, flares[k]!),
    }));
    const { bore } = l.throttles[0]!;
    const walled = l.balances.length > 0;
    const plenum = new THREE.Mesh(
      colourable(
        plenumGeometry(
          size,
          l.plenum.base,
          bore,
          l.throttles.map((t) => t.centre.x - centre.x),
          openings,
          l.plenum.taper,
          walled ? l.balances.map((b) => ({ z: b.centre.z - centre.z, y: b.centre.y - centre.y, radius: b.radius })) : null,
        ),
      ),
      this.pipeMetal,
    );
    plenum.position.copy(centre);
    this.group.add(plenum);
    this.plenum = [plenum];
    // The boss on its front face each throttle body bolts to, cast with it, standing a little proud.
    const front = centre.z - size.z / 2;
    // A ring round the bore, its inside the bore's own wall, so the throttle body opens into the plenum.
    const [inside, outside] = [bore / 2, bore / 2 + THROTTLE_WALL + 0.008];
    const ring = [
      new THREE.Vector2(inside, 0),
      new THREE.Vector2(outside, 0),
      new THREE.Vector2(outside, 0.03),
      new THREE.Vector2(inside, 0.03),
      new THREE.Vector2(inside, 0),
    ];
    for (const t of l.throttles) {
      const boss = new THREE.Mesh(colourable(new THREE.LatheGeometry(ring, 40)), this.pipeMetal);
      boss.rotation.x = -Math.PI / 2;
      boss.position.set(t.centre.x, t.centre.y, front + 0.01);
      this.group.add(boss);
      this.plenum.push(boss);
    }
    // Each balance valve: a plate on an upright shaft in its hole through the wall.
    for (const b of l.balances) {
      const valve = new THREE.Object3D();
      valve.position.copy(b.centre);
      valve.add(new THREE.Mesh(new THREE.CircleGeometry(b.radius - 0.0005, 32), this.plate));
      const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.003, 0.003, 2 * b.radius + 0.01, 8), this.clamp);
      valve.add(shaft);
      valve.rotation.y = this.balanced ? 0 : Math.PI / 2;
      this.group.add(valve);
      this.balanceValves.push(valve);
    }


    for (const [k, r] of l.runners.entries()) {
      const curve = new THREE.CubicBezierCurve3(
        r.from,
        r.from.clone().add(r.leaving),
        r.to.clone().sub(r.arriving),
        r.to,
      );
      // Flaring into the plenum where it leaves it.
      const length = curve.getLength();
      const tube = sweep(
        curve,
        (u) => {
          const radius = runnerRadius(r.radius, u * length, flares[k]!);
          return [radius, radius];
        },
        true,
      );
      tube.computeVertexNormals();
      const runner = new THREE.Mesh(colourable(tube), this.pipeMetal);
      this.group.add(runner);
      this.runners.push(runner);
    }

    // Each throttle body: a cast barrel with a flange each end, a shaft across, and the butterfly on it.
    for (const t of l.throttles) this.buildThrottle(t);
  }

  /** A throttle body, as `inletLayout` places it. */
  private buildThrottle(t: InletLayout['throttles'][number]): void {
    const { bore, length } = t;
    const body = new THREE.Group();
    body.position.copy(t.centre);
    const barrel = new THREE.Mesh(
      new THREE.CylinderGeometry(bore / 2 + THROTTLE_WALL, bore / 2 + THROTTLE_WALL, length, 32, 1, true),
      this.cast,
    );
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
    this.butterflies.push(butterfly);
    this.group.add(body);
  }

  /** The tube, the airbox and the snorkel. */
  private buildTract(l: InletLayout): void {
    // Each tube: a bellows coupler off the throttle body, then smooth, a clamp at each end.
    const r = l.tubeRadius + WALL;
    for (const curve of l.tubes) {
      const tubeLength = curve.getLength();
      const ribs = BELLOWS / tubeLength;
      const tube = new THREE.Mesh(
        sweep(curve, (u) => {
          const s = u * tubeLength;
          const rib = u < ribs && s > 0.02 ? 0.0045 * Math.max(0, Math.sin((s / RIB_PITCH) * Math.PI * 2)) : 0;
          return [r + rib, r + rib];
        }, false, Math.ceil(tubeLength / RIB_STATION)),
        this.rubber,
      );
      this.tract.add(tube);
      this.tract.add(band(curve, 0.012, r + 0.0015, this.clamp));
      this.tract.add(band(curve, Math.min(ribs + 0.02, 0.5), r + 0.0015, this.clamp));
      this.tract.add(band(curve, 0.985, r + 0.0015, this.clamp));
      // The air flow meter on the tube, ahead of the airbox: a small housing with its plug.
      const at = curve.getPointAt(0.7);
      const maf = new THREE.Mesh(new RoundedBoxGeometry(0.03, 0.022, 0.04, 2, 0.004), this.plastic);
      maf.position.copy(at).add(new THREE.Vector3(0, r + 0.008, 0));
      maf.lookAt(maf.position.clone().add(curve.getTangentAt(0.7)));
      this.tract.add(maf);
    }

    // The airbox: a rounded black plastic box, hollow, open where the tube and the snorkel join it so they can
    // be seen into, its lid's seam a lip round it, clipped down front and back. It stays black in the pressure
    // view: its cells would show only as their mean.
    const { centre, size } = l.airbox;
    const round = Math.sqrt(l.snorkelArea / Math.PI);
    // Where each pipe meets it, on the face it comes in square to.
    const holeAt = (at: THREE.Vector3, outwards: THREE.Vector3, radius: number): BoxHole => {
      const axis = ([0, 1, 2] as const).reduce((m, k) => (Math.abs(outwards.getComponent(k)) > Math.abs(outwards.getComponent(m)) ? k : m), 0);
      return { axis, sign: outwards.getComponent(axis) > 0 ? 1 : -1, centre: at.clone().sub(centre), radius };
    };
    const holes = [
      ...l.tubes.map((tube) => holeAt(tube.getPointAt(1), tube.getTangentAt(1).negate(), l.tubeRadius)),
      holeAt(l.snorkel.getPointAt(0), l.snorkel.getTangentAt(0), round),
    ];
    // Rounded as far as leaves each hole on the flat of its face.
    const clear = Math.min(
      ...holes.flatMap((h) =>
        ([0, 1, 2] as const)
          .filter((k) => k !== h.axis)
          .map((k) => size.getComponent(k) / 2 - Math.abs(h.centre.getComponent(k)) - h.radius - 0.002),
      ),
    );
    const rounding = Math.max(Math.min(Math.min(size.y, size.z) * 0.18, clear), AIRBOX_WALL + 0.001);
    const box = new THREE.Mesh(airboxGeometry(size, rounding, AIRBOX_WALL, holes), this.plastic);
    box.position.copy(centre);
    this.tract.add(box);
    // The lid's seam on the flat of the sides, above the openings, or below them where there is no room
    // above, so the lip does not run across them; the lip round it rounded at the corners as the box is.
    const sides = size.y / 2 - rounding - 0.006;
    const above = Math.max(size.y * 0.15, ...holes.map((h) => h.centre.y + h.radius + 0.012));
    const below = Math.min(...holes.map((h) => h.centre.y - h.radius - 0.012));
    const seamY = centre.y + (above <= sides ? above : below >= -sides ? below : Math.min(size.y * 0.15, sides));
    const lip = new THREE.Mesh(
      airboxLip(size, rounding, seamY - centre.y, holes.map((h) => ({ ...h, radius: h.radius + WALL + 0.004 }))),
      this.plastic,
    );
    lip.position.copy(centre);
    this.tract.add(lip);
    for (const fx of [-0.3, 0.3]) {
      for (const fz of [-1, 1]) {
        const clip = new THREE.Mesh(new RoundedBoxGeometry(0.018, 0.028, 0.008, 2, 0.002), this.clamp);
        clip.position.set(centre.x + fx * size.x, seamY, centre.z + (fz * (size.z + 0.012)) / 2);
        // None on a pipe's collar.
        const rel = clip.position.clone().sub(centre);
        if (holes.some((h) => rel.clone().sub(h.centre).setComponent(h.axis, 0).length() < h.radius + WALL + 0.004 + 0.016)) continue;
        this.tract.add(clip);
      }
    }
    // A spigot each end, where the tube and the snorkel join it: a collar round each, open through.
    for (const [curve, u, radius] of [
      ...l.tubes.map((tube) => [tube, 1, r + 0.004] as const),
      [l.snorkel, 0, round + SNORKEL_WALL + 0.004] as const,
    ]) {
      const spigot = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 0.03, 32, 1, true), this.plastic);
      spigot.position.copy(curve.getPointAt(u));
      spigot.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), curve.getTangentAt(u).normalize());
      this.tract.add(spigot);
    }

    // The snorkel: round where it leaves the airbox, flattening over its first stretch to fit under the
    // bonnet with the solver's area kept, and flaring at its mouth.
    const aspect = SNORKEL_ASPECT;
    const snorkelLength = l.snorkel.getLength();
    const snorkel = new THREE.Mesh(
      sweep(l.snorkel, (u) => {
        const fromMouth = (1 - u) * snorkelLength;
        const flare = fromMouth < 0.04 ? 1 + 0.35 * (1 - fromMouth / 0.04) ** 2 : 1;
        const flat = Math.min(1, (u * snorkelLength) / 0.08);
        const wide = round * (1 + (Math.sqrt(aspect) - 1) * flat);
        const high = (round * round) / wide;
        return [wide * flare + SNORKEL_WALL, high * flare + SNORKEL_WALL];
      }),
      this.plastic,
    );
    this.tract.add(snorkel);
    // A rolled lip round the mouth, the flare's edge.
    const [wide, high] = [round * Math.sqrt(aspect) * 1.35 + SNORKEL_WALL, (round / Math.sqrt(aspect)) * 1.35 + SNORKEL_WALL];
    const rim = new THREE.Mesh(new THREE.TorusGeometry(1, 0.0035 / Math.min(wide, high), 8, 48), this.plastic);
    rim.scale.set(wide, high, Math.min(wide, high));
    rim.position.copy(l.mouth);
    rim.lookAt(l.mouth.clone().add(l.snorkel.getTangentAt(1)));
    this.tract.add(rim);
  }

  /** Colour the plenum and the runners by a snapshot's gauge pressures on `scale`, Pa; with `null`, plain. */
  private paint(snap: EngineSnapshot | null, scale = 1): void {
    if (!this.layout) return;
    if (!snap || !this.showPressure) {
      for (const m of [...this.plenum, ...this.runners]) fill(m.geometry, PIPE_METAL);
      return;
    }
    const rgb = new THREE.Color();
    // The plenum by the zone each point is in along it, front to back: dual plenums' by the side of the
    // wall between them it is on.
    const zones = snap.plenumZones?.length ? snap.plenumZones : [snap.plenumPressure];
    const { centre, size, sides } = this.layout.plenum;
    const per = Math.max(Math.floor(zones.length / sides.length), 1);
    for (const m of this.plenum) {
      const pos = m.geometry.getAttribute('position') as THREE.BufferAttribute;
      const c = m.geometry.getAttribute('color') as THREE.BufferAttribute | undefined;
      if (!c) continue;
      const arr = c.array as Float32Array;
      const at = new THREE.Vector3();
      for (let i = 0; i < pos.count; i++) {
        at.fromBufferAttribute(pos, i).applyMatrix4(m.matrix);
        const along = (at.z - centre.z) / size.z + 0.5;
        const row = sides.length > 1 && Math.sign(at.x - centre.x) !== sides[0] ? 1 : 0;
        const k = Math.min(row * per + Math.min(Math.max(Math.floor(along * per), 0), per - 1), zones.length - 1);
        pressureColor(zones[k]! / scale, rgb);
        arr[i * 3] = rgb.r;
        arr[i * 3 + 1] = rgb.g;
        arr[i * 3 + 2] = rgb.b;
      }
      c.needsUpdate = true;
    }
    // A runner's cells from its valve; drawn from the plenum, so read back.
    let at = 0;
    snap.runnerCells.forEach((n, b) => {
      const runner = this.runners[b];
      const own = snap.runnerPressure.subarray(at, at + n);
      at += n;
      if (!runner || n === 0) return;
      const c = runner.geometry.getAttribute('color') as THREE.BufferAttribute;
      const arr = c.array as Float32Array;
      const rings = ALONG + 1;
      const per = AROUND + 1;
      for (let i = 0; i < rings; i++) {
        pressureColor(own[Math.min(n - 1, Math.floor((1 - i / (rings - 1)) * (n - 1)))]! / scale, rgb);
        for (let j = 0; j < per; j++) {
          const k = (i * per + j) * 3;
          arr[k] = rgb.r;
          arr[k + 1] = rgb.g;
          arr[k + 2] = rgb.b;
        }
      }
      c.needsUpdate = true;
    });
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
    this.butterflies = [];
    this.balanceValves = [];
    this.plenum = [];
    this.runners = [];
  }

  dispose(): void {
    this.clear();
    for (const m of [this.plastic, this.rubber, this.cast, this.pipeMetal, this.clamp, this.plate]) {
      m.dispose();
    }
    this.specks.geometry.dispose();
    (this.specks.material as THREE.Material).dispose();
  }
}
