/**
 * Where a compiled exhaust's turbos go on the engine (`compileExhaust`): one for each bank, halfway along
 * the engine, out from the bank's ports the way they point, its inlet facing back at them and each port
 * piped straight into it. Its shaft lies along the engine, the outlet rearwards and the compressor forwards,
 * and it is put as close in as leaves it clear of the engine.
 *
 * Geometry on the graph and the engine, no scene objects, so it can be tested as the app uses it.
 */

import * as THREE from 'three';

import { endsAt, junctionAt, type ExhaustDuct, type ExhaustGraph, type TurboMount } from '../model/exhaustGraph.js';
import { engineShell, engineShellDistance, type EngineShell, type Vec3 } from '../model/geometry.js';
import { makeSegment, segmentDiameter, type EngineSpec } from '../model/spec.js';
import {
  graphTurboSize,
  localInlet,
  quatRotate,
  turboBody,
  turboPorts,
  type TurboSize,
} from '../model/turbo.js';
import { fitCurve } from './drawing.js';
import type { ExhaustPort } from './exhaustLayout.js';

/** The least gap a turbo is left from the engine, m. */
const CLEARANCE = 0.008;
/** How finely a turbo is moved out until it is clear of the engine, m, and the furthest it goes. */
const STEP = 0.004;
const FURTHEST = 0.6;

/** How long a port's pipe runs straight out of it before it bends into its bank's turbo, m. */
const PORT_FLANGE = 0.03;
/** How far out from its ports a bank's turbo is at the least, in its ports' bores: room for them to bend in. */
const LEAST_REACH = 2.5;

/** How far behind the back of the engine, and below the lower of their outlets, two turbos' downpipes meet, m. */
const MERGE_BEHIND = 0.15;
const MERGE_DROP = 0.05;

/**
 * Put each turbo the compiled exhaust has and nobody has put anywhere yet where it goes on the engine (see
 * the module's note), and where two turbos' downpipes meet, fix the junction behind the engine
 * (`seatDownpipes`). Done once: after this every turbo has a place.
 */
export function seatEngineTurbos(graph: ExhaustGraph, ports: ExhaustPort[], spec: EngineSpec): void {
  const size = graphTurboSize(graph, spec);
  const shell = engineShell(spec);
  const samples = surfaceSamples(size);
  for (const mount of graph.turbos ?? []) {
    if (mount.position) continue;
    const feeds = endsAt(graph, mount.node)
      .filter((e) => e.end === 'outlet')
      .map((e) => e.duct);
    if (feeds.length > 0 && feeds.every((d) => d.from.kind === 'valve' && d.role === 'runner')) {
      bankTurbo(mount, feeds, ports, size, shell, samples);
    }
  }
  seatDownpipes(graph, spec, shell);
}

/**
 * Halfway along the engine, out from the middle of its bank's ports the way they point, under the bank of a
 * V or a boxer, whose ports point down, its inlet facing back at them, as near as leaves room for their pipes to bend in and it clear of
 * the engine. Each port's pipe runs a flange's length straight out, then bends into the inlet
 * (`refitBends`).
 */
function bankTurbo(
  mount: TurboMount,
  feeds: ExhaustDuct[],
  ports: ExhaustPort[],
  size: TurboSize,
  shell: EngineShell,
  samples: Vec3[],
): void {
  const at = feeds.map((d) => ports[(d.from as { cylinder: number }).cylinder]).filter((p) => !!p);
  if (at.length === 0) return;
  const centre = new THREE.Vector3();
  const out = new THREE.Vector3();
  for (const p of at) {
    centre.add(p.position);
    out.add(p.direction);
  }
  centre.multiplyScalar(1 / at.length).setZ(0);
  out.setZ(0).normalize();
  const bore = Math.max(...feeds.map((d) => (d.segments[0] ? segmentDiameter(d.segments[0], 0) : 0.04)));
  mount.rotation = alongEngine([out.x, out.y, out.z]);
  const inletAt = (reach: number): Vec3 => {
    const p = centre.clone().addScaledVector(out, reach);
    return [p.x, p.y, p.z];
  };
  seatClear(mount, size, shell, samples, inletAt, LEAST_REACH * bore);
  for (const d of feeds) {
    const first = d.segments[0];
    const dia = first ? segmentDiameter(first, 0) : bore;
    d.segments = [makeSegment({ kind: 'pipe', length: PORT_FLANGE, dIn: dia, dOut: dia })];
    d.headingYaw = 0;
    d.headingPitch = 0;
    delete d.fitted;
    delete d.swing;
    delete d.square;
    delete d.arriveRoll;
  }
}

/**
 * A turn that lays a turbo along the engine: its shaft along the crank, the compressor forwards and the
 * outlet rearwards (the world's +z), and its inlet taking gas arriving along `arriving`, which is square to
 * the crank.
 */
function alongEngine(arriving: Vec3): TurboMount['rotation'] {
  const x = new THREE.Vector3(0, 0, -1);
  const z = new THREE.Vector3(...arriving).normalize();
  const y = z.clone().cross(x);
  const q = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, z));
  return [q.x, q.y, q.z, q.w];
}

/**
 * Put `mount`, turned as it is, with its inlet at `inletAt(reach)` for the least `reach` from `least` that
 * leaves it `CLEARANCE` from the engine.
 */
function seatClear(
  mount: TurboMount,
  size: TurboSize,
  shell: EngineShell,
  samples: Vec3[],
  inletAt: (reach: number) => Vec3,
  least: number,
): void {
  const offset = quatRotate(localInlet(size), mount.rotation);
  const world = samples.map((p) => quatRotate(p, mount.rotation));
  const place = (reach: number): Vec3 => {
    const at = inletAt(reach);
    return [at[0] - offset[0], at[1] - offset[1], at[2] - offset[2]];
  };
  let reach = least;
  for (; reach < FURTHEST; reach += STEP) {
    const p = place(reach);
    if (world.every((w) => engineShellDistance(shell, [p[0] + w[0], p[1] + w[1], p[2] + w[2]]) >= CLEARANCE)) break;
  }
  mount.position = place(Math.min(reach, FURTHEST));
}

/** Points over the surface of the room a turbo takes up (`turboBody`), in its own frame: rings round each part's ends and middle. */
function surfaceSamples(size: TurboSize): Vec3[] {
  const out: Vec3[] = [];
  const RING = 12;
  for (const part of turboBody(size)) {
    const [i, j, k] = part.axis === 0 ? [0, 1, 2] : part.axis === 1 ? [1, 2, 0] : [2, 0, 1];
    for (const along of [-part.half, 0, part.half]) {
      for (let n = 0; n <= RING; n++) {
        // The ring round the part, and its axis at each end.
        const r = n === RING ? 0 : part.radius;
        const t = (2 * Math.PI * n) / RING;
        const p: Vec3 = [...part.centre];
        p[i] += along;
        p[j] += r * Math.cos(t);
        p[k] += r * Math.sin(t);
        out.push(p);
      }
    }
  }
  return out;
}

/**
 * Where two turbos each have a downpipe to one junction and a collector out of it: the junction fixed on the
 * engine's centreline behind it, below the lower of their outlets, the collector leaving it rearwards, and
 * each downpipe one bend from its turbo's outlet into it.
 *
 * Only for downpipes nothing has touched yet. Once done, the junction is fixed, so it is done once.
 */
function seatDownpipes(graph: ExhaustGraph, spec: EngineSpec, shell: EngineShell): void {
  const size = graphTurboSize(graph, spec);
  const turbos = (graph.turbos ?? []).filter((t) => t.position);
  const downpipes = turbos.map((t) => graph.ducts.find((d) => d.role === 'downpipe' && d.from.kind === 'node' && d.from.node === t.node));
  const node = downpipes[0]?.to.kind === 'node' ? downpipes[0].to.node : null;
  if (turbos.length < 2 || !node || junctionAt(graph, node)) return;
  if (downpipes.some((d) => !d || d.fitted || d.to.kind !== 'node' || d.to.node !== node)) return;
  const collector = graph.ducts.find((d) => d.role === 'collector' && d.from.kind === 'node' && d.from.node === node);
  if (!collector) return;
  const outlets = turbos.map((t) => turboPorts(t as TurboMount & { position: Vec3 }, size).outlet);
  const x = outlets.reduce((a, o) => a + o.point[0], 0) / outlets.length;
  const y = Math.min(...outlets.map((o) => o.point[1])) - MERGE_DROP;
  const z = Math.max(shell.crankcase.length / 2, ...outlets.map((o) => o.point[2])) + MERGE_BEHIND;
  const merge = new THREE.Vector3(x, y, z);
  const axis = new THREE.Vector3(0, 0, 1);
  (graph.junctions ??= []).push({ node, position: [x, y, z], axis: [0, 0, 1] });
  collector.headingYaw = 0;
  collector.headingPitch = 0;
  downpipes.forEach((d, i) => {
    const o = outlets[i]!;
    const bore = segmentDiameter(d!.segments[0]!, 0);
    d!.segments = [fitCurve(new THREE.Vector3(...o.point), new THREE.Vector3(...o.dir), merge, axis, { dIn: bore, dOut: bore })];
    d!.fitted = true;
  });
}
