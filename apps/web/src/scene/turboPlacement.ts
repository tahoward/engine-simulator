/**
 * Where turbos go in the scene: seating the ones a compiled layout asked for, and moving one with the
 * pipes that feed it.
 *
 * Geometry on the graph and the layout, no scene objects, so it can be tested as the app uses it.
 */

import * as THREE from 'three';

import { drawnSegments, endsAt, junctionAt, type ExhaustDuct, type ExhaustGraph, type Quat } from '../model/exhaustGraph.js';
import type { Vec3 } from '../model/geometry.js';
import { makeSegment, segmentDiameter, type EngineSpec, type PipeSegment } from '../model/spec.js';
import { graphTurboSize, seatTurbo, turboPortsOf } from '../model/turbo.js';
import { MIN_BEND_BORES, bendAnchor, fitCurve, type BendAnchor } from './drawing.js';
import { bendRadius } from './PipeMesh.js';
import { layoutGraph, type ExhaustPort } from './exhaustLayout.js';
import { layoutPipe } from './PipeMesh.js';

/** Where a turbo is put down unless it is snapped to a pipe: level with the exhaust ports. */
export function turboHeight(ports: ExhaustPort[]): number {
  return ports.length > 0 ? ports.reduce((a, p) => a + p.position.y, 0) / ports.length : 0;
}

/**
 * Put every turbo a compiled layout asked for where that layout's junction is: its inlet flange where the
 * pipes meet, turned to take them.
 */
export function seatTurbos(graph: ExhaustGraph, ports: ExhaustPort[], spec: EngineSpec): void {
  const unseated = (graph.turbos ?? []).filter((t) => !t.position);
  if (unseated.length === 0) return;
  const placement = layoutGraph(ports, graph, turboPortsOf(graph, spec));
  const size = graphTurboSize(graph, spec);
  for (const mount of unseated) {
    const joint = placement.joints.get(mount.node);
    const out = graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === mount.node);
    const leaving = out ? placement.ducts.get(out.id) : undefined;
    // With nothing drawn from its outlet, it goes on the end of the one pipe into it.
    const feed = graph.ducts.find((d) => d.to.kind === 'node' && d.to.node === mount.node);
    const fed = feed ? placement.ducts.get(feed.id) : undefined;
    const end = feed && fed && feed.segments.length > 0 ? layoutPipe(feed.segments, fed.origin, fed.heading) : undefined;
    const point = joint?.centre ?? leaving?.origin ?? end?.joints.at(-1);
    const dir = joint?.axis ?? leaving?.heading ?? end?.jointDirections.at(-1);
    if (!point || !dir) {
      mount.position = [0, turboHeight(ports), 0];
      continue;
    }
    seatTurbo(mount, [point.x, point.y, point.z], [dir.x, dir.y, dir.z], size);
  }
}

/**
 * Move a junction to `position`: it has a place of its own from then on, and the pipes into it bend in to
 * meet it there. `axis` is the way it points, which pipes leaving it are turned off, taken where it stood the
 * first time it is moved.
 */
export function moveJunction(
  graph: ExhaustGraph,
  ports: ExhaustPort[],
  spec: EngineSpec,
  node: string,
  position: Vec3,
  axis: Vec3,
): void {
  const pinned = junctionAt(graph, node);
  if (pinned) pinned.position = position;
  else (graph.junctions ??= []).push({ node, position, axis });
  refitBends(graph, ports, spec);
}

/** Move or turn a turbo. The pipes feeding it follow, their bends into its inlet fitted again. */
export function moveTurbo(
  graph: ExhaustGraph,
  ports: ExhaustPort[],
  spec: EngineSpec,
  id: string,
  position: Vec3,
  rotation: Quat,
): void {
  const mount = graph.turbos?.find((t) => t.id === id);
  if (!mount) return;
  mount.position = position;
  mount.rotation = rotation;
  refitBends(graph, ports, spec);
}

/**
 * Fit the bend every joined pipe finishes in: from where the pipe as drawn ends, one smooth bend to what it
 * joins (`bendAnchor`, `fitCurve`). Every pipe into a turbo has one, and so does every pipe drawn to join
 * another.
 *
 * Done on every rebuild, so each bend follows whatever moved: the turbo or the pipe it joins, or the pipe
 * drawn up to it.
 */
export function refitBends(graph: ExhaustGraph, ports: ExhaustPort[], spec: EngineSpec): void {
  const turbos = turboPortsOf(graph, spec);
  if (turbos.size === 0 && !graph.junctions?.length && !graph.ducts.some((d) => d.fitted)) return;
  const placement = layoutGraph(ports, graph, turbos);
  for (const duct of graph.ducts) {
    // A pipe leaving a turbo starts at its outlet's bore.
    const outlet = duct.from.kind === 'node' ? turbos.get(duct.from.node)?.outlet : undefined;
    if (outlet && duct.segments[0]) duct.segments[0].dIn = outlet.dia;
    if (duct.to.kind !== 'node') continue;
    // Every pipe into a turbo, or into a junction that has been moved, bends in to meet it.
    if (!duct.fitted && !turbos.has(duct.to.node) && !junctionAt(graph, duct.to.node)) continue;
    const anchor = bendAnchor(graph, placement, duct.to.node, duct.id);
    const place = placement.ducts.get(duct.id);
    if (anchor && place) fitBend(duct, place.origin, place.heading, anchor);
  }
}

/**
 * Fit `duct`'s end to `anchor`: its drawn part, then a bend arriving there along the anchor's direction.
 * The bend tapers from the bore the drawn part ends at to the anchor's, so it matches at both ends.
 */
export function fitBend(duct: ExhaustDuct, origin: THREE.Vector3, heading: THREE.Vector3, anchor: BendAnchor): void {
  const drawn = duct.fitted ? duct.segments.slice(0, -1) : duct.segments;
  const swept = layoutPipe(drawn, origin, heading);
  const entry = drawn.length > 0 ? swept.joints.at(-1)! : origin;
  const entryDir = drawn.length > 0 ? swept.jointDirections.at(-1)! : heading;
  const target = anchor.point;
  // Where the drawn pipe already meets it there is nothing to fit: a turbo put down on its end.
  if (entry.distanceTo(target) < 1e-4 && drawn.length > 0) {
    duct.segments = drawn;
    delete duct.fitted;
    delete duct.swing;
    return;
  }
  const last = drawn.at(-1);
  const start = last ? segmentDiameter(last, 1) : (duct.segments[0]?.dIn ?? anchor.dia);
  const bend = fitCurve(entry, entryDir, target, anchor.dir, { dIn: start, dOut: anchor.dia });
  // The same bend as before keeps its id, so the panel's row for it stays put.
  const old = duct.fitted ? duct.segments.at(-1) : undefined;
  if (old) bend.id = old.id;
  duct.segments = [...drawn, bend];
  duct.fitted = true;
}

/** The shortest straight a header's primary leaves its port with before it bends, m. */
const HEADER_FLANGE = 0.03;

/** How near its length a pipe fitted to one has to come, m. */
const LENGTH_TOLERANCE = 5e-4;

/**
 * Give every compiled header its bends: each primary straight out of its port, square to the head, then
 * one smooth bend into its collector, which is fixed where the primaries meet, pointing the way the ports
 * do. Each primary stays the length it was compiled at, so the headers are still equal-length and the
 * exhaust sounds as it was tuned: the collector sits where those with furthest to come reach it at their
 * length, and those nearer take a swing on the way to make theirs up, as real equal-length headers do
 * (`fitToLength`).
 *
 * Only for a merge nothing has touched yet: every pipe into it a compiled runner from a port, and one
 * collector out of it, not yet fixed in place. Once done, the merge is fixed, so it is done once.
 */
export function seatHeaders(graph: ExhaustGraph, ports: ExhaustPort[], _spec: EngineSpec): void {
  const merges = new Set<string>();
  for (const d of graph.ducts) {
    if (d.role === 'collector' && d.from.kind === 'node' && !junctionAt(graph, d.from.node)) merges.add(d.from.node);
  }
  for (const node of merges) {
    const ends = endsAt(graph, node);
    const feeds = ends.filter((e) => e.end === 'outlet').map((e) => e.duct);
    const outs = ends.filter((e) => e.end === 'inlet').map((e) => e.duct);
    const header = (d: ExhaustDuct) => d.role === 'runner' && d.from.kind === 'valve' && !d.fitted && d.segments.length === 1;
    if (feeds.length < 2 || outs.length !== 1 || !feeds.every(header)) continue;
    const at = feeds.map((d) => ports[(d.from as { cylinder: number }).cylinder]);
    if (at.some((p) => !p)) continue;
    const primaries = feeds.map((duct, i) => ({
      duct,
      port: at[i]!,
      length: duct.segments[0]!.length,
      bore: segmentDiameter(duct.segments[0]!, 1),
    }));
    const centre = new THREE.Vector3();
    const axis = new THREE.Vector3();
    for (const p of at) {
      centre.add(p!.position);
      axis.add(p!.direction);
    }
    centre.multiplyScalar(1 / at.length);
    axis.normalize();

    const merge = headerMerge(primaries, centre, axis);
    (graph.junctions ??= []).push({ node, position: [merge.x, merge.y, merge.z], axis: [axis.x, axis.y, axis.z], collector: true });
    for (const p of primaries) {
      const first = p.duct.segments[0]!;
      p.duct.segments = [makeSegment({ ...first, id: first.id, length: HEADER_FLANGE })];
      fitToLength(p.duct, p.port.position, p.port.direction, { point: merge, dir: axis, dia: p.bore }, p.length);
    }
  }
}

interface Primary {
  port: ExhaustPort;
  /** The length it was tuned to, m. */
  length: number;
}

/** How long a primary is with a straight of `straight` and one bend into `merge`. */
function primaryLength(p: Primary, straight: number, merge: THREE.Vector3, axis: THREE.Vector3): number {
  const entry = p.port.position.clone().addScaledVector(p.port.direction, straight);
  return straight + fitCurve(entry, p.port.direction, merge, axis).length;
}

/**
 * Where a header's primaries meet: straight out from the middle of their ports along the way they point,
 * as far out as it can be with the primary that has furthest to come no longer than it should be. Any
 * further, and every primary would have to be longer than it is.
 */
function headerMerge(primaries: Primary[], centre: THREE.Vector3, axis: THREE.Vector3): THREE.Vector3 {
  const at = (out: number) => centre.clone().addScaledVector(axis, out);
  const tooLong = (out: number) =>
    primaries.some((p) => primaryLength(p, HEADER_FLANGE, at(out), axis) > p.length);
  let lo = 0.05;
  let hi = Math.min(...primaries.map((p) => p.length));
  for (let i = 0; i < 30; i++) {
    const mid = (lo + hi) / 2;
    if (tooLong(mid)) hi = mid;
    else lo = mid;
  }
  return at(lo);
}

/** How a pipe came out when fitted to a length. */
export interface LengthFit {
  /** Whether it came out the length asked for. */
  reached: boolean;
  /** How long it is now, m. */
  length: number;
}

/**
 * Fit `duct`, joined at its far end to `anchor`, to be `length` long, keeping it smooth: no bend tighter
 * than `MIN_BEND_BORES`.
 *
 * Its drawn part is kept, but for its last straight, which is lengthened or shortened so the bend into
 * `anchor` takes up the rest. Where even the longest straight a smooth bend allows leaves it short, the
 * pipe takes a swing on its way instead: out square to the line to the anchor, downwards, and back, far
 * enough to make its length up, as the inner primaries of equal-length headers do. The swing is fitted and
 * locked with the bend (`ExhaustDuct.swing`). A pipe that cannot be made that short is left as short as it
 * goes. A pipe with nothing drawn starts at `startBore`, or the bore its bend starts at now.
 */
export function fitToLength(
  duct: ExhaustDuct,
  origin: THREE.Vector3,
  heading: THREE.Vector3,
  anchor: BendAnchor,
  length: number,
  startBore?: number,
): LengthFit {
  const drawn = drawnSegments(duct);
  const last = drawn.at(-1);
  const adjustable = !!last && !last.curve && last.kind !== 'chamber';
  const before = adjustable ? drawn.slice(0, -1) : drawn;
  const already = before.reduce((a, seg) => a + seg.length, 0);
  const lead = layoutPipe(before, origin, heading);
  const start = before.length > 0 ? lead.joints.at(-1)! : origin;
  const startDir = before.length > 0 ? lead.jointDirections.at(-1)! : heading;
  const endBore = last ? segmentDiameter(last, 1) : (startBore ?? duct.segments[0]?.dIn ?? anchor.dia);
  const tightest = MIN_BEND_BORES * endBore;
  const bore = { dIn: endBore, dOut: anchor.dia };

  // Where the pipe is heading after a last straight of `s`, if it has one to change.
  const tipAfter = (s: number) => {
    if (!adjustable) return { entry: start, dir: startDir };
    const swept = layoutPipe([makeSegment({ ...last!, length: s })], start, startDir);
    return { entry: swept.joints.at(-1)!, dir: swept.jointDirections.at(-1)! };
  };
  const direct = (s: number) => {
    const { entry, dir } = tipAfter(s);
    const bend = fitCurve(entry, dir, anchor.point, anchor.dir, bore);
    const radius = bend.curve ? bendRadius(entry, dir, anchor.point, anchor.dir) : Infinity;
    return { bend, length: already + (adjustable ? s : 0) + bend.length, radius };
  };
  const apply = (segments: PipeSegment[], swing: boolean): LengthFit => {
    const old = duct.segments.at(-1);
    const bend = segments.at(-1)!;
    if (duct.fitted && old) bend.id = old.id;
    duct.segments = segments;
    duct.fitted = true;
    if (swing) duct.swing = true;
    else delete duct.swing;
    const total = segments.reduce((a, seg) => a + seg.length, 0);
    return { reached: Math.abs(total - length) < LENGTH_TOLERANCE * 2, length: total };
  };
  const straight = (s: number) => (adjustable ? [makeSegment({ ...last!, id: last!.id, length: s })] : []);

  let s = adjustable ? last!.length : 0;
  if (adjustable) {
    // The longest straight a smooth bend allows: the longer the straight, the less room the bend has, and
    // the tighter it turns. No longer than takes it level with the anchor, past which it would double back.
    const along = tipAfter(1).entry.clone().sub(start).normalize();
    const reach = Math.max(anchor.point.clone().sub(start).dot(along), HEADER_FLANGE);
    let longest = HEADER_FLANGE;
    if (direct(HEADER_FLANGE).radius >= tightest) {
      if (direct(reach).radius >= tightest) longest = reach;
      else {
        let lo = HEADER_FLANGE;
        let hi = reach;
        for (let i = 0; i < 30; i++) {
          const mid = (lo + hi) / 2;
          if (direct(mid).radius >= tightest) lo = mid;
          else hi = mid;
        }
        longest = lo;
      }
    }
    // Never less than the straight already is: a bend as tight as the one drawn is one the user accepted.
    longest = Math.max(longest, s);
    if (direct(HEADER_FLANGE).length >= length - LENGTH_TOLERANCE) {
      // As short as it goes: the straight at its least.
      return apply([...before, ...straight(HEADER_FLANGE), direct(HEADER_FLANGE).bend], false);
    }
    if (direct(longest).length >= length - LENGTH_TOLERANCE) {
      // The longer the straight, the longer the two together.
      let a = HEADER_FLANGE;
      let b = longest;
      for (let i = 0; i < 40; i++) {
        const mid = (a + b) / 2;
        if (direct(mid).length < length) a = mid;
        else b = mid;
      }
      const at = (a + b) / 2;
      return apply([...before, ...straight(at), direct(at).bend], false);
    }
    // Not long enough even so: a swing from where the pipe was drawn to, or from as far as a smooth bend allows.
    s = Math.min(s, longest);
  } else if (direct(0).length >= length - LENGTH_TOLERANCE) {
    return apply([...before, direct(0).bend], false);
  }

  const { entry, dir } = tipAfter(s);
  const chord = anchor.point.clone().sub(entry);
  const toward = chord.clone().normalize();
  let away = toward.clone().cross(new THREE.Vector3(0, 0, 1));
  if (away.lengthSq() < 1e-8) away = new THREE.Vector3(0, -1, 0);
  away.normalize();
  if (away.y > 0) away.negate();
  const swingAt = (height: number) => {
    const point = entry.clone().addScaledVector(chord, 0.5).addScaledVector(away, height);
    const there = fitCurve(entry, dir, point, toward, { dIn: endBore, dOut: endBore });
    const back = fitCurve(point, toward, anchor.point, anchor.dir, bore);
    return { there, back, length: already + (adjustable ? s : 0) + there.length + back.length };
  };
  // The further out it swings, the longer it is; far enough out, as long as anything asked for.
  let lo = 0;
  let hi = Math.max(0.5 * chord.length(), length);
  for (let i = 0; i < 40; i++) {
    const mid = (lo + hi) / 2;
    if (swingAt(mid).length < length) lo = mid;
    else hi = mid;
  }
  const swing = swingAt((lo + hi) / 2);
  return apply([...before, ...straight(s), swing.there, swing.back], true);
}

/**
 * Make pipe `ductId`, joined at its far end, `length` long (`fitToLength`). `null` where it is not joined
 * to anything it bends in to meet.
 */
export function matchLength(
  graph: ExhaustGraph,
  ports: ExhaustPort[],
  spec: EngineSpec,
  ductId: string,
  length: number,
): LengthFit | null {
  const duct = graph.ducts.find((d) => d.id === ductId);
  if (!duct || duct.to.kind !== 'node') return null;
  const placement = layoutGraph(ports, graph, turboPortsOf(graph, spec));
  const anchor = bendAnchor(graph, placement, duct.to.node, ductId);
  const place = placement.ducts.get(ductId);
  if (!anchor || !place) return null;
  return fitToLength(duct, place.origin, place.heading, anchor, length);
}
