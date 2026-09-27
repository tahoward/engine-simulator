/**
 * The exhaust as a graph of ducts joined at nodes.
 *
 * This is the thing the solver is built from, rather than one primary geometry shared by every cylinder
 * plus one collector per bank. That shape suits a topology picked from a dropdown, but it cannot express
 * what drawing pipes implies: runners of different lengths, a tri-Y, or a branch part way along another
 * duct.
 *
 * The model is deliberately small. A duct is a list of `PipeSegment`s with something at each end —
 * a cylinder's exhaust valve, a node, or open air — and a node is nothing but an id that several duct
 * ends happen to share. That is enough for every layout the app has today and for the ones it does
 * not, with no special cases: a 4-into-1 is four ducts and a collector sharing one node, a tri-Y is
 * two pairs sharing two nodes that feed a third, and a T-branch is a duct that was split in two so
 * its middle became a node.
 *
 * Nothing here knows about three.js or about the solver. `compileLayout` turns the existing
 * `exhaustLayout` spec into a graph, so presets, saved URLs and the layout dropdown all keep working
 * and the solver only ever sees a graph.
 */

import { type Vec3, distance, exhaustPortOf, sweepEnd, turnBetweenDirs } from './geometry.js';
import {
  type EngineSpec,
  type PipeSegment,
  collectorGroups,
  crankPins,
  cylinderSpacing,
  exhaustLayoutOf,
  makeSegment,
  physicalBank,
  segmentDiameter,
} from './spec.js';

/**
 * What feeds a duct's inlet: a cylinder's exhaust valve, a node, or nothing yet, for a loose pipe put down
 * at `position` (m, world) and not attached to anything. A loose pipe carries no gas, so the solver is not
 * given it (`solverGraph`); once a pipe is drawn into its start, that start is a node like any other.
 */
export type DuctSource =
  | { kind: 'valve'; cylinder: number }
  | { kind: 'node'; node: string }
  | { kind: 'free'; position: Vec3 };

/** Where a duct's outlet goes. */
export type DuctSink = { kind: 'node'; node: string } | { kind: 'mouth' };

export interface ExhaustDuct {
  id: string;
  segments: PipeSegment[];
  from: DuctSource;
  to: DuctSink;
  /**
   * Initial heading, as yaw and pitch *relative to* whatever the duct starts from — the port frame
   * for a valve, the upstream end direction for a node.
   *
   * Relative, never world coordinates. The ports move when the bore or the vee angle changes, so a
   * stored world heading would quietly detach the moment the engine was resized. Absent means "work
   * it out", which is what compiled graphs do: `layoutGraph` derives a heading that aims the runner
   * at its collector.
   */
  headingYaw?: number;
  headingPitch?: number;
  /**
   * What the heading is relative to when it is not the default: `'world'` for world +X.
   *
   * For a pipe leaving a junction whose direction was frozen where the layout had put it. A junction's own
   * axis is worked out afresh from the pipes arriving at it each time, and editing one of them could turn
   * it — and with it, everything downstream stored relative to it.
   */
  headingFrame?: 'world';
  /**
   * The duct this one carries straight on from, when it was made by cutting or extending that duct.
   *
   * Set on the downstream half of a split, on the pipe added after a joined end, and by `compileLayout` on
   * each manifold length after the first and on a single bank's collector. It is what says the
   * junction was made by attaching to a pipe that was already there, so that pipe keeps going the way it
   * was: without it the junction would take its direction from the average of its feeds' *starting*
   * directions — right for a collector's runners, which all head one way, and a 90-degree swing for a pipe
   * attached from the other side — and the collar search would re-aim the pipe being attached to.
   */
  continues?: string;
  /**
   * What a compiled duct is for, where it was compiled rather than drawn.
   *
   * A manifold's stubs and lengths are generated from the engine, not from the runner and collector
   * geometry a preset gives, so anything that reads that geometry back out of a graph — carrying it
   * across a change of cylinder count, say — has to know which ducts actually hold it. Reading "the
   * first runner" and "the first pipe after a junction" off a manifold would find a stub and a manifold
   * length.
   */
  role?: 'runner' | 'stub' | 'manifold' | 'downpipe' | 'collector';
  /**
   * Whether the duct's last segment is a bend fitted to what the duct joins, rather than drawn.
   *
   * A pipe drawn to join something, a turbo's inlet or another pipe, finishes in one smooth bend worked out
   * from where the drawn pipe ends and the place and direction of what it joins (`bendAnchor`), and fitted
   * again whenever either moves. Joined at both ends, it is not edited: the drawn pipe up to it is.
   */
  fitted?: true;
  /**
   * Whether the segment before its fitted bend is a swing fitted with it: the bend out and back a pipe takes
   * to come out a given length, as a header's inner primaries do. Fitted, not drawn, and not edited; it
   * goes with the bend when the pipe comes off what it joins.
   */
  swing?: true;
}

/** How many of `duct`'s last segments were fitted rather than drawn: its bend, and a swing before it. */
export function fittedCount(duct: ExhaustDuct): number {
  if (!duct.fitted || duct.segments.length === 0) return 0;
  return duct.swing && duct.segments.length >= 2 ? 2 : 1;
}

/** `duct`'s segments as drawn: without the bend, and any swing, fitted at its end. */
export function drawnSegments(duct: ExhaustDuct): PipeSegment[] {
  return duct.segments.slice(0, duct.segments.length - fittedCount(duct));
}

/**
 * A turbocharger placed in the exhaust: its turbine sits at `node`, so the ducts ending there feed its
 * inlet and the one leaving it is its outlet. See `turbo.ts`.
 *
 * Unlike a junction, which is wherever the pipes meeting at it end, a turbo has a place of its own: where
 * it was put down, `position` (m, world), turned by `rotation`, a unit quaternion `[x, y, z, w]` from its
 * own frame (see `turbo.ts`) into the world's. A `position` of `null` is a turbo a compiled
 * layout asked for and nobody has placed yet, which the app seats where that layout's junction is.
 */
export interface TurboMount {
  id: string;
  node: string;
  position: Vec3 | null;
  rotation: Quat;
}

/** A rotation as a unit quaternion, `[x, y, z, w]`. */
export type Quat = [number, number, number, number];

/**
 * A junction given a place of its own by being moved there: `position` (m, world), with pipes leaving it
 * turned off `axis`. A junction is otherwise wherever the pipes meeting at it end; once placed, they bend in
 * to meet it (`ExhaustDuct.fitted`).
 */
export interface JunctionMount {
  node: string;
  position: Vec3;
  axis: Vec3;
  /**
   * A header's collector, where several primaries merge: each bends in at its own bore, as their gas
   * arrives, rather than tapering to the pipe carrying it on.
   */
  collector?: true;
}

export interface ExhaustGraph {
  ducts: ExhaustDuct[];
  /** Turbochargers, each at one node. One whose node no duct names is not connected yet. */
  turbos?: TurboMount[];
  /** Junctions that have been moved, and so have a place of their own. */
  junctions?: JunctionMount[];
}

/** Where the junction at `node` has been moved to, if it has. */
export function junctionAt(graph: ExhaustGraph, node: string): JunctionMount | undefined {
  return graph.junctions?.find((j) => j.node === node);
}

/** The turbo whose turbine sits at `node`, if one does. */
export function turboAt(graph: ExhaustGraph, node: string): TurboMount | undefined {
  return graph.turbos?.find((t) => t.node === node);
}

const ROLES = new Set(['runner', 'stub', 'manifold', 'downpipe', 'collector']);

/**
 * A graph rebuilt from parsed JSON, such as a shared link, or `null` if it has no ducts.
 *
 * Every field the layout reads is carried across: the segments, through `makeSegment`, so a hand-edited or
 * truncated link cannot put `undefined` diameters into the solver; the heading and the frame it is measured
 * in; and `continues` and `role`, which decide what each pipe is aimed at. Dropping any of them lays the
 * exhaust out differently when the page is reloaded — a frozen heading read in the wrong frame turns the
 * pipe, and a manifold length that no longer knows what it continues gets re-aimed.
 */
export function graphFromJson(raw: unknown): ExhaustGraph | null {
  const ducts = (raw as { ducts?: unknown } | null)?.ducts;
  if (!Array.isArray(ducts) || ducts.length === 0) return null;
  const finite = (v: unknown): v is number => typeof v === 'number' && Number.isFinite(v);
  const turbos = (raw as { turbos?: unknown }).turbos;
  const mounts: TurboMount[] = Array.isArray(turbos)
    ? turbos.flatMap((t: Record<string, unknown>) => {
        if (typeof t?.id !== 'string' || typeof t.node !== 'string') return [];
        const p = t.position;
        const position =
          Array.isArray(p) && p.length === 3 && p.every(finite) ? ([p[0], p[1], p[2]] as Vec3) : null;
        const r = (Array.isArray(t.rotation) ? t.rotation : []) as number[];
        const norm = r.length === 4 && r.every(finite) ? Math.hypot(...r) : 0;
        const rotation: Quat =
          norm > 1e-9 ? [r[0] / norm, r[1] / norm, r[2] / norm, r[3] / norm] : [0, 0, 0, 1];
        return [{ id: t.id, node: t.node, position, rotation }];
      })
    : [];
  const junctions = (raw as { junctions?: unknown }).junctions;
  const triple = (v: unknown): v is Vec3 => Array.isArray(v) && v.length === 3 && v.every(finite);
  const placed: JunctionMount[] = Array.isArray(junctions)
    ? junctions.flatMap((j: Record<string, unknown>) =>
        typeof j?.node === 'string' && triple(j.position) && triple(j.axis)
          ? [{ node: j.node, position: [...j.position] as Vec3, axis: [...j.axis] as Vec3, ...(j.collector === true ? { collector: true as const } : {}) }]
          : [],
      )
    : [];
  return {
    ...(mounts.length > 0 ? { turbos: mounts } : {}),
    ...(placed.length > 0 ? { junctions: placed } : {}),
    ducts: ducts.map((d: Record<string, unknown>) => ({
      id: String(d.id),
      segments: (Array.isArray(d.segments) ? d.segments : []).map((sg) => makeSegment(sg)),
      from: d.from as DuctSource,
      to: d.to as DuctSink,
      ...(finite(d.headingYaw) ? { headingYaw: d.headingYaw } : {}),
      ...(finite(d.headingPitch) ? { headingPitch: d.headingPitch } : {}),
      ...(d.headingFrame === 'world' ? { headingFrame: 'world' as const } : {}),
      ...(typeof d.continues === 'string' ? { continues: d.continues } : {}),
      ...(typeof d.role === 'string' && ROLES.has(d.role) ? { role: d.role as ExhaustDuct['role'] } : {}),
      ...(d.fitted === true ? { fitted: true as const } : {}),
      ...(d.fitted === true && d.swing === true ? { swing: true as const } : {}),
    })),
  };
}

/**
 * The duct to select when nothing has been chosen yet: the first one leaving a valve, else the first.
 *
 * One rule, shared by the scene's handles and the panel's duct menu, so a reseed lands both on the same
 * duct. A graph always starts at the valves, but a hand-edited one need not list them first.
 */
export function defaultDuctId(graph: ExhaustGraph): string | undefined {
  return graph.ducts.find((d) => d.from.kind === 'valve')?.id ?? graph.ducts[0]?.id;
}

/** Whether any duct has had its heading fixed — drawn, or frozen by an edit — rather than worked out. */
export function hasBeenEdited(graph: ExhaustGraph): boolean {
  return graph.ducts.some((d) => d.headingYaw !== undefined || d.headingPitch !== undefined);
}

/** Node ids in the order they first appear, which is the order the solver indexes them in. */
export function nodeOrder(graph: ExhaustGraph): string[] {
  const seen: string[] = [];
  for (const duct of graph.ducts) {
    for (const ref of [duct.from, duct.to]) {
      if (ref.kind === 'node' && !seen.includes(ref.node)) seen.push(ref.node);
    }
  }
  return seen;
}

/** Every duct end that meets at `node`, with the side of the duct it is. */
export function endsAt(
  graph: ExhaustGraph,
  node: string,
): Array<{ duct: ExhaustDuct; end: 'inlet' | 'outlet' }> {
  const ends: Array<{ duct: ExhaustDuct; end: 'inlet' | 'outlet' }> = [];
  for (const duct of graph.ducts) {
    if (duct.from.kind === 'node' && duct.from.node === node) ends.push({ duct, end: 'inlet' });
    if (duct.to.kind === 'node' && duct.to.node === node) ends.push({ duct, end: 'outlet' });
  }
  return ends;
}

/**
 * Ducts that vent to air, downstream-most first.
 *
 * The order matters and is not cosmetic: `refreshMouthPaths` lays the mouths out *in a line* by
 * index and gives each its own delay and gain, so reordering them changes the sound. Node-fed ducts
 * come first because a node-fed duct is further downstream than a valve-fed one — which is a real
 * ordering rather than an arbitrary one: collectors, then cylinders venting alone.
 */
export function radiatingDucts(graph: ExhaustGraph): ExhaustDuct[] {
  const mouths = graph.ducts.filter((d) => d.to.kind === 'mouth');
  const rank = (d: ExhaustDuct) => (d.from.kind === 'node' ? 0 : 1);
  return mouths
    .map((d, i) => ({ d, i }))
    .sort((a, b) => rank(a.d) - rank(b.d) || a.i - b.i)
    .map((m) => m.d);
}

/** The duct each cylinder's valve feeds, or `null` if the graph does not give it one. */
export function valveDucts(graph: ExhaustGraph, cylinders: number): Array<ExhaustDuct | null> {
  const out: Array<ExhaustDuct | null> = new Array(cylinders).fill(null);
  for (const duct of graph.ducts) {
    if (duct.from.kind !== 'valve') continue;
    if (duct.from.cylinder < 0 || duct.from.cylinder >= cylinders) continue;
    out[duct.from.cylinder] ??= duct;
  }
  return out;
}

/**
 * A name for a duct that means something to whoever drew it.
 *
 * Derived rather than stored: a duct's identity is where it starts, and an id like `collector0` is for
 * the code. Nodes are numbered by the order they appear so the labels stay stable as ducts are added.
 *
 * A cylinder's own duct is named for what it is: a *primary* where it runs to a merge, as a header's
 * does, a *stub* where it only joins a manifold along the ports, and the cylinder's *exhaust* where it is
 * the whole of it, straight to the air. Never a runner, which is the intake's.
 */
export function ductLabel(graph: ExhaustGraph, duct: ExhaustDuct): string {
  if (duct.from.kind === 'valve') {
    const n = duct.from.cylinder + 1;
    if (duct.role === 'stub') return `Cylinder ${n} stub`;
    if (duct.to.kind === 'mouth') return `Cylinder ${n} exhaust`;
    return `Cylinder ${n} primary`;
  }
  if (duct.from.kind === 'free') {
    const loose = graph.ducts.filter((d) => d.from.kind === 'free');
    return `Loose pipe ${loose.indexOf(duct) + 1}`;
  }
  const turbo = graph.turbos?.findIndex((t) => t.node === (duct.from as { node: string }).node) ?? -1;
  if (turbo >= 0) return `After turbo ${turbo + 1}`;
  const nodes = nodeOrder(graph).filter((n) => !turboAt(graph, n));
  const at = nodes.indexOf(duct.from.node) + 1;
  const siblings = endsAt(graph, duct.from.node).filter((e) => e.end === 'inlet');
  if (siblings.length <= 1) return `After junction ${at}`;
  const which = siblings.findIndex((e) => e.duct.id === duct.id) + 1;
  return `After junction ${at}, branch ${which}`;
}

/**
 * The exhaust a layout choice describes, built from straight tube that snaps together.
 *
 * Each group of cylinders sharing an exhaust (`collectorGroups`) becomes a manifold along its bank —
 * see the comments inside — and a group across both banks of a V becomes a manifold per bank meeting
 * behind the engine. Cylinders venting alone keep the given runner geometry. The equal-length
 * alternative, every runner into one junction, is `compileCollectorLayout`.
 *
 * Every duct gets its *own copy* of the segments, which is what later lets one pipe be changed without
 * dragging the others with it.
 */
export function compileLayout(
  spec: EngineSpec,
  pipe: PipeSegment[],
  collector: PipeSegment[],
  turbos = 0,
): ExhaustGraph {
  const groups = collectorGroups(spec);
  const ducts: ExhaustDuct[] = [];
  const mounts: TurboMount[] = [];
  const copy = (segments: PipeSegment[]) => segments.map((s) => makeSegment(s));

  // Where each cylinder sits along the crank, for chaining a bank's runners in order.
  const pinOf = new Map<number, number>();
  crankPins(spec).forEach((pin, i) => pin.cylinders.forEach((c) => pinOf.set(c, i)));
  const spacing = cylinderSpacing(spec);
  const runnerDia = pipe.length > 0 ? segmentDiameter(pipe[pipe.length - 1]!, 1) : 0.042;
  /**
   * The widest the collector gets, short of a silencer's can: the most a manifold need ever widen to.
   *
   * Not its inlet. A fitted collector opens with a cone from just over a runner's bore, so capping at the
   * inlet would hold an inline six's manifold at 33 mm all the way along — five cylinders' gas through one
   * runner's worth of pipe. That chokes: the junction at its end clamps on nearly every sample and the gas
   * runs at 2300 K.
   */
  let collectorDia = runnerDia;
  for (const seg of collector) {
    if (seg.kind === 'chamber') continue;
    collectorDia = Math.max(collectorDia, segmentDiameter(seg, 0), segmentDiameter(seg, 1));
  }
  // Wider as it gathers more cylinders, for roughly constant gas speed: the rule `fittedExhaust` sizes a
  // collector by, up to the collector's own bore.
  const gathering = (n: number) => Math.min(Math.max(runnerDia * Math.sqrt(n) * 0.92, runnerDia), collectorDia);
  /** The collector, opening no narrower than the manifold it carries on from. */
  const collectorAfter = (manifoldDia: number): PipeSegment[] => {
    const segs = copy(collector);
    const first = segs[0];
    if (first && first.kind !== 'chamber' && first.dIn < manifoldDia) {
      segs[0] = makeSegment({ ...first, dIn: manifoldDia, ...(first.kind === 'pipe' ? { dOut: manifoldDia } : {}) });
    }
    return segs;
  };

  /** The node each cylinder's runner ends at, filled in per group below. */
  const runnerTo = new Map<number, string>();
  /** A cylinder's runner, where it is not the one given: the stub onto a manifold along the ports. */
  const runnerSegments = new Map<number, PipeSegment[]>();
  const stub = () => [makeSegment({ kind: 'pipe', length: MANIFOLD_STUB, dIn: runnerDia, dOut: runnerDia })];

  const groupCount = groups.reduce((max, g) => Math.max(max, g + 1), 0);
  const tail: ExhaustDuct[] = [];
  for (let g = 0; g < groupCount; g++) {
    const members = groups.flatMap((grp, c) => (grp === g ? [c] : []));
    if (members.length === 0) continue;

    /**
     * Each bank gets a manifold along its ports, built from straight tube: one link from each port to the
     * next and the outlet carrying on from the last, so a bank of four is three links and an outlet. The
     * ports are in a line one pin spacing apart along the crank, and that is the links' length. Each
     * cylinder feeds the manifold through a short stub, the only part of a runner left: see
     * `MANIFOLD_STUB` for why it cannot go altogether.
     */
    const banks = [...new Set(members.map((c) => physicalBank(spec, c)))];
    /**
     * Chain one bank's cylinders into a manifold ending at `last`. Returns the duct carrying the manifold
     * into `last`, for whatever leaves it to carry on from.
     *
     * The first cylinder's duct is its stub *and* the manifold's first length, turning at a corner onto
     * the line of ports. As a separate duct it would make a junction of just two pipes, which is a corner,
     * not a junction — and the junction solve does not hold one: measured on a V8 at 8500 rpm, those two
     * nodes go fully out of balance on 43 samples in a second while every three-way node stays under 1.3%.
     */
    const chain = (bankMembers: number[], last: string, tag: string, reverse = false): string | null => {
      const order = [...bankMembers].sort((a, b) => (pinOf.get(a) ?? a) - (pinOf.get(b) ?? b));
      if (reverse) order.reverse();
      if (order.length < 2) {
        if (order[0] !== undefined) runnerTo.set(order[0], last);
        return null;
      }
      // Junction k is where cylinder k's stub meets the manifold, for k from 1.
      const nodeAt = (k: number) => (k === order.length - 1 ? last : `${last}-${k}`);
      const gapAfter = (k: number) =>
        Math.abs((pinOf.get(order[k + 1]!) ?? k + 1) - (pinOf.get(order[k]!) ?? k)) * spacing;
      // The manifold after junction k has gathered k + 1 cylinders.
      const diaAfter = (k: number) => gathering(k + 1);

      // Along the line of ports, first to last: they differ only in where they sit along the crank.
      const first = exhaustPortOf(spec, order[0]!);
      const next = exhaustPortOf(spec, order[1]!);
      const along: Vec3 = [0, 0, Math.sign(next.position[2] - first.position[2]) || 1];
      const corner = turnBetweenDirs(first.direction, along);

      order.forEach((c, k) => {
        runnerSegments.set(
          c,
          k === 0
            ? [
                ...stub(),
                makeSegment({
                  kind: 'pipe',
                  length: gapAfter(0),
                  dIn: diaAfter(0),
                  dOut: diaAfter(0),
                  yaw: corner.yaw,
                  pitch: corner.pitch,
                }),
              ]
            : stub(),
        );
        runnerTo.set(c, nodeAt(Math.max(k, 1)));
      });

      let carrying = `runner${order[0]}`;
      for (let k = 1; k < order.length - 1; k++) {
        const id = `link${tag}-${k}`;
        tail.push({
          id,
          segments: [makeSegment({ kind: 'pipe', length: gapAfter(k), dIn: diaAfter(k), dOut: diaAfter(k) })],
          from: { kind: 'node', node: nodeAt(k) },
          to: { kind: 'node', node: nodeAt(k + 1) },
          continues: carrying,
          role: 'manifold',
        });
        carrying = id;
      }
      return carrying;
    };

    /**
     * Twin turbos on one bank, as on the RB26: the bank's front and rear halves each chain out to a turbo
     * at their own end of the engine, and the turbos' downpipes meet between them. Each downpipe's length
     * comes out of the collector, as the downpipes of a V's two banks do.
     */
    if (banks.length === 1 && turbos === 2 && mounts.length === 0 && members.length >= 2) {
      const order = [...members].sort((a, b) => (pinOf.get(a) ?? a) - (pinOf.get(b) ?? b));
      const half = Math.ceil(order.length / 2);
      const halves = [order.slice(0, half), order.slice(half)];
      let downDia = 0;
      halves.forEach((halfMembers, h) => {
        const node = `merge${g}-t${h}`;
        // The front half chains towards the front, the rear half towards the rear.
        chain(halfMembers, node, `${g}-t${h}`, h === 0);
        const dia = gathering(halfMembers.length) * 1.25;
        downDia = Math.max(downDia, dia);
        tail.push({
          id: `down${g}-t${h}`,
          segments: [makeSegment({ kind: 'pipe', length: TWIN_DOWNPIPE, dIn: dia, dOut: dia })],
          from: { kind: 'node', node },
          to: { kind: 'node', node: `merge${g}` },
          role: 'downpipe',
        });
        mounts.push({ id: `turbo${h + 1}`, node, position: null, rotation: [0, 0, 0, 1] });
      });
      tail.push({
        id: `collector${g}`,
        segments: shortened(collectorAfter(downDia), TWIN_DOWNPIPE),
        from: { kind: 'node', node: `merge${g}` },
        to: { kind: 'mouth' },
        role: 'collector',
      });
      continue;
    }

    if (banks.length === 1) {
      const lastLink = chain(members, `merge${g}`, `${g}`);
      tail.push({
        id: `collector${g}`,
        segments: members.length > 1 ? collectorAfter(gathering(members.length - 1)) : copy(collector),
        from: { kind: 'node', node: `merge${g}` },
        to: { kind: 'mouth' },
        ...(lastLink ? { continues: lastLink } : {}),
        role: 'collector',
      });
      continue;
    }

    /**
     * Across both banks — a V-twin's 2-into-1, a V8's 8-into-1 — each bank is chained on its own side and
     * the two sides meet behind the engine, on its centreline, one pin spacing past the last cylinder.
     *
     * A bank of one cylinder aims its runner straight there. A bank of several gets a downpipe from the end
     * of its manifold, cut to the length that reaches: the banks are mirror images, so the two downpipes are
     * the same length and meet exactly, at a plain fitting. That length is taken out of the collector, so
     * the path from each valve to the air — which is what the tuning depends on — keeps the length the
     * runner and collector give it.
     */
    const ends = banks.map((bank) => {
      const bankMembers = members.filter((c) => physicalBank(spec, c) === bank);
      const lastCyl = [...bankMembers].sort((a, b) => (pinOf.get(b) ?? b) - (pinOf.get(a) ?? a))[0]!;
      const port = exhaustPortOf(spec, lastCyl);
      const runner = bankMembers.length > 1 ? stub() : pipe;
      return { bank, bankMembers, end: sweepEnd(runner, port.position, port.direction).end };
    });
    const mid: Vec3 = [
      ends.reduce((a, e) => a + e.end[0], 0) / ends.length,
      ends.reduce((a, e) => a + e.end[1], 0) / ends.length,
      Math.max(...ends.map((e) => e.end[2])) + spacing,
    ];
    let downpipe = 0;
    let downpipeDia = 0;
    for (const { bank, bankMembers, end } of ends) {
      if (bankMembers.length === 1) {
        runnerTo.set(bankMembers[0]!, `merge${g}`);
        continue;
      }
      const bankNode = `merge${g}-b${bank}`;
      const carried = chain(bankMembers, bankNode, `${g}-b${bank}`);
      const length = distance(end, mid);
      downpipe = Math.max(downpipe, length);
      const dia = gathering(bankMembers.length);
      downpipeDia = Math.max(downpipeDia, dia);
      tail.push({
        id: `down${g}-b${bank}`,
        segments: [makeSegment({ kind: 'pipe', length, dIn: dia, dOut: dia })],
        from: { kind: 'node', node: bankNode },
        to: { kind: 'node', node: `merge${g}` },
        // The manifold carries on into it, so its junction is laid out on the manifold rather than
        // aimed. Without this, a bank of two — whose manifold is its first runner — would have both
        // runners aimed at a collar, and the fitting would come out a 29 cm collector. The downpipe itself
        // is still aimed where the banks meet: see `layoutGraph`.
        ...(carried ? { continues: carried } : {}),
        role: 'downpipe',
      });
    }
    tail.push({
      id: `collector${g}`,
      segments: shortened(downpipeDia > 0 ? collectorAfter(downpipeDia) : copy(collector), downpipe),
      from: { kind: 'node', node: `merge${g}` },
      to: { kind: 'mouth' },
      role: 'collector',
    });
  }

  groups.forEach((group, cylinder) => {
    const node = runnerTo.get(cylinder);
    const stubbed = runnerSegments.get(cylinder);
    ducts.push({
      id: `runner${cylinder}`,
      segments: stubbed ?? copy(pipe),
      from: { kind: 'valve', cylinder },
      to: group < 0 || !node ? { kind: 'mouth' } : { kind: 'node', node },
      role: stubbed ? 'stub' : 'runner',
    });
  });
  ducts.push(...tail);
  return { ducts, ...(mounts.length > 0 ? { turbos: mounts } : {}) };
}

/** How long each of a single bank's twin turbos' downpipes is, m. */
const TWIN_DOWNPIPE = 0.3;

/**
 * The equal-length alternative: every runner of a group into one junction, then its collector.
 *
 * The textbook tuned header — each
 * cylinder's path to air the same length, so its pulses reach the merge evenly spaced and matched
 * cylinders cancel their low orders. `compileLayout` builds manifolds instead, because that is what straight
 * pipes snapping together make; this is kept for anything that needs the symmetric system, which is what
 * the physics tests of cancellation and pulse spacing are about.
 */
/**
 * The exhaust `spec` asks for: equal-length headers into one merge per collector where it has
 * `exhaustHeaders` and something to merge, and a manifold along the ports otherwise.
 */
export function compileExhaust(
  spec: EngineSpec,
  pipe: PipeSegment[],
  collector: PipeSegment[],
  turbos = 0,
): ExhaustGraph {
  const graph =
    spec.exhaustHeaders && exhaustLayoutOf(spec) !== 'open'
      ? compileCollectorLayout(spec, pipe, collector)
      : compileLayout(spec, pipe, collector, turbos);
  // Otherwise a turbo goes where each collector starts, as many as asked for and there are collectors.
  if (turbos > 0 && !graph.turbos) {
    const at = graph.ducts.filter((d) => d.role === 'collector' && d.from.kind === 'node').slice(0, turbos);
    const mounts = at.map(
      (d, i): TurboMount => ({ id: `turbo${i + 1}`, node: (d.from as { node: string }).node, position: null, rotation: [0, 0, 0, 1] }),
    );
    if (mounts.length > 0) graph.turbos = mounts;
  }
  return graph;
}

export function compileCollectorLayout(
  spec: EngineSpec,
  pipe: PipeSegment[],
  collector: PipeSegment[],
): ExhaustGraph {
  const groups = collectorGroups(spec);
  const ducts: ExhaustDuct[] = groups.map((group, cylinder) => ({
    id: `runner${cylinder}`,
    segments: pipe.map((s) => makeSegment(s)),
    from: { kind: 'valve', cylinder },
    to: group < 0 ? { kind: 'mouth' } : { kind: 'node', node: `merge${group}` },
    role: 'runner',
  }));
  for (const g of [...new Set(groups)].filter((g) => g >= 0).sort((a, b) => a - b)) {
    ducts.push({
      id: `collector${g}`,
      segments: collector.map((s) => makeSegment(s)),
      from: { kind: 'node', node: `merge${g}` },
      to: { kind: 'mouth' },
      role: 'collector',
    });
  }
  return { ducts };
}

/**
 * How far a manifold stands off the ports, m: the stub each cylinder feeds it through.
 *
 * Not zero, because the solver models a valve as the inlet of a pipe — it cannot open one straight into a
 * junction — so every cylinder needs a duct of its own. And not much shorter than this, because the
 * shortest cell anywhere sets the time step for the whole exhaust: with the head port's own length on
 * top and the fewest cells a duct may have, this keeps one step per sample at 44.1 kHz with room to
 * spare. At 85 mm the cells would be 35 mm, just under the 37.3 mm that needs, and a V8 would pay double.
 */
const MANIFOLD_STUB = 0.1;

/**
 * `segments` with `length` taken out of its longest plain pipe, as far as that pipe can spare.
 *
 * For when a compiled exhaust needs extra pipe to reach — a downpipe to meet the other bank — and the path
 * to air should stay the length it was tuned to. Kept to at least a tenth of a metre, so a short collector
 * is shortened only as far as it sensibly can be.
 */
function shortened(segments: PipeSegment[], length: number): PipeSegment[] {
  if (length <= 0) return segments;
  let longest = -1;
  segments.forEach((seg, i) => {
    if (seg.kind === 'pipe' && (longest < 0 || seg.length > segments[longest]!.length)) longest = i;
  });
  if (longest >= 0) {
    const seg = segments[longest]!;
    seg.length = Math.max(seg.length - length, Math.min(seg.length, 0.1));
  }
  return segments;
}

/**
 * The runner and collector geometry a graph holds, to carry across a rebuild for a new topology.
 *
 * Only ducts that actually carry that geometry count: a runner compiled from it, or failing that one a
 * user drew from a port, and a compiled collector, or failing that a pipe venting to air after a
 * junction. A manifold's stubs, lengths and downpipes are generated from the engine, and carrying one
 * as "the runner" would make the next exhaust out of 10 cm pipes. `undefined` means keep what there was.
 */
export function carriedGeometry(graph: ExhaustGraph): { pipe?: PipeSegment[]; collector?: PipeSegment[] } {
  const generated = (d: ExhaustDuct) => d.role === 'stub' || d.role === 'manifold' || d.role === 'downpipe';
  const runner =
    graph.ducts.find((d) => d.role === 'runner') ??
    graph.ducts.find((d) => d.from.kind === 'valve' && !d.role && d.segments.length > 0);
  const collector =
    graph.ducts.find((d) => d.role === 'collector') ??
    graph.ducts.find((d) => d.from.kind === 'node' && d.to.kind === 'mouth' && !generated(d));
  return {
    ...(runner ? { pipe: runner.segments.map((sg) => makeSegment(sg)) } : {}),
    ...(collector ? { collector: collector.segments.map((sg) => makeSegment(sg)) } : {}),
  };
}

/**
 * The ducts a cylinder's gas passes through on its way to air, in order.
 *
 * What "the tuned length" means once the exhaust is a graph: a runner plus whatever it merges into,
 * however many stages that takes. Follows the first duct leaving each junction, which is the only
 * choice for every layout that exists today and an arbitrary but harmless one for a diverging Y.
 * Stops if it ever revisits a duct, so a loop cannot hang the panel.
 */
export function pathToAir(graph: ExhaustGraph, cylinder: number): ExhaustDuct[] {
  const path: ExhaustDuct[] = [];
  let duct = graph.ducts.find(
    (d) => d.from.kind === 'valve' && d.from.cylinder === cylinder,
  );
  const seen = new Set<string>();
  while (duct && !seen.has(duct.id)) {
    seen.add(duct.id);
    path.push(duct);
    if (duct.to.kind === 'mouth') break;
    const onward = endsAt(graph, duct.to.node).find((e) => e.end === 'inlet');
    duct = onward?.duct;
  }
  return path;
}

/**
 * Copy one runner's shape onto every other runner.
 *
 * What "apply to every cylinder" means. A symmetric engine is the normal case, so eight identical runners
 * should not need eight identical edits.
 *
 * The whole list is copied rather than the individual edit replayed, which is both simpler and more
 * robust: whatever was just done — a length, a diameter, a delete, a reorder — the others end up
 * identical without every mutation needing mirroring logic of its own.
 *
 * Only *runners* are linked. A collector is a different part of the exhaust and there is generally one of
 * it, so linking it to anything would be meaningless.
 */
export function copyToSiblingRunners(graph: ExhaustGraph, source: ExhaustDuct): void {
  if (source.from.kind !== 'valve') return;
  // What was drawn: a bend fitted into a turbo belongs to its own pipe, and each keeps its own.
  const drawn = drawnSegments;
  /**
   * Not onto, nor from, a runner that carries a manifold on.
   *
   * On a manifold the first cylinder's duct is its stub *and* the manifold's first length, so it is not
   * a sibling of the other stubs: copying a stub over it would cut the manifold short, and copying it
   * onto the stubs would put a length of manifold on every cylinder.
   */
  const carries = (d: ExhaustDuct) => graph.ducts.some((o) => o.continues === d.id);
  if (carries(source)) return;
  for (const other of graph.ducts) {
    if (other === source || other.from.kind !== 'valve' || carries(other)) continue;
    const bend = other.segments.slice(other.segments.length - fittedCount(other));
    other.segments = [...drawn(source).map((sg) => makeSegment(sg)), ...bend];
  }
}

/** An id of the form `prefix1`, `prefix2`, … that nothing in the graph is using. */
function freeId(taken: Set<string>, prefix: string): string {
  for (let n = 1; ; n++) {
    const id = `${prefix}${n}`;
    if (!taken.has(id)) return id;
  }
}

export function newDuctId(graph: ExhaustGraph, prefix = 'duct'): string {
  return freeId(new Set(graph.ducts.map((d) => d.id)), prefix);
}

export function newNodeId(graph: ExhaustGraph, prefix = 'join'): string {
  return freeId(new Set(nodeOrder(graph)), prefix);
}

/**
 * Cut a segment list at arc distance `x`, returning the two halves.
 *
 * What a T-branch needs: dropping a pipe onto the side of another one turns that one into two ducts with
 * a junction between them. The cut interpolates the straddling segment's diameters so the profile is
 * unchanged — the two halves describe the same duct as the original.
 *
 * **A `chamber` cannot be cut mid-body.** Its profile is defined across the whole segment — a throat, a
 * body, then a throat again — so half a chamber is not a chamber, and interpolating its diameter at the
 * cut would silently turn a muffler into a cone. A cut landing inside one is moved to whichever of its
 * ends is nearer.
 *
 * Returns `null` when there is nothing to cut: `x` at or beyond either end would leave an empty half, and
 * a duct with no segments is not a duct.
 */
export function splitSegments(
  segments: PipeSegment[],
  x: number,
): [PipeSegment[], PipeSegment[]] | null {
  if (segments.length === 0) return null;

  // Which segment holds the cut, and how far into it.
  let acc = 0;
  let index = -1;
  for (let i = 0; i < segments.length; i++) {
    const seg = segments[i]!;
    if (x < acc + seg.length) {
      index = i;
      break;
    }
    acc += seg.length;
  }
  if (index < 0) return null;

  const seg = segments[index]!;
  let within = x - acc;

  if (seg.kind === 'chamber') {
    // Move the cut to the nearer end of the chamber rather than halving it.
    within = within < seg.length / 2 ? 0 : seg.length;
  }

  const head = segments.slice(0, index).map((sg) => makeSegment(sg));
  const tail: PipeSegment[] = [];

  const EPS = 1e-6;
  if (within <= EPS) {
    // Cut falls on this segment's inlet: it belongs entirely to the tail.
    tail.push(...segments.slice(index).map((sg) => makeSegment(sg)));
  } else if (within >= seg.length - EPS) {
    head.push(makeSegment(seg));
    tail.push(...segments.slice(index + 1).map((sg) => makeSegment(sg)));
  } else {
    const u = within / seg.length;
    const mid = segmentDiameter(seg, u);
    head.push(
      makeSegment({ ...seg, id: undefined, length: within, dIn: seg.dIn, dOut: mid }),
    );
    // The segment's corner is at its start, which the head keeps; the tail carries straight on from the
    // cut. Copying the turn onto the tail as well would put a second corner where the pipe is cut.
    tail.push(
      makeSegment({
        ...seg,
        id: undefined,
        length: seg.length - within,
        dIn: mid,
        dOut: seg.dOut,
        yaw: 0,
        pitch: 0,
      }),
    );
    tail.push(...segments.slice(index + 1).map((sg) => makeSegment(sg)));
  }

  if (head.length === 0 || tail.length === 0) return null;
  return [head, tail];
}

/**
 * Split a duct in two at arc distance `x`, joined by a new junction.
 *
 * The upstream half keeps the duct's identity and its inlet; the downstream half is a new duct carrying
 * the original outlet. Returns the new node's id, which is what the branch being drawn then connects to.
 *
 * Note this changes how many ducts vent to air only if the original did — the halves inherit one outlet
 * between them — but it does renumber the duct list, and mouth order is what sets each mouth's delay. A
 * T-branch therefore changes the sound of the whole system slightly, which is correct: it *is* a
 * different exhaust.
 */
export function splitDuctAt(
  graph: ExhaustGraph,
  ductId: string,
  x: number,
): string | null {
  const index = graph.ducts.findIndex((d) => d.id === ductId);
  if (index < 0) return null;
  const duct = graph.ducts[index]!;
  const halves = splitSegments(duct.segments, x);
  if (!halves) return null;

  const node = newNodeId(graph);
  const downstream: ExhaustDuct = {
    id: newDuctId(graph, `${duct.id}-`),
    segments: halves[1],
    from: { kind: 'node', node },
    to: duct.to,
    continues: duct.id,
  };
  duct.segments = halves[0];
  duct.to = { kind: 'node', node };
  // Immediately after its upstream half, so the list reads along the flow.
  graph.ducts.splice(index + 1, 0, downstream);
  return node;
}

/**
 * The pipes leaving the junction `duct` ends at: its children, which carry on from it.
 *
 * A pipe with children is not deleted, nor anything else that would take them with it, so what was built
 * downstream is never lost as a side effect: they have to go first.
 */
export function childDucts(graph: ExhaustGraph, duct: ExhaustDuct): ExhaustDuct[] {
  if (duct.to.kind !== 'node') return [];
  return endsAt(graph, duct.to.node)
    .filter((e) => e.end === 'inlet')
    .map((e) => e.duct);
}

/**
 * Delete a duct and tidy the junctions at either end of it.
 *
 * A cylinder's runner is never deleted — every cylinder must have a pipe, and `validateGraph` rejects an
 * engine where one does not — so asking to delete one is refused, and so is deleting the only pipe into a
 * junction others leave (`childDucts`) until they are freed. Anything else goes, and the junctions it touched are tidied by `tidyJunctions`. Returns
 * whether anything was deleted.
 */
export function removeDuct(graph: ExhaustGraph, ductId: string, dirs?: DuctDirections): boolean {
  const duct = graph.ducts.find((d) => d.id === ductId);
  if (!duct || duct.from.kind === 'valve') return false;
  // The only pipe into a junction others leave is not deleted here: they would go with it. See
  // `loosenChildren`, which frees them first.
  const feeds = duct.to.kind === 'node' ? endsAt(graph, duct.to.node).filter((e) => e.end === 'outlet').length : 0;
  if (feeds === 1 && childDucts(graph, duct).length > 0) return false;
  graph.ducts = graph.ducts.filter((d) => d !== duct);
  tidyJunctions(graph, touchedNodes(duct), dirs);
  return true;
}

/**
 * Take a duct's far end off its junction and leave it open to the air.
 *
 * For a pipe that no longer reaches the junction it was joined to — shortened by deleting a segment,
 * say. Pipes are straight tube that snaps together, so one cut short does not stretch to stay joined,
 * and a fitting grown to bridge the gap would be 36 cm across. The junction it left is tidied.
 */
export function disconnectEnd(graph: ExhaustGraph, ductId: string, dirs?: DuctDirections): boolean {
  const duct = graph.ducts.find((d) => d.id === ductId);
  if (!duct || duct.to.kind !== 'node') return false;
  const node = duct.to.node;
  // The only pipe into a junction stays on it, or the pipes leaving it would go with nothing to feed them.
  // A turbo's outlet pipe goes with the last pipe into it, as it goes with the turbo, so long as nothing
  // carries on from it.
  const feeds = endsAt(graph, node).filter((e) => e.end === 'outlet');
  const outs = childDucts(graph, duct);
  const blocked = turboAt(graph, node) ? outs.some((out) => childDucts(graph, out).length > 0) : outs.length > 0;
  if (feeds.length === 1 && blocked) return false;
  duct.to = { kind: 'mouth' };
  releaseBend(duct);
  for (const d of graph.ducts) if (d.continues === duct.id) delete d.continues;
  tidyJunctions(graph, [node], dirs);
  return true;
}

/**
 * Delete a junction.
 *
 * The pipes that went into it end in open air where it was. A pipe running straight *through* is rejoined:
 * a tee's through pipe was only split so something could join it, so deleting the tee rejoins it into one
 * pipe and leaves the branch that joined it open. Where the graph records which pipe carries on
 * (`ExhaustDuct.continues`), that decides it; otherwise the caller says, from the geometry, as `[in, out]`.
 * Refused, returning `false`, while any other pipe leaves it: see `junctionRemoval`. With `outsGo` those
 * go with it instead, as a turbo's outlet pipe does.
 */
export function removeJunction(
  graph: ExhaustGraph,
  node: string,
  through?: [string, string] | null,
  dirs?: DuctDirections,
  outsGo = false,
): boolean {
  const plan = junctionRemoval(graph, node, through, outsGo);
  if (!plan) return false;
  const { feeds, outs, keepIn, keepOut } = plan;

  const touched = new Set<string>();
  for (const feed of feeds) {
    if (feed === keepIn) continue;
    feed.to = { kind: 'mouth' };
    releaseBend(feed);
  }
  for (const out of outs) {
    if (out === keepOut) continue;
    graph.ducts = graph.ducts.filter((d) => d !== out);
    for (const n of touchedNodes(out)) if (n !== node) touched.add(n);
  }
  if (keepIn && keepOut) fuse(graph, keepIn, keepOut, dirs);
  tidyJunctions(graph, touched, dirs);
  return true;
}

/**
 * What deleting the junction at `node` would do: which pipes run into and out of it, and the pair that runs
 * straight through, to be rejoined. `null` where it is refused: where any pipe leaves it but the one running
 * straight through, which has to be deleted first.
 */
export function junctionRemoval(
  graph: ExhaustGraph,
  node: string,
  through?: [string, string] | null,
  outsGo = false,
): { feeds: ExhaustDuct[]; outs: ExhaustDuct[]; keepIn?: ExhaustDuct; keepOut?: ExhaustDuct } | null {
  const ends = endsAt(graph, node);
  const feeds = ends.filter((e) => e.end === 'outlet').map((e) => e.duct);
  const outs = ends.filter((e) => e.end === 'inlet').map((e) => e.duct);
  // What the graph says runs through, where it says — a manifold, or a pipe that was split — else the
  // caller's reading of the geometry. A manifold widens at each junction, so geometry alone misses it.
  const carried = outs.find((d) => d.continues && feeds.some((f) => f.id === d.continues));
  const via: [string, string] | null | undefined = carried ? [carried.continues!, carried.id] : through;
  const keepIn = via ? feeds.find((d) => d.id === via[0]) : undefined;
  const keepOut = via ? outs.find((d) => d.id === via[1]) : undefined;
  // Nothing may leave it but a pipe running straight through, which is rejoined: a pipe branching off it,
  // or carrying the flow on from it, goes first. With `outsGo`, as for a turbo, whose outlet pipe comes
  // with it, the pipes leaving go with it instead, so long as nothing carries on from them.
  const others = outs.filter((out) => out !== keepOut);
  if (outsGo ? others.some((out) => childDucts(graph, out).length > 0) : others.length > 0) return null;
  return { feeds, outs, ...(keepIn ? { keepIn } : {}), ...(keepOut ? { keepOut } : {}) };
}

/**
 * Put junctions back into a state the solver accepts after something attached to them was removed.
 *
 * A junction nothing flows into takes its outgoing pipes with it, since their gas would come from nowhere,
 * and that can empty the junctions *they* led to, so this works outward until it settles. A junction with
 * nothing leaving it is dissolved and the pipes that fed it end in open air. A junction left with exactly
 * one pipe in and one out is no longer joining anything, so the two are rejoined into one pipe — which is
 * what undoes the split a branch made, once the branch is gone.
 */
export function tidyJunctions(graph: ExhaustGraph, nodes: Iterable<string>, dirs?: DuctDirections): void {
  tidy(graph, nodes, dirs);
  // A junction that has gone takes the place it was moved to with it.
  if (graph.junctions) {
    const live = new Set(nodeOrder(graph));
    graph.junctions = graph.junctions.filter((j) => live.has(j.node));
    if (graph.junctions.length === 0) delete graph.junctions;
  }
}

function tidy(graph: ExhaustGraph, nodes: Iterable<string>, dirs?: DuctDirections): void {
  const queue = [...nodes];
  for (let guard = 0; guard < graph.ducts.length * 4 + 16 && queue.length > 0; guard++) {
    const node = queue.shift()!;
    const ends = endsAt(graph, node);
    const feeds = ends.filter((e) => e.end === 'outlet').map((e) => e.duct);
    const outs = ends.filter((e) => e.end === 'inlet').map((e) => e.duct);
    if (ends.length === 0) continue;

    if (feeds.length === 0) {
      graph.ducts = graph.ducts.filter((d) => !outs.includes(d));
      for (const out of outs) for (const n of touchedNodes(out)) if (n !== node) queue.push(n);
    } else if (turboAt(graph, node)) {
      // A turbo stays as it is, fed, with or without a pipe drawn from its outlet.
    } else if (outs.length === 0 && feeds.length < 2) {
      // A junction of one pipe is not one; two or more are a merge waiting for the pipe after it.
      for (const feed of feeds) {
        feed.to = { kind: 'mouth' };
        releaseBend(feed);
      }
    } else if (feeds.length === 1 && outs.length === 1) {
      // A junction fixed in place stays, as something may yet be drawn into it, unless it is a pipe that
      // was split for a branch, and the branch has gone.
      if (!junctionAt(graph, node) || outs[0]!.continues === feeds[0]!.id) fuse(graph, feeds[0]!, outs[0]!, dirs);
    }
  }
}

/**
 * Where ducts point, from the layout, for keeping pipes where they were when two are rejoined.
 *
 * `end` is the way a duct is going where it finishes; `first` the way its first segment runs. Optional
 * everywhere: the graph has no geometry of its own, so without it a rejoin keeps each segment's turn as
 * it was.
 */
export type DuctDirections = (ductId: string) => { end: Vec3; first: Vec3 } | undefined;

/**
 * Join `out` onto the end of `into`, which it continues: one pipe where there were two.
 *
 * `out`'s first segment turns off the junction's direction while the two are separate, and off the end of
 * `into` once they are joined, which need not be the same — a manifold's next length runs along the bank,
 * off a stub that points out of the port. Given the directions, that segment's turn is recomputed so it
 * still runs the way it did; without them the rest of the manifold would swing round to follow the stub.
 */
function fuse(graph: ExhaustGraph, into: ExhaustDuct, out: ExhaustDuct, dirs?: DuctDirections): void {
  const into_ = dirs?.(into.id);
  const out_ = dirs?.(out.id);
  const first = out.segments[0];
  if (into_ && out_ && first) {
    const turn = turnBetweenDirs(into_.end, out_.first);
    out.segments[0] = makeSegment({ ...first, yaw: turn.yaw, pitch: turn.pitch });
  }
  // Where the two meet in one straight tube, as where a branch cut a segment in two, they are one again.
  const last = into.segments[into.segments.length - 1];
  const next = out.segments[0];
  if (last && next && oneTube(last, next)) {
    into.segments[into.segments.length - 1] = makeSegment({
      ...last,
      length: last.length + next.length,
      dOut: next.dOut,
      offsetOut: next.offsetOut,
    });
    into.segments.push(...out.segments.slice(1));
  } else {
    into.segments.push(...out.segments);
  }
  into.to = out.to;
  // A bend it was fitted with is in the middle of it now; only the one `out` finished in is fitted still.
  if (out.fitted) into.fitted = true;
  else delete into.fitted;
  if (out.swing) into.swing = true;
  else delete into.swing;
  // Anything that carried straight on from `out` now carries on from `into`, which it has become.
  for (const d of graph.ducts) if (d.continues === out.id) d.continues = into.id;
  graph.ducts = graph.ducts.filter((d) => d !== out);
}

/**
 * Whether `b` carries straight on from `a` as the same tube: both straight pipe, `b` not turning off `a`,
 * and one taper running through both, so that joined they are the one segment `splitSegments` cut.
 */
function oneTube(a: PipeSegment, b: PipeSegment): boolean {
  const EPS = 1e-6;
  const pipe = (s: PipeSegment) => (s.kind === 'pipe' || s.kind === 'cone') && !s.curve;
  if (!pipe(a) || !pipe(b) || a.kind !== b.kind) return false;
  if (Math.abs(b.yaw) > EPS || Math.abs(b.pitch) > EPS) return false;
  if (Math.abs(a.dOut - b.dIn) > EPS) return false;
  if (a.length <= 0 || b.length <= 0) return false;
  const taper = (s: PipeSegment) => (s.dOut - s.dIn) / s.length;
  return Math.abs(taper(a) - taper(b)) < 1e-4;
}

/**
 * Take off the bend a duct was fitted into a turbo with, now it is not joined to one: the bend was only
 * ever the way from the pipe as drawn to the turbo's inlet, so the pipe ends where it was drawn to.
 */
export function releaseBend(duct: ExhaustDuct): void {
  if (!duct.fitted) return;
  duct.segments = drawnSegments(duct);
  delete duct.fitted;
  delete duct.swing;
}

function touchedNodes(duct: ExhaustDuct): string[] {
  const nodes: string[] = [];
  if (duct.from.kind === 'node') nodes.push(duct.from.node);
  if (duct.to.kind === 'node') nodes.push(duct.to.node);
  return nodes;
}

/**
 * Turn a duct's outlet into a junction, so something else can be drawn into it.
 *
 * Only the junction: the pipe carrying the merged flow on from it is drawn, not made. Until it is, the
 * junction's pipes end in open air there (`solverGraph`). With `at`, where the outlet is and which way it
 * points, the junction is fixed there, and the pipes joining it bend in to meet it.
 *
 * Returns the new node's id, or `null` if the duct does not end in air, in which case it already has a
 * junction and that is what the caller should connect to.
 */
export function joinDuctEnd(
  graph: ExhaustGraph,
  ductId: string,
  at?: { position: Vec3; axis: Vec3 },
): string | null {
  const duct = graph.ducts.find((d) => d.id === ductId);
  if (!duct || duct.to.kind !== 'mouth' || duct.segments.length === 0) return null;
  const node = newNodeId(graph);
  duct.to = { kind: 'node', node };
  if (at) (graph.junctions ??= []).push({ node, position: [...at.position], axis: [...at.axis] });
  return node;
}

/** The ducts gas reaches: from the cylinders' valves, through whatever they join. */
function fedDucts(graph: ExhaustGraph): Set<string> {
  const fed = new Set<string>();
  const frontier = graph.ducts.filter((d) => d.from.kind === 'valve');
  for (const d of frontier) fed.add(d.id);
  for (let guard = 0; guard < graph.ducts.length + 1 && frontier.length > 0; guard++) {
    const duct = frontier.pop()!;
    if (duct.to.kind !== 'node') continue;
    for (const e of endsAt(graph, duct.to.node)) {
      if (e.end !== 'inlet' || fed.has(e.duct.id)) continue;
      fed.add(e.duct.id);
      frontier.push(e.duct);
    }
  }
  return fed;
}

/** How far the gas out of a turbo with no pipe drawn from its outlet runs before the air, m. */
const TURBO_EXIT = 0.05;

/**
 * The graph as the solver is given it: without loose pipes, nor anything reached only through one, since
 * no gas reaches them, and with the pipes into a junction nothing leaves yet ending in open air. What is
 * left is every duct fed from a cylinder. A turbo fed with nothing drawn from its outlet exhausts to the air
 * at its outlet flange, through the shortest of pipes there, a little wider than what feeds it.
 */
export function solverGraph(graph: ExhaustGraph): ExhaustGraph {
  const leaving = new Set(graph.ducts.flatMap((d) => (d.from.kind === 'node' ? [d.from.node] : [])));
  const ending = graph.ducts.some((d) => d.to.kind === 'node' && !leaving.has(d.to.node));
  if (!ending && !graph.ducts.some((d) => d.from.kind === 'free')) return graph;
  const fed = fedDucts(graph);
  const turbos = new Set((graph.turbos ?? []).map((t) => t.node));
  const kept = graph.ducts.filter((d) => fed.has(d.id));
  const ducts = kept
    // A junction nothing leaves yet: its pipes end in open air there.
    .map((d) => (d.to.kind === 'node' && !leaving.has(d.to.node) && !turbos.has(d.to.node) ? { ...d, to: { kind: 'mouth' as const } } : d));
  for (const node of turbos) {
    if (leaving.has(node)) continue;
    let area = 0;
    for (const d of kept) {
      const last = d.segments[d.segments.length - 1];
      if (d.to.kind === 'node' && d.to.node === node && last) area += (Math.PI * segmentDiameter(last, 1) ** 2) / 4;
    }
    if (area === 0) continue;
    const dia = 1.3 * Math.sqrt((4 * area) / Math.PI);
    ducts.push({
      id: freeId(new Set(ducts.map((d) => d.id)), `${node}-exit`),
      segments: [makeSegment({ kind: 'pipe', length: TURBO_EXIT, dIn: dia, dOut: dia })],
      from: { kind: 'node', node },
      to: { kind: 'mouth' },
    });
  }
  return { ...graph, ducts };
}

/**
 * Put down a loose pipe at `position`, a straight length of `dia` bore heading along world +x. Returns its
 * id.
 */
export function placeLoosePipe(graph: ExhaustGraph, position: Vec3, dia: number): string {
  const id = newDuctId(graph, 'loose');
  graph.ducts.push({
    id,
    segments: [makeSegment({ kind: 'pipe', length: 0.3, dIn: dia, dOut: dia })],
    from: { kind: 'free', position: [...position] },
    to: { kind: 'mouth' },
    headingYaw: 0,
    headingPitch: 0,
    headingFrame: 'world',
  });
  return id;
}

/**
 * Attach `ductId`'s far end to the start of the loose pipe `looseId`, which heads along `axis`. Its start
 * becomes a junction, fixed where the loose pipe began, and the loose pipe carries on from it: attached, it
 * is a pipe like any other. Returns the junction.
 */
export function attachToLooseStart(graph: ExhaustGraph, ductId: string, looseId: string, axis: Vec3): string | null {
  const duct = graph.ducts.find((d) => d.id === ductId);
  const loose = graph.ducts.find((d) => d.id === looseId);
  if (!duct || !loose || loose.from.kind !== 'free') return null;
  const node = newNodeId(graph);
  (graph.junctions ??= []).push({ node, position: [...loose.from.position], axis: [...axis] });
  duct.to = { kind: 'node', node };
  loose.from = { kind: 'node', node };
  loose.headingFrame = 'world';
  return node;
}

/**
 * Reasons a graph cannot be solved, as readable sentences. Empty means it can.
 *
 * Checked rather than trusted because a drawn graph is user input: a half-finished route or a
 * deleted duct can leave a node with one end or a cylinder with no pipe, and the solver's failure
 * mode for that is silence rather than a message.
 */
export function validateGraph(graph: ExhaustGraph, cylinders: number): string[] {
  const problems: string[] = [];
  const ids = new Set<string>();
  for (const duct of graph.ducts) {
    if (ids.has(duct.id)) problems.push(`two ducts share the id "${duct.id}"`);
    ids.add(duct.id);
  }

  const perValve = new Map<number, number>();
  for (const duct of graph.ducts) {
    if (duct.from.kind !== 'valve') continue;
    perValve.set(duct.from.cylinder, (perValve.get(duct.from.cylinder) ?? 0) + 1);
  }
  for (let c = 0; c < cylinders; c++) {
    const n = perValve.get(c) ?? 0;
    if (n === 0) problems.push(`cylinder ${c + 1} has no exhaust pipe`);
    if (n > 1) problems.push(`cylinder ${c + 1} has ${n} pipes on its exhaust port`);
  }
  for (const [c] of perValve) {
    if (c < 0 || c >= cylinders) problems.push(`a pipe is attached to cylinder ${c + 1}, which does not exist`);
  }

  for (const node of nodeOrder(graph)) {
    const ends = endsAt(graph, node);
    const downstream = ends.filter((e) => e.end === 'inlet');
    const upstream = ends.filter((e) => e.end === 'outlet');
    // A turbo fed by one pipe, with nothing drawn from its outlet, exhausts to the air there.
    if (ends.length < 2 && !turboAt(graph, node)) problems.push(`junction "${node}" joins only one pipe`);
    if (upstream.length === 0) problems.push(`junction "${node}" has nothing flowing into it`);
    // A junction with nothing leaving it yet is its pipes ending in air there: see `solverGraph`.
    if (downstream.length > 1 && turboAt(graph, node)) {
      problems.push(`the turbo at "${node}" has ${downstream.length} pipes leaving its one outlet`);
    }
  }

  // Every duct must trace back to a valve, or the gas in it came from nowhere: or to a loose pipe, which
  // carries no gas yet and is not given to the solver (`solverGraph`).
  const reachable = new Set<string>();
  const frontier = graph.ducts.filter((d) => d.from.kind === 'valve' || d.from.kind === 'free');
  for (const d of frontier) reachable.add(d.id);
  for (let guard = 0; guard < graph.ducts.length + 1 && frontier.length > 0; guard++) {
    const duct = frontier.pop()!;
    if (duct.to.kind !== 'node') continue;
    for (const e of endsAt(graph, duct.to.node)) {
      if (e.end !== 'inlet' || reachable.has(e.duct.id)) continue;
      reachable.add(e.duct.id);
      frontier.push(e.duct);
    }
  }
  for (const duct of graph.ducts) {
    if (!reachable.has(duct.id)) problems.push(`pipe "${duct.id}" is not connected to any cylinder`);
  }

  return problems;
}
