/**
 * The exhaust as a graph of ducts joined at nodes.
 *
 * This replaces "one primary geometry shared by every cylinder, plus one collector per bank" as the
 * thing the solver is built from. That shape was fine while the topology came from a dropdown, but it
 * cannot express what drawing pipes implies: runners of different lengths, a tri-Y, or a branch part
 * way along another duct.
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
  makeSegment,
  physicalBank,
  segmentDiameter,
} from './spec.js';

/** What feeds a duct's inlet. */
export type DuctSource =
  | { kind: 'valve'; cylinder: number }
  | { kind: 'node'; node: string };

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
   * was: without it the junction took its direction from the average of its feeds' *starting* directions
   * — right for a collector's runners, which all head one way, and a 90-degree swing for a pipe attached
   * from the other side — and the collar search re-aimed the pipe being attached to.
   */
  continues?: string;
  /**
   * What a compiled duct is for, where it was compiled rather than drawn.
   *
   * A manifold's stubs and lengths are generated from the engine, not from the runner and collector
   * geometry a preset gives, so anything that reads that geometry back out of a graph — carrying it
   * across a change of cylinder count, say — has to know which ducts actually hold it. Reading "the
   * first runner" and "the first pipe after a junction" off a manifold found a stub and a manifold length.
   */
  role?: 'runner' | 'stub' | 'manifold' | 'downpipe' | 'collector';
}

export interface ExhaustGraph {
  ducts: ExhaustDuct[];
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
  return {
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
 * ordering rather than an arbitrary one, and happens to be the order the pre-graph code produced
 * (`[...collectors, ...soloPrimaries]`), so the change to a graph can be shown to be inert.
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
 */
export function ductLabel(graph: ExhaustGraph, duct: ExhaustDuct): string {
  if (duct.from.kind === 'valve') return `Cylinder ${duct.from.cylinder + 1} runner`;
  const nodes = nodeOrder(graph);
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
): ExhaustGraph {
  const groups = collectorGroups(spec);
  const ducts: ExhaustDuct[] = [];
  const copy = (segments: PipeSegment[]) => segments.map((s) => makeSegment(s));

  // Where each cylinder sits along the crank, for chaining a bank's runners in order.
  const pinOf = new Map<number, number>();
  crankPins(spec).forEach((pin, i) => pin.cylinders.forEach((c) => pinOf.set(c, i)));
  const spacing = cylinderSpacing(spec);
  const runnerDia = pipe.length > 0 ? segmentDiameter(pipe[pipe.length - 1]!, 1) : 0.042;
  /**
   * The widest the collector gets, short of a silencer's can: the most a manifold need ever widen to.
   *
   * Not its inlet, which is where it used to be read. A fitted collector opens with a cone from just over
   * a runner's bore, so capping at the inlet held an inline six's manifold at 33 mm all the way along —
   * five cylinders' gas through one runner's worth of pipe. It choked: the junction at its end clamped on
   * nearly every sample and the gas ran at 2300 K.
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
     * the line of ports. As a separate duct it made a junction of just two pipes, which is a corner, not a
     * junction — and the junction solve does not hold one: measured on a V8 at 8500 rpm, those two nodes
     * went fully out of balance on 43 samples in a second while every three-way node stayed under 1.3%.
     */
    const chain = (bankMembers: number[], last: string, tag: string): string | null => {
      const order = [...bankMembers].sort((a, b) => (pinOf.get(a) ?? a) - (pinOf.get(b) ?? b));
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
     * the path from each valve to the air — which is what the tuning depends on — is as long as it was.
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
        // aimed. Without this, a bank of two — whose manifold is its first runner — had both runners
        // aimed at a collar, and the fitting came out a 29 cm collector. The downpipe itself is still
        // aimed where the banks meet: see `layoutGraph`.
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
  return { ducts };
}

/**
 * The equal-length alternative: every runner of a group into one junction, then its collector.
 *
 * What the compiled exhaust used to be for every engine, and still the textbook tuned header — each
 * cylinder's path to air the same length, so its pulses reach the merge evenly spaced and matched
 * cylinders cancel their low orders. `compileLayout` builds manifolds instead, because that is what straight
 * pipes snapping together make; this is kept for anything that needs the symmetric system, which is what
 * the physics tests of cancellation and pulse spacing are about.
 */
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
 * spare. At 85 mm it was 35 mm cells, just under the 37.3 mm that needs, and a V8 paid double.
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
 * as "the runner" made the next exhaust out of 10 cm pipes. `undefined` means keep what there was.
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
 * What "apply to every cylinder" means. A symmetric engine is the normal case and used to be the only
 * one the model could express — every cylinder shared a single segment array — so eight identical runners
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
    other.segments = source.segments.map((sg) => makeSegment(sg));
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
    // cut. Copying the turn onto the tail as well put a second corner where the pipe was cut.
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
 * Delete a duct and tidy the junctions at either end of it.
 *
 * A cylinder's runner is never deleted — every cylinder must have a pipe, and `validateGraph` rejects an
 * engine where one does not — so asking to delete one is refused. Anything else goes, and the junctions it
 * touched are tidied by `tidyJunctions`. Returns whether anything was deleted.
 */
export function removeDuct(graph: ExhaustGraph, ductId: string, dirs?: DuctDirections): boolean {
  const duct = graph.ducts.find((d) => d.id === ductId);
  if (!duct || duct.from.kind === 'valve') return false;
  graph.ducts = graph.ducts.filter((d) => d !== duct);
  tidyJunctions(graph, touchedNodes(duct), dirs);
  return true;
}

/**
 * Take a duct's far end off its junction and leave it open to the air.
 *
 * For a pipe that no longer reaches the junction it was joined to — shortened by deleting a segment,
 * say. Pipes are straight tube that snaps together, so one cut short does not stretch to stay joined,
 * and a fitting grown to bridge the gap was 36 cm across. The junction it left is tidied.
 */
export function disconnectEnd(graph: ExhaustGraph, ductId: string, dirs?: DuctDirections): void {
  const duct = graph.ducts.find((d) => d.id === ductId);
  if (!duct || duct.to.kind !== 'node') return;
  const node = duct.to.node;
  duct.to = { kind: 'mouth' };
  for (const d of graph.ducts) if (d.continues === duct.id) delete d.continues;
  tidyJunctions(graph, [node], dirs);
}

/**
 * Delete a junction.
 *
 * The pipes that went into it end in open air where it was, and the pipes that came out of it go too,
 * since nothing feeds them any more. The exception is a pipe running straight *through*: a tee's through
 * pipe was only split so something could join it, so deleting the tee rejoins it into one pipe and takes
 * away just the branch. Where the graph records which pipe carries on (`ExhaustDuct.continues`), that
 * decides it; otherwise the caller says, from the geometry, as `[in, out]`.
 */
export function removeJunction(
  graph: ExhaustGraph,
  node: string,
  through?: [string, string] | null,
  dirs?: DuctDirections,
): void {
  const ends = endsAt(graph, node);
  const feeds = ends.filter((e) => e.end === 'outlet').map((e) => e.duct);
  const outs = ends.filter((e) => e.end === 'inlet').map((e) => e.duct);
  // What the graph says runs through, where it says — a manifold, or a pipe that was split — else the
  // caller's reading of the geometry. A manifold widens at each junction, so geometry alone misses it.
  const carried = outs.find((d) => d.continues && feeds.some((f) => f.id === d.continues));
  const via: [string, string] | null | undefined = carried ? [carried.continues!, carried.id] : through;
  const keepIn = via ? feeds.find((d) => d.id === via[0]) : undefined;
  const keepOut = via ? outs.find((d) => d.id === via[1]) : undefined;

  const touched = new Set<string>();
  for (const feed of feeds) if (feed !== keepIn) feed.to = { kind: 'mouth' };
  for (const out of outs) {
    if (out === keepOut) continue;
    graph.ducts = graph.ducts.filter((d) => d !== out);
    for (const n of touchedNodes(out)) if (n !== node) touched.add(n);
  }
  if (keepIn && keepOut) fuse(graph, keepIn, keepOut, dirs);
  tidyJunctions(graph, touched, dirs);
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
    } else if (outs.length === 0) {
      for (const feed of feeds) feed.to = { kind: 'mouth' };
    } else if (feeds.length === 1 && outs.length === 1) {
      fuse(graph, feeds[0]!, outs[0]!, dirs);
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
 * `out`'s first segment used to turn off the junction's direction, and now turns off the end of `into`,
 * which need not be the same — a manifold's next length ran along the bank, off a stub that points out
 * of the port. Given the directions, that segment's turn is recomputed so it still runs the way it did;
 * without them the rest of the manifold swung round to follow the stub.
 */
function fuse(graph: ExhaustGraph, into: ExhaustDuct, out: ExhaustDuct, dirs?: DuctDirections): void {
  const into_ = dirs?.(into.id);
  const out_ = dirs?.(out.id);
  const first = out.segments[0];
  if (into_ && out_ && first) {
    const turn = turnBetweenDirs(into_.end, out_.first);
    out.segments[0] = makeSegment({ ...first, yaw: turn.yaw, pitch: turn.pitch });
  }
  into.segments.push(...out.segments);
  into.to = out.to;
  // Anything that carried straight on from `out` now carries on from `into`, which it has become.
  for (const d of graph.ducts) if (d.continues === out.id) d.continues = into.id;
  graph.ducts = graph.ducts.filter((d) => d !== out);
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
 * Unlike `splitDuctAt` this has no existing pipe to hand the junction's outlet, because the duct ended in
 * open air — so one is invented. That is not a liberty: merging two pipes physically *requires* a pipe
 * after the merge, and a collector is exactly that. It is sized so its area is the sum of what feeds it,
 * which is ordinary exhaust practice, and it is a normal duct that can be edited or deleted afterwards.
 *
 * Returns the new node's id, or `null` if the duct does not end in air — in which case it already has a
 * junction and that is what the caller should connect to.
 */
export function joinDuctEnd(
  graph: ExhaustGraph,
  ductId: string,
  extraArea = 0,
): string | null {
  const index = graph.ducts.findIndex((d) => d.id === ductId);
  if (index < 0) return null;
  const duct = graph.ducts[index]!;
  if (duct.to.kind !== 'mouth' || duct.segments.length === 0) return null;

  const outletDia = segmentDiameter(duct.segments[duct.segments.length - 1]!, 1);
  const area = (Math.PI * outletDia * outletDia) / 4 + Math.max(extraArea, 0);
  const dia = Math.sqrt((4 * area) / Math.PI);

  const node = newNodeId(graph);
  const onward: ExhaustDuct = {
    id: newDuctId(graph, 'collector'),
    segments: [makeSegment({ kind: 'pipe', length: 0.3, dIn: dia, dOut: dia })],
    from: { kind: 'node', node },
    to: { kind: 'mouth' },
    continues: duct.id,
  };
  duct.to = { kind: 'node', node };
  graph.ducts.splice(index + 1, 0, onward);
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
    if (ends.length < 2) problems.push(`junction "${node}" joins only one pipe`);
    if (upstream.length === 0) problems.push(`junction "${node}" has nothing flowing into it`);
    if (downstream.length === 0) problems.push(`junction "${node}" has no pipe leaving it`);
  }

  // Every duct must trace back to a valve, or the gas in it came from nowhere.
  const reachable = new Set<string>();
  const frontier = graph.ducts.filter((d) => d.from.kind === 'valve');
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
