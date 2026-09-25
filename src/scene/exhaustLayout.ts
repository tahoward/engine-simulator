/**
 * Where each duct of the exhaust is drawn: one primary per cylinder, plus a collector per group.
 *
 * Pure geometry, no scene objects, so the layout can be tested for the thing that actually went
 * wrong — pipes occupying the same space — rather than inspected by eye.
 *
 * None of this touches the sound. The 1D solver integrates area against *axial* distance, so how a
 * runner is routed between its port and the junction is a rendering choice; the physics has already
 * merged them by the time anything is drawn.
 */

import * as THREE from 'three';

import { segmentDiameter } from '../model/spec.js';
import {
  endsAt,
  nodeOrder,
  type DuctDirections,
  type ExhaustDuct,
  type ExhaustGraph,
} from '../model/exhaustGraph.js';
import { layoutPipe, pipeSpan, solveHeading, turnBetween, turnHeading } from './PipeMesh.js';
import type { JointLimb, JointPlacement } from './jointMesh.js';

export interface ExhaustPort {
  position: THREE.Vector3;
  direction: THREE.Vector3;
}

export interface DuctPlacement {
  origin: THREE.Vector3;
  heading: THREE.Vector3;
}

/**
 * Where each duct meets a junction, for `jointMesh` to build geometry from.
 *
 * Limbs rather than a profile. The junction used to be described as a mouth radius, a throat radius and a
 * length — enough to lathe a surface of revolution about the junction's axis, and nothing more. That could
 * only ever *enclose* the pipes, because they are not axisymmetric about the joint: they arrive at their own
 * angles and offsets, so a body wide enough to contain them all was wider than any of them, which read as a
 * balloon on a 2-into-1. A tee had to be excluded from having a body at all, since no surface of revolution
 * around one makes sense.
 *
 * Giving the mesh the actual limbs lets it size a fitting to the pipes instead — a ball where they meet at a
 * point, a collector drum where they arrive spread apart — with every pipe end inside it, so there is no gap.
 */
export interface ExhaustPlacement {
  /** Where each duct is drawn, by duct id. */
  ducts: Map<string, DuctPlacement>;
  /** The geometry of each junction, by node id. Every junction of two or more pipes has one. */
  joints: Map<string, JointPlacement>;
}

/**
 * Collar radii to try, as multiples of the radius at which `n` pipes would just touch on a half turn.
 *
 * Tried tight first. Values below 1 are worth trying because the ends do not land on their targets —
 * a runner whose port is closer than the junction overshoots past it — so the bundle the geometry
 * actually produces is often tighter than the targets suggest.
 */
const COLLAR_SEARCH = [0.55, 0.65, 0.75, 0.85, 1.0, 1.15, 1.35, 1.6, 1.9];

/** Sampled clearance a trial collar must reach, as a multiple of the pipes' combined radii. */
const COLLAR_CLEARANCE = 1.06;

/**
 * How far back over the runner ends the aiming search looks, as a fraction of a pipe diameter.
 *
 * The collar search measures clearance over the zone where the ends bundle together, not just at their
 * tips: they are still converging there, so they sit progressively further out than the tips suggest, and a
 * search that only looked at the tips passed layouts whose runners crossed a couple of centimetres
 * upstream.
 */
const SOCKET_RATIO = 1.3;

/**
 * How far apart the feed *ends* must be, as a multiple of the widest pipe's radius, before the joint is
 * treated as gathering them at their centroid rather than sitting on the widest pipe's end.
 *
 * A merge body exists to gather ends that are spread around a collar. At a T they are not spread: the
 * branch and the pipe it joins finish at the same point, so there is nothing to gather and a body is pure
 * invention — a 40 mm branch onto a 40 mm pipe was producing a 391 mm mouth tapering over 720 mm, which is
 * the funnel-instead-of-a-joint complaint exactly.
 *
 * Measured on the *ends*, not on the socket-zone bundle radius, and that distinction matters: the bundle
 * radius is inflated by a steeply-arriving branch passing through the socket zone well off-axis, so it says
 * 195 mm for a joint whose ends coincide. Where the pipes finish is what decides whether anything needs
 * gathering.
 */
const POINT_JOINT_SPREAD = 0.8;

/**
 * How closely a junction's feeds must agree on a direction, as the length of their mean heading, before the
 * merge simply follows them. Below this they come from opposite sides of the engine.
 */
const OPPOSED_FEEDS = 0.5;

/** Below this angle between them, two pipes meeting from opposite sides meet as a tee. */
const TEE_BELOW = (120 * Math.PI) / 180;

const WORLD_X = new THREE.Vector3(1, 0, 0);

/** The crankshaft's axis, which is square to every bank of a V. */
const CRANK_AXIS = new THREE.Vector3(0, 0, 1);



/** Outlet radius of a duct, m. */
function ductOutletRadius(duct: ExhaustDuct): number {
  const segs = duct.segments;
  return segs.length > 0 ? segmentDiameter(segs[segs.length - 1]!, 1) / 2 : 0.02;
}

/** Sampled centreline of a placed duct, for clearance checks and limb directions. */
interface RunnerSample {
  duct: ExhaustDuct;
  end: THREE.Vector3;
  /** Direction the duct is travelling where it finishes. */
  endDir: THREE.Vector3;
  stations: Array<{ p: THREE.Vector3; r: number }>;
}

function sampleRunner(duct: ExhaustDuct, place: DuctPlacement, stride = 3): RunnerSample {
  const layout = layoutPipe(duct.segments, place.origin, place.heading);
  const stations: Array<{ p: THREE.Vector3; r: number }> = [];
  for (let t = 0; t < layout.stations.length; t += stride) {
    stations.push({ p: layout.stations[t]!.position, r: layout.stations[t]!.radius });
  }
  /**
   * A duct with no segments ends where it starts.
   *
   * `layoutPipe` sweeps per segment, so an empty list yields no joints — and the obvious
   * `joints[joints.length - 1]!` threw. `pipeSpan` and `solveHeading` both guard for it; this was the one
   * caller that assumed one.
   */
  const last = layout.joints[layout.joints.length - 1];
  const dir = layout.jointDirections[layout.jointDirections.length - 1];
  return {
    duct,
    end: last ? last.clone() : place.origin.clone(),
    endDir: dir ? dir.clone().normalize() : place.heading.clone().normalize(),
    stations,
  };
}


/**
 * Where every duct in the graph is drawn, and the pipes that meet at every junction.
 *
 * Nodes are resolved in topological order, because a node's position is the centroid of the ends of
 * whatever feeds it — so a tri-Y's final junction cannot be placed until its two middle ducts are.
 * That is the same ordering constraint the solver has when it sizes inlet areas.
 *
 * A duct's heading comes from one of two places. If it was *drawn*, the stored yaw and pitch turn it
 * off its port or its node and that is that. If it was compiled from a layout, there is no stored
 * heading and it gets aimed — the collar search below picks a direction for each runner so the bundle
 * is as tight as the geometry allows. Drawing therefore makes the aiming machinery redundant for the
 * ducts it touches, which is the intended direction of travel: a drawn route needs no derivation.
 */
export function layoutGraph(ports: ExhaustPort[], graph: ExhaustGraph): ExhaustPlacement {
  const ducts = new Map<string, DuctPlacement>();
  const joints = new Map<string, JointPlacement>();

  // Valve-fed ducts start at a known port. A drawn one is finished here; one that will be aimed gets
  // its port direction for now and is overwritten when its junction is solved.
  for (const duct of graph.ducts) {
    if (duct.from.kind !== 'valve') continue;
    const port = ports[duct.from.cylinder];
    if (!port) continue;
    ducts.set(duct.id, {
      origin: port.position.clone(),
      heading: turnHeading(port.direction, duct.headingYaw, duct.headingPitch),
    });
  }

  const drawn = (duct: ExhaustDuct) =>
    duct.headingYaw !== undefined || duct.headingPitch !== undefined;

  /** Where the one runner feeding `node` ends, if exactly one does and it is already placed. */
  const runnerEndAt = (node: string): THREE.Vector3 | null => {
    const feeds = endsAt(graph, node).filter((e) => e.end === 'outlet' && e.duct.from.kind === 'valve');
    if (feeds.length !== 1) return null;
    const place = ducts.get(feeds[0]!.duct.id);
    return place ? sampleRunner(feeds[0]!.duct, place).end : null;
  };

  const pending = nodeOrder(graph).slice();
  for (let pass = 0; pass < pending.length + 1 && pending.length > 0; pass++) {
    let progress = false;
    for (let idx = 0; idx < pending.length; idx++) {
      const node = pending[idx]!;
      const upstream = endsAt(graph, node)
        .filter((e) => e.end === 'outlet')
        .map((e) => e.duct);
      const downstream = endsAt(graph, node)
        .filter((e) => e.end === 'inlet')
        .map((e) => e.duct);
      // Ready once every feeding duct has somewhere to start from.
      if (!upstream.every((d) => ducts.has(d.id))) continue;

      /**
       * The pipe this junction was made on, if it was made by attaching to one.
       *
       * Then that pipe carries straight on through the junction and nothing is aimed: pipes that snap
       * together stay where they were put. See `ExhaustDuct.continues`.
       */
      const carried = downstream.find((d) => d.continues !== undefined);
      const primary = carried ? upstream.find((d) => d.id === carried.continues) : undefined;

      /**
       * Pipes this node may aim: compiled ones, which neither carry a manifold on nor were drawn, at a
       * junction nothing was snapped onto. A runner from a port, or a downpipe from one bank's manifold
       * to where the banks meet.
       */
      const free = primary
        ? []
        : upstream.filter(
            (d) =>
              !drawn(d) &&
              // A downpipe carries its bank's manifold on, but still has to be cut to where the banks meet.
              (d.continues === undefined || d.role === 'downpipe') &&
              d.segments.length > 0,
          );
      const fixed = upstream.filter((d) => !free.includes(d));

      // The axis the merge points down: the mean of what arrives, however it arrives.
      const centre = new THREE.Vector3();
      const bisector = new THREE.Vector3();
      for (const d of upstream) {
        const place = ducts.get(d.id)!;
        centre.add(place.origin);
        bisector.add(place.heading);
      }
      centre.multiplyScalar(1 / Math.max(upstream.length, 1));
      /**
       * Feeds that arrive from opposite sides have no mean direction worth following.
       *
       * With each bank's exhaust on the outside of the vee, a V-twin's two ports face almost exactly away
       * from each other, so their headings cancel and what is left is the slight downward tilt they share —
       * which aimed the merge straight down through the crank. A real V-twin routes both pipes round one
       * side of the engine to meet, and the one direction square to both banks is the crank axis, so the
       * merge goes that way, tilted by whatever the headings still agree on.
       */
      const agreement = bisector.length() / Math.max(upstream.length, 1);
      const opposed = agreement < OPPOSED_FEEDS;
      if (opposed) {
        bisector.add(CRANK_AXIS.clone().multiplyScalar(upstream.length));
      }
      if (bisector.lengthSq() < 1e-8) bisector.set(1, 0, 0);
      bisector.normalize();

      // Seeded at zero, not at the fallback: seeding at 0.02 made the default a *floor*, so a 34 mm
      // runner was treated as 40 mm and its weld socket came out a fifth too long.
      let rOut = 0;
      for (const d of upstream) rOut = Math.max(rOut, ductOutletRadius(d));
      if (rOut <= 0) rOut = 0.02;
      const socket = rOut * SOCKET_RATIO;

      let samples: RunnerSample[];
      // Two banks' downpipes are cut to meet at one point, whichever way they happen to start out.
      const meetAtPoint = opposed || (free.length > 0 && free.every((d) => d.role === 'downpipe'));
      if (free.length > 1) {
        samples = aimRunners(free, fixed, ducts, centre, bisector, rOut, socket, meetAtPoint);
      } else {
        // Nothing to aim: either everything was drawn, or a single runner goes where it points.
        samples = upstream.map((d) => sampleRunner(d, ducts.get(d.id)!));
      }

      /**
       * Where the joint is, and which way it points.
       *
       * A collector sits at the centre of the ends it gathers. A *branch* does not: it sits on the pipe
       * being teed into, because that pipe has to run through the joint unbroken. Averaging the ends there
       * pulls the joint off the pipe's axis by roughly half a radius — measured as an 11.6 mm step in a pipe
       * of 20 mm radius — and the step shows as a kink where the two halves of the pipe meet.
       *
       * Which case it is comes from how far apart the feeds *finish*. Not from the socket-zone bundle
       * radius: a steeply-arriving branch crosses that zone well off-axis, so it reported 195 mm for a joint
       * between two 40 mm pipes whose ends coincide.
       */
      const centroid = new THREE.Vector3();
      for (const sm of samples) centroid.add(sm.end);
      centroid.multiplyScalar(1 / Math.max(samples.length, 1));

      let endSpread = 0;
      for (const sm of samples) {
        endSpread = Math.max(
          endSpread,
          sm.end.clone().sub(centroid).projectOnPlane(bisector).length(),
        );
      }
      /**
       * Only a joint nothing was aimed at can be a tee. Runners the collar search placed are a collector by
       * construction, and packed tightly enough their ends are close — which, read as a tee, sent a V-twin's
       * collector off along one runner, 24 degrees to that side, instead of down the middle between them.
       */
      const pointJoint = (free.length <= 1 || opposed) && endSpread <= rOut * POINT_JOINT_SPREAD;

      // The widest feed is the pipe that runs through a branch joint.
      let widestFeed = 0;
      let through: RunnerSample | null = null;
      for (const sm of samples) {
        const r = ductOutletRadius(sm.duct);
        if (r > widestFeed) {
          widestFeed = r;
          through = sm;
        }
      }

      /**
       * Pipes from opposite sides meeting at a point: a tee if they converge, a plain meeting if head on.
       *
       * Converging at an acute angle — a V-twin's two runners, about 50 degrees apart — two pipes meeting
       * symmetrically cross each other for several centimetres before the point, and a fitting big enough
       * to hide that is a big ball. So one carries straight on into the outlet and the other joins its side,
       * as a drawn tee does. Arriving nearly head on — two banks' downpipes — they barely overlap, and meet
       * symmetrically with the outlet leaving between them.
       */
      let teeInto: RunnerSample | null = null;
      if (opposed && pointJoint && samples.length === 2) {
        const [a, b] = samples as [RunnerSample, RunnerSample];
        if (a.endDir.angleTo(b.endDir) < TEE_BELOW) teeInto = a;
      }

      // A pipe that was attached to runs straight on from where, and the way, it finished.
      const primarySample = primary ? samples.find((sm) => sm.duct === primary) : undefined;
      const joinAt = primarySample ?? teeInto ?? (pointJoint && !opposed ? through : null);
      const mouth = joinAt ? joinAt.end.clone() : centroid;
      const axis = joinAt ? joinAt.endDir.clone() : bisector.clone();

      /**
       * The joint's limbs: every pipe that meets here, and which way it arrives.
       *
       * No cases and no sizing. A tee is two limbs and an outlet, a 4-into-1 is five limbs, a tri-Y is two
       * joints of three — and `jointMesh` sizes a fitting to whichever it is. The old body
       * needed a mouth radius, a throat radius, a length and a socket, plus a rule to suppress it entirely at
       * a tee, because a surface of revolution could not follow the pipes.
       */
      const limbs: JointLimb[] = samples.map((sm) => ({
        point: sm.end.clone(),
        dir: sm.endDir.clone(),
        radius: ductOutletRadius(sm.duct),
        duct: sm.duct.id,
      }));

      for (const d of downstream) {
        /**
         * A compiled pipe from one junction to another is aimed at where the next one will be.
         *
         * A manifold's links run from one runner's end to the next, and the next junction sits at that
         * runner's end — known already, since runners are placed first. Drawn pipes keep their heading.
         */
        const anchor = !drawn(d) && d.to.kind === 'node' ? runnerEndAt(d.to.node) : null;
        const frame = d.headingFrame === 'world' ? WORLD_X : axis;
        ducts.set(d.id, {
          origin: mouth.clone(),
          heading: anchor
            ? solveHeading(d.segments, mouth, anchor, axis)
            : turnHeading(frame, d.headingYaw, d.headingPitch),
        });
      }

      /**
       * Downstream limbs point *back* into the joint, since `dir` is always into it.
       *
       * A duct leaving the node starts at the joint and heads away, so its limb runs from the joint's centre
       * outward — which is the same capsule with the direction reversed.
       */
      for (const d of downstream) {
        const place = ducts.get(d.id);
        if (!place || d.segments.length === 0) continue;
        limbs.push({
          point: mouth.clone(),
          dir: place.heading.clone().negate(),
          radius: segmentDiameter(d.segments[0]!, 0) / 2,
          duct: d.id,
        });
      }

      if (limbs.length > 1) {
        joints.set(node, { centre: mouth.clone(), axis: axis.clone(), limbs });
      }

      pending.splice(idx, 1);
      idx--;
      progress = true;
    }
    if (!progress) break;
  }

  // Anything a loop left unplaced still needs somewhere to be, or the renderer has nothing to draw.
  for (const duct of graph.ducts) {
    if (ducts.has(duct.id)) continue;
    ducts.set(duct.id, {
      origin: new THREE.Vector3(),
      heading: new THREE.Vector3(1, 0, 0),
    });
  }

  return { ducts, joints };
}

/**
 * Pick a heading for each aimable runner so the bundle is as tight as the geometry allows.
 *
 * The collar is searched for, not computed. A closed formula has to be conservative, because the
 * runner ends do not land where they are aimed: `solveHeading` fixes the *direction* the pipe finishes
 * in, but the distance it covers is its own span, so an end overshoots or falls short depending on how
 * far that port happens to be from the junction. Sizing the collar from the targets left the bundle
 * looser than it needed to be — on a V8 the body came out 223 mm across for 44 mm pipes, a 21-degree
 * taper, which reads as a megaphone.
 *
 * So candidates are tried from tight to loose and the first whose runners actually clear each other is
 * kept. That is the same clearance condition the layout test asserts, so the layout checks its own
 * work.
 */
function aimRunners(
  free: ExhaustDuct[],
  fixed: ExhaustDuct[],
  ducts: Map<string, DuctPlacement>,
  centre: THREE.Vector3,
  bisector: THREE.Vector3,
  rOut: number,
  socket: number,
  meetAtPoint = false,
): RunnerSample[] {
  const n = free.length;
  const origins = free.map((d) => ducts.get(d.id)!.origin);
  const bases = free.map((d) => ducts.get(d.id)!.heading);

  /**
   * How far a runner actually reaches, rather than how long its centreline is.
   *
   * A pipe with corners in it covers less ground than its own length, so aiming with the arc length made every
   * runner overshoot the junction it was pointed at.
   */
  const span = free.reduce((m, d, i) => Math.max(m, pipeSpan(d.segments, bases[i]!)), 0);
  let spread = 0;
  for (const o of origins) spread = Math.max(spread, o.distanceTo(centre));
  const reach = Math.sqrt(Math.max(span * span - spread * spread, 0));
  const junction = centre.clone().addScaledVector(bisector, reach);

  /**
   * A frame for the collar plane, and the runners' true spatial order along it.
   *
   * Ordering by cylinder index is wrong, and not harmlessly so. A crossplane V8's banks follow the
   * firing order, so one collector's members are cylinders 1, 4, 6, 7 — sitting at z = -0.194, +0.065,
   * +0.194, -0.065. Handing them collar slots in index order sends runner 4 past runner 7 to get to its
   * place, and they interpenetrate at 41% of their combined radii however wide the collar is made; the
   * other bank happens to be ordered already, which is exactly the sort of asymmetry that makes this
   * look like a sizing problem.
   */
  const offsets = origins.map((o) => o.clone().sub(centre).projectOnPlane(bisector));
  const e1 = new THREE.Vector3();
  let widest = 0;
  for (const o of offsets) {
    const len = o.length();
    if (len > widest) {
      widest = len;
      e1.copy(o);
    }
  }
  if (e1.lengthSq() < 1e-10) e1.copy(new THREE.Vector3(0, 0, 1)).projectOnPlane(bisector);
  if (e1.lengthSq() < 1e-10) e1.copy(new THREE.Vector3(0, 1, 0)).projectOnPlane(bisector);
  e1.normalize();
  const e2 = new THREE.Vector3().crossVectors(bisector, e1).normalize();

  const order = free
    .map((_, k) => ({ k, along: offsets[k]!.dot(e1) }))
    .sort((a, b) => a.along - b.along)
    .map((m) => m.k);

  const fixedSamples = fixed.map((f) => sampleRunner(f, ducts.get(f.id)!));

  /**
   * Ends and headings for a trial collar, plus how close the runners come to touching *outside* the
   * weld.
   *
   * Interpenetration inside the weld is not a defect, it is how the part is made: a fabricated
   * collector has the pipe walls cut away where they meet, and the body covers the join. So what has to
   * be checked is that the runners clear each other everywhere the body does *not* enclose them.
   * Requiring clearance along the whole length is what forced the bundle out to two hundred millimetres
   * for forty-four millimetre pipes.
   */
  const trial = (collar: number, wrap: number) => {
    const placed: DuctPlacement[] = [];
    const samples: RunnerSample[] = [];
    for (let slot = 0; slot < n; slot++) {
      const k = order[slot]!;
      /**
       * Slots follow the runners' order across the collar, starting on the side the first one comes from.
       *
       * The half turn does this by construction. The ring was a quarter turn out — its first slot sat
       * square to the direction the runners are ordered along — which for two runners arriving from
       * opposite sides of a V-twin stacked them one above the other, so each had to pass the other to
       * reach its slot, and they clipped at 94% of their combined radii.
       */
      const phase = wrap > Math.PI * 1.5 ? -Math.PI / 2 : 0;
      const theta = n > 1 ? wrap * (1 - (slot + 0.5) / n) + phase : Math.PI / 2;
      const target = junction
        .clone()
        .addScaledVector(e1, collar * Math.cos(theta))
        .addScaledVector(e2, collar * Math.sin(theta));
      const heading = solveHeading(free[k]!.segments, origins[k]!, target, bases[k]!);
      placed[k] = { origin: origins[k]!.clone(), heading };
      samples[k] = sampleRunner(free[k]!, placed[k]!);
    }

    const all = [...samples, ...fixedSamples];
    const mouthAxial =
      all.reduce((acc, sm) => acc + sm.end.dot(bisector), 0) / Math.max(all.length, 1);
    const weldFrom = mouthAxial - socket;

    let worst = Infinity;
    for (let a = 0; a < all.length; a++) {
      for (let b = a + 1; b < all.length; b++) {
        for (const x of all[a]!.stations) {
          if (x.p.dot(bisector) >= weldFrom) continue;
          for (const y of all[b]!.stations) {
            if (y.p.dot(bisector) >= weldFrom) continue;
            const need = x.r + y.r;
            if (need <= 0) continue;
            const ratio = x.p.distanceTo(y.p) / need;
            if (ratio < worst) worst = ratio;
          }
        }
      }
    }
    return { placed, samples: all, clearance: worst };
  };

  /**
   * Candidates tight to loose: the closed ring first, then the half turn.
   *
   * The ring is the packing a fabricator uses and about half the radius of the arc, so it is preferred
   * when it works. For runners from ports in a line it usually does not: straight runners can only
   * reach targets that stay monotonic across the collar, and a ring is not monotonic, so two of them
   * always cross. Measured that way they interpenetrated at 67% of their combined radii even inside the
   * weld exemption, which is why the arc is still what most engines get.
   */
  const ringCollar = rOut / Math.sin(Math.PI / n);
  const arcCollar = rOut / Math.sin(Math.PI / (2 * n));
  const candidates: Array<[number, number]> = [];
  for (const scale of COLLAR_SEARCH) candidates.push([ringCollar * scale, 2 * Math.PI]);
  for (const scale of COLLAR_SEARCH) candidates.push([arcCollar * scale, Math.PI]);

  /**
   * Pipes from opposite sides of a V always meet at a point.
   *
   * They cannot collide on the way — they come from opposite sides — and where they overlap at the end is
   * the joint itself, drawn as a tee or a fitting. A collar here only spreads their ends apart around the
   * junction, which is what made a V-twin's 2-into-1 a big cone.
   */
  if (meetAtPoint) {
    /**
     * Two pipes of different lengths still meet exactly, where they can.
     *
     * One point for both only works if they are the same length; shorten one — delete a segment of a
     * V-twin's runner — and the shorter fell short, leaving a gap the fitting grew to 20 cm to span.
     * The point both can reach is on the circle where a sphere of each one's reach about its own start
     * meets the other's, and of that circle the point furthest along the way the merge heads.
     */
    if (n === 2) {
      const [a, b] = origins as [THREE.Vector3, THREE.Vector3];
      const ra = pipeSpan(free[0]!.segments, bases[0]!);
      const rb = pipeSpan(free[1]!.segments, bases[1]!);
      const ab = b.clone().sub(a);
      const d = ab.length();
      if (d > 1e-9 && ra + rb >= d && Math.abs(ra - rb) <= d) {
        const u = ab.clone().divideScalar(d);
        const along = (ra * ra - rb * rb + d * d) / (2 * d);
        const off = Math.sqrt(Math.max(ra * ra - along * along, 0));
        const side = bisector.clone().projectOnPlane(u);
        if (side.lengthSq() < 1e-10) side.copy(CRANK_AXIS).projectOnPlane(u);
        side.normalize();
        const meet = a.clone().addScaledVector(u, along).addScaledVector(side, off);
        const samples = free.map((duct, k) => {
          const heading = solveHeading(duct.segments, origins[k]!, meet, bases[k]!);
          const place = { origin: origins[k]!.clone(), heading };
          ducts.set(duct.id, place);
          return sampleRunner(duct, place);
        });
        return [...samples, ...fixedSamples];
      }
    }
    const point = trial(0, Math.PI);
    free.forEach((d, k) => ducts.set(d.id, point.placed[k]!));
    return point.samples;
  }

  let chosen = trial(arcCollar * 1.9, Math.PI);
  for (const [collar, wrap] of candidates) {
    const t = trial(collar, wrap);
    if (t.clearance >= COLLAR_CLEARANCE) {
      chosen = t;
      break;
    }
  }
  free.forEach((d, k) => ducts.set(d.id, chosen.placed[k]!));
  return chosen.samples;
}

/** Every duct's directions in a layout, for rejoining pipes without swinging them. */
export function ductDirections(graph: ExhaustGraph, placement: ExhaustPlacement): DuctDirections {
  return (id) => {
    const duct = graph.ducts.find((d) => d.id === id);
    const place = placement.ducts.get(id);
    if (!duct || !place) return undefined;
    const swept = layoutPipe(duct.segments, place.origin, place.heading);
    const end = swept.jointDirections[swept.jointDirections.length - 1] ?? place.heading;
    const first = swept.stations[0]?.direction ?? place.heading;
    return { end: [end.x, end.y, end.z], first: [first.x, first.y, first.z] };
  };
}

/** How far a pipe's end may be from its junction and still count as joined to it, m. */
const JOIN_TOLERANCE = 0.005;

/**
 * Whether every pipe running into `node` actually reaches it.
 *
 * Pipes are straight tube that snaps together, so a junction whose pipes do not meet is not a joint but
 * a gap — the fitting would have to grow to bridge it.
 */
export function pipesMeetAt(graph: ExhaustGraph, placement: ExhaustPlacement, node: string): boolean {
  const joint = placement.joints.get(node);
  if (!joint) return true;
  return graph.ducts.every((d) => {
    if (d.to.kind !== 'node' || d.to.node !== node) return true;
    const place = placement.ducts.get(d.id);
    if (!place || d.segments.length === 0) return false;
    const end = layoutPipe(d.segments, place.origin, place.heading).joints.at(-1)!;
    return end.distanceTo(joint.centre) <= JOIN_TOLERANCE;
  });
}

/**
 * Fix every pipe where the layout currently has it, so an edit moves only what it edits.
 *
 * A compiled exhaust is partly *worked out*: runners and downpipes are aimed to meet, and a junction's
 * direction follows whatever arrives at it. That is right until the user starts changing things, after
 * which re-aiming swung pipes they had not touched — deleting one runner of a V-twin turned the other
 * and sent the collector 70 cm off, and on an 8-into-1 a downpipe no longer aimed missed its junction by
 * 33 cm. So on the first edit every pipe's direction is stored as it stands: a runner's relative to its
 * port, which moves with the engine, and a pipe leaving a junction in world terms, since the junction's
 * own direction is worked out again each time. Pipes already drawn keep what they have.
 */
export function freezeHeadings(graph: ExhaustGraph, placement: ExhaustPlacement, ports: ExhaustPort[]): void {
  for (const duct of graph.ducts) {
    if (duct.headingYaw !== undefined || duct.headingPitch !== undefined) continue;
    const place = placement.ducts.get(duct.id);
    if (!place) continue;
    const base = duct.from.kind === 'valve' ? ports[duct.from.cylinder]?.direction : WORLD_X;
    if (!base) continue;
    const turn = turnBetween(base, place.heading);
    duct.headingYaw = turn.yaw;
    duct.headingPitch = turn.pitch;
    if (duct.from.kind === 'node') duct.headingFrame = 'world';
  }
}
