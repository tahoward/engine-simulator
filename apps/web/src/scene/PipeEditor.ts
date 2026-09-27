/**
 * Direct manipulation of the exhaust in 3D.
 *
 * The selected segment has a triad (`Triad.ts`): its arrows and squares move the segment's end along an
 * axis or in a plane, the length and the corner following, and its rings turn the segment about where it
 * starts. A ring around each joint sets the diameter there: drag it outward to open the pipe up, inward to
 * choke it down. A selected turbo has a triad of its own, which moves and turns it.
 *
 * Both write through the same mutation helpers the numeric panel uses, so the two
 * editors cannot drift apart — there is exactly one `PipeSegment[]`.
 *
 * Turning is free acoustically: the 1D model integrates area against *axial*
 * distance, so folding a long pipe to fit the viewport does not change the note.
 * That is a real property of the physics, not a simplification.
 */

import * as THREE from 'three';
import { makeSegment, type PipeSegment, segmentDiameter } from '../model/spec.js';
import type { Vec3 } from '../model/geometry.js';
import {
  IDENTITY,
  ensureTurboOutlet,
  fittedBend,
  quatFromAxisAngle,
  quatMultiply,
  quatNormalise,
  seatTurbo,
  type TurboSize,
} from '../model/turbo.js';
import type { Quat } from '../model/exhaustGraph.js';
import {
  joinDuctEnd,
  newDuctId,
  splitDuctAt,
  type DuctSink,
  type DuctSource,
  type ExhaustDuct,
  type ExhaustGraph,
} from '../model/exhaustGraph.js';
import { layoutPipe, type PipeMesh } from './PipeMesh.js';
import { TurboMesh } from './TurboMesh.js';
import {
  AXIS_COLOURS,
  MOVE_STEP,
  TURN_STEP_DEG as TRIAD_TURN_DEG,
  Triad,
  angleStep,
  axisOffset,
  planePoint,
  frameAlong,
  ringAngle,
  snapTo,
  snapTurnToEngine,
  type RingFrame,
  type TriadHandle,
} from './Triad.js';
import type { DuctPlacement, ExhaustPlacement, ExhaustPort } from './exhaustLayout.js';
import {
  MIN_DRAW_LENGTH,
  bendAnchor,
  collectSnapTargets,
  swingPipe,
  continuingDiameter,
  fitCurve,
  fitSegment,
  headingOffsetTo,
  nearestSnap,
  quantiseLength,
  quantiseTurn,
  routeTip,
  snapToEngine,
  type SnapTarget,
} from './drawing.js';

const MIN_RADIUS = 0.006;
const MAX_RADIUS = 0.22;

/** Live geometry edits are cheap to draw but force a waveguide rebuild, so throttle those. */
const AUDIO_COMMIT_MS = 200;

/** A diameter handle: the ring at a joint, or at the inlet. */
type HandleKind = 'ring' | 'inlet';

interface HandleData {
  kind: HandleKind;
  segment: number;
}

/** Size of the selected segment's triad, m. */
const PIPE_TRIAD_SIZE = 0.11;

/**
 * A triad being dragged, on a pipe segment or a turbo.
 *
 * Everything is measured from where the drag began, so a move or a turn is where the pointer is now rather
 * than the sum of every frame's step.
 */
interface TriadDrag {
  on: 'pipe' | 'turbo';
  handle: TriadHandle;
  /** The part's axis the handle works along or about, and for a ring its frame, as they were at the start. */
  axis: THREE.Vector3;
  ring: RingFrame;
  /** The pointer's offset along the axis, or its point in the plane, when the drag began. */
  grabOffset: number;
  grabPoint: THREE.Vector3;
  /** Where the moving handles and the rings were when the drag began. */
  moveOrigin: THREE.Vector3;
  rotateOrigin: THREE.Vector3;
  /** For a ring: the pointer's angle at the last move, and how far it has turned since the drag began. */
  lastAngle: number;
  turned: number;
  /**
   * A pipe's: which, the direction its heading is stored off, how many of its segments were drawn (the
   * rest being the bend fitted to what it joins), and each drawn segment's direction when the drag began.
   */
  ductId?: string;
  base?: THREE.Vector3;
  drawn?: number;
  dirs0?: THREE.Vector3[];
  /** What a pipe's arrows move: the junction it starts from, or the turbo, from where it was. */
  startNode?: string;
  startAxis?: Vec3;
  startTurbo?: { id: string; position: Vec3; rotation: Quat };
  /** The last place a move was sent to, for the final commit. */
  lastPosition?: Vec3;
  /** A turbo's: which, and its rotation when the drag began. */
  turbo?: string;
  rotation0?: Quat;
  lastCommit: number;
  moved: boolean;
}
/** Something in the exhaust a click picked out, outside draw mode. */
export type ScenePick =
  | { kind: 'segment'; duct: string; segment: number }
  | { kind: 'joint'; node: string }
  | { kind: 'turbo'; turbo: string };

/** Where a turbo being placed goes, and the open pipe end it was put on, if any. */
export interface TurboPlacement {
  position: Vec3;
  rotation: Quat;
  attach?: string;
}

export interface PipeEditorCallbacks {
  /** Geometry changed. `commit` is false for intermediate frames of a drag. */
  onChange: (commit: boolean) => void;
  /** A handle of the duct being edited was grabbed, selecting its segment. */
  onSelect: (index: number | null) => void;
  /**
   * A click landed on a pipe or a junction — any of them, not only the duct being edited — or on nothing.
   * The owner decides what selecting it means, since switching ducts is its business, not the editor's.
   */
  onPick?: (pick: ScenePick | null) => void;
  /** A route was started or finished, so the UI can show whether drawing is in progress. */
  onDrawing?: (active: boolean) => void;
  /** Where the next segment is aimed, in words — "up, 250 mm" — or `null` when nothing is. */
  onAim?: (aim: string | null) => void;
  /** A turbo was put down. */
  onPlaceTurbo?: (placement: TurboPlacement) => void;
  /** Placing turbos was started or ended from the view, by Escape. */
  onPlacing?: (active: boolean) => void;
  /**
   * The junction at `node` was moved to here by the triad of a pipe starting from it; `axis` is the way it
   * points, for the first time it is moved. `commit` is false for intermediate frames of a drag.
   */
  onMoveJunction?: (node: string, position: Vec3, axis: Vec3, commit: boolean) => void;
  /** A turbo was moved or turned to here by its triad. `commit` is false for intermediate frames of a drag. */
  onMoveTurbo?: (turbo: string, position: Vec3, rotation: Quat, commit: boolean) => void;
}

/**
 * How a free click is tidied while drawing.
 *
 * `engine` locks the segment to the engine's axes and the diagonals between them, which is what makes a
 * route readable in a perspective view; `turn` rounds the bend off the pipe being left instead; `free`
 * takes the point as clicked.
 */
type DrawSnap = 'engine' | 'turn' | 'free';

function drawSnapOf(e: { shiftKey: boolean; altKey: boolean }): DrawSnap {
  if (e.shiftKey) return 'free';
  return e.altKey ? 'turn' : 'engine';
}

// A locked segment is drawn in its engine axis's colour (`Triad.ts`), so it says which way it runs. A
// diagonal mixes the two it lies between.
/** A segment carrying straight on in a direction that is none of the engine's. */
const STRAIGHT_ON_COLOUR = new THREE.Color(0xffffff);
/** A segment not locked to a direction. */
const PREVIEW_COLOUR = 0x8cff9e;
/** How far the guide along a locked direction runs on past the segment's end, m. */
const GUIDE_REACH = 0.6;

function axisColour(dir: THREE.Vector3): THREE.Color {
  const c = new THREE.Color(0, 0, 0);
  const weights = [Math.abs(dir.x), Math.abs(dir.y), Math.abs(dir.z)];
  const total = weights[0]! + weights[1]! + weights[2]!;
  weights.forEach((w, i) => c.add(AXIS_COLOURS[i]!.clone().multiplyScalar(w / total)));
  return c;
}

/** What the editor needs to know about the scene to draw into it. */
export interface DrawContext {
  graph: ExhaustGraph;
  placement: ExhaustPlacement;
  ports: ExhaustPort[];
  /** One mesh per duct, in `graph.ducts` order, for picking a duct's surface. */
  meshes: PipeMesh[];
  /** Each junction's mesh, by node, for picking a junction. */
  joints: Array<{ node: string; target: THREE.Object3D | null }>;
  /** Each turbo's mesh, by turbo id, for picking and dragging one. */
  turbos: Array<{ turbo: string; target: THREE.Object3D }>;
}



/** How close, in pixels, the pointer has to be for a snap target to take. */
const SNAP_PIXELS = 14;
/** Turn quantisation while drawing with alt held, degrees. */
const TURN_STEP_DEG = 15;
/** Length quantisation, m. */
const LENGTH_GRID_M = 0.025;

export class PipeEditor {
  readonly group = new THREE.Group();

  private readonly raycaster = new THREE.Raycaster();
  private readonly pointer = new THREE.Vector2();
  private readonly handles: THREE.Mesh[] = [];
  /** The selected segment's triad, and the selected turbo's. */
  private readonly pipeTriad = new Triad(PIPE_TRIAD_SIZE);
  private turboTriad = new Triad();
  private triadDrag: TriadDrag | null = null;

  private selected: number | null = null;
  private hovered: THREE.Mesh | null = null;

  /** A diameter ring being dragged. */
  private drag: {
    handle: THREE.Mesh;
    data: HandleData;
    plane: THREE.Plane;
    /** A point on, and the direction of, the pipe axis at the dragged joint. */
    axisPoint: THREE.Vector3;
    axisDir: THREE.Vector3;
    lastCommit: number;
  } | null = null;

  private origin = new THREE.Vector3();
  private heading = new THREE.Vector3(1, 0, 0);

  private drawMode = false;
  private context: DrawContext | null = null;

  /** Placing a turbo: the see-through one following the pointer, and where it is. */
  private placeMode = false;
  private readonly ghost = new TurboMesh(true);
  private ghostAt: TurboPlacement = { position: [0, 0, 0], rotation: [...IDENTITY] };
  /** Turbos are drawn this size, and put down at this height unless snapped to a pipe. Set by the owner. */
  private size: TurboSize = { scroll: 0.07, depth: 0.06, outletDia: 0.058 };
  turboHeight = 0;
  /** The bore a turbo's outlet is given when a pipe is first drawn into it. Set by the owner. */
  turboOutletDia = 0.058;
  /** The turbo the owner has selected, which the turbo triad is on. */
  private selectedTurbo: string | null = null;
  /**
   * The route in progress.
   *
   * The duct is added to the graph as soon as drawing starts, rather than being assembled and inserted at
   * the end. That keeps one source of truth — the renderer draws it, the solver hears it, and the panel
   * lists it, all while it is still being drawn — instead of a second, parallel representation that has to
   * be kept in step with the first.
   */
  private route: {
    ductId: string;
    place: DuctPlacement;
    /**
     * The graph's ducts as they were before the route began, so abandoning it puts everything back.
     *
     * A snapshot rather than a record of what to undo, because starting a route can change the graph
     * three different ways — replacing a port's pipe, splitting a pipe for a branch, or extending one — and
     * a snapshot undoes all of them the same way. There is no undo in the app, so Escape has to be exact.
     */
    snapshot: string;
    /** Segments the duct already had, when the route continues an existing pipe rather than a new one. */
    base: number;
  } | null = null;
  /** The target the preview is currently offering, for anything that wants to describe it. */
  snapped: SnapTarget | null = null;
  private readonly preview: THREE.Line;
  private readonly previewGeom = new THREE.BufferGeometry();
  private readonly previewMat = new THREE.LineDashedMaterial({
    color: PREVIEW_COLOUR,
    dashSize: 0.02,
    gapSize: 0.012,
  });
  /** A faint line through the tip along the locked direction, so its orientation reads in 3D. */
  private readonly guide: THREE.Line;
  private readonly guideGeom = new THREE.BufferGeometry();
  private readonly guideMat = new THREE.LineBasicMaterial({ transparent: true, opacity: 0.35 });
  private readonly marker: THREE.Mesh;

  private readonly matHover = new THREE.MeshBasicMaterial({ color: 0xffd166 });
  private readonly matSelected = new THREE.MeshBasicMaterial({ color: 0x8cff9e });
  private readonly matRing = new THREE.MeshBasicMaterial({
    color: 0x4fd1ff,
    transparent: true,
    opacity: 0.6,
  });

  constructor(
    private readonly dom: HTMLElement,
    private readonly camera: THREE.Camera,
    private readonly controls: { enabled: boolean },
    private pipeMesh: PipeMesh,
    private pipe: PipeSegment[],
    private readonly cb: PipeEditorCallbacks,
  ) {
    dom.addEventListener('pointerdown', this.onPointerDown);
    dom.addEventListener('pointermove', this.onPointerMove);
    window.addEventListener('pointerup', this.onPointerUp);
    window.addEventListener('keydown', this.onKeyDown);
    // Right-click finishes a route, so the browser menu must not appear over it.
    dom.addEventListener('contextmenu', this.onContextMenu);

    this.previewGeom.setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
    this.preview = new THREE.Line(this.previewGeom, this.previewMat);
    this.preview.visible = false;
    this.preview.renderOrder = 12;
    this.group.add(this.preview);

    this.guideGeom.setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
    this.guide = new THREE.Line(this.guideGeom, this.guideMat);
    this.guide.visible = false;
    this.guide.renderOrder = 11;
    this.group.add(this.guide);

    this.marker = new THREE.Mesh(
      new THREE.SphereGeometry(0.016, 16, 12),
      new THREE.MeshBasicMaterial({ color: 0xffd166, transparent: true, opacity: 0.85 }),
    );
    this.marker.visible = false;
    this.marker.renderOrder = 13;
    this.group.add(this.marker);

    this.ghost.group.visible = false;
    this.group.add(this.ghost.group);
    this.group.add(this.pipeTriad.group, this.turboTriad.group);
  }

  get turboSize(): TurboSize {
    return this.size;
  }

  set turboSize(size: TurboSize) {
    const changed = size.scroll !== this.size.scroll;
    this.size = size;
    if (changed) this.sizeTurboTriad(size);
  }

  /** Show the turbo triad on `turbo`, or with `null` on none. Called by the owner on select and rebuild. */
  setSelectedTurbo(turbo: string | null): void {
    this.selectedTurbo = turbo;
    this.placeTurboTriad();
  }

  /** Put the turbo triad on the selected turbo, sized to it, or hide it. */
  private placeTurboTriad(): void {
    const mount = this.context?.graph.turbos?.find((t) => t.id === this.selectedTurbo);
    const show = !!mount?.position && !this.drawMode && !this.placeMode;
    if (show) {
      const at = new THREE.Vector3(...mount!.position!);
      this.turboTriad.setMoveOrigin(at);
      this.turboTriad.setRotateOrigin(at);
      // Turned with the turbo: red along its shaft.
      this.turboTriad.setOrientation(new THREE.Quaternion(...mount!.rotation));
    }
    this.turboTriad.setVisible(show);
  }

  /** Resize the turbo triad for turbos of `size`. */
  private sizeTurboTriad(size: TurboSize): void {
    this.group.remove(this.turboTriad.group);
    this.turboTriad.dispose();
    this.turboTriad = new Triad(size.scroll * 1.9);
    this.group.add(this.turboTriad.group);
    this.placeTurboTriad();
  }

  // -------------------------------------------------------------------------
  // Placing turbos
  // -------------------------------------------------------------------------

  /** Start or stop placing a turbo. Stops drawing, since the two share the pointer. */
  setPlaceMode(on: boolean): void {
    if (on && this.drawMode) this.setDrawMode(false);
    this.placeMode = on;
    this.ghost.rebuild(this.size);
    this.ghost.group.visible = false;
    this.applyHandleVisibility();
    this.placeTurboTriad();
  }

  get placing(): boolean {
    return this.placeMode;
  }

  /**
   * Follow the pointer with the turbo being placed: on the level it is put down at, or, near an open pipe
   * end, with its inlet flange on that end and turned to take the pipe.
   */
  private updateGhost(): void {
    const ctx = this.context;
    if (!ctx) return;
    const rect = this.dom.getBoundingClientRect();
    const ends = collectSnapTargets(ctx.graph, ctx.placement, ctx.ports).filter((t) => t.kind === 'ductEnd');
    const end = nearestSnap(ends, this.pointer, this.camera, SNAP_PIXELS, {
      width: rect.width,
      height: rect.height,
    });
    if (end && end.kind === 'ductEnd') {
      const duct = ctx.graph.ducts.find((d) => d.id === end.duct);
      const place = ctx.placement.ducts.get(end.duct);
      if (duct && place) {
        const dir = layoutPipe(duct.segments, place.origin, place.heading).jointDirections.at(-1) ?? place.heading;
        const mount = { id: '', node: '', position: null as Vec3 | null, rotation: [...IDENTITY] as Quat };
        seatTurbo(mount, [end.point.x, end.point.y, end.point.z], [dir.x, dir.y, dir.z], this.size);
        this.ghostAt = { position: mount.position!, rotation: mount.rotation, attach: duct.id };
        this.showGhost();
        return;
      }
    }
    const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -this.turboHeight);
    const point = new THREE.Vector3();
    if (!this.raycaster.ray.intersectPlane(plane, point)) {
      this.ghost.group.visible = false;
      return;
    }
    this.ghostAt = { position: [point.x, point.y, point.z], rotation: [...IDENTITY] };
    this.showGhost();
  }

  private showGhost(): void {
    this.ghost.place(this.ghostAt.position, this.ghostAt.rotation);
    this.ghost.group.visible = true;
  }

  // -------------------------------------------------------------------------
  // Draw mode
  // -------------------------------------------------------------------------

  /** Everything the editor needs to draw into the current scene. Refreshed on every rebuild. */
  setDrawContext(context: DrawContext): void {
    this.context = context;
    this.placeTurboTriad();
    if (!this.route) return;
    /**
     * Follow the duct through the rebuild, and only abandon the route if the duct is *gone from the
     * graph* — not merely missing a placement.
     *
     * `cancelRoute` publishes a change, which rebuilds, which lands back here; keying the decision on the
     * graph rather than on the placement keeps that from being able to loop, and the graph is the thing
     * that actually says whether the route still exists.
     */
    if (!context.graph.ducts.some((d) => d.id === this.route!.ductId)) {
      this.route = null;
      this.hidePreview();
      this.cb.onDrawing?.(false);
      return;
    }
    const place = context.placement.ducts.get(this.route.ductId);
    if (place) this.route.place = place;
  }

  setDrawMode(on: boolean): void {
    this.drawMode = on;
    if (!on) this.cancelRoute();
    this.group.visible = true;
    this.applyHandleVisibility();
    this.placeTurboTriad();
  }

  /** Start a route from a junction, as clicking it in draw mode would. For the panel's button. */
  startAtJunction(node: string): void {
    const ctx = this.context;
    const joint = ctx?.placement.joints.get(node);
    if (!ctx || this.route) return;
    const point = joint?.centre ?? ctx.placement.ducts.get(
      ctx.graph.ducts.find((d) => d.from.kind === 'node' && d.from.node === node)?.id ?? '',
    )?.origin;
    if (!point) return;
    this.beginRoute({ kind: 'node', point: point.clone(), node });
  }

  get drawing(): boolean {
    return this.route !== null;
  }

  /** Whether a handle is being dragged, a diameter ring or a triad's: an edit that has not settled yet. */
  get dragging(): boolean {
    return this.drag !== null || this.triadDrag !== null;
  }

  /**
   * Abandon a route, removing the duct it had started.
   *
   * A part-drawn duct is left in the graph while it is being drawn, so abandoning has to take it out
   * again — otherwise Escape would leave a stub hanging off a port and, worse, a cylinder with a pipe
   * going nowhere, which `validateGraph` rejects and the solver refuses to build.
   */
  private cancelRoute(): void {
    const ctx = this.context;
    if (this.route && ctx) {
      ctx.graph.ducts = JSON.parse(this.route.snapshot) as ExhaustDuct[];
      this.route = null;
      this.hidePreview();
      this.cb.onDrawing?.(false);
      this.cb.onChange(true);
      return;
    }
    this.route = null;
    this.hidePreview();
    this.cb.onDrawing?.(false);
  }

  private hidePreview(): void {
    this.preview.visible = false;
    this.guide.visible = false;
    this.marker.visible = false;
    this.snapped = null;
    this.cb.onAim?.(null);
  }

  /**
   * Where a segment from `tip` towards `target` actually ends, tidied as `snap` says.
   *
   * Only a free point is tidied: one that connects to something has to land on it. `dir` is set when the
   * segment is locked to an engine direction.
   */
  private aim(
    tip: { point: THREE.Vector3; dir: THREE.Vector3 },
    target: SnapTarget,
    snap: DrawSnap,
  ): { point: THREE.Vector3; dir?: THREE.Vector3; name?: string } {
    const point = target.point.clone();
    if (target.kind !== 'free' || snap === 'free') return { point };
    if (snap === 'engine') {
      const rect = this.dom.getBoundingClientRect();
      const locked = snapToEngine(
        tip.point,
        tip.dir,
        this.raycaster.ray,
        this.pointer,
        this.camera,
        { width: rect.width, height: rect.height },
        LENGTH_GRID_M,
      );
      if (locked) return locked;
    }
    // Quantise the turn and the length so a hand-drawn route comes out tidy.
    const dir = quantiseTurn(point.clone().sub(tip.point), tip.dir, TURN_STEP_DEG);
    const len = quantiseLength(point.distanceTo(tip.point), LENGTH_GRID_M);
    return { point: tip.point.clone().addScaledVector(dir, len) };
  }

  /** Handles are a nuisance while drawing: they sit exactly where the route is being aimed. */
  private applyHandleVisibility(): void {
    const hide = this.drawMode || this.placeMode;
    for (const h of this.handles) h.visible = !hide;
    this.pipeTriad.setVisible(!hide && this.pipeTriadAt !== null);
  }

  /** Where the selected pipe's triad goes: where the pipe starts, which it turns about. */
  private pipeTriadAt: { start: THREE.Vector3 } | null = null;

  private onContextMenu = (e: MouseEvent): void => {
    if (this.drawMode) e.preventDefault();
  };

  private onKeyDown = (e: KeyboardEvent): void => {
    if (this.placeMode && e.key === 'Escape') {
      this.setPlaceMode(false);
      this.cb.onPlacing?.(false);
      e.preventDefault();
      return;
    }
    if (!this.drawMode || !this.route) return;
    if (e.key === 'Escape') {
      this.cancelRoute();
      e.preventDefault();
    } else if (e.key === 'Enter') {
      this.finishRoute({ kind: 'mouth' });
      e.preventDefault();
    }
  };

  /** Where a free click lands: on a plane through the route's tip, facing the camera. */
  private freePoint(tip: THREE.Vector3): THREE.Vector3 | null {
    const normal = this.camera.getWorldDirection(new THREE.Vector3()).negate();
    const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(normal, tip);
    const point = new THREE.Vector3();
    return this.raycaster.ray.intersectPlane(plane, point) ? point : null;
  }

  /** The snap target under the pointer, including duct surfaces, or a free point. */
  private resolveSnap(tip: THREE.Vector3): SnapTarget | null {
    const ctx = this.context;
    if (!ctx) return null;
    const rect = this.dom.getBoundingClientRect();
    const viewport = { width: rect.width, height: rect.height };

    const point = nearestSnap(
      collectSnapTargets(ctx.graph, ctx.placement, ctx.ports).filter(
        (t) => !(this.route && t.kind === 'ductEnd' && t.duct === this.route.ductId),
      ),
      this.pointer,
      this.camera,
      SNAP_PIXELS,
      viewport,
    );
    if (point) return point;

    // Nothing within reach, so try the tubes themselves: a hit on one is a T.
    for (let i = 0; i < ctx.meshes.length; i++) {
      const duct = ctx.graph.ducts[i];
      if (!duct || duct.id === this.route?.ductId) continue;
      const target = ctx.meshes[i]!.pickTarget;
      if (!target) continue;
      const hit = this.raycaster.intersectObject(target, false)[0];
      if (!hit) continue;
      const st = ctx.meshes[i]!.stationAt(hit.point);
      if (!st) continue;
      // A bend into a turbo is fitted, not drawn, so nothing branches off it.
      if (fittedBend(ctx.graph, duct) === st.segment) continue;
      /**
       * Aim at the pipe's *axis*, not the skin the ray hit.
       *
       * A tee's joint sits on the centreline of the pipe being teed into, because that pipe has to run
       * through it unbroken. Stopping the branch on the surface leaves its end a whole radius short of the
       * joint, which both leaves a gap and — because the joint is classified by how far apart the feeds
       * finish — makes a tee look half like a collector. Running the branch in to the centreline is also how
       * one is really cut: the pipe it joins covers the overlap.
       */
      const layout = ctx.meshes[i]!.pipeLayout;
      const station = layout
        ? layout.stations.reduce((best, s) => (Math.abs(s.x - st.x) < Math.abs(best.x - st.x) ? s : best))
        : null;
      return {
        kind: 'ductSurface',
        point: (station?.position ?? hit.point).clone(),
        duct: duct.id,
        x: st.x,
        ...(station ? { dir: station.direction.clone() } : {}),
      };
    }

    const free = this.freePoint(tip);
    return free ? { kind: 'free', point: free } : null;
  }

  /** Whether a route can start from this target. */
  private static startable(target: SnapTarget | null): boolean {
    return !!target && target.kind !== 'free' && target.kind !== 'turboInlet';
  }

  /**
   * Start a route.
   *
   * From a port it is a new runner, replacing whatever the cylinder had. From a junction it is a new pipe
   * leaving it. From the open end of a pipe it *continues that pipe* — the same duct, with segments added —
   * since a pipe carrying on is one pipe, not two joined end to end. From the side of a pipe it is a
   * branch: the pipe is split there, as a T is when something is drawn *into* its side, and the new pipe
   * leaves the junction that makes.
   */
  private beginRoute(target: SnapTarget): void {
    const ctx = this.context;
    if (!ctx) return;
    const snapshot = JSON.stringify(ctx.graph.ducts);

    if (target.kind === 'ductEnd') {
      const duct = ctx.graph.ducts.find((d) => d.id === target.duct);
      const place = ctx.placement.ducts.get(target.duct);
      if (!duct || !place) return;
      this.route = { ductId: duct.id, place, snapshot, base: duct.segments.length };
      this.cb.onDrawing?.(true);
      return;
    }

    let from: DuctSource;
    let place: DuctPlacement;
    if (target.kind === 'port') {
      from = { kind: 'valve', cylinder: target.cylinder };
      place = { origin: target.point.clone(), heading: target.dir.clone() };
      /**
       * Drawing from a port replaces whatever is on it.
       *
       * A cylinder may only have one pipe, so the old one has to come out — and replacing it is the
       * obvious reading of "draw a pipe from here". The snapshot puts it back on Escape.
       */
      ctx.graph.ducts = ctx.graph.ducts.filter(
        (d) => !(d.from.kind === 'valve' && d.from.cylinder === target.cylinder),
      );
    } else if (target.kind === 'node') {
      const joint = ctx.placement.joints.get(target.node);
      from = { kind: 'node', node: target.node };
      place = {
        origin: target.point.clone(),
        heading: joint ? joint.axis.clone() : this.heading.clone(),
      };
    } else if (target.kind === 'ductSurface') {
      const node = splitDuctAt(ctx.graph, target.duct, target.x);
      if (!node) return;
      from = { kind: 'node', node };
      // Leaves along the pipe until the first click turns it, which is the frame the layout will use too.
      place = { origin: target.point.clone(), heading: (target.dir ?? this.heading).clone() };
    } else {
      return;
    }

    const id = newDuctId(ctx.graph, 'drawn');
    ctx.graph.ducts.push({ id, segments: [], from, to: { kind: 'mouth' } });
    this.route = { ductId: id, place, snapshot, base: 0 };
    this.cb.onDrawing?.(true);
    this.cb.onChange(true);
  }

  private finishRoute(sink: DuctSink): void {
    const ctx = this.context;
    if (!ctx || !this.route) return;
    const duct = ctx.graph.ducts.find((d) => d.id === this.route!.ductId);
    if (!duct) {
      this.route = null;
      this.hidePreview();
      return;
    }
    // A route that added nothing is not a pipe, nor an extension of one; put the graph back as it was.
    if (duct.segments.length <= this.route.base) {
      this.cancelRoute();
      return;
    }
    duct.to = sink;
    this.route = null;
    this.hidePreview();
    this.cb.onDrawing?.(false);
    this.cb.onChange(true);
  }

  /** Extend the route to a point, or connect it to whatever the point belongs to. */
  private extendRoute(target: SnapTarget, snap: DrawSnap): void {
    const ctx = this.context;
    if (!ctx || !this.route) return;
    const duct = ctx.graph.ducts.find((d) => d.id === this.route!.ductId);
    if (!duct) return;

    const tip = routeTip(duct.segments, this.route.place);
    const dia = continuingDiameter(
      duct.segments.length > 0 ? duct : this.startingDuct(duct),
      this.startingDiameter(duct),
    );

    const { point } = this.aim(tip, target, snap);

    /**
     * Joining something ends the route in one smooth bend, fitted to arrive along what it joins: square
     * into a turbo's flange, beside another pipe's end, into the flow along a pipe's side, or along a
     * junction's axis. See `bendAnchor`.
     */
    const anchor = this.connectionAnchor(target, duct.id);
    if (anchor) {
      if (anchor.point.distanceTo(tip.point) < MIN_DRAW_LENGTH) return;
      if (duct.segments.length === 0) {
        // Straight out of where it starts: the bend does the turning.
        duct.headingYaw = 0;
        duct.headingPitch = 0;
      }
      duct.segments.push(fitCurve(tip.point, tip.dir, anchor.point, anchor.dir, { dIn: dia, dOut: dia }));
      duct.fitted = true;
      const node = this.connect(target, dia);
      if (node) {
        this.finishRoute({ kind: 'node', node });
        return;
      }
      duct.segments.pop();
      delete duct.fitted;
    }

    if (duct.segments.length === 0) {
      /**
       * The first segment is straight, and the duct's stored heading carries the direction.
       *
       * Unlike every segment after it, it gets no corner of its own: the heading is free, so the heading is
       * turned to face the point instead.
       */
      const dir = point.clone().sub(tip.point);
      if (dir.length() < MIN_DRAW_LENGTH) return;
      const base = this.route.place.heading.clone();
      const turn = headingOffsetTo(base, dir.clone().normalize());
      duct.headingYaw = turn.yaw;
      duct.headingPitch = turn.pitch;
      duct.segments.push(
        makeSegment({ kind: 'pipe', length: dir.length(), dIn: dia, dOut: dia }),
      );
    } else {
      if (point.distanceTo(tip.point) < MIN_DRAW_LENGTH) return;
      duct.segments.push(fitSegment(tip.point, tip.dir, point, { kind: 'pipe', dIn: dia, dOut: dia }));
    }

    // Connecting ends the route; a free point just carries on.
    if (target.kind === 'node') {
      this.finishRoute({ kind: 'node', node: target.node });
      return;
    }
    if (target.kind === 'ductSurface') {
      const node = splitDuctAt(ctx.graph, target.duct, target.x);
      if (node) {
        this.finishRoute({ kind: 'node', node });
        return;
      }
    }
    if (target.kind === 'ductEnd') {
      const area = (Math.PI * dia * dia) / 4;
      const node = joinDuctEnd(ctx.graph, target.duct, area);
      if (node) {
        this.finishRoute({ kind: 'node', node });
        return;
      }
    }
    this.cb.onChange(true);
  }

  /**
   * Where a route ending on `target` bends in to, and the way it arrives, or `null` where it has nothing
   * fixed to arrive along and ends in a corner, as a free point does.
   */
  private connectionAnchor(target: SnapTarget, ductId: string): { point: THREE.Vector3; dir: THREE.Vector3 } | null {
    const ctx = this.context;
    if (!ctx) return null;
    switch (target.kind) {
      case 'turboInlet':
        return { point: target.point.clone(), dir: target.dir.clone() };
      case 'ductSurface':
        return target.dir ? { point: target.point.clone(), dir: target.dir.clone() } : null;
      case 'ductEnd': {
        const other = ctx.graph.ducts.find((d) => d.id === target.duct);
        const place = ctx.placement.ducts.get(target.duct);
        if (!other || !place || other.segments.length === 0) return null;
        const swept = layoutPipe(other.segments, place.origin, place.heading);
        return { point: swept.joints.at(-1)!.clone(), dir: swept.jointDirections.at(-1)!.clone() };
      }
      case 'node':
        return bendAnchor(ctx.graph, ctx.placement, target.node, ductId);
      default:
        return null;
    }
  }

  /**
   * Join the route's duct onto `target`, making the junction that takes where needed: on another pipe's
   * side or open end, or at a turbo's inlet. Returns the node it now ends at.
   */
  private connect(target: SnapTarget, dia: number): string | null {
    const ctx = this.context!;
    switch (target.kind) {
      case 'turboInlet': {
        const mount = ctx.graph.turbos?.find((t) => t.id === target.turbo);
        if (!mount) return null;
        const duct = ctx.graph.ducts.find((d) => d.id === this.route!.ductId)!;
        duct.to = { kind: 'node', node: mount.node };
        ensureTurboOutlet(ctx.graph, mount.node, this.turboOutletDia);
        return mount.node;
      }
      case 'node':
        return target.node;
      case 'ductSurface':
        return splitDuctAt(ctx.graph, target.duct, target.x);
      case 'ductEnd':
        return joinDuctEnd(ctx.graph, target.duct, (Math.PI * dia * dia) / 4);
      default:
        return null;
    }
  }

  /** The duct feeding this one, for continuing at its diameter. */
  private startingDuct(duct: ExhaustDuct): ExhaustDuct | null {
    const ctx = this.context;
    if (!ctx || duct.from.kind !== 'node') return null;
    const feed = ctx.graph.ducts.find(
      (d) => d.to.kind === 'node' && d.to.node === (duct.from as { node: string }).node,
    );
    return feed ?? null;
  }

  private startingDiameter(duct: ExhaustDuct): number {
    const ctx = this.context;
    if (duct.from.kind === 'valve' && ctx) return Math.max(this.portDiameter, 0.02);
    return 0.042;
  }

  /**
   * Where the edited duct's segments stop being editable: its bend into a turbo, joined at both ends, which
   * is fitted rather than drawn. `null` when all of it is. Set by the owner.
   */
  lockedFrom: number | null = null;

  /** Header diameter to start a runner at. Set by the owner from the engine's port. */
  portDiameter = 0.042;

  setPipe(pipe: PipeSegment[]): void {
    this.pipe = pipe;
    this.rebuildHandles();
  }

  /**
   * Point the handles at a different duct.
   *
   * Both halves have to move together: the mesh is what the pointer ray hits and the segment list is
   * what a drag writes to, so pointing one at a new duct and not the other would edit one pipe while
   * showing another. `setPortFrame` supplies the frame separately, because only the caller knows where
   * the duct was actually placed.
   */
  setTarget(mesh: PipeMesh, pipe: PipeSegment[]): void {
    this.pipeMesh = mesh;
    this.pipe = pipe;
  }

  setPortFrame(origin: THREE.Vector3, heading: THREE.Vector3): void {
    this.origin.copy(origin);
    this.heading.copy(heading).normalize();
  }

  get selectedIndex(): number | null {
    return this.selected;
  }

  select(index: number | null): void {
    const moved = index !== this.selected;
    this.selected = index;
    // The rotation rings sit on the selected segment, so they have to move with the selection.
    if (moved) this.rebuildHandles();
    else this.applyHandleColours();
  }

  setHandlesVisible(on: boolean): void {
    this.group.visible = on;
  }

  // -------------------------------------------------------------------------
  // Handles
  // -------------------------------------------------------------------------

  rebuildHandles(): void {
    for (const h of this.handles) {
      h.geometry.dispose();
      this.group.remove(h);
    }
    this.handles.length = 0;

    const layout = layoutPipe(this.pipe, this.origin, this.heading);

    // Inlet ring: the header diameter, the one inlet not fixed by a previous segment.
    if (this.pipe.length > 0) {
      const r0 = this.pipe[0]!.dIn / 2;
      this.addRing(this.origin, this.heading, r0, { kind: 'inlet', segment: 0 });
    }
    // No handles on a bend into a turbo: it is fitted to both its ends, not drawn.
    const editable = Math.min(this.lockedFrom ?? layout.joints.length, layout.joints.length);
    for (let i = 0; i < editable; i++) {
      this.addRing(layout.joints[i]!, layout.jointDirections[i]!, layout.jointRadii[i]!, { kind: 'ring', segment: i });
    }

    /**
     * The selected pipe's triad, at the connection it starts from: its port, junction or turbo outlet. Where
     * it starts is held there, so it has no arrows; its rings swing the whole pipe about that point, and are
     * turned with the pipe, red along the way it sets off.
     */
    if (this.selected !== null && editable > 0) {
      this.pipeTriadAt = { start: this.origin.clone() };
      this.pipeTriad.setMoveOrigin(this.origin);
      this.pipeTriad.setRotateOrigin(this.origin);
      this.pipeTriad.setOrientation(frameAlong(layout.stations[0]!.direction));
      // A pipe from a junction or a turbo has arrows too, which move what it starts from; one from a
      // port has none, since the port is part of the engine.
      const duct = this.context?.graph.ducts.find((d) => d.segments === this.pipe);
      this.pipeTriad.showMoves(duct?.from.kind === 'node');
      // A bent pipe swung about the way it sets off moves the rest of it, so that ring is offered too.
      this.pipeTriad.hideRing(0, false);
    } else {
      this.pipeTriadAt = null;
    }

    this.applyHandleColours();
    // Rebuilt handles come back visible, so draw mode has to hide them again.
    this.applyHandleVisibility();
  }

  private addRing(
    position: THREE.Vector3,
    direction: THREE.Vector3,
    radius: number,
    data: HandleData,
  ): void {
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(Math.max(radius, MIN_RADIUS), 0.0035, 8, 28),
      this.matRing,
    );
    ring.position.copy(position);
    // A torus lies in its local XY plane, so its +Z must point along the pipe.
    ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), direction.clone().normalize());
    ring.userData = data;
    ring.renderOrder = 10;
    this.handles.push(ring);
    this.group.add(ring);
  }

  private applyHandleColours(): void {
    for (const h of this.handles) {
      const data = h.userData as HandleData;
      if (h === this.hovered) h.material = this.matHover;
      else if (data.segment === this.selected) h.material = this.matSelected;
      else h.material = this.matRing;
    }
  }

  // -------------------------------------------------------------------------
  // Pointer handling
  // -------------------------------------------------------------------------

  private updatePointer(e: PointerEvent): void {
    const rect = this.dom.getBoundingClientRect();
    this.pointer.set(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    this.raycaster.setFromCamera(this.pointer, this.camera);
  }

  private onPointerDown = (e: PointerEvent): void => {
    if (this.drawMode && e.button === 2) {
      // Right-click ends the route in open air, the usual way a polyline tool finishes.
      this.updatePointer(e);
      if (this.route) this.finishRoute({ kind: 'mouth' });
      e.preventDefault();
      return;
    }
    if (e.button !== 0) return;
    this.updatePointer(e);

    if (this.placeMode) {
      this.updateGhost();
      if (this.ghost.group.visible) this.cb.onPlaceTurbo?.({ ...this.ghostAt });
      e.preventDefault();
      return;
    }

    if (this.drawMode) {
      this.onDrawClick(e);
      return;
    }

    // A triad first: it sits on what is selected, in front of everything.
    const turboHandle = this.turboTriad.pick(this.raycaster);
    if (turboHandle) {
      this.beginTriadDrag('turbo', turboHandle);
      e.preventDefault();
      return;
    }
    const pipeHandle = this.pipeTriad.pick(this.raycaster);
    if (pipeHandle) {
      this.beginTriadDrag('pipe', pipeHandle);
      e.preventDefault();
      return;
    }

    const hit = this.raycaster.intersectObjects(this.handles, false)[0];
    if (hit) {
      const handle = hit.object as THREE.Mesh;
      const data = handle.userData as HandleData;
      this.beginDrag(handle, data);
      this.select(data.segment);
      this.cb.onSelect(data.segment);
      e.preventDefault();
      return;
    }

    // Anything else under the pointer: a segment of any pipe, a junction, or a turbo.
    const pick = this.pickScene();
    if (this.cb.onPick) {
      this.cb.onPick(pick);
      return;
    }
    const own = pick?.kind === 'segment' && this.isEdited(pick.duct) ? pick.segment : null;
    this.select(own);
    this.cb.onSelect(own);
  };

  /** Whether `ductId` is the duct the handles are on. */
  private isEdited(ductId: string): boolean {
    const ctx = this.context;
    if (!ctx) return false;
    const i = ctx.graph.ducts.findIndex((d) => d.id === ductId);
    return i >= 0 && ctx.meshes[i] === this.pipeMesh;
  }

  /**
   * The pipe segment or junction nearest the pointer along its ray, if any.
   *
   * Nearest wins, across pipes and junctions alike: a junction encloses the ends of the pipes meeting at
   * it, so a click on its body hits it first and a click on a pipe beyond it hits the pipe.
   */
  private pickScene(): ScenePick | null {
    const ctx = this.context;
    if (!ctx) return null;
    let best: { distance: number; pick: ScenePick } | null = null;

    ctx.meshes.forEach((mesh, i) => {
      const duct = ctx.graph.ducts[i];
      const target = mesh.pickTarget;
      if (!duct || !target) return;
      const hit = this.raycaster.intersectObject(target, false)[0];
      if (!hit || (best && hit.distance >= best.distance)) return;
      const st = mesh.stationAt(hit.point);
      if (st) best = { distance: hit.distance, pick: { kind: 'segment', duct: duct.id, segment: st.segment } };
    });
    for (const joint of ctx.joints) {
      if (!joint.target) continue;
      const hit = this.raycaster.intersectObject(joint.target, false)[0];
      if (!hit || (best && hit.distance >= best.distance)) continue;
      best = { distance: hit.distance, pick: { kind: 'joint', node: joint.node } };
    }
    for (const turbo of ctx.turbos) {
      const hit = this.raycaster.intersectObject(turbo.target, true)[0];
      if (!hit || (best && hit.distance >= best.distance)) continue;
      best = { distance: hit.distance, pick: { kind: 'turbo', turbo: turbo.turbo } };
    }
    return (best as { pick: ScenePick } | null)?.pick ?? null;
  }

  /** Take hold of a diameter ring. */
  private beginDrag(handle: THREE.Mesh, data: HandleData): void {
    const layout = layoutPipe(this.pipe, this.origin, this.heading);
    const i = data.segment;
    // Always in a plane facing the camera.
    //
    // A plane perpendicular to the pipe would be the obvious choice, but the pipe is normally viewed
    // side-on, which puts the pointer ray almost parallel to that plane. The intersection then shoots off
    // to infinity for a few pixels of movement, enough to take a 40 mm pipe to the 440 mm clamp in one
    // short drag. A camera-facing plane is always well conditioned; the radius is recovered afterwards as
    // the perpendicular distance from the pipe's axis.
    const normal = new THREE.Vector3();
    this.camera.getWorldDirection(normal);
    const plane = new THREE.Plane();
    plane.setFromNormalAndCoplanarPoint(normal, handle.position);

    const axisPoint = data.kind === 'inlet' ? this.origin.clone() : layout.joints[i]!.clone();
    const axisDir = (
      data.kind === 'inlet' ? this.heading.clone() : layout.jointDirections[i]!.clone()
    ).normalize();

    this.drag = { handle, data, plane, axisPoint, axisDir, lastCommit: 0 };
    this.controls.enabled = false;
  }

  /** Take hold of a triad's handle, on the selected segment or the selected turbo. */
  private beginTriadDrag(on: 'pipe' | 'turbo', handle: TriadHandle): void {
    const triad = on === 'pipe' ? this.pipeTriad : this.turboTriad;
    const moveOrigin = triad.moveOrigin;
    const rotateOrigin = triad.rotateOrigin;
    const axis = triad.axisDir(handle.axis);
    const ring = triad.ring(handle.axis);
    const drag: TriadDrag = {
      on,
      handle,
      axis,
      ring,
      grabOffset: 0,
      grabPoint: moveOrigin.clone(),
      moveOrigin,
      rotateOrigin,
      lastAngle: 0,
      turned: 0,
      lastCommit: 0,
      moved: false,
    };
    const ray = this.raycaster.ray;
    if (handle.kind === 'axis') drag.grabOffset = axisOffset(ray, moveOrigin, axis) ?? 0;
    else if (handle.kind === 'plane') drag.grabPoint = planePoint(ray, moveOrigin, axis) ?? moveOrigin.clone();
    else drag.lastAngle = this.ringAngleNow(rotateOrigin, ring) ?? 0;

    if (on === 'pipe') {
      const ctx = this.context;
      const duct = ctx?.graph.ducts.find((d) => d.segments === this.pipe);
      const drawn = Math.min(this.lockedFrom ?? this.pipe.length, this.pipe.length);
      if (!ctx || !duct || drawn === 0) return;
      const layout = layoutPipe(this.pipe, this.origin, this.heading);
      drag.ductId = duct.id;
      drag.drawn = drawn;
      drag.dirs0 = layout.jointDirections.slice(0, drawn).map((d) => d.clone());
      drag.base = this.headingBase(duct);
      if (duct.from.kind === 'node') {
        const node = duct.from.node;
        const mount = ctx.graph.turbos?.find((t) => t.node === node);
        if (mount?.position) {
          drag.startTurbo = { id: mount.id, position: [...mount.position], rotation: [...mount.rotation] };
        } else {
          drag.startNode = node;
          const axis = ctx.placement.joints.get(node)?.axis ?? this.heading;
          drag.startAxis = [axis.x, axis.y, axis.z];
        }
      }
    } else {
      const mount = this.context?.graph.turbos?.find((t) => t.id === this.selectedTurbo);
      if (!mount?.position) return;
      drag.turbo = mount.id;
      drag.rotation0 = [...mount.rotation];
    }
    this.triadDrag = drag;
    this.controls.enabled = false;
  }

  private ringAngleNow(centre: THREE.Vector3, ring: RingFrame): number | null {
    return ringAngle(this.raycaster.ray, centre, ring, this.camera.getWorldDirection(new THREE.Vector3()));
  }

  /**
   * Where the triad being dragged has moved its point to, from where it began: along the arrow's axis, or
   * in the square's plane, in 5 mm steps with shift held.
   */
  private triadMove(drag: TriadDrag, snap: boolean): THREE.Vector3 | null {
    const axis = drag.axis;
    const ray = this.raycaster.ray;
    if (drag.handle.kind === 'axis') {
      const at = axisOffset(ray, drag.moveOrigin, axis);
      if (at === null) return null;
      const t = at - drag.grabOffset;
      return drag.moveOrigin.clone().addScaledVector(axis, snap ? snapTo(t, MOVE_STEP) : t);
    }
    const point = planePoint(ray, drag.moveOrigin, axis);
    if (!point) return null;
    const d = point.sub(drag.grabPoint);
    d.addScaledVector(axis, -d.dot(axis));
    if (snap) {
      // In steps along the plane's own two axes.
      const { u, v } = drag.ring;
      const a = snapTo(d.dot(u), MOVE_STEP);
      const b = snapTo(d.dot(v), MOVE_STEP);
      d.copy(u).multiplyScalar(a).addScaledVector(v, b);
    }
    return drag.moveOrigin.clone().add(d);
  }

  /** How far the ring being dragged has turned since the drag began, radians, accumulated so it never wraps. */
  private triadTurn(drag: TriadDrag): number | null {
    const angle = this.ringAngleNow(drag.rotateOrigin, drag.ring);
    if (angle === null) return null;
    drag.turned += angleStep(drag.lastAngle, angle);
    drag.lastAngle = angle;
    return drag.turned;
  }

  /** A frame of a triad drag: move or turn what it is on. */
  private dragTriad(snap: boolean): void {
    const drag = this.triadDrag!;
    if (drag.on === 'pipe') this.dragPipeTriad(drag, snap);
    else this.dragTurboTriad(drag, snap);
  }

  /**
   * The selected pipe's arrows: moving what it starts from, the junction or the turbo, and the pipe with
   * it. The pipes feeding it bend in to follow.
   */
  private dragPipeStart(drag: TriadDrag, snap: boolean): void {
    const at = this.triadMove(drag, snap);
    if (!at) return;
    drag.moved = true;
    const delta = at.clone().sub(drag.moveOrigin);
    if (drag.startTurbo) {
      const p = drag.startTurbo.position;
      const position: Vec3 = [p[0] + delta.x, p[1] + delta.y, p[2] + delta.z];
      drag.lastPosition = position;
      const rotation = drag.startTurbo.rotation;
      const id = drag.startTurbo.id;
      this.commitFrame(
        drag,
        () => this.cb.onMoveTurbo?.(id, position, rotation, false),
        () => this.cb.onMoveTurbo?.(id, position, rotation, true),
      );
    } else if (drag.startNode && drag.startAxis) {
      const position: Vec3 = [at.x, at.y, at.z];
      drag.lastPosition = position;
      const node = drag.startNode;
      const axis = drag.startAxis;
      this.commitFrame(
        drag,
        () => this.cb.onMoveJunction?.(node, position, axis, false),
        () => this.cb.onMoveJunction?.(node, position, axis, true),
      );
    }
  }

  /**
   * The direction `duct`'s stored heading is turned off, as `layoutGraph` reads it: its port's, a turbo's
   * outlet flange's, or the world's for a pipe leaving a junction, which is stored in world terms from here
   * on, since the junction's own direction is worked out afresh each time.
   */
  private headingBase(duct: ExhaustDuct): THREE.Vector3 {
    const ctx = this.context!;
    if (duct.from.kind === 'valve') return ctx.ports[duct.from.cylinder]?.direction.clone() ?? this.heading.clone();
    const outlet = ctx.placement.turbos.get(duct.from.node)?.outlet;
    if (outlet && duct.headingFrame !== 'world') return new THREE.Vector3(...outlet.dir);
    duct.headingFrame = 'world';
    return new THREE.Vector3(1, 0, 0);
  }

  /**
   * The selected pipe by its triad: swung as one piece about where it starts.
   *
   * Every drawn segment keeps its length and turns with the rest, so the bends between them stay as they
   * were: the way the pipe sets off goes into its heading, and each corner after is worked out again from
   * the turned directions either side of it. A bend fitted to what the pipe joins at its far end is fitted
   * again when the scene rebuilds. With shift held it lands on 15-degree steps from the engine's axes, so
   * a pipe at an odd angle squares up.
   */
  private dragPipeTriad(drag: TriadDrag, snap: boolean): void {
    const duct = this.context?.graph.ducts.find((d) => d.id === drag.ductId);
    if (!duct || !drag.dirs0 || !drag.base) return;
    if (drag.handle.kind !== 'ring') {
      this.dragPipeStart(drag, snap);
      return;
    }
    let turn = this.triadTurn(drag);
    if (turn === null) return;
    if (snap) turn = snapTurnToEngine(drag.dirs0[0]!, turn, drag.axis, (TRIAD_TURN_DEG * Math.PI) / 180);
    swingPipe(duct, drag.base, drag.dirs0, drag.axis, turn);
    drag.moved = true;
    this.commitFrame(drag, () => this.cb.onChange(false), () => this.cb.onChange(true));
  }

  /**
   * The selected turbo by its triad: moved along one of its own axes or in the plane of two, or turned
   * about one through its centre.
   */
  private dragTurboTriad(drag: TriadDrag, snap: boolean): void {
    const mount = this.context?.graph.turbos?.find((t) => t.id === drag.turbo);
    if (!mount?.position || !drag.rotation0) return;
    let position: Vec3 = [...mount.position];
    let rotation: Quat = mount.rotation;
    if (drag.handle.kind === 'ring') {
      let turn = this.triadTurn(drag);
      if (turn === null) return;
      // Its other two axes swing round the ring: snapped, they land square to the engine's where they can.
      if (snap) turn = snapTurnToEngine(drag.ring.u, turn, drag.axis, (TRIAD_TURN_DEG * Math.PI) / 180);
      const a = drag.axis;
      rotation = quatNormalise(quatMultiply(quatFromAxisAngle([a.x, a.y, a.z], turn), drag.rotation0));
    } else {
      const at = this.triadMove(drag, snap);
      if (!at) return;
      position = [at.x, at.y, at.z];
    }
    drag.moved = true;
    const send = (commit: boolean) => this.cb.onMoveTurbo?.(mount.id, position, rotation, commit);
    this.commitFrame(drag, () => send(false), () => send(true));
  }

  /** Report a drag frame, throttling the ones that rebuild the audio. */
  private commitFrame(drag: TriadDrag, frame: () => void, commit: () => void): void {
    const now = performance.now();
    if (now - drag.lastCommit > AUDIO_COMMIT_MS) {
      drag.lastCommit = now;
      commit();
    } else {
      frame();
    }
  }

  /**
   * A click while drawing: start a route, extend it, or connect it and finish.
   *
   * A double-click ends the route, which is what makes a bare tailpipe possible without hunting for the
   * keyboard — the second click of the pair has already added its segment, so finishing here leaves it.
   */
  private onDrawClick(e: PointerEvent): void {
    const ctx = this.context;
    if (!ctx) return;
    e.preventDefault();

    if (!this.route) {
      const target = this.resolveSnap(this.origin);
      if (target && PipeEditor.startable(target)) this.beginRoute(target);
      return;
    }
    if (e.detail >= 2) {
      this.finishRoute({ kind: 'mouth' });
      return;
    }
    const duct = ctx.graph.ducts.find((d) => d.id === this.route!.ductId);
    const tip = duct ? routeTip(duct.segments, this.route.place) : { point: this.origin, dir: this.heading };
    const target = this.resolveSnap(tip.point);
    if (target) this.extendRoute(target, drawSnapOf(e));
  }

  /** Ghost the segment the next click would add, and mark what it would snap to. */
  private updateDrawPreview(snap: DrawSnap): void {
    const ctx = this.context;
    if (!this.drawMode || !ctx) {
      this.hidePreview();
      return;
    }

    if (!this.route) {
      // Before a route starts, only the places one *could* start from are worth marking.
      const target = this.resolveSnap(this.origin);
      const startable = PipeEditor.startable(target);
      this.marker.visible = startable;
      if (startable) this.marker.position.copy(target!.point);
      this.preview.visible = false;
      return;
    }

    const duct = ctx.graph.ducts.find((d) => d.id === this.route!.ductId);
    if (!duct) return;
    const tip = routeTip(duct.segments, this.route.place);
    const target = this.resolveSnap(tip.point);
    if (!target) {
      this.hidePreview();
      return;
    }

    const { point, dir, name } = this.aim(tip, target, snap);

    // Joining something, the bend the pipe will take to arrive along it.
    const anchor = this.connectionAnchor(target, this.route.ductId);
    const path = anchor
      ? layoutPipe([fitCurve(tip.point, tip.dir, anchor.point, anchor.dir)], tip.point, tip.dir).stations.map(
          (st) => st.position,
        )
      : [tip.point, point];
    this.previewGeom.setFromPoints(path);
    this.preview.computeLineDistances();
    this.preview.visible = true;
    if (dir) {
      this.previewMat.color.copy(name === 'straight on' ? STRAIGHT_ON_COLOUR : axisColour(dir));
      this.guideMat.color.copy(this.previewMat.color);
      this.guideGeom.setFromPoints([
        tip.point,
        tip.point.clone().addScaledVector(dir, point.distanceTo(tip.point) + GUIDE_REACH),
      ]);
      this.guide.visible = true;
    } else {
      this.previewMat.color.set(PREVIEW_COLOUR);
      this.guide.visible = false;
    }
    const length = `${Math.round(point.distanceTo(tip.point) * 1000)} mm`;
    this.cb.onAim?.(target.kind === 'free' ? `${name ? `${name}, ` : ''}${length}` : null);
    // Marked only when it would *connect*, so the highlight means something.
    this.marker.visible = target.kind !== 'free';
    this.marker.position.copy(point);
    this.snapped = target;
  }

  private onPointerMove = (e: PointerEvent): void => {
    if (this.placeMode) {
      this.updatePointer(e);
      this.updateGhost();
      return;
    }
    if (this.triadDrag) {
      this.updatePointer(e);
      this.dragTriad(e.shiftKey);
      e.preventDefault();
      return;
    }
    if (this.drawMode && !this.drag) {
      this.updatePointer(e);
      this.updateDrawPreview(drawSnapOf(e));
      return;
    }
    this.updatePointer(e);

    if (!this.drag) {
      const onTriad = this.turboTriad.hover(this.raycaster) || this.pipeTriad.hover(this.raycaster);
      const hit = onTriad ? undefined : this.raycaster.intersectObjects(this.handles, false)[0];
      const next = (hit?.object as THREE.Mesh | undefined) ?? null;
      if (next !== this.hovered) {
        this.hovered = next;
        this.applyHandleColours();
      }
      // A hand over anything clickable, so it is discoverable that pipes and junctions can be picked.
      this.dom.style.cursor = onTriad || next ? 'grab' : this.pickScene() ? 'pointer' : '';
      return;
    }

    const seg = this.pipe[this.drag.data.segment];
    if (!seg) return;
    const point = new THREE.Vector3();
    if (!this.raycaster.ray.intersectPlane(this.drag.plane, point)) return;
    this.dragRadius(seg, point);

    const now = performance.now();
    const commit = now - this.drag.lastCommit > AUDIO_COMMIT_MS;
    if (commit) this.drag.lastCommit = now;
    this.cb.onChange(commit);
    e.preventDefault();
  };

  /** Grab a joint ring and pull: sets the diameter at that joint. */
  private dragRadius(seg: PipeSegment, target: THREE.Vector3): void {
    // Perpendicular distance from the pipe's axis to the dragged point.
    const { axisPoint, axisDir } = this.drag!;
    const rel = target.clone().sub(axisPoint);
    rel.sub(axisDir.clone().multiplyScalar(rel.dot(axisDir)));
    const radius = clamp(rel.length(), MIN_RADIUS, MAX_RADIUS);
    const d = radius * 2;
    const i = this.drag!.data.segment;

    if (this.drag!.data.kind === 'inlet') {
      seg.dIn = d;
      if (seg.kind === 'pipe') seg.dOut = d;
      // A 'pipe' segment's outlet tracks its inlet, so the next segment has to
      // follow too or the duct develops a phantom step.
      this.propagate(i);
      return;
    }

    // The outlet diameter is `dIn` for a chamber (it necks back down to the throat)
    // or a pipe, and `dOut` for a cone — matching `segmentDiameter(seg, 1)`.
    if (seg.kind === 'chamber') seg.dIn = d;
    else if (seg.kind === 'pipe') {
      seg.dIn = d;
      seg.dOut = d;
    } else seg.dOut = d;

    this.propagate(i);
  }

  /** Keep the duct continuous: the next segment starts where this one ends. */
  private propagate(index: number): void {
    const seg = this.pipe[index];
    const next = this.pipe[index + 1];
    if (!seg || !next) return;
    next.dIn = segmentDiameter(seg, 1);
    if (next.kind === 'pipe') next.dOut = next.dIn;
  }

  private onPointerUp = (): void => {
    if (this.triadDrag) {
      const drag = this.triadDrag;
      this.triadDrag = null;
      this.controls.enabled = true;
      if (!drag.moved) return;
      // Final authoritative push, since intermediate frames were throttled.
      if (drag.on === 'turbo') {
        const mount = this.context?.graph.turbos?.find((t) => t.id === drag.turbo);
        if (mount?.position) this.cb.onMoveTurbo?.(mount.id, mount.position, mount.rotation, true);
      } else if (drag.lastPosition && drag.startTurbo) {
        this.cb.onMoveTurbo?.(drag.startTurbo.id, drag.lastPosition, drag.startTurbo.rotation, true);
      } else if (drag.lastPosition && drag.startNode && drag.startAxis) {
        this.cb.onMoveJunction?.(drag.startNode, drag.lastPosition, drag.startAxis, true);
      } else {
        this.cb.onChange(true);
        this.rebuildHandles();
      }
      return;
    }
    if (!this.drag) return;
    this.drag = null;
    this.controls.enabled = true;
    this.dom.style.cursor = '';
    // Final authoritative push, since intermediate frames were throttled.
    this.cb.onChange(true);
    this.rebuildHandles();
  };

  dispose(): void {
    this.dom.removeEventListener('pointerdown', this.onPointerDown);
    this.dom.removeEventListener('pointermove', this.onPointerMove);
    window.removeEventListener('pointerup', this.onPointerUp);
    for (const h of this.handles) h.geometry.dispose();
    this.pipeTriad.dispose();
    this.turboTriad.dispose();
    this.previewGeom.dispose();
    this.guideGeom.dispose();
    this.ghost.dispose();
    for (const m of [this.matHover, this.matSelected, this.matRing, this.previewMat, this.guideMat]) {
      m.dispose();
    }
  }
}

function clamp(x: number, lo: number, hi: number): number {
  return x < lo ? lo : x > hi ? hi : x;
}
