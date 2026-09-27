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
  lockedFrom,
  quatFromAxisAngle,
  quatMultiply,
  quatNormalise,
  seatTurbo,
  type TurboSize,
} from '../model/turbo.js';
import type { Quat } from '../model/exhaustGraph.js';
import {
  attachToLooseStart,
  joinDuctEnd,
  junctionAt,
  newDuctId,
  splitDuctAt,
  type DuctSink,
  type DuctSource,
  type ExhaustDuct,
  type ExhaustGraph,
} from '../model/exhaustGraph.js';
import { PipeMesh, layoutPipe, turnHeading } from './PipeMesh.js';
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
  MIN_BEND_BORES,
  MIN_DRAW_LENGTH,
  bendSegment,
  bendWhole,
  bendAnchor,
  collectSnapTargets,
  flipLoosePipe,
  diameterAt,
  type BendAnchor,
  swingPipe,
  pipeShape,
  type PipeShape,
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

/** The attachment dot's colours: on a port, pipe or turbo, and on a junction. */
const ATTACH_MARKER = 0xffd166;
const JUNCTION_MARKER = 0xc792ff;

/** Length of a loose pipe as it is put down, m: `placeLoosePipe`'s. */
const LOOSE_PIPE_LENGTH = 0.3;

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
  shape0?: PipeShape;
  /** What a pipe's arrows move: the junction it starts from, and the way it points, or a loose pipe. */
  startNode?: string;
  startAxis?: Vec3;
  loose?: Vec3;
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
  /** The bend the bend tool is making, described, or `null` when it is making none. */
  onBendAim?: (aim: string | null) => void;
  /**
   * The bend tool bent pipe `ductId`, which was `length` long: it is that long still, but for a bend fitted
   * into what it joins, which the owner fits to that length again.
   */
  onBent?: (ductId: string, length: number) => void;
  /** A turbo was put down. */
  onPlaceTurbo?: (placement: TurboPlacement) => void;
  /** A loose pipe was put down, starting at `position`. */
  onPlacePipe?: (position: Vec3) => void;
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

  /** Placing a turbo, or a loose pipe: the see-through one following the pointer, and where it is. */
  private placeMode = false;
  private placeKind: 'turbo' | 'pipe' = 'turbo';
  private readonly ghostPipe = new THREE.Mesh(
    new THREE.CylinderGeometry(0.021, 0.021, LOOSE_PIPE_LENGTH, 24).rotateZ(-Math.PI / 2).translate(LOOSE_PIPE_LENGTH / 2, 0, 0),
    new THREE.MeshStandardMaterial({ color: 0xb9bdc3, transparent: true, opacity: 0.45, depthWrite: false }),
  );
  private readonly ghost = new TurboMesh(true);
  private ghostAt: TurboPlacement = { position: [0, 0, 0], rotation: [...IDENTITY] };
  /** Turbos are drawn this size, and put down at this height unless snapped to a pipe. Set by the owner. */
  private size: TurboSize = { scroll: 0.07, depth: 0.06, outletDia: 0.058, inletDia: 0.042 };
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
  /** The plane a bend being drawn turns in, faintly, so which way it goes reads in 3D. */
  private readonly bendDisc: THREE.Mesh;

  /**
   * Whether clicks while drawing lay bends rather than corners: set from the panel, and flipped for as long
   * as B is held.
   */
  bendMode = false;
  private bendHeld = false;

  /**
   * The bend tool: click a straight, then drag one of the two rings put where it starts, each lying in a
   * plane of the pipe's own, up and down or side to side, to bend the whole straight into one arc in that
   * plane. It keeps its length; a ghost shows where it is going until it is let go.
   */
  private bendTool = false;
  private readonly bendTriad = new Triad(PIPE_TRIAD_SIZE);
  /** The pipe as the bend being dragged would leave it. */
  private readonly bendGhost = new PipeMesh(true);
  private bendAt: {
    ductId: string;
    index: number;
    /** The way the straight runs, in the world. */
    dir: THREE.Vector3;
    /** The pipe's length before it is bent, m. */
    length: number;
  } | null = null;
  private bendDrag: {
    axis: THREE.Vector3;
    ring: RingFrame;
    lastAngle: number;
    turned: number;
    proposed: PipeSegment[] | null;
  } | null = null;
  private lastSnap: DrawSnap = 'engine';

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
    window.addEventListener('keyup', this.onKeyUp);
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
      new THREE.MeshBasicMaterial({ color: ATTACH_MARKER, transparent: true, opacity: 0.85 }),
    );
    this.marker.visible = false;
    this.marker.renderOrder = 13;
    this.group.add(this.marker);

    this.bendDisc = new THREE.Mesh(
      new THREE.CircleGeometry(1, 48),
      new THREE.MeshBasicMaterial({ color: PREVIEW_COLOUR, transparent: true, opacity: 0.12, side: THREE.DoubleSide, depthWrite: false }),
    );
    this.bendDisc.visible = false;
    this.bendDisc.renderOrder = 10;
    this.group.add(this.bendDisc);

    this.ghost.group.visible = false;
    this.group.add(this.ghost.group);
    this.ghostPipe.visible = false;
    this.group.add(this.ghostPipe);
    this.group.add(this.pipeTriad.group, this.turboTriad.group, this.bendTriad.group, this.bendGhost.group);
    this.bendGhost.group.visible = false;
    // Only its rings: a bend is made by turning, and not about the pipe's own axis, which would do nothing.
    this.bendTriad.showMoves(false);
    this.bendTriad.hideRing(0, true);
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

  /** Start or stop placing a turbo, or a loose pipe. Stops drawing, since the two share the pointer. */
  setPlaceMode(on: boolean, kind: 'turbo' | 'pipe' = 'turbo'): void {
    if (on && this.drawMode) this.setDrawMode(false);
    this.placeMode = on;
    this.placeKind = kind;
    this.ghost.rebuild(this.size);
    this.ghost.group.visible = false;
    this.ghostPipe.visible = false;
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
    if (this.placeKind === 'pipe') {
      // A loose pipe goes down level with the ports, heading along the engine's x; its triad turns it.
      const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -this.turboHeight);
      const point = new THREE.Vector3();
      this.ghostPipe.visible = !!this.raycaster.ray.intersectPlane(plane, point);
      if (this.ghostPipe.visible) this.ghostPipe.position.copy(point);
      return;
    }
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
    return this.drag !== null || this.triadDrag !== null || this.bendDrag !== null;
  }

  /** Switch the bend tool on or off. Drawing and placing stop while it is on. */
  setBendTool(on: boolean): void {
    if (on && this.drawMode) this.setDrawMode(false);
    if (on && this.placeMode) this.setPlaceMode(false);
    this.bendTool = on;
    this.bendAt = null;
    this.bendDrag = null;
    this.bendTriad.setVisible(false);
    this.hidePreview();
    this.cb.onBendAim?.(null);
    this.applyHandleVisibility();
  }

  get bendToolOn(): boolean {
    return this.bendTool;
  }

  /**
   * Put the bend tool's rings where the straight under the pointer starts, if it is one that can be bent: a
   * pipe's own straight, not a can, a bend, or the bend fitted into what it joins.
   */
  private placeBend(): boolean {
    const ctx = this.context;
    if (!ctx) return false;
    let best: { distance: number; duct: ExhaustDuct; index: number } | null = null;
    ctx.meshes.forEach((mesh, i) => {
      const duct = ctx.graph.ducts[i];
      const target = mesh.pickTarget;
      if (!duct || !target) return;
      const hit = this.raycaster.intersectObject(target, false)[0];
      if (!hit || (best && hit.distance >= best.distance)) return;
      const st = mesh.stationAt(hit.point);
      if (st) best = { distance: hit.distance, duct, index: st.segment };
    });
    if (!best) return false;
    const { duct, index } = best as { duct: ExhaustDuct; index: number };
    const seg = duct.segments[index];
    const locked = lockedFrom(ctx.graph, duct);
    const place = ctx.placement.ducts.get(duct.id);
    if (!seg || !place || seg.curve || seg.kind === 'chamber' || (locked !== null && index >= locked)) return false;
    const swept = layoutPipe(duct.segments, place.origin, place.heading);
    const start = index === 0 ? place.origin : swept.joints[index - 1]!;
    const end = swept.joints[index]!;
    const dir = end.clone().sub(start).normalize();
    this.bendAt = { ductId: duct.id, index, dir, length: duct.segments.reduce((a, s) => a + s.length, 0) };
    this.bendTriad.setOrientation(frameAlong(dir));
    this.bendTriad.setRotateOrigin(start);
    this.bendTriad.setMoveOrigin(start);
    this.bendTriad.setVisible(true);
    this.cb.onBendAim?.('Drag a ring to bend the pipe in its plane');
    return true;
  }

  private beginBendDrag(handle: TriadHandle): void {
    const ring = this.bendTriad.ring(handle.axis);
    this.bendDrag = {
      axis: this.bendTriad.axisDir(handle.axis),
      ring,
      lastAngle: this.ringAngleNow(this.bendTriad.rotateOrigin, ring) ?? 0,
      turned: 0,
      proposed: null,
    };
    this.controls.enabled = false;
  }

  /** Follow the bend being dragged: the pipe it would make, ghosted, and what it is. */
  private dragBend(snap: boolean): void {
    const ctx = this.context;
    const drag = this.bendDrag;
    const at = this.bendAt;
    if (!ctx || !drag || !at) return;
    const duct = ctx.graph.ducts.find((d) => d.id === at.ductId);
    const seg = duct?.segments[at.index];
    const place = ctx.placement.ducts.get(at.ductId);
    if (!duct || !seg || !place) return;
    const now = this.ringAngleNow(this.bendTriad.rotateOrigin, drag.ring);
    if (now === null) return;
    drag.turned += angleStep(drag.lastAngle, now);
    drag.lastAngle = now;
    const step = (TRIAD_TURN_DEG * Math.PI) / 180;
    const angle = snap ? snapTo(drag.turned, step) : drag.turned;
    const tightest = MIN_BEND_BORES * Math.max(seg.dIn, seg.dOut);
    const bent = bendWhole(seg, at.dir, drag.axis, angle, tightest);
    drag.proposed = [...duct.segments.slice(0, at.index), bent.segment, ...duct.segments.slice(at.index + 1)];
    // The ghost: the pipe as it would be.
    this.bendGhost.rebuild(drag.proposed, place.origin, place.heading);
    this.bendGhost.group.visible = true;
    const degrees = Math.round((Math.abs(bent.angle) * 180) / Math.PI);
    const plane = this.bendTriad.axisDir(1).angleTo(drag.axis) < 0.1 ? 'side to side' : 'up and down';
    this.cb.onBendAim?.(
      `bend ${degrees}° ${plane}` +
        (Number.isFinite(bent.radius) ? `, radius ${Math.round(bent.radius * 1000)} mm` : '') +
        (bent.clamped ? ': as far as it turns, no tighter than one and a half bores' : ''),
    );
  }

  /** Let go of the bend: the pipe takes the shape the ghost showed. */
  private endBendDrag(): void {
    const ctx = this.context;
    const drag = this.bendDrag;
    const at = this.bendAt;
    this.bendDrag = null;
    this.controls.enabled = true;
    this.bendGhost.group.visible = false;
    if (!ctx || !drag?.proposed || !at) return;
    const duct = ctx.graph.ducts.find((d) => d.id === at.ductId);
    if (!duct || !drag.proposed[at.index]?.curve) return;
    duct.segments = drag.proposed;
    this.bendAt = null;
    this.bendTriad.setVisible(false);
    this.cb.onBendAim?.(null);
    this.cb.onBent?.(duct.id, at.length);
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

  /** The attachment dot: violet on a junction, so it reads apart from a pipe end or port's yellow. */
  private colourMarker(target: SnapTarget): void {
    (this.marker.material as THREE.MeshBasicMaterial).color.setHex(
      target.kind === 'node' ? JUNCTION_MARKER : ATTACH_MARKER,
    );
  }

  private hidePreview(): void {
    this.preview.visible = false;
    this.guide.visible = false;
    this.bendDisc.visible = false;
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
    const hide = this.drawMode || this.placeMode || this.bendTool;
    for (const h of this.handles) h.visible = !hide;
    this.pipeTriad.setVisible(!hide && this.pipeTriadAt !== null);
  }

  /** Where the selected pipe's triad goes: where the pipe starts, which it turns about. */
  private pipeTriadAt: { start: THREE.Vector3 } | null = null;

  private onContextMenu = (e: MouseEvent): void => {
    if (this.drawMode) e.preventDefault();
  };

  /** Whether the next click lays a bend. */
  private get bending(): boolean {
    return this.bendMode !== this.bendHeld;
  }

  private onKeyUp = (e: KeyboardEvent): void => {
    if ((e.key === 'b' || e.key === 'B') && this.bendHeld) {
      this.bendHeld = false;
      if (this.drawMode) this.updateDrawPreview(this.lastSnap);
    }
  };

  /**
   * The bend a click would lay from `tip`: turning to head the way the pointer is aimed, `dir` where that is
   * locked to one, round a radius of how far the pointer is from the tip, no tighter than `MIN_BEND_BORES`.
   * `null` where the aim leaves no bend to make, the way the pipe already goes or straight back.
   */
  private bendFor(
    tip: { point: THREE.Vector3; dir: THREE.Vector3 },
    point: THREE.Vector3,
    dir: THREE.Vector3 | undefined,
    dia: number,
  ): { bend: PipeSegment; to: THREE.Vector3; radius: number } | null {
    const to = (dir ?? point.clone().sub(tip.point)).clone().normalize();
    if (to.lengthSq() < 0.5) return null;
    const radius = Math.max(quantiseLength(point.distanceTo(tip.point), LENGTH_GRID_M), MIN_BEND_BORES * dia);
    const bend = bendSegment(tip.point, tip.dir, to, radius, { kind: 'pipe', dIn: dia, dOut: dia });
    return bend ? { bend, to, radius } : null;
  }

  /** The bore the next segment of the route is drawn at: the bore the pipe ends at, or starts at. */
  private routeDiameter(duct: ExhaustDuct): number {
    const ctx = this.context!;
    const fromTurbo = duct.from.kind === 'node' && !!ctx.graph.turbos?.some((t) => t.node === (duct.from as { node: string }).node);
    return continuingDiameter(
      duct.segments.length > 0 ? duct : fromTurbo ? null : this.startingDuct(duct),
      this.startingDiameter(duct),
    );
  }

  private onKeyDown = (e: KeyboardEvent): void => {
    const typing = e.target instanceof HTMLInputElement || e.target instanceof HTMLSelectElement || e.target instanceof HTMLTextAreaElement;
    if ((e.key === 'b' || e.key === 'B') && !typing && !e.repeat && this.drawMode) {
      this.bendHeld = true;
      this.updateDrawPreview(this.lastSnap);
      return;
    }
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
      // A bend into a turbo or a junction is fitted, not drawn, so nothing branches off it.
      const locked = lockedFrom(ctx.graph, duct);
      if (locked !== null && st.segment >= locked) continue;
      // Nor off a bend drawn into it: a pipe is split along its length, and a bend is not cut.
      if (duct.segments[st.segment]?.curve) continue;
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
    return !!target && target.kind !== 'free' && target.kind !== 'turboInlet' && target.kind !== 'looseStart';
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
    } else if (target.kind === 'turboOutlet') {
      from = { kind: 'node', node: target.node };
      place = { origin: target.point.clone(), heading: target.dir.clone() };
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
    const dia = this.routeDiameter(duct);

    const { point, dir: locked } = this.aim(tip, target, snap);

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
      // Tapering from the bore it leaves at to the bore of what it joins, so it matches at both.
      duct.segments.push(fitCurve(tip.point, tip.dir, anchor.point, anchor.dir, { dIn: dia, dOut: anchor.dia }));
      duct.fitted = true;
      const node = this.connect(target);
      if (node) {
        this.finishRoute({ kind: 'node', node });
        return;
      }
      duct.segments.pop();
      delete duct.fitted;
    }

    const straightOut = target.kind === 'free' ? this.portRun(duct, tip, point) : null;
    // A bend, from where the pipe is heading round to where it is aimed; out of a port it runs straight first.
    const bend = target.kind === 'free' && straightOut === null && this.bending ? this.bendFor(tip, point, locked, dia) : null;
    if (bend) {
      if (duct.segments.length === 0) {
        duct.headingYaw = 0;
        duct.headingPitch = 0;
      }
      duct.segments.push(bend.bend);
      this.cb.onChange(true);
      return;
    }
    if (straightOut !== null) {
      // Out of an exhaust port, straight on at first: the next segment turns.
      if (straightOut < MIN_DRAW_LENGTH) return;
      duct.headingYaw = 0;
      duct.headingPitch = 0;
      duct.segments.push(makeSegment({ kind: 'pipe', length: straightOut, dIn: dia, dOut: dia }));
    } else if (duct.segments.length === 0) {
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
      const node = joinDuctEnd(ctx.graph, target.duct);
      if (node) {
        this.finishRoute({ kind: 'node', node });
        return;
      }
    }
    this.cb.onChange(true);
  }

  /**
   * How long a pipe's first segment out of an exhaust port runs, m, straight on out of it towards `point`:
   * as far along the port's direction as the point is, in the drawing's length steps. `null` for any other
   * segment, which may turn.
   */
  private portRun(duct: ExhaustDuct, tip: { point: THREE.Vector3; dir: THREE.Vector3 }, point: THREE.Vector3): number | null {
    if (duct.from.kind !== 'valve' || duct.segments.length > 0) return null;
    return quantiseLength(Math.max(point.clone().sub(tip.point).dot(tip.dir), 0), LENGTH_GRID_M);
  }

  /**
   * Where a route ending on `target` bends in to, and the way it arrives, or `null` where it has nothing
   * fixed to arrive along and ends in a corner, as a free point does.
   */
  private connectionAnchor(target: SnapTarget, ductId: string): BendAnchor | null {
    const ctx = this.context;
    if (!ctx) return null;
    switch (target.kind) {
      case 'turboInlet':
      case 'looseStart':
        return { point: target.point.clone(), dir: target.dir.clone(), dia: target.dia };
      case 'ductSurface': {
        const other = ctx.graph.ducts.find((d) => d.id === target.duct);
        if (!target.dir || !other) return null;
        return { point: target.point.clone(), dir: target.dir.clone(), dia: diameterAt(other.segments, target.x) };
      }
      case 'ductEnd': {
        const other = ctx.graph.ducts.find((d) => d.id === target.duct);
        const place = ctx.placement.ducts.get(target.duct);
        if (!other || !place || other.segments.length === 0) return null;
        const swept = layoutPipe(other.segments, place.origin, place.heading);
        // A loose pipe is turned round to carry on from here, so the bend arrives going the way it will.
        const loose = other.from.kind === 'free';
        const dir = swept.jointDirections.at(-1)!.clone();
        return {
          point: swept.joints.at(-1)!.clone(),
          dir: loose ? dir.negate() : dir,
          dia: segmentDiameter(other.segments.at(-1)!, 1),
        };
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
  private connect(target: SnapTarget): string | null {
    const ctx = this.context!;
    switch (target.kind) {
      case 'turboInlet': {
        const mount = ctx.graph.turbos?.find((t) => t.id === target.turbo);
        if (!mount) return null;
        const duct = ctx.graph.ducts.find((d) => d.id === this.route!.ductId)!;
        duct.to = { kind: 'node', node: mount.node };
        return mount.node;
      }
      case 'node':
        return target.node;
      case 'looseStart':
        return attachToLooseStart(ctx.graph, this.route!.ductId, target.duct, [target.dir.x, target.dir.y, target.dir.z]);
      case 'ductSurface':
        return splitDuctAt(ctx.graph, target.duct, target.x);
      case 'ductEnd': {
        // A loose pipe's far end: turned round so that end is where it starts, it carries on from the
        // pipe drawn into it, with no junction of pipes and nothing added. Anything else's is a merge.
        const other = ctx.graph.ducts.find((d) => d.id === target.duct);
        const place = ctx.placement.ducts.get(target.duct);
        if (other?.from.kind === 'free' && place) {
          flipLoosePipe(other, place);
          const start = new THREE.Vector3(...other.from.position);
          const heading = turnHeadingWorld(other);
          const first = layoutPipe(other.segments, start, heading).stations[0]!.direction;
          return attachToLooseStart(ctx.graph, this.route!.ductId, other.id, [first.x, first.y, first.z]);
        }
        // Fixed where the pipe ends, which way it points: the drawn pipe bends in alongside it.
        const anchor = this.connectionAnchor(target, this.route!.ductId);
        return joinDuctEnd(
          ctx.graph,
          target.duct,
          anchor ? { position: [anchor.point.x, anchor.point.y, anchor.point.z], axis: [anchor.dir.x, anchor.dir.y, anchor.dir.z] } : undefined,
        );
      }
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
    // Out of a turbo, at the bore of its outlet.
    const from = duct.from;
    if (from.kind === 'node' && ctx?.graph.turbos?.some((t) => t.node === from.node)) return this.turboOutletDia;
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
     * The selected pipe's triad, at where it starts.
     *
     * A loose pipe is free: its arrows move it and its three rings swing it every way, turned with the pipe,
     * red along the way it sets off. A pipe that starts at a connection is held to the face it starts from,
     * as a pipe is to a flange: it turns only in that face's plane, about the connection's axis (`faceAxis`),
     * by the one ring round it, so where it sets off from stays put. Its arrows move the junction it
     * leaves, where it leaves one of its own; a port, a turbo's outlet and the end of another pipe hold it.
     * A straight pipe along that axis would turn to no effect, so it has no ring.
     */
    const graph = this.context?.graph;
    const duct = graph?.ducts.find((d) => d.segments === this.pipe);
    this.pipeTriadAt = null;
    if (this.selected !== null && editable > 0 && duct) {
      const face = this.faceAxis(duct);
      const shape = pipeShape(this.pipe.slice(0, editable), this.heading);
      const turns = face
        ? [...shape.starts, ...shape.ends].some((d) => d.clone().cross(face).lengthSq() > 1e-8)
        : true;
      const movable = duct.from.kind === 'free' || (duct.from.kind === 'node' && !this.heldAtStart(duct));
      if (turns || movable) {
        this.pipeTriadAt = { start: this.origin.clone() };
        this.pipeTriad.setMoveOrigin(this.origin);
        this.pipeTriad.setRotateOrigin(this.origin);
        this.pipeTriad.setOrientation(frameAlong(face ?? layout.stations[0]!.direction));
        this.pipeTriad.showMoves(movable);
        // Held to a face, only the ring round its axis; loose, all three.
        this.pipeTriad.hideRing(0, !turns);
        this.pipeTriad.hideRing(1, !!face);
        this.pipeTriad.hideRing(2, !!face);
      }
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
      if (this.placeKind === 'pipe') {
        const p = this.ghostPipe.position;
        if (this.ghostPipe.visible) this.cb.onPlacePipe?.([p.x, p.y, p.z]);
      } else if (this.ghost.group.visible) {
        this.cb.onPlaceTurbo?.({ ...this.ghostAt });
      }
      e.preventDefault();
      return;
    }

    if (this.drawMode) {
      this.onDrawClick(e);
      return;
    }

    if (this.bendTool) {
      const ring = this.bendTriad.pick(this.raycaster);
      if (ring) this.beginBendDrag(ring);
      else if (!this.placeBend()) {
        this.bendAt = null;
        this.bendTriad.setVisible(false);
        this.cb.onBendAim?.(null);
      }
      e.preventDefault();
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
      drag.ductId = duct.id;
      drag.drawn = drawn;
      drag.shape0 = pipeShape(this.pipe.slice(0, drawn), this.heading);
      drag.base = this.headingBase(duct);
      if (duct.from.kind === 'free') {
        drag.loose = [...duct.from.position];
      } else if (duct.from.kind === 'node') {
        const node = duct.from.node;
        drag.startNode = node;
        const axis = ctx.placement.joints.get(node)?.axis ?? this.heading;
        drag.startAxis = [axis.x, axis.y, axis.z];
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
   * The selected pipe's arrows: moving the junction it starts from, and the pipe with it, the pipes feeding
   * it bending in to follow; or a loose pipe, which has nothing attached.
   */
  private dragPipeStart(drag: TriadDrag, snap: boolean): void {
    const at = this.triadMove(drag, snap);
    if (!at) return;
    drag.moved = true;
    // A loose pipe moves itself: it is where it was put down, and nothing is attached to it.
    const duct = this.context?.graph.ducts.find((d) => d.id === drag.ductId);
    if (drag.loose && duct?.from.kind === 'free') {
      duct.from = { kind: 'free', position: [at.x, at.y, at.z] };
      this.commitFrame(drag, () => this.cb.onChange(false), () => this.cb.onChange(true));
      return;
    }
    if (drag.startNode && drag.startAxis) {
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
  /**
   * The axis of the face a pipe starts from, which it turns about, held to it: its port's, its turbo
   * outlet's, the way the pipe it carries on from finishes, or its junction's. `null` for a loose pipe,
   * which is held to nothing.
   */
  private faceAxis(duct: ExhaustDuct): THREE.Vector3 | null {
    const ctx = this.context!;
    if (duct.from.kind === 'free') return null;
    if (duct.from.kind === 'valve') return ctx.ports[duct.from.cylinder]?.direction.clone().normalize() ?? this.heading.clone();
    const node = duct.from.node;
    const outlet = ctx.placement.turbos.get(node)?.outlet;
    if (outlet) return new THREE.Vector3(...outlet.dir).normalize();
    const carried = duct.continues ? ctx.graph.ducts.find((d) => d.id === duct.continues) : undefined;
    const place = carried ? ctx.placement.ducts.get(carried.id) : undefined;
    if (carried && place && carried.segments.length > 0) {
      return layoutPipe(carried.segments, place.origin, place.heading).jointDirections.at(-1)!.clone();
    }
    const pinned = junctionAt(ctx.graph, node);
    if (pinned) return new THREE.Vector3(...pinned.axis).normalize();
    return ctx.placement.joints.get(node)?.axis.clone().normalize() ?? this.heading.clone();
  }

  /**
   * Whether a pipe from a junction is held where it starts: leaving a turbo's outlet, or carrying straight
   * on from a pipe, it goes where that points it, so its junction is not moved from it.
   */
  private heldAtStart(duct: ExhaustDuct): boolean {
    const ctx = this.context!;
    if (duct.from.kind !== 'node') return duct.from.kind === 'valve';
    const node = duct.from.node;
    if (ctx.graph.turbos?.some((t) => t.node === node)) return true;
    const carried = duct.continues ? ctx.graph.ducts.find((d) => d.id === duct.continues) : undefined;
    return carried !== undefined && !carried.fitted;
  }

  private headingBase(duct: ExhaustDuct): THREE.Vector3 {
    const ctx = this.context!;
    if (duct.from.kind === 'valve') return ctx.ports[duct.from.cylinder]?.direction.clone() ?? this.heading.clone();
    if (duct.from.kind === 'free') return new THREE.Vector3(1, 0, 0);
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
    if (!duct || !drag.shape0 || !drag.base) return;
    if (drag.handle.kind !== 'ring') {
      this.dragPipeStart(drag, snap);
      return;
    }
    let turn = this.triadTurn(drag);
    if (turn === null) return;
    if (snap) {
      // Squared up by the first way the pipe goes that swings round: turning about a face's axis, the first
      // straight out of it may lie along the axis and not swing at all.
      const dirs = [...drag.shape0.starts, ...drag.shape0.ends];
      const ref = dirs.find((d) => d.clone().cross(drag.axis).lengthSq() > 1e-6) ?? drag.shape0.starts[0]!;
      turn = snapTurnToEngine(ref, turn, drag.axis, (TRIAD_TURN_DEG * Math.PI) / 180);
    }
    swingPipe(duct, drag.base, drag.shape0, drag.axis, turn);
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
      if (startable) {
        this.marker.position.copy(target!.point);
        this.colourMarker(target!);
      }
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

    const aimed = this.aim(tip, target, snap);
    let { point, dir, name } = aimed;
    // Out of an exhaust port the first segment runs straight on.
    const run = target.kind === 'free' ? this.portRun(duct, tip, point) : null;
    if (run !== null) {
      point = tip.point.clone().addScaledVector(tip.dir, run);
      dir = tip.dir.clone();
      name = 'straight on';
    }

    // Joining something, the bend the pipe will take to arrive along it; laying a bend, that bend.
    const anchor = this.connectionAnchor(target, this.route.ductId);
    const bend =
      !anchor && run === null && target.kind === 'free' && this.bending ? this.bendFor(tip, point, dir, this.routeDiameter(duct)) : null;
    const curve = anchor ? fitCurve(tip.point, tip.dir, anchor.point, anchor.dir) : bend?.bend;
    const path = curve ? layoutPipe([curve], tip.point, tip.dir).stations.map((st) => st.position) : [tip.point, point];
    if (bend) {
      // The disc it turns round, centred where its radius is from.
      const across = bend.to.clone().addScaledVector(tip.dir, -bend.to.dot(tip.dir)).normalize();
      const normal = tip.dir.clone().cross(across).normalize();
      this.bendDisc.position.copy(tip.point).addScaledVector(across, bend.radius);
      this.bendDisc.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), normal);
      this.bendDisc.scale.setScalar(bend.radius);
      this.bendDisc.visible = true;
    } else {
      this.bendDisc.visible = false;
    }
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
    const described = bend
      ? `bend ${Math.round((tip.dir.angleTo(bend.to) * 180) / Math.PI)}°${name ? ` to ${name}` : ''}, radius ${Math.round(bend.radius * 1000)} mm`
      : `${name ? `${name}, ` : ''}${length}`;
    this.cb.onAim?.(target.kind === 'free' ? described : null);
    // Marked only when it would *connect*, so the highlight means something.
    this.marker.visible = target.kind !== 'free';
    this.marker.position.copy(point);
    this.colourMarker(target);
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
    if (this.bendTool) {
      this.updatePointer(e);
      if (this.bendDrag) this.dragBend(e.shiftKey);
      else this.bendTriad.hover(this.raycaster);
      return;
    }
    if (this.drawMode && !this.drag) {
      this.updatePointer(e);
      this.lastSnap = drawSnapOf(e);
      this.updateDrawPreview(this.lastSnap);
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

    // A pipe's two ends are set apart, so it tapers between them; the inlet ring sets where it starts.
    if (this.drag!.data.kind === 'inlet') {
      seg.dIn = d;
      return;
    }

    // The outlet diameter is `dIn` for a chamber, which necks back down to its throat, and `dOut` for a
    // pipe, matching `segmentDiameter(seg, 1)`. The next segment starts at it.
    if (seg.kind === 'chamber') seg.dIn = d;
    else seg.dOut = d;

    this.propagate(i);
  }

  /** Keep the duct continuous: the next segment starts where this one ends. */
  private propagate(index: number): void {
    const seg = this.pipe[index];
    const next = this.pipe[index + 1];
    if (!seg || !next) return;
    next.dIn = segmentDiameter(seg, 1);
  }

  private onPointerUp = (): void => {
    if (this.bendDrag) {
      this.endBendDrag();
      return;
    }
    if (this.triadDrag) {
      const drag = this.triadDrag;
      this.triadDrag = null;
      this.controls.enabled = true;
      if (!drag.moved) return;
      // Final authoritative push, since intermediate frames were throttled.
      if (drag.on === 'turbo') {
        const mount = this.context?.graph.turbos?.find((t) => t.id === drag.turbo);
        if (mount?.position) this.cb.onMoveTurbo?.(mount.id, mount.position, mount.rotation, true);
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
    window.removeEventListener('keydown', this.onKeyDown);
    window.removeEventListener('keyup', this.onKeyUp);
    this.bendDisc.geometry.dispose();
    this.bendTriad.dispose();
    this.bendGhost.dispose();
    (this.bendDisc.material as THREE.Material).dispose();
    for (const h of this.handles) h.geometry.dispose();
    this.pipeTriad.dispose();
    this.turboTriad.dispose();
    this.previewGeom.dispose();
    this.guideGeom.dispose();
    this.ghost.dispose();
    this.ghostPipe.geometry.dispose();
    (this.ghostPipe.material as THREE.Material).dispose();
    for (const m of [this.matHover, this.matSelected, this.matRing, this.previewMat, this.guideMat]) {
      m.dispose();
    }
  }
}

/** The way a loose pipe heads where it starts: its heading, stored off world +x. */
function turnHeadingWorld(duct: ExhaustDuct): THREE.Vector3 {
  return turnHeading(new THREE.Vector3(1, 0, 0), duct.headingYaw, duct.headingPitch);
}

function clamp(x: number, lo: number, hi: number): number {
  return x < lo ? lo : x > hi ? hi : x;
}
