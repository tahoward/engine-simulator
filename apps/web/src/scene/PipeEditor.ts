/**
 * Direct manipulation of the exhaust in 3D.
 *
 * The selected pipe has a triad (`Triad.ts`) where it starts: its arrows and squares move the junction it
 * starts from along an axis or in a plane, the pipe with it, or a loose pipe itself, and its rings turn the
 * pipe about where it starts. A bend further along has a ring of its own, which rolls it. A ring around
 * each joint sets the diameter there: drag it outward to open the pipe up, inward to choke it down. A
 * selected turbo has a triad of its own, which moves and turns it.
 *
 * Both edit the same `PipeSegment[]` the segment menu does, so the two editors cannot drift apart —
 * there is exactly one.
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
  UPRIGHT,
  type TurboSize,
} from '../model/turbo.js';
import type { Quat } from '../model/exhaustGraph.js';
import {
  attachToLooseStart,
  carryBore,
  drawnSegments,
  endsAt,
  joinDuctEnd,
  junctionAt,
  newDuctId,
  newNodeId,
  splitDuctAt,
  turboAt,
  type DuctSink,
  type DuctSource,
  type ExhaustDuct,
  type ExhaustGraph,
} from '../model/exhaustGraph.js';
import { PipeMesh, layoutPipe, turnBetween } from './PipeMesh.js';
import { TurboMesh } from './TurboMesh.js';
import {
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
  bendWhole,
  bendAnchor,
  squareArrival,
  sideArrival,
  sideLeaving,
  closesLoop,
  collectSnapTargets,
  diameterAt,
  type BendAnchor,
  swingPipe,
  pipeShape,
  type PipeShape,
  continuingDiameter,
  fitCurve,
  headingOffsetTo,
  nearestSnap,
  quantiseLength,
  routeTip,
  type SnapTarget,
} from './drawing.js';
import {
  collectorGhost,
  headerCollectorBore,
  headerOpenings,
  headerPrimaries,
  mirrorPlan,
  openingKey,
  shortestHeader,
  type HeaderPlan,
  type HeaderPrimary,
  type OpeningAt,
} from './headerTool.js';

const MIN_RADIUS = 0.006;
const MAX_RADIUS = 0.22;

/** Live geometry edits are cheap to draw but force the solver's ducts to be rebuilt, so throttle those. */
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
/** The attachment dot's radius, m, at the least, and against the bore it sits on: wider, so it shows round the pipe. */
const MARKER_RADIUS = 0.016;
const MARKER_OVER_BORE = 0.7;

/** Size of the selected segment's triad, m. */
const PIPE_TRIAD_SIZE = 0.11;

/**
 * A triad being dragged, on a pipe segment or a turbo.
 *
 * Everything is measured from where the drag began, so a move or a turn is where the pointer is now rather
 * than the sum of every frame's step.
 */
interface TriadDrag {
  on: 'pipe' | 'turbo' | 'header';
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
  /** The first segment that swings: 0 for the whole pipe, or a bend further along and what follows it. */
  from?: number;
  shape0?: PipeShape;
  /**
   * The pipes carrying on from its far end that swing round with it (`carriedOn`), each as `base` and
   * `shape0` are, and the way it set off: all a pipe that is nothing but a fitted bend has to turn.
   */
  carried?: { id: string; base: THREE.Vector3; shape0: PipeShape; heading0: THREE.Vector3 }[];
  /** What a pipe's arrows move: the junction it starts from, and the way it points, or a loose pipe. */
  startNode?: string;
  startAxis?: Vec3;
  loose?: Vec3;
  /** The last place a move was sent to, for the final commit. */
  lastPosition?: Vec3;
  /** A turbo's: which, and its rotation when the drag began. */
  turbo?: string;
  rotation0?: Quat;
  /** A header's: the way its collector pointed, and the triad's orientation, when the drag began. */
  axis0?: THREE.Vector3;
  frame0?: THREE.Quaternion;
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
  /**
   * Geometry changed. `commit` is false for intermediate frames of a drag. `edited` is the duct whose
   * shape was drawn or dragged, when it was one, for the owner to copy onto the other runners if linked.
   */
  onChange: (commit: boolean, edited?: string) => void;
  /** A handle of the duct being edited was grabbed, selecting its segment. */
  onSelect: (index: number | null) => void;
  /**
   * A click landed on a pipe or a junction — any of them, not only the duct being edited — or on nothing.
   * The owner decides what selecting it means, since switching ducts is its business, not the editor's.
   */
  onPick?: (pick: ScenePick | null) => void;
  /** A pipe segment, a junction or a turbo was right-clicked, at `x`, `y` on the page, with no tool on: it is picked first. */
  onMenu?: (pick: ScenePick, x: number, y: number) => void;
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
  /** The tool that was on — drawing, placing, bending or the header — was ended from the view. */
  onToolEnded?: () => void;
  /**
   * The junction at `node` was moved to here by the triad of a pipe starting from it; `axis` is the way it
   * points, for the first time it is moved. `commit` is false for intermediate frames of a drag.
   */
  onMoveJunction?: (node: string, position: Vec3, axis: Vec3, commit: boolean) => void;
  /** A turbo was moved or turned to here by its triad. `commit` is false for intermediate frames of a drag. */
  onMoveTurbo?: (turbo: string, position: Vec3, rotation: Quat, commit: boolean) => void;
  /** What the header being placed comes to, in words. */
  onHeaderAim?: (aim: string) => void;
  /** Build the pipes the ghost shows: each merge's plan and its pipes. */
  onApplyHeader?: (builds: Array<{ plan: HeaderPlan; primaries: HeaderPrimary[] }>) => void;
  /** The pipes' length, where it follows what is picked. */
  onHeaderLength?: (length: number) => void;
}

/**
 * The equal-length pipes being placed: where they merge, which the triad moves and turns, how long they are,
 * and which openings they run from. With `mirrored`, on an engine with two banks, the ports on the bank away
 * from the triad merge at its mirror image instead.
 */
export interface HeaderSetup {
  merge: THREE.Vector3;
  axis: THREE.Vector3;
  length: number;
  /** The openings picked, by `openingKey`. */
  picked: Set<string>;
  /** Each cylinder's bank, and the engine's middle, which one bank is the mirror image of the other in. */
  banks: number[];
  mirror: { point: THREE.Vector3; normal: THREE.Vector3 } | null;
  mirrored: boolean;
  /** The bore a pipe from a port with none on it starts at. */
  portBore: number;
  /** Whether the length was set; until it is, it is the shortest that reaches, following what is picked. */
  lengthSet: boolean;
}

/** The openings' dots: picked, and not. */
const OPENING_PICKED = 0xff8c42;
const OPENING_UNPICKED = 0x9aa3ad;

/**
 * The keys held while drawing: `free`, with Shift, joins a pipe's side square across it (`squareInto`); the
 * others join along it. A click in open space runs straight on from the opening either way.
 */
type DrawSnap = 'engine' | 'turn' | 'free';

/** Whether a route ending on `target` meets it square: a pipe's side, with Shift held. */
function squareInto(target: SnapTarget, snap: DrawSnap): boolean {
  return target.kind === 'ductSurface' && snap === 'free';
}

function drawSnapOf(e: { shiftKey: boolean; altKey: boolean }): DrawSnap {
  if (e.shiftKey) return 'free';
  return e.altKey ? 'turn' : 'engine';
}

/** The ghost of a straight run, carrying straight on from the opening it is drawn out of. */
const STRAIGHT_ON_COLOUR = new THREE.Color(0xffffff);
/** The ghost of a fitted pipe onto what it will join. */
const PREVIEW_COLOUR = 0x8cff9e;

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
/** How far, in pixels, a right-click may drag and still end a tool rather than pan the view. */
const RIGHT_CLICK_PIXELS = 5;
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
  /** The loose pipe being placed: from where it starts, along the crank (+z), as `setLoosePipe` sizes it. */
  private readonly ghostPipe = new THREE.Mesh(
    new THREE.BufferGeometry(),
    new THREE.MeshStandardMaterial({ color: 0xb9bdc3, transparent: true, opacity: 0.45, depthWrite: false }),
  );
  private readonly ghost = new TurboMesh(true);
  private ghostAt: TurboPlacement = { position: [0, 0, 0], rotation: [...IDENTITY] };
  /** Turbos are drawn this size, and put down at this height unless snapped to a pipe. Set by the owner. */
  private size: TurboSize = { scroll: 0.07, depth: 0.06, outletDia: 0.058, inletDia: 0.042 };
  turboHeight = 0;
  /** The bore a pipe drawn from a turbo's outlet starts at. Set by the owner. */
  turboOutletDia = 0.058;
  /** The turbo the owner has selected, which the turbo triad is on. */
  private selectedTurbo: string | null = null;
  /**
   * The route in progress.
   *
   * The duct is added to the graph as soon as drawing starts, rather than being assembled and inserted at
   * the end. That keeps one source of truth — the renderer draws it and the solver hears it, all while it
   * is still being drawn — instead of a second, parallel representation that has to
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
    /**
     * Out of the side of a pipe, or where two of its segments meet: the way that pipe runs there. The route's
     * fitted pipe sets off along it (`sideLeaving`).
     */
    side?: THREE.Vector3;
    /**
     * Whether it runs straight on into open space: out of an opening — a port, a turbo's outlet, a pipe's
     * open end or a loose pipe's start. From anywhere else, only a fitted pipe onto something is drawn.
     */
    open: boolean;
    /** What it was started from, for a route finished onto a port, which is drawn from there onto this. */
    start: SnapTarget;
    /** Whether its heading was set as it started, in the world's terms, which its first segment keeps. */
    preset?: true;
  } | null = null;
  /**
   * The segment the next click would add, as a see-through pipe of the bore it would be: a straight, or the
   * bend fitted into what it would join. Tinted the colour of the way it runs.
   */
  private readonly drawGhost = new PipeMesh(true);
  private readonly marker: THREE.Mesh;

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
    /** The way the straight runs, or the bend sets off, in the world. */
    dir: THREE.Vector3;
    /** The pipe's length before it is bent, m. */
    length: number;
    /** For a bend being bent again: the axis it turns about, and how far, radians, from the straight it was. */
    turn?: { axis: THREE.Vector3; angle: number };
  } | null = null;
  private bendDrag: {
    axis: THREE.Vector3;
    ring: RingFrame;
    lastAngle: number;
    turned: number;
    proposed: PipeSegment[] | null;
  } | null = null;
  private lastSnap: DrawSnap = 'engine';
  /**
   * Whether Ctrl is held, lining the triads up with the engine's axes rather than the part's; and whether
   * they are, which waits for a drag to end, since the drag keeps the axis it took hold of.
   */
  private engineFrameHeld = false;
  private engineFrameOn = false;
  /**
   * Where a right button went down: let go there, it ends the tool that is on, or with none opens the menu of
   * the segment or junction under it; dragged, it pans.
   */
  private rightDown: { x: number; y: number } | null = null;

  /** The equal-length header tool: what it is building, its triad on the collector, and the ghost. */
  private header: HeaderSetup | null = null;
  private readonly headerTriad = new Triad(PIPE_TRIAD_SIZE * 1.4);
  private readonly headerGhosts: PipeMesh[] = [];
  /** A dot on every opening the pipes could run from, and where each is. */
  private readonly openingDots: THREE.Mesh[] = [];
  private openings: OpeningAt[] = [];

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
    window.addEventListener('blur', this.onBlur);
    // Right-click ends a tool or opens a menu, so the browser's must not appear over it.
    dom.addEventListener('contextmenu', this.onContextMenu);

    this.drawGhost.group.visible = false;
    this.group.add(this.drawGhost.group);

    this.marker = new THREE.Mesh(
      new THREE.SphereGeometry(1, 16, 12),
      new THREE.MeshBasicMaterial({ color: ATTACH_MARKER, transparent: true, opacity: 0.85 }),
    );
    this.marker.visible = false;
    this.marker.renderOrder = 13;
    this.group.add(this.marker);

    this.ghost.group.visible = false;
    this.group.add(this.ghost.group);
    this.ghostPipe.visible = false;
    this.group.add(this.ghostPipe);
    this.group.add(this.pipeTriad.group, this.turboTriad.group, this.bendTriad.group, this.bendGhost.group, this.headerTriad.group);
    this.headerTriad.setVisible(false);
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
    this.turboTriad.setEngineFrame(this.engineFrameOn);
    this.group.add(this.turboTriad.group);
    this.placeTurboTriad();
  }

  // -------------------------------------------------------------------------
  // Placing turbos
  // -------------------------------------------------------------------------

  /** Start or stop placing a turbo, or a loose pipe. Stops drawing, since the two share the pointer. */
  setPlaceMode(on: boolean, kind: 'turbo' | 'pipe' = 'turbo'): void {
    if (on && this.drawMode) this.setDrawMode(false);
    if (on) this.setHeaderTool(null);
    this.placeMode = on;
    this.placeKind = kind;
    this.ghost.rebuild(this.size);
    this.ghost.group.visible = false;
    this.ghostPipe.visible = false;
    this.applyHandleVisibility();
    this.placeTurboTriad();
  }

  /** Size the loose pipe that placing puts down, as `placeLoosePipe` will: `length` long, of `dia` bore. */
  setLoosePipe(length: number, dia: number): void {
    this.ghostPipe.geometry.dispose();
    this.ghostPipe.geometry = new THREE.CylinderGeometry(dia / 2, dia / 2, length, 24)
      .rotateX(Math.PI / 2)
      .translate(0, 0, length / 2);
  }

  /**
   * Follow the pointer with the turbo being placed: on the level it is put down at, or, near an open pipe
   * end, with its inlet flange on that end and turned to take the pipe. A loose pipe being placed follows
   * it on the level of the ports.
   */
  private updateGhost(): void {
    const ctx = this.context;
    if (!ctx) return;
    if (this.placeKind === 'pipe') {
      // A loose pipe goes down level with the ports, running along the crank; its triad turns it.
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
    this.ghostAt = { position: [point.x, point.y, point.z], rotation: [...UPRIGHT] };
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
    if (this.header) this.updateHeader();
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
    if (on) this.setHeaderTool(null);
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
    if (on) this.setHeaderTool(null);
    this.bendTool = on;
    this.bendAt = null;
    this.bendDrag = null;
    this.bendTriad.setVisible(false);
    this.hidePreview();
    this.cb.onBendAim?.(null);
    this.applyHandleVisibility();
  }

  /** Whether a tool is on: drawing, placing, bending or the header. */
  get toolOn(): boolean {
    return this.drawMode || this.placeMode || this.bendTool || this.header !== null;
  }

  /**
   * End whichever tool is on, as a right-click does, and tell the owner. A route being drawn is kept,
   * ending in open air; a header not yet applied is abandoned.
   */
  exitTool(): void {
    if (!this.toolOn) return;
    if (this.route) this.finishRoute({ kind: 'mouth' });
    if (this.drawMode) this.setDrawMode(false);
    if (this.placeMode) this.setPlaceMode(false);
    if (this.bendTool) this.setBendTool(false);
    if (this.header) this.setHeaderTool(null);
    this.controls.enabled = true;
    this.cb.onToolEnded?.();
  }

  // -------------------------------------------------------------------------
  // Equal-length headers
  // -------------------------------------------------------------------------

  /**
   * Start the equal-length pipes tool on `setup`, or with `null` end it. A triad sits where they merge: its
   * arrows move it and its rings point it. Each opening has a dot, which a click picks or leaves out, and a
   * ghost shows the pipes it would build.
   */
  setHeaderTool(setup: HeaderSetup | null): void {
    if (setup) {
      if (this.drawMode) this.setDrawMode(false);
      if (this.placeMode) this.setPlaceMode(false);
      if (this.bendTool) this.setBendTool(false);
    }
    this.header = setup;
    if (this.triadDrag?.on === 'header') {
      this.triadDrag = null;
      this.controls.enabled = true;
    }
    if (setup) this.headerTriad.setOrientation(frameAlong(setup.axis));
    this.updateHeader();
    this.applyHandleVisibility();
  }

  /** The shortest the pipes can all be and reach where they merge, m: 0 with no openings picked. */
  get headerReach(): number {
    return Math.max(0, ...this.headerPlans().map((p) => shortestHeader(p)));
  }

  /** Make the pipes `length` m each. */
  setHeaderLength(length: number): void {
    if (!this.header) return;
    this.header.length = length;
    this.header.lengthSet = true;
    this.updateHeader();
  }

  /** Merge the other bank's ports at the mirror image, or all at the one place. */
  setHeaderMirrored(on: boolean): void {
    if (!this.header) return;
    this.header.mirrored = on;
    this.updateHeader();
  }

  /**
   * The merges the ghost shows. Everything picked merges at the triad; mirrored, the ports on the bank away
   * from it merge at its mirror image instead, which only ports have.
   */
  private headerPlans(): HeaderPlan[] {
    const ctx = this.context;
    const h = this.header;
    if (!ctx || !h) return [];
    const picked = this.openings.filter((o) => h.picked.has(openingKey(o.opening)));
    const plan = (openings: OpeningAt[], merge: THREE.Vector3, axis: THREE.Vector3): HeaderPlan => ({
      openings,
      merge,
      axis,
      length: h.length,
      collectorBore: headerCollectorBore(ctx.graph, openings),
    });
    // A port's bank, and a port's pipe's: anything else merges at the triad however it is mirrored.
    const bankOf = (o: OpeningAt) => {
      if (o.opening.kind === 'port') return h.banks[o.opening.cylinder] ?? 0;
      const id = o.opening.duct;
      const from = ctx.graph.ducts.find((d) => d.id === id)?.from;
      return from?.kind === 'valve' ? (h.banks[from.cylinder] ?? 0) : -1;
    };
    const ports = picked.filter((o) => bankOf(o) >= 0);
    if (!h.mirrored || !h.mirror || ports.length === 0) return picked.length > 0 ? [plan(picked, h.merge, h.axis)] : [];
    // The triad's side: the bank whose picked ports are nearer it on average.
    const near = (bank: number) => {
      const own = ports.filter((o) => bankOf(o) === bank);
      return own.length > 0 ? own.reduce((a, o) => a + o.point.distanceTo(h.merge), 0) / own.length : Infinity;
    };
    const here = near(0) <= near(1) ? 0 : 1;
    const there = picked.filter((o) => bankOf(o) === 1 - here);
    const main = plan(picked.filter((o) => bankOf(o) !== 1 - here), h.merge, h.axis);
    const plans = main.openings.length > 0 ? [main] : [];
    if (there.length > 0) plans.push({ ...mirrorPlan(main, h.mirror, there), collectorBore: headerCollectorBore(ctx.graph, there) });
    return plans;
  }

  /** Build what the ghost shows, and end the tool. */
  applyHeader(): void {
    if (!this.header) return;
    const builds = this.headerPlans().map((plan) => ({ plan, primaries: headerPrimaries(plan) }));
    this.setHeaderTool(null);
    if (builds.length > 0) this.cb.onApplyHeader?.(builds);
  }

  /** Put the triad where the pipes merge, a dot on each opening, ghost the pipes, and say what they come to. */
  private updateHeader(): void {
    const ctx = this.context;
    const h = this.header;
    this.headerTriad.setVisible(!!h && !!ctx);
    if (!h || !ctx) {
      for (const g of this.headerGhosts) g.group.visible = false;
      for (const d of this.openingDots) d.visible = false;
      this.openings = [];
      return;
    }
    this.headerTriad.setMoveOrigin(h.merge);
    this.headerTriad.setRotateOrigin(h.merge);

    this.openings = headerOpenings(ctx.graph, ctx.placement, ctx.ports, h.portBore);
    while (this.openingDots.length < this.openings.length) {
      const dot = new THREE.Mesh(
        new THREE.SphereGeometry(1, 16, 12),
        new THREE.MeshBasicMaterial({ color: OPENING_UNPICKED, transparent: true, opacity: 0.9, depthTest: false }),
      );
      dot.renderOrder = 14;
      this.openingDots.push(dot);
      this.group.add(dot);
    }
    this.openingDots.forEach((dot, i) => {
      const o = this.openings[i];
      dot.visible = !!o;
      if (!o) return;
      dot.position.copy(o.point);
      dot.scale.setScalar(Math.max(o.bore * 0.35, 0.008));
      (dot.material as THREE.MeshBasicMaterial).color.setHex(h.picked.has(openingKey(o.opening)) ? OPENING_PICKED : OPENING_UNPICKED);
    });
    // Picks that are no longer openings — a pipe drawn on the port, say — are not picked.
    const live = new Set(this.openings.map((o) => openingKey(o.opening)));
    for (const key of [...h.picked]) if (!live.has(key)) h.picked.delete(key);
    if (!h.lengthSet) {
      h.length = this.headerReach;
      this.cb.onHeaderLength?.(h.length);
    }

    const plans = this.headerPlans();
    const pipes: Array<{ segments: PipeSegment[]; origin: THREE.Vector3; heading: THREE.Vector3 }> = [];
    for (const plan of plans) {
      headerPrimaries(plan).forEach((p, i) => {
        const o = plan.openings[i]!;
        pipes.push({ segments: p.segments, origin: o.point, heading: o.dir });
      });
      pipes.push({ segments: collectorGhost(plan), origin: plan.merge, heading: plan.axis });
    }
    while (this.headerGhosts.length < pipes.length) {
      const g = new PipeMesh(true);
      this.headerGhosts.push(g);
      this.group.add(g.group);
    }
    this.headerGhosts.forEach((g, i) => {
      const pipe = pipes[i];
      g.group.visible = !!pipe;
      if (pipe) g.rebuild(pipe.segments, pipe.origin, pipe.heading);
    });

    const mm = (m: number) => `${Math.round(m * 1000)} mm`;
    const count = plans.reduce((a, p) => a + p.openings.length, 0);
    const reach = this.headerReach;
    this.cb.onHeaderAim?.(
      count === 0
        ? 'Click the openings to run pipes from'
        : h.length < reach - 5e-4
          ? `Too short to reach: the furthest needs ${mm(reach)}`
          : `${count} pipes of ${mm(h.length)} · the shortest that reaches is ${mm(reach)} · click an opening to add or leave it out`,
    );
  }

  /** The opening whose dot is under the pointer, within a few pixels, if any. */
  private openingUnderPointer(): OpeningAt | null {
    const rect = this.dom.getBoundingClientRect();
    let best: { d: number; o: OpeningAt } | null = null;
    for (const o of this.openings) {
      const p = o.point.clone().project(this.camera);
      if (p.z > 1) continue;
      const d = Math.hypot(((p.x - this.pointer.x) * rect.width) / 2, ((p.y - this.pointer.y) * rect.height) / 2);
      if (d <= SNAP_PIXELS && (!best || d < best.d)) best = { d, o };
    }
    return best?.o ?? null;
  }

  /** Pick an opening for the pipes, or leave it out. */
  private toggleOpening(o: OpeningAt): void {
    const h = this.header;
    if (!h) return;
    const key = openingKey(o.opening);
    if (h.picked.has(key)) h.picked.delete(key);
    else h.picked.add(key);
    this.updateHeader();
  }

  /** The header's triad: its arrows move the collector, its rings turn the way it points. */
  private dragHeaderTriad(drag: TriadDrag, snap: boolean): void {
    const h = this.header;
    if (!h || !drag.axis0 || !drag.frame0) return;
    if (drag.handle.kind === 'ring') {
      let turn = this.triadTurn(drag);
      if (turn === null) return;
      if (snap) turn = snapTurnToEngine(drag.axis0, turn, drag.axis, (TRIAD_TURN_DEG * Math.PI) / 180);
      const q = new THREE.Quaternion().setFromAxisAngle(drag.axis, turn);
      h.axis = drag.axis0.clone().applyQuaternion(q).normalize();
      this.headerTriad.setOrientation(q.multiply(drag.frame0));
    } else {
      const at = this.triadMove(drag, snap);
      if (!at) return;
      h.merge = at;
    }
    drag.moved = true;
    this.updateHeader();
  }

  /**
   * Put the bend tool's rings where the straight or bend under the pointer starts, if it is one that can be
   * bent: a pipe's own, not a can, or the bend fitted into what it joins. A bend is bent again from the
   * straight it was, the ring in its own plane starting where it is now.
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
    return this.placeBendAt(duct, index);
  }

  /**
   * Switch the bend tool on with its rings on segment `index` of pipe `ductId`, as clicking it would, from its
   * menu. Returns whether it is one that can be bent.
   */
  startBend(ductId: string, index: number): boolean {
    const duct = this.context?.graph.ducts.find((d) => d.id === ductId);
    if (!duct) return false;
    if (!this.bendTool) this.setBendTool(true);
    return this.placeBendAt(duct, index);
  }

  /** Put the bend tool's rings where segment `index` of `duct` starts, if it can be bent. */
  private placeBendAt(duct: ExhaustDuct, index: number): boolean {
    const ctx = this.context;
    if (!ctx) return false;
    const seg = duct.segments[index];
    const locked = lockedFrom(duct);
    const place = ctx.placement.ducts.get(duct.id);
    if (!seg || !place || seg.kind === 'chamber' || (locked !== null && index >= locked)) return false;
    const swept = layoutPipe(duct.segments, place.origin, place.heading);
    const start = index === 0 ? place.origin : swept.joints[index - 1]!;
    const end = swept.joints[index]!;
    // The way it sets off: along a straight, and a bend's tangent where it starts.
    const dir = seg.curve
      ? swept.stations.find((st) => st.segment === index)!.direction.clone().normalize()
      : end.clone().sub(start).normalize();
    let turn: { axis: THREE.Vector3; angle: number } | undefined;
    if (seg.curve) {
      const out = swept.jointDirections[index]!.clone().normalize();
      const axis = dir.clone().cross(out);
      if (axis.lengthSq() > 1e-12) turn = { axis: axis.normalize(), angle: dir.angleTo(out) };
    }
    this.bendAt = { ductId: duct.id, index, dir, length: duct.segments.reduce((a, s) => a + s.length, 0), ...(turn ? { turn } : {}) };
    this.bendTriad.setOrientation(frameAlong(dir));
    this.bendTriad.setRotateOrigin(start);
    this.bendTriad.setMoveOrigin(start);
    this.bendTriad.setVisible(true);
    this.cb.onBendAim?.(seg.curve ? 'Drag a ring to bend it again, or back to straight' : 'Drag a ring to bend the pipe in its plane');
    return true;
  }

  private beginBendDrag(handle: TriadHandle): void {
    const ring = this.bendTriad.ring(handle.axis);
    const axis = this.bendTriad.axisDir(handle.axis);
    // A bend in this ring's plane starts from how far it turns already; in the other, from straight.
    const turn = this.bendAt?.turn;
    const along = turn ? turn.axis.dot(axis) : 0;
    this.bendDrag = {
      axis,
      ring,
      lastAngle: this.ringAngleNow(this.bendTriad.rotateOrigin, ring) ?? 0,
      turned: turn && Math.abs(along) > 0.99 ? Math.sign(along) * turn.angle : 0,
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
    // From the straight it was, if it is a bend already: it keeps its length, its corner, and its bores.
    const straight = seg.curve ? makeSegment({ ...seg, curve: undefined }) : seg;
    const bent = bendWhole(straight, at.dir, drag.axis, angle, tightest);
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
    // A straight left straight is no change; a bend taken back to straight is.
    const was = duct?.segments[at.index];
    if (!duct || (!drag.proposed[at.index]?.curve && !was?.curve)) return;
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
      const was = JSON.parse(this.route.snapshot) as { ducts: ExhaustDuct[]; junctions: NonNullable<ExhaustGraph['junctions']> };
      ctx.graph.ducts = was.ducts;
      if (was.junctions.length > 0) ctx.graph.junctions = was.junctions;
      else delete ctx.graph.junctions;
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

  /**
   * The attachment dot: violet on a junction, so it reads apart from a pipe end or port's yellow, and wider
   * than the pipe it is on, so the pipe does not swallow it.
   */
  private styleMarker(target: SnapTarget): void {
    (this.marker.material as THREE.MeshBasicMaterial).color.setHex(
      target.kind === 'node' ? JUNCTION_MARKER : ATTACH_MARKER,
    );
    const bore =
      target.kind === 'port'
        ? this.portDiameter
        : target.kind === 'node'
          ? this.widestAt(target.node)
          : (this.connectionAnchor(target, this.route?.ductId ?? '')?.dia ?? 0);
    this.marker.scale.setScalar(Math.max(MARKER_RADIUS, bore * MARKER_OVER_BORE));
  }

  /** The bore of the widest pipe meeting at `node` where it meets it, m: what a junction's dot shows round. */
  private widestAt(node: string): number {
    const ctx = this.context;
    if (!ctx) return 0;
    let dia = 0;
    for (const e of endsAt(ctx.graph, node)) {
      const seg = e.end === 'outlet' ? e.duct.segments.at(-1) : e.duct.segments[0];
      if (seg) dia = Math.max(dia, segmentDiameter(seg, e.end === 'outlet' ? 1 : 0));
    }
    return dia;
  }

  /**
   * The way a pipe drawn from the junction at `node` sets off, straight on: out along the one straight pipe
   * there, away from it, as a pipe carries on from a straight one. That is the way it finishes, where it ends
   * there, and back out of its start, where it leaves. Only where two pipes meet, the straight one and a bend
   * joining it: at a merge of more, as a header's collector is, the junction's own way is taken instead, and
   * where there is not just one straight pipe.
   */
  private straightOff(node: string): THREE.Vector3 | null {
    const ctx = this.context;
    if (!ctx) return null;
    const ends = endsAt(ctx.graph, node);
    if (ends.length !== 2 || turboAt(ctx.graph, node)) return null;
    const ways: THREE.Vector3[] = [];
    for (const e of ends) {
      const place = ctx.placement.ducts.get(e.duct.id);
      if (!place || e.duct.segments.length === 0) continue;
      const swept = layoutPipe(e.duct.segments, place.origin, place.heading);
      if (e.end === 'outlet' && !e.duct.fitted && !e.duct.segments.at(-1)!.curve) ways.push(swept.jointDirections.at(-1)!.clone());
      if (e.end === 'inlet' && !e.duct.segments[0]!.curve) ways.push(swept.stations[0]!.direction.clone().negate());
    }
    return ways.length === 1 ? ways[0]!.normalize() : null;
  }

  private hidePreview(): void {
    this.drawGhost.group.visible = false;
    this.marker.visible = false;
    this.cb.onAim?.(null);
  }

  /** Handles are a nuisance while drawing: they sit exactly where the route is being aimed. */
  private applyHandleVisibility(): void {
    const hide = this.toolOn;
    for (const h of this.handles) h.visible = !hide;
    this.pipeTriad.setVisible(!hide && this.pipeTriadAt !== null);
  }

  /**
   * Where the selected pipe's triad goes: where the pipe starts, which it turns about, or where the
   * selected bend starts, and `from` that bend, which the rest of the pipe turns about. `root`, for the far
   * half of a pipe a tee split, the pipe it carries on from, which its ring rolls, and this half with it.
   */
  private pipeTriadAt: { start: THREE.Vector3; from: number; root?: string } | null = null;

  private onContextMenu = (e: MouseEvent): void => {
    e.preventDefault();
  };

  /** The bore the next segment of the route is drawn at: the bore the pipe ends at, or starts at. */
  private routeDiameter(duct: ExhaustDuct): number {
    const ctx = this.context!;
    const fromTurbo = duct.from.kind === 'node' && !!ctx.graph.turbos?.some((t) => t.node === (duct.from as { node: string }).node);
    return continuingDiameter(
      duct.segments.length > 0 ? duct : fromTurbo ? null : this.startingDuct(duct),
      this.startingDiameter(duct),
    );
  }

  /** Hold the triads in the engine's frame, or the parts', as Ctrl is held. */
  private holdEngineFrame(on: boolean): void {
    this.engineFrameHeld = on;
    if (!this.triadDrag) this.applyEngineFrame();
  }

  private applyEngineFrame(): void {
    if (this.engineFrameOn === this.engineFrameHeld) return;
    this.engineFrameOn = this.engineFrameHeld;
    for (const t of [this.pipeTriad, this.turboTriad, this.headerTriad]) t.setEngineFrame(this.engineFrameOn);
  }

  private onKeyUp = (e: KeyboardEvent): void => {
    if (e.key === 'Control') this.holdEngineFrame(false);
    this.resnapDraw(e);
  };

  /** Show at once what Shift or Alt, pressed or let go while drawing, does to the next segment. */
  private resnapDraw(e: KeyboardEvent): void {
    if (!this.drawMode || !this.route || (e.key !== 'Shift' && e.key !== 'Alt')) return;
    this.lastSnap = drawSnapOf(e);
    this.updateDrawPreview(this.lastSnap);
  }

  private onBlur = (): void => this.holdEngineFrame(false);

  private onKeyDown = (e: KeyboardEvent): void => {
    if (e.key === 'Control') this.holdEngineFrame(true);
    this.resnapDraw(e);
    const typing = e.target instanceof HTMLInputElement || e.target instanceof HTMLSelectElement || e.target instanceof HTMLTextAreaElement;
    if (this.header && !typing && e.key === 'Enter') {
      this.applyHeader();
      e.preventDefault();
      return;
    }
    // Escape abandons a route being drawn; with none, it ends the tool.
    if (e.key === 'Escape' && !typing && this.toolOn) {
      if (this.route) this.cancelRoute();
      else this.exitTool();
      e.preventDefault();
      return;
    }
    if (!this.drawMode || !this.route) return;
    if (e.key === 'Enter') {
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

    // Nothing a route ending there would leave with no way out to the air.
    const route = this.route;
    const trapped = (t: SnapTarget): boolean => !!route && closesLoop(ctx.graph, route.ductId, t);
    const point = nearestSnap(
      collectSnapTargets(ctx.graph, ctx.placement, ctx.ports).filter(
        (t) => !(route && (t.kind === 'ductEnd' || t.kind === 'ductSurface') && t.duct === route.ductId) && !trapped(t) && this.endable(t),
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
      const locked = lockedFrom(duct);
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
      const surface: SnapTarget = {
        kind: 'ductSurface',
        point: (station?.position ?? hit.point).clone(),
        duct: duct.id,
        // Split where the station is, so the junction that makes is where the ghost went.
        x: station?.x ?? st.x,
        ...(station ? { dir: station.direction.clone() } : {}),
      };
      if (trapped(surface) || !this.endable(surface)) continue;
      return surface;
    }

    const free = this.freePoint(tip);
    return free ? { kind: 'free', point: free } : null;
  }

  /**
   * Whether the route being drawn can end on this target. Not back where it started, nor on the port of the
   * cylinder whose pipe it is, or is drawn from.
   */
  private endable(target: SnapTarget): boolean {
    const route = this.route;
    const ctx = this.context;
    if (!route || !ctx) return true;
    const duct = ctx.graph.ducts.find((d) => d.id === route.ductId);
    if (target.kind === 'node') return !(duct?.from.kind === 'node' && duct.from.node === target.node);
    if (target.kind === 'port') {
      const own = (id: string | undefined) => {
        const d = ctx.graph.ducts.find((x) => x.id === id);
        return d?.from.kind === 'valve' && d.from.cylinder === target.cylinder;
      };
      const startDuct = 'duct' in route.start ? route.start.duct : undefined;
      return !own(route.ductId) && !own(startDuct) && route.start.kind !== 'port';
    }
    return true;
  }

  /**
   * Whether a route can start from this target: not a turbo's inlet, which a route may only end on, the side
   * of a pipe only where it has a way along it to set off by, and a loose pipe's start only where it runs
   * to open air at its other end, through any tees on it, so it can be turned round to carry on from there.
   */
  private startable(target: SnapTarget | null): boolean {
    if (!target || target.kind === 'free' || target.kind === 'turboInlet') return false;
    if (target.kind === 'ductSurface') return !!target.dir;
    if (target.kind === 'looseStart') return !!this.context?.graph.ducts.some((d) => d.id === target.duct && d.from.kind === 'free');
    return true;
  }

  /**
   * Start a route.
   *
   * From a port it is a new runner, replacing whatever the cylinder had. From a junction it is a new pipe
   * leaving it. From the open end of a pipe it *continues that pipe* — the same duct, with segments added —
   * since a pipe carrying on is one pipe, not two joined end to end. From the side of a pipe it is a
   * branch: the pipe is split there, as it is when something is drawn into its side, and the new pipe leaves
   * the junction that makes, in a bend off the pipe (`route.side`).
   */
  private beginRoute(target: SnapTarget): void {
    const ctx = this.context;
    if (!ctx) return;
    const snapshot = JSON.stringify({ ducts: ctx.graph.ducts, junctions: ctx.graph.junctions ?? [] });

    if (target.kind === 'ductEnd') {
      const duct = ctx.graph.ducts.find((d) => d.id === target.duct);
      const place = ctx.placement.ducts.get(target.duct);
      if (!duct || !place) return;
      this.route = { ductId: duct.id, place, snapshot, base: duct.segments.length, open: true, start: target };
      this.cb.onDrawing?.(true);
      return;
    }

    let from: DuctSource;
    let place: DuctPlacement;
    let side: THREE.Vector3 | undefined;
    // A heading already set, in the world's terms, which its first segment keeps.
    let heading: THREE.Vector3 | null = null;
    // Whether it may run straight on into open space, from an opening, or only onto something.
    let open = false;
    if (target.kind === 'port') {
      from = { kind: 'valve', cylinder: target.cylinder };
      place = { origin: target.point.clone(), heading: target.dir.clone() };
      open = true;
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
      open = true;
    } else if (target.kind === 'looseStart') {
      // Out of a loose pipe's start, which stays where it is and the way it runs: a junction is fixed there,
      // which it now starts from, and the route leaves it back the way the pipe runs.
      const loose = ctx.graph.ducts.find((d) => d.id === target.duct);
      const at = ctx.placement.ducts.get(target.duct);
      if (!loose || !at || loose.from.kind !== 'free' || loose.segments.length === 0) return;
      const runs = layoutPipe(loose.segments, at.origin, at.heading).stations[0]!.direction.clone();
      const node = newNodeId(ctx.graph);
      (ctx.graph.junctions ??= []).push({ node, position: [...loose.from.position], axis: [runs.x, runs.y, runs.z] });
      const turn = turnBetween(new THREE.Vector3(1, 0, 0), at.heading);
      loose.from = { kind: 'node', node };
      loose.headingYaw = turn.yaw;
      loose.headingPitch = turn.pitch;
      loose.headingFrame = 'world';
      from = { kind: 'node', node };
      heading = runs.negate();
      place = { origin: at.origin.clone(), heading: heading.clone() };
      open = true;
    } else if (target.kind === 'node') {
      const joint = ctx.placement.joints.get(target.node);
      from = { kind: 'node', node: target.node };
      // A junction nothing leaves yet is an opening, the pipes meeting there ending in open air: a straight
      // runs out of it the way they come in (`bendAnchor`). Anything else is not, and only a fitted pipe onto
      // something leaves it, along the straight pipe there.
      open = !endsAt(ctx.graph, target.node).some((e) => e.end === 'inlet');
      heading = open
        ? (bendAnchor(ctx.graph, ctx.placement, target.node, '')?.dir.clone() ?? (joint ? joint.axis.clone() : this.heading.clone()))
        : (this.straightOff(target.node) ?? (joint ? joint.axis.clone() : this.heading.clone()));
      place = { origin: target.point.clone(), heading: heading.clone() };
    } else if (target.kind === 'ductSurface' && target.dir) {
      // The side of a pipe, or where two of its segments meet: split there, and only a fitted pipe onto
      // something, setting off along the pipe (`sideLeaving`).
      const node = splitDuctAt(ctx.graph, target.duct, target.x);
      if (!node) return;
      from = { kind: 'node', node };
      // Its heading is turned off the way the pipe runs there, which the layout takes the junction's to be.
      side = target.dir.clone().normalize();
      place = { origin: target.point.clone(), heading: side.clone() };
    } else {
      return;
    }

    const id = newDuctId(ctx.graph, 'drawn');
    const along = heading ? turnBetween(new THREE.Vector3(1, 0, 0), heading) : null;
    ctx.graph.ducts.push({
      id,
      segments: [],
      from,
      to: { kind: 'mouth' },
      ...(along ? { headingYaw: along.yaw, headingPitch: along.pitch, headingFrame: 'world' as const } : {}),
    });
    this.route = { ductId: id, place, snapshot, base: 0, open, start: target, ...(side ? { side } : {}), ...(heading ? { preset: true as const } : {}) };
    this.cb.onDrawing?.(true);
    this.cb.onChange(true, id);
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
    this.cb.onChange(true, duct.id);
  }

  /**
   * Extend the route, or connect it to what `target` is.
   *
   * Onto something — another pipe's end, side or segment joint, a junction, a turbo's inlet, or a port — it
   * ends in one bend fitted to it. Into open space it runs straight on, from an opening, as far along the way
   * it points as the pointer is; from anything else it does nothing, since from there only a fitted pipe
   * onto something is drawn.
   */
  private extendRoute(target: SnapTarget, snap: DrawSnap): void {
    const ctx = this.context;
    if (!ctx || !this.route) return;
    const duct = ctx.graph.ducts.find((d) => d.id === this.route!.ductId);
    if (!duct) return;
    if (target.kind === 'port') {
      this.drawToPort(target);
      return;
    }

    const routeEnd = routeTip(duct.segments, this.route.place);
    const dia = this.routeDiameter(duct);
    const tip = this.setOff(duct, routeEnd, target.point);

    /**
     * Joining something ends the route in one smooth bend, fitted to arrive along what it joins: square
     * into a turbo's flange, beside another pipe's end, into the flow along a pipe's side, or along a
     * junction's axis. See `bendAnchor`. Into a pipe's side with Shift held, square across it instead.
     */
    const square = squareInto(target, snap);
    const anchor = target.kind === 'free' ? null : this.connectionAnchor(target, duct.id, tip, square);
    if (anchor) {
      if (anchor.point.distanceTo(tip.point) < MIN_DRAW_LENGTH) return;
      if (duct.segments.length === 0 && !this.route.preset) {
        // Straight out of where it starts, or along the pipe it branches from: the bend does the turning.
        const turn = this.route.side ? headingOffsetTo(this.route.place.heading, tip.dir) : { yaw: 0, pitch: 0 };
        duct.headingYaw = turn.yaw;
        duct.headingPitch = turn.pitch;
      }
      // Tapering from the bore it leaves at to the bore of what it joins, so it matches at both.
      duct.segments.push(fitCurve(tip.point, tip.dir, anchor.point, anchor.dir, { dIn: dia, dOut: anchor.dia }));
      duct.fitted = true;
      if (square) duct.square = true;
      else delete duct.square;
      const node = this.connect(target);
      if (node) {
        this.finishRoute({ kind: 'node', node });
        return;
      }
      duct.segments.pop();
      delete duct.fitted;
      delete duct.square;
      return;
    }

    // Into open space: straight on from an opening, and nothing from anywhere else.
    if (target.kind !== 'free' || !this.route.open) return;
    const length = this.alongOpening(tip);
    if (length < MIN_DRAW_LENGTH) return;
    if (duct.segments.length === 0 && !this.route.preset) {
      duct.headingYaw = 0;
      duct.headingPitch = 0;
    }
    duct.segments.push(makeSegment({ kind: 'pipe', length, dIn: dia, dOut: dia }));
    this.cb.onChange(true, duct.id);
  }

  /**
   * The route finished onto a cylinder's port: as a pipe drawn from that port onto where the route stands —
   * the open end it has got to, or where it started if it drew nothing — its bend fitted into the other end.
   * That becomes the cylinder's pipe, replacing whatever it had.
   */
  private drawToPort(port: Extract<SnapTarget, { kind: 'port' }>): void {
    const route = this.route!;
    const ctx = this.context!;
    const duct = ctx.graph.ducts.find((d) => d.id === route.ductId);
    if (!duct || route.start.kind === 'port') return;
    let onto: SnapTarget | undefined;
    if (duct.segments.length > route.base) {
      this.finishRoute({ kind: 'mouth' });
      const now = this.context!;
      onto = collectSnapTargets(now.graph, now.placement, now.ports).find((t) => t.kind === 'ductEnd' && t.duct === duct.id);
    } else {
      onto = route.start;
      this.cancelRoute();
    }
    if (!onto) return;
    this.beginRoute(port);
    if (!this.route) return;
    this.extendRoute(onto, 'engine');
    // Could not be joined there: leave things as they were before the port was drawn from.
    if (this.route) this.cancelRoute();
  }

  /**
   * How far the route runs straight on from `tip` towards the pointer, m: along the way it points, to where
   * the pointer's ray passes nearest that line, in the drawing's length steps.
   */
  private alongOpening(tip: { point: THREE.Vector3; dir: THREE.Vector3 }): number {
    const far = tip.point.clone().addScaledVector(tip.dir, 3);
    const onLine = new THREE.Vector3();
    this.raycaster.ray.distanceSqToSegment(tip.point, far, undefined, onLine);
    return quantiseLength(onLine.distanceTo(tip.point), LENGTH_GRID_M);
  }

  /**
   * Where the route's next segment sets off from, and which way: from where it ends, the way it is going,
   * but for its first out of the side of a pipe, which sets off along the pipe towards `point`, whichever
   * way along it that is (`sideLeaving`).
   */
  private setOff(
    duct: ExhaustDuct,
    tip: { point: THREE.Vector3; dir: THREE.Vector3 },
    point: THREE.Vector3,
  ): { point: THREE.Vector3; dir: THREE.Vector3 } {
    if (!this.route?.side || duct.segments.length > 0) return tip;
    return { point: tip.point, dir: sideLeaving(this.route.side, tip.point, point) };
  }

  /**
   * Where a route ending on `target` bends in to, and the way it arrives, or `null` where it has nothing
   * fixed to arrive along and ends in a corner, as a free point does. `square`, into a pipe's side across
   * it from the route's `tip`, rather than along its flow.
   */
  private connectionAnchor(
    target: SnapTarget,
    ductId: string,
    tip?: { point: THREE.Vector3; dir: THREE.Vector3 },
    square = false,
  ): BendAnchor | null {
    const ctx = this.context;
    if (!ctx) return null;
    switch (target.kind) {
      case 'turboInlet':
      case 'looseStart':
        return { point: target.point.clone(), dir: target.dir.clone(), dia: target.dia };
      case 'ductSurface': {
        const other = ctx.graph.ducts.find((d) => d.id === target.duct);
        if (!target.dir || !other) return null;
        const dir = !tip
          ? target.dir.clone()
          : square
            ? squareArrival(target.point, target.dir, tip.point, tip.dir)
            : sideArrival(target.point, target.dir, tip.point, tip.dir);
        return { point: target.point.clone(), dir, dia: diameterAt(other.segments, target.x) };
      }
      case 'ductEnd': {
        const other = ctx.graph.ducts.find((d) => d.id === target.duct);
        const place = ctx.placement.ducts.get(target.duct);
        if (!other || !place || other.segments.length === 0) return null;
        const swept = layoutPipe(other.segments, place.origin, place.heading);
        // A junction at its end, which the bend comes in to along the pipe (`bendAnchor`), as it will once joined.
        return {
          point: swept.joints.at(-1)!.clone(),
          dir: swept.jointDirections.at(-1)!.clone(),
          dia: segmentDiameter(other.segments.at(-1)!, 1),
        };
      }
      case 'node':
        return bendAnchor(ctx.graph, ctx.placement, target.node, ductId) ?? this.fixedAnchor(target.node, ductId);
      default:
        return null;
    }
  }

  /**
   * Where a route bends in to a junction that is only where the pipes at it end, fixed where it is now, as
   * joining it fixes it (`connect`): so every pipe into it bends in to meet it, and it stays where they meet
   * as they move. `null` where it has no place of its own to be fixed at, or is a turbo's.
   */
  private fixedAnchor(node: string, ductId: string): BendAnchor | null {
    const ctx = this.context!;
    const joint = ctx.placement.joints.get(node);
    if (!joint || junctionAt(ctx.graph, node) || turboAt(ctx.graph, node)) return null;
    const graph = structuredClone(ctx.graph);
    (graph.junctions ??= []).push({ node, position: [joint.centre.x, joint.centre.y, joint.centre.z], axis: [joint.axis.x, joint.axis.y, joint.axis.z] });
    return bendAnchor(graph, ctx.placement, node, ductId);
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
      case 'node': {
        // A junction only where the pipes at it end is fixed where it is, so they all bend in to meet it.
        const joint = ctx.placement.joints.get(target.node);
        const free = !junctionAt(ctx.graph, target.node) && !turboAt(ctx.graph, target.node);
        if (free && joint && !bendAnchor(ctx.graph, ctx.placement, target.node, this.route!.ductId)) {
          (ctx.graph.junctions ??= []).push({
            node: target.node,
            position: [joint.centre.x, joint.centre.y, joint.centre.z],
            axis: [joint.axis.x, joint.axis.y, joint.axis.z],
          });
        }
        return target.node;
      }
      case 'looseStart':
        return attachToLooseStart(ctx.graph, this.route!.ductId, target.duct, [target.dir.x, target.dir.y, target.dir.z]);
      case 'ductSurface':
        return splitDuctAt(ctx.graph, target.duct, target.x);
      case 'ductEnd':
        // A junction at the pipe's end, which stays where that end is as the pipe moves: nothing is turned
        // round, and the bend drawn into it comes in along the pipe (`bendAnchor`).
        return joinDuctEnd(ctx.graph, target.duct);
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
   * Where the edited duct's segments stop being editable: its fitted bend, and any swing before it, joined
   * at both ends and fitted rather than drawn. `null` when all of it is. Set by the owner.
   */
  lockedFrom: number | null = null;

  /** Header diameter to start a runner at. Set by the owner from the engine's port. */
  portDiameter = 0.042;

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
    // No handles on a fitted bend: it is fitted to both its ends, not drawn.
    const editable = Math.min(this.lockedFrom ?? layout.joints.length, layout.joints.length);
    for (let i = 0; i < editable; i++) {
      this.addRing(layout.joints[i]!, layout.jointDirections[i]!, layout.jointRadii[i]!, { kind: 'ring', segment: i });
    }

    /**
     * The selected pipe's triad, where the pipe it is part of starts: the first of the halves a tee split it
     * into (`runRoot`), which never moves, since nothing is turned round to join it. Its arrows move it, and
     * its rings swing it every way, the pipes carried on from it and the bends joining it following. On a
     * port, or a turbo's outlet, it is held there, and only rolls, about the way it leaves. A pipe that is
     * nothing but a bend fitted to what it joins is fitted, not drawn, and has none.
     */
    const graph = this.context?.graph;
    const duct = graph?.ducts.find((d) => d.segments === this.pipe);
    this.pipeTriadAt = null;
    const root = duct && drawnSegments(duct).length > 0 ? this.runRoot(duct) : null;
    const rootAt = root ? this.context?.placement.ducts.get(root.id) : undefined;
    if (this.selected !== null && root && rootAt && root.segments.length > 0) {
      const opening = layoutPipe(root.segments, rootAt.origin, rootAt.heading).stations[0]!.direction.clone().normalize();
      const held = root.from.kind === 'valve' || (root.from.kind === 'node' && !!graph?.turbos?.some((t) => t.node === (root.from as { node: string }).node));
      this.pipeTriadAt = { start: rootAt.origin.clone(), from: 0, root: root.id };
      this.pipeTriad.setMoveOrigin(rootAt.origin);
      this.pipeTriad.setRotateOrigin(rootAt.origin);
      this.pipeTriad.setOrientation(frameAlong(opening));
      this.pipeTriad.setRingsOwn(true);
      this.pipeTriad.showMoves(!held);
      this.pipeTriad.hideRing(0, false);
      this.pipeTriad.hideRing(1, held);
      this.pipeTriad.hideRing(2, held);
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
    if (e.button === 2) {
      // Acted on once the button is let go without dragging, which pans the view.
      this.rightDown = { x: e.clientX, y: e.clientY };
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

    if (this.header) {
      // Only the triad: anywhere else turns the view, to see the header from all round.
      const handle = this.headerTriad.pick(this.raycaster);
      const opening = handle ? null : this.openingUnderPointer();
      if (handle) this.beginTriadDrag('header', handle);
      else if (opening) this.toggleOpening(opening);
      if (handle || opening) e.preventDefault();
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
  private beginTriadDrag(on: TriadDrag['on'], handle: TriadHandle): void {
    const triad = on === 'pipe' ? this.pipeTriad : on === 'header' ? this.headerTriad : this.turboTriad;
    const moveOrigin = triad.moveOrigin;
    const rotateOrigin = triad.rotateOrigin;
    const axis = triad.axisDir(handle.axis, handle.kind);
    const ring = triad.ring(handle.axis, handle.kind);
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

    if (on === 'header') {
      if (!this.header) return;
      drag.axis0 = this.header.axis.clone();
      drag.frame0 = frameAlong(this.header.axis);
    } else if (on === 'pipe') {
      const ctx = this.context;
      // The whole pipe the selected one is part of, from where it starts (`runRoot`).
      const rootId = this.pipeTriadAt?.root;
      const duct = rootId ? ctx?.graph.ducts.find((d) => d.id === rootId) : undefined;
      const place = duct ? ctx?.placement.ducts.get(duct.id) : undefined;
      const segments = duct ? drawnSegments(duct) : [];
      if (!ctx || !duct || !place || segments.length === 0) return;
      drag.ductId = duct.id;
      drag.drawn = segments.length;
      drag.from = 0;
      drag.shape0 = pipeShape(segments, place.heading);
      drag.base = this.headingBase(duct);
      drag.carried = this.carriedOn(duct).flatMap((c) => {
        const place = ctx.placement.ducts.get(c.id);
        if (!place) return [];
        const base = this.headingBase(c);
        return [{ id: c.id, base, shape0: pipeShape(drawnSegments(c), place.heading), heading0: place.heading.clone() }];
      });
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
    else if (drag.on === 'header') this.dragHeaderTriad(drag, snap);
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
   * The pipe `duct` is part of, from where it starts: back through each junction a tee split it at
   * (`ExhaustDuct.continues`) to the first half, which the triad is on.
   */
  private runRoot(duct: ExhaustDuct): ExhaustDuct {
    const graph = this.context?.graph;
    if (!graph) return duct;
    let root = duct;
    const seen = new Set([duct.id]);
    while (root.from.kind === 'node' && root.continues) {
      const node = root.from.node;
      const before = graph.ducts.find((d) => d.id === root.continues);
      if (!before || seen.has(before.id) || before.to.kind !== 'node' || before.to.node !== node || before.fitted) break;
      seen.add(before.id);
      root = before;
    }
    return root;
  }

  /**
   * The pipes carrying on from `duct`'s far end, and from theirs in turn, which turning it swings round with
   * it: each starts where the one before ends, so turned as one piece, the whole run keeps its shape. Not past
   * a fitted bend, nor a junction that was moved or a turbo, which stay where they are.
   */
  private carriedOn(duct: ExhaustDuct): ExhaustDuct[] {
    const graph = this.context?.graph;
    if (!graph) return [];
    const carried: ExhaustDuct[] = [];
    const seen = new Set([duct.id]);
    const queue = [duct];
    while (queue.length > 0) {
      const d = queue.shift()!;
      if (d.to.kind !== 'node' || d.fitted) continue;
      const node = d.to.node;
      if (junctionAt(graph, node) || turboAt(graph, node)) continue;
      for (const e of endsAt(graph, node)) {
        if (e.end !== 'inlet' || seen.has(e.duct.id)) continue;
        seen.add(e.duct.id);
        carried.push(e.duct);
        queue.push(e.duct);
      }
    }
    return carried;
  }

  /**
   * The direction `duct`'s stored heading is turned off, as `layoutGraph` reads it: its port's, a turbo's
   * outlet flange's, or the world's for a pipe leaving a junction, which is stored in world terms from here
   * on, since the junction's own direction is worked out afresh each time.
   */
  private headingBase(duct: ExhaustDuct): THREE.Vector3 {
    const ctx = this.context!;
    if (duct.from.kind === 'valve') return ctx.ports[duct.from.cylinder]?.direction.clone() ?? this.heading.clone();
    if (duct.from.kind === 'free') return new THREE.Vector3(1, 0, 0);
    const outlet = ctx.placement.turbos.get(duct.from.node)?.outlet;
    if (outlet && duct.headingFrame !== 'world') return new THREE.Vector3(...outlet.dir);
    const world = new THREE.Vector3(1, 0, 0);
    if (duct.headingFrame !== 'world') {
      // Stored off the junction's axis until now: stored again off the world's the way it points, or read off
      // the world's it would turn by the angle between them, a right angle off a fitted pipe's end.
      const heading = ctx.placement.ducts.get(duct.id)?.heading;
      if (heading) {
        const turn = turnBetween(world, heading);
        duct.headingYaw = turn.yaw;
        duct.headingPitch = turn.pitch;
      }
      duct.headingFrame = 'world';
    }
    return world;
  }

  /**
   * The selected pipe by its triad: swung as one piece about where it starts.
   *
   * Every drawn segment keeps its length and turns with the rest, so the bends between them stay as they
   * were: the way the pipe sets off goes into its heading, and each corner after is worked out again from
   * the turned directions either side of it. A bend fitted to what the pipe joins at its far end is fitted
   * again when the scene rebuilds. With shift held it lands on 15-degree steps from the engine's axes, so
   * a pipe at an odd angle squares up. On a bend further along, the same from that bend on.
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
      const from = drag.from ?? 0;
      const dirs = [...drag.shape0.starts.slice(from), ...drag.shape0.ends.slice(from)];
      const ref = dirs.find((d) => d.clone().cross(drag.axis).lengthSq() > 1e-6) ?? drag.shape0.starts[from]!;
      turn = snapTurnToEngine(ref, turn, drag.axis, (TRIAD_TURN_DEG * Math.PI) / 180);
    }
    swingPipe(duct, drag.base, drag.shape0, drag.axis, turn, drag.from ?? 0);
    // Turned where it starts, a junction fixed there points the way it now sets off.
    const pinned = duct.from.kind === 'node' ? junctionAt(this.context!.graph, duct.from.node) : undefined;
    if (pinned) {
      const set = drag.shape0.starts[0]!.clone().applyAxisAngle(drag.axis, turn);
      pinned.axis = [set.x, set.y, set.z];
    }
    // What carries on from its end swings round with it, as one piece.
    for (const c of drag.carried ?? []) {
      const on = this.context!.graph.ducts.find((d) => d.id === c.id);
      if (!on) continue;
      if (c.shape0.starts.length > 0) {
        swingPipe(on, c.base, c.shape0, drag.axis, turn);
      } else {
        // Nothing but a fitted bend: it sets off the way it now turns to, and is fitted again from there.
        const h = turnBetween(c.base, c.heading0.clone().applyAxisAngle(drag.axis, turn));
        on.headingYaw = h.yaw;
        on.headingPitch = h.pitch;
      }
    }
    drag.moved = true;
    this.commitFrame(drag, () => this.cb.onChange(false, duct.id), () => this.cb.onChange(true, duct.id));
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
      if (target && this.startable(target)) this.beginRoute(target);
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
      const startable = this.startable(target);
      this.marker.visible = startable;
      if (startable) {
        this.marker.position.copy(target!.point);
        this.styleMarker(target!);
      }
      this.drawGhost.group.visible = false;
      return;
    }

    const duct = ctx.graph.ducts.find((d) => d.id === this.route!.ductId);
    if (!duct) return;
    const routeEnd = routeTip(duct.segments, this.route.place);
    const target = this.resolveSnap(routeEnd.point);
    if (!target) {
      this.hidePreview();
      return;
    }

    const tip = this.setOff(duct, routeEnd, target.point);
    const dia = this.routeDiameter(duct);
    let shown = false;
    let point = target.point.clone();
    if (target.kind === 'port') {
      // Onto a port: the cylinder's pipe, bent from its flange round into where the route stands.
      this.drawGhost.rebuild([fitCurve(tip.point, tip.dir, target.point, target.dir.clone().negate(), { dIn: dia, dOut: this.portDiameter })], tip.point, tip.dir);
      shown = true;
    } else {
      // Joining something, the bend the pipe will take to arrive along it, tapering to its bore.
      const anchor = target.kind === 'free' ? null : this.connectionAnchor(target, this.route.ductId, tip, squareInto(target, snap));
      if (anchor) {
        this.drawGhost.rebuild([fitCurve(tip.point, tip.dir, anchor.point, anchor.dir, { dIn: dia, dOut: anchor.dia })], tip.point, tip.dir);
        shown = true;
      } else if (target.kind === 'free' && this.route.open) {
        // Into open space from an opening: straight on, as far along the way it points as the pointer is.
        const length = this.alongOpening(tip);
        point = tip.point.clone().addScaledVector(tip.dir, length);
        if (length > 1e-4) {
          this.drawGhost.rebuild([makeSegment({ kind: 'pipe', length, dIn: dia, dOut: dia })], tip.point, tip.dir);
          shown = true;
        }
        this.cb.onAim?.(`straight on, ${Math.round(length * 1000)} mm`);
      }
    }
    if (target.kind !== 'free') this.cb.onAim?.(null);
    else if (!this.route.open) this.cb.onAim?.('onto a pipe or a port');
    this.drawGhost.group.visible = shown;
    this.drawGhost.setTint(target.kind === 'free' ? STRAIGHT_ON_COLOUR : new THREE.Color(PREVIEW_COLOUR));
    // Marked only when it would *connect*, so the highlight means something.
    this.marker.visible = target.kind !== 'free';
    this.marker.position.copy(point);
    this.styleMarker(target);
  }

  private onPointerMove = (e: PointerEvent): void => {
    // Ctrl pressed or let go while the window was not listening for keys.
    if (e.ctrlKey !== this.engineFrameHeld) this.holdEngineFrame(e.ctrlKey);
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
    if (this.header) {
      this.updatePointer(e);
      this.dom.style.cursor = this.headerTriad.hover(this.raycaster) ? 'grab' : this.openingUnderPointer() ? 'pointer' : '';
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
    this.cb.onChange(commit, this.pipeDuctId());
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

    // Where the pipe starts or ends at a junction, the pipes meeting it there follow (`carryBore`).
    const graph = this.context?.graph;
    const duct = graph?.ducts.find((x) => x.segments === this.pipe);

    // A pipe's two ends are set apart, so it tapers between them; the inlet ring sets where it starts.
    if (this.drag!.data.kind === 'inlet') {
      const was = segmentDiameter(seg, 0);
      seg.dIn = d;
      if (graph && duct && i === 0) carryBore(graph, duct, 'start', was, d);
      return;
    }

    // The outlet diameter is `dIn` for a chamber, which necks back down to its throat, and `dOut` for a
    // pipe, matching `segmentDiameter(seg, 1)`. The next segment starts at it.
    const was = segmentDiameter(seg, 1);
    if (seg.kind === 'chamber') seg.dIn = d;
    else seg.dOut = d;

    this.propagate(i);
    if (graph && duct && i === this.pipe.length - 1) carryBore(graph, duct, 'end', was, d);
  }

  /** The duct whose segments the handles are on. */
  private pipeDuctId(): string | undefined {
    return this.context?.graph.ducts.find((d) => d.segments === this.pipe)?.id;
  }

  /** Keep the duct continuous: the next segment starts where this one ends. */
  private propagate(index: number): void {
    const seg = this.pipe[index];
    const next = this.pipe[index + 1];
    if (!seg || !next) return;
    next.dIn = segmentDiameter(seg, 1);
  }

  private onPointerUp = (e: PointerEvent): void => {
    if (e.button === 2) {
      const down = this.rightDown;
      this.rightDown = null;
      if (!down || Math.hypot(e.clientX - down.x, e.clientY - down.y) > RIGHT_CLICK_PIXELS) return;
      if (this.toolOn) {
        this.exitTool();
        return;
      }
      this.updatePointer(e);
      const pick = this.pickScene();
      if (!pick) return;
      this.cb.onPick?.(pick);
      this.cb.onMenu?.(pick, e.clientX, e.clientY);
      return;
    }
    if (this.bendDrag) {
      this.endBendDrag();
      return;
    }
    if (this.triadDrag) {
      const drag = this.triadDrag;
      this.triadDrag = null;
      this.controls.enabled = true;
      this.applyEngineFrame();
      if (!drag.moved || drag.on === 'header') return;
      // Final authoritative push, since intermediate frames were throttled.
      if (drag.on === 'turbo') {
        const mount = this.context?.graph.turbos?.find((t) => t.id === drag.turbo);
        if (mount?.position) this.cb.onMoveTurbo?.(mount.id, mount.position, mount.rotation, true);
      } else if (drag.lastPosition && drag.startNode && drag.startAxis) {
        this.cb.onMoveJunction?.(drag.startNode, drag.lastPosition, drag.startAxis, true);
      } else {
        this.cb.onChange(true, drag.ductId);
        this.rebuildHandles();
      }
      return;
    }
    if (!this.drag) return;
    this.drag = null;
    this.controls.enabled = true;
    this.dom.style.cursor = '';
    // Final authoritative push, since intermediate frames were throttled.
    this.cb.onChange(true, this.pipeDuctId());
    this.rebuildHandles();
  };

  dispose(): void {
    this.dom.removeEventListener('pointerdown', this.onPointerDown);
    this.dom.removeEventListener('pointermove', this.onPointerMove);
    window.removeEventListener('pointerup', this.onPointerUp);
    window.removeEventListener('keydown', this.onKeyDown);
    window.removeEventListener('keyup', this.onKeyUp);
    window.removeEventListener('blur', this.onBlur);
    this.bendTriad.dispose();
    this.bendGhost.dispose();
    for (const h of this.handles) h.geometry.dispose();
    this.pipeTriad.dispose();
    this.turboTriad.dispose();
    this.headerTriad.dispose();
    for (const g of this.headerGhosts) g.dispose();
    for (const d of this.openingDots) {
      d.geometry.dispose();
      (d.material as THREE.Material).dispose();
    }
    this.drawGhost.dispose();
    this.ghost.dispose();
    this.ghostPipe.geometry.dispose();
    (this.ghostPipe.material as THREE.Material).dispose();
    for (const m of [this.matHover, this.matSelected, this.matRing]) {
      m.dispose();
    }
  }
}

function clamp(x: number, lo: number, hi: number): number {
  return x < lo ? lo : x > hi ? hi : x;
}
