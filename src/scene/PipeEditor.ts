/**
 * Direct manipulation of the exhaust in 3D.
 *
 * Two handle types per joint:
 *   - a sphere at the end of each segment. Drag it anywhere and the segment follows:
 *     the distance sets its length, the direction sets the corner it turns at.
 *   - a ring around each joint. Drag it outward to open the pipe up, inward to
 *     choke it down.
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
import {
  joinDuctEnd,
  newDuctId,
  splitDuctAt,
  type DuctSink,
  type DuctSource,
  type ExhaustDuct,
  type ExhaustGraph,
} from '../model/exhaustGraph.js';
import { layoutPipe, turnBetween, type PipeMesh } from './PipeMesh.js';
import type { DuctPlacement, ExhaustPlacement, ExhaustPort } from './exhaustLayout.js';
import {
  MIN_DRAW_LENGTH,
  collectSnapTargets,
  continuingDiameter,
  fitSegment,
  headingOffsetTo,
  nearestSnap,
  quantiseLength,
  quantiseTurn,
  routeTip,
  type SnapTarget,
} from './drawing.js';

const MIN_LENGTH = 0.02;
const MAX_LENGTH = 2.0;
const MIN_RADIUS = 0.006;
const MAX_RADIUS = 0.22;

/**
 * How far the end-handle sphere is lifted clear of the pipe, m (plus the local
 * radius). Without an offset it sits inside the diameter ring at the same joint, and
 * since the ring is the larger target the sphere becomes effectively unclickable —
 * and on an intermediate joint it would be buried inside the next segment's wall.
 */
const HANDLE_LIFT = 0.035;

/** Live geometry edits are cheap to draw but force a waveguide rebuild, so throttle those. */
const AUDIO_COMMIT_MS = 200;

type HandleKind = 'end' | 'ring' | 'inlet';

interface HandleData {
  kind: HandleKind;
  segment: number;
  /** Displacement from the point the handle controls to the handle itself. */
  dragOffset?: THREE.Vector3;
}

/** Something in the exhaust a click picked out, outside draw mode. */
export type ScenePick =
  | { kind: 'segment'; duct: string; segment: number }
  | { kind: 'joint'; node: string };

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
}

/** How close, in pixels, the pointer has to be for a snap target to take. */
const SNAP_PIXELS = 14;
/** Turn and length quantisation while drawing. Hold shift to draw freely. */
const TURN_STEP_DEG = 15;
const LENGTH_GRID_M = 0.025;

export class PipeEditor {
  readonly group = new THREE.Group();

  private readonly raycaster = new THREE.Raycaster();
  private readonly pointer = new THREE.Vector2();
  private readonly handles: THREE.Mesh[] = [];
  /** Leader lines joining each lifted end-handle to the joint it controls. */
  private readonly stalks: THREE.Line[] = [];

  private selected: number | null = null;
  private hovered: THREE.Mesh | null = null;

  private drag: {
    handle: THREE.Mesh;
    data: HandleData;
    /** Centreline point where the dragged segment begins. */
    start: THREE.Vector3;
    /** Heading entering the dragged segment. */
    heading: THREE.Vector3;
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
  private readonly marker: THREE.Mesh;

  private readonly matNormal = new THREE.MeshBasicMaterial({ color: 0x4fd1ff });
  private readonly matHover = new THREE.MeshBasicMaterial({ color: 0xffd166 });
  private readonly matSelected = new THREE.MeshBasicMaterial({ color: 0x8cff9e });
  private readonly matRing = new THREE.MeshBasicMaterial({
    color: 0x4fd1ff,
    transparent: true,
    opacity: 0.6,
  });
  private readonly matStalk = new THREE.LineBasicMaterial({
    color: 0x4fd1ff,
    transparent: true,
    opacity: 0.35,
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
    this.preview = new THREE.Line(
      this.previewGeom,
      new THREE.LineDashedMaterial({ color: 0x8cff9e, dashSize: 0.02, gapSize: 0.012 }),
    );
    this.preview.visible = false;
    this.preview.renderOrder = 12;
    this.group.add(this.preview);

    this.marker = new THREE.Mesh(
      new THREE.SphereGeometry(0.016, 16, 12),
      new THREE.MeshBasicMaterial({ color: 0xffd166, transparent: true, opacity: 0.85 }),
    );
    this.marker.visible = false;
    this.marker.renderOrder = 13;
    this.group.add(this.marker);
  }

  // -------------------------------------------------------------------------
  // Draw mode
  // -------------------------------------------------------------------------

  /** Everything the editor needs to draw into the current scene. Refreshed on every rebuild. */
  setDrawContext(context: DrawContext): void {
    this.context = context;
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

  /** Whether a handle is being dragged: an edit that has not settled yet. */
  get dragging(): boolean {
    return this.drag !== null;
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
    this.marker.visible = false;
    this.snapped = null;
  }

  /** Handles are a nuisance while drawing: they sit exactly where the route is being aimed. */
  private applyHandleVisibility(): void {
    for (const h of this.handles) h.visible = !this.drawMode;
    for (const st of this.stalks) st.visible = !this.drawMode;
  }

  private onContextMenu = (e: MouseEvent): void => {
    if (this.drawMode) e.preventDefault();
  };

  private onKeyDown = (e: KeyboardEvent): void => {
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
    return !!target && target.kind !== 'free';
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
  private extendRoute(target: SnapTarget, free: boolean): void {
    const ctx = this.context;
    if (!ctx || !this.route) return;
    const duct = ctx.graph.ducts.find((d) => d.id === this.route!.ductId);
    if (!duct) return;

    const tip = routeTip(duct.segments, this.route.place);
    const dia = continuingDiameter(
      duct.segments.length > 0 ? duct : this.startingDuct(duct),
      this.startingDiameter(duct),
    );

    let point = target.point.clone();
    if (target.kind === 'free' && !free) {
      // Quantise the turn and the length so a hand-drawn route comes out tidy.
      const dir = quantiseTurn(point.clone().sub(tip.point), tip.dir, TURN_STEP_DEG);
      const len = quantiseLength(point.distanceTo(tip.point), LENGTH_GRID_M);
      point = tip.point.clone().addScaledVector(dir, len);
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
    this.selected = index;
    this.applyHandleColours();
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
    for (const s of this.stalks) {
      s.geometry.dispose();
      this.group.remove(s);
    }
    this.stalks.length = 0;

    const layout = layoutPipe(this.pipe, this.origin, this.heading);

    // Inlet ring: the header diameter, the one inlet not fixed by a previous segment.
    if (this.pipe.length > 0) {
      const r0 = this.pipe[0]!.dIn / 2;
      this.addRing(this.origin, this.heading, r0, { kind: 'inlet', segment: 0 });
    }

    for (let i = 0; i < layout.joints.length; i++) {
      const p = layout.joints[i]!;
      const d = layout.jointDirections[i]!;
      const r = layout.jointRadii[i]!;

      // Lift the sphere clear of the pipe, with a stalk so it is obvious which joint
      // it belongs to. `dragOffset` is subtracted when dragging so the pipe end
      // lands where the pointer is, not where the handle is.
      const lift = liftVector(d).multiplyScalar(r + HANDLE_LIFT);
      const sphere = new THREE.Mesh(new THREE.SphereGeometry(0.013, 16, 12), this.matNormal);
      sphere.position.copy(p).add(lift);
      sphere.userData = {
        kind: 'end',
        segment: i,
        dragOffset: lift.clone(),
      } satisfies HandleData;
      sphere.renderOrder = 10;
      this.handles.push(sphere);
      this.group.add(sphere);

      const stalk = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([p.clone(), sphere.position.clone()]),
        this.matStalk,
      );
      this.stalks.push(stalk);
      this.group.add(stalk);

      this.addRing(p, d, r, { kind: 'ring', segment: i });
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
      const isRing = data.kind !== 'end';
      if (h === this.hovered) h.material = this.matHover;
      else if (data.segment === this.selected) h.material = this.matSelected;
      else h.material = isRing ? this.matRing : this.matNormal;
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

    if (this.drawMode) {
      this.onDrawClick(e);
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

    // Anything else under the pointer: a segment of any pipe, or a junction.
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
    return (best as { pick: ScenePick } | null)?.pick ?? null;
  }

  private beginDrag(handle: THREE.Mesh, data: HandleData): void {
    const layout = layoutPipe(this.pipe, this.origin, this.heading);
    const i = data.segment;
    const start = i === 0 ? this.origin.clone() : layout.joints[i - 1]!.clone();
    const heading = i === 0 ? this.heading.clone() : layout.jointDirections[i - 1]!.clone();

    // Always drag in a plane facing the camera.
    //
    // A plane perpendicular to the pipe would be the obvious choice for a radius
    // handle, but the pipe is normally viewed side-on, which puts the pointer ray
    // almost parallel to that plane. The intersection then shoots off to infinity for
    // a few pixels of movement, enough to take a 40 mm pipe to the 440 mm clamp in one
    // short drag. A camera-facing plane is always well conditioned; the radius is
    // recovered afterwards as the perpendicular distance from the pipe's axis.
    const normal = new THREE.Vector3();
    this.camera.getWorldDirection(normal);
    const plane = new THREE.Plane();
    plane.setFromNormalAndCoplanarPoint(normal, handle.position);

    const axisPoint = data.kind === 'inlet' ? this.origin.clone() : layout.joints[i]!.clone();
    const axisDir = (
      data.kind === 'inlet' ? this.heading.clone() : layout.jointDirections[i]!.clone()
    ).normalize();

    this.drag = { handle, data, start, heading, plane, axisPoint, axisDir, lastCommit: 0 };
    this.controls.enabled = false;
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
    if (target) this.extendRoute(target, e.shiftKey);
  }

  /** Ghost the segment the next click would add, and mark what it would snap to. */
  private updateDrawPreview(shift: boolean): void {
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

    let point = target.point.clone();
    if (target.kind === 'free' && !shift) {
      const dir = quantiseTurn(point.clone().sub(tip.point), tip.dir, TURN_STEP_DEG);
      const len = quantiseLength(point.distanceTo(tip.point), LENGTH_GRID_M);
      point = tip.point.clone().addScaledVector(dir, len);
    }

    this.previewGeom.setFromPoints([tip.point, point]);
    this.preview.computeLineDistances();
    this.preview.visible = true;
    // Marked only when it would *connect*, so the highlight means something.
    this.marker.visible = target.kind !== 'free';
    this.marker.position.copy(point);
    this.snapped = target;
  }

  private onPointerMove = (e: PointerEvent): void => {
    if (this.drawMode && !this.drag) {
      this.updatePointer(e);
      this.updateDrawPreview(e.shiftKey);
      return;
    }
    this.updatePointer(e);

    if (!this.drag) {
      const hit = this.raycaster.intersectObjects(this.handles, false)[0];
      const next = (hit?.object as THREE.Mesh | undefined) ?? null;
      if (next !== this.hovered) {
        this.hovered = next;
        this.applyHandleColours();
      }
      // A hand over anything clickable, so it is discoverable that pipes and junctions can be picked.
      this.dom.style.cursor = next ? 'grab' : this.pickScene() ? 'pointer' : '';
      return;
    }

    const point = new THREE.Vector3();
    if (!this.raycaster.ray.intersectPlane(this.drag.plane, point)) return;

    const seg = this.pipe[this.drag.data.segment];
    if (!seg) return;

    if (this.drag.data.kind === 'end') this.dragEnd(seg, point);
    else this.dragRadius(seg, point);

    const now = performance.now();
    const commit = now - this.drag.lastCommit > AUDIO_COMMIT_MS;
    if (commit) this.drag.lastCommit = now;
    this.cb.onChange(commit);
    e.preventDefault();
  };

  /**
   * Grab the end of a segment and move it: distance sets length, direction sets the corner.
   *
   * A segment is straight and turns where it starts, so the pointer's position gives both exactly — the
   * turn from the direction coming in to the direction of the pointer, and the distance to it.
   */
  private dragEnd(seg: PipeSegment, rawTarget: THREE.Vector3): void {
    // The sphere is drawn lifted clear of the pipe, so undo that displacement to get
    // where the user actually wants the pipe end to be.
    const offset = this.drag!.data.dragOffset;
    const target = offset ? rawTarget.clone().sub(offset) : rawTarget;
    const chord = target.clone().sub(this.drag!.start);
    const chordLen = chord.length();
    if (chordLen < 1e-4) return;
    const turn = turnBetween(this.drag!.heading, chord);
    seg.yaw = turn.yaw;
    seg.pitch = turn.pitch;
    seg.length = clamp(chordLen, MIN_LENGTH, MAX_LENGTH);
  }

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
    for (const s of this.stalks) s.geometry.dispose();
    for (const m of [
      this.matNormal,
      this.matHover,
      this.matSelected,
      this.matRing,
      this.matStalk,
    ]) {
      m.dispose();
    }
  }
}

/**
 * A unit direction to lift a handle away from the pipe: perpendicular to the pipe
 * and as close to straight up as that allows, so handles do not disappear into the
 * geometry on a vertical run.
 */
function liftVector(direction: THREE.Vector3): THREE.Vector3 {
  const up = new THREE.Vector3(0, 1, 0);
  const v = up.clone().projectOnPlane(direction.clone().normalize());
  if (v.lengthSq() < 1e-6) {
    return new THREE.Vector3(0, 0, 1).projectOnPlane(direction.clone().normalize()).normalize();
  }
  return v.normalize();
}

function clamp(x: number, lo: number, hi: number): number {
  return x < lo ? lo : x > hi ? hi : x;
}
