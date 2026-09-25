/**
 * Wiring. Owns the single source of truth for the configuration and keeps the three
 * consumers — audio thread, 3D scene, control panel — pointed at the same object.
 *
 * Data flow:
 *   panel / 3D handles  ->  config (mutated in place)  ->  AudioEngine.setGraph/setEngine
 *                                                      ->  PipeMesh.rebuild, JointMesh.rebuild
 *   worklet snapshot @60 Hz  ->  EngineMesh.update, PipeMesh.update, Panel, Scope
 */

import * as THREE from 'three';
import { AudioEngine } from './audio/AudioEngine.js';
import {
  defaultConfig,
  fullLoadTorque,
  makeSegment,
  type EngineConfig,
  type EngineSnapshot,
  type EngineSpec,
} from './model/spec.js';
import { EngineMesh } from './scene/EngineMesh.js';
import { PipeEditor } from './scene/PipeEditor.js';
import { PipeMesh } from './scene/PipeMesh.js';
import { ductDirections, freezeHeadings, layoutGraph, pipesMeetAt, type ExhaustPlacement } from './scene/exhaustLayout.js';
import { JointMesh, throughPipe } from './scene/jointMesh.js';
import {
  carriedGeometry,
  compileLayout,
  defaultDuctId,
  disconnectEnd,
  graphFromJson,
  hasBeenEdited,
  removeDuct,
  removeJunction,
  validateGraph,
  type DuctDirections,
} from './model/exhaustGraph.js';
import { Viewer } from './scene/Viewer.js';
import { Panel, SAMPLE_RATES, type ViewOptions } from './ui/Panel.js';
import { Scope } from './ui/Scope.js';

const viewportEl = must<HTMLElement>('#viewport');
const panelEl = must<HTMLElement>('#panel');
const scopeEl = must<HTMLCanvasElement>('#scope');
const overlayEl = must<HTMLElement>('#overlay');
const hudEl = must<HTMLElement>('#hud');

const config: EngineConfig = loadConfig();
/**
 * The duct graph, seeded from the layout spec and authoritative thereafter.
 *
 * The renderer and the solver work from the same graph, so what is on screen is what is being solved.
 * It is re-seeded when the *topology* changes — a different cylinder count or merge plan makes
 * whatever was drawn meaningless — when the ports move while the exhaust is still as compiled, and when
 * a preset is loaded. `reseedGraph` preserves as much of the drawn geometry as the new topology can carry.
 */
config.graph ??= compileLayout(config.engine, config.pipe, config.collector);


/**
 * The audio sample rate, kept per device rather than in the URL: it is a choice about what this
 * machine can afford, not part of the engine, so a shared link should not carry a phone's setting.
 */
const SAMPLE_RATE_KEY = 'engine-simulator:sampleRate';
const sampleRate = loadSampleRate();
const audio = new AudioEngine(config, sampleRate);
const viewer = new Viewer(viewportEl);
const engineMesh = new EngineMesh(config.engine, viewer.clipPlane);
/**
 * One mesh per duct in the graph, in the graph's order.
 *
 * The editor's handles sit on one duct at a time, `editedDuctId`. Every duct has its own
 * `PipeSegment[]`; the panel copies an edit to the other runners when they are linked.
 */
const pipeMeshes: PipeMesh[] = [];
/**
 * Which duct the drag handles are attached to.
 *
 * Starts on the graph's default duct (`defaultDuctId`, the same rule the panel uses), and follows
 * whichever duct is picked in the scene or in the panel's duct menu. Reset to it whenever the graph is
 * reseeded.
 */
let editedDuctId = defaultDuctId(config.graph!) ?? 'runner0';
/** The junction selected in the scene, if one is. Segments and junctions are selected one or the other. */
let selectedJoint: string | null = null;
/** One mesh per junction, blended from the pipes that meet there. */
const jointMeshes: JointMesh[] = [];
/** Which junction each of `jointMeshes` is, by index. */
let jointNodes: string[] = [];
/** The layout the scene was last built from, for questions about geometry such as a tee's through pipe. */
let lastPlacement: ExhaustPlacement | null = null;
/**
 * The layout as it was before the edit in progress: not updated during a drag.
 *
 * What a pipe pointed at before it was changed is what tidying afterwards should keep it pointing at.
 */
let stablePlacement: ExhaustPlacement | null = null;

/** Fix every pipe where it stands before an edit, so the edit moves only what it edits. */
function freeze(): void {
  if (!stablePlacement) return;
  const ports = Array.from({ length: engineMesh.bankCount }, (_, b) => engineMesh.exhaustPort(b));
  freezeHeadings(config.graph!, stablePlacement, ports);
}

/** Every duct's directions in a layout, for rejoining pipes without swinging them. */
function directionsOf(placement: ExhaustPlacement | null): DuctDirections | undefined {
  return placement ? ductDirections(config.graph!, placement) : undefined;
}

/**
 * Take the edited pipe off its junction if the edit left the pipes there no longer meeting.
 *
 * Pipes are straight tube: shorten one that runs into a junction and it falls short of it. Left joined,
 * the fitting grew to bridge the gap — up to 36 cm across. So its end is left open instead, which is what
 * cutting a real pipe short does. If the edited pipe was the one the junction follows, it is the *others*
 * that stop meeting, and it is still the edited one that comes off.
 */
function detachIfShort(ductId: string): void {
  const graph = config.graph!;
  const duct = graph.ducts.find((d) => d.id === ductId);
  if (!duct || duct.to.kind !== 'node') return;
  const ports = Array.from({ length: engineMesh.bankCount }, (_, b) => engineMesh.exhaustPort(b));
  if (!pipesMeetAt(graph, layoutGraph(ports, graph), duct.to.node)) {
    disconnectEnd(graph, duct.id, directionsOf(stablePlacement));
  }
}

viewer.scene.add(engineMesh.group);

const editorTarget = new PipeMesh();
pipeMeshes.push(editorTarget);
viewer.scene.add(editorTarget.group);

const editor = new PipeEditor(
  viewer.renderer.domElement,
  viewer.camera,
  viewer.controls,
  editorTarget,
  config.pipe,
  {
    onChange: (commit) => {
      freeze();
      // Once an edit has settled — not on every frame of a drag, nor while a route is being drawn.
      if (!editor.dragging && !editor.drawing) detachIfShort(editedDuctId);
      rebuildPipeGeometry();
      panel.syncPipe();
      // Rebuilding the waveguide reallocates and briefly ramps the audio, so during
      // a drag it is throttled; the editor always sends a final commit on release.
      if (commit) audio.setGraph(config.graph!);
    },
    onSelect: (i) => {
      selectJoint(null);
      panel.setSelected(i);
    },
    onPick: (pick) => {
      if (pick?.kind === 'joint') {
        selectJoint(pick.node);
        editor.select(null);
        panel.setSelected(null);
        return;
      }
      selectJoint(null);
      if (!pick) {
        editor.select(null);
        panel.setSelected(null);
        return;
      }
      // A segment of another pipe: the handles and the panel move to that pipe, then select it.
      if (pick.duct !== editedDuctId) {
        editedDuctId = pick.duct;
        panel.showDuct(pick.duct);
        rebuildPipeGeometry();
      }
      editor.select(pick.segment);
      panel.setSelected(pick.segment);
    },
    onDrawing: (active) => panel.setDrawingState(active),
  },
);

/**
 * Delete or Backspace removes whatever is selected in the scene: a junction, or a pipe segment.
 *
 * Not while typing — Backspace in a length field must edit the number — and not mid-route, where the
 * editor's own keys apply.
 */
window.addEventListener('keydown', (e) => {
  if (e.key !== 'Delete' && e.key !== 'Backspace') return;
  const active = document.activeElement;
  if (active instanceof HTMLInputElement || active instanceof HTMLSelectElement || active instanceof HTMLTextAreaElement) {
    return;
  }
  if (editor.drawing) return;

  if (selectedJoint) {
    const joint = lastPlacement?.joints.get(selectedJoint);
    freeze();
    removeJunction(config.graph!, selectedJoint, joint ? throughPipe(joint) : null, directionsOf(stablePlacement));
    selectJoint(null);
    editor.select(null);
    panel.rebuildPipeList();
    rebuildPipeGeometry();
    audio.setGraph(config.graph!);
    saveConfig();
    e.preventDefault();
    return;
  }
  // The panel owns segment deletion, so linked runners stay linked however the delete was asked for.
  if (panel.deleteSelected()) e.preventDefault();
});

/** Select a junction, or none: highlight it in the scene and describe it in the panel. */
function selectJoint(node: string | null): void {
  if (node === selectedJoint) return;
  selectedJoint = node;
  jointMeshes.forEach((m, i) => m.setSelected(jointNodes[i] === node));
  panel.showJoint(node);
}
viewer.scene.add(editor.group);

const panel = new Panel(panelEl, config, {
  onEngine: (partial) => {
    Object.assign(config.engine, partial);
    audio.setEngine(partial);
    engineMesh.setSpec(config.engine);
    // Bore and valve changes move the port, so the pipe has to follow it.
    /**
     * A different cylinder count or merge plan makes the drawn routing meaningless, so the graph is
     * rebuilt from the geometry it carried. See `reseedGraph`.
     *
     * And so does anything that moves the ports, while the exhaust is still as compiled: a manifold's
     * lengths are cut to the spacing of the ports, so after a bore change they no longer reached, and
     * the fittings grew to bridge the gaps. An exhaust the user has edited is theirs, and is left alone.
     */
    if (touchesTopology(partial) || (touchesGeometry(partial) && !hasBeenEdited(config.graph!))) {
      reseedGraph();
      audio.setGraph(config.graph!);
      panel.rebuildPipeList();
    }
    if (touchesGeometry(partial)) rebuildPipeGeometry();
    saveConfig();
  },
  onPipe: () => {
    freeze();
    detachIfShort(editedDuctId);
    rebuildPipeGeometry();
    audio.setGraph(config.graph!);
    saveConfig();
  },
  onRemoveDuct: (id) => {
    freeze();
    removeDuct(config.graph!, id, directionsOf(stablePlacement));
    rebuildPipeGeometry();
    audio.setGraph(config.graph!);
    saveConfig();
  },
  onSelectDuct: (id) => {
    editedDuctId = id;
    rebuildPipeGeometry();
  },
  onDrawMode: (on) => {
    editor.setDrawMode(on);
  },
  onDrawFromJoint: (node) => {
    editor.setDrawMode(true);
    editor.startAtJunction(node);
  },
  onReseed: () => {
    reseedGraph(true);
    rebuildPipeGeometry();
    audio.setGraph(config.graph!);
    saveConfig();
  },
  onSelect: (i) => editor.select(i),
  onToggleAudio: () => {
    void audio.toggle().then((running) => {
      panel.setRunning(running);
      overlayEl.classList.toggle('hidden', running);
    });
  },
  onSampleRate: (hz) => {
    try {
      localStorage.setItem(SAMPLE_RATE_KEY, String(hz));
    } catch {
      // Storage can be off (private browsing); the choice then lasts until reload.
    }
    void audio.setSampleRate(hz);
  },
  onView: applyView,
  onResetView: () => viewer.frameBounds(sceneBounds()),
}, sampleRate);

const scope = new Scope(scopeEl, audio);

// ---------------------------------------------------------------------------
// Geometry sync
// ---------------------------------------------------------------------------

/**
 * Changes that invalidate a *drawn* graph, as opposed to merely moving the ports.
 *
 * Narrower than `touchesGeometry` on purpose. Growing the bore moves where a runner starts but the
 * routing still means something; changing the cylinder count or the merge plan does not, because the
 * ducts a runner was drawn into may no longer exist.
 */
function touchesTopology(partial: Partial<EngineSpec>): boolean {
  return (
    'cylinders' in partial ||
    'exhaustLayout' in partial ||
    'crankType' in partial ||
    'firingOffset' in partial ||
    'vAngle' in partial
  );
}

function touchesGeometry(partial: Partial<EngineSpec>): boolean {
  return (
    'bore' in partial ||
    'stroke' in partial ||
    'rodLength' in partial ||
    'compressionRatio' in partial ||
    'exValveDia' in partial ||
    'cylinders' in partial ||
    'vAngle' in partial ||
    'crankType' in partial ||
    'firingOffset' in partial ||
    'exhaustLayout' in partial
  );
}


/**
 * Rebuild the graph for a new topology, keeping the geometry the user had.
 *
 * The runner and collector shapes are taken from the existing graph rather than from `config.pipe`, so
 * a change of layout does not throw away edits. Routing cannot survive — a runner aimed at a junction
 * that no longer exists has nowhere to go — so stored headings are dropped and the new runners are
 * aimed again.
 *
 * With `fromConfig` the graph is built from `config.pipe` and `config.collector` as they stand, which is
 * what loading a preset wants.
 */
function reseedGraph(fromConfig = false): void {
  if (!fromConfig) {
    // Carry the drawn geometry across: a runner that was lengthened should stay lengthened even
    // though its routing cannot survive a change of topology.
    const carried = config.graph ? carriedGeometry(config.graph) : {};
    if (carried.pipe) config.pipe = carried.pipe;
    if (carried.collector) config.collector = carried.collector;
  }
  config.graph = compileLayout(config.engine, config.pipe, config.collector);
  editedDuctId = defaultDuctId(config.graph) ?? editedDuctId;
}

function rebuildPipeGeometry(): void {
  const cylinders = engineMesh.bankCount;
  const graph = config.graph!;

  // One mesh per duct, one merge body per junction that has one.
  while (pipeMeshes.length < graph.ducts.length) {
    const m = new PipeMesh();
    pipeMeshes.push(m);
    viewer.scene.add(m.group);
  }
  while (pipeMeshes.length > graph.ducts.length) {
    const m = pipeMeshes.pop()!;
    viewer.scene.remove(m.group);
    m.dispose();
  }

  const ports = Array.from({ length: cylinders }, (_, b) => engineMesh.exhaustPort(b));
  const placement = layoutGraph(ports, graph);
  lastPlacement = placement;
  if (!editor.dragging) stablePlacement = placement;

  graph.ducts.forEach((duct, i) => {
    const place = placement.ducts.get(duct.id);
    if (place) pipeMeshes[i]!.rebuild(duct.segments, place.origin, place.heading);
  });

  const bodies = [...placement.joints.entries()];
  while (jointMeshes.length < bodies.length) {
    const m = new JointMesh();
    jointMeshes.push(m);
    viewer.scene.add(m.group);
  }
  while (jointMeshes.length > bodies.length) {
    const m = jointMeshes.pop()!;
    viewer.scene.remove(m.group);
    m.dispose();
  }
  jointNodes = bodies.map(([node]) => node);
  bodies.forEach(([node, body], i) => {
    jointMeshes[i]!.rebuild(body);
    jointMeshes[i]!.setSelected(node === selectedJoint);
  });
  // A selected junction that no longer exists — merged away, or the graph reseeded — is no longer selected.
  if (selectedJoint && !placement.joints.has(selectedJoint)) {
    selectedJoint = null;
    panel.showJoint(null);
  }

  /**
   * The editor's handles belong to one duct, so they must use the frame that duct was actually built
   * with — not the bare port direction. With a collector those differ by however far the runner had to
   * be aimed to reach the collar, which is exactly why the handles used to sit off the pipe.
   */
  editor.setDrawContext({
    graph,
    placement,
    ports,
    meshes: pipeMeshes,
    joints: bodies.map(([node], i) => ({ node, target: jointMeshes[i]!.pickTarget })),
  });
  editor.portDiameter = config.engine.exValveDia;

  const editedDuct = graph.ducts.find((d) => d.id === editedDuctId) ?? graph.ducts[0];
  if (editedDuct) {
    const place = placement.ducts.get(editedDuct.id);
    const meshIndex = graph.ducts.indexOf(editedDuct);
    if (place) {
      editor.setPortFrame(place.origin, place.heading);
      editor.setTarget(pipeMeshes[meshIndex]!, editedDuct.segments);
    }
  }
  editor.rebuildHandles();
}

function sceneBounds(): THREE.Box3 {
  const box = new THREE.Box3();
  for (const m of pipeMeshes) box.union(m.boundingBox());
  for (const m of jointMeshes) box.union(m.boundingBox());
  box.expandByPoint(new THREE.Vector3(0, -0.12, 0));
  box.expandByPoint(new THREE.Vector3(0, engineMesh.exhaustPort(0).position.y + 0.1, 0));
  return box;
}

function applyView(v: ViewOptions): void {
  viewer.setCutaway(v.cutaway);
  for (const m of pipeMeshes) {
    m.setPressureVisible(v.pressure);
  }
  editor.setHandlesVisible(v.handles);
}

// ---------------------------------------------------------------------------
// Snapshot -> visuals
// ---------------------------------------------------------------------------

let latest: EngineSnapshot | null = null;
/** Crank angle the renderer is showing, extrapolated between snapshots. */
let displayAngle = 0;
let displayRpm = 0;

audio.onSnapshot((s) => {
  latest = s;
  displayRpm = s.rpm;
  // Snapshots arrive at 60 Hz but the crank may be turning at 160 rev/s, so the
  // angle is advanced continuously between them and only nudged towards the
  // authoritative value. Snapping straight to it makes the piston strobe.
  const err = shortestAngle(s.crankAngle - displayAngle);
  displayAngle += err * 0.25;
  // Every duct gets the same pressure colouring. The snapshot carries the pressure along whichever duct
  // the listener mostly hears — the collector if there is one — so the primaries show an
  // indicative field rather than their own.
  for (const m of pipeMeshes) m.update(s.pipePressure);
  scope.onSnapshot(s);
  panel.updateReadouts(s);
  hudEl.textContent =
    `${Math.round(s.rpm)} rpm · ${(s.cylPressure / 1e5).toFixed(1)} bar · ` +
    `${Math.round(s.cylTemp)} K · wall ${Math.round(s.wallTemp)} K · ` +
    `${s.pipeCells} cells × ${s.substeps}`;
});

viewer.onFrame((dt) => {
  if (displayRpm > 0) displayAngle = (displayAngle + displayRpm * 6 * dt) % 720;
  // Combustion glow: a short flash after the burn begins.
  if (latest) {
    // Extrapolate each bank from its own snapshot angle, keeping the phase relationship.
    const posed = latest.banks.map((b) => ({
      ...b,
      crankAngle:
        (displayAngle + (b.crankAngle - latest!.banks[0]!.crankAngle) + 1440) % 720,
    }));
    engineMesh.update(
      posed,
      posed.map((b) => glow(b.crankAngle, config.engine.ignition)),
    );
  }
  scope.draw();
});

/** Wraps to (-360, 360]. */
function shortestAngle(d: number): number {
  let x = d % 720;
  if (x > 360) x -= 720;
  if (x < -360) x += 720;
  return x;
}

function glow(angle: number, ignition: number): number {
  let rel = (angle - ignition) % 720;
  if (rel < 0) rel += 720;
  const window = config.engine.burnDuration * 1.6;
  if (rel > window) return 0;
  const t = rel / window;
  return Math.sin(Math.PI * t) ** 1.5;
}

// ---------------------------------------------------------------------------
// Persistence: the whole configuration round-trips through the URL hash, so a
// pipe someone likes is a shareable link rather than something to screenshot.
// ---------------------------------------------------------------------------

function loadConfig(): EngineConfig {
  const base = defaultConfig();
  const hash = location.hash.replace(/^#/, '');
  if (!hash) return base;
  try {
    const parsed = JSON.parse(decodeURIComponent(atob(hash))) as Partial<EngineConfig>;
    if (parsed.engine) {
      Object.assign(base.engine, parsed.engine);
      // Links from before the load was a fraction carry it in N*m.
      const legacy = (parsed.engine as { loadTorque?: unknown }).loadTorque;
      if (typeof legacy === 'number' && parsed.engine.load === undefined) {
        base.engine.load = Math.min(Math.max(legacy / fullLoadTorque(base.engine), 0), 1.5);
      }
      delete (base.engine as { loadTorque?: unknown }).loadTorque;
    }
    if (Array.isArray(parsed.pipe) && parsed.pipe.length > 0) {
      base.pipe = parsed.pipe.map((s) => makeSegment(s));
    }
    // The collector used to be dropped here, so a shared link silently lost it and came back with the
    // default. It is part of the geometry like anything else.
    if (Array.isArray(parsed.collector) && parsed.collector.length > 0) {
      base.collector = parsed.collector.map((s) => makeSegment(s));
    }

    /**
     * A drawn graph, if the link carried one, rebuilt by `graphFromJson` rather than trusted as-is. A
     * graph that does not describe this engine is discarded in favour of compiling a fresh one, which is
     * the same thing a change of topology does.
     */
    const graph = graphFromJson(parsed.graph);
    if (graph) {
      if (validateGraph(graph, base.engine.cylinders).length === 0) base.graph = graph;
      else console.warn('[main] the exhaust in the URL does not fit this engine; rebuilding it');
    }
    return base;
  } catch {
    console.warn('[main] could not read the configuration in the URL; using defaults');
    return base;
  }
}

function loadSampleRate(): number {
  let stored: number;
  try {
    stored = Number(localStorage.getItem(SAMPLE_RATE_KEY));
  } catch {
    return 48000;
  }
  return SAMPLE_RATES.some(([hz]) => hz === stored) ? stored : 48000;
}

let saveTimer = 0;
function saveConfig(): void {
  // Debounced: a slider drag would otherwise re-encode the whole config on every step.
  clearTimeout(saveTimer);
  saveTimer = window.setTimeout(writeConfig, 400);
}

function writeConfig(): void {
  saveTimer = 0;
  const json = JSON.stringify(config);
  history.replaceState(null, '', `#${btoa(encodeURIComponent(json))}`);
}

// Best effort for a refresh inside the debounce, which would otherwise reload the state from before the
// last edit. Whether a URL written this late reaches the reload is up to the browser.
window.addEventListener('pagehide', () => {
  if (saveTimer === 0) return;
  clearTimeout(saveTimer);
  writeConfig();
});

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

rebuildPipeGeometry();
applyView(panel.viewOptions);
viewer.frameBounds(sceneBounds());
viewer.start();

// Audio needs a user gesture, so the whole viewport is the start button until it is
// running.
overlayEl.addEventListener('click', () => {
  void audio.start().then(() => {
    panel.setRunning(true);
    overlayEl.classList.add('hidden');
  });
});

document.addEventListener('keydown', (e) => {
  if (e.target instanceof HTMLInputElement || e.target instanceof HTMLSelectElement) return;
  if (e.code === 'Space') {
    e.preventDefault();
    void audio.toggle().then((running) => {
      panel.setRunning(running);
      overlayEl.classList.toggle('hidden', running);
    });
  }
});

function must<T extends HTMLElement>(selector: string): T {
  const node = document.querySelector<T>(selector);
  if (!node) throw new Error(`main: missing required element ${selector}`);
  return node;
}
