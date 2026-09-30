/**
 * Wiring. Owns the single source of truth for the configuration and keeps the three
 * consumers — audio thread, 3D scene, control panel — pointed at the same object.
 *
 * Data flow:
 *   panel / 3D handles  ->  config (mutated in place)  ->  EngineHost.setGraph/setEngine
 *                                                      ->  PipeMesh.rebuild, JointMesh.rebuild
 *   worklet snapshot @60 Hz  ->  EngineMesh.update, PipeMesh.update, Panel, Scope
 */

import * as THREE from 'three';
import { AudioEngine } from './audio/AudioEngine.js';
import type { EngineHost } from './audio/EngineHost.js';
import {
  ENGINE_PRESETS,
  defaultConfig,
  engineLength,
  exhaustPortDiameter,
  physicalBank,
  presetEngine,
  type EngineConfig,
  type EngineSnapshot,
  type EngineSpec,
} from './model/spec.js';
import { EngineMesh } from './scene/EngineMesh.js';
import { PipeEditor } from './scene/PipeEditor.js';
import { PipeMesh, PressureScale } from './scene/PipeMesh.js';
import { ductDirections, freezeHeadings, layoutGraph, pipesMeetAt, type ExhaustPlacement } from './scene/exhaustLayout.js';
import { JointMesh } from './scene/jointMesh.js';
import { TurboMesh } from './scene/TurboMesh.js';
import { engineFile, engineFileName, readConfig, readEngineFile } from './model/engineFile.js';
import { launchSettingsJson, readLaunchSettings } from './model/launchSettings.js';
import { detachDuct, removePipe, reshapeBendKeepingLength, slideBend, splitDuct } from './scene/drawing.js';
import {
  applyHeader,
  seatLengthwaysHeaders,
  bankCylinders,
  bankMirror,
  defaultMerge,
  runnerBore,
} from './scene/headerTool.js';
import { seatEngineTurbos } from './scene/engineTurbos.js';
import {
  matchLength,
  moveJunction,
  moveTurbo,
  refitBends,
  seatHeaders,
  seatManifolds,
  seatTurbos,
  turboHeight,
} from './scene/turboPlacement.js';
import {
  lockedFrom,
  graphTurboSize,
  engineTurboSettings,
  newTurbo,
  placeTurbo,
  setTurbosSynced,
  turbosSynced,
  removeTurbo,
  turboPortsOf,
} from './model/turbo.js';
import {
  carriedGeometry,
  compileExhaust,
  graphFromJson,
  addAtJunction,
  copyToSiblingRunners,
  defaultDuctId,
  disconnectEnd,
  drawnSegments,
  hasBeenEdited,
  placeLoosePipe,
  removeDuct,
  reversedDucts,
  siblingRunners,
  type DuctDirections,
  type ExhaustGraph,
} from './model/exhaustGraph.js';
import { Viewer } from './scene/Viewer.js';
import { Panel, SAMPLE_RATES, type ViewOptions } from './ui/Panel.js';
import { Scope } from './ui/Scope.js';
import { LaunchSheet } from './ui/LaunchSheet.js';
import { LagNotice } from './ui/LagNotice.js';

const viewportEl = must<HTMLElement>('#viewport');
const panelEl = must<HTMLElement>('#panel');
const scopeEl = must<HTMLCanvasElement>('#scope');
const overlayEl = must<HTMLElement>('#overlay');
const hudEl = must<HTMLElement>('#hud');
const toolsEl = must<HTMLElement>('#tools');

const saved = savedState();
const config: EngineConfig = loadConfig(saved);
/**
 * The duct graph, seeded from the layout spec and authoritative thereafter.
 *
 * The renderer and the solver work from the same graph, so what is on screen is what is being solved.
 * It is re-seeded when the *topology* changes — a different cylinder count or merge plan makes
 * whatever was drawn meaningless — when the ports move while the exhaust is still as compiled, and when
 * a preset is loaded. `reseedGraph` preserves as much of the drawn geometry as the new topology can carry.
 */
config.graph ??= compileExhaust(config.engine, config.pipe, config.collector);


/**
 * The audio sample rate, kept per device rather than in the URL: it is a choice about what this
 * machine can afford, not part of the engine, so a shared link should not carry a phone's setting.
 */
const SAMPLE_RATE_KEY = 'engine-simulator:sampleRate';
const sampleRate = loadSampleRate();
// A touch screen stands in for "probably a phone": the device where the audio thread runs late, and
// where a larger buffer turns a late block into a little delay rather than crackle.
//
// In the desktop app the simulation runs natively instead, and nothing is loaded into the page for it.
const audio: EngineHost =
  import.meta.env.VITE_TARGET === 'desktop'
    ? new (await import('./audio/NativeEngine.js')).NativeEngine(config, sampleRate)
    : new AudioEngine(config, sampleRate, matchMedia('(pointer: coarse)').matches ? 'playback' : 'interactive');
const viewer = new Viewer(viewportEl);
const engineMesh = new EngineMesh(config.engine);
/**
 * One mesh per duct in the graph, in the graph's order.
 *
 * The editor's handles sit on one duct at a time, `editedDuctId`. Every duct has its own
 * `PipeSegment[]`; the panel copies an edit to the other runners when they are linked.
 */
const pipeMeshes: PipeMesh[] = [];
/** The pressure scale every duct is coloured on. */
const pipeScale = new PressureScale();
/**
 * Which duct the drag handles are attached to.
 *
 * Starts on the graph's default duct (`defaultDuctId`, the same rule the panel uses), and follows
 * whichever duct is picked in the scene. Reset to it whenever the graph is reseeded.
 */
let editedDuctId = defaultDuctId(config.graph!) ?? 'runner0';
/** The junction selected in the scene, if one is. Segments and junctions are selected one or the other. */
let selectedJoint: string | null = null;
/** The turbo selected in the scene, if one is. */
let selectedTurbo: string | null = null;
/** One mesh per turbo, in the order of `config.graph.turbos`. */
const turboMeshes: TurboMesh[] = [];
/** One mesh per junction, blended from the pipes that meet there. */
const jointMeshes: JointMesh[] = [];
/** Which junction each of `jointMeshes` is, by index. */
let jointNodes: string[] = [];
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
 * it would end short of the junction, with a gap between. So its end is left open instead, which is what
 * cutting a real pipe short does. Pipes that join it in a fitted bend are fitted again first, so they
 * follow it wherever it goes; only a pipe drawn into it without one can fall short. If the edited pipe was
 * the one the junction follows, it is those *others* that stop meeting, and it is still the edited one
 * that comes off.
 */
function detachIfShort(ductId: string): void {
  const graph = config.graph!;
  const duct = graph.ducts.find((d) => d.id === ductId);
  if (!duct || duct.to.kind !== 'node') return;
  const ports = Array.from({ length: engineMesh.bankCount }, (_, b) => engineMesh.exhaustPort(b));
  // The bends into the junction follow what they join first, so it is only a drawn pipe that falls short.
  refitBends(graph, ports, config.engine);
  if (!pipesMeetAt(graph, layoutGraph(ports, graph, turboPortsOf(graph, config.engine)), duct.to.node)) {
    disconnectEnd(graph, duct.id, directionsOf(stablePlacement));
  }
}

/** With "Apply to every cylinder" on, copy runner `ductId`'s shape onto the others (`copyToSiblingRunners`). */
function mirrorRunners(ductId: string): void {
  const graph = config.graph!;
  const duct = graph.ducts.find((d) => d.id === ductId);
  if (panel.runnersLinked && duct) copyToSiblingRunners(graph, duct, config.engine);
}

/** With the runners linked, an edit reshapes them all: any left short of its junction comes off it (`detachIfShort`). */
function detachRunnersIfShort(): void {
  if (!panel.runnersLinked) return;
  for (const d of config.graph!.ducts) if (d.from.kind === 'valve') detachIfShort(d.id);
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
    onChange: (commit, edited) => {
      freeze();
      if (edited) mirrorRunners(edited);
      // Once an edit has settled — not on every frame of a drag, nor while a route is being drawn.
      if (!editor.dragging && !editor.drawing) {
        detachIfShort(editedDuctId);
        detachRunnersIfShort();
      }
      rebuildPipeGeometry();
      panel.syncPipe();
      panel.syncTurbos();
      // Rebuilding the solver's ducts reallocates and briefly ramps the audio, so during
      // a drag it waits: the editor always sends a final commit on release.
      // A settled edit is kept in the link too, so a reload comes back to it.
      if (commit) {
        audio.setGraph(config.graph!);
        saveConfig();
      }
    },
    onSelect: (i) => {
      selectJoint(null);
      panel.setSelected(i);
    },
    onPick: (pick) => {
      if (pick?.kind === 'turbo') {
        selectJoint(null);
        selectTurbo(pick.turbo);
        editor.select(null);
        panel.setSelected(null);
        return;
      }
      selectTurbo(null);
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
      // Against the panel's too: a pipe drawn since the edited one went can have been given its id.
      if (pick.duct !== editedDuctId || panel.ductId !== pick.duct) {
        editedDuctId = pick.duct;
        panel.showDuct(pick.duct);
        rebuildPipeGeometry();
      }
      editor.select(pick.segment);
      panel.setSelected(pick.segment);
    },
    onMenu: (_pick, x, y) => panel.openMenu(x, y),
    onDrawing: (active) => panel.setDrawingState(active),
    onAim: (aim) => panel.setDrawAim(aim),
    onBendAim: (aim) => panel.setBendAim(aim),
    onBent: (id, length) => {
      freeze();
      const duct = config.graph!.ducts.find((d) => d.id === id);
      // Bent where it lies it kept its length, but for the bend fitted into what it joins, which is fitted
      // again from wherever the pipe now reaches: fitted to the length again, all of it is.
      if (duct?.fitted) {
        const ports = Array.from({ length: engineMesh.bankCount }, (_, b) => engineMesh.exhaustPort(b));
        refitBends(config.graph!, ports, config.engine);
        matchLength(config.graph!, ports, config.engine, id, length);
      }
      mirrorRunners(id);
      detachIfShort(id);
      detachRunnersIfShort();
      rebuildPipeGeometry();
      panel.rebuildPipeList();
      audio.setGraph(config.graph!);
      saveConfig();
    },
    onPlaceTurbo: ({ position, rotation, attach }) => {
      freeze();
      const graph = config.graph!;
      const mount = newTurbo(graph, position, rotation);
      // Out of sync, a new turbo starts from the engine's settings, as its own.
      if (!turbosSynced(graph)) mount.settings = engineTurboSettings(config.engine);
      placeTurbo(graph, mount, attach);
      afterTurboEdit(true);
      selectTurbo(mount.id);
    },
    onToolEnded: () => panel.toolsEnded(),
    onPlacePipe: (position) => {
      freeze();
      const id = placeLoosePipe(config.graph!, position, exhaustPortDiameter(config.engine), engineLength(config.engine));
      editedDuctId = id;
      afterTurboEdit(true);
      panel.showDuct(id);
      editor.select(0);
      panel.setSelected(0);
      rebuildPipeGeometry();
    },
    onMoveJunction: (node, position, axis, commit) => {
      freeze();
      const ports = Array.from({ length: engineMesh.bankCount }, (_, b) => engineMesh.exhaustPort(b));
      moveJunction(config.graph!, ports, config.engine, node, position, axis);
      afterTurboEdit(commit);
    },
    onHeaderAim: (aim) => panel.setHeaderAim(aim),
    onHeaderLength: (length) => panel.setHeaderLength(length),
    onApplyHeader: (builds) => {
      freeze();
      const graph = config.graph!;
      for (const { plan, primaries } of builds) applyHeader(graph, plan, primaries, directionsOf(stablePlacement));
      panel.setHeaderToolState(false);
      afterTurboEdit(true);
    },
    onMoveTurbo: (id, position, rotation, commit) => {
      freeze();
      const ports = Array.from({ length: engineMesh.bankCount }, (_, b) => engineMesh.exhaustPort(b));
      moveTurbo(config.graph!, ports, config.engine, id, position, rotation);
      afterTurboEdit(commit);
    },
  },
);

/** Rebuild and resend after a turbo was placed, moved or taken out. */
function afterTurboEdit(commit: boolean): void {
  rebuildPipeGeometry();
  panel.rebuildPipeList();
  panel.syncTurbos();
  if (commit) {
    audio.setGraph(config.graph!);
    saveConfig();
  }
}

/** Select a turbo, or none: highlight it and describe it in the panel. */
function selectTurbo(id: string | null): void {
  selectedTurbo = id;
  editor.setSelectedTurbo(id);
  const turbos = config.graph!.turbos ?? [];
  turboMeshes.forEach((m, i) => m.setSelected(turbos[i]?.id === id));
  panel.showTurbo(id);
}


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

  if (selectedTurbo) {
    freeze();
    if (removeTurbo(config.graph!, selectedTurbo, directionsOf(stablePlacement))) {
      selectTurbo(null);
      afterTurboEdit(true);
    } else {
      panel.notify('Pipes carry on from this turbo’s outlet pipe: delete them first.');
    }
    e.preventDefault();
    return;
  }

  // A junction is not deleted on its own: it goes when the pipes attached to it do, dissolving once only one
  // is left, and a pipe running straight through it rejoins.
  if (selectedJoint) {
    panel.notify('A junction goes when its pipes do: delete the pipes attached to it.');
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

const panel = new Panel(panelEl, toolsEl, config, {
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
     * lengths are cut to the spacing of the ports, so after a bore change they would not reach, and
     * would leave gaps at the junctions. An exhaust the user has edited is theirs, and is left alone.
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
    detachRunnersIfShort();
    rebuildPipeGeometry();
    audio.setGraph(config.graph!);
    saveConfig();
  },
  onExportEngine: () => exportEngine(),
  onImportEngine: (text) => importEngine(text),
  onBendTool: (on) => editor.setBendTool(on),
  onBendSegment: (id, index) => {
    editor.startBend(id, index);
  },
  onHeaderTool: (on) => {
    if (!on) {
      editor.setHeaderTool(null);
      return;
    }
    const graph = config.graph!;
    const spec = config.engine;
    const ports = Array.from({ length: engineMesh.bankCount }, (_, b) => engineMesh.exhaustPort(b));
    // The bank of the pipe being edited, if it is a cylinder's.
    const edited = graph.ducts.find((d) => d.id === editedDuctId);
    const bank = edited?.from.kind === 'valve' ? physicalBank(spec, edited.from.cylinder) : 0;
    // Starting out beside that bank's ports, with nothing picked: the user picks the openings.
    const portBore = exhaustPortDiameter(spec);
    const { merge, axis } = defaultMerge(
      bankCylinders(spec, bank).map((cylinder) => ({
        opening: { kind: 'port' as const, cylinder },
        point: ports[cylinder]!.position,
        dir: ports[cylinder]!.direction,
        bore: runnerBore(graph, cylinder, portBore),
      })),
    );
    editor.setHeaderTool({
      merge,
      axis,
      length: 0,
      picked: new Set(),
      banks: ports.map((_, cylinder) => physicalBank(spec, cylinder)),
      mirror: bankMirror(spec),
      mirrored: panel.headerMirrored,
      portBore,
      lengthSet: false,
    });
  },
  onHeaderLength: (length) => editor.setHeaderLength(length),
  onHeaderMirror: (on) => editor.setHeaderMirrored(on),
  onApplyHeader: () => editor.applyHeader(),
  onReshapeBend: (id, index, angle, radius, changed) => {
    const duct = config.graph!.ducts.find((d) => d.id === id);
    if (!duct) return;
    freeze();
    if (!reshapeBendKeepingLength(duct.segments, index, angle, radius, changed)) return;
    mirrorRunners(id);
    detachIfShort(id);
    detachRunnersIfShort();
    rebuildPipeGeometry();
    panel.rebuildPipeList();
    audio.setGraph(config.graph!);
    saveConfig();
  },
  onSlideBend: (id, index, before) => {
    const duct = config.graph!.ducts.find((d) => d.id === id);
    if (!duct) return;
    freeze();
    if (!slideBend(duct.segments, index, before)) return;
    mirrorRunners(id);
    detachIfShort(id);
    detachRunnersIfShort();
    rebuildPipeGeometry();
    panel.rebuildPipeList();
    audio.setGraph(config.graph!);
    saveConfig();
  },
  onMatchLength: (id, length) => {
    freeze();
    const ports = Array.from({ length: engineMesh.bankCount }, (_, b) => engineMesh.exhaustPort(b));
    const fit = matchLength(config.graph!, ports, config.engine, id, length);
    if (!fit) return;
    if (!fit.reached) {
      panel.notify(`That is as near as it goes: ${Math.round(fit.length * 1000)} mm, with its bends no tighter than they may be.`);
    }
    rebuildPipeGeometry();
    panel.rebuildPipeList();
    audio.setGraph(config.graph!);
    saveConfig();
  },
  onRemoveDuct: (id) => {
    freeze();
    // A fitted pipe by itself, what it joined put back as it was; a straight one whole, with what joins it.
    removePipe(config.graph!, id, stablePlacement, directionsOf(stablePlacement));
    rebuildPipeGeometry();
    audio.setGraph(config.graph!);
    saveConfig();
  },
  onDetachDuct: (id) => {
    freeze();
    const graph = config.graph!;
    if (!detachDuct(graph, id, stablePlacement, directionsOf(stablePlacement))) {
      panel.notify('Pipes carry on from where this pipe joins: delete them first.');
      return;
    }
    rebuildPipeGeometry();
    audio.setGraph(graph);
    saveConfig();
  },
  onSplitDuct: (id, index) => {
    freeze();
    const graph = config.graph!;
    const place = stablePlacement?.ducts.get(id);
    const duct = graph.ducts.find((d) => d.id === id);
    if (!duct || !place) return;
    // With the runners linked the others lose the same segment, each leaving its own loose pipe, or the
    // next edit to one of them would copy the deleted segment back.
    const siblings = panel.runnersLinked ? siblingRunners(graph, duct) : [];
    if (!splitDuct(graph, id, index, place)) return;
    for (const other of siblings) {
      const at = stablePlacement?.ducts.get(other.id);
      if (at && index < drawnSegments(other).length - 1) splitDuct(graph, other.id, index, at);
    }
    // Split at its first segment, a pipe is left with nothing in it, and goes unless it is a cylinder's.
    if (duct.segments.length === 0 && duct.from.kind !== 'valve') {
      removeDuct(graph, id, directionsOf(stablePlacement));
    }
    rebuildPipeGeometry();
    audio.setGraph(config.graph!);
    saveConfig();
  },
  onDrawMode: (on) => {
    editor.setDrawMode(on);
  },
  onAddAtJunction: (node, kind) => {
    if (kind !== 'pipe' && kind !== 'chamber') return;
    freeze();
    const id = addAtJunction(config.graph!, node, kind);
    if (!id) return;
    selectJoint(null);
    editedDuctId = id;
    panel.showDuct(id);
    rebuildPipeGeometry();
    editor.select(0);
    panel.setSelected(0);
    audio.setGraph(config.graph!);
    saveConfig();
  },
  onDrawFromJoint: (node) => {
    editor.setDrawMode(true);
    editor.startAtJunction(node);
  },
  onPlaceMode: (on) => editor.setPlaceMode(on),
  onPlacePipeMode: (on) => editor.setPlaceMode(on, 'pipe'),
  onRemoveTurbo: (id) => {
    freeze();
    if (!removeTurbo(config.graph!, id, directionsOf(stablePlacement))) {
      panel.notify('Pipes carry on from this turbo’s outlet pipe: delete them first.');
      return;
    }
    selectTurbo(null);
    afterTurboEdit(true);
  },
  onTurbosSynced: (synced) => {
    const patch = setTurbosSynced(config.graph!, config.engine, synced);
    if (patch) {
      Object.assign(config.engine, patch);
      audio.setEngine(patch);
    }
    audio.setGraph(config.graph!);
    panel.syncTurbos();
    saveConfig();
  },
  onTurboSettings: (id, settings) => {
    const mount = config.graph!.turbos?.find((t) => t.id === id);
    if (!mount) return;
    if (settings) mount.settings = settings;
    else delete mount.settings;
    // Only the turbo is resized: the solver keeps the gas in the pipes.
    audio.setGraph(config.graph!);
    panel.syncTurbos();
    saveConfig();
  },
  onReseed: (turbos, graph) => {
    reseedGraph(true, turbos, graph);
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
  onLaunch: (launchConfig) => {
    if (!launchConfig) {
      audio.launch(null);
      return;
    }
    // A run needs the engine running; starting it is what the button asks for.
    const ready = audio.running
      ? Promise.resolve()
      : audio.start().then(() => {
          panel.setRunning(true);
          overlayEl.classList.add('hidden');
        });
    void ready.then(() => {
      launchSheet.begin(launchConfig);
      audio.launch(launchConfig);
    });
  },
  onSampleRate: changeSampleRate,
  onView: applyView,
  onResetView: () => viewer.frameBounds(sceneBounds()),
  onLaunchSettings: saveConfig,
}, sampleRate, readLaunchSettings(saved?.launch));

const lagNotice = new LagNotice(must<HTMLElement>('#stage'), (hz) => {
  panel.setSampleRate(hz);
  changeSampleRate(hz);
});
audio.onLag((behind) => lagNotice.update(behind, audio.sampleRate));

function changeSampleRate(hz: number): void {
  try {
    localStorage.setItem(SAMPLE_RATE_KEY, String(hz));
  } catch {
    // Storage can be off (private browsing); the choice then lasts until reload.
  }
  void audio.setSampleRate(hz);
}

const scope = new Scope(scopeEl, audio);
const launchSheet = new LaunchSheet(must<HTMLElement>('#stage'));

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
    'exhaustHeaders' in partial ||
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
    'exValveCount' in partial ||
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
 * what loading a preset wants, or is `drawn`, the exhaust a preset has drawn for it.
 */
function reseedGraph(fromConfig = false, turbos?: number, drawn?: ExhaustGraph): void {
  // As many turbos as there were, unless a preset says how many it has.
  const count = turbos ?? config.graph?.turbos?.length ?? 0;
  if (!fromConfig) {
    // Carry the drawn geometry across: a runner that was lengthened should stay lengthened even
    // though its routing cannot survive a change of topology.
    const carried = config.graph ? carriedGeometry(config.graph) : {};
    if (carried.pipe) config.pipe = carried.pipe;
    if (carried.collector) config.collector = carried.collector;
  }
  const graph = (drawn ? graphFromJson(drawn) : null) ?? compileExhaust(config.engine, config.pipe, config.collector, count);
  config.graph = graph;
  editedDuctId = defaultDuctId(graph) ?? editedDuctId;
}

function rebuildPipeGeometry(): void {
  const cylinders = engineMesh.bankCount;
  const graph = config.graph!;

  // Seated first: building a compiled manifold or header can add pipes.
  const ports = Array.from({ length: cylinders }, (_, b) => engineMesh.exhaustPort(b));
  seatManifolds(graph, ports);
  seatLengthwaysHeaders(graph, ports, config.engine);
  seatHeaders(graph, ports);
  seatEngineTurbos(graph, ports, config.engine);
  seatTurbos(graph, ports, config.engine);
  refitBends(graph, ports, config.engine);

  // One mesh per duct, and a mark for each junction of two or more pipes.
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

  const turboPorts = turboPortsOf(graph, config.engine);
  const placement = layoutGraph(ports, graph, turboPorts);
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

  // One mesh per turbo, where it was put.
  const turbos = graph.turbos ?? [];
  while (turboMeshes.length < turbos.length) {
    const m = new TurboMesh();
    turboMeshes.push(m);
    viewer.scene.add(m.group);
  }
  while (turboMeshes.length > turbos.length) {
    const m = turboMeshes.pop()!;
    viewer.scene.remove(m.group);
    m.dispose();
  }
  const size = graphTurboSize(graph, config.engine);
  turbos.forEach((t, i) => {
    turboMeshes[i]!.rebuild(size);
    if (t.position) turboMeshes[i]!.place(t.position, t.rotation);
    turboMeshes[i]!.setSelected(t.id === selectedTurbo);
  });
  if (selectedTurbo && !turbos.some((t) => t.id === selectedTurbo)) selectTurbo(null);

  editor.setDrawContext({
    graph,
    placement,
    ports,
    meshes: pipeMeshes,
    joints: bodies.map(([node], i) => ({ node, target: jointMeshes[i]!.pickTarget })),
    turbos: turbos.map((t, i) => ({ turbo: t.id, target: turboMeshes[i]!.pickTarget })),
  });
  editor.portDiameter = exhaustPortDiameter(config.engine);
  editor.setLoosePipe(engineLength(config.engine), exhaustPortDiameter(config.engine));
  editor.turboSize = size;
  editor.turboHeight = turboHeight(ports);
  editor.turboOutletDia = size.outletDia;

  /**
   * The editor's handles belong to one duct, so they must use the frame that duct was actually built
   * with — not the bare port direction. With a collector those differ by however far the runner had to
   * be aimed to reach the collar, and handles laid out on the port direction would sit off the pipe.
   */
  const editedDuct = graph.ducts.find((d) => d.id === editedDuctId) ?? graph.ducts[0];
  // The edited pipe gone, the one the handles go to instead is the edited one, as it is the panel's.
  if (editedDuct) editedDuctId = editedDuct.id;
  editor.lockedFrom = editedDuct ? lockedFrom(editedDuct) : null;
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
  for (const m of pipeMeshes) {
    m.setPressureVisible(v.pressure);
  }
  editor.setHandlesVisible(v.handles);
  timeScale = v.speed;
  audio.setTimeScale(v.speed);
}

// ---------------------------------------------------------------------------
// Snapshot -> visuals
// ---------------------------------------------------------------------------

let latest: EngineSnapshot | null = null;
/** Crank angle the renderer is showing, extrapolated between snapshots. */
let displayAngle = 0;
let displayRpm = 0;
/** Share of real time the simulation runs at, which the crank is turned between snapshots at too. */
let timeScale = 1;

audio.onSnapshot((s) => {
  latest = s;
  displayRpm = s.rpm;
  // Snapshots arrive at 60 Hz but the crank may be turning at 160 rev/s, so the
  // angle is advanced continuously between them and only nudged towards the
  // authoritative value. Snapping straight to it makes the piston strobe.
  const err = shortestAngle(s.crankAngle - displayAngle);
  displayAngle += err * 0.25;
  // Each duct shows its own cells, all on one scale so their colours compare. A duct the solver has
  // that is not drawn, a turbo's exit, is passed over.
  const scale = pipeScale.track(s.ductPressure);
  // A pipe the solver has turned round has its cells from its far end, so they are read back.
  const ducts = config.graph!.ducts;
  const reversed = reversedDucts(config.graph!);
  let at = 0;
  s.ductIds.forEach((id, k) => {
    const n = s.ductCells[k]!;
    const i = ducts.findIndex((d) => d.id === id);
    const cells = s.ductPressure.subarray(at, at + n);
    if (i >= 0) pipeMeshes[i]?.update(reversed.has(id) ? cells.slice().reverse() : cells, scale);
    at += n;
  });
  launchSheet.onSnapshot(s.launch);
  panel.updateReadouts(s);
  hudEl.textContent =
    `${Math.round(s.rpm)} rpm · ${(s.cylPressure / 1e5).toFixed(1)} bar · ` +
    `${Math.round(s.cylTemp)} K · wall ${Math.round(s.wallTemp)} K · ` +
    `${s.pipeCells} cells × ${s.substeps}`;
});

viewer.onFrame((dt) => {
  if (displayRpm > 0) displayAngle = (displayAngle + displayRpm * 6 * dt * timeScale) % 720;
  // Combustion glow: a short flash after the burn begins.
  if (latest) {
    // Extrapolate each bank from its own snapshot angle, keeping the phase relationship.
    const posed = latest.banks.map((b) => ({
      ...b,
      crankAngle:
        (displayAngle + (b.crankAngle - latest!.banks[0]!.crankAngle) + 1440) % 720,
    }));
    engineMesh.intakeCamAdvance = latest.intakeCamAdvance;
    engineMesh.exhaustCamRetard = latest.exhaustCamRetard;
    engineMesh.highCam = latest.highCam;
    engineMesh.update(
      posed,
      posed.map((b) => glow(b.crankAngle, config.engine.ignition)),
    );
  }
  scope.draw();
  launchSheet.draw();
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

/**
 * Where the app starts without a configuration in the URL: the first preset, idling.
 *
 * The app always runs the crank free, with the speed following the throttle and the load, so
 * `freeRunning` is set here and carried across presets; a held speed is a measurement setting only.
 */
function startingConfig(): EngineConfig {
  const cfg = defaultConfig();
  const first = ENGINE_PRESETS[0]!;
  cfg.engine = presetEngine(first, { ...cfg.engine, freeRunning: true });
  cfg.pipe = first.pipe();
  if (first.collector) cfg.collector = first.collector();
  return cfg;
}

/** What the URL holds, or `null` where it holds nothing that can be read. */
function savedState(): { launch?: unknown } | null {
  const hash = location.hash.replace(/^#/, '');
  if (!hash) return null;
  try {
    return JSON.parse(decodeURIComponent(atob(hash))) as { launch?: unknown };
  } catch {
    console.warn('[main] could not read the configuration in the URL; using defaults');
    return null;
  }
}

function loadConfig(saved: unknown): EngineConfig {
  if (!saved) return startingConfig();
  try {
    const { config: read, graphDropped } = readConfig(saved, startingConfig());
    if (graphDropped) console.warn('[main] the exhaust in the URL does not fit this engine; rebuilding it');
    return read;
  } catch {
    console.warn('[main] could not read the configuration in the URL; using defaults');
    return startingConfig();
  }
}

/** Save the engine as it stands, exhaust and all, as a file the user can keep and import again. */
function exportEngine(): void {
  const blob = new Blob([engineFile(config)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = `${engineFileName(config.engine)}.json`;
  document.body.appendChild(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(url), 0);
}

/**
 * Replace the engine with one from an exported file's text: the engine, its pipe and collector, and its
 * exhaust, or one compiled from the pipe and collector where the file's does not fit. Says why in the
 * panel if it cannot be read, and leaves the engine as it was.
 */
function importEngine(text: string): void {
  const read = readEngineFile(text, startingConfig());
  if (!read) {
    panel.notify('That file is not an exported engine.');
    return;
  }
  const next = read.config;
  Object.assign(config.engine, next.engine);
  config.pipe = next.pipe;
  config.collector = next.collector;
  config.graph = next.graph ?? compileExhaust(config.engine, config.pipe, config.collector);
  editedDuctId = defaultDuctId(config.graph) ?? editedDuctId;
  selectTurbo(null);
  selectJoint(null);
  editor.select(null);
  audio.setEngine(config.engine);
  engineMesh.setSpec(config.engine);
  rebuildPipeGeometry();
  audio.setGraph(config.graph);
  panel.rebuildAll();
  viewer.frameBounds(sceneBounds());
  saveConfig();
  if (read.graphDropped) panel.notify('Its exhaust does not fit its engine, so one was built from its pipe and collector.');
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
  // The launch's car and settings ride along with the engine, so a refresh keeps them too.
  const json = JSON.stringify({ ...config, launch: launchSettingsJson(panel.launchSettings()) });
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
// running. The desktop app needs none, and starts at once.
if (import.meta.env.VITE_TARGET === 'desktop') {
  void audio.start().then(() => {
    panel.setRunning(true);
    overlayEl.classList.add('hidden');
  });
}
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
