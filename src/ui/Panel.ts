/**
 * Hand-rolled control panel.
 *
 * The segment list edits the selected duct's own `PipeSegment[]` in the exhaust graph, the
 * same array the 3D handles edit when they are on that duct, so this never owns a copy of
 * the geometry — it renders whatever the array currently says
 * and writes edits straight back. `syncPipe` refreshes the input values in place
 * (called while the user drags handles in 3D) and `rebuildPipeList` is reserved for
 * structural changes, so typing in a field is never interrupted by a re-render.
 */

import {
  copyToSiblingRunners,
  defaultDuctId,
  ductLabel,
  endsAt,
  nodeOrder,
  pathToAir,
  type ExhaustDuct,
  type ExhaustGraph,
} from '../model/exhaustGraph.js';
import {
  ENGINE_PRESETS,
  bankFiringIntervals,
  exhaustLayoutOf,
  firingOffsetDeg,
  firingPlan,
  fullLoadTorque,
  isBoxer,
  presetEngine,
  type ExhaustLayout,
  type EngineConfig,
  type EngineSnapshot,
  type EngineSpec,
  type PipeSegment,
  type SegmentKind,
  makeSegment,
  segmentDiameter,
  speedOfSound,
  totalPipeLength,
} from '../model/spec.js';

export interface PanelCallbacks {
  onEngine: (partial: Partial<EngineSpec>) => void;
  /** The selected duct's segments were mutated in place, and copied to the other runners if linked. */
  onPipe: () => void;
  onSelect: (index: number | null) => void;
  /** The user picked a different duct to edit. */
  onSelectDuct: (id: string) => void;
  /** `config.pipe` and `config.collector` were replaced; rebuild the graph from them. */
  onReseed: () => void;
  /** Draw mode was switched on or off. */
  onDrawMode: (on: boolean) => void;
  /** Start drawing a new pipe out of this junction. */
  onDrawFromJoint: (node: string) => void;
  /** A pipe had its last segment deleted and should go, tidying the junctions around it. */
  onRemoveDuct: (id: string) => void;
  onToggleAudio: () => void;
  /** The user picked a different audio sample rate, Hz. */
  onSampleRate: (hz: number) => void;
  onView: (view: ViewOptions) => void;
  onResetView: () => void;
}

export interface ViewOptions {
  cutaway: boolean;
  pressure: boolean;
  handles: boolean;
}

interface SegmentRow {
  el: HTMLElement;
  kind: HTMLSelectElement;
  length: HTMLInputElement;
  dIn: HTMLInputElement;
  dOut: HTMLInputElement;
  dOutWrap: HTMLElement;
}

const MM = 1000;

/**
 * What the Sample rate menu offers, with the exhaust's band limit at each: the solver's finest cell is
 * `1400 / (fs · 0.85)` and it resolves up to about `c / (5 dx)` with `c` = 400 m/s at the mouth.
 */
export const SAMPLE_RATES: Array<[number, string]> = [
  [48000, '48 kHz · full detail'],
  [32000, '32 kHz · ~63% CPU, exhaust to ~1.5 kHz'],
  [24000, '24 kHz · ~47% CPU, exhaust to ~1.2 kHz'],
];

/** What the Cylinders menu offers: a count, and for six whether it is a V. */
const ENGINE_TYPES: Array<[string, string]> = [
  ['1', 'Single'],
  ['2', 'Twin'],
  ['3', 'Inline three'],
  ['4', 'Inline four'],
  ['5', 'Inline five'],
  ['6', 'Inline six'],
  ['6v', 'V6'],
  ['8', 'V8'],
  ['4b', 'Boxer four'],
  ['6b', 'Boxer six'],
];

/** The Cylinders menu entry for an engine. */
function engineTypeOf(eng: EngineSpec): string {
  if (isBoxer(eng)) return `${eng.cylinders}b`;
  if (eng.cylinders === 6 && eng.vAngle > 0) return '6v';
  return String(eng.cylinders);
}

/** What draw mode says before a route has started. */
const START_HINT = 'Pick a port, a junction, or a pipe to continue or branch from';

export class Panel {
  private readonly listEl: HTMLElement;
  private ductSelect!: HTMLSelectElement;
  /** Which duct the segment list edits. Falls back to the first duct if it disappears. */
  private selectedDuctId = 'runner0';
  /**
   * The graph the current selection belongs to.
   *
   * A preset or a change of cylinder count replaces the graph wholesale, and an id that still exists in
   * the new one is not the same duct: staying on `collector0` after picking a V8 left the list editing a
   * collector buried inside the merge body, where deleting a segment is invisible and nearly inaudible.
   * A new graph therefore resets the selection to the first runner.
   */
  private selectionGraph: ExhaustGraph | null = null;
  /**
   * Whether editing one runner edits them all.
   *
   * On by default because it is what the panel did before ducts were separable, and because a symmetric
   * engine is the normal case — eight identical runners should not need eight identical edits. Turning it
   * off is what unequal-length headers need, and that is now a thing the model can express.
   */
  private linkRunners = true;
  private drawing = false;
  private drawHint!: HTMLElement;
  private drawBtn!: HTMLButtonElement;
  /** Details of the junction selected in the scene, hidden when none is. */
  private jointEl!: HTMLElement;
  private readonly rows: SegmentRow[] = [];
  private cylSel!: HTMLSelectElement;
  private layoutSel!: HTMLSelectElement;
  private twinWrap!: HTMLElement;
  private crankSel!: HTMLSelectElement;
  private crankRow!: HTMLElement;
  private vAngleRow!: HTMLElement;
  private offsetToggleEl!: HTMLElement;
  private offsetWrapEl!: HTMLElement;
  private readonly resyncers: Resync[] = [];
  private readonly statsEl: HTMLElement;
  private readonly rpmEl: HTMLElement;
  private readonly readoutEl: HTMLElement;
  private readonly meterFill: HTMLElement;
  private readonly startBtn: HTMLButtonElement;
  private readonly rpmSlider: HTMLInputElement;
  private readonly rpmRow: HTMLElement;
  private readonly loadRow: HTMLElement;

  private selected: number | null = null;
  private readonly view: ViewOptions = {
    cutaway: true,
    pressure: true,
    handles: true,
  };

  constructor(
    root: HTMLElement,
    private readonly config: EngineConfig,
    private readonly cb: PanelCallbacks,
    sampleRate: number,
  ) {
    const spec = config.engine;

    // ---- Transport -------------------------------------------------------
    const transport = section(root, 'Transport', false);
    this.startBtn = el('button', 'primary', transport) as HTMLButtonElement;
    this.startBtn.textContent = 'Start engine';
    this.startBtn.addEventListener('click', () => this.cb.onToggleAudio());

    const meter = el('div', 'meter', transport);
    this.meterFill = el('div', 'meter-fill', meter);

    this.rpmEl = el('div', 'big-readout', transport);
    this.rpmEl.textContent = '— rpm';
    this.readoutEl = el('div', 'readout', transport);

    const rateRow = el('div', 'row', transport);
    el('label', '', rateRow).textContent = 'Sample rate';
    const rateSel = el('select', '', rateRow) as HTMLSelectElement;
    for (const [hz, label] of SAMPLE_RATES) rateSel.appendChild(option(String(hz), label));
    rateSel.value = String(sampleRate);
    rateSel.addEventListener('change', () => this.cb.onSampleRate(Number(rateSel.value)));
    rateRow.title =
      'The solver takes one step per audio sample, so a lower rate means fewer steps and coarser ' +
      'cells: much less CPU, for a duller exhaust. For phones and slow machines. Changing it ' +
      'restarts the audio, and the pipes warm up again from cold.';

    // ---- Operating point -------------------------------------------------
    const op = section(root, 'Operating point', false);
    const rpmSlider = slider(op, {
      label: 'Engine speed',
      min: 600,
      max: 10000,
      step: 25,
      value: spec.rpm,
      unit: 'rpm',
      onInput: (v) => this.cb.onEngine({ rpm: v }),
    });
    this.rpmSlider = rpmSlider.input;
    this.rpmRow = rpmSlider.row;
    this.resyncers.push(() => rpmSlider.render(this.config.engine.rpm));

    const revLimit = slider(op, {
      label: 'Rev limiter',
      min: 2000,
      max: 12000,
      step: 100,
      value: spec.revLimit,
      unit: 'rpm',
      onInput: (v) => this.cb.onEngine({ revLimit: v }),
    });
    revLimit.row.title =
      'The spark is cut above this and returns once the crank has dropped back, so the engine ' +
      'bounces off it. An engine speed set at or past it revs freely into the limiter.';
    this.resyncers.push(() => revLimit.render(this.config.engine.revLimit));

    slider(op, {
      label: 'Throttle',
      min: 0,
      max: 1,
      step: 0.01,
      value: spec.throttle,
      format: (v) => `${Math.round(v * 100)}%`,
      onInput: (v) => this.cb.onEngine({ throttle: v }),
    });

    const free = toggle(op, 'Free-running crank', spec.freeRunning, (on) => {
      this.cb.onEngine({ freeRunning: on });
      this.rpmRow.classList.toggle('disabled', on);
      this.rpmSlider.disabled = on;
      this.loadRow.classList.toggle('hidden', !on);
    });
    free.title =
      'Integrate crank speed from gas torque, inertia and load instead of holding it fixed. ' +
      'The exhaust tuning then affects the speed the engine settles at.';

    const loadWrap = el('div', 'subgroup', op);
    this.loadRow = loadWrap;
    loadWrap.classList.toggle('hidden', !spec.freeRunning);
    const load = slider(loadWrap, {
      label: 'Load',
      min: 0,
      // Past full throttle's worth, so the engine can be bogged down and stalled.
      max: 1.5,
      step: 0.01,
      value: spec.load,
      format: (v) => `${Math.round(v * 100)}% · ${Math.round(v * fullLoadTorque(this.config.engine))} N·m`,
      onInput: (v) => this.cb.onEngine({ load: v }),
    });
    load.row.title =
      'Braking torque at the crank, as a share of what this engine makes at full throttle, ' +
      'so the same setting loads a single and a V8 alike.';
    // Also redraws the N·m figure, which follows the engine's size.
    this.resyncers.push(() => load.render(this.config.engine.load));
    const flywheel = slider(loadWrap, {
      label: 'Flywheel inertia',
      min: 0.02,
      max: 1.2,
      step: 0.01,
      value: spec.flywheelInertia,
      unit: 'kg·m²',
      onInput: (v) => this.cb.onEngine({ flywheelInertia: v }),
    });
    this.resyncers.push(() => flywheel.render(this.config.engine.flywheelInertia));
    this.rpmRow.classList.toggle('disabled', spec.freeRunning);
    this.rpmSlider.disabled = spec.freeRunning;

    // ---- Engine preset ---------------------------------------------------
    // Whole-engine presets, because a V-twin is a layout as well as a pipe.
    const enginePresetRow = el('div', 'row', transport);
    el('label', '', enginePresetRow).textContent = 'Engine preset';
    const engineSel = el('select', '', enginePresetRow) as HTMLSelectElement;
    engineSel.appendChild(option('', 'Choose…'));
    ENGINE_PRESETS.forEach((p, i) => engineSel.appendChild(option(String(i), p.name)));
    engineSel.addEventListener('change', () => {
      if (!engineSel.value) return;
      const preset = ENGINE_PRESETS[Number(engineSel.value)]!;
      engineSel.title = preset.description;
      engineSel.value = '';
      this.selected = null;
      /**
       * Engine first, then the preset's own pipework, then one rebuild from it.
       *
       * `onEngine` re-seeds the exhaust itself when the topology changes (or the ports move on an
       * exhaust that has not been edited), carrying the previous
       * engine's runner and collector across so an edit survives a change of cylinder count. Loading
       * the preset's geometry *before* it let that carry-over overwrite the geometry just loaded, so
       * going crossplane to flatplane and back came out with a different exhaust each time. And the
       * collector is always replaced — emptied if the preset has none — so no preset inherits one.
       */
      this.cb.onEngine(presetEngine(preset, this.config.engine));
      this.config.pipe.length = 0;
      this.config.pipe.push(...preset.pipe());
      this.config.collector.length = 0;
      if (preset.collector) this.config.collector.push(...preset.collector());
      this.cb.onReseed();
      this.rebuildAll();
      this.cb.onResetView();
    });

    // ---- Layout ----------------------------------------------------------
    const layout = section(root, 'Layout', false);

    const cylRow = el('div', 'row', layout);
    el('label', '', cylRow).textContent = 'Cylinders';
    const cylSel = el('select', '', cylRow) as HTMLSelectElement;
    this.cylSel = cylSel;
    for (const [value, label] of ENGINE_TYPES) cylSel.appendChild(option(value, label));
    cylSel.value = engineTypeOf(spec);

    const multiWrap = el('div', 'subgroup', layout);
    this.twinWrap = multiWrap;
    const showMulti = () => multiWrap.classList.toggle('hidden', this.config.engine.cylinders < 2);

    cylSel.addEventListener('change', () => {
      const boxer = cylSel.value.endsWith('b');
      const vee = cylSel.value === '6v' || cylSel.value === '8' || boxer;
      const n = parseInt(cylSel.value, 10) as EngineSpec['cylinders'];
      const eng = this.config.engine;
      this.cb.onEngine({
        cylinders: n,
        // Keep the plumbing sensible for the new count: a single has nothing to merge, and a
        // V engine's default is a collector per bank.
        exhaustLayout: n === 1 ? 'open' : vee ? 'perBank' : 'merged',
        // A V angle is what makes a six a V6, and means nothing on an inline engine. A twin keeps
        // whatever angle it had, which is how it chooses between parallel and V. A boxer is its
        // banks laid flat, 180 degrees apart, on a crank of its own.
        ...(cylSel.value === '8' ? { vAngle: 90 } : {}),
        ...(cylSel.value === '6v' ? { vAngle: 60 } : {}),
        ...(boxer ? { vAngle: 180, crankType: 'boxer' as const } : {}),
        ...(n !== 2 && !vee ? { vAngle: 0 } : {}),
        // Leaving a boxer hands the crank back: a V8 gets the crank its menu shows.
        ...(!boxer && eng.crankType === 'boxer'
          ? { crankType: n === 8 ? ('crossplane' as const) : ('shared' as const) }
          : {}),
      });
      showMulti();
      this.syncLayoutOptions();
      this.rebuildPipeList();
      this.syncStats();
    });

    const layoutRow = el('div', 'row', multiWrap);
    el('label', '', layoutRow).textContent = 'Exhaust';
    const layoutSel = el('select', '', layoutRow) as HTMLSelectElement;
    this.layoutSel = layoutSel;
    layoutSel.addEventListener('change', () => {
      this.cb.onEngine({ exhaustLayout: layoutSel.value as EngineSpec['exhaustLayout'] });
      this.rebuildPipeList();
      this.syncStats();
    });
    layoutRow.title =
      'A shared collector lets each cylinder\u2019s pulse travel up the other primaries, where it ' +
      'either helps scavenge those cylinders or blocks them. That cross-talk is most of what ' +
      'makes a shared-header engine sound unlike several singles \u2014 and, on a V8, the whole ' +
      'difference between a crossplane and a flatplane crank.';

    const crankRow = el('div', 'row', multiWrap);
    this.crankRow = crankRow;
    el('label', '', crankRow).textContent = 'Crank';
    const crankSel = el('select', '', crankRow) as HTMLSelectElement;
    this.crankSel = crankSel;
    crankSel.appendChild(option('crossplane', 'Crossplane (American V8)'));
    crankSel.appendChild(option('flatplane', 'Flatplane (Ferrari)'));
    crankSel.value = spec.crankType === 'flatplane' ? 'flatplane' : 'crossplane';
    crankSel.addEventListener('change', () => {
      this.cb.onEngine({ crankType: crankSel.value as EngineSpec['crankType'] });
      this.syncStats();
    });
    crankRow.title =
      'Both fire every 90 degrees, so through one collector they sound much the same. The ' +
      'difference is which bank each firing belongs to: a crossplane deals them out 180-90-180-270 ' +
      'down each bank, a flatplane evenly every 180. Give each bank its own collector and that ' +
      'is the burble against the shriek.';

    const vRow = slider(multiWrap, {
      label: 'V angle',
      min: 0,
      max: 120,
      step: 1,
      value: spec.vAngle,
      format: (v) => {
        const off = firingOffsetDeg({ ...this.config.engine, vAngle: v });
        const named = v === 0 ? ' parallel' : v === 45 ? ' Harley' : v === 90 ? ' Ducati' : '';
        return `${v.toFixed(0)}\u00b0${named} \u2192 fires ${off.toFixed(0)}/${(720 - off).toFixed(0)}`;
      },
      onInput: (v) => {
        /**
         * A V6 stays a V6. At no angle a six is an inline six, so letting the angle reach zero would turn
         * the engine into a different one mid-drag — and hide this slider, which only shows for a V, from
         * under the pointer. Inline six has its own entry in the Cylinders menu.
         */
        const vee = this.config.engine.cylinders === 6 ? Math.max(v, 15) : v;
        this.cb.onEngine({ vAngle: vee });
        this.syncStats();
      },
    });
    this.resyncers.push(() => vRow.render(this.config.engine.vAngle));
    this.vAngleRow = vRow.row;
    vRow.row.title =
      'The included angle between the cylinders. With a shared crankpin it also sets the firing ' +
      'interval: 45 degrees gives the 405/315 of a Harley, 90 the 450/270 of a Ducati L-twin. ' +
      'The more uneven those two intervals, the stronger the half-order thump.';

    const offsetToggle = toggle(multiWrap, 'Override firing offset', spec.firingOffset !== null, (on) => {
      this.cb.onEngine({ firingOffset: on ? firingOffsetDeg(this.config.engine) : null });
      offsetWrap.classList.toggle('hidden', !on);
      this.syncStats();
    });
    offsetToggle.title =
      'Break the shared-crankpin relationship, for layouts it cannot make: 360 for a parallel ' +
      'twin with both pistons together, 270 for a modern crossplane twin, or something small ' +
      'for a big-bang layout.';
    this.offsetToggleEl = offsetToggle;
    const offsetWrap = el('div', 'subgroup', multiWrap);
    this.offsetWrapEl = offsetWrap;
    offsetWrap.classList.toggle('hidden', spec.firingOffset === null);
    slider(offsetWrap, {
      label: 'Firing offset',
      min: 0,
      max: 719,
      step: 1,
      value: spec.firingOffset ?? firingOffsetDeg(spec),
      format: (v) => `${v.toFixed(0)}\u00b0 / ${(720 - v).toFixed(0)}\u00b0`,
      onInput: (v) => {
        this.cb.onEngine({ firingOffset: v });
        this.syncStats();
      },
    });
    showMulti();
    this.syncLayoutOptions();

    // ---- Exhaust ---------------------------------------------------------
    const exhaust = section(root, 'Exhaust system', false);

    this.statsEl = el('div', 'stats', exhaust);

    /**
     * Which duct is being edited.
     *
     * Replaces the old split between an "Exhaust" list and a separate "Collector" list. Once the
     * exhaust is a graph a collector is simply another duct, and giving it its own permanently-visible
     * panel made no more sense than giving cylinder 3's runner one.
     */
    const ductRow = el('div', 'row', exhaust);
    el('label', '', ductRow).textContent = 'Editing';
    this.ductSelect = el('select', '', ductRow) as HTMLSelectElement;
    this.ductSelect.addEventListener('change', () => {
      this.selectedDuctId = this.ductSelect.value;
      this.selected = null;
      this.rebuildPipeList();
      this.cb.onSelectDuct(this.selectedDuctId);
    });

    /**
     * Draw mode.
     *
     * A mode rather than a replacement for the handles: the handles adjust a route that exists — length,
     * bend, diameter — and drawing creates one. Two tools with distinct jobs, and the one that already
     * worked keeps working.
     */
    const drawRow = el('div', 'row', exhaust);
    this.drawBtn = el('button', '', drawRow) as HTMLButtonElement;
    this.drawBtn.textContent = 'Draw a pipe';
    this.drawBtn.title =
      'Start from an exhaust port, a junction, the open end of a pipe (to continue it) or the side of a ' +
      'pipe (to branch off it), then click to add bends. Click a junction, a pipe or a pipe end to join ' +
      'it. Shift draws off the angle grid; Escape abandons; right-click or a double-click finishes in ' +
      'open air.';
    this.drawHint = el('span', 'hint', drawRow);
    this.drawBtn.addEventListener('click', () => {
      this.setDrawMode(!this.drawing);
      this.cb.onDrawMode(this.drawing);
    });

    /**
     * The junction selected in the scene.
     *
     * A junction has nothing to set — its size and shape follow from the pipes meeting at it — so what it
     * offers is what it joins, and a way to draw another pipe out of it.
     */
    this.jointEl = el('div', 'joint hidden', exhaust);

    const linkRow = el('div', 'row', exhaust);
    const linkLabel = el('label', '', linkRow) as HTMLLabelElement;
    const linkBox = el('input', '', linkLabel) as HTMLInputElement;
    linkBox.type = 'checkbox';
    linkBox.checked = this.linkRunners;
    linkLabel.append(' Apply to every cylinder');
    linkLabel.title =
      'Keeps all the runners identical, as a symmetric engine has them. Turn it off to build ' +
      'unequal-length headers.';
    linkBox.addEventListener('change', () => {
      this.linkRunners = linkBox.checked;
      if (this.linkRunners) this.commit();
    });

    this.listEl = el('div', 'segments', exhaust);

    const addRow = el('div', 'row buttons', exhaust);
    for (const kind of ['pipe', 'cone', 'chamber'] as SegmentKind[]) {
      const b = el('button', '', addRow) as HTMLButtonElement;
      b.textContent = `+ ${kind}`;
      b.addEventListener('click', () => this.addSegment(kind));
    }

    // ---- Engine geometry -------------------------------------------------
    const geo = section(root, 'Engine geometry', true);
    slider(geo, {
      label: 'Bore',
      min: 0.05,
      max: 0.12,
      step: 0.001,
      value: spec.bore,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ bore: v }),
    });
    slider(geo, {
      label: 'Stroke',
      min: 0.04,
      max: 0.12,
      step: 0.001,
      value: spec.stroke,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ stroke: v }),
    });
    slider(geo, {
      label: 'Rod length',
      min: 0.09,
      max: 0.24,
      step: 0.001,
      value: spec.rodLength,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ rodLength: v }),
    });
    slider(geo, {
      label: 'Reciprocating mass',
      min: 0.1,
      max: 2,
      step: 0.01,
      value: spec.recipMass,
      unit: 'kg',
      onInput: (v) => this.cb.onEngine({ recipMass: v }),
    }).row.title =
      'Piston, rings, pin and rod small end. Its inertia torque averages to zero over a ' +
      'cycle so it does not change the speed, but it is as large as the gas torque and ' +
      'sets how unevenly the crank turns.';
    slider(geo, {
      label: 'Compression ratio',
      min: 6,
      max: 15,
      step: 0.1,
      value: spec.compressionRatio,
      format: (v) => `${v.toFixed(1)}:1`,
      onInput: (v) => this.cb.onEngine({ compressionRatio: v }),
    });

    // ---- Valves ----------------------------------------------------------
    const valves = section(root, 'Valves and timing', true);
    slider(valves, {
      label: 'Exhaust valve',
      min: 0.018,
      max: 0.05,
      step: 0.0005,
      value: spec.exValveDia,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ exValveDia: v }),
    });
    slider(valves, {
      label: 'Intake valve',
      min: 0.018,
      max: 0.056,
      step: 0.0005,
      value: spec.inValveDia,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ inValveDia: v }),
    });
    slider(valves, {
      label: 'Port length',
      min: 0.01,
      max: 0.2,
      step: 0.001,
      value: spec.portLength,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ portLength: v }),
    }).row.title =
      'The duct from the valve seat to the header flange. It is part of the acoustic ' +
      'system, so the tuned length is measured from the valve, not the flange.';
    slider(valves, {
      label: 'Max lift',
      min: 0.002,
      max: 0.016,
      step: 0.0001,
      value: spec.maxLift,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ maxLift: v }),
    });
    slider(valves, {
      label: 'Exhaust opens',
      min: 90,
      max: 180,
      step: 1,
      value: spec.evo,
      format: (v) => `${Math.round(180 - v)}° BBDC`,
      onInput: (v) => this.cb.onEngine({ evo: v }),
    });
    slider(valves, {
      label: 'Exhaust closes',
      min: 340,
      max: 430,
      step: 1,
      value: spec.evc,
      format: (v) => `${Math.round(v - 360)}° ATDC`,
      onInput: (v) => this.cb.onEngine({ evc: v }),
    });
    slider(valves, {
      label: 'Intake opens',
      min: 300,
      max: 380,
      step: 1,
      value: spec.ivo,
      format: (v) => `${Math.round(360 - v)}° BTDC`,
      onInput: (v) => this.cb.onEngine({ ivo: v }),
    });
    slider(valves, {
      label: 'Intake closes',
      min: 520,
      max: 630,
      step: 1,
      value: spec.ivc,
      format: (v) => `${Math.round(v - 540)}° ABDC`,
      onInput: (v) => this.cb.onEngine({ ivc: v }),
    });

    // ---- Combustion ------------------------------------------------------
    const comb = section(root, 'Combustion', true);
    slider(comb, {
      label: 'Ignition advance',
      min: 0,
      max: 50,
      step: 1,
      value: 720 - spec.ignition,
      unit: '° BTDC',
      // Stored as deg ATDC; 25 deg BTDC is 695.
      onInput: (v) => this.cb.onEngine({ ignition: 720 - v }),
    });
    slider(comb, {
      label: 'Burn duration',
      min: 15,
      max: 110,
      step: 1,
      value: spec.burnDuration,
      unit: '°',
      onInput: (v) => this.cb.onEngine({ burnDuration: v }),
    });
    slider(comb, {
      label: 'Cycle-to-cycle scatter',
      min: 0,
      max: 2.5,
      step: 0.05,
      value: spec.combustionVariability,
      format: (v) => (v === 0 ? 'off (identical cycles)' : `${v.toFixed(2)}×`),
      onInput: (v) => this.cb.onEngine({ combustionVariability: v }),
    }).row.title =
      'How much the flame kernel varies from one cycle to the next. 1.0 is realistic: ' +
      'about 2% variation in indicated work under load, over 10% near idle. Set it to ' +
      'zero and every cycle becomes identical — which is what makes simulated engines ' +
      'sound like a looped sample.';
    slider(comb, {
      label: 'Pipe wall thickness',
      min: 0.0004,
      max: 0.005,
      step: 0.0001,
      value: spec.pipeWallThickness,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ pipeWallThickness: v }),
    }).row.title =
      "The wall's thermal mass, so how long the system takes to come up to temperature — " +
      'tens of seconds for typical 1.2 mm tubing. Gas temperature sets the speed of sound, ' +
      'so the note genuinely shifts as the pipe warms.';
    slider(comb, {
      label: 'Air speed past pipe',
      min: 0,
      max: 45,
      step: 0.5,
      value: spec.airSpeed,
      format: (v) =>
        v < 0.5 ? 'still air' : `${v.toFixed(0)} m/s (${(v * 3.6).toFixed(0)} km/h)`,
      onInput: (v) => this.cb.onEngine({ airSpeed: v }),
    }).row.title =
      'Cools the pipe wall, which cools the gas, which slows the wave speed and drops the ' +
      'tuning. Radiation off oxidised steel matters as much as convection here.';
    slider(comb, {
      label: 'Solver resolution',
      min: 0.035,
      max: 0.12,
      step: 0.001,
      value: spec.pipeCellSize,
      format: (v) =>
        `${(v * 1000).toFixed(0)} mm cells · ~${(550 / (10 * v) / 1000).toFixed(1)} kHz`,
      onInput: (v) => this.cb.onEngine({ pipeCellSize: v }),
    }).row.title =
      'Cell length for the exhaust gas-dynamics solver. Smaller cells resolve higher ' +
      'frequencies and cost more. The solver takes one step per audio sample, so cells ' +
      'cannot be shorter than about 35 mm at 48 kHz (51 mm at 32 kHz, 69 mm at 24 kHz); a big engine may be given ' +
      'coarser cells than asked for, to keep it in real time.';
    slider(comb, {
      label: 'Port gas temp',
      min: 350,
      max: 1250,
      step: 5,
      value: spec.portGasTemp,
      format: (v) => `${Math.round(v)} K · c=${Math.round(speedOfSound(v))} m/s`,
      onInput: (v) => {
        this.cb.onEngine({ portGasTemp: v });
        // The tuning readout's wave speed comes from this.
        this.syncStats();
      },
    });

    // ---- Listener --------------------------------------------------------
    const mix = section(root, 'Listener', true);
    slider(mix, {
      label: 'Mic distance',
      min: 0.3,
      max: 12,
      step: 0.1,
      value: spec.micDistance,
      unit: 'm',
      format: (v) => `${v.toFixed(1)} m`,
      onInput: (v) => this.cb.onEngine({ micDistance: v }),
    });
    slider(mix, {
      label: 'Ear height',
      min: 0.05,
      max: 3,
      step: 0.05,
      value: spec.micHeight,
      unit: 'm',
      format: (v) => `${v.toFixed(2)} m`,
      onInput: (v) => this.cb.onEngine({ micHeight: v }),
    });
    slider(mix, {
      label: 'Exhaust height',
      min: 0.05,
      max: 2,
      step: 0.05,
      value: spec.exhaustHeight,
      unit: 'm',
      format: (v) => `${v.toFixed(2)} m`,
      onInput: (v) => this.cb.onEngine({ exhaustHeight: v }),
    });
    const spacingRow = slider(mix, {
      label: 'Mouth spacing',
      min: 0,
      max: 2.5,
      step: 0.05,
      value: spec.mouthSpacing,
      unit: 'm',
      format: (v) => (v < 0.03 ? 'coincident' : `${v.toFixed(2)} m`),
      onInput: (v) => this.cb.onEngine({ mouthSpacing: v }),
    });
    this.resyncers.push(() => spacingRow.render(this.config.engine.mouthSpacing));
    spacingRow.row.title =
      'How far apart the tailpipes are. Only does anything with more than one of them, and then ' +
      'it matters a great deal: set it to zero and the mouths sum at a single point, where the ' +
      'two banks of a flatplane V8 fire in antiphase and annihilate their own firing order — ' +
      '44 dB of it — jumping the engine an octave. Real pipes are a metre or so apart.';

    const spreadRow = slider(mix, {
      label: 'Cylinder spread',
      min: 0,
      max: 1,
      step: 0.01,
      value: spec.cylinderSpread,
      format: (v) => (v < 0.01 ? 'perfectly matched' : `${(v * 4).toFixed(1)}%`),
      onInput: (v) => this.cb.onEngine({ cylinderSpread: v }),
    });
    this.resyncers.push(() => spreadRow.render(this.config.engine.cylinderSpread));
    spreadRow.row.title =
      'How unequally the cylinders breathe, as a spread in runner pressure. No two cylinders of ' +
      'a real engine are matched, and that is what stops the firing orders cancelling perfectly. ' +
      'At zero an inline four is a pure tone on one frequency with no rumble under it; at 4% the ' +
      'low orders sit 35 dB down, where real engines measure 20 to 35.';

    slider(mix, {
      label: 'Ground reflection',
      min: 0,
      max: 1,
      step: 0.01,
      value: spec.groundReflection,
      format: (v) =>
        v < 0.15 ? 'anechoic' : v < 0.5 ? `${v.toFixed(2)} grass` : `${v.toFixed(2)} asphalt`,
      onInput: (v) => this.cb.onEngine({ groundReflection: v }),
    }).row.title =
      'The ground sends a second, slightly later copy of everything to your ear, and the ' +
      'two comb-filter each other. Set it to zero to hear the engine in free space — ' +
      'which is how simulated engines usually sound, and why they sound wrong.';
    slider(mix, {
      label: 'Output gain',
      min: 0,
      max: 1.5,
      step: 0.01,
      value: spec.outputGain,
      format: (v) => v.toFixed(2),
      onInput: (v) => this.cb.onEngine({ outputGain: v }),
    });
    slider(mix, {
      label: 'Mechanical noise',
      min: 0,
      max: 1,
      step: 0.01,
      value: spec.mechNoise,
      format: (v) => `${Math.round(v * 100)}%`,
      onInput: (v) => this.cb.onEngine({ mechNoise: v }),
    });
    slider(mix, {
      label: 'Valve throat noise',
      min: 0,
      max: 1,
      step: 0.01,
      value: spec.throatNoise,
      format: (v) => `${Math.round(v * 100)}%`,
      onInput: (v) => this.cb.onEngine({ throatNoise: v }),
    });

    // ---- View ------------------------------------------------------------
    const viewSec = section(root, 'View', true);
    toggle(viewSec, 'Cutaway section', this.view.cutaway, (on) => {
      this.view.cutaway = on;
      this.cb.onView(this.view);
    });
    toggle(viewSec, 'Pressure colouring', this.view.pressure, (on) => {
      this.view.pressure = on;
      this.cb.onView(this.view);
    });
    toggle(viewSec, 'Edit handles', this.view.handles, (on) => {
      this.view.handles = on;
      this.cb.onView(this.view);
    });
    const fit = el('button', '', viewSec) as HTMLButtonElement;
    fit.textContent = 'Frame the exhaust';
    fit.addEventListener('click', () => this.cb.onResetView());

    this.rebuildPipeList();
  }

  /**
   * Re-render every control from the config.
   *
   * Needed after an engine preset, which changes layout, V angle and firing all at once —
   * individual slider callbacks would each fire a rebuild and the selects would go stale.
   */
  rebuildAll(): void {
    this.cylSel.value = engineTypeOf(this.config.engine);
    this.crankSel.value =
      this.config.engine.crankType === 'flatplane' ? 'flatplane' : 'crossplane';
    this.twinWrap.classList.toggle('hidden', this.config.engine.cylinders < 2);
    this.syncLayoutOptions();
    for (const r of this.resyncers) r();
    this.rebuildPipeList();
  }

  /**
   * Offer only the plumbing and the controls that mean something for this cylinder count.
   *
   * A single has nothing to merge with; an inline four has one bank, so per-bank and merged are
   * the same pipework and its V angle is meaningless; only a V8 has a crank choice; and the
   * firing-offset override is a twin's shared-crankpin escape hatch, not a general control.
   */
  private syncLayoutOptions(): void {
    const eng = this.config.engine;
    const n = eng.cylinders;
    const plan = firingPlan(eng);
    const current = exhaustLayoutOf(eng);

    const choices: Array<[ExhaustLayout, string]> = [];
    if (n === 1) {
      choices.push(['open', 'Single pipe']);
    } else {
      choices.push(['open', `${n} separate pipes`]);
      if (plan.bankCount > 1) choices.push(['perBank', `${n / 2}-into-1 per bank`]);
      choices.push(['merged', `${n}-into-1 (one collector)`]);
    }

    this.layoutSel.replaceChildren();
    for (const [value, label] of choices) this.layoutSel.appendChild(option(value, label));
    this.layoutSel.value = choices.some(([v]) => v === current) ? current : choices[0]![0];
    this.layoutSel.parentElement?.classList.toggle('hidden', n === 1);

    this.crankRow.classList.toggle('hidden', n !== 8);
    // A boxer's banks are flat by definition; at any other angle it would be a V on a boxer's crank.
    this.vAngleRow.classList.toggle('hidden', plan.bankCount < 2 || isBoxer(eng));
    this.offsetToggleEl.classList.toggle('hidden', n !== 2);
    this.offsetWrapEl.classList.toggle('hidden', n !== 2 || eng.firingOffset === null);
  }

  get viewOptions(): ViewOptions {
    return this.view;
  }

  setRunning(running: boolean): void {
    this.startBtn.textContent = running ? 'Stop engine' : 'Start engine';
    this.startBtn.classList.toggle('running', running);
  }

  // -------------------------------------------------------------------------
  // Segment list
  // -------------------------------------------------------------------------

  private addSegment(kind: SegmentKind): void {
    const pipe = this.currentSegments();
    const last = pipe[pipe.length - 1];
    const dIn = last ? segmentDiameter(last, 1) : 0.042;
    pipe.push(
      makeSegment({
        kind,
        length: kind === 'chamber' ? 0.3 : 0.25,
        dIn,
        dOut: kind === 'chamber' ? dIn * 3 : kind === 'cone' ? dIn * 1.8 : dIn,
      }),
    );
    this.selected = pipe.length - 1;
    this.commit();
    this.rebuildPipeList();
    this.cb.onSelect(this.selected);
  }

  /**
   * Publish an edit, copying it to the other runners when they are linked.
   *
   * Mirroring is a copy of the *whole* segment list rather than a replay of the edit, which is both
   * simpler and more robust: whatever the user just did — a length, a diameter, a delete, a reorder —
   * the other runners end up identical without every mutation needing its own mirroring logic.
   *
   * Only runners are linked. A collector is a different part of the exhaust and there is generally one
   * of it, so linking it to anything would be meaningless.
   */
  private commit(): void {
    const graph = this.config.graph;
    const duct = this.currentDuct();
    if (this.linkRunners && graph && duct) copyToSiblingRunners(graph, duct);
    this.cb.onPipe();
  }

  /** The duct the list is editing, or the first one if the selection has gone stale. */
  private currentDuct(): ExhaustDuct | null {
    const graph = this.config.graph;
    if (!graph || graph.ducts.length === 0) return null;
    return graph.ducts.find((d) => d.id === this.selectedDuctId) ?? graph.ducts[0]!;
  }

  /** Segments the list edits. Falls back to `config.pipe` when there is no graph yet, so callers need no guard. */
  private currentSegments(): PipeSegment[] {
    return this.currentDuct()?.segments ?? this.config.pipe;
  }

  /** Full structural re-render. Only for add / delete / reorder / preset. */
  rebuildPipeList(): void {
    const graph = this.config.graph ?? null;
    if (graph !== this.selectionGraph) {
      this.selectionGraph = graph;
      this.selectedDuctId = (graph ? defaultDuctId(graph) : undefined) ?? 'runner0';
      this.selected = null;
    }
    const duct = this.currentDuct();
    if (duct) this.selectedDuctId = duct.id;

    // Repopulate the picker. Rebuilt wholesale because adding a duct renumbers the labels.
    this.ductSelect.replaceChildren();
    if (graph) {
      for (const d of graph.ducts) {
        const opt = el('option', '', this.ductSelect) as HTMLOptionElement;
        opt.value = d.id;
        opt.textContent = ductLabel(graph, d);
      }
      this.ductSelect.value = this.selectedDuctId;
    }

    const segments = this.currentSegments();
    this.listEl.replaceChildren();
    this.rows.length = 0;
    segments.forEach((seg, i) =>
      this.rows.push(this.buildRow(this.listEl, segments, seg, i, true)),
    );

    this.applySelection();
    this.syncStats();
  }

  /**
   * Build one segment row. Parameterised by which array it edits, and by whether picking
   * the row selects the segment; every caller now passes the selected duct's segments and `true`.
   */
  private buildRow(
    container: HTMLElement,
    list: PipeSegment[],
    seg: PipeSegment,
    index: number,
    selectable: boolean,
  ): SegmentRow {
    const wrap = el('div', 'segment', container);
    if (selectable) {
      wrap.addEventListener('pointerdown', () => {
        this.selected = index;
        this.applySelection();
        this.cb.onSelect(index);
      });
    }

    const head = el('div', 'segment-head', wrap);
    el('span', 'segment-index', head).textContent = String(index + 1);

    const kind = el('select', 'segment-kind', head) as HTMLSelectElement;
    for (const k of ['pipe', 'cone', 'chamber'] as SegmentKind[]) {
      kind.appendChild(option(k, k));
    }
    kind.value = seg.kind;
    kind.addEventListener('change', () => {
      seg.kind = kind.value as SegmentKind;
      if (seg.kind === 'pipe') seg.dOut = seg.dIn;
      else if (seg.dOut === seg.dIn) seg.dOut = seg.dIn * (seg.kind === 'chamber' ? 3 : 1.8);
      this.commit();
      this.rebuildPipeList();
    });

    const tools = el('div', 'segment-tools', head);
    const mkTool = (label: string, title: string, fn: () => void) => {
      const b = el('button', 'tool', tools) as HTMLButtonElement;
      b.textContent = label;
      b.title = title;
      b.addEventListener('click', (e) => {
        e.stopPropagation();
        fn();
      });
    };
    mkTool('↑', 'Move earlier in the exhaust', () => this.move(list, index, -1));
    mkTool('↓', 'Move later in the exhaust', () => this.move(list, index, 1));
    mkTool('⧉', 'Duplicate', () => this.duplicate(list, index));
    mkTool('×', 'Delete', () => this.remove(list, index));

    const grid = el('div', 'segment-grid', wrap);
    const length = numberField(grid, 'Length', seg.length * MM, 10, 2000, 5, 'mm', (v) => {
      seg.length = v / MM;
      this.commit();
      this.syncStats();
    });
    const dIn = numberField(grid, 'Inlet ⌀', seg.dIn * MM, 6, 250, 1, 'mm', (v) => {
      seg.dIn = v / MM;
      if (seg.kind === 'pipe') seg.dOut = seg.dIn;
      this.propagate(list, index);
      this.commit();
      this.syncPipe();
    });
    const dOutWrap = el('div', 'field', grid);
    const dOut = numberInto(
      dOutWrap,
      seg.kind === 'chamber' ? 'Body ⌀' : 'Outlet ⌀',
      seg.dOut * MM,
      6,
      400,
      1,
      'mm',
      (v) => {
        seg.dOut = v / MM;
        this.propagate(list, index);
        this.commit();
        this.syncPipe();
      },
    );
    dOutWrap.classList.toggle('hidden', seg.kind === 'pipe');

    const bends = el('div', 'segment-grid', wrap);
    numberField(bends, 'Yaw', deg(seg.yaw), -120, 120, 1, '°', (v) => {
      seg.yaw = rad(v);
      this.commit();
    });
    numberField(bends, 'Pitch', deg(seg.pitch), -120, 120, 1, '°', (v) => {
      seg.pitch = rad(v);
      this.commit();
    });

    return { el: wrap, kind, length, dIn, dOut, dOutWrap };
  }

  /** Keep the duct continuous after an inlet/outlet edit. */
  private propagate(list: PipeSegment[], index: number): void {
    const seg = list[index];
    const next = list[index + 1];
    if (!seg || !next) return;
    next.dIn = segmentDiameter(seg, 1);
    if (next.kind === 'pipe') next.dOut = next.dIn;
  }

  private move(pipe: PipeSegment[], index: number, delta: number): void {
    const to = index + delta;
    if (to < 0 || to >= pipe.length) return;
    const [seg] = pipe.splice(index, 1);
    pipe.splice(to, 0, seg!);
    this.selected = to;
    this.commit();
    this.rebuildPipeList();
    this.cb.onSelect(to);
  }

  private duplicate(list: PipeSegment[], index: number): void {
    const seg = list[index];
    if (!seg) return;
    list.splice(index + 1, 0, makeSegment({ ...seg, id: undefined }));
    this.selected = index + 1;
    this.commit();
    this.rebuildPipeList();
    this.cb.onSelect(this.selected);
  }

  private remove(list: PipeSegment[], index: number): void {
    list.splice(index, 1);
    this.selected = null;
    /**
     * A pipe with nothing left in it goes, unless it is a cylinder's runner.
     *
     * An empty duct is drawn as nothing but still solved as a short stub, so leaving it would keep a pipe
     * the user can no longer see or select. A runner is kept, empty, because a cylinder must have one.
     */
    const graph = this.config.graph;
    const duct = this.currentDuct();
    if (list.length === 0 && graph && duct && duct.from.kind !== 'valve') {
      // The owner removes it, because tidying the junctions needs the layout to keep pipes where they were.
      this.cb.onRemoveDuct(duct.id);
    } else {
      this.commit();
    }
    this.rebuildPipeList();
    this.cb.onSelect(null);
  }

  /** Delete the selected segment, as its × button would. Returns whether one was selected. */
  deleteSelected(): boolean {
    const list = this.currentSegments();
    if (this.selected === null || !list[this.selected]) return false;
    this.remove(list, this.selected);
    return true;
  }

  /** Refresh input values in place. Safe to call every frame of a 3D drag. */
  syncPipe(): void {
    const segments = this.currentSegments();
    const graph = this.config.graph;
    if (
      this.rows.length !== segments.length ||
      (graph && this.ductSelect.options.length !== graph.ducts.length)
    ) {
      this.rebuildPipeList();
      return;
    }
    this.syncRows(this.rows, segments);
    this.syncStats();
  }

  private syncRows(rows: SegmentRow[], list: PipeSegment[]): void {
    list.forEach((seg, i) => {
      const row = rows[i];
      if (!row) return;
      // Never fight the field the user is currently typing in.
      if (document.activeElement !== row.length) row.length.value = round(seg.length * MM, 1);
      if (document.activeElement !== row.dIn) row.dIn.value = round(seg.dIn * MM, 1);
      if (document.activeElement !== row.dOut) row.dOut.value = round(seg.dOut * MM, 1);
      row.kind.value = seg.kind;
      row.dOutWrap.classList.toggle('hidden', seg.kind === 'pipe');
    });
  }

  /** Tell the user whether a route is in progress, since the 3D preview is easy to miss. */
  setDrawingState(active: boolean): void {
    if (!this.drawing) return;
    this.drawHint.textContent = active
      ? 'Click to add a bend, or a junction or pipe to join it'
      : START_HINT;
  }

  /** Show draw mode as on or off, without telling anyone: for when the owner switched it. */
  private setDrawMode(on: boolean): void {
    this.drawing = on;
    this.drawBtn.classList.toggle('active', on);
    this.drawBtn.textContent = on ? 'Stop drawing' : 'Draw a pipe';
    this.drawHint.textContent = on ? START_HINT : '';
  }

  /** Switch the list to a duct picked in the scene. */
  showDuct(id: string): void {
    this.selectedDuctId = id;
    this.selected = null;
    this.rebuildPipeList();
  }

  /** Describe the junction selected in the scene, or hide the description when there is none. */
  showJoint(node: string | null): void {
    const graph = this.config.graph;
    this.jointEl.replaceChildren();
    this.jointEl.classList.toggle('hidden', !node || !graph);
    if (!node || !graph) return;

    const ends = endsAt(graph, node);
    el('div', 'joint-title', this.jointEl).textContent = `Junction ${nodeOrder(graph).indexOf(node) + 1}`;
    const list = (label: string, ducts: ExhaustDuct[]) => {
      if (ducts.length === 0) return;
      const row = el('div', 'joint-row', this.jointEl);
      el('span', 'joint-label', row).textContent = label;
      el('span', '', row).textContent = ducts.map((d) => ductLabel(graph, d)).join(', ');
    };
    list('In', ends.filter((e) => e.end === 'outlet').map((e) => e.duct));
    list('Out', ends.filter((e) => e.end === 'inlet').map((e) => e.duct));

    const btn = el('button', '', el('div', 'row', this.jointEl)) as HTMLButtonElement;
    btn.textContent = 'Draw a pipe from here';
    btn.addEventListener('click', () => {
      this.setDrawMode(true);
      this.cb.onDrawFromJoint(node);
      this.setDrawingState(true);
    });
  }

  setSelected(index: number | null): void {
    this.selected = index;
    this.applySelection();
  }

  private applySelection(): void {
    this.rows.forEach((r, i) => r.el.classList.toggle('selected', i === this.selected));
  }

  private syncStats(): void {
    const eng = this.config.engine;
    const graph = this.config.graph;
    /**
     * The tuned length a cylinder sees: everything between its valve and open air.
     *
     * Walked along the graph rather than added up as "primary plus collector", which only described
     * the layouts that had exactly those two parts. A tri-Y has three.
     */
    const path = graph ? pathToAir(graph, 0) : [];
    const len = path.length > 0
      ? path.reduce((a, d) => a + totalPipeLength(d.segments), 0)
      : totalPipeLength(this.config.pipe);
    // Quarter-wave resonance using the mean wave speed along the duct.
    const c = speedOfSound((this.config.engine.portGasTemp + 400) / 2);
    const f1 = len > 0 ? c / (4 * len) : 0;

    // The rpm the pipe is tuned for is *not* where the firing frequency equals f1 —
    // that would be an absurd 18,000 rpm for a normal pipe. What matters is when the
    // negative wave reflected from the open end gets back to the port during valve
    // overlap. The round trip takes 2L/c seconds, which at N rpm is 12*L*N/c crank
    // degrees; setting that to the ~180 deg from exhaust-valve-opening to overlap
    // gives N = 15c/L, i.e. 60*f1.
    const tunedRpm = f1 * 60;

    const segs = path.length > 0
      ? path.reduce((a, d) => a + d.segments.length, 0)
      : this.config.pipe.length;
    const layoutNote = eng.cylinders === 1 ? '' : ` · ${firingNote(eng)}`;
    this.statsEl.textContent =
      `${segs} segment${segs === 1 ? '' : 's'} · ` +
      `${(len * MM).toFixed(0)} mm · ` +
      `1st peak ≈ ${f1.toFixed(0)} Hz · tuned near ${tunedRpm.toFixed(0)} rpm${layoutNote}`;
  }

  // -------------------------------------------------------------------------
  // Readouts
  // -------------------------------------------------------------------------


  updateReadouts(s: EngineSnapshot): void {
    this.rpmEl.textContent = `${Math.round(s.rpm)} rpm${s.limiter ? ' · limiter' : ''}`;
    const stroke = strokeName(s.crankAngle);
    this.readoutEl.textContent =
      `${stroke} · ${s.crankAngle.toFixed(0)}° · ` +
      `${(s.cylPressure / 1e5).toFixed(1)} bar · ${Math.round(s.cylTemp)} K · ` +
      `ex ${(s.exLift * MM).toFixed(1)} mm · in ${(s.inLift * MM).toFixed(1)} mm · ` +
      `pipe wall ${Math.round(s.wallTemp)} K`;
    // Peak is a linear sample magnitude; show it on a dB scale so quiet mufflers
    // still move the meter.
    const db = 20 * Math.log10(Math.max(s.peak, 1e-5));
    this.meterFill.style.width = `${Math.max(0, Math.min(100, ((db + 60) / 60) * 100))}%`;
    this.meterFill.classList.toggle('hot', s.peak > 0.95);
  }
}

// ---------------------------------------------------------------------------
// Small DOM helpers
// ---------------------------------------------------------------------------

function strokeName(deg: number): string {
  if (deg < 180) return 'Power';
  if (deg < 360) return 'Exhaust';
  if (deg < 540) return 'Intake';
  return 'Compression';
}

function el<K extends keyof HTMLElementTagNameMap>(
  tag: K,
  cls: string,
  parent?: HTMLElement,
): HTMLElementTagNameMap[K] {
  const node = document.createElement(tag);
  if (cls) node.className = cls;
  parent?.appendChild(node);
  return node;
}

function option(value: string, label: string): HTMLOptionElement {
  const o = document.createElement('option');
  o.value = value;
  o.textContent = label;
  return o;
}

function section(root: HTMLElement, title: string, collapsed: boolean): HTMLElement {
  const details = document.createElement('details');
  details.className = 'section';
  details.open = !collapsed;
  const summary = document.createElement('summary');
  summary.textContent = title;
  details.appendChild(summary);
  const body = el('div', 'section-body');
  details.appendChild(body);
  root.appendChild(details);
  return body;
}

/** Re-reads one control from the config; collected so a preset can refresh them all. */
type Resync = () => void;

interface SliderOpts {
  label: string;
  min: number;
  max: number;
  step: number;
  value: number;
  /** Multiplier for display only, e.g. 1000 to show metres as millimetres. */
  scale?: number;
  unit?: string;
  format?: (v: number) => string;
  onInput: (v: number) => void;
}

function slider(
  parent: HTMLElement,
  o: SliderOpts,
): { row: HTMLElement; input: HTMLInputElement; render: (v: number) => void } {
  const row = el('div', 'row slider-row', parent);
  const head = el('div', 'slider-head', row);
  el('label', '', head).textContent = o.label;
  const valueEl = el('span', 'value', head);

  const input = el('input', '', row) as HTMLInputElement;
  input.type = 'range';
  input.min = String(o.min);
  input.max = String(o.max);
  input.step = String(o.step);
  input.value = String(o.value);

  const scale = o.scale ?? 1;
  const render = (v: number) => {
    valueEl.textContent = o.format
      ? o.format(v)
      : `${trim(v * scale)}${o.unit ? ` ${o.unit}` : ''}`;
  };
  render(o.value);

  input.addEventListener('input', () => {
    const v = Number(input.value);
    render(v);
    o.onInput(v);
  });

  return {
    row,
    input,
    render: (v: number) => {
      input.value = String(v);
      render(v);
    },
  };
}

function numberField(
  parent: HTMLElement,
  label: string,
  value: number,
  min: number,
  max: number,
  step: number,
  unit: string,
  onChange: (v: number) => void,
): HTMLInputElement {
  const field = el('div', 'field', parent);
  return numberInto(field, label, value, min, max, step, unit, onChange);
}

function numberInto(
  field: HTMLElement,
  label: string,
  value: number,
  min: number,
  max: number,
  step: number,
  unit: string,
  onChange: (v: number) => void,
): HTMLInputElement {
  el('label', '', field).textContent = label;
  const wrap = el('div', 'number-wrap', field);
  const input = el('input', '', wrap) as HTMLInputElement;
  input.type = 'number';
  input.min = String(min);
  input.max = String(max);
  input.step = String(step);
  input.value = round(value, 1);
  el('span', 'unit', wrap).textContent = unit;

  /**
   * Applied on `change` — Enter, leaving the field, or the spinner — not on every keystroke.
   *
   * Committing as you type meant committing every prefix of what you were typing: on the way to a 50 mm
   * inlet the field held "5", which the 6 mm minimum clamped and wrote straight back as "6". And a
   * prefix that is in range is still wrong: typing a 1200 mm length would briefly make it 12 mm, short
   * enough to pull the pipe off its junction.
   */
  const commit = () => {
    const v = Number(input.value);
    if (input.value.trim() === '' || !Number.isFinite(v)) return;
    const clamped = Math.max(min, Math.min(max, v));
    if (clamped !== v) input.value = round(clamped, 1);
    onChange(clamped);
  };
  input.addEventListener('change', commit);
  input.addEventListener('pointerdown', (e) => e.stopPropagation());
  return input;
}

function toggle(
  parent: HTMLElement,
  label: string,
  value: boolean,
  onChange: (v: boolean) => void,
): HTMLElement {
  const row = el('label', 'row toggle', parent);
  const input = el('input', '', row) as HTMLInputElement;
  input.type = 'checkbox';
  input.checked = value;
  el('span', '', row).textContent = label;
  input.addEventListener('change', () => onChange(input.checked));
  return row;
}

function trim(v: number): string {
  if (Math.abs(v) >= 100) return v.toFixed(0);
  if (Math.abs(v) >= 10) return v.toFixed(1);
  return v.toFixed(2);
}

function round(v: number, dp: number): string {
  return String(Number(v.toFixed(dp)));
}

const deg = (r: number): number => (r * 180) / Math.PI;
const rad = (d: number): number => (d * Math.PI) / 180;

/**
 * How this engine fires, in words.
 *
 * For a twin the two intervals say it all. For anything with two banks and a collector each,
 * what matters is the pattern *within a bank* — that is what its collector hears, and what
 * separates a crossplane V8 from a flatplane one.
 */
function firingNote(eng: EngineSpec): string {
  const plan = firingPlan(eng);
  const layout = exhaustLayoutOf(eng);
  if (eng.cylinders === 2) {
    const fire = firingOffsetDeg(eng);
    return `fires ${fire.toFixed(0)}/${(720 - fire).toFixed(0)}`;
  }
  const every = 720 / eng.cylinders;
  const base = `fires every ${every.toFixed(0)}\u00b0`;
  if (plan.bankCount < 2 || layout !== 'perBank') return base;
  const per = bankFiringIntervals(eng, 0).join('-');
  const even = new Set(bankFiringIntervals(eng, 0)).size === 1;
  return `${base} · each bank ${per}${even ? ' (even)' : ' (uneven)'}`;
}
