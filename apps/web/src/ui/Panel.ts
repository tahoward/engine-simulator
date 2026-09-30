/**
 * Hand-rolled control panel.
 *
 * The segment menu edits the selected duct's own `PipeSegment[]` in the exhaust graph, the
 * same array the 3D handles edit when they are on that duct, so this never owns a copy of
 * the geometry — it renders whatever the array currently says
 * and writes edits straight back. `syncPipe` refreshes the input values in place
 * (called while the user drags handles in 3D) and `rebuildPipeList` is reserved for
 * structural changes, so typing in a field is never interrupted by a re-render.
 */

import {
  carryBore,
  childDucts,
  copyToSiblingRunners,
  defaultDuctId,
  ductLabel,
  endsAt,
  nodeOrder,
  pathToAir,
  turboAt,
  type ExhaustDuct,
  type ExhaustGraph,
  type TurboSettings,
} from '../model/exhaustGraph.js';
import { lockedFrom, isTurbocharged, turboSettingsOf, turbosSynced } from '../model/turbo.js';
import { bendShape } from '../model/geometry.js';
import {
  ENGINE_PRESETS,
  bankFiringIntervals,
  exhaustLayoutOf,
  firingOffsetDeg,
  firingPlan,
  fitDyno,
  fitLaunch,
  fullLoadTorque,
  DRIVEN_LOAD,
  DRIVER_MASS,
  MANUAL_SHIFT_TIME,
  TYRE_GRIP,
  DYNO_FROM_RPM,
  DYNO_SWEEP_RATE,
  LAUNCH_RATIOS,
  LAUNCH_RPM_MARGIN,
  MAX_GEARS,
  MIN_GEARS,
  intakeRunnerOf,
  runnerTunedRpm,
  isBoxer,
  presetEngine,
  type BlowOff,
  type Car,
  type LaunchConfig,
  type EngineConfig,
  type EngineSnapshot,
  type EngineSpec,
  type ChamberSection,
  type PipeSegment,
  type SegmentKind,
  BLOW_OFFS,
  CHAMBER_SECTIONS,
  makeSegment,
  physicalBankCount,
  segmentDiameter,
  speedOfSound,
  totalPipeLength,
} from '../model/spec.js';
import { plenumVolumeOf, throttleDiaOf } from '../model/intakeSizing.js';
import { autoLaunchSettings, type LaunchSettings } from '../model/launchSettings.js';
import { SECTION_ICONS, TOOL_ICONS, toolButton } from './toolbar.js';

export interface PanelCallbacks {
  onEngine: (partial: Partial<EngineSpec>) => void;
  /** The selected duct's segments were mutated in place, and copied to the other runners if linked. */
  onPipe: () => void;
  onSelect: (index: number | null) => void;
  /**
   * `config.pipe` and `config.collector` were replaced; rebuild the graph from them, with `turbos` turbos
   * placed where the layout puts them.
   */
  onReseed: (turbos: number, graph?: ExhaustGraph) => void;
  /** Placing turbos in the view was switched on or off. */
  onPlaceMode: (on: boolean) => void;
  /** Placing loose pipes in the view was switched on or off. */
  onPlacePipeMode: (on: boolean) => void;
  /** Take this turbo out of the exhaust. */
  onRemoveTurbo: (id: string) => void;
  /** Give this turbo its own settings, or with `null` put it back on the engine's. */
  onTurboSettings: (id: string, settings: TurboSettings | null) => void;
  /** Keep the turbos in sync, all on the engine's settings, or let each have its own. */
  onTurbosSynced: (synced: boolean) => void;
  /** Draw mode was switched on or off. */
  onDrawMode: (on: boolean) => void;
  /** Start drawing a new pipe out of this junction. */
  onDrawFromJoint: (node: string) => void;
  /** Add a segment of `kind` at the opening of this junction. */
  onAddAtJunction: (node: string, kind: SegmentKind) => void;
  /** A pipe had its last segment deleted and should go, tidying the junctions around it. */
  onRemoveDuct: (id: string) => void;
  /** Take this pipe's far end off what it joins, with the bend it was fitted in. */
  onDetachDuct: (id: string) => void;
  /** Save the engine as it stands to a file. */
  onExportEngine: () => void;
  /** Replace the engine with one from an exported file's text. */
  onImportEngine: (text: string) => void;
  /** The bend tool was switched on or off. */
  onBendTool: (on: boolean) => void;
  /** Switch the bend tool on, with its rings on segment `index` of this pipe. */
  onBendSegment: (ductId: string, index: number) => void;
  /** The equal-length header tool was switched on or off. */
  onHeaderTool: (on: boolean) => void;
  /** The header's primaries are to be `length` m each. */
  onHeaderLength: (length: number) => void;
  /** Whether the header is mirrored onto the other bank. */
  onHeaderMirror: (on: boolean) => void;
  /** Build the header the ghost shows. */
  onApplyHeader: () => void;
  /**
   * Reshape bend `index` of this pipe to turn through `angle` radians round `radius` m, the straights either
   * side of it taking up the difference so the pipe keeps its length.
   */
  onReshapeBend: (ductId: string, index: number, angle: number, radius: number, changed: 'angle' | 'radius') => void;
  /** Slide bend `index` along this pipe so the straight before it is `before` m long. */
  onSlideBend: (ductId: string, index: number, before: number) => void;
  /** Make this pipe, joined at its far end, `length` m long, with a swing on the way if it needs one. */
  onMatchLength: (ductId: string, length: number) => void;
  /** Delete segment `index` from the middle of this pipe, leaving the segments after it loose. */
  onSplitDuct: (id: string, index: number) => void;
  onToggleAudio: () => void;
  /** The user picked a different audio sample rate, Hz. */
  onSampleRate: (hz: number) => void;
  onView: (view: ViewOptions) => void;
  onResetView: () => void;
  /** Start a launch through `config`, or with `null` stop the one in progress. */
  onLaunch: (config: LaunchConfig | null) => void;
  /** A launch setting, or the car, changed. */
  onLaunchSettings: () => void;
}

export interface ViewOptions {
  pressure: boolean;
  handles: boolean;
  /** Share of real time the simulation runs at: 1 is real time, less is slow motion. */
  speed: number;
}

interface SegmentRow {
  el: HTMLElement;
  kind: HTMLSelectElement;
  length: HTMLInputElement;
  dIn: HTMLInputElement;
  dOut: HTMLInputElement;
  dOutWrap: HTMLElement;
  /** A chamber's shape controls; absent for other segments. */
  height?: HTMLInputElement;
  offsetIn?: HTMLInputElement;
  offsetOut?: HTMLInputElement;
}

const SECTION_LABELS: Record<ChamberSection, string> = { round: 'round', oval: 'oval', rect: 'rectangular' };

/** What the body-size field is called: a diameter for a round can, a width otherwise. */
function bodyLabel(seg: PipeSegment): string {
  if (seg.kind !== 'chamber') return 'Outlet ⌀';
  return (seg.section ?? 'round') === 'round' ? 'Body ⌀' : 'Width';
}

const MM = 1000;

/**
 * What the Speed menu offers. A pressure wave crosses a metre of pipe in about 2 ms, so it is a blur at
 * real time, a few frames at a hundredth, and a couple of seconds at a thousandth.
 */
const SPEEDS: Array<[number, string]> = [
  [1, 'Real time'],
  [0.1, '1/10 · slow motion'],
  [0.01, '1/100 · pressure waves'],
  [0.001, '1/1000 · single pulses'],
];

/**
 * What the Sample rate menu offers, with the exhaust's band limit at each: the solver's finest cell is
 * `1400 / (fs · 0.85)` and it resolves up to about `c / (5 dx)` with `c` = 400 m/s at the mouth.
 */
export const SAMPLE_RATES: Array<[number, string]> = [
  [48000, '48 kHz · full detail'],
  [32000, '32 kHz · ~63% CPU, exhaust to ~1.5 kHz'],
  [24000, '24 kHz · ~47% CPU, exhaust to ~1.2 kHz'],
];

/** What the Cylinders menu offers: a count, and for a twin or a six whether it is a V. */
const ENGINE_TYPES: Array<[string, string]> = [
  ['1', 'Single'],
  ['2', 'Parallel twin'],
  ['2v', 'V-twin'],
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
  if ((eng.cylinders === 2 || eng.cylinders === 6) && eng.vAngle > 0) return `${eng.cylinders}v`;
  return String(eng.cylinders);
}

/** What draw mode says before a route has started. */
const START_HINT = 'Pick a port, a junction, or the open end of a pipe to continue';
const ROUTE_HINT = 'Click to add a corner, or a junction, pipe or turbo inlet to join it';
const BEND_HINT = 'Click a straight to bend, or a bend to bend again';
const PLACE_HINT = 'Click to put it down, or on an open pipe end to attach it';

export class Panel {
  /** Which duct the menu edits. Falls back to the first duct if it disappears. */
  private selectedDuctId = 'runner0';
  /**
   * The graph the current selection belongs to.
   *
   * A preset or a change of cylinder count replaces the graph wholesale, and an id that still exists in
   * the new one is not the same duct: staying on `collector0` after picking a V8 would leave the menu
   * editing a pipe nobody picked.
   * A new graph therefore resets the selection to the first runner.
   */
  private selectionGraph: ExhaustGraph | null = null;
  /**
   * Whether editing one runner edits them all.
   *
   * On by default because a symmetric engine is the normal case — eight identical runners should not need
   * eight identical edits. Turning it off is what unequal-length headers need.
   */
  private linkRunners = true;
  private drawing = false;
  private noticeEl!: HTMLElement;
  private noticeTimer = 0;
  /** When the readout stops showing the last afterfire, ms on `performance.now`'s clock: a pop is shorter than a frame. */
  private placing = false;
  private placingPipe = false;
  private placePipeBtn!: HTMLButtonElement;
  private bendToolBtn!: HTMLButtonElement;
  private bendToolHint!: HTMLElement;
  private bendingTool = false;
  /** The equal-length header tool: whether it is on, and its controls. */
  private headerOn = false;
  private headerMirror = true;
  private headerBtn!: HTMLButtonElement;
  private headerApplyBtn!: HTMLButtonElement;
  private headerHint!: HTMLElement;
  private headerLengthInput!: HTMLInputElement;
  private headerMirrorLabel!: HTMLLabelElement;
  private placePipeHint!: HTMLElement;
  private placeBtn!: HTMLButtonElement;
  private placeHint!: HTMLElement;
  /** The turbo tool's own menu, and in it whether the turbos are kept in sync. */
  private turboTool!: HTMLElement;
  private turboOptions!: HTMLElement;
  private turboSyncRow!: HTMLElement;
  /** The turbo selected in the view, whose menu a right-click opens, and that menu's live readout. */
  private turboId: string | null = null;
  private turboMenuReadout: Record<TurboReading, HTMLElement> | null = null;
  private drawHint!: HTMLElement;
  /** Whether a route is in progress, so the hint can show where it is aimed. */
  private drawingRoute = false;
  private drawBtn!: HTMLButtonElement;
  /** The card beside the toolbar with the tool that is on's hint and settings, and each tool's part of it. */
  private toolOptions!: HTMLElement;
  private drawGroup!: HTMLElement;
  private placePipeGroup!: HTMLElement;
  private bendToolGroup!: HTMLElement;
  private headerGroup!: HTMLElement;
  private placeGroup!: HTMLElement;
  /** The junction selected in the scene, which adding a segment adds at. */
  private jointNode: string | null = null;
  /**
   * The menu a right-click on a segment or a junction opens, on the one selected, which it follows; its row,
   * what it was built for, so it is rebuilt only when that changes, and the exhaust's figures.
   */
  private segMenu: { el: HTMLElement; row: SegmentRow | null; key: string; stats: HTMLElement | null } | null = null;
  private cylSel!: HTMLSelectElement;
  private twinWrap!: HTMLElement;
  private crankSel!: HTMLSelectElement;
  private crankRow!: HTMLElement;
  private vAngleRow!: HTMLElement;
  private offsetToggleEl!: HTMLElement;
  private offsetWrapEl!: HTMLElement;
  private readonly resyncers: Resync[] = [];
  private readonly rpmEl: HTMLElement;
  private readonly launchBtn: HTMLButtonElement;
  private readonly dynoBtn: HTMLButtonElement;
  /** Which of the two runs was last started: the one whose button stops it while it goes. */
  private runKind: 'launch' | 'dyno' = 'launch';
  /**
   * The launch's car and settings. A `null` setting fits it to the engine; the gear ratios' `null` is the
   * car's, or the stock six-speed, and the final drive's and mass's are the car's, or fitted.
   */
  private launch: LaunchSettings;
  private launchRunning = false;
  /** Rebuilds the gearbox list from the settings above. */
  private renderGearbox: () => void = () => {};
  /** Rewrites the gearbox's figures, which follow the engine and the other settings, in place. */
  private refreshGearbox: () => void = () => {};
  /** Where the phasers have the cams, updated from each snapshot. */
  private camReadout!: HTMLElement;
  private lobeReadout!: HTMLElement;
  private lobeText = '';
  /** Whether a two-stage intake was on its short runners at the last readout. */
  private shortRunnersNow = false;
  private camText = '';
  /** Rewrites the intake section's tuning readout if anything it shows has changed. */
  private refreshIntake: () => void = () => {};
  private readonly meterFill: HTMLElement;
  private readonly startBtn: HTMLButtonElement;
  private readonly rateSel: HTMLSelectElement;

  private selected: number | null = null;
  private readonly view: ViewOptions = {
    pressure: true,
    handles: true,
    speed: 1,
  };

  constructor(
    root: HTMLElement,
    tools: HTMLElement,
    private readonly config: EngineConfig,
    private readonly cb: PanelCallbacks,
    sampleRate: number,
    launchSettings: LaunchSettings = autoLaunchSettings(),
  ) {
    this.launch = launchSettings;
    const spec = config.engine;
    const section = sectionRail(root);

    // ---- Transport -------------------------------------------------------
    const transport = section('Transport', 'transport', 'Start and stop the engine, its throttle and load, and pick, save or load an engine.');
    this.startBtn = el('button', 'primary', transport) as HTMLButtonElement;
    this.startBtn.textContent = 'Start engine';
    this.startBtn.addEventListener('click', () => this.cb.onToggleAudio());

    const meter = el('div', 'meter', transport);
    this.meterFill = el('div', 'meter-fill', meter);

    this.rpmEl = el('div', 'big-readout', transport);
    this.rpmEl.textContent = '— rpm';

    this.slider(transport, {
      label: 'Throttle',
      min: 0,
      max: 1,
      step: 0.01,
      value: spec.throttle,
      sync: () => this.config.engine.throttle,
      format: (v) => `${Math.round(v * 100)}%`,
      onInput: (v) => this.cb.onEngine({ throttle: v }),
    }).row.title =
      'The engine speed follows from this and the load: the crank is driven by the gas torque ' +
      'against friction and the load, so the exhaust tuning moves it too.';

    const load = this.slider(transport, {
      label: 'Load',
      min: 0,
      // Past full throttle's worth, so the engine can be bogged down and stalled.
      max: 1.5,
      step: 0.01,
      value: spec.load,
      sync: () => this.config.engine.load,
      format: (v) => `${Math.round(v * 100)}% · ${Math.round(v * fullLoadTorque(this.config.engine, isTurbocharged(this.config.graph)))} N·m`,
      onInput: (v) => this.cb.onEngine({ load: v }),
    });
    load.row.title =
      'Braking torque at the crank, as a share of what this engine makes at full throttle, ' +
      'so the same setting loads a single and a V8 alike.';

    const rateRow = el('div', 'row', transport);
    el('label', '', rateRow).textContent = 'Sample rate';
    const rateSel = el('select', '', rateRow) as HTMLSelectElement;
    this.rateSel = rateSel;
    for (const [hz, label] of SAMPLE_RATES) rateSel.appendChild(option(String(hz), label));
    rateSel.value = String(sampleRate);
    rateSel.addEventListener('change', () => this.cb.onSampleRate(Number(rateSel.value)));
    rateRow.title =
      'The solver takes one step per audio sample, so a lower rate means fewer steps and coarser ' +
      'cells: much less CPU, for a duller exhaust. For phones and slow machines. Changing it ' +
      'restarts the audio, and the pipes warm up again from cold.';

    // ---- Launch ----------------------------------------------------------
    const launch = section('Launch', 'launch', 'A timed standing start through the gears, and the car and gearbox it runs through.');
    this.launchBtn = el('button', 'primary', launch) as HTMLButtonElement;
    this.launchBtn.textContent = 'Start launch';
    this.launchBtn.title =
      'A standing start at full throttle: the clutch slipped at the launch speed, then every gear ' +
      'pulled to the shift point. The run drives the throttle; the sheet times 0–60 mph, the quarter ' +
      'and the half mile, and plots crank power and torque against rpm.';
    this.launchBtn.addEventListener('click', () => {
      if (this.launchRunning) {
        this.cb.onLaunch(null);
        return;
      }
      this.runKind = 'launch';
      this.cb.onLaunch(this.launchConfig());
    });
    this.slider(launch, {
      label: 'Launch at',
      min: 1000,
      max: 12000,
      step: 50,
      value: this.launchConfig().launchRpm,
      sync: () => this.launch.launchRpm ?? this.launchConfig().launchRpm,
      format: (v) => `${Math.round(v)} rpm${this.launch.launchRpm === null ? ' (auto)' : ''}`,
      onInput: (v) => this.setLaunch('launchRpm', v),
    }).row.title =
      'Engine speed the clutch is slipped at off the line, until the car has caught up with it. Auto is ' +
      `half the rev limiter. It stays at least ${LAUNCH_RPM_MARGIN} rpm under the shift point.`;
    this.slider(launch, {
      label: 'Shift at',
      min: 1500,
      max: 12000,
      step: 50,
      value: this.launchConfig().shiftRpm,
      sync: () => this.launch.shiftRpm ?? this.launchConfig().shiftRpm,
      format: (v) => `${Math.round(v)} rpm${this.launch.shiftRpm === null ? ' (auto)' : ''}`,
      onInput: (v) => {
        this.setLaunch('shiftRpm', v);
        this.refreshGearbox();
      },
    }).row.title =
      'Engine speed each gear is pulled to before the next goes in. Auto is just under the rev ' +
      'limiter. A setting past the limiter shifts just under it.';
    this.slider(launch, {
      label: 'Car mass',
      min: 100,
      max: 2500,
      step: 10,
      value: this.launchConfig().mass,
      sync: () => this.launch.mass ?? this.launchConfig().mass,
      format: (v) =>
        `${Math.round(v)} kg${this.launch.mass !== null ? '' : this.launch.car ? ' (stock)' : ' (auto)'}`,
      onInput: (v) => this.setLaunch('mass', v),
    }).row.title =
      'What the engine accelerates, driver included. Stock is the real car\'s kerb weight and a ' +
      `${DRIVER_MASS} kg driver, for an engine from one; auto sizes a car to the engine, about 9 kg per kW. ` +
      'Lighter is quicker, until the tyres can take no more.';
    this.slider(launch, {
      label: 'Shift time',
      min: 0.05,
      max: 1,
      step: 0.01,
      value: this.launchConfig().shiftTime,
      sync: () => this.launch.shiftTime ?? this.launchConfig().shiftTime,
      format: (v) =>
        `${v.toFixed(2)} s${this.launch.shiftTime !== null ? '' : this.launch.car ? ' (stock)' : ' (auto)'}`,
      onInput: (v) => this.setLaunch('shiftTime', v),
    }).row.title =
      'How long each shift takes, from lifting off to full throttle in the next gear. Auto is a quick ' +
      `manual shift, ${MANUAL_SHIFT_TIME} s; a dual-clutch gearbox, as the Corvettes have, takes about 0.1 s.`;
    this.slider(launch, {
      label: 'Tyre grip',
      min: 0.6,
      max: 1.6,
      step: 0.01,
      value: this.launchConfig().tyreGrip,
      sync: () => this.launch.tyreGrip ?? this.launchConfig().tyreGrip,
      format: (v) => `μ ${v.toFixed(2)}${this.launch.tyreGrip !== null ? '' : this.launch.car ? ' (stock)' : ' (auto)'}`,
      onInput: (v) => this.setLaunch('tyreGrip', v),
    }).row.title =
      `The tyres' friction coefficient at their peak grip, driving. Auto is a road tyre, ${TYRE_GRIP.road}. ` +
      `Stock is the real car's: ${TYRE_GRIP.corvette.toFixed(2)} for the Corvettes, from the 1.22 g the Z06 ` +
      'pulls on a skidpad and a little more for grip driving rather than cornering.';
    const dct = toggle(launch, 'Dual-clutch gearbox', this.launchConfig().dualClutch, (v) => this.setLaunch('dualClutch', v));
    dct.title =
      'Shift with no gap in the drive: the next gear\'s clutch takes it as the last one lets go, and the ' +
      'throttle stays open. Off, each shift lifts off and takes the clutch out, as a manual does. Stock ' +
      'for the Corvettes.';
    this.resyncers.push(() => (checkbox(dct).checked = this.launchConfig().dualClutch));
    const awd = toggle(launch, 'All-wheel drive', this.launchIsAwd(), (v) => this.setLaunch('awd', v));
    awd.title =
      'Drive all four wheels, so the tyres can take the whole weight of the car pulling away rather than ' +
      'what is on the rear. It launches harder before the tyres spin. Stock for the Skyline.';
    this.resyncers.push(() => (checkbox(awd).checked = this.launchIsAwd()));
    const tc = toggle(launch, 'Traction control', this.launch.tractionControl, (v) => this.setLaunch('tractionControl', v));
    tc.title =
      'Ease the throttle whenever the driven tyres slip past where they grip best, and open it again as ' +
      'they come back, as a launch control does. Off, an engine with more torque than the tyres can take ' +
      'spins them, and reaches the shift point before the car has the speed for it.';
    this.resyncers.push(() => (checkbox(tc).checked = this.launch.tractionControl));
    this.buildGearbox(launch);
    this.resyncers.push(() => this.renderGearbox());

    // ---- Dyno ------------------------------------------------------------
    const dyno = section('Dyno', 'dyno', 'A full-throttle pull on an engine dyno, for its power and torque curves.');
    this.dynoBtn = el('button', 'primary', dyno) as HTMLButtonElement;
    this.dynoBtn.textContent = 'Start dyno pull';
    this.dynoBtn.title =
      'One pull at full throttle on an engine dyno, the crank driving its absorber at 1:1: held at the ' +
      'start speed, then swept up at the sweep rate to the end. The absorber brakes with whatever torque ' +
      'holds the engine to the sweep. The sheet plots crank power and torque against rpm.';
    this.dynoBtn.addEventListener('click', () => {
      if (this.launchRunning) {
        this.cb.onLaunch(null);
        return;
      }
      this.runKind = 'dyno';
      this.cb.onLaunch(this.dynoConfig());
    });
    this.slider(dyno, {
      label: 'Pull from',
      min: 1000,
      max: 12000,
      step: 50,
      value: this.dynoConfig().launchRpm,
      sync: () => this.launch.dynoFrom ?? this.dynoConfig().launchRpm,
      format: (v) => `${Math.round(v)} rpm${this.launch.dynoFrom === null ? ' (auto)' : ''}`,
      onInput: (v) => this.setLaunch('dynoFrom', v),
    }).row.title =
      `Engine speed the pull starts from, held for a second before the sweep. Auto is a quarter of the rev ` +
      `limiter, and at least ${DYNO_FROM_RPM} rpm. It stays at least ${LAUNCH_RPM_MARGIN} rpm under the end.`;
    this.slider(dyno, {
      label: 'Pull to',
      min: 1500,
      max: 12000,
      step: 50,
      value: this.dynoConfig().shiftRpm,
      sync: () => this.launch.dynoTo ?? this.dynoConfig().shiftRpm,
      format: (v) => `${Math.round(v)} rpm${this.launch.dynoTo === null ? ' (auto)' : ''}`,
      onInput: (v) => this.setLaunch('dynoTo', v),
    }).row.title =
      'Engine speed the pull ends at. Auto is just under the rev limiter. A setting past the limiter ' +
      'ends just under it.';
    this.slider(dyno, {
      label: 'Sweep rate',
      min: 100,
      max: 2000,
      step: 50,
      value: this.dynoConfig().sweepRate,
      sync: () => this.dynoConfig().sweepRate,
      format: (v) => `${Math.round(v)} rpm/s${this.launch.sweepRate === null ? ' (auto)' : ''}`,
      onInput: (v) => this.setLaunch('sweepRate', v),
    }).row.title =
      `How fast the absorber lets the engine speed up. Auto is ${DYNO_SWEEP_RATE} rpm/s, an engine dyno's ` +
      'steady sweep. Faster gives a turbo less time to spool at each speed, so it reads less boost low down.';

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
       * the preset's geometry *before* it would let that carry-over overwrite the geometry just loaded,
       * so going crossplane to flatplane and back would come out with a different exhaust each time. And the
       * collector is always replaced — emptied if the preset has none — so no preset inherits one.
       */
      // A different engine gets a car fitted to it.
      this.resetLaunch(preset.car ?? null);
      this.cb.onEngine(presetEngine(preset, this.config.engine));
      this.config.pipe.length = 0;
      this.config.pipe.push(...preset.pipe());
      this.config.collector.length = 0;
      if (preset.collector) this.config.collector.push(...preset.collector());
      this.cb.onReseed(preset.turbos ?? 0, preset.graph?.());
      this.rebuildAll();
      this.cb.onResetView();
    });

    // ---- Export and import ----------------------------------------------
    // The whole engine, exhaust and all, as a file: to keep, to share, and to bring back.
    const fileRow = el('div', 'row buttons', transport);
    const exportBtn = el('button', '', fileRow) as HTMLButtonElement;
    exportBtn.textContent = 'Export engine';
    exportBtn.title = 'Save this engine to a file: its layout, its settings and its exhaust as drawn.';
    exportBtn.addEventListener('click', () => this.cb.onExportEngine());
    const importBtn = el('button', '', fileRow) as HTMLButtonElement;
    importBtn.textContent = 'Import engine…';
    importBtn.title = 'Load an engine saved by Export engine, in place of this one.';
    const picker = el('input', 'hidden', fileRow) as HTMLInputElement;
    picker.type = 'file';
    picker.accept = '.json,application/json';
    importBtn.addEventListener('click', () => picker.click());
    picker.addEventListener('change', () => {
      const file = picker.files?.[0];
      // Cleared, so choosing the same file again still counts as a change.
      picker.value = '';
      if (!file) return;
      void file.text().then((text) => {
        this.selected = null;
        this.resetLaunch(null);
        this.cb.onImportEngine(text);
        this.cb.onResetView();
      });
    });

    // ---- Layout ----------------------------------------------------------
    const layout = section('Layout', 'layout', 'Cylinders, crank and firing order, headers and turbos.');

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
      const vTwin = cylSel.value === '2v';
      const vee = cylSel.value === '6v' || cylSel.value === '8' || boxer;
      const n = parseInt(cylSel.value, 10) as EngineSpec['cylinders'];
      const eng = this.config.engine;
      this.cb.onEngine({
        cylinders: n,
        // Keep the plumbing sensible for the new count: a single has nothing to merge, and a
        // V engine's default is a collector per bank.
        exhaustLayout: n === 1 ? 'open' : vee ? 'perBank' : 'merged',
        // A V angle is what makes a twin a V-twin or a six a V6, and means nothing on an inline engine. A
        // V-twin keeps the angle it had, and one from a parallel twin gets a Ducati's 90. A boxer is its
        // banks laid flat, 180 degrees apart, on a crank of its own.
        ...(vTwin ? { vAngle: eng.cylinders === 2 && eng.vAngle > 0 && !isBoxer(eng) ? eng.vAngle : 90 } : {}),
        ...(cylSel.value === '8' ? { vAngle: 90 } : {}),
        ...(cylSel.value === '6v' ? { vAngle: 60 } : {}),
        ...(boxer ? { vAngle: 180, crankType: 'boxer' as const } : {}),
        ...(!vTwin && !vee ? { vAngle: 0 } : {}),
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

    const vRow = this.slider(multiWrap, {
      label: 'V angle',
      // A V stays a V: at no angle it is an inline engine, which has its own entry in the Cylinders menu.
      min: 15,
      max: 120,
      step: 1,
      value: spec.vAngle,
      sync: () => this.config.engine.vAngle,
      format: (v) => {
        const off = firingOffsetDeg({ ...this.config.engine, vAngle: v });
        const named = v === 45 ? ' Harley' : v === 90 ? ' Ducati' : '';
        return `${v.toFixed(0)}\u00b0${named} \u2192 fires ${off.toFixed(0)}/${(720 - off).toFixed(0)}`;
      },
      onInput: (v) => {
        this.cb.onEngine({ vAngle: v });
        this.syncStats();
      },
    });
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
    this.resyncers.push(() => (checkbox(offsetToggle).checked = this.config.engine.firingOffset !== null));
    const offsetWrap = el('div', 'subgroup', multiWrap);
    this.offsetWrapEl = offsetWrap;
    offsetWrap.classList.toggle('hidden', spec.firingOffset === null);
    this.slider(offsetWrap, {
      label: 'Firing offset',
      min: 0,
      max: 719,
      step: 1,
      value: spec.firingOffset ?? firingOffsetDeg(spec),
      sync: () => this.config.engine.firingOffset ?? firingOffsetDeg(this.config.engine),
      format: (v) => `${v.toFixed(0)}\u00b0 / ${(720 - v).toFixed(0)}\u00b0`,
      onInput: (v) => {
        this.cb.onEngine({ firingOffset: v });
        this.syncStats();
      },
    });
    showMulti();
    this.syncLayoutOptions();

    // ---- Exhaust ---------------------------------------------------------
    // Built in the view: the tools in its toolbar, and a pipe or junction's settings in the menu a right-click
    // on it opens.

    /**
     * The tools, in the view's toolbar: drawing, placing a pipe, bending one, the equal-length header and
     * placing a turbo. Draw mode is a mode rather than a replacement for the handles: the handles adjust a
     * route that exists — length, bend, diameter — and drawing creates one. Two tools with distinct jobs.
     */
    const bar = el('div', 'toolbar', tools);
    this.toolOptions = el('div', 'tool-options hidden', tools);

    this.drawBtn = toolButton(
      bar,
      TOOL_ICONS.draw,
      'Draw a pipe',
      'From an opening — an exhaust port, the open end of a pipe (to continue it), a loose pipe\'s start, or a ' +
        'junction nothing leaves yet — a pipe runs straight out of it: each click sets how far. Click another pipe\'s end or side, where two of ' +
        'its segments meet, a junction, a turbo\'s inlet or a port to join it, in one bent pipe fitted to it; ' +
        'Shift joins a pipe\'s side square. From the side of a pipe, where its segments meet, or a junction ' +
        'something already leaves, only such a fitted pipe onto something can be drawn. A double-click or Enter finishes in open air; ' +
        'Escape abandons the pipe; right-click finishes it and exits. Turn and bend straights afterwards with ' +
        'the triad and the bend tool.',
    );
    this.drawGroup = el('div', 'tool-group', this.toolOptions);
    el('div', 'tool-name', this.drawGroup).textContent = 'Draw a pipe';
    this.drawHint = el('div', 'hint', this.drawGroup);

    this.placePipeBtn = toolButton(
      bar,
      TOOL_ICONS.placePipe,
      'Place a pipe',
      'Put a straight pipe down in the view, attached to nothing: click where it should start. It runs ' +
        'along the crank, as long as the engine. Select it for its triad, to move it with the arrows and ' +
        'turn it with the rings. It carries no gas until a pipe is drawn into its start, which attaches it.',
    );
    this.placePipeGroup = el('div', 'tool-group', this.toolOptions);
    el('div', 'tool-name', this.placePipeGroup).textContent = 'Place a pipe';
    this.placePipeHint = el('div', 'hint', this.placePipeGroup);

    this.bendToolBtn = toolButton(
      bar,
      TOOL_ICONS.bend,
      'Bend a pipe',
      'Click a straight to bend: two rings appear where it starts, one lying in the pipe’s up-and-down ' +
        'plane and one in its side-to-side plane. Drag the ring of the plane to bend in, and the whole ' +
        'straight curves into one arc; Shift turns in 15 degree steps. It keeps its length, as a tube does ' +
        'when it is bent, and a ghost shows where it is going until you let go.',
    );
    this.bendToolGroup = el('div', 'tool-group', this.toolOptions);
    el('div', 'tool-name', this.bendToolGroup).textContent = 'Bend a pipe';
    this.bendToolHint = el('div', 'hint', this.bendToolGroup);

    this.headerBtn = toolButton(
      bar,
      TOOL_ICONS.header,
      'Equal-length header',
      'Runs pipes all the same length from any openings to one collector: from exhaust ports with nothing ' +
        'on them, bending straight out of them, or on from the open ends of pipes. Click an opening’s dot to ' +
        'pick it or leave it out; until a length is set, it is the shortest that reaches. Drag the triad’s ' +
        'arrows to put the collector where it goes, and its rings to point it; a ghost shows the pipes, the ' +
        'nearer ones swinging on their way to make their length up. Apply or Enter builds it; Escape or ' +
        'right-click abandons it.',
    );
    this.headerGroup = el('div', 'tool-group', this.toolOptions);
    el('div', 'tool-name', this.headerGroup).textContent = 'Equal-length header';
    this.headerHint = el('div', 'hint', this.headerGroup);
    this.headerLengthInput = numberField(this.headerGroup, 'Pipe length', 400, 50, 3000, 1, 'mm', (v) =>
      this.cb.onHeaderLength(v / MM),
    );
    this.headerMirrorLabel = el('label', 'toggle', this.headerGroup) as HTMLLabelElement;
    const mirrorBox = el('input', '', this.headerMirrorLabel) as HTMLInputElement;
    mirrorBox.type = 'checkbox';
    mirrorBox.checked = this.headerMirror;
    this.headerMirrorLabel.append(' Mirror ports onto other bank');
    this.headerMirrorLabel.title =
      'The ports on the bank away from the triad get the mirror image, merging at the mirrored place. Off, ' +
      'every opening merges at the triad. Open pipe ends always merge at the triad.';
    mirrorBox.addEventListener('change', () => {
      this.headerMirror = mirrorBox.checked;
      this.cb.onHeaderMirror(this.headerMirror);
    });
    this.headerApplyBtn = el('button', 'primary', this.headerGroup) as HTMLButtonElement;
    this.headerApplyBtn.textContent = 'Apply';

    this.turboTool = el('div', 'tool-split', bar);
    this.placeBtn = toolButton(
      this.turboTool,
      TOOL_ICONS.turbo,
      'Place a turbo',
      'Put a turbo down in the view, then draw pipes into its inlet: the open flange on the side of its ' +
        'turbine. Put it on the open end of a pipe to attach that pipe as it goes down. Until you draw a ' +
        'pipe from its outlet flange, it exhausts straight to the air there. Click a turbo for its triad: ' +
        'drag an arrow to move it along that axis, a square to move it in that plane, a ring to turn it; ' +
        'shift snaps to 5 mm and 15 degrees. Its pipes follow. Right-click a turbo to set it up; Delete ' +
        'takes it out. The small arrow, or a right-click here, opens the turbos’ menu.',
    );
    /**
     * The turbos' menu, off the tool's button: whether they are kept in sync. In sync, they all run on the
     * engine's settings, so setting any one sets them all; out of it, each has its own.
     */
    const caret = el('button', 'tool-caret', this.turboTool) as HTMLButtonElement;
    caret.setAttribute('aria-label', 'Turbo options');
    caret.title = 'Turbo options';
    this.turboOptions = el('div', 'tool-menu hidden', this.turboTool);
    el('div', 'tool-name', this.turboOptions).textContent = 'Turbos';
    this.turboSyncRow = toggle(this.turboOptions, 'Keep turbos in sync', true, (on) => this.cb.onTurbosSynced(on));
    el('div', 'hint', this.turboOptions).textContent =
      'On, every turbo runs on the same settings, and setting one sets them all. Off, each keeps its own: ' +
      'right-click a turbo to set it. Syncing again puts them all on the first turbo’s.';
    const openOptions = (open: boolean) => {
      this.turboOptions.classList.toggle('hidden', !open);
      this.turboTool.classList.toggle('open', open);
      if (open) document.addEventListener('pointerdown', this.onTurboOptionsOutside, true);
      else document.removeEventListener('pointerdown', this.onTurboOptionsOutside, true);
    };
    this.openTurboOptions = openOptions;
    caret.addEventListener('click', () => openOptions(this.turboOptions.classList.contains('hidden')));
    this.placeBtn.addEventListener('contextmenu', (e) => {
      e.preventDefault();
      openOptions(true);
    });
    this.placeGroup = el('div', 'tool-group', this.toolOptions);
    el('div', 'tool-name', this.placeGroup).textContent = 'Place a turbo';
    this.placeHint = el('div', 'hint', this.placeGroup);

    el('div', 'tool-exit', this.toolOptions).textContent = 'Right-click or Esc to exit the tool';

    this.headerBtn.addEventListener('click', () => {
      this.setHeaderToolState(!this.headerOn);
      this.cb.onHeaderTool(this.headerOn);
    });
    this.headerApplyBtn.addEventListener('click', () => this.cb.onApplyHeader());
    this.bendToolBtn.addEventListener('click', () => {
      this.setBendToolState(!this.bendingTool);
      this.cb.onBendTool(this.bendingTool);
    });
    this.placePipeBtn.addEventListener('click', () => {
      this.setPlacingPipeState(!this.placingPipe);
      this.cb.onPlacePipeMode(this.placingPipe);
    });
    this.drawBtn.addEventListener('click', () => {
      if (!this.drawing && this.bendingTool) {
        this.setBendToolState(false);
        this.cb.onBendTool(false);
      }
      this.setDrawMode(!this.drawing);
      this.cb.onDrawMode(this.drawing);
    });
    this.placeBtn.addEventListener('click', () => {
      this.setPlacingState(!this.placing);
      this.cb.onPlaceMode(this.placing);
    });
    this.syncTools();

    // Why an edit was refused, and such, over the view.
    this.noticeEl = el('div', 'notice-toast hidden', tools.parentElement ?? tools);

    // ---- Engine geometry -------------------------------------------------
    const geo = section('Engine geometry', 'geometry', 'Bore, stroke, rod length, moving masses and compression.');
    this.slider(geo, {
      label: 'Bore',
      min: 0.05,
      max: 0.12,
      step: 0.001,
      value: spec.bore,
      sync: () => this.config.engine.bore,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ bore: v }),
    });
    this.slider(geo, {
      label: 'Stroke',
      min: 0.04,
      max: 0.12,
      step: 0.001,
      value: spec.stroke,
      sync: () => this.config.engine.stroke,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ stroke: v }),
    });
    this.slider(geo, {
      label: 'Rod length',
      min: 0.09,
      max: 0.24,
      step: 0.001,
      value: spec.rodLength,
      sync: () => this.config.engine.rodLength,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ rodLength: v }),
    });
    this.slider(geo, {
      label: 'Reciprocating mass',
      min: 0.1,
      max: 2,
      step: 0.01,
      value: spec.recipMass,
      sync: () => this.config.engine.recipMass,
      unit: 'kg',
      onInput: (v) => this.cb.onEngine({ recipMass: v }),
    }).row.title =
      'Piston, rings, pin and rod small end. Its inertia torque averages to zero over a ' +
      'cycle so it does not change the speed, but it is as large as the gas torque and ' +
      'sets how unevenly the crank turns.';
    this.slider(geo, {
      label: 'Flywheel inertia',
      min: 0.02,
      max: 1.2,
      step: 0.01,
      value: spec.flywheelInertia,
      sync: () => this.config.engine.flywheelInertia,
      unit: 'kg·m²',
      onInput: (v) => this.cb.onEngine({ flywheelInertia: v }),
    });
    this.slider(geo, {
      label: 'Compression ratio',
      min: 6,
      max: 15,
      step: 0.1,
      value: spec.compressionRatio,
      sync: () => this.config.engine.compressionRatio,
      format: (v) => `${v.toFixed(1)}:1`,
      onInput: (v) => this.cb.onEngine({ compressionRatio: v }),
    });

    // ---- Valves ----------------------------------------------------------
    const valves = section('Valves and timing', 'valves', 'Valve sizes, cam timing and lift, and variable valve timing.');
    const headRow = el('div', 'row', valves);
    el('label', '', headRow).textContent = 'Valves per cylinder';
    const headSel = el('select', '', headRow) as HTMLSelectElement;
    headSel.appendChild(option('2', '2 (one intake, one exhaust)'));
    headSel.appendChild(option('4', '4 (two of each)'));
    const headOf = (e: EngineSpec) => (e.exValveCount === 2 && e.inValveCount === 2 ? '4' : '2');
    headSel.value = headOf(spec);
    this.resyncers.push(() => (headSel.value = headOf(this.config.engine)));
    headSel.addEventListener('change', () => {
      const count = headSel.value === '4' ? 2 : 1;
      this.cb.onEngine({ exValveCount: count, inValveCount: count });
    });
    headRow.title =
      'Two small valves open more of the cylinder than one big one: at the same lift, √2 as much ' +
      'curtain for the same total area. That is what lets a four-valve engine breathe at high rpm. ' +
      'The diameters below are each valve\'s.';
    this.slider(valves, {
      label: 'Exhaust valve',
      min: 0.018,
      max: 0.05,
      step: 0.0005,
      value: spec.exValveDia,
      sync: () => this.config.engine.exValveDia,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ exValveDia: v }),
    });
    this.slider(valves, {
      label: 'Intake valve',
      min: 0.018,
      max: 0.056,
      step: 0.0005,
      value: spec.inValveDia,
      sync: () => this.config.engine.inValveDia,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ inValveDia: v }),
    });
    this.slider(valves, {
      label: 'Port length',
      min: 0.01,
      max: 0.2,
      step: 0.001,
      value: spec.portLength,
      sync: () => this.config.engine.portLength,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ portLength: v }),
    }).row.title =
      'The duct from the valve seat to the header flange. It is part of the acoustic ' +
      'system, so the tuned length is measured from the valve, not the flange.';
    this.slider(valves, {
      label: 'Max lift',
      min: 0.002,
      max: 0.016,
      step: 0.0001,
      value: spec.maxLift,
      sync: () => this.config.engine.maxLift,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ maxLift: v }),
    });
    this.slider(valves, {
      label: 'Exhaust opens',
      min: 90,
      max: 180,
      step: 1,
      value: spec.evo,
      sync: () => this.config.engine.evo,
      format: (v) => `${Math.round(180 - v)}° BBDC`,
      onInput: (v) => this.cb.onEngine({ evo: v }),
    });
    this.slider(valves, {
      label: 'Exhaust closes',
      min: 340,
      max: 430,
      step: 1,
      value: spec.evc,
      sync: () => this.config.engine.evc,
      format: (v) => `${Math.round(v - 360)}° ATDC`,
      onInput: (v) => this.cb.onEngine({ evc: v }),
    });
    this.slider(valves, {
      label: 'Intake opens',
      min: 300,
      max: 380,
      step: 1,
      value: spec.ivo,
      sync: () => this.config.engine.ivo,
      format: (v) => `${Math.round(360 - v)}° BTDC`,
      onInput: (v) => this.cb.onEngine({ ivo: v }),
    });
    this.slider(valves, {
      label: 'Intake closes',
      min: 520,
      max: 630,
      step: 1,
      value: spec.ivc,
      sync: () => this.config.engine.ivc,
      format: (v) => `${Math.round(v - 540)}° ABDC`,
      onInput: (v) => this.cb.onEngine({ ivc: v }),
    });

    // ---- Cam profile switching ---------------------------------------------
    el('div', 'subhead', valves).textContent = 'Cam profile switching (VTEC)';
    this.lobeReadout = el('div', 'readout', valves);
    this.lobeReadout.textContent = 'One cam profile';
    this.lobeReadout.title = 'Which lobes the valves are following now.';
    this.slider(valves, {
      label: 'Switch to high cam at',
      min: 0,
      max: 10000,
      step: 100,
      value: spec.camSwitchRpm,
      sync: () => this.config.engine.camSwitchRpm,
      format: (v) => (v > 0 ? `${Math.round(v)} rpm` : 'one profile'),
      onInput: (v) => this.cb.onEngine({ camSwitchRpm: v }),
    }).row.title =
      'Where each valve switches to a second, high-speed cam lobe, with the lift and timing below. ' +
      'Unlike variable valve timing, which slides the valve events together, a second lobe opens ' +
      'the valve for longer and further. It switches back 150 rpm lower. At 0 there is one profile: ' +
      'the timing above.';
    this.slider(valves, {
      label: 'High cam lift',
      min: 0.002,
      max: 0.016,
      step: 0.0001,
      value: spec.highMaxLift,
      sync: () => this.config.engine.highMaxLift,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ highMaxLift: v }),
    });
    const highEvent = (
      label: string,
      key: 'highEvo' | 'highEvc' | 'highIvo' | 'highIvc',
      min: number,
      max: number,
      format: (v: number) => string,
    ) =>
      this.slider(valves, {
        label,
        min,
        max,
        step: 1,
        value: spec[key],
        sync: () => this.config.engine[key],
        format,
        onInput: (v) => this.cb.onEngine({ [key]: v }),
      });
    highEvent('High cam exhaust opens', 'highEvo', 90, 180, (v) => `${Math.round(180 - v)}° BBDC`);
    highEvent('High cam exhaust closes', 'highEvc', 340, 430, (v) => `${Math.round(v - 360)}° ATDC`);
    highEvent('High cam intake opens', 'highIvo', 300, 380, (v) => `${Math.round(360 - v)}° BTDC`);
    highEvent('High cam intake closes', 'highIvc', 520, 660, (v) => `${Math.round(v - 540)}° ABDC`);

    // ---- Variable valve timing ---------------------------------------------
    el('div', 'subhead', valves).textContent = 'Variable valve timing';
    this.camReadout = el('div', 'readout', valves);
    this.camReadout.textContent = 'Cams at rest';
    this.camReadout.title =
      'Where the phasers have the cams now, from the timing above, which is their rest position.';
    const vvt = (label: string, key: 'vvtIntakeLow' | 'vvtIntakeHigh' | 'vvtExhaustLow' | 'vvtExhaustHigh', what: string) =>
      this.slider(valves, {
        label,
        min: 0,
        max: 60,
        step: 1,
        value: spec[key],
        sync: () => this.config.engine[key],
        format: (v) => (v === 0 ? 'at rest' : `${Math.round(v)}° ${what}`),
        onInput: (v) => this.cb.onEngine({ [key]: v }),
      });
    vvt('Intake cam, low speed', 'vvtIntakeLow', 'advanced').row.title =
      'How far the ECU advances the intake cam under load at low speed. Advanced, the intake closes ' +
      'earlier, before the slow-moving charge is pushed back out, and opens earlier, into more overlap.';
    vvt('Intake cam, high speed', 'vvtIntakeHigh', 'advanced').row.title =
      'The same near the rev limit, where a late close lets the runners ram the charge in.';
    vvt('Exhaust cam, low speed', 'vvtExhaustLow', 'retarded').row.title =
      'How far the ECU retards the exhaust cam under load at low speed: the exhaust opens later, ' +
      'getting more work from the expansion, and closes later, into more overlap.';
    vvt('Exhaust cam, high speed', 'vvtExhaustHigh', 'retarded');
    const mapRpm = (label: string, key: 'vvtLowRpm' | 'vvtHighRpm') =>
      this.slider(valves, {
        label,
        min: 1000,
        max: 10000,
        step: 100,
        value: spec[key],
        sync: () => this.config.engine[key],
        format: (v) => `${Math.round(v)} rpm`,
        onInput: (v) => this.cb.onEngine({ [key]: v }),
      });
    mapRpm('Low speed is', 'vvtLowRpm').row.title =
      'Up to this speed the cams take the low-speed settings. Between it and the high speed the map ' +
      'blends from one to the other in a straight line.';
    mapRpm('High speed is', 'vvtHighRpm').row.title = 'From this speed the cams take the high-speed settings.';
    const linked = toggle(valves, 'One phaser for both cams', spec.vvtLinked, (on) =>
      this.cb.onEngine({ vvtLinked: on }),
    );
    linked.title =
      'As a pushrod engine\u2019s single camshaft has: the whole cam moves by the intake\u2019s advance, ' +
      'exhaust lobes with it, and the exhaust settings do nothing. At idle and light load every cam ' +
      'sits at rest, for a steady idle.';
    this.resyncers.push(() => (checkbox(linked).checked = this.config.engine.vvtLinked));

    // ---- Intake ------------------------------------------------------------
    const intake = section('Intake', 'intake', 'Runners, plenum and throttle body.');
    const tuned = el('div', 'readout', intake);
    let tunedKey = '';
    // The auto runner follows the rev limit and the intake valves as well as its own sliders, so this
    // is checked on every readout and rewritten only when something it shows has moved.
    const showTuned = () => {
      const e = this.config.engine;
      const r = intakeRunnerOf(e);
      const short = e.intakeRunnerShortLength;
      const text =
        short > 0
          ? `Runners ${Math.round(r.length * 1000)} mm, tuned for about ${formatRpm(runnerTunedRpm(e))} rpm, ` +
            `and ${Math.round(short * 1000)} mm, for about ` +
            `${formatRpm(runnerTunedRpm({ ...e, intakeRunnerLength: short }))} rpm, ` +
            `${(r.diameter * 1000).toFixed(1)} mm bore · on the ${this.shortRunnersNow ? 'short' : 'long'} ones`
          : `Runners ${Math.round(r.length * 1000)} mm × ${(r.diameter * 1000).toFixed(1)} mm, ` +
            `tuned for about ${formatRpm(runnerTunedRpm(e))} rpm`;
      if (text === tunedKey) return;
      tunedKey = text;
      tuned.textContent = text;
    };
    showTuned();
    this.refreshIntake = showTuned;
    tuned.title =
      'The speed at which the runners ram the charge in hardest: where their quarter-wave ' +
      'resonance is 2.3 times the crank speed. Torque peaks near it.';
    this.slider(intake, {
      label: 'Intake runner length',
      min: 0,
      max: 0.9,
      step: 0.005,
      value: spec.intakeRunnerLength,
      sync: () => this.config.engine.intakeRunnerLength,
      format: (v) =>
        v > 0
          ? `${Math.round(v * 1000)} mm`
          : `auto (${Math.round(intakeRunnerOf(this.config.engine).length * 1000)} mm)`,
      onInput: (v) => {
        this.cb.onEngine({ intakeRunnerLength: v });
        showTuned();
      },
    }).row.title =
      'From the intake valve to the plenum. The air in it rams the charge in after bottom dead ' +
      'centre, most strongly at the speed its length is tuned for: longer for low-rpm torque, ' +
      'shorter for high-rpm power. At 0 it is tuned to three quarters of the rev limit. With a ' +
      'two-stage intake, this is the long runners\u2019 length.';
    this.slider(intake, {
      label: 'Short runner length',
      min: 0,
      max: 0.9,
      step: 0.005,
      value: spec.intakeRunnerShortLength,
      sync: () => this.config.engine.intakeRunnerShortLength,
      format: (v) => (v > 0 ? `${Math.round(v * 1000)} mm` : 'single stage'),
      onInput: (v) => {
        this.cb.onEngine({ intakeRunnerShortLength: v });
        showTuned();
      },
    }).row.title =
      'A two-stage intake: each cylinder has a second, shorter path to the plenum, which a flap opens ' +
      'at the switch speed below. The long runners make the torque low down and the short ones the ' +
      'power at the top. At 0 there is one runner length.';
    this.slider(intake, {
      label: 'Switch to short runners at',
      min: 1000,
      max: 10000,
      step: 100,
      value: spec.intakeSwitchRpm,
      sync: () => this.config.engine.intakeSwitchRpm,
      format: (v) => `${Math.round(v)} rpm`,
      onInput: (v) => this.cb.onEngine({ intakeSwitchRpm: v }),
    }).row.title =
      'Where a two-stage intake switches to its short runners. It switches back 150 rpm lower, so it ' +
      'does not flap back and forth at the switch speed. Best where the two sets make the same torque.';
    this.slider(intake, {
      label: 'Intake runner bore',
      min: 0,
      max: 0.08,
      step: 0.0005,
      value: spec.intakeRunnerDia,
      sync: () => this.config.engine.intakeRunnerDia,
      format: (v) =>
        v > 0
          ? `${(v * 1000).toFixed(1)} mm`
          : `auto (${(intakeRunnerOf(this.config.engine).diameter * 1000).toFixed(1)} mm)`,
      onInput: (v) => {
        this.cb.onEngine({ intakeRunnerDia: v });
        showTuned();
      },
    }).row.title =
      'At 0 it passes the intake valves\' area, a little narrowed, as a port does. Narrower ' +
      'speeds the air up and rams harder at low rpm; wider breathes better at the top.';
    this.slider(intake, {
      label: 'Plenum volume',
      min: 0,
      max: 0.02,
      step: 0.0001,
      value: spec.plenumVolume,
      sync: () => this.config.engine.plenumVolume,
      format: (v) =>
        v > 0 ? `${(v * 1000).toFixed(1)} L` : `auto (${(plenumVolumeOf(this.config.engine) * 1000).toFixed(1)} L)`,
      onInput: (v) => this.cb.onEngine({ plenumVolume: v }),
    }).row.title =
      'The manifold the runners draw from, downstream of the throttle. It holds what the cylinders ' +
      'push back up their runners and hands it back next cycle. At 0 it is one and a half times ' +
      'the engine\u2019s displacement.';
    this.slider(intake, {
      label: 'Throttle bore',
      min: 0,
      max: 0.12,
      step: 0.0005,
      value: spec.throttleDia,
      sync: () => this.config.engine.throttleDia,
      format: (v) =>
        v > 0 ? `${(v * 1000).toFixed(1)} mm` : `auto (${(throttleDiaOf(this.config.engine) * 1000).toFixed(1)} mm)`,
      onInput: (v) => this.cb.onEngine({ throttleDia: v }),
    }).row.title =
      'At 0 it is sized so the engine can breathe at full throttle and 7000 rpm, with the air at ' +
      '25 m/s through it. Smaller chokes the top end; larger makes the throttle touchier at small ' +
      'openings.';

    // ---- Combustion ------------------------------------------------------
    const comb = section('Combustion', 'combustion', 'Rev limiter, ignition advance, burn duration and mixture.');
    const revLimit = this.slider(comb, {
      label: 'Rev limiter',
      min: 2000,
      max: 12000,
      step: 100,
      value: spec.revLimit,
      sync: () => this.config.engine.revLimit,
      unit: 'rpm',
      onInput: (v) => this.cb.onEngine({ revLimit: v }),
    });
    revLimit.row.title =
      'The spark is cut above this and returns once the crank has dropped back, so the engine ' +
      'bounces off it.';

    this.slider(comb, {
      label: 'Ignition advance',
      min: 0,
      max: 50,
      step: 1,
      value: 720 - spec.ignition,
      sync: () => 720 - this.config.engine.ignition,
      unit: '° BTDC',
      // Stored as deg ATDC; 25 deg BTDC is 695.
      onInput: (v) => this.cb.onEngine({ ignition: 720 - v }),
    });
    this.slider(comb, {
      label: 'Burn duration',
      min: 15,
      max: 110,
      step: 1,
      value: spec.burnDuration,
      sync: () => this.config.engine.burnDuration,
      unit: '°',
      onInput: (v) => this.cb.onEngine({ burnDuration: v }),
    }).row.title =
      'How long the charge takes to burn at full throttle, stoichiometric, at 10 m/s mean ' +
      'piston speed. Each cycle burns faster or slower than this with its own flame speed: ' +
      'slower at part throttle, with residual gas and lean, and a little slower the faster ' +
      'the engine turns.';
    this.slider(comb, {
      label: 'Mixture',
      min: 0.7,
      max: 1.6,
      step: 0.01,
      value: spec.lambda,
      sync: () => this.config.engine.lambda,
      format: (v) =>
        `λ ${v.toFixed(2)}${Math.abs(v - 1) < 0.005 ? ' (stoichiometric)' : v < 1 ? ' (rich)' : ' (lean)'}`,
      onInput: (v) => this.cb.onEngine({ lambda: v }),
    }).row.title =
      'Air-fuel ratio as a multiple of stoichiometric. Rich, the extra fuel has no oxygen to ' +
      'burn with; lean, each charge carries less fuel and burns slower, and past about 1.5 ' +
      'cycles start to misfire.';
    const fuelCut = toggle(comb, 'Overrun fuel cut', spec.fuelCut, (on) => this.cb.onEngine({ fuelCut: on }));
    this.resyncers.push(() => (checkbox(fuelCut).checked = this.config.engine.fuelCut));
    fuelCut.title =
      'With the throttle shut above 1500 rpm the fuel stops, as an injected engine does, until ' +
      'the speed falls below 1200 or the throttle opens. Off, the engine keeps firing weakly on ' +
      'the air leaking past the throttle, as a carburettor does.';
    const crackle = toggle(comb, 'Overrun crackle', spec.overrunCrackle, (on) =>
      this.cb.onEngine({ overrunCrackle: on }),
    );
    this.resyncers.push(() => (checkbox(crackle).checked = this.config.engine.overrunCrackle));
    crackle.title =
      'A "pops and bangs" map: for up to 3 s after the throttle shuts above 2500 rpm, it holds off ' +
      'the fuel cut, cracks the throttle open and fires the spark long after top dead centre, ' +
      'skipping it on some cycles. The unburned charges light in the hot header and pop.';
    this.slider(comb, {
      label: 'Crackle',
      min: 0,
      max: 1,
      step: 0.05,
      value: spec.crackleIntensity,
      sync: () => this.config.engine.crackleIntensity,
      format: (v) => `${Math.round(v * 100)}%`,
      onInput: (v) => this.cb.onEngine({ crackleIntensity: v }),
    }).row.title =
      'How hard the crackle map works: the spark from 15° to 45° after top dead centre, from 10% to ' +
      '35% of the sparks skipped, and the throttle further open to feed them.';
    this.slider(comb, {
      label: 'Cycle-to-cycle scatter',
      min: 0,
      max: 2.5,
      step: 0.05,
      value: spec.combustionVariability,
      sync: () => this.config.engine.combustionVariability,
      format: (v) => (v === 0 ? 'off (identical cycles)' : `${v.toFixed(2)}×`),
      onInput: (v) => this.cb.onEngine({ combustionVariability: v }),
    }).row.title =
      'How much the flame kernel varies from one cycle to the next. 1.0 is realistic: ' +
      'about 2% variation in indicated work under load, over 10% near idle. Set it to ' +
      'zero and every cycle becomes identical — which is what makes simulated engines ' +
      'sound like a looped sample.';
    this.slider(comb, {
      label: 'Pipe wall thickness',
      min: 0.0004,
      max: 0.005,
      step: 0.0001,
      value: spec.pipeWallThickness,
      sync: () => this.config.engine.pipeWallThickness,
      scale: MM,
      unit: 'mm',
      onInput: (v) => this.cb.onEngine({ pipeWallThickness: v }),
    }).row.title =
      "The wall's thermal mass, so how long the system takes to come up to temperature — " +
      'tens of seconds for typical 1.2 mm tubing. Gas temperature sets the speed of sound, ' +
      'so the note genuinely shifts as the pipe warms.';
    this.slider(comb, {
      label: 'Air speed past pipe',
      min: 0,
      max: 45,
      step: 0.5,
      value: spec.airSpeed,
      sync: () => this.config.engine.airSpeed,
      format: (v) =>
        v < 0.5 ? 'still air' : `${v.toFixed(0)} m/s (${(v * 3.6).toFixed(0)} km/h)`,
      onInput: (v) => this.cb.onEngine({ airSpeed: v }),
    }).row.title =
      'Cools the pipe wall, which cools the gas, which slows the wave speed and drops the ' +
      'tuning. Radiation off oxidised steel matters as much as convection here.';
    this.slider(comb, {
      label: 'Solver resolution',
      min: 0.035,
      max: 0.12,
      step: 0.001,
      value: spec.pipeCellSize,
      sync: () => this.config.engine.pipeCellSize,
      format: (v) =>
        `${(v * 1000).toFixed(0)} mm cells · ~${(550 / (10 * v) / 1000).toFixed(1)} kHz`,
      onInput: (v) => this.cb.onEngine({ pipeCellSize: v }),
    }).row.title =
      'Cell length for the exhaust gas-dynamics solver. Smaller cells resolve higher ' +
      'frequencies and cost more. The solver takes one step per audio sample, so cells ' +
      'cannot be shorter than about 35 mm at 48 kHz (51 mm at 32 kHz, 69 mm at 24 kHz); a big engine may be given ' +
      'coarser cells than asked for, to keep it in real time.';
    this.slider(comb, {
      label: 'Port gas temp',
      min: 350,
      max: 1250,
      step: 5,
      value: spec.portGasTemp,
      sync: () => this.config.engine.portGasTemp,
      format: (v) => `${Math.round(v)} K · c=${Math.round(speedOfSound(v))} m/s`,
      onInput: (v) => {
        this.cb.onEngine({ portGasTemp: v });
        // The tuning readout's wave speed comes from this.
        this.syncStats();
      },
    });

    // ---- Listener --------------------------------------------------------
    const mix = section('Listener', 'listener', 'Where the microphone sits, and the mix of what it hears.');
    this.slider(mix, {
      label: 'Mic distance',
      min: 0.3,
      max: 12,
      step: 0.1,
      value: spec.micDistance,
      sync: () => this.config.engine.micDistance,
      unit: 'm',
      format: (v) => `${v.toFixed(1)} m`,
      onInput: (v) => this.cb.onEngine({ micDistance: v }),
    });
    this.slider(mix, {
      label: 'Ear height',
      min: 0.05,
      max: 3,
      step: 0.05,
      value: spec.micHeight,
      sync: () => this.config.engine.micHeight,
      unit: 'm',
      format: (v) => `${v.toFixed(2)} m`,
      onInput: (v) => this.cb.onEngine({ micHeight: v }),
    });
    this.slider(mix, {
      label: 'Exhaust height',
      min: 0.05,
      max: 2,
      step: 0.05,
      value: spec.exhaustHeight,
      sync: () => this.config.engine.exhaustHeight,
      unit: 'm',
      format: (v) => `${v.toFixed(2)} m`,
      onInput: (v) => this.cb.onEngine({ exhaustHeight: v }),
    });
    const spacingRow = this.slider(mix, {
      label: 'Mouth spacing',
      min: 0,
      max: 2.5,
      step: 0.05,
      value: spec.mouthSpacing,
      sync: () => this.config.engine.mouthSpacing,
      unit: 'm',
      format: (v) => (v < 0.03 ? 'coincident' : `${v.toFixed(2)} m`),
      onInput: (v) => this.cb.onEngine({ mouthSpacing: v }),
    });
    spacingRow.row.title =
      'How far apart the tailpipes are. Only does anything with more than one of them, and then ' +
      'it matters a great deal: set it to zero and the mouths sum at a single point, where the ' +
      'two banks of a flatplane V8 fire in antiphase and annihilate their own firing order — ' +
      '44 dB of it — jumping the engine an octave. Real pipes are a metre or so apart.';

    const spreadRow = this.slider(mix, {
      label: 'Cylinder spread',
      min: 0,
      max: 1,
      step: 0.01,
      value: spec.cylinderSpread,
      sync: () => this.config.engine.cylinderSpread,
      format: (v) => (v < 0.01 ? 'perfectly matched' : `${(v * 4).toFixed(1)}%`),
      onInput: (v) => this.cb.onEngine({ cylinderSpread: v }),
    });
    spreadRow.row.title =
      'How unequally the cylinders breathe, as a spread in the pressure each intake runner opens ' +
      'onto. No two cylinders of ' +
      'a real engine are matched, and that is what stops the firing orders cancelling perfectly. ' +
      'At zero an inline four is a pure tone on one frequency with no rumble under it; at 4% the ' +
      'low orders sit 35 dB down, where real engines measure 20 to 35.';

    this.slider(mix, {
      label: 'Ground reflection',
      min: 0,
      max: 1,
      step: 0.01,
      value: spec.groundReflection,
      sync: () => this.config.engine.groundReflection,
      format: (v) =>
        v < 0.15 ? 'anechoic' : v < 0.5 ? `${v.toFixed(2)} grass` : `${v.toFixed(2)} asphalt`,
      onInput: (v) => this.cb.onEngine({ groundReflection: v }),
    }).row.title =
      'The ground sends a second, slightly later copy of everything to your ear, and the ' +
      'two comb-filter each other. Set it to zero to hear the engine in free space — ' +
      'which is how simulated engines usually sound, and why they sound wrong.';
    this.slider(mix, {
      label: 'Output gain',
      min: 0,
      max: 1.5,
      step: 0.01,
      value: spec.outputGain,
      sync: () => this.config.engine.outputGain,
      format: (v) => v.toFixed(2),
      onInput: (v) => this.cb.onEngine({ outputGain: v }),
    });
    this.slider(mix, {
      label: 'Mechanical noise',
      min: 0,
      max: 1,
      step: 0.01,
      value: spec.mechNoise,
      sync: () => this.config.engine.mechNoise,
      format: (v) => `${Math.round(v * 100)}%`,
      onInput: (v) => this.cb.onEngine({ mechNoise: v }),
    });
    this.slider(mix, {
      label: 'Valve throat noise',
      min: 0,
      max: 1,
      step: 0.01,
      value: spec.throatNoise,
      sync: () => this.config.engine.throatNoise,
      format: (v) => `${Math.round(v * 100)}%`,
      onInput: (v) => this.cb.onEngine({ throatNoise: v }),
    });

    this.slider(mix, {
      label: 'Turbo sound',
      min: 0,
      max: 2,
      step: 0.01,
      value: spec.turboNoise,
      sync: () => this.config.engine.turboNoise,
      format: (v) => `${Math.round(v * 100)}%`,
      onInput: (v) => this.cb.onEngine({ turboNoise: v }),
    }).row.title =
      'The turbos’ own sounds: the compressors’ whine, the blow-off valves, the flutter of a ' +
      'surge and the wastegates’ rattle. 100% is realistic.';

    // ---- View ------------------------------------------------------------
    const viewSec = section('View', 'view', 'What the 3D view shows, and how fast the simulation runs.');
    toggle(viewSec, 'Pressure colouring', this.view.pressure, (on) => {
      this.view.pressure = on;
      this.cb.onView(this.view);
    });
    toggle(viewSec, 'Edit handles', this.view.handles, (on) => {
      this.view.handles = on;
      this.cb.onView(this.view);
    });
    const speedRow = el('div', 'row', viewSec);
    el('label', '', speedRow).textContent = 'Speed';
    const speedSel = el('select', '', speedRow) as HTMLSelectElement;
    for (const [scale, label] of SPEEDS) speedSel.appendChild(option(String(scale), label));
    speedSel.value = String(this.view.speed);
    speedSel.addEventListener('change', () => {
      this.view.speed = Number(speedSel.value);
      this.cb.onView(this.view);
    });
    speedRow.title =
      'Slow the simulation down to watch the pressure waves travel the pipes. The sound slows and ' +
      'drops in pitch with it; the rpm shown is still the engine\'s own.';
    const fit = el('button', '', viewSec) as HTMLButtonElement;
    fit.textContent = 'Frame the exhaust';
    fit.addEventListener('click', () => this.cb.onResetView());

    this.rebuildPipeList();
  }

  /**
   * The car and gearbox a launch runs through: the user's settings, then the real car where the engine
   * comes from one, and a fit to the engine for the rest.
   */
  private launchConfig(): LaunchConfig {
    const eng = this.config.engine;
    const stock = this.launch.car;
    const fit = fitLaunch(eng, isTurbocharged(this.config.graph), this.launch.ratios ?? stock?.ratios ?? LAUNCH_RATIOS);
    const shiftRpm = Math.min(this.launch.shiftRpm ?? fit.shiftRpm, eng.revLimit - 50);
    return {
      ...fit,
      finalDrive: this.launch.finalDrive ?? stock?.finalDrive ?? fit.finalDrive,
      tyreRadius: stock?.tyreRadius ?? fit.tyreRadius,
      tyreGrip: this.launch.tyreGrip ?? stock?.tyreGrip ?? fit.tyreGrip,
      drivenLoad: this.launchDrivenLoad(),
      tractionControl: this.launch.tractionControl,
      shiftTime: this.launch.shiftTime ?? stock?.shiftTime ?? fit.shiftTime,
      dualClutch: this.launch.dualClutch ?? stock?.dualClutch ?? fit.dualClutch,
      mass: this.launch.mass ?? stock?.mass ?? fit.mass,
      shiftRpm,
      launchRpm: Math.max(Math.min(this.launch.launchRpm ?? fit.launchRpm, shiftRpm - LAUNCH_RPM_MARGIN), 1000),
    };
  }

  /** A dyno pull through the user's settings, the rest fitted to the engine. */
  private dynoConfig(): LaunchConfig {
    return fitDyno(this.config.engine, this.launch.dynoFrom, this.launch.dynoTo, this.launch.sweepRate);
  }

  /** Whether a launch drives all four wheels: the user's choice, or the real car's. */
  private launchIsAwd(): boolean {
    return this.launch.awd ?? (this.launch.car?.drivenLoad ?? 0) >= DRIVEN_LOAD.awd;
  }

  /** The share of the car's weight a launch's driven wheels carry: the real car's where it drives them. */
  private launchDrivenLoad(): number {
    if (this.launchIsAwd()) return DRIVEN_LOAD.awd;
    const own = this.launch.car?.drivenLoad;
    return own !== undefined && own < DRIVEN_LOAD.awd ? own : DRIVEN_LOAD.rwd;
  }

  /**
   * Every launch setting back to auto, for a different engine: the car it comes from where there is one,
   * and a car fitted to it otherwise.
   */
  private resetLaunch(car: Car | null): void {
    this.launch = autoLaunchSettings(car);
    this.cb.onLaunchSettings();
  }

  /** One launch setting changed by the user. */
  private setLaunch<K extends keyof LaunchSettings>(key: K, value: LaunchSettings[K]): void {
    this.launch[key] = value;
    this.cb.onLaunchSettings();
  }

  /** The launch's car and settings, for a link to carry. */
  launchSettings(): LaunchSettings {
    return this.launch;
  }

  /**
   * The gearbox: a ratio for each gear, which can be added and taken away, and the final drive. Each gear
   * shows the road speed it reaches at the shift point, so the gaps between them can be judged.
   */
  private buildGearbox(parent: HTMLElement): void {
    const head = el('div', 'subhead', parent);
    const list = el('div', 'subgroup', parent);
    const finalField = el('div', 'field', parent);
    let finalInput: HTMLInputElement | null = null;
    const finalLabel = (): string =>
      `Final drive${this.launch.finalDrive !== null ? '' : this.launch.car ? ' (stock)' : ' (auto)'}`;
    const buttons = el('div', 'row buttons', parent);
    const addBtn = el('button', '', buttons) as HTMLButtonElement;
    addBtn.textContent = 'Add gear';
    addBtn.title = `Add a gear above the top one, a step taller. At most ${MAX_GEARS}.`;
    const resetBtn = el('button', '', buttons) as HTMLButtonElement;
    resetBtn.textContent = 'Reset gearing';
    let speeds: HTMLElement[] = [];

    const ratios = (): number[] => [...(this.launch.ratios ?? this.launch.car?.ratios ?? LAUNCH_RATIOS)];
    const setRatios = (next: number[]): void => {
      this.setLaunch('ratios', next);
      this.renderGearbox();
    };

    this.refreshGearbox = () => {
      const cfg = this.launchConfig();
      const edited = this.launch.ratios !== null || this.launch.finalDrive !== null;
      head.textContent = this.launch.car
        ? `Gearbox · ${this.launch.car.name}${edited ? ', edited' : ''}`
        : 'Gearbox';
      resetBtn.title = this.launch.car
        ? `Back to the ${this.launch.car.name}'s own ratios and final drive.`
        : 'Back to the stock close-ratio six-speed, with the final drive fitted to the engine.';
      const shiftOmega = (cfg.shiftRpm * 2 * Math.PI) / 60;
      cfg.ratios.forEach((r, i) => {
        const mph = ((shiftOmega * cfg.tyreRadius) / (r * cfg.finalDrive)) * 2.2369363;
        if (speeds[i]) speeds[i]!.textContent = `${Math.round(mph)} mph`;
      });
      if (finalInput && this.launch.finalDrive === null) finalInput.value = round(cfg.finalDrive, 3);
      const label = finalField.querySelector('label');
      if (label) label.textContent = finalLabel();
    };

    this.renderGearbox = () => {
      list.replaceChildren();
      speeds = [];
      const current = ratios();
      current.forEach((ratio, i) => {
        const row = el('div', 'gear-row', list);
        el('span', 'gear-name', row).textContent = ordinal(i + 1);
        const wrap = el('div', 'number-wrap', row);
        const input = el('input', '', wrap) as HTMLInputElement;
        input.type = 'number';
        input.min = '0.3';
        input.max = '6';
        input.step = '0.001';
        input.value = round(ratio, 3);
        input.title = `${ordinal(i + 1)} gear's ratio: engine turns per gearbox output turn.`;
        el('span', 'unit', wrap).textContent = ':1';
        // Applied on `change`, as `numberInto` does, so a half-typed ratio is not committed.
        input.addEventListener('change', () => {
          const v = Number(input.value);
          if (input.value.trim() === '' || !Number.isFinite(v)) return;
          const clamped = Math.max(0.3, Math.min(6, v));
          input.value = round(clamped, 3);
          const next = ratios();
          next[i] = clamped;
          this.setLaunch('ratios', next);
          this.refreshGearbox();
        });
        speeds.push(el('span', 'gear-speed', row));
        const remove = el('button', 'tool', row) as HTMLButtonElement;
        remove.textContent = '×';
        remove.title = `Take out ${ordinal(i + 1)} gear`;
        remove.disabled = current.length <= MIN_GEARS;
        remove.addEventListener('click', () => setRatios(ratios().filter((_, k) => k !== i)));
      });
      addBtn.disabled = current.length >= MAX_GEARS;

      finalField.replaceChildren();
      finalInput = numberInto(finalField, finalLabel(), this.launchConfig().finalDrive, 1, 10, 0.001, ':1', (v) => {
        this.setLaunch('finalDrive', v);
        this.refreshGearbox();
      }, 3);
      finalField.title =
        'Turns of the gearbox output per turn of the wheels. Auto gears the top gear so the shift point ' +
        'comes where the car would run out of power against its drag; stock is the real car\'s, for an engine ' +
        'from one. Higher is quicker off the line, lower is faster at the top.';
      this.refreshGearbox();
    };

    addBtn.addEventListener('click', () => {
      const current = ratios();
      if (current.length >= MAX_GEARS) return;
      const top = current[current.length - 1]!;
      setRatios([...current, Math.max(top * 0.82, 0.3)]);
    });
    resetBtn.addEventListener('click', () => {
      this.setLaunch('ratios', null);
      this.setLaunch('finalDrive', null);
      this.renderGearbox();
    });
    this.renderGearbox();
  }

  /** `slider`, kept in step with the config when it has a `sync`. */
  private slider(parent: HTMLElement, o: SliderOpts): ReturnType<typeof slider> {
    const s = slider(parent, o);
    const read = o.sync;
    if (read) this.resyncers.push(() => s.render(read()));
    return s;
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
   * Offer only the controls that mean something for this engine.
   *
   * An inline engine's V angle is meaningless; only a V8 has a crank choice; and the firing-offset
   * override is a twin's shared-crankpin escape hatch, not a general control.
   */
  private syncLayoutOptions(): void {
    const eng = this.config.engine;
    const n = eng.cylinders;
    const plan = firingPlan(eng);

    this.crankRow.classList.toggle('hidden', n !== 8);
    // A boxer's banks are flat by definition; at any other angle it would be a V on a boxer's crank.
    // Nor has a parallel twin one: it is the V-twin's entry that has.
    this.vAngleRow.classList.toggle('hidden', plan.bankCount < 2 || isBoxer(eng) || (n === 2 && eng.vAngle === 0));
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

  /** Show a sample rate chosen somewhere other than the menu. */
  setSampleRate(hz: number): void {
    this.rateSel.value = String(hz);
  }

  // -------------------------------------------------------------------------
  // Segment menu
  // -------------------------------------------------------------------------

  private addSegment(kind: SegmentKind): void {
    // With a junction selected, at its opening, which the owner does: it may make a pipe to hold it.
    if (this.jointNode) {
      this.cb.onAddAtJunction(this.jointNode, kind);
      return;
    }
    const pipe = this.currentSegments();
    // Added to the pipe as drawn: before a fitted bend, which is fitted to what comes before it.
    const at = this.lockedFrom() ?? pipe.length;
    const last = pipe[at - 1];
    const dIn = last ? segmentDiameter(last, 1) : 0.042;
    pipe.splice(
      at,
      0,
      makeSegment({
        kind,
        length: kind === 'chamber' ? 0.3 : 0.25,
        dIn,
        dOut: kind === 'chamber' ? dIn * 3 : dIn,
      }),
    );
    this.selected = at;
    this.commit();
    this.rebuildPipeList();
    this.cb.onSelect(this.selected);
  }

  /** Whether "Apply to every cylinder" is on. */
  get runnersLinked(): boolean {
    return this.linkRunners;
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
   *
   * `emptied` is a runner a delete left with nothing drawn, which the others follow (`copyToSiblingRunners`).
   */
  private commit(emptied = false): void {
    const graph = this.config.graph;
    const duct = this.currentDuct();
    if (this.linkRunners && graph && duct) copyToSiblingRunners(graph, duct, this.config.engine, emptied);
    this.cb.onPipe();
  }

  /** Where the edited pipe stops being editable: its fitted bend, and any swing before it (`lockedFrom`). */
  private lockedFrom(): number | null {
    const duct = this.currentDuct();
    return duct ? lockedFrom(duct) : null;
  }

  /** The duct the menu is editing, or the first one if the selection has gone stale. */
  /** The pipe the menu edits, if there is one. */
  get ductId(): string | undefined {
    return this.currentDuct()?.id;
  }

  private currentDuct(): ExhaustDuct | null {
    const graph = this.config.graph;
    if (!graph || graph.ducts.length === 0) return null;
    return graph.ducts.find((d) => d.id === this.selectedDuctId) ?? graph.ducts[0]!;
  }

  /** Segments the menu edits. Falls back to `config.pipe` when there is no graph yet, so callers need no guard. */
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
    this.renderMenu();
  }

  /**
   * Open the menu of the selected segment, or junction, at `x`, `y` on the page: its settings, and what to
   * carry on with.
   */
  openMenu(x: number, y: number): void {
    this.closeMenu();
    if (this.selected === null && this.jointNode === null && this.turboId === null) return;
    const menu = el('div', 'seg-menu', document.body);
    this.segMenu = { el: menu, row: null, key: '', stats: null };
    this.renderMenu();
    if (!this.segMenu) return;
    const box = menu.getBoundingClientRect();
    menu.style.left = `${Math.max(8, Math.min(x, innerWidth - box.width - 8))}px`;
    menu.style.top = `${Math.max(8, Math.min(y, innerHeight - box.height - 8))}px`;
    document.addEventListener('pointerdown', this.onMenuOutside, true);
    window.addEventListener('keydown', this.onMenuKey);
  }

  closeMenu(): void {
    if (!this.segMenu) return;
    this.segMenu.el.remove();
    this.segMenu = null;
    document.removeEventListener('pointerdown', this.onMenuOutside, true);
    window.removeEventListener('keydown', this.onMenuKey);
  }

  private onMenuOutside = (e: PointerEvent): void => {
    if (this.segMenu && !this.segMenu.el.contains(e.target as Node)) this.closeMenu();
  };

  private onMenuKey = (e: KeyboardEvent): void => {
    if (e.key === 'Escape') this.closeMenu();
  };

  /** Show the selected turbo, junction or segment in the menu, or close it with none selected. */
  private renderMenu(): void {
    if (!this.segMenu) return;
    if (this.turboId !== null) this.renderTurboMenu(this.turboId);
    else if (this.jointNode !== null) this.renderJointMenu(this.jointNode);
    else this.renderSegmentMenu();
  }

  /** The menu's head: `title`, and a button closing it. */
  private menuHead(title: string): void {
    const head = el('div', 'seg-menu-head', this.segMenu!.el);
    el('span', '', head).textContent = title;
    const close = el('button', 'tool', head) as HTMLButtonElement;
    close.textContent = '×';
    close.title = 'Close';
    close.addEventListener('click', () => this.closeMenu());
  }

  /** Buttons adding a pipe or a chamber, as `addSegment` does, under `label`. */
  private addButtons(label: string): void {
    const add = el('div', 'seg-menu-add', this.segMenu!.el);
    el('label', '', add).textContent = label;
    const buttons = el('div', 'row buttons', add);
    for (const kind of ['pipe', 'chamber'] as SegmentKind[]) {
      const b = el('button', '', buttons) as HTMLButtonElement;
      b.textContent = `+ ${kind}`;
      b.addEventListener('click', () => this.addSegment(kind));
    }
  }

  /**
   * A junction's menu. A junction has nothing to set — its size and shape follow from the pipes meeting at
   * it — so what it offers is what it joins, and ways to carry on out of it.
   */
  private renderJointMenu(node: string): void {
    const menu = this.segMenu!;
    const graph = this.config.graph;
    if (!graph) {
      this.closeMenu();
      return;
    }
    const ends = endsAt(graph, node);
    const key = ['joint', node, ...ends.map((e) => `${e.duct.id}:${e.end}`)].join('|');
    if (key === menu.key) return;
    menu.key = key;
    menu.row = null;
    menu.stats = null;
    menu.el.replaceChildren();
    const junctions = nodeOrder(graph).filter((n) => !turboAt(graph, n));
    this.menuHead(`Junction ${junctions.indexOf(node) + 1}`);
    const info = el('div', 'joint', menu.el);
    const list = (label: string, ducts: ExhaustDuct[]) => {
      if (ducts.length === 0) return;
      const row = el('div', 'joint-row', info);
      el('span', 'joint-label', row).textContent = label;
      el('span', '', row).textContent = ducts.map((d) => ductLabel(graph, d)).join(', ');
    };
    list('In', ends.filter((e) => e.end === 'outlet').map((e) => e.duct));
    list('Out', ends.filter((e) => e.end === 'inlet').map((e) => e.duct));
    const btn = el('button', '', menu.el) as HTMLButtonElement;
    btn.textContent = 'Draw a pipe from here';
    btn.addEventListener('click', () => {
      this.closeMenu();
      this.setDrawMode(true);
      this.cb.onDrawFromJoint(node);
      this.setDrawingState(true);
    });
    this.addButtons('Or carry on out of it with');
  }

  /**
   * A segment's menu: its row, and for the pipe it is in, whether the other cylinders'
   * follow it and, joined at its far end, its length. A segment that ends the pipe in open air, with nothing
   * attached, offers a pipe or a chamber to carry on with.
   */
  private renderSegmentMenu(): void {
    const menu = this.segMenu!;
    const graph = this.config.graph;
    const duct = this.currentDuct();
    const index = this.selected;
    const seg = index === null ? undefined : duct?.segments[index];
    if (!graph || !duct || index === null || !seg) {
      this.closeMenu();
      return;
    }
    const locked = this.lockedFrom();
    const isLocked = locked !== null && index >= locked;
    const open = duct.to.kind === 'mouth' && locked === null && index === duct.segments.length - 1;
    const joined = duct.to.kind === 'node' && locked !== null;
    const runner = duct.from.kind === 'valve' && this.config.engine.cylinders > 1;
    const key = [duct.id, index, duct.segments.length, seg.kind, seg.section ?? '', !!bendShape(seg), isLocked, open, joined, runner].join('|');
    if (key === menu.key && menu.row) {
      this.syncRows([menu.row], [seg]);
      this.syncStats();
      return;
    }
    menu.key = key;
    menu.el.replaceChildren();
    this.menuHead(`${ductLabel(graph, duct)} · segment ${index + 1}`);
    menu.stats = el('div', 'stats', menu.el);
    this.syncStats();
    menu.row = this.buildRow(menu.el, duct.segments, seg, index);
    if (isLocked) lockRow(menu.row);
    // A pipe's own straight or bend, as the bend tool takes them: not a can, nor the bend fitted into what it joins.
    if (!isLocked && seg.kind !== 'chamber') {
      const bend = el('button', '', menu.el) as HTMLButtonElement;
      bend.textContent = seg.curve ? 'Bend it again' : 'Bend this pipe';
      bend.title =
        'The bend tool, on this segment: drag the ring of the plane to bend in, and the straight curves into ' +
        'one arc, keeping its length. Right-click or Esc when done.';
      bend.addEventListener('click', () => {
        this.closeMenu();
        this.setBendToolState(true);
        this.cb.onBendSegment(duct.id, index);
      });
    }
    if (joined) {
      const total = duct.segments.reduce((a, s) => a + s.length, 0);
      numberField(menu.el, 'Pipe length', total * MM, 30, 5000, 1, 'mm', (v) =>
        this.cb.onMatchLength(duct.id, v / MM),
      ).title = 'Fits the pipe to this length: its last straight is lengthened or shortened, or it takes a swing on its way.';
    }
    if (runner) {
      const label = el('label', 'toggle', menu.el) as HTMLLabelElement;
      const box = el('input', '', label) as HTMLInputElement;
      box.type = 'checkbox';
      box.checked = this.linkRunners;
      label.append(' Apply to every cylinder');
      label.title =
        'Keeps every cylinder\u2019s primary identical, as a symmetric engine has them. Turn it off ' +
        'to build unequal-length headers.';
      box.addEventListener('change', () => {
        this.linkRunners = box.checked;
        if (this.linkRunners) this.commit();
      });
    }
    if (open) this.addButtons('Nothing is attached here. Carry on with');
  }

  /**
   * Build segment `index` of `list`'s row, the selected duct's segments. Every edit in it shows in the view,
   * and a structural one rebuilds it.
   */
  private buildRow(container: HTMLElement, list: PipeSegment[], seg: PipeSegment, index: number): SegmentRow {
    const wrap = el('div', 'segment', container);

    const head = el('div', 'segment-head', wrap);
    el('span', 'segment-index', head).textContent = String(index + 1);

    const kind = el('select', 'segment-kind', head) as HTMLSelectElement;
    // Every pipe can taper, so a cone is shown as a pipe here.
    for (const k of ['pipe', 'chamber'] as SegmentKind[]) {
      kind.appendChild(option(k, k));
    }
    kind.value = seg.kind === 'cone' ? 'pipe' : seg.kind;
    kind.addEventListener('change', () => {
      const was = seg.kind;
      seg.kind = kind.value as SegmentKind;
      // A can goes back to a pipe the width of its throats; a pipe becomes a can three times as wide.
      if (seg.kind === 'pipe' && was === 'chamber') seg.dOut = seg.dIn;
      else if (seg.kind === 'chamber' && seg.dOut <= seg.dIn * 1.05) seg.dOut = seg.dIn * 3;
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
    mkTool('×', 'Delete. The segments after it are left as a loose pipe where they lie; a fitted bend takes the pipe off what it joins.', () =>
      this.remove(list, index),
    );

    const grid = el('div', 'segment-grid', wrap);
    const length = numberField(grid, 'Length', seg.length * MM, 10, 2000, 5, 'mm', (v) => {
      seg.length = v / MM;
      this.commit();
      this.syncStats();
    });
    const dIn = numberField(grid, 'Inlet ⌀', seg.dIn * MM, 6, 250, 1, 'mm', (v) => {
      const was = segmentDiameter(seg, 0);
      seg.dIn = v / MM;
      // The segment before ends where this one starts, and where the pipe starts or ends at a junction, the
      // pipes meeting it there follow: a can's throats are both this bore.
      this.matchPrevious(list, index);
      this.carryAtEnds(list, index, 'start', was, seg.dIn);
      if (seg.kind === 'chamber') this.carryAtEnds(list, index, 'end', was, seg.dIn);
      this.commit();
      this.syncPipe();
    });
    const dOutWrap = el('div', 'field', grid);
    const dOut = numberInto(
      dOutWrap,
      bodyLabel(seg),
      seg.dOut * MM,
      6,
      400,
      1,
      'mm',
      (v) => {
        const was = segmentDiameter(seg, 1);
        seg.dOut = v / MM;
        this.propagate(list, index);
        // A can's is its body, not where it ends.
        if (seg.kind !== 'chamber') this.carryAtEnds(list, index, 'end', was, seg.dOut);
        this.commit();
        this.syncPipe();
      },
    );

    const chamber = seg.kind === 'chamber' ? this.chamberFields(wrap, seg) : {};

    const bends = el('div', 'segment-grid', wrap);
    const shape = bendShape(seg);
    const duct = this.currentDuct();
    if (shape && duct) {
      // A bend is its turn and its radius, as a tube bender sets it; its length follows from them.
      length.readOnly = true;
      length.title = 'A bend is as long as its turn and radius make it.';
      let angle = shape.angle;
      let radius = shape.radius;
      numberField(bends, 'Bend', deg(angle), 1, 179, 1, '°', (v) => {
        angle = rad(v);
        this.cb.onReshapeBend(duct.id, index, angle, radius, 'angle');
      });
      numberField(bends, 'Radius', radius * MM, 1.5 * seg.dIn * MM, 2000, 1, 'mm', (v) => {
        radius = Math.max(v / MM, 1.5 * seg.dIn);
        this.cb.onReshapeBend(duct.id, index, angle, radius, 'radius');
      });
      // Between two straights, where along the pipe it is: the straight before it, the one after giving way.
      const prev = list[index - 1];
      const next = list[index + 1];
      const straight = (s: PipeSegment | undefined) => !!s && !s.curve && s.kind !== 'chamber';
      if (straight(prev) && straight(next)) {
        numberField(bends, 'Before', prev!.length * MM, 1, (prev!.length + next!.length) * MM, 1, 'mm', (v) =>
          this.cb.onSlideBend(duct.id, index, v / MM),
        ).title = 'How long the straight before the bend is. The straight after it gives way, so the pipe keeps its length.';
      }
    } else {
      numberField(bends, 'Yaw', deg(seg.yaw), -120, 120, 1, '°', (v) => {
        seg.yaw = rad(v);
        this.commit();
      });
      numberField(bends, 'Pitch', deg(seg.pitch), -120, 120, 1, '°', (v) => {
        seg.pitch = rad(v);
        this.commit();
      });
    }

    return { el: wrap, kind, length, dIn, dOut, dOutWrap, ...chamber };
  }

  /** Shape, height and pipe offsets, for a chamber's row. */
  private chamberFields(
    wrap: HTMLElement,
    seg: PipeSegment,
  ): Pick<SegmentRow, 'height' | 'offsetIn' | 'offsetOut'> {
    const grid = el('div', 'segment-grid', wrap);
    const field = el('div', 'field', grid);
    el('label', '', field).textContent = 'Shape';
    const select = el('select', '', field) as HTMLSelectElement;
    for (const k of CHAMBER_SECTIONS) select.appendChild(option(k, SECTION_LABELS[k]));
    select.value = seg.section ?? 'round';
    select.addEventListener('pointerdown', (e) => e.stopPropagation());
    select.addEventListener('change', () => {
      const next = select.value as ChamberSection;
      if (next === 'round') {
        delete seg.section;
        delete seg.height;
      } else {
        // A round can turned flat keeps its width and starts at half its height, which is a
        // typical oval silencer's proportion.
        if ((seg.section ?? 'round') === 'round') seg.height = seg.dOut / 2;
        seg.section = next;
      }
      this.commit();
      this.rebuildPipeList();
    });

    const heightWrap = el('div', 'field', grid);
    const height = numberInto(heightWrap, 'Height', (seg.height ?? seg.dOut) * MM, 6, 400, 1, 'mm', (v) => {
      seg.height = v / MM;
      this.commit();
      this.syncPipe();
    });
    heightWrap.classList.toggle('hidden', (seg.section ?? 'round') === 'round');

    // Offsets along the width, which is where they move the pipe furthest from the modes' nodes.
    const offsetIn = numberField(grid, 'Inlet offset', (seg.offsetIn ?? 0) * MM, -200, 200, 1, 'mm', (v) => {
      seg.offsetIn = v / MM;
      this.commit();
      this.syncPipe();
    });
    const offsetOut = numberField(grid, 'Outlet offset', (seg.offsetOut ?? 0) * MM, -200, 200, 1, 'mm', (v) => {
      seg.offsetOut = v / MM;
      this.commit();
      this.syncPipe();
    });
    return { height, offsetIn, offsetOut };
  }

  /** Keep the duct continuous after an outlet edit: the next segment starts where this one ends. */
  private propagate(list: PipeSegment[], index: number): void {
    const seg = list[index];
    const next = list[index + 1];
    if (!seg || !next) return;
    next.dIn = segmentDiameter(seg, 1);
  }

  /**
   * Keep the duct continuous after an inlet edit: the segment before ends where this one starts, its
   * outlet for a pipe and its throats for a can.
   */
  private matchPrevious(list: PipeSegment[], index: number): void {
    const seg = list[index];
    const prev = list[index - 1];
    if (!seg || !prev) return;
    if (prev.kind === 'chamber') prev.dIn = seg.dIn;
    else prev.dOut = seg.dIn;
  }

  /**
   * Where segment `index` is the first or last of its pipe, and the pipe starts or ends at a junction, carry
   * its change of bore there to the pipes that met it at the same bore (`carryBore`).
   */
  private carryAtEnds(list: PipeSegment[], index: number, end: 'start' | 'end', was: number, now: number): void {
    const graph = this.config.graph;
    const duct = this.currentDuct();
    if (!graph || !duct || duct.segments !== list) return;
    if (end === 'start' && index === 0) carryBore(graph, duct, 'start', was, now);
    if (end === 'end' && index === list.length - 1) carryBore(graph, duct, 'end', was, now);
  }

  private move(pipe: PipeSegment[], index: number, delta: number): void {
    const to = index + delta;
    const end = this.lockedFrom() ?? pipe.length;
    if (to < 0 || to >= end || index >= end) return;
    const [seg] = pipe.splice(index, 1);
    pipe.splice(to, 0, seg!);
    this.selected = to;
    this.commit();
    this.rebuildPipeList();
    this.cb.onSelect(to);
  }

  private duplicate(list: PipeSegment[], index: number): void {
    const seg = list[index];
    const locked = this.lockedFrom();
    if (!seg || (locked !== null && index >= locked)) return;
    list.splice(index + 1, 0, makeSegment({ ...seg, id: undefined }));
    this.selected = index + 1;
    this.commit();
    this.rebuildPipeList();
    this.cb.onSelect(this.selected);
  }

  private remove(list: PipeSegment[], index: number): void {
    const locked = this.lockedFrom();
    // Deleting a fitted bend takes the pipe off what it joined, ending where it was drawn to; a pipe that
    // is nothing but a bend goes whole.
    if (locked !== null && index >= locked) {
      const duct = this.currentDuct();
      this.selected = null;
      if (duct) this.cb.onDetachDuct(duct.id);
      this.rebuildPipeList();
      this.cb.onSelect(null);
      return;
    }
    // A segment with more after it leaves them as a loose pipe where they lie, which the owner splits off,
    // since it needs the layout. A bend fitted at the pipe's end does not count: it is fitted again from
    // wherever the drawn pipe now ends.
    const graph = this.config.graph;
    const duct = this.currentDuct();
    const drawn = locked ?? list.length;
    if (index < drawn - 1 && graph && duct) {
      this.selected = null;
      this.cb.onSplitDuct(duct.id, index);
      this.rebuildPipeList();
      this.cb.onSelect(null);
      return;
    }
    list.splice(index, 1);
    this.selected = null;
    /**
     * A pipe with nothing left in it goes, unless it is a cylinder's runner.
     *
     * An empty duct is drawn as nothing but still solved as a short stub, so leaving it would keep a pipe
     * the user can no longer see or select. A runner is kept, empty, because a cylinder must have one.
     */
    // A pipe others carry on from, emptied, goes too, leaving them loose; the owner does that, since it
    // needs the layout to leave them where they lie.
    const orphans = list.length === 0 && graph && duct && childDucts(graph, duct).length > 0;
    if (orphans || (list.length === 0 && graph && duct && duct.from.kind !== 'valve')) {
      // The owner removes it, because tidying the junctions needs the layout to keep pipes where they were.
      this.cb.onRemoveDuct(duct.id);
    } else {
      this.commit(list.length === 0);
    }
    this.rebuildPipeList();
    this.cb.onSelect(null);
  }

  /** Say something about the last thing asked for, for a few seconds: why a delete was refused, say. */
  notify(text: string): void {
    this.noticeEl.textContent = text;
    this.noticeEl.classList.remove('hidden');
    clearTimeout(this.noticeTimer);
    this.noticeTimer = window.setTimeout(() => this.noticeEl.classList.add('hidden'), 4000);
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
    this.renderMenu();
  }

  private syncRows(rows: SegmentRow[], list: PipeSegment[]): void {
    list.forEach((seg, i) => {
      const row = rows[i];
      if (!row) return;
      // Never fight the field the user is currently typing in.
      if (document.activeElement !== row.length) row.length.value = round(seg.length * MM, 1);
      if (document.activeElement !== row.dIn) row.dIn.value = round(seg.dIn * MM, 1);
      if (document.activeElement !== row.dOut) row.dOut.value = round(seg.dOut * MM, 1);
      if (row.height && document.activeElement !== row.height) {
        row.height.value = round((seg.height ?? seg.dOut) * MM, 1);
      }
      if (row.offsetIn && document.activeElement !== row.offsetIn) {
        row.offsetIn.value = round((seg.offsetIn ?? 0) * MM, 1);
      }
      if (row.offsetOut && document.activeElement !== row.offsetOut) {
        row.offsetOut.value = round((seg.offsetOut ?? 0) * MM, 1);
      }
      if (!row.el.classList.contains('locked')) row.kind.value = seg.kind === 'cone' ? 'pipe' : seg.kind;
    });
  }

  /** Tell the user whether a route is in progress, since the 3D preview is easy to miss. */
  setDrawingState(active: boolean): void {
    if (!this.drawing) return;
    this.drawingRoute = active;
    this.drawHint.textContent = active ? ROUTE_HINT : START_HINT;
  }

  /** Show the bend tool as on or off, and turn off what it replaces. */
  setBendToolState(on: boolean): void {
    if (on && this.drawing) {
      this.setDrawMode(false);
      this.cb.onDrawMode(false);
    }
    if (on && this.placingPipe) {
      this.setPlacingPipeState(false);
      this.cb.onPlacePipeMode(false);
    }
    if (on) this.stopHeaderTool();
    this.bendingTool = on;
    this.bendToolHint.textContent = on ? BEND_HINT : '';
    this.syncTools();
  }

  /** Show the equal-length header tool as on or off, and turn off what it replaces. */
  setHeaderToolState(on: boolean): void {
    if (on && this.drawing) {
      this.setDrawMode(false);
      this.cb.onDrawMode(false);
    }
    if (on && this.placingPipe) {
      this.setPlacingPipeState(false);
      this.cb.onPlacePipeMode(false);
    }
    if (on && this.placing) {
      this.setPlacingState(false);
      this.cb.onPlaceMode(false);
    }
    if (on && this.bendingTool) {
      this.setBendToolState(false);
      this.cb.onBendTool(false);
    }
    this.headerOn = on;
    this.headerMirrorLabel.classList.toggle('hidden', physicalBankCount(this.config.engine) < 2);
    this.headerHint.textContent = '';
    this.syncTools();
  }

  /** Turn the header tool off, as starting another tool does. */
  private stopHeaderTool(): void {
    if (!this.headerOn) return;
    this.setHeaderToolState(false);
    this.cb.onHeaderTool(false);
  }

  /** Whether the header is to be mirrored onto the other bank. */
  get headerMirrored(): boolean {
    return this.headerMirror;
  }

  /** Show the primary length the header tool is building, m. */
  setHeaderLength(length: number): void {
    this.headerLengthInput.value = round(length * MM, 1);
  }

  /** What the header being placed comes to, from the view. */
  setHeaderAim(aim: string): void {
    if (this.headerOn) this.headerHint.textContent = aim;
  }

  /** What the bend tool is doing, from the view. */
  setBendAim(aim: string | null): void {
    if (!this.bendingTool) return;
    this.bendToolHint.textContent = aim ?? BEND_HINT;
  }

  /** Say which way the next segment is aimed, since a direction is hard to judge in perspective. */
  setDrawAim(aim: string | null): void {
    if (!this.drawing || !this.drawingRoute) return;
    this.drawHint.textContent = aim ? `Next segment: ${aim}` : ROUTE_HINT;
  }

  /** Show whether loose pipes are being placed: from the button, or ended in the view by Escape. */
  setPlacingPipeState(on: boolean): void {
    if (on && this.drawing) {
      this.setDrawMode(false);
      this.cb.onDrawMode(false);
    }
    if (on && this.placing) this.setPlacingState(false);
    if (on) this.stopHeaderTool();
    this.placingPipe = on;
    this.placePipeHint.textContent = on ? 'Click where the pipe should start' : '';
    this.syncTools();
  }

  /** Show whether turbos are being placed: from the button, or ended in the view by Escape. */
  setPlacingState(on: boolean): void {
    if (on && this.placingPipe) this.setPlacingPipeState(false);
    if (on && this.drawing) {
      this.setDrawMode(false);
      this.cb.onDrawMode(false);
    }
    if (on) this.stopHeaderTool();
    this.placing = on;
    this.placeHint.textContent = on ? PLACE_HINT : '';
    this.syncTools();
  }

  /** Show whether the turbos are in sync, and the selected turbo's menu as the graph now has it. */
  syncTurbos(): void {
    const graph = this.config.graph;
    checkbox(this.turboSyncRow).checked = !graph || turbosSynced(graph);
    if (this.turboId !== null) this.renderMenu();
  }

  private openTurboOptions: (open: boolean) => void = () => {};

  private onTurboOptionsOutside = (e: PointerEvent): void => {
    if (!this.turboTool.contains(e.target as Node)) this.openTurboOptions(false);
  };

  /** Select the turbo selected in the view, or none: a right-click opens its menu. */
  showTurbo(id: string | null): void {
    this.turboId = id;
    this.renderMenu();
  }

  /**
   * A turbo's menu: how it is running, what feeds it, and its settings. In sync, they are every turbo's, the
   * engine's; out of it, its own.
   */
  private renderTurboMenu(id: string): void {
    const menu = this.segMenu!;
    const graph = this.config.graph;
    const turbos = graph?.turbos ?? [];
    const index = turbos.findIndex((t) => t.id === id);
    if (!graph || index < 0) {
      this.closeMenu();
      return;
    }
    const mount = turbos[index]!;
    const synced = turbosSynced(graph);
    const ends = endsAt(graph, mount.node);
    const key = ['turbo', id, synced, turbos.length, ...ends.map((e) => `${e.duct.id}:${e.end}`)].join('|');
    if (key === menu.key) return;
    menu.key = key;
    menu.row = null;
    menu.stats = null;
    menu.el.replaceChildren();
    this.menuHead(`Turbo ${index + 1}`);
    // One row for each reading, always there and right-aligned, so the menu holds still as they change.
    const readings = el('div', 'readout turbo-readings', menu.el);
    const row = (label: string) => {
      el('span', '', readings).textContent = label;
      const value = el('span', 'turbo-reading', readings);
      value.textContent = '—';
      return value;
    };
    this.turboMenuReadout = {
      boost: row('Boost'),
      shaft: row('Shaft'),
      exhaust: row('Exhaust at its inlet'),
      wastegate: row('Wastegate'),
      blowOff: row('Blow-off valve'),
    };
    const info = el('div', 'joint', menu.el);
    const list = (label: string, ducts: ExhaustDuct[]) => {
      const row = el('div', 'joint-row', info);
      el('span', 'joint-label', row).textContent = label;
      el('span', '', row).textContent = ducts.length > 0 ? ducts.map((d) => ductLabel(graph, d)).join(', ') : 'nothing yet';
    };
    list('Fed by', ends.filter((e) => e.end === 'outlet').map((e) => e.duct));
    list('Outlet', ends.filter((e) => e.end === 'inlet').map((e) => e.duct));
    if (synced && turbos.length > 1) {
      el('div', 'hint', menu.el).textContent =
        'In sync: these are every turbo’s settings. Turn sync off in the turbo tool’s menu to set each on its own.';
    }

    const current = () => turboSettingsOf(this.config.graph?.turbos?.find((t) => t.id === id), this.config.engine);
    const set = (patch: Partial<TurboSettings>) =>
      synced ? this.cb.onEngine(patch) : this.cb.onTurboSettings(id, { ...current(), ...patch });
    const now = current();
    slider(menu.el, { ...BOOST_SLIDER, value: now.boostTarget / 1e5, onInput: (v) => set({ boostTarget: v * 1e5 }) }).row.title =
      BOOST_TITLE;
    slider(menu.el, { ...SIZE_SLIDER, value: now.turboSize, onInput: (v) => set({ turboSize: v }) }).row.title = SIZE_TITLE;
    slider(menu.el, {
      label: 'Intercooler',
      min: 0,
      max: 1,
      step: 0.01,
      value: now.intercooler,
      format: (v) => (v > 0 ? `${Math.round(v * 100)}%` : 'none'),
      onInput: (v) => set({ intercooler: v }),
    }).row.title =
      'How much of the heat of compression it takes back out of the air the compressor delivers. Hot air ' +
      'is thin, so without one the same boost makes less torque.';
    const bovRow = el('div', 'row', menu.el);
    el('label', '', bovRow).textContent = 'Blow-off valve';
    const bovSel = el('select', '', bovRow) as HTMLSelectElement;
    const bovNames: Record<BlowOff, string> = {
      atmospheric: 'Atmospheric',
      recirculating: 'Recirculating',
      none: 'None (surges)',
    };
    for (const b of BLOW_OFFS) bovSel.appendChild(option(b, bovNames[b]));
    bovSel.value = now.blowOff;
    bovSel.addEventListener('change', () => set({ blowOff: bovSel.value as BlowOff }));
    bovRow.title =
      'When the throttle shuts on boost, the air between the compressor and the throttle has nowhere ' +
      'to go. An atmospheric valve vents it to the air, with the hiss; a recirculating one back into ' +
      'the compressor inlet, quietly. With none, the air pushes back through the compressor, which ' +
      'stalls and recovers over and over: the flutter.';

    const btn = el('button', '', menu.el) as HTMLButtonElement;
    btn.textContent = 'Take this turbo out';
    btn.addEventListener('click', () => {
      this.closeMenu();
      this.cb.onRemoveTurbo(mount.id);
    });
  }

  private setDrawMode(on: boolean): void {
    if (on && this.placing) {
      this.setPlacingState(false);
      this.cb.onPlaceMode(false);
    }
    if (on && this.placingPipe) {
      this.setPlacingPipeState(false);
      this.cb.onPlacePipeMode(false);
    }
    if (on) this.stopHeaderTool();
    this.drawing = on;
    this.drawingRoute = false;
    this.drawHint.textContent = on ? START_HINT : '';
    this.syncTools();
  }

  /** Show every tool as off, without telling anyone: for when the view ended the one that was on. */
  toolsEnded(): void {
    this.setDrawMode(false);
    this.setPlacingState(false);
    this.setPlacingPipeState(false);
    this.setBendToolState(false);
    this.setHeaderToolState(false);
  }

  /** Light the button of the tool that is on, and show its part of the options card, or no card. */
  private syncTools(): void {
    const tools: Array<[boolean, HTMLButtonElement, HTMLElement]> = [
      [this.drawing, this.drawBtn, this.drawGroup],
      [this.placingPipe, this.placePipeBtn, this.placePipeGroup],
      [this.bendingTool, this.bendToolBtn, this.bendToolGroup],
      [this.headerOn, this.headerBtn, this.headerGroup],
      [this.placing, this.placeBtn, this.placeGroup],
    ];
    for (const [on, button, group] of tools) {
      button.classList.toggle('active', on);
      button.setAttribute('aria-pressed', String(on));
      group.classList.toggle('hidden', !on);
    }
    this.toolOptions.classList.toggle('hidden', !tools.some(([on]) => on));
  }

  /** Switch the menu to a duct picked in the scene. */
  showDuct(id: string): void {
    this.selectedDuctId = id;
    this.selected = null;
    this.rebuildPipeList();
  }

  /** Select the junction selected in the scene, or none: the menu shows it, and adding a segment adds there. */
  showJoint(node: string | null): void {
    this.jointNode = node;
    this.renderMenu();
  }

  setSelected(index: number | null): void {
    this.selected = index;
    this.renderMenu();
  }

  private syncStats(): void {
    const eng = this.config.engine;
    const graph = this.config.graph;
    /**
     * The tuned length a cylinder sees: everything between its valve and open air.
     *
     * Walked along the graph rather than added up as "primary plus collector", which would only
     * describe layouts with exactly those two parts. A tri-Y has three.
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
    const stats = this.segMenu?.stats;
    if (!stats) return;
    stats.textContent =
      `${segs} segment${segs === 1 ? '' : 's'} · ` +
      `${(len * MM).toFixed(0)} mm · ` +
      `1st peak ≈ ${f1.toFixed(0)} Hz · tuned near ${tunedRpm.toFixed(0)} rpm${layoutNote}`;
  }

  // -------------------------------------------------------------------------
  // Readouts
  // -------------------------------------------------------------------------

  updateReadouts(s: EngineSnapshot): void {
    this.shortRunnersNow = s.shortRunners;
    this.refreshIntake();
    const cams =
      Math.abs(s.intakeCamAdvance) < 0.5 && Math.abs(s.exhaustCamRetard) < 0.5
        ? 'Cams at rest'
        : `Intake cam ${Math.round(s.intakeCamAdvance)}° advanced · exhaust ${Math.round(Math.abs(s.exhaustCamRetard))}° ` +
          (s.exhaustCamRetard < 0 ? 'advanced, with it' : 'retarded');
    const lobes =
      this.config.engine.camSwitchRpm > 0
        ? s.highCam
          ? 'On the high-speed lobes'
          : 'On the low-speed lobes'
        : 'One cam profile';
    if (lobes !== this.lobeText) {
      this.lobeText = lobes;
      this.lobeReadout.textContent = lobes;
    }
    if (cams !== this.camText) {
      this.camText = cams;
      this.camReadout.textContent = cams;
    }
    // The open turbo menu's: the boost they all make, and this turbo's own speed and valves.
    const readout = this.turboMenuReadout;
    if (readout?.boost.isConnected) {
      const t = s.turbo;
      const u = t?.turbos.find((x) => x.id === this.turboId);
      // Gauge pressures always signed, so a vacuum takes no more room than boost.
      const bar = (pa: number) => `${pa < 0 ? '\u2212' : '+'}${(Math.abs(pa) / 1e5).toFixed(2)} bar`;
      const values: Record<TurboReading, string> =
        t && u
          ? {
              boost: bar(t.boost),
              shaft: `${formatRpm(u.shaftRpm)} rpm`,
              exhaust: bar(t.turbineInlet),
              wastegate: u.wastegate > 0.02 ? `${Math.round(u.wastegate * 100)}% open` : 'shut',
              blowOff: t.surging ? 'surging' : u.blowOff > 0.05 && t.boost > 0.05e5 ? 'venting' : 'shut',
            }
          : { boost: '—', shaft: '—', exhaust: '—', wastegate: '—', blowOff: '—' };
      for (const k of Object.keys(values) as TurboReading[]) {
        if (readout[k].textContent !== values[k]) readout[k].textContent = values[k];
      }
    }
    const running = s.launch !== null;
    if (running !== this.launchRunning) {
      this.launchRunning = running;
      const launching = running && this.runKind === 'launch';
      const pulling = running && this.runKind === 'dyno';
      this.launchBtn.textContent = launching ? 'Stop launch' : 'Start launch';
      this.launchBtn.classList.toggle('running', launching);
      this.launchBtn.disabled = pulling;
      this.dynoBtn.textContent = pulling ? 'Stop dyno pull' : 'Start dyno pull';
      this.dynoBtn.classList.toggle('running', pulling);
      this.dynoBtn.disabled = launching;
    }
    this.rpmEl.textContent = `${Math.round(s.rpm)} rpm${s.limiter ? ' · limiter' : ''}`;
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

/**
 * Show a segment row as not editable: a bend, or the swing before it, fitted to both its ends. Its fields still show
 * its length and bore, kept up to date as it is fitted again, but take no input.
 */
function lockRow(row: SegmentRow): void {
  row.el.classList.add('locked');
  for (const input of row.el.querySelectorAll('input, select, button')) {
    (input as HTMLInputElement).disabled = true;
  }
  row.el.querySelector('.segment-tools')?.classList.add('hidden');
  row.kind.replaceChildren(option('bend', 'bend, fitted'));
  row.el.title =
    'Fitted to both its ends, the pipe before it and what it joins, so it is not edited: change the pipe ' +
    'before it, or move what it joins, and it follows.';
}

/**
 * Lays the panel out as a rail of section icons down its edge beside the section on show, and
 * returns what adds a section: its title, its icon and its tip. The first section added is on show.
 *
 * One section shows at a time. Clicking the open section's icon folds the panel down to the rail,
 * giving its width back to the view.
 */
function sectionRail(root: HTMLElement): (title: string, icon: keyof typeof SECTION_ICONS, tip: string) => HTMLElement {
  const pages = el('div', 'panel-pages', root);
  const rail = el('nav', 'panel-rail', root);
  rail.setAttribute('aria-label', 'Panel sections');
  let open: { page: HTMLElement; button: HTMLButtonElement } | null = null;
  const show = (next: typeof open) => {
    if (open) {
      open.page.classList.add('hidden');
      open.button.classList.remove('active');
    }
    open = next === open ? null : next;
    if (open) {
      open.page.classList.remove('hidden');
      open.button.classList.add('active');
      pages.scrollTop = 0;
    }
    root.classList.toggle('folded', !open);
  };
  return (title, icon, tip) => {
    const page = el('section', 'section hidden', pages);
    el('h2', 'section-title', page).textContent = title;
    const body = el('div', 'section-body', page);
    const button = toolButton(rail, SECTION_ICONS[icon], title, tip);
    const entry = { page, button };
    button.addEventListener('click', () => show(entry));
    if (!open) show(entry);
    return body;
  };
}

/** Re-reads one control from the config; collected so a preset can refresh them all. */
type Resync = () => void;

/** What a turbo's menu reads out while the engine runs. */
type TurboReading = 'boost' | 'shaft' | 'exhaust' | 'wastegate' | 'blowOff';

/** The boost and turbo size sliders' ranges, for the engine's and a turbo's own. */
const BOOST_SLIDER = { label: 'Boost', min: 0.1, max: 2, step: 0.05, format: (v: number) => `${v.toFixed(2)} bar` };
const SIZE_SLIDER = {
  label: 'Turbo size',
  min: 0,
  max: 0.5,
  step: 0.005,
  format: (v: number) => (v > 0 ? `${v.toFixed(3)} kg/s` : 'auto'),
};
const BOOST_TITLE =
  'What the wastegate holds: as the boost reaches it, it opens a bypass around the turbine so the ' +
  'turbine takes less of the exhaust. There is no knock here, so nothing stops you asking for more ' +
  'than the engine would survive.';
const SIZE_TITLE =
  'Each compressor’s airflow at full speed. Small spools early and runs out of breath at the ' +
  'top, nearing its choke with the exhaust backed up behind the turbine; big lags and holds its ' +
  'boost to the limiter. At 0 it is sized for the engine’s airflow near its rev limit.';

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
  /**
   * Where the value lives in the config, for a slider bound to it: read back whenever the config
   * changes underneath the panel, as loading a preset does.
   */
  sync?: () => number;
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
    // Handled first, so a label that reads the state it sets is up to date.
    o.onInput(v);
    render(v);
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
  /** Decimal places the field shows. */
  dp = 1,
): HTMLInputElement {
  el('label', '', field).textContent = label;
  const wrap = el('div', 'number-wrap', field);
  const input = el('input', '', wrap) as HTMLInputElement;
  input.type = 'number';
  input.min = String(min);
  input.max = String(max);
  input.step = String(step);
  input.value = round(value, dp);
  el('span', 'unit', wrap).textContent = unit;

  /**
   * Applied on `change` — Enter, leaving the field, or the spinner — not on every keystroke.
   *
   * Committing as you type would commit every prefix of what you are typing: on the way to a 50 mm
   * inlet the field holds "5", which the 6 mm minimum would clamp and write straight back as "6". And a
   * prefix that is in range is still wrong: typing a 1200 mm length would briefly make it 12 mm, short
   * enough to pull the pipe off its junction.
   */
  const commit = () => {
    const v = Number(input.value);
    if (input.value.trim() === '' || !Number.isFinite(v)) return;
    const clamped = Math.max(min, Math.min(max, v));
    if (clamped !== v) input.value = round(clamped, dp);
    onChange(clamped);
  };
  input.addEventListener('change', commit);
  input.addEventListener('pointerdown', (e) => e.stopPropagation());
  return input;
}

/** The checkbox in a `toggle` row. */
function checkbox(row: HTMLElement): HTMLInputElement {
  return row.querySelector('input')!;
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

function ordinal(n: number): string {
  return `${n}${n === 1 ? 'st' : n === 2 ? 'nd' : n === 3 ? 'rd' : 'th'}`;
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

/** An engine speed rounded to the nearest 50, with a thousands separator. */
function formatRpm(rpm: number): string {
  return (Math.round(rpm / 50) * 50).toLocaleString('en-US');
}
