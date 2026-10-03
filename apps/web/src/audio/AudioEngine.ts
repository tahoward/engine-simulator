/**
 * Main-thread side of the audio: owns the AudioContext, loads the worklet, forwards
 * parameter and geometry changes, and re-publishes snapshots to the renderer.
 *
 * Deliberately has no opinion about three.js or the DOM.
 */

import { solverGraph, type ExhaustGraph } from '../model/exhaustGraph.js';
import type { LaunchConfig, EngineConfig, EngineSpec, SoundSources } from '../model/spec.js';
import { CONTROL_PARAMS } from './worklet/controls.js';
import { solvedPlenum } from '../scene/inletLayout.js';
import type { FromWorklet, ToWorklet } from './worklet/processor.js';

// `?worker&url` bundles the processor and everything it imports into one ES module
// chunk and yields its URL. Plain `?url` would ship unbundled TypeScript with bare
// import specifiers, which AudioWorkletGlobalScope cannot resolve.
import processorUrl from './worklet/processor.ts?worker&url';

import type { EngineHost, LagListener, SnapshotListener } from './EngineHost.js';

/** How often the audio clock is checked against the wall clock, ms. */
const LAG_WINDOW_MS = 2000;
/** Share of real time the audio must render over a window to count as keeping up. */
const LAG_RATIO = 0.98;
/** Windows in a row that must be late before the lag is reported, and on time before it is cleared. */
const LAG_WINDOWS_BAD = 2;
const LAG_WINDOWS_GOOD = 3;

export class AudioEngine implements EngineHost {
  private ctx: AudioContext | null = null;
  private node: AudioWorkletNode | null = null;
  private analyser: AnalyserNode | null = null;
  private readonly listeners = new Set<SnapshotListener>();
  private readonly lagListeners = new Set<LagListener>();
  private lagTimer: ReturnType<typeof setInterval> | null = null;
  /** The audio and wall clocks at the start of the current window, s and ms; `null` to start afresh. */
  private lagMark: { audio: number; wall: number } | null = null;
  /** Windows in a row on the wrong side of `behind`. */
  private lagStreak = 0;
  private behind = false;
  private config: EngineConfig;
  private starting: Promise<void> | null = null;
  private timeScale = 1;
  private ignition = true;

  /**
   * @param rate Audio sample rate, Hz. The solver takes one step per sample, so this also sets the
   *   finest cell it can use and so the cost: 32 kHz needs about 63% of the CPU 48 kHz does.
   * @param latency How much output buffering to ask for. `'playback'` trades delay for tolerance of a
   *   late block, which on a slow device is the difference between a clean note and crackle.
   */
  constructor(
    config: EngineConfig,
    private rate = 48000,
    private readonly latency: AudioContextLatencyCategory = 'interactive',
  ) {
    this.config = {
      engine: { ...config.engine },
      pipe: config.pipe.map((s) => ({ ...s })),
      collector: config.collector.map((s) => ({ ...s })),
      // Without it the worklet compiles its own exhaust from the layout at start, and a drawn one
      // restored from the URL would be on screen but not in the sound until the next edit.
      ...(config.graph ? { graph: structuredClone(solverGraph(config.graph)) } : {}),
      ...(config.sources ? { sources: structuredClone(config.sources) } : {}),
      ...(config.listener ? { listener: [...config.listener] } : {}),
    };
  }

  get running(): boolean {
    return this.ctx?.state === 'running' && this.node !== null;
  }

  get sampleRate(): number {
    return this.ctx?.sampleRate ?? this.rate;
  }

  /**
   * Switch the audio sample rate.
   *
   * A context's rate is fixed when it is made, so a live one is closed and a new one booted in its
   * place: the simulation restarts from cold, with the pipe walls back at their starting temperature.
   */
  async setSampleRate(hz: number): Promise<void> {
    if (hz === this.rate) return;
    this.rate = hz;
    if (!this.ctx && !this.starting) return;
    // Let a boot in flight finish, so there is one whole context to close rather than half of one.
    await this.starting;
    // A change made while the first was still closing: the reboot below picks up the newest rate.
    if (!this.ctx) return;
    const wasRunning = this.running;
    const ctx = this.ctx;
    this.stopLagWatch();
    this.node?.disconnect();
    this.ctx = null;
    this.node = null;
    this.analyser = null;
    this.starting = null;
    await ctx.close();
    if (wasRunning) await this.start();
  }

  /** Audio can only begin from a user gesture, so this is called from a click. */
  async start(): Promise<void> {
    if (this.node) {
      await this.ctx?.resume();
      this.lagMark = null;
      return;
    }
    // Guard against double-clicks racing two contexts into existence.
    this.starting ??= this.boot();
    await this.starting;
  }

  private async boot(): Promise<void> {
    // The browser resamples to the device's own rate, so any rate plays; only the simulation's cost changes.
    const ctx = new AudioContext({ latencyHint: this.latency, sampleRate: this.rate });
    this.ctx = ctx;
    await ctx.audioWorklet.addModule(processorUrl);

    const eng = this.config.engine;
    const node = new AudioWorkletNode(ctx, 'engine-processor', {
      numberOfInputs: 0,
      numberOfOutputs: 1,
      outputChannelCount: [1],
      processorOptions: this.startConfig(),
      // Without these the parameters start at their default of 0, and the first block would drop the
      // engine to zero throttle and zero load.
      parameterData: { throttle: eng.throttle, load: eng.load },
    });
    node.port.onmessage = (e: MessageEvent<FromWorklet>) => {
      if (e.data.type === 'snapshot') {
        for (const l of this.listeners) l(e.data.snapshot);
      } else if (e.data.type === 'error') {
        console.error(`[AudioEngine] the simulation failed: ${e.data.message}`);
      }
    };
    node.onprocessorerror = () => {
      // Surfacing this matters: a thrown error inside the worklet silently kills
      // the audio thread, and without a message the app just goes quiet.
      console.error('[AudioEngine] the audio worklet stopped; reload to restart it');
    };

    const analyser = ctx.createAnalyser();
    analyser.fftSize = 2048;
    analyser.smoothingTimeConstant = 0.6;

    const master = ctx.createGain();

    node.connect(analyser);
    analyser.connect(master);
    master.connect(ctx.destination);

    this.node = node;
    if (this.timeScale !== 1) this.post({ type: 'timeScale', scale: this.timeScale });
    // A new simulation starts with the ignition on.
    if (!this.ignition) this.post({ type: 'ignition', on: false });
    this.analyser = analyser;

    await ctx.resume();
    this.startLagWatch();
  }

  async suspend(): Promise<void> {
    await this.ctx?.suspend();
    // Otherwise the pause would count against the window it fell in.
    this.lagMark = null;
  }

  onLag(listener: LagListener): () => void {
    this.lagListeners.add(listener);
    return () => this.lagListeners.delete(listener);
  }

  /**
   * Watch whether the audio thread keeps up with real time.
   *
   * The worklet cannot time itself (see `processor.ts`), but the context's clock only advances by the
   * blocks actually rendered, so when `process` takes longer than a block lasts it falls behind the
   * wall clock, and the browser fills the gap with silence: crackle. Measured over whole windows, since
   * the clock moves in steps of the device's buffer.
   */
  private startLagWatch(): void {
    this.stopLagWatch();
    this.lagTimer = setInterval(() => this.checkLag(), LAG_WINDOW_MS);
  }

  private stopLagWatch(): void {
    if (this.lagTimer !== null) clearInterval(this.lagTimer);
    this.lagTimer = null;
    this.lagMark = null;
    this.lagStreak = 0;
    this.setBehind(false);
  }

  private checkLag(): void {
    const ctx = this.ctx;
    // A suspended context is meant to stand still; start the next window from wherever it resumes.
    if (!ctx || ctx.state !== 'running') {
      this.lagMark = null;
      return;
    }
    // The output timestamp pairs the two clocks at one instant, which `currentTime` read beside
    // `performance.now()` does not. Not every browser has it.
    const stamp = ctx.getOutputTimestamp?.();
    const now =
      stamp?.contextTime !== undefined && stamp.performanceTime !== undefined
        ? { audio: stamp.contextTime, wall: stamp.performanceTime }
        : { audio: ctx.currentTime, wall: performance.now() };
    const mark = this.lagMark;
    this.lagMark = now;
    if (!mark || now.wall <= mark.wall) return;
    const late = (now.audio - mark.audio) / ((now.wall - mark.wall) / 1000) < LAG_RATIO;
    if (late === this.behind) {
      this.lagStreak = 0;
      return;
    }
    if (++this.lagStreak >= (late ? LAG_WINDOWS_BAD : LAG_WINDOWS_GOOD)) {
      this.lagStreak = 0;
      this.setBehind(late);
    }
  }

  private setBehind(behind: boolean): void {
    if (behind === this.behind) return;
    this.behind = behind;
    for (const l of this.lagListeners) l(behind);
  }

  async toggle(): Promise<boolean> {
    if (this.running) {
      await this.suspend();
      return false;
    }
    await this.start();
    return true;
  }

  onSnapshot(listener: SnapshotListener): () => void {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  /** The plenum the simulation is given last: as it is drawn (`solvedPlenum`). */
  private plenumSent: ReturnType<typeof solvedPlenum> | null = null;

  /**
   * `partial`, with the plenum's size as drawn wherever that has moved since it was last given: the
   * simulation solves the plenum the app shows, whose size follows much of the engine where it is left to
   * work itself out.
   */
  private withPlenum(partial: Partial<EngineSpec>): Partial<EngineSpec> {
    // The controls alone move nothing it is sized by.
    if (Object.keys(partial).every((k) => (CONTROL_PARAMS as readonly string[]).includes(k))) return partial;
    const solved = solvedPlenum(this.config.engine);
    const was = this.plenumSent;
    this.plenumSent = solved;
    // And any the change itself sets, which the drawing may take otherwise: dual plenums asked for of an
    // engine that cannot have them, or a size left to work itself out.
    const moved = (Object.keys(solved) as Array<keyof typeof solved>).filter(
      (k) => !was || was[k] !== solved[k] || k in partial,
    );
    if (moved.length === 0) return partial;
    return { ...partial, ...Object.fromEntries(moved.map((k) => [k, solved[k]])) };
  }

  /** The config to start the simulation from: the plenum as drawn. */
  private startConfig(): EngineConfig {
    this.plenumSent = solvedPlenum(this.config.engine);
    return { ...this.config, engine: { ...this.config.engine, ...this.plenumSent } };
  }

  /** Update engine parameters. Safe to call before the context exists. */
  setEngine(partial: Partial<EngineSpec>): void {
    Object.assign(this.config.engine, partial);
    partial = this.withPlenum(partial);
    // The continuous controls go as parameters, which an overloaded audio thread still reads; see
    // `CONTROL_PARAMS`. Everything else, and only if there is anything else, as a message.
    let rest: Partial<EngineSpec> | null = null;
    for (const key of Object.keys(partial) as Array<keyof EngineSpec>) {
      if ((CONTROL_PARAMS as readonly string[]).includes(key)) {
        const param = this.node?.parameters.get(key);
        if (param) param.value = partial[key] as number;
      } else {
        (rest ??= {} as Record<string, unknown>)[key] = partial[key];
      }
    }
    if (rest) this.post({ type: 'engine', engine: rest });
  }

  /** Replace the whole duct graph; `null` hands the solver back to compiling one from the layout spec. */
  setGraph(graph: ExhaustGraph | null): void {
    // Loose pipes carry no gas, so the solver is not given them.
    const solved = graph ? solverGraph(graph) : null;
    this.config.graph = solved ?? undefined;
    this.post({ type: 'graph', graph: solved });
  }

  setSources(sources: SoundSources): void {
    this.config.sources = sources;
    this.post({ type: 'sources', sources });
  }

  setListener(position: [number, number, number] | null): void {
    if (position) this.config.listener = position;
    else delete this.config.listener;
    this.post({ type: 'listener', position });
  }

  /** Start a launch through `config`, or with `null` end the one in progress. */
  launch(config: LaunchConfig | null): void {
    this.post({ type: 'launch', config });
  }

  setTimeScale(scale: number): void {
    this.timeScale = scale;
    this.post({ type: 'timeScale', scale });
  }

  setIgnition(on: boolean): void {
    this.ignition = on;
    this.post({ type: 'ignition', on });
  }

  private post(msg: ToWorklet): void {
    this.node?.port.postMessage(msg);
  }

  /** Fills `out` with the current time-domain waveform. Returns false while the audio is not running. */
  readWaveform(out: Float32Array<ArrayBuffer>): boolean {
    // A suspended analyser holds its last frame, which is not what a stopped engine sounds like.
    if (!this.analyser || !this.running) return false;
    this.analyser.getFloatTimeDomainData(out);
    return true;
  }

  /** Fills `out` with the current magnitude spectrum in dB. Returns false while the audio is not running. */
  readSpectrum(out: Float32Array<ArrayBuffer>): boolean {
    if (!this.analyser || !this.running) return false;
    this.analyser.getFloatFrequencyData(out);
    return true;
  }

  get spectrumSize(): number {
    return this.analyser?.frequencyBinCount ?? 1024;
  }
}
