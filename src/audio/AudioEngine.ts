/**
 * Main-thread side of the audio: owns the AudioContext, loads the worklet, forwards
 * parameter and geometry changes, and re-publishes snapshots to the renderer.
 *
 * Deliberately has no opinion about three.js or the DOM.
 */

import type { ExhaustGraph } from '../model/exhaustGraph.js';
import type { EngineConfig, EngineSnapshot, EngineSpec, PipeSegment } from '../model/spec.js';
import type { FromWorklet, ToWorklet } from './worklet/processor.js';

// `?worker&url` bundles the processor and everything it imports into one ES module
// chunk and yields its URL. Plain `?url` would ship unbundled TypeScript with bare
// import specifiers, which AudioWorkletGlobalScope cannot resolve.
import processorUrl from './worklet/processor.ts?worker&url';

export type SnapshotListener = (snapshot: EngineSnapshot) => void;

export class AudioEngine {
  private ctx: AudioContext | null = null;
  private node: AudioWorkletNode | null = null;
  private master: GainNode | null = null;
  private analyser: AnalyserNode | null = null;
  private readonly listeners = new Set<SnapshotListener>();
  private config: EngineConfig;
  private starting: Promise<void> | null = null;

  constructor(config: EngineConfig) {
    this.config = {
      engine: { ...config.engine },
      pipe: config.pipe.map((s) => ({ ...s })),
      collector: config.collector.map((s) => ({ ...s })),
      // Without it the worklet compiles its own exhaust from the layout at start, and a drawn one
      // restored from the URL would be on screen but not in the sound until the next edit.
      ...(config.graph ? { graph: structuredClone(config.graph) } : {}),
    };
  }

  get running(): boolean {
    return this.ctx?.state === 'running' && this.node !== null;
  }

  get sampleRate(): number {
    return this.ctx?.sampleRate ?? 48000;
  }

  /** Audio can only begin from a user gesture, so this is called from a click. */
  async start(): Promise<void> {
    if (this.node) {
      await this.ctx?.resume();
      return;
    }
    // Guard against double-clicks racing two contexts into existence.
    this.starting ??= this.boot();
    await this.starting;
  }

  private async boot(): Promise<void> {
    const ctx = new AudioContext({ latencyHint: 'interactive' });
    this.ctx = ctx;
    await ctx.audioWorklet.addModule(processorUrl);

    const node = new AudioWorkletNode(ctx, 'engine-processor', {
      numberOfInputs: 0,
      numberOfOutputs: 1,
      outputChannelCount: [1],
      processorOptions: this.config,
    });
    node.port.onmessage = (e: MessageEvent<FromWorklet>) => {
      if (e.data.type === 'snapshot') {
        for (const l of this.listeners) l(e.data.snapshot);
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
    master.gain.value = 1;

    node.connect(analyser);
    analyser.connect(master);
    master.connect(ctx.destination);

    this.node = node;
    this.master = master;
    this.analyser = analyser;

    await ctx.resume();
  }

  async suspend(): Promise<void> {
    await this.ctx?.suspend();
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

  /** Update engine parameters. Safe to call before the context exists. */
  setEngine(partial: Partial<EngineSpec>): void {
    Object.assign(this.config.engine, partial);
    this.post({ type: 'engine', engine: partial });
  }

  /**
   * Replace the whole duct graph, for an exhaust that was drawn rather than chosen.
   *
   * `null` hands the solver back to compiling one from the layout spec, which is what a change of
   * cylinder count or merge plan needs.
   */
  setGraph(graph: ExhaustGraph | null): void {
    this.config.graph = graph ?? undefined;
    this.post({ type: 'graph', graph });
  }

  /** Replace the exhaust geometry. Safe to call before the context exists. */
  setPipe(pipe: PipeSegment[], collector?: PipeSegment[]): void {
    this.config.pipe = pipe.map((s) => ({ ...s }));
    if (collector) this.config.collector = collector.map((s) => ({ ...s }));
    this.post({ type: 'pipe', pipe: this.config.pipe, collector: this.config.collector });
  }

  private post(msg: ToWorklet): void {
    this.node?.port.postMessage(msg);
  }

  /** Master output level, linear. Independent of the physical `outputGain`. */
  setMasterGain(value: number): void {
    if (this.master) this.master.gain.value = Math.max(0, value);
  }

  /** Fills `out` with the current time-domain waveform. Returns false if audio was never started. */
  readWaveform(out: Float32Array<ArrayBuffer>): boolean {
    if (!this.analyser) return false;
    this.analyser.getFloatTimeDomainData(out);
    return true;
  }

  /** Fills `out` with the current magnitude spectrum in dB. Returns false if audio was never started. */
  readSpectrum(out: Float32Array<ArrayBuffer>): boolean {
    if (!this.analyser) return false;
    this.analyser.getFloatFrequencyData(out);
    return true;
  }

  get spectrumSize(): number {
    return this.analyser?.frequencyBinCount ?? 1024;
  }
}
