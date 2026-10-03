/**
 * The desktop app's side of the audio: the simulation runs natively in the app's Rust process
 * (`apps/desktop/src-tauri`), and this drives it through Tauri commands.
 *
 * Everything the worklet would be sent goes as a command. What comes back arrives on a channel of
 * packed frames (`audio::Frame` in the Rust): a snapshot with the latest output samples at the
 * snapshot rate, and a report whenever the simulation falls behind real time or catches up.
 *
 * The scope's waveform is those samples, and its spectrum is worked out from them here the way an
 * AnalyserNode does it: a Blackman window, the magnitude over the FFT size, smoothed over time with
 * a constant of 0.6, in dB. There is no AudioContext in the desktop app to ask.
 */

import { Channel, invoke } from '@tauri-apps/api/core';

import { solverGraph, type ExhaustGraph } from '../model/exhaustGraph.js';
import type { LaunchConfig, EngineConfig, EngineSnapshot, EngineSpec, SoundSources } from '../model/spec.js';
import type { EngineHost, LagListener, SnapshotListener } from './EngineHost.js';
import { CONTROL_PARAMS } from './worklet/controls.js';
import { solvedPlenum } from '../scene/inletLayout.js';

/** Samples in each frame's waveform, and the FFT size of the spectrum: an AnalyserNode's 2048. */
const FFT_SIZE = 2048;
/** The AnalyserNode smoothing the web app's scope is set to. */
const SMOOTHING = 0.6;

type Command =
  | { type: 'engine'; engine: Partial<EngineSpec> }
  | { type: 'graph'; graph: ExhaustGraph | null }
  | { type: 'sources'; sources: SoundSources }
  | { type: 'listener'; position: [number, number, number] | null }
  | { type: 'launch'; config: LaunchConfig | null }
  | { type: 'snapshotRate'; hz: number }
  | { type: 'timeScale'; scale: number }
  | { type: 'controls'; throttle: number; load: number }
  | { type: 'ignition'; on: boolean }
  | { type: 'suspend' }
  | { type: 'resume' };

interface StreamInfo {
  sampleRate: number;
  bufferFrames: number | null;
  leadFrames: number;
}

/** The snapshot as the simulation serialises it, before its sample arrays become typed arrays. */
type RawSnapshot = Omit<EngineSnapshot, 'pipePressure' | 'ductPressure' | 'inletPressure' | 'inletVelocity' | 'runnerPressure' | 'plenumZones' | 'launch'> & {
  pipePressure: number[];
  ductPressure: number[];
  inletPressure: number[];
  inletVelocity: number[];
  runnerPressure: number[];
  plenumZones: number[];
  launch: (Omit<NonNullable<EngineSnapshot['launch']>, 'points'> & { points: number[] }) | null;
};

export class NativeEngine implements EngineHost {
  private readonly config: EngineConfig;
  private readonly snapshotListeners = new Set<SnapshotListener>();
  private readonly lagListeners = new Set<LagListener>();
  private info: StreamInfo | null = null;
  private playing = false;
  private timeScale = 1;
  private ignition = true;
  private opening: Promise<void> | null = null;
  private readonly decoder = new TextDecoder();

  private readonly waveform = new Float32Array(FFT_SIZE);
  private hasWaveform = false;
  private readonly window = blackman(FFT_SIZE);
  private readonly re = new Float64Array(FFT_SIZE);
  private readonly im = new Float64Array(FFT_SIZE);
  private readonly smoothed = new Float64Array(FFT_SIZE / 2);

  /** @param rate Sample rate to ask the device for, Hz. It may give its own instead. */
  constructor(
    config: EngineConfig,
    private rate = 48000,
  ) {
    this.config = structuredClone(config);
    // Loose pipes carry no gas, so the solver is not given them.
    if (this.config.graph) this.config.graph = solverGraph(this.config.graph);
  }

  get running(): boolean {
    return this.playing;
  }

  get sampleRate(): number {
    return this.info?.sampleRate ?? this.rate;
  }

  async setSampleRate(hz: number): Promise<void> {
    this.rate = hz;
    if (!this.info) return;
    const wasPlaying = this.playing;
    await this.open();
    if (!wasPlaying) await this.suspend();
  }

  async start(): Promise<void> {
    if (!this.info) {
      this.opening ??= this.open().finally(() => (this.opening = null));
      await this.opening;
    } else {
      await this.send({ type: 'resume' });
    }
    this.playing = true;
  }

  async suspend(): Promise<void> {
    await this.send({ type: 'suspend' });
    this.playing = false;
  }

  async toggle(): Promise<boolean> {
    if (this.playing) {
      await this.suspend();
      return false;
    }
    await this.start();
    return true;
  }

  onSnapshot(listener: SnapshotListener): () => void {
    this.snapshotListeners.add(listener);
    return () => this.snapshotListeners.delete(listener);
  }

  onLag(listener: LagListener): () => void {
    this.lagListeners.add(listener);
    return () => this.lagListeners.delete(listener);
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

  setEngine(partial: Partial<EngineSpec>): void {
    Object.assign(this.config.engine, partial);
    partial = this.withPlenum(partial);
    // The continuous controls go on their own command, which the simulation applies without any of the
    // rebuilding an engine change can cause.
    let rest: Partial<EngineSpec> | null = null;
    let controls = false;
    for (const key of Object.keys(partial) as Array<keyof EngineSpec>) {
      if ((CONTROL_PARAMS as readonly string[]).includes(key)) controls = true;
      else (rest ??= {} as Record<string, unknown>)[key] = partial[key];
    }
    if (controls) {
      void this.send({ type: 'controls', throttle: this.config.engine.throttle, load: this.config.engine.load });
    }
    if (rest) void this.send({ type: 'engine', engine: rest });
  }

  setGraph(graph: ExhaustGraph | null): void {
    // Loose pipes carry no gas, so the solver is not given them.
    const solved = graph ? solverGraph(graph) : null;
    this.config.graph = solved ?? undefined;
    void this.send({ type: 'graph', graph: solved });
  }

  setSources(sources: SoundSources): void {
    this.config.sources = sources;
    void this.send({ type: 'sources', sources });
  }

  setListener(position: [number, number, number] | null): void {
    if (position) this.config.listener = position;
    else delete this.config.listener;
    void this.send({ type: 'listener', position });
  }

  launch(config: LaunchConfig | null): void {
    void this.send({ type: 'launch', config });
  }

  setTimeScale(scale: number): void {
    this.timeScale = scale;
    void this.send({ type: 'timeScale', scale });
  }

  setIgnition(on: boolean): void {
    this.ignition = on;
    void this.send({ type: 'ignition', on });
  }

  readWaveform(out: Float32Array<ArrayBuffer>): boolean {
    if (!this.hasWaveform || !this.playing) return false;
    const from = Math.max(0, FFT_SIZE - out.length);
    out.set(this.waveform.subarray(from, from + Math.min(out.length, FFT_SIZE)));
    return true;
  }

  readSpectrum(out: Float32Array<ArrayBuffer>): boolean {
    if (!this.hasWaveform || !this.playing) return false;
    const { re, im, window, smoothed } = this;
    for (let i = 0; i < FFT_SIZE; i++) {
      re[i] = this.waveform[i]! * window[i]!;
      im[i] = 0;
    }
    fft(re, im);
    const bins = Math.min(out.length, FFT_SIZE / 2);
    for (let k = 0; k < bins; k++) {
      const mag = Math.hypot(re[k]!, im[k]!) / FFT_SIZE;
      smoothed[k] = SMOOTHING * smoothed[k]! + (1 - SMOOTHING) * mag;
      out[k] = 20 * Math.log10(smoothed[k]!);
    }
    return true;
  }

  get spectrumSize(): number {
    return FFT_SIZE / 2;
  }

  private async send(command: Command): Promise<void> {
    if (!this.info && !this.opening) return;
    await this.opening;
    await invoke('audio_command', { command });
  }

  /** Start the simulation and the stream from the config as it stands, replacing any running. */
  private async open(): Promise<void> {
    const frames = new Channel<ArrayBuffer>();
    frames.onmessage = (buffer) => this.receive(buffer);
    this.info = await invoke<StreamInfo>('audio_start', {
      config: this.startConfig(),
      sampleRate: this.rate,
      bufferFrames: null,
      frames,
    });
    this.smoothed.fill(0);
    this.hasWaveform = false;
    // A new simulation starts in real time.
    if (this.timeScale !== 1) await invoke('audio_command', { command: { type: 'timeScale', scale: this.timeScale } });
    // And with the ignition on.
    if (!this.ignition) await invoke('audio_command', { command: { type: 'ignition', on: false } });
  }

  private receive(buffer: ArrayBuffer): void {
    const view = new DataView(buffer);
    const kind = view.getUint8(0);
    const jsonLength = view.getUint32(4, true);
    const json = this.decoder.decode(new Uint8Array(buffer, 8, jsonLength));
    if (kind === 1) {
      const { behind } = JSON.parse(json) as { behind: boolean };
      for (const l of this.lagListeners) l(behind);
      return;
    }
    const raw = JSON.parse(json) as RawSnapshot;
    const snapshot: EngineSnapshot = {
      ...raw,
      pipePressure: Float32Array.from(raw.pipePressure),
      ductPressure: Float32Array.from(raw.ductPressure),
      inletPressure: Float32Array.from(raw.inletPressure),
      inletVelocity: Float32Array.from(raw.inletVelocity),
      runnerPressure: Float32Array.from(raw.runnerPressure),
      plenumZones: Float32Array.from(raw.plenumZones),
      launch: raw.launch && { ...raw.launch, points: Float32Array.from(raw.launch.points) },
    };
    const waveAt = 8 + jsonLength + ((4 - (jsonLength % 4)) % 4);
    const samples = (buffer.byteLength - waveAt) / 4;
    if (samples > 0) {
      const wave = new Float32Array(buffer, waveAt, Math.min(samples, FFT_SIZE));
      this.waveform.set(wave, FFT_SIZE - wave.length);
      this.hasWaveform = true;
    }
    for (const l of this.snapshotListeners) l(snapshot);
  }
}

/** Blackman window of `n` points, as an AnalyserNode applies it. */
function blackman(n: number): Float64Array {
  const w = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const x = (2 * Math.PI * i) / n;
    w[i] = 0.42 - 0.5 * Math.cos(x) + 0.08 * Math.cos(2 * x);
  }
  return w;
}

/** In-place iterative radix-2 FFT. */
function fft(re: Float64Array, im: Float64Array): void {
  const n = re.length;
  for (let i = 1, j = 0; i < n; i++) {
    let bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      [re[i], re[j]] = [re[j]!, re[i]!];
      [im[i], im[j]] = [im[j]!, im[i]!];
    }
  }
  for (let len = 2; len <= n; len <<= 1) {
    const ang = (-2 * Math.PI) / len;
    const wRe = Math.cos(ang);
    const wIm = Math.sin(ang);
    for (let i = 0; i < n; i += len) {
      let curRe = 1;
      let curIm = 0;
      for (let k = 0; k < len / 2; k++) {
        const a = i + k;
        const b = a + len / 2;
        const vRe = re[b]! * curRe - im[b]! * curIm;
        const vIm = re[b]! * curIm + im[b]! * curRe;
        re[b] = re[a]! - vRe;
        im[b] = im[a]! - vIm;
        re[a] = re[a]! + vRe;
        im[a] = im[a]! + vIm;
        const next = curRe * wRe - curIm * wIm;
        curIm = curRe * wIm + curIm * wRe;
        curRe = next;
      }
    }
  }
}
