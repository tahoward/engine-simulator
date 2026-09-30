/**
 * What the interface needs from wherever the simulation runs: in the browser, `AudioEngine` with the
 * simulation in an AudioWorklet; in the desktop app, `NativeEngine`, with it running natively.
 */

import type { ExhaustGraph } from '../model/exhaustGraph.js';
import type { LaunchConfig, EngineSnapshot, EngineSpec } from '../model/spec.js';

export type SnapshotListener = (snapshot: EngineSnapshot) => void;
/** Called with `true` when the audio stops keeping up with real time, and `false` when it recovers. */
export type LagListener = (behind: boolean) => void;

export interface EngineHost {
  /** Whether sound is playing. */
  readonly running: boolean;
  /** The sample rate the simulation runs at, Hz: the one asked for until the audio has started. */
  readonly sampleRate: number;
  /** Switch the sample rate. The simulation restarts from cold. */
  setSampleRate(hz: number): Promise<void>;
  start(): Promise<void>;
  suspend(): Promise<void>;
  /** Start or suspend; resolves to whether it is now running. */
  toggle(): Promise<boolean>;
  /**
   * Switch the ignition. Off, the engine coasts to a standstill on its friction and pumping and the
   * pipes ring down, with the audio still playing; on, it starts again. Safe to call before the audio
   * has started, and kept across a change of sample rate.
   */
  setIgnition(on: boolean): void;
  onSnapshot(listener: SnapshotListener): () => void;
  onLag(listener: LagListener): () => void;
  /** Change the engine: only the fields given. Safe to call before the audio has started. */
  setEngine(partial: Partial<EngineSpec>): void;
  /** Replace the exhaust graph; `null` compiles one from the layout. */
  setGraph(graph: ExhaustGraph | null): void;
  /** Start a launch through `config`, or with `null` end the one in progress. */
  launch(config: LaunchConfig | null): void;
  /**
   * Run the simulation at `scale` of real time: 1 is real time, 0.01 a hundred times slower, with the
   * sound slowed and pitched down to match. Safe to call before the audio has started.
   */
  setTimeScale(scale: number): void;
  /** The latest output samples into `out`. Returns false if there are none: not started, or stopped. */
  readWaveform(out: Float32Array<ArrayBuffer>): boolean;
  /** The magnitude spectrum of the output, dB, into `out`. Returns false if there is none: not started, or stopped. */
  readSpectrum(out: Float32Array<ArrayBuffer>): boolean;
  /** Bins `readSpectrum` fills. */
  readonly spectrumSize: number;
}
