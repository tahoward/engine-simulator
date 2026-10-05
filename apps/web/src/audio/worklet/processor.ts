/// <reference path="./worklet.d.ts" />

/**
 * AudioWorklet shell. Deliberately thin: the simulation is the Wasm build of `crates/engine-sim`,
 * loaded by `Sim`, and this only moves messages and samples.
 *
 * The simulation runs here, at the audio sample rate, because it is stepped once per audio sample —
 * the gas solver takes one step per sample too. Running it on the main thread would mean resampling,
 * and would put the acoustics at the mercy of garbage collection and rendering hitches.
 */

import type { LaunchConfig, EngineConfig, EngineSnapshot, EngineSpec, SoundSources } from '../../model/spec.js';
import type { ExhaustGraph } from '../../model/exhaustGraph.js';
import { CONTROL_PARAMS } from './controls.js';
import { Sim } from './sim.js';

/** Main thread -> worklet. */
export type ToWorklet =
  | { type: 'engine'; engine: Partial<EngineSpec> }
  | { type: 'graph'; graph: ExhaustGraph | null }
  | { type: 'sources'; sources: SoundSources }
  | { type: 'listener'; position: [number, number, number] | null; right: [number, number, number] | null }
  | { type: 'snapshotRate'; hz: number }
  | { type: 'timeScale'; scale: number }
  | { type: 'ignition'; on: boolean }
  | { type: 'launch'; config: LaunchConfig | null };

/** Worklet -> main thread. */
export type FromWorklet =
  | { type: 'snapshot'; snapshot: EngineSnapshot }
  /** The simulation failed and the processor has stopped; the reason, for the console. */
  | { type: 'error'; message: string };

class EngineProcessor extends AudioWorkletProcessor {
  static get parameterDescriptors() {
    return CONTROL_PARAMS.map((name) => ({ name, defaultValue: 0, automationRate: 'k-rate' as const }));
  }

  private readonly sim: Sim;
  private snapshotInterval: number;
  private sinceSnapshot = 0;
  /** Whether the output has two channels to put the two ears in, as it is asked to. */
  private stereo = false;

  constructor(options: AudioWorkletNodeOptions) {
    super();
    const config = options.processorOptions as EngineConfig;
    try {
      this.sim = new Sim(sampleRate, config);
    } catch (err) {
      this.report(err);
      throw err;
    }
    this.snapshotInterval = Math.round(sampleRate / 60);

    this.port.onmessage = (e: MessageEvent<ToWorklet>) => {
      const msg = e.data;
      // A message the simulation rejects is reported and dropped: throwing here would kill the node.
      try {
        switch (msg.type) {
          case 'engine':
            this.sim.setEngine(msg.engine);
            break;
          case 'graph':
            // A drawn exhaust: a new set of ducts on the audio thread, a one-off cost paid only when
            // the user edits, with a short ramp hiding the discontinuity.
            this.sim.setGraph(msg.graph);
            break;
          case 'sources':
            this.sim.setSources(msg.sources);
            break;
          case 'listener':
            this.sim.setListener(msg.position, msg.right);
            break;
          case 'launch':
            // `null` ends the run in progress.
            if (msg.config) this.sim.startLaunch(msg.config);
            else this.sim.stopLaunch();
            break;
          case 'snapshotRate':
            this.snapshotInterval = Math.max(1, Math.round(sampleRate / msg.hz));
            break;
          case 'timeScale':
            this.sim.setTimeScale(msg.scale);
            break;
          case 'ignition':
            this.sim.setIgnition(msg.on);
            break;
        }
      } catch (err) {
        console.error(err);
      }
    };
  }

  /** Tell the main thread why the processor is stopping: an error thrown here reaches it without one. */
  private report(err: unknown): void {
    const message = err instanceof Error ? `${err.message}\n${err.stack ?? ''}` : String(err);
    const msg: FromWorklet = { type: 'error', message };
    this.port.postMessage(msg);
  }

  process(
    _inputs: Float32Array[][],
    outputs: Float32Array[][],
    parameters: Record<string, Float32Array>,
  ): boolean {
    const out = outputs[0];
    if (!out || out.length === 0) return true;
    const left = out[0]!;

    // k-rate, so one value per block. The simulation returns at once when nothing moved.
    this.sim.setControls(parameters.throttle![0]!, parameters.load![0]!);

    // No timing here. `performance` is not exposed in AudioWorkletGlobalScope, and `currentTime` only
    // advances once per block, so the audio thread cannot measure its own cost. The snapshot reports
    // the solver's cell and substep counts instead, which is what cost is proportional to.
    const stereo = out.length >= 2;
    if (stereo !== this.stereo) {
      this.stereo = stereo;
      this.sim.setStereo(stereo);
    }
    if (stereo) this.sim.renderStereoInto(left, out[1]!);
    else this.sim.renderInto(left);

    this.sinceSnapshot += left.length;
    if (this.sinceSnapshot >= this.snapshotInterval) {
      this.sinceSnapshot = 0;
      const snapshot = this.sim.snapshot();
      const msg: FromWorklet = { type: 'snapshot', snapshot };
      // Transfer the pressure arrays rather than structured-cloning them: each snapshot has its own.
      this.port.postMessage(msg, [
        snapshot.pipePressure.buffer as ArrayBuffer,
        snapshot.ductPressure.buffer as ArrayBuffer,
        snapshot.inletPressure.buffer as ArrayBuffer,
        snapshot.inletVelocity.buffer as ArrayBuffer,
        snapshot.runnerPressure.buffer as ArrayBuffer,
      ]);
    }

    return true;
  }
}

registerProcessor('engine-processor', EngineProcessor);
