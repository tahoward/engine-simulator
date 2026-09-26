/// <reference path="./worklet.d.ts" />

/**
 * AudioWorklet shell. Deliberately thin: all the physics lives in `EngineSim`, which
 * knows nothing about audio plumbing and is therefore testable in Node.
 *
 * The simulation runs here, at the audio sample rate, because it is stepped once per
 * audio sample — the gas solver takes one step per sample too. Running it on the main
 * thread would mean resampling, and would put the acoustics at the mercy of garbage
 * collection and rendering hitches.
 */

import type { DynoConfig, EngineConfig, EngineSnapshot, EngineSpec, PipeSegment } from '../../model/spec.js';
import type { ExhaustGraph } from '../../model/exhaustGraph.js';
import { CONTROL_PARAMS } from './controls.js';
import { EngineSim } from './engineSim.js';

/** Main thread -> worklet. */
export type ToWorklet =
  | { type: 'engine'; engine: Partial<EngineSpec> }
  | { type: 'pipe'; pipe: PipeSegment[]; collector: PipeSegment[] }
  | { type: 'graph'; graph: ExhaustGraph | null }
  | { type: 'snapshotRate'; hz: number }
  | { type: 'dyno'; config: DynoConfig | null };

/** Worklet -> main thread. */
export type FromWorklet = { type: 'snapshot'; snapshot: EngineSnapshot };

class EngineProcessor extends AudioWorkletProcessor {
  static get parameterDescriptors() {
    return CONTROL_PARAMS.map((name) => ({ name, defaultValue: 0, automationRate: 'k-rate' as const }));
  }

  private readonly sim: EngineSim;
  private snapshotInterval: number;
  private sinceSnapshot = 0;

  constructor(options: AudioWorkletNodeOptions) {
    super();
    const config = options.processorOptions as EngineConfig;
    this.sim = new EngineSim(sampleRate, config);
    this.snapshotInterval = Math.round(sampleRate / 60);

    this.port.onmessage = (e: MessageEvent<ToWorklet>) => {
      const msg = e.data;
      switch (msg.type) {
        case 'engine':
          this.sim.setEngine(msg.engine);
          break;
        case 'graph':
          // A drawn exhaust. Same one-off cost as a pipe edit: a new set of ducts on the audio
          // thread, with the ramp hiding the discontinuity.
          this.sim.setGraph(msg.graph);
          break;
        case 'pipe':
          // Allocates a new set of ducts on the audio thread. That is a one-off cost
          // paid only when the user edits the pipe, and the simulation ramps back
          // up over ~8 ms to hide the discontinuity.
          this.sim.setPipe(msg.pipe, msg.collector);
          break;
        case 'dyno':
          // `null` ends the run in progress.
          if (msg.config) this.sim.startDyno(msg.config);
          else this.sim.stopDyno();
          break;
        case 'snapshotRate':
          this.snapshotInterval = Math.max(1, Math.round(sampleRate / msg.hz));
          break;
      }
    };
  }

  process(
    _inputs: Float32Array[][],
    outputs: Float32Array[][],
    parameters: Record<string, Float32Array>,
  ): boolean {
    const out = outputs[0];
    if (!out || out.length === 0) return true;
    const mono = out[0]!;
    const n = mono.length;

    // k-rate, so one value per block. `setControls` returns at once when nothing moved.
    this.sim.setControls(parameters.throttle![0]!, parameters.load![0]!);

    // No timing here. `performance` is not exposed in AudioWorkletGlobalScope (verified
    // absent in Chrome), and `currentTime` only advances once per block, so the audio
    // thread genuinely cannot measure its own cost. The snapshot reports the solver's
    // cell and substep counts instead, which is what cost is proportional to.
    for (let i = 0; i < n; i++) mono[i] = this.sim.tick();

    // Mirror to any further channels rather than running a second simulation.
    for (let ch = 1; ch < out.length; ch++) out[ch]!.set(mono);

    this.sinceSnapshot += n;
    if (this.sinceSnapshot >= this.snapshotInterval) {
      this.sinceSnapshot = 0;
      const snapshot = this.sim.snapshot();
      const msg: FromWorklet = { type: 'snapshot', snapshot };
      // Transfer the pressure array rather than structured-cloning it. `snapshot`
      // hands over a fresh copy each time, so the simulation keeps its own buffer.
      this.port.postMessage(msg, [snapshot.pipePressure.buffer as ArrayBuffer]);
    }

    return true;
  }
}

registerProcessor('engine-processor', EngineProcessor);
