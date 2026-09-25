/**
 * AudioWorkletGlobalScope declarations. These globals exist only inside the worklet
 * and are not part of lib.dom, so they have to be declared by hand.
 */

declare const sampleRate: number;
declare const currentTime: number;
declare const currentFrame: number;

declare abstract class AudioWorkletProcessor {
  readonly port: MessagePort;
  constructor(options?: AudioWorkletNodeOptions);
  abstract process(
    inputs: Float32Array[][],
    outputs: Float32Array[][],
    parameters: Record<string, Float32Array>,
  ): boolean;
}

declare function registerProcessor(
  name: string,
  processorCtor: new (options: AudioWorkletNodeOptions) => AudioWorkletProcessor,
): void;
