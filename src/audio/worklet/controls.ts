/**
 * The controls a player works continuously, sent to the worklet as AudioParams rather than messages.
 *
 * Messages are handled on the audio thread in the gaps between render blocks. An overloaded thread
 * renders back to back to catch up and leaves no gaps, so on a phone a throttle sent as a message
 * would simply stop responding while the engine carried on. Parameters arrive with every `process` call instead, and
 * cannot be starved. The main thread keeps these out of `engine` messages, so each has one path.
 *
 * A module of its own because both threads need it, and importing `processor.ts` on the main thread
 * would run its `registerProcessor`.
 */
export const CONTROL_PARAMS = ['throttle', 'load'] as const;
export type ControlParam = (typeof CONTROL_PARAMS)[number];
