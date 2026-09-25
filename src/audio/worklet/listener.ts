/**
 * What reaches the listener's ear: the direct path plus the ground reflection.
 *
 * Nobody hears an engine in free space. Standing near one outdoors, the ground sends a
 * second, slightly later copy of everything, and the two comb-filter each other. For a
 * mouth 0.35 m up and an ear 1.2 m up at 1.5 m away the path difference is about
 * 0.43 m, which is a 1.26 ms delay and a first cancellation near 400 Hz — right in the
 * middle of the exhaust note. It is a big, strongly frequency-dependent colouration
 * that shifts as you move, and leaving it out is much of why simulated engines sound
 * like they were recorded in a vacuum.
 *
 * The reflection is also duller than the direct sound, because real ground absorbs more
 * at high frequency than at low, so it gets a lowpass as well as a level drop.
 *
 * Air absorption over the path is included too. It is genuinely small at these
 * distances — a decibel or so at 10 kHz over 10 m — but it is the reason a distant
 * engine sounds muffled rather than merely quieter.
 */

import { ambientSoundSpeed } from '../../model/spec.js';
import { Delay, OnePole } from './dsp.js';

export interface ListenerGeometry {
  /** Horizontal distance, m. */
  distance: number;
  /** Ear height, m. */
  micHeight: number;
  /** Exhaust mouth height, m. */
  sourceHeight: number;
  /** Ground reflection coefficient, 0..1. */
  reflection: number;
}

export class Listener {
  private readonly delay: Delay;
  /** Ground surfaces absorb high frequencies preferentially. */
  private readonly groundLoss = new OnePole();
  private readonly airDirect = new OnePole();
  private readonly airGround = new OnePole();

  private directGain = 1;
  private groundGain = 0;

  constructor(private readonly sampleRate: number) {
    // 0.35 s of delay line covers any plausible path difference with room to spare.
    this.delay = new Delay(Math.ceil(sampleRate * 0.35));
    this.setGeometry({
      distance: 1.5,
      micHeight: 1.2,
      sourceHeight: 0.35,
      reflection: 0.7,
    });
  }

  setGeometry(g: ListenerGeometry): void {
    const c = ambientSoundSpeed();
    const d = Math.max(g.distance, 0.15);
    const hm = Math.max(g.micHeight, 0.02);
    const hs = Math.max(g.sourceHeight, 0.02);

    // Direct path, and the image-source path reflected in the ground plane.
    const rDirect = Math.hypot(d, hm - hs);
    const rGround = Math.hypot(d, hm + hs);

    this.directGain = 1 / rDirect;
    this.groundGain = Math.max(g.reflection, 0) / rGround;
    this.delay.setDelay(((rGround - rDirect) / c) * this.sampleRate);

    // Ground absorption: progressively duller for a longer, more grazing bounce.
    this.groundLoss.setCutoff(2600, this.sampleRate);

    // Atmospheric absorption. Roughly 0.1 dB/m at 10 kHz at room conditions, which
    // corresponds to a one-pole whose corner falls as the path lengthens.
    this.airDirect.setCutoff(airCutoffHz(rDirect), this.sampleRate);
    this.airGround.setCutoff(airCutoffHz(rGround), this.sampleRate);
  }

  /**
   * @param source Radiated pressure referred to 1 m, Pa.
   * @returns Pressure at the ear, Pa.
   */
  process(source: number): number {
    const direct = this.airDirect.process(source) * this.directGain;
    const bounced = this.delay.process(source);
    const ground = this.airGround.process(this.groundLoss.process(bounced)) * this.groundGain;
    return direct + ground;
  }

  reset(): void {
    this.delay.reset();
    this.groundLoss.reset();
    this.airDirect.reset();
    this.airGround.reset();
  }
}

/**
 * One-pole corner, Hz, approximating atmospheric absorption over `metres`. Tuned so
 * the loss at 10 kHz lands near the standard 0.1 dB/m; effectively transparent at a
 * metre or two and clearly audible across a field.
 */
function airCutoffHz(metres: number): number {
  return 90000 / Math.max(metres, 0.2);
}
