/**
 * Far-field radiation from the exhaust mouth.
 *
 * A small pipe mouth is an acoustic monopole: the pressure it radiates is
 * proportional to the *rate of change* of the volume flow leaving it,
 *
 *     p(r, t) = rho / (4*pi*r) * dQ/dt
 *
 * which applies a 6 dB/octave tilt. That derivative is the single biggest reason a
 * real exhaust sounds sharp and cracky rather than like a muffled thump: it
 * emphasises the steep leading edge of each blowdown pulse.
 *
 * But that formula only holds while the mouth is small compared with a wavelength
 * (ka << 1). Above the corner frequency wc = c/a the mouth radiates like a piston
 * and its efficiency stops climbing. Taken literally, the plain derivative makes a
 * wide megaphone absurdly bright. So the transfer applied here is a first-order
 * highpass at wc, scaled by rho*wc/(4*pi*r):
 *
 *     below wc:  -> rho/(4*pi*r) * dQ/dt   (correct monopole)
 *     above wc:  -> flat                   (piston-like, efficiency saturates)
 *
 * Note that at low frequency the mouth size cancels out entirely, which is the
 * right answer: what escapes depends on how fast the gas volume is changing, not on
 * how big the hole is.
 */

import { GAS, density } from '../../model/spec.js';
import { DCBlocker } from './dsp.js';

const RHO_AMB = density(GAS.pAmb, GAS.tAmb);

export class FarField {
  private x1 = 0;
  private y1 = 0;
  private a = 0;
  private scale = 0;
  private readonly dc = new DCBlocker();

  /**
   * One pole enforcing the band limit `setCutoff` is given: `EngineSim` passes the lower of the
   * duct's plane-wave validity limit and the solver's resolution limit.
   */
  private planeC = 1;
  private plane1 = 0;

  constructor(
    private readonly sampleRate: number,
    cutoffRad: number,
    planeWaveCutoffRad = Infinity,
  ) {
    this.a = 0;
    this.scale = 0;
    this.setCutoff(cutoffRad, planeWaveCutoffRad);
  }

  /** Called whenever the pipe is rebuilt and the mouth or duct size changes. */
  setCutoff(cutoffRad: number, planeWaveCutoffRad = Infinity): void {
    const wc = Math.max(cutoffRad, 100);
    this.a = Math.exp(-wc / this.sampleRate);
    this.scale = (RHO_AMB * wc) / (4 * Math.PI);

    // Roll off above the frequency where the duct stops carrying plane waves. This
    // is a statement about where the model is valid, not a tone control: 1D theory
    // has nothing to say above it, so radiating that content would be inventing it.
    const wp = Math.min(planeWaveCutoffRad, Math.PI * this.sampleRate * 0.9);
    this.planeC = Number.isFinite(wp) ? 1 - Math.exp(-wp / this.sampleRate) : 1;
  }

  /**
   * @param q Volume flow leaving the mouth, m^3/s.
   * @returns Radiated pressure referred to 1 m, Pa. Spreading, the ground reflection
   *   and air absorption are all applied afterwards by `Listener`.
   */
  process(q: number): number {
    // First-order highpass: y = a*(y1 + x - x1).
    const y = this.a * (this.y1 + q - this.x1);
    this.x1 = q;
    this.y1 = y;
    let p = this.scale * y;

    // Plane-wave validity limit.
    this.plane1 += this.planeC * (p - this.plane1);
    p = this.plane1;

    // The source has a net DC outflow every cycle; without blocking it the signal
    // wanders off centre and eats headroom.
    return this.dc.process(p);
  }

  reset(): void {
    this.x1 = 0;
    this.y1 = 0;
    this.plane1 = 0;
    this.dc.reset();
  }
}
