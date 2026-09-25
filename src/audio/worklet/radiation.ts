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
 * (ka << 1). Above the corner frequency wc = 2c/a the mouth radiates like a piston
 * and its efficiency stops climbing: that is where the monopole's power would pass the
 * saturated piston's `rho c A u^2 / 2`. Taken literally, the plain derivative makes a
 * wide megaphone absurdly bright. So the transfer applied here is a first-order
 * highpass at wc, scaled by rho*wc/(4*pi*r):
 *
 *     below wc:  -> rho/(4*pi*r) * dQ/dt   (correct monopole)
 *     above wc:  -> flat                   (piston-like, efficiency saturates)
 *
 * Note that at low frequency the mouth size cancels out entirely, which is the
 * right answer: what escapes depends on how fast the gas volume is changing, not on
 * how big the hole is.
 *
 * The highpass is the bilinear transform of `s / (s + wc)`, which keeps the low-frequency
 * slope `jw / wc` exact at any sample rate, so the monopole level does not depend on it. Its
 * zero at DC also removes the net outflow every cycle carries.
 */

import { GAS, density } from '../../model/spec.js';

const RHO_AMB = density(GAS.pAmb, GAS.tAmb);

export class FarField {
  private x1 = 0;
  private y1 = 0;
  /** Bilinear highpass: `y = b0 (x - x1) + a1 y1`. */
  private b0 = 0;
  private a1 = 0;
  private scale = 0;

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
    this.setCutoff(cutoffRad, planeWaveCutoffRad);
  }

  /** Called whenever the pipe is rebuilt and the mouth or duct size changes. */
  setCutoff(cutoffRad: number, planeWaveCutoffRad = Infinity): void {
    const wc = Math.max(cutoffRad, 100);
    const k = 2 * this.sampleRate;
    this.b0 = k / (k + wc);
    this.a1 = (k - wc) / (k + wc);
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
    // First-order highpass.
    const y = this.b0 * (q - this.x1) + this.a1 * this.y1;
    this.x1 = q;
    this.y1 = y;
    let p = this.scale * y;

    // Plane-wave validity limit.
    this.plane1 += this.planeC * (p - this.plane1);
    p = this.plane1;
    return p;
  }

  reset(): void {
    this.x1 = 0;
    this.y1 = 0;
    this.plane1 = 0;
  }
}
