/**
 * Poppet valve: cam lift profile plus compressible flow through the resulting
 * annular opening.
 *
 * This is where the valves earn their place as a sound source. They are not a gate
 * on a pre-baked waveform — they are the actual orifice whose area, moment by
 * moment, decides how fast the cylinder can dump into the pipe. Change the lift or
 * the timing and the shape of the pressure pulse changes, so the note changes.
 */

import { GAS } from '../../model/spec.js';
import { windowPhase } from './dsp.js';

/** Discharge coefficient of a poppet valve at moderate lift. */
export const VALVE_CD = 0.72;

/** Valve seat half-angle; 45 degrees is near-universal. */
const SEAT_ANGLE = Math.PI / 4;

/**
 * Lift, m, at crank angle `deg`. Handles windows that wrap past 720 degrees, which
 * the exhaust valve always does on a four-stroke with overlap.
 *
 * The profile is `sin(pi*u)^2` — a smooth, symmetric cam with zero lift *and* zero
 * velocity at both ends, so opening and closing are ramped rather than stepped. A
 * discontinuous lift would inject a broadband click every cycle that would be mistaken
 * for a bug.
 *
 * The exponent is set by real cams, through the gap between advertised duration (lift from
 * zero) and duration at 0.050 in: a stock small-block's 256 and 195 degrees give 2.0, a
 * street performance cam's 262 and 218 about 1.7. It was 1.25, fatter at the ends than any
 * real profile, and the ends are what overlap is made of: at top dead centre the intake stood
 * three times as far open as a real one, and at idle, against a near-vacuum manifold, that
 * poured exhaust back up the intake until the trapped charge was half spent gas. With this
 * shape a stock V8 idles on 25%, inside the 20-30% real engines show.
 */
export function valveLift(deg: number, open: number, close: number, maxLift: number): number {
  const u = windowPhase(deg, open, close);
  if (u < 0) return 0;
  const s = Math.sin(Math.PI * u);
  return maxLift * s * s;
}

/**
 * Effective flow area, m^2, for a given lift.
 *
 * At low lift the restriction is the curtain area swept between the valve face and
 * its seat, `pi * D * L * cos(beta)`. Past a certain lift the valve stops being the
 * restriction and the port throat takes over, so the area saturates — which is why
 * fitting a wilder cam eventually stops helping.
 */
export function valveFlowArea(lift: number, valveDia: number): number {
  if (lift <= 0) return 0;
  const curtain = Math.PI * valveDia * lift * Math.cos(SEAT_ANGLE);
  // 0.85 accounts for stem and guide blockage in the throat.
  const throat = ((Math.PI * valveDia * valveDia) / 4) * 0.85;
  return Math.min(curtain, throat);
}

/**
 * Isentropic compressible mass flow through an orifice, kg/s, always positive.
 * The caller decides direction by choosing which side is `up`.
 *
 * Below the critical pressure ratio the throat is choked and the flow stops
 * responding to further downstream pressure drop — the pressure ratio is clamped
 * rather than the equation being allowed to turn over, which it does unphysically
 * past the critical point.
 */
export function orificeMassFlow(
  area: number,
  cd: number,
  pUp: number,
  tUp: number,
  pDown: number,
  gamma: number,
): number {
  // A wrapper small enough to inline, around a solve that takes no floating-point arguments. The
  // callers are the solver's and the engine loop's hottest functions, both too big to inline the
  // solve itself, and each float crossing a call that is not inlined is boxed into a fresh heap object.
  const io = ORIFICE_IO;
  io[0] = area;
  io[1] = cd;
  io[2] = pUp;
  io[3] = tUp;
  io[4] = pDown;
  io[5] = gamma;
  orificeSolve(io);
  return io[6]!;
}

/**
 * `orificeMassFlow`'s arguments in slots 0-5, and its answer in slot 6.
 *
 * Exported with `orificeSolve` for a caller whose budget will not stretch to inlining even the wrapper.
 */
export const ORIFICE_IO = new Float64Array(7);

export function orificeSolve(io: Float64Array): void {
  const area = io[0]!;
  const cd = io[1]!;
  const pUp = io[2]!;
  const tUp = io[3]!;
  const pDown = io[4]!;
  const gamma = io[5]!;
  io[6] = 0;
  if (area <= 0 || pUp <= pDown || pUp <= 0 || tUp <= 0) return;

  // Cached per gamma. This is a constant, and it was being raised to a power on every call
  // — and this function is called for both valves of every cylinder plus the throttle, on
  // every audio sample, so it is squarely on the hot path.
  const c = gammaConstants(gamma);
  let pr = pDown / pUp;
  if (pr < c.critical) pr = c.critical; // choked

  const term = Math.pow(pr, c.exp1) - Math.pow(pr, c.exp2);
  if (term <= 0) return;
  const flux = Math.sqrt(c.fluxScale * term);

  io[6] = (cd * area * pUp * flux) / Math.sqrt(GAS.R * tUp);
}

/**
 * Constants of the isentropic orifice equation that depend only on gamma.
 *
 * A tiny cache rather than a lookup table because the set of gammas in play is decided by
 * the caller, not here: cylinder gas, exhaust gas and ambient air are all used, and a
 * mixture-dependent gamma would add more.
 */
interface GammaConstants {
  critical: number;
  exp1: number;
  exp2: number;
  fluxScale: number;
}

/**
 * Scanned, not a `Map`: a `Map` lookup takes its key as a tagged value, so every lookup with a gamma
 * read out of a typed array boxed it into a fresh heap object. There are only ever a handful of gammas.
 */
const cachedGammas: number[] = [];
const cachedConstants: GammaConstants[] = [];

function gammaConstants(gamma: number): GammaConstants {
  for (let i = 0; i < cachedGammas.length; i++) {
    if (cachedGammas[i] === gamma) return cachedConstants[i]!;
  }
  const c = {
    critical: Math.pow(2 / (gamma + 1), gamma / (gamma - 1)),
    exp1: 2 / gamma,
    exp2: (gamma + 1) / gamma,
    fluxScale: (2 * gamma) / (gamma - 1),
  };
  cachedGammas.push(gamma);
  cachedConstants.push(c);
  return c;
}

/**
 * Signed mass flow across a valve, kg/s. Positive means from `a` to `b`.
 *
 * Both directions matter: reverse flow from the pipe back into the cylinder during
 * overlap is exactly the mechanism a tuned exhaust exploits (or ruins), so it must
 * not be clipped to zero.
 */
export function valveMassFlow(
  area: number,
  pA: number,
  tA: number,
  pB: number,
  tB: number,
  gamma: number,
): number {
  if (area <= 0) return 0;
  if (pA > pB) return orificeMassFlow(area, VALVE_CD, pA, tA, pB, gamma);
  return -orificeMassFlow(area, VALVE_CD, pB, tB, pA, gamma);
}
