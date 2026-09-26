/**
 * Valve lift, for drawing the valves.
 *
 * The same profile the simulation's valves follow (`valve.rs` in `crates/engine-sim`), so the valve
 * drawn is where the gas sees it: `sin(pi u)^2` across the event window, with zero lift and zero
 * velocity at both ends.
 */

/**
 * Position within an event window that may wrap past 720, as a 0..1 fraction. Returns -1 when `deg`
 * is outside the window.
 */
export function windowPhase(deg: number, open: number, close: number): number {
  let span = (close - open) % 720;
  if (span <= 0) span += 720;
  let rel = (deg - open) % 720;
  if (rel < 0) rel += 720;
  return rel <= span ? rel / span : -1;
}

/** Lift, m, at crank angle `deg`, for a valve open from `open` to `close` degrees. */
export function valveLift(deg: number, open: number, close: number, maxLift: number): number {
  const u = windowPhase(deg, open, close);
  if (u < 0) return 0;
  const s = Math.sin(Math.PI * u);
  return maxLift * s * s;
}
