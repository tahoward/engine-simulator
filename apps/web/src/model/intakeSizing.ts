/**
 * What an intake left at "auto" is sized to, for the panel to show.
 *
 * The same rules the simulation sizes it by (`plenum.rs` and `inlet.rs` in `crates/engine-sim`): a
 * plenum of 1.5 times the engine's swept volume, along most of the engine's length, a throttle bore that passes the engine's airflow at
 * 7000 rpm at 25 m/s, an airbox of four times the swept volume, and a snorkel 1.1 times the throttle's
 * bore.
 */

import { sharedHead } from './geometry.js';
import {
  type EngineSpec,
  type PipeSegment,
  crankPins,
  cylinderSpacing,
  cylinderZ,
  displacement,
  intakeRunnerOf,
  makeSegment,
  physicalBankCount,
} from './spec.js';

const PLENUM_VOLUME_RATIO = 1.5;
const AIRBOX_VOLUME_RATIO = 4;
const SNORKEL_BORE_RATIO = 1.1;
const THROTTLE_DUCT_LENGTH = 0.35;
const THROTTLE_DESIGN_VELOCITY = 25;
const THROTTLE_DESIGN_RPM = 7000;

/** The plenum's least height, its rounded edges, and the throttle body's wall, m. */
export const PLENUM_HEIGHT = 0.1;
export const PLENUM_ROUNDING = 0.012;
export const THROTTLE_WALL = 0.006;

function totalDisplacement(spec: EngineSpec): number {
  return displacement(spec) * Math.max(spec.cylinders, 1);
}

/**
 * How many plenums the runners draw from: two with `dualPlenum` on a V or a boxer whose banks have heads
 * of their own, each bank's runners from their own, side by side in one casting; otherwise one. The
 * simulation is told this (`solvedPlenum`), and splits dual plenums by bank (`plenum_count_of`).
 */
export function plenumCountOf(spec: EngineSpec): number {
  return spec.dualPlenum && physicalBankCount(spec) > 1 && !sharedHead(spec) ? 2 : 1;
}

/**
 * A plenum's size, m: along the engine, across and high at its front, and how much of its section it has
 * lost at its back, a fraction, narrowing evenly from none at the front. Dual plenums are one casting
 * this size, divided down its middle.
 */
export interface PlenumShape {
  length: number;
  width: number;
  height: number;
  taper: number;
}

/**
 * The plenum's size: the spec's, each that is 0 or less worked out, as the simulation works it out
 * (`plenum_shape_of`). Its length runs past every runner by its flared mouth, along most of the engine; its
 * height takes the throttle body's flange; its width holds `plenumVolume`, or one and a half times the
 * engine's displacement, at that length and height. A width or height given is no less than the flange,
 * nor a width less than a flange for each plenum side by side.
 */
export function plenumShapeOf(spec: EngineSpec): PlenumShape {
  const taper = Math.min(Math.max(spec.plenumTaper, 0), 0.8);
  let length = spec.plenumLength;
  if (!(length > 0)) {
    const spacing = cylinderSpacing(spec);
    const block = (crankPins(spec).length - 1) * spacing + Math.max(spacing, spec.bore * 1.3);
    const reach = Math.max(0, ...Array.from({ length: spec.cylinders }, (_, c) => Math.abs(cylinderZ(spec, c))));
    const last = reach + (1.5 * intakeRunnerOf(spec).diameter) / 2 + PLENUM_ROUNDING + 0.006;
    length = Math.max(block * 0.85, 2 * last);
  }
  // No smaller across or high than takes the throttle bodies' flanges on its front.
  const face = throttleFaceOf(spec);
  const height = spec.plenumHeight > 0 ? Math.max(spec.plenumHeight, face) : Math.max(PLENUM_HEIGHT, face);
  const target = spec.plenumVolume > 0 ? spec.plenumVolume : PLENUM_VOLUME_RATIO * totalDisplacement(spec);
  const width =
    spec.plenumWidth > 0
      ? Math.max(spec.plenumWidth, face * plenumCountOf(spec))
      : target / (height * length * (1 - taper / 2));
  return { length, width, height, taper };
}

/** How wide and high a plenum's front must be to take a throttle body's flange inside its rounded edges, m. */
export function throttleFaceOf(spec: EngineSpec): number {
  return throttleDiaOf(spec) + 2 * THROTTLE_WALL + 2 * PLENUM_ROUNDING + 0.01;
}

/** Plenum volume, m^3, of the plenum as `plenumShapeOf` sizes it: dual plenums' together. */
export function plenumVolumeOf(spec: EngineSpec): number {
  const s = plenumShapeOf(spec);
  return s.width * s.height * s.length * (1 - s.taper / 2);
}

/**
 * Each throttle body's bore, m: one on the front of each plenum. `spec.throttleDia` overrides; 0 or less
 * sizes them to pass the engine's air together.
 */
export function throttleDiaOf(spec: EngineSpec): number {
  if (spec.throttleDia > 0) return spec.throttleDia;
  const area = (totalDisplacement(spec) * (THROTTLE_DESIGN_RPM / 120)) / THROTTLE_DESIGN_VELOCITY;
  return Math.sqrt((4 * area) / (Math.PI * plenumCountOf(spec)));
}

/**
 * Airbox volume, m^3: dual plenums' two airboxes' together. `spec.airboxVolume` overrides; 0 or less means
 * "size it for me" (`inlet.rs`).
 */
export function airboxVolumeOf(spec: EngineSpec): number {
  if (spec.airboxVolume > 0) return spec.airboxVolume;
  return AIRBOX_VOLUME_RATIO * totalDisplacement(spec);
}

/** Each snorkel's bore, m. `spec.snorkelDia` overrides; 0 or less makes it a little wider than its throttle. */
export function snorkelDiaOf(spec: EngineSpec): number {
  if (spec.snorkelDia > 0) return spec.snorkelDia;
  return SNORKEL_BORE_RATIO * throttleDiaOf(spec);
}

/**
 * The inlet tract from the throttle out to the snorkel's mouth, as the simulation builds it (`inlet_segments`
 * in `inlet.rs`): a duct at the throttle's bore, the airbox, a round can about as long as it is wide, and the
 * snorkel. Dual plenums have one for each throttle body, alike, each airbox with half the volume.
 */
export function inletSegments(spec: EngineSpec): PipeSegment[] {
  const throttle = throttleDiaOf(spec);
  const volume = airboxVolumeOf(spec) / plenumCountOf(spec);
  const length = Math.min(Math.max(Math.cbrt(volume) * 1.5, 0.15), 0.6);
  const body = Math.max(Math.sqrt((4 * volume) / (Math.PI * length)), throttle * 1.5);
  return [
    makeSegment({ kind: 'pipe', length: THROTTLE_DUCT_LENGTH, dIn: throttle, dOut: throttle }),
    makeSegment({ kind: 'chamber', length, dIn: throttle, dOut: body }),
    makeSegment({ kind: 'pipe', length: Math.max(spec.snorkelLength, 0.02), dIn: snorkelDiaOf(spec), dOut: snorkelDiaOf(spec) }),
  ];
}
