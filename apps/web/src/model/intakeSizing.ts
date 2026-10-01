/**
 * What an intake left at "auto" is sized to, for the panel to show.
 *
 * The same rules the simulation sizes it by (`plenum.rs` and `inlet.rs` in `crates/engine-sim`): a
 * plenum of 1.5 times the engine's swept volume, a throttle bore that passes the engine's airflow at
 * 7000 rpm at 25 m/s, an airbox of four times the swept volume, and a snorkel 1.1 times the throttle's
 * bore.
 */

import { type EngineSpec, displacement } from './spec.js';

const PLENUM_VOLUME_RATIO = 1.5;
const AIRBOX_VOLUME_RATIO = 4;
const SNORKEL_BORE_RATIO = 1.1;
const THROTTLE_DESIGN_VELOCITY = 25;
const THROTTLE_DESIGN_RPM = 7000;

function totalDisplacement(spec: EngineSpec): number {
  return displacement(spec) * Math.max(spec.cylinders, 1);
}

/** Plenum volume, m^3. `spec.plenumVolume` overrides; 0 or less means "size it for me". */
export function plenumVolumeOf(spec: EngineSpec): number {
  if (spec.plenumVolume > 0) return spec.plenumVolume;
  return PLENUM_VOLUME_RATIO * totalDisplacement(spec);
}

/** Throttle bore, m. `spec.throttleDia` overrides; 0 or less means "size it for me". */
export function throttleDiaOf(spec: EngineSpec): number {
  if (spec.throttleDia > 0) return spec.throttleDia;
  const area = (totalDisplacement(spec) * (THROTTLE_DESIGN_RPM / 120)) / THROTTLE_DESIGN_VELOCITY;
  return Math.sqrt((4 * area) / Math.PI);
}

/** Airbox volume, m^3. `spec.airboxVolume` overrides; 0 or less means "size it for me" (`inlet.rs`). */
export function airboxVolumeOf(spec: EngineSpec): number {
  if (spec.airboxVolume > 0) return spec.airboxVolume;
  return AIRBOX_VOLUME_RATIO * totalDisplacement(spec);
}

/** The snorkel's bore, m. `spec.snorkelDia` overrides; 0 or less makes it a little wider than the throttle. */
export function snorkelDiaOf(spec: EngineSpec): number {
  if (spec.snorkelDia > 0) return spec.snorkelDia;
  return SNORKEL_BORE_RATIO * throttleDiaOf(spec);
}
