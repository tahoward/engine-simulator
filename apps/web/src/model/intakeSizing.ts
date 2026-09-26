/**
 * What an intake left at "auto" is sized to, for the panel to show.
 *
 * The same rules the simulation sizes it by (`plenum.rs` in `crates/engine-sim`): a plenum of 1.5
 * times the engine's swept volume, and a throttle bore that passes the engine's airflow at 7000 rpm
 * at 25 m/s.
 */

import { type EngineSpec, displacement } from './spec.js';

const PLENUM_VOLUME_RATIO = 1.5;
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
