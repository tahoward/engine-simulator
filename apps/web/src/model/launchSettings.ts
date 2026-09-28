/**
 * The Launch section's settings, as a shared link carries them alongside the engine: the car the loaded
 * preset's engine comes from, and every setting the user has changed from it.
 *
 * The car is carried by name and looked up among the presets' cars as it is read, so a link to a real
 * car gets that car as the presets have it. Nothing else is trusted as it is read: a setting that is not
 * what it should be is left on auto.
 */

import { ENGINE_PRESETS, MAX_GEARS, MIN_GEARS, type Car } from './spec.js';

/** Each setting where the user has set it; `null` leaves it on the car's, or fitted to the engine. */
export interface LaunchSettings {
  /** The car the loaded preset's engine comes from, or `null` for one fitted to the engine. */
  car: Car | null;
  launchRpm: number | null;
  shiftRpm: number | null;
  mass: number | null;
  shiftTime: number | null;
  tyreGrip: number | null;
  dualClutch: boolean | null;
  awd: boolean | null;
  tractionControl: boolean;
  ratios: number[] | null;
  finalDrive: number | null;
}

/** Every setting on auto, with no car. */
export function autoLaunchSettings(car: Car | null = null): LaunchSettings {
  return {
    car,
    launchRpm: null,
    shiftRpm: null,
    mass: null,
    shiftTime: null,
    tyreGrip: null,
    dualClutch: null,
    awd: null,
    tractionControl: true,
    ratios: null,
    finalDrive: null,
  };
}

/** `settings` as a link writes them: the car by name. */
export function launchSettingsJson(settings: LaunchSettings): unknown {
  return { ...settings, car: settings.car?.name ?? null };
}

/** The settings a link's `raw` holds, each one that is missing or not what it should be left on auto. */
export function readLaunchSettings(raw: unknown): LaunchSettings {
  const out = autoLaunchSettings();
  if (!raw || typeof raw !== 'object') return out;
  const r = raw as Record<string, unknown>;
  const number = (v: unknown): number | null => (typeof v === 'number' && Number.isFinite(v) && v > 0 ? v : null);
  const flag = (v: unknown): boolean | null => (typeof v === 'boolean' ? v : null);
  out.car = ENGINE_PRESETS.find((p) => p.car && p.car.name === r.car)?.car ?? null;
  out.launchRpm = number(r.launchRpm);
  out.shiftRpm = number(r.shiftRpm);
  out.mass = number(r.mass);
  out.shiftTime = number(r.shiftTime);
  out.tyreGrip = number(r.tyreGrip);
  out.dualClutch = flag(r.dualClutch);
  out.awd = flag(r.awd);
  out.tractionControl = flag(r.tractionControl) ?? true;
  const ratios = Array.isArray(r.ratios) ? r.ratios.map(number) : null;
  out.ratios =
    ratios && ratios.length >= MIN_GEARS && ratios.length <= MAX_GEARS && ratios.every((v) => v !== null)
      ? (ratios as number[])
      : null;
  out.finalDrive = number(r.finalDrive);
  return out;
}
