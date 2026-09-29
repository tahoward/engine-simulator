/**
 * The launch's settings as a link carries them: they come back as they were, the car among them, and
 * anything unreadable is left on auto.
 */

import { describe, expect, it } from 'vitest';

import { autoLaunchSettings, launchSettingsJson, readLaunchSettings } from '../src/model/launchSettings.js';
import { ENGINE_PRESETS } from '../src/model/spec.js';

/** `settings` through a link and back. */
function throughALink(settings: ReturnType<typeof autoLaunchSettings>) {
  return readLaunchSettings(JSON.parse(JSON.stringify(launchSettingsJson(settings))));
}

describe('launch settings in a link', () => {
  const rs3 = ENGINE_PRESETS.find((p) => p.name === 'Inline five, Audi EA855 EVO')!.car!;

  it('keep a preset car, all-wheel drive and dual clutch included', () => {
    const read = throughALink(autoLaunchSettings(rs3));
    expect(read.car).toBe(rs3);
    expect(read).toEqual(autoLaunchSettings(rs3));
  });

  it('keep what the user has changed', () => {
    const settings = {
      ...autoLaunchSettings(rs3),
      launchRpm: 4200,
      shiftRpm: 6900,
      mass: 1400,
      shiftTime: 0.2,
      tyreGrip: 1.3,
      dualClutch: false,
      awd: false,
      tractionControl: false,
      ratios: [3, 2, 1.5, 1],
      finalDrive: 3.9,
      dynoFrom: 2500,
      dynoTo: 7000,
      sweepRate: 300,
    };
    expect(throughALink(settings)).toEqual(settings);
  });

  it('leave anything unreadable on auto', () => {
    expect(readLaunchSettings(undefined)).toEqual(autoLaunchSettings());
    expect(readLaunchSettings('launch')).toEqual(autoLaunchSettings());
    const read = readLaunchSettings({
      car: 'A car no preset has',
      mass: -5,
      shiftRpm: 'fast',
      awd: 1,
      ratios: [3, 'x'],
      finalDrive: Infinity,
      tractionControl: false,
    });
    expect(read).toEqual({ ...autoLaunchSettings(), tractionControl: false });
  });

  it('keep no more gears than a gearbox can have', () => {
    expect(readLaunchSettings({ ratios: [] }).ratios).toBeNull();
    expect(readLaunchSettings({ ratios: Array(9).fill(1) }).ratios).toBeNull();
  });
});
