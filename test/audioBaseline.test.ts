/**
 * A fingerprint of the audio every preset produces, so a refactor can prove it changed nothing.
 *
 * The exhaust is a general duct graph in the middle of a carefully tuned solver, and the only safe
 * way to restructure it is against fixed output: a change that is meant to be a pure refactor is not
 * finished until every preset still matches its snapshot.
 *
 * A hash rather than the samples themselves, because the requirement is bit-identity and a hash
 * states that exactly in one line per preset. The summary figures alongside it are there so a
 * failure says something useful about *how* the sound moved — a changed hash alone tells you only
 * that it did.
 *
 * When the graph deliberately allows something new — unequal runners, a drawn tri-Y — that is a new
 * case with its own snapshot, never a reason to re-bless these.
 */

import { describe, expect, it } from 'vitest';

import { EngineSim } from '../src/audio/worklet/engineSim.js';
import { ENGINE_PRESETS, defaultConfig, type EngineSpec } from '../src/model/spec.js';

const FS = 48000;

/** FNV-1a over the raw bytes. Cheap, and sensitive to a single flipped mantissa bit. */
function fingerprint(samples: Float32Array): string {
  const bytes = new Uint8Array(samples.buffer, samples.byteOffset, samples.byteLength);
  let h = 0x811c9dc5;
  for (let i = 0; i < bytes.length; i++) {
    h ^= bytes[i]!;
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h.toString(16).padStart(8, '0');
}

function summarise(samples: Float32Array): string {
  let peak = 0;
  let sum = 0;
  let nonFinite = 0;
  for (let i = 0; i < samples.length; i++) {
    const v = samples[i]!;
    if (!Number.isFinite(v)) {
      nonFinite++;
      continue;
    }
    if (Math.abs(v) > peak) peak = Math.abs(v);
    sum += v * v;
  }
  const rms = Math.sqrt(sum / samples.length);
  return `rms ${rms.toExponential(6)} peak ${peak.toFixed(6)} nonFinite ${nonFinite}`;
}

describe('audio fingerprint per preset', () => {
  it.each(ENGINE_PRESETS.map((p) => p.name))('%s', (name) => {
    const preset = ENGINE_PRESETS.find((p) => p.name === name)!;
    const cfg = defaultConfig();
    // Fixed rpm and throttle: free-running would integrate its own state and make the fingerprint
    // depend on how long the warm-up happened to be.
    cfg.engine = {
      ...cfg.engine,
      ...preset.engine,
      rpm: 4200,
      throttle: 0.85,
      freeRunning: false,
    } as EngineSpec;
    cfg.pipe = preset.pipe();
    if (preset.collector) cfg.collector = preset.collector();

    const sim = new EngineSim(FS, cfg);
    // Warm the walls and let the ducts fill, so the fingerprint covers steady running rather than
    // the start-up transient, which is the least interesting second of any engine.
    sim.render(FS / 2);
    const audio = sim.render(FS);

    expect(`${fingerprint(audio)}  ${summarise(audio)}`).toMatchSnapshot();
  });
});
