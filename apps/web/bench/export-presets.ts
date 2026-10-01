/**
 * The engine and exhaust presets, as JSON for the Rust crate's tests: `npm run export:presets`.
 *
 * The presets are the web app's, in `src/model/spec.ts`, and the physics tests run the engines the app
 * ships. Writes `crates/engine-sim/tests/fixtures/presets.json`.
 */

import { writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

import {
  DEFAULT_ENGINE,
  ENGINE_PRESETS,
  PIPE_PRESETS,
  defaultCollector,
  defaultConfig,
  fittedExhaust,
  presetLaunch,
} from '../src/model/spec.js';
import { configSources } from '../src/scene/soundSources.js';
import { presetConfig } from './presetConfig.js';

// Run from `bench/dist`, four levels below the repository root.
const root = fileURLToPath(new URL('../../../../', import.meta.url));
const outPath = `${root}crates/engine-sim/tests/fixtures/presets.json`;

const engines = ENGINE_PRESETS.map((preset) => {
  const cfg = presetConfig(preset);
  // Where the app draws its sources, so the tests hear it from where it is drawn.
  cfg.sources = configSources(cfg);
  return {
    name: preset.name,
    engine: preset.engine,
    config: cfg,
    launch: presetLaunch(cfg.engine, (preset.turbos ?? 0) > 0, preset.car),
    fittedExhaust: fittedExhaust(cfg.engine),
  };
});

const base = defaultConfig();
base.sources = configSources(base);

const out = {
  defaultEngine: DEFAULT_ENGINE,
  defaultConfig: base,
  defaultCollector: defaultCollector(),
  enginePresets: engines,
  pipePresets: PIPE_PRESETS.map((p) => ({ name: p.name, description: p.description, segments: p.build() })),
};
writeFileSync(outPath, `${JSON.stringify(out, null, 1)}\n`);
console.log(`wrote ${outPath}`);
