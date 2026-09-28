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
  fitLaunch,
  fittedExhaust,
} from '../src/model/spec.js';
import { presetConfig } from './presetConfig.js';

// Run from `bench/dist`, four levels below the repository root.
const root = fileURLToPath(new URL('../../../../', import.meta.url));
const outPath = `${root}crates/engine-sim/tests/fixtures/presets.json`;

const engines = ENGINE_PRESETS.map((preset) => {
  const cfg = presetConfig(preset);
  return {
    name: preset.name,
    engine: preset.engine,
    config: cfg,
    launch: fitLaunch(cfg.engine, (preset.turbos ?? 0) > 0),
    fittedExhaust: fittedExhaust(cfg.engine),
  };
});

const out = {
  defaultEngine: DEFAULT_ENGINE,
  defaultConfig: defaultConfig(),
  defaultCollector: defaultCollector(),
  enginePresets: engines,
  pipePresets: PIPE_PRESETS.map((p) => ({ name: p.name, description: p.description, segments: p.build() })),
};
writeFileSync(outPath, `${JSON.stringify(out, null, 1)}\n`);
console.log(`wrote ${outPath}`);
