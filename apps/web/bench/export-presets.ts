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
  fitDyno,
  fittedExhaust,
} from '../src/model/spec.js';
import { compileExhaust, graphFromJson } from '../src/model/exhaustGraph.js';

// Run from `bench/dist`, four levels below the repository root.
const root = fileURLToPath(new URL('../../../../', import.meta.url));
const outPath = `${root}crates/engine-sim/tests/fixtures/presets.json`;

const engines = ENGINE_PRESETS.map((preset) => {
  const cfg = defaultConfig();
  cfg.engine = { ...cfg.engine, ...preset.engine };
  cfg.pipe = preset.pipe();
  if (preset.collector) cfg.collector = preset.collector();
  // A preset with turbos carries its exhaust as the app compiles it, since the turbos are part of it.
  if (preset.turbos) cfg.graph = compileExhaust(cfg.engine, cfg.pipe, cfg.collector, preset.turbos);
  // A preset with an exhaust drawn for it carries that.
  if (preset.graph) cfg.graph = graphFromJson(preset.graph()) ?? undefined;
  return {
    name: preset.name,
    engine: preset.engine,
    config: cfg,
    dyno: fitDyno(cfg.engine, (preset.turbos ?? 0) > 0),
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
