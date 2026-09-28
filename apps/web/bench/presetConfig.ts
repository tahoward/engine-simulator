/**
 * An engine preset's config as the app loads it: its engine, pipes and collector, and the exhaust
 * compiled with its turbos or drawn for it, where it has one.
 */

import { defaultConfig, type EngineConfig, type EnginePreset } from '../src/model/spec.js';
import { compileExhaust, graphFromJson } from '../src/model/exhaustGraph.js';

export function presetConfig(preset: EnginePreset): EngineConfig {
  const cfg = defaultConfig();
  cfg.engine = { ...cfg.engine, ...preset.engine };
  cfg.pipe = preset.pipe();
  if (preset.collector) cfg.collector = preset.collector();
  // A preset with turbos carries its exhaust as the app compiles it, since the turbos are part of it.
  if (preset.turbos) cfg.graph = compileExhaust(cfg.engine, cfg.pipe, cfg.collector, preset.turbos);
  // A preset with an exhaust drawn for it carries that.
  if (preset.graph) cfg.graph = graphFromJson(preset.graph()) ?? undefined;
  return cfg;
}
