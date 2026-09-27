/**
 * An engine as a file: exported, it comes back as it was, exhaust and all; anything else is refused; and an
 * exhaust that does not fit its engine is dropped for one compiled fresh.
 */

import { describe, expect, it } from 'vitest';

import { compileExhaust } from '../src/model/exhaustGraph.js';
import { engineFile, engineFileName, readEngineFile } from '../src/model/engineFile.js';
import { ENGINE_PRESETS, defaultConfig, presetEngine, type EngineConfig } from '../src/model/spec.js';

function presetConfig(name: string): EngineConfig {
  const preset = ENGINE_PRESETS.find((p) => p.name === name)!;
  const cfg = defaultConfig();
  cfg.engine = { ...presetEngine(preset, cfg.engine), freeRunning: true };
  cfg.pipe = preset.pipe();
  cfg.collector = preset.collector?.() ?? [];
  cfg.graph = compileExhaust(cfg.engine, cfg.pipe, cfg.collector, preset.turbos ?? 0);
  return cfg;
}

describe('an engine file', () => {
  it.each(['V8, Chevrolet LT2', 'Inline six, Nissan RB26DETT', 'Boxer four'])('comes back as it was exported: %s', (name) => {
    const cfg = presetConfig(name);
    const read = readEngineFile(engineFile(cfg), defaultConfig())!;
    expect(read).not.toBeNull();
    expect(read.graphDropped).toBe(false);
    expect(read.config.engine).toEqual(cfg.engine);
    expect(read.config.pipe).toEqual(cfg.pipe);
    expect(read.config.collector).toEqual(cfg.collector);
    expect(read.config.graph).toEqual(cfg.graph);
  });

  it('says what it is', () => {
    const file = JSON.parse(engineFile(presetConfig('V8, Chevrolet LT2')));
    expect(file.format).toBe('engine-simulator/engine');
    expect(file.version).toBe(1);
  });

  it.each([
    ['not JSON', 'not an engine at all'],
    ['JSON with no engine', JSON.stringify({ hello: 'world' })],
    ['something else by its own say', JSON.stringify({ format: 'something/else', engine: {} })],
    ['a number', '42'],
  ])('refuses %s', (_, text) => {
    expect(readEngineFile(text, defaultConfig())).toBeNull();
  });

  it('drops an exhaust that does not fit its engine, for one compiled from its pipe and collector', () => {
    const cfg = presetConfig('V8, Chevrolet LT2');
    const file = JSON.parse(engineFile(cfg));
    // Edited by hand to a six, with the V8's exhaust still in it.
    file.engine.cylinders = 6;
    const read = readEngineFile(JSON.stringify(file), defaultConfig())!;
    expect(read.graphDropped).toBe(true);
    expect(read.config.graph).toBeUndefined();
    expect(read.config.engine.cylinders).toBe(6);
  });

  it('is named for its layout', () => {
    expect(engineFileName(presetConfig('V8, Chevrolet LT2').engine)).toBe('engine-v8-crossplane');
    expect(engineFileName(presetConfig('Boxer four').engine)).toBe('engine-flat-4');
    expect(engineFileName(presetConfig('Inline four, Honda F20C').engine)).toBe('engine-inline-4');
    expect(engineFileName(presetConfig('Single, megaphone').engine)).toBe('engine-single');
  });
});
