/**
 * The presets on the dyno as the app runs them: their exhaust seated on the engine as it is drawn, their
 * plenum the size it is drawn, and the pull the Dyno section starts on auto. The simulation's own tests
 * run the exhaust as compiled, which the drawing can reshape: the LT2's headers, run lengthways, open
 * out to the collector's bore in their bends as drawn, and that alone moves its power by several percent.
 */

import { describe, expect, it } from 'vitest';

import { compileExhaust, graphFromJson, solverGraph } from '../src/model/exhaustGraph.js';
import { ENGINE_PRESETS, defaultConfig, fitDyno, presetEngine, type EngineConfig } from '../src/model/spec.js';
import { solvedPlenum } from '../src/scene/inletLayout.js';
import { configSources, exhaustPorts, seatGraph } from '../src/scene/soundSources.js';
import { Sim } from '../src/audio/worklet/sim.js';

/** `name`'s preset as the app loads it, running free. */
function appConfig(name: string): EngineConfig {
  const preset = ENGINE_PRESETS.find((p) => p.name === name)!;
  const cfg = defaultConfig();
  cfg.engine = { ...presetEngine(preset, cfg.engine), freeRunning: true };
  cfg.pipe = preset.pipe();
  if (preset.collector) cfg.collector = preset.collector();
  const graph =
    (preset.graph && graphFromJson(preset.graph())) || compileExhaust(cfg.engine, cfg.pipe, cfg.collector, preset.turbos ?? 0);
  seatGraph(graph, exhaustPorts(cfg.engine), cfg.engine);
  cfg.graph = graph;
  cfg.sources = configSources(cfg);
  cfg.graph = solverGraph(graph);
  cfg.engine = { ...cfg.engine, ...solvedPlenum(cfg.engine) };
  return cfg;
}

/** Peak power, hp, and peak torque, N·m, and the speeds of each, from an auto dyno pull, each cycle
 * averaged with three either side as the dyno sheet draws it. */
function pull(name: string): { hp: number; hpRpm: number; nm: number; nmRpm: number } {
  const cfg = appConfig(name);
  const sim = new Sim(48000, cfg);
  sim.render(24000);
  sim.startLaunch(fitDyno(cfg.engine));
  const points: [number, number][] = [];
  for (let i = 0; i < 3000; i++) {
    sim.render(960);
    const run = sim.snapshot().launch;
    if (!run || run.finished) break;
    for (let k = 0; k + 5 < run.points.length; k += 6) points.push([run.points[k]!, run.points[k + 1]!]);
  }
  const smoothed = points.map((_, i) => {
    const w = points.slice(Math.max(i - 3, 0), Math.min(i + 3, points.length - 1) + 1);
    return [w.reduce((s, p) => s + p[0], 0) / w.length, w.reduce((s, p) => s + p[1], 0) / w.length] as const;
  });
  const hp = (p: readonly [number, number]) => (p[0] * p[1] * 2 * Math.PI) / 60 / 745.7;
  const power = smoothed.reduce((b, p) => (hp(p) > hp(b) ? p : b));
  const torque = smoothed.reduce((b, p) => (p[1] > b[1] ? p : b));
  return { hp: hp(power), hpRpm: power[0], nm: torque[1], nmRpm: torque[0] };
}

describe('the dyno, as the app runs it', () => {
  it('has the LT2 make GM’s 495 hp at 6450 rpm and 470 lb·ft at 5150', () => {
    const r = pull('V8, Chevrolet LT2');
    expect(r.hp).toBeGreaterThan(490);
    expect(r.hp).toBeLessThan(500);
    expect(r.hpRpm).toBeGreaterThan(6300);
    expect(r.nm / 1.3558).toBeGreaterThan(465);
    expect(r.nm / 1.3558).toBeLessThan(475);
    expect(Math.abs(r.nmRpm - 5150)).toBeLessThan(150);
  }, 60_000);

  it('has the LT6 make about the real engine’s 670 hp', () => {
    const r = pull('V8, Chevrolet LT6');
    expect(r.hp).toBeGreaterThan(665);
    expect(r.hp).toBeLessThan(680);
    expect(r.hpRpm).toBeGreaterThan(8200);
  }, 60_000);
});
