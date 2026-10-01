import { describe, expect, it } from 'vitest';

import { presetConfig } from '../bench/presetConfig.js';
import { Sim } from '../src/audio/worklet/sim.js';
import { solverGraph, compileExhaust } from '../src/model/exhaustGraph.js';
import { ENGINE_PRESETS, defaultConfig, intakeRunnerOf } from '../src/model/spec.js';
import { inletSegments } from '../src/model/intakeSizing.js';
import { engineShell, exhaustPortOf, intakePortOf, sharedHead } from '../src/model/geometry.js';
import { inletLayout } from '../src/scene/inletLayout.js';
import { configSources } from '../src/scene/soundSources.js';

describe('where the engine makes its sound', () => {
  it('places every mouth the solver radiates from, on every preset', () => {
    for (const preset of ENGINE_PRESETS) {
      const cfg = presetConfig(preset);
      const graph = cfg.graph ?? compileExhaust(cfg.engine, cfg.pipe, cfg.collector, 0);
      const mouths = solverGraph(graph).ducts.filter((d) => d.to.kind === 'mouth').map((d) => d.id);
      const sources = configSources(cfg);
      expect(sources.mouths.map((m) => m.duct).sort(), preset.name).toEqual([...mouths].sort());
      for (const m of sources.mouths) expect(m.position.every(Number.isFinite), preset.name).toBe(true);
      expect(sources.intake && sources.engine, preset.name).toBeTruthy();
    }
  });

  it('puts a tailpipe at the end of the pipe that is drawn there', () => {
    // The default single's megaphone runs out from the side of the head.
    const sources = configSources(defaultConfig());
    expect(sources.mouths).toHaveLength(1);
    expect(sources.mouths[0]!.position[0]).toBeGreaterThan(0.5);
  });

  it('brings the Z06’s tailpipes together in the middle', () => {
    const lt6 = ENGINE_PRESETS.find((p) => p.name.includes('LT6'))!;
    const [a, b] = configSources(presetConfig(lt6)).mouths;
    expect(Math.abs(a!.position[0] - b!.position[0])).toBeLessThan(0.25);
    expect(Math.abs(a!.position[0] + b!.position[0])).toBeLessThan(0.02);
  });

  it('is heard from where the listener is: twice as far, about half as loud', () => {
    const cfg = defaultConfig();
    const mouth = configSources(cfg).mouths[0]!;
    const at = mouth.position;
    cfg.engine = { ...cfg.engine, groundReflection: 0, combustionVariability: 0 };
    cfg.sources = { mouths: [mouth], intake: at, engine: at, turbo: at };
    const rms = (distance: number) => {
      const sim = new Sim(48000, { ...cfg, listener: [at[0] + distance, at[1], at[2]] });
      const out = new Float32Array(48000);
      sim.renderInto(out);
      sim.renderInto(out);
      return Math.sqrt(out.reduce((s, v) => s + v * v, 0) / out.length);
    };
    expect(rms(1) / rms(2)).toBeGreaterThan(1.85);
    expect(rms(1) / rms(2)).toBeLessThan(2.15);
  });
});

describe('the inlet tract as drawn', () => {
  it('sits on the engine, as long as the solver’s tract, and the intake is heard from its mouth', () => {
    for (const preset of ENGINE_PRESETS) {
      const cfg = presetConfig(preset);
      const at = inletLayout(cfg.engine);
      const { intake } = configSources(cfg);
      expect(intake, preset.name).toEqual([at.mouth.x, at.mouth.y, at.mouth.z]);
      expect(at.tube.getLength(), preset.name).toBeCloseTo(at.segments[0]!.length, 2);
      expect(at.snorkel.getLength(), preset.name).toBeCloseTo(at.segments[2]!.length, 2);
      // The airbox over the engine rather than out in front of it, the snorkel's mouth ahead of it.
      const half = engineShell(cfg.engine).length / 2;
      expect(Math.abs(at.airbox.centre.z), preset.name).toBeLessThan(half);
      expect(at.mouth.z, preset.name).toBeLessThan(at.airbox.centre.z);
    }
  });

  it('keeps the two heads’ intake ports apart in the valley, down to a 45° V', () => {
    for (const name of ['LT6', '45°', '2GR']) {
      const spec = presetConfig(ENGINE_PRESETS.find((p) => p.name.includes(name))!).engine;
      const flange = (intakeRunnerOf(spec).diameter / 2) * 1.53;
      for (let v = 45; v <= 120; v += 5) {
        const s = { ...spec, vAngle: v };
        for (let b = 0; b < s.cylinders; b++) {
          const { position, direction } = intakePortOf(s, b);
          // The flange's edge nearest the middle, still on its own bank's side.
          const inner = Math.abs(position[0]) - flange * Math.abs(direction[1]);
          expect(inner, `${name} at ${v}°`).toBeGreaterThan(0.004);
        }
      }
    }
  });

  it('shares one head between the banks of a V too narrow for runners in its valley, as a VR engine does', () => {
    for (const name of ['LT6', '2GR', '45°']) {
      const spec = { ...presetConfig(ENGINE_PRESETS.find((p) => p.name.includes(name))!).engine, vAngle: 15 };
      expect(sharedHead(spec), name).toBe(true);
      for (let b = 0; b < spec.cylinders; b++) {
        // Every exhaust out of one side, every intake out of the other.
        expect(exhaustPortOf(spec, b).direction[0], name).toBeLessThan(0);
        expect(intakePortOf(spec, b).direction[0], name).toBeGreaterThan(0);
      }
      // And the intake beside it, as an inline engine's is.
      const at = inletLayout(spec);
      expect(at.plenum.centre.x, name).toBeGreaterThan(Math.max(...at.runners.map((r) => r.to.x)));
    }
    expect(sharedHead(presetConfig(ENGINE_PRESETS.find((p) => p.name.includes('45°'))!).engine)).toBe(false);
  });

  it('is the tract the simulation solves, and a turbocharged engine has none', () => {
    const snapshot = (name: string) => {
      const cfg = presetConfig(ENGINE_PRESETS.find((p) => p.name.includes(name))!);
      const sim = new Sim(48000, cfg);
      sim.renderInto(new Float32Array(4800));
      return { cfg, snap: sim.snapshot() };
    };
    const { cfg, snap } = snapshot('F20C');
    const length = inletSegments(cfg.engine).reduce((sum, s) => sum + s.length, 0);
    // One value a cell, the solver's cells a little under its 35 mm cell size.
    expect(snap.inletPressure.length).toBeGreaterThan(length / 0.04);
    expect(snap.inletPressure.length).toBeLessThan(length / 0.03 + 2);
    expect(snapshot('RB26').snap.inletPressure.length).toBe(0);
  });
});
